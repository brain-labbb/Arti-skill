from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image, ImageStat


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp" / "scripts" / "visualize_artiverse_category_tsne.py"


class ArtiverseCategoryTsneTests(unittest.TestCase):
    def load_subject(self):
        spec = importlib.util.spec_from_file_location("visualize_artiverse_category_tsne", SCRIPT)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module

    def write_source_run(
        self,
        directory: Path,
        features: np.ndarray,
        rows: list[dict[str, str | int]],
        *,
        encoder_label: str,
    ) -> None:
        directory.mkdir(parents=True)
        np.save(directory / "model_features.npy", features.astype(np.float32), allow_pickle=False)
        with (directory / "model_tsne_coordinates.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "tsne_x",
                    "tsne_y",
                    "category",
                    "source",
                    "model_id",
                    "view_count",
                ],
            )
            writer.writeheader()
            for index, row in enumerate(rows):
                writer.writerow({"tsne_x": index, "tsne_y": -index, **row})
        manifest = {
            "schema_version": 1,
            "dataset": {
                "model_count": len(rows),
                "category_count": len({str(row["category"]) for row in rows}),
            },
            "feature_manifest": {
                "extraction": {
                    "encoder_label": encoder_label,
                    "model_type": (
                        "dinov2" if encoder_label == "DINOv2" else "clip"
                    ),
                }
            },
            "visualization": {
                "artifacts": {
                    "model_features": "model_features.npy",
                    "model_coordinates": "model_tsne_coordinates.csv",
                }
            },
        }
        (directory / "run_manifest.json").write_text(
            json.dumps(manifest), encoding="utf-8"
        )

    def test_category_aggregation_normalizes_models_before_equal_weight_mean(self) -> None:
        subject = self.load_subject()
        records = [
            subject.ModelRecord("chair", "a", "m1", 16),
            subject.ModelRecord("chair", "b", "m2", 16),
            subject.ModelRecord("lamp", "a", "m3", 16),
            subject.ModelRecord("table", "a", "m4", 16),
        ]
        model_features = np.asarray(
            [[2.0, 0.0], [0.0, 3.0], [4.0, 0.0], [0.0, 5.0]],
            dtype=np.float32,
        )

        features, categories = subject.aggregate_category_features(model_features, records)

        np.testing.assert_allclose(
            features,
            np.asarray(
                [
                    [2**-0.5, 2**-0.5],
                    [1.0, 0.0],
                    [0.0, 1.0],
                ],
                dtype=np.float32,
            ),
            rtol=1e-6,
            atol=1e-6,
        )
        self.assertEqual([record.category for record in categories], ["chair", "lamp", "table"])
        self.assertEqual(categories[0].model_count, 2)
        self.assertEqual(categories[0].source_counts, {"a": 1, "b": 1})

    def test_encoder_alignment_reorders_an_exact_shared_roster(self) -> None:
        subject = self.load_subject()
        first_records = (
            subject.ModelRecord("table", "b", "m2", 16),
            subject.ModelRecord("chair", "a", "m1", 16),
        )
        second_records = tuple(reversed(first_records))
        first = subject.EncoderRun(
            "first",
            "First",
            Path("first"),
            np.asarray([[2.0, 0.0], [1.0, 0.0]], dtype=np.float32),
            first_records,
            {},
        )
        second = subject.EncoderRun(
            "second",
            "Second",
            Path("second"),
            np.asarray([[10.0, 0.0], [20.0, 0.0]], dtype=np.float32),
            second_records,
            {},
        )

        aligned = subject.align_encoder_runs([first, second])

        expected_identities = [("chair", "a", "m1"), ("table", "b", "m2")]
        self.assertEqual(
            [record.identity for record in aligned[0].records], expected_identities
        )
        self.assertEqual(
            [record.identity for record in aligned[1].records], expected_identities
        )
        np.testing.assert_array_equal(aligned[0].model_features[:, 0], [1.0, 2.0])
        np.testing.assert_array_equal(aligned[1].model_features[:, 0], [10.0, 20.0])

    def test_encoder_alignment_rejects_different_rosters(self) -> None:
        subject = self.load_subject()
        first = subject.EncoderRun(
            "first",
            "First",
            Path("first"),
            np.ones((1, 2), dtype=np.float32),
            (subject.ModelRecord("chair", "a", "m1", 16),),
            {},
        )
        second = subject.EncoderRun(
            "second",
            "Second",
            Path("second"),
            np.ones((1, 2), dtype=np.float32),
            (subject.ModelRecord("table", "a", "m2", 16),),
            {},
        )

        with self.assertRaisesRegex(ValueError, "rosters differ"):
            subject.align_encoder_runs([first, second])

    def test_loader_rejects_an_encoder_type_mismatch(self) -> None:
        subject = self.load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "source"
            rows = [
                {
                    "category": category,
                    "source": "source_a",
                    "model_id": f"m{index}",
                    "view_count": 16,
                }
                for index, category in enumerate(("chair", "lamp", "table"))
            ]
            self.write_source_run(
                source,
                np.ones((3, 4), dtype=np.float32),
                rows,
                encoder_label="CLIP ViT-B/32",
            )

            with self.assertRaisesRegex(ValueError, "model_type does not match dinov2"):
                subject.load_encoder_run(source, key="dinov2")

    def test_run_rejects_non_16_view_source_metadata(self) -> None:
        subject = self.load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            rows = [
                {
                    "category": category,
                    "source": "source_a",
                    "model_id": f"m{index}",
                    "view_count": 8,
                }
                for index, category in enumerate(("chair", "lamp", "table"))
            ]
            self.write_source_run(
                root / "dinov2",
                np.ones((3, 4), dtype=np.float32),
                rows,
                encoder_label="DINOv2",
            )
            self.write_source_run(
                root / "clip",
                np.ones((3, 3), dtype=np.float32),
                rows,
                encoder_label="CLIP ViT-B/32",
            )

            with self.assertRaisesRegex(ValueError, "expected exactly 16 views"):
                subject.run(
                    dinov2_dir=root / "dinov2",
                    clip_dir=root / "clip",
                    output_dir=root / "output",
                    requested_perplexity=2.0,
                    random_state=1,
                    max_iter=250,
                    n_jobs=1,
                    plot_dpi=40,
                )

    def test_run_writes_two_category_embeddings_and_a_comparison_plot(self) -> None:
        subject = self.load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dinov2 = root / "dinov2"
            clip = root / "clip"
            output = root / "output"
            rows = []
            for category_index, category in enumerate(("chair", "lamp", "table", "window")):
                for model_index in range(2):
                    rows.append(
                        {
                            "category": category,
                            "source": f"source_{model_index}",
                            "model_id": f"m{category_index}_{model_index}",
                            "view_count": 16,
                        }
                    )
            generator = np.random.default_rng(9)
            self.write_source_run(
                dinov2,
                generator.normal(size=(8, 7)),
                rows,
                encoder_label="DINOv2",
            )
            self.write_source_run(
                clip,
                generator.normal(size=(8, 5)),
                list(reversed(rows)),
                encoder_label="CLIP ViT-B/32",
            )

            result = subject.run(
                dinov2_dir=dinov2,
                clip_dir=clip,
                output_dir=output,
                requested_perplexity=3.0,
                random_state=17,
                max_iter=250,
                n_jobs=1,
                plot_dpi=60,
            )

            expected = {
                "category_index.csv",
                "run_manifest.json",
                "final_audit.json",
                "tsne_category_comparison.png",
            }
            self.assertTrue(expected.issubset({path.name for path in output.iterdir()}))
            for key, expected_dimension in (("dinov2", 7), ("clip", 5)):
                encoder_dir = output / key
                self.assertEqual(
                    {path.name for path in encoder_dir.iterdir()},
                    {"category_features.npy", "tsne_coordinates.csv", "tsne_by_category.png"},
                )
                features = np.load(encoder_dir / "category_features.npy", allow_pickle=False)
                self.assertEqual(features.shape, (4, expected_dimension))
                np.testing.assert_allclose(
                    np.linalg.norm(features, axis=1), 1.0, rtol=2e-5, atol=2e-5
                )
                with (encoder_dir / "tsne_coordinates.csv").open(
                    "r", encoding="utf-8", newline=""
                ) as stream:
                    self.assertEqual(len(list(csv.DictReader(stream))), 4)
            with Image.open(output / "tsne_category_comparison.png") as image:
                self.assertGreaterEqual(image.width, 1_000)
                self.assertGreater(ImageStat.Stat(image.convert("L")).stddev[0], 1.0)
            self.assertTrue(result["audit"]["pass"])
            self.assertEqual(result["manifest"]["dataset"]["category_count"], 4)
            self.assertFalse(
                result["manifest"]["protocol"]["source_2d_coordinates_used_for_aggregation"]
            )


if __name__ == "__main__":
    unittest.main()
