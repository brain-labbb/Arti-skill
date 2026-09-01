from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp" / "scripts" / "compare_pva_artiverse_uniform.py"


def load_subject():
    name = "compare_pva_artiverse_uniform_test_subject"
    spec = importlib.util.spec_from_file_location(name, SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def normalize(array: np.ndarray) -> np.ndarray:
    array = np.asarray(array, dtype=np.float32)
    return array / np.linalg.norm(array, axis=1, keepdims=True)


class UniformComparisonTests(unittest.TestCase):
    def setUp(self) -> None:
        self.subject = load_subject()

    def write_pva(self, root: Path, *, count: int = 4) -> Path:
        pva = root / "pva"
        pva.mkdir()
        roster_rows = [
            {
                "generator_index": f"G{index:04d}",
                "generator_name": f"generator_{index}",
                "source_type": "picture_backed",
                "picture_category": "objects",
            }
            for index in range(1, count + 1)
        ]
        with (pva / "generator_roster_resolved.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(roster_rows[0]))
            writer.writeheader()
            writer.writerows(roster_rows)

        studio = {"camera": [1.25, -1.35, 0.85], "background": [0.8, 0.84, 0.9]}
        pva_renderer = root / "pva_renderer.py"
        pva_renderer.write_text("# frozen shared renderer\n", encoding="utf-8")
        render_config = {
            "schema_version": 2,
            "render_contract": "pva531_uniform_studio_v1",
            "generator_count": count,
            "resolution": 64,
            "samples": 1,
            "studio": studio,
            "blender_version": "Blender test",
            "renderer": str(pva_renderer),
            "renderer_sha256": hashlib.sha256(pva_renderer.read_bytes()).hexdigest(),
        }
        render_config_path = root / "pva_render_config.json"
        render_config_path.write_text(json.dumps(render_config), encoding="utf-8")
        run_manifest = {
            "schema_version": 1,
            "dataset": {
                "render_contract": "pva531_uniform_studio_v1",
                "input_mode": "uniform_blender_seed_0000",
                "generator_count": count,
                "render_config": str(render_config_path),
                "render_config_sha256": hashlib.sha256(render_config_path.read_bytes()).hexdigest(),
            },
        }
        (pva / "run_manifest.json").write_text(json.dumps(run_manifest), encoding="utf-8")

        generator = np.random.default_rng(31)
        for encoder, dimension in (("dinov2", 7), ("clip", 5)):
            encoder_dir = pva / encoder
            encoder_dir.mkdir()
            model_dir = root / f"{encoder}_snapshot"
            model_dir.mkdir()
            (model_dir / "weights.bin").write_bytes((encoder * 3).encode("ascii"))
            fingerprint = self.subject._pva_helper()._model_fingerprint(model_dir)
            features = normalize(generator.normal(size=(count, dimension)))
            np.save(encoder_dir / "class_features.npy", features, allow_pickle=False)
            feature_manifest = {
                "schema_version": 2,
                "model_path": str(model_dir),
                "model_fingerprint": fingerprint,
                "raw_image_count": count,
                "preprocessing": {
                    "image_processor_use_fast": False,
                    "alpha_composite_background_rgb": [255, 255, 255],
                },
                "extraction": {
                    "model_type": encoder,
                    "encoder_label": "DINOv2" if encoder == "dinov2" else "CLIP ViT-B/32",
                    "feature_dim": dimension,
                },
            }
            (encoder_dir / "feature_manifest.json").write_text(
                json.dumps(feature_manifest), encoding="utf-8"
            )
        return pva

    def write_artiverse(
        self,
        root: Path,
        *,
        pva_render_config: dict,
        categories: tuple[str, ...] = ("chair", "lamp", "table"),
        one_shot_only: bool = False,
    ) -> Path:
        render_root = root / "artiverse_renders"
        render_root.mkdir()
        data_root = root / "artiverse_data"
        data_root.mkdir()
        official_roots = [
            f"data/{category}/source_{model_index}/{category}_{model_index}"
            for category in categories
            for model_index in range(2)
        ]
        dataset_manifest = {
            "format": "artiverse-data-tar-gz-chunks-v1",
            "model_count": len(official_roots),
            "chunks": [
                {
                    "model_count": len(official_roots),
                    "roots": official_roots,
                }
            ],
        }
        dataset_manifest_path = root / "artiverse_manifest.json"
        dataset_manifest_path.write_text(json.dumps(dataset_manifest), encoding="utf-8")
        winners = {
            category: min(
                (value for value in official_roots if value.split("/")[1] == category),
                key=lambda value: (hashlib.sha256(value.encode()).hexdigest(), value),
            )
            for category in categories
        }
        manifest_rows = []
        one_shot_rows = []
        for ordinal, manifest_root in enumerate(official_roots, start=1):
            _, category, source, model_id = manifest_root.split("/")
            selected = manifest_root == winners[category]
            if one_shot_only and not selected:
                continue
            model_dir = data_root / category / source / model_id
            model_dir.mkdir(parents=True)
            glb_path = model_dir / f"{model_id}.segmented.glb"
            glb_path.write_bytes(f"glb:{manifest_root}".encode())
            output_path = render_root / category / source / model_id / "imgs" / "000.png"
            output_path.parent.mkdir(parents=True)
            Image.new("RGB", (64, 64), (ordinal * 20 % 255, 50, 90)).save(output_path)
            row = {
                "ordinal": ordinal,
                "category": category,
                "source": source,
                "model_id": model_id,
                "manifest_root": manifest_root,
                "identity_sha256": hashlib.sha256(manifest_root.encode()).hexdigest(),
                "category_one_shot": str(selected).lower(),
                "glb_path": str(glb_path),
                "glb_bytes": glb_path.stat().st_size,
                "glb_sha256": hashlib.sha256(glb_path.read_bytes()).hexdigest(),
                "output_path": str(output_path),
                "status": "rendered",
                "png_bytes": output_path.stat().st_size,
                "png_sha256": hashlib.sha256(output_path.read_bytes()).hexdigest(),
            }
            manifest_rows.append(row)
            if selected:
                one_shot_rows.append(
                    {
                        "category": category,
                        "source": source,
                        "model_id": model_id,
                        "manifest_root": manifest_root,
                    }
                )
        with (render_root / "render_manifest.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(manifest_rows[0]))
            writer.writeheader()
            writer.writerows(manifest_rows)
        with (render_root / "category_one_shot_roster.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(one_shot_rows[0]))
            writer.writeheader()
            writer.writerows(sorted(one_shot_rows, key=lambda row: row["category"]))
        config = {
            "schema_version": 1,
            "render_contract": "artiverse_uniform_studio_v1",
            "model_count": len(official_roots),
            "category_count": len(categories),
            "dataset_manifest": str(dataset_manifest_path),
            "dataset_manifest_sha256": hashlib.sha256(dataset_manifest_path.read_bytes()).hexdigest(),
            "data_root": str(data_root),
            "input_receipt": {"sha256": "0" * 64},
            "pose_policy": "canonical transforms embedded in segmented.glb",
            "material_policy": "native glTF materials and textures",
            "resolution": pva_render_config["resolution"],
            "samples": pva_render_config["samples"],
            "studio": pva_render_config["studio"],
            "blender_version": pva_render_config["blender_version"],
        }
        art_renderer = root / "art_renderer.py"
        art_renderer.write_text("# Artiverse adapter\n", encoding="utf-8")
        config.update(
            {
                "renderer": str(art_renderer),
                "renderer_sha256": hashlib.sha256(art_renderer.read_bytes()).hexdigest(),
                "shared_renderer": pva_render_config["renderer"],
                "shared_renderer_sha256": pva_render_config["renderer_sha256"],
            }
        )
        (render_root / "render_config.json").write_text(json.dumps(config), encoding="utf-8")
        return render_root

    def test_loaders_pin_snapshot_studio_and_official_order(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            pva_root = self.write_pva(root)
            pva = self.subject.load_pva_frozen(pva_root, strict_counts=False)
            render_root = self.write_artiverse(root, pva_render_config=pva.render_config)
            artiverse = self.subject.load_artiverse_uniform(
                render_root, pva_render_config=pva.render_config, strict_counts=False
            )
            self.assertEqual(len(artiverse.records), 6)
            self.assertEqual(artiverse.categories, ("chair", "lamp", "table"))
            self.assertEqual(sum(record.category_one_shot for record in artiverse.records), 3)

            manifest_path = render_root / "render_manifest.csv"
            with manifest_path.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            rows[0], rows[1] = rows[1], rows[0]
            with manifest_path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            with self.assertRaisesRegex(ValueError, "ordered roster mismatch"):
                self.subject.load_artiverse_uniform(
                    render_root, pva_render_config=pva.render_config, strict_counts=False
                )

    def test_one_shot_loader_uses_full_official_universe_without_touching_unselected_assets(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            pva = self.subject.load_pva_frozen(
                self.write_pva(root), strict_counts=False
            )
            render_root = self.write_artiverse(
                root, pva_render_config=pva.render_config, one_shot_only=True
            )
            artiverse = self.subject.load_artiverse_uniform(
                render_root,
                pva_render_config=pva.render_config,
                strict_counts=False,
                one_shot_only=True,
            )

            self.assertTrue(artiverse.one_shot_only)
            self.assertEqual(artiverse.universe_model_count, 6)
            self.assertEqual(len(artiverse.records), 3)
            self.assertEqual(artiverse.categories, ("chair", "lamp", "table"))
            self.assertTrue(all(record.category_one_shot for record in artiverse.records))
            self.assertEqual(
                artiverse.universe_source_counts,
                {
                    category: {"source_0": 1, "source_1": 1}
                    for category in artiverse.categories
                },
            )
            features = np.eye(3, dtype=np.float32)
            full, one_shot, categories = self.subject.aggregate_artiverse_features(
                features, artiverse
            )
            np.testing.assert_array_equal(full, one_shot)
            self.assertEqual([record.model_count for record in categories], [2, 2, 2])

            with self.assertRaisesRegex(ValueError, "official roster counts differ"):
                self.subject.load_artiverse_uniform(
                    render_root,
                    pva_render_config=pva.render_config,
                    strict_counts=False,
                    one_shot_only=False,
                )

            roster_path = render_root / "category_one_shot_roster.csv"
            with roster_path.open("r", encoding="utf-8", newline="") as stream:
                roster_rows = list(csv.DictReader(stream))
            roster_rows[0]["manifest_root"] = "data/chair/source_0/chair_0"
            with roster_path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(roster_rows[0]))
                writer.writeheader()
                writer.writerows(roster_rows)
            with self.assertRaisesRegex(ValueError, "official hash selection"):
                self.subject.load_artiverse_uniform(
                    render_root,
                    pva_render_config=pva.render_config,
                    strict_counts=False,
                    one_shot_only=True,
                )

    def test_compare_run_requests_one_shot_loader(self) -> None:
        args = argparse.Namespace(
            batch_size=1,
            num_workers=1,
            plot_dpi=100,
            neighbor_fraction=0.2,
            output_dir=Path("output"),
            pva_dir=Path("pva"),
            artiverse_render_root=Path("artiverse"),
            artiverse_one_shot_only=True,
            allow_count_drift=True,
            skip_glb_hash_verification=False,
        )
        pva = mock.Mock()
        pva.render_config = {}
        with (
            mock.patch.object(self.subject, "load_pva_frozen", return_value=pva),
            mock.patch.object(
                self.subject,
                "load_artiverse_uniform",
                side_effect=RuntimeError("one-shot loader sentinel"),
            ) as loader,
        ):
            with self.assertRaisesRegex(RuntimeError, "one-shot loader sentinel"):
                self.subject.run(args)
        self.assertTrue(loader.call_args.kwargs["one_shot_only"])

    def test_pva_loader_rejects_changed_model_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            pva_root = self.write_pva(root)
            (root / "dinov2_snapshot" / "weights.bin").write_bytes(b"changed")
            with self.assertRaisesRegex(ValueError, "snapshot fingerprint mismatch"):
                self.subject.load_pva_frozen(pva_root, strict_counts=False)

    def test_artiverse_loader_rejects_a_different_shared_renderer(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            pva = self.subject.load_pva_frozen(
                self.write_pva(root), strict_counts=False
            )
            render_root = self.write_artiverse(root, pva_render_config=pva.render_config)
            config_path = render_root / "render_config.json"
            config = json.loads(config_path.read_text(encoding="utf-8"))
            replacement = root / "different_shared.py"
            replacement.write_text("# different\n", encoding="utf-8")
            config["shared_renderer"] = str(replacement)
            config["shared_renderer_sha256"] = hashlib.sha256(
                replacement.read_bytes()
            ).hexdigest()
            config_path.write_text(json.dumps(config), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "did not use the frozen PV-A"):
                self.subject.load_artiverse_uniform(
                    render_root, pva_render_config=pva.render_config, strict_counts=False
                )

    def test_output_contract_rejects_unknown_or_different_runs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "output"
            output.mkdir()
            (output / "stale.txt").write_text("stale", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "no recognized run contract"):
                self.subject.validate_output_contract(output, {"schema_version": 1})
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "output"
            output.mkdir()
            (output / "run_contract.json").write_text(
                json.dumps({"schema_version": 1, "seed": 1}), encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "different frozen input"):
                self.subject.validate_output_contract(
                    output, {"schema_version": 1, "seed": 2}
                )

    def test_aggregation_uses_all_models_and_frozen_one_shot(self) -> None:
        records = (
            self.subject.ArtiverseModel(1, "chair", "a", "m1", "data/chair/a/m1", Path("1"), Path("1"), "0" * 64, 1, False),
            self.subject.ArtiverseModel(2, "chair", "b", "m2", "data/chair/b/m2", Path("2"), Path("2"), "1" * 64, 1, True),
            self.subject.ArtiverseModel(3, "lamp", "a", "m3", "data/lamp/a/m3", Path("3"), Path("3"), "2" * 64, 1, True),
        )
        bundle = self.subject.ArtiverseRenderBundle(
            root=Path("root"),
            records=records,
            config={},
            categories=("chair", "lamp"),
            universe_model_count=3,
            universe_source_counts={"chair": {"a": 1, "b": 1}, "lamp": {"a": 1}},
            one_shot_only=False,
            render_manifest_path=Path("manifest"),
            dataset_manifest_path=Path("dataset"),
            receipts={},
        )
        features = np.asarray([[2.0, 0.0], [0.0, 3.0], [4.0, 0.0]], dtype=np.float32)
        full, one_shot, categories = self.subject.aggregate_artiverse_features(features, bundle)
        np.testing.assert_allclose(full[0], [2**-0.5, 2**-0.5], atol=1e-6)
        np.testing.assert_allclose(one_shot[0], [0.0, 1.0], atol=1e-6)
        self.assertEqual(categories[0].source_counts, {"a": 1, "b": 1})
        self.assertEqual(categories[0].one_shot_model_id, "m2")

    def test_high_dimensional_metrics_are_chance_adjusted_and_finite(self) -> None:
        generator = np.random.default_rng(4)
        dino = normalize(generator.normal(size=(7, 6)))
        clip = dino.copy()
        agreement = self.subject.encoder_agreement_metrics(
            dino, clip, neighbor_fraction=0.2
        )
        self.assertAlmostEqual(agreement["pairwise_cosine_distance_spearman"], 1.0)
        self.assertAlmostEqual(agreement["neighbor_agreement"]["raw_overlap"], 1.0)
        self.assertAlmostEqual(
            agreement["neighbor_agreement"]["chance_adjusted_overlap"], 1.0
        )
        cross = self.subject.cross_dataset_metrics(
            dino[:4], dino[4:], neighbor_fraction=0.2
        )
        self.assertTrue(self.subject._finite_json(cross))
        self.assertIn("pva_to_artiverse", cross["nearest_other_source_cosine_distance"])

    def test_small_end_to_end_one_shot_run_uses_only_selected_images(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            pva_root = self.write_pva(root)
            pva = self.subject.load_pva_frozen(pva_root, strict_counts=False)
            render_root = self.write_artiverse(
                root, pva_render_config=pva.render_config, one_shot_only=True
            )
            output = root / "output"
            generator = np.random.default_rng(44)

            def fake_extract(paths, *, model_path, **_kwargs):
                encoder = "dinov2" if "dinov2" in model_path.name else "clip"
                dimension = 7 if encoder == "dinov2" else 5
                matrix = normalize(generator.normal(size=(len(paths), dimension)))
                return matrix, {
                    "model_type": encoder,
                    "encoder_label": "DINOv2" if encoder == "dinov2" else "CLIP ViT-B/32",
                    "feature_dim": dimension,
                }

            def fake_tsne(features, **_kwargs):
                values = np.asarray(features, dtype=np.float32)
                if values.shape[1] == 1:
                    coordinates = np.column_stack([values[:, 0], np.arange(len(values))])
                else:
                    coordinates = values[:, :2]
                return coordinates.astype(np.float32), {"sample_count": len(values), "stub": True}

            args = argparse.Namespace(
                pva_dir=pva_root,
                artiverse_render_root=render_root,
                artiverse_one_shot_only=True,
                output_dir=output,
                batch_size=2,
                device="cpu",
                num_workers=1,
                use_amp=False,
                perplexity=2.0,
                seed=1,
                tsne_max_iter=250,
                tsne_jobs=1,
                tsne_verbose=0,
                plot_dpi=100,
                neighbor_fraction=0.2,
                force_extract=True,
                skip_glb_hash_verification=False,
                allow_count_drift=True,
            )
            helper = self.subject._pva_helper()
            with mock.patch.object(helper, "extract_image_features", side_effect=fake_extract), mock.patch.object(
                self.subject, "_compute_tsne", side_effect=fake_tsne
            ):
                result = self.subject.run(args)

            self.assertTrue(result["audit"]["pass"])
            for encoder in ("dinov2", "clip"):
                encoder_dir = output / encoder
                self.assertEqual(
                    np.load(encoder_dir / "artiverse_one_shot_features.npy").shape[0], 3
                )
                self.assertFalse((encoder_dir / "artiverse_model_features.npy").exists())
                self.assertFalse((encoder_dir / "artiverse_category_features.npy").exists())
                self.assertFalse((encoder_dir / "joint_full_tsne_coordinates.csv").exists())
                self.assertTrue((encoder_dir / "joint_one_shot_tsne_coordinates.csv").is_file())
                with (encoder_dir / "joint_one_shot_tsne_coordinates.csv").open(
                    "r", encoding="utf-8", newline=""
                ) as stream:
                    self.assertEqual(len(list(csv.DictReader(stream))), 7)
            self.assertTrue((output / "high_dimensional_metrics.json").is_file())
            self.assertTrue((output / "joint_source_comparison.png").is_file())
            metrics = json.loads(
                (output / "high_dimensional_metrics.json").read_text(encoding="utf-8")
            )
            self.assertEqual(metrics["artiverse_mode"], "one_shot_only")
            self.assertNotIn("artiverse_full", metrics["encoder_agreement"])


if __name__ == "__main__":
    unittest.main()
