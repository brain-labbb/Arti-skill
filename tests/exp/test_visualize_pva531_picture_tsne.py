from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp" / "scripts" / "visualize_pva531_picture_tsne.py"


def load_subject():
    spec = importlib.util.spec_from_file_location("visualize_pva531_picture_tsne", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Pva531PictureTsneTests(unittest.TestCase):
    def test_uniform_render_discovery_requires_audited_one_to_one_images(self) -> None:
        subject = load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            render_root = root / "renders"
            render_root.mkdir()
            rows = [
                {
                    "generator_index": "G0001",
                    "generator_name": "picture_one",
                    "source_type": "picture_backed",
                    "picture_category": "Tools",
                    "picture_label": "One",
                    "picture_source_path": "unused",
                },
                {
                    "generator_index": "G0002",
                    "generator_name": "builtin_one",
                    "source_type": "articraft_builtin_dataset_no_picture",
                    "picture_category": "",
                    "picture_label": "",
                    "picture_source_path": "",
                },
            ]
            index = root / "index.csv"
            with index.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            manifest_rows = []
            for row in rows:
                image_path = render_root / (
                    f"{row['generator_index']}__{row['generator_name']}__seed_0000.png"
                )
                Image.new("RGBA", (8, 8), (10, 20, 30, 255)).save(image_path)
                manifest_rows.append(
                    {
                        "generator_index": row["generator_index"],
                        "generator_name": row["generator_name"],
                        "status": "rendered",
                        "output_path": str(image_path),
                        "png_bytes": image_path.stat().st_size,
                        "png_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
                    }
                )
            (render_root / "render_config.json").write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "render_contract": "pva531_uniform_studio_v1",
                        "generator_count": 2,
                        "index_csv_sha256": hashlib.sha256(index.read_bytes()).hexdigest(),
                        "input_receipt": {
                            "asset_and_dependency_sha256": "0" * 64,
                            "file_count": 1,
                            "total_bytes": 1,
                        },
                        "resolution": 8,
                        "samples": 1,
                    }
                ),
                encoding="utf-8",
            )
            with (render_root / "render_manifest.csv").open(
                "w", encoding="utf-8", newline=""
            ) as stream:
                writer = csv.DictWriter(stream, fieldnames=list(manifest_rows[0]))
                writer.writeheader()
                writer.writerows(manifest_rows)

            bundle = subject.discover_uniform_render_records(
                index, render_root=render_root, strict_counts=False
            )

            self.assertEqual(len(bundle.records), 2)
            self.assertEqual(len(bundle.raw_image_paths), 2)
            self.assertEqual(bundle.summary["input_mode"], "uniform_blender_seed_0000")
            self.assertEqual(bundle.summary["uniform_image_dimensions"], [8, 8])
            self.assertEqual(bundle.records[1].source_type, "builtin_no_picture")

            Image.new("RGBA", (8, 8), (200, 10, 20, 255)).save(bundle.raw_image_paths[0])
            with self.assertRaisesRegex(ValueError, "manifest/PNG (size|SHA) mismatch"):
                subject.discover_uniform_render_records(
                    index, render_root=render_root, strict_counts=False
                )

    def test_roster_discovery_keeps_generator_alias_and_deduplicates_raw_paths(self) -> None:
        subject = load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            picture = root / "picture"
            builtin = root / "builtin"
            (picture / "Tools" / "Shared object").mkdir(parents=True)
            builtin.mkdir()
            for name in ("001.png", "007.png"):
                Image.new("RGB", (4, 4), "white").save(picture / "Tools" / "Shared object" / name)
            Image.new("RGBA", (4, 4), (10, 20, 30, 255)).save(
                builtin / "builtin_one__seed_0000.png"
            )
            rows = [
                {
                    "generator_index": "G0001",
                    "generator_name": "builtin_one",
                    "source_type": "articraft_builtin_dataset_no_picture",
                    "picture_category": "",
                    "picture_label": "",
                    "picture_source_path": "",
                },
                {
                    "generator_index": "G0002",
                    "generator_name": "shared_a",
                    "source_type": "picture_backed",
                    "picture_category": "Tools",
                    "picture_label": "Shared object",
                    "picture_source_path": "articraft_data/picture/Tools/Shared object",
                },
                {
                    "generator_index": "G0003",
                    "generator_name": "shared_b",
                    "source_type": "picture_backed",
                    "picture_category": "Tools",
                    "picture_label": "Shared object",
                    "picture_source_path": "articraft_data/picture/Tools/Shared object",
                },
            ]
            index = root / "index.csv"
            with index.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)

            bundle = subject.discover_generator_records(
                index,
                picture_root=picture,
                builtin_root=builtin,
                strict_counts=False,
            )

            self.assertEqual(len(bundle.records), 3)
            self.assertEqual(len(bundle.raw_image_paths), 3)
            self.assertEqual(bundle.records[2].alias_of_generator_index, "G0002")
            self.assertEqual(bundle.summary["unique_picture_directory_count"], 1)
            self.assertEqual(bundle.summary["unique_picture_image_count"], 2)

    def test_class_aggregation_normalizes_mean_vectors(self) -> None:
        subject = load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            image_a = root / "a.png"
            image_b = root / "b.png"
            image_c = root / "c.png"
            for image in (image_a, image_b, image_c):
                Image.new("RGB", (2, 2), "white").save(image)
            records = (
                subject.GeneratorRecord(
                    "G0001", "a", "picture_backed", "Tools", "a", root, (image_a, image_b)
                ),
                subject.GeneratorRecord(
                    "G0002", "b", "builtin_no_picture", "builtin_no_picture", "", None, (image_c,)
                ),
            )
            bundle = subject.DatasetBundle(
                records=records,
                raw_image_paths=(image_a, image_b, image_c),
                raw_path_to_index={image_a: 0, image_b: 1, image_c: 2},
                summary={},
            )
            raw = np.asarray([[3.0, 0.0], [0.0, 4.0], [0.0, 2.0]], dtype=np.float32)
            centers = subject.aggregate_class_features(raw, bundle)
            expected_first = np.asarray([0.5, 0.5], dtype=np.float32)
            expected_first /= np.linalg.norm(expected_first)
            np.testing.assert_allclose(centers[0], expected_first, atol=1e-6)
            np.testing.assert_allclose(centers[1], [0.0, 1.0], atol=1e-6)
            np.testing.assert_allclose(np.linalg.norm(centers, axis=1), 1.0, atol=1e-6)

    def test_alpha_images_are_composited_over_white(self) -> None:
        subject = load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "alpha.png"
            Image.new("RGBA", (1, 1), (0, 0, 0, 0)).save(path)
            image = subject.load_rgb_image(path)
            self.assertEqual(image.mode, "RGB")
            self.assertEqual(image.getpixel((0, 0)), (255, 255, 255))


if __name__ == "__main__":
    unittest.main()
