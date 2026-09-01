from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

from PIL import Image, ImageStat


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp" / "scripts" / "plot_pva531_generator_colors.py"


class Pva531GeneratorColorPlotTests(unittest.TestCase):
    def load_subject(self):
        spec = importlib.util.spec_from_file_location("plot_pva531_generator_colors", SCRIPT)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module

    def sha256(self, path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def write_fixture(self, root: Path, *, count: int = 6, misalign_clip: bool = False) -> None:
        records = [
            {
                "generator_index": f"G{index:04d}",
                "generator_name": f"generator_{index:04d}",
                "source_type": "picture_backed" if index % 2 else "builtin_no_picture",
                "picture_category": "Furniture" if index % 2 else "",
                "picture_label": "chair" if index % 2 else "",
                "picture_dir": "",
                "image_count": "1",
                "alias_of_generator_index": "",
                "representative_image": f"render_{index:04d}.png",
            }
            for index in range(1, count + 1)
        ]
        root.mkdir(parents=True)
        with (root / "generator_roster_resolved.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
        manifest = {
            "schema_version": 1,
            "dataset": {"generator_count": count},
            "models": {
                "dinov2": {"encoder_label": "DINOv2"},
                "clip": {"encoder_label": "CLIP ViT-B/32"},
            },
        }
        (root / "run_manifest.json").write_text(
            json.dumps(manifest), encoding="utf-8"
        )
        for encoder_index, key in enumerate(("dinov2", "clip")):
            encoder_dir = root / key
            encoder_dir.mkdir()
            coordinate_records = list(records)
            if key == "clip" and misalign_clip:
                coordinate_records = list(reversed(coordinate_records))
            with (encoder_dir / "tsne_coordinates.csv").open(
                "w", encoding="utf-8", newline=""
            ) as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=["tsne_x", "tsne_y", "generator_index", "generator_name"],
                )
                writer.writeheader()
                for point_index, record in enumerate(coordinate_records):
                    if point_index < 2:
                        tsne_x = point_index * 0.00001 + encoder_index * 0.25
                        tsne_y = point_index * 0.00001 - encoder_index * 0.5
                    else:
                        tsne_x = point_index + encoder_index * 0.25
                        tsne_y = (point_index % 3) - encoder_index * 0.5
                    writer.writerow(
                        {
                            "tsne_x": tsne_x,
                            "tsne_y": tsne_y,
                            "generator_index": record["generator_index"],
                            "generator_name": record["generator_name"],
                        }
                    )

    def test_palette_has_531_deterministic_unique_hex_colors(self) -> None:
        subject = self.load_subject()

        first = subject.build_unique_palette(531)
        second = subject.build_unique_palette(531)

        self.assertEqual(first, second)
        self.assertEqual(len(first), 531)
        self.assertEqual(len(set(first)), 531)
        self.assertTrue(all(len(color) == 7 and color.startswith("#") for color in first))

    def test_loader_rejects_a_coordinate_roster_order_mismatch(self) -> None:
        subject = self.load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "input"
            self.write_fixture(root, misalign_clip=True)

            with self.assertRaisesRegex(ValueError, "not exactly aligned"):
                subject.load_inputs(root, expected_count=6)

    def test_run_writes_unique_class_plots_without_mutating_coordinates(self) -> None:
        subject = self.load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "input"
            self.write_fixture(root)
            coordinate_paths = [
                root / key / "tsne_coordinates.csv" for key in ("dinov2", "clip")
            ]
            before_hashes = [self.sha256(path) for path in coordinate_paths]

            result = subject.run(
                input_dir=root,
                expected_count=6,
                plot_dpi=180,
            )

            self.assertEqual(before_hashes, [self.sha256(path) for path in coordinate_paths])
            self.assertTrue(result["audit"]["pass"])
            self.assertEqual(result["manifest"]["dataset"]["unique_color_count"], 6)
            self.assertGreaterEqual(
                result["manifest"]["display_adjustments"]["dinov2"]["moved_point_count"],
                1,
            )
            expected_root_files = {
                "generator_class_color_index.csv",
                "generator_class_color_key.png",
                "generator_class_color_manifest.json",
                "generator_class_color_audit.json",
                "tsne_generator_class_comparison.png",
            }
            self.assertTrue(expected_root_files.issubset({path.name for path in root.iterdir()}))
            for key in ("dinov2", "clip"):
                self.assertTrue((root / key / "tsne_by_generator_class.png").is_file())
                self.assertTrue(
                    (root / key / "tsne_generator_class_plot_coordinates.csv").is_file()
                )
            with (root / "generator_class_color_index.csv").open(
                "r", encoding="utf-8", newline=""
            ) as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 6)
            self.assertEqual(len({row["color_hex"] for row in rows}), 6)
            for filename in (
                "tsne_generator_class_comparison.png",
                "generator_class_color_key.png",
            ):
                with Image.open(root / filename) as image:
                    self.assertGreaterEqual(image.width, 800)
                    self.assertGreaterEqual(image.height, 600)
                    self.assertGreater(ImageStat.Stat(image.convert("L")).stddev[0], 1.0)


if __name__ == "__main__":
    unittest.main()
