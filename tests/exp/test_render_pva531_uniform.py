from __future__ import annotations

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp" / "scripts" / "render_pva531_uniform.py"


def load_subject():
    spec = importlib.util.spec_from_file_location("render_pva531_uniform", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RenderPva531UniformTests(unittest.TestCase):
    def test_roster_maps_each_generator_to_seed_zero_and_stable_filename(self) -> None:
        subject = load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            assets = root / "assets"
            output = root / "output"
            rows = [
                {
                    "generator_index": "G0001",
                    "generator_name": "one_class",
                    "source_type": "picture_backed",
                    "picture_category": "Tools",
                },
                {
                    "generator_index": "G0002",
                    "generator_name": "builtin_class",
                    "source_type": "articraft_builtin_dataset_no_picture",
                    "picture_category": "",
                },
            ]
            for row in rows:
                seed = assets / row["generator_name"] / "seed_0000"
                seed.mkdir(parents=True)
                (seed / "model.urdf").write_text("<robot/>", encoding="utf-8")
                (seed / "appearance.json").write_text("{}", encoding="utf-8")
            index = root / "index.csv"
            with index.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)

            items = subject.load_render_items(
                index, asset_root=assets, output_root=output, strict_count=False
            )

            self.assertEqual(len(items), 2)
            self.assertEqual(items[0].asset_dir, assets / "one_class" / "seed_0000")
            self.assertEqual(items[0].output_path.name, "G0001__one_class__seed_0000.png")
            self.assertEqual(items[1].source_type, "builtin_no_picture")

    def test_resume_requires_a_matching_prior_content_receipt(self) -> None:
        subject = load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "G0001__one__seed_0000.png"
            Image.new("RGBA", (8, 8), (10, 20, 30, 255)).save(output)
            item = subject.RenderItem(
                ordinal=1,
                generator_index="G0001",
                generator_name="one",
                source_type="picture_backed",
                picture_category="Tools",
                asset_dir=root,
                output_path=output,
            )
            receipt = {
                "status": "rendered",
                "generator_name": "one",
                "output_path": str(output),
                "png_bytes": str(output.stat().st_size),
                "png_sha256": subject._sha256(output),
            }
            self.assertTrue(subject._receipt_allows_reuse(item, receipt, resolution=8))

            Image.new("RGBA", (8, 8), (200, 20, 30, 255)).save(output)
            self.assertFalse(subject._receipt_allows_reuse(item, receipt, resolution=8))


if __name__ == "__main__":
    unittest.main()
