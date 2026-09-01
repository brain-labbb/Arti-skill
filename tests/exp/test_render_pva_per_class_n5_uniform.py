from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp" / "scripts" / "render_pva_per_class_n5_uniform.py"


def load_subject():
    spec = importlib.util.spec_from_file_location(
        "render_pva_per_class_n5_uniform", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RenderPvaPerClassN5UniformTests(unittest.TestCase):
    def test_loader_joins_by_name_and_uses_asset_id_for_paths(self) -> None:
        subject = load_subject()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "renders"
            index = root / "index.csv"
            rows = [
                {
                    "generator_index": "G0001",
                    "generator_name": "category_b",
                    "source_type": "articraft_builtin_dataset_no_picture",
                    "picture_category": "",
                },
                {
                    "generator_index": "G0002",
                    "generator_name": "category_a",
                    "source_type": "picture_backed",
                    "picture_category": "Tools",
                },
            ]
            with index.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)

            assets = []
            for category, asset_id, rank, seed in [
                ("category_a", "seed_0042", "b" * 64, 7),
                ("category_a", "seed_0003", "a" * 64, 99),
                ("category_b", "seed_0011", "c" * 64, 11),
                ("category_b", "seed_0010", "a" * 64, 10),
            ]:
                package = root / "assets" / category / asset_id
                package.mkdir(parents=True)
                urdf = package / "model.urdf"
                urdf.write_text(f"<robot name='{asset_id}'/>", encoding="utf-8")
                (package / "appearance.json").write_text("{}", encoding="utf-8")
                assets.append(
                    {
                        "category": category,
                        "asset_id": asset_id,
                        "seed": seed,
                        "rank_sha256": rank,
                        "package": str(package),
                        "urdf_sha256": subject.baseline._sha256(urdf),
                        "package_binding": {"content_manifest_sha256": "d" * 64},
                    }
                )
            manifest = {
                "n_eval": 4,
                "class_count": 2,
                "per_class": 2,
                "selection": {},
                "assets": assets,
            }
            manifest["manifest_content_sha256"] = subject._canonical_sha256(manifest)
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

            items, _ = subject.load_frozen_items(
                manifest_path,
                index_csv=index,
                output_root=output,
                strict_release=False,
            )

            self.assertEqual([item.generator_name for item in items[:2]], ["category_b"] * 2)
            self.assertEqual([item.asset_id for item in items[:2]], ["seed_0010", "seed_0011"])
            self.assertEqual(items[0].seed, 10)
            self.assertEqual(items[2].asset_id, "seed_0003")
            self.assertEqual(items[2].seed, 99)
            self.assertEqual(items[0].source_type, "builtin_no_picture")
            self.assertEqual(
                items[0].output_path.name,
                "G0001__S01__seed_0010__category_b.png",
            )
            self.assertEqual(
                items[0].baseline_item().generator_index,
                "G0001__S01__seed_0010",
            )

    def test_gpu_parser_rejects_duplicates(self) -> None:
        subject = load_subject()
        self.assertEqual(subject._parse_gpus(["4,6"]), ("4", "6"))
        with self.assertRaises(ValueError):
            subject._parse_gpus(["4", "4"])


if __name__ == "__main__":
    unittest.main()
