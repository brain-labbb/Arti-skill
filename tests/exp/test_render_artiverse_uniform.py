from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp" / "scripts" / "render_artiverse_uniform.py"
PVA_SCRIPT = ROOT / "exp" / "scripts" / "render_pva531_uniform.py"
OFFICIAL_MANIFEST_SHA256 = "8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586"


def load_subject():
    spec = importlib.util.spec_from_file_location("render_artiverse_uniform", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def literal_studio_contract(path: Path) -> dict[str, object]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "build_run_config"
    )
    returned = next(node.value for node in ast.walk(function) if isinstance(node, ast.Return))
    assert isinstance(returned, ast.Dict)
    for key, value in zip(returned.keys, returned.values, strict=True):
        if isinstance(key, ast.Constant) and key.value == "studio":
            contract = ast.literal_eval(value)
            assert isinstance(contract, dict)
            return contract
    raise AssertionError(f"no literal studio contract in {path}")


def write_manifest(path: Path, roots: list[str]) -> None:
    path.write_text(
        json.dumps(
            {
                "format": "artiverse-data-tar-gz-chunks-v1",
                "model_count": len(roots),
                "chunks": [{"model_count": len(roots), "roots": roots}],
            }
        ),
        encoding="utf-8",
    )


class RenderArtiverseUniformTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.subject = load_subject()
        cls.temporary = tempfile.TemporaryDirectory()
        cls.official_manifest = json.loads(
            cls.subject.DEFAULT_DATASET_MANIFEST.read_text(encoding="utf-8")
        )
        cls.official_items = cls.subject.load_render_items(
            cls.subject.DEFAULT_DATASET_MANIFEST,
            data_root=cls.subject.DEFAULT_DATA_ROOT,
            output_root=Path(cls.temporary.name) / "output",
            strict_counts=True,
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.temporary.cleanup()

    def test_official_manifest_is_the_exact_ordered_3544_model_roster(self) -> None:
        flattened = tuple(
            root
            for chunk in self.official_manifest["chunks"]
            for root in chunk["roots"]
        )
        items = self.official_items

        self.assertEqual(self.subject._sha256(self.subject.DEFAULT_DATASET_MANIFEST), OFFICIAL_MANIFEST_SHA256)
        self.assertEqual(len(flattened), 3544)
        self.assertEqual(len(items), 3544)
        self.assertEqual(len({item.category for item in items}), 84)
        self.assertEqual(tuple(item.manifest_root for item in items), flattened)
        self.assertEqual(tuple(item.ordinal for item in items), tuple(range(1, 3545)))
        self.assertTrue(all(item.glb_path.is_file() for item in items))

    def test_one_shot_is_one_hash_selected_model_per_category_without_feature_inputs(self) -> None:
        items = self.official_items
        winners = [item for item in items if item.category_one_shot]
        categories = sorted({item.category for item in items})

        self.assertEqual(len(winners), 84)
        self.assertEqual(sorted(item.category for item in winners), categories)
        self.assertEqual(len({item.manifest_root for item in winners}), 84)
        for category in categories:
            candidates = [item for item in items if item.category == category]
            expected = min(
                candidates,
                key=lambda item: (
                    hashlib.sha256(item.manifest_root.encode("utf-8")).hexdigest(),
                    item.manifest_root,
                ),
            )
            selected = next(item for item in candidates if item.category_one_shot)
            self.assertEqual(selected.manifest_root, expected.manifest_root)
            self.assertEqual(selected.identity_sha256, self.subject._identity_sha256(selected.manifest_root))

        with tempfile.TemporaryDirectory() as temporary:
            roster_path = Path(temporary) / "category_one_shot_roster.csv"
            receipts = {
                item.manifest_root: (item.ordinal, hashlib.sha256(item.manifest_root.encode()).hexdigest())
                for item in items
            }
            self.subject._write_one_shot_roster(roster_path, items, receipts)
            with roster_path.open("r", encoding="utf-8", newline="") as stream:
                roster_rows = list(csv.DictReader(stream))
        self.assertEqual(len(roster_rows), 84)
        self.assertEqual([row["category"] for row in roster_rows], categories)
        self.assertEqual(
            {row["manifest_root"] for row in roster_rows},
            {item.manifest_root for item in winners},
        )

        source = SCRIPT.read_text(encoding="utf-8").lower()
        for forbidden in ("dinov2", "clip", "tsne", "feature", "embedding"):
            self.assertNotIn(forbidden, source)

    def test_fresh_one_shot_dry_run_selects_all_84_categories(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output_root = Path(temporary) / "fresh"
            args = self.subject.build_argument_parser().parse_args(
                ["--output-root", str(output_root), "--one-shot-only", "--dry-run"]
            )
            receipts = {
                item.manifest_root: (
                    item.ordinal,
                    hashlib.sha256(item.manifest_root.encode("utf-8")).hexdigest(),
                )
                for item in self.official_items
            }
            with (
                mock.patch.object(
                    self.subject, "load_render_items", return_value=self.official_items
                ),
                mock.patch.object(
                    self.subject,
                    "_input_receipt",
                    return_value=(
                        {"segmented_glb_count": len(self.official_items)},
                        receipts,
                    ),
                ),
                mock.patch.object(
                    self.subject, "_blender_version", return_value="Blender 4.2.19 LTS"
                ),
            ):
                result = self.subject.run(args)

            self.assertEqual(result["status"], "dry_run")
            self.assertEqual(result["selection"]["selected_count"], 84)
            self.assertEqual(result["selection"]["selected_category_count"], 84)
            self.assertTrue(result["selection"]["one_shot_only"])
            with (output_root / "category_one_shot_roster.csv").open(
                "r", encoding="utf-8", newline=""
            ) as stream:
                roster = list(csv.DictReader(stream))
            self.assertEqual(len(roster), 84)
            self.assertEqual(len({row["category"] for row in roster}), 84)

    def test_model_root_symlink_cannot_escape_data_root(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data_root = root / "data"
            outside = root / "outside" / "model"
            outside.mkdir(parents=True)
            (outside / "model.segmented.glb").write_bytes(b"glTF")
            link = data_root / "category" / "source" / "model"
            link.parent.mkdir(parents=True)
            link.symlink_to(outside, target_is_directory=True)
            manifest = root / "manifest.json"
            write_manifest(manifest, ["data/category/source/model"])

            with self.assertRaisesRegex(ValueError, "escapes declared root"):
                self.subject.load_render_items(
                    manifest,
                    data_root=data_root,
                    output_root=root / "output",
                    strict_counts=False,
                )

    def test_config_studio_matches_pva_contract_field_for_field(self) -> None:
        expected = literal_studio_contract(PVA_SCRIPT)
        self.assertEqual(self.subject._studio_contract(), expected)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = root / "manifest.json"
            write_manifest(manifest, [])
            renderer = root / "renderer.py"
            shared_renderer = root / "shared_renderer.py"
            renderer.write_text("# renderer\n", encoding="utf-8")
            shared_renderer.write_text("# shared renderer\n", encoding="utf-8")
            data_root = root / "data"
            data_root.mkdir()
            args = argparse.Namespace(
                dataset_manifest=manifest,
                data_root=data_root,
                output_root=root / "output",
                resolution=256,
                samples=4,
                gpu="7",
                workers=4,
                timeout_seconds=900.0,
            )
            item = self.subject.RenderItem(
                ordinal=1,
                category="category",
                source="source",
                model_id="model",
                manifest_root="data/category/source/model",
                identity_sha256=self.subject._identity_sha256("data/category/source/model"),
                category_one_shot=True,
                glb_path=root / "model.segmented.glb",
                output_path=root / "output" / "category" / "source" / "model" / "imgs" / "000.png",
            )
            with mock.patch.object(self.subject, "_blender_version", return_value="Blender test"):
                config = self.subject.build_run_config(
                    args=args,
                    items=(item,),
                    renderer=renderer,
                    shared_renderer=shared_renderer,
                    blender=root / "blender",
                    input_receipt={"segmented_glb_count": 1},
                )

        self.assertEqual(config["studio"], expected)
        self.assertEqual(config["resolution"], 256)
        self.assertEqual(config["samples"], 4)

    def test_resume_receipt_rejects_png_glb_identity_and_path_tampering(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            glb = root / "model.segmented.glb"
            glb.write_bytes(b"original glb")
            output = root / "000.png"
            Image.new("RGBA", (8, 8), (10, 20, 30, 255)).save(output)
            identity = "data/category/source/model"
            item = self.subject.RenderItem(
                ordinal=1,
                category="category",
                source="source",
                model_id="model",
                manifest_root=identity,
                identity_sha256=self.subject._identity_sha256(identity),
                category_one_shot=True,
                glb_path=glb,
                output_path=output,
            )
            receipt = {
                "status": "rendered",
                "category": item.category,
                "source": item.source,
                "model_id": item.model_id,
                "manifest_root": item.manifest_root,
                "identity_sha256": item.identity_sha256,
                "glb_bytes": str(glb.stat().st_size),
                "glb_sha256": self.subject._sha256(glb),
                "output_path": str(output),
                "png_bytes": str(output.stat().st_size),
                "png_sha256": self.subject._sha256(output),
            }

            self.assertTrue(
                self.subject._receipt_allows_reuse(
                    item,
                    receipt,
                    resolution=8,
                    glb_bytes=glb.stat().st_size,
                    glb_sha256=self.subject._sha256(glb),
                )
            )

            Image.new("RGBA", (8, 8), (200, 20, 30, 255)).save(output)
            self.assertFalse(
                self.subject._receipt_allows_reuse(
                    item,
                    receipt,
                    resolution=8,
                    glb_bytes=glb.stat().st_size,
                    glb_sha256=self.subject._sha256(glb),
                )
            )

            Image.new("RGBA", (8, 8), (10, 20, 30, 255)).save(output)
            glb.write_bytes(b"tampered glb")
            self.assertFalse(
                self.subject._receipt_allows_reuse(
                    item,
                    receipt,
                    resolution=8,
                    glb_bytes=glb.stat().st_size,
                    glb_sha256=self.subject._sha256(glb),
                )
            )
            for field, value in (
                ("identity_sha256", "0" * 64),
                ("output_path", str(root / "other.png")),
            ):
                tampered = dict(receipt)
                tampered[field] = value
                self.assertFalse(
                    self.subject._receipt_allows_reuse(
                        item,
                        tampered,
                        resolution=8,
                        glb_bytes=int(receipt["glb_bytes"]),
                        glb_sha256=str(receipt["glb_sha256"]),
                    )
                )

    def test_contract_mismatch_does_not_overwrite_existing_one_shot_roster(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output_root = root / "output"
            output_root.mkdir()
            roster = output_root / "category_one_shot_roster.csv"
            sentinel = b"authoritative prior roster\n"
            roster.write_bytes(sentinel)
            (output_root / "render_config.json").write_text(
                json.dumps({"render_contract": "prior"}), encoding="utf-8"
            )
            renderer = root / "renderer.py"
            shared_renderer = root / "shared.py"
            blender = root / "blender"
            for path in (renderer, shared_renderer, blender):
                path.write_text("placeholder", encoding="utf-8")
            identity = "data/category/source/model"
            item = self.subject.RenderItem(
                ordinal=1,
                category="category",
                source="source",
                model_id="model",
                manifest_root=identity,
                identity_sha256=self.subject._identity_sha256(identity),
                category_one_shot=True,
                glb_path=root / "model.segmented.glb",
                output_path=output_root / "category" / "source" / "model" / "imgs" / "000.png",
            )
            args = argparse.Namespace(
                resolution=256,
                samples=4,
                workers=1,
                timeout_seconds=10.0,
                checkpoint_every=1,
                output_root=output_root,
                renderer=renderer,
                shared_renderer=shared_renderer,
                blender=blender,
                dataset_manifest=root / "manifest.json",
                data_root=root,
                allow_count_drift=True,
                one_shot_only=False,
                categories=None,
                limit=None,
                gpu="0",
                force=False,
                dry_run=True,
            )
            with (
                mock.patch.object(self.subject, "load_render_items", return_value=(item,)),
                mock.patch.object(
                    self.subject,
                    "_input_receipt",
                    return_value=({"segmented_glb_count": 1}, {identity: (1, "0" * 64)}),
                ),
                mock.patch.object(
                    self.subject,
                    "build_run_config",
                    return_value={"render_contract": "new"},
                ),
            ):
                with self.assertRaisesRegex(ValueError, "different render contract"):
                    self.subject.run(args)

            self.assertEqual(roster.read_bytes(), sentinel)

    def test_fresh_one_shot_run_writes_only_one_success_per_category(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data_root = root / "data"
            output_root = root / "output"
            roots = [
                f"data/{category}/source_{index}/model_{index}"
                for category in ("chair", "lamp", "table")
                for index in range(2)
            ]
            for manifest_root in roots:
                _, category, source, model_id = manifest_root.split("/")
                model_root = data_root / category / source / model_id
                model_root.mkdir(parents=True)
                (model_root / f"{model_id}.segmented.glb").write_bytes(
                    f"glb:{manifest_root}".encode("utf-8")
                )
            dataset_manifest = root / "manifest.json"
            write_manifest(dataset_manifest, roots)
            renderer = root / "renderer.py"
            shared_renderer = root / "shared_renderer.py"
            blender = root / "blender"
            for path in (renderer, shared_renderer, blender):
                path.write_text("test fixture\n", encoding="utf-8")

            args = argparse.Namespace(
                resolution=64,
                samples=1,
                workers=2,
                timeout_seconds=10.0,
                checkpoint_every=1,
                output_root=output_root,
                renderer=renderer,
                shared_renderer=shared_renderer,
                blender=blender,
                dataset_manifest=dataset_manifest,
                data_root=data_root,
                allow_count_drift=True,
                one_shot_only=True,
                categories=None,
                limit=None,
                gpu="0",
                force=False,
                dry_run=False,
            )

            def fake_render(item, *, input_receipt, **_kwargs):
                glb_bytes, glb_sha256 = input_receipt
                item.output_path.parent.mkdir(parents=True, exist_ok=True)
                Image.new("RGBA", (64, 64), (item.ordinal * 20, 40, 80, 255)).save(
                    item.output_path
                )
                return {
                    "ordinal": item.ordinal,
                    "category": item.category,
                    "source": item.source,
                    "model_id": item.model_id,
                    "manifest_root": item.manifest_root,
                    "identity_sha256": item.identity_sha256,
                    "category_one_shot": item.category_one_shot,
                    "glb_path": str(item.glb_path),
                    "glb_bytes": glb_bytes,
                    "glb_sha256": glb_sha256,
                    "output_path": str(item.output_path),
                    "status": "rendered",
                    "elapsed_seconds": 0.01,
                    "png_bytes": item.output_path.stat().st_size,
                    "png_sha256": self.subject._sha256(item.output_path),
                    "imported_cameras_removed": 0,
                    "imported_lights_removed": 0,
                    "started_at": "2026-01-01T00:00:00Z",
                    "finished_at": "2026-01-01T00:00:01Z",
                    "error": "",
                    "renderer_result": {"output": str(item.output_path)},
                }

            with (
                mock.patch.object(self.subject, "_blender_version", return_value="Blender test"),
                mock.patch.object(self.subject, "_render_one", side_effect=fake_render),
            ):
                summary = self.subject.run(args)

            with (output_root / "render_manifest.csv").open(
                "r", encoding="utf-8", newline=""
            ) as stream:
                manifest_rows = list(csv.DictReader(stream))
            with (output_root / "category_one_shot_roster.csv").open(
                "r", encoding="utf-8", newline=""
            ) as stream:
                roster_rows = list(csv.DictReader(stream))
            written_pngs = sorted(output_root.glob("*/*/*/imgs/000.png"))

            self.assertTrue(summary["one_shot_only"])
            self.assertEqual(summary["selected_count"], 3)
            self.assertEqual(summary["selected_category_count"], 3)
            self.assertEqual(summary["selected_valid_png_count"], 3)
            self.assertEqual(summary["full_valid_png_count"], 3)
            self.assertFalse(summary["full_complete"])
            self.assertEqual(len(manifest_rows), 3)
            self.assertEqual(len(roster_rows), 3)
            self.assertEqual(len(written_pngs), 3)
            self.assertEqual(len({row["category"] for row in manifest_rows}), 3)
            self.assertTrue(
                all(row["category_one_shot"].lower() == "true" for row in manifest_rows)
            )
            self.assertEqual(
                {row["manifest_root"] for row in manifest_rows},
                {row["manifest_root"] for row in roster_rows},
            )


if __name__ == "__main__":
    unittest.main()
