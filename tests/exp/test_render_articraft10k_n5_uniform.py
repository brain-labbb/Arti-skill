from __future__ import annotations

import importlib.util
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SUBJECT_PATH = ROOT / "exp/scripts/render_articraft10k_n5_uniform.py"


def _load_subject() -> Any:
    spec = importlib.util.spec_from_file_location("render_articraft10k_n5_uniform_test", SUBJECT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _item(subject: Any, *, ordinal: int, category: str, asset_id: str, rank: str, root: Path) -> Any:
    source = root / "source" / asset_id
    source.mkdir(parents=True, exist_ok=True)
    urdf = source / "model.urdf"
    urdf.write_text("<robot name='fixture'><link name='base'/></robot>\n", encoding="utf-8")
    return subject.BASE.RenderItem(
        ordinal=ordinal,
        category=category,
        asset_id=asset_id,
        cohort_origin="fixture",
        source_path=source,
        urdf_path=urdf,
        urdf_bytes=urdf.stat().st_size,
        urdf_sha256=subject._sha256(urdf),
        package_binding_sha256="b" * 64,
        identity_sha256=rank,
        category_one_shot=False,
        output_path=root / "unused.png",
    )


def test_select_samples_uses_min_five_without_replacement(tmp_path: Path) -> None:
    subject = _load_subject()
    items = []
    ordinal = 0
    for category, support in (("alpha", 7), ("beta", 3), ("gamma", 1)):
        for index in range(support):
            ordinal += 1
            items.append(
                _item(
                    subject,
                    ordinal=ordinal,
                    category=category,
                    asset_id=f"{category}-{index}",
                    rank=f"{support - index:064x}",
                    root=tmp_path,
                )
            )

    selected, stats = subject.select_samples(items, output_root=tmp_path / "renders")

    counts = Counter(sample.class_name for sample in selected)
    assert counts == {"alpha": 5, "beta": 3, "gamma": 1}
    assert stats["per_class_count_values"] == [1, 3, 5]
    assert stats["balanced_n5_eligible"] == 1
    assert stats["short_category_count"] == 2
    assert len({sample.item.asset_id for sample in selected}) == len(selected) == 9
    assert [sample.sample_index for sample in selected if sample.class_name == "alpha"] == [1, 2, 3, 4, 5]
    assert [sample.item.asset_id for sample in selected if sample.class_name == "alpha"] == [
        "alpha-6",
        "alpha-5",
        "alpha-4",
        "alpha-3",
        "alpha-2",
    ]


def test_frozen_roster_selects_1193_distinct_assets_across_244_classes(tmp_path: Path) -> None:
    subject = _load_subject()
    all_items = subject.BASE.load_render_items(
        subject.DEFAULT_DATASET_MANIFEST,
        output_root=tmp_path / "renders",
        strict_counts=True,
        validate_inputs=False,
    )
    selected, stats = subject.select_samples(all_items, output_root=tmp_path / "renders")

    counts = Counter(sample.class_name for sample in selected)
    assert len(all_items) == 10_787
    assert len(counts) == stats["class_count"] == 244
    assert len(selected) == stats["asset_count"] == 1_193
    assert len({sample.item.asset_id for sample in selected}) == 1_193
    assert sorted(set(counts.values())) == stats["per_class_count_values"] == [1, 3, 4, 5]
    assert sum(value == 5 for value in counts.values()) == stats["balanced_n5_eligible"] == 236
    assert stats["short_category_count"] == 8


def test_verified_one_shot_image_is_reused_with_png_receipt(tmp_path: Path) -> None:
    subject = _load_subject()
    original = _item(
        subject,
        ordinal=9,
        category="alpha",
        asset_id="alpha-asset",
        rank="0" * 64,
        root=tmp_path,
    )
    selected, _stats = subject.select_samples([original], output_root=tmp_path / "new")
    sample = selected[0]
    reuse_root = tmp_path / "reuse"
    source = reuse_root / "alpha" / "alpha-asset" / "imgs" / "000.png"
    source.parent.mkdir(parents=True)
    Image.new("RGB", (256, 256), (90, 120, 150)).save(source)
    receipt = {
        "status": "rendered",
        "category": "alpha",
        "asset_id": "alpha-asset",
        "urdf_sha256": original.urdf_sha256,
        "package_binding_sha256": original.package_binding_sha256,
        "output_path": str(source),
        "png_sha256": subject._sha256(source),
    }

    result = subject._reuse_one_shot(
        sample,
        reuse_root=reuse_root,
        baseline={"alpha-asset": receipt},
        resolution=256,
    )

    assert result is not None
    assert result["status"] == "reused_valid"
    assert result["png_sha256"] == subject._sha256(sample.item.output_path)
    assert result["png_bytes"] == sample.item.output_path.stat().st_size
    assert sample.item.output_path.stat().st_ino == source.stat().st_ino


def test_finalize_binds_final_roster_manifest_and_config_hashes(tmp_path: Path) -> None:
    subject = _load_subject()
    root = tmp_path / "renders"
    output = root / "alpha" / "asset-a" / "imgs" / "000.png"
    output.parent.mkdir(parents=True)
    Image.new("RGB", (256, 256), (40, 80, 120)).save(output)
    png_sha256 = subject._sha256(output)
    row = {
        "ordinal": 1,
        "render_key": "A0001__S01__asset-a",
        "generator_index": "A0001",
        "generator_name": "alpha",
        "class_id": "A0001",
        "category": "alpha",
        "sample_index": 1,
        "category_support": 1,
        "balanced_n5_eligible": False,
        "source_type": "articraft10k",
        "asset_id": "asset-a",
        "cohort_origin": "fixture",
        "source_path": str(tmp_path / "source"),
        "urdf_path": str(tmp_path / "source/model.urdf"),
        "urdf_sha256": "a" * 64,
        "package_binding_sha256": "b" * 64,
        "identity_sha256": "c" * 64,
        "output_path": str(output),
        "png_bytes": output.stat().st_size,
        "png_sha256": png_sha256,
    }
    manifest_row = {
        **row,
        "gpu": "fixture",
        "status": "rendered",
        "elapsed_seconds": 1.0,
        "started_at": "fixture",
        "finished_at": "fixture",
        "error": "",
        "renderer_result": "{}",
    }
    subject._write_csv(root / "render_roster.csv", [row], subject.ROSTER_FIELDS)
    subject._write_csv(root / "render_manifest.csv", [manifest_row], subject.MANIFEST_FIELDS)
    subject._write_json(root / "render_config.json", {"resolution": 256, "driver_sha256": "0" * 64})
    subject._write_json(
        root / "render_summary.json",
        {"selected_count": 1, "selected_complete": True, "failure_count": 0},
    )

    summary = subject.finalize_existing(root)
    config = json.loads((root / "render_config.json").read_text(encoding="utf-8"))

    assert config["render_roster_rows"] == config["render_manifest_rows"] == 1
    assert config["render_roster_sha256"] == subject._sha256(root / "render_roster.csv")
    assert config["render_manifest_sha256"] == subject._sha256(root / "render_manifest.csv")
    assert config["driver_sha256"] == subject._sha256(subject.SCRIPT)
    assert summary["render_config_sha256"] == subject._sha256(root / "render_config.json")
    assert summary["finalized"] is True
