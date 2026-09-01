from __future__ import annotations

import csv
import importlib.util
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import pytest
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp/scripts/render_artiverse_n5_uniform.py"


def _load_subject() -> Any:
    spec = importlib.util.spec_from_file_location("render_artiverse_n5_uniform_test", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def subject() -> Any:
    return _load_subject()


def _item(subject: Any, root: Path, category: str, index: int) -> Any:
    source = "source"
    model_id = f"model_{index:03d}"
    manifest_root = f"data/{category}/{source}/{model_id}"
    return subject.BASE.RenderItem(
        ordinal=index,
        category=category,
        source=source,
        model_id=model_id,
        manifest_root=manifest_root,
        identity_sha256=subject.BASE._identity_sha256(manifest_root),
        category_one_shot=False,
        glb_path=root / f"{model_id}.segmented.glb",
        output_path=root / "old" / category / source / model_id / "imgs/000.png",
    )


def test_selection_caps_at_five_without_replacement(subject: Any, tmp_path: Path) -> None:
    items = tuple(
        [_item(subject, tmp_path, "alpha", index) for index in range(1, 8)]
        + [_item(subject, tmp_path, "beta", index) for index in range(20, 23)]
    )

    selected = subject.select_items(items, output_root=tmp_path / "renders")
    counts = Counter(entry.category for entry in selected)

    assert counts == {"alpha": 5, "beta": 3}
    assert len(selected) == 8
    assert len({entry.render_key for entry in selected}) == 8
    assert len({entry.asset_id for entry in selected}) == 8
    assert [entry.sample_index for entry in selected if entry.category == "beta"] == [1, 2, 3]
    for category in counts:
        candidates = sorted(
            [item for item in items if item.category == category],
            key=lambda item: (item.identity_sha256, item.manifest_root),
        )
        assert [entry.item.manifest_root for entry in selected if entry.category == category] == [
            item.manifest_root for item in candidates[:5]
        ]


def test_balanced_only_excludes_under_supported_categories(subject: Any, tmp_path: Path) -> None:
    items = tuple(
        [_item(subject, tmp_path, "alpha", index) for index in range(1, 6)]
        + [_item(subject, tmp_path, "beta", index) for index in range(20, 23)]
    )

    selected = subject.select_items(
        items,
        output_root=tmp_path / "renders",
        balanced_only=True,
    )

    assert len(selected) == 5
    assert {entry.category for entry in selected} == {"alpha"}
    assert [entry.sample_index for entry in selected] == [1, 2, 3, 4, 5]


def test_official_roster_yields_84_classes_368_assets_and_65_n5_classes(
    subject: Any,
    tmp_path: Path,
) -> None:
    items = subject.BASE.load_render_items(
        subject.BASE.DEFAULT_DATASET_MANIFEST,
        data_root=subject.BASE.DEFAULT_DATA_ROOT,
        output_root=tmp_path / "renders",
        strict_counts=True,
    )
    selected = subject.select_items(items, output_root=tmp_path / "renders")
    counts = Counter(entry.category for entry in selected)
    shortfalls, eligible = subject._shortfall_summary(items, 5)

    assert len(items) == 3_544
    assert len(counts) == 84
    assert len(selected) == 368
    assert eligible == 65
    assert len(shortfalls) == 19
    assert sorted(set(counts.values())) == [1, 2, 3, 4, 5]
    assert sum(count == 5 for count in counts.values()) == 65


def test_published_roster_has_shared_loader_fields_and_png_receipts(
    subject: Any,
    tmp_path: Path,
) -> None:
    entry = subject.SelectedItem(
        item=_item(subject, tmp_path, "alpha", 1),
        source_ordinal=9,
        class_id="C001",
        sample_index=1,
        render_key="C001__S01__alpha__source__model_001",
    )
    output = entry.item.output_path
    output.parent.mkdir(parents=True)
    Image.new("RGBA", (8, 8), (10, 20, 30, 255)).save(output)
    row = subject._entry_row(
        entry,
        png_bytes=output.stat().st_size,
        png_sha256=subject.BASE._sha256(output),
    )
    row.update({"glb_bytes": 4, "glb_sha256": "a" * 64})
    path = tmp_path / "render_roster.csv"
    subject._write_csv(path, subject.ROSTER_FIELDS, [row])

    with path.open("r", encoding="utf-8", newline="") as stream:
        published = list(csv.DictReader(stream))
    assert len(published) == 1
    required = {
        "ordinal",
        "render_key",
        "generator_index",
        "generator_name",
        "sample_index",
        "source_type",
        "asset_id",
        "output_path",
        "png_bytes",
        "png_sha256",
    }
    assert required.issubset(published[0])
    assert published[0]["png_sha256"] == subject.BASE._sha256(output)
