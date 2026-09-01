from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp/scripts/render_partnet_mobility_n5_uniform.py"


@pytest.fixture(scope="module")
def subject():
    spec = importlib.util.spec_from_file_location("partnet_n5_test_subject", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _item(category: str, asset_id: str, identity: str):
    return SimpleNamespace(
        category=category,
        asset_id=asset_id,
        identity_sha256=identity,
        ordinal=int(asset_id),
        output_path=Path("/tmp/old") / category / asset_id / "imgs/000.png",
        source_path=Path("/tmp/source") / asset_id,
        urdf_path=Path("/tmp/source") / asset_id / "mobility.urdf",
        source_relative_path=asset_id,
        urdf_relative_path=f"{asset_id}/mobility.urdf",
        urdf_bytes=1,
        urdf_sha256="0" * 64,
        package_file_count=1,
        package_total_bytes=1,
        package_content_manifest_sha256="1" * 64,
        package_binding_sha256="2" * 64,
        category_one_shot=False,
    )


def test_select_five_is_balanced_and_hash_ranked(subject):
    items = [
        _item("Beta", str(i), f"{i:064x}")
        for i in range(7, 0, -1)
    ] + [_item("Alpha", str(i), f"{(20 - i):064x}") for i in range(1, 8)]
    selected = subject._select_five(items)
    assert len(selected) == 10
    assert [x[1] for x in selected] == [1, 2, 3, 4, 5] * 2
    assert [x[0].category for x in selected[:5]] == ["Alpha"] * 5
    alpha = [x[0].asset_id for x in selected if x[0].category == "Alpha"]
    assert alpha == ["7", "6", "5", "4", "3"]


def test_select_five_rejects_short_category(subject):
    items = [_item("Only", str(i), f"{i:064x}") for i in range(4)]
    with pytest.raises(ValueError, match="cannot select 5"):
        subject._select_five(items)


def test_common_row_contains_shared_loader_contract(subject, tmp_path):
    item = _item("Chair", "42", "a" * 64)
    item.output_path = tmp_path / "renders" / "Chair/42/imgs/000.png"
    row = subject._common_row(
        item, ordinal=3, source_ordinal=42, sample_index=4, category_index=7
    )
    assert row["ordinal"] == 3
    assert row["source_ordinal"] == 42
    assert row["render_key"] == "partnet_mobility:Chair:42:4"
    assert row["generator_index"] == 7
    assert row["generator_name"] == "Chair"
    assert row["source_type"] == "partnet_mobility"
    assert row["sample_index"] == 4
    assert row["output_path"].endswith("renders/Chair/42/imgs/000.png")


def test_default_contract_is_five_per_46_categories(subject):
    parser = subject.build_argument_parser()
    args = parser.parse_args([])
    assert args.samples_per_category == 5
    assert subject.BASE.EXPECTED_CATEGORY_COUNT == 46
    assert subject.DEFAULT_OUTPUT_ROOT == ROOT / "exp/partnet_mobility_uniform_n5_studio_256_v1"


def test_full_release_selection_has_contiguous_panel_ordinals(subject, tmp_path):
    items = subject.BASE.load_render_items(
        subject.BASE.DEFAULT_DATASET_MANIFEST,
        output_root=tmp_path / "renders",
        strict_counts=True,
        validate_inputs=False,
    )
    selected = subject._select_five(items)
    assert len(selected) == 230
    assert len({row[0].category for row in selected}) == 46
    assert {count for count in __import__("collections").Counter(row[0].category for row in selected).values()} == {5}
    assert len({row[0].asset_id for row in selected}) == 230
    panel, source_ordinals = subject._materialize_panel(selected, tmp_path / "panel")
    assert [item.ordinal for item, _rank in panel] == list(range(1, 231))
    assert set(source_ordinals) == {item.asset_id for item, _rank in panel}


def test_full_release_renderable_selection_excludes_missing_mesh(subject, tmp_path):
    items = subject.BASE.load_render_items(
        subject.BASE.DEFAULT_DATASET_MANIFEST,
        output_root=tmp_path / "renders",
        strict_counts=True,
        validate_inputs=False,
    )
    selected, exclusions = subject._select_renderable_five(items)
    assert len(selected) == 230
    assert {count for count in __import__("collections").Counter(row[0].category for row in selected).values()} == {5}
    excluded = {(row["category"], row["asset_id"]): row["reason"] for row in exclusions}
    assert ("Dishwasher", "12071") in excluded
    assert "missing or uncontained mesh" in excluded[("Dishwasher", "12071")]
    assert "12071" not in {row[0].asset_id for row in selected}
