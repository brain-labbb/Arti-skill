from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import sys
from collections import Counter
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[2]
RENDER_DRIVER = ROOT / "exp/scripts/render_articraft10k_uniform.py"
RENDERER = ROOT / "exp/scripts/render_articraft10k_asset_blender.py"
COMPARISON = ROOT / "exp/scripts/compare_pva_artiverse_articraft_uniform.py"
SMALL_PACKAGE = (
    ROOT
    / "exp/Articraft-10K/released_urdf"
    / "rec_single_revolute_hinge_4c93546a5c654bb9b472798cdb4a6a9f"
)


def load_subject(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def render_driver() -> Any:
    return load_subject("render_articraft10k_uniform_test_subject", RENDER_DRIVER)


@pytest.fixture(scope="module")
def renderer() -> Any:
    return load_subject("render_articraft10k_asset_blender_test_subject", RENDERER)


@pytest.fixture(scope="module")
def comparison() -> Any:
    return load_subject("compare_pva_artiverse_articraft_test_subject", COMPARISON)


def test_frozen_roster_selects_one_identity_hash_winner_per_244_categories(
    render_driver: Any,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def reject_file_hash(*_args: Any, **_kwargs: Any) -> str:
        raise AssertionError("metadata-only roster loading must not hash package files")

    monkeypatch.setattr(render_driver, "_sha256", reject_file_hash)
    items = render_driver.load_render_items(
        render_driver.DEFAULT_DATASET_MANIFEST,
        output_root=tmp_path / "renders",
        strict_counts=True,
        validate_inputs=False,
    )

    winners = [item for item in items if item.category_one_shot]
    category_counts = Counter(item.category for item in winners)
    assert len(items) == 10_787
    assert len({item.category for item in items}) == 244
    assert len(winners) == 244
    assert set(category_counts.values()) == {1}
    assert len({item.asset_id for item in winners}) == 244

    for category in category_counts:
        candidates = [item for item in items if item.category == category]
        expected = min(candidates, key=lambda item: (item.identity_sha256, item.asset_id))
        selected = next(item for item in candidates if item.category_one_shot)
        assert selected.asset_id == expected.asset_id


def test_native_urdf_parser_loads_small_release_package_and_rest_pose(renderer: Any) -> None:
    package = renderer.load_asset_package(SMALL_PACKAGE)
    matrices = renderer.rest_link_matrices(package)

    assert package.root_link == "fork"
    assert package.links == ("fork", "tab")
    assert [(joint.parent, joint.child) for joint in package.joints] == [("fork", "tab")]
    assert Counter(visual.geometry_type for visual in package.visuals) == {
        "mesh": 1,
        "cylinder": 2,
        "box": 1,
    }
    assert package.visuals[0].mesh_path is not None
    assert package.visuals[0].mesh_path.is_file()
    assert set(matrices) == set(package.links)
    assert matrices["tab"][2][3] == pytest.approx(0.04)


def test_joint_coordinate_writer_preserves_three_source_segments(
    comparison: Any,
    tmp_path: Path,
) -> None:
    pva = [
        SimpleNamespace(
            generator_index=f"G{index:04d}",
            generator_name=f"generator_{index}",
            source_type="picture_backed",
        )
        for index in range(1, 4)
    ]
    artiverse = [
        SimpleNamespace(category=f"artiverse_{index}", one_shot_source=f"source_{index}")
        for index in range(1, 3)
    ]
    articraft = [
        SimpleNamespace(category=f"articraft_{index}", cohort_origin="released_package_9996")
        for index in range(1, 5)
    ]
    coordinates = np.arange(18, dtype=np.float32).reshape(9, 2)
    output = tmp_path / "joint.csv"

    comparison.write_joint_coordinates(
        output,
        coordinates,
        pva_records=pva,
        artiverse_records=artiverse,
        articraft_records=articraft,
    )

    with output.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert [row["dataset"] for row in rows] == [
        "PV-A",
        "PV-A",
        "PV-A",
        "Artiverse",
        "Artiverse",
        "Articraft-10K",
        "Articraft-10K",
        "Articraft-10K",
        "Articraft-10K",
    ]
    assert [int(row["joint_index"]) for row in rows] == list(range(9))
    assert [row["class_id"] for row in rows] == [
        "G0001",
        "G0002",
        "G0003",
        "C01",
        "C02",
        "C001",
        "C002",
        "C003",
        "C004",
    ]
    np.testing.assert_allclose(
        np.asarray([[float(row["tsne_x"]), float(row["tsne_y"])] for row in rows]),
        coordinates,
    )


def test_articraft_palette_has_244_unique_colors(comparison: Any) -> None:
    colors = comparison.category_colors(244)

    assert len(colors) == 244
    assert len(set(colors)) == 244
    assert all(len(color) == 7 and color.startswith("#") for color in colors)


def test_recovery_journal_latest_row_overrides_checkpoint(
    render_driver: Any,
    tmp_path: Path,
) -> None:
    manifest = tmp_path / "render_manifest.csv"
    state = tmp_path / "render_state.jsonl"
    manifest.write_text(
        "asset_id,ordinal,status,error\nasset-a,7,rendered,\n",
        encoding="utf-8",
    )
    state.write_text(
        "\n".join(
            json.dumps(row, sort_keys=True)
            for row in (
                {"asset_id": "asset-a", "ordinal": 7, "status": "failed", "error": "retry"},
                {"asset_id": "asset-a", "ordinal": 7, "status": "reused_valid", "error": ""},
            )
        )
        + "\n",
        encoding="utf-8",
    )

    receipts = render_driver._read_recovery_receipts(
        manifest_path=manifest,
        state_path=state,
        roster={"asset-a": SimpleNamespace(ordinal=7)},
    )

    assert set(receipts) == {"asset-a"}
    assert receipts["asset-a"]["status"] == "reused_valid"


def test_selection_receipt_is_order_stable_and_subset_specific(
    render_driver: Any,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    first = SimpleNamespace(
        category="alpha",
        asset_id="asset-a",
        identity_sha256=hashlib.sha256(b"asset-a").hexdigest(),
    )
    second = SimpleNamespace(
        category="beta",
        asset_id="asset-b",
        identity_sha256=hashlib.sha256(b"asset-b").hexdigest(),
    )
    replacement = SimpleNamespace(
        category="gamma",
        asset_id="asset-c",
        identity_sha256=hashlib.sha256(b"asset-c").hexdigest(),
    )

    expected = render_driver._selection_receipt((first, second))
    reordered = render_driver._selection_receipt((second, first))
    different_subset = render_driver._selection_receipt((first, replacement))

    assert expected == reordered
    assert expected["count"] == different_subset["count"] == 2
    assert expected["identity_category_sha256"] != different_subset["identity_category_sha256"]

    monkeypatch.setattr(render_driver, "_sha256", lambda _path: "f" * 64)
    monkeypatch.setattr(render_driver, "_blender_version", lambda _path: "Blender fixture")
    args = SimpleNamespace(
        dataset_manifest=render_driver.DEFAULT_DATASET_MANIFEST,
        output_root=tmp_path / "renders",
        one_shot_only=True,
        resolution=256,
        samples=4,
        gpu="0",
        workers=1,
        timeout_seconds=10.0,
    )
    config = render_driver.build_run_config(
        args=args,
        all_items=(first, second, replacement),
        selected=(first, second),
        renderer=RENDERER,
        shared_renderer=render_driver.DEFAULT_SHARED_RENDERER,
        blender=Path("/fixture/blender"),
    )
    alternate = render_driver.build_run_config(
        args=args,
        all_items=(first, second, replacement),
        selected=(first, replacement),
        renderer=RENDERER,
        shared_renderer=render_driver.DEFAULT_SHARED_RENDERER,
        blender=Path("/fixture/blender"),
    )

    assert config["selection"]["selected_receipt"] == expected
    assert config["selection"]["selected_receipt"] != alternate["selection"]["selected_receipt"]


def test_shared_renderer_is_loaded_from_exact_receipted_file(
    renderer: Any,
    tmp_path: Path,
) -> None:
    helper = tmp_path / "studio_helper.py"
    helper.write_text(
        "\n".join(
            (
                "def _enable_cycles(*args): return 'cycles'",
                "def _scene_bounds(*args): return 'bounds'",
                "def _look_at(*args): return 'look'",
                "def _add_principled_material(*args): return 'material'",
            )
        )
        + "\n",
        encoding="utf-8",
    )
    expected_sha256 = hashlib.sha256(helper.read_bytes()).hexdigest()

    helpers, receipt = renderer._shared_helpers(helper, expected_sha256)

    assert [function() for function in helpers] == ["cycles", "bounds", "look", "material"]
    assert receipt == {"path": str(helper.resolve()), "sha256": expected_sha256}
    with pytest.raises(renderer.AssetPackageError, match="SHA-256 mismatch"):
        renderer._shared_helpers(helper, "0" * 64)


def test_count_drift_mode_accepts_self_consistent_smaller_roster(
    render_driver: Any,
    tmp_path: Path,
) -> None:
    package = tmp_path / "package"
    package.mkdir()
    urdf = package / "model.urdf"
    urdf.write_text("<robot name='fixture'><link name='root'/></robot>\n", encoding="utf-8")
    urdf_sha256 = hashlib.sha256(urdf.read_bytes()).hexdigest()
    rows = [
        {
            "ordinal": ordinal,
            "asset_id": asset_id,
            "category": category,
            "cohort_origin": "fixture",
            "source_path": str(package),
            "primary_urdf_path": str(urdf),
            "primary_urdf_size": urdf.stat().st_size,
            "primary_urdf_sha256": urdf_sha256,
            "package_binding_sha256": "1" * 64,
        }
        for ordinal, (asset_id, category) in enumerate(
            (("asset-a", "alpha"), ("asset-b", "beta"))
        )
    ]
    manifest = {
        "schema_version": render_driver.EXPECTED_SCHEMA,
        "dataset": render_driver.EXPECTED_DATASET,
        "N_eval": len(rows),
        "roster_sha256": render_driver._canonical_sha256(rows),
        "rows": rows,
    }
    manifest["manifest_content_sha256"] = render_driver._canonical_sha256(manifest)
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")

    with pytest.raises(ValueError, match="expected exactly"):
        render_driver.load_render_items(
            manifest_path,
            output_root=tmp_path / "strict",
            strict_counts=True,
        )
    items = render_driver.load_render_items(
        manifest_path,
        output_root=tmp_path / "drift",
        strict_counts=False,
    )

    assert len(items) == 2
    assert {item.category for item in items} == {"alpha", "beta"}
    assert all(item.category_one_shot for item in items)
