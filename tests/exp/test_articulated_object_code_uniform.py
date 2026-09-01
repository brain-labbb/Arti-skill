from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / "exp/scripts/render_articulated_object_code_uniform.py"
WORKER_PATH = REPO_ROOT / "exp/scripts/render_articulated_object_code_asset_blender.py"


def _module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


runner = _module(RUNNER_PATH, "_test_aoc_uniform")
worker = _module(WORKER_PATH, "_test_aoc_worker")


def test_full_selection_contract(tmp_path: Path) -> None:
    items = runner.load_render_items(
        runner.DEFAULT_DATASET_MANIFEST,
        output_root=tmp_path,
        validate_inputs=False,
    )
    winners = sorted((item for item in items if item.category_one_shot), key=lambda item: item.ordinal)
    assert len(items) == 2_832
    assert len(winners) == 660
    assert [item.ordinal for item in winners] == list(range(660))
    assert len({item.category for item in winners}) == 660
    assert sum(item.tier == "viable" for item in winners) == 597
    assert sum(item.tier == "loads_only" for item in winners) == 63
    assert runner._selection_receipt(winners)["identity_category_sha256"] == (
        "f79294b0d4e5ea243c638a06a197d7f90aee57ed479165d08abdbfd0db8981c9"
    )


def test_viable_is_preferred_and_rel_path_hash_breaks_ties(tmp_path: Path) -> None:
    items = runner.load_render_items(
        runner.DEFAULT_DATASET_MANIFEST,
        output_root=tmp_path,
        validate_inputs=False,
    )
    by_category: dict[str, list] = {}
    for item in items:
        by_category.setdefault(item.category, []).append(item)
    for group in by_category.values():
        winner = next(item for item in group if item.category_one_shot)
        viable = [item for item in group if item.tier == "viable"]
        pool = viable or [item for item in group if item.tier == "loads_only"]
        expected = min(pool, key=lambda item: (runner._selection_sha256(item.source_relative_path), item.source_relative_path))
        assert winner.identity == expected.identity


def test_roster_is_contiguous_and_comparison_compatible(tmp_path: Path) -> None:
    items = runner.load_render_items(
        runner.DEFAULT_DATASET_MANIFEST,
        output_root=tmp_path,
        validate_inputs=False,
    )
    roster = tmp_path / "category_one_shot_roster.csv"
    runner._write_one_shot_roster(roster, items)
    with roster.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert [int(row["ordinal"]) for row in rows] == list(range(660))
    assert [row["category"] for row in rows] == sorted(row["category"] for row in rows)
    assert all(row["dataset_id"] == row["asset_id"] for row in rows)
    assert all(row["package_path"] == row["source_relative_path"] for row in rows)


def test_worker_parses_selected_generated_urdf() -> None:
    base = _module(runner.DEFAULT_BASE_RENDERER, "_test_aoc_base")
    package = worker.load_asset_package(
        runner.DEFAULT_SOURCE_ROOT
        / "objects/accordion_with_bellows_and_buttons/accordion_with_bellows_and_buttons_000"
    )
    assert package.root_link == "headrail"
    assert len(package.visuals) == 3
    assert all(visual.mesh_path is not None and visual.mesh_path.suffix == ".obj" for visual in package.visuals)
    matrices = base.rest_link_matrices(package)
    assert set(matrices) == set(package.links)


def test_render_dependency_receipt_binds_urdf_and_meshes() -> None:
    package = (
        runner.DEFAULT_SOURCE_ROOT
        / "objects/accordion_with_bellows_and_buttons/accordion_with_bellows_and_buttons_000"
    )
    count, total, content_hash, binding_hash = runner._package_receipt(package)
    assert count == 4
    assert total == 1_070_127
    assert content_hash == "42981fb721c56e73c23ec596b5f1c1d3c16650fe273894ce9a2e28d6a3b08fc8"
    assert binding_hash == "c518b05612388be70a4b2315a04a4b63b546d8c04def9c8afd62680bdf48701b"


def test_dry_run_has_no_output_side_effects(tmp_path: Path) -> None:
    output = tmp_path / "absent"
    args = runner.build_argument_parser().parse_args(
        ["--dry-run", "--limit", "1", "--output-root", str(output)]
    )
    result = runner.run(args)
    assert result["status"] == "dry_run"
    assert result["selection"]["selected_count"] == 1
    assert result["config"]["official_model_count"] == 3_217
    assert result["config"]["candidate_model_count"] == 2_832
    assert result["config"]["official_category_count"] == 787
    assert result["config"]["loadable_category_count"] == 660
    assert not output.exists()
