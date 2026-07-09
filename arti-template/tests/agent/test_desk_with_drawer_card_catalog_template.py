from __future__ import annotations

from pathlib import Path

from agent.templates.desk_with_drawer_card_catalog import (
    build_desk_with_drawer,
    config_from_seed,
    resolve_config,
)
from sdk import AssetContext
from sdk._core.v0.testing import TestContext as SDKTestContext


def test_card_catalog_grid_drawer_dimensions_vary_within_allowed_bounds() -> None:
    observed_columns: set[int] = set()
    observed_rows: set[int] = set()

    for seed in range(40):
        resolved = resolve_config(config_from_seed(seed))
        observed_columns.add(resolved.drawer_columns)
        observed_rows.add(resolved.drawer_rows)

        assert 2 <= resolved.drawer_columns <= 6
        assert 2 <= resolved.drawer_rows <= 5
        assert resolved.drawer_count == resolved.drawer_columns * resolved.drawer_rows
        assert resolved.drawer_layout == "card_catalog_grid"

    assert len(observed_columns) > 1
    assert len(observed_rows) > 1


def test_card_catalog_closed_pose_has_no_apron_drawer_overlaps(tmp_path: Path) -> None:
    for seed in (3, 4, 6, 7, 8, 10):
        assets = AssetContext(tmp_path / f"seed_{seed}")
        model = build_desk_with_drawer(config_from_seed(seed), assets=assets)
        ctx = SDKTestContext(model, asset_root=assets.root)

        ctx.check_model_valid()
        ctx.fail_if_isolated_parts()
        ctx.fail_if_parts_overlap_in_current_pose(overlap_tol=0.003, overlap_volume_tol=1e-7)
        report = ctx.report()

        assert report.passed, report.failures


def test_card_catalog_grid_openings_stay_practical_when_grid_varies() -> None:
    for seed in range(40):
        resolved = resolve_config(config_from_seed(seed))
        assert resolved.opening_width >= 0.12
        assert resolved.opening_height >= 0.09
