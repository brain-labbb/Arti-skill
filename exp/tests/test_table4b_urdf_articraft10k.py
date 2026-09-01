from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4b_articraft10k.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("run_urdf_table4b_articraft10k", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_frozen_manifest_builds_exact_ordered_jobs() -> None:
    runner = _load_runner()

    manifest = runner.load_source_manifest()
    jobs = runner.build_jobs(manifest)

    assert len(jobs) == 800
    assert [job["selection_index"] for job in jobs] == list(range(800))
    assert jobs[0]["asset_id"] == "rec_fidget_toy_d3e5dba051334aca88e7cba99ac794df"
    assert jobs[0]["expected_urdf_sha256"] == (
        "5c8edef469d3d86aee2f4fabb8ae12af4876d0323a1c6e931e8c50e62360a69a"
    )
    assert jobs[0]["urdf_path"].endswith(
        "/released_urdf/rec_fidget_toy_d3e5dba051334aca88e7cba99ac794df/model.urdf"
    )
    assert jobs[-1]["asset_id"] == "rec_bell_tower_with_swinging_bell_c2ebae809533486fba06f1158cfe13c2"
    assert jobs[-1]["expected_urdf_sha256"] == (
        "5bb49c6975aeb41ca4e17cd26c94b693261e58f5c81db5612d41cf42af3fe1b8"
    )


def test_package_binding_rejects_byte_drift(tmp_path: Path) -> None:
    runner = _load_runner()
    package = tmp_path / "package"
    package.mkdir()
    urdf = package / "model.urdf"
    urdf.write_text('<robot name="fixture"><link name="base"/></robot>\n', encoding="utf-8")
    expected = runner.compute_package_binding(package)

    runner.verify_package_binding(package, expected)
    urdf.write_text('<robot name="changed"><link name="base"/></robot>\n', encoding="utf-8")

    with pytest.raises(ValueError, match="package binding mismatch"):
        runner.verify_package_binding(package, expected)


@pytest.mark.parametrize(
    ("mode", "n", "expected"),
    [
        (
            "formal",
            800,
            "table4b_urdf_articraft10k_table2cohort_n800_seed20260813_20260821T120000Z",
        ),
        ("smoke", 3, "table4b_urdf_articraft10k_smoke_n3_20260821T120000Z"),
    ],
)
def test_output_directory_name_identifies_articraft(mode: str, n: int, expected: str) -> None:
    runner = _load_runner()

    assert runner.output_directory_name(mode, n, "20260821T120000Z") == expected


def test_summary_renders_unmeasured_load_time_as_ne() -> None:
    runner = _load_runner()
    ne = {"status": "N/E", "measured": 0, "intended": 3, "value": None, "median": None, "p90": None}
    summary = {
        "protocol_id": runner.PROTOCOL_ID,
        "geometry_protocol_id": runner.base.geometry.GEOMETRY_PROTOCOL_ID,
        "run_directory": "/tmp/fixture",
        "cohort": {"n_eval": 3},
        "status_counts": {"completed": 3, "error": 0, "total": 3},
        "metrics": {
            "analytic_collision_share": {
                "analytic_elements": 0,
                "loadable_collision_elements": 0,
                "rate": None,
            },
            "visual_to_collision_p95_normalized": dict(ne),
            "collision_to_visual_p95_normalized": dict(ne),
            "shapes_per_visual_bearing_link": dict(ne, mean=None),
            "collision_mesh_triangles_per_asset": dict(ne, mean=None),
            "intra_link_redundancy": {
                "status": "N/E",
                "value": None,
                "measured": 0,
                "intended": 0,
                "measured_assets": 0,
                "intended_assets": 3,
            },
            "collision_load_time_seconds": dict(ne, mean=None),
        },
    }

    rendered = runner.base.render_summary_md(summary)

    assert "# Table 4b - Articraft-10K" in rendered
    assert "| Collision Load Time/Asset | N/E |" in rendered
