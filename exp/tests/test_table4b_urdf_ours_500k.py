from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4b_ours_500k.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("table4b_ours", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_frozen_brain_manifest_builds_500_ordered_jobs() -> None:
    runner = load_runner()
    manifest = runner.load_source_manifest()
    jobs = runner.build_jobs(manifest)

    assert len(jobs) == 500
    assert [job["selection_index"] for job in jobs] == list(range(500))
    assert jobs[0]["asset_id"] == "Science_First_aid_cabinet/seed_0"
    assert jobs[0]["category"] == "Science_First_aid_cabinet"
    assert jobs[0]["urdf_path"].endswith(
        "/Science_First_aid_cabinet/seed_0/model.urdf"
    )


def test_output_names_identify_ours() -> None:
    runner = load_runner()
    assert runner.output_directory_name("formal", 500, "20260823T120000Z") == (
        "table4b_urdf_ours_500k_table1cohort_n500_20260823T120000Z"
    )
    assert runner.output_directory_name("smoke", 5, "20260823T120000Z") == (
        "table4b_urdf_ours_500k_smoke_n5_20260823T120000Z"
    )
