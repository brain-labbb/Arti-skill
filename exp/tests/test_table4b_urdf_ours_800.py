from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4b_ours_800.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("table4b_ours_800", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_mixed_table4_manifest_builds_800_bound_jobs() -> None:
    runner = load_runner()
    manifest = runner.load_source_manifest()
    jobs = runner.build_jobs(manifest)

    assert len(jobs) == 800
    assert [job["selection_index"] for job in jobs] == list(range(800))
    assert jobs[0]["asset_id"] == "Science_First_aid_cabinet/seed_0"
    assert jobs[0]["source_component"] == "Brain-500"
    assert jobs[500]["asset_id"].startswith("PV-A/")
    assert jobs[500]["source_component"] == "PV-A-300"
    assert jobs[500]["package"].startswith(
        "/root/.cache/torch/arti-skill/ours_pva_800_cohort_v2/pva_assets/"
    )
    assert Path(jobs[500]["urdf_path"]).is_file()


def test_output_names_identify_mixed_ours_800() -> None:
    runner = load_runner()

    assert runner.output_directory_name("formal", 800, "20260823T120000Z") == (
        "table4b_urdf_ours_brain500_pva300_n800_20260823T120000Z"
    )
    assert runner.output_directory_name("smoke", 5, "20260823T120000Z") == (
        "table4b_urdf_ours_brain500_pva300_smoke_n5_20260823T120000Z"
    )
