from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4_ours_800.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("run_urdf_table4_ours_800", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_mixed_cohort_loads_brain_then_pva_with_per_asset_packages() -> None:
    runner = load_runner()

    cohort = runner.load_cohort(runner.DEFAULT_COHORT_MANIFEST)

    assert len(cohort["rows"]) == 800
    assert cohort["rows"][0]["asset_id"] == "Science_First_aid_cabinet/seed_0"
    assert cohort["rows"][0]["source_component"] == "Brain-500"
    assert cohort["rows"][500]["asset_id"].startswith("PV-A/")
    assert cohort["rows"][500]["source_component"] == "PV-A-300"
    assert Path(cohort["rows"][500]["package"]).joinpath("model.urdf").is_file()


def test_frozen_items_bind_each_absolute_package() -> None:
    runner = load_runner()
    cohort = runner.load_cohort(runner.DEFAULT_COHORT_MANIFEST)
    rows = cohort["rows"][:2] + cohort["rows"][500:502]
    audits = {row["asset_id"]: runner.audit_asset(Path("/ignored"), row) for row in rows}

    items = runner.build_frozen_items(rows, audits, {"fixture": True})

    assert [item["dataset_id"] for item in items] == [row["asset_id"] for row in rows]
    assert [item["package"] for item in items] == [row["package"] for row in rows]
    assert all(item["package_audit_success"] for item in items)
    assert all(item["primary_urdf_relpath"].endswith("/model.urdf") for item in items)
