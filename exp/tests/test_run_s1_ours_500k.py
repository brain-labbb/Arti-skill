from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_s1_ours_500k.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("s1_ours", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_frozen_brain_cohort_preserves_nested_asset_ids() -> None:
    runner = load_runner()
    cohort = runner.load_cohort(
        runner.SOURCE_MANIFEST, runner.DATASET_ROOT, formal=True
    )

    assert len(cohort["records"]) == 500
    assert cohort["records"][0]["dataset_id"] == (
        "Science_First_aid_cabinet/seed_0"
    )
    assert cohort["records"][0]["primary_urdf_relative_path"] == "model.urdf"
    assert cohort["records"][0]["selection_index"] == 0


def test_frozen_table4_jsonl_is_independently_reaggregated() -> None:
    runner = load_runner()
    cohort = runner.load_cohort(
        runner.SOURCE_MANIFEST, runner.DATASET_ROOT, formal=True
    )
    evidence = runner.load_table4_evidence(
        runner.SOURCE_MANIFEST,
        runner.TABLE4_ASSET_RECORDS,
        runner.TABLE4_STATE_RECORDS,
        cohort,
        formal=True,
        verification_path=runner.TABLE4_VERIFICATION,
    )

    assert evidence["denominator"] == 500
    assert evidence["strict_passed"] == 485


def test_summary_renderer_uses_ours_dataset_name_without_recursing() -> None:
    runner = load_runner()
    metric = {"passed": 0, "denominator": 1, "rate": 0.0}
    summary = {
        "protocol_id": runner.PROTOCOL_ID,
        "status": "COMPLETE",
        "n_eval": 1,
        "metrics": {
            "receipt_bound_assets": metric,
            "receipt_replay_pass": metric,
            "deterministic_rebuild_match": {
                "status": "N/E", "eligible_assets": 0, "asset_denominator": 1,
            },
            "allowance_density": {"registered_pairs": 0, "eligible_pairs": 1, "rate": 0.0},
            "strict_pass_no_method_allowance": metric,
            "registered_allowance_gain_pp": {"value": 0.0},
        },
    }

    rendered = runner.render_summary(summary)

    assert "Supplementary Table S1: Ours-500K" in rendered
