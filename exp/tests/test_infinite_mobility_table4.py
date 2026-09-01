from __future__ import annotations

import importlib.util
from pathlib import Path


EXP = Path(__file__).resolve().parents[1]
RUNNER_PATH = EXP / "scripts/run_table4_infinite_mobility.py"
VERIFIER_PATH = EXP / "scripts/verify_table4_infinite_mobility.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("infinite_table4_test_runner", RUNNER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_verifier():
    spec = importlib.util.spec_from_file_location(
        "infinite_table4_test_verifier", VERIFIER_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_formal_state_plan_preserves_infinite_mobility_denominators() -> None:
    runner = _load_runner()
    plan = runner.formal_state_plan()

    assert plan == {
        "n_eval": 720,
        "j_eval": 4723,
        "zero_joint_assets": 55,
        "movable_assets": 665,
        "range_evaluable_joints": 4687,
        "all_range_evaluable_movable_assets": 629,
        "rest_expected": 720,
        "single_expected": 99183,
        "sobol_expected": 42560,
        "total_expected": 142463,
    }


def test_zero_collision_geometry_is_blocked_and_not_vacuous_pass() -> None:
    runner = _load_runner()
    record = runner.blocked_record(
        {
            "asset_id": "fixture/seed_000",
            "factory": "fixture",
            "seed": 0,
            "movable_dof_count": 1,
            "range_evaluable_dof_count": 1,
            "native_collision_element_count": 0,
            "rest_state_expected": 1,
            "single_state_expected": 21,
            "sobol_state_expected": 64,
        }
    )

    assert record["collision_metric_status"] == runner.BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT
    assert record["strict_collision_pass"] is None
    assert record["rest_all_pair_cf"] is None
    assert record["state_records"] == []
    assert record["unexecuted_state_count"] == 86


def test_blocked_summary_reports_n_e_for_collision_metrics() -> None:
    runner = _load_runner()
    summary = runner.summarize_blocked(
        [
            {
                "native_collision_element_count": 0,
                "rest_state_expected": 1,
                "single_state_expected": 21,
                "sobol_state_expected": 64,
                "unexecuted_state_count": 86,
                "strict_collision_pass": None,
            }
        ],
        n_eval=1,
        j_eval=1,
    )

    assert summary["metrics"]["rest_all_pair_cf"]["status"] == "N/E"
    assert summary["metrics"]["collision_state_rate"]["status"] == "N/E"
    assert summary["metrics"]["max_penetration"]["status"] == "N/E"
    assert summary["metrics"]["strict_collision_pass"]["status"] == "N/E"
    assert summary["claim_boundary"]["native_collision_element_total"] == 0


def test_verifier_requires_exact_frozen_source_identity_binding() -> None:
    verifier = _load_verifier()
    source = {
        "asset_id": "factory/seed_000",
        "factory": "factory",
        "raw_category": "factory",
        "seed": 0,
        "source": "primary",
        "original_status": "PASS",
        "recovery_used": False,
        "selection_index": 1,
        "urdf_relpath": "0/scene.urdf",
        "primary_urdf_sha256": "u" * 64,
        "package_path": "/cohort/factory/seed_000/package",
        "package_binding": {
            "content_manifest_sha256": "p" * 64,
            "file_count": 1,
            "total_bytes": 2,
        },
    }
    record = {
        "asset_id": "factory/seed_000",
        "asset_key": "factory/seed_000",
        "order": 1,
        "selection_index": 1,
        "factory": "factory",
        "raw_category": "factory",
        "seed": 0,
        "source": "primary",
        "original_status": "PASS",
        "recovery_used": False,
        "urdf_relpath": "0/scene.urdf",
        "urdf_sha256": "u" * 64,
        "package_path": "/cohort/factory/seed_000/package",
        "package_content_manifest_sha256": "p" * 64,
        "package_file_count": 1,
        "package_total_bytes": 2,
    }

    assert verifier.source_identity_matches(source, record, expected_order=1)
    record["asset_id"] = "factory/seed_001"
    assert not verifier.source_identity_matches(source, record, expected_order=1)
