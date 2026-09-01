from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import check_table4_full_release as checker  # noqa: E402


def _record(
    asset_id: str = "asset-0",
    *,
    status: str = "completed",
    dof: int = 1,
    collision_status: str = "measured",
    pass_value: bool | None = True,
) -> dict[str, object]:
    expected = {"rest": 1, "single": 21 * dof, "sobol": 64 if dof else 0}
    row: dict[str, object] = {
        "asset_id": asset_id,
        "selection_index": 0,
        "status": status,
        "range_evaluable_dof_count": dof,
        "rest_state_expected": expected["rest"],
        "single_state_expected": expected["single"],
        "sobol_state_expected": expected["sobol"],
        "rest_state_executed": expected["rest"],
        "single_state_executed": expected["single"],
        "sobol_state_executed": expected["sobol"],
        "rest_non_adjacent_free": expected["rest"],
        "single_non_adjacent_free": expected["single"],
        "sobol_non_adjacent_free": expected["sobol"],
        "rest_all_pair_cf": pass_value,
        "rest_non_adjacent_cf": pass_value,
        "single_joint_sweep_cf": pass_value,
        "multi_joint_sobol_cf": pass_value,
        "strict_collision_pass": pass_value,
        "collision_metric_status": collision_status,
        "native_collision_elements": 0 if collision_status.upper() in {"N/E", "NE", "BLOCKED"} else 1,
        "measurement_complete": status == "completed",
        "max_penetration_normalized": 0.125 if collision_status == "measured" else None,
        "category": "fixture",
    }
    return row


def test_aggregate_recomputes_state_denominators_and_rates() -> None:
    records = [_record("a"), _record("b", dof=2)]
    summary = checker.aggregate_records(records, n_eval=2, j_eval=3)

    assert summary["expected_states"] == {"rest": 2, "single": 63, "sobol": 128}
    assert summary["executed_states"] == {"rest": 2, "single": 63, "sobol": 128}
    assert summary["metrics"]["rest_all_pair_cf"] == {
        "numerator": 2,
        "denominator": 2,
        "rate": 1.0,
    }
    assert summary["metrics"]["collision_state_rate"]["numerator"] == 0
    assert summary["metrics"]["collision_state_rate"]["denominator"] == 193


def test_no_native_collision_geometry_is_explicit_ne_and_not_vacuous_pass() -> None:
    records = [
        _record(
            status="blocked",
            collision_status="N/E",
            pass_value=False,
        )
    ]
    summary = checker.aggregate_records(records, n_eval=1, j_eval=1)

    assert summary["status"] == "BLOCKED"
    assert summary["metrics"]["collision_state_rate"]["status"] == "N/E"
    assert summary["metrics"]["collision_free_range"]["status"] == "N/E"
    assert summary["metrics"]["max_penetration"]["status"] == "N/E"
    for key in (
        "rest_all_pair_cf",
        "rest_non_adjacent_cf",
        "single_joint_sweep_cf",
        "multi_joint_sobol_cf",
        "strict_collision_pass",
    ):
        assert summary["metrics"][key]["status"] == "N/E"


def test_mixed_blocked_assets_keep_full_release_asset_denominator() -> None:
    records = [
        _record("a", pass_value=True),
        _record("b", status="blocked", collision_status="N/E", pass_value=False),
    ]
    summary = checker.aggregate_records(records, n_eval=2, j_eval=2)
    assert summary["metrics"]["rest_all_pair_cf"]["numerator"] == 1
    assert summary["metrics"]["rest_all_pair_cf"]["denominator"] == 2


def test_n_e_record_cannot_publish_true_collision_pass() -> None:
    record = _record(status="blocked", collision_status="N/E", pass_value=True)
    with pytest.raises(checker.AutomationError, match="despite collision N/E"):
        checker.aggregate_records([record], n_eval=1, j_eval=1)


def test_native_zero_dof_cannot_publish_vacuous_sweep_or_strict_pass() -> None:
    record = _record(dof=0, collision_status="measured", pass_value=True)
    with pytest.raises(checker.AutomationError, match="zero-DoF pass"):
        checker.aggregate_records([record], n_eval=1, j_eval=0)


def test_state_accounting_rejects_executed_above_frozen_expected() -> None:
    record = _record()
    record["single_state_executed"] = 22
    with pytest.raises(checker.AutomationError, match="exceeds expected"):
        checker.aggregate_records([record], n_eval=1, j_eval=1)


def test_record_identity_binds_order_hash_and_joint_count() -> None:
    row = {
        "asset_id": "fixture",
        "primary_urdf_sha256": "a" * 64,
        "joint_count": 2,
    }
    record = {
        "asset_id": "fixture",
        "selection_index": 0,
        "expected_primary_urdf_sha256": "a" * 64,
        "roster_joint_count": 2,
    }
    checker._verify_record_identity(record, row, 0)
    record["selection_index"] = 1
    with pytest.raises(checker.AutomationError, match="order mismatch"):
        checker._verify_record_identity(record, row, 0)


def test_self_hash_is_schema_specific() -> None:
    summary = {"schema_version": "table4_summary_v1", "metrics": {"ok": 1}}
    summary["summary_content_sha256"] = checker.common.canonical_sha256(summary)
    assert checker._check_self_hash(
        summary,
        "fixture summary",
        field="summary_content_sha256",
        required=True,
    ) == summary["summary_content_sha256"]


def test_receipt_requires_exact_eight_slugs(tmp_path: Path) -> None:
    root = tmp_path / "run"
    root.mkdir()
    receipt = {
        "schema_version": "table4_full_release_receipt_v1",
        "root": str(root),
        "methods": [{"slug": "articraft"}],
    }
    (root / "full_release_receipt.json").write_text(
        json.dumps(receipt), encoding="utf-8"
    )
    with pytest.raises(checker.AutomationError, match="exactly eight"):
        checker._receipt_entries(root)


def _v2_record() -> dict[str, object]:
    record = _record(dof=2)
    record.update(
        {
            "sampling_protocol": checker.SAMPLING_PROTOCOL_V2,
            "movable_dof_count": 2,
            "independent_dof_count": 1,
            "range_evaluable_independent_dof_count": 1,
            "mimic_joint_count": 1,
            "joint_sampling_plan_sha256": "a" * 64,
            "sampling_plan_error": None,
            "single_state_expected": 21,
            "single_state_executed": 21,
            "single_non_adjacent_free": 21,
            "sobol_state_expected": 64,
            "sobol_state_executed": 64,
            "sobol_non_adjacent_free": 64,
        }
    )
    return record


def test_v2_aggregate_uses_independent_not_declared_dof() -> None:
    summary = checker.aggregate_records([_v2_record()], n_eval=1, j_eval=2)
    assert summary["declared_dof_count"] == 2
    assert summary["independent_dof_count"] == 1
    assert summary["mimic_joint_count"] == 1
    assert summary["expected_states"] == {"rest": 1, "single": 21, "sobol": 64}


def test_v2_rejects_declared_denominator_or_plan_hash_drift() -> None:
    record = _v2_record()
    record["single_state_expected"] = 42
    with pytest.raises(checker.AutomationError, match="denominator mismatch"):
        checker.aggregate_records([record], n_eval=1, j_eval=2)

    record = _v2_record()
    record["joint_sampling_plan_sha256"] = "not-a-hash"
    with pytest.raises(checker.AutomationError, match="hash is malformed"):
        checker.aggregate_records([record], n_eval=1, j_eval=2)


def test_v2_state_coverage_rejects_plan_binding_drift(tmp_path: Path) -> None:
    record = _v2_record()
    record.update(
        {
            "dataset_id": "asset-0",
            "order": 0,
            "input_identity_sha256": "b" * 64,
            "rest_state_executed": 1,
            "single_state_executed": 0,
            "sobol_state_executed": 0,
            "measurement_complete": False,
        }
    )
    state = {
        "schema_version": "table4_state_v2",
        "dataset_id": "asset-0",
        "order": 0,
        "sampling_protocol": checker.SAMPLING_PROTOCOL_V2,
        "joint_sampling_plan_sha256": "c" * 64,
        "input_identity_sha256": "b" * 64,
        "phase": "rest",
        "sample_index": 0,
        "joint_name": None,
    }
    (tmp_path / "state_records.jsonl").write_text(
        json.dumps(state) + "\n", encoding="utf-8"
    )
    with pytest.raises(checker.AutomationError, match="sampling-plan hash mismatch"):
        checker._verify_state_records(
            tmp_path,
            [record],
            expected_sampling_protocol=checker.SAMPLING_PROTOCOL_V2,
        )
