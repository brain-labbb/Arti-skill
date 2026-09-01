from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sqlite3
import sys
import zlib

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts/audit_ours500_pva_table4_cohort.py"
)
SPEC = importlib.util.spec_from_file_location(
    "audit_ours500_pva_table4_cohort_tested", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
AUDIT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = AUDIT
SPEC.loader.exec_module(AUDIT)

CORE_SHA = "1" * 64
BULLET_SHA = "2" * 64
ADAPTER_SHA = "3" * 64
ROSTER_FILE_SHA = "4" * 64
ROSTER_CONTENT_SHA = "5" * 64


def _ours_item(order: int, category: str, seed: int, urdf_sha: str) -> dict[str, object]:
    seed_name = f"seed_{seed}"
    asset_id = f"{category}/{seed_name}"
    item: dict[str, object] = {
        "protocol_id": AUDIT.OURS_PROTOCOL_ID,
        "order": order,
        "dataset_id": asset_id,
        "asset_id": asset_id,
        "raw_category": category,
        "seed_name": seed_name,
        "asset_root_relpath": asset_id,
        "selection_rank": order + 1,
        "category": category,
        "package_audit_success": True,
        "audit_issue": None,
        "primary_urdf_relpath": f"{asset_id}/model.urdf",
        "urdf_sha256": urdf_sha,
        "valid_tree": True,
        "movable_dof_count": 1,
        "range_evaluable_dof_count": 1,
        "joint_specs_sha256": "6" * 64,
        "collision_mesh_inventory_sha256": "7" * 64,
        "missing_collision_mesh_reference_count": 0,
        "unsafe_collision_mesh_reference_count": 0,
        "scale_derivation_sha256": "8" * 64,
        "object_bbox_diagonal_m": 1.0,
        "rest_state_expected": 1,
        "single_state_expected": 21,
        "sobol_state_expected": 64,
    }
    item["input_identity_sha256"] = AUDIT.canonical_sha256(
        {field: item[field] for field in AUDIT.OURS_FROZEN_INPUT_FIELDS}
    )
    return item


def _ours_record(
    item: dict[str, object], passed: bool, schedule: list[dict[str, object]]
) -> dict[str, object]:
    states: list[dict[str, object]] = []

    def state(
        phase: str,
        sample_index: int,
        joint_name: str | None,
        *,
        collision: bool = False,
    ) -> dict[str, object]:
        penetration = 2e-6 if collision else 0.0
        count = int(collision)
        return {
            "protocol_id": item["protocol_id"],
            "order": item["order"],
            "dataset_id": item["dataset_id"],
            "category": item["category"],
            "input_identity_sha256": item["input_identity_sha256"],
            "phase": phase,
            "sample_index": sample_index,
            "joint_name": joint_name,
            "non_adjacent_illegal_penetration_count": count,
            "non_adjacent_max_penetration_m": penetration,
            "all_pair_illegal_penetration_count": count,
            "all_pair_max_penetration_m": penetration,
            "metric_max_penetration_m": penetration,
            "reset_readback_max_abs_error": 0.0,
        }

    for expected in schedule:
        row = state(
            str(expected["phase"]), int(expected["sample_index"]),
            expected["joint_name"] if expected["joint_name"] is None else str(expected["joint_name"]),
            collision=bool(not passed and expected["phase"] == "multi_joint_sobol" and expected["sample_index"] == 0),
        )
        row["joint_values_sha256"] = expected["joint_values_sha256"]
        states.append(row)
    max_penetration = 0.0 if passed else 2e-6
    return {
        "protocol_id": item["protocol_id"],
        "order": item["order"],
        "dataset_id": item["dataset_id"],
        "category": item["category"],
        "input_identity_sha256": item["input_identity_sha256"],
        "movable_dof_count": item["movable_dof_count"],
        "range_evaluable_dof_count": item["range_evaluable_dof_count"],
        "rest_state_expected": item["rest_state_expected"],
        "single_state_expected": item["single_state_expected"],
        "sobol_state_expected": item["sobol_state_expected"],
        "rest_state_executed": 1,
        "single_state_executed": item["single_state_expected"],
        "sobol_state_executed": item["sobol_state_expected"],
        "rest_non_adjacent_free": 1,
        "single_non_adjacent_free": item["single_state_expected"],
        "sobol_non_adjacent_free": int(item["sobol_state_expected"]) if passed else int(item["sobol_state_expected"]) - 1,
        "joint_single_sweep_cf_passed": item["range_evaluable_dof_count"],
        "rest_all_pair_cf": True,
        "rest_non_adjacent_cf": True,
        "single_joint_sweep_cf": True,
        "multi_joint_sobol_cf": passed,
        "measurement_complete": True,
        "strict_collision_pass": passed,
        "max_penetration_m": max_penetration,
        "max_penetration_normalized": max_penetration,
        "max_reset_readback_error": 0.0,
        "object_bbox_diagonal_m": 1.0,
        "load_success": True,
        "child_timed_out": False,
        "child_returncode": None,
        "issues": [],
        "state_records": states,
        "state_records_sha256": AUDIT.canonical_sha256(states),
        "collision_core_sha256": CORE_SHA,
        "runner_sha256": ADAPTER_SHA,
    }


def _pva_row(ordinal: int, category: str, seed: int, urdf_sha: str) -> dict[str, object]:
    seed_name = f"seed_{seed:04d}"
    asset_id = f"PV-A/{category}/{seed_name}"
    package = f"/sealed/PV-A/extracted/{category}/{seed_name}"
    return {
        "ordinal": ordinal,
        "asset_id": asset_id,
        "category": category,
        "raw_category": category,
        "joint_count": 1,
        "seed": seed,
        "source_asset_id": seed_name,
        "source_path": package,
        "primary_urdf_path": f"{package}/model.urdf",
        "primary_urdf_relative_path": f"extracted/{category}/{seed_name}/model.urdf",
        "primary_urdf_sha256": urdf_sha,
        "package_binding_sha256": hashlib.sha256(asset_id.encode()).hexdigest(),
    }


def _pva_states(row: dict[str, object], passed: bool) -> list[dict[str, object]]:
    identity = AUDIT._pva_input_identity(row, str(row["category"]), int(row["joint_count"]))
    result = []
    phases = [("rest", 1), ("single_joint_sweep", 21), ("multi_joint_sobol", 64)]
    for phase, count in phases:
        for index in range(count):
            collision = not passed and phase == "multi_joint_sobol" and index == 0
            penetration = 2e-6 if collision else 0.0
            result.append({
                "protocol_id": AUDIT.PVA_PROTOCOL_ID, "order": row["ordinal"],
                "dataset_id": row["asset_id"], "category": row["category"],
                "input_identity_sha256": identity, "phase": phase,
                "sample_index": index, "joint_name": "joint_0" if phase == "single_joint_sweep" else None,
                "non_adjacent_max_penetration_m": penetration,
                "non_adjacent_illegal_penetration_count": int(collision),
                "all_pair_max_penetration_m": penetration,
                "all_pair_illegal_penetration_count": int(collision),
                "metric_max_penetration_m": penetration,
                "reset_readback_max_abs_error": 0.0,
            })
    return result


def _pva_result(
    row: dict[str, object], passed: bool, states: list[dict[str, object]]
) -> dict[str, object]:
    category = str(row["category"])
    joint_count = int(row["joint_count"])
    return {
        "protocol_id": AUDIT.PVA_PROTOCOL_ID,
        "order": row["ordinal"],
        "dataset_id": row["asset_id"],
        "category": category,
        "expected_primary_urdf_sha256": row["primary_urdf_sha256"],
        "expected_movable_joints": joint_count,
        "primary_urdf_relative_path": row["primary_urdf_relative_path"],
        "package_binding_sha256": row["package_binding_sha256"],
        "input_identity_sha256": AUDIT._pva_input_identity(row, category, joint_count),
        "package": row["source_path"],
        "urdf_path": row["primary_urdf_path"],
        "strict_collision_pass": passed,
        "status": "completed",
        "measurement_complete": True,
        "load_success": True,
        "rest_state_expected": 1, "rest_state_executed": 1,
        "single_state_expected": 21, "single_state_executed": 21,
        "sobol_state_expected": 64, "sobol_state_executed": 64,
        "rest_non_adjacent_free": 1, "single_non_adjacent_free": 21,
        "sobol_non_adjacent_free": 64 if passed else 63,
        "rest_all_pair_cf": True, "rest_non_adjacent_cf": True,
        "single_joint_sweep_cf": True, "multi_joint_sobol_cf": passed,
        "max_penetration_m": 0.0 if passed else 2e-6,
        "max_reset_readback_error": 0.0,
        "state_records_count": len(states),
        "state_records_sha256": AUDIT.canonical_sha256(states),
    }


def _write_json(path: Path, value: object) -> None:
    path.write_text(AUDIT.canonical_text(value) + "\n", encoding="ascii")


def _self_hashed(value: dict[str, object], field: str) -> dict[str, object]:
    result = dict(value)
    result[field] = AUDIT.canonical_sha256(result)
    return result


def _rewrite_ours_records(root: Path, records: list[dict[str, object]]) -> None:
    (root / "asset_records.jsonl").write_text(
        "".join(AUDIT.canonical_text(row) + "\n" for row in records),
        encoding="ascii",
    )
    by_order = {int(row["order"]): row for row in records}
    (root / "state_records.jsonl").write_text(
        "".join(
            json.dumps(state, sort_keys=True, ensure_ascii=True) + "\n"
            for order in sorted(by_order)
            for state in by_order[order]["state_records"]
        ),
        encoding="ascii",
    )


def _build_fixture(tmp_path: Path) -> tuple[Path, Path]:
    ours_root = tmp_path / "ours"
    pva_root = tmp_path / "pva"
    ours_root.mkdir()
    pva_root.mkdir()

    dataset_root = ours_root / "dataset"
    urdf_payloads = {
        ("A", 0): (
            '<robot name="a"><link name="base"/><link name="leader"/>'
            '<link name="follower"/><joint name="leader_joint" type="revolute">'
            '<parent link="base"/><child link="leader"/><limit lower="-1" upper="1"/></joint>'
            '<joint name="follower_joint" type="revolute">'
            '<parent link="leader"/><child link="follower"/>'
            '<mimic joint="leader_joint" multiplier="1" offset="0"/>'
            '<limit lower="-1" upper="1"/>'
            '</joint></robot>\n'
        ),
        ("B", 0): (
            '<robot name="b"><link name="base"/><link name="door"/>'
            '<joint name="door_joint" type="revolute">'
            '<parent link="base"/><child link="door"/><limit lower="-1" upper="1"/></joint></robot>\n'
        ),
    }
    urdf_hashes: dict[tuple[str, int], str] = {}
    for (category, seed), payload in urdf_payloads.items():
        path = dataset_root / category / f"seed_{seed}" / "model.urdf"
        path.parent.mkdir(parents=True)
        path.write_text(payload, encoding="ascii")
        urdf_hashes[(category, seed)] = AUDIT.sha256_file(path)

    ours_items = [
        _ours_item(0, "A", 0, urdf_hashes[("A", 0)]),
        _ours_item(1, "B", 0, urdf_hashes[("B", 0)]),
    ]
    schedules: dict[int, list[dict[str, object]]] = {}
    for item in ours_items:
        urdf = dataset_root / str(item["primary_urdf_relpath"])
        joints, schedule = AUDIT._ours_v1_schedule(urdf)
        item["movable_dof_count"] = len(joints)
        item["range_evaluable_dof_count"] = sum(bool(row["range_evaluable"]) for row in joints)
        item["joint_specs_sha256"] = AUDIT.canonical_sha256(joints)
        item["single_state_expected"] = 21 * int(item["range_evaluable_dof_count"])
        item["sobol_state_expected"] = 64
        item["input_identity_sha256"] = AUDIT.canonical_sha256(
            {field: item[field] for field in AUDIT.OURS_FROZEN_INPUT_FIELDS}
        )
        schedules[int(item["order"])] = schedule
    ours_manifest: dict[str, object] = {
        "schema_version": AUDIT.OURS_MANIFEST_SCHEMA,
        "protocol_id": AUDIT.OURS_PROTOCOL_ID,
        "source": {
            "n_eval": 2,
            "n_release": 2,
            "category_count": 2,
            "dataset_root": str(dataset_root),
        },
        "evaluation": {
            "core_sha256": CORE_SHA,
            "adapter_sha256": ADAPTER_SHA,
            "single_samples": 21,
            "sobol_samples": 64,
            "sobol_seed": 20260813,
            "penetration_threshold_m": 1e-6,
            "reset_tolerance": 1e-9,
            "runtime_identity": {
                "collision_core_sha256": CORE_SHA,
                "pybullet_module_sha256": BULLET_SHA,
                "pybullet_api_version": 202010061,
            },
        },
        "selection": {
            "selected_asset_ids_sha256": AUDIT.canonical_sha256(
                [item["asset_id"] for item in ours_items]
            ),
            "ordered_identities_sha256": AUDIT.canonical_sha256(
                [
                    {
                        field: item[field]
                        for field in AUDIT.OURS_FROZEN_INPUT_FIELDS
                    }
                    for item in ours_items
                ]
            ),
        },
        "items": ours_items,
    }
    ours_manifest = _self_hashed(ours_manifest, "manifest_content_sha256")
    _write_json(ours_root / "frozen_manifest.json", ours_manifest)
    records = [
        _ours_record(ours_items[1], False, schedules[1]),
        _ours_record(ours_items[0], True, schedules[0]),
    ]
    (ours_root / "asset_records.jsonl").write_text(
        "".join(AUDIT.canonical_text(row) + "\n" for row in records),
        encoding="ascii",
    )
    records_by_order = {int(row["order"]): row for row in records}
    (ours_root / "state_records.jsonl").write_text(
        "".join(
            json.dumps(state, sort_keys=True, ensure_ascii=True) + "\n"
            for order in range(len(ours_items))
            for state in records_by_order[order]["state_records"]
        ),
        encoding="ascii",
    )
    summary: dict[str, object] = {
        "n_eval": 2,
        "n_release": 2,
        "status": "COMPLETE",
        "protocol_id": AUDIT.OURS_PROTOCOL_ID,
        "manifest_content_sha256": ours_manifest["manifest_content_sha256"],
        "cohort": {
            "category_count": 2,
            "selected": 2,
            "measurement_complete": 2,
        },
        "metrics": {
            "rest_all_pair_cf": {"denominator": 2, "passed": 2, "rate": 1.0},
            "rest_non_adjacent_cf": {"denominator": 2, "passed": 2, "rate": 1.0},
            "single_joint_sweep_cf": {"denominator": 2, "passed": 2, "rate": 1.0},
            "multi_joint_sobol_cf": {"denominator": 2, "passed": 1, "rate": 0.5},
            "strict_collision_pass": {"denominator": 2, "passed": 1, "rate": 0.5},
            "collision_free_range": {
                "denominator": 63,
                "passed_states": 63,
                "rate": 1.0,
            },
            "collision_state_rate": {
                "collision_states": 1,
                "definition": "fail-closed collision-or-unexecuted configurations / frozen expected configurations",
                "denominator": 193,
                "executed_states": 193,
                "observed_collision_rate_executed": 1 / 193,
                "observed_collision_states": 1,
                "rate": 1 / 193,
                "unexecuted_states": 0,
            },
            "max_penetration": {
                "denominator": 2,
                "fully_measured_assets": 2,
                "maximum_observed_normalized": 2e-6,
                "normalization": "fixture",
                "observed_assets": 2,
                "status": "COMPLETE",
            },
        },
    }
    _write_json(ours_root / "summary.json", summary)
    verification = {
        "checks": {
            "record_count_matches_frozen_items": True,
            "state_counter_replay_all_records": True,
            "strict_pass_consistent": True,
        },
        "executed_states": 193,
        "expected_states": 193,
        "status": "PASS",
    }
    _write_json(ours_root / "verification.json", verification)
    _write_json(ours_root / "checkpoint.json", {"state": "complete"})
    (ours_root / "report.md").write_text("fixture report\n", encoding="ascii")
    (ours_root / "protocol_document_at_freeze.md").write_text(
        "fixture protocol\n", encoding="ascii"
    )

    execution: dict[str, object] = {
        "schema_version": AUDIT.PVA_MANIFEST_SCHEMA,
        "N_eval": 4,
        "category_count": 3,
        "roster_manifest_sha256": ROSTER_FILE_SHA,
        "roster_manifest_content_sha256": ROSTER_CONTENT_SHA,
        "source_hashes": {"table4_core": CORE_SHA},
        "runtime_identity": {
            "collision_core_sha256": CORE_SHA,
            "pybullet_module_sha256": BULLET_SHA,
            "pybullet_api_version": 202010061,
        },
        "protocol": {
            "table4_single_joint_samples": 21,
            "table4_sobol_samples": 64,
            "table4_sobol_seed": 20260813,
            "table4_penetration_threshold_m": 1e-6,
        },
    }
    execution = _self_hashed(execution, "manifest_content_sha256")
    execution_path = pva_root / "manifest.json"
    _write_json(execution_path, execution)

    database = pva_root / "results.sqlite3"
    connection = sqlite3.connect(database)
    connection.executescript(
        """
        CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);
        CREATE TABLE assets (
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            category TEXT NOT NULL,
            joint_count INTEGER NOT NULL,
            row_sha256 TEXT NOT NULL,
            row_json TEXT NOT NULL
        );
        CREATE TABLE results (
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            table1_json TEXT NOT NULL,
            table2_json TEXT NOT NULL,
            table2sup_json TEXT NOT NULL,
            table3_json TEXT NOT NULL,
            table4_json TEXT NOT NULL,
            table4_states_zlib BLOB NOT NULL,
            table4_state_count INTEGER NOT NULL,
            worker_status TEXT NOT NULL,
            worker_seconds REAL NOT NULL,
            completed_at_utc TEXT NOT NULL
        );
        """
    )
    meta = {
        "schema_version": AUDIT.PVA_DATABASE_SCHEMA,
        "asset_import_state": "COMPLETE",
        "selected_n": 4,
        "selected_category_count": 3,
        "roster_manifest_sha256": ROSTER_FILE_SHA,
        "roster_manifest_content_sha256": ROSTER_CONTENT_SHA,
    }
    connection.executemany(
        "INSERT INTO meta(key,value) VALUES(?,?)",
        [(key, AUDIT.canonical_text(value)) for key, value in meta.items()],
    )
    rows = [
        (_pva_row(0, "A", 0, urdf_hashes[("A", 0)]), False),
        (_pva_row(1, "B", 0, "c" * 64), True),
        (_pva_row(2, "A", 1, "d" * 64), True),
        (_pva_row(3, "C", 0, "e" * 64), False),
    ]
    # Generator seed is metadata, not the cross-release package identity.
    rows[0][0]["seed"] = 913
    for row, passed in rows:
        row_text = AUDIT.canonical_text(row)
        states = _pva_states(row, passed)
        state_payload = "".join(AUDIT.canonical_text(state) + "\n" for state in states).encode()
        connection.execute(
            "INSERT INTO assets VALUES(?,?,?,?,?,?)",
            (
                row["ordinal"],
                row["asset_id"],
                row["category"],
                row["joint_count"],
                hashlib.sha256(row_text.encode()).hexdigest(),
                row_text,
            ),
        )
        connection.execute(
            "INSERT INTO results VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
            (
                row["ordinal"],
                row["asset_id"],
                "{}",
                "{}",
                "{}",
                "{}",
                AUDIT.canonical_text(_pva_result(row, passed, states)),
                zlib.compress(state_payload),
                len(states),
                "completed",
                0.1,
                "2026-08-27T00:00:00Z",
            ),
        )
    connection.commit()
    connection.close()

    receipt: dict[str, object] = {
        "schema_version": AUDIT.PVA_RECEIPT_SCHEMA,
        "N_eval": 4,
        "N_release": 4,
        "eval_category_count": 3,
        "release_category_count": 3,
        "execution_manifest": "manifest.json",
        "execution_manifest_sha256": AUDIT.sha256_file(execution_path),
        "result_database": "results.sqlite3",
        "result_database_bytes": database.stat().st_size,
        "result_database_sha256": AUDIT.sha256_file(database),
        "roster_manifest_sha256": ROSTER_FILE_SHA,
        "roster_manifest_content_sha256": ROSTER_CONTENT_SHA,
    }
    receipt = _self_hashed(receipt, "receipt_content_sha256")
    _write_json(pva_root / "full_release_receipt.json", receipt)
    return ours_root, pva_root


def test_fixture_replays_paired_and_full_cohorts_read_only(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    database = pva_root / "results.sqlite3"
    before = database.read_bytes()

    result = AUDIT.build_audit(ours_root, pva_root)

    paired = result["label_matched_category_seed_outputs"]
    assert paired["ours_500k_output"] == {
        "denominator": 2,
        "passed": 1,
        "rate": 0.5,
    }
    assert paired["current_pva_v1_output"] == {
        "denominator": 2,
        "passed": 1,
        "rate": 0.5,
    }
    assert paired["contingency"] == {
        "both_pass": 0,
        "ours_only_pass": 1,
        "pva_only_pass": 1,
        "both_fail": 0,
    }
    assert paired["byte_identical_primary_urdf_count"] == 1
    payloads = result["inputs"]["ours_500k"]["primary_urdf_payloads"]
    assert payloads["verified_count"] == 2
    assert payloads["structured_xml_parsed_count"] == 2
    assert payloads["mimic_asset_count"] == 1
    assert payloads["mimic_joint_count"] == 1
    partition = result["current_pva_v1_cohort_partition"]
    assert partition["ours_500k_categories"]["category_count"] == 2
    assert partition["ours_500k_categories"]["denominator"] == 3
    assert partition["ours_500k_categories"]["passed"] == 2
    assert partition["remaining_categories"] == {
        "category_count": 1,
        "denominator": 1,
        "passed": 0,
        "rate": 0.0,
    }
    assert partition["full_release"]["denominator"] == 4
    assert partition["full_release"]["passed"] == 2
    assert result["protocol_alignment"]["all_aligned"] is True
    assert result["protocol_alignment"]["causal_protocol_attribution_supported"] is False
    assert result["audit_semantics"]["causal_protocol_attribution_supported"] is False
    closure = result["inputs"]["ours_500k"]["retrospective_artifact_closure"]
    assert closure["original_formal_artifact_manifest_present"] is False
    assert AUDIT.verify_audit_self_hash(result)
    assert database.read_bytes() == before


def test_canonical_output_round_trips_with_self_hash(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    result = AUDIT.build_audit(ours_root, pva_root)
    output = tmp_path / "audit.json"

    AUDIT.write_canonical_json(output, result)

    raw = output.read_text(encoding="ascii")
    assert raw == AUDIT.canonical_text(result) + "\n"
    assert AUDIT.verify_audit_self_hash(json.loads(raw))


def test_duplicate_ours_result_fails_closed(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    records_path = ours_root / "asset_records.jsonl"
    rows = [json.loads(line) for line in records_path.read_text().splitlines()]
    rows[1] = copy.deepcopy(rows[0])
    records_path.write_text(
        "".join(AUDIT.canonical_text(row) + "\n" for row in rows),
        encoding="ascii",
    )

    with pytest.raises(AUDIT.CohortAuditError, match="duplicate Ours-500K record order"):
        AUDIT.build_audit(ours_root, pva_root)


def test_missing_pva_pair_fails_closed(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    manifest_path = ours_root / "frozen_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest.pop("manifest_content_sha256")
    item = manifest["items"][1]
    item["raw_category"] = "Z"
    item["category"] = "Z"
    item["dataset_id"] = "Z/seed_0"
    item["asset_id"] = "Z/seed_0"
    item["asset_root_relpath"] = "Z/seed_0"
    item["primary_urdf_relpath"] = "Z/seed_0/model.urdf"
    original_urdf = ours_root / "dataset/B/seed_0/model.urdf"
    replacement_urdf = ours_root / "dataset/Z/seed_0/model.urdf"
    replacement_urdf.parent.mkdir(parents=True)
    replacement_urdf.write_bytes(original_urdf.read_bytes())
    item["input_identity_sha256"] = AUDIT.canonical_sha256(
        {field: item[field] for field in AUDIT.OURS_FROZEN_INPUT_FIELDS}
    )
    manifest["selection"]["selected_asset_ids_sha256"] = AUDIT.canonical_sha256(
        [row["asset_id"] for row in manifest["items"]]
    )
    manifest["selection"]["ordered_identities_sha256"] = AUDIT.canonical_sha256(
        [
            {field: row[field] for field in AUDIT.OURS_FROZEN_INPUT_FIELDS}
            for row in manifest["items"]
        ]
    )
    manifest = _self_hashed(manifest, "manifest_content_sha256")
    _write_json(manifest_path, manifest)

    summary_path = ours_root / "summary.json"
    summary = json.loads(summary_path.read_text())
    summary["manifest_content_sha256"] = manifest["manifest_content_sha256"]
    _write_json(summary_path, summary)

    records_path = ours_root / "asset_records.jsonl"
    records = [json.loads(line) for line in records_path.read_text().splitlines()]
    record = next(row for row in records if row["order"] == 1)
    record["dataset_id"] = item["dataset_id"]
    record["category"] = item["category"]
    record["input_identity_sha256"] = item["input_identity_sha256"]
    for state in record["state_records"]:
        state["dataset_id"] = item["dataset_id"]
        state["category"] = item["category"]
        state["input_identity_sha256"] = item["input_identity_sha256"]
    record["state_records_sha256"] = AUDIT.canonical_sha256(record["state_records"])
    _rewrite_ours_records(ours_root, records)

    with pytest.raises(AUDIT.CohortAuditError, match="missing 1 Ours-500K pairs"):
        AUDIT.build_audit(ours_root, pva_root)


def test_pva_roster_row_hash_drift_fails_closed(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    database = pva_root / "results.sqlite3"
    connection = sqlite3.connect(database)
    connection.execute("UPDATE assets SET row_sha256=? WHERE ordinal=0", ("0" * 64,))
    connection.commit()
    connection.close()

    with pytest.raises(AUDIT.CohortAuditError, match="row_sha256 mismatch"):
        AUDIT.build_audit(ours_root, pva_root)


def test_ours_primary_urdf_hash_drift_fails_closed(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    urdf = ours_root / "dataset/A/seed_0/model.urdf"
    urdf.write_text('<robot name="drifted"/>\n', encoding="ascii")

    with pytest.raises(
        AUDIT.CohortAuditError, match="primary URDF SHA256 mismatch"
    ):
        AUDIT.build_audit(ours_root, pva_root)


def test_ours_published_strict_flip_fails_independent_replay(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    records_path = ours_root / "asset_records.jsonl"
    records = [json.loads(line) for line in records_path.read_text().splitlines()]
    records[0]["strict_collision_pass"] = not records[0]["strict_collision_pass"]
    _rewrite_ours_records(ours_root, records)

    with pytest.raises(
        AUDIT.CohortAuditError,
        match="published field strict_collision_pass mismatch",
    ):
        AUDIT.build_audit(ours_root, pva_root)


def test_ours_load_failure_fails_execution_replay(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    records_path = ours_root / "asset_records.jsonl"
    records = [json.loads(line) for line in records_path.read_text().splitlines()]
    records[0]["load_success"] = False
    _rewrite_ours_records(ours_root, records)

    with pytest.raises(AUDIT.CohortAuditError, match="unsuccessful load/child/worker"):
        AUDIT.build_audit(ours_root, pva_root)


def test_ours_reset_readback_above_frozen_tolerance_fails(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    records_path = ours_root / "asset_records.jsonl"
    records = [json.loads(line) for line in records_path.read_text().splitlines()]
    records[0]["state_records"][0]["reset_readback_max_abs_error"] = 2e-9
    records[0]["state_records_sha256"] = AUDIT.canonical_sha256(
        records[0]["state_records"]
    )
    _rewrite_ours_records(ours_root, records)

    with pytest.raises(AUDIT.CohortAuditError, match="exceeds frozen tolerance"):
        AUDIT.build_audit(ours_root, pva_root)


@pytest.mark.parametrize(
    ("field", "value"),
    [("issues", ["parent_worker_exception: RuntimeError: injected"]), ("child_returncode", 7)],
)
def test_ours_worker_or_child_failure_fails_execution_replay(
    tmp_path: Path, field: str, value: object
) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    records_path = ours_root / "asset_records.jsonl"
    records = [json.loads(line) for line in records_path.read_text().splitlines()]
    records[0][field] = value
    _rewrite_ours_records(ours_root, records)

    with pytest.raises(AUDIT.CohortAuditError, match="unsuccessful load/child/worker"):
        AUDIT.build_audit(ours_root, pva_root)


@pytest.mark.parametrize("tamper", ["states", "phase", "free"])
def test_ours_state_semantic_tamper_fails_independent_replay(
    tmp_path: Path, tamper: str
) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    records_path = ours_root / "asset_records.jsonl"
    records = [json.loads(line) for line in records_path.read_text().splitlines()]
    record = records[0]
    if tamper == "states":
        record["state_records"].pop()
        expected = "schedule length mismatch"
    elif tamper == "phase":
        record["state_records"][0]["phase"] = "invented_phase"
        expected = "schedule mismatch: phase"
    else:
        record["state_records"][0]["non_adjacent_illegal_penetration_count"] = 1
        expected = "non-adjacent threshold/count mismatch"
    record["state_records_sha256"] = AUDIT.canonical_sha256(record["state_records"])
    _rewrite_ours_records(ours_root, records)

    with pytest.raises(AUDIT.CohortAuditError, match=expected):
        AUDIT.build_audit(ours_root, pva_root)


def test_ours_external_state_artifact_binding_drift_fails_closed(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    states_path = ours_root / "state_records.jsonl"
    states_path.write_text(states_path.read_text() + "\n", encoding="ascii")

    with pytest.raises(
        AUDIT.CohortAuditError,
        match="external state-record artifact does not match embedded states",
    ):
        AUDIT.build_audit(ours_root, pva_root)


def test_ours_joint_value_schedule_hash_tamper_fails(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    records_path = ours_root / "asset_records.jsonl"
    records = [json.loads(line) for line in records_path.read_text().splitlines()]
    records[0]["state_records"][1]["joint_values_sha256"] = "0" * 64
    records[0]["state_records_sha256"] = AUDIT.canonical_sha256(records[0]["state_records"])
    _rewrite_ours_records(ours_root, records)
    with pytest.raises(AUDIT.CohortAuditError, match="schedule mismatch: joint_values_sha256"):
        AUDIT.build_audit(ours_root, pva_root)


def test_pva_state_blob_semantic_tamper_fails_replay(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    database = pva_root / "results.sqlite3"
    connection = sqlite3.connect(database)
    record_text, blob = connection.execute(
        "SELECT table4_json, table4_states_zlib FROM results WHERE ordinal=1"
    ).fetchone()
    record = json.loads(record_text)
    states = [json.loads(line) for line in zlib.decompress(blob).decode().splitlines()]
    states[-1]["non_adjacent_max_penetration_m"] = 2e-6
    states[-1]["non_adjacent_illegal_penetration_count"] = 1
    states[-1]["metric_max_penetration_m"] = 2e-6
    record["state_records_sha256"] = AUDIT.canonical_sha256(states)
    payload = "".join(AUDIT.canonical_text(state) + "\n" for state in states).encode()
    connection.execute(
        "UPDATE results SET table4_json=?, table4_states_zlib=? WHERE ordinal=1",
        (AUDIT.canonical_text(record), zlib.compress(payload)),
    )
    connection.commit()
    connection.close()
    with pytest.raises(AUDIT.CohortAuditError, match="independently replayed field mismatch"):
        AUDIT.build_audit(ours_root, pva_root)


def test_pva_database_receipt_sha_drift_fails_closed(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    receipt_path = pva_root / "full_release_receipt.json"
    receipt = json.loads(receipt_path.read_text())
    receipt["result_database_sha256"] = "f" * 64
    receipt.pop("receipt_content_sha256")
    receipt = _self_hashed(receipt, "receipt_content_sha256")
    _write_json(receipt_path, receipt)

    with pytest.raises(
        AUDIT.CohortAuditError, match="database SHA256 disagrees with receipt"
    ):
        AUDIT.build_audit(ours_root, pva_root)


def test_protocol_drift_fails_closed(tmp_path: Path) -> None:
    ours_root, pva_root = _build_fixture(tmp_path)
    manifest_path = pva_root / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest.pop("manifest_content_sha256")
    manifest["protocol"]["table4_sobol_seed"] = 7
    manifest = _self_hashed(manifest, "manifest_content_sha256")
    _write_json(manifest_path, manifest)
    receipt_path = pva_root / "full_release_receipt.json"
    receipt = json.loads(receipt_path.read_text())
    receipt.pop("receipt_content_sha256")
    receipt["execution_manifest_sha256"] = AUDIT.sha256_file(manifest_path)
    receipt = _self_hashed(receipt, "receipt_content_sha256")
    _write_json(receipt_path, receipt)

    with pytest.raises(AUDIT.CohortAuditError, match="protocol alignment failed"):
        AUDIT.build_audit(ours_root, pva_root)
