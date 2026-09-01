from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sqlite3
import zlib

import pytest


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts" / "reaggregate_pva_table4_status_aware_metrics.py"
spec = importlib.util.spec_from_file_location("pva_status_metrics_test", SCRIPT)
assert spec and spec.loader
metrics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def _self_hash(value, field):
    payload = dict(value)
    payload.pop(field, None)
    return hashlib.sha256(_canonical(payload).encode()).hexdigest()


def _state(
    asset_id,
    order,
    plan,
    phase,
    sample,
    *,
    all_illegal,
    non_illegal,
    metric,
    raw,
    joint=None,
):
    return {
        "schema_version": "table4_state_v3",
        "protocol_id": metrics.PROTOCOL_ID,
        "dataset": "pva",
        "dataset_id": asset_id,
        "order": order,
        "category": "fixture",
        "sampling_protocol": "mimic_aware_independent_sampling_v2",
        "joint_sampling_plan_sha256": plan,
        "input_identity_sha256": "input",
        "joint_values_sha256": "values",
        "phase": phase,
        "sample_index": sample,
        "joint_name": joint,
        "all_pair_contact_count": max(1, all_illegal),
        "all_pair_illegal_penetration_count": all_illegal,
        "all_pair_max_penetration_m": metric,
        "non_adjacent_contact_count": max(1, non_illegal),
        "non_adjacent_illegal_penetration_count": non_illegal,
        "non_adjacent_max_penetration_m": metric if phase != "rest" else 0.0,
        "metric_max_penetration_m": metric,
        "raw_all_pair_max_penetration_m": raw,
        "raw_non_adjacent_max_penetration_m": raw if phase != "rest" else 0.0,
        "reset_readback_max_abs_error": 0.0,
    }


def _record(
    order,
    asset_id,
    *,
    status="completed",
    independent=1,
    range_dof=1,
    native_collision=1,
    measurement=True,
    issues=None,
    states_count=86,
    strict=False,
):
    return {
        "schema_version": "table4_full_release_run_v3",
        "protocol_id": metrics.PROTOCOL_ID,
        "dataset": "pva",
        "dataset_id": asset_id,
        "order": order,
        "category": "fixture",
        "status": status,
        "issues": issues or [],
        "expected_movable_joints": independent,
        "movable_dof_count": independent,
        "independent_dof_count": independent,
        "range_evaluable_independent_dof_count": range_dof,
        "mimic_joint_count": 0,
        "native_mimic_joint_count": 0,
        "external_joint_constraint_count": 0,
        "fixed_root_joint_count": 0,
        "joint_sampling_plan_sha256": "plan0",
        "input_identity_sha256": "input",
        "rest_state_expected": 1,
        "rest_state_executed": 1 if measurement else 0,
        "single_state_expected": 21,
        "single_state_executed": 21 if measurement else 0,
        "sobol_state_expected": 64,
        "sobol_state_executed": 64 if measurement else 0,
        "native_collision_elements": native_collision,
        "collision_metric_status": "MEASURED" if native_collision else "N/E",
        "measurement_complete": measurement,
        "package_binding_verified": measurement,
        "load_success": measurement,
        "runtime_identity": {} if measurement else None,
        "rest_all_pair_cf": False,
        "rest_non_adjacent_cf": True,
        "single_joint_sweep_cf": False,
        "multi_joint_sobol_cf": True,
        "strict_collision_pass": strict,
        "joint_single_sweep_cf_passed": 0,
        "rest_non_adjacent_free": 1 if measurement else 0,
        "single_non_adjacent_free": 20 if measurement else 0,
        "sobol_non_adjacent_free": 64 if measurement else 0,
        "max_penetration_m": 0.002 if measurement else None,
        "max_penetration_normalized": 0.2 if measurement else None,
        "object_bbox_diagonal_m": 0.01 if measurement else None,
        "state_records_count": states_count,
        "state_records_sha256": "unused",
        "zero_margin_oracle_sha256": "0" * 64,
    }


def _make_fixture(tmp_path: Path) -> Path:
    root = tmp_path / "sealed"
    root.mkdir()
    asset0 = "PV-A/fixture/seed_0000"
    asset1 = "PV-A/fixture/seed_0001"
    states0 = [
        _state(
            asset0,
            0,
            "plan0",
            "rest",
            0,
            all_illegal=1,
            non_illegal=0,
            metric=0.002,
            raw=0.003,
        )
    ]
    states0.extend(
        _state(
            asset0,
            0,
            "plan0",
            "single_joint_sweep",
            sample,
            all_illegal=0,
            non_illegal=1 if sample == 0 else 0,
            metric=0.0002 if sample == 0 else 0.0,
            raw=0.0012 if sample == 0 else 0.0,
            joint="hinge",
        )
        for sample in range(21)
    )
    states0.extend(
        _state(
            asset0,
            0,
            "plan0",
            "multi_joint_sobol",
            sample,
            all_illegal=0,
            non_illegal=0,
            metric=0.0,
            raw=0.001,
        )
        for sample in range(64)
    )
    payload0 = ("\n".join(_canonical(state) for state in states0) + "\n").encode()
    record0 = _record(0, asset0)
    record0["state_records_sha256"] = hashlib.sha256(
        _canonical(states0).encode()
    ).hexdigest()
    record1 = _record(
        1,
        asset1,
        status="error",
        independent=1,
        range_dof=0,
        native_collision=1,
        measurement=False,
        issues=["error: GetBasePositionAndOrientation failed."],
        states_count=0,
    )
    record1["state_records_sha256"] = hashlib.sha256(b"[]").hexdigest()
    database = root / "results.sqlite3"
    connection = sqlite3.connect(database)
    connection.executescript(
        """
        CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);
        CREATE TABLE results(
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            record_json TEXT NOT NULL,
            states_zlib BLOB NOT NULL,
            state_count INTEGER NOT NULL,
            worker_status TEXT NOT NULL,
            worker_seconds REAL NOT NULL,
            completed_at_utc TEXT NOT NULL
        );
        """
    )
    meta = {
        "schema_version": metrics.RESULT_DB_SCHEMA,
        "manifest_content_sha256": "manifest-content",
        "N_eval": 2,
        "J_eval": 2,
        "protocol_id": metrics.PROTOCOL_ID,
    }
    connection.executemany(
        "INSERT INTO meta VALUES (?, ?)", [(k, _canonical(v)) for k, v in meta.items()]
    )
    connection.executemany(
        "INSERT INTO results VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
        [
            (
                0,
                asset0,
                _canonical(record0),
                zlib.compress(payload0),
                len(states0),
                "completed",
                1.0,
                "now",
            ),
            (
                1,
                asset1,
                _canonical(record1),
                zlib.compress(b""),
                0,
                "completed",
                1.0,
                "now",
            ),
        ],
    )
    connection.commit()
    connection.close()

    # Small placeholder artifacts satisfy the sealed artifact closure.  The
    # reaggregator intentionally derives metrics from SQLite, not these files.
    for name, content in (
        ("protocol_snapshot.md", "fixture\n"),
        ("records.jsonl", "\n"),
        ("asset_records.jsonl", "\n"),
        ("state_records.jsonl", "\n"),
        ("summary.md", "fixture\n"),
    ):
        (root / name).write_text(content, encoding="utf-8")
    summary = {
        "schema_version": "fixture",
        "status_counts": {"completed": 1, "error": 1},
        "declared_dof_count": 2,
        "independent_dof_count": 2,
        "range_evaluable_independent_dof_count": 1,
        "mimic_joint_count": 0,
        "external_joint_constraint_count": 0,
        "fixed_root_joint_count": 0,
        "expected_states": {"rest": 2, "single": 42, "sobol": 128},
        "executed_states": {"rest": 1, "single": 21, "sobol": 64},
        "collision_geometry_assets": 2,
        "metrics": {
            "rest_all_pair_cf": {"passed": 0},
            "rest_non_adjacent_cf": {"passed": 1},
            "single_joint_sweep_cf": {"passed": 0},
            "multi_joint_sobol_cf": {"passed": 1},
            "strict_collision_pass": {"passed": 0},
            "collision_free_range": {"passed_states": 20},
            "collision_state_rate": {"collision_states": 87},
            "max_penetration": {"maximum_observed_normalized": 0.2},
        },
        "summary_content_sha256": "",
    }
    summary["summary_content_sha256"] = _self_hash(summary, "summary_content_sha256")
    (root / "summary.json").write_text(_canonical(summary) + "\n", encoding="utf-8")
    checkpoint = {
        "schema_version": metrics.CHECKPOINT_SCHEMA,
        "protocol_id": metrics.PROTOCOL_ID,
        "state": "complete",
        "records": 2,
        "manifest_content_sha256": "manifest-content",
        "checkpoint_content_sha256": "",
    }
    checkpoint["checkpoint_content_sha256"] = _self_hash(
        checkpoint, "checkpoint_content_sha256"
    )
    (root / "checkpoint.json").write_text(
        _canonical(checkpoint) + "\n", encoding="utf-8"
    )
    manifest = {
        "schema_version": metrics.RUN_SCHEMA,
        "protocol_id": metrics.PROTOCOL_ID,
        "N_eval": 2,
        "J_eval": 2,
        "manifest_content_sha256": "",
    }
    manifest["manifest_content_sha256"] = _self_hash(
        manifest, "manifest_content_sha256"
    )
    # The DB meta binds the manifest content, so update it after the manifest
    # is defined and rewrite the DB before computing its receipt hash.
    connection = sqlite3.connect(database)
    connection.execute(
        "UPDATE meta SET value=? WHERE key='manifest_content_sha256'",
        (_canonical(manifest["manifest_content_sha256"]),),
    )
    connection.commit()
    connection.close()
    checkpoint["manifest_content_sha256"] = manifest["manifest_content_sha256"]
    checkpoint["checkpoint_content_sha256"] = _self_hash(
        checkpoint, "checkpoint_content_sha256"
    )
    (root / "checkpoint.json").write_text(
        _canonical(checkpoint) + "\n", encoding="utf-8"
    )
    (root / "manifest.json").write_text(_canonical(manifest) + "\n", encoding="utf-8")

    artifact_rows = []
    for path in sorted(root.iterdir()):
        if path.name in {"full_release_receipt.json", "artifact_manifest.json"}:
            continue
        artifact_rows.append(
            {
                "path": path.name,
                "bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
        )
    artifact = {
        "schema_version": metrics.ARTIFACT_SCHEMA,
        "artifacts": artifact_rows,
        "artifact_manifest_content_sha256": "",
    }
    artifact["artifact_manifest_content_sha256"] = _self_hash(
        artifact, "artifact_manifest_content_sha256"
    )
    (root / "artifact_manifest.json").write_text(
        _canonical(artifact) + "\n", encoding="utf-8"
    )
    receipt = {
        "schema_version": metrics.RECEIPT_SCHEMA,
        "protocol_id": metrics.PROTOCOL_ID,
        "N_eval": 2,
        "J_eval": 2,
        "manifest": "manifest.json",
        "manifest_sha256": hashlib.sha256(
            (root / "manifest.json").read_bytes()
        ).hexdigest(),
        "summary": "summary.json",
        "summary_sha256": hashlib.sha256(
            (root / "summary.json").read_bytes()
        ).hexdigest(),
        "artifact_manifest": "artifact_manifest.json",
        "artifact_manifest_sha256": hashlib.sha256(
            (root / "artifact_manifest.json").read_bytes()
        ).hexdigest(),
        "result_database": "results.sqlite3",
        "result_database_sha256": hashlib.sha256(database.read_bytes()).hexdigest(),
        "receipt_content_sha256": "",
    }
    receipt["receipt_content_sha256"] = _self_hash(receipt, "receipt_content_sha256")
    (root / "full_release_receipt.json").write_text(
        _canonical(receipt) + "\n", encoding="utf-8"
    )
    return root


def test_reaggregate_reports_layers_transitions_and_errors(tmp_path: Path) -> None:
    root = _make_fixture(tmp_path)
    report = metrics.reaggregate(root)
    assert report["status"] == "COMPLETE"
    assert report["coverage"]["release"]["assets"] == 2
    assert report["coverage"]["observed"]["assets"] == 1
    assert report["coverage"]["collision_measured"]["assets"] == 1
    assert report["metrics"]["rest_adjacent_only"]["release"]["passed"] == 1
    transition = report["metrics"]["motion_transition"]
    assert transition["categories"]["rest_pass_motion_fail"]["passed"] == 1
    assert report["error_taxonomy"]["counts"]["backend_capacity"] == 1
    assert report["metrics"]["severity"]["asset_max_penetration_mm"]["quantiles"][
        "max"
    ] == pytest.approx(2.0)
    assert report["metrics"]["dof_bins"]["1"]["safe_dof_retention"]["denominator"] == 2
    assert report["metrics"]["category_macro"]["category_count"] == 1


def test_asset_rate_lower_tail_and_reset_readback_taxonomy() -> None:
    quantiles = metrics._asset_rate_quantiles([0.0, 0.25, 0.5, 0.75, 1.0])
    assert quantiles["min"] == 0.0
    assert quantiles["p10"] == pytest.approx(0.1)
    assert quantiles["p50"] == 0.5
    assert quantiles["max"] == 1.0

    record = {
        "status": "error",
        "issues": ["RuntimeError: reset/readback error 0.008 exceeds 1e-09"],
        "measurement_complete": False,
    }
    assert metrics._classify_error(record, "completed", 42) == "reset_readback_partial"


def test_reaggregate_rejects_database_hash_drift(tmp_path: Path) -> None:
    root = _make_fixture(tmp_path)
    with (root / "results.sqlite3").open("ab") as stream:
        stream.write(b"tampered")
    with pytest.raises(
        metrics.ReaggregateError,
        match="(database SHA-256 mismatch|artifact row .* byte count mismatch)",
    ):
        metrics.reaggregate(root)


def test_reaggregate_allows_shared_memory_sidecar(tmp_path: Path) -> None:
    root = _make_fixture(tmp_path)
    (root / "results.sqlite3-shm").write_bytes(b"shared-memory")
    # A non-empty WAL/journal remains prohibited, while an shm sidecar is
    # allowed because SQLite may create it during an immutable read.
    report = metrics.reaggregate(root)
    assert report["source"]["database_sha256"]
