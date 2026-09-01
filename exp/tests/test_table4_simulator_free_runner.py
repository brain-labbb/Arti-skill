from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sqlite3
import sys

import pytest


EXP_ROOT = Path(__file__).parents[1]
SCRIPT_DIR = EXP_ROOT / "scripts"


def _load(name: str):
    path = SCRIPT_DIR / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


adapter = _load("run_pva_table4_simulator_free_full_release")
runner = adapter.simulator_free


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_sealed_source_evaluation(tmp_path: Path) -> tuple[Path, Path, Path]:
    source = tmp_path / "source-evaluation"
    source.mkdir()
    database = source / "results.sqlite3"
    with sqlite3.connect(database) as connection:
        connection.execute(
            "CREATE TABLE fixture (ordinal INTEGER PRIMARY KEY, payload TEXT NOT NULL)"
        )
        connection.execute("INSERT INTO fixture VALUES (0, 'sealed')")

    receipt_path = source / "full_release_receipt.json"
    receipt = {
        "schema_version": adapter.pva_run.RECEIPT_SCHEMA_VERSION,
        "result_database": "results.sqlite3",
        "result_database_sha256": _sha256(database),
        "fixture": "minimal-source-release",
    }
    receipt["receipt_content_sha256"] = adapter.v2._self_hash(
        receipt, "receipt_content_sha256"
    )
    receipt_path.write_text(
        json.dumps(receipt, sort_keys=True) + "\n",
        encoding="ascii",
    )
    return source, database, receipt_path


def _write_empty_registry(tmp_path: Path) -> Path:
    path = tmp_path / "reviewed_contacts.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": "table4_intended_contact_registry_v1",
                "policy_id": "fixture_empty_reviewed_contacts_v1",
                "entries": [],
            },
            sort_keys=True,
        ),
        encoding="ascii",
    )
    return path


def _write_one_dof_urdf(tmp_path: Path, obstacle_x_m: float) -> Path:
    package = tmp_path / "package"
    package.mkdir()
    urdf = package / "model.urdf"
    urdf.write_text(
        f"""<robot name="simulator_free_fixture">
  <link name="base"/>
  <link name="moving">
    <collision name="moving_box">
      <geometry><box size="0.2 0.2 0.2"/></geometry>
    </collision>
  </link>
  <link name="obstacle">
    <collision name="obstacle_box">
      <geometry><box size="0.2 0.2 0.2"/></geometry>
    </collision>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="moving"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="obstacle_mount" type="fixed">
    <parent link="base"/>
    <child link="obstacle"/>
    <origin xyz="{obstacle_x_m} 0 0"/>
  </joint>
</robot>
""",
        encoding="ascii",
    )
    return urdf


def _build_job(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    *,
    obstacle_x_m: float,
    asset_id: str,
) -> tuple[dict[str, object], Path]:
    registry = _write_empty_registry(tmp_path)
    registry_hash = _sha256(registry)
    monkeypatch.setattr(adapter, "_ACTIVE_CONTACT_REGISTRY", registry)
    monkeypatch.setattr(adapter, "_ACTIVE_CONTACT_REGISTRY_SHA256", registry_hash)

    urdf = _write_one_dof_urdf(tmp_path, obstacle_x_m)
    urdf_hash = _sha256(urdf)
    row = {
        "ordinal": 0,
        "asset_id": asset_id,
        "raw_category": "fixture",
        "joint_count": 1,
        "source_path": str(urdf.parent),
        "primary_urdf_path": str(urdf),
        "primary_urdf_relative_path": "model.urdf",
        "primary_urdf_sha256": urdf_hash,
        "package_binding_sha256": "fixture-package-binding",
    }
    plan = runner.sampling_core().sampling_plan_metadata(
        urdf,
        declared_dof=1,
        expected_sha256=urdf_hash,
        external_joint_constraints=[],
    )
    assert plan["sampling_plan_error"] is None
    job = adapter._job_with_plan(row, plan)
    return job, registry


@pytest.mark.parametrize(
    ("obstacle_x_m", "asset_id", "expected_strict_pass"),
    (
        (1.0, "fixture/collision_free", True),
        (0.05, "fixture/clear_penetration", False),
    ),
)
def test_full_release_job_runs_simulator_free_with_bound_evidence(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    obstacle_x_m: float,
    asset_id: str,
    expected_strict_pass: bool,
) -> None:
    job, registry = _build_job(
        tmp_path,
        monkeypatch,
        obstacle_x_m=obstacle_x_m,
        asset_id=asset_id,
    )

    result = runner.evaluate_job(job)

    assert job["protocol_id"] == adapter.PROTOCOL_ID
    assert job["evaluator_protocol_id"] == runner.PROTOCOL_ID
    assert result["protocol_id"] == adapter.PROTOCOL_ID
    assert result["evaluator_protocol_id"] == runner.PROTOCOL_ID
    assert adapter.PROTOCOL_ID != runner.PROTOCOL_ID
    assert {
        (state["protocol_id"], state["evaluator_protocol_id"])
        for state in result["state_records"]
    } == {(adapter.PROTOCOL_ID, runner.PROTOCOL_ID)}

    assert result["status"] == "completed", result["issues"]
    assert result["measurement_complete"] is True
    assert result["rest_state_executed"] == 1
    assert result["single_state_executed"] == runner.SINGLE_SAMPLES
    assert result["sobol_state_executed"] == runner.SOBOL_SAMPLES
    assert result["state_records_count"] == (
        1 + runner.SINGLE_SAMPLES + runner.SOBOL_SAMPLES
    )
    assert result["state_records_count"] == len(result["state_records"])
    assert result["state_records_sha256"] == runner.canonical_sha256(
        result["state_records"]
    )
    assert result["unexecuted_state_count"] == 0
    runner.validate_result_binding(result, job)

    runtime_hash = runner.canonical_sha256(result["runtime_identity"])
    assert job["child_runtime_identity_sha256"] == runtime_hash
    assert result["child_runtime_identity_sha256"] == runtime_hash
    assert runner.canonical_sha256(result["final_runtime_identity"]) == runtime_hash
    assert result["runtime_integrity_fatal"] is False
    assert result["execution_source_integrity_fatal"] is False

    registry_hash = _sha256(registry)
    assert job["intended_contact_registry_path"] == str(registry.resolve())
    assert job["intended_contact_registry_sha256"] == registry_hash
    assert result["intended_contact_registry_path"] == str(registry.resolve())
    assert result["intended_contact_registry_sha256"] == registry_hash
    assert result["intended_contact_policy_id"] == (
        "fixture_empty_reviewed_contacts_v1"
    )
    assert result["bound_intended_contact_registration_count"] == 0
    assert {
        state["intended_contact_registry_sha256"] for state in result["state_records"]
    } == {registry_hash}

    assert result["strict_collision_pass"] is expected_strict_pass
    assert result["raw_strict_penetration_proxy_pass"] is expected_strict_pass
    assert result["adjusted_strict_intersection_free_pass"] is expected_strict_pass
    assert result["raw_strict_intersection_free_pass"] is expected_strict_pass
    assert result["max_penetration_m"] is None
    assert result["max_penetration_normalized"] is None
    assert result["max_penetration_depth_proxy_view"] == "contact_adjusted"
    assert result["raw_max_penetration_depth_proxy_m"] == pytest.approx(
        result["adjusted_max_penetration_depth_proxy_m"]
    )
    assert result["raw_max_penetration_depth_proxy_normalized"] == pytest.approx(
        result["adjusted_max_penetration_depth_proxy_normalized"]
    )
    assert result["max_penetration_depth_proxy_m"] == pytest.approx(
        result["adjusted_max_penetration_depth_proxy_m"]
    )
    assert result["max_penetration_depth_proxy_normalized"] == pytest.approx(
        result["adjusted_max_penetration_depth_proxy_normalized"]
    )
    assert result["runtime_identity"]["simulator_free_runtime_verified"] is True
    assert result["runtime_identity"]["simulator_modules_loaded"] == []

    summary = runner.aggregate_records([result], n_eval=1, j_eval=1)
    metrics = summary["metrics"]
    expected_collision_states = (
        0 if expected_strict_pass else result["state_records_count"]
    )
    expected_free_single_states = runner.SINGLE_SAMPLES if expected_strict_pass else 0
    assert metrics["collision_state_rate"] == {
        "collision_states": expected_collision_states,
        "denominator": result["state_records_count"],
        "rate": expected_collision_states / result["state_records_count"],
        "executed_states": result["state_records_count"],
        "unexecuted_states": 0,
    }
    assert metrics["raw_collision_state_rate"] == metrics["collision_state_rate"]
    assert metrics["collision_free_range"] == {
        "passed_states": expected_free_single_states,
        "denominator": runner.SINGLE_SAMPLES,
        "rate": expected_free_single_states / runner.SINGLE_SAMPLES,
    }
    assert metrics["raw_collision_free_range"] == metrics["collision_free_range"]
    assert metrics["max_penetration"] == {
        "status": "N/E",
        "reason": "arbitrary_triangle_mesh_global_mtd_not_available",
    }
    assert (
        metrics["max_penetration_depth_proxy"]
        == metrics["adjusted_max_penetration_depth_proxy"]
    )
    assert metrics["raw_max_penetration_depth_proxy"]["view"] == "raw"
    assert metrics["adjusted_max_penetration_depth_proxy"]["view"] == (
        "contact_adjusted"
    )
    assert metrics["raw_max_penetration_depth_proxy"][
        "maximum_observed_normalized"
    ] == pytest.approx(
        metrics["adjusted_max_penetration_depth_proxy"]["maximum_observed_normalized"]
    )

    if expected_strict_pass:
        assert result["max_penetration_depth_proxy_m"] == 0.0
        assert all(
            state["non_adjacent_intersection_count"] == 0
            for state in result["state_records"]
        )
    else:
        assert result["max_penetration_depth_proxy_m"] > 0.1
        assert all(
            state["non_adjacent_intersection_count"] > 0
            for state in result["state_records"]
        )
        assert all(
            state["raw_non_adjacent_illegal_penetration_count"] > 0
            for state in result["state_records"]
        )


def test_output_lock_is_nonblocking_and_reusable(tmp_path: Path) -> None:
    output = tmp_path / "v4-results"

    with adapter._output_lock(output) as locked:
        assert locked == output.resolve()
        with pytest.raises(RuntimeError, match="another v4 run owns output lock"):
            with adapter._output_lock(output):
                pass

    with adapter._output_lock(output) as reacquired:
        assert reacquired == output.resolve()


def test_zero_independent_dof_range_is_not_evaluable_in_report() -> None:
    record = {
        "status": "completed",
        "native_collision_elements": 1,
        "measurement_complete": True,
        "runtime_integrity_fatal": False,
        "execution_source_integrity_fatal": False,
        "independent_dof_count": 0,
        "rest_state_expected": 1,
        "rest_state_executed": 1,
        "single_state_expected": 0,
        "single_state_executed": 0,
        "sobol_state_expected": 0,
        "sobol_state_executed": 0,
        "raw_rest_all_pair_cf": True,
        "rest_all_pair_cf": True,
        "raw_rest_non_adjacent_cf": True,
        "rest_non_adjacent_cf": True,
        "raw_rest_non_adjacent_intersection_free": True,
        "adjusted_rest_non_adjacent_intersection_free": True,
        "raw_rest_non_adjacent_free": 1,
        "rest_non_adjacent_free": 1,
        "raw_max_penetration_depth_proxy_normalized": 0.0,
        "adjusted_max_penetration_depth_proxy_normalized": 0.0,
    }

    summary = runner.aggregate_records([record], n_eval=1, j_eval=0)
    report = runner.report_text(summary, "zero-dof-fixture")

    for key in ("raw_collision_free_range", "collision_free_range"):
        assert summary["metrics"][key] == {
            "passed_states": 0,
            "denominator": 0,
            "rate": None,
            "status": "N/E",
            "reason": "cohort_has_no_independent_single_joint_states",
        }
    assert "| Raw collision-free range | N/E |" in report
    assert "| Contact-adjusted collision-free range | N/E |" in report


def test_source_database_attestation_seals_observed_database(
    tmp_path: Path,
) -> None:
    source, database, receipt = _write_sealed_source_evaluation(tmp_path)

    attestation = adapter._verify_source_database(source)

    database_hash = _sha256(database)
    assert attestation["schema_version"] == (
        "pva_table4_source_database_attestation_v1"
    )
    assert attestation["source_evaluation"] == str(source.resolve())
    assert attestation["source_receipt"] == str(receipt.resolve())
    assert attestation["source_receipt_sha256"] == _sha256(receipt)
    assert attestation["source_result_database"] == str(database.resolve())
    assert attestation["source_result_database_bytes"] == database.stat().st_size
    assert attestation["source_result_database_declared_sha256"] == database_hash
    assert attestation["source_result_database_observed_sha256"] == database_hash
    assert attestation["attestation_content_sha256"] == adapter.v2._self_hash(
        attestation, "attestation_content_sha256"
    )


def test_source_database_attestation_rejects_post_receipt_tampering(
    tmp_path: Path,
) -> None:
    source, database, _receipt = _write_sealed_source_evaluation(tmp_path)
    with sqlite3.connect(database) as connection:
        connection.execute("UPDATE fixture SET payload='tampered' WHERE ordinal=0")

    with pytest.raises(ValueError, match="result database SHA-256 mismatch"):
        adapter._verify_source_database(source)


def test_source_database_attestation_rejects_mutation_during_hash(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source, database, _receipt = _write_sealed_source_evaluation(tmp_path)
    database = database.resolve()
    real_sha256_file = adapter.v2._sha256_file

    def mutate_after_hash(path: Path) -> str:
        path = Path(path).resolve()
        digest = real_sha256_file(path)
        if path == database:
            with path.open("ab") as stream:
                stream.write(b"changed-during-hash")
        return digest

    monkeypatch.setattr(adapter.v2, "_sha256_file", mutate_after_hash)

    with pytest.raises(RuntimeError, match="changed while it was being hashed"):
        adapter._verify_source_database(source)


def test_source_database_attestation_rejects_nonempty_wal(tmp_path: Path) -> None:
    source, database, _receipt = _write_sealed_source_evaluation(tmp_path)
    Path(f"{database}-wal").write_bytes(b"uncheckpointed-transaction")

    with pytest.raises(ValueError, match="uncheckpointed SQLite sidecar"):
        adapter._verify_source_database(source)


def test_verified_source_connection_is_immutable_and_sidecar_free(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source, database, _receipt = _write_sealed_source_evaluation(tmp_path)
    empty_wal = Path(f"{database}-wal")
    empty_wal.write_bytes(b"")
    attestation = adapter._verify_source_database(source)
    monkeypatch.setattr(adapter, "_ACTIVE_SOURCE_DATABASE_ATTESTATION", attestation)

    connect_calls: list[tuple[str, dict[str, object]]] = []
    real_connect = adapter.sqlite3.connect

    def recording_connect(
        target: str, *args: object, **kwargs: object
    ) -> sqlite3.Connection:
        connect_calls.append((target, dict(kwargs)))
        return real_connect(target, *args, **kwargs)

    monkeypatch.setattr(adapter.sqlite3, "connect", recording_connect)

    connection = adapter._connect_verified_source(database)
    try:
        assert connection.execute("SELECT payload FROM fixture").fetchone() == (
            "sealed",
        )
        assert connection.execute("PRAGMA query_only").fetchone() == (1,)
        with pytest.raises(sqlite3.OperationalError, match="readonly"):
            connection.execute("UPDATE fixture SET payload='write-attempt'")
    finally:
        connection.close()

    assert connect_calls == [
        (
            f"{database.resolve().as_uri()}?mode=ro&immutable=1",
            {"uri": True},
        )
    ]
    assert empty_wal.is_file() and empty_wal.stat().st_size == 0
    assert not Path(f"{database}-journal").exists()
    assert not Path(f"{database}-shm").exists()


def test_v4_refuses_to_resume_or_overwrite_formally_sealed_output(
    tmp_path: Path,
) -> None:
    output = tmp_path / "sealed-output"
    output.mkdir()
    (output / "full_release_receipt.json").write_text("{}\n", encoding="ascii")

    with pytest.raises(ValueError, match="already has a formal receipt"):
        adapter._run_pva_table4_v4_locked(
            tmp_path / "missing-source",
            output,
            resume=True,
        )
