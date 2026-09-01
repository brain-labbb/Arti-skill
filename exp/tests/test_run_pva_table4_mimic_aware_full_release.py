from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sqlite3
import sys
from typing import Any
import zlib

import pytest


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "run_pva_table4_mimic_aware_full_release.py"
)
spec = importlib.util.spec_from_file_location("pva_table4_mimic_aware_test", SCRIPT)
assert spec and spec.loader
runner = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = runner
spec.loader.exec_module(runner)

import check_pva_table4_mimic_aware_full_release as checker  # noqa: E402


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixture_row(tmp_path: Path) -> dict[str, Any]:
    package = tmp_path / "package"
    package.mkdir()
    urdf = package / "model.urdf"
    urdf.write_text(
        """<?xml version="1.0"?>
<robot name="mimic_fixture">
  <link name="base"/>
  <link name="driver_link"/>
  <link name="follower_link"/>
  <joint name="driver" type="revolute">
    <parent link="base"/><child link="driver_link"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="follower" type="revolute">
    <parent link="driver_link"/><child link="follower_link"/>
    <limit lower="-0.4" upper="0.6" effort="1" velocity="1"/>
    <mimic joint="driver" multiplier="0.5" offset="0.1"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )
    package_files, package_binding = runner.pva_run.common._package_binding(package)
    return {
        "ordinal": 0,
        "asset_id": "fixture/seed_0000",
        "category": "fixture",
        "raw_category": "fixture",
        "joint_count": 2,
        "source_path": str(package),
        "primary_urdf_path": str(urdf),
        "primary_urdf_relative_path": "model.urdf",
        "primary_urdf_sha256": _sha(urdf),
        "package_files": package_files,
        "package_binding_sha256": package_binding,
    }


def _source_evaluation(tmp_path: Path, row: dict[str, Any]) -> Path:
    source = tmp_path / "source_evaluation"
    source.mkdir()
    roster = {
        "schema_version": "pva_table1234_full_release_roster_v1",
        "N_eval": 1,
        "J_eval": 2,
        "source_bindings": {
            "extracted_root": str(Path(row["source_path"]).parent),
        },
    }
    roster["manifest_content_sha256"] = runner._self_hash(
        roster, "manifest_content_sha256"
    )
    roster_path = tmp_path / "roster_manifest.json"
    roster_path.write_text(json.dumps(roster), encoding="utf-8")
    roster_hash = _sha(roster_path)
    roster_content_hash = roster["manifest_content_sha256"]
    database = source / "results.sqlite3"
    connection = sqlite3.connect(database)
    connection.executescript(
        """
        CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);
        CREATE TABLE assets(
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            category TEXT NOT NULL,
            joint_count INTEGER NOT NULL,
            row_sha256 TEXT NOT NULL,
            row_json TEXT NOT NULL
        );
        """
    )
    meta = {
        "schema_version": runner.pva_run.RESULT_DB_SCHEMA_VERSION,
        "asset_import_state": "COMPLETE",
        "selected_n": 1,
        "selected_j": 2,
        "selected_category_count": 1,
        "roster_manifest_sha256": roster_hash,
        "roster_manifest_content_sha256": roster_content_hash,
    }
    connection.executemany(
        "INSERT INTO meta(key, value) VALUES(?, ?)",
        [(key, runner._canonical_text(value)) for key, value in meta.items()],
    )
    row_text = runner._canonical_text(row)
    connection.execute(
        "INSERT INTO assets VALUES(?, ?, ?, ?, ?, ?)",
        (
            0,
            row["asset_id"],
            row["raw_category"],
            row["joint_count"],
            hashlib.sha256(row_text.encode("utf-8")).hexdigest(),
            row_text,
        ),
    )
    connection.commit()
    connection.close()

    receipt = {
        "schema_version": runner.pva_run.RECEIPT_SCHEMA_VERSION,
        "N_eval": 1,
        "J_eval": 2,
        "eval_category_count": 1,
        "roster_manifest": str(roster_path),
        "roster_manifest_sha256": roster_hash,
        "roster_manifest_content_sha256": roster_content_hash,
        "result_database": "results.sqlite3",
        "result_database_sha256": _sha(database),
    }
    receipt["receipt_content_sha256"] = runner._self_hash(
        receipt, "receipt_content_sha256"
    )
    (source / "full_release_receipt.json").write_text(
        json.dumps(receipt), encoding="utf-8"
    )
    return source


def test_build_job_freezes_mimic_aware_independent_denominator(tmp_path: Path) -> None:
    job = runner._build_job(_fixture_row(tmp_path))

    assert job["sampling_protocol"] == runner.SAMPLING_PROTOCOL
    assert job["protocol_id"] == runner.PROTOCOL_ID
    assert job["expected_movable_joints"] == 2
    assert job["independent_dof_count"] == 1
    assert job["range_evaluable_independent_dof_count"] == 1
    assert job["mimic_joint_count"] == 1
    assert job["single_state_expected"] == 21
    assert job["sobol_state_expected"] == 64
    assert isinstance(job["joint_sampling_plan_sha256"], str)


def test_result_binding_rejects_plan_or_identity_drift(tmp_path: Path) -> None:
    job = runner._build_job(_fixture_row(tmp_path))
    result = runner.table4._empty_record(job, "fixture")
    result["state_records_count"] = 0
    assert runner._validate_result(result, job)["dataset_id"] == job["dataset_id"]

    changed = dict(result)
    changed["joint_sampling_plan_sha256"] = "0" * 64
    with pytest.raises(ValueError, match="joint_sampling_plan_sha256"):
        runner._validate_result(changed, job)


def test_mirror_job_reads_physical_package_but_keeps_logical_identity(
    tmp_path: Path,
) -> None:
    row = _fixture_row(tmp_path)
    source = _source_evaluation(tmp_path, row)
    logical_job = runner._build_job(row)
    mirror_root = tmp_path / "mirror"
    shutil.copytree(Path(row["source_path"]), mirror_root / "package")
    context, connection = runner._source_context(source)
    try:
        binding = runner._package_root_binding(context, mirror_root)
    finally:
        connection.close()
    assert binding is not None
    execution_paths = runner._verified_execution_paths(row, binding)
    assert execution_paths is not None
    shutil.rmtree(Path(row["source_path"]))
    mirror_job = runner._build_job(row, binding, execution_paths)

    assert mirror_job["input_identity_sha256"] == logical_job["input_identity_sha256"]
    assert mirror_job["package"] == logical_job["package"]
    assert mirror_job["urdf_path"] == logical_job["urdf_path"]
    assert mirror_job["evaluation_package_path"] == str(
        (mirror_root / "package").resolve()
    )
    assert mirror_job["evaluation_urdf_path"] == str(
        (mirror_root / "package" / "model.urdf").resolve()
    )
    assert mirror_job["execution_input_sha256"]

    result = runner.table4.evaluate_job(mirror_job)
    assert result["status"] == "blocked"
    assert result["package"] == logical_job["package"]
    assert result["urdf_path"] == logical_job["urdf_path"]
    assert result["input_identity_sha256"] == logical_job["input_identity_sha256"]
    assert result["execution_input_sha256"] == mirror_job["execution_input_sha256"]


def test_mirror_package_drift_is_retained_fail_closed(tmp_path: Path) -> None:
    row = _fixture_row(tmp_path)
    source = _source_evaluation(tmp_path, row)
    mirror_root = tmp_path / "mirror"
    shutil.copytree(Path(row["source_path"]), mirror_root / "package")
    (mirror_root / "package" / "extra.bin").write_bytes(b"drift")
    context, connection = runner._source_context(source)
    try:
        binding = runner._package_root_binding(context, mirror_root)
    finally:
        connection.close()
    assert binding is not None

    payload = runner._evaluate_row(row, tmp_path / "output", 30.0, binding)
    record = json.loads(payload["record_json"])
    assert payload["worker_status"] == "package_error"
    assert record["package_binding_verified"] is False
    assert record["strict_collision_pass"] is None
    assert "package binding drift" in record["issues"][0]
    assert record["package_root_binding_content_sha256"] == binding[
        "binding_content_sha256"
    ]


def test_mirror_mapping_rejects_symlinks_and_logical_escape(tmp_path: Path) -> None:
    row = _fixture_row(tmp_path)
    source = _source_evaluation(tmp_path, row)
    real_mirror = tmp_path / "real_mirror"
    shutil.copytree(Path(row["source_path"]), real_mirror / "package")
    mirror_link = tmp_path / "mirror_link"
    mirror_link.symlink_to(real_mirror, target_is_directory=True)
    context, connection = runner._source_context(source)
    try:
        with pytest.raises(Exception, match="symlink"):
            runner._package_root_binding(context, mirror_link)
        binding = runner._package_root_binding(context, real_mirror)
    finally:
        connection.close()
    assert binding is not None

    escaped = dict(row)
    escaped["source_path"] = str(tmp_path.parent / "outside" / "package")
    escaped["primary_urdf_path"] = str(
        Path(escaped["source_path"]) / "model.urdf"
    )
    with pytest.raises(ValueError, match="escapes"):
        runner._mapped_execution_paths(escaped, binding)

    shutil.rmtree(real_mirror / "package")
    (real_mirror / "package").symlink_to(
        Path(row["source_path"]), target_is_directory=True
    )
    with pytest.raises(Exception, match="symlink"):
        runner._verified_execution_paths(row, binding)


def test_prefix_validation_rejects_gaps_and_source_identity_drift() -> None:
    source = sqlite3.connect(":memory:")
    output = sqlite3.connect(":memory:")
    source.execute(
        "CREATE TABLE assets(ordinal INTEGER PRIMARY KEY, asset_id TEXT NOT NULL)"
    )
    output.execute(
        "CREATE TABLE results(ordinal INTEGER PRIMARY KEY, asset_id TEXT NOT NULL)"
    )
    source.executemany(
        "INSERT INTO assets VALUES(?, ?)", [(0, "a"), (1, "b"), (2, "c")]
    )
    output.executemany("INSERT INTO results VALUES(?, ?)", [(0, "a"), (1, "b")])
    assert runner._validated_result_prefix(output, source) == 2

    output.execute("DELETE FROM results WHERE ordinal=0")
    with pytest.raises(ValueError, match="contiguous prefix"):
        runner._validated_result_prefix(output, source)


def test_smoke_run_publishes_separate_resumable_v2_receipt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    row = _fixture_row(tmp_path)
    source = _source_evaluation(tmp_path, row)
    output = tmp_path / "v2_output"

    def fake_evaluate(
        frozen_row: dict[str, Any], _output: Path, _timeout: float
    ) -> dict[str, Any]:
        job = runner._build_job(frozen_row)
        record = runner.table4._empty_record(job, "no_native_collision_geometry", status="blocked")
        record.update(
            {
                "state_records_count": 0,
                "category": frozen_row["raw_category"],
                "package_binding_verified": True,
                "package_binding_sha256": frozen_row["package_binding_sha256"],
                "roster_ordinal": frozen_row["ordinal"],
            }
        )
        # Legacy v2 records predate the explicit fixed-root partition field.
        record.pop("fixed_root_joint_count")
        return {
            "ordinal": 0,
            "asset_id": frozen_row["asset_id"],
            "record_json": runner._canonical_text(record),
            "states_zlib": zlib.compress(b""),
            "state_count": 0,
            "worker_status": "completed",
            "worker_seconds": 0.01,
            "completed_at_utc": runner.pva_run.utc_now(),
        }

    monkeypatch.setattr(runner, "_evaluate_row", fake_evaluate)
    monkeypatch.setattr(
        runner.table4,
        "runtime_identity",
        lambda: {
            "python_executable": sys.executable,
            "runner_sha256": runner._sha256_file(Path(runner.table4.__file__)),
            "collision_core_sha256": runner._sha256_file(runner.table4.CORE_SCRIPT),
        },
    )
    runner.run_pva_table4_v2(source, output, workers=1, batch_size=1)
    runner.run_pva_table4_v2(
        source, output, workers=1, batch_size=1, resume=True
    )

    receipt = json.loads((output / "full_release_receipt.json").read_text())
    summary = json.loads((output / "summary.json").read_text())
    published_record = json.loads((output / "records.jsonl").read_text())
    assert receipt["sampling_protocol"] == runner.SAMPLING_PROTOCOL
    assert receipt["source"]["source_evaluation"] == str(source.resolve())
    assert summary["independent_dof_count"] == 1
    assert summary["mimic_joint_count"] == 1
    assert summary["fixed_root_joint_count"] == 0
    assert "fixed_root_joint_count" not in published_record
    assert (output / "records.jsonl").read_text().count("\n") == 1
    assert (output / "state_records.jsonl").read_bytes() == b""
    acceptance = checker.check(output)
    assert acceptance["all_pass"] is True
    assert acceptance["independent_dof_count"] == 1


def test_override_run_freezes_manifest_database_receipt_and_refuses_retarget(
    tmp_path: Path,
) -> None:
    row = _fixture_row(tmp_path)
    source = _source_evaluation(tmp_path, row)
    mirror_a = tmp_path / "mirror_a"
    mirror_b = tmp_path / "mirror_b"
    shutil.copytree(Path(row["source_path"]), mirror_a / "package")
    shutil.copytree(Path(row["source_path"]), mirror_b / "package")
    output = tmp_path / "mirror_output"

    runner.run_pva_table4_v2(
        source,
        output,
        workers=1,
        batch_size=1,
        package_root_override=mirror_a,
    )

    manifest = json.loads((output / "manifest.json").read_text())
    receipt = json.loads((output / "full_release_receipt.json").read_text())
    record = json.loads((output / "records.jsonl").read_text())
    binding = manifest["package_root_binding"]
    assert binding["logical_root"] == str(tmp_path)
    assert binding["physical_root"] == str(mirror_a.resolve())
    assert binding["binding_content_sha256"] == runner._self_hash(
        binding, "binding_content_sha256"
    )
    assert receipt["package_root_binding"] == binding
    database = sqlite3.connect(output / "results.sqlite3")
    try:
        meta = runner._decoded_meta(database)
    finally:
        database.close()
    assert meta["package_root_binding_content_sha256"] == binding[
        "binding_content_sha256"
    ]
    assert record["package"] == str(Path(row["source_path"]).resolve())
    assert record["urdf_path"] == str(Path(row["primary_urdf_path"]).resolve())
    assert record["package_binding_verified"] is True
    assert record["package_root_binding_content_sha256"] == binding[
        "binding_content_sha256"
    ]
    assert record["execution_input_sha256"]
    assert str(mirror_a.resolve()) not in (output / "records.jsonl").read_text()

    with pytest.raises(ValueError, match="package_root_binding"):
        runner.run_pva_table4_v2(
            source,
            output,
            workers=1,
            batch_size=1,
            resume=True,
            package_root_override=mirror_b,
        )
