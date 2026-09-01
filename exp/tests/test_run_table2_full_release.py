from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import run_table2_full_release as runner
import table123_full_release_common as common


URDF = """<robot name="fixture">
  <link name="base"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><collision><geometry><box size="1 1 1"/></geometry></collision></link>
</robot>
"""


def _package(root: Path, asset_id: str, *, malformed: bool = False) -> dict:
    package = root / asset_id
    package.mkdir(parents=True)
    urdf = package / "model.urdf"
    urdf.write_text("<robot>" if malformed else URDF, encoding="utf-8")
    return {
        "asset_id": asset_id,
        "source_path": str(package),
        "source_relative_path": asset_id,
        "primary_urdf_relative_path": "model.urdf",
        "primary_urdf_path": str(urdf),
        "raw_category": "fixture",
        "non_fixed_joints": [],
    }


def _roster(tmp_path: Path, rows: list[dict]) -> Path:
    path = tmp_path / "full_release_manifest.json"
    common.freeze_roster(rows, path, dataset="Fixture", source_bindings=[])
    return path


def test_audit_row_binds_roster_identity_and_keeps_core_metrics(tmp_path: Path) -> None:
    row = _package(tmp_path / "source", "asset-a")
    record = runner.audit_row(row, run_standard_parser=False)
    assert record["asset_id"] == "asset-a"
    assert record["status"] == "completed"
    assert record["raw_category"] == "fixture"
    assert record["primary_urdf_sha256"] == common.sha256_file(Path(row["primary_urdf_path"]))
    assert record["metrics"]["parse_rate"]["pass"] is True


def test_audit_row_uses_absolute_primary_binding_when_relative_path_is_source_root_relative(
    tmp_path: Path,
) -> None:
    row = _package(tmp_path / "source", "asset-root-relative")
    # Release rosters retain a portable source-root-relative path for
    # provenance, while the legacy core expects a path relative to `package`.
    row["primary_urdf_relative_path"] = "asset-root-relative/model.urdf"
    record = runner.audit_row(row, run_standard_parser=False)
    assert record["status"] == "completed"
    assert record["primary_urdf_sha256"] == common.sha256_file(Path(row["primary_urdf_path"]))


def test_audit_row_fails_closed_when_primary_urdf_changes_during_audit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    row = _package(tmp_path / "source", "asset-toctou")
    expected_hash = common.sha256_file(Path(row["primary_urdf_path"]))
    original = runner.core.audit_asset_package

    def mutate_after_audit(*args, **kwargs):
        result = original(*args, **kwargs)
        Path(row["primary_urdf_path"]).write_text("changed", encoding="utf-8")
        return result

    monkeypatch.setattr(runner.core, "audit_asset_package", mutate_after_audit)
    record = runner.audit_row(row, run_standard_parser=False)
    assert record["status"] == "error"
    assert "changed during evaluation" in record["error"]
    assert record["primary_urdf_sha256"] == expected_hash


def test_audit_row_preserves_core_failure_as_denominator_record(tmp_path: Path) -> None:
    row = _package(tmp_path / "source", "asset-b", malformed=True)
    record = runner.audit_row(row, run_standard_parser=False)
    assert record["status"] == "error"
    assert record["error"].startswith("xml_parse_failed:")
    assert record["metrics"]["strict_urdf_pass"]["pass"] is False


def test_aggregate_full_release_uses_roster_denominator_including_timeout(tmp_path: Path) -> None:
    good = _package(tmp_path / "source", "good")
    timed = _package(tmp_path / "source", "timed")
    roster = _roster(tmp_path, [good, timed])
    manifest = common.load_roster(roster)
    records = [runner.audit_row(good, run_standard_parser=False)]
    records.append(runner.failure_record(manifest["rows"][1], "asset_timeout", status="timeout"))
    summary = runner.aggregate_full_release(records, manifest)
    assert summary["n_eval"] == 2
    assert summary["records_present"] == 2
    assert summary["error_count"] == 1
    assert summary["metrics"]["parse_rate"]["denominator"] == 2


def test_run_rejects_legacy_n800_manifest(tmp_path: Path) -> None:
    path = tmp_path / "old.json"
    path.write_text(json.dumps({"schema_version": "1.0.0", "selection": {"n_eval": 800}}), encoding="utf-8")
    with pytest.raises(ValueError, match="full-release roster"):
        runner.run_full_release(path, tmp_path / "out", workers=1, timeout_seconds=1)


def test_run_full_release_is_deterministic_and_writes_checkpoint_and_artifacts(tmp_path: Path) -> None:
    source = tmp_path / "source"
    rows = [_package(source, "b"), _package(source, "a")]
    roster = _roster(tmp_path, rows)
    out = tmp_path / "out"
    result = runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False)
    assert result == out.resolve()
    records = [json.loads(line) for line in (out / "asset_records.jsonl").read_text().splitlines()]
    assert [row["asset_id"] for row in records] == ["a", "b"]
    checkpoint = json.loads((out / "checkpoint.json").read_text())
    assert checkpoint["state"] == "complete"
    common.verify_artifacts(out)


def test_resume_keeps_completed_record_bytes_and_finishes_remaining(tmp_path: Path) -> None:
    source = tmp_path / "source"
    rows = [_package(source, "a"), _package(source, "b")]
    roster = _roster(tmp_path, rows)
    out = tmp_path / "out"
    runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False)
    original = (out / "asset_records.jsonl").read_bytes()
    resumed = runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False, resume=True)
    assert resumed == out.resolve()
    assert (out / "asset_records.jsonl").read_bytes() == original


def test_run_full_release_pins_blas_environment_for_child(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    source = tmp_path / "source"
    roster = _roster(tmp_path, [_package(source, "a")])
    monkeypatch.setenv("OMP_NUM_THREADS", "99")
    out = tmp_path / "out"
    runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False)
    record = json.loads((out / "asset_records.jsonl").read_text())
    assert record["evaluation_environment"]["OMP_NUM_THREADS"] == "1"


def test_run_full_release_timeout_is_recorded_without_dropping_asset(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    source = tmp_path / "source"
    roster = _roster(tmp_path, [_package(source, "slow")])
    original = runner.audit_row

    def slow(row: dict, run_standard_parser: bool) -> dict:
        del row, run_standard_parser
        time.sleep(0.2)
        return original(_package(source, "late"), False)

    monkeypatch.setattr(runner, "audit_row", slow)
    out = tmp_path / "out"
    runner.run_full_release(roster, out, workers=1, timeout_seconds=0.05, run_standard_parser=False)
    record = json.loads((out / "asset_records.jsonl").read_text())
    assert record["asset_id"] == "slow"
    assert record["status"] == "timeout"
    assert "asset_timeout" in record["error"]


def test_run_rows_drains_large_child_result_before_process_exit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Large full-release diagnostics must not deadlock on the IPC pipe."""

    row = _package(tmp_path / "source", "large")

    def large_result(row: dict, run_standard_parser: bool) -> dict:
        del run_standard_parser
        return {"asset_id": row["asset_id"], "status": "completed", "payload": "x" * (2 * 1024 * 1024)}

    monkeypatch.setattr(runner, "audit_row", large_result)
    records: list[dict] = []
    runner._run_rows(
        [row],
        workers=1,
        timeout_seconds=5,
        run_standard_parser=False,
        on_record=records.append,
    )
    assert len(records) == 1
    assert records[0]["status"] == "completed"
    assert len(records[0]["payload"]) == 2 * 1024 * 1024


def test_run_rejects_package_hash_drift_before_audit(tmp_path: Path) -> None:
    source = tmp_path / "source"
    row = _package(source, "asset")
    roster = _roster(tmp_path, [row])
    (source / "asset" / "extra.bin").write_bytes(b"drift")
    with pytest.raises(ValueError, match="package.*binding.*drift"):
        runner.run_full_release(roster, tmp_path / "out", workers=1, timeout_seconds=1, run_standard_parser=False)


def test_run_rejects_urdf_hash_drift_before_audit(tmp_path: Path) -> None:
    source = tmp_path / "source"
    row = _package(source, "asset")
    roster = _roster(tmp_path, [row])
    Path(row["primary_urdf_path"]).write_text("changed", encoding="utf-8")
    with pytest.raises(ValueError, match="URDF hash drift"):
        runner.run_full_release(roster, tmp_path / "out", workers=1, timeout_seconds=1, run_standard_parser=False)


def test_resume_rejects_tampered_checkpoint(tmp_path: Path) -> None:
    source = tmp_path / "source"
    roster = _roster(tmp_path, [_package(source, "asset")])
    out = tmp_path / "out"
    runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False)
    checkpoint = out / "checkpoint.json"
    payload = json.loads(checkpoint.read_text(encoding="utf-8"))
    payload["state"] = "running"
    checkpoint.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="checkpoint"):
        runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False, resume=True)


def test_artifact_manifest_self_hash_is_verified_on_resume(tmp_path: Path) -> None:
    source = tmp_path / "source"
    roster = _roster(tmp_path, [_package(source, "asset")])
    out = tmp_path / "out"
    runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False)
    artifact = out / "artifact_manifest.json"
    payload = json.loads(artifact.read_text(encoding="utf-8"))
    payload["artifacts"][0]["size"] += 1
    artifact.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="artifact"):
        runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False, resume=True)


def test_resume_rejects_tampered_run_manifest_self_hash(tmp_path: Path) -> None:
    source = tmp_path / "source"
    roster = _roster(tmp_path, [_package(source, "asset")])
    out = tmp_path / "out"
    runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False)
    manifest = out / "manifest.json"
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    payload["workers"] = 2
    manifest.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="manifest.*hash"):
        runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False, resume=True)


def test_resume_rejects_tampered_asset_records_binding(tmp_path: Path) -> None:
    source = tmp_path / "source"
    roster = _roster(tmp_path, [_package(source, "asset")])
    out = tmp_path / "out"
    runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False)
    records = out / "asset_records.jsonl"
    records.write_text(records.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="(artifact|asset records hash)"):
        runner.run_full_release(roster, out, workers=1, timeout_seconds=5, run_standard_parser=False, resume=True)
