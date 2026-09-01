from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

import pytest


EXP_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = EXP_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import infinite_mobility_table123_common as common  # noqa: E402
import run_table2_infinite_mobility as runner  # noqa: E402


def _write_package(root: Path, name: str) -> tuple[Path, str]:
    package = root / name
    package.mkdir(parents=True)
    urdf = package / "nested" / "scene.urdf"
    urdf.parent.mkdir()
    urdf.write_text(
        """<robot name="fixture">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
    <inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
  </link>
  <link name="door">
    <visual><geometry><box size="0.5 0.5 0.5"/></geometry></visual>
    <collision><geometry><box size="0.5 0.5 0.5"/></geometry></collision>
    <inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
  </link>
  <joint name="mount" type="fixed"><parent link="base"/><child link="door"/></joint>
</robot>
""",
        encoding="utf-8",
    )
    return package, "nested/scene.urdf"


def _write_cohort(tmp_path: Path, count: int = 2) -> tuple[Path, dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for index in range(count):
        package, relative = _write_package(tmp_path / "packages", f"asset_{index}")
        scanned = common.scan_package(package)
        original_status = "TIMEOUT" if index == count - 1 else "PASS"
        rows.append(
            {
                "asset_id": f"Factory{index}/seed_{index:03d}",
                "factory": f"Factory{index}",
                "raw_category": f"Factory{index}",
                "seed": index,
                "original_status": original_status,
                "recovery_used": original_status == "TIMEOUT",
                "recovery_provenance": (
                    {"recovery_record_sha256": "a" * 64}
                    if original_status == "TIMEOUT"
                    else None
                ),
                "package_path": str(package.absolute()),
                "urdf_relpath": relative,
                "primary_urdf_sha256": scanned["files_by_path"][relative]["sha256"],
                "baseline_package_sha256": scanned["baseline_package_sha256"],
                "package_binding": scanned["package_binding"],
                "selection_index": index + 1,
                "source": "recovery" if original_status == "TIMEOUT" else "primary",
                "declared_joint_count_hint": 0,
            }
        )
    manifest: dict[str, Any] = {
        "schema_version": 2,
        "dataset": "Infinite Mobility",
        "release_status": "SUPPLEMENTARY_FULL_GENERATED_COHORT",
        "cohort_type": "SUPPLEMENTARY_FULL_GENERATED_COHORT_NOT_OFFICIAL_FINITE_RELEASE",
        "N_release": count,
        "N_eval": count,
        "factory_order": [row["factory"] for row in rows],
        "seeds": list(range(count)),
        "source_selection": {
            "identity_policy": "identity preservation with pre-freeze recovery overlay; no post-freeze reselection"
        },
        "source_bindings": [
            {"label": "fixture_source", "path": str(__file__), "sha256": common.sha256_file(Path(__file__))}
        ],
        "evaluation": {"fixture_upstream_evaluator": "bound"},
        "assets": rows,
    }
    manifest["manifest_content_sha256"] = common.manifest_self_hash(manifest)
    cohort_root = tmp_path / "cohort"
    cohort_root.mkdir()
    manifest_path = cohort_root / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    note = cohort_root / "cohort_protocol_snapshot.json"
    note.write_text("{}\n", encoding="utf-8")
    files = {}
    for path in (manifest_path, note):
        files[path.name] = {
            "bytes": path.stat().st_size,
            "sha256": common.sha256_file(path),
        }
    (cohort_root / "artifact_manifest.json").write_text(
        json.dumps({"schema_version": 1, "files": files}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest_path, manifest


def _args(
    cohort: Path,
    output: Path,
    *,
    resume: bool = False,
    limit: int | None = None,
) -> argparse.Namespace:
    argv = [
        "--mode", "smoke",
        "--cohort-manifest", str(cohort),
        "--output", str(output),
        "--workers", "1",
        "--asset-timeout-seconds", "20",
        "--no-standard-parser",
    ]
    if resume:
        argv.append("--resume")
    if limit is not None:
        argv.extend(["--limit", str(limit)])
    return runner.parse_args(argv)


def _trust_fixture(monkeypatch: pytest.MonkeyPatch, manifest: dict[str, Any]) -> None:
    def verify(path: Path, *, formal: bool = False) -> dict[str, Any]:
        assert Path(path).name == "manifest.json"
        assert formal is False
        return manifest

    monkeypatch.setattr(runner.common, "verify_cohort_manifest", verify)


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def _jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


@pytest.fixture(scope="module")
def canonical_formal_cohort() -> tuple[dict[str, Any], dict[str, Any]]:
    _checked, cohort, receipt = runner.load_cohort(
        runner.DEFAULT_COHORT_MANIFEST, formal=True
    )
    return cohort, receipt


def _formal_args(tmp_path: Path) -> argparse.Namespace:
    return runner.parse_args(
        [
            "--mode", "formal",
            "--cohort-manifest", str(runner.DEFAULT_COHORT_MANIFEST),
            "--output", str(tmp_path / "formal"),
        ]
    )


def _replace_formal_asset(
    cohort: dict[str, Any], index: int, **changes: Any
) -> dict[str, Any]:
    assets = list(cohort["assets"])
    assets[index] = {**assets[index], **changes}
    return {**cohort, "assets": assets}


def _prepare_completed_resume_fixture(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, name: str
) -> tuple[Path, Path, dict[str, Any]]:
    cohort_path, cohort = _write_cohort(tmp_path / name, count=1)
    _trust_fixture(monkeypatch, cohort)
    output = tmp_path / f"{name}_output"
    runner.run(_args(cohort_path, output))
    return cohort_path, output, cohort


def test_smoke_run_uses_fresh_shared_core_children_and_publishes_bound_artifacts(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path)
    _trust_fixture(monkeypatch, cohort)
    output = tmp_path / "table2"

    assert runner.run(_args(cohort_path, output)) == output.absolute()

    manifest = _json(output / "manifest.json")
    assert manifest["manifest_content_sha256"] == runner.core.manifest_self_hash(manifest)
    assert manifest["dataset"] == "Infinite Mobility"
    assert manifest["classification"] == "NON_FORMAL_SMOKE"
    assert manifest["source"]["cohort_manifest_sha256"] == common.sha256_file(cohort_path)
    assert manifest["source"]["cohort_manifest_content_sha256"] == cohort["manifest_content_sha256"]
    assert manifest["source"]["source_bindings"] == cohort["source_bindings"]
    assert manifest["source"]["cohort_evaluation_bindings"] == cohort["evaluation"]
    assert manifest["source"]["recovery_overlay_count"] == 1
    assert manifest["evaluation"]["adapter_sha256"] == common.sha256_file(runner.SCRIPT_PATH)
    assert manifest["evaluation"]["shared_core_sha256"] == common.sha256_file(runner.core.SCRIPT_PATH)
    assert manifest["evaluation"]["environment"]["shared_core_path"] == str(
        runner.SHARED_CORE_PATH
    )
    assert manifest["evaluation"]["environment"]["shared_core_sha256"] == common.sha256_file(
        runner.SHARED_CORE_PATH
    )
    retry_policy = manifest["evaluation"]["adapter_config"][
        "spawn_eagain_retry_policy"
    ]
    assert retry_policy == manifest["evaluation"]["environment"][
        "spawn_eagain_retry_policy"
    ]
    assert retry_policy["total_backoff_wait_seconds"] == 1800.0
    assert retry_policy["maximum_backoff_seconds"] == 30.0
    assert [row["selection_index"] for row in manifest["records"]] == [1, 2]
    assert [row["source"] for row in manifest["records"]] == ["primary", "recovery"]
    assert [row["declared_joint_count_hint"] for row in manifest["records"]] == [0, 0]

    records = _jsonl(output / "records.jsonl")
    assert [row["asset_id"] for row in records] == [row["asset_id"] for row in cohort["assets"]]
    assert [row["original_status"] for row in records] == ["PASS", "TIMEOUT"]
    assert all(row["result_origin"] == "child_attested" for row in records)
    assert all(
        row["job_runtime_binding"] == row["worker_runtime_binding"]
        for row in records
    )
    assert all(row["worker_evidence"]["fresh_interpreter"] is True for row in records)
    assert all(row["status"] == "completed" for row in records)
    assert all("package_binding" not in row for row in records)
    assert [row["package_content_manifest_sha256"] for row in records] == [
        row["package_binding"]["content_manifest_sha256"] for row in cohort["assets"]
    ]

    summary = _json(output / "summary.json")
    assert summary["n_eval"] == 2
    assert summary["records_present"] == 2
    assert summary["source_provenance"] == {"original_pass": 1, "recovery_overlay": 1}
    report = (output / "report.md").read_text(encoding="utf-8")
    assert "supplementary generated cohort" in report.lower()
    assert "1 recovery" in report.lower()
    assert (output / "protocol_snapshot.md").read_bytes() == runner.core.PROTOCOL_PATH.read_bytes()
    common.verify_artifacts(output)

    with pytest.raises(FileExistsError, match="already exists"):
        runner.run(_args(cohort_path, output))
    with runner.core.output_run_lock(output):
        with pytest.raises(RuntimeError, match="already locked"):
            runner.run(_args(cohort_path, output, resume=True))


def test_package_drift_becomes_failed_record_without_shrinking_denominator(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path, count=1)
    _trust_fixture(monkeypatch, cohort)
    package = Path(cohort["assets"][0]["package_path"])
    (package / "late.txt").write_text("drift\n", encoding="utf-8")
    output = tmp_path / "table2_drift"

    runner.run(_args(cohort_path, output))

    record = _jsonl(output / "records.jsonl")[0]
    assert record["status"] == "error"
    assert "source_changed_before_audit" in record["error"]
    assert all(metric["pass"] is False for metric in record["metrics"].values())
    summary = _json(output / "summary.json")
    assert summary["n_eval"] == 1
    assert summary["records_present"] == 1
    assert summary["error_count"] == 1
    assert all(item["denominator"] == 1 for item in summary["metrics"].values())


def test_spawn_eagain_retries_batch_with_same_tokens_and_no_metric_pollution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path, count=2)
    _trust_fixture(monkeypatch, cohort)
    output = tmp_path / "spawn_eagain_recovers"
    original_popen = runner.core.subprocess.Popen
    observed_tokens: dict[str, list[str]] = {}
    failures_remaining = 3

    def flaky_popen(command: list[str], *args: Any, **kwargs: Any) -> Any:
        nonlocal failures_remaining
        if "--internal-child-job" not in command:
            return original_popen(command, *args, **kwargs)
        job_path = Path(command[command.index("--internal-child-job") + 1])
        job = _json(job_path)
        observed_tokens.setdefault(job["asset_id"], []).append(job["run_token"])
        if job["asset_id"] == cohort["assets"][1]["asset_id"] and failures_remaining:
            failures_remaining -= 1
            raise BlockingIOError(11, "Resource temporarily unavailable")
        return original_popen(command, *args, **kwargs)

    monkeypatch.setattr(runner.core.subprocess, "Popen", flaky_popen)
    monkeypatch.setattr(runner, "_sleep_spawn_retry", lambda _seconds: None, raising=False)

    args = _args(cohort_path, output)
    args.workers = 2
    runner.run(args)

    records = _jsonl(output / "records.jsonl")
    assert [row["asset_id"] for row in records] == [
        row["asset_id"] for row in cohort["assets"]
    ]
    assert all(row["result_origin"] == "child_attested" for row in records)
    assert all("BlockingIOError" not in str(row.get("error")) for row in records)
    assert len(observed_tokens[cohort["assets"][0]["asset_id"]]) == 1
    assert len(observed_tokens[cohort["assets"][1]["asset_id"]]) == 4
    assert all(len(set(tokens)) == 1 for tokens in observed_tokens.values())
    assert not (output / ".worker_scratch").exists()
    assert _json(output / "checkpoint.json")["state"] == "complete"


def test_spawn_eagain_exhaustion_leaves_clean_resumable_checkpoint(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path, count=1)
    _trust_fixture(monkeypatch, cohort)
    output = tmp_path / "spawn_eagain_exhausted"
    original_popen = runner.core.subprocess.Popen

    def always_eagain(command: list[str], *args: Any, **kwargs: Any) -> Any:
        if "--internal-child-job" not in command:
            return original_popen(command, *args, **kwargs)
        raise BlockingIOError(11, "Resource temporarily unavailable")

    monkeypatch.setattr(runner.core.subprocess, "Popen", always_eagain)
    monkeypatch.setattr(runner, "_sleep_spawn_retry", lambda _seconds: None, raising=False)

    with pytest.raises(RuntimeError, match="spawn EAGAIN retry budget exhausted"):
        runner.run(_args(cohort_path, output))

    assert _jsonl(output / "records.jsonl") == []
    assert _json(output / "checkpoint.json")["state"] == "running"
    assert not (output / ".worker_scratch").exists()
    assert not (output / "artifact_manifest.json").exists()

    monkeypatch.setattr(runner.core.subprocess, "Popen", original_popen)
    runner.run(_args(cohort_path, output, resume=True))
    resumed = _jsonl(output / "records.jsonl")
    assert len(resumed) == 1
    assert resumed[0]["asset_id"] == cohort["assets"][0]["asset_id"]
    assert resumed[0]["result_origin"] == "child_attested"
    assert "BlockingIOError" not in str(resumed[0].get("error"))


def test_non_eagain_spawn_failure_remains_fail_closed_metric_record(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path, count=1)
    _trust_fixture(monkeypatch, cohort)
    output = tmp_path / "spawn_permission_error"
    attempts = 0
    original_popen = runner.core.subprocess.Popen

    def permission_denied(command: list[str], *args: Any, **kwargs: Any) -> Any:
        nonlocal attempts
        if "--internal-child-job" not in command:
            return original_popen(command, *args, **kwargs)
        attempts += 1
        raise PermissionError(13, "Permission denied")

    monkeypatch.setattr(runner.core.subprocess, "Popen", permission_denied)

    runner.run(_args(cohort_path, output))

    record = _jsonl(output / "records.jsonl")[0]
    assert attempts == 1
    assert record["asset_id"] == cohort["assets"][0]["asset_id"]
    assert record["status"] == "error"
    assert record["result_origin"] == "parent_synthesized"
    assert "child_spawn_failed: PermissionError: [Errno 13]" in record["error"]
    assert all(metric["pass"] is False for metric in record["metrics"].values())
    assert not (output / ".worker_scratch").exists()


def test_post_popen_ownership_eagain_is_fail_closed_without_batch_restart(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path, count=2)
    _trust_fixture(monkeypatch, cohort)
    output = tmp_path / "ownership_eagain"
    original_popen = runner.core.subprocess.Popen
    original_atomic_write_json = runner.core.atomic_write_json
    popen_counts: dict[str, int] = {}
    ownership_failure_pending = True

    def recording_popen(command: list[str], *args: Any, **kwargs: Any) -> Any:
        if "--internal-child-job" not in command:
            return original_popen(command, *args, **kwargs)
        job_path = Path(command[command.index("--internal-child-job") + 1])
        asset_id = _json(job_path)["asset_id"]
        popen_counts[asset_id] = popen_counts.get(asset_id, 0) + 1
        return original_popen(command, *args, **kwargs)

    def flaky_ownership_write(path: Path, value: Any) -> None:
        nonlocal ownership_failure_pending
        if (
            ownership_failure_pending
            and path.name == "ownership.json"
            and path.parent.name == "job_000001"
            and isinstance(value, dict)
            and value.get("pid") is not None
        ):
            ownership_failure_pending = False
            raise BlockingIOError(11, "Resource temporarily unavailable")
        original_atomic_write_json(path, value)

    monkeypatch.setattr(runner.core.subprocess, "Popen", recording_popen)
    monkeypatch.setattr(runner.core, "atomic_write_json", flaky_ownership_write)
    args = _args(cohort_path, output)
    args.workers = 2

    runner.run(args)

    records = _jsonl(output / "records.jsonl")
    first_id, second_id = [row["asset_id"] for row in cohort["assets"]]
    assert popen_counts == {first_id: 1, second_id: 1}
    assert records[0]["asset_id"] == first_id
    assert records[0]["result_origin"] == "child_attested"
    assert records[0]["status"] == "completed"
    assert records[1]["asset_id"] == second_id
    assert records[1]["result_origin"] == "parent_synthesized"
    assert records[1]["status"] == "error"
    assert "child_spawn_failed: BlockingIOError: [Errno 11]" in records[1]["error"]
    assert all(metric["pass"] is False for metric in records[1]["metrics"].values())
    assert not (output / ".worker_scratch").exists()


def test_resume_keeps_attested_record_and_completes_only_missing_asset(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path)
    _trust_fixture(monkeypatch, cohort)
    output = tmp_path / "table2_resume"
    runner.run(_args(cohort_path, output))
    first = _jsonl(output / "records.jsonl")[0]

    (output / "artifact_manifest.json").unlink()
    (output / "summary.json").unlink()
    (output / "report.md").unlink()
    scratch_recoveries = _json(output / "checkpoint.json")["scratch_recoveries"]
    runner.core.atomic_write_jsonl(output / "records.jsonl", [first])
    partial_records_sha256 = common.sha256_file(output / "records.jsonl")
    runner.core.atomic_write_json(
        output / "checkpoint.json",
        {
            "manifest_content_sha256": _json(output / "manifest.json")["manifest_content_sha256"],
            "completed": 1,
            "n_eval": 2,
            "remaining": 1,
            "scratch_recoveries": scratch_recoveries,
            "state": "running",
            "records_sha256": partial_records_sha256,
            "manifest_file_sha256": common.sha256_file(output / "manifest.json"),
        },
    )

    runner.run(_args(cohort_path, output, resume=True))

    resumed = _jsonl(output / "records.jsonl")
    assert [row["asset_id"] for row in resumed] == [row["asset_id"] for row in cohort["assets"]]
    assert resumed[0]["worker_evidence"]["pid"] == first["worker_evidence"]["pid"]
    assert resumed[0]["completion_order"] == first["completion_order"]
    assert resumed[1]["completion_order"] > resumed[0]["completion_order"]
    assert _json(output / "checkpoint.json")["state"] == "complete"
    common.verify_artifacts(output)


@pytest.mark.parametrize("target", ["manifest.json", "protocol_snapshot.md", "records.jsonl"])
def test_resume_rejects_tampered_frozen_or_checkpoint_artifact(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, target: str
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path, count=1)
    _trust_fixture(monkeypatch, cohort)
    output = tmp_path / f"tamper_{target.replace('.', '_')}"
    runner.run(_args(cohort_path, output))
    (output / "artifact_manifest.json").unlink()
    path = output / target
    path.write_bytes(path.read_bytes() + b"\n")

    with pytest.raises((RuntimeError, ValueError), match="hash|self-hash|SHA-256|JSONL|record"):
        runner.run(_args(cohort_path, output, resume=True))


def test_formal_contract_pins_frozen_cohort_and_canonical_python(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    canonical_formal_cohort: tuple[dict[str, Any], dict[str, Any]],
) -> None:
    cohort, receipt = canonical_formal_cohort
    args = _formal_args(tmp_path)
    monkeypatch.setattr(runner.sys, "executable", str(runner.FORMAL_PYTHON))
    runner.validate_contract(args, cohort, receipt)

    assert runner.FORMAL_COHORT_MANIFEST_SHA256 == (
        "cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08"
    )
    assert runner.FORMAL_COHORT_CONTENT_SHA256 == (
        "f5e29f1becd47cae991f5d238dff3f86b2b009365738df3e46cdbea297032c23"
    )
    assert runner.FORMAL_COHORT_ARTIFACT_MANIFEST_SHA256 == (
        "ac31de70d50ed7153178482bb5283659be94fb5945cc2b7157754ac61dfc5439"
    )
    receipt_fields = {
        "cohort_manifest_sha256": runner.FORMAL_COHORT_MANIFEST_SHA256,
        "cohort_manifest_content_sha256": runner.FORMAL_COHORT_CONTENT_SHA256,
        "cohort_artifact_manifest_sha256": runner.FORMAL_COHORT_ARTIFACT_MANIFEST_SHA256,
    }
    for field, expected in receipt_fields.items():
        assert receipt[field] == expected
        with pytest.raises(ValueError, match="frozen cohort identity"):
            runner.validate_contract(args, cohort, {**receipt, field: "0" * 64})

    with pytest.raises(ValueError, match="exactly 720"):
        runner.validate_contract(args, {**cohort, "N_eval": 719}, receipt)
    args.limit = 1
    with pytest.raises(ValueError, match="limit"):
        runner.validate_contract(args, cohort, receipt)
    args.limit = None
    monkeypatch.setattr(runner.sys, "executable", "/wrong/python")
    with pytest.raises(RuntimeError, match="Python environment"):
        runner.validate_contract(args, cohort, receipt)


def test_formal_validator_rejects_reorder_status_provenance_and_joint_drift(
    canonical_formal_cohort: tuple[dict[str, Any], dict[str, Any]],
) -> None:
    cohort, _receipt = canonical_formal_cohort
    runner.validate_formal_cohort(cohort)

    reordered = list(cohort["assets"])
    reordered[0], reordered[1] = (
        {**reordered[1], "selection_index": 1},
        {**reordered[0], "selection_index": 2},
    )
    with pytest.raises(ValueError, match="ordered 20 x 36"):
        runner.validate_formal_cohort({**cohort, "assets": reordered})

    pass_index = next(
        index for index, row in enumerate(cohort["assets"])
        if row["original_status"] == "PASS"
    )
    invalid_pass = _replace_formal_asset(
        cohort,
        pass_index,
        recovery_used=True,
        source="recovery",
        recovery_provenance={"recovery_record_sha256": "a" * 64},
    )
    with pytest.raises(ValueError, match="713 primary PASS"):
        runner.validate_formal_cohort(invalid_pass)

    wrong_source = _replace_formal_asset(cohort, pass_index, source="recovery")
    with pytest.raises(ValueError, match="713 primary PASS"):
        runner.validate_formal_cohort(wrong_source)

    wrong_category = _replace_formal_asset(cohort, pass_index, raw_category="wrong")
    with pytest.raises(ValueError, match="identity fields"):
        runner.validate_formal_cohort(wrong_category)

    row = cohort["assets"][pass_index]
    wrong_joint_total = _replace_formal_asset(
        cohort,
        pass_index,
        declared_joint_count_hint=row["declared_joint_count_hint"] + 1,
    )
    with pytest.raises(ValueError, match="J=4723"):
        runner.validate_formal_cohort(wrong_joint_total)

    zero_index = next(
        index for index, item in enumerate(cohort["assets"])
        if item["declared_joint_count_hint"] == 0
    )
    nonzero_index = next(
        index for index, item in enumerate(cohort["assets"])
        if item["declared_joint_count_hint"] > 1
    )
    zero_assets = list(cohort["assets"])
    zero_assets[zero_index] = {**zero_assets[zero_index], "declared_joint_count_hint": 1}
    zero_assets[nonzero_index] = {
        **zero_assets[nonzero_index],
        "declared_joint_count_hint": zero_assets[nonzero_index]["declared_joint_count_hint"] - 1,
    }
    with pytest.raises(ValueError, match="zero-joint=55"):
        runner.validate_formal_cohort({**cohort, "assets": zero_assets})


def test_formal_validator_requires_complete_exact_recovery_provenance(
    canonical_formal_cohort: tuple[dict[str, Any], dict[str, Any]],
) -> None:
    cohort, _receipt = canonical_formal_cohort
    timeout_index = next(
        index for index, row in enumerate(cohort["assets"])
        if row["original_status"] == "TIMEOUT"
    )
    row = cohort["assets"][timeout_index]
    provenance = row["recovery_provenance"]
    assert isinstance(provenance, dict)
    for field in runner.FORMAL_RECOVERY_PROVENANCE_FIELDS:
        incomplete = dict(provenance)
        incomplete.pop(field)
        mutated = _replace_formal_asset(
            cohort, timeout_index, recovery_provenance=incomplete
        )
        with pytest.raises(ValueError, match="recovery provenance"):
            runner.validate_formal_cohort(mutated)

    bad_hash = _replace_formal_asset(
        cohort,
        timeout_index,
        recovery_provenance={**provenance, "recovery_record_sha256": "0" * 64},
    )
    with pytest.raises(ValueError, match="recovery provenance"):
        runner.validate_formal_cohort(bad_hash)

    bad_path = _replace_formal_asset(
        cohort,
        timeout_index,
        recovery_provenance={**provenance, "recovery_record_path": provenance["original_record_path"]},
    )
    with pytest.raises(ValueError, match="recovery provenance"):
        runner.validate_formal_cohort(bad_path)


def test_cohort_validation_failure_is_fatal_before_output_creation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, _ = _write_cohort(tmp_path, count=1)

    def reject(_path: Path, *, formal: bool = False) -> dict[str, Any]:
        raise ValueError("source binding drift")

    monkeypatch.setattr(runner.common, "verify_cohort_manifest", reject)
    output = tmp_path / "must_not_exist"
    with pytest.raises(ValueError, match="source binding drift"):
        runner.run(_args(cohort_path, output))
    assert not output.exists()


def test_smoke_rejects_cohort_selection_index_inconsistent_with_frozen_order(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path, count=1)
    cohort["assets"][0]["selection_index"] = 7
    _trust_fixture(monkeypatch, cohort)
    output = tmp_path / "bad_order"

    with pytest.raises(ValueError, match="selection_index"):
        runner.run(_args(cohort_path, output))
    assert not output.exists()


def test_child_runtime_binding_uses_child_thread_environment_and_normalized_interpreter(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path, count=1)
    _trust_fixture(monkeypatch, cohort)
    inherited = {name: str(index + 7) for index, name in enumerate(runner.core.CHILD_THREAD_ENVIRONMENT)}
    for name, value in inherited.items():
        monkeypatch.setenv(name, value)
    unnormalized_python = str(EXP_ROOT / "../arti-template/.venv/bin/python")
    monkeypatch.setattr(runner.sys, "executable", unnormalized_python)
    output = tmp_path / "child_environment"

    runner.run(_args(cohort_path, output))

    manifest = _json(output / "manifest.json")
    record = _jsonl(output / "records.jsonl")[0]
    assert manifest["evaluation"]["environment"]["executable"] == os.path.abspath(
        unnormalized_python
    )
    assert manifest["evaluation"]["child_thread_environment"] == runner.core.CHILD_THREAD_ENVIRONMENT
    assert record["job_runtime_binding"] == record["worker_runtime_binding"]
    assert record["worker_runtime_binding"]["environment"] == manifest["evaluation"]["environment"]
    observed = record["worker_runtime_binding"]["environment"]
    assert observed["thread_environment_observed"] == runner.FROZEN_THREAD_ENVIRONMENT
    assert observed["openblas_threadpools_observed"]
    assert {
        pool["num_threads"] for pool in observed["openblas_threadpools_observed"]
    } == {1}
    assert {name: os.environ.get(name) for name in inherited} == inherited


def test_subprocess_bootstrap_constrains_actual_openblas_threadpools() -> None:
    inherited = {
        name: str(index + 7)
        for index, name in enumerate(runner.core.CHILD_THREAD_ENVIRONMENT)
    }
    process = subprocess.run(
        [
            str(EXP_ROOT.parent / "arti-template/.venv/bin/python"),
            str(runner.SCRIPT_PATH),
            "--internal-thread-runtime-probe",
        ],
        cwd=EXP_ROOT,
        env={**os.environ, **inherited},
        text=True,
        capture_output=True,
        timeout=30,
        check=False,
    )
    assert process.returncode == 0, process.stderr
    observed = json.loads(process.stdout)
    assert observed["thread_environment_observed"] == runner.FROZEN_THREAD_ENVIRONMENT
    assert observed["openblas_threadpools_observed"]
    assert {
        pool["num_threads"] for pool in observed["openblas_threadpools_observed"]
    } == {1}


def test_child_runtime_binding_rejects_shared_core_drift(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path, count=1)
    _trust_fixture(monkeypatch, cohort)
    drifted_core = tmp_path / "drifted_run_table2_urdf_articraft.py"
    drifted_core.write_bytes(runner.SHARED_CORE_PATH.read_bytes() + b"\n# drift\n")
    monkeypatch.setattr(runner, "SHARED_CORE_PATH", drifted_core)
    output = tmp_path / "shared_core_drift"

    with pytest.raises(
        runner.core.FatalRuntimeBindingError,
        match="child runtime binding failed|environment",
    ):
        runner.run(_args(cohort_path, output))


def test_package_root_symlink_redirect_is_failed_closed_in_child(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, cohort = _write_cohort(tmp_path, count=1)
    _trust_fixture(monkeypatch, cohort)
    package = Path(cohort["assets"][0]["package_path"])
    redirected = package.with_name(f"{package.name}_redirected")
    package.rename(redirected)
    package.symlink_to(redirected, target_is_directory=True)
    output = tmp_path / "table2_root_symlink"

    runner.run(_args(cohort_path, output))

    record = _jsonl(output / "records.jsonl")[0]
    assert record["status"] == "error"
    assert "package_path_precheck_failed" in record["error"]
    assert "symlink" in record["error"]
    assert record["result_origin"] == "child_attested"
    assert _json(output / "summary.json")["n_eval"] == 1


def test_resume_quarantines_dead_stale_worker_scratch_and_records_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, output, _cohort = _prepare_completed_resume_fixture(
        tmp_path, monkeypatch, "dead_stale"
    )
    stale = output / ".worker_scratch" / "job_000000"
    stale.mkdir(parents=True)
    (stale / "ownership.json").write_text(
        json.dumps({"pid": 99999999, "pgid": 99999999}), encoding="utf-8"
    )

    runner.run(_args(cohort_path, output, resume=True))

    assert not (output / ".worker_scratch").exists()
    checkpoint = _json(output / "checkpoint.json")
    recovery = checkpoint["scratch_recoveries"][-1]
    assert recovery["quarantined"] is True
    assert recovery["terminated_owned_groups"] == []
    assert Path(recovery["quarantine_path"]).is_dir()


def test_resume_never_kills_unproven_stale_process(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, output, _cohort = _prepare_completed_resume_fixture(
        tmp_path, monkeypatch, "unproven_stale"
    )
    stale = output / ".worker_scratch" / "job_000000"
    stale.mkdir(parents=True)
    job_path = stale / "job.json"
    job_path.write_text(json.dumps({"run_token": "expected"}), encoding="utf-8")
    process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    try:
        (stale / "ownership.json").write_text(
            json.dumps(
                {
                    "pid": process.pid,
                    "pgid": os.getpgid(process.pid),
                    "run_token": "forged",
                    "job_path": str(job_path),
                    "runner_script": str(runner.SCRIPT_PATH),
                    "runner_sha256": common.sha256_file(runner.SCRIPT_PATH),
                    "output_root": str(output),
                    "process_start_identity": runner.core.proc_start_identity(process.pid),
                }
            ),
            encoding="utf-8",
        )
        runner.run(_args(cohort_path, output, resume=True))
        assert process.poll() is None
        recovery = _json(output / "checkpoint.json")["scratch_recoveries"][-1]
        assert recovery["quarantined"] is True
        assert recovery["terminated_owned_groups"] == []
    finally:
        if process.poll() is None:
            process.terminate()
        process.wait(timeout=5)


def test_resume_terminates_only_proven_owned_adapter_child(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cohort_path, output, _cohort = _prepare_completed_resume_fixture(
        tmp_path, monkeypatch, "owned_stale"
    )
    stale = output / ".worker_scratch" / "job_000000"
    stale.mkdir(parents=True)
    job_path = stale / "job.json"
    result_path = stale / "result.json"
    token = f"owned-{time.time_ns()}"
    job_path.write_text(
        json.dumps(
            {
                "asset_id": "owned",
                "internal_test_action": "sleep",
                "sleep": 30,
                "run_token": token,
            }
        ),
        encoding="utf-8",
    )
    process = subprocess.Popen(
        [
            sys.executable,
            str(runner.SCRIPT_PATH),
            "--internal-child-job",
            str(job_path),
            "--internal-child-result",
            str(result_path),
        ],
        start_new_session=True,
    )
    try:
        deadline = time.monotonic() + 5
        identity = None
        while identity is None and process.poll() is None and time.monotonic() < deadline:
            identity = runner.core.proc_start_identity(process.pid)
            time.sleep(0.01)
        assert identity is not None
        (stale / "ownership.json").write_text(
            json.dumps(
                {
                    "pid": process.pid,
                    "pgid": process.pid,
                    "run_token": token,
                    "job_path": str(job_path),
                    "runner_script": str(runner.SCRIPT_PATH),
                    "runner_sha256": common.sha256_file(runner.SCRIPT_PATH),
                    "output_root": str(output),
                    "process_start_identity": identity,
                }
            ),
            encoding="utf-8",
        )
        runner.run(_args(cohort_path, output, resume=True))
        process.wait(timeout=5)
        recovery = _json(output / "checkpoint.json")["scratch_recoveries"][-1]
        assert process.pid in recovery["terminated_owned_groups"]
    finally:
        if process.poll() is None:
            os.killpg(process.pid, 9)
        process.wait(timeout=5)
