#!/usr/bin/env python3
"""Isolated PyBullet/MuJoCo strict-load evaluation for a Table 5 manifest."""

from __future__ import annotations

import argparse
import copy
import fcntl
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import traceback
import uuid
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterator, Mapping, Sequence


_SCRIPT_DIRECTORY = str(Path(__file__).resolve().parent)
if _SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, _SCRIPT_DIRECTORY)
import table5_n200_runtime as _base


_legacy = _base._legacy
PyBulletAdapter = _base.PyBulletAdapter
MuJoCoAdapter = _base.MuJoCoAdapter

ASSET_SCHEMA = "table5_n200_load_only_asset_v1"
EVALUATION_SCHEMA = "table5_n200_load_only_evaluation_v1"
WORKER_REQUEST_SCHEMA = "table5_n200_load_only_worker_request_v1"
INTENT_SUMMARY_SCHEMA = "table5_n200_load_only_summary_v1"
RUN_SUMMARY_SCHEMA = "table5_n200_load_only_run_summary_v1"
EXECUTION_PROFILE = "strict_load_only_v1"
SIMULATORS = ("pybullet", "mujoco")
PLANNED_METRICS = ("load",)
NOT_EVALUATED_METRICS = (
    "reset",
    "settling",
    "actuation",
    "limit_enforcement",
    "constraint_drift",
    "simulator_pass",
)
PROFILE_CONTRACT = {
    "execution_profile": EXECUTION_PROFILE,
    "simulators": list(SIMULATORS),
    "planned_metrics": list(PLANNED_METRICS),
    "not_evaluated_metrics": list(NOT_EVALUATED_METRICS),
    "load_definition": (
        "native adapter construction followed by exact observed/expected "
        "URDF link and joint name multisets"
    ),
    "pybullet_actual_state_capacity": "not_applicable_to_load_only",
    "failure_policy": "fail_closed_without_denominator_removal",
}
EXECUTION_PROFILE_SHA256 = _base.canonical_sha256(PROFILE_CONTRACT)
DEFAULT_TIMEOUT_S = 300.0
TAIL_LIMIT = 8192
PROCESS_TERM_GRACE_S = 5.0
TERMINAL_STATUSES = {
    "completed",
    "preflight_failure",
    "diagnostic_failure",
    "timeout",
    "native_crash",
    "malformed_response",
    "worker_error",
    "parent_error",
}


class LoadOnlyContractError(ValueError):
    """Raised when an input or retained terminal record is not trustworthy."""


@dataclass
class WorkerOutcome:
    returncode: int | None = None
    timed_out: bool = False
    duration_s: float = 0.0
    response: dict[str, Any] | None = None
    response_error: str | None = None
    parent_error: str | None = None
    stdout_tail: str = ""
    stderr_tail: str = ""
    command: list[str] | None = None


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (
        json.dumps(
            value,
            sort_keys=True,
            indent=2,
            ensure_ascii=True,
            allow_nan=False,
        )
        + "\n"
    )
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, path)
    finally:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise LoadOnlyContractError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise LoadOnlyContractError(f"{label} must be a JSON object: {path}")
    return value


def _resolve_executable(value: str) -> str:
    located = shutil.which(value)
    if located is not None:
        return os.path.abspath(located)
    candidate = Path(value).expanduser()
    return (
        os.path.abspath(candidate) if candidate.is_absolute() or "/" in value else value
    )


def _resolve_urdf(row: Mapping[str, Any], manifest_path: Path) -> Path:
    return _base._resolve_urdf(row, manifest_path)


def _identity(
    bundle: _base.ManifestBundle,
    row: Mapping[str, Any],
    *,
    simulator: str,
    executable: str,
    timeout_s: float,
    effective_workers: int,
) -> dict[str, Any]:
    return {
        "dataset_slug": row["dataset_slug"],
        "dataset_name": row["dataset_name"],
        "dataset_id": row["dataset_id"],
        "asset_id": row["asset_id"],
        "simulator": simulator,
        "manifest_sha256": bundle.manifest_sha256,
        "protocol_sha256": bundle.protocol_sha256,
        "row_sha256": row["row_sha256"],
        "urdf_path": str(_resolve_urdf(row, bundle.path)),
        "urdf_sha256": row["urdf_sha256"],
        "executable": _resolve_executable(executable),
        "runner_source_sha256": sha256_file(Path(__file__).resolve()),
        "timeout_s": float(timeout_s),
        "effective_workers": effective_workers,
        "execution_profile": EXECUTION_PROFILE,
        "execution_profile_sha256": EXECUTION_PROFILE_SHA256,
    }


def _preflight_failure(row: Mapping[str, Any], urdf_path: Path) -> str | None:
    preflight = row["preflight"]
    if preflight["simulator_eligible"] is False:
        issues = preflight.get("issues")
        detail = (
            ",".join(str(item) for item in issues) if isinstance(issues, list) else ""
        )
        return f"manifest_preflight_failed{': ' + detail if detail else ''}"
    if not urdf_path.is_file():
        return f"urdf_missing: {urdf_path}"
    try:
        actual = sha256_file(urdf_path)
    except OSError as error:
        return f"urdf_unreadable: {type(error).__name__}: {error}"
    if actual != row["urdf_sha256"]:
        return f"urdf_hash_mismatch: expected {row['urdf_sha256']}, observed {actual}"
    return None


def _profile_fields() -> dict[str, Any]:
    return {
        "execution_profile": EXECUTION_PROFILE,
        "execution_profile_sha256": EXECUTION_PROFILE_SHA256,
        "planned_metrics": list(PLANNED_METRICS),
        "not_evaluated_metrics": list(NOT_EVALUATED_METRICS),
    }


def _failure_evaluation(kind: str, message: str) -> dict[str, Any]:
    return {
        "schema_version": EVALUATION_SCHEMA,
        **_profile_fields(),
        "metrics": {"load": False},
        "load": None,
        "support": None,
        "diagnostics": {"warnings": [], "errors": [message[-TAIL_LIMIT:]]},
        "failure": {"kind": kind, "message": message[-TAIL_LIMIT:]},
    }


def evaluate_load_only(adapter: Any, row: Mapping[str, Any]) -> dict[str, Any]:
    movable = [
        joint for joint in row["joint_tree"]["joints"] if joint.get("type") != "fixed"
    ]
    support_map = _legacy.classify_support(movable, adapter.mapped_joint_names)
    supported = _legacy._eligible(support_map)
    unsupported = [entry for entry in support_map.values() if not entry["eligible"]]
    supported_names = sorted(entry["name"] for entry in supported)
    load = _legacy._strict_load(adapter, row, supported_names)
    return {
        "schema_version": EVALUATION_SCHEMA,
        **_profile_fields(),
        "metrics": {"load": bool(load["strict_load"])},
        "load": load,
        "support": {
            "joints": list(support_map.values()),
            "supported_joint_names": supported_names,
            "unsupported_joint_names": sorted(entry["name"] for entry in unsupported),
            "any_unsupported_movable_joint": bool(unsupported),
            "no_supported_movable_joint": not bool(supported),
        },
        "diagnostics": {
            "warnings": list(getattr(adapter, "warnings", [])),
            "errors": [],
        },
        "failure": None,
    }


def _make_load_only_adapter(
    simulator: str,
    raw_urdf_path: Path,
    row: dict[str, Any],
    protocol: dict[str, Any],
) -> Any:
    adapter_type = {
        "pybullet": PyBulletAdapter,
        "mujoco": MuJoCoAdapter,
    }.get(simulator)
    if adapter_type is None:
        raise LoadOnlyContractError(f"unsupported load-only simulator: {simulator}")
    original_validator = _legacy.validate_runtime_protocol
    original_capacity_check = _legacy._require_pybullet_actual_state_capacity
    _legacy.validate_runtime_protocol = _base._generic_protocol_check
    if simulator == "pybullet":
        _legacy._require_pybullet_actual_state_capacity = lambda _joint_count: None
    try:
        return adapter_type(raw_urdf_path, row, protocol)
    finally:
        _legacy.validate_runtime_protocol = original_validator
        _legacy._require_pybullet_actual_state_capacity = original_capacity_check


def _validate_worker_request(
    request: Mapping[str, Any],
) -> tuple[str, dict[str, Any], dict[str, Any], Path]:
    if request.get("schema_version") != WORKER_REQUEST_SCHEMA:
        raise LoadOnlyContractError("worker request schema mismatch")
    if (
        request.get("execution_profile") != EXECUTION_PROFILE
        or request.get("execution_profile_sha256") != EXECUTION_PROFILE_SHA256
    ):
        raise LoadOnlyContractError("worker request execution profile mismatch")
    simulator = request.get("simulator")
    if simulator not in SIMULATORS:
        raise LoadOnlyContractError("worker request simulator is invalid")
    row, protocol, identity = (
        request.get("row"),
        request.get("protocol"),
        request.get("identity"),
    )
    if not isinstance(row, dict) or not isinstance(identity, dict):
        raise LoadOnlyContractError("worker request row/identity is invalid")
    validated_protocol = _base._validate_protocol(protocol)
    if (
        identity.get("execution_profile") != EXECUTION_PROFILE
        or identity.get("execution_profile_sha256") != EXECUTION_PROFILE_SHA256
    ):
        raise LoadOnlyContractError("worker identity execution profile mismatch")
    if identity.get("simulator") != simulator:
        raise LoadOnlyContractError("worker identity simulator mismatch")
    declared_row_hash = row.get("row_sha256")
    if (
        identity.get("row_sha256") != declared_row_hash
        or _base.canonical_sha256(row, exclude_fields=("row_sha256",))
        != declared_row_hash
    ):
        raise LoadOnlyContractError("worker identity row hash mismatch")
    declared_protocol_hash = validated_protocol.get("protocol_sha256")
    if (
        identity.get("protocol_sha256") != declared_protocol_hash
        or _base.canonical_sha256(
            validated_protocol, exclude_fields=("protocol_sha256",)
        )
        != declared_protocol_hash
    ):
        raise LoadOnlyContractError("worker identity protocol hash mismatch")
    raw_urdf_path = Path(str(request.get("urdf_path", ""))).resolve(strict=True)
    if str(raw_urdf_path) != identity.get("urdf_path"):
        raise LoadOnlyContractError("worker identity URDF path mismatch")
    expected_urdf_hash = identity.get("urdf_sha256")
    if (
        not isinstance(expected_urdf_hash, str)
        or sha256_file(raw_urdf_path) != expected_urdf_hash
    ):
        raise LoadOnlyContractError("worker URDF hash mismatch")
    return simulator, copy.deepcopy(row), validated_protocol, raw_urdf_path


def worker_main(request_path: Path, response_path: Path) -> int:
    adapter: Any | None = None
    try:
        request = _read_json(request_path, "worker request")
        simulator, row, protocol, raw_urdf_path = _validate_worker_request(request)
        adapter = _make_load_only_adapter(simulator, raw_urdf_path, row, protocol)
        response = evaluate_load_only(adapter, row)
    except _legacy.DiagnosticFailure as error:
        message = str(error.evidence.get("message", error.evidence.get("reason", "")))
        response = _failure_evaluation("diagnostic_failure", message)
        response["diagnostic_failure"] = copy.deepcopy(error.evidence)
    except BaseException as error:
        response = _failure_evaluation(
            "worker_error", f"{type(error).__name__}: {error}"
        )
        response["worker_error"] = f"{type(error).__name__}: {error}"
        response["traceback_tail"] = traceback.format_exc()[-TAIL_LIMIT:]
    finally:
        if adapter is not None:
            try:
                adapter.close()
            except BaseException as error:
                response["close_error"] = f"{type(error).__name__}: {error}"
    atomic_write_json(response_path, response)
    return 0


def _terminate_group(process: subprocess.Popen[Any]) -> None:
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=PROCESS_TERM_GRACE_S)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def _tail(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")[-TAIL_LIMIT:]
    except OSError:
        return ""


def spawn_worker_process(
    *,
    request: dict[str, Any],
    executable: str,
    timeout_s: float,
    work_root: Path,
) -> WorkerOutcome:
    run_id = f"{request['row']['dataset_id']}-{uuid.uuid4().hex}"
    request_path = work_root / ".worker_requests" / f"{run_id}.json"
    response_path = work_root / ".worker_responses" / f"{run_id}.json"
    stdout_path = work_root / "worker_logs" / f"{run_id}.stdout.log"
    stderr_path = work_root / "worker_logs" / f"{run_id}.stderr.log"
    atomic_write_json(request_path, request)
    response_path.parent.mkdir(parents=True, exist_ok=True)
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        _resolve_executable(executable),
        str(Path(__file__).resolve()),
        "worker",
        "--request",
        str(request_path),
        "--response",
        str(response_path),
    ]
    environment = os.environ.copy()
    thread_caps = request["protocol"].get("runtime", {}).get("thread_caps", {})
    thread_names = {
        "omp": "OMP_NUM_THREADS",
        "mkl": "MKL_NUM_THREADS",
        "openblas": "OPENBLAS_NUM_THREADS",
        "numexpr": "NUMEXPR_NUM_THREADS",
        "veclib": "VECLIB_MAXIMUM_THREADS",
        "taichi": "TI_NUM_THREADS",
    }
    for key, environment_name in thread_names.items():
        value = thread_caps.get(key)
        if isinstance(value, int) and not isinstance(value, bool) and value > 0:
            environment[environment_name] = str(value)
    started = time.monotonic()
    try:
        with stdout_path.open("wb") as stdout_handle, stderr_path.open(
            "wb"
        ) as stderr_handle:
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=stdout_handle,
                stderr=stderr_handle,
                env=environment,
                start_new_session=True,
            )
            timed_out = False
            try:
                process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                timed_out = True
                _terminate_group(process)
    except Exception as error:
        return WorkerOutcome(
            duration_s=time.monotonic() - started,
            parent_error=f"{type(error).__name__}: {error}",
            stdout_tail=_tail(stdout_path),
            stderr_tail=_tail(stderr_path),
            command=command,
        )
    response: dict[str, Any] | None = None
    response_error: str | None = None
    if response_path.is_file():
        try:
            loaded = json.loads(response_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                response = loaded
            else:
                response_error = "worker response is not a JSON object"
        except (OSError, json.JSONDecodeError) as error:
            response_error = f"cannot parse worker response: {error}"
    else:
        response_error = "worker response file is missing"
    return WorkerOutcome(
        returncode=process.returncode,
        timed_out=timed_out,
        duration_s=time.monotonic() - started,
        response=response,
        response_error=response_error,
        stdout_tail=_tail(stdout_path),
        stderr_tail=_tail(stderr_path),
        command=command,
    )


def _extract_load(response: Any) -> bool | None:
    if not isinstance(response, dict):
        return None
    if response.get("schema_version") != EVALUATION_SCHEMA:
        return None
    if (
        response.get("execution_profile") != EXECUTION_PROFILE
        or response.get("execution_profile_sha256") != EXECUTION_PROFILE_SHA256
    ):
        return None
    if response.get("planned_metrics") != list(PLANNED_METRICS) or response.get(
        "not_evaluated_metrics"
    ) != list(NOT_EVALUATED_METRICS):
        return None
    metrics = response.get("metrics")
    if not isinstance(metrics, dict) or set(metrics) != {"load"}:
        return None
    return metrics["load"] if isinstance(metrics["load"], bool) else None


def _terminal_record(
    identity: dict[str, Any],
    outcome: WorkerOutcome,
    *,
    preflight_error: str | None = None,
) -> dict[str, Any]:
    response = outcome.response
    load = _extract_load(response)
    failure: dict[str, str] | None = None
    if preflight_error is not None:
        status = "preflight_failure"
        failure = {"kind": status, "message": preflight_error}
        response, load = _failure_evaluation(status, preflight_error), False
    elif outcome.parent_error is not None:
        status = "parent_error"
        failure = {"kind": status, "message": outcome.parent_error}
        response, load = _failure_evaluation(status, outcome.parent_error), False
    elif outcome.timed_out:
        status = "timeout"
        message = f"worker exceeded {identity['timeout_s']} seconds"
        failure = {"kind": status, "message": message}
        closed = _failure_evaluation(status, message)
        if response is not None:
            closed["raw_worker_output"] = response
        response, load = closed, False
    elif outcome.returncode not in (0, None):
        status = "native_crash"
        message = f"worker exited with code {outcome.returncode}"
        failure = {"kind": status, "message": message}
        closed = _failure_evaluation(status, message)
        if response is not None:
            closed["raw_worker_output"] = response
        response, load = closed, False
    elif outcome.response_error is not None:
        status = "malformed_response"
        failure = {"kind": status, "message": outcome.response_error}
        response, load = _failure_evaluation(status, outcome.response_error), False
    elif isinstance(response, dict) and "worker_error" in response:
        status = "worker_error"
        message = str(response["worker_error"])
        failure, load = {"kind": status, "message": message}, False
    elif isinstance(response, dict) and isinstance(
        response.get("diagnostic_failure"), dict
    ):
        status = "diagnostic_failure"
        evidence = response["diagnostic_failure"]
        message = str(evidence.get("message", evidence.get("reason", status)))
        failure, load = {"kind": status, "message": message}, False
    elif load is None:
        status = "malformed_response"
        message = "worker response violates the load-only evaluation schema"
        failure, load = {"kind": status, "message": message}, False
        response = _failure_evaluation(status, message)
    else:
        status = "completed"
    return {
        "schema_version": ASSET_SCHEMA,
        "terminal": True,
        "terminal_status": status,
        "identity": identity,
        **_profile_fields(),
        "metrics": {"load": bool(load)},
        "evaluation": response,
        "failure": failure,
        "process": {
            "command": outcome.command,
            "returncode": outcome.returncode,
            "timeout": outcome.timed_out,
            "duration_s": outcome.duration_s,
            "stdout_tail": outcome.stdout_tail[-TAIL_LIMIT:],
            "stderr_tail": outcome.stderr_tail[-TAIL_LIMIT:],
        },
    }


def validate_terminal_record(record: Any, expected_identity: Mapping[str, Any]) -> None:
    if (
        not isinstance(record, dict)
        or record.get("schema_version") != ASSET_SCHEMA
        or record.get("terminal") is not True
    ):
        raise LoadOnlyContractError("existing terminal record has an invalid schema")
    if record.get("identity") != dict(expected_identity):
        raise LoadOnlyContractError(
            "existing terminal record identity does not match exactly"
        )
    if any(record.get(key) != value for key, value in _profile_fields().items()):
        raise LoadOnlyContractError("existing terminal record profile is invalid")
    status = record.get("terminal_status")
    if status not in TERMINAL_STATUSES:
        raise LoadOnlyContractError("existing terminal record status is invalid")
    metrics = record.get("metrics")
    if (
        not isinstance(metrics, dict)
        or set(metrics) != {"load"}
        or not isinstance(metrics["load"], bool)
    ):
        raise LoadOnlyContractError("existing terminal metrics are malformed")
    evaluation = record.get("evaluation")
    if (
        not isinstance(evaluation, dict)
        or _extract_load(evaluation) is not metrics["load"]
    ):
        raise LoadOnlyContractError("existing terminal evaluation is malformed")
    if status == "completed":
        load_evidence = evaluation.get("load")
        if (
            record.get("failure") is not None
            or not isinstance(load_evidence, dict)
            or load_evidence.get("strict_load") is not metrics["load"]
        ):
            raise LoadOnlyContractError("completed terminal record is inconsistent")
    elif metrics["load"] or not isinstance(record.get("failure"), dict):
        raise LoadOnlyContractError("failure terminal record is not fail-closed")
    if not isinstance(record.get("process"), dict):
        raise LoadOnlyContractError("existing terminal process evidence is malformed")
    try:
        json.dumps(record, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise LoadOnlyContractError(
            f"existing terminal record is not finite JSON: {error}"
        ) from error


@contextmanager
def _output_lock(root: Path) -> Iterator[None]:
    root.mkdir(parents=True, exist_ok=True)
    with (root / ".load_only.lock").open("a+", encoding="utf-8") as handle:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise LoadOnlyContractError(f"output is already locked: {root}") from error
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


WorkerLauncher = Callable[..., WorkerOutcome]


def run_intent(
    bundle: _base.ManifestBundle,
    dataset: _base.DatasetRows,
    simulator: str,
    *,
    runtime_root: Path,
    executable: str,
    workers: int,
    timeout_s: float,
    launcher: WorkerLauncher = spawn_worker_process,
) -> dict[str, Any]:
    if simulator not in SIMULATORS:
        raise LoadOnlyContractError(f"unsupported load-only simulator: {simulator}")
    if not isinstance(workers, int) or isinstance(workers, bool) or workers < 1:
        raise LoadOnlyContractError("workers must be a positive integer")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise LoadOnlyContractError("timeout must be finite and positive")
    output_root = Path(runtime_root) / dataset.slug / simulator
    assets_root = output_root / "assets"
    records: dict[str, dict[str, Any]] = {}
    resumed = 0
    pending: list[tuple[dict[str, Any], dict[str, Any], str | None]] = []
    with _output_lock(output_root):
        assets_root.mkdir(parents=True, exist_ok=True)
        for row in dataset.rows:
            identity = _identity(
                bundle,
                row,
                simulator=simulator,
                executable=executable,
                timeout_s=timeout_s,
                effective_workers=workers,
            )
            terminal_path = assets_root / f"{row['dataset_id']}.json"
            if terminal_path.exists():
                existing = _read_json(terminal_path, "terminal record")
                try:
                    validate_terminal_record(existing, identity)
                except LoadOnlyContractError as error:
                    raise LoadOnlyContractError(f"{terminal_path}: {error}") from error
                records[row["dataset_id"]] = existing
                resumed += 1
                continue
            urdf_path = Path(identity["urdf_path"])
            pending.append((row, identity, _preflight_failure(row, urdf_path)))

        def invoke(
            item: tuple[dict[str, Any], dict[str, Any], str | None],
        ) -> tuple[dict[str, Any], dict[str, Any]]:
            row, identity, preflight_error = item
            if preflight_error is not None:
                outcome = WorkerOutcome()
            else:
                request = {
                    "schema_version": WORKER_REQUEST_SCHEMA,
                    **_profile_fields(),
                    "simulator": simulator,
                    "identity": identity,
                    "row": row,
                    "protocol": bundle.protocol,
                    "urdf_path": identity["urdf_path"],
                }
                try:
                    outcome = launcher(
                        request=request,
                        executable=executable,
                        timeout_s=timeout_s,
                        work_root=output_root,
                    )
                except Exception as error:
                    outcome = WorkerOutcome(
                        parent_error=f"{type(error).__name__}: {error}"
                    )
            return row, _terminal_record(
                identity, outcome, preflight_error=preflight_error
            )

        def retain(row: Mapping[str, Any], record: dict[str, Any]) -> None:
            terminal_path = assets_root / f"{row['dataset_id']}.json"
            if terminal_path.exists():
                raise LoadOnlyContractError(
                    f"refusing to overwrite terminal record: {terminal_path}"
                )
            validate_terminal_record(record, record["identity"])
            atomic_write_json(terminal_path, record)
            records[row["dataset_id"]] = record

        if workers == 1:
            for item in pending:
                row, record = invoke(item)
                retain(row, record)
        elif pending:
            with ThreadPoolExecutor(
                max_workers=workers,
                thread_name_prefix=f"table5-load-{dataset.slug}-{simulator}",
            ) as executor:
                futures = [executor.submit(invoke, item) for item in pending]
                for future in as_completed(futures):
                    row, record = future.result()
                    retain(row, record)

        status_counts = Counter(
            record["terminal_status"] for record in records.values()
        )
        summary = {
            "schema_version": INTENT_SUMMARY_SCHEMA,
            **_profile_fields(),
            "manifest_sha256": bundle.manifest_sha256,
            "protocol_sha256": bundle.protocol_sha256,
            "dataset_slug": dataset.slug,
            "dataset_name": dataset.name,
            "simulator": simulator,
            "executable": _resolve_executable(executable),
            "timeout_s": float(timeout_s),
            "effective_workers": workers,
            "intent_count": len(dataset.rows),
            "terminal_count": len(records),
            "resumed_count": resumed,
            "terminal_status_counts": dict(sorted(status_counts.items())),
            "load_pass_count": sum(
                int(record["metrics"]["load"]) for record in records.values()
            ),
            "metric_denominator": len(dataset.rows),
            "complete": len(records) == len(dataset.rows),
        }
        atomic_write_json(output_root / "summary.json", summary)
        return summary


def _parse_csv(value: str | None) -> list[str] | None:
    if value is None:
        return None
    result = [item.strip() for item in value.split(",") if item.strip()]
    if not result or len(result) != len(set(result)):
        raise LoadOnlyContractError("comma-separated selection is empty or duplicated")
    return result


def _parse_assignments(
    values: Sequence[str] | None,
    *,
    default: Any,
    converter: Callable[[str], Any],
) -> dict[str, Any]:
    result = {simulator: default for simulator in SIMULATORS}
    if not values:
        return result
    for value in values:
        for assignment in value.split(","):
            if not assignment.strip():
                continue
            if "=" not in assignment:
                converted = converter(assignment.strip())
                result = {simulator: converted for simulator in SIMULATORS}
                continue
            simulator, raw = assignment.split("=", 1)
            simulator, raw = simulator.strip(), raw.strip()
            if simulator not in SIMULATORS or not raw:
                raise LoadOnlyContractError(
                    f"invalid simulator assignment: {assignment}"
                )
            result[simulator] = converter(raw)
    return result


def run_manifest(
    manifest: Path | str,
    runtime_root: Path | str,
    *,
    datasets: Sequence[str] | None = None,
    simulators: Sequence[str] = SIMULATORS,
    workers: Mapping[str, int] | None = None,
    executables: Mapping[str, str] | None = None,
    timeout_s: float | None = None,
    launcher: WorkerLauncher = spawn_worker_process,
) -> dict[str, Any]:
    bundle = _base.load_manifest(manifest)
    requested_simulators = tuple(simulators)
    if (
        not requested_simulators
        or len(requested_simulators) != len(set(requested_simulators))
        or any(simulator not in SIMULATORS for simulator in requested_simulators)
    ):
        raise LoadOnlyContractError(
            "simulators must be a unique non-empty subset of pybullet,mujoco"
        )
    selected = list(bundle.datasets)
    if datasets is not None:
        wanted = set(datasets)
        known = {dataset.slug for dataset in bundle.datasets}
        if wanted - known:
            raise LoadOnlyContractError(
                f"unknown dataset slugs: {sorted(wanted - known)}"
            )
        selected = [dataset for dataset in bundle.datasets if dataset.slug in wanted]
    worker_counts = {simulator: 1 for simulator in SIMULATORS}
    if workers is not None:
        worker_counts.update(workers)
    if any(
        not isinstance(value, int) or isinstance(value, bool) or value < 1
        for value in worker_counts.values()
    ):
        raise LoadOnlyContractError("worker counts must be positive integers")
    python_executables = {simulator: sys.executable for simulator in SIMULATORS}
    if executables is not None:
        python_executables.update(executables)
    effective_timeout = (
        float(bundle.protocol["runtime"]["child_timeout_s"])
        if timeout_s is None
        else float(timeout_s)
    )
    if not math.isfinite(effective_timeout) or effective_timeout <= 0:
        raise LoadOnlyContractError("timeout must be finite and positive")
    summaries: list[dict[str, Any]] = []
    for dataset in selected:
        for simulator in requested_simulators:
            summaries.append(
                run_intent(
                    bundle,
                    dataset,
                    simulator,
                    runtime_root=Path(runtime_root),
                    executable=python_executables[simulator],
                    workers=worker_counts[simulator],
                    timeout_s=effective_timeout,
                    launcher=launcher,
                )
            )
    summary = {
        "schema_version": RUN_SUMMARY_SCHEMA,
        **_profile_fields(),
        "manifest": str(bundle.path),
        "manifest_sha256": bundle.manifest_sha256,
        "protocol_sha256": bundle.protocol_sha256,
        "datasets": [dataset.slug for dataset in selected],
        "simulators": list(requested_simulators),
        "intent_count": sum(item["intent_count"] for item in summaries),
        "terminal_count": sum(item["terminal_count"] for item in summaries),
        "runs": summaries,
    }
    atomic_write_json(Path(runtime_root) / "summary.json", summary)
    return summary


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True, metavar="{run}")
    run = commands.add_parser("run", help="run strict-load intents")
    run.add_argument("--manifest", type=Path, required=True)
    run.add_argument(
        "--runtime-root", "--out", dest="runtime_root", type=Path, required=True
    )
    run.add_argument("--datasets", help="comma-separated dataset slugs; default: all")
    run.add_argument(
        "--simulators",
        default=",".join(SIMULATORS),
        help="comma-separated subset of pybullet,mujoco",
    )
    run.add_argument(
        "--workers",
        action="append",
        help="COUNT for all or comma-separated simulator=COUNT assignments",
    )
    run.add_argument(
        "--executables",
        action="append",
        help="PYTHON for all or comma-separated simulator=PYTHON assignments",
    )
    run.add_argument(
        "--timeout",
        type=float,
        help="per-asset seconds; default: manifest protocol child_timeout_s",
    )
    worker = commands.add_parser("worker", help=argparse.SUPPRESS)
    worker.add_argument("--request", type=Path, required=True, help=argparse.SUPPRESS)
    worker.add_argument("--response", type=Path, required=True, help=argparse.SUPPRESS)
    commands._choices_actions = [
        action for action in commands._choices_actions if action.dest != "worker"
    ]
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "worker":
        return worker_main(args.request, args.response)
    datasets = _parse_csv(args.datasets)
    simulators = _parse_csv(args.simulators)
    if simulators is None:
        raise LoadOnlyContractError("--simulators is empty")
    workers = _parse_assignments(args.workers, default=1, converter=int)
    executables = _parse_assignments(
        args.executables, default=sys.executable, converter=str
    )
    summary = run_manifest(
        args.manifest,
        args.runtime_root,
        datasets=datasets,
        simulators=simulators,
        workers=workers,
        executables=executables,
        timeout_s=args.timeout,
    )
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (LoadOnlyContractError, _base.RuntimeContractError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
