#!/usr/bin/env python3
"""Run the shared Table 5 evaluator for a frozen multi-dataset manifest.

Every asset is evaluated in a fresh Python process.  The parent retains one
terminal JSON record per manifest row, including launch, parse, and timeout
failures, so the manifest remains the metric denominator.
"""

from __future__ import annotations

import argparse
import copy
import fcntl
import hashlib
import json
import logging
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import traceback
import uuid
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Iterator, Mapping, Sequence

_SCRIPT_DIRECTORY = str(Path(__file__).resolve().parent)
if _SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, _SCRIPT_DIRECTORY)
import run_table5_sketch_mobility as _legacy


ASSET_SCHEMA = "table5_n200_runtime_asset_v1"
WORKER_REQUEST_SCHEMA = "table5_n200_worker_request_v1"
SUMMARY_SCHEMA = "table5_n200_runtime_summary_v1"
DEFAULT_TIMEOUT_S = 300.0
TAIL_LIMIT = 8192
PROCESS_TERM_GRACE_S = 5.0
SIMULATORS = ("pybullet", "mujoco", "genesis")
METRIC_NAMES = (
    "load",
    "reset",
    "settling",
    "actuation",
    "limit_enforcement",
    "constraint_drift",
    "simulator_pass",
)
TERMINAL_STATUSES = {
    "completed",
    "diagnostic_failure",
    "preflight_failure",
    "timeout",
    "native_crash",
    "worker_error",
    "malformed_response",
    "parent_error",
}
SAFE_COMPONENT = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")
SHA256 = re.compile(r"[0-9a-f]{64}")

# These are intentionally the established scientific implementation.  The
# orchestration and hardware contract below are dataset-agnostic.
evaluate_asset = _legacy.evaluate_asset
PyBulletAdapter = _legacy.PyBulletAdapter
MuJoCoAdapter = _legacy.MuJoCoAdapter


class RuntimeContractError(ValueError):
    """Raised when input or an existing terminal artifact is not trustworthy."""


@dataclass(frozen=True)
class DatasetRows:
    slug: str
    name: str
    rows: tuple[dict[str, Any], ...]


@dataclass(frozen=True)
class ManifestBundle:
    path: Path
    raw: dict[str, Any]
    protocol: dict[str, Any]
    datasets: tuple[DatasetRows, ...]
    manifest_sha256: str
    protocol_sha256: str


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


def canonical_sha256(value: Any, *, exclude_fields: Iterable[str] = ()) -> str:
    excluded = set(exclude_fields)

    def filtered(item: Any) -> Any:
        if isinstance(item, dict):
            return {
                key: filtered(child)
                for key, child in item.items()
                if key not in excluded
            }
        if isinstance(item, list):
            return [filtered(child) for child in item]
        return item

    encoded = json.dumps(
        filtered(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


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
        raise RuntimeContractError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise RuntimeContractError(f"{label} must be a JSON object: {path}")
    return value


def _safe_component(value: Any, label: str) -> str:
    if not isinstance(value, str) or SAFE_COMPONENT.fullmatch(value) is None:
        raise RuntimeContractError(f"{label} is not a safe path component: {value!r}")
    return value


def _validate_protocol(protocol: Any) -> dict[str, Any]:
    if not isinstance(protocol, dict):
        raise RuntimeContractError("manifest.protocol must be an embedded object")
    for section in ("runtime", "metrics", "cross_simulator", "adapters"):
        if not isinstance(protocol.get(section), dict):
            raise RuntimeContractError(f"manifest.protocol.{section} is missing")
    for simulator in SIMULATORS:
        if not isinstance(protocol["adapters"].get(simulator), dict):
            raise RuntimeContractError(
                f"manifest.protocol.adapters.{simulator} is missing"
            )
    runtime = protocol["runtime"]
    required_runtime = (
        "gravity_m_per_s2",
        "timestep_s",
        "solver_iterations",
        "reset_repetitions",
        "passive_settling",
        "actuation",
        "limit_enforcement",
        "child_timeout_s",
    )
    missing = [name for name in required_runtime if name not in runtime]
    if missing:
        raise RuntimeContractError(
            "manifest protocol lacks evaluator runtime fields: " + ", ".join(missing)
        )
    child_timeout = runtime["child_timeout_s"]
    if (
        not isinstance(child_timeout, (int, float))
        or isinstance(child_timeout, bool)
        or not math.isfinite(float(child_timeout))
        or float(child_timeout) <= 0
    ):
        raise RuntimeContractError("manifest protocol child_timeout_s is invalid")
    logical_and = protocol["metrics"].get("simulator_pass", {}).get("logical_and")
    if not isinstance(logical_and, list) or set(logical_and) != set(METRIC_NAMES[:-1]):
        raise RuntimeContractError(
            "manifest protocol has an invalid simulator_pass gate"
        )
    implementation = protocol.get("implementation")
    if isinstance(implementation, dict):
        expected_sources = {
            Path(__file__).resolve(): implementation.get("runtime_script_sha256"),
            Path(_legacy.__file__).resolve(): implementation.get(
                "evaluator_script_sha256"
            ),
        }
        for source_path, expected_hash in expected_sources.items():
            if (
                not isinstance(expected_hash, str)
                or SHA256.fullmatch(expected_hash) is None
                or sha256_file(source_path) != expected_hash
            ):
                raise RuntimeContractError(
                    f"manifest protocol implementation hash mismatch: {source_path}"
                )
    return copy.deepcopy(protocol)


def _validate_row(row: Any, slug: str, name: str) -> dict[str, Any]:
    if not isinstance(row, dict):
        raise RuntimeContractError(f"{slug} manifest row must be an object")
    normalized = copy.deepcopy(row)
    for field in ("asset_id", "dataset_id", "urdf_path"):
        if not isinstance(normalized.get(field), str) or not normalized[field]:
            raise RuntimeContractError(f"{slug} row has invalid {field}")
    _safe_component(normalized["dataset_id"], f"{slug}.dataset_id")
    preflight = normalized.get("preflight")
    if not isinstance(preflight, dict) or not isinstance(
        preflight.get("simulator_eligible"), bool
    ):
        raise RuntimeContractError(
            f"{slug}/{normalized['dataset_id']} has invalid preflight evidence"
        )
    urdf_hash = normalized.get("urdf_sha256")
    if isinstance(urdf_hash, str):
        if SHA256.fullmatch(urdf_hash) is None:
            raise RuntimeContractError(
                f"{slug}/{normalized['dataset_id']} has invalid URDF hash"
            )
    elif urdf_hash is not None or preflight["simulator_eligible"]:
        raise RuntimeContractError(
            f"{slug}/{normalized['dataset_id']} has invalid URDF hash"
        )
    declared_row_hash = normalized.get("row_sha256")
    if (
        not isinstance(declared_row_hash, str)
        or SHA256.fullmatch(declared_row_hash) is None
        or canonical_sha256(normalized, exclude_fields=("row_sha256",))
        != declared_row_hash
    ):
        raise RuntimeContractError(
            f"{slug}/{normalized['dataset_id']} row_sha256 self-check failed"
        )
    if not isinstance(normalized.get("joint_tree"), dict):
        preflight = normalized.get("preflight")
        eligible = isinstance(preflight, dict) and preflight.get("simulator_eligible")
        if eligible is not False:
            raise RuntimeContractError(
                f"{slug}/{normalized['dataset_id']} lacks joint_tree without a failed preflight"
            )
    if not isinstance(normalized.get("scalar_joints"), list):
        normalized["scalar_joints"] = []
    normalized.setdefault("dataset_slug", slug)
    normalized.setdefault("dataset_name", name)
    if normalized["dataset_slug"] != slug or normalized["dataset_name"] != name:
        raise RuntimeContractError(
            f"{slug}/{normalized['dataset_id']} dataset identity conflicts with its group"
        )
    return normalized


def load_manifest(path: Path | str) -> ManifestBundle:
    manifest_path = Path(path).resolve()
    raw = _read_json(manifest_path, "manifest")
    protocol = _validate_protocol(raw.get("protocol"))
    declared_protocol_hash = raw.get("protocol_sha256")
    embedded_protocol_hash = protocol.get("protocol_sha256")
    computed_protocol_hash = canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    if (
        not isinstance(declared_protocol_hash, str)
        or SHA256.fullmatch(declared_protocol_hash) is None
        or embedded_protocol_hash != declared_protocol_hash
        or computed_protocol_hash != declared_protocol_hash
    ):
        raise RuntimeContractError("manifest protocol_sha256 self-check failed")
    declared_manifest_hash = raw.get("manifest_sha256")
    if (
        not isinstance(declared_manifest_hash, str)
        or SHA256.fullmatch(declared_manifest_hash) is None
        or canonical_sha256(raw, exclude_fields=("manifest_sha256",))
        != declared_manifest_hash
    ):
        raise RuntimeContractError("manifest_sha256 self-check failed")
    datasets: list[DatasetRows] = []
    grouped = raw.get("datasets")
    if isinstance(grouped, list):
        by_slug: dict[str, dict[str, Any]] = {}
        for group in grouped:
            if not isinstance(group, dict):
                raise RuntimeContractError("manifest.datasets entries must be objects")
            slug = _safe_component(group.get("dataset_slug"), "dataset_slug")
            name = group.get("dataset_name")
            if not isinstance(name, str) or not name:
                raise RuntimeContractError(f"{slug} has invalid dataset_name")
            if slug in by_slug:
                raise RuntimeContractError(f"duplicate dataset_slug: {slug}")
            by_slug[slug] = group
        ordered = raw.get("ordered_dataset_slugs", list(by_slug))
        if (
            not isinstance(ordered, list)
            or any(not isinstance(item, str) for item in ordered)
            or len(ordered) != len(set(ordered))
            or set(ordered) != set(by_slug)
        ):
            raise RuntimeContractError("ordered_dataset_slugs is not a permutation")
        for slug in ordered:
            group = by_slug[slug]
            rows = group.get("rows")
            if not isinstance(rows, list):
                raise RuntimeContractError(f"{slug}.rows must be a list")
            name = group["dataset_name"]
            datasets.append(
                DatasetRows(
                    slug,
                    name,
                    tuple(_validate_row(row, slug, name) for row in rows),
                )
            )
    else:
        # Compatibility for early flat manifests used while the six-dataset
        # builder was being finalized.
        rows = raw.get("rows", raw.get("assets"))
        if not isinstance(rows, list):
            raise RuntimeContractError(
                "manifest requires datasets[].rows or rows/assets"
            )
        buckets: dict[str, list[dict[str, Any]]] = {}
        names: dict[str, str] = {}
        order: list[str] = []
        for row in rows:
            if not isinstance(row, dict):
                raise RuntimeContractError("flat manifest row must be an object")
            slug = _safe_component(
                row.get("dataset_slug", row.get("slug")), "dataset_slug"
            )
            name = row.get("dataset_name", row.get("dataset", slug))
            if not isinstance(name, str) or not name:
                raise RuntimeContractError(f"{slug} has invalid dataset_name")
            if slug not in buckets:
                buckets[slug], names[slug] = [], name
                order.append(slug)
            if names[slug] != name:
                raise RuntimeContractError(f"inconsistent dataset_name for {slug}")
            buckets[slug].append(_validate_row(row, slug, name))
        datasets = [
            DatasetRows(slug, names[slug], tuple(buckets[slug])) for slug in order
        ]

    seen: set[tuple[str, str]] = set()
    for dataset in datasets:
        for row in dataset.rows:
            key = (dataset.slug, row["dataset_id"])
            if key in seen:
                raise RuntimeContractError(f"duplicate runtime identity: {key}")
            seen.add(key)
    return ManifestBundle(
        path=manifest_path,
        raw=raw,
        protocol=protocol,
        datasets=tuple(datasets),
        manifest_sha256=declared_manifest_hash,
        protocol_sha256=declared_protocol_hash,
    )


def _resolve_urdf(row: Mapping[str, Any], manifest_path: Path) -> Path:
    value = Path(str(row["urdf_path"]))
    if value.is_absolute():
        return value.resolve()
    package_root = row.get("package_root")
    if isinstance(package_root, str) and package_root:
        package = Path(package_root)
        if not package.is_absolute():
            package = manifest_path.parent / package
        package_candidate = (package / value).resolve()
        if package_candidate.is_file() or value.parent == Path("."):
            return package_candidate
    return (manifest_path.parent / value).resolve()


def _resolve_executable(value: str) -> str:
    located = shutil.which(value)
    if located is not None:
        return os.path.abspath(located)
    candidate = Path(value).expanduser()
    return (
        os.path.abspath(candidate) if candidate.is_absolute() or "/" in value else value
    )


def _finite_positive(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        and float(value) > 0
    )


def _identity(
    bundle: ManifestBundle,
    row: Mapping[str, Any],
    *,
    simulator: str,
    executable: str,
    timeout_s: float,
    gpu_binding: str | None,
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
        "worker_source_sha256": sha256_file(Path(__file__).resolve()),
        "timeout_s": float(timeout_s),
        "effective_workers": effective_workers,
        "cuda_visible_devices": gpu_binding if simulator == "genesis" else None,
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


def _false_metrics() -> dict[str, bool]:
    return {name: False for name in METRIC_NAMES}


def _extract_metrics(response: Any) -> dict[str, bool] | None:
    if not isinstance(response, dict):
        return None
    metrics = response.get("metrics")
    if not isinstance(metrics, dict) or set(metrics) != set(METRIC_NAMES):
        return None
    if not all(isinstance(metrics[name], bool) for name in METRIC_NAMES):
        return None
    return {name: metrics[name] for name in METRIC_NAMES}


def _failure_payload(kind: str, message: str) -> dict[str, Any]:
    return {
        "failure": {"kind": kind, "message": message[-TAIL_LIMIT:]},
        "metrics": _false_metrics(),
    }


def _terminal_record(
    identity: dict[str, Any],
    outcome: WorkerOutcome,
    *,
    preflight_error: str | None = None,
) -> dict[str, Any]:
    failure: dict[str, str] | None = None
    response = outcome.response
    metrics = _extract_metrics(response)
    if preflight_error is not None:
        status = "preflight_failure"
        failure = {"kind": status, "message": preflight_error}
        response, metrics = _failure_payload(status, preflight_error), _false_metrics()
    elif outcome.parent_error is not None:
        status = "parent_error"
        failure = {"kind": status, "message": outcome.parent_error}
        response, metrics = (
            _failure_payload(status, outcome.parent_error),
            _false_metrics(),
        )
    elif outcome.timed_out:
        status = "timeout"
        message = f"worker exceeded {identity['timeout_s']} seconds"
        failure = {"kind": status, "message": message}
        raw_response = response
        response, metrics = _failure_payload(status, message), _false_metrics()
        if raw_response is not None:
            response["raw_worker_output"] = raw_response
    elif outcome.returncode not in (0, None):
        status = "native_crash"
        message = f"worker exited with code {outcome.returncode}"
        failure = {"kind": status, "message": message}
        metrics = _false_metrics()
        raw_response = response
        response = _failure_payload(status, message)
        if raw_response is not None:
            response["raw_worker_output"] = raw_response
    elif outcome.response_error is not None:
        status = "malformed_response"
        failure = {"kind": status, "message": outcome.response_error}
        response, metrics = (
            _failure_payload(status, outcome.response_error),
            _false_metrics(),
        )
    elif isinstance(response, dict) and "worker_error" in response:
        status = "worker_error"
        message = str(response["worker_error"])
        failure, metrics = {"kind": status, "message": message}, _false_metrics()
        response = copy.deepcopy(response)
        response["metrics"] = _false_metrics()
    elif isinstance(response, dict) and isinstance(
        response.get("diagnostic_failure"), dict
    ):
        status = "diagnostic_failure"
        evidence = response["diagnostic_failure"]
        message = str(evidence.get("message", evidence.get("reason", status)))
        failure, metrics = {"kind": status, "message": message}, _false_metrics()
    elif metrics is None:
        status = "malformed_response"
        message = "worker response lacks the exact boolean metric set"
        failure, metrics = {"kind": status, "message": message}, _false_metrics()
    else:
        status = "completed"
    return {
        "schema_version": ASSET_SCHEMA,
        "terminal": True,
        "terminal_status": status,
        "identity": identity,
        "metrics": metrics,
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
        raise RuntimeContractError("existing terminal record has an invalid schema")
    if record.get("identity") != dict(expected_identity):
        raise RuntimeContractError(
            "existing terminal record identity does not match exactly"
        )
    status = record.get("terminal_status")
    if status not in TERMINAL_STATUSES:
        raise RuntimeContractError("existing terminal record status is invalid")
    metrics = record.get("metrics")
    if (
        not isinstance(metrics, dict)
        or set(metrics) != set(METRIC_NAMES)
        or not all(isinstance(metrics[name], bool) for name in METRIC_NAMES)
    ):
        raise RuntimeContractError("existing terminal record metrics are malformed")
    evaluation = record.get("evaluation")
    if not isinstance(evaluation, dict) or not isinstance(record.get("process"), dict):
        raise RuntimeContractError("existing terminal record evidence is malformed")
    evaluation_metrics = evaluation.get("metrics")
    if isinstance(evaluation_metrics, dict) and evaluation_metrics != metrics:
        raise RuntimeContractError(
            "existing terminal evaluation metrics do not match the terminal metrics"
        )
    if status == "completed":
        if record.get("failure") is not None or evaluation_metrics != metrics:
            raise RuntimeContractError(
                "existing completed terminal record is internally inconsistent"
            )
    elif any(metrics.values()) or not isinstance(record.get("failure"), dict):
        raise RuntimeContractError(
            "existing failure terminal record is not fail-closed"
        )
    try:
        json.dumps(record, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise RuntimeContractError(
            f"existing terminal record is not finite JSON: {error}"
        )


@contextmanager
def _output_lock(root: Path) -> Iterator[None]:
    root.mkdir(parents=True, exist_ok=True)
    with (root / ".runtime.lock").open("a+", encoding="utf-8") as handle:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeContractError(
                f"runtime output is already locked: {root}"
            ) from error
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


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
    gpu_binding: str | None,
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
    for key, env_name in thread_names.items():
        value = thread_caps.get(key)
        if isinstance(value, int) and not isinstance(value, bool) and value > 0:
            environment[env_name] = str(value)
    if request["simulator"] == "genesis":
        if gpu_binding is not None:
            environment["CUDA_VISIBLE_DEVICES"] = gpu_binding
        environment.setdefault("CUDA_DEVICE_ORDER", "PCI_BUS_ID")
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


class DynamicGenesisAdapter(_legacy.GenesisAdapter):
    """Genesis adapter bound only by this worker's CUDA environment.

    It deliberately omits the historical L20X UUID, idle-device, driver, and
    software snapshot gates.  Each worker still uses logical CUDA device zero,
    which maps to its parent-selected ``CUDA_VISIBLE_DEVICES`` value.
    """

    def __init__(
        self, raw_urdf_path: Path, row: dict[str, Any], protocol: dict[str, Any]
    ) -> None:
        import genesis as gs
        import torch

        expected_version = protocol["adapters"]["genesis"].get("version")
        if isinstance(expected_version, str) and expected_version:
            _legacy._distribution_version(
                ["genesis-world", "genesis"], expected_version
            )
        if not torch.cuda.is_available() or torch.cuda.device_count() != 1:
            raise RuntimeContractError(
                "Genesis worker must have exactly one visible CUDA device"
            )
        physical_token = os.environ.get("CUDA_VISIBLE_DEVICES")
        if not physical_token or "," in physical_token:
            raise RuntimeContractError(
                "Genesis worker lacks an exact single CUDA_VISIBLE_DEVICES binding"
            )
        try:
            query = subprocess.run(
                [
                    "nvidia-smi",
                    f"--id={physical_token}",
                    "--query-gpu=index,uuid,name,memory.total,driver_version",
                    "--format=csv,noheader,nounits",
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=15,
            )
            fields = [item.strip() for item in query.stdout.strip().split(",")]
            if len(fields) != 5 or not fields[1]:
                raise RuntimeContractError("nvidia-smi returned malformed GPU identity")
            physical_index, gpu_uuid, gpu_name, total_memory_mib, driver_version = (
                fields
            )
            total_memory = int(total_memory_mib)
        except (
            OSError,
            ValueError,
            subprocess.CalledProcessError,
            subprocess.TimeoutExpired,
        ) as error:
            raise RuntimeContractError(
                f"could not bind dynamic Genesis GPU identity: {error}"
            ) from error
        self.gs = gs
        self.device_receipt = {
            "binding_policy": "current_cuda_visible_devices",
            "physical_device_token": physical_token,
            "cuda_visible_devices": physical_token,
            "physical_device_index": int(physical_index),
            "gpu_uuid": gpu_uuid,
            "device_name": gpu_name,
            "total_memory_mib": total_memory,
            "driver_version": driver_version,
            "logical_device": "cuda:0",
            "logical_device_count": int(torch.cuda.device_count()),
            "torch_device_name": str(torch.cuda.get_device_name(0)),
            "torch_version": str(torch.__version__),
            "torch_cuda_version": str(torch.version.cuda),
        }
        gs.init(
            backend=gs.cuda,
            precision="32",
            seed=int(protocol["runtime"].get("random_seed", 0)),
            logging_level=logging.WARNING,
        )
        timestep = protocol["runtime"]["timestep_s"]
        contacts_value = protocol["runtime"].get("contacts", "enabled")
        contacts_enabled = contacts_value is True or str(contacts_value).lower() in {
            "enabled",
            "required",
            "on",
            "true",
        }
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(
                dt=timestep["numerator"] / timestep["denominator"],
                substeps=1,
                gravity=tuple(protocol["runtime"]["gravity_m_per_s2"]),
            ),
            rigid_options=gs.options.RigidOptions(
                enable_collision=contacts_enabled,
                enable_self_collision=contacts_enabled,
                enable_neutral_collision=contacts_enabled,
                iterations=protocol["runtime"]["solver_iterations"],
            ),
            show_viewer=False,
        )
        morph = gs.morphs.URDF(
            file=str(raw_urdf_path),
            fixed=True,
            visualization=False,
            collision=contacts_enabled,
            merge_fixed_links=False,
            requires_jac_and_IK=False,
            recompute_inertia=False,
        )
        try:
            self.entity = self.scene.add_entity(morph)
            self.scene.build()
        except Exception as error:
            try:
                self.scene.destroy()
            except Exception:
                pass
            try:
                self.gs.destroy()
            except Exception:
                pass
            raise _legacy._diagnostic_failure(
                reason="simulator_asset_load_rejected",
                stage="adapter_initialization",
                operation="scene.add_entity_or_build",
                error=error,
            ) from error
        self.root_name = row["joint_tree"]["root_links"][0]
        self.links = {link.name: link for link in self.entity.links}
        expected_joints = {joint["name"] for joint in row["joint_tree"]["joints"]}
        runtime_joints: dict[str, Any] = {}
        synthetic_root_names: set[str] = set()
        for joint in self.entity.joints:
            if joint.link.name == self.root_name and joint.name not in expected_joints:
                synthetic_root_names.add(joint.name)
                continue
            runtime_joints[joint.name] = joint
        self.observed_link_names = sorted(self.links)
        self.observed_joint_names = sorted(runtime_joints)
        expected_scalar = {joint["name"] for joint in row["scalar_joints"]}
        self.mapping = {
            name: runtime_joints[name].dofs_idx_local[0]
            for name in sorted(expected_scalar & set(runtime_joints))
            if runtime_joints[name].n_dofs == 1
            and len(runtime_joints[name].dofs_idx_local) == 1
        }
        self.mapped_joint_names = sorted(self.mapping)
        self.warnings = [
            f"ignored internal fixed-root joint: {name}"
            for name in sorted(synthetic_root_names)
        ]


def _generic_protocol_check(protocol: Mapping[str, Any]) -> None:
    _validate_protocol(protocol)


def _make_adapter(
    simulator: str,
    raw_urdf_path: Path,
    row: dict[str, Any],
    protocol: dict[str, Any],
) -> Any:
    if simulator == "genesis":
        return DynamicGenesisAdapter(raw_urdf_path, row, protocol)
    adapter_type = {
        "pybullet": PyBulletAdapter,
        "mujoco": MuJoCoAdapter,
    }.get(simulator)
    if adapter_type is None:
        raise RuntimeContractError(f"unsupported simulator: {simulator}")
    # PyBullet's established implementation calls a Sketch-specific semantic
    # validator.  Replace only that validation hook inside this isolated worker;
    # the adapter and evaluator implementations themselves remain unchanged.
    original_validator = _legacy.validate_runtime_protocol
    _legacy.validate_runtime_protocol = _generic_protocol_check
    try:
        return adapter_type(raw_urdf_path, row, protocol)
    finally:
        _legacy.validate_runtime_protocol = original_validator


def worker_main(request_path: Path, response_path: Path) -> int:
    adapter: Any | None = None
    try:
        request = _read_json(request_path, "worker request")
        if request.get("schema_version") != WORKER_REQUEST_SCHEMA:
            raise RuntimeContractError("worker request schema mismatch")
        simulator = request.get("simulator")
        if simulator not in SIMULATORS:
            raise RuntimeContractError("worker request simulator is invalid")
        row = copy.deepcopy(request["row"])
        protocol = _validate_protocol(request["protocol"])
        raw_urdf_path = Path(request["urdf_path"]).resolve(strict=True)
        missing_bbox = not _finite_positive(row.get("bounding_box_diagonal"))
        original_bbox = row.get("bounding_box_diagonal")
        if missing_bbox:
            row["bounding_box_diagonal"] = 1.0
        adapter = _make_adapter(simulator, raw_urdf_path, row, protocol)
        response = evaluate_asset(adapter, row, protocol)
        if missing_bbox:
            response["metrics"]["constraint_drift"] = False
            response["metrics"]["simulator_pass"] = False
            response["missing_bbox_normalizer"] = True
            response.setdefault("diagnostics", {})["missing_bbox_normalizer"] = {
                "reason": "missing_bbox_normalizer",
                "source_value": original_bbox,
                "substitute_used_only_for_evaluator_execution": 1.0,
            }
        device_receipt = getattr(adapter, "device_receipt", None)
        if device_receipt is not None:
            response["device_receipt"] = copy.deepcopy(device_receipt)
    except _legacy.DiagnosticFailure as error:
        response = {
            "diagnostic_failure": copy.deepcopy(error.evidence),
            "metrics": _false_metrics(),
        }
    except BaseException as error:
        response = {
            "worker_error": f"{type(error).__name__}: {error}",
            "traceback_tail": traceback.format_exc()[-TAIL_LIMIT:],
        }
    finally:
        if adapter is not None:
            try:
                adapter.close()
            except BaseException as error:
                response.setdefault("close_error", f"{type(error).__name__}: {error}")
    atomic_write_json(response_path, response)
    return 0


WorkerLauncher = Callable[..., WorkerOutcome]


def run_intent(
    bundle: ManifestBundle,
    dataset: DatasetRows,
    simulator: str,
    *,
    runtime_root: Path,
    executable: str,
    workers: int,
    timeout_s: float,
    gpu_bindings: Sequence[str | None],
    launcher: WorkerLauncher = spawn_worker_process,
) -> dict[str, Any]:
    if simulator not in SIMULATORS:
        raise RuntimeContractError(f"unsupported simulator: {simulator}")
    if not isinstance(workers, int) or isinstance(workers, bool) or workers < 1:
        raise RuntimeContractError("workers must be a positive integer")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise RuntimeContractError("timeout must be finite and positive")
    protocol_timeout = float(bundle.protocol["runtime"]["child_timeout_s"])
    if float(timeout_s) != protocol_timeout:
        raise RuntimeContractError(
            f"timeout {timeout_s} does not match frozen protocol value {protocol_timeout}"
        )
    if not gpu_bindings:
        gpu_bindings = _default_gpu_bindings()
    if simulator == "genesis":
        if any(
            not isinstance(binding, str) or not binding or "," in binding
            for binding in gpu_bindings
        ):
            raise RuntimeContractError(
                "each Genesis worker must receive exactly one CUDA device token"
            )
        effective_workers = min(workers, len(gpu_bindings))
    else:
        effective_workers = workers
    gpu_locks = (
        {binding: threading.Lock() for binding in gpu_bindings}
        if simulator == "genesis"
        else {}
    )
    output_root = Path(runtime_root) / dataset.slug / simulator
    assets_root = output_root / "assets"
    resumed = 0
    records: dict[str, dict[str, Any]] = {}
    pending: list[tuple[dict[str, Any], dict[str, Any], str | None, str | None]] = []
    with _output_lock(output_root):
        assets_root.mkdir(parents=True, exist_ok=True)
        for index, row in enumerate(dataset.rows):
            gpu_binding = (
                gpu_bindings[index % len(gpu_bindings)]
                if simulator == "genesis"
                else None
            )
            identity = _identity(
                bundle,
                row,
                simulator=simulator,
                executable=executable,
                timeout_s=timeout_s,
                gpu_binding=gpu_binding,
                effective_workers=effective_workers,
            )
            terminal_path = assets_root / f"{row['dataset_id']}.json"
            if terminal_path.exists():
                existing = _read_json(terminal_path, "terminal record")
                try:
                    validate_terminal_record(existing, identity)
                except RuntimeContractError as error:
                    raise RuntimeContractError(f"{terminal_path}: {error}") from error
                records[row["dataset_id"]] = existing
                resumed += 1
                continue
            urdf_path = Path(identity["urdf_path"])
            pending.append(
                (row, identity, gpu_binding, _preflight_failure(row, urdf_path))
            )

        def invoke(
            item: tuple[dict[str, Any], dict[str, Any], str | None, str | None],
        ) -> tuple[dict[str, Any], dict[str, Any]]:
            row, identity, gpu_binding, preflight_error = item
            if preflight_error is not None:
                outcome = WorkerOutcome()
            else:
                request = {
                    "schema_version": WORKER_REQUEST_SCHEMA,
                    "simulator": simulator,
                    "row": row,
                    "protocol": bundle.protocol,
                    "urdf_path": identity["urdf_path"],
                }
                try:
                    launch_arguments = {
                        "request": request,
                        "executable": executable,
                        "timeout_s": timeout_s,
                        "gpu_binding": gpu_binding,
                        "work_root": output_root,
                    }
                    if simulator == "genesis":
                        with gpu_locks[gpu_binding]:
                            outcome = launcher(**launch_arguments)
                    else:
                        outcome = launcher(**launch_arguments)
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
                raise RuntimeContractError(
                    f"refusing to overwrite terminal record: {terminal_path}"
                )
            validate_terminal_record(record, record["identity"])
            atomic_write_json(terminal_path, record)
            records[row["dataset_id"]] = record

        if effective_workers == 1:
            for item in pending:
                row, record = invoke(item)
                retain(row, record)
        elif pending:
            with ThreadPoolExecutor(
                max_workers=effective_workers,
                thread_name_prefix=f"table5-{dataset.slug}-{simulator}",
            ) as executor:
                futures = [executor.submit(invoke, item) for item in pending]
                for future in as_completed(futures):
                    row, record = future.result()
                    retain(row, record)

        status_counts = Counter(
            record["terminal_status"] for record in records.values()
        )
        metric_counts = {
            metric: sum(int(record["metrics"][metric]) for record in records.values())
            for metric in METRIC_NAMES
        }
        summary = {
            "dataset_slug": dataset.slug,
            "dataset_name": dataset.name,
            "simulator": simulator,
            "intent_count": len(dataset.rows),
            "terminal_count": len(records),
            "resumed_count": resumed,
            "launched_or_retained_count": len(pending),
            "requested_workers": workers,
            "effective_workers": effective_workers,
            "terminal_status_counts": dict(sorted(status_counts.items())),
            "metric_pass_counts": metric_counts,
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
        raise RuntimeContractError("comma-separated selection is empty or duplicated")
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
                raise RuntimeContractError(
                    f"invalid simulator assignment: {assignment}"
                )
            result[simulator] = converter(raw)
    return result


def _gpu_bindings(values: Sequence[str] | None) -> tuple[str | None, ...]:
    if values:
        flattened: list[str] = []
        for value in values:
            if value.startswith("genesis="):
                value = value.partition("=")[2]
            flattened.extend(item.strip() for item in value.split(",") if item.strip())
        if not flattened or len(flattened) != len(set(flattened)):
            raise RuntimeContractError("--gpus must contain unique CUDA device tokens")
        return tuple(flattened)
    inherited = os.environ.get("CUDA_VISIBLE_DEVICES", "0")
    inherited_tokens = tuple(
        item.strip() for item in inherited.split(",") if item.strip()
    )
    if not inherited_tokens:
        return ("0",)
    if len(inherited_tokens) != len(set(inherited_tokens)):
        raise RuntimeContractError("CUDA_VISIBLE_DEVICES contains duplicate tokens")
    return inherited_tokens


def _default_gpu_bindings() -> tuple[str, ...]:
    return tuple(str(item) for item in _gpu_bindings(None))


def run_manifest(
    manifest: Path | str,
    runtime_root: Path | str,
    *,
    datasets: Sequence[str] | None = None,
    simulators: Sequence[str] = SIMULATORS,
    workers: Mapping[str, int] | None = None,
    executables: Mapping[str, str] | None = None,
    gpu_bindings: Sequence[str | None] | None = None,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    launcher: WorkerLauncher = spawn_worker_process,
) -> dict[str, Any]:
    bundle = load_manifest(manifest)
    requested_simulators = tuple(simulators)
    if (
        not requested_simulators
        or len(requested_simulators) != len(set(requested_simulators))
        or any(simulator not in SIMULATORS for simulator in requested_simulators)
    ):
        raise RuntimeContractError(
            "simulators must be a unique non-empty supported list"
        )
    selected = list(bundle.datasets)
    if datasets is not None:
        wanted = set(datasets)
        known = {dataset.slug for dataset in bundle.datasets}
        if wanted - known:
            raise RuntimeContractError(
                f"unknown dataset slugs: {sorted(wanted - known)}"
            )
        selected = [dataset for dataset in bundle.datasets if dataset.slug in wanted]
    worker_counts = {simulator: 1 for simulator in SIMULATORS}
    if workers is not None:
        worker_counts.update(workers)
    python_executables = {simulator: sys.executable for simulator in SIMULATORS}
    if executables is not None:
        python_executables.update(executables)
    bindings = tuple(gpu_bindings or _default_gpu_bindings())
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
                    timeout_s=timeout_s,
                    gpu_bindings=bindings,
                    launcher=launcher,
                )
            )
    summary = {
        "schema_version": SUMMARY_SCHEMA,
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
    run = commands.add_parser("run", help="run selected manifest intents")
    run.add_argument("--manifest", type=Path, required=True)
    run.add_argument(
        "--runtime-root", "--out", dest="runtime_root", type=Path, required=True
    )
    run.add_argument("--datasets", help="comma-separated dataset slugs; default: all")
    run.add_argument(
        "--simulators",
        default=",".join(SIMULATORS),
        help="comma-separated simulator names",
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
        "--gpus",
        action="append",
        help="Genesis CUDA device tokens; defaults to current CUDA_VISIBLE_DEVICES",
    )
    run.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
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
    selected_datasets = _parse_csv(args.datasets)
    selected_simulators = _parse_csv(args.simulators)
    if selected_simulators is None:
        raise RuntimeContractError("--simulators is empty")
    workers = _parse_assignments(args.workers, default=1, converter=int)
    if any(
        not isinstance(value, int) or isinstance(value, bool) or value < 1
        for value in workers.values()
    ):
        raise RuntimeContractError("worker counts must be positive integers")
    executables = _parse_assignments(
        args.executables, default=sys.executable, converter=str
    )
    summary = run_manifest(
        args.manifest,
        args.runtime_root,
        datasets=selected_datasets,
        simulators=selected_simulators,
        workers=workers,
        executables=executables,
        gpu_bindings=_gpu_bindings(args.gpus),
        timeout_s=args.timeout,
    )
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeContractError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
