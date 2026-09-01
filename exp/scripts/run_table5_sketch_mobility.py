#!/usr/bin/env python3
"""Isolated runtime evaluation for the frozen SketchMobility Table 1 cohort."""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.metadata
import importlib.util
import json
import logging
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import traceback
import urllib.parse
import uuid
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Protocol

try:
    from table5_sketch_mobility_common import (
        ManifestError,
        atomic_write_json,
        canonical_sha256,
        fk_link_poses,
        output_lock,
        package_binding,
        validate_manifest,
        validate_output_path,
        validate_receipt_set,
    )
except ModuleNotFoundError:
    from exp.scripts.table5_sketch_mobility_common import (
        ManifestError,
        atomic_write_json,
        canonical_sha256,
        fk_link_poses,
        output_lock,
        package_binding,
        validate_manifest,
        validate_output_path,
        validate_receipt_set,
    )

ASSET_SCHEMA = "table5_sketch_mobility_runtime_asset_v1"
EVALUATION_SCHEMA = "table5_sketch_mobility_runtime_evaluation_v1"
DIAGNOSTIC_FAILURE_SCHEMA = "table5_sketch_mobility_runtime_diagnostic_failure_v1"
IMPLEMENTATION_RECEIPT_SCHEMA = "table5_adapter_implementation_receipt_v2"
SUMMARY_SCHEMA = "table5_sketch_mobility_runtime_summary_v1"
PROGRESS_SCHEMA = "table5_sketch_mobility_runtime_progress_v1"
GENESIS_DEVICE_RECEIPT_SCHEMA = "table5_sketch_mobility_genesis_device_receipt_v1"
PARENT_GPU_RECEIPT_SCHEMA = "table5_sketch_mobility_parent_gpu_gate_receipt_v1"
CHILD_GPU_RECEIPT_SCHEMA = "table5_sketch_mobility_child_gpu_gate_receipt_v1"
SOURCE_BINDING_RECEIPT_SCHEMA = "table5_sketch_mobility_worker_source_binding_receipt_v1"
TAIL_LIMIT = 8192
WARNING_ENTRY_LIMIT = 100
WARNING_MESSAGE_LIMIT = 1024
PROCESS_TERM_GRACE_S = 5.0
PYBULLET_MAX_ACTUAL_STATE_JOINTS = 127
METRIC_NAMES = (
    "load",
    "reset",
    "settling",
    "actuation",
    "limit_enforcement",
    "constraint_drift",
    "simulator_pass",
)
ADAPTER_NAMES = {
    "pybullet": "PyBulletAdapter",
    "mujoco": "MuJoCoAdapter",
    "genesis": "GenesisAdapter",
}
ADAPTER_MODULES = {
    "pybullet": ("pybullet", ("pybullet",)),
    "mujoco": ("mujoco", ("mujoco",)),
    "genesis": ("genesis", ("genesis-world", "genesis")),
}
THREAD_ENV_NAMES = {
    "omp": "OMP_NUM_THREADS",
    "mkl": "MKL_NUM_THREADS",
    "openblas": "OPENBLAS_NUM_THREADS",
    "numexpr": "NUMEXPR_NUM_THREADS",
    "veclib": "VECLIB_MAXIMUM_THREADS",
    "taichi": "TI_NUM_THREADS",
}
DATASET_ID_PATTERN = re.compile(r"sketch_[0-9]{4}")
GENESIS_GPU_BINDING = {
    "cuda_visible_devices": "3",
    "physical_device_index": 3,
    "visible_device_index": 0,
    "gpu_uuid": "GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7",
}
FROZEN_GENESIS_GPU_HARDWARE = {
    "physical_device_index": 3,
    "gpu_uuid": "GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7",
    "device_name": "NVIDIA L20X",
    "total_memory_mib": 143771,
    "driver_version": "570.172.08",
    "compute_capability": "8.9",
}
FROZEN_GENESIS_TORCH_DEVICE = {
    "torch_device_name": "NVIDIA L20X",
    "torch_total_memory_bytes": 150121021440,
    "torch_total_memory_mib": 143166,
    "torch_compute_capability": "9.0",
}
FROZEN_GENESIS_SOFTWARE = {
    "torch_version": "2.8.0+cu128",
    "torch_cuda_version": "12.8",
    "cudnn_version": 91002,
    "quadrants_version": "1.2.0",
    "python_version": "3.12.13",
}


class RuntimeContractError(ValueError):
    """Raised when a terminal artifact or invocation violates the frozen contract."""


class GpuGateError(RuntimeContractError):
    """Raised when the frozen Genesis GPU identity or availability gate fails."""


class DiagnosticFailure(RuntimeError):
    """A normally retained simulator diagnostic interruption, distinct from completion."""

    def __init__(self, evidence: dict[str, Any]):
        self.evidence = evidence
        super().__init__(f"{evidence.get('reason')}: {evidence.get('message')}")


class ParentStop(BaseException):
    """A deliberate SIGINT/SIGTERM that must not become a scientific failure."""

    def __init__(self, signum: int):
        super().__init__(f"parent received signal {signum}")
        self.signum = signum


class Adapter(Protocol):
    observed_link_names: list[str]
    observed_joint_names: list[str]
    mapped_joint_names: list[str]

    def reset(self, positions: dict[str, float]) -> None: ...

    def state(self) -> dict[str, dict[str, float]]: ...

    def step(self, efforts: dict[str, float]) -> None: ...

    def link_poses(self) -> dict[str, dict[str, list[float]]]: ...

    def close(self) -> None: ...


@dataclass
class ProcessOutcome:
    exit_code: int | None = None
    signal: int | None = None
    timeout: bool = False
    stdout: str = ""
    stderr: str = ""
    response: dict[str, Any] | None = None
    parent_error: str | None = None
    preflight_error: str | None = None
    parent_gpu_receipt: dict[str, Any] | None = None


def _valid_dataset_id(value: Any) -> bool:
    return isinstance(value, str) and DATASET_ID_PATTERN.fullmatch(value) is not None


def _validate_row_identity(row: dict[str, Any]) -> None:
    dataset_id = row.get("dataset_id")
    order = row.get("order")
    if not _valid_dataset_id(dataset_id):
        raise RuntimeContractError("dataset_id must match sketch_[0-9]{4}")
    if not isinstance(order, int) or isinstance(order, bool) or not 0 <= order < 800:
        raise RuntimeContractError("SketchMobility row order must be an integer in [0, 799]")
    if dataset_id != f"sketch_{order:04d}":
        raise RuntimeContractError("dataset_id/order binding mismatch")
    manifest_root = row.get("manifest_root")
    if (
        not isinstance(manifest_root, str)
        or not manifest_root
        or row.get("asset_id") != manifest_root
    ):
        raise RuntimeContractError("asset_id/manifest_root authority mismatch")
    rank = row.get("selection_rank")
    if not isinstance(rank, int) or isinstance(rank, bool) or rank != order + 1:
        raise RuntimeContractError("selection_rank/order binding mismatch")
    if not _sha256_text(row.get("selection_hash")):
        raise RuntimeContractError("selection_hash must be a lowercase SHA-256")


def _preflight_eligible(row: dict[str, Any]) -> bool:
    preflight = row.get("preflight")
    if not isinstance(preflight, dict) or set(preflight) != {
        "status",
        "issues",
        "simulator_eligible",
    }:
        raise RuntimeContractError("manifest row preflight is malformed")
    status, issues, eligible = (
        preflight["status"],
        preflight["issues"],
        preflight["simulator_eligible"],
    )
    if (
        status not in {"pass", "failed"}
        or not isinstance(issues, list)
        or not all(isinstance(issue, str) and issue for issue in issues)
        or not isinstance(eligible, bool)
    ):
        raise RuntimeContractError("manifest row preflight is malformed")
    if (status == "pass") != eligible or eligible != (issues == []):
        raise RuntimeContractError("manifest row preflight fields are inconsistent")
    if eligible and not isinstance(row.get("joint_tree"), dict):
        raise RuntimeContractError(
            "preflight eligibility requires a valid joint_tree"
        )
    return eligible


def _preflight_message(row: dict[str, Any]) -> str:
    return ",".join(row["preflight"]["issues"])


def validate_runtime_protocol(protocol: dict[str, Any]) -> None:
    try:
        runtime = protocol["runtime"]
        pybullet = protocol["adapters"]["pybullet"]
        mujoco = protocol["adapters"]["mujoco"]
        genesis = protocol["adapters"]["genesis"]
    except (KeyError, TypeError) as error:
        raise RuntimeContractError("runtime protocol is incomplete") from error
    mismatches = {
        "runtime.contacts": runtime.get("contacts") != "enabled",
        "pybullet.ignore_collision": pybullet.get("ignore_collision") is not False,
        "pybullet.self_collision": pybullet.get("self_collision") is not True,
        "pybullet.contacts": pybullet.get("contacts") != "enabled",
        "pybullet.visual_shapes": pybullet.get("visual_shapes")
        != "ignored_after_manifest_resource_validation",
        "mujoco.contacts": mujoco.get("contacts") != "enabled",
        "mujoco.self_collision": mujoco.get("self_collision") is not True,
        "genesis.backend": genesis.get("backend") != "cuda",
        "genesis.precision": genesis.get("precision") != "float32",
        "genesis.collision": genesis.get("collision") is not True,
        "genesis.self_collision": genesis.get("self_collision") is not True,
        "genesis.enable_neutral_collision": genesis.get("enable_neutral_collision")
        is not True,
        "genesis.contacts": genesis.get("contacts") != "enabled",
        "genesis.visualization": genesis.get("visualization") is not False,
        "genesis.gpu_binding": genesis.get("gpu_binding") != GENESIS_GPU_BINDING,
    }
    failed = [name for name, mismatch in mismatches.items() if mismatch]
    if failed:
        raise RuntimeContractError(
            "SketchMobility runtime protocol semantic mismatch: " + ", ".join(failed)
        )


def _gpu_hardware_identity(receipt: dict[str, Any]) -> dict[str, Any]:
    return {key: receipt[key] for key in FROZEN_GENESIS_GPU_HARDWARE}


def _validate_parent_gpu_receipt(
    receipt: Any,
    *,
    require_idle: bool = True,
    allowed_compute_pids: set[int] | None = None,
) -> None:
    fields = {
        "schema_version",
        *FROZEN_GENESIS_GPU_HARDWARE,
        "used_memory_mib",
        "free_memory_mib",
        "utilization_percent",
        "compute_pids",
    }
    if not isinstance(receipt, dict) or set(receipt) != fields:
        raise GpuGateError("parent GPU gate receipt fields are malformed")
    if receipt["schema_version"] != PARENT_GPU_RECEIPT_SCHEMA:
        raise GpuGateError("parent GPU gate receipt schema mismatch")
    if _gpu_hardware_identity(receipt) != FROZEN_GENESIS_GPU_HARDWARE:
        raise GpuGateError("parent GPU hardware identity does not match frozen receipt")
    for field in ("used_memory_mib", "free_memory_mib", "utilization_percent"):
        if (
            not isinstance(receipt[field], int)
            or isinstance(receipt[field], bool)
            or receipt[field] < 0
        ):
            raise GpuGateError(f"parent GPU gate {field} is malformed")
    if not isinstance(receipt["compute_pids"], list) or not all(
        isinstance(pid, int) and not isinstance(pid, bool) and pid > 0
        for pid in receipt["compute_pids"]
    ):
        raise GpuGateError("parent GPU gate compute PID list is malformed")
    if receipt["compute_pids"] != sorted(set(receipt["compute_pids"])):
        raise GpuGateError("parent GPU gate compute PID list is not canonical")
    allowed = set() if allowed_compute_pids is None else allowed_compute_pids
    if not isinstance(allowed, set) or not all(
        isinstance(pid, int) and not isinstance(pid, bool) and pid > 0
        for pid in allowed
    ):
        raise GpuGateError("allowed GPU compute PID set is malformed")
    external_compute_pids = set(receipt["compute_pids"]) - allowed
    if require_idle and (
        receipt["used_memory_mib"] > 1024
        or receipt["free_memory_mib"] < 32768
        or receipt["utilization_percent"] != 0
        or external_compute_pids
    ):
        raise GpuGateError(
            "frozen Genesis GPU is occupied; refusing to race or fall back"
        )


def _query_parent_gpu_receipt(
    protocol: dict[str, Any], *, allowed_compute_pids: set[int] | None = None
) -> dict[str, Any]:
    validate_runtime_protocol(protocol)
    index = protocol["adapters"]["genesis"]["gpu_binding"]["physical_device_index"]
    fields = (
        "index,uuid,name,memory.total,memory.used,memory.free,"
        "utilization.gpu,driver_version,compute_cap"
    )
    try:
        gpu_result = subprocess.run(
            [
                "nvidia-smi",
                f"--id={index}",
                f"--query-gpu={fields}",
                "--format=csv,noheader,nounits",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        parts = [part.strip() for part in gpu_result.stdout.strip().split(",")]
        if len(parts) != 9:
            raise GpuGateError("nvidia-smi GPU receipt output is malformed")
        apps_result = subprocess.run(
            [
                "nvidia-smi",
                "--query-compute-apps=pid,gpu_uuid",
                "--format=csv,noheader,nounits",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        target_uuid = parts[1]
        compute_pids: list[int] = []
        for line in apps_result.stdout.splitlines():
            if not line.strip():
                continue
            app_parts = [part.strip() for part in line.split(",")]
            if len(app_parts) != 2:
                raise GpuGateError("nvidia-smi compute-app output is malformed")
            if app_parts[1] == target_uuid:
                compute_pids.append(int(app_parts[0]))
        receipt = {
            "schema_version": PARENT_GPU_RECEIPT_SCHEMA,
            "physical_device_index": int(parts[0]),
            "gpu_uuid": target_uuid,
            "device_name": parts[2],
            "total_memory_mib": int(parts[3]),
            "used_memory_mib": int(parts[4]),
            "free_memory_mib": int(parts[5]),
            "utilization_percent": int(parts[6]),
            "driver_version": parts[7],
            "compute_capability": parts[8],
            "compute_pids": sorted(compute_pids),
        }
    except (OSError, subprocess.CalledProcessError, ValueError) as error:
        raise GpuGateError(f"could not query frozen Genesis GPU: {error}") from error
    _validate_parent_gpu_receipt(receipt, allowed_compute_pids=allowed_compute_pids)
    return receipt


def _validate_child_gpu_receipt(
    receipt: Any, *, expected_worker_pid: int | None = None
) -> None:
    parent_fields = {
        "schema_version",
        *FROZEN_GENESIS_GPU_HARDWARE,
        "used_memory_mib",
        "free_memory_mib",
        "utilization_percent",
        "compute_pids",
    }
    if not isinstance(receipt, dict) or set(receipt) != parent_fields | {"worker_pid"}:
        raise GpuGateError("child GPU gate receipt fields are malformed")
    worker_pid = receipt["worker_pid"]
    if (
        not isinstance(worker_pid, int)
        or isinstance(worker_pid, bool)
        or worker_pid <= 0
    ):
        raise GpuGateError("child GPU gate worker PID is malformed")
    if expected_worker_pid is not None and worker_pid != expected_worker_pid:
        raise GpuGateError("child GPU gate worker PID does not match current process")
    if receipt["schema_version"] != CHILD_GPU_RECEIPT_SCHEMA:
        raise GpuGateError("child GPU gate receipt schema mismatch")
    parent_receipt = {
        key: copy.deepcopy(value)
        for key, value in receipt.items()
        if key != "worker_pid"
    }
    parent_receipt["schema_version"] = PARENT_GPU_RECEIPT_SCHEMA
    _validate_parent_gpu_receipt(parent_receipt, allowed_compute_pids={worker_pid})
    if not set(receipt["compute_pids"]).issubset({worker_pid}):
        raise GpuGateError("frozen Genesis GPU is occupied by an external process")


def _query_child_gpu_receipt(protocol: dict[str, Any]) -> dict[str, Any]:
    worker_pid = os.getpid()
    receipt = _query_parent_gpu_receipt(protocol, allowed_compute_pids={worker_pid})
    receipt["schema_version"] = CHILD_GPU_RECEIPT_SCHEMA
    receipt["worker_pid"] = worker_pid
    _validate_child_gpu_receipt(receipt, expected_worker_pid=worker_pid)
    return receipt


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _bounded_tail(value: str, limit: int = TAIL_LIMIT) -> str:
    return value[-limit:]


def _bounded_warnings(value: Any) -> list[dict[str, str]]:
    if not isinstance(value, list):
        return []
    bounded: list[dict[str, str]] = []
    for item in value[:WARNING_ENTRY_LIMIT]:
        if isinstance(item, dict):
            logger_name = str(item.get("logger", "adapter"))
            level = str(item.get("level", "WARNING"))
            message = str(item.get("message", ""))
        else:
            logger_name, level, message = "adapter", "WARNING", str(item)
        bounded.append(
            {
                "logger": logger_name[:WARNING_MESSAGE_LIMIT],
                "level": level[:WARNING_MESSAGE_LIMIT],
                "message": message[:WARNING_MESSAGE_LIMIT],
            }
        )
    return bounded


def _diagnostic_failure(
    *,
    reason: str,
    stage: str,
    operation: str,
    error: BaseException | None = None,
    message: str | None = None,
    joint_name: str | None = None,
    steps_completed: int = 0,
    observed_joint_count: int | None = None,
    supported_max_joint_count: int | None = None,
    full_measured_positions_q: Iterable[float] = (),
    sample_steps: Iterable[int] = (),
    normalized_positions: Iterable[float] = (),
) -> DiagnosticFailure:
    if error is None:
        exception_type = "RuntimeContractError"
        failure_message = _bounded_tail(str(message or reason))
    else:
        exception_type = type(error).__name__
        failure_message = _bounded_tail(str(error)) or exception_type
    return DiagnosticFailure(
        {
            "reason": reason,
            "stage": stage,
            "operation": operation,
            "joint_name": joint_name,
            "steps_completed": steps_completed,
            "exception_type": exception_type,
            "message": failure_message,
            "observed_joint_count": observed_joint_count,
            "supported_max_joint_count": supported_max_joint_count,
            "full_measured_positions_q": [
                float(value) for value in full_measured_positions_q
            ],
            "trajectory": {
                "sample_steps": list(sample_steps),
                "normalized_positions": [
                    float(value) for value in normalized_positions
                ],
            },
        }
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _expected_source_binding_receipt(row: dict[str, Any]) -> dict[str, Any]:
    package_receipt = row.get("package_binding")
    package_hash = row.get("package_content_manifest_sha256")
    if (
        not isinstance(package_receipt, dict)
        or not _sha256_text(package_hash)
        or package_receipt.get("content_manifest_sha256") != package_hash
        or not _sha256_text(row.get("urdf_sha256"))
        or not isinstance(row.get("package_relative_path"), str)
        or not row["package_relative_path"]
        or not isinstance(row.get("urdf_relative_path"), str)
        or not row["urdf_relative_path"]
    ):
        raise RuntimeContractError("manifest row package source binding is malformed")
    return {
        "schema_version": SOURCE_BINDING_RECEIPT_SCHEMA,
        "status": "pass",
        "urdf_relative_path": row["urdf_relative_path"],
        "urdf_sha256": row["urdf_sha256"],
        "package_relative_path": row["package_relative_path"],
        "package_content_manifest_sha256": package_hash,
        "package_binding": copy.deepcopy(package_receipt),
    }


def _validate_source_binding_receipt(
    receipt: Any, row: dict[str, Any], *, required: bool
) -> None:
    if receipt is None and not required:
        return
    if receipt != _expected_source_binding_receipt(row):
        raise RuntimeContractError(
            "worker source binding receipt does not match manifest row"
        )
    try:
        json.dumps(receipt, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise RuntimeContractError(
            "worker source binding receipt is not finite JSON"
        ) from error


def _validate_worker_source_binding(request: dict[str, Any]) -> dict[str, Any]:
    try:
        row = request["row"]
        dataset_root = Path(request["dataset_root"]).resolve(strict=True)
        package = (dataset_root / row["package_relative_path"]).resolve(strict=True)
        expected_urdf = (dataset_root / row["urdf_relative_path"]).resolve(strict=True)
        raw_urdf = Path(request["raw_urdf_path"]).resolve(strict=True)
        package.relative_to(dataset_root)
        expected_urdf.relative_to(package)
        raw_urdf.relative_to(package)
    except (KeyError, TypeError, OSError, ValueError) as error:
        raise RuntimeContractError(
            f"worker source binding path verification failed: {error}"
        ) from error
    if (
        not dataset_root.is_dir()
        or not package.is_dir()
        or raw_urdf != expected_urdf
        or not raw_urdf.is_file()
    ):
        raise RuntimeContractError("worker source binding path drift")
    try:
        current_package_binding = package_binding(package)
        current_urdf_sha256 = _sha256_file(raw_urdf)
    except (ManifestError, OSError, ValueError) as error:
        raise RuntimeContractError(
            f"worker source binding could not be rebuilt: {error}"
        ) from error
    relative_urdf = raw_urdf.relative_to(package).as_posix()
    package_urdf = next(
        (
            entry
            for entry in current_package_binding.get("files", [])
            if isinstance(entry, dict) and entry.get("path") == relative_urdf
        ),
        None,
    )
    observed = {
        "schema_version": SOURCE_BINDING_RECEIPT_SCHEMA,
        "status": "pass",
        "urdf_relative_path": row["urdf_relative_path"],
        "urdf_sha256": current_urdf_sha256,
        "package_relative_path": row["package_relative_path"],
        "package_content_manifest_sha256": current_package_binding.get(
            "content_manifest_sha256"
        ),
        "package_binding": current_package_binding,
    }
    expected = _expected_source_binding_receipt(row)
    if (
        package_urdf is None
        or package_urdf.get("sha256") != current_urdf_sha256
        or observed != expected
    ):
        raise RuntimeContractError("worker source binding drift from frozen manifest")
    return observed


def _git_receipt(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {"root": None, "head": None, "dirty": None, "status_sha256": None}
    try:
        root_result = subprocess.run(
            ["git", "-C", str(path), "rev-parse", "--show-toplevel"],
            check=True,
            capture_output=True,
            text=True,
        )
        root = Path(root_result.stdout.strip()).resolve()
        head = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        status = subprocess.run(
            [
                "git",
                "-C",
                str(root),
                "status",
                "--porcelain=v1",
                "--untracked-files=all",
            ],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError):
        return {"root": None, "head": None, "dirty": None, "status_sha256": None}
    return {
        "root": str(root),
        "head": head,
        "dirty": bool(status),
        "status_sha256": hashlib.sha256(status.encode("utf-8")).hexdigest(),
    }


def adapter_implementation_receipt(simulator: str) -> dict[str, Any]:
    """Describe installed adapter code without importing the simulator package."""
    if simulator not in ADAPTER_MODULES:
        raise RuntimeContractError(f"unknown simulator: {simulator}")
    module_name, distribution_names = ADAPTER_MODULES[simulator]
    distribution = None
    distribution_name = None
    for candidate in distribution_names:
        try:
            distribution = importlib.metadata.distribution(candidate)
            distribution_name = candidate
            break
        except importlib.metadata.PackageNotFoundError:
            continue
    if distribution is None or distribution_name is None:
        raise RuntimeContractError(
            f"adapter distribution is not installed: {simulator}"
        )
    files = list(distribution.files or [])

    def distribution_file(name: str) -> Path | None:
        matches = [
            distribution.locate_file(item).resolve()
            for item in files
            if str(item).endswith(f".dist-info/{name}")
        ]
        return matches[0] if len(matches) == 1 and matches[0].is_file() else None

    metadata_path = distribution_file("METADATA")
    record_path = distribution_file("RECORD")
    direct_url_path = distribution_file("direct_url.json")
    if metadata_path is None or record_path is None:
        raise RuntimeContractError(
            f"adapter distribution receipt is incomplete: {simulator}"
        )
    spec = importlib.util.find_spec(module_name)
    if spec is None or spec.origin is None:
        raise RuntimeContractError(
            f"adapter module origin is unavailable: {module_name}"
        )
    module_origin = Path(spec.origin).resolve()
    if not module_origin.is_file():
        raise RuntimeContractError(
            f"adapter module origin is not a file: {module_origin}"
        )
    evaluator_source = Path(__file__).resolve()
    direct_url: Any = None
    direct_url_sha256: str | None = None
    if direct_url_path is not None:
        direct_url_sha256 = _sha256_file(direct_url_path)
        try:
            direct_url = json.loads(direct_url_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as error:
            raise RuntimeContractError(
                f"adapter direct_url.json is malformed: {simulator}"
            ) from error
    editable_root: Path | None = None
    if (
        isinstance(direct_url, dict)
        and direct_url.get("dir_info", {}).get("editable") is True
    ):
        parsed_url = urllib.parse.urlparse(str(direct_url.get("url", "")))
        if parsed_url.scheme != "file":
            raise RuntimeContractError(
                f"editable adapter direct URL is not local: {simulator}"
            )
        editable_root = Path(urllib.parse.unquote(parsed_url.path)).resolve()
    receipt = {
        "schema_version": IMPLEMENTATION_RECEIPT_SCHEMA,
        "simulator": simulator,
        "distribution_name": distribution_name,
        "distribution_version": distribution.version,
        "distribution_metadata_sha256": _sha256_file(metadata_path),
        "distribution_record_sha256": _sha256_file(record_path),
        "direct_url": direct_url,
        "direct_url_sha256": direct_url_sha256,
        "module_name": module_name,
        "module_origin": str(module_origin),
        "module_origin_sha256": _sha256_file(module_origin),
        "evaluator_source": str(evaluator_source),
        "evaluator_source_sha256": _sha256_file(evaluator_source),
        "git": _git_receipt(editable_root),
    }
    try:
        json.dumps(receipt, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise RuntimeContractError(
            f"adapter implementation receipt is not finite JSON: {error}"
        ) from error
    return receipt


def validate_runtime_output_root(
    receipt_root: Path, phase: str, simulator: str, output_root: Path
) -> Path:
    if phase not in {"qualification", "formal"} or simulator not in ADAPTER_NAMES:
        raise RuntimeContractError("invalid simulator or phase")
    expected = (Path(receipt_root).resolve() / phase / simulator).resolve()
    actual = Path(output_root).resolve()
    if actual != expected:
        raise RuntimeContractError(
            f"runtime output root must be exactly <receipt-root>/{phase}/{simulator}: {expected}"
        )
    return actual


def _at_most(value: float, threshold: float) -> bool:
    return value <= threshold


def _at_least(value: float, threshold: float) -> bool:
    return value >= threshold


def _vector(value: Any, length: int) -> list[float]:
    if (
        not isinstance(value, (list, tuple))
        or len(value) != length
        or not all(_finite_number(item) for item in value)
    ):
        raise RuntimeContractError(f"expected finite vector of length {length}")
    return [float(item) for item in value]


def _quat_normalized(value: Any) -> list[float]:
    quaternion = _vector(value, 4)
    norm = math.sqrt(sum(item * item for item in quaternion))
    if not math.isfinite(norm) or norm <= 0:
        raise RuntimeContractError("quaternion norm must be finite and positive")
    return [item / norm for item in quaternion]


def _quat_conjugate(value: list[float]) -> list[float]:
    return [value[0], -value[1], -value[2], -value[3]]


def _quat_multiply(left: list[float], right: list[float]) -> list[float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return [
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ]


def _quat_rotate(value: list[float], vector: list[float]) -> list[float]:
    rotated = _quat_multiply(
        _quat_multiply(value, [0.0, *vector]), _quat_conjugate(value)
    )
    return rotated[1:]


def quaternion_angular_error(left: Any, right: Any) -> float:
    lhs, rhs = _quat_normalized(left), _quat_normalized(right)
    dot = abs(sum(a * b for a, b in zip(lhs, rhs)))
    return 2.0 * math.acos(max(0.0, min(1.0, dot)))


def relative_link_poses(
    world_poses: dict[str, dict[str, list[float]]], root_name: str
) -> dict[str, dict[str, list[float]]]:
    if root_name not in world_poses:
        raise RuntimeContractError(f"root link pose missing: {root_name}")
    root_translation = _vector(world_poses[root_name].get("translation"), 3)
    root_rotation = _quat_normalized(world_poses[root_name].get("rotation"))
    inverse = _quat_conjugate(root_rotation)
    aligned: dict[str, dict[str, list[float]]] = {}
    for name, pose in world_poses.items():
        translation = _vector(pose.get("translation"), 3)
        rotation = _quat_normalized(pose.get("rotation"))
        delta = [translation[index] - root_translation[index] for index in range(3)]
        aligned[name] = {
            "translation": _quat_rotate(inverse, delta),
            "rotation": _quat_normalized(_quat_multiply(inverse, rotation)),
        }
    return aligned


def minimum_jerk(progress: float) -> float:
    progress = max(0.0, min(1.0, float(progress)))
    return 10.0 * progress**3 - 15.0 * progress**4 + 6.0 * progress**5


def _unsupported_reason(joint: dict[str, Any], mapped_names: set[str]) -> str | None:
    if joint.get("type") not in {"revolute", "prismatic"}:
        return "unsupported_joint_type"
    lower, upper = joint.get("lower"), joint.get("upper")
    if not _finite_number(lower) or not _finite_number(upper):
        return "missing_or_nonfinite_limit"
    if float(lower) >= float(upper):
        return "nonpositive_range"
    effort = joint.get("effort")
    if not _finite_number(effort) or float(effort) <= 0:
        return "missing_or_nonpositive_effort"
    velocity = joint.get("velocity")
    if not _finite_number(velocity) or float(velocity) <= 0:
        return "missing_or_nonpositive_velocity"
    if joint.get("name") not in mapped_names:
        return "missing_runtime_mapping"
    return None


def classify_support(
    joints: Iterable[dict[str, Any]], mapped_names: Iterable[str]
) -> dict[str, dict[str, Any]]:
    mapped = set(mapped_names)
    support: dict[str, dict[str, Any]] = {}
    for joint in joints:
        reason = _unsupported_reason(joint, mapped)
        support[joint["name"]] = {
            "name": joint["name"],
            "type": joint.get("type"),
            "parent": joint.get("parent"),
            "child": joint.get("child"),
            "lower": joint.get("lower"),
            "upper": joint.get("upper"),
            "effort": joint.get("effort"),
            "velocity": joint.get("velocity"),
            "runtime_mapped": joint["name"] in mapped,
            "eligible": reason is None,
            "unsupported_reason": reason,
        }
    return support


def _eligible(support: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in support.values() if row["eligible"]]


def _midpoint(joint: dict[str, Any]) -> float:
    return (float(joint["lower"]) + float(joint["upper"])) / 2.0


def _finite_state(state: Any, names: Iterable[str]) -> bool:
    if not isinstance(state, dict):
        return False
    return all(
        name in state
        and isinstance(state[name], dict)
        and _finite_number(state[name].get("q"))
        and _finite_number(state[name].get("qdot"))
        for name in names
    )


def settling_result(
    states: list[dict[str, dict[str, float]]],
    support: dict[str, dict[str, Any]],
    *,
    mapped_names: Iterable[str] | None = None,
    max_displacement: float,
    max_speed: float,
) -> dict[str, Any]:
    supported = _eligible(support)
    maximum_displacement = 0.0
    maximum_speed = 0.0
    mapped = list(mapped_names if mapped_names is not None else support)
    finite = all(_finite_state(state, mapped) for state in states)
    displacement_within = True
    speed_within = True
    for state in states:
        for joint in supported:
            sample = state.get(joint["name"], {})
            if not _finite_number(sample.get("q")) or not _finite_number(
                sample.get("qdot")
            ):
                finite = False
                continue
            span = float(joint["upper"]) - float(joint["lower"])
            midpoint = _midpoint(joint)
            displacement = abs(float(sample["q"]) - midpoint) / span
            speed = abs(float(sample["qdot"])) / span
            lower_bound = midpoint - max_displacement * span
            upper_bound = midpoint + max_displacement * span
            sample_displacement_within = (
                lower_bound <= float(sample["q"]) <= upper_bound
            )
            sample_speed_within = (
                -max_speed * span <= float(sample["qdot"]) <= max_speed * span
            )
            displacement_within &= sample_displacement_within
            speed_within &= sample_speed_within
            maximum_displacement = max(maximum_displacement, displacement)
            maximum_speed = max(maximum_speed, speed)
    passed = finite and displacement_within and speed_within
    return {
        "passed": passed,
        "finite": finite,
        "max_normalized_displacement": maximum_displacement,
        "max_normalized_speed_per_s": maximum_speed,
        "state_trace": copy.deepcopy(states),
    }


def actuation_result(
    positions: list[float], joint: dict[str, Any], *, minimum_fraction: float
) -> dict[str, Any]:
    finite = bool(positions) and all(_finite_number(value) for value in positions)
    span = float(joint["upper"]) - float(joint["lower"])
    realized = (max(positions) - min(positions)) / span if finite else None
    return {
        "passed": bool(
            finite and realized is not None and _at_least(realized, minimum_fraction)
        ),
        "finite": finite,
        "realized_range_fraction": realized,
        "minimum_q": min(positions) if finite else None,
        "maximum_q": max(positions) if finite else None,
    }


def limit_result(positions: list[float], joint: dict[str, Any]) -> dict[str, Any]:
    span = float(joint["upper"]) - float(joint["lower"])
    tolerance = max(1e-6, 1e-3 * span)
    finite = bool(positions) and all(_finite_number(value) for value in positions)
    passed = bool(
        finite
        and all(
            _at_least(value, float(joint["lower"]) - tolerance)
            and _at_most(value, float(joint["upper"]) + tolerance)
            for value in positions
        )
    )
    return {
        "passed": passed,
        "finite": finite,
        "tolerance": tolerance,
        "minimum_q": min(positions) if finite else None,
        "maximum_q": max(positions) if finite else None,
    }


def drift_result(
    pose_pairs: list[tuple[list[float], list[float], list[float], list[float]]],
    bounding_box_diagonal: float,
    translation_threshold: float,
    rotation_threshold: float,
) -> dict[str, Any]:
    if (
        not _finite_number(bounding_box_diagonal)
        or float(bounding_box_diagonal) <= 0
        or not pose_pairs
    ):
        return {
            "passed": False,
            "finite": False,
            "max_translation_over_bbox_diagonal": None,
            "max_rotation_error_rad": None,
            "steps_compared": len(pose_pairs),
            "normalized_translation_errors": [],
            "rotation_errors_rad": [],
        }
    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    translation_within = True
    rotation_within = True
    try:
        for (
            expected_translation,
            expected_rotation,
            observed_translation,
            observed_rotation,
        ) in pose_pairs:
            lhs, rhs = _vector(expected_translation, 3), _vector(
                observed_translation, 3
            )
            delta_norm = math.sqrt(sum((a - b) ** 2 for a, b in zip(lhs, rhs)))
            normalized_translation = delta_norm / float(bounding_box_diagonal)
            lhs_q, rhs_q = _quat_normalized(expected_rotation), _quat_normalized(
                observed_rotation
            )
            quaternion_dot = abs(sum(a * b for a, b in zip(lhs_q, rhs_q)))
            angular_error = 2.0 * math.acos(max(0.0, min(1.0, quaternion_dot)))
            translation_within &= delta_norm <= translation_threshold * float(
                bounding_box_diagonal
            )
            rotation_within &= quaternion_dot >= math.cos(rotation_threshold / 2.0)
            translation_errors.append(normalized_translation)
            rotation_errors.append(angular_error)
    except RuntimeContractError:
        return {
            "passed": False,
            "finite": False,
            "max_translation_over_bbox_diagonal": None,
            "max_rotation_error_rad": None,
            "steps_compared": len(pose_pairs),
            "normalized_translation_errors": [],
            "rotation_errors_rad": [],
        }
    maximum_translation = max(translation_errors)
    maximum_rotation = max(rotation_errors)
    return {
        "passed": translation_within and rotation_within,
        "finite": True,
        "max_translation_over_bbox_diagonal": maximum_translation,
        "max_rotation_error_rad": maximum_rotation,
        "steps_compared": len(pose_pairs),
        "normalized_translation_errors": translation_errors,
        "rotation_errors_rad": rotation_errors,
    }


def _controller_efforts(
    state: dict[str, dict[str, float]],
    supported: list[dict[str, Any]],
    target_name: str,
    target_normalized: float,
    protocol: dict[str, Any],
) -> dict[str, float]:
    controller = protocol["runtime"]["actuation"]["effort_controller"]
    lower_clip, upper_clip = controller["clip"]
    efforts: dict[str, float] = {}
    for joint in supported:
        name = joint["name"]
        sample = state.get(name, {})
        if not _finite_number(sample.get("q")) or not _finite_number(
            sample.get("qdot")
        ):
            raise RuntimeContractError(
                f"non-finite or missing controlled state: {name}"
            )
        span = float(joint["upper"]) - float(joint["lower"])
        normalized_position = (float(sample["q"]) - float(joint["lower"])) / span
        normalized_speed = float(sample["qdot"]) / float(joint["velocity"])
        desired = target_normalized if name == target_name else 0.5
        command = (
            controller["kp"] * (desired - normalized_position)
            - controller["kd"] * normalized_speed
        )
        efforts[name] = float(joint["effort"]) * max(
            lower_clip, min(upper_clip, command)
        )
    return efforts


def fk_link_pose_branch(
    joint_tree: dict[str, Any], scalar_positions: dict[str, float], link_name: str
) -> tuple[dict[str, list[float]] | None, list[str] | None]:
    """Evaluate only the root-to-link chain, excluding unrelated unsupported branches."""
    try:
        roots = set(joint_tree["root_links"])
        if link_name in roots:
            return {
                "translation": [0.0, 0.0, 0.0],
                "rotation": [1.0, 0.0, 0.0, 0.0],
            }, None
        by_child = {joint["child"]: joint for joint in joint_tree["joints"]}
        path: list[dict[str, Any]] = []
        current = link_name
        seen: set[str] = set()
        while current not in roots:
            if current in seen or current not in by_child:
                return None, [f"disconnected:{current}"]
            seen.add(current)
            joint = by_child[current]
            path.append(joint)
            current = joint["parent"]
        unsupported = sorted(
            joint["name"] for joint in path if not joint.get("fk_supported", False)
        )
        if unsupported:
            return None, unsupported
        ordered = list(reversed(path))
        links = {current}
        for joint in ordered:
            links.add(joint["parent"])
            links.add(joint["child"])
        branch = {"links": sorted(links), "root_links": [current], "joints": ordered}
        return fk_link_poses(branch, scalar_positions).get(link_name), None
    except (ManifestError, RuntimeContractError, ValueError, TypeError):
        return None, ["fk_error"]


def _expected_pose(
    joint_tree: dict[str, Any], scalar_positions: dict[str, float], child_name: str
) -> tuple[dict[str, list[float]] | None, list[str] | None]:
    return fk_link_pose_branch(joint_tree, scalar_positions, child_name)


def _descendant_names(joint_tree: dict[str, Any], child_name: str) -> list[str]:
    reachable = {child_name}
    while True:
        expanded = reachable | {
            joint["child"]
            for joint in joint_tree["joints"]
            if joint.get("parent") in reachable
        }
        if expanded == reachable:
            break
        reachable = expanded
    return sorted(reachable)


def _strict_load(
    adapter: Adapter, row: dict[str, Any], supported_names: list[str]
) -> dict[str, Any]:
    expected_links = sorted(row["joint_tree"]["links"])
    expected_joints = sorted(joint["name"] for joint in row["joint_tree"]["joints"])
    observed_links = sorted(adapter.observed_link_names)
    observed_joints = sorted(adapter.observed_joint_names)
    expected_fixed = sorted(
        joint["name"]
        for joint in row["joint_tree"]["joints"]
        if joint["type"] == "fixed"
    )
    missing_links = sorted(
        (Counter(expected_links) - Counter(observed_links)).elements()
    )
    extra_links = sorted((Counter(observed_links) - Counter(expected_links)).elements())
    missing_joints = sorted(
        (Counter(expected_joints) - Counter(observed_joints)).elements()
    )
    extra_joints = sorted(
        (Counter(observed_joints) - Counter(expected_joints)).elements()
    )
    strict = observed_links == expected_links and observed_joints == expected_joints
    return {
        "created": True,
        "expected_link_names": expected_links,
        "observed_link_names": observed_links,
        "missing_link_names": missing_links,
        "extra_link_names": extra_links,
        "expected_joint_names": expected_joints,
        "observed_joint_names": observed_joints,
        "missing_joint_names": missing_joints,
        "extra_joint_names": extra_joints,
        "expected_fixed_joint_names": expected_fixed,
        "missing_fixed_joint_names": sorted(set(expected_fixed) - set(observed_joints)),
        "fixed_structure_preserved": not bool(
            set(expected_fixed) - set(observed_joints)
        ),
        "mapped_supported_joint_names": sorted(supported_names),
        "mapped_scalar_joint_names": sorted(adapter.mapped_joint_names),
        "strict_load": strict,
    }


def evaluate_asset(
    adapter: Adapter, row: dict[str, Any], protocol: dict[str, Any]
) -> dict[str, Any]:
    """Run the frozen evaluator through an adapter normalized by URDF names."""
    movable = [
        joint for joint in row["joint_tree"]["joints"] if joint.get("type") != "fixed"
    ]
    support_map = classify_support(movable, adapter.mapped_joint_names)
    supported = _eligible(support_map)
    unsupported = [entry for entry in support_map.values() if not entry["eligible"]]
    supported_names = sorted(entry["name"] for entry in supported)
    load = _strict_load(adapter, row, supported_names)
    midpoints = {joint["name"]: _midpoint(joint) for joint in supported}
    reset_rows: list[dict[str, Any]] = []
    reset_pass = True
    for repetition in range(1, int(protocol["runtime"]["reset_repetitions"]) + 1):
        try:
            adapter.reset(midpoints)
        except Exception as error:
            raise _diagnostic_failure(
                reason="simulator_diagnostic_interrupted",
                stage="reset",
                operation="reset",
                error=error,
            ) from error
        try:
            state = adapter.state()
        except Exception as error:
            raise _diagnostic_failure(
                reason="simulator_diagnostic_interrupted",
                stage="reset",
                operation="state",
                error=error,
            ) from error
        finite = _finite_state(state, adapter.mapped_joint_names)
        reset_rows.append(
            {"repetition": repetition, "finite": finite, "state": state, "error": None}
        )
        reset_pass &= finite

    settling_states: list[dict[str, dict[str, float]]] = []
    try:
        adapter.reset(midpoints)
    except Exception as error:
        raise _diagnostic_failure(
            reason="simulator_diagnostic_interrupted",
            stage="settling",
            operation="reset",
            error=error,
        ) from error
    for _ in range(int(protocol["runtime"]["passive_settling"]["steps"])):
        try:
            adapter.step({name: 0.0 for name in supported_names})
        except Exception as error:
            raise _diagnostic_failure(
                reason="simulator_diagnostic_interrupted",
                stage="settling",
                operation="step",
                error=error,
                steps_completed=len(settling_states),
            ) from error
        try:
            settling_states.append(adapter.state())
        except Exception as error:
            raise _diagnostic_failure(
                reason="simulator_diagnostic_interrupted",
                stage="settling",
                operation="state",
                error=error,
                steps_completed=len(settling_states),
            ) from error
    settling = settling_result(
        settling_states,
        support_map,
        mapped_names=adapter.mapped_joint_names,
        max_displacement=float(
            protocol["metrics"]["settling"]["max_normalized_displacement"]
        ),
        max_speed=float(protocol["metrics"]["settling"]["max_normalized_speed_per_s"]),
    )
    if len(settling_states) != int(protocol["runtime"]["passive_settling"]["steps"]):
        settling["passed"] = False
    settling["steps_completed"] = len(settling_states)
    settling["error"] = None

    actuation_rows: list[dict[str, Any]] = []
    limit_rows: list[dict[str, Any]] = []
    actuation_pass = bool(supported) and not unsupported
    limit_pass = bool(supported) and not unsupported
    drift_pass = bool(supported) and not unsupported
    trajectory = protocol["runtime"]["actuation"]["trajectory"]
    sample_steps = protocol["cross_simulator"]["joint_rmse"]["sample_steps"]
    ramp_steps, hold_steps = int(trajectory["ramp_steps"]), int(
        trajectory["hold_steps"]
    )
    drift_thresholds = protocol["metrics"]["constraint_drift"]
    for tested in supported:
        tested_name = tested["name"]
        positions = dict(midpoints)
        positions[tested_name] = float(tested["lower"])
        samples: list[float] = []
        measured_positions: list[float] = []
        pose_pairs: list[tuple[list[float], list[float], list[float], list[float]]] = []
        final_poses: dict[str, dict[str, list[float]]] = {}
        unsupported_ancestors: set[str] = set()
        operation = "reset"
        try:
            adapter.reset(positions)
            operation = "state"
            initial = adapter.state()
            if not _finite_state(initial, supported_names):
                raise RuntimeContractError("non-finite initial controlled state")
            initial_q = float(initial[tested_name]["q"])
            measured_positions.append(initial_q)
            samples.append(
                (initial_q - float(tested["lower"]))
                / (float(tested["upper"]) - float(tested["lower"]))
            )
            for step in range(1, ramp_steps + hold_steps + 1):
                operation = "state"
                state_before = adapter.state()
                target = minimum_jerk(step / ramp_steps) if step <= ramp_steps else 1.0
                operation = "step"
                adapter.step(
                    _controller_efforts(
                        state_before, supported, tested_name, target, protocol
                    )
                )
                operation = "state"
                measured = adapter.state()
                if not _finite_state(measured, supported_names):
                    raise RuntimeContractError("non-finite controlled state")
                q = float(measured[tested_name]["q"])
                measured_positions.append(q)
                if step in sample_steps:
                    samples.append(
                        (q - float(tested["lower"]))
                        / (float(tested["upper"]) - float(tested["lower"]))
                    )
                expected, ancestor_errors = _expected_pose(
                    row["joint_tree"],
                    {name: float(measured[name]["q"]) for name in supported_names},
                    tested["child"],
                )
                if ancestor_errors:
                    unsupported_ancestors.update(ancestor_errors)
                try:
                    observed = adapter.link_poses().get(tested["child"])
                except Exception:
                    observed = None
                if expected is not None and observed is not None:
                    pose_pairs.append(
                        (
                            expected["translation"],
                            expected["rotation"],
                            observed.get("translation"),
                            observed.get("rotation"),
                        )
                    )
            try:
                final_poses = adapter.link_poses()
            except Exception:
                final_poses = {}
        except Exception as error:
            completed_steps = max(0, len(measured_positions) - 1)
            retained_sample_steps = (
                [step for step in sample_steps if step <= completed_steps]
                if measured_positions
                else []
            )
            raise _diagnostic_failure(
                reason="simulator_diagnostic_interrupted",
                stage="actuation",
                operation=operation,
                error=error,
                joint_name=tested_name,
                steps_completed=completed_steps,
                full_measured_positions_q=measured_positions,
                sample_steps=retained_sample_steps,
                normalized_positions=samples,
            ) from error
        actuation = actuation_result(
            measured_positions,
            tested,
            minimum_fraction=float(
                protocol["metrics"]["actuation"]["min_declared_range_fraction"]
            ),
        )
        if len(samples) != len(sample_steps):
            actuation["passed"] = False
        descendants = _descendant_names(row["joint_tree"], tested["child"])
        retained_poses = {
            name: final_poses[name] for name in descendants if name in final_poses
        }
        missing_descendants = sorted(set(descendants) - set(retained_poses))
        drift = drift_result(
            pose_pairs,
            float(row["bounding_box_diagonal"]),
            float(drift_thresholds["translation_over_bbox_diagonal_max"]),
            float(drift_thresholds["rotation_rad_max"]),
        )
        if len(pose_pairs) != ramp_steps + hold_steps or missing_descendants:
            drift["passed"] = False
        if unsupported_ancestors:
            drift["passed"] = False
        drift["unsupported_ancestor_joint_names"] = sorted(unsupported_ancestors)
        actuation_rows.append(
            {
                "joint_name": tested_name,
                "joint_type": tested["type"],
                **actuation,
                "steps_completed": ramp_steps + hold_steps,
                "full_measured_positions_q": measured_positions,
                "trajectory": {
                    "sample_steps": list(sample_steps),
                    "normalized_positions": samples,
                },
                "constraint_drift": drift,
                "final_descendant_root_frame_poses": retained_poses,
                "missing_descendant_link_names": missing_descendants,
                "error": None,
            }
        )
        actuation_pass &= actuation["passed"]
        drift_pass &= drift["passed"]

        tested_limit_rows: list[dict[str, Any]] = []
        for target in protocol["runtime"]["limit_enforcement"]["targets_normalized"]:
            observed_positions: list[float] = []
            operation = "reset"
            try:
                adapter.reset(midpoints)
                for _ in range(
                    int(protocol["runtime"]["limit_enforcement"]["steps_each"])
                ):
                    operation = "state"
                    before = adapter.state()
                    operation = "step"
                    adapter.step(
                        _controller_efforts(
                            before, supported, tested_name, float(target), protocol
                        )
                    )
                    operation = "state"
                    after = adapter.state()
                    if not _finite_state(after, supported_names):
                        raise RuntimeContractError("non-finite limit state")
                    observed_positions.append(float(after[tested_name]["q"]))
            except Exception as error:
                raise _diagnostic_failure(
                    reason="simulator_diagnostic_interrupted",
                    stage="limit_enforcement",
                    operation=operation,
                    error=error,
                    joint_name=tested_name,
                    steps_completed=len(observed_positions),
                    full_measured_positions_q=observed_positions,
                ) from error
            limit = limit_result(observed_positions, tested)
            if len(observed_positions) != int(
                protocol["runtime"]["limit_enforcement"]["steps_each"]
            ):
                limit["passed"] = False
            tested_limit_rows.append(
                {
                    "target_normalized": target,
                    **limit,
                    "steps_completed": len(observed_positions),
                    "error": None,
                }
            )
            limit_pass &= limit["passed"]
        limit_rows.append({"joint_name": tested_name, "targets": tested_limit_rows})

    metrics = {
        "load": bool(load["created"]),
        "reset": bool(reset_pass),
        "settling": bool(settling["passed"]),
        "actuation": bool(actuation_pass),
        "limit_enforcement": bool(limit_pass),
        "constraint_drift": bool(drift_pass),
    }
    metrics["simulator_pass"] = all(
        metrics[name] for name in protocol["metrics"]["simulator_pass"]["logical_and"]
    )
    warnings = list(getattr(adapter, "warnings", []))
    return {
        "evaluation_schema_version": EVALUATION_SCHEMA,
        "load": load,
        "support": {
            "joints": list(support_map.values()),
            "supported_joint_names": supported_names,
            "unsupported_joint_names": sorted(entry["name"] for entry in unsupported),
            "any_unsupported_movable_joint": bool(unsupported),
            "no_supported_movable_joint": not bool(supported),
        },
        "metrics": metrics,
        "diagnostics": {
            "reset": reset_rows,
            "settling": settling,
            "actuation": actuation_rows,
            "limit_enforcement": limit_rows,
            "warnings": warnings,
            "errors": [row["error"] for row in actuation_rows if row["error"]],
        },
    }


def adapter_identity(
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    row: dict[str, Any],
    simulator: str,
    phase: str,
    timeout_s: float | None = None,
    *,
    implementation_receipt: dict[str, Any],
    workers: int = 1,
) -> dict[str, Any]:
    if simulator not in ADAPTER_NAMES or phase not in {"qualification", "formal"}:
        raise RuntimeContractError("invalid simulator or phase")
    validate_runtime_protocol(protocol)
    _validate_row_identity(row)
    _preflight_eligible(row)
    _expected_source_binding_receipt(row)
    if not isinstance(workers, int) or isinstance(workers, bool) or workers < 1:
        raise RuntimeContractError("effective workers must be a positive integer")
    if simulator == "genesis" and workers != 1:
        raise RuntimeContractError("Genesis requires workers=1")
    protocol_timeout = float(protocol["runtime"]["child_timeout_s"])
    effective_timeout = protocol_timeout if timeout_s is None else float(timeout_s)
    if not math.isfinite(effective_timeout) or effective_timeout != protocol_timeout:
        raise RuntimeContractError(
            "effective child timeout must equal the frozen protocol value"
        )
    adapter_config = {
        "adapter": protocol["adapters"][simulator],
        "runtime": protocol["runtime"],
        "effective_child_timeout_s": effective_timeout,
        "effective_workers": workers,
    }
    _validate_implementation_receipt(
        implementation_receipt,
        simulator=simulator,
        expected_version=protocol["adapters"][simulator]["version"],
    )
    return {
        "dataset_id": row["dataset_id"],
        "asset_id": row["asset_id"],
        "manifest_root": row["manifest_root"],
        "order": row["order"],
        "selection_rank": row["selection_rank"],
        "selection_hash": row["selection_hash"],
        "simulator": simulator,
        "adapter_name": ADAPTER_NAMES[simulator],
        "adapter_version": protocol["adapters"][simulator]["version"],
        "adapter_config_sha256": canonical_sha256(adapter_config),
        "adapter_implementation_sha256": canonical_sha256(implementation_receipt),
        "parent_gpu_hardware_sha256": (
            canonical_sha256(FROZEN_GENESIS_GPU_HARDWARE)
            if simulator == "genesis"
            else None
        ),
        "urdf_sha256": row["urdf_sha256"],
        "package_content_manifest_sha256": row["package_content_manifest_sha256"],
        "effective_workers": workers,
        "protocol_sha256": manifest["protocol_sha256"],
        "cohort_sha256": manifest["cohort_sha256"],
        "manifest_row_sha256": canonical_sha256(row),
        "run_phase": phase,
    }


def _sha256_text(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _validate_implementation_receipt(
    receipt: Any, *, simulator: str, expected_version: str
) -> None:
    if (
        not isinstance(receipt, dict)
        or receipt.get("schema_version") != IMPLEMENTATION_RECEIPT_SCHEMA
    ):
        raise RuntimeContractError("invalid adapter implementation receipt schema")
    if (
        receipt.get("simulator") != simulator
        or receipt.get("distribution_version") != expected_version
    ):
        raise RuntimeContractError(
            "adapter implementation receipt simulator/version mismatch"
        )
    for field in (
        "distribution_name",
        "module_name",
        "module_origin",
        "evaluator_source",
    ):
        if not isinstance(receipt.get(field), str) or not receipt[field]:
            raise RuntimeContractError(
                f"adapter implementation receipt field is invalid: {field}"
            )
    for field in (
        "distribution_metadata_sha256",
        "distribution_record_sha256",
        "module_origin_sha256",
        "evaluator_source_sha256",
    ):
        if not _sha256_text(receipt.get(field)):
            raise RuntimeContractError(
                f"adapter implementation receipt hash is invalid: {field}"
            )
    direct_url, direct_url_hash = receipt.get("direct_url"), receipt.get(
        "direct_url_sha256"
    )
    if (direct_url is None) != (direct_url_hash is None) or (
        direct_url_hash is not None and not _sha256_text(direct_url_hash)
    ):
        raise RuntimeContractError(
            "adapter implementation direct-url receipt is invalid"
        )
    git = receipt.get("git")
    if not isinstance(git, dict) or set(git) != {
        "root",
        "head",
        "dirty",
        "status_sha256",
    }:
        raise RuntimeContractError("adapter implementation git receipt is invalid")
    absent = all(
        git[field] is None for field in ("root", "head", "dirty", "status_sha256")
    )
    present = (
        isinstance(git["root"], str)
        and bool(git["root"])
        and isinstance(git["head"], str)
        and bool(git["head"])
        and isinstance(git["dirty"], bool)
        and _sha256_text(git["status_sha256"])
    )
    if not absent and not present:
        raise RuntimeContractError("adapter implementation git receipt is incomplete")
    try:
        json.dumps(receipt, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise RuntimeContractError(
            f"adapter implementation receipt is not finite JSON: {error}"
        ) from error


def _source_contract(row: dict[str, Any]) -> dict[str, Any]:
    tree = row.get("joint_tree")
    if isinstance(tree, dict):
        links = tree["links"]
        joints = tree["joints"]
    else:
        links = row.get("link_names", [])
        joints = row.get("joints", [])
        if not isinstance(links, list) or not isinstance(joints, list):
            raise RuntimeContractError(
                "preflight-failed row source metadata is malformed"
            )
    return {
        "expected_link_names": sorted(links),
        "expected_joint_names": sorted(joint["name"] for joint in joints),
        "expected_fixed_joint_names": sorted(
            joint["name"] for joint in joints if joint["type"] == "fixed"
        ),
        "expected_movable_joint_names": sorted(
            joint["name"] for joint in joints if joint["type"] != "fixed"
        ),
        "bounding_box_diagonal": row.get("bounding_box_diagonal"),
    }


def _failure_evaluation(
    row: dict[str, Any], status: str, message: str
) -> dict[str, Any]:
    source = _source_contract(row)
    tree = row.get("joint_tree")
    joints = tree["joints"] if isinstance(tree, dict) else row.get("joints", [])
    movable = [joint for joint in joints if joint["type"] != "fixed"]
    support = classify_support(movable, set())
    metrics = {name: False for name in METRIC_NAMES}
    return {
        "load": {
            "created": False,
            "expected_link_names": source["expected_link_names"],
            "observed_link_names": [],
            "missing_link_names": source["expected_link_names"],
            "extra_link_names": [],
            "expected_joint_names": source["expected_joint_names"],
            "observed_joint_names": [],
            "missing_joint_names": source["expected_joint_names"],
            "extra_joint_names": [],
            "expected_fixed_joint_names": source["expected_fixed_joint_names"],
            "missing_fixed_joint_names": source["expected_fixed_joint_names"],
            "fixed_structure_preserved": False,
            "mapped_supported_joint_names": [],
            "mapped_scalar_joint_names": [],
            "strict_load": False,
        },
        "support": {
            "joints": list(support.values()),
            "supported_joint_names": [],
            "unsupported_joint_names": sorted(support),
            "any_unsupported_movable_joint": bool(support),
            "no_supported_movable_joint": True,
        },
        "metrics": metrics,
        "diagnostics": {
            "reset": [],
            "settling": {},
            "actuation": [],
            "limit_enforcement": [],
            "warnings": [],
            "errors": [f"{status}: {message}"],
        },
    }


def _valid_evaluation(
    response: Any,
    row: dict[str, Any],
    protocol: dict[str, Any],
    implementation_receipt: dict[str, Any],
) -> bool:
    if (
        not isinstance(response, dict)
        or response.get("evaluation_schema_version") != EVALUATION_SCHEMA
    ):
        return False
    if not all(
        isinstance(response.get(section), dict)
        for section in ("load", "support", "metrics", "diagnostics")
    ):
        return False
    metrics = response["metrics"]
    if set(metrics) != set(METRIC_NAMES) or not all(
        isinstance(metrics.get(name), bool) for name in METRIC_NAMES
    ):
        return False
    if response.get("adapter_implementation_receipt") != implementation_receipt:
        return False
    try:
        _validate_source_binding_receipt(
            response.get("source_binding_receipt"), row, required=True
        )
        _validate_device_receipt(
            response.get("device_receipt"),
            str(implementation_receipt["simulator"]),
            completed=True,
        )
        _validate_source_load_support(response, row)
        recomputed = _validate_completed_diagnostics(
            response["load"],
            response["support"],
            response["diagnostics"],
            protocol,
            row,
        )
        recomputed["simulator_pass"] = all(
            recomputed[name]
            for name in protocol["metrics"]["simulator_pass"]["logical_and"]
        )
        if any(metrics[name] is not recomputed[name] for name in METRIC_NAMES):
            raise RuntimeContractError("completed metrics do not match diagnostics")
        json.dumps(response, allow_nan=False)
    except (
        RuntimeContractError,
        KeyError,
        AttributeError,
        IndexError,
        TypeError,
        ValueError,
    ):
        return False
    return True


def _validate_evaluation_or_raise(
    response: dict[str, Any],
    row: dict[str, Any],
    protocol: dict[str, Any],
    implementation_receipt: dict[str, Any],
) -> None:
    if response.get("adapter_implementation_receipt") != implementation_receipt:
        raise RuntimeContractError("completed adapter implementation receipt mismatch")
    _validate_source_binding_receipt(
        response.get("source_binding_receipt"), row, required=True
    )
    _validate_device_receipt(
        response.get("device_receipt"),
        str(implementation_receipt["simulator"]),
        completed=True,
    )
    _validate_source_load_support(response, row)
    recomputed = _validate_completed_diagnostics(
        response["load"], response["support"], response["diagnostics"], protocol, row
    )
    recomputed["simulator_pass"] = all(
        recomputed[name]
        for name in protocol["metrics"]["simulator_pass"]["logical_and"]
    )
    if any(response["metrics"][name] is not recomputed[name] for name in METRIC_NAMES):
        raise RuntimeContractError("completed metrics do not match diagnostics")


def _validate_pose(pose: Any, location: str) -> None:
    if not isinstance(pose, dict):
        raise RuntimeContractError(f"invalid descendant pose at {location}")
    _vector(pose.get("translation"), 3)
    _quat_normalized(pose.get("rotation"))


def _validate_source_load_support(
    response: dict[str, Any], row: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    if response.get("source_contract") is not None:
        raise RuntimeContractError(
            "worker evaluation must not override the parent source contract"
        )
    load = response.get("load")
    support = response.get("support")
    if not isinstance(load, dict) or not isinstance(support, dict):
        raise RuntimeContractError("completed load/support sections are malformed")
    expected_links = sorted(row["joint_tree"]["links"])
    expected_joints = sorted(joint["name"] for joint in row["joint_tree"]["joints"])
    expected_fixed = sorted(
        joint["name"]
        for joint in row["joint_tree"]["joints"]
        if joint["type"] == "fixed"
    )
    observed_links = load.get("observed_link_names")
    observed_joints = load.get("observed_joint_names")
    mapped_scalar = load.get("mapped_scalar_joint_names")
    if not all(
        isinstance(values, list) and all(isinstance(name, str) for name in values)
        for values in (observed_links, observed_joints, mapped_scalar)
    ):
        raise RuntimeContractError("completed load name lists are malformed")
    if (
        observed_links != sorted(set(observed_links))
        or observed_joints != sorted(set(observed_joints))
        or mapped_scalar != sorted(set(mapped_scalar))
    ):
        raise RuntimeContractError(
            "completed runtime names are not canonical unique sets"
        )
    movable = [
        joint for joint in row["joint_tree"]["joints"] if joint["type"] != "fixed"
    ]
    expected_support = classify_support(movable, mapped_scalar)
    supported_names = sorted(
        entry["name"] for entry in expected_support.values() if entry["eligible"]
    )
    unsupported_names = sorted(
        entry["name"] for entry in expected_support.values() if not entry["eligible"]
    )
    expected_load = {
        "created": True,
        "expected_link_names": expected_links,
        "observed_link_names": observed_links,
        "missing_link_names": sorted(
            (Counter(expected_links) - Counter(observed_links)).elements()
        ),
        "extra_link_names": sorted(
            (Counter(observed_links) - Counter(expected_links)).elements()
        ),
        "expected_joint_names": expected_joints,
        "observed_joint_names": observed_joints,
        "missing_joint_names": sorted(
            (Counter(expected_joints) - Counter(observed_joints)).elements()
        ),
        "extra_joint_names": sorted(
            (Counter(observed_joints) - Counter(expected_joints)).elements()
        ),
        "expected_fixed_joint_names": expected_fixed,
        "missing_fixed_joint_names": sorted(set(expected_fixed) - set(observed_joints)),
        "fixed_structure_preserved": not bool(
            set(expected_fixed) - set(observed_joints)
        ),
        "mapped_supported_joint_names": supported_names,
        "mapped_scalar_joint_names": mapped_scalar,
        "strict_load": observed_links == expected_links
        and observed_joints == expected_joints,
    }
    if load != expected_load:
        raise RuntimeContractError("completed load does not match the manifest row")
    expected_support_section = {
        "joints": list(expected_support.values()),
        "supported_joint_names": supported_names,
        "unsupported_joint_names": unsupported_names,
        "any_unsupported_movable_joint": bool(unsupported_names),
        "no_supported_movable_joint": not bool(supported_names),
    }
    if support != expected_support_section:
        raise RuntimeContractError(
            "completed support partition does not match the manifest row"
        )
    return expected_support


def _validate_completed_diagnostics(
    load: dict[str, Any],
    support: dict[str, Any],
    diagnostics: dict[str, Any],
    protocol: dict[str, Any],
    source_row: dict[str, Any],
) -> dict[str, bool]:
    supported_names = support.get("supported_joint_names")
    unsupported_names = support.get("unsupported_joint_names")
    if not isinstance(supported_names, list) or not all(
        isinstance(name, str) for name in supported_names
    ):
        raise RuntimeContractError("invalid supported joint names")
    if not isinstance(unsupported_names, list) or not all(
        isinstance(name, str) for name in unsupported_names
    ):
        raise RuntimeContractError("invalid unsupported joint names")
    if (
        supported_names != sorted(set(supported_names))
        or unsupported_names != sorted(set(unsupported_names))
        or set(supported_names) & set(unsupported_names)
    ):
        raise RuntimeContractError("invalid support partition")
    support_by_name = {
        entry["name"]: entry
        for entry in support.get("joints", [])
        if isinstance(entry, dict) and isinstance(entry.get("name"), str)
    }
    if set(support_by_name) != set(supported_names) | set(unsupported_names):
        raise RuntimeContractError("invalid support rows")
    reset_rows = diagnostics.get("reset")
    settling = diagnostics.get("settling")
    if not isinstance(reset_rows, list) or len(reset_rows) != int(
        protocol["runtime"]["reset_repetitions"]
    ):
        raise RuntimeContractError(
            "completed reset diagnostics have wrong repetition count"
        )
    mapped_names = load["mapped_scalar_joint_names"]
    reset_pass = True
    for repetition, reset_row in enumerate(reset_rows, start=1):
        if not isinstance(reset_row, dict) or reset_row.get("repetition") != repetition:
            raise RuntimeContractError("completed reset diagnostic row is malformed")
        expected_finite = _finite_state(reset_row.get("state"), mapped_names)
        if reset_row.get("finite") is not expected_finite:
            raise RuntimeContractError("completed reset finite flag is inconsistent")
        error = reset_row.get("error")
        if error is not None and not isinstance(error, str):
            raise RuntimeContractError("completed reset error is malformed")
        reset_pass &= expected_finite and error is None
    passive_steps = int(protocol["runtime"]["passive_settling"]["steps"])
    if (
        not isinstance(settling, dict)
        or settling.get("steps_completed") != passive_steps
    ):
        raise RuntimeContractError(
            "completed settling diagnostics have wrong step count"
        )
    if set(settling) != {
        "passed",
        "finite",
        "max_normalized_displacement",
        "max_normalized_speed_per_s",
        "state_trace",
        "steps_completed",
        "error",
    }:
        raise RuntimeContractError("completed settling diagnostic fields are malformed")
    state_trace = settling["state_trace"]
    if not isinstance(state_trace, list) or len(state_trace) != passive_steps:
        raise RuntimeContractError(
            "completed record must retain the full settling state trace"
        )
    recomputed_settling = settling_result(
        state_trace,
        support_by_name,
        mapped_names=mapped_names,
        max_displacement=float(
            protocol["metrics"]["settling"]["max_normalized_displacement"]
        ),
        max_speed=float(protocol["metrics"]["settling"]["max_normalized_speed_per_s"]),
    )
    for field in (
        "finite",
        "passed",
        "max_normalized_displacement",
        "max_normalized_speed_per_s",
    ):
        if settling[field] != recomputed_settling[field]:
            raise RuntimeContractError(
                "completed settling summary does not match full state trace"
            )
    if settling["error"] is not None:
        raise RuntimeContractError("completed settling error must be null")
    settling_expected = bool(recomputed_settling["passed"])
    actuation_rows = diagnostics.get("actuation")
    limit_rows = diagnostics.get("limit_enforcement")
    if not isinstance(actuation_rows, list) or not isinstance(limit_rows, list):
        raise RuntimeContractError("invalid completed diagnostic lists")
    if sorted(
        row.get("joint_name") for row in actuation_rows if isinstance(row, dict)
    ) != sorted(supported_names):
        raise RuntimeContractError(
            "completed actuation diagnostics do not match supported joints"
        )
    if sorted(
        row.get("joint_name") for row in limit_rows if isinstance(row, dict)
    ) != sorted(supported_names):
        raise RuntimeContractError(
            "completed limit diagnostics do not match supported joints"
        )
    exact_steps = list(range(0, 361, 12))
    actuation_pass = bool(supported_names) and not bool(unsupported_names)
    drift_pass = bool(supported_names) and not bool(unsupported_names)
    for row in actuation_rows:
        if not isinstance(row, dict):
            raise RuntimeContractError("completed actuation row is malformed")
        joint = support_by_name[row["joint_name"]]
        trajectory = row.get("trajectory")
        if (
            not isinstance(trajectory, dict)
            or trajectory.get("sample_steps") != exact_steps
        ):
            raise RuntimeContractError(
                "completed record must retain the exact 31 trajectory sample steps"
            )
        samples = trajectory.get("normalized_positions")
        if (
            not isinstance(samples, list)
            or len(samples) != 31
            or not all(_finite_number(value) for value in samples)
        ):
            raise RuntimeContractError(
                "completed record must retain 31 trajectory samples"
            )
        full_positions = row.get("full_measured_positions_q")
        if (
            not isinstance(full_positions, list)
            or len(full_positions) != 361
            or not all(_finite_number(value) for value in full_positions)
        ):
            raise RuntimeContractError(
                "completed record must retain 361 measured positions"
            )
        if row.get("steps_completed") != 360:
            raise RuntimeContractError(
                "completed actuation diagnostics have wrong step count"
            )
        for field in ("passed", "finite"):
            if not isinstance(row.get(field), bool):
                raise RuntimeContractError("completed actuation flags are malformed")
        reported_realized = row.get("realized_range_fraction")
        reported_minimum_q = row.get("minimum_q")
        reported_maximum_q = row.get("maximum_q")
        if not all(
            _finite_number(value)
            for value in (reported_realized, reported_minimum_q, reported_maximum_q)
        ):
            raise RuntimeContractError("completed actuation range is malformed")
        span = float(joint["upper"]) - float(joint["lower"])
        minimum_q = min(float(value) for value in full_positions)
        maximum_q = max(float(value) for value in full_positions)
        realized = (maximum_q - minimum_q) / span
        if (
            float(reported_minimum_q) != minimum_q
            or float(reported_maximum_q) != maximum_q
            or float(reported_realized) != realized
        ):
            raise RuntimeContractError(
                "completed actuation range does not match full trace"
            )
        expected_samples = [
            (float(full_positions[step]) - float(joint["lower"])) / span
            for step in exact_steps
        ]
        if samples != expected_samples:
            raise RuntimeContractError(
                "completed sparse trajectory does not match full trace"
            )
        expected_actuation = bool(
            row["finite"]
            and realized
            >= float(protocol["metrics"]["actuation"]["min_declared_range_fraction"])
            and row.get("error") is None
        )
        if row["passed"] is not expected_actuation:
            raise RuntimeContractError("completed actuation pass flag is inconsistent")
        actuation_pass &= row["passed"]
        poses = row.get("final_descendant_root_frame_poses")
        missing = row.get("missing_descendant_link_names")
        if (
            not isinstance(poses, dict)
            or not isinstance(missing, list)
            or not all(isinstance(name, str) for name in missing)
        ):
            raise RuntimeContractError(
                "completed record has invalid descendant pose surface"
            )
        if set(poses) & set(missing) or len(set(missing)) != len(missing):
            raise RuntimeContractError(
                "completed record has conflicting descendant pose mappings"
            )
        descendants = set(_descendant_names(source_row["joint_tree"], joint["child"]))
        if set(poses) | set(missing) != descendants:
            raise RuntimeContractError(
                "completed descendant pose partition does not match the source topology"
            )
        for link_name, pose in poses.items():
            if not isinstance(link_name, str):
                raise RuntimeContractError(
                    "completed record has invalid descendant pose name"
                )
            _validate_pose(pose, link_name)
        drift = row.get("constraint_drift")
        if not isinstance(drift, dict) or set(drift) != {
            "passed",
            "finite",
            "max_translation_over_bbox_diagonal",
            "max_rotation_error_rad",
            "steps_compared",
            "normalized_translation_errors",
            "rotation_errors_rad",
            "unsupported_ancestor_joint_names",
        }:
            raise RuntimeContractError(
                "completed record has invalid constraint drift diagnostic"
            )
        unsupported_ancestors = drift.get("unsupported_ancestor_joint_names")
        if not isinstance(unsupported_ancestors, list) or not all(
            isinstance(name, str) for name in unsupported_ancestors
        ):
            raise RuntimeContractError(
                "completed drift ancestor diagnostics are malformed"
            )
        translation_trace = drift["normalized_translation_errors"]
        rotation_trace = drift["rotation_errors_rad"]
        if not (
            isinstance(translation_trace, list)
            and isinstance(rotation_trace, list)
            and len(translation_trace) == 360
            and len(rotation_trace) == 360
            and all(
                _finite_number(value) and float(value) >= 0.0
                for value in translation_trace
            )
            and all(
                _finite_number(value) and float(value) >= 0.0
                for value in rotation_trace
            )
        ):
            raise RuntimeContractError(
                "completed record must retain 360 finite drift error samples"
            )
        translation_error = drift.get("max_translation_over_bbox_diagonal")
        rotation_error = drift.get("max_rotation_error_rad")
        recomputed_translation = max(float(value) for value in translation_trace)
        recomputed_rotation = max(float(value) for value in rotation_trace)
        if (
            drift.get("finite") is not True
            or translation_error != recomputed_translation
            or rotation_error != recomputed_rotation
            or drift.get("steps_compared") != 360
        ):
            raise RuntimeContractError(
                "completed drift summary does not match full error traces"
            )
        expected_drift = bool(
            recomputed_translation
            <= float(
                protocol["metrics"]["constraint_drift"][
                    "translation_over_bbox_diagonal_max"
                ]
            )
            and recomputed_rotation
            <= float(protocol["metrics"]["constraint_drift"]["rotation_rad_max"])
            and not missing
            and not unsupported_ancestors
            and row.get("error") is None
        )
        if drift["passed"] is not expected_drift:
            raise RuntimeContractError("completed drift pass flag is inconsistent")
        drift_pass &= drift["passed"]
    limit_pass = bool(supported_names) and not bool(unsupported_names)
    for row in limit_rows:
        if not isinstance(row, dict):
            raise RuntimeContractError("completed limit row is malformed")
        targets = row.get("targets")
        if (
            not isinstance(targets, list)
            or len(targets) != 2
            or not all(isinstance(entry, dict) for entry in targets)
            or [entry.get("target_normalized") for entry in targets] != [-0.1, 1.1]
        ):
            raise RuntimeContractError(
                "completed record has invalid limit target diagnostics"
            )
        if not all(isinstance(entry.get("passed"), bool) for entry in targets):
            raise RuntimeContractError(
                "completed record has invalid limit pass diagnostics"
            )
        joint = support_by_name[row["joint_name"]]
        tolerance = max(
            float(protocol["runtime"]["limit_enforcement"]["tolerance"]["absolute_si"]),
            float(
                protocol["runtime"]["limit_enforcement"]["tolerance"][
                    "relative_declared_range"
                ]
            )
            * (float(joint["upper"]) - float(joint["lower"])),
        )
        for entry in targets:
            if entry.get("steps_completed") != int(
                protocol["runtime"]["limit_enforcement"]["steps_each"]
            ):
                raise RuntimeContractError(
                    "completed limit diagnostics have wrong step count"
                )
            expected_limit = bool(
                entry.get("finite") is True
                and _finite_number(entry.get("minimum_q"))
                and _finite_number(entry.get("maximum_q"))
                and entry["minimum_q"] >= float(joint["lower"]) - tolerance
                and entry["maximum_q"] <= float(joint["upper"]) + tolerance
                and entry.get("tolerance") == tolerance
                and entry.get("error") is None
            )
            if entry["passed"] is not expected_limit:
                raise RuntimeContractError("completed limit pass flag is inconsistent")
            limit_pass &= entry["passed"]
    return {
        "load": bool(load["created"]),
        "reset": bool(reset_pass),
        "settling": bool(settling_expected),
        "actuation": bool(actuation_pass),
        "limit_enforcement": bool(limit_pass),
        "constraint_drift": bool(drift_pass),
    }


def _validate_diagnostic_failure_evidence(
    evidence: Any,
    row: dict[str, Any],
    protocol: dict[str, Any],
    simulator: str,
) -> None:
    required = {
        "reason",
        "stage",
        "operation",
        "joint_name",
        "steps_completed",
        "exception_type",
        "message",
        "observed_joint_count",
        "supported_max_joint_count",
        "full_measured_positions_q",
        "trajectory",
    }
    if not isinstance(evidence, dict) or set(evidence) != required:
        raise RuntimeContractError("diagnostic failure evidence has invalid fields")
    for field in ("reason", "stage", "operation", "exception_type", "message"):
        if not isinstance(evidence[field], str) or not evidence[field]:
            raise RuntimeContractError(
                f"diagnostic failure evidence has invalid {field}"
            )
    if len(evidence["message"]) > TAIL_LIMIT:
        raise RuntimeContractError("diagnostic failure evidence message is too long")
    if evidence["stage"] not in {
        "adapter_initialization",
        "reset",
        "settling",
        "actuation",
        "limit_enforcement",
    }:
        raise RuntimeContractError("diagnostic failure evidence has invalid stage")
    joint_name = evidence["joint_name"]
    movable_names = {
        joint["name"]
        for joint in row["joint_tree"]["joints"]
        if joint["type"] != "fixed"
    }
    if joint_name is not None and (
        not isinstance(joint_name, str) or joint_name not in movable_names
    ):
        raise RuntimeContractError("diagnostic failure evidence has invalid joint name")
    steps_completed = evidence["steps_completed"]
    if (
        not isinstance(steps_completed, int)
        or isinstance(steps_completed, bool)
        or steps_completed < 0
    ):
        raise RuntimeContractError("diagnostic failure evidence has invalid step count")
    trace = evidence["full_measured_positions_q"]
    trajectory = evidence["trajectory"]
    if not isinstance(trace, list) or not all(_finite_number(value) for value in trace):
        raise RuntimeContractError(
            "diagnostic failure evidence has invalid finite prefix"
        )
    if not isinstance(trajectory, dict) or set(trajectory) != {
        "sample_steps",
        "normalized_positions",
    }:
        raise RuntimeContractError("diagnostic failure evidence has invalid trajectory")
    sample_steps = trajectory["sample_steps"]
    samples = trajectory["normalized_positions"]
    if (
        not isinstance(sample_steps, list)
        or not all(
            isinstance(step, int) and not isinstance(step, bool)
            for step in sample_steps
        )
        or sample_steps != sorted(set(sample_steps))
        or not isinstance(samples, list)
        or len(samples) != len(sample_steps)
        or not all(_finite_number(value) for value in samples)
    ):
        raise RuntimeContractError(
            "diagnostic failure evidence has invalid sparse trajectory"
        )

    observed_count = evidence["observed_joint_count"]
    supported_count = evidence["supported_max_joint_count"]
    counts_absent = observed_count is None and supported_count is None
    counts_present = all(
        isinstance(value, int) and not isinstance(value, bool) and value >= 0
        for value in (observed_count, supported_count)
    )
    if not counts_absent and not counts_present:
        raise RuntimeContractError(
            "diagnostic failure evidence has invalid capacity counts"
        )
    if evidence["reason"] == "pybullet_actual_state_link_limit":
        if not (
            simulator == "pybullet"
            and counts_present
            and evidence["stage"] == "adapter_initialization"
            and evidence["operation"] == "getJointState"
            and joint_name is None
            and observed_count >= PYBULLET_MAX_ACTUAL_STATE_JOINTS + 1
            and observed_count == row["xml_counts"]["joints"]
            and supported_count == PYBULLET_MAX_ACTUAL_STATE_JOINTS
            and steps_completed == 0
            and trace == []
            and sample_steps == []
            and samples == []
            and evidence["exception_type"] == "RuntimeContractError"
            and evidence["message"]
            == "PyBullet actual-state API does not support multibodies with 128 or more joints"
        ):
            raise RuntimeContractError(
                "diagnostic failure PyBullet capacity evidence is inconsistent"
            )
    elif evidence["reason"] == "simulator_diagnostic_interrupted":
        allowed_operations = {
            "reset": {"reset", "state"},
            "settling": {"reset", "step", "state"},
            "actuation": {"reset", "step", "state"},
            "limit_enforcement": {"reset", "step", "state"},
        }
        if (
            not counts_absent
            or evidence["stage"] not in allowed_operations
            or evidence["operation"] not in allowed_operations[evidence["stage"]]
        ):
            raise RuntimeContractError(
                "diagnostic failure interruption evidence is inconsistent"
            )
    elif evidence["reason"] == "simulator_asset_load_rejected":
        native_load_operations = {
            "pybullet": "loadURDF",
            "mujoco": "MjModel.from_xml_path",
            "genesis": "scene.add_entity_or_build",
        }
        if not (
            simulator in native_load_operations
            and counts_absent
            and evidence["stage"] == "adapter_initialization"
            and evidence["operation"] == native_load_operations[simulator]
            and joint_name is None
            and steps_completed == 0
            and trace == []
            and sample_steps == []
            and samples == []
        ):
            raise RuntimeContractError(
                "diagnostic failure simulator asset-load evidence is inconsistent"
            )
    else:
        raise RuntimeContractError("diagnostic failure evidence has unknown reason")

    if evidence["stage"] == "actuation":
        joint = next(
            (item for item in row["scalar_joints"] if item["name"] == joint_name), None
        )
        if joint is None or steps_completed > 360:
            raise RuntimeContractError(
                "diagnostic failure actuation evidence is inconsistent"
            )
        if trace:
            if len(trace) != steps_completed + 1:
                raise RuntimeContractError(
                    "diagnostic failure actuation prefix length is inconsistent"
                )
            expected_steps = [
                step
                for step in protocol["cross_simulator"]["joint_rmse"]["sample_steps"]
                if step <= steps_completed
            ]
            span = float(joint["upper"]) - float(joint["lower"])
            expected_samples = [
                (float(trace[step]) - float(joint["lower"])) / span
                for step in expected_steps
            ]
            if sample_steps != expected_steps or samples != expected_samples:
                raise RuntimeContractError(
                    "diagnostic failure actuation sparse trace is inconsistent"
                )
        elif steps_completed != 0 or sample_steps or samples:
            raise RuntimeContractError(
                "diagnostic failure empty actuation prefix is inconsistent"
            )
    elif evidence["stage"] == "limit_enforcement":
        if (
            joint_name is None
            or steps_completed
            > int(protocol["runtime"]["limit_enforcement"]["steps_each"])
            or len(trace) != steps_completed
            or sample_steps
            or samples
        ):
            raise RuntimeContractError(
                "diagnostic failure limit evidence is inconsistent"
            )
    elif evidence["stage"] == "settling":
        if (
            steps_completed > int(protocol["runtime"]["passive_settling"]["steps"])
            or trace
            or sample_steps
            or samples
        ):
            raise RuntimeContractError(
                "diagnostic failure settling evidence is inconsistent"
            )
    elif evidence["stage"] in {"adapter_initialization", "reset"}:
        if steps_completed != 0 or trace or sample_steps or samples:
            raise RuntimeContractError(
                "diagnostic failure pre-diagnostic evidence is inconsistent"
            )
    try:
        json.dumps(evidence, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise RuntimeContractError(
            f"diagnostic failure evidence is not finite JSON: {error}"
        ) from error


def _valid_diagnostic_failure_response(
    response: Any,
    row: dict[str, Any],
    protocol: dict[str, Any],
    implementation_receipt: dict[str, Any],
) -> bool:
    try:
        if not isinstance(response, dict) or set(response) != {
            "diagnostic_failure_schema_version",
            "adapter_implementation_receipt",
            "device_receipt",
            "source_binding_receipt",
            "diagnostic_failure",
            "worker_warnings",
        }:
            raise RuntimeContractError("diagnostic failure response has invalid fields")
        if response["diagnostic_failure_schema_version"] != DIAGNOSTIC_FAILURE_SCHEMA:
            raise RuntimeContractError("diagnostic failure response has invalid schema")
        if response["adapter_implementation_receipt"] != implementation_receipt:
            raise RuntimeContractError(
                "diagnostic failure implementation receipt mismatch"
            )
        _validate_source_binding_receipt(
            response["source_binding_receipt"], row, required=True
        )
        _validate_device_receipt(
            response["device_receipt"],
            str(implementation_receipt["simulator"]),
            completed=False,
        )
        warnings = response["worker_warnings"]
        if warnings != _bounded_warnings(warnings):
            raise RuntimeContractError("diagnostic failure warnings are malformed")
        _validate_diagnostic_failure_evidence(
            response["diagnostic_failure"],
            row,
            protocol,
            str(implementation_receipt["simulator"]),
        )
    except (RuntimeContractError, KeyError, TypeError, ValueError):
        return False
    return True


def _terminal_status(
    outcome: ProcessOutcome,
    row: dict[str, Any],
    protocol: dict[str, Any],
    implementation_receipt: dict[str, Any],
) -> tuple[str, str, dict[str, Any] | None]:
    if outcome.preflight_error is not None:
        return "preflight_failure", outcome.preflight_error, None
    if outcome.timeout:
        return "timeout", "child timeout", None
    if outcome.parent_error is not None:
        return "parent_error", outcome.parent_error, None
    if outcome.exit_code not in (None, 0):
        return "native_crash", f"worker exited with code {outcome.exit_code}", None
    if outcome.response is None:
        return "missing_response", "worker produced no response", None
    if "worker_error" in outcome.response:
        return "worker_error", str(outcome.response["worker_error"]), None
    if "close_error" in outcome.response:
        return "worker_error", str(outcome.response["close_error"]), None
    if outcome.response.get("diagnostic_failure_schema_version") is not None:
        if not _valid_diagnostic_failure_response(
            outcome.response, row, protocol, implementation_receipt
        ):
            return (
                "malformed_response",
                "worker diagnostic failure response failed schema",
                None,
            )
        return (
            "diagnostic_failure",
            str(outcome.response["diagnostic_failure"]["message"]),
            outcome.response,
        )
    if not _valid_evaluation(outcome.response, row, protocol, implementation_receipt):
        return "malformed_response", "worker response failed evaluation schema", None
    return "completed", "", outcome.response


def _terminal_record(
    row: dict[str, Any],
    identity: dict[str, Any],
    phase: str,
    outcome: ProcessOutcome,
    protocol: dict[str, Any],
    implementation_receipt: dict[str, Any],
) -> dict[str, Any]:
    status, message, evaluation = _terminal_status(
        outcome, row, protocol, implementation_receipt
    )
    device_receipt = (
        copy.deepcopy(outcome.response.get("device_receipt"))
        if isinstance(outcome.response, dict)
        else None
    )
    source_binding_receipt = None
    if (
        isinstance(outcome.response, dict)
        and outcome.response.get("source_binding_receipt") is not None
    ):
        candidate = outcome.response["source_binding_receipt"]
        try:
            _validate_source_binding_receipt(candidate, row, required=True)
        except RuntimeContractError:
            pass
        else:
            source_binding_receipt = copy.deepcopy(candidate)
    if status == "diagnostic_failure":
        response = evaluation
        evaluation = _failure_evaluation(row, status, message)
        evaluation["diagnostics"]["diagnostic_failure"] = copy.deepcopy(
            response["diagnostic_failure"]
        )
        evaluation["diagnostics"]["warnings"] = _bounded_warnings(
            response.get("worker_warnings")
        )
    elif evaluation is None:
        evaluation = _failure_evaluation(row, status, message)
        evaluation["diagnostics"]["warnings"] = _bounded_warnings(
            outcome.response.get("worker_warnings")
            if isinstance(outcome.response, dict)
            else None
        )
    if status == "preflight_failure":
        evaluation["diagnostics"]["preflight"] = copy.deepcopy(row["preflight"])
    evaluation["diagnostics"]["warnings"] = _bounded_warnings(
        evaluation["diagnostics"].get("warnings", [])
    )
    process = {
        "exit_code": outcome.exit_code,
        "signal": (
            outcome.signal
            if outcome.signal is not None
            else (
                -outcome.exit_code
                if outcome.exit_code is not None and outcome.exit_code < 0
                else None
            )
        ),
        "timeout": bool(outcome.timeout),
        "stdout_tail": _bounded_tail(outcome.stdout),
        "stderr_tail": _bounded_tail(outcome.stderr),
        "retained_warnings": _bounded_warnings(
            evaluation["diagnostics"].get("warnings", [])
        ),
    }
    return {
        "schema_version": ASSET_SCHEMA,
        "run_phase": phase,
        "terminal": True,
        "terminal_status": status,
        "identity": {
            key: value for key, value in identity.items() if key != "run_phase"
        },
        "provenance": {
            "urdf_relative_path": row["urdf_relative_path"],
            "manifest_row_sha256": identity["manifest_row_sha256"],
            "adapter_implementation_receipt": copy.deepcopy(implementation_receipt),
            "device_receipt": device_receipt,
            "parent_gpu_receipt": copy.deepcopy(outcome.parent_gpu_receipt),
            "source_binding_receipt": source_binding_receipt,
        },
        "process": process,
        "source_contract": _source_contract(row),
        "load": evaluation["load"],
        "support": evaluation["support"],
        "metrics": evaluation["metrics"],
        "diagnostics": evaluation["diagnostics"],
    }


TERMINAL_STATUSES = {
    "completed",
    "diagnostic_failure",
    "timeout",
    "native_crash",
    "malformed_response",
    "missing_response",
    "worker_error",
    "parent_error",
    "preflight_failure",
}


def _validate_process_evidence(status: str, process: dict[str, Any]) -> None:
    if set(process) != {
        "exit_code",
        "signal",
        "timeout",
        "stdout_tail",
        "stderr_tail",
        "retained_warnings",
    }:
        raise RuntimeContractError("terminal process evidence has invalid fields")
    exit_code = process.get("exit_code")
    signal_value = process.get("signal")
    timeout = process.get("timeout")
    if not (
        exit_code is None
        or isinstance(exit_code, int)
        and not isinstance(exit_code, bool)
    ):
        raise RuntimeContractError("terminal process evidence has invalid exit code")
    if not (
        signal_value is None
        or isinstance(signal_value, int)
        and not isinstance(signal_value, bool)
        and signal_value > 0
    ):
        raise RuntimeContractError("terminal process evidence has invalid signal")
    if not isinstance(timeout, bool):
        raise RuntimeContractError("terminal process evidence has invalid timeout flag")
    if exit_code is not None and exit_code < 0:
        if signal_value != -exit_code:
            raise RuntimeContractError(
                "terminal process evidence has inconsistent signal"
            )
    elif signal_value is not None:
        raise RuntimeContractError("terminal process evidence has unexpected signal")
    for field in ("stdout_tail", "stderr_tail"):
        value = process.get(field)
        if not isinstance(value, str) or len(value) > TAIL_LIMIT:
            raise RuntimeContractError(
                "terminal process evidence has invalid diagnostic tail"
            )
    warnings = process.get("retained_warnings")
    if not isinstance(warnings, list) or len(warnings) > WARNING_ENTRY_LIMIT:
        raise RuntimeContractError(
            "terminal process evidence has invalid retained warnings"
        )
    for warning in warnings:
        if not isinstance(warning, dict) or set(warning) != {
            "logger",
            "level",
            "message",
        }:
            raise RuntimeContractError(
                "terminal process evidence has invalid retained warning"
            )
        if any(
            not isinstance(warning[field], str)
            or len(warning[field]) > WARNING_MESSAGE_LIMIT
            for field in ("logger", "level", "message")
        ):
            raise RuntimeContractError(
                "terminal process evidence has invalid retained warning"
            )

    clean_exit = exit_code == 0 and signal_value is None and timeout is False
    if status in {"completed", "diagnostic_failure"} and not clean_exit:
        raise RuntimeContractError(
            f"{status} terminal process evidence is inconsistent"
        )
    if status == "timeout" and timeout is not True:
        raise RuntimeContractError("timeout terminal process evidence is inconsistent")
    if status == "native_crash" and not (
        timeout is False and isinstance(exit_code, int) and exit_code != 0
    ):
        raise RuntimeContractError(
            "native-crash terminal process evidence is inconsistent"
        )
    if (
        status in {"malformed_response", "missing_response", "worker_error"}
        and not clean_exit
    ):
        raise RuntimeContractError(
            f"{status} terminal process evidence is inconsistent"
        )
    if status in {"parent_error", "preflight_failure"} and not (
        exit_code is None and signal_value is None and timeout is False
    ):
        raise RuntimeContractError(
            f"{status} terminal process evidence is inconsistent"
        )


def validate_terminal_record(
    record: dict[str, Any],
    expected_identity: dict[str, Any],
    row: dict[str, Any],
    protocol: dict[str, Any],
) -> None:
    if (
        not isinstance(record, dict)
        or record.get("schema_version") != ASSET_SCHEMA
        or record.get("terminal") is not True
    ):
        raise RuntimeContractError("invalid terminal record schema")
    if set(record) != {
        "schema_version",
        "run_phase",
        "terminal",
        "terminal_status",
        "identity",
        "provenance",
        "process",
        "source_contract",
        "load",
        "support",
        "metrics",
        "diagnostics",
    }:
        raise RuntimeContractError("terminal record fields are malformed")
    if record.get("run_phase") != expected_identity["run_phase"]:
        raise RuntimeContractError("resume binding mismatch: run_phase")
    identity = record.get("identity")
    expected_identity_fields = set(expected_identity) - {"run_phase"}
    if not isinstance(identity, dict) or set(identity) != expected_identity_fields:
        raise RuntimeContractError("resume binding mismatch: identity")
    for key, expected in expected_identity.items():
        if key == "run_phase":
            continue
        if identity.get(key) != expected:
            raise RuntimeContractError(f"resume binding mismatch: {key}")
    status = record.get("terminal_status")
    if status not in TERMINAL_STATUSES:
        raise RuntimeContractError(f"invalid terminal status: {status}")
    for section in (
        "provenance",
        "process",
        "source_contract",
        "load",
        "support",
        "metrics",
        "diagnostics",
    ):
        if not isinstance(record.get(section), dict):
            raise RuntimeContractError(f"invalid terminal section: {section}")
    _validate_process_evidence(status, record["process"])
    diagnostic_warnings = record["diagnostics"].get("warnings")
    if (
        not isinstance(diagnostic_warnings, list)
        or diagnostic_warnings != _bounded_warnings(diagnostic_warnings)
        or record["process"]["retained_warnings"] != diagnostic_warnings
    ):
        raise RuntimeContractError(
            "terminal process evidence warnings do not match diagnostics"
        )
    metrics = record["metrics"]
    if set(metrics) != set(METRIC_NAMES) or not all(
        isinstance(metrics.get(name), bool) for name in METRIC_NAMES
    ):
        raise RuntimeContractError("terminal metric booleans are malformed")
    eligible = _preflight_eligible(row)
    if (status == "preflight_failure") != (not eligible):
        raise RuntimeContractError("preflight eligibility/terminal status mismatch")
    if record["source_contract"] != _source_contract(row):
        raise RuntimeContractError(
            "terminal source contract does not match manifest row"
        )
    provenance = record["provenance"]
    if set(provenance) != {
        "urdf_relative_path",
        "manifest_row_sha256",
        "adapter_implementation_receipt",
        "device_receipt",
        "parent_gpu_receipt",
        "source_binding_receipt",
    }:
        raise RuntimeContractError("terminal provenance fields are malformed")
    if (
        provenance.get("urdf_relative_path") != row["urdf_relative_path"]
        or provenance.get("manifest_row_sha256")
        != expected_identity["manifest_row_sha256"]
    ):
        raise RuntimeContractError("terminal provenance does not match manifest row")
    implementation_receipt = provenance.get("adapter_implementation_receipt")
    try:
        _validate_implementation_receipt(
            implementation_receipt,
            simulator=expected_identity["simulator"],
            expected_version=expected_identity["adapter_version"],
        )
    except RuntimeContractError as error:
        raise RuntimeContractError(
            f"terminal implementation receipt is invalid: {error}"
        ) from error
    if (
        canonical_sha256(implementation_receipt)
        != expected_identity["adapter_implementation_sha256"]
    ):
        raise RuntimeContractError(
            "terminal implementation receipt does not match identity"
        )
    device_receipt = provenance.get("device_receipt")
    _validate_device_receipt(
        device_receipt, expected_identity["simulator"], completed=status == "completed"
    )
    parent_gpu_receipt = provenance.get("parent_gpu_receipt")
    if expected_identity["simulator"] == "genesis":
        if parent_gpu_receipt is None:
            if status != "preflight_failure":
                raise RuntimeContractError(
                    "Genesis terminal lacks parent GPU gate receipt"
                )
        else:
            _validate_parent_gpu_receipt(parent_gpu_receipt)
            if (
                canonical_sha256(_gpu_hardware_identity(parent_gpu_receipt))
                != expected_identity["parent_gpu_hardware_sha256"]
            ):
                raise RuntimeContractError(
                    "parent GPU hardware receipt does not match identity"
                )
    elif parent_gpu_receipt is not None:
        raise RuntimeContractError(
            "CPU simulator terminal contains a parent GPU receipt"
        )
    source_binding_receipt = provenance.get("source_binding_receipt")
    _validate_source_binding_receipt(
        source_binding_receipt,
        row,
        required=status in {"completed", "diagnostic_failure"},
    )
    if status == "completed":
        evaluation = {
            "evaluation_schema_version": EVALUATION_SCHEMA,
            "adapter_implementation_receipt": implementation_receipt,
            "device_receipt": device_receipt,
            "source_binding_receipt": source_binding_receipt,
            "load": record["load"],
            "support": record["support"],
            "metrics": record["metrics"],
            "diagnostics": record["diagnostics"],
        }
        try:
            _validate_evaluation_or_raise(
                evaluation, row, protocol, implementation_receipt
            )
        except (KeyError, AttributeError, IndexError, TypeError, ValueError) as error:
            if isinstance(error, RuntimeContractError):
                raise
            raise RuntimeContractError(
                f"completed evaluation structure is malformed: {error}"
            ) from error
    else:
        expected_failure = _failure_evaluation(
            row, status, record["diagnostics"].get("errors", [""])[0].partition(": ")[2]
        )
        if any(metrics[name] is not False for name in METRIC_NAMES):
            raise RuntimeContractError("failure metrics must all be false")
        for section in ("load", "support"):
            if record[section] != expected_failure[section]:
                raise RuntimeContractError(
                    f"failure {section} does not match the closed template"
                )
        diagnostics = record["diagnostics"]
        if (
            diagnostics.get("reset") != []
            or diagnostics.get("settling") != {}
            or diagnostics.get("actuation") != []
            or diagnostics.get("limit_enforcement") != []
            or not isinstance(diagnostics.get("warnings"), list)
            or not isinstance(diagnostics.get("errors"), list)
            or len(diagnostics["errors"]) != 1
        ):
            raise RuntimeContractError(
                "failure diagnostics do not match the closed template"
            )
        diagnostic_failure = diagnostics.get("diagnostic_failure")
        if status == "diagnostic_failure":
            _validate_diagnostic_failure_evidence(
                diagnostic_failure,
                row,
                protocol,
                expected_identity["simulator"],
            )
            expected_error = f"diagnostic_failure: {diagnostic_failure['message']}"
            if diagnostics["errors"] != [expected_error]:
                raise RuntimeContractError(
                    "diagnostic failure error message is inconsistent"
                )
        elif diagnostic_failure is not None:
            raise RuntimeContractError(
                "non-diagnostic failure contains diagnostic evidence"
            )
        if status == "preflight_failure":
            if diagnostics.get("preflight") != row["preflight"]:
                raise RuntimeContractError(
                    "preflight failure evidence does not match manifest row"
                )
            expected_error = f"preflight_failure: {_preflight_message(row)}"
            if diagnostics["errors"] != [expected_error]:
                raise RuntimeContractError(
                    "preflight failure error message is inconsistent"
                )
        elif diagnostics.get("preflight") is not None:
            raise RuntimeContractError(
                "non-preflight failure contains preflight evidence"
            )
    try:
        json.dumps(record, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise RuntimeContractError(
            f"terminal record is not finite JSON: {error}"
        ) from error


def summarize_records(
    records: list[dict[str, Any]],
    *,
    intent_count: int,
    simulator: str,
    phase: str,
    workers: int = 1,
) -> dict[str, Any]:
    if (
        intent_count < len(records)
        or not isinstance(workers, int)
        or isinstance(workers, bool)
        or workers < 1
    ):
        raise RuntimeContractError("terminal count exceeds intent count")
    status_counts: dict[str, int] = {}
    metric_counts = {name: 0 for name in METRIC_NAMES}
    for record in records:
        status = record["terminal_status"]
        status_counts[status] = status_counts.get(status, 0) + 1
        for metric in METRIC_NAMES:
            metric_counts[metric] += int(record["metrics"][metric])
    terminal_count = len(records)
    return {
        "schema_version": SUMMARY_SCHEMA,
        "run_phase": phase,
        "simulator": simulator,
        "effective_workers": workers,
        "intent_count": intent_count,
        "terminal_count": terminal_count,
        "remaining_count": intent_count - terminal_count,
        "complete": terminal_count == intent_count,
        "terminal_status_counts": status_counts,
        "metric_pass_counts": metric_counts,
        "metric_denominator": intent_count,
    }


def _publish_progress(
    output_root: Path,
    records: list[dict[str, Any]],
    *,
    intent_count: int,
    simulator: str,
    phase: str,
    workers: int,
) -> None:
    summary = summarize_records(
        records,
        intent_count=intent_count,
        simulator=simulator,
        phase=phase,
        workers=workers,
    )
    progress = copy.deepcopy(summary)
    progress["schema_version"] = PROGRESS_SCHEMA
    atomic_write_json(output_root / "progress.json", progress)
    atomic_write_json(output_root / "summary.json", summary)


SpawnWorker = Callable[[dict[str, Any], dict[str, Any]], ProcessOutcome]


def run_intent(
    *,
    rows: list[dict[str, Any]],
    manifest: dict[str, Any],
    protocol: dict[str, Any],
    simulator: str,
    phase: str,
    output_root: Path,
    spawn_worker: SpawnWorker,
    workers: int = 1,
    timeout_s: float | None = None,
    implementation_receipt: dict[str, Any],
) -> dict[str, Any]:
    if not isinstance(workers, int) or isinstance(workers, bool) or workers < 1:
        raise RuntimeContractError("workers must be positive")
    if simulator == "genesis" and workers != 1:
        raise RuntimeContractError("Genesis requires workers=1")
    validate_runtime_protocol(protocol)
    if phase not in {"qualification", "formal"}:
        raise RuntimeContractError("invalid run phase")
    output_root = Path(output_root)
    identities = {
        row["dataset_id"]: adapter_identity(
            protocol,
            manifest,
            row,
            simulator,
            phase,
            timeout_s,
            implementation_receipt=implementation_receipt,
            workers=workers,
        )
        for row in rows
    }
    row_by_id = {row["dataset_id"]: row for row in rows}
    if len(row_by_id) != len(rows):
        raise RuntimeContractError("intent contains duplicate dataset IDs")
    with output_lock(output_root):
        assets_root = output_root / "assets"
        assets_root.mkdir(parents=True, exist_ok=True)
        records: dict[str, dict[str, Any]] = {}
        for path in sorted(assets_root.glob("*.json")):
            if not _valid_dataset_id(path.stem) or path.stem not in identities:
                raise RuntimeContractError(
                    f"unexpected terminal record in output root: {path.name}"
                )
            dataset_id = path.stem
            try:
                record = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as error:
                raise RuntimeContractError(
                    f"invalid existing terminal record {path}: {error}"
                ) from error
            validate_terminal_record(
                record, identities[dataset_id], row_by_id[dataset_id], protocol
            )
            records[dataset_id] = record
        _publish_progress(
            output_root,
            list(records.values()),
            intent_count=len(rows),
            simulator=simulator,
            phase=phase,
            workers=workers,
        )
        pending = [row for row in rows if row["dataset_id"] not in records]

        def invoke(row: dict[str, Any]) -> tuple[dict[str, Any], ProcessOutcome]:
            if not _preflight_eligible(row):
                return row, ProcessOutcome(preflight_error=_preflight_message(row))
            try:
                return row, spawn_worker(row, identities[row["dataset_id"]])
            except GpuGateError:
                raise
            except Exception as error:
                return row, ProcessOutcome(
                    parent_error=f"{type(error).__name__}: {error}"
                )

        def retain(row: dict[str, Any], outcome: ProcessOutcome) -> None:
            dataset_id = row["dataset_id"]
            path = assets_root / f"{dataset_id}.json"
            if path.exists():
                raise RuntimeContractError(
                    f"refusing to overwrite terminal record: {path}"
                )
            record = _terminal_record(
                row,
                identities[dataset_id],
                phase,
                outcome,
                protocol,
                implementation_receipt,
            )
            validate_terminal_record(record, identities[dataset_id], row, protocol)
            atomic_write_json(path, record)
            records[dataset_id] = record
            _publish_progress(
                output_root,
                list(records.values()),
                intent_count=len(rows),
                simulator=simulator,
                phase=phase,
                workers=workers,
            )

        if workers == 1:
            for row in pending:
                current, outcome = invoke(row)
                retain(current, outcome)
        elif pending:
            executor = ThreadPoolExecutor(
                max_workers=workers, thread_name_prefix=f"table5-{simulator}"
            )
            futures = [executor.submit(invoke, row) for row in pending]
            try:
                for future in as_completed(futures):
                    current, outcome = future.result()
                    retain(current, outcome)
            except BaseException:
                for future in futures:
                    future.cancel()
                executor.shutdown(wait=False, cancel_futures=True)
                raise
            else:
                executor.shutdown(wait=True)
        return summarize_records(
            list(records.values()),
            intent_count=len(rows),
            simulator=simulator,
            phase=phase,
            workers=workers,
        )


def _distribution_version(distributions: list[str], expected: str) -> None:
    actual: str | None = None
    for distribution in distributions:
        try:
            actual = importlib.metadata.version(distribution)
            break
        except importlib.metadata.PackageNotFoundError:
            continue
    if actual != expected:
        raise RuntimeContractError(
            f"adapter version mismatch: expected {expected}, found {actual}"
        )


def _pybullet_load_flags(p: Any, protocol: dict[str, Any]) -> int:
    validate_runtime_protocol(protocol)
    flags = (
        p.URDF_USE_INERTIA_FROM_FILE
        | p.URDF_IGNORE_VISUAL_SHAPES
        | p.URDF_USE_SELF_COLLISION
    )
    if protocol["adapters"]["pybullet"]["maintain_link_order"] is True:
        flags |= p.URDF_MAINTAIN_LINK_ORDER
    return flags


def _require_pybullet_actual_state_capacity(joint_count: int) -> None:
    if joint_count >= PYBULLET_MAX_ACTUAL_STATE_JOINTS + 1:
        raise _diagnostic_failure(
            reason="pybullet_actual_state_link_limit",
            stage="adapter_initialization",
            operation="getJointState",
            message=(
                "PyBullet actual-state API does not support multibodies with "
                f"{PYBULLET_MAX_ACTUAL_STATE_JOINTS + 1} or more joints"
            ),
            observed_joint_count=joint_count,
            supported_max_joint_count=PYBULLET_MAX_ACTUAL_STATE_JOINTS,
        )


class PyBulletAdapter:
    def __init__(
        self, raw_urdf_path: Path, row: dict[str, Any], protocol: dict[str, Any]
    ):
        import pybullet as p

        _distribution_version(["pybullet"], protocol["adapters"]["pybullet"]["version"])
        self.p = p
        self.client = p.connect(p.DIRECT)
        if self.client < 0:
            raise RuntimeContractError("PyBullet DIRECT connection failed")
        timestep = protocol["runtime"]["timestep_s"]
        p.setGravity(
            *protocol["runtime"]["gravity_m_per_s2"], physicsClientId=self.client
        )
        p.setTimeStep(
            timestep["numerator"] / timestep["denominator"], physicsClientId=self.client
        )
        p.setPhysicsEngineParameter(
            numSolverIterations=protocol["runtime"]["solver_iterations"],
            physicsClientId=self.client,
        )
        flags = _pybullet_load_flags(p, protocol)
        try:
            self.body = p.loadURDF(
                str(raw_urdf_path),
                useFixedBase=True,
                flags=flags,
                physicsClientId=self.client,
            )
        except Exception as error:
            try:
                p.disconnect(physicsClientId=self.client)
            except Exception:
                pass
            self.client = -1
            raise _diagnostic_failure(
                reason="simulator_asset_load_rejected",
                stage="adapter_initialization",
                operation="loadURDF",
                error=error,
            ) from error
        joint_count = p.getNumJoints(self.body, physicsClientId=self.client)
        try:
            _require_pybullet_actual_state_capacity(joint_count)
        except DiagnosticFailure:
            p.disconnect(physicsClientId=self.client)
            self.client = -1
            raise
        body_info = p.getBodyInfo(self.body, physicsClientId=self.client)
        base_name = body_info[0].decode("utf-8", errors="strict")
        self.root_name = row["joint_tree"]["root_links"][0]
        self.observed_link_names = [base_name]
        self.observed_joint_names = []
        self.joint_indices: dict[str, int] = {}
        for index in range(joint_count):
            info = p.getJointInfo(self.body, index, physicsClientId=self.client)
            name = info[1].decode("utf-8", errors="strict")
            child_name = info[12].decode("utf-8", errors="strict")
            self.observed_joint_names.append(name)
            self.observed_link_names.append(child_name)
            if info[2] in {p.JOINT_REVOLUTE, p.JOINT_PRISMATIC}:
                self.joint_indices[name] = index
                p.setJointMotorControl2(
                    self.body,
                    index,
                    p.VELOCITY_CONTROL,
                    force=0,
                    physicsClientId=self.client,
                )
        expected_scalar = {joint["name"] for joint in row["scalar_joints"]}
        self.mapped_joint_names = sorted(expected_scalar & set(self.joint_indices))
        self.warnings: list[str] = []

    def reset(self, positions: dict[str, float]) -> None:
        for name, index in self.joint_indices.items():
            self.p.resetJointState(
                self.body,
                index,
                float(positions.get(name, 0.0)),
                targetVelocity=0.0,
                physicsClientId=self.client,
            )
        self._apply({})

    def state(self) -> dict[str, dict[str, float]]:
        return {
            name: {
                "q": float(
                    self.p.getJointState(
                        self.body, self.joint_indices[name], physicsClientId=self.client
                    )[0]
                ),
                "qdot": float(
                    self.p.getJointState(
                        self.body, self.joint_indices[name], physicsClientId=self.client
                    )[1]
                ),
            }
            for name in self.mapped_joint_names
        }

    def _apply(self, efforts: dict[str, float]) -> None:
        for name, index in self.joint_indices.items():
            self.p.setJointMotorControl2(
                self.body,
                index,
                self.p.TORQUE_CONTROL,
                force=float(efforts.get(name, 0.0)),
                physicsClientId=self.client,
            )

    def step(self, efforts: dict[str, float]) -> None:
        self._apply(efforts)
        self.p.stepSimulation(physicsClientId=self.client)

    def link_poses(self) -> dict[str, dict[str, list[float]]]:
        base_position, base_xyzw = self.p.getBasePositionAndOrientation(
            self.body, physicsClientId=self.client
        )
        dynamics = self.p.getDynamicsInfo(self.body, -1, physicsClientId=self.client)
        local_inertial_position, local_inertial_xyzw = dynamics[3], dynamics[4]
        inverse_position, inverse_xyzw = self.p.invertTransform(
            local_inertial_position, local_inertial_xyzw
        )
        root_position, root_xyzw = self.p.multiplyTransforms(
            base_position, base_xyzw, inverse_position, inverse_xyzw
        )
        world = {
            self.observed_link_names[0]: {
                "translation": list(root_position),
                "rotation": [root_xyzw[3], root_xyzw[0], root_xyzw[1], root_xyzw[2]],
            }
        }
        for name, index in self.joint_indices.items():
            del name
            state = self.p.getLinkState(
                self.body,
                index,
                computeForwardKinematics=True,
                physicsClientId=self.client,
            )
            child_name = self.p.getJointInfo(
                self.body, index, physicsClientId=self.client
            )[12].decode("utf-8")
            xyzw = state[5]
            world[child_name] = {
                "translation": list(state[4]),
                "rotation": [xyzw[3], xyzw[0], xyzw[1], xyzw[2]],
            }
        # Fixed-joint child frames are required too, not only scalar joint frames.
        for index in range(self.p.getNumJoints(self.body, physicsClientId=self.client)):
            info = self.p.getJointInfo(self.body, index, physicsClientId=self.client)
            child_name = info[12].decode("utf-8")
            if child_name in world:
                continue
            state = self.p.getLinkState(
                self.body,
                index,
                computeForwardKinematics=True,
                physicsClientId=self.client,
            )
            xyzw = state[5]
            world[child_name] = {
                "translation": list(state[4]),
                "rotation": [xyzw[3], xyzw[0], xyzw[1], xyzw[2]],
            }
        finite_world: dict[str, dict[str, list[float]]] = {}
        for name, pose in world.items():
            try:
                _vector(pose.get("translation"), 3)
                _quat_normalized(pose.get("rotation"))
            except RuntimeContractError:
                continue
            finite_world[name] = pose
        return relative_link_poses(finite_world, self.root_name)

    def close(self) -> None:
        if self.client >= 0:
            self.p.disconnect(physicsClientId=self.client)
            self.client = -1


class MuJoCoAdapter:
    def __init__(
        self, raw_urdf_path: Path, row: dict[str, Any], protocol: dict[str, Any]
    ):
        import mujoco

        _distribution_version(["mujoco"], protocol["adapters"]["mujoco"]["version"])
        self.mujoco = mujoco
        try:
            self.model = mujoco.MjModel.from_xml_path(str(raw_urdf_path))
        except Exception as error:
            raise _diagnostic_failure(
                reason="simulator_asset_load_rejected",
                stage="adapter_initialization",
                operation="MjModel.from_xml_path",
                error=error,
            ) from error
        self.data = mujoco.MjData(self.model)
        timestep = protocol["runtime"]["timestep_s"]
        self.model.opt.timestep = timestep["numerator"] / timestep["denominator"]
        self.model.opt.gravity[:] = protocol["runtime"]["gravity_m_per_s2"]
        self.model.opt.iterations = protocol["runtime"]["solver_iterations"]
        self.root_name = row["joint_tree"]["root_links"][0]
        self.body_ids: dict[str, int] = {}
        for index in range(1, self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, index)
            if name:
                self.body_ids[name] = index
        self.joint_ids: dict[str, int] = {}
        for index in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, index)
            if name:
                self.joint_ids[name] = index
        self.observed_link_names, _aligned = _mujoco_observed_and_aligned_poses(
            {
                name: {"translation": [0.0, 0.0, 0.0], "rotation": [1.0, 0.0, 0.0, 0.0]}
                for name in self.body_ids
            },
            self.root_name,
        )
        self.observed_joint_names = sorted(self.joint_ids)
        scalar_by_name = {joint["name"]: joint for joint in row["scalar_joints"]}
        self.mapping: dict[str, tuple[int, int]] = {}
        for name in sorted(set(scalar_by_name) & set(self.joint_ids)):
            joint_id = self.joint_ids[name]
            kind = int(self.model.jnt_type[joint_id])
            if kind in {
                int(mujoco.mjtJoint.mjJNT_HINGE),
                int(mujoco.mjtJoint.mjJNT_SLIDE),
            }:
                self.mapping[name] = (
                    int(self.model.jnt_qposadr[joint_id]),
                    int(self.model.jnt_dofadr[joint_id]),
                )
        self.mapped_joint_names = sorted(self.mapping)
        self.warnings: list[str] = []

    def reset(self, positions: dict[str, float]) -> None:
        self.mujoco.mj_resetData(self.model, self.data)
        for name, (qpos_address, dof_address) in self.mapping.items():
            self.data.qpos[qpos_address] = float(positions.get(name, 0.0))
            self.data.qvel[dof_address] = 0.0
        self.data.qfrc_applied[:] = 0.0
        self.mujoco.mj_forward(self.model, self.data)

    def state(self) -> dict[str, dict[str, float]]:
        return {
            name: {"q": float(self.data.qpos[qpos]), "qdot": float(self.data.qvel[dof])}
            for name, (qpos, dof) in self.mapping.items()
        }

    def step(self, efforts: dict[str, float]) -> None:
        self.data.qfrc_applied[:] = 0.0
        for name, (_qpos, dof) in self.mapping.items():
            self.data.qfrc_applied[dof] = float(efforts.get(name, 0.0))
        self.mujoco.mj_step(self.model, self.data)

    def link_poses(self) -> dict[str, dict[str, list[float]]]:
        world = {
            name: {
                "translation": [float(value) for value in self.data.xpos[index]],
                "rotation": [float(value) for value in self.data.xquat[index]],
            }
            for name, index in self.body_ids.items()
        }
        _observed, aligned = _mujoco_observed_and_aligned_poses(world, self.root_name)
        return aligned

    def close(self) -> None:
        pass


def _mujoco_observed_and_aligned_poses(
    body_world: dict[str, dict[str, list[float]]], root_name: str
) -> tuple[list[str], dict[str, dict[str, list[float]]]]:
    """Use folded world as root alignment without claiming it as an observed URDF body."""
    observed = sorted(body_world)
    world = dict(body_world)
    world.setdefault(
        root_name,
        {"translation": [0.0, 0.0, 0.0], "rotation": [1.0, 0.0, 0.0, 0.0]},
    )
    return observed, relative_link_poses(world, root_name)


class GenesisAdapter:
    def __init__(
        self,
        raw_urdf_path: Path,
        row: dict[str, Any],
        protocol: dict[str, Any],
        parent_gpu_receipt: dict[str, Any],
    ):
        import genesis as gs
        import torch

        validate_runtime_protocol(protocol)
        _distribution_version(
            ["genesis-world", "genesis"], protocol["adapters"]["genesis"]["version"]
        )
        self.gs = gs
        gpu_snapshot = _assert_unique_genesis_gpu(torch, protocol, parent_gpu_receipt)
        gs.init(
            backend=gs.cuda,
            precision="32",
            seed=protocol["runtime"]["random_seed"],
            logging_level=logging.WARNING,
        )
        self.device_receipt = _genesis_device_receipt(gs, gpu_snapshot)
        timestep = protocol["runtime"]["timestep_s"]
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(
                dt=timestep["numerator"] / timestep["denominator"],
                substeps=1,
                gravity=tuple(protocol["runtime"]["gravity_m_per_s2"]),
            ),
            rigid_options=gs.options.RigidOptions(
                enable_collision=True,
                enable_self_collision=True,
                enable_neutral_collision=True,
                iterations=protocol["runtime"]["solver_iterations"],
            ),
            show_viewer=False,
        )
        morph = gs.morphs.URDF(
            file=str(raw_urdf_path),
            fixed=True,
            visualization=False,
            collision=True,
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
            raise _diagnostic_failure(
                reason="simulator_asset_load_rejected",
                stage="adapter_initialization",
                operation="scene.add_entity_or_build",
                error=error,
            ) from error
        self.root_name = row["joint_tree"]["root_links"][0]
        self.links = {link.name: link for link in self.entity.links}
        expected_joints = {joint["name"] for joint in row["joint_tree"]["joints"]}
        runtime_joints = {}
        synthetic_root_names = set()
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

    @staticmethod
    def _values(tensor: Any) -> list[float]:
        if hasattr(tensor, "detach"):
            tensor = tensor.detach().cpu()
        if hasattr(tensor, "tolist"):
            value = tensor.tolist()
        else:
            value = list(tensor)
        return [float(item) for item in value]

    def reset(self, positions: dict[str, float]) -> None:
        self.scene.reset()
        if self.mapping:
            names = sorted(self.mapping)
            indices = [self.mapping[name] for name in names]
            self.entity.set_dofs_position(
                [float(positions.get(name, 0.0)) for name in names],
                indices,
                zero_velocity=True,
            )

    def state(self) -> dict[str, dict[str, float]]:
        if not self.mapping:
            return {}
        names = sorted(self.mapping)
        indices = [self.mapping[name] for name in names]
        positions = self._values(self.entity.get_dofs_position(indices))
        velocities = self._values(self.entity.get_dofs_velocity(indices))
        return {
            name: {"q": positions[index], "qdot": velocities[index]}
            for index, name in enumerate(names)
        }

    def step(self, efforts: dict[str, float]) -> None:
        if self.mapping:
            names = sorted(self.mapping)
            indices = [self.mapping[name] for name in names]
            self.entity.control_dofs_force(
                [float(efforts.get(name, 0.0)) for name in names], indices
            )
        self.scene.step(update_visualizer=False, refresh_visualizer=False)

    def link_poses(self) -> dict[str, dict[str, list[float]]]:
        world = {
            name: {
                "translation": self._values(link.get_pos(relative=False)),
                "rotation": self._values(link.get_quat(relative=False)),
            }
            for name, link in self.links.items()
        }
        return relative_link_poses(world, self.root_name)

    def close(self) -> None:
        try:
            self.scene.destroy()
        finally:
            self.gs.destroy()


def _make_adapter(
    simulator: str,
    raw_urdf_path: Path,
    row: dict[str, Any],
    protocol: dict[str, Any],
    parent_gpu_receipt: dict[str, Any] | None = None,
) -> Adapter:
    if simulator == "pybullet":
        return PyBulletAdapter(raw_urdf_path, row, protocol)
    if simulator == "mujoco":
        return MuJoCoAdapter(raw_urdf_path, row, protocol)
    if simulator == "genesis":
        if parent_gpu_receipt is None:
            raise RuntimeContractError(
                "Genesis worker is missing parent GPU gate receipt"
            )
        return GenesisAdapter(raw_urdf_path, row, protocol, parent_gpu_receipt)
    raise RuntimeContractError(f"unknown simulator: {simulator}")


def _normalized_gpu_uuid(value: Any) -> str:
    if not isinstance(value, str) or not value:
        raise RuntimeContractError("GPU UUID is missing")
    normalized = value.lower()
    if normalized.startswith("gpu-"):
        normalized = normalized[4:]
    normalized = normalized.replace("-", "")
    if len(normalized) != 32 or any(
        character not in "0123456789abcdef" for character in normalized
    ):
        raise RuntimeContractError("GPU UUID is malformed")
    return normalized


def _assert_unique_genesis_gpu(
    torch_module: Any,
    protocol: dict[str, Any],
    parent_gpu_receipt: dict[str, Any],
    *,
    quadrants_version: str | None = None,
    python_version: str | None = None,
) -> dict[str, Any]:
    validate_runtime_protocol(protocol)
    _validate_parent_gpu_receipt(parent_gpu_receipt)
    binding = protocol["adapters"]["genesis"]["gpu_binding"]
    if os.environ.get("CUDA_VISIBLE_DEVICES") != binding["cuda_visible_devices"]:
        raise RuntimeContractError(
            "CUDA_VISIBLE_DEVICES does not match frozen Genesis binding"
        )
    if os.environ.get("CUDA_DEVICE_ORDER") != "PCI_BUS_ID":
        raise RuntimeContractError("CUDA_DEVICE_ORDER must be PCI_BUS_ID")
    cuda = torch_module.cuda
    if cuda.is_available() is not True or cuda.device_count() != 1:
        raise RuntimeContractError("Genesis worker must see exactly one CUDA device")
    if cuda.current_device() != binding["visible_device_index"]:
        raise RuntimeContractError(
            "Genesis worker current CUDA device is not logical device 0"
        )
    properties = cuda.get_device_properties(binding["visible_device_index"])
    torch_uuid = str(getattr(properties, "uuid", ""))
    if _normalized_gpu_uuid(torch_uuid) != _normalized_gpu_uuid(binding["gpu_uuid"]):
        raise RuntimeContractError(
            "Genesis worker logical GPU UUID does not match frozen binding"
        )
    total_memory = getattr(properties, "total_memory", None)
    if (
        not isinstance(total_memory, int)
        or isinstance(total_memory, bool)
        or total_memory <= 0
    ):
        raise RuntimeContractError("Genesis worker GPU total memory is unavailable")
    cudnn_version = torch_module.backends.cudnn.version()
    if not isinstance(cudnn_version, int) or isinstance(cudnn_version, bool):
        raise RuntimeContractError("Genesis worker cuDNN version is unavailable")
    torch_compute_capability = (
        f"{getattr(properties, 'major', '')}.{getattr(properties, 'minor', '')}"
    )
    if quadrants_version is None:
        try:
            quadrants_version = importlib.metadata.version("quadrants")
        except importlib.metadata.PackageNotFoundError as error:
            raise RuntimeContractError(
                "Quadrants distribution is unavailable"
            ) from error
    software = {
        "torch_version": str(torch_module.__version__),
        "torch_cuda_version": str(torch_module.version.cuda),
        "cudnn_version": cudnn_version,
        "quadrants_version": quadrants_version,
        "python_version": python_version
        or ".".join(str(value) for value in sys.version_info[:3]),
    }
    observed_torch_device = {
        "torch_device_name": str(getattr(properties, "name", "")),
        "torch_total_memory_bytes": total_memory,
        "torch_total_memory_mib": total_memory // (1024 * 1024),
        "torch_compute_capability": torch_compute_capability,
    }
    if (
        observed_torch_device != FROZEN_GENESIS_TORCH_DEVICE
        or software != FROZEN_GENESIS_SOFTWARE
    ):
        raise RuntimeContractError("Genesis worker hardware/software receipt mismatch")
    child_gpu_gate_receipt = _query_child_gpu_receipt(protocol)
    return {
        "schema_version": GENESIS_DEVICE_RECEIPT_SCHEMA,
        "backend": "cuda",
        "logical_device": "cuda:0",
        "logical_device_count": 1,
        "cuda_visible_devices": binding["cuda_visible_devices"],
        "cuda_device_order": "PCI_BUS_ID",
        "physical_device_index": binding["physical_device_index"],
        "visible_device_index": binding["visible_device_index"],
        "nvidia_smi_gpu_uuid": binding["gpu_uuid"],
        "torch_gpu_uuid": torch_uuid,
        "normalized_gpu_uuid": _normalized_gpu_uuid(torch_uuid),
        "nvidia_smi_device_name": child_gpu_gate_receipt["device_name"],
        "nvidia_smi_total_memory_mib": child_gpu_gate_receipt["total_memory_mib"],
        "driver_version": child_gpu_gate_receipt["driver_version"],
        "nvidia_smi_compute_capability": child_gpu_gate_receipt["compute_capability"],
        **observed_torch_device,
        **software,
        "child_gpu_gate_receipt": child_gpu_gate_receipt,
    }


def _genesis_device_receipt(gs: Any, snapshot: dict[str, Any]) -> dict[str, Any]:
    if getattr(gs, "backend", None) != gs.cuda:
        raise RuntimeContractError(
            "Genesis backend is not gs.cuda; fallback is forbidden"
        )
    if str(getattr(gs, "device", "")) != "cuda:0":
        raise RuntimeContractError(
            "Genesis device is not cuda:0; fallback is forbidden"
        )
    receipt = copy.deepcopy(snapshot)
    _validate_device_receipt(receipt, "genesis", completed=True)
    return receipt


def _validate_device_receipt(receipt: Any, simulator: str, *, completed: bool) -> None:
    if simulator != "genesis":
        if receipt is not None:
            raise RuntimeContractError(
                "CPU simulator must not claim a Genesis GPU receipt"
            )
        return
    if receipt is None and not completed:
        return
    fields = {
        "schema_version",
        "backend",
        "logical_device",
        "logical_device_count",
        "cuda_visible_devices",
        "cuda_device_order",
        "physical_device_index",
        "visible_device_index",
        "nvidia_smi_gpu_uuid",
        "torch_gpu_uuid",
        "normalized_gpu_uuid",
        "nvidia_smi_device_name",
        "nvidia_smi_total_memory_mib",
        "driver_version",
        "nvidia_smi_compute_capability",
        *FROZEN_GENESIS_TORCH_DEVICE,
        "torch_version",
        "torch_cuda_version",
        "cudnn_version",
        "quadrants_version",
        "python_version",
        "child_gpu_gate_receipt",
    }
    if not isinstance(receipt, dict) or set(receipt) != fields:
        raise RuntimeContractError("Genesis device receipt fields are malformed")
    _validate_child_gpu_receipt(receipt["child_gpu_gate_receipt"])
    if (
        receipt["schema_version"] != GENESIS_DEVICE_RECEIPT_SCHEMA
        or receipt["backend"] != "cuda"
        or receipt["logical_device"] != "cuda:0"
        or receipt["logical_device_count"] != 1
        or receipt["cuda_visible_devices"]
        != GENESIS_GPU_BINDING["cuda_visible_devices"]
        or receipt["cuda_device_order"] != "PCI_BUS_ID"
        or receipt["physical_device_index"]
        != GENESIS_GPU_BINDING["physical_device_index"]
        or receipt["visible_device_index"]
        != GENESIS_GPU_BINDING["visible_device_index"]
        or receipt["nvidia_smi_gpu_uuid"] != GENESIS_GPU_BINDING["gpu_uuid"]
        or receipt["normalized_gpu_uuid"]
        != _normalized_gpu_uuid(receipt["torch_gpu_uuid"])
        or receipt["normalized_gpu_uuid"]
        != _normalized_gpu_uuid(GENESIS_GPU_BINDING["gpu_uuid"])
        or receipt["nvidia_smi_device_name"]
        != FROZEN_GENESIS_GPU_HARDWARE["device_name"]
        or receipt["nvidia_smi_total_memory_mib"]
        != FROZEN_GENESIS_GPU_HARDWARE["total_memory_mib"]
        or receipt["nvidia_smi_compute_capability"]
        != FROZEN_GENESIS_GPU_HARDWARE["compute_capability"]
        or {key: receipt[key] for key in FROZEN_GENESIS_TORCH_DEVICE}
        != FROZEN_GENESIS_TORCH_DEVICE
        or {key: receipt[key] for key in FROZEN_GENESIS_SOFTWARE}
        != FROZEN_GENESIS_SOFTWARE
        or receipt["driver_version"] != FROZEN_GENESIS_GPU_HARDWARE["driver_version"]
    ):
        raise RuntimeContractError(
            "Genesis device receipt does not match frozen GPU binding"
        )


def _worker_main(request_path: Path, response_path: Path) -> int:
    adapter: Adapter | None = None
    device_receipt: dict[str, Any] | None = None
    source_binding_receipt: dict[str, Any] | None = None
    captured_warnings: list[dict[str, str]] = []

    class WarningCapture(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            if (
                record.levelno < logging.WARNING
                or len(captured_warnings) >= WARNING_ENTRY_LIMIT
            ):
                return
            try:
                message = record.getMessage()
            except Exception:
                message = "<unformattable warning>"
            captured_warnings.append(
                {
                    "logger": str(record.name)[:WARNING_MESSAGE_LIMIT],
                    "level": str(record.levelname)[:WARNING_MESSAGE_LIMIT],
                    "message": message[:WARNING_MESSAGE_LIMIT],
                }
            )

    warning_handler = WarningCapture()
    root_logger = logging.getLogger()
    root_logger.addHandler(warning_handler)
    try:
        request = json.loads(request_path.read_text(encoding="utf-8"))
        implementation_receipt = adapter_implementation_receipt(request["simulator"])
        if implementation_receipt != request.get("implementation_receipt"):
            raise RuntimeContractError(
                "parent/worker adapter implementation receipt mismatch"
            )
        parent_gpu_receipt = request.get("parent_gpu_receipt")
        if request["simulator"] == "genesis":
            _validate_parent_gpu_receipt(parent_gpu_receipt)
        elif parent_gpu_receipt is not None:
            raise RuntimeContractError(
                "CPU worker received a Genesis parent GPU receipt"
            )
        source_binding_receipt = _validate_worker_source_binding(request)
        adapter = _make_adapter(
            request["simulator"],
            Path(request["raw_urdf_path"]),
            request["row"],
            request["protocol"],
            parent_gpu_receipt,
        )
        device_receipt = copy.deepcopy(getattr(adapter, "device_receipt", None))
        response = evaluate_asset(adapter, request["row"], request["protocol"])
        response["adapter_implementation_receipt"] = implementation_receipt
        response["device_receipt"] = device_receipt
        response["source_binding_receipt"] = source_binding_receipt
    except DiagnosticFailure as error:
        response = {
            "diagnostic_failure_schema_version": DIAGNOSTIC_FAILURE_SCHEMA,
            "adapter_implementation_receipt": implementation_receipt,
            "device_receipt": device_receipt,
            "source_binding_receipt": source_binding_receipt,
            "diagnostic_failure": copy.deepcopy(error.evidence),
        }
    except BaseException as error:
        response = {
            "worker_error": f"{type(error).__name__}: {error}",
            "traceback_tail": _bounded_tail(traceback.format_exc()),
        }
        if "implementation_receipt" in locals():
            response["adapter_implementation_receipt"] = implementation_receipt
        response["device_receipt"] = device_receipt
        response["source_binding_receipt"] = source_binding_receipt
    finally:
        root_logger.removeHandler(warning_handler)
        if adapter is not None:
            try:
                adapter.close()
            except BaseException as error:
                if "response" in locals() and isinstance(response, dict):
                    response.setdefault(
                        "close_error", f"{type(error).__name__}: {error}"
                    )
    response["worker_warnings"] = captured_warnings
    if response.get("evaluation_schema_version") == EVALUATION_SCHEMA:
        existing = response["diagnostics"].get("warnings", [])
        response["diagnostics"]["warnings"] = list(existing) + captured_warnings
    atomic_write_json(response_path, response)
    return 0


_ACTIVE_PROCESS_GROUPS: set[int] = set()
_ACTIVE_LOCK = threading.Lock()
_STOP_REQUESTED = threading.Event()
_STOP_SIGNAL = signal.SIGTERM


def _request_parent_stop(signum: int) -> None:
    """Record a stop without acquiring locks from the signal handler."""
    global _STOP_SIGNAL
    _STOP_SIGNAL = signum
    _STOP_REQUESTED.set()


def _raise_if_stop_requested() -> None:
    if _STOP_REQUESTED.is_set():
        raise ParentStop(_STOP_SIGNAL)


def _register_process_group(pid: int) -> None:
    with _ACTIVE_LOCK:
        _ACTIVE_PROCESS_GROUPS.add(pid)


def _unregister_process_group(pid: int) -> None:
    with _ACTIVE_LOCK:
        _ACTIVE_PROCESS_GROUPS.discard(pid)


def _terminate_process_group(pid: int) -> None:
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        return


def _kill_process_group(pid: int) -> None:
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        return


def _stop_process_group(process: subprocess.Popen[Any]) -> None:
    """Give every member a bounded TERM grace, then kill any surviving descendants."""
    started = time.monotonic()
    _terminate_process_group(process.pid)
    try:
        process.wait(timeout=PROCESS_TERM_GRACE_S)
    except subprocess.TimeoutExpired:
        pass
    remaining = PROCESS_TERM_GRACE_S - (time.monotonic() - started)
    if remaining > 0:
        time.sleep(remaining)
    _kill_process_group(process.pid)
    if process.poll() is None:
        process.wait()


def _terminate_all_active_groups() -> None:
    with _ACTIVE_LOCK:
        pids = list(_ACTIVE_PROCESS_GROUPS)
    for pid in pids:
        _terminate_process_group(pid)
    deadline = time.monotonic() + PROCESS_TERM_GRACE_S
    while time.monotonic() < deadline:
        if not any(_process_group_exists(pid) for pid in pids):
            return
        time.sleep(0.01)
    for pid in pids:
        _kill_process_group(pid)


def _process_group_exists(pid: int) -> bool:
    try:
        os.killpg(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _subprocess_worker(
    *,
    row: dict[str, Any],
    identity: dict[str, Any],
    protocol: dict[str, Any],
    dataset_root: Path,
    output_root: Path,
    simulator: str,
    timeout_s: float,
    implementation_receipt: dict[str, Any] | None = None,
    parent_gpu_receipt: dict[str, Any] | None = None,
) -> ProcessOutcome:
    _raise_if_stop_requested()
    run_id = f"{row['dataset_id']}-{uuid.uuid4().hex}"
    request_path = output_root / ".worker_requests" / f"{run_id}.json"
    response_path = output_root / ".worker_responses" / f"{run_id}.json"
    logs_root = output_root / "worker_logs"
    stdout_path = logs_root / f"{run_id}.stdout.log"
    stderr_path = logs_root / f"{run_id}.stderr.log"
    raw_path = (dataset_root / row["urdf_relative_path"]).resolve()
    try:
        raw_path.relative_to(dataset_root.resolve())
    except ValueError as error:
        raise RuntimeContractError("raw URDF path escapes dataset root") from error
    if not raw_path.is_file():
        raise RuntimeContractError(f"raw URDF missing: {raw_path}")
    atomic_write_json(
        request_path,
        {
            "simulator": simulator,
            "identity": identity,
            "dataset_root": str(dataset_root.resolve()),
            "raw_urdf_path": str(raw_path),
            "row": row,
            "protocol": protocol,
            "implementation_receipt": implementation_receipt,
            "parent_gpu_receipt": parent_gpu_receipt,
        },
    )
    logs_root.mkdir(parents=True, exist_ok=True)
    response_path.parent.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    for protocol_name, environment_name in THREAD_ENV_NAMES.items():
        environment[environment_name] = str(
            protocol["runtime"]["thread_caps"][protocol_name]
        )
    environment["QD_NUM_THREADS"] = str(protocol["runtime"]["thread_caps"]["taichi"])
    if simulator == "genesis":
        binding = protocol["adapters"]["genesis"]["gpu_binding"]
        environment["CUDA_VISIBLE_DEVICES"] = binding["cuda_visible_devices"]
        environment["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker-request",
        str(request_path),
        "--worker-response",
        str(response_path),
    ]
    timed_out = False
    with stdout_path.open("wb") as stdout_handle, stderr_path.open(
        "wb"
    ) as stderr_handle:
        _raise_if_stop_requested()
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=stdout_handle,
            stderr=stderr_handle,
            env=environment,
            start_new_session=True,
        )
        _register_process_group(process.pid)
        try:
            _raise_if_stop_requested()
            try:
                deadline = time.monotonic() + timeout_s
                while process.poll() is None:
                    _raise_if_stop_requested()
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise subprocess.TimeoutExpired(command, timeout_s)
                    try:
                        process.wait(timeout=min(0.1, remaining))
                    except subprocess.TimeoutExpired:
                        continue
            except subprocess.TimeoutExpired:
                timed_out = True
                _stop_process_group(process)
            _raise_if_stop_requested()
        except BaseException:
            _stop_process_group(process)
            raise
        finally:
            _unregister_process_group(process.pid)
    response: dict[str, Any] | None = None
    if response_path.is_file():
        try:
            loaded = json.loads(response_path.read_text(encoding="utf-8"))
            response = (
                loaded
                if isinstance(loaded, dict)
                else {"malformed_nonobject_response": loaded}
            )
        except (OSError, json.JSONDecodeError):
            response = {"malformed_json_response": True}
    stdout = stdout_path.read_text(encoding="utf-8", errors="replace")
    stderr = stderr_path.read_text(encoding="utf-8", errors="replace")
    return ProcessOutcome(
        exit_code=process.returncode,
        signal=(
            -process.returncode
            if process.returncode is not None and process.returncode < 0
            else None
        ),
        timeout=timed_out,
        stdout=_bounded_tail(stdout),
        stderr=_bounded_tail(stderr),
        response=response,
        parent_gpu_receipt=copy.deepcopy(parent_gpu_receipt),
    )


def _parse_ids(value: str | None) -> list[str] | None:
    if value is None:
        return None
    values = value.split(",")
    if not values or any(not _valid_dataset_id(item) for item in values):
        raise RuntimeContractError(
            "--ids must be comma-separated identifiers matching sketch_[0-9]{4}"
        )
    if len(values) != len(set(values)):
        raise RuntimeContractError("--ids must contain unique SketchMobility IDs")
    return values


def _parent_main(args: argparse.Namespace) -> int:
    protocol_path, manifest_path = (
        Path(args.protocol).resolve(),
        Path(args.manifest).resolve(),
    )
    if (
        protocol_path.name != "protocol.json"
        or manifest_path.name != "manifest.json"
        or protocol_path.parent != manifest_path.parent
    ):
        raise RuntimeContractError(
            "protocol and manifest must be one published receipt set"
        )
    protocol, manifest = validate_receipt_set(protocol_path.parent)
    if (
        protocol_path != protocol_path.parent / "protocol.json"
        or manifest_path != manifest_path.parent / "manifest.json"
    ):
        raise RuntimeContractError(
            "protocol and manifest paths must name the published receipt files"
        )
    dataset_root = Path(args.dataset_root).resolve()
    table1_manifest = Path(args.table1_manifest).resolve()
    upstream_roots = {
        "table2": Path(args.table2_root).resolve(),
        "table3": Path(args.table3_root).resolve(),
        "table4": Path(args.table4_root).resolve(),
    }
    validate_runtime_protocol(protocol)
    validate_manifest(
        manifest, dataset_root, table1_manifest, upstream_roots, protocol=protocol
    )
    receipt_root = protocol_path.parent.resolve()
    output_root = validate_runtime_output_root(
        receipt_root, args.phase, args.simulator, Path(args.out)
    )
    validate_output_path(dataset_root, upstream_roots.values(), output_root)
    selected_ids = _parse_ids(args.ids)
    all_rows = manifest["rows"]
    all_ids = {row["dataset_id"] for row in all_rows}
    if args.phase == "qualification" and selected_ids is None:
        raise RuntimeContractError("qualification requires explicit --ids")
    if args.phase == "formal" and selected_ids is not None:
        raise RuntimeContractError(
            "formal phase must use the full frozen cohort without --ids"
        )
    if selected_ids is None:
        rows = all_rows
    else:
        missing = sorted(set(selected_ids) - all_ids)
        if missing:
            raise RuntimeContractError(
                f"requested IDs are outside the frozen manifest: {missing}"
            )
        requested = set(selected_ids)
        rows = [row for row in all_rows if row["dataset_id"] in requested]
    timeout_s = float(
        args.timeout
        if args.timeout is not None
        else protocol["runtime"]["child_timeout_s"]
    )
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise RuntimeContractError("timeout must be finite and positive")
    implementation_receipt = adapter_implementation_receipt(args.simulator)
    if args.simulator == "genesis":
        _query_parent_gpu_receipt(protocol)

    def spawn(row: dict[str, Any], identity: dict[str, Any]) -> ProcessOutcome:
        parent_gpu_receipt = (
            _query_parent_gpu_receipt(protocol) if args.simulator == "genesis" else None
        )
        return _subprocess_worker(
            row=row,
            identity=identity,
            protocol=protocol,
            dataset_root=dataset_root,
            output_root=output_root,
            simulator=args.simulator,
            timeout_s=timeout_s,
            implementation_receipt=implementation_receipt,
            parent_gpu_receipt=parent_gpu_receipt,
        )

    run_intent(
        rows=rows,
        manifest=manifest,
        protocol=protocol,
        simulator=args.simulator,
        phase=args.phase,
        output_root=output_root,
        spawn_worker=spawn,
        workers=args.workers,
        timeout_s=timeout_s,
        implementation_receipt=implementation_receipt,
    )
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulator", choices=sorted(ADAPTER_NAMES))
    parser.add_argument("--phase", choices=("qualification", "formal"))
    parser.add_argument("--dataset-root")
    parser.add_argument("--table1-manifest")
    parser.add_argument("--table2-root")
    parser.add_argument("--table3-root")
    parser.add_argument("--table4-root")
    parser.add_argument("--manifest")
    parser.add_argument("--protocol")
    parser.add_argument("--out")
    parser.add_argument("--ids")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--worker-request", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--worker-response", type=Path, help=argparse.SUPPRESS)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.worker_request is not None or args.worker_response is not None:
        if args.worker_request is None or args.worker_response is None:
            raise RuntimeContractError("worker request and response must be paired")
        return _worker_main(args.worker_request, args.worker_response)
    required = (
        "simulator",
        "phase",
        "dataset_root",
        "table1_manifest",
        "table2_root",
        "table3_root",
        "table4_root",
        "manifest",
        "protocol",
        "out",
    )
    missing = [name for name in required if getattr(args, name) is None]
    if missing:
        raise RuntimeContractError(f"missing parent arguments: {', '.join(missing)}")
    previous_handlers: dict[int, Any] = {}
    _STOP_REQUESTED.clear()

    def stop(signum: int, _frame: Any) -> None:
        _request_parent_stop(signum)

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, stop)
    try:
        return _parent_main(args)
    except ParentStop as stopped:
        return 128 + stopped.signum
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ManifestError, RuntimeContractError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
