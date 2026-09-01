#!/usr/bin/env python3
"""Run Table 3 on the frozen supplementary Infinite Mobility cohort."""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import CancelledError, ThreadPoolExecutor, as_completed
from contextlib import contextmanager
from datetime import datetime, timezone
import errno
import fcntl
import hashlib
from importlib.metadata import version as distribution_version
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import threading
from typing import Any, Iterator
import uuid


SCRIPT_PATH = Path(__file__).absolute()
EXP_ROOT = SCRIPT_PATH.parents[1]
REPO_ROOT = SCRIPT_PATH.parents[2]
CORE_PATH = SCRIPT_PATH.with_name("run_urdf_table3_lam.py")
PROTOCOL_PATH = EXP_ROOT / "URDF-Sim-Ready-Automatic-Evaluation.md"
DEFAULT_COHORT_MANIFEST = (
    EXP_ROOT / "runtime/infinite_mobility_urdf_table123_cohort/manifest.json"
)
DEFAULT_TABLE2_MANIFEST = (
    EXP_ROOT / "runtime/table2_infinite_mobility_720/manifest.json"
)
DEFAULT_OUTPUT_PARENT = EXP_ROOT / "runtime"
FORMAL_PYTHON = EXP_ROOT / ".venv_low_medium/bin/python"

DATASET_NAME = "Infinite Mobility"
FORMAL_N = 720
FORMAL_JOINT_COUNT = 4723
FORMAL_ZERO_JOINT_COUNT = 55
FORMAL_COHORT_MANIFEST_FILE_SHA256 = (
    "cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08"
)
FORMAL_COHORT_MANIFEST_CONTENT_SHA256 = (
    "f5e29f1becd47cae991f5d238dff3f86b2b009365738df3e46cdbea297032c23"
)
FORMAL_DECLARED_JOINT_COUNTS_SHA256 = (
    "1a1306cdd5c1fa9afccff3e81cb2efb5e4038b3dd9a799326282b035cb3c0b35"
)
FORMAL_ZERO_JOINT_ASSET_IDS_SHA256 = (
    "9b58c190e65e742f36bfe03fcf3a3ead49dc147b9b20230c65cd39bc331c83cf"
)
FORMAL_TABLE2_MANIFEST_FILE_SHA256: str | None = (
    "3dce6436aac2d25507d7843a3e0e5cbee130e83e0c24c2bfbfb08467ca356290"
)
FORMAL_TABLE2_MANIFEST_CONTENT_SHA256: str | None = (
    "f1cc7c062767ec6e6cb8d05caea122f17baf42d1a9b12e5bc40d16648c3306c3"
)
DEFAULT_SAMPLES = 21
DEFAULT_WORKERS = 4
DEFAULT_TIMEOUT_SECONDS = 120.0
SPAWN_EAGAIN_TOTAL_WAIT_SECONDS = 1800.0
SPAWN_EAGAIN_INITIAL_BACKOFF_SECONDS = 1.0
SPAWN_EAGAIN_MAX_BACKOFF_SECONDS = 30.0
SPAWN_EAGAIN_BACKOFF_MULTIPLIER = 2.0
FORMAL_RUNTIME_FINGERPRINT = {
    "python_version": [3, 12, 3],
    "numpy": "2.5.1",
    "trimesh": "5.0.0",
}
TABLE2_METRIC_NAMES = (
    "parse_rate",
    "resource_resolution",
    "finite_fields",
    "valid_tree",
    "valid_joint_spec",
    "collision_coverage",
    "inertial_coverage",
    "inertia_validity",
    "strict_urdf_pass",
)
TABLE2_RUNTIME_BINDING_FIELDS = (
    "evaluator_path",
    "evaluator_sha256",
    "protocol_path",
    "protocol_sha256",
    "config",
    "config_sha256",
    "environment",
    "environment_sha256",
)
TABLE2_ARTIFACT_FILES = frozenset(
    {
        "manifest.json",
        "protocol_snapshot.md",
        "environment.json",
        "records.jsonl",
        "summary.json",
        "report.md",
        "checkpoint.json",
    }
)
OUTPUT_ARTIFACT_FILES = frozenset(
    {
        "manifest.json",
        "asset_records.jsonl",
        "summary.json",
        "report.md",
        "environment.json",
        "protocol_snapshot.md",
        "checkpoint.json",
    }
)

APPROVED_FACTORIES = (
    "OfficeChairFactory",
    "BarChairFactory",
    "BeverageFridgeFactory",
    "DishwasherFactory",
    "MicrowaveFactory",
    "OvenFactory",
    "TVFactory",
    "TapFactory",
    "ToiletFactory",
    "LiteDoorFactory",
    "LampFactory",
    "PlateOnRackBaseFactory",
    "KitchenCabinetFactory",
    "VaseFactory",
    "BottleFactory",
    "TableCocktailFactory",
    "TableDiningFactory",
    "PotFactory",
    "PanFactory",
    "WindowFactory",
)
APPROVED_RECOVERY_IDENTITIES = (
    "KitchenCabinetFactory/seed_008",
    "KitchenCabinetFactory/seed_019",
    "KitchenCabinetFactory/seed_020",
    "KitchenCabinetFactory/seed_023",
    "KitchenCabinetFactory/seed_031",
    "OfficeChairFactory/seed_026",
    "WindowFactory/seed_029",
)
CHILD_THREAD_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}


class SpawnEagainRetryBudgetExhausted(RuntimeError):
    """Abort without emitting a metric record so the run can resume."""


class OwnedProcessLifecycleError(RuntimeError):
    """Abort when a spawned worker cannot be reaped after bounded cleanup."""

    def __init__(self, message: str, termination: dict[str, Any]) -> None:
        super().__init__(message)
        self.termination = termination


class LifecycleRunAborted(RuntimeError):
    """Skip a job that has not acquired child-process ownership."""


class FatalLifecycleController:
    def __init__(self, retry_wakeup: threading.Event) -> None:
        self._lock = threading.Lock()
        self._fatal = threading.Event()
        self._retry_wakeup = retry_wakeup
        self._failure: OwnedProcessLifecycleError | None = None

    def mark_fatal(self, failure: OwnedProcessLifecycleError) -> None:
        with self._lock:
            if self._failure is None:
                self._failure = failure
            self._fatal.set()
            self._retry_wakeup.set()

    def raise_if_fatal(self) -> None:
        if self._fatal.is_set():
            raise LifecycleRunAborted(
                "job aborted before Popen after owned process lifecycle fatal"
            ) from self._failure

    def popen(self, factory: Any) -> subprocess.Popen[bytes]:
        with self._lock:
            if self._fatal.is_set():
                raise LifecycleRunAborted(
                    "job aborted at Popen admission after owned process "
                    "lifecycle fatal"
                ) from self._failure
            return factory()

    @property
    def is_fatal(self) -> bool:
        return self._fatal.is_set()

    @property
    def failure(self) -> OwnedProcessLifecycleError | None:
        with self._lock:
            return self._failure


def _spawn_eagain_retry_policy() -> dict[str, Any]:
    return {
        "retryable_exception": "BlockingIOError",
        "retryable_errno": errno.EAGAIN,
        "scope": "one shared cumulative backoff budget per run across all workers",
        "popen_action": "retry the same job/run_token spawn in place",
        "total_backoff_wait_seconds": SPAWN_EAGAIN_TOTAL_WAIT_SECONDS,
        "initial_backoff_seconds": SPAWN_EAGAIN_INITIAL_BACKOFF_SECONDS,
        "maximum_backoff_seconds": SPAWN_EAGAIN_MAX_BACKOFF_SECONDS,
        "backoff_multiplier": SPAWN_EAGAIN_BACKOFF_MULTIPLIER,
        "exhaustion": "raise nonzero with running checkpoint preserved for --resume",
        "other_spawn_failures": "fail-closed asset and declared-joint record",
    }


def _table2_spawn_eagain_retry_policy() -> dict[str, Any]:
    return {
        "retryable_exception": "BlockingIOError",
        "retryable_errno": errno.EAGAIN,
        "scope": "subprocess.Popen only",
        "popen_action": "retry the same spawn in place while preserving active children",
        "total_backoff_wait_seconds": 1800.0,
        "initial_backoff_seconds": 1.0,
        "maximum_backoff_seconds": 30.0,
        "backoff_multiplier": 2.0,
        "exhaustion": "raise nonzero with running checkpoint preserved for --resume",
        "all_other_spawn_step_failures": "fail-closed metric record",
    }


def _wait_spawn_retry(event: threading.Event, seconds: float) -> bool:
    return event.wait(seconds)


class SpawnEagainRetryBudget:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.exhausted = threading.Event()
        self.total_backoff_wait = 0.0

    def reserve_delay(self, consecutive_failures: int) -> tuple[float, float]:
        delay = min(
            SPAWN_EAGAIN_INITIAL_BACKOFF_SECONDS
            * (
                SPAWN_EAGAIN_BACKOFF_MULTIPLIER
                ** min(consecutive_failures, 30)
            ),
            SPAWN_EAGAIN_MAX_BACKOFF_SECONDS,
        )
        with self._lock:
            if self.exhausted.is_set() or (
                self.total_backoff_wait + delay
                > SPAWN_EAGAIN_TOTAL_WAIT_SECONDS
            ):
                self.exhausted.set()
                raise SpawnEagainRetryBudgetExhausted(
                    "spawn EAGAIN retry budget exhausted; running checkpoint is "
                    "preserved for --resume"
                )
            self.total_backoff_wait += delay
            return delay, self.total_backoff_wait

    def raise_if_exhausted(self) -> None:
        if self.exhausted.is_set():
            raise SpawnEagainRetryBudgetExhausted(
                "spawn EAGAIN retry budget exhausted; running checkpoint is "
                "preserved for --resume"
            )


def _load_core():
    spec = importlib.util.spec_from_file_location("infinite_mobility_table3_core", CORE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import shared Table 3 core: {CORE_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CORE = _load_core()


def formal_runtime_fingerprint() -> dict[str, Any]:
    return {
        "python_version": list(sys.version_info[:3]),
        "numpy": CORE.np.__version__,
        "trimesh": distribution_version("trimesh"),
    }


def child_runtime_fingerprint() -> dict[str, Any]:
    """Return the runtime identity expected after child thread vars are frozen."""

    return {
        "python_executable": str(Path(sys.executable).absolute()),
        "python_prefix": str(Path(sys.prefix).absolute()),
        "python_version": list(sys.version_info[:3]),
        "numpy": CORE.np.__version__,
        "trimesh": distribution_version("trimesh"),
        "thread_environment": dict(CHILD_THREAD_ENVIRONMENT),
    }


def _observed_child_runtime_fingerprint() -> dict[str, Any]:
    observed = child_runtime_fingerprint()
    observed["thread_environment"] = {
        name: os.environ.get(name) for name in CHILD_THREAD_ENVIRONMENT
    }
    return observed


def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def _atomic_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def atomic_write_json(path: Path, value: Any) -> None:
    _atomic_bytes(path, json.dumps(
        value, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False
    ).encode("utf-8") + b"\n")


def atomic_write_text(path: Path, value: str) -> None:
    _atomic_bytes(path, value.encode("utf-8"))


def append_jsonl(path: Path, value: dict[str, Any]) -> None:
    with path.open("ab") as handle:
        handle.write(canonical_json_bytes(value) + b"\n")
        handle.flush()
        os.fsync(handle.fileno())


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    _atomic_bytes(path, b"".join(canonical_json_bytes(row) + b"\n" for row in rows))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"JSONL is missing or symlinked: {path}")
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSONL line {line_number}: {path}") from exc
        if not isinstance(value, dict):
            raise ValueError(f"JSONL line {line_number} is not an object: {path}")
        rows.append(value)
    return rows


def _absolute_without_symlink_components(path: Path, label: str) -> Path:
    raw = path.absolute()
    current = Path(raw.anchor)
    for part in raw.parts[1:]:
        current = current / part
        try:
            mode = os.lstat(current).st_mode
        except OSError as exc:
            raise ValueError(f"{label} is missing: {raw}") from exc
        if stat.S_ISLNK(mode):
            raise ValueError(f"{label} has a symlink path component: {current}")
    return raw


def _regular_file(path: Path, label: str) -> Path:
    raw = _absolute_without_symlink_components(path, label)
    mode = os.lstat(raw).st_mode
    if not stat.S_ISREG(mode):
        raise ValueError(f"{label} is not a regular file: {path}")
    return raw


def _read_json(path: Path, label: str) -> dict[str, Any]:
    resolved = _regular_file(path, label)
    try:
        value = json.loads(resolved.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"{label} is invalid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object: {path}")
    return value


def _safe_relative(raw: str, label: str) -> Path:
    relative = Path(raw)
    if not raw or relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"unsafe {label}: {raw!r}")
    return relative


def _contained_file(root: Path, raw_relative: str, label: str) -> Path:
    relative = _safe_relative(raw_relative, label)
    candidate = root / relative
    if candidate.is_symlink():
        raise ValueError(f"{label} is symlinked: {raw_relative}")
    resolved = _regular_file(candidate, label)
    try:
        resolved.relative_to(root.resolve(strict=True))
    except ValueError as exc:
        raise ValueError(f"{label} escapes package: {raw_relative}") from exc
    return resolved


def package_file_manifest(package: Path) -> list[dict[str, Any]]:
    if package.is_symlink():
        raise ValueError(f"package is symlinked: {package}")
    try:
        root = package.resolve(strict=True)
    except OSError as exc:
        raise ValueError(f"package is missing: {package}") from exc
    if not root.is_dir():
        raise ValueError(f"package is not a directory: {package}")
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(root, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if child.is_symlink():
                raise ValueError(
                    f"package contains directory symlink: {child.relative_to(root)}"
                )
        for name in file_names:
            path = current / name
            relative = path.relative_to(root).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            resolved = path.resolve(strict=True)
            resolved.relative_to(root)
            mode = resolved.stat().st_mode
            if not resolved.is_file() or not stat.S_ISREG(mode):
                raise ValueError(f"package entry is not a regular file: {relative}")
            rows.append(
                {
                    "path": relative,
                    "bytes": resolved.stat().st_size,
                    "sha256": sha256_file(resolved),
                }
            )
    return rows


def package_binding(package: Path) -> dict[str, Any]:
    files = package_file_manifest(package)
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def _validate_package_binding(binding: Any, label: str) -> dict[str, Any]:
    if not isinstance(binding, dict) or not isinstance(binding.get("files"), list):
        raise ValueError(f"{label} package binding is malformed")
    files = binding["files"]
    paths: list[str] = []
    total_bytes = 0
    for row in files:
        if not isinstance(row, dict):
            raise ValueError(f"{label} package binding has a non-object file")
        relative = str(row.get("path", ""))
        _safe_relative(relative, f"{label} package binding path")
        digest = row.get("sha256")
        size = row.get("bytes")
        if not isinstance(digest, str) or len(digest) != 64:
            raise ValueError(f"{label} package binding has an invalid SHA256")
        if not isinstance(size, int) or size < 0:
            raise ValueError(f"{label} package binding has an invalid byte count")
        paths.append(relative)
        total_bytes += size
    if len(paths) != len(set(paths)):
        raise ValueError(f"{label} package binding has duplicate paths")
    if binding.get("file_count") != len(files):
        raise ValueError(f"{label} package binding file count mismatch")
    if binding.get("total_bytes") != total_bytes:
        raise ValueError(f"{label} package binding byte count mismatch")
    if binding.get("content_manifest_sha256") != canonical_sha256(files):
        raise ValueError(f"{label} package binding content hash mismatch")
    return binding


def verify_artifact_manifest(
    root: Path,
    *,
    expected_files: frozenset[str] | None = None,
    exact: bool = False,
    required_sidecars: frozenset[str] = frozenset(),
) -> dict[str, Any]:
    root = _absolute_without_symlink_components(root, "artifact root")
    if not stat.S_ISDIR(os.lstat(root).st_mode):
        raise ValueError(f"artifact root is not a directory: {root}")
    artifact_path = root / "artifact_manifest.json"
    artifact = _read_json(artifact_path, "artifact manifest")
    files = artifact.get("files")
    if not isinstance(files, dict) or "manifest.json" not in files:
        raise ValueError("artifact manifest must bind manifest.json")
    if expected_files is not None and set(files) != set(expected_files):
        raise ValueError(
            "artifact closure key mismatch: "
            f"observed={sorted(files)}, expected={sorted(expected_files)}"
        )
    for relative, receipt in files.items():
        if not isinstance(relative, str) or not isinstance(receipt, dict):
            raise ValueError("artifact manifest contains a malformed receipt")
        path = _contained_file(root, relative, f"artifact {relative.replace('_', ' ')}")
        display = relative.replace("_", " ")
        if path.stat().st_size != receipt.get("bytes"):
            raise ValueError(f"artifact {display} byte count mismatch")
        if sha256_file(path) != receipt.get("sha256"):
            raise ValueError(f"artifact {display} hash mismatch")
    for sidecar in required_sidecars:
        if Path(sidecar).name != sidecar or sidecar in files:
            raise ValueError(f"artifact sidecar name is invalid: {sidecar!r}")
        _contained_file(root, sidecar, f"artifact sidecar {sidecar}")
    if exact:
        expected_root = set(files) | {"artifact_manifest.json"} | set(required_sidecars)
        observed_root = {path.name for path in root.iterdir()}
        undeclared = sorted(observed_root - expected_root)
        missing = sorted(expected_root - observed_root)
        if undeclared or missing:
            raise ValueError(
                f"artifact closure has undeclared={undeclared} missing={missing}"
            )
    return artifact


def _artifact_manifest_hash(root: Path) -> str:
    return sha256_file(_regular_file(root / "artifact_manifest.json", "artifact manifest"))


def _verify_path_hash(path_value: Any, hash_value: Any, label: str) -> Path:
    if not isinstance(path_value, str) or not isinstance(hash_value, str):
        raise ValueError(f"{label} binding is missing path or SHA256")
    path = _regular_file(Path(path_value), label)
    if sha256_file(path) != hash_value:
        raise ValueError(f"{label} hash mismatch")
    return path


def _source_bindings(manifest: dict[str, Any]) -> Any:
    if "source_bindings" in manifest:
        return manifest["source_bindings"]
    source = manifest.get("source")
    return source.get("bindings") if isinstance(source, dict) else None


def _verify_source_bindings(bindings: Any, label: str) -> None:
    if isinstance(bindings, list):
        if not bindings:
            raise ValueError(f"{label} source bindings are empty")
        seen: set[str] = set()
        for entry in bindings:
            if not isinstance(entry, dict):
                raise ValueError(f"{label} source bindings contain a non-object")
            name = str(entry.get("label", ""))
            if not name or name in seen:
                raise ValueError(f"{label} source binding labels are invalid")
            seen.add(name)
            _verify_path_hash(entry.get("path"), entry.get("sha256"), f"{label} {name}")
        return
    if isinstance(bindings, dict) and bindings:
        for name, entry in bindings.items():
            if not isinstance(entry, dict):
                raise ValueError(f"{label} source binding {name} is malformed")
            _verify_path_hash(entry.get("path"), entry.get("sha256"), f"{label} {name}")
        return
    raise ValueError(f"{label} source bindings are missing")


def _verify_cohort_evaluation(manifest: dict[str, Any]) -> None:
    evaluation = manifest.get("evaluation")
    if not isinstance(evaluation, dict):
        raise ValueError("cohort evaluator/protocol binding is missing")
    evaluator_pairs = (
        ("freezer_path", "freezer_sha256", "cohort freezer"),
        ("preparer_path", "preparer_sha256", "cohort preparer"),
        ("table1_runner_path", "table1_runner_sha256", "cohort Table 1 runner"),
        ("table1_evaluator_path", "table1_evaluator_sha256", "cohort Table 1 evaluator"),
    )
    verified = 0
    for path_key, hash_key, label in evaluator_pairs:
        if path_key in evaluation or hash_key in evaluation:
            _verify_path_hash(evaluation.get(path_key), evaluation.get(hash_key), label)
            verified += 1
    if verified == 0:
        raise ValueError("cohort evaluator binding is missing")
    _verify_path_hash(
        evaluation.get("protocol_path"), evaluation.get("protocol_sha256"), "cohort protocol"
    )


def _verify_table2_evaluation(manifest: dict[str, Any], root: Path) -> None:
    evaluation = manifest.get("evaluation")
    if not isinstance(evaluation, dict):
        raise ValueError("Table 2 evaluator/protocol binding is missing")
    _verify_path_hash(
        evaluation.get("evaluator_path"),
        evaluation.get("evaluator_sha256"),
        "Table 2 evaluator",
    )
    _verify_path_hash(
        evaluation.get("adapter_path"),
        evaluation.get("adapter_sha256"),
        "Table 2 adapter",
    )
    _verify_path_hash(
        evaluation.get("shared_core_path"),
        evaluation.get("shared_core_sha256"),
        "Table 2 shared core",
    )
    if "protocol_source_path" in evaluation or "protocol_source_sha256_at_freeze" in evaluation:
        _verify_path_hash(
            evaluation.get("protocol_source_path"),
            evaluation.get("protocol_source_sha256_at_freeze"),
            "Table 2 protocol source",
        )
    _verify_path_hash(
        evaluation.get("protocol_path"),
        evaluation.get("protocol_sha256"),
        "Table 2 protocol",
    )
    config = evaluation.get("config")
    if not isinstance(config, dict) or canonical_sha256(config) != evaluation.get("config_sha256"):
        raise ValueError("Table 2 evaluator config hash mismatch")
    environment = evaluation.get("environment")
    if not isinstance(environment, dict) or canonical_sha256(environment) != evaluation.get(
        "environment_sha256"
    ):
        raise ValueError("Table 2 environment hash mismatch")
    adapter_config = evaluation.get("adapter_config")
    if adapter_config is not None and (
        not isinstance(adapter_config, dict)
        or canonical_sha256(adapter_config) != evaluation.get("adapter_config_sha256")
    ):
        raise ValueError("Table 2 adapter config hash mismatch")
    if evaluation.get("environment_file_sha256") is not None:
        environment_file = _regular_file(root / "environment.json", "Table 2 environment file")
        if sha256_file(environment_file) != evaluation["environment_file_sha256"]:
            raise ValueError("Table 2 environment file hash mismatch")
    snapshot = _regular_file(root / "protocol_snapshot.md", "Table 2 protocol snapshot")
    expected_snapshot = evaluation.get("protocol_snapshot_sha256", evaluation.get("protocol_sha256"))
    if sha256_file(snapshot) != expected_snapshot:
        raise ValueError("Table 2 protocol snapshot hash mismatch")


def _validate_formal_table2_result(
    record: dict[str, Any],
    source_row: dict[str, Any],
    evaluation: dict[str, Any],
    manifest_hash: str,
) -> tuple[str, int]:
    source_fields = (
        "selection_index", "selection_rank", "selection_hash", "cohort_record_sha256",
        "asset_id", "factory", "raw_category", "seed", "original_status",
        "recovery_used", "recovery_provenance", "source",
        "declared_joint_count_hint", "baseline_package_sha256", "package",
        "expected_package_path",
        "primary_urdf_relative_path", "primary_urdf_sha256", "model_urdf_sha256",
    )
    if any(record.get(field) != source_row.get(field) for field in source_fields):
        raise ValueError("formal Table 2 result source/order mismatch")
    if record.get("status") not in {"completed", "error", "timeout"}:
        raise ValueError("formal Table 2 result status is invalid")
    metrics = record.get("metrics")
    if not isinstance(metrics, dict) or set(metrics) != set(TABLE2_METRIC_NAMES):
        raise ValueError("formal Table 2 result metric schema mismatch")
    passes: dict[str, bool] = {}
    for name in TABLE2_METRIC_NAMES:
        metric = metrics[name]
        if not isinstance(metric, dict) or type(metric.get("pass")) is not bool:
            raise ValueError(f"formal Table 2 result metric pass is invalid: {name}")
        passes[name] = metric["pass"]
    strict = all(
        passes[name] for name in TABLE2_METRIC_NAMES if name != "strict_urdf_pass"
    )
    if passes["strict_urdf_pass"] is not strict or record.get("strict_urdf_pass") is not strict:
        raise ValueError("formal Table 2 strict metric is inconsistent")
    if record.get("manifest_content_sha256") != manifest_hash:
        raise ValueError("formal Table 2 result manifest binding mismatch")
    if record.get("package_content_manifest_sha256") != source_row["package_binding"][
        "content_manifest_sha256"
    ]:
        raise ValueError("formal Table 2 result package binding mismatch")
    binding = record.get("job_runtime_binding")
    if not isinstance(binding, dict):
        raise ValueError("formal Table 2 result runtime binding is missing")
    token = binding.get("run_token")
    if not isinstance(token, str) or re.fullmatch(r"[0-9a-f]{32}", token) is None:
        raise ValueError("formal Table 2 result runtime token is invalid")
    expected_runtime = {
        "run_token": token,
        **{field: evaluation[field] for field in TABLE2_RUNTIME_BINDING_FIELDS},
    }
    if set(binding) != set(expected_runtime) or binding != expected_runtime:
        raise ValueError("formal Table 2 result runtime binding mismatch")
    origin = record.get("result_origin")
    if origin == "child_attested":
        if record.get("worker_runtime_binding") != binding:
            raise ValueError("formal Table 2 child runtime attestation mismatch")
    elif origin == "parent_synthesized":
        if "worker_runtime_binding" in record:
            raise ValueError("formal Table 2 parent result has false worker attestation")
        if record.get("status") not in {"error", "timeout"}:
            raise ValueError("formal Table 2 parent result origin/status mismatch")
        if any(metrics[name]["pass"] for name in TABLE2_METRIC_NAMES):
            raise ValueError("formal Table 2 parent result is not fail-closed")
    else:
        raise ValueError("formal Table 2 result origin/attestation is invalid")
    completion_order = record.get("completion_order")
    if type(completion_order) is not int or completion_order <= 0:
        raise ValueError("formal Table 2 completion order is invalid")
    return token, completion_order


def verify_formal_table2_publication(
    manifest_path: Path,
    manifest: dict[str, Any],
    expected_records: list[dict[str, Any]],
) -> dict[str, Any]:
    if (
        FORMAL_TABLE2_MANIFEST_FILE_SHA256 is None
        or FORMAL_TABLE2_MANIFEST_CONTENT_SHA256 is None
    ):
        raise RuntimeError(
            "formal Table 2 manifest hashes are not pinned; complete Table 2 before Table 3"
        )
    manifest_path = _regular_file(manifest_path, "formal Table 2 manifest")
    if sha256_file(manifest_path) != FORMAL_TABLE2_MANIFEST_FILE_SHA256:
        raise ValueError("formal Table 2 manifest file hash mismatch")
    if manifest.get("manifest_content_sha256") != FORMAL_TABLE2_MANIFEST_CONTENT_SHA256:
        raise ValueError("formal Table 2 manifest content hash mismatch")
    root = manifest_path.parent
    verify_artifact_manifest(
        root,
        expected_files=TABLE2_ARTIFACT_FILES,
        exact=True,
        required_sidecars=frozenset({".run.lock"}),
    )
    evaluation = manifest.get("evaluation")
    if not isinstance(evaluation, dict):
        raise ValueError("formal Table 2 evaluation binding is missing")
    expected_evaluation = {
        "workers": 8,
        "asset_timeout_seconds": 300.0,
        "standard_parser": "urdfpy",
        "standard_parser_version": "0.0.22",
    }
    for field, expected in expected_evaluation.items():
        if evaluation.get(field) != expected:
            raise ValueError(f"formal Table 2 evaluation mismatch: {field}")
    adapter_config = evaluation.get("adapter_config")
    if not isinstance(adapter_config, dict) or (
        adapter_config.get("workers") != 8
        or float(adapter_config.get("asset_timeout_seconds", -1)) != 300.0
        or adapter_config.get("standard_parser") is not True
    ):
        raise ValueError("formal Table 2 adapter config mismatch")
    expected_table2_retry_policy = _table2_spawn_eagain_retry_policy()
    table2_environment = evaluation.get("environment")
    if (
        adapter_config.get("spawn_eagain_retry_policy")
        != expected_table2_retry_policy
        or not isinstance(table2_environment, dict)
        or table2_environment.get("spawn_eagain_retry_policy")
        != expected_table2_retry_policy
    ):
        raise ValueError("formal Table 2 spawn EAGAIN retry policy mismatch")
    result_rows = load_jsonl(root / "records.jsonl")
    if len(result_rows) != FORMAL_N or len(expected_records) != FORMAL_N:
        raise ValueError("formal Table 2 records must contain the full asset denominator")
    tokens: set[str] = set()
    completion_orders: set[int] = set()
    status_counts: Counter[str] = Counter()
    for record, source_row in zip(result_rows, expected_records):
        token, order = _validate_formal_table2_result(
            record, source_row, evaluation, manifest["manifest_content_sha256"]
        )
        if token in tokens or order in completion_orders:
            raise ValueError("formal Table 2 runtime token/completion order is duplicated")
        tokens.add(token)
        completion_orders.add(order)
        status_counts[str(record["status"])] += 1
    checkpoint = _read_json(root / "checkpoint.json", "formal Table 2 checkpoint")
    if (
        checkpoint.get("state") != "complete"
        or checkpoint.get("completed") != FORMAL_N
        or checkpoint.get("remaining") != 0
        or checkpoint.get("n_eval") != FORMAL_N
        or checkpoint.get("completion_order") != max(completion_orders, default=0)
        or checkpoint.get("manifest_content_sha256") != manifest["manifest_content_sha256"]
        or checkpoint.get("manifest_file_sha256") != sha256_file(manifest_path)
        or checkpoint.get("records_sha256") != sha256_file(root / "records.jsonl")
    ):
        raise ValueError("formal Table 2 checkpoint is not a complete bound receipt")
    summary = _read_json(root / "summary.json", "formal Table 2 summary")
    if (
        summary.get("status") != "completed"
        or summary.get("mode") != "formal"
        or summary.get("classification") != "FORMAL"
        or summary.get("n_eval") != FORMAL_N
        or summary.get("records_present") != FORMAL_N
        or summary.get("records_missing_counted_as_failures") != 0
        or summary.get("manifest_content_sha256") != manifest["manifest_content_sha256"]
        or summary.get("status_counts") != dict(sorted(status_counts.items()))
    ):
        raise ValueError("formal Table 2 summary does not match terminal records")
    return {
        "records_sha256": sha256_file(root / "records.jsonl"),
        "summary_sha256": sha256_file(root / "summary.json"),
        "checkpoint_sha256": sha256_file(root / "checkpoint.json"),
        "artifact_manifest_sha256": sha256_file(root / "artifact_manifest.json"),
        "status_counts": dict(sorted(status_counts.items())),
    }


def _valid_recovery_provenance(value: Any) -> bool:
    if not isinstance(value, dict):
        return False
    for field in ("original_record_sha256", "recovery_record_sha256"):
        digest = value.get(field)
        if not isinstance(digest, str) or len(digest) != 64:
            return False
    return True


def _validate_identity(row: dict[str, Any], expected_index: int, label: str) -> None:
    factory = row.get("factory")
    seed = row.get("seed")
    expected_id = f"{factory}/seed_{seed:03d}" if isinstance(factory, str) and isinstance(seed, int) else None
    if (
        row.get("selection_index") != expected_index
        or row.get("asset_id") != expected_id
        or row.get("raw_category") != factory
    ):
        raise ValueError(f"{label} identity/order mismatch at position {expected_index}")
    recovered = row.get("recovery_used")
    if recovered is True:
        if row.get("original_status") != "TIMEOUT" or not _valid_recovery_provenance(
            row.get("recovery_provenance")
        ):
            raise ValueError(f"{label} recovery provenance mismatch: {row.get('asset_id')}")
    elif recovered is False:
        if row.get("original_status") != "PASS" or row.get("recovery_provenance") is not None:
            raise ValueError(f"{label} primary provenance mismatch: {row.get('asset_id')}")
    else:
        raise ValueError(f"{label} recovery_used is not boolean: {row.get('asset_id')}")


def validate_formal_assets(assets: list[dict[str, Any]]) -> None:
    expected = [
        (index + 1, factory, seed, f"{factory}/seed_{seed:03d}")
        for index, (factory, seed) in enumerate(
            (factory, seed) for factory in APPROVED_FACTORIES for seed in range(36)
        )
    ]
    observed = [
        (row.get("selection_index"), row.get("factory"), row.get("seed"), row.get("asset_id"))
        for row in assets
    ]
    if observed != expected:
        raise ValueError("formal assets must follow the exact 20 x 36 order")
    recovery_ids = {str(row["asset_id"]) for row in assets if row.get("recovery_used") is True}
    if recovery_ids != set(APPROVED_RECOVERY_IDENTITIES):
        raise ValueError("formal assets must preserve the exact seven recovery identities")
    for index, row in enumerate(assets, 1):
        _validate_identity(row, index, "formal cohort")
        if row["asset_id"] in recovery_ids and not _valid_recovery_provenance(
            row.get("recovery_provenance")
        ):
            raise ValueError(f"formal recovery provenance mismatch: {row['asset_id']}")
    if Counter(row["original_status"] for row in assets) != {"PASS": 713, "TIMEOUT": 7}:
        raise ValueError("formal cohort provenance split must be 713 PASS + 7 TIMEOUT recovery")
    joint_counts = [row.get("declared_joint_count_hint") for row in assets]
    if any(type(value) is not int or value < 0 for value in joint_counts):
        raise ValueError("formal declared joint-count hints are invalid")
    if (
        sum(joint_counts) != FORMAL_JOINT_COUNT
        or canonical_sha256(joint_counts) != FORMAL_DECLARED_JOINT_COUNTS_SHA256
    ):
        raise ValueError("formal joint-count denominator must be exactly 4,723 and frozen")
    zero_ids = [
        str(row["asset_id"])
        for row in assets
        if row["declared_joint_count_hint"] == 0
    ]
    if (
        len(zero_ids) != FORMAL_ZERO_JOINT_COUNT
        or canonical_sha256(zero_ids) != FORMAL_ZERO_JOINT_ASSET_IDS_SHA256
    ):
        raise ValueError("formal zero-joint roster is not the frozen 55 assets")


def _resolved_package(raw: Any, label: str) -> Path:
    if not isinstance(raw, str) or not raw:
        raise ValueError(f"{label} package path is missing")
    path = _absolute_without_symlink_components(Path(raw), f"{label} package")
    if not stat.S_ISDIR(os.lstat(path).st_mode):
        raise ValueError(f"{label} package is not a directory")
    return path


def load_inputs(
    cohort_manifest_path: Path, table2_manifest_path: Path, *, formal: bool
) -> dict[str, Any]:
    cohort_manifest_path = _regular_file(cohort_manifest_path, "cohort manifest")
    cohort = _read_json(cohort_manifest_path, "cohort manifest")
    if cohort.get("manifest_content_sha256") != manifest_self_hash(cohort):
        raise ValueError("cohort manifest self-hash mismatch")
    if cohort.get("dataset") != DATASET_NAME:
        raise ValueError("cohort manifest dataset mismatch")
    cohort_root = cohort_manifest_path.parent
    if formal:
        if cohort_manifest_path != DEFAULT_COHORT_MANIFEST.absolute():
            raise ValueError("formal Table 3 requires the canonical cohort manifest")
        if sha256_file(cohort_manifest_path) != FORMAL_COHORT_MANIFEST_FILE_SHA256:
            raise ValueError("formal cohort manifest file hash mismatch")
        if cohort.get("manifest_content_sha256") != FORMAL_COHORT_MANIFEST_CONTENT_SHA256:
            raise ValueError("formal cohort manifest content hash mismatch")
        verify_artifact_manifest(
            cohort_root,
            expected_files=frozenset(
                {"manifest.json", "cohort_protocol_snapshot.json", "source_selection.json"}
            ),
            exact=True,
        )
    else:
        verify_artifact_manifest(cohort_root)
    cohort_artifact_hash = _artifact_manifest_hash(cohort_root)
    cohort_bindings = _source_bindings(cohort)
    _verify_source_bindings(cohort_bindings, "cohort")
    _verify_cohort_evaluation(cohort)
    cohort_assets = cohort.get("assets")
    if not isinstance(cohort_assets, list) or not cohort_assets:
        raise ValueError("cohort assets are missing")
    if cohort.get("N_release") != len(cohort_assets) or cohort.get("N_eval") != len(cohort_assets):
        raise ValueError("cohort asset count mismatch")
    for index, row in enumerate(cohort_assets, 1):
        if not isinstance(row, dict):
            raise ValueError("cohort contains a non-object asset")
        _validate_identity(row, index, "cohort row")

    table2_manifest_path = _regular_file(table2_manifest_path, "Table 2 manifest")
    table2 = _read_json(table2_manifest_path, "Table 2 manifest")
    if table2.get("manifest_content_sha256") != manifest_self_hash(table2):
        raise ValueError("Table 2 manifest self-hash mismatch")
    if table2.get("dataset") != DATASET_NAME:
        raise ValueError("Table 2 manifest dataset mismatch")
    table2_root = table2_manifest_path.parent
    verify_artifact_manifest(table2_root)
    table2_artifact_hash = _artifact_manifest_hash(table2_root)
    _verify_table2_evaluation(table2, table2_root)
    source = table2.get("source")
    if not isinstance(source, dict):
        raise ValueError("Table 2 cohort source binding is missing")
    bound_cohort = _regular_file(
        Path(str(source.get("cohort_manifest_path", ""))), "Table 2 bound cohort manifest"
    )
    if bound_cohort != cohort_manifest_path:
        raise ValueError("Table 2 cohort manifest path mismatch")
    if source.get("cohort_manifest_sha256") != sha256_file(cohort_manifest_path):
        raise ValueError("Table 2 cohort manifest file hash mismatch")
    if source.get("cohort_manifest_content_sha256") != cohort.get("manifest_content_sha256"):
        raise ValueError("Table 2 cohort manifest content hash mismatch")
    if source.get("cohort_artifact_manifest_sha256") != cohort_artifact_hash:
        raise ValueError("Table 2 cohort artifact manifest hash mismatch")
    if canonical_sha256(source.get("source_bindings")) != canonical_sha256(cohort_bindings):
        raise ValueError("Table 2 source bindings differ from the cohort")

    table2_records = table2.get("records")
    if not isinstance(table2_records, list) or len(table2_records) != len(cohort_assets):
        raise ValueError("Table 2 record count mismatch")
    selection = table2.get("selection")
    if isinstance(selection, dict):
        if selection.get("n_eval") not in (None, len(table2_records)):
            raise ValueError("Table 2 selection count mismatch")
        ids_hash = selection.get("selected_asset_ids_sha256")
        if ids_hash is not None and ids_hash != canonical_sha256(
            [row.get("asset_id") for row in table2_records]
        ):
            raise ValueError("Table 2 selected asset IDs hash mismatch")
        if selection.get("selection_order_preserved", True) is not True:
            raise ValueError("Table 2 selection order is not preserved")
        if selection.get("outcome_based_reselection", False) is not False:
            raise ValueError("Table 2 allows outcome-based reselection")

    assets: list[dict[str, Any]] = []
    seen_packages: set[str] = set()
    identity_fields = (
        "selection_index",
        "asset_id",
        "factory",
        "raw_category",
        "seed",
        "original_status",
        "recovery_used",
        "recovery_provenance",
    )
    for index, (cohort_row, table2_row) in enumerate(zip(cohort_assets, table2_records), 1):
        if not isinstance(table2_row, dict):
            raise ValueError(f"Table 2 record at position {index} is not an object")
        if any(table2_row.get(field) != cohort_row.get(field) for field in identity_fields):
            raise ValueError(f"Table 2 row identity/order mismatch at position {index}")
        _validate_identity(table2_row, index, "Table 2 row")
        expected_row_provenance = {
            "selection_rank": index,
            "selection_hash": canonical_sha256([index - 1, cohort_row["asset_id"]]),
            "cohort_record_sha256": canonical_sha256(cohort_row),
            "source": cohort_row.get("source"),
            "declared_joint_count_hint": cohort_row.get("declared_joint_count_hint"),
            "baseline_package_sha256": cohort_row.get("baseline_package_sha256"),
        }
        if any(
            table2_row.get(field) != expected
            for field, expected in expected_row_provenance.items()
        ):
            raise ValueError(
                f"Table 2 row provenance mismatch: {table2_row.get('asset_id')}"
            )
        cohort_package = _resolved_package(cohort_row.get("package_path"), "cohort")
        table2_package = _resolved_package(
            table2_row.get("package", table2_row.get("package_path")), "Table 2"
        )
        if table2_package != cohort_package:
            raise ValueError(f"Table 2 package path mismatch: {table2_row.get('asset_id')}")
        package_text = str(table2_package)
        if package_text in seen_packages:
            raise ValueError(f"Table 2 package is duplicated: {package_text}")
        seen_packages.add(package_text)
        cohort_binding = _validate_package_binding(
            cohort_row.get("package_binding"), "cohort"
        )
        table2_binding = _validate_package_binding(
            table2_row.get("package_binding"), "Table 2"
        )
        if table2_binding != cohort_binding:
            raise ValueError(f"Table 2 package binding mismatch: {table2_row.get('asset_id')}")
        relative = table2_row.get(
            "primary_urdf_relative_path",
            table2_row.get("urdf_relpath", cohort_row.get("urdf_relpath")),
        )
        if relative != cohort_row.get("urdf_relpath"):
            raise ValueError(f"Table 2 URDF relative path mismatch: {table2_row.get('asset_id')}")
        urdf = _contained_file(table2_package, str(relative), "selected URDF")
        urdf_hash = table2_row.get(
            "primary_urdf_sha256", table2_row.get("model_urdf_sha256")
        )
        if urdf_hash != cohort_row.get("primary_urdf_sha256"):
            raise ValueError(f"Table 2 URDF hash mismatch: {table2_row.get('asset_id')}")
        model_hash = table2_row.get("model_urdf_sha256")
        if model_hash is not None and model_hash != urdf_hash:
            raise ValueError(f"Table 2 model/primary URDF hash mismatch: {table2_row.get('asset_id')}")
        declared_hint = table2_row.get(
            "declared_joint_count_hint", cohort_row.get("declared_joint_count_hint")
        )
        if not isinstance(declared_hint, int) or declared_hint < 0:
            raise ValueError(f"declared joint hint is invalid: {table2_row.get('asset_id')}")
        assets.append(
            {
                **table2_row,
                "asset_key": table2_row["asset_id"],
                "category": table2_row["raw_category"],
                "package": package_text,
                "package_binding": table2_binding,
                "package_content_manifest_sha256": table2_binding[
                    "content_manifest_sha256"
                ],
                "urdf_path": str(urdf),
                "urdf_relpath": str(relative),
                "urdf_sha256": urdf_hash,
                "declared_joint_count_hint": declared_hint,
            }
        )

    if formal:
        if (
            cohort.get("N_release") != FORMAL_N
            or cohort.get("N_eval") != FORMAL_N
            or cohort.get("factory_order") != list(APPROVED_FACTORIES)
            or cohort.get("seeds") != list(range(36))
        ):
            raise ValueError("formal cohort must be the exact 20 x 36 generated cohort")
        if table2.get("mode") != "formal" or table2.get("classification") != "FORMAL":
            raise ValueError("formal Table 3 requires this run's formal Table 2 manifest")
        validate_formal_assets(assets)
        table2_receipt = verify_formal_table2_publication(
            table2_manifest_path, table2, table2_records
        )
    else:
        table2_receipt = None

    return {
        "cohort_manifest_path": str(cohort_manifest_path),
        "cohort_manifest_file_sha256": sha256_file(cohort_manifest_path),
        "cohort_manifest_content_sha256": cohort["manifest_content_sha256"],
        "cohort_artifact_manifest_sha256": cohort_artifact_hash,
        "table2_manifest_path": str(table2_manifest_path),
        "table2_manifest_file_sha256": sha256_file(table2_manifest_path),
        "table2_manifest_content_sha256": table2["manifest_content_sha256"],
        "table2_artifact_manifest_sha256": table2_artifact_hash,
        "table2_completion_receipt": table2_receipt,
        "source_bindings": cohort_bindings,
        "assets": assets,
    }


def _environment() -> dict[str, Any]:
    environment = dict(CORE._environment())
    environment.update(
        {
            "python_executable": str(Path(sys.executable).absolute()),
            "python_prefix": str(Path(sys.prefix).absolute()),
            "core_path": str(CORE_PATH),
            "core_sha256": sha256_file(CORE_PATH),
            "child_thread_environment": dict(CHILD_THREAD_ENVIRONMENT),
            "spawn_eagain_retry_policy": _spawn_eagain_retry_policy(),
        }
    )
    try:
        import trimesh

        environment["trimesh"] = trimesh.__version__
    except ImportError:
        environment["trimesh"] = None
    return environment


def _config(args: argparse.Namespace) -> dict[str, Any]:
    config = job_evaluation_config(samples=args.samples)
    config.update({
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "workers": args.workers,
    })
    return config


def job_evaluation_config(*, samples: int) -> dict[str, Any]:
    return {
        "protocol_id": "urdf-sim-ready-table3-infinite-mobility-full-generated-v1",
        "samples_per_joint": samples,
        "continuous_interval": list(CORE.CONTINUOUS_INTERVAL),
        "continuous_non_degenerate_policy": "maximum_excursion_from_q0_over_frozen_interval",
        "q0_policy": "zero_clipped_to_declared_interval",
        "translation_motion_threshold_bbox_diagonal": CORE.TRANSLATION_MOTION_THRESHOLD,
        "rotation_motion_threshold_rad": CORE.ROTATION_MOTION_THRESHOLD_RAD,
        "unchanged_translation_tolerance_bbox_diagonal": CORE.UNCHANGED_TRANSLATION_TOLERANCE,
        "unchanged_rotation_tolerance_rad": CORE.UNCHANGED_ROTATION_TOLERANCE_RAD,
        "roundtrip_translation_tolerance_bbox_diagonal": CORE.ROUNDTRIP_TRANSLATION_TOLERANCE,
        "roundtrip_rotation_tolerance_rad": CORE.ROUNDTRIP_ROTATION_TOLERANCE_RAD,
        "strict_asset_requires_at_least_one_declared_movable_joint": True,
        "child_process": "one fresh interpreter per asset",
        "package_binding": "Table 2 full recursive binding checked before and after evaluation",
        "denominator_policy": "all frozen assets and declared movable joints fail closed",
        "spawn_eagain_retry_policy": _spawn_eagain_retry_policy(),
    }


def build_manifest(
    args: argparse.Namespace,
    loaded: dict[str, Any],
    protocol_bytes: bytes,
    environment: dict[str, Any],
) -> dict[str, Any]:
    selected = loaded["assets"] if args.limit is None else loaded["assets"][: args.limit]
    config = _config(args)
    protocol_sha256 = hashlib.sha256(protocol_bytes).hexdigest()
    frozen_records = []
    for row in selected:
        run_token = canonical_sha256(
            {
                "protocol_sha256": protocol_sha256,
                "table2_manifest_content_sha256": loaded[
                    "table2_manifest_content_sha256"
                ],
                "selection_index": row["selection_index"],
                "asset_id": row["asset_id"],
                "package_content_manifest_sha256": row[
                    "package_content_manifest_sha256"
                ],
            }
        )[:32]
        frozen_records.append({**row, "run_token": run_token})
    if len({row["run_token"] for row in frozen_records}) != len(frozen_records):
        raise RuntimeError("frozen Table 3 run tokens are not unique")
    child_runtime = child_runtime_fingerprint()
    manifest = {
        "schema_version": 1,
        "dataset": DATASET_NAME,
        "classification": "FORMAL" if args.mode == "formal" else "NON_FORMAL_SMOKE",
        "cohort_type": "SUPPLEMENTARY_FULL_GENERATED_COHORT_NOT_OFFICIAL_FINITE_RELEASE",
        "created_at": utc_now(),
        "source": {
            key: loaded[key]
            for key in (
                "cohort_manifest_path",
                "cohort_manifest_file_sha256",
                "cohort_manifest_content_sha256",
                "cohort_artifact_manifest_sha256",
                "table2_manifest_path",
                "table2_manifest_file_sha256",
                "table2_manifest_content_sha256",
                "table2_artifact_manifest_sha256",
                "table2_completion_receipt",
                "source_bindings",
            )
        },
        "selection": {
            "algorithm": "exact Table 2 records order; optional smoke prefix only",
            "requested_limit": args.limit,
            "n_release": len(loaded["assets"]),
            "n_eval": len(selected),
            "selected_asset_ids_sha256": canonical_sha256(
                [row["asset_id"] for row in frozen_records]
            ),
            "selected_package_bindings_sha256": canonical_sha256(
                [row["package_binding"] for row in frozen_records]
            ),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
            "recovery_count": sum(bool(row["recovery_used"]) for row in frozen_records),
        },
        "evaluation": {
            "protocol_path": str(args.protocol_path.absolute()),
            "protocol_sha256": protocol_sha256,
            "protocol_snapshot_name": "protocol_snapshot.md",
            "adapter_path": str(SCRIPT_PATH),
            "adapter_sha256": sha256_file(SCRIPT_PATH),
            "core_evaluator_path": str(CORE_PATH),
            "core_evaluator_sha256": sha256_file(CORE_PATH),
            "config": config,
            "config_sha256": canonical_sha256(config),
            "environment": environment,
            "environment_sha256": canonical_sha256(environment),
            "child_runtime": child_runtime,
            "child_runtime_sha256": canonical_sha256(child_runtime),
        },
        "records": frozen_records,
    }
    manifest["manifest_content_sha256"] = manifest_self_hash(manifest)
    return manifest


def make_job(
    row: dict[str, Any], *, manifest_content_sha256: str, samples: int, mode: str
) -> dict[str, Any]:
    run_token = row.get("run_token")
    if not isinstance(run_token, str) or re.fullmatch(r"[0-9a-f]{32}", run_token) is None:
        run_token = canonical_sha256(
            [manifest_content_sha256, row["selection_index"], row["asset_id"]]
        )[:32]
    return {
        **row,
        "samples": samples,
        "mode": mode,
        "manifest_content_sha256": manifest_content_sha256,
        "run_token": run_token,
        "parent_pid": os.getpid(),
    }


def attach_runtime_binding(
    job: dict[str, Any], manifest: dict[str, Any], manifest_path: Path
) -> dict[str, Any]:
    evaluation = manifest.get("evaluation")
    if not isinstance(evaluation, dict):
        raise ValueError("manifest evaluation binding is missing")
    manifest_path = _regular_file(manifest_path, "Table 3 run manifest")
    config = evaluation.get("config")
    child_runtime = evaluation.get("child_runtime")
    if not isinstance(config, dict) or not isinstance(child_runtime, dict):
        raise ValueError("manifest config/child runtime binding is missing")
    binding = {
        "run_token": job["run_token"],
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "manifest_path": str(manifest_path),
        "manifest_file_sha256": sha256_file(manifest_path),
        "adapter_path": evaluation["adapter_path"],
        "adapter_sha256": evaluation["adapter_sha256"],
        "core_evaluator_path": evaluation["core_evaluator_path"],
        "core_evaluator_sha256": evaluation["core_evaluator_sha256"],
        "protocol_path": evaluation["protocol_path"],
        "protocol_sha256": evaluation["protocol_sha256"],
        "config": config,
        "config_sha256": evaluation["config_sha256"],
        "child_runtime": child_runtime,
        "child_runtime_sha256": evaluation["child_runtime_sha256"],
    }
    job["evaluation_config"] = config
    for field in ("workers", "asset_timeout_seconds"):
        if field in config:
            job[field] = config[field]
    job["runtime_binding"] = binding
    return job


def _observe_runtime_binding(job: dict[str, Any]) -> dict[str, Any]:
    expected = job["runtime_binding"]
    observed_runtime = _observed_child_runtime_fingerprint()
    observed_config = job_evaluation_config(samples=job["samples"])
    for field in ("workers", "asset_timeout_seconds"):
        if field in job.get("evaluation_config", {}):
            observed_config[field] = job.get(field)
    observed = {
        "run_token": job.get("run_token"),
        "manifest_content_sha256": job.get("manifest_content_sha256"),
        "manifest_path": expected.get("manifest_path"),
        "manifest_file_sha256": sha256_file(
            _regular_file(Path(expected["manifest_path"]), "Table 3 run manifest")
        ),
        "adapter_path": str(SCRIPT_PATH),
        "adapter_sha256": sha256_file(SCRIPT_PATH),
        "core_evaluator_path": str(CORE_PATH),
        "core_evaluator_sha256": sha256_file(CORE_PATH),
        "protocol_path": expected.get("protocol_path"),
        "protocol_sha256": sha256_file(
            _regular_file(Path(expected["protocol_path"]), "Table 3 protocol")
        ),
        "config": observed_config,
        "config_sha256": canonical_sha256(observed_config),
        "child_runtime": observed_runtime,
        "child_runtime_sha256": canonical_sha256(observed_runtime),
    }
    return observed


def verify_live_runtime_binding(
    manifest: dict[str, Any], manifest_path: Path
) -> None:
    evaluation = manifest.get("evaluation")
    if not isinstance(evaluation, dict):
        raise RuntimeError("live evaluation binding is missing")
    checks = (
        (SCRIPT_PATH, evaluation.get("adapter_path"), evaluation.get("adapter_sha256"), "adapter"),
        (CORE_PATH, evaluation.get("core_evaluator_path"), evaluation.get("core_evaluator_sha256"), "core evaluator"),
        (Path(str(evaluation.get("protocol_path", ""))), evaluation.get("protocol_path"), evaluation.get("protocol_sha256"), "protocol"),
    )
    for actual, frozen_path, frozen_hash, label in checks:
        if str(actual.absolute()) != frozen_path or sha256_file(
            _regular_file(actual, f"live {label}")
        ) != frozen_hash:
            raise RuntimeError(f"live {label} binding drift")
    expected_retry_policy = _spawn_eagain_retry_policy()
    for binding_name in ("config", "environment"):
        binding = evaluation.get(binding_name)
        if (
            not isinstance(binding, dict)
            or binding.get("spawn_eagain_retry_policy") != expected_retry_policy
        ):
            raise RuntimeError(
                f"live {binding_name} spawn EAGAIN retry policy drift"
            )
    if canonical_sha256(evaluation.get("config")) != evaluation.get("config_sha256"):
        raise RuntimeError("live evaluation config binding drift")
    if "environment" in evaluation and canonical_sha256(
        evaluation.get("environment")
    ) != evaluation.get("environment_sha256"):
        raise RuntimeError("live environment binding drift")
    expected_child = evaluation.get("child_runtime")
    if (
        canonical_sha256(expected_child) != evaluation.get("child_runtime_sha256")
        or expected_child != child_runtime_fingerprint()
    ):
        raise RuntimeError("live child runtime binding drift")
    checked_manifest = _regular_file(manifest_path, "live Table 3 manifest")
    on_disk = _read_json(checked_manifest, "live Table 3 manifest")
    if on_disk != manifest or (
        manifest.get("manifest_content_sha256") == manifest_self_hash(manifest)
        and on_disk.get("manifest_content_sha256") != manifest_self_hash(on_disk)
    ):
        raise RuntimeError("live manifest binding drift")


def _worker_evidence(job: dict[str, Any]) -> dict[str, Any]:
    return {
        "fresh_interpreter": os.getpid() != job.get("parent_pid"),
        "child_pid": os.getpid(),
        "parent_pid": job.get("parent_pid"),
        "python_executable": str(Path(sys.executable).absolute()),
        "package_binding_before": False,
        "package_binding_after": False,
        "runtime_binding_expected": job.get("runtime_binding"),
        "runtime_binding_observed": None,
        "runtime_binding_match": False,
    }


def _parent_worker_evidence(job: dict[str, Any], child_pid: int | None = None) -> dict[str, Any]:
    return {
        "fresh_interpreter": child_pid is not None and child_pid != os.getpid(),
        "child_pid": child_pid,
        "parent_pid": os.getpid(),
        "python_executable": str(Path(sys.executable).absolute()),
        "package_binding_before": False,
        "package_binding_after": False,
        "runtime_binding_expected": job.get("runtime_binding"),
        "runtime_binding_observed": None,
        "runtime_binding_match": False,
    }


def _normalize_failed_joint_samples(record: dict[str, Any], samples: int) -> None:
    for joint in record.get("joints", []):
        joint["sample_count_expected"] = samples


def _bind_record(
    record: dict[str, Any],
    job: dict[str, Any],
    *,
    result_origin: str,
    worker_evidence: dict[str, Any],
) -> dict[str, Any]:
    record.update(
        {
            "asset_id": job["asset_id"],
            "factory": job["factory"],
            "category": job["raw_category"],
            "raw_category": job["raw_category"],
            "seed": job["seed"],
            "selection_index": job["selection_index"],
            "selection_rank": job.get("selection_rank", job["selection_index"]),
            "selection_hash": job.get("selection_hash"),
            "original_status": job["original_status"],
            "recovery_used": job["recovery_used"],
            "recovery_provenance": job["recovery_provenance"],
            "source": job.get("source"),
            "package": job["package"],
            "package_content_manifest_sha256": job[
                "package_content_manifest_sha256"
            ],
            "urdf_relpath": job["urdf_relpath"],
            "urdf_sha256": job["urdf_sha256"],
            "declared_joint_count_hint": job["declared_joint_count_hint"],
            "samples": job["samples"],
            "manifest_content_sha256": job["manifest_content_sha256"],
            "run_token": job["run_token"],
            "result_origin": result_origin,
            "worker_evidence": worker_evidence,
            "completed_at": utc_now(),
        }
    )
    return record


def _failed_for_job(
    job: dict[str, Any], reason: str, *, status_value: str = "error", evidence: dict[str, Any]
) -> dict[str, Any]:
    record = CORE.failed_record(
            job["asset_key"],
            job["declared_joint_count_hint"],
            reason,
            status=status_value,
        )
    _normalize_failed_joint_samples(record, job["samples"])
    return _bind_record(
        record,
        job,
        result_origin="parent_synthesized",
        worker_evidence=evidence,
    )


def run_internal_job(job_path: Path, result_path: Path) -> int:
    job = _read_json(job_path, "internal job")
    evidence = _worker_evidence(job)
    runtime_reason: str | None = None
    try:
        observed_binding = _observe_runtime_binding(job)
    except Exception as exc:  # noqa: BLE001
        observed_binding = {
            "observation_error": f"{type(exc).__name__}: {exc}",
            "run_token": job.get("run_token"),
        }
    evidence["runtime_binding_observed"] = observed_binding
    evidence["runtime_binding_match"] = observed_binding == job.get("runtime_binding")
    if not evidence["runtime_binding_match"]:
        runtime_reason = "child runtime binding attestation mismatch"

    action = job.get("internal_test_action")
    if action is not None:
        if job.get("mode") != "smoke":
            raise ValueError("internal test actions are forbidden outside smoke mode")
        if action == "sleep":
            import time

            time.sleep(float(job.get("internal_test_seconds", 60)))
        elif action == "raise":
            raise RuntimeError("requested smoke child failure")
        else:
            raise ValueError(f"unknown internal test action: {action!r}")

    package = Path(job["package"])
    urdf = Path(job["urdf_path"])
    reason: str | None = runtime_reason
    try:
        if reason is not None:
            raise RuntimeError(reason)
        before = package_binding(package)
        evidence["observed_package_binding_before_sha256"] = canonical_sha256(before)
        evidence["package_binding_before"] = before == job["package_binding"]
        if not evidence["package_binding_before"]:
            reason = "selected package changed before evaluation"
        elif urdf.is_symlink() or not urdf.is_file():
            reason = "selected URDF is missing or symlinked"
        elif sha256_file(urdf) != job["urdf_sha256"]:
            reason = "selected URDF changed before evaluation"
    except Exception as exc:  # noqa: BLE001
        if runtime_reason is None:
            reason = f"pre-evaluation package binding failed: {type(exc).__name__}: {exc}"

    if reason is None:
        record = CORE.evaluate_urdf(
            urdf,
            job["asset_key"],
            samples=job["samples"],
            declared_joint_count_hint=job["declared_joint_count_hint"],
        )
        try:
            after = package_binding(package)
            evidence["observed_package_binding_after_sha256"] = canonical_sha256(after)
            evidence["package_binding_after"] = after == job["package_binding"]
        except Exception as exc:  # noqa: BLE001
            evidence["package_binding_after"] = False
            reason = f"post-evaluation package binding failed: {type(exc).__name__}: {exc}"
        if reason is None and not evidence["package_binding_after"]:
            reason = "selected package changed during evaluation"
        if reason is None and (urdf.is_symlink() or not urdf.is_file()):
            reason = "selected URDF disappeared during evaluation"
        if reason is None and sha256_file(urdf) != job["urdf_sha256"]:
            reason = "selected URDF changed during evaluation"
    else:
        record = CORE.failed_record(
            job["asset_key"], job["declared_joint_count_hint"], reason
        )

    if reason is not None:
        record = CORE.failed_record(
            job["asset_key"], job["declared_joint_count_hint"], reason
        )
        _normalize_failed_joint_samples(record, job["samples"])
    bound = _bind_record(
        record,
        job,
        result_origin="child_attested",
        worker_evidence=evidence,
    )
    atomic_write_json(result_path, bound)
    return 0


JOINT_RESULT_FIELDS = frozenset(
    {
        "joint_name", "joint_type", "range_source", "range_lower", "range_upper",
        "sample_count_expected", "sample_count_executed", "sample_values",
        "valid_range_pass", "joint_sweep_success", "non_degenerate_motion_pass",
        "subtree_consistency_pass", "fk_roundtrip_pass", "joint_level_pass",
        "max_descendant_normalized_translation", "max_descendant_rotation_rad",
        "max_non_descendant_translation", "max_non_descendant_rotation_rad",
        "fk_roundtrip_max_translation", "fk_roundtrip_max_rotation_rad", "issue",
    }
)
JOINT_BOOLEAN_FIELDS = (
    "valid_range_pass", "joint_sweep_success", "non_degenerate_motion_pass",
    "subtree_consistency_pass", "fk_roundtrip_pass", "joint_level_pass",
)


def validate_record(
    job: dict[str, Any],
    record: Any,
    *,
    require_completion_order: bool = False,
    expected_child_pid: int | None = None,
) -> dict[str, Any]:
    if not isinstance(record, dict):
        raise ValueError("worker result is not an object")
    expected = {
        "asset_key": job["asset_key"],
        "asset_id": job["asset_id"],
        "selection_index": job["selection_index"],
        "manifest_content_sha256": job["manifest_content_sha256"],
        "package_content_manifest_sha256": job["package_content_manifest_sha256"],
        "urdf_sha256": job["urdf_sha256"],
        "samples": job["samples"],
        "run_token": job["run_token"],
        "factory": job["factory"],
        "raw_category": job["raw_category"],
        "seed": job["seed"],
        "original_status": job["original_status"],
        "recovery_used": job["recovery_used"],
        "recovery_provenance": job["recovery_provenance"],
    }
    for field, value in expected.items():
        if record.get(field) != value:
            raise ValueError(f"worker result {field} binding mismatch")
    status_value = record.get("status")
    if status_value not in {"completed", "error", "timeout"}:
        raise ValueError("worker result status is invalid")
    count = record.get("declared_joint_count")
    joints = record.get("joints")
    if (
        count != job["declared_joint_count_hint"]
        or not isinstance(joints, list)
        or len(joints) != count
    ):
        raise ValueError("worker result declared joint denominator mismatch")
    for index, joint in enumerate(joints):
        if not isinstance(joint, dict) or not JOINT_RESULT_FIELDS.issubset(joint):
            raise ValueError(f"worker result joint schema mismatch at index {index}")
        if any(type(joint.get(field)) is not bool for field in JOINT_BOOLEAN_FIELDS):
            raise ValueError(f"worker result joint boolean schema mismatch at index {index}")
        if (
            type(joint.get("sample_count_expected")) is not int
            or joint["sample_count_expected"] != job["samples"]
            or type(joint.get("sample_count_executed")) is not int
            or not 0 <= joint["sample_count_executed"] <= job["samples"]
            or not isinstance(joint.get("sample_values"), list)
        ):
            raise ValueError(f"worker result joint sample schema mismatch at index {index}")
        if (
            not isinstance(joint.get("joint_name"), str)
            or not joint["joint_name"]
            or not isinstance(joint.get("joint_type"), str)
            or joint.get("issue") is not None
            and not isinstance(joint.get("issue"), str)
        ):
            raise ValueError(f"worker result joint identity schema mismatch at index {index}")
        numeric_fields = (
            "range_lower", "range_upper",
            "max_descendant_normalized_translation", "max_descendant_rotation_rad",
            "max_non_descendant_translation", "max_non_descendant_rotation_rad",
            "fk_roundtrip_max_translation", "fk_roundtrip_max_rotation_rad",
        )
        if any(
            value is not None
            and (type(value) not in {int, float} or not math.isfinite(float(value)))
            for value in (joint.get(field) for field in numeric_fields)
        ) or any(
            type(value) not in {int, float} or not math.isfinite(float(value))
            for value in joint["sample_values"]
        ):
            raise ValueError(f"worker result joint numeric schema mismatch at index {index}")
        if joint["joint_sweep_success"] is not (
            joint["sample_count_executed"] == job["samples"]
        ):
            raise ValueError(f"worker result joint sweep count mismatch at index {index}")
        if joint["joint_sweep_success"] and len(joint["sample_values"]) != job["samples"]:
            raise ValueError(f"worker result joint sample values mismatch at index {index}")
        expected_joint_pass = all(bool(joint[field]) for field in JOINT_BOOLEAN_FIELDS[:-1])
        if joint["joint_level_pass"] is not expected_joint_pass:
            raise ValueError(f"worker result joint pass is inconsistent at index {index}")
    strict = record.get("strict_kinematic_pass")
    if type(strict) is not bool:
        raise ValueError("worker result strict pass is not boolean")
    expected_strict = bool(joints) and all(joint["joint_level_pass"] for joint in joints)
    if strict is not expected_strict:
        raise ValueError("worker result strict pass is inconsistent")
    if status_value != "completed" and strict:
        raise ValueError("worker failure must fail closed")
    if type(record.get("parse_success")) is not bool or type(record.get("tree_valid")) is not bool:
        raise ValueError("worker result parse/tree schema mismatch")
    if record["tree_valid"] and not record["parse_success"]:
        raise ValueError("worker result tree cannot be valid without parsing")
    if status_value == "completed" and record.get("error") is not None:
        raise ValueError("completed worker result has an error")
    if status_value != "completed" and not isinstance(record.get("error"), str):
        raise ValueError("failed worker result lacks an error")
    origin = record.get("result_origin")
    if origin not in {"child_attested", "parent_synthesized"}:
        raise ValueError("worker result origin mismatch")
    evidence = record.get("worker_evidence")
    if not isinstance(evidence, dict):
        raise ValueError("worker result attestation is missing")
    if origin == "child_attested":
        if evidence.get("fresh_interpreter") is not True:
            raise ValueError("worker result lacks fresh-interpreter attestation")
        child_pid = evidence.get("child_pid")
        if type(child_pid) is not int or child_pid <= 0 or child_pid == job.get("parent_pid"):
            raise ValueError("worker result child PID attestation is invalid")
        if expected_child_pid is not None and child_pid != expected_child_pid:
            raise ValueError("worker result child PID binding mismatch")
        expected_runtime = evidence.get("runtime_binding_expected")
        observed_runtime = evidence.get("runtime_binding_observed")
        runtime_match = expected_runtime == observed_runtime == job.get("runtime_binding")
        if evidence.get("runtime_binding_match") is not runtime_match:
            raise ValueError("worker result runtime attestation is inconsistent")
        if status_value == "completed" and not runtime_match:
            raise ValueError("completed worker result lacks runtime attestation")
        if status_value == "completed" and (
            evidence.get("package_binding_before") is not True
            or evidence.get("package_binding_after") is not True
        ):
            raise ValueError("completed worker result lacks package attestation")
        if runtime_match and evidence.get("python_executable") != job[
            "runtime_binding"
        ]["child_runtime"]["python_executable"]:
            raise ValueError("worker result Python executable attestation mismatch")
        expected_package_receipt = canonical_sha256(job["package_binding"])
        if evidence.get("package_binding_before") is True and evidence.get(
            "observed_package_binding_before_sha256"
        ) != expected_package_receipt:
            raise ValueError("worker result package-before attestation mismatch")
        if evidence.get("package_binding_after") is True and evidence.get(
            "observed_package_binding_after_sha256"
        ) != expected_package_receipt:
            raise ValueError("worker result package-after attestation mismatch")
    elif status_value not in {"error", "timeout"} or strict:
        raise ValueError("parent-synthesized result must fail closed")
    elif (
        evidence.get("runtime_binding_expected") != job.get("runtime_binding")
        or evidence.get("runtime_binding_observed") is not None
        or evidence.get("runtime_binding_match") is not False
    ):
        raise ValueError("parent-synthesized result runtime binding is invalid")
    if require_completion_order:
        completion_order = record.get("completion_order")
        if type(completion_order) is not int or completion_order <= 0:
            raise ValueError("worker result completion order is invalid")
    return record


_validate_record = validate_record


def _popen_job_with_spawn_retry(
    command: list[str],
    *,
    environment: dict[str, str],
    retry_budget: SpawnEagainRetryBudget,
    lifecycle_controller: FatalLifecycleController,
) -> subprocess.Popen[bytes]:
    consecutive_failures = 0
    while True:
        lifecycle_controller.raise_if_fatal()
        retry_budget.raise_if_exhausted()
        try:
            process = lifecycle_controller.popen(
                lambda: subprocess.Popen(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    start_new_session=True,
                    env=environment,
                )
            )
        except BlockingIOError as error:
            if error.errno != errno.EAGAIN:
                raise
            lifecycle_controller.raise_if_fatal()
            delay, total = retry_budget.reserve_delay(consecutive_failures)
            consecutive_failures += 1
            print(
                f"spawn EAGAIN; retrying the same job/run_token after {delay:g}s "
                f"(shared backoff {total:g}/"
                f"{SPAWN_EAGAIN_TOTAL_WAIT_SECONDS:g}s)",
                flush=True,
            )
            if _wait_spawn_retry(retry_budget.exhausted, delay):
                lifecycle_controller.raise_if_fatal()
                retry_budget.raise_if_exhausted()
            continue
        return process


def _owned_process_termination(
    process: subprocess.Popen[bytes], *, reason: str
) -> dict[str, Any]:
    termination: dict[str, Any] = {
        "reason": reason,
        "process_group_id": process.pid,
        "term_sent": False,
        "kill_sent": False,
        "reaped": False,
        "returncode": None,
    }

    try:
        returncode = process.poll()
    except ChildProcessError:
        termination["reaped"] = True
        termination["reap_state"] = "already_reaped"
        termination["returncode"] = getattr(process, "returncode", None)
        return termination
    except Exception as exc:  # noqa: BLE001
        termination["poll_error"] = f"{type(exc).__name__}: {exc}"
        returncode = None

    if returncode is not None:
        try:
            returncode = process.wait(timeout=2.0)
        except ChildProcessError:
            termination["reap_state"] = "already_reaped"
            returncode = getattr(process, "returncode", returncode)
        except Exception as exc:  # noqa: BLE001
            termination["already_exited_wait_error"] = (
                f"{type(exc).__name__}: {exc}"
            )
            raise OwnedProcessLifecycleError(
                "owned process lifecycle cleanup failed while reaping an "
                "already-exited worker",
                termination,
            ) from exc
        termination["reaped"] = True
        termination["returncode"] = returncode
        return termination

    try:
        os.killpg(process.pid, signal.SIGTERM)
        termination["term_sent"] = True
    except ProcessLookupError:
        termination["term_process_group_missing"] = True
    except Exception as exc:  # noqa: BLE001
        termination["term_signal_error"] = f"{type(exc).__name__}: {exc}"

    try:
        returncode = process.wait(timeout=2.0)
    except ChildProcessError:
        termination["reaped"] = True
        termination["reap_state"] = "already_reaped_after_term"
        termination["returncode"] = getattr(process, "returncode", None)
        return termination
    except subprocess.TimeoutExpired:
        termination["term_wait_timed_out"] = True
    except Exception as exc:  # noqa: BLE001
        termination["term_wait_error"] = f"{type(exc).__name__}: {exc}"
    else:
        termination["reaped"] = True
        termination["returncode"] = returncode
        return termination

    try:
        os.killpg(process.pid, signal.SIGKILL)
        termination["kill_sent"] = True
    except ProcessLookupError:
        termination["kill_process_group_missing"] = True
    except Exception as exc:  # noqa: BLE001
        termination["kill_signal_error"] = f"{type(exc).__name__}: {exc}"

    try:
        returncode = process.wait(timeout=2.0)
    except ChildProcessError:
        termination["reaped"] = True
        termination["reap_state"] = "already_reaped_after_kill"
        termination["returncode"] = getattr(process, "returncode", None)
        return termination
    except subprocess.TimeoutExpired as exc:
        termination["kill_wait_timed_out"] = True
        raise OwnedProcessLifecycleError(
            "owned process lifecycle cleanup failed after SIGKILL",
            termination,
        ) from exc
    except Exception as exc:  # noqa: BLE001
        termination["kill_wait_error"] = f"{type(exc).__name__}: {exc}"
        raise OwnedProcessLifecycleError(
            "owned process lifecycle cleanup failed after SIGKILL",
            termination,
        ) from exc
    termination["reaped"] = True
    termination["returncode"] = returncode
    return termination


def _completed_process_termination(
    process: subprocess.Popen[bytes], *, reason: str
) -> dict[str, Any]:
    if process.returncode is None:
        return _owned_process_termination(process, reason=reason)
    return {
        "reason": reason,
        "process_group_id": process.pid,
        "term_sent": False,
        "kill_sent": False,
        "reaped": True,
        "returncode": process.returncode,
    }


def _preserve_lifecycle_failure(
    job_root: Path,
    job: dict[str, Any],
    evidence: dict[str, Any],
    error: OwnedProcessLifecycleError,
    *,
    trigger: str,
) -> None:
    evidence["termination"] = error.termination
    try:
        atomic_write_json(
            job_root / "lifecycle_failure.json",
            {
                "schema_version": 1,
                "asset_key": job["asset_key"],
                "run_token": job["run_token"],
                "process_group_id": error.termination.get(
                    "process_group_id"
                ),
                "error": str(error),
                "trigger": trigger,
                "worker_evidence": evidence,
            },
        )
    except Exception as exc:  # noqa: BLE001
        error.termination["diagnostic_write_error"] = (
            f"{type(exc).__name__}: {exc}"
        )


def _execute_job(
    job: dict[str, Any],
    scratch_root: Path,
    timeout_seconds: float,
    spawn_retry_budget: SpawnEagainRetryBudget | None = None,
    lifecycle_controller: FatalLifecycleController | None = None,
) -> dict[str, Any]:
    retry_budget = spawn_retry_budget or SpawnEagainRetryBudget()
    controller = lifecycle_controller or FatalLifecycleController(
        retry_budget.exhausted
    )
    controller.raise_if_fatal()
    try:
        current_binding = package_binding(Path(job["package"]))
        current_urdf = _regular_file(Path(job["urdf_path"]), "selected URDF")
        if current_binding != job["package_binding"] or sha256_file(
            current_urdf
        ) != job["urdf_sha256"]:
            controller.raise_if_fatal()
            return _failed_for_job(
                job,
                "parent preflight detected selected package or URDF drift",
                evidence=_parent_worker_evidence(job),
            )
    except Exception as exc:  # noqa: BLE001
        controller.raise_if_fatal()
        return _failed_for_job(
            job,
            f"parent preflight package binding failed: {type(exc).__name__}: {exc}",
            evidence=_parent_worker_evidence(job),
        )
    controller.raise_if_fatal()
    if scratch_root.is_symlink():
        raise RuntimeError(f"worker scratch is symlinked: {scratch_root}")
    scratch_root.mkdir(parents=True, exist_ok=True)
    if scratch_root.is_symlink() or not stat.S_ISDIR(os.lstat(scratch_root).st_mode):
        raise RuntimeError(f"worker scratch is not a safe directory: {scratch_root}")
    job_root = Path(tempfile.mkdtemp(prefix="job_", dir=scratch_root))
    job_path = job_root / "job.json"
    result_path = job_root / "result.json"
    atomic_write_json(job_path, job)
    environment = dict(os.environ)
    environment.update(CHILD_THREAD_ENVIRONMENT)
    preserve_job_root = False
    try:
        process = _popen_job_with_spawn_retry(
            [
                sys.executable,
                str(SCRIPT_PATH),
                "--internal-job",
                str(job_path),
                "--internal-result",
                str(result_path),
            ],
            environment=environment,
            retry_budget=retry_budget,
            lifecycle_controller=controller,
        )
        evidence = _parent_worker_evidence(job, process.pid)
        try:
            _stdout, stderr = process.communicate(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            try:
                evidence["termination"] = _owned_process_termination(
                    process, reason="asset_timeout"
                )
            except OwnedProcessLifecycleError as exc:
                preserve_job_root = True
                controller.mark_fatal(exc)
                _preserve_lifecycle_failure(
                    job_root,
                    job,
                    evidence,
                    exc,
                    trigger=f"TimeoutExpired after {timeout_seconds:g} seconds",
                )
                raise
            return _failed_for_job(
                job,
                f"asset timeout after {timeout_seconds:g} seconds",
                status_value="timeout",
                evidence=evidence,
            )
        except Exception as exc:  # noqa: BLE001
            try:
                evidence["termination"] = _owned_process_termination(
                    process, reason="post_popen_exception"
                )
            except OwnedProcessLifecycleError as lifecycle_exc:
                preserve_job_root = True
                controller.mark_fatal(lifecycle_exc)
                _preserve_lifecycle_failure(
                    job_root,
                    job,
                    evidence,
                    lifecycle_exc,
                    trigger=f"{type(exc).__name__}: {exc}",
                )
                raise lifecycle_exc from exc
            return _failed_for_job(
                job,
                f"post-Popen worker exception: {type(exc).__name__}: {exc}",
                evidence=evidence,
            )
        try:
            evidence["termination"] = _completed_process_termination(
                process, reason="communicate_completed"
            )
        except OwnedProcessLifecycleError as exc:
            preserve_job_root = True
            controller.mark_fatal(exc)
            _preserve_lifecycle_failure(
                job_root,
                job,
                evidence,
                exc,
                trigger="communicate returned without a reaped child",
            )
            raise
        if process.returncode != 0 or not result_path.is_file():
            try:
                detail = stderr.decode("utf-8", errors="replace")[-4000:]
            except Exception as exc:  # noqa: BLE001
                return _failed_for_job(
                    job,
                    f"post-Popen worker exception: {type(exc).__name__}: {exc}",
                    evidence=evidence,
                )
            return _failed_for_job(
                job,
                f"worker failed with exit {process.returncode}: {detail}",
                evidence=evidence,
            )
        try:
            record = json.loads(result_path.read_text(encoding="utf-8"))
            return validate_record(job, record, expected_child_pid=process.pid)
        except Exception as exc:  # noqa: BLE001
            return _failed_for_job(
                job,
                f"worker result validation failed: {type(exc).__name__}: {exc}",
                evidence=evidence,
            )
    finally:
        if not preserve_job_root:
            shutil.rmtree(job_root, ignore_errors=True)


def execute_jobs(
    jobs: list[dict[str, Any]],
    *,
    scratch_root: Path,
    timeout_seconds: float,
    max_workers: int,
) -> list[dict[str, Any]]:
    ordered: list[dict[str, Any] | None] = [None] * len(jobs)
    completion_order = 0
    retry_budget = SpawnEagainRetryBudget()
    lifecycle_controller = FatalLifecycleController(retry_budget.exhausted)
    exhaustion: SpawnEagainRetryBudgetExhausted | None = None
    lifecycle_failure: OwnedProcessLifecycleError | None = None
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(
                _execute_job,
                job,
                scratch_root,
                timeout_seconds,
                retry_budget,
                lifecycle_controller,
            ): (index, job)
            for index, job in enumerate(jobs)
        }
        for future in as_completed(futures):
            index, job = futures[future]
            try:
                ordered[index] = future.result()
            except SpawnEagainRetryBudgetExhausted as exc:
                if exhaustion is None:
                    exhaustion = exc
                continue
            except OwnedProcessLifecycleError as exc:
                if lifecycle_failure is None:
                    lifecycle_failure = exc
                for pending in futures:
                    if pending is not future:
                        pending.cancel()
                continue
            except (LifecycleRunAborted, CancelledError):
                continue
            except Exception as exc:  # noqa: BLE001
                ordered[index] = _failed_for_job(
                    job,
                    f"parent worker exception: {type(exc).__name__}: {exc}",
                    evidence=_parent_worker_evidence(job),
                )
            assert ordered[index] is not None
            completion_order += 1
            ordered[index]["completion_order"] = completion_order
    fatal = lifecycle_failure or lifecycle_controller.failure
    if fatal is not None:
        raise fatal
    if exhaustion is not None:
        raise exhaustion
    return [record for record in ordered if record is not None]


def _percent(rate: float | None) -> str:
    return "N/A" if rate is None else f"{100 * rate:.2f}%"


def _report(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    recovery_count = manifest["selection"]["recovery_count"]
    recovery_label = "recovery" if recovery_count == 1 else "recoveries"
    lines = [
        "# Infinite Mobility Table 3 Kinematic Executability",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            "This is a supplementary full generated cohort, not an official finite release. "
            f"It preserves {summary['n_eval']} frozen factory/seed identities, including "
            f"{recovery_count} {recovery_label}, in the exact Table 2 manifest order."
        ),
        "",
        f"N_eval={summary['n_eval']}; J_eval={summary['j_eval']}; K=21 states per joint.",
        "",
        (
            "Expected/observed joint denominator: "
            f"{summary['joint_denominator']['expected']} / "
            f"{summary['joint_denominator']['observed']}."
        ),
        "",
        (
            f"Zero-joint assets: {summary['zero_joint_assets']['count']} / "
            f"{summary['n_eval']} "
            f"({json.dumps(summary['zero_joint_assets']['factory_counts'], sort_keys=True)}); "
            "these assets fail Strict Kinematic Pass by protocol."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    display_names = {
        "valid_range": "Valid Range",
        "joint_sweep_success": "Joint Sweep Success",
        "non_degenerate_motion": "Non-degenerate Motion",
        "subtree_consistency": "Subtree Consistency",
        "joint_level_pass": "Joint-level Pass",
    }
    for metric in CORE.JOINT_RATE_METRICS:
        value = metrics[metric]
        lines.append(
            f"| {display_names[metric]} | {value['passed']} / "
            f"{value['denominator']} ({_percent(value['rate'])}) |"
        )
    roundtrip = metrics["fk_roundtrip_error"]
    lines.append(
        "| FK Round-trip Error | "
        f"translation={roundtrip['max_normalized_translation']}; "
        f"rotation_rad={roundtrip['max_rotation_rad']}; "
        f"coverage={roundtrip['measured_joint_count']} / {roundtrip['denominator']} "
        f"({roundtrip['status']}) |"
    )
    strict = metrics["strict_kinematic_pass"]
    lines.append(
        f"| Strict Kinematic Pass | {strict['passed']} / {strict['denominator']} "
        f"({_percent(strict['rate'])}) |"
    )
    macro = summary["category_macro"]
    lines.extend(
        [
            "",
            (
                "Category macro over "
                f"{macro['category_count']} factories "
                f"({macro['joint_metric_category_count']} with movable joints):"
            ),
            "",
            "| Metric | Category macro |",
            "|---|---:|",
        ]
    )
    for metric in (*CORE.JOINT_RATE_METRICS, "strict_kinematic_pass"):
        value = macro["metrics"][metric]
        label = display_names.get(metric, "Strict Kinematic Pass")
        lines.append(
            f"| {label} | {_percent(value['rate'])} "
            f"(categories={value['category_count']}) |"
        )
    lines.extend(
        [
            "",
            f"Worker status counts: `{json.dumps(summary['status_counts'], sort_keys=True)}`.",
            "",
            (
                f"Parse/tree: {summary['parse_success']} / {summary['n_eval']} parsed; "
                f"{summary['valid_tree']} / {summary['n_eval']} valid trees."
            ),
            "",
            (
                "Provenance: "
                f"{summary['source_provenance']['original_pass']} original PASS and "
                f"{summary['source_provenance']['recovery_overlay']} recovery overlays."
            ),
            "",
            (
                "Attestation: package before/after "
                f"{summary['attestation']['package_before']} / "
                f"{summary['attestation']['package_after']}; runtime matches "
                f"{summary['attestation']['runtime_match']} / {summary['n_eval']}."
            ),
            "",
            "Hash evidence:",
            "",
            f"- manifest content: `{manifest['manifest_content_sha256']}`",
            f"- adapter: `{manifest['evaluation']['adapter_sha256']}`",
            f"- core evaluator: `{manifest['evaluation']['core_evaluator_sha256']}`",
            f"- protocol: `{manifest['evaluation']['protocol_sha256']}`",
            f"- environment: `{manifest['evaluation']['environment_sha256']}`",
            f"- cohort manifest: `{manifest['source']['cohort_manifest_file_sha256']}`",
            f"- Table 2 manifest: `{manifest['source']['table2_manifest_file_sha256']}`",
            "",
            (
                "Artifact closure: `manifest.json`, `asset_records.jsonl`, `summary.json`, "
                "`report.md`, `environment.json`, `protocol_snapshot.md`, "
                "`checkpoint.json`, and `artifact_manifest.json`."
            ),
            "",
            "All timeout, exception, malformed-result, and package-drift cases remain in the asset and declared-joint denominators.",
            "",
            "This discrete FK evaluation does not establish semantic joint correctness, collision safety, dynamics, or real-world fidelity.",
            "",
        ]
    )
    return "\n".join(lines)


@contextmanager
def output_lock(output: Path) -> Iterator[None]:
    output = output.absolute()
    output.parent.mkdir(parents=True, exist_ok=True)
    lock_path = output.parent / f".{output.name}.table3.lock"
    descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"output is locked by another process: {output}") from exc
        yield
    finally:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)


def _output_path(args: argparse.Namespace, n_eval: int) -> Path:
    if args.output is None:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        args.output = DEFAULT_OUTPUT_PARENT / f"table3_infinite_mobility_n{n_eval}_{timestamp}"
    return args.output.absolute()


def _protocol_bytes(path: Path) -> bytes:
    return _regular_file(path, "Table 3 protocol").read_bytes()


def _write_artifact_manifest(output: Path) -> None:
    names = tuple(sorted(OUTPUT_ARTIFACT_FILES))
    atomic_write_json(
        output / "artifact_manifest.json",
        {
            "schema_version": 1,
            "files": {
                name: {
                    "bytes": (output / name).stat().st_size,
                    "sha256": sha256_file(output / name),
                }
                for name in names
            },
        },
    )


def _records_bytes(records: list[dict[str, Any]]) -> bytes:
    return b"".join(canonical_json_bytes(row) + b"\n" for row in records)


def write_records(path: Path, records: list[dict[str, Any]]) -> None:
    _atomic_bytes(path, _records_bytes(records))


def _validate_complete_completion_order(
    *,
    state: Any,
    n_eval: int,
    record_orders: list[Any],
    completion_order: Any,
) -> None:
    if state != "complete":
        return
    if (
        type(completion_order) is not int
        or completion_order != n_eval
        or len(record_orders) != n_eval
        or sorted(record_orders) != list(range(1, n_eval + 1))
    ):
        raise ValueError(
            "complete checkpoint completion order must be exactly 1..N"
        )


def write_checkpoint(
    output: Path,
    manifest: dict[str, Any],
    records: list[dict[str, Any]],
    *,
    state: str,
    completion_order: int,
) -> None:
    if state not in {"frozen", "running", "aggregating", "complete"}:
        raise ValueError(f"invalid checkpoint state: {state}")
    record_orders = [record.get("completion_order") for record in records]
    if any(type(order) is not int or order <= 0 for order in record_orders):
        if records:
            raise ValueError("checkpoint records lack valid completion orders")
    _validate_complete_completion_order(
        state=state,
        n_eval=len(manifest["records"]),
        record_orders=record_orders,
        completion_order=completion_order,
    )
    if completion_order < max(record_orders, default=0):
        raise ValueError("checkpoint completion order precedes a retained record")
    records_path = output / "asset_records.jsonl"
    write_records(records_path, records)
    payload = _regular_file(records_path, "Table 3 checkpoint records").read_bytes()
    manifest_path = _regular_file(output / "manifest.json", "Table 3 manifest")
    atomic_write_json(
        output / "checkpoint.json",
        {
            "schema_version": 1,
            "state": state,
            "completed": len(records),
            "remaining": len(manifest["records"]) - len(records),
            "n_eval": len(manifest["records"]),
            "completion_order": completion_order,
            "completed_asset_ids": [record["asset_id"] for record in records],
            "completed_asset_keys": [record["asset_key"] for record in records],
            "completed_run_tokens": [record["run_token"] for record in records],
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "manifest_file_sha256": sha256_file(manifest_path),
            "records_bytes": len(payload),
            "records_sha256": hashlib.sha256(payload).hexdigest(),
            "updated_at": utc_now(),
        },
    )


def _job_for_manifest_row(
    row: dict[str, Any], manifest: dict[str, Any], manifest_path: Path
) -> dict[str, Any]:
    job = make_job(
        row,
        manifest_content_sha256=manifest["manifest_content_sha256"],
        samples=manifest["evaluation"]["config"]["samples_per_joint"],
        mode="formal" if manifest["classification"] == "FORMAL" else "smoke",
    )
    return attach_runtime_binding(job, manifest, manifest_path)


def _resume_package_matches(row: dict[str, Any]) -> bool:
    try:
        current = package_binding(Path(row["package"]))
        urdf = _regular_file(Path(row["urdf_path"]), "resume selected URDF")
        return current == row["package_binding"] and sha256_file(urdf) == row["urdf_sha256"]
    except Exception:  # noqa: BLE001
        return False


def load_checkpoint_records(
    output: Path, manifest: dict[str, Any]
) -> tuple[dict[str, dict[str, Any]], int, bool]:
    checkpoint = _read_json(output / "checkpoint.json", "resume checkpoint")
    manifest_path = _regular_file(output / "manifest.json", "resume manifest")
    if checkpoint.get("schema_version") != 1:
        raise ValueError("resume checkpoint schema mismatch")
    if (
        checkpoint.get("manifest_content_sha256") != manifest["manifest_content_sha256"]
        or checkpoint.get("manifest_file_sha256") != sha256_file(manifest_path)
        or checkpoint.get("n_eval") != len(manifest["records"])
    ):
        raise ValueError("resume checkpoint manifest binding mismatch")
    records_path = _regular_file(
        output / "asset_records.jsonl", "resume checkpoint records"
    )
    payload = records_path.read_bytes()
    records = load_jsonl(records_path)
    completed = checkpoint.get("completed")
    if type(completed) is not int or not 0 <= completed <= len(manifest["records"]):
        raise ValueError("resume checkpoint completed count is invalid")
    exact_receipt = (
        checkpoint.get("records_bytes") == len(payload)
        and checkpoint.get("records_sha256") == hashlib.sha256(payload).hexdigest()
        and len(records) == completed
    )
    recovered_append_window = False
    if not exact_receipt:
        if len(records) != completed + 1 or checkpoint.get("state") == "complete":
            raise ValueError("resume checkpoint records receipt mismatch")
        current_by_id = {row.get("asset_id"): row for row in records}
        completed_ids = checkpoint.get("completed_asset_ids")
        if (
            not isinstance(completed_ids, list)
            or len(completed_ids) != completed
            or len(current_by_id) != len(records)
            or any(asset_id not in current_by_id for asset_id in completed_ids)
        ):
            raise ValueError("resume append-window identity proof mismatch")
        prior_records = [current_by_id[asset_id] for asset_id in completed_ids]
        prefix_payload = _records_bytes(prior_records)
        if (
            checkpoint.get("records_bytes") != len(prefix_payload)
            or checkpoint.get("records_sha256")
            != hashlib.sha256(prefix_payload).hexdigest()
        ):
            raise ValueError("resume checkpoint records prefix receipt mismatch")
        recovered_append_window = True
    if checkpoint.get("remaining") != len(manifest["records"]) - completed:
        raise ValueError("resume checkpoint counts are inconsistent")
    receipt_records = records if exact_receipt else prior_records
    if checkpoint.get("completed_asset_ids") != [
        row.get("asset_id") for row in receipt_records
    ] or checkpoint.get("completed_asset_keys") != [
        row.get("asset_key") for row in receipt_records
    ] or checkpoint.get("completed_run_tokens") != [
        row.get("run_token") for row in receipt_records
    ]:
        raise ValueError("resume checkpoint completed identity receipt mismatch")

    manifest_by_key = {row["asset_key"]: row for row in manifest["records"]}
    manifest_order = [row["asset_key"] for row in manifest["records"]]
    by_key: dict[str, dict[str, Any]] = {}
    completion_orders: set[int] = set()
    tokens: set[str] = set()
    last_manifest_position = -1
    for record in records:
        key = record.get("asset_key")
        if key not in manifest_by_key or key in by_key:
            raise ValueError(f"invalid or duplicate resume record: {key!r}")
        position = manifest_order.index(str(key))
        if position <= last_manifest_position:
            raise ValueError("resume records are not in frozen manifest order")
        last_manifest_position = position
        job = _job_for_manifest_row(manifest_by_key[str(key)], manifest, manifest_path)
        validate_record(job, record, require_completion_order=True)
        order = record["completion_order"]
        token = record["run_token"]
        if order in completion_orders or token in tokens:
            raise ValueError("resume record completion order or run token is duplicated")
        completion_orders.add(order)
        tokens.add(token)
        by_key[str(key)] = record
    checkpoint_order = checkpoint.get("completion_order")
    prefix_orders = [record["completion_order"] for record in receipt_records]
    if (
        type(checkpoint_order) is not int
        or checkpoint_order < max(prefix_orders, default=0)
    ):
        raise ValueError("resume checkpoint completion order mismatch")
    _validate_complete_completion_order(
        state=checkpoint.get("state"),
        n_eval=len(manifest["records"]),
        record_orders=[record["completion_order"] for record in records],
        completion_order=checkpoint_order,
    )
    if recovered_append_window:
        completed_ids = set(checkpoint["completed_asset_ids"])
        extra_records = [
            record for record in records if record["asset_id"] not in completed_ids
        ]
        if (
            len(extra_records) != 1
            or extra_records[0]["completion_order"] != checkpoint_order + 1
        ):
            raise ValueError("resume append-window completion order is not contiguous")

    drifted = [key for key in by_key if not _resume_package_matches(manifest_by_key[key])]
    for key in drifted:
        del by_key[key]
    if drifted:
        retained_by_completion = sorted(
            by_key.values(), key=lambda record: record["completion_order"]
        )
        for normalized_order, record in enumerate(retained_by_completion, 1):
            record["completion_order"] = normalized_order
        next_completion_order = len(retained_by_completion)
    else:
        next_completion_order = max(
            checkpoint_order,
            max((record["completion_order"] for record in records), default=0),
        )
    return (
        by_key,
        next_completion_order,
        recovered_append_window or bool(drifted),
    )


_resume_records = load_checkpoint_records


def _find_lifecycle_failure_markers(scratch: Path) -> list[Path]:
    markers: list[Path] = []
    pending = [scratch]
    while pending:
        current = pending.pop()
        with os.scandir(current) as entries:
            for entry in sorted(entries, key=lambda item: item.name):
                entry_path = Path(entry.path)
                mode = entry.stat(follow_symlinks=False).st_mode
                if stat.S_ISLNK(mode):
                    raise RuntimeError(
                        f"worker scratch contains a symlink: {entry_path}"
                    )
                if stat.S_ISDIR(mode):
                    pending.append(entry_path)
                elif stat.S_ISREG(mode):
                    if entry.name == "lifecycle_failure.json":
                        markers.append(entry_path)
                else:
                    raise RuntimeError(
                        f"worker scratch contains an unsafe entry: {entry_path}"
                    )
    return sorted(markers)


def _lifecycle_marker_process_groups(
    markers: list[Path], jobs: list[dict[str, Any]]
) -> list[int]:
    expected = {job["asset_key"]: job for job in jobs}
    if len(expected) != len(jobs):
        raise RuntimeError("pending worker jobs contain duplicate asset identities")
    observed_assets: set[str] = set()
    observed_process_groups: set[int] = set()
    process_groups: list[int] = []
    for marker_path in markers:
        marker = _read_json(marker_path, "worker lifecycle failure marker")
        asset_key = marker.get("asset_key")
        run_token = marker.get("run_token")
        process_group_id = marker.get("process_group_id")
        evidence = marker.get("worker_evidence")
        termination = evidence.get("termination") if isinstance(evidence, dict) else None
        if (
            marker.get("schema_version") != 1
            or not isinstance(asset_key, str)
            or asset_key not in expected
            or not isinstance(run_token, str)
            or run_token != expected[asset_key].get("run_token")
            or type(process_group_id) is not int
            or process_group_id <= 0
            or not isinstance(termination, dict)
            or termination.get("process_group_id") != process_group_id
            or evidence.get("child_pid") != process_group_id
            or termination.get("reaped") is not False
        ):
            raise RuntimeError(
                f"worker lifecycle failure marker binding is invalid: {marker_path}"
            )
        job_receipt = _read_json(
            marker_path.parent / "job.json", "preserved worker job receipt"
        )
        if (
            job_receipt.get("asset_key") != asset_key
            or job_receipt.get("run_token") != run_token
            or asset_key in observed_assets
            or process_group_id in observed_process_groups
        ):
            raise RuntimeError(
                f"worker lifecycle failure marker ownership is invalid: {marker_path}"
            )
        observed_assets.add(asset_key)
        observed_process_groups.add(process_group_id)
        process_groups.append(process_group_id)
    return process_groups


def _require_prior_process_groups_gone(process_groups: list[int]) -> None:
    for process_group_id in process_groups:
        try:
            os.killpg(process_group_id, 0)
        except ProcessLookupError:
            continue
        except OSError as exc:
            if exc.errno == errno.ESRCH:
                continue
            raise RuntimeError(
                "cannot prove prior worker process group is gone: "
                f"pgid={process_group_id}: {type(exc).__name__}: {exc}"
            ) from exc
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "cannot prove prior worker process group is gone: "
                f"pgid={process_group_id}: {type(exc).__name__}: {exc}"
            ) from exc
        raise RuntimeError(
            "prior worker process group still exists; refusing resume without "
            f"signalling it: pgid={process_group_id}"
        )


def _prepare_worker_scratch(
    output: Path, jobs: list[dict[str, Any]]
) -> Path:
    scratch = output / ".worker_scratch"
    if scratch.is_symlink():
        raise RuntimeError(f"worker scratch is a symlink: {scratch}")
    if scratch.exists():
        if not stat.S_ISDIR(os.lstat(scratch).st_mode):
            raise RuntimeError(f"worker scratch is not a directory: {scratch}")
        markers = _find_lifecycle_failure_markers(scratch)
        if markers:
            process_groups = _lifecycle_marker_process_groups(markers, jobs)
            _require_prior_process_groups_gone(process_groups)
        if scratch.is_symlink() or not stat.S_ISDIR(os.lstat(scratch).st_mode):
            raise RuntimeError("worker scratch changed during recovery preflight")
        quarantine = output.parent / (
            f".{output.name}.worker_scratch.quarantine.{uuid.uuid4().hex}"
        )
        os.replace(scratch, quarantine)
    scratch.mkdir(mode=0o700, exist_ok=False)
    return scratch


def _validate_seal_only_terminal_state(
    output: Path,
    manifest: dict[str, Any],
    records: list[dict[str, Any]],
    *,
    mode: str,
) -> None:
    observed_names = {entry.name for entry in output.iterdir()}
    if observed_names != set(OUTPUT_ARTIFACT_FILES):
        raise ValueError(
            "seal-only validation artifact set mismatch: "
            f"observed={sorted(observed_names)}, "
            f"expected={sorted(OUTPUT_ARTIFACT_FILES)}"
        )
    for name in OUTPUT_ARTIFACT_FILES:
        _regular_file(output / name, f"seal-only validation artifact {name}")

    recomputed = CORE.aggregate_records(records, expected_n=len(manifest["records"]))
    summary = _read_json(output / "summary.json", "seal-only validation summary")
    for field in (
        "n_eval",
        "j_eval",
        "metrics",
        "status_counts",
        "parse_success",
        "valid_tree",
        "category_macro",
    ):
        if summary.get(field) != recomputed[field]:
            raise ValueError(f"seal-only validation summary mismatch: {field}")
    expected_joints = sum(
        row["declared_joint_count_hint"] for row in manifest["records"]
    )
    zero_rows = [
        row for row in manifest["records"] if row["declared_joint_count_hint"] == 0
    ]
    zero_factory_counts = dict(
        sorted(Counter(row["raw_category"] for row in zero_rows).items())
    )
    expected_fixed = {
        "schema_version": 1,
        "status": "completed",
        "mode": mode,
        "classification": manifest["classification"],
        "dataset": DATASET_NAME,
        "cohort_type": manifest["cohort_type"],
        "recovery_count": manifest["selection"]["recovery_count"],
        "records_present": len(records),
        "records_missing_counted_as_failures": 0,
        "joint_denominator": {
            "expected": expected_joints,
            "observed": recomputed["j_eval"],
            "match": expected_joints == recomputed["j_eval"],
        },
        "zero_joint_assets": {
            "count": len(zero_rows),
            "factory_counts": zero_factory_counts,
            "asset_ids_sha256": canonical_sha256(
                [row["asset_id"] for row in zero_rows]
            ),
            "strict_fail_count": sum(
                not record["strict_kinematic_pass"]
                for record, source in zip(records, manifest["records"])
                if source["declared_joint_count_hint"] == 0
            ),
        },
        "source_provenance": {
            "original_pass": sum(
                row["original_status"] == "PASS" and row["recovery_used"] is False
                for row in manifest["records"]
            ),
            "recovery_overlay": sum(
                row["original_status"] == "TIMEOUT" and row["recovery_used"] is True
                for row in manifest["records"]
            ),
        },
        "attestation": {
            "package_before": sum(
                record["worker_evidence"].get("package_binding_before") is True
                for record in records
            ),
            "package_after": sum(
                record["worker_evidence"].get("package_binding_after") is True
                for record in records
            ),
            "runtime_match": sum(
                record["worker_evidence"].get("runtime_binding_match") is True
                for record in records
            ),
            "child_attested": sum(
                record["result_origin"] == "child_attested" for record in records
            ),
        },
        "manifest_content_sha256": manifest["manifest_content_sha256"],
    }
    for field, expected in expected_fixed.items():
        if summary.get(field) != expected:
            raise ValueError(f"seal-only validation summary mismatch: {field}")
    completed_at = summary.get("completed_at")
    if not isinstance(completed_at, str) or re.fullmatch(
        r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z", completed_at
    ) is None:
        raise ValueError("seal-only validation summary completion timestamp is invalid")
    report_path = _regular_file(output / "report.md", "seal-only validation report")
    if report_path.read_text(encoding="utf-8") != _report(summary, manifest):
        raise ValueError("seal-only validation report mismatch")


def run(args: argparse.Namespace) -> Path:
    validate_contract(args)
    formal = args.mode == "formal"
    loaded = load_inputs(args.cohort_manifest, args.table2_manifest, formal=formal)
    selected_count = len(loaded["assets"]) if args.limit is None else args.limit
    output = _output_path(args, selected_count)
    protocol_bytes = _protocol_bytes(args.protocol_path)
    environment = _environment()
    with output_lock(output):
        if args.resume:
            if output.is_symlink() or not output.is_dir():
                raise FileNotFoundError(f"resume output does not exist: {output}")
        else:
            if output.exists() or output.is_symlink():
                raise RuntimeError(f"refusing to overwrite existing output: {output}")
            output.mkdir(parents=False, exist_ok=False)

        manifest_path = output / "manifest.json"
        checkpoint_path = output / "checkpoint.json"
        seal_only = False
        if args.resume:
            manifest = _read_json(manifest_path, "resume manifest")
            if manifest.get("manifest_content_sha256") != manifest_self_hash(manifest):
                raise ValueError("resume manifest self-hash mismatch")
            resume_checkpoint = _read_json(checkpoint_path, "resume checkpoint")
            if (output / "artifact_manifest.json").exists() or (
                output / "artifact_manifest.json"
            ).is_symlink():
                raise RuntimeError(f"output is already complete and immutable: {output}")
            seal_only = resume_checkpoint.get("state") == "complete"
            if not seal_only and resume_checkpoint.get("state") not in {
                "frozen",
                "running",
                "aggregating",
            }:
                raise ValueError("resume checkpoint state is invalid")
            if (
                resume_checkpoint.get("manifest_content_sha256")
                != manifest["manifest_content_sha256"]
                or resume_checkpoint.get("n_eval") != selected_count
            ):
                raise ValueError("resume checkpoint binding mismatch")
            fresh = build_manifest(args, loaded, protocol_bytes, environment)
            for field in ("source", "selection", "evaluation", "records"):
                if fresh[field] != manifest.get(field):
                    raise ValueError(f"resume manifest binding mismatch: {field}")
            if _regular_file(
                output / "protocol_snapshot.md", "resume protocol snapshot"
            ).read_bytes() != protocol_bytes:
                raise ValueError("resume protocol snapshot mismatch")
            stored_environment = _read_json(output / "environment.json", "resume environment")
            if stored_environment != environment:
                raise ValueError("resume environment mismatch")
        else:
            manifest = build_manifest(args, loaded, protocol_bytes, environment)
            atomic_write_json(manifest_path, manifest)
            _atomic_bytes(output / "protocol_snapshot.md", protocol_bytes)
            atomic_write_json(output / "environment.json", environment)
            write_checkpoint(
                output, manifest, [], state="frozen", completion_order=0
            )

        verify_live_runtime_binding(manifest, manifest_path)
        if args.resume:
            by_key, completion_order, recovered = load_checkpoint_records(output, manifest)
            if seal_only:
                selected_keys = [row["asset_key"] for row in manifest["records"]]
                if recovered or set(by_key) != set(selected_keys):
                    raise ValueError(
                        "seal-only validation failed package/URDF or record binding recheck"
                    )
                ordered = [by_key[key] for key in selected_keys]
                verify_live_runtime_binding(manifest, manifest_path)
                _validate_seal_only_terminal_state(
                    output, manifest, ordered, mode=args.mode
                )
                verify_live_runtime_binding(manifest, manifest_path)
                _write_artifact_manifest(output)
                verify_artifact_manifest(
                    output, expected_files=OUTPUT_ARTIFACT_FILES, exact=True
                )
                return output.resolve(strict=True)
            if recovered:
                recovered_rows = [
                    by_key[row["asset_key"]]
                    for row in manifest["records"]
                    if row["asset_key"] in by_key
                ]
                write_checkpoint(
                    output,
                    manifest,
                    recovered_rows,
                    state="running",
                    completion_order=completion_order,
                )
        else:
            by_key = {}
            completion_order = 0
        selected_keys = [row["asset_key"] for row in manifest["records"]]
        pending_rows = [row for row in manifest["records"] if row["asset_key"] not in by_key]
        jobs = [
            _job_for_manifest_row(row, manifest, manifest_path)
            for row in pending_rows
        ]
        scratch: Path | None = None
        if jobs:
            ordered_present = [
                by_key[item] for item in selected_keys if item in by_key
            ]
            write_checkpoint(
                output,
                manifest,
                ordered_present,
                state="running",
                completion_order=completion_order,
            )
            retry_budget = SpawnEagainRetryBudget()
            lifecycle_controller = FatalLifecycleController(
                retry_budget.exhausted
            )
            scratch = _prepare_worker_scratch(output, jobs)
            exhaustion: SpawnEagainRetryBudgetExhausted | None = None
            lifecycle_failure: OwnedProcessLifecycleError | None = None
            try:
                with ThreadPoolExecutor(max_workers=args.workers) as executor:
                    futures = {
                        executor.submit(
                            _execute_job,
                            job,
                            scratch,
                            args.asset_timeout_seconds,
                            retry_budget,
                            lifecycle_controller,
                        ): job
                        for job in jobs
                    }
                    for future in as_completed(futures):
                        verify_live_runtime_binding(manifest, manifest_path)
                        job = futures[future]
                        try:
                            record = future.result()
                        except SpawnEagainRetryBudgetExhausted as exc:
                            if exhaustion is None:
                                exhaustion = exc
                            continue
                        except OwnedProcessLifecycleError as exc:
                            if lifecycle_failure is None:
                                lifecycle_failure = exc
                            for pending in futures:
                                if pending is not future:
                                    pending.cancel()
                            continue
                        except (LifecycleRunAborted, CancelledError):
                            continue
                        except Exception as exc:  # noqa: BLE001
                            record = _failed_for_job(
                                job,
                                f"parent worker exception: {type(exc).__name__}: {exc}",
                                evidence=_parent_worker_evidence(job),
                            )
                        validate_record(job, record)
                        key = record["asset_key"]
                        if key in by_key:
                            raise RuntimeError(f"worker returned duplicate asset: {key}")
                        completion_order += 1
                        record["completion_order"] = completion_order
                        validate_record(job, record, require_completion_order=True)
                        by_key[key] = record
                        ordered_present = [
                            by_key[item] for item in selected_keys if item in by_key
                        ]
                        write_checkpoint(
                            output,
                            manifest,
                            ordered_present,
                            state=(
                                "running"
                                if len(ordered_present) < selected_count
                                else "aggregating"
                            ),
                            completion_order=completion_order,
                        )
                        verify_live_runtime_binding(manifest, manifest_path)
                        print(
                            f"[{len(ordered_present)}/{selected_count}] {key} "
                            f"{record['status']}",
                            flush=True,
                        )
            finally:
                if (
                    scratch.is_symlink()
                    or not scratch.exists()
                    or not stat.S_ISDIR(os.lstat(scratch).st_mode)
                ):
                    raise RuntimeError("worker scratch changed before cleanup")
                if not lifecycle_controller.is_fatal:
                    shutil.rmtree(scratch)
            fatal = lifecycle_failure or lifecycle_controller.failure
            if fatal is not None:
                raise fatal
            if exhaustion is not None:
                raise exhaustion

        verify_live_runtime_binding(manifest, manifest_path)
        if set(by_key) != set(selected_keys):
            raise RuntimeError("evaluation ended without every frozen asset record")
        ordered = [by_key[key] for key in selected_keys]
        for row, record in zip(manifest["records"], ordered):
            validate_record(
                _job_for_manifest_row(row, manifest, manifest_path),
                record,
                require_completion_order=True,
            )
        summary = CORE.aggregate_records(ordered, expected_n=selected_count)
        expected_joints = sum(
            row["declared_joint_count_hint"] for row in manifest["records"]
        )
        zero_joint_records = [
            row for row in manifest["records"] if row["declared_joint_count_hint"] == 0
        ]
        zero_factory_counts = dict(
            sorted(Counter(row["raw_category"] for row in zero_joint_records).items())
        )
        if summary["j_eval"] != expected_joints:
            raise RuntimeError("final observed joint denominator differs from frozen hints")
        if formal and (
            selected_count != FORMAL_N
            or expected_joints != FORMAL_JOINT_COUNT
            or summary["j_eval"] != FORMAL_JOINT_COUNT
            or len(zero_joint_records) != FORMAL_ZERO_JOINT_COUNT
        ):
            raise RuntimeError("formal Table 3 terminal denominator contract failed")
        if any(
            record["strict_kinematic_pass"] is not False
            for record, source in zip(ordered, manifest["records"])
            if source["declared_joint_count_hint"] == 0
        ):
            raise RuntimeError("zero-joint asset violated fail-closed strict semantics")
        evidence_rows = [record["worker_evidence"] for record in ordered]
        summary.update(
            {
                "schema_version": 1,
                "status": "completed",
                "mode": args.mode,
                "classification": manifest["classification"],
                "dataset": DATASET_NAME,
                "cohort_type": manifest["cohort_type"],
                "recovery_count": manifest["selection"]["recovery_count"],
                "records_present": len(ordered),
                "records_missing_counted_as_failures": selected_count - len(ordered),
                "joint_denominator": {
                    "expected": expected_joints,
                    "observed": summary["j_eval"],
                    "match": expected_joints == summary["j_eval"],
                },
                "zero_joint_assets": {
                    "count": len(zero_joint_records),
                    "factory_counts": zero_factory_counts,
                    "asset_ids_sha256": canonical_sha256(
                        [row["asset_id"] for row in zero_joint_records]
                    ),
                    "strict_fail_count": sum(
                        not record["strict_kinematic_pass"]
                        for record, source in zip(ordered, manifest["records"])
                        if source["declared_joint_count_hint"] == 0
                    ),
                },
                "source_provenance": {
                    "original_pass": sum(
                        row["original_status"] == "PASS"
                        and row["recovery_used"] is False
                        for row in manifest["records"]
                    ),
                    "recovery_overlay": sum(
                        row["original_status"] == "TIMEOUT"
                        and row["recovery_used"] is True
                        for row in manifest["records"]
                    ),
                },
                "attestation": {
                    "package_before": sum(
                        evidence.get("package_binding_before") is True
                        for evidence in evidence_rows
                    ),
                    "package_after": sum(
                        evidence.get("package_binding_after") is True
                        for evidence in evidence_rows
                    ),
                    "runtime_match": sum(
                        evidence.get("runtime_binding_match") is True
                        for evidence in evidence_rows
                    ),
                    "child_attested": sum(
                        record["result_origin"] == "child_attested"
                        for record in ordered
                    ),
                },
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "completed_at": utc_now(),
            }
        )
        atomic_write_json(output / "summary.json", summary)
        atomic_write_text(output / "report.md", _report(summary, manifest))
        verify_live_runtime_binding(manifest, manifest_path)
        write_checkpoint(
            output,
            manifest,
            ordered,
            state="complete",
            completion_order=completion_order,
        )
        _write_artifact_manifest(output)
        verify_artifact_manifest(
            output, expected_files=OUTPUT_ARTIFACT_FILES, exact=True
        )
    return output.resolve(strict=True)


def validate_contract(
    args: argparse.Namespace,
    *,
    python_executable: Path | None = None,
    python_prefix: Path | None = None,
    runtime_fingerprint: dict[str, Any] | None = None,
) -> None:
    if args.samples < 2:
        raise ValueError("samples must be at least 2")
    if args.workers <= 0 or args.asset_timeout_seconds <= 0:
        raise ValueError("workers and asset timeout must be positive")
    if args.limit is not None and args.limit <= 0:
        raise ValueError("limit must be positive")
    if args.mode == "formal":
        if args.samples != DEFAULT_SAMPLES:
            raise ValueError("formal Table 3 freezes samples=21")
        if args.limit is not None:
            raise ValueError("formal Table 3 requires N=720 and forbids --limit")
        if args.workers != DEFAULT_WORKERS:
            raise ValueError("formal Table 3 freezes workers=4")
        if float(args.asset_timeout_seconds) != DEFAULT_TIMEOUT_SECONDS:
            raise ValueError("formal Table 3 freezes timeout=120 seconds")
        if args.cohort_manifest.absolute() != DEFAULT_COHORT_MANIFEST.absolute():
            raise ValueError("formal Table 3 requires the canonical cohort manifest")
        if args.table2_manifest.absolute() != DEFAULT_TABLE2_MANIFEST.absolute():
            raise ValueError("formal Table 3 requires the canonical Table 2 manifest")
        if args.protocol_path.absolute() != PROTOCOL_PATH.absolute():
            raise ValueError("formal Table 3 freezes the canonical protocol path")
        observed_executable = Path(
            sys.executable if python_executable is None else python_executable
        ).absolute()
        observed_prefix = Path(sys.prefix if python_prefix is None else python_prefix).absolute()
        if (
            observed_executable != FORMAL_PYTHON.absolute()
            or observed_prefix != FORMAL_PYTHON.parent.parent.absolute()
        ):
            raise ValueError(
                f"formal Table 3 must run with the low_medium Python environment: {FORMAL_PYTHON}"
            )
        observed_runtime = (
            formal_runtime_fingerprint()
            if runtime_fingerprint is None
            else runtime_fingerprint
        )
        if observed_runtime != FORMAL_RUNTIME_FINGERPRINT:
            raise ValueError(
                "formal Table 3 low_medium runtime fingerprint mismatch: "
                f"observed={observed_runtime!r}"
            )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), default="formal")
    parser.add_argument("--cohort-manifest", type=Path, default=DEFAULT_COHORT_MANIFEST)
    parser.add_argument("--table2-manifest", type=Path, default=DEFAULT_TABLE2_MANIFEST)
    parser.add_argument("--protocol-path", type=Path, default=PROTOCOL_PATH)
    parser.add_argument("--limit", type=int, help="smoke mode: exact Table 2 prefix")
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument(
        "--asset-timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--internal-job", type=Path)
    parser.add_argument("--internal-result", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.internal_job is not None or args.internal_result is not None:
        if args.internal_job is None or args.internal_result is None:
            raise ValueError("internal job mode requires both paths")
        return run_internal_job(args.internal_job, args.internal_result)
    output = run(args)
    print(json.dumps({"status": "completed", "output": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
