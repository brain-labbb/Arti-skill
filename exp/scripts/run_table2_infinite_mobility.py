#!/usr/bin/env python3
"""Audit the frozen supplementary Infinite Mobility cohort for Table 2."""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import contextmanager
import ctypes
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys
import time
from typing import Any
import uuid


# This bootstrap must run before importing the shared core, which imports NumPy.
FROZEN_THREAD_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}
for _thread_variable, _thread_value in FROZEN_THREAD_ENVIRONMENT.items():
    os.environ[_thread_variable] = _thread_value

import infinite_mobility_table123_common as common  # noqa: E402
import run_table2_urdf_articraft as core  # noqa: E402


if core.CHILD_THREAD_ENVIRONMENT != FROZEN_THREAD_ENVIRONMENT:
    raise RuntimeError("shared Table 2 core thread environment contract drift")


SCRIPT_PATH = Path(__file__).absolute()
SHARED_CORE_PATH = Path(core.SCRIPT_PATH)
_CORE_ENVIRONMENT_METADATA = core.environment_metadata
REPO_ROOT = SCRIPT_PATH.parents[2]
DEFAULT_COHORT_MANIFEST = common.DEFAULT_OUTPUT / "manifest.json"
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/table2_infinite_mobility_720"
FORMAL_PYTHON = REPO_ROOT / "arti-template/.venv/bin/python"
FORMAL_WORKERS = 8
FORMAL_TIMEOUT_SECONDS = 300.0
CORE_DATASET_PROFILE = "Artiverse"
SCHEMA_VERSION = "1.0.0"
_MISSING_ENV = object()
FORMAL_COHORT_MANIFEST_SHA256 = (
    "cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08"
)
FORMAL_COHORT_CONTENT_SHA256 = (
    "f5e29f1becd47cae991f5d238dff3f86b2b009365738df3e46cdbea297032c23"
)
FORMAL_COHORT_ARTIFACT_MANIFEST_SHA256 = (
    "ac31de70d50ed7153178482bb5283659be94fb5945cc2b7157754ac61dfc5439"
)
FORMAL_DECLARED_JOINT_COUNT = 4723
FORMAL_ZERO_JOINT_ASSET_COUNT = 55
SPAWN_EAGAIN_TOTAL_WAIT_SECONDS = 1800.0
SPAWN_EAGAIN_INITIAL_BACKOFF_SECONDS = 1.0
SPAWN_EAGAIN_MAX_BACKOFF_SECONDS = 30.0
SPAWN_EAGAIN_BACKOFF_MULTIPLIER = 2.0
SPAWN_EAGAIN_EXHAUSTED_REASON_PREFIX = (
    "child_spawn_failed: SpawnEagainRetryBudgetExhausted:"
)
_SPAWN_EAGAIN_SENTINEL = "_infinite_mobility_spawn_eagain_retry"
FORMAL_RECOVERY_PROVENANCE_FIELDS = (
    "factory",
    "original_elapsed_seconds",
    "original_record_path",
    "original_record_sha256",
    "original_status",
    "package_sha256",
    "recovery_elapsed_seconds",
    "recovery_record",
    "recovery_record_path",
    "recovery_record_sha256",
    "recovery_status",
    "seed",
)
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")


class SpawnEagainRetryBudgetExhausted(RuntimeError):
    """Leave a resumable checkpoint after bounded spawn backoff is exhausted."""


_sleep_spawn_retry = time.sleep


def _spawn_eagain_retry_policy() -> dict[str, Any]:
    return {
        "retryable_exception": "BlockingIOError",
        "retryable_errno": 11,
        "scope": "subprocess.Popen only",
        "popen_action": "retry the same spawn in place while preserving active children",
        "total_backoff_wait_seconds": SPAWN_EAGAIN_TOTAL_WAIT_SECONDS,
        "initial_backoff_seconds": SPAWN_EAGAIN_INITIAL_BACKOFF_SECONDS,
        "maximum_backoff_seconds": SPAWN_EAGAIN_MAX_BACKOFF_SECONDS,
        "backoff_multiplier": SPAWN_EAGAIN_BACKOFF_MULTIPLIER,
        "exhaustion": "raise nonzero with running checkpoint preserved for --resume",
        "all_other_spawn_step_failures": "fail-closed metric record",
    }


class _RetryingPopen:
    def __init__(self, delegate: Any) -> None:
        self._delegate = delegate
        self.total_backoff_wait = 0.0
        self.consecutive_failures = 0

    def backoff(self) -> None:
        delay = min(
            SPAWN_EAGAIN_INITIAL_BACKOFF_SECONDS
            * (
                SPAWN_EAGAIN_BACKOFF_MULTIPLIER
                ** min(self.consecutive_failures, 30)
            ),
            SPAWN_EAGAIN_MAX_BACKOFF_SECONDS,
        )
        if self.total_backoff_wait + delay > SPAWN_EAGAIN_TOTAL_WAIT_SECONDS:
            raise SpawnEagainRetryBudgetExhausted(
                "spawn EAGAIN retry budget exhausted; running checkpoint is "
                "preserved for --resume"
            )
        self.total_backoff_wait += delay
        self.consecutive_failures += 1
        print(
            f"spawn EAGAIN; retrying the same child after {delay:g}s "
            f"(backoff {self.total_backoff_wait:g}/"
            f"{SPAWN_EAGAIN_TOTAL_WAIT_SECONDS:g}s)",
            flush=True,
        )
        _sleep_spawn_retry(delay)

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        while True:
            try:
                process = self._delegate(*args, **kwargs)
            except BlockingIOError as error:
                if error.errno != 11:
                    raise
                self.backoff()
                continue
            self.consecutive_failures = 0
            return process


class _CoreSubprocessProxy:
    def __init__(self, module: Any, popen: _RetryingPopen) -> None:
        self._module = module
        self.Popen = popen

    def __getattr__(self, name: str) -> Any:
        return getattr(self._module, name)


@contextmanager
def _core_spawn_eagain_retry() -> Any:
    original_subprocess = core.subprocess
    retrying_popen = _RetryingPopen(original_subprocess.Popen)
    core.subprocess = _CoreSubprocessProxy(original_subprocess, retrying_popen)
    try:
        yield retrying_popen
    finally:
        core.subprocess = original_subprocess

SOURCE_RECORD_FIELDS = (
    "selection_index",
    "selection_rank",
    "selection_hash",
    "cohort_record_sha256",
    "asset_id",
    "factory",
    "raw_category",
    "seed",
    "original_status",
    "recovery_used",
    "recovery_provenance",
    "source",
    "declared_joint_count_hint",
    "baseline_package_sha256",
    "package",
    "expected_package_path",
    "primary_urdf_relative_path",
    "primary_urdf_sha256",
    "model_urdf_sha256",
)


def _checked_file(path: Path, *, label: str) -> tuple[Path, bytes]:
    checked = core.absolute_path_without_symlink_components(path.absolute(), label=label)
    payload = core.read_regular_file_bytes_nofollow(checked, label=label)
    return checked, payload


def _json_object(payload: bytes, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{label} is not valid UTF-8 JSON") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain one JSON object")
    return value


def load_cohort(path: Path, *, formal: bool) -> tuple[Path, dict[str, Any], dict[str, Any]]:
    """Validate the common freezer receipt and bind every published artifact."""

    checked, payload = _checked_file(path, label="Infinite Mobility cohort manifest")
    manifest = common.verify_cohort_manifest(checked, formal=formal)
    if manifest.get("dataset") != "Infinite Mobility":
        raise ValueError("cohort dataset must be Infinite Mobility")
    assets = manifest.get("assets")
    if not isinstance(assets, list):
        raise ValueError("cohort assets must be an array")
    if manifest.get("N_release") != len(assets) or manifest.get("N_eval") != len(assets):
        raise ValueError("cohort declarations must match the complete asset array")

    artifact_path, artifact_payload = _checked_file(
        checked.parent / "artifact_manifest.json", label="cohort artifact manifest"
    )
    common.verify_artifacts(checked.parent)
    artifact_manifest = _json_object(artifact_payload, label="cohort artifact manifest")
    frozen_manifest_entry = artifact_manifest.get("files", {}).get("manifest.json")
    if not isinstance(frozen_manifest_entry, dict):
        raise ValueError("cohort artifact manifest does not bind manifest.json")
    if (
        frozen_manifest_entry.get("bytes") != len(payload)
        or frozen_manifest_entry.get("sha256") != hashlib.sha256(payload).hexdigest()
    ):
        raise ValueError("cohort manifest artifact binding drift")
    receipt = {
        "cohort_manifest_path": str(checked),
        "cohort_manifest_bytes": len(payload),
        "cohort_manifest_sha256": hashlib.sha256(payload).hexdigest(),
        "cohort_manifest_content_sha256": manifest["manifest_content_sha256"],
        "cohort_artifact_manifest_path": str(artifact_path),
        "cohort_artifact_manifest_bytes": len(artifact_payload),
        "cohort_artifact_manifest_sha256": hashlib.sha256(artifact_payload).hexdigest(),
        "cohort_artifact_manifest": artifact_manifest,
    }
    return checked, manifest, receipt


def _formal_source_bindings(cohort: dict[str, Any]) -> dict[str, dict[str, str]]:
    bindings = cohort.get("source_bindings")
    if not isinstance(bindings, list):
        raise ValueError("formal cohort source bindings are missing")
    indexed: dict[str, dict[str, str]] = {}
    for item in bindings:
        if not isinstance(item, dict):
            raise ValueError("formal cohort source binding is not an object")
        label = item.get("label")
        path = item.get("path")
        digest = item.get("sha256")
        if (
            not isinstance(label, str)
            or label in indexed
            or not isinstance(path, str)
            or not Path(path).is_absolute()
            or not isinstance(digest, str)
            or _SHA256_PATTERN.fullmatch(digest) is None
        ):
            raise ValueError("formal cohort source binding schema is invalid")
        checked = common.regular_file(Path(path), label=f"formal source {label}")
        if common.sha256_file(checked) != digest:
            raise ValueError(f"formal cohort source binding drift: {label}")
        indexed[label] = {"label": label, "path": str(checked), "sha256": digest}
    required = {
        "primary_manifest",
        "primary_records",
        "recovery_manifest",
        "recovery_records",
        "cohort_freezer",
        "cohort_preparer",
    }
    if set(indexed) != required:
        raise ValueError("formal cohort source binding labels are invalid")
    return indexed


def _formal_json(path: Path, *, label: str) -> Any:
    checked = common.regular_file(path, label=label)
    try:
        return json.loads(checked.read_text(encoding="utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{label} is not valid UTF-8 JSON") from error


def validate_formal_cohort(cohort: dict[str, Any]) -> None:
    """Independently enforce the exact frozen 20 x 36 formal cohort contract."""

    assets = cohort.get("assets")
    if (
        cohort.get("factory_order") != list(common.APPROVED_FACTORIES)
        or cohort.get("seeds") != list(common.FORMAL_SEEDS)
        or not isinstance(assets, list)
        or len(assets) != common.FORMAL_COUNT
    ):
        raise ValueError("formal cohort must be the exact ordered 20 x 36 matrix")

    bindings = _formal_source_bindings(cohort)
    primary_records_path = Path(bindings["primary_records"]["path"])
    recovery_root = Path(bindings["recovery_records"]["path"]).parent
    primary_root = primary_records_path.parent
    primary_records = _formal_json(primary_records_path, label="formal primary records")
    if not isinstance(primary_records, list):
        raise ValueError("formal primary records must be an array")
    primary_by_identity: dict[tuple[str, int], dict[str, Any]] = {}
    for record in primary_records:
        if not isinstance(record, dict):
            raise ValueError("formal primary record is not an object")
        factory = record.get("factory")
        seed = record.get("seed")
        if not isinstance(factory, str) or type(seed) is not int:
            raise ValueError("formal primary record identity is invalid")
        identity = (factory, seed)
        if identity in primary_by_identity:
            raise ValueError("formal primary record identity is duplicated")
        primary_by_identity[identity] = record

    expected_pairs = [
        (factory, seed)
        for factory in common.APPROVED_FACTORIES
        for seed in common.FORMAL_SEEDS
    ]
    timeout_ids: set[str] = set()
    pass_count = 0
    declared_joint_count = 0
    zero_joint_count = 0
    required_provenance = set(FORMAL_RECOVERY_PROVENANCE_FIELDS)
    for index, ((factory, seed), row) in enumerate(zip(expected_pairs, assets), start=1):
        if not isinstance(row, dict):
            raise ValueError("formal cohort row is not an object")
        asset_id = f"{factory}/seed_{seed:03d}"
        if (
            row.get("selection_index") != index
            or row.get("factory") != factory
            or row.get("seed") != seed
            or row.get("asset_id") != asset_id
            or row.get("raw_category") != factory
        ):
            if (row.get("factory"), row.get("seed")) != (factory, seed):
                raise ValueError("formal cohort must be the exact ordered 20 x 36 matrix")
            raise ValueError("formal cohort identity fields are inconsistent")

        joint_count = row.get("declared_joint_count_hint")
        if type(joint_count) is not int or joint_count < 0:
            raise ValueError("formal cohort declared joint count is invalid")
        declared_joint_count += joint_count
        zero_joint_count += joint_count == 0

        original_status = row.get("original_status")
        if original_status == "PASS":
            if (
                row.get("recovery_used") is not False
                or row.get("source") != "primary"
                or row.get("recovery_provenance") is not None
            ):
                raise ValueError("formal cohort must retain exactly 713 primary PASS rows")
            pass_count += 1
            expected_package = (
                primary_root / "cases" / factory / f"seed_{seed:03d}" / "package"
            )
        elif original_status == "TIMEOUT":
            timeout_ids.add(asset_id)
            if row.get("recovery_used") is not True or row.get("source") != "recovery":
                raise ValueError("formal cohort recovery provenance is inconsistent")
            provenance = row.get("recovery_provenance")
            if not isinstance(provenance, dict) or set(provenance) != required_provenance:
                raise ValueError("formal cohort recovery provenance is incomplete")
            if (
                provenance.get("factory") != factory
                or provenance.get("seed") != seed
                or provenance.get("original_status") != "TIMEOUT"
                or provenance.get("recovery_status") != "PASS"
                or provenance.get("package_sha256") != row.get("baseline_package_sha256")
                or provenance.get("original_record_path") != str(primary_records_path)
                or not isinstance(provenance.get("original_elapsed_seconds"), (int, float))
                or provenance["original_elapsed_seconds"] < 0
                or not isinstance(provenance.get("recovery_elapsed_seconds"), (int, float))
                or provenance["recovery_elapsed_seconds"] < 0
            ):
                raise ValueError("formal cohort recovery provenance values are inconsistent")
            for hash_field in (
                "original_record_sha256",
                "recovery_record_sha256",
                "package_sha256",
            ):
                if _SHA256_PATTERN.fullmatch(str(provenance.get(hash_field, ""))) is None:
                    raise ValueError("formal cohort recovery provenance hash is invalid")
            primary_record = primary_by_identity.get((factory, seed))
            if (
                primary_record is None
                or primary_record.get("status") != "TIMEOUT"
                or common.canonical_sha256(primary_record)
                != provenance["original_record_sha256"]
            ):
                raise ValueError("formal cohort recovery provenance original record mismatch")
            recovery_record_path = (
                recovery_root / "cases" / factory / f"seed_{seed:03d}" / "record.json"
            )
            try:
                expected_relative = recovery_record_path.relative_to(REPO_ROOT).as_posix()
            except ValueError as error:
                raise ValueError("formal cohort recovery record escapes repository") from error
            if (
                provenance.get("recovery_record_path") != str(recovery_record_path)
                or provenance.get("recovery_record") != expected_relative
            ):
                raise ValueError("formal cohort recovery provenance path mismatch")
            checked_recovery = common.regular_file(
                recovery_record_path, label="formal recovery record"
            )
            if common.sha256_file(checked_recovery) != provenance["recovery_record_sha256"]:
                raise ValueError("formal cohort recovery provenance record hash mismatch")
            recovered = _formal_json(checked_recovery, label="formal recovery record")
            if (
                not isinstance(recovered, dict)
                or recovered.get("factory") != factory
                or recovered.get("seed") != seed
                or recovered.get("status") != "PASS"
            ):
                raise ValueError("formal cohort recovery provenance record identity mismatch")
            expected_package = (
                recovery_root / "cases" / factory / f"seed_{seed:03d}" / "package"
            )
        else:
            raise ValueError("formal cohort must retain exactly 713 primary PASS rows")

        if row.get("package_path") != str(expected_package):
            raise ValueError("formal cohort source package path is inconsistent")

    if pass_count != 713:
        raise ValueError("formal cohort must retain exactly 713 primary PASS rows")
    if timeout_ids != set(common.APPROVED_RECOVERY_IDENTITIES):
        raise ValueError("formal cohort must use the exact seven TIMEOUT recoveries")
    if declared_joint_count != FORMAL_DECLARED_JOINT_COUNT:
        raise ValueError("formal cohort must have J=4723 declared joints")
    if zero_joint_count != FORMAL_ZERO_JOINT_ASSET_COUNT:
        raise ValueError("formal cohort must have zero-joint=55")


def validate_contract(
    args: argparse.Namespace, cohort: dict[str, Any], receipt: dict[str, Any]
) -> None:
    if args.workers <= 0 or args.asset_timeout_seconds <= 0:
        raise ValueError("workers and asset timeout must be positive")
    if args.limit is not None and args.limit <= 0:
        raise ValueError("limit must be positive")
    if args.mode == "smoke":
        return
    if args.mode != "formal":
        raise ValueError(f"unsupported mode: {args.mode!r}")
    if args.limit is not None:
        raise ValueError("formal Table 2 does not permit --limit")
    assets = cohort.get("assets")
    if (
        cohort.get("N_release") != common.FORMAL_COUNT
        or cohort.get("N_eval") != common.FORMAL_COUNT
        or not isinstance(assets, list)
        or len(assets) != common.FORMAL_COUNT
    ):
        raise ValueError("formal Infinite Mobility Table 2 requires exactly 720 assets")
    trusted_receipt = {
        "cohort_manifest_sha256": FORMAL_COHORT_MANIFEST_SHA256,
        "cohort_manifest_content_sha256": FORMAL_COHORT_CONTENT_SHA256,
        "cohort_artifact_manifest_sha256": FORMAL_COHORT_ARTIFACT_MANIFEST_SHA256,
    }
    if any(receipt.get(field) != value for field, value in trusted_receipt.items()):
        raise ValueError("formal Infinite Mobility frozen cohort identity mismatch")
    validate_formal_cohort(cohort)
    if args.workers != FORMAL_WORKERS or args.asset_timeout_seconds != FORMAL_TIMEOUT_SECONDS:
        raise ValueError(
            f"formal Table 2 requires workers={FORMAL_WORKERS} and "
            f"asset_timeout_seconds={FORMAL_TIMEOUT_SECONDS:g}"
        )
    if args.no_standard_parser:
        raise ValueError("formal Table 2 requires the standard urdfpy parser")
    if Path(sys.executable).absolute() != FORMAL_PYTHON.absolute():
        raise RuntimeError(
            f"formal Table 2 requires the canonical Python environment: {FORMAL_PYTHON}"
        )
    if Path(args.cohort_manifest).absolute() != DEFAULT_COHORT_MANIFEST.absolute():
        raise ValueError("formal Table 2 requires the canonical frozen cohort manifest")
    core.validate_formal_parent_runtime(args)


def _freeze_records(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f"cohort row {index} is not an object")
        required = {
            "asset_id", "factory", "raw_category", "seed", "original_status",
            "recovery_used", "recovery_provenance", "package_path", "urdf_relpath",
            "primary_urdf_sha256", "baseline_package_sha256", "package_binding",
            "selection_index", "source", "declared_joint_count_hint",
        }
        missing = sorted(required - set(row))
        if missing:
            raise ValueError(f"cohort row {index} lacks fields: {', '.join(missing)}")
        asset_id = row["asset_id"]
        if not isinstance(asset_id, str) or not asset_id:
            raise ValueError(f"cohort row {index} has invalid asset_id")
        package = Path(str(row["package_path"]))
        if not package.is_absolute():
            raise ValueError(f"cohort package path must be absolute: {asset_id}")
        relative = row["urdf_relpath"]
        core.safe_package_relative_path(str(relative), field="primary URDF relative path")
        if row["selection_index"] != index + 1:
            raise ValueError(
                f"cohort selection_index does not match frozen order: {asset_id}"
            )
        expected_source = "recovery" if row["recovery_used"] is True else "primary"
        if row["source"] != expected_source:
            raise ValueError(f"cohort source provenance is inconsistent: {asset_id}")
        if type(row["declared_joint_count_hint"]) is not int or row[
            "declared_joint_count_hint"
        ] < 0:
            raise ValueError(f"cohort declared joint count is invalid: {asset_id}")
        records.append(
            {
                "selection_index": index + 1,
                "selection_rank": index + 1,
                "selection_hash": core.canonical_sha256([index, asset_id]),
                "cohort_record_sha256": common.canonical_sha256(row),
                "asset_id": asset_id,
                "factory": row["factory"],
                "raw_category": row["raw_category"],
                "seed": row["seed"],
                "original_status": row["original_status"],
                "recovery_used": row["recovery_used"],
                "recovery_provenance": row["recovery_provenance"],
                "source": row["source"],
                "declared_joint_count_hint": row["declared_joint_count_hint"],
                "baseline_package_sha256": row["baseline_package_sha256"],
                "package": str(package),
                "expected_package_path": str(package),
                "primary_urdf_relative_path": str(relative),
                "primary_urdf_sha256": row["primary_urdf_sha256"],
                "model_urdf_sha256": row["primary_urdf_sha256"],
                "package_binding": row["package_binding"],
            }
        )
    if len({record["asset_id"] for record in records}) != len(records):
        raise ValueError("cohort asset IDs must be unique")
    return records


def _openblas_threadpools_observed() -> list[dict[str, Any]]:
    site_packages = Path(core.np.__file__).resolve().parent.parent
    libraries = sorted(
        {
            path.resolve()
            for directory in ("numpy.libs", "scipy.libs")
            for path in (site_packages / directory).glob("*openblas*.so*")
            if path.is_file()
        },
        key=str,
    )
    symbols = (
        "scipy_openblas_get_num_threads64_",
        "scipy_openblas_get_num_threads_64_",
        "scipy_openblas_get_num_threads",
        "scipy_openblas_get_num_threads_",
    )
    observed: list[dict[str, Any]] = []
    for path in libraries:
        library = ctypes.CDLL(str(path))
        for symbol in symbols:
            try:
                getter = getattr(library, symbol)
            except AttributeError:
                continue
            getter.argtypes = []
            getter.restype = ctypes.c_int
            observed.append(
                {
                    "library": path.name,
                    "path": str(path),
                    "symbol": symbol,
                    "num_threads": int(getter()),
                }
            )
            break
        else:
            raise core.FatalRuntimeBindingError(
                f"cannot attest OpenBLAS thread count for {path}"
            )
    if not observed:
        raise core.FatalRuntimeBindingError("no OpenBLAS thread pool could be attested")
    return observed


def _thread_runtime_evidence() -> dict[str, Any]:
    environment = {
        name: os.environ.get(name) for name in FROZEN_THREAD_ENVIRONMENT
    }
    if environment != FROZEN_THREAD_ENVIRONMENT:
        raise core.FatalRuntimeBindingError(
            f"thread environment drift: observed {environment!r}"
        )
    pools = _openblas_threadpools_observed()
    if any(pool["num_threads"] != 1 for pool in pools):
        raise core.FatalRuntimeBindingError(
            f"OpenBLAS thread pool drift: {pools!r}"
        )
    return {
        "thread_environment_observed": environment,
        "openblas_threadpools_observed": pools,
    }


def _environment(run_standard_parser: bool, workers: int) -> dict[str, Any]:
    return {
        **_CORE_ENVIRONMENT_METADATA(run_standard_parser, workers),
        "shared_core_path": str(SHARED_CORE_PATH),
        "shared_core_sha256": core.sha256_file(SHARED_CORE_PATH),
        "spawn_eagain_retry_policy": _spawn_eagain_retry_policy(),
        **_thread_runtime_evidence(),
    }


def _validate_environment_binding(
    evaluation: dict[str, Any],
    *,
    run_standard_parser: bool,
    workers: int,
    phase: str,
) -> None:
    observed = _environment(run_standard_parser, workers)
    expected = evaluation.get("environment")
    if (
        expected != observed
        or evaluation.get("environment_sha256") != core.canonical_sha256(observed)
    ):
        raise core.FatalRuntimeBindingError(
            f"{phase} evaluator environment binding drift"
        )


@contextmanager
def child_runtime_environment():
    """Make freeze, parent preflight, and fresh children observe one runtime."""

    original_executable = sys.executable
    original_environment = {
        name: os.environ.get(name, _MISSING_ENV) for name in FROZEN_THREAD_ENVIRONMENT
    }
    sys.executable = os.path.abspath(sys.executable)
    os.environ.update(FROZEN_THREAD_ENVIRONMENT)
    try:
        yield
    finally:
        sys.executable = original_executable
        for name, value in original_environment.items():
            if value is _MISSING_ENV:
                os.environ.pop(name, None)
            else:
                os.environ[name] = str(value)


@contextmanager
def adapter_core_runtime():
    """Bind the shared scheduler and runtime attestation to this bootstrap."""

    original_script_path = core.SCRIPT_PATH
    original_environment_metadata = core.environment_metadata
    core.SCRIPT_PATH = SCRIPT_PATH
    core.environment_metadata = _environment
    try:
        yield
    finally:
        core.SCRIPT_PATH = original_script_path
        core.environment_metadata = original_environment_metadata


def _adapter_config(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "dataset": "Infinite Mobility",
        "cohort": "supplementary full generated 20 factory x 36 seed cohort",
        "shared_table2_core_profile": CORE_DATASET_PROFILE,
        "metrics": list(core.METRIC_NAMES),
        "workers": args.workers,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "standard_parser": not args.no_standard_parser,
        "package_binding_policy": "frozen recursive binding checked inside each child before and after audit",
        "denominator_policy": "all frozen identities including timeout, exception, and package drift failures",
        "recovery_policy": "seven pre-freeze recovery packages retain original_status=TIMEOUT provenance",
        "spawn_eagain_retry_policy": _spawn_eagain_retry_policy(),
        "child_process": core.EVALUATOR_CONFIG["child_process"],
    }


def _command(args: argparse.Namespace) -> list[str]:
    command = [
        str(SCRIPT_PATH),
        "--mode", args.mode,
        "--cohort-manifest", str(Path(args.cohort_manifest).absolute()),
        "--output", str(Path(args.output).absolute()),
        "--workers", str(args.workers),
        "--asset-timeout-seconds", str(args.asset_timeout_seconds),
    ]
    if args.limit is not None:
        command.extend(["--limit", str(args.limit)])
    if args.no_standard_parser:
        command.append("--no-standard-parser")
    return command


def build_manifest(
    args: argparse.Namespace,
    cohort: dict[str, Any],
    cohort_receipt: dict[str, Any],
    records: list[dict[str, Any]],
    protocol_binding: dict[str, str],
) -> dict[str, Any]:
    run_standard_parser = not args.no_standard_parser
    environment = _environment(run_standard_parser, args.workers)
    core_config = core.evaluator_config_for_dataset(CORE_DATASET_PROFILE)
    adapter_config = _adapter_config(args)
    status_counts = Counter(str(row["original_status"]) for row in records)
    recovery_count = sum(row["recovery_used"] is True for row in records)
    evaluation = {
        "metric_names": list(core.METRIC_NAMES),
        "standard_parser": "urdfpy" if run_standard_parser else "disabled",
        "standard_parser_version": "0.0.22" if run_standard_parser else None,
        "workers": args.workers,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "gpu_required": False,
        "child_thread_environment": dict(FROZEN_THREAD_ENVIRONMENT),
        "denominator_policy": "all frozen selected assets, including failures",
        "category_macro_policy": "unweighted mean over the 20 factory names",
        "command": _command(args),
        "environment": environment,
        "environment_sha256": core.canonical_sha256(environment),
        "environment_file_sha256": hashlib.sha256(
            core.formatted_json(environment).encode("utf-8")
        ).hexdigest(),
        "evaluator_path": str(SCRIPT_PATH),
        "evaluator_sha256": core.sha256_file(SCRIPT_PATH),
        "shared_core_path": str(SHARED_CORE_PATH),
        "shared_core_sha256": core.sha256_file(SHARED_CORE_PATH),
        "adapter_path": str(SCRIPT_PATH),
        "adapter_sha256": core.sha256_file(SCRIPT_PATH),
        **core.protocol_binding_fields(protocol_binding),
        "config": core_config,
        "config_sha256": core.canonical_sha256(core_config),
        "adapter_config": adapter_config,
        "adapter_config_sha256": core.canonical_sha256(adapter_config),
    }
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "created_at": core.utc_now(),
        "dataset": "Infinite Mobility",
        "mode": args.mode,
        "classification": core.run_classification(args.mode),
        "release_status": "SUPPLEMENTARY_FULL_GENERATED_COHORT",
        "cohort_type": "SUPPLEMENTARY_FULL_GENERATED_COHORT_NOT_OFFICIAL_FINITE_RELEASE",
        "source": {
            **cohort_receipt,
            "N_release": cohort["N_release"],
            "factory_order": cohort["factory_order"],
            "seeds": cohort["seeds"],
            "source_selection": cohort["source_selection"],
            "source_bindings": cohort["source_bindings"],
            "cohort_source_bindings": cohort.get("source"),
            "cohort_evaluation_bindings": cohort.get("evaluation"),
            "cohort_validator_path": str(common.SCRIPT),
            "cohort_validator_sha256": common.sha256_file(common.SCRIPT),
            "original_status_counts": dict(sorted(status_counts.items())),
            "recovery_overlay_count": recovery_count,
        },
        "selection": {
            "algorithm": "exact common cohort manifest order; optional prefix only in smoke mode",
            "n_eval": len(records),
            "limit": args.limit,
            "selected_asset_ids_sha256": core.canonical_sha256(
                [record["asset_id"] for record in records]
            ),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
            "failed_assets_retained_without_replacement": True,
        },
        "evaluation": evaluation,
        "records": records,
    }
    manifest["manifest_content_sha256"] = core.manifest_self_hash(manifest)
    return manifest


def _prepare_output(output: Path, *, resume: bool) -> Path:
    output = core.absolute_path_without_symlink_components(
        output.absolute(), label="Table 2 output root"
    )
    if resume:
        if not output.is_dir():
            raise FileNotFoundError(f"resume output does not exist: {output}")
    else:
        try:
            output.mkdir(parents=True, exist_ok=False)
        except FileExistsError as error:
            raise FileExistsError(f"output root already exists: {output}") from error
    return output


def _load_run_manifest(output: Path) -> dict[str, Any]:
    path, payload = _checked_file(output / "manifest.json", label="Table 2 run manifest")
    manifest = _json_object(payload, label="Table 2 run manifest")
    if manifest.get("manifest_content_sha256") != core.manifest_self_hash(manifest):
        raise RuntimeError("Table 2 run manifest self-hash mismatch")
    if path.parent != output:
        raise RuntimeError("Table 2 run manifest path escaped output")
    return manifest


def _validate_resume_manifest(
    manifest: dict[str, Any],
    args: argparse.Namespace,
    cohort: dict[str, Any],
    receipt: dict[str, Any],
    expected_records: list[dict[str, Any]],
    output: Path,
) -> None:
    if manifest.get("dataset") != "Infinite Mobility" or manifest.get("mode") != args.mode:
        raise RuntimeError("resume manifest dataset or mode mismatch")
    if manifest.get("records") != expected_records:
        raise RuntimeError("resume manifest frozen record binding mismatch")
    source = manifest.get("source", {})
    for field, value in receipt.items():
        if source.get(field) != value:
            raise RuntimeError(f"resume cohort binding mismatch: {field}")
    if source.get("source_bindings") != cohort.get("source_bindings"):
        raise RuntimeError("resume source binding mismatch")
    evaluation = manifest.get("evaluation", {})
    expected_runtime = {
        "workers": args.workers,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "standard_parser": "disabled" if args.no_standard_parser else "urdfpy",
        "child_thread_environment": dict(FROZEN_THREAD_ENVIRONMENT),
        "adapter_sha256": core.sha256_file(SCRIPT_PATH),
        "shared_core_sha256": core.sha256_file(SHARED_CORE_PATH),
        "environment_sha256": core.canonical_sha256(
            _environment(not args.no_standard_parser, args.workers)
        ),
        "config_sha256": core.canonical_sha256(
            core.evaluator_config_for_dataset(CORE_DATASET_PROFILE)
        ),
        "adapter_config_sha256": core.canonical_sha256(_adapter_config(args)),
    }
    for field, value in expected_runtime.items():
        if evaluation.get(field) != value:
            raise RuntimeError(f"resume evaluator binding mismatch: {field}")
    _validate_environment_binding(
        evaluation,
        run_standard_parser=not args.no_standard_parser,
        workers=args.workers,
        phase="resume parent preflight",
    )
    environment_path, environment_bytes = _checked_file(
        output / "environment.json", label="Table 2 environment"
    )
    if environment_path.parent != output:
        raise RuntimeError("environment artifact escaped output")
    if hashlib.sha256(environment_bytes).hexdigest() != evaluation.get(
        "environment_file_sha256"
    ):
        raise RuntimeError("resume environment SHA-256 mismatch")
    core.validate_protocol_snapshot_binding(output, evaluation)


def _checked_raw_package_path(package_value: Any, expected_value: Any) -> Path:
    if not isinstance(package_value, str) or not isinstance(expected_value, str):
        raise ValueError("package path binding is missing")
    if package_value != expected_value:
        raise ValueError("package path differs from expected frozen path")
    package = Path(package_value)
    if not package.is_absolute():
        raise ValueError("package path must be absolute")
    if any(component in {"", ".", ".."} for component in package_value.split("/")[1:]):
        raise ValueError("package path contains a non-canonical component")
    current = Path(package.anchor)
    for component in package.parts[1:]:
        current = current / component
        try:
            mode = os.lstat(current).st_mode
        except OSError as error:
            raise ValueError(f"missing package path component: {current}") from error
        if stat.S_ISLNK(mode):
            raise ValueError(f"symlink in package path: {current}")
        if not stat.S_ISDIR(mode):
            raise ValueError(f"package path component is not a directory: {current}")
    return package


def _child_attested_failure(job: dict[str, Any], reason: str) -> dict[str, Any]:
    observed = core.validate_child_runtime_binding(job)
    record = core.bound_job_failure(job, reason)
    record["result_origin"] = "child_attested"
    record["job_runtime_binding"] = job["runtime_binding"]
    record["worker_runtime_binding"] = observed
    return record


def audit_frozen_job(job: dict[str, Any]) -> dict[str, Any]:
    """Add raw path identity gates around the unchanged shared audit."""

    core.validate_child_runtime_binding(job)
    try:
        _checked_raw_package_path(job.get("package"), job.get("expected_package_path"))
    except Exception as error:  # noqa: BLE001
        return _child_attested_failure(
            job, f"package_path_precheck_failed: {type(error).__name__}: {error}"
        )
    record = core.audit_frozen_job(job)
    try:
        _checked_raw_package_path(job.get("package"), job.get("expected_package_path"))
    except Exception as error:  # noqa: BLE001
        return _child_attested_failure(
            job, f"package_path_postcheck_failed: {type(error).__name__}: {error}"
        )
    return record


def run_internal_child(job_path: Path, result_path: Path) -> int:
    checked_job, payload = _checked_file(job_path, label="Table 2 internal child job")
    job_root = checked_job.parent
    raw_result = result_path.absolute()
    if checked_job.name != "job.json" or raw_result.name != "result.json":
        raise RuntimeError("internal child paths must use exact job.json/result.json names")
    if raw_result.parent != job_root or not job_root.name.startswith("job_"):
        raise RuntimeError("internal child paths must share one owned job directory")
    if raw_result.exists() or raw_result.is_symlink():
        raise RuntimeError("internal child result already exists")
    job = _json_object(payload, label="Table 2 internal child job")
    if not isinstance(job.get("asset_id"), str):
        raise RuntimeError("internal child job schema is invalid")
    core.atomic_write_json(raw_result, audit_frozen_job(job))
    return 0


def _record_source_fields(record: dict[str, Any]) -> dict[str, Any]:
    return {field: record[field] for field in SOURCE_RECORD_FIELDS}


def _enrich_result(result: dict[str, Any], source: dict[str, Any]) -> dict[str, Any]:
    result.update(_record_source_fields(source))
    return result


def _validate_checkpoint_record(
    record: dict[str, Any], source: dict[str, Any], manifest: dict[str, Any]
) -> str:
    run_token = core.validate_resume_record(
        record,
        source["asset_id"],
        source["model_urdf_sha256"],
        source["package_binding"]["content_manifest_sha256"],
        manifest["manifest_content_sha256"],
        source,
        manifest["evaluation"],
    )
    if run_token is None:
        raise RuntimeError("resume record lacks runtime token")
    for field, value in _record_source_fields(source).items():
        if record.get(field) != value:
            raise RuntimeError(f"resume record source binding mismatch: {field}")
    return run_token


def _checkpoint(
    output: Path,
    manifest_hash: str,
    ordered_present: list[dict[str, Any]],
    n_eval: int,
    *,
    completion_order: int,
    state: str,
    scratch_recoveries: list[dict[str, Any]],
) -> None:
    records_path = output / "records.jsonl"
    core.atomic_write_jsonl(records_path, ordered_present)
    core.atomic_write_json(
        output / "checkpoint.json",
        {
            "schema_version": SCHEMA_VERSION,
            "manifest_content_sha256": manifest_hash,
            "manifest_file_sha256": core.sha256_file(output / "manifest.json"),
            "completed": len(ordered_present),
            "n_eval": n_eval,
            "remaining": n_eval - len(ordered_present),
            "completion_order": completion_order,
            "records_sha256": core.sha256_file(records_path),
            "scratch_recoveries": scratch_recoveries,
            "state": state,
            "updated_at": core.utc_now(),
        },
    )


def _load_checkpoint_records(
    output: Path, manifest: dict[str, Any]
) -> tuple[dict[str, dict[str, Any]], set[str], int, list[dict[str, Any]]]:
    checkpoint_path, checkpoint_bytes = _checked_file(
        output / "checkpoint.json", label="Table 2 checkpoint"
    )
    checkpoint = _json_object(checkpoint_bytes, label="Table 2 checkpoint")
    if checkpoint.get("manifest_content_sha256") != manifest["manifest_content_sha256"]:
        raise RuntimeError("checkpoint manifest hash mismatch")
    if checkpoint.get("manifest_file_sha256") != core.sha256_file(output / "manifest.json"):
        raise RuntimeError("checkpoint manifest file SHA-256 mismatch")
    records_path, records_bytes = _checked_file(
        output / "records.jsonl", label="Table 2 checkpoint records"
    )
    if checkpoint.get("records_sha256") != hashlib.sha256(records_bytes).hexdigest():
        raise RuntimeError("checkpoint records SHA-256 mismatch")
    records = core.load_jsonl(records_path)
    manifest_by_id = {row["asset_id"]: row for row in manifest["records"]}
    by_id: dict[str, dict[str, Any]] = {}
    tokens: set[str] = set()
    completion_orders: set[int] = set()
    for record in records:
        asset_id = record.get("asset_id")
        if asset_id not in manifest_by_id or asset_id in by_id:
            raise RuntimeError(f"invalid or duplicate checkpoint record: {asset_id!r}")
        token = _validate_checkpoint_record(record, manifest_by_id[asset_id], manifest)
        if token in tokens:
            raise RuntimeError("duplicate checkpoint runtime token")
        tokens.add(token)
        order = record.get("completion_order")
        if type(order) is not int or order <= 0 or order in completion_orders:
            raise RuntimeError("checkpoint completion order is invalid or duplicate")
        completion_orders.add(order)
        by_id[asset_id] = record
    if checkpoint.get("completed") != len(by_id) or checkpoint.get("n_eval") != len(
        manifest["records"]
    ):
        raise RuntimeError("checkpoint counts do not match records")
    completion_order = max(completion_orders, default=0)
    if checkpoint.get("completion_order", completion_order) != completion_order:
        raise RuntimeError("checkpoint completion order mismatch")
    recoveries = checkpoint.get("scratch_recoveries")
    if not isinstance(recoveries, list):
        raise RuntimeError("checkpoint scratch recovery evidence is missing")
    for recovery in recoveries:
        if (
            not isinstance(recovery, dict)
            or not isinstance(recovery.get("observed_at"), str)
            or not isinstance(recovery.get("quarantined"), bool)
            or not isinstance(recovery.get("terminated_owned_groups"), list)
            or any(type(pgid) is not int or pgid <= 0 for pgid in recovery["terminated_owned_groups"])
        ):
            raise RuntimeError("checkpoint scratch recovery evidence is invalid")
        if recovery["quarantined"]:
            quarantine = Path(str(recovery.get("quarantine_path", "")))
            if (
                not quarantine.is_absolute()
                or quarantine.parent != output
                or not quarantine.name.startswith("stale_worker_scratch_")
            ):
                raise RuntimeError("checkpoint scratch quarantine binding is invalid")
        elif "quarantine_path" in recovery:
            raise RuntimeError("checkpoint has a false scratch quarantine path")
    return by_id, tokens, completion_order, recoveries


def report_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    recovery = manifest["source"]["recovery_overlay_count"]
    lines = [
        "# Infinite Mobility Table 2 URDF audit",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            "This is a supplementary generated cohort, not an official finite release or "
            "a shared-category balanced panel."
        ),
        "",
        (
            f"Frozen cohort: {summary['n_eval']} identities in exact manifest order; "
            f"{recovery} recovery overlay packages retain their original TIMEOUT provenance."
        ),
        "",
        "| Metric | Result | Factory macro |",
        "|---|---:|---:|",
    ]
    macro = summary["category_macro"]["metrics"]
    for name in core.METRIC_NAMES:
        value = summary["metrics"][name]
        rate = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        macro_rate = macro[name]["rate"]
        macro_text = "N/A" if macro_rate is None else f"{100 * macro_rate:.2f}%"
        lines.append(
            f"| {name} | {value['passed']} / {value['denominator']} ({rate}) | {macro_text} |"
        )
    lines.extend(["", "All timeouts, exceptions, and package drift records remain fail-closed in the asset denominator.", ""])
    return "\n".join(lines)


def _write_artifact_manifest(output: Path) -> None:
    names = (
        "manifest.json",
        "protocol_snapshot.md",
        "environment.json",
        "records.jsonl",
        "summary.json",
        "report.md",
        "checkpoint.json",
    )
    files: dict[str, dict[str, Any]] = {}
    for name in names:
        path, payload = _checked_file(output / name, label=f"Table 2 artifact {name}")
        if path.parent != output:
            raise RuntimeError(f"Table 2 artifact escaped output: {name}")
        files[name] = {"bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()}
    core.atomic_write_json(
        output / "artifact_manifest.json", {"schema_version": 1, "files": files}
    )
    common.verify_artifacts(output)


def _scheduler_exception_record(
    job: dict[str, Any], reason: str
) -> dict[str, Any]:
    if reason.startswith(SPAWN_EAGAIN_EXHAUSTED_REASON_PREFIX):
        return {
            "asset_id": job["asset_id"],
            "error": reason,
            _SPAWN_EAGAIN_SENTINEL: True,
        }
    return core.bound_job_failure(job, f"worker_exception: {reason}")


def run_locked(
    args: argparse.Namespace,
    output: Path,
    cohort: dict[str, Any],
    receipt: dict[str, Any],
    expected_records: list[dict[str, Any]],
) -> Path:
    scratch_recovery = {
        "observed_at": core.utc_now(),
        **core.recover_stale_worker_scratch(output),
    }
    if args.resume:
        if (output / "artifact_manifest.json").exists() or (
            output / "artifact_manifest.json"
        ).is_symlink():
            common.verify_artifacts(output)
        manifest = _load_run_manifest(output)
        _validate_resume_manifest(
            manifest, args, cohort, receipt, expected_records, output
        )
        by_id, issued_tokens, completion_order, scratch_recoveries = (
            _load_checkpoint_records(output, manifest)
        )
        scratch_recoveries.append(scratch_recovery)
    else:
        protocol_binding = core.freeze_protocol_snapshot(output)
        manifest = build_manifest(args, cohort, receipt, expected_records, protocol_binding)
        core.atomic_write_json(output / "manifest.json", manifest)
        core.atomic_write_json(output / "environment.json", manifest["evaluation"]["environment"])
        if core.sha256_file(output / "environment.json") != manifest["evaluation"][
            "environment_file_sha256"
        ]:
            raise RuntimeError("environment metadata write verification failed")
        by_id = {}
        issued_tokens = set()
        completion_order = 0
        scratch_recoveries = [scratch_recovery]
        _checkpoint(
            output,
            manifest["manifest_content_sha256"],
            [],
            len(expected_records),
            completion_order=0,
            state="running",
            scratch_recoveries=scratch_recoveries,
        )

    _validate_environment_binding(
        manifest["evaluation"],
        run_standard_parser=not args.no_standard_parser,
        workers=args.workers,
        phase="parent preflight",
    )
    selected_ids = [record["asset_id"] for record in manifest["records"]]
    manifest_by_id = {record["asset_id"]: record for record in manifest["records"]}

    # A completed checkpoint row is reusable only while its exact package still matches.
    for asset_id in list(by_id):
        source = manifest_by_id[asset_id]
        try:
            _checked_raw_package_path(
                source.get("package"), source.get("expected_package_path")
            )
            current = core.package_binding(Path(source["package"]))
            _checked_raw_package_path(
                source.get("package"), source.get("expected_package_path")
            )
        except Exception:  # noqa: BLE001
            del by_id[asset_id]
            continue
        if current != source["package_binding"]:
            del by_id[asset_id]

    jobs: list[dict[str, Any]] = []
    for asset_id in selected_ids:
        if asset_id in by_id:
            continue
        token = uuid.uuid4().hex
        while token in issued_tokens:
            token = uuid.uuid4().hex
        issued_tokens.add(token)
        job = {
            **manifest_by_id[asset_id],
            "dataset": CORE_DATASET_PROFILE,
            "output_root": str(output),
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "run_standard_parser": not args.no_standard_parser,
            "workers": args.workers,
            "run_token": token,
        }
        job["runtime_binding"] = core.frozen_worker_runtime_binding(manifest, token)
        jobs.append(job)

    def checkpoint_result(result: dict[str, Any]) -> None:
        nonlocal completion_order
        if result.get(_SPAWN_EAGAIN_SENTINEL) is True:
            raise SpawnEagainRetryBudgetExhausted(
                "spawn EAGAIN retry budget exhausted; running checkpoint is "
                "preserved for --resume"
            )
        asset_id = result.get("asset_id")
        if asset_id not in manifest_by_id or asset_id in by_id:
            raise RuntimeError(f"worker returned unknown or duplicate asset: {asset_id!r}")
        _enrich_result(result, manifest_by_id[asset_id])
        _validate_checkpoint_record(result, manifest_by_id[asset_id], manifest)
        completion_order += 1
        result["completion_order"] = completion_order
        by_id[asset_id] = result
        ordered_present = [by_id[item] for item in selected_ids if item in by_id]
        _checkpoint(
            output,
            manifest["manifest_content_sha256"],
            ordered_present,
            len(selected_ids),
            completion_order=completion_order,
            state="running",
            scratch_recoveries=scratch_recoveries,
        )
        print(f"[{len(ordered_present)}/{len(selected_ids)}] {asset_id}", flush=True)

    if jobs:
        scratch = output / ".worker_scratch"
        with _core_spawn_eagain_retry():
            core.execute_killable_jobs(
                jobs,
                worker_scratch=scratch,
                timeout_seconds=args.asset_timeout_seconds,
                max_workers=args.workers,
                on_result=checkpoint_result,
                timeout_factory=lambda job: core.bound_job_failure(
                    job,
                    f"asset_timeout_after_{args.asset_timeout_seconds:g}_seconds",
                    status="timeout",
                ),
                exception_factory=_scheduler_exception_record,
                result_validator=core.validate_frozen_job_result,
            )

    core.validate_protocol_snapshot_binding(output, manifest["evaluation"])
    if set(by_id) != set(selected_ids):
        raise RuntimeError("evaluation ended without one record per frozen identity")
    ordered = [by_id[asset_id] for asset_id in selected_ids]
    summary = core.aggregate_records(
        ordered, expected_n=len(selected_ids), category_field="raw_category"
    )
    summary.update(
        {
            "schema_version": SCHEMA_VERSION,
            "status": "completed",
            "mode": args.mode,
            "classification": core.run_classification(args.mode),
            "dataset": "Infinite Mobility",
            "cohort_type": manifest["cohort_type"],
            "manifest_content_sha256": manifest["manifest_content_sha256"],
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
            "completed_at": core.utc_now(),
        }
    )
    core.atomic_write_json(output / "summary.json", summary)
    core.atomic_write_text(output / "report.md", report_markdown(summary, manifest))
    _checkpoint(
        output,
        manifest["manifest_content_sha256"],
        ordered,
        len(selected_ids),
        completion_order=completion_order,
        state="complete",
        scratch_recoveries=scratch_recoveries,
    )
    _validate_environment_binding(
        manifest["evaluation"],
        run_standard_parser=not args.no_standard_parser,
        workers=args.workers,
        phase="artifact seal",
    )
    _write_artifact_manifest(output)
    return output


def run(args: argparse.Namespace) -> Path:
    with child_runtime_environment(), adapter_core_runtime():
        checked_cohort, cohort, receipt = load_cohort(
            args.cohort_manifest, formal=args.mode == "formal"
        )
        args.cohort_manifest = checked_cohort
        validate_contract(args, cohort, receipt)
        selected_rows = list(cohort["assets"])
        if args.limit is not None:
            selected_rows = selected_rows[: args.limit]
        expected_records = _freeze_records(selected_rows)
        output = _prepare_output(args.output, resume=args.resume)
        with core.output_run_lock(output):
            return run_locked(args, output, cohort, receipt, expected_records)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), default="formal")
    parser.add_argument("--cohort-manifest", type=Path, default=DEFAULT_COHORT_MANIFEST)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=FORMAL_WORKERS)
    parser.add_argument(
        "--asset-timeout-seconds", type=float, default=FORMAL_TIMEOUT_SECONDS
    )
    parser.add_argument("--limit", type=int, help="evaluate only a prefix in smoke mode")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--no-standard-parser", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    effective_argv = list(sys.argv[1:] if argv is None else argv)
    if effective_argv == ["--internal-thread-runtime-probe"]:
        try:
            print(json.dumps(_thread_runtime_evidence(), sort_keys=True))
        except Exception as error:  # noqa: BLE001
            print(f"ERROR: thread runtime probe failed: {type(error).__name__}: {error}", file=sys.stderr)
            return 1
        return 0
    if effective_argv and effective_argv[0] == "--internal-child-job":
        if len(effective_argv) != 4 or effective_argv[2] != "--internal-child-result":
            print("ERROR: invalid internal child invocation", file=sys.stderr)
            return 2
        try:
            with adapter_core_runtime():
                return run_internal_child(
                    Path(effective_argv[1]), Path(effective_argv[3])
                )
        except core.FatalRuntimeBindingError as error:
            print(f"FATAL_RUNTIME_BINDING: {error}", file=sys.stderr)
            return core.RUNTIME_BINDING_FATAL_EXIT_CODE
        except Exception as error:  # noqa: BLE001
            print(
                f"ERROR: internal child failed: {type(error).__name__}: {error}",
                file=sys.stderr,
            )
            return 1
    try:
        output = run(parse_args(effective_argv))
    except Exception as error:  # noqa: BLE001
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps({"status": "completed", "output": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
