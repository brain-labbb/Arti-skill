#!/usr/bin/env python3
"""Evaluate a frozen dataset cohort against the Table 2 URDF gates.

Source packages are opened read-only. Articraft selection uses only sorted release
IDs; Artiverse consumes the exact frozen Table 1 manifest order. The evaluation
manifest is persisted before any selected package is audited.
"""

from __future__ import annotations

import argparse
import base64
import binascii
import builtins
from collections import Counter, deque
from contextlib import contextmanager
import csv
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.metadata
from io import BytesIO, StringIO
import json
import math
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import platform
import random
import re
import signal
import shlex
import shutil
import stat
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any, Iterable
import uuid
import xml.etree.ElementTree as ET


def _early_internal_test_child() -> None:
    argv = sys.argv[1:]
    if len(argv) != 4 or argv[0] != "--internal-child-job" or argv[2] != "--internal-child-result":
        return
    job_path = Path(argv[1])
    result_path = Path(argv[3])
    try:
        job = json.loads(job_path.read_text(encoding="utf-8"))
    except Exception:
        return
    action = job.get("internal_test_action") if isinstance(job, dict) else None
    if action not in {
        "delayed_echo_environment", "echo_environment", "emit_result", "large_stderr",
        "sleep", "spawn_descendant_sleep", "spawn_ignoring_descendant_and_exit"
    }:
        return
    if action == "delayed_echo_environment":
        time.sleep(float(job["sleep"]))
        action = "echo_environment"
    if action == "emit_result":
        result = job["result_payload"]
        temporary = result_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(result, sort_keys=True) + "\n", encoding="utf-8")
        temporary.replace(result_path)
        raise SystemExit(0)
    if action in {"echo_environment", "large_stderr"}:
        if action == "large_stderr":
            remaining = int(job["stderr_bytes"])
            chunk = b"x" * min(65536, remaining)
            while remaining:
                written = os.write(2, chunk[:remaining])
                remaining -= written
        result = {
            "asset_id": job["asset_id"],
            "status": "completed",
            "thread_environment": {
                name: os.environ.get(name)
                for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS")
            },
        }
        temporary = result_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(result, sort_keys=True) + "\n", encoding="utf-8")
        temporary.replace(result_path)
        raise SystemExit(0)
    if action == "spawn_descendant_sleep":
        subprocess.Popen(
            [
                sys.executable,
                "-c",
                "import time; time.sleep(60)",
                str(job["process_group_token"]),
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    if action == "spawn_ignoring_descendant_and_exit":
        ready_path = result_path.parent / "descendant.ready"
        subprocess.Popen(
            [
                sys.executable,
                "-c",
                (
                    "import pathlib,signal,sys,time; "
                    "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
                    "pathlib.Path(sys.argv[1]).write_text('ready'); time.sleep(60)"
                ),
                str(ready_path),
                str(job["process_group_token"]),
            ],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        deadline = time.monotonic() + 1.0
        while not ready_path.is_file() and time.monotonic() < deadline:
            time.sleep(0.002)
        if not ready_path.is_file():
            raise RuntimeError("ignoring descendant did not become ready")
        result = {"asset_id": job["asset_id"], "status": "completed"}
        temporary = result_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(result) + "\n", encoding="utf-8")
        temporary.replace(result_path)
        raise SystemExit(0)
    time.sleep(float(job["sleep"]))
    raise SystemExit(0)


_early_internal_test_child()

import numpy as np
from PIL import Image
import trimesh
from pygltflib import GLTF2


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
INVENTORY_PATH = REPO_ROOT / "exp/dataset_inventory.json"
PROTOCOL_PATH = REPO_ROOT / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
PROTOCOL_SNAPSHOT_NAME = "protocol_snapshot.md"
DEFAULT_SOURCE_ROOT = REPO_ROOT / "exp/Articraft-10K/released_urdf"
DEFAULT_ARTIVERSE_SOURCE_ROOT = REPO_ROOT / "exp/artiverse"
DEFAULT_ARTIVERSE_COHORT_MANIFEST = REPO_ROOT / "exp/runtime/table1_artiverse/manifest.json"
DEFAULT_PARTNET_MOBILITY_SOURCE_ROOT = REPO_ROOT / "exp/PartNet-Mobility/data/dataset"
DEFAULT_PARTNET_MOBILITY_COHORT_MANIFEST = (
    REPO_ROOT / "exp/runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json"
)
DEFAULT_LAM_SOURCE_ROOT = REPO_ROOT / "exp/Articulated-Object-Code/released_outputs"
DEFAULT_LAM_COHORT_RECORDS = (
    REPO_ROOT
    / "exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl"
)
DEFAULT_LAM_TABLE3_MANIFEST = DEFAULT_LAM_COHORT_RECORDS.parent / "manifest.json"
DEFAULT_LAM_RELEASE_MANIFEST = REPO_ROOT / "exp/Articulated-Object-Code/manifest.csv"
DEFAULT_LAM_DATASET_API = REPO_ROOT / "exp/Articulated-Object-Code/dataset_api.json"
DEFAULT_OUTPUT_PARENT = REPO_ROOT / "exp/runtime"
DEFAULT_N = 800
DEFAULT_SEED = 20260813
FORMAL_RELEASE_COUNT = 9996
ARTIVERSE_FORMAL_RELEASE_COUNT = 3544
ARTIVERSE_FORMAL_CATEGORY_COUNT = 67
ARTIVERSE_COHORT_MANIFEST_SHA256 = (
    "f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c"
)
ARTIVERSE_RELEASE_MANIFEST_SHA256 = (
    "8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586"
)
ARTIVERSE_SELECTION_PROTOCOL = "artiverse-table1-global-sample-v1"
PARTNET_MOBILITY_COHORT_MANIFEST_SHA256 = (
    "2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900"
)
PARTNET_MOBILITY_SELECTION_PROTOCOL = "urdf_sim_ready_table4_partnet_mobility_n800_v1"
PARTNET_MOBILITY_FORMAL_RELEASE_COUNT = 2347
PARTNET_MOBILITY_FORMAL_CATEGORY_COUNT = 46
PARTNET_MOBILITY_CANDIDATE_POOL_SHA256 = (
    "0203a510202510cea7e469048e84b133bd65ccbc6e1e3aa90c9bfeea7807959d"
)
PARTNET_MOBILITY_ITEMS_SHA256 = (
    "5f4d0eaa7d50087edc3491a92868950a035dabcb2ad5f8c4d4970aa4c890e5e3"
)
PARTNET_MOBILITY_SELECTED_IDS_SHA256 = (
    "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883"
)
PARTNET_MOBILITY_ARCHIVE_BYTES = 3268124298
PARTNET_MOBILITY_ARCHIVE_SHA256 = (
    "b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff"
)
PARTNET_MOBILITY_SELECTION_SALT = "urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813"
PARTNET_MOBILITY_SELECTION_ALGORITHM = (
    "SHA256(salt + NUL + numeric dataset_id), ascending by (digest, numeric ID)"
)
LAM_DATASET_NAME = "LAM released outputs"
LAM_TABLE3_DATASET_NAME = "LAM released outputs (Articulated-Object-Code)"
LAM_TABLE3_SELECTION_ALGORITHM = "random.Random(seed).sample(sorted(asset_key), n)"
LAM_TABLE2_SELECTION_ALGORITHM = (
    "existing Table 3 manifest records by selection_rank; join completion JSONL by asset_key; "
    "no resampling/reselection"
)
LAM_FORMAL_RELEASE_COUNT = 3217
LAM_FORMAL_COHORT_COUNT = 800
LAM_FORMAL_CATEGORY_COUNT = 305
LAM_FORMAL_TIER_COUNTS = {"broken": 385, "loads_only": 299, "viable": 2533}
LAM_UPSTREAM_REVISION = "28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0"
LAM_COHORT_RECORDS_SHA256 = "7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94"
LAM_TABLE3_MANIFEST_SHA256 = "7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951"
LAM_TABLE3_MANIFEST_CONTENT_SHA256 = (
    "f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2"
)
LAM_RELEASE_MANIFEST_SHA256 = "70216593ec02b71d596e456498ff9863ad0f8e519d5d27d2cf4f58792d412412"
LAM_DATASET_API_SHA256 = "4ef6d64310968fab9e280ca19545ef0647e6c5e2eb81dbd681b5ca20fbbc727d"
LAM_DATASET_INVENTORY_SHA256 = (
    "e281119f870bb6bae9599c3edc02de0a42a257e0d433335361d4a774592c1b5a"
)
LAM_CANDIDATE_POOL_SHA256 = "81304d094da5b8411cbb7e0fb3f911b41a934cb3733766cfda4c2f8364f780fd"
LAM_SELECTED_ASSET_KEYS_SHA256 = (
    "643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3"
)
LAM_FORMAL_ARCHIVES = {
    "viable.tar.gz": {
        "bytes": 1185271461,
        "sha256": "a582ef0aa0f3073749adcc73d289a12200e500c1a5762a4ee1530eefc2c4920d",
    },
    "loads_only.tar.gz": {
        "bytes": 194746559,
        "sha256": "e616dc455450ca0f8ea1c76955929f340b75b9fa51b974327e160890620f9a9e",
    },
    "broken.tar.gz": {
        "bytes": 199627935,
        "sha256": "ef0f6e8506e0432febd5ccc4159c6cbf66a9c5cdadd3bdbcd521478e4c0fda3a",
    },
}
ASSET_TIMEOUT_SECONDS = 120.0
AXIS_EPSILON = 1e-12
INERTIA_RELATIVE_TOLERANCE = 1e-12
METRIC_NAMES = (
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
ARTIVERSE_METADATA_FIELDS = (
    "manifest_root",
    "raw_category",
    "source",
    "model_id",
    "chunk_archive",
    "selection_rank",
    "selection_hash",
)
PARTNET_MOBILITY_METADATA_FIELDS = (
    "dataset_id", "category", "order", "protocol_id", "input_identity_sha256",
    "selection_digest", "frozen_item_sha256", "source_item",
)
LAM_METADATA_FIELDS = (
    "asset_key", "category", "tier", "rel_path", "object_release_id", "release_order",
    "declared_joint_count_hint", "selection_rank", "selection_hash",
    "table3_manifest_record_sha256", "table3_record_sha256", "table3_completion_index",
    "table3_status", "table3_error", "table3_parse_success", "table3_tree_valid",
    "table3_strict_kinematic_pass", "table3_manifest_content_sha256",
)
FROZEN_SOURCE_METADATA_FIELDS = (
    ARTIVERSE_METADATA_FIELDS + PARTNET_MOBILITY_METADATA_FIELDS + LAM_METADATA_FIELDS
)
MESH_SUFFIXES = {".obj", ".stl", ".ply", ".off", ".dae", ".glb", ".gltf"}
TEXTURE_SUFFIXES = {
    ".png", ".jpg", ".jpeg", ".bmp", ".tga", ".gif", ".webp", ".tif", ".tiff"
}
SUPPORTED_JOINT_TYPES = {"fixed", "revolute", "continuous", "prismatic", "planar", "floating"}
AXIS_JOINT_TYPES = {"revolute", "continuous", "prismatic", "planar"}
BOUNDED_JOINT_TYPES = {"revolute", "prismatic"}
URI_SCHEME = re.compile(r"^[A-Za-z][A-Za-z0-9+.-]*:")
MTL_TEXTURE_KEYS = {
    "map_ka", "map_kd", "map_ks", "map_ke", "map_d", "bump", "map_bump",
    "disp", "decal", "refl", "norm",
}
NUMPY_COMPATIBILITY_POLICY = {
    "scope": "synchronized URDF.load call only; alias is removed immediately after parsing",
    "aliases": {"numpy.float": "builtins.float"},
    "activation": "only when the standard parser is enabled and numpy.float is absent",
    "site_packages_modified": False,
    "parser_errors_suppressed": False,
}
STANDARD_PARSER_LOCK = threading.Lock()
CHILD_THREAD_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}
EVALUATION_DEPENDENCIES = (
    "numpy",
    "urdfpy",
    "trimesh",
    "Pillow",
    "networkx",
    "pycollada",
    "pygltflib",
    "lxml",
    "six",
    "scipy",
)
EVALUATOR_CONFIG = {
    "schema_version": "1.0.0",
    "metrics": list(METRIC_NAMES),
    "standard_parser": {"package": "urdfpy", "required_version": "0.0.22"},
    "bound_dependencies": list(EVALUATION_DEPENDENCIES),
    "workers": 4,
    "asset_timeout_seconds": int(ASSET_TIMEOUT_SECONDS),
    "child_process": {
        "interpreter": "sys.executable fresh interpreter",
        "start_new_session": True,
        "termination": "SIGTERM owned process group, grace 0.2 seconds, then SIGKILL",
        "thread_environment": CHILD_THREAD_ENVIRONMENT,
        "job_protocol": "per-job JSON input and atomic per-job JSON result in output-owned scratch",
    },
    "axis_epsilon": AXIS_EPSILON,
    "axis_policy": "Euclidean norm must be greater than AXIS_EPSILON",
    "inertia_relative_tolerance": INERTIA_RELATIVE_TOLERANCE,
    "inertia_dtype": "numpy.float64",
    "inertia_policy": (
        "eigvalsh(full symmetric float64 tensor); tau=INERTIA_RELATIVE_TOLERANCE*"
        "largest_eigenvalue; smallest>tau; largest<=sum(two_smaller)+tau"
    ),
    "selection_algorithm": "random.Random(seed).sample(sorted(unique_asset_ids), n)",
    "source_binding": "all regular package files hashed before freeze and before/after audit",
    "denominator_policy": "all frozen selected assets, including failures and exceptions",
    "parse_rate_scope": (
        "urdfpy 0.0.22 full standard-parser load, including eagerly loaded referenced resources; "
        "parse_rate and resource_resolution are not independent"
    ),
    "resource_validation_scope": (
        "URDF mesh/texture, OBJ-MTL-texture, glTF external buffers/images, DAE image closure; "
        "trimesh structured scene loading validates GLB/glTF containers and embedded payloads, "
        "Pillow validates external raster images"
    ),
}
RUNTIME_BINDING_STATIC_FIELDS = (
    "evaluator_path",
    "evaluator_sha256",
    "protocol_path",
    "protocol_sha256",
    "config",
    "config_sha256",
    "environment",
    "environment_sha256",
)
RUNTIME_BINDING_FATAL_EXIT_CODE = 86


class FatalRuntimeBindingError(RuntimeError):
    """The process no longer matches the frozen evaluator provenance."""


def evaluator_config_for_dataset(dataset: str) -> dict[str, Any]:
    if dataset == "Articraft-10K":
        return dict(EVALUATOR_CONFIG)
    if dataset == "Artiverse":
        return {
            **EVALUATOR_CONFIG,
            "selection_algorithm": "existing Table 1 manifest order; no resampling/reselection",
        }
    if dataset == "PartNet-Mobility":
        return {
            **EVALUATOR_CONFIG,
            "selection_algorithm": "existing frozen manifest items order; no resampling/reselection",
        }
    if dataset == LAM_DATASET_NAME:
        return {**EVALUATOR_CONFIG, "selection_algorithm": LAM_TABLE2_SELECTION_ALGORITHM}
    raise ValueError(f"unsupported dataset for evaluator config: {dataset!r}")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def package_file_manifest(package: Path) -> list[dict[str, Any]]:
    """Hash every regular package file without following links outside the package."""

    package = package.resolve(strict=True)
    if not package.is_dir():
        raise NotADirectoryError(package)
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in list(directory_names):
            child = current / name
            if child.is_symlink():
                raise RuntimeError(f"package contains directory symlink: {child.relative_to(package)}")
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise RuntimeError(f"package contains file symlink: {relative}")
            canonical = path.resolve(strict=True)
            canonical.relative_to(package)
            if not canonical.is_file():
                raise RuntimeError(f"package entry is not a regular file: {relative}")
            rows.append(
                {
                    "path": relative,
                    "bytes": canonical.stat().st_size,
                    "sha256": sha256_file(canonical),
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


def manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def atomic_write_bytes(path: Path, payload: bytes) -> None:
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
    atomic_write_text(path, formatted_json(value))


def formatted_json(value: Any) -> str:
    return json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n"


def atomic_write_jsonl(path: Path, records: Iterable[dict[str, Any]]) -> None:
    atomic_write_text(
        path,
        "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in records),
    )


PROTOCOL_BINDING_FIELDS = (
    "protocol_source_path",
    "protocol_source_sha256_at_freeze",
    "protocol_path",
    "protocol_sha256",
)


def read_regular_file_bytes_nofollow(
    path: Path,
    *,
    label: str,
    error_type: type[RuntimeError] = RuntimeError,
) -> bytes:
    """Read one regular-file inode without following a leaf symlink."""

    if path.is_symlink():
        raise error_type(f"{label} must be a regular non-symlink file: {path}")
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
        | getattr(os, "O_NONBLOCK", 0)
    )
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise error_type(f"{label} must be a readable regular non-symlink file: {path}") from exc
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise error_type(f"{label} must be a regular non-symlink file: {path}")
        with os.fdopen(descriptor, "rb") as handle:
            descriptor = -1
            return handle.read()
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def absolute_path_without_symlink_components(
    path: Path,
    *,
    label: str,
    error_type: type[RuntimeError] = RuntimeError,
) -> Path:
    candidate = Path(os.path.abspath(path))
    current = Path(candidate.anchor)
    for part in candidate.parts[1:]:
        current /= part
        if current.is_symlink():
            raise error_type(f"{label} contains a symlink component: {candidate}")
    return candidate


def checked_protocol_snapshot_path(
    output_root: Path,
    protocol_path: str | Path,
    *,
    error_type: type[RuntimeError] = RuntimeError,
) -> Path:
    output = absolute_path_without_symlink_components(
        output_root, label="protocol snapshot output root", error_type=error_type
    )
    if not output.is_dir():
        raise error_type(f"protocol snapshot output root must be a directory: {output}")
    supplied = Path(protocol_path)
    if not supplied.is_absolute():
        raise error_type("protocol snapshot path must be absolute")
    supplied = Path(os.path.abspath(supplied))
    expected = output / PROTOCOL_SNAPSHOT_NAME
    if supplied != expected:
        raise error_type(
            f"protocol snapshot path is outside the deterministic output path: {supplied} != {expected}"
        )
    return absolute_path_without_symlink_components(
        supplied, label="protocol snapshot path", error_type=error_type
    )


def protocol_binding_fields(
    evaluation: dict[str, Any],
    *,
    error_type: type[RuntimeError] = RuntimeError,
) -> dict[str, str]:
    if not isinstance(evaluation, dict):
        raise error_type("protocol snapshot evaluation binding is missing")
    missing = [field for field in PROTOCOL_BINDING_FIELDS if field not in evaluation]
    if missing:
        raise error_type(
            f"protocol snapshot binding is incomplete: {', '.join(sorted(missing))}"
        )
    binding = {field: evaluation[field] for field in PROTOCOL_BINDING_FIELDS}
    if not all(isinstance(value, str) and value for value in binding.values()):
        raise error_type("protocol snapshot binding fields must be non-empty strings")
    if not Path(binding["protocol_source_path"]).is_absolute():
        raise error_type("protocol snapshot source path must be absolute")
    for field in ("protocol_source_sha256_at_freeze", "protocol_sha256"):
        if re.fullmatch(r"[0-9a-f]{64}", binding[field]) is None:
            raise error_type(f"protocol snapshot binding has invalid {field}")
    if binding["protocol_source_sha256_at_freeze"] != binding["protocol_sha256"]:
        raise error_type("protocol snapshot hash does not match its frozen source bytes")
    return binding


def freeze_protocol_snapshot(output: Path) -> dict[str, str]:
    source = absolute_path_without_symlink_components(
        PROTOCOL_PATH, label="live protocol source"
    )
    source_bytes = read_regular_file_bytes_nofollow(source, label="live protocol source")
    source_sha256 = hashlib.sha256(source_bytes).hexdigest()
    snapshot = output / PROTOCOL_SNAPSHOT_NAME
    if snapshot.exists() or snapshot.is_symlink():
        raise RuntimeError(f"protocol snapshot already exists before manifest freeze: {snapshot}")
    atomic_write_bytes(snapshot, source_bytes)
    checked = checked_protocol_snapshot_path(output, snapshot)
    snapshot_bytes = read_regular_file_bytes_nofollow(checked, label="protocol snapshot")
    if snapshot_bytes != source_bytes:
        raise RuntimeError("protocol snapshot write verification failed")
    return {
        "protocol_source_path": str(source),
        "protocol_source_sha256_at_freeze": source_sha256,
        "protocol_path": str(checked),
        "protocol_sha256": source_sha256,
    }


def validate_protocol_snapshot_binding(
    output: Path,
    evaluation: dict[str, Any],
    *,
    error_type: type[RuntimeError] = RuntimeError,
) -> dict[str, str]:
    binding = protocol_binding_fields(evaluation, error_type=error_type)
    snapshot = checked_protocol_snapshot_path(
        output, binding["protocol_path"], error_type=error_type
    )
    payload = read_regular_file_bytes_nofollow(
        snapshot, label="protocol snapshot", error_type=error_type
    )
    if hashlib.sha256(payload).hexdigest() != binding["protocol_sha256"]:
        raise error_type("protocol snapshot SHA-256 drifted from the frozen manifest")
    return binding


def metric(passed: bool, issues: Iterable[str] = (), **details: Any) -> dict[str, Any]:
    return {"pass": bool(passed), "issues": list(issues), **details}


def failed_record(
    asset_id: str,
    package: Path,
    reason: str,
    *,
    primary_urdf_relative_path: str = "model.urdf",
) -> dict[str, Any]:
    metrics = {name: metric(False, [reason]) for name in METRIC_NAMES}
    return {
        "asset_id": asset_id,
        "package": str(package),
        "primary_urdf_relative_path": primary_urdf_relative_path,
        "status": "error",
        "error": reason,
        "metrics": metrics,
        "strict_urdf_pass": False,
    }


def run_classification(mode: str) -> str:
    if mode == "formal":
        return "FORMAL"
    if mode == "smoke":
        return "NON_FORMAL_SMOKE"
    raise ValueError(f"unknown run mode: {mode}")


def validate_run_contract(args: argparse.Namespace) -> None:
    if args.dataset not in {
        "Articraft-10K", "Artiverse", "PartNet-Mobility", LAM_DATASET_NAME
    }:
        raise ValueError(f"unknown dataset profile: {args.dataset}")
    if args.mode == "formal":
        required = {
            "n": DEFAULT_N,
            "seed": DEFAULT_SEED,
            "limit": None,
            "workers": 4,
            "no_standard_parser": False,
            "asset_timeout_seconds": ASSET_TIMEOUT_SECONDS,
        }
        observed = {name: getattr(args, name) for name in required}
        if observed != required:
            raise ValueError(f"formal mode requires exactly {required}; observed {observed}")
        if args.dataset == "Artiverse":
            expected_source = DEFAULT_ARTIVERSE_SOURCE_ROOT.resolve(strict=False)
            expected_cohort = DEFAULT_ARTIVERSE_COHORT_MANIFEST.resolve(strict=False)
            if (
                args.source_root.resolve(strict=False) != expected_source
                or args.cohort_manifest is None
                or args.cohort_manifest.resolve(strict=False) != expected_cohort
            ):
                raise ValueError(
                    "formal Artiverse mode requires the canonical source root and exact "
                    "Table 1 cohort manifest"
                )
        elif args.dataset == "PartNet-Mobility":
            expected_source = DEFAULT_PARTNET_MOBILITY_SOURCE_ROOT.resolve(strict=False)
            expected_cohort = DEFAULT_PARTNET_MOBILITY_COHORT_MANIFEST.resolve(strict=False)
            if (
                args.source_root.resolve(strict=False) != expected_source
                or args.cohort_manifest is None
                or args.cohort_manifest.resolve(strict=False) != expected_cohort
            ):
                raise ValueError(
                    "formal PartNet-Mobility mode requires the canonical source root and "
                    "exact frozen cohort manifest"
                )
        elif args.dataset == LAM_DATASET_NAME:
            expected_source = Path(os.path.abspath(DEFAULT_LAM_SOURCE_ROOT))
            expected_cohort = Path(os.path.abspath(DEFAULT_LAM_COHORT_RECORDS))
            if (
                Path(os.path.abspath(args.source_root)) != expected_source
                or args.cohort_manifest is None
                or Path(os.path.abspath(args.cohort_manifest)) != expected_cohort
            ):
                raise ValueError(
                    "formal LAM released outputs mode requires the canonical source root and "
                    "exact Table 3 asset_records.jsonl cohort"
                )
        elif args.cohort_manifest is not None:
            raise ValueError("formal Articraft-10K mode does not accept a cohort manifest")
    elif args.mode == "smoke":
        if args.n < 0 or (args.limit is not None and args.limit < 0):
            raise ValueError("smoke n and limit must be non-negative")
        if not 1 <= args.workers <= 4:
            raise ValueError("smoke mode requires 1-4 workers")
        if args.dataset in {"Artiverse", "PartNet-Mobility", LAM_DATASET_NAME} and args.cohort_manifest is None:
            raise ValueError(f"{args.dataset} smoke mode requires a cohort manifest")
        if args.dataset == "Articraft-10K" and args.cohort_manifest is not None:
            raise ValueError("Articraft-10K smoke mode does not accept a cohort manifest")
    else:
        raise ValueError(f"unknown run mode: {args.mode}")


def select_asset_ids(asset_ids: Iterable[str], n: int, seed: int) -> list[str]:
    """Sample sorted unique IDs without inspecting any asset content or outcome."""

    pool = sorted(set(asset_ids))
    if n < 0:
        raise ValueError("n must be non-negative")
    if n > len(pool):
        raise ValueError(f"cannot select {n} assets from a pool of {len(pool)}")
    return random.Random(seed).sample(pool, n)


def process_group_exists(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def terminate_owned_process_group(pgid: int, process: subprocess.Popen[bytes] | None = None) -> str:
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        if process is not None:
            process.wait(timeout=0.1)
        return "already_exited"
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline and process_group_exists(pgid):
        time.sleep(0.005)
    termination = "SIGTERM"
    if process_group_exists(pgid):
        try:
            os.killpg(pgid, signal.SIGKILL)
            termination = "SIGKILL"
        except ProcessLookupError:
            pass
    if process is not None:
        try:
            process.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            pass
    return termination


def _read_child_result(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError("child result is not a JSON object")
    return payload


def bounded_file_tail(path: Path, max_bytes: int = 4000) -> dict[str, Any]:
    """Read at most ``max_bytes`` from the end while reporting exact file size."""

    if max_bytes <= 0:
        raise ValueError("max_bytes must be positive")
    with path.open("rb") as handle:
        size = os.fstat(handle.fileno()).st_size
        offset = max(0, size - max_bytes)
        handle.seek(offset)
        payload = handle.read(max_bytes)
    return {
        "stderr_bytes": size,
        "stderr_truncated": size > max_bytes,
        "stderr_tail": payload.decode("utf-8", errors="replace"),
    }


def execute_killable_jobs(
    jobs: list[dict[str, Any]],
    *,
    worker_scratch: Path,
    timeout_seconds: float,
    max_workers: int,
    on_result: Any,
    timeout_factory: Any,
    exception_factory: Any | None = None,
    result_validator: Any | None = None,
) -> list[dict[str, Any]]:
    """Run fresh-interpreter jobs and terminate only their owned process groups."""

    if timeout_seconds <= 0 or max_workers <= 0:
        raise ValueError("timeout_seconds and max_workers must be positive")
    if worker_scratch.exists():
        raise FileExistsError(f"worker scratch already exists: {worker_scratch}")
    worker_scratch.mkdir(parents=False)
    worker_scratch = worker_scratch.resolve(strict=True)
    pending = deque(enumerate(jobs))
    active: dict[
        int, tuple[int, dict[str, Any], subprocess.Popen[bytes], Path, Path, Path, float]
    ] = {}
    results: dict[int, dict[str, Any]] = {}

    def failure(job: dict[str, Any], reason: str) -> dict[str, Any]:
        if exception_factory is not None:
            return exception_factory(job, reason)
        return {"asset_id": job.get("asset_id"), "status": "error", "error": reason}

    try:
        while pending or active:
            while pending and len(active) < max_workers:
                index, job = pending.popleft()
                job_root = worker_scratch / f"job_{index:06d}"
                stderr_handle = None
                process: subprocess.Popen[bytes] | None = None
                try:
                    job_root.mkdir()
                    job_path = job_root / "job.json"
                    result_path = job_root / "result.json"
                    stderr_path = job_root / "stderr.log"
                    atomic_write_json(job_path, job)
                    if "runtime_binding" in job:
                        validate_child_runtime_binding(job)
                    environment = dict(os.environ)
                    environment.update(CHILD_THREAD_ENVIRONMENT)
                    ownership_path = job_root / "ownership.json"
                    planned_ownership = {
                        "pid": None,
                        "pgid": None,
                        "run_token": job.get("run_token"),
                        "job_path": str(job_path),
                        "runner_script": str(SCRIPT_PATH),
                        "runner_sha256": sha256_file(SCRIPT_PATH),
                        "output_root": str(worker_scratch.parent),
                        "process_start_identity": None,
                    }
                    atomic_write_json(ownership_path, planned_ownership)
                    stderr_handle = stderr_path.open("xb")
                    process = subprocess.Popen(
                        [
                            sys.executable,
                            str(SCRIPT_PATH),
                            "--internal-child-job",
                            str(job_path),
                            "--internal-child-result",
                            str(result_path),
                        ],
                        stdin=subprocess.DEVNULL,
                        stdout=subprocess.DEVNULL,
                        stderr=stderr_handle,
                        env=environment,
                        start_new_session=True,
                    )
                    pgid = process.pid
                    atomic_write_json(
                        ownership_path,
                        {
                            **planned_ownership,
                            "pid": process.pid,
                            "pgid": pgid,
                            "process_start_identity": proc_start_identity(process.pid),
                        },
                    )
                except FatalRuntimeBindingError:
                    if stderr_handle is not None:
                        stderr_handle.close()
                    if process is not None:
                        terminate_owned_process_group(process.pid, process)
                    if job_root.is_dir():
                        shutil.rmtree(job_root)
                    raise
                except Exception as exc:  # noqa: BLE001
                    if stderr_handle is not None:
                        stderr_handle.close()
                    if process is not None:
                        terminate_owned_process_group(process.pid, process)
                    if job_root.is_dir():
                        shutil.rmtree(job_root)
                    record = failure(job, f"child_spawn_failed: {type(exc).__name__}: {exc}")
                    results[index] = record
                    on_result(record)
                    continue
                stderr_handle.close()
                active[process.pid] = (
                    index,
                    job,
                    process,
                    job_root,
                    result_path,
                    stderr_path,
                    time.monotonic(),
                )

            emitted = False
            for pid, state in list(active.items()):
                index, job, process, job_root, result_path, stderr_path, started = state
                returncode = process.poll()
                timed_out = returncode is None and time.monotonic() - started >= timeout_seconds
                if returncode is None and not timed_out:
                    continue
                termination = None
                if timed_out:
                    termination = terminate_owned_process_group(process.pid, process)
                    returncode = process.returncode
                stderr_evidence = bounded_file_tail(stderr_path)
                stderr_tail = stderr_evidence["stderr_tail"]
                if timed_out:
                    record = timeout_factory(job)
                elif returncode == RUNTIME_BINDING_FATAL_EXIT_CODE:
                    raise FatalRuntimeBindingError(
                        f"child runtime binding failed for {job.get('asset_id')}: {stderr_tail}"
                    )
                elif returncode != 0:
                    record = failure(job, f"child_exit_{returncode}: {stderr_tail}")
                elif not result_path.is_file():
                    record = failure(job, f"child_result_missing: {stderr_tail}")
                else:
                    try:
                        record = _read_child_result(result_path)
                        if result_validator is not None:
                            result_validator(job, record)
                    except FatalRuntimeBindingError:
                        raise
                    except Exception as exc:  # noqa: BLE001
                        record = failure(
                            job, f"child_result_invalid: {type(exc).__name__}: {exc}"
                        )
                if not timed_out and process_group_exists(pid):
                    termination = terminate_owned_process_group(pid, process)
                record["worker_evidence"] = {
                    "pid": pid,
                    "returncode": returncode,
                    "termination": termination,
                    **stderr_evidence,
                    "fresh_interpreter": True,
                    "start_new_session": True,
                }
                del active[pid]
                shutil.rmtree(job_root)
                results[index] = record
                on_result(record)
                emitted = True
            if active and not emitted:
                time.sleep(0.005)
    finally:
        for (
            _index,
            _job,
            process,
            job_root,
            _result_path,
            _stderr_path,
            _started,
        ) in active.values():
            terminate_owned_process_group(process.pid, process)
            if job_root.parent == worker_scratch and job_root.name.startswith("job_"):
                shutil.rmtree(job_root, ignore_errors=True)
        if worker_scratch.is_dir() and not any(worker_scratch.iterdir()):
            worker_scratch.rmdir()
    return [results[index] for index in range(len(jobs))]


def local_tag(node: ET.Element) -> str:
    return node.tag.rsplit("}", 1)[-1].lower()


def children(node: ET.Element, tag: str) -> list[ET.Element]:
    return [child for child in node if local_tag(child) == tag]


def first_child(node: ET.Element, tag: str) -> ET.Element | None:
    return next((child for child in node if local_tag(child) == tag), None)


def descendants(node: ET.Element, tag: str) -> Iterable[ET.Element]:
    return (child for child in node.iter() if local_tag(child) == tag)


def finite_scalar(raw: str | None) -> float | None:
    if raw is None:
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def finite_vector(raw: str | None, length: int = 3) -> list[float] | None:
    if raw is None:
        return None
    try:
        values = [float(value) for value in raw.split()]
    except ValueError:
        return None
    if len(values) != length or not all(math.isfinite(value) for value in values):
        return None
    return values


def standard_parser_result(urdf_path: Path, enabled: bool) -> tuple[bool, list[str], dict[str, Any]]:
    parse_scope = EVALUATOR_CONFIG["parse_rate_scope"]
    try:
        parser_version = importlib.metadata.version("urdfpy")
    except importlib.metadata.PackageNotFoundError:
        parser_version = None
    if not enabled:
        return True, [], {
            "enabled": False,
            "parser": "urdfpy",
            "version": parser_version,
            "scope": parse_scope,
            "numpy_compatibility": {**NUMPY_COMPATIBILITY_POLICY, "active_aliases": []},
        }
    if parser_version != "0.0.22":
        return False, [f"standard_parser_version_mismatch: {parser_version!r} != '0.0.22'"], {
            "enabled": True,
            "parser": "urdfpy",
            "version": parser_version,
            "available": parser_version is not None,
            "scope": parse_scope,
            "numpy_compatibility": {**NUMPY_COMPATIBILITY_POLICY, "active_aliases": []},
        }
    try:
        from urdfpy import URDF  # type: ignore[import-not-found]
    except Exception as exc:  # noqa: BLE001
        return False, [f"standard_parser_unavailable: {type(exc).__name__}: {exc}"], {
            "enabled": True,
            "parser": "urdfpy",
            "version": parser_version,
            "available": False,
            "scope": parse_scope,
            "numpy_compatibility": {**NUMPY_COMPATIBILITY_POLICY, "active_aliases": []},
        }
    shim_applied = False
    parser_error: Exception | None = None
    with STANDARD_PARSER_LOCK:
        if "float" not in np.__dict__:
            np.float = builtins.float  # type: ignore[attr-defined]
            shim_applied = True
        try:
            URDF.load(str(urdf_path))
        except Exception as exc:  # noqa: BLE001
            parser_error = exc
        finally:
            if shim_applied:
                delattr(np, "float")
    compatibility = {
        **NUMPY_COMPATIBILITY_POLICY,
        "active_aliases_during_parse": ["numpy.float"] if shim_applied else [],
        "applied_this_call": shim_applied,
        "restored_after_parse": shim_applied,
    }
    if parser_error is not None:
        return False, [
            f"standard_parser_failed: {type(parser_error).__name__}: {parser_error}"
        ], {
            "enabled": True,
            "parser": "urdfpy",
            "version": parser_version,
            "available": True,
            "scope": parse_scope,
            "numpy_compatibility": compatibility,
        }
    return True, [], {
        "enabled": True,
        "parser": "urdfpy",
        "version": parser_version,
        "available": True,
        "scope": parse_scope,
        "numpy_compatibility": compatibility,
    }


def resolve_reference(package: Path, declaring_file: Path, raw: str) -> tuple[Path | None, str | None]:
    reference = raw.strip()
    if not reference:
        return None, "empty resource reference"
    if "\\" in reference:
        return None, f"non_portable_resource_reference: {reference}"
    if URI_SCHEME.match(reference) or Path(reference).is_absolute() or PureWindowsPath(reference).is_absolute():
        return None, f"non_relative_resource_reference: {reference}"
    try:
        resolved = (declaring_file.parent / reference).resolve(strict=False)
        resolved.relative_to(package)
    except (OSError, ValueError):
        return None, f"resource_escapes_package: {reference}"
    if not resolved.is_file():
        return None, f"resource_missing: {reference}"
    try:
        with resolved.open("rb") as handle:
            if not handle.read(1):
                return None, f"resource_empty: {reference}"
    except OSError as exc:
        return None, f"resource_unreadable: {reference}: {type(exc).__name__}: {exc}"
    return resolved, None


def obj_mtllib_values(obj_path: Path, raw: str) -> list[str]:
    """Parse an OBJ mtllib remainder, preserving existing filenames with spaces.

    OBJ exporters commonly emit an unquoted filename containing spaces.  The
    tokenized form is still useful for the standard multi-library syntax, so
    prefer the complete remainder only when it names an existing file beside
    the OBJ; otherwise retain the tokenized interpretation.
    """

    raw = raw.strip()
    values = shlex.split(raw, comments=True)
    if len(values) <= 1:
        return values
    filename = raw.split("#", 1)[0].rstrip()
    if filename and (obj_path.parent / filename).is_file():
        return [filename]
    return values


def mtl_texture_value(mtl_path: Path, raw: str) -> str | None:
    """Recover an MTL texture path when an exporter leaves spaces unquoted."""

    values = shlex.split(raw.strip(), comments=True)
    if not values:
        return None
    for start in range(len(values)):
        candidate = " ".join(values[start:])
        if (mtl_path.parent / candidate).is_file():
            return candidate
    return values[-1]


def nested_resource_specs(path: Path) -> list[tuple[str, str]]:
    suffix = path.suffix.lower()
    specs: list[tuple[str, str]] = []
    if suffix == ".obj":
        text = path.read_text(encoding="utf-8")
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.lower().startswith("mtllib "):
                values = obj_mtllib_values(path, stripped[7:])
                specs.extend(("obj_mtl", value) for value in values)
    elif suffix == ".mtl":
        text = path.read_text(encoding="utf-8")
        for line in text.splitlines():
            values = shlex.split(line, comments=True)
            if len(values) >= 2 and values[0].lower() in MTL_TEXTURE_KEYS:
                texture = mtl_texture_value(path, line.strip()[len(values[0]) :])
                if texture:
                    specs.append(("mtl_texture", texture))
    elif suffix == ".gltf":
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("GLTF root is not an object")
        for section in ("buffers", "images"):
            entries = payload.get(section, [])
            if not isinstance(entries, list):
                raise ValueError(f"GLTF {section} is not a list")
            for entry in entries:
                uri = entry.get("uri") if isinstance(entry, dict) else None
                if isinstance(uri, str) and not uri.startswith("data:"):
                    specs.append((f"gltf_{section[:-1]}", uri))
    elif suffix == ".dae":
        dae_root = ET.parse(path).getroot()
        for image_node in (node for node in dae_root.iter() if local_tag(node) == "image"):
            for node in image_node.iter():
                if local_tag(node) == "init_from" and node.text and node.text.strip():
                    value = node.text.strip()
                    if not value.startswith("#"):
                        specs.append(("dae_image", value))
    return specs


def supported_for_kind(kind: str, path: Path) -> bool:
    suffix = path.suffix.lower()
    if kind in {"urdf_mesh"}:
        return suffix in MESH_SUFFIXES
    if kind in {"urdf_texture", "mtl_texture", "dae_image"}:
        return suffix in TEXTURE_SUFFIXES
    if kind == "gltf_image":
        return True
    if kind == "obj_mtl":
        return suffix == ".mtl"
    if kind == "gltf_buffer":
        return True
    return True


def obj_material_issues(obj_path: Path, package: Path) -> list[str]:
    """Verify that every OBJ usemtl symbol is defined by one of its declared MTLs."""

    declared_mtls: list[str] = []
    used_materials: set[str] = set()
    for line in obj_path.read_text(encoding="utf-8").splitlines():
        values = shlex.split(line, comments=True)
        if not values:
            continue
        if values[0].lower() == "mtllib":
            declared_mtls.extend(obj_mtllib_values(obj_path, line.strip()[6:]))
        elif values[0].lower() == "usemtl" and len(values) >= 2:
            used_materials.add(" ".join(values[1:]))
    defined_materials: set[str] = set()
    issues: list[str] = []
    for raw in declared_mtls:
        mtl_path, error = resolve_reference(package, obj_path, raw)
        if error:
            continue  # The closure reports the path error separately.
        assert mtl_path is not None
        for line in mtl_path.read_text(encoding="utf-8").splitlines():
            values = shlex.split(line, comments=True)
            if values and values[0].lower() == "newmtl" and len(values) >= 2:
                defined_materials.add(" ".join(values[1:]))
    for name in sorted(used_materials - defined_materials):
        issues.append(f"obj_usemtl_without_newmtl: {obj_path.relative_to(package)}: {name}")
    return issues


def nested_resource_preflight_issue(
    path: Path, package: Path, visited: set[Path] | None = None
) -> str | None:
    """Recursively contain nested references before third-party mesh loading."""

    if visited is None:
        visited = set()
    canonical = path.resolve(strict=False)
    if canonical in visited:
        return None
    visited.add(canonical)
    try:
        specs = nested_resource_specs(path)
    except Exception as exc:  # noqa: BLE001
        return f"nested_resource_unreadable: {type(exc).__name__}: {exc}"
    for kind, raw in specs:
        resolved, error = resolve_reference(package, path, raw)
        if error:
            return f"{kind}_{error}"
        assert resolved is not None
        if not supported_for_kind(kind, resolved):
            return f"unsupported_{kind}_format: {resolved.suffix.lower() or '<none>'}: {raw}"
        if resolved.suffix.lower() in {".obj", ".mtl", ".dae", ".gltf"}:
            nested_issue = nested_resource_preflight_issue(resolved, package, visited)
            if nested_issue:
                return f"{kind}_{nested_issue}"
    return None


def package_resource_containment_preflight(
    root: ET.Element, package: Path, urdf_path: Path
) -> dict[str, Any]:
    """Resolve the package's complete declared resource closure without loaders."""

    package = package.resolve(strict=True)
    urdf_path = urdf_path.resolve(strict=True)
    queue: deque[tuple[str, str, Path]] = deque()
    for node in descendants(root, "mesh"):
        queue.append(("urdf_mesh", node.attrib.get("filename", ""), urdf_path))
    for node in descendants(root, "texture"):
        queue.append(("urdf_texture", node.attrib.get("filename", ""), urdf_path))
    issues: list[str] = []
    visited_files: set[Path] = set()
    visited_edges: set[tuple[str, str, Path]] = set()
    while queue:
        kind, raw, declaring = queue.popleft()
        edge = (kind, raw, declaring.resolve(strict=False))
        if edge in visited_edges:
            continue
        visited_edges.add(edge)
        resolved, error = resolve_reference(package, declaring, raw)
        if error:
            issues.append(f"{kind}: {error}")
            continue
        assert resolved is not None
        if not supported_for_kind(kind, resolved):
            issues.append(
                f"{kind}: unsupported_{kind}_format: "
                f"{resolved.suffix.lower() or '<none>'}: {raw}"
            )
            continue
        canonical = resolved.resolve(strict=False)
        if canonical in visited_files:
            continue
        visited_files.add(canonical)
        try:
            nested = nested_resource_specs(resolved)
        except Exception as exc:  # noqa: BLE001
            issues.append(
                f"{kind}: nested_resource_unreadable: "
                f"{type(exc).__name__}: {exc}"
            )
            continue
        queue.extend((nested_kind, nested_raw, resolved) for nested_kind, nested_raw in nested)
    return {
        "pass": not issues,
        "issues": issues,
        "package": str(package),
        "urdf_path": str(urdf_path),
        "visited_files": sorted(path.relative_to(package).as_posix() for path in visited_files),
    }


def containment_preflight_token_valid(
    token: dict[str, Any] | None, package: Path, urdf_path: Path
) -> bool:
    return bool(
        token
        and token.get("pass") is True
        and token.get("package") == str(package.resolve(strict=True))
        and token.get("urdf_path") == str(urdf_path.resolve(strict=True))
    )


def readable_resource_issue(kind: str, path: Path, package: Path | None = None) -> str | None:
    suffix = path.suffix.lower()
    try:
        if kind == "urdf_mesh":
            nested_issue = nested_resource_preflight_issue(path, package or path.parent)
            if nested_issue:
                return nested_issue
            if suffix in {".glb", ".gltf"}:
                gltf = GLTF2.load(str(path))
                buffers: list[bytes] = []
                for index, buffer in enumerate(gltf.buffers or []):
                    if buffer.uri is None:
                        data = gltf.binary_blob()
                    elif buffer.uri.startswith("data:"):
                        data = decode_data_uri(buffer.uri, expected_image=False)
                    else:
                        resolved, error = resolve_reference(package or path.parent, path, buffer.uri)
                        if error:
                            return f"gltf_buffer_{index}_{error}"
                        assert resolved is not None
                        data = resolved.read_bytes()
                    if data is None or len(data) < int(buffer.byteLength or 0):
                        return f"gltf_buffer_{index}_missing_or_short"
                    buffers.append(data)
                for index, view in enumerate(gltf.bufferViews or []):
                    if view.buffer is None or not 0 <= view.buffer < len(buffers):
                        return f"gltf_buffer_view_{index}_invalid_buffer"
                    start = int(view.byteOffset or 0)
                    end = start + int(view.byteLength or 0)
                    if start < 0 or end > len(buffers[view.buffer]) or end <= start:
                        return f"gltf_buffer_view_{index}_out_of_bounds"
                for index, image_spec in enumerate(gltf.images or []):
                    if image_spec.uri is not None:
                        if image_spec.uri.startswith("data:"):
                            try:
                                image_bytes = decode_data_uri(image_spec.uri, expected_image=True)
                                with Image.open(BytesIO(image_bytes)) as image:
                                    image.verify()
                            except Exception as exc:  # noqa: BLE001
                                return f"gltf_data_uri_image_{index}_invalid: {type(exc).__name__}: {exc}"
                        else:
                            resolved, error = resolve_reference(
                                package or path.parent, path, image_spec.uri
                            )
                            if error:
                                return f"gltf_image_{index}_{error}"
                            assert resolved is not None
                            try:
                                with Image.open(BytesIO(resolved.read_bytes())) as image:
                                    image.verify()
                            except Exception as exc:  # noqa: BLE001
                                return (
                                    f"gltf_external_image_{index}_invalid: "
                                    f"{type(exc).__name__}: {exc}"
                                )
                        continue
                    if image_spec.bufferView is None:
                        continue
                    if not 0 <= image_spec.bufferView < len(gltf.bufferViews or []):
                        return f"gltf_embedded_image_{index}_invalid_buffer_view"
                    view = gltf.bufferViews[image_spec.bufferView]
                    if view.buffer is None or not 0 <= view.buffer < len(buffers):
                        return f"gltf_embedded_image_{index}_invalid_buffer"
                    start = int(view.byteOffset or 0)
                    end = start + int(view.byteLength or 0)
                    try:
                        with Image.open(BytesIO(buffers[view.buffer][start:end])) as image:
                            image.verify()
                    except Exception as exc:  # noqa: BLE001
                        return (
                            f"gltf_embedded_image_{index}_invalid: "
                            f"{type(exc).__name__}: {exc}"
                        )
            loaded = trimesh.load(path, force="scene", process=False)
            geometries = list(getattr(loaded, "geometry", {}).values())
            if not geometries:
                return "mesh_has_no_geometry"
            for geometry in geometries:
                vertices = np.asarray(geometry.vertices)
                faces = np.asarray(geometry.faces)
                if vertices.size == 0 or faces.size == 0 or not np.all(np.isfinite(vertices)):
                    return "mesh_geometry_empty_or_nonfinite"
        elif kind in {"urdf_texture", "mtl_texture", "dae_image", "gltf_image"}:
            with Image.open(path) as image:
                image.verify()
        elif kind == "gltf_buffer" and path.stat().st_size == 0:
            return "gltf_buffer_empty"
        elif suffix in {".obj", ".mtl", ".gltf", ".dae"}:
            path.read_text(encoding="utf-8")
    except Exception as exc:  # noqa: BLE001
        return f"resource_parse_failed: {type(exc).__name__}: {exc}"
    return None


def decode_data_uri(uri: str, *, expected_image: bool) -> bytes:
    if not uri.startswith("data:") or "," not in uri:
        raise ValueError("invalid data URI")
    header, encoded = uri[5:].split(",", 1)
    parts = header.split(";")
    mime = parts[0].lower()
    if parts[-1].lower() != "base64":
        raise ValueError("only strict base64 data URIs are supported")
    if expected_image and not mime.startswith("image/"):
        raise ValueError(f"data URI MIME is not image/*: {mime}")
    try:
        return base64.b64decode(encoded, validate=True)
    except binascii.Error as exc:
        raise ValueError(f"invalid base64 payload: {exc}") from exc


def resource_audit(root: ET.Element, package: Path, urdf_path: Path) -> dict[str, Any]:
    queue: deque[tuple[str, str, Path]] = deque()
    for node in descendants(root, "mesh"):
        queue.append(("urdf_mesh", node.attrib.get("filename", ""), urdf_path))
    for node in descendants(root, "texture"):
        queue.append(("urdf_texture", node.attrib.get("filename", ""), urdf_path))

    issues: list[str] = []
    top_level_materials = {
        node.attrib.get("name", "").strip()
        for node in children(root, "material")
        if node.attrib.get("name", "").strip()
    }
    for material_node in descendants(root, "material"):
        if material_node in children(root, "material"):
            continue
        name = material_node.attrib.get("name", "").strip()
        has_inline_definition = bool(list(material_node))
        if name and not has_inline_definition and name not in top_level_materials:
            issues.append(f"urdf_material_unresolved: {name}")
    records: list[dict[str, Any]] = []
    seen_specs: set[tuple[str, str, str]] = set()
    expanded: set[Path] = set()
    while queue:
        kind, raw, declaring = queue.popleft()
        declared_by = declaring.relative_to(package).as_posix()
        key = (kind, raw, declared_by)
        if key in seen_specs:
            continue
        seen_specs.add(key)
        resolved, error = resolve_reference(package, declaring, raw)
        if error is None and resolved is not None and not supported_for_kind(kind, resolved):
            error = f"unsupported_{kind}_format: {resolved.suffix.lower() or '<none>'}: {raw}"
            resolved = None
        if error is None and resolved is not None:
            readability_error = readable_resource_issue(kind, resolved, package)
            if readability_error:
                error = f"{readability_error}: {raw}"
                resolved = None
        record = {
            "kind": kind,
            "reference": raw,
            "declared_by": declared_by,
            "resolved_path": resolved.relative_to(package).as_posix() if resolved else None,
            "pass": error is None,
            "error": error,
        }
        records.append(record)
        if error:
            issues.append(f"{kind}: {error}")
            continue
        assert resolved is not None
        if resolved in expanded:
            continue
        expanded.add(resolved)
        try:
            nested = nested_resource_specs(resolved)
            if resolved.suffix.lower() == ".obj":
                issues.extend(obj_material_issues(resolved, package))
        except Exception as exc:  # noqa: BLE001
            issues.append(
                f"nested_resource_unreadable: {resolved.relative_to(package)}: "
                f"{type(exc).__name__}: {exc}"
            )
            continue
        queue.extend((nested_kind, nested_raw, resolved) for nested_kind, nested_raw in nested)
    return metric(not issues, issues, references=records, reference_count=len(records))


def finite_fields_audit(root: ET.Element) -> dict[str, Any]:
    issues: list[str] = []
    vector_fields = {"origin": ("xyz", "rpy"), "axis": ("xyz",)}
    scalar_fields = {
        "limit": ("lower", "upper", "effort", "velocity"),
        "mass": ("value",),
        "inertia": ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"),
        "dynamics": ("damping", "friction"),
    }
    for node in root.iter():
        tag = local_tag(node)
        for attribute in vector_fields.get(tag, ()):
            if attribute in node.attrib and finite_vector(node.attrib[attribute]) is None:
                issues.append(f"non_finite_or_invalid_vector: {tag}.{attribute}={node.attrib[attribute]!r}")
        for attribute in scalar_fields.get(tag, ()):
            if attribute in node.attrib and finite_scalar(node.attrib[attribute]) is None:
                issues.append(f"non_finite_or_invalid_scalar: {tag}.{attribute}={node.attrib[attribute]!r}")
    return metric(not issues, issues)


def valid_tree_audit(root: ET.Element) -> dict[str, Any]:
    link_nodes = children(root, "link")
    names = [node.attrib.get("name", "").strip() for node in link_nodes]
    name_set = set(names)
    issues: list[str] = []
    if not names:
        issues.append("no_links")
    if "" in name_set:
        issues.append("empty_link_name")
    if len(name_set) != len(names):
        issues.append("duplicate_link_names")

    joint_nodes = children(root, "joint")
    joint_names = [node.attrib.get("name", "").strip() for node in joint_nodes]
    if "" in joint_names:
        issues.append("empty_joint_name")
    if len(set(joint_names)) != len(joint_names):
        issues.append("duplicate_joint_names")

    adjacency: dict[str, list[str]] = {name: [] for name in name_set if name}
    parent_counts: dict[str, int] = {name: 0 for name in name_set if name}
    for index, joint in enumerate(joint_nodes):
        parent_nodes = children(joint, "parent")
        child_nodes = children(joint, "child")
        if len(parent_nodes) != 1 or len(child_nodes) != 1:
            issues.append(f"joint_{index}_parent_child_cardinality")
        parent_node = parent_nodes[0] if len(parent_nodes) == 1 else None
        child_node = child_nodes[0] if len(child_nodes) == 1 else None
        parent = parent_node.attrib.get("link", "").strip() if parent_node is not None else ""
        child = child_node.attrib.get("link", "").strip() if child_node is not None else ""
        if parent not in name_set or child not in name_set:
            issues.append(f"joint_{index}_unresolved_link")
            continue
        adjacency.setdefault(parent, []).append(child)
        parent_counts[child] = parent_counts.get(child, 0) + 1
        if parent_counts[child] > 1:
            issues.append(f"multiple_parents: {child}")

    roots = [name for name in names if name and parent_counts.get(name, 0) == 0]
    if len(roots) != 1:
        issues.append(f"root_count: {len(roots)}")
    visited: set[str] = set()
    active: set[str] = set()
    cycle = False

    def visit(name: str) -> None:
        nonlocal cycle
        if name in active:
            cycle = True
            return
        if name in visited:
            return
        active.add(name)
        for child in adjacency.get(name, []):
            visit(child)
        active.remove(name)
        visited.add(name)

    for name in names:
        if name:
            visit(name)
    if cycle:
        issues.append("cycle")
    if len(roots) == 1:
        reachable: set[str] = set()
        pending = [roots[0]]
        while pending:
            name = pending.pop()
            if name not in reachable:
                reachable.add(name)
                pending.extend(adjacency.get(name, []))
        if reachable != {name for name in names if name}:
            issues.append("orphan_or_unreachable_links")
        for name in names:
            expected = 0 if name == roots[0] else 1
            if parent_counts.get(name, 0) != expected:
                issues.append(
                    f"invalid_indegree: {name}: {parent_counts.get(name, 0)} != {expected}"
                )
    return metric(
        not issues, issues, link_count=len(names), joint_count=len(joint_nodes), root_links=roots
    )


def valid_joint_spec_audit(root: ET.Element) -> dict[str, Any]:
    link_names = {node.attrib.get("name", "").strip() for node in children(root, "link")}
    issues: list[str] = []
    joints = children(root, "joint")
    for index, joint in enumerate(joints):
        label = joint.attrib.get("name", f"index_{index}")
        kind = joint.attrib.get("type", "").strip().lower()
        if kind not in SUPPORTED_JOINT_TYPES:
            issues.append(f"{label}: unsupported_joint_type: {kind!r}")
        parent_node = first_child(joint, "parent")
        child_node = first_child(joint, "child")
        parent = parent_node.attrib.get("link", "").strip() if parent_node is not None else ""
        child = child_node.attrib.get("link", "").strip() if child_node is not None else ""
        if parent not in link_names or child not in link_names:
            issues.append(f"{label}: parent_child_missing")
        if parent and parent == child:
            issues.append(f"{label}: parent_child_same")
        if kind in AXIS_JOINT_TYPES:
            axis_node = first_child(joint, "axis")
            raw_axis = axis_node.attrib.get("xyz") if axis_node is not None else "1 0 0"
            axis = finite_vector(raw_axis)
            if axis is None or math.sqrt(sum(value * value for value in axis)) <= AXIS_EPSILON:
                issues.append(f"{label}: axis_invalid_or_zero")
        limit = first_child(joint, "limit")
        if kind in BOUNDED_JOINT_TYPES:
            lower = finite_scalar(limit.attrib.get("lower")) if limit is not None else None
            upper = finite_scalar(limit.attrib.get("upper")) if limit is not None else None
            if lower is None or upper is None or not lower < upper:
                issues.append(f"{label}: lower_upper_invalid")
        if kind == "continuous" and limit is not None:
            if "lower" in limit.attrib or "upper" in limit.attrib:
                issues.append(f"{label}: continuous_finite_interval")
    return metric(not issues, issues, joint_count=len(joints))


def primitive_geometry_valid(geometry: ET.Element) -> bool:
    shape_nodes = list(geometry)
    if len(shape_nodes) != 1:
        return False
    shape = shape_nodes[0]
    tag = local_tag(shape)
    if tag == "box":
        values = finite_vector(shape.attrib.get("size"))
        return values is not None and all(value > 0 for value in values)
    if tag == "sphere":
        radius = finite_scalar(shape.attrib.get("radius"))
        return radius is not None and radius > 0
    if tag == "cylinder":
        radius = finite_scalar(shape.attrib.get("radius"))
        length = finite_scalar(shape.attrib.get("length"))
        return radius is not None and length is not None and radius > 0 and length > 0
    if tag == "mesh":
        return bool(shape.attrib.get("filename", "").strip())
    return False


def loadable_collision_geometry(
    geometry: ET.Element,
    package: Path,
    urdf_path: Path,
    containment_preflight: dict[str, Any] | None = None,
) -> tuple[bool, str | None]:
    if not primitive_geometry_valid(geometry):
        return False, "invalid_or_unsupported_geometry"
    shape = list(geometry)[0]
    if local_tag(shape) != "mesh":
        return True, None
    scale = finite_vector(shape.attrib.get("scale", "1 1 1"))
    if scale is None or any(value <= 0 for value in scale):
        return False, "mesh_scale_not_positive_finite"
    mesh_path, error = resolve_reference(package, urdf_path, shape.attrib.get("filename", ""))
    if error:
        return False, error
    assert mesh_path is not None
    if mesh_path.suffix.lower() not in MESH_SUFFIXES:
        return False, f"unsupported_mesh_format: {mesh_path.suffix.lower()}"
    if not containment_preflight_token_valid(containment_preflight, package, urdf_path):
        preflight_issue = nested_resource_preflight_issue(mesh_path, package)
        if preflight_issue:
            return False, f"containment_preflight_failed: {preflight_issue}"
    try:
        loaded = trimesh.load(mesh_path, force="scene", process=False)
        geometries = list(getattr(loaded, "geometry", {}).values())
        if not geometries:
            return False, "mesh_has_no_geometry"
        for mesh in geometries:
            vertices = np.asarray(mesh.vertices)
            faces = np.asarray(mesh.faces)
            if vertices.size == 0 or faces.size == 0 or not np.all(np.isfinite(vertices)):
                return False, "mesh_geometry_empty_or_nonfinite"
    except Exception as exc:  # noqa: BLE001
        return False, f"mesh_load_failed: {type(exc).__name__}: {exc}"
    return True, None


def collision_coverage_audit(
    root: ET.Element,
    package: Path,
    urdf_path: Path,
    containment_preflight: dict[str, Any] | None = None,
) -> dict[str, Any]:
    links = children(root, "link")
    if not containment_preflight_token_valid(containment_preflight, package, urdf_path):
        containment_preflight = package_resource_containment_preflight(root, package, urdf_path)
    if not containment_preflight["pass"]:
        return metric(
            False,
            ["containment_preflight_blocked_collision_loading", *containment_preflight["issues"]],
            denominator_links=len(links),
            covered_links=0,
            collision_count=sum(len(children(link, "collision")) for link in links),
        )
    issues: list[str] = []
    covered_links = 0
    collision_count = 0
    for index, link in enumerate(links):
        name = link.attrib.get("name", f"index_{index}")
        collision_nodes = children(link, "collision")
        if not collision_nodes:
            issues.append(f"collision_missing: {name}")
            continue
        link_pass = True
        for collision_index, collision in enumerate(collision_nodes):
            collision_count += 1
            geometry_nodes = children(collision, "geometry")
            if len(geometry_nodes) != 1:
                link_pass = False
                issues.append(f"collision_geometry_cardinality: {name}:{collision_index}")
                continue
            loadable, error = loadable_collision_geometry(
                geometry_nodes[0], package, urdf_path, containment_preflight
            )
            if not loadable:
                link_pass = False
                issues.append(f"collision_unloadable: {name}:{collision_index}: {error}")
        if link_pass:
            covered_links += 1
    return metric(
        bool(links) and not issues,
        issues if links else ["no_declared_links"],
        denominator_links=len(links),
        covered_links=covered_links,
        collision_count=collision_count,
    )


def inertial_coverage_audit(root: ET.Element) -> dict[str, Any]:
    links = children(root, "link")
    missing: list[str] = []
    for index, link in enumerate(links):
        name = link.attrib.get("name", f"index_{index}")
        inertials = children(link, "inertial")
        inertial = inertials[0] if len(inertials) == 1 else None
        complete = (
            inertial is not None
            and len(children(inertial, "mass")) == 1
            and len(children(inertial, "inertia")) == 1
        )
        if not complete:
            missing.append(f"inertial_incomplete: {name}")
    return metric(
        bool(links) and not missing,
        missing if links else ["no_declared_links"],
        denominator_links=len(links),
        covered_links=len(links) - len(missing),
    )


def inertia_validity_audit(root: ET.Element) -> dict[str, Any]:
    links = children(root, "link")
    issues: list[str] = []
    valid_links = 0
    evidence: list[dict[str, Any]] = []
    for index, link in enumerate(links):
        name = link.attrib.get("name", f"index_{index}")
        inertials = children(link, "inertial")
        if len(inertials) != 1:
            issues.append(f"{name}: inertial_cardinality_{len(inertials)}")
            continue
        inertial = inertials[0]
        origin = first_child(inertial, "origin")
        mass_nodes = children(inertial, "mass")
        tensor_nodes = children(inertial, "inertia")
        mass_node = mass_nodes[0] if len(mass_nodes) == 1 else None
        tensor_node = tensor_nodes[0] if len(tensor_nodes) == 1 else None
        link_issues: list[str] = []
        if origin is not None and finite_vector(origin.attrib.get("xyz", "0 0 0")) is None:
            link_issues.append("com_invalid")
        mass = finite_scalar(mass_node.attrib.get("value")) if mass_node is not None else None
        if mass is None or mass <= 0:
            link_issues.append("mass_not_positive")
        attributes = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        values = (
            {attribute: finite_scalar(tensor_node.attrib.get(attribute)) for attribute in attributes}
            if tensor_node is not None
            else {attribute: None for attribute in attributes}
        )
        if any(value is None for value in values.values()):
            link_issues.append("inertia_tensor_invalid")
        else:
            tensor = np.array(
                [
                    [values["ixx"], values["ixy"], values["ixz"]],
                    [values["ixy"], values["iyy"], values["iyz"]],
                    [values["ixz"], values["iyz"], values["izz"]],
                ],
                dtype=np.float64,
            )
            eigenvalues = np.linalg.eigvalsh(tensor)
            largest = float(eigenvalues[-1])
            smallest = float(eigenvalues[0])
            tolerance = INERTIA_RELATIVE_TOLERANCE * largest
            condition = largest / smallest if smallest > 0 else None
            evidence.append({
                "link": name,
                "eigenvalues": [float(value) for value in eigenvalues],
                "tau": tolerance,
                "condition_number": condition,
            })
            if smallest <= tolerance:
                link_issues.append("inertia_not_positive_definite")
            elif largest > float(eigenvalues[0] + eigenvalues[1]) + tolerance:
                link_issues.append("principal_inertia_triangle_inequality")
        if link_issues:
            issues.extend(f"{name}: {issue}" for issue in link_issues)
        else:
            valid_links += 1
    return metric(
        bool(links) and not issues,
        issues if links else ["no_declared_links"],
        denominator_links=len(links),
        valid_links=valid_links,
        link_evidence=evidence,
    )


def safe_package_relative_path(raw: str, *, field: str) -> PurePosixPath:
    if not isinstance(raw, str) or not raw or "\\" in raw:
        raise ValueError(f"invalid {field}: {raw!r}")
    relative = PurePosixPath(raw)
    if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
        raise ValueError(f"invalid {field}: {raw!r}")
    if relative.as_posix() != raw or PureWindowsPath(raw).is_absolute():
        raise ValueError(f"non-canonical {field}: {raw!r}")
    return relative


def primary_urdf_path(package: Path, relative_raw: str) -> Path:
    relative = safe_package_relative_path(relative_raw, field="primary URDF relative path")
    candidate = package.joinpath(*relative.parts)
    if candidate.is_symlink() or not candidate.is_file():
        raise FileNotFoundError(candidate)
    resolved = candidate.resolve(strict=True)
    try:
        resolved.relative_to(package.resolve(strict=True))
    except ValueError as exc:
        raise RuntimeError(f"primary URDF escapes package: {relative_raw}") from exc
    return resolved


def audit_asset_package(
    package: Path | str,
    run_standard_parser: bool = True,
    *,
    asset_id: str | None = None,
    primary_urdf_relative_path: str = "model.urdf",
) -> dict[str, Any]:
    """Audit one immutable package and return all nine asset-level metric states."""

    package_path = Path(package)
    resolved_asset_id = package_path.name if asset_id is None else asset_id
    try:
        package_resolved = package_path.resolve(strict=True)
        if not package_resolved.is_dir():
            raise NotADirectoryError(package_resolved)
        urdf_path = primary_urdf_path(package_resolved, primary_urdf_relative_path)
        try:
            root = ET.parse(urdf_path).getroot()
            xml_issues: list[str] = []
            xml_pass = local_tag(root) == "robot"
            if not xml_pass:
                xml_issues.append(f"root_element_is_not_robot: {local_tag(root)}")
        except Exception as exc:  # noqa: BLE001
            return failed_record(
                resolved_asset_id,
                package_resolved,
                f"xml_parse_failed: {type(exc).__name__}: {exc}",
                primary_urdf_relative_path=primary_urdf_relative_path,
            )

        containment = package_resource_containment_preflight(
            root, package_resolved, urdf_path
        )
        if containment["pass"]:
            parser_pass, parser_issues, parser_details = standard_parser_result(
                urdf_path, run_standard_parser
            )
            parse_result = metric(
                xml_pass and parser_pass, xml_issues + parser_issues, **parser_details
            )
            resources = resource_audit(root, package_resolved, urdf_path)
        else:
            try:
                parser_version = importlib.metadata.version("urdfpy")
            except importlib.metadata.PackageNotFoundError:
                parser_version = None
            parse_result = metric(
                False,
                [*xml_issues, "containment_preflight_failed_before_standard_parser"],
                enabled=run_standard_parser,
                parser="urdfpy",
                version=parser_version,
                scope=EVALUATOR_CONFIG["parse_rate_scope"],
                standard_parser_attempted=False,
                containment_preflight_issues=containment["issues"],
            )
            resources = metric(
                False,
                containment["issues"],
                containment_preflight_pass=False,
                references=[],
                reference_count=0,
            )
        metrics = {
            "parse_rate": parse_result,
            "resource_resolution": resources,
            "finite_fields": finite_fields_audit(root),
            "valid_tree": valid_tree_audit(root),
            "valid_joint_spec": valid_joint_spec_audit(root),
            "collision_coverage": collision_coverage_audit(
                root, package_resolved, urdf_path, containment
            ),
            "inertial_coverage": inertial_coverage_audit(root),
            "inertia_validity": inertia_validity_audit(root),
        }
        strict = all(result["pass"] for result in metrics.values())
        metrics["strict_urdf_pass"] = metric(strict, [] if strict else [
            "one_or_more_table2_metrics_failed"
        ])
        return {
            "asset_id": resolved_asset_id,
            "package": str(package_resolved),
            "primary_urdf_relative_path": primary_urdf_relative_path,
            "status": "completed",
            "error": None,
            "metrics": metrics,
            "strict_urdf_pass": strict,
        }
    except Exception as exc:  # noqa: BLE001
        return failed_record(
            resolved_asset_id,
            package_path,
            f"audit_exception: {type(exc).__name__}: {exc}",
            primary_urdf_relative_path=primary_urdf_relative_path,
        )


def aggregate_records(
    records: Iterable[dict[str, Any]],
    expected_n: int,
    category_field: str | None = None,
) -> dict[str, Any]:
    rows = list(records)
    if expected_n < len(rows):
        raise ValueError("expected_n cannot be smaller than the record count")
    missing = expected_n - len(rows)
    metrics: dict[str, dict[str, Any]] = {}
    for name in METRIC_NAMES:
        passed = sum(row.get("metrics", {}).get(name, {}).get("pass") is True for row in rows)
        metrics[name] = {
            "passed": passed,
            "denominator": expected_n,
            "rate": passed / expected_n if expected_n else None,
        }
    strict_passed = sum(
        row.get("metrics", {}).get("strict_urdf_pass", {}).get("pass") is True for row in rows
    )
    status_counts = Counter(str(row.get("status", "unknown")) for row in rows)
    if category_field is None:
        category_macro: dict[str, Any] = {
            "state": "not_evaluable",
            "reason": "Articraft-10K release has no authoritative category labels",
        }
    else:
        grouped: dict[str, list[dict[str, Any]]] = {}
        for row in rows:
            category = row.get(category_field)
            if not isinstance(category, str) or not category:
                raise ValueError(f"record lacks non-empty {category_field}: {row.get('asset_id')!r}")
            grouped.setdefault(category, []).append(row)
        categories: dict[str, Any] = {}
        for category, category_rows in sorted(grouped.items()):
            denominator = len(category_rows)
            categories[category] = {
                "denominator": denominator,
                "metrics": {
                    name: {
                        "passed": sum(
                            row.get("metrics", {}).get(name, {}).get("pass") is True
                            for row in category_rows
                        ),
                        "denominator": denominator,
                        "rate": sum(
                            row.get("metrics", {}).get(name, {}).get("pass") is True
                            for row in category_rows
                        ) / denominator,
                    }
                    for name in METRIC_NAMES
                },
            }
        category_macro = {
            "state": "evaluated",
            "category_field": category_field,
            "category_count": len(categories),
            "denominator_policy": (
                f"unweighted mean of per-{category_field} asset rates; all frozen assets and "
                "failures retained"
            ),
            "metrics": {
                name: {
                    "rate": (
                        sum(row["metrics"][name]["rate"] for row in categories.values())
                        / len(categories)
                        if categories else None
                    ),
                    "category_denominator": len(categories),
                }
                for name in METRIC_NAMES
            },
            "categories": categories,
        }
    return {
        "n_eval": expected_n,
        "records_present": len(rows),
        "records_missing_counted_as_failures": missing,
        "error_count": sum(row.get("status") in {"error", "timeout"} for row in rows) + missing,
        "status_counts": dict(sorted(status_counts.items())),
        "metrics": metrics,
        "strict_urdf_pass": {
            "passed": strict_passed,
            "denominator": expected_n,
            "rate": strict_passed / expected_n if expected_n else None,
        },
        "category_macro": category_macro,
    }


def load_inventory_entry() -> tuple[dict[str, Any], str]:
    payload = json.loads(INVENTORY_PATH.read_text(encoding="utf-8"))
    for entry in payload.get("datasets", []):
        if entry.get("name") == "Articraft-10K":
            return entry, sha256_file(INVENTORY_PATH)
    raise RuntimeError("Articraft-10K is absent from dataset_inventory.json")


def release_asset_ids(
    source_root: Path, reject_non_directories: bool = False
) -> list[str]:
    source_root = source_root.resolve(strict=True)
    if not source_root.is_dir():
        raise FileNotFoundError(source_root)
    ids: list[str] = []
    for path in sorted(source_root.iterdir(), key=lambda child: child.name):
        if path.is_symlink():
            raise RuntimeError(f"release child symlink is forbidden: {path.name}")
        if not path.is_dir():
            if reject_non_directories:
                raise RuntimeError(f"release child is a non-directory: {path.name}")
            continue
        canonical = path.resolve(strict=True)
        try:
            canonical.relative_to(source_root)
        except ValueError as exc:
            raise RuntimeError(f"release package escapes source root: {path.name}") from exc
        model = path / "model.urdf"
        if model.is_symlink() or not model.is_file():
            raise RuntimeError(f"package model.urdf must be a regular non-symlink file: {path.name}")
        ids.append(path.name)
    if len(ids) != len(set(ids)):
        raise RuntimeError("release contains duplicate asset IDs")
    return ids


def _artiverse_manifest_identity(raw: Any, *, field: str) -> tuple[str, str, str, str]:
    relative = safe_package_relative_path(str(raw), field=field)
    if len(relative.parts) != 4 or relative.parts[0] != "data":
        raise ValueError(f"invalid {field}: {raw!r}")
    return relative.parts  # type: ignore[return-value]


def _reject_symlink_path(root: Path, relative: PurePosixPath, *, field: str) -> Path:
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise RuntimeError(f"{field} contains a symlink: {relative.as_posix()}")
    resolved = current.resolve(strict=True)
    try:
        resolved.relative_to(root.resolve(strict=True))
    except ValueError as exc:
        raise RuntimeError(f"{field} escapes source root: {relative.as_posix()}") from exc
    return resolved


def load_artiverse_release_manifest(
    source_root: Path, release_manifest_path: Path
) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    release = json.loads(release_manifest_path.read_text(encoding="utf-8"))
    if release.get("format") != "artiverse-data-tar-gz-chunks-v1":
        raise ValueError("unexpected Artiverse release manifest format")
    chunks = release.get("chunks")
    if not isinstance(chunks, list) or len(chunks) != release.get("chunk_count"):
        raise ValueError("Artiverse release manifest chunk count mismatch")
    identities: dict[str, dict[str, Any]] = {}
    for chunk in chunks:
        if not isinstance(chunk, dict):
            raise ValueError("Artiverse release manifest chunk is not an object")
        roots = chunk.get("roots")
        if not isinstance(roots, list) or len(roots) != chunk.get("model_count"):
            raise ValueError("Artiverse release manifest root count mismatch")
        archive = str(chunk.get("archive", ""))
        for raw_root in roots:
            _data, category, upstream_source, model_id = _artiverse_manifest_identity(
                raw_root, field="release manifest root"
            )
            root = str(raw_root)
            if root in identities:
                raise ValueError(f"duplicate release manifest root: {root}")
            identities[root] = {
                "asset_id": root,
                "manifest_root": root,
                "raw_category": category,
                "source": upstream_source,
                "model_id": model_id,
                "chunk_archive": archive,
            }
    if len(identities) != release.get("model_count"):
        raise ValueError("Artiverse release manifest model count mismatch")
    return release, identities


def load_artiverse_cohort(
    source_root: Path,
    cohort_manifest_path: Path,
    *,
    formal: bool,
) -> dict[str, Any]:
    source_root = source_root.resolve(strict=True)
    if not source_root.is_dir():
        raise NotADirectoryError(source_root)
    cohort_manifest_path = cohort_manifest_path.resolve(strict=True)
    cohort_hash = sha256_file(cohort_manifest_path)
    cohort = json.loads(cohort_manifest_path.read_text(encoding="utf-8"))
    if not isinstance(cohort, dict) or cohort.get("dataset") != "Artiverse":
        raise ValueError("cohort manifest dataset must be Artiverse")
    if cohort.get("release_status") != "PRE_RELEASE_SUBSET":
        raise ValueError("Artiverse cohort must identify the PRE_RELEASE_SUBSET release")
    if cohort.get("cohort_type") != "GLOBAL_FIXED_SAMPLE_NOT_CATEGORY_BALANCED":
        raise ValueError("unexpected Artiverse cohort type")
    if str(cohort.get("seed")) != str(DEFAULT_SEED):
        raise ValueError(f"Artiverse cohort seed must be {DEFAULT_SEED}")
    if cohort.get("selection_protocol") != ARTIVERSE_SELECTION_PROTOCOL:
        raise ValueError("unexpected Artiverse selection protocol")
    if cohort.get("release_manifest") != "dataset_chunks/manifest.json":
        raise ValueError("unexpected Artiverse release manifest path")
    release_path = source_root / "dataset_chunks/manifest.json"
    if release_path.is_symlink() or not release_path.is_file():
        raise RuntimeError("Artiverse release manifest must be a regular non-symlink file")
    release_hash = sha256_file(release_path)
    if cohort.get("release_manifest_sha256") != release_hash:
        raise ValueError("Artiverse cohort release manifest SHA-256 mismatch")
    release, release_identities = load_artiverse_release_manifest(source_root, release_path)
    if cohort.get("N_release") != len(release_identities):
        raise ValueError("Artiverse cohort N_release mismatch")
    universe_hash = hashlib.sha256(
        "".join(f"{asset_id}\n" for asset_id in sorted(release_identities)).encode("utf-8")
    ).hexdigest()
    if cohort.get("release_universe_sha256") != universe_hash:
        raise ValueError("Artiverse release universe SHA-256 mismatch")
    assets = cohort.get("assets")
    if not isinstance(assets, list) or cohort.get("N_eval") != len(assets):
        raise ValueError("Artiverse cohort N_eval mismatch")
    if formal:
        formal_expectations = {
            "cohort_manifest_sha256": (
                cohort_hash,
                ARTIVERSE_COHORT_MANIFEST_SHA256,
            ),
            "release_manifest_sha256": (
                release_hash,
                ARTIVERSE_RELEASE_MANIFEST_SHA256,
            ),
            "N_release": (len(release_identities), ARTIVERSE_FORMAL_RELEASE_COUNT),
            "N_eval": (len(assets), DEFAULT_N),
        }
        for name, (observed, expected) in formal_expectations.items():
            if observed != expected:
                raise RuntimeError(
                    f"formal Artiverse {name} mismatch: {observed!r} != {expected!r}"
                )
    seen_ids: set[str] = set()
    seen_hashes: set[str] = set()
    frozen_assets: list[dict[str, Any]] = []
    for expected_rank, raw in enumerate(assets, start=1):
        if not isinstance(raw, dict):
            raise ValueError("Artiverse cohort asset is not an object")
        manifest_root = raw.get("manifest_root")
        asset_id = raw.get("asset_id")
        if asset_id != manifest_root:
            raise ValueError("Artiverse asset_id must equal manifest root")
        _data, category, upstream_source, model_id = _artiverse_manifest_identity(
            manifest_root, field="manifest root"
        )
        if asset_id in seen_ids:
            raise ValueError(f"duplicate Artiverse asset ID: {asset_id}")
        seen_ids.add(str(asset_id))
        if (
            raw.get("raw_category") != category
            or raw.get("source") != upstream_source
            or raw.get("model_id") != model_id
        ):
            raise ValueError(f"Artiverse path-field inconsistency: {manifest_root}")
        if raw.get("selection_rank") != expected_rank:
            raise ValueError(f"Artiverse selection rank mismatch at {manifest_root}")
        selection_hash = raw.get("selection_hash")
        expected_hash = hashlib.sha256(
            "\0".join(
                (ARTIVERSE_SELECTION_PROTOCOL, release_hash, str(DEFAULT_SEED), str(asset_id))
            ).encode("utf-8")
        ).hexdigest()
        if selection_hash != expected_hash:
            raise ValueError(f"Artiverse selection hash mismatch at {manifest_root}")
        if selection_hash in seen_hashes:
            raise ValueError(f"duplicate Artiverse selection hash: {selection_hash}")
        seen_hashes.add(str(selection_hash))
        release_identity = release_identities.get(str(manifest_root))
        if release_identity is None:
            raise ValueError(f"Artiverse manifest root is absent from release: {manifest_root}")
        for field in ("raw_category", "source", "model_id", "chunk_archive"):
            if raw.get(field) != release_identity[field]:
                raise ValueError(f"Artiverse release identity mismatch for {field}: {manifest_root}")
        relative = safe_package_relative_path(str(manifest_root), field="manifest root")
        model_root = _reject_symlink_path(source_root, relative, field="manifest root")
        if not model_root.is_dir():
            raise RuntimeError(f"Artiverse manifest root is not a directory: {manifest_root}")
        package_raw = model_root / "urdf_w_collider"
        if package_raw.is_symlink() or not package_raw.is_dir():
            raise RuntimeError(f"Artiverse package must be a non-symlink directory: {manifest_root}")
        package = package_raw.resolve(strict=True)
        try:
            package.relative_to(source_root)
        except ValueError as exc:
            raise RuntimeError(f"Artiverse package escapes source root: {manifest_root}") from exc
        urdf_candidates = sorted(package.glob("*.urdf"), key=lambda path: path.name)
        if len(urdf_candidates) != 1:
            raise RuntimeError(
                f"Artiverse package must contain exactly one top-level URDF: {manifest_root}"
            )
        primary = urdf_candidates[0]
        if primary.is_symlink() or not primary.is_file():
            raise RuntimeError(
                f"Artiverse primary URDF must be a regular non-symlink file: {manifest_root}"
            )
        if primary.name != f"{model_id}.urdf":
            raise RuntimeError(f"Artiverse primary URDF filename mismatch: {manifest_root}")
        frozen_assets.append(
            {
                **{field: raw[field] for field in ARTIVERSE_METADATA_FIELDS},
                "asset_id": str(asset_id),
                "package": str(package),
                "primary_urdf_relative_path": primary.name,
            }
        )
    expected_order = sorted(
        frozen_assets, key=lambda row: (row["selection_hash"], row["asset_id"])
    )
    if [row["asset_id"] for row in frozen_assets] != [
        row["asset_id"] for row in expected_order
    ]:
        raise ValueError("Artiverse selection order does not match frozen selection hashes")
    if formal and len({row["raw_category"] for row in frozen_assets}) != ARTIVERSE_FORMAL_CATEGORY_COUNT:
        raise RuntimeError("formal Artiverse cohort raw-category count is not exactly 67")
    return {
        "source_root": str(source_root),
        "cohort_manifest_path": str(cohort_manifest_path),
        "cohort_manifest_sha256": cohort_hash,
        "release_manifest_path": str(release_path.resolve(strict=True)),
        "release_manifest_sha256": release_hash,
        "release_manifest": release,
        "release_universe_sha256": universe_hash,
        "release_status": cohort["release_status"],
        "n_release": len(release_identities),
        "cohort_asset_count": len(frozen_assets),
        "seed": str(cohort["seed"]),
        "selection_protocol": cohort["selection_protocol"],
        "assets": frozen_assets,
    }


def _partnet_mobility_inventory_entry() -> tuple[dict[str, Any], str]:
    payload = json.loads(INVENTORY_PATH.read_text(encoding="utf-8"))
    for entry in payload.get("datasets", []):
        if entry.get("name") == "PartNet-Mobility":
            return entry, sha256_file(INVENTORY_PATH)
    raise RuntimeError("PartNet-Mobility is absent from dataset_inventory.json")


def partnet_mobility_checked_identity_path(path: Path, *, field: str) -> Path:
    """Make an existing identity path absolute without following symlink components."""

    candidate = Path(os.path.abspath(path))
    current = Path(candidate.anchor)
    for part in candidate.parts[1:]:
        current /= part
        if current.is_symlink():
            raise RuntimeError(f"PartNet-Mobility {field} contains a symlink: {candidate}")
    return candidate


def partnet_mobility_source_path(
    package: Path,
    raw: str,
    *,
    field: str,
    required_kind: str | None,
) -> Path:
    """Contain a PartNet package path before any stat, read, or hash operation."""

    normalized = raw.replace("\\", "/")
    relative = PurePosixPath(normalized)
    if (
        not normalized
        or URI_SCHEME.match(normalized)
        or relative.is_absolute()
        or PureWindowsPath(raw).is_absolute()
        or any(part in {"", ".", ".."} for part in relative.parts)
    ):
        raise RuntimeError(f"PartNet-Mobility {field} is not a contained relative path: {raw!r}")
    candidate = package
    for part in relative.parts:
        candidate /= part
        if candidate.is_symlink():
            raise RuntimeError(f"PartNet-Mobility {field} contains a symlink: {raw!r}")
    try:
        candidate.relative_to(package)
    except ValueError as exc:
        raise RuntimeError(f"PartNet-Mobility {field} escapes package: {raw!r}") from exc
    if required_kind == "file" and not candidate.is_file():
        raise RuntimeError(f"PartNet-Mobility {field} must be a regular file: {raw!r}")
    if required_kind == "directory" and not candidate.is_dir():
        raise RuntimeError(f"PartNet-Mobility {field} must be a directory: {raw!r}")
    return candidate


def _partnet_mobility_release_ids(source_root: Path) -> list[str]:
    ids: list[str] = []
    for child in source_root.iterdir():
        if child.is_symlink():
            raise RuntimeError(f"PartNet-Mobility release child symlink is forbidden: {child.name}")
        if not child.is_dir():
            raise RuntimeError(f"PartNet-Mobility release child is not a directory: {child.name}")
        if not child.name.isdecimal():
            raise RuntimeError(f"PartNet-Mobility release ID is not numeric: {child.name}")
        canonical = child.resolve(strict=True)
        try:
            canonical.relative_to(source_root)
        except ValueError as exc:
            raise RuntimeError(f"PartNet-Mobility package escapes source root: {child.name}") from exc
        ids.append(child.name)
    return sorted(ids, key=int)


def partnet_mobility_selection_digest(dataset_id: str) -> str:
    return hashlib.sha256(
        f"{PARTNET_MOBILITY_SELECTION_SALT}\0{dataset_id}".encode("utf-8")
    ).hexdigest()


def partnet_mobility_collision_mesh_inventory(
    asset_dir: Path, urdf_path: Path
) -> list[dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    references = sorted({
        mesh.get("filename", "").replace("\\", "/")
        for mesh in root.findall("link/collision/geometry/mesh")
        if mesh.get("filename")
    })
    inventory = []
    for reference in references:
        path = partnet_mobility_source_path(
            asset_dir, reference, field="collision mesh reference", required_kind=None
        )
        exists = path.is_file()
        inventory.append({
            "path": reference,
            "exists": exists,
            "size_bytes": path.stat().st_size if exists else None,
            "sha256": sha256_file(path) if exists else None,
        })
    return inventory


def partnet_mobility_input_identity(item: dict[str, Any]) -> str:
    return canonical_sha256({
        key: item[key]
        for key in (
            "protocol_id", "order", "dataset_id", "selection_digest", "category",
            "movable_dof_count", "range_evaluable_dof_count", "joint_specs_sha256",
            "runtime_identity_sha256", "urdf_sha256", "bounding_box_sha256",
            "collision_mesh_inventory_sha256", "object_bbox_diagonal_m",
            "rest_state_expected", "single_state_expected", "sobol_state_expected",
        )
    })


def load_partnet_mobility_cohort(
    source_root: Path,
    cohort_manifest_path: Path,
    *,
    formal: bool,
) -> dict[str, Any]:
    """Load the Table 4 frozen PartNet-Mobility cohort without reselection."""

    source_root = partnet_mobility_checked_identity_path(source_root, field="source root")
    source_root = source_root.resolve(strict=True)
    if not source_root.is_dir():
        raise NotADirectoryError(source_root)
    cohort_manifest_path = partnet_mobility_checked_identity_path(
        cohort_manifest_path, field="cohort manifest"
    )
    if not cohort_manifest_path.is_file():
        raise RuntimeError("PartNet-Mobility cohort manifest must be a regular file")
    cohort_manifest_path = cohort_manifest_path.resolve(strict=True)
    cohort_hash = sha256_file(cohort_manifest_path)
    cohort = json.loads(cohort_manifest_path.read_text(encoding="utf-8"))
    if not isinstance(cohort, dict) or cohort.get("status") != "FROZEN":
        raise ValueError("PartNet-Mobility cohort status must be FROZEN")
    if cohort.get("protocol_id") != PARTNET_MOBILITY_SELECTION_PROTOCOL:
        raise ValueError("unexpected PartNet-Mobility protocol")
    selection_policy = cohort.get("selection_policy")
    if not isinstance(selection_policy, dict):
        raise ValueError("PartNet-Mobility selection policy is missing")
    if selection_policy.get("algorithm") != PARTNET_MOBILITY_SELECTION_ALGORITHM:
        raise ValueError("unexpected PartNet-Mobility selection algorithm")
    if selection_policy.get("salt") != PARTNET_MOBILITY_SELECTION_SALT:
        raise ValueError("unexpected PartNet-Mobility selection salt")
    if selection_policy.get("outcome_based_filtering") is not False:
        raise ValueError("PartNet-Mobility selection policy permits outcome filtering")
    if selection_policy.get("selected_failures_retained_without_replacement") is not True:
        raise ValueError("PartNet-Mobility selection policy does not retain failures")
    items = cohort.get("items")
    if not isinstance(items, list) or cohort.get("sample_size") != len(items):
        raise ValueError("PartNet-Mobility cohort selected item count mismatch")
    if canonical_sha256(items) != cohort.get("items_sha256"):
        raise ValueError("PartNet-Mobility items SHA-256 mismatch")
    release_ids = _partnet_mobility_release_ids(source_root)
    if cohort.get("release_asset_count") != len(release_ids):
        raise RuntimeError("PartNet-Mobility release asset count mismatch")
    pool_hash = canonical_sha256(release_ids)
    if cohort.get("candidate_pool_identity_sha256") != pool_hash:
        raise RuntimeError("PartNet-Mobility candidate pool identity mismatch")
    archive_payload = cohort.get("archive")
    if not isinstance(archive_payload, dict):
        raise ValueError("PartNet-Mobility archive identity is missing")
    archive_path = partnet_mobility_checked_identity_path(
        Path(str(archive_payload.get("path", ""))), field="archive"
    )
    if not archive_path.is_file():
        raise RuntimeError("PartNet-Mobility archive must be a regular non-symlink file")
    archive_path = archive_path.resolve(strict=True)
    if archive_path.stat().st_size != archive_payload.get("size_bytes"):
        raise RuntimeError("PartNet-Mobility archive size mismatch")
    archive_sha256 = sha256_file(archive_path)
    if archive_sha256 != archive_payload.get("sha256"):
        raise RuntimeError("PartNet-Mobility archive SHA-256 mismatch")
    if formal:
        inventory, inventory_hash = _partnet_mobility_inventory_entry()
        source = inventory.get("source", {})
        verification = inventory.get("verification", {})
        formal_expectations = {
            "cohort_manifest_sha256": (cohort_hash, PARTNET_MOBILITY_COHORT_MANIFEST_SHA256),
            "sample_size": (len(items), DEFAULT_N),
            "release_asset_count": (len(release_ids), PARTNET_MOBILITY_FORMAL_RELEASE_COUNT),
            "candidate_pool_identity_sha256": (pool_hash, PARTNET_MOBILITY_CANDIDATE_POOL_SHA256),
            "items_sha256": (cohort.get("items_sha256"), PARTNET_MOBILITY_ITEMS_SHA256),
            "ordered_selected_ids_sha256": (
                cohort.get("ordered_selected_ids_sha256"), PARTNET_MOBILITY_SELECTED_IDS_SHA256,
            ),
            "archive_bytes": (archive_path.stat().st_size, PARTNET_MOBILITY_ARCHIVE_BYTES),
            "archive_sha256": (archive_sha256, PARTNET_MOBILITY_ARCHIVE_SHA256),
            "inventory_status": (inventory.get("status"), "LOCAL_COMPLETE_PROVENANCE_LIMITED"),
            "inventory_revision": (source.get("revision"), "ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f"),
            "inventory_local_ids": (verification.get("local_ids_listed_by_fixed_hf_revision"), 2347),
            "inventory_root": (
                partnet_mobility_checked_identity_path(
                    Path(str(inventory.get("urdf_root", ""))), field="inventory source root"
                ).resolve(strict=False),
                source_root,
            ),
        }
        for name, (observed, expected) in formal_expectations.items():
            if observed != expected:
                raise RuntimeError(f"formal PartNet-Mobility {name} mismatch: {observed!r} != {expected!r}")
        if source_root != partnet_mobility_checked_identity_path(
            DEFAULT_PARTNET_MOBILITY_SOURCE_ROOT, field="formal source root"
        ).resolve(strict=False):
            raise RuntimeError("formal PartNet-Mobility source root mismatch")
        if archive_path != partnet_mobility_checked_identity_path(
            Path(inventory["archive"]["path"]), field="inventory archive"
        ).resolve(strict=True):
            raise RuntimeError("formal PartNet-Mobility archive path mismatch")
    else:
        inventory = None
        inventory_hash = None
    seen_ids: set[str] = set()
    frozen_assets: list[dict[str, Any]] = []
    for order, raw in enumerate(items):
        if not isinstance(raw, dict):
            raise ValueError("PartNet-Mobility item is not an object")
        if raw.get("order") != order:
            raise ValueError(f"PartNet-Mobility item order mismatch at {order}")
        dataset_id = raw.get("dataset_id")
        if not isinstance(dataset_id, str) or not dataset_id.isdecimal():
            raise ValueError("PartNet-Mobility dataset_id must be numeric")
        if dataset_id in seen_ids:
            raise ValueError(f"duplicate PartNet-Mobility dataset_id: {dataset_id}")
        seen_ids.add(dataset_id)
        if dataset_id not in release_ids:
            raise ValueError(f"PartNet-Mobility item is absent from live pool: {dataset_id}")
        if "package" in raw:
            raise ValueError("PartNet-Mobility item must not override package path")
        category = raw.get("category")
        if not isinstance(category, str) or not category:
            raise ValueError(f"PartNet-Mobility category is invalid: {dataset_id}")
        if raw.get("protocol_id") != PARTNET_MOBILITY_SELECTION_PROTOCOL:
            raise ValueError(f"PartNet-Mobility item protocol mismatch: {dataset_id}")
        if raw.get("selection_digest") != partnet_mobility_selection_digest(dataset_id):
            raise ValueError(f"PartNet-Mobility selection digest mismatch: {dataset_id}")
        try:
            if partnet_mobility_input_identity(raw) != raw.get("input_identity_sha256"):
                raise ValueError(f"PartNet-Mobility input identity mismatch: {dataset_id}")
        except KeyError as exc:
            raise ValueError(
                f"PartNet-Mobility input identity field is missing: {exc.args[0]}"
            ) from exc
        package = partnet_mobility_source_path(
            source_root, dataset_id, field="package", required_kind="directory"
        )
        primary = partnet_mobility_source_path(
            package, "mobility.urdf", field="mobility.urdf", required_kind="file"
        )
        primary_hash = sha256_file(primary)
        if raw.get("urdf_sha256") != primary_hash:
            raise RuntimeError(f"PartNet-Mobility primary URDF SHA-256 mismatch: {dataset_id}")
        bounding_box_hash = raw.get("bounding_box_sha256")
        if bounding_box_hash is not None:
            bounding_box = partnet_mobility_source_path(
                package, "bounding_box.json", field="bounding-box", required_kind="file"
            )
            if sha256_file(bounding_box) != bounding_box_hash:
                raise RuntimeError(f"PartNet-Mobility bounding-box content drift: {dataset_id}")
        observed_collision_inventory = partnet_mobility_collision_mesh_inventory(package, primary)
        if (
            observed_collision_inventory != raw.get("collision_mesh_files")
            or canonical_sha256(observed_collision_inventory)
            != raw.get("collision_mesh_inventory_sha256")
        ):
            raise RuntimeError(f"PartNet-Mobility collision mesh inventory drift: {dataset_id}")
        meta_path = partnet_mobility_source_path(
            package, "meta.json", field="meta.json", required_kind="file"
        )
        try:
            live_category = json.loads(meta_path.read_text(encoding="utf-8")).get("model_cat")
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f"PartNet-Mobility meta.json is unreadable: {dataset_id}") from exc
        if live_category != category:
            raise RuntimeError(f"PartNet-Mobility meta.json category drift: {dataset_id}")
        frozen_assets.append({
            "selection_index": order,
            "asset_id": dataset_id,
            "dataset_id": dataset_id,
            "category": category,
            "order": order,
            "protocol_id": raw["protocol_id"],
            "input_identity_sha256": raw.get("input_identity_sha256"),
            "selection_digest": raw.get("selection_digest"),
            "frozen_item_sha256": canonical_sha256(raw),
            "source_item": raw,
            "package": str(package),
            "primary_urdf_relative_path": "mobility.urdf",
        })
    selected_ids = [row["asset_id"] for row in frozen_assets]
    if canonical_sha256(selected_ids) != cohort.get("ordered_selected_ids_sha256"):
        raise ValueError("PartNet-Mobility ordered selected ID hash mismatch")
    if formal and len({row["category"] for row in frozen_assets}) != PARTNET_MOBILITY_FORMAL_CATEGORY_COUNT:
        raise RuntimeError("formal PartNet-Mobility category count is not exactly 46")
    return {
        "source_root": str(source_root),
        "cohort_manifest_path": str(cohort_manifest_path),
        "cohort_manifest_sha256": cohort_hash,
        "items_sha256": cohort["items_sha256"],
        "ordered_selected_ids_sha256": cohort["ordered_selected_ids_sha256"],
        "candidate_pool_identity_sha256": pool_hash,
        "release_asset_count": len(release_ids),
        "archive": {"path": str(archive_path), "bytes": archive_path.stat().st_size, "sha256": archive_sha256},
        "inventory": inventory,
        "inventory_sha256": inventory_hash,
        "selection_policy": selection_policy,
        "assets": frozen_assets,
    }


def lam_checked_identity_path(path: Path, *, field: str) -> Path:
    """Return an absolute LAM identity path without following any symlink component."""

    candidate = Path(os.path.abspath(path))
    current = Path(candidate.anchor)
    for part in candidate.parts[1:]:
        current /= part
        if current.is_symlink():
            raise RuntimeError(f"LAM {field} contains a symlink: {candidate}")
    return candidate


def lam_source_path(
    root: Path,
    raw: str,
    *,
    field: str,
    required_kind: str,
) -> Path:
    """Validate a contained LAM path component by component before reading it."""

    try:
        relative = safe_package_relative_path(raw, field=f"LAM {field}")
    except ValueError as exc:
        raise RuntimeError(f"LAM {field} is not a contained relative path: {raw!r}") from exc
    candidate = root
    for part in relative.parts:
        candidate /= part
        if candidate.is_symlink():
            raise RuntimeError(f"LAM {field} contains a symlink: {raw!r}")
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise RuntimeError(f"LAM {field} escapes source root: {raw!r}") from exc
    if required_kind == "directory" and not candidate.is_dir():
        raise RuntimeError(f"LAM {field} must be a directory: {raw!r}")
    if required_kind == "file" and not candidate.is_file():
        raise RuntimeError(f"LAM {field} must be a regular file: {raw!r}")
    return candidate


def _lam_json_object(payload: bytes, *, field: str) -> dict[str, Any]:
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"LAM {field} is not valid UTF-8 JSON") from exc
    if not isinstance(value, dict):
        raise RuntimeError(f"LAM {field} must contain one JSON object")
    return value


def _lam_regular_file_identity(path: Path, *, field: str) -> dict[str, Any]:
    """Stream one regular file through a no-follow descriptor."""

    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise RuntimeError(f"LAM {field} must be a readable regular non-symlink file") from exc
    digest = hashlib.sha256()
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise RuntimeError(f"LAM {field} must be a regular non-symlink file")
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
        return {"bytes": metadata.st_size, "sha256": digest.hexdigest()}
    finally:
        os.close(descriptor)


def _lam_inventory_entry() -> tuple[dict[str, Any], str, Path]:
    inventory_path = lam_checked_identity_path(INVENTORY_PATH, field="dataset inventory")
    payload = read_regular_file_bytes_nofollow(inventory_path, label="LAM dataset inventory")
    inventory = _lam_json_object(payload, field="dataset inventory")
    matches = [
        entry for entry in inventory.get("datasets", [])
        if isinstance(entry, dict) and entry.get("name") == LAM_DATASET_NAME
    ]
    if len(matches) != 1:
        raise RuntimeError("LAM dataset inventory must contain exactly one released-output entry")
    return matches[0], hashlib.sha256(payload).hexdigest(), inventory_path


def _lam_release_rows(
    release_payload: bytes,
    source_root: Path,
) -> list[dict[str, Any]]:
    try:
        text = release_payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise RuntimeError("LAM release manifest is not valid UTF-8 CSV") from exc
    reader = csv.DictReader(StringIO(text, newline=""))
    required = {"object_release_id", "category", "tier", "rel_path", "n_movable"}
    if reader.fieldnames is None or not required.issubset(reader.fieldnames):
        raise ValueError(f"LAM release manifest is missing columns: {sorted(required)}")
    rows: list[dict[str, Any]] = []
    seen_keys: set[str] = set()
    seen_paths: set[str] = set()
    for release_order, raw in enumerate(reader):
        object_release_id = str(raw.get("object_release_id", "")).strip()
        category = str(raw.get("category", "")).strip()
        tier = str(raw.get("tier", "")).strip()
        rel_path = str(raw.get("rel_path", "")).strip()
        if not object_release_id or not category:
            raise ValueError(f"LAM release identity is empty at row {release_order}")
        if tier not in {"viable", "loads_only", "broken"}:
            raise ValueError(f"LAM release tier is invalid at row {release_order}: {tier!r}")
        package = lam_source_path(
            source_root, rel_path, field="release package", required_kind="directory"
        )
        primary = lam_source_path(
            package, "generated.urdf", field="generated URDF", required_kind="file"
        )
        asset_key = f"{tier}:{rel_path}"
        if rel_path in seen_paths:
            raise ValueError(f"duplicate LAM release rel_path: {rel_path}")
        if asset_key in seen_keys:
            raise ValueError(f"duplicate LAM release asset_key: {asset_key}")
        seen_paths.add(rel_path)
        seen_keys.add(asset_key)
        try:
            declared_hint = max(0, int(str(raw.get("n_movable", "0")) or "0"))
        except ValueError as exc:
            raise ValueError(f"LAM release n_movable is invalid: {asset_key}") from exc
        rows.append({
            "release_order": release_order,
            "asset_key": asset_key,
            "object_release_id": object_release_id,
            "category": category,
            "tier": tier,
            "rel_path": rel_path,
            "declared_joint_count_hint": declared_hint,
            "package": str(package),
            "primary": str(primary),
        })
    return rows


def load_lam_cohort(
    source_root: Path,
    cohort_records_path: Path,
    *,
    formal: bool,
) -> dict[str, Any]:
    """Load Table 3 identities in rank order and join its completion-order JSONL."""

    source_root = lam_checked_identity_path(source_root, field="source root")
    if not source_root.is_dir():
        raise NotADirectoryError(source_root)
    source_root = source_root.resolve(strict=True)
    cohort_records_path = lam_checked_identity_path(
        cohort_records_path, field="cohort asset_records.jsonl"
    )
    cohort_payload = read_regular_file_bytes_nofollow(
        cohort_records_path, label="LAM cohort asset_records.jsonl"
    )
    cohort_hash = hashlib.sha256(cohort_payload).hexdigest()
    cohort_records_path = cohort_records_path.resolve(strict=True)
    table3_manifest_path = lam_checked_identity_path(
        cohort_records_path.parent / "manifest.json", field="Table 3 manifest"
    )
    table3_payload = read_regular_file_bytes_nofollow(
        table3_manifest_path, label="LAM Table 3 manifest"
    )
    table3_hash = hashlib.sha256(table3_payload).hexdigest()
    table3_manifest = _lam_json_object(table3_payload, field="Table 3 manifest")
    table3_content_hash = table3_manifest.get("manifest_content_sha256")
    if table3_content_hash != manifest_self_hash(table3_manifest):
        raise ValueError("LAM Table 3 manifest self-hash mismatch")
    if table3_manifest.get("dataset") != LAM_TABLE3_DATASET_NAME:
        raise ValueError("LAM Table 3 manifest dataset mismatch")

    table3_source = table3_manifest.get("source")
    if not isinstance(table3_source, dict):
        raise ValueError("LAM Table 3 source provenance is missing")
    bound_source_root = lam_checked_identity_path(
        Path(str(table3_source.get("source_root", ""))), field="Table 3 source root"
    )
    if bound_source_root.resolve(strict=False) != source_root:
        raise RuntimeError("LAM Table 3 source root identity mismatch")
    release_path = lam_checked_identity_path(
        Path(str(table3_source.get("release_manifest", ""))), field="release manifest"
    )
    release_payload = read_regular_file_bytes_nofollow(
        release_path, label="LAM release manifest"
    )
    release_hash = hashlib.sha256(release_payload).hexdigest()
    if table3_source.get("release_manifest_sha256") != release_hash:
        raise RuntimeError("LAM release manifest SHA-256 drift")
    release_rows = _lam_release_rows(release_payload, source_root)
    release_by_key = {row["asset_key"]: row for row in release_rows}
    tier_counts = dict(sorted(Counter(row["tier"] for row in release_rows).items()))
    candidate_pool_hash = canonical_sha256(sorted(release_by_key))
    source_expectations = {
        "n_release": len(release_rows),
        "tier_counts": tier_counts,
        "candidate_pool_sha256": candidate_pool_hash,
    }
    for field, expected in source_expectations.items():
        if table3_source.get(field) != expected:
            raise RuntimeError(f"LAM Table 3 source {field} mismatch")

    dataset_api_path = lam_checked_identity_path(
        Path(str(table3_source.get("dataset_api", ""))), field="dataset API"
    )
    dataset_api_payload = read_regular_file_bytes_nofollow(
        dataset_api_path, label="LAM dataset API"
    )
    dataset_api_hash = hashlib.sha256(dataset_api_payload).hexdigest()
    if table3_source.get("dataset_api_sha256") != dataset_api_hash:
        raise RuntimeError("LAM dataset API SHA-256 drift")
    dataset_api = _lam_json_object(dataset_api_payload, field="dataset API")
    upstream_revision = dataset_api.get("sha")
    if not isinstance(upstream_revision, str) or table3_source.get("upstream_revision") != upstream_revision:
        raise RuntimeError("LAM upstream revision mismatch")

    inventory, inventory_hash, inventory_path = _lam_inventory_entry()
    dataset_root = source_root.parent
    inventory_source = inventory.get("source")
    verification = inventory.get("verification")
    if not isinstance(inventory_source, dict) or not isinstance(verification, dict):
        raise RuntimeError("LAM dataset inventory provenance is incomplete")
    inventory_root = lam_checked_identity_path(
        Path(str(inventory.get("canonical_path", ""))), field="inventory dataset root"
    )
    inventory_source_root = lam_checked_identity_path(
        Path(str(inventory.get("released_outputs_root", ""))), field="inventory source root"
    )
    inventory_manifest_path = lam_checked_identity_path(
        Path(str(inventory.get("manifest", ""))), field="inventory release manifest"
    )
    inventory_expectations = {
        "status": (inventory.get("status"), "VERIFIED_RELEASE_COMPLETE"),
        "dataset_root": (inventory_root.resolve(strict=False), dataset_root),
        "source_root": (inventory_source_root.resolve(strict=False), source_root),
        "release_manifest": (inventory_manifest_path.resolve(strict=False), release_path.resolve(strict=True)),
        "source_type": (inventory_source.get("type"), "huggingface_dataset"),
        "repo_id": (inventory_source.get("repo_id"), "YipengGao/Articulated-Object-Code"),
        "revision": (inventory_source.get("revision"), upstream_revision),
        "license": (str(inventory_source.get("license", "")).lower(), "mit"),
        "manifest_objects": (verification.get("manifest_objects"), len(release_rows)),
        "tier_counts": (verification.get("tier_counts"), tier_counts),
        "urdf_files": (verification.get("manifest_urdf_files_present"), len(release_rows)),
        "urdf_parse_failures": (verification.get("manifest_urdf_xml_parse_failures"), 0),
        "archive_failures": (verification.get("archive_sha256_failures"), 0),
        "archive_collisions": (verification.get("cross_archive_path_collisions"), 0),
    }
    for field, (observed, expected) in inventory_expectations.items():
        if observed != expected:
            raise RuntimeError(f"LAM dataset inventory {field} mismatch")

    archive_entries = inventory.get("archives")
    if not isinstance(archive_entries, list):
        raise RuntimeError("LAM dataset inventory archive identity is missing")
    archives: dict[str, dict[str, Any]] = {}
    for raw_archive in archive_entries:
        if not isinstance(raw_archive, dict) or not isinstance(raw_archive.get("name"), str):
            raise RuntimeError("LAM dataset inventory archive entry is invalid")
        name = raw_archive["name"]
        if name in archives or name not in LAM_FORMAL_ARCHIVES:
            raise RuntimeError(f"LAM dataset inventory archive name is invalid: {name!r}")
        archive_path = lam_checked_identity_path(dataset_root / name, field=f"archive {name}")
        archive_identity = _lam_regular_file_identity(archive_path, field=f"archive {name}")
        observed = {
            "path": str(archive_path.resolve(strict=True)),
            **archive_identity,
        }
        if (
            raw_archive.get("bytes") != observed["bytes"]
            or raw_archive.get("sha256") != observed["sha256"]
        ):
            raise RuntimeError(f"LAM archive identity drift: {name}")
        archives[name] = observed
    if set(archives) != set(LAM_FORMAL_ARCHIVES):
        raise RuntimeError("LAM dataset inventory must bind all three release archives")

    selection = table3_manifest.get("selection")
    manifest_records = table3_manifest.get("records")
    if not isinstance(selection, dict) or not isinstance(manifest_records, list):
        raise ValueError("LAM Table 3 selection or records are missing")
    if selection.get("algorithm") != LAM_TABLE3_SELECTION_ALGORITHM:
        raise ValueError("LAM Table 3 selection algorithm mismatch")
    if selection.get("quality_label_blind") is not True:
        raise ValueError("LAM Table 3 selection is not quality-label blind")
    if selection.get("seed") != DEFAULT_SEED:
        raise ValueError(f"LAM Table 3 selection seed must be {DEFAULT_SEED}")
    if selection.get("n_eval") != len(manifest_records):
        raise ValueError("LAM Table 3 selected record count mismatch")

    selected_keys: list[str] = []
    manifest_by_key: dict[str, dict[str, Any]] = {}
    for selection_rank, raw in enumerate(manifest_records, start=1):
        if not isinstance(raw, dict):
            raise ValueError("LAM Table 3 manifest record is not an object")
        if raw.get("selection_rank") != selection_rank:
            raise ValueError(f"LAM Table 3 selection rank mismatch at {selection_rank}")
        asset_key = raw.get("asset_key")
        if not isinstance(asset_key, str) or not asset_key:
            raise ValueError("LAM Table 3 asset_key is invalid")
        if asset_key in manifest_by_key:
            raise ValueError(f"duplicate LAM Table 3 asset_key: {asset_key}")
        rel_path_raw = raw.get("rel_path")
        if not isinstance(rel_path_raw, str):
            raise ValueError(f"LAM Table 3 relative path is invalid: {asset_key}")
        try:
            safe_package_relative_path(rel_path_raw, field="LAM Table 3 relative path")
        except ValueError as exc:
            raise RuntimeError(f"LAM Table 3 relative path escapes source root: {rel_path_raw!r}") from exc
        release = release_by_key.get(asset_key)
        if release is None:
            raise ValueError(f"LAM Table 3 asset is absent from release manifest: {asset_key}")
        identity_fields = (
            "asset_key", "object_release_id", "category", "tier", "rel_path", "release_order",
            "declared_joint_count_hint",
        )
        for field in identity_fields:
            if raw.get(field) != release[field]:
                raise ValueError(f"LAM Table 3 release identity mismatch for {field}: {asset_key}")
        expected_selection_hash = hashlib.sha256(
            f"lam-table3-v1\0{DEFAULT_SEED}\0{asset_key}".encode("utf-8")
        ).hexdigest()
        if raw.get("selection_hash") != expected_selection_hash:
            raise ValueError(f"LAM Table 3 selection hash mismatch: {asset_key}")
        package = Path(release["package"])
        primary = Path(release["primary"])
        if raw.get("urdf_path") != str(primary) or raw.get("urdf_exists") is not True:
            raise RuntimeError(f"LAM Table 3 primary URDF identity mismatch: {asset_key}")
        primary_hash = sha256_file(primary)
        if raw.get("urdf_sha256") != primary_hash:
            raise RuntimeError(f"LAM primary URDF SHA-256 drift: {asset_key}")
        selected_keys.append(asset_key)
        manifest_by_key[asset_key] = {
            **raw,
            "package": str(package),
            "primary_urdf_relative_path": "generated.urdf",
        }
    selected_keys_hash = canonical_sha256(selected_keys)
    if selection.get("selected_asset_keys_sha256") != selected_keys_hash:
        raise ValueError("LAM Table 3 selected asset keys SHA-256 mismatch")

    completion_rows: list[dict[str, Any]] = []
    try:
        cohort_text = cohort_payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise RuntimeError("LAM cohort JSONL is not valid UTF-8") from exc
    completion_by_key: dict[str, tuple[int, dict[str, Any]]] = {}
    for line_number, line in enumerate(cohort_text.splitlines(), start=1):
        if not line.strip():
            continue
        try:
            raw = json.loads(line)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"invalid LAM cohort JSONL at line {line_number}: {exc}") from exc
        if not isinstance(raw, dict):
            raise ValueError(f"LAM cohort JSONL line {line_number} is not an object")
        asset_key = raw.get("asset_key")
        if not isinstance(asset_key, str) or not asset_key:
            raise ValueError(f"LAM cohort asset_key is invalid at line {line_number}")
        if asset_key in completion_by_key:
            raise ValueError(f"duplicate LAM cohort asset_key: {asset_key}")
        completion_by_key[asset_key] = (line_number, raw)
        completion_rows.append(raw)
    if len(completion_rows) != len(manifest_records):
        raise ValueError("LAM cohort completion record count mismatch")
    if set(completion_by_key) != set(manifest_by_key):
        raise ValueError("LAM cohort completion keys are missing or unexpected")

    frozen_assets: list[dict[str, Any]] = []
    completion_identity_fields = (
        "asset_key", "object_release_id", "category", "tier", "rel_path", "selection_rank",
        "selection_hash", "urdf_sha256",
    )
    for selection_index, asset_key in enumerate(selected_keys):
        frozen = manifest_by_key[asset_key]
        completion_index, completion = completion_by_key[asset_key]
        for field in completion_identity_fields:
            if completion.get(field) != frozen.get(field):
                raise ValueError(f"LAM cohort metadata mismatch for {field}: {asset_key}")
        if completion.get("manifest_content_sha256") != table3_content_hash:
            raise ValueError(f"LAM cohort manifest binding mismatch: {asset_key}")
        status = completion.get("status")
        if status not in {"completed", "error", "timeout"}:
            raise ValueError(f"LAM cohort status is invalid: {asset_key}")
        error = completion.get("error")
        if error is not None and not isinstance(error, str):
            raise ValueError(f"LAM cohort error annotation is invalid: {asset_key}")
        for field in ("parse_success", "tree_valid", "strict_kinematic_pass"):
            if not isinstance(completion.get(field), bool):
                raise ValueError(f"LAM cohort {field} annotation is invalid: {asset_key}")
        frozen_assets.append({
            "selection_index": selection_index,
            "asset_id": asset_key,
            **{field: frozen[field] for field in (
                "asset_key", "category", "tier", "rel_path", "object_release_id",
                "release_order", "declared_joint_count_hint", "selection_rank", "selection_hash",
            )},
            "table3_manifest_record_sha256": canonical_sha256(manifest_records[selection_index]),
            "table3_record_sha256": canonical_sha256(completion),
            "table3_completion_index": completion_index,
            "table3_status": status,
            "table3_error": error,
            "table3_parse_success": completion["parse_success"],
            "table3_tree_valid": completion["tree_valid"],
            "table3_strict_kinematic_pass": completion["strict_kinematic_pass"],
            "table3_manifest_content_sha256": table3_content_hash,
            "package": frozen["package"],
            "primary_urdf_relative_path": "generated.urdf",
        })

    if formal:
        formal_expectations = {
            "source_root": (
                source_root,
                lam_checked_identity_path(DEFAULT_LAM_SOURCE_ROOT, field="formal source root").resolve(strict=False),
            ),
            "cohort_records_path": (
                cohort_records_path,
                lam_checked_identity_path(DEFAULT_LAM_COHORT_RECORDS, field="formal cohort").resolve(strict=False),
            ),
            "table3_manifest_path": (
                table3_manifest_path.resolve(strict=True),
                lam_checked_identity_path(DEFAULT_LAM_TABLE3_MANIFEST, field="formal Table 3 manifest").resolve(strict=False),
            ),
            "release_manifest_path": (
                release_path.resolve(strict=True),
                lam_checked_identity_path(DEFAULT_LAM_RELEASE_MANIFEST, field="formal release manifest").resolve(strict=False),
            ),
            "dataset_api_path": (
                dataset_api_path.resolve(strict=True),
                lam_checked_identity_path(DEFAULT_LAM_DATASET_API, field="formal dataset API").resolve(strict=False),
            ),
            "cohort_records_sha256": (cohort_hash, LAM_COHORT_RECORDS_SHA256),
            "table3_manifest_sha256": (table3_hash, LAM_TABLE3_MANIFEST_SHA256),
            "table3_manifest_content_sha256": (
                table3_content_hash, LAM_TABLE3_MANIFEST_CONTENT_SHA256,
            ),
            "release_manifest_sha256": (release_hash, LAM_RELEASE_MANIFEST_SHA256),
            "dataset_api_sha256": (dataset_api_hash, LAM_DATASET_API_SHA256),
            "dataset_inventory_sha256": (inventory_hash, LAM_DATASET_INVENTORY_SHA256),
            "release_count": (len(release_rows), LAM_FORMAL_RELEASE_COUNT),
            "cohort_count": (len(frozen_assets), LAM_FORMAL_COHORT_COUNT),
            "tier_counts": (tier_counts, LAM_FORMAL_TIER_COUNTS),
            "category_count": (
                len({row["category"] for row in frozen_assets}), LAM_FORMAL_CATEGORY_COUNT,
            ),
            "candidate_pool_sha256": (candidate_pool_hash, LAM_CANDIDATE_POOL_SHA256),
            "selected_asset_keys_sha256": (
                selected_keys_hash, LAM_SELECTED_ASSET_KEYS_SHA256,
            ),
            "upstream_revision": (upstream_revision, LAM_UPSTREAM_REVISION),
        }
        for field, (observed, expected) in formal_expectations.items():
            if observed != expected:
                raise RuntimeError(f"formal LAM {field} mismatch: {observed!r} != {expected!r}")
        for name, expected in LAM_FORMAL_ARCHIVES.items():
            observed = archives.get(name)
            if observed is None or {
                "bytes": observed["bytes"], "sha256": observed["sha256"]
            } != expected:
                raise RuntimeError(f"formal LAM archive identity mismatch: {name}")

    return {
        "source_root": str(source_root),
        "release_manifest_path": str(release_path.resolve(strict=True)),
        "release_manifest_sha256": release_hash,
        "dataset_api_path": str(dataset_api_path.resolve(strict=True)),
        "dataset_api_sha256": dataset_api_hash,
        "upstream_revision": upstream_revision,
        "release_asset_count": len(release_rows),
        "tier_counts": tier_counts,
        "candidate_pool_sha256": candidate_pool_hash,
        "cohort_records_path": str(cohort_records_path),
        "cohort_records_sha256": cohort_hash,
        "table3_manifest_path": str(table3_manifest_path.resolve(strict=True)),
        "table3_manifest_sha256": table3_hash,
        "table3_manifest_content_sha256": table3_content_hash,
        "selected_asset_keys_sha256": selected_keys_hash,
        "seed": selection["seed"],
        "source_selection_algorithm": selection["algorithm"],
        "quality_label_blind": selection["quality_label_blind"],
        "inventory": inventory,
        "inventory_path": str(inventory_path.resolve(strict=True)),
        "inventory_sha256": inventory_hash,
        "archives": archives,
        "assets": frozen_assets,
    }


def archive_release_identity(dataset_root: Path, extracted_ids: Iterable[str]) -> dict[str, Any]:
    filenames = sorted(path.name for path in dataset_root.glob("rec_*.tar.gz") if path.is_file())
    archive_ids = [name[:-7] for name in filenames]
    extracted = sorted(set(extracted_ids))
    if archive_ids != extracted:
        raise RuntimeError("archive/extracted release identity mismatch")
    return {
        "archive_count": len(filenames),
        "archive_ids": archive_ids,
        "archive_id_list_sha256": canonical_sha256(archive_ids),
        "archive_filename_list_sha256": canonical_sha256(filenames),
        "provenance": "sorted rec_*.tar.gz filenames in canonical Articraft-10K dataset root",
    }


def proc_start_identity(pid: int) -> str | None:
    try:
        return Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()[21]
    except (FileNotFoundError, IndexError, OSError):
        return None


def proven_owned_process(metadata: dict[str, Any]) -> bool:
    try:
        pid = int(metadata["pid"])
        pgid = int(metadata["pgid"])
        live_pgid = os.getpgid(pid)
        live_sid = os.getsid(pid)
        job_path = str(Path(metadata["job_path"]).resolve(strict=False))
        token = str(metadata["run_token"])
        cmdline = [
            value.decode("utf-8", errors="surrogateescape")
            for value in Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
            if value
        ]
    except (KeyError, FileNotFoundError, OSError, ValueError):
        return False
    return (
        pgid == pid
        and live_pgid == pgid
        and live_sid == pid
        and proc_start_identity(pid) == str(metadata.get("process_start_identity"))
        and metadata.get("runner_script") == str(SCRIPT_PATH)
        and metadata.get("runner_sha256") == sha256_file(SCRIPT_PATH)
        and Path(str(metadata.get("output_root"))).resolve(strict=False)
        == Path(job_path).parents[2]
        and str(SCRIPT_PATH) in cmdline
        and job_path in cmdline
        and token in Path(job_path).read_text(encoding="utf-8")
    )


def recover_stale_worker_scratch(output: Path) -> dict[str, Any]:
    stale = output / ".worker_scratch"
    if not stale.exists():
        return {"quarantined": False, "terminated_owned_groups": []}
    terminated: list[int] = []
    for job_root in sorted(stale.glob("job_*")):
        metadata_path = job_root / "ownership.json"
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            continue
        if proven_owned_process(metadata):
            pgid = int(metadata["pgid"])
            terminate_owned_process_group(pgid)
            terminated.append(pgid)
    quarantine = output / f"stale_worker_scratch_{uuid.uuid4().hex}"
    stale.replace(quarantine)
    return {
        "quarantined": True,
        "quarantine_path": str(quarantine),
        "terminated_owned_groups": terminated,
    }


def environment_metadata(run_standard_parser: bool, workers: int) -> dict[str, Any]:
    def version(name: str) -> str | None:
        try:
            return importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            return None

    dependencies = {name: version(name) for name in EVALUATION_DEPENDENCIES}
    return {
        "python": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "executable": sys.executable,
        "platform": platform.platform(),
        "dependencies": dependencies,
        "standard_parser_enabled": run_standard_parser,
        "standard_parser": {
            "package": "urdfpy",
            "required_version": "0.0.22",
            "observed_version": version("urdfpy"),
            "numpy_compatibility": NUMPY_COMPATIBILITY_POLICY,
        },
        "worker_count": workers,
        "gpu_required": False,
    }


def current_worker_runtime_binding(job: dict[str, Any]) -> dict[str, Any]:
    dataset = job.get("dataset")
    run_token = job.get("run_token")
    run_standard_parser = job.get("run_standard_parser")
    workers = job.get("workers")
    expected = job.get("runtime_binding")
    output_root = job.get("output_root")
    if dataset not in {
        "Articraft-10K", "Artiverse", "PartNet-Mobility", LAM_DATASET_NAME
    }:
        raise FatalRuntimeBindingError("worker job dataset is invalid")
    if not isinstance(run_token, str) or not run_token:
        raise FatalRuntimeBindingError("worker job run token is invalid")
    if not isinstance(run_standard_parser, bool):
        raise FatalRuntimeBindingError("worker job parser flag is invalid")
    if type(workers) is not int or workers <= 0:
        raise FatalRuntimeBindingError("worker job count is invalid")
    if not isinstance(expected, dict) or not isinstance(expected.get("protocol_path"), str):
        raise FatalRuntimeBindingError("worker protocol snapshot binding is missing")
    if not isinstance(output_root, str) or not Path(output_root).is_absolute():
        raise FatalRuntimeBindingError("worker protocol snapshot output root is invalid")
    snapshot = checked_protocol_snapshot_path(
        Path(output_root), expected["protocol_path"], error_type=FatalRuntimeBindingError
    )
    protocol_bytes = read_regular_file_bytes_nofollow(
        snapshot, label="protocol snapshot", error_type=FatalRuntimeBindingError
    )
    config = evaluator_config_for_dataset(dataset)
    environment = environment_metadata(run_standard_parser, workers)
    return {
        "run_token": run_token,
        "evaluator_path": str(SCRIPT_PATH),
        "evaluator_sha256": sha256_file(SCRIPT_PATH),
        "protocol_path": str(snapshot),
        "protocol_sha256": hashlib.sha256(protocol_bytes).hexdigest(),
        "config": config,
        "config_sha256": canonical_sha256(config),
        "environment": environment,
        "environment_sha256": canonical_sha256(environment),
    }


def runtime_evaluation_binding(evaluation: dict[str, Any]) -> dict[str, Any]:
    try:
        return {field: evaluation[field] for field in RUNTIME_BINDING_STATIC_FIELDS}
    except KeyError as exc:
        raise FatalRuntimeBindingError(
            f"frozen evaluation runtime binding is missing {exc.args[0]}"
        ) from exc


def frozen_worker_runtime_binding(
    manifest: dict[str, Any], run_token: str
) -> dict[str, Any]:
    return {
        "run_token": run_token,
        **runtime_evaluation_binding(manifest["evaluation"]),
    }


def validate_child_runtime_binding(job: dict[str, Any]) -> dict[str, Any]:
    expected = job.get("runtime_binding")
    if not isinstance(expected, dict):
        raise FatalRuntimeBindingError("worker runtime binding is missing")
    observed = current_worker_runtime_binding(job)
    if observed != expected:
        changed = sorted(
            key for key in set(expected) | set(observed) if expected.get(key) != observed.get(key)
        )
        raise FatalRuntimeBindingError(f"worker runtime binding drift: {', '.join(changed)}")
    return observed


def frozen_command(args: argparse.Namespace) -> list[str]:
    command = [
        str(SCRIPT_PATH),
        "--mode", args.mode,
        "--dataset", args.dataset,
        "--source-root", str(args.source_root.resolve(strict=False)),
        "--n", str(args.n),
        "--seed", str(args.seed),
        "--workers", str(args.workers),
        "--asset-timeout-seconds", str(args.asset_timeout_seconds),
    ]
    if args.limit is not None:
        command.extend(["--limit", str(args.limit)])
    if args.no_standard_parser:
        command.append("--no-standard-parser")
    if args.cohort_manifest is not None:
        command.extend([
            "--cohort-manifest", str(args.cohort_manifest.resolve(strict=False))
        ])
    return command


def build_manifest(
    source_root: Path,
    all_ids: list[str],
    selected_ids: list[str],
    seed: int,
    requested_n: int,
    limit: int | None,
    standard_parser: bool,
    workers: int,
    protocol_binding: dict[str, str],
    mode: str = "formal",
    command: list[str] | None = None,
    archive_identity: dict[str, Any] | None = None,
) -> dict[str, Any]:
    frozen_protocol = protocol_binding_fields(protocol_binding)
    inventory, inventory_hash = load_inventory_entry()
    configured_root = Path(inventory["urdf_root"]).resolve()
    if source_root.resolve() != configured_root:
        raise RuntimeError(
            f"source root does not match the verified inventory entry: {source_root} != {configured_root}"
        )
    frozen_environment = environment_metadata(standard_parser, workers)
    records = []
    for selection_index, asset_id in enumerate(selected_ids):
        package = source_root / asset_id
        binding = package_binding(package)
        records.append(
            {
                "selection_index": selection_index,
                "asset_id": asset_id,
                "package": str(package.resolve()),
                "model_urdf_sha256": next(
                    row["sha256"] for row in binding["files"] if row["path"] == "model.urdf"
                ),
                "package_binding": binding,
            }
        )
    manifest = {
        "schema_version": "1.0.0",
        "created_at": utc_now(),
        "dataset": "Articraft-10K",
        "mode": mode,
        "classification": run_classification(mode),
        "source": {
            "root": str(source_root.resolve()),
            "repo_id": inventory["source"]["repo_id"],
            "revision": inventory["source"]["revision"],
            "inventory_path": str(INVENTORY_PATH),
            "inventory_sha256": inventory_hash,
            "release_asset_count": len(all_ids),
            "release_asset_ids_sha256": canonical_sha256(all_ids),
            "archive_identity": archive_identity,
        },
        "selection": {
            "algorithm": "random.Random(seed).sample(sorted(unique_asset_ids), n)",
            "seed": seed,
            "requested_n": requested_n,
            "limit": limit,
            "n_eval": len(selected_ids),
            "selected_asset_ids_sha256": canonical_sha256(selected_ids),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
        },
        "evaluation": {
            "metric_names": list(METRIC_NAMES),
            "standard_parser": "urdfpy" if standard_parser else "disabled",
            "standard_parser_version": "0.0.22" if standard_parser else None,
            "numpy_compatibility": NUMPY_COMPATIBILITY_POLICY,
            "workers": workers,
            "gpu_required": False,
            "denominator_policy": "all frozen selected assets, including errors",
            "asset_timeout_seconds": ASSET_TIMEOUT_SECONDS,
            "command": command if command is not None else [str(SCRIPT_PATH)],
            "environment": frozen_environment,
            "environment_sha256": canonical_sha256(frozen_environment),
            "environment_file_sha256": hashlib.sha256(
                formatted_json(frozen_environment).encode("utf-8")
            ).hexdigest(),
            "evaluator_path": str(SCRIPT_PATH),
            "evaluator_sha256": sha256_file(SCRIPT_PATH),
            **frozen_protocol,
            "config": evaluator_config_for_dataset("Articraft-10K"),
            "config_sha256": canonical_sha256(
                evaluator_config_for_dataset("Articraft-10K")
            ),
        },
        "records": records,
    }
    manifest["manifest_content_sha256"] = manifest_self_hash(manifest)
    return manifest


def build_artiverse_manifest(
    loaded: dict[str, Any],
    *,
    requested_n: int,
    limit: int | None,
    standard_parser: bool,
    workers: int,
    protocol_binding: dict[str, str],
    mode: str,
    command: list[str] | None = None,
) -> dict[str, Any]:
    frozen_protocol = protocol_binding_fields(protocol_binding)
    frozen_environment = environment_metadata(standard_parser, workers)
    frozen_config = evaluator_config_for_dataset("Artiverse")
    records = []
    for selection_index, source_record in enumerate(loaded["assets"]):
        package = Path(source_record["package"])
        binding = package_binding(package)
        primary_relative = source_record["primary_urdf_relative_path"]
        primary = primary_urdf_path(package, primary_relative)
        records.append(
            {
                "selection_index": selection_index,
                **{
                    field: source_record[field]
                    for field in ("asset_id", *ARTIVERSE_METADATA_FIELDS)
                },
                "package": str(package.resolve(strict=True)),
                "primary_urdf_relative_path": primary_relative,
                "primary_urdf_sha256": sha256_file(primary),
                "model_urdf_sha256": sha256_file(primary),
                "package_binding": binding,
            }
        )
    manifest = {
        "schema_version": "1.1.0",
        "created_at": utc_now(),
        "dataset": "Artiverse",
        "mode": mode,
        "classification": run_classification(mode),
        "source": {
            "root": loaded["source_root"],
            "release_status": loaded["release_status"],
            "release_asset_count": loaded["n_release"],
            "release_manifest_path": loaded["release_manifest_path"],
            "release_manifest_sha256": loaded["release_manifest_sha256"],
            "release_universe_sha256": loaded["release_universe_sha256"],
            "cohort_manifest_path": loaded["cohort_manifest_path"],
            "cohort_manifest_sha256": loaded["cohort_manifest_sha256"],
            "cohort_asset_count": loaded["cohort_asset_count"],
        },
        "selection": {
            "algorithm": "existing Table 1 manifest order; no resampling/reselection",
            "source_protocol": loaded["selection_protocol"],
            "seed": loaded["seed"],
            "requested_n": requested_n,
            "limit": limit,
            "n_eval": len(records),
            "selected_asset_ids_sha256": canonical_sha256(
                [record["asset_id"] for record in records]
            ),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
            "cohort_type": "GLOBAL_FIXED_SAMPLE_NOT_CATEGORY_BALANCED",
        },
        "evaluation": {
            "metric_names": list(METRIC_NAMES),
            "standard_parser": "urdfpy" if standard_parser else "disabled",
            "standard_parser_version": "0.0.22" if standard_parser else None,
            "numpy_compatibility": NUMPY_COMPATIBILITY_POLICY,
            "workers": workers,
            "gpu_required": False,
            "denominator_policy": "all frozen selected assets, including errors",
            "category_macro_policy": (
                "unweighted mean of per-raw_category asset rates; all frozen assets and "
                "failures retained"
            ),
            "asset_timeout_seconds": ASSET_TIMEOUT_SECONDS,
            "command": command if command is not None else [str(SCRIPT_PATH)],
            "environment": frozen_environment,
            "environment_sha256": canonical_sha256(frozen_environment),
            "environment_file_sha256": hashlib.sha256(
                formatted_json(frozen_environment).encode("utf-8")
            ).hexdigest(),
            "evaluator_path": str(SCRIPT_PATH),
            "evaluator_sha256": sha256_file(SCRIPT_PATH),
            **frozen_protocol,
            "config": frozen_config,
            "config_sha256": canonical_sha256(frozen_config),
        },
        "records": records,
    }
    manifest["manifest_content_sha256"] = manifest_self_hash(manifest)
    return manifest


def build_partnet_mobility_manifest(
    loaded: dict[str, Any],
    *,
    requested_n: int,
    limit: int | None,
    standard_parser: bool,
    workers: int,
    protocol_binding: dict[str, str],
    mode: str,
    command: list[str] | None = None,
) -> dict[str, Any]:
    frozen_protocol = protocol_binding_fields(protocol_binding)
    frozen_environment = environment_metadata(standard_parser, workers)
    frozen_config = evaluator_config_for_dataset("PartNet-Mobility")
    records = []
    for selection_index, source_record in enumerate(loaded["assets"]):
        package = Path(source_record["package"])
        binding = package_binding(package)
        primary = primary_urdf_path(package, "mobility.urdf")
        records.append({
            "selection_index": selection_index,
            **{field: source_record[field] for field in PARTNET_MOBILITY_METADATA_FIELDS},
            "asset_id": source_record["asset_id"],
            "package": str(package.resolve(strict=True)),
            "primary_urdf_relative_path": "mobility.urdf",
            "primary_urdf_sha256": sha256_file(primary),
            "model_urdf_sha256": sha256_file(primary),
            "package_binding": binding,
        })
    manifest = {
        "schema_version": "1.1.0",
        "created_at": utc_now(),
        "dataset": "PartNet-Mobility",
        "mode": mode,
        "classification": run_classification(mode),
        "source": {
            "root": loaded["source_root"],
            "release_asset_count": loaded["release_asset_count"],
            "candidate_pool_identity_sha256": loaded["candidate_pool_identity_sha256"],
            "archive": loaded["archive"],
            "cohort_manifest_path": loaded["cohort_manifest_path"],
            "cohort_manifest_sha256": loaded["cohort_manifest_sha256"],
            "items_sha256": loaded["items_sha256"],
            "ordered_selected_ids_sha256": loaded["ordered_selected_ids_sha256"],
            "selection_policy": loaded["selection_policy"],
            "dataset_inventory": loaded["inventory"],
            "dataset_inventory_sha256": loaded["inventory_sha256"],
        },
        "selection": {
            "algorithm": "existing frozen manifest items order; no resampling/reselection",
            "source_protocol": PARTNET_MOBILITY_SELECTION_PROTOCOL,
            "selection_policy": loaded["selection_policy"],
            "requested_n": requested_n,
            "limit": limit,
            "n_eval": len(records),
            "selected_asset_ids_sha256": canonical_sha256([record["asset_id"] for record in records]),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
        },
        "evaluation": {
            "metric_names": list(METRIC_NAMES),
            "standard_parser": "urdfpy" if standard_parser else "disabled",
            "standard_parser_version": "0.0.22" if standard_parser else None,
            "numpy_compatibility": NUMPY_COMPATIBILITY_POLICY,
            "workers": workers,
            "gpu_required": False,
            "denominator_policy": "all frozen selected assets, including errors",
            "category_macro_policy": (
                "unweighted mean of per-category asset rates; all frozen assets and failures retained"
            ),
            "asset_timeout_seconds": ASSET_TIMEOUT_SECONDS,
            "command": command if command is not None else [str(SCRIPT_PATH)],
            "environment": frozen_environment,
            "environment_sha256": canonical_sha256(frozen_environment),
            "environment_file_sha256": hashlib.sha256(formatted_json(frozen_environment).encode("utf-8")).hexdigest(),
            "evaluator_path": str(SCRIPT_PATH),
            "evaluator_sha256": sha256_file(SCRIPT_PATH),
            **frozen_protocol,
            "config": frozen_config,
            "config_sha256": canonical_sha256(frozen_config),
        },
        "records": records,
    }
    manifest["manifest_content_sha256"] = manifest_self_hash(manifest)
    return manifest


def build_lam_manifest(
    loaded: dict[str, Any],
    *,
    requested_n: int,
    limit: int | None,
    standard_parser: bool,
    workers: int,
    protocol_binding: dict[str, str],
    mode: str,
    command: list[str] | None = None,
) -> dict[str, Any]:
    frozen_protocol = protocol_binding_fields(protocol_binding)
    frozen_environment = environment_metadata(standard_parser, workers)
    frozen_config = evaluator_config_for_dataset(LAM_DATASET_NAME)
    records = []
    for selection_index, source_record in enumerate(loaded["assets"]):
        package = Path(source_record["package"])
        binding = package_binding(package)
        primary = primary_urdf_path(package, "generated.urdf")
        primary_hash = sha256_file(primary)
        records.append({
            "selection_index": selection_index,
            **{field: source_record[field] for field in LAM_METADATA_FIELDS},
            "asset_id": source_record["asset_key"],
            "package": str(package.resolve(strict=True)),
            "primary_urdf_relative_path": "generated.urdf",
            "primary_urdf_sha256": primary_hash,
            "model_urdf_sha256": primary_hash,
            "package_binding": binding,
        })
    manifest = {
        "schema_version": "1.1.0",
        "created_at": utc_now(),
        "dataset": LAM_DATASET_NAME,
        "mode": mode,
        "classification": run_classification(mode),
        "source": {
            "root": loaded["source_root"],
            "release_asset_count": loaded["release_asset_count"],
            "tier_counts": loaded["tier_counts"],
            "candidate_pool_sha256": loaded["candidate_pool_sha256"],
            "release_manifest_path": loaded["release_manifest_path"],
            "release_manifest_sha256": loaded["release_manifest_sha256"],
            "dataset_api_path": loaded["dataset_api_path"],
            "dataset_api_sha256": loaded["dataset_api_sha256"],
            "upstream_revision": loaded["upstream_revision"],
            "cohort_records_path": loaded["cohort_records_path"],
            "cohort_records_sha256": loaded["cohort_records_sha256"],
            "table3_manifest_path": loaded["table3_manifest_path"],
            "table3_manifest_sha256": loaded["table3_manifest_sha256"],
            "table3_manifest_content_sha256": loaded["table3_manifest_content_sha256"],
            "dataset_inventory_path": loaded["inventory_path"],
            "dataset_inventory_sha256": loaded["inventory_sha256"],
            "dataset_inventory": loaded["inventory"],
            "archives": loaded["archives"],
        },
        "selection": {
            "algorithm": LAM_TABLE2_SELECTION_ALGORITHM,
            "source_algorithm": loaded["source_selection_algorithm"],
            "seed": loaded["seed"],
            "quality_label_blind": loaded["quality_label_blind"],
            "requested_n": requested_n,
            "limit": limit,
            "n_eval": len(records),
            "selected_asset_ids_sha256": canonical_sha256(
                [record["asset_id"] for record in records]
            ),
            "source_selected_asset_keys_sha256": loaded["selected_asset_keys_sha256"],
            "selection_order_preserved": True,
            "completion_order_used_for_selection": False,
            "outcome_based_reselection": False,
            "selected_failures_retained_without_replacement": True,
        },
        "evaluation": {
            "metric_names": list(METRIC_NAMES),
            "standard_parser": "urdfpy" if standard_parser else "disabled",
            "standard_parser_version": "0.0.22" if standard_parser else None,
            "numpy_compatibility": NUMPY_COMPATIBILITY_POLICY,
            "workers": workers,
            "gpu_required": False,
            "denominator_policy": "all frozen selected assets, including errors",
            "category_macro_policy": (
                "unweighted mean of per-category asset rates; all frozen assets and failures retained"
            ),
            "asset_timeout_seconds": ASSET_TIMEOUT_SECONDS,
            "command": command if command is not None else [str(SCRIPT_PATH)],
            "environment": frozen_environment,
            "environment_sha256": canonical_sha256(frozen_environment),
            "environment_file_sha256": hashlib.sha256(
                formatted_json(frozen_environment).encode("utf-8")
            ).hexdigest(),
            "evaluator_path": str(SCRIPT_PATH),
            "evaluator_sha256": sha256_file(SCRIPT_PATH),
            **frozen_protocol,
            "config": frozen_config,
            "config_sha256": canonical_sha256(frozen_config),
        },
        "records": records,
    }
    manifest["manifest_content_sha256"] = manifest_self_hash(manifest)
    return manifest


def validate_manifest_record_source(record: dict[str, Any]) -> None:
    package = Path(record["package"])
    if package_binding(package) != record["package_binding"]:
        raise RuntimeError(f"resume source package drifted: {record['asset_id']}")
    primary = primary_urdf_path(
        package, record.get("primary_urdf_relative_path", "model.urdf")
    )
    expected_hash = record.get("primary_urdf_sha256", record.get("model_urdf_sha256"))
    if sha256_file(primary) != expected_hash:
        raise RuntimeError(f"resume primary URDF drifted: {record['asset_id']}")


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"invalid JSONL at line {line_number}: {exc}") from exc
        if not isinstance(row, dict):
            raise RuntimeError(f"JSONL line {line_number} is not an object")
        rows.append(row)
    return rows


def validate_resume_record(
    record: dict[str, Any],
    expected_asset_id: str,
    expected_model_hash: str,
    expected_package_hash: str,
    expected_manifest_hash: str,
    expected_record: dict[str, Any] | None = None,
    expected_runtime_evaluation: dict[str, Any] | None = None,
) -> str | None:
    if record.get("asset_id") != expected_asset_id:
        raise RuntimeError("resume record asset ID mismatch")
    expected_bindings = {
        "model_urdf_sha256": expected_model_hash,
        "package_content_manifest_sha256": expected_package_hash,
        "manifest_content_sha256": expected_manifest_hash,
    }
    for name, expected in expected_bindings.items():
        if record.get(name) != expected:
            raise RuntimeError(f"resume record {name} binding mismatch")
    if expected_record is not None:
        optional_bindings = (
            "primary_urdf_relative_path",
            "primary_urdf_sha256",
            *FROZEN_SOURCE_METADATA_FIELDS,
        )
        for name in optional_bindings:
            if name in expected_record and record.get(name) != expected_record[name]:
                raise RuntimeError(f"resume record {name} binding mismatch")
    if record.get("status") not in {"completed", "error", "timeout"}:
        raise RuntimeError("resume record status is invalid")
    if not isinstance(record.get("metrics"), dict) or set(record["metrics"]) != set(METRIC_NAMES):
        raise RuntimeError("resume record metric schema mismatch")
    for name in METRIC_NAMES:
        value = record["metrics"][name]
        if not isinstance(value, dict) or not isinstance(value.get("pass"), bool):
            raise RuntimeError(f"resume record metric pass is not boolean: {name}")
    recomputed_strict = all(
        record["metrics"][name]["pass"] for name in METRIC_NAMES if name != "strict_urdf_pass"
    )
    if record["metrics"]["strict_urdf_pass"]["pass"] is not recomputed_strict:
        raise RuntimeError("resume record strict metric is inconsistent")
    if record.get("strict_urdf_pass") is not recomputed_strict:
        raise RuntimeError("resume record strict compatibility field is inconsistent")
    if expected_runtime_evaluation is None:
        return None
    expected_static = runtime_evaluation_binding(expected_runtime_evaluation)
    job_binding = record.get("job_runtime_binding")
    if not isinstance(job_binding, dict):
        raise RuntimeError("resume record runtime provenance is missing job binding")
    expected_keys = {"run_token", *RUNTIME_BINDING_STATIC_FIELDS}
    if set(job_binding) != expected_keys:
        raise RuntimeError("resume record runtime provenance job binding schema mismatch")
    run_token = job_binding.get("run_token")
    if not isinstance(run_token, str) or re.fullmatch(r"[0-9a-f]{32}", run_token) is None:
        raise RuntimeError("resume record runtime provenance run token is invalid")
    if {field: job_binding[field] for field in RUNTIME_BINDING_STATIC_FIELDS} != expected_static:
        raise RuntimeError("resume record runtime provenance static binding mismatch")
    origin = record.get("result_origin")
    worker_binding = record.get("worker_runtime_binding")
    if origin == "child_attested":
        if worker_binding != job_binding:
            raise RuntimeError("resume record runtime provenance worker binding mismatch")
    elif origin == "parent_synthesized":
        if "worker_runtime_binding" in record:
            raise RuntimeError("resume record runtime provenance has false worker attestation")
        if record.get("status") not in {"error", "timeout"}:
            raise RuntimeError("resume record runtime provenance parent status is invalid")
        if any(record["metrics"][name]["pass"] for name in METRIC_NAMES):
            raise RuntimeError("resume record runtime provenance parent metrics are not fail-closed")
    else:
        raise RuntimeError("resume record runtime provenance origin is invalid")
    return run_token


def validate_frozen_job_result(job: dict[str, Any], record: dict[str, Any]) -> None:
    expected_runtime = job.get("runtime_binding")
    if not isinstance(expected_runtime, dict):
        raise FatalRuntimeBindingError("job runtime binding is missing")
    if record.get("result_origin") != "child_attested":
        raise FatalRuntimeBindingError("worker runtime binding result origin mismatch")
    if record.get("job_runtime_binding") != expected_runtime:
        raise FatalRuntimeBindingError("worker runtime binding job echo mismatch")
    if record.get("worker_runtime_binding") != expected_runtime:
        raise FatalRuntimeBindingError("worker runtime binding mismatch")
    validate_resume_record(
        record,
        job["asset_id"],
        job["model_urdf_sha256"],
        job["package_binding"]["content_manifest_sha256"],
        job["manifest_content_sha256"],
        job,
    )


def summary_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    dataset = manifest["dataset"]
    selection = manifest["selection"]
    cohort_description = (
        f"Frozen cohort: N={summary['n_eval']}, exact frozen items order; "
        f"{selection['selection_policy']['algorithm']}; "
        f"salt={selection['selection_policy']['salt']}."
        if dataset == "PartNet-Mobility"
        else f"Frozen cohort: N={summary['n_eval']}, seed={selection['seed']}."
    )
    lines = [
        f"# {dataset} Table 2 URDF audit",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        cohort_description,
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    for name in METRIC_NAMES:
        result = summary["metrics"][name]
        percentage = "N/A" if result["rate"] is None else f"{100 * result['rate']:.2f}%"
        lines.append(
            f"| {name} | {result['passed']} / {result['denominator']} ({percentage}) |"
        )
    category_macro = summary["category_macro"]
    if category_macro["state"] == "evaluated":
        lines.extend([
            "",
            (
                "Category macro average: evaluated over "
                f"{category_macro['category_count']} observed {category_macro['category_field']} groups using an "
                "unweighted mean. This global fixed cohort is not category-balanced or a full release."
            ),
            "",
            "| Metric | Category macro |",
            "|---|---:|",
        ])
        for name in METRIC_NAMES:
            rate = category_macro["metrics"][name]["rate"]
            lines.append(f"| {name} | {'N/A' if rate is None else f'{100 * rate:.2f}%'} |")
        lines.append("")
    else:
        lines.extend([
            "",
            f"Category macro average: `not_evaluable` because {category_macro['reason']}.",
            "",
        ])
    return "\n".join(lines)


def prepare_output(output: Path | None, resume: bool, dataset: str = "Articraft-10K") -> Path:
    if output is None:
        dataset_slug = {
            "Articraft-10K": "articraft",
            "Artiverse": "artiverse",
            "PartNet-Mobility": "partnet_mobility",
            LAM_DATASET_NAME: "lam",
        }[dataset]
        output = DEFAULT_OUTPUT_PARENT / (
            f"table2_{dataset_slug}_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}_{uuid.uuid4().hex[:8]}"
        )
    output = output.resolve(strict=False)
    try:
        output.relative_to(REPO_ROOT.resolve())
    except ValueError as exc:
        raise RuntimeError(f"output must be inside the repository: {output}") from exc
    if output.exists() and not resume:
        raise FileExistsError(f"output root already exists; choose a unique path or use --resume: {output}")
    if resume and not output.is_dir():
        raise FileNotFoundError(f"resume output does not exist: {output}")
    output.mkdir(parents=True, exist_ok=resume)
    return output


def validate_formal_parent_runtime(args: argparse.Namespace) -> None:
    """Fail the formal launch before creating output when urdfpy is unavailable."""

    if args.mode != "formal" or args.no_standard_parser:
        return
    try:
        version = importlib.metadata.version("urdfpy")
    except importlib.metadata.PackageNotFoundError as exc:
        raise FatalRuntimeBindingError("formal standard parser urdfpy is unavailable") from exc
    if version != "0.0.22":
        raise FatalRuntimeBindingError(
            f"formal standard parser requires urdfpy 0.0.22, found {version!r}"
        )
    try:
        module = importlib.import_module("urdfpy")
        getattr(module, "URDF")
    except Exception as exc:  # noqa: BLE001
        raise FatalRuntimeBindingError("formal standard parser URDF import failed") from exc


def validate_partnet_mobility_launch_paths(args: argparse.Namespace) -> None:
    """Reject PartNet identity aliases before output ownership is established."""

    if args.dataset != "PartNet-Mobility":
        return
    source_root = partnet_mobility_checked_identity_path(
        args.source_root, field="source root"
    )
    if not source_root.is_dir():
        raise NotADirectoryError(source_root)
    if args.cohort_manifest is None:
        raise ValueError("PartNet-Mobility requires a cohort manifest")
    cohort_manifest = partnet_mobility_checked_identity_path(
        args.cohort_manifest, field="cohort manifest"
    )
    if not cohort_manifest.is_file():
        raise RuntimeError("PartNet-Mobility cohort manifest must be a regular file")


def validate_lam_launch_paths(args: argparse.Namespace) -> None:
    """Reject LAM source/cohort aliases before creating an output root."""

    if args.dataset != LAM_DATASET_NAME:
        return
    source_root = lam_checked_identity_path(args.source_root, field="source root")
    if not source_root.is_dir():
        raise NotADirectoryError(source_root)
    if args.cohort_manifest is None:
        raise ValueError("LAM released outputs requires a cohort asset_records.jsonl")
    cohort = lam_checked_identity_path(
        args.cohort_manifest, field="cohort asset_records.jsonl"
    )
    if not cohort.is_file():
        raise RuntimeError("LAM cohort asset_records.jsonl must be a regular file")


def bound_job_failure(job: dict[str, Any], reason: str, *, status: str = "error") -> dict[str, Any]:
    runtime_binding = job.get("runtime_binding")
    if not isinstance(runtime_binding, dict):
        raise FatalRuntimeBindingError("cannot bind parent failure without job runtime binding")
    primary_relative = job.get("primary_urdf_relative_path", "model.urdf")
    failed = failed_record(
        job["asset_id"],
        Path(job["package"]),
        reason,
        primary_urdf_relative_path=primary_relative,
    )
    failed["status"] = status
    failed["model_urdf_sha256"] = job["model_urdf_sha256"]
    failed["primary_urdf_sha256"] = job.get(
        "primary_urdf_sha256", job["model_urdf_sha256"]
    )
    failed["package_content_manifest_sha256"] = job["package_binding"][
        "content_manifest_sha256"
    ]
    failed["manifest_content_sha256"] = job["manifest_content_sha256"]
    failed["result_origin"] = "parent_synthesized"
    failed["job_runtime_binding"] = runtime_binding
    for field in FROZEN_SOURCE_METADATA_FIELDS:
        if field in job:
            failed[field] = job[field]
    return failed


def audit_frozen_job(job: dict[str, Any]) -> dict[str, Any]:
    if job.get("internal_test_action") == "sleep":
        time.sleep(float(job["sleep"]))
        return {"asset_id": job["asset_id"], "status": "completed"}
    if job.get("internal_test_action") == "echo_environment":
        return {
            "asset_id": job["asset_id"],
            "status": "completed",
            "thread_environment": {
                name: os.environ.get(name) for name in CHILD_THREAD_ENVIRONMENT
            },
        }
    validate_child_runtime_binding(job)

    def finish(record: dict[str, Any]) -> dict[str, Any]:
        observed_runtime = validate_child_runtime_binding(job)
        record["result_origin"] = "child_attested"
        record["job_runtime_binding"] = job["runtime_binding"]
        record["worker_runtime_binding"] = observed_runtime
        return record

    package = Path(job["package"])
    expected_binding = job["package_binding"]
    try:
        before_binding = package_binding(package)
    except Exception as exc:  # noqa: BLE001
        return finish(
            bound_job_failure(job, f"source_precheck_failed: {type(exc).__name__}: {exc}")
        )
    if before_binding != expected_binding:
        return finish(bound_job_failure(job, "source_changed_before_audit"))
    record = audit_asset_package(
        package,
        run_standard_parser=job["run_standard_parser"],
        asset_id=job["asset_id"],
        primary_urdf_relative_path=job.get("primary_urdf_relative_path", "model.urdf"),
    )
    try:
        after_binding = package_binding(package)
    except Exception as exc:  # noqa: BLE001
        return finish(
            bound_job_failure(job, f"source_postcheck_failed: {type(exc).__name__}: {exc}")
        )
    if after_binding != expected_binding:
        return finish(bound_job_failure(job, "source_changed_during_audit"))
    record["model_urdf_sha256"] = job["model_urdf_sha256"]
    record["primary_urdf_sha256"] = job.get(
        "primary_urdf_sha256", job["model_urdf_sha256"]
    )
    record["package_content_manifest_sha256"] = expected_binding["content_manifest_sha256"]
    record["manifest_content_sha256"] = job["manifest_content_sha256"]
    for field in FROZEN_SOURCE_METADATA_FIELDS:
        if field in job:
            record[field] = job[field]
    return finish(record)


def run_internal_child(job_path: Path, result_path: Path) -> int:
    job_path = job_path.resolve(strict=True)
    job_root = job_path.parent
    result_path = result_path.resolve(strict=False)
    if job_path.name != "job.json" or result_path.name != "result.json":
        raise RuntimeError("internal child paths must use exact job.json/result.json names")
    if result_path.parent != job_root or job_root.name.startswith("job_") is False:
        raise RuntimeError("internal child paths must share one owned job directory")
    job = json.loads(job_path.read_text(encoding="utf-8"))
    if not isinstance(job, dict) or not isinstance(job.get("asset_id"), str):
        raise RuntimeError("internal child job schema is invalid")
    record = audit_frozen_job(job)
    atomic_write_json(result_path, record)
    return 0


def acquire_output_lock(output: Path) -> Any:
    output = output.resolve(strict=True)
    lock_path = output / ".run.lock"
    flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(lock_path, flags, 0o600)
    except OSError as exc:
        raise RuntimeError(
            f"output lock must be a writable regular non-symlink file: {lock_path}"
        ) from exc
    try:
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            raise RuntimeError(
                f"output lock must be a writable regular non-symlink file: {lock_path}"
            )
        handle = os.fdopen(descriptor, "r+", encoding="utf-8")
        descriptor = -1
    finally:
        if descriptor >= 0:
            os.close(descriptor)
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        handle.close()
        raise RuntimeError(f"output root is already locked by another writer: {output}") from exc
    handle.seek(0)
    handle.truncate()
    handle.write(f"pid={os.getpid()}\n")
    handle.flush()
    os.fsync(handle.fileno())
    return handle


@contextmanager
def output_run_lock(output: Path) -> Iterable[Any]:
    handle = acquire_output_lock(output)
    try:
        yield handle
    finally:
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        handle.close()


def run_locked(args: argparse.Namespace, output: Path) -> Path:
    recover_stale_worker_scratch(output)
    manifest_path = output / "manifest.json"
    records_path = output / "asset_records.jsonl"
    checkpoint_path = output / "checkpoint.json"
    if args.resume:
        if manifest_path.is_symlink() or not manifest_path.is_file():
            raise RuntimeError(f"resume manifest must be a regular non-symlink file: {manifest_path}")
        manifest_bytes = read_regular_file_bytes_nofollow(
            manifest_path, label="resume manifest"
        )
        try:
            manifest = json.loads(manifest_bytes)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("resume manifest is not valid UTF-8 JSON") from exc
        if not isinstance(manifest, dict):
            raise RuntimeError("resume manifest must contain one JSON object")
        if manifest.get("manifest_content_sha256") != manifest_self_hash(manifest):
            raise RuntimeError("resume manifest self-hash mismatch")
        protocol_binding = validate_protocol_snapshot_binding(
            output, manifest.get("evaluation")
        )
    else:
        protocol_binding = freeze_protocol_snapshot(output)
    source_root = (
        args.source_root
        if args.dataset in {"PartNet-Mobility", LAM_DATASET_NAME}
        else args.source_root.resolve(strict=True)
    )
    evaluation_n = min(args.n, args.limit) if args.limit is not None else args.n
    if args.dataset == "Artiverse":
        if args.cohort_manifest is None:
            raise ValueError("Artiverse requires a cohort manifest")
        loaded = load_artiverse_cohort(
            source_root, args.cohort_manifest, formal=args.mode == "formal"
        )
        if evaluation_n > len(loaded["assets"]):
            raise ValueError(
                f"cannot evaluate {evaluation_n} Artiverse assets from a cohort of "
                f"{len(loaded['assets'])}"
            )
        run_loaded = {**loaded, "assets": loaded["assets"][:evaluation_n]}

        def fresh_manifest() -> dict[str, Any]:
            return build_artiverse_manifest(
                run_loaded,
                requested_n=args.n,
                limit=args.limit,
                standard_parser=not args.no_standard_parser,
                workers=args.workers,
                protocol_binding=protocol_binding,
                mode=args.mode,
                command=frozen_command(args),
            )

    elif args.dataset == "PartNet-Mobility":
        if args.cohort_manifest is None:
            raise ValueError("PartNet-Mobility requires a cohort manifest")
        loaded = load_partnet_mobility_cohort(
            source_root, args.cohort_manifest, formal=args.mode == "formal"
        )
        if evaluation_n > len(loaded["assets"]):
            raise ValueError(
                f"cannot evaluate {evaluation_n} PartNet-Mobility assets from a cohort of "
                f"{len(loaded['assets'])}"
            )
        run_loaded = {**loaded, "assets": loaded["assets"][:evaluation_n]}

        def fresh_manifest() -> dict[str, Any]:
            return build_partnet_mobility_manifest(
                run_loaded,
                requested_n=args.n,
                limit=args.limit,
                standard_parser=not args.no_standard_parser,
                workers=args.workers,
                protocol_binding=protocol_binding,
                mode=args.mode,
                command=frozen_command(args),
            )

    elif args.dataset == LAM_DATASET_NAME:
        if args.cohort_manifest is None:
            raise ValueError("LAM released outputs requires a cohort asset_records.jsonl")
        loaded = load_lam_cohort(
            source_root, args.cohort_manifest, formal=args.mode == "formal"
        )
        if evaluation_n > len(loaded["assets"]):
            raise ValueError(
                f"cannot evaluate {evaluation_n} LAM assets from a cohort of "
                f"{len(loaded['assets'])}"
            )
        run_loaded = {**loaded, "assets": loaded["assets"][:evaluation_n]}

        def fresh_manifest() -> dict[str, Any]:
            return build_lam_manifest(
                run_loaded,
                requested_n=args.n,
                limit=args.limit,
                standard_parser=not args.no_standard_parser,
                workers=args.workers,
                protocol_binding=protocol_binding,
                mode=args.mode,
                command=frozen_command(args),
            )

    else:
        all_ids = release_asset_ids(
            source_root, reject_non_directories=args.mode == "formal"
        )
        archive_identity = None
        if args.mode == "formal":
            inventory, _inventory_hash = load_inventory_entry()
            verification = inventory.get("verification", {})
            expected_counts = {
                "release_archives_expected": FORMAL_RELEASE_COUNT,
                "release_archives_present": FORMAL_RELEASE_COUNT,
                "extracted_object_directories": FORMAL_RELEASE_COUNT,
                "model_urdf_files": FORMAL_RELEASE_COUNT,
            }
            for name, expected in expected_counts.items():
                if verification.get(name) != expected:
                    raise RuntimeError(
                        f"formal inventory count mismatch for {name}: "
                        f"{verification.get(name)!r} != {expected}"
                    )
            if len(all_ids) != FORMAL_RELEASE_COUNT:
                raise RuntimeError(
                    f"formal local release count mismatch: {len(all_ids)} != "
                    f"{FORMAL_RELEASE_COUNT}"
                )
            archive_identity = archive_release_identity(source_root.parent, all_ids)
            if archive_identity["archive_count"] != FORMAL_RELEASE_COUNT:
                raise RuntimeError("formal archive count is not exactly 9996")
        selected_articraft_ids = select_asset_ids(all_ids, evaluation_n, args.seed)

        def fresh_manifest() -> dict[str, Any]:
            return build_manifest(
                source_root,
                all_ids,
                selected_articraft_ids,
                args.seed,
                args.n,
                args.limit,
                not args.no_standard_parser,
                args.workers,
                protocol_binding,
                mode=args.mode,
                command=frozen_command(args),
                archive_identity=archive_identity,
            )

    if args.resume:
        environment_path = output / "environment.json"
        if (
            not environment_path.is_file()
            or sha256_file(environment_path)
            != manifest.get("evaluation", {}).get("environment_file_sha256")
        ):
            raise RuntimeError("resume environment metadata hash mismatch")
        fresh = fresh_manifest()
        for field in ("source", "selection", "evaluation", "records"):
            if manifest.get(field) != fresh.get(field):
                raise RuntimeError(f"resume manifest binding mismatch: {field}")
        selected_ids = [row["asset_id"] for row in manifest["records"]]
    else:
        manifest = fresh_manifest()
        selected_ids = [row["asset_id"] for row in manifest["records"]]
        atomic_write_json(manifest_path, manifest)
        environment_path = output / "environment.json"
        atomic_write_json(environment_path, manifest["evaluation"]["environment"])
        if sha256_file(environment_path) != manifest["evaluation"]["environment_file_sha256"]:
            raise RuntimeError("environment metadata write verification failed")
        atomic_write_jsonl(records_path, [])
        atomic_write_json(checkpoint_path, {
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "completed": 0,
            "n_eval": len(selected_ids),
            "updated_at": utc_now(),
        })

    records = load_jsonl(records_path)
    by_id: dict[str, dict[str, Any]] = {}
    seen_runtime_tokens: set[str] = set()
    manifest_by_id = {row["asset_id"]: row for row in manifest["records"]}
    for row in records:
        asset_id = row.get("asset_id")
        if asset_id not in selected_ids or asset_id in by_id:
            raise RuntimeError(f"invalid or duplicate resume record: {asset_id!r}")
        manifest_record = manifest_by_id[asset_id]
        run_token = validate_resume_record(
            row,
            asset_id,
            manifest_record["model_urdf_sha256"],
            manifest_record["package_binding"]["content_manifest_sha256"],
            manifest["manifest_content_sha256"],
            manifest_record,
            manifest["evaluation"],
        )
        if run_token in seen_runtime_tokens:
            raise RuntimeError("duplicate resume record runtime provenance run token")
        if run_token is not None:
            seen_runtime_tokens.add(run_token)
        validate_manifest_record_source(manifest_record)
        by_id[asset_id] = row
    pending = [asset_id for asset_id in selected_ids if asset_id not in by_id]
    jobs = []
    issued_runtime_tokens = set(seen_runtime_tokens)
    for asset_id in pending:
        run_token = uuid.uuid4().hex
        while run_token in issued_runtime_tokens:
            run_token = uuid.uuid4().hex
        issued_runtime_tokens.add(run_token)
        jobs.append({
            **manifest_by_id[asset_id],
            "dataset": args.dataset,
            "output_root": str(output),
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "run_standard_parser": not args.no_standard_parser,
            "workers": args.workers,
            "run_token": run_token,
            "runtime_binding": frozen_worker_runtime_binding(manifest, run_token),
        })
    completion_order = max(
        (int(row.get("completion_order", 0)) for row in records), default=0
    )

    def checkpoint_result(record: dict[str, Any]) -> None:
        nonlocal completion_order
        completion_order += 1
        record["completion_order"] = completion_order
        by_id[record["asset_id"]] = record
        ordered_present = [by_id[asset_id] for asset_id in selected_ids if asset_id in by_id]
        atomic_write_jsonl(records_path, ordered_present)
        atomic_write_json(checkpoint_path, {
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "completed": len(ordered_present),
            "n_eval": len(selected_ids),
            "remaining": len(selected_ids) - len(ordered_present),
            "last_completed_asset_id": record["asset_id"],
            "completion_order": completion_order,
            "updated_at": utc_now(),
        })
        print(f"[{len(ordered_present)}/{len(selected_ids)}] {record['asset_id']}", flush=True)

    execute_killable_jobs(
        jobs,
        worker_scratch=output / ".worker_scratch",
        timeout_seconds=args.asset_timeout_seconds,
        max_workers=args.workers,
        on_result=checkpoint_result,
        timeout_factory=lambda job: bound_job_failure(
            job, f"asset_timeout_after_{args.asset_timeout_seconds:g}_seconds", status="timeout"
        ),
        exception_factory=lambda job, reason: bound_job_failure(
            job, f"worker_exception: {reason}"
        ),
        result_validator=validate_frozen_job_result,
    )

    validate_protocol_snapshot_binding(output, manifest["evaluation"])
    ordered = [by_id[asset_id] for asset_id in selected_ids]
    summary = aggregate_records(
        ordered,
        expected_n=len(selected_ids),
        category_field=(
            "raw_category" if args.dataset == "Artiverse"
            else "category"
            if args.dataset in {"PartNet-Mobility", LAM_DATASET_NAME}
            else None
        ),
    )
    summary.update({
        "schema_version": manifest["schema_version"],
        "status": "completed",
        "mode": args.mode,
        "classification": run_classification(args.mode),
        "dataset": args.dataset,
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "completed_at": utc_now(),
    })
    atomic_write_json(output / "summary.json", summary)
    atomic_write_text(output / "summary.md", summary_markdown(summary, manifest))
    atomic_write_json(checkpoint_path, {
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "completed": len(ordered),
        "n_eval": len(selected_ids),
        "remaining": 0,
        "state": "complete",
        "updated_at": utc_now(),
    })
    return output


def run(args: argparse.Namespace) -> Path:
    validate_run_contract(args)
    validate_formal_parent_runtime(args)
    validate_partnet_mobility_launch_paths(args)
    validate_lam_launch_paths(args)
    output = prepare_output(args.output, args.resume, args.dataset)
    with output_run_lock(output):
        return run_locked(args, output)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset",
        choices=("Articraft-10K", "Artiverse", "PartNet-Mobility", LAM_DATASET_NAME),
        default="Articraft-10K",
    )
    parser.add_argument("--mode", choices=("formal", "smoke"), default="formal")
    parser.add_argument("--source-root", type=Path)
    parser.add_argument("--cohort-manifest", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--n", type=int, default=DEFAULT_N)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--limit", type=int, help="freeze only this many assets for a smoke run")
    parser.add_argument(
        "--asset-timeout-seconds",
        type=float,
        default=ASSET_TIMEOUT_SECONDS,
        help="per-asset child-process timeout; formal mode freezes this at 120 seconds",
    )
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--no-standard-parser", action="store_true")
    args = parser.parse_args(argv)
    if args.source_root is None:
        args.source_root = {
            "Articraft-10K": DEFAULT_SOURCE_ROOT,
            "Artiverse": DEFAULT_ARTIVERSE_SOURCE_ROOT,
            "PartNet-Mobility": DEFAULT_PARTNET_MOBILITY_SOURCE_ROOT,
            LAM_DATASET_NAME: DEFAULT_LAM_SOURCE_ROOT,
        }[args.dataset]
    if args.cohort_manifest is None:
        if args.dataset == "Artiverse":
            args.cohort_manifest = DEFAULT_ARTIVERSE_COHORT_MANIFEST
        elif args.dataset == "PartNet-Mobility":
            args.cohort_manifest = DEFAULT_PARTNET_MOBILITY_COHORT_MANIFEST
        elif args.dataset == LAM_DATASET_NAME:
            args.cohort_manifest = DEFAULT_LAM_COHORT_RECORDS
    return args


def main(argv: list[str] | None = None) -> int:
    effective_argv = list(sys.argv[1:] if argv is None else argv)
    if effective_argv and effective_argv[0] == "--internal-child-job":
        if len(effective_argv) != 4 or effective_argv[2] != "--internal-child-result":
            print("ERROR: invalid internal child invocation", file=sys.stderr)
            return 2
        try:
            return run_internal_child(Path(effective_argv[1]), Path(effective_argv[3]))
        except FatalRuntimeBindingError as exc:
            print(f"FATAL_RUNTIME_BINDING: {exc}", file=sys.stderr)
            return RUNTIME_BINDING_FATAL_EXIT_CODE
        except Exception as exc:  # noqa: BLE001
            print(f"ERROR: internal child failed: {type(exc).__name__}: {exc}", file=sys.stderr)
            return 1
    try:
        output = run(parse_args(effective_argv))
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(json.dumps({"status": "completed", "output": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
