#!/usr/bin/env python3
"""Evaluate Table 1 on the frozen LAM released-output N=800 cohort."""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import csv
from datetime import datetime, timezone
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
from typing import Any
import uuid
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
DEFAULT_DATASET_ROOT = REPO / "exp/Articulated-Object-Code"
DEFAULT_INPUT_RECORDS = (
    REPO
    / "exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl"
)
DEFAULT_OUTPUT = REPO / "exp/runtime/table1_lam_released_outputs"
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"

FORMAL_RELEASE_COUNT = 3217
FORMAL_RELEASE_CATEGORY_COUNT = 787
FORMAL_EVAL_CATEGORY_COUNT = 305
FORMAL_N_EVAL = 800
FORMAL_SEED = 20260813
FORMAL_UPSTREAM_REVISION = "28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0"
FORMAL_INPUT_MANIFEST_FILE_SHA256 = (
    "7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951"
)
FORMAL_INPUT_MANIFEST_CONTENT_SHA256 = (
    "f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2"
)
FORMAL_INPUT_RECORDS_FILE_SHA256 = (
    "7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94"
)
FORMAL_RELEASE_MANIFEST_SHA256 = (
    "70216593ec02b71d596e456498ff9863ad0f8e519d5d27d2cf4f58792d412412"
)
FORMAL_DATASET_API_SHA256 = (
    "4ef6d64310968fab9e280ca19545ef0647e6c5e2eb81dbd681b5ca20fbbc727d"
)
FORMAL_SELECTED_ASSET_KEYS_SHA256 = (
    "643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3"
)
FORMAL_TABLE1_PROTOCOL_SHA256 = (
    "de6f83e93ddcc5f6414561aaa17e7dfd78b1f323b84ce1430cd4b3173707f813"
)
FORMAL_SHARED_METRIC_RUNNER_SHA256 = (
    "2b6aac690c5e1715e76ddda00686e76fb957e3ef8905c4d33072d038ce871bcf"
)


def _load_module(path: Path, name: str):
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load required module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


SHARED = _load_module(
    Path(__file__).with_name("run_table1_artiverse.py"),
    "table1_structural_metrics_shared",
)
TABLE3 = _load_module(
    Path(__file__).with_name("run_urdf_table3_lam.py"),
    "table3_lam_frozen_cohort_shared",
)


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return _canonical_sha256(payload)


def _ensure_no_symlink_components(path: Path, root: Path) -> None:
    try:
        relative = path.relative_to(root)
    except ValueError as error:
        raise ValueError(f"path escapes source root: {path}") from error
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise ValueError(f"package path contains symlink: {current}")


def _package_file_manifest(package: Path) -> list[dict[str, Any]]:
    if package.is_symlink() or not package.is_dir():
        raise ValueError(f"package must be a regular directory: {package}")
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if child.is_symlink():
                raise ValueError(
                    f"package contains directory symlink: {child.relative_to(package)}"
                )
            if not child.is_dir():
                raise ValueError(
                    f"package contains non-directory entry: {child.relative_to(package)}"
                )
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            try:
                canonical = path.resolve(strict=True)
                canonical.relative_to(package.resolve(strict=True))
            except (OSError, ValueError) as error:
                raise ValueError(f"package file escapes package: {relative}") from error
            if not canonical.is_file():
                raise ValueError(f"package entry is not a regular file: {relative}")
            before = canonical.stat()
            digest = SHARED.sha256_file(canonical)
            after = canonical.stat()
            stable_before = (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            stable_after = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
            if stable_before != stable_after:
                raise ValueError(f"package file changed while hashing: {relative}")
            rows.append(
                {
                    "path": relative,
                    "bytes": after.st_size,
                    "sha256": digest,
                }
            )
    return rows


def _package_binding(package: Path) -> dict[str, Any]:
    files = _package_file_manifest(package)
    return {
        "file_count": len(files),
        "total_bytes": sum(int(row["bytes"]) for row in files),
        "files": files,
        "content_manifest_sha256": _canonical_sha256(files),
    }


def _parse_json_object_bytes(payload: bytes, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{label} is invalid") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _parse_jsonl_bytes(payload: bytes) -> list[dict[str, Any]]:
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError("completion records are not UTF-8") from error
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(text.splitlines(), 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(
                f"completion records contain invalid JSON at line {line_number}"
            ) from error
        if not isinstance(value, dict):
            raise ValueError(f"completion record line {line_number} is not an object")
        rows.append(value)
    return rows


def _load_release_rows_from_bytes(
    payload: bytes,
    source_root: Path,
) -> list[dict[str, Any]]:
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError("release manifest is not UTF-8") from error
    rows: list[dict[str, Any]] = []
    seen_paths: set[str] = set()
    reader = csv.DictReader(io.StringIO(text, newline=""))
    required = {"object_release_id", "category", "tier", "rel_path", "n_movable"}
    if reader.fieldnames is None or not required.issubset(reader.fieldnames):
        raise ValueError(f"release manifest is missing columns: {sorted(required)}")
    for index, raw in enumerate(reader):
        relative = TABLE3._safe_relative_path(raw["rel_path"])
        relative_text = relative.as_posix()
        if relative_text in seen_paths:
            raise ValueError(f"duplicate release rel_path: {relative_text}")
        seen_paths.add(relative_text)
        tier = raw["tier"].strip()
        if tier not in {"viable", "loads_only", "broken"}:
            raise ValueError(f"unexpected release tier: {tier!r}")
        try:
            declared_hint = max(0, int(raw["n_movable"] or 0))
        except ValueError as error:
            raise ValueError(f"invalid n_movable for {relative_text}") from error
        urdf_path = source_root / relative / "generated.urdf"
        try:
            urdf_path.resolve(strict=False).relative_to(source_root)
        except ValueError as error:
            raise ValueError(f"release path escapes source root: {relative_text}") from error
        is_regular = urdf_path.is_file() and not urdf_path.is_symlink()
        rows.append(
            {
                "release_order": index,
                "asset_key": f"{tier}:{relative_text}",
                "object_release_id": raw["object_release_id"],
                "category": raw["category"],
                "tier": tier,
                "rel_path": relative_text,
                "declared_joint_count_hint": declared_hint,
                "urdf_path": str(urdf_path),
                "urdf_exists": is_regular,
                "urdf_sha256": SHARED.sha256_file(urdf_path) if is_regular else None,
            }
        )
    return rows


def _require_unchanged_bytes(path: Path, expected: bytes, *, label: str) -> None:
    try:
        current = path.read_bytes()
    except OSError as error:
        raise ValueError(f"{label} changed during cohort snapshot") from error
    if current != expected:
        raise ValueError(f"{label} changed during cohort snapshot")


def _validate_metric_contract(protocol: Path, *, formal: bool) -> dict[str, Any]:
    protocol = protocol.resolve(strict=True)
    if protocol.is_symlink() or not protocol.is_file():
        raise ValueError("Table 1 protocol must be a regular non-symlink file")
    protocol_bytes = protocol.read_bytes()
    protocol_sha256 = hashlib.sha256(protocol_bytes).hexdigest()
    shared_path = Path(SHARED.__file__).resolve(strict=True)
    shared_sha256 = SHARED.sha256_file(shared_path)
    if formal and protocol_sha256 != FORMAL_TABLE1_PROTOCOL_SHA256:
        raise ValueError("formal Table 1 protocol hash mismatch")
    if formal and shared_sha256 != FORMAL_SHARED_METRIC_RUNNER_SHA256:
        raise ValueError("formal shared metric runner hash mismatch")
    return {
        "protocol": str(protocol),
        "protocol_bytes": protocol_bytes,
        "protocol_sha256": protocol_sha256,
        "shared_metric_runner": str(shared_path),
        "shared_metric_runner_sha256": shared_sha256,
    }


def _validate_formal_contract(
    *,
    manifest: dict[str, Any],
    manifest_file_sha256: str,
    records_file_sha256: str,
    expected_n: int,
) -> None:
    if expected_n != FORMAL_N_EVAL:
        raise ValueError(f"formal LAM evaluation requires N={FORMAL_N_EVAL}")
    if manifest_file_sha256 != FORMAL_INPUT_MANIFEST_FILE_SHA256:
        raise ValueError("formal input manifest file hash mismatch")
    if manifest.get("manifest_content_sha256") != FORMAL_INPUT_MANIFEST_CONTENT_SHA256:
        raise ValueError("formal input manifest content hash mismatch")
    if records_file_sha256 != FORMAL_INPUT_RECORDS_FILE_SHA256:
        raise ValueError("formal input completion records file hash mismatch")
    source = manifest.get("source", {})
    selection = manifest.get("selection", {})
    expected = {
        "release count": (source.get("n_release"), FORMAL_RELEASE_COUNT),
        "release manifest hash": (
            source.get("release_manifest_sha256"),
            FORMAL_RELEASE_MANIFEST_SHA256,
        ),
        "dataset API hash": (
            source.get("dataset_api_sha256"),
            FORMAL_DATASET_API_SHA256,
        ),
        "upstream revision": (
            source.get("upstream_revision"),
            FORMAL_UPSTREAM_REVISION,
        ),
        "selection seed": (selection.get("seed"), FORMAL_SEED),
        "selected asset keys hash": (
            selection.get("selected_asset_keys_sha256"),
            FORMAL_SELECTED_ASSET_KEYS_SHA256,
        ),
    }
    for label, (actual, wanted) in expected.items():
        if actual != wanted:
            raise ValueError(f"formal {label} mismatch")


def _matching_frozen_row(
    frozen: dict[str, Any],
    fresh: dict[str, Any],
) -> bool:
    keys = (
        "release_order",
        "asset_key",
        "object_release_id",
        "category",
        "tier",
        "rel_path",
        "declared_joint_count_hint",
        "urdf_exists",
        "urdf_sha256",
    )
    if any(frozen.get(key) != fresh.get(key) for key in keys):
        return False
    try:
        return Path(str(frozen.get("urdf_path"))).resolve(strict=True) == Path(
            str(fresh.get("urdf_path"))
        ).resolve(strict=True)
    except OSError:
        return False


def _matching_completion_record(
    completion: dict[str, Any],
    frozen: dict[str, Any],
) -> bool:
    keys = (
        "asset_key",
        "object_release_id",
        "category",
        "tier",
        "rel_path",
        "selection_rank",
        "selection_hash",
        "urdf_sha256",
    )
    return all(completion.get(key) == frozen.get(key) for key in keys)


def load_frozen_cohort(
    records_path: Path,
    *,
    dataset_root: Path,
    expected_n: int,
    formal: bool,
) -> dict[str, Any]:
    dataset_root = dataset_root.resolve(strict=True)
    if dataset_root.is_symlink():
        raise ValueError("dataset root must not be a symlink")
    records_path = records_path.resolve(strict=True)
    records_bytes = records_path.read_bytes()
    records_file_sha256 = hashlib.sha256(records_bytes).hexdigest()
    manifest_path = records_path.with_name("manifest.json").resolve(strict=True)
    manifest_bytes = manifest_path.read_bytes()
    manifest_file_sha256 = hashlib.sha256(manifest_bytes).hexdigest()
    manifest = _parse_json_object_bytes(manifest_bytes, label="input manifest")
    if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
        raise ValueError("input manifest self-hash mismatch")
    if (
        manifest.get("dataset") != "LAM released outputs (Articulated-Object-Code)"
        or manifest.get("classification") != "FORMAL"
    ):
        raise ValueError("input manifest is not a formal LAM released-output cohort")
    if formal:
        _validate_formal_contract(
            manifest=manifest,
            manifest_file_sha256=manifest_file_sha256,
            records_file_sha256=records_file_sha256,
            expected_n=expected_n,
        )

    source_root = (dataset_root / "released_outputs").resolve(strict=True)
    release_manifest = (dataset_root / "manifest.csv").resolve(strict=True)
    dataset_api_path = (dataset_root / "dataset_api.json").resolve(strict=True)
    release_manifest_bytes = release_manifest.read_bytes()
    dataset_api_bytes = dataset_api_path.read_bytes()
    source = manifest.get("source", {})
    expected_paths = {
        "source root": (source.get("source_root"), source_root),
        "release manifest": (source.get("release_manifest"), release_manifest),
        "dataset API": (source.get("dataset_api"), dataset_api_path),
    }
    for label, (raw, expected_path) in expected_paths.items():
        try:
            actual = Path(str(raw)).resolve(strict=True)
        except OSError as error:
            raise ValueError(f"input manifest {label} is unavailable") from error
        if actual != expected_path:
            raise ValueError(f"input manifest {label} does not match --dataset-root")
    if hashlib.sha256(release_manifest_bytes).hexdigest() != source.get(
        "release_manifest_sha256"
    ):
        raise ValueError("release manifest file hash mismatch")
    if hashlib.sha256(dataset_api_bytes).hexdigest() != source.get("dataset_api_sha256"):
        raise ValueError("dataset API file hash mismatch")
    dataset_api = _parse_json_object_bytes(dataset_api_bytes, label="dataset API")
    if dataset_api.get("sha") != source.get("upstream_revision"):
        raise ValueError("dataset API upstream revision mismatch")

    fresh_rows = _load_release_rows_from_bytes(release_manifest_bytes, source_root)
    if len(fresh_rows) != source.get("n_release"):
        raise ValueError("release asset count differs from the frozen manifest")
    if len({str(row["asset_key"]) for row in fresh_rows}) != len(fresh_rows):
        raise ValueError("release manifest asset keys are not unique")
    fresh_tier_counts = dict(sorted(Counter(str(row["tier"]) for row in fresh_rows).items()))
    if fresh_tier_counts != source.get("tier_counts"):
        raise ValueError("release tier counts differ from the frozen manifest")
    candidate_pool_sha256 = _canonical_sha256(
        [str(row["asset_key"]) for row in sorted(fresh_rows, key=lambda row: row["asset_key"])]
    )
    if candidate_pool_sha256 != source.get("candidate_pool_sha256"):
        raise ValueError("release candidate pool differs from the frozen manifest")
    categories = [str(row.get("category", "")).strip() for row in fresh_rows]
    if not all(categories):
        raise ValueError("release manifest contains an empty category")
    release_category_count = len(set(categories))

    frozen_records = manifest.get("records")
    selection = manifest.get("selection", {})
    if not isinstance(frozen_records, list) or len(frozen_records) != expected_n:
        raise ValueError(f"expected exactly {expected_n} frozen records")
    if selection.get("n_eval") != expected_n:
        raise ValueError("selection n_eval differs from expected N")
    if selection.get("quality_label_blind") is not True:
        raise ValueError("input selection was not quality-label blind")
    expected_ranks = list(range(1, expected_n + 1))
    ranks = [row.get("selection_rank") for row in frozen_records]
    if ranks != expected_ranks:
        raise ValueError("frozen records are not in complete selection-rank order")
    asset_keys = [str(row.get("asset_key", "")) for row in frozen_records]
    if not all(asset_keys) or len(set(asset_keys)) != expected_n:
        raise ValueError("frozen asset keys must be unique and nonempty")
    if _canonical_sha256(asset_keys) != selection.get("selected_asset_keys_sha256"):
        raise ValueError("selected asset keys hash mismatch")
    fresh_by_key = {str(row["asset_key"]): row for row in fresh_rows}
    for frozen in frozen_records:
        fresh = fresh_by_key.get(str(frozen.get("asset_key")))
        if fresh is None or not _matching_frozen_row(frozen, fresh):
            raise ValueError(f"frozen release row mismatch: {frozen.get('asset_key')}")

    completion_rows = _parse_jsonl_bytes(records_bytes)
    if len(completion_rows) != expected_n:
        raise ValueError(f"expected exactly {expected_n} completion records")
    completion_by_key: dict[str, dict[str, Any]] = {}
    for completion in completion_rows:
        asset_key = str(completion.get("asset_key", ""))
        if not asset_key or asset_key in completion_by_key:
            raise ValueError("completion records contain an empty or duplicate asset key")
        completion_by_key[asset_key] = completion
    if set(completion_by_key) != set(asset_keys):
        raise ValueError("completion records do not match the frozen selected assets")

    assets: list[dict[str, Any]] = []
    for frozen in frozen_records:
        asset_key = str(frozen["asset_key"])
        completion = completion_by_key[asset_key]
        if not _matching_completion_record(completion, frozen):
            raise ValueError(f"completion record mismatch: {asset_key}")
        if completion.get("manifest_content_sha256") != manifest.get(
            "manifest_content_sha256"
        ):
            raise ValueError(f"completion manifest binding mismatch: {asset_key}")
        relative = TABLE3._safe_relative_path(str(frozen["rel_path"]))
        package = source_root / relative
        _ensure_no_symlink_components(package, source_root)
        if not package.is_dir():
            raise ValueError(f"selected package is missing: {asset_key}")
        primary = package / "generated.urdf"
        if primary.is_symlink() or not primary.is_file():
            raise ValueError(f"generated.urdf must be a regular non-symlink file: {asset_key}")
        if primary.resolve(strict=True) != Path(str(frozen["urdf_path"])).resolve(strict=True):
            raise ValueError(f"primary URDF path mismatch: {asset_key}")
        if SHARED.sha256_file(primary) != frozen.get("urdf_sha256"):
            raise ValueError(f"primary URDF hash mismatch: {asset_key}")
        package_binding = _package_binding(package)
        assets.append(
            {
                "asset_id": asset_key,
                "asset_key": asset_key,
                "object_release_id": str(frozen["object_release_id"]),
                "selection_index": int(frozen["selection_rank"]) - 1,
                "selection_rank": int(frozen["selection_rank"]),
                "selection_hash": str(frozen["selection_hash"]),
                "raw_category": str(frozen["category"]),
                "tier": str(frozen["tier"]),
                "rel_path": str(frozen["rel_path"]),
                "package": str(package),
                "primary_urdf_relative_path": "generated.urdf",
                "primary_urdf_sha256": str(frozen["urdf_sha256"]),
                "package_binding": package_binding,
                "source_table3_status": completion.get("status"),
                "source_table3_error": completion.get("error"),
                "source_table3_parse_success": completion.get("parse_success"),
                "source_table3_strict_kinematic_pass": completion.get(
                    "strict_kinematic_pass"
                ),
                "source_table3_record_sha256": _canonical_sha256(completion),
            }
        )

    eval_category_count = len({row["raw_category"] for row in assets})
    if formal:
        if len(fresh_rows) != FORMAL_RELEASE_COUNT:
            raise ValueError("formal release asset count mismatch")
        if release_category_count != FORMAL_RELEASE_CATEGORY_COUNT:
            raise ValueError("formal release category count mismatch")
        if eval_category_count != FORMAL_EVAL_CATEGORY_COUNT:
            raise ValueError("formal evaluation category count mismatch")

    _require_unchanged_bytes(
        manifest_path,
        manifest_bytes,
        label="input manifest",
    )
    _require_unchanged_bytes(
        records_path,
        records_bytes,
        label="completion records",
    )
    _require_unchanged_bytes(
        release_manifest,
        release_manifest_bytes,
        label="release manifest",
    )
    _require_unchanged_bytes(
        dataset_api_path,
        dataset_api_bytes,
        label="dataset API",
    )

    return {
        "manifest": manifest,
        "manifest_path": str(manifest_path),
        "manifest_file_sha256": manifest_file_sha256,
        "records_path": str(records_path),
        "records_file_sha256": records_file_sha256,
        "dataset_root": str(dataset_root),
        "source_root": str(source_root),
        "release_manifest": str(release_manifest),
        "release_manifest_sha256": source["release_manifest_sha256"],
        "dataset_api": str(dataset_api_path),
        "dataset_api_sha256": source["dataset_api_sha256"],
        "source_repo_id": dataset_api.get("id"),
        "source_revision": dataset_api.get("sha"),
        "release_asset_count": len(fresh_rows),
        "release_category_count": release_category_count,
        "eval_category_count": eval_category_count,
        "candidate_pool_sha256": candidate_pool_sha256,
        "tier_counts": fresh_tier_counts,
        "assets": assets,
    }


def _record_template(identity: dict[str, Any]) -> dict[str, Any]:
    return {
        "asset_id": identity["asset_id"],
        "asset_key": identity["asset_key"],
        "object_release_id": identity["object_release_id"],
        "selection_index": identity["selection_index"],
        "selection_rank": identity["selection_rank"],
        "selection_hash": identity["selection_hash"],
        "raw_category": identity["raw_category"],
        "tier": identity["tier"],
        "rel_path": identity["rel_path"],
        "package": identity["package"],
        "primary_urdf_relative_path": identity["primary_urdf_relative_path"],
        "primary_urdf_sha256": identity["primary_urdf_sha256"],
        "package_content_manifest_sha256": identity["package_binding"][
            "content_manifest_sha256"
        ],
        "source_table3_status": identity["source_table3_status"],
        "source_table3_error": identity["source_table3_error"],
        "source_table3_parse_success": identity["source_table3_parse_success"],
        "source_table3_strict_kinematic_pass": identity[
            "source_table3_strict_kinematic_pass"
        ],
        "source_table3_record_sha256": identity["source_table3_record_sha256"],
        "status": None,
        "parse_success": False,
        "link_count": None,
        "joint_count": None,
        "joint_type_counts": None,
        "non_fixed_joint_count": None,
        "valid_tree": False,
        "topology_hash": None,
        "fingerprint_complete": False,
        "package_fingerprint": None,
        "referenced_resource_count": None,
        "missing_resources": [],
        "error": None,
    }


def _validate_asset(identity: dict[str, Any]) -> None:
    package = Path(identity["package"])
    primary = package / identity["primary_urdf_relative_path"]
    if primary.is_symlink() or not primary.is_file():
        raise ValueError("generated.urdf is no longer a regular non-symlink file")
    if SHARED.sha256_file(primary) != identity["primary_urdf_sha256"]:
        raise ValueError("generated.urdf changed after cohort freeze")
    if _package_binding(package) != identity["package_binding"]:
        raise ValueError("package changed after Table 1 snapshot")


def _fingerprint_observation(urdf_path: Path) -> dict[str, Any]:
    try:
        return {"result": SHARED.fingerprint_package(urdf_path), "error": None}
    except Exception as error:
        return {"result": None, "error": f"{type(error).__name__}: {error}"}


def evaluate_package(identity: dict[str, Any]) -> dict[str, Any]:
    record = _record_template(identity)
    urdf_path = Path(identity["package"]) / identity["primary_urdf_relative_path"]
    try:
        _validate_asset(identity)
        record.update(SHARED.analyze_urdf(urdf_path))
        record["parse_success"] = True
    except (ET.ParseError, OSError, ValueError, RuntimeError) as error:
        failed = _record_template(identity)
        failed["status"] = "EVALUATION_FAILED"
        failed["error"] = f"{type(error).__name__}: {error}"
        return failed

    first = _fingerprint_observation(urdf_path)
    try:
        _validate_asset(identity)
        second = _fingerprint_observation(urdf_path)
        if second != first:
            raise ValueError("simulation-resource closure changed during evaluation")
        _validate_asset(identity)
    except (OSError, ValueError, RuntimeError) as error:
        failed = _record_template(identity)
        failed["status"] = "EVALUATION_FAILED"
        failed["error"] = f"{type(error).__name__}: {error}"
        return failed

    if first["error"] is not None:
        record["status"] = "EVALUATED_FINGERPRINT_INCOMPLETE"
        record["error"] = first["error"]
        return record
    fingerprint = first["result"]
    record["fingerprint_complete"] = bool(fingerprint["complete"])
    record["package_fingerprint"] = fingerprint["fingerprint"]
    record["referenced_resource_count"] = fingerprint["resource_count"]
    record["missing_resources"] = fingerprint["missing_resources"]
    if record["fingerprint_complete"]:
        record["status"] = "EVALUATED"
    else:
        record["status"] = "EVALUATED_FINGERPRINT_INCOMPLETE"
        record["error"] = "one or more referenced simulation resources are unavailable"
    return record


def _evaluate_package_fail_closed(identity: dict[str, Any]) -> dict[str, Any]:
    try:
        return evaluate_package(identity)
    except Exception as error:
        record = _record_template(identity)
        record["status"] = "EVALUATION_FAILED"
        record["error"] = f"{type(error).__name__}: {error}"
        return record


def _post_validate_records(
    assets: list[dict[str, Any]],
    records: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    if len(assets) != len(records):
        raise ValueError("evaluation record count differs from the frozen cohort")
    validated: list[dict[str, Any]] = []
    for identity, record in zip(assets, records, strict=True):
        if (
            record.get("asset_key") != identity["asset_key"]
            or record.get("selection_rank") != identity["selection_rank"]
        ):
            raise ValueError("evaluation records are not in frozen selection order")
        try:
            _validate_asset(identity)
        except (OSError, ValueError, RuntimeError) as error:
            failed = _record_template(identity)
            failed["status"] = "EVALUATION_FAILED"
            failed["package_postcheck"] = "FAILED"
            failed["error"] = f"{type(error).__name__}: {error}"
            validated.append(failed)
            continue
        stable = dict(record)
        stable["package_postcheck"] = "PASSED"
        validated.append(stable)
    return validated


def aggregate_lam_records(
    records: list[dict[str, Any]],
    *,
    release_asset_count: int,
    release_category_count: int,
) -> dict[str, Any]:
    summary = SHARED.aggregate_records(
        records,
        release_asset_count=release_asset_count,
        release_category_count=release_category_count,
    )
    summary["cohort"]["cohort_type"] = (
        "FROZEN_RANDOM_SAMPLE_NOT_CATEGORY_BALANCED_ALL_RELEASE_TIERS"
    )
    return summary


def _percentage(value: float | None) -> str:
    return "N/E" if value is None else f"{100.0 * value:.2f}%"


def _report(summary: dict[str, Any], run_manifest: dict[str, Any]) -> str:
    cohort = summary["cohort"]
    links = summary["links_per_asset"]
    movable = summary["movable_joints_per_asset"]
    multi = summary["multi_joint_assets"]
    topology = summary["unique_topologies"]
    duplicate = summary["exact_duplicate_rate"]
    return "\n".join(
        (
            "# LAM Released Outputs Table 1",
            "",
            "| Dataset | N_release | N_eval | Categories (release / eval) | Links/Asset (mean / median / P90) | Movable Joints/Asset (mean / median / P90) | Multi-joint Assets | Unique Topologies | Exact Duplicate Rate |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
            (
                f"| LAM released outputs | {cohort['N_release']:,} | {cohort['N_eval']} | "
                f"{cohort['release_raw_categories']} / {cohort['eval_raw_categories']} | "
                f"{links['mean']:.2f} / {links['median']:g} / {links['p90_nearest_rank']} | "
                f"{movable['mean']:.2f} / {movable['median']:g} / "
                f"{movable['p90_nearest_rank']} | {_percentage(multi['rate'])} | "
                f"{_percentage(topology['rate'])} (n={topology['denominator']}) | "
                f"{_percentage(duplicate['rate'])} (n={duplicate['denominator']}) |"
            ),
            "",
            (
                f"Frozen Table 3 manifest: "
                f"`{run_manifest['input_manifest_content_sha256']}`; "
                f"seed {run_manifest['seed']}."
            ),
            "",
        )
    )


def _run_to_output(args: argparse.Namespace, output: Path) -> dict[str, Any]:
    metric_contract = _validate_metric_contract(args.protocol, formal=args.formal)
    cohort = load_frozen_cohort(
        args.input_records,
        dataset_root=args.dataset_root,
        expected_n=args.expected_n,
        formal=args.formal,
    )
    assets = cohort["assets"]
    input_manifest = cohort["manifest"]
    run_manifest = {
        "schema_version": 1,
        "dataset": "LAM released outputs",
        "release_status": "LOCAL_RELEASE_BOUND_TO_UPSTREAM_REVISION",
        "paper_reported_assets": None,
        "paper_reported_categories": None,
        "N_release": cohort["release_asset_count"],
        "release_raw_category_count": cohort["release_category_count"],
        "N_eval": len(assets),
        "eval_raw_category_count": cohort["eval_category_count"],
        "cohort_type": "FROZEN_RANDOM_SAMPLE_NOT_CATEGORY_BALANCED_ALL_RELEASE_TIERS",
        "seed": input_manifest["selection"]["seed"],
        "selection_protocol": input_manifest["selection"]["algorithm"],
        "selected_asset_keys_sha256": input_manifest["selection"][
            "selected_asset_keys_sha256"
        ],
        "source_failures_retained_without_replacement": True,
        "input_records": cohort["records_path"],
        "input_records_file_sha256": cohort["records_file_sha256"],
        "input_manifest": cohort["manifest_path"],
        "input_manifest_file_sha256": cohort["manifest_file_sha256"],
        "input_manifest_content_sha256": input_manifest["manifest_content_sha256"],
        "input_manifest_protocol_sha256": input_manifest.get("evaluation", {}).get(
            "protocol_sha256"
        ),
        "dataset_root": cohort["dataset_root"],
        "source_root": cohort["source_root"],
        "source_repo_id": cohort["source_repo_id"],
        "source_revision": cohort["source_revision"],
        "release_manifest": cohort["release_manifest"],
        "release_manifest_sha256": cohort["release_manifest_sha256"],
        "dataset_api": cohort["dataset_api"],
        "dataset_api_sha256": cohort["dataset_api_sha256"],
        "candidate_pool_sha256": cohort["candidate_pool_sha256"],
        "tier_counts": cohort["tier_counts"],
        "category_mapping_policy": "exact manifest.csv category joined by tier:rel_path",
        "package_binding_scope": "all regular files below each selected asset directory",
        "package_binding_time_scope": (
            "snapshot before asset evaluation plus global recheck after all asset tasks "
            "finish and before aggregation"
        ),
        "historical_package_provenance_limit": (
            "The Table 3 freeze authenticated primary URDF bytes but did not preserve "
            "contemporaneous full-package bindings for non-URDF files."
        ),
        "protocol": metric_contract["protocol"],
        "protocol_sha256": metric_contract["protocol_sha256"],
        "protocol_snapshot": "protocol_snapshot.md",
        "runner": str(Path(__file__).resolve()),
        "runner_sha256": SHARED.sha256_file(Path(__file__).resolve()),
        "shared_metric_runner": metric_contract["shared_metric_runner"],
        "shared_metric_runner_sha256": metric_contract[
            "shared_metric_runner_sha256"
        ],
        "frozen_cohort_runner": str(Path(TABLE3.__file__).resolve()),
        "frozen_cohort_runner_sha256": SHARED.sha256_file(Path(TABLE3.__file__).resolve()),
        "topology_protocol": SHARED.TOPOLOGY_PROTOCOL,
        "fingerprint_protocol": SHARED.FINGERPRINT_PROTOCOL,
        "workers": args.workers,
        "formal": bool(args.formal),
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "assets": [
            {
                key: row[key]
                for key in (
                    "selection_index",
                    "selection_rank",
                    "asset_key",
                    "object_release_id",
                    "selection_hash",
                    "raw_category",
                    "tier",
                    "rel_path",
                    "package",
                    "primary_urdf_relative_path",
                    "primary_urdf_sha256",
                    "package_binding",
                    "source_table3_status",
                    "source_table3_error",
                    "source_table3_parse_success",
                    "source_table3_strict_kinematic_pass",
                    "source_table3_record_sha256",
                )
            }
            for row in assets
        ],
    }
    SHARED.write_json(output / "manifest.json", run_manifest)
    (output / "protocol_snapshot.md").write_bytes(metric_contract["protocol_bytes"])

    if args.workers == 1:
        records = [_evaluate_package_fail_closed(row) for row in assets]
    else:
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            records = list(executor.map(_evaluate_package_fail_closed, assets))
    records = _post_validate_records(assets, records)
    SHARED.write_jsonl(output / "asset_records.jsonl", records)
    summary = aggregate_lam_records(
        records,
        release_asset_count=cohort["release_asset_count"],
        release_category_count=cohort["release_category_count"],
    )
    summary["status_counts"] = dict(
        sorted(Counter(str(row["status"]) for row in records).items())
    )
    summary["source_table3_status_counts"] = dict(
        sorted(Counter(str(row["source_table3_status"]) for row in records).items())
    )
    summary["package_postcheck_counts"] = dict(
        sorted(Counter(str(row["package_postcheck"]) for row in records).items())
    )
    summary["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
    SHARED.write_json(output / "summary.json", summary)
    SHARED._atomic_write_text(output / "report.md", _report(summary, run_manifest))

    artifact_files = (
        "manifest.json",
        "asset_records.jsonl",
        "summary.json",
        "report.md",
        "protocol_snapshot.md",
    )
    artifact_manifest = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "files": {
            name: {
                "bytes": (output / name).stat().st_size,
                "sha256": SHARED.sha256_file(output / name),
            }
            for name in artifact_files
        },
    }
    SHARED.write_json(output / "artifact_manifest.json", artifact_manifest)
    if Path(metric_contract["protocol"]).read_bytes() != metric_contract["protocol_bytes"]:
        raise ValueError("Table 1 protocol changed during evaluation")
    if (
        SHARED.sha256_file(Path(metric_contract["shared_metric_runner"]))
        != metric_contract["shared_metric_runner_sha256"]
    ):
        raise ValueError("shared metric runner changed during evaluation")
    return summary


def run(args: argparse.Namespace) -> dict[str, Any]:
    output = args.output.absolute()
    dataset_root = args.dataset_root.resolve(strict=True)
    try:
        output.resolve(strict=False).relative_to(dataset_root)
    except ValueError:
        pass
    else:
        raise ValueError("output must not be inside the LAM dataset root")
    with SHARED._output_lock(output):
        staging = output.parent / (
            f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}"
        )
        staging.mkdir(parents=True, exist_ok=False)
        try:
            summary = _run_to_output(args, staging)
            SHARED._verify_staged_artifacts(staging)
            SHARED._publish_staged_output(staging, output)
            return summary
        finally:
            if staging.exists():
                import shutil

                shutil.rmtree(staging)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--input-records", type=Path, default=DEFAULT_INPUT_RECORDS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--expected-n", type=int, default=FORMAL_N_EVAL)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--formal", action="store_true")
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    if args.expected_n <= 0 or args.workers <= 0:
        raise SystemExit("--expected-n and --workers must be positive")
    summary = run(args)
    print(
        json.dumps(
            {
                "state": "COMPLETE",
                "N_release": summary["cohort"]["N_release"],
                "N_eval": summary["cohort"]["N_eval"],
                "N_parse": summary["cohort"]["N_parse"],
                "output": str(args.output.absolute()),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
