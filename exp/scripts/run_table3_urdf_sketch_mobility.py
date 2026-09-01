#!/usr/bin/env python3
"""Evaluate Table 3 on the frozen SketchMobility Table 1/2 N=800 cohort.

This is a dataset-specific provenance and orchestration adapter. Joint
kinematics are evaluated by the unchanged shared Table 3 FK core through
the existing fresh-interpreter Ours-500K child harness.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import platform
import shutil
import subprocess
import sys
from typing import Any
import uuid
import xml.etree.ElementTree as ET

import numpy


SCRIPT_PATH = Path(__file__).resolve()
REPO = SCRIPT_PATH.parents[2]
CORE_PATH = SCRIPT_PATH.with_name("run_urdf_table3_lam.py")
CHILD_HARNESS_PATH = SCRIPT_PATH.with_name("run_urdf_table3_ours_500k.py")
VERIFIER_PATH = SCRIPT_PATH.with_name("verify_table3_urdf_sketch_mobility.py")
PROTOCOL_PATH = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
DEFAULT_DATASET_ROOT = REPO / "exp/SketchMobility"
DEFAULT_TABLE1_RECEIPT = (
    REPO / "exp/runtime/table1_sketch_mobility_rerun_20260821T021838Z"
)
DEFAULT_TABLE2_MANIFEST = (
    REPO
    / "exp/runtime/table2_urdf_sketch_mobility_table1cohort_n800_20260821T035015Z/manifest.json"
)
DEFAULT_OUTPUT_PARENT = REPO / "exp/runtime"

DATASET_NAME = "SketchMobility"
FORMAL_N_RELEASE = 4956
FORMAL_N_EVAL = 800
FORMAL_J_EVAL = 1824
FORMAL_RELEASE_CATEGORY_COUNT = 70
FORMAL_EVAL_CATEGORY_COUNT = 67
FORMAL_TABLE2_MANIFEST_FILE_SHA256 = (
    "0be3e21f079bd86ba9ab680f1d709dd676b623bea01d8e43a3db85943a64a8e5"
)
FORMAL_TABLE2_MANIFEST_CONTENT_SHA256 = (
    "a4cd711698d46ce25fa306bd1f1aa751f26d8277c62f592ba9e40b16f08ee2ff"
)
FORMAL_TABLE1_MANIFEST_SHA256 = (
    "081e9e9125f8945cad67a751949e659f6d4e73817704c07cd3fcd4b657ffc696"
)
FORMAL_TABLE1_RECORDS_SHA256 = (
    "4c7dc19d2a0558e07e6a0f42ce12bd96e7a8b199849c885374a3e396f7b16cca"
)
FORMAL_RELEASE_ROSTER_SHA256 = (
    "9b3f3776162e59baa0b73996b398f97bc52b632b966cedef7bb83ff99acdb765"
)
FORMAL_RELEASE_MANIFEST_SHA256 = (
    "5b4b0891bafeba5029e5e1dd71042e5be8543eccb2990edfbc9aba71a1ac56fb"
)
FORMAL_RELEASE_UNIVERSE_SHA256 = (
    "a9c128d24ab9cba03d593ecff17f1e7284ef1f74f09f69939cbef7c0cc8af346"
)
FORMAL_SELECTION_PROTOCOL = "sketch-mobility-table1-global-sample-v1"
FORMAL_SEED = "arti-skill-table1-sketch-mobility-n800-v1"
FORMAL_PYTHON_VERSION = "3.12.3"
FORMAL_NUMPY_VERSION = "2.5.1"
FORMAL_CORE_SHA256 = "0da075f077ce13c78bb6b4ee66b0abe77668ccf7bb3c105660b321e667fc2acf"
DEFAULT_SAMPLES = 21
DEFAULT_WORKERS = 4
DEFAULT_TIMEOUT_SECONDS = 120.0
PROTOCOL_SNAPSHOT_NAME = "protocol_snapshot.md"
SOURCE_SNAPSHOT_DIR = "source_snapshot"
RECORD_JOURNAL_DIR = ".record_journal"
EFFECTIVE_CHILD_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}


def _load_module(path: Path, name: str):
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import required module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


HARNESS = _load_module(CHILD_HARNESS_PATH, "table3_sketch_child_harness")
CORE = HARNESS.core


def _safe_package(dataset_root: Path, asset_id: str) -> Path:
    relative = PurePosixPath(asset_id)
    if (
        relative.is_absolute()
        or len(relative.parts) != 4
        or relative.parts[0] != "data"
        or any(part in {"", ".", ".."} for part in relative.parts)
    ):
        raise ValueError(f"invalid SketchMobility asset ID: {asset_id!r}")
    unresolved = dataset_root / Path(*relative.parts)
    cursor = dataset_root
    for part in relative.parts:
        cursor = cursor / part
        if cursor.is_symlink():
            raise ValueError(f"asset package path contains a symlink: {asset_id}")
    package = unresolved.resolve(strict=True)
    package.relative_to(dataset_root)
    if not package.is_dir():
        raise ValueError(f"asset package is not a regular directory: {asset_id}")
    return package


def _table1_receipt_hashes(receipt: Path) -> dict[str, str]:
    observed = {
        "manifest": CORE.sha256_file(receipt / "manifest.json"),
        "asset_records": CORE.sha256_file(receipt / "asset_records.jsonl"),
        "release_roster": CORE.sha256_file(receipt / "release_roster.jsonl"),
    }
    expected = {
        "manifest": FORMAL_TABLE1_MANIFEST_SHA256,
        "asset_records": FORMAL_TABLE1_RECORDS_SHA256,
        "release_roster": FORMAL_RELEASE_ROSTER_SHA256,
    }
    if observed != expected:
        raise ValueError(f"Table 1 receipt hash mismatch: {observed} != {expected}")
    return observed


def load_frozen_cohort(
    table2_manifest: Path,
    table1_receipt: Path,
    dataset_root: Path,
    *,
    formal: bool,
) -> dict[str, Any]:
    """Cross-bind the exact Table 2 receipt to its Table 1 membership authority."""
    table2_manifest = table2_manifest.resolve(strict=True)
    table1_receipt = table1_receipt.resolve(strict=True)
    dataset_root = dataset_root.resolve(strict=True)
    table2_file_hash = CORE.sha256_file(table2_manifest)
    if formal and table2_file_hash != FORMAL_TABLE2_MANIFEST_FILE_SHA256:
        raise ValueError(
            "Table 2 manifest file hash mismatch: "
            f"{table2_file_hash} != {FORMAL_TABLE2_MANIFEST_FILE_SHA256}"
        )
    table2 = json.loads(table2_manifest.read_text(encoding="utf-8"))
    if table2.get("manifest_content_sha256") != CORE._manifest_self_hash(table2):
        raise ValueError("Table 2 manifest self-hash mismatch")
    if (
        table2.get("dataset") != DATASET_NAME
        or table2.get("classification") != "FORMAL"
        or table2.get("mode") != "formal"
    ):
        raise ValueError("cohort manifest must be the formal SketchMobility Table 2 run")
    if formal and table2.get("manifest_content_sha256") != FORMAL_TABLE2_MANIFEST_CONTENT_SHA256:
        raise ValueError("Table 2 manifest content hash mismatch")

    receipt_hashes = _table1_receipt_hashes(table1_receipt)
    table1 = json.loads((table1_receipt / "manifest.json").read_text(encoding="utf-8"))
    source = table2.get("source")
    if not isinstance(source, dict):
        raise ValueError("Table 2 source binding is missing")
    expected_source = {
        "table1_manifest_sha256": FORMAL_TABLE1_MANIFEST_SHA256,
        "table1_asset_records_sha256": FORMAL_TABLE1_RECORDS_SHA256,
        "table1_release_roster_sha256": FORMAL_RELEASE_ROSTER_SHA256,
        "selection_protocol": FORMAL_SELECTION_PROTOCOL,
        "seed": FORMAL_SEED,
    }
    observed_source = {key: source.get(key) for key in expected_source}
    if observed_source != expected_source:
        raise ValueError(f"Table 2 source identity mismatch: {observed_source}")
    if Path(str(source.get("table1_receipt", ""))).resolve(strict=True) != table1_receipt:
        raise ValueError("Table 2 receipt path does not match the fixed Table 1 receipt")
    release = source.get("release_reverification")
    expected_release = {
        "release_manifest_sha256": FORMAL_RELEASE_MANIFEST_SHA256,
        "release_roster_sha256": FORMAL_RELEASE_ROSTER_SHA256,
        "release_universe_sha256": FORMAL_RELEASE_UNIVERSE_SHA256,
        "release_roster_status_counts": {"COMPLETE": FORMAL_N_RELEASE},
    }
    if not isinstance(release, dict) or {
        key: release.get(key) for key in expected_release
    } != expected_release:
        raise ValueError("Table 2 release reverification identity mismatch")

    table1_rows = table1.get("assets")
    table2_rows = table2.get("assets")
    if (
        not isinstance(table1_rows, list)
        or not isinstance(table2_rows, list)
        or len(table1_rows) != FORMAL_N_EVAL
        or len(table2_rows) != FORMAL_N_EVAL
        or table2.get("n_eval") != FORMAL_N_EVAL
    ):
        raise ValueError("Table 1/Table 2 cohort must contain exactly 800 assets")

    rows: list[dict[str, Any]] = []
    for rank, (table1_row, table2_row) in enumerate(
        zip(table1_rows, table2_rows, strict=True), start=1
    ):
        if not isinstance(table1_row, dict) or not isinstance(table2_row, dict):
            raise ValueError(f"cohort row {rank} is not an object")
        expected = {
            "asset_id": table1_row.get("asset_id"),
            "selection_rank": rank,
            "selection_hash": table1_row.get("selection_hash"),
            "source": table1_row.get("source"),
            "category": table1_row.get("category"),
            "source_category": table1_row.get("source_category"),
            "display_id": table1_row.get("display_id"),
            "primary_urdf_sha256": table1_row.get("mobility_urdf_sha256"),
        }
        observed = {key: table2_row.get(key) for key in expected}
        if observed != expected or table2_row.get("selection_index") != rank - 1:
            raise ValueError(f"Table 1/Table 2 cohort mismatch at rank {rank}")
        asset_id = str(expected["asset_id"] or "")
        expected_package = _safe_package(dataset_root, asset_id)
        if Path(str(table2_row.get("package", ""))).resolve(strict=True) != expected_package:
            raise ValueError(f"Table 2 package path mismatch: {asset_id}")
        if table2_row.get("primary_urdf_relative_path") != "mobility.urdf":
            raise ValueError(f"Table 2 primary URDF path mismatch: {asset_id}")
        rows.append({**table2_row})

    eval_category_count = len({str(row["source_category"]) for row in rows})
    if formal and eval_category_count != FORMAL_EVAL_CATEGORY_COUNT:
        raise ValueError("formal cohort must contain 67 source/category groups")
    if formal:
        required_paths = {
            "dataset_root": DEFAULT_DATASET_ROOT.resolve(strict=True),
            "table1_receipt": DEFAULT_TABLE1_RECEIPT.resolve(strict=True),
            "table2_manifest": DEFAULT_TABLE2_MANIFEST.resolve(strict=True),
        }
        observed_paths = {
            "dataset_root": dataset_root,
            "table1_receipt": table1_receipt,
            "table2_manifest": table2_manifest,
        }
        if observed_paths != required_paths:
            raise ValueError(f"formal cohort paths differ: {observed_paths}")
    return {
        "rows": rows,
        "n_release": FORMAL_N_RELEASE,
        "release_category_count": FORMAL_RELEASE_CATEGORY_COUNT,
        "eval_category_count": eval_category_count,
        "table2_manifest_path": str(table2_manifest),
        "table2_manifest_file_sha256": table2_file_hash,
        "table2_manifest_content_sha256": table2["manifest_content_sha256"],
        "table1_receipt": str(table1_receipt),
        "table1_receipt_hashes": receipt_hashes,
        "cohort_type": source.get("cohort_type"),
        "release_reverification": release,
    }


def _package_file_manifest(package: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if child.is_symlink():
                raise ValueError(f"package contains directory symlink: {child}")
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            canonical = path.resolve(strict=True)
            canonical.relative_to(package)
            if not canonical.is_file():
                raise ValueError(f"package entry is not a regular file: {relative}")
            rows.append(
                {
                    "path": relative,
                    "bytes": canonical.stat().st_size,
                    "sha256": CORE.sha256_file(canonical),
                }
            )
    return rows


def _package_binding(package: Path) -> dict[str, Any]:
    files = _package_file_manifest(package)
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": CORE.canonical_sha256(files),
    }


def _declared_joint_hint(urdf_path: Path) -> int:
    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:  # noqa: BLE001
        return 0
    return sum(node.get("type", "") != "fixed" for node in root.findall("joint"))


def build_assets(rows: list[dict[str, Any]], dataset_root: Path) -> list[dict[str, Any]]:
    dataset_root = dataset_root.resolve(strict=True)
    assets: list[dict[str, Any]] = []
    for row in rows:
        asset_id = str(row["asset_id"])
        package = _safe_package(dataset_root, asset_id)
        urdf_path = package / "mobility.urdf"
        if urdf_path.is_symlink() or not urdf_path.is_file():
            raise ValueError(f"mobility.urdf is missing or symlinked: {asset_id}")
        urdf_hash = CORE.sha256_file(urdf_path)
        if urdf_hash != row["primary_urdf_sha256"]:
            raise ValueError(f"mobility.urdf hash mismatch: {asset_id}")
        binding = _package_binding(package)
        assets.append(
            {
                "asset_key": asset_id,
                "asset_id": asset_id,
                "category": str(row["source_category"]),
                "raw_category": str(row["source_category"]),
                "selection_index": int(row["selection_index"]),
                "selection_rank": int(row["selection_rank"]),
                "selection_hash": str(row["selection_hash"]),
                "package": str(package),
                "package_binding": binding,
                "package_content_manifest_sha256": binding["content_manifest_sha256"],
                "urdf_path": str(urdf_path.resolve(strict=True)),
                "urdf_sha256": urdf_hash,
                "declared_joint_count_hint": _declared_joint_hint(urdf_path),
            }
        )
    return assets


def validate_formal_runtime(python_version: str, numpy_version: str) -> None:
    if python_version != FORMAL_PYTHON_VERSION:
        raise ValueError(f"formal mode requires Python {FORMAL_PYTHON_VERSION}")
    if numpy_version != FORMAL_NUMPY_VERSION:
        raise ValueError(f"formal mode requires numpy {FORMAL_NUMPY_VERSION}")
    observed_core = CORE.sha256_file(CORE_PATH.resolve(strict=True))
    if observed_core != FORMAL_CORE_SHA256:
        raise ValueError(
            f"formal mode requires shared FK core {FORMAL_CORE_SHA256}; "
            f"observed {observed_core}"
        )


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    temporary = path.parent / f".{path.name}.{uuid.uuid4().hex}.tmp"
    try:
        temporary.write_bytes(payload)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def freeze_protocol_snapshot(output: Path) -> dict[str, str]:
    source = PROTOCOL_PATH.resolve(strict=True)
    if source.is_symlink() or not source.is_file():
        raise ValueError(f"protocol source is not a regular file: {source}")
    payload = source.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    snapshot = output / PROTOCOL_SNAPSHOT_NAME
    if snapshot.exists() or snapshot.is_symlink():
        raise ValueError(f"protocol snapshot already exists: {snapshot}")
    _atomic_write_bytes(snapshot, payload)
    return {
        "protocol_source_path": str(source),
        "protocol_source_sha256_at_freeze": digest,
        "protocol_path": str(snapshot.resolve(strict=True)),
        "protocol_sha256": digest,
    }


def freeze_source_snapshots(output: Path) -> dict[str, dict[str, str]]:
    snapshot_root = output / SOURCE_SNAPSHOT_DIR
    snapshot_root.mkdir(parents=False, exist_ok=False)
    sources = {
        "adapter": SCRIPT_PATH,
        "child_harness": CHILD_HARNESS_PATH,
        "core_evaluator": CORE_PATH,
        "independent_verifier": VERIFIER_PATH,
    }
    result: dict[str, dict[str, str]] = {}
    for role, source_raw in sources.items():
        source = source_raw.resolve(strict=True)
        if source.is_symlink() or not source.is_file():
            raise ValueError(f"source is not a regular file: {source}")
        before = CORE.sha256_file(source)
        destination = snapshot_root / source.name
        shutil.copyfile(source, destination)
        after = CORE.sha256_file(source)
        snapshot_hash = CORE.sha256_file(destination)
        if before != after or snapshot_hash != before:
            raise RuntimeError(f"source changed while it was frozen: {source}")
        result[role] = {
            "source_path": str(source),
            "path": str(destination.resolve(strict=True)),
            "sha256": snapshot_hash,
        }
    if result["core_evaluator"]["sha256"] != CORE.sha256_file(CORE_PATH):
        raise RuntimeError("frozen core evaluator identity mismatch")
    return result


def validate_protocol_snapshot_binding(
    output: Path, evaluation: dict[str, Any]
) -> dict[str, str]:
    fields = (
        "protocol_source_path",
        "protocol_source_sha256_at_freeze",
        "protocol_path",
        "protocol_sha256",
    )
    binding = {field: evaluation.get(field) for field in fields}
    if not all(isinstance(value, str) and value for value in binding.values()):
        raise RuntimeError("protocol snapshot binding is incomplete")
    expected = output.resolve(strict=True) / PROTOCOL_SNAPSHOT_NAME
    supplied = Path(binding["protocol_path"]).resolve(strict=True)
    if supplied != expected:
        raise RuntimeError(f"protocol snapshot path mismatch: {supplied} != {expected}")
    payload_hash = CORE.sha256_file(supplied)
    if (
        payload_hash != binding["protocol_sha256"]
        or binding["protocol_source_sha256_at_freeze"] != binding["protocol_sha256"]
    ):
        raise RuntimeError("protocol snapshot hash mismatch")
    return binding


def _config(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "protocol_id": "urdf-sim-ready-table3-sketch-mobility-table1-cohort-v1",
        "samples_per_joint": args.samples,
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
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "workers": args.workers,
        "primary_urdf_relative_path": "mobility.urdf",
        "category_policy": "exact source/category pair; no semantic merging",
    }


def build_manifest(
    args: argparse.Namespace,
    loaded: dict[str, Any],
    assets: list[dict[str, Any]],
    protocol_binding: dict[str, str],
    source_snapshots: dict[str, dict[str, str]],
) -> dict[str, Any]:
    config = _config(args)
    environment = CORE._environment()
    manifest = {
        "schema_version": "1.0.0-sketch-mobility",
        "dataset": DATASET_NAME,
        "classification": "FORMAL" if args.mode == "formal" else "NON_FORMAL_SMOKE",
        "created_at": CORE.utc_now(),
        "source": {
            "dataset_root": str(args.dataset_root.resolve(strict=True)),
            "n_release": loaded["n_release"],
            "table1_receipt": loaded["table1_receipt"],
            "table1_receipt_hashes": loaded["table1_receipt_hashes"],
            "table2_manifest_path": loaded["table2_manifest_path"],
            "table2_manifest_file_sha256": loaded["table2_manifest_file_sha256"],
            "table2_manifest_content_sha256": loaded["table2_manifest_content_sha256"],
            "cohort_type": loaded["cohort_type"],
            "release_reverification": loaded["release_reverification"],
            "release_category_count": loaded["release_category_count"],
            "eval_category_count": loaded["eval_category_count"],
            "category_mapping_policy": "exact declared source/category pair; no external join",
        },
        "selection": {
            "algorithm": "exact existing Table 2 manifest .assets[] / Table 1 selection-rank order",
            "source_protocol": FORMAL_SELECTION_PROTOCOL,
            "seed": FORMAL_SEED,
            "requested_limit": args.limit,
            "n_eval": len(assets),
            "selected_asset_ids_sha256": CORE.canonical_sha256(
                [row["asset_id"] for row in assets]
            ),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
        },
        "evaluation": {
            **protocol_binding,
            "source_snapshots": source_snapshots,
            "adapter_path": source_snapshots["adapter"]["path"],
            "adapter_sha256": source_snapshots["adapter"]["sha256"],
            "core_evaluator_path": source_snapshots["core_evaluator"]["path"],
            "core_evaluator_sha256": source_snapshots["core_evaluator"]["sha256"],
            "child_harness_path": source_snapshots["child_harness"]["path"],
            "child_harness_sha256": source_snapshots["child_harness"]["sha256"],
            "independent_verifier_path": source_snapshots["independent_verifier"]["path"],
            "independent_verifier_sha256": source_snapshots["independent_verifier"]["sha256"],
            "effective_child_environment": EFFECTIVE_CHILD_ENVIRONMENT,
            "effective_child_environment_sha256": CORE.canonical_sha256(
                EFFECTIVE_CHILD_ENVIRONMENT
            ),
            "config": config,
            "config_sha256": CORE.canonical_sha256(config),
            "environment": environment,
            "environment_sha256": CORE.canonical_sha256(environment),
        },
        "records": assets,
    }
    manifest["manifest_content_sha256"] = CORE._manifest_self_hash(manifest)
    return manifest


def _failed_bound_record(
    job: dict[str, Any], reason: str, harness: Any = HARNESS
) -> dict[str, Any]:
    record = CORE.failed_record(job["asset_key"], job["declared_joint_count_hint"], reason)
    return harness._bind_record(record, job)


def _record_binding(job: dict[str, Any]) -> dict[str, Any]:
    return {
        "asset_key": job["asset_key"],
        "asset_id": job["asset_id"],
        "selection_rank": job["selection_rank"],
        "selection_hash": job["selection_hash"],
        "package_content_manifest_sha256": job["package_content_manifest_sha256"],
        "urdf_sha256": job["urdf_sha256"],
        "manifest_content_sha256": job["manifest_content_sha256"],
    }


def _validate_child_record(
    record: dict[str, Any], job: dict[str, Any], harness: Any = HARNESS
) -> dict[str, Any]:
    expected = _record_binding(job)
    if {key: record.get(key) for key in expected} != expected:
        return _failed_bound_record(
            job, "child result provenance binding mismatch", harness
        )
    record["result_origin"] = "frozen_fresh_interpreter_child_harness"
    record["effective_child_environment_sha256"] = job[
        "effective_child_environment_sha256"
    ]
    return record


def _write_journal_record(journal: Path, record: dict[str, Any]) -> None:
    rank = int(record["selection_rank"])
    path = journal / f"{rank:04d}.json"
    if path.exists() or path.is_symlink():
        raise RuntimeError(f"journal record already exists: {path}")
    CORE.atomic_write_json(path, record)


def load_journal_records(
    journal: Path, jobs: list[dict[str, Any]]
) -> dict[str, dict[str, Any]]:
    by_key: dict[str, dict[str, Any]] = {}
    by_rank = {int(job["selection_rank"]): job for job in jobs}
    for path in sorted(journal.glob("*.json")):
        if path.is_symlink() or not path.is_file():
            raise RuntimeError(f"invalid journal entry: {path}")
        try:
            rank = int(path.stem)
        except ValueError as exc:
            raise RuntimeError(f"invalid journal filename: {path.name}") from exc
        job = by_rank.get(rank)
        if job is None or path.name != f"{rank:04d}.json":
            raise RuntimeError(f"journal rank is outside the frozen cohort: {path.name}")
        record = json.loads(path.read_text(encoding="utf-8"))
        expected = _record_binding(job)
        expected.update(
            {
                "result_origin": "frozen_fresh_interpreter_child_harness",
                "effective_child_environment_sha256": job[
                    "effective_child_environment_sha256"
                ],
            }
        )
        if {key: record.get(key) for key in expected} != expected:
            raise RuntimeError(f"journal record binding mismatch: {path.name}")
        key = job["asset_key"]
        if key in by_key:
            raise RuntimeError(f"duplicate journal asset: {key}")
        by_key[key] = record
    return by_key


def _execute_jobs(
    jobs: list[dict[str, Any]],
    scratch: Path,
    journal: Path,
    *,
    harness: Any,
    existing: dict[str, dict[str, Any]],
    workers: int,
    timeout_seconds: float,
    checkpoint_path: Path,
    stop_after: int | None = None,
) -> dict[str, dict[str, Any]]:
    by_key = dict(existing)
    status_counts: Counter[str] = Counter(
        str(record.get("status")) for record in by_key.values()
    )
    pending = [job for job in jobs if job["asset_key"] not in by_key]
    newly_completed = 0
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(harness._execute_job, job, scratch, timeout_seconds): job
            for job in pending
        }
        for future in as_completed(futures):
            job = futures[future]
            try:
                record = _validate_child_record(future.result(), job, harness)
            except Exception as exc:  # noqa: BLE001
                record = _failed_bound_record(
                    job,
                    f"parent worker exception: {type(exc).__name__}: {exc}",
                    harness,
                )
                record["result_origin"] = "frozen_fresh_interpreter_child_harness"
                record["effective_child_environment_sha256"] = job[
                    "effective_child_environment_sha256"
                ]
            _write_journal_record(journal, record)
            by_key[job["asset_key"]] = record
            status_counts[str(record.get("status"))] += 1
            completed = len(by_key)
            CORE.atomic_write_json(
                checkpoint_path,
                {
                    "state": "running" if completed < len(jobs) else "aggregating",
                    "completed": completed,
                    "remaining": len(jobs) - completed,
                    "n_eval": len(jobs),
                    "status_counts": dict(sorted(status_counts.items())),
                    "last_completed_asset_key": job["asset_key"],
                    "manifest_content_sha256": job["manifest_content_sha256"],
                    "updated_at": CORE.utc_now(),
                },
            )
            print(
                f"[{completed}/{len(jobs)}] {job['asset_key']} {record['status']}",
                flush=True,
            )
            newly_completed += 1
            if stop_after is not None and newly_completed >= stop_after:
                raise KeyboardInterrupt("test interruption")
    return by_key


def _summary_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    lines = [
        "# SketchMobility Table 3 Kinematic Executability",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            f"Exact frozen Table 1/2 cohort: N_eval={summary['n_eval']} from "
            f"N_release={manifest['source']['n_release']}; J_eval={summary['j_eval']}. "
            "Existing selection-rank order was preserved without resampling."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    for metric in CORE.JOINT_RATE_METRICS:
        value = summary["metrics"][metric]
        rate = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        lines.append(f"| {metric} | {value['passed']} / {value['denominator']} ({rate}) |")
    roundtrip = summary["metrics"]["fk_roundtrip_error"]
    lines.append(
        "| fk_roundtrip_error | "
        f"translation={roundtrip['max_normalized_translation']}; "
        f"rotation_rad={roundtrip['max_rotation_rad']}; "
        f"coverage={roundtrip['measured_joint_count']} / {roundtrip['denominator']} "
        f"({roundtrip['status']}) |"
    )
    strict = summary["metrics"]["strict_kinematic_pass"]
    strict_rate = "N/A" if strict["rate"] is None else f"{100 * strict['rate']:.2f}%"
    lines.extend(
        [
            f"| strict_kinematic_pass | {strict['passed']} / {strict['denominator']} ({strict_rate}) |",
            "",
            "This evaluates executable declared kinematics only, not semantic joints, collision safety, dynamics, or real-world fidelity.",
            "",
        ]
    )
    return "\n".join(lines)


def _write_artifact_manifest(output: Path) -> None:
    names = (
        "manifest.json",
        "asset_records.jsonl",
        "summary.json",
        "summary.md",
        "environment.json",
        "checkpoint.json",
        PROTOCOL_SNAPSHOT_NAME,
        "verification.json",
        f"{SOURCE_SNAPSHOT_DIR}/{SCRIPT_PATH.name}",
        f"{SOURCE_SNAPSHOT_DIR}/{CHILD_HARNESS_PATH.name}",
        f"{SOURCE_SNAPSHOT_DIR}/{CORE_PATH.name}",
        f"{SOURCE_SNAPSHOT_DIR}/{VERIFIER_PATH.name}",
    )
    artifact = {
        "schema_version": 1,
        "created_at_utc": CORE.utc_now(),
        "files": {
            name: {
                "bytes": (output / name).stat().st_size,
                "sha256": CORE.sha256_file(output / name),
            }
            for name in names
        },
    }
    CORE.atomic_write_json(output / "artifact_manifest.json", artifact)


def verify_artifacts(output: Path) -> None:
    artifact = json.loads((output / "artifact_manifest.json").read_text(encoding="utf-8"))
    files = artifact.get("files")
    if not isinstance(files, dict) or not files:
        raise ValueError("artifact manifest has no files")
    for name, expected in files.items():
        path = output / name
        if not path.is_file():
            raise ValueError(f"artifact is missing: {name}")
        if path.stat().st_size != expected.get("bytes"):
            raise ValueError(f"artifact byte count mismatch: {name}")
        if CORE.sha256_file(path) != expected.get("sha256"):
            raise ValueError(f"artifact hash mismatch: {name}")
    expected_paths = set(files) | {"artifact_manifest.json"}
    observed_paths = {
        path.relative_to(output).as_posix()
        for path in output.rglob("*")
        if path.is_file() or path.is_symlink()
    }
    if observed_paths != expected_paths:
        unexpected = sorted(observed_paths - expected_paths)
        missing = sorted(expected_paths - observed_paths)
        raise ValueError(
            f"unexpected receipt files: unexpected={unexpected}, missing={missing}"
        )


@contextmanager
def _output_lock(output: Path):
    output.parent.mkdir(parents=True, exist_ok=True)
    lock_path = output.parent / f".{output.name}.lock"
    handle = lock_path.open("a+", encoding="utf-8")
    try:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"output is locked by another run: {output}") from exc
        handle.seek(0)
        handle.truncate()
        handle.write(json.dumps({"pid": os.getpid(), "output": str(output)}) + "\n")
        handle.flush()
        yield
    finally:
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        handle.close()


def _resume_index_path(output: Path) -> Path:
    return output.parent / f".{output.name}.resume.json"


def _validate_managed_version(output: Path, version: Path) -> Path:
    runs_root = output.parent / f".{output.name}.runs"
    runs_root = runs_root.resolve(strict=True)
    version = version.resolve(strict=True)
    relative = version.relative_to(runs_root)
    if (
        len(relative.parts) != 1
        or not relative.name.startswith("run.")
        or len(relative.name) != 36
        or not version.is_dir()
    ):
        raise RuntimeError("resume target is not an immediate managed run directory")
    return version


def _allocate_fresh_version(output: Path) -> tuple[Path, Path]:
    if os.path.lexists(output):
        raise RuntimeError(f"output already exists; refusing to overwrite: {output}")
    resume_index = _resume_index_path(output)
    if os.path.lexists(resume_index):
        raise RuntimeError(f"unfinished run already exists; use --resume: {resume_index}")
    runs_root = output.parent / f".{output.name}.runs"
    runs_root.mkdir(parents=True, exist_ok=True)
    version = runs_root / f"run.{uuid.uuid4().hex}"
    version.mkdir(parents=False, exist_ok=False)
    CORE.atomic_write_json(
        resume_index,
        {"schema_version": 1, "output": str(output), "version": str(version)},
    )
    return version.resolve(strict=True), resume_index


def _load_resume_version(output: Path) -> tuple[Path, Path]:
    if os.path.lexists(output):
        raise RuntimeError(f"output already exists; nothing to resume: {output}")
    resume_index = _resume_index_path(output)
    if resume_index.is_symlink() or not resume_index.is_file():
        raise RuntimeError(f"resume index is missing: {resume_index}")
    value = json.loads(resume_index.read_text(encoding="utf-8"))
    if value.get("output") != str(output):
        raise RuntimeError("resume index output binding mismatch")
    version = _validate_managed_version(output, Path(str(value.get("version", ""))))
    return version, resume_index


def _publish_final_version(version: Path, output: Path, resume_index: Path) -> None:
    _validate_managed_version(output, version)
    if os.path.lexists(output):
        raise RuntimeError(f"output already exists; refusing to overwrite: {output}")
    output.symlink_to(os.path.relpath(version, output.parent))
    resume_index.unlink()


def _validate_frozen_sources(
    output: Path, declared: dict[str, dict[str, str]]
) -> None:
    expected = {
        "adapter": SCRIPT_PATH.name,
        "child_harness": CHILD_HARNESS_PATH.name,
        "core_evaluator": CORE_PATH.name,
        "independent_verifier": VERIFIER_PATH.name,
    }
    if set(declared) != set(expected):
        raise RuntimeError("resume source snapshot roles mismatch")
    for role, name in expected.items():
        path = Path(declared[role]["path"]).resolve(strict=True)
        if path != output / SOURCE_SNAPSHOT_DIR / name or path.is_symlink():
            raise RuntimeError(f"resume source snapshot path mismatch: {role}")
        if CORE.sha256_file(path) != declared[role]["sha256"]:
            raise RuntimeError(f"resume source snapshot hash mismatch: {role}")
        source = Path(declared[role]["source_path"]).resolve(strict=True)
        if CORE.sha256_file(source) != declared[role]["sha256"]:
            raise RuntimeError(f"resume live source differs from frozen snapshot: {role}")


def _validate_resume_manifest(
    args: argparse.Namespace,
    output: Path,
    manifest: dict[str, Any],
    loaded: dict[str, Any],
    assets: list[dict[str, Any]],
) -> None:
    if manifest.get("manifest_content_sha256") != CORE._manifest_self_hash(manifest):
        raise RuntimeError("resume manifest self-hash mismatch")
    if manifest.get("classification") != (
        "FORMAL" if args.mode == "formal" else "NON_FORMAL_SMOKE"
    ):
        raise RuntimeError("resume classification mismatch")
    if manifest.get("records") != assets:
        raise RuntimeError("resume asset/package binding mismatch")
    expected_source = {
        "n_release": loaded["n_release"],
        "table1_receipt": loaded["table1_receipt"],
        "table1_receipt_hashes": loaded["table1_receipt_hashes"],
        "table2_manifest_path": loaded["table2_manifest_path"],
        "table2_manifest_file_sha256": loaded["table2_manifest_file_sha256"],
        "table2_manifest_content_sha256": loaded["table2_manifest_content_sha256"],
    }
    observed_source = {
        key: manifest.get("source", {}).get(key) for key in expected_source
    }
    if observed_source != expected_source:
        raise RuntimeError("resume upstream receipt binding mismatch")
    if manifest.get("evaluation", {}).get("config") != _config(args):
        raise RuntimeError("resume evaluation config mismatch")
    validate_protocol_snapshot_binding(output, manifest["evaluation"])
    _validate_frozen_sources(output, manifest["evaluation"]["source_snapshots"])


def _run_independent_verifier(output: Path, manifest: dict[str, Any]) -> None:
    verifier = Path(
        manifest["evaluation"]["source_snapshots"]["independent_verifier"]["path"]
    ).resolve(strict=True)
    environment = dict(os.environ)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    process = subprocess.run(
        [
            sys.executable,
            str(verifier),
            "--output-root",
            str(output),
            "--write-receipt",
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
        env=environment,
    )
    if process.returncode != 0:
        detail = (process.stdout + "\n" + process.stderr)[-8000:]
        raise RuntimeError(f"independent verification failed: {detail}")
    receipt = json.loads((output / "verification.json").read_text(encoding="utf-8"))
    if receipt.get("status") != "PASS" or not all(receipt.get("checks", {}).values()):
        raise RuntimeError("independent verification receipt is not PASS")


def _run_to_output(
    args: argparse.Namespace, output: Path, *, resume: bool
) -> dict[str, Any]:
    loaded = load_frozen_cohort(
        args.table2_manifest,
        args.table1_receipt,
        args.dataset_root,
        formal=args.mode == "formal",
    )
    rows = loaded["rows"] if args.limit is None else loaded["rows"][: args.limit]
    assets = build_assets(rows, args.dataset_root)
    checkpoint_path = output / "checkpoint.json"
    journal = output / RECORD_JOURNAL_DIR
    if resume:
        manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
        _validate_resume_manifest(args, output, manifest, loaded, assets)
        if journal.is_symlink() or not journal.is_dir():
            raise RuntimeError("resume record journal is missing")
    else:
        protocol_binding = freeze_protocol_snapshot(output)
        source_snapshots = freeze_source_snapshots(output)
        manifest = build_manifest(
            args, loaded, assets, protocol_binding, source_snapshots
        )
        CORE.atomic_write_json(output / "manifest.json", manifest)
        CORE.atomic_write_json(
            output / "environment.json", manifest["evaluation"]["environment"]
        )
        journal.mkdir(parents=False, exist_ok=False)
        CORE.atomic_write_json(
            checkpoint_path,
            {
                "state": "frozen",
                "completed": 0,
                "remaining": len(assets),
                "n_eval": len(assets),
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "updated_at": CORE.utc_now(),
            },
        )
    frozen_harness = _load_module(
        Path(
            manifest["evaluation"]["source_snapshots"]["child_harness"]["path"]
        ),
        f"table3_sketch_frozen_harness_{uuid.uuid4().hex}",
    )
    scratch = output / ".worker_scratch"
    if scratch.exists():
        shutil.rmtree(scratch)
    scratch.mkdir()
    jobs = [
        {
            **asset,
            "samples": args.samples,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "effective_child_environment_sha256": manifest["evaluation"][
                "effective_child_environment_sha256"
            ],
        }
        for asset in assets
    ]
    existing = load_journal_records(journal, jobs)
    try:
        by_key = _execute_jobs(
            jobs,
            scratch,
            journal,
            harness=frozen_harness,
            existing=existing,
            workers=args.workers,
            timeout_seconds=args.asset_timeout_seconds,
            checkpoint_path=checkpoint_path,
        )
    finally:
        shutil.rmtree(scratch, ignore_errors=True)
    records = [by_key[job["asset_key"]] for job in jobs]
    records_path = output / "asset_records.jsonl"
    temporary_records = output / f".{records_path.name}.{uuid.uuid4().hex}.tmp"
    with temporary_records.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")
    os.replace(temporary_records, records_path)

    summary = CORE.aggregate_records(records, len(assets))
    summary.update(
        {
            "schema_version": "1.0.0-sketch-mobility",
            "status": "completed",
            "classification": manifest["classification"],
            "dataset": DATASET_NAME,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "completed_at": CORE.utc_now(),
        }
    )
    if args.mode == "formal" and summary["j_eval"] != FORMAL_J_EVAL:
        raise RuntimeError(f"formal movable-joint count mismatch: {summary['j_eval']}")
    CORE.atomic_write_json(output / "summary.json", summary)
    CORE.atomic_write_text(output / "summary.md", _summary_markdown(summary, manifest))
    CORE.atomic_write_json(
        checkpoint_path,
        {
            "state": "complete",
            "completed": len(assets),
            "remaining": 0,
            "n_eval": len(assets),
            "status_counts": summary["status_counts"],
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "updated_at": CORE.utc_now(),
        },
    )
    _run_independent_verifier(output, manifest)
    shutil.rmtree(journal)
    shutil.rmtree(output / SOURCE_SNAPSHOT_DIR / "__pycache__", ignore_errors=True)
    _write_artifact_manifest(output)
    validate_protocol_snapshot_binding(output, manifest["evaluation"])
    verify_artifacts(output)
    return summary


def run(args: argparse.Namespace) -> dict[str, Any]:
    validate_contract(args)
    output = args.output.absolute()
    dataset_root = args.dataset_root.resolve(strict=True)
    try:
        output.resolve(strict=False).relative_to(dataset_root)
    except ValueError:
        pass
    else:
        raise ValueError("output must not be inside the SketchMobility dataset root")
    with _output_lock(output):
        if args.resume:
            version, resume_index = _load_resume_version(output)
        else:
            version, resume_index = _allocate_fresh_version(output)
        summary = _run_to_output(args, version, resume=args.resume)
        _publish_final_version(version, output, resume_index)
        return summary


def validate_contract(args: argparse.Namespace) -> None:
    if args.samples < 2 or args.workers <= 0 or args.asset_timeout_seconds <= 0:
        raise ValueError("samples must be >=2 and workers/timeout must be positive")
    if args.limit is not None and not 0 < args.limit <= FORMAL_N_EVAL:
        raise ValueError(f"limit must be in [1, {FORMAL_N_EVAL}]")
    if args.mode == "formal":
        required = {
            "limit": None,
            "samples": DEFAULT_SAMPLES,
            "workers": DEFAULT_WORKERS,
            "asset_timeout_seconds": DEFAULT_TIMEOUT_SECONDS,
            "dataset_root": DEFAULT_DATASET_ROOT.resolve(strict=False),
            "table1_receipt": DEFAULT_TABLE1_RECEIPT.resolve(strict=False),
            "table2_manifest": DEFAULT_TABLE2_MANIFEST.resolve(strict=False),
        }
        observed = {
            "limit": args.limit,
            "samples": args.samples,
            "workers": args.workers,
            "asset_timeout_seconds": args.asset_timeout_seconds,
            "dataset_root": args.dataset_root.resolve(strict=False),
            "table1_receipt": args.table1_receipt.resolve(strict=False),
            "table2_manifest": args.table2_manifest.resolve(strict=False),
        }
        if observed != required:
            raise ValueError(f"formal mode requires exactly {required}; observed {observed}")
        validate_formal_runtime(platform.python_version(), numpy.__version__)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), default="formal")
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--table1-receipt", type=Path, default=DEFAULT_TABLE1_RECEIPT)
    parser.add_argument("--table2-manifest", type=Path, default=DEFAULT_TABLE2_MANIFEST)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--asset-timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resume", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.output is None:
        if args.resume:
            raise ValueError("--resume requires an explicit --output path")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        suffix = f"smoke_n{args.limit}" if args.mode == "smoke" else "n800"
        args.output = DEFAULT_OUTPUT_PARENT / f"urdf_table3_sketch_mobility_{suffix}_{stamp}"
    summary = run(args)
    print(
        json.dumps(
            {
                "status": "completed",
                "n_eval": summary["n_eval"],
                "j_eval": summary["j_eval"],
                "output": str(args.output.absolute()),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
