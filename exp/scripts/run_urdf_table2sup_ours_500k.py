#!/usr/bin/env python3
"""Formal Table 2 supplementary runner for Ours-500K (static, fail-closed).

Cohort
------
The sample is the frozen Table 1 Ours-500K manifest
(``exp/runtime/table1_ours_500k/manifest.json``, byte SHA-256
``bc3eb334...``): all 500 ``.assets[]`` entries in their original order
(``FULL_ACQUIRED_RELEASE_SAMPLE_NO_SUBSAMPLING``).  This is exactly the
cohort of the formal Table 2 / Table 3 Ours-500K runs.  There is no
resampling, no replacement of failed assets and no outcome-based selection.

Package binding
---------------
Ours-500K packages live persistently under
``exp/Brain/extracted/arti_cabinet_drawer_geometry_500_20260813``.  Each
asset is bound to the absolute ``package`` recorded in the frozen Table 2
manifest, and the binding is verified before evaluation by recomputing the
package content manifest in the frozen walk-order canonicalization and
comparing its SHA-256 with the frozen Table 2
``package_content_manifest_sha256`` (plus the primary URDF SHA-256).  Any
mismatch is fail-closed.

Denominators
------------
- Asset-level denominator: ``N_eval = 500`` (parse/binding failures retained).
- Joint-level denominator: ``J_eval = 2467`` taken from the frozen Table 3
  Ours-500K asset records (``declared_joint_count`` per asset).
- Placeholder-mass registry: frozen empty, consistent with all other methods.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import importlib.util
import json
import multiprocessing
import os
from pathlib import Path
import platform
import shutil
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from typing import Any, Mapping

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import table2_supplementary_static as static_evaluator  # noqa: E402

PROTOCOL_ID = "table2_supplementary_ours_500k_table1cohort_n500_v1"
DATASET = "Ours-500K"
SCHEMA_VERSION = "table2-supplementary-ours-500k/v1"

OURS_ROOT = REPO / "exp/Brain/extracted/arti_cabinet_drawer_geometry_500_20260813"
TABLE1_MANIFEST = REPO / "exp/runtime/table1_ours_500k/manifest.json"
TABLE2_MANIFEST = REPO / "exp/runtime/table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z/manifest.json"
TABLE3_RECORDS = REPO / "exp/runtime/urdf_table3_ours_500k_table2_n500_20260819T101000Z/asset_records.jsonl"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"

EXPECTED_TABLE1_MANIFEST_SHA256 = "bc3eb334b1fc1c57378e50e7c2fab5d765a7599db8b3e82bc1d91536570b7c06"
EXPECTED_TABLE2_MANIFEST_SELF_SHA256 = "8b1bdb53bef17ac104bb42daa331899dfa498020be131fe5727f16df0f0427fa"
EXPECTED_COHORT_SIZE = 500
EXPECTED_J_EVAL = 2467

PLACEHOLDER_REGISTRY: list[dict[str, Any]] = []
PLACEHOLDER_REGISTRY_RATIONALE = (
    "frozen empty: consistent with the Artiverse / LAM / PartNet-Mobility / PhysX-Mobility "
    "precedent; no Ours-500K exporter default template was validated from frozen tool defaults "
    "or public documentation before result inspection; incidence therefore reported N/E "
    "with complete-inertial coverage"
)

DEFAULT_WORKERS = 16
ASSET_TIMEOUT_SECONDS = 900

INPUT_IDENTITY_FIELDS = (
    "selection_index", "asset_id", "asset_root", "raw_category", "seed_name",
    "selection_rank", "package", "primary_urdf_relative_path",
    "expected_declared_joint_count", "model_urdf_sha256_expected",
    "package_content_manifest_sha256_expected",
)


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def atomic_json(path: Path, value: Any) -> None:
    atomic_write_text(path, canonical_json(value) + "\n")


def utc_now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def timestamp_tag() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def module_sha256(module_name: str) -> str:
    target = SCRIPT.with_name(f"{module_name}.py")
    return sha256_file(target)


class ProtocolViolation(RuntimeError):
    """Raised when a frozen input or binding check fails."""


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


# --------------------------------------------------------------------------
# Cohort freezing
# --------------------------------------------------------------------------

def verify_table1_manifest() -> dict[str, Any]:
    observed = sha256_file(TABLE1_MANIFEST)
    if observed != EXPECTED_TABLE1_MANIFEST_SHA256:
        raise ProtocolViolation(
            f"Table 1 Ours-500K manifest SHA-256 mismatch: expected "
            f"{EXPECTED_TABLE1_MANIFEST_SHA256}, observed {observed}"
        )
    payload = load_json(TABLE1_MANIFEST)
    assets = payload.get("assets")
    if not isinstance(assets, list) or len(assets) != EXPECTED_COHORT_SIZE:
        raise ProtocolViolation(f"Table 1 manifest must contain {EXPECTED_COHORT_SIZE} assets")
    if payload.get("dataset") != "Ours-500K":
        raise ProtocolViolation(f"Table 1 manifest dataset is {payload.get('dataset')!r}")
    return payload


def verify_table2_manifest() -> dict[str, Any]:
    payload = load_json(TABLE2_MANIFEST)
    declared = payload.get("manifest_content_sha256")
    body = {key: value for key, value in payload.items() if key != "manifest_content_sha256"}
    computed = canonical_sha256(body)
    if declared != EXPECTED_TABLE2_MANIFEST_SELF_SHA256 or computed != EXPECTED_TABLE2_MANIFEST_SELF_SHA256:
        raise ProtocolViolation(
            "Table 2 PhysX-Mobility manifest self-hash mismatch: declared "
            f"{declared}, computed {computed}, expected {EXPECTED_TABLE2_MANIFEST_SELF_SHA256}"
        )
    assets = payload.get("assets")
    if not isinstance(assets, list) or len(assets) != EXPECTED_COHORT_SIZE:
        raise ProtocolViolation(f"Table 2 manifest must contain {EXPECTED_COHORT_SIZE} assets")
    return payload


def load_table2_records() -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    with TABLE2_MANIFEST.with_name("asset_records.jsonl").open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            asset_id = row.get("asset_id")
            if not isinstance(asset_id, str) or asset_id in records:
                raise ProtocolViolation(f"invalid or duplicate Table 2 record for {asset_id!r}")
            records[asset_id] = row
    if len(records) != EXPECTED_COHORT_SIZE:
        raise ProtocolViolation("Table 2 asset records must cover the full cohort")
    return records


def load_table3_expected_joints() -> dict[str, int]:
    expected: dict[str, int] = {}
    with TABLE3_RECORDS.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            asset_id = row.get("asset_id")
            count = row.get("declared_joint_count")
            if not isinstance(asset_id, str) or isinstance(count, bool) or not isinstance(count, int) or count < 0:
                raise ProtocolViolation(f"invalid Table 3 declared_joint_count row for {asset_id!r}")
            if asset_id in expected:
                raise ProtocolViolation(f"duplicate Table 3 asset_id: {asset_id}")
            expected[asset_id] = count
    if len(expected) != EXPECTED_COHORT_SIZE:
        raise ProtocolViolation(f"Table 3 records must cover {EXPECTED_COHORT_SIZE} assets, found {len(expected)}")
    total = sum(expected.values())
    if total != EXPECTED_J_EVAL:
        raise ProtocolViolation(f"Table 3 declared_joint_count total {total} != frozen J_eval {EXPECTED_J_EVAL}")
    return expected


def freeze_cohort(*, limit: int | None) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    table1 = verify_table1_manifest()
    table2 = verify_table2_manifest()
    table2_records = load_table2_records()
    expected_joints = load_table3_expected_joints()
    table2_by_id = {asset["asset_id"]: asset for asset in table2["assets"]}
    if len(table2_by_id) != EXPECTED_COHORT_SIZE:
        raise ProtocolViolation("Table 2 manifest asset_id values are not unique")

    items: list[dict[str, Any]] = []
    for index, asset in enumerate(table1["assets"]):
        asset_id = str(asset["asset_id"])
        record = table2_by_id.get(asset_id)
        if record is None:
            raise ProtocolViolation(f"asset missing from Table 2 manifest: {asset_id}")
        table2_record = table2_records.get(asset_id)
        if table2_record is None:
            raise ProtocolViolation(f"asset missing from Table 2 asset records: {asset_id}")
        if asset_id not in expected_joints:
            raise ProtocolViolation(f"asset missing from Table 3 records: {asset_id}")
        package = record["package"]
        expected_package = (OURS_ROOT / record["asset_root"]).as_posix()
        if package != expected_package:
            raise ProtocolViolation(f"package binding mismatch for {asset_id}: {package} != {expected_package}")
        primary_relative = record["primary_urdf"].split(asset_id + "/", 1)[-1]
        item = {
            "selection_index": index,
            "asset_id": asset_id,
            "asset_root": record["asset_root"],
            "raw_category": record["raw_category"],
            "seed_name": record["seed_name"],
            "selection_rank": asset["selection_rank"],
            "package": package,
            "primary_urdf_relative_path": primary_relative,
            "expected_declared_joint_count": expected_joints[asset_id],
            "model_urdf_sha256_expected": record["primary_urdf_sha256"],
            "package_content_manifest_sha256_expected": table2_record["package_content_manifest_sha256"],
        }
        item["input_identity_sha256"] = canonical_sha256(
            {field: item[field] for field in INPUT_IDENTITY_FIELDS}
        )
        items.append(item)

    if limit is not None:
        items = items[:limit]

    provenance = {
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "schema_version": SCHEMA_VERSION,
        "cohort_type": "FULL_ACQUIRED_RELEASE_SAMPLE_NO_SUBSAMPLING",
        "selection_policy": "existing Table 1 manifest order; no resampling/reselection",
        "sample_expression": "jq -r '.assets[].asset_id' exp/runtime/table1_ours_500k/manifest.json",
        "n_eval": len(items),
        "j_eval": sum(item["expected_declared_joint_count"] for item in items),
        "table1_manifest_path": str(TABLE1_MANIFEST),
        "table1_manifest_sha256": EXPECTED_TABLE1_MANIFEST_SHA256,
        "table2_manifest_path": str(TABLE2_MANIFEST),
        "table2_manifest_self_sha256": EXPECTED_TABLE2_MANIFEST_SELF_SHA256,
        "table3_records_path": str(TABLE3_RECORDS),
        "ours_root": str(OURS_ROOT),
        "protocol_document_path": str(PROTOCOL_DOCUMENT),
        "protocol_document_sha256": sha256_file(PROTOCOL_DOCUMENT),
        "placeholder_mass_registry": PLACEHOLDER_REGISTRY,
        "placeholder_mass_registry_rationale": PLACEHOLDER_REGISTRY_RATIONALE,
        "asset_timeout_seconds": ASSET_TIMEOUT_SECONDS,
        "evaluator_modules": {
            "table2_supplementary_static_sha256": module_sha256("table2_supplementary_static"),
            "lam_supplementary_static_sha256": module_sha256("lam_supplementary_static"),
        },
        "runner_sha256": sha256_file(SCRIPT),
        "created_at_utc": utc_now_iso(),
    }
    return items, provenance


def write_frozen_manifest(output_root: Path, items: list[dict[str, Any]], provenance: dict[str, Any]) -> dict[str, Any]:
    manifest = {
        "protocol_id": PROTOCOL_ID,
        "schema_version": SCHEMA_VERSION,
        "dataset": DATASET,
        "provenance": provenance,
        "items": items,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(
        {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    )
    atomic_json(output_root / "frozen_manifest.json", manifest)
    return manifest


# --------------------------------------------------------------------------
# Staging and binding verification
# --------------------------------------------------------------------------

def walk_order_content_manifest(package: Path) -> str:
    """Reproduce the frozen Table 2 walk-order content-manifest SHA-256."""

    package = package.resolve(strict=True)
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        directory_names.sort()
        file_names.sort()
        current = Path(current_raw)
        for name in file_names:
            path = current / name
            if path.is_symlink() or not path.is_file():
                continue
            relative = path.relative_to(package).as_posix()
            rows.append({"path": relative, "bytes": path.stat().st_size, "sha256": sha256_file(path)})
    return canonical_sha256(rows)


def verify_binding(item: Mapping[str, Any]) -> dict[str, Any]:
    """Verify package bytes against the frozen Table 2 binding (walk-order hash)."""

    package = Path(item["package"])
    issues: list[str] = []
    if package.is_symlink():
        issues.append("package_is_symlink")
    elif not package.is_dir():
        issues.append("package_missing")
    else:
        try:
            resolved = package.resolve(strict=True)
            resolved.relative_to(OURS_ROOT.resolve(strict=True))
        except (OSError, ValueError):
            issues.append("package_escapes_ours_root")
        if not issues:
            observed_sha = walk_order_content_manifest(resolved)
            if observed_sha != item["package_content_manifest_sha256_expected"]:
                issues.append(
                    "package_content_manifest_sha256_mismatch: expected "
                    f"{item['package_content_manifest_sha256_expected']}, observed {observed_sha}"
                )
            urdf_path = resolved / item["primary_urdf_relative_path"]
            if not urdf_path.is_file() or urdf_path.is_symlink():
                issues.append("primary_urdf_missing")
            else:
                observed_urdf_sha = sha256_file(urdf_path)
                if observed_urdf_sha != item["model_urdf_sha256_expected"]:
                    issues.append(
                        "model_urdf_sha256_mismatch: expected "
                        f"{item['model_urdf_sha256_expected']}, observed {observed_urdf_sha}"
                    )
    return {
        "verified": not issues,
        "issues": issues,
        "verified_at_utc": utc_now_iso(),
    }


def timeout_record(item: Mapping[str, Any], reason: str) -> dict[str, Any]:
    return {
        "schema_version": "table2-supplementary-static/v1",
        "asset_id": item["asset_id"],
        "package": item["package"],
        "urdf_relative_path": item["primary_urdf_relative_path"],
        "urdf_sha256": None,
        "status": "error",
        "parse": {"success": False, "issues": [reason]},
        "table2_supplementary": {
            "visual_bearing_collision_coverage": {
                "status": "NOT_EVALUABLE", "asset_intended": 1, "asset_passed": 0,
                "asset_pass": False, "visual_bearing_links_declared": 0,
                "covered_visual_bearing_links": 0, "link_extraction_complete": False,
                "collision_elements_declared_on_visual_links": 0,
                "loadable_collision_elements_on_visual_links": 0,
                "link_records": [], "issues": [reason],
            },
            "joint_limit_portability": {
                "status": "NOT_EVALUABLE",
                "joints_intended": item["expected_declared_joint_count"],
                "joints_extracted": 0, "joints_passed": 0, "extraction_complete": False,
                "joint_records": [], "issues": [reason],
            },
            "joint_dynamics_coverage": {
                "status": "NOT_EVALUABLE",
                "joints_intended": item["expected_declared_joint_count"],
                "joints_extracted": 0, "joints_covered": 0, "extraction_complete": False,
                "joint_records": [], "issues": [reason],
            },
            "placeholder_mass_incidence": {
                "status": "N/E", "dynamic_link_policy": "all_declared_links",
                "dynamic_links": 0, "complete_inertial_links": 0,
                "complete_inertial_coverage_numerator": 0,
                "complete_inertial_coverage_denominator": 0,
                "classified_complete_inertial_links": 0,
                "unclassified_complete_inertial_links": 0,
                "placeholder_links": None, "incidence_numerator": None,
                "incidence_denominator": 0, "registry_ids": [],
                "link_records": [], "incomplete_inertial_links": [], "issues": [reason],
            },
        },
        "resource_closure": {
            "status": "NOT_EVALUABLE", "complete": False, "file_count": 0,
            "sha256": None, "files": [], "issues": [reason],
        },
        "issues": [reason],
    }


def _dispatch_payload(item: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "package": item["package"],
        "primary_urdf_relative_path": item["primary_urdf_relative_path"],
        "asset_id": item["asset_id"],
        "expected_declared_joint_count": item["expected_declared_joint_count"],
        "placeholder_registry": PLACEHOLDER_REGISTRY,
    }


def child_receipt_path(output_root: Path, selection_index: int) -> Path:
    return output_root / "children" / f"rank_{selection_index:04d}.json"


def run_evaluation(
    output_root: Path,
    items: list[dict[str, Any]],
    binding: dict[str, dict[str, Any]],
    *,
    workers: int,
) -> dict[int, dict[str, Any]]:
    receipts: dict[int, dict[str, Any]] = {}
    for item in items:
        path = child_receipt_path(output_root, item["selection_index"])
        if not path.exists():
            continue
        payload = load_json(path)
        if payload.get("input_identity_sha256") != item["input_identity_sha256"]:
            raise ProtocolViolation(
                f"stale child receipt for selection_index={item['selection_index']} "
                "(frozen input identity changed); refusing to resume"
            )
        receipts[item["selection_index"]] = payload

    queue = [item for item in items if item["selection_index"] not in receipts]
    evaluable: list[dict[str, Any]] = []
    for item in queue:
        bind = binding[item["asset_id"]]
        if bind["verified"]:
            evaluable.append(item)
            continue
        reason = "binding_failed: " + "; ".join(bind["issues"])
        receipt = {
            "input_identity_sha256": item["input_identity_sha256"],
            "selection_index": item["selection_index"],
            "asset_id": item["asset_id"],
            "status": "binding_failed",
            "binding": bind,
            "audit": timeout_record(item, reason),
            "completed_at_utc": utc_now_iso(),
        }
        atomic_json(child_receipt_path(output_root, item["selection_index"]), receipt)
        receipts[item["selection_index"]] = receipt

    context = multiprocessing.get_context("fork")
    while evaluable:
        pool = context.Pool(processes=workers)
        in_flight: list[tuple[dict[str, Any], Any]] = []
        try:
            for item in evaluable:
                in_flight.append((item, pool.apply_async(static_evaluator.audit_worker, (_dispatch_payload(item),))))
            survivor: list[dict[str, Any]] = []
            for item, async_result in in_flight:
                index = item["selection_index"]
                if index in receipts:
                    continue
                try:
                    audit = async_result.get(timeout=ASSET_TIMEOUT_SECONDS)
                    status = "completed" if audit.get("status") == "completed" else "error"
                except multiprocessing.TimeoutError:
                    audit = timeout_record(item, f"asset_timeout_after_{ASSET_TIMEOUT_SECONDS}s")
                    status = "timeout"
                    pool.terminate()
                    pool.join()
                    receipt = {
                        "input_identity_sha256": item["input_identity_sha256"],
                        "selection_index": index,
                        "asset_id": item["asset_id"],
                        "status": status,
                        "binding": binding[item["asset_id"]],
                        "audit": audit,
                        "completed_at_utc": utc_now_iso(),
                    }
                    atomic_json(child_receipt_path(output_root, index), receipt)
                    receipts[index] = receipt
                    evaluable = [
                        pending_item for pending_item, _ in in_flight
                        if pending_item["selection_index"] not in receipts
                    ]
                    break
                except Exception as exc:  # noqa: BLE001
                    audit = timeout_record(item, f"worker_exception: {type(exc).__name__}: {exc}")
                    status = "error"
                receipt = {
                    "input_identity_sha256": item["input_identity_sha256"],
                    "selection_index": index,
                    "asset_id": item["asset_id"],
                    "status": status,
                    "binding": binding[item["asset_id"]],
                    "audit": audit,
                    "completed_at_utc": utc_now_iso(),
                }
                atomic_json(child_receipt_path(output_root, index), receipt)
                receipts[index] = receipt
            else:
                evaluable = []
        finally:
            pool.terminate()
            pool.join()
    return receipts


# --------------------------------------------------------------------------
# Aggregation
# --------------------------------------------------------------------------

def ratio(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": numerator,
        "denominator": denominator,
        "percentage": (100.0 * numerator / denominator) if denominator else None,
    }


def aggregate(items: list[dict[str, Any]], receipts: dict[int, dict[str, Any]], j_eval: int) -> dict[str, Any]:
    n_eval = len(items)
    asset_pass = 0
    link_declared = 0
    link_covered = 0
    link_extraction_complete_assets = 0
    portability_passed = 0
    portability_intended = 0
    portability_extracted = 0
    dynamics_covered = 0
    dynamics_intended = 0
    placeholder_status_counts: dict[str, int] = {}
    complete_inertial_links = 0
    dynamic_links = 0
    status_counts: dict[str, int] = {}
    parse_success = 0

    category: dict[str, dict[str, int]] = {}
    joint_type: dict[str, dict[str, int]] = {}

    for item in items:
        receipt = receipts[item["selection_index"]]
        audit = receipt["audit"]
        status = receipt["status"]
        status_counts[status] = status_counts.get(status, 0) + 1
        table2 = audit.get("table2_supplementary", {})
        visual = table2.get("visual_bearing_collision_coverage", {})
        portability = table2.get("joint_limit_portability", {})
        dynamics = table2.get("joint_dynamics_coverage", {})
        placeholder = table2.get("placeholder_mass_incidence", {})

        passed = int(visual.get("asset_passed", 0))
        asset_pass += passed
        link_declared += int(visual.get("visual_bearing_links_declared", 0))
        link_covered += int(visual.get("covered_visual_bearing_links", 0))
        if visual.get("link_extraction_complete"):
            link_extraction_complete_assets += 1
        portability_passed += int(portability.get("joints_passed", 0))
        portability_intended += int(portability.get("joints_intended", 0))
        portability_extracted += int(portability.get("joints_extracted", 0))
        dynamics_covered += int(dynamics.get("joints_covered", 0))
        dynamics_intended += int(dynamics.get("joints_intended", 0))
        placeholder_status_counts[str(placeholder.get("status"))] = (
            placeholder_status_counts.get(str(placeholder.get("status")), 0) + 1
        )
        complete_inertial_links += int(placeholder.get("complete_inertial_links", 0))
        dynamic_links += int(placeholder.get("dynamic_links", 0))
        if audit.get("parse", {}).get("success"):
            parse_success += 1

        bucket = category.setdefault(item["raw_category"], {
            "assets": 0, "asset_pass": 0,
            "portability_passed": 0, "portability_intended": 0,
            "dynamics_covered": 0, "dynamics_intended": 0,
        })
        bucket["assets"] += 1
        bucket["asset_pass"] += passed
        bucket["portability_passed"] += int(portability.get("joints_passed", 0))
        bucket["portability_intended"] += int(portability.get("joints_intended", 0))
        bucket["dynamics_covered"] += int(dynamics.get("joints_covered", 0))
        bucket["dynamics_intended"] += int(dynamics.get("joints_intended", 0))

        for joint_record in portability.get("joint_records", []):
            jt = joint_type.setdefault(str(joint_record.get("joint_type")), {
                "joints": 0, "portability_passed": 0, "dynamics_covered": 0,
            })
            jt["joints"] += 1
            jt["portability_passed"] += int(bool(joint_record.get("limit_portability_pass")))
        for joint_record in dynamics.get("joint_records", []):
            jt = joint_type.setdefault(str(joint_record.get("joint_type")), {
                "joints": 0, "portability_passed": 0, "dynamics_covered": 0,
            })
            jt["dynamics_covered"] += int(bool(joint_record.get("covered")))

    macro_visual: list[float] = []
    macro_portability: list[float] = []
    macro_dynamics: list[float] = []
    for bucket in category.values():
        if bucket["assets"]:
            macro_visual.append(100.0 * bucket["asset_pass"] / bucket["assets"])
        if bucket["portability_intended"]:
            macro_portability.append(100.0 * bucket["portability_passed"] / bucket["portability_intended"])
        if bucket["dynamics_intended"]:
            macro_dynamics.append(100.0 * bucket["dynamics_covered"] / bucket["dynamics_intended"])

    summary = {
        "protocol_id": PROTOCOL_ID,
        "schema_version": SCHEMA_VERSION,
        "dataset": DATASET,
        "n_eval": n_eval,
        "j_eval": j_eval,
        "status_counts": dict(sorted(status_counts.items())),
        "parse_success_assets": parse_success,
        "metrics": {
            "visual_bearing_collision_coverage": {
                "asset_level": ratio(asset_pass, n_eval),
                "link_micro": ratio(link_covered, link_declared),
                "link_extraction_coverage": ratio(link_extraction_complete_assets, n_eval),
            },
            "joint_limit_portability": {
                "joint_level": ratio(portability_passed, j_eval),
                "intended_from_frozen_scan": ratio(portability_intended, j_eval),
                "extracted": portability_extracted,
            },
            "joint_dynamics_coverage": {
                "joint_level": ratio(dynamics_covered, j_eval),
                "intended_from_frozen_scan": ratio(dynamics_intended, j_eval),
            },
            "placeholder_mass_incidence": {
                "status": "N/E" if not PLACEHOLDER_REGISTRY else "COMPLETE",
                "reason": PLACEHOLDER_REGISTRY_RATIONALE if not PLACEHOLDER_REGISTRY else None,
                "registry_ids": [],
                "placeholder_status_counts": dict(sorted(placeholder_status_counts.items())),
                "complete_inertial_coverage": ratio(complete_inertial_links, dynamic_links),
            },
        },
        "category_macro": {
            "category_count": len(category),
            "visual_bearing_collision_coverage_pct": (sum(macro_visual) / len(macro_visual)) if macro_visual else None,
            "joint_limit_portability_pct": (sum(macro_portability) / len(macro_portability)) if macro_portability else None,
            "joint_dynamics_coverage_pct": (sum(macro_dynamics) / len(macro_dynamics)) if macro_dynamics else None,
        },
        "category_breakdown": {
            key: {
                "assets": value["assets"],
                "visual_bearing_collision_coverage": ratio(value["asset_pass"], value["assets"]),
                "joint_limit_portability": ratio(value["portability_passed"], value["portability_intended"]),
                "joint_dynamics_coverage": ratio(value["dynamics_covered"], value["dynamics_intended"]),
            }
            for key, value in sorted(category.items())
        },
        "joint_type_breakdown": {
            key: {
                "joints": value["joints"],
                "portability_passed": value["portability_passed"],
                "dynamics_covered": value["dynamics_covered"],
            }
            for key, value in sorted(joint_type.items())
        },
        "completed_at_utc": utc_now_iso(),
    }
    return summary


def write_asset_records(output_root: Path, items: list[dict[str, Any]], receipts: dict[int, dict[str, Any]]) -> None:
    lines: list[str] = []
    for item in items:
        receipt = receipts[item["selection_index"]]
        audit = receipt["audit"]
        table2 = audit.get("table2_supplementary", {})
        visual = table2.get("visual_bearing_collision_coverage", {})
        portability = table2.get("joint_limit_portability", {})
        dynamics = table2.get("joint_dynamics_coverage", {})
        placeholder = table2.get("placeholder_mass_incidence", {})
        record = {
            "selection_index": item["selection_index"],
            "asset_id": item["asset_id"],
            "asset_root": item["asset_root"],
            "raw_category": item["raw_category"],
            "seed_name": item["seed_name"],
            "selection_rank": item["selection_rank"],
            "package": item["package"],
            "primary_urdf_relative_path": item["primary_urdf_relative_path"],
            "expected_declared_joint_count": item["expected_declared_joint_count"],
            "input_identity_sha256": item["input_identity_sha256"],
            "binding_verified": receipt["binding"]["verified"],
            "binding_issues": receipt["binding"]["issues"],
            "status": receipt["status"],
            "parse_success": bool(audit.get("parse", {}).get("success")),
            "urdf_sha256": audit.get("urdf_sha256"),
            "visual_bearing_collision_coverage_asset_pass": bool(visual.get("asset_pass")),
            "visual_bearing_links_declared": int(visual.get("visual_bearing_links_declared", 0)),
            "covered_visual_bearing_links": int(visual.get("covered_visual_bearing_links", 0)),
            "joint_limit_portability_passed": int(portability.get("joints_passed", 0)),
            "joint_limit_portability_intended": int(portability.get("joints_intended", 0)),
            "joint_dynamics_covered": int(dynamics.get("joints_covered", 0)),
            "joint_dynamics_intended": int(dynamics.get("joints_intended", 0)),
            "placeholder_mass_status": placeholder.get("status"),
            "complete_inertial_links": int(placeholder.get("complete_inertial_links", 0)),
            "dynamic_links": int(placeholder.get("dynamic_links", 0)),
            "issues": audit.get("issues", []),
            "audit": audit,
            "completed_at_utc": receipt["completed_at_utc"],
        }
        lines.append(canonical_json(record))
    atomic_write_text(output_root / "asset_records.jsonl", "\n".join(lines) + "\n")


def render_summary_md(summary: Mapping[str, Any]) -> str:
    metrics = summary["metrics"]
    visual = metrics["visual_bearing_collision_coverage"]
    portability = metrics["joint_limit_portability"]
    dynamics = metrics["joint_dynamics_coverage"]
    placeholder = metrics["placeholder_mass_incidence"]

    def pct(cell: Mapping[str, Any]) -> str:
        if cell.get("denominator") and cell.get("percentage") is not None:
            return f"{cell['numerator']} / {cell['denominator']} ({cell['percentage']:.2f}%)"
        return f"{cell.get('numerator')} / {cell.get('denominator')} (N/E)"

    lines = [
        f"# Table 2 supplementary — {summary['dataset']} (protocol {summary['protocol_id']})",
        "",
        f"- N_eval = {summary['n_eval']}, J_eval = {summary['j_eval']}",
        f"- status counts: {summary['status_counts']}",
        f"- parse success assets: {summary['parse_success_assets']}",
        "",
        "| Metric | Result |",
        "|---|---|",
        f"| Visual-bearing Collision Coverage (asset) | {pct(visual['asset_level'])} |",
        f"| Visual-bearing Collision Coverage (link-micro) | {pct(visual['link_micro'])} |",
        f"| Link extraction coverage | {pct(visual['link_extraction_coverage'])} |",
        f"| Joint-limit Portability | {pct(portability['joint_level'])} |",
        f"| Joint Dynamics Coverage | {pct(dynamics['joint_level'])} |",
        f"| Placeholder-mass Incidence | {placeholder['status']} ({placeholder['reason']}) |",
        f"| Complete-inertial coverage | {pct(placeholder['complete_inertial_coverage'])} |",
        "",
    ]
    return "\n".join(lines)


def write_environment(output_root: Path, workers: int) -> None:
    environment = {
        "python": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "executable": sys.executable,
        "platform": platform.platform(),
        "worker_count": workers,
        "gpu_required": False,
        "evaluator_modules": {
            "table2_supplementary_static_sha256": module_sha256("table2_supplementary_static"),
            "lam_supplementary_static_sha256": module_sha256("lam_supplementary_static"),
        },
        "runner_sha256": sha256_file(SCRIPT),
        "recorded_at_utc": utc_now_iso(),
    }
    atomic_json(output_root / "environment.json", environment)


def write_protocol_snapshot(output_root: Path) -> str:
    snapshot = PROTOCOL_DOCUMENT.read_text(encoding="utf-8")
    atomic_write_text(output_root / "protocol_snapshot.md", snapshot)
    return sha256_file(PROTOCOL_DOCUMENT)


def default_output_root(*, limit: int | None) -> Path:
    if limit is None:
        return REPO / f"exp/runtime/table2sup_urdf_ours_500k_table1cohort_n500_{timestamp_tag()}"
    return REPO / f"exp/runtime/table2sup_urdf_ours_500k_smoke_n{limit}_{timestamp_tag()}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--skip-verify", action="store_true")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args(argv)

    if args.limit is not None and (args.limit <= 0 or args.limit > EXPECTED_COHORT_SIZE):
        raise SystemExit(f"--limit must be within 1..{EXPECTED_COHORT_SIZE}")

    output_root = args.output or default_output_root(limit=args.limit)
    if args.resume:
        if not (output_root / "frozen_manifest.json").exists():
            raise SystemExit(f"--resume requires an existing frozen_manifest.json under {output_root}")
    else:
        if output_root.exists() and any(output_root.iterdir()):
            raise SystemExit(f"output root already exists and is not empty: {output_root}")
        output_root.mkdir(parents=True, exist_ok=True)

    started = time.monotonic()
    items, provenance = freeze_cohort(limit=args.limit)
    provenance["output_root"] = str(output_root)
    j_eval = provenance["j_eval"]

    print("[binding] verifying package bindings against the frozen Table 2 records ...")
    binding: dict[str, dict[str, Any]] = {}
    binding_failures = 0
    for item in items:
        result = verify_binding(item)
        binding[item["asset_id"]] = result
        if not result["verified"]:
            binding_failures += 1
            print(f"[binding] FAIL {item['asset_id']}: {'; '.join(result['issues'][:3])}")
    print(f"[binding] verified {len(items) - binding_failures} / {len(items)}")

    if args.resume:
        existing = load_json(output_root / "frozen_manifest.json")
        declared = existing.get("manifest_content_sha256")
        body = {key: value for key, value in existing.items() if key != "manifest_content_sha256"}
        if canonical_sha256(body) != declared:
            raise SystemExit("existing frozen manifest self-hash is invalid; refusing to resume")
        existing_ids = [item.get("input_identity_sha256") for item in existing.get("items", [])]
        current_ids = [item["input_identity_sha256"] for item in items]
        if existing_ids != current_ids:
            raise SystemExit("frozen cohort identity changed since the original freeze; refusing to resume")
        manifest = existing
        snapshot_path = output_root / "protocol_snapshot.md"
        if not snapshot_path.exists():
            raise SystemExit("protocol_snapshot.md missing; refusing to resume")
        snapshot_sha = sha256_file(snapshot_path)
        print(f"[resume] reusing frozen manifest sha256={manifest['manifest_content_sha256']}")
    else:
        manifest = write_frozen_manifest(output_root, items, provenance)
        snapshot_sha = write_protocol_snapshot(output_root)
        write_environment(output_root, args.workers)
    print(f"[freeze] {len(items)} items, J_eval={j_eval}, manifest sha256={manifest['manifest_content_sha256']}")
    print(f"[freeze] protocol snapshot sha256={snapshot_sha}")

    receipts = run_evaluation(output_root, items, binding, workers=args.workers)

    summary = aggregate(items, receipts, j_eval)
    summary["frozen_manifest_sha256"] = manifest["manifest_content_sha256"]
    summary["protocol_snapshot_sha256"] = snapshot_sha
    summary["elapsed_seconds"] = time.monotonic() - started
    atomic_json(output_root / "summary.json", summary)
    atomic_write_text(output_root / "summary.md", render_summary_md(summary))
    write_asset_records(output_root, items, receipts)

    print(f"[done] summary written to {output_root / 'summary.json'}")
    print(json.dumps(summary["metrics"], indent=2, sort_keys=True))

    if not args.skip_verify:
        verifier_path = SCRIPT.with_name("verify_table2_supplementary_v1.py")
        spec = importlib.util.spec_from_file_location("verify_table2_supplementary_v1", verifier_path)
        if spec is None or spec.loader is None:
            raise RuntimeError("cannot load verifier")
        verifier = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(verifier)
        verification = verifier.verify_run(
            output_root,
            table1_manifest=TABLE1_MANIFEST,
            expected_table1_sha256=EXPECTED_TABLE1_MANIFEST_SHA256,
            identity_fields=INPUT_IDENTITY_FIELDS,
            table1_id_key="asset_id",
        )
        atomic_json(output_root / "verification.json", verification)
        if verification["status"] != "PASS":
            print(f"[verify] FAILED: {output_root / 'verification.json'}")
            return 1
        print(f"[verify] PASS ({len(verification['checks'])} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
