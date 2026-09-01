#!/usr/bin/env python3
"""Run Supplementary Table S1 for the frozen Artiverse Table 1 cohort.

The cohort is exactly ``.assets[].manifest_root`` from the frozen Table 1
manifest, in its existing order.  Mechanical evidence is rescanned from each
bound package.  The no-allowance strict outcome is reused only after binding
the asset to the frozen Table 4 manifest and per-asset strict result.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import platform
import sys
import tempfile
import time
from typing import Any, Mapping
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import lam_supplementary_static as static  # noqa: E402


PROTOCOL_ID = "s1_artiverse_table1cohort_n800_seed20260813_v1"
SCHEMA_VERSION = "supplementary-s1-artiverse/v1"
DATASET = "Artiverse"
CLASSIFICATION = "FORMAL"

DATASET_ROOT = REPO / "exp/artiverse"
TABLE1_MANIFEST = REPO / "exp/runtime/table1_artiverse/manifest.json"
TABLE4_MANIFEST = REPO / "exp/runtime/urdf_table4_artiverse_table1_n800_20260814/frozen_manifest.json"
TABLE4_RECORDS = TABLE4_MANIFEST.with_name("asset_records.json")
TABLE4_VERIFICATION = TABLE4_MANIFEST.with_name("verification.json")
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"

EXPECTED_TABLE1_SHA256 = "f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c"
EXPECTED_TABLE4_SHA256 = "0e69335a3d1574a1e1510124ade6e743cfd66fe894c1da3816b072954c75aedb"
EXPECTED_TABLE4_RECORDS_SHA256 = "b112bbfd8d6094a4c109ee25faf6c79d9e9898797bfce99dcb053190cb8c3952"
EXPECTED_TABLE4_VERIFICATION_SHA256 = "a0db9b3846e33e448ab5cba0e385377e4883ed40be78da7fff4d26a3464fb066"
EXPECTED_N_EVAL = 800
DEFAULT_WORKERS = 16

PAIR_POLICY = {
    "eligible_pairs": "distinct source-URDF links with collision geometry",
    "shared_topology_exclusion": "exclude_direct_parent_child",
    "method_specific_allowance": "none in headline",
    "surface_contact_allowed": True,
    "penetration_threshold_m": 1e-6,
}

S1_IDENTITY_FIELDS = (
    "selection_index", "asset_id", "manifest_root", "dataset_id", "model_id",
    "raw_category", "source", "selection_rank", "package",
    "primary_urdf_relative_path", "urdf_sha256_expected",
    "collision_mesh_files_expected", "table4_input_identity_sha256",
    "strict_pass_no_method_allowance",
)


class ProtocolViolation(RuntimeError):
    """Raised when a frozen input or its identity binding has drifted."""


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


def utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def timestamp_tag() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def atomic_json(path: Path, value: Any) -> None:
    atomic_write_text(path, canonical_json(value) + "\n")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def ratio(passed: int, denominator: int) -> dict[str, int | float | None]:
    rate = passed / denominator if denominator else None
    return {
        "passed": passed,
        "denominator": denominator,
        "rate": rate,
        "percentage": None if rate is None else rate * 100.0,
    }


def freeze_cohort(
    *,
    table1_manifest: Path,
    expected_table1_sha256: str,
    table4_manifest: Path,
    expected_table4_sha256: str,
    table4_records: Path,
    expected_table4_records_sha256: str,
    table4_verification: Path,
    expected_table4_verification_sha256: str,
    dataset_root: Path,
    expected_size: int,
    limit: int | None,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    observed_table1 = sha256_file(table1_manifest)
    if observed_table1 != expected_table1_sha256:
        raise ProtocolViolation(
            f"Table 1 manifest SHA-256 mismatch: expected {expected_table1_sha256}, observed {observed_table1}"
        )
    observed_table4 = sha256_file(table4_manifest)
    if observed_table4 != expected_table4_sha256:
        raise ProtocolViolation(
            f"Table 4 manifest SHA-256 mismatch: expected {expected_table4_sha256}, observed {observed_table4}"
        )
    observed_table4_records = sha256_file(table4_records)
    if observed_table4_records != expected_table4_records_sha256:
        raise ProtocolViolation(
            "Table 4 records SHA-256 mismatch: expected "
            f"{expected_table4_records_sha256}, observed {observed_table4_records}"
        )
    observed_table4_verification = sha256_file(table4_verification)
    if observed_table4_verification != expected_table4_verification_sha256:
        raise ProtocolViolation(
            "Table 4 verification SHA-256 mismatch: expected "
            f"{expected_table4_verification_sha256}, observed {observed_table4_verification}"
        )

    table1 = load_json(table1_manifest)
    frozen_table4 = load_json(table4_manifest)
    strict_rows = load_json(table4_records)
    table4_receipt = load_json(table4_verification)
    receipt_artifacts = table4_receipt.get("artifact_sha256", {})
    if (
        table4_receipt.get("status") != "PASS"
        or receipt_artifacts.get("frozen_manifest.json") != observed_table4
        or receipt_artifacts.get("asset_records.json") != observed_table4_records
    ):
        raise ProtocolViolation("Table 4 verification receipt does not bind the frozen inputs")
    table1_assets = table1.get("assets")
    table4_items = frozen_table4.get("items")
    if table1.get("dataset") != DATASET:
        raise ProtocolViolation(f"unexpected Table 1 dataset: {table1.get('dataset')!r}")
    if not isinstance(table1_assets, list) or len(table1_assets) != expected_size:
        raise ProtocolViolation(f"Table 1 must contain exactly {expected_size} assets")
    if not isinstance(table4_items, list) or len(table4_items) != expected_size:
        raise ProtocolViolation(f"Table 4 must contain exactly {expected_size} items")
    if not isinstance(strict_rows, list) or len(strict_rows) != expected_size:
        raise ProtocolViolation(f"Table 4 records must contain exactly {expected_size} rows")

    table4_by_root = {row.get("manifest_root"): row for row in table4_items}
    strict_by_id = {row.get("dataset_id"): row for row in strict_rows}
    if len(table4_by_root) != expected_size or len(strict_by_id) != expected_size:
        raise ProtocolViolation("Table 4 manifest or records contain duplicate identities")

    items: list[dict[str, Any]] = []
    for index, source in enumerate(table1_assets):
        manifest_root = source.get("manifest_root")
        frozen = table4_by_root.get(manifest_root)
        if not isinstance(manifest_root, str) or frozen is None:
            raise ProtocolViolation(f"Table 1 asset is absent from Table 4: {manifest_root!r}")
        dataset_id = frozen.get("dataset_id", frozen.get("asset_id"))
        strict = strict_by_id.get(dataset_id)
        strict_pass = strict.get("strict_collision_pass") if isinstance(strict, Mapping) else None
        if not isinstance(strict_pass, bool):
            raise ProtocolViolation(f"invalid or missing strict result for {manifest_root}")
        expected_result_identity = {
            "manifest_root": manifest_root,
            "input_identity_sha256": frozen.get("input_identity_sha256"),
            "order": frozen.get("order", index),
            "protocol_id": frozen_table4.get("protocol_id"),
        }
        observed_result_identity = {
            key: strict.get(key) for key in expected_result_identity
        }
        if observed_result_identity != expected_result_identity:
            raise ProtocolViolation(
                f"Table 4 result identity mismatch for {manifest_root}: "
                f"expected {expected_result_identity}, observed {observed_result_identity}"
            )
        expected_primary = f"{manifest_root}/urdf_w_collider/{frozen['model_id']}.urdf"
        if frozen.get("primary_urdf_relpath") != expected_primary:
            raise ProtocolViolation(f"unexpected primary URDF binding for {manifest_root}")
        item = {
            "selection_index": index,
            "asset_id": manifest_root,
            "manifest_root": manifest_root,
            "dataset_id": dataset_id,
            "model_id": frozen["model_id"],
            "raw_category": frozen.get("raw_category", source.get("raw_category")),
            "source": frozen.get("source", source.get("source")),
            "selection_rank": source.get("selection_rank", index + 1),
            "package": (dataset_root / manifest_root / "urdf_w_collider").as_posix(),
            "primary_urdf_relative_path": f"{frozen['model_id']}.urdf",
            "urdf_sha256_expected": frozen.get("urdf_sha256"),
            "collision_mesh_files_expected": frozen.get("collision_mesh_files", []),
            "table4_input_identity_sha256": frozen.get("input_identity_sha256"),
            "strict_pass_no_method_allowance": strict_pass,
        }
        item["s1_input_identity_sha256"] = canonical_sha256(
            {field: item[field] for field in S1_IDENTITY_FIELDS}
        )
        items.append(item)

    ordered_roots = [item["manifest_root"] for item in items]
    table4_ordered_roots = [item.get("manifest_root") for item in table4_items]
    if ordered_roots != table4_ordered_roots:
        raise ProtocolViolation("Table 1 and Table 4 asset order differ")
    if limit is not None:
        items = items[:limit]
    return items, {
        "n_eval": len(items),
        "intended_full_cohort": expected_size,
        "strict_passed": sum(bool(item["strict_pass_no_method_allowance"]) for item in items),
        "table1_manifest": str(table1_manifest),
        "table1_manifest_sha256": observed_table1,
        "table4_manifest": str(table4_manifest),
        "table4_manifest_sha256": observed_table4,
        "table4_records": str(table4_records),
        "table4_records_sha256": observed_table4_records,
        "table4_verification": str(table4_verification),
        "table4_verification_sha256": observed_table4_verification,
        "ordered_manifest_roots_sha256": canonical_sha256(ordered_roots[: len(items)]),
        "selection_policy": "exact Table 1 .assets[].manifest_root order; no resampling, replacement, or result filtering",
    }


def _path_within(path: Path, root: Path) -> bool:
    try:
        path.resolve(strict=True).relative_to(root.resolve(strict=True))
        return True
    except (OSError, ValueError):
        return False


def verify_asset_binding(item: Mapping[str, Any], dataset_root: Path) -> dict[str, Any]:
    issues: list[str] = []
    package = Path(str(item["package"]))
    if not _path_within(package, dataset_root):
        issues.append("package_missing_or_outside_dataset_root")
        return {"verified": False, "issues": issues}
    urdf = package / str(item["primary_urdf_relative_path"])
    if not urdf.is_file() or urdf.is_symlink():
        issues.append("primary_urdf_missing_or_symlink")
    elif sha256_file(urdf) != item.get("urdf_sha256_expected"):
        issues.append("primary_urdf_sha256_mismatch")

    for mesh in item.get("collision_mesh_files_expected", []):
        if not mesh.get("safe") or not mesh.get("exists"):
            continue
        relpath = mesh.get("resolved_relpath")
        if not isinstance(relpath, str):
            issues.append("collision_mesh_relpath_missing")
            continue
        full = dataset_root / relpath
        if not _path_within(full, dataset_root) or not full.is_file() or full.is_symlink():
            issues.append(f"collision_mesh_missing_or_unsafe:{relpath}")
        elif sha256_file(full) != mesh.get("sha256"):
            issues.append(f"collision_mesh_sha256_mismatch:{relpath}")
    return {"verified": not issues, "issues": issues}


def failed_evidence(reason: str) -> dict[str, Any]:
    return {
        "status": "NOT_EVALUABLE",
        "receipt": {"receipt_bound_asset": 0, "issues": [reason]},
        "receipt_replay": {"passed": False, "status": "NOT_RUN_INPUT_FAILURE"},
        "rebuild": {"eligible_asset": 0, "status": "N/E"},
        "allowance": {
            "status": "NOT_EVALUABLE",
            "registered_excluded_pair_count": None,
            "eligible_nonadjacent_pair_count": None,
            "issues": [reason],
        },
    }


def evaluate_asset(item: Mapping[str, Any], *, dataset_root: Path) -> dict[str, Any]:
    binding = verify_asset_binding(item, dataset_root)
    if not binding["verified"]:
        return {
            **dict(item),
            "status": "binding_failed",
            "binding": binding,
            "resource_closure": {"status": "NOT_EVALUABLE", "complete": False, "sha256": None},
            "s1_evidence": failed_evidence(";".join(binding["issues"])),
            "registered_allowance_strict_pass": None,
        }

    audit = static.audit_lam_package(
        Path(str(item["package"])),
        urdf_relative_path=str(item["primary_urdf_relative_path"]),
        asset_id=str(item["asset_id"]),
    )
    evidence = audit.get("s1_evidence")
    if not isinstance(evidence, dict):
        evidence = failed_evidence("s1_evidence_missing_from_static_audit")
    receipt_bound = int(evidence.get("receipt", {}).get("receipt_bound_asset", 0))
    evidence["receipt_replay"] = {
        "eligible_receipt_count": receipt_bound,
        "attempted": 0,
        "passed": False,
        "status": (
            "NOT_RUN_NO_VALID_RECEIPT"
            if receipt_bound == 0
            else "NOT_EVALUABLE_NO_REGISTERED_REPLAY_BACKEND"
        ),
    }
    allowance = evidence.get("allowance", {})
    discovered_pairs = sum(
        int(record.get("registered_pair_count") or 0)
        for record in allowance.get("records", [])
        if isinstance(record, Mapping) and record.get("valid") is True
    )
    allowance["discovery_status"] = allowance.get("status")
    allowance["discovery_issues"] = list(allowance.get("issues", []))
    allowance["discovered_unregistered_pair_count"] = discovered_pairs
    urdf_path = Path(str(item["package"])) / str(item["primary_urdf_relative_path"])
    try:
        eligible_pairs, topology_issues = static._eligible_nonadjacent_pairs(
            ET.parse(urdf_path).getroot()
        )
    except (OSError, ET.ParseError) as exc:
        eligible_pairs = set()
        topology_issues = [f"allowance_topology_parse_failed: {type(exc).__name__}: {exc}"]
    allowance["status"] = "COMPLETE" if not topology_issues else "NOT_EVALUABLE"
    allowance["eligible_nonadjacent_pair_count"] = (
        len(eligible_pairs) if not topology_issues else None
    )
    allowance["registered_excluded_pair_count"] = 0
    allowance["issues"] = topology_issues
    allowance["registration_status"] = "NO_PREREGISTERED_METHOD_SPECIFIC_REGISTRY"
    registered_strict = bool(item["strict_pass_no_method_allowance"])
    return {
        **dict(item),
        "status": "completed" if audit.get("status") == "completed" else "audit_failed",
        "binding": binding,
        "resource_closure": audit.get("resource_closure"),
        "s1_evidence": evidence,
        "registered_allowance_strict_pass": registered_strict,
        "audit_issues": audit.get("issues", []),
    }


def evidence_inventory_for_package(package: Path) -> dict[str, list[dict[str, Any]]]:
    candidates: dict[str, list[dict[str, Any]]] = {
        "receipt_candidates": [],
        "rebuild_recipe_candidates": [],
        "allowance_candidates": [],
    }
    for path in static._iter_regular_package_files(package):
        lower = path.name.lower()
        entry = {
            "path": path.relative_to(package).as_posix(),
            "sha256": sha256_file(path),
            "size_bytes": path.stat().st_size,
        }
        if path.suffix.lower() == ".json" and static.RECEIPT_NAME_RE.search(lower):
            candidates["receipt_candidates"].append(entry)
        if lower in static.REBUILD_RECIPE_NAMES:
            candidates["rebuild_recipe_candidates"].append(entry)
        if path.suffix.lower() == ".json" and static.ALLOWANCE_NAME_RE.search(lower):
            candidates["allowance_candidates"].append(entry)
    return candidates


def build_evidence_inventory(items: list[Mapping[str, Any]]) -> dict[str, Any]:
    return {
        "schema_version": "s1-artiverse-evidence-inventory/v1",
        "frozen_at_utc": utc_now_iso(),
        "allowance_registry": {
            "status": "ABSENT_FROZEN_EMPTY",
            "registered_pair_count": 0,
            "policy": "live package candidates are discovery-only and never preregistered",
        },
        "assets": [
            {
                "selection_index": item["selection_index"],
                "asset_id": item["asset_id"],
                **evidence_inventory_for_package(Path(str(item["package"]))),
            }
            for item in items
        ],
    }


def multiprocessing_map(function: Any, items: list[Any], *, workers: int) -> list[Any]:
    """Map CPU-heavy package audits across forked workers in input order."""
    context = multiprocessing.get_context("fork")
    with context.Pool(processes=workers) as pool:
        return pool.map(function, items)


def evaluate_asset_payload(payload: tuple[dict[str, Any], str]) -> dict[str, Any]:
    item, dataset_root = payload
    try:
        return evaluate_asset(item, dataset_root=Path(dataset_root))
    except Exception as exc:  # noqa: BLE001
        return {
            **item,
            "status": "worker_error",
            "binding": {"verified": False, "issues": [f"{type(exc).__name__}: {exc}"]},
            "resource_closure": {"status": "NOT_EVALUABLE", "complete": False, "sha256": None},
            "s1_evidence": failed_evidence(f"worker_error:{type(exc).__name__}:{exc}"),
            "registered_allowance_strict_pass": None,
        }


def aggregate(records: list[Mapping[str, Any]], *, intended_assets: int) -> dict[str, Any]:
    receipt_bound = 0
    receipt_replayed = 0
    rebuild_eligible = 0
    rebuild_matched = 0
    rebuild_complete = True
    registered_pairs = 0
    eligible_pairs = 0
    allowance_complete = True
    allowance_measured_assets = 0
    strict_passed = 0
    registered_passed = 0
    registered_outcomes_complete = True
    status_counts: dict[str, int] = {}

    for record in records:
        status = str(record.get("status", "missing"))
        status_counts[status] = status_counts.get(status, 0) + 1
        binding = record.get("binding")
        result_eligible = status == "completed" and (
            not isinstance(binding, Mapping) or binding.get("verified") is True
        )
        evidence = record.get("s1_evidence") if isinstance(record.get("s1_evidence"), Mapping) else {}
        receipt = evidence.get("receipt") if isinstance(evidence.get("receipt"), Mapping) else {}
        replay = evidence.get("receipt_replay") if isinstance(evidence.get("receipt_replay"), Mapping) else {}
        rebuild = evidence.get("rebuild") if isinstance(evidence.get("rebuild"), Mapping) else {}
        allowance = evidence.get("allowance") if isinstance(evidence.get("allowance"), Mapping) else {}
        receipt_bound += int(bool(receipt.get("receipt_bound_asset")))
        receipt_replayed += int(bool(replay.get("passed")))
        eligible = int(bool(rebuild.get("eligible_asset")))
        rebuild_eligible += eligible
        rebuild_matched += int(bool(record.get("deterministic_rebuild_match"))) if eligible else 0
        if eligible and record.get("rebuild_replay_status") != "COMPLETE":
            rebuild_complete = False
        if allowance.get("status") != "COMPLETE":
            allowance_complete = False
        else:
            allowance_measured_assets += 1
            registered_pairs += int(allowance.get("registered_excluded_pair_count", 0))
            eligible_pairs += int(allowance.get("eligible_nonadjacent_pair_count", 0))
        strict_passed += int(result_eligible and bool(record.get("strict_pass_no_method_allowance")))
        registered_outcome = record.get("registered_allowance_strict_pass")
        if result_eligible and isinstance(registered_outcome, bool):
            registered_passed += int(registered_outcome)
        else:
            registered_outcomes_complete = False

    rebuild_metric: dict[str, Any] = {
        "status": "N/E" if rebuild_eligible == 0 else ("COMPLETE" if rebuild_complete else "NOT_EVALUABLE"),
        "passed": None if rebuild_eligible == 0 else rebuild_matched,
        "denominator": rebuild_eligible,
        "rate": None if rebuild_eligible == 0 else rebuild_matched / rebuild_eligible,
        "percentage": None if rebuild_eligible == 0 else 100.0 * rebuild_matched / rebuild_eligible,
        "eligible_assets": rebuild_eligible,
        "asset_denominator": intended_assets,
    }
    allowance_rate = registered_pairs / eligible_pairs if eligible_pairs else (0.0 if allowance_complete else None)
    allowance_metric = {
        "status": "COMPLETE" if allowance_complete else "PARTIAL",
        "registered_pairs": registered_pairs,
        "eligible_pairs": eligible_pairs,
        "rate": allowance_rate,
        "percentage": None if allowance_rate is None else allowance_rate * 100.0,
        "measured_assets": allowance_measured_assets,
        "intended_assets": intended_assets,
    }
    strict_metric = ratio(strict_passed, intended_assets)
    if registered_pairs == 0 and allowance_complete:
        gain_metric = {
            "status": "COMPLETE",
            "value": 0.0,
            "registered_passed": strict_passed,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
        }
    elif registered_outcomes_complete:
        gain_metric = {
            "status": "COMPLETE",
            "value": 100.0 * (registered_passed - strict_passed) / intended_assets,
            "registered_passed": registered_passed,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
        }
    else:
        gain_metric = {
            "status": "NOT_EVALUABLE",
            "value": None,
            "registered_passed": None,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
            "reason": "registered allowance exists but no frozen sensitivity replay is available",
        }
    return {
        "protocol_id": PROTOCOL_ID,
        "schema_version": SCHEMA_VERSION,
        "dataset": DATASET,
        "n_eval": intended_assets,
        "status_counts": dict(sorted(status_counts.items())),
        "metrics": {
            "receipt_bound_assets": ratio(receipt_bound, intended_assets),
            "receipt_replay_pass": ratio(receipt_replayed, intended_assets),
            "deterministic_rebuild_match": rebuild_metric,
            "allowance_density": allowance_metric,
            "strict_pass_no_method_allowance": strict_metric,
            "registered_allowance_gain_pp": gain_metric,
        },
    }


def render_summary(summary: Mapping[str, Any]) -> str:
    metrics = summary["metrics"]

    def fraction(cell: Mapping[str, Any]) -> str:
        percentage = cell.get("percentage")
        suffix = "N/E" if percentage is None else f"{float(percentage):.2f}%"
        return f"{cell.get('passed')} / {cell.get('denominator')} ({suffix})"

    rebuild = metrics["deterministic_rebuild_match"]
    allowance = metrics["allowance_density"]
    gain = metrics["registered_allowance_gain_pp"]
    lines = [
        f"# Supplementary Table S1 - {summary['dataset']}",
        "",
        f"- Protocol: `{summary['protocol_id']}`",
        f"- N_eval: {summary['n_eval']}",
        f"- Status counts: {summary['status_counts']}",
        "",
        "| Metric | Result |",
        "|---|---|",
        f"| Receipt-bound Assets | {fraction(metrics['receipt_bound_assets'])} |",
        f"| Receipt Replay Pass | {fraction(metrics['receipt_replay_pass'])} |",
        f"| Deterministic Rebuild Match | {rebuild['status']} ({rebuild['eligible_assets']} / {rebuild['asset_denominator']} eligible) |",
        (
            f"| Allowance Density | {allowance['registered_pairs']} / {allowance['eligible_pairs']} "
            f"({allowance['percentage']:.2f}%) |"
            if allowance.get("percentage") is not None
            else f"| Allowance Density | {allowance['status']} "
            f"({allowance.get('measured_assets', 0)} / {allowance.get('intended_assets', summary['n_eval'])} assets) |"
        ),
        f"| Strict Pass (No Method-specific Allowance) | {fraction(metrics['strict_pass_no_method_allowance'])} |",
        f"| Registered-allowance Gain | {gain['value']} pp ({gain['status']}) |",
        "",
    ]
    return "\n".join(lines)


def default_output(limit: int | None) -> Path:
    if limit is None:
        name = f"s1_artiverse_table1cohort_n800_seed20260813_{timestamp_tag()}"
    else:
        name = f"s1_artiverse_smoke_n{limit}_{timestamp_tag()}"
    return REPO / "exp/runtime" / name


def build_manifest(
    output: Path,
    *,
    classification: str,
    completed_at: str,
    command: list[str],
    verification_status: str,
) -> dict[str, Any]:
    output_names = (
        "frozen_config.json", "environment.json", "evidence_inventory.json",
        "asset_records.jsonl", "summary.json", "summary.md", "protocol_snapshot.md",
        "verification.json",
    )
    verifier_path = SCRIPT.with_name("verify_s1_artiverse.py")
    return {
        "protocol_id": PROTOCOL_ID,
        "classification": classification,
        "created_at_utc": completed_at,
        "command": command,
        "outputs": {
            name: sha256_file(output / name) for name in output_names if (output / name).is_file()
        },
        "verifier": {"path": str(verifier_path), "sha256": sha256_file(verifier_path)},
        "verification_status": verification_status,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--skip-verify", action="store_true")
    args = parser.parse_args(argv)
    if args.limit is not None and not 1 <= args.limit <= EXPECTED_N_EVAL:
        raise SystemExit(f"--limit must be within 1..{EXPECTED_N_EVAL}")
    if args.workers <= 0:
        raise SystemExit("--workers must be positive")
    output = args.output or default_output(args.limit)
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)

    items, provenance = freeze_cohort(
        table1_manifest=TABLE1_MANIFEST,
        expected_table1_sha256=EXPECTED_TABLE1_SHA256,
        table4_manifest=TABLE4_MANIFEST,
        expected_table4_sha256=EXPECTED_TABLE4_SHA256,
        table4_records=TABLE4_RECORDS,
        expected_table4_records_sha256=EXPECTED_TABLE4_RECORDS_SHA256,
        table4_verification=TABLE4_VERIFICATION,
        expected_table4_verification_sha256=EXPECTED_TABLE4_VERIFICATION_SHA256,
        dataset_root=DATASET_ROOT,
        expected_size=EXPECTED_N_EVAL,
        limit=args.limit,
    )
    protocol_snapshot = PROTOCOL_DOCUMENT.read_text(encoding="utf-8")
    atomic_write_text(output / "protocol_snapshot.md", protocol_snapshot)
    evidence_inventory = build_evidence_inventory(items)
    atomic_json(output / "evidence_inventory.json", evidence_inventory)
    frozen_config = {
        "protocol_id": PROTOCOL_ID,
        "schema_version": SCHEMA_VERSION,
        "classification": CLASSIFICATION if args.limit is None else "SMOKE",
        "dataset": DATASET,
        "frozen_at_utc": utc_now_iso(),
        "cohort": provenance,
        "dataset_root": str(DATASET_ROOT),
        "pair_policy": PAIR_POLICY,
        "mechanical_evidence_policy": {
            "scanner": "lam_supplementary_static.s1_evidence",
            "receipt_missing_or_unreplayable": "fail_closed",
            "rebuild_without_public_frozen_recipe": "N/E",
            "allowance_registration_boundary": (
                "no method-specific registry exists; registry frozen empty before S1 evaluation; "
                "live package candidates are unregistered discovery evidence only"
            ),
        },
        "evidence_inventory": {
            "path": str(output / "evidence_inventory.json"),
            "sha256": sha256_file(output / "evidence_inventory.json"),
            "allowance_registry_status": "ABSENT_FROZEN_EMPTY",
        },
        "runner": {"path": str(SCRIPT), "sha256": sha256_file(SCRIPT)},
        "static_module": {"path": str(Path(static.__file__).resolve()), "sha256": sha256_file(Path(static.__file__).resolve())},
        "protocol_snapshot_sha256": hashlib.sha256(protocol_snapshot.encode("utf-8")).hexdigest(),
        "source_verification": {
            "table4_verification": str(TABLE4_VERIFICATION),
            "table4_verification_sha256": sha256_file(TABLE4_VERIFICATION),
            "table4_verification_status": load_json(TABLE4_VERIFICATION).get("status"),
        },
        "execution": {"workers": args.workers, "denominator_policy": "all frozen assets including failures"},
    }
    atomic_json(output / "frozen_config.json", frozen_config)
    atomic_json(output / "environment.json", {
        "python": platform.python_version(),
        "executable": sys.executable,
        "platform": platform.platform(),
        "workers": args.workers,
        "recorded_at_utc": utc_now_iso(),
    })

    started = time.monotonic()
    ordered_records = multiprocessing_map(
        evaluate_asset_payload,
        [(item, str(DATASET_ROOT)) for item in items],
        workers=args.workers,
    )
    print(f"[s1] completed {len(ordered_records)}/{len(items)}", flush=True)
    atomic_write_text(
        output / "asset_records.jsonl",
        "\n".join(canonical_json(record) for record in ordered_records) + "\n",
    )
    summary = aggregate(ordered_records, intended_assets=len(items))
    summary.update({
        "classification": CLASSIFICATION if args.limit is None else "SMOKE",
        "started_at_utc": frozen_config["frozen_at_utc"],
        "completed_at_utc": utc_now_iso(),
        "wall_seconds": round(time.monotonic() - started, 3),
        "frozen_config_sha256": sha256_file(output / "frozen_config.json"),
    })
    atomic_json(output / "summary.json", summary)
    atomic_write_text(output / "summary.md", render_summary(summary))

    verification: dict[str, Any] = {"status": "SKIPPED"}
    if not args.skip_verify:
        from exp.scripts import verify_s1_artiverse as verifier

        verification = verifier.verify_run(output)
        atomic_json(output / "verification.json", verification)
    manifest = build_manifest(
        output,
        classification=summary["classification"],
        completed_at=summary["completed_at_utc"],
        command=sys.argv if argv is None else [str(SCRIPT), *argv],
        verification_status=verification["status"],
    )
    atomic_json(output / "manifest.json", manifest)
    if not args.skip_verify:
        post_manifest = verifier.verify_run(output)
        if post_manifest["status"] != "PASS":
            print(json.dumps(post_manifest, indent=2, sort_keys=True))
            return 1
    manifest["manifest_self_sha256_at_write"] = sha256_file(output / "manifest.json")
    atomic_json(output / "manifest.json", manifest)
    print(json.dumps({"run_directory": str(output), **summary["metrics"], "verification": verification["status"]}, indent=2))
    return 0 if verification["status"] in {"PASS", "SKIPPED"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
