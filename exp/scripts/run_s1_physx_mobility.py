#!/usr/bin/env python3
"""Run Supplementary Table S1 for the frozen PhysX-Mobility cohort."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import lam_supplementary_static as static_atoms  # noqa: E402
from exp.scripts import run_table1_physx_mobility as table1_physx  # noqa: E402


PROTOCOL_ID = "s1_physx_mobility_table5cohort_n800_v1"
SCHEMA_VERSION = "supplementary-s1-physx-mobility/v1"
DATASET = "PhysX-Mobility"
N_EVAL = 800
DEFAULT_WORKERS = 16
DATASET_ROOT = REPO / "exp/PhysX-Mobility/extracted/PhysX_mobility"
SOURCE_RECEIPT_SET = REPO / "exp/runtime/table5_physx_mobility_n800_v2"
SOURCE_MANIFEST = SOURCE_RECEIPT_SET / "manifest.json"
TABLE4_RUN = REPO / "exp/runtime/urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z"
TABLE4_MANIFEST = TABLE4_RUN / "manifest.json"
TABLE4_ASSET_RECORDS = TABLE4_RUN / "asset_records.json"
TABLE4_STATE_RECORDS = TABLE4_RUN / "state_records.jsonl"
TABLE4_ARTIFACT_MANIFEST = TABLE4_RUN / "artifact_manifest.json"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
VERIFIER_PATH = SCRIPT.with_name("verify_s1_physx_mobility.py")
DEFAULT_OUTPUT_PARENT = REPO / "exp/runtime"
MANIFEST_ARTIFACT_NAMES = (
    "protocol_snapshot.md",
    "frozen_config.json",
    "asset_records.jsonl",
    "summary.json",
    "summary.md",
)
POST_MANIFEST_RECEIPTS = {
    "verification.json": (
        "independent verifier receipt generated after manifest finalization; "
        "excluded from the non-circular manifest artifact closure"
    )
}

EXPECTED_SOURCE_MANIFEST_SHA256 = "ccb54f4b726fe717efd28a37948e6b92bac994a2c0ba8fb4ea9ac4548d3a9882"
EXPECTED_COHORT_SHA256 = "a9c9c710d9617dea366696603984e330780ce177fead2a34c60410588cc1273c"
EXPECTED_ORDERED_IDS_SHA256 = "12dddf242e3e65dbe5d41b047752ed9b59d7ecb4387b81c9327bc0bc533172ad"
EXPECTED_TABLE4_MANIFEST_SHA256 = "671d107836fd1344fee34565aa9b16439b208598d179ee5bb59bbfb9bdbfef87"
EXPECTED_TABLE4_ASSET_RECORDS_SHA256 = "1efcfea5920e8db21f7b8afe9de0713a6bb478577d8b4417a6feb2b207397a3d"
EXPECTED_TABLE4_STATE_RECORDS_SHA256 = "eaa461e205e96b9bc2db4538de4295ba5efdd9bce0ac5d52ed0524648b6ec5fa"
EXPECTED_TABLE4_ARTIFACT_MANIFEST_SHA256 = "29193c31078ebe67c2b7202a092627c6e90a40db82d051c4d8dd02c2aa4e16f5"
EXPECTED_TABLE4_PROTOCOL_ID = "urdf_sim_ready_table4_physx_mobility_n800_v1"
EXPECTED_FORMAL_STRICT_PASSED = 786
EXPECTED_FORMAL_STATE_COUNT = 104453


class ProtocolViolation(RuntimeError):
    """Raised when frozen evidence differs from the registered inputs."""


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def atomic_write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_write_json(path: Path, value: Any) -> None:
    atomic_write_text(path, json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False) + "\n")


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw in enumerate(stream, start=1):
            if not raw.strip():
                continue
            value = json.loads(raw)
            if not isinstance(value, dict):
                raise ProtocolViolation(f"JSONL row {line_number} is not an object: {path}")
            records.append(value)
    return records


def ratio(passed: int, denominator: int) -> dict[str, int | float | None]:
    rate = passed / denominator if denominator else None
    return {
        "passed": passed,
        "denominator": denominator,
        "rate": rate,
        "percentage": None if rate is None else 100.0 * passed / denominator,
    }


def _nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ProtocolViolation(f"{label} must be a non-negative integer")
    return value


def _require_equal(observed: Any, expected: Any, label: str) -> None:
    if observed != expected:
        raise ProtocolViolation(f"{label} mismatch: expected {expected!r}, observed {observed!r}")


def reaggregate_table4_asset(
    item: Mapping[str, Any],
    asset: Mapping[str, Any],
    states: list[dict[str, Any]],
) -> dict[str, Any]:
    """Recompute one strict result from raw states and frozen expected counts."""

    dataset_id = str(item["dataset_id"])
    _require_equal(asset.get("state_records_sha256"), canonical_sha256(states), f"{dataset_id} state hash")
    phases = {"rest": [], "single_joint_sweep": [], "multi_joint_sobol": []}
    for state in states:
        for field, expected in (
            ("dataset_id", dataset_id),
            ("order", item.get("order")),
            ("protocol_id", item.get("protocol_id")),
        ):
            observed = str(state.get(field)) if field == "dataset_id" else state.get(field)
            _require_equal(observed, expected, f"{dataset_id} state {field}")
        phase = state.get("phase")
        if phase not in phases:
            raise ProtocolViolation(f"{dataset_id} has unknown Table 4 phase: {phase!r}")
        _nonnegative_int(
            state.get("non_adjacent_illegal_penetration_count"),
            f"{dataset_id} illegal penetration count",
        )
        phases[str(phase)].append(state)

    expected = {
        "rest": _nonnegative_int(item.get("rest_state_expected"), "rest expected"),
        "single_joint_sweep": _nonnegative_int(item.get("single_state_expected"), "single expected"),
        "multi_joint_sobol": _nonnegative_int(item.get("sobol_state_expected"), "sobol expected"),
    }
    executed = {phase: len(rows) for phase, rows in phases.items()}
    free = {
        phase: sum(row["non_adjacent_illegal_penetration_count"] == 0 for row in rows)
        for phase, rows in phases.items()
    }
    movable = _nonnegative_int(item.get("movable_dof_count"), "movable DoF count")
    range_evaluable = _nonnegative_int(asset.get("range_evaluable_dof_count"), "range-evaluable DoF count")
    rest_pass = executed["rest"] == expected["rest"] == free["rest"]
    single_pass = executed["single_joint_sweep"] == expected["single_joint_sweep"] == free["single_joint_sweep"]
    sobol_pass = bool(
        movable > 0
        and range_evaluable == movable
        and executed["multi_joint_sobol"] == expected["multi_joint_sobol"] == free["multi_joint_sobol"]
    )
    measurement_complete = bool(
        range_evaluable == movable and sum(executed.values()) == sum(expected.values())
    )
    strict_pass = bool(measurement_complete and rest_pass and single_pass and sobol_pass)
    recorded = {
        "rest_state_executed": executed["rest"],
        "rest_non_adjacent_free": free["rest"],
        "rest_non_adjacent_cf": rest_pass,
        "single_state_executed": executed["single_joint_sweep"],
        "single_non_adjacent_free": free["single_joint_sweep"],
        "single_joint_sweep_cf": single_pass,
        "sobol_state_executed": executed["multi_joint_sobol"],
        "sobol_non_adjacent_free": free["multi_joint_sobol"],
        "multi_joint_sobol_cf": sobol_pass,
        "measurement_complete": measurement_complete,
        "strict_collision_pass": strict_pass,
    }
    for field, value in recorded.items():
        _require_equal(asset.get(field), value, f"{dataset_id} Table 4 {field}")
    return {
        "dataset_id": dataset_id,
        "order": item["order"],
        "measurement_complete": measurement_complete,
        "strict_collision_pass": strict_pass,
        "state_record_count": len(states),
        "state_records_sha256": asset["state_records_sha256"],
        "table4_asset_record_sha256": canonical_sha256(asset),
    }


def load_cohort(*, verify_archive: bool, limit: int | None) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    _require_equal(sha256_file(SOURCE_MANIFEST), EXPECTED_SOURCE_MANIFEST_SHA256, "Table 5 manifest SHA-256")
    cohort = table1_physx.load_formal_cohort(
        SOURCE_RECEIPT_SET,
        DATASET_ROOT,
        expected_n=N_EVAL,
        verify_archive=verify_archive,
    )
    manifest = cohort["manifest"]
    _require_equal(manifest.get("cohort_sha256"), EXPECTED_COHORT_SHA256, "Table 5 cohort hash")
    rows = manifest.get("rows")
    if not isinstance(rows, list) or len(rows) != N_EVAL:
        raise ProtocolViolation("Table 5 manifest must contain exactly 800 rows")
    ordered_ids_sha256 = canonical_sha256([str(row["dataset_id"]) for row in rows])
    _require_equal(ordered_ids_sha256, EXPECTED_ORDERED_IDS_SHA256, "ordered dataset IDs hash")
    selected = rows if limit is None else rows[:limit]
    return selected, {
        "source_manifest": str(SOURCE_MANIFEST),
        "source_manifest_file_sha256": sha256_file(SOURCE_MANIFEST),
        "cohort_sha256": manifest["cohort_sha256"],
        "protocol_sha256": manifest["protocol_sha256"],
        "ordered_dataset_ids_sha256": ordered_ids_sha256,
        "selection": manifest["selection"],
        "source_receipt": manifest["source_receipt"],
        "archive_binding_verified": verify_archive,
        "n_eval": len(selected),
    }


def _verify_table4_artifact_manifest() -> None:
    _require_equal(
        sha256_file(TABLE4_ARTIFACT_MANIFEST),
        EXPECTED_TABLE4_ARTIFACT_MANIFEST_SHA256,
        "Table 4 artifact manifest SHA-256",
    )
    receipt = json.loads(TABLE4_ARTIFACT_MANIFEST.read_text(encoding="utf-8"))
    expected = {
        "manifest.json": EXPECTED_TABLE4_MANIFEST_SHA256,
        "asset_records.json": EXPECTED_TABLE4_ASSET_RECORDS_SHA256,
        "state_records.jsonl": EXPECTED_TABLE4_STATE_RECORDS_SHA256,
    }
    for name, digest in expected.items():
        _require_equal(receipt.get("files", {}).get(name, {}).get("sha256"), digest, f"artifact receipt {name}")


def load_table4_evidence(rows: list[dict[str, Any]], *, formal: bool) -> dict[str, Any]:
    for path, expected in (
        (TABLE4_MANIFEST, EXPECTED_TABLE4_MANIFEST_SHA256),
        (TABLE4_ASSET_RECORDS, EXPECTED_TABLE4_ASSET_RECORDS_SHA256),
        (TABLE4_STATE_RECORDS, EXPECTED_TABLE4_STATE_RECORDS_SHA256),
    ):
        _require_equal(sha256_file(path), expected, f"frozen Table 4 artifact {path.name}")
    _verify_table4_artifact_manifest()
    manifest = json.loads(TABLE4_MANIFEST.read_text(encoding="utf-8"))
    assets = json.loads(TABLE4_ASSET_RECORDS.read_text(encoding="utf-8"))
    states = load_jsonl(TABLE4_STATE_RECORDS)
    _require_equal(manifest.get("protocol_id"), EXPECTED_TABLE4_PROTOCOL_ID, "Table 4 protocol")
    _require_equal(
        manifest.get("manifest_content_sha256"),
        canonical_sha256({key: value for key, value in manifest.items() if key != "manifest_content_sha256"}),
        "Table 4 manifest self-hash",
    )
    items = manifest.get("items")
    if not isinstance(items, list) or not isinstance(assets, list) or len(items) != N_EVAL or len(assets) != N_EVAL:
        raise ProtocolViolation("Table 4 manifest and asset records must each contain 800 rows")
    states_by_order: dict[int, list[dict[str, Any]]] = {index: [] for index in range(N_EVAL)}
    previous = -1
    for state in states:
        order = _nonnegative_int(state.get("order"), "Table 4 state order")
        if order not in states_by_order or order < previous:
            raise ProtocolViolation("Table 4 states are outside or out of cohort order")
        previous = order
        states_by_order[order].append(state)
    if formal:
        _require_equal(len(states), EXPECTED_FORMAL_STATE_COUNT, "Table 4 state count")

    evidence: list[dict[str, Any]] = []
    for order, row in enumerate(rows):
        item = items[order]
        asset = assets[order]
        dataset_id = str(row["dataset_id"])
        for source, observed, label in (
            (dataset_id, str(item.get("dataset_id")), "manifest dataset_id"),
            (dataset_id, str(asset.get("dataset_id")), "asset dataset_id"),
            (order, item.get("order"), "manifest order"),
            (order, asset.get("order"), "asset order"),
            (row["rank"], item.get("rank"), "rank"),
            (row["rank_sha256"], item.get("rank_sha256"), "rank hash"),
            (row["urdf_sha256"], item.get("urdf_sha256"), "URDF hash"),
            (row["resource_sha256"], item.get("resource_sha256"), "resource hash"),
            (canonical_sha256(row), item.get("manifest_row_sha256"), "manifest row hash"),
            (0, item.get("declared_collision_elements"), "declared collision count"),
        ):
            _require_equal(observed, source, f"{dataset_id} Table 4 {label}")
        embedded = asset.get("state_records")
        if not isinstance(embedded, list) or embedded != states_by_order[order]:
            raise ProtocolViolation(f"{dataset_id} embedded and external Table 4 states differ")
        evidence.append(reaggregate_table4_asset(item, asset, states_by_order[order]))
    strict_passed = sum(record["strict_collision_pass"] for record in evidence)
    if formal:
        _require_equal(strict_passed, EXPECTED_FORMAL_STRICT_PASSED, "formal strict pass count")
    return {
        "protocol_id": manifest["protocol_id"],
        "manifest_path": str(TABLE4_MANIFEST),
        "manifest_file_sha256": sha256_file(TABLE4_MANIFEST),
        "asset_records_path": str(TABLE4_ASSET_RECORDS),
        "asset_records_file_sha256": sha256_file(TABLE4_ASSET_RECORDS),
        "state_records_path": str(TABLE4_STATE_RECORDS),
        "state_records_file_sha256": sha256_file(TABLE4_STATE_RECORDS),
        "artifact_manifest_path": str(TABLE4_ARTIFACT_MANIFEST),
        "artifact_manifest_file_sha256": sha256_file(TABLE4_ARTIFACT_MANIFEST),
        "strict_passed": strict_passed,
        "denominator": len(evidence),
        "state_record_count": sum(record["state_record_count"] for record in evidence),
        "records": evidence,
    }


def _expected_package_binding(dataset_root: Path, row: Mapping[str, Any]) -> dict[str, Any]:
    entries = [(str(row["urdf_relative_path"]), str(row["urdf_sha256"]))]
    entries.extend((str(value["relative_path"]), str(value["sha256"])) for value in row["resources"])
    file_rows: list[dict[str, Any]] = []
    for relative, digest in sorted(entries):
        path = dataset_root / relative
        file_rows.append({"path": relative, "bytes": path.stat().st_size, "sha256": digest})
    return {
        "file_count": len(file_rows),
        "total_bytes": sum(record["bytes"] for record in file_rows),
        "content_manifest_sha256": canonical_sha256(file_rows),
    }


def _stage_evidence_package(dataset_root: Path, row: Mapping[str, Any], parent: Path) -> Path:
    package = parent / str(row["dataset_id"])
    entries = [(str(row["urdf_relative_path"]), str(row["urdf_sha256"]))]
    entries.extend((str(value["relative_path"]), str(value["sha256"])) for value in row["resources"])
    entries.append((str(row["finaljson_relative_path"]), str(row["finaljson_sha256"])))
    seen: set[str] = set()
    for relative, expected in entries:
        if relative in seen:
            continue
        seen.add(relative)
        source = dataset_root / relative
        target = package / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        _require_equal(sha256_file(target), expected, f"staged {row['dataset_id']} {relative}")
    return package


def _audit_asset(payload: tuple[dict[str, Any], dict[str, Any], str, str]) -> dict[str, Any]:
    row, table4, dataset_root_raw, work_root_raw = payload
    dataset_root = Path(dataset_root_raw)
    work_root = Path(work_root_raw)
    dataset_id = str(row["dataset_id"])
    temporary = Path(tempfile.mkdtemp(prefix=f"asset_{dataset_id}_", dir=work_root))
    try:
        package = _stage_evidence_package(dataset_root, row, temporary)
        urdf = package / str(row["urdf_relative_path"])
        root = ET.parse(urdf).getroot()
        evidence = static_atoms._s1_evidence_atoms(  # noqa: SLF001
            root,
            package,
            urdf,
            {"complete": True, "sha256": row["resource_sha256"]},
        )
        expected_binding = _expected_package_binding(dataset_root, row)
        return {
            "schema_version": SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "selection_index": int(row["rank"]) - 1,
            "dataset_id": dataset_id,
            "rank": int(row["rank"]),
            "rank_sha256": row["rank_sha256"],
            "category": row["category"],
            "status": "completed",
            "binding_verified": True,
            "manifest_row_sha256": canonical_sha256(row),
            "urdf_sha256": row["urdf_sha256"],
            "resource_sha256": row["resource_sha256"],
            "package_binding": expected_binding,
            "declared_collision_elements": int(row["xml_counts"]["collision_elements"]),
            "receipt_candidate_count": int(evidence["receipt"]["candidate_count"]),
            "receipt_bound": bool(evidence["receipt"]["receipt_bound_asset"]),
            "receipt_replay_pass": False,
            "receipt_replay_status": "MISSING_FAIL_CLOSED" if not evidence["receipt"]["receipt_bound_asset"] else "NOT_REPLAYED_FAIL_CLOSED",
            "rebuild_recipe_candidate_count": int(evidence["rebuild"]["candidate_recipe_count"]),
            "rebuild_eligible": bool(evidence["rebuild"]["eligible_asset"]),
            "rebuild_match": None,
            "allowance_candidate_count": int(evidence["allowance"]["candidate_file_count"]),
            "allowance_complete": evidence["allowance"]["status"] == "COMPLETE",
            "registered_allowance_pairs": evidence["allowance"]["registered_excluded_pair_count"],
            "eligible_nonadjacent_pairs": evidence["allowance"]["eligible_nonadjacent_pair_count"],
            "strict_pass_no_method_allowance": bool(table4["strict_collision_pass"]),
            "registered_allowance_strict_pass": bool(table4["strict_collision_pass"]),
            "table4_measurement_complete": bool(table4["measurement_complete"]),
            "table4_state_record_count": int(table4["state_record_count"]),
            "table4_state_records_sha256": table4["state_records_sha256"],
            "table4_asset_record_sha256": table4["table4_asset_record_sha256"],
            "claim_boundary": "official URDF declares no collision elements; strict collision-free result is vacuous",
        }
    except Exception as exc:  # noqa: BLE001
        return {
            "schema_version": SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "selection_index": int(row["rank"]) - 1,
            "dataset_id": dataset_id,
            "status": "failed",
            "binding_verified": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
    finally:
        shutil.rmtree(temporary, ignore_errors=True)


def aggregate(records: list[Mapping[str, Any]], *, intended_assets: int) -> dict[str, Any]:
    completed = [record for record in records if record.get("status") == "completed" and record.get("binding_verified") is True]
    receipt_bound = sum(bool(record.get("receipt_bound")) for record in completed)
    receipt_replay = sum(bool(record.get("receipt_replay_pass")) for record in completed)
    rebuild_eligible = sum(bool(record.get("rebuild_eligible")) for record in completed)
    rebuild_match = sum(bool(record.get("rebuild_match")) for record in completed if record.get("rebuild_eligible"))
    allowance_complete = len(completed) == intended_assets and all(record.get("allowance_complete") is True for record in completed)
    measured_assets = sum(record.get("allowance_complete") is True for record in completed)
    registered_pairs = sum(int(record.get("registered_allowance_pairs") or 0) for record in completed if record.get("allowance_complete") is True)
    eligible_pairs = sum(int(record.get("eligible_nonadjacent_pairs") or 0) for record in completed if record.get("allowance_complete") is True)
    strict_passed = sum(bool(record.get("strict_pass_no_method_allowance")) for record in completed)
    status_counts: dict[str, int] = {}
    for record in records:
        status = str(record.get("status", "missing"))
        status_counts[status] = status_counts.get(status, 0) + 1
    if not allowance_complete:
        allowance = {
            "status": "PARTIAL",
            "registered_pairs": registered_pairs,
            "eligible_pairs": eligible_pairs,
            "rate": None,
            "percentage": None,
            "measured_assets": measured_assets,
            "intended_assets": intended_assets,
            "reason": "allowance extraction incomplete",
        }
    elif eligible_pairs == 0:
        allowance = {
            "status": "N/E",
            "registered_pairs": registered_pairs,
            "eligible_pairs": 0,
            "rate": None,
            "percentage": None,
            "measured_assets": measured_assets,
            "intended_assets": intended_assets,
            "reason": "no eligible non-adjacent collision-bearing source-link pairs",
        }
    else:
        rate = registered_pairs / eligible_pairs
        allowance = {
            "status": "COMPLETE",
            "registered_pairs": registered_pairs,
            "eligible_pairs": eligible_pairs,
            "rate": rate,
            "percentage": 100.0 * rate,
            "measured_assets": measured_assets,
            "intended_assets": intended_assets,
        }
    return {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "n_eval": intended_assets,
        "status_counts": dict(sorted(status_counts.items())),
        "claim_boundary": {
            "declared_collision_element_total": sum(int(record.get("declared_collision_elements", 0)) for record in completed),
            "strict_collision_outcome_is_vacuous": True,
        },
        "metrics": {
            "receipt_bound_assets": ratio(receipt_bound, intended_assets),
            "receipt_replay_pass": ratio(receipt_replay, intended_assets),
            "deterministic_rebuild_match": {
                "status": "N/E" if rebuild_eligible == 0 else "COMPLETE",
                "passed": None if rebuild_eligible == 0 else rebuild_match,
                "denominator": rebuild_eligible,
                "rate": None if rebuild_eligible == 0 else rebuild_match / rebuild_eligible,
                "percentage": None if rebuild_eligible == 0 else 100.0 * rebuild_match / rebuild_eligible,
                "eligible_assets": rebuild_eligible,
                "asset_denominator": intended_assets,
            },
            "allowance_density": allowance,
            "strict_pass_no_method_allowance": ratio(strict_passed, intended_assets),
            "registered_allowance_gain_pp": {
                "status": "COMPLETE" if registered_pairs == 0 and allowance_complete else "NOT_EVALUABLE",
                "value": 0.0 if registered_pairs == 0 and allowance_complete else None,
                "registered_passed": strict_passed if registered_pairs == 0 and allowance_complete else None,
                "no_allowance_passed": strict_passed,
                "denominator": intended_assets,
            },
        },
    }


def render_summary(summary: Mapping[str, Any]) -> str:
    metrics = summary["metrics"]

    def fraction(metric: Mapping[str, Any]) -> str:
        percentage = metric.get("percentage")
        suffix = "N/E" if percentage is None else f"{float(percentage):.2f}%"
        return f"{metric.get('passed')} / {metric.get('denominator')} ({suffix})"

    rebuild = metrics["deterministic_rebuild_match"]
    allowance = metrics["allowance_density"]
    gain = metrics["registered_allowance_gain_pp"]
    allowance_text = (
        f"{allowance['registered_pairs']} / {allowance['eligible_pairs']} ({allowance['percentage']:.2f}%)"
        if allowance.get("percentage") is not None
        else f"{allowance['registered_pairs']} / {allowance['eligible_pairs']} ({allowance['status']})"
    )
    return "\n".join(
        [
            f"# Supplementary Table S1 - {summary['dataset']}",
            "",
            f"- Protocol: `{summary['protocol_id']}`",
            f"- N_eval: {summary['n_eval']}",
            f"- Status counts: {summary['status_counts']}",
            "- Claim boundary: official URDFs declare zero collision elements; strict collision-free outcomes are vacuous.",
            "",
            "| Metric | Result |",
            "|---|---|",
            f"| Receipt-bound Assets | {fraction(metrics['receipt_bound_assets'])} |",
            f"| Receipt Replay Pass | {fraction(metrics['receipt_replay_pass'])} |",
            f"| Deterministic Rebuild Match | {rebuild['status']} ({rebuild['eligible_assets']} / {rebuild['asset_denominator']} eligible) |",
            f"| Allowance Density | {allowance_text} |",
            f"| Strict Pass (No Method-specific Allowance) | {fraction(metrics['strict_pass_no_method_allowance'])} |",
            f"| Registered-allowance Gain | {gain['value']:.2f} pp ({gain['status']}) |" if gain["value"] is not None else f"| Registered-allowance Gain | N/E ({gain['status']}) |",
            "",
        ]
    )


def _artifact_binding(path: Path) -> dict[str, Any]:
    return {"bytes": path.stat().st_size, "sha256": sha256_file(path)}


def default_output(mode: str, n: int) -> Path:
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    name = f"s1_physx_mobility_table5cohort_n800_{timestamp}" if mode == "formal" else f"s1_physx_mobility_smoke_n{n}_{timestamp}"
    return DEFAULT_OUTPUT_PARENT / name


def run_evaluation(
    output: Path,
    *,
    formal: bool,
    n: int,
    workers: int,
    verify_archive: bool,
) -> dict[str, Any]:
    if output.exists():
        if any(output.iterdir()):
            raise ProtocolViolation(f"output directory is not empty: {output}")
    else:
        output.mkdir(parents=True)
    started_at = utc_now()
    started = time.monotonic()
    rows, cohort = load_cohort(verify_archive=verify_archive, limit=None if formal else n)
    table4 = load_table4_evidence(rows, formal=formal)
    protocol_snapshot = PROTOCOL_DOCUMENT.read_text(encoding="utf-8")
    atomic_write_text(output / "protocol_snapshot.md", protocol_snapshot)
    frozen_config = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "classification": "FORMAL" if formal else "SMOKE",
        "dataset": DATASET,
        "frozen_at": started_at,
        "cohort": cohort,
        "table4_source": {key: value for key, value in table4.items() if key != "records"},
        "pair_policy": {
            "eligible_pairs": "distinct collision-bearing source links",
            "shared_topology_exclusion": "exclude direct parent-child",
            "method_specific_allowance": "none",
            "penetration_threshold_m": 1e-6,
        },
        "zero_denominator_policy": "Allowance Density is N/E when eligible collision-bearing pair count is zero",
        "claim_boundary": "official PhysX-Mobility URDFs declare zero collision elements; strict outcomes are vacuous collision evidence",
        "execution": {
            "workers": workers,
            "archive_binding_verified": verify_archive,
            "python": sys.version,
            "python_executable": sys.executable,
            "platform": platform.platform(),
        },
        "code_identity": {
            "runner_path": str(SCRIPT),
            "runner_sha256": sha256_file(SCRIPT),
            "verifier_path": str(VERIFIER_PATH),
            "verifier_sha256": sha256_file(VERIFIER_PATH),
            "static_atoms_path": str(Path(static_atoms.__file__).resolve()),
            "static_atoms_sha256": sha256_file(Path(static_atoms.__file__).resolve()),
            "cohort_loader_path": str(Path(table1_physx.__file__).resolve()),
            "cohort_loader_sha256": sha256_file(Path(table1_physx.__file__).resolve()),
            "table5_contract_path": str(Path(table1_physx.TABLE5.__file__).resolve()),
            "table5_contract_sha256": sha256_file(Path(table1_physx.TABLE5.__file__).resolve()),
        },
        "protocol_document": {"path": str(PROTOCOL_DOCUMENT), "sha256_at_freeze": hashlib.sha256(protocol_snapshot.encode("utf-8")).hexdigest()},
    }
    atomic_write_json(output / "frozen_config.json", frozen_config)
    work_root = output / ".asset_work"
    work_root.mkdir()
    try:
        payloads = [
            (row, table4_record, str(DATASET_ROOT), str(work_root))
            for row, table4_record in zip(rows, table4["records"], strict=True)
        ]
        if workers == 1:
            records = [_audit_asset(payload) for payload in payloads]
        else:
            with multiprocessing.get_context("fork").Pool(processes=workers) as pool:
                records = pool.map(_audit_asset, payloads)
    finally:
        shutil.rmtree(work_root, ignore_errors=True)
    atomic_write_text(output / "asset_records.jsonl", "".join(canonical_json(record) + "\n" for record in records))
    summary = aggregate(records, intended_assets=len(rows))
    summary.update(
        {
            "classification": "FORMAL" if formal else "SMOKE",
            "started_at": started_at,
            "completed_at": utc_now(),
            "wall_seconds": round(time.monotonic() - started, 3),
        }
    )
    if formal:
        metrics = summary["metrics"]
        contract = (
            summary["status_counts"] == {"completed": N_EVAL}
            and summary["claim_boundary"]["declared_collision_element_total"] == 0
            and metrics["receipt_bound_assets"]["passed"] == 0
            and metrics["receipt_replay_pass"]["passed"] == 0
            and metrics["deterministic_rebuild_match"]["eligible_assets"] == 0
            and metrics["allowance_density"]["status"] == "N/E"
            and metrics["allowance_density"]["registered_pairs"] == 0
            and metrics["allowance_density"]["eligible_pairs"] == 0
            and metrics["strict_pass_no_method_allowance"]["passed"] == EXPECTED_FORMAL_STRICT_PASSED
            and metrics["registered_allowance_gain_pp"]["value"] == 0.0
        )
        if not contract:
            raise ProtocolViolation("formal S1 aggregate contract mismatch")
    atomic_write_json(output / "summary.json", summary)
    atomic_write_text(output / "summary.md", render_summary(summary))
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "classification": summary["classification"],
        "dataset": DATASET,
        "created_at": summary["completed_at"],
        "n_eval": len(rows),
        "cohort": cohort,
        "table4_source": frozen_config["table4_source"],
        "code_identity": frozen_config["code_identity"],
        "artifacts": {
            name: _artifact_binding(output / name)
            for name in MANIFEST_ARTIFACT_NAMES
        },
        "post_manifest_receipts": POST_MANIFEST_RECEIPTS,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    atomic_write_json(output / "manifest.json", manifest)
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), required=True)
    parser.add_argument("--n", type=int)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--skip-archive-binding", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    n = N_EVAL if args.mode == "formal" else (args.n or 5)
    try:
        if args.workers < 1:
            raise ValueError("workers must be positive")
        if args.mode == "formal" and (n != N_EVAL or args.workers != DEFAULT_WORKERS or args.skip_archive_binding):
            raise ValueError("formal mode requires n=800, workers=16, and archive binding")
        if args.mode == "smoke" and not 1 <= n <= N_EVAL:
            raise ValueError("smoke n must be in [1, 800]")
        output = (args.output or default_output(args.mode, n)).resolve()
        run_evaluation(
            output,
            formal=args.mode == "formal",
            n=n,
            workers=args.workers,
            verify_archive=not args.skip_archive_binding,
        )
        command = [sys.executable, str(VERIFIER_PATH), "--run", str(output)]
        if args.mode == "formal":
            command.append("--formal")
        result = subprocess.run(command, check=False)
        if result.returncode != 0:
            raise RuntimeError(f"independent verifier exited {result.returncode}")
    except Exception as exc:  # noqa: BLE001
        print(f"S1 PhysX-Mobility evaluation failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
