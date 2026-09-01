#!/usr/bin/env python3
"""Independent verifier for the LAM Table 2 supplementary run.

Recomputes every aggregate from the child receipts and the frozen manifest,
and checks denominator closure, cohort identity and artifact integrity.  The
verifier never trusts ``summary.json`` values without recomputation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Mapping

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]

# Frozen cohort identity constants (identical to run_urdf_table2sup_lam.py).
EXPECTED_TABLE3_MANIFEST_FILE_SHA256 = "7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951"
EXPECTED_TABLE3_RECORDS_FILE_SHA256 = "7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94"
EXPECTED_ORDERED_ASSET_KEYS_SHA256 = "643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3"
TABLE3_MANIFEST = REPO / "exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/manifest.json"
TABLE3_RECORDS = REPO / "exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl"
ALLOWED_STATUSES = {"completed", "error", "timeout", "binding_failed"}

INPUT_IDENTITY_FIELDS = (
    "selection_index", "asset_id", "tier", "rel_path", "object_release_id",
    "category", "release_order", "selection_rank", "selection_hash",
    "package", "primary_urdf_relative_path", "expected_declared_joint_count",
    "model_urdf_sha256_expected", "source_root",
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


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


class CheckLog:
    def __init__(self) -> None:
        self.checks: list[dict[str, Any]] = []

    def add(self, name: str, passed: bool, detail: str = "") -> None:
        self.checks.append({"name": name, "passed": bool(passed), "detail": detail})

    @property
    def ok(self) -> bool:
        return all(check["passed"] for check in self.checks)


def ratio_cell(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": numerator,
        "denominator": denominator,
        "percentage": (100.0 * numerator / denominator) if denominator else None,
    }


def verify_run(output_root: Path) -> dict[str, Any]:
    log = CheckLog()
    output_root = Path(output_root)
    manifest_path = output_root / "frozen_manifest.json"
    summary_path = output_root / "summary.json"
    records_path = output_root / "asset_records.jsonl"
    if not (manifest_path.exists() and summary_path.exists() and records_path.exists()):
        log.add("required_artifacts_present", False, "frozen_manifest/summary/asset_records missing")
        return {"status": "FAIL", "output_root": str(output_root), "checks": log.checks}
    log.add("required_artifacts_present", True)

    manifest = load_json(manifest_path)
    declared_hash = manifest.get("manifest_content_sha256")
    body = {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    computed_hash = canonical_sha256(body)
    log.add(
        "frozen_manifest_self_hash",
        declared_hash == computed_hash,
        f"declared={declared_hash} computed={computed_hash}",
    )

    items = manifest.get("items", [])
    provenance = manifest.get("provenance", {})
    n_eval = provenance.get("n_eval")
    j_eval = provenance.get("j_eval")
    log.add("cohort_size", len(items) == n_eval, f"items={len(items)} n_eval={n_eval}")
    log.add(
        "j_eval_consistency",
        sum(item.get("expected_declared_joint_count", 0) for item in items) == j_eval,
        f"j_eval={j_eval}",
    )

    # Cohort identity against the frozen Table 3 LAM manifest (byte-level hash).
    if TABLE3_MANIFEST.exists():
        log.add(
            "table3_manifest_hash",
            sha256_file(TABLE3_MANIFEST) == EXPECTED_TABLE3_MANIFEST_FILE_SHA256,
        )
        table3 = load_json(TABLE3_MANIFEST)
        table3_order = [record["asset_key"] for record in table3.get("records", [])][: len(items)]
        item_order = [item["asset_id"] for item in items]
        log.add(
            "cohort_order_matches_table3",
            table3_order == item_order,
            f"first={item_order[0] if item_order else None}",
        )
        if len(items) == len(table3.get("records", [])):
            payload = json.dumps(table3_order, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
            log.add(
                "ordered_asset_keys_sha256",
                hashlib.sha256(payload).hexdigest() == EXPECTED_ORDERED_ASSET_KEYS_SHA256,
            )
        else:
            log.add(
                "ordered_asset_keys_sha256",
                True,
                f"skipped: partial cohort smoke run ({len(items)} of {len(table3.get('records', []))} items)",
            )
    else:
        log.add("table3_manifest_hash", False, "Table 3 manifest not found")
    if TABLE3_RECORDS.exists():
        log.add(
            "table3_records_hash",
            sha256_file(TABLE3_RECORDS) == EXPECTED_TABLE3_RECORDS_FILE_SHA256,
        )
    else:
        log.add("table3_records_hash", False, "Table 3 asset_records not found")

    # Unique selection indices and input identity hashes.
    seen_indices = set()
    identity_ok = True
    for item in items:
        if item["selection_index"] in seen_indices:
            identity_ok = False
            break
        seen_indices.add(item["selection_index"])
        expected_identity = canonical_sha256({field: item.get(field) for field in INPUT_IDENTITY_FIELDS})
        if item.get("input_identity_sha256") != expected_identity:
            identity_ok = False
            break
    log.add("input_identity_hashes", identity_ok)

    # Load child receipts.
    receipts: dict[int, dict[str, Any]] = {}
    missing_receipts = 0
    identity_mismatch = 0
    bad_status = 0
    for item in items:
        receipt_path = output_root / "children" / f"rank_{item['selection_index']:04d}.json"
        if not receipt_path.exists():
            missing_receipts += 1
            continue
        receipt = load_json(receipt_path)
        receipts[item["selection_index"]] = receipt
        if receipt.get("input_identity_sha256") != item["input_identity_sha256"]:
            identity_mismatch += 1
        if receipt.get("status") not in ALLOWED_STATUSES:
            bad_status += 1
    log.add("receipts_present", missing_receipts == 0, f"missing={missing_receipts}")
    log.add("receipt_identity_matches", identity_mismatch == 0, f"mismatch={identity_mismatch}")
    log.add("receipt_statuses_allowed", bad_status == 0, f"bad={bad_status}")

    # Recompute aggregates from the audit atoms.
    asset_pass = 0
    link_declared = 0
    link_covered = 0
    link_complete_assets = 0
    portability_passed = 0
    dynamics_covered = 0
    complete_inertial = 0
    dynamic_links = 0
    parse_success = 0
    status_counts: dict[str, int] = {}
    for item in items:
        receipt = receipts.get(item["selection_index"])
        if receipt is None:
            continue
        status_counts[receipt["status"]] = status_counts.get(receipt["status"], 0) + 1
        audit = receipt["audit"]
        table2 = audit.get("table2_supplementary", {})
        visual = table2.get("visual_bearing_collision_coverage", {})
        portability = table2.get("joint_limit_portability", {})
        dynamics = table2.get("joint_dynamics_coverage", {})
        placeholder = table2.get("placeholder_mass_incidence", {})
        asset_pass += int(visual.get("asset_passed", 0))
        link_declared += int(visual.get("visual_bearing_links_declared", 0))
        link_covered += int(visual.get("covered_visual_bearing_links", 0))
        if visual.get("link_extraction_complete"):
            link_complete_assets += 1
        portability_passed += int(portability.get("joints_passed", 0))
        dynamics_covered += int(dynamics.get("joints_covered", 0))
        complete_inertial += int(placeholder.get("complete_inertial_links", 0))
        dynamic_links += int(placeholder.get("dynamic_links", 0))
        if audit.get("parse", {}).get("success"):
            parse_success += 1

    summary = load_json(summary_path)
    metrics = summary.get("metrics", {})
    visual_summary = metrics.get("visual_bearing_collision_coverage", {})
    portability_summary = metrics.get("joint_limit_portability", {})
    dynamics_summary = metrics.get("joint_dynamics_coverage", {})
    placeholder_summary = metrics.get("placeholder_mass_incidence", {})

    def matches(cell: Mapping[str, Any], numerator: int, denominator: int) -> bool:
        return int(cell.get("numerator", -1)) == numerator and int(cell.get("denominator", -1)) == denominator

    log.add(
        "summary_visual_asset_level",
        matches(visual_summary.get("asset_level", {}), asset_pass, n_eval),
        f"recomputed={asset_pass}/{n_eval}",
    )
    log.add(
        "summary_visual_link_micro",
        matches(visual_summary.get("link_micro", {}), link_covered, link_declared),
        f"recomputed={link_covered}/{link_declared}",
    )
    log.add(
        "summary_link_extraction_coverage",
        matches(visual_summary.get("link_extraction_coverage", {}), link_complete_assets, n_eval),
        f"recomputed={link_complete_assets}/{n_eval}",
    )
    log.add(
        "summary_portability",
        matches(portability_summary.get("joint_level", {}), portability_passed, j_eval),
        f"recomputed={portability_passed}/{j_eval}",
    )
    log.add(
        "summary_dynamics",
        matches(dynamics_summary.get("joint_level", {}), dynamics_covered, j_eval),
        f"recomputed={dynamics_covered}/{j_eval}",
    )
    log.add(
        "summary_placeholder_coverage",
        matches(placeholder_summary.get("complete_inertial_coverage", {}), complete_inertial, dynamic_links),
        f"recomputed={complete_inertial}/{dynamic_links}",
    )
    log.add(
        "summary_n_eval_j_eval",
        summary.get("n_eval") == n_eval and summary.get("j_eval") == j_eval,
    )
    log.add(
        "summary_status_counts",
        summary.get("status_counts") == dict(sorted(status_counts.items())),
        f"recomputed={dict(sorted(status_counts.items()))}",
    )
    log.add(
        "summary_parse_success",
        summary.get("parse_success_assets") == parse_success,
        f"recomputed={parse_success}",
    )
    log.add(
        "summary_manifest_hash_binding",
        summary.get("frozen_manifest_sha256") == declared_hash,
    )

    # Protocol snapshot integrity.
    snapshot_path = output_root / "protocol_snapshot.md"
    if snapshot_path.exists():
        log.add(
            "protocol_snapshot_hash",
            sha256_file(snapshot_path) == summary.get("protocol_snapshot_sha256"),
        )
    else:
        log.add("protocol_snapshot_hash", False, "protocol_snapshot.md missing")

    # asset_records.jsonl integrity: selection order + per-asset pass fields.
    record_lines = [line for line in records_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    records_ok = len(record_lines) == len(items)
    if records_ok:
        for line, item in zip(record_lines, items):
            record = json.loads(line)
            receipt = receipts[item["selection_index"]]
            audit = receipt["audit"]
            table2 = audit.get("table2_supplementary", {})
            visual = table2.get("visual_bearing_collision_coverage", {})
            portability = table2.get("joint_limit_portability", {})
            dynamics = table2.get("joint_dynamics_coverage", {})
            if (
                record.get("asset_id") != item["asset_id"]
                or record.get("selection_index") != item["selection_index"]
                or record.get("status") != receipt["status"]
                or bool(record.get("visual_bearing_collision_coverage_asset_pass")) != bool(visual.get("asset_pass"))
                or int(record.get("joint_limit_portability_passed", -1)) != int(portability.get("joints_passed", 0))
                or int(record.get("joint_dynamics_covered", -1)) != int(dynamics.get("joints_covered", 0))
            ):
                records_ok = False
                break
    log.add("asset_records_consistent", records_ok)

    return {
        "status": "PASS" if log.ok else "FAIL",
        "output_root": str(output_root),
        "recomputed": {
            "asset_pass": asset_pass,
            "n_eval": n_eval,
            "link_micro": f"{link_covered}/{link_declared}",
            "portability": f"{portability_passed}/{j_eval}",
            "dynamics": f"{dynamics_covered}/{j_eval}",
            "complete_inertial": f"{complete_inertial}/{dynamic_links}",
            "status_counts": dict(sorted(status_counts.items())),
        },
        "checks": log.checks,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_root", type=Path)
    args = parser.parse_args(argv)
    result = verify_run(args.output_root)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
