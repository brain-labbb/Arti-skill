#!/usr/bin/env python3
"""Cross-check two complete LAM Naming runs and write self_check.json."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
DEFAULT_PRIMARY = REPO_ROOT / "exp" / "runtime" / "lam_naming_v1"
DEFAULT_REPRO = REPO_ROOT / ".tmp" / "lam_naming_repro_v11"


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(contained(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"expected object: {path}")
    return payload


def read_records(path: Path) -> tuple[list[dict[str, Any]], str]:
    data = contained(path).read_bytes()
    records = [json.loads(line) for line in data.decode("utf-8").splitlines()]
    return records, hashlib.sha256(data).hexdigest()


def file_sha256(path: Path) -> str:
    return hashlib.sha256(contained(path).read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--primary", type=Path, default=DEFAULT_PRIMARY)
    parser.add_argument("--repro", type=Path, default=DEFAULT_REPRO)
    args = parser.parse_args()
    primary = contained(args.primary)
    repro = contained(args.repro)
    first = read_json(primary / "summary.json")
    second = read_json(repro / "summary.json")
    first_records, first_sha = read_records(primary / "records.jsonl")
    second_records, second_sha = read_records(repro / "records.jsonl")
    first_report_sha = file_sha256(primary / "report.md")
    second_report_sha = file_sha256(repro / "report.md")

    first_stable = dict(first)
    second_stable = dict(second)
    first_stable.pop("generated_at_utc", None)
    second_stable.pop("generated_at_utc", None)
    coverage = first["coverage"]
    archive = first["archive_audit"]
    direct = first["direct_metrics"]
    sums = {
        "records": len(first_records),
        "manifest_links": sum(row["manifest_n_links"] for row in first_records),
        "parsed_links": sum(row["link_count"] for row in first_records),
        "parts": sum(row["renderable_part_count"] for row in first_records),
        "named_parts": sum(row["named_renderable_part_count"] for row in first_records),
        "placeholder_parts": sum(row["placeholder_renderable_part_count"] for row in first_records),
        "valid_visual_geometry": sum(row["valid_visual_geometry_count"] for row in first_records),
        "invalid_visual_geometry": sum(row["invalid_visual_geometry_count"] for row in first_records),
        "assets_with_placeholders": sum(row["placeholder_renderable_part_count"] > 0 for row in first_records),
    }
    checks = {
        "both_status_complete": first["status"] == second["status"] == "COMPLETE",
        "summary_equal_excluding_generated_at": first_stable == second_stable,
        "records_byte_identical": first_sha == second_sha,
        "reports_byte_identical": first_report_sha == second_report_sha,
        "records_sha_matches_primary_summary": first_sha == first["provenance"]["records_sha256"],
        "records_sha_matches_repro_summary": second_sha == second["provenance"]["records_sha256"],
        "stable_metrics_sha_equal": first["provenance"]["stable_metrics_sha256"] == second["provenance"]["stable_metrics_sha256"],
        "manifest_records_conserved": coverage["requested_viable_assets"] == sums["records"],
        "manifest_artifact_parse_evaluable_conserved": coverage["requested_viable_assets"] == coverage["artifact_found_assets"] == coverage["parse_success_assets"] == coverage["naming_evaluable_assets"],
        "tar_manifest_urdf_set_exact_match": archive["manifest_urdf_set_exact_match"] and archive["expected_generated_urdf_count"] == archive["generated_urdf_member_count"] == sums["records"],
        "intermediate_urdfs_excluded_from_final_set": archive["all_named_generated_urdf_member_count"] == archive["generated_urdf_member_count"] + archive["intermediate_log_generated_urdf_count"] and archive["intermediate_log_generated_urdf_count"] > 0,
        "manifest_parsed_links_conserved": sums["manifest_links"] == sums["parsed_links"],
        "parts_summary_conserved": sums["parts"] == direct["total_renderable_parts"],
        "named_summary_conserved": sums["named_parts"] == direct["total_named_renderable_parts"],
        "placeholder_conserved": sums["parts"] == sums["named_parts"] + sums["placeholder_parts"] and sums["placeholder_parts"] == direct["placeholder_renderable_parts"],
        "placeholder_asset_count_conserved": sums["assets_with_placeholders"] == direct["assets_with_placeholder_parts"],
        "geometry_summary_conserved": sums["valid_visual_geometry"] == direct["valid_visual_geometry_count"] and sums["invalid_visual_geometry"] == direct["invalid_visual_geometry_count"],
        "archive_unambiguous": archive["archive_safe_for_evaluation"] and archive["unsafe_member_count"] == archive["duplicate_member_count"] == archive["link_member_count"] == archive["special_member_count"] == 0,
    }
    payload = {
        "protocol_id": "nano3d_lam_naming_self_check_v1.1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "primary": str(primary.relative_to(WORKSPACE_ROOT)),
        "repro": str(repro.relative_to(WORKSPACE_ROOT)),
        "checks": checks,
        "conservation_sums": sums,
        "primary_records_sha256": first_sha,
        "repro_records_sha256": second_sha,
        "primary_report_sha256": first_report_sha,
        "repro_report_sha256": second_report_sha,
        "stable_metrics_sha256": first["provenance"]["stable_metrics_sha256"],
        "verifier_sha256": file_sha256(Path(__file__)),
    }
    output = primary / "self_check.json"
    temporary = primary / ".self_check.json.tmp"
    temporary.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(output)
    print(json.dumps({"status": payload["status"], "checks": len(checks), "output": str(output)}))
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
