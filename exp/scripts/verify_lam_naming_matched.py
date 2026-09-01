#!/usr/bin/env python3
"""Verify deterministic reproduction and conservation for LAM matched Naming."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
DEFAULT_PRIMARY = REPO_ROOT / "exp/runtime/lam_naming_matched_v1"
DEFAULT_REPRO = REPO_ROOT / ".tmp/lam_naming_matched_repro_v1"


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def data(path: Path) -> bytes:
    resolved = contained(path)
    if resolved.is_symlink() or not resolved.is_file():
        raise RuntimeError(f"missing regular file: {resolved}")
    return resolved.read_bytes()


def digest(path: Path) -> str:
    return hashlib.sha256(data(path)).hexdigest()


def object_at(path: Path) -> dict[str, Any]:
    value = json.loads(data(path))
    if not isinstance(value, dict):
        raise RuntimeError(f"expected object: {path}")
    return value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--primary", type=Path, default=DEFAULT_PRIMARY)
    parser.add_argument("--repro", type=Path, default=DEFAULT_REPRO)
    args = parser.parse_args()
    primary = contained(args.primary)
    repro = contained(args.repro)
    first = object_at(primary / "summary.json")
    second = object_at(repro / "summary.json")
    first_stable = dict(first)
    second_stable = dict(second)
    first_stable.pop("generated_at_utc", None)
    second_stable.pop("generated_at_utc", None)
    records = [json.loads(line) for line in data(primary / "records.jsonl").decode().splitlines()]
    category_counts: dict[str, int] = {}
    for row in records:
        category_counts[row["canonical_category"]] = category_counts.get(row["canonical_category"], 0) + 1
    direct = first["direct_metrics"]
    total_parts = sum(row["renderable_part_count"] for row in records)
    total_named = sum(row["named_renderable_part_count"] for row in records)
    total_placeholders = sum(row["placeholder_renderable_part_count"] for row in records)
    checks = {
        "both_complete": first["status"] == second["status"] == "COMPLETE",
        "summary_equal_excluding_time": first_stable == second_stable,
        "records_byte_identical": data(primary / "records.jsonl") == data(repro / "records.jsonl"),
        "cohort_manifest_byte_identical": data(primary / "cohort_manifest.json") == data(repro / "cohort_manifest.json"),
        "report_byte_identical": data(primary / "report.md") == data(repro / "report.md"),
        "records_hash_matches_summary": digest(primary / "records.jsonl") == first["provenance"]["records_sha256"],
        "cohort_hash_matches_summary": digest(primary / "cohort_manifest.json") == first["provenance"]["cohort_manifest_sha256"],
        "five_by_seven_conserved": len(records) == 35 and len(category_counts) == 5 and set(category_counts.values()) == {7},
        "selection_ranks_conserved": all(
            sorted(row["selection_rank"] for row in records if row["canonical_category"] == category) == list(range(1, 8))
            for category in category_counts
        ),
        "selected_records_evaluable": all(row["parse_success"] and row["naming_evaluable"] for row in records),
        "part_count_conserved": total_parts == direct["total_renderable_parts"],
        "named_count_conserved": total_named == direct["total_named_renderable_parts"],
        "placeholder_count_conserved": total_placeholders == direct["placeholder_renderable_parts"] and total_parts == total_named + total_placeholders,
        "generation_not_run": first["generation_preflight"]["status"] == "NOT_RUN" and not first["network_accessed"] and not first["generated_code_executed"],
        "semantic_metrics_fail_closed": all(value is None for value in first["semantic_metrics"].values()),
        "comparison_not_mislabeled": "not same-prompt" in first["comparison_scope"] and "not same-seed" in first["comparison_scope"],
    }
    payload = {
        "protocol_id": "nano3d_lam_naming_matched_self_check_v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "category_counts": dict(sorted(category_counts.items())),
        "conservation": {
            "records": len(records),
            "renderable_parts": total_parts,
            "named_parts": total_named,
            "placeholder_parts": total_placeholders,
        },
        "primary_hashes": {
            name: digest(primary / name)
            for name in ("summary.json", "records.jsonl", "cohort_manifest.json", "report.md")
        },
        "repro_hashes": {
            name: digest(repro / name)
            for name in ("summary.json", "records.jsonl", "cohort_manifest.json", "report.md")
        },
        "verifier_sha256": digest(Path(__file__)),
    }
    output = primary / "self_check.json"
    temporary = primary / ".self_check.json.tmp"
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(output)
    print(json.dumps({"status": payload["status"], "checks": len(checks), "output": str(output)}))
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
