#!/usr/bin/env python3
"""Build non-semantic smoke fixtures and exercise the Naming judge aggregator.

These fixtures are deliberately mechanical. They test the 0%-coverage and
100%-coverage calculation paths, but they are not independent judgments and
must never be copied into the benchmark result table as semantic evidence.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any


EXP_ROOT = Path("/mnt/zsn/lyb/arti-skill/exp").resolve()
PACKET_ROOT = EXP_ROOT / "runtime/nano3d_naming_judge_packet_v1"
AUDIT_PATH = PACKET_ROOT / "audit_tasks.jsonl"
SMOKE_ROOT = PACKET_ROOT / "smoke_v23"
AGGREGATOR = EXP_ROOT / "scripts/aggregate_nano3d_naming_judges.py"
SOURCE_SUMMARY = EXP_ROOT / "runtime/nano3d_naming/summary.json"


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows), encoding="utf-8")


def incomplete_row(task: dict[str, Any]) -> dict[str, Any]:
    row = dict(task)
    row.update(
        {
            "judge_verdict": "uncertain",
            "judge_matched_role": None,
            "judge_instance_id": None,
            "judge_same_semantic_part_as": None,
            "judge_reason": "synthetic incomplete-path schema smoke; not a semantic judgment",
        }
    )
    return row


def source_synthetic_row(task: dict[str, Any]) -> dict[str, Any]:
    row = dict(task)
    role = task.get("assigned_source_role")
    if role is None:
        verdict = "extra_real_part"
        instance_id = "not_applicable"
    else:
        verdict = "spec_match"
        role_entry = next(item for item in task["reference_roles"] if item["role"] == role)
        instance_id = task["node_name"] if int(role_entry["min_count"]) > 1 else "not_applicable"
    row.update(
        {
            "judge_verdict": verdict,
            "judge_matched_role": role,
            "judge_instance_id": instance_id,
            "judge_same_semantic_part_as": "none",
            "judge_reason": "synthetic source-assignment formula smoke; not a semantic judgment",
        }
    )
    return row


def run_case(name: str, rows: list[dict[str, Any]]) -> dict[str, Any]:
    case_root = SMOKE_ROOT / name
    judges = []
    for judge_name in ("judge_a", "judge_b", "judge_c"):
        path = case_root / f"{judge_name}.jsonl"
        write_jsonl(path, rows)
        judges.append(path)
    output = case_root / "consensus.json"
    command = [
        sys.executable,
        str(AGGREGATOR),
        "--queue",
        str(AUDIT_PATH),
    ]
    for judge in judges:
        command.extend(["--judge", str(judge)])
    command.extend(["--output", str(output)])
    completed = subprocess.run(command, check=True, capture_output=True, text=True)
    (case_root / "aggregator_stdout.txt").write_text(completed.stdout, encoding="utf-8")
    return json.loads(output.read_text(encoding="utf-8"))


def run_rejection_case(name: str, rows: list[dict[str, Any]], expected_error: str) -> bool:
    case_root = SMOKE_ROOT / name
    judges = []
    for judge_name in ("judge_a", "judge_b", "judge_c"):
        path = case_root / f"{judge_name}.jsonl"
        write_jsonl(path, rows)
        judges.append(path)
    command = [sys.executable, str(AGGREGATOR), "--queue", str(AUDIT_PATH)]
    for judge in judges:
        command.extend(["--judge", str(judge)])
    command.extend(["--output", str(case_root / "must_not_exist.json")])
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    (case_root / "aggregator_stderr.txt").write_text(completed.stderr, encoding="utf-8")
    return completed.returncode != 0 and expected_error in completed.stderr


def same(left: float | None, right: float | None) -> bool:
    if left is None or right is None:
        return left is right
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1e-12)


def main() -> int:
    audit = read_jsonl(AUDIT_PATH)
    if len(audit) != 233:
        raise RuntimeError(f"expected 233 audit tasks, found {len(audit)}")

    incomplete = run_case("incomplete_all_uncertain", [incomplete_row(task) for task in audit])
    complete_rows = [source_synthetic_row(task) for task in audit]
    complete = run_case("complete_source_synthetic", complete_rows)
    one_fragment_rows = [dict(row) for row in complete_rows]
    first_asset = one_fragment_rows[0]["asset_id"]
    fragment_pair = [row for row in one_fragment_rows if row["asset_id"] == first_asset][:2]
    if len(fragment_pair) != 2:
        raise RuntimeError("could not construct one-fragment over-segmentation smoke")
    fragment_pair[0]["judge_same_semantic_part_as"] = fragment_pair[1]["node_name"]
    one_fragment = run_case("complete_one_fragment_synthetic", one_fragment_rows)
    missing_same_part_rows = [dict(row) for row in complete_rows]
    missing_same_part_rows[0]["judge_same_semantic_part_as"] = None
    missing_same_part_rejected = run_rejection_case(
        "invalid_missing_same_part",
        missing_same_part_rows,
        "real part requires judge_same_semantic_part_as",
    )
    source = json.loads(SOURCE_SUMMARY.read_text(encoding="utf-8"))

    incomplete_checks = {
        "coverage_is_zero": same(incomplete["consensus_coverage"], 0.0),
        "precision_is_null": incomplete["semantic_precision"] is None,
        "recall_is_null": incomplete["semantic_recall_micro"] is None,
        "functional_is_null": incomplete["functional_core_coverage_micro"] is None,
        "instance_is_null": incomplete["instance_discriminability"] is None,
        "overseg_is_null": incomplete["over_segmentation_rate_micro"] is None,
    }
    complete_checks = {
        "coverage_is_one": same(complete["consensus_coverage"], 1.0),
        "all_completion_gates_true": all(
            complete[key]
            for key in (
                "consensus_complete",
                "role_consensus_complete",
                "instance_consensus_complete",
                "same_part_consensus_complete",
            )
        ),
        "synthetic_precision_is_one": same(complete["semantic_precision"], 1.0),
        "recall_macro_matches_source_proxy": same(
            complete["semantic_recall_macro"], source["source_role_recall_macro"]
        ),
        "recall_micro_matches_source_proxy": same(
            complete["semantic_recall_micro"], source["source_role_recall_micro"]
        ),
        "functional_macro_matches_source_proxy": same(
            complete["functional_core_coverage_macro"], source["functional_core_coverage_macro"]
        ),
        "functional_micro_matches_source_proxy": same(
            complete["functional_core_coverage_micro"], source["functional_core_coverage_micro"]
        ),
        "instance_matches_source_proxy": same(
            complete["instance_discriminability"], source["instance_discriminability"]
        ),
        "synthetic_no_fragment_overseg_is_zero": same(
            complete["over_segmentation_rate_micro"], 0.0
        ),
    }
    one_fragment_checks = {
        "all_completion_gates_true": all(
            one_fragment[key]
            for key in (
                "consensus_complete",
                "role_consensus_complete",
                "instance_consensus_complete",
                "same_part_consensus_complete",
            )
        ),
        "one_excess_fragment": one_fragment["over_segmentation_excess_fragments"] == 1,
        "micro_rate_is_one_over_233": same(
            one_fragment["over_segmentation_rate_micro"], 1.0 / len(audit)
        ),
    }
    checks = {
        "incomplete": incomplete_checks,
        "complete_source_synthetic": complete_checks,
        "complete_one_fragment_synthetic": one_fragment_checks,
        "schema_rejection": {
            "missing_same_part_is_rejected": missing_same_part_rejected,
        },
    }
    if not all(value for group in checks.values() for value in group.values()):
        raise RuntimeError(f"smoke checks failed: {json.dumps(checks, indent=2)}")

    manifest = {
        "protocol": "nano3d_naming_judge_aggregator_smoke_v2.3",
        "warning": (
            "All judge rows are mechanical synthetic fixtures. They validate formulas only and are "
            "not independent semantic judgments or benchmark measurements."
        ),
        "task_count": len(audit),
        "checks": checks,
        "incomplete_formal_metrics": {
            key: incomplete[key]
            for key in (
                "semantic_precision",
                "semantic_recall_micro",
                "functional_core_coverage_micro",
                "instance_discriminability",
                "over_segmentation_rate_micro",
            )
        },
        "complete_source_synthetic_formula_outputs": {
            key: complete[key]
            for key in (
                "semantic_precision",
                "semantic_recall_macro",
                "semantic_recall_micro",
                "judge_validated_richness_mean",
                "judge_validated_richness_micro",
                "functional_core_coverage_macro",
                "functional_core_coverage_micro",
                "instance_discriminability",
                "over_segmentation_rate_macro",
                "over_segmentation_rate_micro",
            )
        },
        "complete_one_fragment_synthetic_formula_outputs": {
            key: one_fragment[key]
            for key in (
                "over_segmentation_excess_fragments",
                "over_segmentation_real_node_count",
                "over_segmentation_rate_macro",
                "over_segmentation_rate_micro",
            )
        },
    }
    SMOKE_ROOT.mkdir(parents=True, exist_ok=True)
    (SMOKE_ROOT / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2, ensure_ascii=False))
    print(f"output={SMOKE_ROOT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
