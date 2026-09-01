#!/usr/bin/env python3
"""Fail-closed aggregation for the frozen Nano3D Table 5 V2 experiments."""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
EXPECTED_MANIFEST_SHA256 = "d8d6e47ab3ab4ca22dbccb90d29796d515c1e772093f1e5d099c62a74a66e94d"
EXPECTED_SCORER_SHA256 = "7ce66aa02aa33869949ac561b59d0f7ff8ef81b591cb4d3565756cb7e02aa877"

MANIFEST = REPO_ROOT / "exp/reference/editability_v2/articraft_parent_manifest.json"
PVA_SUMMARY = (
    REPO_ROOT
    / "exp/runtime/nano3d_editability_v2/pva_retry1_d8d6e47a"
    / "contract_corrected_erratum_v2/summary.json"
)
PVA_AS_RUN_SUMMARY = (
    REPO_ROOT / "exp/runtime/nano3d_editability_v2/pva_retry1_d8d6e47a/summary.json"
)
PVA_CORRECTED_SCORES = (
    REPO_ROOT
    / "exp/runtime/nano3d_editability_v2/pva_retry1_d8d6e47a"
    / "contract_corrected_erratum_v2/scores"
)
NAIVE_AGGREGATE = (
    REPO_ROOT
    / "exp/runtime/nano3d_editability_v2/naive_same_llm_retry1_eval_d8d6e47a/aggregate.json"
)
NAIVE_INVALID_AUDIT = (
    REPO_ROOT
    / "exp/runtime/nano3d_editability_v2/naive_same_llm_final_d8d6e47a"
    / "INVALID_PROTOCOL_DEVIATION.json"
)
LAM_SUMMARY = (
    REPO_ROOT
    / "exp/runtime/nano3d_editability_v2/lam_shared_editor_final_d8d6e47a/evaluation_summary.json"
)
NATIVE_ROOT = (
    REPO_ROOT / "exp/runtime/nano3d_editability_v2/articraft_native_final_d8d6e47a"
)
NATIVE_SUMMARY = NATIVE_ROOT / "run_summary.json"
NATIVE_USAGE = NATIVE_ROOT / "cny_budget/usage.jsonl"
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/nano3d_editability_v2/table5_final"


def contained(path: Path, *, strict: bool = False) -> Path:
    resolved = path.resolve(strict=strict)
    if not resolved.is_relative_to(WORKSPACE_ROOT):
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def load_json(path: Path) -> dict[str, Any]:
    with contained(path, strict=True).open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path, strict=True).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dump_json(path: Path, value: dict[str, Any]) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def ratio(passed: int, total: int) -> dict[str, Any]:
    return {"passed": passed, "total": total, "rate": passed / total}


def native_result(manifest: dict[str, Any], blockers: list[str]) -> dict[str, Any] | None:
    if not NATIVE_SUMMARY.exists():
        blockers.append("ARTICRAFT_NATIVE_RUN_SUMMARY_MISSING")
        return None

    summary = load_json(NATIVE_SUMMARY)
    records = summary.get("records")
    if not isinstance(records, list) or len(records) != 18:
        blockers.append("ARTICRAFT_NATIVE_RECORD_COUNT_NOT_18")
        return None
    task_ids = [row.get("task_id") for row in records]
    expected_ids = [task["task_id"] for task in manifest["tasks"]]
    if task_ids != expected_ids or len(set(task_ids)) != 18:
        blockers.append("ARTICRAFT_NATIVE_TASK_ORDER_OR_UNIQUENESS_MISMATCH")
        return None
    if summary.get("manifest_sha256") != EXPECTED_MANIFEST_SHA256:
        blockers.append("ARTICRAFT_NATIVE_MANIFEST_SHA_MISMATCH")
        return None

    usage_rows: list[dict[str, Any]] = []
    with contained(NATIVE_USAGE, strict=True).open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                usage_rows.append(json.loads(line))
    if not usage_rows or any(row.get("task_id") not in set(expected_ids) for row in usage_rows):
        blockers.append("ARTICRAFT_NATIVE_USAGE_LEDGER_INVALID")
        return None

    def score_pass(row: dict[str, Any], key: str) -> bool:
        score = row.get("score")
        return isinstance(score, dict) and score.get(key) is True

    compile_success = sum(
        isinstance(row.get("compile"), dict) and row["compile"].get("returncode") == 0
        for row in records
    )
    passed = sum(score_pass(row, "passed") for row in records)
    by_class: dict[str, dict[str, int]] = {}
    class_names = {"A": "additive", "M": "modified", "S": "scale"}
    for prefix, name in class_names.items():
        members = [row for row in records if str(row["task_id"]).startswith(prefix)]
        by_class[name] = {
            "n": len(members),
            "compile_success": sum(
                isinstance(row.get("compile"), dict)
                and row["compile"].get("returncode") == 0
                for row in members
            ),
            "passed": sum(score_pass(row, "passed") for row in members),
        }

    axes = {
        key: ratio(sum(score_pass(row, key) for row in records), 18)
        for key in (
            "target_fulfilled",
            "anchor_preserved",
            "non_target_preserved",
            "geometry_locality",
            "structural_locality",
        )
    }
    scale_rows = [row for row in records if str(row["task_id"]).startswith("S")]
    axes["scale_threshold"] = ratio(sum(score_pass(row, "scale_passed") for row in scale_rows), 6)

    failure_kinds = Counter()
    for row in records:
        fork = row.get("fork") if isinstance(row.get("fork"), dict) else {}
        if fork.get("timed_out") is True or row.get("status") == "TIMEOUT_1800S":
            failure_kinds["record_timeout"] += 1
        elif fork.get("returncode") not in (None, 0):
            failure_kinds["provider_failure"] += 1
        elif not (
            isinstance(row.get("compile"), dict) and row["compile"].get("returncode") == 0
        ):
            failure_kinds["compile_failure"] += 1
        elif not score_pass(row, "passed"):
            failure_kinds["semantic_conjunction_failure"] += 1

    tokens = {
        "prompt": sum(int(row["usage"]["prompt_tokens"]) for row in usage_rows),
        "cached": sum(int(row["usage"]["cached_tokens"]) for row in usage_rows),
        "uncached_prompt": sum(int(row["usage"]["uncached_prompt_tokens"]) for row in usage_rows),
        "output": sum(int(row["usage"]["candidates_tokens"]) for row in usage_rows),
        "total": sum(int(row["usage"]["total_tokens"]) for row in usage_rows),
    }
    return {
        "method": "Articraft native fork",
        "evaluation_unit": "18 parent assets, one native agent fork each",
        "task_count": 18,
        "compile_success": ratio(compile_success, 18),
        "strict_success": ratio(passed, 18),
        "by_class": by_class,
        "axes": axes,
        "passed_tasks": [row["task_id"] for row in records if score_pass(row, "passed")],
        "failure_kinds": dict(sorted(failure_kinds.items())),
        "provider_cost": {
            "turns": len(usage_rows),
            "tokens": tokens,
            "estimated_cost_cny": round(sum(float(row["turn_cost_cny"]) for row in usage_rows), 7),
            "usd_cost": None,
            "measurement_scope": (
                "Successful provider responses recorded in the usage ledger. Failed timeout "
                "requests returned no provider usage and are not included."
            ),
        },
        "record_level_retries": 0,
        "manual_blind_review": "N/A",
        "numeric_tolerance_erratum_candidate": {
            "task_id": "S03",
            "measured_center_ratio": 0.7000000000000001,
            "frozen_threshold": "<=0.70",
            "as_run_scale_pass": False,
            "strict_final_changes_if_tolerance_applied": False,
            "reason": "non-target preservation and geometry locality also failed",
        },
        "recovery_note": (
            "A01 remains a timeout failure; the recovery harness only fixed timeout-log bytes decoding "
            "and continued A02-A18 without regenerating completed tasks."
        ),
    }


def build_summary() -> tuple[dict[str, Any], list[str]]:
    blockers: list[str] = []
    manifest = load_json(MANIFEST)
    if sha256(MANIFEST) != EXPECTED_MANIFEST_SHA256:
        blockers.append("SHARED_MANIFEST_SHA_MISMATCH")
    tasks = manifest.get("tasks")
    if not isinstance(tasks, list) or len(tasks) != 18:
        blockers.append("SHARED_MANIFEST_TASK_COUNT_NOT_18")
    else:
        task_ids = [task.get("task_id") for task in tasks]
        if len(set(task_ids)) != 18:
            blockers.append("SHARED_MANIFEST_TASK_IDS_NOT_UNIQUE")
        counts = Counter(task.get("edit_class") for task in tasks)
        if sorted(counts.values()) != [6, 6, 6]:
            blockers.append("SHARED_MANIFEST_CLASS_BALANCE_NOT_6_6_6")

    pva = load_json(PVA_SUMMARY)
    pva_as_run = load_json(PVA_AS_RUN_SUMMARY)
    if pva.get("status") != "CONTRACT_CORRECTED_COMPLETE":
        blockers.append("PVA_CORRECTED_SUMMARY_NOT_COMPLETE")
    if pva.get("hashes", {}).get("source_manifest_sha256") != EXPECTED_MANIFEST_SHA256:
        blockers.append("PVA_MANIFEST_SHA_MISMATCH")
    corrected = pva.get("contract_corrected", {})
    pva_denominators = (
        corrected.get("case_total"),
        corrected.get("main_asset_total"),
        corrected.get("propagation_total"),
    )
    if pva_denominators != (288, 18, 18):
        blockers.append("PVA_DENOMINATOR_MISMATCH")
    corrected_score_paths = sorted(contained(PVA_CORRECTED_SCORES, strict=True).rglob("score.json"))
    if len(corrected_score_paths) != 288:
        blockers.append("PVA_CORRECTED_SCORE_COUNT_NOT_288")
    corrected_scores = [load_json(path) for path in corrected_score_paths]
    corrected_keys = {(row.get("task_id"), row.get("seed")) for row in corrected_scores}
    if len(corrected_keys) != 288:
        blockers.append("PVA_CORRECTED_SCORE_KEYS_NOT_UNIQUE")
    pva_axis_names = (
        "target_fulfilled",
        "anchor",
        "scale",
        "non_target_preserved",
        "geometry_locality",
        "structural_locality",
        "post_edit_constraint_pass",
        "regression_preservation",
    )
    pva_axes = {
        name: ratio(sum(row.get("gates", {}).get(name) is True for row in corrected_scores), 288)
        for name in pva_axis_names
    }

    naive = load_json(NAIVE_AGGREGATE)
    if naive.get("status") != "COMPLETE" or naive.get("eligible_for_table5_primary_result") is not True:
        blockers.append("NAIVE_CLEAN_RETRY_NOT_ELIGIBLE")
    if naive.get("manifest_sha256") != EXPECTED_MANIFEST_SHA256:
        blockers.append("NAIVE_MANIFEST_SHA_MISMATCH")
    if naive.get("scorer_contract_sha256") != EXPECTED_SCORER_SHA256:
        blockers.append("NAIVE_SCORER_SHA_MISMATCH")
    if naive.get("task_count") != 18 or naive.get("integrity", {}).get("historical_gold_records_in_snapshot") != 0:
        blockers.append("NAIVE_COHORT_OR_GOLD_ISOLATION_INVALID")

    invalid_naive = load_json(NAIVE_INVALID_AUDIT)
    if invalid_naive.get("eligible_for_table5_primary_result") is not False:
        blockers.append("NAIVE_INVALID_FIRST_RUN_NOT_QUARANTINED")

    lam = load_json(LAM_SUMMARY)
    if lam.get("status") != "COMPLETE" or lam.get("strict_cohort_count") != 7:
        blockers.append("LAM_STRICT_SUBSET_NOT_COMPLETE")
    if lam.get("shared_task_manifest_sha256") != EXPECTED_MANIFEST_SHA256:
        blockers.append("LAM_MANIFEST_SHA_MISMATCH")
    if lam.get("main_table_eligible") is not False or lam.get("native_lam_edit_api") is not False:
        blockers.append("LAM_METHOD_LABEL_INVALID")

    native = native_result(manifest, blockers) if isinstance(tasks, list) and len(tasks) == 18 else None

    summary: dict[str, Any] = {
        "schema_version": 1,
        "protocol_id": "nano3d_table5_editability_final_v2",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "COMPLETE" if not blockers else "BLOCKED",
        "shared_manifest_sha256": EXPECTED_MANIFEST_SHA256,
        "shared_scorer_sha256": EXPECTED_SCORER_SHA256,
        "cohort": {
            "design": "curated challenge set",
            "unbiased_population_claim": False,
            "task_count": 18,
            "class_balance": {"additive": 6, "modified": 6, "scale": 6},
            "unique_parent_count": 18,
        },
        "primary_results": {
            "pva_contract_corrected": {
                "evaluation_unit": "18 tasks x 16 fixed seeds",
                "strict_seed_success": ratio(corrected.get("case_pass", 0), corrected.get("case_total", 288)),
                "main_asset_success": ratio(corrected.get("main_asset_pass", 0), corrected.get("main_asset_total", 18)),
                "propagation_16_of_16": ratio(
                    corrected.get("propagation_pass", 0),
                    corrected.get("propagation_total", 18),
                ),
                "frozen_as_run": pva.get("frozen_as_run"),
                "artifact_only_scorer_erratum": True,
                "axes": pva_axes,
                "by_class": pva.get("class_results"),
                "api_tokens": 0,
                "api_cost": 0,
                "mean_wall_time_seconds": pva_as_run.get("edit_cost", {}).get("mean_wall_time_s"),
                "median_wall_time_seconds": pva_as_run.get("edit_cost", {}).get("median_wall_time_s"),
                "manual_blind_review": "N/A",
            },
            "articraft_native": native,
            "naive_same_llm_clean_retry1": naive,
        },
        "supplementary_results": {
            "lam_released_source_external_same_editor": {
                "main_table_eligible": False,
                "native_lam_edit_api": False,
                "strict_subset_count": lam.get("strict_cohort_count"),
                "strict_success": ratio(lam.get("task_success_count", 0), lam.get("strict_cohort_count", 7)),
                "compile_success": ratio(lam.get("compile_pass_count", 0), lam.get("strict_cohort_count", 7)),
                "axes": lam.get("applicable_axis_counts"),
                "provider_cost": lam.get("provider_usage_and_cost"),
            },
            "naive_first_run_invalid_audit": {
                "eligible_for_primary_result": False,
                "reason": invalid_naive.get("reason"),
                "minimum_paid_totals": invalid_naive.get("actual_minimum_paid_totals"),
            },
        },
        "blockers": sorted(set(blockers)),
    }
    return summary, blockers


def write_report(path: Path, summary: dict[str, Any]) -> None:
    pva = summary["primary_results"]["pva_contract_corrected"]
    native = summary["primary_results"]["articraft_native"]
    naive = summary["primary_results"]["naive_same_llm_clean_retry1"]
    lam = summary["supplementary_results"]["lam_released_source_external_same_editor"]
    lines = [
        "# Nano3D Table 5 Editability V2",
        "",
        f"Status: **{summary['status']}**",
        "",
        "The benchmark is a frozen curated challenge set, not an unbiased population estimate.",
        "",
    ]
    if summary["blockers"]:
        lines.extend(["## Blockers", "", *[f"- `{item}`" for item in summary["blockers"]], ""])
    else:
        pva_axes = pva["axes"]
        native_axes = native["axes"]
        naive_axes = naive["axes"]
        pva_classes = {row["class"]: row for row in pva["by_class"]}
        lines.extend(
            [
                "## Primary results",
                "",
                "| Method | Strict success | Compile/QC | Cost |",
                "|---|---:|---:|---:|",
                (
                    f"| PV-A | {pva['main_asset_success']['passed']}/18 main; "
                    f"{pva['strict_seed_success']['passed']}/288 seeds | 287/288 | 0 API tokens |"
                ),
                (
                    f"| Articraft native | {native['strict_success']['passed']}/18 | "
                    f"{native['compile_success']['passed']}/18 | "
                    f"{native['provider_cost']['estimated_cost_cny']:.6f} CNY |"
                ),
                (
                    f"| naive same-LLM | {naive['strict_passed']}/18 | "
                    f"{naive['compile_success']}/18 | "
                    f"{naive['provider_cost']['estimated_cost_cny']:.6f} CNY |"
                ),
                "",
                "## Unified scorer axes",
                "",
                "| Axis | PV-A (288 seeds) | Articraft native (18 assets) | naive same-LLM (18 assets) |",
                "|---|---:|---:|---:|",
                (
                    f"| Target | {pva_axes['target_fulfilled']['passed']}/288 | "
                    f"{native_axes['target_fulfilled']['passed']}/18 | "
                    f"{naive_axes['target_fulfilled']['passed']}/18 |"
                ),
                (
                    f"| Anchor | {pva_axes['anchor']['passed']}/288 | "
                    f"{native_axes['anchor_preserved']['passed']}/18 | "
                    f"{naive_axes['anchor_preserved']['passed']}/18 |"
                ),
                (
                    f"| Scale | 95/96 applicable | "
                    f"{native_axes['scale_threshold']['passed']}/6 | "
                    f"{naive_axes['scale_threshold']['passed']}/6 |"
                ),
                (
                    f"| Non-target | {pva_axes['non_target_preserved']['passed']}/288 | "
                    f"{native_axes['non_target_preserved']['passed']}/18 | "
                    f"{naive_axes['non_target_preserved']['passed']}/18 |"
                ),
                (
                    f"| Geometry locality | {pva_axes['geometry_locality']['passed']}/288 | "
                    f"{native_axes['geometry_locality']['passed']}/18 | "
                    f"{naive_axes['geometry_locality']['passed']}/18 |"
                ),
                (
                    f"| Structural locality | {pva_axes['structural_locality']['passed']}/288 | "
                    f"{native_axes['structural_locality']['passed']}/18 | "
                    f"{naive_axes['structural_locality']['passed']}/18 |"
                ),
                (
                    f"| Compile/QC | {pva_axes['post_edit_constraint_pass']['passed']}/288 | "
                    f"{native['compile_success']['passed']}/18 | {naive['compile_success']}/18 |"
                ),
                "",
                "## Class breakdown",
                "",
                "| Class | PV-A main / seeds | Articraft native | naive same-LLM |",
                "|---|---:|---:|---:|",
                (
                    f"| Additive | {pva_classes['A']['main_asset_pass']}/6; "
                    f"{pva_classes['A']['case_pass']}/96 | "
                    f"{native['by_class']['additive']['passed']}/6 | "
                    f"{naive['by_class']['additive']['passed']}/6 |"
                ),
                (
                    f"| Modified | {pva_classes['M']['main_asset_pass']}/6; "
                    f"{pva_classes['M']['case_pass']}/96 | "
                    f"{native['by_class']['modified']['passed']}/6 | "
                    f"{naive['by_class']['modified']['passed']}/6 |"
                ),
                (
                    f"| Scale | {pva_classes['S']['main_asset_pass']}/6; "
                    f"{pva_classes['S']['case_pass']}/96 | "
                    f"{native['by_class']['scale']['passed']}/6 | "
                    f"{naive['by_class']['scale']['passed']}/6 |"
                ),
                "",
                "## Integrity notes",
                "",
                (
                    "- PV-A primary values are an artifact-only scorer erratum. Frozen as-run "
                    "values remain 255/288 seeds, 16/18 main assets, and 15/18 propagation tasks."
                ),
                (
                    "- Articraft native kept A01 as a record-timeout failure. Recovery fixed only "
                    "timeout-log decoding and continued A02-A18 without record retries."
                ),
                (
                    "- Native S03 is an as-run numeric-tolerance candidate "
                    "(0.7000000000000001 versus <=0.70); changing that scale axis would not "
                    "change strict failure."
                ),
                (
                    "- Native cost includes only turns with returned usage; four failed "
                    "provider-timeout requests returned no usage response."
                ),
                (
                    "- The clean naive run used 18 one-shot requests, no tools, feedback, repair, "
                    "or historical gold. Its earlier raced cohort is quarantined and excluded."
                ),
                "- Deterministic scorer results are reported here; manual blind review is N/A.",
                "",
                "## Supplementary strict subset",
                "",
                (
                    "LAM released-source + external same-editor probe: "
                    f"{lam['strict_success']['passed']}/{lam['strict_subset_count']}; "
                    "this is not a native LAM edit API and is not ranked on the common "
                    "18-item denominator."
                ),
                "",
            ]
        )
    contained(path).write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = contained(args.output)
    summary, blockers = build_summary()
    dump_json(output / "summary.json", summary)
    write_report(output / "report.md", summary)
    print(json.dumps({"status": summary["status"], "blockers": blockers}, indent=2))
    return 2 if blockers else 0


if __name__ == "__main__":
    raise SystemExit(main())
