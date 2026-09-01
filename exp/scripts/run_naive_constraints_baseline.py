#!/usr/bin/env python3
"""Fail-closed Table 4 Constraints preflight for naive same-LLM runs.

The T2 authoring runs are useful evidence of executable template generation,
but Table 4A additionally requires output-independent constraint specs and
final GLBs that a compatible scorer can measure. This runner audits those
requirements without deriving targets from generated code, URDF, or QC data.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
EXP_ROOT = REPO_ROOT / "exp"
PROTOCOL_PATH = EXP_ROOT / "reference" / "paper_constraints_protocol_v1.json"
RUNS_ROOT = EXP_ROOT / "runtime" / "t2_formal_v1" / "authoring" / "runs"
COMMON_INPUT_ROOT = EXP_ROOT / "reference" / "paper_constraints"
DEFAULT_OUTPUT = (
    EXP_ROOT / "runtime" / "table4_constraints_baselines" / "naive_same_llm"
)


def contained(path: Path, *, strict: bool = False) -> Path:
    resolved = path.resolve(strict=strict)
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path is outside authorized workspace: {resolved}")
    return resolved


def relative(path: Path) -> str:
    return str(contained(path).relative_to(WORKSPACE_ROOT))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path, strict=True).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    with contained(path, strict=True).open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def regular_files(root: Path) -> list[Path]:
    root = contained(root)
    if root.is_symlink() or not root.is_dir():
        return []
    return sorted(
        path
        for path in root.rglob("*")
        if not path.is_symlink() and path.is_file()
    )


def nonempty_packet_constraint_fields(packet: dict[str, Any]) -> list[str]:
    names = {
        "constraints",
        "constraint_spec",
        "frozen_spec",
        "spec",
        "measure_recipe",
        "measure_recipes",
        "tolerance",
        "tolerances",
    }
    return sorted(name for name in names if packet.get(name) not in (None, "", [], {}))


def audit_run(run_dir: Path) -> dict[str, Any]:
    run_dir = contained(run_dir, strict=True)
    packet_path = run_dir / "packet.json"
    result_path = run_dir / "run_result.json"
    distribution_path = run_dir / "evaluations" / "distribution_final" / "summary.json"
    packet = load_json(packet_path)
    result = load_json(result_path)
    distribution = load_json(distribution_path)
    files = regular_files(run_dir)
    specs = [path for path in files if path.suffix.lower() in {".yaml", ".yml"}]
    glbs = [path for path in files if path.suffix.lower() == ".glb"]
    final_urdfs = [
        path
        for path in files
        if path.name == "model.urdf"
        and "evaluations/distribution_final/cases/" in relative(path)
    ]
    output_programs = [
        path
        for path in files
        if path.suffix.lower() == ".py" and path.parent == run_dir / "output"
    ]
    constraint_fields = nonempty_packet_constraint_fields(packet)
    method = packet.get("method") if isinstance(packet.get("method"), dict) else {}
    return {
        "run_key": packet.get("run_key"),
        "task_slug": packet.get("task_slug"),
        "method_id": method.get("method_id"),
        "model": packet.get("model"),
        "packet": relative(packet_path),
        "packet_sha256": sha256(packet_path),
        "packet_keys": sorted(packet),
        "nonempty_constraint_fields": constraint_fields,
        "output_independent_frozen_spec_available": bool(specs or constraint_fields),
        "frozen_spec_files": [relative(path) for path in specs],
        "run_result": relative(result_path),
        "run_result_sha256": sha256(result_path),
        "valid_experiment_run": result.get("valid_experiment_run") is True,
        "t2_artifact_saved": result.get("artifact_saved") is True,
        "t2_final_success": result.get("final_success") is True,
        "t2_seed_full_qc_pass": (result.get("distribution") or {}).get(
            "seed_compile_full_qc_pass"
        ),
        "t2_seed_total": (result.get("distribution") or {}).get("seed_total"),
        "distribution_summary": relative(distribution_path),
        "distribution_summary_sha256": sha256(distribution_path),
        "output_programs": [relative(path) for path in output_programs],
        "distribution_final_urdf_count": len(final_urdfs),
        "final_glb_count": len(glbs),
        "final_glbs": [relative(path) for path in glbs],
        "table4_measurable_artifact_available": bool(glbs),
    }


def common_input_audit() -> dict[str, Any]:
    root = contained(COMMON_INPUT_ROOT)
    prompt_path = root / "prompt_manifest.jsonl"
    prompt_count = 0
    prompt_errors: list[str] = []
    if not prompt_path.is_symlink() and prompt_path.is_file():
        for line_number, line in enumerate(
            prompt_path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            if not line.strip():
                continue
            try:
                json.loads(line)
                prompt_count += 1
            except json.JSONDecodeError as error:
                prompt_errors.append(f"line {line_number}: {error.msg}")
    specs = [
        path
        for path in regular_files(root / "specs")
        if path.suffix.lower() in {".yaml", ".yml"}
    ]
    glbs = [
        path for path in regular_files(root / "glbs") if path.suffix.lower() == ".glb"
    ]
    scorer_names = {
        "score_constraints.py",
        "score_paper_constraints.py",
        "constraint_scorer.py",
    }
    scorers = [path for path in regular_files(root) if path.name in scorer_names]
    return {
        "root": relative(root),
        "prompt_manifest": relative(prompt_path),
        "prompt_manifest_exists": prompt_path.is_file() and not prompt_path.is_symlink(),
        "valid_prompt_rows": prompt_count,
        "prompt_json_errors": prompt_errors,
        "frozen_spec_count": len(specs),
        "frozen_specs": [relative(path) for path in specs],
        "final_glb_count": len(glbs),
        "final_glbs": [relative(path) for path in glbs],
        "compatible_scorer_count": len(scorers),
        "compatible_scorers": [relative(path) for path in scorers],
    }


def build_summary() -> dict[str, Any]:
    protocol = load_json(PROTOCOL_PATH)
    runs_root = contained(RUNS_ROOT, strict=True)
    run_dirs = sorted(
        path
        for path in runs_root.glob("*__naive_same_llm__r1")
        if not path.is_symlink() and path.is_dir()
    )
    runs = [audit_run(path) for path in run_dirs]
    common = common_input_audit()
    required_items = int(protocol["required_item_count"])
    local_specs = sum(bool(run["output_independent_frozen_spec_available"]) for run in runs)
    local_glbs = sum(int(run["final_glb_count"]) for run in runs)

    requirements = {
        "original_constrained_prompt_manifest_18_items": (
            common["valid_prompt_rows"] == required_items
            and not common["prompt_json_errors"]
        ),
        "complete_naive_same_llm_cohort_18_items": len(runs) == required_items,
        "output_independent_frozen_specs_18_items": (
            common["frozen_spec_count"] == required_items or local_specs == required_items
        ),
        "attributable_final_glbs_18_items": (
            common["final_glb_count"] == required_items or local_glbs == required_items
        ),
        "paper_measure_recipes_and_scorer": common["compatible_scorer_count"] > 0,
    }
    blocker_map = {
        "original_constrained_prompt_manifest_18_items": "MISSING_FROZEN_CONSTRAINED_PROMPTS",
        "complete_naive_same_llm_cohort_18_items": "INCOMPLETE_NAIVE_COHORT",
        "output_independent_frozen_specs_18_items": "MISSING_OUTPUT_INDEPENDENT_FROZEN_SPECS",
        "attributable_final_glbs_18_items": "MISSING_NAIVE_FINAL_GLBS",
        "paper_measure_recipes_and_scorer": "MISSING_PAPER_COMPATIBLE_SCORER",
    }
    blockers = [blocker_map[name] for name, passed in requirements.items() if not passed]
    status = "BLOCKED" if blockers else "READY_NOT_RUN"
    return {
        "schema_version": 1,
        "protocol_id": "nano3d_table4_constraints_naive_same_llm_preflight_v1",
        "baseline": "naive same-LLM",
        "table": "Table 4A: Paper-aligned constraint satisfaction",
        "status": status,
        "evidence_class": "PREFLIGHT_ONLY",
        "protocol": {
            "source": relative(PROTOCOL_PATH),
            "id": protocol["protocol"],
            "sha256": sha256(PROTOCOL_PATH),
            "required_items": required_items,
            "paper_reference_constraint_count": protocol["required_constraint_count"],
            "paper_reference_count_constraint_count": protocol[
                "required_count_constraint_count"
            ],
            "paper_reference_numeric_constraint_count": protocol[
                "required_numeric_constraint_count"
            ],
            "reference_counts_are_not_local_measurements": True,
        },
        "safety": {
            "workspace_root": str(WORKSPACE_ROOT),
            "network_accessed": False,
            "model_or_api_called": False,
            "gpu_job_started": False,
            "existing_job_touched": False,
            "paper_values_reused_as_local_metrics": False,
            "targets_inferred_from_outputs": False,
            "t2_reliability_reused_as_constraints": False,
        },
        "existing_t2_run_audit": {
            "root": relative(runs_root),
            "run_count": len(runs),
            "valid_experiment_runs": sum(run["valid_experiment_run"] for run in runs),
            "t2_final_success_runs": sum(run["t2_final_success"] for run in runs),
            "t2_distribution_seed_total": sum(
                int(run["t2_seed_total"] or 0) for run in runs
            ),
            "t2_distribution_full_qc_pass": sum(
                int(run["t2_seed_full_qc_pass"] or 0) for run in runs
            ),
            "output_independent_frozen_spec_runs": local_specs,
            "final_glb_count": local_glbs,
            "distribution_final_urdf_count": sum(
                int(run["distribution_final_urdf_count"]) for run in runs
            ),
            "runs": runs,
            "reliability_evidence_table4_compatible": False,
            "reason": (
                "Compile/full-QC and generated URDFs do not supply prompt-stated "
                "numeric/count targets, independent measurement recipes, tolerances, "
                "or final GLB scene nodes."
            ),
        },
        "common_input_audit": common,
        "requirements": requirements,
        "blocker_codes": blockers,
        "local_evaluation": {
            "status": "NOT_RUN" if blockers else "READY_NOT_RUN",
            "display_value": "N/R",
            "items": 0,
            "constraints_scored": 0,
            "metrics": {
                "constraints": None,
                "measurable": None,
                "passed": None,
                "coverage": None,
                "satisfaction": None,
                "conditional_accuracy": None,
                "count_pass": None,
            },
            "reason": (
                "Fail-closed: the existing runs lack a complete pre-output frozen "
                "constraint benchmark and protocol-compatible final artifacts/scorer."
            ),
        },
        "paper_reference_policy": {
            "paper_values_imported": False,
            "paper_reported_52_24_11_used_as_local_constraints": False,
            "reason": (
                "Paper-reported values are comparison context only; this preflight "
                "does not transcribe them as local measurements."
            ),
        },
        "minimum_unblock_inputs": [
            "The complete frozen 18-item constrained prompt manifest.",
            "One output-independent spec.yaml per item, frozen before generation.",
            "All measurement recipes, comparators, semantic anchors, and tolerances.",
            "Eighteen attributable final naive same-LLM GLBs linked by item ID and hash.",
            "A locally runnable paper-compatible GLB constraint scorer.",
        ],
        "run_command": "python exp/scripts/run_naive_constraints_baseline.py",
    }


def write_report(summary: dict[str, Any], path: Path) -> None:
    audit = summary["existing_t2_run_audit"]
    common = summary["common_input_audit"]
    blockers = ", ".join(summary["blocker_codes"])
    lines = [
        "# naive same-LLM Constraints baseline",
        "",
        f"Status: **{summary['status']}** (`PREFLIGHT_ONLY`; Table 4A **N/R**)",
        "",
        "No local Table 4A score was produced. Paper-reported 52/24/11 values were not",
        "imported, generated outputs were not used to invent targets, and T2 reliability",
        "was not relabeled as constraint satisfaction.",
        "",
        "| Evidence | Available | Required |",
        "|---|---:|---:|",
        f"| Existing attributable naive same-LLM runs | {audit['run_count']} | {summary['protocol']['required_items']} |",
        f"| Original constrained prompt rows | {common['valid_prompt_rows']} | {summary['protocol']['required_items']} |",
        f"| Output-independent frozen specs | {audit['output_independent_frozen_spec_runs']} | {summary['protocol']['required_items']} |",
        f"| Final protocol-measurable GLBs | {audit['final_glb_count']} | {summary['protocol']['required_items']} |",
        f"| Compatible constraint scorer | {common['compatible_scorer_count']} | 1 |",
        "",
        "## Existing-run audit",
        "",
        f"The six T2 runs are real authoring evidence: {audit['valid_experiment_runs']}/{audit['run_count']} are valid experiment runs, {audit['t2_final_success_runs']}/{audit['run_count']} reached T2 final success, and {audit['t2_distribution_full_qc_pass']}/{audit['t2_distribution_seed_total']} distribution seeds passed compile + Full-QC. They contain {audit['distribution_final_urdf_count']} final-distribution URDF packages, but zero frozen constraint specs and zero GLBs.",
        "",
        "Those results establish executable/template reliability only. They do not define",
        "prompt-stated exact counts or numeric dimensions, semantic GLB anchors, measurement",
        "recipes, comparators, or tolerances, so no Table 4 constraint is measurable.",
        "",
        f"Blockers: `{blockers}`.",
        "",
    ]
    contained(path.parent).mkdir(parents=True, exist_ok=True)
    contained(path).write_text("\n".join(lines), encoding="utf-8")


def validate_summary(summary: dict[str, Any]) -> None:
    assert summary["status"] == "BLOCKED", summary["status"]
    assert summary["evidence_class"] == "PREFLIGHT_ONLY"
    assert summary["local_evaluation"]["display_value"] == "N/R"
    assert summary["local_evaluation"]["constraints_scored"] == 0
    assert all(
        value is None for value in summary["local_evaluation"]["metrics"].values()
    )
    assert summary["paper_reference_policy"]["paper_values_imported"] is False
    assert summary["safety"]["t2_reliability_reused_as_constraints"] is False
    assert summary["existing_t2_run_audit"]["run_count"] > 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--assert-blocked", action="store_true")
    args = parser.parse_args()
    output_dir = contained(args.output_dir)
    summary = build_summary()
    if args.assert_blocked:
        validate_summary(summary)
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    write_report(summary, output_dir / "report.md")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 2 if summary["status"] == "BLOCKED" else 0


if __name__ == "__main__":
    raise SystemExit(main())
