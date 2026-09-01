#!/usr/bin/env python3
"""Execute the formal stratified unseen-category T2 authoring experiment."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import re
import shutil
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
PREP = EXP_ROOT / "runtime/t2_formal_v1/preparation"
DEFAULT_OUT = EXP_ROOT / "runtime/t2_formal_v1/authoring"
MANIFEST = PREP / "formal_source_manifest.json"
DESIGN_REVIEW = PREP / "design_logs/summary.json"
EVALUATOR = EXP_ROOT / "scripts/evaluate_t2_generated_template.py"
CODEX = Path("/mnt/zsn/miniconda3/bin/codex")
MODEL = "gpt-5.6-sol"
REPEATS = (2026080701, 2026080702, 2026080703)
FORMAL_SELECTED_SLUGS = (
    # Two categories per frozen complexity stratum.  The choices cover
    # single-revolute, multi-revolute, revolute+selector, and prismatic motion.
    "flip_phone",
    "clamp_meter_with_hinged_jaw_and_rotary_selector",
    "garden_gate",
    "air_purifier_with_filter_door",
    "folding_kick_scooter",
    "extension_ladder",
)
METHODS = (
    {
        "method_id": "naive_same_llm",
        "display": "Naive same-LLM",
        "source_map": False,
        "design": False,
    },
    {
        "method_id": "without_source_map",
        "display": "w/o Source Map",
        "source_map": False,
        "design": True,
    },
    {
        "method_id": "without_template_design",
        "display": "w/o TemplateDesign / slot abstraction",
        "source_map": True,
        "design": False,
    },
    {
        "method_id": "full_ours",
        "display": "Full Ours",
        "source_map": True,
        "design": True,
    },
)
RECORD_RE = re.compile(r"\brec_[A-Za-z0-9_-]+\b")


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def usage(events: Path) -> dict[str, int]:
    total = {
        "input_tokens": 0,
        "cached_input_tokens": 0,
        "cache_write_input_tokens": 0,
        "output_tokens": 0,
        "reasoning_output_tokens": 0,
    }
    if not events.is_file():
        return total
    for line in events.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get("type") != "turn.completed":
            continue
        for key in total:
            total[key] += int((row.get("usage") or {}).get(key) or 0)
    return total


def selected_record_dirs(task: dict[str, Any]) -> list[Path]:
    return [
        (PROJECT_ROOT / row["source_record"]).resolve()
        for row in task["records"]
    ]


def authoring_seed(slug: str, method: str, repeat: int) -> int:
    raw = f"{slug}:{method}:{repeat}".encode("utf-8")
    return int.from_bytes(hashlib.sha256(raw).digest()[:4], "big") & 0x7FFFFFFF


def prompt(
    task: dict[str, Any],
    method: dict[str, Any],
    repeat_index: int,
    run_root: Path,
    template: Path,
    record_dirs: list[Path],
) -> str:
    evidence = [f"- raw source record: `{path}`" for path in record_dirs]
    if method["source_map"]:
        evidence.append(f"- frozen SourceMap: `{run_root / 'input/source_map.md'}`")
    if method["design"]:
        evidence.append(f"- frozen TemplateDesign: `{run_root / 'input/template_design.json'}`")
    return f"""
You are the authoring model in a controlled formal experiment. Author one new,
self-contained Articraft procedural template for the genuinely unseen category
`{task['slug']}` (complexity stratum: {task['complexity']}). This is fixed repeat
index {repeat_index + 1}, deterministic prompt seed {authoring_seed(task['slug'], method['method_id'], repeat_index)}.

Read the common contract and priors:
- `{TEMPLATE_ROOT / 'articraft_template_authoring/AUTHORING.md'}`
- `{TEMPLATE_ROOT / 'articraft_template_authoring/MECHANICAL_PRIORS.md'}`
- `{TEMPLATE_ROOT / 'articraft_template_authoring/VISUAL_DIVERSITY_MODEL.md'}`
- the SDK public surface under `{TEMPLATE_ROOT / 'sdk'}`

The only category evidence allowed in this arm is:
{chr(10).join(evidence)}

Do not read any existing file under `{TEMPLATE_ROOT / 'agent/templates'}`, any
nonlisted source record, evidence withheld from this arm, another authoring run,
or prior evaluator output. Do not use web/network sources. The differences among
arms are intentional; do not reconstruct withheld SourceMap/TemplateDesign files.

Write only `{template}`. The module must be self-contained: no imports of source
records, snapshots, generated outputs, or other templates at runtime. It must expose
exactly one public `build_*`, exactly one public `run_*_tests`, `config_from_seed`,
`TEMPLATE_DOMAIN`, `TEMPLATE_CORNERS`, and a precise `__all__`. Use metric units,
real named parts, a connected hierarchy, mechanically solved joints/interfaces,
finite limits, deterministic seeded configs, source-grounded structural diversity,
category-anchor checks, mesh readiness, isolated-part and sampled-motion collision
checks. Avoid compatibility gates by adaptation or inseparable structural families.

You may run author-visible smoke tests while writing, using only the common SDK and
your output. Finish only after the file imports, several seeds build, and its own
tests execute. Do not modify any other file. Report the path and smoke counts only.
""".strip()


def repair_prompt(
    task: dict[str, Any],
    method: dict[str, Any],
    run_root: Path,
    template: Path,
    feedback: Path,
    repair_turn: int,
) -> str:
    return f"""
This is automated repair turn {repair_turn}/3 for formal run
`{task['slug']} / {method['method_id']}`. Read the existing generated template
`{template}` and normalized hidden-evaluator feedback `{feedback}`. The same evidence
allowlist and prohibitions in `{run_root / 'packet.json'}` remain binding. Do not read
raw hidden evaluator case files, other runs, existing templates, or withheld evidence.

Edit only `{template}`. Fix the general root cause, preserve already passing behavior,
and keep the template self-contained and distributional. You may run author-visible
smoke tests. Do not tailor to individual hidden seeds or hard-code evaluator messages.
Report the repaired path and concise smoke counts only.
""".strip()


def codex_turn(prompt_text: str, cwd: Path, prefix: Path, timeout: float) -> dict[str, Any]:
    started_wall = time.monotonic()
    completed = subprocess.run(
        [
            str(CODEX),
            "exec",
            "--json",
            "--model",
            MODEL,
            "-c",
            "model_reasoning_effort='high'",
            "--dangerously-bypass-approvals-and-sandbox",
            "--skip-git-repo-check",
            prompt_text,
        ],
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )
    events = prefix.with_suffix(".events.jsonl")
    stderr = prefix.with_suffix(".stderr.txt")
    events.parent.mkdir(parents=True, exist_ok=True)
    events.write_text(completed.stdout, encoding="utf-8")
    stderr.write_text(completed.stderr, encoding="utf-8")
    return {
        "process_exit_code": completed.returncode,
        "wall_time_s": time.monotonic() - started_wall,
        "events": str(events),
        "stderr": str(stderr),
        "usage": usage(events),
    }


def normalized_feedback(summary: dict[str, Any], output: Path) -> None:
    unique: list[dict[str, Any]] = []
    seen: set[str] = set()
    for row in summary.get("records", []):
        if row.get("verdict") == "pass" and row.get("artifact_saved"):
            continue
        error = str(row.get("error") or "unknown evaluator failure")
        # Absolute runtime paths and seed identities are unnecessary repair
        # leakage; the message and final traceback tail are sufficient.
        error = re.sub(r"/mnt/zsn/lyb/arti-skill/exp/runtime/[^\s:'\"]+", "<runtime>", error)
        key = error[:1000]
        if key in seen:
            continue
        seen.add(key)
        trace = str(row.get("traceback") or "")[-2500:]
        trace = re.sub(r"/mnt/zsn/lyb/arti-skill/exp/runtime/[^\s:'\"]+", "<runtime>", trace)
        unique.append({"error": error[:2000], "traceback_tail": trace})
        if len(unique) == 10:
            break
    dump_json(
        output,
        {
            "schema_version": 1,
            "seed_pass": summary.get("seed_compile_and_full_qc_pass"),
            "seed_total": summary.get("seed_total"),
            "artifact_saved_any": summary.get("artifact_saved_any"),
            "unique_failure_classes": unique,
            "policy": "normalized root-cause feedback; hidden case identities withheld",
        },
    )


def evaluate(template: Path, out: Path, seeds: str, corners: bool, timeout: float) -> dict[str, Any]:
    command = [
        "python",
        str(EVALUATOR),
        "--template",
        str(template),
        "--out",
        str(out),
        "--seeds",
        seeds,
        "--workers",
        "4",
        "--timeout",
        "600",
    ]
    if corners:
        command.append("--include-corners")
    try:
        completed = subprocess.run(
            command,
            cwd=PROJECT_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {
            "all_seeds_pass": False,
            "all_corners_pass": False,
            "artifact_saved_any": False,
            "error": f"evaluator_timeout({timeout}s)",
            "records": [],
        }
    summary_path = out / "summary.json"
    if summary_path.is_file():
        return json.loads(summary_path.read_text(encoding="utf-8"))
    return {
        "all_seeds_pass": False,
        "all_corners_pass": False,
        "artifact_saved_any": False,
        "error": f"evaluator_exit_{completed.returncode}: {completed.stderr[-4000:]}",
        "records": [],
    }


def audit_trace(events_paths: list[Path], forbidden: list[str]) -> list[dict[str, str]]:
    violations: list[dict[str, str]] = []
    for events in events_paths:
        if not events.is_file():
            continue
        for line in events.read_text(encoding="utf-8", errors="replace").splitlines():
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            item = row.get("item") or {}
            command = str(item.get("command") or "")
            for path in forbidden:
                if path and path in command:
                    violations.append(
                        {"events": str(events), "forbidden": path, "command": command[:2000]}
                    )
    return violations


def execute_run(
    task: dict[str, Any],
    method: dict[str, Any],
    repeat_index: int,
    repeat_seed: int,
    out: Path,
    model_timeout: float,
) -> dict[str, Any]:
    run_key = f"{task['slug']}__{method['method_id']}__r{repeat_index + 1}"
    run_root = out / "runs" / run_key
    final_record = run_root / "run_result.json"
    if final_record.is_file():
        return json.loads(final_record.read_text(encoding="utf-8"))
    run_root.mkdir(parents=True, exist_ok=True)
    source_map = PREP / "source_maps" / f"{task['slug']}.md"
    design = PREP / "designs" / f"{task['slug']}.json"
    if method["source_map"]:
        (run_root / "input").mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_map, run_root / "input/source_map.md")
    if method["design"]:
        (run_root / "input").mkdir(parents=True, exist_ok=True)
        shutil.copy2(design, run_root / "input/template_design.json")
    record_dirs = selected_record_dirs(task)
    template = run_root / "output" / f"{task['slug']}.py"
    template.parent.mkdir(parents=True, exist_ok=True)
    forbidden = [str(TEMPLATE_ROOT / "agent/templates")]
    if not method["source_map"]:
        forbidden.extend([str(source_map), str(PREP / "source_maps")])
    if not method["design"]:
        forbidden.extend([str(design), str(PREP / "designs")])
    packet = {
        "schema_version": 1,
        "run_key": run_key,
        "task_slug": task["slug"],
        "complexity": task["complexity"],
        "method": method,
        "repeat_index": repeat_index,
        "repeat_seed": repeat_seed,
        "authoring_seed": authoring_seed(task["slug"], method["method_id"], repeat_index),
        "model": MODEL,
        "reasoning_effort": "high",
        "record_dirs": [str(path) for path in record_dirs],
        "record_hashes": {
            path.name: sha256(path / "model.py")
            for path in record_dirs
            if (path / "model.py").is_file()
        },
        "source_map_exposed": method["source_map"],
        "template_design_exposed": method["design"],
        "max_repair_turns": 3,
        "forbidden_paths": forbidden,
    }
    dump_json(run_root / "packet.json", packet)
    started = datetime.now(timezone.utc).isoformat()
    turns: list[dict[str, Any]] = []
    trace_paths: list[Path] = []
    initial = codex_turn(
        prompt(task, method, repeat_index, run_root, template, record_dirs),
        run_root,
        run_root / "traces/authoring_turn_0",
        model_timeout,
    )
    turns.append(initial)
    trace_paths.append(Path(initial["events"]))
    if template.is_file():
        shutil.copy2(template, run_root / "first_shot_template.py")
    first = evaluate(template, run_root / "evaluations/first_shot", "0-15", False, 14400.0) if template.is_file() else {
        "all_seeds_pass": False,
        "artifact_saved_any": False,
        "records": [],
        "error": "authoring model did not create template",
    }
    current = first
    repair_turns = 0
    for repair_turn in range(1, 4):
        if current.get("all_seeds_pass"):
            break
        feedback = run_root / f"repair_feedback_{repair_turn}.json"
        normalized_feedback(current, feedback)
        turn = codex_turn(
            repair_prompt(task, method, run_root, template, feedback, repair_turn),
            run_root,
            run_root / f"traces/authoring_turn_{repair_turn}",
            model_timeout,
        )
        turns.append(turn)
        trace_paths.append(Path(turn["events"]))
        repair_turns = repair_turn
        if template.is_file():
            shutil.copy2(template, run_root / f"repair_{repair_turn}_template.py")
            current = evaluate(
                template,
                run_root / f"evaluations/repair_{repair_turn}",
                "0-15",
                False,
                14400.0,
            )
        else:
            current = {"all_seeds_pass": False, "artifact_saved_any": False, "records": []}
    distribution = evaluate(
        template,
        run_root / "evaluations/distribution_final",
        "0-35",
        True,
        28800.0,
    ) if template.is_file() else {
        "all_seeds_pass": False,
        "all_corners_pass": False,
        "seed_compile_and_full_qc_pass": 0,
        "seed_total": 36,
        "corner_pass": 0,
        "corner_count": 0,
        "records": [],
    }
    first_passed_seeds = {
        int(row["case_value"])
        for row in first.get("records", [])
        if row.get("case_kind") == "seed"
        and row.get("verdict") == "pass"
        and row.get("artifact_saved")
    }
    final_passed_seeds = {
        int(row["case_value"])
        for row in distribution.get("records", [])
        if row.get("case_kind") == "seed"
        and row.get("verdict") == "pass"
        and row.get("artifact_saved")
    }
    violations = audit_trace(trace_paths, forbidden)
    total_usage = {key: sum(turn["usage"][key] for turn in turns) for key in turns[0]["usage"]}
    result = {
        "schema_version": 1,
        "run_key": run_key,
        "task_slug": task["slug"],
        "complexity": task["complexity"],
        "method_id": method["method_id"],
        "repeat_index": repeat_index,
        "repeat_seed": repeat_seed,
        "started_at": started,
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "model": MODEL,
        "executable": any(
            row.get("verdict") == "pass" for row in first.get("records", [])
        ),
        "artifact_saved": bool(first.get("artifact_saved_any")),
        "first_shot": bool(first.get("all_seeds_pass")),
        "final_success": bool(current.get("all_seeds_pass")),
        "repair_turns": repair_turns,
        "human_intervention": False,
        "protocol_violations": violations,
        "valid_experiment_run": not violations,
        "authoring_wall_time_s": sum(turn["wall_time_s"] for turn in turns),
        "token_usage": total_usage,
        "authoring_cost_usd": None,
        "cost_note": "internal Codex CLI exposes tokens but no billed USD amount",
        "template_sha256": sha256(template) if template.is_file() else None,
        "distribution": {
            "seed_compile_full_qc_pass": distribution.get("seed_compile_and_full_qc_pass", 0),
            "seed_total": distribution.get("seed_total", 36),
            "all_36": bool(distribution.get("all_seeds_pass")),
            "corner_pass": distribution.get("corner_pass", 0),
            "corner_total": distribution.get("corner_count", 0),
            "all_corners": bool(distribution.get("all_corners_pass")),
            "regression_retained": len(first_passed_seeds & final_passed_seeds),
            "regression_expected": len(first_passed_seeds),
            "regression_retention_rate": (
                len(first_passed_seeds & final_passed_seeds) / len(first_passed_seeds)
                if first_passed_seeds
                else None
            ),
        },
        "turns": turns,
    }
    dump_json(final_record, result)
    return result


def aggregate(
    out: Path,
    records: list[dict[str, Any]],
    *,
    task_count: int,
    repeat_count: int,
) -> dict[str, Any]:
    methods = []
    for method in METHODS:
        rows = [
            row
            for row in records
            if row["method_id"] == method["method_id"] and row["valid_experiment_run"]
        ]
        total = len(rows)
        methods.append(
            {
                "method_id": method["method_id"],
                "display": method["display"],
                "valid_runs": total,
                "expected_runs": task_count * repeat_count,
                "executable_rate": sum(row["executable"] for row in rows) / total if total else None,
                "artifact_saved_rate": sum(row["artifact_saved"] for row in rows) / total if total else None,
                "first_shot_rate": sum(row["first_shot"] for row in rows) / total if total else None,
                "final_success_rate": sum(row["final_success"] for row in rows) / total if total else None,
                "mean_repair_turns": sum(row["repair_turns"] for row in rows) / total if total else None,
                "human_intervention_rate": sum(row["human_intervention"] for row in rows) / total if total else None,
                "mean_authoring_wall_time_s": sum(row["authoring_wall_time_s"] for row in rows) / total if total else None,
                "input_tokens": sum(row["token_usage"]["input_tokens"] for row in rows),
                "output_tokens": sum(row["token_usage"]["output_tokens"] for row in rows),
                "seed_compile_full_qc": sum(row["distribution"]["seed_compile_full_qc_pass"] for row in rows),
                "seed_total": sum(row["distribution"]["seed_total"] for row in rows),
                "all_36_templates": sum(row["distribution"]["all_36"] for row in rows),
                "corner_pass": sum(row["distribution"]["corner_pass"] for row in rows),
                "corner_total": sum(row["distribution"]["corner_total"] for row in rows),
                "regression_retained": sum(row["distribution"]["regression_retained"] for row in rows),
                "regression_expected": sum(row["distribution"]["regression_expected"] for row in rows),
            }
        )
    summary = {
        "schema_version": 1,
        "protocol": "t2_stratified_unseen_authoring_v1",
        "model": MODEL,
        "reasoning_effort": "high",
        "task_count": task_count,
        "method_count": 4,
        "repeat_count": repeat_count,
        "expected_runs": task_count * len(METHODS) * repeat_count,
        "completed_runs": len(records),
        "valid_runs": sum(row["valid_experiment_run"] for row in records),
        "methods": methods,
    }
    dump_json(out / "records.json", records)
    dump_json(out / "summary.json", summary)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--model-timeout", type=float, default=10800.0)
    parser.add_argument(
        "--task-slugs",
        default=",".join(FORMAL_SELECTED_SLUGS),
        help="comma-separated frozen unseen categories; default is the six-category stratified cohort",
    )
    parser.add_argument(
        "--repeat-count",
        type=int,
        default=1,
        choices=(1, 2, 3),
        help="number of fixed prompt-seed repeats; one is the formal scoped default",
    )
    args = parser.parse_args()
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    all_tasks = manifest["tasks"]
    if len(all_tasks) != 12:
        raise RuntimeError(f"expected 12 tasks, found {len(all_tasks)}")
    requested = tuple(item.strip() for item in args.task_slugs.split(",") if item.strip())
    by_slug = {task["slug"]: task for task in all_tasks}
    missing = sorted(set(requested) - set(by_slug))
    if missing:
        raise RuntimeError(f"unknown frozen task slugs: {missing}")
    tasks = [by_slug[slug] for slug in requested]
    if not tasks:
        raise RuntimeError("at least one frozen task slug is required")
    design_review_records: dict[str, dict[str, Any]] = {}
    for task in tasks:
        path = PREP / "design_logs" / f"{task['slug']}_run.json"
        if not path.is_file():
            raise RuntimeError(f"formal TemplateDesign review is incomplete: {task['slug']}")
        record = json.loads(path.read_text(encoding="utf-8"))
        if not record.get("passed"):
            raise RuntimeError(f"formal TemplateDesign review failed: {task['slug']}")
        design_review_records[task["slug"]] = {
            "path": str(path),
            "sha256": sha256(path),
            "design_sha256": record.get("design_sha256"),
        }
    repeat_seeds = REPEATS[: args.repeat_count]
    out = args.out.resolve()
    out.relative_to(EXP_ROOT.resolve())
    jobs = [
        (task, method, repeat_index, repeat_seed)
        for repeat_index, repeat_seed in enumerate(repeat_seeds)
        for task in tasks
        for method in METHODS
    ]
    jobs.sort(
        key=lambda job: hashlib.sha256(
            f"{job[0]['slug']}:{job[1]['method_id']}:{job[2]}".encode()
        ).hexdigest()
    )
    dump_json(
        out / "experiment_manifest.json",
        {
            "schema_version": 1,
            "protocol": "t2_stratified_unseen_authoring_v1",
            "created_at": datetime.now(timezone.utc).isoformat(),
            "model": MODEL,
            "reasoning_effort": "high",
            "repeat_seeds": repeat_seeds,
            "selected_task_slugs": requested,
            "selection_note": "two categories per frozen simple/medium/complex stratum; selected before authoring execution",
            "methods": METHODS,
            "source_manifest": str(MANIFEST),
            "source_manifest_sha256": sha256(MANIFEST),
            "design_review_records": design_review_records,
            "job_order": [f"{t['slug']}__{m['method_id']}__r{i + 1}" for t, m, i, _ in jobs],
        },
    )
    records: list[dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = {
            pool.submit(
                execute_run,
                task,
                method,
                repeat_index,
                repeat_seed,
                out,
                args.model_timeout,
            ): (task, method, repeat_index)
            for task, method, repeat_index, repeat_seed in jobs
        }
        for index, future in enumerate(concurrent.futures.as_completed(futures), 1):
            result = future.result()
            records.append(result)
            print(
                f"T2 {index}/{len(jobs)} {result['run_key']} "
                f"first={result['first_shot']} final={result['final_success']} "
                f"all36={result['distribution']['all_36']}",
                flush=True,
            )
            aggregate(
                out,
                sorted(records, key=lambda row: row["run_key"]),
                task_count=len(tasks),
                repeat_count=len(repeat_seeds),
            )
    records.sort(key=lambda row: row["run_key"])
    print(
        json.dumps(
            aggregate(
                out,
                records,
                task_count=len(tasks),
                repeat_count=len(repeat_seeds),
            ),
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
