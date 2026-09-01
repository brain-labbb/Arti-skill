#!/usr/bin/env python3
"""Canonicalize and summarize the adjudicated real Articraft Table 4 v2 run."""

from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
EXP_ROOT = WORKSPACE / "arti-skill" / "exp"
RUN_ROOT = EXP_ROOT / "runtime" / "table4_constraints_v2" / "articraft"
STORAGE_ROOT = RUN_ROOT / "workspace_data"
MATERIALIZATION_ROOT = STORAGE_ROOT / "data" / "cache" / "record_materialization"
CANONICALIZER = EXP_ROOT / "scripts" / "canonicalize_table4_artifact.py"
MANIFEST = RUN_ROOT / "artifact_manifest.jsonl"
PROMPT_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
FORCED_FAILURES = {
    "T4C001": "official_cli_timeout_1800s",
    "T4C005": "model_attempt_consumed_before_boundary_incident",
    "T4C006": "model_attempt_consumed_before_boundary_incident",
    "T4C009": "model_attempt_consumed_and_stopped_at_boundary_incident",
    "T4C010": "model_attempt_consumed_and_stopped_at_boundary_incident",
}


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    return hashlib.sha256(contained(path).read_bytes()).hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(contained(path).read_text(encoding="utf-8"))


def log_metrics(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"turns_started": 0, "compile_passes": 0, "compile_failures": 0}
    text = contained(path).read_text(encoding="utf-8", errors="replace")
    turns = [int(value) for value in re.findall(r"Starting turn (\d+)/100", text)]
    costs = re.findall(r"Total cost: \$([0-9.]+)", text)
    completed_call_costs = re.findall(r"^\s*llm[^\n]*\$([0-9.]+)", text, flags=re.MULTILINE)
    token_matches = re.findall(r"Total tokens: (\{[^\n]+\})", text)
    result: dict[str, Any] = {
        "turns_started": max(turns, default=0),
        "compile_passes": len(re.findall(r"compile ✓", text)),
        "compile_failures": len(re.findall(r"compile ✗", text)),
        "reported_cost_usd": float(costs[-1]) if costs else None,
        "observed_completed_call_cost_usd": sum(float(value) for value in completed_call_costs),
    }
    if token_matches:
        result["reported_tokens_text"] = token_matches[-1]
    return result


def verify_no_symlinks(root: Path) -> dict[str, Any]:
    root = contained(root)
    checked = 0
    links: list[str] = []
    for current, dirnames, filenames in os.walk(root, followlinks=False):
        current_path = contained(Path(current))
        for name in [*dirnames, *filenames]:
            path = current_path / name
            checked += 1
            if path.is_symlink():
                links.append(str(path))
            else:
                contained(path)
    if links:
        raise RuntimeError(f"isolated storage unexpectedly contains symlinks: {links}")
    return {"root": str(root), "checked_entries": checked, "symlink_count": 0}


def record_cost(record_dir: Path) -> tuple[dict[str, Any] | None, Path | None]:
    paths = sorted(contained(record_dir).glob("revisions/*/cost.json"))
    if len(paths) != 1:
        return None, None
    return load_json(paths[0]), paths[0]


def main() -> int:
    pre = verify_no_symlinks(STORAGE_ROOT)
    rows: list[dict[str, Any]] = []
    task_summaries: list[dict[str, Any]] = []
    for index in range(1, 19):
        task_id = f"T4C{index:03d}"
        task_dir = contained(RUN_ROOT / task_id)
        run_path = task_dir / "run.json"
        command_path = task_dir / "command.json"
        prompt_path = task_dir / "prompt.txt"
        log_path = task_dir / "generation.log"
        run = load_json(run_path) if run_path.is_file() else None
        metrics = log_metrics(log_path)
        forced_reason = FORCED_FAILURES.get(task_id)
        eligible = bool(run and run.get("status") == "success" and not forced_reason)
        row: dict[str, Any] = {
            "task_id": task_id,
            "status": "failed",
            "prompt": str(prompt_path),
            "command": str(command_path),
            "generation_log": str(log_path),
            "run_record": str(run_path) if run_path.is_file() else None,
            "repair_budget": 0,
            **metrics,
        }
        if run:
            row.update({
                "elapsed_seconds": run.get("elapsed_seconds"),
                "timed_out": run.get("timed_out"),
                "model_attempt": run.get("model_attempt"),
                "infra_launch_attempt": run.get("infra_launch_attempt"),
            })
        if not eligible:
            if forced_reason:
                row["failure"] = forced_reason
            elif run and run.get("timed_out"):
                row["failure"] = f"official_cli_timeout_{int(round(run.get('elapsed_seconds', 1800)))}s"
            elif run:
                row["failure"] = f"official_cli_{run.get('status', 'failed')}"
            else:
                row["failure"] = "no_terminal_run_record"
            rows.append(row)
            task_summaries.append(row.copy())
            continue

        record_dir = contained(Path(str(run["attributable_record_dir"])))
        record_name = record_dir.name
        source = contained(MATERIALIZATION_ROOT / record_name / "model.urdf")
        if not source.is_file() or source.is_symlink():
            row["failure"] = "missing_isolated_final_urdf"
            rows.append(row)
            task_summaries.append(row.copy())
            continue
        canonical_dir = contained(RUN_ROOT / "canonical" / task_id)
        canonical_dir.mkdir(parents=True, exist_ok=True)
        canonical_log = canonical_dir / "canonicalize.log"
        command = [
            sys.executable,
            str(CANONICALIZER),
            "--input",
            str(source),
            "--artifact-type",
            "urdf",
            "--unit-scale-to-m",
            "1.0",
            "--output-dir",
            str(canonical_dir),
        ]
        result = subprocess.run(command, text=True, capture_output=True, check=False)
        canonical_log.write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode != 0:
            row.update({
                "failure": "canonicalize_failed",
                "canonicalize_returncode": result.returncode,
                "source_urdf": str(source),
                "canonicalize_log": str(canonical_log),
            })
            rows.append(row)
            task_summaries.append(row.copy())
            continue
        cost, cost_path = record_cost(record_dir)
        row.update({
            "status": "success",
            "record_dir": str(record_dir),
            "source_urdf": str(source),
            "source_urdf_sha256": sha256(source),
            "canonical_dir": str(canonical_dir),
            "canonicalize_log": str(canonical_log),
            "cost_record": str(cost_path) if cost_path else None,
            "cost_usd": cost.get("all_in_total", {}).get("costs_usd", {}).get("total") if cost else None,
            "total_tokens": cost.get("all_in_total", {}).get("tokens", {}).get("total_tokens") if cost else None,
            "elapsed_seconds": run.get("elapsed_seconds"),
            "model_attempt": run.get("model_attempt"),
            "infra_launch_attempt": run.get("infra_launch_attempt"),
        })
        rows.append(row)
        task_summaries.append(row.copy())

    MANIFEST.write_text(
        "".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    post = verify_no_symlinks(STORAGE_ROOT)
    successes = [row for row in rows if row["status"] == "success"]
    failures = [row for row in rows if row["status"] != "success"]
    failure_observed_cost = sum(
        float(row.get("reported_cost_usd") or row.get("observed_completed_call_cost_usd") or 0.0)
        for row in failures
    )
    smoke_metrics = log_metrics(RUN_ROOT / "smoke" / "generation.log")
    smoke_cost = float(
        smoke_metrics.get("reported_cost_usd")
        or smoke_metrics.get("observed_completed_call_cost_usd")
        or 0.0
    )
    provenance = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "benchmark": "table4_constraints_v2",
        "method": "articraft",
        "panel": "structured_main",
        "prompt_manifest": str(EXP_ROOT / "reference" / "table4_constraints_v2" / "prompts.jsonl"),
        "prompt_manifest_sha256": PROMPT_SHA256,
        "official_repo": str(WORKSPACE / "arti-skill" / "articraft_data"),
        "official_repo_commit": "06cd75fbc9e90fae33f127b494d13c35090356be",
        "official_repo_remote_origin": "/mnt/zsn/lyb/articraft_data.git",
        "official_entry": "uv run articraft generate",
        "official_package_version": "0.1.0",
        "uv_version": "0.11.16",
        "sdk_package": "sdk",
        "sdk_version": "workspace/0.1.0",
        "storage_repo_root": str(STORAGE_ROOT),
        "storage_boundary_pre": pre,
        "storage_boundary_post": post,
        "boundary_incident": str(RUN_ROOT / "boundary_incident.json"),
        "model": "qwen3.6-plus",
        "provider": "dashscope",
        "temperature": None,
        "temperature_policy": "unset_provider_native_default",
        "thinking_level": "high",
        "max_turns": 100,
        "max_cost_usd_per_task": 5.0,
        "timeout_seconds_per_task": 1800,
        "repair_budget": 0,
        "method_native_agent_iteration": True,
        "credential_policy": "original workspace .env loaded into child process only; no values recorded",
        "smoke": {
            "dashscope_success_run": str(RUN_ROOT / "smoke" / "run.json"),
            "dashscope_success_log": str(RUN_ROOT / "smoke" / "generation.log"),
            "initial_openai_401_run": str(RUN_ROOT / "smoke_openai_failed" / "run.json"),
            "initial_openai_401_log": str(RUN_ROOT / "smoke_openai_failed" / "generation.log"),
        },
        "artifact_manifest": str(MANIFEST),
        "successful_artifacts": len(successes),
        "failed_tasks": [row["task_id"] for row in rows if row["status"] != "success"],
        "successful_total_cost_usd": sum(float(row.get("cost_usd") or 0.0) for row in successes),
        "successful_total_tokens": sum(int(row.get("total_tokens") or 0) for row in successes),
        "failed_tasks_observed_cost_usd": failure_observed_cost,
        "formal_tasks_observed_cost_usd": (
            sum(float(row.get("cost_usd") or 0.0) for row in successes) + failure_observed_cost
        ),
        "smoke_cost_usd": smoke_cost,
        "experiment_observed_cost_including_smoke_usd": (
            sum(float(row.get("cost_usd") or 0.0) for row in successes)
            + failure_observed_cost
            + smoke_cost
        ),
        "failure_cost_policy": (
            "Use official final total when present; otherwise sum displayed costs for completed LLM calls. "
            "Interrupted in-flight calls may be absent, so timeout/stopped costs are lower-bound observations."
        ),
        "tasks": task_summaries,
    }
    scorer_files = ("records.json", "summary.json", "report.md")
    scorer_run1 = RUN_ROOT / "scorer_run1"
    scorer_run2 = RUN_ROOT / "scorer_run2"
    if all((scorer_run1 / name).is_file() and (scorer_run2 / name).is_file() for name in scorer_files):
        hashes1 = {name: sha256(scorer_run1 / name) for name in scorer_files}
        hashes2 = {name: sha256(scorer_run2 / name) for name in scorer_files}
        byte_identical = hashes1 == hashes2
        score_dir = contained(RUN_ROOT / "score")
        score_dir.mkdir(parents=True, exist_ok=True)
        for name in scorer_files:
            shutil.copy2(scorer_run1 / name, score_dir / name)
        provenance["scorer_reproducibility"] = {
            "run1": str(scorer_run1),
            "run2": str(scorer_run2),
            "byte_identical": byte_identical,
            "sha256": hashes1,
            "final_score_dir": str(score_dir),
        }
        score_summary = load_json(score_dir / "summary.json")
        report_lines = [
            "# Table 4 Constraints v2: real Articraft", "",
            f"Prompt manifest SHA-256: `{PROMPT_SHA256}`.", "",
            "## Result", "",
            f"- Final attributable artifacts: {len(successes)}/18",
            f"- Constraints passed: {score_summary['passed']}/{score_summary['constraints']}",
            f"- Coverage: {score_summary['coverage']:.6f}",
            f"- Satisfaction: {score_summary['satisfaction']:.6f}",
            f"- Conditional accuracy: {score_summary['conditional_accuracy']:.6f}",
            f"- Count pass: {score_summary['count_pass']}",
            f"- Numeric pass: {score_summary['numeric_pass']}",
            f"- Scorer double-run byte-identical: {str(byte_identical).lower()}", "",
            "## Runtime", "",
            f"- Successful-artifact exact cost: ${provenance['successful_total_cost_usd']:.6f}",
            f"- Failed-task observed cost: ${failure_observed_cost:.6f}",
            f"- Formal-task observed cost: ${provenance['formal_tasks_observed_cost_usd']:.6f}",
            f"- Smoke cost: ${smoke_cost:.6f}",
            f"- Experiment observed cost including smoke: "
            f"${provenance['experiment_observed_cost_including_smoke_usd']:.6f}",
            "- Timeout/stopped costs are lower-bound observations from completed-call logs.",
            f"- Successful-artifact tokens: {provenance['successful_total_tokens']}", "",
            "## Tasks", "",
            "| Task | Status | Turns | Elapsed (s) | Cost (USD) | Failure |",
            "|---|---|---:|---:|---:|---|",
        ]
        for row in rows:
            elapsed = row.get("elapsed_seconds")
            cost = row.get("cost_usd")
            if cost is None:
                cost = row.get("reported_cost_usd") or row.get("observed_completed_call_cost_usd")
            elapsed_text = f"{elapsed:.2f}" if isinstance(elapsed, (int, float)) else "-"
            cost_text = f"{cost:.6f}" if isinstance(cost, (int, float)) else "-"
            report_lines.append(
                f"| {row['task_id']} | {row['status']} | {row.get('turns_started', 0)} | "
                f"{elapsed_text} | {cost_text} | {row.get('failure', '-')} |"
            )
        (RUN_ROOT / "report.md").write_text("\n".join(report_lines) + "\n", encoding="utf-8")
    (RUN_ROOT / "provenance.json").write_text(
        json.dumps(provenance, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "manifest": str(MANIFEST),
        "successes": len(successes),
        "failures": 18 - len(successes),
        "successful_total_cost_usd": provenance["successful_total_cost_usd"],
        "successful_total_tokens": provenance["successful_total_tokens"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
