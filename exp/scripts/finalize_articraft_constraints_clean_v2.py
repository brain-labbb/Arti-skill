#!/usr/bin/env python3
"""Officially compile, canonicalize, and report the Articraft clean-v2 cohort."""

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
REPO_ROOT = WORKSPACE / "arti-skill"
ARTICRAFT_ROOT = REPO_ROOT / "articraft_data"
EXP_ROOT = REPO_ROOT / "exp"
RUN_ROOT = EXP_ROOT / "runtime" / "table4_constraints_v2" / "articraft_clean_v2"
STORAGE_ROOT = RUN_ROOT / "workspace_data"
MATERIALIZATION_ROOT = STORAGE_ROOT / "data" / "cache" / "record_materialization"
CANONICALIZER = EXP_ROOT / "scripts" / "canonicalize_table4_artifact.py"
MANIFEST = RUN_ROOT / "artifact_manifest.jsonl"
PROMPT_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
OFFICIAL_COMMIT = "06cd75fbc9e90fae33f127b494d13c35090356be"


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def contained_under(path: Path, root: Path) -> Path:
    resolved = contained(path)
    canonical_root = contained(root)
    if resolved != canonical_root and canonical_root not in resolved.parents:
        raise ValueError(f"path escapes clean cohort root: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    return hashlib.sha256(contained(path).read_bytes()).hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(contained(path).read_text(encoding="utf-8"))


def verify_tree_boundary(root: Path) -> dict[str, Any]:
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
        raise RuntimeError(f"clean cohort contains symlinks: {links}")
    return {
        "root": str(root),
        "checked_entries": checked,
        "symlink_count": 0,
        "all_realpaths_in_workspace": True,
    }


def log_metrics(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"turns_started": 0, "compile_passes": 0, "compile_failures": 0}
    text = contained(path).read_text(encoding="utf-8", errors="replace")
    turns = [int(value) for value in re.findall(r"Starting turn (\d+)/100", text)]
    costs = re.findall(r"Total cost: \$([0-9.]+)", text)
    completed = re.findall(r"^\s*llm[^\n]*\$([0-9.]+)", text, flags=re.MULTILINE)
    token_matches = re.findall(r"Total tokens: (\{[^\n]+\})", text)
    result: dict[str, Any] = {
        "turns_started": max(turns, default=0),
        "compile_passes": len(re.findall(r"compile ✓", text)),
        "compile_failures": len(re.findall(r"compile ✗", text)),
        "reported_cost_usd": float(costs[-1]) if costs else None,
        "observed_completed_call_cost_usd": sum(float(value) for value in completed),
    }
    if token_matches:
        result["reported_tokens_text"] = token_matches[-1]
    return result


def record_cost(record_dir: Path) -> tuple[dict[str, Any] | None, Path | None]:
    paths = sorted(contained(record_dir).glob("revisions/*/cost.json"))
    if len(paths) != 1:
        return None, None
    return load_json(paths[0]), paths[0]


def run_official_compile(task_id: str, record_dir: Path) -> tuple[int, Path]:
    log_path = contained(RUN_ROOT / task_id / "official_compile.log")
    command = [
        str(ARTICRAFT_ROOT / ".venv" / "bin" / "python"),
        "-m",
        "cli.main",
        "compile",
        "--repo-root",
        str(STORAGE_ROOT),
        str(contained(record_dir)),
        "--target",
        "full",
        "--validate",
    ]
    result = subprocess.run(
        command,
        cwd=contained(ARTICRAFT_ROOT),
        env=offline_child_environment(),
        text=True,
        capture_output=True,
        check=False,
    )
    log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
    verify_tree_boundary(RUN_ROOT)
    return result.returncode, log_path


def offline_child_environment() -> dict[str, str]:
    env = {
        key: value
        for key, value in os.environ.items()
        if not any(marker in key.upper() for marker in ("API_KEY", "TOKEN", "SECRET", "PASSWORD"))
    }
    process_root = contained(RUN_ROOT / "_process_env")
    env.update(
        {
            "HOME": str(process_root / "home"),
            "TMPDIR": str(process_root / "tmp"),
            "XDG_CACHE_HOME": str(process_root / "cache"),
            "XDG_CONFIG_HOME": str(process_root / "config"),
            "XDG_DATA_HOME": str(process_root / "data"),
            "PYTHONDONTWRITEBYTECODE": "1",
        }
    )
    return env


def main() -> int:
    prompt_manifest = contained(EXP_ROOT / "reference" / "table4_constraints_v2" / "prompts.jsonl")
    if sha256(prompt_manifest) != PROMPT_SHA256:
        raise RuntimeError("frozen prompt hash mismatch")
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=contained(ARTICRAFT_ROOT),
        text=True,
        capture_output=True,
        check=True,
    ).stdout.strip()
    if commit != OFFICIAL_COMMIT:
        raise RuntimeError(f"official checkout commit mismatch: {commit}")
    pre = verify_tree_boundary(RUN_ROOT)
    rows: list[dict[str, Any]] = []
    for index in range(1, 19):
        task_id = f"T4C{index:03d}"
        task_dir = contained(RUN_ROOT / task_id)
        run_path = task_dir / "run.json"
        command_path = task_dir / "command.json"
        prompt_path = task_dir / "prompt.txt"
        log_path = task_dir / "generation.log"
        run = load_json(run_path) if run_path.is_file() else None
        metrics = log_metrics(log_path)
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
            row.update(
                {
                    "elapsed_seconds": run.get("elapsed_seconds"),
                    "timed_out": run.get("timed_out"),
                    "model_attempt": run.get("model_attempt"),
                    "infra_launch_attempt": run.get("infra_launch_attempt"),
                }
            )
        if not run or run.get("status") != "success":
            if run and run.get("timed_out"):
                row["failure"] = "official_runner_timeout_1800s"
            elif run:
                row["failure"] = f"official_runner_exit_{run.get('exit_code')}"
            else:
                row["failure"] = "no_terminal_run_record"
            rows.append(row)
            continue

        record_dir = contained_under(Path(str(run["attributable_record_dir"])), STORAGE_ROOT)
        compile_rc, compile_log = run_official_compile(task_id, record_dir)
        row["official_compile_log"] = str(compile_log)
        row["official_compile_returncode"] = compile_rc
        if compile_rc != 0:
            row["failure"] = "official_compile_failed"
            rows.append(row)
            continue

        source = contained(MATERIALIZATION_ROOT / record_dir.name / "model.urdf")
        if not source.is_file() or source.is_symlink():
            row["failure"] = "missing_clean_final_urdf"
            rows.append(row)
            continue
        canonical_dir = contained(RUN_ROOT / "canonical" / task_id)
        canonical_dir.mkdir(parents=True, exist_ok=False)
        canonical_log = canonical_dir / "canonicalize.log"
        result = subprocess.run(
            [
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
            ],
            text=True,
            capture_output=True,
            check=False,
        )
        canonical_log.write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode != 0:
            row.update(
                {
                    "failure": "canonicalize_failed",
                    "canonicalize_returncode": result.returncode,
                    "source_urdf": str(source),
                    "canonicalize_log": str(canonical_log),
                }
            )
            rows.append(row)
            continue
        cost, cost_path = record_cost(record_dir)
        row.update(
            {
                "status": "success",
                "record_dir": str(record_dir),
                "source_urdf": str(source),
                "source_urdf_sha256": sha256(source),
                "canonical_dir": str(canonical_dir),
                "canonicalize_log": str(canonical_log),
                "cost_record": str(cost_path) if cost_path else None,
                "cost_usd": cost.get("all_in_total", {}).get("costs_usd", {}).get("total") if cost else None,
                "total_tokens": cost.get("all_in_total", {}).get("tokens", {}).get("total_tokens") if cost else None,
            }
        )
        rows.append(row)

    MANIFEST.write_text(
        "".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    post = verify_tree_boundary(RUN_ROOT)
    successes = [row for row in rows if row["status"] == "success"]
    failures = [row for row in rows if row["status"] != "success"]
    failed_observed_cost = sum(
        float(row.get("reported_cost_usd") or row.get("observed_completed_call_cost_usd") or 0.0)
        for row in failures
    )
    provenance = {
        "schema_version": 2,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "benchmark": "table4_constraints_v2",
        "method": "articraft_clean_v2",
        "panel": "structured_main",
        "clean_cohort_only": True,
        "legacy_cohort_used": False,
        "prompt_manifest": str(EXP_ROOT / "reference" / "table4_constraints_v2" / "prompts.jsonl"),
        "prompt_manifest_sha256": PROMPT_SHA256,
        "official_repo": str(ARTICRAFT_ROOT),
        "official_repo_commit": OFFICIAL_COMMIT,
        "generation_entry": "articraft generate",
        "official_compile_entry": "python -m cli.main compile --target full --validate",
        "storage_repo_root": str(STORAGE_ROOT),
        "cohort_boundary_pre": pre,
        "cohort_boundary_post": post,
        "model": "qwen3.6-plus",
        "provider": "dashscope",
        "temperature": None,
        "temperature_policy": "unset_provider_native_default",
        "thinking_level": "high",
        "max_turns": 100,
        "max_cost_usd_per_task": 5.0,
        "timeout_seconds_per_task": 1800,
        "repair_budget": 0,
        "method_native_compile_feedback": True,
        "credential_policy": "workspace .env loaded into child process only; no values recorded",
        "artifact_manifest": str(MANIFEST),
        "successful_artifacts": len(successes),
        "failed_tasks": [row["task_id"] for row in failures],
        "successful_total_cost_usd": sum(float(row.get("cost_usd") or 0.0) for row in successes),
        "successful_total_tokens": sum(int(row.get("total_tokens") or 0) for row in successes),
        "failed_tasks_observed_cost_usd": failed_observed_cost,
        "formal_tasks_observed_cost_usd": sum(float(row.get("cost_usd") or 0.0) for row in successes)
        + failed_observed_cost,
        "tasks": rows,
    }
    (RUN_ROOT / "provenance.json").write_text(
        json.dumps(provenance, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    scorer = contained(EXP_ROOT / "scripts" / "score_table4_constraints_v2.py")
    scorer_files = ("records.json", "summary.json", "report.md")
    score_pairs = (("scorer_structured_run1", "scorer_structured_run2", "score_structured"),
                   ("scorer_numeric_run1", "scorer_numeric_run2", "score_numeric"))
    for run1_name, run2_name, final_name in score_pairs:
        panel = "structured_main" if final_name == "score_structured" else "cad_numeric"
        for output_name in (run1_name, run2_name):
            output_dir = contained(RUN_ROOT / output_name)
            result = subprocess.run(
                [
                    sys.executable,
                    str(scorer),
                    "--method",
                    "articraft_clean_v2",
                    "--panel",
                    panel,
                    "--artifact-manifest",
                    str(MANIFEST),
                    "--output-dir",
                    str(output_dir),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            (RUN_ROOT / f"{output_name}.log").write_text(
                result.stdout + result.stderr, encoding="utf-8"
            )
            if result.returncode != 0:
                raise RuntimeError(f"scorer failed for {output_name}: rc={result.returncode}")
    reproducibility: dict[str, Any] = {}
    for run1_name, run2_name, final_name in score_pairs:
        run1 = RUN_ROOT / run1_name
        run2 = RUN_ROOT / run2_name
        if not all((run1 / name).is_file() and (run2 / name).is_file() for name in scorer_files):
            continue
        hashes1 = {name: sha256(run1 / name) for name in scorer_files}
        hashes2 = {name: sha256(run2 / name) for name in scorer_files}
        final_dir = contained(RUN_ROOT / final_name)
        final_dir.mkdir(parents=True, exist_ok=True)
        for name in scorer_files:
            shutil.copy2(run1 / name, final_dir / name)
        reproducibility[final_name] = {
            "run1": str(run1),
            "run2": str(run2),
            "byte_identical": hashes1 == hashes2,
            "sha256": hashes1,
            "final_score_dir": str(final_dir),
        }
    if reproducibility:
        provenance["scorer_reproducibility"] = reproducibility
        structured = load_json(RUN_ROOT / "score_structured" / "summary.json")
        numeric = load_json(RUN_ROOT / "score_numeric" / "summary.json")
        report_lines = [
            "# Table 4 Constraints v2: Articraft clean v2",
            "",
            f"Prompt manifest SHA-256: `{PROMPT_SHA256}`.",
            "",
            "## Result",
            "",
            f"- Final attributable artifacts: {len(successes)}/18",
            f"- Structured constraints passed: {structured['passed']}/{structured['constraints']}",
            f"- Structured coverage: {structured['coverage']:.6f}",
            f"- Structured satisfaction: {structured['satisfaction']:.6f}",
            f"- Structured conditional accuracy: {structured['conditional_accuracy']:.6f}",
            f"- Structured count pass: {structured['count_pass']}",
            f"- Structured numeric pass: {structured['numeric_pass']}",
            f"- Numeric-only passed: {numeric['passed']}/{numeric['constraints']}",
            f"- Numeric-only measurable: {numeric['measurable']}/{numeric['constraints']}",
            f"- Numeric-only coverage: {numeric['coverage']:.6f}",
            f"- Numeric-only satisfaction: {numeric['satisfaction']:.6f}",
            f"- Numeric-only conditional accuracy: {numeric['conditional_accuracy']:.6f}",
            f"- Structured scorer byte-identical: {str(reproducibility['score_structured']['byte_identical']).lower()}",
            f"- Numeric scorer byte-identical: {str(reproducibility['score_numeric']['byte_identical']).lower()}",
            "",
            "## Runtime",
            "",
            f"- Successful-artifact exact cost: ${provenance['successful_total_cost_usd']:.6f}",
            f"- Failed-task observed cost: ${failed_observed_cost:.6f}",
            f"- Formal-task observed cost: ${provenance['formal_tasks_observed_cost_usd']:.6f}",
            f"- Successful-artifact tokens: {provenance['successful_total_tokens']}",
            "- External repair attempts: 0",
            "- Legacy cohort used: false",
            "",
            "## Tasks",
            "",
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
    print(
        json.dumps(
            {
                "manifest": str(MANIFEST),
                "successes": len(successes),
                "failures": len(failures),
                "successful_total_cost_usd": provenance["successful_total_cost_usd"],
                "successful_total_tokens": provenance["successful_total_tokens"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
