#!/usr/bin/env python3
"""Run one frozen same-LLM Table 4 arm and register final artifacts."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
WORKSPACE = PROJECT_ROOT.parent.resolve()
REFERENCE = EXP_ROOT / "reference/table4_constraints_v2"
PROMPTS = REFERENCE / "prompts.jsonl"
PROTOCOL = REFERENCE / "protocol.json"
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
AUTHORING = TEMPLATE_ROOT / "articraft_template_authoring/AUTHORING.md"
AUTHORING_README = TEMPLATE_ROOT / "articraft_template_authoring/README.md"
MECHANICAL_PRIORS = TEMPLATE_ROOT / "articraft_template_authoring/MECHANICAL_PRIORS.md"
VISUAL_DIVERSITY = TEMPLATE_ROOT / "articraft_template_authoring/VISUAL_DIVERSITY_MODEL.md"
SDK_ALLOWANCE = TEMPLATE_ROOT / "agent/prompts/sections/sdk_base.md"
BUILD_TEMPLATE_SKILL = PROJECT_ROOT / ".agents/skills/build-template/SKILL.md"
EVALUATOR = EXP_ROOT / "scripts/evaluate_t2_generated_template.py"
CANONICALIZER = EXP_ROOT / "scripts/canonicalize_table4_artifact.py"
SCORER = EXP_ROOT / "scripts/score_table4_constraints_v2.py"
CODEX = Path("/mnt/zsn/miniconda3/bin/codex")
PYTHON = TEMPLATE_ROOT / ".venv/bin/python"
MODEL = "gpt-5.6-sol"
REASONING_EFFORT = "high"


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def dump_json(path: Path, payload: Any) -> None:
    contained(path).parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )


def sha256(path: Path) -> str:
    return hashlib.sha256(contained(path).read_bytes()).hexdigest()


def load_protocol() -> tuple[dict[str, Any], list[dict[str, Any]]]:
    protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    if sha256(PROMPTS) != protocol["prompt_manifest_sha256"]:
        raise RuntimeError("frozen prompt manifest hash mismatch")
    prompts = [json.loads(line) for line in PROMPTS.read_text(encoding="utf-8").splitlines() if line]
    if len(prompts) != protocol["task_count"]:
        raise RuntimeError("frozen task count mismatch")
    return protocol, prompts


def usage(events: Path) -> dict[str, int]:
    totals = {
        "input_tokens": 0,
        "cached_input_tokens": 0,
        "output_tokens": 0,
        "reasoning_output_tokens": 0,
    }
    if not events.is_file():
        return totals
    for line in events.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get("type") != "turn.completed":
            continue
        values = row.get("usage") or {}
        for key in totals:
            totals[key] += int(values.get(key) or 0)
    return totals


def ours_prompt(row: dict[str, Any], output: Path, full_docs: bool = False) -> str:
    template = output / "template.py"
    if full_docs:
        reading = f"""Read the complete method-standard authoring stack:
- `{BUILD_TEMPLATE_SKILL}`
- `{AUTHORING_README}`
- `{AUTHORING}`
- `{MECHANICAL_PRIORS}`
- `{VISUAL_DIVERSITY}`
- `{SDK_ALLOWANCE}`
- the public SDK under `{TEMPLATE_ROOT / 'sdk'}`"""
    else:
        reading = f"""Read only the common authoring contract `{AUTHORING}` and the public SDK under
`{TEMPLATE_ROOT / 'sdk'}`"""
    return f"""
You are the generation backend in a frozen, zero-repair constraints benchmark.
Create exactly one procedural Arti-Template implementation for this user request:

{row['prompt']}

{reading}. Do not inspect benchmark specs, scorer code, other
methods, other tasks, generated outputs, or existing templates. The user request
above is the complete category-specific evidence.

Write only `{template}`. The module must be self-contained and use metric metres.
It must expose exactly one public `build_*`, exactly one public `run_*_tests`,
`Config`, `config_from_seed`, `resolve_config`, `TEMPLATE_DOMAIN`, optional
`TEMPLATE_CORNERS`, and a precise `__all__`. Every requested counted role must be
represented by exactly that many separately named mesh-bearing parts or visuals;
use zero-padded role names such as `leg_01`. Preserve the requested overall scale,
Z-up orientation, upright ground contact, connected hierarchy, finite geometry,
and deterministic seed 0. Use the SDK's real geometry and ObjectModel APIs; do not
emit metadata-only placeholders and do not call external services.

You may run import/build smoke checks while authoring. Do not modify any other
file. Finish after the requested file exists; there is no repair turn.
""".strip()


def naive_prompt(row: dict[str, Any], output: Path) -> str:
    script = output / "generate.py"
    artifact = output / "artifact.glb"
    return f"""
You are the generation backend in a frozen, zero-repair constraints benchmark.
Create exactly one 3D object for this user request:

{row['prompt']}

Write only the self-contained Python program `{script}`. It will be executed once
with Python, numpy, and trimesh installed and must write a valid binary GLB to
exactly `{artifact}`. Use metric metres, Z up, upright ground contact, and real
finite triangle meshes. Put every requested counted role in exactly that many
separate scene nodes with semantic zero-padded names such as `leg_01`; do not merge
those nodes. Preserve the requested overall dimensions. You may use trimesh
primitives and transforms, but may not import repository SDKs, read benchmark
specs/scorers/other outputs, call external services, or write any other file.

Do not merely print code in prose. Finish after the requested program exists;
there is no repair turn.
""".strip()


def run_command(command: list[str], cwd: Path, timeout: float) -> dict[str, Any]:
    started = time.monotonic()
    try:
        completed = subprocess.run(
            command,
            cwd=contained(cwd),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
        )
        return {
            "exit_code": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
            "elapsed_s": time.monotonic() - started,
            "timed_out": False,
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "exit_code": 124,
            "stdout": exc.stdout or "",
            "stderr": exc.stderr or "",
            "elapsed_s": time.monotonic() - started,
            "timed_out": True,
        }


def codex_generate(prompt: str, output: Path, timeout: float) -> dict[str, Any]:
    events = output / "generation.events.jsonl"
    stderr = output / "generation.stderr.txt"
    result = run_command(
        [
            str(CODEX), "exec", "--json", "--model", MODEL,
            "-c", f"model_reasoning_effort='{REASONING_EFFORT}'",
            "--dangerously-bypass-approvals-and-sandbox",
            "--skip-git-repo-check", prompt,
        ],
        PROJECT_ROOT,
        timeout,
    )
    events.write_text(str(result.pop("stdout")), encoding="utf-8")
    stderr.write_text(str(result.pop("stderr")), encoding="utf-8")
    return {
        **result,
        "cwd": str(PROJECT_ROOT),
        "events": str(events),
        "stderr": str(stderr),
        "usage": usage(events),
    }


def execute_ours(output: Path, timeout: float) -> tuple[Path | None, dict[str, Any]]:
    template = output / "template.py"
    evaluation = output / "evaluation"
    if not template.is_file():
        return None, {"exit_code": 2, "error": "missing generated template"}
    result = run_command(
        [
            str(PYTHON), str(EVALUATOR), "--template", str(template),
            "--out", str(evaluation), "--seeds", "0-0", "--workers", "1",
            "--timeout", str(min(timeout, 600.0)),
        ],
        PROJECT_ROOT,
        timeout,
    )
    (output / "execution.stdout.txt").write_text(str(result.pop("stdout")), encoding="utf-8")
    (output / "execution.stderr.txt").write_text(str(result.pop("stderr")), encoding="utf-8")
    urdf = evaluation / "cases/seed_000/model.urdf"
    return (urdf if urdf.is_file() else None), result


def execute_naive(output: Path, timeout: float) -> tuple[Path | None, dict[str, Any]]:
    script = output / "generate.py"
    artifact = output / "artifact.glb"
    if not script.is_file():
        return None, {"exit_code": 2, "error": "missing generated script"}
    env = os.environ.copy()
    env.update({"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"})
    started = time.monotonic()
    try:
        completed = subprocess.run(
            [str(PYTHON), str(script)], cwd=contained(output), env=env,
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=min(timeout, 600.0), check=False,
        )
        result = {
            "exit_code": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
            "elapsed_s": time.monotonic() - started,
            "timed_out": False,
        }
    except subprocess.TimeoutExpired as exc:
        result = {
            "exit_code": 124, "stdout": exc.stdout or "", "stderr": exc.stderr or "",
            "elapsed_s": time.monotonic() - started, "timed_out": True,
        }
    (output / "execution.stdout.txt").write_text(str(result.pop("stdout")), encoding="utf-8")
    (output / "execution.stderr.txt").write_text(str(result.pop("stderr")), encoding="utf-8")
    return (artifact if artifact.is_file() else None), result


def canonicalize(method: str, source: Path, output: Path, timeout: float) -> dict[str, Any]:
    canonical = output / "canonical"
    artifact_type = "urdf" if method.startswith("ours") else "glb"
    result = run_command(
        [
            str(PYTHON), str(CANONICALIZER), "--input", str(source),
            "--artifact-type", artifact_type, "--unit-scale-to-m", "1.0",
            "--output-dir", str(canonical),
        ],
        PROJECT_ROOT,
        timeout,
    )
    (output / "canonicalize.stdout.txt").write_text(str(result.pop("stdout")), encoding="utf-8")
    (output / "canonicalize.stderr.txt").write_text(str(result.pop("stderr")), encoding="utf-8")
    return {**result, "canonical_dir": str(canonical)}


def run_task(method: str, row: dict[str, Any], root: Path, timeout: float, force: bool) -> dict[str, Any]:
    output = contained(root / row["task_id"])
    output.mkdir(parents=True, exist_ok=True)
    record_path = output / "record.json"
    if record_path.is_file() and not force:
        return json.loads(record_path.read_text(encoding="utf-8"))
    if method == "naive_same_llm":
        prompt_text = naive_prompt(row, output)
    else:
        prompt_text = ours_prompt(row, output, full_docs=method == "ours_full_docs")
    (output / "prompt.txt").write_text(prompt_text + "\n", encoding="utf-8")
    generation = codex_generate(prompt_text, output, timeout)
    source, execution = (
        execute_ours(output, timeout) if method.startswith("ours") else execute_naive(output, timeout)
    )
    canonical = None
    status = "failed"
    if source is not None:
        canonical = canonicalize(method, source, output, timeout)
        if canonical["exit_code"] == 0 and (output / "canonical/artifact.json").is_file():
            status = "success"
    record = {
        "schema_version": 2,
        "benchmark_id": "table4_constraints_v2",
        "task_id": row["task_id"],
        "method": method,
        "status": status,
        "model": MODEL,
        "reasoning_effort": REASONING_EFFORT,
        "repair_budget": 0,
        "prompt_sha256": hashlib.sha256((row["prompt"] + "\n").encode()).hexdigest(),
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "generation": generation,
        "execution": execution,
        "source_artifact": str(source) if source else None,
        "source_artifact_sha256": sha256(source) if source else None,
        "canonicalization": canonical,
        "canonical_dir": str(output / "canonical") if status == "success" else None,
    }
    dump_json(record_path, record)
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--method", choices=("ours", "ours_full_docs", "naive_same_llm"), required=True
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=1800.0)
    parser.add_argument("--task-id", action="append")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    protocol, prompts = load_protocol()
    root = contained(args.output_dir)
    root.mkdir(parents=True, exist_ok=True)
    if args.task_id:
        selected = set(args.task_id)
        prompts = [row for row in prompts if row["task_id"] in selected]
        missing = selected - {row["task_id"] for row in prompts}
        if missing:
            raise ValueError(f"unknown task IDs: {sorted(missing)}")
    dump_json(root / "run_config.json", {
        "schema_version": 2,
        "benchmark_id": protocol["benchmark_id"],
        "prompt_manifest_sha256": protocol["prompt_manifest_sha256"],
        "method": args.method,
        "model": MODEL,
        "reasoning_effort": REASONING_EFFORT,
        "repair_budget": 0,
        "workers": args.workers,
        "timeout_s": args.timeout,
        "selected_task_ids": [row["task_id"] for row in prompts],
    })
    records = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = {
            pool.submit(run_task, args.method, row, root, args.timeout, args.force): row["task_id"]
            for row in prompts
        }
        for future in concurrent.futures.as_completed(futures):
            records.append(future.result())
    records.sort(key=lambda row: row["task_id"])
    manifest = root / "artifact_manifest.jsonl"
    manifest.write_text(
        "".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in records),
        encoding="utf-8",
    )
    score_dir = root / "score"
    score = run_command(
        [
            str(PYTHON), str(SCORER), "--method", args.method,
            "--panel", "structured_main", "--artifact-manifest", str(manifest),
            "--output-dir", str(score_dir),
        ],
        PROJECT_ROOT,
        args.timeout,
    )
    (root / "score.stdout.txt").write_text(str(score.pop("stdout")), encoding="utf-8")
    (root / "score.stderr.txt").write_text(str(score.pop("stderr")), encoding="utf-8")
    summary = {
        "tasks": len(records),
        "success": sum(row["status"] == "success" for row in records),
        "failed": sum(row["status"] != "success" for row in records),
        "score_exit_code": score["exit_code"],
        "score_summary": str(score_dir / "summary.json"),
    }
    dump_json(root / "run_summary.json", summary)
    print(json.dumps(summary, indent=2))
    return 0 if score["exit_code"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
