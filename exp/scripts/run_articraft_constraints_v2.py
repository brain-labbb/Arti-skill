#!/usr/bin/env python3
"""Run one frozen Table 4 prompt through the official Articraft CLI."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from dotenv import dotenv_values


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
ARTICRAFT_ROOT = WORKSPACE / "arti-skill" / "articraft_data"
REFERENCE = WORKSPACE / "arti-skill" / "exp" / "reference" / "table4_constraints_v2"
RUN_ROOT = WORKSPACE / "arti-skill" / "exp" / "runtime" / "table4_constraints_v2" / "articraft"
STORAGE_ROOT = RUN_ROOT / "workspace_data"
PROMPT_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
MODEL = "qwen3.6-plus"
PROVIDER = "dashscope"
THINKING = "high"
MAX_TURNS = 100
MAX_COST_USD = 5.0
TIMEOUT_SECONDS = 1800


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def verify_storage_boundary() -> dict[str, object]:
    root = contained(STORAGE_ROOT)
    symlinks: list[str] = []
    checked = 0
    if root.exists():
        for current, dirnames, filenames in os.walk(root, followlinks=False):
            current_path = contained(Path(current))
            for name in [*dirnames, *filenames]:
                path = current_path / name
                checked += 1
                if path.is_symlink():
                    symlinks.append(str(path))
                else:
                    contained(path)
    if symlinks:
        raise RuntimeError(f"storage contains symlinks: {symlinks}")
    return {"storage_root": str(root), "checked_entries": checked, "symlink_count": 0}


def build_child_environment() -> tuple[dict[str, str], dict[str, bool]]:
    env = os.environ.copy()
    values = dotenv_values(contained(ARTICRAFT_ROOT / ".env"))
    keys = ("DASHSCOPE_API_KEY", "DASHSCOPE_API_KEYS", "DASHSCOPE_BASE_URL")
    configured: dict[str, bool] = {}
    for key in keys:
        value = values.get(key)
        configured[key] = bool(value)
        if value:
            env[key] = value
    if not (configured["DASHSCOPE_API_KEY"] or configured["DASHSCOPE_API_KEYS"]):
        raise RuntimeError("DashScope credentials are not configured")
    return env, configured


def load_prompt(task_id: str | None, smoke_prompt: str | None) -> tuple[str, str]:
    if smoke_prompt is not None:
        return "SMOKE", smoke_prompt
    if not task_id:
        raise ValueError("--task-id or --smoke-prompt is required")
    manifest = contained(REFERENCE / "prompts.jsonl")
    if sha256(manifest) != PROMPT_SHA256:
        raise RuntimeError("frozen prompt hash mismatch")
    rows = [json.loads(line) for line in manifest.read_text(encoding="utf-8").splitlines()]
    row = next((item for item in rows if item["task_id"] == task_id), None)
    if row is None:
        raise ValueError(f"unknown task id: {task_id}")
    return task_id, str(row["prompt"])


def matching_records(prompt: str, started_ns: int) -> list[Path]:
    matches = []
    records = contained(STORAGE_ROOT / "data" / "records")
    for prompt_path in records.glob("*/revisions/*/prompt.txt"):
        try:
            if prompt_path.stat().st_mtime_ns >= started_ns and prompt_path.read_text(
                encoding="utf-8"
            ) == prompt:
                matches.append(prompt_path.parents[2])
        except (FileNotFoundError, OSError, UnicodeDecodeError):
            continue
    return sorted(set(matches))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-id")
    parser.add_argument("--smoke-prompt")
    parser.add_argument("--infra-launch-attempt", type=int, default=1)
    parser.add_argument("--model-attempt", type=int, default=1)
    args = parser.parse_args()
    task_id, prompt = load_prompt(args.task_id, args.smoke_prompt)
    task_dir = contained(RUN_ROOT / ("smoke" if task_id == "SMOKE" else task_id))
    task_dir.mkdir(parents=True, exist_ok=True)
    boundary_pre = verify_storage_boundary()
    child_env, provider_env_audit = build_child_environment()
    prompt_path = task_dir / "prompt.txt"
    prompt_path.write_text(prompt, encoding="utf-8")
    command = [
        "uv",
        "run",
        "articraft",
        "generate",
        "--repo-root",
        str(STORAGE_ROOT),
        "--provider",
        PROVIDER,
        "--model",
        MODEL,
        "--thinking-level",
        THINKING,
        "--max-cost-usd",
        str(MAX_COST_USD),
        prompt,
    ]
    command_record = {
        "argv": command,
        "cwd": str(ARTICRAFT_ROOT),
        "storage_repo_root": str(STORAGE_ROOT),
        "sdk_package": "sdk",
        "sdk_version": "workspace/0.1.0",
        "model": MODEL,
        "provider": PROVIDER,
        "temperature": None,
        "temperature_policy": "unset_provider_native_default",
        "thinking_level": THINKING,
        "max_turns": MAX_TURNS,
        "max_cost_usd": MAX_COST_USD,
        "timeout_seconds": TIMEOUT_SECONDS,
        "repair_budget": 0,
        "infra_launch_attempt": args.infra_launch_attempt,
        "model_attempt": args.model_attempt,
        "provider_env_configured": provider_env_audit,
        "provider_env_policy": "loaded_into_child_process_only",
    }
    (task_dir / "command.json").write_text(
        json.dumps(command_record, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    started_at = utc_now()
    started_ns = time.time_ns()
    started = time.monotonic()
    timed_out = False
    log_path = task_dir / "generation.log"
    with log_path.open("w", encoding="utf-8") as log:
        log.write(f"started_at={started_at}\n")
        log.flush()
        process = subprocess.Popen(
            command,
            cwd=ARTICRAFT_ROOT,
            env=child_env,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            exit_code = process.wait(timeout=TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            timed_out = True
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            exit_code = 124
        finished_at = utc_now()
        elapsed = time.monotonic() - started
        log.write(
            f"\nfinished_at={finished_at}\nelapsed_seconds={elapsed:.6f}\n"
            f"exit_code={exit_code}\ntimed_out={str(timed_out).lower()}\n"
        )
    records = matching_records(prompt, started_ns)
    boundary_post = verify_storage_boundary()
    result = {
        "task_id": task_id,
        "infra_launch_attempt": args.infra_launch_attempt,
        "model_attempt": args.model_attempt,
        "started_at": started_at,
        "finished_at": finished_at,
        "elapsed_seconds": elapsed,
        "exit_code": exit_code,
        "timed_out": timed_out,
        "matching_record_dirs": [str(path) for path in records],
        "attributable_record_dir": str(records[0]) if len(records) == 1 else None,
        "status": "success" if exit_code == 0 and len(records) == 1 else "failed",
        "storage_boundary_pre": boundary_pre,
        "storage_boundary_post": boundary_post,
    }
    (task_dir / "run.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["status"] == "success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
