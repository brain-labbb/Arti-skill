#!/usr/bin/env python3
"""Run one frozen Table 4 task in the isolated Articraft clean-v2 cohort."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from dotenv import dotenv_values


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
REPO_ROOT = WORKSPACE / "arti-skill"
ARTICRAFT_ROOT = REPO_ROOT / "articraft_data"
REFERENCE_ROOT = REPO_ROOT / "exp" / "reference" / "table4_constraints_v2"
RUN_ROOT = REPO_ROOT / "exp" / "runtime" / "table4_constraints_v2" / "articraft_clean_v2"
STORAGE_ROOT = RUN_ROOT / "workspace_data"
PROCESS_ENV_ROOT = RUN_ROOT / "_process_env"
PROMPT_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
OFFICIAL_COMMIT = "06cd75fbc9e90fae33f127b494d13c35090356be"
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
    with contained(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def verify_tree_boundary(root: Path) -> dict[str, Any]:
    root = contained(root)
    checked = 0
    symlinks: list[str] = []
    if root.exists():
        if root.is_symlink():
            symlinks.append(str(root))
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
        raise RuntimeError(f"clean cohort contains symlinks: {symlinks}")
    return {
        "root": str(root),
        "checked_entries": checked,
        "symlink_count": 0,
        "all_realpaths_in_workspace": True,
    }


def verify_official_commit() -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=contained(ARTICRAFT_ROOT),
        text=True,
        capture_output=True,
        check=True,
    )
    commit = result.stdout.strip()
    if commit != OFFICIAL_COMMIT:
        raise RuntimeError(f"official checkout commit mismatch: {commit}")
    return commit


def ensure_empty_storage() -> None:
    run_root = contained(RUN_ROOT)
    storage_root = contained(STORAGE_ROOT)
    if run_root.exists() and any(run_root.iterdir()):
        raise RuntimeError(f"clean cohort root is not empty: {run_root}")
    storage_root.mkdir(parents=True, exist_ok=False)
    (storage_root / "data").mkdir()
    verify_tree_boundary(run_root)


def build_child_environment() -> tuple[dict[str, str], dict[str, bool]]:
    source = contained(ARTICRAFT_ROOT / ".env")
    values = dotenv_values(source)
    keys = ("DASHSCOPE_API_KEY", "DASHSCOPE_API_KEYS", "DASHSCOPE_BASE_URL")
    configured = {key: bool(values.get(key)) for key in keys}
    if not (configured["DASHSCOPE_API_KEY"] or configured["DASHSCOPE_API_KEYS"]):
        raise RuntimeError("DashScope credentials are not configured")

    env = {
        key: value
        for key, value in os.environ.items()
        if not any(marker in key.upper() for marker in ("API_KEY", "TOKEN", "SECRET", "PASSWORD"))
    }
    env.pop("DASHSCOPE_BASE_URL", None)
    for key in keys:
        value = values.get(key)
        if value:
            env[key] = str(value)
    process_root = contained(PROCESS_ENV_ROOT)
    for child in ("home", "tmp", "cache", "config", "data"):
        (process_root / child).mkdir(parents=True, exist_ok=True)
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
    return env, configured


def load_prompt(task_id: str) -> str:
    manifest = contained(REFERENCE_ROOT / "prompts.jsonl")
    if sha256(manifest) != PROMPT_SHA256:
        raise RuntimeError("frozen prompt hash mismatch")
    rows = [json.loads(line) for line in manifest.read_text(encoding="utf-8").splitlines()]
    row = next((item for item in rows if item["task_id"] == task_id), None)
    if row is None:
        raise ValueError(f"unknown task id: {task_id}")
    return str(row["prompt"])


def matching_records(prompt: str, started_ns: int) -> list[Path]:
    matches: list[Path] = []
    records = contained(STORAGE_ROOT / "data" / "records")
    for prompt_path in records.glob("*/revisions/*/prompt.txt"):
        try:
            if (
                prompt_path.stat().st_mtime_ns >= started_ns
                and prompt_path.read_text(encoding="utf-8") == prompt
            ):
                matches.append(contained(prompt_path.parents[2]))
        except (FileNotFoundError, OSError, UnicodeDecodeError):
            continue
    return sorted(set(matches))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-id")
    parser.add_argument("--infra-launch-attempt", type=int, default=1)
    parser.add_argument("--model-attempt", type=int, default=1)
    parser.add_argument("--init-only", action="store_true")
    args = parser.parse_args()

    official_commit = verify_official_commit()
    if args.init_only:
        ensure_empty_storage()
        print(json.dumps({"storage_root": str(STORAGE_ROOT), "boundary": verify_tree_boundary(RUN_ROOT)}))
        return 0
    if not args.task_id:
        parser.error("--task-id is required")
    if args.model_attempt != 1:
        raise RuntimeError("clean protocol allows exactly model_attempt=1")

    prompt = load_prompt(args.task_id)
    boundary_pre = verify_tree_boundary(RUN_ROOT)
    child_env, provider_env_audit = build_child_environment()
    task_dir = contained(RUN_ROOT / args.task_id)
    task_dir.mkdir(parents=True, exist_ok=False)
    prompt_path = task_dir / "prompt.txt"
    prompt_path.write_text(prompt, encoding="utf-8")
    command = [
        str(ARTICRAFT_ROOT / ".venv" / "bin" / "articraft"),
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
        "official_commit": official_commit,
        "official_entry": "articraft generate",
        "storage_repo_root": str(STORAGE_ROOT),
        "model": MODEL,
        "provider": PROVIDER,
        "temperature": None,
        "temperature_policy": "unset_provider_native_default",
        "thinking_level": THINKING,
        "max_turns": MAX_TURNS,
        "max_turns_policy": "official qwen default resolved by agent.defaults to 100",
        "max_cost_usd": MAX_COST_USD,
        "timeout_seconds": TIMEOUT_SECONDS,
        "repair_budget": 0,
        "infra_launch_attempt": args.infra_launch_attempt,
        "model_attempt": args.model_attempt,
        "provider_env_configured": provider_env_audit,
        "provider_env_policy": "workspace .env loaded into child process only",
        "boundary_pre": boundary_pre,
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
            cwd=contained(ARTICRAFT_ROOT),
            env=child_env,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            exit_code = process.wait(timeout=TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(process.pid, 15)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, 9)
                process.wait()
            exit_code = 124
        finished_at = utc_now()
        elapsed = time.monotonic() - started
        log.write(
            f"\nfinished_at={finished_at}\nelapsed_seconds={elapsed:.6f}\n"
            f"exit_code={exit_code}\ntimed_out={str(timed_out).lower()}\n"
        )

    records = matching_records(prompt, started_ns)
    boundary_post = verify_tree_boundary(RUN_ROOT)
    result = {
        "task_id": args.task_id,
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
