#!/usr/bin/env python3
"""Resume the frozen Articraft Table 5 cohort after the A01 harness crash.

This wrapper preserves the frozen manifest, scorer, budget guard, prompt, and
native runner. It audits the already-consumed A01 attempt and runs A02-A18 by
calling the original runner implementation. The only runtime override decodes
TimeoutExpired output before writing the timeout log.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import subprocess
import sys
import time
from pathlib import Path
from types import ModuleType
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
EXP_ROOT = SCRIPT_DIR.parent
WORKSPACE_ROOT = EXP_ROOT.parent.parent.resolve()
ORIGINAL_RUNNER = SCRIPT_DIR / "run_articraft_editability_v2.py"
MANIFEST = EXP_ROOT / "reference" / "editability_v2" / "articraft_parent_manifest.json"
SCORER = EXP_ROOT / "reference" / "editability_v2" / "articraft_scorer_contract_v2.json"
GUARD = SCRIPT_DIR / "articraft_budget_sitecustomize.py"
DEFAULT_OUTPUT = (
    EXP_ROOT / "runtime" / "nano3d_editability_v2" / "articraft_native_final_d8d6e47a"
)

EXPECTED_MANIFEST_SHA256 = "d8d6e47ab3ab4ca22dbccb90d29796d515c1e772093f1e5d099c62a74a66e94d"
EXPECTED_RUNNER_SHA256 = "7b030beac35338975b587e8c06af466ee4ce3c739d2286241de3d26c1593f5e4"
EXPECTED_GUARD_SHA256 = "d76d833031b05ccfbeb11d674746134d32681ddff590093246caa036ed5de368"
EXPECTED_SCORER_SHA256 = "7ce66aa02aa33869949ac561b59d0f7ff8ef81b591cb4d3565756cb7e02aa877"
EXPECTED_MODEL = "qwen3.7-max-2026-05-20"
EXPECTED_A01_TURNS = 19
EXPECTED_A01_COST_CNY = 5.230068
EXPECTED_TIMEOUT_SECONDS = 1800


def contained(path: Path, *, strict: bool = False) -> Path:
    resolved = path.resolve(strict=strict)
    if not resolved.is_relative_to(WORKSPACE_ROOT):
        raise RuntimeError(f"path escapes workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path, strict=True).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(contained(path, strict=True).read_text(encoding="utf-8"))


def dump_json(path: Path, value: Any) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_original_runner() -> ModuleType:
    spec = importlib.util.spec_from_file_location("articraft_table5_frozen_runner", ORIGINAL_RUNNER)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load frozen runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def decode_timeout_stream(value: str | bytes | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def safe_run_command(
    command: list[str],
    *,
    cwd: Path,
    timeout: int,
    log_path: Path,
    env: dict[str, str] | None = None,
) -> dict[str, Any]:
    started = time.monotonic()
    try:
        completed = subprocess.run(
            command,
            cwd=contained(cwd, strict=True),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
            env=env,
        )
        result = {
            "returncode": completed.returncode,
            "timed_out": False,
            "elapsed_seconds": round(time.monotonic() - started, 3),
            "command": command,
        }
        output = completed.stdout
    except subprocess.TimeoutExpired as exc:
        result = {
            "returncode": 124,
            "timed_out": True,
            "elapsed_seconds": round(time.monotonic() - started, 3),
            "command": command,
        }
        output = decode_timeout_stream(exc.stdout) + "\nTIMEOUT\n"
    target = contained(log_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(output, encoding="utf-8")
    result["log"] = str(target.relative_to(WORKSPACE_ROOT))
    return result


def read_usage(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with contained(path, strict=True).open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def aggregate_usage(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "turns": len(rows),
        "prompt_tokens": sum(row["usage"]["prompt_tokens"] for row in rows),
        "cached_tokens": sum(row["usage"]["cached_tokens"] for row in rows),
        "uncached_prompt_tokens": sum(row["usage"]["uncached_prompt_tokens"] for row in rows),
        "output_tokens": sum(row["usage"]["candidates_tokens"] for row in rows),
        "total_tokens": sum(row["usage"]["total_tokens"] for row in rows),
        "cost_cny": round(sum(row["turn_cost_cny"] for row in rows), 7),
    }


def static_gate(output: Path, native: ModuleType) -> tuple[dict[str, Any], dict[str, Any]]:
    errors: list[str] = []
    hashes = {
        "manifest": sha256(MANIFEST),
        "original_runner": sha256(ORIGINAL_RUNNER),
        "budget_guard": sha256(GUARD),
        "scorer": sha256(SCORER),
    }
    expected_hashes = {
        "manifest": EXPECTED_MANIFEST_SHA256,
        "original_runner": EXPECTED_RUNNER_SHA256,
        "budget_guard": EXPECTED_GUARD_SHA256,
        "scorer": EXPECTED_SCORER_SHA256,
    }
    for name, expected in expected_hashes.items():
        if hashes[name] != expected:
            errors.append(f"{name.upper()}_SHA256_MISMATCH")

    manifest = load_json(MANIFEST)
    errors.extend(native.validate_manifest(manifest, MANIFEST))
    if native.MODEL_SNAPSHOT != EXPECTED_MODEL:
        errors.append("MODEL_SNAPSHOT_MISMATCH")
    if native.RECORD_TIMEOUT_SECONDS != EXPECTED_TIMEOUT_SECONDS:
        errors.append("RECORD_TIMEOUT_MISMATCH")

    snapshot = contained(output / "repo_snapshot", strict=True)
    stamp = contained(snapshot / "MANIFEST.sha256", strict=True).read_text(encoding="utf-8").strip()
    if stamp != EXPECTED_MANIFEST_SHA256:
        errors.append("SNAPSHOT_MANIFEST_SHA256_MISMATCH")

    usage_rows = read_usage(output / "cny_budget" / "usage.jsonl")
    a01_rows = [row for row in usage_rows if row.get("task_id") == "A01"]
    other_rows = [row for row in usage_rows if row.get("task_id") != "A01"]
    usage = aggregate_usage(a01_rows)
    if other_rows:
        errors.append("NON_A01_USAGE_ALREADY_PRESENT")
    if usage["turns"] != EXPECTED_A01_TURNS:
        errors.append("A01_TURN_COUNT_MISMATCH")
    if abs(usage["cost_cny"] - EXPECTED_A01_COST_CNY) > 1e-7:
        errors.append("A01_COST_MISMATCH")
    total_state = load_json(output / "cny_budget" / "total.json")
    if abs(float(total_state["spent_cny"]) - EXPECTED_A01_COST_CNY) > 1e-7:
        errors.append("COHORT_LEDGER_MISMATCH")

    tasks = manifest["tasks"]
    if [task["task_id"] for task in tasks] != [
        "A01", "A02", "A03", "A04", "A05", "A06",
        "M01", "M02", "M03", "M04", "M05", "M06",
        "S01", "S02", "S03", "S04", "S05", "S06",
    ]:
        errors.append("TASK_ORDER_MISMATCH")
    if len({task["fresh_record_id"] for task in tasks}) != len(tasks):
        errors.append("DUPLICATE_FRESH_RECORD_ID")
    if len({task["parent_record_id"] for task in tasks}) != len(tasks):
        errors.append("DUPLICATE_PARENT_RECORD_ID")
    for task in tasks:
        child = snapshot / "data" / "records" / task["fresh_record_id"]
        if child.exists():
            errors.append(f"{task['task_id']}:FRESH_CHILD_ALREADY_EXISTS")
    if (output / "fresh_results" / "A01.json").exists():
        errors.append("A01_FRESH_RESULT_ALREADY_EXISTS")
    if (output / "run_summary.json").exists():
        errors.append("RUN_SUMMARY_ALREADY_EXISTS")

    audit = {
        "task_id": "A01",
        "record_id": tasks[0]["fresh_record_id"],
        "status": "TIMEOUT_1800S",
        "harness_failure": "HARNESS_LOGGING_TYPEERROR",
        "fork": {
            "returncode": 124,
            "timed_out": True,
            "elapsed_seconds": 1800.0,
            "log": None,
        },
        "compile": {"status": "N/A", "reason": "FORK_DID_NOT_RETURN"},
        "score": {"status": "N/A", "reason": "FORK_DID_NOT_RETURN"},
        "usage": usage,
        "record_level_retry": 0,
    }
    gate = {
        "schema_version": 1,
        "status": "READY_TO_RESUME" if not errors else "BLOCKED",
        "hashes": hashes,
        "model_snapshot": native.MODEL_SNAPSHOT,
        "record_timeout_seconds": native.RECORD_TIMEOUT_SECONDS,
        "resume_task_ids": [task["task_id"] for task in tasks[1:]],
        "a01_audit": audit,
        "errors": sorted(set(errors)),
    }
    return gate, manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--expected-manifest-sha256", required=True)
    parser.add_argument("--confirm-model-snapshot", required=True)
    parser.add_argument("--record-timeout-seconds", type=int, default=EXPECTED_TIMEOUT_SECONDS)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        assert decode_timeout_stream(None) == ""
        assert decode_timeout_stream("text") == "text"
        assert decode_timeout_stream(b"bytes\xff") == "bytes\ufffd"
        print(json.dumps({"timeout_bytes_decode": True}))
        return 0
    output = contained(args.output_dir, strict=True)
    native = load_original_runner()
    gate, manifest = static_gate(output, native)
    if args.expected_manifest_sha256.lower() != EXPECTED_MANIFEST_SHA256:
        gate["errors"].append("CLI_MANIFEST_SHA256_MISMATCH")
    if args.confirm_model_snapshot != EXPECTED_MODEL:
        gate["errors"].append("CLI_MODEL_SNAPSHOT_MISMATCH")
    if args.record_timeout_seconds != EXPECTED_TIMEOUT_SECONDS:
        gate["errors"].append("CLI_RECORD_TIMEOUT_MISMATCH")
    gate["errors"] = sorted(set(gate["errors"]))
    gate["status"] = "READY_TO_RESUME" if not gate["errors"] else "BLOCKED"
    dump_json(output / "recovery_preflight.json", gate)
    print(json.dumps(gate, indent=2, sort_keys=True))
    if gate["errors"]:
        return 2
    if not args.execute:
        return 0

    a01_audit = gate["a01_audit"]
    dump_json(output / "fresh_results" / "A01.json", a01_audit)
    subset = copy.deepcopy(manifest)
    subset["tasks"] = manifest["tasks"][1:]
    native.run_command = safe_run_command
    recovered = native.run_forks(
        subset,
        output / "repo_snapshot",
        output,
        args.record_timeout_seconds,
    )
    records = [a01_audit, *recovered]
    summary = {
        "schema_version": 1,
        "protocol_id": "table5_articraft_native_fork_v2_recovery",
        "manifest_sha256": EXPECTED_MANIFEST_SHA256,
        "model_snapshot": EXPECTED_MODEL,
        "paid_api_called": True,
        "recovery_harness_sha256": sha256(Path(__file__)),
        "original_runner_sha256": EXPECTED_RUNNER_SHA256,
        "records": records,
        "passed": sum(bool(item.get("score", {}).get("passed")) for item in records),
        "failed": sum(not bool(item.get("score", {}).get("passed")) for item in records),
        "manual_blind_review": "N/A",
    }
    dump_json(output / "run_summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
