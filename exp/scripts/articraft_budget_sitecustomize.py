"""Fail-closed DashScope CNY budget guard for the Table 5 native-fork run."""

from __future__ import annotations

import asyncio
import fcntl
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from agent.providers.dashscope import DashScopeLLM


PER_RECORD_CNY = 20.0
TOTAL_CNY = 360.0
UNCACHED_INPUT_CNY_PER_M = 12.0
CACHED_INPUT_CNY_PER_M = 2.4
OUTPUT_CNY_PER_M = 36.0
MAX_OUTPUT_TOKENS = 65_536
MIN_OUTPUT_TOKENS = 16


def _required_path(name: str) -> Path:
    value = os.environ.get(name, "").strip()
    if not value:
        raise RuntimeError(f"missing required budget environment variable: {name}")
    return Path(value).resolve()


def _payload_input_token_upper_bound(payload: dict[str, Any]) -> int:
    # A UTF-8 byte count is a conservative upper bound for byte-level BPE tokens.
    payload_bytes = len(
        json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    )
    return payload_bytes + 4_096


def _usage_cost_cny(usage: dict[str, Any]) -> tuple[dict[str, int], float]:
    prompt = max(0, int(usage.get("prompt_tokens", 0)))
    cached = min(prompt, max(0, int(usage.get("cached_tokens", 0))))
    output = max(0, int(usage.get("candidates_tokens", 0)))
    uncached = prompt - cached
    cost = (
        uncached * UNCACHED_INPUT_CNY_PER_M
        + cached * CACHED_INPUT_CNY_PER_M
        + output * OUTPUT_CNY_PER_M
    ) / 1_000_000
    return {
        "prompt_tokens": prompt,
        "cached_tokens": cached,
        "uncached_prompt_tokens": uncached,
        "candidates_tokens": output,
        "total_tokens": max(0, int(usage.get("total_tokens", prompt + output))),
    }, cost


def _read_total_locked(handle: Any) -> float:
    handle.seek(0)
    raw = handle.read().strip()
    if not raw:
        return 0.0
    payload = json.loads(raw)
    return float(payload.get("spent_cny", 0.0))


def _write_total_locked(handle: Any, spent_cny: float) -> None:
    handle.seek(0)
    handle.truncate()
    json.dump({"spent_cny": round(spent_cny, 10)}, handle, sort_keys=True)
    handle.write("\n")
    handle.flush()
    os.fsync(handle.fileno())


_original_generate_with_tools = DashScopeLLM.generate_with_tools


async def _guarded_generate_with_tools(
    self: DashScopeLLM,
    system_prompt: str,
    messages: list[dict],
    tools: list[dict],
) -> dict[str, Any]:
    task_id = os.environ.get("ARTICRAFT_BUDGET_TASK_ID", "unknown")
    state_path = _required_path("ARTICRAFT_CNY_TOTAL_STATE")
    usage_path = _required_path("ARTICRAFT_CNY_USAGE_JSONL")
    state_path.parent.mkdir(parents=True, exist_ok=True)
    usage_path.parent.mkdir(parents=True, exist_ok=True)
    record_spent = float(getattr(self, "_articraft_record_spent_cny", 0.0))

    with state_path.open("a+", encoding="utf-8") as state_handle:
        fcntl.flock(state_handle.fileno(), fcntl.LOCK_EX)
        total_spent = _read_total_locked(state_handle)
        available = min(PER_RECORD_CNY - record_spent, TOTAL_CNY - total_spent)
        fcntl.flock(state_handle.fileno(), fcntl.LOCK_UN)

    original_max_tokens = self.max_tokens
    self.max_tokens = MAX_OUTPUT_TOKENS
    preview = self._build_chat_payload(system_prompt=system_prompt, messages=messages, tools=tools)
    input_token_upper = _payload_input_token_upper_bound(preview)
    input_cost_reserve = input_token_upper * UNCACHED_INPUT_CNY_PER_M / 1_000_000
    output_cap = math.floor((available - input_cost_reserve) * 1_000_000 / OUTPUT_CNY_PER_M)
    output_cap = min(MAX_OUTPUT_TOKENS, output_cap)
    if output_cap < MIN_OUTPUT_TOKENS:
        self.max_tokens = original_max_tokens
        raise RuntimeError(
            f"CNY_BUDGET_PRE_REQUEST_BLOCKED task={task_id} available={available:.6f} "
            f"input_reserve={input_cost_reserve:.6f}"
        )

    self.max_tokens = output_cap
    try:
        response = await _original_generate_with_tools(self, system_prompt, messages, tools)
    finally:
        self.max_tokens = original_max_tokens
    usage = response.get("usage") if isinstance(response, dict) else None
    if not isinstance(usage, dict):
        raise RuntimeError(f"CNY_BUDGET_USAGE_MISSING task={task_id}")
    normalized_usage, turn_cost = _usage_cost_cny(usage)
    if turn_cost > available + 1e-9:
        raise RuntimeError(f"CNY_BUDGET_ACCOUNTING_OVERRUN task={task_id}")

    with state_path.open("a+", encoding="utf-8") as state_handle:
        fcntl.flock(state_handle.fileno(), fcntl.LOCK_EX)
        current_total = _read_total_locked(state_handle)
        updated_total = current_total + turn_cost
        if updated_total > TOTAL_CNY + 1e-9:
            fcntl.flock(state_handle.fileno(), fcntl.LOCK_UN)
            raise RuntimeError(f"CNY_TOTAL_BUDGET_ACCOUNTING_OVERRUN task={task_id}")
        _write_total_locked(state_handle, updated_total)
        fcntl.flock(state_handle.fileno(), fcntl.LOCK_UN)

    record_spent += turn_cost
    setattr(self, "_articraft_record_spent_cny", record_spent)
    event = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "task_id": task_id,
        "input_token_upper_bound": input_token_upper,
        "requested_max_output_tokens": output_cap,
        "usage": normalized_usage,
        "turn_cost_cny": round(turn_cost, 10),
        "record_spent_cny": round(record_spent, 10),
        "total_spent_cny": round(updated_total, 10),
        "per_record_cap_cny": PER_RECORD_CNY,
        "total_cap_cny": TOTAL_CNY,
    }
    with usage_path.open("a", encoding="utf-8") as usage_handle:
        fcntl.flock(usage_handle.fileno(), fcntl.LOCK_EX)
        usage_handle.write(json.dumps(event, sort_keys=True) + "\n")
        usage_handle.flush()
        os.fsync(usage_handle.fileno())
        fcntl.flock(usage_handle.fileno(), fcntl.LOCK_UN)
    return response


DashScopeLLM.generate_with_tools = _guarded_generate_with_tools
DashScopeLLM._articraft_cny_budget_guard = "table5_v2_20cny_per_record_360cny_total"


def run_mock_self_tests() -> dict[str, bool]:
    """Exercise every fail-closed branch without constructing a network client."""
    root = _required_path("ARTICRAFT_CNY_SELF_TEST_DIR")
    root.mkdir(parents=True, exist_ok=True)
    state_path = root / "total.json"
    usage_path = root / "usage.jsonl"
    os.environ["ARTICRAFT_CNY_TOTAL_STATE"] = str(state_path)
    os.environ["ARTICRAFT_CNY_USAGE_JSONL"] = str(usage_path)
    os.environ["ARTICRAFT_BUDGET_TASK_ID"] = "MOCK"
    results: dict[str, bool] = {}

    async def success_transport(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        return {
            "content": "ok",
            "tool_calls": [],
            "usage": {
                "prompt_tokens": 100,
                "cached_tokens": 20,
                "candidates_tokens": 50,
                "total_tokens": 150,
            },
        }

    async def missing_usage_transport(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        return {"content": "missing", "tool_calls": []}

    async def invoke(client: DashScopeLLM) -> dict[str, Any]:
        return await _guarded_generate_with_tools(client, "system", [{"role": "user", "content": "edit"}], [])

    global _original_generate_with_tools
    saved_transport = _original_generate_with_tools
    try:
        state_path.write_text('{"spent_cny": 0.0}\n', encoding="utf-8")
        usage_path.write_text("", encoding="utf-8")
        _original_generate_with_tools = success_transport
        client = DashScopeLLM(model_id="qwen3.7-max-2026-05-20", dry_run=True)
        asyncio.run(invoke(client))
        asyncio.run(invoke(client))
        ledger = json.loads(state_path.read_text(encoding="utf-8"))
        usage_lines = [line for line in usage_path.read_text(encoding="utf-8").splitlines() if line]
        results["success_usage_and_total_ledger"] = float(ledger["spent_cny"]) > 0 and len(usage_lines) == 2

        state_path.write_text(json.dumps({"spent_cny": TOTAL_CNY}) + "\n", encoding="utf-8")
        total_client = DashScopeLLM(model_id="qwen3.7-max-2026-05-20", dry_run=True)
        try:
            asyncio.run(invoke(total_client))
        except RuntimeError as exc:
            results["pre_request_total_block"] = "CNY_BUDGET_PRE_REQUEST_BLOCKED" in str(exc)
        else:
            results["pre_request_total_block"] = False

        state_path.write_text('{"spent_cny": 0.0}\n', encoding="utf-8")
        record_client = DashScopeLLM(model_id="qwen3.7-max-2026-05-20", dry_run=True)
        setattr(record_client, "_articraft_record_spent_cny", PER_RECORD_CNY)
        try:
            asyncio.run(invoke(record_client))
        except RuntimeError as exc:
            results["pre_request_per_record_block"] = "CNY_BUDGET_PRE_REQUEST_BLOCKED" in str(exc)
        else:
            results["pre_request_per_record_block"] = False

        _original_generate_with_tools = missing_usage_transport
        missing_client = DashScopeLLM(model_id="qwen3.7-max-2026-05-20", dry_run=True)
        try:
            asyncio.run(invoke(missing_client))
        except RuntimeError as exc:
            results["usage_missing_fail_closed"] = "CNY_BUDGET_USAGE_MISSING" in str(exc)
        else:
            results["usage_missing_fail_closed"] = False
    finally:
        _original_generate_with_tools = saved_transport
    return results
