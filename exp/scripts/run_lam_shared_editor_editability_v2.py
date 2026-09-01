#!/usr/bin/env python3
"""Run the frozen one-shot shared editor on strict official LAM parents.

Default mode is offline preflight and request-preview generation. Paid provider
execution requires all explicit gates and is never implied by preflight.
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from run_naive_same_llm_editability_v2 import (
    cny_cost,
    create_dashscope_client,
    estimated_prompt_tokens,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
EXP_ROOT = REPO_ROOT / "exp"
DEFAULT_ROOT = EXP_ROOT / "runtime/nano3d_editability_v2/lam_shared_editor"
DEFAULT_MANIFEST = DEFAULT_ROOT / "frozen_parent_manifest.json"
DEFAULT_CONTRACT = DEFAULT_ROOT / "editor_contract.json"
DEFAULT_SMOKE = DEFAULT_ROOT / "offline_parent_smoke/summary.json"
DEFAULT_AUDIT = DEFAULT_ROOT / "category_match_audit.json"
DEFAULT_OUT = DEFAULT_ROOT / "shared_editor_run"
MODEL_SNAPSHOT = "qwen3.7-max-2026-05-20"
PAID_CONFIRMATION = "RUN_FROZEN_LAM_SHARED_EDITOR_COHORT"


def contained(path: Path, *, strict: bool = False) -> Path:
    resolved = path.resolve(strict=strict)
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path outside authorized workspace: {resolved}")
    return resolved


def relative(path: Path) -> str:
    return str(contained(path).relative_to(WORKSPACE_ROOT))


def sha_file(path: Path) -> str:
    return hashlib.sha256(contained(path, strict=True).read_bytes()).hexdigest()


def dump(path: Path, value: Any) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")


def load(path: Path) -> Any:
    return json.loads(contained(path, strict=True).read_text(encoding="utf-8"))


def render(contract: dict[str, Any], item: dict[str, Any], source: str) -> str:
    return contract["user_message_template"].format(
        original_object_prompt=item["caption"].strip(),
        normalized_instruction=item["normalized_instruction"].strip(),
        parent_source_sha256=item["threejs_code_sha256"],
        parent_source_bytes=len(source.encode("utf-8")),
        parent_source=source,
    )


def validate(
    manifest_path: Path,
    contract_path: Path,
    smoke_path: Path,
    expected_manifest_sha256: str,
    model_snapshot: str,
) -> tuple[list[str], dict[str, Any], dict[str, Any], dict[str, Any]]:
    errors: list[str] = []
    manifest = load(manifest_path)
    contract = load(contract_path)
    smoke = load(smoke_path)
    manifest_sha = sha_file(manifest_path)
    if manifest_sha != expected_manifest_sha256.lower():
        errors.append("COHORT_MANIFEST_SHA256_MISMATCH")
    if contract.get("manifest_sha256") != manifest_sha:
        errors.append("EDITOR_CONTRACT_NOT_BOUND_TO_COHORT")
    if model_snapshot != MODEL_SNAPSHOT or contract.get("model_snapshot") != MODEL_SNAPSHOT:
        errors.append("IMMUTABLE_MODEL_SNAPSHOT_MISMATCH")
    if contract.get("provider") != "dashscope":
        errors.append("PROVIDER_MISMATCH")
    frozen = {
        "context_window_tokens": 1_000_000,
        "thinking_level": "high",
        "max_turns": 1,
        "provider_request_limit": 1,
        "max_output_tokens": 65_536,
        "output_safety_tokens": 1_024,
        "max_cost_usd": None,
        "tools": [],
        "automatic_retries": 0,
        "repair_turns": 0,
        "compile_feedback": False,
        "source_truncation": False,
    }
    for key, expected in frozen.items():
        if contract.get(key) != expected:
            errors.append(f"FROZEN_CONTRACT_MISMATCH:{key}")
    items = manifest.get("items", [])
    ids = [item.get("task_id") for item in items]
    if not items or len(set(ids)) != len(items):
        errors.append("STRICT_COHORT_MUST_BE_NONEMPTY_WITH_UNIQUE_TASKS")
    if manifest.get("main_table_eligible") is not False or manifest.get("api_called") is not False:
        errors.append("COHORT_SCOPE_MISMATCH")
    if manifest.get("shared_manifest_binding_status") != "EXACT_FINAL_MANIFEST":
        errors.append("FINAL_SHARED_MANIFEST_BINDING_REQUIRED")
    shared_path = contained(WORKSPACE_ROOT / manifest["shared_task_manifest"], strict=True)
    if sha_file(shared_path) != manifest.get("shared_task_manifest_sha256"):
        errors.append("SHARED_TASK_MANIFEST_SHA256_DRIFT")
    if smoke.get("status") != "PASS" or smoke.get("manifest_sha256") != manifest_sha:
        errors.append("OFFLINE_SOURCE_SMOKE_NOT_PASS_FOR_COHORT")
    if smoke.get("case_count") != len(items) or smoke.get("pass_count") != len(items):
        errors.append("OFFLINE_SOURCE_SMOKE_INCOMPLETE")
    return sorted(set(errors)), manifest, contract, smoke


def build_previews(
    manifest: dict[str, Any], contract: dict[str, Any], smoke: dict[str, Any], out: Path
) -> tuple[list[dict[str, Any]], list[str]]:
    client = create_dashscope_client(dry_run=True, output_dir=out)
    smoke_by_id = {row["task_id"]: row for row in smoke["records"]}
    rows: list[dict[str, Any]] = []
    errors: list[str] = []
    for item in manifest["items"]:
        task_id = item["task_id"]
        source_path = contained(Path(smoke_by_id[task_id]["raw_source"]), strict=True)
        source = source_path.read_text(encoding="utf-8")
        if hashlib.sha256(source.encode("utf-8")).hexdigest() != item["threejs_code_sha256"]:
            errors.append(f"{task_id}:PARENT_SOURCE_SHA256_MISMATCH")
            continue
        user_message = render(contract, item, source)
        preview = client.build_request_preview(
            system_prompt=contract["system_message"],
            messages=[{"role": "user", "content": user_message}],
            tools=[],
        )
        messages = preview.get("messages") if isinstance(preview, dict) else None
        preview_user = next(
            (row.get("content") for row in messages or [] if isinstance(row, dict) and row.get("role") == "user"),
            None,
        )
        full_source = preview_user == user_message and source in str(preview_user)
        output_cap = preview.get("max_tokens") == 65_536
        no_tools = "tools" not in preview or preview.get("tools") == []
        if not full_source:
            errors.append(f"{task_id}:FULL_SOURCE_NOT_IN_PROVIDER_PREVIEW")
        if not output_cap:
            errors.append(f"{task_id}:OUTPUT_CAP_REDUCED_BY_CONTEXT")
        if not no_tools:
            errors.append(f"{task_id}:TOOLS_PRESENT_IN_PROVIDER_PREVIEW")
        preview_path = contained(out / "request_previews" / f"{task_id}.json")
        dump(preview_path, preview)
        prompt_tokens = estimated_prompt_tokens(preview)
        projected = {
            "prompt_tokens": prompt_tokens,
            "cached_tokens": 0,
            "candidates_tokens": 65_536,
            "total_tokens": prompt_tokens + 65_536,
        }
        rows.append(
            {
                "task_id": task_id,
                "object_release_id": item["object_release_id"],
                "parent_source_sha256": item["threejs_code_sha256"],
                "parent_source_utf8_bytes": len(source.encode("utf-8")),
                "request_preview": relative(preview_path),
                "request_preview_sha256": sha_file(preview_path),
                "full_source_present": full_source,
                "output_cap_preserved": output_cap,
                "tools": [],
                "estimated_prompt_tokens_conservative": prompt_tokens,
                "projected_max_usage": projected,
                "projected_max_cost_cny": cny_cost(projected),
                "gold_or_scorer_in_request": False,
            }
        )
    return rows, errors


def assess_js(response: dict[str, Any], task_dir: Path) -> dict[str, Any]:
    content = response.get("content") if isinstance(response.get("content"), str) else ""
    usage = response.get("usage") if isinstance(response.get("usage"), dict) else {}
    finish_reason = str(response.get("finish_reason") or "").lower()
    no_fence = "```" not in content
    export_present = "export function createScene" in content or "export async function createScene" in content
    source_path = contained(task_dir / "edited_source.mjs")
    source_path.write_text(content + ("" if content.endswith("\n") else "\n"), encoding="utf-8")
    checked = subprocess.run(
        ["node", "--check", str(source_path)],
        cwd=contained(task_dir),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    (task_dir / "node_syntax_check.log").write_text(checked.stdout, encoding="utf-8")
    candidate_tokens = usage.get("candidates_tokens")
    cap_hit = isinstance(candidate_tokens, int) and candidate_tokens >= 65_536
    length_finish = finish_reason in {"length", "max_tokens", "max_output_tokens"}
    accepted = bool(content and no_fence and export_present and checked.returncode == 0 and not cap_hit and not length_finish)
    return {
        "status": "OUTPUT_ACCEPTED" if accepted else "OUTPUT_TRUNCATED_OR_INVALID",
        "accepted": accepted,
        "plain_javascript_only": no_fence,
        "create_scene_export_present": export_present,
        "syntax_valid": checked.returncode == 0,
        "finish_reason": finish_reason or None,
        "token_cap_hit": cap_hit,
        "edited_source": relative(source_path),
        "edited_source_sha256": sha_file(source_path),
        "usage": usage,
        "estimated_cost_cny": cny_cost(usage),
    }


async def execute_requests(
    manifest: dict[str, Any], contract: dict[str, Any], smoke: dict[str, Any], out: Path
) -> list[dict[str, Any]]:
    client = create_dashscope_client(dry_run=False, output_dir=out)
    smoke_by_id = {row["task_id"]: row for row in smoke["records"]}
    records: list[dict[str, Any]] = []
    try:
        for item in manifest["items"]:
            task_id = item["task_id"]
            task_dir = contained(out / "tasks" / task_id)
            if task_dir.exists():
                raise RuntimeError(f"{task_id}: output exists; refusing possible duplicate paid request")
            task_dir.mkdir(parents=True, exist_ok=False)
            source = contained(Path(smoke_by_id[task_id]["raw_source"]), strict=True).read_text(encoding="utf-8")
            user_message = render(contract, item, source)
            started = time.monotonic()
            try:
                response = await client.generate_with_tools(
                    contract["system_message"], [{"role": "user", "content": user_message}], []
                )
            except Exception as exc:  # noqa: BLE001
                record = {
                    "task_id": task_id,
                    "status": "PROVIDER_ERROR",
                    "accepted": False,
                    "provider_requests": 1,
                    "error": f"{type(exc).__name__}: {str(exc)[:2000]}",
                    "usage": {},
                    "estimated_cost_cny": cny_cost({}),
                }
                dump(task_dir / "record.json", record)
                records.append(record)
                break
            raw_path = task_dir / "raw_response.json"
            dump(raw_path, response)
            record = {
                "task_id": task_id,
                "object_release_id": item["object_release_id"],
                "provider_requests": 1,
                "turns": 1,
                "tools_exposed": 0,
                "compile_feedback_to_model": False,
                "repair_turns": 0,
                "elapsed_seconds": time.monotonic() - started,
                "raw_response": relative(raw_path),
                "raw_response_sha256": sha_file(raw_path),
                **assess_js(response, task_dir),
            }
            dump(task_dir / "record.json", record)
            records.append(record)
    finally:
        await client.close()
    return records


def totals(records: list[dict[str, Any]]) -> dict[str, Any]:
    keys = ("prompt_tokens", "cached_tokens", "candidates_tokens", "total_tokens")
    tokens = {key: 0 for key in keys}
    cost = 0.0
    for record in records:
        usage = record.get("usage") if isinstance(record.get("usage"), dict) else {}
        for key in keys:
            if isinstance(usage.get(key), int):
                tokens[key] += usage[key]
        estimate = record.get("estimated_cost_cny")
        if isinstance(estimate, dict) and isinstance(estimate.get("total"), (int, float)):
            cost += estimate["total"]
    return {"actual_tokens": tokens, "estimated_cost_cny": round(cost, 8), "usd_cost": None}


def write_report(out: Path, summary: dict[str, Any]) -> None:
    ids = ", ".join(summary["cohort_task_ids"])
    report = f"""# LAM released-source shared-editor probe v2

Status: **{summary['status']}**

This is an external-editor probe over official released LAM Three.js parents,
not a LAM-native editing API and not an 18-item main-table result.

| Evidence | Value |
|---|---:|
| Strict matched subset | {summary['cohort_count']}/18 ({ids}) |
| Official release rows / viable rows | {summary['official_release_rows']} / {summary['official_release_viable_rows']} |
| Official code commit | `{summary['official_code_commit']}` |
| Official dataset revision | `{summary['official_dataset_revision']}` |
| Parent source re-executed | {str(summary['generated_parent_code_executed']).lower()} |
| Three.js -> mesh/URDF smoke | {summary['offline_parent_smoke_status']} |
| Exact provider previews | {len(summary['previews'])}/{summary['cohort_count']} |
| Model snapshot | `{summary['model_snapshot']}` |
| API called | {str(summary['api_called']).lower()} |
| Actual tokens / estimated CNY | {summary['totals']['actual_tokens']['total_tokens']} / {summary['totals']['estimated_cost_cny']} |

The editor is one shot (`max_turns=1`, one provider request per item), with
65,536 maximum output tokens, high thinking, no tools, retries, compile
feedback, LAM checker, or repair. Pricing is frozen at 12 CNY/M uncached input,
2.4 CNY/M cached input, and 36 CNY/M output; no USD conversion is inferred.

Real execution requires `--execute`, the exact cohort SHA, immutable model
snapshot, and the paid-confirmation string. Upstream shared-manifest drift
blocks before any provider call.
"""
    contained(out / "report.md").write_text(report, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--offline-smoke", type=Path, default=DEFAULT_SMOKE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--expected-manifest-sha256", required=True)
    parser.add_argument("--model-snapshot", required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-paid-api")
    args = parser.parse_args()

    manifest_path = contained(args.manifest, strict=True)
    contract_path = contained(args.contract, strict=True)
    smoke_path = contained(args.offline_smoke, strict=True)
    audit_path = contained(DEFAULT_AUDIT, strict=True)
    out = contained(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    audit = load(audit_path)
    blockers, manifest, contract, smoke = validate(
        manifest_path, contract_path, smoke_path, args.expected_manifest_sha256, args.model_snapshot
    )
    if args.execute and args.confirm_paid_api != PAID_CONFIRMATION:
        blockers.append("PAID_API_CONFIRMATION_REQUIRED")

    previews: list[dict[str, Any]] = []
    if not blockers:
        previews, preview_errors = build_previews(manifest, contract, smoke, out)
        blockers.extend(preview_errors)
    records: list[dict[str, Any]] = []
    if args.execute and not blockers:
        records = asyncio.run(execute_requests(manifest, contract, smoke, out))

    if blockers:
        status = "BLOCKED"
    elif not args.execute:
        status = "READY_API_NOT_CALLED"
    elif len(records) == len(manifest["items"]) and all(row.get("accepted") for row in records):
        status = "COMPLETE"
    else:
        status = "COMPLETE_WITH_OUTPUT_FAILURES"
    summary = {
        "schema_version": 2,
        "protocol_id": "nano3d_table5_lam_shared_external_editor_run_v2",
        "status": status,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "probe_kind": "LAM released-source + external shared editor",
        "main_table_eligible": False,
        "cohort_count": len(manifest.get("items", [])),
        "cohort_task_ids": [item["task_id"] for item in manifest.get("items", [])],
        "manifest_sha256": sha_file(manifest_path),
        "expected_manifest_sha256": args.expected_manifest_sha256,
        "shared_task_manifest_sha256": manifest.get("shared_task_manifest_sha256"),
        "official_code_commit": manifest.get("code_commit"),
        "official_dataset_revision": manifest.get("dataset_revision"),
        "official_release_rows": audit.get("official_release_rows"),
        "official_release_viable_rows": audit.get("official_release_viable_rows"),
        "contract_sha256": sha_file(contract_path),
        "model_snapshot": MODEL_SNAPSHOT,
        "network_accessed": bool(args.execute and records),
        "paid_api_called": bool(args.execute and records),
        "api_called": bool(args.execute and records),
        "generated_parent_code_executed": smoke.get("generated_code_executed"),
        "offline_parent_smoke_status": smoke.get("status"),
        "blockers": sorted(set(blockers)),
        "previews": previews,
        "records": records,
        "totals": totals(records),
        "run_policy": {
            "execute_flag_required": True,
            "exact_cohort_sha_required": True,
            "exact_model_snapshot_required": True,
            "paid_confirmation_required": PAID_CONFIRMATION,
            "provider_request_limit_per_task": 1,
            "transport_retries": 0,
            "repair_turns": 0,
        },
    }
    dump(out / "summary.json", summary)
    write_report(out, summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 2 if blockers else 0


if __name__ == "__main__":
    raise SystemExit(main())
