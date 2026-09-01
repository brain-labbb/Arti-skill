#!/usr/bin/env python3
"""Prepare and audit the frozen T2 template-authoring development pilot.

This script intentionally does not invoke a paid model.  It creates executor-neutral,
method-isolated packets, validates returned results, and summarizes resumable runs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
DEFAULT_PROTOCOL = EXP_ROOT / "t2_authoring_pilot" / "protocol.json"
DEFAULT_OUT = EXP_ROOT / "runtime" / "t2_authoring_pilot"
RESULT_SCHEMA = EXP_ROOT / "t2_authoring_pilot" / "schemas" / "authoring_result.schema.json"
RECORD_RE = re.compile(r"\brec_[A-Za-z0-9_-]+\b")
SAFE_ID_RE = re.compile(r"^[A-Za-z0-9_.-]+$")
MODEL_PLACEHOLDER = "TO_BE_FROZEN_BEFORE_PAID_RUN"


class ProtocolError(ValueError):
    """The frozen protocol is internally inconsistent or references missing inputs."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def canonical_json_bytes(payload: Any) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def dump_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        "".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    temporary.replace(path)


def project_path(relative: str) -> Path:
    candidate = (PROJECT_ROOT / relative).resolve()
    try:
        candidate.relative_to(PROJECT_ROOT.resolve())
    except ValueError as exc:
        raise ProtocolError(f"path escapes project root: {relative}") from exc
    return candidate


def git_snapshot(path: Path) -> dict[str, Any]:
    def git(*args: str) -> str | None:
        proc = subprocess.run(
            ["git", "-C", str(path), *args],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        return proc.stdout.strip() if proc.returncode == 0 else None

    head = git("rev-parse", "HEAD")
    status = git("status", "--porcelain", "--untracked-files=no")
    return {
        "path": str(path.resolve()),
        "git_head": head,
        "tracked_worktree_dirty": bool(status) if status is not None else None,
    }


def record_ids(source_map: Path) -> list[str]:
    return sorted(set(RECORD_RE.findall(source_map.read_text(encoding="utf-8"))))


def resolve_record(record_id: str) -> Path | None:
    candidates = (
        PROJECT_ROOT / "arti-template" / "data" / "records" / record_id,
        PROJECT_ROOT / "articraft_data" / "data" / "records" / record_id,
    )
    for candidate in candidates:
        if (candidate / "record.json").is_file():
            return candidate.resolve()
    return None


def validate_protocol(protocol: dict[str, Any]) -> list[dict[str, Any]]:
    required = {
        "schema_version",
        "experiment_id",
        "scope",
        "expected_task_count",
        "expected_methods",
        "repeat_seeds",
        "methods",
        "tasks",
    }
    missing = sorted(required - protocol.keys())
    if missing:
        raise ProtocolError(f"protocol is missing fields: {', '.join(missing)}")
    if protocol["schema_version"] != 1:
        raise ProtocolError("only protocol schema_version=1 is supported")

    tasks = protocol["tasks"]
    methods = protocol["methods"]
    repeats = protocol["repeat_seeds"]
    if len(tasks) != protocol["expected_task_count"]:
        raise ProtocolError(
            f"expected {protocol['expected_task_count']} tasks, found {len(tasks)}"
        )
    if len(methods) != protocol["expected_methods"]:
        raise ProtocolError(
            f"expected {protocol['expected_methods']} methods, found {len(methods)}"
        )
    if len(repeats) != 3 or len(set(repeats)) != len(repeats):
        raise ProtocolError("the pilot requires exactly three unique repeat seeds")

    slugs = [task["slug"] for task in tasks]
    method_ids = [method["method_id"] for method in methods]
    if len(set(slugs)) != len(slugs):
        raise ProtocolError("task slugs must be unique")
    if len(set(method_ids)) != len(method_ids):
        raise ProtocolError("method ids must be unique")
    if Counter(task["complexity"] for task in tasks) != {
        "simple": 4,
        "medium": 4,
        "complex": 4,
    }:
        raise ProtocolError("task cohort must contain 4 simple, 4 medium, and 4 complex tasks")

    shared_paths = protocol.get("shared_context_paths", [])
    for relative in shared_paths:
        if not project_path(relative).exists():
            raise ProtocolError(f"shared context path does not exist: {relative}")

    enriched: list[dict[str, Any]] = []
    for task in tasks:
        paths = {
            key: project_path(task[key])
            for key in ("source_map", "template_design", "hidden_reference_template")
        }
        for key, path in paths.items():
            if not path.is_file():
                raise ProtocolError(f"{task['slug']} has missing {key}: {path}")
        if len(set(paths.values())) != 3:
            raise ProtocolError(f"{task['slug']} has aliased evidence/reference paths")

        ids = record_ids(paths["source_map"])
        if not ids:
            raise ProtocolError(f"{task['slug']} SourceMap contains no record ids")
        resolved = {record_id: resolve_record(record_id) for record_id in ids}
        unresolved = [record_id for record_id, path in resolved.items() if path is None]
        if unresolved:
            preview = ", ".join(unresolved[:5])
            raise ProtocolError(
                f"{task['slug']} has {len(unresolved)} unresolved source records: {preview}"
            )
        enriched.append(
            {
                **task,
                "record_ids": ids,
                "record_paths": [
                    str(resolved[record_id].relative_to(PROJECT_ROOT.resolve()))
                    for record_id in ids
                    if resolved[record_id] is not None
                ],
                "source_map_sha256": sha256_file(paths["source_map"]),
                "template_design_sha256": sha256_file(paths["template_design"]),
                "hidden_reference_template_sha256": sha256_file(
                    paths["hidden_reference_template"]
                ),
            }
        )
    return enriched


def deterministic_seed(repeat_seed: int, slug: str, method_id: str) -> int:
    raw = f"{repeat_seed}:{slug}:{method_id}".encode("utf-8")
    return int.from_bytes(hashlib.sha256(raw).digest()[:4], "big") & 0x7FFFFFFF


def run_key(slug: str, method_id: str, repeat_index: int) -> str:
    return f"{slug}__{method_id}__r{repeat_index}"


def packet_for(
    protocol: dict[str, Any],
    task: dict[str, Any],
    method: dict[str, Any],
    repeat_index: int,
    repeat_seed: int,
    model_id: str,
) -> dict[str, Any]:
    key = run_key(task["slug"], method["method_id"], repeat_index)
    evidence_paths = list(task["record_paths"])
    forbidden = [task["hidden_reference_template"]]
    if method["expose_source_map"]:
        evidence_paths.append(task["source_map"])
    else:
        forbidden.append(task["source_map"])
    if method["expose_template_design"]:
        evidence_paths.append(task["template_design"])
    else:
        forbidden.append(task["template_design"])

    return {
        "schema_version": 1,
        "experiment_id": protocol["experiment_id"],
        "run_key": key,
        "task_slug": task["slug"],
        "complexity": task["complexity"],
        "method_id": method["method_id"],
        "method_display_name": method["display_name"],
        "repeat_index": repeat_index,
        "repeat_seed": repeat_seed,
        "authoring_seed": deterministic_seed(repeat_seed, task["slug"], method["method_id"]),
        "authoring_model": model_id,
        "budget": protocol["authoring_budget"],
        "task_instruction": (
            "Author one self-contained Articraft procedural template for task_slug using only "
            "allowed_inputs. Follow the shared authoring contract, do not copy or inspect any "
            "forbidden input, and emit the template and structured result at the declared paths."
        ),
        "allowed_inputs": {
            "shared_context_paths": protocol["shared_context_paths"],
            "task_evidence_paths": evidence_paths,
            "source_record_ids": task["record_ids"],
        },
        "forbidden_inputs": {
            "paths": sorted(forbidden),
            "rules": [
                "Do not read the hidden reference template or its historical outputs.",
                "Do not read evidence withheld by this method arm.",
                "Do not read any other arm's prompt, output, trace, or evaluator report.",
                "The executor must enforce the allowlist in an isolated worktree or sandbox.",
            ],
        },
        "deliverable": {
            "template_relative_path": f"output/{task['slug']}.py",
            "result_relative_path": "authoring_result.json",
            "result_schema": str(RESULT_SCHEMA.relative_to(PROJECT_ROOT)),
        },
        "evaluation_contract": protocol["evaluation_contract"],
        "claim_boundary": protocol["claim_boundary"],
    }


def prepare(protocol_path: Path, output_root: Path, run_id: str, model_id: str) -> Path:
    if not SAFE_ID_RE.fullmatch(run_id):
        raise ProtocolError("run id may contain only letters, digits, dot, underscore, and hyphen")
    protocol = load_json(protocol_path)
    tasks = validate_protocol(protocol)
    protocol_hash = sha256_bytes(canonical_json_bytes(protocol))
    output_dir = output_root / run_id
    manifest_path = output_dir / "experiment_manifest.json"

    if manifest_path.exists():
        existing = load_json(manifest_path)
        if existing.get("protocol_sha256") != protocol_hash:
            raise ProtocolError(f"resume refused: protocol changed for existing run {output_dir}")
        if existing.get("authoring_model") != model_id:
            raise ProtocolError(f"resume refused: authoring model changed for existing run {output_dir}")
        print(f"resume: verified existing experiment manifest at {manifest_path}")
    else:
        dump_json(
            manifest_path,
            {
                "schema_version": 1,
                "experiment_id": protocol["experiment_id"],
                "run_id": run_id,
                "created_at": utc_now(),
                "scope": protocol["scope"],
                "claim_boundary": protocol["claim_boundary"],
                "protocol_path": str(protocol_path.resolve()),
                "protocol_sha256": protocol_hash,
                "runner_path": str(Path(__file__).resolve()),
                "runner_sha256": sha256_file(Path(__file__)),
                "result_schema_sha256": sha256_file(RESULT_SCHEMA),
                "authoring_model": model_id,
                "execution_ready": model_id != MODEL_PLACEHOLDER,
                "repositories": {
                    "project": git_snapshot(PROJECT_ROOT),
                    "arti_template": git_snapshot(PROJECT_ROOT / "arti-template"),
                    "articraft_data": git_snapshot(PROJECT_ROOT / "articraft_data"),
                },
            },
        )

    task_rows = [
        {
            key: value
            for key, value in task.items()
            if key
            in {
                "slug",
                "complexity",
                "source_map",
                "template_design",
                "hidden_reference_template",
                "record_ids",
                "record_paths",
                "source_map_sha256",
                "template_design_sha256",
                "hidden_reference_template_sha256",
            }
        }
        for task in tasks
    ]
    task_manifest = output_dir / "task_manifest.jsonl"
    if task_manifest.exists():
        existing_hash = sha256_file(task_manifest)
        expected_hash = sha256_bytes(
            "".join(
                json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in task_rows
            ).encode("utf-8")
        )
        if existing_hash != expected_hash:
            raise ProtocolError("resume refused: task manifest differs from frozen inputs")
    else:
        dump_jsonl(task_manifest, task_rows)

    run_rows: list[dict[str, Any]] = []
    for repeat_index, repeat_seed in enumerate(protocol["repeat_seeds"]):
        for task in tasks:
            for method in protocol["methods"]:
                packet = packet_for(
                    protocol, task, method, repeat_index, repeat_seed, model_id
                )
                key = packet["run_key"]
                packet_path = output_dir / "runs" / key / "packet.json"
                if packet_path.exists():
                    if load_json(packet_path) != packet:
                        raise ProtocolError(f"resume refused: packet changed: {key}")
                else:
                    dump_json(packet_path, packet)
                run_rows.append(
                    {
                        "run_key": key,
                        "task_slug": task["slug"],
                        "complexity": task["complexity"],
                        "method_id": method["method_id"],
                        "repeat_index": repeat_index,
                        "repeat_seed": repeat_seed,
                        "authoring_seed": packet["authoring_seed"],
                        "packet_path": str(packet_path.relative_to(output_dir)),
                        "result_path": str(
                            (packet_path.parent / "authoring_result.json").relative_to(output_dir)
                        ),
                    }
                )
    run_manifest = output_dir / "run_manifest.jsonl"
    expected_run_text = "".join(
        json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in run_rows
    )
    if run_manifest.exists():
        if run_manifest.read_text(encoding="utf-8") != expected_run_text:
            raise ProtocolError("resume refused: run manifest differs from frozen matrix")
    else:
        dump_jsonl(run_manifest, run_rows)

    dump_json(output_dir / "status.json", summarize(output_dir, write=False))
    print(
        f"prepared {len(run_rows)} runs ({len(tasks)} tasks x {len(protocol['methods'])} methods "
        f"x {len(protocol['repeat_seeds'])} repeats) at {output_dir}"
    )
    if model_id == MODEL_PLACEHOLDER:
        print("execution_ready=false: freeze --model-id before any paid run")
    return output_dir


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def validate_result(payload: dict[str, Any], expected: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    required = {
        "schema_version",
        "run_key",
        "task_slug",
        "method_id",
        "repeat_index",
        "status",
        "backend",
        "metrics",
        "output",
    }
    missing = sorted(required - payload.keys())
    if missing:
        return [f"missing fields: {', '.join(missing)}"]
    if payload["schema_version"] != 1:
        errors.append("schema_version must be 1")
    for key in ("run_key", "task_slug", "method_id", "repeat_index"):
        if payload.get(key) != expected.get(key):
            errors.append(f"{key} does not match run manifest")
    if payload["status"] not in {"completed", "failed", "aborted"}:
        errors.append("status must be completed, failed, or aborted")

    backend = payload.get("backend")
    if not isinstance(backend, dict) or not backend.get("provider") or not backend.get("model"):
        errors.append("backend must include non-empty provider and model")
    metrics = payload.get("metrics")
    metric_fields = {
        "first_shot_pass",
        "final_success",
        "artifact_saved",
        "repair_turns",
        "human_intervention",
        "wall_seconds",
        "input_tokens",
        "output_tokens",
        "api_cost_usd",
    }
    if not isinstance(metrics, dict):
        errors.append("metrics must be an object")
    else:
        metric_missing = sorted(metric_fields - metrics.keys())
        if metric_missing:
            errors.append(f"metrics missing fields: {', '.join(metric_missing)}")
        if not isinstance(metrics.get("repair_turns"), int) or metrics.get("repair_turns", -1) < 0:
            errors.append("repair_turns must be a non-negative integer")
        if not isinstance(metrics.get("human_intervention"), bool):
            errors.append("human_intervention must be boolean")
        for key in ("first_shot_pass", "final_success", "artifact_saved"):
            if metrics.get(key) is not None and not isinstance(metrics.get(key), bool):
                errors.append(f"{key} must be boolean or null")
        for key in ("wall_seconds", "input_tokens", "output_tokens"):
            value = metrics.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0:
                errors.append(f"{key} must be non-negative")
        cost = metrics.get("api_cost_usd")
        if cost is not None and (
            isinstance(cost, bool) or not isinstance(cost, (int, float)) or cost < 0
        ):
            errors.append("api_cost_usd must be non-negative or null")
    output = payload.get("output")
    if not isinstance(output, dict):
        errors.append("output must be an object")
    else:
        output_missing = sorted(
            {"template_path", "template_sha256", "evaluator_report_path"} - output.keys()
        )
        if output_missing:
            errors.append(f"output missing fields: {', '.join(output_missing)}")
        digest = output.get("template_sha256")
        if digest is not None and not re.fullmatch(r"[0-9a-f]{64}", str(digest)):
            errors.append("template_sha256 must be a lowercase SHA-256 digest or null")
    return errors


def summarize(output_dir: Path, write: bool = True) -> dict[str, Any]:
    manifest = output_dir / "run_manifest.jsonl"
    if not manifest.is_file():
        raise ProtocolError(f"missing run manifest: {manifest}")
    rows = read_jsonl(manifest)
    counts = Counter()
    invalid: list[dict[str, Any]] = []
    valid_results: list[dict[str, Any]] = []
    for row in rows:
        result_path = output_dir / row["result_path"]
        if not result_path.is_file():
            counts["pending"] += 1
            continue
        try:
            payload = load_json(result_path)
        except (OSError, json.JSONDecodeError) as exc:
            counts["invalid"] += 1
            invalid.append({"run_key": row["run_key"], "errors": [str(exc)]})
            continue
        errors = validate_result(payload, row)
        if errors:
            counts["invalid"] += 1
            invalid.append({"run_key": row["run_key"], "errors": errors})
            continue
        counts[payload["status"]] += 1
        valid_results.append(payload)

    completed = [row for row in valid_results if row["status"] == "completed"]
    by_method: dict[str, dict[str, Any]] = {}
    for method_id in sorted({row["method_id"] for row in rows}):
        method_results = [row for row in completed if row["method_id"] == method_id]
        method_metrics = [row["metrics"] for row in method_results]
        by_method[method_id] = {
            "completed": len(method_results),
            "first_shot_pass": sum(metric["first_shot_pass"] is True for metric in method_metrics),
            "final_success": sum(metric["final_success"] is True for metric in method_metrics),
            "human_intervention": sum(
                metric["human_intervention"] is True for metric in method_metrics
            ),
            "total_input_tokens": sum(metric["input_tokens"] for metric in method_metrics),
            "total_output_tokens": sum(metric["output_tokens"] for metric in method_metrics),
            "total_api_cost_usd": (
                sum(metric["api_cost_usd"] for metric in method_metrics)
                if method_metrics and all(metric["api_cost_usd"] is not None for metric in method_metrics)
                else None
            ),
        }
    summary = {
        "schema_version": 1,
        "updated_at": utc_now(),
        "output_dir": str(output_dir.resolve()),
        "expected_runs": len(rows),
        "status_counts": {
            key: counts.get(key, 0)
            for key in ("pending", "completed", "failed", "aborted", "invalid")
        },
        "by_method": by_method,
        "invalid_results": invalid,
    }
    if write:
        dump_json(output_dir / "status.json", summary)
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    prepare_parser = subparsers.add_parser("prepare", help="validate and materialize run packets")
    prepare_parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    prepare_parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    prepare_parser.add_argument("--run-id", default="dev_v1")
    prepare_parser.add_argument("--model-id", default=MODEL_PLACEHOLDER)

    status_parser = subparsers.add_parser("status", help="validate results and update status.json")
    status_parser.add_argument("run_dir", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.command == "prepare":
            prepare(args.protocol.resolve(), args.out.resolve(), args.run_id, args.model_id)
            return 0
        summary = summarize(args.run_dir.resolve())
        print(json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True))
        return 1 if summary["status_counts"]["invalid"] else 0
    except (OSError, json.JSONDecodeError, ProtocolError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
