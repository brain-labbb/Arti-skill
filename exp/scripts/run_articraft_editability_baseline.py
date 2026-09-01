#!/usr/bin/env python3
"""Audit and recompile the provenance-valid Articraft Table 5 edit cohort.

This runner never calls an LLM provider. It copies frozen parent/child records
into an experiment-local repository snapshot, invokes the official Articraft
compiler there, and reports deterministic structural proxies. Human blind
review fields remain N/A.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
EXP_ROOT = SCRIPT_DIR.parent
PROJECT_ROOT = EXP_ROOT.parent
WORKSPACE_ROOT = Path("/mnt/zsn/lyb").resolve()
ARTICRAFT_ROOT = (PROJECT_ROOT / "articraft_data").resolve()
MANIFEST_PATH = (
    EXP_ROOT / "reference" / "editability_baselines" / "articraft_manifest.json"
).resolve()
DEFAULT_OUT = (EXP_ROOT / "runtime" / "nano3d_editability_baselines" / "articraft").resolve()


def within_workspace(path: Path, *, allow_missing: bool = False) -> Path:
    resolved = path.resolve(strict=not allow_missing)
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"Path escapes workspace boundary: {path} -> {resolved}")
    return resolved


def dump_json(path: Path, payload: Any) -> None:
    within_workspace(path.parent, allow_missing=True).mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8"
    )


def load_json(path: Path) -> Any:
    return json.loads(within_workspace(path).read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(within_workspace(path).read_bytes()).hexdigest()


def record_revision(
    repo_root: Path, record_id: str, expected_revision: str | None = None
) -> tuple[Path, str, dict[str, Any]]:
    record_dir = within_workspace(repo_root / "data" / "records" / record_id)
    record = load_json(record_dir / "record.json")
    revision_id = str(record.get("active_revision_id") or "")
    if expected_revision and revision_id != expected_revision:
        raise ValueError(
            f"{record_id}: active revision {revision_id!r} != frozen {expected_revision!r}"
        )
    revision_dir = within_workspace(record_dir / "revisions" / revision_id)
    return revision_dir, revision_id, record


def preflight_task(task: dict[str, Any]) -> dict[str, Any]:
    source_map = within_workspace(
        PROJECT_ROOT
        / "arti-template"
        / "articraft_template_authoring"
        / "source_maps"
        / f"{task['slug']}.md"
    )
    row: dict[str, Any] = {
        "task_id": task["task_id"],
        "status": task["status"],
        "source_map": str(source_map),
        "source_map_sha256": sha256(source_map),
        "checks": {"source_map": True},
    }
    if task["status"] != "exact_historical_pair":
        evidence_id = task.get("evidence_record_id")
        row["blocked_reason"] = task["blocked_reason"]
        if evidence_id:
            evidence_dir = within_workspace(ARTICRAFT_ROOT / "data" / "records" / evidence_id)
            row["evidence_record_available"] = evidence_dir.is_dir()
        return row

    parent_dir, parent_revision, parent_record = record_revision(
        ARTICRAFT_ROOT, task["parent_record_id"], task["parent_revision_id"]
    )
    edited_dir, edited_revision, edited_record = record_revision(
        ARTICRAFT_ROOT, task["edited_record_id"], task["edited_revision_id"]
    )
    parent_model = within_workspace(parent_dir / "model.py")
    edited_model = within_workspace(edited_dir / "model.py")
    edited_prompt = within_workspace(edited_dir / "prompt.txt")
    provenance_path = within_workspace(edited_dir / "provenance.json")
    provenance = load_json(provenance_path)
    lineage = edited_record.get("lineage") if isinstance(edited_record.get("lineage"), dict) else {}
    generation = (
        provenance.get("generation") if isinstance(provenance.get("generation"), dict) else {}
    )
    run_summary = (
        provenance.get("run_summary") if isinstance(provenance.get("run_summary"), dict) else {}
    )
    checks = {
        "parent_model": parent_model.is_file(),
        "edited_model": edited_model.is_file(),
        "edited_prompt": edited_prompt.is_file(),
        "provenance": provenance_path.is_file(),
        "direct_parent": lineage.get("parent_record_id") == task["parent_record_id"],
        "parent_revision": lineage.get("parent_revision_id") == parent_revision,
        "copy_edit": lineage.get("edit_mode") == "copy",
        "provider": bool(generation.get("provider")),
        "model_id": bool(generation.get("model_id")),
        "run_success": run_summary.get("final_status") == "success",
    }
    row.update(
        {
            "checks": checks,
            "ready": all(checks.values()),
            "parent_record_id": task["parent_record_id"],
            "parent_revision_id": parent_revision,
            "parent_model_sha256": sha256(parent_model),
            "edited_record_id": task["edited_record_id"],
            "edited_revision_id": edited_revision,
            "edited_model_sha256": sha256(edited_model),
            "historical_prompt": edited_prompt.read_text(encoding="utf-8").strip(),
            "requested_prompt": task["instruction"],
            "prompt_relation": task["prompt_relation"],
            "provider": generation.get("provider"),
            "model_id": generation.get("model_id"),
            "thinking_level": generation.get("thinking_level"),
            "max_turns": generation.get("max_turns"),
            "historical_max_cost_usd": generation.get("max_cost_usd"),
            "historical_cost_usd": None,
            "historical_cost_note": "No USD total is stored in the frozen revision provenance.",
            "historical_run_summary": run_summary,
            "parent_record_sha256": sha256(within_workspace(parent_dir.parents[1] / "record.json")),
            "edited_record_sha256": sha256(within_workspace(edited_dir.parents[1] / "record.json")),
        }
    )
    _ = parent_record
    return row


def copy_record(snapshot: Path, record_id: str) -> None:
    source = within_workspace(ARTICRAFT_ROOT / "data" / "records" / record_id)
    target = within_workspace(snapshot / "data" / "records" / record_id, allow_missing=True)
    if target.exists():
        return
    shutil.copytree(source, target, symlinks=False)


def compile_record(
    snapshot: Path, record_id: str, logs_dir: Path, *, timeout_seconds: int = 600
) -> dict[str, Any]:
    log_path = within_workspace(logs_dir / f"{record_id}.log", allow_missing=True)
    command = [
        "uv",
        "run",
        "articraft",
        "compile",
        "--repo-root",
        str(snapshot),
        "--target",
        "full",
        "--validate",
        record_id,
    ]
    try:
        completed = subprocess.run(
            command,
            cwd=ARTICRAFT_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
            timeout=timeout_seconds,
            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
        )
        returncode = completed.returncode
        output = completed.stdout
    except subprocess.TimeoutExpired as exc:
        returncode = 124
        output = (exc.stdout or "") + f"\nTIMEOUT after {timeout_seconds}s\n"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(output, encoding="utf-8")
    materialized = within_workspace(
        snapshot / "data" / "cache" / "record_materialization" / record_id,
        allow_missing=True,
    )
    return {
        "record_id": record_id,
        "command": command,
        "returncode": returncode,
        "timeout_seconds": timeout_seconds,
        "log": str(log_path),
        "materialized": str(materialized),
        "urdf": str(materialized / "model.urdf"),
        "compile_report": str(materialized / "compile_report.json"),
    }


def urdf_graph(path: Path) -> dict[str, Any]:
    root = ET.parse(within_workspace(path)).getroot()
    links = {node.attrib.get("name", "") for node in root.findall("link")}
    edges: set[tuple[str, str, str]] = set()
    children: dict[str, set[str]] = {name: set() for name in links}
    child_names: set[str] = set()
    valid_refs = True
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            valid_refs = False
            continue
        p = parent.attrib.get("link", "")
        c = child.attrib.get("link", "")
        kind = joint.attrib.get("type", "")
        valid_refs = valid_refs and p in links and c in links
        edges.add((p, c, kind))
        children.setdefault(p, set()).add(c)
        child_names.add(c)
    roots = links - child_names
    reached: set[str] = set()
    pending = list(roots)
    while pending:
        node = pending.pop()
        if node in reached:
            continue
        reached.add(node)
        pending.extend(children.get(node, ()))
    return {
        "links": links,
        "edges": edges,
        "hierarchy_connected": valid_refs and len(roots) == 1 and reached == links,
        "root_count": len(roots),
    }


def token_hits(names: set[str], tokens: list[str]) -> set[str]:
    lowered = tuple(token.lower() for token in tokens)
    return {name for name in names if any(token in name.lower() for token in lowered)}


def model_mentions(path: Path, tokens: list[str]) -> int:
    text = within_workspace(path).read_text(encoding="utf-8").lower()
    return sum(text.count(token.lower()) for token in tokens)


def score_pair(
    snapshot: Path, task: dict[str, Any], compiles: dict[str, dict[str, Any]]
) -> dict[str, Any]:
    parent_id = task["parent_record_id"]
    edited_id = task["edited_record_id"]
    parent_compile = compiles[parent_id]
    edited_compile = compiles[edited_id]
    if parent_compile["returncode"] != 0 or edited_compile["returncode"] != 0:
        return {
            "task_id": task["task_id"],
            "status": "compile_failed",
            "parent_compile": parent_compile,
            "edited_compile": edited_compile,
            "human_blind_review": "N/A",
        }
    parent_graph = urdf_graph(Path(parent_compile["urdf"]))
    edited_graph = urdf_graph(Path(edited_compile["urdf"]))
    tokens = task["target_tokens"]
    parent_hits = token_hits(parent_graph["links"], tokens)
    edited_hits = token_hits(edited_graph["links"], tokens)
    parent_non_target = parent_graph["links"] - parent_hits
    edited_non_target = edited_graph["links"] - edited_hits
    preservation = (
        len(parent_non_target & edited_non_target) / len(parent_non_target)
        if parent_non_target
        else 1.0
    )
    parent_revision, _, _ = record_revision(snapshot, parent_id)
    edited_revision, _, _ = record_revision(snapshot, edited_id)
    parent_mentions = model_mentions(parent_revision / "model.py", tokens)
    edited_mentions = model_mentions(edited_revision / "model.py", tokens)
    source_changed = sha256(parent_revision / "model.py") != sha256(edited_revision / "model.py")
    target_proxy = source_changed and (
        len(edited_hits) > len(parent_hits) or edited_mentions > parent_mentions
    )
    return {
        "task_id": task["task_id"],
        "status": "compiled",
        "parent_record_id": parent_id,
        "edited_record_id": edited_id,
        "target_fulfilled_proxy": target_proxy,
        "target_proxy_kind": "lexical node/model delta; not human semantic adjudication",
        "target_node_hits_base": sorted(parent_hits),
        "target_node_hits_edited": sorted(edited_hits),
        "target_model_mentions_base": parent_mentions,
        "target_model_mentions_edited": edited_mentions,
        "source_changed": source_changed,
        "non_target_structural_preservation_proxy": preservation,
        "locality_proxy_pass": preservation >= 0.5,
        "hierarchy_proxy_pass": parent_graph["hierarchy_connected"]
        and edited_graph["hierarchy_connected"],
        "base_link_count": len(parent_graph["links"]),
        "edited_link_count": len(edited_graph["links"]),
        "base_edge_count": len(parent_graph["edges"]),
        "edited_edge_count": len(edited_graph["edges"]),
        "qc_compile_pass": True,
        "historical_cost_usd": None,
        "new_api_cost_usd": 0.0,
        "human_blind_review": "N/A",
        "human_target_fulfilled": "N/A",
        "human_non_target_preserved": "N/A",
        "human_locality": "N/A",
        "parent_compile": parent_compile,
        "edited_compile": edited_compile,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=MANIFEST_PATH)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()
    manifest_path = within_workspace(args.manifest)
    out = within_workspace(args.out, allow_missing=True)
    if out != DEFAULT_OUT:
        raise ValueError(f"Output must be the frozen Articraft runtime path: {DEFAULT_OUT}")
    out.mkdir(parents=True, exist_ok=True)
    manifest = load_json(manifest_path)
    rows = [preflight_task(task) for task in manifest["tasks"]]
    exact = [row for row in rows if row["status"] == "exact_historical_pair"]
    blocked = [row for row in rows if row["status"] == "blocked"]
    preflight = {
        "schema_version": 1,
        "protocol_task_count": len(rows),
        "exact_historical_pair_count": len(exact),
        "blocked_count": len(blocked),
        "common_18_parent_gate_pass": len(exact) == len(rows),
        "new_api_requests": False,
        "new_api_cost_usd": 0.0,
        "rows": rows,
    }
    dump_json(out / "preflight.json", preflight)
    if args.preflight_only or not args.run:
        print(json.dumps({k: preflight[k] for k in preflight if k != "rows"}, indent=2))
        return 0
    if not all(row.get("ready") for row in exact):
        print("Exact historical cohort preflight failed; refusing to compile.", file=sys.stderr)
        return 2

    snapshot = within_workspace(out / "repo_snapshot", allow_missing=True)
    (snapshot / "data" / "records").mkdir(parents=True, exist_ok=True)
    (snapshot / "data" / "cache").mkdir(parents=True, exist_ok=True)
    record_ids = sorted(
        {
            record_id
            for task in manifest["tasks"]
            if task["status"] == "exact_historical_pair"
            for record_id in (task["parent_record_id"], task["edited_record_id"])
        }
    )
    for record_id in record_ids:
        copy_record(snapshot, record_id)
    logs_dir = within_workspace(out / "compile_logs", allow_missing=True)
    logs_dir.mkdir(parents=True, exist_ok=True)
    compiles = {
        record_id: compile_record(snapshot, record_id, logs_dir) for record_id in record_ids
    }
    results = [
        score_pair(snapshot, task, compiles)
        for task in manifest["tasks"]
        if task["status"] == "exact_historical_pair"
    ]
    compile_pass = sum(row["status"] == "compiled" for row in results)
    summary = {
        "schema_version": 1,
        "cohort_kind": manifest["cohort_kind"],
        "protocol_task_count": len(manifest["tasks"]),
        "evaluated_pair_count": len(results),
        "blocked_count": len(blocked),
        "compile_pair_pass_count": compile_pass,
        "target_proxy_pass_count": sum(
            row.get("target_fulfilled_proxy") is True for row in results
        ),
        "locality_proxy_pass_count": sum(row.get("locality_proxy_pass") is True for row in results),
        "hierarchy_proxy_pass_count": sum(
            row.get("hierarchy_proxy_pass") is True for row in results
        ),
        "human_blind_review": "N/A",
        "new_api_requests": False,
        "new_api_cost_usd": 0.0,
        "historical_cost_usd": None,
        "historical_cost_note": "Frozen provenance records provider/model/turns but not USD totals.",
        "results": results,
        "blocked": blocked,
    }
    dump_json(out / "results.json", summary)
    print(json.dumps({k: summary[k] for k in summary if k not in {"results", "blocked"}}, indent=2))
    return 0 if compile_pass == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
