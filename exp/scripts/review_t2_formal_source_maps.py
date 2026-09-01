#!/usr/bin/env python3
"""Review frozen T2 SourceMaps one category at a time with durable Codex logs."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
PREPARATION = EXP_ROOT / "runtime" / "t2_formal_v1" / "preparation"
MANIFEST = PREPARATION / "formal_source_manifest.json"
ARTICRAFT = PROJECT_ROOT / "arti-template" / ".venv" / "bin" / "articraft"
CODEX = Path("/mnt/zsn/miniconda3/bin/codex")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def protected_hashes(target: Path) -> dict[str, str]:
    roots = (PREPARATION / "records", PREPARATION / "source_maps")
    return {
        str(path.relative_to(PREPARATION)): sha256(path)
        for root in roots
        for path in sorted(root.rglob("*"))
        if path.is_file() and path != target
    }


def checker(slug: str, *, capture: bool = True) -> subprocess.CompletedProcess[str]:
    command = [
        str(ARTICRAFT),
        "template",
        "source-map-check",
        "--source-map",
        str(PREPARATION / "source_maps" / f"{slug}.md"),
        "--records-root",
        str(PREPARATION / "records"),
    ]
    return subprocess.run(
        command,
        cwd=PROJECT_ROOT / "arti-template",
        check=False,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
    )


def is_pass(slug: str) -> bool:
    result = checker(slug)
    if result.returncode != 0:
        return False
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError:
        return False
    return payload.get("verdict") == "pass" and not payload.get("problems")


def prompt_for(slug: str, record_count: int) -> str:
    return f"""
You are preparing a formal, leakage-controlled authoring benchmark. Work only on
`source_maps/{slug}.md` in the current directory. Do not modify records, other SourceMaps,
designs, templates, scripts, or manifests. Do not spawn subagents.

Read all {record_count} synced records for `{slug}`: each prompt and complete active
`model.py`. Replace every pending SourceMap scaffold field with a genuine source review.
Preserve every sync_record exactly. For every record add one reviewed decision and a
specific note. Build compact, causally meaningful candidate slots: candidates in one slot
must be valid alternatives; topology-coupled structure must not be split into falsely
independent slots. Include auxiliary mechanisms only when independently composable.
Every used candidate needs exact, verified `model.py:Lx-Ly` spans covering its defining
geometry and articulation, a distinction, and concrete evidence. Reject duplicates and
non-structural variants explicitly. Set a precise category scope.

Run the official checker until it passes with zero problems:
`/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/articraft template source-map-check
 --source-map {PREPARATION}/source_maps/{slug}.md
 --records-root {PREPARATION}/records`

Stop after the target SourceMap passes. In the final response report reviewed/used counts,
slot/candidate counts, checker verdict, and confirm that no other file was modified.
""".strip()


def review(slug: str, record_count: int, model: str, effort: str) -> dict[str, Any]:
    target = PREPARATION / "source_maps" / f"{slug}.md"
    logs = PREPARATION / "llm_logs"
    logs.mkdir(parents=True, exist_ok=True)
    before = protected_hashes(target)
    started = datetime.now(timezone.utc).isoformat()
    events_path = logs / f"{slug}_source_map_events.jsonl"
    stderr_path = logs / f"{slug}_source_map_stderr.txt"
    final_path = logs / f"{slug}_source_map_last.txt"

    command = [
        str(CODEX),
        "exec",
        "--json",
        "--model",
        model,
        "-c",
        f"model_reasoning_effort='{effort}'",
        "--dangerously-bypass-approvals-and-sandbox",
        "--skip-git-repo-check",
        "-o",
        str(final_path),
        prompt_for(slug, record_count),
    ]
    with events_path.open("w", encoding="utf-8") as stdout, stderr_path.open(
        "w", encoding="utf-8"
    ) as stderr:
        result = subprocess.run(
            command,
            cwd=PREPARATION,
            check=False,
            text=True,
            stdout=stdout,
            stderr=stderr,
        )

    after = protected_hashes(target)
    changed_protected = sorted(
        path for path in set(before) | set(after) if before.get(path) != after.get(path)
    )
    check_result = checker(slug)
    checker_path = logs / f"{slug}_source_map_checker.json"
    checker_path.write_text(check_result.stdout or "", encoding="utf-8")
    passed = check_result.returncode == 0
    try:
        checker_payload = json.loads(check_result.stdout)
        passed = passed and checker_payload.get("verdict") == "pass"
    except json.JSONDecodeError:
        checker_payload = {"raw_output": check_result.stdout}
        passed = False

    summary = {
        "slug": slug,
        "started_at": started,
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "model": model,
        "reasoning_effort": effort,
        "codex_exit_code": result.returncode,
        "checker_exit_code": check_result.returncode,
        "checker_verdict": checker_payload.get("verdict"),
        "candidate_count": checker_payload.get("candidate_count"),
        "checker_problem_count": len(checker_payload.get("problems", [])),
        "protected_files_changed": changed_protected,
        "passed": passed and result.returncode == 0 and not changed_protected,
        "source_map_sha256": sha256(target),
        "events": str(events_path.relative_to(PROJECT_ROOT)),
        "checker_output": str(checker_path.relative_to(PROJECT_ROOT)),
    }
    summary_path = logs / f"{slug}_source_map_run.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--slug", action="append", help="review only this slug; repeatable")
    parser.add_argument("--model", default="gpt-5.6-sol")
    parser.add_argument("--effort", choices=("low", "medium", "high"), default="high")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    manifest = load_json(MANIFEST)
    tasks = manifest["tasks"]
    requested = set(args.slug or [])
    unknown = requested - {task["slug"] for task in tasks}
    if unknown:
        raise SystemExit(f"unknown slugs: {sorted(unknown)}")

    selected = [task for task in tasks if not requested or task["slug"] in requested]
    for task in selected:
        slug = task["slug"]
        if not args.force and is_pass(slug):
            print(json.dumps({"slug": slug, "status": "already_passed"}), flush=True)
            continue
        print(json.dumps({"slug": slug, "status": "starting"}), flush=True)
        summary = review(slug, int(task["selected_record_count"]), args.model, args.effort)
        print(json.dumps(summary, ensure_ascii=False, sort_keys=True), flush=True)
        if not summary["passed"]:
            print(f"stopping after failed review: {slug}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
