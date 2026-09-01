#!/usr/bin/env python3
"""Add explicit expected-edge labels to one completed blinded T3 review."""

from __future__ import annotations

import argparse
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
ROOT = EXP_ROOT / "runtime/t3_formal_v1/semantic_review"
CLAUDE = Path("/mnt/zsn/miniconda3/bin/claude")
CODEX = Path("/mnt/zsn/miniconda3/bin/codex")


def validate(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    problems: list[str] = []
    for panel in payload.get("panels", []):
        panel_id = panel.get("panel_id", "unknown")
        fields = panel.get("review_fields", {})
        expected = fields.get("expected_parent_child_edges")
        if not isinstance(expected, list):
            problems.append(f"{panel_id}: expected_parent_child_edges missing")
            continue
        emitted = {tuple(edge) for edge in panel.get("emitted_parent_child_edges", [])}
        names = set(panel.get("emitted_parts", []))
        for row in expected:
            required = {
                "parent_role",
                "child_role",
                "matched_parent",
                "matched_child",
                "matched_emitted_edge",
                "notes",
            }
            if set(row) != required:
                problems.append(f"{panel_id}: malformed expected edge keys")
                continue
            parent, child = row["matched_parent"], row["matched_child"]
            if parent is not None and parent not in names:
                problems.append(f"{panel_id}: unknown matched parent {parent}")
            if child is not None and child not in names:
                problems.append(f"{panel_id}: unknown matched child {child}")
            should_match = parent is not None and child is not None and (parent, child) in emitted
            if row["matched_emitted_edge"] is not should_match:
                problems.append(f"{panel_id}: inconsistent matched edge bool")
    return {"valid": not problems, "problems": problems, "panel_count": len(payload.get("panels", []))}


def prompt(reviewer: str, path: Path) -> str:
    return f"""
Continue only your own blinded T3 review in `{path.name}`. Do not read the other
reviewer, source templates, source code, prior scores, or experiment summaries.
Re-inspect the four-view render for each of all 33 panels and use your own existing
expected roles and emitted-edge judgments.

Edit only `{path.name}`. Under every panel's `review_fields`, add
`expected_parent_child_edges`, a list of every mechanically required directed relation
between distinct expected rigid roles. Each list item must have exactly:
- `parent_role`, `child_role`: role strings;
- `matched_parent`, `matched_child`: one emitted part name each or null;
- `matched_emitted_edge`: true exactly when those two non-null names appear as that
  directed emitted edge;
- `notes`: concise uncertainty.

Use an empty list only when the category truly requires no relation between distinct
rigid roles. Do not alter any existing labels or fields. Validate all matched names
against emitted_parts and all true matches against emitted_parent_child_edges. Report
only completion counts.
""".strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reviewer", choices=("claude", "codex"), required=True)
    parser.add_argument("--timeout", type=float, default=7200.0)
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate an already staged reviewer-owned augmentation without invoking a model",
    )
    args = parser.parse_args()
    path = ROOT / f"reviewer_{args.reviewer}.json"
    started = datetime.now(timezone.utc).isoformat()
    if args.validate_only:
        result = {
            "reviewer": args.reviewer,
            "started_at": started,
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "process_exit_code": 0,
            "application_mode": "reviewer-owned staged labels, mechanically applied and validated",
            **validate(path),
        }
        (ROOT / f"reviewer_{args.reviewer}_edge_run.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0 if result["valid"] else 1
    if args.reviewer == "claude":
        command = [
            str(CLAUDE),
            "-p",
            prompt(args.reviewer, path),
            "--model",
            "opus",
            "--effort",
            "high",
            "--output-format",
            "stream-json",
            "--verbose",
            "--settings",
            json.dumps(
                {
                    "permissions": {
                        "allow": ["Read", "Write"],
                        "deny": ["Bash", "Edit", "WebFetch", "WebSearch"],
                    }
                }
            ),
            "--allowedTools",
            "Read,Write",
        ]
    else:
        command = [
            str(CODEX),
            "exec",
            "--json",
            "--model",
            "gpt-5.6-sol",
            "-c",
            "model_reasoning_effort='high'",
            "--dangerously-bypass-approvals-and-sandbox",
            "--skip-git-repo-check",
            prompt(args.reviewer, path),
        ]
    completed = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=args.timeout,
        check=False,
    )
    (ROOT / f"reviewer_{args.reviewer}_edge_events.jsonl").write_text(
        completed.stdout, encoding="utf-8"
    )
    (ROOT / f"reviewer_{args.reviewer}_edge_stderr.txt").write_text(
        completed.stderr, encoding="utf-8"
    )
    result = {
        "reviewer": args.reviewer,
        "started_at": started,
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "process_exit_code": completed.returncode,
        **validate(path),
    }
    (ROOT / f"reviewer_{args.reviewer}_edge_run.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if completed.returncode == 0 and result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
