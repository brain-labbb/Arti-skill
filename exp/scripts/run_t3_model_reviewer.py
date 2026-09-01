#!/usr/bin/env python3
"""Run one independent blinded T3 model reviewer and retain the complete trace."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
REVIEW_ROOT = EXP_ROOT / "runtime/t3_formal_v1/semantic_review"
PACKET = REVIEW_ROOT / "review_packet.json"
CLAUDE = Path("/mnt/zsn/miniconda3/bin/claude")
CODEX = Path("/mnt/zsn/miniconda3/bin/codex")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    problems: list[str] = []
    panels = payload.get("panels", [])
    if len(panels) != 33:
        problems.append(f"expected 33 panels, found {len(panels)}")
    for panel in panels:
        panel_id = panel.get("panel_id", "unknown")
        fields = panel.get("review_fields", {})
        for part in fields.get("parts", []):
            if not isinstance(part.get("semantically_valid"), bool) or not part.get("role"):
                problems.append(f"{panel_id}: incomplete part {part.get('name')}")
        roles = fields.get("expected_visible_or_functional_roles")
        if not isinstance(roles, list) or not roles:
            problems.append(f"{panel_id}: missing expected roles")
        else:
            for role in roles:
                if not role.get("role") or not isinstance(role.get("min_instances"), int) or not isinstance(role.get("matched_parts"), list):
                    problems.append(f"{panel_id}: malformed expected role")
        for edge in fields.get("edges", []):
            if not isinstance(edge.get("semantically_correct"), bool):
                problems.append(f"{panel_id}: incomplete edge")
        for key in ("hierarchy_exact_match", "instance_discriminability"):
            if not isinstance(fields.get(key), bool):
                problems.append(f"{panel_id}: incomplete {key}")
    return {"panel_count": len(panels), "problems": problems, "valid": not problems}


def prompt(reviewer: str, output: Path) -> str:
    return f"""
Act as independent blinded semantic reviewer `{reviewer}` for a formal 3D asset study.
Read `review_packet.json` completely and inspect every one of its 33 four-view PNG renders
with the image-reading tool. Do not inspect source templates, source model.py files, prior
experiment summaries, or any other reviewer output. Follow the packet instructions exactly.

Edit only `{output.name}` (a private copy of the packet). Replace every review null for all
33 panels. Each expected role must be an object with exactly: `role` (specific string),
`min_instances` (positive integer), `matched_parts` (list of emitted names), and `notes`.
Use conservative visible/functionally necessary role recall. Give every emitted part a
nonempty role even if `semantically_valid` is false. Preserve panel IDs, asset IDs, render
paths, emitted names, edges, protocol, and instructions. Do not change any other file.

Before finishing, programmatically re-read the output and make sure there are 33 panels,
no nulls in requested review fields, every expected-role list is nonempty, and all matched
part names are emitted names in that panel. Report only completion counts and uncertainty;
do not read or infer another reviewer's labels.
""".strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reviewer", choices=("claude", "codex"), required=True)
    parser.add_argument("--timeout", type=float, default=7200.0)
    args = parser.parse_args()
    output = REVIEW_ROOT / f"reviewer_{args.reviewer}.json"
    if not output.exists():
        shutil.copy2(PACKET, output)
    started = datetime.now(timezone.utc).isoformat()
    if args.reviewer == "claude":
        command = [
            str(CLAUDE),
            "-p",
            prompt(args.reviewer, output),
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
            prompt(args.reviewer, output),
        ]
    completed = subprocess.run(
        command,
        cwd=REVIEW_ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=args.timeout,
    )
    (REVIEW_ROOT / f"reviewer_{args.reviewer}_events.jsonl").write_text(
        completed.stdout, encoding="utf-8"
    )
    (REVIEW_ROOT / f"reviewer_{args.reviewer}_stderr.txt").write_text(
        completed.stderr, encoding="utf-8"
    )
    verdict = validate(output)
    summary = {
        "reviewer": args.reviewer,
        "started_at": started,
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "process_exit_code": completed.returncode,
        "review_sha256": sha256(output),
        **verdict,
    }
    (REVIEW_ROOT / f"reviewer_{args.reviewer}_run.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if completed.returncode == 0 and verdict["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
