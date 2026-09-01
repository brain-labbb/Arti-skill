#!/usr/bin/env python3
"""Build the final method-blind packet for fields unresolved after rereview."""

from __future__ import annotations

import hashlib
import json
from collections import Counter
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = PROJECT_ROOT.parent.resolve()
RUNTIME = PROJECT_ROOT / "exp/runtime/table2_naming_semantic_v1"
REREVIEW_QUEUE = RUNTIME / "rereview/blind_tasks.jsonl"
REREVIEW_JUDGES = [RUNTIME / f"rereview/judge_{label}.jsonl" for label in "abc"]
PROTOCOL = PROJECT_ROOT / "exp/reference/table2_naming_semantic_tiebreak_v1.json"


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise RuntimeError(f"outside workspace: {resolved}")
    return resolved


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with contained(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def rows(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in contained(path).read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def majority(values: list[Any]) -> Any:
    value, count = Counter(values).most_common(1)[0]
    return value if count >= 2 else None


def main() -> int:
    tasks = {str(row["item_id"]): row for row in rows(REREVIEW_QUEUE)}
    judges = [
        {str(row["item_id"]): row for row in rows(path)} for path in REREVIEW_JUDGES
    ]
    output_rows = []
    for item_id, task in sorted(tasks.items()):
        unresolved = [
            field
            for field in task["rejudge_fields"]
            if majority([judge[item_id][field] for judge in judges]) is None
        ]
        if not unresolved:
            continue
        output_rows.append(
            {
                **task,
                "schema": "table2_naming_semantic_blind_tiebreak_v1",
                "adjudicate_fields": unresolved,
                "judge_geometry_role": None
                if "judge_geometry_role" in unresolved
                else "locked",
                "judge_instance_id": None
                if "judge_instance_id" in unresolved
                else "locked",
                "judge_reason": None,
            }
        )
    output = contained(RUNTIME / "tiebreak")
    output.mkdir(parents=True, exist_ok=True)
    task_path = output / "blind_tasks.jsonl"
    judge_path = output / "adjudicator.jsonl"
    encoded = "".join(
        json.dumps(row, sort_keys=True, ensure_ascii=False) + "\n"
        for row in output_rows
    )
    contained(task_path).write_text(encoded, encoding="utf-8")
    contained(judge_path).write_text(encoded, encoding="utf-8")
    manifest = {
        "protocol_id": "nano3d_table2_naming_semantic_blind_tiebreak_v1",
        "protocol_sha256": digest(PROTOCOL),
        "rereview_judge_sha256": {
            path.stem: digest(path) for path in REREVIEW_JUDGES
        },
        "task_count": len(output_rows),
        "asset_count": len({row["asset_id"] for row in output_rows}),
        "field_counts": dict(
            sorted(
                Counter(
                    field
                    for row in output_rows
                    for field in row["adjudicate_fields"]
                ).items()
            )
        ),
        "blind_tasks_sha256": digest(task_path),
        "method_audit_read": False,
        "prior_votes_exposed": False,
    }
    contained(output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
