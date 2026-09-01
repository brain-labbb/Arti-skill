#!/usr/bin/env python3
"""Build a method-blind field-only packet for missing three-judge consensus."""

from __future__ import annotations

import hashlib
import json
from collections import Counter
from pathlib import Path
from typing import Any, Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = PROJECT_ROOT.parent.resolve()
RUNTIME = PROJECT_ROOT / "exp/runtime/table2_naming_semantic_v1"
QUEUE = RUNTIME / "blind_tasks.jsonl"
JUDGES = [RUNTIME / f"judges/judge_{label}.jsonl" for label in "abc"]
PROTOCOL = PROJECT_ROOT / "exp/reference/table2_naming_semantic_adjudication_v1.json"
FIELDS = ("judge_geometry_role", "judge_instance_id")


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


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in contained(path).read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        "".join(json.dumps(row, sort_keys=True, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )


def majority(values: list[Any]) -> Any:
    value, count = Counter(values).most_common(1)[0]
    return value if count >= 2 else None


def main() -> int:
    queue = read_jsonl(QUEUE)
    queue_by_id = {str(row["item_id"]): row for row in queue}
    judges = []
    for path in JUDGES:
        rows = read_jsonl(path)
        indexed = {str(row["item_id"]): row for row in rows}
        if len(indexed) != 1107 or set(indexed) != set(queue_by_id):
            raise RuntimeError(f"invalid first-round judge file: {path}")
        judges.append(indexed)
    tasks = []
    field_counts = Counter()
    for item_id in sorted(queue_by_id):
        requested = [
            field
            for field in FIELDS
            if majority([judge[item_id][field] for judge in judges]) is None
        ]
        if not requested:
            continue
        field_counts.update(requested)
        task = queue_by_id[item_id]
        tasks.append(
            {
                "schema": "table2_naming_semantic_blind_rereview_v1",
                "item_id": item_id,
                "asset_id": task["asset_id"],
                "category": task["category"],
                "node_name": task["node_name"],
                "preview_path": task["preview_path"],
                "asset_mesh_node_names": task["asset_mesh_node_names"],
                "required_roles": task["required_roles"],
                "optional_roles": task["optional_roles"],
                "instance_discriminability_rules": task[
                    "instance_discriminability_rules"
                ],
                "rejudge_fields": requested,
                "judge_geometry_role": None
                if "judge_geometry_role" in requested
                else "locked",
                "judge_instance_id": None
                if "judge_instance_id" in requested
                else "locked",
                "judge_reason": None,
            }
        )
    output = contained(RUNTIME / "rereview")
    output.mkdir(parents=True, exist_ok=True)
    write_jsonl(output / "blind_tasks.jsonl", tasks)
    for label in "abc":
        write_jsonl(output / f"judge_{label}.jsonl", tasks)
    manifest = {
        "protocol_id": "nano3d_table2_naming_semantic_blind_re_review_v1",
        "adjudication_protocol_sha256": digest(PROTOCOL),
        "first_round_judge_sha256": {
            path.stem: digest(path) for path in JUDGES
        },
        "task_count": len(tasks),
        "asset_count": len({row["asset_id"] for row in tasks}),
        "field_counts": dict(sorted(field_counts.items())),
        "blind_task_sha256": digest(output / "blind_tasks.jsonl"),
        "method_audit_read": False,
        "other_judge_votes_exposed": False,
    }
    contained(output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
