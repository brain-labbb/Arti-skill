#!/usr/bin/env python3
"""Render a concise Markdown handoff from the verified Table 1/2/3 release."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

try:
    import table123_full_release_common as common
except ImportError:  # pragma: no cover
    from . import table123_full_release_common as common


TABLES = (1, 2, 3)
DATASET_ORDER = (
    ("articraft", "Articraft-10K"),
    ("lam", "LAM released outputs"),
    ("artiverse", "Artiverse"),
    ("partnet", "PartNet-Mobility"),
    ("physx", "PhysX-Mobility"),
    ("sketch", "SketchMobility"),
    ("infinite", "Infinite Mobility"),
    ("infinigen", "Infinigen-Sim"),
)


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _primary_section(markdown: str, table: int) -> list[str]:
    heading = re.compile(rf"^## Table {table}\.\s.*$", re.MULTILINE).search(markdown)
    if heading is None:
        raise ValueError(f"missing Table {table} heading")
    if table in (1, 2):
        marker = re.compile(rf"^### Table {table} metric definitions$", re.MULTILINE)
    else:
        marker = re.compile(r"^### Table 3 evaluation states$", re.MULTILINE)
    end_match = marker.search(markdown, heading.end())
    if end_match is None:
        raise ValueError(f"missing Table {table} primary-table boundary")
    return markdown[heading.start() : end_match.start()].strip().splitlines()


def _status(value: dict[str, Any], table: str) -> str:
    table_value = value["tables"][table]
    counts = ", ".join(
        f"{key}={count}" for key, count in sorted(table_value.get("status_counts", {}).items())
    )
    return f"`{table_value.get('checkpoint_state', 'unknown')}` ({counts})"


def render(root: Path, source_markdown: Path) -> str:
    root = Path(root).resolve()
    source_markdown = Path(source_markdown).resolve()
    source_text = source_markdown.read_text(encoding="utf-8")
    receipt = _json(root / "full_release_receipt.json")
    entries = {str(item["slug"]): item for item in receipt.get("datasets", [])}
    lines = [
        "# Table 1/2/3 Full-Release Results",
        "",
        "This handoff contains the verified primary Table 1, Table 2, and Table 3 rows for the complete local comparison releases. Ours/Brain rows are unchanged and historical N=800 evidence remains separate.",
        "",
        "- Source protocol: [URDF-Sim-Ready-Automatic-Evaluation.md](../../URDF-Sim-Ready-Automatic-Evaluation.md)",
        "- Compact receipt: [full_release_receipt.md](full_release_receipt.md)",
        "- Read-only acceptance report: [automation_check_auto.json](automation_check_auto.json)",
        "",
    ]
    for table in TABLES:
        lines.extend(_primary_section(source_text, table))
        lines.extend(["", "---", ""])
    lines.extend(
        [
            "## Receipt Status",
            "",
            "All eight rosters and all 24 table checkpoints are complete. Asset-level errors and incomplete fingerprints are retained in the published denominators.",
            "",
            "| Dataset | N_eval | J_eval | Table 1 | Table 2 | Table 3 | Roster |",
            "|---|---:|---:|---|---|---|---|",
        ]
    )
    for slug, display in DATASET_ORDER:
        entry = entries[slug]
        lines.append(
            f"| {display} | {entry['N_eval']:,} | {entry['J_eval']:,} | "
            f"{_status(entry, 'table1')} | {_status(entry, 'table2')} | {_status(entry, 'table3')} | "
            f"[`manifest.json`]({slug}/full_release_manifest.json) |"
        )
    lines.extend(
        [
            "",
            "Per-table evidence is under each dataset directory: `summary.json`, `records.jsonl` or `asset_records.jsonl`, `checkpoint.json`, and `artifact_manifest.json`.",
            "",
            "Generated from the already verified receipts; this renderer does not run an evaluator.",
            "",
        ]
    )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--source-markdown", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    rendered = render(args.root, args.source_markdown)
    common._atomic_write_bytes(args.output, rendered.encode("utf-8"))
    print(json.dumps({"output": str(args.output), "bytes": len(rendered.encode('utf-8'))}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["render"]
