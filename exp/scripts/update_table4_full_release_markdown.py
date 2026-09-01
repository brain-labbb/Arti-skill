#!/usr/bin/env python3
"""Update the primary Table 4 comparison rows from a full-release receipt.

The protocol's Ours/Brain rows are evidence owned by the source document and
are copied byte-for-byte.  Only the eight comparison rows are regenerated;
the historical N=800 narrative below the table is deliberately retained.
"""

from __future__ import annotations

import argparse
import json
import importlib.util
from pathlib import Path
import re
import sys
from typing import Any, Mapping

try:
    from . import render_table4_full_release_results as renderer
except ImportError:  # pragma: no cover
    try:
        import render_table4_full_release_results as renderer  # type: ignore
    except ImportError:
        _renderer_path = Path(__file__).with_name("render_table4_full_release_results.py")
        _spec = importlib.util.spec_from_file_location("table4_full_release_renderer", _renderer_path)
        if _spec is None or _spec.loader is None:
            raise
        renderer = importlib.util.module_from_spec(_spec)
        sys.modules.setdefault("table4_full_release_renderer", renderer)
        _spec.loader.exec_module(renderer)


DATASETS = renderer.DATASETS
METRICS = renderer.METRICS


def _json(path: Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _source_rows(section: str) -> tuple[str, str, list[str], list[str]]:
    """Return table prefix, separator, preserved Ours/Brain lines, suffix."""

    lines = section.splitlines()
    header_index = next(
        (i for i, line in enumerate(lines) if line.startswith("|") and "Dataset / Outputs" in line),
        None,
    )
    if header_index is None or header_index + 1 >= len(lines):
        raise ValueError("Table 4 primary table header is missing")
    separator = lines[header_index + 1]
    if not re.match(r"^\|\s*:?-{2,}", separator):
        raise ValueError("Table 4 primary table separator is malformed")
    end = header_index + 2
    while end < len(lines) and (lines[end].startswith("|") or not lines[end].strip()):
        end += 1
    body = lines[header_index + 2 : end]
    preserved = []
    for line in body:
        if not line.startswith("|"):
            continue
        label = line.strip().strip("|").split("|", 1)[0].strip().lower()
        if label.startswith("ours") or label.startswith("brain"):
            preserved.append(line)
    prefix = "\n".join(lines[: header_index + 2])
    suffix = "\n".join(lines[end:])
    return prefix, separator, preserved, suffix


def _metric_values(entry: Mapping[str, Any], root: Path) -> list[str]:
    normalized = renderer.normalize_entry(entry, root=root)
    return [renderer._format_metric(renderer._metric(normalized["metrics"], key), key) for key, _label in METRICS]


def _receipt_entries(path: Path, root: Path) -> dict[str, Mapping[str, Any]]:
    receipt = _json(path)
    if not isinstance(receipt, Mapping):
        raise ValueError("combined receipt is not an object")
    entries = renderer._entries(receipt)
    result: dict[str, Mapping[str, Any]] = {}
    for entry in entries:
        normalized = renderer.normalize_entry(entry, root=root)
        slug = normalized["slug"]
        if slug in result:
            raise ValueError(f"duplicate receipt dataset: {slug}")
        result[slug] = entry
    missing = [slug for slug, _display in DATASETS if slug not in result]
    if missing:
        raise ValueError("receipt missing datasets: " + ", ".join(missing))
    return result


def update(source: Path, receipt: Path, output: Path | None = None, *, root: Path | None = None) -> str:
    source = Path(source).resolve(strict=True)
    receipt = Path(receipt).resolve(strict=True)
    root = Path(root or receipt.parent).resolve()
    text = source.read_text(encoding="utf-8")
    heading = re.search(r"^##\s+Table\s+4\.\s+Collision\s+and\s+Mechanical\s+Clearance\s*$", text, re.MULTILINE | re.IGNORECASE)
    if heading is None:
        raise ValueError("source is missing the primary Table 4 heading")
    after = text[heading.end() :]
    boundary = re.search(r"^###\s+Table\s+4\s+evaluation\s+states\s*$", after, re.MULTILINE | re.IGNORECASE)
    if boundary is None:
        raise ValueError("source is missing the Table 4 evaluation-state boundary")
    section = text[heading.end() : heading.end() + boundary.start()]
    prefix, _separator, preserved, suffix = _source_rows(section)
    entries = _receipt_entries(receipt, root)
    generated: list[str] = []
    for slug, display in DATASETS:
        normalized = renderer.normalize_entry(entries[slug], root=root)
        values = _metric_values(entries[slug], root)
        generated.append(f"| {display} | " + " | ".join(values) + " |")

    # Preserve source-owned rows first, then publish canonical comparison rows.
    table = prefix + "\n" + "\n".join(preserved + generated)
    note = (
        "\n\n> Full-release comparison update (2026-08-26): the eight comparison rows above use every row in the frozen full-release rosters (N/J are recorded in the linked receipt). Ours/Brain rows are unchanged; historical N=800 evidence paragraphs below are retained as historical evidence."
    )
    rebuilt_section = table + note + ("\n" + suffix if suffix else "")
    # Keep the next protocol heading visually and syntactically separate from
    # the generated callout; the boundary itself is retained verbatim.
    boundary_text = text[heading.end() + boundary.start() :]
    result = text[: heading.end()] + rebuilt_section.rstrip() + "\n\n" + boundary_text.lstrip("\n")
    target = Path(output or source)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(result, encoding="utf-8")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--root", type=Path)
    args = parser.parse_args(argv)
    update(args.source, args.receipt, args.output, root=args.root)
    print(json.dumps({"output": str((args.output or args.source).resolve()), "datasets": len(DATASETS)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["update"]
