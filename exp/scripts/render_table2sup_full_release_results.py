#!/usr/bin/env python3
"""Render a standalone full-release Table 2 supplementary handoff.

This is a read-only renderer.  It accepts the combined receipt emitted by a
runner, while tolerating either ``methods`` or ``datasets`` and either a
top-level ``metrics`` object or the runner's ``table2_supplementary`` object.
"""

from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path
from typing import Any, Mapping


BASELINE_DATASETS = (
    ("articraft", "Articraft-10K"),
    ("lam", "LAM released outputs"),
    ("artiverse", "Artiverse"),
    ("partnet", "PartNet-Mobility"),
    ("physx", "PhysX-Mobility"),
    ("sketch", "SketchMobility"),
    ("infinite", "Infinite Mobility"),
    ("infinigen", "Infinigen-Sim"),
)
OURS_DEFAULTS = {
    "Ours-500K": (500, 2467),
    "Ours per-class N=5 (supplementary)": (2655, 14968),
}
METRICS = (
    ("visual_bearing_collision_coverage", "Visual-bearing Collision Coverage"),
    ("joint_limit_portability", "Joint-limit Portability"),
    ("joint_dynamics_coverage", "Joint Dynamics Coverage"),
    ("placeholder_mass_incidence", "Placeholder-mass Incidence"),
)


def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _metric_map(entry: Mapping[str, Any]) -> Mapping[str, Any]:
    value = entry.get("metrics")
    if not isinstance(value, Mapping):
        value = entry.get("table2_supplementary")
    if not isinstance(value, Mapping):
        raise ValueError("entry has no metrics/table2_supplementary object")
    return value


def normalize_entry(entry: Mapping[str, Any]) -> dict[str, Any]:
    """Adapt one runner entry to the renderer's stable internal schema."""
    display = entry.get("display") or entry.get("display_name") or entry.get("dataset")
    slug = entry.get("slug") or entry.get("id") or entry.get("dataset_id")
    if not display and not slug:
        raise ValueError("dataset identity is missing")
    if not display:
        display = dict(BASELINE_DATASETS).get(str(slug), str(slug))
    n_eval = entry.get("N_eval", entry.get("n_eval"))
    j_eval = entry.get("J_eval", entry.get("j_eval"))
    if n_eval is None or j_eval is None:
        raise ValueError(f"dataset {display!r} is missing N_eval/J_eval")
    evidence = entry.get("evidence") or entry.get("artifacts") or {}
    if not isinstance(evidence, Mapping):
        evidence = {}
    return {
        "slug": str(slug or display).lower().replace(" ", "-"),
        "display": str(display),
        "n_eval": int(n_eval),
        "j_eval": int(j_eval),
        "status": str(entry.get("status", entry.get("checkpoint_state", "unknown"))),
        "evidence": dict(evidence),
        "metrics": _metric_map(entry),
    }


def _format_metric(value: Any) -> str:
    if isinstance(value, str):
        return value
    if not isinstance(value, Mapping):
        return str(value) if value is not None else "N/E"
    # The runner keeps the asset-level visual rate under ``asset`` while the
    # other atoms are flat.  Render the same human-readable fraction for both
    # shapes rather than leaking an implementation detail into the table.
    if isinstance(value.get("asset"), Mapping):
        value = value["asset"]
    if value.get("value") is not None:
        return str(value["value"])
    passed = value.get("passed", value.get("numerator"))
    denominator = value.get("denominator")
    if passed is None or denominator is None:
        return str(value.get("status", "N/E"))
    try:
        pct = 100.0 * float(passed) / float(denominator)
        return f"{int(passed):,} / {int(denominator):,} ({pct:.2f}%)"
    except (TypeError, ValueError, ZeroDivisionError):
        return f"{passed} / {denominator}"


def _evidence_link(entry: Mapping[str, Any], root: Path) -> str:
    evidence = entry.get("evidence", {})
    candidate = evidence.get("summary") or evidence.get("report") or evidence.get("manifest")
    if not candidate:
        candidate = f"{entry['slug']}/summary.json"
    path = Path(str(candidate))
    if path.is_absolute():
        try:
            path = path.relative_to(root)
        except ValueError:
            return str(path)
    return str(path).replace("\\", "/")


def _source_ours_rows(source: str) -> list[tuple[str, list[str]]]:
    heading = re.search(r"^### Table 2 supplementary\..*$", source, re.MULTILINE | re.IGNORECASE)
    if not heading:
        raise ValueError("missing Table 2 supplementary heading")
    end = re.search(r"^#### Table 2 supplementary metric definitions$", source[heading.end():], re.MULTILINE)
    section = source[heading.end(): heading.end() + end.start()] if end else source[heading.end():]
    rows = []
    for line in section.splitlines():
        if not line.startswith("|") or line.startswith("|---"):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if cells and cells[0].startswith("Ours"):
            rows.append((cells[0], cells[1:]))
    if len(rows) != 2:
        raise ValueError(f"expected two source Ours rows, found {len(rows)}")
    return rows


def render(root: Path, source_markdown: Path, *, receipt_path: Path | None = None) -> str:
    root = Path(root).resolve()
    source_markdown = Path(source_markdown).resolve()
    receipt_path = Path(receipt_path or root / "full_release_receipt.json").resolve()
    receipt = _json(receipt_path)
    raw_entries = receipt.get("methods", receipt.get("datasets")) if isinstance(receipt, Mapping) else None
    if isinstance(raw_entries, Mapping):
        raw_entries = list(raw_entries.values())
    if not isinstance(raw_entries, list):
        raise ValueError("combined receipt must contain a methods or datasets list")
    entries = {normalize_entry(item)["display"]: normalize_entry(item) for item in raw_entries if isinstance(item, Mapping)}
    baselines = []
    for slug, display in BASELINE_DATASETS:
        match = entries.get(display) or entries.get(slug) or next((v for v in entries.values() if v["slug"] == slug), None)
        if match is None:
            raise ValueError(f"missing full-release entry for {display}")
        # Use the canonical comparison label in the publication even when a
        # legacy receipt calls the same cohort simply ``LAM``.
        baselines.append({**match, "display": display})

    ours = _source_ours_rows(source_markdown.read_text(encoding="utf-8"))
    try:
        source_link = os.path.relpath(source_markdown, root)
    except ValueError:
        source_link = str(source_markdown)
    lines = [
        "# Table 2 Supplementary Full-Release Results",
        "",
        "Collision, Joint, and Inertial Diagnostics for eight full-release comparison cohorts plus the unchanged Ours rows.",
        "",
        f"- Combined receipt: `{receipt_path}`",
        f"- Source protocol: [{source_markdown.name}]({source_link.replace(os.sep, '/')})",
        "- This renderer is read-only and does not run an evaluator.",
        "",
        "| Dataset / Outputs | N_eval | J_eval | Visual-bearing Collision Coverage | Joint-limit Portability | Joint Dynamics Coverage | Placeholder-mass Incidence | Status | Evidence |",
        "|---|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for display, cells in ours:
        n_eval, j_eval = next((entry[1] for entry in OURS_DEFAULTS.items() if entry[0] == display), ("source", "source"))
        lines.append(f"| {display} | {n_eval:,} | {j_eval:,} | " + " | ".join(cells[:4]) + " | source | source |")
    for entry in baselines:
        values = [_format_metric(entry["metrics"].get(key)) for key, _ in METRICS]
        lines.append(f"| {entry['display']} | {entry['n_eval']:,} | {entry['j_eval']:,} | " + " | ".join(values) + f" | {entry['status']} | [`summary`](" + _evidence_link(entry, root) + ") |")
    lines += ["", "N/E denotes not estimable under the frozen placeholder registry. Asset-level errors and incomplete records remain in the published denominators.", ""]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--source-markdown", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receipt", type=Path)
    args = parser.parse_args(argv)
    output = render(args.root, args.source_markdown, receipt_path=args.receipt)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output, encoding="utf-8")
    print(json.dumps({"output": str(args.output), "rows": 10}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
