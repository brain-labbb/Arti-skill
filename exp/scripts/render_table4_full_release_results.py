#!/usr/bin/env python3
"""Render a standalone Markdown handoff for the full-release Table 4 run.

The renderer is deliberately read-only with respect to evaluation: it only
reads a combined receipt and the frozen protocol document, then writes the
requested Markdown file.  Receipts from early versions of the runner used
slightly different nesting (``methods`` versus ``datasets`` and inline versus
path-backed summaries); the small adapter below keeps those representations
interoperable without changing their values.
"""

from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path
from typing import Any, Mapping, Sequence


DATASETS: tuple[tuple[str, str], ...] = (
    ("articraft", "Articraft-10K"),
    ("lam", "LAM released outputs"),
    ("artiverse", "Artiverse"),
    ("partnet", "PartNet-Mobility"),
    ("physx", "PhysX-Mobility"),
    ("sketch", "SketchMobility"),
    ("infinite", "Infinite Mobility"),
    ("infinigen", "Infinigen-Sim"),
)

# The order and labels are frozen by Table 4 in the protocol document.
METRICS: tuple[tuple[str, str], ...] = (
    ("rest_all_pair_cf", "Rest All-pair CF"),
    ("rest_non_adjacent_cf", "Rest Non-adjacent CF"),
    ("single_joint_sweep_cf", "Single-joint Sweep CF"),
    ("multi_joint_sobol_cf", "Multi-joint Sobol CF"),
    ("collision_state_rate", "Collision-state Rate"),
    ("aor", "AOR"),
    ("max_penetration", "Max Penetration"),
    ("collision_free_range", "Collision-free Range"),
    ("strict_collision_pass", "Strict Collision Pass"),
)

# Historical protocol rows are intentionally copied from the source Markdown;
# these values are not recomputed or silently replaced by a new receipt.
OURS_DEFAULTS: Mapping[str, tuple[int, int]] = {
    "Ours-500K": (500, 2467),
}


def _json(path: Path) -> Any:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    return value


def _mapping(value: Any) -> Mapping[str, Any] | None:
    return value if isinstance(value, Mapping) else None


def _slug(value: Any) -> str:
    text = str(value or "").strip().lower()
    text = re.sub(r"[^a-z0-9]+", "-", text).strip("-")
    aliases = {
        "articraft-10k": "articraft",
        "lam-released-outputs": "lam",
        "lam": "lam",
        "artiverse": "artiverse",
        "partnet-mobility": "partnet",
        "physx-mobility": "physx",
        "sketchmobility": "sketch",
        "infinite-mobility": "infinite",
        "infinite-mobility-supplementary-generated-cohort": "infinite",
        "infinigen-sim": "infinigen",
        "infinigen": "infinigen",
    }
    return aliases.get(text, text)


def _entries(receipt: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    raw: Any = receipt.get("methods", receipt.get("datasets", receipt.get("entries")))
    if isinstance(raw, Mapping):
        raw = list(raw.values())
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes, bytearray)):
        raise ValueError("combined receipt must contain a methods or datasets list")
    result = [item for item in raw if isinstance(item, Mapping)]
    if len(result) != len(raw):
        raise ValueError("combined receipt contains a non-object entry")
    return result


def _load_relative(value: Any, root: Path) -> Any:
    if not isinstance(value, (str, os.PathLike)):
        return value
    path = Path(value)
    if not path.is_absolute():
        path = root / path
    if path.is_file():
        try:
            return _json(path)
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"cannot read receipt summary {path}: {exc}") from exc
    return value


def _metric_object(entry: Mapping[str, Any], root: Path) -> Mapping[str, Any]:
    """Find the metric map in the known receipt nesting variants."""

    candidates: list[Any] = [entry.get("metrics"), entry.get("table4_metrics"), entry.get("table4")]
    for summary_key in ("summary", "summary_path", "summary_json"):
        summary = entry.get(summary_key)
        if summary is not None:
            candidates.append(_load_relative(summary, root))
    evidence = _mapping(entry.get("evidence")) or _mapping(entry.get("artifacts")) or {}
    for key in ("summary", "report", "summary_json"):
        if key in evidence:
            candidates.append(_load_relative(evidence[key], root))

    keys = {name for name, _label in METRICS} | {"multi_joint_sweep_cf"}
    for candidate in candidates:
        candidate = _load_relative(candidate, root)
        obj = _mapping(candidate)
        if obj is None:
            continue
        nested = _mapping(obj.get("metrics"))
        if nested is not None:
            obj = nested
        if any(key in obj for key in keys):
            return obj
    raise ValueError("entry has no Table 4 metrics object")


def normalize_entry(entry: Mapping[str, Any], *, root: Path | None = None) -> dict[str, Any]:
    """Adapt one combined-receipt row to a stable renderer representation."""

    root = Path(root or ".").resolve()
    raw_slug = entry.get("slug") or entry.get("id") or entry.get("dataset_id") or entry.get("dataset")
    slug = _slug(raw_slug)
    display = entry.get("display") or entry.get("display_name") or entry.get("dataset")
    if not slug and not display:
        raise ValueError("dataset identity is missing")
    if not display:
        display = dict(DATASETS).get(slug, str(raw_slug))
    summary_obj: Mapping[str, Any] = {}
    summary_value = entry.get("summary", entry.get("summary_path", entry.get("summary_json")))
    if summary_value is not None:
        loaded_summary = _load_relative(summary_value, root)
        if isinstance(loaded_summary, Mapping):
            summary_obj = loaded_summary
    n_eval = entry.get("N_eval", entry.get("n_eval", summary_obj.get("N_eval", summary_obj.get("n_eval"))))
    j_eval = entry.get("J_eval", entry.get("j_eval", summary_obj.get("J_eval", summary_obj.get("j_eval"))))
    if n_eval is None or j_eval is None:
        raise ValueError(f"dataset {display!r} is missing N_eval/J_eval")
    status = entry.get("status", entry.get("checkpoint_state", summary_obj.get("status", "unknown")))
    evidence = entry.get("evidence") or entry.get("artifacts") or {}
    if isinstance(evidence, str):
        evidence = {"summary": evidence}
    if not isinstance(evidence, Mapping):
        evidence = {}
    return {
        "slug": slug,
        "display": str(display),
        "n_eval": int(n_eval),
        "j_eval": int(j_eval),
        "status": str(status),
        "evidence": dict(evidence),
        "metrics": _metric_object(entry, root),
    }


def _metric(metrics: Mapping[str, Any], key: str) -> Any:
    value = metrics.get(key)
    if value is None and key == "multi_joint_sobol_cf":
        value = metrics.get("multi_joint_sweep_cf")
    return value


def _number(value: Any) -> float | None:
    try:
        return float(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _count(value: Any) -> str:
    """Format a count while allowing source-preserved placeholders."""

    if isinstance(value, bool):
        return str(value)
    try:
        return f"{int(value):,}"
    except (TypeError, ValueError):
        return str(value)


def _fraction(value: Mapping[str, Any], *, numerator_keys: Sequence[str] = ("passed", "numerator")) -> str | None:
    numerator: Any = None
    for key in numerator_keys:
        if value.get(key) is not None:
            numerator = value[key]
            break
    denominator = value.get("denominator", value.get("expected_states", value.get("total_states")))
    if numerator is None or denominator is None:
        return None
    n = _number(numerator)
    d = _number(denominator)
    if n is None or d is None:
        return f"{numerator} / {denominator}"
    if d == 0:
        return f"{int(n):,} / {int(d):,} (N/E)"
    return f"{int(n):,} / {int(d):,} ({100.0 * n / d:.3f}%)"


def _format_metric(value: Any, key: str) -> str:
    if isinstance(value, str):
        return value
    obj = _mapping(value)
    if obj is None:
        return "N/E" if value is None else str(value)
    status = str(obj.get("status", ""))
    # N/E is a semantic result, not a zero-valued pass.  Preserve it even when
    # a blocked metric carries a denominator for audit purposes.
    if status.upper() in {"N/E", "NE", "NOT_ESTIMABLE", "BLOCKED"} and obj.get("value") is None:
        if key != "max_penetration" or obj.get("maximum_observed_normalized") is None:
            return "N/E"

    if key == "max_penetration":
        raw = obj.get(
            "maximum_observed_normalized",
            obj.get("max_penetration", obj.get("maximum", obj.get("value"))),
        )
        if raw is None:
            return status or "N/E"
        number = _number(raw)
        rendered = f"{number:.6f}" if number is not None else str(raw)
        measured = obj.get("fully_measured_assets", obj.get("measured_assets", obj.get("observed_assets")))
        denominator = obj.get("denominator")
        if measured is not None and denominator is not None:
            suffix = f" ({int(measured):,} / {int(denominator):,} measured"
            if status:
                suffix += f"; {status}"
            return rendered + suffix + ")"
        return rendered + (f" ({status})" if status else "")

    if key == "aor":
        raw = obj.get("value", obj.get("mean", obj.get("average")))
        if raw is None or status.upper() in {"N/E", "NE", "NOT_ESTIMABLE", "BLOCKED"}:
            return "N/E"
        number = _number(raw)
        return f"{number:.6f}" if number is not None else str(raw)

    numerator_keys: Sequence[str] = ("passed", "passed_assets", "pass_count", "numerator")
    if key == "collision_state_rate":
        numerator_keys = ("collision_states", "failed_states", "fail_count", "numerator", "passed")
    elif key == "collision_free_range":
        numerator_keys = ("passed_states", "pass_count", "passed", "numerator")
    rendered = _fraction(obj, numerator_keys=numerator_keys)
    if rendered is not None:
        return rendered
    if obj.get("value") is not None:
        return str(obj["value"])
    if obj.get("rate") is not None:
        rate = _number(obj["rate"])
        if rate is not None:
            # Rates in receipts are stored as fractions, not percentages.
            return f"{100.0 * rate:.3f}%"
        return str(obj["rate"])
    return status or "N/E"


def _source_ours_rows(source: str) -> list[tuple[str, list[str]]]:
    heading = re.search(r"^##\s+Table\s+4\.\s+.*$", source, re.MULTILINE | re.IGNORECASE)
    if heading is None:
        raise ValueError("missing Table 4 primary heading")
    boundary = re.search(r"^###\s+Table\s+4\s+evaluation states\s*$", source[heading.end() :], re.MULTILINE | re.IGNORECASE)
    section = source[heading.end() : heading.end() + boundary.start()] if boundary else source[heading.end() :]
    rows: list[tuple[str, list[str]]] = []
    for line in section.splitlines():
        if not line.startswith("|") or line.startswith("|---"):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if cells and (cells[0].lower().startswith("ours") or cells[0].lower().startswith("brain")):
            if len(cells) < 2:
                raise ValueError("malformed Ours Table 4 row")
            rows.append((cells[0], cells[1:]))
    if not rows:
        raise ValueError("Table 4 primary table has no preserved Ours row")
    return rows


def _evidence_link(entry: Mapping[str, Any], root: Path) -> str:
    evidence = entry.get("evidence", {})
    candidate: Any = None
    if isinstance(evidence, Mapping):
        candidate = evidence.get("summary") or evidence.get("report") or evidence.get("manifest")
    candidate = candidate or entry.get("summary_path") or entry.get("output_dir")
    if not candidate:
        candidate = f"{entry['slug']}/summary.json"
    path = Path(str(candidate))
    if path.is_absolute():
        try:
            path = path.relative_to(root)
        except ValueError:
            return str(path).replace("\\", "/")
    return str(path).replace("\\", "/")


def render(root: Path, source_markdown: Path, *, receipt_path: Path | None = None) -> str:
    """Render a standalone Table 4 Markdown report from a combined receipt."""

    root = Path(root).resolve()
    source_markdown = Path(source_markdown).resolve()
    receipt_path = Path(receipt_path or root / "full_release_receipt.json").resolve()
    receipt = _json(receipt_path)
    if not isinstance(receipt, Mapping):
        raise ValueError("combined receipt must be a JSON object")
    entries = [normalize_entry(item, root=root) for item in _entries(receipt)]
    by_slug: dict[str, dict[str, Any]] = {}
    for entry in entries:
        if entry["slug"] in by_slug:
            raise ValueError(f"duplicate dataset entry: {entry['slug']}")
        by_slug[entry["slug"]] = entry
    missing = [slug for slug, _display in DATASETS if slug not in by_slug]
    if missing:
        raise ValueError("combined receipt is missing datasets: " + ", ".join(missing))

    source_text = source_markdown.read_text(encoding="utf-8")
    ours_rows = _source_ours_rows(source_text)
    try:
        source_link = os.path.relpath(source_markdown, root).replace(os.sep, "/")
    except ValueError:
        source_link = str(source_markdown).replace("\\", "/")

    lines = [
        "# Table 4 Full-Release Results",
        "",
        "Collision and mechanical-clearance diagnostics for eight full-release comparison cohorts; the measured or blocked status is shown per row. Ours rows are copied unchanged from the protocol source.",
        "",
        f"- Combined receipt: `{receipt_path}`",
        f"- Source protocol: [{source_markdown.name}]({source_link})",
        "- This renderer is read-only and does not run an evaluator.",
        "",
        "## Table 4. Collision and Mechanical Clearance",
        "",
        "| Dataset / Outputs | N_eval | J_eval | " + " | ".join(label for _key, label in METRICS) + " | Status | Evidence |",
        "|---|---:|---:|" + "---:|" * len(METRICS) + "---|---|---|",
    ]

    for label, cells in ours_rows:
        n_eval, j_eval = OURS_DEFAULTS.get(label, ("source", "source"))
        # Keep source cells byte-for-byte at the Markdown token level.  If an
        # older protocol row has fewer cells, pad visibly rather than inventing
        # metrics.
        source_cells = list(cells[: len(METRICS)])
        source_cells.extend(["source"] * (len(METRICS) - len(source_cells)))
        lines.append(
            f"| {label} | {_count(n_eval)} | {_count(j_eval)} | "
            + " | ".join(source_cells)
            + " | source (unchanged) | protocol source |"
        )

    for slug, display in DATASETS:
        entry = by_slug[slug]
        values = [_format_metric(_metric(entry["metrics"], key), key) for key, _label in METRICS]
        lines.append(
            f"| {display} | {entry['n_eval']:,} | {entry['j_eval']:,} | "
            + " | ".join(values)
            + f" | {entry['status']} | [`summary`]({_evidence_link(entry, root)}) |"
        )

    lines.extend(
        [
            "",
            "N/E denotes not estimable under the frozen protocol. In particular, a release with no native collision geometry is blocked for collision-dependent metrics; an empty contact query is never treated as a collision-free pass.",
            "",
            "The frozen states are rest q=0, K=21 single-joint sweeps, and R=64 Sobol multi-joint configurations (seed 20260813). Pair policy reports all-pair and non-adjacent results and treats penetration strictly greater than 1e-6 m as illegal.",
            "",
            "Generated from the combined receipt; per-dataset records, checkpoints, manifests, and artifact hashes remain in the linked evidence directories.",
            "",
        ]
    )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--source-markdown", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receipt", type=Path)
    args = parser.parse_args(argv)
    rendered = render(args.root, args.source_markdown, receipt_path=args.receipt)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered, encoding="utf-8")
    print(json.dumps({"output": str(args.output), "rows": len(DATASETS) + 1}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["DATASETS", "METRICS", "normalize_entry", "render"]
