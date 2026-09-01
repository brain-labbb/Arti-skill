#!/usr/bin/env python3
"""Aggregate verified Table 4 v2 score summaries into paper-ready tables."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
EXP_ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = EXP_ROOT / "reference/table4_constraints_v2/protocol.json"


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(contained(path).read_text(encoding="utf-8"))


def parse_input(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("expected METHOD=SUMMARY_JSON")
    method, raw_path = value.split("=", 1)
    if not method or not raw_path:
        raise argparse.ArgumentTypeError("expected METHOD=SUMMARY_JSON")
    return method, contained(Path(raw_path))


def percent(value: float | None) -> str:
    return f"{100.0 * value:.1f}" if value is not None else "N/A"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", action="append", type=parse_input, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    protocol = load_json(PROTOCOL)
    output = contained(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    seen = set()
    for declared_method, path in args.summary:
        if declared_method in seen:
            raise ValueError(f"duplicate method: {declared_method}")
        seen.add(declared_method)
        summary = load_json(path)
        if summary.get("method") != declared_method:
            raise ValueError(f"method mismatch in {path}")
        if summary.get("benchmark_id") != protocol["benchmark_id"]:
            raise ValueError(f"benchmark mismatch in {path}")
        if summary.get("prompt_manifest_sha256") != protocol["prompt_manifest_sha256"]:
            raise ValueError(f"prompt hash mismatch in {path}")
        if summary.get("task_count") != protocol["task_count"]:
            raise ValueError(f"task count mismatch in {path}")
        rows.append(summary)
    panel_order = {"structured_main": 0, "cad_numeric": 1, "image_count": 2}
    rows.sort(key=lambda row: (panel_order.get(row["panel"], 99), row["method"]))
    payload = {
        "schema_version": 2,
        "benchmark_id": protocol["benchmark_id"],
        "prompt_manifest_sha256": protocol["prompt_manifest_sha256"],
        "rows": rows,
    }
    (output / "table4_constraints_v2.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8"
    )
    lines = [
        "# Table 4: Constraint Satisfaction", "",
        "| Panel | Method | Artifacts | Coverage (%) | Satisfaction (%) | Conditional (%) | Count pass | Numeric pass |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['panel']} | {row['method']} | {row['artifact_count']}/{row['task_count']} | "
            f"{percent(row['coverage'])} | {percent(row['satisfaction'])} | "
            f"{percent(row['conditional_accuracy'])} | {row['count_pass'] or 'N/A'} | "
            f"{row['numeric_pass'] or 'N/A'} |"
        )
    lines.extend([
        "",
        f"Frozen prompt manifest SHA-256: `{protocol['prompt_manifest_sha256']}`.",
        "",
        "Missing or invalid final artifacts remain in the all-constraint denominator. "
        "Conditional accuracy is reported only to separate generation/measurement coverage from measured compliance.",
        "",
    ])
    (output / "table4_constraints_v2.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({"methods": len(rows), "output": str(output)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
