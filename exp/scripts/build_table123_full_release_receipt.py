#!/usr/bin/env python3
"""Build a compact index for the published Table 1/2/3 full-release receipts."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import table123_full_release_common as common


DATASETS = (
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
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected object: {path}")
    return value


def _status_counts(output: Path) -> dict[str, int]:
    record_name = "records.jsonl" if (output / "records.jsonl").is_file() else "asset_records.jsonl"
    counts: dict[str, int] = {}
    for line in (output / record_name).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        status = str(json.loads(line).get("status", "unknown"))
        counts[status] = counts.get(status, 0) + 1
    return dict(sorted(counts.items()))


def _table_metrics(table: str, summary: dict[str, Any]) -> dict[str, Any]:
    if table == "table1":
        # Table 1 keeps its structural aggregates at the summary top level,
        # unlike Tables 2/3.  Copy the metric payload into the compact receipt
        # so the read-only acceptance script can compare every published cell.
        names = (
            "links_per_asset",
            "movable_joints_per_asset",
            "multi_joint_assets",
            "unique_topologies",
            "exact_duplicate_rate",
        )
        return {name: summary.get(name, {}) for name in names}
    return summary.get("metrics", {})


def build_receipt(root: Path) -> dict[str, Any]:
    root = Path(root).resolve()
    datasets: list[dict[str, Any]] = []
    for slug, display in DATASETS:
        dataset = root / slug
        roster = _json(dataset / "full_release_manifest.json")
        entry: dict[str, Any] = {
            "slug": slug,
            "dataset": display,
            "output_dir": str(dataset),
            "N_release": roster.get("N_release"),
            "N_eval": roster.get("N_eval"),
            "J_eval": roster.get("J_eval"),
            "roster_sha256": roster.get("roster_sha256"),
            "manifest_content_sha256": roster.get("manifest_content_sha256"),
            "source_bindings": roster.get("source_bindings", []),
            "tables": {},
        }
        for table in ("table1", "table2", "table3"):
            output = dataset / table
            summary = _json(output / "summary.json")
            manifest = _json(output / "manifest.json")
            checkpoint = _json(output / "checkpoint.json")
            artifact = _json(output / "artifact_manifest.json")
            entry["tables"][table] = {
                "N_eval": summary.get("n_eval", summary.get("N_eval")),
                "J_eval": summary.get("j_eval", summary.get("J_eval")),
                "status_counts": summary.get("status_counts") or _status_counts(output),
                "asset_failure_count": summary.get("asset_failure_count", summary.get("error_count", 0)),
                "error_count": summary.get("error_count", 0),
                "metrics": _table_metrics(table, summary),
                "checkpoint_state": checkpoint.get("state"),
                "run_manifest_sha256": manifest.get("manifest_content_sha256"),
                "artifact_manifest_sha256": common.sha256_file(output / "artifact_manifest.json"),
            }
        datasets.append(entry)
    verification_path = root / "full_release_verification_auto.json"
    verification: dict[str, Any] = {}
    if verification_path.is_file():
        verification = _json(verification_path)
    receipt: dict[str, Any] = {
        "schema_version": "table123_full_release_receipt_v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "runtime_root": str(root),
        "scope": {
            "comparison_datasets": [display for _slug, display in DATASETS],
            "ours_unchanged": True,
            "full_release_rosters": True,
            "historical_n800_receipts_overwritten": False,
            "table2_timeout_seconds": 300,
            "table3_samples_per_joint": 21,
        },
        "datasets": datasets,
        "source_aware_verification": {
            "mode": "auto",
            "receipt_path": str(verification_path),
            "receipt_sha256": common.sha256_file(verification_path) if verification_path.is_file() else None,
            "all_pass": bool(verification) and all("error" not in value for value in verification.values()),
        },
        "infinigen_archive_validation_receipt_sha256": common.sha256_file(
            root / "infinigen_archive_validation_receipt.json"
        ),
    }
    return receipt


def render_markdown(receipt: dict[str, Any]) -> str:
    lines = [
        "# Table 1/2/3 Full-Release Receipt",
        "",
        f"Generated: `{receipt['generated_at_utc']}`",
        "",
        "The comparison rows use complete local rosters. Ours/Brain rows are unchanged; historical N=800 receipts remain separate.",
        "",
        "| Dataset | N_eval | J_eval | Table 1 | Table 2 | Table 3 | Roster SHA-256 |",
        "|---|---:|---:|---|---|---|---|",
    ]
    for entry in receipt["datasets"]:
        statuses = []
        for table in ("table1", "table2", "table3"):
            value = entry["tables"][table]
            state = value["checkpoint_state"]
            status = ", ".join(f"{k}={v}" for k, v in sorted(value["status_counts"].items()))
            statuses.append(f"`{state}` ({status})")
        lines.append(
            f"| {entry['dataset']} | {entry['N_eval']:,} | {entry['J_eval']:,} | "
            + " | ".join(statuses)
            + f" | `{entry['roster_sha256']}` |"
        )
    lines.extend(
        [
            "",
            f"Infinigen archive validation receipt SHA-256: `{receipt['infinigen_archive_validation_receipt_sha256']}`.",
            "",
            "Ordinal-only recovery normalization was applied to legacy resumable records; asset IDs, source hashes, statuses, metrics, and roster order were unchanged. The affected output artifact and checkpoint hashes were recomputed.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    args = parser.parse_args()
    receipt = build_receipt(args.root)
    common._atomic_write_json(args.json, receipt)
    common._atomic_write_bytes(args.markdown, render_markdown(receipt).encode("utf-8"))
    print(json.dumps({"json": str(args.json), "markdown": str(args.markdown)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
