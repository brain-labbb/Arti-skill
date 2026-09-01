#!/usr/bin/env python3
"""Repair protocol-invalid zero-DoF asset booleans in a Table 4 receipt.

The full evaluator intentionally records every asset.  Older running workers
could report an empty single-joint/Sobol plan as a vacuous pass for a native
collision asset with zero movable joints.  The frozen Table 4 protocol says
such assets remain in the asset denominator and are fail-closed for the
    Sobol/strict asset-level metrics.  This utility changes only those
asset rows, then deterministically rebuilds the summary and artifact hashes;
it never reruns collision sampling.
"""

from __future__ import annotations

import argparse
import json
import importlib.util
from pathlib import Path
from typing import Any, Mapping


SCRIPT = Path(__file__).resolve()
RUNNER_PATH = SCRIPT.with_name("run_table4_full_release.py")


def _runner() -> Any:
    spec = importlib.util.spec_from_file_location("table4_full_release_runner_for_repair", RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import runner: {RUNNER_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, indent=2, ensure_ascii=True, allow_nan=False) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: list[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(dict(row), sort_keys=True, ensure_ascii=True, allow_nan=False) + "\n" for row in rows), encoding="utf-8")


def repair(output: Path) -> dict[str, Any]:
    """Apply the zero-DoF rule to one completed dataset output directory."""

    runner = _runner()
    output = Path(output).resolve(strict=True)
    manifest = _load(output / "manifest.json")
    records_path = output / "records.jsonl"
    records = [json.loads(line) for line in records_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not isinstance(records, list) or not records:
        raise ValueError(f"no records in {records_path}")
    changed: list[str] = []
    for row in records:
        if not isinstance(row, dict):
            raise ValueError("record is not an object")
        dof = int(row.get("movable_dof_count", row.get("expected_movable_joints", 0)) or 0)
        native = int(row.get("native_collision_elements", 0) or 0) > 0
        status = str(row.get("collision_metric_status", "")).upper()
        if dof == 0 and native and status not in {"N/E", "NE", "BLOCKED"}:
            asset_id = str(row.get("dataset_id", row.get("asset_id", "")))
            changed.append(asset_id)
            row["multi_joint_sobol_cf"] = False
            row["strict_collision_pass"] = False
            row["zero_dof_fail_closed"] = True

    if changed:
        _write_jsonl(records_path, records)
        _write_jsonl(output / "asset_records.jsonl", records)
        _write_json(output / "asset_records.json", records)
        # Child receipts are provenance artifacts.  Update their compact
        # headline fields when they correspond to a repaired asset.
        children = output / "children"
        for child_path in children.glob("*.json") if children.is_dir() else ():
            try:
                child = _load(child_path)
            except (OSError, json.JSONDecodeError):
                continue
            if isinstance(child, dict) and str(child.get("dataset_id", "")) in set(changed):
                child["multi_joint_sobol_cf"] = False
                child["strict_collision_pass"] = False
                child["zero_dof_fail_closed"] = True
                _write_json(child_path, child)

    dataset_slug = str(manifest.get("dataset_slug") or "")
    old_summary = _load(output / "summary.json") if (output / "summary.json").is_file() else {}
    summary = runner.aggregate_records(records, int(manifest["N_eval"]), int(manifest["J_eval"]))
    if isinstance(old_summary, dict):
        # Preserve provenance/claim-boundary fields emitted by the evaluator,
        # while letting the deterministic aggregate replace all metrics and
        # state counters.
        preserved = {key: value for key, value in old_summary.items() if key not in {
            "metrics", "status_counts", "expected_states", "executed_states",
            "state_records_expected", "state_records_executed", "summary_content_sha256",
        }}
        preserved.update(summary)
        summary = preserved
    summary["repair"] = {
        "rule": "zero_dof_native_collision_fail_closed_v1",
        "changed_assets": changed,
        "changed_count": len(changed),
    }
    summary["records_sha256"] = runner.sha256_file(records_path)
    summary["summary_content_sha256"] = runner._self_hash(summary, "summary_content_sha256")
    _write_json(output / "summary.json", summary)
    (output / "summary.md").write_text(runner.report_text(summary, dataset_slug), encoding="utf-8")

    checkpoint = _load(output / "checkpoint.json") if (output / "checkpoint.json").is_file() else {
        "schema_version": "table4_checkpoint_v1",
        "state": "complete",
        "records": len(records),
        "manifest_content_sha256": manifest.get("manifest_content_sha256"),
    }
    checkpoint["state"] = "complete"
    checkpoint["records"] = len(records)
    checkpoint["records_sha256"] = runner.sha256_file(records_path)
    checkpoint["summary_sha256"] = runner.sha256_file(output / "summary.json")
    checkpoint["checkpoint_content_sha256"] = runner._self_hash(checkpoint, "checkpoint_content_sha256")
    _write_json(output / "checkpoint.json", checkpoint)
    _write_json(output / "progress.json", {"status": "COMPLETE", "completed_assets": len(records), "total_assets": len(records), "state_records": sum(int(row.get("state_records_count", 0) or 0) for row in records)})
    _write_json(output / "artifact_manifest.json", runner._artifact_manifest(output))
    return {"output": str(output), "changed_count": len(changed), "changed_assets": changed}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, action="append", required=True, help="completed Table 4 dataset output (repeatable)")
    args = parser.parse_args(argv)
    results = [repair(path) for path in args.output]
    print(json.dumps({"datasets": results}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
