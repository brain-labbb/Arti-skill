#!/usr/bin/env python3
"""Normalize stale local ordinals in resumable full-release receipts.

This is a one-time, identity-preserving repair for receipts produced by an
older runner.  It never changes asset order, status, metrics, or source
bindings; it only rebinds each record's ordinal to the frozen roster row and
recomputes the affected checkpoint/artifact hashes.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import table123_full_release_common as common


def _records_path(table: Path) -> Path:
    for name in ("records.jsonl", "asset_records.jsonl"):
        path = table / name
        if path.is_file():
            return path
    raise ValueError(f"missing records file in {table}")


def _write_records(path: Path, records: list[dict[str, Any]]) -> None:
    payload = b"".join(common._canonical_bytes(record) + b"\n" for record in records)
    common._atomic_write_bytes(path, payload)


def repair_dataset(dataset: Path) -> dict[str, Any]:
    dataset = Path(dataset).resolve()
    roster = common.load_roster(dataset / "full_release_manifest.json", verify_sources=False)
    rows = list(roster["rows"])
    expected_ids = [str(row["asset_id"]) for row in rows]
    changed: dict[str, list[int]] = {}
    for table_name in ("table1", "table2", "table3"):
        table = dataset / table_name
        path = _records_path(table)
        records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        if len(records) != len(rows):
            raise ValueError(f"{dataset.name}/{table_name}: record count mismatch")
        if [str(record.get("asset_id", record.get("asset_key"))) for record in records] != expected_ids:
            raise ValueError(f"{dataset.name}/{table_name}: record identity/order mismatch")
        indexes: list[int] = []
        for index, (row, record) in enumerate(zip(rows, records)):
            expected = int(row.get("ordinal", index))
            if record.get("ordinal") != expected:
                indexes.append(index)
                record["ordinal"] = expected
        if not indexes:
            continue
        _write_records(path, records)
        checkpoint_path = table / "checkpoint.json"
        checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))
        checkpoint["completed_ordinals"] = list(range(len(rows)))
        if "asset_records_sha256" in checkpoint:
            checkpoint["asset_records_sha256"] = common.sha256_file(path)
        common.write_checkpoint(checkpoint_path, checkpoint)
        artifact_path = table / "artifact_manifest.json"
        artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
        for entry in artifact.get("artifacts", []):
            target = table / entry["path"]
            entry["size"] = target.stat().st_size
            entry["sha256"] = common.sha256_file(target)
        artifact.pop("artifact_manifest_content_sha256", None)
        artifact["artifact_manifest_content_sha256"] = common.canonical_sha256(artifact)
        common._atomic_write_json(artifact_path, artifact)
        common.verify_artifacts(table)
        changed[table_name] = indexes
    return {"dataset": roster["dataset"], "N_eval": roster["N_eval"], "changed": changed}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path, nargs="+")
    parser.add_argument("--receipt", type=Path)
    args = parser.parse_args()
    results = [repair_dataset(path) for path in args.dataset]
    payload = {"schema_version": "table123_full_release_ordinal_repair_v1", "datasets": results}
    if args.receipt:
        args.receipt.parent.mkdir(parents=True, exist_ok=True)
        common._atomic_write_json(args.receipt, payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
