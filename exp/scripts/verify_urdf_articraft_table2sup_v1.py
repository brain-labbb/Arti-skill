#!/usr/bin/env python3
"""Independent verification for the Articraft-10K Table 2 supplementary run.

Re-reads the frozen run directory, re-derives cohort bindings from disk,
recomputes every aggregate independently from asset records, re-hashes all
packages, and spot-recomputes static atoms.  Writes verification.json.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
RUNNER_PATH = SCRIPT_PATH.with_name("run_urdf_articraft_table2sup_v1.py")
STATIC_ATOMS_PATH = SCRIPT_PATH.with_name("lam_supplementary_static.py")
SPOT_AUDIT_STRIDE = 32
SPOT_AUDIT_LIMIT = 25


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


runner = _load_module("table2sup_runner", RUNNER_PATH)
static_atoms = _load_module("table2sup_static_atoms", STATIC_ATOMS_PATH)


class CheckLog:
    def __init__(self) -> None:
        self.checks: list[dict[str, Any]] = []

    def add(self, name: str, passed: bool, detail: str = "") -> None:
        self.checks.append({"check": name, "passed": bool(passed), "detail": detail})
        print(f"[{'PASS' if passed else 'FAIL'}] {name}" + (f": {detail}" if detail else ""), flush=True)

    @property
    def all_passed(self) -> bool:
        return all(row["passed"] for row in self.checks)


def verify(run_dir: Path) -> int:
    log = CheckLog()
    manifest = json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))
    records = runner.load_jsonl(run_dir / "asset_records.jsonl")
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))

    manifest_self = runner.canonical_sha256(
        {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    )
    log.add(
        "manifest_self_hash",
        manifest_self == manifest.get("manifest_content_sha256"),
        manifest.get("manifest_content_sha256", ""),
    )

    source = manifest.get("source", {})
    selection = manifest.get("selection", {})
    log.add(
        "frozen_cohort_file_sha",
        source.get("cohort_manifest_file_sha256") == runner.FORMAL_COHORT_FILE_SHA256,
    )
    log.add(
        "frozen_cohort_content_sha",
        source.get("cohort_manifest_content_sha256") == runner.FORMAL_COHORT_CONTENT_SHA256,
    )
    log.add(
        "frozen_selected_asset_ids_sha",
        selection.get("selected_asset_ids_sha256") == runner.FORMAL_SELECTED_ASSET_IDS_SHA256,
    )
    log.add("frozen_seed", selection.get("seed") == runner.FORMAL_SEED)
    log.add("n_eval_800", selection.get("n_eval") == runner.FORMAL_N_EVAL)
    log.add(
        "category_mapping_sha",
        source.get("category_mapping_sha256") == runner.FORMAL_SELECTED_CATEGORY_MAPPING_SHA256,
    )
    log.add("eval_category_count_222", source.get("eval_category_count") == runner.FORMAL_EVAL_CATEGORY_COUNT)
    log.add(
        "category_records_revision",
        source.get("category_records_revision") == runner.FORMAL_CATEGORY_RECORDS_REVISION,
    )
    log.add("table3_records_sha", source.get("table3_records_sha256") == runner.FORMAL_TABLE3_RECORDS_FILE_SHA256)
    log.add("classification_formal", manifest.get("classification") == "FORMAL")
    log.add(
        "placeholder_registry_frozen_empty",
        manifest.get("evaluation", {}).get("config", {}).get("placeholder_mass_registry") == [],
    )

    manifest_records = manifest.get("records", [])
    rows_by_key = {row["asset_key"]: row for row in manifest_records}
    log.add("record_count_matches", len(records) == len(manifest_records), f"{len(records)} vs {len(manifest_records)}")
    keys_ok = sorted(record.get("asset_key") for record in records) == sorted(rows_by_key)
    log.add("record_keys_match_manifest", keys_ok)
    rank_ok = all(
        record.get("selection_index") == rows_by_key.get(record.get("asset_key"), {}).get("selection_index")
        and record.get("selection_rank") == rows_by_key.get(record.get("asset_key"), {}).get("selection_rank")
        for record in records
    )
    log.add("records_bound_to_selection_ranks", rank_ok)
    manifest_hash = manifest.get("manifest_content_sha256")
    binding_ok = all(record.get("manifest_content_sha256") == manifest_hash for record in records)
    log.add("records_bound_to_manifest_hash", binding_ok)
    log.add("no_duplicate_records", len({record.get("asset_key") for record in records}) == len(records))
    status_counts = summary.get("status_counts", {})
    log.add("all_records_completed", status_counts.get("completed") == len(records), str(status_counts))
    log.add("summary_n_eval", summary.get("n_eval") == runner.FORMAL_N_EVAL)
    log.add("summary_j_eval_frozen", summary.get("j_eval") == runner.FORMAL_J_EVAL, str(summary.get("j_eval")))

    # Independent re-aggregation from records.
    recomputed = runner.aggregate_records(records, len(records))
    for key in ("metrics", "category_macro", "breakdown"):
        log.add(f"reaggregate_{key}_matches_summary", recomputed[key] == summary.get(key))

    # Re-hash every package and URDF against the frozen manifest rows.
    binding_failures: list[str] = []
    urdf_failures: list[str] = []
    for row in manifest_records:
        package = Path(row["package"])
        try:
            if runner._package_binding(package) != row["package_binding"]:
                binding_failures.append(row["asset_key"])
            urdf_path = package / runner.URDF_RELATIVE_PATH
            if runner.sha256_file(urdf_path) != row["urdf_sha256"]:
                urdf_failures.append(row["asset_key"])
        except Exception as exc:  # noqa: BLE001
            binding_failures.append(f"{row['asset_key']}: {type(exc).__name__}")
    log.add("package_bindings_revalidated", not binding_failures, ";".join(binding_failures[:5]))
    log.add("urdf_hashes_revalidated", not urdf_failures, ";".join(urdf_failures[:5]))

    # Code identity.
    evaluation = manifest.get("evaluation", {})
    log.add("adapter_sha_matches_disk", evaluation.get("adapter_sha256") == runner.sha256_file(RUNNER_PATH))
    log.add(
        "static_atoms_sha_matches_disk",
        evaluation.get("static_atoms_sha256") == runner.sha256_file(STATIC_ATOMS_PATH),
    )
    snapshot_path = evaluation.get("protocol_snapshot_path")
    snapshot_ok = bool(snapshot_path) and Path(snapshot_path).is_file()
    if snapshot_ok:
        snapshot_ok = runner.sha256_file(Path(snapshot_path)) == evaluation.get("protocol_snapshot_sha256")
    log.add("protocol_snapshot_bound", snapshot_ok)

    # Spot audit: recompute atoms for a deterministic subset.
    spot_failures: list[str] = []
    spot_count = 0
    ordered_by_rank = sorted(records, key=lambda record: int(record.get("selection_rank") or 0))
    for index, record in enumerate(ordered_by_rank):
        if index % SPOT_AUDIT_STRIDE != 0 or spot_count >= SPOT_AUDIT_LIMIT:
            continue
        spot_count += 1
        row = rows_by_key[record["asset_key"]]
        recomputed_record = static_atoms.audit_lam_package(
            Path(row["package"]),
            urdf_relative_path=runner.URDF_RELATIVE_PATH,
            asset_id=row["asset_key"],
            expected_movable_joints=int(row["expected_movable_joint_count"]),
            placeholder_registry=runner.PLACEHOLDER_MASS_REGISTRY,
        )
        if recomputed_record.get("table2_supplementary") != record.get("table2_supplementary"):
            spot_failures.append(row["asset_key"])
    log.add("spot_audit_atoms_recomputed", not spot_failures and spot_count > 0, f"n={spot_count}")

    result = {
        "schema_version": 1,
        "run_dir": str(run_dir),
        "manifest_content_sha256": manifest_hash,
        "checks": log.checks,
        "overall": "PASS" if log.all_passed else "FAIL",
        "passed": sum(int(row["passed"]) for row in log.checks),
        "total": len(log.checks),
    }
    runner.atomic_write_json(run_dir / "verification.json", result)
    print(f"{result['passed']} / {result['total']} checks PASS" if log.all_passed else "VERIFICATION FAILED")
    return 0 if log.all_passed else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    return verify(args.run_dir.resolve(strict=True))


if __name__ == "__main__":
    raise SystemExit(main())
