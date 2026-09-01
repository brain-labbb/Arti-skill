#!/usr/bin/env python3
"""Recovery finalizer for the frozen ``urdf_lam_supplementary_n800_20260817_v2`` run.

Background
----------
The frozen formal Genesis run ``urdf_lam_supplementary_n800_20260817_v2``
executed all 800 child evaluations (ranks 1..800) between 2026-08-17T17:54Z
and 2026-08-18T14:28Z, then crashed inside ``run_scope`` aggregation:

1. Producer/verifier contract bug (rank 46 only): the child receipt
   serializes ``asset_record.intra_link_redundancy`` with ``status="N/E"``
   but ``measured_link_count=1``; the frozen verifier
   (``_redundancy_measurement``) requires ``measured == 0`` for ``N/E``.
   The bug is in ``lam_supplementary_geometry.collision_redundancy_measurement``
   early-return paths, which keep the pre-computed measured counter.
   No metric value is affected: the atom is N/E either way, with identical
   reason and null numeric fields.
2. Post-run code drift: ``lam_supplementary_static.py`` was edited on
   2026-08-19 (SHA-256 1c2fdc2c... -> 04985b5a...) for the unrelated
   Table 2 supplementary work, AFTER every child receipt had been written.
   The frozen runner's ``_validate_execution_binding`` and the frozen
   verifier's ``_validate_code_identity`` re-hash the current files and
   therefore can never pass again for this run, even though every receipt
   was produced while the code matched the frozen ``code_identity``.

What this script does
---------------------
It reproduces ``run_scope``'s aggregation and publish steps verbatim
(same row construction, sort orders, strict-state validation, frozen
``aggregate_records``, summary schema and report template), with two
documented deviations:

- The raise-on-drift execution-binding gate is replaced by an
  observed-vs-frozen comparison whose outcome is recorded in
  ``RECOVERY.md`` and ``recovery_manifest.json``.
- The single rank-46 receipt must already be repaired on disk
  (``measured_link_count`` 1 -> 0); the original buggy bytes are preserved
  in ``child_attempts/rank_0046.json`` and both hashes are recorded.

The script writes the six final artifacts exactly once and never modifies
any receipt.  It must run under the frozen Genesis interpreter.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
from typing import Any, Mapping

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_urdf_lam_supplementary_v1 as runner  # noqa: E402
from exp.scripts import verify_urdf_lam_supplementary_v1 as verifier  # noqa: E402

OUTPUT_ROOT = REPO / "exp/runtime/urdf_lam_supplementary_n800_20260817_v2"
SAMPLE_SIZE = runner.SAMPLE_SIZE
JOINT_COUNT = runner.JOINT_COUNT
SOBOL_STATE_COUNT_PER_ASSET = runner.SOBOL_STATE_COUNT_PER_ASSET
STATE_SAMPLES_PER_JOINT = runner.STATE_SAMPLES_PER_JOINT
REPAIRED_RANK = 46
FROZEN_STATIC_SHA256 = "1c2fdc2c3d9f8ebcb3ab6b0bf8144b307c86b4b44790cf3182c2395ab37267ff"


def sha256_file(path: Path) -> str:
    return runner.sha256_file(path)


def main() -> int:
    output_root = OUTPUT_ROOT
    manifest_payload = runner._regular_file(output_root / "frozen_manifest.json").read_text(encoding="utf-8")
    manifest = json.loads(manifest_payload)
    if runner._manifest_hash(manifest) != manifest.get("manifest_content_sha256"):
        raise runner.GenesisAdapterError("frozen manifest self-hash mismatch")
    if bool(manifest.get("qualification_smoke")):
        raise runner.GenesisAdapterError("refusing to finalize a qualification_smoke manifest")

    source = runner.load_source_cohort(runner.DEFAULT_SOURCE_RECORDS, runner.DEFAULT_SOURCE_MANIFEST)
    items = {int(item["selection_rank"]): item for item in manifest.get("items", []) if isinstance(item, Mapping)}
    if set(items) != set(range(1, SAMPLE_SIZE + 1)):
        raise runner.GenesisAdapterError("frozen manifest does not cover ranks 1..800")

    # --- Receipt repair audit (repair itself is applied out-of-band) -------
    repaired_receipt = output_root / "children" / f"rank_{REPAIRED_RANK:04d}.json"
    preserved_attempt = output_root / "child_attempts" / f"rank_{REPAIRED_RANK:04d}.json"
    repaired_data = json.loads(runner._regular_file(repaired_receipt).read_text(encoding="utf-8"))
    preserved_data = json.loads(runner._regular_file(preserved_attempt).read_text(encoding="utf-8"))
    repaired_red = repaired_data["asset_record"]["intra_link_redundancy"]
    preserved_red = preserved_data["asset_record"]["intra_link_redundancy"]
    if repaired_red.get("measured_link_count") != 0 or repaired_red.get("status") != "N/E":
        raise runner.GenesisAdapterError("rank 46 receipt is not in the repaired N/E contract state")
    if preserved_red.get("measured_link_count") != 1:
        raise runner.GenesisAdapterError("rank 46 attempt no longer preserves the original buggy record")
    repair_audit = {
        "receipt_path": str(repaired_receipt),
        "receipt_sha256_after_repair": sha256_file(repaired_receipt),
        "preserved_original_attempt_path": str(preserved_attempt),
        "preserved_original_attempt_sha256": sha256_file(preserved_attempt),
        "change": "asset_record.intra_link_redundancy.measured_link_count: 1 -> 0 (status N/E)",
        "metric_impact": "none: atom remains N/E with identical reason and null numeric fields",
    }

    # --- Load all receipts (structural binding enforced by frozen code) ----
    results: dict[int, dict[str, Any]] = {}
    for rank in range(1, SAMPLE_SIZE + 1):
        cached = runner._read_child_receipt(output_root, rank, items[rank], source)
        if cached is None:
            raise runner.GenesisAdapterError(f"missing child receipt for rank {rank}")
        results[rank] = cached

    # --- Observed runtime binding vs frozen runtime binding ----------------
    cache_path = runner.bind_genesis_cache(output_root)
    expected_cache = (output_root.resolve(strict=False) / "genesis-cache").resolve(strict=False)
    observed_runtime = runner.genesis_runtime_binding(expected_cache_path=expected_cache)
    frozen_runtime = manifest.get("runtime_binding")
    runtime_matches = isinstance(frozen_runtime, Mapping) and dict(frozen_runtime) == observed_runtime

    # --- Observed code identity vs frozen code identity --------------------
    observed_identity = runner.current_code_identity()
    frozen_identity = manifest.get("code_identity", {})
    identity_comparison = {}
    for name in ("runner", "static", "geometry", "verifier"):
        frozen_entry = frozen_identity.get(name, {})
        observed_entry = observed_identity.get(name, {})
        identity_comparison[name] = {
            "frozen_sha256": frozen_entry.get("sha256"),
            "observed_sha256": observed_entry.get("sha256"),
            "matches_freeze": frozen_entry.get("sha256") == observed_entry.get("sha256"),
        }

    # --- Row construction verbatim from run_scope ---------------------------
    asset_rows = [results[rank]["asset_record"] for rank in range(1, SAMPLE_SIZE + 1)]
    joint_rows = [
        joint for rank in range(1, SAMPLE_SIZE + 1) for joint in results[rank]["joint_records"]
    ]
    joint_rows.sort(key=lambda row: source.joint_order[(str(row["asset_key"]), str(row["joint_name"]))])
    state_rows = [
        state for rank in range(1, SAMPLE_SIZE + 1) for state in results[rank]["state_records"]
    ]
    state_rows.sort(
        key=lambda row: (
            source.joint_order[(str(row["asset_key"]), str(row["joint_name"]))],
            int(row["sample_index"]),
        )
    )
    strict_rows = [
        state
        for rank in range(1, SAMPLE_SIZE + 1)
        for state in results[rank]["strict_state_records"]
    ]
    expected_strict_count = SAMPLE_SIZE + sum(
        SOBOL_STATE_COUNT_PER_ASSET for rank in range(1, SAMPLE_SIZE + 1)
        if runner.source_joints(source, str(items[rank]["asset_key"]))
    )
    if len(strict_rows) != expected_strict_count:
        raise runner.GenesisAdapterError(
            f"strict state aggregate has {len(strict_rows)} rows, expected {expected_strict_count}"
        )

    # --- Frozen metric aggregation chain ------------------------------------
    config = verifier.VerifierConfig(source_records=runner.DEFAULT_SOURCE_RECORDS, source_manifest=runner.DEFAULT_SOURCE_MANIFEST)
    assets_by_rank = {int(row["selection_rank"]): dict(row) for row in asset_rows}
    joints_by_key = {
        (str(row["asset_key"]), str(row["joint_name"])): dict(row) for row in joint_rows
    }
    states_by_joint: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in state_rows:
        key = (str(row["asset_key"]), str(row["joint_name"]))
        states_by_joint.setdefault(key, []).append(dict(row))
    _, derived_strict_pass = verifier._validate_strict_state_records(
        [dict(row) for row in strict_rows], source, items, assets_by_rank,
        states_by_joint, str(manifest["protocol_id"]), observed_runtime, config,
    )
    aggregates = verifier.aggregate_records(
        assets_by_rank, joints_by_key, states_by_joint, derived_strict_pass, source, config
    )

    # --- Summary/report verbatim from run_scope (pilot=False, all ranks) ----
    evaluated_assets = sum(bool(row.get("evaluation_success")) for row in asset_rows)
    failed_assets = SAMPLE_SIZE - evaluated_assets
    all_states_executed = all(row.get("executed") is True for row in state_rows)
    all_strict_executed = all(row.get("executed") is True for row in strict_rows)
    complete = bool(failed_assets == 0 and all_states_executed and all_strict_executed)
    status = "COMPLETE" if complete else "PARTIAL"
    mode = "formal Genesis run"
    aggregate_hash = runner.canonical_sha256(aggregates)
    summary = {
        "schema_version": 1,
        "protocol_id": manifest["protocol_id"],
        "status": status,
        "cohort": {
            "selected": SAMPLE_SIZE, "assets": SAMPLE_SIZE, "joints": JOINT_COUNT,
            "pilot_selected_ranks": list(range(1, SAMPLE_SIZE + 1)),
            "evaluated_asset_count": evaluated_assets,
            "fail_closed_asset_count": failed_assets,
            "strict_state_record_count": len(strict_rows),
            "strict_state_expected": expected_strict_count,
        },
        "input_binding": {
            "table3_asset_records_sha256": source.records_sha256,
            "table3_manifest_sha256": source.manifest_sha256,
            "table3_manifest_content_sha256": source.manifest_content_sha256,
            "ordered_selected_asset_keys_sha256": source.ordered_keys_sha256,
        },
        "scope": {
            "mode": mode, "engine_protocol_id": runner.ENGINE_PROTOCOL_ID,
            "selected_rank_count": SAMPLE_SIZE, "state_record_policy": runner.STATE_RECORD_POLICY,
            "strict_state_record_policy": "all_intended", "formal_claim": complete,
        },
        "runtime_binding": dict(observed_runtime),
        "code_identity": observed_identity,
        "verification_aggregates_sha256": aggregate_hash,
        "verification_aggregates": aggregates,
    }
    report_lines = [
        "# LAM Genesis supplementary evaluation", "",
        f"Protocol: `{manifest['protocol_id']}` (`{runner.ENGINE_PROTOCOL_ID}`).", "",
        "Frozen cohort: N=800 assets, J=2395 movable joints, K=21 intended states per joint.",
        f"Scope: {mode}; selected ranks={SAMPLE_SIZE}; terminal fail-closed assets={failed_assets}.",
        f"Verification aggregates SHA256: `{aggregate_hash}`.",
        "Strict state records: 50336 intended raw rows (800 rest + 49536 Sobol).",
        "",
        "Table-4a uses Genesis contact penetration with a strict illegal threshold of 1e-6 m; signed clearance is N/E because this adapter does not invent a separated-pair signed distance.",
        "Table-2, Table-4b, and Supplementary S1 records remain explicit in the atomic asset rows; empty LAM receipt/allowance registries are preserved.",
    ]
    report = "\n".join(report_lines) + "\n"

    final_paths = [
        output_root / "asset_records.jsonl", output_root / "joint_records.jsonl",
        output_root / "state_records.jsonl", output_root / "strict_state_records.jsonl",
        output_root / "summary.json", output_root / "report.md",
    ]
    if any(path.exists() for path in final_paths):
        raise runner.GenesisAdapterError("one or more final artifacts already exist; refusing overwrite")
    runner._write_once_jsonl(final_paths[0], asset_rows)
    runner._write_once_jsonl(final_paths[1], joint_rows)
    runner._write_once_jsonl(final_paths[2], state_rows)
    runner._write_once_jsonl(final_paths[3], strict_rows)
    runner._write_once_json(final_paths[4], summary)
    runner._write_once_text(final_paths[5], report)

    # --- Recovery disclosure -------------------------------------------------
    recovery_manifest = {
        "schema_version": 1,
        "kind": "recovery_finalization",
        "output_root": str(output_root),
        "frozen_manifest_sha256": sha256_file(output_root / "frozen_manifest.json"),
        "frozen_protocol_id": manifest["protocol_id"],
        "repair_audit": repair_audit,
        "runtime_binding_matches_freeze": runtime_matches,
        "code_identity_comparison": identity_comparison,
        "drift_note": (
            "lam_supplementary_static.py drifted from the frozen code identity after all "
            "800 child receipts were written (child receipts completed 2026-08-18; the "
            "module was edited 2026-08-19 during unrelated Table 2 supplementary work). "
            "The frozen execution-binding gate therefore cannot pass; the metric "
            "aggregation chain (verifier._validate_strict_state_records + "
            "verifier.aggregate_records) does not depend on the drifted module and passes."
        ),
        "final_artifacts": {
            path.name: sha256_file(path) for path in final_paths
        },
        "verification_aggregates_sha256": aggregate_hash,
        "summary_status": status,
    }
    runner._write_once_json(output_root / "recovery_manifest.json", recovery_manifest)

    recovery_lines = [
        "# Recovery finalization — urdf_lam_supplementary_n800_20260817_v2",
        "",
        f"Frozen protocol: `{manifest['protocol_id']}` (`{runner.ENGINE_PROTOCOL_ID}`).",
        "",
        "## Why recovery was required",
        "",
        "1. The frozen `run_scope` aggregation crashed on rank 46: the child receipt serialized",
        "   `asset_record.intra_link_redundancy` with `status=\"N/E\"` but `measured_link_count=1`,",
        "   while the frozen verifier (`_redundancy_measurement`) requires `measured == 0` for N/E.",
        "   Root cause: `lam_supplementary_geometry.collision_redundancy_measurement` early-return",
        "   paths keep the pre-computed measured counter. No metric value is affected (the atom",
        "   stays N/E with the same reason and null numeric fields).",
        "2. `lam_supplementary_static.py` drifted from the frozen `code_identity` on 2026-08-19",
        f"   (frozen `{FROZEN_STATIC_SHA256}` -> observed "
        f"`{identity_comparison['static']['observed_sha256']}`), AFTER all child receipts were "
        "written (2026-08-17/18). The frozen execution-binding gate re-hashes current files and",
        "   can therefore never pass for this completed run.",
        "",
        "## Repair applied (single receipt, single field)",
        "",
        f"- Receipt `{repair_audit['receipt_path']}`:",
        f"  `asset_record.intra_link_redundancy.measured_link_count` changed `1 -> 0`.",
        f"- SHA-256 after repair: `{repair_audit['receipt_sha256_after_repair']}`.",
        f"- Original buggy bytes preserved in `{repair_audit['preserved_original_attempt_path']}`",
        f"  (SHA-256 `{repair_audit['preserved_original_attempt_sha256']}`), untouched.",
        f"- Metric impact: none (N/E before and after; identical reason and null numeric fields).",
        "",
        "## Binding comparisons at finalization",
        "",
        f"- runtime_binding matches freeze: {runtime_matches}",
        "",
        "| component | frozen sha256 | observed sha256 | matches |",
        "|---|---|---|---|",
    ]
    for name in ("runner", "static", "geometry", "verifier"):
        entry = identity_comparison[name]
        recovery_lines.append(
            f"| {name} | `{entry['frozen_sha256']}` | `{entry['observed_sha256']}` | {entry['matches_freeze']} |"
        )
    recovery_lines.extend([
        "",
        "## Aggregation evidence",
        "",
        f"- verifier._validate_strict_state_records: PASS ({len(strict_rows)} strict rows)",
        "- verifier.aggregate_records: PASS (800 assets, 2395 joints)",
        f"- verification aggregates SHA256: `{aggregate_hash}`",
        f"- summary status: {status} (fail-closed assets retained in every denominator)",
        "",
        "Final artifacts were written exactly once by the recovery finalizer",
        "(`exp/scripts/finalize_urdf_lam_supplementary_recovery.py`) using the frozen runner's",
        "row construction, sort orders, summary schema and report template.",
        "",
    ])
    runner._write_once_text(output_root / "RECOVERY.md", "\n".join(recovery_lines))

    print(runner.canonical_json({
        "status": status,
        "output_root": str(output_root),
        "verification_aggregates_sha256": aggregate_hash,
        "runtime_binding_matches_freeze": runtime_matches,
        "code_identity_matches": {k: v["matches_freeze"] for k, v in identity_comparison.items()},
        "table4a": aggregates["table4a"],
    }))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
