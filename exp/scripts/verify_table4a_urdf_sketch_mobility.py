#!/usr/bin/env python3
"""Standalone verifier for SketchMobility Table 4a receipts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

from exp.scripts import run_table4a_urdf_sketch_mobility as adapter


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def _source_snapshots_valid(
    root: Path, frozen_config: Mapping[str, Any]
) -> bool:
    expected = frozen_config.get("source_snapshots")
    if not isinstance(expected, dict) or not expected:
        return False
    snapshot_root = root / "source_snapshots"
    observed_paths = {
        path.relative_to(snapshot_root).as_posix()
        for path in snapshot_root.rglob("*")
        if path.is_file()
    }
    return observed_paths == set(expected) and all(
        adapter.common.sha256_file(snapshot_root / relative) == digest
        for relative, digest in expected.items()
    )


def _raw_record_valid(job: Mapping[str, Any], record: Mapping[str, Any]) -> bool:
    if (
        int(record.get("selection_index", -1)) != int(job["selection_index"])
        or record.get("dataset_id") != job["dataset_id"]
        or record.get("category") != job["category"]
        or record.get("package") != job["package"]
        or record.get("expected_urdf_sha256") != job["expected_urdf_sha256"]
        or record.get("expected_package_content_manifest_sha256")
        != job["expected_package_content_manifest_sha256"]
        or record.get("package_content_manifest_sha256")
        != job["expected_package_content_manifest_sha256"]
        or int(record.get("expected_movable_dof", -1))
        != int(job["expected_movable_dof"])
        or int(record.get("states_intended", -1))
        != int(job["expected_state_count"])
    ):
        return False
    joints = record.get("joint_records", [])
    if not isinstance(joints, list) or len(joints) != len(job["joints"]):
        return False
    if not job["genesis_eligible"]:
        return (
            record.get("status") == "error"
            and record.get("issues")
            == ["table2_collision_coverage_incomplete"]
            and int(record.get("states_executed", -1)) == 0
            and all(
                joint.get("joint_name") == expected["name"]
                and joint.get("joint_type") == expected["type"]
                and int(joint.get("states_intended", -1)) == adapter.base.SINGLE_SAMPLES
                and int(joint.get("states_executed", -1)) == 0
                and joint.get("full_range_cf_pass") is False
                and int(joint.get("safe_dof", -1)) == 0
                for expected, joint in zip(job["joints"], joints, strict=True)
            )
        )

    total_executed = 0
    verified_hashes = mismatch_hashes = no_reference_hashes = 0
    for expected, joint in zip(job["joints"], joints, strict=True):
        if (
            joint.get("joint_name") != expected["name"]
            or joint.get("joint_type") != expected["type"]
            or bool(joint.get("table3_joint_level_pass"))
            != bool(expected["table3_joint_level_pass"])
        ):
            return False
        states = joint.get("state_summaries", [])
        if not states:
            if int(joint.get("states_executed", -1)) != 0:
                return False
            continue
        if len(states) != adapter.base.SINGLE_SAMPLES:
            return False
        executed = illegal = 0
        endpoints_ok: dict[int, bool] = {}
        for sample_index, state in enumerate(states):
            if int(state.get("sample_index", -1)) != sample_index:
                return False
            was_executed = state.get("executed") is True
            executed += int(was_executed)
            if was_executed:
                illegal += int(bool(state.get("illegal_collision")))
                reference = expected["state_hash_references"][sample_index]
                observed_hash = state.get("q_intended_values_sha256")
                if reference is None:
                    no_reference_hashes += 1
                elif observed_hash == reference:
                    verified_hashes += 1
                else:
                    mismatch_hashes += 1
                if sample_index in {0, adapter.base.SINGLE_SAMPLES - 1}:
                    endpoints_ok[sample_index] = (
                        state.get("observation_status") == "COMPLETE"
                        and state.get("q_readback_finite") is True
                    )
        full_range = executed == adapter.base.SINGLE_SAMPLES and illegal == 0
        bounded = expected["type"] != "continuous"
        limit_reachable = bool(
            bounded
            and full_range
            and endpoints_ok.get(0, False)
            and endpoints_ok.get(adapter.base.SINGLE_SAMPLES - 1, False)
        )
        safe_dof = int(full_range and bool(expected["table3_joint_level_pass"]))
        if (
            int(joint.get("states_executed", -1)) != executed
            or int(joint.get("illegal_states", -1)) != illegal
            or bool(joint.get("full_range_cf_pass")) != full_range
            or bool(joint.get("limit_reachable")) != limit_reachable
            or int(joint.get("safe_dof", -1)) != safe_dof
        ):
            return False
        total_executed += executed
    cross = record.get("state_hash_cross_check", {})
    return (
        int(record.get("states_executed", -1)) == total_executed
        and int(cross.get("verified", -1)) == verified_hashes
        and int(cross.get("mismatch", -1)) == mismatch_hashes
        and int(cross.get("no_reference", -1)) == no_reference_hashes
        and mismatch_hashes == 0
    )


def verify_records(
    records: Sequence[Mapping[str, Any]], aggregates: Mapping[str, Any]
) -> dict[str, Any]:
    source_manifest = adapter.load_source_manifest()
    strict = adapter.load_table4_strict_pass()
    table3, _ = adapter.load_table3_joint_pass()
    jobs = adapter.build_jobs(
        source_manifest, table3, adapter.load_table4_state_hashes()
    )
    replay = adapter.base.verify_run(
        source_manifest, records, aggregates, strict
    )
    checks = {
        str(row["check"]): bool(row["pass"])
        for row in replay["checks"]
    }
    checks["formal_record_count"] = len(records) == adapter.N_EVAL
    checks["table2_gate_partition"] = (
        sum(
            record.get("issues") == ["table2_collision_coverage_incomplete"]
            for record in records
        )
        == 489
    )
    checks["formal_joint_denominator"] = (
        aggregates.get("joint_level_full_range_cf", {}).get("denominator")
        == adapter.J_EVAL
    )
    checks["formal_retention_denominator"] = (
        aggregates.get("collision_safe_dof_retention", {}).get("denominator")
        == adapter.J_EVAL
    )
    checks["raw_record_atoms_recomputed"] = len(records) == len(jobs) and all(
        _raw_record_valid(job, record)
        for job, record in zip(jobs, records, strict=True)
    )
    return {
        "schema_version": "table4a-sketchmobility-verification/v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "check_count": len(checks),
        "checks": checks,
    }


def verify_output(root: Path, *, write: bool = True) -> dict[str, Any]:
    root = root.resolve(strict=True)
    summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
    run_manifest = json.loads(
        (root / "manifest.json").read_text(encoding="utf-8")
    )
    frozen_config = json.loads(
        (root / "frozen_config.json").read_text(encoding="utf-8")
    )
    asset_records = _load_jsonl(root / "asset_records.jsonl")
    joint_records = _load_jsonl(root / "joint_records.jsonl")
    by_index: dict[int, list[dict[str, Any]]] = {}
    for joint in joint_records:
        by_index.setdefault(int(joint["selection_index"]), []).append(joint)
    records: list[dict[str, Any]] = []
    for record in asset_records:
        selection_index = int(record["selection_index"])
        rows = sorted(
            by_index.get(selection_index, []),
            key=lambda row: int(row.get("dof_position", 10**9)),
        )
        records.append({**record, "joint_records": rows})
    if summary.get("mode") == "formal":
        result = verify_records(records, summary.get("metrics", {}))
    else:
        source_manifest = adapter.load_source_manifest()
        table3, _ = adapter.load_table3_joint_pass()
        jobs = adapter.build_jobs(
            source_manifest, table3, adapter.load_table4_state_hashes()
        )[:5]
        adapter.validate_jobs(jobs, adapter.WORKERS)
        recomputed = adapter.base.aggregate(
            records, adapter.load_table4_strict_pass()
        )
        smoke_checks = {
            "smoke_exact_n5": len(records) == len(jobs) == 5,
            "smoke_fixed_prefix": [record.get("dataset_id") for record in records]
            == [job["dataset_id"] for job in jobs],
            "smoke_raw_record_atoms_recomputed": len(records) == len(jobs)
            and all(
                _raw_record_valid(job, record)
                for job, record in zip(jobs, records, strict=True)
            ),
            "smoke_summary_reaggregation": adapter.common.canonical_sha256(
                recomputed
            )
            == adapter.common.canonical_sha256(summary.get("metrics", {})),
            "smoke_execution_config": frozen_config.get("execution", {}).get(
                "workers"
            )
            == adapter.WORKERS
            and frozen_config.get("execution", {}).get("child_timeout_seconds")
            == adapter.CHILD_TIMEOUT_SECONDS,
        }
        result = {
            "schema_version": "table4a-sketchmobility-verification/v1",
            "status": "PASS" if all(smoke_checks.values()) else "FAIL",
            "check_count": len(smoke_checks),
            "checks": smoke_checks,
        }
    outputs = run_manifest.get("outputs", {})
    artifact_checks = {
        "asset_records_sha256": adapter.base.lam4a.sha256_file(
            root / "asset_records.jsonl"
        )
        == outputs.get("asset_records_sha256"),
        "joint_records_sha256": adapter.base.lam4a.sha256_file(
            root / "joint_records.jsonl"
        )
        == outputs.get("joint_records_sha256"),
        "summary_sha256": adapter.base.lam4a.sha256_file(root / "summary.json")
        == outputs.get("summary_sha256"),
        "frozen_config_sha256": adapter.base.lam4a.sha256_file(
            root / "frozen_config.json"
        )
        == run_manifest.get("frozen_config_sha256"),
        "classification_and_mode": summary.get("classification")
        == ("FORMAL" if summary.get("mode") == "formal" else "SMOKE"),
        "source_snapshots_exact": _source_snapshots_valid(root, frozen_config),
    }
    if summary.get("mode") == "formal":
        binding = frozen_config.get("smoke_receipt")
        try:
            replayed_binding = adapter.validate_smoke_receipt(
                Path(str(binding.get("path"))) if isinstance(binding, dict) else None
            )
            artifact_checks["formal_smoke_binding"] = replayed_binding == binding
        except Exception:  # noqa: BLE001
            artifact_checks["formal_smoke_binding"] = False
    result["checks"].update(artifact_checks)
    result["check_count"] = len(result["checks"])
    result["status"] = (
        "PASS" if all(result["checks"].values()) else "FAIL"
    )
    if write:
        adapter.base.atomic_write_json(root / "standalone_verification.json", result)
        if result["status"] == "PASS":
            adapter.common.write_receipt_closure(
                root, adapter.base.atomic_write_json
            )
            return verify_output(root, write=False)
        return result
    result["checks"]["receipt_artifact_closure"] = (
        adapter.common.validate_receipt_closure(root)
    )
    result["check_count"] = len(result["checks"])
    result["status"] = (
        "PASS" if all(result["checks"].values()) else "FAIL"
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--no-write", action="store_true")
    args = parser.parse_args()
    result = verify_output(args.output, write=not args.no_write)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
