#!/usr/bin/env python3
"""Standalone verifier for SketchMobility Table 4b receipts."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
from typing import Any, Mapping, Sequence

from exp.scripts import run_table4b_urdf_sketch_mobility as adapter


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


def _measurement_from_direction(direction: Mapping[str, Any]) -> dict[str, Any]:
    value = direction.get("normalized_p95")
    if (
        direction.get("status") == "COMPLETE"
        and isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    ):
        return {"status": "COMPLETE", "value": float(value), "reason": None}
    return {
        "status": "N/E",
        "value": None,
        "reason": str(
            direction.get("reason") or "exact surface measurement unavailable"
        ),
    }


def _raw_record_valid(job: Mapping[str, Any], record: Mapping[str, Any]) -> bool:
    if (
        int(record.get("selection_index", -1)) != int(job["selection_index"])
        or record.get("dataset_id") != job["dataset_id"]
        or record.get("asset_id") != job["asset_id"]
        or record.get("category") != job["category"]
        or record.get("package") != job["package"]
        or record.get("expected_urdf_sha256") != job["expected_urdf_sha256"]
        or record.get("expected_package_content_manifest_sha256")
        != job["expected_package_content_manifest_sha256"]
        or record.get("package_content_manifest_sha256")
        != job["expected_package_content_manifest_sha256"]
    ):
        return False
    if record.get("status") != "completed":
        return record.get("urdf_sha256") is None and not record.get("geometry_record")
    geometry_record = record.get("geometry_record")
    if not isinstance(geometry_record, dict):
        return False
    direct_fields = {
        "issues": list(geometry_record.get("issues", [])),
        "geometry_status": geometry_record.get("status"),
        "tree_valid": bool(geometry_record.get("tree_valid")),
        "declared_link_count": int(geometry_record.get("declared_link_count", 0)),
        "declared_visual_element_count": int(
            geometry_record.get("declared_visual_element_count", 0)
        ),
        "loadable_visual_element_count": int(
            geometry_record.get("loadable_visual_element_count", 0)
        ),
        "declared_collision_element_count": int(
            geometry_record.get("declared_collision_element_count", 0)
        ),
        "loadable_collision_element_count": int(
            geometry_record.get("loadable_collision_element_count", 0)
        ),
        "visual_bearing_link_count": int(
            geometry_record.get("visual_bearing_link_count", 0)
        ),
        "d_visual": geometry_record.get("d_visual"),
        "d_visual_status": geometry_record.get("d_visual_status"),
        "analytic_collision_element_count": int(
            geometry_record.get("analytic_collision_element_count", 0)
        ),
        "collision_mesh_element_count": int(
            geometry_record.get("collision_mesh_element_count", 0)
        ),
        "collision_mesh_valid_triangle_count": int(
            geometry_record.get("collision_mesh_valid_triangle_count", 0)
        ),
        "collision_watertight_mesh_count": int(
            geometry_record.get("collision_watertight_mesh_count", 0)
        ),
    }
    if any(record.get(key) != value for key, value in direct_fields.items()):
        return False
    declared = direct_fields["declared_collision_element_count"]
    loadable = direct_fields["loadable_collision_element_count"]
    visual_links = direct_fields["visual_bearing_link_count"]
    extraction_complete = (
        direct_fields["tree_valid"]
        and not direct_fields["issues"]
        and loadable > 0
        and loadable == declared
        and visual_links > 0
    )
    expected_shapes = (
        {"status": "COMPLETE", "value": loadable / visual_links, "reason": None}
        if extraction_complete
        else {
            "status": "N/E",
            "value": None,
            "reason": "tree/resource/collision extraction or visual-bearing denominator is incomplete",
        }
    )
    triangle = adapter.base.geometry.collision_triangle_validation_measurement(
        geometry_record
    )
    expected_triangle = {
        "status": triangle["status"],
        "value": triangle.get("value"),
        "reason": triangle.get("reason"),
        "intended_mesh_count": triangle.get("intended_mesh_count"),
        "measured_mesh_count": triangle.get("measured_mesh_count"),
    }
    redundancy = adapter.base.geometry.collision_redundancy_measurement(
        geometry_record
    )
    timing = record.get("collision_load_time_seconds", {})
    raw_times = timing.get("raw_times_seconds", [])
    timing_valid = (
        timing.get("status") != "COMPLETE"
        or (
            isinstance(raw_times, list)
            and len(raw_times) == adapter.base.geometry.COLLISION_LOAD_TIME_MEASURED_REPEATS
            and all(
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and math.isfinite(float(value))
                and float(value) > 0
                for value in raw_times
            )
            and timing.get("value") == statistics.median(raw_times)
            and int(timing.get("completed_repeats", -1)) == len(raw_times)
        )
    )
    return (
        record.get("visual_to_collision_p95_normalized")
        == _measurement_from_direction(geometry_record.get("visual_to_collision", {}))
        and record.get("collision_to_visual_p95_normalized")
        == _measurement_from_direction(geometry_record.get("collision_to_visual", {}))
        and record.get("shapes_per_visual_bearing_link") == expected_shapes
        and record.get("collision_mesh_triangles_per_asset") == expected_triangle
        and record.get("intra_link_redundancy") == redundancy
        and timing_valid
    )


def verify_records(
    records: Sequence[Mapping[str, Any]], aggregates: Mapping[str, Any]
) -> dict[str, Any]:
    source_manifest = adapter.load_source_manifest()
    jobs = adapter.build_jobs(source_manifest)
    replay = adapter.verify_run(source_manifest, records, aggregates)
    checks = {
        str(row["check"]): bool(row["pass"])
        for row in replay["checks"]
    }
    checks["formal_record_count"] = len(records) == adapter.N_EVAL
    checks["raw_record_atoms_recomputed"] = len(records) == len(jobs) and all(
        _raw_record_valid(job, record)
        for job, record in zip(jobs, records, strict=True)
    )
    return {
        "schema_version": "table4b-sketchmobility-verification/v1",
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
    records = _load_jsonl(root / "asset_records.jsonl")
    if summary.get("mode") == "formal":
        result = verify_records(records, summary.get("metrics", {}))
    else:
        jobs = adapter.build_jobs(adapter.load_source_manifest())[:5]
        adapter.validate_jobs(jobs, adapter.base.WORKERS)
        recomputed = adapter.base.aggregate(records)
        smoke_checks = {
            "smoke_exact_n5": len(records) == len(jobs) == 5,
            "smoke_fixed_prefix": [record.get("asset_id") for record in records]
            == [job["asset_id"] for job in jobs],
            "smoke_raw_record_atoms_recomputed": len(records) == len(jobs)
            and all(
                _raw_record_valid(job, record)
                for job, record in zip(jobs, records, strict=True)
            ),
            "smoke_summary_reaggregation": json.dumps(recomputed, sort_keys=True)
            == json.dumps(summary.get("metrics", {}), sort_keys=True),
            "smoke_execution_config": frozen_config.get("execution", {}).get(
                "workers"
            )
            == adapter.base.WORKERS
            and frozen_config.get("execution", {}).get("child_timeout_seconds")
            == adapter.base.CHILD_TIMEOUT_SECONDS,
        }
        result = {
            "schema_version": "table4b-sketchmobility-verification/v1",
            "status": "PASS" if all(smoke_checks.values()) else "FAIL",
            "check_count": len(smoke_checks),
            "checks": smoke_checks,
        }
    outputs = run_manifest.get("outputs", {})
    result["checks"].update(
        {
            "asset_records_sha256": adapter.base.sha256_bytes(
                (root / "asset_records.jsonl").read_bytes()
            )
            == outputs.get("asset_records_sha256"),
            "summary_sha256": adapter.base.sha256_bytes(
                (root / "summary.json").read_bytes()
            )
            == outputs.get("summary_sha256"),
            "frozen_config_sha256": adapter.base.sha256_bytes(
                (root / "frozen_config.json").read_bytes()
            )
            == run_manifest.get("frozen_config_sha256"),
            "classification_and_mode": summary.get("classification")
            == ("FORMAL" if summary.get("mode") == "formal" else "SMOKE"),
            "source_snapshots_exact": _source_snapshots_valid(
                root, frozen_config
            ),
        }
    )
    if summary.get("mode") == "formal":
        binding = frozen_config.get("smoke_receipt")
        try:
            replayed_binding = adapter.validate_smoke_receipt(
                Path(str(binding.get("path"))) if isinstance(binding, dict) else None
            )
            result["checks"]["formal_smoke_binding"] = replayed_binding == binding
        except Exception:  # noqa: BLE001
            result["checks"]["formal_smoke_binding"] = False
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
