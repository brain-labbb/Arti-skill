#!/usr/bin/env python3
"""Create conservative T5 CCD metrics using the full frozen-gold denominator."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path


EXP_ROOT = Path(__file__).resolve().parents[1]
ROOT = EXP_ROOT / "runtime/t5_formal_v1/ccd_adaptive_v3"
GOLD = EXP_ROOT / "runtime/t5_formal_v1/gold_then_compile/summary.json"


def main() -> int:
    raw = json.loads((ROOT / "summary.json").read_text(encoding="utf-8"))
    gold = json.loads(GOLD.read_text(encoding="utf-8"))
    records = [json.loads(path.read_text(encoding="utf-8")) for path in sorted((ROOT / "assets").glob("*.json"))]
    completed = [row for row in records if not row.get("error")]
    expected_joints = int(gold["expected_joint_count"])
    valid_joints = sum(int(row["joint_level_geom_valid_count"]) for row in completed)
    reached_endpoints = sum(int(row["endpoint_reachability"].split("/")[0]) for row in completed)
    strict = {
        "schema_version": 1,
        "protocol": "t5_fcl_clearance_certificate_adaptive_strict_adjudication_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "raw_summary": str(ROOT / "summary.json"),
        "denominator_policy": "All frozen-gold assets/joints are denominators; timeout/error cases contribute zero passes.",
        "asset_count": int(gold["asset_count"]),
        "completed_assets": len(completed),
        "failed_or_timed_out_assets": int(gold["asset_count"]) - len(completed),
        "joint_count": expected_joints,
        "joint_level_geom_valid": f"{valid_joints}/{expected_joints}",
        "joint_level_geom_valid_rate": valid_joints / expected_joints,
        "asset_level_geom_valid": raw["asset_level_geom_valid"],
        "asset_level_geom_valid_rate": sum(bool(row["asset_level_geom_valid"]) for row in completed) / int(gold["asset_count"]),
        "full_range_collision_free": raw["full_range_collision_free"],
        "full_range_collision_free_rate": sum(bool(row["full_range_collision_free_on_evaluated_paths"]) for row in completed) / int(gold["asset_count"]),
        "endpoint_reachability": f"{reached_endpoints}/{2 * expected_joints}",
        "endpoint_reachability_rate": reached_endpoints / (2 * expected_joints),
        "collision_observed_assets": sum(int(row["discrete_collision_state_count"]) > 0 for row in completed),
        "uncertified_interval_assets": sum(int(row["uncertified_pair_interval_count"]) > 0 for row in completed),
        "collision_and_uncertified_assets": sum(
            int(row["discrete_collision_state_count"]) > 0
            and int(row["uncertified_pair_interval_count"]) > 0
            for row in completed
        ),
        "strict_invalid_or_uncertified_assets": int(gold["asset_count"]) - sum(
            bool(row["asset_level_geom_valid"]) for row in completed
        ),
        "evaluated_discrete_states": sum(int(row["evaluated_discrete_state_count"]) for row in completed),
        "discrete_collision_states": raw["discrete_collision_states"],
        "uncertified_pair_intervals": raw["uncertified_pair_intervals"],
        "continuous_pair_certificates": raw["continuous_pair_certificates"],
        "max_penetration_m": raw["max_penetration_m"],
        "minimum_clearance_m": raw["minimum_clearance_m"],
        "aor_mean": None,
        "aor_note": "Not reported because collisions, uncertified intervals, and one timeout occurred.",
        "failures": raw["failures"],
    }
    path = ROOT / "summary_strict.json"
    path.write_text(json.dumps(strict, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(strict, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
