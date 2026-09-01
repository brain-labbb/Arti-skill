#!/usr/bin/env python3
"""Reclassify strict pose collisions using a q=0 penetration baseline."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any, Iterable

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import structural_integrity_eval as structural  # noqa: E402


def _pair(left: str, right: str) -> tuple[str, str]:
    return tuple(sorted((left, right)))


def _fixed_clusters(asset: structural.AssetModel) -> dict[str, str]:
    parent = {name: name for name in asset.links}

    def find(name: str) -> str:
        while parent[name] != name:
            parent[name] = parent[parent[name]]
            name = parent[name]
        return name

    def union(left: str, right: str) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    for joint in asset.joints:
        if joint.joint_type == "fixed":
            union(joint.parent, joint.child)
    return {name: find(name) for name in asset.links}


def _candidate_pairs(asset: structural.AssetModel) -> list[tuple[str, str]]:
    clusters = _fixed_clusters(asset)
    adjacent = {_pair(joint.parent, joint.child) for joint in asset.joints}
    names = sorted(asset.links)
    return [
        (left, right)
        for index, left in enumerate(names)
        for right in names[index + 1 :]
        if _pair(left, right) not in adjacent and clusters[left] != clusters[right]
    ]


def _q0_depths(asset: structural.AssetModel) -> tuple[dict[tuple[str, str], float], list[str]]:
    transforms = structural._fk(asset, {})
    meshes = {
        name: structural._world_mesh(geometry.mesh, transforms[name])
        for name, geometry in asset.links.items()
    }
    depths: dict[tuple[str, str], float] = {}
    errors: list[str] = []
    for left, right in _candidate_pairs(asset):
        left_mesh, right_mesh = meshes[left], meshes[right]
        if np.any(left_mesh.bounds[1] < right_mesh.bounds[0]) or np.any(
            right_mesh.bounds[1] < left_mesh.bounds[0]
        ):
            depths[(left, right)] = 0.0
            continue
        try:
            collided, depth = structural._fcl_collision(left_mesh, right_mesh)
            depths[(left, right)] = float(depth) if collided else 0.0
        except Exception as exc:
            errors.append(f"{left}|{right}:{type(exc).__name__}:{exc}")
    return depths, errors


def _world_diagonal(asset: structural.AssetModel) -> float:
    transforms = structural._fk(asset, {})
    bounds = [
        structural._world_mesh(geometry.mesh, transforms[name]).bounds
        for name, geometry in asset.links.items()
    ]
    low = np.min(np.vstack([item[0] for item in bounds]), axis=0)
    high = np.max(np.vstack([item[1] for item in bounds]), axis=0)
    diagonal = float(np.linalg.norm(high - low))
    if not math.isfinite(diagonal) or diagonal <= 0:
        raise ValueError("invalid q=0 asset diagonal")
    return diagonal


def _read_jsonl(path: Path) -> Iterable[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                yield json.loads(line)


def evaluate(
    record: dict[str, Any],
    *,
    absolute_fraction: float,
    growth_fraction: float,
) -> dict[str, Any]:
    asset = structural._parse_asset(
        Path(record["urdf_path"]), Path(record["package_root"]), "collision"
    )
    if any(geometry.mesh is None for geometry in asset.links.values()):
        raise ValueError("one or more links have no collision/visual-fallback geometry")
    diagonal = _world_diagonal(asset)
    baseline, baseline_errors = _q0_depths(asset)
    candidates = set(baseline)
    baseline_fraction = {pair: depth / diagonal for pair, depth in baseline.items()}
    initial_pairs = [
        {"left": pair[0], "right": pair[1], "depth_fraction": depth}
        for pair, depth in baseline_fraction.items()
        if depth > absolute_fraction
    ]
    pose_rows: list[dict[str, Any]] = []
    positive_growths: list[float] = []
    for pose in record.get("pose_swept", {}).get("rows", []):
        induced: list[dict[str, Any]] = []
        errors: list[dict[str, Any]] = []
        for event in pose.get("illegal_collision_pairs", []):
            pair = _pair(str(event.get("left")), str(event.get("right")))
            if pair not in candidates:
                continue
            if event.get("status") == "not_evaluable":
                errors.append(event)
                continue
            depth = float(event.get("depth_fraction", 0.0))
            growth = max(0.0, depth - baseline_fraction.get(pair, 0.0))
            if growth > 0:
                positive_growths.append(growth)
            if depth > absolute_fraction and growth > growth_fraction:
                induced.append(
                    {
                        "left": pair[0],
                        "right": pair[1],
                        "depth_fraction": depth,
                        "baseline_depth_fraction": baseline_fraction.get(pair, 0.0),
                        "growth_fraction": growth,
                    }
                )
        pose_rows.append(
            {
                "joint": pose.get("joint"),
                "pose_index": pose.get("pose_index"),
                "q": pose.get("q"),
                "free": not induced and not errors,
                "induced_pairs": induced,
                "errors": errors,
            }
        )
    free_poses = sum(bool(row["free"]) for row in pose_rows)
    return {
        "dataset_slug": record.get("dataset_slug"),
        "asset_id": record.get("asset_id"),
        "status": "complete" if not baseline_errors else "partial",
        "asset_diagonal_m": diagonal,
        "candidate_pairs": len(candidates),
        "baseline_errors": baseline_errors,
        "initial_interpenetrating_pairs": initial_pairs,
        "initial_interpenetration_free": not initial_pairs and not baseline_errors,
        "candidate_poses": len(pose_rows),
        "free_poses": free_poses,
        "pose_free_rate": free_poses / len(pose_rows) if pose_rows else None,
        "motion_induced_collision_free": bool(pose_rows)
        and free_poses == len(pose_rows)
        and not baseline_errors,
        "positive_growth_p95_fraction": float(np.percentile(positive_growths, 95))
        if positive_growths
        else 0.0,
        "positive_growth_event_count": len(positive_growths),
        "pose_rows": pose_rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--records", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--absolute-fraction", type=float, default=0.002)
    parser.add_argument("--growth-fraction", type=float, default=0.002)
    args = parser.parse_args()
    rows: list[dict[str, Any]] = []
    for index, record in enumerate(_read_jsonl(args.records), start=1):
        try:
            result = evaluate(
                record,
                absolute_fraction=args.absolute_fraction,
                growth_fraction=args.growth_fraction,
            )
        except Exception as exc:
            result = {
                "dataset_slug": record.get("dataset_slug"),
                "asset_id": record.get("asset_id"),
                "status": "error",
                "reason": f"{type(exc).__name__}: {exc}",
            }
        rows.append(result)
        print(f"{index} {result.get('asset_id')}: {result.get('status')}", flush=True)
    complete = [row for row in rows if row.get("status") == "complete"]
    pose_count = sum(int(row.get("candidate_poses", 0)) for row in complete)
    free_pose_count = sum(int(row.get("free_poses", 0)) for row in complete)
    pose_rates = [
        float(row["pose_free_rate"])
        for row in complete
        if isinstance(row.get("pose_free_rate"), (int, float))
    ]
    asset_pass = sum(bool(row.get("motion_induced_collision_free")) for row in complete)
    initial_pass = sum(bool(row.get("initial_interpenetration_free")) for row in complete)
    growth_p95_values = [
        float(row["positive_growth_p95_fraction"])
        for row in complete
        if isinstance(row.get("positive_growth_p95_fraction"), (int, float))
    ]
    summary = {
        "protocol": "motion-induced-collision-q0-baseline-v1-pilot",
        "input_records": str(args.records.resolve()),
        "asset_count": len(rows),
        "complete_asset_count": len(complete),
        "absolute_fraction": args.absolute_fraction,
        "growth_fraction": args.growth_fraction,
        "initial_interpenetration_free": {
            "passed": initial_pass,
            "denominator": len(complete),
            "percentage": 100 * initial_pass / len(complete) if complete else None,
        },
        "motion_induced_collision_free": {
            "passed": asset_pass,
            "denominator": len(complete),
            "percentage": 100 * asset_pass / len(complete) if complete else None,
            "free_poses": free_pose_count,
            "candidate_poses": pose_count,
            "pose_macro_percentage": 100 * statistics.mean(pose_rates)
            if pose_rates
            else None,
            "pose_micro_percentage": 100 * free_pose_count / pose_count
            if pose_count
            else None,
        },
        "asset_positive_growth_p95_fraction": {
            "median": statistics.median(growth_p95_values) if growth_p95_values else None,
            "p95": float(np.percentile(growth_p95_values, 95)) if growth_p95_values else None,
        },
    }
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (args.out / "asset_records.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=True, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=True, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
