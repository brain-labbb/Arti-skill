#!/usr/bin/env python3
"""Quality census over the full 10,787 visual+collision (no-validate) exports.

For every exported package the census records the export outcome and a set of
static URDF defect signals computed BEFORE any simulation:

  * placeholder / export failure
  * links without collision geometry
  * links without inertial (mass) data
  * links disconnected from the root link in the joint graph
  * revolute/prismatic joints without finite declared limits
  * revolute/prismatic joints without positive finite effort/velocity

The defect score is a documented fixed weighted sum used by the diagnostic
Table 5 resample to stratify assets by exported quality.
"""

from __future__ import annotations

import argparse
import json
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Any, Sequence
import xml.etree.ElementTree as ET

SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
EXP_ROOT = REPO_ROOT / "exp"
DEFAULT_EXPORT_DIR = (
    EXP_ROOT / "runtime/articraft_github_full10787_export_noverify_20260830"
)
MERGED_ROSTER = (
    EXP_ROOT
    / "runtime/articraft_github_merged_10787_20260827/rosters/merged/full_release_roster.jsonl"
)

WEIGHTS = {
    "export_failed": 100.0,
    "disconnected_links": 3.0,
    "links_without_collision": 2.0,
    "links_without_inertial": 1.0,
    "scalar_joints_without_limits": 1.0,
    "scalar_joints_without_effort_velocity": 0.5,
}


def _finite(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    if parsed != parsed or parsed in (float("inf"), float("-inf")):
        return None
    return parsed


def audit_package(package_dir: str, asset_id: str) -> dict[str, Any]:
    package = Path(package_dir)
    urdf = package / "model.urdf"
    record: dict[str, Any] = {"asset_id": asset_id}
    if not urdf.is_file():
        record.update(export_status="missing", defect_score=WEIGHTS["export_failed"])
        return record
    try:
        text = urdf.read_text(encoding="utf-8", errors="replace")
    except OSError:
        record.update(export_status="unreadable", defect_score=WEIGHTS["export_failed"])
        return record
    if text.startswith("ARTICRAFT_TABLE5_COMPILE_FAILED"):
        record.update(export_status="placeholder", defect_score=WEIGHTS["export_failed"])
        return record
    try:
        root = ET.fromstring(text)
    except ET.ParseError:
        record.update(export_status="unparseable", defect_score=WEIGHTS["export_failed"])
        return record
    links = root.findall("link")
    joints = root.findall("joint")
    link_names = [link.get("name") for link in links]
    links_with_collision = set()
    links_with_inertial = set()
    for link in links:
        name = link.get("name")
        if link.findall("collision"):
            links_with_collision.add(name)
        inertial = link.find("inertial")
        if inertial is not None and inertial.find("mass") is not None:
            mass = _finite(inertial.find("mass").get("value"))
            if mass is not None and mass > 0:
                links_with_inertial.add(name)
    adjacency: dict[str, set[str]] = {name: set() for name in link_names}
    parent_of: dict[str, str] = {}
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_link = parent.get("link")
        child_link = child.get("link")
        if parent_link in adjacency and child_link in adjacency:
            adjacency[parent_link].add(child_link)
            adjacency[child_link].add(parent_link)
            parent_of.setdefault(child_link, parent_link)
    roots = [name for name in link_names if name not in parent_of]
    # A well-formed URDF has exactly one root link; extra parent-less links are
    # disconnected floating components.  BFS from a single root only.
    root = roots[0] if roots else (link_names[0] if link_names else None)
    extra_roots = [name for name in roots[1:]]
    reachable: set[str] = set()
    stack = [root] if root is not None else []
    while stack:
        node = stack.pop()
        if node in reachable:
            continue
        reachable.add(node)
        stack.extend(adjacency.get(node, ()))
    disconnected = [name for name in link_names if name not in reachable] + extra_roots
    scalar_no_limits = 0
    scalar_no_effort_velocity = 0
    scalar_count = 0
    movable = 0
    for joint in joints:
        joint_type = joint.get("type")
        if joint_type in ("revolute", "prismatic", "continuous"):
            movable += 1
        if joint_type not in ("revolute", "prismatic"):
            continue
        scalar_count += 1
        limit = joint.find("limit")
        lower = _finite(limit.get("lower")) if limit is not None else None
        upper = _finite(limit.get("upper")) if limit is not None else None
        if lower is None or upper is None or not lower < upper:
            scalar_no_limits += 1
        effort_ok = False
        velocity_ok = False
        if limit is not None:
            effort = _finite(limit.get("effort"))
            velocity = _finite(limit.get("velocity"))
            effort_ok = effort is not None and effort > 0
            velocity_ok = velocity is not None and velocity > 0
        if not (effort_ok and velocity_ok):
            scalar_no_effort_velocity += 1
    counts = {
        "links": len(link_names),
        "joints": len(joints),
        "movable_joints": movable,
        "scalar_joints": scalar_count,
        "links_without_collision": len(link_names) - len(links_with_collision),
        "links_without_inertial": len(link_names) - len(links_with_inertial),
        "disconnected_links": len(disconnected),
        "scalar_joints_without_limits": scalar_no_limits,
        "scalar_joints_without_effort_velocity": scalar_no_effort_velocity,
    }
    inertial_fraction = (
        counts["links_without_inertial"] / len(link_names) if link_names else 0.0
    )
    counts["links_without_inertial_fraction"] = round(inertial_fraction, 4)
    score = (
        sum(WEIGHTS[key] * counts[key] for key in (
            "disconnected_links",
            "links_without_collision",
            "scalar_joints_without_limits",
            "scalar_joints_without_effort_velocity",
        ))
        + WEIGHTS["links_without_inertial"] * inertial_fraction * len(link_names)
    )
    record.update(export_status="success", counts=counts, defect_score=score)
    return record


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-dir", type=Path, default=DEFAULT_EXPORT_DIR)
    parser.add_argument("--workers", type=int, default=32)
    arguments = parser.parse_args(argv)
    export_dir = arguments.export_dir.resolve(strict=True)
    staging = export_dir / "staging/data/cache/record_materialization"
    asset_ids: list[str] = []
    with MERGED_ROSTER.open("r", encoding="utf-8") as handle:
        for line in handle:
            asset_ids.append(str(json.loads(line)["asset_id"]))
    results: dict[str, dict[str, Any]] = {}
    with ProcessPoolExecutor(max_workers=arguments.workers) as executor:
        futures = {
            executor.submit(audit_package, str(staging / asset_id), asset_id): asset_id
            for asset_id in asset_ids
        }
        done = 0
        for future in as_completed(futures):
            result = future.result()
            results[str(result["asset_id"])] = result
            done += 1
            if done % 2000 == 0:
                print(f"census progress: {done}/{len(asset_ids)}", flush=True)
    ordered = [results[asset_id] for asset_id in asset_ids if asset_id in results]
    success = sum(1 for r in ordered if r["export_status"] == "success")
    out_path = export_dir / "quality_census.json"
    out_path.write_text(
        json.dumps(
            {
                "schema_version": "articraft_full10787_quality_census_v1",
                "weights": WEIGHTS,
                "total": len(asset_ids),
                "export_success": success,
                "export_failed": len(asset_ids) - success,
                "records": ordered,
            },
            sort_keys=True,
            indent=1,
        )
        + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "stage": "census_complete",
                "total": len(asset_ids),
                "export_success": success,
                "census": str(out_path),
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
