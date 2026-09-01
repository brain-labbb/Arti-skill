#!/usr/bin/env python3
"""Audit the locally materialized Artiverse pre-release for Nano3D Table 6.

The static phase is dependency-free and operates on manifest-defined roots.  It
never treats annotation-to-URDF agreement as independent semantic accuracy.
The functional phase runs each selected URDF in a fresh subprocess because a
malformed simulator export can abort the native PyBullet process.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
DEFAULT_DATASET = REPO / "exp/artiverse"
DEFAULT_OUTPUT = REPO / "exp/runtime/table6_artiverse"
SALT = "nano3d-table6-artiverse-v1"
SINGLE_SAMPLES = 11
MULTI_SAMPLES = 64
GENERIC_REVOLUTE_DEG = 300.0
ZERO_RANGE_TOLERANCE = 1e-12
PENETRATION_TOLERANCE_M = 1e-6
SUPPORTED_PYBULLET_TYPES = {"revolute", "prismatic", "continuous"}


def safe(path: Path, *, must_exist: bool = True) -> Path:
    root = WORKSPACE.resolve(strict=True)
    resolved = path.resolve(strict=must_exist)
    if resolved != root and root not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def rel(path: Path) -> str:
    return safe(path, must_exist=False).relative_to(WORKSPACE).as_posix()


def read_json(path: Path) -> Any:
    return json.loads(safe(path).read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    destination = safe(path, must_exist=False)
    safe(destination.parent).mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)


def write_text(path: Path, value: str) -> None:
    destination = safe(path, must_exist=False)
    safe(destination.parent).mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(destination)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with safe(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stable_hash(value: str) -> str:
    return hashlib.sha256(f"{SALT}|{value}".encode()).hexdigest()


def finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def finite_vector(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 3
        and all(finite_number(component) for component in value)
    )


def xml_vector(node: ET.Element | None, field: str, *, default_ok: bool) -> bool:
    if node is None or not node.get(field):
        return default_ok
    try:
        values = [float(value) for value in node.get(field, "").split()]
    except ValueError:
        return False
    return len(values) == 3 and all(math.isfinite(value) for value in values)


def nonzero_xml_axis(node: ET.Element | None) -> bool:
    if not xml_vector(node, "xyz", default_ok=False):
        return False
    values = [float(value) for value in node.get("xyz", "").split()]  # type: ignore[union-attr]
    return sum(value * value for value in values) > 1e-18


def chunk_roots(manifest: dict[str, Any], chunk_index: int) -> list[str]:
    matches = [entry for entry in manifest["chunks"] if int(entry["index"]) == chunk_index]
    if len(matches) != 1:
        raise RuntimeError(f"expected exactly one chunk index {chunk_index}, found {len(matches)}")
    roots = [str(value) for value in matches[0]["roots"]]
    if len(roots) != int(matches[0]["model_count"]) or len(set(roots)) != len(roots):
        raise RuntimeError("chunk root count or uniqueness does not match manifest")
    return roots


def model_identity(root_rel: str) -> tuple[str, str, str]:
    parts = Path(root_rel).parts
    if len(parts) != 4 or parts[0] != "data":
        raise RuntimeError(f"unexpected model root: {root_rel}")
    return parts[1], parts[2], parts[3]


def one_matching(root: Path, pattern: str) -> Path | None:
    values = sorted(root.glob(pattern))
    return values[0] if len(values) == 1 else None


def graph_audit(links: set[str], edges: list[tuple[str, str]]) -> dict[str, Any]:
    indegree = Counter(child for _, child in edges)
    adjacency: dict[str, list[str]] = defaultdict(list)
    for parent, child in edges:
        adjacency[parent].append(child)
    roots = sorted(link for link in links if indegree[link] == 0)
    seen: set[str] = set()
    active: set[str] = set()
    cycle = False

    def visit(node: str) -> None:
        nonlocal cycle
        if node in active:
            cycle = True
            return
        if node in seen:
            return
        active.add(node)
        for child in adjacency.get(node, []):
            visit(child)
        active.remove(node)
        seen.add(node)

    for link in sorted(links):
        visit(link)
    reachable: set[str] = set()
    stack = list(roots)
    while stack:
        node = stack.pop()
        if node in reachable:
            continue
        reachable.add(node)
        stack.extend(adjacency.get(node, []))
    return {
        "root_count": len(roots),
        "multi_parent_link_count": sum(value > 1 for value in indegree.values()),
        "cycle": cycle,
        "connected": len(roots) == 1 and len(reachable) == len(links),
        "valid_tree": (
            len(roots) == 1
            and not cycle
            and not any(value > 1 for value in indegree.values())
            and len(edges) == max(0, len(links) - 1)
        ),
    }


def annotation_audit(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    data = read_json(path)
    articulations = data.get("articulations", [])
    if not isinstance(articulations, list):
        raise RuntimeError("articulations is not a list")
    rows: list[dict[str, Any]] = []
    for item in articulations:
        joint_type = str(item.get("type", "")).lower()
        range_keys = (
            ("rangeMin", "rangeMax"),
            ("prismatic_rangeMin", "prismatic_rangeMax"),
            ("transRangeMin", "transRangeMax"),
        )
        range_valid = False
        for lower_key, upper_key in range_keys:
            if lower_key not in item and upper_key not in item:
                continue
            lower, upper = item.get(lower_key), item.get(upper_key)
            range_valid = finite_number(lower) and finite_number(upper) and lower <= upper
            break
        axis = item.get("axis")
        rows.append(
            {
                "pid": item.get("pid"),
                "type": joint_type,
                "base_present": isinstance(item.get("base"), list) and bool(item.get("base")),
                "axis_present": "axis" in item,
                "axis_valid": finite_vector(axis) and sum(float(x) ** 2 for x in axis) > 1e-18
                if finite_vector(axis)
                else False,
                "origin_present": "origin" in item,
                "origin_valid": finite_vector(item.get("origin")),
                "range_present": any(
                    lower in item or upper in item for lower, upper in range_keys
                ),
                "range_valid": range_valid,
            }
        )
    graph = data.get("graphParentByPid")
    connectivity = data.get("connectivity")
    return (
        {
            "annotation_parse_success": True,
            "annotation_model_id": data.get("modelId"),
            "annotation_worker_id_present": bool(data.get("workerId")),
            "annotation_joint_count": len(rows),
            "annotation_graph_parent_present": isinstance(graph, dict),
            "annotation_connectivity_present": isinstance(connectivity, list),
        },
        rows,
    )


def urdf_audit(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    root = ET.parse(safe(path)).getroot()
    links = {node.get("name", "") for node in root.findall("link") if node.get("name")}
    joints: list[dict[str, Any]] = []
    edges: list[tuple[str, str]] = []
    for node in root.findall("joint"):
        parent_node, child_node = node.find("parent"), node.find("child")
        parent = parent_node.get("link", "") if parent_node is not None else ""
        child = child_node.get("link", "") if child_node is not None else ""
        edges.append((parent, child))
        joint_type = node.get("type", "")
        if joint_type == "fixed":
            continue
        limit = node.find("limit")
        bounded = joint_type in {"revolute", "prismatic"}
        limit_valid = not bounded
        lower = upper = None
        if bounded:
            try:
                lower = float(limit.get("lower"))  # type: ignore[union-attr]
                upper = float(limit.get("upper"))  # type: ignore[union-attr]
                limit_valid = math.isfinite(lower) and math.isfinite(upper) and lower <= upper
            except (AttributeError, TypeError, ValueError):
                limit_valid = False
        original_type = node.get("artiverse_original_type", joint_type)
        motion_sweepable = bool(
            joint_type == "continuous"
            or (
                bounded
                and limit_valid
                and lower is not None
                and upper is not None
                and upper - lower > ZERO_RANGE_TOLERANCE
            )
        )
        joints.append(
            {
                "name": node.get("name", ""),
                "type": joint_type,
                "original_type": original_type,
                "component": node.get("artiverse_component"),
                "parent": parent,
                "child": child,
                "parent_child_structurally_valid": parent in links and child in links,
                "axis_metadata_valid": nonzero_xml_axis(node.find("axis")),
                "origin_metadata_valid": (
                    xml_vector(node.find("origin"), "xyz", default_ok=True)
                    and xml_vector(node.find("origin"), "rpy", default_ok=True)
                ),
                "bounded": bounded,
                "limit_metadata_valid": limit_valid,
                "lower": lower,
                "upper": upper,
                "motion_sweepable": motion_sweepable,
                "zero_range": bool(
                    bounded
                    and limit_valid
                    and lower is not None
                    and upper is not None
                    and upper - lower <= ZERO_RANGE_TOLERANCE
                ),
                "generic_revolute_range": bool(
                    original_type == "revolute"
                    and joint_type == "revolute"
                    and limit_valid
                    and lower is not None
                    and upper is not None
                    and math.degrees(upper - lower) >= GENERIC_REVOLUTE_DEG
                ),
            }
        )
    collision_elements = root.findall(".//collision")
    collision_mesh_nodes = root.findall(".//collision/geometry/mesh")
    visual_mesh_nodes = root.findall(".//visual/geometry/mesh")
    mesh_nodes = collision_mesh_nodes + visual_mesh_nodes
    missing_meshes: list[str] = []
    for node in mesh_nodes:
        filename = node.get("filename", "")
        candidate = safe(path.parent / filename, must_exist=False)
        if not candidate.is_file():
            missing_meshes.append(filename)
    graph = graph_audit(links, edges)
    return (
        {
            "urdf_parse_success": True,
            "urdf_link_count": len(links),
            "urdf_joint_count": len(root.findall("joint")),
            "urdf_movable_dof_count": len(joints),
            "urdf_collision_element_count": len(collision_elements),
            "urdf_collision_mesh_count": len(collision_mesh_nodes),
            "urdf_visual_mesh_count": len(visual_mesh_nodes),
            "urdf_inertial_count": len(root.findall(".//inertial")),
            "urdf_missing_mesh_reference_count": len(missing_meshes),
            "urdf_missing_mesh_references": missing_meshes[:16],
            **graph,
        },
        joints,
    )


def audit_root(dataset: Path, root_rel: str) -> dict[str, Any]:
    category, source, model_id = model_identity(root_rel)
    root = safe(dataset / Path(root_rel), must_exist=False)
    record: dict[str, Any] = {
        "asset_id": f"{category}/{source}/{model_id}",
        "root": root_rel,
        "category": category,
        "source": source,
        "model_id": model_id,
        "complete_core_package": False,
        "annotation_parse_success": False,
        "urdf_parse_success": False,
        "issues": [],
    }
    annotation = one_matching(root, "*.articulations.json") if root.is_dir() else None
    glb = one_matching(root, "*.segmented.glb") if root.is_dir() else None
    urdf = one_matching(root, "urdf_w_collider/*.urdf") if root.is_dir() else None
    material = root / "material.json"
    record.update(
        {
            "annotation_present": annotation is not None,
            "segmented_glb_present": glb is not None,
            "urdf_present": urdf is not None,
            "material_present": material.is_file(),
            "mass_furniture_heuristic_present": (root / "mass_furniture_heuristic.json").is_file(),
        }
    )
    record["complete_core_package"] = all((annotation, glb, urdf))
    annotation_rows: list[dict[str, Any]] = []
    urdf_rows: list[dict[str, Any]] = []
    if annotation is not None:
        try:
            summary, annotation_rows = annotation_audit(annotation)
            record.update(summary)
            record["annotation_sha256"] = sha256_file(annotation)
        except Exception as exc:  # noqa: BLE001
            record["issues"].append(f"annotation: {type(exc).__name__}: {exc}")
    if urdf is not None:
        try:
            summary, urdf_rows = urdf_audit(urdf)
            record.update(summary)
            record["urdf_path"] = rel(urdf)
            record["urdf_sha256"] = sha256_file(urdf)
        except Exception as exc:  # noqa: BLE001
            record["issues"].append(f"urdf: {type(exc).__name__}: {exc}")
    annotation_types = Counter(row["type"] for row in annotation_rows)
    urdf_types = Counter(row["type"] for row in urdf_rows)
    record.update(
        {
            "annotation_type_counts": dict(sorted(annotation_types.items())),
            "annotation_axis_present_count": sum(row["axis_present"] for row in annotation_rows),
            "annotation_axis_valid_count": sum(row["axis_valid"] for row in annotation_rows),
            "annotation_origin_present_count": sum(row["origin_present"] for row in annotation_rows),
            "annotation_origin_valid_count": sum(row["origin_valid"] for row in annotation_rows),
            "annotation_range_present_count": sum(row["range_present"] for row in annotation_rows),
            "annotation_range_valid_count": sum(row["range_valid"] for row in annotation_rows),
            "urdf_type_counts": dict(sorted(urdf_types.items())),
            "urdf_original_type_counts": dict(
                sorted(Counter(row["original_type"] for row in urdf_rows).items())
            ),
            "urdf_parent_child_structurally_valid_count": sum(
                row["parent_child_structurally_valid"] for row in urdf_rows
            ),
            "urdf_axis_metadata_valid_count": sum(row["axis_metadata_valid"] for row in urdf_rows),
            "urdf_origin_metadata_valid_count": sum(row["origin_metadata_valid"] for row in urdf_rows),
            "urdf_bounded_joint_count": sum(row["bounded"] for row in urdf_rows),
            "urdf_limit_metadata_valid_count": sum(
                row["bounded"] and row["limit_metadata_valid"] for row in urdf_rows
            ),
            "urdf_generic_revolute_count": sum(row["generic_revolute_range"] for row in urdf_rows),
            "urdf_native_simple_dof_count": sum(
                row["type"] in SUPPORTED_PYBULLET_TYPES
                and row["original_type"] in SUPPORTED_PYBULLET_TYPES
                for row in urdf_rows
            ),
            "urdf_lossy_complex_dof_count": sum(
                row["original_type"] not in SUPPORTED_PYBULLET_TYPES for row in urdf_rows
            ),
            "urdf_motion_sweepable_dof_count": sum(
                row["motion_sweepable"] for row in urdf_rows
            ),
            "urdf_zero_range_dof_count": sum(row["zero_range"] for row in urdf_rows),
        }
    )
    return record


def sum_field(records: Iterable[dict[str, Any]], field: str) -> int:
    return sum(int(record.get(field, 0) or 0) for record in records)


def aggregate_static(records: list[dict[str, Any]]) -> dict[str, Any]:
    annotation_types: Counter[str] = Counter()
    urdf_types: Counter[str] = Counter()
    original_types: Counter[str] = Counter()
    for record in records:
        annotation_types.update(record.get("annotation_type_counts", {}))
        urdf_types.update(record.get("urdf_type_counts", {}))
        original_types.update(record.get("urdf_original_type_counts", {}))
    annotation_joints = sum_field(records, "annotation_joint_count")
    urdf_dofs = sum_field(records, "urdf_movable_dof_count")
    return {
        "asset_count": len(records),
        "category_count": len({record["category"] for record in records}),
        "source_count": len({record["source"] for record in records}),
        "complete_core_package_count": sum(record["complete_core_package"] for record in records),
        "annotation_parse_success_count": sum(record["annotation_parse_success"] for record in records),
        "urdf_parse_success_count": sum(record["urdf_parse_success"] for record in records),
        "annotation_articulable_asset_count": sum(
            int(record.get("annotation_joint_count", 0)) > 0 for record in records
        ),
        "urdf_articulable_asset_count": sum(
            int(record.get("urdf_movable_dof_count", 0)) > 0 for record in records
        ),
        "annotation_joint_count": annotation_joints,
        "annotation_joints_per_asset_mean": annotation_joints / len(records) if records else None,
        "annotation_type_counts": dict(sorted(annotation_types.items())),
        "annotation_axis_present_count": sum_field(records, "annotation_axis_present_count"),
        "annotation_axis_valid_count": sum_field(records, "annotation_axis_valid_count"),
        "annotation_origin_present_count": sum_field(records, "annotation_origin_present_count"),
        "annotation_origin_valid_count": sum_field(records, "annotation_origin_valid_count"),
        "annotation_range_present_count": sum_field(records, "annotation_range_present_count"),
        "annotation_range_valid_count": sum_field(records, "annotation_range_valid_count"),
        "annotation_graph_parent_asset_count": sum(
            record.get("annotation_graph_parent_present", False) for record in records
        ),
        "annotation_connectivity_asset_count": sum(
            record.get("annotation_connectivity_present", False) for record in records
        ),
        "urdf_movable_dof_count": urdf_dofs,
        "urdf_dofs_per_asset_mean": urdf_dofs / len(records) if records else None,
        "urdf_type_counts": dict(sorted(urdf_types.items())),
        "urdf_original_type_counts": dict(sorted(original_types.items())),
        "urdf_native_simple_dof_count": sum_field(records, "urdf_native_simple_dof_count"),
        "urdf_lossy_complex_dof_count": sum_field(records, "urdf_lossy_complex_dof_count"),
        "urdf_motion_sweepable_dof_count": sum_field(
            records, "urdf_motion_sweepable_dof_count"
        ),
        "urdf_zero_range_dof_count": sum_field(records, "urdf_zero_range_dof_count"),
        "urdf_parent_child_structurally_valid_count": sum_field(
            records, "urdf_parent_child_structurally_valid_count"
        ),
        "urdf_axis_metadata_valid_count": sum_field(records, "urdf_axis_metadata_valid_count"),
        "urdf_origin_metadata_valid_count": sum_field(records, "urdf_origin_metadata_valid_count"),
        "urdf_bounded_joint_count": sum_field(records, "urdf_bounded_joint_count"),
        "urdf_limit_metadata_valid_count": sum_field(records, "urdf_limit_metadata_valid_count"),
        "urdf_generic_revolute_count": sum_field(records, "urdf_generic_revolute_count"),
        "urdf_valid_tree_asset_count": sum(record.get("valid_tree", False) for record in records),
        "urdf_cycle_asset_count": sum(record.get("cycle", False) for record in records),
        "urdf_multi_parent_asset_count": sum(
            int(record.get("multi_parent_link_count", 0)) > 0 for record in records
        ),
        "urdf_collision_element_count": sum_field(records, "urdf_collision_element_count"),
        "urdf_collision_available_asset_count": sum(
            int(record.get("urdf_collision_element_count", 0)) > 0 for record in records
        ),
        "urdf_inertial_count": sum_field(records, "urdf_inertial_count"),
        "urdf_missing_mesh_reference_count": sum_field(records, "urdf_missing_mesh_reference_count"),
        "material_asset_count": sum(record.get("material_present", False) for record in records),
        "mass_furniture_heuristic_asset_count": sum(
            record.get("mass_furniture_heuristic_present", False) for record in records
        ),
    }


def freeze_functional_cohort(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_category: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        by_category[record["category"]].append(record)
    selected: list[dict[str, Any]] = []
    for category, candidates in sorted(by_category.items()):
        winner = min(candidates, key=lambda row: stable_hash(row["asset_id"]))
        selected.append(
            {
                "asset_id": winner["asset_id"],
                "category": category,
                "source": winner["source"],
                "model_id": winner["model_id"],
                "root": winner["root"],
                "urdf_path": winner.get("urdf_path"),
                "urdf_sha256": winner.get("urdf_sha256"),
                "selection_hash": stable_hash(winner["asset_id"]),
                "static_valid_tree": winner.get("valid_tree", False),
                "static_movable_dofs": winner.get("urdf_movable_dof_count", 0),
                "static_motion_sweepable_dofs": winner.get(
                    "urdf_motion_sweepable_dof_count", 0
                ),
                "static_zero_range_dofs": winner.get("urdf_zero_range_dof_count", 0),
                "static_lossy_complex_dofs": winner.get("urdf_lossy_complex_dof_count", 0),
            }
        )
    return selected


def run_static(args: argparse.Namespace) -> int:
    dataset = safe(args.dataset)
    output = safe(args.output, must_exist=False)
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = dataset / "dataset_chunks/manifest.json"
    manifest = read_json(manifest_path)
    selected_indices = (
        [int(entry["index"]) for entry in manifest["chunks"]]
        if args.all_chunks
        else [args.chunk_index]
    )
    roots = [
        root
        for chunk_index in selected_indices
        for root in chunk_roots(manifest, chunk_index)
    ]
    if len(set(roots)) != len(roots):
        raise RuntimeError("selected manifest chunks contain duplicate roots")
    records = [audit_root(dataset, root) for root in roots]
    aggregate = aggregate_static(records)
    complete = aggregate["complete_core_package_count"] == len(roots)
    status = "COMPLETE" if complete else "BLOCKED_INCOMPLETE_MATERIALIZATION"
    cohort = freeze_functional_cohort(records) if complete else []
    summary = {
        "protocol_id": "nano3d_table6_artiverse_release_v1",
        "status": status,
        "method": "Artiverse",
        "role": "human-verified pre-release real-data reference",
        "paper_values_reused": False,
        "release": {
            "dataset_revision": "8c4b120418e7cbdf9ac4c9580c5dbfdbf128a248",
            "dataset_card_license": "other",
            "access": "manual gated; locally materialized by authorized user",
            "manifest_path": rel(manifest_path),
            "manifest_sha256": sha256_file(manifest_path),
            "manifest_model_count": manifest["model_count"],
            "manifest_category_count": len({model_identity(root)[0] for entry in manifest["chunks"] for root in entry["roots"]}),
            "manifest_archive_bytes": sum(entry["archive_bytes"] for entry in manifest["chunks"]),
            "snapshot_state": "pre-release subset; README says cleanup is ongoing",
        },
        "scope": {
            "chunk_index": "all" if args.all_chunks else args.chunk_index,
            "chunk_indices": selected_indices,
            "manifest_defined_roots": len(roots),
            "selection": "all roots listed in the selected manifest chunk; no outcome filtering",
            "functional_selection": (
                "one asset per category by argmin SHA256(salt|asset_id), frozen from the complete manifest before and without filtering on static validity or PyBullet outcomes"
                if complete
                else "not frozen because core package materialization is incomplete"
            ),
            "salt": SALT,
        },
        "static": aggregate,
        "semantic_metric_boundary": {
            "joint_type_accuracy": "N/A for Artiverse itself: dataset reference, not a prediction",
            "joint_recall": "N/A for Artiverse itself: annotation and URDF share a release pipeline",
            "parent_child_accuracy": "N/A for Artiverse itself: structural validity is reported separately",
            "axis_valid": "N/A semantic; annotation and URDF metadata coverage are reported separately",
            "origin_valid": "N/A semantic; annotation and URDF metadata coverage are reported separately",
            "limit_valid": "N/A semantic; annotation and URDF metadata coverage are reported separately",
        },
        "limitations": [
            "Artiverse is a human-verified pre-release dataset reference, not a generated prediction method.",
            "Annotation-to-URDF agreement is export fidelity, not independent semantic accuracy.",
            "Complex joints can be split or downgraded in standard URDF; logical annotation joints and simulator DoFs are reported separately.",
            "The release card has license=other and requires acceptance of upstream-source licenses.",
        ],
    }
    write_json(output / "asset_records.json", records)
    write_json(output / "summary.json", summary)
    write_json(
        output / "frozen_functional_cohort.json",
        {
            "protocol_id": "nano3d_table6_artiverse_functional_cohort_v1",
            "status": "FROZEN" if complete else "NOT_FROZEN",
            "salt": SALT,
            "size": len(cohort),
            "items": cohort,
        },
    )
    write_json(
        output / "preflight.json",
        {
            "protocol_id": "nano3d_table6_artiverse_preflight_v3",
            "status": (
                "READY_LOCAL_GATED_SNAPSHOT_WITH_RELEASE_DEFECTS"
                if complete and aggregate["urdf_parse_success_count"] != len(records)
                else "READY_LOCAL_GATED_SNAPSHOT"
                if complete
                else status
            ),
            "method": "Artiverse",
            "role": "human-verified pre-release real-data reference",
            "evaluated_asset_count": len(records) if complete else 0,
            "paper_values_reused": False,
            "gpu_task_launched": False,
            "access": {
                "dataset_revision": summary["release"]["dataset_revision"],
                "dataset_card_license": "other",
                "manual_gate": True,
                "locally_materialized": True,
                "manifest_sha256": summary["release"]["manifest_sha256"],
                "archive_bytes": summary["release"]["manifest_archive_bytes"],
            },
            "gates": {
                "manifest_defined_roots_complete": complete,
                "annotation_carrier_verified": aggregate["annotation_parse_success_count"] == len(records),
                "collision_urdf_verified": aggregate["urdf_parse_success_count"] == len(records),
                "mesh_references_complete": aggregate["urdf_missing_mesh_reference_count"] == 0,
                "functional_cohort_frozen": bool(cohort),
                "independent_prediction_gold_applicable_to_reference_row": False,
            },
            "claim_boundary": "Artiverse self-metrics are reference completeness/export health, not prediction accuracy.",
        },
    )
    print(json.dumps(summary, indent=2, ensure_ascii=True))
    return 0 if complete else 2


def sample_values(joint: dict[str, Any]) -> list[float]:
    if joint["type"] in {"revolute", "prismatic"}:
        lower, upper = float(joint["lower"]), float(joint["upper"])
    else:
        lower, upper = -math.pi, math.pi
    return [lower + index * (upper - lower) / (SINGLE_SAMPLES - 1) for index in range(SINGLE_SAMPLES)]


def run_self_collision_flag_smoke(output: Path) -> dict[str, Any]:
    fixture = output / "self_collision_flag_smoke.urdf"
    write_text(
        fixture,
        """<?xml version="1.0"?>
<robot name="self_collision_flag_smoke">
  <link name="base"/>
  <link name="left"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="right"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <joint name="left_joint" type="fixed"><parent link="base"/><child link="left"/></joint>
  <joint name="right_joint" type="fixed"><parent link="base"/><child link="right"/></joint>
</robot>
""",
    )
    import pybullet as bullet

    variants = {
        "none": 0,
        "self_collision": bullet.URDF_USE_SELF_COLLISION,
        "exclude_parent_only": bullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        "self_plus_exclude_parent": (
            bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
        ),
    }
    contacts: dict[str, int] = {}
    client = bullet.connect(bullet.DIRECT)
    try:
        for name, flags in variants.items():
            bullet.resetSimulation(physicsClientId=client)
            body = bullet.loadURDF(
                str(fixture), useFixedBase=True, flags=flags, physicsClientId=client
            )
            bullet.performCollisionDetection(physicsClientId=client)
            contacts[name] = len(
                bullet.getContactPoints(
                    bodyA=body, bodyB=body, physicsClientId=client
                )
            )
    finally:
        bullet.disconnect(client)
    passed = (
        contacts["none"] == 0
        and contacts["exclude_parent_only"] == 0
        and contacts["self_collision"] > 0
        and contacts["self_plus_exclude_parent"] > 0
    )
    payload = {
        "protocol_id": "nano3d_table6_pybullet_self_collision_flag_smoke_v2",
        "status": "PASS" if passed else "FAIL",
        "pybullet_api_version": int(bullet.getAPIVersion()),
        "fixture": fixture.name,
        "fixture_design": "two overlapping sibling collision boxes attached to a common fixed parent",
        "contacts": contacts,
        "conclusion": (
            "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT does not enable self-collision; "
            "the formal sweep combines it with URDF_USE_SELF_COLLISION."
        ),
    }
    write_json(output / "self_collision_flag_smoke.json", payload)
    if not passed:
        raise RuntimeError(f"self-collision flag smoke failed: {contacts}")
    return payload


def child_payload(payload_path: Path, result_path: Path) -> int:
    payload = read_json(payload_path)
    urdf = safe(WORKSPACE / payload["urdf_path"])
    actual_urdf_sha256 = sha256_file(urdf)
    root, joints = urdf_audit(urdf)
    sweepable_dofs = sum(joint["motion_sweepable"] for joint in joints)
    expected_single_states = SINGLE_SAMPLES * sweepable_dofs
    expected_multi_states = MULTI_SAMPLES if sweepable_dofs > 1 else 0
    result: dict[str, Any] = {
        "asset_id": payload["asset_id"],
        "category": payload["category"],
        "source": payload["source"],
        "urdf_path": payload["urdf_path"],
        "urdf_sha256": actual_urdf_sha256,
        "static_valid_tree": root["valid_tree"],
        "movable_dof_count": len(joints),
        "motion_sweepable_dof_count": sweepable_dofs,
        "zero_range_dof_count": sum(joint["zero_range"] for joint in joints),
        "expected_single_state_count": expected_single_states,
        "expected_multi_state_count": expected_multi_states,
        "measurement_complete": False,
        "pybullet_load_success": False,
        "drive_success": False,
        "reference_state_contact_free": None,
        "reference_state_penetration_free": None,
        "single_state_count": 0,
        "single_contact_free_count": 0,
        "single_penetration_free_count": 0,
        "multi_state_count": 0,
        "multi_contact_free_count": 0,
        "multi_penetration_free_count": 0,
        "joint_single_contact_free_sweep_pass_count": 0,
        "joint_single_penetration_free_sweep_pass_count": 0,
        "asset_contact_free_proxy": False,
        "asset_penetration_free_proxy": False,
        "max_reset_pose_abs_error": 0.0,
        "max_penetration_depth": 0.0,
        "issues": [],
    }
    if actual_urdf_sha256 != payload.get("urdf_sha256"):
        result["issues"].append("static_gate: URDF SHA-256 differs from frozen cohort")
        write_json(result_path, result)
        return 0
    if not root["valid_tree"]:
        result["issues"].append("static_gate: URDF is not a single-root acyclic tree")
        write_json(result_path, result)
        return 0
    try:
        import pybullet as bullet
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"pybullet_import: {type(exc).__name__}: {exc}")
        write_json(result_path, result)
        return 0
    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = (
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        body = bullet.loadURDF(str(urdf), useFixedBase=True, flags=flags)
        result["pybullet_load_success"] = True
        by_name: dict[str, int] = {}
        for index in range(bullet.getNumJoints(body)):
            info = bullet.getJointInfo(body, index)
            name = info[1].decode() if isinstance(info[1], bytes) else str(info[1])
            by_name[name] = index
        if any(joint["name"] not in by_name for joint in joints):
            missing = [joint["name"] for joint in joints if joint["name"] not in by_name]
            result["issues"].append(f"joint_mapping_missing: {missing[:8]}")
            write_json(result_path, result)
            return 0
        indices = [by_name[joint["name"]] for joint in joints]
        sweep_positions = [
            position for position, joint in enumerate(joints) if joint["motion_sweepable"]
        ]
        defaults = [
            (float(joint["lower"]) + float(joint["upper"])) / 2
            if joint["bounded"]
            else 0.0
            for joint in joints
        ]

        for index in indices:
            bullet.setJointMotorControl2(
                body,
                index,
                bullet.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
            )

        def evaluate(values: list[float]) -> tuple[bool, bool, float]:
            for index, value in zip(indices, values):
                bullet.resetJointState(body, index, value)
            readback = [float(bullet.getJointState(body, index)[0]) for index in indices]
            reset_error = max(
                [abs(actual - target) for actual, target in zip(readback, values)] or [0.0]
            )
            result["max_reset_pose_abs_error"] = max(
                result["max_reset_pose_abs_error"], reset_error
            )
            if not all(math.isfinite(actual) for actual in readback) or reset_error > 1e-9:
                raise RuntimeError(f"joint reset readback mismatch: max_abs_error={reset_error}")
            bullet.performCollisionDetection()
            contacts = bullet.getContactPoints(bodyA=body, bodyB=body)
            penetration = max([max(0.0, -float(contact[8])) for contact in contacts] or [0.0])
            return not contacts, penetration <= PENETRATION_TOLERANCE_M, penetration

        values = list(defaults)
        reference_contact_free, reference_penetration_free, penetration = evaluate(values)
        result["reference_state_contact_free"] = reference_contact_free
        result["reference_state_penetration_free"] = reference_penetration_free
        result["max_penetration_depth"] = penetration
        all_contact_free = reference_contact_free
        all_penetration_free = reference_penetration_free
        for position in sweep_positions:
            joint = joints[position]
            joint_contact_free = True
            joint_penetration_free = True
            for value in sample_values(joint):
                values = list(defaults)
                values[position] = value
                contact_free, penetration_free, penetration = evaluate(values)
                result["single_state_count"] += 1
                result["single_contact_free_count"] += int(contact_free)
                result["single_penetration_free_count"] += int(penetration_free)
                result["max_penetration_depth"] = max(result["max_penetration_depth"], penetration)
                joint_contact_free = joint_contact_free and contact_free
                joint_penetration_free = joint_penetration_free and penetration_free
                all_contact_free = all_contact_free and contact_free
                all_penetration_free = all_penetration_free and penetration_free
            result["joint_single_contact_free_sweep_pass_count"] += int(joint_contact_free)
            result["joint_single_penetration_free_sweep_pass_count"] += int(
                joint_penetration_free
            )
        if len(sweep_positions) > 1:
            from scipy.stats import qmc

            unit = qmc.Sobol(d=len(sweep_positions), scramble=False).random_base2(m=6)
            for row in unit:
                values = list(defaults)
                for value, position in zip(row, sweep_positions):
                    joint = joints[position]
                    if joint["bounded"]:
                        lower, upper = float(joint["lower"]), float(joint["upper"])
                    else:
                        lower, upper = -math.pi, math.pi
                    values[position] = lower + float(value) * (upper - lower)
                contact_free, penetration_free, penetration = evaluate(values)
                result["multi_state_count"] += 1
                result["multi_contact_free_count"] += int(contact_free)
                result["multi_penetration_free_count"] += int(penetration_free)
                result["max_penetration_depth"] = max(result["max_penetration_depth"], penetration)
                all_contact_free = all_contact_free and contact_free
                all_penetration_free = all_penetration_free and penetration_free
        result["drive_success"] = result["max_reset_pose_abs_error"] <= 1e-9
        result["asset_contact_free_proxy"] = all_contact_free
        result["asset_penetration_free_proxy"] = all_penetration_free
        result["measurement_complete"] = bool(
            result["drive_success"]
            and result["single_state_count"] == expected_single_states
            and result["multi_state_count"] == expected_multi_states
            and not result["issues"]
        )
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"pybullet: {type(exc).__name__}: {exc}")
    write_json(result_path, result)
    # This function is only called by the isolated `_child` entry point, which
    # exits via os._exit after the durable result write. Avoid native teardown:
    # several high-link-count releases spend minutes in removeBody/disconnect
    # after all evaluated states are already complete.
    return 0


def run_functional(args: argparse.Namespace) -> int:
    output = safe(args.output)
    run_self_collision_flag_smoke(output)
    cohort = read_json(output / "frozen_functional_cohort.json")
    if cohort.get("status") != "FROZEN" or not cohort.get("items"):
        raise RuntimeError("functional cohort is not frozen")
    workers = output / "functional_workers"
    workers.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    for item in cohort["items"]:
        asset_hash = hashlib.sha256(item["asset_id"].encode()).hexdigest()[:16]
        payload = workers / f"{asset_hash}.input.json"
        result = workers / f"{asset_hash}.result.json"
        stderr = workers / f"{asset_hash}.stderr"
        write_json(payload, item)
        result.unlink(missing_ok=True)
        timed_out = False
        with stderr.open("wb") as error_stream:
            try:
                completed = subprocess.run(
                    [sys.executable, str(SCRIPT), "_child", "--payload", str(payload), "--result", str(result)],
                    cwd=REPO,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=error_stream,
                    timeout=args.timeout,
                    check=False,
                )
                returncode = completed.returncode
            except subprocess.TimeoutExpired:
                timed_out = True
                returncode = 124
        if result.is_file():
            record = read_json(result)
        else:
            record = {
                "asset_id": item["asset_id"],
                "category": item["category"],
                "source": item["source"],
                "movable_dof_count": item["static_movable_dofs"],
                "motion_sweepable_dof_count": item["static_motion_sweepable_dofs"],
                "zero_range_dof_count": item["static_zero_range_dofs"],
                "expected_single_state_count": (
                    SINGLE_SAMPLES * item["static_motion_sweepable_dofs"]
                ),
                "expected_multi_state_count": (
                    MULTI_SAMPLES if item["static_motion_sweepable_dofs"] > 1 else 0
                ),
                "measurement_complete": False,
                "pybullet_load_success": False,
                "drive_success": False,
                "single_state_count": 0,
                "single_contact_free_count": 0,
                "single_penetration_free_count": 0,
                "multi_state_count": 0,
                "multi_contact_free_count": 0,
                "multi_penetration_free_count": 0,
                "joint_single_contact_free_sweep_pass_count": 0,
                "joint_single_penetration_free_sweep_pass_count": 0,
                "asset_contact_free_proxy": False,
                "asset_penetration_free_proxy": False,
                "max_reset_pose_abs_error": None,
                "issues": [
                    "child_timeout" if timed_out else f"child_exit_without_result: rc={returncode}"
                ],
            }
        record["child_returncode"] = returncode
        record["child_timed_out"] = timed_out
        record["child_stderr_bytes"] = stderr.stat().st_size
        records.append(record)
        print(
            f"[{len(records)}/{len(cohort['items'])}] {record['asset_id']} "
            f"load={record['pybullet_load_success']} drive={record['drive_success']} "
            f"timeout={timed_out}",
            file=sys.stderr,
            flush=True,
        )
    expected_state_total = sum_field(records, "expected_single_state_count") + sum_field(
        records, "expected_multi_state_count"
    )
    executed_state_total = sum_field(records, "single_state_count") + sum_field(
        records, "multi_state_count"
    )
    contact_free = sum_field(records, "single_contact_free_count") + sum_field(
        records, "multi_contact_free_count"
    )
    penetration_free = sum_field(records, "single_penetration_free_count") + sum_field(
        records, "multi_penetration_free_count"
    )
    loaded = sum(record["pybullet_load_success"] for record in records)
    driven = sum(record["drive_success"] for record in records)
    completed = sum(record["measurement_complete"] for record in records)
    try:
        import pybullet as bullet

        pybullet_api_version: int | None = int(bullet.getAPIVersion())
        collision_flags_value: int | None = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
    except Exception:  # noqa: BLE001
        pybullet_api_version = None
        collision_flags_value = None
    summary = {
        "protocol_id": "nano3d_table6_artiverse_functional_proxy_v3",
        "status": "COMPLETE" if completed == len(records) else "PARTIAL_COMPLETE",
        "cohort_size": len(records),
        "cohort_selection": "frozen_functional_cohort.json; one asset/category selected from the audited manifest scope before PyBullet outcomes",
        "python_executable": sys.executable,
        "python_prefix": sys.prefix,
        "pybullet_api_version": pybullet_api_version,
        "subprocess_isolation": True,
        "timeout_seconds_per_asset": args.timeout,
        "load_success": f"{loaded}/{len(records)}",
        "drive_success": f"{driven}/{len(records)}",
        "measurement_complete_assets": f"{completed}/{len(records)}",
        "movable_dofs": sum_field(records, "movable_dof_count"),
        "motion_sweepable_dofs": sum_field(records, "motion_sweepable_dof_count"),
        "zero_range_dofs_excluded_from_motion_sweep": sum_field(records, "zero_range_dof_count"),
        "expected_states": expected_state_total,
        "executed_states": executed_state_total,
        "single_joint_states": sum_field(records, "single_state_count"),
        "multi_joint_sobol_states": sum_field(records, "multi_state_count"),
        "contact_free_states_executed": f"{contact_free}/{executed_state_total}",
        "contact_free_state_rate_executed": (
            contact_free / executed_state_total if executed_state_total else None
        ),
        "penetration_free_states_executed": f"{penetration_free}/{executed_state_total}",
        "penetration_free_state_rate_executed": (
            penetration_free / executed_state_total if executed_state_total else None
        ),
        "contact_free_states_intent_to_run_fail_closed": f"{contact_free}/{expected_state_total}",
        "penetration_free_states_intent_to_run_fail_closed": (
            f"{penetration_free}/{expected_state_total}"
        ),
        "joint_single_contact_free_sweep_pass": (
            f"{sum_field(records, 'joint_single_contact_free_sweep_pass_count')}/"
            f"{sum_field(records, 'motion_sweepable_dof_count')}"
        ),
        "joint_single_penetration_free_sweep_pass": (
            f"{sum_field(records, 'joint_single_penetration_free_sweep_pass_count')}/"
            f"{sum_field(records, 'motion_sweepable_dof_count')}"
        ),
        "asset_contact_free_proxy": (
            f"{sum(record['asset_contact_free_proxy'] for record in records)}/{len(records)}"
        ),
        "asset_penetration_free_proxy": (
            f"{sum(record['asset_penetration_free_proxy'] for record in records)}/{len(records)}"
        ),
        "child_process_nonzero_exit_count": sum(record["child_returncode"] != 0 for record in records),
        "child_process_timeout_count": sum(record["child_timed_out"] for record in records),
        "collision_policy": "PyBullet URDF_USE_SELF_COLLISION | URDF_USE_SELF_COLLISION_EXCLUDE_PARENT",
        "collision_flags": (
            "URDF_USE_INERTIA_FROM_FILE | URDF_USE_SELF_COLLISION | "
            "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | URDF_IGNORE_VISUAL_SHAPES"
        ),
        "collision_flags_value": collision_flags_value,
        "pose_evaluation": "resetJointState + performCollisionDetection; no dynamics step",
        "motor_policy": "all movable joints disabled with VELOCITY_CONTROL force=0",
        "reset_pose_readback_tolerance": 1e-9,
        "max_reset_pose_abs_error": max(
            [
                float(record["max_reset_pose_abs_error"])
                for record in records
                if record.get("max_reset_pose_abs_error") is not None
            ]
            or [0.0]
        ),
        "contact_definition": "no PyBullet self-contact points",
        "penetration_definition": (
            f"maximum negative contact distance <= {PENETRATION_TOLERANCE_M} m"
        ),
        "continuous_collision_detection": "not_run",
        "semantic_joint_geometric_validity": "N/A: collision proxy does not prove axis-on-moving-part semantics",
        "complex_joint_warning": "standard URDF may downgrade or split screw/cylindrical/free annotations; simulator DoF sweep is not logical-joint fidelity",
    }
    write_json(output / "functional_asset_records.json", records)
    write_json(output / "functional_summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=True))
    return 0


def render_report(output: Path) -> None:
    static = read_json(output / "summary.json")
    functional_path = output / "functional_summary.json"
    functional = read_json(functional_path) if functional_path.is_file() else None
    aggregate = static["static"]
    lines = [
        "# Table 6 Artiverse pre-release reference audit",
        "",
        f"Status: **{static['status']}**",
        "",
        f"The manifest-defined scope ({static['scope']['chunk_index']}) contains {aggregate['asset_count']} assets across {aggregate['category_count']} categories and {aggregate['source_count']} upstream sources. Selection used every listed root and did not depend on outcomes.",
        "",
        "## Static release audit",
        "",
        f"- core annotation + segmented GLB + collider URDF: {aggregate['complete_core_package_count']}/{aggregate['asset_count']}",
        f"- human-verified reference annotations: {aggregate['annotation_joint_count']} logical articulations ({aggregate['annotation_joints_per_asset_mean']:.3f}/asset)",
        f"- simulator export: {aggregate['urdf_movable_dof_count']} movable URDF DoFs ({aggregate['urdf_dofs_per_asset_mean']:.3f}/asset)",
        f"- parseable collider URDF: {aggregate['urdf_parse_success_count']}/{aggregate['asset_count']}",
        f"- annotation axis metadata: {aggregate['annotation_axis_valid_count']}/{aggregate['annotation_joint_count']}; range metadata: {aggregate['annotation_range_valid_count']}/{aggregate['annotation_joint_count']}",
        f"- valid URDF trees: {aggregate['urdf_valid_tree_asset_count']}/{aggregate['asset_count']}; cycle assets: {aggregate['urdf_cycle_asset_count']}",
        f"- collision geometry: {aggregate['urdf_collision_available_asset_count']}/{aggregate['asset_count']} assets; missing mesh references: {aggregate['urdf_missing_mesh_reference_count']}",
        f"- material metadata: {aggregate['material_asset_count']}/{aggregate['asset_count']}; README-declared mass_furniture_heuristic.json observed: {aggregate['mass_furniture_heuristic_asset_count']}/{aggregate['asset_count']}",
        "",
    ]
    if functional is not None:
        lines.extend(
            [
                "## Functional proxy",
                "",
                f"- load: {functional['load_success']}; reset/readback: {functional['drive_success']}",
                f"- complete measurements: {functional['measurement_complete_assets']}",
                f"- declared/motion/zero-range DoFs: {functional['movable_dofs']}/{functional['motion_sweepable_dofs']}/{functional['zero_range_dofs_excluded_from_motion_sweep']}",
                f"- contact-free states (executed): {functional['contact_free_states_executed']}",
                f"- penetration-free states (executed): {functional['penetration_free_states_executed']}",
                f"- per-joint penetration-free single sweep: {functional['joint_single_penetration_free_sweep_pass']}",
                f"- per-asset penetration-free proxy: {functional['asset_penetration_free_proxy']}",
                "",
            ]
        )
    lines.extend(
        [
            "## Claim boundary",
            "",
            "Artiverse is a manually corrected and expert-verified dataset reference, not a prediction method. Its annotation JSON and URDF belong to the same release pipeline, so their agreement is export fidelity rather than independent joint accuracy. Complex joint annotations can be lossy in standard URDF. All PyBullet results are discrete resetJointState + performCollisionDetection proxies with motors disabled and direct parent-child collisions excluded; no dynamics step, CCD, or semantic axis-on-moving-part judgment was run.",
            "",
            "This is a gated, license=other, pre-release snapshot whose README says cleanup is ongoing.",
        ]
    )
    write_text(output / "report.md", "\n".join(lines) + "\n")


def run_self_check(output: Path) -> int:
    static = read_json(output / "summary.json")
    cohort = read_json(output / "frozen_functional_cohort.json")
    records = read_json(output / "asset_records.json")
    functional_path = output / "functional_summary.json"
    functional = read_json(functional_path) if functional_path.is_file() else None
    collision_smoke_path = output / "self_collision_flag_smoke.json"
    collision_smoke = read_json(collision_smoke_path) if collision_smoke_path.is_file() else None
    manifest = read_json(WORKSPACE / static["release"]["manifest_path"])
    expected_roots = {
        root
        for chunk_index in static["scope"]["chunk_indices"]
        for root in chunk_roots(manifest, int(chunk_index))
    }
    recomputed_cohort = freeze_functional_cohort(records)
    checks = {
        "static_status_complete": static["status"] == "COMPLETE",
        "record_count_matches_scope": len(records) == static["scope"]["manifest_defined_roots"],
        "record_roots_match_manifest": {record["root"] for record in records} == expected_roots,
        "all_core_packages_complete": static["static"]["complete_core_package_count"] == len(records),
        "all_annotation_json_parsed": static["static"]["annotation_parse_success_count"] == len(records),
        "urdf_parse_accounting_complete": (
            static["static"]["urdf_parse_success_count"]
            + sum(not record["urdf_parse_success"] for record in records)
            == len(records)
        ),
        "no_missing_mesh_reference": static["static"]["urdf_missing_mesh_reference_count"] == 0,
        "cohort_frozen": cohort["status"] == "FROZEN" and cohort["size"] == len(cohort["items"]),
        "cohort_unique_categories": len({item["category"] for item in cohort["items"]}) == cohort["size"],
        "cohort_unique_assets": len({item["asset_id"] for item in cohort["items"]}) == cohort["size"],
        "cohort_matches_recomputed_hash_winners": cohort["items"] == recomputed_cohort,
        "semantic_accuracy_fail_closed": all(
            str(value).startswith("N/A") for value in static["semantic_metric_boundary"].values()
        ),
    }
    if functional is not None:
        functional_records = read_json(output / "functional_asset_records.json")
        checks.update(
            {
                "functional_status_complete": functional["status"] == "COMPLETE",
                "functional_cohort_matches": functional["cohort_size"] == cohort["size"],
                "functional_expected_state_accounting": (
                    functional["executed_states"] == functional["expected_states"]
                    and functional["measurement_complete_assets"]
                    == f"{cohort['size']}/{cohort['size']}"
                ),
                "functional_records_match_frozen_ids": (
                    [record["asset_id"] for record in functional_records]
                    == [item["asset_id"] for item in cohort["items"]]
                ),
                "functional_each_record_complete": all(
                    record.get("measurement_complete")
                    and record["single_state_count"]
                    == record["expected_single_state_count"]
                    and record["multi_state_count"]
                    == record["expected_multi_state_count"]
                    for record in functional_records
                ),
                "functional_urdf_hashes_match_frozen": all(
                    record.get("urdf_sha256") == item.get("urdf_sha256")
                    for record, item in zip(functional_records, cohort["items"])
                ),
                "functional_no_timeout": functional["child_process_timeout_count"] == 0,
                "functional_no_native_abort": (
                    functional["child_process_nonzero_exit_count"] == 0
                ),
                "functional_protocol_v3": functional["protocol_id"].endswith("_v3"),
                "functional_reset_pose_exact": (
                    functional["max_reset_pose_abs_error"]
                    <= functional["reset_pose_readback_tolerance"]
                ),
                "ccd_not_claimed": functional["continuous_collision_detection"] == "not_run",
                "self_collision_flag_smoke_pass": bool(
                    collision_smoke
                    and collision_smoke.get("status") == "PASS"
                    and collision_smoke.get("contacts", {}).get("exclude_parent_only") == 0
                    and collision_smoke.get("contacts", {}).get("self_plus_exclude_parent", 0) > 0
                ),
            }
        )
    payload = {
        "protocol_id": "nano3d_table6_artiverse_self_check_v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "passed": sum(checks.values()),
        "total": len(checks),
        "checks": checks,
    }
    write_json(output / "self_check.json", payload)
    print(json.dumps(payload, indent=2, ensure_ascii=True))
    return 0 if all(checks.values()) else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="phase", required=True)
    static_parser = subparsers.add_parser("static")
    static_parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    static_parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    static_parser.add_argument("--chunk-index", type=int, default=1)
    static_parser.add_argument("--all-chunks", action="store_true")
    functional_parser = subparsers.add_parser("functional")
    functional_parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    functional_parser.add_argument("--timeout", type=int, default=900)
    report_parser = subparsers.add_parser("report")
    report_parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    check_parser = subparsers.add_parser("self-check")
    check_parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    child_parser = subparsers.add_parser("_child")
    child_parser.add_argument("--payload", type=Path, required=True)
    child_parser.add_argument("--result", type=Path, required=True)
    args = parser.parse_args()
    if args.phase == "static":
        return run_static(args)
    if args.phase == "functional":
        return run_functional(args)
    if args.phase == "report":
        render_report(safe(args.output))
        return 0
    if args.phase == "self-check":
        return run_self_check(safe(args.output))
    if args.phase == "_child":
        return_code = child_payload(args.payload, args.result)
        # Some PyBullet builds spend minutes unloading native state after all
        # results are durably written. Each child owns its DIRECT client, so an
        # immediate process exit avoids misclassifying teardown as task timeout.
        os._exit(return_code)
    raise AssertionError(args.phase)


if __name__ == "__main__":
    raise SystemExit(main())
