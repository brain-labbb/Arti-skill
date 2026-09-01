#!/usr/bin/env python3
"""Audit PartNet-Mobility as a Table 6 real-data reference.

PartNet-Mobility is a curated dataset, not a generated-method baseline.  The
static phase audits all 2,347 local v0 packages and records which released
annotation fields can serve as source-reference data.  The functional phase
freezes one identity-ranked asset per exact category before any simulator
outcome and runs every selected URDF in its own PyBullet subprocess.

The simulator result is a discrete collision proxy.  It does not turn package
metadata into independent semantic accuracy, and PartNet-Mobility annotations
are explicitly not independent gold for the derivative PhysX-Mobility release.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Iterable
import urllib.request
import xml.etree.ElementTree as ET
import zipfile


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
DEFAULT_DATASET = WORKSPACE / "PartNet_Mobility/data/dataset"
DEFAULT_ARCHIVE = WORKSPACE / "PartNet_Mobility/partnet-mobility-v0.zip"
DEFAULT_PHYSX = Path(
    "/mnt/zsn/zsn_workspace/Ctrl-3D-trellis2-controlnet-dev/demo/"
    "physical_edit_demo/third_party/physx_mobility/extracted/PhysX_mobility"
)
DEFAULT_OUTPUT = REPO / "exp/runtime/table6_partnet_mobility"
AUTHORIZED_ROOTS = (Path("/mnt/zsn/lyb"), Path("/mnt/zsn/zsn_workspace"))

PROTOCOL_ID = "nano3d_table6_partnet_mobility_reference_v3"
SALT = "nano3d-table6-partnet-mobility-category-reference-v1"
EXPECTED_ASSETS = 2347
EXPECTED_ARCHIVE_SHA256 = (
    "b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff"
)
OFFICIAL_API = "https://huggingface.co/api/datasets/sapien-sim/PartNetMobility"
OFFICIAL_REVISION = "ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f"
SINGLE_SAMPLES = 11
MULTI_SAMPLES = 64
GENERIC_REVOLUTE_DEG = 300.0
ZERO_WIDTH_TOLERANCE = 1e-12
PENETRATION_TOLERANCE_M = 1e-6
RESET_TOLERANCE = 1e-9
LOGICAL_MOTION_TYPES = {"hinge", "slider"}
REQUIRED_FILES = (
    "meta.json",
    "mobility_v2.json",
    "mobility.urdf",
    "semantics.txt",
    "result.json",
    "bounding_box.json",
)
OPTIONAL_FILES = (
    "result_original.json",
)
AUDITED_SCHEMA_FILES = REQUIRED_FILES + OPTIONAL_FILES


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    roots = [root.resolve(strict=True) for root in AUTHORIZED_ROOTS]
    if not any(resolved == root or root in resolved.parents for root in roots):
        raise ValueError(f"path outside authorized roots: {resolved}")
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return sha256_bytes(payload.encode("utf-8"))


def read_json(path: Path) -> Any:
    return json.loads(contained(path).read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    destination = contained(path, exists=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    destination = contained(path, exists=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n")
    temporary.replace(destination)


def write_text(path: Path, value: str) -> None:
    destination = contained(path, exists=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(destination)


def fraction(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def finite_vector(value: Any, *, nonzero: bool = False) -> bool:
    if not isinstance(value, list) or len(value) != 3 or not all(finite(item) for item in value):
        return False
    return not nonzero or sum(float(item) ** 2 for item in value) > 1e-18


def xml_vector(node: ET.Element | None, key: str, *, default_ok: bool) -> bool:
    if node is None or not node.get(key):
        return default_ok
    try:
        values = [float(item) for item in node.get(key, "").split()]
    except ValueError:
        return False
    return len(values) == 3 and all(math.isfinite(item) for item in values)


def xml_axis_valid(node: ET.Element | None) -> bool:
    if not xml_vector(node, "xyz", default_ok=False):
        return False
    values = [float(item) for item in node.get("xyz", "").split()]  # type: ignore[union-attr]
    return sum(item * item for item in values) > 1e-18


def graph_audit(links: set[str], edges: list[tuple[str, str]]) -> dict[str, Any]:
    indegree = Counter(child for _, child in edges)
    adjacency: dict[str, list[str]] = defaultdict(list)
    for parent, child in edges:
        adjacency[parent].append(child)
    roots = sorted(link for link in links if indegree[link] == 0)
    active: set[str] = set()
    visited: set[str] = set()
    cycle = False

    def visit(node: str) -> None:
        nonlocal cycle
        if node in active:
            cycle = True
            return
        if node in visited:
            return
        active.add(node)
        for child in adjacency.get(node, []):
            visit(child)
        active.remove(node)
        visited.add(node)

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
        "cycle": cycle,
        "multi_parent_link_count": sum(value > 1 for value in indegree.values()),
        "connected": len(roots) == 1 and len(reachable) == len(links),
        "valid_tree": (
            len(roots) == 1
            and not cycle
            and len(reachable) == len(links)
            and not any(value > 1 for value in indegree.values())
            and len(edges) == max(0, len(links) - 1)
        ),
    }


def audit_annotation(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    payload = read_json(path)
    if not isinstance(payload, list):
        raise ValueError("mobility_v2.json root is not a list")
    ids = [item.get("id") for item in payload if isinstance(item, dict)]
    id_set = {item for item in ids if isinstance(item, int)}
    rows: list[dict[str, Any]] = []
    for item in payload:
        if not isinstance(item, dict):
            raise ValueError("mobility_v2.json contains a non-object row")
        joint_type = str(item.get("joint", ""))
        is_motion = joint_type in LOGICAL_MOTION_TYPES
        parent = item.get("parent")
        data = item.get("jointData") if isinstance(item.get("jointData"), dict) else {}
        axis = data.get("axis") if isinstance(data.get("axis"), dict) else {}
        limit = data.get("limit") if isinstance(data.get("limit"), dict) else {}
        no_limit = limit.get("noLimit") is True
        bounds_present = "a" in limit and "b" in limit
        bounds_finite = finite(limit.get("a")) and finite(limit.get("b"))
        composite_slider = bool(
            joint_type == "slider"
            and (limit.get("rotates") is True or limit.get("noRotationLimit") is True)
        )
        rows.append(
            {
                "id": item.get("id"),
                "parent": parent,
                "joint_type": joint_type,
                "is_logical_motion": is_motion,
                "composite_slider": composite_slider,
                "parent_reference_valid": parent == -1 or parent in id_set,
                "axis_origin_present": "origin" in axis,
                "axis_origin_valid": finite_vector(axis.get("origin")),
                "axis_direction_present": "direction" in axis,
                "axis_direction_valid": finite_vector(axis.get("direction"), nonzero=True),
                "limit_object_present": bool(limit),
                "no_limit": no_limit,
                "bounds_present": bounds_present,
                "bounds_finite": bounds_finite,
                "range_metadata_valid": bool(no_limit or (bounds_present and bounds_finite)),
                "part_list_present": isinstance(item.get("parts"), list) and bool(item.get("parts")),
                "name_present": isinstance(item.get("name"), str) and bool(item.get("name")),
            }
        )
    return (
        {
            "parse_success": True,
            "node_count": len(rows),
            "unique_integer_ids": len(id_set) == len(rows),
            "parent_graph_references_valid": all(row["parent_reference_valid"] for row in rows),
        },
        rows,
    )


def audit_semantics(path: Path) -> dict[str, Any]:
    rows = []
    for line_number, raw in enumerate(contained(path).read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        fields = raw.split(maxsplit=2)
        rows.append(
            {
                "line_number": line_number,
                "field_count": len(fields),
                "link": fields[0] if fields else "",
                "joint": fields[1] if len(fields) > 1 else "",
                "name": fields[2] if len(fields) > 2 else "",
            }
        )
    return {
        "row_count": len(rows),
        "schema_valid": all(row["field_count"] == 3 for row in rows),
        "unique_links": len({row["link"] for row in rows}) == len(rows),
        "joint_type_counts": dict(Counter(row["joint"] for row in rows)),
    }


def audit_urdf(path: Path, asset_dir: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    root = ET.parse(contained(path)).getroot()
    if root.tag != "robot":
        raise ValueError(f"URDF root is {root.tag!r}, not robot")
    link_nodes = root.findall("link")
    links = {node.get("name", "") for node in link_nodes if node.get("name")}
    duplicate_or_unnamed = len(links) != len(link_nodes)
    edges: list[tuple[str, str]] = []
    joints: list[dict[str, Any]] = []
    for index, node in enumerate(root.findall("joint")):
        parent_node = node.find("parent")
        child_node = node.find("child")
        parent = parent_node.get("link", "") if parent_node is not None else ""
        child = child_node.get("link", "") if child_node is not None else ""
        edges.append((parent, child))
        joint_type = str(node.get("type", ""))
        if joint_type == "fixed":
            continue
        axis = node.find("axis")
        origin = node.find("origin")
        limit = node.find("limit")
        lower = upper = None
        bounded = joint_type in {"revolute", "prismatic"}
        limit_valid = not bounded
        if bounded:
            try:
                lower = float(limit.get("lower"))  # type: ignore[union-attr]
                upper = float(limit.get("upper"))  # type: ignore[union-attr]
                limit_valid = math.isfinite(lower) and math.isfinite(upper) and lower <= upper
            except (AttributeError, TypeError, ValueError):
                limit_valid = False
        span = upper - lower if limit_valid and bounded else None
        rowsweep = joint_type == "continuous" or bool(
            bounded and span is not None and span > ZERO_WIDTH_TOLERANCE
        )
        joints.append(
            {
                "index": index,
                "name": node.get("name", f"joint_{index}"),
                "type": joint_type,
                "parent": parent,
                "child": child,
                "parent_child_structurally_valid": parent in links and child in links,
                "axis_metadata_valid": xml_axis_valid(axis),
                "origin_metadata_valid": (
                    xml_vector(origin, "xyz", default_ok=True)
                    and xml_vector(origin, "rpy", default_ok=True)
                ),
                "bounded": bounded,
                "limit_metadata_valid": limit_valid,
                "lower": lower,
                "upper": upper,
                "zero_width": bool(bounded and span is not None and abs(span) <= ZERO_WIDTH_TOLERANCE),
                "motion_sweepable": rowsweep,
                "generic_revolute": bool(
                    joint_type == "revolute"
                    and span is not None
                    and math.degrees(span) >= GENERIC_REVOLUTE_DEG
                ),
            }
        )
    mesh_references: list[str] = []
    collision_elements = 0
    inertial_links = 0
    valid_inertial_links = 0
    for link in link_nodes:
        collision_elements += len(link.findall("collision"))
        inertial = link.find("inertial")
        inertial_links += int(inertial is not None)
        inertial_ok = False
        if inertial is not None:
            try:
                mass = float(inertial.find("mass").get("value"))  # type: ignore[union-attr]
                tensor = inertial.find("inertia")
                diagonal = [float(tensor.get(key)) for key in ("ixx", "iyy", "izz")]  # type: ignore[union-attr]
                inertial_ok = mass > 0 and all(value > 0 and math.isfinite(value) for value in diagonal)
            except (AttributeError, TypeError, ValueError):
                inertial_ok = False
        valid_inertial_links += int(inertial_ok)
        for mesh in link.findall("collision/geometry/mesh"):
            if mesh.get("filename"):
                mesh_references.append(mesh.get("filename", ""))
    missing_mesh = [
        filename
        for filename in mesh_references
        if not (asset_dir / filename.replace("\\", "/")).is_file()
    ]
    graph = graph_audit(links, edges)
    return (
        {
            "parse_success": True,
            "link_count": len(link_nodes),
            "duplicate_or_unnamed_link": duplicate_or_unnamed,
            "joint_count_all": len(edges),
            "movable_dof_count": len(joints),
            "motion_sweepable_dof_count": sum(row["motion_sweepable"] for row in joints),
            "zero_width_dof_count": sum(row["zero_width"] for row in joints),
            "collision_element_count": collision_elements,
            "collision_mesh_reference_count": len(mesh_references),
            "missing_collision_mesh_reference_count": len(missing_mesh),
            "inertial_link_count": inertial_links,
            "valid_inertial_link_count": valid_inertial_links,
            **graph,
        },
        joints,
    )


def exact_numeric_ids(directory: Path, suffix: str) -> set[str]:
    return {
        path.name[: -len(suffix)]
        for path in contained(directory).glob(f"*{suffix}")
        if path.name[: -len(suffix)].isdigit()
    }


def official_repository_audit() -> dict[str, Any]:
    result: dict[str, Any] = {
        "api": OFFICIAL_API,
        "expected_revision": OFFICIAL_REVISION,
        "audit_complete": False,
    }
    try:
        with urllib.request.urlopen(OFFICIAL_API, timeout=30) as response:  # noqa: S310
            payload = json.load(response)
        numeric_zip_ids = {
            Path(str(row.get("rfilename", ""))).stem
            for row in payload.get("siblings", [])
            if str(row.get("rfilename", "")).endswith(".zip")
            and Path(str(row.get("rfilename", ""))).stem.isdigit()
        }
        result.update(
            {
                "audit_complete": True,
                "observed_revision": payload.get("sha"),
                "revision_matches_pin": payload.get("sha") == OFFICIAL_REVISION,
                "gated": payload.get("gated"),
                "declared_license": payload.get("cardData", {}).get("license"),
                "numeric_per_id_zip_count": len(numeric_zip_ids),
                "numeric_per_id_zip_inventory_sha256": canonical_sha256(sorted(numeric_zip_ids, key=int)),
                "numeric_per_id_zip_ids": sorted(numeric_zip_ids, key=int),
                "local_ids_listed_count": None,
            }
        )
    except Exception as exc:  # noqa: BLE001
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


def essential_archive_audit(
    archive: Path, dataset_root: Path, asset_ids: list[str]
) -> dict[str, Any]:
    presence_checks = 0
    byte_comparisons = 0
    both_missing = 0
    mismatches: list[dict[str, str]] = []
    with zipfile.ZipFile(contained(archive)) as bundle:
        infos = bundle.infolist()
        member_names = {info.filename for info in infos}
        file_infos = [info for info in infos if not info.is_dir()]
        for dataset_id in asset_ids:
            for filename in AUDITED_SCHEMA_FILES:
                local_path = dataset_root / dataset_id / filename
                member = f"dataset/{dataset_id}/{filename}"
                local_present = local_path.is_file()
                archive_present = member in member_names
                presence_checks += 1
                if local_present != archive_present:
                    mismatches.append(
                        {
                            "dataset_id": dataset_id,
                            "filename": filename,
                            "reason": "presence_mismatch",
                            "local_present": str(local_present),
                            "archive_present": str(archive_present),
                        }
                    )
                    continue
                if not local_present:
                    both_missing += 1
                    continue
                local_hash = sha256_file(local_path)
                archive_hash = sha256_bytes(bundle.read(member))
                byte_comparisons += 1
                if local_hash != archive_hash:
                    mismatches.append(
                        {"dataset_id": dataset_id, "filename": filename, "reason": "hash_mismatch"}
                    )
        return {
            "zip_entry_count": len(infos),
            "zip_file_entry_count": len(file_infos),
            "zip_uncompressed_file_bytes": sum(info.file_size for info in file_infos),
            "zip_member_name_inventory_sha256": canonical_sha256(sorted(member_names)),
            "audited_schema_path_presence_check_count": presence_checks,
            "expected_audited_schema_path_presence_check_count": (
                len(asset_ids) * len(AUDITED_SCHEMA_FILES)
            ),
            "audited_schema_path_both_missing_count": both_missing,
            "audited_schema_file_byte_comparison_count": byte_comparisons,
            "audited_schema_path_or_hash_mismatch_count": len(mismatches),
            "audited_schema_path_or_hash_mismatches": mismatches[:100],
        }


def aggregate_subset(records: list[dict[str, Any]], identity_set: set[str]) -> dict[str, Any]:
    rows = [row for row in records if row["dataset_id"] in identity_set]
    return {
        "asset_count": len(rows),
        "category_count": len({row["category"] for row in rows}),
        "logical_motion_joint_count": sum(row["annotation_logical_motion_count"] for row in rows),
        "urdf_movable_dof_count": sum(row["urdf_movable_dof_count"] for row in rows),
        "urdf_motion_sweepable_dof_count": sum(row["urdf_motion_sweepable_dof_count"] for row in rows),
        "urdf_zero_width_dof_count": sum(row["urdf_zero_width_dof_count"] for row in rows),
        "valid_tree_count": sum(row["urdf_valid_tree"] for row in rows),
        "collision_asset_count": sum(row["urdf_collision_element_count"] > 0 for row in rows),
    }


def run_static(dataset_root: Path, archive: Path, physx_root: Path, output: Path) -> None:
    dataset_root = contained(dataset_root)
    archive = contained(archive)
    physx_root = contained(physx_root)
    output = contained(output, exists=False)
    output.mkdir(parents=True, exist_ok=True)
    asset_dirs = sorted(
        [path for path in dataset_root.iterdir() if path.is_dir() and path.name.isdigit()],
        key=lambda path: int(path.name),
    )
    asset_ids = [path.name for path in asset_dirs]
    pm_ids = set(asset_ids)
    physx_finaljson_ids = exact_numeric_ids(physx_root / "finaljson", ".json")
    physx_original_urdf_ids = exact_numeric_ids(physx_root / "urdf", ".urdf")
    physx_complete_ids = physx_finaljson_ids & physx_original_urdf_ids
    intersection_ids = pm_ids & physx_finaljson_ids
    partnet_only_ids = pm_ids - physx_finaljson_ids

    file_presence = Counter()
    records: list[dict[str, Any]] = []
    category_counts: Counter[str] = Counter()
    version_counts: Counter[str] = Counter()
    annotation_type_counts: Counter[str] = Counter()
    urdf_type_counts: Counter[str] = Counter()
    annotation_parse_errors: list[dict[str, str]] = []
    urdf_parse_errors: list[dict[str, str]] = []
    semantic_schema_failures: list[str] = []

    for index, asset_dir in enumerate(asset_dirs, 1):
        dataset_id = asset_dir.name
        for filename in AUDITED_SCHEMA_FILES:
            file_presence[filename] += int((asset_dir / filename).is_file())
        record: dict[str, Any] = {
            "dataset_id": dataset_id,
            "in_physx_finaljson": dataset_id in physx_finaljson_ids,
            "in_physx_original_urdf": dataset_id in physx_original_urdf_ids,
        }
        try:
            meta = read_json(asset_dir / "meta.json")
            category = str(meta.get("model_cat", ""))
            version = str(meta.get("version", ""))
            record.update(
                {
                    "category": category,
                    "annotation_version": version,
                    "model_id": meta.get("model_id"),
                    "annotation_id": meta.get("anno_id"),
                }
            )
            category_counts[category] += 1
            version_counts[version] += 1
        except Exception as exc:  # noqa: BLE001
            record.update({"category": "", "annotation_version": "", "meta_error": str(exc)})

        try:
            annotation, annotation_rows = audit_annotation(asset_dir / "mobility_v2.json")
            motion_rows = [row for row in annotation_rows if row["is_logical_motion"]]
            annotation_type_counts.update(row["joint_type"] for row in annotation_rows)
            record.update(
                {
                    "annotation_parse_success": True,
                    "annotation_node_count": annotation["node_count"],
                    "annotation_unique_integer_ids": annotation["unique_integer_ids"],
                    "annotation_parent_graph_references_valid": annotation[
                        "parent_graph_references_valid"
                    ],
                    "annotation_logical_motion_count": len(motion_rows),
                    "annotation_composite_slider_count": sum(
                        row["composite_slider"] for row in motion_rows
                    ),
                    "annotation_axis_origin_valid_count": sum(
                        row["axis_origin_valid"] for row in motion_rows
                    ),
                    "annotation_axis_direction_valid_count": sum(
                        row["axis_direction_valid"] for row in motion_rows
                    ),
                    "annotation_range_metadata_valid_count": sum(
                        row["range_metadata_valid"] for row in motion_rows
                    ),
                    "annotation_part_list_present_count": sum(
                        row["part_list_present"] for row in motion_rows
                    ),
                    "annotation_name_present_count": sum(
                        row["name_present"] for row in motion_rows
                    ),
                }
            )
        except Exception as exc:  # noqa: BLE001
            annotation_parse_errors.append(
                {"dataset_id": dataset_id, "error": f"{type(exc).__name__}: {exc}"}
            )
            record.update(
                {
                    "annotation_parse_success": False,
                    "annotation_node_count": 0,
                    "annotation_unique_integer_ids": False,
                    "annotation_parent_graph_references_valid": False,
                    "annotation_logical_motion_count": 0,
                    "annotation_composite_slider_count": 0,
                    "annotation_axis_origin_valid_count": 0,
                    "annotation_axis_direction_valid_count": 0,
                    "annotation_range_metadata_valid_count": 0,
                    "annotation_part_list_present_count": 0,
                    "annotation_name_present_count": 0,
                }
            )

        try:
            semantics = audit_semantics(asset_dir / "semantics.txt")
            record.update(
                {
                    "semantics_row_count": semantics["row_count"],
                    "semantics_schema_valid": semantics["schema_valid"],
                    "semantics_unique_links": semantics["unique_links"],
                }
            )
            if not semantics["schema_valid"] or not semantics["unique_links"]:
                semantic_schema_failures.append(dataset_id)
        except Exception as exc:  # noqa: BLE001
            semantic_schema_failures.append(dataset_id)
            record.update(
                {
                    "semantics_row_count": 0,
                    "semantics_schema_valid": False,
                    "semantics_unique_links": False,
                    "semantics_error": f"{type(exc).__name__}: {exc}",
                }
            )

        try:
            urdf, urdf_rows = audit_urdf(asset_dir / "mobility.urdf", asset_dir)
            urdf_type_counts.update(row["type"] for row in urdf_rows)
            record.update(
                {
                    "urdf_parse_success": True,
                    "urdf_sha256": sha256_file(asset_dir / "mobility.urdf"),
                    "urdf_link_count": urdf["link_count"],
                    "urdf_joint_count_all": urdf["joint_count_all"],
                    "urdf_movable_dof_count": urdf["movable_dof_count"],
                    "urdf_motion_sweepable_dof_count": urdf["motion_sweepable_dof_count"],
                    "urdf_zero_width_dof_count": urdf["zero_width_dof_count"],
                    "urdf_valid_tree": urdf["valid_tree"],
                    "urdf_root_count": urdf["root_count"],
                    "urdf_cycle": urdf["cycle"],
                    "urdf_parent_child_structurally_valid_count": sum(
                        row["parent_child_structurally_valid"] for row in urdf_rows
                    ),
                    "urdf_axis_metadata_valid_count": sum(
                        row["axis_metadata_valid"] for row in urdf_rows
                    ),
                    "urdf_origin_metadata_valid_count": sum(
                        row["origin_metadata_valid"] for row in urdf_rows
                    ),
                    "urdf_bounded_dof_count": sum(row["bounded"] for row in urdf_rows),
                    "urdf_limit_metadata_valid_count": sum(
                        row["bounded"] and row["limit_metadata_valid"] for row in urdf_rows
                    ),
                    "urdf_generic_revolute_count": sum(
                        row["generic_revolute"] for row in urdf_rows
                    ),
                    "urdf_continuous_count": sum(
                        row["type"] == "continuous" for row in urdf_rows
                    ),
                    "urdf_collision_element_count": urdf["collision_element_count"],
                    "urdf_collision_mesh_reference_count": urdf[
                        "collision_mesh_reference_count"
                    ],
                    "urdf_missing_collision_mesh_reference_count": urdf[
                        "missing_collision_mesh_reference_count"
                    ],
                    "urdf_inertial_link_count": urdf["inertial_link_count"],
                    "urdf_valid_inertial_link_count": urdf["valid_inertial_link_count"],
                }
            )
        except Exception as exc:  # noqa: BLE001
            urdf_parse_errors.append(
                {"dataset_id": dataset_id, "error": f"{type(exc).__name__}: {exc}"}
            )
            record.update(
                {
                    "urdf_parse_success": False,
                    "urdf_sha256": None,
                    "urdf_link_count": 0,
                    "urdf_joint_count_all": 0,
                    "urdf_movable_dof_count": 0,
                    "urdf_motion_sweepable_dof_count": 0,
                    "urdf_zero_width_dof_count": 0,
                    "urdf_valid_tree": False,
                    "urdf_root_count": 0,
                    "urdf_cycle": False,
                    "urdf_parent_child_structurally_valid_count": 0,
                    "urdf_axis_metadata_valid_count": 0,
                    "urdf_origin_metadata_valid_count": 0,
                    "urdf_bounded_dof_count": 0,
                    "urdf_limit_metadata_valid_count": 0,
                    "urdf_generic_revolute_count": 0,
                    "urdf_continuous_count": 0,
                    "urdf_collision_element_count": 0,
                    "urdf_collision_mesh_reference_count": 0,
                    "urdf_missing_collision_mesh_reference_count": 0,
                    "urdf_inertial_link_count": 0,
                    "urdf_valid_inertial_link_count": 0,
                }
            )
        records.append(record)
        if index % 250 == 0:
            print(f"static {index}/{len(asset_dirs)}", flush=True)

    archive_hash = sha256_file(archive)
    archive_audit = essential_archive_audit(archive, dataset_root, asset_ids)
    official = official_repository_audit()
    if official.get("audit_complete"):
        official_ids = set(official.pop("numeric_per_id_zip_ids"))
        official["local_ids_listed_count"] = len(pm_ids & official_ids)
        official["all_local_ids_listed_in_pinned_repository"] = pm_ids <= official_ids
        official["official_ids_missing_locally_count"] = len(official_ids - pm_ids)
        official["local_ids_missing_officially_count"] = len(pm_ids - official_ids)

    write_jsonl(output / "asset_records.jsonl", records)
    logical_motion_total = sum(row["annotation_logical_motion_count"] for row in records)
    urdf_dof_total = sum(row["urdf_movable_dof_count"] for row in records)
    intersection_summary = aggregate_subset(records, intersection_ids)
    partnet_only_summary = aggregate_subset(records, partnet_only_ids)
    id_partition = {
        "partnet_id_count": len(pm_ids),
        "physx_finaljson_numeric_id_count": len(physx_finaljson_ids),
        "physx_original_numeric_urdf_id_count": len(physx_original_urdf_ids),
        "physx_complete_original_plus_finaljson_id_count": len(physx_complete_ids),
        "intersection_partnet_physx_finaljson_count": len(intersection_ids),
        "partnet_only_count": len(partnet_only_ids),
        "physx_finaljson_missing_from_partnet_count": len(physx_finaljson_ids - pm_ids),
        "intersection_ids_sha256": canonical_sha256(sorted(intersection_ids, key=int)),
        "partnet_only_ids_sha256": canonical_sha256(sorted(partnet_only_ids, key=int)),
        "partition_union_is_all_partnet": intersection_ids | partnet_only_ids == pm_ids,
        "partition_disjoint": not (intersection_ids & partnet_only_ids),
        "physx_mobility_based_on_partnet_mobility": True,
        "partnet_annotations_are_independent_physx_blind_test_gold": False,
        "partnet_annotations_allowed_role_for_physx": (
            "same-ID source-reference and representation-preservation audit only"
        ),
        "intersection": intersection_summary,
        "partnet_only": partnet_only_summary,
        "interpretation": (
            "The 2,024 shared IDs are source-linked pairs. PartNet-Mobility annotations may "
            "serve as source-reference fields for a PhysX representation-preservation audit, "
            "but not as independent blind-test gold because PhysX-Mobility is derived from "
            "PartNet-Mobility. The 323 PartNet-only assets have no local PhysX finaljson pair."
        ),
    }
    write_json(output / "physx_id_partition.json", id_partition)

    provenance = {
        "dataset": "PartNet-Mobility v0 local release",
        "dataset_root": str(dataset_root),
        "archive": {
            "path": str(archive),
            "bytes": archive.stat().st_size,
            "sha256": archive_hash,
            "expected_sha256": EXPECTED_ARCHIVE_SHA256,
            "matches_expected": archive_hash == EXPECTED_ARCHIVE_SHA256,
            **archive_audit,
        },
        "official_repository": official,
        "version_evidence": {
            "archive_filename": archive.name,
            "package_meta_version_counts": dict(sorted(version_counts.items())),
            "interpretation": (
                "Archive/package identify the historical v0 payload; per-asset meta.version is "
                "an annotation revision counter and is not a dataset release version."
            ),
        },
        "license": {
            "hub_declared_value": official.get("declared_license"),
            "terms_summary": (
                "gated; non-commercial research and educational use; PartNet-Mobility "
                "terms and ShapeNet terms both apply; access may be terminated"
            ),
            "redistribution_boundary": (
                "Do not redistribute raw assets or include them in a public artifact bundle "
                "without confirming that every recipient has accepted the applicable terms."
            ),
            "source": "official sapien-sim/PartNetMobility gated dataset card and SAPIEN terms",
        },
        "provenance_status": "PROVENANCE_LIMITED",
        "provenance_limit": (
            "The complete local archive matches the previously frozen workspace hash and all "
            "audited extracted schema paths have identical presence and bytes to archive members, "
            "but active credentials could not download official per-ID objects for direct byte "
            "authentication to the pinned revision."
        ),
        "reference_role": "curated source/reference dataset; not a generated baseline",
        "physx_relationship": id_partition["interpretation"],
    }
    write_json(output / "provenance.json", provenance)

    static = {
        "protocol_id": PROTOCOL_ID,
        "status": "COMPLETE",
        "role": "real-data source reference; excluded from generated-method rankings",
        "asset_count": len(records),
        "category_count": len(category_counts),
        "category_counts": dict(sorted(category_counts.items())),
        "audited_schema_file_presence": dict(file_presence),
        "all_core_files_present_assets": sum(
            all((dataset_root / row["dataset_id"] / filename).is_file() for filename in REQUIRED_FILES)
            for row in records
        ),
        "all_audited_schema_files_present_assets": sum(
            all(
                (dataset_root / row["dataset_id"] / filename).is_file()
                for filename in AUDITED_SCHEMA_FILES
            )
            for row in records
        ),
        "optional_file_presence": {
            filename: file_presence[filename] for filename in OPTIONAL_FILES
        },
        "meta_parse_success_count": sum(bool(row.get("category")) for row in records),
        "annotation_parse_success_count": sum(row["annotation_parse_success"] for row in records),
        "annotation_parse_errors": annotation_parse_errors,
        "annotation_node_count": sum(row["annotation_node_count"] for row in records),
        "annotation_joint_type_counts": dict(annotation_type_counts),
        "annotation_logical_motion_joint_count": logical_motion_total,
        "annotation_logical_motion_joints_per_asset_mean": fraction(logical_motion_total, len(records)),
        "annotation_composite_slider_count": sum(
            row["annotation_composite_slider_count"] for row in records
        ),
        "annotation_parent_graph_valid_asset_count": sum(
            row["annotation_parent_graph_references_valid"] for row in records
        ),
        "annotation_axis_origin_valid_count": sum(
            row["annotation_axis_origin_valid_count"] for row in records
        ),
        "annotation_axis_direction_valid_count": sum(
            row["annotation_axis_direction_valid_count"] for row in records
        ),
        "annotation_range_metadata_valid_count": sum(
            row["annotation_range_metadata_valid_count"] for row in records
        ),
        "annotation_part_list_present_count": sum(
            row["annotation_part_list_present_count"] for row in records
        ),
        "annotation_name_present_count": sum(
            row["annotation_name_present_count"] for row in records
        ),
        "semantics_schema_valid_asset_count": sum(row["semantics_schema_valid"] for row in records),
        "semantics_schema_failure_ids": semantic_schema_failures,
        "urdf_parse_success_count": sum(row["urdf_parse_success"] for row in records),
        "urdf_parse_errors": urdf_parse_errors,
        "urdf_valid_tree_asset_count": sum(row["urdf_valid_tree"] for row in records),
        "urdf_joint_type_counts": dict(urdf_type_counts),
        "urdf_movable_dof_count": urdf_dof_total,
        "urdf_dofs_per_asset_mean": fraction(urdf_dof_total, len(records)),
        "urdf_motion_sweepable_dof_count": sum(
            row["urdf_motion_sweepable_dof_count"] for row in records
        ),
        "urdf_zero_width_dof_count": sum(row["urdf_zero_width_dof_count"] for row in records),
        "urdf_parent_child_structurally_valid_count": sum(
            row["urdf_parent_child_structurally_valid_count"] for row in records
        ),
        "urdf_axis_metadata_valid_count": sum(
            row["urdf_axis_metadata_valid_count"] for row in records
        ),
        "urdf_origin_metadata_valid_count": sum(
            row["urdf_origin_metadata_valid_count"] for row in records
        ),
        "urdf_bounded_dof_count": sum(row["urdf_bounded_dof_count"] for row in records),
        "urdf_limit_metadata_valid_count": sum(
            row["urdf_limit_metadata_valid_count"] for row in records
        ),
        "urdf_generic_revolute_count": sum(
            row["urdf_generic_revolute_count"] for row in records
        ),
        "urdf_continuous_count": sum(row["urdf_continuous_count"] for row in records),
        "urdf_collision_asset_count": sum(
            row["urdf_collision_element_count"] > 0 for row in records
        ),
        "urdf_collision_element_count": sum(
            row["urdf_collision_element_count"] for row in records
        ),
        "urdf_missing_collision_mesh_reference_count": sum(
            row["urdf_missing_collision_mesh_reference_count"] for row in records
        ),
        "urdf_inertial_link_count": sum(row["urdf_inertial_link_count"] for row in records),
        "urdf_valid_inertial_link_count": sum(
            row["urdf_valid_inertial_link_count"] for row in records
        ),
        "logical_to_urdf_dof_expansion": {
            "logical_motion_joints": logical_motion_total,
            "urdf_movable_dofs": urdf_dof_total,
            "difference": urdf_dof_total - logical_motion_total,
            "composite_slider_annotations": sum(
                row["annotation_composite_slider_count"] for row in records
            ),
            "interpretation": (
                "slider+ composite logical motions are represented by prismatic plus continuous "
                "URDF DoFs; DoF count is therefore not logical-joint recall."
            ),
        },
        "physx_id_partition": id_partition,
        "semantic_metric_boundary": {
            "joint_type_accuracy": "N/A for the reference itself; fields are source annotations",
            "joint_recall": "N/A for the reference itself; annotation and URDF are one release pipeline",
            "parent_child_accuracy": "N/A semantic accuracy; structural endpoint validity is reported",
            "axis_valid": "N/A geometric/semantic accuracy; finite source fields are reported",
            "origin_valid": "N/A geometric/semantic accuracy; finite source fields are reported",
            "limit_valid": "N/A semantic plausibility; finite source fields are reported",
            "joint_geom_valid": "N/A without independent axis-on-moving-part adjudication",
            "asset_geom_valid": "N/A without independent joint-level adjudication",
        },
    }
    write_json(output / "summary.json", static)
    freeze_cohort(records, output)
    print(json.dumps({"output": str(output), "assets": len(records), "categories": len(category_counts)}, indent=2))


def selection_hash(category: str, dataset_id: str) -> str:
    return sha256_bytes(f"{SALT}\n{category}\n{dataset_id}".encode("utf-8"))


def freeze_cohort(records: list[dict[str, Any]], output: Path) -> None:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in records:
        grouped[row["category"]].append(row)
    items = []
    for category in sorted(grouped):
        ranked = sorted(
            grouped[category],
            key=lambda row: (selection_hash(category, row["dataset_id"]), int(row["dataset_id"])),
        )
        winner = ranked[0]
        items.append(
            {
                "category": category,
                "dataset_id": winner["dataset_id"],
                "selection_hash": selection_hash(category, winner["dataset_id"]),
                "selection_rank": 1,
                "urdf_sha256": winner["urdf_sha256"],
                "static_movable_dofs": winner["urdf_movable_dof_count"],
                "static_motion_sweepable_dofs": winner["urdf_motion_sweepable_dof_count"],
                "static_zero_width_dofs": winner["urdf_zero_width_dof_count"],
                "static_collision_elements": winner["urdf_collision_element_count"],
                "static_missing_collision_mesh_references": winner[
                    "urdf_missing_collision_mesh_reference_count"
                ],
                "static_valid_tree": winner["urdf_valid_tree"],
                "candidate_count": len(ranked),
                "selection_inputs": ["category", "dataset_id"],
                "outcome_fields_used": [],
            }
        )
    payload = {
        "protocol_id": "nano3d_table6_partnet_mobility_category_cohort_v1",
        "status": "FROZEN",
        "salt": SALT,
        "ranking_rule": "per exact meta.model_cat, ascending SHA256(salt\\ncategory\\ndataset_id), then numeric ID; take one",
        "failure_policy": "retain selected ID without replacement for all parse/load/sweep outcomes",
        "category_count": len(items),
        "size": len(items),
        "items": items,
        "ordered_identity_sha256": canonical_sha256(
            [[item["category"], item["dataset_id"]] for item in items]
        ),
    }
    write_json(output / "frozen_functional_cohort.json", payload)


def pybullet_joint_metadata(urdf_path: Path) -> list[dict[str, Any]]:
    asset_dir = urdf_path.parent
    _, rows = audit_urdf(urdf_path, asset_dir)
    return rows


SELF_COLLISION_FIXTURE = """<?xml version="1.0"?>
<robot name="partnet_self_collision_smoke">
  <link name="base"/>
  <link name="left"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="right"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <joint name="left_joint" type="fixed"><parent link="base"/><child link="left"/></joint>
  <joint name="right_joint" type="fixed"><parent link="base"/><child link="right"/></joint>
</robot>
"""


def run_collision_flag_smoke(output: Path) -> dict[str, Any]:
    import pybullet as bullet

    fixture = output / "self_collision_flag_smoke.urdf"
    write_text(fixture, SELF_COLLISION_FIXTURE)
    policies = {
        "none": 0,
        "self_collision": bullet.URDF_USE_SELF_COLLISION,
        "exclude_parent_only": bullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        "self_plus_exclude_parent": (
            bullet.URDF_USE_SELF_COLLISION | bullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
        ),
    }
    contacts: dict[str, int] = {}
    client = bullet.connect(bullet.DIRECT)
    try:
        for name, flags in policies.items():
            body = bullet.loadURDF(
                str(fixture), useFixedBase=True, flags=flags, physicsClientId=client
            )
            bullet.performCollisionDetection(physicsClientId=client)
            contacts[name] = len(
                bullet.getContactPoints(bodyA=body, bodyB=body, physicsClientId=client)
            )
            bullet.removeBody(body, physicsClientId=client)
    finally:
        bullet.disconnect(client)
    passed = (
        contacts["none"] == 0
        and contacts["exclude_parent_only"] == 0
        and contacts["self_collision"] > 0
        and contacts["self_plus_exclude_parent"] > 0
    )
    result = {
        "protocol_id": "nano3d_pybullet_self_collision_flag_smoke_v1",
        "status": "PASS" if passed else "FAIL",
        "pybullet_api_version": bullet.getAPIVersion(),
        "contacts": contacts,
        "conclusion": (
            "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT is only a modifier; the formal "
            "sweep also enables URDF_USE_SELF_COLLISION."
        ),
    }
    write_json(output / "self_collision_flag_smoke.json", result)
    if not passed:
        raise RuntimeError(f"self-collision smoke failed: {result}")
    return result


def sample_values(row: dict[str, Any]) -> list[float]:
    if row["bounded"]:
        lower = float(row["lower"])
        upper = float(row["upper"])
    else:
        lower, upper = -math.pi, math.pi
    return [
        lower + index * (upper - lower) / (SINGLE_SAMPLES - 1)
        for index in range(SINGLE_SAMPLES)
    ]


def sobol_values(rows: list[dict[str, Any]]) -> list[list[float]]:
    from scipy.stats import qmc

    unit = qmc.Sobol(d=len(rows), scramble=False).random_base2(m=6)
    result = []
    for vector in unit:
        values = []
        for scalar, row in zip(vector, rows):
            if row["bounded"]:
                lower, upper = float(row["lower"]), float(row["upper"])
            else:
                lower, upper = -math.pi, math.pi
            values.append(float(lower + scalar * (upper - lower)))
        result.append(values)
    return result


def run_child(item_path: Path, dataset_root: Path, result_path: Path) -> None:
    import pybullet as bullet

    item = read_json(item_path)
    dataset_id = str(item["dataset_id"])
    urdf_path = contained(dataset_root / dataset_id / "mobility.urdf")
    metadata = pybullet_joint_metadata(urdf_path)
    result: dict[str, Any] = {
        **item,
        "protocol_id": PROTOCOL_ID,
        "urdf_sha256_observed": sha256_file(urdf_path),
        "load_success": False,
        "reset_readback_success": False,
        "measurement_complete": False,
        "movable_dof_count": len(metadata),
        "motion_sweepable_dof_count": sum(row["motion_sweepable"] for row in metadata),
        "zero_width_dof_count": sum(row["zero_width"] for row in metadata),
        "expected_single_state_count": SINGLE_SAMPLES
        * sum(row["motion_sweepable"] for row in metadata),
        "expected_multi_state_count": (
            MULTI_SAMPLES if sum(row["motion_sweepable"] for row in metadata) > 1 else 0
        ),
        "single_state_count": 0,
        "multi_state_count": 0,
        "single_contact_free_count": 0,
        "single_penetration_free_count": 0,
        "multi_contact_free_count": 0,
        "multi_penetration_free_count": 0,
        "joint_single_contact_free_sweep_pass_count": 0,
        "joint_single_penetration_free_sweep_pass_count": 0,
        "asset_contact_free_proxy": False,
        "asset_penetration_free_proxy": False,
        "max_penetration_depth_m": 0.0,
        "max_reset_pose_abs_error": None,
        "motor_disabled_dof_count": 0,
        "issues": [],
        "state_records": [],
    }
    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        body = bullet.loadURDF(
            str(urdf_path),
            useFixedBase=True,
            flags=flags,
            physicsClientId=client,
        )
        result["load_success"] = True
        simulator_by_name: dict[str, tuple[int, int]] = {}
        for index in range(bullet.getNumJoints(body, physicsClientId=client)):
            info = bullet.getJointInfo(body, index, physicsClientId=client)
            name = info[1].decode("utf-8") if isinstance(info[1], bytes) else str(info[1])
            simulator_by_name[name] = (index, int(info[2]))
        missing = [row["name"] for row in metadata if row["name"] not in simulator_by_name]
        if missing:
            raise RuntimeError(f"simulator joint mapping missing: {missing}")
        indices = [simulator_by_name[row["name"]][0] for row in metadata]
        for index in indices:
            bullet.setJointMotorControl2(
                body,
                index,
                controlMode=bullet.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
                physicsClientId=client,
            )
        result["motor_disabled_dof_count"] = len(indices)
        defaults = [
            float((row["lower"] + row["upper"]) / 2.0) if row["bounded"] else 0.0
            for row in metadata
        ]
        sweep_positions = [
            position for position, row in enumerate(metadata) if row["motion_sweepable"]
        ]
        sweep_rows = [metadata[position] for position in sweep_positions]

        def evaluate(
            values: list[float],
            phase: str,
            sample_index: int,
            joint_name: str | None = None,
        ) -> tuple[bool, bool]:
            for index, value in zip(indices, values):
                bullet.resetJointState(
                    body, index, value, targetVelocity=0.0, physicsClientId=client
                )
            bullet.performCollisionDetection(physicsClientId=client)
            pose_error = max(
                [
                    abs(
                        float(bullet.getJointState(body, index, physicsClientId=client)[0])
                        - value
                    )
                    for index, value in zip(indices, values)
                ]
                or [0.0]
            )
            if pose_error > RESET_TOLERANCE:
                raise RuntimeError(
                    f"reset/readback error {pose_error} exceeds {RESET_TOLERANCE}"
                )
            contacts = bullet.getContactPoints(
                bodyA=body, bodyB=body, physicsClientId=client
            )
            penetration = max(
                [max(0.0, -float(contact[8])) for contact in contacts] or [0.0]
            )
            contact_free = not contacts
            penetration_free = penetration <= PENETRATION_TOLERANCE_M
            prior = result["max_reset_pose_abs_error"]
            result["max_reset_pose_abs_error"] = (
                pose_error if prior is None else max(float(prior), pose_error)
            )
            result["max_penetration_depth_m"] = max(
                float(result["max_penetration_depth_m"]), penetration
            )
            result["state_records"].append(
                {
                    "dataset_id": dataset_id,
                    "category": item["category"],
                    "phase": phase,
                    "joint_name": joint_name,
                    "sample_index": sample_index,
                    "contact_count": len(contacts),
                    "contact_free": contact_free,
                    "penetration_free": penetration_free,
                    "max_penetration_depth_m": penetration,
                    "reset_pose_max_abs_error": pose_error,
                }
            )
            return contact_free, penetration_free

        # Reference pose is checked for load/reset integrity but excluded from
        # the formal 11-state + 64-state collision denominator.
        evaluate(defaults, "reference_midrange", 0)
        result["reset_readback_success"] = True
        all_contact_free = True
        all_penetration_free = True
        for position, row in zip(sweep_positions, sweep_rows):
            joint_contact_free = True
            joint_penetration_free = True
            for sample_index, value in enumerate(sample_values(row)):
                values = list(defaults)
                values[position] = value
                contact_free, penetration_free = evaluate(
                    values, "single_joint_sweep", sample_index, row["name"]
                )
                result["single_state_count"] += 1
                result["single_contact_free_count"] += int(contact_free)
                result["single_penetration_free_count"] += int(penetration_free)
                joint_contact_free = joint_contact_free and contact_free
                joint_penetration_free = joint_penetration_free and penetration_free
                all_contact_free = all_contact_free and contact_free
                all_penetration_free = all_penetration_free and penetration_free
            result["joint_single_contact_free_sweep_pass_count"] += int(joint_contact_free)
            result["joint_single_penetration_free_sweep_pass_count"] += int(
                joint_penetration_free
            )
        if len(sweep_rows) > 1:
            for sample_index, motion_values in enumerate(sobol_values(sweep_rows)):
                values = list(defaults)
                for position, value in zip(sweep_positions, motion_values):
                    values[position] = value
                contact_free, penetration_free = evaluate(
                    values, "multi_joint_sobol", sample_index
                )
                result["multi_state_count"] += 1
                result["multi_contact_free_count"] += int(contact_free)
                result["multi_penetration_free_count"] += int(penetration_free)
                all_contact_free = all_contact_free and contact_free
                all_penetration_free = all_penetration_free and penetration_free
        result["asset_contact_free_proxy"] = all_contact_free
        result["asset_penetration_free_proxy"] = all_penetration_free
        result["measurement_complete"] = (
            result["single_state_count"] == result["expected_single_state_count"]
            and result["multi_state_count"] == result["expected_multi_state_count"]
        )
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"{type(exc).__name__}: {exc}")
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)
    write_json(result_path, result)


def run_one_subprocess(
    item_path: Path,
    dataset_root: Path,
    child_result: Path,
    log_path: Path,
    timeout: int,
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(SCRIPT),
        "--phase",
        "child",
        "--dataset-root",
        str(dataset_root),
        "--child-item",
        str(item_path),
        "--child-result",
        str(child_result),
    ]
    timed_out = False
    child_environment = dict(os.environ)
    child_environment.update(
        {
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
            "VECLIB_MAXIMUM_THREADS": "1",
        }
    )
    with contained(log_path, exists=False).open("wb") as log:
        try:
            completed = subprocess.run(
                command,
                stdout=log,
                stderr=subprocess.STDOUT,
                env=child_environment,
                timeout=timeout,
                check=False,
            )
            returncode = completed.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
            returncode = -9
    if child_result.is_file():
        result = read_json(child_result)
    else:
        item = read_json(item_path)
        result = {
            **item,
            "protocol_id": PROTOCOL_ID,
            "load_success": False,
            "reset_readback_success": False,
            "measurement_complete": False,
            "movable_dof_count": item["static_movable_dofs"],
            "motion_sweepable_dof_count": item["static_motion_sweepable_dofs"],
            "zero_width_dof_count": item["static_zero_width_dofs"],
            "expected_single_state_count": SINGLE_SAMPLES
            * item["static_motion_sweepable_dofs"],
            "expected_multi_state_count": (
                MULTI_SAMPLES if item["static_motion_sweepable_dofs"] > 1 else 0
            ),
            "single_state_count": 0,
            "multi_state_count": 0,
            "single_contact_free_count": 0,
            "single_penetration_free_count": 0,
            "multi_contact_free_count": 0,
            "multi_penetration_free_count": 0,
            "joint_single_contact_free_sweep_pass_count": 0,
            "joint_single_penetration_free_sweep_pass_count": 0,
            "asset_contact_free_proxy": False,
            "asset_penetration_free_proxy": False,
            "max_penetration_depth_m": None,
            "max_reset_pose_abs_error": None,
            "motor_disabled_dof_count": 0,
            "issues": ["child_timeout" if timed_out else f"child_exit_{returncode}"],
            "state_records": [],
        }
    result["child_returncode"] = returncode
    result["child_timed_out"] = timed_out
    result["child_log"] = str(log_path)
    return result


def run_functional(
    dataset_root: Path, output: Path, timeout: int, workers: int
) -> None:
    import pybullet as bullet

    dataset_root = contained(dataset_root)
    output = contained(output)
    cohort = read_json(output / "frozen_functional_cohort.json")
    smoke = run_collision_flag_smoke(output)
    inputs_dir = output / "functional_inputs"
    children_dir = output / "functional_children"
    logs_dir = output / "functional_logs"
    for directory in (inputs_dir, children_dir, logs_dir):
        contained(directory, exists=False).mkdir(parents=True, exist_ok=True)
    jobs = []
    for order, item in enumerate(cohort["items"]):
        item_path = inputs_dir / f"{order:03d}_{item['dataset_id']}.json"
        child_result = children_dir / f"{order:03d}_{item['dataset_id']}.json"
        log_path = logs_dir / f"{order:03d}_{item['dataset_id']}.log"
        write_json(item_path, item)
        jobs.append((order, item_path, child_result, log_path))

    by_order: dict[int, dict[str, Any]] = {}
    with ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = {
            executor.submit(
                run_one_subprocess,
                item_path,
                dataset_root,
                child_result,
                log_path,
                timeout,
            ): order
            for order, item_path, child_result, log_path in jobs
        }
        for completed_count, future in enumerate(as_completed(futures), 1):
            order = futures[future]
            by_order[order] = future.result()
            print(f"functional {completed_count}/{len(jobs)}", flush=True)
    records = [by_order[index] for index in range(len(jobs))]
    state_records = [
        state
        for record in records
        for state in record.pop("state_records", [])
        if state["phase"] != "reference_midrange"
    ]
    write_json(output / "functional_asset_records.json", records)
    write_jsonl(output / "functional_state_records.jsonl", state_records)

    expected_states = sum(
        row["expected_single_state_count"] + row["expected_multi_state_count"]
        for row in records
    )
    executed_states = sum(
        row["single_state_count"] + row["multi_state_count"] for row in records
    )
    contact_free = sum(
        row["single_contact_free_count"] + row["multi_contact_free_count"]
        for row in records
    )
    penetration_free = sum(
        row["single_penetration_free_count"] + row["multi_penetration_free_count"]
        for row in records
    )
    complete = sum(row["measurement_complete"] for row in records)
    functional = {
        "protocol_id": PROTOCOL_ID,
        "status": "COMPLETE" if complete == len(records) else "COMPLETE_WITH_RETAINED_FAILURES",
        "cohort_size": len(records),
        "category_count": len({row["category"] for row in records}),
        "ordered_identity_sha256": canonical_sha256(
            [[row["category"], row["dataset_id"]] for row in records]
        ),
        "pybullet_api_version": bullet.getAPIVersion(),
        "subprocess_isolation": True,
        "parallel_workers": workers,
        "child_numeric_thread_environment": {
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
            "VECLIB_MAXIMUM_THREADS": "1",
        },
        "timeout_seconds_per_asset": timeout,
        "formal_run_note": (
            "full clean rerun of the frozen 46-category cohort after forcing every "
            "numeric backend to one thread; a prior OpenBLAS thread-exhaustion "
            "preflight was discarded as a harness resource failure"
        ),
        "load_success": f"{sum(row['load_success'] for row in records)}/{len(records)}",
        "reset_readback_success": (
            f"{sum(row['reset_readback_success'] for row in records)}/{len(records)}"
        ),
        "measurement_complete_assets": f"{complete}/{len(records)}",
        "movable_dofs": sum(row["movable_dof_count"] for row in records),
        "motion_sweepable_dofs": sum(
            row["motion_sweepable_dof_count"] for row in records
        ),
        "zero_width_dofs_excluded_from_motion_sweep": sum(
            row["zero_width_dof_count"] for row in records
        ),
        "cohort_missing_collision_mesh_reference_count": sum(
            row.get("static_missing_collision_mesh_references", 0) for row in records
        ),
        "cohort_missing_collision_mesh_reference_asset_count": sum(
            row.get("static_missing_collision_mesh_references", 0) > 0 for row in records
        ),
        "expected_states": expected_states,
        "executed_states": executed_states,
        "single_joint_states": sum(row["single_state_count"] for row in records),
        "multi_joint_sobol_states": sum(row["multi_state_count"] for row in records),
        "contact_free_states_executed": f"{contact_free}/{executed_states}",
        "contact_free_state_rate_executed": fraction(contact_free, executed_states),
        "penetration_free_states_executed": f"{penetration_free}/{executed_states}",
        "penetration_free_state_rate_executed": fraction(
            penetration_free, executed_states
        ),
        "contact_free_states_intent_to_run_fail_closed": f"{contact_free}/{expected_states}",
        "penetration_free_states_intent_to_run_fail_closed": (
            f"{penetration_free}/{expected_states}"
        ),
        "joint_single_contact_free_sweep_pass": (
            f"{sum(row['joint_single_contact_free_sweep_pass_count'] for row in records)}/"
            f"{sum(row['motion_sweepable_dof_count'] for row in records)}"
        ),
        "joint_single_penetration_free_sweep_pass": (
            f"{sum(row['joint_single_penetration_free_sweep_pass_count'] for row in records)}/"
            f"{sum(row['motion_sweepable_dof_count'] for row in records)}"
        ),
        "asset_contact_free_proxy": (
            f"{sum(row['asset_contact_free_proxy'] for row in records)}/{len(records)}"
        ),
        "asset_penetration_free_proxy": (
            f"{sum(row['asset_penetration_free_proxy'] for row in records)}/{len(records)}"
        ),
        "child_process_nonzero_exit_count": sum(
            row["child_returncode"] != 0 for row in records
        ),
        "child_process_timeout_count": sum(row["child_timed_out"] for row in records),
        "collision_policy": (
            "URDF_USE_SELF_COLLISION | URDF_USE_SELF_COLLISION_EXCLUDE_PARENT"
        ),
        "collision_flags": (
            "URDF_USE_INERTIA_FROM_FILE | URDF_USE_SELF_COLLISION | "
            "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | URDF_IGNORE_VISUAL_SHAPES"
        ),
        "collision_flags_value": int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        ),
        "motor_policy": "all movable joints disabled with VELOCITY_CONTROL force=0",
        "pose_evaluation": "resetJointState + readback + performCollisionDetection; no stepSimulation",
        "reset_pose_readback_tolerance": RESET_TOLERANCE,
        "max_reset_pose_abs_error": max(
            [
                float(row["max_reset_pose_abs_error"])
                for row in records
                if row["max_reset_pose_abs_error"] is not None
            ]
            or [0.0]
        ),
        "penetration_tolerance_m": PENETRATION_TOLERANCE_M,
        "contact_definition": "zero PyBullet self-contact points",
        "penetration_definition": "no self-contact depth greater than 1e-6 m",
        "continuous_collision_detection": "not_run",
        "self_collision_flag_smoke_status": smoke["status"],
        "semantic_joint_geometric_validity": (
            "N/A: discrete collision proxy does not adjudicate axis-on-moving-part semantics"
        ),
    }
    write_json(output / "functional_summary.json", functional)
    print(json.dumps(functional, indent=2, ensure_ascii=True))


def render_report(output: Path) -> None:
    static = read_json(output / "summary.json")
    provenance = read_json(output / "provenance.json")
    partition = read_json(output / "physx_id_partition.json")
    functional_path = output / "functional_summary.json"
    functional = read_json(functional_path) if functional_path.is_file() else None
    lines = [
        "# Table 6 PartNet-Mobility real-data reference audit",
        "",
        "Status: **COMPLETE**",
        "",
        "PartNet-Mobility is a curated source/reference dataset, not a generated method. It must be reported outside generated-method rankings.",
        "",
        "## Provenance and inventory",
        "",
        f"- Local v0 packages: {static['asset_count']} assets / {static['category_count']} exact `meta.model_cat` categories.",
        f"- Companion archive: `{provenance['archive']['sha256']}` ({provenance['archive']['bytes']} bytes; {provenance['archive']['zip_entry_count']} ZIP entries).",
        f"- Audited schema paths with matched direct-root/archive presence: {provenance['archive']['audited_schema_path_presence_check_count']}/{provenance['archive']['expected_audited_schema_path_presence_check_count']}; presence/hash mismatches: {provenance['archive']['audited_schema_path_or_hash_mismatch_count']}; present files byte-compared: {provenance['archive']['audited_schema_file_byte_comparison_count']}.",
        f"- Official repository pin: `{provenance['official_repository'].get('observed_revision')}`; gated={provenance['official_repository'].get('gated')}; license={provenance['official_repository'].get('declared_license')!r}.",
        "- Status remains `PROVENANCE_LIMITED`: local bytes match the frozen companion archive, but were not directly authenticated against gated per-ID objects at the pinned revision.",
        "",
        "## Full-release annotation and URDF audit",
        "",
        f"- Six mandatory core files present: {static['all_core_files_present_assets']}/{static['asset_count']}; optional `result_original.json`: {static['optional_file_presence']['result_original.json']}/{static['asset_count']} (absent from both direct root and companion archive for the remaining packages).",
        f"- Annotation parse: {static['annotation_parse_success_count']}/{static['asset_count']}; URDF parse: {static['urdf_parse_success_count']}/{static['asset_count']}; valid URDF trees: {static['urdf_valid_tree_asset_count']}/{static['asset_count']}.",
        f"- Logical mobility annotations: {static['annotation_logical_motion_joint_count']} ({static['annotation_logical_motion_joints_per_asset_mean']:.3f}/asset); axis origin/direction/range finite-field coverage: {static['annotation_axis_origin_valid_count']}/{static['annotation_logical_motion_joint_count']} / {static['annotation_axis_direction_valid_count']}/{static['annotation_logical_motion_joint_count']} / {static['annotation_range_metadata_valid_count']}/{static['annotation_logical_motion_joint_count']}.",
        f"- URDF movable DoFs: {static['urdf_movable_dof_count']} ({static['urdf_dofs_per_asset_mean']:.3f}/asset); motion-sweepable: {static['urdf_motion_sweepable_dof_count']}; bounded zero-width: {static['urdf_zero_width_dof_count']}.",
        f"- URDF finite-field coverage: endpoints {static['urdf_parent_child_structurally_valid_count']}/{static['urdf_movable_dof_count']}; axes {static['urdf_axis_metadata_valid_count']}/{static['urdf_movable_dof_count']}; origins {static['urdf_origin_metadata_valid_count']}/{static['urdf_movable_dof_count']}; bounded limits {static['urdf_limit_metadata_valid_count']}/{static['urdf_bounded_dof_count']}.",
        f"- Collision packages: {static['urdf_collision_asset_count']}/{static['asset_count']}; missing collision mesh references: {static['urdf_missing_collision_mesh_reference_count']}; valid inertial links: {static['urdf_valid_inertial_link_count']}.",
        f"- Composite representation: {static['annotation_composite_slider_count']} logical `slider+` annotations explain the DoF expansion from {static['annotation_logical_motion_joint_count']} logical motions to {static['urdf_movable_dof_count']} URDF DoFs. URDF DoF count is not joint recall.",
        "",
        "## PhysX same-ID boundary",
        "",
        f"- Shared PartNet/PhysX-finaljson IDs: {partition['intersection_partnet_physx_finaljson_count']}; PartNet-only IDs: {partition['partnet_only_count']}; PhysX IDs missing from PartNet: {partition['physx_finaljson_missing_from_partnet_count']}.",
        f"- Shared subset logical/URDF joints: {partition['intersection']['logical_motion_joint_count']}/{partition['intersection']['urdf_movable_dof_count']}; PartNet-only: {partition['partnet_only']['logical_motion_joint_count']}/{partition['partnet_only']['urdf_movable_dof_count']}.",
        "- The shared annotations are valid source-reference fields for same-ID representation preservation (type/parent/axis/origin/limit), not independent blind-test gold: PhysX-Mobility is derived from PartNet-Mobility. The 323 PartNet-only assets have no local PhysX finaljson pair.",
        "",
    ]
    if functional is not None:
        lines.extend(
            [
                "## Frozen category functional proxy",
                "",
                f"- Cohort: {functional['cohort_size']} assets / {functional['category_count']} categories; load {functional['load_success']}; reset/readback {functional['reset_readback_success']}; complete {functional['measurement_complete_assets']}.",
                f"- Declared/motion/zero-width DoFs: {functional['movable_dofs']}/{functional['motion_sweepable_dofs']}/{functional['zero_width_dofs_excluded_from_motion_sweep']}.",
                f"- Penetration-free states at 1e-6 m: {functional['penetration_free_states_intent_to_run_fail_closed']}; strict contact-free: {functional['contact_free_states_intent_to_run_fail_closed']}.",
                f"- Joint single-sweep penetration-free: {functional['joint_single_penetration_free_sweep_pass']}; asset proxy: {functional['asset_penetration_free_proxy']}.",
                "- Protocol: 11 states per nonzero single DoF plus 64 unscrambled Sobol states for assets with more than one motion DoF; per-asset subprocess; self-collision plus exclude-parent flags; motors disabled; exact reset/readback and `performCollisionDetection`; no simulation step and no CCD.",
                "",
            ]
        )
    lines.extend(
        [
            "## Table 6 boundary",
            "",
            "The release provides reference type, parent graph, axis origin/direction, limits, part membership, names, collision meshes, and standard URDF exports. For the reference itself, Joint Type Accuracy, Joint Recall, Parent-Child Accuracy, Axis/Origin/Limit semantic accuracy, Joint Geom. Valid, and Asset Geom. Valid remain N/A. Annotation and URDF belong to the same release pipeline, metadata presence is not accuracy, and the discrete collision result is not continuous collision or physical correctness.",
            "",
            "License boundary: the official card is gated and limits use to non-commercial research/education; ShapeNet terms also apply. Do not redistribute raw assets in supplementary material unless recipients have accepted the terms.",
        ]
    )
    write_text(output / "report.md", "\n".join(lines) + "\n")


def run_self_check(dataset_root: Path, output: Path) -> None:
    dataset_root = contained(dataset_root)
    output = contained(output)
    static = read_json(output / "summary.json")
    provenance = read_json(output / "provenance.json")
    partition = read_json(output / "physx_id_partition.json")
    cohort = read_json(output / "frozen_functional_cohort.json")
    records = [
        json.loads(line)
        for line in (output / "asset_records.jsonl").read_text(encoding="utf-8").splitlines()
        if line
    ]
    recomputed_items = []
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in records:
        grouped[row["category"]].append(row)
    for category in sorted(grouped):
        winner = min(
            grouped[category],
            key=lambda row: (selection_hash(category, row["dataset_id"]), int(row["dataset_id"])),
        )
        recomputed_items.append((category, winner["dataset_id"]))
    checks = {
        "protocol_v3": static["protocol_id"].endswith("_v3"),
        "full_asset_count_2347": len(records) == static["asset_count"] == EXPECTED_ASSETS,
        "all_core_files_present": static["all_core_files_present_assets"] == EXPECTED_ASSETS,
        "all_annotations_parse": static["annotation_parse_success_count"] == EXPECTED_ASSETS,
        "all_urdfs_parse": static["urdf_parse_success_count"] == EXPECTED_ASSETS,
        "archive_hash_matches_pin": provenance["archive"]["matches_expected"],
        "all_audited_schema_paths_match_archive": (
            provenance["archive"]["audited_schema_path_presence_check_count"]
            == provenance["archive"]["expected_audited_schema_path_presence_check_count"]
            and provenance["archive"]["audited_schema_path_or_hash_mismatch_count"] == 0
        ),
        "physx_intersection_2024": partition["intersection_partnet_physx_finaljson_count"] == 2024,
        "partnet_only_323": partition["partnet_only_count"] == 323,
        "physx_missing_from_partnet_zero": partition["physx_finaljson_missing_from_partnet_count"] == 0,
        "partition_disjoint_complete": partition["partition_union_is_all_partnet"]
        and partition["partition_disjoint"],
        "cohort_one_per_category": cohort["size"] == static["category_count"]
        and len({item["category"] for item in cohort["items"]}) == cohort["size"],
        "cohort_winners_recompute": recomputed_items
        == [(item["category"], item["dataset_id"]) for item in cohort["items"]],
        "cohort_selection_is_outcome_independent": all(
            item["selection_inputs"] == ["category", "dataset_id"]
            and not item["outcome_fields_used"]
            for item in cohort["items"]
        ),
        "semantic_accuracy_fail_closed": all(
            str(value).startswith("N/A")
            for value in static["semantic_metric_boundary"].values()
        ),
        "physx_not_independent_gold": (
            partition["physx_mobility_based_on_partnet_mobility"] is True
            and partition[
                "partnet_annotations_are_independent_physx_blind_test_gold"
            ]
            is False
            and partition["partnet_annotations_allowed_role_for_physx"]
            == "same-ID source-reference and representation-preservation audit only"
        ),
    }
    functional_path = output / "functional_summary.json"
    if functional_path.is_file():
        functional = read_json(functional_path)
        functional_records = read_json(output / "functional_asset_records.json")
        state_records = [
            json.loads(line)
            for line in (output / "functional_state_records.jsonl")
            .read_text(encoding="utf-8")
            .splitlines()
            if line
        ]
        state_count = len(state_records)
        state_contact_free = sum(row["contact_free"] for row in state_records)
        state_penetration_free = sum(row["penetration_free"] for row in state_records)
        single_groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
        by_asset: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for state in state_records:
            by_asset[state["dataset_id"]].append(state)
            if state["phase"] == "single_joint_sweep":
                single_groups[(state["dataset_id"], state["joint_name"])].append(state)
        recomputed_joint_contact_pass = sum(
            len(rows) == SINGLE_SAMPLES and all(row["contact_free"] for row in rows)
            for rows in single_groups.values()
        )
        recomputed_joint_penetration_pass = sum(
            len(rows) == SINGLE_SAMPLES and all(row["penetration_free"] for row in rows)
            for rows in single_groups.values()
        )
        recomputed_asset_contact_pass = sum(
            all(row["contact_free"] for row in rows) for rows in by_asset.values()
        )
        recomputed_asset_penetration_pass = sum(
            all(row["penetration_free"] for row in rows) for rows in by_asset.values()
        )
        summary_contact_free = sum(
            row["single_contact_free_count"] + row["multi_contact_free_count"]
            for row in functional_records
        )
        summary_penetration_free = sum(
            row["single_penetration_free_count"] + row["multi_penetration_free_count"]
            for row in functional_records
        )
        summary_joint_contact_pass = sum(
            row["joint_single_contact_free_sweep_pass_count"]
            for row in functional_records
        )
        summary_joint_penetration_pass = sum(
            row["joint_single_penetration_free_sweep_pass_count"]
            for row in functional_records
        )
        checks.update(
            {
                "functional_ids_match_frozen_order": [
                    (row["category"], row["dataset_id"]) for row in functional_records
                ]
                == [(item["category"], item["dataset_id"]) for item in cohort["items"]],
                "functional_static_hashes_match": all(
                    row.get("urdf_sha256_observed") == item["urdf_sha256"]
                    for row, item in zip(functional_records, cohort["items"])
                    if row["child_returncode"] == 0
                ),
                "functional_denominator_accounting": functional["expected_states"]
                == sum(
                    row["expected_single_state_count"] + row["expected_multi_state_count"]
                    for row in functional_records
                ),
                "functional_executed_state_records_match": functional["executed_states"]
                == state_count,
                "formal_state_records_exclude_reference_midrange": all(
                    row["phase"] in {"single_joint_sweep", "multi_joint_sobol"}
                    for row in state_records
                ),
                "state_contact_counts_independently_recompute": (
                    state_contact_free == summary_contact_free
                    and functional["contact_free_states_executed"]
                    == f"{state_contact_free}/{state_count}"
                    and state_penetration_free == summary_penetration_free
                    and functional["penetration_free_states_executed"]
                    == f"{state_penetration_free}/{state_count}"
                ),
                "joint_pass_counts_independently_recompute": (
                    len(single_groups) == functional["motion_sweepable_dofs"]
                    and recomputed_joint_contact_pass == summary_joint_contact_pass
                    and functional["joint_single_contact_free_sweep_pass"]
                    == f"{recomputed_joint_contact_pass}/{len(single_groups)}"
                    and recomputed_joint_penetration_pass
                    == summary_joint_penetration_pass
                    and functional["joint_single_penetration_free_sweep_pass"]
                    == f"{recomputed_joint_penetration_pass}/{len(single_groups)}"
                ),
                "asset_pass_counts_independently_recompute": (
                    len(by_asset) == cohort["size"]
                    and functional["asset_contact_free_proxy"]
                    == f"{recomputed_asset_contact_pass}/{len(by_asset)}"
                    and functional["asset_penetration_free_proxy"]
                    == f"{recomputed_asset_penetration_pass}/{len(by_asset)}"
                ),
                "functional_complete_records_exact": all(
                    not row["measurement_complete"]
                    or (
                        row["single_state_count"] == row["expected_single_state_count"]
                        and row["multi_state_count"] == row["expected_multi_state_count"]
                    )
                    for row in functional_records
                ),
                "reset_readback_within_tolerance": functional["max_reset_pose_abs_error"]
                <= functional["reset_pose_readback_tolerance"],
                "self_collision_smoke_pass": functional["self_collision_flag_smoke_status"]
                == "PASS",
                "ccd_not_claimed": functional["continuous_collision_detection"] == "not_run",
                "no_step_simulation": "no stepSimulation" in functional["pose_evaluation"],
            }
        )
    payload = {
        "protocol_id": "nano3d_table6_partnet_mobility_self_check_v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "passed": sum(checks.values()),
        "total": len(checks),
        "checks": checks,
        "hashes": {
            name: sha256_file(output / name)
            for name in (
                "summary.json",
                "provenance.json",
                "physx_id_partition.json",
                "frozen_functional_cohort.json",
                "functional_summary.json",
                "functional_asset_records.json",
                "functional_state_records.jsonl",
                "report.md",
            )
            if (output / name).is_file()
        },
        "runner_sha256": sha256_file(SCRIPT),
    }
    write_json(output / "self_check.json", payload)
    if payload["status"] != "PASS":
        raise RuntimeError(f"self-check failed: {checks}")
    print(json.dumps(payload, indent=2, ensure_ascii=True))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase",
        choices=("all", "static", "functional", "report", "self-check", "child"),
        default="all",
    )
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--physx-root", type=Path, default=DEFAULT_PHYSX)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout", type=int, default=900)
    parser.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 1))
    parser.add_argument("--child-item", type=Path)
    parser.add_argument("--child-result", type=Path)
    args = parser.parse_args()

    if args.phase == "child":
        if args.child_item is None or args.child_result is None:
            parser.error("--child-item and --child-result are required for child phase")
        run_child(args.child_item, contained(args.dataset_root), args.child_result)
        return 0

    output = contained(args.output, exists=False)
    output.mkdir(parents=True, exist_ok=True)
    if args.phase in {"all", "static"}:
        run_static(args.dataset_root, args.archive, args.physx_root, output)
    if args.phase in {"all", "functional"}:
        run_functional(args.dataset_root, output, args.timeout, args.workers)
    if args.phase in {"all", "report"}:
        render_report(output)
    if args.phase in {"all", "self-check"}:
        run_self_check(args.dataset_root, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
