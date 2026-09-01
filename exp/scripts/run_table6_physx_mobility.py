#!/usr/bin/env python3
"""Audit the official PhysX-Mobility release for Nano3D Table 6.

PhysX-Mobility is a PartNet-Mobility derivative dataset, not a generated-method
output.  The official archive URDFs also contain no collision geometry.  This
runner therefore reports package/export fidelity and PyBullet joint
load/reset/readback only.  It deliberately refuses to turn vacuous zero-contact
observations into collision or geometric-validity scores.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
import hashlib
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import sys
import time
from typing import Any
import xml.etree.ElementTree as ET
import zipfile


PHYSX_ROOT = Path(
    "/mnt/zsn/zsn_workspace/Ctrl-3D-trellis2-controlnet-dev/demo/physical_edit_demo/"
    "third_party/physx_mobility/extracted/PhysX_mobility"
)
ARCHIVE = PHYSX_ROOT.parents[1] / "PhysX-Mobility.zip"
HF_METADATA = PHYSX_ROOT.parents[1] / ".cache/huggingface/download/PhysX-Mobility.zip.metadata"
PARTNET_ROOT = Path("/mnt/zsn/lyb/PartNet_Mobility/data/dataset")
OUTPUT = Path(__file__).resolve().parents[1] / "runtime/table6_physx_mobility"
HF_REVISION = "d0768ee9e1415f6be8db78d6389ba018b85134c0"
ARCHIVE_SHA256 = "88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908"
SELECTION_SALT = "nano3d-table6-physx-mobility-one-per-partnet-category-v1"
SINGLE_STATES = 11
SOBOL_STATES = 64
GENERIC_REVOLUTE_DEG = 300.0
ZERO_WIDTH_TOLERANCE = 1e-12
RESET_TOLERANCE = 1e-9
CHILD_TIMEOUT_SECONDS = 900
MOVABLE_TYPES = {"revolute", "continuous", "prismatic", "planar", "floating"}
FUNCTIONAL_TYPES = {"revolute", "continuous", "prismatic"}
AUTHORIZED_ROOTS = (Path("/mnt/zsn/lyb"), Path("/mnt/zsn/zsn_workspace"))


def contained(path: Path, *, exists: bool) -> Path:
    resolved = path.resolve(strict=exists)
    if not any(resolved == root or root in resolved.parents for root in AUTHORIZED_ROOTS):
        raise ValueError(f"path outside authorized roots: {resolved}")
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    raw = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(raw.encode("utf-8")).hexdigest()


def string_inventory_sha256(values: set[str]) -> str:
    digest = hashlib.sha256()
    for value in sorted(values):
        digest.update(value.encode("utf-8"))
        digest.update(b"\0")
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def parse_vector(raw: str | None, *, default: list[float] | None = None) -> list[float] | None:
    if raw is None:
        return default
    try:
        values = [float(value) for value in raw.split()]
    except ValueError:
        return None
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        return None
    return values


def norm(values: list[float] | None) -> float:
    return math.sqrt(sum(value * value for value in values or []))


def parse_urdf(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError(f"expected robot root, found {root.tag!r}")
    link_nodes = root.findall("link")
    link_names = [node.attrib.get("name", "") for node in link_nodes]
    links = set(link_names)
    if not links or len(links) != len(link_names) or any(not value for value in link_names):
        raise ValueError("empty, unnamed, or duplicate links")
    visual_refs: list[str] = []
    collision_refs: list[str] = []
    valid_inertial = 0
    placeholder_inertial = 0
    for link in link_nodes:
        visual_refs.extend(
            mesh.attrib["filename"]
            for mesh in link.findall("visual/geometry/mesh")
            if mesh.attrib.get("filename")
        )
        collision_refs.extend(
            mesh.attrib["filename"]
            for mesh in link.findall("collision/geometry/mesh")
            if mesh.attrib.get("filename")
        )
        inertial = link.find("inertial")
        try:
            mass = float(inertial.find("mass").attrib["value"])  # type: ignore[union-attr]
            tensor = inertial.find("inertia")  # type: ignore[union-attr]
            diagonal = [float(tensor.attrib[key]) for key in ("ixx", "iyy", "izz")]  # type: ignore[union-attr]
            okay = mass > 0 and math.isfinite(mass) and all(
                value > 0 and math.isfinite(value) for value in diagonal
            )
            valid_inertial += int(okay)
            placeholder_inertial += int(
                okay
                and mass == 1.0
                and diagonal == [1.0, 1.0, 1.0]
                and all(
                    float(tensor.attrib.get(key, "nan")) == 0.0  # type: ignore[union-attr]
                    for key in ("ixy", "ixz", "iyz")
                )
            )
        except (AttributeError, KeyError, TypeError, ValueError):
            pass

    joints: list[dict[str, Any]] = []
    indegree: Counter[str] = Counter()
    adjacency: dict[str, list[str]] = defaultdict(list)
    for index, node in enumerate(root.findall("joint")):
        parent_node = node.find("parent")
        child_node = node.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        joint_type = node.attrib.get("type", "")
        axis_node = node.find("axis")
        axis = parse_vector(axis_node.attrib.get("xyz")) if axis_node is not None else None
        origin_node = node.find("origin")
        origin_xyz = parse_vector(
            origin_node.attrib.get("xyz") if origin_node is not None else None,
            default=[0.0, 0.0, 0.0],
        )
        origin_rpy = parse_vector(
            origin_node.attrib.get("rpy") if origin_node is not None else None,
            default=[0.0, 0.0, 0.0],
        )
        limit_node = node.find("limit")
        lower: float | None = None
        upper: float | None = None
        if limit_node is not None:
            try:
                lower = float(limit_node.attrib["lower"])
                upper = float(limit_node.attrib["upper"])
            except (KeyError, TypeError, ValueError):
                lower = upper = None
        bounded_expected = joint_type in {"revolute", "prismatic"}
        bounded = bool(
            bounded_expected
            and lower is not None
            and upper is not None
            and math.isfinite(lower)
            and math.isfinite(upper)
            and lower <= upper
        )
        zero_width = bool(bounded and abs(float(upper) - float(lower)) <= ZERO_WIDTH_TOLERANCE)
        functional_motion = bool(
            joint_type == "continuous"
            or (bounded and not zero_width)
        )
        joints.append(
            {
                "index": index,
                "name": node.attrib.get("name", ""),
                "type": joint_type,
                "parent": parent,
                "child": child,
                "endpoints_valid": parent in links and child in links,
                "axis": axis,
                "axis_valid": joint_type not in FUNCTIONAL_TYPES or norm(axis) > 1e-9,
                "origin_xyz": origin_xyz,
                "origin_rpy": origin_rpy,
                "origin_valid": origin_xyz is not None and origin_rpy is not None,
                "bounded_expected": bounded_expected,
                "bounded": bounded,
                "lower": lower,
                "upper": upper,
                "zero_width": zero_width,
                "functional_motion": functional_motion,
                "generic_revolute": bool(
                    joint_type == "revolute"
                    and bounded
                    and math.degrees(float(upper) - float(lower)) >= GENERIC_REVOLUTE_DEG
                ),
            }
        )
        if parent in links and child in links:
            indegree[child] += 1
            adjacency[parent].append(child)

    roots = [name for name in link_names if indegree[name] == 0]
    reached: set[str] = set()
    queue = deque(roots)
    while queue:
        current = queue.popleft()
        if current in reached:
            continue
        reached.add(current)
        queue.extend(adjacency[current])
    valid_tree = bool(
        len(roots) == 1
        and len(reached) == len(links)
        and len(joints) == len(links) - 1
        and all(count <= 1 for count in indegree.values())
        and all(row["endpoints_valid"] for row in joints)
    )
    return {
        "link_count": len(link_nodes),
        "joint_count": len(joints),
        "movable_joints": [row for row in joints if row["type"] != "fixed"],
        "fixed_joint_count": sum(row["type"] == "fixed" for row in joints),
        "valid_tree": valid_tree,
        "root_count": len(roots),
        "visual_mesh_references": visual_refs,
        "collision_mesh_references": collision_refs,
        "collision_element_count": len(root.findall(".//collision")),
        "valid_inertial_link_count": valid_inertial,
        "placeholder_inertial_link_count": placeholder_inertial,
    }


def resolve_mesh(urdf: Path, filename: str) -> Path:
    return (urdf.parent / filename).resolve(strict=False)


def annotation_audit(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    required = {"object_name", "category", "dimension", "parts", "group_info"}
    parts = payload.get("parts")
    group = payload.get("group_info")
    if not required.issubset(payload) or not isinstance(parts, list) or not isinstance(group, dict):
        raise ValueError("unexpected finaljson schema")
    part_required = {
        "label", "name", "material", "density", "Young's Modulus (GPa)",
        "Poisson's Ratio", "priority_rank", "Basic_description",
        "Functional_description", "Movement_description", "obj",
    }
    part_schema_complete = all(
        isinstance(part, dict)
        and part_required.issubset(part)
        and isinstance(part["obj"], list)
        for part in parts
    )
    group_schema_complete = "0" in group and all(
        key == "0"
        or (
            isinstance(value, list)
            and len(value) == 4
            and isinstance(value[0], list)
            and isinstance(value[1], str)
            and isinstance(value[2], list)
            and len(value[2]) == 8
            and value[3] in {"A", "B", "C"}
        )
        for key, value in group.items()
    )
    object_refs = [str(obj) for part in parts for obj in part.get("obj", [])]
    return {
        "category": str(payload.get("category", "")),
        "object_name": str(payload.get("object_name", "")),
        "part_count": len(parts),
        "part_schema_complete": part_schema_complete,
        "group_schema_complete": group_schema_complete,
        "group_movable_count": max(0, len(group) - 1),
        "group_type_counts": dict(sorted(Counter(value[3] for key, value in group.items() if key != "0").items())),
        "object_reference_count": len(object_refs),
        "unique_object_reference_count": len(set(object_refs)),
        "duplicate_object_reference_count": len(object_refs) - len(set(object_refs)),
        "payload": payload,
    }


def rank_hash(category: str, dataset_id: str) -> str:
    raw = f"{SELECTION_SALT}\n{category}\n{dataset_id}".encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def freeze_cohort(records: list[dict[str, Any]]) -> dict[str, Any]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        category = record["partnet_category"]
        grouped[category].append(record)
    rows = []
    for category in sorted(grouped):
        ranked = sorted(
            grouped[category],
            key=lambda row: (rank_hash(category, row["dataset_id"]), int(row["dataset_id"])),
        )
        winner = ranked[0]
        functional_count = int(winner["functional_motion_joint_count"])
        rows.append(
            {
                "dataset_id": winner["dataset_id"],
                "partnet_category": category,
                "selection_hash": rank_hash(category, winner["dataset_id"]),
                "candidate_count": len(ranked),
                "urdf_sha256": winner["urdf_sha256"],
                "archive_urdf_sha256": winner["archive_urdf_sha256"],
                "urdf_archive_byte_exact": winner["urdf_archive_byte_exact"],
                "declared_movable_joint_count": winner["movable_joint_count"],
                "functional_motion_joint_count": functional_count,
                "zero_width_joint_count": winner["zero_width_joint_count"],
                "unsupported_motion_joint_count": winner["unsupported_motion_joint_count"],
                "single_state_expected": functional_count * SINGLE_STATES,
                "sobol_state_expected": SOBOL_STATES if functional_count > 1 else 0,
            }
        )
    return {
        "protocol_id": "nano3d_table6_physx_mobility_category_cohort_v1",
        "selection_inputs": ["PartNet meta.json model_cat", "dataset_id"],
        "selection_salt": SELECTION_SALT,
        "selection_rule": "per exact PartNet category, minimum SHA256(salt\\ncategory\\ndataset_id), then numeric ID",
        "selection_is_outcome_independent": True,
        "failure_policy": "retain every selected ID; no outcome replacement",
        "category_count": len(rows),
        "rows": rows,
        "rows_sha256": canonical_sha256(rows),
    }


def _float_list(values: Any) -> list[float] | None:
    if not isinstance(values, list) or len(values) != 3:
        return None
    try:
        parsed = [float(value) for value in values]
    except (TypeError, ValueError):
        return None
    return parsed if all(math.isfinite(value) for value in parsed) else None


def _close_vec(left: list[float] | None, right: list[float] | None, tolerance: float = 1e-6) -> bool:
    return bool(
        left is not None
        and right is not None
        and len(left) == len(right)
        and max(abs(a - b) for a, b in zip(left, right)) <= tolerance
    )


def _normalized_line_match(left: list[float] | None, right: list[float] | None) -> bool:
    left_norm = norm(left)
    right_norm = norm(right)
    if left_norm <= 1e-9 or right_norm <= 1e-9 or left is None or right is None:
        return False
    dot = sum(a * b for a, b in zip(left, right)) / (left_norm * right_norm)
    return abs(dot) >= 1.0 - 1e-6


def group_urdf_fidelity(annotation: dict[str, Any], urdf: dict[str, Any]) -> dict[str, Any]:
    group = annotation["payload"]["group_info"]
    movable = urdf["movable_joints"]
    by_name = {row["name"]: row for row in movable}
    matched = []
    for key, value in sorted(group.items(), key=lambda item: int(item[0])):
        if key == "0":
            continue
        child_ids, parent_id, vector, code = value
        if not child_ids:
            continue
        child = str(child_ids[0])
        if code == "B":
            expected_name = f"joint_prismatic_l_{parent_id}_abstract_{parent_id}_{child}"
            expected_type = "prismatic"
        elif code == "C":
            expected_name = f"joint_revolute_l_{parent_id}_abstract_{parent_id}_{child}"
            expected_type = "revolute"
        else:
            expected_name = f"joint_free_l_{parent_id}_abstract_{parent_id}_{child}"
            expected_type = "floating"
        row = by_name.get(expected_name)
        axis = _float_list(vector[:3])
        origin = _float_list(vector[3:6])
        lower = float(vector[6])
        upper = float(vector[7])
        matched.append(
            {
                "group_key": key,
                "code": code,
                "expected_joint_name": expected_name,
                "name_match": row is not None,
                "type_match": bool(row and row["type"] == expected_type),
                "axis_match": bool(
                    row
                    and (
                        code == "A"
                        or _close_vec(row["axis"], axis)
                    )
                ),
                "origin_match": bool(
                    row
                    and (
                        code in {"A", "B"}
                        and _close_vec(row["origin_xyz"], [0.0, 0.0, 0.0])
                        or code == "C"
                        and _close_vec(row["origin_xyz"], origin)
                    )
                ),
                "range_match": bool(
                    row
                    and (
                        code == "A"
                        or (
                            row["lower"] is not None
                            and row["upper"] is not None
                            and abs(float(row["lower"]) - lower) <= 1e-6
                            and abs(float(row["upper"]) - upper) <= 1e-6
                        )
                    )
                ),
            }
        )
    all_fields = [
        row
        for row in matched
        if all(row[field] for field in ("name_match", "type_match", "axis_match", "origin_match", "range_match"))
    ]
    return {
        "group_movable_count": len(matched),
        "urdf_movable_count": len(movable),
        "joint_name_match_count": sum(row["name_match"] for row in matched),
        "joint_type_match_count": sum(row["type_match"] for row in matched),
        "axis_match_count": sum(row["axis_match"] for row in matched),
        "origin_match_count": sum(row["origin_match"] for row in matched),
        "range_match_count": sum(row["range_match"] for row in matched),
        "all_fields_match_count": len(all_fields),
        "matched": matched,
    }


def partnet_urdf_fidelity(physx_urdf: dict[str, Any], partnet_urdf: dict[str, Any]) -> dict[str, Any]:
    def mesh_key(row: dict[str, Any], package: dict[str, Any]) -> tuple[str, ...]:
        # The generated PhysX URDF inserts an abstract link between the movable
        # joint and visual link.  Resolve the fixed child when needed.
        child = row["child"]
        root = package["root"]
        meshes = [
            Path(mesh.attrib["filename"]).name
            for link in root.findall("link")
            if link.attrib.get("name") == child
            for mesh in link.findall("visual/geometry/mesh")
            if mesh.attrib.get("filename")
        ]
        if meshes:
            return tuple(sorted(meshes))
        fixed_children = [
            node.find("child").attrib.get("link", "")
            for node in root.findall("joint")
            if node.attrib.get("type") == "fixed"
            and node.find("parent") is not None
            and node.find("parent").attrib.get("link") == child
            and node.find("child") is not None
        ]
        return tuple(
            sorted(
                Path(mesh.attrib["filename"]).name
                for link in root.findall("link")
                if link.attrib.get("name") in fixed_children
                for mesh in link.findall("visual/geometry/mesh")
                if mesh.attrib.get("filename")
            )
        )

    pm_by_mesh: dict[tuple[str, ...], list[dict[str, Any]]] = defaultdict(list)
    px_by_mesh: dict[tuple[str, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in partnet_urdf["movable_joints"]:
        pm_by_mesh[mesh_key(row, partnet_urdf)].append(row)
    for row in physx_urdf["movable_joints"]:
        px_by_mesh[mesh_key(row, physx_urdf)].append(row)
    matches = []
    for key, pm_rows in pm_by_mesh.items():
        px_rows = px_by_mesh.get(key, [])
        if not key or len(pm_rows) != 1 or len(px_rows) != 1:
            continue
        pm_row = pm_rows[0]
        px_row = px_rows[0]
        matches.append(
            {
                "mesh_key": list(key),
                "type_exact": pm_row["type"] == px_row["type"],
                "type_rotational_class": (
                    pm_row["type"] == px_row["type"]
                    or {pm_row["type"], px_row["type"]} <= {"continuous", "revolute"}
                ),
                "axis_line_match": _normalized_line_match(pm_row["axis"], px_row["axis"]),
                "origin_exact": _close_vec(pm_row["origin_xyz"], px_row["origin_xyz"]),
                "limit_exact": (
                    pm_row["lower"] == px_row["lower"]
                    and pm_row["upper"] == px_row["upper"]
                ),
            }
        )
    return {
        "partnet_movable_count": len(partnet_urdf["movable_joints"]),
        "physx_movable_count": len(physx_urdf["movable_joints"]),
        "unique_mesh_matched_count": len(matches),
        "type_exact_count": sum(row["type_exact"] for row in matches),
        "rotational_class_count": sum(row["type_rotational_class"] for row in matches),
        "axis_line_count": sum(row["axis_line_match"] for row in matches),
        "origin_exact_count": sum(row["origin_exact"] for row in matches),
        "limit_exact_count": sum(row["limit_exact"] for row in matches),
        "matches": matches,
    }


def parse_with_root(path: Path) -> dict[str, Any]:
    parsed = parse_urdf(path)
    parsed["root"] = ET.parse(path).getroot()
    return parsed


def static_audit(physx_root: Path, partnet_root: Path, archive: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    with zipfile.ZipFile(archive) as zipped:
        infos = zipped.infolist()
        urdf_infos = [
            info
            for info in infos
            if info.filename.startswith("PhysX_mobility/urdf/")
            and Path(info.filename).suffix == ".urdf"
            and Path(info.filename).stem.isdigit()
            and not info.is_dir()
        ]
        json_infos = [
            info
            for info in infos
            if info.filename.startswith("PhysX_mobility/finaljson/")
            and Path(info.filename).suffix == ".json"
            and Path(info.filename).stem.isdigit()
            and not info.is_dir()
        ]
        zip_urdf_ids = {
            Path(info.filename).stem for info in urdf_infos
        }
        zip_json_ids = {
            Path(info.filename).stem for info in json_infos
        }
        partseg_prefix = "PhysX_mobility/partseg/"
        zip_partseg_members = [
            info.filename[len(partseg_prefix):]
            for info in infos
            if info.filename.startswith(partseg_prefix) and not info.is_dir()
        ]
        zip_partseg_inventory = set(zip_partseg_members)
        zip_partseg_ids = {
            name.split("/", 1)[0]
            for name in zip_partseg_inventory
            if "/" in name
        }
        zip_urdf_hashes = {
            Path(info.filename).stem: hashlib.sha256(zipped.read(info)).hexdigest()
            for info in urdf_infos
        }
        zip_json_hashes = {
            Path(info.filename).stem: hashlib.sha256(zipped.read(info)).hexdigest()
            for info in json_infos
        }
        zip_urdf_sizes = {Path(info.filename).stem: info.file_size for info in urdf_infos}
        zip_json_sizes = {Path(info.filename).stem: info.file_size for info in json_infos}
    official_ids = sorted(zip_urdf_ids & zip_json_ids & zip_partseg_ids, key=int)
    if (
        len(urdf_infos) != 2024
        or len(json_infos) != 2024
        or len(official_ids) != 2024
        or zip_urdf_ids != zip_json_ids
        or zip_urdf_ids != zip_partseg_ids
    ):
        raise RuntimeError("archive official identity inventory is not exactly 2024 paired IDs")

    local_partseg_root = physx_root / "partseg"
    local_partseg_inventory: set[str] = set()
    for directory, _, filenames in os.walk(local_partseg_root):
        relative_directory = Path(directory).relative_to(local_partseg_root)
        for filename in filenames:
            local_partseg_inventory.add((relative_directory / filename).as_posix())
    partseg_missing = zip_partseg_inventory - local_partseg_inventory
    partseg_extra = local_partseg_inventory - zip_partseg_inventory

    local_extra_urdfs = sorted(
        path.name for path in (physx_root / "urdf").glob("*.urdf") if not path.stem.isdigit()
    )
    records: list[dict[str, Any]] = []
    for index, dataset_id in enumerate(official_ids, 1):
        urdf_path = physx_root / "urdf" / f"{dataset_id}.urdf"
        annotation_path = physx_root / "finaljson" / f"{dataset_id}.json"
        partnet_dir = partnet_root / dataset_id
        record: dict[str, Any] = {
            "dataset_id": dataset_id,
            "official_archive_member": True,
            "static_status": "PASS",
            "issues": [],
        }
        try:
            local_urdf_sha256 = sha256_file(urdf_path)
            local_annotation_sha256 = sha256_file(annotation_path)
            urdf_archive_byte_exact = bool(
                local_urdf_sha256 == zip_urdf_hashes[dataset_id]
                and urdf_path.stat().st_size == zip_urdf_sizes[dataset_id]
            )
            annotation_archive_byte_exact = bool(
                local_annotation_sha256 == zip_json_hashes[dataset_id]
                and annotation_path.stat().st_size == zip_json_sizes[dataset_id]
            )
            annotation = annotation_audit(annotation_path)
            physx_urdf = parse_with_root(urdf_path)
            pm_urdf = parse_with_root(partnet_dir / "mobility.urdf")
            meta = json.loads((partnet_dir / "meta.json").read_text(encoding="utf-8"))
            pm_v2 = json.loads((partnet_dir / "mobility_v2.json").read_text(encoding="utf-8"))
            mesh_paths = [resolve_mesh(urdf_path, filename) for filename in physx_urdf["visual_mesh_references"]]
            missing_meshes = [str(path) for path in mesh_paths if not path.is_file()]
            group_fidelity = group_urdf_fidelity(annotation, physx_urdf)
            pair_fidelity = partnet_urdf_fidelity(physx_urdf, pm_urdf)
            movable = physx_urdf["movable_joints"]
            record.update(
                {
                    "partnet_category": str(meta.get("model_cat", "")),
                    "annotation_category": annotation["category"],
                    "annotation_part_count": annotation["part_count"],
                    "annotation_part_schema_complete": annotation["part_schema_complete"],
                    "annotation_group_schema_complete": annotation["group_schema_complete"],
                    "annotation_group_movable_count": annotation["group_movable_count"],
                    "annotation_group_type_counts": annotation["group_type_counts"],
                    "annotation_duplicate_object_reference_count": annotation["duplicate_object_reference_count"],
                    "partnet_mobility_v2_entry_count": len(pm_v2),
                    "link_count": physx_urdf["link_count"],
                    "joint_count": physx_urdf["joint_count"],
                    "fixed_joint_count": physx_urdf["fixed_joint_count"],
                    "movable_joint_count": len(movable),
                    "functional_motion_joint_count": sum(row["functional_motion"] for row in movable),
                    "zero_width_joint_count": sum(row["zero_width"] for row in movable),
                    "unsupported_motion_joint_count": sum(
                        row["type"] not in FUNCTIONAL_TYPES for row in movable
                    ),
                    "joint_type_counts": dict(sorted(Counter(row["type"] for row in movable).items())),
                    "valid_tree": physx_urdf["valid_tree"],
                    "root_count": physx_urdf["root_count"],
                    "parent_child_structurally_valid_count": sum(row["endpoints_valid"] for row in movable),
                    "axis_metadata_valid_count": sum(row["axis_valid"] for row in movable),
                    "origin_metadata_valid_count": sum(row["origin_valid"] for row in movable),
                    "bounded_expected_count": sum(row["bounded_expected"] for row in movable),
                    "bounded_limit_valid_count": sum(row["bounded"] for row in movable),
                    "generic_revolute_count": sum(row["generic_revolute"] for row in movable),
                    "revolute_count": sum(row["type"] == "revolute" for row in movable),
                    "continuous_count": sum(row["type"] == "continuous" for row in movable),
                    "visual_mesh_reference_count": len(physx_urdf["visual_mesh_references"]),
                    "unique_visual_mesh_reference_count": len(set(physx_urdf["visual_mesh_references"])),
                    "missing_visual_mesh_reference_count": len(missing_meshes),
                    "missing_visual_mesh_references": missing_meshes[:20],
                    "collision_element_count": physx_urdf["collision_element_count"],
                    "collision_mesh_reference_count": len(physx_urdf["collision_mesh_references"]),
                    "valid_inertial_link_count": physx_urdf["valid_inertial_link_count"],
                    "placeholder_inertial_link_count": physx_urdf["placeholder_inertial_link_count"],
                    "group_urdf_export_fidelity": {k: v for k, v in group_fidelity.items() if k != "matched"},
                    "partnet_physx_export_fidelity": {k: v for k, v in pair_fidelity.items() if k != "matches"},
                    "urdf_sha256": local_urdf_sha256,
                    "archive_urdf_sha256": zip_urdf_hashes[dataset_id],
                    "urdf_archive_byte_exact": urdf_archive_byte_exact,
                    "annotation_sha256": local_annotation_sha256,
                    "archive_annotation_sha256": zip_json_hashes[dataset_id],
                    "annotation_archive_byte_exact": annotation_archive_byte_exact,
                }
            )
            if (
                missing_meshes
                or not annotation["part_schema_complete"]
                or not annotation["group_schema_complete"]
                or not urdf_archive_byte_exact
                or not annotation_archive_byte_exact
            ):
                record["static_status"] = "FAIL"
                record["issues"].append("archive binding, package schema, or mesh completeness failure")
        except Exception as exc:  # noqa: BLE001
            record["static_status"] = "FAIL"
            record["issues"].append(f"{type(exc).__name__}: {exc}")
        records.append(record)
        if index % 250 == 0 or index == len(official_ids):
            print(f"[static {index}/{len(official_ids)}]", flush=True)

    summary = {
        "protocol_id": "nano3d_table6_physx_mobility_static_v2",
        "official_archive_asset_count": len(official_ids),
        "official_archive_ids_sha256": canonical_sha256(official_ids),
        "archive_urdf_id_count": len(zip_urdf_ids),
        "archive_finaljson_id_count": len(zip_json_ids),
        "archive_partseg_id_count": len(zip_partseg_ids),
        "archive_urdf_member_file_count": len(urdf_infos),
        "archive_finaljson_member_file_count": len(json_infos),
        "archive_partseg_member_file_count": len(zip_partseg_members),
        "archive_partseg_duplicate_member_count": len(zip_partseg_members) - len(zip_partseg_inventory),
        "local_partseg_file_count": len(local_partseg_inventory),
        "partseg_inventory_archive_sha256": string_inventory_sha256(zip_partseg_inventory),
        "partseg_inventory_local_sha256": string_inventory_sha256(local_partseg_inventory),
        "partseg_inventory_missing_count": len(partseg_missing),
        "partseg_inventory_extra_count": len(partseg_extra),
        "partseg_inventory_missing_sample": sorted(partseg_missing)[:20],
        "partseg_inventory_extra_sample": sorted(partseg_extra)[:20],
        "partseg_local_extra_members_excluded": sorted(partseg_extra),
        "partseg_inventory_exact": not partseg_missing and not partseg_extra,
        "partseg_archive_subset_complete": not partseg_missing,
        "partseg_archive_subset_local_sha256": string_inventory_sha256(
            local_partseg_inventory & zip_partseg_inventory
        ),
        "partseg_inventory_policy": (
            "freeze every official archive member; require zero missing; catalogue and exclude "
            "all local-only additions"
        ),
        "urdf_archive_byte_exact_assets": sum(row.get("urdf_archive_byte_exact", False) for row in records),
        "urdf_archive_byte_mismatch_assets": sum(not row.get("urdf_archive_byte_exact", False) for row in records),
        "annotation_archive_byte_exact_assets": sum(row.get("annotation_archive_byte_exact", False) for row in records),
        "annotation_archive_byte_mismatch_assets": sum(not row.get("annotation_archive_byte_exact", False) for row in records),
        "local_nonofficial_urdf_variant_count": len(local_extra_urdfs),
        "local_nonofficial_urdf_variants_excluded": local_extra_urdfs,
        "static_pass_assets": sum(row["static_status"] == "PASS" for row in records),
        "partnet_same_id_assets": sum((partnet_root / row["dataset_id"]).is_dir() for row in records),
        "partnet_category_count": len({row.get("partnet_category") for row in records if row.get("partnet_category")}),
        "annotation_schema_complete_assets": sum(
            row.get("annotation_part_schema_complete") and row.get("annotation_group_schema_complete")
            for row in records
        ),
        "valid_tree_assets": sum(row.get("valid_tree", False) for row in records),
        "link_total": sum(row.get("link_count", 0) for row in records),
        "joint_total": sum(row.get("joint_count", 0) for row in records),
        "movable_joint_total": sum(row.get("movable_joint_count", 0) for row in records),
        "functional_motion_joint_total": sum(row.get("functional_motion_joint_count", 0) for row in records),
        "zero_width_joint_total": sum(row.get("zero_width_joint_count", 0) for row in records),
        "unsupported_motion_joint_total": sum(row.get("unsupported_motion_joint_count", 0) for row in records),
        "joint_type_counts": dict(sorted(sum((Counter(row.get("joint_type_counts", {})) for row in records), Counter()).items())),
        "parent_child_structurally_valid_total": sum(row.get("parent_child_structurally_valid_count", 0) for row in records),
        "axis_metadata_valid_total": sum(row.get("axis_metadata_valid_count", 0) for row in records),
        "origin_metadata_valid_total": sum(row.get("origin_metadata_valid_count", 0) for row in records),
        "bounded_expected_total": sum(row.get("bounded_expected_count", 0) for row in records),
        "bounded_limit_valid_total": sum(row.get("bounded_limit_valid_count", 0) for row in records),
        "generic_revolute_total": sum(row.get("generic_revolute_count", 0) for row in records),
        "revolute_total": sum(row.get("revolute_count", 0) for row in records),
        "continuous_total": sum(row.get("continuous_count", 0) for row in records),
        "visual_mesh_reference_total": sum(row.get("visual_mesh_reference_count", 0) for row in records),
        "missing_visual_mesh_reference_total": sum(row.get("missing_visual_mesh_reference_count", 0) for row in records),
        "collision_element_total": sum(row.get("collision_element_count", 0) for row in records),
        "collision_mesh_reference_total": sum(row.get("collision_mesh_reference_count", 0) for row in records),
        "valid_inertial_link_total": sum(row.get("valid_inertial_link_count", 0) for row in records),
        "placeholder_inertial_link_total": sum(row.get("placeholder_inertial_link_count", 0) for row in records),
        "annotation_group_movable_total": sum(row.get("annotation_group_movable_count", 0) for row in records),
        "group_urdf_joint_name_match_total": sum(row.get("group_urdf_export_fidelity", {}).get("joint_name_match_count", 0) for row in records),
        "group_urdf_all_fields_match_total": sum(row.get("group_urdf_export_fidelity", {}).get("all_fields_match_count", 0) for row in records),
        "partnet_unique_mesh_matched_joint_total": sum(row.get("partnet_physx_export_fidelity", {}).get("unique_mesh_matched_count", 0) for row in records),
        "partnet_unique_mesh_type_exact_total": sum(row.get("partnet_physx_export_fidelity", {}).get("type_exact_count", 0) for row in records),
        "partnet_unique_mesh_rotational_class_total": sum(row.get("partnet_physx_export_fidelity", {}).get("rotational_class_count", 0) for row in records),
        "partnet_unique_mesh_axis_line_total": sum(row.get("partnet_physx_export_fidelity", {}).get("axis_line_count", 0) for row in records),
        "lineage_interpretation": (
            "same-ID derivative export fidelity only; PhysX-Mobility card says based on PartNet-Mobility, "
            "so comparisons against PartNet are not independent accuracy"
        ),
    }
    return records, summary


def joint_samples(row: dict[str, Any], np: Any) -> list[float]:
    if row["bounded"]:
        return [float(value) for value in np.linspace(row["lower"], row["upper"], SINGLE_STATES)]
    return [float(value) for value in np.linspace(-math.pi, math.pi, SINGLE_STATES)]


def functional_child(urdf_path: str, output_path: str) -> None:
    result: dict[str, Any] = {
        "status": "FAIL",
        "pybullet_load_success": False,
        "joint_mapping_complete": False,
        "movable_name_set_exact": False,
        "bullet_traversal_order_matches_xml": False,
        "movable_type_exact": False,
        "movable_axis_exact": False,
        "movable_limit_exact": False,
        "bullet_joint_count": 0,
        "bullet_movable_joint_count": 0,
        "motors_disabled_count": 0,
        "declared_movable_joint_count": 0,
        "functional_motion_joint_count": 0,
        "zero_width_joint_count": 0,
        "unsupported_motion_joint_count": 0,
        "single_state_expected": 0,
        "single_state_executed": 0,
        "sobol_state_expected": 0,
        "sobol_state_executed": 0,
        "collision_detection_calls": 0,
        "max_reset_readback_error": None,
        "issues": [],
    }
    try:
        import numpy as np
        from scipy.stats import qmc
        import pybullet as pb

        path = Path(urdf_path)
        parsed = parse_urdf(path)
        movable = parsed["movable_joints"]
        functional = [row for row in movable if row["functional_motion"]]
        result.update(
            {
                "declared_movable_joint_count": len(movable),
                "functional_motion_joint_count": len(functional),
                "zero_width_joint_count": sum(row["zero_width"] for row in movable),
                "unsupported_motion_joint_count": sum(row["type"] not in FUNCTIONAL_TYPES for row in movable),
                "single_state_expected": len(functional) * SINGLE_STATES,
                "sobol_state_expected": SOBOL_STATES if len(functional) > 1 else 0,
            }
        )
        client = pb.connect(pb.DIRECT)
        body: int | None = None
        try:
            flags = (
                pb.URDF_USE_INERTIA_FROM_FILE
                | pb.URDF_USE_SELF_COLLISION
                | pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
                | pb.URDF_IGNORE_VISUAL_SHAPES
                | pb.URDF_MERGE_FIXED_LINKS
            )
            body = pb.loadURDF(
                str(path), useFixedBase=True, flags=flags, physicsClientId=client
            )
            result["pybullet_load_success"] = True
            bullet_joint_count = pb.getNumJoints(body, physicsClientId=client)
            bullet_movable: list[dict[str, Any]] = []
            for index in range(bullet_joint_count):
                info = pb.getJointInfo(body, index, physicsClientId=client)
                name = info[1].decode("utf-8") if isinstance(info[1], bytes) else str(info[1])
                if int(info[2]) != pb.JOINT_FIXED:
                    bullet_movable.append(
                        {
                            "index": index,
                            "name": name,
                            "type": int(info[2]),
                            "lower": float(info[8]),
                            "upper": float(info[9]),
                            "axis": [float(value) for value in info[13]],
                        }
                    )
            result["bullet_joint_count"] = bullet_joint_count
            result["bullet_movable_joint_count"] = len(bullet_movable)
            xml_names = [row["name"] for row in movable]
            bullet_names = [row["name"] for row in bullet_movable]
            result["bullet_traversal_order_matches_xml"] = bullet_names == xml_names
            result["movable_name_set_exact"] = bool(
                len(xml_names) == len(set(xml_names))
                and len(bullet_names) == len(set(bullet_names))
                and set(bullet_names) == set(xml_names)
            )
            bullet_by_name = {row["name"]: row for row in bullet_movable}
            ordered_bullet = (
                [bullet_by_name[name] for name in xml_names]
                if result["movable_name_set_exact"]
                else []
            )
            type_map = {
                "revolute": pb.JOINT_REVOLUTE,
                "continuous": pb.JOINT_REVOLUTE,
                "prismatic": pb.JOINT_PRISMATIC,
                "planar": pb.JOINT_PLANAR,
                "floating": pb.JOINT_SPHERICAL,
            }
            result["movable_type_exact"] = len(ordered_bullet) == len(movable) and all(
                bullet_row["type"] == type_map.get(xml_row["type"], -1)
                for xml_row, bullet_row in zip(movable, ordered_bullet)
            )
            result["movable_axis_exact"] = len(ordered_bullet) == len(movable) and all(
                xml_row["type"] in {"floating"}
                or _close_vec(xml_row["axis"], bullet_row["axis"], tolerance=1e-9)
                for xml_row, bullet_row in zip(movable, ordered_bullet)
            )
            result["movable_limit_exact"] = len(ordered_bullet) == len(movable) and all(
                (
                    xml_row["type"] == "continuous"
                    and bullet_row["lower"] > bullet_row["upper"]
                )
                or (
                    xml_row["bounded"]
                    and abs(float(xml_row["lower"]) - bullet_row["lower"]) <= 1e-9
                    and abs(float(xml_row["upper"]) - bullet_row["upper"]) <= 1e-9
                )
                or xml_row["type"] in {"floating", "planar"}
                for xml_row, bullet_row in zip(movable, ordered_bullet)
            )
            kinematic_checks = {
                "name_set": result["movable_name_set_exact"],
                "type": result["movable_type_exact"],
                "axis": result["movable_axis_exact"],
                "limit": result["movable_limit_exact"],
            }
            result["movable_kinematic_checks"] = kinematic_checks
            if not all(kinematic_checks.values()):
                raise RuntimeError(
                    "URDF_MERGE_FIXED_LINKS changed XML/Bullet movable joint semantics: "
                    f"{kinematic_checks}; xml_names={xml_names[:20]}; bullet_names={bullet_names[:20]}"
                )
            result["joint_mapping_complete"] = True
            indices = [row["index"] for row in ordered_bullet]
            for index in indices:
                pb.setJointMotorControl2(
                    body,
                    index,
                    controlMode=pb.VELOCITY_CONTROL,
                    targetVelocity=0.0,
                    force=0.0,
                    physicsClientId=client,
                )
            result["motors_disabled_count"] = len(indices)
            defaults = [
                float((row["lower"] + row["upper"]) / 2.0) if row["bounded"] else 0.0
                for row in movable
            ]
            positions = [index for index, row in enumerate(movable) if row["functional_motion"]]
            max_error = 0.0

            def reset_readback(values: list[float]) -> None:
                nonlocal max_error
                for joint_index, value in zip(indices, values):
                    pb.resetJointState(body, joint_index, value, physicsClientId=client)
                pb.performCollisionDetection(physicsClientId=client)
                result["collision_detection_calls"] += 1
                observed = [
                    float(pb.getJointState(body, joint_index, physicsClientId=client)[0])
                    for joint_index in indices
                ]
                error = max([abs(actual - expected) for actual, expected in zip(observed, values)] or [0.0])
                max_error = max(max_error, error)
                if error > RESET_TOLERANCE:
                    raise RuntimeError(f"reset/readback error {error} exceeds {RESET_TOLERANCE}")

            for position in positions:
                for value in joint_samples(movable[position], np):
                    values = list(defaults)
                    values[position] = value
                    reset_readback(values)
                    result["single_state_executed"] += 1
            if len(positions) > 1:
                unit = qmc.Sobol(d=len(positions), scramble=False).random_base2(m=6)
                for sample in unit:
                    values = list(defaults)
                    for position, fraction in zip(positions, sample):
                        row = movable[position]
                        values[position] = (
                            float(row["lower"] + fraction * (row["upper"] - row["lower"]))
                            if row["bounded"]
                            else float(-math.pi + fraction * 2.0 * math.pi)
                        )
                    reset_readback(values)
                    result["sobol_state_executed"] += 1
            result["max_reset_readback_error"] = max_error
            expected = result["single_state_expected"] + result["sobol_state_expected"]
            executed = result["single_state_executed"] + result["sobol_state_executed"]
            if expected != executed:
                raise RuntimeError(f"state denominator mismatch: expected={expected}, executed={executed}")
            result["status"] = "PASS"
        finally:
            if body is not None:
                pb.removeBody(body, physicsClientId=client)
            pb.disconnect(client)
    except BaseException as exc:  # noqa: BLE001
        result["issues"].append(f"{type(exc).__name__}: {exc}")
    Path(output_path).write_text(
        json.dumps(result, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def self_collision_flag_smoke(output: Path) -> dict[str, Any]:
    """Verify that the exact self-collision flag composition is active.

    The fixture has two overlapping sibling collision links.  This validates
    the PyBullet flags only and is never included in dataset collision scores.
    """
    import pybullet as pb

    fixture = output / "self_collision_flag_smoke.urdf"
    fixture.write_text(
        """<?xml version="1.0"?>
<robot name="self_collision_flag_smoke">
  <link name="base">
    <inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="sibling_a">
    <inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><origin xyz="0 0 0"/><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <link name="sibling_b">
    <inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><origin xyz="0 0 0"/><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <joint name="joint_a" type="revolute"><parent link="base"/><child link="sibling_a"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  <joint name="joint_b" type="revolute"><parent link="base"/><child link="sibling_b"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>
""",
        encoding="utf-8",
    )
    cases = {
        "none": 0,
        "exclude_parent_only": pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        "self_collision": pb.URDF_USE_SELF_COLLISION,
        "self_collision_exclude_parent": (
            pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
        ),
    }
    client = pb.connect(pb.DIRECT)
    contacts: dict[str, int] = {}
    try:
        for name, self_collision_flags in cases.items():
            body = pb.loadURDF(
                str(fixture),
                useFixedBase=True,
                flags=(
                    pb.URDF_USE_INERTIA_FROM_FILE
                    | pb.URDF_MERGE_FIXED_LINKS
                    | self_collision_flags
                ),
                physicsClientId=client,
            )
            pb.performCollisionDetection(physicsClientId=client)
            contacts[name] = len(
                pb.getContactPoints(bodyA=body, bodyB=body, physicsClientId=client)
            )
            pb.removeBody(body, physicsClientId=client)
    finally:
        pb.disconnect(client)
    checks = {
        "none_has_zero_contacts": contacts.get("none") == 0,
        "exclude_parent_only_has_zero_contacts": contacts.get("exclude_parent_only") == 0,
        "self_collision_detects_siblings": contacts.get("self_collision", 0) > 0,
        "self_collision_exclude_parent_detects_siblings": contacts.get(
            "self_collision_exclude_parent", 0
        ) > 0,
    }
    return {
        "passed": all(checks.values()),
        "fixture": fixture.name,
        "fixture_sha256": sha256_file(fixture),
        "pybullet_api_version": pb.getAPIVersion(),
        "contacts": contacts,
        "checks": checks,
        "interpretation": "protocol flag smoke only; excluded from official dataset metrics",
    }


def functional_audit(
    physx_root: Path, cohort: dict[str, Any], output: Path, timeout_seconds: int
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    child_dir = output / "functional_children"
    child_dir.mkdir(exist_ok=True)
    records = []
    context = mp.get_context("spawn")
    for index, selected in enumerate(cohort["rows"], 1):
        dataset_id = selected["dataset_id"]
        urdf_path = physx_root / "urdf" / f"{dataset_id}.urdf"
        child_output = child_dir / f"{dataset_id}.json"
        if child_output.exists():
            child_output.unlink()
        input_hash = sha256_file(urdf_path)
        process_exitcode: int | None = None
        timed_out = False
        started = time.monotonic()
        if input_hash != selected["urdf_sha256"] or not selected["urdf_archive_byte_exact"]:
            payload = {
                "status": "INPUT_HASH_MISMATCH",
                "issues": ["selected URDF no longer matches frozen local/archive hash"],
            }
        else:
            process = context.Process(
                target=functional_child,
                args=(str(urdf_path), str(child_output)),
            )
            process.start()
            process.join(timeout_seconds)
            timed_out = process.is_alive()
            if timed_out:
                process.terminate()
                process.join(10)
            process_exitcode = process.exitcode
            if timed_out:
                payload = {"status": "TIMEOUT", "issues": [f"child exceeded {timeout_seconds} seconds"]}
            elif not child_output.is_file():
                payload = {
                    "status": "NATIVE_ABORT" if process.exitcode else "MISSING_OUTPUT",
                    "issues": [f"child exitcode={process.exitcode}; output absent"],
                }
            else:
                payload = json.loads(child_output.read_text(encoding="utf-8"))
        elapsed = time.monotonic() - started
        expected_fields = (
            "declared_movable_joint_count",
            "functional_motion_joint_count",
            "zero_width_joint_count",
            "unsupported_motion_joint_count",
            "single_state_expected",
            "sobol_state_expected",
        )
        child_reported = {field: payload.get(field) for field in expected_fields}
        expectation_checks = {
            field: child_reported[field] == selected[field] for field in expected_fields
        }
        if payload.get("status") == "PASS" and not all(expectation_checks.values()):
            payload["status"] = "FAIL_EXPECTATION_MISMATCH"
            payload.setdefault("issues", []).append(
                "child-declared kinematics/state denominators differ from frozen static intent"
            )
        payload.update({field: selected[field] for field in expected_fields})
        if payload.get("status") == "PASS":
            failure_class = None
        elif payload.get("pybullet_load_success") and payload.get("joint_mapping_complete"):
            failure_class = "EVALUATOR_RESET_READBACK_FAILURE"
        elif payload.get("pybullet_load_success"):
            failure_class = "EVALUATOR_JOINT_MAPPING_FAILURE"
        else:
            failure_class = "EVALUATOR_LOAD_OR_PROCESS_FAILURE"
        payload.update(
            {
                "dataset_id": dataset_id,
                "partnet_category": selected["partnet_category"],
                "selection_hash": selected["selection_hash"],
                "frozen_urdf_sha256": selected["urdf_sha256"],
                "input_urdf_sha256": input_hash,
                "input_hash_matches_frozen": input_hash == selected["urdf_sha256"],
                "child_reported_expected_fields": child_reported,
                "child_expectation_checks": expectation_checks,
                "child_expectations_match": all(expectation_checks.values()),
                "failure_class": failure_class,
                "child_exitcode": process_exitcode,
                "elapsed_seconds": elapsed,
                "timed_out": timed_out,
            }
        )
        records.append(payload)
        print(f"[functional {index}/{len(cohort['rows'])}] {dataset_id}: {payload['status']}", flush=True)
    state_expected = sum(
        row["single_state_expected"] + row["sobol_state_expected"] for row in cohort["rows"]
    )
    state_executed = sum(row.get("single_state_executed", 0) + row.get("sobol_state_executed", 0) for row in records)
    summary = {
        "protocol_id": "nano3d_table6_physx_mobility_reset_readback_v3",
        "cohort_asset_count": len(records),
        "cohort_sha256": cohort["rows_sha256"],
        "selection_is_outcome_independent": True,
        "failure_policy": "selected failures retained; no replacement",
        "pybullet_load_success_assets": sum(row.get("pybullet_load_success", False) for row in records),
        "joint_mapping_complete_assets": sum(row.get("joint_mapping_complete", False) for row in records),
        "movable_name_set_exact_assets": sum(row.get("movable_name_set_exact", False) for row in records),
        "bullet_traversal_order_matches_xml_assets": sum(
            row.get("bullet_traversal_order_matches_xml", False) for row in records
        ),
        "bullet_traversal_order_changed_assets": sum(
            not row.get("bullet_traversal_order_matches_xml", False) for row in records
        ),
        "movable_type_exact_assets": sum(row.get("movable_type_exact", False) for row in records),
        "movable_axis_exact_assets": sum(row.get("movable_axis_exact", False) for row in records),
        "movable_limit_exact_assets": sum(row.get("movable_limit_exact", False) for row in records),
        "reset_readback_complete_assets": sum(row.get("status") == "PASS" for row in records),
        "status_counts": dict(sorted(Counter(row["status"] for row in records).items())),
        "retained_failure_count": sum(row["status"] != "PASS" for row in records),
        "retained_failures": [
            {
                "dataset_id": row["dataset_id"],
                "partnet_category": row["partnet_category"],
                "status": row["status"],
                "failure_class": row["failure_class"],
                "issues": row.get("issues", []),
                "state_expected": row["single_state_expected"] + row["sobol_state_expected"],
                "state_executed": row.get("single_state_executed", 0)
                + row.get("sobol_state_executed", 0),
            }
            for row in records
            if row["status"] != "PASS"
        ],
        "intent_to_run_asset_count": len(cohort["rows"]),
        "declared_movable_joints": sum(row["declared_movable_joint_count"] for row in cohort["rows"]),
        "functional_motion_joints": sum(row["functional_motion_joint_count"] for row in cohort["rows"]),
        "zero_width_joints_excluded": sum(row["zero_width_joint_count"] for row in cohort["rows"]),
        "unsupported_motion_joints_excluded": sum(row["unsupported_motion_joint_count"] for row in cohort["rows"]),
        "motors_disabled": sum(row.get("motors_disabled_count", 0) for row in records),
        "single_state_expected": sum(row["single_state_expected"] for row in cohort["rows"]),
        "single_state_executed": sum(row.get("single_state_executed", 0) for row in records),
        "sobol_state_expected": sum(row["sobol_state_expected"] for row in cohort["rows"]),
        "sobol_state_executed": sum(row.get("sobol_state_executed", 0) for row in records),
        "all_state_expected": state_expected,
        "all_state_executed": state_executed,
        "max_reset_readback_error": max(
            [float(row["max_reset_readback_error"]) for row in records if row.get("max_reset_readback_error") is not None]
            or [0.0]
        ),
        "reset_tolerance": RESET_TOLERANCE,
        "single_states_per_functional_joint": SINGLE_STATES,
        "sobol_states_per_asset_with_gt_1_functional_joint": SOBOL_STATES,
        "pybullet_load_flags": (
            "URDF_USE_INERTIA_FROM_FILE | URDF_USE_SELF_COLLISION | "
            "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | URDF_IGNORE_VISUAL_SHAPES | "
            "URDF_MERGE_FIXED_LINKS"
        ),
        "merge_fixed_links_policy": (
            "uniform for all selected assets; accepted only when Bullet/XML movable "
            "unique-name set/type/axis/limit checks all pass; traversal reorder is diagnostic"
        ),
        "motor_policy": "VELOCITY_CONTROL targetVelocity=0 force=0",
        "pose_evaluation": "resetJointState -> performCollisionDetection -> getJointState readback; no stepSimulation",
        "collision_metric_status": "N/A: official archive URDFs have zero collision elements",
        "contact_counts_collected": False,
        "penetration_counts_collected": False,
        "ccd": False,
        "child_timeout_seconds": timeout_seconds,
        "timeout_count": sum(row["status"] == "TIMEOUT" for row in records),
        "native_abort_count": sum(row["status"] == "NATIVE_ABORT" for row in records),
        "input_hash_mismatch_count": sum(not row["input_hash_matches_frozen"] for row in records),
        "child_expectation_mismatch_count": sum(not row["child_expectations_match"] for row in records),
        "selected_urdf_hashes_sha256": canonical_sha256(
            [row["urdf_sha256"] for row in cohort["rows"]]
        ),
        "records_sha256": canonical_sha256(records),
        "self_collision_flag_smoke": self_collision_flag_smoke(output),
        "thread_caps": {
            key: os.environ.get(key)
            for key in (
                "OMP_NUM_THREADS",
                "OPENBLAS_NUM_THREADS",
                "MKL_NUM_THREADS",
                "NUMEXPR_NUM_THREADS",
                "VECLIB_MAXIMUM_THREADS",
            )
        },
        "python_executable": sys.executable,
    }
    return records, summary


def provenance(archive: Path, metadata: Path) -> dict[str, Any]:
    lines = metadata.read_text(encoding="utf-8").splitlines()
    actual_hash = sha256_file(archive)
    checks = {
        "hf_revision_matches_pin": len(lines) >= 1 and lines[0] == HF_REVISION,
        "hf_object_hash_matches_archive": len(lines) >= 2 and lines[1] == actual_hash,
        "archive_hash_matches_pin": actual_hash == ARCHIVE_SHA256,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "dataset": "PhysX-Mobility",
        "official_huggingface_repository": "Caoza/PhysX-Mobility",
        "huggingface_revision": lines[0] if lines else None,
        "archive_sha256": actual_hash,
        "archive_bytes": archive.stat().st_size,
        "license": "CC-BY-NC-4.0",
        "dataset_card_relationship": "PhysX-Mobility is based on PartNet-Mobility",
        "role": "paired derivative dataset reference; not PhysX-Omni/PhysX-Anything method output",
        "independent_joint_gold": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--physx-root", type=Path, default=PHYSX_ROOT)
    parser.add_argument("--archive", type=Path, default=ARCHIVE)
    parser.add_argument("--hf-metadata", type=Path, default=HF_METADATA)
    parser.add_argument("--partnet-root", type=Path, default=PARTNET_ROOT)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--child-timeout", type=int, default=CHILD_TIMEOUT_SECONDS)
    args = parser.parse_args()
    physx_root = contained(args.physx_root, exists=True)
    archive = contained(args.archive, exists=True)
    hf_metadata = contained(args.hf_metadata, exists=True)
    partnet_root = contained(args.partnet_root, exists=True)
    output = contained(args.output, exists=False)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)

    source = provenance(archive, hf_metadata)
    if not source["passed"]:
        raise RuntimeError(f"provenance checks failed: {source['checks']}")
    write_json(output / "provenance.json", source)
    started = time.monotonic()
    static_records, static = static_audit(physx_root, partnet_root, archive)
    write_json(output / "static_asset_records.json", static_records)
    write_json(output / "static_summary.json", static)
    cohort = freeze_cohort(static_records)
    write_json(output / "frozen_functional_cohort.json", cohort)
    functional_records, functional = functional_audit(
        physx_root, cohort, output, args.child_timeout
    )
    write_json(output / "functional_asset_records.json", functional_records)
    write_json(output / "functional_summary.json", functional)

    n = static["official_archive_asset_count"]
    joints = static["movable_joint_total"]
    selected_ids = [row["dataset_id"] for row in cohort["rows"]]
    record_ids = [row["dataset_id"] for row in functional_records]
    selected_hashes = [row["urdf_sha256"] for row in cohort["rows"]]
    record_hashes = [row["frozen_urdf_sha256"] for row in functional_records]
    static_complete = bool(
        static["protocol_id"].endswith("_v2")
        and n == 2024
        and static["static_pass_assets"] == n
        and static["urdf_archive_byte_exact_assets"] == n
        and static["annotation_archive_byte_exact_assets"] == n
        and static["partseg_archive_subset_complete"]
    )
    functional_complete = bool(
        functional["reset_readback_complete_assets"] == cohort["category_count"]
        and functional["all_state_expected"] == functional["all_state_executed"]
        and functional["input_hash_mismatch_count"] == 0
        and functional["child_expectation_mismatch_count"] == 0
        and functional["self_collision_flag_smoke"]["passed"]
        and record_ids == selected_ids
        and record_hashes == selected_hashes
    )
    run_complete = static_complete and functional_complete
    retained_failures = [row for row in functional_records if row["status"] != "PASS"]
    successful_records = [row for row in functional_records if row["status"] == "PASS"]
    collision_na = "N/A: 0 official collision elements; no vacuous zero-contact score"
    table6 = {
        "method": "PhysX-Mobility (PartNet-derived dataset reference; supplementary only)",
        "articulable": f"{sum(row.get('movable_joint_count', 0) > 0 for row in static_records)}/{n} official assets expose movable URDF joints",
        "joints_per_asset": static["movable_joint_total"] / n,
        "native_joint_exposure": f"{joints}/{joints} declared URDF joints (package exposure proxy)",
        "joint_type_accuracy": "N/A: same-source derivative annotations; no independent gold",
        "joint_recall": "N/A: same-source PartNet derivative; no independent expected-joint gold",
        "parent_child_accuracy": "N/A: structural validity/export fidelity is not independent semantic accuracy",
        "axis_valid": "N/A: metadata/export fidelity only; no independent axis-on-moving-part judgment",
        "origin_valid": "N/A: metadata/export fidelity only; no independent pivot geometry judgment",
        "limit_valid": "N/A: metadata/export fidelity only; no independent semantic range gold",
        "joint_geometric_validity": collision_na,
        "asset_geometric_validity": collision_na,
        "full_range_collision_free": collision_na,
        "generic_range": f"{static['generic_revolute_total']}/{static['revolute_total']} revolute >=300 deg metadata proxy; {static['continuous_total']} continuous",
    }
    summary = {
        "protocol_id": "nano3d_table6_physx_mobility_release_audit_v1",
        "status": (
            "COMPLETE_STATIC_AND_RESET_READBACK_COLLISION_NA"
            if run_complete
            else "PARTIAL_RETAINED_FAILURES_COLLISION_NA"
        ),
        "elapsed_seconds": time.monotonic() - started,
        "provenance": source,
        "static": static,
        "functional": functional,
        "table6": table6,
        "publication_boundary": (
            "supplementary same-ID representation/export audit only; exclude from generated-method ranking and "
            "do not count PhysX-Mobility and PartNet-Mobility as independent confirmations"
        ),
        "semantic_na_boundary": (
            "PartNet-Mobility lineage removes independence for type/recall/parent/axis/origin/limit; "
            "zero collision geometry removes collision and geometry metrics"
        ),
        "hashes": {
            "runner_sha256": sha256_file(Path(__file__)),
            "provenance_sha256": sha256_file(output / "provenance.json"),
            "static_records_sha256": sha256_file(output / "static_asset_records.json"),
            "static_summary_sha256": sha256_file(output / "static_summary.json"),
            "cohort_sha256": sha256_file(output / "frozen_functional_cohort.json"),
            "functional_records_sha256": sha256_file(output / "functional_asset_records.json"),
            "functional_summary_sha256": sha256_file(output / "functional_summary.json"),
        },
    }
    write_json(output / "summary.json", summary)

    checks = {
        "source_provenance_passed": source["passed"],
        "official_archive_exactly_2024": n == 2024,
        "static_protocol_v2": static["protocol_id"].endswith("_v2"),
        "all_static_assets_passed": static["static_pass_assets"] == n,
        "all_2024_urdfs_byte_exact_to_archive": static["urdf_archive_byte_exact_assets"] == n
        and static["urdf_archive_byte_mismatch_assets"] == 0,
        "all_2024_finaljson_byte_exact_to_archive": static["annotation_archive_byte_exact_assets"] == n
        and static["annotation_archive_byte_mismatch_assets"] == 0,
        "partseg_archive_subset_complete": static["partseg_archive_subset_complete"]
        and static["partseg_inventory_missing_count"] == 0
        and static["partseg_archive_subset_local_sha256"]
        == static["partseg_inventory_archive_sha256"],
        "partseg_local_extras_catalogued_and_excluded": len(
            static["partseg_local_extra_members_excluded"]
        )
        == static["partseg_inventory_extra_count"]
        and not set(static["partseg_local_extra_members_excluded"])
        & set(static["partseg_inventory_missing_sample"]),
        "official_ids_all_same_id_partnet": static["partnet_same_id_assets"] == n,
        "nonofficial_local_variants_excluded": static["local_nonofficial_urdf_variant_count"] == 28,
        "all_official_urdfs_have_no_collision": static["collision_element_total"] == 0,
        "collision_scores_not_collected": not functional["contact_counts_collected"] and not functional["penetration_counts_collected"],
        "functional_selection_recomputes": cohort == freeze_cohort(static_records),
        "functional_protocol_v3": functional["protocol_id"].endswith("_v3"),
        "all_selected_assets_retained": len(functional_records) == cohort["category_count"],
        "functional_record_ids_and_order_match_cohort": record_ids == selected_ids,
        "functional_record_hashes_match_cohort": record_hashes == selected_hashes
        and all(row["input_hash_matches_frozen"] for row in functional_records),
        "selected_hash_digest_matches_cohort": functional["selected_urdf_hashes_sha256"]
        == canonical_sha256(selected_hashes),
        "functional_records_digest_recomputes": functional["records_sha256"]
        == canonical_sha256(functional_records),
        "zero_input_hash_mismatches": functional["input_hash_mismatch_count"] == 0,
        "zero_child_expectation_mismatches": functional["child_expectation_mismatch_count"] == 0,
        "merge_fixed_links_used_uniformly": "URDF_MERGE_FIXED_LINKS"
        in functional["pybullet_load_flags"],
        "all_bullet_xml_movable_name_sets_exact": functional[
            "movable_name_set_exact_assets"
        ]
        == cohort["category_count"],
        "all_bullet_xml_movable_types_exact": functional["movable_type_exact_assets"]
        == cohort["category_count"],
        "all_bullet_xml_movable_axes_exact": functional["movable_axis_exact_assets"]
        == cohort["category_count"],
        "all_bullet_xml_movable_limits_exact": functional["movable_limit_exact_assets"]
        == cohort["category_count"],
        "self_collision_flag_smoke_passed": functional["self_collision_flag_smoke"]["passed"],
        "success_and_retained_failure_partition_complete": len(successful_records)
        + len(retained_failures)
        == cohort["category_count"],
        "retained_failures_not_replaced": [row["dataset_id"] for row in retained_failures]
        == [row["dataset_id"] for row in functional["retained_failures"]]
        and functional["retained_failure_count"] == len(retained_failures),
        "intent_to_run_expected_denominator_preserved": functional["all_state_expected"]
        == sum(
            row["single_state_expected"] + row["sobol_state_expected"]
            for row in cohort["rows"]
        ),
        "successful_assets_executed_all_expected_states": all(
            row.get("single_state_executed", 0) + row.get("sobol_state_executed", 0)
            == row["single_state_expected"] + row["sobol_state_expected"]
            for row in successful_records
        ),
        "failed_assets_retain_frozen_expected_denominators": all(
            row["single_state_expected"] + row["sobol_state_expected"]
            == cohort["rows"][index]["single_state_expected"]
            + cohort["rows"][index]["sobol_state_expected"]
            for index, row in enumerate(functional_records)
            if row["status"] != "PASS"
        ),
        "executed_states_do_not_exceed_intent": functional["all_state_executed"]
        <= functional["all_state_expected"],
        "reset_readback_exact": functional["max_reset_readback_error"] <= RESET_TOLERANCE,
        "zero_timeouts": functional["timeout_count"] == 0,
        "zero_native_aborts": functional["native_abort_count"] == 0,
        "summary_status_matches_retained_outcomes": (
            summary["status"] == "COMPLETE_STATIC_AND_RESET_READBACK_COLLISION_NA"
            if not retained_failures
            else summary["status"] == "PARTIAL_RETAINED_FAILURES_COLLISION_NA"
        ),
        "semantic_accuracy_fail_closed": all(str(table6[key]).startswith("N/A") for key in (
            "joint_type_accuracy", "joint_recall", "parent_child_accuracy", "axis_valid",
            "origin_valid", "limit_valid", "joint_geometric_validity",
            "asset_geometric_validity", "full_range_collision_free",
        )),
    }
    verification = {
        "passed": all(checks.values()),
        "checks": checks,
        "summary_sha256": sha256_file(output / "summary.json"),
    }
    write_json(output / "self_check.json", verification)

    report = [
        "# Table 6: PhysX-Mobility release reference",
        "",
        f"Status: **{summary['status']}**. This is a same-ID PartNet-Mobility derivative dataset reference, not PhysX-Omni or PhysX-Anything generated-method output.",
        "",
        "## Provenance and scope",
        "",
        f"- Official HF revision: `{source['huggingface_revision']}`; archive SHA-256: `{source['archive_sha256']}`; license: {source['license']}.",
        f"- Main denominator: {n} numeric URDF/JSON/partseg IDs present in the official archive. The {static['local_nonofficial_urdf_variant_count']} local `_collision`/`_sim` variants are excluded.",
        f"- Archive binding: URDF {static['urdf_archive_byte_exact_assets']}/{n} and finaljson {static['annotation_archive_byte_exact_assets']}/{n} are byte-exact by size+SHA-256. All {static['archive_partseg_member_file_count']} archive partseg members are present (missing {static['partseg_inventory_missing_count']}); {static['partseg_inventory_extra_count']} local-only additions are catalogued and excluded from the official cohort.",
        f"- All {static['partnet_same_id_assets']}/{n} IDs exist in the local PartNet-Mobility release. Dataset-card lineage and same-ID reuse make PartNet agreement export fidelity, not independent accuracy.",
        "",
        "## Static release audit",
        "",
        f"- Static package pass: {static['static_pass_assets']}/{n}; valid URDF trees: {static['valid_tree_assets']}/{n}.",
        f"- Movable joints: {static['movable_joint_total']} ({json.dumps(static['joint_type_counts'], sort_keys=True)}), mean {static['movable_joint_total']/n:.6f}/asset; functional-motion {static['functional_motion_joint_total']}; zero-width {static['zero_width_joint_total']}; unsupported/floating {static['unsupported_motion_joint_total']}.",
        f"- Metadata proxies: parent/child structurally valid {static['parent_child_structurally_valid_total']}/{static['movable_joint_total']}; axis metadata valid {static['axis_metadata_valid_total']}/{static['movable_joint_total']}; origin metadata valid {static['origin_metadata_valid_total']}/{static['movable_joint_total']}; bounded limits {static['bounded_limit_valid_total']}/{static['bounded_expected_total']}.",
        f"- Visual mesh references: {static['visual_mesh_reference_total']}; missing: {static['missing_visual_mesh_reference_total']}. Collision elements: {static['collision_element_total']}; collision mesh references: {static['collision_mesh_reference_total']}.",
        f"- Inertial tags are syntactically positive on {static['valid_inertial_link_total']}/{static['link_total']} links, but {static['placeholder_inertial_link_total']}/{static['link_total']} are the uniform mass=1 / unit-diagonal placeholders and are not physical mass fidelity.",
        f"- `finaljson.group_info` to released URDF export fidelity: names {static['group_urdf_joint_name_match_total']}/{static['annotation_group_movable_total']}; all encoded fields {static['group_urdf_all_fields_match_total']}/{static['annotation_group_movable_total']}. This is same-release serialization fidelity.",
        f"- Exact child-mesh matched PartNet/PhysX pairs: {static['partnet_unique_mesh_matched_joint_total']}; exact type {static['partnet_unique_mesh_type_exact_total']}; rotational class {static['partnet_unique_mesh_rotational_class_total']}; axis line {static['partnet_unique_mesh_axis_line_total']}. These are same-source preservation diagnostics.",
        "",
        "## PyBullet v3 reset/readback",
        "",
        f"The frozen cohort contains one outcome-independent SHA-256 winner from each of {cohort['category_count']} exact PartNet categories. Load/reset outcomes never affect selection and failures are not replaced.",
        f"- Load / joint-map / complete: {functional['pybullet_load_success_assets']}/{cohort['category_count']} / {functional['joint_mapping_complete_assets']}/{cohort['category_count']} / {functional['reset_readback_complete_assets']}/{cohort['category_count']}.",
        f"- Uniform `URDF_MERGE_FIXED_LINKS` preservation checks: unique name set {functional['movable_name_set_exact_assets']}/{cohort['category_count']}; type {functional['movable_type_exact_assets']}/{cohort['category_count']}; axis {functional['movable_axis_exact_assets']}/{cohort['category_count']}; limits {functional['movable_limit_exact_assets']}/{cohort['category_count']}. Bullet traversal order changed for {functional['bullet_traversal_order_changed_assets']}/{cohort['category_count']} assets and is diagnostic only; reset indices are reordered to XML movable order by name.",
        f"- Declared/functionally swept/zero-width/unsupported joints: {functional['declared_movable_joints']} / {functional['functional_motion_joints']} / {functional['zero_width_joints_excluded']} / {functional['unsupported_motion_joints_excluded']}.",
        f"- Executed states: {functional['single_state_executed']}/{functional['single_state_expected']} single-joint and {functional['sobol_state_executed']}/{functional['sobol_state_expected']} Sobol; max reset/readback error {functional['max_reset_readback_error']:.3g} (tol {RESET_TOLERANCE:g}).",
        f"- Retained evaluator failures (no replacement): {json.dumps(functional['retained_failures'], sort_keys=True)}.",
        f"- Self-collision protocol smoke: {'PASS' if functional['self_collision_flag_smoke']['passed'] else 'FAIL'}; contacts by flag {json.dumps(functional['self_collision_flag_smoke']['contacts'], sort_keys=True)}. This fixture validates flags only and is excluded from dataset metrics.",
        "- Motors are disabled. Each pose uses resetJointState -> performCollisionDetection -> getJointState with no stepSimulation. Each asset runs in an isolated subprocess.",
        "- Contact/penetration rates are intentionally not collected: zero official collision elements make them vacuous. This is not CCD or a geometry-validity test.",
        "",
        "## Table 6 candidate row",
        "",
        "| Method | Articulable | Joints/Asset | Native Joint Exposure | Joint Type Accuracy | Joint Recall | Parent-Child Accuracy | Axis Valid | Origin Valid | Limit Valid | Joint Geom. Valid | Asset Geom. Valid | Full-Range Collision-Free | Generic Range |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| PhysX-Mobility (PartNet-derived dataset reference; supplementary) | {sum(row.get('movable_joint_count', 0) > 0 for row in static_records)}/{n} expose movable URDF joints* | {static['movable_joint_total']/n:.3f} ({static['movable_joint_total']}/{n})* | {static['movable_joint_total']}/{static['movable_joint_total']} declared joints* | N/A (same-source derivative) | N/A (same-source derivative) | N/A ({static['parent_child_structurally_valid_total']}/{static['movable_joint_total']} structurally valid*) | N/A ({static['axis_metadata_valid_total']}/{static['movable_joint_total']} metadata-valid*) | N/A ({static['origin_metadata_valid_total']}/{static['movable_joint_total']} metadata-valid*) | N/A ({static['bounded_limit_valid_total']}/{static['bounded_expected_total']} bounded metadata-valid*) | N/A (0 collision elements/no independent axis gold) | N/A (0 collision elements/no independent gold) | N/A (0 collision elements) | {static['generic_revolute_total']}/{static['revolute_total']} revolute >=300 deg*; {static['continuous_total']} continuous* |",
        "",
        "`*` denotes package exposure, metadata, or same-source export fidelity, never independent articulation accuracy. Keep this row supplementary and out of generated-method rankings.",
    ]
    (output / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps({"status": summary["status"], "self_check": verification}, indent=2))
    return 0 if verification["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
