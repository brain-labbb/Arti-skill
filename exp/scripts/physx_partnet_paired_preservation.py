#!/usr/bin/env python3
"""Same-ID PartNet-Mobility to PhysX-Mobility preservation diagnostics."""

from __future__ import annotations

from collections import Counter, defaultdict, deque
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from typing import Any
import zipfile


KNOWN_MOVABLE_TYPES = {"revolute", "continuous", "prismatic", "planar", "floating"}
EPSILON = 1e-6


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def mesh_basename(filename: str) -> str:
    return Path(filename.replace("\\", "/")).name


def floats(value: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if value is None:
        return default
    parsed = tuple(float(item) for item in value.split())
    return parsed if parsed else default


def parse_urdf(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError(f"expected robot root, found {root.tag!r}")
    links: dict[str, dict[str, Any]] = {}
    for link in root.findall("link"):
        name = link.attrib.get("name", "")
        if not name or name in links:
            raise ValueError("URDF contains unnamed or duplicate links")
        visual_meshes = [
            mesh_basename(mesh.attrib["filename"])
            for mesh in link.findall("visual/geometry/mesh")
            if mesh.attrib.get("filename")
        ]
        collision_meshes = [
            mesh_basename(mesh.attrib["filename"])
            for mesh in link.findall("collision/geometry/mesh")
            if mesh.attrib.get("filename")
        ]
        links[name] = {
            "name": name,
            "visual_meshes": visual_meshes,
            "collision_meshes": collision_meshes,
            "has_visual_tag": link.find("visual/geometry") is not None,
            "has_collision_tag": link.find("collision/geometry") is not None,
        }
    joints = []
    for index, joint in enumerate(root.findall("joint")):
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        if parent not in links or child not in links:
            raise ValueError(f"joint has invalid endpoint: {parent}->{child}")
        joint_type = joint.attrib.get("type", "")
        axis_node = joint.find("axis")
        limit_node = joint.find("limit")
        axis = (
            floats(axis_node.attrib.get("xyz"), (1.0, 0.0, 0.0))
            if joint_type in {"revolute", "continuous", "prismatic"}
            else None
        )
        limit = {
            "lower": (
                float(limit_node.attrib["lower"])
                if limit_node is not None and "lower" in limit_node.attrib
                else None
            ),
            "upper": (
                float(limit_node.attrib["upper"])
                if limit_node is not None and "upper" in limit_node.attrib
                else None
            ),
        }
        joints.append(
            {
                "name": joint.attrib.get("name", f"joint_{index}"),
                "type": joint_type,
                "parent": parent,
                "child": child,
                "axis": list(axis) if axis is not None else None,
                "limit": limit,
            }
        )
    return {"links": links, "joints": joints}


class UnionFind:
    def __init__(self, names: list[str]) -> None:
        self.parent = {name: name for name in names}

    def find(self, name: str) -> str:
        current = name
        while self.parent[current] != current:
            self.parent[current] = self.parent[self.parent[current]]
            current = self.parent[current]
        return current

    def union(self, left: str, right: str) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root != right_root:
            self.parent[right_root] = left_root


def joint_class(joint_type: str) -> str:
    if joint_type in {"continuous", "revolute"}:
        return "rotational"
    if joint_type == "prismatic":
        return "prismatic"
    return joint_type


def contracted_graph(package: dict[str, Any]) -> dict[str, Any]:
    links = package["links"]
    union = UnionFind(list(links))
    for joint in package["joints"]:
        if joint["type"] == "fixed":
            union.union(joint["parent"], joint["child"])
    members: dict[str, list[str]] = defaultdict(list)
    for link in links:
        members[union.find(link)].append(link)
    components_raw = []
    for root, component_links in members.items():
        meshes = sorted(
            {
                mesh
                for link in component_links
                for mesh in links[link]["visual_meshes"]
            }
        )
        components_raw.append(
            {
                "union_root": root,
                "links": sorted(component_links),
                "visual_meshes": meshes,
                "mesh_key": "|".join(meshes),
            }
        )
    components_raw.sort(key=lambda row: (row["mesh_key"], row["links"]))
    components = []
    component_by_link: dict[str, int] = {}
    for index, row in enumerate(components_raw):
        component = {key: value for key, value in row.items() if key != "union_root"}
        component["component_id"] = index
        components.append(component)
        for link in row["links"]:
            component_by_link[link] = index
    edges = []
    for joint in package["joints"]:
        if joint["type"] == "fixed":
            continue
        parent_component = component_by_link[joint["parent"]]
        child_component = component_by_link[joint["child"]]
        edges.append(
            {
                **joint,
                "parent_component": parent_component,
                "child_component": child_component,
                "parent_mesh_key": components[parent_component]["mesh_key"],
                "child_mesh_key": components[child_component]["mesh_key"],
                "joint_class": joint_class(joint["type"]),
            }
        )
    edges.sort(key=lambda row: (row["child_mesh_key"], row["parent_mesh_key"], row["name"]))

    adjacency: dict[int, list[int]] = defaultdict(list)
    indegree: Counter[int] = Counter()
    for edge in edges:
        adjacency[edge["parent_component"]].append(edge["child_component"])
        indegree[edge["child_component"]] += 1
    roots = [row["component_id"] for row in components if indegree[row["component_id"]] == 0]
    depth = 0
    reached: set[int] = set()
    queue = deque((root, 1) for root in roots)
    while queue:
        node, value = queue.popleft()
        if node in reached:
            continue
        reached.add(node)
        depth = max(depth, value)
        queue.extend((child, value + 1) for child in adjacency[node])
    valid = (
        len(roots) == 1
        and len(reached) == len(components)
        and len(edges) == len(components) - 1
        and all(count <= 1 for count in indegree.values())
    )
    return {
        "component_count": len(components),
        "movable_edge_count": len(edges),
        "depth": depth,
        "root_count": len(roots),
        "valid_tree": valid,
        "components": components,
        "edges": edges,
    }


def close(left: float | None, right: float | None) -> bool:
    if left is None or right is None:
        return left is None and right is None
    return abs(left - right) <= EPSILON


def normalize(vector: list[float] | None) -> tuple[float, ...] | None:
    if vector is None:
        return None
    magnitude = sum(item * item for item in vector) ** 0.5
    if magnitude <= EPSILON:
        return None
    return tuple(item / magnitude for item in vector)


def axis_comparison(left: list[float] | None, right: list[float] | None) -> dict[str, Any]:
    left_norm = normalize(left)
    right_norm = normalize(right)
    comparable = left_norm is not None and right_norm is not None and len(left_norm) == len(right_norm)
    if not comparable:
        return {
            "comparable": False,
            "directed_direction_preserved": None,
            "undirected_direction_preserved": None,
        }
    dot = sum(a * b for a, b in zip(left_norm, right_norm))
    return {
        "comparable": True,
        "directed_direction_preserved": dot >= 1.0 - EPSILON,
        "undirected_direction_preserved": abs(dot) >= 1.0 - EPSILON,
        "normalized_dot": dot,
        "axis_location_or_plucker_line_compared": False,
        "reason_location_not_compared": "joint origins and axes are expressed in package-specific link frames; no proven common frame is available",
    }


def limit_comparison(left: dict[str, Any], right: dict[str, Any]) -> dict[str, Any]:
    comparable = any(value is not None for value in (*left.values(), *right.values()))
    return {
        "comparable": comparable,
        "preserved": close(left["lower"], right["lower"])
        and close(left["upper"], right["upper"]),
        "partnet": left,
        "physx": right,
    }


def edge_signature(edge: dict[str, Any], *, rotational_class: bool) -> tuple[str, str, str]:
    label = edge["joint_class"] if rotational_class else edge["type"]
    return edge["parent_mesh_key"], edge["child_mesh_key"], label


def match_joints(partnet: dict[str, Any], physx: dict[str, Any]) -> dict[str, Any]:
    px_by_child: dict[str, list[int]] = defaultdict(list)
    for index, edge in enumerate(physx["edges"]):
        px_by_child[edge["child_mesh_key"]].append(index)
    used: set[int] = set()
    matches = []
    unmatched_partnet = []
    for pm_edge in partnet["edges"]:
        candidates = [
            index for index in px_by_child.get(pm_edge["child_mesh_key"], []) if index not in used
        ]
        if not pm_edge["child_mesh_key"]:
            unmatched_partnet.append(
                {"joint": pm_edge["name"], "reason": "empty_child_mesh_identity"}
            )
            continue
        if not candidates:
            unmatched_partnet.append(
                {"joint": pm_edge["name"], "reason": "no_exact_child_mesh_component"}
            )
            continue
        if len(candidates) > 1:
            unmatched_partnet.append(
                {
                    "joint": pm_edge["name"],
                    "reason": "ambiguous_exact_child_mesh_component",
                    "candidate_count": len(candidates),
                }
            )
            continue
        index = candidates[0]
        used.add(index)
        px_edge = physx["edges"][index]
        axis = axis_comparison(pm_edge["axis"], px_edge["axis"])
        limit = limit_comparison(pm_edge["limit"], px_edge["limit"])
        matches.append(
            {
                "partnet_joint": pm_edge["name"],
                "physx_joint": px_edge["name"],
                "child_mesh_key": pm_edge["child_mesh_key"],
                "parent_preserved": pm_edge["parent_mesh_key"] == px_edge["parent_mesh_key"],
                "partnet_parent_mesh_key": pm_edge["parent_mesh_key"],
                "physx_parent_mesh_key": px_edge["parent_mesh_key"],
                "partnet_type": pm_edge["type"],
                "physx_type": px_edge["type"],
                "exact_type_preserved": pm_edge["type"] == px_edge["type"],
                "rotational_class_preserved": pm_edge["joint_class"] == px_edge["joint_class"],
                "axis": axis,
                "limit": limit,
            }
        )
    unmatched_physx = [
        {"joint": edge["name"], "reason": "no_unique_partnet_child_mesh_match"}
        for index, edge in enumerate(physx["edges"])
        if index not in used
    ]
    pm_type_multiset = Counter(edge["type"] for edge in partnet["edges"])
    px_type_multiset = Counter(edge["type"] for edge in physx["edges"])
    pm_class_multiset = Counter(edge["joint_class"] for edge in partnet["edges"])
    px_class_multiset = Counter(edge["joint_class"] for edge in physx["edges"])
    pm_exact_graph = Counter(edge_signature(edge, rotational_class=False) for edge in partnet["edges"])
    px_exact_graph = Counter(edge_signature(edge, rotational_class=False) for edge in physx["edges"])
    pm_class_graph = Counter(edge_signature(edge, rotational_class=True) for edge in partnet["edges"])
    px_class_graph = Counter(edge_signature(edge, rotational_class=True) for edge in physx["edges"])
    return {
        "partnet_movable_joint_count": len(partnet["edges"]),
        "physx_movable_joint_count": len(physx["edges"]),
        "movable_joint_count_preserved": len(partnet["edges"]) == len(physx["edges"]),
        "partnet_joint_type_multiset": dict(sorted(pm_type_multiset.items())),
        "physx_joint_type_multiset": dict(sorted(px_type_multiset.items())),
        "exact_joint_type_multiset_preserved": pm_type_multiset == px_type_multiset,
        "partnet_joint_class_multiset": dict(sorted(pm_class_multiset.items())),
        "physx_joint_class_multiset": dict(sorted(px_class_multiset.items())),
        "rotational_class_multiset_preserved": pm_class_multiset == px_class_multiset,
        "contracted_graph_exact_type_preserved": pm_exact_graph == px_exact_graph,
        "contracted_graph_rotational_class_preserved": pm_class_graph == px_class_graph,
        "matching_rule": "unique exact child-component visual-mesh basename set; joint type and numeric parameters are not matching inputs",
        "matched_count": len(matches),
        "matches": matches,
        "unmatched_partnet": unmatched_partnet,
        "unmatched_physx": unmatched_physx,
        "unmatched_partnet_reason_counts": dict(
            sorted(Counter(row["reason"] for row in unmatched_partnet).items())
        ),
        "unmatched_physx_reason_counts": dict(
            sorted(Counter(row["reason"] for row in unmatched_physx).items())
        ),
    }


def mesh_retention(
    dataset_id: str,
    partnet_root: Path,
    partnet_package: dict[str, Any],
    physx_package: dict[str, Any],
    archive: zipfile.ZipFile,
) -> dict[str, Any]:
    pm_names = sorted(
        {mesh for link in partnet_package["links"].values() for mesh in link["visual_meshes"]}
    )
    px_names = sorted(
        {mesh for link in physx_package["links"].values() for mesh in link["visual_meshes"]}
    )
    retained = sorted(set(pm_names) & set(px_names))
    missing = sorted(set(pm_names) - set(px_names))
    extra = sorted(set(px_names) - set(pm_names))
    byte_exact = []
    byte_mismatch = []
    missing_payload = []
    rows = []
    for name in retained:
        pm_path = partnet_root / dataset_id / "textured_objs" / name
        px_archive_path = f"PhysX_mobility/partseg/{dataset_id}/objs/{name}"
        if not pm_path.is_file():
            missing_payload.append({"mesh": name, "side": "partnet"})
            continue
        try:
            px_bytes = archive.read(px_archive_path)
        except KeyError:
            missing_payload.append({"mesh": name, "side": "physx_archive"})
            continue
        pm_bytes = pm_path.read_bytes()
        exact = pm_bytes == px_bytes
        (byte_exact if exact else byte_mismatch).append(name)
        rows.append(
            {
                "mesh": name,
                "partnet_bytes": len(pm_bytes),
                "physx_bytes": len(px_bytes),
                "partnet_sha256": sha256_bytes(pm_bytes),
                "physx_sha256": sha256_bytes(px_bytes),
                "byte_exact": exact,
            }
        )
    denominator = len(pm_names)
    return {
        "partnet_referenced_unique_mesh_count": denominator,
        "physx_referenced_unique_mesh_count": len(px_names),
        "name_retained_count": len(retained),
        "name_retention_rate": len(retained) / denominator if denominator else None,
        "byte_exact_count": len(byte_exact),
        "byte_exact_retention_rate": len(byte_exact) / denominator if denominator else None,
        "missing_mesh_names": missing,
        "extra_mesh_names": extra,
        "byte_mismatch_mesh_names": byte_mismatch,
        "missing_payloads": missing_payload,
        "mesh_records": rows,
    }


def recover_physx_link_labels(
    physx_package: dict[str, Any], finaljson_path: Path
) -> tuple[dict[str, str], dict[str, Any]]:
    annotation = json.loads(finaljson_path.read_text(encoding="utf-8"))
    object_to_names: dict[str, set[str]] = defaultdict(set)
    duplicate_object_assignment = []
    for part in annotation.get("parts", []):
        name = str(part.get("name", "")).strip()
        for obj in part.get("obj", []):
            object_to_names[str(obj)].add(name)
    for obj, names in sorted(object_to_names.items()):
        if len(names) != 1:
            duplicate_object_assignment.append({"object": obj, "names": sorted(names)})

    labels: dict[str, str] = {}
    link_records = []
    total_mesh_references = 0
    mapped_mesh_references = 0
    visual_link_count = 0
    ambiguous_link_count = 0
    incomplete_link_count = 0
    for link_name, link in physx_package["links"].items():
        meshes = link["visual_meshes"]
        if not meshes:
            continue
        visual_link_count += 1
        names = set()
        missing = []
        ambiguous_objects = []
        for mesh in meshes:
            total_mesh_references += 1
            obj = Path(mesh).stem
            object_names = object_to_names.get(obj, set())
            if len(object_names) == 1:
                mapped_mesh_references += 1
                names.update(object_names)
            elif not object_names:
                missing.append(obj)
            else:
                ambiguous_objects.append({"object": obj, "names": sorted(object_names)})
        status = "recovered"
        if missing:
            status = "incomplete_mesh_mapping"
            incomplete_link_count += 1
        elif ambiguous_objects or len(names) != 1:
            status = "ambiguous_part_label"
            ambiguous_link_count += 1
        else:
            labels[link_name] = next(iter(names))
        link_records.append(
            {
                "link": link_name,
                "visual_mesh_count": len(meshes),
                "mesh_names": meshes,
                "candidate_part_names": sorted(names),
                "missing_objects": missing,
                "ambiguous_objects": ambiguous_objects,
                "status": status,
                "recovered_label": labels.get(link_name),
            }
        )
    return labels, {
        "mapping_rule": (
            "a visual-bearing PhysX link receives a finaljson part name only when every visual "
            "mesh basename maps to exactly one annotation obj and all mapped meshes agree on one part name"
        ),
        "total_package_link_count": len(physx_package["links"]),
        "visual_bearing_link_count": visual_link_count,
        "recovered_link_label_count": len(labels),
        "recovered_label_coverage_all_links": (
            len(labels) / len(physx_package["links"]) if physx_package["links"] else None
        ),
        "recovered_label_coverage_visual_links": (
            len(labels) / visual_link_count if visual_link_count else None
        ),
        "visual_mesh_reference_count": total_mesh_references,
        "mapped_visual_mesh_reference_count": mapped_mesh_references,
        "mapped_visual_mesh_reference_rate": (
            mapped_mesh_references / total_mesh_references if total_mesh_references else None
        ),
        "ambiguous_link_count": ambiguous_link_count,
        "incomplete_link_count": incomplete_link_count,
        "duplicate_annotation_object_assignment_count": len(duplicate_object_assignment),
        "duplicate_annotation_object_assignments": duplicate_object_assignment,
        "link_records": link_records,
        "used_as_reference_hierarchy": False,
        "used_as_prediction_side_metadata": True,
    }


def evaluate_pair(
    dataset_id: str,
    partnet_root: Path,
    physx_urdf: Path,
    physx_finaljson: Path,
    archive: zipfile.ZipFile,
) -> tuple[dict[str, Any], dict[str, str], dict[str, Any]]:
    pm_urdf = partnet_root / dataset_id / "mobility.urdf"
    pm_package = parse_urdf(pm_urdf)
    px_package = parse_urdf(physx_urdf)
    pm_contracted = contracted_graph(pm_package)
    px_contracted = contracted_graph(px_package)
    matching = match_joints(pm_contracted, px_contracted)
    retention = mesh_retention(dataset_id, partnet_root, pm_package, px_package, archive)
    labels, label_audit = recover_physx_link_labels(px_package, physx_finaljson)
    result = {
        "dataset_id": dataset_id,
        "partnet_raw_link_count": len(pm_package["links"]),
        "physx_raw_link_count": len(px_package["links"]),
        "raw_link_count_delta_physx_minus_partnet": len(px_package["links"])
        - len(pm_package["links"]),
        "partnet_released_collision_link_count": sum(
            link["has_collision_tag"] for link in pm_package["links"].values()
        ),
        "partnet_released_collision_link_ratio": sum(
            link["has_collision_tag"] for link in pm_package["links"].values()
        )
        / len(pm_package["links"]),
        "physx_released_collision_link_count": sum(
            link["has_collision_tag"] for link in px_package["links"].values()
        ),
        "physx_released_collision_link_ratio": sum(
            link["has_collision_tag"] for link in px_package["links"].values()
        )
        / len(px_package["links"]),
        "partnet_contracted": {
            key: value for key, value in pm_contracted.items() if key not in {"components", "edges"}
        },
        "physx_contracted": {
            key: value for key, value in px_contracted.items() if key not in {"components", "edges"}
        },
        "contracted_component_count_delta_physx_minus_partnet": px_contracted[
            "component_count"
        ]
        - pm_contracted["component_count"],
        "contracted_depth_delta_physx_minus_partnet": px_contracted["depth"]
        - pm_contracted["depth"],
        "mesh_retention": retention,
        "joint_preservation": matching,
        "metadata_assisted_link_label_audit": label_audit,
    }
    return result, labels, {"partnet": pm_package, "physx": px_package}
