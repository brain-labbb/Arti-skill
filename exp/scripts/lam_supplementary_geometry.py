#!/usr/bin/env python3
"""Geometry primitives for the frozen LAM supplementary URDF evaluation.

This module deliberately separates geometry parsing from the formal cohort runner.
It never falls back from exact point-to-triangle proximity to sampled KD-tree
distances: if the rtree-backed trimesh proximity backend cannot be qualified, the
surface-distance fields are returned as ``N/E`` while the static inventory remains
available.

The loader is intentionally narrow.  It accepts URDF primitives and local OBJ
resources, rejects external entities, URIs, path traversal, symlinks, non-OBJ mesh
formats, and non-finite geometry.  Link poses are evaluated at historical q=0 using
URDF joint/origin semantics.
"""

from __future__ import annotations

from collections import Counter, defaultdict, deque
from functools import lru_cache
import hashlib
import math
from pathlib import Path
import statistics
import time
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np


GEOMETRY_PROTOCOL_ID = "lam_supplementary_geometry_v1"
SEED_PROTOCOL = "sha256(protocol_id|asset_key|direction)_pcg64dxsm_v1"
OBJ_PROTOCOL = "local_obj_triangle_fan_weld_relative_diagonal_v1"
EXACT_BACKEND = "trimesh.proximity.ProximityQuery.on_surface+rtree"
MAX_URDF_BYTES = 64 * 1024 * 1024
MAX_OBJ_BYTES = 1024 * 1024 * 1024
DEFAULT_CLOSEST_CHUNK = 4096
ANALYTIC_KINDS = frozenset(("box", "cylinder", "sphere"))
COLLISION_LOAD_TIME_PROTOCOL_ID = "trimesh_process_false_rtree_bvh_warm5_v1"
COLLISION_LOAD_TIME_BACKEND = "trimesh.Trimesh(process=False).triangles_tree+rtree"
COLLISION_LOAD_TIME_WARMUP_REPEATS = 1
COLLISION_LOAD_TIME_MEASURED_REPEATS = 5
COLLISION_LOAD_TIME_THREADS = 1


class GeometryShape:
    """One successfully loaded URDF geometry element (internal representation)."""

    __slots__ = (
        "role",
        "link_name",
        "element_index",
        "kind",
        "mesh_path",
        "vertices",
        "faces",
        "bounds_min",
        "bounds_max",
        "triangle_count",
        "watertight",
        "winding_consistent",
        "finite_positive_volume",
        "volume",
        "surface_exact_eligible",
        "validation",
    )

    def __init__(
        self,
        *,
        role: str,
        link_name: str,
        element_index: int,
        kind: str,
        mesh_path: str | None,
        vertices: np.ndarray | None,
        faces: np.ndarray | None,
        bounds_min: np.ndarray,
        bounds_max: np.ndarray,
        triangle_count: int,
        watertight: bool,
        winding_consistent: bool,
        finite_positive_volume: bool,
        volume: float | None,
        surface_exact_eligible: bool,
        validation: dict[str, Any],
    ) -> None:
        self.role = role
        self.link_name = link_name
        self.element_index = element_index
        self.kind = kind
        self.mesh_path = mesh_path
        self.vertices = vertices
        self.faces = faces
        self.bounds_min = bounds_min
        self.bounds_max = bounds_max
        self.triangle_count = triangle_count
        self.watertight = watertight
        self.winding_consistent = winding_consistent
        self.finite_positive_volume = finite_positive_volume
        self.volume = volume
        self.surface_exact_eligible = surface_exact_eligible
        self.validation = validation


class GeometryInventory:
    """Parsed q=0 geometry plus fail-closed extraction diagnostics."""

    __slots__ = (
        "urdf_path",
        "package_root",
        "link_names",
        "tree_valid",
        "link_transforms",
        "declared_counts",
        "visual_bearing_links",
        "declared_collision_bearing_links",
        "shapes",
        "issues",
        "weld_rel_tol",
    )

    def __init__(self, urdf_path: Path, package_root: Path, weld_rel_tol: float) -> None:
        self.urdf_path = urdf_path
        self.package_root = package_root
        self.link_names: list[str] = []
        self.tree_valid = False
        self.link_transforms: dict[str, np.ndarray] = {}
        self.declared_counts = {"visual": 0, "collision": 0}
        self.visual_bearing_links: set[str] = set()
        self.declared_collision_bearing_links: set[str] = set()
        self.shapes: list[GeometryShape] = []
        self.issues: list[str] = []
        self.weld_rel_tol = weld_rel_tol

    def role_shapes(self, role: str) -> list[GeometryShape]:
        return [shape for shape in self.shapes if shape.role == role]


def _finite_float(value: str, label: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{label} is not finite")
    return parsed


def _vector(
    raw: str | None,
    default: Sequence[float],
    label: str,
    *,
    length: int = 3,
) -> np.ndarray:
    values = list(default) if raw is None else raw.split()
    if len(values) != length:
        raise ValueError(f"{label} must contain {length} values")
    parsed = np.asarray(
        [_finite_float(str(value), f"{label}[{index}]") for index, value in enumerate(values)],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(parsed)):
        raise ValueError(f"{label} contains non-finite values")
    return parsed


def _origin_transform(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    if element is None:
        return transform
    xyz = _vector(element.get("xyz"), (0.0, 0.0, 0.0), "origin xyz")
    roll, pitch, yaw = _vector(
        element.get("rpy"), (0.0, 0.0, 0.0), "origin rpy"
    )
    cr, sr = math.cos(float(roll)), math.sin(float(roll))
    cp, sp = math.cos(float(pitch)), math.sin(float(pitch))
    cy, sy = math.cos(float(yaw)), math.sin(float(yaw))
    rx = np.asarray(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=np.float64)
    ry = np.asarray(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=np.float64)
    rz = np.asarray(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=np.float64)
    transform[:3, :3] = rz @ ry @ rx
    transform[:3, 3] = xyz
    if not np.all(np.isfinite(transform)):
        raise ValueError("origin transform is not finite")
    return transform


def _transform_vertices(vertices: np.ndarray, transform: np.ndarray) -> np.ndarray:
    result = vertices @ transform[:3, :3].T + transform[:3, 3]
    if not np.all(np.isfinite(result)):
        raise ValueError("transformed vertices are not finite")
    return np.asarray(result, dtype=np.float64)


def _tree_transforms(
    link_names: Sequence[str], joint_elements: Sequence[ET.Element]
) -> tuple[dict[str, np.ndarray], str | None]:
    link_set = set(link_names)
    children: set[str] = set()
    outgoing: dict[str, list[tuple[str, np.ndarray]]] = defaultdict(list)
    joint_names: set[str] = set()
    for index, joint in enumerate(joint_elements):
        name = joint.get("name") or f"__unnamed_joint_{index}"
        if name in joint_names:
            return {}, "duplicate joint name"
        joint_names.add(name)
        joint_type = joint.get("type", "")
        if joint_type not in {"fixed", "revolute", "continuous", "prismatic"}:
            return {}, f"unsupported q0 joint type: {joint_type or '<empty>'}"
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = None if parent_node is None else parent_node.get("link")
        child = None if child_node is None else child_node.get("link")
        if parent not in link_set or child not in link_set or parent == child:
            return {}, "joint references invalid parent or child"
        if child in children:
            return {}, "link has multiple parent joints"
        children.add(str(child))
        try:
            origin = _origin_transform(joint.find("origin"))
        except (TypeError, ValueError) as error:
            return {}, f"joint {name} origin invalid: {error}"
        # At q=0, revolute/continuous/prismatic motion is the identity transform.
        outgoing[str(parent)].append((str(child), origin))
    roots = [link for link in link_names if link not in children]
    if len(roots) != 1:
        return {}, f"expected one root link, found {len(roots)}"
    transforms = {roots[0]: np.eye(4, dtype=np.float64)}
    queue: deque[str] = deque((roots[0],))
    while queue:
        parent = queue.popleft()
        for child, origin in outgoing.get(parent, []):
            if child in transforms:
                return {}, "joint graph contains a cycle"
            transforms[child] = transforms[parent] @ origin
            if not np.all(np.isfinite(transforms[child])):
                return {}, "q0 FK produced a non-finite transform"
            queue.append(child)
    if set(transforms) != link_set:
        return {}, "joint graph is disconnected or cyclic"
    return transforms, None


def _safe_mesh_path(raw: str | None, package_root: Path) -> Path:
    if not raw:
        raise ValueError("mesh filename is empty")
    if "\x00" in raw or "\\" in raw:
        raise ValueError("mesh filename contains a forbidden character")
    lowered = raw.lower()
    if "://" in raw or lowered.startswith(("package:", "file:", "model:")):
        raise ValueError("mesh URI schemes are unsupported")
    relative = Path(raw)
    if relative.is_absolute():
        raise ValueError("absolute mesh filename is forbidden")
    candidate = package_root.joinpath(relative)
    if candidate.is_symlink():
        raise ValueError("mesh symlinks are forbidden")
    try:
        resolved = candidate.resolve(strict=True)
    except FileNotFoundError as error:
        raise ValueError("mesh resource does not exist") from error
    try:
        resolved.relative_to(package_root)
    except ValueError as error:
        raise ValueError("mesh filename escapes the package root") from error
    if not resolved.is_file():
        raise ValueError("mesh resource is not a regular file")
    if resolved.suffix.lower() != ".obj":
        raise ValueError("only local OBJ meshes are supported")
    if resolved.stat().st_size > MAX_OBJ_BYTES:
        raise ValueError("OBJ exceeds the frozen size limit")
    return resolved


def _parse_obj_face_index(token: str, vertex_count: int) -> int:
    head = token.split("/", 1)[0]
    if not head:
        raise ValueError("OBJ face has an empty vertex index")
    index = int(head)
    if index == 0:
        raise ValueError("OBJ indices are one-based and may not be zero")
    result = index - 1 if index > 0 else vertex_count + index
    if result < 0 or result >= vertex_count:
        raise ValueError("OBJ face vertex index is out of range")
    return result


def _mesh_topology_and_volume(
    vertices: np.ndarray, faces: np.ndarray
) -> tuple[bool, bool, float | None]:
    undirected: Counter[tuple[int, int]] = Counter()
    directed: Counter[tuple[int, int]] = Counter()
    for face in faces.tolist():
        for left, right in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            undirected[tuple(sorted((int(left), int(right))))] += 1
            directed[(int(left), int(right))] += 1
    watertight = bool(undirected) and all(count == 2 for count in undirected.values())
    winding_consistent = watertight and all(
        directed.get((left, right), 0) == directed.get((right, left), 0)
        for left, right in undirected
    )
    triangles = vertices[faces]
    signed = float(
        np.einsum(
            "ij,ij->i",
            triangles[:, 0],
            np.cross(triangles[:, 1], triangles[:, 2]),
        ).sum()
        / 6.0
    )
    volume = abs(signed)
    if not math.isfinite(volume) or volume <= 0.0:
        return watertight, winding_consistent, None
    return watertight, winding_consistent, volume


def _load_obj(path: Path, scale: np.ndarray, weld_rel_tol: float) -> dict[str, Any]:
    raw_vertices: list[list[float]] = []
    polygon_tokens: list[list[str]] = []
    with path.open("r", encoding="utf-8", errors="strict") as stream:
        for line_number, line in enumerate(stream, 1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if fields[0] == "v":
                if len(fields) < 4:
                    raise ValueError(f"OBJ vertex at line {line_number} has fewer than 3 values")
                vertex = [
                    _finite_float(fields[index], f"OBJ vertex line {line_number}")
                    for index in (1, 2, 3)
                ]
                raw_vertices.append(vertex)
            elif fields[0] == "f":
                if len(fields) < 4:
                    raise ValueError(f"OBJ face at line {line_number} has fewer than 3 vertices")
                polygon_tokens.append(fields[1:])
    if not raw_vertices:
        raise ValueError("OBJ has no vertices")
    if not polygon_tokens:
        raise ValueError("OBJ has no faces")
    vertices = np.asarray(raw_vertices, dtype=np.float64) * scale.reshape((1, 3))
    if not np.all(np.isfinite(vertices)):
        raise ValueError("scaled OBJ vertices are non-finite")
    raw_faces: list[tuple[int, int, int]] = []
    polygon_face_count = 0
    for tokens in polygon_tokens:
        indices = [_parse_obj_face_index(token, len(raw_vertices)) for token in tokens]
        polygon_face_count += int(len(indices) > 3)
        for offset in range(1, len(indices) - 1):
            raw_faces.append((indices[0], indices[offset], indices[offset + 1]))

    lower = np.min(vertices, axis=0)
    upper = np.max(vertices, axis=0)
    diagonal = float(np.linalg.norm(upper - lower))
    if not math.isfinite(diagonal) or diagonal <= 0.0:
        raise ValueError("OBJ vertex AABB diagonal is not positive finite")
    tolerance = weld_rel_tol * diagonal
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("OBJ weld tolerance is not positive finite")
    keys = np.rint((vertices - lower.reshape((1, 3))) / tolerance).astype(np.int64)
    welded_vertices: list[np.ndarray] = []
    key_to_index: dict[tuple[int, int, int], int] = {}
    old_to_new: list[int] = []
    for vertex, key_array in zip(vertices, keys, strict=True):
        key = tuple(int(value) for value in key_array)
        if key not in key_to_index:
            key_to_index[key] = len(welded_vertices)
            welded_vertices.append(vertex)
        old_to_new.append(key_to_index[key])
    welded = np.asarray(welded_vertices, dtype=np.float64)
    retained_faces: list[tuple[int, int, int]] = []
    degenerate_count = 0
    for raw_face in raw_faces:
        face = tuple(old_to_new[index] for index in raw_face)
        if len(set(face)) != 3:
            degenerate_count += 1
            continue
        triangle = welded[np.asarray(face, dtype=np.int64)]
        twice_area = float(np.linalg.norm(np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])))
        if not math.isfinite(twice_area) or twice_area <= tolerance * tolerance:
            degenerate_count += 1
            continue
        retained_faces.append(face)
    if not retained_faces:
        raise ValueError("OBJ has no non-degenerate triangles after welding")
    faces = np.asarray(retained_faces, dtype=np.int64)
    used = sorted({int(value) for value in faces.reshape(-1).tolist()})
    compact_map = {old: new for new, old in enumerate(used)}
    compact_vertices = welded[np.asarray(used, dtype=np.int64)]
    compact_faces = np.asarray(
        [[compact_map[int(value)] for value in face] for face in faces.tolist()],
        dtype=np.int64,
    )
    watertight, winding_consistent, volume = _mesh_topology_and_volume(
        compact_vertices, compact_faces
    )
    return {
        "vertices": compact_vertices,
        "faces": compact_faces,
        "triangle_count": int(len(compact_faces)),
        "watertight": bool(watertight),
        "winding_consistent": bool(winding_consistent),
        "volume": volume,
        "finite_positive_volume": volume is not None,
        "validation": {
            "protocol": OBJ_PROTOCOL,
            "source_vertex_count": len(raw_vertices),
            "source_polygon_count": len(polygon_tokens),
            "fan_triangulated_polygon_count": polygon_face_count,
            "triangles_before_degenerate_filter": len(raw_faces),
            "degenerate_triangle_count": degenerate_count,
            "valid_triangle_count": int(len(compact_faces)),
            "welded_vertex_count": int(len(compact_vertices)),
            "weld_relative_tolerance": float(weld_rel_tol),
            "weld_absolute_tolerance": float(tolerance),
        },
    }


def _box_surface(size: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    half = size / 2.0
    vertices = np.asarray(
        [
            (-half[0], -half[1], -half[2]),
            (half[0], -half[1], -half[2]),
            (half[0], half[1], -half[2]),
            (-half[0], half[1], -half[2]),
            (-half[0], -half[1], half[2]),
            (half[0], -half[1], half[2]),
            (half[0], half[1], half[2]),
            (-half[0], half[1], half[2]),
        ],
        dtype=np.float64,
    )
    faces = np.asarray(
        [
            (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
            (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
            (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7),
        ],
        dtype=np.int64,
    )
    return vertices, faces


def _load_geometry_element(
    *,
    role: str,
    link_name: str,
    element_index: int,
    container: ET.Element,
    link_transform: np.ndarray,
    package_root: Path,
    weld_rel_tol: float,
) -> GeometryShape:
    geometry = container.find("geometry")
    if geometry is None:
        raise ValueError("geometry node is missing")
    children = list(geometry)
    if len(children) != 1:
        raise ValueError("geometry must contain exactly one shape")
    shape_node = children[0]
    kind = shape_node.tag
    if kind not in ANALYTIC_KINDS and kind != "mesh":
        raise ValueError(f"unsupported geometry kind: {kind}")
    world = link_transform @ _origin_transform(container.find("origin"))
    mesh_path: str | None = None
    vertices: np.ndarray | None = None
    faces: np.ndarray | None = None
    triangle_count = 0
    watertight = True
    winding_consistent = True
    finite_positive_volume = True
    volume: float | None
    validation: dict[str, Any]
    surface_exact_eligible = True

    if kind == "mesh":
        resolved = _safe_mesh_path(shape_node.get("filename"), package_root)
        mesh_path = resolved.relative_to(package_root).as_posix()
        scale = _vector(shape_node.get("scale"), (1.0, 1.0, 1.0), "mesh scale")
        if np.any(scale == 0.0):
            raise ValueError("mesh scale components must be non-zero")
        loaded = _load_obj(resolved, scale, weld_rel_tol)
        vertices = _transform_vertices(loaded["vertices"], world)
        faces = loaded["faces"]
        triangle_count = int(loaded["triangle_count"])
        watertight = bool(loaded["watertight"])
        winding_consistent = bool(loaded["winding_consistent"])
        finite_positive_volume = bool(loaded["finite_positive_volume"])
        volume = loaded["volume"]
        validation = loaded["validation"]
        bounds_min = np.min(vertices, axis=0)
        bounds_max = np.max(vertices, axis=0)
    elif kind == "box":
        size = _vector(shape_node.get("size"), (), "box size")
        if np.any(size <= 0.0):
            raise ValueError("box size components must be positive")
        local_vertices, faces = _box_surface(size)
        vertices = _transform_vertices(local_vertices, world)
        triangle_count = 0  # Analytic primitives never enter collision mesh triangles.
        volume = float(np.prod(size))
        validation = {"protocol": "analytic_box_exact_v1", "size": size.tolist()}
        bounds_min = np.min(vertices, axis=0)
        bounds_max = np.max(vertices, axis=0)
    elif kind == "sphere":
        radius = _finite_float(shape_node.get("radius", "nan"), "sphere radius")
        if radius <= 0.0:
            raise ValueError("sphere radius must be positive")
        center = world[:3, 3]
        bounds_min = center - radius
        bounds_max = center + radius
        volume = float(4.0 * math.pi * radius ** 3 / 3.0)
        validation = {"protocol": "analytic_sphere_exact_v1", "radius": radius}
        # No tessellation is allowed to masquerade as exact sphere proximity.
        surface_exact_eligible = False
    else:
        radius = _finite_float(shape_node.get("radius", "nan"), "cylinder radius")
        length = _finite_float(shape_node.get("length", "nan"), "cylinder length")
        if radius <= 0.0 or length <= 0.0:
            raise ValueError("cylinder radius and length must be positive")
        rotation = world[:3, :3]
        radial_extent = radius * np.sqrt(rotation[:, 0] ** 2 + rotation[:, 1] ** 2)
        extent = radial_extent + (length / 2.0) * np.abs(rotation[:, 2])
        center = world[:3, 3]
        bounds_min = center - extent
        bounds_max = center + extent
        volume = float(math.pi * radius ** 2 * length)
        validation = {
            "protocol": "analytic_cylinder_exact_bounds_volume_v1",
            "radius": radius,
            "length": length,
        }
        # No tessellation is allowed to masquerade as exact cylinder proximity.
        surface_exact_eligible = False

    if not np.all(np.isfinite(bounds_min)) or not np.all(np.isfinite(bounds_max)):
        raise ValueError("geometry bounds are non-finite")
    if np.any(bounds_max < bounds_min):
        raise ValueError("geometry bounds are inverted")
    return GeometryShape(
        role=role,
        link_name=link_name,
        element_index=element_index,
        kind=kind,
        mesh_path=mesh_path,
        vertices=vertices,
        faces=faces,
        bounds_min=np.asarray(bounds_min, dtype=np.float64),
        bounds_max=np.asarray(bounds_max, dtype=np.float64),
        triangle_count=triangle_count,
        watertight=watertight,
        winding_consistent=winding_consistent,
        finite_positive_volume=finite_positive_volume,
        volume=volume,
        surface_exact_eligible=surface_exact_eligible,
        validation=validation,
    )


def load_urdf_geometry(
    urdf_path: str | Path,
    *,
    weld_rel_tol: float = 1e-9,
    roles: Sequence[str] = ("visual", "collision"),
) -> GeometryInventory:
    """Safely load selected local q=0 geometry roles from one URDF package."""

    if not math.isfinite(weld_rel_tol) or weld_rel_tol <= 0.0:
        raise ValueError("weld_rel_tol must be positive finite")
    selected_roles = tuple(roles)
    if (
        not selected_roles
        or len(set(selected_roles)) != len(selected_roles)
        or any(role not in {"visual", "collision"} for role in selected_roles)
    ):
        raise ValueError("roles must be a non-empty unique subset of visual/collision")
    path = Path(urdf_path).resolve(strict=True)
    if not path.is_file():
        raise ValueError("URDF path is not a regular file")
    if path.stat().st_size > MAX_URDF_BYTES:
        raise ValueError("URDF exceeds the frozen size limit")
    package_root = path.parent.resolve(strict=True)
    inventory = GeometryInventory(path, package_root, weld_rel_tol)
    payload = path.read_bytes()
    upper_payload = payload.upper()
    if b"<!DOCTYPE" in upper_payload or b"<!ENTITY" in upper_payload:
        raise ValueError("URDF DTD/entity declarations are forbidden")
    root = ET.fromstring(payload)
    if root.tag != "robot":
        raise ValueError("URDF root is not <robot>")
    links = root.findall("link")
    link_names = [link.get("name", "") for link in links]
    inventory.link_names = link_names
    if not link_names or any(not name for name in link_names):
        inventory.issues.append("link names are empty or absent")
        return inventory
    if len(set(link_names)) != len(link_names):
        inventory.issues.append("link names are duplicated")
        return inventory

    for link in links:
        name = str(link.get("name"))
        for role in selected_roles:
            for container in link.findall(role):
                if container.find("geometry") is None:
                    continue
                inventory.declared_counts[role] += 1
                if role == "visual":
                    inventory.visual_bearing_links.add(name)
                else:
                    inventory.declared_collision_bearing_links.add(name)

    transforms, tree_issue = _tree_transforms(link_names, root.findall("joint"))
    if tree_issue is not None:
        inventory.issues.append(f"tree/q0 FK unavailable: {tree_issue}")
        return inventory
    inventory.tree_valid = True
    inventory.link_transforms = transforms
    for link in links:
        name = str(link.get("name"))
        for role in selected_roles:
            containers = [
                container
                for container in link.findall(role)
                if container.find("geometry") is not None
            ]
            for element_index, container in enumerate(containers):
                try:
                    shape = _load_geometry_element(
                        role=role,
                        link_name=name,
                        element_index=element_index,
                        container=container,
                        link_transform=transforms[name],
                        package_root=package_root,
                        weld_rel_tol=weld_rel_tol,
                    )
                    inventory.shapes.append(shape)
                except (OSError, UnicodeError, TypeError, ValueError) as error:
                    inventory.issues.append(
                        f"{role} geometry {name}[{element_index}] unavailable: {error}"
                    )
    return inventory


def _fresh_collision_bvh_construction(
    urdf_path: str | Path,
    weld_rel_tol: float,
    trimesh_module: Any,
) -> list[tuple[Any, Any]]:
    """Build one fresh collision-only trimesh and exact triangle tree per shape."""

    inventory = load_urdf_geometry(
        urdf_path,
        weld_rel_tol=weld_rel_tol,
        roles=("collision",),
    )
    if not inventory.tree_valid:
        reason = "; ".join(inventory.issues) or "URDF tree/q0 transforms are unavailable"
        raise ValueError(reason)
    collision_shapes = inventory.role_shapes("collision")
    declared = int(inventory.declared_counts["collision"])
    if declared == 0:
        raise ValueError("URDF declares no collision geometry")
    if len(collision_shapes) != declared:
        reason = "; ".join(inventory.issues) or "collision geometry closure is incomplete"
        raise ValueError(
            f"loaded {len(collision_shapes)}/{declared} collision elements: {reason}"
        )

    constructions: list[tuple[Any, Any]] = []
    for shape in collision_shapes:
        if shape.vertices is None or shape.faces is None:
            raise ValueError(
                f"collision {shape.link_name}[{shape.element_index}] {shape.kind} "
                "has no exact triangle-BVH representation"
            )
        mesh = trimesh_module.Trimesh(
            vertices=np.array(shape.vertices, dtype=np.float64, copy=True),
            faces=np.array(shape.faces, dtype=np.int64, copy=True),
            process=False,
            validate=False,
        )
        if len(mesh.faces) != len(shape.faces) or len(mesh.faces) == 0:
            raise ValueError("trimesh process=False construction changed or lost faces")
        if not np.all(np.isfinite(mesh.vertices)):
            raise ValueError("trimesh process=False construction is non-finite")
        tree = mesh.triangles_tree
        if len(tree) != len(mesh.faces):
            raise ValueError("exact triangle R-tree does not cover every collision face")
        # Force a real index query so lazy wrappers cannot masquerade as a built tree.
        if int(tree.count(tuple(tree.bounds))) != len(mesh.faces):
            raise ValueError("exact triangle R-tree full-bounds query is incomplete")
        constructions.append((mesh, tree))
    return constructions


def _collision_load_time_record(urdf_path: str | Path, weld_rel_tol: float) -> dict[str, Any]:
    return {
        "protocol": COLLISION_LOAD_TIME_PROTOCOL_ID,
        "backend": COLLISION_LOAD_TIME_BACKEND,
        "status": "N/E",
        "value": None,
        "unit": "seconds",
        "repeats": COLLISION_LOAD_TIME_MEASURED_REPEATS,
        "completed_repeats": 0,
        "warmup_repeats": COLLISION_LOAD_TIME_WARMUP_REPEATS,
        "raw_times_seconds": [],
        "reason": None,
        "thread_count": COLLISION_LOAD_TIME_THREADS,
        "cache_mode": "same_asset_one_unmeasured_warmup_then_warm_cache",
        "process_isolation": "caller_enforced_one_asset_per_child",
        "fresh_geometry_construction_per_repeat": True,
        "timed_scope": (
            "URDF XML read/parse, local collision-path resolution, frozen collision "
            "geometry parsing, Trimesh(process=False), and per-shape exact triangle R-tree build"
        ),
        "excluded_scope": "process startup, imports, warmup, hashing, result serialization/I/O",
        "weld_relative_tolerance": float(weld_rel_tol),
        "urdf_path": str(urdf_path),
        "no_visual_fallback": True,
        "no_simulator_fallback": True,
    }


def measure_collision_load_time_in_asset_child(
    urdf_path: str | Path,
    *,
    weld_rel_tol: float = 1e-9,
) -> dict[str, Any]:
    """Measure collision construction in an already isolated one-asset child.

    The protocol fixes one unmeasured warmup followed by five fresh measured
    constructions.  The caller must create one process per asset; process startup
    and result transport therefore remain outside the timer by construction.
    """

    result = _collision_load_time_record(urdf_path, weld_rel_tol)
    if not math.isfinite(weld_rel_tol) or weld_rel_tol <= 0.0:
        result["status"] = "FAILED"
        result["reason"] = "weld_rel_tol must be positive finite"
        return result
    try:
        import rtree
        import trimesh
        from threadpoolctl import threadpool_info, threadpool_limits
    except (ImportError, OSError) as error:
        result["reason"] = f"required exact timing backend unavailable: {type(error).__name__}: {error}"
        return result

    result["runtime"] = {
        "trimesh_version": str(getattr(trimesh, "__version__", "unknown")),
        "rtree_version": str(getattr(rtree, "__version__", "unknown")),
        "threadpoolctl": True,
    }
    raw_times: list[float] = []
    try:
        with threadpool_limits(limits=COLLISION_LOAD_TIME_THREADS):
            controllers = threadpool_info()
            noncompliant = [
                controller
                for controller in controllers
                if int(controller.get("num_threads", COLLISION_LOAD_TIME_THREADS))
                != COLLISION_LOAD_TIME_THREADS
            ]
            if noncompliant:
                raise RuntimeError("threadpoolctl could not enforce native thread_count=1")
            result["threadpool_controllers"] = [
                {
                    "user_api": str(controller.get("user_api", "unknown")),
                    "internal_api": str(controller.get("internal_api", "unknown")),
                    "num_threads": int(
                        controller.get("num_threads", COLLISION_LOAD_TIME_THREADS)
                    ),
                }
                for controller in controllers
            ]

            warmup = _fresh_collision_bvh_construction(
                urdf_path, weld_rel_tol, trimesh
            )
            del warmup
            for _ in range(COLLISION_LOAD_TIME_MEASURED_REPEATS):
                started_ns = time.perf_counter_ns()
                construction = _fresh_collision_bvh_construction(
                    urdf_path, weld_rel_tol, trimesh
                )
                elapsed = (time.perf_counter_ns() - started_ns) / 1_000_000_000.0
                del construction
                if not math.isfinite(elapsed) or elapsed <= 0.0:
                    raise RuntimeError("collision construction timer was not positive finite")
                raw_times.append(float(elapsed))
                result["completed_repeats"] = len(raw_times)
                result["raw_times_seconds"] = list(raw_times)
    except Exception as error:  # Per-asset fail-closed timing boundary.
        result["status"] = "FAILED"
        result["reason"] = f"{type(error).__name__}: {error}"
        return result

    result.update(
        {
            "status": "COMPLETE",
            "value": float(statistics.median(raw_times)),
            "completed_repeats": len(raw_times),
            "raw_times_seconds": raw_times,
            "reason": None,
        }
    )
    return result


def collision_triangle_validation_measurement(
    geometry_record: Mapping[str, Any],
) -> dict[str, Any]:
    """Return an all-or-nothing per-asset collision-mesh triangle measurement."""

    intended = geometry_record.get("collision_mesh_validation_intended")
    measured = geometry_record.get("collision_mesh_validation_measured")
    triangles = geometry_record.get("collision_mesh_valid_triangle_count")
    declared_collision = geometry_record.get("declared_collision_element_count")
    loadable_collision = geometry_record.get("loadable_collision_element_count")
    base = {
        "status": "N/E",
        "value": None,
        "intended_mesh_count": intended,
        "measured_mesh_count": measured,
        "reason": None,
    }
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value < 0
        for value in (
            intended,
            measured,
            triangles,
            declared_collision,
            loadable_collision,
        )
    ):
        base.update(status="FAILED", reason="triangle validation counts are malformed")
        return base
    if not geometry_record.get("tree_valid"):
        base["reason"] = "URDF tree/q0 geometry extraction was not complete"
    elif declared_collision == 0:
        base["reason"] = "asset declares no collision geometry"
    elif loadable_collision != declared_collision:
        base["reason"] = (
            f"loaded {loadable_collision}/{declared_collision} declared collision elements"
        )
    elif measured > intended:
        base.update(status="FAILED", reason="measured collision meshes exceed intended meshes")
    elif measured != intended:
        base["reason"] = f"validated {measured}/{intended} declared collision meshes"
    else:
        base.update(status="COMPLETE", value=int(triangles), reason=None)
    return base


def collision_redundancy_measurement(
    geometry_record: Mapping[str, Any],
) -> dict[str, Any]:
    """Return volume redundancy with explicit per-link coverage."""

    intended = geometry_record.get("redundancy_intended_collision_bearing_link_count")
    measured = geometry_record.get("redundancy_measured_link_count")
    declared_links = geometry_record.get("declared_collision_bearing_link_count")
    declared_collision = geometry_record.get("declared_collision_element_count")
    loadable_collision = geometry_record.get("loadable_collision_element_count")
    base = {
        "status": "N/E",
        "value": None,
        "shape_volume_m3": None,
        "redundant_volume_m3": None,
        "intended_link_count": intended,
        "measured_link_count": measured,
        "reason": None,
    }
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value < 0
        for value in (
            intended,
            measured,
            declared_links,
            declared_collision,
            loadable_collision,
        )
    ):
        base.update(status="FAILED", reason="redundancy coverage counts are malformed")
        return base
    if not geometry_record.get("tree_valid"):
        base["reason"] = "URDF tree/q0 geometry extraction was not complete"
        base["measured_link_count"] = 0
        return base
    if declared_collision == 0:
        base["reason"] = "asset declares no collision geometry"
        base["measured_link_count"] = 0
        return base
    if loadable_collision != declared_collision:
        base["reason"] = (
            f"loaded {loadable_collision}/{declared_collision} declared collision elements"
        )
        base["measured_link_count"] = 0
        return base
    if intended != declared_links:
        base["reason"] = (
            f"redundancy link extraction covers {intended}/{declared_links} declared links"
        )
        base["measured_link_count"] = 0
        return base
    if intended == 0:
        base["reason"] = "asset has no loadable collision-bearing link"
        base["measured_link_count"] = 0
        return base
    records = geometry_record.get("redundancy_link_records")
    if not isinstance(records, list) or len(records) != intended:
        base.update(status="FAILED", reason="redundancy link records do not close")
        return base
    shape_volume = 0.0
    redundant_volume = 0.0
    complete_count = 0
    for record in records:
        if not isinstance(record, Mapping):
            base.update(status="FAILED", reason="a redundancy link record is malformed")
            return base
        if record.get("status") != "COMPLETE":
            if record.get("status") not in {"N/E", "FAILED", "UNMEASURED"}:
                base.update(status="FAILED", reason="a redundancy link record has an invalid status")
                return base
            if any(record.get(field) is not None for field in ("sum_eligible_shape_volume", "union_volume", "redundancy")):
                base.update(status="FAILED", reason="an incomplete redundancy link record has numeric values")
                return base
            continue
        summed = record.get("sum_eligible_shape_volume")
        union = record.get("union_volume")
        if (
            isinstance(summed, bool)
            or isinstance(union, bool)
            or not isinstance(summed, (int, float))
            or not isinstance(union, (int, float))
            or not math.isfinite(float(summed))
            or not math.isfinite(float(union))
            or float(summed) <= 0.0
            or float(union) < 0.0
            or float(union) > float(summed) + 1e-15
        ):
            base.update(status="FAILED", reason="a redundancy volume record is malformed")
            return base
        complete_count += 1
        shape_volume += float(summed)
        redundant_volume += max(0.0, float(summed) - float(union))
    if complete_count != measured:
        base.update(status="FAILED", reason=f"redundancy measured count is {measured}, records contain {complete_count} complete links")
        return base
    if complete_count == 0:
        base["reason"] = f"exact union-volume coverage is {measured}/{intended} links"
        base["measured_link_count"] = 0
        return base
    if shape_volume <= 0.0 or not math.isfinite(shape_volume):
        base.update(status="FAILED", reason="redundancy denominator is not positive finite")
        return base
    if measured == intended and geometry_record.get("intra_link_redundancy_status") != "COMPLETE":
        base.update(status="FAILED", reason="complete redundancy count has a non-COMPLETE source status")
        return base
    if measured < intended and geometry_record.get("intra_link_redundancy_status") != "PARTIAL":
        base.update(status="FAILED", reason="partial redundancy count has a non-PARTIAL source status")
        return base
    base.update(
        status="COMPLETE" if measured == intended else "PARTIAL",
        value=float(redundant_volume / shape_volume),
        shape_volume_m3=float(shape_volume),
        redundant_volume_m3=float(redundant_volume),
        reason=None,
    )
    return base


def seed_sha256(protocol_id: str, asset_key: str, direction: str) -> str:
    return hashlib.sha256(
        f"{protocol_id}|{asset_key}|{direction}".encode("utf-8")
    ).hexdigest()


def pcg64dxsm_rng(protocol_id: str, asset_key: str, direction: str) -> np.random.Generator:
    digest = seed_sha256(protocol_id, asset_key, direction)
    seed = int(digest, 16)
    return np.random.Generator(np.random.PCG64DXSM(seed))


def _surface_union(shapes: Sequence[GeometryShape]) -> tuple[np.ndarray, np.ndarray]:
    vertices: list[np.ndarray] = []
    faces: list[np.ndarray] = []
    offset = 0
    for shape in shapes:
        if not shape.surface_exact_eligible or shape.vertices is None or shape.faces is None:
            raise ValueError(
                f"{shape.kind} geometry has no exact triangle-surface representation"
            )
        vertices.append(shape.vertices)
        faces.append(shape.faces + offset)
        offset += int(len(shape.vertices))
    if not vertices or not faces:
        raise ValueError("surface union is empty")
    return np.vstack(vertices), np.vstack(faces)


def sample_surface_area_weighted(
    vertices: np.ndarray,
    faces: np.ndarray,
    sample_count: int,
    rng: np.random.Generator,
) -> np.ndarray:
    """Sample a triangle union by exact triangle area using PCG64DXSM draws."""

    if sample_count <= 0:
        raise ValueError("sample_count must be positive")
    triangles = np.asarray(vertices, dtype=np.float64)[np.asarray(faces, dtype=np.int64)]
    twice_area = np.linalg.norm(
        np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]),
        axis=1,
    )
    area = twice_area / 2.0
    if not np.all(np.isfinite(area)) or np.any(area <= 0.0):
        raise ValueError("surface contains non-positive or non-finite triangle area")
    cumulative = np.cumsum(area, dtype=np.float64)
    total = float(cumulative[-1])
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError("surface area is not positive finite")
    selected = np.searchsorted(cumulative, rng.random(sample_count) * total, side="right")
    chosen = triangles[selected]
    first = rng.random(sample_count)
    second = rng.random(sample_count)
    root = np.sqrt(first)
    result = (
        (1.0 - root)[:, None] * chosen[:, 0]
        + (root * (1.0 - second))[:, None] * chosen[:, 1]
        + (root * second)[:, None] * chosen[:, 2]
    )
    if not np.all(np.isfinite(result)):
        raise ValueError("surface sampler produced non-finite coordinates")
    return result


def _exact_backend_qualification_uncached() -> dict[str, Any]:
    result: dict[str, Any] = {
        "backend": EXACT_BACKEND,
        "available": False,
        "status": "N/E",
        "no_kdtree_fallback": True,
    }
    try:
        import rtree
        import trimesh

        vertices = np.asarray(((0, 0, 0), (1, 0, 0), (0, 1, 0)), dtype=np.float64)
        faces = np.asarray(((0, 1, 2),), dtype=np.int64)
        points = np.asarray(((0.25, 0.25, 2.0), (2.0, 0.0, 0.0)), dtype=np.float64)
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        query = trimesh.proximity.ProximityQuery(mesh)
        _, distances, _ = query.on_surface(points)
        observed = np.asarray(distances, dtype=np.float64)
        if not np.allclose(observed, np.asarray((2.0, 1.0)), rtol=0.0, atol=1e-12):
            raise RuntimeError(f"known-distance smoke mismatch: {observed.tolist()}")
        result.update(
            {
                "available": True,
                "status": "QUALIFIED",
                "trimesh_version": str(trimesh.__version__),
                "rtree_version": str(rtree.__version__),
                "qualification": "two_known_triangle_distances_pass",
            }
        )
    except Exception as error:  # Qualification must convert dependency/runtime failures to N/E.
        result["reason"] = f"{type(error).__name__}: {error}"
    return result


@lru_cache(maxsize=1)
def qualify_exact_surface_backend() -> dict[str, Any]:
    """Qualify rtree-backed exact point-to-triangle distance; never approximate."""

    return dict(_exact_backend_qualification_uncached())


def exact_closest_surface_distances(
    points: np.ndarray,
    target_vertices: np.ndarray,
    target_faces: np.ndarray,
    *,
    chunk_size: int = DEFAULT_CLOSEST_CHUNK,
) -> dict[str, Any]:
    """Return exact point-to-triangle distances or an explicit unavailable record."""

    qualification = qualify_exact_surface_backend()
    if not qualification.get("available"):
        return {
            "status": "N/E",
            "available": False,
            "backend": qualification,
            "distances": None,
        }
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    import trimesh

    query_points = np.asarray(points, dtype=np.float64)
    if query_points.ndim != 2 or query_points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if not np.all(np.isfinite(query_points)):
        raise ValueError("query points are non-finite")
    mesh = trimesh.Trimesh(
        vertices=np.asarray(target_vertices, dtype=np.float64),
        faces=np.asarray(target_faces, dtype=np.int64),
        process=False,
    )
    chunks: list[np.ndarray] = []
    for start in range(0, len(query_points), chunk_size):
        stop = min(len(query_points), start + chunk_size)
        _, distances, _ = trimesh.proximity.ProximityQuery(mesh).on_surface(
            query_points[start:stop]
        )
        chunk = np.asarray(distances, dtype=np.float64)
        if not np.all(np.isfinite(chunk)) or np.any(chunk < 0.0):
            raise RuntimeError("exact surface backend returned invalid distances")
        chunks.append(chunk)
    distances = np.concatenate(chunks) if chunks else np.empty((0,), dtype=np.float64)
    return {
        "status": "COMPLETE",
        "available": True,
        "backend": qualification,
        "distances": distances.tolist(),
    }


def _surface_direction(
    *,
    source_shapes: Sequence[GeometryShape],
    target_shapes: Sequence[GeometryShape],
    direction: str,
    protocol_id: str,
    asset_key: str,
    sample_count: int,
    d_visual: float | None,
) -> dict[str, Any]:
    seed = seed_sha256(protocol_id, asset_key, direction)
    base = {
        "direction": direction,
        "seed_protocol": SEED_PROTOCOL,
        "seed_sha256": seed,
        "sample_count_intended": int(sample_count),
        "sample_count_executed": 0,
        "raw_p95_m": None,
        "normalized_p95": None,
        "status": "N/E",
    }
    qualification = qualify_exact_surface_backend()
    if not qualification.get("available"):
        base["reason"] = "exact surface backend unavailable"
        base["backend"] = qualification
        return base
    if d_visual is None or not math.isfinite(d_visual) or d_visual <= 0.0:
        base["reason"] = "D_visual is unavailable or non-positive"
        base["backend"] = qualification
        return base
    try:
        source_vertices, source_faces = _surface_union(source_shapes)
        target_vertices, target_faces = _surface_union(target_shapes)
        samples = sample_surface_area_weighted(
            source_vertices,
            source_faces,
            sample_count,
            pcg64dxsm_rng(protocol_id, asset_key, direction),
        )
        exact = exact_closest_surface_distances(samples, target_vertices, target_faces)
        if not exact.get("available"):
            base["reason"] = "exact surface backend became unavailable"
            base["backend"] = exact.get("backend")
            return base
        distances = np.asarray(exact["distances"], dtype=np.float64)
        raw_p95 = float(np.percentile(distances, 95.0, method="linear"))
        base.update(
            {
                "sample_count_executed": int(len(distances)),
                "raw_p95_m": raw_p95,
                "normalized_p95": float(raw_p95 / d_visual),
                "status": "COMPLETE",
                "backend": qualification,
                "percentile": "numpy_linear_p95_v1",
            }
        )
    except (ImportError, OSError, RuntimeError, TypeError, ValueError) as error:
        base["reason"] = f"{type(error).__name__}: {error}"
        base["backend"] = qualification
    return base


def _volume_and_redundancy(inventory: GeometryInventory) -> dict[str, Any]:
    collision_shapes = inventory.role_shapes("collision")
    by_link: dict[str, list[GeometryShape]] = defaultdict(list)
    for shape in collision_shapes:
        by_link[shape.link_name].append(shape)
    records: list[dict[str, Any]] = []
    measured_sum_volume = 0.0
    measured_overlap_volume = 0.0
    for link_name in sorted(by_link):
        shapes = by_link[link_name]
        eligible = [
            shape
            for shape in shapes
            if shape.finite_positive_volume
            and shape.volume is not None
            and (shape.kind in ANALYTIC_KINDS or shape.watertight)
        ]
        sum_volume = float(sum(float(shape.volume) for shape in eligible))
        record: dict[str, Any] = {
            "link_name": link_name,
            "shape_count": len(shapes),
            "volume_eligible_shape_count": len(eligible),
            "sum_eligible_shape_volume": sum_volume if eligible else None,
            "union_volume": None,
            "redundancy": None,
            "status": "N/E",
        }
        if len(eligible) != len(shapes):
            record["reason"] = "one or more shapes lack watertight finite-positive volume"
        elif len(shapes) == 1 and sum_volume > 0.0:
            # Atomic identity: union(G_1) == G_1, requiring no boolean approximation.
            record.update(
                {
                    "union_volume": sum_volume,
                    "redundancy": 0.0,
                    "status": "COMPLETE",
                    "protocol": "single_shape_union_identity_v1",
                }
            )
            measured_sum_volume += sum_volume
        elif len(shapes) > 1:
            record["reason"] = "qualified exact union-volume backend not configured"
        records.append(record)
    complete = [record for record in records if record["status"] == "COMPLETE"]
    redundancy = (
        measured_overlap_volume / measured_sum_volume
        if measured_sum_volume > 0.0
        else None
    )
    if records and len(complete) == len(records):
        status = "COMPLETE"
    elif complete:
        status = "PARTIAL"
    else:
        status = "N/E"
    return {
        "collision_volume_eligible_shape_count": sum(
            record["volume_eligible_shape_count"] for record in records
        ),
        "collision_volume_intended_shape_count": len(collision_shapes),
        "collision_volume_coverage": (
            sum(record["volume_eligible_shape_count"] for record in records)
            / len(collision_shapes)
            if collision_shapes
            else None
        ),
        "intra_link_redundancy": redundancy,
        "intra_link_redundancy_status": status,
        "redundancy_measured_link_count": len(complete),
        "redundancy_intended_collision_bearing_link_count": len(records),
        "redundancy_link_records": records,
    }


def _base_evaluation_record(
    urdf_path: str | Path,
    asset_key: str,
    protocol_id: str,
    samples_per_direction: int,
    weld_rel_tol: float,
) -> dict[str, Any]:
    return {
        "geometry_protocol_id": GEOMETRY_PROTOCOL_ID,
        "protocol_id": protocol_id,
        "asset_key": asset_key,
        "urdf_path": str(urdf_path),
        "status": "N/E",
        "issues": [],
        "tree_valid": False,
        "declared_link_count": 0,
        "declared_visual_element_count": 0,
        "loadable_visual_element_count": 0,
        "declared_collision_element_count": 0,
        "loadable_collision_element_count": 0,
        "visual_bearing_link_count": 0,
        "declared_collision_bearing_link_count": 0,
        "d_visual": None,
        "d_visual_status": "N/E",
        "analytic_collision_element_count": 0,
        "collision_mesh_element_count": 0,
        "analytic_collision_share": None,
        "collision_shapes_per_visual_bearing_link": None,
        "collision_mesh_valid_triangle_count": 0,
        "collision_mesh_validation_measured": 0,
        "collision_mesh_validation_intended": 0,
        "collision_mesh_validation_status": "N/E",
        "collision_watertight_mesh_count": 0,
        "collision_finite_positive_volume_mesh_count": 0,
        "surface_samples_per_direction": int(samples_per_direction),
        "weld_relative_tolerance": float(weld_rel_tol),
        "exact_surface_backend": qualify_exact_surface_backend(),
        "visual_to_collision": {"status": "N/E"},
        "collision_to_visual": {"status": "N/E"},
    }


def evaluate_table4b_geometry(
    urdf_path: str | Path,
    asset_key: str,
    protocol_id: str,
    samples_per_direction: int = 32768,
    weld_rel_tol: float = 1e-9,
) -> dict[str, Any]:
    """Evaluate one asset's Table 4b geometry fields without hiding failures.

    The return value is fully JSON serializable.  Parsing or resource failures are
    retained in ``issues`` and do not throw away the asset-level record.
    """

    if samples_per_direction <= 0:
        raise ValueError("samples_per_direction must be positive")
    result = _base_evaluation_record(
        urdf_path, asset_key, protocol_id, samples_per_direction, weld_rel_tol
    )
    try:
        inventory = load_urdf_geometry(urdf_path, weld_rel_tol=weld_rel_tol)
    except Exception as error:  # Asset-level fail-closed boundary.
        result["issues"] = [f"URDF geometry load failed: {type(error).__name__}: {error}"]
        return result

    visual = inventory.role_shapes("visual")
    collision = inventory.role_shapes("collision")
    result.update(
        {
            "urdf_path": str(inventory.urdf_path),
            "issues": list(inventory.issues),
            "tree_valid": inventory.tree_valid,
            "declared_link_count": len(inventory.link_names),
            "declared_visual_element_count": inventory.declared_counts["visual"],
            "loadable_visual_element_count": len(visual),
            "declared_collision_element_count": inventory.declared_counts["collision"],
            "loadable_collision_element_count": len(collision),
            "visual_bearing_link_count": len(inventory.visual_bearing_links),
            "declared_collision_bearing_link_count": len(
                inventory.declared_collision_bearing_links
            ),
        }
    )
    if visual:
        lower = np.min(np.vstack([shape.bounds_min for shape in visual]), axis=0)
        upper = np.max(np.vstack([shape.bounds_max for shape in visual]), axis=0)
        diagonal = float(np.linalg.norm(upper - lower))
        if math.isfinite(diagonal) and diagonal > 0.0:
            result["d_visual"] = diagonal
            result["d_visual_bounds_min"] = lower.tolist()
            result["d_visual_bounds_max"] = upper.tolist()
            result["d_visual_status"] = (
                "COMPLETE"
                if len(visual) == inventory.declared_counts["visual"]
                else "PARTIAL"
            )
        else:
            result["issues"].append("D_visual is non-positive or non-finite")
    else:
        result["issues"].append("no loadable visual geometry for D_visual")

    analytic = [shape for shape in collision if shape.kind in ANALYTIC_KINDS]
    collision_meshes = [shape for shape in collision if shape.kind == "mesh"]
    result.update(
        {
            "analytic_collision_element_count": len(analytic),
            "collision_mesh_element_count": len(collision_meshes),
            "analytic_collision_share": len(analytic) / len(collision) if collision else None,
            "collision_shapes_per_visual_bearing_link": (
                len(collision) / len(inventory.visual_bearing_links)
                if inventory.visual_bearing_links
                else None
            ),
            "collision_mesh_valid_triangle_count": sum(
                shape.triangle_count for shape in collision_meshes
            ),
            "collision_mesh_validation_measured": len(collision_meshes),
            "collision_mesh_validation_intended": sum(
                1
                for link in ET.fromstring(inventory.urdf_path.read_bytes()).findall("link")
                for container in link.findall("collision")
                if (container.find("geometry/mesh") is not None)
            ),
            "collision_watertight_mesh_count": sum(
                int(shape.watertight) for shape in collision_meshes
            ),
            "collision_finite_positive_volume_mesh_count": sum(
                int(shape.finite_positive_volume) for shape in collision_meshes
            ),
            "collision_mesh_records": [
                {
                    "link_name": shape.link_name,
                    "element_index": shape.element_index,
                    "mesh_path": shape.mesh_path,
                    "valid_triangle_count": shape.triangle_count,
                    "watertight": shape.watertight,
                    "winding_consistent": shape.winding_consistent,
                    "finite_positive_volume": shape.finite_positive_volume,
                    "volume": shape.volume,
                    "validation": shape.validation,
                }
                for shape in collision_meshes
            ],
        }
    )
    intended_meshes = int(result["collision_mesh_validation_intended"])
    measured_meshes = int(result["collision_mesh_validation_measured"])
    if intended_meshes:
        result["collision_mesh_validation_status"] = (
            "COMPLETE" if measured_meshes == intended_meshes else "PARTIAL"
        )
    result.update(_volume_and_redundancy(inventory))

    result["visual_to_collision"] = _surface_direction(
        source_shapes=visual,
        target_shapes=collision,
        direction="visual_to_collision",
        protocol_id=protocol_id,
        asset_key=asset_key,
        sample_count=samples_per_direction,
        d_visual=result["d_visual"],
    )
    result["collision_to_visual"] = _surface_direction(
        source_shapes=collision,
        target_shapes=visual,
        direction="collision_to_visual",
        protocol_id=protocol_id,
        asset_key=asset_key,
        sample_count=samples_per_direction,
        d_visual=result["d_visual"],
    )
    declared_total = (
        inventory.declared_counts["visual"] + inventory.declared_counts["collision"]
    )
    loadable_total = len(visual) + len(collision)
    surfaces_complete = (
        result["visual_to_collision"]["status"] == "COMPLETE"
        and result["collision_to_visual"]["status"] == "COMPLETE"
    )
    if inventory.tree_valid and declared_total == loadable_total and surfaces_complete:
        result["status"] = "COMPLETE"
    elif inventory.tree_valid and loadable_total:
        result["status"] = "PARTIAL"
    return result


__all__ = [
    "GeometryInventory",
    "GeometryShape",
    "collision_redundancy_measurement",
    "collision_triangle_validation_measurement",
    "evaluate_table4b_geometry",
    "exact_closest_surface_distances",
    "load_urdf_geometry",
    "measure_collision_load_time_in_asset_child",
    "pcg64dxsm_rng",
    "qualify_exact_surface_backend",
    "sample_surface_area_weighted",
    "seed_sha256",
]
