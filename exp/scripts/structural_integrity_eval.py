#!/usr/bin/env python3
"""Simulator-free structural-integrity evaluation for frozen URDF cohorts.

The evaluator is intentionally independent from the Table 5 simulator workers.
It uses URDF FK, collision-preferred (visual fallback) geometry, area-weighted
surface samples, trimesh proximity queries, and optional FCL collision checks.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
DEFAULT_MANIFEST = (
    REPO_ROOT
    / "exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json"
)
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/structural_integrity_pva_articraft_core200"
ARTICRAFT_STAGE = (
    REPO_ROOT
    / "exp/runtime/articraft_github_full10787_export_noverify_20260830/staging/data/cache/record_materialization"
)
PROTOCOL_ID = "structural-integrity-urdf-v1-pilot"
SCHEMA_VERSION = "structural_integrity_eval_v1"


def _finite(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_hash(value: Any) -> str:
    payload = json.dumps(
        value, ensure_ascii=True, allow_nan=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _stable_joint_seed(asset_seed: int, joint_name: str) -> int:
    """Derive a process-independent seed for a joint's local sampling."""
    digest = hashlib.sha256(f"{asset_seed}|{joint_name}".encode("utf-8")).hexdigest()
    return asset_seed + int(digest[:12], 16)


def _rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = (float(x) for x in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _origin(element: ET.Element | None) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    if element is None:
        return result
    xyz = np.asarray(
        [float(x) for x in (element.get("xyz") or "0 0 0").split()], dtype=np.float64
    )
    rpy = np.asarray(
        [float(x) for x in (element.get("rpy") or "0 0 0").split()], dtype=np.float64
    )
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(xyz).all() or not np.isfinite(rpy).all():
        raise ValueError("origin must contain finite xyz/rpy vectors")
    result[:3, :3] = _rpy_matrix(rpy)
    result[:3, 3] = xyz
    return result


def _axis(element: ET.Element | None) -> np.ndarray:
    raw = (element.get("xyz") if element is not None else None) or "0 0 1"
    value = np.asarray([float(x) for x in raw.split()], dtype=np.float64)
    if value.shape != (3,) or not np.isfinite(value).all() or np.linalg.norm(value) <= 1e-12:
        raise ValueError("joint axis must be a non-zero finite vector")
    return value / np.linalg.norm(value)


def _motion(joint_type: str, axis: np.ndarray, q: float) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    if joint_type in {"revolute", "continuous"}:
        x, y, z = (float(v) for v in axis)
        c, s = math.cos(q), math.sin(q)
        result[:3, :3] = np.asarray(
            [
                [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
            ],
            dtype=np.float64,
        )
    elif joint_type == "prismatic":
        result[:3, 3] = axis * float(q)
    return result


@dataclass(frozen=True)
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None


@dataclass
class LinkGeometry:
    name: str
    mesh: Any | None
    source: str
    declared_collision: int
    declared_visual: int
    loaded_elements: int
    issues: list[str]
    samples: np.ndarray | None = None


@dataclass
class AssetModel:
    urdf_path: Path
    package_root: Path
    links: dict[str, LinkGeometry]
    joints: list[Joint]
    roots: list[str]
    issues: list[str]
    diagonal: float | None


@dataclass(frozen=True)
class Config:
    geometry_role: str = "visual"
    roi_fraction: float = 0.02
    gap_fraction: float = 0.002
    support_threshold: float = 0.10
    pose_samples: int = 9
    surface_samples: int = 512
    penetration_depth_fraction: float = 0.0
    skip_penetration: bool = False


def _resolve_mesh(raw: str, urdf_path: Path, package_root: Path) -> Path:
    if raw.startswith("file://"):
        candidates = [Path(raw[7:])]
    elif raw.startswith("package://"):
        suffix = raw[len("package://") :]
        bits = Path(suffix).parts
        candidates = [package_root / suffix]
        if len(bits) > 1:
            candidates.append(package_root / Path(*bits[1:]))
    else:
        raw_path = Path(raw)
        candidates = [raw_path if raw_path.is_absolute() else urdf_path.parent / raw_path]
        if not raw_path.is_absolute():
            candidates.append(package_root / raw_path)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(raw)


def _load_shape(shape: ET.Element, urdf_path: Path, package_root: Path) -> Any:
    import trimesh

    kind = shape.tag
    if kind == "mesh":
        path = _resolve_mesh(shape.get("filename", ""), urdf_path, package_root)
        mesh = trimesh.load(str(path), force="mesh", process=False)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
        if not isinstance(mesh, trimesh.Trimesh):
            raise ValueError(f"unsupported mesh payload: {path}")
        scale = np.asarray(
            [float(x) for x in (shape.get("scale") or "1 1 1").split()], dtype=np.float64
        )
        if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale == 0):
            raise ValueError("mesh scale must be finite and non-zero")
        mesh = mesh.copy()
        mesh.apply_scale(scale)
    elif kind == "box":
        size = np.asarray([float(x) for x in (shape.get("size") or "").split()], dtype=np.float64)
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError("box size must be positive")
        mesh = trimesh.creation.box(extents=size)
    elif kind == "cylinder":
        radius, length = float(shape.get("radius", "nan")), float(shape.get("length", "nan"))
        if not (_finite(radius) and _finite(length) and radius > 0 and length > 0):
            raise ValueError("cylinder dimensions must be positive")
        mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=32)
    elif kind == "sphere":
        radius = float(shape.get("radius", "nan"))
        if not (_finite(radius) and radius > 0):
            raise ValueError("sphere radius must be positive")
        mesh = trimesh.creation.icosphere(subdivisions=2, radius=radius)
    else:
        raise ValueError(f"unsupported geometry kind: {kind}")
    return mesh


def _load_element(container: ET.Element, urdf_path: Path, package_root: Path) -> Any:
    geometry = container.find("geometry")
    if geometry is None or len(geometry) != 1:
        raise ValueError("geometry must contain one shape")
    import trimesh

    shape = _load_shape(geometry[0], urdf_path, package_root)
    shape.apply_transform(_origin(container.find("origin")))
    if not isinstance(shape, trimesh.Trimesh) or len(shape.faces) == 0:
        raise ValueError("geometry has no triangle faces")
    if not np.isfinite(shape.vertices).all():
        raise ValueError("geometry vertices are not finite")
    return shape


def _combine(meshes: Sequence[Any]) -> Any:
    import trimesh

    if not meshes:
        return None
    return trimesh.util.concatenate(tuple(meshes))


def _parse_asset(urdf_path: Path, package_root: Path, geometry_role: str = "visual") -> AssetModel:
    payload = urdf_path.read_bytes()
    root = ET.fromstring(payload)
    if root.tag != "robot":
        raise ValueError("URDF root is not robot")
    links: dict[str, LinkGeometry] = {}
    asset_issues: list[str] = []
    for link in root.findall("link"):
        name = link.get("name") or ""
        if not name or name in links:
            asset_issues.append(f"invalid or duplicate link name: {name!r}")
            continue
        collision_nodes = [x for x in link.findall("collision") if x.find("geometry") is not None]
        visual_nodes = [x for x in link.findall("visual") if x.find("geometry") is not None]
        collision_meshes: list[Any] = []
        visual_meshes: list[Any] = []
        issues: list[str] = []
        for index, node in enumerate(collision_nodes):
            try:
                collision_meshes.append(_load_element(node, urdf_path, package_root))
            except Exception as exc:  # per-element fail-closed, preserve the asset
                issues.append(f"collision[{index}]: {type(exc).__name__}: {exc}")
        for index, node in enumerate(visual_nodes):
            try:
                visual_meshes.append(_load_element(node, urdf_path, package_root))
            except Exception as exc:
                issues.append(f"visual[{index}]: {type(exc).__name__}: {exc}")
        if geometry_role == "visual" and visual_meshes:
            mesh, source = _combine(visual_meshes), "visual"
            loaded = len(visual_meshes)
        elif collision_meshes:
            mesh, source = _combine(collision_meshes), "collision"
            loaded = len(collision_meshes)
        elif visual_meshes:
            mesh, source = _combine(visual_meshes), "visual_fallback"
            loaded = len(visual_meshes)
        else:
            mesh, source, loaded = None, "missing", 0
        links[name] = LinkGeometry(
            name=name,
            mesh=mesh,
            source=source,
            declared_collision=len(collision_nodes),
            declared_visual=len(visual_nodes),
            loaded_elements=loaded,
            issues=issues,
        )
    joints: list[Joint] = []
    children: set[str] = set()
    for index, node in enumerate(root.findall("joint")):
        name = node.get("name") or f"__unnamed_joint_{index}"
        joint_type = (node.get("type") or "").lower()
        parent_node, child_node = node.find("parent"), node.find("child")
        parent = parent_node.get("link") if parent_node is not None else None
        child = child_node.get("link") if child_node is not None else None
        if parent not in links or child not in links or parent == child or child in children:
            asset_issues.append(f"invalid joint tree entry: {name}")
            continue
        children.add(str(child))
        limit = node.find("limit")
        lower = upper = None
        if limit is not None:
            try:
                lower = float(limit.get("lower")) if limit.get("lower") is not None else None
                upper = float(limit.get("upper")) if limit.get("upper") is not None else None
                if lower is not None and upper is not None and not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
                    lower = upper = None
            except (TypeError, ValueError):
                lower = upper = None
        try:
            joint = Joint(
                name=name,
                joint_type=joint_type,
                parent=str(parent),
                child=str(child),
                origin=_origin(node.find("origin")),
                axis=_axis(node.find("axis")),
                lower=lower,
                upper=upper,
            )
        except Exception as exc:
            asset_issues.append(f"joint {name} parse failed: {type(exc).__name__}: {exc}")
            continue
        joints.append(joint)
    roots = sorted(set(links) - children)
    diagonal = None
    bounds = [geometry.mesh.bounds for geometry in links.values() if geometry.mesh is not None]
    if bounds:
        low = np.min(np.vstack([x[0] for x in bounds]), axis=0)
        high = np.max(np.vstack([x[1] for x in bounds]), axis=0)
        value = float(np.linalg.norm(high - low))
        if math.isfinite(value) and value > 0:
            diagonal = value
    return AssetModel(urdf_path, package_root, links, joints, roots, asset_issues, diagonal)


def _fk(asset: AssetModel, q: Mapping[str, float]) -> dict[str, np.ndarray]:
    if len(asset.roots) != 1:
        raise ValueError(f"expected one root, found {asset.roots}")
    outgoing: dict[str, list[Joint]] = {}
    for joint in asset.joints:
        outgoing.setdefault(joint.parent, []).append(joint)
    transforms: dict[str, np.ndarray] = {asset.roots[0]: np.eye(4, dtype=np.float64)}
    queue = [asset.roots[0]]
    while queue:
        parent = queue.pop(0)
        for joint in outgoing.get(parent, []):
            if joint.child in transforms:
                raise ValueError("joint graph contains a cycle")
            transforms[joint.child] = transforms[parent] @ joint.origin @ _motion(
                joint.joint_type, joint.axis, float(q.get(joint.name, 0.0))
            )
            queue.append(joint.child)
    if set(transforms) != set(asset.links):
        raise ValueError("joint graph is disconnected")
    return transforms


def _sample_surface(mesh: Any, count: int, seed: int) -> np.ndarray:
    triangles = np.asarray(mesh.vertices, dtype=np.float64)[np.asarray(mesh.faces, dtype=np.int64)]
    area = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1) / 2.0
    valid = np.isfinite(area) & (area > 0)
    if not np.any(valid):
        raise ValueError("mesh has no positive-area triangles")
    triangles, area = triangles[valid], area[valid]
    cumulative = np.cumsum(area)
    rng = np.random.default_rng(seed)
    selected = np.searchsorted(cumulative, rng.random(count) * cumulative[-1], side="right")
    chosen = triangles[selected]
    u, v = rng.random(count), rng.random(count)
    root = np.sqrt(u)
    return (1 - root)[:, None] * chosen[:, 0] + (root * (1 - v))[:, None] * chosen[:, 1] + (root * v)[:, None] * chosen[:, 2]


def _transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def _world_mesh(mesh: Any, transform: np.ndarray) -> Any:
    import trimesh

    return trimesh.Trimesh(
        vertices=_transform_points(np.asarray(mesh.vertices), transform),
        faces=np.asarray(mesh.faces),
        process=False,
        validate=False,
    )


def _pose_values(joint: Joint, count: int) -> list[float]:
    if joint.joint_type == "fixed":
        return [0.0]
    if joint.lower is not None and joint.upper is not None:
        return np.linspace(joint.lower, joint.upper, count).tolist()
    if joint.joint_type in {"revolute", "continuous"}:
        return np.linspace(-math.pi, math.pi, count).tolist()
    if joint.joint_type == "prismatic":
        return np.linspace(-0.05, 0.05, count).tolist()
    return [0.0]


def _fcl_collision(mesh_a: Any, mesh_b: Any) -> tuple[bool, float]:
    import fcl

    def obj(mesh: Any) -> Any:
        model = fcl.BVHModel()
        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        faces = np.asarray(mesh.faces, dtype=np.int32)
        model.beginModel(len(vertices), len(faces))
        model.addSubModel(vertices, faces)
        model.endModel()
        return fcl.CollisionObject(model)

    request = fcl.CollisionRequest(num_max_contacts=32, enable_contact=True)
    result = fcl.CollisionResult()
    fcl.collide(obj(mesh_a), obj(mesh_b), request, result)
    depths = [max(0.0, float(contact.penetration_depth)) for contact in result.contacts]
    return bool(result.is_collision), (max(depths) if depths else 0.0)


def _roi_surface_points(mesh: Any, center_local: np.ndarray, radius: float, count: int, seed: int) -> np.ndarray:
    """Sample only triangles whose AABB intersects the joint-local ROI."""
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.faces, dtype=np.int64)
    triangles = vertices[faces]
    lower = triangles.min(axis=1)
    upper = triangles.max(axis=1)
    delta = np.maximum(np.maximum(lower - center_local, 0.0), center_local - upper)
    candidate = np.einsum("ij,ij->i", delta, delta) <= radius * radius
    if not np.any(candidate):
        return np.empty((0, 3), dtype=np.float64)
    triangles = triangles[candidate]
    area = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1) / 2.0
    valid = np.isfinite(area) & (area > 0.0)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)
    triangles, area = triangles[valid], area[valid]
    rng = np.random.default_rng(seed)
    cumulative = np.cumsum(area)
    collected: list[np.ndarray] = []
    for _ in range(6):
        draw_count = max(count * 2, 64)
        selected = np.searchsorted(cumulative, rng.random(draw_count) * cumulative[-1], side="right")
        chosen = triangles[selected]
        u, v = rng.random(draw_count), rng.random(draw_count)
        root = np.sqrt(u)
        points = (1 - root)[:, None] * chosen[:, 0] + (root * (1 - v))[:, None] * chosen[:, 1] + (root * v)[:, None] * chosen[:, 2]
        points = points[np.linalg.norm(points - center_local, axis=1) <= radius]
        if len(points):
            collected.append(points)
        if sum(len(part) for part in collected) >= count:
            break
    if not collected:
        return np.empty((0, 3), dtype=np.float64)
    return np.vstack(collected)[:count]


def _axis_mesh_distance(mesh: Any, origin: np.ndarray, axis: np.ndarray, half_length: float) -> float:
    """Minimum surface/AABB-center distance to a bounded joint axis segment."""
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    relative = vertices - origin
    axial = relative @ axis
    radial = np.linalg.norm(relative - axial[:, None] * axis, axis=1)
    within_segment = np.abs(axial) <= half_length
    best = float(np.min(radial[within_segment])) if np.any(within_segment) else float("inf")
    center = np.asarray(mesh.bounds, dtype=np.float64).mean(axis=0)
    center_relative = center - origin
    center_axial = float(center_relative @ axis)
    if abs(center_axial) <= half_length:
        center_radial = float(np.linalg.norm(center_relative - center_axial * axis))
        best = min(best, center_radial)
    return best


def _joint_axis_measure(
    asset: AssetModel,
    joint: Joint,
    transforms: Mapping[str, np.ndarray],
    world_meshes: Mapping[str, Any],
    config: Config,
) -> dict[str, Any]:
    """Measure whether a joint's axis/point is carried by both link geometries.

    Revolute/continuous/prismatic joints use a bounded axis segment. This is
    intentionally separate from the origin-centered surface ROI: a bearing or
    collar can support an axis while its center contains no surface triangles.
    Fixed joints use the joint origin as a point probe.
    """
    if asset.diagonal is None:
        return {"status": "not_evaluable", "joint": joint.name, "reason": "missing_asset_diagonal"}
    joint_frame = transforms[joint.parent] @ joint.origin
    origin = joint_frame[:3, 3]
    axis = joint_frame[:3, :3] @ joint.axis
    axis /= np.linalg.norm(axis)
    parent_mesh = world_meshes[joint.parent]
    child_mesh = world_meshes[joint.child]
    if joint.joint_type == "fixed":
        parent_distance = float(np.min(np.linalg.norm(parent_mesh.vertices - origin, axis=1)))
        child_distance = float(np.min(np.linalg.norm(child_mesh.vertices - origin, axis=1)))
        half_length = 0.0
        metric = "point"
    else:
        projections = []
        for mesh in (parent_mesh, child_mesh):
            projections.append((mesh.vertices - origin) @ axis)
        half_length = max(1e-9, 0.5 * max(float(np.max(x) - np.min(x)) for x in projections))
        parent_distance = _axis_mesh_distance(parent_mesh, origin, axis, half_length)
        child_distance = _axis_mesh_distance(child_mesh, origin, axis, half_length)
        metric = "axis"
    tolerance = config.roi_fraction * asset.diagonal
    return {
        "status": "complete",
        "joint": joint.name,
        "parent": joint.parent,
        "child": joint.child,
        "metric": metric,
        "parent_axis_distance_m": parent_distance,
        "child_axis_distance_m": child_distance,
        "parent_axis_distance_fraction": parent_distance / asset.diagonal,
        "child_axis_distance_fraction": child_distance / asset.diagonal,
        "axis_half_length_m": half_length,
        "tolerance_fraction": config.roi_fraction,
        "supported": bool(parent_distance <= tolerance and child_distance <= tolerance),
    }


def _joint_measure(
    asset: AssetModel,
    joint: Joint,
    transforms: Mapping[str, np.ndarray],
    world_meshes: Mapping[str, Any],
    proximity_queries: Mapping[str, Any],
    config: Config,
    seed: int,
) -> dict[str, Any]:
    if asset.diagonal is None:
        return {"status": "not_evaluable", "reason": "missing_asset_diagonal", "joint": joint.name}
    center = transforms[joint.child][:3, 3]
    radius = config.roi_fraction * asset.diagonal
    parent_transform = transforms[joint.parent]
    child_transform = transforms[joint.child]
    parent_local_center = _transform_points(center.reshape(1, 3), np.linalg.inv(parent_transform))[0]
    child_local_center = _transform_points(center.reshape(1, 3), np.linalg.inv(child_transform))[0]
    parent_local = _roi_surface_points(asset.links[joint.parent].mesh, parent_local_center, radius, config.surface_samples, seed)
    child_local = _roi_surface_points(asset.links[joint.child].mesh, child_local_center, radius, config.surface_samples, seed + 1)
    parent_points = _transform_points(parent_local, parent_transform)
    child_points = _transform_points(child_local, child_transform)
    if len(parent_points) == 0 or len(child_points) == 0:
        return {
            "status": "not_evaluable",
            "reason": "empty_joint_roi",
            "joint": joint.name,
            "parent": joint.parent,
            "child": joint.child,
            "parent_roi_points": int(len(parent_points)),
            "child_roi_points": int(len(child_points)),
        }
    # Keep one spatial index per link. Querying local meshes after transforming
    # points avoids rebuilding an rtree for every joint at every pose.
    parent_query = proximity_queries[joint.parent]
    child_query = proximity_queries[joint.child]
    parent_in_child = _transform_points(parent_points, np.linalg.inv(child_transform))
    child_in_parent = _transform_points(child_points, np.linalg.inv(parent_transform))
    _, p_to_c, _ = child_query.on_surface(parent_in_child)
    _, c_to_p, _ = parent_query.on_surface(child_in_parent)
    p_to_c = np.asarray(p_to_c, dtype=np.float64)
    c_to_p = np.asarray(c_to_p, dtype=np.float64)
    gap_tol = config.gap_fraction * asset.diagonal
    cp = float(np.mean(p_to_c <= gap_tol))
    cc = float(np.mean(c_to_p <= gap_tol))
    # Q5 captures whether there is a nearby mating patch; support already
    # measures how much of each ROI is close, so a median would double-count.
    gap = float(np.percentile(np.concatenate((p_to_c, c_to_p)), 5))
    support = min(cp, cc)
    return {
        "status": "complete",
        "joint": joint.name,
        "parent": joint.parent,
        "child": joint.child,
        "parent_roi_points": int(len(parent_points)),
        "child_roi_points": int(len(child_points)),
        "parent_to_child_support": cp,
        "child_to_parent_support": cc,
        "support": support,
        "gap_m": gap,
        "gap_fraction": gap / asset.diagonal,
        "supported": bool(support >= config.support_threshold),
    }


def _evaluate_asset(asset: AssetModel, config: Config, asset_seed: int) -> dict[str, Any]:
    if asset.diagonal is None or not asset.links:
        return {
            "status": "not_evaluable",
            "reason": "missing_geometry_or_diagonal",
            "issues": asset.issues,
        }
    if any(geometry.mesh is None for geometry in asset.links.values()):
        return {
            "status": "not_evaluable",
            "reason": "one_or_more_links_have_no_geometry",
            "issues": asset.issues + [f"{name}: {g.issues}" for name, g in asset.links.items() if g.mesh is None],
            "geometry_source_counts": dict(__import__("collections").Counter(g.source for g in asset.links.values())),
        }

    # Normalize all local gap measurements by the q=0 world-space asset extent.
    # Link-local bounds are not comparable when links have non-zero joint origins.
    try:
        q0_transforms = _fk(asset, {})
        q0_meshes = {name: _world_mesh(g.mesh, q0_transforms[name]) for name, g in asset.links.items()}
        q0_bounds = [mesh.bounds for mesh in q0_meshes.values()]
        low = np.min(np.vstack([bounds[0] for bounds in q0_bounds]), axis=0)
        high = np.max(np.vstack([bounds[1] for bounds in q0_bounds]), axis=0)
        q0_diagonal = float(np.linalg.norm(high - low))
        if not math.isfinite(q0_diagonal) or q0_diagonal <= 0:
            raise ValueError("q=0 world asset diagonal is not positive")
        asset.diagonal = q0_diagonal
    except Exception as exc:
        return {"status": "not_evaluable", "reason": f"invalid_fk_or_world_bounds:{type(exc).__name__}: {exc}", "issues": asset.issues}

    try:
        import trimesh

        proximity_queries = {name: trimesh.proximity.ProximityQuery(geometry.mesh) for name, geometry in asset.links.items()}
    except Exception as exc:
        return {"status": "not_evaluable", "reason": f"proximity_index_failed:{type(exc).__name__}: {exc}", "issues": asset.issues}

    def pose_report(q: Mapping[str, float], *, include_collision: bool) -> dict[str, Any]:
        transforms = _fk(asset, q)
        world_meshes = {name: _world_mesh(g.mesh, transforms[name]) for name, g in asset.links.items()}
        joint_rows = [_joint_measure(asset, joint, transforms, world_meshes, proximity_queries, config, _stable_joint_seed(asset_seed, joint.name)) for joint in asset.joints]
        axis_joint_rows = [_joint_axis_measure(asset, joint, transforms, world_meshes, config) for joint in asset.joints]
        evaluated = [row for row in joint_rows if row.get("status") == "complete"]
        support_pass = bool(evaluated) and len(evaluated) == len(joint_rows) and all(row["supported"] for row in evaluated)
        gap_pass = bool(evaluated) and len(evaluated) == len(joint_rows) and all(
            float(row["gap_fraction"]) <= config.gap_fraction for row in evaluated
        )
        axis_evaluated = [row for row in axis_joint_rows if row.get("status") == "complete"]
        axis_support_pass = bool(axis_evaluated) and len(axis_evaluated) == len(axis_joint_rows) and all(row["supported"] for row in axis_evaluated)
        illegal_pairs: list[dict[str, Any]] = []
        if include_collision and not config.skip_penetration:
            adjacent = {frozenset((joint.parent, joint.child)) for joint in asset.joints}
            names = sorted(world_meshes)
            for left_index, left in enumerate(names):
                for right in names[left_index + 1 :]:
                    if frozenset((left, right)) in adjacent:
                        continue
                    left_mesh, right_mesh = world_meshes[left], world_meshes[right]
                    if np.any(left_mesh.bounds[1] < right_mesh.bounds[0]) or np.any(right_mesh.bounds[1] < left_mesh.bounds[0]):
                        continue
                    try:
                        collided, depth = _fcl_collision(left_mesh, right_mesh)
                    except Exception as exc:
                        illegal_pairs.append({"left": left, "right": right, "status": "not_evaluable", "reason": f"{type(exc).__name__}: {exc}"})
                        continue
                    if collided and depth > config.penetration_depth_fraction * asset.diagonal:
                        illegal_pairs.append({"left": left, "right": right, "status": "penetration", "depth_m": depth, "depth_fraction": depth / asset.diagonal})
        return {
            "joint_rows": joint_rows,
            "axis_joint_rows": axis_joint_rows,
            "support_pass": support_pass,
            "gap_pass": gap_pass,
            "axis_support_pass": axis_support_pass,
            "collision_checked": bool(include_collision and not config.skip_penetration),
            "illegal_collision_pairs": illegal_pairs,
            "transforms": transforms,
        }

    static = pose_report({}, include_collision=False)
    declared_edges = len(asset.joints)
    static_complete = [row for row in static["joint_rows"] if row.get("status") == "complete"]
    rooted = bool(len(asset.roots) == 1 and declared_edges > 0 and len(static_complete) == declared_edges and all(row["supported"] for row in static_complete))

    pose_rows: list[dict[str, Any]] = []
    movable = [j for j in asset.joints if j.joint_type != "fixed"]
    for joint in movable:
        for pose_index, value in enumerate(_pose_values(joint, config.pose_samples)):
            q = {joint.name: float(value)}
            try:
                result = pose_report(q, include_collision=not config.skip_penetration)
                support_gap_passed = bool(result["support_pass"] and result["gap_pass"])
                axis_support_pass = bool(result["axis_support_pass"])
                collision_checked = bool(result["collision_checked"])
                passed = bool(support_gap_passed and (not collision_checked or not result["illegal_collision_pairs"]))
                pose_rows.append({
                    "joint": joint.name,
                    "pose_index": pose_index,
                    "q": float(value),
                    "passed": passed,
                    "support_pass": result["support_pass"],
                    "gap_pass": result["gap_pass"],
                    "support_gap_passed": support_gap_passed,
                    "axis_support_pass": axis_support_pass,
                    "collision_checked": collision_checked,
                    "collision_free": (not result["illegal_collision_pairs"]) if collision_checked else None,
                    "joint_rows": result["joint_rows"],
                    "illegal_collision_pairs": result["illegal_collision_pairs"],
                })
            except Exception as exc:
                pose_rows.append({"joint": joint.name, "pose_index": pose_index, "q": float(value), "passed": False, "status": "error", "reason": f"{type(exc).__name__}: {exc}"})
    gap_values = [float(row["gap_fraction"]) for row in static_complete if _finite(row.get("gap_fraction"))]
    support_rates = [float(row["support"]) if _finite(row.get("support")) else 0.0 for row in static["joint_rows"]]
    supported_count = sum(bool(row.get("supported")) for row in static["joint_rows"])
    axis_complete = [row for row in static["axis_joint_rows"] if row.get("status") == "complete"]
    axis_supported_count = sum(bool(row.get("supported")) for row in axis_complete)
    axis_support_rates = [1.0 if row.get("supported") else 0.0 for row in axis_complete]
    collision_rows = [row for row in pose_rows if row.get("collision_checked") is True]
    collision_events = sum(len(row.get("illegal_collision_pairs", [])) for row in collision_rows)
    collision_free_poses = sum(not row.get("illegal_collision_pairs") for row in collision_rows)
    collision_asset_free = (
        bool(collision_rows)
        and len(collision_rows) == len(pose_rows)
        and collision_free_poses == len(collision_rows)
    ) if not config.skip_penetration else None
    support_gap_valid = bool(pose_rows) and all(bool(row.get("support_gap_passed")) for row in pose_rows)
    axis_pose_valid = bool(pose_rows) and all(bool(row.get("axis_support_pass")) for row in pose_rows)
    return {
        "status": "complete",
        "urdf_path": str(asset.urdf_path),
        "package_root": str(asset.package_root),
        "asset_diagonal_m": asset.diagonal,
        "link_count": len(asset.links),
        "joint_count": len(asset.joints),
        "movable_joint_count": len(movable),
        "roots": asset.roots,
        "issues": asset.issues + [f"{name}: {g.issues}" for name, g in asset.links.items() if g.issues],
        "geometry_source_counts": dict(__import__("collections").Counter(g.source for g in asset.links.values())),
        "rooted_asset": rooted,
        "static_joint_support": {
            "supported": supported_count,
            "evaluable": len(static_complete),
            "candidate": declared_edges,
            "rate": (supported_count / declared_edges) if declared_edges else None,
            "mean_support": statistics.fmean(support_rates) if support_rates else None,
        },
        "static_joint_gap_fraction_p50": (float(np.percentile(gap_values, 50)) if gap_values else None),
        "static_joint_gap_fraction_p95": (float(np.percentile(gap_values, 95)) if gap_values else None),
        "static_joint_rows": static["joint_rows"],
        "static_axis_joint_rows": static["axis_joint_rows"],
        "axis_rooted_asset": bool(len(asset.roots) == 1 and declared_edges > 0 and len(axis_complete) == declared_edges and axis_supported_count == declared_edges),
        "static_axis_support": {
            "supported": axis_supported_count,
            "evaluable": len(axis_complete),
            "candidate": declared_edges,
            "rate": (axis_supported_count / declared_edges) if declared_edges else None,
            "mean_support": statistics.fmean(axis_support_rates) if axis_support_rates else None,
        },
        "pose_swept": {
            "candidate_poses": len(pose_rows),
            "evaluated_poses": sum("status" not in row for row in pose_rows),
            "passed_poses": sum(bool(row.get("passed")) for row in pose_rows),
            "support_gap_valid": support_gap_valid,
            "axis_support_valid": axis_pose_valid,
            "valid": support_gap_valid and collision_asset_free if collision_asset_free is not None else None,
            "collision_checked": not config.skip_penetration,
            "rows": pose_rows,
        },
        "joint_self_collision": {
            "enabled": not config.skip_penetration,
            "candidate_poses": len(pose_rows),
            "evaluable_poses": len(collision_rows),
            "free_poses": collision_free_poses,
            "collision_events": collision_events,
            "asset_free": collision_asset_free,
        },
    }


def _resolve_row_path(row: Mapping[str, Any], dataset_slug: str) -> tuple[Path, Path, str | None]:
    original = Path(str(row.get("urdf_path", "")))
    if original.is_file():
        return original.resolve(), Path(str(row.get("package_root") or original.parent)).resolve(), None
    if dataset_slug == "articraft_10k":
        candidate = ARTICRAFT_STAGE / str(row.get("asset_id")) / "model.urdf"
        if candidate.is_file():
            return candidate.resolve(), candidate.parent.resolve(), f"stale_manifest_path:{original}"
    raise FileNotFoundError(original)


def _iter_rows(manifest: Mapping[str, Any], datasets: set[str], max_assets: int | None) -> Iterable[tuple[str, str, dict[str, Any]]]:
    count = 0
    for dataset in manifest.get("datasets", []):
        slug = str(dataset.get("dataset_slug"))
        if slug not in datasets:
            continue
        for row in dataset.get("rows", []):
            if max_assets is not None and count >= max_assets:
                return
            count += 1
            yield slug, str(dataset.get("dataset_name") or slug), dict(row)


def _aggregate(records: Sequence[Mapping[str, Any]], config: Config) -> dict[str, Any]:
    complete = [record for record in records if record.get("status") == "complete"]
    rooted = [record for record in complete if record.get("rooted_asset") is True]
    asset_support = [record["static_joint_support"]["rate"] for record in complete if _finite(record.get("static_joint_support", {}).get("rate"))]
    axis_asset_support = [record["static_axis_support"]["rate"] for record in complete if _finite(record.get("static_axis_support", {}).get("rate"))]
    micro_supported = sum(int(record["static_joint_support"]["supported"]) for record in complete)
    micro_candidate = sum(int(record["static_joint_support"]["candidate"]) for record in complete)
    micro_evaluable = sum(int(record["static_joint_support"]["evaluable"]) for record in complete)
    axis_micro_supported = sum(int(record["static_axis_support"]["supported"]) for record in complete)
    axis_micro_candidate = sum(int(record["static_axis_support"]["candidate"]) for record in complete)
    asset_gaps = [record["static_joint_gap_fraction_p95"] for record in complete if _finite(record.get("static_joint_gap_fraction_p95"))]
    pose_valid = [record.get("pose_swept", {}).get("valid") is True for record in complete]
    collision_assets = [record.get("joint_self_collision", {}).get("asset_free") for record in complete if record.get("joint_self_collision", {}).get("asset_free") is not None]
    collision_evaluable_poses = sum(int(record.get("joint_self_collision", {}).get("evaluable_poses", 0)) for record in complete)
    collision_free_poses = sum(int(record.get("joint_self_collision", {}).get("free_poses", 0)) for record in complete)
    support_gap_assets = [record.get("pose_swept", {}).get("support_gap_valid") is True for record in complete]
    axis_rooted_assets = [record.get("axis_rooted_asset") is True for record in complete]
    axis_pose_assets = [record.get("pose_swept", {}).get("axis_support_valid") is True for record in complete]
    support_gap_poses = [pose for record in complete for pose in record.get("pose_swept", {}).get("rows", [])]
    return {
        "asset_count": len(records),
        "complete_asset_count": len(complete),
        "rooted_assets": {"passed": len(rooted), "denominator": len(records), "percentage": 100 * len(rooted) / len(records) if records else None},
        "joint_support_asset_macro": {"mean_rate": 100 * statistics.fmean(asset_support) if asset_support else None, "evaluable_assets": len(asset_support), "denominator": len(records)},
        "joint_support_micro": {"supported": micro_supported, "candidate": micro_candidate, "percentage": 100 * micro_supported / micro_candidate if micro_candidate else None},
        "joint_roi_coverage": {"evaluable": micro_evaluable, "candidate": micro_candidate, "percentage": 100 * micro_evaluable / micro_candidate if micro_candidate else None},
        "axis_rooted_assets": {"passed": sum(axis_rooted_assets), "denominator": len(records), "percentage": 100 * sum(axis_rooted_assets) / len(records) if records else None},
        "joint_axis_support_asset_macro": {"mean_rate": 100 * statistics.fmean(axis_asset_support) if axis_asset_support else None, "evaluable_assets": len(axis_asset_support), "denominator": len(records)},
        "joint_axis_support_micro": {"supported": axis_micro_supported, "candidate": axis_micro_candidate, "percentage": 100 * axis_micro_supported / axis_micro_candidate if axis_micro_candidate else None},
        "joint_gap_p95_fraction": {"value": 100 * float(np.percentile(asset_gaps, 95)) if asset_gaps else None, "unit": "% asset bbox diagonal", "asset_evaluable": len(asset_gaps), "denominator": len(records)},
        "pose_swept_valid": {"passed": sum(pose_valid), "evaluable_assets": len([x for x in complete if x.get("pose_swept", {}).get("valid") is not None]), "denominator": len(records), "percentage": 100 * sum(pose_valid) / len([x for x in complete if x.get("pose_swept", {}).get("valid") is not None]) if any(x.get("pose_swept", {}).get("valid") is not None for x in complete) else None},
        "pose_swept_support_gap_valid": {"passed": sum(support_gap_assets), "denominator": len(records), "percentage": 100 * sum(support_gap_assets) / len(records) if records else None, "passed_poses": sum(bool(pose.get("support_gap_passed")) for pose in support_gap_poses), "candidate_poses": len(support_gap_poses), "pose_percentage": 100 * sum(bool(pose.get("support_gap_passed")) for pose in support_gap_poses) / len(support_gap_poses) if support_gap_poses else None},
        "pose_swept_axis_support_valid": {"passed": sum(axis_pose_assets), "denominator": len(records), "percentage": 100 * sum(axis_pose_assets) / len(records) if records else None, "passed_poses": sum(bool(pose.get("axis_support_pass")) for pose in support_gap_poses), "candidate_poses": len(support_gap_poses), "pose_percentage": 100 * sum(bool(pose.get("axis_support_pass")) for pose in support_gap_poses) / len(support_gap_poses) if support_gap_poses else None},
        "joint_self_collision_free": {"passed": sum(bool(x) for x in collision_assets), "evaluable_assets": len(collision_assets), "denominator": len(records), "percentage": 100 * sum(bool(x) for x in collision_assets) / len(collision_assets) if collision_assets else None, "evaluable_poses": collision_evaluable_poses, "free_poses": collision_free_poses},
        "geometry_coverage": {"complete_assets": len(complete), "denominator": len(records), "percentage": 100 * len(complete) / len(records) if records else None},
        "settings": config.__dict__,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--datasets", default="pva,articraft_10k")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--max-assets", type=int, default=None)
    parser.add_argument("--workers", type=int, default=1, help="reserved for a future isolated worker mode")
    parser.add_argument("--skip-penetration", action="store_true", help="skip optional FCL non-adjacent collision checks")
    parser.add_argument("--geometry-role", choices=("visual", "collision"), default="visual", help="mesh role used for attachment, gap, and collision checks")
    parser.add_argument("--roi-fraction", type=float, default=0.02, help="joint-local ROI radius as a fraction of asset diagonal")
    parser.add_argument("--gap-fraction", type=float, default=0.002, help="gap/support distance threshold as a fraction of asset diagonal")
    parser.add_argument("--pose-samples", type=int, default=9)
    parser.add_argument("--surface-samples", type=int, default=512)
    args = parser.parse_args(argv)
    if args.workers != 1:
        parser.error("only --workers 1 is currently supported")
    if args.pose_samples < 1 or args.surface_samples < 32 or args.roi_fraction <= 0 or args.gap_fraction <= 0:
        parser.error("pose-samples must be >=1, surface-samples >=32, and ROI/gap fractions must be positive")
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    datasets = {item.strip() for item in args.datasets.split(",") if item.strip()}
    config = Config(geometry_role=args.geometry_role, roi_fraction=args.roi_fraction, gap_fraction=args.gap_fraction, pose_samples=args.pose_samples, surface_samples=args.surface_samples, skip_penetration=bool(args.skip_penetration))
    args.out.mkdir(parents=True, exist_ok=True)
    records_by_dataset: dict[str, list[dict[str, Any]]] = {}
    for slug, dataset_name, row in _iter_rows(manifest, datasets, args.max_assets):
        record: dict[str, Any] = {"dataset_slug": slug, "dataset_name": dataset_name, "asset_id": row.get("asset_id"), "dataset_id": row.get("dataset_id")}
        try:
            urdf_path, package_root, rebind = _resolve_row_path(row, slug)
            asset = _parse_asset(urdf_path, package_root, config.geometry_role)
            if rebind:
                asset.issues.append(rebind)
            seed = int(hashlib.sha256(f"{PROTOCOL_ID}|{slug}|{row.get('asset_id')}".encode()).hexdigest()[:16], 16)
            record.update(_evaluate_asset(asset, config, seed))
        except Exception as exc:
            record.update({"status": "error", "reason": f"{type(exc).__name__}: {exc}"})
        records_by_dataset.setdefault(slug, []).append(record)
        print(f"{slug} {record.get('asset_id')}: {record.get('status')}", flush=True)
    all_records = [record for rows in records_by_dataset.values() for record in rows]
    summary = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "script_sha256": _sha256_file(SCRIPT_PATH),
        "manifest_path": str(args.manifest.resolve()),
        "manifest_sha256": _sha256_file(args.manifest),
        "datasets": {slug: _aggregate(rows, config) for slug, rows in records_by_dataset.items()},
        "config": config.__dict__,
    }
    (args.out / "summary.json").write_text(json.dumps(summary, ensure_ascii=True, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (args.out / "asset_records.jsonl").write_text("".join(json.dumps(record, ensure_ascii=True, sort_keys=True) + "\n" for record in all_records), encoding="utf-8")
    with (args.out / "table.csv").open("w", newline="", encoding="utf-8") as handle:
        fields = ["dataset_slug", "asset_id", "status", "rooted_asset", "joint_support_rate", "axis_rooted_asset", "joint_axis_support_rate", "joint_gap_p95_percent_diag", "pose_swept_valid", "pose_swept_axis_support_valid", "joint_self_collision_free", "joint_self_collision_events", "geometry_source_counts"]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for record in all_records:
            support = record.get("static_joint_support", {})
            writer.writerow({
                "dataset_slug": record.get("dataset_slug"),
                "asset_id": record.get("asset_id"),
                "status": record.get("status"),
                "rooted_asset": record.get("rooted_asset"),
                "joint_support_rate": support.get("rate"),
                "axis_rooted_asset": record.get("axis_rooted_asset"),
                "joint_axis_support_rate": record.get("static_axis_support", {}).get("rate"),
                "joint_gap_p95_percent_diag": (100 * record["static_joint_gap_fraction_p95"] if _finite(record.get("static_joint_gap_fraction_p95")) else None),
                "pose_swept_valid": record.get("pose_swept", {}).get("valid"),
                "pose_swept_axis_support_valid": record.get("pose_swept", {}).get("axis_support_valid"),
                "joint_self_collision_free": record.get("joint_self_collision", {}).get("asset_free"),
                "joint_self_collision_events": record.get("joint_self_collision", {}).get("collision_events"),
                "geometry_source_counts": json.dumps(record.get("geometry_source_counts", {}), sort_keys=True),
            })
    print(json.dumps(summary["datasets"], ensure_ascii=True, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
