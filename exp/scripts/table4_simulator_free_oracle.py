#!/usr/bin/env python3
"""Simulator-free URDF FK and FCL collision oracle for Table 4 v4.

This module deliberately does not load a physics engine.  It parses the URDF
tree, evaluates forward kinematics analytically, and submits each authored
collision element to python-fcl as an independent geometry object.  FCL is a
collision-detection library, not a dynamics simulator.

Two collision views are reported because they answer different questions:

* ``intersection`` counts every FCL contact, including zero-depth touching;
* ``illegal_penetration`` follows the historical Table 4 tolerance and counts
  contacts whose FCL depth proxy is greater than 1e-6 m.  A collision without
  usable contact/depth evidence fails closed in this view.

For arbitrary non-convex triangle meshes, FCL contact depth is not a globally
exact minimum translation distance.  The output therefore calls it a depth
proxy and never promotes it to an exact physical penetration measurement.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import importlib.metadata
import importlib.util
import math
import os
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np


ORACLE_POLICY_ID = "urdf_analytic_fk_python_fcl_per_element_v1"
ORACLE_SCHEMA = "table4_simulator_free_oracle_v1"
PENETRATION_THRESHOLD_M = 1e-6
MAX_RECORDED_CONTACTS = 64
SUPPORTED_JOINT_TYPES = frozenset({"fixed", "revolute", "continuous", "prismatic"})
SUPPORTED_GEOMETRY_TYPES = frozenset({"box", "sphere", "cylinder", "capsule", "mesh"})
FORBIDDEN_SIMULATOR_MODULES = ("pybullet", "genesis", "mujoco")


class SimulatorFreeOracleError(ValueError):
    """Raised when an asset cannot be evaluated without weakening the protocol."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _finite_values(
    text: str | None,
    *,
    count: int,
    default: Sequence[float],
    label: str,
) -> tuple[float, ...]:
    try:
        values = (
            tuple(float(value) for value in text.split()) if text else tuple(default)
        )
    except (TypeError, ValueError) as exc:
        raise SimulatorFreeOracleError(f"{label} must be numeric") from exc
    if len(values) != count or any(not math.isfinite(value) for value in values):
        raise SimulatorFreeOracleError(f"{label} must contain {count} finite values")
    return values


def _positive(raw: str | None, label: str) -> float:
    try:
        value = float(raw) if raw is not None else math.nan
    except (TypeError, ValueError) as exc:
        raise SimulatorFreeOracleError(f"{label} must be numeric") from exc
    if not math.isfinite(value) or value <= 0.0:
        raise SimulatorFreeOracleError(f"{label} must be positive and finite")
    return value


def _rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=np.float64,
    )


def _origin_transform(node: ET.Element | None, label: str) -> np.ndarray:
    xyz = _finite_values(
        node.get("xyz") if node is not None else None,
        count=3,
        default=(0.0, 0.0, 0.0),
        label=f"{label} xyz",
    )
    rpy = _finite_values(
        node.get("rpy") if node is not None else None,
        count=3,
        default=(0.0, 0.0, 0.0),
        label=f"{label} rpy",
    )
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = _rpy_matrix(rpy)
    result[:3, 3] = xyz
    return result


def _axis_angle_transform(axis: Sequence[float], angle: float) -> np.ndarray:
    vector = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 0.0:
        raise SimulatorFreeOracleError("joint axis must be non-zero and finite")
    x, y, z = vector / norm
    cosine, sine = math.cos(float(angle)), math.sin(float(angle))
    complement = 1.0 - cosine
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(
        (
            (
                cosine + x * x * complement,
                x * y * complement - z * sine,
                x * z * complement + y * sine,
            ),
            (
                y * x * complement + z * sine,
                cosine + y * y * complement,
                y * z * complement - x * sine,
            ),
            (
                z * x * complement - y * sine,
                z * y * complement + x * sine,
                cosine + z * z * complement,
            ),
        ),
        dtype=np.float64,
    )
    return result


def _motion_transform(
    joint_type: str, axis: Sequence[float], value: float
) -> np.ndarray:
    if not math.isfinite(float(value)):
        raise SimulatorFreeOracleError("joint value must be finite")
    if joint_type in {"revolute", "continuous"}:
        return _axis_angle_transform(axis, value)
    if joint_type == "prismatic":
        vector = np.asarray(axis, dtype=np.float64)
        norm = float(np.linalg.norm(vector))
        if not math.isfinite(norm) or norm <= 0.0:
            raise SimulatorFreeOracleError("joint axis must be non-zero and finite")
        result = np.eye(4, dtype=np.float64)
        result[:3, 3] = vector / norm * float(value)
        return result
    if joint_type == "fixed":
        return np.eye(4, dtype=np.float64)
    raise SimulatorFreeOracleError(f"unsupported joint type: {joint_type!r}")


def _resolve_mesh(package: Path, urdf: Path, filename: str) -> Path:
    raw = str(filename).replace("\\", "/").strip()
    if not raw:
        raise SimulatorFreeOracleError("collision mesh filename is empty")
    if (
        raw.startswith("file://")
        or Path(raw).is_absolute()
        or (len(raw) >= 3 and raw[1] == ":" and raw[2] == "/")
    ):
        raise SimulatorFreeOracleError(
            f"absolute/file URI collision mesh reference is forbidden: {filename}"
        )
    if raw.startswith("package://"):
        remainder = raw[len("package://") :]
        components = remainder.split("/", 1)
        relative = components[1] if len(components) == 2 else components[0]
        candidate = package / relative
    else:
        candidate = urdf.parent / raw
    candidate = candidate.resolve(strict=False)
    try:
        candidate.relative_to(package)
    except ValueError as exc:
        raise SimulatorFreeOracleError(
            f"collision mesh escapes package: {filename}"
        ) from exc
    if not candidate.is_file():
        raise FileNotFoundError(f"collision mesh is missing: {filename}")
    return candidate.resolve(strict=True)


def _mesh_geometry(
    path: Path, scale: Sequence[float]
) -> tuple[Any, np.ndarray, np.ndarray]:
    import fcl
    import trimesh

    loaded = trimesh.load(path, force="scene", process=False)
    if isinstance(loaded, trimesh.Scene):
        loaded = loaded.dump(concatenate=True)
    if not isinstance(loaded, trimesh.Trimesh):
        raise SimulatorFreeOracleError(
            f"collision mesh did not resolve to triangles: {path}"
        )
    mesh = loaded.copy()
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.faces, dtype=np.int32)
    scale_vector = np.asarray(scale, dtype=np.float64)
    if (
        vertices.ndim != 2
        or vertices.shape[1:] != (3,)
        or faces.ndim != 2
        or faces.shape[1:] != (3,)
        or len(vertices) == 0
        or len(faces) == 0
    ):
        raise SimulatorFreeOracleError(
            f"collision mesh is empty or non-triangular: {path}"
        )
    vertices = vertices * scale_vector
    if not np.all(np.isfinite(vertices)):
        raise SimulatorFreeOracleError(
            f"collision mesh contains non-finite vertices: {path}"
        )
    model = fcl.BVHModel()
    model.beginModel(len(vertices), len(faces))
    model.addSubModel(vertices, faces)
    model.endModel()
    return model, np.min(vertices, axis=0), np.max(vertices, axis=0)


def _fcl_transform(matrix: np.ndarray) -> Any:
    import fcl

    return fcl.Transform(
        np.asarray(matrix[:3, :3], dtype=np.float64),
        np.asarray(matrix[:3, 3], dtype=np.float64),
    )


def _world_aabb(
    lower: np.ndarray, upper: np.ndarray, transform: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    corners = np.asarray(
        [
            (x, y, z, 1.0)
            for x in (float(lower[0]), float(upper[0]))
            for y in (float(lower[1]), float(upper[1]))
            for z in (float(lower[2]), float(upper[2]))
        ],
        dtype=np.float64,
    )
    world = (transform @ corners.T).T[:, :3]
    return np.min(world, axis=0), np.max(world, axis=0)


def _aabbs_overlap(
    left: tuple[np.ndarray, np.ndarray], right: tuple[np.ndarray, np.ndarray]
) -> bool:
    return bool(np.all(left[1] >= right[0]) and np.all(right[1] >= left[0]))


def _world_to_local(
    transform: np.ndarray, point: Sequence[float]
) -> tuple[float, float, float]:
    vector = np.asarray(point, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise SimulatorFreeOracleError("contact point is not a finite 3-vector")
    local = transform[:3, :3].T @ (vector - transform[:3, 3])
    return tuple(float(value) for value in local)


@dataclass(frozen=True)
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: tuple[float, float, float]
    movable_index: int | None


@dataclass(frozen=True)
class CollisionElement:
    link_name: str
    element_index: int
    authored_name: str | None
    element_id: str
    geometry_kind: str
    geometry: Any
    local_transform: np.ndarray
    local_aabb_lower: np.ndarray
    local_aabb_upper: np.ndarray


@dataclass(frozen=True)
class _WorldElement:
    source: CollisionElement
    link_transform: np.ndarray
    element_transform: np.ndarray
    aabb: tuple[np.ndarray, np.ndarray]
    collision_object: Any


class SimulatorFreeModel:
    def __init__(
        self,
        *,
        urdf: Path,
        package: Path,
        links: Sequence[str],
        root_link: str,
        joints: Sequence[Joint],
        outgoing: Mapping[str, Sequence[Joint]],
        collision_elements: Sequence[CollisionElement],
    ) -> None:
        self.urdf = Path(urdf)
        self.package = Path(package)
        self.links = tuple(links)
        self.root_link = root_link
        self.joints = tuple(joints)
        self.outgoing = {name: tuple(values) for name, values in outgoing.items()}
        self.collision_elements = tuple(collision_elements)
        self.movable_joints = tuple(
            joint for joint in self.joints if joint.movable_index is not None
        )
        self.direct_parent_pairs = frozenset(
            frozenset((joint.parent, joint.child)) for joint in self.joints
        )

    def _joint_value_map(
        self, values: Mapping[str, float] | Sequence[float]
    ) -> dict[str, float]:
        if isinstance(values, Mapping):
            unknown = set(values) - {joint.name for joint in self.movable_joints}
            if unknown:
                raise SimulatorFreeOracleError(
                    f"joint state contains unknown names: {sorted(unknown)}"
                )
            result = {
                joint.name: float(values.get(joint.name, 0.0))
                for joint in self.movable_joints
            }
        else:
            if isinstance(values, (str, bytes)) or len(values) != len(
                self.movable_joints
            ):
                raise SimulatorFreeOracleError(
                    "joint state length does not match the non-fixed URDF joint count"
                )
            result = {
                joint.name: float(value)
                for joint, value in zip(self.movable_joints, values, strict=True)
            }
        if any(not math.isfinite(value) for value in result.values()):
            raise SimulatorFreeOracleError("joint state contains non-finite values")
        return result

    def forward_kinematics(
        self, values: Mapping[str, float] | Sequence[float]
    ) -> dict[str, np.ndarray]:
        joint_values = self._joint_value_map(values)
        transforms = {self.root_link: np.eye(4, dtype=np.float64)}
        queue = [self.root_link]
        while queue:
            parent = queue.pop(0)
            for joint in self.outgoing.get(parent, ()):
                motion = _motion_transform(
                    joint.joint_type,
                    joint.axis,
                    joint_values.get(joint.name, 0.0),
                )
                child_transform = transforms[parent] @ joint.origin @ motion
                if not np.all(np.isfinite(child_transform)):
                    raise SimulatorFreeOracleError("FK produced a non-finite transform")
                if joint.child in transforms:
                    raise SimulatorFreeOracleError(
                        f"FK reached link more than once: {joint.child}"
                    )
                transforms[joint.child] = child_transform
                queue.append(joint.child)
        if set(transforms) != set(self.links):
            missing = sorted(set(self.links) - set(transforms))
            raise SimulatorFreeOracleError(f"FK did not reach every link: {missing}")
        return transforms

    def _world_elements(
        self, transforms: Mapping[str, np.ndarray]
    ) -> list[_WorldElement]:
        import fcl

        result: list[_WorldElement] = []
        for source in self.collision_elements:
            link_transform = transforms[source.link_name]
            element_transform = link_transform @ source.local_transform
            result.append(
                _WorldElement(
                    source=source,
                    link_transform=link_transform,
                    element_transform=element_transform,
                    aabb=_world_aabb(
                        source.local_aabb_lower,
                        source.local_aabb_upper,
                        element_transform,
                    ),
                    collision_object=fcl.CollisionObject(
                        source.geometry, _fcl_transform(element_transform)
                    ),
                )
            )
        return result

    def object_bbox_diagonal_m(
        self, values: Mapping[str, float] | Sequence[float] | None = None
    ) -> float:
        state: Mapping[str, float] | Sequence[float] = {} if values is None else values
        world = self._world_elements(self.forward_kinematics(state))
        if not world:
            raise SimulatorFreeOracleError("URDF has no collision elements")
        lower = np.min(np.asarray([entry.aabb[0] for entry in world]), axis=0)
        upper = np.max(np.asarray([entry.aabb[1] for entry in world]), axis=0)
        diagonal = float(np.linalg.norm(upper - lower))
        if not math.isfinite(diagonal) or diagonal <= 0.0:
            raise SimulatorFreeOracleError("collision AABB diagonal is not positive")
        return diagonal

    def observe(
        self,
        values: Mapping[str, float] | Sequence[float],
        *,
        phase: str = "rest",
        dataset: str = "",
        asset_id: str = "",
        urdf_sha256: str | None = None,
        intended_contact_registry: Mapping[str, Any] | None = None,
        penetration_threshold_m: float = PENETRATION_THRESHOLD_M,
        max_recorded_contacts: int = MAX_RECORDED_CONTACTS,
    ) -> dict[str, Any]:
        import fcl

        threshold = float(penetration_threshold_m)
        if not math.isfinite(threshold) or threshold < 0.0:
            raise SimulatorFreeOracleError(
                "penetration threshold must be finite and non-negative"
            )
        if max_recorded_contacts < 0:
            raise SimulatorFreeOracleError("max_recorded_contacts must be non-negative")
        transforms = self.forward_kinematics(values)
        world = self._world_elements(transforms)
        resolved_urdf_sha256 = urdf_sha256 or sha256_file(self.urdf)

        raw_contacts: list[dict[str, Any]] = []
        all_contact_count = 0
        non_adjacent_contact_count = 0
        all_intersection_count = 0
        non_adjacent_intersection_count = 0
        adjusted_all_intersection_count = 0
        adjusted_non_adjacent_intersection_count = 0
        raw_all_illegal = 0
        raw_non_adjacent_illegal = 0
        adjusted_all_illegal = 0
        adjusted_non_adjacent_illegal = 0
        intended_contact_count = 0
        unknown_depth_count = 0
        depth_values: list[float] = []
        non_adjacent_depth_values: list[float] = []
        adjusted_depth_values: list[float] = []
        adjusted_non_adjacent_depth_values: list[float] = []
        candidate_pair_count = 0
        narrowphase_pair_count = 0
        truncated_contact_pair_count = 0

        matcher = None
        if intended_contact_registry is not None:
            try:
                from table4_intended_contact_policy import match_contact
            except ImportError:
                import importlib.util

                policy_path = Path(__file__).with_name(
                    "table4_intended_contact_policy.py"
                )
                spec = importlib.util.spec_from_file_location(
                    "table4_intended_contact_policy", policy_path
                )
                if spec is None or spec.loader is None:
                    raise SimulatorFreeOracleError(
                        "cannot load intended-contact policy"
                    )
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                match_contact = module.match_contact
            matcher = match_contact

        for left_index, left in enumerate(world):
            for right in world[left_index + 1 :]:
                if left.source.link_name == right.source.link_name:
                    continue
                candidate_pair_count += 1
                if not _aabbs_overlap(left.aabb, right.aabb):
                    continue
                narrowphase_pair_count += 1
                collision_result = fcl.CollisionResult()
                collision_count = int(
                    fcl.collide(
                        left.collision_object,
                        right.collision_object,
                        fcl.CollisionRequest(
                            num_max_contacts=64,
                            enable_contact=True,
                        ),
                        collision_result,
                    )
                )
                if collision_count <= 0:
                    continue

                pair = frozenset((left.source.link_name, right.source.link_name))
                non_adjacent = pair not in self.direct_parent_pairs
                all_intersection_count += 1
                non_adjacent_intersection_count += int(non_adjacent)
                pair_contacts = list(getattr(collision_result, "contacts", ()) or ())
                if not pair_contacts:
                    pair_contacts = [None]
                evidence_truncated = len(pair_contacts) >= 64
                truncated_contact_pair_count += int(evidence_truncated)

                pair_all_intended = not evidence_truncated
                pair_has_illegal = False
                pair_adjusted_has_illegal = False
                for contact in pair_contacts:
                    all_contact_count += 1
                    non_adjacent_contact_count += int(non_adjacent)
                    depth: float | None = None
                    world_position: tuple[float, float, float] | None = None
                    if contact is not None:
                        raw_depth = getattr(contact, "penetration_depth", None)
                        try:
                            numeric_depth = float(raw_depth)
                        except (TypeError, ValueError):
                            numeric_depth = math.nan
                        if math.isfinite(numeric_depth):
                            depth = max(0.0, numeric_depth)
                            depth_values.append(depth)
                            if non_adjacent:
                                non_adjacent_depth_values.append(depth)
                        raw_position = getattr(contact, "pos", None)
                        if raw_position is not None:
                            try:
                                position_values = tuple(
                                    float(value) for value in raw_position
                                )
                            except (TypeError, ValueError):
                                position_values = ()
                            if len(position_values) == 3 and all(
                                math.isfinite(value) for value in position_values
                            ):
                                world_position = position_values  # type: ignore[assignment]

                    unknown_depth = depth is None
                    illegal = unknown_depth or depth > threshold
                    pair_has_illegal = pair_has_illegal or illegal
                    unknown_depth_count += int(unknown_depth)

                    local_a = (
                        _world_to_local(left.link_transform, world_position)
                        if world_position is not None
                        else None
                    )
                    local_b = (
                        _world_to_local(right.link_transform, world_position)
                        if world_position is not None
                        else None
                    )
                    match = {
                        "intended_contact": False,
                        "reason": "intended_contact_registry_not_configured",
                    }
                    if matcher is not None:
                        match = matcher(
                            intended_contact_registry,
                            dataset=dataset,
                            asset_id=asset_id,
                            urdf_sha256=resolved_urdf_sha256,
                            phase=phase,
                            link_a_name=left.source.link_name,
                            link_b_name=right.source.link_name,
                            penetration_depth_m=depth
                            if depth is not None
                            else math.nan,
                            local_position_a_m=local_a,
                            local_position_b_m=local_b,
                            collision_element_a_name=left.source.authored_name,
                            collision_element_b_name=right.source.authored_name,
                        )
                    intended = bool(match.get("intended_contact"))
                    intended_contact_count += int(intended)
                    pair_all_intended = pair_all_intended and intended
                    adjusted_illegal = illegal and not intended
                    pair_adjusted_has_illegal = (
                        pair_adjusted_has_illegal or adjusted_illegal
                    )
                    raw_all_illegal += int(illegal)
                    raw_non_adjacent_illegal += int(illegal and non_adjacent)
                    adjusted_all_illegal += int(adjusted_illegal)
                    adjusted_non_adjacent_illegal += int(
                        adjusted_illegal and non_adjacent
                    )
                    if depth is not None and not intended:
                        adjusted_depth_values.append(depth)
                        if non_adjacent:
                            adjusted_non_adjacent_depth_values.append(depth)

                    if len(raw_contacts) < max_recorded_contacts:
                        raw_contacts.append(
                            {
                                "link_a_name": left.source.link_name,
                                "link_b_name": right.source.link_name,
                                "collision_element_a_name": left.source.authored_name,
                                "collision_element_b_name": right.source.authored_name,
                                "collision_element_a_id": left.source.element_id,
                                "collision_element_b_id": right.source.element_id,
                                "geometry_kind_a": left.source.geometry_kind,
                                "geometry_kind_b": right.source.geometry_kind,
                                "world_position_m": world_position,
                                "local_position_a_m": local_a,
                                "local_position_b_m": local_b,
                                "penetration_depth_proxy_m": depth,
                                "penetration_depth_m": depth,
                                "depth_proxy_available": depth is not None,
                                "illegal_penetration": illegal,
                                "non_adjacent": non_adjacent,
                                "intended_contact": intended,
                                "intended_contact_match": dict(match),
                            }
                        )

                if evidence_truncated:
                    # The backend may have omitted additional contacts.  They
                    # cannot be proven shallow or inside a reviewed region.
                    unknown_depth_count += 1
                    raw_all_illegal += 1
                    raw_non_adjacent_illegal += int(non_adjacent)
                    adjusted_all_illegal += 1
                    adjusted_non_adjacent_illegal += int(non_adjacent)

                adjusted_all_intersection_count += int(not pair_all_intended)
                adjusted_non_adjacent_intersection_count += int(
                    non_adjacent and not pair_all_intended
                )
                # These are pair-level diagnostics; the compatibility metrics
                # above remain contact-level to match the historical oracle.
                _ = pair_has_illegal, pair_adjusted_has_illegal

        raw_max = max(depth_values, default=0.0)
        adjusted_max = max(adjusted_depth_values, default=0.0)
        return {
            "schema_version": ORACLE_SCHEMA,
            "collision_oracle": ORACLE_POLICY_ID,
            "phase": phase,
            "penetration_threshold_m": threshold,
            "contact_depth_semantics": "fcl_contact_depth_proxy_not_global_mesh_mtd",
            "surface_contact_allowed_in_penetration_view": True,
            "same_link_pairs_excluded": True,
            "direct_parent_pairs_excluded_from_non_adjacent_only": True,
            "candidate_collision_element_pair_count": candidate_pair_count,
            "narrowphase_collision_element_pair_count": narrowphase_pair_count,
            "truncated_contact_evidence_pair_count": truncated_contact_pair_count,
            "all_pair_contact_count": all_contact_count,
            "non_adjacent_contact_count": non_adjacent_contact_count,
            "all_pair_intersection_count": all_intersection_count,
            "non_adjacent_intersection_count": non_adjacent_intersection_count,
            "adjusted_all_pair_intersection_count": adjusted_all_intersection_count,
            "adjusted_non_adjacent_intersection_count": adjusted_non_adjacent_intersection_count,
            "raw_all_pair_illegal_penetration_count": raw_all_illegal,
            "raw_non_adjacent_illegal_penetration_count": raw_non_adjacent_illegal,
            "all_pair_illegal_penetration_count": adjusted_all_illegal,
            "non_adjacent_illegal_penetration_count": adjusted_non_adjacent_illegal,
            "raw_all_pair_max_penetration_proxy_m": raw_max,
            "raw_non_adjacent_max_penetration_proxy_m": max(
                non_adjacent_depth_values, default=0.0
            ),
            "all_pair_max_penetration_m": adjusted_max,
            "non_adjacent_max_penetration_m": max(
                adjusted_non_adjacent_depth_values, default=0.0
            ),
            "unknown_contact_depth_count": unknown_depth_count,
            "intended_contact_count": intended_contact_count,
            "recorded_contact_count": len(raw_contacts),
            "unrecorded_contact_count": max(0, all_contact_count - len(raw_contacts)),
            "raw_contacts": raw_contacts,
        }


def load_model(urdf: Path, package: Path | None = None) -> SimulatorFreeModel:
    import fcl

    urdf = Path(urdf).resolve(strict=True)
    package = Path(package if package is not None else urdf.parent).resolve(strict=True)
    if not package.is_dir():
        raise SimulatorFreeOracleError(f"package is not a directory: {package}")
    try:
        urdf.relative_to(package)
    except ValueError as exc:
        raise SimulatorFreeOracleError("URDF is outside its declared package") from exc
    root = ET.parse(urdf).getroot()
    if root.tag != "robot":
        raise SimulatorFreeOracleError("URDF root must be <robot>")

    link_nodes = root.findall("link")
    links = [str(link.get("name") or "") for link in link_nodes]
    if not links or any(not name for name in links) or len(links) != len(set(links)):
        raise SimulatorFreeOracleError("URDF link names must be non-empty and unique")
    link_set = set(links)
    outgoing: dict[str, list[Joint]] = {name: [] for name in links}
    child_links: set[str] = set()
    joint_names: set[str] = set()
    joints: list[Joint] = []
    movable_index = 0
    for xml_index, node in enumerate(root.findall("joint")):
        name = str(node.get("name") or "")
        joint_type = str(node.get("type") or "")
        parent_node, child_node = node.find("parent"), node.find("child")
        parent = str(parent_node.get("link") or "") if parent_node is not None else ""
        child = str(child_node.get("link") or "") if child_node is not None else ""
        if not name or name in joint_names:
            raise SimulatorFreeOracleError(
                "URDF joint names must be non-empty and unique"
            )
        joint_names.add(name)
        if joint_type not in SUPPORTED_JOINT_TYPES:
            raise SimulatorFreeOracleError(
                f"unsupported joint type for simulator-free FK: {joint_type!r}"
            )
        if parent not in link_set or child not in link_set or parent == child:
            raise SimulatorFreeOracleError(
                f"joint {name!r} has invalid parent/child links"
            )
        if child in child_links:
            raise SimulatorFreeOracleError(f"link {child!r} has multiple parent joints")
        child_links.add(child)
        axis_node = node.find("axis")
        axis = _finite_values(
            axis_node.get("xyz") if axis_node is not None else None,
            count=3,
            default=(1.0, 0.0, 0.0),
            label=f"joint {name!r} axis",
        )
        if (
            joint_type != "fixed"
            and math.sqrt(sum(value * value for value in axis)) <= 0.0
        ):
            raise SimulatorFreeOracleError(f"joint {name!r} axis is zero")
        joint = Joint(
            name=name,
            joint_type=joint_type,
            parent=parent,
            child=child,
            origin=_origin_transform(node.find("origin"), f"joint {name!r} origin"),
            axis=axis,  # type: ignore[arg-type]
            movable_index=None if joint_type == "fixed" else movable_index,
        )
        if joint_type != "fixed":
            movable_index += 1
        joints.append(joint)
        outgoing[parent].append(joint)

    roots = [name for name in links if name not in child_links]
    if len(roots) != 1:
        raise SimulatorFreeOracleError(
            f"URDF must have exactly one root link, found {len(roots)}"
        )
    root_link = roots[0]
    visited: set[str] = set()
    queue = [root_link]
    while queue:
        link = queue.pop(0)
        if link in visited:
            raise SimulatorFreeOracleError("URDF joint graph contains a cycle")
        visited.add(link)
        queue.extend(joint.child for joint in outgoing[link])
    if visited != link_set:
        raise SimulatorFreeOracleError("URDF joint graph is disconnected or cyclic")

    elements: list[CollisionElement] = []
    for link in link_nodes:
        link_name = str(link.get("name"))
        seen_authored_names: set[str] = set()
        for element_index, collision in enumerate(link.findall("collision")):
            authored_name = str(collision.get("name") or "").strip() or None
            if authored_name is not None:
                if authored_name in seen_authored_names:
                    raise SimulatorFreeOracleError(
                        f"duplicate collision name on link {link_name!r}: {authored_name!r}"
                    )
                seen_authored_names.add(authored_name)
            geometry_node = collision.find("geometry")
            shapes = list(geometry_node) if geometry_node is not None else []
            if len(shapes) != 1:
                raise SimulatorFreeOracleError(
                    f"{link_name}[{element_index}] must contain exactly one collision shape"
                )
            shape = shapes[0]
            kind = str(shape.tag)
            if kind not in SUPPORTED_GEOMETRY_TYPES:
                raise SimulatorFreeOracleError(
                    f"unsupported collision geometry: {kind!r}"
                )
            if kind == "box":
                size = np.asarray(
                    _finite_values(
                        shape.get("size"),
                        count=3,
                        default=(),
                        label="box size",
                    ),
                    dtype=np.float64,
                )
                if np.any(size <= 0.0):
                    raise SimulatorFreeOracleError("box size must be positive")
                geometry = fcl.Box(*[float(value) for value in size])
                lower, upper = -size / 2.0, size / 2.0
            elif kind == "sphere":
                radius = _positive(shape.get("radius"), "sphere radius")
                geometry = fcl.Sphere(radius)
                lower = np.full(3, -radius, dtype=np.float64)
                upper = np.full(3, radius, dtype=np.float64)
            elif kind == "cylinder":
                radius = _positive(shape.get("radius"), "cylinder radius")
                length = _positive(shape.get("length"), "cylinder length")
                geometry = fcl.Cylinder(radius, length)
                lower = np.asarray((-radius, -radius, -length / 2.0), dtype=np.float64)
                upper = -lower
            elif kind == "capsule":
                radius = _positive(shape.get("radius"), "capsule radius")
                length = _positive(shape.get("length"), "capsule length")
                geometry = fcl.Capsule(radius, length)
                extent = length / 2.0 + radius
                lower = np.asarray((-radius, -radius, -extent), dtype=np.float64)
                upper = -lower
            else:
                scale = _finite_values(
                    shape.get("scale"),
                    count=3,
                    default=(1.0, 1.0, 1.0),
                    label="mesh scale",
                )
                if any(value <= 0.0 for value in scale):
                    raise SimulatorFreeOracleError("mesh scale must be positive")
                mesh_path = _resolve_mesh(
                    package, urdf, str(shape.get("filename") or "")
                )
                geometry, lower, upper = _mesh_geometry(mesh_path, scale)

            elements.append(
                CollisionElement(
                    link_name=link_name,
                    element_index=element_index,
                    authored_name=authored_name,
                    element_id=f"{link_name}:{element_index}",
                    geometry_kind=kind,
                    geometry=geometry,
                    local_transform=_origin_transform(
                        collision.find("origin"),
                        f"collision {link_name}[{element_index}] origin",
                    ),
                    local_aabb_lower=np.asarray(lower, dtype=np.float64),
                    local_aabb_upper=np.asarray(upper, dtype=np.float64),
                )
            )

    return SimulatorFreeModel(
        urdf=urdf,
        package=package,
        links=links,
        root_link=root_link,
        joints=joints,
        outgoing=outgoing,
        collision_elements=elements,
    )


def runtime_identity() -> dict[str, Any]:
    distribution_names = {
        "numpy": "numpy",
        "scipy": "scipy",
        "trimesh": "trimesh",
        "python_fcl": "python-fcl",
    }
    module_names = {
        "numpy": "numpy",
        "scipy": "scipy",
        "trimesh": "trimesh",
        "python_fcl": "fcl",
    }
    modules: dict[str, Path] = {}
    versions: dict[str, str] = {}
    distribution_record_sha256: dict[str, str] = {}
    for label, module_name in module_names.items():
        spec = importlib.util.find_spec(module_name)
        if spec is None or not spec.origin:
            raise RuntimeError(
                f"simulator-free dependency is unavailable: {module_name}"
            )
        modules[label] = Path(spec.origin).resolve(strict=True)
        distribution = importlib.metadata.distribution(distribution_names[label])
        versions[label] = distribution.version
        record = distribution.read_text("RECORD")
        if record is None:
            raise RuntimeError(
                f"simulator-free dependency has no installed RECORD: {module_name}"
            )
        distribution_record_sha256[label] = hashlib.sha256(
            record.encode("utf-8")
        ).hexdigest()
    for label, module_name in {
        "python_fcl_extension": "fcl.fcl",
        "numpy_core_extension": "numpy._core._multiarray_umath",
    }.items():
        spec = importlib.util.find_spec(module_name)
        if spec is None or not spec.origin:
            raise RuntimeError(f"simulator-free binary is unavailable: {module_name}")
        modules[label] = Path(spec.origin).resolve(strict=True)
    loaded_simulators = sorted(
        name
        for name in FORBIDDEN_SIMULATOR_MODULES
        if name in sys.modules
        or any(module.startswith(f"{name}.") for module in sys.modules)
    )
    python_launcher = Path(os.path.abspath(sys.executable))
    python_realpath = python_launcher.resolve(strict=True)
    return {
        "schema_version": "table4_simulator_free_runtime_v1",
        "backend": ORACLE_POLICY_ID,
        "python_executable": str(python_launcher),
        "python_realpath": str(python_realpath),
        "python_realpath_sha256": sha256_file(python_realpath),
        "python_prefix": str(Path(sys.prefix).resolve(strict=True)),
        "python_base_prefix": str(Path(sys.base_prefix).resolve(strict=True)),
        "virtual_environment_active": sys.prefix != sys.base_prefix,
        "python_version": sys.version,
        "numpy_version": versions["numpy"],
        "scipy_version": versions["scipy"],
        "trimesh_version": versions["trimesh"],
        "python_fcl_version": versions["python_fcl"],
        "module_paths": {name: str(path) for name, path in modules.items()},
        "module_sha256": {name: sha256_file(path) for name, path in modules.items()},
        "distribution_record_sha256": distribution_record_sha256,
        "oracle_source_sha256": sha256_file(Path(__file__).resolve()),
        "simulator_modules_loaded": loaded_simulators,
        "simulator_free_runtime_verified": not loaded_simulators,
    }


__all__ = [
    "MAX_RECORDED_CONTACTS",
    "ORACLE_POLICY_ID",
    "ORACLE_SCHEMA",
    "PENETRATION_THRESHOLD_M",
    "SimulatorFreeModel",
    "SimulatorFreeOracleError",
    "load_model",
    "runtime_identity",
    "sha256_file",
]
