#!/usr/bin/env python3
"""Per-collision-element zero-margin proxy oracle for Table 4.

PyBullet's URDF importer gives convex hulls a 1 mm collision margin.  Changing
the dynamics of an articulated child or a compound link does not update every
child shape, so a link-level ``changeDynamics`` call cannot remove that
margin.  This module rebuilds each URDF collision element as a standalone
shape, calibrates that shape before composing chunked proxies for its source
link, and checks all proxies in an isolated collision-filter group.  Chunking
is required because this PyBullet build silently truncates ``createMultiBody``
requests above 127 fixed children.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


ORACLE_POLICY_ID = "pybullet_per_collision_element_numerical_zero_margin_proxy_v1"
NUMERICAL_ZERO_MARGIN_M = 1e-12
MARGIN_READBACK_ABS_TOLERANCE_M = 1e-15
NUMERICAL_ZERO_READBACK_MAX_M = (
    NUMERICAL_ZERO_MARGIN_M + MARGIN_READBACK_ABS_TOLERANCE_M
)
PENETRATION_THRESHOLD_M = 1e-6
PROXY_COLLISION_FILTER_GROUP = 2
SOURCE_COLLISION_FILTER_GROUP = 1
MAX_PROXY_COLLISION_ELEMENTS_PER_BODY = 64
PROXY_CHUNK_POLICY = "source_link_contiguous_collision_elements_max_64_v1"
SAME_SOURCE_AABB_FILTER_TOLERANCE_M = 1e-9


def _floats(
    text: str | None,
    *,
    count: int,
    default: Sequence[float],
    label: str,
) -> tuple[float, ...]:
    values = tuple(float(part) for part in text.split()) if text else tuple(default)
    if len(values) != count or any(not math.isfinite(value) for value in values):
        raise ValueError(f"{label} must contain {count} finite values")
    return values


def _positive(value: str | None, label: str) -> float:
    result = float(value) if value is not None else math.nan
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{label} must be positive and finite")
    return result


def _decode(value: Any) -> str:
    return value.decode("utf-8") if isinstance(value, bytes) else str(value)


def _root_link_name(root: ET.Element) -> str:
    links = [str(link.get("name") or "") for link in root.findall("link")]
    if any(not name for name in links) or len(links) != len(set(links)):
        raise ValueError("URDF link names must be non-empty and unique")
    children = {
        str(child.get("link") or "")
        for joint in root.findall("joint")
        for child in joint.findall("child")
    }
    roots = [name for name in links if name not in children]
    if len(roots) != 1:
        raise ValueError(f"URDF must have exactly one root link, found {len(roots)}")
    return roots[0]


def _link_index_by_name(bullet: Any, body: int, client: int) -> dict[str, int]:
    base_info = bullet.getBodyInfo(body, physicsClientId=client)
    result = {_decode(base_info[0]): -1}
    for index in range(bullet.getNumJoints(body, physicsClientId=client)):
        info = bullet.getJointInfo(body, index, physicsClientId=client)
        name = _decode(info[12])
        if not name or name in result:
            raise ValueError(f"duplicate or empty simulator link name: {name!r}")
        result[name] = index
    return result


def _origin(bullet: Any, collision: ET.Element) -> tuple[tuple[float, ...], tuple[float, ...]]:
    node = collision.find("origin")
    xyz = _floats(
        node.get("xyz") if node is not None else None,
        count=3,
        default=(0.0, 0.0, 0.0),
        label="collision origin xyz",
    )
    rpy = _floats(
        node.get("rpy") if node is not None else None,
        count=3,
        default=(0.0, 0.0, 0.0),
        label="collision origin rpy",
    )
    quaternion = tuple(float(value) for value in bullet.getQuaternionFromEuler(rpy))
    if len(quaternion) != 4 or any(not math.isfinite(value) for value in quaternion):
        raise ValueError("PyBullet returned an invalid collision origin quaternion")
    return xyz, quaternion


def _shape_kwargs(
    bullet: Any,
    collision: ET.Element,
    *,
    package: Path,
    urdf: Path,
    resolve_mesh: Callable[[Path, Path, str], Path],
) -> tuple[
    str,
    dict[str, Any],
    tuple[float, ...],
    tuple[float, ...],
]:
    geometry = collision.find("geometry")
    shapes = list(geometry) if geometry is not None else []
    if len(shapes) != 1:
        raise ValueError("each URDF collision element must contain exactly one geometry")
    shape = shapes[0]
    kind = str(shape.tag)
    frame_position, frame_orientation = _origin(bullet, collision)
    # Keep the created shape naked.  Passing a non-identity collision frame to
    # createCollisionShape makes Bullet wrap the geometry in a compound shape;
    # changeDynamics would then update only the wrapper margin.  The URDF
    # collision origin is applied by the fixed proxy child instead.
    kwargs: dict[str, Any] = {}
    if kind == "box":
        size = _floats(
            shape.get("size"), count=3, default=(), label="box size"
        )
        if any(value <= 0.0 for value in size):
            raise ValueError("box size must be positive")
        kwargs.update(
            shapeType=bullet.GEOM_BOX,
            halfExtents=tuple(value * 0.5 for value in size),
        )
    elif kind == "sphere":
        kwargs.update(
            shapeType=bullet.GEOM_SPHERE,
            radius=_positive(shape.get("radius"), "sphere radius"),
        )
    elif kind == "cylinder":
        kwargs.update(
            shapeType=bullet.GEOM_CYLINDER,
            radius=_positive(shape.get("radius"), "cylinder radius"),
            height=_positive(shape.get("length"), "cylinder length"),
        )
    elif kind == "capsule":
        kwargs.update(
            shapeType=bullet.GEOM_CAPSULE,
            radius=_positive(shape.get("radius"), "capsule radius"),
            height=_positive(shape.get("length"), "capsule length"),
        )
    elif kind == "mesh":
        filename = str(shape.get("filename") or "")
        mesh = resolve_mesh(package, urdf, filename)
        if not mesh.is_file():
            raise FileNotFoundError(f"collision mesh missing: {filename}")
        scale = _floats(
            shape.get("scale"),
            count=3,
            default=(1.0, 1.0, 1.0),
            label="mesh scale",
        )
        if any(value <= 0.0 for value in scale):
            raise ValueError("mesh scale must be positive")
        kwargs.update(
            shapeType=bullet.GEOM_MESH,
            fileName=str(mesh),
            meshScale=scale,
        )
    elif kind == "plane":
        normal = _floats(
            shape.get("normal"),
            count=3,
            default=(),
            label="plane normal",
        )
        norm = math.sqrt(sum(value * value for value in normal))
        if norm <= 0.0:
            raise ValueError("plane normal must be nonzero")
        kwargs.update(
            shapeType=bullet.GEOM_PLANE,
            planeNormal=tuple(value / norm for value in normal),
        )
    else:
        raise ValueError(f"zero-margin proxy does not support geometry {kind!r}")
    return kind, kwargs, frame_position, frame_orientation


def _calibrate_shape_margin(
    bullet: Any,
    client: int,
    shape: int,
    kind: str,
) -> dict[str, Any]:
    temporary = bullet.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=shape,
        physicsClientId=client,
    )
    if int(temporary) < 0:
        raise RuntimeError("failed to create temporary margin calibration body")
    try:
        before = float(
            bullet.getDynamicsInfo(temporary, -1, physicsClientId=client)[-1]
        )
        # Sphere and capsule margins encode their physical radius in Bullet.
        # Meshes and polyhedral primitives use a separable algorithmic margin.
        if kind in {"box", "cylinder", "mesh"}:
            bullet.changeDynamics(
                temporary,
                -1,
                collisionMargin=NUMERICAL_ZERO_MARGIN_M,
                physicsClientId=client,
            )
            strategy = "algorithmic_margin_set_to_numerical_zero"
            expected = NUMERICAL_ZERO_MARGIN_M
        else:
            strategy = "intrinsic_geometry_margin_preserved"
            expected = before
        after = float(
            bullet.getDynamicsInfo(temporary, -1, physicsClientId=client)[-1]
        )
        readback_matches = math.isclose(
            after,
            expected,
            rel_tol=0.0,
            abs_tol=MARGIN_READBACK_ABS_TOLERANCE_M,
        )
        if not math.isfinite(before) or not math.isfinite(after) or not readback_matches:
            raise RuntimeError(
                f"collision margin readback mismatch for {kind}: {after} != {expected}"
            )
        return {
            "geometry_kind": kind,
            "margin_before_m": before,
            "margin_after_m": after,
            "margin_strategy": strategy,
        }
    finally:
        bullet.removeBody(temporary, physicsClientId=client)


def _shape_margin_readback(
    bullet: Any,
    client: int,
    shape: int,
) -> float:
    """Read the naked shared shape after compound composition."""

    temporary = bullet.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=shape,
        basePosition=(1.0e6, 1.0e6, 1.0e6),
        physicsClientId=client,
    )
    if int(temporary) < 0:
        raise RuntimeError("failed to create post-compose margin audit body")
    try:
        return float(
            bullet.getDynamicsInfo(temporary, -1, physicsClientId=client)[-1]
        )
    finally:
        bullet.removeBody(temporary, physicsClientId=client)


def _proxy_body(
    bullet: Any,
    client: int,
    shapes: Sequence[int],
    frames: Sequence[tuple[Sequence[float], Sequence[float]]],
) -> int:
    if not shapes:
        raise ValueError("cannot create a proxy without collision shapes")
    if len(frames) != len(shapes):
        raise ValueError("proxy shape/frame count mismatch")
    count = len(shapes)
    if count > MAX_PROXY_COLLISION_ELEMENTS_PER_BODY:
        raise ValueError(
            "proxy collision-shape chunk exceeds the fixed-child safety limit: "
            f"{count} > {MAX_PROXY_COLLISION_ELEMENTS_PER_BODY}"
        )
    proxy = int(
        bullet.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            basePosition=(0.0, 0.0, 0.0),
            baseOrientation=(0.0, 0.0, 0.0, 1.0),
            physicsClientId=client,
            linkMasses=[0.0] * count,
            linkCollisionShapeIndices=list(shapes),
            linkVisualShapeIndices=[-1] * count,
            linkPositions=[tuple(frame[0]) for frame in frames],
            linkOrientations=[tuple(frame[1]) for frame in frames],
            linkInertialFramePositions=[(0.0, 0.0, 0.0)] * count,
            linkInertialFrameOrientations=[(0.0, 0.0, 0.0, 1.0)] * count,
            linkParentIndices=[0] * count,
            linkJointTypes=[bullet.JOINT_FIXED] * count,
            linkJointAxis=[(0.0, 0.0, 0.0)] * count,
        )
    )
    if proxy < 0:
        raise RuntimeError("failed to create zero-margin link proxy")
    return proxy


def _disable_same_source_chunk_contacts(
    bullet: Any,
    client: int,
    proxies: Sequence[int],
) -> int:
    """Disable only cross-chunk child pairs whose source-local AABBs overlap."""

    disabled = 0
    rows: list[
        tuple[int, int, tuple[float, float, float], tuple[float, float, float]]
    ] = []
    for proxy in proxies:
        for proxy_link in range(
            bullet.getNumJoints(proxy, physicsClientId=client)
        ):
            raw_lower, raw_upper = bullet.getAABB(
                proxy, proxy_link, physicsClientId=client
            )
            lower = tuple(float(value) for value in raw_lower)
            upper = tuple(float(value) for value in raw_upper)
            if any(
                not math.isfinite(value) for value in (*lower, *upper)
            ):
                raise RuntimeError(
                    "same-source proxy filter received a non-finite AABB"
                )
            rows.append((proxy, proxy_link, lower, upper))

    rows.sort(key=lambda row: row[2][0])
    active: list[
        tuple[int, int, tuple[float, float, float], tuple[float, float, float]]
    ] = []
    tolerance = SAME_SOURCE_AABB_FILTER_TOLERANCE_M
    for current in rows:
        current_proxy, current_link, current_lower, current_upper = current
        active = [
            other
            for other in active
            if other[3][0] + tolerance >= current_lower[0]
        ]
        for other_proxy, other_link, other_lower, other_upper in active:
            if other_proxy == current_proxy:
                continue
            if any(
                other_upper[axis] + tolerance < current_lower[axis]
                or current_upper[axis] + tolerance < other_lower[axis]
                for axis in (1, 2)
            ):
                continue
            bullet.setCollisionFilterPair(
                other_proxy,
                current_proxy,
                other_link,
                current_link,
                enableCollision=0,
                physicsClientId=client,
            )
            disabled += 1
        active.append(current)
    return disabled


def _link_world_frame(
    bullet: Any,
    body: int,
    client: int,
    link_index: int,
) -> tuple[Sequence[float], Sequence[float]]:
    if link_index >= 0:
        state = bullet.getLinkState(
            body,
            link_index,
            computeForwardKinematics=True,
            physicsClientId=client,
        )
        position, orientation = state[4], state[5]
    else:
        world_inertial_position, world_inertial_orientation = (
            bullet.getBasePositionAndOrientation(body, physicsClientId=client)
        )
        dynamics = bullet.getDynamicsInfo(body, -1, physicsClientId=client)
        link_to_inertial_position, link_to_inertial_orientation = dynamics[3], dynamics[4]
        inertial_to_link = bullet.invertTransform(
            link_to_inertial_position, link_to_inertial_orientation
        )
        position, orientation = bullet.multiplyTransforms(
            world_inertial_position,
            world_inertial_orientation,
            inertial_to_link[0],
            inertial_to_link[1],
        )
    values = tuple(float(value) for value in (*position, *orientation))
    if len(values) != 7 or any(not math.isfinite(value) for value in values):
        raise RuntimeError(f"non-finite FK frame for link {link_index}")
    return tuple(values[:3]), tuple(values[3:])


def _depth(contact: Sequence[Any]) -> float:
    distance = float(contact[8])
    if not math.isfinite(distance):
        raise RuntimeError("non-finite PyBullet contact distance")
    return max(0.0, -distance)


def _aggregate_contacts(
    contacts: Iterable[Sequence[Any]],
    direct_parent_pairs: set[frozenset[int]],
    *,
    link_pair: tuple[int, int] | None = None,
) -> dict[str, Any]:
    rows = list(contacts)
    all_depths = [_depth(contact) for contact in rows]
    if link_pair is None:
        pairs = [
            frozenset((int(contact[3]), int(contact[4]))) for contact in rows
        ]
    else:
        pair = frozenset(link_pair)
        pairs = [pair] * len(rows)
    non_adjacent = [
        (contact, depth)
        for contact, depth, pair in zip(rows, all_depths, pairs)
        if pair not in direct_parent_pairs
    ]
    return {
        "all_pair_contact_count": len(rows),
        "all_pair_illegal_penetration_count": sum(
            depth > PENETRATION_THRESHOLD_M for depth in all_depths
        ),
        "all_pair_max_penetration_m": max(all_depths or [0.0]),
        "non_adjacent_contact_count": len(non_adjacent),
        "non_adjacent_illegal_penetration_count": sum(
            depth > PENETRATION_THRESHOLD_M for _contact, depth in non_adjacent
        ),
        "non_adjacent_max_penetration_m": max(
            [depth for _contact, depth in non_adjacent] or [0.0]
        ),
    }


@dataclass
class ZeroMarginProxyOracle:
    bullet: Any
    body: int
    client: int
    proxies: dict[int, tuple[int, ...]]
    collision_element_count: int
    calibration_records: list[dict[str, Any]]
    same_source_filter_pair_count: int

    @classmethod
    def build(
        cls,
        bullet: Any,
        body: int,
        client: int,
        urdf: Path,
        package: Path,
        resolve_mesh: Callable[[Path, Path, str], Path],
    ) -> "ZeroMarginProxyOracle":
        root = ET.parse(urdf).getroot()
        root_name = _root_link_name(root)
        simulator_links = _link_index_by_name(bullet, body, client)
        if simulator_links.get(root_name) != -1:
            raise ValueError("URDF root link does not match the loaded PyBullet base")
        proxies: dict[int, tuple[int, ...]] = {}
        created_proxy_ids: list[int] = []
        records: list[dict[str, Any]] = []
        collision_element_count = 0
        same_source_filter_pair_count = 0
        try:
            for source_link_index in range(
                -1, bullet.getNumJoints(body, physicsClientId=client)
            ):
                bullet.setCollisionFilterGroupMask(
                    body,
                    source_link_index,
                    collisionFilterGroup=SOURCE_COLLISION_FILTER_GROUP,
                    collisionFilterMask=SOURCE_COLLISION_FILTER_GROUP,
                    physicsClientId=client,
                )
            for link in root.findall("link"):
                name = str(link.get("name") or "")
                if name not in simulator_links:
                    raise ValueError(f"URDF link missing from PyBullet body: {name}")
                link_index = simulator_links[name]
                collisions = link.findall("collision")
                if not collisions:
                    continue
                loaded = bullet.getCollisionShapeData(
                    body, link_index, physicsClientId=client
                )
                if len(loaded) != len(collisions):
                    raise RuntimeError(
                        f"collision element load mismatch for link {name}: "
                        f"{len(loaded)} != {len(collisions)}"
                    )
                shapes: list[int] = []
                frames: list[tuple[Sequence[float], Sequence[float]]] = []
                link_records: list[dict[str, Any]] = []
                for element_index, collision in enumerate(collisions):
                    kind, kwargs, frame_position, frame_orientation = _shape_kwargs(
                        bullet,
                        collision,
                        package=package,
                        urdf=urdf,
                        resolve_mesh=resolve_mesh,
                    )
                    shape_id = int(
                        bullet.createCollisionShape(
                            physicsClientId=client,
                            **kwargs,
                        )
                    )
                    if shape_id < 0:
                        raise RuntimeError(
                            f"failed to create {kind} proxy shape for link {name}"
                        )
                    calibration = _calibrate_shape_margin(
                        bullet, client, shape_id, kind
                    )
                    record = {
                        "source_link_name": name,
                        "source_link_index": link_index,
                        "collision_element_index": element_index,
                        **calibration,
                    }
                    records.append(record)
                    link_records.append(record)
                    shapes.append(shape_id)
                    frames.append((frame_position, frame_orientation))
                    collision_element_count += 1
                link_proxy_ids: list[int] = []
                chunk_limit = MAX_PROXY_COLLISION_ELEMENTS_PER_BODY
                for chunk_index, start in enumerate(
                    range(0, len(shapes), chunk_limit)
                ):
                    stop = min(start + chunk_limit, len(shapes))
                    chunk_shapes = shapes[start:stop]
                    chunk_frames = frames[start:stop]
                    chunk_records = link_records[start:stop]
                    proxy = _proxy_body(
                        bullet, client, chunk_shapes, chunk_frames
                    )
                    created_proxy_ids.append(proxy)
                    link_proxy_ids.append(proxy)
                    expected_joints = len(chunk_shapes)
                    observed_joints = bullet.getNumJoints(
                        proxy, physicsClientId=client
                    )
                    if observed_joints != expected_joints:
                        raise RuntimeError(
                            "proxy compound shape count mismatch for link "
                            f"{name} chunk {chunk_index}: "
                            f"{observed_joints} != {expected_joints}"
                        )
                    for proxy_shape_index, record, shape_id in zip(
                        range(expected_joints),
                        chunk_records,
                        chunk_shapes,
                        strict=True,
                    ):
                        readback = float(
                            bullet.getDynamicsInfo(
                                proxy,
                                proxy_shape_index,
                                physicsClientId=client,
                            )[-1]
                        )
                        expected_margin = float(record["margin_after_m"])
                        wrapper_readback_matches = (
                            0.0 <= readback <= NUMERICAL_ZERO_READBACK_MAX_M
                            if record["margin_strategy"]
                            == "algorithmic_margin_set_to_numerical_zero"
                            else (
                                0.0
                                <= readback
                                <= MARGIN_READBACK_ABS_TOLERANCE_M
                                or math.isclose(
                                    readback,
                                    expected_margin,
                                    rel_tol=0.0,
                                    abs_tol=MARGIN_READBACK_ABS_TOLERANCE_M,
                                )
                            )
                        )
                        if (
                            not math.isfinite(readback)
                            or not wrapper_readback_matches
                        ):
                            raise RuntimeError(
                                "compound proxy margin readback mismatch for link "
                                f"{name} element "
                                f"{record['collision_element_index']}: "
                                f"{readback} != {expected_margin}"
                            )
                        record["proxy_body_chunk_index"] = chunk_index
                        record["proxy_shape_link_index"] = proxy_shape_index
                        record["proxy_margin_readback_m"] = readback
                        naked_readback = _shape_margin_readback(
                            bullet, client, shape_id
                        )
                        if not math.isfinite(naked_readback) or not math.isclose(
                            naked_readback,
                            expected_margin,
                            rel_tol=0.0,
                            abs_tol=MARGIN_READBACK_ABS_TOLERANCE_M,
                        ):
                            raise RuntimeError(
                                "post-compose naked shape margin drift for link "
                                f"{name} element "
                                f"{record['collision_element_index']}: "
                                f"{naked_readback} != {expected_margin}"
                            )
                        record[
                            "post_compose_shape_margin_readback_m"
                        ] = naked_readback
                    for proxy_link_index in range(-1, expected_joints):
                        bullet.setCollisionFilterGroupMask(
                            proxy,
                            proxy_link_index,
                            collisionFilterGroup=PROXY_COLLISION_FILTER_GROUP,
                            collisionFilterMask=PROXY_COLLISION_FILTER_GROUP,
                            physicsClientId=client,
                        )
                same_source_filter_pair_count += _disable_same_source_chunk_contacts(
                    bullet, client, link_proxy_ids
                )
                proxies[link_index] = tuple(link_proxy_ids)
            if not proxies:
                raise ValueError("zero-margin oracle found no collision-bearing links")
            return cls(
                bullet=bullet,
                body=body,
                client=client,
                proxies=proxies,
                collision_element_count=collision_element_count,
                calibration_records=records,
                same_source_filter_pair_count=same_source_filter_pair_count,
            )
        except BaseException:
            for proxy in created_proxy_ids:
                bullet.removeBody(proxy, physicsClientId=client)
            raise

    def receipt(self) -> dict[str, Any]:
        adjusted = [
            row
            for row in self.calibration_records
            if row["margin_strategy"]
            == "algorithmic_margin_set_to_numerical_zero"
        ]
        intrinsic = len(self.calibration_records) - len(adjusted)
        chunk_counts: dict[tuple[int, int], int] = {}
        for record in self.calibration_records:
            key = (
                int(record["source_link_index"]),
                int(record["proxy_body_chunk_index"]),
            )
            chunk_counts[key] = chunk_counts.get(key, 0) + 1
        proxy_body_count = sum(len(bodies) for bodies in self.proxies.values())
        same_source_chunk_body_pair_count = sum(
            len(bodies) * (len(bodies) - 1) // 2
            for bodies in self.proxies.values()
        )
        same_source_proxy_link_pair_candidate_count = 0
        for source_link_index, bodies in self.proxies.items():
            counts = [
                chunk_counts[(source_link_index, chunk_index)]
                for chunk_index in range(len(bodies))
            ]
            same_source_proxy_link_pair_candidate_count += sum(
                left_count * right_count
                for left_position, left_count in enumerate(counts)
                for right_count in counts[left_position + 1 :]
            )
        return {
            "collision_oracle": ORACLE_POLICY_ID,
            "numerical_zero_margin_m": NUMERICAL_ZERO_MARGIN_M,
            "proxy_link_count": len(self.proxies),
            "proxy_source_link_count": len(self.proxies),
            "proxy_body_count": proxy_body_count,
            "proxy_collision_element_count": self.collision_element_count,
            "proxy_chunk_policy": PROXY_CHUNK_POLICY,
            "proxy_chunk_collision_element_limit": (
                MAX_PROXY_COLLISION_ELEMENTS_PER_BODY
            ),
            "chunked_source_link_count": sum(
                len(bodies) > 1 for bodies in self.proxies.values()
            ),
            "max_proxy_bodies_per_source_link": max(
                (len(bodies) for bodies in self.proxies.values()), default=0
            ),
            "max_collision_elements_per_proxy_body": max(
                chunk_counts.values(), default=0
            ),
            "same_source_chunk_body_pair_count": (
                same_source_chunk_body_pair_count
            ),
            "same_source_proxy_link_pair_filter_count": (
                self.same_source_filter_pair_count
            ),
            "same_source_proxy_link_pair_candidate_count": (
                same_source_proxy_link_pair_candidate_count
            ),
            "same_source_chunk_contact_policy": (
                "source_local_aabb_sweep_filter_with_observe_fail_safe"
            ),
            "algorithmic_margin_adjusted_element_count": len(adjusted),
            "intrinsic_geometry_margin_preserved_element_count": intrinsic,
            "proxy_collision_filter_group": PROXY_COLLISION_FILTER_GROUP,
            "source_collision_filter_group": SOURCE_COLLISION_FILTER_GROUP,
            "source_proxy_isolation_policy": (
                "disjoint_collision_filter_group_and_mask"
            ),
            "proxy_pair_coverage": (
                "all_source_link_pairs_across_all_proxy_body_chunks"
            ),
            "margin_calibration_records": list(self.calibration_records),
        }

    def observe(
        self,
        direct_parent_pairs: set[frozenset[int]],
    ) -> dict[str, Any]:
        for link_index, proxies in sorted(self.proxies.items()):
            position, orientation = _link_world_frame(
                self.bullet, self.body, self.client, link_index
            )
            for proxy in proxies:
                self.bullet.resetBasePositionAndOrientation(
                    proxy,
                    position,
                    orientation,
                    physicsClientId=self.client,
                )
        self.bullet.performCollisionDetection(physicsClientId=self.client)

        raw_contacts = list(
            self.bullet.getContactPoints(
                bodyA=self.body,
                bodyB=self.body,
                physicsClientId=self.client,
            )
        )
        raw = _aggregate_contacts(raw_contacts, direct_parent_pairs)
        raw_candidate_pairs = {
            tuple(sorted((int(contact[3]), int(contact[4]))))
            for contact in raw_contacts
        }
        if any(a == b for a, b in raw_candidate_pairs):
            raise RuntimeError("PyBullet returned a same-link self-contact")

        source_by_proxy = {
            proxy: source_link_index
            for source_link_index, proxies in self.proxies.items()
            for proxy in proxies
        }
        calibrated_rows: list[tuple[Sequence[Any], tuple[int, int]]] = []
        detected_pairs: set[tuple[int, int]] = set()
        ignored_same_source_chunk_contacts = 0
        for contact in self.bullet.getContactPoints(physicsClientId=self.client):
            body_a, body_b = int(contact[1]), int(contact[2])
            if body_a not in source_by_proxy or body_b not in source_by_proxy:
                continue
            source_a = source_by_proxy[body_a]
            source_b = source_by_proxy[body_b]
            if source_a == source_b:
                ignored_same_source_chunk_contacts += 1
                continue
            distance = float(contact[8])
            if not math.isfinite(distance):
                raise RuntimeError("zero-margin proxy contact distance is non-finite")
            if distance > 0.0:
                continue
            pair = tuple(sorted((source_a, source_b)))
            detected_pairs.add(pair)
            calibrated_rows.append((contact, pair))

        calibrated_contacts = [contact for contact, _pair in calibrated_rows]
        depths = [_depth(contact) for contact in calibrated_contacts]
        non_adjacent = [
            (contact, depth)
            for (contact, pair), depth in zip(calibrated_rows, depths)
            if frozenset(pair) not in direct_parent_pairs
        ]
        calibrated = {
            "all_pair_contact_count": len(calibrated_contacts),
            "all_pair_illegal_penetration_count": sum(
                depth > PENETRATION_THRESHOLD_M for depth in depths
            ),
            "all_pair_max_penetration_m": max(depths or [0.0]),
            "non_adjacent_contact_count": len(non_adjacent),
            "non_adjacent_illegal_penetration_count": sum(
                depth > PENETRATION_THRESHOLD_M
                for _contact, depth in non_adjacent
            ),
            "non_adjacent_max_penetration_m": max(
                [depth for _contact, depth in non_adjacent] or [0.0]
            ),
        }
        source_link_count = len(self.proxies)
        return {
            **calibrated,
            **{f"raw_{key}": value for key, value in raw.items()},
            "zero_margin_candidate_link_pair_count": len(raw_candidate_pairs),
            "zero_margin_rechecked_link_pair_count": (
                source_link_count * (source_link_count - 1) // 2
            ),
            "zero_margin_detected_link_pair_count": len(detected_pairs),
            "zero_margin_ignored_same_source_chunk_contact_count": (
                ignored_same_source_chunk_contacts
            ),
        }

    def object_bbox_diagonal(self) -> float:
        """Measure the collision union after syncing every proxy to source FK."""

        bounds: list[tuple[Sequence[float], Sequence[float]]] = []
        for source_link_index, proxies in sorted(self.proxies.items()):
            position, orientation = _link_world_frame(
                self.bullet, self.body, self.client, source_link_index
            )
            for proxy in proxies:
                self.bullet.resetBasePositionAndOrientation(
                    proxy,
                    position,
                    orientation,
                    physicsClientId=self.client,
                )
                for proxy_link_index in range(
                    self.bullet.getNumJoints(
                        proxy, physicsClientId=self.client
                    )
                ):
                    lower, upper = self.bullet.getAABB(
                        proxy, proxy_link_index, physicsClientId=self.client
                    )
                    if not all(
                        math.isfinite(float(value))
                        for value in (*lower, *upper)
                    ):
                        raise RuntimeError(
                            "zero-margin proxy AABB is non-finite"
                        )
                    bounds.append((lower, upper))
        if not bounds:
            raise RuntimeError("zero-margin proxy AABB union is empty")
        low = [
            min(float(pair[0][axis]) for pair in bounds) for axis in range(3)
        ]
        high = [
            max(float(pair[1][axis]) for pair in bounds) for axis in range(3)
        ]
        diagonal = math.sqrt(
            sum((upper - lower) ** 2 for lower, upper in zip(low, high))
        )
        if not math.isfinite(diagonal) or diagonal <= 0.0:
            raise RuntimeError("zero-margin proxy AABB diagonal is not positive")
        return diagonal

    def close(self) -> None:
        for proxies in self.proxies.values():
            for proxy in proxies:
                self.bullet.removeBody(proxy, physicsClientId=self.client)
        self.proxies.clear()
