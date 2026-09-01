#!/usr/bin/env python3
"""Run exact FCL collision queries plus conservative continuous T5 certificates.

The geometry is read from the freshly compiled formal URDF packages.  Bullet is
used only for forward kinematics; python-fcl performs exact triangle-mesh and
primitive collision/distance queries.  Direct parent-child pairs are excluded,
matching the frozen discrete protocol.  Every adjacent 11-state joint interval
and every adjacent 64-state Sobol interval receives a continuous-collision
clearance/motion-bound certificate; low-clearance intervals are recursively bisected.
Intervals that cannot be certified within the frozen depth are explicitly reported
as uncertified instead of being counted collision-free.  This deliberately avoids
python-fcl ``continuousCollide`` because it segfaulted on part of the frozen mesh
cohort in the retained diagnostic run.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
PYTHON = TEMPLATE_ROOT / ".venv/bin/python"
DEFAULT_MANIFEST = EXP_ROOT / "runtime/t5_formal_v1/gold_then_compile/formal_input_manifest.json"
DEFAULT_OUT = EXP_ROOT / "runtime/t5_formal_v1/ccd_adaptive_v3"
SINGLE_STATES = 11
MULTI_STATES = 64
ADAPTIVE_MAX_DEPTH = 4
CLEARANCE_ABSOLUTE_M = 0.002
CLEARANCE_BBOX_FRACTION = 0.005


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def values(raw: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if not raw:
        return default
    return tuple(float(item) for item in raw.split())


def rpy_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=float)
    return rz @ ry @ rx


def transform_matrix(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rpy_matrix(rpy)
    result[:3, 3] = xyz
    return result


def quaternion_matrix(pb: Any, position: Any, quaternion: Any) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = np.asarray(pb.getMatrixFromQuaternion(quaternion), dtype=float).reshape(3, 3)
    result[:3, 3] = np.asarray(position, dtype=float)
    return result


def fcl_transform(fcl: Any, matrix: np.ndarray) -> Any:
    return fcl.Transform(matrix[:3, :3], matrix[:3, 3])


def mesh_geometry(fcl: Any, trimesh: Any, filename: Path, scale: tuple[float, float, float]) -> tuple[Any, float]:
    loaded = trimesh.load(filename, force="scene", process=False)
    if isinstance(loaded, trimesh.Scene):
        meshes = [geometry.copy() for geometry in loaded.geometry.values()]
        if not meshes:
            raise ValueError(f"empty mesh scene: {filename}")
        loaded = trimesh.util.concatenate(meshes)
    vertices = np.asarray(loaded.vertices, dtype=np.float64) * np.asarray(scale, dtype=float)
    faces = np.asarray(loaded.faces, dtype=np.int32)
    if not len(vertices) or not len(faces):
        raise ValueError(f"empty collision mesh: {filename}")
    model = fcl.BVHModel()
    model.beginModel(len(vertices), len(faces))
    model.addSubModel(vertices, faces)
    model.endModel()
    return model, float(np.linalg.norm(vertices, axis=1).max())


def load_shapes(fcl: Any, trimesh: Any, urdf: Path, link_ids: dict[str, int]) -> tuple[list[dict[str, Any]], list[str]]:
    root = ET.parse(urdf).getroot()
    shapes: list[dict[str, Any]] = []
    issues: list[str] = []
    for link in root.findall("link"):
        name = link.attrib.get("name", "")
        link_id = link_ids.get(name)
        if link_id is None:
            issues.append(f"missing Bullet link mapping: {name}")
            continue
        for index, collision in enumerate(link.findall("collision")):
            origin = collision.find("origin")
            local = transform_matrix(
                values(origin.attrib.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0)),
                values(origin.attrib.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0)),
            )
            geometry = collision.find("geometry")
            if geometry is None:
                issues.append(f"{name}[{index}]: missing geometry")
                continue
            try:
                mesh = geometry.find("mesh")
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                sphere = geometry.find("sphere")
                if mesh is not None:
                    path = Path(mesh.attrib["filename"])
                    if not path.is_absolute():
                        path = urdf.parent / path
                    shape, radius = mesh_geometry(
                        fcl,
                        trimesh,
                        path.resolve(),
                        values(mesh.attrib.get("scale"), (1.0, 1.0, 1.0)),
                    )
                    kind = "mesh"
                elif box is not None:
                    size = values(box.attrib.get("size"), (0.0, 0.0, 0.0))
                    shape = fcl.Box(*size)
                    radius = float(np.linalg.norm(np.asarray(size, dtype=float) / 2.0))
                    kind = "box"
                elif cylinder is not None:
                    cylinder_radius = float(cylinder.attrib["radius"])
                    cylinder_length = float(cylinder.attrib["length"])
                    shape = fcl.Cylinder(cylinder_radius, cylinder_length)
                    radius = math.hypot(cylinder_radius, cylinder_length / 2.0)
                    kind = "cylinder"
                elif sphere is not None:
                    radius = float(sphere.attrib["radius"])
                    shape = fcl.Sphere(radius)
                    kind = "sphere"
                else:
                    raise ValueError("unsupported collision geometry")
                shapes.append(
                    {
                        "shape_id": f"{name}[{index}]",
                        "link_name": name,
                        "link_id": link_id,
                        "local": local,
                        "geometry": shape,
                        "radius": radius,
                        "kind": kind,
                    }
                )
            except Exception as exc:  # noqa: BLE001
                issues.append(f"{name}[{index}]: {type(exc).__name__}: {exc}")
    return shapes, issues


def sobol(count: int, dimension: int) -> np.ndarray:
    from scipy.stats import qmc

    if count != 64:
        raise ValueError("formal protocol requires 64 Sobol configurations")
    return qmc.Sobol(d=dimension, scramble=False).random_base2(m=6)


def run_asset(package: Path, asset_id: str) -> dict[str, Any]:
    import fcl
    import pybullet as pb
    import trimesh

    started = time.monotonic()
    urdf = package / "model.urdf"
    client = pb.connect(pb.DIRECT)
    body: int | None = None
    try:
        body = pb.loadURDF(
            str(urdf),
            useFixedBase=True,
            flags=pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        )
        base_name_raw = pb.getBodyInfo(body)[0]
        base_name = base_name_raw.decode() if isinstance(base_name_raw, bytes) else str(base_name_raw)
        link_ids = {base_name: -1}
        parents: dict[int, int] = {}
        joint_types: dict[int, int] = {}
        movable: list[dict[str, Any]] = []
        for index in range(pb.getNumJoints(body)):
            info = pb.getJointInfo(body, index)
            link_name = info[12].decode() if isinstance(info[12], bytes) else str(info[12])
            joint_name = info[1].decode() if isinstance(info[1], bytes) else str(info[1])
            link_ids[link_name] = index
            parents[index] = int(info[16])
            joint_types[index] = int(info[2])
            if int(info[2]) not in (pb.JOINT_REVOLUTE, pb.JOINT_PRISMATIC):
                continue
            lower, upper = float(info[8]), float(info[9])
            if not (math.isfinite(lower) and math.isfinite(upper) and lower <= upper):
                lower, upper = -math.pi, math.pi
            movable.append(
                {
                    "index": index,
                    "name": joint_name,
                    "lower": lower,
                    "upper": upper,
                    "default": (lower + upper) / 2.0,
                }
            )
        shapes, issues = load_shapes(fcl, trimesh, urdf, link_ids)
        # Bullet merges fixed-link subtrees and then excludes contacts across a
        # direct movable interface under URDF_USE_SELF_COLLISION_EXCLUDE_PARENT.
        # Reproduce that exact policy for FCL; using raw XML parent links would
        # incorrectly count intended hinge/interface contact against assets.
        dsu_parent = {link_id: link_id for link_id in link_ids.values()}

        def find(link_id: int) -> int:
            while dsu_parent[link_id] != link_id:
                dsu_parent[link_id] = dsu_parent[dsu_parent[link_id]]
                link_id = dsu_parent[link_id]
            return link_id

        def union(left: int, right: int) -> None:
            left_root, right_root = find(left), find(right)
            if left_root != right_root:
                dsu_parent[right_root] = left_root

        for child, parent in parents.items():
            if joint_types.get(child) == pb.JOINT_FIXED:
                union(child, parent)
        adjacent_clusters: set[frozenset[int]] = set()
        for child, parent in parents.items():
            if joint_types.get(child) != pb.JOINT_FIXED:
                adjacent_clusters.add(frozenset((find(child), find(parent))))
        pairs: list[tuple[int, int]] = []
        for left in range(len(shapes)):
            for right in range(left + 1, len(shapes)):
                a, b = shapes[left], shapes[right]
                cluster_a, cluster_b = find(a["link_id"]), find(b["link_id"])
                if cluster_a == cluster_b:
                    continue
                if frozenset((cluster_a, cluster_b)) in adjacent_clusters:
                    continue
                pairs.append((left, right))
        if len(shapes) == 0 or len(movable) == 0:
            raise RuntimeError(f"insufficient shapes/joints: shapes={len(shapes)}, joints={len(movable)}")

        # Asset scale controls the predeclared low-clearance trigger.
        lower_bounds: list[np.ndarray] = []
        upper_bounds: list[np.ndarray] = []
        for link_id in set(shape["link_id"] for shape in shapes):
            low, high = pb.getAABB(body, link_id)
            lower_bounds.append(np.asarray(low, dtype=float))
            upper_bounds.append(np.asarray(high, dtype=float))
        bbox_diag = float(np.linalg.norm(np.max(upper_bounds, axis=0) - np.min(lower_bounds, axis=0)))
        threshold = max(CLEARANCE_ABSOLUTE_M, CLEARANCE_BBOX_FRACTION * bbox_diag)

        state_cache: dict[tuple[float, ...], dict[str, Any]] = {}
        max_penetration = 0.0
        minimum_clearance = math.inf

        def transforms(config: tuple[float, ...]) -> list[np.ndarray]:
            for joint, value in zip(movable, config):
                pb.resetJointState(body, joint["index"], float(value))
            link_world: dict[int, np.ndarray] = {}
            base_pos, base_quat = pb.getBasePositionAndOrientation(body)
            link_world[-1] = quaternion_matrix(pb, base_pos, base_quat)
            for link_id in set(shape["link_id"] for shape in shapes if shape["link_id"] >= 0):
                state = pb.getLinkState(body, link_id, computeForwardKinematics=True)
                link_world[link_id] = quaternion_matrix(pb, state[4], state[5])
            return [link_world[shape["link_id"]] @ shape["local"] for shape in shapes]

        def evaluate(config: tuple[float, ...]) -> dict[str, Any]:
            nonlocal max_penetration, minimum_clearance
            key = tuple(round(float(value), 12) for value in config)
            if key in state_cache:
                return state_cache[key]
            matrices = transforms(config)
            state_min = math.inf
            state_penetration = 0.0
            collisions: list[dict[str, Any]] = []
            pair_distances: dict[tuple[int, int], float] = {}
            for left, right in pairs:
                obj_a = fcl.CollisionObject(shapes[left]["geometry"], fcl_transform(fcl, matrices[left]))
                obj_b = fcl.CollisionObject(shapes[right]["geometry"], fcl_transform(fcl, matrices[right]))
                distance_result = fcl.DistanceResult()
                distance = float(
                    fcl.distance(
                        obj_a,
                        obj_b,
                        fcl.DistanceRequest(enable_nearest_points=True, enable_signed_distance=True),
                        distance_result,
                    )
                )
                nonnegative_distance = max(0.0, distance)
                pair_distances[(left, right)] = nonnegative_distance
                state_min = min(state_min, nonnegative_distance)
                collision_result = fcl.CollisionResult()
                count = int(
                    fcl.collide(
                        obj_a,
                        obj_b,
                        fcl.CollisionRequest(num_max_contacts=8, enable_contact=True),
                        collision_result,
                    )
                )
                if count:
                    depths = [float(contact.penetration_depth) for contact in collision_result.contacts]
                    penetration = max(depths, default=0.0)
                    state_penetration = max(state_penetration, penetration)
                    if len(collisions) < 8:
                        collisions.append(
                            {
                                "shape_a": shapes[left]["shape_id"],
                                "shape_b": shapes[right]["shape_id"],
                                "max_penetration_m": penetration,
                            }
                        )
            if not pairs:
                state_min = math.inf
            max_penetration = max(max_penetration, state_penetration)
            minimum_clearance = min(minimum_clearance, state_min)
            result = {
                "config": list(key),
                "transforms": matrices,
                "collision_free": not collisions,
                "minimum_clearance_m": state_min,
                "max_penetration_m": state_penetration,
                "collisions": collisions,
                "pair_distances": pair_distances,
            }
            state_cache[key] = result
            return result

        interval_count = 0
        continuous_pair_certificate_count = 0
        adaptive_interval_count = 0
        uncertified_pair_interval_count = 0
        uncertified_examples: list[dict[str, Any]] = []

        def rotation_angle(left: np.ndarray, right: np.ndarray) -> float:
            relative = left[:3, :3].T @ right[:3, :3]
            cosine = min(1.0, max(-1.0, (float(np.trace(relative)) - 1.0) / 2.0))
            return math.acos(cosine)

        def motion_bound(shape: dict[str, Any], left: np.ndarray, right: np.ndarray) -> float:
            translation = float(np.linalg.norm(right[:3, 3] - left[:3, 3]))
            angle = rotation_angle(left, right)
            rotational = 2.0 * float(shape["radius"]) * math.sin(angle / 2.0)
            return translation + rotational

        def certify_interval(start: tuple[float, ...], end: tuple[float, ...], phase: str, depth: int = 0) -> tuple[bool, bool]:
            """Return (collision_observed, continuously_certified_free)."""
            nonlocal interval_count, continuous_pair_certificate_count
            nonlocal adaptive_interval_count, uncertified_pair_interval_count
            interval_count += 1
            a_state = evaluate(start)
            b_state = evaluate(end)
            if not a_state["collision_free"] or not b_state["collision_free"]:
                return True, False
            uncertain: list[tuple[int, int, float, float]] = []
            for left, right in pairs:
                clearance = min(
                    a_state["pair_distances"][(left, right)],
                    b_state["pair_distances"][(left, right)],
                )
                bound = motion_bound(
                    shapes[left], a_state["transforms"][left], b_state["transforms"][left]
                ) + motion_bound(
                    shapes[right], a_state["transforms"][right], b_state["transforms"][right]
                )
                # Distance between two moving closed sets can decrease by at
                # most the sum of their Hausdorff-motion bounds.
                if clearance > bound + 1e-9:
                    continuous_pair_certificate_count += 1
                else:
                    uncertain.append((left, right, clearance, bound))
            if not uncertain:
                return False, True
            if depth < ADAPTIVE_MAX_DEPTH:
                adaptive_interval_count += 1
                midpoint = tuple((left + right) / 2.0 for left, right in zip(start, end))
                left_hit, left_certified = certify_interval(start, midpoint, phase, depth + 1)
                right_hit, right_certified = certify_interval(midpoint, end, phase, depth + 1)
                return left_hit or right_hit, left_certified and right_certified
            uncertified_pair_interval_count += len(uncertain)
            for left, right, clearance, bound in uncertain:
                if len(uncertified_examples) >= 32:
                    break
                uncertified_examples.append(
                    {
                        "phase": phase,
                        "depth": depth,
                        "shape_a": shapes[left]["shape_id"],
                        "shape_b": shapes[right]["shape_id"],
                        "endpoint_clearance_m": clearance,
                        "motion_bound_m": bound,
                    }
                )
            return False, False

        defaults = tuple(float(joint["default"]) for joint in movable)
        evaluate(defaults)
        endpoint_pass = 0
        endpoint_total = 0
        joint_valid = 0
        for position, joint in enumerate(movable):
            samples = np.linspace(joint["lower"], joint["upper"], SINGLE_STATES)
            configs = []
            for sample in samples:
                config = list(defaults)
                config[position] = float(sample)
                configs.append(tuple(config))
            endpoint_total += 2
            endpoint_pass += int(evaluate(configs[0])["collision_free"])
            endpoint_pass += int(evaluate(configs[-1])["collision_free"])
            joint_hit = False
            joint_certified = True
            for index in range(len(configs) - 1):
                observed, certified = certify_interval(
                    configs[index], configs[index + 1], f"single:{joint['name']}:{index}"
                )
                joint_hit = observed or joint_hit
                joint_certified = certified and joint_certified
            joint_valid += int(
                not joint_hit
                and joint_certified
                and all(evaluate(config)["collision_free"] for config in configs)
            )

        multi_interval_count = 0
        multi_certified = True
        multi_hit = False
        if len(movable) > 1:
            unit = sobol(MULTI_STATES, len(movable))
            configs = [
                tuple(
                    joint["lower"] + float(u) * (joint["upper"] - joint["lower"])
                    for joint, u in zip(movable, row)
                )
                for row in unit
            ]
            for index in range(len(configs) - 1):
                multi_interval_count += 1
                observed, certified = certify_interval(
                    configs[index], configs[index + 1], f"multi_sobol:{index}"
                )
                multi_hit = observed or multi_hit
                multi_certified = certified and multi_certified

        discrete_collision_states = sum(not row["collision_free"] for row in state_cache.values())
        any_collision = bool(discrete_collision_states or multi_hit)
        all_intervals_certified = uncertified_pair_interval_count == 0
        # If exact FCL finds no intersections at any queried state or along any
        # continuously checked interval, exact-mesh overlap volume is zero.
        aor: float | None = 0.0 if not any_collision and all_intervals_certified else None
        serializable_states = [
            {
                key: value
                for key, value in row.items()
                if key not in {"transforms", "pair_distances"}
            }
            for row in state_cache.values()
        ]
        return {
            "schema_version": 1,
            "asset_id": asset_id,
            "package": str(package),
            "urdf_sha256": sha256(urdf),
            "collision_engine": "python-fcl 0.7 exact BVH collide + signed distance",
            "continuous_method": "clearance-Lipschitz motion-bound certificate with adaptive bisection",
            "kinematics_engine": "PyBullet resetJointState + forward kinematics",
            "self_collision_policy": "all exact collision shapes except same rigid cluster and clusters adjacent by one movable joint; matches Bullet fixed-link merging + EXCLUDE_PARENT",
            "joint_count": len(movable),
            "collision_shape_count": len(shapes),
            "eligible_shape_pair_count": len(pairs),
            "single_joint_states_per_joint": SINGLE_STATES,
            "multi_joint_sobol_states": MULTI_STATES if len(movable) > 1 else 0,
            "base_interval_count": len(movable) * (SINGLE_STATES - 1) + multi_interval_count,
            "evaluated_interval_count_including_adaptive": interval_count,
            "continuous_pair_certificate_count": continuous_pair_certificate_count,
            "adaptive_interval_count": adaptive_interval_count,
            "adaptive_max_depth": ADAPTIVE_MAX_DEPTH,
            "low_clearance_threshold_m": threshold,
            "bbox_diagonal_m": bbox_diag,
            "evaluated_discrete_state_count": len(state_cache),
            "discrete_collision_state_count": discrete_collision_states,
            "uncertified_pair_interval_count": uncertified_pair_interval_count,
            "uncertified_examples": uncertified_examples,
            "joint_level_geom_valid_count": joint_valid,
            "asset_level_geom_valid": (
                joint_valid == len(movable)
                and not any_collision
                and (len(movable) == 1 or multi_certified)
            ),
            "full_range_collision_free_on_evaluated_paths": (
                not any_collision and all_intervals_certified
            ),
            "average_overlap_ratio": aor,
            "average_overlap_ratio_note": (
                "0 because exact FCL found no intersections and every interval was continuously certified"
                if aor == 0.0
                else "not claimed because collision was observed or some intervals remained uncertified"
            ),
            "max_penetration_m": max_penetration,
            "minimum_clearance_m": None if math.isinf(minimum_clearance) else minimum_clearance,
            "endpoint_reachability": f"{endpoint_pass}/{endpoint_total}",
            "endpoint_reachability_rate": endpoint_pass / endpoint_total if endpoint_total else None,
            "shape_load_issues": issues,
            "state_records": serializable_states,
            "elapsed_s": time.monotonic() - started,
        }
    finally:
        if body is not None:
            pb.removeBody(body)
        pb.disconnect(client)


def worker(input_path: Path, output: Path) -> int:
    row = json.loads(input_path.read_text(encoding="utf-8"))
    started = time.monotonic()
    try:
        payload = run_asset(Path(row["copied_package"]), row["asset_id"])
    except BaseException as exc:  # noqa: BLE001
        import traceback

        payload = {
            "asset_id": row.get("asset_id"),
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc()[-10000:],
            "elapsed_s": time.monotonic() - started,
        }
    dump_json(output, payload)
    return 0 if not payload.get("error") else 1


def run_one(row: dict[str, Any], out: Path, timeout: float) -> dict[str, Any]:
    record = out / "assets" / f"{row['asset_id']}.json"
    if record.is_file():
        payload = json.loads(record.read_text(encoding="utf-8"))
        if not payload.get("error"):
            return payload
    input_path = out / "worker_inputs" / f"{row['asset_id']}.json"
    dump_json(input_path, row)
    env = os.environ.copy()
    for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[key] = "1"
    try:
        completed = subprocess.run(
            [str(PYTHON), str(Path(__file__).resolve()), "--worker", str(input_path), str(record)],
            cwd=TEMPLATE_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {"asset_id": row["asset_id"], "error": f"timeout({timeout}s)"}
    if record.is_file():
        return json.loads(record.read_text(encoding="utf-8"))
    return {
        "asset_id": row["asset_id"],
        "error": f"worker_exit_{completed.returncode}",
        "stderr": completed.stderr[-5000:],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=3600.0)
    parser.add_argument("--worker", nargs=2, metavar=("INPUT", "OUTPUT"))
    args = parser.parse_args()
    if args.worker:
        return worker(Path(args.worker[0]), Path(args.worker[1]))
    manifest = args.manifest.resolve()
    rows = json.loads(manifest.read_text(encoding="utf-8"))
    out = args.out.resolve()
    out.relative_to(EXP_ROOT.resolve())
    records: list[dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = {pool.submit(run_one, row, out, args.timeout): row for row in rows}
        for index, future in enumerate(concurrent.futures.as_completed(futures), 1):
            result = future.result()
            records.append(result)
            print(
                f"{index}/{len(rows)} {result.get('asset_id')} "
                f"{'ERROR' if result.get('error') else 'PASS' if result.get('asset_level_geom_valid') else 'INVALID_OR_UNCERTIFIED'}",
                flush=True,
            )
    records.sort(key=lambda row: row.get("asset_id", ""))
    failures = [row for row in records if row.get("error")]
    completed = [row for row in records if not row.get("error")]
    joint_total = sum(row["joint_count"] for row in completed)
    summary = {
        "schema_version": 1,
        "protocol": "t5_fcl_clearance_certificate_adaptive_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "input_manifest": str(manifest),
        "input_manifest_sha256": sha256(manifest),
        "asset_count": len(rows),
        "completed_assets": len(completed),
        "failed_assets": len(failures),
        "joint_count": joint_total,
        "joint_level_geom_valid": f"{sum(row['joint_level_geom_valid_count'] for row in completed)}/{joint_total}",
        "asset_level_geom_valid": f"{sum(row['asset_level_geom_valid'] for row in completed)}/{len(rows)}",
        "full_range_collision_free": f"{sum(row['full_range_collision_free_on_evaluated_paths'] for row in completed)}/{len(rows)}",
        "base_interval_count": sum(row["base_interval_count"] for row in completed),
        "evaluated_intervals_including_adaptive": sum(
            row["evaluated_interval_count_including_adaptive"] for row in completed
        ),
        "continuous_pair_certificates": sum(
            row["continuous_pair_certificate_count"] for row in completed
        ),
        "uncertified_pair_intervals": sum(
            row["uncertified_pair_interval_count"] for row in completed
        ),
        "discrete_collision_states": sum(
            row["discrete_collision_state_count"] for row in completed
        ),
        "adaptive_interval_count": sum(row["adaptive_interval_count"] for row in completed),
        "aor_mean": (
            sum(row["average_overlap_ratio"] for row in completed) / len(completed)
            if completed and all(row["average_overlap_ratio"] is not None for row in completed)
            else None
        ),
        "aor_max": (
            max(row["average_overlap_ratio"] for row in completed)
            if completed and all(row["average_overlap_ratio"] is not None for row in completed)
            else None
        ),
        "max_penetration_m": max((row["max_penetration_m"] for row in completed), default=None),
        "minimum_clearance_m": min(
            (row["minimum_clearance_m"] for row in completed if row["minimum_clearance_m"] is not None),
            default=None,
        ),
        "endpoint_reachability": f"{sum(int(row['endpoint_reachability'].split('/')[0]) for row in completed)}/{sum(int(row['endpoint_reachability'].split('/')[1]) for row in completed)}",
        "failures": failures,
        "scope_note": (
            "Every adjacent frozen interval is recursively certified using exact FCL mesh "
            "clearance and a conservative rigid-motion bound. Intervals that remain ambiguous "
            "at the frozen maximum depth are reported as uncertified, never as collision-free."
        ),
    }
    dump_json(out / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
