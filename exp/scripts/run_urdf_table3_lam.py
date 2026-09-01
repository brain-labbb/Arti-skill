#!/usr/bin/env python3
"""Run the frozen Table 3 kinematic evaluation on LAM released outputs.

The formal cohort is sampled from all 3,217 manifest rows before any quality
labels or evaluation outcomes are inspected. Each asset runs in an isolated
child process; timeouts and malformed assets remain in both asset and declared
joint denominators.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import random
import signal
import subprocess
import sys
import tempfile
from typing import Any, Iterable
import uuid
import xml.etree.ElementTree as ET

import numpy as np


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
PROTOCOL_PATH = REPO_ROOT / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
DEFAULT_SOURCE_ROOT = REPO_ROOT / "exp/Articulated-Object-Code/released_outputs"
DEFAULT_RELEASE_MANIFEST = REPO_ROOT / "exp/Articulated-Object-Code/manifest.csv"
DEFAULT_DATASET_API = REPO_ROOT / "exp/Articulated-Object-Code/dataset_api.json"
DEFAULT_OUTPUT_PARENT = REPO_ROOT / "exp/runtime"

DATASET_NAME = "LAM released outputs (Articulated-Object-Code)"
FORMAL_RELEASE_COUNT = 3217
DEFAULT_N = 800
DEFAULT_SEED = 20260813
DEFAULT_SAMPLES = 21
DEFAULT_WORKERS = 4
DEFAULT_TIMEOUT_SECONDS = 120.0
CONTINUOUS_INTERVAL = (-math.pi, math.pi)
TRANSLATION_MOTION_THRESHOLD = 1e-6
ROTATION_MOTION_THRESHOLD_RAD = 1e-6
UNCHANGED_TRANSLATION_TOLERANCE = 1e-9
UNCHANGED_ROTATION_TOLERANCE_RAD = 1e-9
ROUNDTRIP_TRANSLATION_TOLERANCE = 1e-9
ROUNDTRIP_ROTATION_TOLERANCE_RAD = 1e-9
AXIS_EPSILON = 1e-12
SUPPORTED_MOVABLE_TYPES = {"revolute", "continuous", "prismatic"}
JOINT_RATE_METRICS = (
    "valid_range",
    "joint_sweep_success",
    "non_degenerate_motion",
    "subtree_consistency",
    "joint_level_pass",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    ).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def atomic_write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def atomic_write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(value)
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def append_jsonl(path: Path, value: dict[str, Any]) -> None:
    payload = canonical_json_bytes(value) + b"\n"
    with path.open("ab") as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"invalid JSONL at line {line_number}: {exc}") from exc
        if not isinstance(value, dict):
            raise RuntimeError(f"JSONL line {line_number} is not an object")
        rows.append(value)
    return rows


def _safe_relative_path(raw: str) -> Path:
    relative = Path(raw)
    if relative.is_absolute() or ".." in relative.parts or not relative.parts:
        raise ValueError(f"unsafe release relative path: {raw!r}")
    return relative


def load_release_rows(manifest_path: Path, source_root: Path) -> list[dict[str, Any]]:
    manifest_path = manifest_path.resolve(strict=True)
    source_root = source_root.resolve(strict=True)
    rows: list[dict[str, Any]] = []
    seen_paths: set[str] = set()
    with manifest_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {"object_release_id", "category", "tier", "rel_path", "n_movable"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"release manifest is missing columns: {sorted(required)}")
        for index, raw in enumerate(reader):
            relative = _safe_relative_path(raw["rel_path"])
            relative_text = relative.as_posix()
            if relative_text in seen_paths:
                raise ValueError(f"duplicate release rel_path: {relative_text}")
            seen_paths.add(relative_text)
            tier = raw["tier"].strip()
            if tier not in {"viable", "loads_only", "broken"}:
                raise ValueError(f"unexpected release tier: {tier!r}")
            try:
                declared_hint = max(0, int(raw["n_movable"] or 0))
            except ValueError as exc:
                raise ValueError(f"invalid n_movable for {relative_text}") from exc
            urdf_path = source_root / relative / "generated.urdf"
            try:
                urdf_path.resolve(strict=False).relative_to(source_root)
            except ValueError as exc:
                raise ValueError(f"release path escapes source root: {relative_text}") from exc
            asset_key = f"{tier}:{relative_text}"
            rows.append({
                "release_order": index,
                "asset_key": asset_key,
                "object_release_id": raw["object_release_id"],
                "category": raw["category"],
                "tier": tier,
                "rel_path": relative_text,
                "declared_joint_count_hint": declared_hint,
                "urdf_path": str(urdf_path),
                "urdf_exists": urdf_path.is_file() and not urdf_path.is_symlink(),
                "urdf_sha256": (
                    sha256_file(urdf_path)
                    if urdf_path.is_file() and not urdf_path.is_symlink()
                    else None
                ),
            })
    return rows


def select_assets(rows: Iterable[dict[str, Any]], n: int, seed: int) -> list[dict[str, Any]]:
    pool = sorted((dict(row) for row in rows), key=lambda row: row["asset_key"])
    if n <= 0 or n > len(pool):
        raise ValueError(f"cannot sample n={n} from release pool of {len(pool)}")
    return random.Random(seed).sample(pool, n)


def _parse_vector(raw: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if raw is None:
        return np.asarray(default, dtype=float)
    values = [float(value) for value in raw.split()]
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"expected finite 3-vector, got {raw!r}")
    return np.asarray(values, dtype=float)


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.asarray(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    rz = np.asarray(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=float)
    return rz @ ry @ rx


def _origin_transform(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    if element is None:
        return transform
    transform[:3, :3] = _rpy_matrix(_parse_vector(element.get("rpy"), (0.0, 0.0, 0.0)))
    transform[:3, 3] = _parse_vector(element.get("xyz"), (0.0, 0.0, 0.0))
    return transform


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if not math.isfinite(norm) or norm <= AXIS_EPSILON:
        raise ValueError("joint axis is zero or non-finite")
    x, y, z = axis / norm
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus = 1.0 - cosine
    return np.asarray((
        (cosine + x*x*one_minus, x*y*one_minus - z*sine, x*z*one_minus + y*sine),
        (y*x*one_minus + z*sine, cosine + y*y*one_minus, y*z*one_minus - x*sine),
        (z*x*one_minus - y*sine, z*y*one_minus + x*sine, cosine + z*z*one_minus),
    ), dtype=float)


def _motion_transform(joint: dict[str, Any], value: float) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    if joint["type"] in {"revolute", "continuous"}:
        transform[:3, :3] = _axis_angle_matrix(joint["axis"], value)
    elif joint["type"] == "prismatic":
        axis = joint["axis"]
        norm = float(np.linalg.norm(axis))
        if not math.isfinite(norm) or norm <= AXIS_EPSILON:
            raise ValueError("joint axis is zero or non-finite")
        transform[:3, 3] = axis / norm * value
    elif joint["type"] != "fixed":
        raise ValueError(f"unsupported joint type: {joint['type']}")
    return transform


def _parse_model(root: ET.Element) -> dict[str, Any]:
    if root.tag != "robot":
        raise ValueError("root XML element is not <robot>")
    link_elements = root.findall("link")
    links = [element.get("name", "") for element in link_elements]
    if not links or any(not name for name in links) or len(set(links)) != len(links):
        raise ValueError("link names are empty or duplicated")
    joints: list[dict[str, Any]] = []
    for index, element in enumerate(root.findall("joint")):
        parent = element.find("parent")
        child = element.find("child")
        axis_element = element.find("axis")
        limit = element.find("limit")
        joint = {
            "index": index,
            "name": element.get("name") or f"__unnamed_joint_{index}",
            "type": element.get("type", ""),
            "parent": None if parent is None else parent.get("link"),
            "child": None if child is None else child.get("link"),
            "origin": _origin_transform(element.find("origin")),
            "axis": _parse_vector(
                None if axis_element is None else axis_element.get("xyz"),
                (1.0, 0.0, 0.0),
            ),
            "lower": None,
            "upper": None,
        }
        if limit is not None:
            for field in ("lower", "upper"):
                raw = limit.get(field)
                if raw is not None:
                    value = float(raw)
                    joint[field] = value if math.isfinite(value) else None
        joints.append(joint)
    return {
        "links": links,
        "link_elements": {element.get("name", ""): element for element in link_elements},
        "joints": joints,
    }


def _validate_tree(model: dict[str, Any]) -> tuple[bool, str | None, list[str], dict[str, list[dict[str, Any]]]]:
    links = model["links"]
    link_set = set(links)
    children: set[str] = set()
    outgoing: dict[str, list[dict[str, Any]]] = defaultdict(list)
    seen_joint_names: set[str] = set()
    for joint in model["joints"]:
        if joint["name"] in seen_joint_names:
            return False, "duplicate joint name", [], outgoing
        seen_joint_names.add(joint["name"])
        parent, child = joint["parent"], joint["child"]
        if parent not in link_set or child not in link_set or parent == child:
            return False, "joint references invalid parent or child", [], outgoing
        if child in children:
            return False, "link has multiple parent joints", [], outgoing
        children.add(child)
        outgoing[parent].append(joint)
    roots = [link for link in links if link not in children]
    if len(roots) != 1:
        return False, f"expected one root link, found {len(roots)}", roots, outgoing
    order: list[str] = []
    queue = deque([roots[0]])
    visited: set[str] = set()
    while queue:
        link = queue.popleft()
        if link in visited:
            return False, "joint graph contains a cycle", roots, outgoing
        visited.add(link)
        order.append(link)
        queue.extend(joint["child"] for joint in outgoing.get(link, []))
    if visited != link_set:
        return False, "joint graph is disconnected or cyclic", roots, outgoing
    return True, None, order, outgoing


def _descendants(joint: dict[str, Any], outgoing: dict[str, list[dict[str, Any]]]) -> set[str]:
    result: set[str] = set()
    queue = deque([joint["child"]])
    while queue:
        link = queue.popleft()
        if link in result:
            continue
        result.add(link)
        queue.extend(child_joint["child"] for child_joint in outgoing.get(link, []))
    return result


def _forward_kinematics(
    model: dict[str, Any],
    root_link: str,
    outgoing: dict[str, list[dict[str, Any]]],
    joint_values: dict[str, float],
) -> dict[str, np.ndarray]:
    transforms = {root_link: np.eye(4, dtype=float)}
    queue = deque([root_link])
    while queue:
        parent = queue.popleft()
        for joint in outgoing.get(parent, []):
            value = float(joint_values.get(joint["name"], 0.0))
            child_transform = transforms[parent] @ joint["origin"] @ _motion_transform(joint, value)
            if not np.all(np.isfinite(child_transform)):
                raise ValueError("FK produced a non-finite link transform")
            transforms[joint["child"]] = child_transform
            queue.append(joint["child"])
    if len(transforms) != len(model["links"]):
        raise ValueError("FK did not produce every link transform")
    return transforms


def _rotation_distance(left: np.ndarray, right: np.ndarray) -> float:
    relative = left[:3, :3].T @ right[:3, :3]
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) / 2.0))
    return math.acos(cosine)


def _translation_distance(left: np.ndarray, right: np.ndarray) -> float:
    return float(np.linalg.norm(left[:3, 3] - right[:3, 3]))


def _bounds_corners(lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return np.asarray([
        (x, y, z, 1.0)
        for x in (float(lower[0]), float(upper[0]))
        for y in (float(lower[1]), float(upper[1]))
        for z in (float(lower[2]), float(upper[2]))
    ], dtype=float)


def _obj_bounds(path: Path) -> tuple[np.ndarray, np.ndarray]:
    lower = np.full(3, math.inf, dtype=float)
    upper = np.full(3, -math.inf, dtype=float)
    vertex_count = 0
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if not line.startswith("v "):
                continue
            fields = line.split()
            if len(fields) < 4:
                continue
            vertex = np.asarray([float(fields[1]), float(fields[2]), float(fields[3])], dtype=float)
            if not np.all(np.isfinite(vertex)):
                raise ValueError(f"OBJ contains a non-finite vertex: {path}")
            lower = np.minimum(lower, vertex)
            upper = np.maximum(upper, vertex)
            vertex_count += 1
    if vertex_count == 0:
        raise ValueError(f"OBJ has no readable vertices: {path}")
    return lower, upper


def _geometry_bounds(geometry: ET.Element, urdf_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    children = list(geometry)
    if len(children) != 1:
        raise ValueError("geometry must contain exactly one shape")
    shape = children[0]
    if shape.tag == "box":
        size = _parse_vector(shape.get("size"), (0.0, 0.0, 0.0))
        if np.any(size <= 0):
            raise ValueError("box size must be positive")
        return -size / 2.0, size / 2.0
    if shape.tag == "sphere":
        radius = float(shape.get("radius", "nan"))
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError("sphere radius must be positive")
        extent = np.asarray((radius, radius, radius), dtype=float)
        return -extent, extent
    if shape.tag == "cylinder":
        radius = float(shape.get("radius", "nan"))
        length = float(shape.get("length", "nan"))
        if not math.isfinite(radius) or not math.isfinite(length) or radius <= 0 or length <= 0:
            raise ValueError("cylinder dimensions must be positive")
        extent = np.asarray((radius, radius, length / 2.0), dtype=float)
        return -extent, extent
    if shape.tag == "mesh":
        filename = shape.get("filename")
        if not filename or filename.startswith("package://") or filename.startswith("file://"):
            raise ValueError(f"unsupported mesh filename: {filename!r}")
        relative = _safe_relative_path(filename)
        mesh_path = (urdf_dir / relative).resolve(strict=True)
        mesh_path.relative_to(urdf_dir.resolve(strict=True))
        if mesh_path.suffix.lower() != ".obj":
            raise ValueError(f"unsupported mesh format for scale derivation: {mesh_path.suffix}")
        lower, upper = _obj_bounds(mesh_path)
        scale = _parse_vector(shape.get("scale"), (1.0, 1.0, 1.0))
        scaled_a = lower * scale
        scaled_b = upper * scale
        return np.minimum(scaled_a, scaled_b), np.maximum(scaled_a, scaled_b)
    raise ValueError(f"unsupported geometry type: {shape.tag}")


def _kinematic_scale(
    model: dict[str, Any], transforms: dict[str, np.ndarray], urdf_path: Path
) -> tuple[float | None, str, str | None]:
    world_points: list[np.ndarray] = []
    geometry_count = 0
    try:
        for link_name, link_element in model["link_elements"].items():
            for element in (*link_element.findall("visual"), *link_element.findall("collision")):
                geometry = element.find("geometry")
                if geometry is None:
                    raise ValueError(f"geometry element is missing for link {link_name}")
                lower, upper = _geometry_bounds(geometry, urdf_path.parent)
                local = _origin_transform(element.find("origin"))
                world = transforms[link_name] @ local
                corners = _bounds_corners(lower, upper)
                world_points.extend((world @ corner)[:3] for corner in corners)
                geometry_count += 1
    except Exception as exc:  # noqa: BLE001
        return None, "NOT_EVALUABLE", f"{type(exc).__name__}: {exc}"
    if geometry_count == 0:
        return None, "NOT_EVALUABLE", "URDF has no visual or collision geometry"
    points = np.asarray(world_points, dtype=float)
    diagonal = float(np.linalg.norm(np.max(points, axis=0) - np.min(points, axis=0)))
    if not math.isfinite(diagonal) or diagonal <= 1e-12:
        return None, "NOT_EVALUABLE", "q0 geometry union AABB is degenerate"
    return diagonal, "COMPLETE", None


def _joint_interval(joint: dict[str, Any]) -> tuple[bool, float | None, float | None, str]:
    joint_type = joint["type"]
    if joint_type == "continuous":
        return True, CONTINUOUS_INTERVAL[0], CONTINUOUS_INTERVAL[1], "frozen_continuous_interval"
    if joint_type not in {"revolute", "prismatic"}:
        return False, None, None, "unsupported_joint_type"
    lower, upper = joint["lower"], joint["upper"]
    if lower is None or upper is None or not lower < upper:
        return False, lower, upper, "missing_nonfinite_or_empty_declared_interval"
    return True, float(lower), float(upper), "declared_limit"


def _failed_joint(name: str, joint_type: str, reason: str) -> dict[str, Any]:
    return {
        "joint_name": name,
        "joint_type": joint_type,
        "range_source": None,
        "range_lower": None,
        "range_upper": None,
        "sample_count_expected": DEFAULT_SAMPLES,
        "sample_count_executed": 0,
        "sample_values": [],
        "valid_range_pass": False,
        "joint_sweep_success": False,
        "non_degenerate_motion_pass": False,
        "subtree_consistency_pass": False,
        "fk_roundtrip_pass": False,
        "joint_level_pass": False,
        "max_descendant_normalized_translation": None,
        "max_descendant_rotation_rad": None,
        "max_non_descendant_translation": None,
        "max_non_descendant_rotation_rad": None,
        "fk_roundtrip_max_translation": None,
        "fk_roundtrip_max_rotation_rad": None,
        "issue": reason,
    }


def _evaluate_joint(
    model: dict[str, Any],
    joint: dict[str, Any],
    root_link: str,
    outgoing: dict[str, list[dict[str, Any]]],
    q0: dict[str, float],
    baseline: dict[str, np.ndarray],
    scale: float | None,
    samples: int,
) -> dict[str, Any]:
    valid_range, lower, upper, range_source = _joint_interval(joint)
    result = _failed_joint(joint["name"], joint["type"], "invalid or unsupported range")
    result.update({
        "range_source": range_source,
        "range_lower": lower,
        "range_upper": upper,
        "sample_count_expected": samples,
        "valid_range_pass": valid_range,
    })
    if not valid_range or lower is None or upper is None:
        return result
    result["issue"] = None
    states = np.linspace(lower, upper, samples, dtype=float).tolist()
    result["sample_values"] = states
    descendants = _descendants(joint, outgoing)
    non_descendants = set(model["links"]) - descendants
    transforms_by_state: list[dict[str, np.ndarray]] = []
    try:
        for state in states:
            values = dict(q0)
            values[joint["name"]] = float(state)
            transforms_by_state.append(_forward_kinematics(model, root_link, outgoing, values))
    except Exception as exc:  # noqa: BLE001
        result["issue"] = f"FK sweep failed: {type(exc).__name__}: {exc}"
        return result
    result["sample_count_executed"] = len(transforms_by_state)
    result["joint_sweep_success"] = len(transforms_by_state) == samples

    endpoint_start = transforms_by_state[0]
    endpoint_end = transforms_by_state[-1]
    comparison_pairs = [(endpoint_start, endpoint_end)]
    if joint["type"] == "continuous":
        # -pi and +pi are the same orientation, so endpoint-only comparison
        # would classify every healthy continuous joint as degenerate.
        comparison_pairs = [(baseline, transforms) for transforms in transforms_by_state]
    descendant_translation = max(
        (
            _translation_distance(left[name], right[name])
            for left, right in comparison_pairs
            for name in descendants
        ),
        default=0.0,
    )
    descendant_rotation = max(
        (
            _rotation_distance(left[name], right[name])
            for left, right in comparison_pairs
            for name in descendants
        ),
        default=0.0,
    )
    normalized_descendant_translation = (
        descendant_translation / scale if scale is not None else None
    )
    result["max_descendant_normalized_translation"] = normalized_descendant_translation
    result["max_descendant_rotation_rad"] = descendant_rotation
    result["non_degenerate_motion_pass"] = bool(
        (normalized_descendant_translation is not None
         and normalized_descendant_translation > TRANSLATION_MOTION_THRESHOLD)
        or descendant_rotation > ROTATION_MOTION_THRESHOLD_RAD
    )

    max_non_descendant_translation = 0.0
    max_non_descendant_rotation = 0.0
    for transforms in transforms_by_state:
        for name in non_descendants:
            translation = _translation_distance(baseline[name], transforms[name])
            normalized = translation / scale if scale is not None else translation
            max_non_descendant_translation = max(max_non_descendant_translation, normalized)
            max_non_descendant_rotation = max(
                max_non_descendant_rotation,
                _rotation_distance(baseline[name], transforms[name]),
            )
    result["max_non_descendant_translation"] = max_non_descendant_translation
    result["max_non_descendant_rotation_rad"] = max_non_descendant_rotation
    result["subtree_consistency_pass"] = bool(
        max_non_descendant_translation <= UNCHANGED_TRANSLATION_TOLERANCE
        and max_non_descendant_rotation <= UNCHANGED_ROTATION_TOLERANCE_RAD
    )

    q1 = dict(q0)
    q1[joint["name"]] = float(upper)
    try:
        _forward_kinematics(model, root_link, outgoing, q1)
        returned = _forward_kinematics(model, root_link, outgoing, q0)
        roundtrip_translation = 0.0
        roundtrip_rotation = 0.0
        for name in model["links"]:
            translation = _translation_distance(baseline[name], returned[name])
            normalized = translation / scale if scale is not None else translation
            roundtrip_translation = max(roundtrip_translation, normalized)
            roundtrip_rotation = max(
                roundtrip_rotation, _rotation_distance(baseline[name], returned[name])
            )
        result["fk_roundtrip_max_translation"] = roundtrip_translation
        result["fk_roundtrip_max_rotation_rad"] = roundtrip_rotation
        result["fk_roundtrip_pass"] = bool(
            roundtrip_translation <= ROUNDTRIP_TRANSLATION_TOLERANCE
            and roundtrip_rotation <= ROUNDTRIP_ROTATION_TOLERANCE_RAD
        )
    except Exception as exc:  # noqa: BLE001
        result["issue"] = f"FK round-trip failed: {type(exc).__name__}: {exc}"

    result["joint_level_pass"] = all((
        result["valid_range_pass"],
        result["joint_sweep_success"],
        result["non_degenerate_motion_pass"],
        result["subtree_consistency_pass"],
        result["fk_roundtrip_pass"],
    ))
    if result["joint_level_pass"]:
        result["issue"] = None
    elif result["issue"] is None:
        failed_checks = [
            name for name, field in (
                ("valid_range", "valid_range_pass"),
                ("joint_sweep", "joint_sweep_success"),
                ("non_degenerate_motion", "non_degenerate_motion_pass"),
                ("subtree_consistency", "subtree_consistency_pass"),
                ("fk_roundtrip", "fk_roundtrip_pass"),
            )
            if not result[field]
        ]
        result["issue"] = f"failed checks: {', '.join(failed_checks)}"
    return result


def failed_record(
    asset_key: str,
    declared_joint_count: int,
    reason: str,
    status: str = "error",
) -> dict[str, Any]:
    count = max(0, int(declared_joint_count))
    return {
        "asset_key": asset_key,
        "status": status,
        "error": reason,
        "parse_success": False,
        "tree_valid": False,
        "tree_issue": reason,
        "declared_joint_count": count,
        "kinematic_scale": None,
        "joints": [
            _failed_joint(f"__unparsed_declared_joint_{index}", "unknown", reason)
            for index in range(count)
        ],
        "strict_kinematic_pass": False,
    }


def _failed_parsed_record(
    asset_key: str,
    movable: list[dict[str, Any]],
    reason: str,
    samples: int,
) -> dict[str, Any]:
    joints = []
    for joint in movable:
        valid, lower, upper, source = _joint_interval(joint)
        failed = _failed_joint(joint["name"], joint["type"], reason)
        failed.update({
            "range_source": source,
            "range_lower": lower,
            "range_upper": upper,
            "valid_range_pass": valid,
            "sample_count_expected": samples,
        })
        joints.append(failed)
    return {
        "asset_key": asset_key,
        "status": "error",
        "error": reason,
        "parse_success": True,
        "tree_valid": True,
        "tree_issue": None,
        "declared_joint_count": len(movable),
        "kinematic_scale": None,
        "kinematic_scale_status": "UNAVAILABLE_INITIAL_FK",
        "kinematic_scale_issue": reason,
        "joints": joints,
        "strict_kinematic_pass": False,
    }


def evaluate_urdf(
    urdf_path: Path,
    asset_key: str,
    samples: int = DEFAULT_SAMPLES,
    declared_joint_count_hint: int | None = None,
) -> dict[str, Any]:
    try:
        root = ET.parse(urdf_path).getroot()
    except Exception as exc:  # noqa: BLE001
        return failed_record(
            asset_key,
            0 if declared_joint_count_hint is None else declared_joint_count_hint,
            f"XML parse failed: {type(exc).__name__}: {exc}",
        )
    movable_elements = [
        element for element in root.findall("joint") if element.get("type", "") != "fixed"
    ]
    declared_count = len(movable_elements)
    try:
        model = _parse_model(root)
    except Exception as exc:  # noqa: BLE001
        return failed_record(
            asset_key, declared_count, f"URDF model parse failed: {type(exc).__name__}: {exc}"
        )
    movable = [joint for joint in model["joints"] if joint["type"] != "fixed"]
    tree_valid, tree_issue, order, outgoing = _validate_tree(model)
    if not tree_valid:
        joints = []
        for joint in movable:
            valid, lower, upper, source = _joint_interval(joint)
            failed = _failed_joint(joint["name"], joint["type"], tree_issue or "invalid tree")
            failed.update({
                "range_source": source,
                "range_lower": lower,
                "range_upper": upper,
                "valid_range_pass": valid,
                "sample_count_expected": samples,
            })
            joints.append(failed)
        return {
            "asset_key": asset_key,
            "status": "completed",
            "error": None,
            "parse_success": True,
            "tree_valid": False,
            "tree_issue": tree_issue,
            "declared_joint_count": declared_count,
            "kinematic_scale": None,
            "joints": joints,
            "strict_kinematic_pass": False,
        }

    root_link = order[0]
    q0: dict[str, float] = {}
    for joint in movable:
        valid, lower, upper, _source = _joint_interval(joint)
        if valid and lower is not None and upper is not None:
            q0[joint["name"]] = min(max(0.0, lower), upper)
        else:
            q0[joint["name"]] = 0.0
    try:
        baseline = _forward_kinematics(model, root_link, outgoing, q0)
    except Exception as exc:  # noqa: BLE001
        return _failed_parsed_record(
            asset_key,
            movable,
            f"initial FK failed: {type(exc).__name__}: {exc}",
            samples,
        )
    scale, scale_status, scale_issue = _kinematic_scale(model, baseline, urdf_path)
    joints = [
        _evaluate_joint(model, joint, root_link, outgoing, q0, baseline, scale, samples)
        for joint in movable
    ]
    strict_pass = bool(joints) and all(joint["joint_level_pass"] for joint in joints)
    return {
        "asset_key": asset_key,
        "status": "completed",
        "error": None,
        "parse_success": True,
        "tree_valid": True,
        "tree_issue": None,
        "declared_joint_count": declared_count,
        "kinematic_scale": scale,
        "kinematic_scale_status": scale_status,
        "kinematic_scale_issue": scale_issue,
        "joints": joints,
        "strict_kinematic_pass": strict_pass,
    }


def aggregate_records(records: list[dict[str, Any]], expected_n: int) -> dict[str, Any]:
    if len(records) != expected_n:
        raise ValueError(f"expected {expected_n} records, got {len(records)}")
    all_joints: list[dict[str, Any]] = []
    declared_total = 0
    for record in records:
        count = int(record.get("declared_joint_count", 0))
        joints = list(record.get("joints", []))
        if len(joints) != count:
            raise ValueError(f"joint denominator mismatch for {record.get('asset_key')}")
        declared_total += count
        all_joints.extend(joints)
    metrics: dict[str, Any] = {}
    field_by_metric = {
        "valid_range": "valid_range_pass",
        "joint_sweep_success": "joint_sweep_success",
        "non_degenerate_motion": "non_degenerate_motion_pass",
        "subtree_consistency": "subtree_consistency_pass",
        "joint_level_pass": "joint_level_pass",
    }
    for metric, field in field_by_metric.items():
        passed = sum(bool(joint.get(field, False)) for joint in all_joints)
        metrics[metric] = {
            "passed": passed,
            "denominator": declared_total,
            "rate": passed / declared_total if declared_total else None,
        }
    measured_roundtrip = [
        joint for joint in all_joints
        if joint.get("fk_roundtrip_max_translation") is not None
        and joint.get("fk_roundtrip_max_rotation_rad") is not None
    ]
    metrics["fk_roundtrip_error"] = {
        "max_normalized_translation": max(
            (float(joint["fk_roundtrip_max_translation"]) for joint in measured_roundtrip),
            default=None,
        ),
        "max_rotation_rad": max(
            (float(joint["fk_roundtrip_max_rotation_rad"]) for joint in measured_roundtrip),
            default=None,
        ),
        "passed_threshold": sum(bool(joint.get("fk_roundtrip_pass")) for joint in all_joints),
        "measured_joint_count": len(measured_roundtrip),
        "denominator": declared_total,
        "status": "COMPLETE" if len(measured_roundtrip) == declared_total else "PARTIAL",
    }
    strict_passed = sum(bool(record.get("strict_kinematic_pass", False)) for record in records)
    metrics["strict_kinematic_pass"] = {
        "passed": strict_passed,
        "denominator": expected_n,
        "rate": strict_passed / expected_n if expected_n else None,
    }
    categories = [record.get("category") for record in records]
    if all(isinstance(category, str) and category for category in categories):
        grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for record in records:
            grouped[str(record["category"])].append(record)
        macro_metrics: dict[str, Any] = {}
        for metric, field in field_by_metric.items():
            category_rates = []
            for group in grouped.values():
                joints = [joint for record in group for joint in record["joints"]]
                if joints:
                    category_rates.append(
                        sum(bool(joint.get(field, False)) for joint in joints) / len(joints)
                    )
            macro_metrics[metric] = {
                "rate": sum(category_rates) / len(category_rates) if category_rates else None,
                "category_count": len(category_rates),
            }
        strict_category_rates = [
            sum(bool(record.get("strict_kinematic_pass", False)) for record in group) / len(group)
            for group in grouped.values()
        ]
        macro_metrics["strict_kinematic_pass"] = {
            "rate": sum(strict_category_rates) / len(strict_category_rates),
            "category_count": len(strict_category_rates),
        }
        category_macro = {
            "state": "evaluated",
            "category_count": len(grouped),
            "joint_metric_category_count": macro_metrics["joint_level_pass"]["category_count"],
            "metrics": macro_metrics,
        }
    else:
        category_macro = {
            "state": "not_evaluable",
            "reason": "one or more records do not carry a category binding",
            "category_count": 0,
            "joint_metric_category_count": 0,
            "metrics": {},
        }
    return {
        "n_eval": expected_n,
        "j_eval": declared_total,
        "metrics": metrics,
        "status_counts": dict(sorted(Counter(str(record.get("status")) for record in records).items())),
        "parse_success": sum(bool(record.get("parse_success")) for record in records),
        "valid_tree": sum(bool(record.get("tree_valid")) for record in records),
        "category_macro": category_macro,
    }


def _environment() -> dict[str, Any]:
    return {
        "python": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "numpy": np.__version__,
        "thread_environment": {
            name: os.environ.get(name)
            for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")
        },
    }


def _manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def build_manifest(args: argparse.Namespace, rows: list[dict[str, Any]]) -> dict[str, Any]:
    selected = select_assets(rows, args.n, args.seed)
    frozen_records = []
    for rank, row in enumerate(selected, 1):
        frozen_records.append({
            **row,
            "selection_rank": rank,
            "selection_hash": hashlib.sha256(
                f"lam-table3-v1\0{args.seed}\0{row['asset_key']}".encode("utf-8")
            ).hexdigest(),
        })
    config = {
        "protocol_id": "urdf-sim-ready-table3-lam-n800-v1",
        "samples_per_joint": args.samples,
        "continuous_interval": list(CONTINUOUS_INTERVAL),
        "continuous_non_degenerate_policy": "maximum_excursion_from_q0_over_frozen_interval",
        "q0_policy": "zero_clipped_to_declared_interval",
        "translation_motion_threshold_bbox_diagonal": TRANSLATION_MOTION_THRESHOLD,
        "rotation_motion_threshold_rad": ROTATION_MOTION_THRESHOLD_RAD,
        "unchanged_translation_tolerance_bbox_diagonal": UNCHANGED_TRANSLATION_TOLERANCE,
        "unchanged_rotation_tolerance_rad": UNCHANGED_ROTATION_TOLERANCE_RAD,
        "roundtrip_translation_tolerance_bbox_diagonal": ROUNDTRIP_TRANSLATION_TOLERANCE,
        "roundtrip_rotation_tolerance_rad": ROUNDTRIP_ROTATION_TOLERANCE_RAD,
        "strict_asset_requires_at_least_one_declared_movable_joint": True,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "workers": args.workers,
    }
    environment = _environment()
    manifest = {
        "schema_version": 1,
        "dataset": DATASET_NAME,
        "classification": "FORMAL" if args.mode == "formal" else "NON_FORMAL_SMOKE",
        "created_at": utc_now(),
        "source": {
            "source_root": str(args.source_root.resolve(strict=True)),
            "release_manifest": str(args.release_manifest.resolve(strict=True)),
            "release_manifest_sha256": sha256_file(args.release_manifest.resolve(strict=True)),
            "dataset_api": str(DEFAULT_DATASET_API.resolve(strict=True)),
            "dataset_api_sha256": sha256_file(DEFAULT_DATASET_API.resolve(strict=True)),
            "upstream_revision": json.loads(DEFAULT_DATASET_API.read_text(encoding="utf-8"))["sha"],
            "n_release": len(rows),
            "tier_counts": dict(sorted(Counter(row["tier"] for row in rows).items())),
            "candidate_pool_sha256": canonical_sha256([row["asset_key"] for row in sorted(rows, key=lambda x: x["asset_key"])]),
        },
        "selection": {
            "algorithm": "random.Random(seed).sample(sorted(asset_key), n)",
            "quality_label_blind": True,
            "seed": args.seed,
            "n_eval": args.n,
            "selected_asset_keys_sha256": canonical_sha256([row["asset_key"] for row in frozen_records]),
        },
        "evaluation": {
            "protocol_path": str(PROTOCOL_PATH.resolve(strict=True)),
            "protocol_sha256": sha256_file(PROTOCOL_PATH.resolve(strict=True)),
            "evaluator_path": str(SCRIPT_PATH),
            "evaluator_sha256": sha256_file(SCRIPT_PATH),
            "config": config,
            "config_sha256": canonical_sha256(config),
            "environment": environment,
            "environment_sha256": canonical_sha256(environment),
        },
        "records": frozen_records,
    }
    manifest["manifest_content_sha256"] = _manifest_self_hash(manifest)
    return manifest


def _bind_record(record: dict[str, Any], job: dict[str, Any]) -> dict[str, Any]:
    record.update({
        "object_release_id": job["object_release_id"],
        "category": job["category"],
        "tier": job["tier"],
        "rel_path": job["rel_path"],
        "selection_rank": job["selection_rank"],
        "selection_hash": job["selection_hash"],
        "urdf_sha256": job["urdf_sha256"],
        "manifest_content_sha256": job["manifest_content_sha256"],
        "completed_at": utc_now(),
    })
    return record


def run_internal_job(job_path: Path, result_path: Path) -> int:
    job = json.loads(job_path.resolve(strict=True).read_text(encoding="utf-8"))
    urdf_path = Path(job["urdf_path"])
    if not urdf_path.is_file() or urdf_path.is_symlink():
        record = failed_record(
            job["asset_key"], job["declared_joint_count_hint"], "selected URDF is missing"
        )
    elif sha256_file(urdf_path) != job["urdf_sha256"]:
        record = failed_record(
            job["asset_key"], job["declared_joint_count_hint"], "selected URDF changed after freeze"
        )
    else:
        record = evaluate_urdf(
            urdf_path,
            job["asset_key"],
            samples=job["samples"],
            declared_joint_count_hint=job["declared_joint_count_hint"],
        )
        if sha256_file(urdf_path) != job["urdf_sha256"]:
            record = failed_record(
                job["asset_key"], job["declared_joint_count_hint"], "selected URDF changed during evaluation"
            )
    atomic_write_json(result_path, _bind_record(record, job))
    return 0


def _execute_job(job: dict[str, Any], scratch_root: Path, timeout_seconds: float) -> dict[str, Any]:
    job_root = Path(tempfile.mkdtemp(prefix="job_", dir=scratch_root))
    job_path = job_root / "job.json"
    result_path = job_root / "result.json"
    atomic_write_json(job_path, job)
    environment = dict(os.environ)
    environment.update({
        "OMP_NUM_THREADS": "1",
        "OPENBLAS_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "NUMEXPR_NUM_THREADS": "1",
    })
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT_PATH), "--internal-job", str(job_path), "--internal-result", str(result_path)],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
        env=environment,
    )
    try:
        stdout, stderr = process.communicate(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate()
        return _bind_record(failed_record(
            job["asset_key"], job["declared_joint_count_hint"],
            f"asset timeout after {timeout_seconds:g} seconds", status="timeout"
        ), job)
    if process.returncode != 0 or not result_path.is_file():
        detail = stderr.decode("utf-8", errors="replace")[-4000:]
        return _bind_record(failed_record(
            job["asset_key"], job["declared_joint_count_hint"],
            f"worker failed with exit {process.returncode}: {detail}", status="error"
        ), job)
    if stdout:
        pass
    result = json.loads(result_path.read_text(encoding="utf-8"))
    if result.get("asset_key") != job["asset_key"]:
        return _bind_record(failed_record(
            job["asset_key"], job["declared_joint_count_hint"], "worker result asset binding mismatch"
        ), job)
    return result


def _summary_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    lines = [
        "# LAM Table 3 Kinematic Executability",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            f"Frozen full-release random cohort: N_eval={summary['n_eval']} from "
            f"N_release={manifest['source']['n_release']}, seed={manifest['selection']['seed']}; "
            f"J_eval={summary['j_eval']}. No quality-tier filtering was applied."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    for metric in JOINT_RATE_METRICS:
        value = metrics[metric]
        percentage = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        lines.append(f"| {metric} | {value['passed']} / {value['denominator']} ({percentage}) |")
    roundtrip = metrics["fk_roundtrip_error"]
    lines.append(
        "| fk_roundtrip_error | "
        f"translation={roundtrip['max_normalized_translation']}; "
        f"rotation_rad={roundtrip['max_rotation_rad']}; "
        f"coverage={roundtrip['measured_joint_count']} / {roundtrip['denominator']} ({roundtrip['status']}) |"
    )
    strict = metrics["strict_kinematic_pass"]
    strict_percentage = "N/A" if strict["rate"] is None else f"{100 * strict['rate']:.2f}%"
    lines.append(
        f"| strict_kinematic_pass | {strict['passed']} / {strict['denominator']} ({strict_percentage}) |"
    )
    lines.extend([
        "",
        (
            f"Category macro average over {summary['category_macro']['category_count']} observed "
            f"categories ({summary['category_macro']['joint_metric_category_count']} with at least "
            "one declared movable joint):"
        ),
        "",
        "| Metric | Category macro |",
        "|---|---:|",
    ])
    for metric in (*JOINT_RATE_METRICS, "strict_kinematic_pass"):
        value = summary["category_macro"]["metrics"][metric]
        percentage = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        lines.append(f"| {metric} | {percentage} (categories={value['category_count']}) |")
    lines.extend([
        "",
        "This evaluation checks executable declared kinematics only. It does not validate semantic joint correctness, collision-free motion, dynamics, or real-world fidelity.",
        "",
    ])
    return "\n".join(lines)


def _prepare_output(args: argparse.Namespace) -> Path:
    if args.output is None:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        args.output = DEFAULT_OUTPUT_PARENT / f"urdf_table3_lam_n{args.n}_seed{args.seed}_{timestamp}"
    output = args.output.resolve(strict=False)
    try:
        output.relative_to(REPO_ROOT.resolve(strict=True))
    except ValueError as exc:
        raise ValueError(f"output must be inside repository: {output}") from exc
    if args.resume:
        if not output.is_dir():
            raise FileNotFoundError(f"resume output does not exist: {output}")
    else:
        output.mkdir(parents=True, exist_ok=False)
    return output


def run(args: argparse.Namespace) -> Path:
    validate_contract(args)
    output = _prepare_output(args)
    manifest_path = output / "manifest.json"
    records_path = output / "asset_records.jsonl"
    checkpoint_path = output / "checkpoint.json"
    scratch = output / ".worker_scratch"
    scratch.mkdir(exist_ok=True)
    rows = load_release_rows(args.release_manifest, args.source_root)
    if args.mode == "formal" and len(rows) != FORMAL_RELEASE_COUNT:
        raise RuntimeError(f"formal release count mismatch: {len(rows)} != {FORMAL_RELEASE_COUNT}")
    if args.resume:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
            raise RuntimeError("resume manifest self-hash mismatch")
        fresh = build_manifest(args, rows)
        for field in ("source", "selection", "evaluation", "records"):
            if fresh[field] != manifest[field]:
                raise RuntimeError(f"resume binding mismatch: {field}")
    else:
        manifest = build_manifest(args, rows)
        atomic_write_json(manifest_path, manifest)
        records_path.touch(exist_ok=False)
        atomic_write_json(checkpoint_path, {
            "state": "frozen",
            "completed": 0,
            "remaining": args.n,
            "n_eval": args.n,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "updated_at": utc_now(),
        })
    records = load_jsonl(records_path)
    by_key: dict[str, dict[str, Any]] = {}
    selected_keys = [row["asset_key"] for row in manifest["records"]]
    for record in records:
        key = record.get("asset_key")
        if key not in selected_keys or key in by_key:
            raise RuntimeError(f"invalid or duplicate resume record: {key!r}")
        if record.get("manifest_content_sha256") != manifest["manifest_content_sha256"]:
            raise RuntimeError(f"resume record manifest binding mismatch: {key}")
        by_key[key] = record
    pending = [row for row in manifest["records"] if row["asset_key"] not in by_key]
    jobs = [{
        **row,
        "samples": args.samples,
        "manifest_content_sha256": manifest["manifest_content_sha256"],
    } for row in pending]
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(_execute_job, job, scratch, args.asset_timeout_seconds): job
            for job in jobs
        }
        for future in as_completed(futures):
            job = futures[future]
            try:
                record = future.result()
            except Exception as exc:  # noqa: BLE001
                record = _bind_record(failed_record(
                    job["asset_key"], job["declared_joint_count_hint"],
                    f"parent worker exception: {type(exc).__name__}: {exc}"
                ), job)
            by_key[record["asset_key"]] = record
            append_jsonl(records_path, record)
            completed = len(by_key)
            atomic_write_json(checkpoint_path, {
                "state": "running" if completed < args.n else "aggregating",
                "completed": completed,
                "remaining": args.n - completed,
                "n_eval": args.n,
                "last_completed_asset_key": record["asset_key"],
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "updated_at": utc_now(),
            })
            print(f"[{completed}/{args.n}] {record['asset_key']} {record['status']}", flush=True)
    ordered = [by_key[key] for key in selected_keys]
    summary = aggregate_records(ordered, args.n)
    summary.update({
        "schema_version": 1,
        "status": "completed",
        "classification": manifest["classification"],
        "dataset": DATASET_NAME,
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "completed_at": utc_now(),
    })
    atomic_write_json(output / "summary.json", summary)
    atomic_write_text(output / "summary.md", _summary_markdown(summary, manifest))
    atomic_write_json(checkpoint_path, {
        "state": "complete",
        "completed": args.n,
        "remaining": 0,
        "n_eval": args.n,
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "updated_at": utc_now(),
    })
    return output


def validate_contract(args: argparse.Namespace) -> None:
    if args.n <= 0 or args.samples < 2 or args.workers <= 0 or args.asset_timeout_seconds <= 0:
        raise ValueError("n, workers, timeout must be positive and samples must be >= 2")
    if args.mode == "formal":
        expected = {
            "n": DEFAULT_N,
            "seed": DEFAULT_SEED,
            "samples": DEFAULT_SAMPLES,
            "workers": DEFAULT_WORKERS,
            "asset_timeout_seconds": DEFAULT_TIMEOUT_SECONDS,
        }
        mismatches = [name for name, value in expected.items() if getattr(args, name) != value]
        if mismatches:
            raise ValueError(f"formal mode freezes: {', '.join(mismatches)}")
        if args.source_root.resolve(strict=False) != DEFAULT_SOURCE_ROOT.resolve(strict=False):
            raise ValueError("formal mode freezes the LAM source root")
        if args.release_manifest.resolve(strict=False) != DEFAULT_RELEASE_MANIFEST.resolve(strict=False):
            raise ValueError("formal mode freezes the LAM release manifest")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), default="formal")
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--release-manifest", type=Path, default=DEFAULT_RELEASE_MANIFEST)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--n", type=int, default=DEFAULT_N)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--asset-timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--internal-job", type=Path)
    parser.add_argument("--internal-result", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.internal_job is not None or args.internal_result is not None:
        if args.internal_job is None or args.internal_result is None:
            raise ValueError("internal job mode requires both paths")
        return run_internal_job(args.internal_job, args.internal_result)
    output = run(args)
    print(json.dumps({"status": "completed", "output": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
