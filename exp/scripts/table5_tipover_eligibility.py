#!/usr/bin/env python3
"""Build a frozen, dataset-independent free-standing tip-over cohort."""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Mapping, Sequence
from xml.etree import ElementTree


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import numpy as np
from scipy.spatial import ConvexHull
import trimesh

import table5_v2_runtime as _core  # noqa: E402


SCHEMA_VERSION = "table5_tipover_eligibility_v2"
PROTOCOL_ID = "table5-free-standing-support-eligibility-v2"
MIN_SUPPORT_AREA_RATIO = 0.03
MIN_SUPPORT_EXTENT_RATIO = 0.03
BOTTOM_LAYER_RATIO = 0.01
MIN_BOTTOM_LAYER_M = 0.002

# These are semantic installation/handheld categories, fixed before looking at
# the stability outcomes. Floor lamps, towers, tripods, desks, carts, and
# appliances remain eligible unless their collision geometry fails the support
# test below.
EXCLUDED_CATEGORY_PATTERNS = (
    r"(^|_)wall(_|$)",
    r"(^|_)ceiling(_|$)",
    r"window",
    r"faucet",
    r"(^|_)vent(_|$)",
    r"shutter",
    r"mailbox",
    r"gutter",
    r"pipeline",
    r"playground_swing",
    r"(^|_)swing(_|$)",
    r"bicycle_(crankset|fork|dropper|handlebar|seatpost)",
    r"gyroscope",
    r"immersion_blender",
    r"hair_dryer",
    r"camera_flash",
    r"single_revolute_hinge",
    r"service_access_panel",
    r"access_panel",
    r"light_fixture",
)
EXCLUDED_CATEGORY_RE = tuple(re.compile(pattern) for pattern in EXCLUDED_CATEGORY_PATTERNS)


class EligibilityError(RuntimeError):
    pass


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise EligibilityError(f"JSON object expected: {path}")
    return value


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ],
        dtype=float,
    )


def _xyz(value: str | None, default: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    if not value:
        return np.asarray(default, dtype=float)
    parts = value.split()
    if len(parts) != 3:
        raise EligibilityError(f"invalid xyz/rpy value: {value!r}")
    return np.asarray([float(part) for part in parts], dtype=float)


def _origin(element: ElementTree.Element | None) -> tuple[np.ndarray, np.ndarray]:
    if element is None:
        return np.eye(3), np.zeros(3)
    return _rpy_matrix(_xyz(element.get("rpy"))), _xyz(element.get("xyz"))


def _link_transforms(row: Mapping[str, Any]) -> dict[str, np.ndarray]:
    tree = row.get("joint_tree")
    if not isinstance(tree, Mapping):
        raise EligibilityError("joint_tree is missing")
    roots = tree.get("root_links")
    joints = tree.get("joints")
    if not isinstance(roots, list) or len(roots) != 1 or not isinstance(joints, list):
        raise EligibilityError("joint_tree roots/joints are malformed")
    transforms: dict[str, np.ndarray] = {str(roots[0]): np.eye(4)}
    pending = [joint for joint in joints if isinstance(joint, Mapping)]
    while pending:
        progressed = False
        for joint in pending[:]:
            parent, child = joint.get("parent"), joint.get("child")
            if not isinstance(parent, str) or not isinstance(child, str) or parent not in transforms:
                continue
            origin_xyz = joint.get("origin_xyz", [0.0, 0.0, 0.0])
            origin_rpy = joint.get("origin_rpy", [0.0, 0.0, 0.0])
            if not (
                isinstance(origin_xyz, list) and len(origin_xyz) == 3
                and isinstance(origin_rpy, list) and len(origin_rpy) == 3
            ):
                raise EligibilityError(f"joint transform is malformed: {joint}")
            local = np.eye(4)
            local[:3, :3] = _rpy_matrix(origin_rpy)
            local[:3, 3] = np.asarray(origin_xyz, dtype=float)
            transforms[child] = transforms[parent] @ local
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise EligibilityError("joint_tree is disconnected")
    return transforms


def _primitive_points(element: ElementTree.Element) -> np.ndarray | None:
    geometry = element.find("geometry")
    if geometry is None:
        return None
    box = geometry.find("box")
    if box is not None:
        size = _xyz(box.get("size"))
        signs = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
        return signs * size
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.get("radius"))
        length = float(cylinder.get("length"))
        angles = np.linspace(0.0, 2.0 * math.pi, 32, endpoint=False)
        points = [[radius * math.cos(angle), radius * math.sin(angle), z] for angle in angles for z in (-length / 2, length / 2)]
        return np.asarray(points, dtype=float)
    sphere = geometry.find("sphere")
    if sphere is not None:
        radius = float(sphere.get("radius"))
        points = []
        for phi in np.linspace(0.0, math.pi, 12):
            for theta in np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False):
                points.append([radius * math.sin(phi) * math.cos(theta), radius * math.sin(phi) * math.sin(theta), radius * math.cos(phi)])
        return np.asarray(points, dtype=float)
    return None


def _mesh_points(element: ElementTree.Element, source_path: Path) -> np.ndarray | None:
    geometry = element.find("geometry")
    mesh = geometry.find("mesh") if geometry is not None else None
    if mesh is None or not mesh.get("filename"):
        return None
    filename = str(mesh.get("filename"))
    if filename.startswith("package://"):
        filename = filename[len("package://") :]
    path = Path(filename)
    if not path.is_absolute():
        path = source_path.parent / path
    if not path.is_file():
        return None
    loaded = trimesh.load(path, force="scene", process=False)
    if not isinstance(loaded, trimesh.Scene):
        loaded = trimesh.Scene(loaded)
    vertices: list[np.ndarray] = []
    scale = _xyz(mesh.get("scale"), (1.0, 1.0, 1.0))
    for name, geometry_data in loaded.geometry.items():
        vertices.append(np.asarray(geometry_data.vertices, dtype=float) * scale)
    return np.vstack(vertices) if vertices else None


def collision_points(row: Mapping[str, Any], source_path: Path) -> np.ndarray:
    transforms = _link_transforms(row)
    try:
        root = ElementTree.parse(source_path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise EligibilityError(f"cannot parse URDF: {error}") from error
    geometry_role = "collision" if root.findall(".//collision") else "visual"
    points: list[np.ndarray] = []
    for link in root.findall("link"):
        name = link.get("name")
        if not isinstance(name, str) or name not in transforms:
            continue
        link_transform = transforms[name]
        for geometry_element in link.findall(geometry_role):
            local_points = _primitive_points(geometry_element)
            if local_points is None:
                local_points = _mesh_points(geometry_element, source_path)
            if local_points is None:
                continue
            rotation, translation = _origin(geometry_element.find("origin"))
            local_points = local_points @ rotation.T + translation
            homogeneous = np.column_stack((local_points, np.ones(len(local_points))))
            points.append((homogeneous @ link_transform.T)[:, :3])
    if not points:
        raise EligibilityError("no collision geometry points available")
    result = np.vstack(points)
    if not np.isfinite(result).all():
        raise EligibilityError("collision geometry contains non-finite points")
    return result


def support_geometry(row: Mapping[str, Any], source_path: Path) -> dict[str, Any]:
    points = collision_points(row, source_path)
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    extent = maximum - minimum
    height = float(extent[2])
    if height <= 0.0:
        raise EligibilityError("collision geometry height is non-positive")
    bottom_layer = points[points[:, 2] <= minimum[2] + max(MIN_BOTTOM_LAYER_M, BOTTOM_LAYER_RATIO * height)]
    projected = bottom_layer[:, :2]
    if len(projected) >= 3:
        try:
            area = float(ConvexHull(projected).volume)
        except Exception:
            area = 0.0
    else:
        area = 0.0
    footprint_area = max(float(extent[0] * extent[1]), 1.0e-12)
    support_area_ratio = area / footprint_area
    support_extent = projected.max(axis=0) - projected.min(axis=0) if len(projected) else np.zeros(2)
    support_extent_ratio = float(min(support_extent) / max(max(extent[0], extent[1]), 1.0e-12))
    return {
        "collision_point_count": int(len(points)),
        "minimum_xyz_m": [float(value) for value in minimum],
        "maximum_xyz_m": [float(value) for value in maximum],
        "height_m": height,
        "bottom_layer_thickness_m": max(MIN_BOTTOM_LAYER_M, BOTTOM_LAYER_RATIO * height),
        "bottom_layer_point_count": int(len(bottom_layer)),
        "bottom_support_area_m2": area,
        "support_area_ratio": support_area_ratio,
        "support_extent_ratio": support_extent_ratio,
        "support_geometry_pass": bool(
            support_area_ratio >= MIN_SUPPORT_AREA_RATIO
            and support_extent_ratio >= MIN_SUPPORT_EXTENT_RATIO
        ),
    }


def semantic_reason(category: str) -> str | None:
    normalized = category.lower()
    for pattern in EXCLUDED_CATEGORY_RE:
        if pattern.search(normalized):
            return f"excluded_installation_or_handheld_semantics:{pattern.pattern}"
    return None


def evaluate_row(row: Mapping[str, Any]) -> dict[str, Any]:
    category = str(row.get("category", ""))
    semantic = semantic_reason(category)
    source = row.get("simulator_sources", {}).get("genesis") if isinstance(row.get("simulator_sources"), Mapping) else None
    source_path = Path(str(source.get("path"))) if isinstance(source, Mapping) else Path(str(row.get("urdf_path")))
    result: dict[str, Any] = {
        "dataset_id": row.get("dataset_id"),
        "category": category,
        "source_path": str(source_path),
        "source_sha256": source.get("sha256") if isinstance(source, Mapping) else row.get("urdf_sha256"),
        "eligible": False,
        "reasons": [],
        "support_geometry": None,
    }
    if semantic is not None:
        result["reasons"].append(semantic)
    bbox = row.get("bounding_box")
    if not isinstance(bbox, Mapping) or bbox.get("status") != "available":
        result["reasons"].append("canonical_collision_bbox_unavailable")
    if not source_path.is_file():
        result["reasons"].append("genesis_source_missing")
    if not result["reasons"]:
        try:
            geometry = support_geometry(row, source_path)
            result["support_geometry"] = geometry
            if not geometry["support_geometry_pass"]:
                result["reasons"].append("insufficient_canonical_bottom_support_geometry")
        except EligibilityError as error:
            result["reasons"].append(f"support_geometry_error:{error}")
    result["eligible"] = not result["reasons"]
    return result


def _manifest_hash(value: Mapping[str, Any]) -> str:
    return _core._runtime.canonical_sha256(value, exclude_fields=("manifest_sha256",))


def make_filtered_manifest(manifest: Mapping[str, Any], eligible_ids: set[str], output_path: Path) -> dict[str, Any]:
    filtered = deepcopy(dict(manifest))
    groups = []
    total_rows = 0
    for group in filtered.get("datasets", []):
        if not isinstance(group, Mapping):
            continue
        copied = deepcopy(dict(group))
        copied["rows"] = [row for row in copied.get("rows", []) if isinstance(row, Mapping) and row.get("dataset_id") in eligible_ids]
        total_rows += len(copied["rows"])
        groups.append(copied)
    filtered["datasets"] = groups
    filtered["total_rows"] = total_rows
    filtered["sample_size"] = total_rows
    filtered["eligibility_protocol_id"] = PROTOCOL_ID
    filtered["eligibility_source_manifest_sha256"] = manifest.get("manifest_sha256")
    filtered["manifest_sha256"] = _manifest_hash(filtered)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    _core._runtime.atomic_write_json(output_path, filtered)
    return filtered


def build(manifest_path: Path, output_dir: Path, slug: str) -> dict[str, Any]:
    manifest = _read_json(manifest_path)
    groups = [group for group in manifest.get("datasets", []) if isinstance(group, Mapping) and group.get("dataset_slug") == slug]
    if len(groups) != 1:
        raise EligibilityError(f"dataset {slug!r} is not unique")
    rows = [row for row in groups[0].get("rows", []) if isinstance(row, Mapping)]
    decisions = [evaluate_row(row) for row in rows]
    eligible_ids = {str(item["dataset_id"]) for item in decisions if item["eligible"]}
    output_dir.mkdir(parents=True, exist_ok=True)
    _core._runtime.atomic_write_json(output_dir / "eligibility.json", {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "protocol": {
            "semantic_exclusion_patterns": list(EXCLUDED_CATEGORY_PATTERNS),
            "geometry_selection": "all collision geometries when present; otherwise all visual geometries",
            "minimum_support_area_ratio": MIN_SUPPORT_AREA_RATIO,
            "minimum_support_extent_ratio": MIN_SUPPORT_EXTENT_RATIO,
            "bottom_layer_ratio": BOTTOM_LAYER_RATIO,
            "minimum_bottom_layer_m": MIN_BOTTOM_LAYER_M,
        },
        "source_manifest": str(manifest_path.resolve()),
        "source_manifest_sha256": manifest.get("manifest_sha256"),
        "dataset_slug": slug,
        "n": len(rows),
        "eligible_count": len(eligible_ids),
        "ineligible_count": len(rows) - len(eligible_ids),
        "decisions": decisions,
    })
    filtered_manifest = make_filtered_manifest(manifest, eligible_ids, output_dir / "manifest.json")
    return {
        "dataset_slug": slug,
        "n": len(rows),
        "eligible_count": len(eligible_ids),
        "ineligible_count": len(rows) - len(eligible_ids),
        "filtered_manifest": str((output_dir / "manifest.json").resolve()),
        "filtered_manifest_sha256": filtered_manifest["manifest_sha256"],
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--out", type=Path, required=True)
    arguments = parser.parse_args(argv)
    print(json.dumps(build(arguments.manifest, arguments.out, arguments.dataset), sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (EligibilityError, OSError, ValueError) as error:
        print(f"table5_tipover_eligibility: {error}", file=sys.stderr)
        raise SystemExit(2)
