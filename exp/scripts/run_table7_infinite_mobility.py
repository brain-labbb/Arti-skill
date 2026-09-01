#!/usr/bin/env python3
"""Audit Infinite Mobility for Nano3D Table 7 Production Readiness.

This is a static audit of the exact 20-factory x 36-seed cohort frozen by the
Table 6 run. It never resamples successful assets: generation timeouts stay in
the 720-case intent-to-run denominator, and generated assets with no movable
joint remain an explicit stratum.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import os
import re
import shlex
import shutil
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path, PureWindowsPath
from typing import Any
from urllib.parse import urlparse


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent
DEFAULT_PROTOCOL = REPO_ROOT / "exp/reference/table7_production_readiness_protocol_v1.json"
DEFAULT_TABLE6_ROOT = REPO_ROOT / "exp/runtime/table6_infinite_mobility"
DEFAULT_GENERATION_ROOT = REPO_ROOT / "exp/runtime/infinite_mobility_v1"
DEFAULT_BASELINE_ROOT = REPO_ROOT / ".cache/Infinite-Mobility"
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/table7_infinite_mobility"
MESH_SUFFIXES = {".obj", ".stl", ".ply", ".off", ".dae", ".glb", ".gltf"}
JOINT_TYPES = {"fixed", "revolute", "continuous", "prismatic", "planar", "floating"}
ONE_AXIS_JOINT_TYPES = {"revolute", "continuous", "prismatic"}
BOUNDED_JOINT_TYPES = {"revolute", "prismatic"}
TEXTURE_DIRECTIVES = {
    "map_ka",
    "map_kd",
    "map_ks",
    "map_ke",
    "map_ns",
    "map_d",
    "map_bump",
    "bump",
    "disp",
    "decal",
    "refl",
    "norm",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def contained(path: Path, *, must_exist: bool) -> Path:
    root = WORKSPACE_ROOT.resolve(strict=True)
    resolved = path.resolve(strict=must_exist)
    if resolved != root and root not in resolved.parents:
        raise RuntimeError(f"path escapes authorized workspace: {resolved}")
    return resolved


def assert_workspace() -> None:
    cwd = Path.cwd().resolve(strict=True)
    root = WORKSPACE_ROOT.resolve(strict=True)
    if cwd != root:
        raise RuntimeError(f"run from the frozen workspace root {root}; current cwd is {cwd}")
    contained(SCRIPT_PATH, must_exist=True)


def read_json(path: Path) -> Any:
    safe = contained(path, must_exist=True)
    return json.loads(safe.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    safe = contained(path, must_exist=False)
    safe.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256_file(path: Path) -> str:
    safe = contained(path, must_exist=True)
    digest = hashlib.sha256()
    with safe.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative_to_repo(path: Path) -> str:
    return contained(path, must_exist=True).relative_to(REPO_ROOT).as_posix()


def relative_to_workspace(path: Path) -> str:
    return contained(path, must_exist=True).relative_to(WORKSPACE_ROOT).as_posix()


def rate(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": int(numerator),
        "denominator": int(denominator),
        "value": numerator / denominator if denominator else None,
    }


def mean(values: list[float | int]) -> float | None:
    return float(sum(values) / len(values)) if values else None


def tree_files(root: Path) -> tuple[list[Path], list[str]]:
    safe_root = contained(root, must_exist=True)
    files: list[Path] = []
    symlinks: list[str] = []
    for dirpath, dirnames, filenames in os.walk(safe_root, followlinks=False):
        current = contained(Path(dirpath), must_exist=True)
        kept_dirs: list[str] = []
        for name in sorted(dirnames):
            candidate = current / name
            if candidate.is_symlink():
                symlinks.append(candidate.relative_to(safe_root).as_posix())
            else:
                contained(candidate, must_exist=True)
                kept_dirs.append(name)
        dirnames[:] = kept_dirs
        for name in sorted(filenames):
            candidate = current / name
            if candidate.is_symlink():
                symlinks.append(candidate.relative_to(safe_root).as_posix())
                continue
            safe = contained(candidate, must_exist=True)
            if safe.is_file():
                files.append(safe)
    return sorted(files), sorted(symlinks)


def safe_dependency(base: Path, reference: str, package_root: Path) -> Path:
    raw = reference.strip()
    parsed = urlparse(raw)
    if not raw or parsed.scheme or parsed.netloc or Path(raw).is_absolute() or PureWindowsPath(raw).is_absolute():
        raise ValueError(f"dependency is not a relative filesystem path: {reference!r}")
    normalized = os.path.normpath(raw.replace("\\", "/"))
    if normalized == ".." or normalized.startswith("../"):
        raise ValueError(f"dependency traverses outside package: {reference!r}")
    candidate = contained(base / normalized, must_exist=False)
    root = contained(package_root, must_exist=True)
    if candidate != root and root not in candidate.parents:
        raise ValueError(f"dependency escapes copied package: {reference!r}")
    return candidate


def finite_vector(node: ET.Element | None, attribute: str, length: int = 3) -> list[float] | None:
    if node is None or attribute not in node.attrib:
        return None
    try:
        values = [float(value) for value in node.attrib[attribute].split()]
    except (TypeError, ValueError):
        return None
    if len(values) != length or not all(math.isfinite(value) for value in values):
        return None
    return values


def finite_float(value: str | None) -> float | None:
    try:
        parsed = float(value) if value is not None else None
    except (TypeError, ValueError):
        return None
    return parsed if parsed is not None and math.isfinite(parsed) else None


def valid_tree(root: ET.Element) -> tuple[bool, list[str]]:
    links = [node.attrib.get("name", "") for node in root.findall("link")]
    errors: list[str] = []
    if not links or any(not name for name in links) or len(set(links)) != len(links):
        return False, ["links are empty, unnamed, or duplicated"]
    link_set = set(links)
    children: dict[str, list[str]] = {name: [] for name in links}
    indegree: Counter[str] = Counter()
    valid_edges = 0
    for joint in root.findall("joint"):
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        if parent not in link_set or child not in link_set:
            errors.append(f"joint {joint.attrib.get('name', '')!r} has unresolved endpoint")
            continue
        children[parent].append(child)
        indegree[child] += 1
        valid_edges += 1
    roots = [name for name in links if indegree[name] == 0]
    if len(roots) != 1 or any(indegree[name] > 1 for name in links):
        errors.append(f"invalid root/parent multiplicity: roots={len(roots)}")
        return False, errors
    visited: set[str] = set()
    active: set[str] = set()

    def walk(name: str) -> bool:
        if name in active:
            return False
        if name in visited:
            return True
        active.add(name)
        for child in children[name]:
            if not walk(child):
                return False
        active.remove(name)
        visited.add(name)
        return True

    acyclic = walk(roots[0])
    passed = acyclic and len(visited) == len(links) and valid_edges == len(links) - 1 and not errors
    if not passed and not errors:
        errors.append("link graph is cyclic, disconnected, or has wrong edge count")
    return passed, errors


def placeholder_name(name: str) -> bool:
    normalized = name.strip().lower()
    if not normalized:
        return True
    patterns = (
        r"^\d+$",
        r"^(?:l|link|part|mesh|object|obj|node|geometry|geom|visual)[_. -]*\d+$",
        r"^abstract(?:[_. -].*)?$",
        r"^(?:unnamed|default|none|null)(?:[_. -].*)?$",
    )
    return any(re.fullmatch(pattern, normalized) for pattern in patterns)


def positive_definite_inertia(inertia: ET.Element | None, np: Any) -> bool:
    if inertia is None:
        return False
    values = {key: finite_float(inertia.attrib.get(key)) for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
    if any(value is None for value in values.values()):
        return False
    matrix = np.array(
        [
            [values["ixx"], values["ixy"], values["ixz"]],
            [values["ixy"], values["iyy"], values["iyz"]],
            [values["ixz"], values["iyz"], values["izz"]],
        ],
        dtype=float,
    )
    return bool(np.all(np.linalg.eigvalsh(matrix) > 0.0))


def has_contact_metadata(root: ET.Element) -> bool:
    names = {
        "contact",
        "friction",
        "mu",
        "mu1",
        "mu2",
        "lateral_friction",
        "rolling_friction",
        "spinning_friction",
    }
    for node in root.iter():
        local = node.tag.rsplit("}", 1)[-1].lower()
        if local in names:
            return True
        if any(key.lower() in names for key in node.attrib):
            return True
    return False


def urdf_field_audit(urdf_path: Path, np: Any) -> dict[str, Any]:
    safe_urdf = contained(urdf_path, must_exist=True)
    robot = ET.parse(safe_urdf).getroot()
    if robot.tag != "robot":
        raise ValueError(f"expected URDF robot root, found {robot.tag!r}")
    links = robot.findall("link")
    link_names = [node.attrib.get("name", "") for node in links]
    link_set = set(link_names)
    joints = robot.findall("joint")
    tree_pass, tree_errors = valid_tree(robot)

    mesh_bearing_names = [
        node.attrib.get("name", "") for node in links if node.findall(".//visual/geometry/mesh")
    ]
    semantic_named = [name for name in mesh_bearing_names if name and not placeholder_name(name)]
    semantic_role_nodes = 0
    for node in robot.iter():
        if any(key.lower() in {"semantic", "semantic_role", "role", "category", "label"} for key in node.attrib):
            semantic_role_nodes += 1
    semantic_proxy_pass = bool(mesh_bearing_names) and len(semantic_named) == len(mesh_bearing_names) and tree_pass

    joint_rows: list[dict[str, Any]] = []
    for node in joints:
        joint_type = node.attrib.get("type", "")
        parent_node = node.find("parent")
        child_node = node.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        origin = node.find("origin")
        origin_valid = finite_vector(origin, "xyz") is not None and finite_vector(origin, "rpy") is not None
        axis = finite_vector(node.find("axis"), "xyz")
        axis_valid = axis is not None and sum(value * value for value in axis) > 1e-18
        limit = node.find("limit")
        lower = finite_float(limit.attrib.get("lower")) if limit is not None else None
        upper = finite_float(limit.attrib.get("upper")) if limit is not None else None
        limit_valid = lower is not None and upper is not None and lower <= upper
        dynamics = node.find("dynamics")
        damping = finite_float(dynamics.attrib.get("damping")) if dynamics is not None else None
        friction = finite_float(dynamics.attrib.get("friction")) if dynamics is not None else None
        joint_rows.append(
            {
                "name": node.attrib.get("name", ""),
                "type": joint_type,
                "movable": joint_type != "fixed",
                "endpoints_valid": parent in link_set and child in link_set,
                "type_valid": joint_type in JOINT_TYPES,
                "origin_valid": origin_valid,
                "axis_required": joint_type in ONE_AXIS_JOINT_TYPES,
                "axis_valid": axis_valid,
                "limit_required": joint_type in BOUNDED_JOINT_TYPES,
                "limit_valid": limit_valid,
                "dynamics_valid": damping is not None and friction is not None,
            }
        )

    movable_rows = [row for row in joint_rows if row["movable"]]
    kinematic_gates = {
        "all_endpoints_resolve": all(row["endpoints_valid"] for row in joint_rows),
        "all_joint_types_recognized": all(row["type_valid"] for row in joint_rows),
        "all_required_axes_valid": all(not row["axis_required"] or row["axis_valid"] for row in joint_rows),
        "all_required_limits_valid": all(not row["limit_required"] or row["limit_valid"] for row in joint_rows),
        "all_joint_origins_valid": all(row["origin_valid"] for row in joint_rows),
    }
    if not movable_rows:
        kinematic_state = "not_evaluable"
        kinematic_reason = "native package exposes no movable joint"
    else:
        kinematic_state = "pass" if all(kinematic_gates.values()) else "fail"
        kinematic_reason = None if kinematic_state == "pass" else "one or more frozen protocol field gates failed"

    visual_links = [node for node in links if node.findall("visual")]
    collision_complete = all(bool(node.findall("collision")) for node in visual_links) and bool(visual_links)
    mass_complete = True
    inertia_complete = True
    for link in links:
        inertial = link.find("inertial")
        mass = finite_float(inertial.find("mass").attrib.get("value")) if inertial is not None and inertial.find("mass") is not None else None
        mass_complete &= mass is not None and mass > 0.0
        inertia_complete &= inertial is not None and positive_definite_inertia(inertial.find("inertia"), np)
    dynamics_complete = all(row["dynamics_valid"] for row in movable_rows)
    contact_metadata = has_contact_metadata(robot)
    physical_gates = {
        "visual_links_have_native_collision": bool(collision_complete),
        "all_links_have_positive_finite_mass": bool(mass_complete),
        "all_links_have_positive_definite_inertia": bool(inertia_complete),
        "movable_joints_have_native_damping_and_friction": bool(dynamics_complete),
        "native_contact_or_friction_metadata_present": bool(contact_metadata),
    }
    physical_state = "pass" if all(physical_gates.values()) else "fail"

    return {
        "valid_urdf": True,
        "valid_tree": tree_pass,
        "tree_errors": tree_errors,
        "link_count": len(links),
        "joint_count": len(joints),
        "movable_joint_count": len(movable_rows),
        "visual_count": len(robot.findall(".//visual")),
        "collision_element_count": len(robot.findall(".//collision")),
        "mesh_reference_count": len(robot.findall(".//mesh")),
        "semantic_complete_state": "not_evaluable",
        "semantic_complete_reason": "no output-independent required-part and role specification",
        "semantic_field_proxy_state": "pass" if semantic_proxy_pass else "fail",
        "semantic_field_proxy": {
            "mesh_bearing_part_count": len(mesh_bearing_names),
            "nonempty_nonplaceholder_name_count": len(semantic_named),
            "hierarchy_is_rooted_connected_acyclic": tree_pass,
            "native_semantic_role_node_count": semantic_role_nodes,
            "native_semantic_roles_required_by_frozen_contract": False,
        },
        "kinematic_complete_state": kinematic_state,
        "kinematic_complete_reason": kinematic_reason,
        "kinematic_field_gates": kinematic_gates,
        "physical_complete_state": physical_state,
        "physical_complete_reason": None if physical_state == "pass" else "one or more native physical metadata gates failed",
        "physical_field_gates": physical_gates,
        "physical_counts": {
            "visual_bearing_link_count": len(visual_links),
            "visual_bearing_links_with_collision": sum(bool(node.findall("collision")) for node in visual_links),
            "links_with_positive_finite_mass": sum(
                (
                    (value := finite_float(node.find("inertial/mass").attrib.get("value"))) is not None
                    and value > 0.0
                )
                for node in links
                if node.find("inertial/mass") is not None
            ),
            "movable_joints_with_native_damping_and_friction": sum(row["dynamics_valid"] for row in movable_rows),
        },
    }


def geometry_audit(mesh_paths: list[Path], package_root: Path, np: Any, trimesh: Any) -> dict[str, Any]:
    geometry_count = 0
    readable_mesh_files = 0
    watertight = 0
    manifold = 0
    open_edges = 0
    degenerate_faces = 0
    errors: list[str] = []
    inventory = hashlib.sha256()
    mesh_bytes = 0
    for mesh_path in mesh_paths:
        safe = contained(mesh_path, must_exist=True)
        relative = safe.relative_to(package_root).as_posix()
        size = safe.stat().st_size
        mesh_bytes += size
        encoded = relative.encode("utf-8")
        inventory.update(len(encoded).to_bytes(8, "big"))
        inventory.update(encoded)
        inventory.update(size.to_bytes(8, "big"))
        try:
            loaded = trimesh.load(safe, force="scene", process=False)
            geometries = list(loaded.geometry.values()) if hasattr(loaded, "geometry") else [loaded]
            geometries = [item for item in geometries if hasattr(item, "faces") and hasattr(item, "vertices")]
            scored = 0
            for index, mesh in enumerate(geometries):
                faces = np.asarray(mesh.faces)
                vertices = np.asarray(mesh.vertices)
                if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0:
                    errors.append(f"{relative}#{index}: no readable triangle geometry")
                    continue
                if not np.isfinite(vertices).all() or not np.isfinite(np.asarray(mesh.area_faces)).all():
                    errors.append(f"{relative}#{index}: non-finite geometry")
                    continue
                edges = np.sort(
                    np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]), axis=0),
                    axis=1,
                )
                _, counts = np.unique(edges, axis=0, return_counts=True)
                repeated = (faces[:, 0] == faces[:, 1]) | (faces[:, 1] == faces[:, 2]) | (faces[:, 0] == faces[:, 2])
                areas = np.asarray(mesh.area_faces)
                geometry_count += 1
                scored += 1
                watertight += int(bool(mesh.is_watertight))
                manifold += int(bool(np.all(counts <= 2)))
                open_edges += int(np.count_nonzero(counts == 1))
                degenerate_faces += int(np.count_nonzero(repeated | (areas <= 1e-12)))
            readable_mesh_files += int(scored > 0)
            if scored == 0 and not geometries:
                errors.append(f"{relative}: no mesh geometry")
        except Exception as exc:  # noqa: BLE001
            errors.append(f"{relative}: {type(exc).__name__}: {exc}")
    return {
        "mesh_file_count": len(mesh_paths),
        "readable_mesh_file_count": readable_mesh_files,
        "readable_geometry_count": geometry_count,
        "mesh_bytes": mesh_bytes,
        "mesh_inventory_sha256": inventory.hexdigest(),
        "watertight_geometry_count": watertight,
        "watertight_fraction": watertight / geometry_count if geometry_count else None,
        "watertight_state": "pass" if geometry_count and watertight == geometry_count else ("fail" if geometry_count else "not_evaluable"),
        "edge_manifold_geometry_count": manifold,
        "edge_manifold_fraction": manifold / geometry_count if geometry_count else None,
        "manifold_state": "pass" if geometry_count and manifold == geometry_count else ("fail" if geometry_count else "not_evaluable"),
        "open_edges": open_edges,
        "degenerate_faces": degenerate_faces,
        "self_intersection_state": "not_evaluable",
        "self_intersection_reason": "no exact triangle-triangle backend with adjacent-face exclusion was run",
        "geometry_evaluable": geometry_count > 0,
        "geometry_loader_clean": bool(mesh_paths) and readable_mesh_files == len(mesh_paths) and not errors,
        "geometry_errors": errors,
    }


def parse_obj_dependencies(obj_path: Path, package_root: Path) -> tuple[list[Path], list[str]]:
    dependencies: list[Path] = []
    errors: list[str] = []
    safe = contained(obj_path, must_exist=True)
    for line_number, line in enumerate(safe.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        try:
            tokens = shlex.split(stripped, comments=True, posix=True)
        except ValueError as exc:
            errors.append(f"{safe.name}:{line_number}: invalid OBJ directive: {exc}")
            continue
        if tokens and tokens[0].lower() == "mtllib":
            for reference in tokens[1:]:
                try:
                    dependencies.append(safe_dependency(safe.parent, reference, package_root))
                except ValueError as exc:
                    errors.append(f"{safe.name}:{line_number}: {exc}")
    return dependencies, errors


def parse_mtl_dependencies(mtl_path: Path, package_root: Path) -> tuple[list[Path], list[str]]:
    dependencies: list[Path] = []
    errors: list[str] = []
    safe = contained(mtl_path, must_exist=True)
    for line_number, line in enumerate(safe.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        try:
            tokens = shlex.split(stripped, comments=True, posix=True)
        except ValueError as exc:
            errors.append(f"{safe.name}:{line_number}: invalid MTL directive: {exc}")
            continue
        if tokens and tokens[0].lower() in TEXTURE_DIRECTIVES:
            if len(tokens) < 2:
                errors.append(f"{safe.name}:{line_number}: missing texture reference")
                continue
            reference = tokens[-1]
            try:
                dependencies.append(safe_dependency(safe.parent, reference, package_root))
            except ValueError as exc:
                errors.append(f"{safe.name}:{line_number}: {exc}")
    return dependencies, errors


def parse_gltf_dependencies(gltf_path: Path, package_root: Path) -> tuple[list[Path], list[str]]:
    dependencies: list[Path] = []
    errors: list[str] = []
    safe = contained(gltf_path, must_exist=True)
    try:
        payload = json.loads(safe.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        return [], [f"{safe.name}: invalid glTF JSON: {type(exc).__name__}: {exc}"]
    for section in ("buffers", "images"):
        for item in payload.get(section, []):
            reference = item.get("uri")
            if not reference or str(reference).startswith("data:"):
                continue
            try:
                dependencies.append(safe_dependency(safe.parent, str(reference), package_root))
            except ValueError as exc:
                errors.append(f"{safe.name}: {exc}")
    return dependencies, errors


def portable_copy_audit(package_root: Path, urdf_relative: str, temp_parent: Path) -> dict[str, Any]:
    source = contained(package_root, must_exist=True)
    temp_base = contained(temp_parent, must_exist=True)
    _, source_symlinks = tree_files(source)
    errors: list[str] = []
    if source_symlinks:
        errors.extend(f"source package contains symlink: {item}" for item in source_symlinks[:20])
    dependency_paths: set[Path] = set()
    dependency_types: Counter[str] = Counter()
    primary_parsed = False
    with tempfile.TemporaryDirectory(prefix="portable_", dir=temp_base) as temporary:
        temporary_root = contained(Path(temporary), must_exist=True)
        copied = temporary_root / "package"
        shutil.copytree(source, copied, symlinks=True)
        copied = contained(copied, must_exist=True)
        _, copied_symlinks = tree_files(copied)
        if copied_symlinks:
            errors.extend(f"copied package contains symlink: {item}" for item in copied_symlinks[:20])
        try:
            copied_urdf = safe_dependency(copied, urdf_relative, copied)
            if not copied_urdf.is_file() or copied_urdf.stat().st_size == 0:
                raise ValueError("copied primary URDF is missing or empty")
            robot = ET.parse(copied_urdf).getroot()
            if robot.tag != "robot":
                raise ValueError(f"copied primary root is {robot.tag!r}, expected robot")
            primary_parsed = True
            queue: list[Path] = []
            for node in robot.findall(".//mesh"):
                reference = node.attrib.get("filename", "")
                try:
                    queue.append(safe_dependency(copied_urdf.parent, reference, copied))
                    dependency_types["urdf_mesh"] += 1
                except ValueError as exc:
                    errors.append(f"URDF mesh reference {reference!r}: {exc}")
            seen: set[Path] = set()
            while queue:
                dependency = queue.pop()
                if dependency in seen:
                    continue
                seen.add(dependency)
                dependency_paths.add(dependency)
                if not dependency.is_file() or dependency.stat().st_size == 0:
                    errors.append(f"missing or empty dependency: {dependency.relative_to(copied).as_posix()}")
                    continue
                suffix = dependency.suffix.lower()
                nested: list[Path] = []
                nested_errors: list[str] = []
                if suffix == ".obj":
                    nested, nested_errors = parse_obj_dependencies(dependency, copied)
                    dependency_types["obj"] += 1
                elif suffix == ".mtl":
                    nested, nested_errors = parse_mtl_dependencies(dependency, copied)
                    dependency_types["mtl"] += 1
                elif suffix == ".gltf":
                    nested, nested_errors = parse_gltf_dependencies(dependency, copied)
                    dependency_types["gltf"] += 1
                else:
                    dependency_types[suffix.lstrip(".") or "extensionless"] += 1
                errors.extend(nested_errors)
                queue.extend(nested)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"copied primary parse failed: {type(exc).__name__}: {exc}")
    passed = primary_parsed and not errors
    return {
        "portable_package_state": "pass" if passed else "fail",
        "portable_primary_parsed": primary_parsed,
        "portable_declared_dependency_count": len(dependency_paths),
        "portable_dependency_type_counts": dict(sorted(dependency_types.items())),
        "portable_errors": errors,
    }


def audit_available_asset(payload: dict[str, Any]) -> dict[str, Any]:
    import numpy as np
    import trimesh

    package_root = contained(Path(payload["package_root"]), must_exist=True)
    urdf_path = contained(Path(payload["urdf_path"]), must_exist=True)
    temp_parent = contained(Path(payload["temp_parent"]), must_exist=True)
    files, symlinks = tree_files(package_root)
    if symlinks:
        raise RuntimeError(f"input package contains symlinks: {symlinks[:20]}")
    mesh_paths = sorted({path.resolve(strict=True) for path in files if path.suffix.lower() in MESH_SUFFIXES})
    urdf_relative = urdf_path.relative_to(package_root).as_posix()
    record = dict(payload["record"])
    record.update(
        {
            "availability_state": "pass",
            "package_evaluable": True,
            "package_path": relative_to_repo(package_root),
            "urdf_path": relative_to_repo(urdf_path),
            "urdf_bytes": urdf_path.stat().st_size,
            "urdf_sha256": sha256_file(urdf_path),
            "package_file_count": len(files),
            "package_bytes": sum(path.stat().st_size for path in files),
            "package_symlink_count": 0,
            "source_bytes": None,
            "source_size_state": "not_evaluable",
            "source_size_reason": "factory source is shared and reported separately; no per-asset executable source is delivered",
            "deterministic_build_state": "not_evaluable",
            "deterministic_build_reason": "no second fresh same-factory same-seed build was executed",
        }
    )
    record.update(urdf_field_audit(urdf_path, np))
    record.update(geometry_audit(mesh_paths, package_root, np, trimesh))
    record.update(portable_copy_audit(package_root, urdf_relative, temp_parent))
    if record["movable_joint_count"] != payload["table6_movable_joint_count"]:
        raise RuntimeError(
            f"{record['asset_id']}: parsed movable joints {record['movable_joint_count']} "
            f"!= Table 6 {payload['table6_movable_joint_count']}"
        )
    return record


def unavailable_record(factory: str, seed: int, source: dict[str, Any], table6: dict[str, Any], factory_source: dict[str, Any]) -> dict[str, Any]:
    return {
        "asset_id": f"{factory}__seed_{seed:03d}",
        "factory": factory,
        "seed": seed,
        "requested": True,
        "generation_status": source["status"],
        "table6_evaluation_status": table6["evaluation_status"],
        "availability_state": "not_evaluable",
        "availability_reason": "frozen generation did not produce a strict final package",
        "package_evaluable": False,
        "geometry_evaluable": False,
        "watertight_state": "not_evaluable",
        "manifold_state": "not_evaluable",
        "self_intersection_state": "not_evaluable",
        "self_intersection_reason": "no final mesh package",
        "source_bytes": None,
        "source_size_state": "not_evaluable",
        "shared_factory_source_file": factory_source["path"],
        "shared_factory_source_bytes": factory_source["bytes"],
        "urdf_bytes": None,
        "mesh_bytes": None,
        "portable_package_state": "not_evaluable",
        "deterministic_build_state": "not_evaluable",
        "deterministic_build_reason": "no final package and no second fresh build",
        "semantic_complete_state": "not_evaluable",
        "semantic_complete_reason": "no final package and no output-independent required-part and role specification",
        "semantic_field_proxy_state": "not_evaluable",
        "kinematic_complete_state": "not_evaluable",
        "kinematic_complete_reason": "no final package",
        "physical_complete_state": "not_evaluable",
        "physical_complete_reason": "no native final simulation package",
        "generation_errors": list(source.get("validation", {}).get("errors", [])),
    }


def find_factory_sources(baseline_root: Path, factories: list[str]) -> dict[str, dict[str, Any]]:
    source_root = contained(baseline_root / "infinigen/assets/objects", must_exist=True)
    files, symlinks = tree_files(source_root)
    if symlinks:
        raise RuntimeError(f"factory source tree contains symlinks: {symlinks[:20]}")
    matches: dict[str, list[Path]] = {factory: [] for factory in factories}
    wanted = set(factories)
    for path in (item for item in files if item.suffix == ".py"):
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except (SyntaxError, UnicodeDecodeError):
            continue
        names = {node.name for node in tree.body if isinstance(node, ast.ClassDef)}
        for name in names & wanted:
            matches[name].append(path)
    result: dict[str, dict[str, Any]] = {}
    for factory in factories:
        paths = matches[factory]
        if len(paths) != 1:
            raise RuntimeError(f"expected one defining source file for {factory}, found {paths}")
        path = paths[0]
        result[factory] = {
            "path": relative_to_repo(path),
            "bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
    return result


def build_summary(records: list[dict[str, Any]], factory_sources: dict[str, dict[str, Any]], elapsed: float) -> dict[str, Any]:
    requested = len(records)
    available = [row for row in records if row["availability_state"] == "pass"]
    unavailable = [row for row in records if row["availability_state"] == "not_evaluable"]
    geometry = [row for row in available if row.get("geometry_evaluable")]
    geometry_count = sum(row["readable_geometry_count"] for row in geometry)
    watertight_count = sum(row["watertight_geometry_count"] for row in geometry)
    manifold_count = sum(row["edge_manifold_geometry_count"] for row in geometry)
    portable_counts = Counter(row["portable_package_state"] for row in records)
    semantic_counts = Counter(row["semantic_complete_state"] for row in records)
    semantic_proxy_counts = Counter(row["semantic_field_proxy_state"] for row in records)
    kinematic_counts = Counter(row["kinematic_complete_state"] for row in records)
    physical_counts = Counter(row["physical_complete_state"] for row in records)
    movable = [row for row in available if row["movable_joint_count"] > 0]
    no_movable = [row for row in available if row["movable_joint_count"] == 0]
    unique_sources = {item["path"]: item for item in factory_sources.values()}
    collision_total = sum(row["collision_element_count"] for row in available)
    return {
        "protocol_id": "nano3d_table7_production_readiness_v1",
        "method": "Infinite Mobility",
        "generated_at": utc_now(),
        "elapsed_seconds": elapsed,
        "denominators": {
            "requested_assets": requested,
            "available_assets": len(available),
            "unavailable_assets": len(unavailable),
            "geometry_evaluable_assets": len(geometry),
            "package_evaluable_assets": len(available),
            "readable_geometries": geometry_count,
            "native_articulated_packages": len(movable),
            "native_simulation_packages": len(available),
        },
        "cohort_strata": {
            "generation_status_counts": dict(sorted(Counter(row["generation_status"] for row in records).items())),
            "generated_with_movable_joint": len(movable),
            "generated_without_movable_joint": len(no_movable),
            "generation_timeout": sum(row["generation_status"] == "TIMEOUT" for row in records),
        },
        "geometry": {
            "watertight": {
                "per_asset_mean_fraction": mean([row["watertight_fraction"] for row in geometry]),
                "geometry_level": rate(watertight_count, geometry_count),
                "all_readable_geometries_pass_assets": rate(
                    sum(row["watertight_state"] == "pass" for row in geometry), len(geometry)
                ),
            },
            "edge_manifold_proxy": {
                "definition": "every undirected edge has at most two incident faces; vertex-manifold is not claimed",
                "per_asset_mean_fraction": mean([row["edge_manifold_fraction"] for row in geometry]),
                "geometry_level": rate(manifold_count, geometry_count),
                "all_readable_geometries_pass_assets": rate(
                    sum(row["manifold_state"] == "pass" for row in geometry), len(geometry)
                ),
            },
            "open_edges": {
                "total": sum(row["open_edges"] for row in geometry),
                "per_geometry_evaluable_asset_mean": mean([row["open_edges"] for row in geometry]),
            },
            "degenerate_faces": {
                "total": sum(row["degenerate_faces"] for row in geometry),
                "per_geometry_evaluable_asset_mean": mean([row["degenerate_faces"] for row in geometry]),
            },
            "self_intersection": {
                "state": "not_evaluable",
                "reason": "no exact triangle-triangle backend with adjacent-face exclusion was run",
            },
            "mesh_files": sum(row["mesh_file_count"] for row in available),
            "readable_mesh_files": sum(row["readable_mesh_file_count"] for row in available),
            "loader_clean_assets": rate(sum(row["geometry_loader_clean"] for row in available), len(available)),
        },
        "size_bytes": {
            "source": {
                "per_asset_state": "not_evaluable",
                "reason": "Infinite Mobility uses shared factory source; no per-asset executable source is delivered",
                "shared_factory_count": len(factory_sources),
                "unique_defining_module_count": len(unique_sources),
                "unique_defining_module_bytes_total": sum(item["bytes"] for item in unique_sources.values()),
                "factory_assignment_module_bytes_mean": mean([item["bytes"] for item in factory_sources.values()]),
            },
            "urdf": {
                "denominator": "available_assets",
                "total": sum(row["urdf_bytes"] for row in available),
                "mean": mean([row["urdf_bytes"] for row in available]),
            },
            "mesh": {
                "denominator": "available_assets",
                "total": sum(row["mesh_bytes"] for row in available),
                "mean": mean([row["mesh_bytes"] for row in available]),
            },
        },
        "portable_package": {
            "state_counts_requested": dict(sorted(portable_counts.items())),
            "pass_available": rate(portable_counts["pass"], len(available)),
            "pass_intent_to_run": rate(portable_counts["pass"], requested),
        },
        "deterministic_build": {
            "state_counts_requested": dict(sorted(Counter(row["deterministic_build_state"] for row in records).items())),
            "state": "not_evaluable",
            "reason": "no second fresh same-factory same-seed build was executed",
        },
        "semantic_complete": {
            "strict_state_counts_requested": dict(sorted(semantic_counts.items())),
            "strict_state": "not_evaluable",
            "reason": "no output-independent required-part and role specification",
            "field_proxy_state_counts_requested": dict(sorted(semantic_proxy_counts.items())),
            "field_proxy_pass_available": rate(semantic_proxy_counts["pass"], len(available)),
        },
        "kinematic_complete": {
            "state_counts_requested": dict(sorted(kinematic_counts.items())),
            "pass_applicable": rate(kinematic_counts["pass"], len(movable)),
            "pass_intent_to_run": rate(kinematic_counts["pass"], requested),
            "not_evaluable_no_movable_joint": len(no_movable),
            "not_evaluable_no_package": len(unavailable),
        },
        "physical_complete": {
            "state_counts_requested": dict(sorted(physical_counts.items())),
            "pass_applicable": rate(physical_counts["pass"], len(available)),
            "pass_intent_to_run": rate(physical_counts["pass"], requested),
            "native_collision_element_total": collision_total,
            "fail_closed_reason": (
                "all available exported URDFs contain zero native collision elements; "
                "absence of detected contact is not physical completeness"
            ),
        },
        "limitations": [
            "The frozen public 20-factory cohort is not a common-category matched subset.",
            "Seven generation timeouts remain in the 720-case intent-to-run denominator.",
            "Self-intersection is not evaluable because no exact backend with adjacent-face exclusion was run.",
            "Deterministic build is not evaluable because this static audit did not execute two fresh builds.",
            "Semantic completeness is not evaluable without output-independent required-part and role specifications; the name/tree field proxy is separate.",
            "Physical completeness fails closed on native metadata; runtime defaults and no-contact observations are never accepted.",
        ],
    }


def render_report(summary: dict[str, Any], protocol_sha: str, manifest_sha: str | None = None) -> str:
    geometry = summary["geometry"]
    sizes = summary["size_bytes"]
    portable = summary["portable_package"]
    kinematic = summary["kinematic_complete"]
    physical = summary["physical_complete"]
    den = summary["denominators"]
    wt = geometry["watertight"]
    mf = geometry["edge_manifold_proxy"]
    manifest_text = manifest_sha or "recorded after report generation in self_check.json"
    return f"""# Infinite Mobility Table 7 Production Readiness

## Frozen cohort

- Protocol: `nano3d_table7_production_readiness_v1`
- Protocol SHA-256: `{protocol_sha}`
- Manifest SHA-256: `{manifest_text}`
- Requested: {den['requested_assets']} (20 factories x 36 frozen seeds)
- Available strict final packages: {den['available_assets']}
- Generation timeouts retained: {den['unavailable_assets']}
- Generated with movable joints: {summary['cohort_strata']['generated_with_movable_joint']}
- Generated without movable joints: {summary['cohort_strata']['generated_without_movable_joint']}

## Table 7 row evidence

| Metric | Locally measured result |
|---|---|
| Watertight | per-asset mean geometry fraction {wt['per_asset_mean_fraction']:.6f}; geometry-level {wt['geometry_level']['numerator']}/{wt['geometry_level']['denominator']} = {wt['geometry_level']['value']:.6f} |
| Manifold | edge-manifold proxy per-asset mean {mf['per_asset_mean_fraction']:.6f}; geometry-level {mf['geometry_level']['numerator']}/{mf['geometry_level']['denominator']} = {mf['geometry_level']['value']:.6f}; vertex-manifold not claimed |
| Open Edges | {geometry['open_edges']['per_geometry_evaluable_asset_mean']:.3f}/geometry-evaluable asset mean; {geometry['open_edges']['total']} total |
| Degenerate Faces | {geometry['degenerate_faces']['per_geometry_evaluable_asset_mean']:.3f}/geometry-evaluable asset mean; {geometry['degenerate_faces']['total']} total |
| Self-Intersection | N/A (`not_evaluable`: no exact backend with adjacent-face exclusion) |
| Source KB | per-asset N/A; {sizes['source']['unique_defining_module_bytes_total'] / 1024.0:.3f} KiB across {sizes['source']['unique_defining_module_count']} unique shared factory-defining modules |
| URDF KB | {sizes['urdf']['mean'] / 1024.0:.3f} KiB/available asset; {sizes['urdf']['total'] / 1024.0:.3f} KiB total |
| Mesh KB | {sizes['mesh']['mean'] / 1024.0:.3f} KiB/available asset; {sizes['mesh']['total'] / 1024.0:.3f} KiB total packaged mesh payload |
| Portable Package | {portable['pass_available']['numerator']}/{portable['pass_available']['denominator']} available; {portable['pass_intent_to_run']['numerator']}/{portable['pass_intent_to_run']['denominator']} intent-to-run |
| Deterministic Build | N/A (`not_evaluable`: no second fresh same-factory same-seed build) |
| Semantic Complete | N/A strict; name/tree field proxy {summary['semantic_complete']['field_proxy_pass_available']['numerator']}/{summary['semantic_complete']['field_proxy_pass_available']['denominator']} available |
| Kinematic Complete | {kinematic['pass_applicable']['numerator']}/{kinematic['pass_applicable']['denominator']} native articulated packages; {kinematic['not_evaluable_no_movable_joint']} generated packages have no movable joint; {kinematic['not_evaluable_no_package']} unavailable |
| Physical Complete | {physical['pass_applicable']['numerator']}/{physical['pass_applicable']['denominator']} available native simulation packages; {physical['native_collision_element_total']} native collision elements |

## Claim boundary

The 720 requested factory-seed identities are conserved without success-based reselection. Geometry load failures, if any, are excluded only from geometry-evaluable denominators and are not counted as clean meshes. Semantic strict completeness and deterministic rebuild remain N/A. Physical completeness is fail-closed: these exports contain zero native collision elements, so a simulator reporting no contact would be vacuous and is not used.
"""


def make_self_check(
    records: list[dict[str, Any]],
    summary: dict[str, Any],
    expected_ids: list[str],
    evidence_paths: list[Path],
    protocol_sha: str,
    manifest_sha: str,
    table6_no_movable: int,
) -> dict[str, Any]:
    identities = [row["asset_id"] for row in records]
    requested = summary["denominators"]["requested_assets"]
    available = summary["denominators"]["available_assets"]
    portable_counts = Counter(row["portable_package_state"] for row in records)
    checks = [
        {"name": "manifest_identities_unique", "pass": len(identities) == len(set(identities))},
        {"name": "manifest_matches_frozen_cartesian_product", "pass": identities == expected_ids},
        {"name": "requested_is_720", "pass": requested == 720},
        {
            "name": "requested_equals_available_plus_unavailable",
            "pass": requested == available + summary["denominators"]["unavailable_assets"],
        },
        {
            "name": "portable_states_conserve_requested",
            "pass": requested == sum(portable_counts.values()) and available == portable_counts["pass"] + portable_counts["fail"],
        },
        {
            "name": "generation_strata_conserve_requested",
            "pass": requested
            == summary["cohort_strata"]["generated_with_movable_joint"]
            + summary["cohort_strata"]["generated_without_movable_joint"]
            + summary["cohort_strata"]["generation_timeout"],
        },
        {
            "name": "no_movable_stratum_matches_table6",
            "pass": summary["cohort_strata"]["generated_without_movable_joint"] == table6_no_movable == 55,
        },
        {
            "name": "all_rates_have_explicit_numerators_and_denominators",
            "pass": all(
                set(item) >= {"numerator", "denominator", "value"}
                for item in (
                    summary["geometry"]["watertight"]["geometry_level"],
                    summary["geometry"]["edge_manifold_proxy"]["geometry_level"],
                    summary["portable_package"]["pass_available"],
                    summary["kinematic_complete"]["pass_applicable"],
                    summary["physical_complete"]["pass_applicable"],
                )
            ),
        },
        {
            "name": "all_evidence_files_exist_inside_workspace",
            "pass": all(contained(path, must_exist=True).is_file() for path in evidence_paths),
        },
        {
            "name": "protocol_and_manifest_sha256_recorded",
            "pass": len(protocol_sha) == 64 and len(manifest_sha) == 64,
        },
        {
            "name": "physical_complete_fail_closed_on_zero_collision",
            "pass": summary["physical_complete"]["native_collision_element_total"] == 0
            and summary["physical_complete"]["pass_applicable"]["numerator"] == 0,
        },
    ]
    return {
        "protocol_id": summary["protocol_id"],
        "generated_at": utc_now(),
        "checks": checks,
        "passed": sum(item["pass"] for item in checks),
        "total": len(checks),
        "all_passed": all(item["pass"] for item in checks),
        "hashes": {
            "protocol_snapshot_sha256": protocol_sha,
            "manifest_sha256": manifest_sha,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--table6-root", type=Path, default=DEFAULT_TABLE6_ROOT)
    parser.add_argument("--generation-root", type=Path, default=DEFAULT_GENERATION_ROOT)
    parser.add_argument("--baseline-root", type=Path, default=DEFAULT_BASELINE_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=2)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    assert_workspace()
    if args.workers < 1:
        raise ValueError("--workers must be >= 1")
    protocol_path = contained(args.protocol, must_exist=True)
    table6_root = contained(args.table6_root, must_exist=True)
    generation_root = contained(args.generation_root, must_exist=True)
    baseline_root = contained(args.baseline_root, must_exist=True)
    output = contained(args.output, must_exist=False)
    output.mkdir(parents=True, exist_ok=True)
    temp_parent = contained(output / ".portable_tmp", must_exist=False)
    temp_parent.mkdir(parents=True, exist_ok=True)

    protocol = read_json(protocol_path)
    if protocol.get("protocol_id") != "nano3d_table7_production_readiness_v1":
        raise RuntimeError(f"unexpected protocol: {protocol.get('protocol_id')!r}")
    snapshot_path = contained(output / "protocol_snapshot.json", must_exist=False)
    shutil.copyfile(protocol_path, snapshot_path)
    protocol_sha = sha256_file(snapshot_path)
    if protocol_sha != sha256_file(protocol_path):
        raise RuntimeError("protocol snapshot hash mismatch")

    generation_manifest_path = contained(generation_root / "manifest.json", must_exist=True)
    generation_records_path = contained(generation_root / "records.json", must_exist=True)
    table6_manifest_path = contained(table6_root / "manifest.json", must_exist=True)
    table6_records_path = contained(table6_root / "asset_records.json", must_exist=True)
    table6_summary_path = contained(table6_root / "summary.json", must_exist=True)
    generation_manifest = read_json(generation_manifest_path)
    generation_records = read_json(generation_records_path)
    table6_manifest = read_json(table6_manifest_path)
    table6_records = read_json(table6_records_path)
    table6_summary = read_json(table6_summary_path)

    generation_manifest_sha = sha256_file(generation_manifest_path)
    generation_records_sha = sha256_file(generation_records_path)
    if table6_summary["input"]["generation_manifest_sha256"] != generation_manifest_sha:
        raise RuntimeError("Table 6 generation manifest hash does not match current frozen input")
    if table6_summary["input"]["generation_records_sha256"] != generation_records_sha:
        raise RuntimeError("Table 6 generation records hash does not match current frozen input")
    if table6_manifest["source_generation_manifest"] != generation_manifest:
        raise RuntimeError("Table 6 embedded generation manifest differs from frozen generation manifest")

    factories = list(generation_manifest["factories"])
    seeds = [int(seed) for seed in generation_manifest["seeds"]]
    expected_pairs = [(factory, seed) for factory in factories for seed in seeds]
    expected_set = set(expected_pairs)
    generation_by_id = {(row["factory"], int(row["seed"])): row for row in generation_records}
    table6_by_id = {(row["factory"], int(row["seed"])): row for row in table6_records}
    if len(generation_by_id) != len(generation_records) or set(generation_by_id) != expected_set:
        raise RuntimeError("generation records do not exactly match frozen 20 x 36 Cartesian manifest")
    if len(table6_by_id) != len(table6_records) or set(table6_by_id) != expected_set:
        raise RuntimeError("Table 6 records do not exactly match frozen 20 x 36 Cartesian manifest")
    if len(expected_pairs) != 720:
        raise RuntimeError(f"expected 720 frozen requests, found {len(expected_pairs)}")

    factory_sources = find_factory_sources(baseline_root, factories)
    base_records: list[dict[str, Any]] = []
    payloads: list[dict[str, Any]] = []
    for factory, seed in expected_pairs:
        source = generation_by_id[(factory, seed)]
        table6 = table6_by_id[(factory, seed)]
        if source["status"] != table6["generation_status"]:
            raise RuntimeError(f"generation status mismatch for {factory} seed {seed}")
        if bool(source["status"] == "PASS") != bool(table6["strict_package_pass"]):
            raise RuntimeError(f"strict package status mismatch for {factory} seed {seed}")
        source_info = factory_sources[factory]
        if source["status"] != "PASS":
            base_records.append(unavailable_record(factory, seed, source, table6, source_info))
            continue
        relative_urdf = Path(table6["urdf_path"])
        if relative_urdf.is_absolute() or ".." in relative_urdf.parts:
            raise RuntimeError(f"non-relative Table 6 URDF path for {factory} seed {seed}")
        urdf_path = contained(REPO_ROOT / relative_urdf, must_exist=True)
        package_root = contained(urdf_path.parents[1], must_exist=True)
        record = {
            "asset_id": f"{factory}__seed_{seed:03d}",
            "factory": factory,
            "seed": seed,
            "requested": True,
            "generation_status": source["status"],
            "table6_evaluation_status": table6["evaluation_status"],
            "frozen_package_sha256": source.get("package_sha256"),
            "frozen_package_hash_recomputed": False,
            "shared_factory_source_file": source_info["path"],
            "shared_factory_source_bytes": source_info["bytes"],
            "shared_factory_source_sha256": source_info["sha256"],
        }
        payloads.append(
            {
                "record": record,
                "package_root": str(package_root),
                "urdf_path": str(urdf_path),
                "temp_parent": str(temp_parent),
                "table6_movable_joint_count": int(table6.get("movable_joint_count", 0)),
            }
        )

    started = time.monotonic()
    completed: list[dict[str, Any]] = []
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        futures = {executor.submit(audit_available_asset, payload): payload["record"]["asset_id"] for payload in payloads}
        for index, future in enumerate(as_completed(futures), 1):
            asset_id = futures[future]
            try:
                completed.append(future.result())
            except Exception as exc:  # noqa: BLE001
                raise RuntimeError(f"asset audit failed for {asset_id}: {type(exc).__name__}: {exc}") from exc
            if index % 25 == 0 or index == len(futures):
                print(f"[{index}/{len(futures)}] available packages audited", flush=True)
    if any(temp_parent.iterdir()):
        raise RuntimeError(f"portable audit left temporary files under {temp_parent}")
    temp_parent.rmdir()

    records = sorted(base_records + completed, key=lambda row: (factories.index(row["factory"]), row["seed"]))
    elapsed = time.monotonic() - started
    summary = build_summary(records, factory_sources, elapsed)
    if summary["denominators"]["available_assets"] != 713 or summary["cohort_strata"]["generation_timeout"] != 7:
        raise RuntimeError("frozen 713 generated / 7 timeout strata changed")
    table6_no_movable = sum(
        row.get("strict_package_pass") and int(row.get("movable_joint_count", 0)) == 0 for row in table6_records
    )

    records_path = contained(output / "asset_records.json", must_exist=False)
    summary_path = contained(output / "summary.json", must_exist=False)
    report_path = contained(output / "report.md", must_exist=False)
    manifest_path = contained(output / "manifest.json", must_exist=False)
    self_check_path = contained(output / "self_check.json", must_exist=False)
    hashes_path = contained(output / "hashes.json", must_exist=False)
    write_json(records_path, records)
    write_json(summary_path, summary)
    report_path.write_text(render_report(summary, protocol_sha), encoding="utf-8")

    expected_ids = [f"{factory}__seed_{seed:03d}" for factory, seed in expected_pairs]
    manifest = {
        "protocol_id": protocol["protocol_id"],
        "method": "Infinite Mobility",
        "generated_at": utc_now(),
        "command": [str(SCRIPT_PATH), *sys.argv[1:]],
        "workspace_root": str(WORKSPACE_ROOT),
        "cohort": {
            "factories": factories,
            "seeds": seeds,
            "requested_identities": expected_ids,
            "requested_count": len(expected_ids),
            "selection": "exact Table 6 frozen Cartesian manifest; no outcome-based reselection",
        },
        "inputs": {
            relative_to_workspace(protocol_path): sha256_file(protocol_path),
            relative_to_workspace(generation_manifest_path): generation_manifest_sha,
            relative_to_workspace(generation_records_path): generation_records_sha,
            relative_to_workspace(table6_manifest_path): sha256_file(table6_manifest_path),
            relative_to_workspace(table6_records_path): sha256_file(table6_records_path),
            relative_to_workspace(table6_summary_path): sha256_file(table6_summary_path),
        },
        "protocol_snapshot_sha256": protocol_sha,
        "script_sha256": sha256_file(SCRIPT_PATH),
        "factory_sources": factory_sources,
        "output_hashes_before_manifest": {
            "protocol_snapshot.json": sha256_file(snapshot_path),
            "asset_records.json": sha256_file(records_path),
            "summary.json": sha256_file(summary_path),
            "report.md": sha256_file(report_path),
        },
        "environment": {
            "python": sys.version,
            "workers": args.workers,
            "geometry_loader": protocol["geometry"]["loader"],
        },
        "required_outputs": protocol["required_outputs"],
    }
    write_json(manifest_path, manifest)
    manifest_sha = sha256_file(manifest_path)
    evidence_paths = [
        protocol_path,
        generation_manifest_path,
        generation_records_path,
        table6_manifest_path,
        table6_records_path,
        table6_summary_path,
        SCRIPT_PATH,
        snapshot_path,
        records_path,
        summary_path,
        report_path,
        manifest_path,
    ] + [REPO_ROOT / item["path"] for item in factory_sources.values()]
    self_check = make_self_check(
        records,
        summary,
        expected_ids,
        evidence_paths,
        protocol_sha,
        manifest_sha,
        table6_no_movable,
    )
    write_json(self_check_path, self_check)
    hashes = {
        "algorithm": "sha256",
        "generated_at": utc_now(),
        "files": {
            name: sha256_file(output / name)
            for name in [
                "protocol_snapshot.json",
                "manifest.json",
                "asset_records.json",
                "summary.json",
                "self_check.json",
                "report.md",
            ]
        },
        "script": {relative_to_workspace(SCRIPT_PATH): sha256_file(SCRIPT_PATH)},
    }
    write_json(hashes_path, hashes)
    if not self_check["all_passed"]:
        failed = [item["name"] for item in self_check["checks"] if not item["pass"]]
        raise RuntimeError(f"self-check failed: {failed}")
    print(
        f"complete: requested={len(records)} available={len(completed)} "
        f"geometry_evaluable={summary['denominators']['geometry_evaluable_assets']} "
        f"self_check={self_check['passed']}/{self_check['total']}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
