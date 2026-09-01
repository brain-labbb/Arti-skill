#!/usr/bin/env python3
"""Run the frozen Table 7 production-readiness audit for LAM.

The input is the outcome-independent 100-category viable release cohort frozen
for Table 6.  This script audits released files only; it never executes LAM
generation code and never treats re-hashing a release as a deterministic build.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import hashlib
import json
import math
import re
import shlex
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath
from typing import Any


WORKSPACE = Path("/mnt/zsn/lyb")
REPO = Path(__file__).resolve().parents[2]
EXP = REPO / "exp"
DEFAULT_PROTOCOL = EXP / "reference/table7_production_readiness_protocol_v1.json"
DEFAULT_COHORT = EXP / "runtime/table6_lam/frozen_cohort.json"
DEFAULT_COHORT_ROOT = REPO / ".cache/table6_sources/lam/cohort_100"
DEFAULT_TABLE6_PREFLIGHT = EXP / "runtime/table6_lam/package_preflight.json"
DEFAULT_TABLE6_METADATA = EXP / "runtime/table6_lam/metadata_summary.json"
DEFAULT_TABLE6_FUNCTIONAL = EXP / "runtime/table6_lam/functional_summary.json"
DEFAULT_OUTPUT = EXP / "runtime/table7_lam"
MESH_SUFFIXES = {".obj", ".ply", ".stl", ".off", ".glb", ".gltf", ".dae"}
SOURCE_SUFFIXES = {".js", ".mjs", ".cjs", ".py", ".ts", ".tsx"}
ONE_AXIS_TYPES = {"revolute", "continuous", "prismatic"}
BOUNDED_TYPES = {"revolute", "prismatic"}
RECOGNIZED_JOINT_TYPES = {
    "fixed",
    "revolute",
    "continuous",
    "prismatic",
    "planar",
    "floating",
}
PLACEHOLDER_NAME = re.compile(
    r"^(?:link|part|mesh|object|component|unnamed)(?:[_-]?\d+)?$", re.IGNORECASE
)


def authorized(path: Path, *, must_exist: bool = True) -> Path:
    """Resolve a path and enforce the run's fixed workspace boundary."""
    resolved = path.resolve(strict=must_exist)
    workspace = WORKSPACE.resolve(strict=True)
    if resolved != workspace and workspace not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def relative_workspace(path: Path) -> str:
    return authorized(path).relative_to(authorized(WORKSPACE)).as_posix()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with authorized(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(authorized(path).read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    destination = authorized(path, must_exist=False)
    destination.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def write_text(path: Path, text: str) -> None:
    authorized(path, must_exist=False).write_text(text, encoding="utf-8")


def finite_vector(raw: str | None, length: int = 3) -> bool:
    if raw is None:
        return False
    try:
        values = [float(value) for value in raw.split()]
    except ValueError:
        return False
    return len(values) == length and all(math.isfinite(value) for value in values)


def finite_nonzero_vector(raw: str | None) -> bool:
    if not finite_vector(raw):
        return False
    values = [float(value) for value in raw.split()]  # type: ignore[union-attr]
    return sum(value * value for value in values) > 1e-18


def finite_float(raw: str | None) -> float | None:
    try:
        value = float(raw)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def state_counts(records: list[dict[str, Any]], key: str) -> dict[str, int]:
    counts = Counter(record[key]["state"] for record in records)
    return {
        "pass": counts["pass"],
        "fail": counts["fail"],
        "not_evaluable": counts["not_evaluable"],
        "requested": len(records),
    }


def rate(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": int(numerator),
        "denominator": int(denominator),
        "value": numerator / denominator if denominator else None,
    }


def package_files(package: Path) -> tuple[list[Path], list[str]]:
    root = authorized(package)
    files: list[Path] = []
    symlinks: list[str] = []
    for path in sorted(root.rglob("*"), key=lambda item: item.relative_to(root).as_posix()):
        candidate = authorized(path, must_exist=False)
        if candidate.is_symlink():
            symlinks.append(path.relative_to(root).as_posix())
        elif candidate.is_file():
            files.append(authorized(candidate))
    return files, symlinks


def package_content_hash(package: Path, files: list[Path]) -> str:
    root = authorized(package)
    digest = hashlib.sha256()
    for path in files:
        rel = path.relative_to(root).as_posix()
        digest.update(rel.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(path.stat().st_size).encode("ascii"))
        digest.update(b"\0")
        digest.update(sha256_file(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def lexical_relative_reference(raw: str) -> tuple[PurePosixPath | None, str | None]:
    value = raw.strip()
    if not value:
        return None, "empty dependency reference"
    if "\x00" in value:
        return None, "NUL in dependency reference"
    if "://" in value or re.match(r"^[A-Za-z]:[\\/]", value):
        return None, f"non-relative dependency reference: {value}"
    pure = PurePosixPath(value)
    if pure.is_absolute() or ".." in pure.parts:
        return None, f"unsafe dependency reference: {value}"
    return pure, None


def resolve_dependency(package: Path, base: Path, raw: str) -> tuple[Path | None, str | None]:
    pure, error = lexical_relative_reference(raw)
    if error:
        return None, error
    root = authorized(package)
    candidate = base.joinpath(*pure.parts)  # type: ignore[union-attr]
    resolved = authorized(candidate, must_exist=False)
    if resolved != root and root not in resolved.parents:
        return None, f"dependency escapes package: {raw}"
    if not resolved.exists():
        return None, f"missing dependency: {raw}"
    if resolved.is_symlink():
        return None, f"dependency is symlink: {raw}"
    resolved = authorized(resolved)
    if resolved != root and root not in resolved.parents:
        return None, f"dependency resolves outside package: {raw}"
    if not resolved.is_file():
        return None, f"dependency is not a file: {raw}"
    return resolved, None


def parse_obj_dependencies(path: Path) -> list[str]:
    dependencies: list[str] = []
    for line in authorized(path).read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = line.strip()
        if not stripped.lower().startswith("mtllib "):
            continue
        try:
            dependencies.extend(shlex.split(stripped)[1:])
        except ValueError:
            dependencies.append(stripped.split(maxsplit=1)[1])
    return dependencies


def parse_mtl_dependencies(path: Path) -> list[str]:
    tags = {
        "map_ka", "map_kd", "map_ks", "map_ke", "map_ns", "map_d",
        "map_bump", "bump", "disp", "decal", "refl", "norm",
    }
    dependencies: list[str] = []
    for line in authorized(path).read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            fields = shlex.split(line.strip())
        except ValueError:
            fields = line.strip().split()
        if len(fields) >= 2 and fields[0].lower() in tags:
            dependencies.append(fields[-1])
    return dependencies


def declared_dependency_audit(primary: Path, package: Path) -> dict[str, Any]:
    """Resolve URDF mesh/texture dependencies and representation-side dependencies."""
    root = authorized(package)
    primary = authorized(primary)
    errors: list[str] = []
    declared: list[dict[str, str]] = []
    resolved_paths: set[Path] = set()
    mesh_paths: set[Path] = set()
    try:
        urdf_root = ET.parse(primary).getroot()
    except Exception as exc:  # noqa: BLE001
        return {
            "parse_success": False,
            "errors": [f"primary_parse: {type(exc).__name__}: {exc}"],
            "declared": [],
            "resolved_paths": [],
            "mesh_paths": [],
        }

    queue: deque[tuple[Path, str, str]] = deque()
    for node in urdf_root.findall(".//mesh"):
        queue.append((primary.parent, node.get("filename", ""), "urdf_mesh"))
    for node in urdf_root.findall(".//texture"):
        queue.append((primary.parent, node.get("filename", ""), "urdf_texture"))

    expanded: set[Path] = set()
    while queue:
        base, raw, kind = queue.popleft()
        path, error = resolve_dependency(root, base, raw)
        declared.append({"kind": kind, "reference": raw})
        if error:
            errors.append(error)
            continue
        assert path is not None
        resolved_paths.add(path)
        if kind == "urdf_mesh":
            mesh_paths.add(path)
        if path in expanded:
            continue
        expanded.add(path)
        suffix = path.suffix.lower()
        if suffix == ".obj":
            for ref in parse_obj_dependencies(path):
                queue.append((path.parent, ref, "obj_material"))
        elif suffix == ".mtl":
            for ref in parse_mtl_dependencies(path):
                queue.append((path.parent, ref, "mtl_texture"))
        elif suffix == ".gltf":
            try:
                payload = read_json(path)
                for section, kind_name in (("buffers", "gltf_buffer"), ("images", "gltf_image")):
                    for entry in payload.get(section, []):
                        uri = entry.get("uri")
                        if isinstance(uri, str) and not uri.startswith("data:"):
                            queue.append((path.parent, uri, kind_name))
            except Exception as exc:  # noqa: BLE001
                errors.append(f"gltf_parse {path.name}: {type(exc).__name__}: {exc}")
    return {
        "parse_success": True,
        "errors": sorted(set(errors)),
        "declared": declared,
        "resolved_paths": sorted(
            (path.relative_to(root).as_posix() for path in resolved_paths)
        ),
        "mesh_paths": sorted(mesh_paths),
    }


def hierarchy_gate(root: ET.Element) -> dict[str, Any]:
    links = root.findall("link")
    joints = root.findall("joint")
    names = [link.get("name", "").strip() for link in links]
    link_set = set(names)
    valid_unique_links = bool(names) and all(names) and len(names) == len(link_set)
    adjacency: dict[str, list[str]] = {name: [] for name in link_set}
    indegree: dict[str, int] = {name: 0 for name in link_set}
    references_resolve = True
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        parent_name = parent.get("link", "") if parent is not None else ""
        child_name = child.get("link", "") if child is not None else ""
        if parent_name not in link_set or child_name not in link_set:
            references_resolve = False
            continue
        adjacency[parent_name].append(child_name)
        indegree[child_name] += 1
    roots = [name for name, degree in indegree.items() if degree == 0]
    visited: set[str] = set()
    active: set[str] = set()
    cycle = False

    def visit(name: str) -> None:
        nonlocal cycle
        if name in active:
            cycle = True
            return
        if name in visited:
            return
        active.add(name)
        for child in adjacency.get(name, []):
            visit(child)
        active.remove(name)
        visited.add(name)

    for name in link_set:
        visit(name)
    valid_tree = (
        valid_unique_links
        and references_resolve
        and len(roots) == 1
        and not cycle
        and len(visited) == len(link_set)
        and len(joints) == max(0, len(link_set) - 1)
        and all(degree <= 1 for degree in indegree.values())
    )
    return {
        "link_count": len(links),
        "joint_count": len(joints),
        "root_count": len(roots),
        "references_resolve": references_resolve,
        "unique_nonempty_links": valid_unique_links,
        "connected_acyclic_single_root": valid_tree,
    }


def semantic_proxy(root: ET.Element) -> dict[str, Any]:
    hierarchy = hierarchy_gate(root)
    mesh_names = []
    for link in root.findall("link"):
        if link.findall("./visual/geometry/mesh") or link.findall("./collision/geometry/mesh"):
            mesh_names.append(link.get("name", "").strip())
    names_ok = bool(mesh_names) and all(
        name and not PLACEHOLDER_NAME.fullmatch(name) for name in mesh_names
    )
    proxy_pass = names_ok and hierarchy["connected_acyclic_single_root"]
    return {
        "state": "pass" if proxy_pass else "fail",
        "mesh_bearing_part_count": len(mesh_names),
        "nonempty_nonplaceholder_names": names_ok,
        "hierarchy": hierarchy,
        "native_role_contract_required": False,
        "note": "field proxy only; not semantic correctness",
    }


def kinematic_gate(root: ET.Element) -> dict[str, Any]:
    joints = root.findall("joint")
    if not joints:
        return {"state": "not_evaluable", "reason": "native representation exposes no joints"}
    links = {link.get("name", "") for link in root.findall("link")}
    counts = {
        "joint_count": len(joints),
        "movable_joint_count": 0,
        "parent_child_resolve_count": 0,
        "recognized_type_count": 0,
        "required_axis_count": 0,
        "valid_axis_count": 0,
        "bounded_joint_count": 0,
        "valid_bounded_limit_count": 0,
        "explicit_finite_origin_count": 0,
    }
    for joint in joints:
        joint_type = joint.get("type", "")
        parent = joint.find("parent")
        child = joint.find("child")
        if (
            parent is not None and child is not None
            and parent.get("link", "") in links and child.get("link", "") in links
        ):
            counts["parent_child_resolve_count"] += 1
        if joint_type in RECOGNIZED_JOINT_TYPES:
            counts["recognized_type_count"] += 1
        if joint_type != "fixed":
            counts["movable_joint_count"] += 1
        if joint_type in ONE_AXIS_TYPES:
            counts["required_axis_count"] += 1
            axis = joint.find("axis")
            if axis is not None and finite_nonzero_vector(axis.get("xyz")):
                counts["valid_axis_count"] += 1
        if joint_type in BOUNDED_TYPES:
            counts["bounded_joint_count"] += 1
            limit = joint.find("limit")
            lower = finite_float(limit.get("lower") if limit is not None else None)
            upper = finite_float(limit.get("upper") if limit is not None else None)
            if lower is not None and upper is not None and lower <= upper:
                counts["valid_bounded_limit_count"] += 1
        origin = joint.find("origin")
        if (
            origin is not None
            and finite_vector(origin.get("xyz"))
            and finite_vector(origin.get("rpy"))
        ):
            counts["explicit_finite_origin_count"] += 1
    passed = (
        counts["parent_child_resolve_count"] == len(joints)
        and counts["recognized_type_count"] == len(joints)
        and counts["valid_axis_count"] == counts["required_axis_count"]
        and counts["valid_bounded_limit_count"] == counts["bounded_joint_count"]
        and counts["explicit_finite_origin_count"] == len(joints)
    )
    return {
        "state": "pass" if passed else "fail",
        **counts,
        "origin_rule": "every joint must explicitly declare finite xyz and rpy",
    }


def positive_definite_inertia(inertial: ET.Element | None) -> bool:
    if inertial is None:
        return False
    inertia = inertial.find("inertia")
    if inertia is None:
        return False
    values = {name: finite_float(inertia.get(name)) for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
    if any(value is None for value in values.values()):
        return False
    try:
        import numpy as np

        matrix = np.array(
            [
                [values["ixx"], values["ixy"], values["ixz"]],
                [values["ixy"], values["iyy"], values["iyz"]],
                [values["ixz"], values["iyz"], values["izz"]],
            ],
            dtype=float,
        )
        return bool(np.all(np.linalg.eigvalsh(matrix) > 0.0))
    except Exception:
        return False


def collision_geometry_present(link: ET.Element) -> bool:
    for collision in link.findall("collision"):
        geometry = collision.find("geometry")
        if geometry is not None and list(geometry):
            return True
    return False


def contact_metadata_present(root: ET.Element) -> bool:
    tags = {
        "contact", "surface", "friction", "lateral_friction", "rolling_friction",
        "spinning_friction", "mu1", "mu2",
    }
    for node in root.iter():
        tag = node.tag.rsplit("}", 1)[-1].lower()
        if tag in tags and not (tag == "friction" and node.tag == "dynamics"):
            return True
    return False


def physical_gate(root: ET.Element) -> dict[str, Any]:
    links = root.findall("link")
    physical_links = [
        link for link in links if link.find("visual") is not None or link.find("collision") is not None
    ]
    if not physical_links:
        return {"state": "not_evaluable", "reason": "no native physical links"}
    visual_links = [link for link in physical_links if link.find("visual") is not None]
    collision_complete = sum(collision_geometry_present(link) for link in visual_links)
    positive_mass = 0
    valid_inertia = 0
    for link in physical_links:
        inertial = link.find("inertial")
        mass = inertial.find("mass") if inertial is not None else None
        mass_value = finite_float(mass.get("value") if mass is not None else None)
        positive_mass += int(mass_value is not None and mass_value > 0.0)
        valid_inertia += int(positive_definite_inertia(inertial))
    movable = [joint for joint in root.findall("joint") if joint.get("type") != "fixed"]
    dynamics_complete = 0
    for joint in movable:
        dynamics = joint.find("dynamics")
        damping = finite_float(dynamics.get("damping") if dynamics is not None else None)
        friction = finite_float(dynamics.get("friction") if dynamics is not None else None)
        dynamics_complete += int(
            damping is not None and damping >= 0.0 and friction is not None and friction >= 0.0
        )
    contact = contact_metadata_present(root)
    passed = (
        collision_complete == len(visual_links)
        and positive_mass == len(physical_links)
        and valid_inertia == len(physical_links)
        and dynamics_complete == len(movable)
        and contact
    )
    return {
        "state": "pass" if passed else "fail",
        "physical_link_count": len(physical_links),
        "visual_link_count": len(visual_links),
        "visual_links_with_native_collision": collision_complete,
        "links_with_positive_finite_native_mass": positive_mass,
        "links_with_positive_definite_native_inertia": valid_inertia,
        "movable_joint_count": len(movable),
        "movable_joints_with_native_damping_and_friction": dynamics_complete,
        "native_contact_friction_metadata_present": contact,
        "runtime_defaults_counted_as_native": False,
    }


def mesh_geometry_audit(mesh_paths: list[Path], package: Path) -> dict[str, Any]:
    import numpy as np
    import trimesh

    root = authorized(package)
    geometry_records: list[dict[str, Any]] = []
    errors: list[dict[str, str]] = []
    readable_files = 0
    for path in mesh_paths:
        rel = path.relative_to(root).as_posix()
        if path.suffix.lower() not in MESH_SUFFIXES:
            errors.append({"mesh": rel, "error": "unsupported mesh suffix"})
            continue
        try:
            loaded = trimesh.load(authorized(path), force="scene", process=False)
            geometries = list(loaded.geometry.items()) if hasattr(loaded, "geometry") else [("geometry_0", loaded)]
            readable_in_file = 0
            for key, mesh in geometries:
                if not hasattr(mesh, "faces") or not hasattr(mesh, "vertices"):
                    continue
                faces = np.asarray(mesh.faces)
                if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0:
                    continue
                vertices = np.asarray(mesh.vertices)
                repeated = (
                    (faces[:, 0] == faces[:, 1])
                    | (faces[:, 1] == faces[:, 2])
                    | (faces[:, 0] == faces[:, 2])
                )
                areas = np.asarray(mesh.area_faces, dtype=float)
                degenerate = repeated | (~np.isfinite(areas)) | (areas <= 1e-12)
                edges = np.sort(
                    np.concatenate(
                        [faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]], axis=0
                    ),
                    axis=1,
                )
                _, incidence = np.unique(edges, axis=0, return_counts=True)
                readable_in_file += 1
                geometry_records.append(
                    {
                        "mesh": rel,
                        "geometry_key": str(key),
                        "vertex_count": int(len(vertices)),
                        "face_count": int(len(faces)),
                        "watertight": bool(mesh.is_watertight),
                        "edge_manifold": bool(np.all(incidence <= 2)),
                        "open_edges": int(np.sum(incidence == 1)),
                        "nonmanifold_edges": int(np.sum(incidence > 2)),
                        "degenerate_faces": int(np.sum(degenerate)),
                    }
                )
            if readable_in_file:
                readable_files += 1
            else:
                errors.append({"mesh": rel, "error": "no readable triangle geometry"})
        except Exception as exc:  # noqa: BLE001
            errors.append({"mesh": rel, "error": f"{type(exc).__name__}: {exc}"})
    denominator = len(geometry_records)
    return {
        "mesh_file_count": len(mesh_paths),
        "readable_mesh_file_count": readable_files,
        "readable_geometry_count": denominator,
        "watertight_geometry_count": sum(row["watertight"] for row in geometry_records),
        "edge_manifold_geometry_count": sum(row["edge_manifold"] for row in geometry_records),
        "open_edges": sum(row["open_edges"] for row in geometry_records),
        "nonmanifold_edges": sum(row["nonmanifold_edges"] for row in geometry_records),
        "degenerate_faces": sum(row["degenerate_faces"] for row in geometry_records),
        "watertight_fraction": (
            sum(row["watertight"] for row in geometry_records) / denominator if denominator else None
        ),
        "edge_manifold_fraction": (
            sum(row["edge_manifold"] for row in geometry_records) / denominator if denominator else None
        ),
        "load_errors": errors,
        "geometries": geometry_records,
        "self_intersection": {
            "state": "not_evaluable",
            "backend": None,
            "reason": "no recorded exact triangle-triangle backend with adjacent-face exclusion",
        },
        "manifold_definition": "edge-manifold proxy: every undirected edge has <=2 incident faces",
        "vertex_manifold_claimed": False,
    }


def portable_audit(package: Path, primary_name: str, temp_root: Path, token: str) -> dict[str, Any]:
    source = authorized(package)
    destination = authorized(temp_root / token, must_exist=False)
    try:
        shutil.copytree(source, destination, symlinks=True)
        copied_files, symlinks = package_files(destination)
        copied_primary = destination / primary_name
        dependency = declared_dependency_audit(copied_primary, destination)
        passed = not symlinks and dependency["parse_success"] and not dependency["errors"]
        return {
            "state": "pass" if passed else "fail",
            "fresh_copy_file_count": len(copied_files),
            "symlinks": symlinks,
            "declared_dependency_count": len(dependency["declared"]),
            "resolved_dependency_count": len(dependency["resolved_paths"]),
            "errors": dependency["errors"],
        }
    except Exception as exc:  # noqa: BLE001
        return {"state": "fail", "errors": [f"{type(exc).__name__}: {exc}"]}
    finally:
        if destination.exists():
            shutil.rmtree(authorized(destination))


def unavailable_record(item: dict[str, Any], reason: str) -> dict[str, Any]:
    not_evaluable = {"state": "not_evaluable", "reason": reason}
    return {
        "object_release_id": item["object_release_id"],
        "category": item["category"],
        "rel_path": item["rel_path"],
        "artifact_layer": "released",
        "availability": "unavailable",
        "unavailable_reason": reason,
        "geometry_evaluable": False,
        "portable_package": dict(not_evaluable),
        "deterministic_build": dict(not_evaluable),
        "semantic_complete": dict(not_evaluable),
        "kinematic_complete": dict(not_evaluable),
        "physical_complete": dict(not_evaluable),
    }


def audit_asset(
    item: dict[str, Any], cohort_root: Path, temp_root: Path, index: int
) -> dict[str, Any]:
    package = authorized(cohort_root / item["rel_path"], must_exist=False)
    primary = package / "generated.urdf"
    if not package.is_dir() or not primary.is_file():
        return unavailable_record(item, "released final package or generated.urdf missing")
    package = authorized(package)
    primary = authorized(primary)
    files, symlinks = package_files(package)
    dependency = declared_dependency_audit(primary, package)
    mesh_paths = [authorized(path) for path in dependency["mesh_paths"]]
    primary_hash = sha256_file(primary)
    release_hash_match = primary_hash == item["inline_urdf_sha256"]
    try:
        urdf_root = ET.parse(primary).getroot()
        parse_error = None
    except Exception as exc:  # noqa: BLE001
        urdf_root = None
        parse_error = f"{type(exc).__name__}: {exc}"
    geometry = mesh_geometry_audit(mesh_paths, package) if mesh_paths else {
        "mesh_file_count": 0,
        "readable_mesh_file_count": 0,
        "readable_geometry_count": 0,
        "watertight_geometry_count": 0,
        "edge_manifold_geometry_count": 0,
        "open_edges": 0,
        "nonmanifold_edges": 0,
        "degenerate_faces": 0,
        "watertight_fraction": None,
        "edge_manifold_fraction": None,
        "load_errors": [],
        "geometries": [],
        "self_intersection": {"state": "not_evaluable", "backend": None, "reason": "no mesh geometry"},
        "manifold_definition": "edge-manifold proxy: every undirected edge has <=2 incident faces",
        "vertex_manifold_claimed": False,
    }
    source_files = [path for path in files if path.suffix.lower() in SOURCE_SUFFIXES]
    mesh_bytes = sum(path.stat().st_size for path in set(mesh_paths))
    portable = portable_audit(package, "generated.urdf", temp_root, f"{index:03d}_{item['object_release_id']}")
    if urdf_root is None:
        failed = {"state": "fail", "reason": f"URDF parse failed: {parse_error}"}
        semantic = {"state": "not_evaluable", "reason": "URDF hierarchy unavailable"}
        kinematic = dict(failed)
        physical = dict(failed)
    else:
        semantic = {
            "state": "not_evaluable",
            "reason": "no output-independent required-part and role specification",
            "field_proxy": semantic_proxy(urdf_root),
        }
        kinematic = kinematic_gate(urdf_root)
        physical = physical_gate(urdf_root)
    return {
        "object_release_id": item["object_release_id"],
        "category": item["category"],
        "rel_path": item["rel_path"],
        "artifact_layer": "released",
        "artifact_provenance": "official viable.tar.gz extract frozen for Table 6",
        "local_generation_executed": False,
        "generated_or_recovered_artifacts": [],
        "availability": "available",
        "primary_artifact": "generated.urdf",
        "primary_artifact_name_is_provenance_neutral": True,
        "primary_urdf_sha256": primary_hash,
        "expected_inline_urdf_sha256": item["inline_urdf_sha256"],
        "release_hash_match": release_hash_match,
        "release_package_file_count": len(files),
        "release_package_symlinks": symlinks,
        "release_package_content_sha256": package_content_hash(package, files),
        "dependency_audit": {
            "parse_success": dependency["parse_success"],
            "declared_count": len(dependency["declared"]),
            "resolved_count": len(dependency["resolved_paths"]),
            "errors": dependency["errors"],
        },
        "geometry_scope": "unique mesh payloads declared by final generated.urdf",
        "geometry_evaluable": geometry["readable_geometry_count"] > 0,
        "geometry": geometry,
        "size_bytes": {
            "source": sum(path.stat().st_size for path in source_files),
            "source_file_count": len(source_files),
            "source_files": [path.relative_to(package).as_posix() for path in source_files],
            "urdf": primary.stat().st_size,
            "mesh": mesh_bytes,
            "mesh_file_count": len(set(mesh_paths)),
            "mesh_scope": "URDF-referenced payloads, canonical-path deduplicated within asset",
        },
        "portable_package": portable,
        "deterministic_build": {
            "state": "not_evaluable",
            "reason": "no second fresh build executed; release content hash is not a rebuild",
            "fresh_build_count": 0,
        },
        "semantic_complete": semantic,
        "kinematic_complete": kinematic,
        "physical_complete": physical,
    }


def mean(values: list[float | int]) -> float | None:
    return sum(values) / len(values) if values else None


def aggregate(records: list[dict[str, Any]], manifest_hash: str, protocol_hash: str) -> dict[str, Any]:
    available = [record for record in records if record["availability"] == "available"]
    geometry_assets = [record for record in available if record["geometry_evaluable"]]
    geometries = [geometry for record in geometry_assets for geometry in record["geometry"]["geometries"]]
    source_total = sum(record["size_bytes"]["source"] for record in available)
    urdf_total = sum(record["size_bytes"]["urdf"] for record in available)
    mesh_total = sum(record["size_bytes"]["mesh"] for record in available)
    semantic_proxy_counts = Counter(
        record["semantic_complete"].get("field_proxy", {}).get("state", "not_evaluable")
        for record in records
    )
    return {
        "protocol_id": "nano3d_table7_production_readiness_v1",
        "method": "LAM",
        "status": "COMPLETE",
        "protocol_sha256": protocol_hash,
        "manifest_sha256": manifest_hash,
        "cohort": {
            "requested_assets": len(records),
            "available_assets": len(available),
            "unavailable_assets": len(records) - len(available),
            "geometry_evaluable_assets": len(geometry_assets),
            "package_evaluable_assets": len(available),
            "selection": "Table 6 frozen viable release cohort; no Table 7 outcome filtering",
        },
        "artifact_layers": {
            "scored_layer": "released",
            "released_assets": len(available),
            "locally_generated_assets": 0,
            "recovered_assets": 0,
        },
        "geometry": {
            "loader": "trimesh scene load, process=false",
            "mesh_scope": "unique final-URDF referenced mesh payloads",
            "readable_mesh_files": sum(record["geometry"]["readable_mesh_file_count"] for record in geometry_assets),
            "readable_geometries": len(geometries),
            "watertight": {
                "geometry_level": rate(sum(row["watertight"] for row in geometries), len(geometries)),
                "per_asset_fraction_mean": mean([record["geometry"]["watertight_fraction"] for record in geometry_assets]),
            },
            "manifold": {
                "definition": "edge-manifold proxy: every undirected edge has <=2 incident faces",
                "tool": "numpy incidence count on trimesh process=false triangle faces",
                "vertex_manifold_claimed": False,
                "geometry_level": rate(sum(row["edge_manifold"] for row in geometries), len(geometries)),
                "per_asset_fraction_mean": mean([record["geometry"]["edge_manifold_fraction"] for record in geometry_assets]),
            },
            "open_edges": {
                "total": sum(row["open_edges"] for row in geometries),
                "per_asset_mean": mean([record["geometry"]["open_edges"] for record in geometry_assets]),
            },
            "nonmanifold_edges": {
                "total": sum(row["nonmanifold_edges"] for row in geometries),
                "per_asset_mean": mean([record["geometry"]["nonmanifold_edges"] for record in geometry_assets]),
            },
            "degenerate_faces": {
                "definition": "repeated vertex index OR area <=1e-12 in native squared units",
                "total": sum(row["degenerate_faces"] for row in geometries),
                "per_asset_mean": mean([record["geometry"]["degenerate_faces"] for record in geometry_assets]),
            },
            "load_failures": sum(len(record["geometry"]["load_errors"]) for record in geometry_assets),
            "self_intersection": {
                "state": "not_evaluable",
                "backend": None,
                "reason": "no recorded exact triangle-triangle backend with adjacent-face exclusion",
            },
        },
        "size_bytes": {
            "denominator": len(available),
            "source": {"total": source_total, "mean": source_total / len(available) if available else None},
            "urdf": {"total": urdf_total, "mean": urdf_total / len(available) if available else None},
            "mesh": {"total": mesh_total, "mean": mesh_total / len(available) if available else None},
        },
        "portable_package": state_counts(records, "portable_package"),
        "deterministic_build": state_counts(records, "deterministic_build"),
        "semantic_complete": {
            **state_counts(records, "semantic_complete"),
            "strict_blocker": "no output-independent required-part and role specification",
            "field_proxy": {
                "pass": semantic_proxy_counts["pass"],
                "fail": semantic_proxy_counts["fail"],
                "not_evaluable": semantic_proxy_counts["not_evaluable"],
                "requested": len(records),
            },
        },
        "kinematic_complete": state_counts(records, "kinematic_complete"),
        "physical_complete": state_counts(records, "physical_complete"),
        "release_integrity": {
            "primary_urdf_hash_match": rate(sum(record.get("release_hash_match", False) for record in available), len(available)),
            "dependency_clean_assets": rate(sum(not record["dependency_audit"]["errors"] for record in available), len(available)),
        },
        "blockers": [
            "Self-intersection: exact triangle-triangle backend with adjacent-face exclusion unavailable/not recorded.",
            "Deterministic build: no two fresh builds from frozen identical inputs were executed.",
            "Semantic completeness: no output-independent required-part and role specification.",
        ],
    }


def report_markdown(summary: dict[str, Any]) -> str:
    cohort = summary["cohort"]
    geometry = summary["geometry"]
    sizes = summary["size_bytes"]
    watertight = geometry["watertight"]
    manifold = geometry["manifold"]
    portable = summary["portable_package"]
    semantic = summary["semantic_complete"]
    kinematic = summary["kinematic_complete"]
    physical = summary["physical_complete"]
    fmt = lambda value: "N/A" if value is None else f"{value:.3f}"
    return f"""# Table 7 LAM production readiness

Status: **COMPLETE**

This is a representation-aware static audit of the official released LAM
`viable` package cohort frozen for Table 6. It is not a generation rerun. The
100 requested categories were fixed before Table 7 scoring and no asset was
selected or dropped based on geometry or completeness outcomes.

## Cohort and artifact provenance

- Requested/available: {cohort['requested_assets']}/{cohort['available_assets']}; geometry-evaluable: {cohort['geometry_evaluable_assets']}; package-evaluable: {cohort['package_evaluable_assets']}.
- Scored layer: official **released** archive extracts only. Locally generated=0; recovered=0.
- Final geometry scope: unique mesh payloads declared by each top-level `generated.urdf`; intermediate package meshes outside that final dependency graph are not scored.
- A filename such as `generated.urdf` does not imply local generation; release hashes are checked against the frozen Table 6 inline URDF hashes.

## Static results

- Readable final geometries: {geometry['readable_geometries']} from {geometry['readable_mesh_files']} mesh files; load failures={geometry['load_failures']}.
- Watertight: {watertight['geometry_level']['numerator']}/{watertight['geometry_level']['denominator']} geometries; per-asset mesh-fraction mean={fmt(watertight['per_asset_fraction_mean'])}.
- Edge-manifold proxy: {manifold['geometry_level']['numerator']}/{manifold['geometry_level']['denominator']} geometries; per-asset mesh-fraction mean={fmt(manifold['per_asset_fraction_mean'])}. Tool: NumPy undirected-edge incidence on trimesh `process=false` triangle faces. Vertex-manifold is explicitly not claimed.
- Open edges: {geometry['open_edges']['total']} total; {fmt(geometry['open_edges']['per_asset_mean'])}/asset mean. Nonmanifold edges (>2 incident faces): {geometry['nonmanifold_edges']['total']} total.
- Degenerate faces (repeated index or area <=1e-12): {geometry['degenerate_faces']['total']} total; {fmt(geometry['degenerate_faces']['per_asset_mean'])}/asset mean.
- Self-intersection: **N/A**; no exact triangle-triangle backend with adjacent-face exclusion was recorded.

## Package and completeness

- Source: {sizes['source']['total']/1024:.2f} KiB total, {sizes['source']['mean']/1024:.2f} KiB/asset. This is per-package executable source (`.js/.mjs/.cjs/.py/.ts/.tsx`).
- URDF: {sizes['urdf']['total']/1024:.2f} KiB total, {sizes['urdf']['mean']/1024:.2f} KiB/asset.
- Final referenced mesh: {sizes['mesh']['total']/1024:.2f} KiB total, {sizes['mesh']['mean']/1024:.2f} KiB/asset, canonical-path deduplicated within asset.
- Portable package: {portable['pass']}/{portable['requested']} pass after complete-package copy to a fresh workspace-internal directory and recursive relative dependency resolution; fail={portable['fail']}, N/A={portable['not_evaluable']}.
- Deterministic build: **N/A ({summary['deterministic_build']['not_evaluable']}/{summary['deterministic_build']['requested']})**. No second fresh build was run; release/content hashes are provenance evidence only.
- Semantic complete: **N/A ({semantic['not_evaluable']}/{semantic['requested']})** without an output-independent required-part/role specification. The separately labelled name+tree field proxy passes {semantic['field_proxy']['pass']}/{semantic['field_proxy']['requested']}.
- Kinematic complete: {kinematic['pass']}/{kinematic['requested']} pass, {kinematic['fail']} fail, {kinematic['not_evaluable']} N/A. Every joint must explicitly declare finite `origin xyz/rpy`; simulator identity defaults do not pass this gate.
- Physical complete: {physical['pass']}/{physical['requested']} pass, {physical['fail']} fail, {physical['not_evaluable']} N/A. Native collision, positive mass/inertia, joint damping/friction, and contact/friction metadata are all required; PyBullet defaults never count.

## Table 7 row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| LAM official release (frozen N={cohort['requested_assets']}) | {fmt(watertight['per_asset_fraction_mean'])} mean/asset; {watertight['geometry_level']['numerator']}/{watertight['geometry_level']['denominator']} geom | {fmt(manifold['per_asset_fraction_mean'])} edge-manifold mean/asset; {manifold['geometry_level']['numerator']}/{manifold['geometry_level']['denominator']} geom | {fmt(geometry['open_edges']['per_asset_mean'])}/asset; {geometry['open_edges']['total']} total | {fmt(geometry['degenerate_faces']['per_asset_mean'])}/asset; {geometry['degenerate_faces']['total']} total | N/A | {sizes['source']['mean']/1024:.2f}/asset; {sizes['source']['total']/1024:.2f} total | {sizes['urdf']['mean']/1024:.2f}/asset; {sizes['urdf']['total']/1024:.2f} total | {sizes['mesh']['mean']/1024:.2f}/asset; {sizes['mesh']['total']/1024:.2f} total | {portable['pass']}/{portable['requested']} | N/A (no two fresh builds) | N/A; field proxy {semantic['field_proxy']['pass']}/{semantic['field_proxy']['requested']} | {kinematic['pass']}/{kinematic['requested']} | {physical['pass']}/{physical['requested']} |

## Evidence

- `protocol_snapshot.json`: byte-for-byte frozen shared protocol.
- `manifest.json`: pre-score frozen identities and input evidence hashes.
- `asset_records.json`: per-asset, per-geometry, dependency, size, and field-gate evidence.
- `summary.json`: explicit denominators and aggregates.
- `self_check.json`: cohort/accounting/path/hash assertions.
- `hashes.sha256`: hashes for the runner and all required outputs.
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--cohort", type=Path, default=DEFAULT_COHORT)
    parser.add_argument("--cohort-root", type=Path, default=DEFAULT_COHORT_ROOT)
    parser.add_argument("--table6-preflight", type=Path, default=DEFAULT_TABLE6_PREFLIGHT)
    parser.add_argument("--table6-metadata", type=Path, default=DEFAULT_TABLE6_METADATA)
    parser.add_argument("--table6-functional", type=Path, default=DEFAULT_TABLE6_FUNCTIONAL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    if Path.cwd().resolve(strict=True) != WORKSPACE.resolve(strict=True):
        raise RuntimeError(f"run from authorized workspace root {WORKSPACE}")
    authorized(REPO)
    protocol_path = authorized(args.protocol)
    cohort_path = authorized(args.cohort)
    cohort_root = authorized(args.cohort_root)
    table6_preflight_path = authorized(args.table6_preflight)
    table6_metadata_path = authorized(args.table6_metadata)
    table6_functional_path = authorized(args.table6_functional)
    output = authorized(args.output, must_exist=False)
    authorized(output.parent).mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=True)
    output = authorized(output)

    protocol = read_json(protocol_path)
    if protocol.get("protocol_id") != "nano3d_table7_production_readiness_v1":
        raise RuntimeError("unexpected shared Table 7 protocol")
    cohort = read_json(cohort_path)
    if cohort.get("protocol_id") != "nano3d_table6_lam_cohort_v1" or cohort.get("size") != 100:
        raise RuntimeError("expected frozen Table 6 LAM cohort of size 100")
    items = cohort.get("items", [])
    if len(items) != 100:
        raise RuntimeError(f"frozen cohort item mismatch: {len(items)}")
    preflight = read_json(table6_preflight_path)
    preflight_by_id = {row["object_release_id"]: row for row in preflight["records"]}
    for item in items:
        previous = preflight_by_id.get(item["object_release_id"])
        if previous is None or previous["rel_path"] != item["rel_path"]:
            raise RuntimeError(f"Table 6 preflight mismatch: {item['object_release_id']}")
        if previous["generated_urdf_sha256"] != item["inline_urdf_sha256"]:
            raise RuntimeError(f"Table 6 URDF hash mismatch: {item['object_release_id']}")

    snapshot_path = output / "protocol_snapshot.json"
    shutil.copyfile(protocol_path, authorized(snapshot_path, must_exist=False))
    if sha256_file(snapshot_path) != sha256_file(protocol_path):
        raise RuntimeError("protocol snapshot copy mismatch")
    evidence_hashes = {
        "protocol": sha256_file(protocol_path),
        "table6_frozen_cohort": sha256_file(cohort_path),
        "table6_package_preflight": sha256_file(table6_preflight_path),
        "table6_metadata_summary": sha256_file(table6_metadata_path),
        "table6_functional_summary": sha256_file(table6_functional_path),
    }
    manifest = {
        "protocol_id": protocol["protocol_id"],
        "method": "LAM",
        "manifest_frozen_before_scoring": True,
        "requested_assets": len(items),
        "selection_policy": cohort["selection_policy"],
        "selection_salt": cohort["salt"],
        "source_cohort_protocol_id": cohort["protocol_id"],
        "artifact_layers": {
            "scored": "released",
            "released_source": "official viable.tar.gz cohort extract",
            "locally_generated": [],
            "recovered": [],
        },
        "adapter": {
            "primary_artifact": "top-level generated.urdf",
            "mesh_scope": "unique mesh payloads declared by primary URDF",
            "mesh_loader": "trimesh scene load with process=false",
        },
        "input_evidence_sha256": evidence_hashes,
        "assets": [
            {
                "index": index,
                "category": item["category"],
                "object_release_id": item["object_release_id"],
                "rel_path": item["rel_path"],
                "selection_hash": item["selection_hash"],
                "category_hash": item["category_hash"],
                "expected_inline_urdf_sha256": item["inline_urdf_sha256"],
                "artifact_layer": "released",
            }
            for index, item in enumerate(items)
        ],
    }
    manifest_path = output / "manifest.json"
    write_json(manifest_path, manifest)
    manifest_hash = sha256_file(manifest_path)

    temp_root = output / "_portable_tmp"
    if temp_root.exists():
        shutil.rmtree(authorized(temp_root))
    temp_root.mkdir(parents=True)
    temp_root = authorized(temp_root)
    try:
        records = [
            audit_asset(item, cohort_root, temp_root, index)
            for index, item in enumerate(items)
        ]
    finally:
        if temp_root.exists():
            shutil.rmtree(authorized(temp_root))

    records_path = output / "asset_records.json"
    write_json(records_path, records)
    summary = aggregate(records, manifest_hash, evidence_hashes["protocol"])
    summary_path = output / "summary.json"
    write_json(summary_path, summary)
    report_path = output / "report.md"
    write_text(report_path, report_markdown(summary))

    required_names = protocol["required_outputs"]
    self_check_path = output / "self_check.json"
    write_json(self_check_path, {"status": "PROVISIONAL"})
    identities = [record["object_release_id"] for record in records]
    state_keys = [
        "portable_package", "deterministic_build", "semantic_complete",
        "kinematic_complete", "physical_complete",
    ]
    accounting = {
        key: sum(state_counts(records, key)[state] for state in ("pass", "fail", "not_evaluable")) == len(records)
        for key in state_keys
    }
    evidence_files = [output / name for name in required_names]
    evidence_checks = []
    for path in evidence_files:
        resolved = authorized(path, must_exist=False)
        evidence_checks.append(
            {
                "path": resolved.relative_to(authorized(WORKSPACE)).as_posix(),
                "inside_workspace": True,
                "exists": resolved.exists(),
            }
        )
    checks = {
        "manifest_identities_unique": len(identities) == len(set(identities)) == len(items),
        "requested_matches_manifest_and_records": manifest["requested_assets"] == len(records) == 100,
        "state_accounting": accounting,
        "all_rates_have_explicit_numerators_denominators": all(
            "numerator" in value and "denominator" in value
            for value in (
                summary["geometry"]["watertight"]["geometry_level"],
                summary["geometry"]["manifold"]["geometry_level"],
                summary["release_integrity"]["primary_urdf_hash_match"],
                summary["release_integrity"]["dependency_clean_assets"],
            )
        ),
        "all_evidence_files_inside_workspace_and_exist": all(
            entry["inside_workspace"] and entry["exists"] for entry in evidence_checks
        ),
        "protocol_snapshot_exact": sha256_file(snapshot_path) == evidence_hashes["protocol"],
        "manifest_hash_exact": sha256_file(manifest_path) == manifest_hash,
        "frozen_release_urdf_hashes_match": all(
            record.get("release_hash_match", False)
            for record in records if record["availability"] == "available"
        ),
        "no_table7_outcome_reselection": [asset["object_release_id"] for asset in manifest["assets"]] == identities,
        "released_generated_recovered_layers_separate": summary["artifact_layers"] == {
            "scored_layer": "released",
            "released_assets": summary["cohort"]["available_assets"],
            "locally_generated_assets": 0,
            "recovered_assets": 0,
        },
    }
    flattened = [value for value in checks.values() if isinstance(value, bool)] + list(accounting.values())
    self_check = {
        "protocol_id": protocol["protocol_id"],
        "status": "PASS" if all(flattened) else "FAIL",
        "checks": checks,
        "evidence_files": evidence_checks,
        "sha256": {
            "protocol_snapshot.json": sha256_file(snapshot_path),
            "manifest.json": manifest_hash,
            "asset_records.json": sha256_file(records_path),
            "summary.json": sha256_file(summary_path),
            "report.md": sha256_file(report_path),
        },
    }
    write_json(self_check_path, self_check)
    if self_check["status"] != "PASS":
        raise RuntimeError("self-check failed")

    hashes_path = output / "hashes.sha256"
    hash_targets = [
        snapshot_path, manifest_path, records_path, summary_path,
        self_check_path, report_path, authorized(Path(__file__)),
    ]
    hash_lines = [
        f"{sha256_file(path)}  {path.relative_to(authorized(WORKSPACE)).as_posix()}"
        for path in hash_targets
    ]
    write_text(hashes_path, "\n".join(hash_lines) + "\n")
    print(json.dumps({
        "status": summary["status"],
        "requested_assets": summary["cohort"]["requested_assets"],
        "available_assets": summary["cohort"]["available_assets"],
        "readable_geometries": summary["geometry"]["readable_geometries"],
        "portable_pass": summary["portable_package"]["pass"],
        "kinematic_pass": summary["kinematic_complete"]["pass"],
        "physical_pass": summary["physical_complete"]["pass"],
        "self_check": self_check["status"],
        "output": relative_workspace(output),
    }, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
