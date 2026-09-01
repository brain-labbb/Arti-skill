#!/usr/bin/env python3
"""Fail-closed static atoms for the LAM supplementary URDF evaluation.

This module deliberately has no command-line entry point and performs no writes.
It is imported by the formal runner to inspect one immutable LAM release package.
The returned record contains only JSON-serializable values.

Privacy boundary: files named ``generation_config.*`` are never parsed and their
contents are never returned.  Only their existence, relative path, byte size and
SHA-256 digest may appear in the record.
"""

from __future__ import annotations

from collections import deque
from io import BytesIO
import hashlib
import json
import math
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import shlex
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


SCHEMA_VERSION = "lam-supplementary-static/v1.2"
DEFAULT_URDF_RELATIVE_PATH = "generated.urdf"
BOUNDED_JOINT_TYPES = frozenset({"revolute", "prismatic"})
MOVABLE_JOINT_TYPES = frozenset({"revolute", "continuous", "prismatic", "planar", "floating"})
SUPPORTED_MESH_SUFFIXES = frozenset({".obj", ".stl", ".ply", ".off", ".dae", ".glb", ".gltf"})
GENERATION_CONFIG_RE = re.compile(r"^generation_config(?:\..+)?$", re.IGNORECASE)
RECEIPT_NAME_RE = re.compile(r"(?:^|[-_])(?:mechanical[-_])?receipt(?:[-_]|\.|$)", re.IGNORECASE)
ALLOWANCE_NAME_RE = re.compile(r"allow(?:ance|list|ed)|exclu(?:de|sion)", re.IGNORECASE)
REBUILD_RECIPE_NAMES = frozenset({
    "build_recipe.json",
    "build-recipe.json",
    "rebuild_recipe.json",
    "rebuild-recipe.json",
    "deterministic_rebuild.json",
})
MAX_EVIDENCE_JSON_BYTES = 8 * 1024 * 1024
INERTIA_FIELDS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")


def sha256_file(path: Path) -> str:
    """Hash a file without retaining or returning its bytes."""

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def local_tag(node: ET.Element) -> str:
    return node.tag.rsplit("}", 1)[-1]


def children(node: ET.Element, tag: str) -> list[ET.Element]:
    return [child for child in node if local_tag(child) == tag]


def descendants(node: ET.Element, tag: str) -> Iterable[ET.Element]:
    return (child for child in node.iter() if local_tag(child) == tag)


def finite_scalar(raw: Any) -> float | None:
    if raw is None or isinstance(raw, bool):
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def finite_vector(raw: str | None, length: int) -> tuple[float, ...] | None:
    if raw is None:
        return None
    try:
        values = tuple(float(item) for item in raw.split())
    except ValueError:
        return None
    if len(values) != length or not all(math.isfinite(item) for item in values):
        return None
    return values


def safe_package_relative_path(raw: str, *, field: str) -> PurePosixPath:
    """Normalize package-relative POSIX path references.

    Frozen rule v1.2 (Table 2 supplementary unified resolver): references are
    normalized with POSIX dot-segment semantics — ``.`` segments are dropped
    and interior ``..`` segments cancel the preceding segment, matching how
    standard URDF consumers resolve relative ``filename`` references (for
    example ``./../partseg/<id>/objs/part.obj`` declared by a URDF inside a
    ``urdf/`` subdirectory).  Leading ``..`` segments are preserved in the
    normalized form; whether such a reference escapes the package is decided
    by the strict containment check at resolution time.  Absolute paths,
    backslashes, NUL bytes, empty segments (double/leading/trailing slashes)
    and references that normalize to nothing remain rejected here.
    """

    if not isinstance(raw, str) or not raw or "\\" in raw or "\x00" in raw:
        raise ValueError(f"invalid_{field}: {raw!r}")
    if raw.startswith("/") or PureWindowsPath(raw).is_absolute():
        raise ValueError(f"unsafe_{field}: {raw!r}")
    stack: list[str] = []
    for segment in raw.split("/"):
        if segment == "":
            raise ValueError(f"noncanonical_{field}: {raw!r}")
        if segment == ".":
            continue
        if segment == "..":
            if stack and stack[-1] != "..":
                stack.pop()
            else:
                stack.append("..")
        else:
            stack.append(segment)
    if not stack:
        raise ValueError(f"invalid_{field}: {raw!r}")
    return PurePosixPath(*stack)


def _is_generation_config_name(path: Path | PurePosixPath) -> bool:
    return bool(GENERATION_CONFIG_RE.fullmatch(path.name))


def _resolve_package_file(
    package: Path,
    declaring_file: Path,
    raw: str,
    *,
    field: str,
    permit_generation_config_hash_only: bool = False,
) -> tuple[Path | None, str | None]:
    try:
        relative = safe_package_relative_path(raw.strip(), field=field)
    except (AttributeError, ValueError) as exc:
        return None, str(exc)
    if _is_generation_config_name(relative) and not permit_generation_config_hash_only:
        return None, f"generation_config_not_readable_as_resource: {relative.as_posix()}"
    candidate = declaring_file.parent.joinpath(*relative.parts)
    if candidate.is_symlink():
        return None, f"symlink_{field}: {relative.as_posix()}"
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(package)
    except FileNotFoundError:
        return None, f"missing_{field}: {relative.as_posix()}"
    except (OSError, ValueError):
        return None, f"escaping_{field}: {relative.as_posix()}"
    if not resolved.is_file():
        return None, f"not_file_{field}: {relative.as_posix()}"
    try:
        if resolved.stat().st_size <= 0:
            return None, f"empty_{field}: {relative.as_posix()}"
    except OSError as exc:
        return None, f"unreadable_{field}: {type(exc).__name__}: {exc}"
    return resolved, None


def _primary_urdf(package: Path, relative_raw: str) -> Path:
    relative = safe_package_relative_path(relative_raw, field="urdf_path")
    if _is_generation_config_name(relative):
        raise ValueError("generation_config_cannot_be_primary_urdf")
    candidate = package.joinpath(*relative.parts)
    if candidate.is_symlink() or not candidate.is_file():
        raise FileNotFoundError(candidate)
    resolved = candidate.resolve(strict=True)
    resolved.relative_to(package)
    return resolved


def _mesh_from_obj_bytes(payload: bytes) -> tuple[bool, str | None]:
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        return False, f"obj_utf8_decode_failed: {exc}"
    vertex_count = 0
    face_indices: list[int] = []
    face_count = 0
    try:
        for line in text.splitlines():
            tokens = shlex.split(line, comments=True)
            if not tokens:
                continue
            if tokens[0] == "v":
                if len(tokens) < 4:
                    return False, "obj_vertex_too_short"
                xyz = tuple(float(value) for value in tokens[1:4])
                if not all(math.isfinite(value) for value in xyz):
                    return False, "obj_vertex_nonfinite"
                vertex_count += 1
            elif tokens[0] == "f":
                if len(tokens) < 4:
                    return False, "obj_face_too_short"
                for token in tokens[1:]:
                    index_text = token.split("/", 1)[0]
                    index = int(index_text)
                    if index == 0:
                        return False, "obj_face_zero_index"
                    face_indices.append(index)
                face_count += len(tokens) - 3
    except (ValueError, IndexError) as exc:
        return False, f"obj_parse_failed: {type(exc).__name__}: {exc}"
    if vertex_count < 3 or face_count <= 0:
        return False, "obj_has_no_surface"
    if any(index > vertex_count or index < -vertex_count for index in face_indices):
        return False, "obj_face_index_out_of_bounds"
    return True, None


def _mesh_bytes_loadable(path: Path) -> tuple[bool, str | None]:
    suffix = path.suffix.lower()
    if suffix not in SUPPORTED_MESH_SUFFIXES:
        return False, f"unsupported_mesh_format: {suffix or '<none>'}"
    try:
        payload = path.read_bytes()
    except OSError as exc:
        return False, f"mesh_read_failed: {type(exc).__name__}: {exc}"
    if not payload:
        return False, "mesh_empty"
    if suffix == ".obj":
        return _mesh_from_obj_bytes(payload)
    try:
        import numpy as np  # Imported lazily so primitive-only audits stay stdlib-only.
        import trimesh
    except Exception as exc:  # noqa: BLE001
        return False, f"mesh_loader_unavailable: {type(exc).__name__}: {exc}"
    try:
        loaded = trimesh.load(
            file_obj=BytesIO(payload),
            file_type=suffix[1:],
            force="scene",
            process=False,
        )
        geometries = list(getattr(loaded, "geometry", {}).values())
        if not geometries:
            return False, "mesh_has_no_geometry"
        for geometry in geometries:
            vertices = np.asarray(geometry.vertices)
            faces = np.asarray(geometry.faces)
            if (
                vertices.ndim != 2
                or vertices.shape[0] < 3
                or faces.ndim != 2
                or faces.shape[0] < 1
                or not np.all(np.isfinite(vertices))
            ):
                return False, "mesh_geometry_empty_or_nonfinite"
    except Exception as exc:  # noqa: BLE001
        return False, f"mesh_load_failed: {type(exc).__name__}: {exc}"
    return True, None


def _collision_geometry_loadable(
    geometry: ET.Element,
    package: Path,
    urdf_path: Path,
) -> tuple[bool, str | None, str | None]:
    shapes = list(geometry)
    if len(shapes) != 1:
        return False, "geometry_shape_cardinality", None
    shape = shapes[0]
    kind = local_tag(shape)
    if kind == "box":
        size = finite_vector(shape.attrib.get("size"), 3)
        return (
            (True, None, kind)
            if size is not None and all(value > 0 for value in size)
            else (False, "invalid_box_size", kind)
        )
    if kind == "sphere":
        radius = finite_scalar(shape.attrib.get("radius"))
        return (
            (True, None, kind)
            if radius is not None and radius > 0
            else (False, "invalid_sphere_radius", kind)
        )
    if kind == "cylinder":
        radius = finite_scalar(shape.attrib.get("radius"))
        length = finite_scalar(shape.attrib.get("length"))
        return (
            (True, None, kind)
            if radius is not None and radius > 0 and length is not None and length > 0
            else (False, "invalid_cylinder_dimensions", kind)
        )
    if kind != "mesh":
        return False, f"unsupported_collision_geometry: {kind}", kind
    scale = finite_vector(shape.attrib.get("scale", "1 1 1"), 3)
    if scale is None or any(value <= 0 for value in scale):
        return False, "mesh_scale_not_positive_finite", kind
    mesh_path, issue = _resolve_package_file(
        package,
        urdf_path,
        shape.attrib.get("filename", ""),
        field="mesh_resource",
    )
    if issue is not None:
        return False, issue, kind
    assert mesh_path is not None
    loaded, load_issue = _mesh_bytes_loadable(mesh_path)
    return loaded, load_issue, kind


def _visual_collision_atoms(
    root: ET.Element,
    package: Path,
    urdf_path: Path,
    *,
    link_extraction_complete: bool,
) -> dict[str, Any]:
    visual_links = 0
    covered_links = 0
    collision_elements = 0
    loadable_elements = 0
    issues: list[str] = []
    link_records: list[dict[str, Any]] = []
    for index, link in enumerate(children(root, "link")):
        name = link.attrib.get("name", "").strip() or f"index_{index}"
        visual_nodes = children(link, "visual")
        visual_geometry_count = sum(len(children(node, "geometry")) for node in visual_nodes)
        if visual_geometry_count <= 0:
            continue
        visual_links += 1
        link_collision_count = 0
        link_loadable_count = 0
        link_issues: list[str] = []
        shape_counts: dict[str, int] = {}
        for collision_index, collision in enumerate(children(link, "collision")):
            geometry_nodes = children(collision, "geometry")
            if len(geometry_nodes) != 1:
                link_issues.append(f"collision_{collision_index}: geometry_cardinality_{len(geometry_nodes)}")
                continue
            link_collision_count += 1
            collision_elements += 1
            loadable, issue, kind = _collision_geometry_loadable(
                geometry_nodes[0], package, urdf_path
            )
            if kind is not None:
                shape_counts[kind] = shape_counts.get(kind, 0) + 1
            if loadable:
                link_loadable_count += 1
                loadable_elements += 1
            else:
                link_issues.append(f"collision_{collision_index}: {issue}")
        link_pass = link_loadable_count > 0
        if link_pass:
            covered_links += 1
        else:
            link_issues.append("no_loadable_collision_geometry")
        issues.extend(f"{name}: {issue}" for issue in link_issues)
        link_records.append({
            "link_name": name,
            "visual_geometry_count": visual_geometry_count,
            "collision_element_count": link_collision_count,
            "loadable_collision_element_count": link_loadable_count,
            "collision_shape_counts": dict(sorted(shape_counts.items())),
            "covered": link_pass,
            "issues": link_issues,
        })
    asset_pass = bool(
        link_extraction_complete and visual_links > 0 and covered_links == visual_links
    )
    if visual_links == 0:
        issues.append("no_declared_visual_bearing_links")
    if not link_extraction_complete:
        issues.append("link_extraction_incomplete")
    return {
        "status": "COMPLETE" if link_extraction_complete else "PARTIAL",
        "asset_intended": 1,
        "asset_passed": int(asset_pass),
        "asset_pass": asset_pass,
        "visual_bearing_links_declared": visual_links,
        "covered_visual_bearing_links": covered_links,
        "link_extraction_complete": link_extraction_complete,
        "collision_elements_declared_on_visual_links": collision_elements,
        "loadable_collision_elements_on_visual_links": loadable_elements,
        "link_records": link_records,
        "issues": issues,
    }


def _joint_atoms(
    root: ET.Element,
    *,
    expected_movable_joints: int | None,
) -> tuple[dict[str, Any], dict[str, Any], list[str]]:
    joint_records: list[dict[str, Any]] = []
    portability_passed = 0
    dynamics_covered = 0
    extraction_issues: list[str] = []
    seen_names: set[str] = set()
    for index, joint in enumerate(children(root, "joint")):
        kind = joint.attrib.get("type", "").strip().lower()
        if kind == "fixed":
            continue
        name = joint.attrib.get("name", "").strip() or f"index_{index}"
        if name in seen_names:
            extraction_issues.append(f"duplicate_movable_joint_name: {name}")
        seen_names.add(name)
        portability_issues: list[str] = []
        limit_nodes = children(joint, "limit")
        limit = limit_nodes[0] if len(limit_nodes) == 1 else None
        if kind in BOUNDED_JOINT_TYPES:
            if limit is None:
                portability_issues.append(f"limit_cardinality_{len(limit_nodes)}")
            else:
                lower = finite_scalar(limit.attrib.get("lower"))
                upper = finite_scalar(limit.attrib.get("upper"))
                if lower is None or upper is None or not lower < upper:
                    portability_issues.append("lower_upper_invalid")
        elif kind == "continuous":
            if limit is None:
                portability_issues.append(f"limit_cardinality_{len(limit_nodes)}")
        else:
            portability_issues.append(f"unsupported_joint_mapping: {kind or '<missing>'}")
        effort = finite_scalar(limit.attrib.get("effort")) if limit is not None else None
        velocity = finite_scalar(limit.attrib.get("velocity")) if limit is not None else None
        if effort is None or effort < 0:
            portability_issues.append("effort_missing_or_invalid")
        if velocity is None or velocity <= 0:
            portability_issues.append("velocity_missing_or_invalid")
        portability_pass = not portability_issues
        portability_passed += int(portability_pass)

        dynamics_nodes = children(joint, "dynamics")
        dynamics = dynamics_nodes[0] if len(dynamics_nodes) == 1 else None
        damping = finite_scalar(dynamics.attrib.get("damping")) if dynamics is not None else None
        friction = finite_scalar(dynamics.attrib.get("friction")) if dynamics is not None else None
        dynamics_issues: list[str] = []
        if dynamics is None:
            dynamics_issues.append(f"dynamics_cardinality_{len(dynamics_nodes)}")
        if damping is None or damping < 0:
            dynamics_issues.append("damping_missing_or_invalid")
        if friction is None or friction < 0:
            dynamics_issues.append("friction_missing_or_invalid")
        dynamics_pass = not dynamics_issues
        dynamics_covered += int(dynamics_pass)
        joint_records.append({
            "joint_name": name,
            "joint_type": kind,
            "limit_portability_pass": portability_pass,
            "limit_portability_issues": portability_issues,
            "dynamics_covered": dynamics_pass,
            "dynamics_issues": dynamics_issues,
        })

    extracted = len(joint_records)
    if expected_movable_joints is not None:
        if isinstance(expected_movable_joints, bool) or expected_movable_joints < 0:
            raise ValueError("expected_movable_joints must be a non-negative integer or None")
        if expected_movable_joints != extracted:
            extraction_issues.append(
                f"expected_movable_joint_mismatch: expected={expected_movable_joints}, extracted={extracted}"
            )
    intended = expected_movable_joints if expected_movable_joints is not None else extracted
    extraction_complete = intended == extracted and not extraction_issues
    status = "COMPLETE" if extraction_complete else "PARTIAL"
    portability = {
        "status": status,
        "joints_intended": intended,
        "joints_extracted": extracted,
        "joints_passed": portability_passed,
        "extraction_complete": extraction_complete,
        "joint_records": joint_records,
        "issues": [
            *extraction_issues,
            *(f"{record['joint_name']}: {issue}" for record in joint_records for issue in record["limit_portability_issues"]),
        ],
    }
    dynamics = {
        "status": status,
        "joints_intended": intended,
        "joints_extracted": extracted,
        "joints_covered": dynamics_covered,
        "extraction_complete": extraction_complete,
        "joint_records": [
            {
                "joint_name": record["joint_name"],
                "joint_type": record["joint_type"],
                "covered": record["dynamics_covered"],
                "issues": record["dynamics_issues"],
            }
            for record in joint_records
        ],
        "issues": [
            *extraction_issues,
            *(f"{record['joint_name']}: {issue}" for record in joint_records for issue in record["dynamics_issues"]),
        ],
    }
    return portability, dynamics, extraction_issues


def normalize_placeholder_registry(
    registry: Sequence[Mapping[str, Any]] | None,
) -> list[dict[str, Any]]:
    """Validate a pre-registered placeholder template registry.

    Each entry specifies ``registry_id`` and at least one of ``mass`` or an
    ``inertia`` mapping.  Every supplied field must match for the entry to hit.
    """

    if registry is None:
        return []
    normalized: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    for index, raw in enumerate(registry):
        if not isinstance(raw, Mapping):
            raise ValueError(f"placeholder_registry[{index}] is not a mapping")
        registry_id = raw.get("registry_id")
        if not isinstance(registry_id, str) or not registry_id.strip():
            raise ValueError(f"placeholder_registry[{index}] has no registry_id")
        registry_id = registry_id.strip()
        if registry_id in seen_ids:
            raise ValueError(f"duplicate placeholder registry_id: {registry_id}")
        seen_ids.add(registry_id)
        mass = finite_scalar(raw.get("mass")) if "mass" in raw else None
        inertia_raw = raw.get("inertia")
        inertia: dict[str, float] | None = None
        if inertia_raw is not None:
            if not isinstance(inertia_raw, Mapping):
                raise ValueError(f"placeholder_registry[{index}].inertia is not a mapping")
            inertia = {}
            for field in INERTIA_FIELDS:
                value = finite_scalar(inertia_raw.get(field))
                if value is None:
                    raise ValueError(
                        f"placeholder_registry[{index}].inertia.{field} is missing or non-finite"
                    )
                inertia[field] = value
        if mass is None and inertia is None:
            raise ValueError(f"placeholder_registry[{index}] has no match fields")
        absolute_tolerance = finite_scalar(raw.get("absolute_tolerance", 1e-12))
        relative_tolerance = finite_scalar(raw.get("relative_tolerance", 1e-9))
        if absolute_tolerance is None or absolute_tolerance < 0:
            raise ValueError(f"placeholder_registry[{index}] invalid absolute_tolerance")
        if relative_tolerance is None or relative_tolerance < 0:
            raise ValueError(f"placeholder_registry[{index}] invalid relative_tolerance")
        normalized.append({
            "registry_id": registry_id,
            "mass": mass,
            "inertia": inertia,
            "absolute_tolerance": absolute_tolerance,
            "relative_tolerance": relative_tolerance,
        })
    return normalized


def _matches_placeholder(
    mass: float,
    inertia: Mapping[str, float],
    template: Mapping[str, Any],
) -> bool:
    atol = float(template["absolute_tolerance"])
    rtol = float(template["relative_tolerance"])
    if template["mass"] is not None and not math.isclose(
        mass, float(template["mass"]), rel_tol=rtol, abs_tol=atol
    ):
        return False
    if template["inertia"] is not None:
        for field in INERTIA_FIELDS:
            if not math.isclose(
                inertia[field],
                float(template["inertia"][field]),
                rel_tol=rtol,
                abs_tol=atol,
            ):
                return False
    return True


def _placeholder_mass_atoms(
    root: ET.Element,
    registry: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    declared_links = children(root, "link")
    complete_records: list[dict[str, Any]] = []
    incomplete_links: list[str] = []
    for index, link in enumerate(declared_links):
        name = link.attrib.get("name", "").strip() or f"index_{index}"
        inertial_nodes = children(link, "inertial")
        if len(inertial_nodes) != 1:
            incomplete_links.append(name)
            continue
        inertial = inertial_nodes[0]
        mass_nodes = children(inertial, "mass")
        tensor_nodes = children(inertial, "inertia")
        if len(mass_nodes) != 1 or len(tensor_nodes) != 1:
            incomplete_links.append(name)
            continue
        mass = finite_scalar(mass_nodes[0].attrib.get("value"))
        tensor = {field: finite_scalar(tensor_nodes[0].attrib.get(field)) for field in INERTIA_FIELDS}
        if mass is None or any(value is None for value in tensor.values()):
            incomplete_links.append(name)
            continue
        finite_tensor = {field: float(tensor[field]) for field in INERTIA_FIELDS}
        matches = [
            template["registry_id"]
            for template in registry
            if _matches_placeholder(float(mass), finite_tensor, template)
        ]
        complete_records.append({
            "link_name": name,
            "mass": float(mass),
            "inertia": finite_tensor,
            "placeholder_match_ids": matches,
            "placeholder_match": bool(matches) if registry else None,
        })
    complete_count = len(complete_records)
    if not registry:
        status = "N/E"
        classified = 0
        unclassified = complete_count
        placeholder_links: int | None = None
        incidence_numerator: int | None = None
        incidence_denominator = 0
        issue = "placeholder_registry_empty"
    elif complete_count == 0:
        status = "N/E"
        classified = 0
        unclassified = 0
        placeholder_links = 0
        incidence_numerator = None
        incidence_denominator = 0
        issue = "no_complete_inertial_links"
    else:
        status = "COMPLETE"
        classified = complete_count
        unclassified = 0
        placeholder_links = sum(bool(record["placeholder_match_ids"]) for record in complete_records)
        incidence_numerator = placeholder_links
        incidence_denominator = complete_count
        issue = None
    return {
        "status": status,
        "dynamic_link_policy": "all_declared_links",
        "dynamic_links": len(declared_links),
        "complete_inertial_links": complete_count,
        "complete_inertial_coverage_numerator": complete_count,
        "complete_inertial_coverage_denominator": len(declared_links),
        "classified_complete_inertial_links": classified,
        "unclassified_complete_inertial_links": unclassified,
        "placeholder_links": placeholder_links,
        "incidence_numerator": incidence_numerator,
        "incidence_denominator": incidence_denominator,
        "registry_ids": [template["registry_id"] for template in registry],
        "link_records": complete_records,
        "incomplete_inertial_links": incomplete_links,
        "issues": [issue] if issue else [],
    }


def _nested_resource_specs(path: Path) -> tuple[list[tuple[str, str]], str | None]:
    suffix = path.suffix.lower()
    try:
        if suffix == ".obj":
            specs: list[tuple[str, str]] = []
            for line in path.read_text(encoding="utf-8").splitlines():
                tokens = shlex.split(line, comments=True)
                if tokens and tokens[0].lower() == "mtllib":
                    specs.extend(("obj_mtl", token) for token in tokens[1:])
            return specs, None
        if suffix == ".mtl":
            specs = []
            for line in path.read_text(encoding="utf-8").splitlines():
                tokens = shlex.split(line, comments=True)
                if len(tokens) >= 2 and tokens[0].lower() in {
                    "map_ka", "map_kd", "map_ks", "map_ke", "map_d", "bump", "map_bump", "disp", "decal", "norm",
                }:
                    specs.append(("mtl_resource", tokens[-1]))
            return specs, None
        if suffix == ".gltf":
            payload = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                return [], "gltf_root_not_object"
            specs = []
            for section in ("buffers", "images"):
                entries = payload.get(section, [])
                if not isinstance(entries, list):
                    return [], f"gltf_{section}_not_list"
                for entry in entries:
                    uri = entry.get("uri") if isinstance(entry, dict) else None
                    if isinstance(uri, str) and not uri.startswith("data:"):
                        specs.append((f"gltf_{section[:-1]}", uri))
            return specs, None
        if suffix == ".dae":
            dae_root = ET.parse(path).getroot()
            specs = []
            for image in descendants(dae_root, "image"):
                for node in image.iter():
                    if local_tag(node) == "init_from" and node.text and node.text.strip():
                        raw = node.text.strip()
                        if not raw.startswith("#"):
                            specs.append(("dae_image", raw))
            return specs, None
    except Exception as exc:  # noqa: BLE001
        return [], f"nested_resource_parse_failed: {type(exc).__name__}: {exc}"
    return [], None


def _resource_closure(root: ET.Element, package: Path, urdf_path: Path) -> dict[str, Any]:
    queue: deque[tuple[str, str, Path]] = deque()
    for mesh in descendants(root, "mesh"):
        queue.append(("urdf_mesh", mesh.attrib.get("filename", ""), urdf_path))
    for texture in descendants(root, "texture"):
        queue.append(("urdf_texture", texture.attrib.get("filename", ""), urdf_path))
    records: dict[str, dict[str, Any]] = {
        urdf_path.relative_to(package).as_posix(): {
            "path": urdf_path.relative_to(package).as_posix(),
            "sha256": sha256_file(urdf_path),
        }
    }
    issues: list[str] = []
    visited_edges: set[tuple[str, str, str]] = set()
    expanded: set[Path] = set()
    while queue:
        kind, raw, declaring = queue.popleft()
        edge = (kind, raw, declaring.relative_to(package).as_posix())
        if edge in visited_edges:
            continue
        visited_edges.add(edge)
        resolved, issue = _resolve_package_file(
            package,
            declaring,
            raw,
            field=kind,
        )
        if issue is not None:
            issues.append(f"{edge[2]}:{kind}: {issue}")
            continue
        assert resolved is not None
        relative = resolved.relative_to(package).as_posix()
        if relative not in records:
            records[relative] = {"path": relative, "sha256": sha256_file(resolved)}
        if resolved in expanded:
            continue
        expanded.add(resolved)
        nested, nested_issue = _nested_resource_specs(resolved)
        if nested_issue:
            issues.append(f"{relative}: {nested_issue}")
            continue
        queue.extend((nested_kind, nested_raw, resolved) for nested_kind, nested_raw in nested)
    ordered = [records[key] for key in sorted(records)]
    complete = not issues
    return {
        "status": "COMPLETE" if complete else "PARTIAL",
        "complete": complete,
        "file_count": len(ordered),
        "sha256": canonical_sha256(ordered) if complete else None,
        "files": ordered,
        "issues": issues,
    }


def _iter_regular_package_files(package: Path) -> Iterable[Path]:
    for candidate in sorted(package.rglob("*")):
        try:
            if candidate.is_symlink() or not candidate.is_file():
                continue
            resolved = candidate.resolve(strict=True)
            resolved.relative_to(package)
        except (OSError, ValueError):
            continue
        yield resolved


def _metadata_only(path: Path, package: Path) -> dict[str, Any]:
    return {
        "path": path.relative_to(package).as_posix(),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def _load_evidence_json(path: Path) -> tuple[Any | None, str | None]:
    try:
        size = path.stat().st_size
        if size <= 0 or size > MAX_EVIDENCE_JSON_BYTES:
            return None, f"evidence_json_size_invalid: {size}"
        return json.loads(path.read_text(encoding="utf-8")), None
    except Exception as exc:  # noqa: BLE001
        return None, f"evidence_json_parse_failed: {type(exc).__name__}: {exc}"


def _get_path(payload: Any, path: Sequence[str]) -> Any:
    current = payload
    for key in path:
        if not isinstance(current, Mapping) or key not in current:
            return None
        current = current[key]
    return current


def _first_present(payload: Any, paths: Sequence[Sequence[str]]) -> Any:
    for path in paths:
        value = _get_path(payload, path)
        if value is not None:
            return value
    return None


def _validate_receipt(
    payload: Any,
    *,
    urdf_sha256: str,
    closure_sha256: str | None,
) -> dict[str, Any]:
    if not isinstance(payload, Mapping):
        return {"valid": False, "missing": ["object_root"], "mismatches": []}
    asset_hash = _first_present(payload, (
        ("asset_sha256",), ("urdf_sha256",), ("asset", "urdf_sha256"), ("bindings", "asset_sha256"),
    ))
    closure_hash = _first_present(payload, (
        ("resource_closure_sha256",), ("bindings", "resource_closure_sha256"), ("resources", "closure_sha256"),
    ))
    protocol = _first_present(payload, (("protocol_id",), ("protocol", "id")))
    runner = _first_present(payload, (
        ("runner_sha256",), ("runner_id",), ("runner", "sha256"), ("runner", "id"),
    ))
    pair_policy = _first_present(payload, (("pair_policy",), ("protocol", "pair_policy")))
    thresholds = _first_present(payload, (("thresholds",), ("protocol", "thresholds")))
    conclusion = _first_present(payload, (("conclusion",), ("verdict",), ("result", "pass")))
    fields = {
        "asset_binding": asset_hash,
        "resource_closure_binding": closure_hash,
        "protocol_identity": protocol,
        "runner_identity": runner,
        "pair_policy": pair_policy,
        "thresholds": thresholds,
        "conclusion": conclusion,
    }
    missing = [key for key, value in fields.items() if value is None or value == "" or value == {}]
    mismatches: list[str] = []
    if asset_hash is not None and asset_hash != urdf_sha256:
        mismatches.append("asset_sha256_mismatch")
    if closure_hash is not None and closure_hash != closure_sha256:
        mismatches.append("resource_closure_sha256_mismatch")
    if closure_sha256 is None:
        mismatches.append("resource_closure_incomplete")
    return {"valid": not missing and not mismatches, "missing": missing, "mismatches": mismatches}


def _validate_rebuild_recipe(
    payload: Any,
    *,
    package: Path,
) -> dict[str, Any]:
    if not isinstance(payload, Mapping):
        return {"eligible": False, "issues": ["recipe_root_not_object"]}
    issues: list[str] = []
    runner = _first_present(payload, (("runner", "sha256"), ("runner", "id"), ("runner_sha256",), ("runner_id",)))
    inputs = payload.get("inputs")
    output = payload.get("output")
    if not runner:
        issues.append("runner_identity_missing")
    if not isinstance(inputs, list) or not inputs:
        issues.append("complete_inputs_missing")
        inputs = []
    if not isinstance(output, Mapping) or not _first_present(output, (("canonical_fingerprint_sha256",), ("sha256",))):
        issues.append("output_fingerprint_missing")
    checked_inputs = 0
    for index, item in enumerate(inputs):
        if not isinstance(item, Mapping):
            issues.append(f"input_{index}_not_object")
            continue
        raw_path = item.get("path")
        expected_hash = item.get("sha256")
        if not isinstance(raw_path, str) or not isinstance(expected_hash, str):
            issues.append(f"input_{index}_binding_incomplete")
            continue
        resolved, issue = _resolve_package_file(
            package,
            package / "recipe-anchor",
            raw_path,
            field=f"rebuild_input_{index}",
            permit_generation_config_hash_only=True,
        )
        if issue:
            issues.append(f"input_{index}: {issue}")
            continue
        assert resolved is not None
        if sha256_file(resolved) != expected_hash:
            issues.append(f"input_{index}_sha256_mismatch")
            continue
        checked_inputs += 1
    return {
        "eligible": not issues and checked_inputs == len(inputs),
        "checked_input_count": checked_inputs,
        "declared_input_count": len(inputs),
        "issues": issues,
    }


def _eligible_nonadjacent_pairs(root: ET.Element) -> tuple[set[tuple[str, str]], list[str]]:
    links = children(root, "link")
    names = [link.attrib.get("name", "").strip() for link in links]
    issues: list[str] = []
    if any(not name for name in names):
        issues.append("unnamed_link")
    if len(set(names)) != len(names):
        issues.append("duplicate_link_name")
    if issues:
        return set(), issues
    adjacent: set[tuple[str, str]] = set()
    for joint in children(root, "joint"):
        parents = children(joint, "parent")
        children_nodes = children(joint, "child")
        if len(parents) != 1 or len(children_nodes) != 1:
            issues.append("joint_parent_child_cardinality")
            continue
        parent = parents[0].attrib.get("link", "").strip()
        child = children_nodes[0].attrib.get("link", "").strip()
        if parent not in names or child not in names or parent == child:
            issues.append("joint_parent_child_invalid")
            continue
        adjacent.add(tuple(sorted((parent, child))))
    collision_names = [
        link.attrib.get("name", "").strip()
        for link in links
        if children(link, "collision")
    ]
    all_pairs = {
        tuple(sorted((collision_names[left], collision_names[right])))
        for left in range(len(collision_names))
        for right in range(left + 1, len(collision_names))
    }
    return all_pairs - adjacent, issues


def _extract_allowance_pairs(payload: Any) -> tuple[set[tuple[str, str]], list[str]]:
    raw_pairs = _first_present(payload, (
        ("excluded_non_adjacent_pairs",),
        ("allowances", "excluded_non_adjacent_pairs"),
        ("pair_policy", "excluded_non_adjacent_pairs"),
    ))
    if not isinstance(raw_pairs, list):
        return set(), ["excluded_non_adjacent_pairs_missing"]
    pairs: set[tuple[str, str]] = set()
    issues: list[str] = []
    for index, raw in enumerate(raw_pairs):
        if isinstance(raw, (list, tuple)) and len(raw) == 2:
            left, right = raw
        elif isinstance(raw, Mapping):
            left, right = raw.get("link_a"), raw.get("link_b")
        else:
            issues.append(f"pair_{index}_invalid_shape")
            continue
        if not isinstance(left, str) or not isinstance(right, str) or not left or not right or left == right:
            issues.append(f"pair_{index}_invalid_names")
            continue
        pairs.add(tuple(sorted((left, right))))
    return pairs, issues


def _s1_evidence_atoms(
    root: ET.Element,
    package: Path,
    urdf_path: Path,
    resource_closure: Mapping[str, Any],
) -> dict[str, Any]:
    generation_configs: list[dict[str, Any]] = []
    supporting_rebuild_artifacts: list[dict[str, Any]] = []
    receipt_paths: list[Path] = []
    rebuild_paths: list[Path] = []
    allowance_paths: list[Path] = []
    for path in _iter_regular_package_files(package):
        lower_name = path.name.lower()
        if _is_generation_config_name(path):
            generation_configs.append(_metadata_only(path, package))
            continue
        if lower_name in {"workflow.json", "export.js"}:
            supporting_rebuild_artifacts.append(_metadata_only(path, package))
        if path.suffix.lower() == ".json" and RECEIPT_NAME_RE.search(lower_name):
            receipt_paths.append(path)
        if lower_name in REBUILD_RECIPE_NAMES:
            rebuild_paths.append(path)
        if path.suffix.lower() == ".json" and ALLOWANCE_NAME_RE.search(lower_name):
            allowance_paths.append(path)

    receipts: list[dict[str, Any]] = []
    for path in receipt_paths:
        payload, load_issue = _load_evidence_json(path)
        validation = (
            {"valid": False, "missing": [], "mismatches": [load_issue]}
            if load_issue
            else _validate_receipt(
                payload,
                urdf_sha256=sha256_file(urdf_path),
                closure_sha256=resource_closure.get("sha256"),
            )
        )
        receipts.append({"path": path.relative_to(package).as_posix(), **validation})
    valid_receipts = sum(record["valid"] for record in receipts)

    rebuild_recipes: list[dict[str, Any]] = []
    for path in rebuild_paths:
        payload, load_issue = _load_evidence_json(path)
        validation = (
            {"eligible": False, "issues": [load_issue]}
            if load_issue
            else _validate_rebuild_recipe(payload, package=package)
        )
        rebuild_recipes.append({"path": path.relative_to(package).as_posix(), **validation})
    rebuild_eligible = any(record["eligible"] for record in rebuild_recipes)

    eligible_pairs, topology_issues = _eligible_nonadjacent_pairs(root)
    registered_pairs: set[tuple[str, str]] = set()
    allowance_records: list[dict[str, Any]] = []
    allowance_issues: list[str] = []
    for path in allowance_paths:
        payload, load_issue = _load_evidence_json(path)
        if load_issue:
            record_issues = [load_issue]
            pairs: set[tuple[str, str]] = set()
        else:
            pairs, record_issues = _extract_allowance_pairs(payload)
        for pair in sorted(pairs):
            if pair not in eligible_pairs:
                record_issues.append(f"pair_not_eligible: {pair[0]}::{pair[1]}")
        if not record_issues:
            registered_pairs.update(pairs)
        allowance_issues.extend(
            f"{path.relative_to(package).as_posix()}: {issue}" for issue in record_issues
        )
        allowance_records.append({
            "path": path.relative_to(package).as_posix(),
            "valid": not record_issues,
            "registered_pair_count": len(pairs) if not record_issues else None,
            "issues": record_issues,
        })
    allowance_issues.extend(topology_issues)
    allowance_complete = not allowance_issues
    return {
        "receipt": {
            "candidate_count": len(receipts),
            "valid_mechanical_receipt_count": valid_receipts,
            "receipt_bound_asset": int(valid_receipts > 0),
            "records": receipts,
            "issues": [
                f"{record['path']}: {issue}"
                for record in receipts
                for issue in [*record.get("missing", []), *record.get("mismatches", [])]
            ],
        },
        "receipt_replay": {
            "eligible_receipt_count": valid_receipts,
            "attempted": 0,
            "passed": None,
            "status": "NOT_RUN_STATIC_STAGE",
        },
        "rebuild": {
            "status": "ELIGIBLE_NOT_RUN" if rebuild_eligible else "N/E",
            "eligible_asset": int(rebuild_eligible),
            "candidate_recipe_count": len(rebuild_recipes),
            "valid_recipe_count": sum(record["eligible"] for record in rebuild_recipes),
            "recipes": rebuild_recipes,
            "generation_config_metadata_only": generation_configs,
            "supporting_artifacts_metadata_only": supporting_rebuild_artifacts,
            "privacy_note": "generation_config.* was hashed only; content was not parsed or returned",
        },
        "allowance": {
            "status": "COMPLETE" if allowance_complete else "NOT_EVALUABLE",
            "candidate_file_count": len(allowance_records),
            "valid_file_count": sum(record["valid"] for record in allowance_records),
            "registered_excluded_pair_count": len(registered_pairs) if allowance_complete else None,
            "eligible_nonadjacent_pair_count": len(eligible_pairs) if not topology_issues else None,
            "records": allowance_records,
            "issues": allowance_issues,
        },
    }


def _failed_record(
    *,
    package: Path,
    asset_id: str,
    urdf_relative_path: str,
    expected_movable_joints: int | None,
    issue: str,
) -> dict[str, Any]:
    intended_joints = expected_movable_joints if expected_movable_joints is not None else 0
    return {
        "schema_version": SCHEMA_VERSION,
        "asset_id": asset_id,
        "package": str(package),
        "urdf_relative_path": urdf_relative_path,
        "urdf_sha256": None,
        "status": "error",
        "parse": {"success": False, "issues": [issue]},
        "table2_supplementary": {
            "visual_bearing_collision_coverage": {
                "status": "NOT_EVALUABLE",
                "asset_intended": 1,
                "asset_passed": 0,
                "asset_pass": False,
                "visual_bearing_links_declared": 0,
                "covered_visual_bearing_links": 0,
                "link_extraction_complete": False,
                "collision_elements_declared_on_visual_links": 0,
                "loadable_collision_elements_on_visual_links": 0,
                "link_records": [],
                "issues": [issue],
            },
            "joint_limit_portability": {
                "status": "NOT_EVALUABLE",
                "joints_intended": intended_joints,
                "joints_extracted": 0,
                "joints_passed": 0,
                "extraction_complete": False,
                "joint_records": [],
                "issues": [issue],
            },
            "joint_dynamics_coverage": {
                "status": "NOT_EVALUABLE",
                "joints_intended": intended_joints,
                "joints_extracted": 0,
                "joints_covered": 0,
                "extraction_complete": False,
                "joint_records": [],
                "issues": [issue],
            },
            "placeholder_mass_incidence": {
                "status": "N/E",
                "dynamic_link_policy": "all_declared_links",
                "dynamic_links": 0,
                "complete_inertial_links": 0,
                "complete_inertial_coverage_numerator": 0,
                "complete_inertial_coverage_denominator": 0,
                "classified_complete_inertial_links": 0,
                "unclassified_complete_inertial_links": 0,
                "placeholder_links": None,
                "incidence_numerator": None,
                "incidence_denominator": 0,
                "registry_ids": [],
                "link_records": [],
                "incomplete_inertial_links": [],
                "issues": [issue],
            },
        },
        "resource_closure": {
            "status": "NOT_EVALUABLE",
            "complete": False,
            "file_count": 0,
            "sha256": None,
            "files": [],
            "issues": [issue],
        },
        "s1_evidence": {
            "status": "NOT_EVALUABLE",
            "issues": [issue],
        },
        "issues": [issue],
    }


def audit_lam_package(
    package: Path | str,
    *,
    urdf_relative_path: str = DEFAULT_URDF_RELATIVE_PATH,
    asset_id: str | None = None,
    expected_movable_joints: int | None = None,
    placeholder_registry: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Return Table 2 supplementary and S1 static atoms for one LAM package.

    The caller owns the frozen cohort, expected joint denominator and placeholder
    registry.  Any path, XML or extraction failure remains an asset-level failure.
    """

    package_raw = Path(package)
    resolved_asset_id = asset_id if asset_id is not None else package_raw.name
    try:
        if expected_movable_joints is not None and (
            isinstance(expected_movable_joints, bool)
            or not isinstance(expected_movable_joints, int)
            or expected_movable_joints < 0
        ):
            raise ValueError("expected_movable_joints must be a non-negative integer or None")
        registry = normalize_placeholder_registry(placeholder_registry)
        if package_raw.is_symlink():
            raise ValueError("package_is_symlink")
        package_resolved = package_raw.resolve(strict=True)
        if not package_resolved.is_dir():
            raise NotADirectoryError(package_resolved)
        urdf_path = _primary_urdf(package_resolved, urdf_relative_path)
    except Exception as exc:  # noqa: BLE001
        return _failed_record(
            package=package_raw,
            asset_id=resolved_asset_id,
            urdf_relative_path=urdf_relative_path,
            expected_movable_joints=expected_movable_joints,
            issue=f"package_preflight_failed: {type(exc).__name__}: {exc}",
        )

    urdf_sha256 = sha256_file(urdf_path)
    try:
        root = ET.parse(urdf_path).getroot()
        if local_tag(root) != "robot":
            raise ValueError(f"root_element_not_robot: {local_tag(root)}")
    except Exception as exc:  # noqa: BLE001
        record = _failed_record(
            package=package_resolved,
            asset_id=resolved_asset_id,
            urdf_relative_path=urdf_relative_path,
            expected_movable_joints=expected_movable_joints,
            issue=f"xml_parse_failed: {type(exc).__name__}: {exc}",
        )
        record["urdf_sha256"] = urdf_sha256
        return record

    link_names = [link.attrib.get("name", "").strip() for link in children(root, "link")]
    link_issues: list[str] = []
    if not link_names:
        link_issues.append("no_declared_links")
    if any(not name for name in link_names):
        link_issues.append("unnamed_link")
    if len(set(link_names)) != len(link_names):
        link_issues.append("duplicate_link_name")
    link_extraction_complete = not link_issues

    visual_collision = _visual_collision_atoms(
        root,
        package_resolved,
        urdf_path,
        link_extraction_complete=link_extraction_complete,
    )
    joint_limit, joint_dynamics, joint_extraction_issues = _joint_atoms(
        root,
        expected_movable_joints=expected_movable_joints,
    )
    placeholder = _placeholder_mass_atoms(root, registry)
    resource_closure = _resource_closure(root, package_resolved, urdf_path)
    s1_evidence = _s1_evidence_atoms(
        root,
        package_resolved,
        urdf_path,
        resource_closure,
    )
    issues = [
        *link_issues,
        *joint_extraction_issues,
        *resource_closure["issues"],
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "asset_id": resolved_asset_id,
        "package": str(package_resolved),
        "urdf_relative_path": urdf_relative_path,
        "urdf_sha256": urdf_sha256,
        "status": "completed",
        "parse": {"success": True, "issues": []},
        "table2_supplementary": {
            "visual_bearing_collision_coverage": visual_collision,
            "joint_limit_portability": joint_limit,
            "joint_dynamics_coverage": joint_dynamics,
            "placeholder_mass_incidence": placeholder,
        },
        "resource_closure": resource_closure,
        "s1_evidence": s1_evidence,
        "issues": issues,
    }


__all__ = [
    "DEFAULT_URDF_RELATIVE_PATH",
    "SCHEMA_VERSION",
    "audit_lam_package",
    "canonical_sha256",
    "normalize_placeholder_registry",
    "safe_package_relative_path",
    "sha256_file",
]
