#!/usr/bin/env python3
"""Build and validate deterministic PV-A physics-injected URDF overlays."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping
import xml.etree.ElementTree as ET

import numpy as np


SIDECAR_SCHEMA = "articraft-seed-physics/v1"
PLAN_SCHEMA = "table5_pva_physics_plan_v1"
POLICY_ID = "pva-physics-common-link-overlay-v1"
PHYSICS_FIELDS = (
    "density_kg_m3",
    "youngs_modulus_pa",
    "poissons_ratio",
    "static_friction_coefficient",
    "dynamic_friction_coefficient",
    "restitution_coefficient",
)
UNSUPPORTED_FIELDS = (
    "youngs_modulus_pa",
    "poissons_ratio",
    "static_friction_coefficient",
    "restitution_coefficient",
)
_EPS = 1.0e-12


class PhysicsInjectionError(ValueError):
    """Raised when a sidecar cannot be injected without hidden assumptions."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    finally:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass


def _json_bytes(value: Any) -> bytes:
    return (
        json.dumps(
            value,
            ensure_ascii=True,
            allow_nan=False,
            sort_keys=True,
            indent=2,
        )
        + "\n"
    ).encode("utf-8")


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise PhysicsInjectionError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise PhysicsInjectionError(f"{label} must be a JSON object: {path}")
    return value


def _number(
    value: Any,
    *,
    field: str,
    positive: bool = False,
    minimum: float | None = None,
    maximum: float | None = None,
) -> float:
    if isinstance(value, bool):
        raise PhysicsInjectionError(f"{field} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise PhysicsInjectionError(f"{field} must be numeric") from error
    if not math.isfinite(result):
        raise PhysicsInjectionError(f"{field} must be finite")
    if positive and result <= 0.0:
        raise PhysicsInjectionError(f"{field} must be positive")
    if minimum is not None and result < minimum:
        raise PhysicsInjectionError(f"{field} must be >= {minimum}")
    if maximum is not None and result > maximum:
        raise PhysicsInjectionError(f"{field} must be <= {maximum}")
    return result


def _values(binding: Mapping[str, Any], *, surface_key: str) -> dict[str, float]:
    raw = binding.get("values")
    if not isinstance(raw, Mapping) or set(raw) != set(PHYSICS_FIELDS):
        raise PhysicsInjectionError(
            f"physics binding {surface_key!r} must contain the exact field set"
        )
    values = {
        "density_kg_m3": _number(
            raw["density_kg_m3"], field=f"{surface_key}.density", positive=True
        ),
        "youngs_modulus_pa": _number(
            raw["youngs_modulus_pa"], field=f"{surface_key}.youngs", positive=True
        ),
        "poissons_ratio": _number(
            raw["poissons_ratio"],
            field=f"{surface_key}.poissons_ratio",
            minimum=-1.0,
            maximum=0.5,
        ),
        "static_friction_coefficient": _number(
            raw["static_friction_coefficient"],
            field=f"{surface_key}.static_friction",
            minimum=0.0,
        ),
        "dynamic_friction_coefficient": _number(
            raw["dynamic_friction_coefficient"],
            field=f"{surface_key}.dynamic_friction",
            minimum=0.0,
        ),
        "restitution_coefficient": _number(
            raw["restitution_coefficient"],
            field=f"{surface_key}.restitution",
            minimum=0.0,
            maximum=1.0,
        ),
    }
    return values


def load_sidecar(
    path: Path,
    *,
    source_urdf_sha256: str,
) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    sidecar = _read_json(path, "physics sidecar")
    if sidecar.get("schema_version") != SIDECAR_SCHEMA:
        raise PhysicsInjectionError(
            f"unsupported physics schema: {sidecar.get('schema_version')!r}"
        )
    if sidecar.get("model_urdf_sha256") != source_urdf_sha256:
        raise PhysicsInjectionError("physics sidecar is not bound to the source URDF")
    if set(sidecar.get("fields", ())) != set(PHYSICS_FIELDS):
        raise PhysicsInjectionError("physics sidecar declares an unexpected field set")
    bindings = sidecar.get("bindings")
    if not isinstance(bindings, list) or not bindings:
        raise PhysicsInjectionError("physics sidecar bindings must be a non-empty list")
    by_surface: dict[str, dict[str, Any]] = {}
    for index, binding in enumerate(bindings):
        if not isinstance(binding, Mapping):
            raise PhysicsInjectionError(f"physics binding {index} is not an object")
        surface_key = binding.get("surface_key")
        if not isinstance(surface_key, str) or surface_key.count("::") != 1:
            raise PhysicsInjectionError(f"physics binding {index} has an invalid surface_key")
        if surface_key in by_surface:
            raise PhysicsInjectionError(f"duplicate physics binding: {surface_key}")
        normalized = dict(binding)
        appearance_only = binding.get("appearance_only")
        if not isinstance(appearance_only, bool):
            raise PhysicsInjectionError(
                f"physics binding {surface_key!r} has invalid appearance_only"
            )
        if appearance_only:
            raw_values = binding.get("values")
            if not isinstance(raw_values, Mapping) or any(
                value is not None for value in raw_values.values()
            ):
                raise PhysicsInjectionError(
                    f"appearance-only binding {surface_key!r} declares physical values"
                )
            normalized["values"] = None
        else:
            normalized["values"] = _values(binding, surface_key=surface_key)
        by_surface[surface_key] = normalized
    return sidecar, by_surface


def _triplet(value: str | None, *, default: tuple[float, float, float]) -> np.ndarray:
    if value is None:
        return np.asarray(default, dtype=float)
    parts = value.split()
    if len(parts) != 3:
        raise PhysicsInjectionError(f"expected three values, found {value!r}")
    result = np.asarray([_number(float(item), field="URDF vector") for item in parts])
    return result


def _origin(element: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    origin = element.find("origin")
    if origin is None:
        xyz = np.zeros(3, dtype=float)
        rpy = np.zeros(3, dtype=float)
    else:
        xyz = _triplet(origin.get("xyz"), default=(0.0, 0.0, 0.0))
        rpy = _triplet(origin.get("rpy"), default=(0.0, 0.0, 0.0))
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )
    return xyz, rotation


def _mesh_path(filename: str, source_urdf: Path) -> Path:
    if filename.startswith("package://"):
        raise PhysicsInjectionError("package:// mesh paths are not supported by PV-A overlay")
    path = Path(filename)
    if not path.is_absolute():
        path = source_urdf.parent / path
    return path.resolve(strict=True)


def _primitive_properties(
    geometry: ET.Element,
) -> tuple[float, float, np.ndarray, np.ndarray, str]:
    children = list(geometry)
    if len(children) != 1:
        raise PhysicsInjectionError("each collision geometry must contain exactly one shape")
    shape = children[0]
    if shape.tag == "box":
        size = _triplet(shape.get("size"), default=(0.0, 0.0, 0.0))
        if np.any(size <= 0.0):
            raise PhysicsInjectionError("box dimensions must be positive")
        x, y, z = size
        volume = float(x * y * z)
        area = float(2.0 * (x * y + x * z + y * z))
        inertia = np.diag(
            [volume * (y * y + z * z) / 12.0,
             volume * (x * x + z * z) / 12.0,
             volume * (x * x + y * y) / 12.0]
        )
    elif shape.tag == "cylinder":
        radius = _number(shape.get("radius"), field="cylinder.radius", positive=True)
        length = _number(shape.get("length"), field="cylinder.length", positive=True)
        volume = math.pi * radius * radius * length
        area = 2.0 * math.pi * radius * (radius + length)
        transverse = volume * (3.0 * radius * radius + length * length) / 12.0
        inertia = np.diag([transverse, transverse, 0.5 * volume * radius * radius])
    elif shape.tag == "sphere":
        radius = _number(shape.get("radius"), field="sphere.radius", positive=True)
        volume = 4.0 * math.pi * radius**3 / 3.0
        area = 4.0 * math.pi * radius**2
        inertia = np.eye(3) * (0.4 * volume * radius * radius)
    else:
        raise PhysicsInjectionError(f"unsupported primitive geometry: {shape.tag}")
    return volume, area, np.zeros(3, dtype=float), inertia, f"analytic_{shape.tag}"


def _mesh_properties(
    geometry: ET.Element,
    *,
    source_urdf: Path,
) -> tuple[float, float, np.ndarray, np.ndarray, str, Path]:
    import trimesh

    mesh_element = geometry.find("mesh")
    if mesh_element is None or len(list(geometry)) != 1:
        raise PhysicsInjectionError("expected one mesh geometry")
    filename = mesh_element.get("filename")
    if not filename:
        raise PhysicsInjectionError("mesh geometry has no filename")
    path = _mesh_path(filename, source_urdf)
    loaded = trimesh.load_mesh(path, force="mesh", process=False, maintain_order=True)
    if not isinstance(loaded, trimesh.Trimesh) or not len(loaded.faces):
        raise PhysicsInjectionError(f"mesh has no usable triangles: {path}")
    if not np.isfinite(np.asarray(loaded.vertices, dtype=float)).all():
        raise PhysicsInjectionError(f"mesh has non-finite vertices: {path}")
    scale = _triplet(mesh_element.get("scale"), default=(1.0, 1.0, 1.0))
    if np.any(scale <= 0.0):
        raise PhysicsInjectionError(f"mesh scale must be positive: {path}")
    mesh = loaded.copy()
    if not np.allclose(scale, 1.0):
        mesh.apply_scale(scale)
    contact_area = float(mesh.area)
    if contact_area <= _EPS or not math.isfinite(contact_area):
        raise PhysicsInjectionError(f"mesh has invalid surface area: {path}")
    policy = "mesh_exact"
    if not bool(mesh.is_watertight):
        try:
            mesh = mesh.convex_hull
        except Exception as error:
            raise PhysicsInjectionError(
                f"non-watertight mesh convex hull failed: {path}: {error}"
            ) from error
        policy = "mesh_convex_hull_fallback"
    volume = float(mesh.volume)
    if volume < 0.0:
        mesh.invert()
        volume = -volume
        policy += "+inverted_winding"
    center = np.asarray(mesh.center_mass, dtype=float)
    inertia = np.asarray(mesh.moment_inertia, dtype=float)
    if (
        volume <= _EPS
        or not np.isfinite(center).all()
        or not np.isfinite(inertia).all()
    ):
        raise PhysicsInjectionError(f"mesh has invalid mass properties: {path}")
    return volume, contact_area, center, inertia, policy, path


def _collision_properties(
    collision: ET.Element,
    *,
    source_urdf: Path,
) -> dict[str, Any]:
    geometry = collision.find("geometry")
    if geometry is None:
        raise PhysicsInjectionError("collision has no geometry")
    if geometry.find("mesh") is not None:
        volume, area, center, inertia, policy, mesh_path = _mesh_properties(
            geometry, source_urdf=source_urdf
        )
    else:
        volume, area, center, inertia, policy = _primitive_properties(geometry)
        mesh_path = None
    translation, rotation = _origin(collision)
    center = rotation @ center + translation
    inertia = rotation @ inertia @ rotation.T
    return {
        "volume_m3": volume,
        "surface_area_m2": area,
        "center_of_mass_xyz": center,
        "unit_density_inertia_kg_m2": inertia,
        "mass_geometry_policy": policy,
        "mesh_path": mesh_path,
    }


def _valid_inertial(link: ET.Element) -> tuple[bool, dict[str, Any] | None]:
    inertial = link.find("inertial")
    if inertial is None:
        return False, None
    try:
        mass_element = inertial.find("mass")
        inertia_element = inertial.find("inertia")
        if mass_element is None or inertia_element is None:
            return False, None
        mass = _number(mass_element.get("value"), field="inertial.mass", positive=True)
        ixx = _number(inertia_element.get("ixx"), field="inertial.ixx")
        ixy = _number(inertia_element.get("ixy"), field="inertial.ixy")
        ixz = _number(inertia_element.get("ixz"), field="inertial.ixz")
        iyy = _number(inertia_element.get("iyy"), field="inertial.iyy")
        iyz = _number(inertia_element.get("iyz"), field="inertial.iyz")
        izz = _number(inertia_element.get("izz"), field="inertial.izz")
        matrix = np.asarray(
            [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float
        )
        eigenvalues = np.linalg.eigvalsh(matrix)
        valid = bool(
            eigenvalues[0] > _EPS
            and eigenvalues[2] <= eigenvalues[0] + eigenvalues[1] + 1.0e-9
        )
        return valid, {
            "mass_kg": mass,
            "inertia_eigenvalues_kg_m2": [float(value) for value in eigenvalues],
        }
    except PhysicsInjectionError:
        return False, None


def _format(value: float) -> str:
    if not math.isfinite(value):
        raise PhysicsInjectionError("cannot serialize a non-finite physical value")
    return format(float(value), ".17g")


def _insert_inertial(
    link: ET.Element,
    *,
    mass: float,
    center: np.ndarray,
    inertia: np.ndarray,
) -> None:
    previous = link.find("inertial")
    if previous is not None:
        link.remove(previous)
    inertial = ET.Element("inertial")
    ET.SubElement(
        inertial,
        "origin",
        {"xyz": " ".join(_format(value) for value in center), "rpy": "0 0 0"},
    )
    ET.SubElement(inertial, "mass", {"value": _format(mass)})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": _format(inertia[0, 0]),
            "ixy": _format(inertia[0, 1]),
            "ixz": _format(inertia[0, 2]),
            "iyy": _format(inertia[1, 1]),
            "iyz": _format(inertia[1, 2]),
            "izz": _format(inertia[2, 2]),
        },
    )
    link.insert(0, inertial)


def _derive_link_inertial(collisions: list[dict[str, Any]]) -> dict[str, Any]:
    masses = np.asarray(
        [row["volume_m3"] * row["values"]["density_kg_m3"] for row in collisions],
        dtype=float,
    )
    total_mass = float(masses.sum())
    if total_mass <= _EPS:
        raise PhysicsInjectionError("link collision solids have no positive mass")
    centers = np.asarray([row["center_of_mass_xyz"] for row in collisions], dtype=float)
    center = (masses[:, None] * centers).sum(axis=0) / total_mass
    inertia = np.zeros((3, 3), dtype=float)
    for mass, collision_center, row in zip(masses, centers, collisions, strict=True):
        density = row["values"]["density_kg_m3"]
        inertia += density * row["unit_density_inertia_kg_m2"]
        offset = collision_center - center
        inertia += mass * (
            float(np.dot(offset, offset)) * np.eye(3) - np.outer(offset, offset)
        )
    inertia = 0.5 * (inertia + inertia.T)
    eigenvalues = np.linalg.eigvalsh(inertia)
    if (
        not np.isfinite(inertia).all()
        or eigenvalues[0] <= _EPS
        or eigenvalues[2] > eigenvalues[0] + eigenvalues[1] + 1.0e-9
    ):
        raise PhysicsInjectionError(
            f"derived link inertia is invalid: eigenvalues={eigenvalues.tolist()}"
        )
    return {
        "mass_kg": total_mass,
        "center_of_mass_xyz": center,
        "inertia_kg_m2": inertia,
        "inertia_eigenvalues_kg_m2": eigenvalues,
    }


def build_injected_asset(
    *,
    source_urdf: Path,
    physics_path: Path,
    destination_urdf: Path,
    plan_path: Path,
) -> dict[str, Any]:
    source_urdf = source_urdf.resolve(strict=True)
    physics_path = physics_path.resolve(strict=True)
    source_hash = sha256_file(source_urdf)
    sidecar, by_surface = load_sidecar(
        physics_path, source_urdf_sha256=source_hash
    )
    try:
        tree = ET.parse(source_urdf)
    except (OSError, ET.ParseError) as error:
        raise PhysicsInjectionError(f"cannot parse source URDF {source_urdf}: {error}") from error
    root = tree.getroot()
    if root.tag != "robot":
        raise PhysicsInjectionError("source URDF root must be <robot>")

    source_collision_keys: set[str] = set()
    mass_bearing_collision_keys: set[str] = set()
    visual_keys: set[str] = set()
    link_plans: list[dict[str, Any]] = []
    derived_count = 0
    preserved_count = 0
    fallback_count = 0
    appearance_only_collision_count = 0
    collisionless_inertial_link_count = 0
    for link_index, link in enumerate(root.findall("link")):
        link_name = link.get("name")
        if not link_name:
            raise PhysicsInjectionError("URDF link has no name")
        for visual_index, visual in enumerate(link.findall("visual")):
            visual_name = visual.get("name", f"visual_{visual_index}")
            visual_keys.add(f"{link_name}::{visual_name}")
        collision_rows: list[dict[str, Any]] = []
        for collision_index, collision in enumerate(link.findall("collision")):
            collision_name = collision.get("name", f"collision_{collision_index}")
            surface_key = f"{link_name}::{collision_name}"
            if surface_key in source_collision_keys:
                raise PhysicsInjectionError(f"duplicate collision surface: {surface_key}")
            source_collision_keys.add(surface_key)
            binding = by_surface.get(surface_key)
            if binding is None:
                raise PhysicsInjectionError(f"collision has no physics binding: {surface_key}")
            if binding["appearance_only"] is True:
                link.remove(collision)
                appearance_only_collision_count += 1
                continue
            mass_bearing_collision_keys.add(surface_key)
            properties = _collision_properties(collision, source_urdf=source_urdf)
            unique_geom_name = f"physics_{link_index:04d}_{collision_index:04d}"
            collision.set("name", unique_geom_name)
            row = {
                "surface_key": surface_key,
                "source_collision_name": collision_name,
                "injected_geom_name": unique_geom_name,
                "values": binding["values"],
                "volume_m3": properties["volume_m3"],
                "surface_area_m2": properties["surface_area_m2"],
                "center_of_mass_xyz": properties["center_of_mass_xyz"],
                "unit_density_inertia_kg_m2": properties[
                    "unit_density_inertia_kg_m2"
                ],
                "mass_geometry_policy": properties["mass_geometry_policy"],
            }
            if "convex_hull" in properties["mass_geometry_policy"]:
                fallback_count += 1
            collision_rows.append(row)

        valid_inertial, source_inertial = _valid_inertial(link)
        if not collision_rows and not valid_inertial:
            raise PhysicsInjectionError(
                f"link {link_name!r} has neither mass-bearing collision nor valid inertial"
            )
        total_area = sum(row["surface_area_m2"] for row in collision_rows)
        if collision_rows and total_area <= _EPS:
            raise PhysicsInjectionError(f"link {link_name!r} has no positive collision area")
        dynamic_friction = (
            sum(
                row["surface_area_m2"]
                * row["values"]["dynamic_friction_coefficient"]
                for row in collision_rows
            )
            / total_area
            if collision_rows
            else None
        )
        if valid_inertial:
            preserved_count += 1
            collisionless_inertial_link_count += not collision_rows
            inertial_plan: dict[str, Any] = {
                "action": "preserved_valid_source_urdf",
                **(source_inertial or {}),
            }
        else:
            derived = _derive_link_inertial(collision_rows)
            _insert_inertial(
                link,
                mass=derived["mass_kg"],
                center=derived["center_of_mass_xyz"],
                inertia=derived["inertia_kg_m2"],
            )
            derived_count += 1
            inertial_plan = {
                "action": "derived_from_collision_solids_and_surface_density",
                "mass_kg": derived["mass_kg"],
                "center_of_mass_xyz": [
                    float(value) for value in derived["center_of_mass_xyz"]
                ],
                "inertia_kg_m2": [
                    [float(value) for value in row]
                    for row in derived["inertia_kg_m2"]
                ],
                "inertia_eigenvalues_kg_m2": [
                    float(value) for value in derived["inertia_eigenvalues_kg_m2"]
                ],
                "composition": "additive_collision_solids_with_parallel_axis_theorem",
            }
        link_plans.append(
            {
                "link_name": link_name,
                "dynamic_friction_coefficient": dynamic_friction,
                "friction_reduction": (
                    "collision_surface_area_weighted_mean"
                    if collision_rows
                    else "not_applicable_no_mass_bearing_collision"
                ),
                "collision_surface_area_m2": total_area,
                "collision_count": len(collision_rows),
                "collisions": [
                    {
                        key: (
                            [float(value) for value in value]
                            if isinstance(value, np.ndarray) and value.ndim == 1
                            else [
                                [float(item) for item in row]
                                for row in value
                            ]
                            if isinstance(value, np.ndarray)
                            else value
                        )
                        for key, value in row.items()
                        if key != "unit_density_inertia_kg_m2"
                    }
                    for row in collision_rows
                ],
                "inertial": inertial_plan,
            }
        )

    if source_collision_keys != set(by_surface):
        missing = sorted(set(by_surface) - source_collision_keys)
        extra = sorted(source_collision_keys - set(by_surface))
        raise PhysicsInjectionError(
            f"physics/collision coverage mismatch: missing={missing[:5]}, extra={extra[:5]}"
        )
    if visual_keys != source_collision_keys:
        raise PhysicsInjectionError("PV-A visual and collision surface keys do not match")

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename:
            mesh.set("filename", str(_mesh_path(filename, source_urdf)))
    ET.indent(tree, space="  ")
    urdf_payload = ET.tostring(root, encoding="utf-8", xml_declaration=False) + b"\n"
    _atomic_write(destination_urdf, urdf_payload)
    injected_hash = sha256_file(destination_urdf)
    plan: dict[str, Any] = {
        "schema_version": PLAN_SCHEMA,
        "policy_id": POLICY_ID,
        "source_urdf_path": str(source_urdf),
        "source_urdf_sha256": source_hash,
        "physics_path": str(physics_path),
        "physics_sha256": sha256_file(physics_path),
        "physics_schema_version": sidecar["schema_version"],
        "physics_model_urdf_sha256": sidecar["model_urdf_sha256"],
        "injected_urdf_path": str(destination_urdf.resolve()),
        "injected_urdf_sha256": injected_hash,
        "binding_count": len(by_surface),
        "mass_bearing_binding_count": len(mass_bearing_collision_keys),
        "appearance_only_collision_count": appearance_only_collision_count,
        "link_count": len(link_plans),
        "collisionless_valid_inertial_link_count": collisionless_inertial_link_count,
        "preserved_inertial_link_count": preserved_count,
        "derived_inertial_link_count": derived_count,
        "convex_hull_fallback_collision_count": fallback_count,
        "field_disposition": {
            "density_kg_m3": (
                "derive only missing or invalid link inertials; preserve valid source inertials"
            ),
            "dynamic_friction_coefficient": (
                "surface-area weighted to link and applied identically at link granularity"
            ),
            "static_friction_coefficient": "unsupported across all three rigid APIs",
            "restitution_coefficient": "unsupported as a common exact rigid contact field",
            "youngs_modulus_pa": "unsupported as a common exact rigid contact field",
            "poissons_ratio": "unsupported as a common exact rigid contact field",
            "joint_damping": "not present in physics.json; no value invented",
            "joint_friction": "not present in physics.json; no value invented",
        },
        "links": link_plans,
    }
    _atomic_write(plan_path, _json_bytes(plan))
    plan["plan_path"] = str(plan_path.resolve())
    plan["plan_sha256"] = sha256_file(plan_path)
    return plan


def load_plan(
    path: Path,
    *,
    source_urdf_sha256: str,
    physics_sha256: str,
    injected_urdf_sha256: str,
) -> dict[str, Any]:
    plan = _read_json(path, "physics injection plan")
    expected = {
        "schema_version": PLAN_SCHEMA,
        "policy_id": POLICY_ID,
        "source_urdf_sha256": source_urdf_sha256,
        "physics_sha256": physics_sha256,
        "injected_urdf_sha256": injected_urdf_sha256,
    }
    for field, value in expected.items():
        if plan.get(field) != value:
            raise PhysicsInjectionError(
                f"physics injection plan {field} mismatch: {plan.get(field)!r} != {value!r}"
            )
    links = plan.get("links")
    if not isinstance(links, list) or len(links) != plan.get("link_count"):
        raise PhysicsInjectionError("physics injection plan has invalid links")
    names = [row.get("link_name") for row in links if isinstance(row, Mapping)]
    if len(names) != len(links) or len(names) != len(set(names)):
        raise PhysicsInjectionError("physics injection plan has invalid link names")
    return plan


__all__ = [
    "PHYSICS_FIELDS",
    "PLAN_SCHEMA",
    "POLICY_ID",
    "SIDECAR_SCHEMA",
    "PhysicsInjectionError",
    "build_injected_asset",
    "load_plan",
    "load_sidecar",
    "sha256_file",
]
