#!/usr/bin/env python3
"""Render one Articraft-10K URDF package with the frozen PV-A studio.

The released Articraft-10K packages contain ``model.urdf`` and, for mesh
visuals, a relative ``assets/`` tree.  Unlike the portable Articraft export
renderer, these release packages do not carry ``appearance.json``.  This
worker therefore uses the URDF visual material color directly and keeps the
same Cycles, camera, world, ground, and area-light policy as the PV-A and
Artiverse uniform renderers.

Run through Blender, for example::

    blender -b --factory-startup -P render_articraft10k_asset_blender.py -- \
        --asset-dir /path/to/record --output /path/to/000.png \
        --resolution 256 --samples 4
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import math
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path, PureWindowsPath
from typing import Any, Sequence


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_SHARED_RENDERER = REPO_ROOT / "arti-template/scripts/render_exported_asset_blender.py"
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class AssetPackageError(RuntimeError):
    """Raised when a release package cannot be rendered without guessing."""


@dataclass(frozen=True, slots=True)
class Transform:
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class JointAsset:
    name: str
    parent: str
    child: str
    origin: Transform


@dataclass(frozen=True, slots=True)
class VisualAsset:
    link_name: str
    visual_name: str
    geometry_type: str
    geometry_params: tuple[float, ...]
    mesh_path: Path | None
    mesh_scale: tuple[float, float, float]
    origin: Transform
    color: tuple[float, float, float, float]
    material_name: str


@dataclass(frozen=True, slots=True)
class AssetPackage:
    root: Path
    root_link: str
    links: tuple[str, ...]
    joints: tuple[JointAsset, ...]
    visuals: tuple[VisualAsset, ...]


Matrix4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _shared_helpers(
    shared_renderer: Path,
    expected_sha256: str | None,
) -> tuple[tuple[Any, Any, Any, Any], dict[str, object]]:
    """Load the exact audited studio helper file after checking its receipt."""

    try:
        path = shared_renderer.expanduser().resolve(strict=True)
    except OSError as exc:
        raise AssetPackageError(f"shared renderer does not exist: {shared_renderer}") from exc
    if not path.is_file():
        raise AssetPackageError(f"shared renderer is not a regular file: {path}")
    actual_sha256 = _sha256(path)
    if expected_sha256 is not None:
        if SHA256_RE.fullmatch(expected_sha256) is None:
            raise AssetPackageError("shared renderer SHA-256 receipt is malformed")
        if actual_sha256 != expected_sha256:
            raise AssetPackageError(
                f"shared renderer SHA-256 mismatch: expected {expected_sha256}, found {actual_sha256}"
            )

    module_name = f"_articraft10k_shared_renderer_{actual_sha256}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise AssetPackageError(f"cannot create an import spec for shared renderer: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception as exc:
        sys.modules.pop(module_name, None)
        raise AssetPackageError(f"cannot load shared renderer {path}: {exc}") from exc

    names = (
        "_enable_cycles",
        "_scene_bounds",
        "_look_at",
        "_add_principled_material",
    )
    helpers = tuple(getattr(module, name, None) for name in names)
    if not all(callable(helper) for helper in helpers):
        raise AssetPackageError(f"shared renderer lacks required studio helpers: {path}")
    return helpers, {"path": str(path), "sha256": actual_sha256}


def _triplet(
    value: object,
    *,
    field: str,
    default: tuple[float, float, float],
) -> tuple[float, float, float]:
    if value is None:
        return default
    chunks = str(value).split()
    if len(chunks) != 3:
        raise AssetPackageError(f"{field} must contain exactly three numbers")
    try:
        result = tuple(float(chunk) for chunk in chunks)
    except (TypeError, ValueError) as exc:
        raise AssetPackageError(f"{field} contains a non-numeric value") from exc
    if not all(math.isfinite(item) for item in result):
        raise AssetPackageError(f"{field} contains a non-finite value")
    return result  # type: ignore[return-value]


def _origin(element: ET.Element | None, *, field: str) -> Transform:
    if element is None:
        return Transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    return Transform(
        _triplet(element.get("xyz"), field=f"{field}.xyz", default=(0.0, 0.0, 0.0)),
        _triplet(element.get("rpy"), field=f"{field}.rpy", default=(0.0, 0.0, 0.0)),
    )


def _contained_file(root: Path, value: object, *, field: str) -> Path:
    text = str(value or "").strip()
    # The official packages use assets/...; reject URI and Windows path forms
    # instead of silently resolving them outside the package.
    relative = Path(text)
    windows = PureWindowsPath(text)
    if (
        not text
        or "\\" in text
        or "://" in text
        or relative.is_absolute()
        or windows.is_absolute()
        or bool(windows.drive)
        or any(part in {"", ".", ".."} for part in relative.parts)
    ):
        raise AssetPackageError(f"{field} must be a contained relative path")
    root = root.resolve(strict=True)
    try:
        path = (root / relative).resolve(strict=True)
        path.relative_to(root)
    except (OSError, ValueError) as exc:
        raise AssetPackageError(
            f"{field} must resolve to an existing contained file"
        ) from exc
    if not path.is_file():
        raise AssetPackageError(f"{field} does not resolve to a regular file")
    return path


def _rgba(
    element: ET.Element | None,
    *,
    field: str,
    default: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    if element is None:
        return default
    text = str(element.get("rgba") or "").split()
    if len(text) not in {3, 4}:
        raise AssetPackageError(f"{field}.rgba must contain three or four numbers")
    try:
        values = [float(value) for value in text]
    except (TypeError, ValueError) as exc:
        raise AssetPackageError(f"{field}.rgba contains a non-numeric value") from exc
    if len(values) == 3:
        values.append(1.0)
    if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in values):
        raise AssetPackageError(f"{field}.rgba must be finite and in [0, 1]")
    return tuple(values)  # type: ignore[return-value]


def load_asset_package(asset_dir: Path) -> AssetPackage:
    """Parse one URDF release package and resolve every visual resource."""

    try:
        root = asset_dir.expanduser().resolve(strict=True)
    except OSError as exc:
        raise AssetPackageError(f"asset directory does not exist: {asset_dir}") from exc
    if not root.is_dir():
        raise AssetPackageError(f"asset directory is not a directory: {root}")
    urdf_path = _contained_file(root, "model.urdf", field="model.urdf")
    try:
        robot = ET.parse(urdf_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise AssetPackageError(f"cannot parse model.urdf: {exc}") from exc
    if robot.tag != "robot":
        raise AssetPackageError("model.urdf root element must be <robot>")

    # URDF permits a document-level material declaration and a per-visual
    # declaration.  Missing colors are intentionally rendered as one fixed
    # neutral gray, so a malformed/nameless material never changes selection.
    default_color = (0.62, 0.64, 0.68, 1.0)
    global_materials: dict[str, tuple[float, float, float, float]] = {}
    for material in robot.findall("material"):
        name = str(material.get("name") or "").strip()
        if not name:
            continue
        global_materials[name] = _rgba(
            material.find("color"),
            field=f"material {name!r}",
            default=default_color,
        )

    links: list[str] = []
    visuals: list[VisualAsset] = []
    seen_links: set[str] = set()
    for link_element in robot.findall("link"):
        link_name = str(link_element.get("name") or "").strip()
        if not link_name or link_name in seen_links:
            raise AssetPackageError(f"invalid or duplicate link {link_name!r}")
        seen_links.add(link_name)
        links.append(link_name)
        for visual_index, visual_element in enumerate(link_element.findall("visual")):
            visual_name = str(visual_element.get("name") or "").strip()
            if not visual_name:
                visual_name = f"visual_{visual_index:04d}"
            material_element = visual_element.find("material")
            material_name = "__default__"
            color = default_color
            if material_element is not None:
                material_name = str(material_element.get("name") or "").strip() or "__unnamed__"
                inline_color = material_element.find("color")
                if inline_color is not None:
                    color = _rgba(
                        inline_color,
                        field=f"visual {link_name}::{visual_name} material",
                        default=default_color,
                    )
                elif material_name in global_materials:
                    color = global_materials[material_name]
            geometry = visual_element.find("geometry")
            if geometry is None:
                raise AssetPackageError(f"visual {link_name}::{visual_name} has no geometry")
            mesh_element = geometry.find("mesh")
            geometry_type = "mesh"
            geometry_params: tuple[float, ...] = ()
            mesh_path: Path | None = None
            mesh_scale = (1.0, 1.0, 1.0)
            if mesh_element is not None:
                mesh_path = _contained_file(
                    root,
                    mesh_element.get("filename"),
                    field=f"visual {link_name}::{visual_name} mesh filename",
                )
                mesh_scale = _triplet(
                    mesh_element.get("scale"),
                    field=f"visual {link_name}::{visual_name} mesh scale",
                    default=(1.0, 1.0, 1.0),
                )
                if any(value == 0.0 for value in mesh_scale):
                    raise AssetPackageError(
                        f"visual {link_name}::{visual_name} mesh scale cannot contain zero"
                    )
            else:
                box_element = geometry.find("box")
                cylinder_element = geometry.find("cylinder")
                sphere_element = geometry.find("sphere")
                if box_element is not None:
                    geometry_type = "box"
                    geometry_params = _triplet(
                        box_element.get("size"),
                        field=f"visual {link_name}::{visual_name} box size",
                        default=(0.0, 0.0, 0.0),
                    )
                    if any(value <= 0.0 for value in geometry_params):
                        raise AssetPackageError(
                            f"visual {link_name}::{visual_name} box size must be positive"
                        )
                elif cylinder_element is not None:
                    geometry_type = "cylinder"
                    try:
                        radius = float(cylinder_element.get("radius", "nan"))
                        length = float(cylinder_element.get("length", "nan"))
                    except (TypeError, ValueError) as exc:
                        raise AssetPackageError(
                            f"visual {link_name}::{visual_name} cylinder dimensions must be numeric"
                        ) from exc
                    if (
                        not math.isfinite(radius)
                        or not math.isfinite(length)
                        or radius <= 0.0
                        or length <= 0.0
                    ):
                        raise AssetPackageError(
                            f"visual {link_name}::{visual_name} cylinder dimensions must be positive"
                        )
                    geometry_params = (radius, length)
                elif sphere_element is not None:
                    geometry_type = "sphere"
                    try:
                        radius = float(sphere_element.get("radius", "nan"))
                    except (TypeError, ValueError) as exc:
                        raise AssetPackageError(
                            f"visual {link_name}::{visual_name} sphere radius must be numeric"
                        ) from exc
                    if not math.isfinite(radius) or radius <= 0.0:
                        raise AssetPackageError(
                            f"visual {link_name}::{visual_name} sphere radius must be positive"
                        )
                    geometry_params = (radius,)
                else:
                    raise AssetPackageError(
                        f"visual {link_name}::{visual_name} has unsupported geometry"
                    )
            visuals.append(
                VisualAsset(
                    link_name=link_name,
                    visual_name=visual_name,
                    geometry_type=geometry_type,
                    geometry_params=geometry_params,
                    mesh_path=mesh_path,
                    mesh_scale=mesh_scale,
                    origin=_origin(
                        visual_element.find("origin"),
                        field=f"visual {link_name}::{visual_name} origin",
                    ),
                    color=color,
                    material_name=material_name,
                )
            )

    if not visuals:
        raise AssetPackageError("model.urdf contains no renderable visuals")

    joints: list[JointAsset] = []
    child_links: set[str] = set()
    seen_joints: set[str] = set()
    for joint_element in robot.findall("joint"):
        name = str(joint_element.get("name") or "").strip()
        parent_element = joint_element.find("parent")
        child_element = joint_element.find("child")
        parent = (
            "" if parent_element is None else str(parent_element.get("link") or "").strip()
        )
        child = "" if child_element is None else str(child_element.get("link") or "").strip()
        if not name or name in seen_joints or parent not in seen_links or child not in seen_links:
            raise AssetPackageError(f"joint {name!r} has an invalid parent or child")
        if child in child_links:
            raise AssetPackageError(f"link {child!r} has more than one parent joint")
        seen_joints.add(name)
        child_links.add(child)
        joints.append(
            JointAsset(
                name=name,
                parent=parent,
                child=child,
                origin=_origin(joint_element.find("origin"), field=f"joint {name!r} origin"),
            )
        )
    roots = [link for link in links if link not in child_links]
    if len(roots) != 1:
        raise AssetPackageError(f"model.urdf must have one root link; found {roots!r}")
    return AssetPackage(
        root=root,
        root_link=roots[0],
        links=tuple(links),
        joints=tuple(joints),
        visuals=tuple(visuals),
    )


def _origin_matrix(transform: Transform) -> Matrix4:
    roll, pitch, yaw = transform.rpy
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    x, y, z = transform.xyz
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y),
        (-sp, cp * sr, cp * cr, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _matmul(left: Matrix4, right: Matrix4) -> Matrix4:
    return tuple(
        tuple(
            sum(left[row][inner] * right[inner][column] for inner in range(4))
            for column in range(4)
        )
        for row in range(4)
    )  # type: ignore[return-value]


def rest_link_matrices(package: AssetPackage) -> dict[str, Matrix4]:
    """Compute all link transforms at the zero/rest joint configuration."""

    identity: Matrix4 = (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    matrices = {package.root_link: identity}
    pending = list(package.joints)
    while pending:
        progress = False
        remaining: list[JointAsset] = []
        for joint in pending:
            parent_matrix = matrices.get(joint.parent)
            if parent_matrix is None:
                remaining.append(joint)
                continue
            matrices[joint.child] = _matmul(parent_matrix, _origin_matrix(joint.origin))
            progress = True
        if not progress:
            unresolved = ", ".join(joint.name for joint in remaining)
            raise AssetPackageError(f"joint graph is cyclic or disconnected: {unresolved}")
        pending = remaining
    missing = sorted(set(package.links) - set(matrices))
    if missing:
        raise AssetPackageError(f"joint graph omits links: {missing!r}")
    return matrices


def _color_material(bpy: Any, color: tuple[float, float, float, float], key: str) -> Any:
    material = bpy.data.materials.new(f"URDF::{key}")
    material.use_nodes = True
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    if bsdf is None:
        raise AssetPackageError("Blender Principled BSDF node is unavailable")
    bsdf.inputs["Base Color"].default_value = color
    bsdf.inputs["Roughness"].default_value = 0.62
    bsdf.inputs["Metallic"].default_value = 0.0
    if "Alpha" in bsdf.inputs:
        bsdf.inputs["Alpha"].default_value = color[3]
    if color[3] < 0.999:
        if hasattr(material, "surface_render_method"):
            material.surface_render_method = "DITHERED"
        elif hasattr(material, "blend_method"):
            material.blend_method = "BLEND"
    return material


def render_asset(
    package: AssetPackage,
    output: Path,
    *,
    resolution: int,
    samples: int,
    shared_renderer: Path = DEFAULT_SHARED_RENDERER,
    shared_renderer_sha256: str | None = None,
) -> dict[str, object]:
    """Build the common studio scene and render the URDF rest pose."""

    try:
        import bpy
        import mathutils
    except ModuleNotFoundError as exc:  # pragma: no cover - Blender only
        raise AssetPackageError("rendering requires Blender's bpy Python environment") from exc
    if resolution < 64 or samples < 1:
        raise AssetPackageError("resolution >= 64 and samples >= 1 are required")

    helpers, shared_renderer_receipt = _shared_helpers(
        shared_renderer,
        shared_renderer_sha256,
    )
    _enable_cycles, _scene_bounds, _look_at, _add_principled_material = helpers
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    device = _enable_cycles(bpy, scene, samples)
    scene.render.resolution_x = resolution
    scene.render.resolution_y = resolution
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.film_transparent = False
    scene.render.use_file_extension = True
    if hasattr(scene, "view_settings"):
        try:
            scene.view_settings.view_transform = "AgX"
            scene.view_settings.look = "AgX - Medium High Contrast"
        except (TypeError, ValueError):
            pass

    world = bpy.data.worlds.new("Articraft Studio")
    world.use_nodes = True
    background = world.node_tree.nodes.get("Background")
    if background is None:
        raise AssetPackageError("Blender world has no Background node")
    background.inputs["Color"].default_value = (0.80, 0.84, 0.90, 1.0)
    background.inputs["Strength"].default_value = 0.55
    scene.world = world

    matrices = rest_link_matrices(package)
    material_cache: dict[tuple[float, float, float, float, str], Any] = {}
    imported: list[Any] = []
    imported_mesh_files = 0
    primitive_counts: dict[str, int] = {"box": 0, "cylinder": 0, "sphere": 0}
    for visual in package.visuals:
        before = set(scene.objects)
        geometry_scale = mathutils.Matrix.Identity(4)
        if visual.geometry_type == "mesh":
            if visual.mesh_path is None:
                raise AssetPackageError(f"visual {visual.link_name}::{visual.visual_name} has no mesh")
            if hasattr(bpy.ops.wm, "obj_import"):
                result = bpy.ops.wm.obj_import(filepath=str(visual.mesh_path))
            else:  # pragma: no cover - Blender 3.x compatibility
                result = bpy.ops.import_scene.obj(filepath=str(visual.mesh_path))
            if "FINISHED" not in result:
                raise AssetPackageError(f"OBJ import did not finish: {visual.mesh_path}")
            imported_mesh_files += 1
        elif visual.geometry_type == "box":
            bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0.0, 0.0, 0.0))
            geometry_scale = mathutils.Matrix.Diagonal((*visual.geometry_params, 1.0))
            primitive_counts["box"] += 1
        elif visual.geometry_type == "cylinder":
            radius, length = visual.geometry_params
            bpy.ops.mesh.primitive_cylinder_add(
                vertices=48,
                radius=radius,
                depth=length,
                location=(0.0, 0.0, 0.0),
            )
            primitive_counts["cylinder"] += 1
        elif visual.geometry_type == "sphere":
            (radius,) = visual.geometry_params
            bpy.ops.mesh.primitive_uv_sphere_add(
                segments=48,
                ring_count=24,
                radius=radius,
                location=(0.0, 0.0, 0.0),
            )
            primitive_counts["sphere"] += 1
        else:  # pragma: no cover - parser rejects this
            raise AssetPackageError(f"unsupported geometry type: {visual.geometry_type}")
        objects = [
            obj for obj in scene.objects if obj not in before and obj.type == "MESH"
        ]
        if not objects:
            raise AssetPackageError(
                f"visual {visual.link_name}::{visual.visual_name} imported no mesh objects"
            )
        world_matrix = mathutils.Matrix(matrices[visual.link_name])
        visual_matrix = mathutils.Matrix(_origin_matrix(visual.origin))
        scale_matrix = mathutils.Matrix.Diagonal((*visual.mesh_scale, 1.0))
        cache_key = (*visual.color, visual.material_name)
        material = material_cache.get(cache_key)
        if material is None:
            material = _color_material(bpy, visual.color, f"{visual.material_name}::{len(material_cache):04d}")
            material_cache[cache_key] = material
        for index, obj in enumerate(objects):
            obj.name = (
                f"{visual.link_name}::{visual.visual_name}"
                if index == 0
                else f"{visual.link_name}::{visual.visual_name}::{index}"
            )
            obj.matrix_world = world_matrix @ visual_matrix @ geometry_scale @ scale_matrix
            obj.data.materials.clear()
            obj.data.materials.append(material)
            imported.append(obj)

    minimum, maximum = _scene_bounds(imported, mathutils)
    center = (minimum + maximum) * 0.5
    extents = maximum - minimum
    radius = max(float(extents.length) * 0.5, 1e-4)

    bpy.ops.mesh.primitive_plane_add(
        size=radius * 12.0,
        location=(float(center.x), float(center.y), float(minimum.z) - radius * 0.015),
    )
    ground = bpy.context.object
    ground.name = "Studio Ground"
    ground.data.materials.append(
        _add_principled_material(
            bpy,
            "Studio Ground",
            (0.32, 0.35, 0.40, 1.0),
            0.82,
        )
    )

    camera_data = bpy.data.cameras.new("Camera")
    camera_data.sensor_fit = "VERTICAL"
    camera_data.angle_y = math.radians(42.0)
    camera = bpy.data.objects.new("Camera", camera_data)
    bpy.context.collection.objects.link(camera)
    camera_direction = mathutils.Vector((1.25, -1.35, 0.85)).normalized()
    camera_distance = radius / math.sin(camera_data.angle_y * 0.5) * 1.18
    camera.location = center + camera_direction * camera_distance
    camera_data.clip_start = max(1e-6, camera_distance * 1e-4)
    camera_data.clip_end = max(1.0, camera_distance * 100.0)
    _look_at(camera, center, mathutils)
    scene.camera = camera

    for index, (direction_values, gain, size_ratio) in enumerate(
        (
            ((0.4, -0.8, 1.5), 42.0, 1.5),
            ((-1.2, -0.3, 0.6), 15.0, 1.8),
            ((0.2, 1.0, 1.2), 24.0, 1.2),
        )
    ):
        light_direction = mathutils.Vector(direction_values).normalized()
        light_distance = radius * 4.0
        light_data = bpy.data.lights.new(f"Studio Light {index}", type="AREA")
        light_data.energy = gain * light_distance * light_distance
        light_data.size = size_ratio * radius
        light = bpy.data.objects.new(f"Studio Light {index}", light_data)
        bpy.context.collection.objects.link(light)
        light.location = center + light_direction * light_distance
        _look_at(light, center, mathutils)

    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(output)
    bpy.ops.render.render(write_still=True)
    if (
        not output.is_file()
        or output.stat().st_size <= len(PNG_SIGNATURE)
        or output.read_bytes()[: len(PNG_SIGNATURE)] != PNG_SIGNATURE
    ):
        raise AssetPackageError(f"Blender did not produce a valid PNG: {output}")
    return {
        "asset_dir": str(package.root),
        "camera_distance": float(camera_distance),
        "device": device,
        "geometry": {
            "mesh_visual_count": imported_mesh_files,
            "primitive_counts": primitive_counts,
        },
        "materials": len(material_cache),
        "objects": len(imported),
        "output": str(output),
        "radius": float(radius),
        "root_link": package.root_link,
        "shared_renderer": shared_renderer_receipt,
        "visuals": len(package.visuals),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--shared-renderer", type=Path, default=DEFAULT_SHARED_RENDERER)
    parser.add_argument("--shared-renderer-sha256")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    args = build_parser().parse_args(argv)
    try:
        package = load_asset_package(args.asset_dir)
        result = render_asset(
            package,
            args.output,
            resolution=args.resolution,
            samples=args.samples,
            shared_renderer=args.shared_renderer,
            shared_renderer_sha256=args.shared_renderer_sha256,
        )
    except (OSError, ValueError, AssetPackageError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    print(__import__("json").dumps(result, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
