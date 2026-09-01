#!/usr/bin/env python3
"""Render one PartNet-Mobility v0 package in the frozen PV-A studio."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Sequence


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_BASE_RENDERER = REPO_ROOT / "exp/scripts/render_articraft10k_asset_blender.py"
DEFAULT_SHARED_RENDERER = REPO_ROOT / "arti-template/scripts/render_exported_asset_blender.py"


class PartNetRenderError(RuntimeError):
    """Raised when a PartNet package cannot be rendered without guessing."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_module(path: Path, expected_sha256: str, name: str) -> Any:
    path = path.expanduser().resolve(strict=True)
    actual = _sha256(path)
    if actual != expected_sha256:
        raise PartNetRenderError(
            f"support renderer SHA-256 mismatch: expected {expected_sha256}, found {actual}"
        )
    module_name = f"_{name}_{actual}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise PartNetRenderError(f"cannot import support renderer: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(module_name, None)
        raise
    return module


def load_partnet_package(asset_dir: Path, base: Any) -> Any:
    """Parse ``mobility.urdf`` using the audited generic URDF data types."""

    root = asset_dir.expanduser().resolve(strict=True)
    if not root.is_dir():
        raise PartNetRenderError(f"asset directory is not a directory: {root}")
    urdf_path = base._contained_file(root, "mobility.urdf", field="mobility.urdf")
    try:
        robot = ET.parse(urdf_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise PartNetRenderError(f"cannot parse mobility.urdf: {exc}") from exc
    if robot.tag != "robot":
        raise PartNetRenderError("mobility.urdf root must be <robot>")

    default_color = (0.62, 0.64, 0.68, 1.0)
    global_materials: dict[str, tuple[float, float, float, float]] = {}
    for material in robot.findall("material"):
        name = str(material.get("name") or "").strip()
        if name:
            global_materials[name] = base._rgba(
                material.find("color"), field=f"material {name!r}", default=default_color
            )

    links: list[str] = []
    visuals: list[Any] = []
    seen_links: set[str] = set()
    for link_element in robot.findall("link"):
        link_name = str(link_element.get("name") or "").strip()
        if not link_name or link_name in seen_links:
            raise PartNetRenderError(f"invalid or duplicate link {link_name!r}")
        seen_links.add(link_name)
        links.append(link_name)
        for visual_index, visual_element in enumerate(link_element.findall("visual")):
            visual_name = str(visual_element.get("name") or "").strip() or f"visual_{visual_index:04d}"
            material_element = visual_element.find("material")
            material_name = "__default__"
            color = default_color
            if material_element is not None:
                material_name = str(material_element.get("name") or "").strip() or "__unnamed__"
                inline_color = material_element.find("color")
                if inline_color is not None:
                    color = base._rgba(
                        inline_color,
                        field=f"visual {link_name}::{visual_name} material",
                        default=default_color,
                    )
                elif material_name in global_materials:
                    color = global_materials[material_name]

            geometry = visual_element.find("geometry")
            if geometry is None:
                raise PartNetRenderError(f"visual {link_name}::{visual_name} has no geometry")
            mesh = geometry.find("mesh")
            geometry_type = "mesh"
            geometry_params: tuple[float, ...] = ()
            mesh_path: Path | None = None
            mesh_scale = (1.0, 1.0, 1.0)
            if mesh is not None:
                mesh_path = base._contained_file(
                    root,
                    mesh.get("filename"),
                    field=f"visual {link_name}::{visual_name} mesh filename",
                )
                if mesh_path.suffix.lower() != ".obj":
                    raise PartNetRenderError(f"PartNet v0 visual is not OBJ: {mesh_path}")
                mesh_scale = base._triplet(
                    mesh.get("scale"),
                    field=f"visual {link_name}::{visual_name} mesh scale",
                    default=(1.0, 1.0, 1.0),
                )
                if any(value == 0.0 for value in mesh_scale):
                    raise PartNetRenderError("mesh scale cannot contain zero")
            else:
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                sphere = geometry.find("sphere")
                if box is not None:
                    geometry_type = "box"
                    geometry_params = base._triplet(
                        box.get("size"),
                        field=f"visual {link_name}::{visual_name} box size",
                        default=(0.0, 0.0, 0.0),
                    )
                elif cylinder is not None:
                    geometry_type = "cylinder"
                    geometry_params = (
                        float(cylinder.get("radius", "nan")),
                        float(cylinder.get("length", "nan")),
                    )
                elif sphere is not None:
                    geometry_type = "sphere"
                    geometry_params = (float(sphere.get("radius", "nan")),)
                else:
                    raise PartNetRenderError(
                        f"visual {link_name}::{visual_name} has unsupported geometry"
                    )
                if not all(math.isfinite(value) and value > 0.0 for value in geometry_params):
                    raise PartNetRenderError(
                        f"visual {link_name}::{visual_name} has invalid primitive dimensions"
                    )
            visuals.append(
                base.VisualAsset(
                    link_name=link_name,
                    visual_name=visual_name,
                    geometry_type=geometry_type,
                    geometry_params=geometry_params,
                    mesh_path=mesh_path,
                    mesh_scale=mesh_scale,
                    origin=base._origin(
                        visual_element.find("origin"),
                        field=f"visual {link_name}::{visual_name} origin",
                    ),
                    color=color,
                    material_name=material_name,
                )
            )
    if not visuals:
        raise PartNetRenderError("mobility.urdf contains no renderable visuals")

    joints: list[Any] = []
    seen_joints: set[str] = set()
    child_links: set[str] = set()
    for joint_element in robot.findall("joint"):
        name = str(joint_element.get("name") or "").strip()
        parent_element = joint_element.find("parent")
        child_element = joint_element.find("child")
        parent = "" if parent_element is None else str(parent_element.get("link") or "").strip()
        child = "" if child_element is None else str(child_element.get("link") or "").strip()
        if not name or name in seen_joints or parent not in seen_links or child not in seen_links:
            raise PartNetRenderError(f"joint {name!r} has an invalid parent or child")
        if child in child_links:
            raise PartNetRenderError(f"link {child!r} has more than one parent joint")
        seen_joints.add(name)
        child_links.add(child)
        joints.append(
            base.JointAsset(
                name=name,
                parent=parent,
                child=child,
                origin=base._origin(joint_element.find("origin"), field=f"joint {name!r} origin"),
            )
        )
    roots = [link for link in links if link not in child_links]
    if len(roots) != 1:
        raise PartNetRenderError(f"mobility.urdf must have one root link; found {roots!r}")
    return base.AssetPackage(
        root=root,
        root_link=roots[0],
        links=tuple(links),
        joints=tuple(joints),
        visuals=tuple(visuals),
    )


def _force_opaque(material: Any) -> None:
    material.use_nodes = True
    bsdf = material.node_tree.nodes.get("Principled BSDF") if material.node_tree else None
    if bsdf is not None and "Alpha" in bsdf.inputs:
        alpha = bsdf.inputs["Alpha"]
        for link in list(alpha.links):
            material.node_tree.links.remove(link)
        alpha.default_value = 1.0
    material.diffuse_color[3] = 1.0
    if hasattr(material, "surface_render_method"):
        material.surface_render_method = "DITHERED"
    elif hasattr(material, "blend_method"):
        material.blend_method = "OPAQUE"


def render_asset(
    package: Any,
    output: Path,
    *,
    resolution: int,
    samples: int,
    shared_renderer: Path,
    shared_renderer_sha256: str,
    base: Any,
    base_receipt: dict[str, str],
) -> dict[str, object]:
    try:
        import bpy
        import mathutils
    except ModuleNotFoundError as exc:  # pragma: no cover - Blender only
        raise PartNetRenderError("rendering requires Blender bpy") from exc
    if resolution < 64 or samples < 1:
        raise PartNetRenderError("resolution >= 64 and samples >= 1 are required")

    helpers, shared_receipt = base._shared_helpers(shared_renderer, shared_renderer_sha256)
    enable_cycles, scene_bounds, look_at, add_principled_material = helpers
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    device = enable_cycles(bpy, scene, samples)
    scene.render.resolution_x = resolution
    scene.render.resolution_y = resolution
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.film_transparent = False
    scene.render.use_file_extension = True
    scene.view_settings.view_transform = "AgX"
    scene.view_settings.look = "AgX - Medium High Contrast"

    world = bpy.data.worlds.new("PartNet Studio")
    world.use_nodes = True
    background = world.node_tree.nodes.get("Background")
    if background is None:
        raise PartNetRenderError("Blender world has no Background node")
    background.inputs["Color"].default_value = (0.80, 0.84, 0.90, 1.0)
    background.inputs["Strength"].default_value = 0.55
    scene.world = world

    matrices = base.rest_link_matrices(package)
    fallback_cache: dict[tuple[float, float, float, float, str], Any] = {}
    imported: list[Any] = []
    imported_mesh_files = 0
    native_materials: set[str] = set()
    primitive_counts = {"box": 0, "cylinder": 0, "sphere": 0}
    for visual in package.visuals:
        before = set(scene.objects)
        geometry_scale = mathutils.Matrix.Identity(4)
        if visual.geometry_type == "mesh":
            # PartNet OBJ vertices are already in the URDF coordinate frame.
            # Pin identity axis conversion instead of relying on Blender's
            # version-dependent OBJ defaults.
            result = bpy.ops.wm.obj_import(
                filepath=str(visual.mesh_path),
                forward_axis="Y",
                up_axis="Z",
            )
            if "FINISHED" not in result:
                raise PartNetRenderError(f"OBJ import did not finish: {visual.mesh_path}")
            imported_mesh_files += 1
        elif visual.geometry_type == "box":
            bpy.ops.mesh.primitive_cube_add(size=1.0, location=(0.0, 0.0, 0.0))
            geometry_scale = mathutils.Matrix.Diagonal((*visual.geometry_params, 1.0))
            primitive_counts["box"] += 1
        elif visual.geometry_type == "cylinder":
            radius, length = visual.geometry_params
            bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=radius, depth=length)
            primitive_counts["cylinder"] += 1
        elif visual.geometry_type == "sphere":
            (radius,) = visual.geometry_params
            bpy.ops.mesh.primitive_uv_sphere_add(segments=48, ring_count=24, radius=radius)
            primitive_counts["sphere"] += 1
        else:
            raise PartNetRenderError(f"unsupported geometry type: {visual.geometry_type}")

        objects = [obj for obj in scene.objects if obj not in before and obj.type == "MESH"]
        if not objects:
            raise PartNetRenderError(
                f"visual {visual.link_name}::{visual.visual_name} imported no mesh"
            )
        world_matrix = mathutils.Matrix(matrices[visual.link_name])
        visual_matrix = mathutils.Matrix(base._origin_matrix(visual.origin))
        scale_matrix = mathutils.Matrix.Diagonal((*visual.mesh_scale, 1.0))
        for index, obj in enumerate(objects):
            obj.name = (
                f"{visual.link_name}::{visual.visual_name}"
                if index == 0
                else f"{visual.link_name}::{visual.visual_name}::{index}"
            )
            obj.matrix_world = world_matrix @ visual_matrix @ geometry_scale @ scale_matrix
            if len(obj.data.materials) == 0:
                key = (*visual.color, visual.material_name)
                material = fallback_cache.get(key)
                if material is None:
                    material = base._color_material(
                        bpy, visual.color, f"{visual.material_name}::{len(fallback_cache):04d}"
                    )
                    fallback_cache[key] = material
                obj.data.materials.append(material)
            for material in obj.data.materials:
                if material is not None:
                    _force_opaque(material)
                    native_materials.add(material.name)
            imported.append(obj)

    minimum, maximum = scene_bounds(imported, mathutils)
    center = (minimum + maximum) * 0.5
    radius = max(float((maximum - minimum).length) * 0.5, 1e-4)
    bpy.ops.mesh.primitive_plane_add(
        size=radius * 12.0,
        location=(float(center.x), float(center.y), float(minimum.z) - radius * 0.015),
    )
    ground = bpy.context.object
    ground.name = "Studio Ground"
    ground.data.materials.append(
        add_principled_material(bpy, "Studio Ground", (0.32, 0.35, 0.40, 1.0), 0.82)
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
    look_at(camera, center, mathutils)
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
        look_at(light, center, mathutils)

    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(output)
    bpy.ops.render.render(write_still=True)
    if not output.is_file():
        raise PartNetRenderError(f"Blender did not create output: {output}")
    return {
        "asset_dir": str(package.root),
        "output": str(output),
        "device": device,
        "root_link": package.root_link,
        "visuals": len(package.visuals),
        "objects": len(imported),
        "geometry": {
            "mesh_visual_count": imported_mesh_files,
            "primitive_counts": primitive_counts,
        },
        "materials": len(native_materials),
        "material_policy": "native OBJ/MTL diffuse materials, forced opaque; neutral fallback",
        "radius": radius,
        "camera_distance": camera_distance,
        "base_renderer": base_receipt,
        "shared_renderer": shared_receipt,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--base-renderer", type=Path, default=DEFAULT_BASE_RENDERER)
    parser.add_argument("--base-renderer-sha256", required=True)
    parser.add_argument("--shared-renderer", type=Path, default=DEFAULT_SHARED_RENDERER)
    parser.add_argument("--shared-renderer-sha256", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    raw = sys.argv[sys.argv.index("--") + 1 :] if argv is None and "--" in sys.argv else argv
    args = _parser().parse_args(raw)
    try:
        base_path = args.base_renderer.expanduser().resolve(strict=True)
        base = _load_module(
            base_path, args.base_renderer_sha256, "partnet_articraft_renderer_support"
        )
        package = load_partnet_package(args.asset_dir, base)
        result = render_asset(
            package,
            args.output,
            resolution=args.resolution,
            samples=args.samples,
            shared_renderer=args.shared_renderer,
            shared_renderer_sha256=args.shared_renderer_sha256,
            base=base,
            base_receipt={"path": str(base_path), "sha256": args.base_renderer_sha256},
        )
    except Exception as exc:
        print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        return 1
    print(json.dumps(result, sort_keys=True, ensure_ascii=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
