#!/usr/bin/env python3
"""Render one Artiverse segmented GLB with the frozen PV-A studio setup.

Run this script through Blender, for example::

    blender -b --factory-startup -P render_artiverse_asset_blender.py -- \
        --glb /path/to/model.segmented.glb --output /path/to/000.png \
        --resolution 256 --samples 4

The glTF importer owns coordinate-system conversion and native material
creation.  This worker only supplies the same Cycles, world, ground, camera,
and area-light policy used by ``render_exported_asset_blender.py``.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Sequence


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
TEMPLATE_ROOT = REPO_ROOT / "arti-template"
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _shared_helpers() -> tuple[Any, Any, Any, Any]:
    """Import the audited studio helpers from the existing PV-A renderer."""

    if str(TEMPLATE_ROOT) not in sys.path:
        sys.path.insert(0, str(TEMPLATE_ROOT))
    from scripts.render_exported_asset_blender import (  # type: ignore[import-not-found]
        _add_principled_material,
        _enable_cycles,
        _look_at,
        _scene_bounds,
    )

    return _enable_cycles, _scene_bounds, _look_at, _add_principled_material


def render_glb(
    glb_path: Path,
    output: Path,
    *,
    resolution: int,
    samples: int,
) -> dict[str, object]:
    try:
        import bpy
        import mathutils
    except ModuleNotFoundError as exc:  # pragma: no cover - only Blender provides bpy
        raise RuntimeError("rendering requires Blender's bpy Python environment") from exc

    glb_path = glb_path.expanduser().resolve(strict=True)
    if not glb_path.is_file() or not glb_path.name.endswith(".segmented.glb"):
        raise ValueError(f"expected an existing *.segmented.glb file: {glb_path}")
    if resolution < 64 or samples < 1:
        raise ValueError("resolution >= 64 and samples >= 1 are required")

    _enable_cycles, _scene_bounds, _look_at, _add_principled_material = _shared_helpers()

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
        raise RuntimeError("Blender world has no Background node")
    background.inputs["Color"].default_value = (0.80, 0.84, 0.90, 1.0)
    background.inputs["Strength"].default_value = 0.55
    scene.world = world

    before = set(scene.objects)
    result = bpy.ops.import_scene.gltf(filepath=str(glb_path))
    if "FINISHED" not in result:
        raise RuntimeError(f"Blender glTF importer did not finish: {sorted(result)}")
    bpy.context.view_layer.update()
    new_objects = [obj for obj in scene.objects if obj not in before]
    imported = [obj for obj in new_objects if obj.type == "MESH"]
    imported_camera_count = sum(obj.type == "CAMERA" for obj in new_objects)
    imported_light_count = sum(obj.type == "LIGHT" for obj in new_objects)
    for obj in tuple(new_objects):
        if obj.type in {"CAMERA", "LIGHT"}:
            bpy.data.objects.remove(obj, do_unlink=True)
    rendered_objects = [obj for obj in imported if not obj.hide_render]
    if not rendered_objects:
        raise RuntimeError(f"segmented GLB imported no renderable mesh objects: {glb_path}")

    minimum, maximum = _scene_bounds(rendered_objects, mathutils)
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

    native_materials = {
        material.name
        for obj in rendered_objects
        for material in obj.data.materials
        if material is not None
    }
    native_images = {
        image.name
        for material_name in native_materials
        for material in [bpy.data.materials.get(material_name)]
        if material is not None and material.use_nodes and material.node_tree is not None
        for node in material.node_tree.nodes
        for image in [getattr(node, "image", None)]
        if image is not None
    }

    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(output)
    bpy.ops.render.render(write_still=True)
    if (
        not output.is_file()
        or output.stat().st_size <= len(PNG_SIGNATURE)
        or output.read_bytes()[: len(PNG_SIGNATURE)] != PNG_SIGNATURE
    ):
        raise RuntimeError(f"Blender did not produce a valid PNG: {output}")

    return {
        "bounds_max": [float(value) for value in maximum],
        "bounds_min": [float(value) for value in minimum],
        "camera_distance": float(camera_distance),
        "device": device,
        "glb": str(glb_path),
        "imported_cameras_removed": imported_camera_count,
        "imported_lights_removed": imported_light_count,
        "native_images": len(native_images),
        "native_materials": len(native_materials),
        "objects": len(rendered_objects),
        "output": str(output),
        "radius": float(radius),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--glb", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    args = build_parser().parse_args(argv)
    result = render_glb(
        args.glb,
        args.output,
        resolution=args.resolution,
        samples=args.samples,
    )
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
