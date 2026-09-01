"""Deterministic Blender Eevee worker for the visual evaluation pilot.

The template-side job builder exports one closed-pose object and four camera
poses.  This worker renders each pose twice: once with the template material
and once with a world-space normal emission material.  Keeping both passes in
one Blender process guarantees identical camera, visibility, and transforms.
"""

from __future__ import annotations

import json
import math
import os
import sys

import bpy
import mathutils


def _job_dir() -> str:
    if "--" not in sys.argv:
        raise SystemExit("expected: blender -b -P blender_worker.py -- <job_dir>")
    return sys.argv[sys.argv.index("--") + 1]


def _mat(flat16: list[float]) -> mathutils.Matrix:
    return mathutils.Matrix([flat16[0:4], flat16[4:8], flat16[8:12], flat16[12:16]])


def _srgb_to_linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _reset() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)


def _set_cycles(scene: bpy.types.Scene, samples: int) -> None:
    scene.render.engine = "CYCLES"
    scene.cycles.samples = max(1, int(samples))
    scene.cycles.use_denoising = True
    try:
        scene.cycles.denoiser = "OPENIMAGEDENOISE"
    except (AttributeError, TypeError, ValueError):
        pass
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    scene.render.resolution_percentage = 100
    scene.view_settings.view_transform = "Standard"
    try:
        scene.view_settings.look = "None"
    except (TypeError, ValueError):
        pass


def _setup_world(scene: bpy.types.Scene, spec: dict) -> None:
    world = bpy.data.worlds.new("eval_world")
    scene.world = world
    world.use_nodes = True
    color = spec.get("bg_color", [0.91, 0.93, 0.96])
    nodes = world.node_tree.nodes
    links = world.node_tree.links
    nodes.clear()
    output = nodes.new("ShaderNodeOutputWorld")
    mix = nodes.new("ShaderNodeMixShader")
    light_path = nodes.new("ShaderNodeLightPath")
    camera_background = nodes.new("ShaderNodeBackground")
    fill_background = nodes.new("ShaderNodeBackground")
    camera_background.inputs["Color"].default_value = (*color, 1.0)
    camera_background.inputs["Strength"].default_value = float(spec.get("bg_strength", 1.0))
    fill_background.inputs["Color"].default_value = (1.0, 1.0, 1.0, 1.0)
    fill_background.inputs["Strength"].default_value = float(spec.get("fill_strength", 0.2))
    links.new(light_path.outputs["Is Camera Ray"], mix.inputs[0])
    links.new(fill_background.outputs["Background"], mix.inputs[1])
    links.new(camera_background.outputs["Background"], mix.inputs[2])
    links.new(mix.outputs["Shader"], output.inputs["Surface"])


def _look_at(obj: bpy.types.Object, target: list[float]) -> None:
    obj.rotation_euler = (mathutils.Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()


def _add_lights(scene: bpy.types.Scene, center: list[float], lights: list[dict]) -> None:
    for index, light in enumerate(lights):
        data = bpy.data.lights.new(f"eval_area_{index}", type="AREA")
        data.energy = float(light["energy"])
        data.shape = "DISK"
        data.size = float(light.get("size", 1.0))
        obj = bpy.data.objects.new(f"eval_area_{index}", data)
        scene.collection.objects.link(obj)
        obj.location = light["loc"]
        _look_at(obj, center)


def _add_camera(scene: bpy.types.Scene, job: dict) -> bpy.types.Object:
    center = job["camera"]["center"]
    data = bpy.data.cameras.new("eval_camera")
    data.sensor_fit = "VERTICAL"
    data.angle_y = math.radians(float(job["camera"]["angle_y_deg"]))
    poses = job["camera"]["poses"]
    distances = [math.dist(pose[3:16:4][:3], center) for pose in poses]
    orbit_radius = min(distances) if distances else 1.0
    data.clip_start = max(1e-5, 1e-3 * orbit_radius)
    data.clip_end = max(1e-2, 1e3 * orbit_radius)
    camera = bpy.data.objects.new("eval_camera", data)
    scene.collection.objects.link(camera)
    scene.camera = camera
    return camera


def _add_ground(scene: bpy.types.Scene, job: dict) -> bpy.types.Object | None:
    spec = job.get("ground", {})
    if not spec.get("enabled", False):
        return None
    center = job["camera"]["center"]
    size = float(spec.get("size", 10.0))
    z = float(spec.get("z", 0.0))
    mesh = bpy.data.meshes.new("eval_ground_mesh")
    mesh.from_pydata(
        [
            (center[0] - size, center[1] - size, z),
            (center[0] + size, center[1] - size, z),
            (center[0] + size, center[1] + size, z),
            (center[0] - size, center[1] + size, z),
        ],
        [],
        [(0, 1, 2, 3)],
    )
    mesh.update()
    obj = bpy.data.objects.new("eval_ground", mesh)
    scene.collection.objects.link(obj)
    material = bpy.data.materials.new("eval_ground_material")
    material.diffuse_color = (0.82, 0.84, 0.88, 1.0)
    material.use_nodes = True
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    if bsdf is not None:
        bsdf.inputs["Base Color"].default_value = (0.82, 0.84, 0.88, 1.0)
        bsdf.inputs["Roughness"].default_value = 0.9
    mesh.materials.append(material)
    return obj


def _material(name: str, spec: dict) -> bpy.types.Material:
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")

    def setv(key: str, value) -> None:
        if bsdf is not None and key in bsdf.inputs:
            bsdf.inputs[key].default_value = value

    base = tuple(float(v) for v in spec.get("base_color", (0.74, 0.74, 0.72)))
    setv("Base Color", (*(_srgb_to_linear(v) for v in base), 1.0))
    setv("Metallic", float(spec.get("metallic", 0.0)))
    setv("Roughness", float(spec.get("roughness", 0.5)))
    setv("IOR", float(spec.get("ior", 1.45)))
    opacity = float(spec.get("opacity", 1.0))
    setv("Alpha", opacity)
    transmission = float(spec.get("transmission", 0.0))
    setv("Transmission Weight", transmission)
    setv("Transmission", transmission)
    setv("Coat Weight", float(spec.get("clearcoat", 0.0)))
    setv("Clearcoat", float(spec.get("clearcoat", 0.0)))
    setv("Coat Roughness", float(spec.get("clearcoat_roughness", 0.2)))
    if opacity < 0.999 or transmission > 0.01:
        try:
            mat.surface_render_method = "DITHERED"
        except (AttributeError, TypeError):
            pass
    return mat


def _normal_material() -> bpy.types.Material:
    mat = bpy.data.materials.new("eval_world_space_normal")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()
    output = nodes.new("ShaderNodeOutputMaterial")
    emission = nodes.new("ShaderNodeEmission")
    geometry = nodes.new("ShaderNodeNewGeometry")
    scale = nodes.new("ShaderNodeVectorMath")
    scale.operation = "SCALE"
    scale.inputs["Scale"].default_value = 0.5
    add = nodes.new("ShaderNodeVectorMath")
    add.operation = "ADD"
    add.inputs[1].default_value = (0.5, 0.5, 0.5)
    links.new(geometry.outputs["Normal"], scale.inputs[0])
    links.new(scale.outputs[0], add.inputs[0])
    links.new(add.outputs[0], emission.inputs["Color"])
    emission.inputs["Strength"].default_value = 1.0
    links.new(emission.outputs[0], output.inputs["Surface"])
    return mat


def _import_visual(job_dir: str, visual: dict) -> bpy.types.Object:
    before = set(bpy.context.scene.objects)
    bpy.ops.wm.ply_import(filepath=os.path.join(job_dir, visual["mesh"]))
    new = [obj for obj in bpy.context.scene.objects if obj not in before]
    if not new:
        raise RuntimeError(f"Blender imported no object for {visual['mesh']}")
    obj = new[0]
    obj.data.materials.clear()
    obj.data.materials.append(_material(visual["mesh"], visual["spec"]))
    for polygon in obj.data.polygons:
        polygon.use_smooth = False
    return obj


def _render_pass(
    scene: bpy.types.Scene,
    camera: bpy.types.Object,
    objects: list[tuple[bpy.types.Object, int, bpy.types.Material]],
    normal: bpy.types.Material,
    job: dict,
    prefix: str,
    output_dir: str,
) -> None:
    scene.render.film_transparent = False
    if prefix == "normal":
        scene.view_settings.view_transform = "Standard"
        try:
            scene.view_settings.look = "None"
        except (TypeError, ValueError):
            pass
    else:
        scene.view_settings.view_transform = "AgX"
        try:
            scene.view_settings.look = "AgX - Medium High Contrast"
        except (TypeError, ValueError):
            pass
    for obj, _, rgb_material in objects:
        obj.data.materials[0] = normal if prefix == "normal" else rgb_material
    for index, pose in enumerate(job["camera"]["poses"]):
        for obj, part_index, _ in objects:
            obj.matrix_world = _mat(job["parts_world"][part_index][0])
        camera.matrix_world = _mat(pose)
        scene.render.filepath = os.path.join(output_dir, f"{prefix}_{index:02d}.png")
        bpy.ops.render.render(write_still=True)


def main() -> None:
    job_dir = _job_dir()
    with open(os.path.join(job_dir, "job.json"), encoding="utf-8") as handle:
        job = json.load(handle)

    _reset()
    scene = bpy.context.scene
    _set_cycles(scene, int(job["render"].get("samples", 8)))
    scene.render.resolution_x = int(job["render"]["width"])
    scene.render.resolution_y = int(job["render"]["height"])
    _setup_world(scene, job.get("world", {}))
    _add_lights(scene, job["camera"]["center"], job.get("lights", []))
    ground = _add_ground(scene, job)
    camera = _add_camera(scene, job)
    objects = []
    for visual in job["visuals"]:
        obj = _import_visual(job_dir, visual)
        objects.append((obj, int(visual["part"]), obj.data.materials[0]))
    normal = _normal_material()
    output_dir = os.path.join(job_dir, "frames")
    os.makedirs(output_dir, exist_ok=True)
    _render_pass(scene, camera, objects, normal, job, "rgb", output_dir)
    if ground is not None:
        ground.hide_render = True
    _render_pass(scene, camera, objects, normal, job, "normal", output_dir)
    print(f"[eval-blender] rendered {len(job['camera']['poses'])} RGB + normal views -> {output_dir}")


if __name__ == "__main__":
    main()
