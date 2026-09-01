#!/usr/bin/env python3
"""Render one PV-A diagnostic pose with collision-link highlighting.

This worker runs inside Blender.  It reuses the frozen PV-A appearance loader
and studio, then renders the exact joint vector supplied by the outer replay
driver.  The normal visual render, highlighted overview, and pair-only detail
are intentionally separate so a small contact is still inspectable.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Sequence


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
BASE_RENDERER = REPO_ROOT / "arti-template/scripts/render_exported_asset_blender.py"
DEFAULT_LIBRARY_ROOT = REPO_ROOT / "arti-template/pbr_material_library"
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _load_module(path: Path) -> Any:
    path = path.resolve(strict=True)
    spec = importlib.util.spec_from_file_location("_pva_p0_base_renderer", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import renderer: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _triplet(value: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if value is None or not value.strip():
        return default
    values = tuple(float(item) for item in value.split())
    if len(values) != 3 or not all(math.isfinite(item) for item in values):
        raise ValueError(f"invalid URDF triplet: {value!r}")
    return values  # type: ignore[return-value]


def _joint_specs(urdf_path: Path) -> dict[str, dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    result: dict[str, dict[str, Any]] = {}
    for element in root.findall("joint"):
        name = str(element.get("name") or "").strip()
        if not name:
            raise ValueError("URDF joint has no name")
        kind = str(element.get("type") or "fixed").strip().lower()
        axis_node = element.find("axis")
        axis = _triplet(
            None if axis_node is None else axis_node.get("xyz"),
            (1.0, 0.0, 0.0),
        )
        norm = math.sqrt(sum(item * item for item in axis))
        if norm <= 1e-12:
            raise ValueError(f"joint {name!r} has a zero axis")
        result[name] = {
            "type": kind,
            "axis": tuple(item / norm for item in axis),
        }
    return result


def _matrix_tuple(matrix: Any) -> tuple[tuple[float, float, float, float], ...]:
    return tuple(
        tuple(float(matrix[row][column]) for column in range(4))
        for row in range(4)
    )


def _posed_link_matrices(
    package: Any,
    base: Any,
    specs: dict[str, dict[str, Any]],
    values: dict[str, float],
) -> dict[str, tuple[tuple[float, float, float, float], ...]]:
    import mathutils

    identity = mathutils.Matrix.Identity(4)
    matrices: dict[str, Any] = {package.root_link: identity}
    pending = list(package.joints)
    while pending:
        remaining = []
        progressed = False
        for joint in pending:
            parent = matrices.get(joint.parent)
            if parent is None:
                remaining.append(joint)
                continue
            spec = specs[joint.name]
            kind = str(spec["type"])
            value = float(values.get(joint.name, 0.0))
            axis = mathutils.Vector(spec["axis"])
            if kind == "prismatic":
                motion = mathutils.Matrix.Translation(axis * value)
            elif kind in {"revolute", "continuous"}:
                motion = mathutils.Matrix.Rotation(value, 4, axis)
            else:
                motion = identity
            origin = mathutils.Matrix(base._origin_matrix(joint.origin))
            matrices[joint.child] = parent @ origin @ motion
            progressed = True
        if not progressed:
            names = ", ".join(joint.name for joint in remaining)
            raise RuntimeError(f"joint graph is cyclic or disconnected: {names}")
        pending = remaining
    missing = sorted(set(package.links) - set(matrices))
    if missing:
        raise RuntimeError(f"missing posed link matrices: {missing[:5]}")
    return {name: _matrix_tuple(matrix) for name, matrix in matrices.items()}


def _emission_material(
    bpy: Any,
    name: str,
    color: tuple[float, float, float, float],
    *,
    metallic: float = 0.0,
    emission_strength: float = 0.35,
) -> Any:
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    bsdf = material.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = color
    bsdf.inputs["Roughness"].default_value = 0.28
    if "Metallic" in bsdf.inputs:
        bsdf.inputs["Metallic"].default_value = metallic
    emission = bsdf.inputs.get("Emission Color") or bsdf.inputs.get("Emission")
    if emission is not None:
        emission.default_value = color
    strength = bsdf.inputs.get("Emission Strength")
    if strength is not None:
        strength.default_value = emission_strength
    return material


def _objects_for_link(package: Any, imported: Sequence[Any], link_name: str) -> list[Any]:
    keys = {
        visual.surface_key
        for visual in package.visuals
        if visual.link_name == link_name
    }
    return [
        obj
        for obj in imported
        if any(
            obj.name == key
            or obj.name.startswith(f"{key}::")
            or obj.name.startswith(f"{key}.")
            for key in keys
        )
        # Blender truncates long ID names.  The link prefix is short and remains
        # stable even when the full appearance surface key does not.
        or obj.name.startswith(f"{link_name}::")
    ]


def _mesh_objects(bpy: Any, package: Any) -> list[Any]:
    keys = {visual.surface_key for visual in package.visuals}
    link_prefixes = {f"{visual.link_name}::" for visual in package.visuals}
    return [
        obj
        for obj in bpy.context.scene.objects
        if obj.type == "MESH"
        and (
            any(
                obj.name == key
                or obj.name.startswith(f"{key}::")
                or obj.name.startswith(f"{key}.")
                for key in keys
            )
            or any(obj.name.startswith(prefix) for prefix in link_prefixes)
        )
    ]


def _valid_png(path: Path) -> bool:
    return (
        path.is_file()
        and path.stat().st_size > len(PNG_SIGNATURE)
        and path.read_bytes()[: len(PNG_SIGNATURE)] == PNG_SIGNATURE
    )


def _render(scene: Any, bpy: Any, path: Path) -> None:
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)
    if not _valid_png(path):
        raise RuntimeError(f"Blender did not write a valid PNG: {path}")


def _reframe(camera: Any, objects: Sequence[Any], base: Any, mathutils: Any, floor: float) -> None:
    minimum, maximum = base._scene_bounds(objects, mathutils)
    center = (minimum + maximum) * 0.5
    radius = max(float((maximum - minimum).length) * 0.5, floor)
    direction = mathutils.Vector((1.25, -1.35, 0.85)).normalized()
    distance = radius / math.sin(camera.data.angle_y * 0.5) * 1.28
    camera.location = center + direction * distance
    camera.data.clip_start = max(1e-7, distance * 1e-5)
    camera.data.clip_end = max(1.0, distance * 100.0)
    base._look_at(camera, center, mathutils)


def render(args: argparse.Namespace) -> dict[str, Any]:
    try:
        import bpy
        import mathutils
    except ModuleNotFoundError as exc:
        raise RuntimeError("this worker must run inside Blender") from exc

    base = _load_module(BASE_RENDERER)
    asset_dir = args.asset_dir.expanduser().resolve(strict=True)
    pose_path = args.pose_json.expanduser().resolve(strict=True)
    pose = json.loads(pose_path.read_text(encoding="utf-8"))
    package = base.load_asset_package(
        asset_dir,
        library_root=args.library_root.expanduser().resolve(strict=True),
    )
    urdf_path = asset_dir / "model.urdf"
    specs = _joint_specs(urdf_path)
    joint_names = list(pose["joint_names"])
    joint_values = [float(value) for value in pose["joint_values"]]
    if len(joint_names) != len(joint_values) or len(joint_names) != len(set(joint_names)):
        raise ValueError("pose joint vector is malformed")
    unknown = sorted(set(joint_names) - set(specs))
    if unknown:
        raise ValueError(f"pose references unknown joints: {unknown[:5]}")
    value_by_name = dict(zip(joint_names, joint_values, strict=True))
    matrices = _posed_link_matrices(package, base, specs, value_by_name)

    original_rest = base.rest_link_matrices
    base.rest_link_matrices = lambda loaded: matrices
    try:
        base_result = base.render_asset(
            package,
            args.raw_output,
            resolution=args.resolution,
            samples=args.samples,
        )
    finally:
        base.rest_link_matrices = original_rest

    scene = bpy.context.scene
    imported = _mesh_objects(bpy, package)
    if not imported:
        raise RuntimeError("base renderer left no identifiable asset mesh objects")
    overall_minimum, overall_maximum = base._scene_bounds(imported, mathutils)
    overall_radius = max(float((overall_maximum - overall_minimum).length) * 0.5, 1e-4)

    pair = pose["primary_pair"]
    link_a = str(pair["link_a_name"])
    link_b = str(pair["link_b_name"])
    objects_a = _objects_for_link(package, imported, link_a)
    objects_b = _objects_for_link(package, imported, link_b)
    if not objects_a or not objects_b:
        raise RuntimeError(
            f"cannot map diagnostic pair to visual objects: {link_a!r}={len(objects_a)}, "
            f"{link_b!r}={len(objects_b)}"
        )

    material_a = _emission_material(bpy, "Problem link A", (0.82, 0.035, 0.045, 1.0), metallic=0.05)
    material_b = _emission_material(bpy, "Problem link B", (1.0, 0.31, 0.025, 1.0), metallic=0.05)
    for obj in objects_a:
        obj.data.materials.clear()
        obj.data.materials.append(material_a)
    for obj in objects_b:
        obj.data.materials.clear()
        obj.data.materials.append(material_b)

    secondary_material = _emission_material(
        bpy, "Secondary problem link", (0.59, 0.12, 0.84, 1.0)
    )
    primary_links = {link_a, link_b}
    for secondary in pose.get("secondary_pairs", []):
        for link in (str(secondary["link_a_name"]), str(secondary["link_b_name"])):
            if link in primary_links:
                continue
            for obj in _objects_for_link(package, imported, link):
                obj.data.materials.clear()
                obj.data.materials.append(secondary_material)

    marker_material = _emission_material(
        bpy,
        "Contact marker",
        (1.0, 0.0, 0.72, 1.0),
        emission_strength=4.0,
    )
    halo_material = _emission_material(
        bpy,
        "Contact marker halo",
        (1.0, 1.0, 1.0, 1.0),
        emission_strength=3.0,
    )
    marker_radius = max(overall_radius * 0.014, 2e-5)
    camera_direction = mathutils.Vector((1.25, -1.35, 0.85)).normalized()
    markers: list[Any] = []
    frame_markers: list[Any] = []
    for index, point in enumerate(pair.get("contact_points", [])[:3]):
        location = mathutils.Vector(tuple(float(value) for value in point))
        location += camera_direction * marker_radius * 2.2
        bpy.ops.mesh.primitive_uv_sphere_add(
            segments=32,
            ring_count=16,
            radius=marker_radius * 1.45,
            location=location,
        )
        halo = bpy.context.object
        halo.name = f"Contact marker halo {index}"
        halo.data.materials.append(halo_material)
        markers.append(halo)
        frame_markers.append(halo)
        bpy.ops.mesh.primitive_uv_sphere_add(
            segments=32,
            ring_count=16,
            radius=marker_radius,
            location=location + camera_direction * marker_radius * 0.8,
        )
        marker = bpy.context.object
        marker.name = f"Contact marker {index}"
        marker.data.materials.append(marker_material)
        markers.append(marker)

    camera = scene.camera
    if camera is None:
        raise RuntimeError("base renderer did not create a camera")
    _render(scene, bpy, args.overview_output)

    detail_objects = list(dict.fromkeys([*objects_a, *objects_b, *markers]))
    for obj in imported:
        if obj not in detail_objects:
            obj.hide_render = True
    for obj in scene.objects:
        if obj.name == "Studio Ground":
            obj.hide_render = True
    _reframe(
        camera,
        frame_markers or detail_objects,
        base,
        mathutils,
        overall_radius * 0.055,
    )
    _render(scene, bpy, args.detail_output)

    return {
        "asset_dir": str(asset_dir),
        "base_render": base_result,
        "detail_object_count": len(detail_objects),
        "highlight_link_a": link_a,
        "highlight_link_b": link_b,
        "joint_count": len(joint_names),
        "marker_count": len(markers),
        "outputs": {
            "raw": str(args.raw_output.resolve()),
            "overview": str(args.overview_output.resolve()),
            "detail": str(args.detail_output.resolve()),
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-dir", type=Path, required=True)
    parser.add_argument("--pose-json", type=Path, required=True)
    parser.add_argument("--raw-output", type=Path, required=True)
    parser.add_argument("--overview-output", type=Path, required=True)
    parser.add_argument("--detail-output", type=Path, required=True)
    parser.add_argument("--library-root", type=Path, default=DEFAULT_LIBRARY_ROOT)
    parser.add_argument("--resolution", type=int, default=512)
    parser.add_argument("--samples", type=int, default=8)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else argv
    args = build_parser().parse_args(raw)
    result = render(args)
    print(json.dumps(result, sort_keys=True, ensure_ascii=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
