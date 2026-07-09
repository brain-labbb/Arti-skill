from __future__ import annotations

import argparse
import importlib
import math
import os
from pathlib import Path
from typing import Iterable

import numpy as np
import trimesh
from PIL import Image, ImageDraw

from cli.template import TEMPLATE_REGISTRY
from sdk import AssetContext, Box, Cylinder, Mesh, Sphere
from sdk._core.v0.assets import resolve_mesh_path
from sdk._core.v0.geometry_qc import _mat4_mul, _origin_to_mat4, compute_part_world_transforms

os.environ.setdefault("PYOPENGL_PLATFORM", "egl")


DEFAULT_SLUGS = (
    "barrier_gate_boom",
    "barrier_gate_leaf_gate",
    "bicycle_crankset_and_pedal_assembly",
    "blender_countertop",
    "immersion_blender",
    "box_fan_with_control_knob",
    "camcorder_with_flipout_screen",
    "camera_flash",
    "camera_lens",
    "cannon",
    "cantilever_articulated_arm",
    "car_sunroof_cassette_soft_top",
    "cctv_mast_with_pantilt_camera_head",
    "ceiling_fan",
    "ceiling_light_fixture_adjustable",
    "chest_freezer_with_hinged_lid",
    "coaxial_rotary_stack",
    "crane_tower",
    "desk_with_drawer",
    "desk_with_drawer_card_catalog",
    "desktop_monitor_with_tilt_swivel_stand",
    "desktop_pc_tower",
)

VIEW_DIRECTIONS = (
    ("front_iso", np.array([1.25, -1.35, 0.85], dtype=float)),
    ("rear_iso", np.array([-1.15, 1.25, 0.8], dtype=float)),
    ("side", np.array([0.1, -1.9, 0.45], dtype=float)),
)


def _parse_seeds(spec: str) -> list[int]:
    seeds: list[int] = []
    for chunk in spec.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "-" in chunk:
            start_s, end_s = chunk.split("-", 1)
            start = int(start_s)
            end = int(end_s)
            if end < start:
                raise ValueError(f"Invalid seed range: {chunk}")
            seeds.extend(range(start, end + 1))
        else:
            seeds.append(int(chunk))
    return seeds


def _matrix(value: object) -> np.ndarray:
    return np.asarray(value, dtype=float)


def _primitive_mesh(geometry: object) -> trimesh.Trimesh | None:
    if isinstance(geometry, Box):
        return trimesh.creation.box(extents=geometry.size)
    if isinstance(geometry, Cylinder):
        return trimesh.creation.cylinder(
            radius=geometry.radius, height=geometry.length, sections=40
        )
    if isinstance(geometry, Sphere):
        return trimesh.creation.uv_sphere(radius=geometry.radius, count=(24, 24))
    return None


def _mesh_geometry(geometry: object, *, assets: AssetContext | None) -> trimesh.Trimesh | None:
    if isinstance(geometry, Mesh):
        if geometry.source_geometry is not None:
            mesh = _primitive_mesh(geometry.source_geometry)
            if mesh is not None and geometry.source_transform is not None:
                mesh.apply_transform(_matrix(geometry.source_transform))
            return mesh
        path_text = geometry.materialized_path or geometry.filename
        if path_text:
            path = Path(path_text)
            if not path.is_absolute() and geometry.materialized_path is None:
                path = resolve_mesh_path(path, assets=assets, ensure_dir=False)
            if path.exists():
                loaded = trimesh.load(path, force="mesh")
                if isinstance(loaded, trimesh.Trimesh):
                    return loaded
    return _primitive_mesh(geometry)


def _rgba(material: object) -> tuple[float, float, float, float]:
    rgba = getattr(material, "rgba", None)
    if isinstance(rgba, tuple | list) and len(rgba) >= 3:
        values = tuple(float(v) for v in rgba)
        if len(values) == 3:
            return (values[0], values[1], values[2], 1.0)
        return (values[0], values[1], values[2], values[3])
    return (0.68, 0.70, 0.72, 1.0)


def _camera_pose(center: np.ndarray, radius: float, direction_scale: np.ndarray) -> np.ndarray:
    pose = np.eye(4)
    pose[:3, 3] = center + direction_scale * radius
    direction = center - pose[:3, 3]
    direction = direction / np.linalg.norm(direction)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(direction, up)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    right = right / np.linalg.norm(right)
    up = np.cross(right, direction)
    pose[:3, :3] = np.stack([right, up, -direction], axis=1)
    return pose


def _render_scene(
    meshes: Iterable[tuple[trimesh.Trimesh, tuple[float, float, float, float]]], out: Path
) -> None:
    import pyrender

    mesh_items = list(meshes)
    if not mesh_items:
        raise ValueError("No renderable meshes")

    scene = pyrender.Scene(bg_color=(0.02, 0.025, 0.03, 1.0), ambient_light=(0.35, 0.35, 0.35))
    combined = trimesh.util.concatenate([mesh for mesh, _ in mesh_items])
    center = np.asarray(combined.bounding_box.centroid, dtype=float)
    radius = float(np.linalg.norm(combined.bounding_box.extents)) or 1.0

    for mesh, color in mesh_items:
        material = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=color,
            metallicFactor=0.0,
            roughnessFactor=0.65,
        )
        scene.add(pyrender.Mesh.from_trimesh(mesh, material=material, smooth=False))

    camera = pyrender.PerspectiveCamera(yfov=math.radians(42.0))
    pose = _camera_pose(center, radius, _render_scene.direction)  # type: ignore[attr-defined]
    scene.add(camera, pose=pose)
    scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=3.5), pose=pose)

    renderer = pyrender.OffscreenRenderer(900, 700)
    color, _depth = renderer.render(scene)
    renderer.delete()
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(color).save(out)


def _render_model(slug: str, seed: int, out_dir: Path) -> list[Path]:
    stem = TEMPLATE_REGISTRY[slug]
    module = importlib.import_module(f"agent.templates.{slug}")
    config = module.config_from_seed(seed)
    assets = AssetContext(out_dir / "assets" / f"{slug}_seed_{seed}")
    model = getattr(module, f"build_{stem}")(config, assets=assets)
    world = compute_part_world_transforms(model, {})

    meshes: list[tuple[trimesh.Trimesh, tuple[float, float, float, float]]] = []
    for part in model.parts:
        part_tf = world.get(part.name)
        if part_tf is None:
            continue
        for visual in part.visuals:
            mesh = _mesh_geometry(visual.geometry, assets=getattr(part, "assets", None) or assets)
            if mesh is None:
                continue
            mesh = mesh.copy()
            mesh.apply_transform(_matrix(_mat4_mul(part_tf, _origin_to_mat4(visual.origin))))
            meshes.append((mesh, _rgba(visual.material)))

    written: list[Path] = []
    for name, direction in VIEW_DIRECTIONS:
        _render_scene.direction = direction  # type: ignore[attr-defined]
        out = out_dir / slug / f"seed_{seed}_{name}.png"
        _render_scene(meshes, out)
        written.append(out)
    return written


def _contact_sheet(paths: list[Path], out: Path) -> None:
    thumbs: list[Image.Image] = []
    for path in paths:
        img = Image.open(path).convert("RGB")
        img.thumbnail((300, 234))
        canvas = Image.new("RGB", (300, 260), (18, 20, 24))
        canvas.paste(img, ((300 - img.width) // 2, 0))
        draw = ImageDraw.Draw(canvas)
        draw.text((8, 240), path.parent.name + "/" + path.stem, fill=(230, 230, 230))
        thumbs.append(canvas)

    cols = 3
    rows = math.ceil(len(thumbs) / cols)
    sheet = Image.new("RGB", (cols * 300, rows * 260), (18, 20, 24))
    for i, thumb in enumerate(thumbs):
        sheet.paste(thumb, ((i % cols) * 300, (i // cols) * 260))
    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--slugs", default=",".join(DEFAULT_SLUGS))
    parser.add_argument("--seeds", default="0-2")
    parser.add_argument("--out", type=Path, default=Path("template_preview_renders"))
    args = parser.parse_args()

    slugs = [slug.strip() for slug in args.slugs.split(",") if slug.strip()]
    seeds = _parse_seeds(args.seeds)
    missing = [slug for slug in slugs if slug not in TEMPLATE_REGISTRY]
    if missing:
        raise SystemExit(f"Unknown template slug(s): {', '.join(missing)}")

    all_paths: list[Path] = []
    for slug in slugs:
        for seed in seeds:
            paths = _render_model(slug, seed, args.out)
            all_paths.extend(paths)
            print(f"{slug} seed={seed}: {len(paths)} views")

    _contact_sheet(all_paths, args.out / "contact_sheet.png")
    print(args.out / "contact_sheet.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
