#!/usr/bin/env python3
"""Canonicalize Table 4 URDF/GLB artifacts while preserving semantic names."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np
import trimesh


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
EXP_ROOT = Path(__file__).resolve().parents[1]
PLACEHOLDER_RE = re.compile(r"^(?:visual|mesh|geometry|object|part|link)(?:[_-]?\d+)?$", re.I)


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def floats(value: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if not value:
        return default
    parsed = tuple(float(item) for item in value.split())
    if len(parsed) == 1 and len(default) == 3:
        return parsed * 3
    if len(parsed) != len(default):
        raise ValueError(f"expected {len(default)} values, got {value!r}")
    return parsed


def origin_matrix(node: ET.Element | None) -> np.ndarray:
    if node is None:
        return np.eye(4)
    xyz = floats(node.attrib.get("xyz"), (0.0, 0.0, 0.0))
    roll, pitch, yaw = floats(node.attrib.get("rpy"), (0.0, 0.0, 0.0))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    matrix[:3, 3] = xyz
    return matrix


def load_mesh(path: Path) -> trimesh.Trimesh:
    loaded = trimesh.load(contained(path), force="scene", process=False)
    if isinstance(loaded, trimesh.Trimesh):
        return loaded.copy()
    meshes = [mesh.copy() for mesh in loaded.geometry.values() if isinstance(mesh, trimesh.Trimesh)]
    if not meshes:
        raise ValueError(f"no triangular geometry: {path}")
    return trimesh.util.concatenate(meshes)


def geometry_mesh(geometry: ET.Element, package: Path) -> trimesh.Trimesh:
    children = list(geometry)
    if len(children) != 1:
        raise ValueError("geometry must have exactly one shape")
    shape = children[0]
    if shape.tag == "box":
        return trimesh.creation.box(extents=floats(shape.attrib.get("size"), (1.0, 1.0, 1.0)))
    if shape.tag == "cylinder":
        return trimesh.creation.cylinder(
            radius=float(shape.attrib["radius"]), height=float(shape.attrib["length"]), sections=48
        )
    if shape.tag == "sphere":
        return trimesh.creation.icosphere(subdivisions=3, radius=float(shape.attrib["radius"]))
    if shape.tag == "mesh":
        reference = shape.attrib["filename"]
        if reference.startswith("package://"):
            reference = reference[len("package://") :]
            candidates = [package / reference, package / Path(reference).name]
        else:
            candidate = Path(reference)
            candidates = [candidate if candidate.is_absolute() else package / candidate]
        path = next((contained(candidate) for candidate in candidates if contained(candidate).is_file()), None)
        if path is None:
            raise FileNotFoundError(f"mesh reference not found: {shape.attrib['filename']}")
        mesh = load_mesh(path)
        scale = floats(shape.attrib.get("scale"), (1.0, 1.0, 1.0))
        matrix = np.eye(4)
        matrix[0, 0], matrix[1, 1], matrix[2, 2] = scale
        mesh.apply_transform(matrix)
        return mesh
    raise ValueError(f"unsupported URDF geometry: {shape.tag}")


def canonicalize_urdf(source: Path, unit_scale_to_m: float) -> tuple[trimesh.Scene, list[dict[str, Any]]]:
    root = ET.parse(source).getroot()
    package = source.parent
    links = {link.attrib["name"]: link for link in root.findall("link")}
    parents: dict[str, tuple[str, np.ndarray]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"joint lacks parent/child: {joint.attrib.get('name')}")
        child_name = child.attrib["link"]
        if child_name in parents:
            raise ValueError(f"multiple parents for link: {child_name}")
        parents[child_name] = (parent.attrib["link"], origin_matrix(joint.find("origin")))
    roots = sorted(set(links) - set(parents))
    if not roots:
        raise ValueError("URDF has no root link")

    scene = trimesh.Scene(base_frame="__table4_world__")
    semantic_nodes: list[dict[str, Any]] = []
    pending = set(links)
    added: set[str] = set()
    while pending:
        progressed = False
        for link_name in sorted(pending):
            if link_name in parents:
                parent_name, link_matrix = parents[link_name]
                if parent_name not in added:
                    continue
                link_matrix = link_matrix.copy()
                link_matrix[:3, 3] *= unit_scale_to_m
            else:
                parent_name, link_matrix = "__table4_world__", np.eye(4)
            link_frame = f"link::{link_name}"
            parent_frame = f"link::{parent_name}" if parent_name in links else parent_name
            scene.graph.update(frame_from=parent_frame, frame_to=link_frame, matrix=link_matrix)
            visuals = link_name and links[link_name].findall("visual") or []
            if visuals:
                semantic_nodes.append({
                    "level": "link",
                    "name": link_name,
                    "frame": link_frame,
                    "mesh_bearing": True,
                })
            for index, visual in enumerate(visuals):
                geometry = visual.find("geometry")
                if geometry is None:
                    continue
                mesh = geometry_mesh(geometry, package)
                mesh.apply_scale(unit_scale_to_m)
                visual_name = visual.attrib.get("name") or ""
                primary_name = visual_name if visual_name and not PLACEHOLDER_RE.fullmatch(visual_name) else link_name
                frame = f"{link_frame}::visual::{index:04d}::{primary_name}"
                geometry_name = f"geometry::{link_name}::{index:04d}"
                scene.geometry[geometry_name] = mesh
                visual_matrix = origin_matrix(visual.find("origin"))
                visual_matrix[:3, 3] *= unit_scale_to_m
                scene.graph.update(
                    frame_from=link_frame,
                    frame_to=frame,
                    matrix=visual_matrix,
                    geometry=geometry_name,
                )
                semantic_nodes.append({
                    "level": "visual",
                    "name": primary_name,
                    "raw_visual_name": visual_name or None,
                    "link_name": link_name,
                    "frame": frame,
                    "mesh_bearing": True,
                })
            pending.remove(link_name)
            added.add(link_name)
            progressed = True
        if not progressed:
            raise ValueError(f"unresolved URDF hierarchy: {sorted(pending)}")
    return scene, semantic_nodes


def canonicalize_glb(source: Path, unit_scale_to_m: float) -> tuple[trimesh.Scene, list[dict[str, Any]]]:
    loaded = trimesh.load(source, force="scene", process=False)
    scene = loaded if isinstance(loaded, trimesh.Scene) else trimesh.Scene(loaded)
    scene = scene.copy()
    if unit_scale_to_m != 1.0:
        matrix = np.eye(4)
        matrix[0, 0] = matrix[1, 1] = matrix[2, 2] = unit_scale_to_m
        scene.apply_transform(matrix)
    semantic_nodes = []
    for node_name in sorted(scene.graph.nodes_geometry):
        semantic_nodes.append({
            "level": "glb_node",
            "name": str(node_name),
            "frame": str(node_name),
            "mesh_bearing": True,
        })
    return scene, semantic_nodes


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--artifact-type", choices=("urdf", "glb"), required=True)
    parser.add_argument("--unit-scale-to-m", type=float, default=1.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    source = contained(args.input)
    output = contained(args.output_dir)
    if args.unit_scale_to_m <= 0 or not math.isfinite(args.unit_scale_to_m):
        raise ValueError("unit scale must be finite and positive")
    output.mkdir(parents=True, exist_ok=True)
    if args.artifact_type == "urdf":
        scene, semantic_nodes = canonicalize_urdf(source, args.unit_scale_to_m)
    else:
        scene, semantic_nodes = canonicalize_glb(source, args.unit_scale_to_m)
    if not scene.geometry:
        raise ValueError("canonical artifact has no geometry")
    glb = output / "artifact.glb"
    glb.write_bytes(scene.export(file_type="glb"))
    bounds = np.asarray(scene.bounds, dtype=float)
    extents = bounds[1] - bounds[0]
    nodes_path = output / "semantic_nodes.json"
    nodes_path.write_text(json.dumps(semantic_nodes, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    artifact = {
        "schema_version": 1,
        "source": str(source),
        "source_sha256": sha256(source),
        "source_type": args.artifact_type,
        "unit_scale_to_m": args.unit_scale_to_m,
        "canonical_glb": str(glb),
        "canonical_glb_sha256": sha256(glb),
        "semantic_nodes": str(nodes_path),
        "semantic_node_count": len(semantic_nodes),
        "bounds_m": bounds.tolist(),
        "extents_m": extents.tolist(),
        "z_extent_m": float(extents[2]),
        "horizontal_max_m": float(max(extents[0], extents[1])),
        "horizontal_min_m": float(min(extents[0], extents[1])),
    }
    (output / "artifact.json").write_text(
        json.dumps(artifact, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(artifact, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
