#!/usr/bin/env python3
"""Deterministic URDF-to-GLB Naming alignment pilot over exp-local packages."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from pygltflib import GLTF2


EXP_ROOT = Path("/mnt/zsn/lyb/arti-skill/exp").resolve()
INPUT_ROOT = EXP_ROOT / "runtime/nano3d_glb_pilot/input_packages"
OUTPUT_ROOT = EXP_ROOT / "runtime/nano3d_glb_pilot/output"
BASE_FRAME = "__urdf_world__"
PLACEHOLDER_RE = re.compile(r"^(?:link|part|mesh|geometry|object)(?:[_-]?(?:\d+|new|object))?$", re.I)


def exp_path(value: str | Path) -> Path:
    path = Path(value).resolve()
    if path != EXP_ROOT and EXP_ROOT not in path.parents:
        raise ValueError(f"write path must stay inside {EXP_ROOT}: {path}")
    return path


def prepare_input_packages(manifest: Path, input_root: Path) -> list[dict[str, Any]]:
    rows = [json.loads(line) for line in manifest.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows:
        raise RuntimeError(f"empty asset manifest: {manifest}")
    input_root.mkdir(parents=True, exist_ok=True)
    prepared = []
    for row in rows:
        source = Path(row["asset_path"]).resolve()
        destination = input_root / row["asset_id"]
        source_urdf = source / "model.urdf"
        if not source_urdf.exists() or not (source / "assets").is_dir():
            raise FileNotFoundError(f"incomplete source package: {source}")
        expected_hash = row.get("model_urdf_sha256")
        actual_hash = sha256(source_urdf)
        if expected_hash and actual_hash != expected_hash:
            raise RuntimeError(f"source URDF changed since manifest: {source}")
        if destination.exists():
            copied_urdf = destination / "model.urdf"
            if not copied_urdf.exists() or sha256(copied_urdf) != actual_hash:
                raise RuntimeError(f"existing exp-local copy differs: {destination}")
        else:
            shutil.copytree(source, destination)
        prepared.append(
            {
                "asset_id": row["asset_id"],
                "source_package": str(source),
                "copied_package": str(destination),
                "model_urdf_sha256": actual_hash,
            }
        )
    (input_root / "input_manifest.json").write_text(
        json.dumps(prepared, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return prepared


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
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    matrix = np.eye(4)
    matrix[:3, :3] = rz @ ry @ rx
    matrix[:3, 3] = xyz
    return matrix


def material_colors(root: ET.Element) -> dict[str, tuple[int, int, int, int]]:
    colors = {}
    for material in root.findall("material"):
        color = material.find("color")
        if color is None:
            continue
        rgba = floats(color.attrib.get("rgba"), (0.7, 0.7, 0.7, 1.0))
        colors[material.attrib.get("name", "")] = tuple(
            max(0, min(255, round(channel * 255))) for channel in rgba
        )
    return colors


def visual_color(visual: ET.Element, colors: dict[str, tuple[int, int, int, int]]) -> tuple[int, int, int, int]:
    material = visual.find("material")
    if material is None:
        return (180, 180, 180, 255)
    inline = material.find("color")
    if inline is not None:
        rgba = floats(inline.attrib.get("rgba"), (0.7, 0.7, 0.7, 1.0))
        return tuple(max(0, min(255, round(channel * 255))) for channel in rgba)
    return colors.get(material.attrib.get("name", ""), (180, 180, 180, 255))


def load_mesh_file(path: Path) -> trimesh.Trimesh:
    loaded = trimesh.load(path, force="scene", process=False)
    if isinstance(loaded, trimesh.Trimesh):
        return loaded.copy()
    meshes = [geometry.copy() for geometry in loaded.geometry.values() if isinstance(geometry, trimesh.Trimesh)]
    if not meshes:
        raise ValueError(f"no triangular geometry in {path}")
    return trimesh.util.concatenate(meshes)


def geometry_mesh(geometry: ET.Element, package: Path) -> tuple[trimesh.Trimesh, dict[str, Any]]:
    if not list(geometry):
        raise ValueError("empty geometry element")
    shape = list(geometry)[0]
    if shape.tag == "box":
        size = floats(shape.attrib.get("size"), (1.0, 1.0, 1.0))
        return trimesh.creation.box(extents=size), {"type": "box"}
    if shape.tag == "cylinder":
        radius = float(shape.attrib["radius"])
        length = float(shape.attrib["length"])
        return trimesh.creation.cylinder(radius=radius, height=length, sections=32), {"type": "cylinder"}
    if shape.tag == "sphere":
        radius = float(shape.attrib["radius"])
        return trimesh.creation.icosphere(subdivisions=3, radius=radius), {"type": "sphere"}
    if shape.tag == "mesh":
        filename = shape.attrib["filename"]
        if filename.startswith("package://"):
            raise ValueError(f"package URI unsupported in pilot: {filename}")
        path = (package / filename).resolve()
        if package.resolve() not in path.parents:
            raise ValueError(f"mesh escapes package: {filename}")
        if not path.exists():
            raise FileNotFoundError(path)
        mesh = load_mesh_file(path)
        scale = floats(shape.attrib.get("scale"), (1.0, 1.0, 1.0))
        scale_matrix = np.eye(4)
        scale_matrix[0, 0], scale_matrix[1, 1], scale_matrix[2, 2] = scale
        mesh.apply_transform(scale_matrix)
        return mesh, {"type": "mesh", "filename": filename, "sha256": sha256(path)}
    raise ValueError(f"unsupported geometry type: {shape.tag}")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_package(package: Path) -> dict[str, Any]:
    urdf = package / "model.urdf"
    root = ET.parse(urdf).getroot()
    colors = material_colors(root)
    links = {node.attrib["name"]: node for node in root.findall("link")}
    parents: dict[str, tuple[str, np.ndarray]] = {}
    expected_edges = []
    for joint in root.findall("joint"):
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        if parent_node is None or child_node is None:
            raise ValueError(f"joint missing parent/child: {joint.attrib.get('name')}")
        parent = parent_node.attrib["link"]
        child = child_node.attrib["link"]
        if child in parents:
            raise ValueError(f"multiple parents for {child}")
        parents[child] = (parent, origin_matrix(joint.find("origin")))
        expected_edges.append((parent, child))
    roots = sorted(set(links) - set(parents))
    if not roots:
        raise ValueError("URDF has no root")
    return {
        "urdf": urdf,
        "root": root,
        "links": links,
        "parents": parents,
        "roots": roots,
        "expected_edges": sorted(expected_edges),
        "colors": colors,
    }


def build_link_mesh(
    link: ET.Element,
    package: Path,
    colors: dict[str, tuple[int, int, int, int]],
) -> tuple[trimesh.Trimesh | None, list[dict[str, Any]]]:
    meshes = []
    records = []
    for index, visual in enumerate(link.findall("visual")):
        geometry = visual.find("geometry")
        if geometry is None:
            records.append({"index": index, "status": "empty_visual"})
            continue
        mesh, source = geometry_mesh(geometry, package)
        mesh.apply_transform(origin_matrix(visual.find("origin")))
        color = visual_color(visual, colors)
        mesh.visual.face_colors = np.tile(np.array(color, dtype=np.uint8), (len(mesh.faces), 1))
        meshes.append(mesh)
        records.append(
            {
                "index": index,
                "name": visual.attrib.get("name"),
                "status": "loaded",
                "vertices": len(mesh.vertices),
                "faces": len(mesh.faces),
                **source,
            }
        )
    return (trimesh.util.concatenate(meshes) if meshes else None), records


def export_package(package: Path, output: Path) -> dict[str, Any]:
    parsed = parse_package(package)
    scene = trimesh.Scene(base_frame=BASE_FRAME)
    link_records = {}
    meshes: dict[str, trimesh.Trimesh | None] = {}
    for name, link in parsed["links"].items():
        mesh, visual_records = build_link_mesh(link, package, parsed["colors"])
        meshes[name] = mesh
        link_records[name] = {
            "visual_count": len(link.findall("visual")),
            "loaded_visual_count": sum(row["status"] == "loaded" for row in visual_records),
            "visuals": visual_records,
            "mesh_vertices": len(mesh.vertices) if mesh is not None else 0,
            "mesh_faces": len(mesh.faces) if mesh is not None else 0,
        }

    pending = set(parsed["links"])
    added = set()
    world_matrices = {BASE_FRAME: np.eye(4)}
    while pending:
        progress = False
        for name in sorted(pending):
            parent_info = parsed["parents"].get(name)
            if parent_info is None:
                parent, matrix = BASE_FRAME, np.eye(4)
            else:
                parent, matrix = parent_info
                if parent not in added:
                    continue
            mesh = meshes[name]
            geometry_name = None
            if mesh is not None:
                geometry_name = f"{name}__geometry"
                scene.geometry[geometry_name] = mesh
            scene.graph.update(
                frame_from=parent,
                frame_to=name,
                matrix=matrix,
                geometry=geometry_name,
            )
            world_matrices[name] = world_matrices[parent] @ matrix
            pending.remove(name)
            added.add(name)
            progress = True
        if not progress:
            raise ValueError(f"could not resolve link hierarchy: {sorted(pending)}")

    world_minima = []
    world_maxima = []
    for name, mesh in meshes.items():
        if mesh is None:
            continue
        transformed = trimesh.transform_points(mesh.vertices, world_matrices[name])
        world_minima.append(transformed.min(axis=0))
        world_maxima.append(transformed.max(axis=0))
    if not world_minima:
        raise ValueError(f"package has no visual geometry: {package}")
    urdf_scene_bounds = np.vstack(
        [np.min(np.vstack(world_minima), axis=0), np.max(np.vstack(world_maxima), axis=0)]
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(scene.export(file_type="glb"))
    return {
        "asset_id": package.name,
        "input_package": str(package),
        "input_urdf_sha256": sha256(parsed["urdf"]),
        "urdf_link_names": sorted(parsed["links"]),
        "urdf_link_count": len(parsed["links"]),
        "urdf_mesh_bearing_link_names": sorted(name for name, mesh in meshes.items() if mesh is not None),
        "urdf_mesh_bearing_link_count": sum(mesh is not None for mesh in meshes.values()),
        "urdf_empty_link_names": sorted(name for name, mesh in meshes.items() if mesh is None),
        "urdf_expected_edges": parsed["expected_edges"],
        "roots": parsed["roots"],
        "source_visual_count": sum(row["visual_count"] for row in link_records.values()),
        "loaded_visual_count": sum(row["loaded_visual_count"] for row in link_records.values()),
        "urdf_scene_bounds": urdf_scene_bounds.tolist(),
        "link_records": link_records,
        "output_glb": str(output),
        "output_glb_sha256": sha256(output),
        "output_glb_bytes": output.stat().st_size,
    }


def inspect_glb(path: Path) -> dict[str, Any]:
    gltf = GLTF2.load_binary(str(path))
    nodes = gltf.nodes or []
    node_names = [node.name or "" for node in nodes]
    mesh_nodes = [node.name or "" for node in nodes if node.mesh is not None]
    placeholder_mesh_nodes = sorted(name for name in mesh_nodes if PLACEHOLDER_RE.fullmatch(name))
    edges = []
    for node in nodes:
        for child_index in node.children or []:
            edges.append((node.name or "", nodes[child_index].name or ""))
    loaded = trimesh.load(path, force="scene", process=False)
    return {
        "glb_node_names": node_names,
        "glb_node_count": len(nodes),
        "glb_mesh_node_names": sorted(mesh_nodes),
        "glb_mesh_node_count": len(mesh_nodes),
        "glb_placeholder_mesh_node_names": placeholder_mesh_nodes,
        "glb_named_mesh_node_count": len(mesh_nodes) - len(placeholder_mesh_nodes),
        "glb_edges": sorted(edges),
        "trimesh_reload_geometry_count": len(loaded.geometry),
        "glb_scene_bounds": np.asarray(loaded.bounds, dtype=float).tolist(),
        "trimesh_reload_ok": True,
    }


def evaluate(record: dict[str, Any], inspection: dict[str, Any], repeat_hash: str) -> dict[str, Any]:
    urdf_links = set(record["urdf_link_names"])
    glb_nodes = set(inspection["glb_node_names"])
    expected_mesh = set(record["urdf_mesh_bearing_link_names"])
    actual_mesh = set(inspection["glb_mesh_node_names"])
    expected_edges = {tuple(edge) for edge in record["urdf_expected_edges"]}
    glb_edges = {tuple(edge) for edge in inspection["glb_edges"]}
    world_bounds_preserved = bool(
        np.allclose(
            np.asarray(record["urdf_scene_bounds"], dtype=float),
            np.asarray(inspection["glb_scene_bounds"], dtype=float),
            rtol=1e-5,
            atol=1e-6,
        )
    )
    return {
        **record,
        **inspection,
        "missing_link_nodes": sorted(urdf_links - glb_nodes),
        "extra_non_wrapper_nodes": sorted(glb_nodes - urdf_links - {BASE_FRAME}),
        "mesh_node_names_exact": expected_mesh == actual_mesh,
        "missing_mesh_nodes": sorted(expected_mesh - actual_mesh),
        "extra_mesh_nodes": sorted(actual_mesh - expected_mesh),
        "hierarchy_edges_preserved": expected_edges.issubset(glb_edges),
        "missing_hierarchy_edges": sorted(expected_edges - glb_edges),
        "visuals_loaded_exact": record["source_visual_count"] == record["loaded_visual_count"],
        "world_bounds_preserved": world_bounds_preserved,
        "deterministic_repeat": record["output_glb_sha256"] == repeat_hash,
        "repeat_glb_sha256": repeat_hash,
        "all_gates_pass": (
            not (urdf_links - glb_nodes)
            and not (glb_nodes - urdf_links - {BASE_FRAME})
            and expected_mesh == actual_mesh
            and expected_edges.issubset(glb_edges)
            and record["source_visual_count"] == record["loaded_visual_count"]
            and world_bounds_preserved
            and record["output_glb_sha256"] == repeat_hash
        ),
    }


def write_report(summary: dict[str, Any], records: list[dict[str, Any]], path: Path) -> None:
    lines = [
        "# URDF → GLB scene-graph alignment",
        "",
        "本测试将每个 URDF link 导出为同名 glTF node；同一 link 的多个 visuals 合并成该 node 的一个 mesh，空运动学 link 保留为空 node。测试验证 Naming/Hierarchy 表示、visual 加载、world-space bounds 与确定性，不验证论文的材质、相机、尺度归一化或几何语义。",
        "",
        "| Asset | URDF links | Mesh links | Visuals loaded | GLB nodes | GLB mesh nodes | Names | Edges | Bounds | Repeat hash | Result |",
        "|---|---:|---:|---:|---:|---:|---|---|---|---|---|",
    ]
    for row in records:
        lines.append(
            f"| {row['asset_id']} | {row['urdf_link_count']} | {row['urdf_mesh_bearing_link_count']} | "
            f"{row['loaded_visual_count']}/{row['source_visual_count']} | {row['glb_node_count']} | "
            f"{row['glb_mesh_node_count']} | {'PASS' if not row['missing_link_nodes'] and not row['extra_non_wrapper_nodes'] else 'FAIL'} | "
            f"{'PASS' if row['hierarchy_edges_preserved'] else 'FAIL'} | {'PASS' if row['world_bounds_preserved'] else 'FAIL'} | "
            f"{'PASS' if row['deterministic_repeat'] else 'FAIL'} | "
            f"{'PASS' if row['all_gates_pass'] else 'FAIL'} |"
        )
    lines.extend(
        [
            "",
            f"总结果：{summary['passed_assets']}/{summary['asset_count']} 通过全部 gate。",
            "",
            "## 尚未对齐论文的部分",
            "",
            "- 转换器是本地 trimesh harness，不是论文未公开的统一 GLB converter。",
            "- 多 visual 在每个 URDF link 内合并，因此 Parts=mesh-bearing links；若论文 converter 保留每个 visual 为独立 node，部件数会不同。",
            "- 不执行论文 shared headless scene 的 scale normalization、camera、lighting 或 rendering。",
            "- 不做 semantic judges、point-mask IoU、材质/纹理保真或 articulation sidecar 转换。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", default=str(INPUT_ROOT))
    parser.add_argument("--output-root", default=str(OUTPUT_ROOT))
    parser.add_argument(
        "--prepare-from-manifest",
        help="Copy complete source packages into --input-root before evaluation; writes remain exp-local.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_root = exp_path(args.input_root)
    output_root = exp_path(args.output_root)
    if args.prepare_from_manifest:
        prepare_input_packages(Path(args.prepare_from_manifest).resolve(), input_root)
    packages = sorted(path for path in input_root.iterdir() if path.is_dir() and (path / "model.urdf").exists())
    if not packages:
        raise RuntimeError(f"no exp-local input packages in {input_root}")
    records = []
    for package in packages:
        asset_output = output_root / package.name
        first = asset_output / "model.glb"
        repeat = asset_output / "model_repeat.glb"
        record = export_package(package, first)
        repeat_record = export_package(package, repeat)
        inspection = inspect_glb(first)
        records.append(evaluate(record, inspection, repeat_record["output_glb_sha256"]))
        (asset_output / "record.json").write_text(
            json.dumps(records[-1], indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    summary = {
        "protocol": "nano3d_urdf_to_glb_scenegraph_v2",
        "asset_count": len(records),
        "passed_assets": sum(row["all_gates_pass"] for row in records),
        "all_pass": all(row["all_gates_pass"] for row in records),
        "urdf_link_count": sum(row["urdf_link_count"] for row in records),
        "urdf_mesh_bearing_link_count": sum(row["urdf_mesh_bearing_link_count"] for row in records),
        "urdf_hierarchy_edge_count": sum(len(row["urdf_expected_edges"]) for row in records),
        "source_visual_count": sum(row["source_visual_count"] for row in records),
        "loaded_visual_count": sum(row["loaded_visual_count"] for row in records),
        "glb_node_count": sum(row["glb_node_count"] for row in records),
        "glb_mesh_node_count": sum(row["glb_mesh_node_count"] for row in records),
        "glb_named_mesh_node_count": sum(row["glb_named_mesh_node_count"] for row in records),
        "glb_nameability": (
            sum(row["glb_named_mesh_node_count"] for row in records)
            / sum(row["glb_mesh_node_count"] for row in records)
        ),
        "exact_link_node_assets": sum(
            not row["missing_link_nodes"] and not row["extra_non_wrapper_nodes"] for row in records
        ),
        "hierarchy_preserved_assets": sum(row["hierarchy_edges_preserved"] for row in records),
        "world_bounds_preserved_assets": sum(row["world_bounds_preserved"] for row in records),
        "deterministic_assets": sum(row["deterministic_repeat"] for row in records),
        "limitations": [
            "Local trimesh conversion is not the paper's unreleased converter.",
            "Multiple visuals are merged within each URDF link.",
            "Scene-graph and world-bounds preservation only; no shared render scene or semantic judging.",
        ],
    }
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "records.json").write_text(json.dumps(records, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output_root / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    write_report(summary, records, output_root / "report.md")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"outputs={output_root}")
    return 0 if summary["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
