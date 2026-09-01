#!/usr/bin/env python3
"""Build the method-blind semantic Naming packet for the matched Table 2 panel."""

from __future__ import annotations

import hashlib
import json
import math
import shutil
import tarfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Iterable
from xml.etree import ElementTree as ET

import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont


PROJECT_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = PROJECT_ROOT.parent.resolve()
PROTOCOL_PATH = PROJECT_ROOT / "exp/reference/table2_naming_matched_protocol_v1.json"
GOLD_PATH = PROJECT_ROOT / "exp/reference/table2_naming_semantic_gold_v1.json"
SEMANTIC_PROTOCOL_PATH = (
    PROJECT_ROOT / "exp/reference/table2_naming_semantic_protocol_v1.json"
)
OUTPUT_ROOT = PROJECT_ROOT / "exp/runtime/table2_naming_semantic_v1"
LAM_ARCHIVE = PROJECT_ROOT / ".cache/table6_sources/lam/dataset/viable.tar.gz"
LAM_RECORDS = PROJECT_ROOT / "exp/runtime/lam_naming_matched_v1/records.jsonl"
ARTICRAFT_RECORDS = PROJECT_ROOT / "exp/runtime/articraft_naming_matched_v1/records.jsonl"
INFINITE_RECORDS = (
    PROJECT_ROOT / "exp/runtime/infinite_mobility_naming_v1/matched35/asset_records.jsonl"
)
OURS_RECORDS = PROJECT_ROOT / "exp/runtime/table2_naming_matched_ours_v1/records.jsonl"


@dataclass(frozen=True)
class AssetInput:
    method: str
    source_id: str
    category: str
    package: Path
    urdf: Path
    expected_urdf_sha256: str
    expected_part_count: int
    expected_names: tuple[str, ...] | None
    blind_asset_id: str


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def digest(path: Path) -> str:
    target = contained(path)
    if target.is_symlink() or not target.is_file():
        raise RuntimeError(f"not a regular workspace file: {target}")
    value = hashlib.sha256()
    with target.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def json_digest(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in contained(path).read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def write_json(path: Path, value: Any) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(target)


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.tmp")
    temporary.write_text(
        "".join(json.dumps(row, sort_keys=True, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )
    temporary.replace(target)


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def children(node: ET.Element, name: str) -> list[ET.Element]:
    return [child for child in node if local_name(child.tag) == name]


def vector(raw: str | None, size: int, default: tuple[float, ...]) -> np.ndarray:
    if raw is None:
        return np.asarray(default, dtype=float)
    values = np.asarray([float(item) for item in raw.split()], dtype=float)
    if len(values) != size or not np.all(np.isfinite(values)):
        raise RuntimeError(f"invalid vector: {raw!r}")
    return values


def pose(node: ET.Element | None) -> np.ndarray:
    if node is None:
        return np.eye(4)
    xyz = vector(node.get("xyz"), 3, (0.0, 0.0, 0.0))
    roll, pitch, yaw = vector(node.get("rpy"), 3, (0.0, 0.0, 0.0))
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = xyz
    return result


def load_mesh(path: Path) -> trimesh.Trimesh:
    loaded = trimesh.load(contained(path), force="scene", process=False)
    if isinstance(loaded, trimesh.Trimesh):
        return loaded
    if not isinstance(loaded, trimesh.Scene) or not loaded.geometry:
        raise RuntimeError(f"mesh has no geometry: {path}")
    meshes = []
    for node_name in sorted(loaded.graph.nodes_geometry):
        transform, geometry_name = loaded.graph.get(node_name)
        if geometry_name is None:
            continue
        geometry = loaded.geometry[geometry_name]
        if not isinstance(geometry, trimesh.Trimesh):
            continue
        copy = geometry.copy()
        copy.apply_transform(transform)
        meshes.append(copy)
    if not meshes:
        raise RuntimeError(f"mesh scene has no triangular geometry: {path}")
    return trimesh.util.concatenate(meshes)


def geometry_mesh(geometry: ET.Element, package: Path) -> trimesh.Trimesh | None:
    shapes = list(geometry)
    if len(shapes) != 1:
        return None
    shape = shapes[0]
    kind = local_name(shape.tag)
    if kind == "mesh":
        filename = (shape.get("filename") or "").strip()
        if not filename:
            return None
        if filename.startswith("package://"):
            filename = filename.removeprefix("package://")
        candidate = contained(package / filename)
        if candidate != package and package not in candidate.parents:
            raise RuntimeError(f"mesh path escapes package: {filename}")
        mesh = load_mesh(candidate)
        scale = vector(shape.get("scale"), 3, (1.0, 1.0, 1.0))
        mesh.apply_scale(scale)
        return mesh
    if kind == "box":
        raw_size = shape.get("size")
        if raw_size is None:
            return None
        size = vector(raw_size, 3, ())
        if np.any(size <= 0):
            return None
        return trimesh.creation.box(extents=size)
    if kind == "cylinder":
        radius = float(shape.get("radius", "nan"))
        length = float(shape.get("length", "nan"))
        if not math.isfinite(radius) or not math.isfinite(length) or min(radius, length) <= 0:
            return None
        return trimesh.creation.cylinder(radius=radius, height=length, sections=24)
    if kind == "sphere":
        radius = float(shape.get("radius", "nan"))
        if not math.isfinite(radius) or radius <= 0:
            return None
        return trimesh.creation.icosphere(subdivisions=2, radius=radius)
    return None


def sample_mesh(mesh: trimesh.Trimesh, maximum: int = 1200) -> np.ndarray:
    vertices = np.asarray(mesh.vertices, dtype=float)
    if len(mesh.faces):
        surface, _ = trimesh.sample.sample_surface(mesh, maximum, seed=260811)
    else:
        surface = np.empty((0, 3))
    if len(vertices) > 400:
        vertices = vertices[np.linspace(0, len(vertices) - 1, 400, dtype=int)]
    points = np.vstack((vertices, surface))
    if not len(points) or not np.all(np.isfinite(points)):
        raise RuntimeError("render geometry has no finite points")
    return points


def link_clouds(asset: AssetInput) -> dict[str, np.ndarray]:
    root = ET.parse(contained(asset.urdf)).getroot()
    link_nodes = {str(link.get("name") or ""): link for link in children(root, "link")}
    child_edges: dict[str, tuple[str, np.ndarray]] = {}
    for joint in children(root, "joint"):
        parents = children(joint, "parent")
        descendants = children(joint, "child")
        if len(parents) != 1 or len(descendants) != 1:
            continue
        parent = str(parents[0].get("link") or "")
        child = str(descendants[0].get("link") or "")
        origins = children(joint, "origin")
        child_edges[child] = (parent, pose(origins[0] if origins else None))

    world: dict[str, np.ndarray] = {
        name: np.eye(4) for name in link_nodes if name not in child_edges
    }
    pending = dict(child_edges)
    while pending:
        progressed = False
        for child, (parent, transform) in list(pending.items()):
            if parent in world:
                world[child] = world[parent] @ transform
                del pending[child]
                progressed = True
        if not progressed:
            raise RuntimeError(f"joint graph is disconnected or cyclic: {asset.source_id}")

    clouds: dict[str, np.ndarray] = {}
    for link_name, link in link_nodes.items():
        visual_clouds = []
        for visual in children(link, "visual"):
            geometries = children(visual, "geometry")
            if len(geometries) != 1:
                continue
            mesh = geometry_mesh(geometries[0], asset.package)
            if mesh is None:
                continue
            origins = children(visual, "origin")
            mesh.apply_transform(world[link_name] @ pose(origins[0] if origins else None))
            visual_clouds.append(sample_mesh(mesh))
        if visual_clouds:
            points = np.vstack(visual_clouds)
            if len(points) > 3000:
                points = points[np.linspace(0, len(points) - 1, 3000, dtype=int)]
            clouds[link_name] = points
    return clouds


def rotated(points: np.ndarray, azimuth: float, elevation: float) -> np.ndarray:
    azimuth = math.radians(azimuth)
    elevation = math.radians(elevation)
    rz = np.array(
        [
            [math.cos(azimuth), -math.sin(azimuth), 0],
            [math.sin(azimuth), math.cos(azimuth), 0],
            [0, 0, 1],
        ],
        dtype=float,
    )
    rx = np.array(
        [
            [1, 0, 0],
            [0, math.cos(elevation), -math.sin(elevation)],
            [0, math.sin(elevation), math.cos(elevation)],
        ],
        dtype=float,
    )
    return points @ (rx @ rz).T


def rasterize(
    points: np.ndarray,
    target: np.ndarray,
    *,
    azimuth: float,
    elevation: float,
    size: int = 224,
) -> Image.Image:
    camera = rotated(points, azimuth, elevation)
    x, y, depth = camera[:, 0], camera[:, 1], camera[:, 2]
    span = max(float(np.ptp(x)), float(np.ptp(y)), 1e-9)
    scale = (size - 24) / span
    u = np.clip(np.rint((x - (x.min() + x.max()) / 2) * scale + size / 2), 0, size - 1).astype(int)
    v = np.clip(np.rint(size / 2 - (y - (y.min() + y.max()) / 2) * scale), 0, size - 1).astype(int)
    depth_norm = (depth - depth.min()) / max(float(np.ptp(depth)), 1e-9)
    base = np.where(target[:, None], np.array([224, 62, 48]), np.array([103, 117, 137]))
    colors = np.clip(base * (0.6 + 0.4 * depth_norm[:, None]), 0, 255).astype(np.uint8)
    offsets = np.array([(x, y) for x in (-1, 0, 1) for y in (-1, 0, 1)], dtype=int)
    uu = np.clip((u[:, None] + offsets[:, 0]).reshape(-1), 0, size - 1)
    vv = np.clip((v[:, None] + offsets[:, 1]).reshape(-1), 0, size - 1)
    flat = vv * size + uu
    depths = np.repeat(depth, len(offsets))
    repeated_colors = np.repeat(colors, len(offsets), axis=0)
    order = np.lexsort((depths, flat))
    ordered_flat = flat[order]
    keep = np.r_[ordered_flat[1:] != ordered_flat[:-1], True]
    selected = order[keep]
    canvas = np.full((size * size, 3), 247, dtype=np.uint8)
    canvas[flat[selected]] = repeated_colors[selected]
    return Image.fromarray(canvas.reshape(size, size, 3), mode="RGB")


def panel(image: Image.Image, label: str) -> Image.Image:
    result = Image.new("RGB", (image.width, image.height + 22), "white")
    result.paste(image, (0, 22))
    ImageDraw.Draw(result).text((6, 4), label, fill=(25, 25, 25), font=ImageFont.load_default())
    return result


def build_preview(asset: AssetInput, node_name: str, clouds: dict[str, np.ndarray], path: Path) -> None:
    context_points = []
    context_targets = []
    for name in sorted(clouds):
        points = clouds[name]
        context_points.append(points)
        context_targets.append(np.full(len(points), name == node_name, dtype=bool))
    all_points = np.vstack(context_points)
    all_targets = np.concatenate(context_targets)
    target = clouds[node_name]
    views = [
        panel(rasterize(all_points, all_targets, azimuth=-42, elevation=28), "context: target red"),
        panel(rasterize(target, np.ones(len(target), bool), azimuth=-42, elevation=28), "target: iso"),
        panel(rasterize(target, np.ones(len(target), bool), azimuth=90, elevation=0), "target: side"),
    ]
    header = 52
    output = Image.new("RGB", (sum(view.width for view in views), views[0].height + header), "white")
    draw = ImageDraw.Draw(output)
    draw.text((8, 6), f"asset={asset.blind_asset_id} category={asset.category}", fill=(20, 20, 20), font=ImageFont.load_default())
    draw.text((8, 25), f"node_name={node_name}", fill=(20, 20, 20), font=ImageFont.load_default())
    x = 0
    for view in views:
        output.paste(view, (x, header))
        x += view.width
    target_path = contained(path)
    target_path.parent.mkdir(parents=True, exist_ok=True)
    output.save(target_path, format="PNG", optimize=True)


def blind_id(protocol_hash: str, method: str, source_id: str) -> str:
    value = hashlib.sha256(f"{protocol_hash}|semantic-v1|{method}|{source_id}".encode()).hexdigest()
    return f"asset_{value[:16]}"


def stage_lam(records: list[dict[str, Any]], protocol_hash: str) -> dict[str, Path]:
    stage = contained(OUTPUT_ROOT / "staged_lam")
    expected = {str(row["rel_path"]): row for row in records}
    reusable = True
    paths: dict[str, Path] = {}
    for rel_path, row in expected.items():
        asset_id = blind_id(protocol_hash, "LAM", str(row["object_release_id"]))
        package = stage / asset_id / "package"
        urdf = package / "generated.urdf"
        if not urdf.is_file() or digest(urdf) != row["urdf_sha256"]:
            reusable = False
            break
        paths[rel_path] = package
    if reusable and len(paths) == len(expected):
        return paths

    temporary = contained(OUTPUT_ROOT / ".staged_lam_build")
    if temporary.exists():
        shutil.rmtree(temporary)
    temporary.mkdir(parents=True)
    extracted_files = 0
    with tarfile.open(contained(LAM_ARCHIVE), "r|gz") as archive:
        for member in archive:
            pure = PurePosixPath(member.name)
            if pure.is_absolute() or ".." in pure.parts or len(pure.parts) < 3:
                continue
            rel_path = "/".join(pure.parts[:3])
            if rel_path not in expected:
                continue
            if member.issym() or member.islnk() or member.isdev():
                raise RuntimeError(f"unsafe selected LAM archive member: {member.name}")
            if not member.isfile():
                continue
            row = expected[rel_path]
            asset_id = blind_id(protocol_hash, "LAM", str(row["object_release_id"]))
            package = temporary / asset_id / "package"
            relative = Path(*pure.parts[3:])
            target = contained(package / relative)
            if target != package and package not in target.parents:
                raise RuntimeError(f"LAM member escaped package: {member.name}")
            target.parent.mkdir(parents=True, exist_ok=True)
            source = archive.extractfile(member)
            if source is None:
                raise RuntimeError(f"LAM member could not be read: {member.name}")
            with target.open("wb") as handle:
                shutil.copyfileobj(source, handle)
            extracted_files += 1
    if extracted_files == 0:
        raise RuntimeError("no selected LAM files were extracted")
    for rel_path, row in expected.items():
        asset_id = blind_id(protocol_hash, "LAM", str(row["object_release_id"]))
        urdf = temporary / asset_id / "package/generated.urdf"
        if digest(urdf) != row["urdf_sha256"]:
            raise RuntimeError(f"LAM staged URDF hash mismatch: {row['object_release_id']}")
    if stage.exists():
        shutil.rmtree(stage)
    temporary.replace(stage)
    return {
        rel_path: stage
        / blind_id(protocol_hash, "LAM", str(row["object_release_id"]))
        / "package"
        for rel_path, row in expected.items()
    }


def asset_inputs(protocol_hash: str) -> list[AssetInput]:
    assets: list[AssetInput] = []
    ours = read_jsonl(OURS_RECORDS)
    for row in ours:
        source_id = f"{row['category']}:{row['seed']}"
        package = contained(WORKSPACE_ROOT / row["package"])
        assets.append(
            AssetInput(
                "Ours",
                source_id,
                row["category"],
                package,
                package / "model.urdf",
                row["urdf_sha256"],
                row["renderable_part_count"],
                tuple(row["renderable_names"]),
                blind_id(protocol_hash, "Ours", source_id),
            )
        )

    lam = read_jsonl(LAM_RECORDS)
    lam_packages = stage_lam(lam, protocol_hash)
    for row in lam:
        source_id = str(row["object_release_id"])
        package = lam_packages[row["rel_path"]]
        assets.append(
            AssetInput(
                "LAM",
                source_id,
                row["canonical_category"],
                package,
                package / "generated.urdf",
                row["urdf_sha256"],
                row["renderable_part_count"],
                None,
                blind_id(protocol_hash, "LAM", source_id),
            )
        )

    articraft = read_jsonl(ARTICRAFT_RECORDS)
    for row in articraft:
        source_id = str(row["record_id"])
        package = contained(
            PROJECT_ROOT / "exp/runtime/articraft_naming_matched_v1/cases" / source_id / "package"
        )
        direct = row["direct_naming"]
        assets.append(
            AssetInput(
                "Articraft",
                source_id,
                row["canonical_category"],
                package,
                package / "model.urdf",
                row["urdf_sha256"],
                direct["renderable_visual_part_count"],
                tuple(direct["renderable_visual_link_names"]),
                blind_id(protocol_hash, "Articraft", source_id),
            )
        )

    infinite = read_jsonl(INFINITE_RECORDS)
    for row in infinite:
        source_id = f"{row['factory']}:{row['seed']}"
        urdf = contained(PROJECT_ROOT / row["urdf_path"])
        package = urdf.parent
        assets.append(
            AssetInput(
                "Infinite Mobility",
                source_id,
                row["canonical_category"],
                package,
                urdf,
                row["urdf_sha256"],
                row["urdf_part_node_count"],
                tuple(row["raw_part_names"]),
                blind_id(protocol_hash, "Infinite Mobility", source_id),
            )
        )
    return sorted(assets, key=lambda item: item.blind_asset_id)


def main() -> int:
    protocol = json.loads(contained(PROTOCOL_PATH).read_text(encoding="utf-8"))
    gold = json.loads(contained(GOLD_PATH).read_text(encoding="utf-8"))
    semantic_protocol = json.loads(
        contained(SEMANTIC_PROTOCOL_PATH).read_text(encoding="utf-8")
    )
    protocol_hash = digest(PROTOCOL_PATH)
    gold_hash = digest(GOLD_PATH)
    semantic_protocol_hash = digest(SEMANTIC_PROTOCOL_PATH)
    if semantic_protocol["parent_matched_protocol"]["sha256"] != protocol_hash:
        raise RuntimeError("semantic protocol parent hash mismatch")
    if semantic_protocol["semantic_gold"]["sha256"] != gold_hash:
        raise RuntimeError("semantic protocol gold hash mismatch")
    categories = gold["categories"]
    assets = asset_inputs(protocol_hash)
    if len(assets) != 140:
        raise RuntimeError(f"expected 140 assets, found {len(assets)}")
    method_counts: dict[str, int] = {}
    category_method_counts: dict[str, dict[str, int]] = {}
    tasks = []
    audit = []
    preview_hashes = {}
    for index, asset in enumerate(assets, 1):
        print(f"[{index}/140] render {asset.blind_asset_id} {asset.category}", flush=True)
        if digest(asset.urdf) != asset.expected_urdf_sha256:
            raise RuntimeError(f"URDF hash mismatch: {asset.source_id}")
        clouds = link_clouds(asset)
        names = tuple(sorted(clouds))
        if len(names) != asset.expected_part_count:
            raise RuntimeError(
                f"part count mismatch {asset.source_id}: {len(names)} != {asset.expected_part_count}"
            )
        if asset.expected_names is not None and set(names) != set(asset.expected_names):
            raise RuntimeError(f"part-name set mismatch: {asset.source_id}")
        method_counts[asset.method] = method_counts.get(asset.method, 0) + 1
        by_method = category_method_counts.setdefault(asset.category, {})
        by_method[asset.method] = by_method.get(asset.method, 0) + 1
        category_gold = categories[asset.category]
        required_roles = [
            {
                "role": role["role_id"],
                "min_count": 1,
                "functional": bool(role["functional_core"]),
                "definition": role["definition"],
                "acceptable_synonyms": role["acceptable_synonyms"],
                "allowed_specializations": role.get("allowed_specializations", []),
            }
            for role in category_gold["minimal_required_roles"]
        ]
        optional_roles = [
            {
                "role": role["role_id"],
                "definition": role.get("definition"),
                "presence_condition": role["presence_condition"],
                "acceptable_synonyms": role["acceptable_synonyms"],
                "allowed_specializations": role.get("allowed_specializations", []),
            }
            for role in category_gold["optional_roles"]
        ]
        for node_name in names:
            item_id = "item_" + hashlib.sha256(
                f"{protocol_hash}|{gold_hash}|{asset.blind_asset_id}|{node_name}".encode()
            ).hexdigest()[:20]
            preview = OUTPUT_ROOT / "previews" / f"{item_id}.png"
            build_preview(asset, node_name, clouds, preview)
            preview_hashes[preview.name] = digest(preview)
            base = {
                "schema": "table2_naming_semantic_blind_task_v1",
                "item_id": item_id,
                "asset_id": asset.blind_asset_id,
                "category": asset.category,
                "node_name": node_name,
                "has_visual": True,
                "preview_path": str(preview.relative_to(WORKSPACE_ROOT)),
                "asset_mesh_node_names": list(names),
                "required_roles": required_roles,
                "optional_roles": optional_roles,
                "instance_discriminability_rules": category_gold[
                    "instance_discriminability_rules"
                ],
                "allowed_verdicts": [
                    "spec_match",
                    "extra_real_part",
                    "invalid_or_hallucinated",
                    "uncertain",
                ],
                "verdict_definition": {
                    "spec_match": "name and geometry support one required role",
                    "extra_real_part": "name semantically describes a real part outside required roles",
                    "invalid_or_hallucinated": "name is opaque/generic, contradicts geometry, or does not identify a semantic part",
                    "uncertain": "preview or name is insufficient for a defensible decision",
                },
                "judge_verdict": None,
                "judge_matched_role": None,
                "judge_geometry_role": None,
                "judge_geometry_is_real_part": None,
                "judge_instance_id": None,
                "judge_same_semantic_part_as": None,
                "judge_reason": None,
            }
            tasks.append(base)
            audit.append(
                {
                    **base,
                    "method": asset.method,
                    "source_id": asset.source_id,
                    "source_urdf": str(asset.urdf.relative_to(WORKSPACE_ROOT)),
                    "source_urdf_sha256": asset.expected_urdf_sha256,
                }
            )
    tasks.sort(key=lambda row: row["item_id"])
    audit.sort(key=lambda row: row["item_id"])
    if len(tasks) != 1107:
        raise RuntimeError(f"expected 1107 tasks, found {len(tasks)}")
    write_jsonl(OUTPUT_ROOT / "blind_tasks.jsonl", tasks)
    write_jsonl(OUTPUT_ROOT / "audit_tasks.jsonl", audit)
    for judge in ("judge_a", "judge_b", "judge_c"):
        target = OUTPUT_ROOT / "judges" / f"{judge}.jsonl"
        if target.exists() and any(
            row.get("judge_verdict") is not None for row in read_jsonl(target)
        ):
            raise RuntimeError(f"refusing to overwrite completed judge file: {target}")
        write_jsonl(target, tasks)
    manifest = {
        "protocol_id": "table2_naming_semantic_packet_v1",
        "matched_protocol_sha256": protocol_hash,
        "semantic_gold_sha256": gold_hash,
        "semantic_protocol_sha256": semantic_protocol_hash,
        "asset_count": len(assets),
        "task_count": len(tasks),
        "preview_count": len(preview_hashes),
        "method_counts_audit_only": dict(sorted(method_counts.items())),
        "category_method_counts_audit_only": category_method_counts,
        "blind_tasks_sha256": digest(OUTPUT_ROOT / "blind_tasks.jsonl"),
        "audit_tasks_sha256": digest(OUTPUT_ROOT / "audit_tasks.jsonl"),
        "preview_hashes": dict(sorted(preview_hashes.items())),
        "preview_set_sha256": json_digest(preview_hashes),
        "method_identity_present_in_blind_tasks": False,
        "judge_count": 3,
        "judge_files_complete": False,
    }
    write_json(OUTPUT_ROOT / "manifest.json", manifest)
    readme = """# Table 2 matched Naming semantic packet v1

This packet contains 1,107 method-blind link tasks from 140 category-matched URDF assets.
Judges receive only blind_tasks.jsonl and previews/. Do not open audit_tasks.jsonl or manifest method counts.

For every task, inspect both the node name and highlighted geometry. Opaque indices such as l_0 do not identify
a semantic part and must be invalid_or_hallucinated, even when the highlighted geometry is real. Use spec_match
only when name and geometry support a frozen required role. Use extra_real_part for a truthful name outside the
required list, including optional roles.

Annotate geometry independently of name correctness. Set judge_geometry_is_real_part to true when the target is
a coherent physical or functional component, and false for a stray fragment or artifact. Set judge_geometry_role
to a frozen required/optional role ID, other_real_part:<short_label>, or unknown. For every geometry-real node,
set judge_same_semantic_part_as to none or another node name when both links are fragments of the same semantic
part; use not_applicable for geometry that is not a real part.

Within each asset, compare all nodes assigned the same geometry role. If there are multiple physical instances,
judge_instance_id records the identity actually conveyed by the node name (left/right, upper/lower, or a stable
ordinal); use ambiguous when the names do not distinguish instances. Use not_applicable for singleton roles.
Every task requires a concise judge_reason. Do not open audit_tasks.jsonl, manifest.json, source URDFs, baseline
summaries, or another judge file.
"""
    contained(OUTPUT_ROOT / "README.md").write_text(readme, encoding="utf-8")
    print(json.dumps({key: value for key, value in manifest.items() if key != "preview_hashes"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
