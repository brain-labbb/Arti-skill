#!/usr/bin/env python3
"""Build deterministic GLB mesh-node previews and blind three-judge tasks."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import shutil
import textwrap
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont


EXP_ROOT = Path("/mnt/zsn/lyb/arti-skill/exp").resolve()
QUEUE_PATH = EXP_ROOT / "runtime/nano3d_naming/judge_queue.jsonl"
GLB_ROOT = EXP_ROOT / "runtime/nano3d_glb_n33/output"
ASSET_RECORDS = EXP_ROOT / "runtime/nano3d_naming/asset_records.json"
OUTPUT_ROOT = EXP_ROOT / "runtime/nano3d_naming_judge_packet_v1"
PREVIEW_ROOT = OUTPUT_ROOT / "previews"
BLIND_PATH = OUTPUT_ROOT / "blind_tasks.jsonl"
AUDIT_PATH = OUTPUT_ROOT / "audit_tasks.jsonl"
JUDGE_ROOT = OUTPUT_ROOT / "judges"
GOLD_TASKS_PATH = OUTPUT_ROOT / "independent_gold_annotation_template.jsonl"
SOURCE_PICTURE_ROOT = Path("/mnt/zsn/lyb/arti-skill/articraft_data/picture/0611").resolve()
BENCHMARK_INPUT_ROOT = OUTPUT_ROOT / "benchmark_inputs"


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows), encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sampled_points(mesh: trimesh.Trimesh, maximum: int = 2400) -> np.ndarray:
    vertices = np.asarray(mesh.vertices, dtype=float)
    if len(mesh.faces):
        surface, _ = trimesh.sample.sample_surface(mesh, maximum, seed=1729)
    else:
        surface = np.empty((0, 3))
    if len(vertices) > 600:
        vertices = vertices[np.linspace(0, len(vertices) - 1, 600, dtype=int)]
    points = np.vstack([vertices, surface])
    if not len(points):
        raise ValueError("mesh has no points")
    return points


def scene_clouds(path: Path) -> dict[str, np.ndarray]:
    scene = trimesh.load(path, force="scene", process=False)
    clouds = {}
    for node_name in sorted(scene.graph.nodes_geometry):
        matrix, geometry_name = scene.graph.get(node_name)
        if geometry_name is None:
            continue
        mesh = scene.geometry[geometry_name]
        if not isinstance(mesh, trimesh.Trimesh):
            continue
        clouds[str(node_name)] = trimesh.transform_points(sampled_points(mesh), matrix)
    return clouds


def rotated(points: np.ndarray, azimuth: float, elevation: float) -> np.ndarray:
    az = math.radians(azimuth)
    el = math.radians(elevation)
    rz = np.array(
        [[math.cos(az), -math.sin(az), 0], [math.sin(az), math.cos(az), 0], [0, 0, 1]],
        dtype=float,
    )
    rx = np.array(
        [[1, 0, 0], [0, math.cos(el), -math.sin(el)], [0, math.sin(el), math.cos(el)]],
        dtype=float,
    )
    return points @ (rx @ rz).T


def rasterize(
    points: np.ndarray,
    target: np.ndarray,
    *,
    azimuth: float,
    elevation: float,
    size: int = 280,
    force_target: bool = False,
) -> Image.Image:
    camera = rotated(points, azimuth, elevation)
    x, y, depth = camera[:, 0], camera[:, 1], camera[:, 2]
    span = max(float(np.ptp(x)), float(np.ptp(y)), 1e-9)
    scale = (size - 30) / span
    u = np.clip(np.rint((x - (x.min() + x.max()) / 2) * scale + size / 2), 0, size - 1).astype(int)
    v = np.clip(np.rint(size / 2 - (y - (y.min() + y.max()) / 2) * scale), 0, size - 1).astype(int)
    depth_norm = (depth - depth.min()) / max(float(np.ptp(depth)), 1e-9)
    base = np.where(target[:, None], np.array([224, 67, 52]), np.array([112, 124, 142]))
    shade = (0.58 + 0.42 * depth_norm)[:, None]
    colors = np.clip(base * shade, 0, 255).astype(np.uint8)

    offsets = np.array([(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)], dtype=int)
    uu = np.clip((u[:, None] + offsets[:, 0]).reshape(-1), 0, size - 1)
    vv = np.clip((v[:, None] + offsets[:, 1]).reshape(-1), 0, size - 1)
    dd = np.repeat(depth, len(offsets))
    cc = np.repeat(colors, len(offsets), axis=0)
    tt = np.repeat(target, len(offsets))
    flat = vv * size + uu
    order = np.lexsort((dd, flat))
    ordered_flat = flat[order]
    keep = np.r_[ordered_flat[1:] != ordered_flat[:-1], True]
    chosen = order[keep]
    canvas = np.full((size * size, 3), 247, dtype=np.uint8)
    canvas[flat[chosen]] = cc[chosen]
    if force_target and np.any(tt):
        target_rows = np.flatnonzero(tt)
        canvas[flat[target_rows]] = np.array([238, 63, 48], dtype=np.uint8)
    image = canvas.reshape(size, size, 3)
    return Image.fromarray(image, mode="RGB")


def label_panel(image: Image.Image, title: str) -> Image.Image:
    panel = Image.new("RGB", (image.width, image.height + 24), "white")
    panel.paste(image, (0, 24))
    ImageDraw.Draw(panel).text((8, 5), title, fill=(30, 30, 30), font=ImageFont.load_default())
    return panel


def build_preview(asset_id: str, node_name: str, clouds: dict[str, np.ndarray], path: Path) -> None:
    if node_name not in clouds:
        raise RuntimeError(f"{asset_id}: GLB mesh node missing: {node_name}")
    all_points = []
    all_target = []
    for name in sorted(clouds):
        points = clouds[name]
        all_points.append(points)
        all_target.append(np.full(len(points), name == node_name, dtype=bool))
    context_points = np.vstack(all_points)
    context_target = np.concatenate(all_target)
    target_points = clouds[node_name]
    panels = [
        label_panel(
            rasterize(
                context_points,
                context_target,
                azimuth=-42,
                elevation=28,
                force_target=True,
            ),
            "context / target in red",
        ),
        label_panel(rasterize(target_points, np.ones(len(target_points), bool), azimuth=-42, elevation=28), "isolated / iso"),
        label_panel(rasterize(target_points, np.ones(len(target_points), bool), azimuth=0, elevation=0), "isolated / front"),
        label_panel(rasterize(target_points, np.ones(len(target_points), bool), azimuth=90, elevation=0), "isolated / side"),
    ]
    header = 58
    output = Image.new("RGB", (sum(panel.width for panel in panels), panels[0].height + header), "white")
    draw = ImageDraw.Draw(output)
    title = f"{asset_id} | {node_name}"
    wrapped = textwrap.wrap(title, width=120)[:2]
    for line_number, line in enumerate(wrapped):
        draw.text((10, 8 + line_number * 18), line, fill=(15, 15, 15), font=ImageFont.load_default())
    x = 0
    for panel in panels:
        output.paste(panel, (x, header))
        x += panel.width
    path.parent.mkdir(parents=True, exist_ok=True)
    output.save(path, format="PNG", optimize=True)


def blind_row(row: dict[str, Any], asset_mesh_nodes: list[str], preview: Path, glb: Path) -> dict[str, Any]:
    return {
        "schema": "nano3d_naming_blind_judge_task_v1.1",
        "item_id": row["item_id"],
        "asset_id": row["asset_id"],
        "category": row["category"],
        "node_name": row["node_name"],
        "has_visual": True,
        "glb_path": str(glb),
        "preview_path": str(preview),
        "asset_mesh_node_names": asset_mesh_nodes,
        "reference_roles": row["reference_roles"],
        "reference_status": "source-derived reference; not independent hidden gold",
        "allowed_verdicts": ["spec_match", "extra_real_part", "invalid_or_hallucinated", "uncertain"],
        "judge_field_contract": {
            "matched_role": "required for spec_match; null otherwise",
            "instance_id": (
                "for spec_match roles with min_count>1, record the instance identity distinguishable "
                "from the node name and preview (for example left/right or 0/1); use the same ID "
                "for fragments of one instance; otherwise use 'not_applicable'; uncertain uses null"
            ),
            "same_semantic_part_as": (
                "for spec_match/extra_real_part, use 'none' for a distinct semantic part or another "
                "node name in this asset for a fragment of the same semantic part; "
                "invalid_or_hallucinated uses 'not_applicable'; uncertain uses null"
            ),
        },
        "judge_verdict": None,
        "judge_matched_role": None,
        "judge_instance_id": None,
        "judge_same_semantic_part_as": None,
        "judge_reason": None,
    }


def normalized_category(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "", value.lower())


def freeze_benchmark_input(record: dict[str, Any]) -> dict[str, Any]:
    slug = str(record["slug"])
    readable = slug.replace("pictureX_0611_", "").replace("_", " ")
    copied_images = []
    copied_image_hashes = {}
    if slug.startswith("pictureX_0611_"):
        subcategory = slug.removeprefix("pictureX_0611_")
        matches = [
            path
            for path in SOURCE_PICTURE_ROOT.iterdir()
            if path.is_dir() and normalized_category(path.name) == normalized_category(subcategory)
        ]
        if len(matches) != 1:
            raise RuntimeError(f"{slug}: expected one source-picture directory, found {matches}")
        destination = BENCHMARK_INPUT_ROOT / record["asset_id"]
        destination.mkdir(parents=True, exist_ok=True)
        for source in sorted(matches[0].glob("*.png")):
            target = destination / source.name
            shutil.copy2(source, target)
            copied_images.append(str(target))
            copied_image_hashes[str(target)] = sha256(target)
        if not copied_images:
            raise RuntimeError(f"{slug}: source-picture directory has no PNG inputs")
        prompt = f"Reference-image object category: {readable}. Annotate only parts supported by the category and images."
        core_status = "ready from output-independent category images and category text"
    else:
        prompt = f"Text object category: {readable}. Annotate the category's required core semantic parts."
        core_status = "ready for category-level core taxonomy from output-independent text category"
    return {
        "benchmark_input_text": prompt,
        "benchmark_input_images": copied_images,
        "benchmark_input_image_hashes": copied_image_hashes,
        "core_taxonomy_input_status": core_status,
        "exact_per_seed_prompt_status": "missing; selected existing export does not preserve an exact frozen generation request",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--skip-previews",
        action="store_true",
        help="reuse and hash-check the existing 233 deterministic previews",
    )
    args = parser.parse_args()
    queue = read_jsonl(QUEUE_PATH)
    asset_records = {row["asset_id"]: row for row in json.loads(ASSET_RECORDS.read_text(encoding="utf-8"))}
    if any(not row.get("has_visual") for row in queue):
        raise RuntimeError("judge queue must contain GLB mesh nodes only")
    if len(queue) != 233:
        raise RuntimeError(f"expected N=32 mesh-node judge queue with 233 items, found {len(queue)}")
    PREVIEW_ROOT.mkdir(parents=True, exist_ok=True)
    blind = []
    audit = []
    by_asset: dict[str, list[dict[str, Any]]] = {}
    for row in queue:
        by_asset.setdefault(row["asset_id"], []).append(row)
    for asset_id in sorted(by_asset):
        glb = GLB_ROOT / asset_id / "model.glb"
        clouds = scene_clouds(glb)
        expected = sorted(
            link["name"] for link in asset_records[asset_id]["links"] if link["has_visual"]
        )
        if sorted(clouds) != expected:
            raise RuntimeError(f"{asset_id}: GLB mesh-node set differs from Naming input")
        for row in sorted(by_asset[asset_id], key=lambda item: item["node_name"]):
            preview = PREVIEW_ROOT / f"{row['item_id']}.png"
            if args.skip_previews:
                if not preview.is_file():
                    raise RuntimeError(f"--skip-previews requested but preview is missing: {preview}")
            else:
                build_preview(asset_id, row["node_name"], clouds, preview)
            task = blind_row(row, sorted(clouds), preview, glb)
            blind.append(task)
            audit.append(
                {
                    **task,
                    "assigned_source_role": row.get("assigned_source_role"),
                    "deterministic_role_candidates": row.get("deterministic_role_candidates", []),
                    "reference_evidence": row.get("reference_evidence"),
                    "source_geometry": row.get("geometry", []),
                }
            )

    blind.sort(key=lambda row: row["item_id"])
    audit.sort(key=lambda row: row["item_id"])
    gold_tasks = []
    for record in sorted(asset_records.values(), key=lambda row: row["asset_id"]):
        frozen_input = freeze_benchmark_input(record)
        gold_tasks.append(
            {
                "schema": "nano3d_independent_naming_gold_annotation_v1",
                "asset_id": record["asset_id"],
                "category": record["slug"],
                **frozen_input,
                "core_taxonomy_annotation_ready": True,
                "per_seed_optional_annotation_ready": False,
                "annotator_required_parts": None,
                "annotator_optional_parts": None,
                "annotator_functional_core_parts": None,
                "annotator_instance_rules": None,
                "annotator_allowed_synonyms": None,
                "annotator_notes": None,
            }
        )
    write_jsonl(BLIND_PATH, blind)
    write_jsonl(AUDIT_PATH, audit)
    write_jsonl(GOLD_TASKS_PATH, gold_tasks)
    for name in ("judge_a", "judge_b", "judge_c"):
        judge_path = JUDGE_ROOT / f"{name}.jsonl"
        if judge_path.exists() and any(
            row.get("judge_verdict") is not None for row in read_jsonl(judge_path)
        ):
            raise RuntimeError(
                f"refusing to overwrite a nonblank judge file: {judge_path}; move it before rebuilding"
            )
        write_jsonl(judge_path, blind)
    preview_files = sorted(PREVIEW_ROOT.glob("*.png"))
    expected_preview_names = {f"{row['item_id']}.png" for row in blind}
    actual_preview_names = {path.name for path in preview_files}
    if expected_preview_names != actual_preview_names:
        raise RuntimeError(
            "preview set mismatch: "
            f"missing={len(expected_preview_names - actual_preview_names)}, "
            f"extra={len(actual_preview_names - expected_preview_names)}"
        )
    manifest = {
        "protocol": "nano3d_naming_judge_packet_v1.1",
        "queue_sha256": sha256(QUEUE_PATH),
        "blind_tasks_sha256": sha256(BLIND_PATH),
        "audit_tasks_sha256": sha256(AUDIT_PATH),
        "independent_gold_tasks_sha256": sha256(GOLD_TASKS_PATH),
        "task_count": len(blind),
        "asset_count": len(by_asset),
        "preview_count": len(preview_files),
        "independent_gold_task_count": len(gold_tasks),
        "independent_gold_core_ready_task_count": sum(
            row["core_taxonomy_annotation_ready"] for row in gold_tasks
        ),
        "independent_gold_exact_prompt_ready_task_count": sum(
            row["per_seed_optional_annotation_ready"] for row in gold_tasks
        ),
        "frozen_benchmark_input_image_count": sum(
            len(row["benchmark_input_images"]) for row in gold_tasks
        ),
        "frozen_benchmark_input_image_hashes": {
            path: digest
            for row in gold_tasks
            for path, digest in row["benchmark_input_image_hashes"].items()
        },
        "assigned_source_role_count": sum(row.get("assigned_source_role") is not None for row in audit),
        "unassigned_candidate_count": sum(row.get("assigned_source_role") is None for row in audit),
        "preview_total_bytes": sum(path.stat().st_size for path in preview_files),
        "preview_hashes": {path.name: sha256(path) for path in preview_files},
        "judge_templates": [str(JUDGE_ROOT / f"{name}.jsonl") for name in ("judge_a", "judge_b", "judge_c")],
        "independent_gold_complete": False,
        "three_judge_complete": False,
    }
    (OUTPUT_ROOT / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (OUTPUT_ROOT / "README.md").write_text(
        "# Nano3D Naming judge packet v1.1\n\n"
        "每个任务对应一个 N=32 source-semantic 资产的 GLB mesh node。预览包含完整资产中的红色目标和三个隔离视角。"
        "`blind_tasks.jsonl` 不包含自动 role assignment；`audit_tasks.jsonl` 仅用于事后审计，不应交给独立 judge。\n\n"
        "三名 judge 分别填写 `judges/judge_a.jsonl`、`judge_b.jsonl`、`judge_c.jsonl`。"
        "必须填写 `judge_verdict` 和 `judge_reason`；`spec_match` 还必须填写 `judge_matched_role`。\n\n"
        "重复角色（`min_count>1`）的 `judge_instance_id` 填从节点名和预览可区分的实例身份（如 `left/right` 或 `0/1`）；"
        "同一实例的碎片使用同一 ID；其他真实 verdict 填 `not_applicable`。"
        "真实节点的 `judge_same_semantic_part_as` 必须显式填 `none`，或填同资产中属于同一语义部件的另一节点名；"
        "invalid 填 `not_applicable`，uncertain 的附加字段保持 null。空值不代表 `none`。\n\n"
        "`independent_gold_annotation_template.jsonl` 是输出盲的 N=33 gold 标注模板。33/33 已冻结输出无关的类别文本；pictureX 类别另复制原始类别参考图，可开始 core-taxonomy 标注。"
        "现有 export 未保存逐 seed 的精确原始生成请求，因此 optional/per-seed gold 仍为 0/33 ready。\n\n"
        "当前 reference roles 仍来自 source-derived gold，不是独立 hidden gold；完成三 judge 只能补 semantic validation，不能消除 gold 来源限制。\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                key: value
                for key, value in manifest.items()
                if key not in {"preview_hashes", "frozen_benchmark_input_image_hashes"}
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    print(f"output={OUTPUT_ROOT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
