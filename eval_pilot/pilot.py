#!/usr/bin/env python3
"""Small, reproducible Articraft evaluation smoke test.

The script intentionally keeps generated artifacts out of Git and keeps API
credentials in the environment. It can run all local stages without a key.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import html
import itertools
import json
import math
import os
import shutil
import subprocess
import sys
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

import networkx as nx
import numpy as np
from PIL import Image, ImageDraw

PILOT_ROOT = Path(__file__).resolve().parent
WORKSPACE = PILOT_ROOT.parent
TEMPLATE_REPO = WORKSPACE / "arti-template"
EXPORT_ROOT = WORKSPACE / "seed_exports"
MANIFEST_PATH = Path(os.environ.get("EVAL_MANIFEST", str(PILOT_ROOT / "manifest.jsonl")))
PROMPT_PATH = PILOT_ROOT / "prompt.md"
ARTIFACTS = Path(os.environ.get("EVAL_ARTIFACTS", str(PILOT_ROOT / "artifacts")))
METRICS_PATH = ARTIFACTS / "metrics.json"
RENDER_INDEX_PATH = ARTIFACTS / "render_index.json"
PAIRS_PATH = ARTIFACTS / "pairs.jsonl"
JUDGE_INDEX_PATH = ARTIFACTS / "judge_index.json"
DEFAULT_BASE_URL = "https://codex.ai02.cn/v1"
DEFAULT_MODEL = "gpt-5.4"
REASONING_EFFORT = os.environ.get("EVAL_REASONING_EFFORT", "").strip() or None
BLENDER_VIEW_NAMES = ("front_iso", "right_iso", "rear_iso", "left_iso")


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
        if not isinstance(row, dict):
            raise ValueError(f"{path}:{line_number}: expected an object")
        rows.append(row)
    return rows


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows)
    path.write_text(text, encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def image_is_readable(path: Path) -> bool:
    if not path.is_file() or path.stat().st_size == 0:
        return False
    try:
        with Image.open(path) as image:
            image.verify()
        return True
    except Exception:
        return False


def image_has_visible_content(path: Path) -> bool:
    if not image_is_readable(path):
        return False
    with Image.open(path).convert("RGB") as image:
        extrema = image.getextrema()
    return max(high for _low, high in extrema) > 12


def _urdf_path(row: dict[str, Any]) -> Path:
    explicit = row.get("urdf")
    if explicit:
        path = Path(str(explicit))
        return path if path.is_absolute() else WORKSPACE / path
    return EXPORT_ROOT / str(row["slug"]) / f"seed_{int(row['seed'])}" / "model.urdf"


def urdf_graph(path: Path) -> tuple[nx.DiGraph, dict[str, Any]]:
    root = ET.parse(path).getroot()
    links = [str(node.attrib.get("name", "")) for node in root.findall("link")]
    graph = nx.DiGraph()
    for name in links:
        graph.add_node(name, incoming_joint="root")

    joint_types: dict[str, int] = defaultdict(int)
    moving_joint_types: dict[str, int] = defaultdict(int)
    for joint in root.findall("joint"):
        joint_type = str(joint.attrib.get("type", "unknown"))
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        if parent_node is None or child_node is None:
            continue
        parent = str(parent_node.attrib.get("link", ""))
        child = str(child_node.attrib.get("link", ""))
        graph.add_node(
            parent, incoming_joint=graph.nodes.get(parent, {}).get("incoming_joint", "root")
        )
        graph.add_node(child, incoming_joint=joint_type)
        graph.add_edge(parent, child, joint_type=joint_type)
        joint_types[joint_type] += 1
        if joint_type != "fixed":
            moving_joint_types[joint_type] += 1

    roots = [node for node, degree in graph.in_degree() if degree == 0]
    max_depth = 0
    for candidate in roots:
        lengths = nx.single_source_shortest_path_length(graph, candidate)
        max_depth = max(max_depth, max(lengths.values(), default=0))

    summary = {
        "links": graph.number_of_nodes(),
        "all_joints": graph.number_of_edges(),
        "movable_joints": sum(moving_joint_types.values()),
        "joint_types": dict(sorted(joint_types.items())),
        "movable_joint_types": dict(sorted(moving_joint_types.items())),
        "roots": len(roots),
        "max_depth": max_depth,
    }
    return graph, summary


def pilot_tree_edit_distance(left: nx.DiGraph, right: nx.DiGraph) -> float | None:
    """Exact small-graph GED with incoming joint type as the node label.

    Infinite Mobility does not publish enough implementation detail to claim
    metric identity, so this value is explicitly named a pilot TED in reports.
    """

    distance = nx.graph_edit_distance(
        left,
        right,
        node_subst_cost=lambda a, b: (
            0.0 if a.get("incoming_joint") == b.get("incoming_joint") else 1.0
        ),
        node_del_cost=lambda _a: 1.0,
        node_ins_cost=lambda _a: 1.0,
        edge_subst_cost=lambda a, b: 0.0 if a.get("joint_type") == b.get("joint_type") else 1.0,
        edge_del_cost=lambda _a: 1.0,
        edge_ins_cost=lambda _a: 1.0,
        timeout=10.0,
    )
    return None if distance is None else float(distance)


def run_metrics() -> dict[str, Any]:
    assets = read_jsonl(MANIFEST_PATH)
    asset_rows: list[dict[str, Any]] = []
    graphs: dict[str, nx.DiGraph] = {}
    by_category: dict[str, list[dict[str, Any]]] = defaultdict(list)

    for asset in assets:
        path = _urdf_path(asset)
        if not path.is_file():
            row = {**asset, "status": "missing", "urdf": str(path)}
        else:
            graph, summary = urdf_graph(path)
            graphs[str(asset["asset_id"])] = graph
            row = {
                **asset,
                "status": "ok",
                "urdf": str(path.relative_to(WORKSPACE)),
                "urdf_sha256": sha256_file(path),
                **summary,
            }
        asset_rows.append(row)
        by_category[str(asset["category"])].append(row)

    category_rows: list[dict[str, Any]] = []
    all_pair_distances: list[float] = []
    for category, rows in sorted(by_category.items()):
        valid = [row for row in rows if row["status"] == "ok"]
        movable = [float(row["movable_joints"]) for row in valid]
        all_joints = [float(row["all_joints"]) for row in valid]
        pair_rows: list[dict[str, Any]] = []
        for left, right in itertools.combinations(valid, 2):
            distance = pilot_tree_edit_distance(
                graphs[str(left["asset_id"])], graphs[str(right["asset_id"])]
            )
            pair_rows.append(
                {
                    "left": left["asset_id"],
                    "right": right["asset_id"],
                    "pilot_ted": distance,
                }
            )
            if distance is not None:
                all_pair_distances.append(distance)
        distances = [float(row["pilot_ted"]) for row in pair_rows if row["pilot_ted"] is not None]
        category_rows.append(
            {
                "category": category,
                "assets": len(valid),
                "movable_joint_mean": float(np.mean(movable)) if movable else None,
                "movable_joint_variance": float(np.var(movable)) if movable else None,
                "all_joint_mean": float(np.mean(all_joints)) if all_joints else None,
                "all_joint_variance": float(np.var(all_joints)) if all_joints else None,
                "pilot_ted_mean": float(np.mean(distances)) if distances else None,
                "pairs": pair_rows,
            }
        )

    payload = {
        "protocol": "smoke_v1_existing_coverage_exports",
        "warning": (
            "Directional smoke-test only: existing coverage-picked seeds and a pilot GED; "
            "not directly comparable to Infinite Mobility Table 3."
        ),
        "assets": asset_rows,
        "categories": category_rows,
        "macro": {
            "categories": len(category_rows),
            "assets_ok": sum(row["status"] == "ok" for row in asset_rows),
            "movable_joint_mean": float(
                np.mean([row["movable_joints"] for row in asset_rows if row["status"] == "ok"])
            ),
            "movable_joint_variance": float(
                np.var([row["movable_joints"] for row in asset_rows if row["status"] == "ok"])
            ),
            "pilot_ted_mean": float(np.mean(all_pair_distances)) if all_pair_distances else None,
        },
    }
    write_json(METRICS_PATH, payload)
    print(json.dumps(payload["macro"], indent=2, sort_keys=True))
    print(METRICS_PATH)
    return payload


def _normal_render(
    meshes: list[Any],
    direction_tuple: tuple[float, float, float],
    out: Path,
) -> None:
    import trimesh
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

    if not meshes:
        raise ValueError("no renderable meshes")
    direction = np.asarray(direction_tuple, dtype=float)
    direction /= np.linalg.norm(direction)
    forward = -direction
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    up /= np.linalg.norm(up)

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.5, 0.5, 0.5)
    for mesh in meshes:
        vertices = np.ascontiguousarray(np.asarray(mesh.vertices, dtype=np.float64))
        faces = np.ascontiguousarray(np.asarray(mesh.faces, dtype=np.int64))
        normals = np.asarray(mesh.face_normals, dtype=float)
        camera_normals = np.column_stack((normals @ right, normals @ up, normals @ direction))
        camera_normals /= np.maximum(np.linalg.norm(camera_normals, axis=1, keepdims=True), 1e-9)
        colors = np.ascontiguousarray(
            np.clip((camera_normals * 0.5 + 0.5) * 255.0, 0, 255).astype(np.uint8)
        )

        points = vtk.vtkPoints()
        points.SetData(numpy_to_vtk(vertices, deep=True))
        cell_data = np.empty((len(faces), 4), dtype=np.int64)
        cell_data[:, 0] = 3
        cell_data[:, 1:] = faces
        cells = vtk.vtkCellArray()
        cells.SetCells(len(faces), numpy_to_vtkIdTypeArray(cell_data.ravel(), deep=True))
        poly = vtk.vtkPolyData()
        poly.SetPoints(points)
        poly.SetPolys(cells)
        vtk_colors = numpy_to_vtk(colors, deep=True, array_type=vtk.VTK_UNSIGNED_CHAR)
        vtk_colors.SetName("camera_normal_rgb")
        vtk_colors.SetNumberOfComponents(3)
        poly.GetCellData().SetScalars(vtk_colors)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)
        mapper.SetScalarModeToUseCellData()
        mapper.SetColorModeToDirectScalars()
        mapper.ScalarVisibilityOn()
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().LightingOff()
        actor.GetProperty().BackfaceCullingOn()
        renderer.AddActor(actor)

    combined = trimesh.util.concatenate(meshes)
    center = np.asarray(combined.bounding_box.centroid, dtype=float)
    radius = max(float(np.linalg.norm(combined.bounding_box.extents)) * 0.5, 1e-3)
    distance = radius / math.sin(math.radians(42.0 / 2.0)) * 1.16
    camera = renderer.GetActiveCamera()
    camera.SetPosition(*(center + direction * distance))
    camera.SetFocalPoint(*center)
    camera.SetViewUp(0.0, 0.0, 1.0)
    camera.SetViewAngle(42.0)

    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(900, 700)
    window.AddRenderer(renderer)
    renderer.ResetCameraClippingRange()
    window.Render()
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetInputBufferTypeToRGB()
    capture.ReadFrontBufferOff()
    capture.Update()
    writer = vtk.vtkPNGWriter()
    out.parent.mkdir(parents=True, exist_ok=True)
    writer.SetFileName(str(out))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()
    renderer.RemoveAllViewProps()
    window.Finalize()


def _render_normals(asset: dict[str, Any], raw_root: Path) -> list[Path]:
    if str(TEMPLATE_REPO) not in sys.path:
        sys.path.insert(0, str(TEMPLATE_REPO))

    import importlib

    from agent.template_registry import TEMPLATE_REGISTRY
    from scripts.render_template_previews import (
        _joint_pose,
        _matrix,
        _mesh_geometry,
        _origin_to_mat4,
        _semantic_view_directions,
    )
    from sdk import AssetContext
    from sdk._core.v0.geometry_qc import compute_part_world_transforms

    slug = str(asset["slug"])
    seed = int(asset["seed"])
    module = importlib.import_module(f"agent.templates.{slug}")
    stem = TEMPLATE_REGISTRY[slug]
    config = module.config_from_seed(seed)
    assets = AssetContext(raw_root / "assets" / f"{slug}_seed_{seed}_normal")
    model = getattr(module, f"build_{stem}")(config, assets=assets)
    world = compute_part_world_transforms(model, _joint_pose(model, "closed"))
    meshes: list[Any] = []
    for part in model.parts:
        part_tf = world.get(part.name)
        if part_tf is None:
            continue
        for visual in part.visuals:
            mesh = _mesh_geometry(visual.geometry, assets=getattr(part, "assets", None) or assets)
            if mesh is None:
                continue
            mesh = mesh.copy()
            mesh.apply_transform(_matrix(_origin_to_mat4(visual.origin)))
            mesh.apply_transform(_matrix(part_tf))
            meshes.append(mesh)

    directions = _semantic_view_directions(module)
    paths: list[Path] = []
    for view_name in ("front_iso", "rear_iso"):
        out = raw_root / slug / f"seed_{seed}_closed_{view_name}_normal.png"
        _normal_render(meshes, directions[view_name], out)
        paths.append(out)
    return paths


def _find_blender() -> str:
    candidates = [
        os.environ.get("BLENDER_BIN", ""),
        shutil.which("blender") or "",
        "/root/tools/blender-5.1.1-linux-x64/blender",
        "/home/artgen/blender/blender-4.2.9-linux-x64/blender",
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file() and os.access(candidate, os.X_OK):
            return candidate
    raise SystemExit("Blender binary not found; set BLENDER_BIN")


def _compact_blender_job(job: dict[str, Any], job_dir: Path) -> None:
    """Bake the fixed closed pose and merge visuals by material.

    Door templates can contain dozens of tiny CAD visuals.  Importing each as
    a separate Blender object is needlessly expensive for a still evaluation;
    the closed-pose job can safely bake link transforms and merge equal
    materials into a handful of meshes.
    """
    import trimesh

    groups: dict[str, list[Any]] = defaultdict(list)
    specs: dict[str, dict[str, Any]] = {}
    for visual in job["visuals"]:
        mesh = trimesh.load(job_dir / visual["mesh"], force="mesh", process=False)
        part_index = int(visual["part"])
        transform = np.asarray(job["parts_world"][part_index][0], dtype=float).reshape(4, 4)
        mesh.apply_transform(transform)
        key = json.dumps(visual["spec"], sort_keys=True, separators=(",", ":"))
        groups[key].append(mesh)
        specs[key] = visual["spec"]

    merged_dir = job_dir / "merged_meshes"
    merged_dir.mkdir(parents=True, exist_ok=True)
    visuals: list[dict[str, Any]] = []
    for index, (key, meshes) in enumerate(sorted(groups.items())):
        path = merged_dir / f"visual_{index:03d}.ply"
        trimesh.util.concatenate(meshes).export(path)
        visuals.append({"mesh": str(path.relative_to(job_dir)), "spec": specs[key], "part": 0})
    job["visuals"] = visuals
    job["parts_world"] = [[np.eye(4, dtype=float).reshape(-1).tolist()]]
    (job_dir / "job.json").write_text(json.dumps(job), encoding="utf-8")


def _render_blender_asset(asset: dict[str, Any], raw_root: Path) -> tuple[list[Path], list[Path]]:
    """Build and render one closed-pose asset with Blender Eevee."""
    if str(TEMPLATE_REPO) not in sys.path:
        sys.path.insert(0, str(TEMPLATE_REPO))
    import importlib

    from scripts.render_template_previews import _semantic_view_directions
    from scripts.render_template_videos_blender import _build_job

    slug = str(asset["slug"])
    seed = int(asset["seed"])
    module = importlib.import_module(f"agent.templates.{slug}")
    directions = _semantic_view_directions(module)
    front = np.asarray(directions["front_iso"], dtype=float)
    job_dir = raw_root / "_blender_jobs" / f"{slug}_seed_{seed}"
    frames = job_dir / "frames"
    complete = all(
        image_has_visible_content(frames / f"{prefix}_{index:02d}.png")
        for prefix in ("rgb", "normal")
        for index in range(len(BLENDER_VIEW_NAMES))
    )
    if not complete:
        if job_dir.exists():
            shutil.rmtree(job_dir)
        job_dir.mkdir(parents=True, exist_ok=True)
        job = _build_job(
            slug,
            seed,
            raw_root,
            job_dir,
            frames=len(BLENDER_VIEW_NAMES),
            view="front_iso",
            view_dir=front.tolist(),
            orbit_revolutions=1.0,
            pose_phase=0.0,
            color_mode="material",
            width=384,
            height=384,
            samples=1,
        )
        if job is None:
            raise RuntimeError(f"no renderable meshes for {slug} seed={seed}")
        _compact_blender_job(job, job_dir)
        worker = PILOT_ROOT / "blender_worker.py"
        command = [
            _find_blender(),
            "-b",
            "--factory-startup",
            "-noaudio",
            "-P",
            str(worker),
            "--",
            str(job_dir),
        ]
        proc = subprocess.run(command, capture_output=True, text=True, cwd=str(WORKSPACE))
        if proc.returncode != 0:
            detail = (proc.stderr or proc.stdout)[-2000:]
            raise RuntimeError(f"Blender failed for {slug} seed={seed}: {detail}")
    (raw_root / slug).mkdir(parents=True, exist_ok=True)
    rgb_views = []
    normal_views = []
    for index, view_name in enumerate(BLENDER_VIEW_NAMES):
        rgb = raw_root / slug / f"seed_{seed}_closed_{view_name}.png"
        normal = raw_root / slug / f"seed_{seed}_closed_{view_name}_normal.png"
        shutil.copy2(frames / f"rgb_{index:02d}.png", rgb)
        shutil.copy2(frames / f"normal_{index:02d}.png", normal)
        rgb_views.append(rgb)
        normal_views.append(normal)
    if not complete:
        (job_dir / "blender.log").write_text(
            "$ " + " ".join(command) + "\n\n" + proc.stdout + "\n" + proc.stderr,
            encoding="utf-8",
        )
    return rgb_views, normal_views


def _run_render_blender() -> dict[str, Any]:
    raw_root = ARTIFACTS / "renders" / "raw"
    sheets_root = ARTIFACTS / "renders" / "sheets"
    index: dict[str, Any] = {}
    for asset in read_jsonl(MANIFEST_PATH):
        asset_id = str(asset["asset_id"])
        rgb_views, normal_views = _render_blender_asset(asset, raw_root)
        rgb_sheet = sheets_root / f"{asset_id}_rgb.png"
        normal_sheet = sheets_root / f"{asset_id}_normal.png"
        from scripts.render_template_previews import make_contact_sheet

        make_contact_sheet(rgb_views, rgb_sheet)
        try:
            make_contact_sheet(normal_views, normal_sheet)
        except Exception as exc:
            # A nearly planar asset can have a valid but low-contrast normal map.
            # Keep the sheet when it contains non-black pixels; RGB remains strict.
            if not image_has_visible_content(normal_sheet):
                raise
            print(f"accepted low-contrast normal sheet for {asset_id}: {exc}")
        index[asset_id] = {
            **asset,
            "renderer": "blender_cycles",
            "views": list(BLENDER_VIEW_NAMES),
            "rgb_sheet": str(rgb_sheet.relative_to(PILOT_ROOT)),
            "normal_sheet": str(normal_sheet.relative_to(PILOT_ROOT)),
            "rgb_sha256": sha256_file(rgb_sheet),
            "normal_sha256": sha256_file(normal_sheet),
        }
        print(f"rendered {asset_id} with Blender Cycles")
    write_json(RENDER_INDEX_PATH, index)
    print(RENDER_INDEX_PATH)
    return index


def run_render(renderer: str) -> dict[str, Any]:
    if str(TEMPLATE_REPO) not in sys.path:
        sys.path.insert(0, str(TEMPLATE_REPO))
    if renderer == "blender":
        return _run_render_blender()

    from scripts.render_template_previews import (
        _render_scene,
        headless_render_display,
        make_contact_sheet,
        render_model_previews,
    )

    _render_scene.renderer_mode = renderer  # type: ignore[attr-defined]
    raw_root = ARTIFACTS / "renders" / "raw"
    sheets_root = ARTIFACTS / "renders" / "sheets"
    index: dict[str, Any] = {}
    with headless_render_display(renderer):
        for asset in read_jsonl(MANIFEST_PATH):
            asset_id = str(asset["asset_id"])
            slug, seed = str(asset["slug"]), int(asset["seed"])
            rgb_sheet = sheets_root / f"{asset_id}_rgb.png"
            normal_sheet = sheets_root / f"{asset_id}_normal.png"
            if image_is_readable(rgb_sheet) and image_is_readable(normal_sheet):
                print(f"reused {asset_id}")
            else:
                rgb_views = render_model_previews(
                    slug,
                    seed,
                    raw_root,
                    poses=("closed",),
                    views=("front_iso", "rear_iso"),
                )
                normal_views = _render_normals(asset, raw_root)
                make_contact_sheet(rgb_views, rgb_sheet)
                make_contact_sheet(normal_views, normal_sheet)
                print(f"rendered {asset_id}")
            index[asset_id] = {
                **asset,
                "rgb_sheet": str(rgb_sheet.relative_to(PILOT_ROOT)),
                "normal_sheet": str(normal_sheet.relative_to(PILOT_ROOT)),
                "rgb_sha256": sha256_file(rgb_sheet),
                "normal_sha256": sha256_file(normal_sheet),
            }
    write_json(RENDER_INDEX_PATH, index)
    print(RENDER_INDEX_PATH)
    return index


def run_pairs() -> list[dict[str, Any]]:
    if not RENDER_INDEX_PATH.is_file():
        raise SystemExit("render index missing; run `pilot.py render` first")
    index = json.loads(RENDER_INDEX_PATH.read_text(encoding="utf-8"))
    by_category: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in index.values():
        by_category[str(row.get("pair_group", row["category"]))].append(row)

    pairs: list[dict[str, Any]] = []
    for pair_group, rows in sorted(by_category.items()):
        rows.sort(key=lambda row: str(row["asset_id"]))
        for left, right in itertools.combinations(rows, 2):
            pair_id = f"{pair_group}__{left['asset_id']}__{right['asset_id']}"
            # Deterministic initial order; the judge also runs the reverse order.
            if int(hashlib.sha256(pair_id.encode()).hexdigest()[:2], 16) % 2:
                left, right = right, left
            pairs.append(
                {
                    "pair_id": pair_id,
                    "category": left["category"],
                    "pair_group": pair_group,
                    "pair_kind": "smoke_intra_ours",
                    "left_asset_id": left["asset_id"],
                    "right_asset_id": right["asset_id"],
                    "left_source": left["source"],
                    "right_source": right["source"],
                    "left_rgb": left["rgb_sheet"],
                    "left_normal": left["normal_sheet"],
                    "right_rgb": right["rgb_sheet"],
                    "right_normal": right["normal_sheet"],
                }
            )
    write_jsonl(PAIRS_PATH, pairs)
    print(f"wrote {len(pairs)} pairs to {PAIRS_PATH}")
    return pairs


def _api_key() -> str:
    key = os.environ.get("OPENAI_API_KEY", "").strip()
    if not key:
        raise SystemExit(
            "OPENAI_API_KEY is not set. Inject it in the environment; never put it in the repo."
        )
    return key


def _base_url() -> str:
    return os.environ.get("OPENAI_BASE_URL", DEFAULT_BASE_URL).rstrip("/")


def _request_json(method: str, url: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=body,
        method=method,
        headers={
            "Authorization": f"Bearer {_api_key()}",
            "Content-Type": "application/json",
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=90) as response:
            result = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")[:1200]
        raise SystemExit(f"API HTTP {exc.code}: {detail}") from exc
    except urllib.error.URLError as exc:
        raise SystemExit(f"API request failed: {exc}") from exc
    if not isinstance(result, dict):
        raise SystemExit("API returned a non-object JSON response")
    return result


def _vision_name_candidate(model_id: str) -> bool:
    value = model_id.lower()
    return any(token in value for token in ("gpt-4v", "vision", "gpt-4o", "gpt-4.1"))


def discover_models() -> list[str]:
    response = _request_json("GET", f"{_base_url()}/models")
    rows = response.get("data", [])
    model_ids = sorted(
        str(row.get("id")) for row in rows if isinstance(row, dict) and row.get("id")
    )
    candidates = [model_id for model_id in model_ids if _vision_name_candidate(model_id)]
    exact_4v = [
        model_id for model_id in model_ids if model_id.lower() in {"gpt-4v", "gpt-4-vision-preview"}
    ]
    print(
        json.dumps(
            {
                "exact_gpt4v": exact_4v,
                "vision_name_candidates": candidates,
                "all_models": model_ids,
            },
            indent=2,
        )
    )
    return model_ids


def _data_url(path: Path) -> str:
    mime = "image/png" if path.suffix.lower() == ".png" else "image/jpeg"
    return f"data:{mime};base64,{base64.b64encode(path.read_bytes()).decode('ascii')}"


def probe_model(model: str) -> dict[str, Any]:
    probe_path = ARTIFACTS / "vision_probe.png"
    probe_path.parent.mkdir(parents=True, exist_ok=True)
    probe = Image.new("RGB", (32, 32), (235, 40, 40))
    ImageDraw.Draw(probe).rectangle((16, 0, 31, 31), fill=(40, 80, 235))
    probe.save(probe_path)
    payload = {
        "model": model,
        "temperature": 0,
        "max_tokens": 40,
        "messages": [
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": "Reply with only the two dominant colors in this image.",
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": _data_url(probe_path), "detail": "low"},
                    },
                ],
            }
        ],
    }
    if REASONING_EFFORT:
        payload["reasoning_effort"] = REASONING_EFFORT
    response = _request_json("POST", f"{_base_url()}/chat/completions", payload)
    content = response.get("choices", [{}])[0].get("message", {}).get("content")
    result = {"model": model, "vision_request_succeeded": bool(content), "response": content}
    print(json.dumps(result, indent=2))
    return result


def _extract_json(content: Any) -> dict[str, Any]:
    if isinstance(content, list):
        content = "".join(
            str(item.get("text", "")) if isinstance(item, dict) else str(item) for item in content
        )
    text = str(content or "").strip()
    if text.startswith("```"):
        text = text.removeprefix("```json").removeprefix("```")
        text = text.removesuffix("```").strip()
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end < start:
        raise ValueError(f"response did not contain JSON: {text[:300]}")
    parsed = json.loads(text[start : end + 1])
    for dimension in ("geometry", "appearance"):
        winner = str(parsed.get(dimension, {}).get("winner", "")).upper()
        if winner not in {"A", "B", "TIE"}:
            raise ValueError(f"invalid {dimension} winner: {winner!r}")
        parsed[dimension]["winner"] = winner
    return parsed


def _judge_payload(model: str, prompt: str, pair: dict[str, Any], order: str) -> dict[str, Any]:
    if order == "ab":
        a_prefix, b_prefix = "left", "right"
    else:
        a_prefix, b_prefix = "right", "left"

    def path(key: str) -> Path:
        return PILOT_ROOT / str(pair[key])

    content: list[dict[str, Any]] = [
        {"type": "text", "text": f"Target category: {pair['category']}. Asset A RGB:"},
        {
            "type": "image_url",
            "image_url": {"url": _data_url(path(f"{a_prefix}_rgb")), "detail": "low"},
        },
        {"type": "text", "text": "Asset A normal map:"},
        {
            "type": "image_url",
            "image_url": {"url": _data_url(path(f"{a_prefix}_normal")), "detail": "low"},
        },
        {"type": "text", "text": "Asset B RGB:"},
        {
            "type": "image_url",
            "image_url": {"url": _data_url(path(f"{b_prefix}_rgb")), "detail": "low"},
        },
        {"type": "text", "text": "Asset B normal map:"},
        {
            "type": "image_url",
            "image_url": {"url": _data_url(path(f"{b_prefix}_normal")), "detail": "low"},
        },
    ]
    payload = {
        "model": model,
        "temperature": 0,
        "max_tokens": 600,
        "messages": [
            {"role": "system", "content": prompt},
            {"role": "user", "content": content},
        ],
    }
    if REASONING_EFFORT:
        payload["reasoning_effort"] = REASONING_EFFORT
    return payload


def _cache_key(model: str, prompt: str, pair: dict[str, Any], order: str) -> str:
    image_hashes = []
    for key in ("left_rgb", "left_normal", "right_rgb", "right_normal"):
        image_hashes.append(sha256_file(PILOT_ROOT / str(pair[key])))
    canonical = json.dumps(
        {
            "base_url": _base_url(),
            "model": model,
            "reasoning_effort": REASONING_EFFORT,
            "prompt_sha256": hashlib.sha256(prompt.encode()).hexdigest(),
            "pair_id": pair["pair_id"],
            "order": order,
            "images": image_hashes,
        },
        sort_keys=True,
    )
    return hashlib.sha256(canonical.encode()).hexdigest()


def run_judge(model: str, limit: int | None) -> list[dict[str, Any]]:
    _api_key()
    if not PAIRS_PATH.is_file():
        raise SystemExit("pairs missing; run `pilot.py pairs` first")
    prompt = PROMPT_PATH.read_text(encoding="utf-8")
    pairs = read_jsonl(PAIRS_PATH)
    if limit is not None:
        pairs = pairs[:limit]
    cache_root = ARTIFACTS / "api_cache"
    results: list[dict[str, Any]] = []
    for pair in pairs:
        for order in ("ab", "ba"):
            key = _cache_key(model, prompt, pair, order)
            cache_path = cache_root / f"{key}.json"
            if cache_path.is_file():
                record = json.loads(cache_path.read_text(encoding="utf-8"))
                print(f"cache hit {pair['pair_id']} {order}")
            else:
                payload = _judge_payload(model, prompt, pair, order)
                response = _request_json("POST", f"{_base_url()}/chat/completions", payload)
                content = response.get("choices", [{}])[0].get("message", {}).get("content")
                parsed = _extract_json(content)
                record = {
                    "pair_id": pair["pair_id"],
                    "order": order,
                    "model": model,
                    "reasoning_effort": REASONING_EFFORT,
                    "base_url": _base_url(),
                    "prompt_sha256": hashlib.sha256(prompt.encode()).hexdigest(),
                    "parsed": parsed,
                    "usage": response.get("usage"),
                    "response_id": response.get("id"),
                }
                write_json(cache_path, record)
                print(f"judged {pair['pair_id']} {order}")
            results.append(record)
    write_json(JUDGE_INDEX_PATH, results)
    print(JUDGE_INDEX_PATH)
    return results


def _normalized_winner(record: dict[str, Any], dimension: str) -> str:
    winner = str(record["parsed"][dimension]["winner"])
    if winner == "TIE":
        return "TIE"
    if record["order"] == "ab":
        return "left" if winner == "A" else "right"
    return "right" if winner == "A" else "left"


def aggregate_judgments(
    pairs: list[dict[str, Any]], results: list[dict[str, Any]]
) -> tuple[dict[str, Any], dict[str, Any]]:
    by_pair: dict[str, dict[str, dict[str, Any]]] = defaultdict(dict)
    for record in results:
        by_pair[str(record["pair_id"])][str(record["order"])] = record
    pair_map = {str(pair["pair_id"]): pair for pair in pairs}
    details: dict[str, Any] = {}
    counts = {dimension: defaultdict(int) for dimension in ("geometry", "appearance")}
    for pair_id, orders in by_pair.items():
        if "ab" not in orders or "ba" not in orders:
            continue
        pair = pair_map[pair_id]
        detail: dict[str, Any] = {}
        for dimension in ("geometry", "appearance"):
            first = _normalized_winner(orders["ab"], dimension)
            second = _normalized_winner(orders["ba"], dimension)
            if first != second:
                verdict = "position_sensitive"
            elif first == "TIE":
                verdict = "tie"
            else:
                winner_asset_id = pair[f"{first}_asset_id"]
                if pair["left_source"] == pair["right_source"]:
                    verdict = f"{winner_asset_id}_win"
                else:
                    verdict = f"{pair[f'{first}_source']}_win"
            counts[dimension][verdict] += 1
            detail[dimension] = {
                "ab_normalized": first,
                "ba_normalized": second,
                "verdict": verdict,
            }
            if first not in {"TIE"} and verdict != "position_sensitive":
                detail[dimension]["winner_asset_id"] = pair[f"{first}_asset_id"]
        details[pair_id] = detail
    return ({key: dict(value) for key, value in counts.items()}, details)


def run_report() -> Path:
    metrics = (
        json.loads(METRICS_PATH.read_text(encoding="utf-8")) if METRICS_PATH.is_file() else None
    )
    renders = (
        json.loads(RENDER_INDEX_PATH.read_text(encoding="utf-8"))
        if RENDER_INDEX_PATH.is_file()
        else {}
    )
    pairs = read_jsonl(PAIRS_PATH) if PAIRS_PATH.is_file() else []
    judge_results = (
        json.loads(JUDGE_INDEX_PATH.read_text(encoding="utf-8"))
        if JUDGE_INDEX_PATH.is_file()
        else []
    )
    judge_counts, judge_details = (
        aggregate_judgments(pairs, judge_results) if judge_results else ({}, {})
    )
    asset_count = len(renders)
    if not asset_count and metrics:
        asset_count = int(metrics.get("macro", {}).get("assets_ok", 0))

    parts = [
        "<!doctype html><html><head><meta charset='utf-8'><title>Articraft eval pilot</title>",
        "<style>body{font:15px system-ui;margin:32px;max-width:1200px;color:#20242a}"
        "table{border-collapse:collapse;margin:12px 0 28px}th,td{border:1px solid #ccd2da;padding:7px 10px}"
        ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:18px}"
        ".card{border:1px solid #ccd2da;border-radius:8px;padding:12px}img{max-width:100%;height:auto}"
        ".warning{background:#fff4ce;padding:12px;border-left:4px solid #c99000}</style></head><body>",
        "<h1>Articraft evaluation smoke test</h1>",
        f"<p class='warning'>Directional smoke-test only: {asset_count} existing coverage-picked exports, "
        "rendered with Blender Cycles; not an unbiased paper sample. Pilot TED is not claimed "
        "to reproduce Infinite Mobility's unpublished implementation details.</p>",
    ]

    if metrics:
        macro = metrics["macro"]
        parts.append("<h2>Structural snapshot</h2>")
        parts.append(
            "<p>Assets: {assets}; movable-joint mean: {mean:.3f}; variance: {variance:.3f}; "
            "pilot TED mean: {ted:.3f}</p>".format(
                assets=macro["assets_ok"],
                mean=macro["movable_joint_mean"],
                variance=macro["movable_joint_variance"],
                ted=macro["pilot_ted_mean"] or 0.0,
            )
        )
        parts.append(
            "<table><tr><th>Category</th><th>N</th><th>Movable joints mean</th><th>Variance</th><th>Pilot TED</th></tr>"
        )
        for row in metrics["categories"]:
            parts.append(
                f"<tr><td>{html.escape(row['category'])}</td><td>{row['assets']}</td>"
                f"<td>{row['movable_joint_mean']:.3f}</td><td>{row['movable_joint_variance']:.3f}</td>"
                f"<td>{(row['pilot_ted_mean'] or 0.0):.3f}</td></tr>"
            )
        parts.append("</table>")

    if judge_counts:
        parts.append("<h2>AB/BA VLM consistency</h2><pre>")
        parts.append(html.escape(json.dumps(judge_counts, indent=2, sort_keys=True)))
        parts.append("</pre>")

    if renders:
        parts.append("<h2>Rendered assets</h2><div class='grid'>")
        report_parent = ARTIFACTS
        for asset_id, row in sorted(renders.items()):
            rgb = os.path.relpath(PILOT_ROOT / row["rgb_sheet"], report_parent)
            normal = os.path.relpath(PILOT_ROOT / row["normal_sheet"], report_parent)
            parts.append(
                f"<div class='card'><h3>{html.escape(asset_id)}</h3>"
                f"<p>{html.escape(row['category'])} · seed {row['seed']}</p>"
                f"<img src='{html.escape(rgb)}' alt='RGB contact sheet'>"
                f"<img src='{html.escape(normal)}' alt='Normal contact sheet'></div>"
            )
        parts.append("</div>")

    if pairs:
        parts.append(
            "<h2>Pair status</h2><table><tr><th>Pair</th><th>Kind</th><th>Geometry</th><th>Appearance</th></tr>"
        )
        for pair in pairs:
            detail = judge_details.get(str(pair["pair_id"]), {})
            parts.append(
                f"<tr><td>{html.escape(str(pair['pair_id']))}</td>"
                f"<td>{html.escape(str(pair['pair_kind']))}</td>"
                f"<td>{html.escape(str(detail.get('geometry', {}).get('verdict', 'not run')))}</td>"
                f"<td>{html.escape(str(detail.get('appearance', {}).get('verdict', 'not run')))}</td></tr>"
            )
        parts.append("</table>")

    parts.append("</body></html>")
    out = ARTIFACTS / "report.html"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("".join(parts), encoding="utf-8")
    print(out)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("metrics")
    render_parser = sub.add_parser("render")
    render_parser.add_argument(
        "--renderer", choices=("blender", "vtk", "software", "egl"), default="blender"
    )
    sub.add_parser("pairs")
    sub.add_parser("discover-models")
    probe_parser = sub.add_parser("probe-model")
    probe_parser.add_argument("--model", default=DEFAULT_MODEL)
    judge_parser = sub.add_parser("judge")
    judge_parser.add_argument("--model", default=DEFAULT_MODEL)
    judge_parser.add_argument("--limit", type=int, default=None)
    sub.add_parser("report")
    args = parser.parse_args()

    if args.command == "metrics":
        run_metrics()
    elif args.command == "render":
        run_render(args.renderer)
    elif args.command == "pairs":
        run_pairs()
    elif args.command == "discover-models":
        discover_models()
    elif args.command == "probe-model":
        probe_model(args.model)
    elif args.command == "judge":
        run_judge(args.model, args.limit)
    elif args.command == "report":
        run_report()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
