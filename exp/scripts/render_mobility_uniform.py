#!/usr/bin/env python3
"""Render one deterministic representative per category for mobility datasets.

Supported datasets are ``infinigen`` (Infinigen-Sim, 17 categories) and
``physx`` (PhysX-Mobility, 132 categories). Both use the exact PV-A studio
contract and the audited URDF worker.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import re
import shlex
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

from PIL import Image


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_MANIFESTS = {
    "infinigen": REPO_ROOT / "exp/runtime/table123_full_release_20260825/rosters/infinigen/full_release_manifest.json",
    "physx": REPO_ROOT / "exp/runtime/table123_full_release_20260825/rosters/physx/full_release_manifest.json",
}
DEFAULT_OUTPUTS = {
    "infinigen": Path("/mnt/zsn/data/particulate/datasets/Infinigen-Sim/renders/uniform17_one_per_category_studio_256_v1"),
    "physx": Path("/mnt/zsn/data/particulate/datasets/PhysX-Mobility/renders/uniform132_one_per_category_studio_256_v1"),
}
EXPECTED = {"infinigen": ("Infinigen-Sim", 8226, 17), "physx": ("PhysX-Mobility", 2024, 132)}
WORKER = REPO_ROOT / "exp/scripts/render_mobility_asset_blender.py"
SUPPORT_RENDERER = REPO_ROOT / "exp/scripts/render_partnet_mobility_asset_blender.py"
BASE_RENDERER = REPO_ROOT / "exp/scripts/render_articraft10k_asset_blender.py"
SHARED_RENDERER = REPO_ROOT / "arti-template/scripts/render_exported_asset_blender.py"
BLENDER_DEFAULT = Path("/mnt/zsn/zsn_workspace/VoxHammer/third_party/blender-4.2.19-linux-x64/blender")
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def valid_png(path: Path, resolution: int) -> bool:
    try:
        if not path.is_file() or path.stat().st_size <= len(PNG_SIGNATURE):
            return False
        with path.open("rb") as stream:
            if stream.read(len(PNG_SIGNATURE)) != PNG_SIGNATURE:
                return False
        with Image.open(path) as image:
            image.load()
            return image.size == (resolution, resolution) and image.mode in {"RGB", "RGBA"}
    except (OSError, ValueError):
        return False


def studio() -> dict[str, Any]:
    return {
        "mode": "opaque_studio", "cycles_denoising": True, "view_transform": "AgX",
        "look": "AgX - Medium High Contrast", "world_rgba": [0.80, 0.84, 0.90, 1.0],
        "world_strength": 0.55, "ground_rgba": [0.32, 0.35, 0.40, 1.0],
        "ground_roughness": 0.82, "camera_vertical_fov_degrees": 42.0,
        "camera_direction": [1.25, -1.35, 0.85],
        "camera_distance_policy": "bounding_sphere_auto_frame_1.18",
        "lights": [
            {"direction": [0.4, -0.8, 1.5], "gain": 42.0, "size_ratio": 1.5},
            {"direction": [-1.2, -0.3, 0.6], "gain": 15.0, "size_ratio": 1.8},
            {"direction": [0.2, 1.0, 1.2], "gain": 24.0, "size_ratio": 1.2},
        ],
    }


def _safe_path(root: Path, value: str) -> Path:
    path = Path(value).expanduser().resolve(strict=True)
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"manifest path escapes source root: {path}") from exc
    return path


def dependency_receipt(source_root: Path, urdf_path: Path) -> dict[str, Any]:
    """Hash the URDF and its transitively referenced OBJ/MTL/textures."""
    source_root = source_root.resolve(strict=True)
    urdf_path = _safe_path(source_root, str(urdf_path))
    pending = [urdf_path]
    files: set[Path] = {urdf_path}

    def add(parent: Path, value: str) -> None:
        candidate = _safe_path(source_root, str(parent / value))
        if not candidate.is_file():
            raise ValueError(f"render dependency is not a regular file: {candidate}")
        if candidate not in files:
            files.add(candidate)
            pending.append(candidate)

    def add_joined(parent: Path, parts: Sequence[str], *, allow_options: bool) -> None:
        starts = range(0, len(parts)) if allow_options else range(0, 1)
        matches: list[str] = []
        for start in starts:
            value = " ".join(parts[start:])
            try:
                candidate = (parent / value).resolve(strict=True)
                candidate.relative_to(source_root)
            except (OSError, ValueError):
                continue
            if candidate.is_file():
                matches.append(value)
        if len(matches) != 1:
            raise ValueError(f"dependency reference resolves to {len(matches)} files: {parts!r}")
        add(parent, matches[0])

    root = ET.parse(urdf_path).getroot()
    for mesh in root.findall(".//visual/geometry/mesh"):
        value = str(mesh.get("filename") or "").strip()
        if not value or "\\" in value or "://" in value or Path(value).is_absolute():
            raise ValueError(f"unsafe URDF visual mesh path in {urdf_path}: {value!r}")
        add(urdf_path.parent, value)

    texture_commands = {"map_ka", "map_kd", "map_ks", "map_ke", "map_ns", "map_d", "bump", "map_bump", "disp", "decal", "refl", "norm", "map_pr"}
    cursor = 0
    while cursor < len(pending):
        path = pending[cursor]; cursor += 1
        if path.suffix.lower() not in {".obj", ".mtl"}:
            continue
        for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
            stripped = raw.strip()
            if not stripped or stripped.startswith("#"):
                continue
            try:
                parts = shlex.split(stripped, comments=True, posix=True)
            except ValueError as exc:
                raise ValueError(f"cannot parse dependency line in {path}: {raw!r}") from exc
            if not parts:
                continue
            command = parts[0].lower()
            if path.suffix.lower() == ".obj" and command == "mtllib":
                add_joined(path.parent, parts[1:], allow_options=False)
            elif path.suffix.lower() == ".mtl" and command in texture_commands and len(parts) >= 2:
                add_joined(path.parent, parts[1:], allow_options=True)
    rows = [
        {"path": path.relative_to(source_root).as_posix(), "bytes": path.stat().st_size, "sha256": sha256(path)}
        for path in sorted(files, key=lambda value: value.relative_to(source_root).as_posix())
    ]
    return {
        "file_count": len(rows), "total_bytes": sum(row["bytes"] for row in rows),
        "content_manifest_sha256": canonical_sha(rows), "files": rows,
        "policy": "URDF visual dependency closure: URDF, OBJ mtllib, MTL texture references",
    }


def load_items(dataset: str, manifest_path: Path, output_root: Path, *, strict: bool = True) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    expected_name, expected_count, expected_categories = EXPECTED[dataset]
    manifest_path = manifest_path.expanduser().resolve(strict=True)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or manifest.get("schema_version") != "table123_full_release_manifest_v1":
        raise ValueError("unsupported frozen mobility manifest")
    if manifest.get("dataset") != expected_name:
        raise ValueError(f"expected {expected_name}, found {manifest.get('dataset')!r}")
    body = dict(manifest); declared = body.pop("manifest_content_sha256", None)
    if declared != canonical_sha(body):
        raise ValueError("manifest self-hash mismatch")
    rows = manifest.get("rows")
    if not isinstance(rows, list) or (strict and len(rows) != expected_count):
        raise ValueError(f"expected {expected_count} manifest rows")
    if manifest.get("roster_sha256") != canonical_sha(rows):
        raise ValueError("manifest roster hash mismatch")
    bindings = manifest.get("source_bindings")
    if not isinstance(bindings, list) or not bindings or not isinstance(bindings[0], Mapping):
        raise ValueError("manifest lacks source binding")
    source_root = Path(str(bindings[0].get("path") or "")).expanduser().resolve(strict=True)
    if not source_root.is_dir():
        raise ValueError(f"source root is not a directory: {source_root}")
    parsed: list[dict[str, Any]] = []
    categories: set[str] = set()
    for row in rows:
        if not isinstance(row, Mapping):
            raise ValueError("manifest row must be an object")
        category = str(row.get("category") or row.get("raw_category") or "").strip()
        asset_id = str(row.get("asset_id") or "").strip()
        rel = str(row.get("primary_urdf_relative_path") or "").strip()
        if not category or not asset_id or not rel or "\\" in rel or Path(rel).is_absolute() or ".." in Path(rel).parts:
            raise ValueError(f"unsafe manifest identity/path: {asset_id!r}")
        urdf = _safe_path(source_root, str(source_root / rel))
        if row.get("primary_urdf_sha256") != sha256(urdf):
            raise ValueError(f"URDF hash mismatch: {urdf}")
        if int(row.get("primary_urdf_bytes", row.get("primary_urdf_size", -1))) != urdf.stat().st_size:
            raise ValueError(f"URDF byte count mismatch: {urdf}")
        package = Path(str(row.get("source_path") or source_root)).expanduser().resolve(strict=True)
        try:
            package.relative_to(source_root)
        except ValueError as exc:
            raise ValueError(f"package escapes source root: {package}") from exc
        if not package.is_dir():
            raise ValueError(f"package is not a directory: {package}")
        categories.add(category)
        parsed.append({"category": category, "asset_id": asset_id, "ordinal": int(row.get("ordinal", len(parsed))), "source_path": package, "urdf_path": urdf, "urdf_sha256": sha256(urdf), "urdf_bytes": urdf.stat().st_size, "manifest_row": dict(row)})
    if strict and len(categories) != expected_categories:
        raise ValueError(f"expected {expected_categories} categories, found {len(categories)}")
    winners: dict[str, dict[str, Any]] = {}
    for item in parsed:
        old = winners.get(item["category"])
        key = (hashlib.sha256(item["asset_id"].encode()).hexdigest(), item["asset_id"])
        if old is None or key < (hashlib.sha256(old["asset_id"].encode()).hexdigest(), old["asset_id"]):
            winners[item["category"]] = item
    selected = [item for item in parsed if winners[item["category"]] is item]
    selected.sort(key=lambda item: item["category"])
    for index, item in enumerate(selected):
        item["ordinal"] = index
        item["category_one_shot"] = True
        slug = f"{index:03d}_{hashlib.sha256(item['category'].encode()).hexdigest()[:10]}"
        item["output_path"] = output_root / slug / "imgs" / "000.png"
    if len(selected) != expected_categories:
        raise ValueError("one-shot selection does not cover all categories")
    return {"dataset": expected_name, "source_root": source_root, "manifest_path": manifest_path, "manifest": manifest, "all_count": len(parsed), "category_count": len(categories)}, selected


def build_config(dataset: str, metadata: Mapping[str, Any], selected: Sequence[Mapping[str, Any]], args: argparse.Namespace, renderer: Path, base: Path, shared: Path, blender: Path) -> dict[str, Any]:
    selected_receipts = [
        {
            "category": item["category"], "dataset_id": item["asset_id"],
            "content_manifest_sha256": item["dependency_receipt"]["content_manifest_sha256"],
            "file_count": item["dependency_receipt"]["file_count"],
            "total_bytes": item["dependency_receipt"]["total_bytes"],
        }
        for item in selected
    ]
    selected_identities = [
        {
            "category": item["category"], "dataset_id": item["asset_id"],
            "identity_sha256": hashlib.sha256(item["asset_id"].encode("utf-8")).hexdigest(),
        }
        for item in selected
    ]
    return {
        "schema_version": 1, "dataset": metadata["dataset"],
        "render_contract": f"{dataset}_uniform_studio_v1",
        "selected_count": len(selected), "selected_category_count": len(selected),
        "model_count": metadata["all_count"], "category_count": metadata["category_count"],
        "universe_count": metadata["all_count"],
        "official_model_count": EXPECTED[dataset][1], "official_category_count": EXPECTED[dataset][2],
        "dataset_manifest": str(metadata["manifest_path"]), "dataset_manifest_sha256": sha256(metadata["manifest_path"]),
        "dataset_manifest_content_sha256": metadata["manifest"]["manifest_content_sha256"],
        "dataset_roster_sha256": metadata["manifest"]["roster_sha256"],
        "source_root": str(metadata["source_root"]), "output_root": str(args.output_root.expanduser().resolve()),
        "driver": str(SCRIPT), "driver_sha256": sha256(SCRIPT),
        "renderer": str(renderer), "renderer_sha256": sha256(renderer),
        "support_renderer": str(args.support_renderer),
        "support_renderer_sha256": sha256(args.support_renderer),
        "base_renderer": str(base), "base_renderer_sha256": sha256(base),
        "shared_renderer": str(shared), "shared_renderer_sha256": sha256(shared),
        "blender": str(blender), "blender_version": subprocess.run([str(blender), "--version"], check=True, capture_output=True, text=True, timeout=60).stdout.splitlines()[0].strip(),
        "resolution": args.resolution, "samples": args.samples, "gpu_visibility": str(args.gpu), "workers": args.workers, "timeout_seconds": args.timeout_seconds,
        "selection_policy": "one per category, minimum (SHA256(asset_id UTF-8), asset_id)",
        "selection_receipt_sha256": canonical_sha(selected_identities),
        "selection_receipt": selected_identities,
        "selected_input_receipt_sha256": canonical_sha(selected_receipts),
        "selected_input_receipts": selected_receipts,
        "pose_policy": "URDF rest pose; all movable joint coordinates are zero",
        "package_validation": "selected URDF visual dependency closure, byte count, and SHA-256",
        "image_layout": "stable category ordinal and category hash/imgs/000.png",
        "studio": studio(), "material_policy": "native OBJ/MTL diffuse materials, forced opaque; neutral fallback",
    }


def parse_worker_json(stdout: str) -> dict[str, Any]:
    for line in reversed(stdout.splitlines()):
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and "output" in value:
            return value
    raise RuntimeError("worker did not emit a JSON receipt")


def render_one(item: Mapping[str, Any], *, args: argparse.Namespace, config: Mapping[str, Any], prior: Mapping[str, str] | None) -> dict[str, Any]:
    output = Path(item["output_path"])
    receipt = item["dependency_receipt"]
    common = {"category": item["category"], "dataset_id": item["asset_id"], "ordinal": item["ordinal"], "source_path": str(item["source_path"]), "package_path": str(item["source_path"]), "urdf_path": str(item["urdf_path"]), "output_path": str(output), "image": str(output), "category_one_shot": True, "urdf_sha256": item["urdf_sha256"], "urdf_bytes": item["urdf_bytes"], "package_file_count": receipt["file_count"], "package_total_bytes": receipt["total_bytes"], "package_content_manifest_sha256": receipt["content_manifest_sha256"]}
    if not args.force and prior and prior.get("dataset_id") == item["asset_id"] and prior.get("urdf_sha256") == item["urdf_sha256"] and valid_png(output, args.resolution):
        digest = sha256(output)
        if prior.get("image_sha256", prior.get("png_sha256")) == digest:
            common.update({"status": "reused_valid", "image_bytes": output.stat().st_size, "png_bytes": output.stat().st_size, "image_sha256": digest, "png_sha256": digest, "renderer_result": json.loads(prior.get("renderer_result") or "null")})
            return common
    command = [str(args.blender), "-b", "--factory-startup", "-P", str(args.worker), "--", "--asset-dir", str(item["source_path"]), "--urdf-path", str(item["urdf_path"]), "--output", str(output), "--resolution", str(args.resolution), "--samples", str(args.samples), "--support-renderer", str(args.support_renderer), "--support-renderer-sha256", config["support_renderer_sha256"], "--base-renderer", str(args.base_renderer), "--base-renderer-sha256", config["base_renderer_sha256"], "--shared-renderer", str(args.shared_renderer), "--shared-renderer-sha256", config["shared_renderer_sha256"]]
    env = dict(os.environ); env["CUDA_VISIBLE_DEVICES"] = str(args.gpu)
    try:
        completed = subprocess.run(command, check=False, capture_output=True, text=True, timeout=args.timeout_seconds, env=env)
        if completed.returncode != 0:
            raise RuntimeError((completed.stderr or completed.stdout)[-4000:])
        receipt = parse_worker_json(completed.stdout)
        if receipt.get("asset_dir") != str(Path(item["source_path"]).resolve()) or receipt.get("urdf_path") != str(Path(item["urdf_path"]).resolve()) or receipt.get("output") != str(output.resolve()):
            raise RuntimeError("worker receipt path mismatch")
        if not valid_png(output, args.resolution):
            raise RuntimeError("worker output is not a valid PNG")
        digest = sha256(output)
        common.update({"status": "rendered", "image_bytes": output.stat().st_size, "png_bytes": output.stat().st_size, "image_sha256": digest, "png_sha256": digest, "renderer_result": receipt})
    except Exception as exc:
        common.update({"status": "failed", "error": f"{type(exc).__name__}: {exc}", "image_bytes": 0, "png_bytes": 0, "image_sha256": "", "png_sha256": "", "renderer_result": None})
    return common


CSV_FIELDS = ["status", "category", "dataset_id", "ordinal", "source_path", "package_path", "urdf_path", "output_path", "image", "image_bytes", "png_bytes", "image_sha256", "png_sha256", "urdf_sha256", "urdf_bytes", "package_file_count", "package_total_bytes", "package_content_manifest_sha256", "category_one_shot", "error", "renderer_result"]


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(value, sort_keys=True, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in sorted(rows, key=lambda value: int(value["ordinal"])):
            output = dict(row); output["renderer_result"] = json.dumps(output.get("renderer_result"), sort_keys=True, ensure_ascii=True)
            writer.writerow(output)
    temporary.replace(path)


def run(args: argparse.Namespace) -> dict[str, Any]:
    metadata, selected = load_items(args.dataset, args.manifest, args.output_root)
    if args.categories:
        requested = set(args.categories)
        selected = [item for item in selected if item["category"] in requested]
        missing = sorted(requested - {item["category"] for item in selected})
        if missing:
            raise ValueError(f"unknown categories: {', '.join(missing)}")
    if args.limit is not None:
        if args.limit < 1:
            raise ValueError("--limit must be positive")
        selected = selected[: args.limit]
    if not selected:
        raise ValueError("selection contains no categories")
    for item in selected:
        item["dependency_receipt"] = dependency_receipt(metadata["source_root"], item["urdf_path"])
    args.output_root = args.output_root.expanduser().resolve()
    args.worker = args.worker.expanduser().resolve(strict=True); args.support_renderer = args.support_renderer.expanduser().resolve(strict=True); args.base_renderer = args.base_renderer.expanduser().resolve(strict=True); args.shared_renderer = args.shared_renderer.expanduser().resolve(strict=True); args.blender = args.blender.expanduser().resolve(strict=True)
    config = build_config(args.dataset, metadata, selected, args, args.worker, args.base_renderer, args.shared_renderer, args.blender)
    config_path = args.output_root / "render_config.json"
    if config_path.is_file():
        previous = json.loads(config_path.read_text(encoding="utf-8"))
        if previous != config:
            raise ValueError(f"output root contains a different render contract: {config_path}")
    if args.dry_run:
        return {"status": "dry_run", "dataset": metadata["dataset"], "selected": len(selected), "config": config}
    args.output_root.mkdir(parents=True, exist_ok=True)
    write_json(config_path, config)
    prior: dict[str, Mapping[str, str]] = {}
    manifest_path = args.output_root / "render_manifest.csv"
    if manifest_path.is_file():
        with manifest_path.open(encoding="utf-8", newline="") as stream:
            prior = {row.get("dataset_id", ""): row for row in csv.DictReader(stream)}
    results: list[dict[str, Any]] = []
    started_at = utc_now()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(render_one, item, args=args, config=config, prior=prior.get(item["asset_id"])): item for item in selected}
        for future in as_completed(futures):
            results.append(future.result())
    write_csv(manifest_path, results)
    roster = [
        {
            "status": "selected", "category": item["category"],
            "dataset_id": item["asset_id"], "ordinal": item["ordinal"],
            "source_path": str(item["source_path"]), "package_path": str(item["source_path"]),
            "urdf_path": str(item["urdf_path"]), "output_path": str(item["output_path"]),
            "image": str(item["output_path"]), "image_bytes": "", "png_bytes": "",
            "image_sha256": "", "png_sha256": "", "urdf_sha256": item["urdf_sha256"],
            "urdf_bytes": item["urdf_bytes"],
            "package_file_count": item["dependency_receipt"]["file_count"],
            "package_total_bytes": item["dependency_receipt"]["total_bytes"],
            "package_content_manifest_sha256": item["dependency_receipt"]["content_manifest_sha256"],
            "category_one_shot": True,
            "error": "", "renderer_result": None,
        }
        for item in selected
    ]
    write_csv(args.output_root / "category_one_shot_roster.csv", roster)
    summary = {"schema_version": 1, "dataset": metadata["dataset"], "selected_count": len(results), "rendered_count": sum(r.get("status") == "rendered" for r in results), "reused_valid_count": sum(r.get("status") == "reused_valid" for r in results), "failure_count": sum(r.get("status") == "failed" for r in results), "valid_png_count": sum(valid_png(Path(r["output_path"]), args.resolution) for r in results), "category_count": len({r["category"] for r in results}), "started_at": started_at, "finished_at": utc_now()}
    write_json(args.output_root / "render_summary.json", summary)
    return summary


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", choices=sorted(DEFAULT_MANIFESTS), required=True)
    p.add_argument("--manifest", type=Path)
    p.add_argument("--output-root", type=Path)
    p.add_argument("--worker", type=Path, default=WORKER)
    p.add_argument("--support-renderer", type=Path, default=SUPPORT_RENDERER)
    p.add_argument("--base-renderer", type=Path, default=BASE_RENDERER)
    p.add_argument("--shared-renderer", type=Path, default=SHARED_RENDERER)
    p.add_argument("--blender", type=Path, default=BLENDER_DEFAULT)
    p.add_argument("--resolution", type=int, default=256); p.add_argument("--samples", type=int, default=4)
    p.add_argument("--gpu", default="7"); p.add_argument("--workers", type=int, default=4); p.add_argument("--timeout-seconds", type=float, default=900.0)
    p.add_argument("--force", action="store_true"); p.add_argument("--dry-run", action="store_true")
    p.add_argument("--category", dest="categories", action="append")
    p.add_argument("--limit", type=int)
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    args.manifest = (args.manifest or DEFAULT_MANIFESTS[args.dataset]).expanduser().resolve()
    args.output_root = (args.output_root or DEFAULT_OUTPUTS[args.dataset]).expanduser().resolve()
    if args.workers < 1 or args.resolution < 64 or args.samples < 1 or args.timeout_seconds <= 0:
        raise SystemExit("invalid render parameters")
    try:
        print(json.dumps(run(args), sort_keys=True, ensure_ascii=True), flush=True)
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
