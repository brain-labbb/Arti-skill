#!/usr/bin/env python3
"""Render all 3,544 Artiverse models with the frozen PV-A Blender studio."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

from PIL import Image


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_DATASET_MANIFEST = REPO_ROOT / "exp" / "artiverse" / "dataset_chunks" / "manifest.json"
DEFAULT_DATA_ROOT = REPO_ROOT / "exp" / "artiverse" / "data"
DEFAULT_OUTPUT_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/Artiverse/renders/uniform3544_studio_256_v1"
)
DEFAULT_RENDERER = REPO_ROOT / "exp" / "scripts" / "render_artiverse_asset_blender.py"
DEFAULT_SHARED_RENDERER = (
    REPO_ROOT / "arti-template" / "scripts" / "render_exported_asset_blender.py"
)
DEFAULT_BLENDER = Path(
    "/mnt/zsn/zsn_workspace/VoxHammer/third_party/blender-4.2.19-linux-x64/blender"
)
EXPECTED_FORMAT = "artiverse-data-tar-gz-chunks-v1"
EXPECTED_MODEL_COUNT = 3544
EXPECTED_CATEGORY_COUNT = 84
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
SUCCESS_STATUSES = {"rendered", "reused_valid"}


@dataclass(frozen=True, slots=True)
class RenderItem:
    ordinal: int
    category: str
    source: str
    model_id: str
    manifest_root: str
    identity_sha256: str
    category_one_shot: bool
    glb_path: Path
    output_path: Path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _identity_sha256(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _inside(root: Path, child: Path, *, must_exist: bool) -> Path:
    root = root.expanduser().resolve(strict=must_exist)
    child = child.expanduser().resolve(strict=must_exist)
    try:
        child.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"path escapes declared root {root}: {child}") from exc
    return child


def _valid_png(path: Path, resolution: int) -> bool:
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


def load_render_items(
    dataset_manifest: Path,
    *,
    data_root: Path,
    output_root: Path,
    strict_counts: bool = True,
) -> tuple[RenderItem, ...]:
    """Resolve the official chunk-manifest order to segmented GLBs."""

    dataset_manifest = dataset_manifest.expanduser().resolve(strict=True)
    data_root = data_root.expanduser().resolve(strict=True)
    output_root = output_root.expanduser().resolve()
    manifest = json.loads(dataset_manifest.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or manifest.get("format") != EXPECTED_FORMAT:
        raise ValueError(f"unsupported Artiverse dataset manifest: {dataset_manifest}")
    chunks = manifest.get("chunks")
    if not isinstance(chunks, list) or not chunks:
        raise ValueError("dataset manifest chunks must be a non-empty list")

    roots: list[str] = []
    for chunk_index, chunk in enumerate(chunks):
        if not isinstance(chunk, dict) or not isinstance(chunk.get("roots"), list):
            raise ValueError(f"invalid chunk entry {chunk_index}")
        chunk_roots = chunk["roots"]
        if int(chunk.get("model_count", -1)) != len(chunk_roots):
            raise ValueError(f"chunk {chunk_index} model_count does not match roots")
        roots.extend(str(value) for value in chunk_roots)
    if int(manifest.get("model_count", -1)) != len(roots):
        raise ValueError("top-level model_count does not match flattened chunk roots")
    if len(roots) != len(set(roots)):
        raise ValueError("dataset manifest contains duplicate model roots")
    if strict_counts and len(roots) != EXPECTED_MODEL_COUNT:
        raise ValueError(f"expected {EXPECTED_MODEL_COUNT} models, found {len(roots)}")

    parsed: list[tuple[str, str, str, str, Path, Path]] = []
    categories: set[str] = set()
    safe_component = re.compile(r"[A-Za-z0-9_.-]+")
    for manifest_root in roots:
        parts = Path(manifest_root).parts
        if len(parts) != 4 or parts[0] != "data" or ".." in parts:
            raise ValueError(f"unsafe model root in dataset manifest: {manifest_root!r}")
        _, category, source, model_id = parts
        if any(safe_component.fullmatch(value) is None for value in (category, source, model_id)):
            raise ValueError(f"unsafe Artiverse identity: {manifest_root!r}")
        categories.add(category)
        model_root = _inside(data_root, data_root / category / source / model_id, must_exist=True)
        glb_path = _inside(
            data_root,
            model_root / f"{model_id}.segmented.glb",
            must_exist=True,
        )
        if not glb_path.is_file():
            raise FileNotFoundError(glb_path)
        output_path = output_root / category / source / model_id / "imgs" / "000.png"
        try:
            output_path.resolve().relative_to(output_root)
        except ValueError as exc:
            raise ValueError(f"output path escapes output root: {output_path}") from exc
        parsed.append((manifest_root, category, source, model_id, glb_path, output_path))
    if strict_counts and len(categories) != EXPECTED_CATEGORY_COUNT:
        raise ValueError(f"expected {EXPECTED_CATEGORY_COUNT} categories, found {len(categories)}")

    winners: dict[str, str] = {}
    for manifest_root, category, *_ in parsed:
        previous = winners.get(category)
        if previous is None or (_identity_sha256(manifest_root), manifest_root) < (
            _identity_sha256(previous),
            previous,
        ):
            winners[category] = manifest_root

    return tuple(
        RenderItem(
            ordinal=ordinal,
            category=category,
            source=source,
            model_id=model_id,
            manifest_root=manifest_root,
            identity_sha256=_identity_sha256(manifest_root),
            category_one_shot=winners[category] == manifest_root,
            glb_path=glb_path,
            output_path=output_path,
        )
        for ordinal, (manifest_root, category, source, model_id, glb_path, output_path) in enumerate(
            parsed, start=1
        )
    )


def _blender_version(blender: Path) -> str:
    result = subprocess.run(
        [str(blender), "--version"],
        check=True,
        capture_output=True,
        text=True,
        timeout=60,
    )
    return result.stdout.splitlines()[0].strip()


def _input_receipt(
    items: Sequence[RenderItem],
) -> tuple[dict[str, Any], dict[str, tuple[int, str]]]:
    """Hash every segmented GLB and return aggregate and per-item receipts."""

    digest = hashlib.sha256()
    total_bytes = 0
    receipts: dict[str, tuple[int, str]] = {}
    for item in items:
        size = item.glb_path.stat().st_size
        content_sha256 = _sha256(item.glb_path)
        digest.update(item.manifest_root.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(size).encode("ascii"))
        digest.update(b"\0")
        digest.update(content_sha256.encode("ascii"))
        digest.update(b"\n")
        total_bytes += size
        receipts[item.manifest_root] = (size, content_sha256)
    return (
        {
            "segmented_glb_count": len(items),
            "segmented_glb_total_bytes": total_bytes,
            "segmented_glb_receipt_sha256": digest.hexdigest(),
            "policy": "manifest_root, byte size, and full segmented GLB SHA256 in official order",
        },
        receipts,
    )


def _studio_contract() -> dict[str, Any]:
    return {
        "mode": "opaque_studio",
        "cycles_denoising": True,
        "view_transform": "AgX",
        "look": "AgX - Medium High Contrast",
        "world_rgba": [0.80, 0.84, 0.90, 1.0],
        "world_strength": 0.55,
        "ground_rgba": [0.32, 0.35, 0.40, 1.0],
        "ground_roughness": 0.82,
        "camera_vertical_fov_degrees": 42.0,
        "camera_direction": [1.25, -1.35, 0.85],
        "camera_distance_policy": "bounding_sphere_auto_frame_1.18",
        "lights": [
            {"direction": [0.4, -0.8, 1.5], "gain": 42.0, "size_ratio": 1.5},
            {"direction": [-1.2, -0.3, 0.6], "gain": 15.0, "size_ratio": 1.8},
            {"direction": [0.2, 1.0, 1.2], "gain": 24.0, "size_ratio": 1.2},
        ],
    }


def build_run_config(
    *,
    args: argparse.Namespace,
    items: Sequence[RenderItem],
    renderer: Path,
    shared_renderer: Path,
    blender: Path,
    input_receipt: dict[str, Any],
) -> dict[str, Any]:
    dataset_manifest = args.dataset_manifest.expanduser().resolve(strict=True)
    return {
        "schema_version": 1,
        "render_contract": "artiverse_uniform_studio_v1",
        "model_count": len(items),
        "category_count": len({item.category for item in items}),
        "dataset_manifest": str(dataset_manifest),
        "dataset_manifest_sha256": _sha256(dataset_manifest),
        "data_root": str(args.data_root.expanduser().resolve(strict=True)),
        "output_root": str(args.output_root.expanduser().resolve()),
        "driver": str(SCRIPT),
        "driver_sha256": _sha256(SCRIPT),
        "renderer": str(renderer),
        "renderer_sha256": _sha256(renderer),
        "shared_renderer": str(shared_renderer),
        "shared_renderer_sha256": _sha256(shared_renderer),
        "blender": str(blender),
        "blender_version": _blender_version(blender),
        "input_receipt": input_receipt,
        "resolution": args.resolution,
        "samples": args.samples,
        "pose_policy": "canonical transforms embedded in segmented.glb",
        "material_policy": "native glTF materials and textures; imported cameras and lights removed",
        "image_layout": "category/source/model_id/imgs/000.png",
        "one_shot": {
            "count": sum(item.category_one_shot for item in items),
            "rule": "minimum (SHA256(manifest_root UTF-8), manifest_root) within category",
            "roster": "category_one_shot_roster.csv",
        },
        "studio": _studio_contract(),
        "gpu_visibility": str(args.gpu),
        "workers": args.workers,
        "timeout_seconds": args.timeout_seconds,
    }


def _stable_config(config: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in config.items() if key not in {"workers", "timeout_seconds"}}


def _write_json(path: Path, payload: Any) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _write_one_shot_roster(
    path: Path,
    items: Sequence[RenderItem],
    receipts: dict[str, tuple[int, str]],
) -> None:
    fields = [
        "category",
        "source",
        "model_id",
        "manifest_root",
        "identity_sha256",
        "glb_path",
        "glb_bytes",
        "glb_sha256",
        "output_path",
    ]
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for item in sorted((value for value in items if value.category_one_shot), key=lambda x: x.category):
            glb_bytes, glb_sha256 = receipts[item.manifest_root]
            writer.writerow(
                {
                    "category": item.category,
                    "source": item.source,
                    "model_id": item.model_id,
                    "manifest_root": item.manifest_root,
                    "identity_sha256": item.identity_sha256,
                    "glb_path": str(item.glb_path),
                    "glb_bytes": glb_bytes,
                    "glb_sha256": glb_sha256,
                    "output_path": str(item.output_path),
                }
            )
    temporary.replace(path)


def _receipt_allows_reuse(
    item: RenderItem,
    receipt: dict[str, Any] | None,
    *,
    resolution: int,
    glb_bytes: int,
    glb_sha256: str,
) -> bool:
    if not receipt or receipt.get("status") not in SUCCESS_STATUSES:
        return False
    if any(
        str(receipt.get(field) or "") != expected
        for field, expected in (
            ("category", item.category),
            ("source", item.source),
            ("model_id", item.model_id),
            ("manifest_root", item.manifest_root),
            ("identity_sha256", item.identity_sha256),
            ("glb_sha256", glb_sha256),
        )
    ):
        return False
    try:
        declared_output = Path(str(receipt.get("output_path") or "")).expanduser().resolve()
        declared_png_bytes = int(receipt.get("png_bytes") or -1)
        declared_glb_bytes = int(receipt.get("glb_bytes") or -1)
    except (OSError, ValueError):
        return False
    if declared_glb_bytes != glb_bytes or declared_output != item.output_path.resolve():
        return False
    if not _valid_png(item.output_path, resolution):
        return False
    if item.output_path.stat().st_size != declared_png_bytes:
        return False
    return _sha256(item.output_path) == str(receipt.get("png_sha256") or "")


def _render_one(
    item: RenderItem,
    *,
    args: argparse.Namespace,
    blender: Path,
    renderer: Path,
    input_receipt: tuple[int, str],
    reuse_receipt: dict[str, Any] | None,
) -> dict[str, Any]:
    started_at = _utc_now()
    start = time.monotonic()
    glb_bytes, glb_sha256 = input_receipt
    base = {
        "ordinal": item.ordinal,
        "category": item.category,
        "source": item.source,
        "model_id": item.model_id,
        "manifest_root": item.manifest_root,
        "identity_sha256": item.identity_sha256,
        "category_one_shot": item.category_one_shot,
        "glb_path": str(item.glb_path),
        "glb_bytes": glb_bytes,
        "glb_sha256": glb_sha256,
        "output_path": str(item.output_path),
        "started_at": started_at,
    }
    if not args.force and _receipt_allows_reuse(
        item,
        reuse_receipt,
        resolution=args.resolution,
        glb_bytes=glb_bytes,
        glb_sha256=glb_sha256,
    ):
        return {
            **base,
            "status": "reused_valid",
            "elapsed_seconds": 0.0,
            "png_bytes": item.output_path.stat().st_size,
            "png_sha256": _sha256(item.output_path),
            "imported_cameras_removed": int(
                (reuse_receipt or {}).get("imported_cameras_removed") or 0
            ),
            "imported_lights_removed": int(
                (reuse_receipt or {}).get("imported_lights_removed") or 0
            ),
            "finished_at": _utc_now(),
            "error": "",
            "renderer_result": None,
        }

    item.output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_output = item.output_path.with_name(
        f".{item.output_path.stem}.{os.getpid()}.{time.time_ns()}.tmp.png"
    )
    log_path = args.output_root / "logs" / f"{item.ordinal:04d}.log"
    command = [
        str(blender),
        "-b",
        "--factory-startup",
        "-noaudio",
        "-P",
        str(renderer),
        "--",
        "--glb",
        str(item.glb_path),
        "--output",
        str(temporary_output),
        "--resolution",
        str(args.resolution),
        "--samples",
        str(args.samples),
    ]
    environment = os.environ.copy()
    environment["CUDA_VISIBLE_DEVICES"] = str(args.gpu)
    status = "failed"
    error = ""
    renderer_result: dict[str, Any] | None = None
    output = ""
    try:
        process = subprocess.run(
            command,
            cwd=REPO_ROOT,
            env=environment,
            capture_output=True,
            text=True,
            timeout=args.timeout_seconds,
            check=False,
        )
        output = process.stdout + ("\n[stderr]\n" + process.stderr if process.stderr else "")
        if process.returncode != 0:
            error = f"Blender exited with code {process.returncode}"
        elif not _valid_png(temporary_output, args.resolution):
            error = "Blender exited successfully but output PNG is missing or invalid"
        else:
            temporary_output.replace(item.output_path)
            status = "rendered"
            for line in reversed(process.stdout.splitlines()):
                try:
                    candidate = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(candidate, dict) and candidate.get("output"):
                    renderer_result = candidate
                    renderer_result["output"] = str(item.output_path)
                    break
    except subprocess.TimeoutExpired as exc:
        error = f"timeout after {args.timeout_seconds:g} seconds"
        stdout = exc.stdout.decode(errors="replace") if isinstance(exc.stdout, bytes) else exc.stdout
        stderr = exc.stderr.decode(errors="replace") if isinstance(exc.stderr, bytes) else exc.stderr
        output = (stdout or "") + ("\n[stderr]\n" + stderr if stderr else "")
    except OSError as exc:
        error = f"subprocess error: {exc}"
        output = error
    finally:
        if temporary_output.exists():
            temporary_output.unlink()
    log_path.write_text(output, encoding="utf-8", errors="replace")

    return {
        **base,
        "status": status,
        "elapsed_seconds": round(time.monotonic() - start, 3),
        "png_bytes": item.output_path.stat().st_size if status == "rendered" else 0,
        "png_sha256": _sha256(item.output_path) if status == "rendered" else "",
        "imported_cameras_removed": int(
            (renderer_result or {}).get("imported_cameras_removed") or 0
        ),
        "imported_lights_removed": int(
            (renderer_result or {}).get("imported_lights_removed") or 0
        ),
        "finished_at": _utc_now(),
        "error": error,
        "renderer_result": renderer_result,
    }


MANIFEST_FIELDS = [
    "ordinal",
    "category",
    "source",
    "model_id",
    "manifest_root",
    "identity_sha256",
    "category_one_shot",
    "glb_path",
    "glb_bytes",
    "glb_sha256",
    "output_path",
    "status",
    "elapsed_seconds",
    "png_bytes",
    "png_sha256",
    "imported_cameras_removed",
    "imported_lights_removed",
    "started_at",
    "finished_at",
    "error",
    "renderer_result",
]


def _write_manifest(path: Path, results: Sequence[dict[str, Any]]) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()
        for result in sorted(results, key=lambda row: int(row["ordinal"])):
            row = {field: result.get(field, "") for field in MANIFEST_FIELDS}
            if not isinstance(row["renderer_result"], str):
                row["renderer_result"] = json.dumps(
                    row["renderer_result"], sort_keys=True, ensure_ascii=True
                )
            writer.writerow(row)
    temporary.replace(path)


def _read_recovery_receipts(
    *,
    manifest_path: Path,
    state_path: Path,
    roster: dict[str, RenderItem],
) -> dict[str, dict[str, Any]]:
    receipts: dict[str, dict[str, Any]] = {}

    def accept(row: dict[str, Any], *, source: Path) -> None:
        identity = str(row.get("manifest_root") or "")
        item = roster.get(identity)
        if item is None:
            raise ValueError(f"prior receipt contains an unknown model root in {source}: {identity!r}")
        try:
            ordinal = int(row.get("ordinal", -1))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid prior receipt ordinal in {source}") from exc
        if ordinal != item.ordinal:
            raise ValueError(f"prior receipt ordinal drift for {identity} in {source}")
        receipts[identity] = row

    if manifest_path.is_file():
        with manifest_path.open("r", encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        identities = [str(row.get("manifest_root") or "") for row in rows]
        if len(identities) != len(set(identities)):
            raise ValueError(f"duplicate model roots in prior manifest: {manifest_path}")
        for row in rows:
            accept(dict(row), source=manifest_path)
    if state_path.is_file():
        state_text = state_path.read_text(encoding="utf-8")
        state_lines = state_text.splitlines()
        for line_number, line in enumerate(state_lines, start=1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                if line_number == len(state_lines) and not state_text.endswith("\n"):
                    break
                raise ValueError(f"invalid recovery JSON at {state_path}:{line_number}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"non-object recovery row at {state_path}:{line_number}")
            accept(row, source=state_path)
    return receipts


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.resolution < 64 or args.samples < 1 or args.workers < 1:
        raise ValueError("resolution >= 64, samples >= 1, and workers >= 1 are required")
    if args.timeout_seconds <= 0 or args.checkpoint_every < 1:
        raise ValueError("timeout and checkpoint interval must be positive")
    args.output_root = args.output_root.expanduser().resolve()
    renderer = args.renderer.expanduser().resolve(strict=True)
    shared_renderer = args.shared_renderer.expanduser().resolve(strict=True)
    blender = args.blender.expanduser().resolve(strict=True)
    all_items = load_render_items(
        args.dataset_manifest,
        data_root=args.data_root,
        output_root=args.output_root,
        strict_counts=not args.allow_count_drift,
    )
    input_receipt, per_item_receipts = _input_receipt(all_items)

    selected = all_items
    if args.one_shot_only:
        selected = tuple(item for item in selected if item.category_one_shot)
    if args.categories:
        requested_categories = set(args.categories)
        known_categories = {item.category for item in all_items}
        missing = sorted(requested_categories - known_categories)
        if missing:
            raise ValueError(f"unknown --categories: {', '.join(missing)}")
        selected = tuple(item for item in selected if item.category in requested_categories)
    if args.limit is not None:
        if args.limit < 1:
            raise ValueError("--limit must be positive")
        selected = selected[: args.limit]
    if not selected:
        raise ValueError("selection contains no Artiverse models")

    config = build_run_config(
        args=args,
        items=all_items,
        renderer=renderer,
        shared_renderer=shared_renderer,
        blender=blender,
        input_receipt=input_receipt,
    )
    config_path = args.output_root / "render_config.json"
    if config_path.is_file():
        previous = json.loads(config_path.read_text(encoding="utf-8"))
        if _stable_config(previous) != _stable_config(config):
            raise ValueError(
                f"output root contains a different render contract: {config_path}; use a new root"
            )
    else:
        if args.output_root.is_dir() and any(args.output_root.iterdir()):
            raise ValueError(
                f"non-empty output root has no render_config.json: {args.output_root}"
            )
        args.output_root.mkdir(parents=True, exist_ok=True)
        _write_json(config_path, config)
    (args.output_root / "logs").mkdir(exist_ok=True)
    _write_one_shot_roster(
        args.output_root / "category_one_shot_roster.csv",
        all_items,
        per_item_receipts,
    )

    selection_summary = {
        "full_model_count": len(all_items),
        "full_category_count": len({item.category for item in all_items}),
        "selected_count": len(selected),
        "selected_category_count": len({item.category for item in selected}),
        "one_shot_only": bool(args.one_shot_only),
        "categories": sorted(args.categories or []),
        "limit": args.limit,
    }
    if args.dry_run:
        return {"config": config, "selection": selection_summary, "status": "dry_run"}

    print(
        f"[render] {len(selected)}/{len(all_items)} Artiverse models -> {args.output_root} "
        f"({args.resolution}px, {args.samples} samples, GPU {args.gpu}, workers={args.workers})",
        flush=True,
    )
    state_path = args.output_root / "render_state.jsonl"
    manifest_path = args.output_root / "render_manifest.csv"
    roster = {item.manifest_root: item for item in all_items}
    prior = _read_recovery_receipts(
        manifest_path=manifest_path,
        state_path=state_path,
        roster=roster,
    )
    latest = dict(prior)
    results: list[dict[str, Any]] = []
    started_at = _utc_now()
    with state_path.open("a", encoding="utf-8") as state_stream:
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {
                pool.submit(
                    _render_one,
                    item,
                    args=args,
                    blender=blender,
                    renderer=renderer,
                    input_receipt=per_item_receipts[item.manifest_root],
                    reuse_receipt=prior.get(item.manifest_root),
                ): item
                for item in selected
            }
            for completed, future in enumerate(as_completed(futures), start=1):
                result = future.result()
                results.append(result)
                latest[str(result["manifest_root"])] = result
                state_stream.write(json.dumps(result, sort_keys=True, ensure_ascii=True) + "\n")
                state_stream.flush()
                if completed % args.checkpoint_every == 0:
                    _write_manifest(manifest_path, list(latest.values()))
                print(
                    f"[render] {completed}/{len(selected)} {result['category']}/"
                    f"{result['model_id']} {result['status']} "
                    f"({result['elapsed_seconds']:.1f}s)",
                    flush=True,
                )
    _write_manifest(manifest_path, list(latest.values()))

    failures = [result for result in results if result["status"] not in SUCCESS_STATUSES]
    full_valid_count = sum(_valid_png(item.output_path, args.resolution) for item in all_items)
    summary = {
        "schema_version": 1,
        "render_contract": "artiverse_uniform_studio_v1",
        "started_at": started_at,
        "finished_at": _utc_now(),
        **selection_summary,
        "rendered_count": sum(result["status"] == "rendered" for result in results),
        "reused_valid_count": sum(result["status"] == "reused_valid" for result in results),
        "failure_count": len(failures),
        "selected_valid_png_count": sum(
            _valid_png(item.output_path, args.resolution) for item in selected
        ),
        "full_valid_png_count": full_valid_count,
        "full_complete": full_valid_count == len(all_items),
        "failure_roots": [str(result["manifest_root"]) for result in failures],
        "manifest": str(manifest_path),
        "config": str(config_path),
        "one_shot_roster": str(args.output_root / "category_one_shot_roster.csv"),
    }
    _write_json(args.output_root / "render_summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    if failures:
        raise RuntimeError(f"{len(failures)} render(s) failed; rerun without --force to resume")
    return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-manifest", type=Path, default=DEFAULT_DATASET_MANIFEST)
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--renderer", type=Path, default=DEFAULT_RENDERER)
    parser.add_argument("--shared-renderer", type=Path, default=DEFAULT_SHARED_RENDERER)
    parser.add_argument("--blender", type=Path, default=DEFAULT_BLENDER)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--gpu", default="7")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--timeout-seconds", type=float, default=900.0)
    parser.add_argument("--checkpoint-every", type=int, default=10)
    parser.add_argument("--one-shot-only", action="store_true")
    parser.add_argument("--categories", nargs="+")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--allow-count-drift", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        result = run(build_argument_parser().parse_args(argv))
        if result.get("status") == "dry_run":
            print(json.dumps(result, indent=2, sort_keys=True), flush=True)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
