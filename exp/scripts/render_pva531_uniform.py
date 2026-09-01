#!/usr/bin/env python3
"""Render one seed from every PV-A generator with one frozen Blender studio setup."""

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
DEFAULT_INDEX_CSV = REPO_ROOT / "template_maps" / "generator_picture_index.csv"
DEFAULT_ASSET_ROOT = Path("/mnt/zsn/data/particulate/datasets/PV-A/extracted")
DEFAULT_OUTPUT_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/PV-A/renders/uniform531_studio_256_v1"
)
DEFAULT_RENDERER = REPO_ROOT / "arti-template" / "scripts" / "render_exported_asset_blender.py"
DEFAULT_LIBRARY_ROOT = REPO_ROOT / "arti-template" / "pbr_material_library"
DEFAULT_BLENDER = Path(
    "/mnt/zsn/zsn_workspace/VoxHammer/third_party/blender-4.2.19-linux-x64/blender"
)
EXPECTED_GENERATOR_COUNT = 531
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


@dataclass(frozen=True, slots=True)
class RenderItem:
    ordinal: int
    generator_index: str
    generator_name: str
    source_type: str
    picture_category: str
    asset_dir: Path
    output_path: Path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _inside(root: Path, child: Path) -> Path:
    root = root.expanduser().resolve(strict=True)
    child = child.expanduser().resolve(strict=True)
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
    index_csv: Path,
    *,
    asset_root: Path,
    output_root: Path,
    strict_count: bool = True,
) -> tuple[RenderItem, ...]:
    """Resolve the authoritative CSV roster to seed_0000 packages and output paths."""
    index_csv = index_csv.expanduser().resolve(strict=True)
    asset_root = asset_root.expanduser().resolve(strict=True)
    output_root = output_root.expanduser().resolve()
    with index_csv.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if strict_count and len(rows) != EXPECTED_GENERATOR_COUNT:
        raise ValueError(f"expected {EXPECTED_GENERATOR_COUNT} generators, found {len(rows)}")

    indices: set[str] = set()
    names: set[str] = set()
    items: list[RenderItem] = []
    for ordinal, row in enumerate(rows, start=1):
        generator_index = (row.get("generator_index") or "").strip()
        generator_name = (row.get("generator_name") or "").strip()
        source_type = (row.get("source_type") or "").strip()
        if not generator_index or not generator_name:
            raise ValueError(f"missing generator identity at CSV row {ordinal + 1}")
        if generator_index in indices or generator_name in names:
            raise ValueError(f"duplicate generator identity: {generator_index} / {generator_name}")
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", generator_index):
            raise ValueError(f"unsafe generator index: {generator_index!r}")
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", generator_name):
            raise ValueError(f"unsafe generator name: {generator_name!r}")
        if source_type not in {"picture_backed", "articraft_builtin_dataset_no_picture"}:
            raise ValueError(f"unsupported source type for {generator_index}: {source_type!r}")
        indices.add(generator_index)
        names.add(generator_name)

        asset_dir = _inside(asset_root, asset_root / generator_name / "seed_0000")
        for required in ("model.urdf", "appearance.json"):
            if not (asset_dir / required).is_file():
                raise FileNotFoundError(asset_dir / required)
        output_path = output_root / f"{generator_index}__{generator_name}__seed_0000.png"
        items.append(
            RenderItem(
                ordinal=ordinal,
                generator_index=generator_index,
                generator_name=generator_name,
                source_type=(
                    "builtin_no_picture"
                    if source_type == "articraft_builtin_dataset_no_picture"
                    else source_type
                ),
                picture_category=(row.get("picture_category") or "").strip(),
                asset_dir=asset_dir,
                output_path=output_path,
            )
        )
    return tuple(items)


def _blender_version(blender: Path) -> str:
    result = subprocess.run(
        [str(blender), "--version"],
        check=True,
        capture_output=True,
        text=True,
        timeout=60,
    )
    return result.stdout.splitlines()[0].strip()


def _asset_input_receipt(
    items: Sequence[RenderItem],
    *,
    library_root: Path,
) -> dict[str, Any]:
    """Hash every rendered URDF/appearance/mesh and the shared render helper."""
    digest = hashlib.sha256()
    file_count = 0
    total_bytes = 0

    def add_file(identity: str, path: Path, *, expected_hash: str | None = None) -> None:
        nonlocal file_count, total_bytes
        size = path.stat().st_size
        actual_hash = _sha256(path)
        if expected_hash is not None and actual_hash != expected_hash:
            raise ValueError(
                f"asset receipt SHA mismatch for {path}: {actual_hash} != {expected_hash}"
            )
        digest.update(identity.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(size).encode("ascii"))
        digest.update(b"\0")
        digest.update(actual_hash.encode("ascii"))
        digest.update(b"\n")
        file_count += 1
        total_bytes += size

    for item in items:
        prefix = f"{item.generator_index}/{item.generator_name}/{item.asset_dir.name}"
        urdf_path = item.asset_dir / "model.urdf"
        appearance_path = item.asset_dir / "appearance.json"
        add_file(f"{prefix}/model.urdf", urdf_path)
        add_file(f"{prefix}/appearance.json", appearance_path)
        appearance = json.loads(appearance_path.read_text(encoding="utf-8"))
        assets = appearance.get("assets")
        mesh_receipts = assets.get("meshes") if isinstance(assets, dict) else None
        if mesh_receipts is None:
            mesh_receipts = []
        if not isinstance(mesh_receipts, list):
            raise ValueError(f"appearance assets.meshes is not a list: {appearance_path}")
        seen_meshes: set[str] = set()
        for receipt in sorted(
            mesh_receipts,
            key=lambda value: str(value.get("path", "")) if isinstance(value, dict) else "",
        ):
            if not isinstance(receipt, dict):
                raise ValueError(f"invalid mesh receipt in {appearance_path}")
            relative_value = receipt.get("path")
            expected_hash = receipt.get("sha256")
            expected_size = receipt.get("size_bytes")
            if (
                not isinstance(relative_value, str)
                or not re.fullmatch(r"[0-9a-f]{64}", str(expected_hash))
                or not isinstance(expected_size, int)
                or expected_size < 0
            ):
                raise ValueError(f"invalid mesh path/SHA receipt in {appearance_path}")
            relative = Path(relative_value)
            if relative.is_absolute() or ".." in relative.parts or relative_value in seen_meshes:
                raise ValueError(f"unsafe or duplicate mesh receipt {relative_value!r}")
            seen_meshes.add(relative_value)
            mesh_path = _inside(item.asset_dir, item.asset_dir / relative)
            if not mesh_path.is_file() or mesh_path.stat().st_size != expected_size:
                raise ValueError(f"mesh size receipt mismatch: {mesh_path}")
            add_file(f"{prefix}/{relative.as_posix()}", mesh_path, expected_hash=str(expected_hash))

    pointer_path = library_root / "unified" / "latest.json"
    helper_path = REPO_ROOT / "arti-template" / "scripts" / "_pbr_material_preview_common.py"
    add_file("pbr_material_library/unified/latest.json", pointer_path)
    add_file("scripts/_pbr_material_preview_common.py", helper_path)
    pointer = json.loads(pointer_path.read_text(encoding="utf-8"))
    return {
        "asset_and_dependency_sha256": digest.hexdigest(),
        "file_count": file_count,
        "total_bytes": total_bytes,
        "pbr_logical_snapshot_sha256": pointer.get("logical_snapshot_sha256"),
        "pbr_pointer_sha256": _sha256(pointer_path),
        "render_helper_sha256": _sha256(helper_path),
        "policy": (
            "full model.urdf, appearance.json, and mesh contents; PBR textures are pinned "
            "by appearance channel SHA receipts and verified by the renderer"
        ),
    }


def build_run_config(
    *,
    args: argparse.Namespace,
    items: Sequence[RenderItem],
    renderer: Path,
    blender: Path,
    library_root: Path,
) -> dict[str, Any]:
    index_csv = args.index_csv.expanduser().resolve(strict=True)
    return {
        "schema_version": 2,
        "render_contract": "pva531_uniform_studio_v1",
        "generator_count": len(items),
        "seed_policy": "seed_0000_per_generator",
        "index_csv": str(index_csv),
        "index_csv_sha256": _sha256(index_csv),
        "asset_root": str(args.asset_root.expanduser().resolve(strict=True)),
        "output_root": str(args.output_root.expanduser().resolve()),
        "renderer": str(renderer),
        "renderer_sha256": _sha256(renderer),
        "blender": str(blender),
        "blender_version": _blender_version(blender),
        "library_root": str(library_root),
        "input_receipt": _asset_input_receipt(items, library_root=library_root),
        "resolution": args.resolution,
        "samples": args.samples,
        "studio": {
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
        },
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


def _receipt_allows_reuse(
    item: RenderItem,
    receipt: dict[str, str] | None,
    *,
    resolution: int,
) -> bool:
    """Reuse only an image covered by a prior successful content receipt."""
    if not receipt or receipt.get("status") not in {"rendered", "reused_valid"}:
        return False
    if receipt.get("generator_name") != item.generator_name:
        return False
    try:
        declared_output = Path(receipt.get("output_path") or "").expanduser().resolve()
        declared_bytes = int(receipt.get("png_bytes") or -1)
    except (OSError, ValueError):
        return False
    if declared_output != item.output_path.resolve() or not _valid_png(item.output_path, resolution):
        return False
    if item.output_path.stat().st_size != declared_bytes:
        return False
    return _sha256(item.output_path) == (receipt.get("png_sha256") or "")


def _render_one(
    item: RenderItem,
    *,
    args: argparse.Namespace,
    blender: Path,
    renderer: Path,
    library_root: Path,
    reuse_receipt: dict[str, str] | None,
) -> dict[str, Any]:
    started_at = _utc_now()
    start = time.monotonic()
    log_path = args.output_root / "logs" / f"{item.generator_index}.log"
    base = {
        "ordinal": item.ordinal,
        "generator_index": item.generator_index,
        "generator_name": item.generator_name,
        "source_type": item.source_type,
        "picture_category": item.picture_category,
        "asset_dir": str(item.asset_dir),
        "output_path": str(item.output_path),
        "started_at": started_at,
    }
    if not args.force and _receipt_allows_reuse(
        item, reuse_receipt, resolution=args.resolution
    ):
        return {
            **base,
            "status": "reused_valid",
            "elapsed_seconds": 0.0,
            "png_bytes": item.output_path.stat().st_size,
            "png_sha256": _sha256(item.output_path),
            "finished_at": _utc_now(),
            "renderer_result": None,
            "error": "",
        }

    temporary_output = item.output_path.with_name(
        f".{item.output_path.stem}.{os.getpid()}.{time.time_ns()}.tmp.png"
    )
    command = [
        str(blender),
        "-b",
        "--factory-startup",
        "-noaudio",
        "-P",
        str(renderer),
        "--",
        "--asset-dir",
        str(item.asset_dir),
        "--output",
        str(temporary_output),
        "--library-root",
        str(library_root),
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
            cwd=REPO_ROOT / "arti-template",
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

    result = {
        **base,
        "status": status,
        "elapsed_seconds": round(time.monotonic() - start, 3),
        "png_bytes": item.output_path.stat().st_size if status == "rendered" else 0,
        "png_sha256": _sha256(item.output_path) if status == "rendered" else "",
        "finished_at": _utc_now(),
        "renderer_result": renderer_result,
        "error": error,
    }
    return result


def _write_manifest(path: Path, results: Sequence[dict[str, Any]]) -> None:
    fields = [
        "ordinal",
        "generator_index",
        "generator_name",
        "source_type",
        "picture_category",
        "asset_dir",
        "output_path",
        "status",
        "elapsed_seconds",
        "png_bytes",
        "png_sha256",
        "started_at",
        "finished_at",
        "error",
        "renderer_result",
    ]
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in sorted(results, key=lambda row: int(row["ordinal"])):
            row = dict(result)
            row["renderer_result"] = json.dumps(
                row.get("renderer_result"), sort_keys=True, ensure_ascii=True
            )
            writer.writerow(row)
    temporary.replace(path)


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.resolution < 64 or args.samples < 1 or args.workers < 1:
        raise ValueError("resolution >= 64, samples >= 1, and workers >= 1 are required")
    if args.timeout_seconds <= 0:
        raise ValueError("timeout must be positive")
    args.output_root = args.output_root.expanduser().resolve()
    renderer = args.renderer.expanduser().resolve(strict=True)
    blender = args.blender.expanduser().resolve(strict=True)
    library_root = args.library_root.expanduser().resolve(strict=True)
    items = load_render_items(
        args.index_csv,
        asset_root=args.asset_root,
        output_root=args.output_root,
        strict_count=not args.allow_count_drift,
    )
    if args.names:
        requested = set(args.names)
        items = tuple(item for item in items if item.generator_name in requested)
        missing = sorted(requested - {item.generator_name for item in items})
        if missing:
            raise ValueError(f"unknown --names: {', '.join(missing)}")
    if args.limit is not None:
        if args.limit < 1:
            raise ValueError("--limit must be positive")
        items = items[: args.limit]
    if not items:
        raise ValueError("selection contains no render items")

    args.output_root.mkdir(parents=True, exist_ok=True)
    (args.output_root / "logs").mkdir(exist_ok=True)
    config = build_run_config(
        args=args,
        items=items,
        renderer=renderer,
        blender=blender,
        library_root=library_root,
    )
    config_path = args.output_root / "render_config.json"
    if config_path.is_file():
        previous = json.loads(config_path.read_text(encoding="utf-8"))
        if _stable_config(previous) != _stable_config(config):
            raise ValueError(
                f"output root contains a different render contract: {config_path}; use a new root"
            )
    else:
        _write_json(config_path, config)

    if args.dry_run:
        return {"config": config, "status": "dry_run", "selected": len(items)}

    print(
        f"[render] {len(items)} generators -> {args.output_root} "
        f"({args.resolution}px, {args.samples} samples, GPU {args.gpu}, workers={args.workers})",
        flush=True,
    )
    results: list[dict[str, Any]] = []
    state_path = args.output_root / "render_state.jsonl"
    manifest_path = args.output_root / "render_manifest.csv"
    reuse_receipts: dict[str, dict[str, str]] = {}
    if manifest_path.is_file():
        with manifest_path.open("r", encoding="utf-8", newline="") as stream:
            previous_rows = list(csv.DictReader(stream))
        previous_indices = [row.get("generator_index", "") for row in previous_rows]
        if len(previous_indices) != len(set(previous_indices)):
            raise ValueError(f"duplicate generator indices in prior manifest: {manifest_path}")
        reuse_receipts = {row["generator_index"]: row for row in previous_rows}
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
                    library_root=library_root,
                    reuse_receipt=reuse_receipts.get(item.generator_index),
                ): item
                for item in items
            }
            for completed, future in enumerate(as_completed(futures), start=1):
                result = future.result()
                results.append(result)
                state_stream.write(json.dumps(result, sort_keys=True, ensure_ascii=True) + "\n")
                state_stream.flush()
                _write_manifest(manifest_path, results)
                print(
                    f"[render] {completed}/{len(items)} {result['generator_index']} "
                    f"{result['status']} ({result['elapsed_seconds']:.1f}s)",
                    flush=True,
                )

    success_statuses = {"rendered", "reused_valid"}
    failures = [result for result in results if result["status"] not in success_statuses]
    summary = {
        "schema_version": 2,
        "started_at": started_at,
        "finished_at": _utc_now(),
        "selected_count": len(items),
        "rendered_count": sum(result["status"] == "rendered" for result in results),
        "reused_valid_count": sum(result["status"] == "reused_valid" for result in results),
        "failure_count": len(failures),
        "valid_png_count": sum(_valid_png(item.output_path, args.resolution) for item in items),
        "manifest": str(manifest_path),
        "config": str(config_path),
        "failure_indices": [result["generator_index"] for result in failures],
    }
    _write_json(args.output_root / "render_summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    if failures:
        raise RuntimeError(f"{len(failures)} render(s) failed; rerun without --force to resume")
    return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index-csv", type=Path, default=DEFAULT_INDEX_CSV)
    parser.add_argument("--asset-root", type=Path, default=DEFAULT_ASSET_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--renderer", type=Path, default=DEFAULT_RENDERER)
    parser.add_argument("--library-root", type=Path, default=DEFAULT_LIBRARY_ROOT)
    parser.add_argument("--blender", type=Path, default=DEFAULT_BLENDER)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--gpu", default="7")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--timeout-seconds", type=float, default=900.0)
    parser.add_argument("--names", nargs="+", help="Render only the named generators.")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--force", action="store_true", help="Rerender valid outputs in this root.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--allow-count-drift", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        run(build_argument_parser().parse_args(argv))
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
