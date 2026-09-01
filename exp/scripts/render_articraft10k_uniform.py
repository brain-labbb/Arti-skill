#!/usr/bin/env python3
"""Render one deterministic Articraft-10K representative per category.

The input is the frozen merged 10,787-record roster produced by the
Articraft-10K release audit.  The complete roster is used for category
support and selection; only one record per category is rendered.  A
content-independent identity hash chooses the winner, so changing geometry
or rendering cannot influence which record enters the visual comparison.

The output images use the same 256px Cycles studio contract as the PV-A and
Artiverse uniform renders.  The default is deliberately one-shot mode; use
``--all-models`` only for an explicit full-data render.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PureWindowsPath
from typing import Any, Iterable, Mapping, Sequence

from PIL import Image


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_DATASET_MANIFEST = (
    REPO_ROOT
    / "exp/runtime/articraft_github_merged_10787_20260829/rosters/merged/full_release_manifest.json"
)
DEFAULT_OUTPUT_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/Articraft-10K/renders/"
    "uniform244_one_per_category_studio_256_v1"
)
DEFAULT_RENDERER = REPO_ROOT / "exp/scripts/render_articraft10k_asset_blender.py"
DEFAULT_SHARED_RENDERER = REPO_ROOT / "arti-template/scripts/render_exported_asset_blender.py"
DEFAULT_BLENDER = Path(
    "/mnt/zsn/zsn_workspace/VoxHammer/third_party/blender-4.2.19-linux-x64/blender"
)
EXPECTED_DATASET = "Articraft-10K-merged-reconstructed-10787"
EXPECTED_MODEL_COUNT = 10_787
EXPECTED_CATEGORY_COUNT = 244
EXPECTED_SCHEMA = "table123_full_release_manifest_v1"
SUCCESS_STATUSES = {"rendered", "reused_valid"}
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True, slots=True)
class RenderItem:
    ordinal: int
    category: str
    asset_id: str
    cohort_origin: str
    source_path: Path
    urdf_path: Path
    urdf_bytes: int
    urdf_sha256: str
    package_binding_sha256: str
    identity_sha256: str
    category_one_shot: bool
    output_path: Path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _identity_sha256(asset_id: str) -> str:
    return hashlib.sha256(asset_id.encode("utf-8")).hexdigest()


def _selection_receipt(items: Sequence[RenderItem]) -> dict[str, Any]:
    rows = sorted(
        (
            {
                "category": item.category,
                "asset_id": item.asset_id,
                "identity_sha256": item.identity_sha256,
            }
            for item in items
        ),
        key=lambda row: (row["category"], row["asset_id"]),
    )
    return {
        "schema_version": 1,
        "ordering": "lexicographic (category, asset_id)",
        "count": len(rows),
        "identity_category_sha256": _canonical_sha256(rows),
    }


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


def _safe_relative(value: object, *, field: str) -> Path:
    text = str(value or "").strip()
    relative = Path(text)
    windows = PureWindowsPath(text)
    if (
        not text
        or "\\" in text
        or relative.is_absolute()
        or windows.is_absolute()
        or bool(windows.drive)
        or any(part in {"", ".", ".."} for part in relative.parts)
    ):
        raise ValueError(f"{field} must be a contained relative path")
    return relative


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


def _load_manifest(
    path: Path,
    *,
    strict_counts: bool = True,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    path = path.expanduser().resolve(strict=True)
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read Articraft roster manifest: {path}") from exc
    if not isinstance(manifest, dict):
        raise ValueError("Articraft roster manifest must be a JSON object")
    if manifest.get("schema_version") != EXPECTED_SCHEMA:
        raise ValueError(f"unsupported Articraft roster schema: {manifest.get('schema_version')!r}")
    if manifest.get("dataset") != EXPECTED_DATASET:
        raise ValueError(f"expected merged Articraft roster, found {manifest.get('dataset')!r}")
    self_hash = manifest.get("manifest_content_sha256")
    without_self = dict(manifest)
    without_self.pop("manifest_content_sha256", None)
    if self_hash != _canonical_sha256(without_self):
        raise ValueError("Articraft roster manifest self-hash mismatch")
    rows = manifest.get("rows")
    if not isinstance(rows, list):
        raise ValueError("Articraft roster rows must be a list")
    if manifest.get("N_eval") != len(rows):
        raise ValueError("Articraft roster N_eval does not match its row count")
    if strict_counts and len(rows) != EXPECTED_MODEL_COUNT:
        raise ValueError(f"expected exactly {EXPECTED_MODEL_COUNT} Articraft rows, found {len(rows)}")
    if manifest.get("roster_sha256") != _canonical_sha256(rows):
        raise ValueError("Articraft roster content hash mismatch")
    return manifest, [dict(row) if isinstance(row, dict) else {} for row in rows]


def _validate_package_files(row: Mapping[str, Any], source: Path) -> str:
    """Validate a package binding and return its canonical aggregate hash.

    This is done only for selected winners, keeping dry-run and category
    selection independent of the 10k package payloads while still producing
    a complete receipt for every rendered input.
    """

    raw_files = row.get("package_files")
    if not isinstance(raw_files, list):
        binding = row.get("package_binding")
        raw_files = binding.get("files") if isinstance(binding, Mapping) else None
    if not isinstance(raw_files, list):
        raise ValueError(f"package file binding is missing: {row.get('asset_id')}")
    normalized: list[dict[str, Any]] = []
    for entry in raw_files:
        if not isinstance(entry, Mapping):
            raise ValueError(f"package file binding entry is invalid: {row.get('asset_id')}")
        relative = _safe_relative(entry.get("path"), field="package file path")
        declared_size = entry.get("size", entry.get("bytes"))
        declared_hash = entry.get("sha256")
        if not isinstance(declared_size, int) or declared_size < 0:
            raise ValueError(f"package file size is invalid: {row.get('asset_id')}")
        if not isinstance(declared_hash, str) or SHA256_RE.fullmatch(declared_hash) is None:
            raise ValueError(f"package file SHA-256 is invalid: {row.get('asset_id')}")
        path = _inside(source, source / relative, must_exist=True)
        if path.stat().st_size != declared_size or _sha256(path) != declared_hash:
            raise ValueError(f"package file binding drift: {row.get('asset_id')}:{relative}")
        normalized.append({"path": relative.as_posix(), "size": declared_size, "sha256": declared_hash})
    normalized.sort(key=lambda item: item["path"])
    aggregate = _canonical_sha256(normalized)
    declared_aggregate = str(row.get("package_binding_sha256") or "")
    if declared_aggregate and aggregate != declared_aggregate:
        # The merged release uses a canonical package binding that records
        # `bytes` in one nested field and `size` in another.  Recompute that
        # exact representation before declaring drift.
        raw_normalized = []
        for entry in raw_files:
            assert isinstance(entry, Mapping)
            raw_normalized.append(
                {
                    "path": str(entry["path"]),
                    "sha256": str(entry["sha256"]),
                    "size": int(entry.get("size", entry.get("bytes", -1))),
                }
            )
        raw_normalized.sort(key=lambda item: item["path"])
        if _canonical_sha256(raw_normalized) != declared_aggregate:
            raise ValueError(f"package binding aggregate drift: {row.get('asset_id')}")
    return declared_aggregate or aggregate


def load_render_items(
    dataset_manifest: Path,
    *,
    output_root: Path,
    strict_counts: bool = True,
    validate_inputs: bool = False,
) -> tuple[RenderItem, ...]:
    """Load the complete official roster and mark one winner per category."""

    dataset_manifest = dataset_manifest.expanduser().resolve(strict=True)
    output_root = output_root.expanduser().resolve()
    manifest, rows = _load_manifest(dataset_manifest, strict_counts=strict_counts)
    if strict_counts and len(rows) != EXPECTED_MODEL_COUNT:
        raise ValueError(f"expected {EXPECTED_MODEL_COUNT} rows, found {len(rows)}")

    parsed: list[tuple[dict[str, Any], Path, Path, str]] = []
    ids: set[str] = set()
    categories: set[str] = set()
    for expected_ordinal, row in enumerate(rows):
        if row.get("ordinal") != expected_ordinal:
            raise ValueError(f"Articraft roster ordinal mismatch at row {expected_ordinal}")
        asset_id = str(row.get("asset_id") or "").strip()
        category = str(row.get("category") or row.get("raw_category") or "").strip()
        source_value = str(row.get("source_path") or "").strip()
        urdf_value = str(row.get("primary_urdf_path") or "").strip()
        if not asset_id or asset_id in ids:
            raise ValueError(f"Articraft asset identity is empty or duplicated: {asset_id!r}")
        if not category:
            raise ValueError(f"Articraft category is missing: {asset_id}")
        if not source_value or not urdf_value:
            raise ValueError(f"Articraft source binding is missing: {asset_id}")
        source = Path(source_value).expanduser().resolve(strict=True)
        urdf = Path(urdf_value).expanduser().resolve(strict=True)
        if not source.is_dir() or not urdf.is_file():
            raise ValueError(f"Articraft package is unavailable: {asset_id}")
        try:
            urdf.relative_to(source)
        except ValueError as exc:
            raise ValueError(f"primary URDF escapes package: {asset_id}") from exc
        if urdf.name != "model.urdf":
            raise ValueError(f"primary URDF must be model.urdf: {asset_id}")
        declared_size = int(row.get("primary_urdf_size", row.get("primary_urdf_bytes", -1)))
        declared_hash = str(row.get("primary_urdf_sha256") or "")
        if declared_size < 0 or SHA256_RE.fullmatch(declared_hash) is None:
            raise ValueError(f"primary URDF receipt is malformed: {asset_id}")
        if validate_inputs and (
            urdf.stat().st_size != declared_size or _sha256(urdf) != declared_hash
        ):
            raise ValueError(f"primary URDF receipt drift: {asset_id}")
        ids.add(asset_id)
        categories.add(category)
        parsed.append((row, source, urdf, declared_hash))
    if strict_counts and len(categories) != EXPECTED_CATEGORY_COUNT:
        raise ValueError(f"expected {EXPECTED_CATEGORY_COUNT} Articraft categories, found {len(categories)}")

    winners: dict[str, tuple[str, str]] = {}
    for row, _source, _urdf, _hash in parsed:
        category = str(row.get("category") or row.get("raw_category") or "")
        asset_id = str(row["asset_id"])
        identity = _identity_sha256(asset_id)
        candidate = (identity, asset_id)
        if category not in winners or candidate < winners[category]:
            winners[category] = candidate

    items: list[RenderItem] = []
    for row, source, urdf, urdf_hash in parsed:
        category = str(row.get("category") or row.get("raw_category") or "")
        asset_id = str(row["asset_id"])
        identity = _identity_sha256(asset_id)
        package_binding = str(row.get("package_binding_sha256") or "")
        if not SHA256_RE.fullmatch(package_binding):
            nested = row.get("package_binding")
            package_binding = (
                str(nested.get("content_manifest_sha256"))
                if isinstance(nested, Mapping)
                else ""
            )
        if not SHA256_RE.fullmatch(package_binding):
            raise ValueError(f"package binding receipt is malformed: {asset_id}")
        if validate_inputs:
            _validate_package_files(row, source)
        output_path = output_root / category / asset_id / "imgs" / "000.png"
        try:
            output_path.resolve().relative_to(output_root)
        except ValueError as exc:
            raise ValueError(f"output path escapes output root: {asset_id}") from exc
        items.append(
            RenderItem(
                ordinal=int(row["ordinal"]),
                category=category,
                asset_id=asset_id,
                cohort_origin=str(row.get("cohort_origin") or ""),
                source_path=source,
                urdf_path=urdf,
                urdf_bytes=int(row.get("primary_urdf_size", row.get("primary_urdf_bytes", urdf.stat().st_size))),
                urdf_sha256=urdf_hash,
                package_binding_sha256=package_binding,
                identity_sha256=identity,
                category_one_shot=(winners[category] == (identity, asset_id)),
                output_path=output_path,
            )
        )
    return tuple(items)


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


def _blender_version(blender: Path) -> str:
    result = subprocess.run(
        [str(blender), "--version"],
        check=True,
        capture_output=True,
        text=True,
        timeout=60,
    )
    return result.stdout.splitlines()[0].strip()


def build_run_config(
    *,
    args: argparse.Namespace,
    all_items: Sequence[RenderItem],
    selected: Sequence[RenderItem],
    renderer: Path,
    shared_renderer: Path,
    blender: Path,
) -> dict[str, Any]:
    manifest = args.dataset_manifest.expanduser().resolve(strict=True)
    return {
        "schema_version": 1,
        "render_contract": "articraft10k_uniform_studio_v1",
        "dataset": "Articraft-10K",
        "model_count": len(all_items),
        "category_count": len({item.category for item in all_items}),
        "selected_count": len(selected),
        "selected_category_count": len({item.category for item in selected}),
        "dataset_manifest": str(manifest),
        "dataset_manifest_sha256": _sha256(manifest),
        "output_root": str(args.output_root.expanduser().resolve()),
        "driver": str(SCRIPT),
        "driver_sha256": _sha256(SCRIPT),
        "renderer": str(renderer),
        "renderer_sha256": _sha256(renderer),
        "shared_renderer": str(shared_renderer),
        "shared_renderer_sha256": _sha256(shared_renderer),
        "blender": str(blender),
        "blender_version": _blender_version(blender),
        "resolution": args.resolution,
        "samples": args.samples,
        "pose_policy": "URDF rest pose; all movable joint coordinates are zero",
        "material_policy": "URDF visual rgba; fixed neutral fallback for missing colors",
        "image_layout": "category/asset_id/imgs/000.png",
        "selection": {
            "one_shot_only": bool(args.one_shot_only),
            "rule": (
                "minimum (SHA256(asset_id UTF-8), asset_id) within category over all "
                f"{len(all_items):,} manifest rows"
            ),
            "full_roster_count": len(all_items),
            "full_category_count": len({item.category for item in all_items}),
            "selected_roster": "category_one_shot_roster.csv",
            "selected_receipt": _selection_receipt(selected),
        },
        "studio": _studio_contract(),
        "gpu_visibility": str(args.gpu),
        "workers": args.workers,
        "timeout_seconds": args.timeout_seconds,
    }


def _stable_config(config: Mapping[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in config.items() if key not in {"workers", "timeout_seconds"}}


def _atomic_write_text(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.is_symlink():
        raise ValueError(f"refusing to overwrite symlink: {path}")
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    temporary = Path(temp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _write_json(path: Path, payload: Any) -> None:
    _atomic_write_text(path, json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True) + "\n")


ROSTER_FIELDS = [
    "ordinal",
    "category",
    "asset_id",
    "cohort_origin",
    "source_path",
    "urdf_path",
    "urdf_bytes",
    "urdf_sha256",
    "package_binding_sha256",
    "identity_sha256",
    "category_one_shot",
    "output_path",
]


def _write_one_shot_roster(path: Path, items: Sequence[RenderItem]) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    path.parent.mkdir(parents=True, exist_ok=True)
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=ROSTER_FIELDS)
        writer.writeheader()
        for item in sorted((x for x in items if x.category_one_shot), key=lambda x: x.category):
            writer.writerow(
                {
                    "ordinal": item.ordinal,
                    "category": item.category,
                    "asset_id": item.asset_id,
                    "cohort_origin": item.cohort_origin,
                    "source_path": str(item.source_path),
                    "urdf_path": str(item.urdf_path),
                    "urdf_bytes": item.urdf_bytes,
                    "urdf_sha256": item.urdf_sha256,
                    "package_binding_sha256": item.package_binding_sha256,
                    "identity_sha256": item.identity_sha256,
                    "category_one_shot": item.category_one_shot,
                    "output_path": str(item.output_path),
                }
            )
    os.replace(temporary, path)


MANIFEST_FIELDS = [
    "ordinal",
    "category",
    "asset_id",
    "cohort_origin",
    "source_path",
    "urdf_path",
    "urdf_bytes",
    "urdf_sha256",
    "package_binding_sha256",
    "identity_sha256",
    "category_one_shot",
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


def _write_manifest(path: Path, results: Iterable[Mapping[str, Any]]) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    path.parent.mkdir(parents=True, exist_ok=True)
    ordered = sorted(results, key=lambda row: int(row["ordinal"]))
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()
        for result in ordered:
            row = {field: result.get(field, "") for field in MANIFEST_FIELDS}
            if not isinstance(row["renderer_result"], str):
                row["renderer_result"] = json.dumps(
                    row["renderer_result"], sort_keys=True, ensure_ascii=True
                )
            writer.writerow(row)
    os.replace(temporary, path)


def _read_recovery_receipts(
    *,
    manifest_path: Path,
    state_path: Path,
    roster: Mapping[str, RenderItem],
) -> dict[str, dict[str, Any]]:
    receipts: dict[str, dict[str, Any]] = {}

    def validated(row: Mapping[str, Any], source: Path) -> tuple[str, dict[str, Any]]:
        asset_id = str(row.get("asset_id") or "")
        item = roster.get(asset_id)
        if item is None:
            raise ValueError(f"prior receipt contains unknown Articraft asset in {source}: {asset_id!r}")
        try:
            ordinal = int(row.get("ordinal", -1))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid prior receipt ordinal in {source}") from exc
        if ordinal != item.ordinal:
            raise ValueError(f"prior receipt ordinal drift for {asset_id}")
        return asset_id, dict(row)

    if manifest_path.is_file():
        with manifest_path.open("r", encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        for row in rows:
            asset_id, receipt = validated(row, manifest_path)
            if asset_id in receipts:
                raise ValueError(f"duplicate prior manifest receipt for {asset_id}")
            receipts[asset_id] = receipt
    if state_path.is_file():
        text = state_path.read_text(encoding="utf-8")
        lines = text.splitlines()
        for line_number, line in enumerate(lines, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                if line_number == len(lines) and not text.endswith("\n"):
                    break
                raise ValueError(f"invalid recovery JSON at {state_path}:{line_number}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"non-object recovery row at {state_path}:{line_number}")
            asset_id, receipt = validated(row, state_path)
            # The state file is an append-only journal.  Rows already present
            # in a checkpointed manifest, and later retry rows for the same
            # asset, are expected; the newest valid journal row wins.
            receipts[asset_id] = receipt
    return receipts


def _receipt_allows_reuse(
    item: RenderItem,
    receipt: Mapping[str, Any] | None,
    *,
    resolution: int,
) -> bool:
    if not receipt or receipt.get("status") not in SUCCESS_STATUSES:
        return False
    for field, expected in (
        ("ordinal", str(item.ordinal)),
        ("category", item.category),
        ("asset_id", item.asset_id),
        ("urdf_sha256", item.urdf_sha256),
        ("package_binding_sha256", item.package_binding_sha256),
        ("identity_sha256", item.identity_sha256),
        ("output_path", str(item.output_path)),
    ):
        if str(receipt.get(field) or "") != expected:
            return False
    try:
        output = Path(str(receipt.get("output_path"))).expanduser().resolve()
        png_bytes = int(receipt.get("png_bytes") or -1)
    except (TypeError, ValueError, OSError):
        return False
    if output != item.output_path.resolve() or not _valid_png(output, resolution):
        return False
    if output.stat().st_size != png_bytes:
        return False
    declared_hash = str(receipt.get("png_sha256") or "")
    return bool(SHA256_RE.fullmatch(declared_hash)) and _sha256(output) == declared_hash


def _render_one(
    item: RenderItem,
    *,
    args: argparse.Namespace,
    blender: Path,
    renderer: Path,
    shared_renderer: Path,
    shared_renderer_sha256: str,
    reuse_receipt: Mapping[str, Any] | None,
) -> dict[str, Any]:
    started_at = _utc_now()
    start = time.monotonic()
    base: dict[str, Any] = {
        "ordinal": item.ordinal,
        "category": item.category,
        "asset_id": item.asset_id,
        "cohort_origin": item.cohort_origin,
        "source_path": str(item.source_path),
        "urdf_path": str(item.urdf_path),
        "urdf_bytes": item.urdf_bytes,
        "urdf_sha256": item.urdf_sha256,
        "package_binding_sha256": item.package_binding_sha256,
        "identity_sha256": item.identity_sha256,
        "category_one_shot": item.category_one_shot,
        "output_path": str(item.output_path),
        "started_at": started_at,
    }
    if not args.force and _receipt_allows_reuse(item, reuse_receipt, resolution=args.resolution):
        return {
            **base,
            "status": "reused_valid",
            "elapsed_seconds": 0.0,
            "png_bytes": item.output_path.stat().st_size,
            "png_sha256": _sha256(item.output_path),
            "finished_at": _utc_now(),
            "error": "",
            "renderer_result": None,
        }

    item.output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_output = item.output_path.with_name(
        f".{item.output_path.stem}.{os.getpid()}.{time.time_ns()}.tmp.png"
    )
    log_path = args.output_root / "logs" / f"{item.ordinal:05d}.log"
    command = [
        str(blender),
        "-b",
        "--factory-startup",
        "-noaudio",
        "-P",
        str(renderer),
        "--",
        "--asset-dir",
        str(item.source_path),
        "--output",
        str(temporary_output),
        "--resolution",
        str(args.resolution),
        "--samples",
        str(args.samples),
        "--shared-renderer",
        str(shared_renderer),
        "--shared-renderer-sha256",
        shared_renderer_sha256,
    ]
    environment = os.environ.copy()
    environment["CUDA_VISIBLE_DEVICES"] = str(args.gpu)
    status = "failed"
    error = ""
    output = ""
    renderer_result: dict[str, Any] | None = None
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
            os.replace(temporary_output, item.output_path)
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
        temporary_output.unlink(missing_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(output, encoding="utf-8", errors="replace")
    return {
        **base,
        "status": status,
        "elapsed_seconds": round(time.monotonic() - start, 3),
        "png_bytes": item.output_path.stat().st_size if status == "rendered" else 0,
        "png_sha256": _sha256(item.output_path) if status == "rendered" else "",
        "finished_at": _utc_now(),
        "error": error,
        "renderer_result": renderer_result,
    }


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
        output_root=args.output_root,
        strict_counts=not args.allow_count_drift,
        validate_inputs=False,
    )
    selected = tuple(item for item in all_items if not args.one_shot_only or item.category_one_shot)
    if args.categories:
        wanted = set(args.categories)
        known = {item.category for item in all_items}
        missing = sorted(wanted - known)
        if missing:
            raise ValueError(f"unknown --categories: {', '.join(missing)}")
        selected = tuple(item for item in selected if item.category in wanted)
    if args.limit is not None:
        if args.limit < 1:
            raise ValueError("--limit must be positive")
        selected = selected[: args.limit]
    if not selected:
        raise ValueError("selection contains no Articraft models")

    # Hash package closures for selected winners before writing the contract.
    # This catches source drift while retaining a cheap metadata-only dry run.
    if not args.dry_run:
        refreshed: list[RenderItem] = []
        manifest, rows = _load_manifest(
            args.dataset_manifest,
            strict_counts=not args.allow_count_drift,
        )
        rows_by_id = {str(row.get("asset_id")): row for row in rows}
        for item in selected:
            row = rows_by_id[item.asset_id]
            actual_binding = _validate_package_files(row, item.source_path)
            if actual_binding != item.package_binding_sha256:
                raise ValueError(f"selected package binding drift: {item.asset_id}")
            if item.urdf_path.stat().st_size != item.urdf_bytes or _sha256(item.urdf_path) != item.urdf_sha256:
                raise ValueError(f"selected URDF receipt drift: {item.asset_id}")
            refreshed.append(item)
        selected = tuple(refreshed)

    config = build_run_config(
        args=args,
        all_items=all_items,
        selected=selected,
        renderer=renderer,
        shared_renderer=shared_renderer,
        blender=blender,
    )
    config_path = args.output_root / "render_config.json"
    if config_path.is_file():
        previous = json.loads(config_path.read_text(encoding="utf-8"))
        if _stable_config(previous) != _stable_config(config):
            raise ValueError(f"output root contains a different render contract: {config_path}")
    else:
        if args.output_root.is_dir() and any(args.output_root.iterdir()):
            raise ValueError(f"non-empty output root has no render_config.json: {args.output_root}")
        _write_json(config_path, config)
    args.output_root.mkdir(parents=True, exist_ok=True)
    _write_one_shot_roster(args.output_root / "category_one_shot_roster.csv", all_items)

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
        f"[render] {len(selected)}/{len(all_items)} Articraft models -> {args.output_root} "
        f"({args.resolution}px, {args.samples} samples, GPU {args.gpu}, workers={args.workers})",
        flush=True,
    )
    state_path = args.output_root / "render_state.jsonl"
    manifest_path = args.output_root / "render_manifest.csv"
    roster = {item.asset_id: item for item in all_items}
    prior = _read_recovery_receipts(
        manifest_path=manifest_path,
        state_path=state_path,
        roster=roster,
    )
    latest = dict(prior)
    started_at = _utc_now()
    results: list[dict[str, Any]] = []
    with state_path.open("a", encoding="utf-8") as state_stream:
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {
                pool.submit(
                    _render_one,
                    item,
                    args=args,
                    blender=blender,
                    renderer=renderer,
                    shared_renderer=shared_renderer,
                    shared_renderer_sha256=str(config["shared_renderer_sha256"]),
                    reuse_receipt=prior.get(item.asset_id),
                ): item
                for item in selected
            }
            for completed, future in enumerate(as_completed(futures), 1):
                result = future.result()
                results.append(result)
                latest[str(result["asset_id"])] = result
                state_stream.write(json.dumps(result, sort_keys=True, ensure_ascii=True) + "\n")
                state_stream.flush()
                if completed % args.checkpoint_every == 0:
                    _write_manifest(manifest_path, latest.values())
                print(
                    f"[render] {completed}/{len(selected)} {result['category']}/"
                    f"{result['asset_id']} {result['status']} ({result['elapsed_seconds']:.1f}s)",
                    flush=True,
                )
    _write_manifest(manifest_path, latest.values())
    failures = [result for result in results if result["status"] not in SUCCESS_STATUSES]
    selected_valid_count = sum(_valid_png(item.output_path, args.resolution) for item in selected)
    summary = {
        "schema_version": 1,
        "render_contract": "articraft10k_uniform_studio_v1",
        "dataset_manifest": str(args.dataset_manifest.expanduser().resolve(strict=True)),
        "started_at": started_at,
        "finished_at": _utc_now(),
        **selection_summary,
        "rendered_count": sum(result["status"] == "rendered" for result in results),
        "reused_valid_count": sum(result["status"] == "reused_valid" for result in results),
        "failure_count": len(failures),
        "selected_valid_png_count": selected_valid_count,
        "selected_complete": selected_valid_count == len(selected),
        "failure_asset_ids": [str(result["asset_id"]) for result in failures],
        "manifest": str(manifest_path),
        "config": str(config_path),
        "one_shot_roster": str(args.output_root / "category_one_shot_roster.csv"),
    }
    _write_json(args.output_root / "render_summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    if failures:
        raise RuntimeError(f"{len(failures)} Articraft render(s) failed; rerun to resume")
    return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-manifest", type=Path, default=DEFAULT_DATASET_MANIFEST)
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
    parser.add_argument("--one-shot-only", dest="one_shot_only", action="store_true")
    parser.add_argument("--all-models", dest="one_shot_only", action="store_false")
    parser.set_defaults(one_shot_only=True)
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
