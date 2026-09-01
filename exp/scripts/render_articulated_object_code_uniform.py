#!/usr/bin/env python3
"""Render one deterministic Articulated-Object-Code representative per category.

The lightweight ``manifest.csv`` is the source index.  All viable and
loads-only rows are candidates; viable rows are preferred for a category and
loads-only is used only when that category has no viable model.  Within the
chosen tier, the minimum ``(SHA256(rel_path UTF-8), rel_path)`` wins.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
import re
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PureWindowsPath
from typing import Any, Iterable, Mapping, Sequence

from PIL import Image


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_DATASET_MANIFEST = REPO_ROOT / "exp/Articulated-Object-Code/manifest.csv"
DEFAULT_SOURCE_ROOT = REPO_ROOT / "exp/Articulated-Object-Code/released_outputs"
DEFAULT_OUTPUT_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/Articulated-Object-Code/renders/"
    "uniform660_one_per_category_studio_256_v1"
)
DEFAULT_RENDERER = REPO_ROOT / "exp/scripts/render_articulated_object_code_asset_blender.py"
DEFAULT_BASE_RENDERER = REPO_ROOT / "exp/scripts/render_articraft10k_asset_blender.py"
DEFAULT_SHARED_RENDERER = REPO_ROOT / "arti-template/scripts/render_exported_asset_blender.py"
DEFAULT_BLENDER = Path(
    "/mnt/zsn/zsn_workspace/VoxHammer/third_party/blender-4.2.19-linux-x64/blender"
)
EXPECTED_DATASET = "Articulated-Object-Code"
EXPECTED_MODEL_COUNT = 3_217
EXPECTED_CANDIDATE_COUNT = 2_832
EXPECTED_CATEGORY_COUNT = 660
EXPECTED_TIERS = ("viable", "loads_only")
SUCCESS_STATUSES = {"rendered", "reused_valid"}
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
MATERIAL_POLICY = "native OBJ/MTL diffuse materials, forced opaque; neutral fallback"


@dataclass(frozen=True, slots=True)
class RenderItem:
    ordinal: int
    category: str
    asset_id: str
    tier: str
    source_path: Path
    source_relative_path: str
    package_path: str
    urdf_path: Path
    urdf_relative_path: str
    urdf_bytes: int
    urdf_sha256: str
    package_file_count: int
    package_total_bytes: int
    package_content_manifest_sha256: str
    package_binding_sha256: str
    selection_sha256: str
    identity: str
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


def _identity_sha256(identity: str) -> str:
    return hashlib.sha256(identity.encode("utf-8")).hexdigest()


def _selection_sha256(rel_path: str) -> str:
    return hashlib.sha256(rel_path.encode("utf-8")).hexdigest()


def _selection_receipt(items: Sequence[RenderItem]) -> dict[str, Any]:
    rows = sorted(
        (
            {
                "category": item.category,
                "asset_id": item.asset_id,
                "tier": item.tier,
                "identity": item.identity,
                "selection_sha256": item.selection_sha256,
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


def _safe_component(value: str, *, field: str) -> str:
    relative = _safe_relative(value, field=field)
    if len(relative.parts) != 1:
        raise ValueError(f"{field} must be one path component")
    return relative.name


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


def _load_manifest(path: Path, *, strict_counts: bool = True) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Load the immutable CSV index shipped with Articulated-Object-Code."""
    path = path.expanduser().resolve(strict=True)
    if path.suffix.lower() != ".csv":
        raise ValueError("Articulated-Object-Code manifest must be manifest.csv")
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = [dict(row) for row in csv.DictReader(stream)]
    if strict_counts and len(rows) != EXPECTED_MODEL_COUNT:
        raise ValueError(f"expected exactly {EXPECTED_MODEL_COUNT} AOC rows, found {len(rows)}")
    required = {"object_release_id", "category", "tier", "status", "rel_path"}
    missing = sorted(required - set(rows[0] if rows else ()))
    if missing:
        raise ValueError(f"AOC manifest is missing fields: {missing}")
    metadata = {
        "path": str(path),
        "sha256": _sha256(path),
        "row_count": len(rows),
        "candidate_count": sum(row.get("tier") in EXPECTED_TIERS for row in rows),
        "candidate_category_count": len({row.get("category") for row in rows if row.get("tier") in EXPECTED_TIERS}),
    }
    return metadata, rows


def _package_receipt(source: Path) -> tuple[int, int, str, str]:
    """Hash the URDF render-dependency closure for reproducible receipts."""
    source = source.expanduser().resolve(strict=True)
    if not source.is_dir():
        raise ValueError(f"package is not a directory: {source}")
    urdf = (source / "generated.urdf").resolve(strict=True)
    try:
        robot = ET.parse(urdf).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ValueError(f"cannot parse selected generated.urdf: {source}") from exc
    files = {urdf}
    for mesh in robot.findall(".//visual/geometry/mesh"):
        relative = _safe_relative(mesh.get("filename"), field="URDF mesh filename")
        path = (source / relative).resolve(strict=True)
        path.relative_to(source)
        files.add(path)
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError as exc:
            raise ValueError(f"cannot read selected OBJ: {path}") from exc
        for line in lines:
            if not line.lstrip().lower().startswith("mtllib "):
                continue
            for name in line.split()[1:]:
                mtl = (path.parent / name).resolve(strict=True)
                mtl.relative_to(source)
                files.add(mtl)
                for mtl_line in mtl.read_text(encoding="utf-8", errors="replace").splitlines():
                    parts = mtl_line.split()
                    if parts and parts[0].lower().startswith("map_") and len(parts) >= 2:
                        texture = (mtl.parent / parts[-1]).resolve(strict=True)
                        texture.relative_to(source)
                        files.add(texture)
    entries: list[dict[str, Any]] = []
    for path in sorted(files, key=lambda item: item.relative_to(source).as_posix()):
        relative = path.relative_to(source).as_posix()
        entries.append({"path": relative, "size": path.stat().st_size, "sha256": _sha256(path)})
    if not entries:
        raise ValueError(f"package is empty: {source}")
    content_hash = _canonical_sha256(entries)
    binding_hash = _canonical_sha256({"root": source.name, "files": entries})
    return len(entries), sum(int(entry["size"]) for entry in entries), content_hash, binding_hash


def load_render_items(
    dataset_manifest: Path,
    *,
    output_root: Path,
    strict_counts: bool = True,
    validate_inputs: bool = False,
    source_root: Path = DEFAULT_SOURCE_ROOT,
) -> tuple[RenderItem, ...]:
    """Load the full index and mark one preferred winner per category."""
    dataset_manifest = dataset_manifest.expanduser().resolve(strict=True)
    output_root = output_root.expanduser().resolve()
    source_root = source_root.expanduser().resolve()
    _metadata, rows = _load_manifest(dataset_manifest, strict_counts=strict_counts)
    candidates = [row for row in rows if row.get("tier") in EXPECTED_TIERS]
    if strict_counts and len(candidates) != EXPECTED_CANDIDATE_COUNT:
        raise ValueError(f"expected {EXPECTED_CANDIDATE_COUNT} viable/loads_only rows, found {len(candidates)}")
    if strict_counts and len({row.get("category") for row in candidates}) != EXPECTED_CATEGORY_COUNT:
        raise ValueError("AOC candidate category count drifted")
    ids: set[str] = set()
    by_category: dict[str, list[dict[str, Any]]] = {}
    for row in candidates:
        asset_id = _safe_component(str(row.get("object_release_id") or "").strip(), field="object_release_id")
        category = _safe_component(str(row.get("category") or "").strip(), field=f"category for {asset_id}")
        rel = _safe_relative(row.get("rel_path"), field=f"rel_path for {asset_id}")
        if asset_id in ids:
            raise ValueError(f"duplicate AOC object_release_id: {asset_id}")
        if rel.as_posix().split("/", 1)[0] != "objects":
            raise ValueError(f"AOC rel_path must be under objects/: {rel}")
        ids.add(asset_id)
        row["_asset_id"], row["_category"], row["_rel"] = asset_id, category, rel.as_posix()
        by_category.setdefault(category, []).append(row)
    winners: dict[str, tuple[str, str]] = {}
    for category, group in by_category.items():
        viable = [row for row in group if row.get("tier") == "viable"]
        pool = viable or [row for row in group if row.get("tier") == "loads_only"]
        winner = min(pool, key=lambda row: (_selection_sha256(row["_rel"]), row["_rel"]))
        winners[category] = (str(winner["tier"]), str(winner["_rel"]))

    items: list[RenderItem] = []
    winner_ordinals = {category: ordinal for ordinal, category in enumerate(sorted(winners))}
    nonwinner_ordinal = len(winner_ordinals)
    for row in candidates:
        asset_id, category, rel = row["_asset_id"], row["_category"], row["_rel"]
        tier = str(row["tier"])
        source = (source_root / rel).resolve()
        urdf_rel = f"{rel}/generated.urdf"
        urdf = (source_root / urdf_rel).resolve()
        one_shot = winners[category] == (tier, rel)
        ordinal = winner_ordinals[category] if one_shot else nonwinner_ordinal
        if not one_shot:
            nonwinner_ordinal += 1
        urdf_bytes = 0
        urdf_hash = ""
        package_count = package_total = 0
        package_content_hash = package_binding_hash = ""
        if validate_inputs and one_shot:
            if not source.is_dir() or not urdf.is_file() or urdf.parent != source:
                raise ValueError(f"selected AOC package is unavailable: {rel}")
            urdf_bytes, urdf_hash = urdf.stat().st_size, _sha256(urdf)
            package_count, package_total, package_content_hash, package_binding_hash = _package_receipt(source)
        identity = f"{tier}:{rel}"
        output_path = output_root / category / asset_id / "imgs" / "000.png"
        output_path.resolve().relative_to(output_root)
        items.append(RenderItem(
            ordinal=ordinal, category=category, asset_id=asset_id, tier=tier,
            source_path=source, source_relative_path=rel, package_path=rel,
            urdf_path=urdf, urdf_relative_path=urdf_rel, urdf_bytes=urdf_bytes,
            urdf_sha256=urdf_hash, package_file_count=package_count,
            package_total_bytes=package_total,
            package_content_manifest_sha256=package_content_hash,
            package_binding_sha256=package_binding_hash,
            selection_sha256=_selection_sha256(rel), identity=identity,
            identity_sha256=_identity_sha256(identity), category_one_shot=one_shot,
            output_path=output_path,
        ))
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
    base_renderer: Path,
    shared_renderer: Path,
    blender: Path,
) -> dict[str, Any]:
    manifest_path = args.dataset_manifest.expanduser().resolve(strict=True)
    strict_release = not bool(getattr(args, "allow_count_drift", False))
    manifest_meta, _rows = _load_manifest(manifest_path, strict_counts=strict_release)
    source_root = args.source_root.expanduser().resolve()
    return {
        "schema_version": 1,
        "render_contract": "articulated_object_code_uniform_studio_v1",
        "dataset": EXPECTED_DATASET,
        "model_count": len(all_items),
        "category_count": len({item.category for item in all_items}),
        "official_model_count": EXPECTED_MODEL_COUNT,
        "official_category_count": 787,
        "loadable_model_count": len(all_items),
        "candidate_model_count": len(all_items),
        "loadable_category_count": len({item.category for item in all_items}),
        "selected_count": len(selected),
        "selected_category_count": len({item.category for item in selected}),
        "dataset_manifest": str(manifest_path),
        "dataset_manifest_sha256": _sha256(manifest_path),
        "dataset_manifest_content_sha256": manifest_meta["sha256"],
        "dataset_roster_sha256": manifest_meta["sha256"],
        "source_root": str(source_root),
        "output_root": str(args.output_root.expanduser().resolve()),
        "driver": str(SCRIPT),
        "driver_sha256": _sha256(SCRIPT),
        "renderer": str(renderer),
        "renderer_sha256": _sha256(renderer),
        "base_renderer": str(base_renderer),
        "base_renderer_sha256": _sha256(base_renderer),
        "shared_renderer": str(shared_renderer),
        "shared_renderer_sha256": _sha256(shared_renderer),
        "blender": str(blender),
        "blender_version": _blender_version(blender),
        "resolution": args.resolution,
        "samples": args.samples,
        "pose_policy": "URDF rest pose; all movable joint coordinates are zero",
        "material_policy": MATERIAL_POLICY,
        "package_validation": "selected package complete closure, byte count, and SHA-256",
        "image_layout": "category/asset_id/imgs/000.png",
        "selection": {
            "one_shot_only": bool(args.one_shot_only),
            "rule": (
                "prefer viable over loads_only; within tier minimum "
                "(SHA256(rel_path UTF-8), rel_path)"
            ),
            "universe": "Articulated-Object-Code manifest.csv; viable + loads_only candidates",
            "full_roster_count": len(all_items),
            "full_category_count": len({item.category for item in all_items}),
            "category_one_shot_roster": "category_one_shot_roster.csv",
            "category_one_shot_roster_scope": "all full-release categories, independent of runtime filters",
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
    "dataset_id",
    "tier",
    "identity",
    "selection_sha256",
    "source_path",
    "package_path",
    "source_relative_path",
    "urdf_path",
    "urdf_relative_path",
    "urdf_bytes",
    "urdf_sha256",
    "package_file_count",
    "package_total_bytes",
    "package_content_manifest_sha256",
    "package_binding_sha256",
    "identity_sha256",
    "category_one_shot",
    "output_path",
]


def _item_row(item: RenderItem) -> dict[str, Any]:
    return {
        "ordinal": item.ordinal,
        "category": item.category,
        "asset_id": item.asset_id,
        "dataset_id": item.asset_id,
        "tier": item.tier,
        "identity": item.identity,
        "selection_sha256": item.selection_sha256,
        "source_path": str(item.source_path),
        "package_path": item.package_path,
        "source_relative_path": item.source_relative_path,
        "urdf_path": str(item.urdf_path),
        "urdf_relative_path": item.urdf_relative_path,
        "urdf_bytes": item.urdf_bytes,
        "urdf_sha256": item.urdf_sha256,
        "package_file_count": item.package_file_count,
        "package_total_bytes": item.package_total_bytes,
        "package_content_manifest_sha256": item.package_content_manifest_sha256,
        "package_binding_sha256": item.package_binding_sha256,
        "identity_sha256": item.identity_sha256,
        "category_one_shot": item.category_one_shot,
        "output_path": str(item.output_path),
    }


def _write_one_shot_roster(path: Path, items: Sequence[RenderItem]) -> None:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=ROSTER_FIELDS)
    writer.writeheader()
    for ordinal, item in enumerate(sorted((x for x in items if x.category_one_shot), key=lambda x: x.category)):
        row = _item_row(item)
        row["ordinal"] = ordinal
        writer.writerow(row)
    _atomic_write_text(path, stream.getvalue())


MANIFEST_FIELDS = ROSTER_FIELDS + [
    "status",
    "elapsed_seconds",
    "png_bytes",
    "png_sha256",
    "image_bytes",
    "image_sha256",
    "image",
    "started_at",
    "finished_at",
    "error",
    "renderer_result",
]


def _write_manifest(path: Path, results: Iterable[Mapping[str, Any]]) -> None:
    stream = io.StringIO(newline="")
    ordered = sorted(results, key=lambda row: int(row["ordinal"]))
    writer = csv.DictWriter(stream, fieldnames=MANIFEST_FIELDS)
    writer.writeheader()
    for result in ordered:
        row = {field: result.get(field, "") for field in MANIFEST_FIELDS}
        if not isinstance(row["renderer_result"], str):
            row["renderer_result"] = json.dumps(
                row["renderer_result"], sort_keys=True, ensure_ascii=True
            )
        writer.writerow(row)
    _atomic_write_text(path, stream.getvalue())


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
            raise ValueError(f"prior receipt contains unknown AOC asset in {source}: {asset_id!r}")
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
            receipts[asset_id] = receipt
    return receipts


def _receipt_allows_reuse(
    item: RenderItem,
    receipt: Mapping[str, Any] | None,
    *,
    resolution: int,
    base_renderer: Path,
    base_renderer_sha256: str,
    shared_renderer: Path,
    shared_renderer_sha256: str,
) -> bool:
    if not receipt or receipt.get("status") not in SUCCESS_STATUSES:
        return False
    for field, expected in (
        ("ordinal", str(item.ordinal)),
        ("category", item.category),
        ("asset_id", item.asset_id),
        ("urdf_sha256", item.urdf_sha256),
        ("package_content_manifest_sha256", item.package_content_manifest_sha256),
        ("package_binding_sha256", item.package_binding_sha256),
        ("identity_sha256", item.identity_sha256),
        ("output_path", str(item.output_path)),
    ):
        observed = receipt.get(field)
        if str("" if observed is None else observed) != expected:
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
    if not SHA256_RE.fullmatch(declared_hash) or _sha256(output) != declared_hash:
        return False
    renderer_result = _renderer_result_from_receipt(receipt)
    return not _validate_renderer_result(
        renderer_result,
        base_renderer=base_renderer,
        base_renderer_sha256=base_renderer_sha256,
        shared_renderer=shared_renderer,
        shared_renderer_sha256=shared_renderer_sha256,
        expected_asset_dir=item.source_path,
        expected_output=item.output_path,
    )


def _renderer_result_from_receipt(
    receipt: Mapping[str, Any] | None,
) -> dict[str, Any] | None:
    if not receipt:
        return None
    value = receipt.get("renderer_result")
    if isinstance(value, Mapping):
        return dict(value)
    if isinstance(value, str) and value.strip():
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError:
            return None
        if isinstance(parsed, dict):
            return parsed
    return None


def _validate_renderer_result(
    result: Mapping[str, Any] | None,
    *,
    base_renderer: Path,
    base_renderer_sha256: str,
    shared_renderer: Path,
    shared_renderer_sha256: str,
    expected_asset_dir: Path,
    expected_output: Path,
) -> str:
    if not isinstance(result, Mapping):
        return "Blender produced no structured renderer receipt"
    for key, path, digest in (
        ("base_renderer", base_renderer, base_renderer_sha256),
        ("shared_renderer", shared_renderer, shared_renderer_sha256),
    ):
        receipt = result.get(key)
        if not isinstance(receipt, Mapping):
            return f"renderer receipt is missing {key}"
        try:
            observed_path = Path(str(receipt.get("path") or "")).expanduser().resolve(strict=True)
        except OSError:
            return f"renderer receipt has invalid {key} path"
        if observed_path != path or receipt.get("sha256") != digest:
            return f"renderer receipt {key} mismatch"
    for key, expected in (
        ("asset_dir", expected_asset_dir),
        ("output", expected_output),
    ):
        try:
            observed = Path(str(result.get(key) or "")).expanduser().resolve(strict=True)
            expected = expected.expanduser().resolve(strict=True)
        except OSError:
            return f"renderer receipt has invalid {key} path"
        if observed != expected:
            return f"renderer receipt {key} mismatch"
    if result.get("material_policy") != MATERIAL_POLICY:
        return "renderer material policy mismatch"
    return ""


def _render_one(
    item: RenderItem,
    *,
    args: argparse.Namespace,
    blender: Path,
    renderer: Path,
    base_renderer: Path,
    base_renderer_sha256: str,
    shared_renderer: Path,
    shared_renderer_sha256: str,
    reuse_receipt: Mapping[str, Any] | None,
) -> dict[str, Any]:
    started_at = _utc_now()
    start = time.monotonic()
    base = {**_item_row(item), "started_at": started_at}
    if not args.force and _receipt_allows_reuse(
        item,
        reuse_receipt,
        resolution=args.resolution,
        base_renderer=base_renderer,
        base_renderer_sha256=base_renderer_sha256,
        shared_renderer=shared_renderer,
        shared_renderer_sha256=shared_renderer_sha256,
    ):
        preserved_renderer_result = _renderer_result_from_receipt(reuse_receipt)
        assert preserved_renderer_result is not None
        return {
            **base,
            "status": "reused_valid",
            "elapsed_seconds": 0.0,
            "png_bytes": item.output_path.stat().st_size,
            "png_sha256": _sha256(item.output_path),
            "image_bytes": item.output_path.stat().st_size,
            "image_sha256": _sha256(item.output_path),
            "image": str(item.output_path),
            "finished_at": _utc_now(),
            "error": "",
            "renderer_result": preserved_renderer_result,
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
        "--base-renderer",
        str(base_renderer),
        "--base-renderer-sha256",
        base_renderer_sha256,
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
            for line in reversed(process.stdout.splitlines()):
                try:
                    candidate = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(candidate, dict) and candidate.get("output"):
                    renderer_result = candidate
                    break
            error = _validate_renderer_result(
                renderer_result,
                base_renderer=base_renderer,
                base_renderer_sha256=base_renderer_sha256,
                shared_renderer=shared_renderer,
                shared_renderer_sha256=shared_renderer_sha256,
                expected_asset_dir=item.source_path,
                expected_output=temporary_output,
            )
            if not error:
                os.replace(temporary_output, item.output_path)
                status = "rendered"
                assert renderer_result is not None
                renderer_result["output"] = str(item.output_path)
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
        "image_bytes": item.output_path.stat().st_size if status == "rendered" else 0,
        "image_sha256": _sha256(item.output_path) if status == "rendered" else "",
        "image": str(item.output_path),
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
    base_renderer = args.base_renderer.expanduser().resolve(strict=True)
    shared_renderer = args.shared_renderer.expanduser().resolve(strict=True)
    blender = args.blender.expanduser().resolve(strict=True)
    all_items = load_render_items(
        args.dataset_manifest,
        output_root=args.output_root,
        strict_counts=not args.allow_count_drift,
        validate_inputs=not args.dry_run,
        source_root=args.source_root,
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
        raise ValueError("selection contains no Articulated-Object-Code models")

    config = build_run_config(
        args=args,
        all_items=all_items,
        selected=selected,
        renderer=renderer,
        base_renderer=base_renderer,
        shared_renderer=shared_renderer,
        blender=blender,
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

    print(
        f"[render] {len(selected)}/{len(all_items)} Articulated-Object-Code models -> "
        f"{args.output_root} ({args.resolution}px, {args.samples} samples, "
        f"GPU {args.gpu}, workers={args.workers})",
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
                    base_renderer=base_renderer,
                    base_renderer_sha256=str(config["base_renderer_sha256"]),
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
        "render_contract": "articulated_object_code_uniform_studio_v1",
        "dataset_manifest": str(args.dataset_manifest.expanduser().resolve(strict=True)),
        "started_at": started_at,
        "finished_at": _utc_now(),
        **selection_summary,
        "rendered_count": sum(result["status"] == "rendered" for result in results),
        "reused_valid_count": sum(result["status"] == "reused_valid" for result in results),
        "failure_count": len(failures),
        "selected_valid_png_count": selected_valid_count,
        "selected_complete": not failures and selected_valid_count == len(selected),
        "failure_asset_ids": [str(result["asset_id"]) for result in failures],
        "manifest": str(manifest_path),
        "config": str(config_path),
        "one_shot_roster": str(args.output_root / "category_one_shot_roster.csv"),
    }
    _write_json(args.output_root / "render_summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    if failures:
        raise RuntimeError(f"{len(failures)} Articulated-Object-Code render(s) failed; rerun to resume")
    return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-manifest", type=Path, default=DEFAULT_DATASET_MANIFEST)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--renderer", type=Path, default=DEFAULT_RENDERER)
    parser.add_argument("--base-renderer", type=Path, default=DEFAULT_BASE_RENDERER)
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
