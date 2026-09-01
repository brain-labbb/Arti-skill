#!/usr/bin/env python3
"""Render an up-to-five, no-replacement LAM (AOC) panel.

LAM is the local ``Articulated-Object-Code`` release.  Its 660 categories do
not all have five valid assets, so this driver selects ``min(5, support)``
assets per category.  ``viable`` assets are ranked first and ``loads_only``
assets fill the remainder; within a tier the audited SHA-256 path rank is
used.  This makes sample 1 identical to the existing 660-category one-shot
roster, allowing verified PNG reuse.

The script deliberately keeps the AOC package/URDF dependency receipt in the
published roster.  Images use the same Blender worker and studio contract as
the PV-A/Artiverse/Articraft/PartNet comparison.
"""

from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import importlib.util
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path, PureWindowsPath
from typing import Any, Mapping, Sequence


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
BASE_DRIVER = REPO_ROOT / "exp/scripts/render_articulated_object_code_uniform.py"
DEFAULT_DATASET_MANIFEST = REPO_ROOT / "exp/Articulated-Object-Code/manifest.csv"
DEFAULT_SOURCE_ROOT = REPO_ROOT / "exp/Articulated-Object-Code/released_outputs"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "exp/lam_n5_uniform_studio_256_v1"
DEFAULT_REUSE_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/Articulated-Object-Code/renders/"
    "uniform660_one_per_category_studio_256_v1"
)
DEFAULT_RENDERER = REPO_ROOT / "exp/scripts/render_articulated_object_code_asset_blender.py"
DEFAULT_BASE_RENDERER = REPO_ROOT / "exp/scripts/render_articraft10k_asset_blender.py"
DEFAULT_SHARED_RENDERER = REPO_ROOT / "arti-template/scripts/render_exported_asset_blender.py"
DEFAULT_BLENDER = Path(
    "/mnt/zsn/zsn_workspace/VoxHammer/third_party/blender-4.2.19-linux-x64/blender"
)
EXPECTED_MODEL_COUNT = 3_217
EXPECTED_CANDIDATE_COUNT = 2_832
EXPECTED_CATEGORY_COUNT = 660
EXPECTED_TIERS = {"viable", "loads_only"}
SUCCESS_STATUSES = {"rendered", "reused_valid"}
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _load_base() -> Any:
    spec = importlib.util.spec_from_file_location("_lam_aoc_uniform_base", BASE_DRIVER)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load AOC driver: {BASE_DRIVER}")
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolves the defining module through sys.modules.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


BASE = _load_base()


@dataclass(frozen=True, slots=True)
class Candidate:
    """Manifest row normalized without touching the filesystem."""

    category: str
    asset_id: str
    tier: str
    rel_path: str
    identity: str
    identity_sha256: str
    selection_sha256: str


@dataclass(frozen=True, slots=True)
class Sampled:
    candidate: Candidate
    class_id: str
    sample_index: int
    ordinal: int
    render_key: str
    support: int
    balanced_n5_eligible: bool
    item: Any


ROSTER_FIELDS = [
    "ordinal",
    "render_key",
    "generator_index",
    "generator_name",
    "class_id",
    "category",
    "sample_index",
    "category_support",
    "balanced_n5_eligible",
    "source_type",
    "asset_id",
    "tier",
    "identity",
    "selection_sha256",
    "identity_sha256",
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
    "category_one_shot",
    "output_path",
    "png_bytes",
    "png_sha256",
]

MANIFEST_FIELDS = ROSTER_FIELDS + [
    "status",
    "elapsed_seconds",
    "started_at",
    "finished_at",
    "error",
    "gpu",
    "reuse_source_path",
    "reuse_source_manifest",
    "renderer_result",
]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _safe_component(value: object, *, field: str) -> str:
    text = str(value or "").strip()
    path = Path(text)
    windows = PureWindowsPath(text)
    if (
        not text
        or "\\" in text
        or path.is_absolute()
        or windows.is_absolute()
        or bool(windows.drive)
        or len(path.parts) != 1
        or path.parts[0] in {".", ".."}
    ):
        raise ValueError(f"{field} must be one safe path component")
    return path.name


def _safe_relative(value: object, *, field: str) -> str:
    text = str(value or "").strip()
    path = Path(text)
    windows = PureWindowsPath(text)
    if (
        not text
        or "\\" in text
        or path.is_absolute()
        or windows.is_absolute()
        or bool(windows.drive)
        or any(part in {"", ".", ".."} for part in path.parts)
    ):
        raise ValueError(f"{field} must be a contained relative path")
    return path.as_posix()


def load_candidates(
    manifest: Path,
    *,
    strict_counts: bool = True,
) -> tuple[dict[str, Any], tuple[Candidate, ...]]:
    """Read only the immutable CSV index; no package resolution is done here."""

    manifest = manifest.expanduser().resolve(strict=True)
    with manifest.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fields = set(reader.fieldnames or ())
    required = {"object_release_id", "category", "tier", "rel_path"}
    missing = sorted(required - fields)
    if missing:
        raise ValueError(f"LAM manifest is missing fields: {missing}")
    if strict_counts and len(rows) != EXPECTED_MODEL_COUNT:
        raise ValueError(f"expected {EXPECTED_MODEL_COUNT} LAM rows, found {len(rows)}")

    candidates: list[Candidate] = []
    for row in rows:
        tier = str(row.get("tier") or "").strip()
        if tier not in EXPECTED_TIERS:
            continue
        asset_id = _safe_component(row.get("object_release_id"), field="object_release_id")
        category = _safe_component(row.get("category"), field=f"category for {asset_id}")
        rel_path = _safe_relative(row.get("rel_path"), field=f"rel_path for {asset_id}")
        if rel_path.split("/", 1)[0] != "objects":
            raise ValueError(f"LAM rel_path must be under objects/: {rel_path}")
        identity = f"{tier}:{rel_path}"
        candidates.append(
            Candidate(
                category=category,
                asset_id=asset_id,
                tier=tier,
                rel_path=rel_path,
                identity=identity,
                identity_sha256=BASE._identity_sha256(identity),
                selection_sha256=BASE._selection_sha256(rel_path),
            )
        )
    if len({item.asset_id for item in candidates}) != len(candidates):
        raise ValueError("LAM candidate asset IDs are not unique")
    categories = {item.category for item in candidates}
    if strict_counts and len(candidates) != EXPECTED_CANDIDATE_COUNT:
        raise ValueError(f"expected {EXPECTED_CANDIDATE_COUNT} candidates, found {len(candidates)}")
    if strict_counts and len(categories) != EXPECTED_CATEGORY_COUNT:
        raise ValueError(f"expected {EXPECTED_CATEGORY_COUNT} categories, found {len(categories)}")
    metadata = {
        "path": str(manifest),
        "sha256": _sha256(manifest),
        "row_count": len(rows),
        "candidate_count": len(candidates),
        "category_count": len(categories),
    }
    return metadata, tuple(candidates)


def select_samples(
    candidates: Sequence[Candidate],
    *,
    output_root: Path,
    source_root: Path,
    samples_per_category: int = 5,
    balanced_only: bool = False,
    validate_inputs: bool = False,
) -> tuple[tuple[Sampled, ...], dict[str, Any]]:
    """Select deterministic, distinct assets and construct base-driver items."""

    if samples_per_category < 1:
        raise ValueError("samples_per_category must be positive")
    grouped: dict[str, list[Candidate]] = defaultdict(list)
    for candidate in candidates:
        grouped[candidate.category].append(candidate)

    selected: list[Sampled] = []
    short_categories: list[dict[str, Any]] = []
    ordinal = 0
    for class_number, category in enumerate(sorted(grouped), 1):
        # Keep viable first (the existing one-shot winner is therefore S01),
        # then use loads_only only when needed to reach the cap.
        ordered = sorted(
            grouped[category],
            key=lambda item: (
                0 if item.tier == "viable" else 1,
                item.selection_sha256,
                item.rel_path,
                item.asset_id,
            ),
        )
        support = len(ordered)
        count = min(samples_per_category, support)
        eligible = support >= samples_per_category
        if not eligible:
            short_categories.append(
                {
                    "class_id": f"L{class_number:04d}",
                    "category": category,
                    "support": support,
                    "selected": count,
                }
            )
        if balanced_only and not eligible:
            continue
        for sample_index, candidate in enumerate(ordered[:count], 1):
            ordinal += 1
            source = source_root / candidate.rel_path
            urdf = source / "generated.urdf"
            output = output_root / category / candidate.asset_id / "imgs" / "000.png"
            if validate_inputs:
                source = source.resolve(strict=True)
                urdf = urdf.resolve(strict=True)
                if not source.is_dir() or not urdf.is_file():
                    raise ValueError(f"selected LAM package is unavailable: {candidate.rel_path}")
                package_count, package_total, package_hash, binding_hash = BASE._package_receipt(source)
                urdf_bytes = urdf.stat().st_size
                urdf_hash = _sha256(urdf)
            else:
                package_count = package_total = urdf_bytes = 0
                package_hash = binding_hash = urdf_hash = ""
            item = BASE.RenderItem(
                ordinal=ordinal,
                category=category,
                asset_id=candidate.asset_id,
                tier=candidate.tier,
                source_path=source,
                source_relative_path=candidate.rel_path,
                package_path=candidate.rel_path,
                urdf_path=urdf,
                urdf_relative_path=f"{candidate.rel_path}/generated.urdf",
                urdf_bytes=urdf_bytes,
                urdf_sha256=urdf_hash,
                package_file_count=package_count,
                package_total_bytes=package_total,
                package_content_manifest_sha256=package_hash,
                package_binding_sha256=binding_hash,
                selection_sha256=candidate.selection_sha256,
                identity=candidate.identity,
                identity_sha256=candidate.identity_sha256,
                category_one_shot=sample_index == 1,
                output_path=output,
            )
            class_id = f"L{class_number:04d}"
            selected.append(
                Sampled(
                    candidate=candidate,
                    class_id=class_id,
                    sample_index=sample_index,
                    ordinal=ordinal,
                    render_key=f"{class_id}__S{sample_index:02d}__{candidate.asset_id}",
                    support=support,
                    balanced_n5_eligible=eligible,
                    item=item,
                )
            )
    # ``support`` is the full category population; the published panel's
    # per-class cardinality is the number actually selected (capped at five).
    counts = Counter(item.candidate.category for item in selected)
    stats = {
        "class_count": len(grouped) if not balanced_only else sum(
            len(group) >= samples_per_category for group in grouped.values()
        ),
        "asset_count": len(selected),
        "samples_per_category_target": samples_per_category,
        "per_class_count_values": sorted(set(counts.values())) if selected else [],
        "balanced_n5_eligible": sum(len(group) >= samples_per_category for group in grouped.values()),
        "short_category_count": len(short_categories),
        "short_categories": short_categories,
        "balanced_only": balanced_only,
    }
    return tuple(selected), stats


def _item_row(sample: Sampled) -> dict[str, Any]:
    item = sample.item
    return {
        "ordinal": sample.ordinal,
        "render_key": sample.render_key,
        "generator_index": sample.class_id,
        "generator_name": sample.candidate.category,
        "class_id": sample.class_id,
        "category": sample.candidate.category,
        "sample_index": sample.sample_index,
        "category_support": sample.support,
        "balanced_n5_eligible": sample.balanced_n5_eligible,
        "source_type": "lam",
        "asset_id": sample.candidate.asset_id,
        "tier": sample.candidate.tier,
        "identity": sample.candidate.identity,
        "selection_sha256": sample.candidate.selection_sha256,
        "identity_sha256": sample.candidate.identity_sha256,
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
        "category_one_shot": item.category_one_shot,
        "output_path": str(item.output_path),
        "png_bytes": "",
        "png_sha256": "",
    }


def _write_csv(path: Path, fields: Sequence[str], rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
            writer.writeheader()
            for row in rows:
                normalized: dict[str, Any] = {}
                for field in fields:
                    value = row.get(field, "")
                    if isinstance(value, (dict, list, tuple)):
                        value = json.dumps(value, sort_keys=True, ensure_ascii=True)
                    normalized[field] = value
                writer.writerow(normalized)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True, ensure_ascii=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _valid_png(path: Path, resolution: int) -> bool:
    try:
        if not path.is_file() or path.stat().st_size <= len(PNG_SIGNATURE):
            return False
        with path.open("rb") as stream:
            if stream.read(len(PNG_SIGNATURE)) != PNG_SIGNATURE:
                return False
        from PIL import Image

        with Image.open(path) as image:
            image.load()
            return image.size == (resolution, resolution) and image.mode in {"RGB", "RGBA"}
    except (OSError, ValueError):
        return False


def _read_reuse_manifest(root: Path) -> tuple[Path | None, dict[tuple[str, str], dict[str, Any]]]:
    manifest = root / "render_manifest.csv"
    if not manifest.is_file():
        return None, {}
    with manifest.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    indexed: dict[tuple[str, str], dict[str, Any]] = {}
    for row in rows:
        key = (str(row.get("category") or ""), str(row.get("asset_id") or ""))
        if key[0] and key[1]:
            indexed[key] = row
    return manifest, indexed


def _renderer_result(value: object) -> dict[str, Any] | None:
    if isinstance(value, Mapping):
        return dict(value)
    if isinstance(value, str) and value.strip():
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError:
            # Older interrupted runs wrote ``str(dict)`` into CSV.  Accept
            # that legacy form only through the safe literal parser so a
            # resume can repair the manifest without rerendering images.
            try:
                parsed = ast.literal_eval(value)
            except (ValueError, SyntaxError):
                return None
        return dict(parsed) if isinstance(parsed, Mapping) else None
    return None


def _external_reuse(
    sample: Sampled,
    *,
    reuse_root: Path,
    reuse_manifest: Path | None,
    baseline: Mapping[tuple[str, str], Mapping[str, Any]],
    resolution: int,
    force: bool,
) -> dict[str, Any] | None:
    """Hard-link a verified one-shot image into the new roster."""

    if force or not sample.item.category_one_shot or reuse_manifest is None:
        return None
    old = baseline.get((sample.candidate.category, sample.candidate.asset_id))
    if not old or str(old.get("status") or "") not in SUCCESS_STATUSES:
        return None
    old_source = Path(str(old.get("output_path") or old.get("image") or ""))
    if not old_source.is_absolute():
        old_source = reuse_root / old_source
    old_source = old_source.expanduser()
    if not _valid_png(old_source, resolution):
        return None
    declared_hash = str(old.get("png_sha256") or old.get("image_sha256") or "")
    try:
        declared_bytes = int(old.get("png_bytes") or old.get("image_bytes") or -1)
    except (TypeError, ValueError):
        return None
    if declared_bytes != old_source.stat().st_size or not declared_hash or _sha256(old_source) != declared_hash:
        return None
    # Bind reuse to the same package identity and frozen renderer when fields exist.
    for field in ("category", "asset_id", "identity_sha256", "package_binding_sha256"):
        observed = str(old.get(field) or "")
        expected = (
            sample.candidate.category
            if field == "category"
            else sample.candidate.asset_id
            if field == "asset_id"
            else sample.candidate.identity_sha256
            if field == "identity_sha256"
            else sample.item.package_binding_sha256
        )
        if observed and expected and observed != expected:
            return None

    target = sample.item.output_path
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() or target.is_symlink():
        if not _valid_png(target, resolution) or _sha256(target) != declared_hash:
            return None
    else:
        temporary = target.with_name(f".{target.name}.{os.getpid()}.{time.time_ns()}.tmp")
        try:
            try:
                os.link(old_source, temporary)
            except OSError:
                shutil.copy2(old_source, temporary)
            os.replace(temporary, target)
        finally:
            temporary.unlink(missing_ok=True)
    result = _renderer_result(old.get("renderer_result")) or {}
    result["output"] = str(target)
    result["reuse_source_output"] = str(old_source)
    return {
        "status": "reused_valid",
        "png_bytes": target.stat().st_size,
        "png_sha256": _sha256(target),
        "reuse_source_path": str(old_source),
        "reuse_source_manifest": str(reuse_manifest),
        "renderer_result": result,
        "error": "",
    }


def _target_receipt(sample: Sampled, prior: Mapping[str, Any] | None, resolution: int) -> dict[str, Any] | None:
    if not prior or str(prior.get("status") or "") not in SUCCESS_STATUSES:
        return None
    if str(prior.get("render_key") or "") != sample.render_key:
        return None
    target = sample.item.output_path
    if not _valid_png(target, resolution):
        return None
    declared = str(prior.get("png_sha256") or "")
    try:
        size = int(prior.get("png_bytes") or -1)
    except (TypeError, ValueError):
        return None
    if size != target.stat().st_size or not declared or _sha256(target) != declared:
        return None
    return {
        "status": "reused_valid",
        "png_bytes": size,
        "png_sha256": declared,
        "reuse_source_path": str(target),
        "reuse_source_manifest": "render_manifest.csv",
        "renderer_result": _renderer_result(prior.get("renderer_result")) or {},
        "error": "",
    }


def _base_args(args: argparse.Namespace) -> argparse.Namespace:
    # The audited base worker consumes this namespace through _render_one.
    return argparse.Namespace(
        output_root=args.output_root,
        resolution=args.resolution,
        samples=args.samples,
        gpu=args.gpu,
        timeout_seconds=args.timeout_seconds,
        force=True,
    )


def _render_sample(
    sample: Sampled,
    *,
    args: argparse.Namespace,
    blender: Path,
    renderer: Path,
    base_renderer: Path,
    shared_renderer: Path,
    base_args: argparse.Namespace,
) -> dict[str, Any]:
    started = _utc_now()
    start = time.monotonic()
    try:
        result = BASE._render_one(
            sample.item,
            args=base_args,
            blender=blender,
            renderer=renderer,
            base_renderer=base_renderer,
            base_renderer_sha256=_sha256(base_renderer),
            shared_renderer=shared_renderer,
            shared_renderer_sha256=_sha256(shared_renderer),
            reuse_receipt=None,
        )
        status = str(result.get("status") or "failed")
        error = str(result.get("error") or "")
        png_bytes = int(result.get("png_bytes") or 0)
        png_hash = str(result.get("png_sha256") or "")
        renderer_result = result.get("renderer_result")
    except Exception as exc:  # keep one failed asset from losing the checkpoint
        status, error, png_bytes, png_hash, renderer_result = "failed", repr(exc), 0, "", None
    return {
        **_item_row(sample),
        "status": status,
        "elapsed_seconds": round(time.monotonic() - start, 3),
        "started_at": started,
        "finished_at": _utc_now(),
        "error": error,
        "gpu": str(args.gpu),
        "reuse_source_path": "",
        "reuse_source_manifest": "",
        "renderer_result": renderer_result,
        "png_bytes": png_bytes,
        "png_sha256": png_hash,
    }


def _manifest_row(sample: Sampled, result: Mapping[str, Any]) -> dict[str, Any]:
    row = _item_row(sample)
    row.update(result)
    if row.get("renderer_result") is not None and not isinstance(row["renderer_result"], str):
        row["renderer_result"] = json.dumps(row["renderer_result"], sort_keys=True, ensure_ascii=True)
    return row


def _selection_receipt(samples: Sequence[Sampled]) -> dict[str, Any]:
    rows = [
        {
            "category": sample.candidate.category,
            "asset_id": sample.candidate.asset_id,
            "tier": sample.candidate.tier,
            "sample_index": sample.sample_index,
            "identity": sample.candidate.identity,
            "identity_sha256": sample.candidate.identity_sha256,
        }
        for sample in samples
    ]
    return {
        "schema_version": 1,
        "count": len(rows),
        "ordering": "ordinal (category, tier priority, SHA256(rel_path), rel_path)",
        "identity_category_sha256": _canonical_sha256(rows),
    }


def _build_config(
    args: argparse.Namespace,
    *,
    metadata: Mapping[str, Any],
    candidates: Sequence[Candidate],
    samples: Sequence[Sampled],
    stats: Mapping[str, Any],
    renderer: Path,
    base_renderer: Path,
    shared_renderer: Path,
    blender: Path,
    reuse_manifest: Path | None,
) -> dict[str, Any]:
    categories = {item.category for item in candidates}
    return {
        "schema_version": 1,
        "render_contract": "lam_articulated_object_code_uniform_studio_n5_v1",
        "sampling_contract": "lam_category_viable_first_hash_rank_fill_min5_v1",
        "dataset": "LAM",
        "dataset_alias": "Articulated-Object-Code",
        "dataset_manifest": str(args.dataset_manifest.expanduser().resolve(strict=True)),
        "dataset_manifest_sha256": metadata["sha256"],
        "official_model_count": EXPECTED_MODEL_COUNT,
        "candidate_model_count": len(candidates),
        "category_count": len(categories),
        "selected_count": len(samples),
        "selected_category_count": len({sample.candidate.category for sample in samples}),
        "per_class_count_values": stats["per_class_count_values"],
        "balanced_n5_eligible": stats["balanced_n5_eligible"],
        "short_category_count": stats["short_category_count"],
        "short_categories": stats["short_categories"],
        "samples_per_category_target": args.samples_per_category,
        "balanced_only": bool(args.balanced_only),
        "selection": _selection_receipt(samples),
        "source_root": str(args.source_root.expanduser().resolve()),
        "output_root": str(args.output_root.expanduser().resolve()),
        "reuse_root": str(args.reuse_root.expanduser().resolve()),
        "reuse_manifest": str(reuse_manifest) if reuse_manifest else "",
        "driver": str(SCRIPT),
        "driver_sha256": _sha256(SCRIPT),
        "renderer": str(renderer),
        "renderer_sha256": _sha256(renderer),
        "base_renderer": str(base_renderer),
        "base_renderer_sha256": _sha256(base_renderer),
        "shared_renderer": str(shared_renderer),
        "shared_renderer_sha256": _sha256(shared_renderer),
        "blender": str(blender),
        "blender_version": BASE._blender_version(blender),
        "resolution": args.resolution,
        "samples": args.samples,
        "pose_policy": "URDF rest pose; all movable joint coordinates are zero",
        "material_policy": BASE.MATERIAL_POLICY,
        "image_layout": "category/asset_id/imgs/000.png",
        "studio": BASE._studio_contract(),
        "gpu_visibility": str(args.gpu),
        "workers": args.workers,
        "timeout_seconds": args.timeout_seconds,
    }


def _stable_config(config: Mapping[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in config.items()
        if key
        not in {
            "workers",
            "timeout_seconds",
            # These are populated after the roster is finalized and must not
            # make a resumable run look like a different sampling contract.
            "render_roster_sha256",
            "render_roster_rows",
        }
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.resolution < 64 or args.samples < 1 or args.workers < 1 or args.samples_per_category < 1:
        raise ValueError("resolution >=64, samples/workers/samples_per_category >=1 required")
    args.output_root = args.output_root.expanduser().resolve()
    args.dataset_manifest = args.dataset_manifest.expanduser().resolve(strict=True)
    args.source_root = args.source_root.expanduser().resolve()
    args.reuse_root = args.reuse_root.expanduser().resolve()
    renderer = args.renderer.expanduser().resolve(strict=True)
    base_renderer = args.base_renderer.expanduser().resolve(strict=True)
    shared_renderer = args.shared_renderer.expanduser().resolve(strict=True)
    blender = args.blender.expanduser().resolve(strict=True)

    metadata, candidates = load_candidates(
        args.dataset_manifest,
        strict_counts=not args.allow_count_drift,
    )
    # Build paths and package receipts only for the selected panel.  This is
    # substantially cheaper than resolving all 2,832 candidates up front.
    selected, stats = select_samples(
        candidates,
        output_root=args.output_root,
        source_root=args.source_root,
        samples_per_category=args.samples_per_category,
        balanced_only=args.balanced_only,
        validate_inputs=not args.dry_run,
    )
    if not selected:
        raise ValueError("selection contains no LAM assets")
    reuse_manifest, baseline = _read_reuse_manifest(args.reuse_root)
    config = _build_config(
        args,
        metadata=metadata,
        candidates=candidates,
        samples=selected,
        stats=stats,
        renderer=renderer,
        base_renderer=base_renderer,
        shared_renderer=shared_renderer,
        blender=blender,
        reuse_manifest=reuse_manifest,
    )
    if args.dry_run:
        return {
            "status": "dry_run",
            "config": config,
            "selection": {
                "class_count": stats["class_count"],
                "selected_count": len(selected),
                "balanced_n5_eligible": stats["balanced_n5_eligible"],
                "per_class_count_values": stats["per_class_count_values"],
            },
        }

    args.output_root.mkdir(parents=True, exist_ok=True)
    config_path = args.output_root / "render_config.json"
    roster_path = args.output_root / "render_roster.csv"
    manifest_path = args.output_root / "render_manifest.csv"
    state_path = args.output_root / "render_state.jsonl"
    roster_rows = [_item_row(sample) for sample in selected]
    roster_bytes = io.StringIO(newline="")
    writer = csv.DictWriter(roster_bytes, fieldnames=ROSTER_FIELDS)
    writer.writeheader()
    for row in roster_rows:
        writer.writerow({field: row.get(field, "") for field in ROSTER_FIELDS})
    roster_payload = roster_bytes.getvalue()
    roster_hash = hashlib.sha256(roster_payload.encode("utf-8")).hexdigest()
    config["render_roster_sha256"] = roster_hash
    config["render_roster_rows"] = len(roster_rows)
    if config_path.is_file():
        previous = json.loads(config_path.read_text(encoding="utf-8"))
        if _stable_config(previous) != _stable_config(config):
            raise ValueError(f"output root contains a different render contract: {config_path}")
    elif any(args.output_root.iterdir()):
        raise ValueError(f"non-empty output root has no render_config.json: {args.output_root}")
    else:
        _write_json(config_path, config)
    if not roster_path.is_file():
        roster_path.write_text(roster_payload, encoding="utf-8")

    # Recover the last complete receipt for each render key.
    prior: dict[str, dict[str, Any]] = {}
    if manifest_path.is_file():
        with manifest_path.open("r", encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                prior[str(row.get("render_key") or "")] = row
    latest = dict(prior)
    base_args = _base_args(args)
    results: list[dict[str, Any]] = []
    pending: list[Sampled] = []
    for sample in selected:
        recovered = None if args.force else _target_receipt(sample, prior.get(sample.render_key), args.resolution)
        if recovered is None:
            recovered = _external_reuse(
                sample,
                reuse_root=args.reuse_root,
                reuse_manifest=reuse_manifest,
                baseline=baseline,
                resolution=args.resolution,
                force=args.force,
            )
        if recovered is not None:
            result = {**_item_row(sample), **recovered, "elapsed_seconds": 0.0, "started_at": _utc_now(), "finished_at": _utc_now(), "gpu": str(args.gpu)}
            results.append(result)
            latest[sample.render_key] = result
        else:
            pending.append(sample)

    # Render only assets not recovered from a verified receipt.
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {
            pool.submit(
                _render_sample,
                sample,
                args=args,
                blender=blender,
                renderer=renderer,
                base_renderer=base_renderer,
                shared_renderer=shared_renderer,
                base_args=base_args,
            ): sample
            for sample in pending
        }
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            latest[futures[future].render_key] = result
            print(
                f"[render] {len(results)}/{len(selected)} {result['category']}/"
                f"{result['asset_id']} {result['status']} ({result['elapsed_seconds']:.1f}s)",
                flush=True,
            )

    ordered = sorted(latest.values(), key=lambda row: int(row.get("ordinal", 0)))
    _write_csv(manifest_path, MANIFEST_FIELDS, ordered)
    # Refresh the feature-loader-facing roster with final PNG receipts.  The
    # initial roster is written before rendering so interrupted runs still
    # have a complete immutable selection index.
    final_roster = []
    for sample in selected:
        result = latest.get(sample.render_key)
        row = _item_row(sample)
        if result is not None:
            row["png_bytes"] = result.get("png_bytes", "")
            row["png_sha256"] = result.get("png_sha256", "")
        final_roster.append(row)
    _write_csv(roster_path, ROSTER_FIELDS, final_roster)
    config["render_roster_sha256"] = _sha256(roster_path)
    config["render_roster_rows"] = len(final_roster)
    _write_json(config_path, config)
    failures = [row for row in ordered if str(row.get("status")) not in SUCCESS_STATUSES]
    valid_count = sum(_valid_png(Path(str(sample.item.output_path)), args.resolution) for sample in selected)
    summary = {
        "schema_version": 1,
        "render_contract": config["render_contract"],
        "dataset": "LAM",
        "started_at": _utc_now(),
        "finished_at": _utc_now(),
        "class_count": stats["class_count"],
        "selected_count": len(selected),
        "selected_category_count": len({sample.candidate.category for sample in selected}),
        "balanced_n5_eligible": stats["balanced_n5_eligible"],
        "short_category_count": stats["short_category_count"],
        "per_class_count_values": stats["per_class_count_values"],
        "rendered_count": sum(row.get("status") == "rendered" for row in ordered),
        "reused_valid_count": sum(row.get("status") == "reused_valid" for row in ordered),
        "failure_count": len(failures),
        "selected_valid_png_count": valid_count,
        "selected_complete": not failures and valid_count == len(selected),
        "failure_render_keys": [str(row.get("render_key")) for row in failures],
        "manifest": str(manifest_path),
        "roster": str(roster_path),
        "config": str(config_path),
    }
    _write_json(args.output_root / "render_summary.json", summary)
    receipt = {
        "schema_version": 1,
        "dataset": "LAM",
        "render_contract": config["render_contract"],
        "selected_count": len(selected),
        "class_count": stats["class_count"],
        "balanced_n5_eligible": stats["balanced_n5_eligible"],
        "render_roster_sha256": _sha256(roster_path),
        "render_manifest_sha256": _sha256(manifest_path),
        "render_config_sha256": _sha256(config_path),
        "render_summary_sha256": _sha256(args.output_root / "render_summary.json"),
        "selected_complete": summary["selected_complete"],
    }
    _write_json(args.output_root / "release_receipt.json", receipt)
    if failures:
        raise RuntimeError(f"{len(failures)} LAM render(s) failed; rerun to resume")
    return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-manifest", type=Path, default=DEFAULT_DATASET_MANIFEST)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--reuse-root", type=Path, default=DEFAULT_REUSE_ROOT)
    parser.add_argument("--renderer", type=Path, default=DEFAULT_RENDERER)
    parser.add_argument("--base-renderer", type=Path, default=DEFAULT_BASE_RENDERER)
    parser.add_argument("--shared-renderer", type=Path, default=DEFAULT_SHARED_RENDERER)
    parser.add_argument("--blender", type=Path, default=DEFAULT_BLENDER)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--samples-per-category", type=int, default=5)
    parser.add_argument("--gpu", default="6")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--timeout-seconds", type=float, default=900.0)
    parser.add_argument("--balanced-only", action="store_true")
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
