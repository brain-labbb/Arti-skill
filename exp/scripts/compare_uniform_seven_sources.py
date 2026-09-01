#!/usr/bin/env python3
"""Compare seven class-level datasets under one frozen Blender studio.

This driver extends the audited four-source comparison with Infinigen-Sim,
Articulated-Object-Code, and PhysX-Mobility.  Every added dataset is represented
by one frozen render per exact category.  DINOv2 and CLIP are extracted with the
same model snapshots and preprocessing as the existing comparison; t-SNE is
used only for visualization and all source-gap numbers are computed in the
original normalized feature spaces.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import itertools
import json
import math
import os
import re
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from PIL import Image, ImageStat

for _thread_variable in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_thread_variable, "1")


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
FOUR_SOURCE_SCRIPT = REPO_ROOT / "exp/scripts/compare_pva_artiverse_articraft_partnet_uniform.py"
DEFAULT_PVA_DIR = REPO_ROOT / "exp/runtime/pva531_uniform_tsne"
DEFAULT_ARTIVERSE_DIR = REPO_ROOT / "exp/runtime/pva_artiverse_uniform_one_shot_comparison"
DEFAULT_ARTIVERSE_RENDER_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/Artiverse/renders/"
    "uniform84_one_per_category_studio_256_v1"
)
DEFAULT_ARTICRAFT_RENDER_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/Articraft-10K/renders/"
    "uniform244_one_per_category_studio_256_v1"
)
DEFAULT_PARTNET_RENDER_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/PartNet-Mobility/renders/"
    "uniform46_one_per_category_studio_256_v1"
)
DEFAULT_INFINIGEN_RENDER_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/Infinigen-Sim/renders/"
    "uniform17_one_per_category_studio_256_v1"
)
DEFAULT_AOC_RENDER_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/Articulated-Object-Code/renders/"
    "uniform660_one_per_category_studio_256_v1"
)
DEFAULT_PHYSX_RENDER_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/PhysX-Mobility/renders/"
    "uniform132_one_per_category_studio_256_v1"
)
DEFAULT_PRIOR_COMPARISON_DIR = REPO_ROOT / "exp/runtime/pva_artiverse_articraft_partnet_uniform_comparison"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exp/runtime/pva_artiverse_articraft_partnet_infinigen_aoc_physx_uniform_comparison"

ENCODERS = ("dinov2", "clip")
SOURCE_ORDER = (
    "PV-A",
    "Artiverse",
    "Articraft-10K",
    "PartNet-Mobility",
    "Infinigen-Sim",
    "Articulated-Object-Code",
    "PhysX-Mobility",
)
SOURCE_KEYS = {
    "PV-A": "pva",
    "Artiverse": "artiverse",
    "Articraft-10K": "articraft10k",
    "PartNet-Mobility": "partnet_mobility",
    "Infinigen-Sim": "infinigen_sim",
    "Articulated-Object-Code": "articulated_object_code",
    "PhysX-Mobility": "physx_mobility",
}
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True, slots=True)
class GenericRecord:
    ordinal: int
    dataset_id: str
    category: str
    image_path: Path
    image_bytes: int
    image_sha256: str
    source_path: Path
    urdf_path: Path | None
    origin: str = ""


@dataclass(frozen=True, slots=True)
class GenericBundle:
    name: str
    key: str
    root: Path
    records: tuple[GenericRecord, ...]
    categories: tuple[str, ...]
    config: dict[str, Any]
    receipts: dict[str, Any]
    official_model_count: int
    official_category_count: int


_FOUR: Any | None = None


def _four() -> Any:
    global _FOUR
    if _FOUR is None:
        name = "_audited_four_source_comparison_for_seven_source"
        spec = importlib.util.spec_from_file_location(name, FOUR_SOURCE_SCRIPT)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot load comparison helper: {FOUR_SOURCE_SCRIPT}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        _FOUR = module
    return _FOUR


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_hash(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def file_receipt(path: Path) -> dict[str, Any]:
    path = path.resolve(strict=True)
    return {"path": str(path), "sha256": sha256_file(path), "size_bytes": path.stat().st_size}


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def atomic_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise ValueError(f"cannot write empty CSV: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def atomic_npy(path: Path, matrix: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("wb") as stream:
        np.save(stream, np.asarray(matrix), allow_pickle=False)
    temporary.replace(path)


def _resolve_path(value: Any, *, parent: Path, field: str, strict: bool = True) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"missing path field {field}")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = parent / path
    return path.resolve(strict=strict)


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"CSV is empty: {path}")
    return rows


def _row_value(row: Mapping[str, Any], *names: str) -> str:
    for name in names:
        value = row.get(name)
        if value is not None and str(value).strip():
            return str(value).strip()
    return ""


def _parse_int(row: Mapping[str, Any], names: Sequence[str], *, default: int = -1) -> int:
    raw = _row_value(row, *names)
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(f"invalid integer {names}: {raw!r}") from exc


def _load_matrix(path: Path, *, rows: int, dimension: int) -> np.ndarray:
    matrix = np.load(path, allow_pickle=False)
    if matrix.ndim != 2 or matrix.shape != (rows, dimension) or matrix.dtype != np.float32:
        raise ValueError(f"unexpected feature matrix {path}: {matrix.shape}/{matrix.dtype}")
    if not np.isfinite(matrix).all():
        raise ValueError(f"non-finite feature matrix: {path}")
    if not np.allclose(np.linalg.norm(matrix, axis=1), 1.0, rtol=2e-5, atol=2e-5):
        raise ValueError(f"feature matrix is not L2 normalized: {path}")
    return np.asarray(matrix, dtype=np.float32)


def _validate_png(path: Path, *, resolution: int) -> tuple[int, str]:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"render image is not a regular file: {path}")
    payload = path.read_bytes()
    if payload[: len(PNG_SIGNATURE)] != PNG_SIGNATURE:
        raise ValueError(f"render image is not PNG: {path}")
    with Image.open(path) as image:
        image.load()
        if image.size != (resolution, resolution) or image.mode not in {"RGB", "RGBA"}:
            raise ValueError(f"render image contract mismatch: {path}")
        if ImageStat.Stat(image.convert("L")).stddev[0] <= 1.0:
            raise ValueError(f"render image is blank: {path}")
    return len(payload), hashlib.sha256(payload).hexdigest()


def _check_support_receipts(config: Mapping[str, Any], root: Path, pva_config: Mapping[str, Any]) -> None:
    fields = ["driver", "renderer", "base_renderer", "shared_renderer"]
    if config.get("support_renderer") is not None:
        fields.append("support_renderer")
    for field in fields:
        raw = config.get(field)
        if not isinstance(raw, str) or not raw.strip():
            raise ValueError(f"{config.get('dataset')} config lacks {field}")
        path = _resolve_path(raw, parent=root, field=field)
        declared = str(config.get(f"{field}_sha256") or "")
        if SHA256_RE.fullmatch(declared) is None or sha256_file(path) != declared:
            raise ValueError(f"{config.get('dataset')} {field} SHA receipt mismatch")
    # The PV-A render config calls the shared studio implementation `renderer`;
    # added runners call it `shared_renderer`.
    pva_shared = pva_config.get("shared_renderer_sha256") or pva_config.get("renderer_sha256")
    if config.get("shared_renderer_sha256") != pva_shared:
        raise ValueError(f"{config.get('dataset')} did not use the PV-A studio renderer")


def load_uniform_bundle(
    root_arg: Path,
    *,
    name: str,
    key: str,
    pva_config: Mapping[str, Any],
    expected_count: int | None = None,
    expected_categories: int | None = None,
    strict_counts: bool = True,
) -> GenericBundle:
    root = root_arg.expanduser().resolve(strict=True)
    if not root.is_dir() or root.is_symlink():
        raise ValueError(f"uniform root must be a regular directory: {root}")
    config_path = root / "render_config.json"
    manifest_path = root / "render_manifest.csv"
    roster_path = root / "category_one_shot_roster.csv"
    for path in (config_path, manifest_path, roster_path):
        if path.is_symlink() or not path.is_file():
            raise FileNotFoundError(f"incomplete uniform root: {path}")
    config = load_json(config_path)
    if "uniform" not in str(config.get("render_contract") or "").lower():
        raise ValueError(f"unexpected render contract for {name}")
    if str(config.get("dataset") or "") not in {name, name.replace("-", "_")}: 
        raise ValueError(f"dataset identity mismatch for {name}: {config.get('dataset')!r}")
    for field in ("resolution", "samples", "studio", "blender_version"):
        if config.get(field) != pva_config.get(field):
            raise ValueError(f"{name}/PV-A studio mismatch for {field}")
    _check_support_receipts(config, root, pva_config)
    configured_output = _resolve_path(config.get("output_root"), parent=root, field="output_root")
    if configured_output != root:
        raise ValueError(f"{name} config output root mismatch")

    rows = _read_csv(manifest_path)
    records: list[GenericRecord] = []
    seen_ids: set[str] = set()
    seen_categories: set[str] = set()
    resolution = int(config.get("resolution", 256))
    for row in rows:
        category = _row_value(row, "category", "model_cat", "raw_category")
        dataset_id = _row_value(row, "dataset_id", "asset_id", "object_release_id", "identity")
        if not category or not dataset_id or dataset_id in seen_ids:
            raise ValueError(f"{name} manifest has an empty/duplicate identity")
        seen_ids.add(dataset_id)
        seen_categories.add(category)
        ordinal = _parse_int(row, ("ordinal", "selection_rank", "release_order"))
        image_raw = _row_value(row, "image_path", "output_path", "png_path", "output")
        image_path = _resolve_path(image_raw, parent=root, field="image_path")
        actual_bytes, actual_sha = _validate_png(image_path, resolution=resolution)
        declared_bytes = _parse_int(row, ("image_bytes", "png_bytes", "output_bytes"))
        declared_sha = _row_value(row, "image_sha256", "png_sha256", "output_sha256")
        if declared_bytes != actual_bytes or declared_sha != actual_sha:
            raise ValueError(f"{name} image receipt mismatch: {dataset_id}")
        source_raw = _row_value(row, "source_path", "package_path", "asset_dir")
        source_path = _resolve_path(source_raw, parent=root, field="source_path")
        urdf_raw = _row_value(row, "urdf_path", "primary_urdf_path")
        urdf_path = _resolve_path(urdf_raw, parent=root, field="urdf_path") if urdf_raw else None
        renderer_raw = _row_value(row, "renderer_result")
        if renderer_raw:
            try:
                renderer_result = json.loads(renderer_raw)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{name} renderer_result is invalid: {dataset_id}") from exc
            if not isinstance(renderer_result, dict):
                raise ValueError(f"{name} renderer_result is not an object: {dataset_id}")
            rendered_output = renderer_result.get("output") or renderer_result.get("image_path")
            if str(rendered_output or "") != str(image_path):
                raise ValueError(f"{name} renderer output receipt mismatch: {dataset_id}")
            rendered_source = renderer_result.get("asset_dir") or renderer_result.get("package")
            if rendered_source and Path(str(rendered_source)).expanduser().resolve() != source_path:
                raise ValueError(f"{name} renderer input receipt mismatch: {dataset_id}")
            shared = renderer_result.get("shared_renderer")
            base = renderer_result.get("base_renderer")
            if not isinstance(shared, Mapping) or shared.get("sha256") != config.get("shared_renderer_sha256"):
                raise ValueError(f"{name} shared renderer receipt mismatch: {dataset_id}")
            if not isinstance(base, Mapping) or base.get("sha256") != config.get("base_renderer_sha256"):
                raise ValueError(f"{name} base renderer receipt mismatch: {dataset_id}")
        records.append(
            GenericRecord(
                ordinal=ordinal,
                dataset_id=dataset_id,
                category=category,
                image_path=image_path,
                image_bytes=actual_bytes,
                image_sha256=actual_sha,
                source_path=source_path,
                urdf_path=urdf_path,
                origin=_row_value(row, "origin", "cohort_origin", "tier"),
            )
        )
    records.sort(key=lambda record: (record.ordinal, record.category, record.dataset_id))
    if [record.ordinal for record in records] != list(range(len(records))):
        raise ValueError(f"{name} ordinals are not contiguous")
    if len({record.category for record in records}) != len(records):
        raise ValueError(f"{name} does not contain exactly one image per category")
    if expected_count is not None and strict_counts and len(records) != expected_count:
        raise ValueError(f"{name} expected {expected_count} records, found {len(records)}")
    if expected_categories is not None and strict_counts and len(seen_categories) != expected_categories:
        raise ValueError(f"{name} expected {expected_categories} categories, found {len(seen_categories)}")
    if int(config.get("selected_count", len(records))) != len(records):
        raise ValueError(f"{name} selected count receipt mismatch")
    if int(config.get("selected_category_count", len(records))) != len(records):
        raise ValueError(f"{name} selected category count receipt mismatch")
    roster_rows = _read_csv(roster_path)
    roster_ids = [_row_value(row, "dataset_id", "asset_id", "object_release_id", "identity") for row in roster_rows]
    if roster_ids != [record.dataset_id for record in records]:
        raise ValueError(f"{name} roster does not match render manifest")
    receipts = {
        "render_config": file_receipt(config_path),
        "render_manifest": file_receipt(manifest_path),
        "category_one_shot_roster": file_receipt(roster_path),
    }
    for field in ("dataset_manifest", "release_manifest", "source_manifest", "provenance"):
        raw = config.get(field)
        if isinstance(raw, str) and raw.strip():
            path = _resolve_path(raw, parent=root, field=field)
            declared = config.get(f"{field}_sha256")
            if declared and sha256_file(path) != declared:
                raise ValueError(f"{name} {field} receipt mismatch")
            receipts[field] = file_receipt(path)
    official_models = int(config.get("official_model_count", config.get("model_count", len(records))))
    official_categories = int(config.get("official_category_count", config.get("category_count", len(records))))
    return GenericBundle(
        name=name,
        key=key,
        root=root,
        records=tuple(records),
        categories=tuple(record.category for record in records),
        config=config,
        receipts=receipts,
        official_model_count=official_models,
        official_category_count=official_categories,
    )


def _record_from_existing(
    record: Any,
    *,
    source: str,
    ordinal: int,
    image_override: Path | None = None,
) -> GenericRecord:
    image_value = image_override or getattr(record, "image_path", None)
    if image_value is None:
        raise ValueError(f"{source} record {ordinal} has no image binding")
    image_path = Path(image_value).expanduser().resolve(strict=True)
    source_value = getattr(record, "package_path", None)
    if source_value is None and getattr(record, "glb_path", None) is not None:
        source_value = Path(record.glb_path).parent
    if source_value is None:
        candidate = getattr(record, "source", None)
        if candidate is not None and Path(str(candidate)).expanduser().exists():
            source_value = candidate
    source_path = Path(source_value or image_path.parent).expanduser().resolve(strict=True)
    dataset_id = str(
        getattr(
            record,
            "dataset_id",
            getattr(
                record,
                "asset_id",
                getattr(record, "model_id", getattr(record, "generator_index", ordinal)),
            ),
        )
    )
    category = str(
        getattr(record, "category", getattr(record, "generator_name", f"{source}_{ordinal}"))
    )
    payload = image_path.read_bytes()
    return GenericRecord(
        ordinal=ordinal,
        dataset_id=dataset_id,
        category=category,
        image_path=image_path,
        image_bytes=len(payload),
        image_sha256=hashlib.sha256(payload).hexdigest(),
        source_path=source_path,
        urdf_path=Path(record.urdf_path).resolve(strict=True) if getattr(record, "urdf_path", None) else None,
        origin=str(getattr(record, "cohort_origin", getattr(record, "source_type", ""))),
    )


def _existing_bundle(name: str, key: str, root: Path, records: Sequence[Any], *, official_models: int, official_categories: int, receipts: Mapping[str, Any]) -> GenericBundle:
    image_overrides: list[Path | None] = [None] * len(records)
    if name == "PV-A":
        roster_rows = _read_csv(root / "generator_roster_resolved.csv")
        by_id = {
            row["generator_index"]: _resolve_path(
                row.get("representative_image"), parent=root, field="representative_image"
            )
            for row in roster_rows
        }
        image_overrides = [by_id[str(record.generator_index)] for record in records]
    converted = tuple(
        _record_from_existing(
            record,
            source=name,
            ordinal=index,
            image_override=image_overrides[index],
        )
        for index, record in enumerate(records)
    )
    return GenericBundle(
        name=name,
        key=key,
        root=root.resolve(strict=True),
        records=converted,
        categories=tuple(record.category for record in converted),
        config={},
        receipts=dict(receipts),
        official_model_count=official_models,
        official_category_count=official_categories,
    )


def _load_cached_matrix(path: Path, manifest_path: Path, *, count: int, dimension: int, model_fingerprint: str) -> np.ndarray:
    manifest = load_json(manifest_path)
    if manifest.get("model_fingerprint") != model_fingerprint or int(manifest.get("model_count", -1)) != count:
        raise ValueError(f"cached feature contract mismatch: {manifest_path}")
    return _load_matrix(path, rows=count, dimension=dimension)


def _extract_bundle(bundle: GenericBundle, *, pva: Any, output_dir: Path, force_extract: bool, batch_size: int, device: str, num_workers: int, use_amp: bool) -> dict[str, np.ndarray]:
    three = _four()._three()
    base = three._base()
    helper = base._pva_helper()
    helper_path = Path(base.PVA_HELPER_SCRIPT).resolve(strict=True)
    import torch

    resolved_device = device
    if device == "auto":
        resolved_device = "cuda:0" if torch.cuda.is_available() else "cpu"
    torch_device = torch.device(resolved_device)
    if torch_device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(f"CUDA device requested but unavailable: {resolved_device}")
    policy = {
        "requested_device": device,
        "resolved_device": str(torch_device),
        "requested_use_amp": bool(use_amp),
        "amp_enabled": bool(use_amp and torch_device.type == "cuda"),
        "batch_size": batch_size,
        "num_workers": num_workers,
    }
    ordered_fp = hashlib.sha256(
        "\n".join(
            f"{record.category}\0{record.dataset_id}\0{record.image_bytes}\0{record.image_sha256}"
            for record in bundle.records
        ).encode("utf-8")
    ).hexdigest()
    result: dict[str, np.ndarray] = {}
    for encoder in ENCODERS:
        enc_dir = output_dir / encoder
        enc_dir.mkdir(parents=True, exist_ok=True)
        feature_path = enc_dir / f"{bundle.key}_one_shot_features.npy"
        manifest_path = enc_dir / f"{bundle.key}_feature_manifest.json"
        model_manifest = pva.feature_manifests[encoder]
        model_path = Path(str(model_manifest["model_path"])).resolve(strict=True)
        expected = {
            "schema_version": 1,
            "dataset": bundle.name,
            "sample_mode": "one_representative_per_exact_category",
            "encoder": encoder,
            "model_type": encoder,
            "model_path": str(model_path),
            "model_fingerprint": model_manifest["model_fingerprint"],
            "model_count": len(bundle.records),
            "ordered_image_fingerprint": ordered_fp,
            "render_manifest_sha256": bundle.receipts.get("render_manifest", {}).get("sha256", ""),
            "roster_sha256": bundle.receipts.get("category_one_shot_roster", {}).get("sha256", ""),
            "feature_helper_path": str(helper_path),
            "feature_helper_sha256": sha256_file(helper_path),
            "execution_policy": policy,
            "preprocessing": {"image_processor_use_fast": False, "alpha_composite_background_rgb": [255, 255, 255]},
        }
        matrix: np.ndarray | None = None
        if not force_extract and feature_path.is_file() and manifest_path.is_file():
            try:
                old = load_json(manifest_path)
                if all(old.get(k) == v for k, v in expected.items()):
                    matrix = _load_matrix(feature_path, rows=len(bundle.records), dimension=int(pva.features[encoder].shape[1]))
                    print(f"[features] reused {feature_path}", flush=True)
            except (OSError, ValueError, TypeError):
                matrix = None
        if matrix is None:
            matrix, extraction = helper.extract_image_features(
                tuple(record.image_path for record in bundle.records),
                model_path=model_path,
                batch_size=batch_size,
                device=device,
                num_workers=num_workers,
                use_amp=use_amp,
            )
            matrix = three.l2_normalize(matrix).astype(np.float32, copy=False)
            if extraction.get("model_type") != encoder or extraction.get("device") != policy["resolved_device"]:
                raise ValueError(f"{bundle.name} {encoder} extraction contract mismatch")
            if bool(extraction.get("amp")) != policy["amp_enabled"] or matrix.shape[1] != pva.features[encoder].shape[1]:
                raise ValueError(f"{bundle.name} {encoder} extraction shape/AMP mismatch")
            atomic_npy(feature_path, matrix)
            atomic_json(manifest_path, {**expected, "dtype": str(matrix.dtype), "extraction": extraction})
        result[encoder] = matrix
    return result


def _copy_existing_features(bundle: GenericBundle, *, pva: Any, source_dir: Path, output_dir: Path, source_key: str) -> dict[str, np.ndarray]:
    result: dict[str, np.ndarray] = {}
    for encoder in ENCODERS:
        source_path = source_dir / encoder / f"{source_key}_one_shot_features.npy"
        source_manifest = source_dir / encoder / (
            "feature_manifest.json" if source_key == "artiverse" else f"{source_key}_feature_manifest.json"
        )
        if not source_path.is_file() or not source_manifest.is_file():
            raise FileNotFoundError(f"missing frozen {bundle.name} feature cache: {source_path}")
        matrix = _load_cached_matrix(
            source_path,
            source_manifest,
            count=len(bundle.records),
            dimension=int(pva.features[encoder].shape[1]),
            model_fingerprint=pva.feature_manifests[encoder]["model_fingerprint"],
        )
        result[encoder] = matrix
    return result


def category_colors(count: int) -> tuple[str, ...]:
    # Golden-ratio hue spacing plus a deterministic 24-bit collision guard.
    import colorsys

    colors: list[str] = []
    used: set[str] = set()
    for index in range(count):
        step = 0
        while True:
            hue = (index * 0.6180339887498949 + step * 0.000137) % 1.0
            saturation = 0.58 + 0.18 * (((index + step) % 5) / 4.0)
            value = 0.72 + 0.20 * (((index // 5 + step) % 4) / 3.0)
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            color = "#%02x%02x%02x" % tuple(round(channel * 255) for channel in rgb)
            if color not in used:
                used.add(color)
                colors.append(color)
                break
            step += 1
    return tuple(colors)


def _compute_tsne(features: np.ndarray, *, perplexity: float, seed: int, max_iter: int, jobs: int, verbose: int) -> tuple[np.ndarray, dict[str, Any]]:
    return _four()._compute_tsne(
        features,
        perplexity=min(float(perplexity), max(2.0, (len(features) - 1) / 3.0)),
        random_state=seed,
        max_iter=max_iter,
        n_jobs=jobs,
        verbose=verbose,
    )


def save_independent_plot(coords: np.ndarray, bundle: GenericBundle, path: Path, *, encoder: str, dpi: int) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = category_colors(len(bundle.records))
    figure, axis = plt.subplots(figsize=(16, 12), dpi=dpi)
    axis.scatter(coords[:, 0], coords[:, 1], s=26 if len(bundle.records) > 250 else 52, c=list(colors), alpha=0.86, edgecolors="white", linewidths=0.25, rasterized=True)
    if len(bundle.records) <= 200:
        for index, point in enumerate(coords, start=1):
            axis.annotate(f"{bundle.key[:2].upper()}{index:03d}", point, xytext=(3, 2), textcoords="offset points", fontsize=4.5)
    axis.set_title(f"{bundle.name} uniform {encoder}: one representative per category (n={len(bundle.records)})")
    axis.set_xlabel("t-SNE 1")
    axis.set_ylabel("t-SNE 2")
    axis.grid(color="#d8d8d8", linewidth=0.4, alpha=0.55)
    axis.spines[["top", "right"]].set_visible(False)
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


SOURCE_SPECS = {
    "PV-A": ("#d1495b", "o", 17, 0.62),
    "Artiverse": ("#2878b5", "^", 36, 0.82),
    "Articraft-10K": ("#2a9d5b", "s", 32, 0.78),
    "PartNet-Mobility": ("#8a5aa8", "D", 42, 0.88),
    "Infinigen-Sim": ("#e07a2d", "P", 42, 0.86),
    "Articulated-Object-Code": ("#00a6a6", "X", 42, 0.82),
    "PhysX-Mobility": ("#6f4e37", "v", 42, 0.86),
}


def save_joint_plot(coords: Mapping[str, np.ndarray], bundles: Mapping[str, GenericBundle], path: Path, *, dpi: int, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    total = sum(len(bundles[source].records) for source in SOURCE_ORDER)
    figure, axes = plt.subplots(1, 2, figsize=(20, 9), dpi=dpi, squeeze=False)
    for axis, encoder in zip(axes[0], ENCODERS, strict=True):
        if coords[encoder].shape != (total, 2):
            raise ValueError(f"joint coordinate shape mismatch for {encoder}")
        cursor = 0
        for source in SOURCE_ORDER:
            count = len(bundles[source].records)
            points = coords[encoder][cursor : cursor + count]
            color, marker, size, alpha = SOURCE_SPECS[source]
            axis.scatter(points[:, 0], points[:, 1], s=size, color=color, marker=marker, alpha=alpha, edgecolors="white" if source != "PV-A" else "none", linewidths=0.25, label=f"{source} (n={count})", rasterized=True)
            cursor += count
        axis.set_title(f"{('DINOv2' if encoder == 'dinov2' else 'CLIP')} joint t-SNE")
        axis.set_xlabel("joint t-SNE 1")
        axis.set_ylabel("joint t-SNE 2")
        axis.grid(color="#d8d8d8", linewidth=0.4, alpha=0.52)
        axis.spines[["top", "right"]].set_visible(False)
        axis.legend(frameon=False, loc="best", fontsize=8)
    figure.suptitle(title, fontsize=17)
    figure.text(0.5, 0.012, "Each encoder is fitted jointly in its own high-dimensional feature space; axes are not cross-encoder coordinates.", ha="center", fontsize=9, color="#444444")
    figure.tight_layout(rect=(0.0, 0.035, 1.0, 0.95))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def source_gap_rows(metrics: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Flatten the 21 pairwise source gaps for both frozen encoders."""
    rows: list[dict[str, Any]] = []
    for encoder in ENCODERS:
        pairwise = metrics["per_encoder"][encoder]["pairwise"]
        for first, second in itertools.combinations(SOURCE_ORDER, 2):
            first_key, second_key = SOURCE_KEYS[first], SOURCE_KEYS[second]
            nearest = pairwise[f"{first_key}_vs_{second_key}"][
                "nearest_other_source_cosine_distance"
            ]
            forward = float(nearest[f"{first_key}_to_{second_key}"]["mean"])
            reverse = float(nearest[f"{second_key}_to_{first_key}"]["mean"])
            rows.append(
                {
                    "encoder": encoder,
                    "source_a": first,
                    "source_b": second,
                    "a_to_b_mean_nearest_cosine_distance": format(forward, ".9g"),
                    "b_to_a_mean_nearest_cosine_distance": format(reverse, ".9g"),
                    "symmetric_mean_nearest_cosine_distance": format(
                        0.5 * (forward + reverse), ".9g"
                    ),
                }
            )
    return rows


def save_source_gap_heatmap(
    rows: Sequence[Mapping[str, Any]], path: Path, *, dpi: int
) -> None:
    """Plot symmetric high-dimensional source gaps; lower means closer."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(22, 9), dpi=dpi, squeeze=False)
    short_names = ("PV-A", "Artiverse", "Articraft", "PartNet", "Infinigen", "AOC", "PhysX")
    for axis, encoder in zip(axes[0], ENCODERS, strict=True):
        matrix = np.zeros((len(SOURCE_ORDER), len(SOURCE_ORDER)), dtype=np.float64)
        positions = {source: index for index, source in enumerate(SOURCE_ORDER)}
        for row in rows:
            if row["encoder"] != encoder:
                continue
            first, second = positions[str(row["source_a"])], positions[str(row["source_b"])]
            value = float(row["symmetric_mean_nearest_cosine_distance"])
            matrix[first, second] = matrix[second, first] = value
        nonzero = matrix[matrix > 0]
        image_handle = axis.imshow(
            matrix,
            cmap="coolwarm",
            vmin=float(nonzero.min()) if len(nonzero) else 0.0,
            vmax=float(nonzero.max()) if len(nonzero) else 1.0,
        )
        axis.set_xticks(range(len(short_names)), short_names, rotation=42, ha="right")
        axis.set_yticks(range(len(short_names)), short_names)
        axis.set_title(f"{('DINOv2' if encoder == 'dinov2' else 'CLIP')} source gap")
        for row_index in range(len(SOURCE_ORDER)):
            for column_index in range(len(SOURCE_ORDER)):
                label = "-" if row_index == column_index else f"{matrix[row_index, column_index]:.3f}"
                axis.text(column_index, row_index, label, ha="center", va="center", fontsize=8)
        figure.colorbar(image_handle, ax=axis, fraction=0.046, pad=0.04)
    figure.suptitle("Symmetric mean nearest-source cosine distance (lower is closer)", fontsize=16)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def write_coords(path: Path, coords: np.ndarray, bundle: GenericBundle, *, prefix: str) -> None:
    rows = []
    for index, (point, record) in enumerate(zip(coords, bundle.records, strict=True), start=1):
        rows.append({"tsne_x": format(float(point[0]), ".9g"), "tsne_y": format(float(point[1]), ".9g"), "category_id": f"{prefix}{index:04d}", "category": record.category, "dataset_id": record.dataset_id, "image_path": str(record.image_path)})
    atomic_csv(path, rows)


def write_joint_coords(path: Path, coords: np.ndarray, bundles: Mapping[str, GenericBundle]) -> None:
    rows: list[dict[str, Any]] = []
    cursor = 0
    for source in SOURCE_ORDER:
        bundle = bundles[source]
        for local, (point, record) in enumerate(zip(coords[cursor : cursor + len(bundle.records)], bundle.records, strict=True), start=1):
            rows.append({"tsne_x": format(float(point[0]), ".9g"), "tsne_y": format(float(point[1]), ".9g"), "joint_index": cursor + local - 1, "dataset": source, "class_id": f"{source[:2].upper()}{local:04d}", "class_name": record.category, "dataset_id": record.dataset_id, "source": record.origin})
        cursor += len(bundle.records)
    atomic_csv(path, rows)


def compute_metrics(features: Mapping[str, Mapping[str, np.ndarray]], coords: Mapping[str, np.ndarray], *, neighbor_fraction: float) -> dict[str, Any]:
    four = _four()
    three = four._three()
    base = three._base()
    output: dict[str, Any] = {
        "schema_version": 1,
        "protocol": {"distance": "cosine distance on L2-normalized high-dimensional embeddings", "joint_projection": "concatenate seven sources per encoder, PCA50, then one fitted t-SNE", "neighbor_fraction": neighbor_fraction, "tsne_policy": "visualization only; high-dimensional metrics are primary"},
        "encoder_agreement": {},
        "per_encoder": {},
    }
    for source in SOURCE_ORDER:
        output["encoder_agreement"][SOURCE_KEYS[source]] = base.encoder_agreement_metrics(features[source]["dinov2"], features[source]["clip"], neighbor_fraction=neighbor_fraction)
    for encoder in ENCODERS:
        encoder_features = {source: features[source][encoder] for source in SOURCE_ORDER}
        pairwise = {}
        for first, second in itertools.combinations(SOURCE_ORDER, 2):
            pairwise[f"{SOURCE_KEYS[first]}_vs_{SOURCE_KEYS[second]}"] = three.named_cross_dataset_metrics(encoder_features[first], encoder_features[second], first_name=SOURCE_KEYS[first], second_name=SOURCE_KEYS[second], neighbor_fraction=neighbor_fraction)
        combined = np.vstack([encoder_features[source] for source in SOURCE_ORDER])
        output["per_encoder"][encoder] = {"pairwise": pairwise, "seven_source": three.multi_source_metrics(encoder_features, SOURCE_ORDER, neighbor_fraction=neighbor_fraction), "joint_projection_trustworthiness": base.projection_trustworthiness(combined, coords[encoder], neighbor_fraction=neighbor_fraction)}
    return output


def _finite(value: Any) -> bool:
    if isinstance(value, dict):
        return all(_finite(item) for item in value.values())
    if isinstance(value, list):
        return all(_finite(item) for item in value)
    if isinstance(value, float):
        return math.isfinite(value)
    return True


def audit_outputs(output_dir: Path, *, bundles: Mapping[str, GenericBundle], features: Mapping[str, Mapping[str, np.ndarray]], coords: Mapping[str, np.ndarray], artifact_paths: Sequence[Path]) -> dict[str, Any]:
    checks: dict[str, bool] = {}
    artifacts: dict[str, Any] = {}
    dimensions = {encoder: int(features["PV-A"][encoder].shape[1]) for encoder in ENCODERS}
    total = sum(len(bundles[source].records) for source in SOURCE_ORDER)
    for path in artifact_paths:
        rel = path.relative_to(output_dir).as_posix()
        if not path.is_file():
            checks[f"exists:{rel}"] = False
            continue
        checks[f"exists:{rel}"] = True
        receipt: dict[str, Any] = {"sha256": sha256_file(path), "size_bytes": path.stat().st_size}
        if path.suffix == ".npy":
            matrix = np.load(path, allow_pickle=False)
            encoder = path.parent.name
            source_key = path.name.removesuffix("_one_shot_features.npy")
            expected_rows = next((len(bundles[s].records) for s in SOURCE_ORDER if bundles[s].key == source_key), None)
            checks[f"shape:{rel}"] = expected_rows is not None and matrix.shape == (expected_rows, dimensions[encoder])
            checks[f"finite:{rel}"] = bool(np.isfinite(matrix).all())
            checks[f"normalized:{rel}"] = bool(np.allclose(np.linalg.norm(matrix, axis=1), 1.0, rtol=2e-5, atol=2e-5))
            receipt.update({"shape": list(matrix.shape), "dtype": str(matrix.dtype)})
        elif path.suffix == ".csv":
            rows = _read_csv(path)
            if path.name in {"joint_tsne_coordinates.csv", "joint_seven_source_tsne_coordinates.csv"}:
                checks[f"rows:{rel}"] = len(rows) == total
                checks[f"source_counts:{rel}"] = __import__("collections").Counter(row.get("dataset") for row in rows) == __import__("collections").Counter({s: len(bundles[s].records) for s in SOURCE_ORDER})
            elif path.name.endswith("_tsne_coordinates.csv"):
                key = path.name.removesuffix("_one_shot_tsne_coordinates.csv")
                expected = next((len(bundles[s].records) for s in SOURCE_ORDER if bundles[s].key == key), None)
                checks[f"rows:{rel}"] = expected is not None and len(rows) == expected
            receipt["row_count"] = len(rows)
        elif path.suffix == ".png":
            with Image.open(path) as image:
                image.load()
                receipt.update({"width": image.width, "height": image.height, "grayscale_std": ImageStat.Stat(image.convert("L")).stddev[0]})
                checks[f"dimensions:{rel}"] = image.width >= 1000 and image.height >= 700
                checks[f"nonblank:{rel}"] = receipt["grayscale_std"] > 1.0
        elif path.name == "high_dimensional_metrics.json":
            metrics = load_json(path)
            checks[f"finite:{rel}"] = _finite(metrics)
            checks[f"pair_count:{rel}"] = all(len(metrics.get("per_encoder", {}).get(encoder, {}).get("pairwise", {})) == 21 for encoder in ENCODERS)
        artifacts[rel] = receipt
    for source in SOURCE_ORDER:
        bundle = bundles[source]
        for encoder in ENCODERS:
            matrix = features[source][encoder]
            checks[f"memory_shape:{SOURCE_KEYS[source]}:{encoder}"] = matrix.shape == (len(bundle.records), dimensions[encoder])
        checks[f"unique_categories:{SOURCE_KEYS[source]}"] = len(set(bundle.categories)) == len(bundle.records)
    for encoder in ENCODERS:
        checks[f"joint_shape:{encoder}"] = coords[encoder].shape == (total, 2)
    return {"schema_version": 1, "pass": all(checks.values()), "checks": checks, "artifacts": artifacts}


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.batch_size < 1 or args.num_workers < 1 or args.plot_dpi < 80:
        raise ValueError("batch size/workers must be positive and plot DPI >= 80")
    if not 0 < args.neighbor_fraction < 1:
        raise ValueError("neighbor fraction must be between zero and one")
    four = _four()
    three = four._three()
    base = three._base()
    pva = base.load_pva_frozen(args.pva_dir, strict_counts=not args.allow_count_drift)
    artiverse = base.load_artiverse_uniform(args.artiverse_render_root, pva_render_config=pva.render_config, strict_counts=not args.allow_count_drift, verify_glb_hashes=not args.skip_artiverse_glb_hash_verification, one_shot_only=True)
    articraft = three.load_articraft_uniform(args.articraft_render_root, pva_render_config=pva.render_config, strict_counts=not args.allow_count_drift)
    partnet = four.load_partnet_uniform(args.partnet_render_root, pva_render_config=pva.render_config, strict_counts=not args.allow_count_drift)
    infinigen = load_uniform_bundle(args.infinigen_render_root, name="Infinigen-Sim", key="infinigen_sim", pva_config=pva.render_config, expected_count=17, expected_categories=17, strict_counts=not args.allow_count_drift)
    aoc = load_uniform_bundle(args.aoc_render_root, name="Articulated-Object-Code", key="articulated_object_code", pva_config=pva.render_config, expected_count=660, expected_categories=660, strict_counts=not args.allow_count_drift)
    physx = load_uniform_bundle(args.physx_render_root, name="PhysX-Mobility", key="physx_mobility", pva_config=pva.render_config, expected_count=132, expected_categories=132, strict_counts=not args.allow_count_drift)
    bundles: dict[str, GenericBundle] = {
        "PV-A": _existing_bundle("PV-A", "pva", pva.root, pva.records, official_models=len(pva.records), official_categories=len(pva.records), receipts=pva.receipts),
        "Artiverse": _existing_bundle("Artiverse", "artiverse", artiverse.root, artiverse.records, official_models=len(artiverse.records), official_categories=len(artiverse.records), receipts=artiverse.receipts),
        "Articraft-10K": _existing_bundle("Articraft-10K", "articraft10k", articraft.root, articraft.records, official_models=articraft.official_model_count, official_categories=articraft.official_category_count, receipts=articraft.receipts),
        "PartNet-Mobility": _existing_bundle("PartNet-Mobility", "partnet_mobility", partnet.root, partnet.records, official_models=partnet.official_model_count, official_categories=partnet.official_category_count, receipts=partnet.receipts),
        "Infinigen-Sim": infinigen,
        "Articulated-Object-Code": aoc,
        "PhysX-Mobility": physx,
    }
    output_dir = args.output_dir.expanduser().resolve()
    input_roots = [bundle.root for bundle in bundles.values()] + [args.artiverse_comparison_dir.expanduser().resolve(), args.prior_comparison_dir.expanduser().resolve()]
    for input_root in input_roots:
        if output_dir == input_root or output_dir.is_relative_to(input_root) or input_root.is_relative_to(output_dir):
            raise ValueError(f"output directory overlaps input root: {input_root}")
    output_dir.mkdir(parents=True, exist_ok=True)

    # Reuse the already audited four-source feature caches, then extract only
    # the three newly rendered datasets.
    artiverse_features = four._three()._load_artiverse_cached(args.artiverse_comparison_dir.expanduser().resolve(strict=True), pva=pva, artiverse=artiverse)
    prior = args.prior_comparison_dir.expanduser().resolve(strict=True)
    articraft_features = _copy_existing_features(bundles["Articraft-10K"], pva=pva, source_dir=prior, output_dir=output_dir, source_key="articraft")
    partnet_features = _copy_existing_features(bundles["PartNet-Mobility"], pva=pva, source_dir=prior, output_dir=output_dir, source_key="partnet")
    # Materialize copies under the new output with a fresh manifest receipt.
    for source, matrices in (("Articraft-10K", articraft_features), ("PartNet-Mobility", partnet_features)):
        bundle = bundles[source]
        for encoder in ENCODERS:
            key = bundle.key
            atomic_npy(output_dir / encoder / f"{key}_one_shot_features.npy", matrices[encoder])
            atomic_json(output_dir / encoder / f"{key}_feature_manifest.json", {"schema_version": 1, "dataset": source, "encoder": encoder, "model_fingerprint": pva.feature_manifests[encoder]["model_fingerprint"], "model_count": len(bundle.records), "ordered_image_fingerprint": hashlib.sha256("\n".join(f"{r.category}\0{r.dataset_id}\0{r.image_bytes}\0{r.image_sha256}" for r in bundle.records).encode()).hexdigest(), "source_feature_cache": str(prior), "source_feature_cache_sha256": sha256_file(prior / encoder / f"{('articraft' if source == 'Articraft-10K' else 'partnet')}_feature_manifest.json")})
    source_features: dict[str, Mapping[str, np.ndarray]] = {
        "PV-A": pva.features,
        "Artiverse": artiverse_features,
        "Articraft-10K": articraft_features,
        "PartNet-Mobility": partnet_features,
    }
    for source in ("Infinigen-Sim", "Articulated-Object-Code", "PhysX-Mobility"):
        source_features[source] = _extract_bundle(bundles[source], pva=pva, output_dir=output_dir, force_extract=args.force_extract, batch_size=args.batch_size, device=args.device, num_workers=args.num_workers, use_amp=args.use_amp)

    # Materialize a uniform feature namespace for every source.  The older
    # frozen PV-A/Artiverse runs use different cache filenames, while the
    # newly extracted bundles already wrote their own files.  Keeping one
    # canonical filename per source makes the final audit and downstream
    # consumers independent of the originating run.
    for source in SOURCE_ORDER:
        bundle = bundles[source]
        ordered_fp = hashlib.sha256(
            "\n".join(
                f"{record.category}\0{record.dataset_id}\0{record.image_bytes}\0{record.image_sha256}"
                for record in bundle.records
            ).encode("utf-8")
        ).hexdigest()
        for encoder in ENCODERS:
            matrix = np.asarray(source_features[source][encoder], dtype=np.float32)
            feature_path = output_dir / encoder / f"{bundle.key}_one_shot_features.npy"
            manifest_path = output_dir / encoder / f"{bundle.key}_feature_manifest.json"
            atomic_npy(feature_path, matrix)
            if not manifest_path.is_file() or source in {"PV-A", "Artiverse", "Articraft-10K", "PartNet-Mobility"}:
                atomic_json(
                    manifest_path,
                    {
                        "schema_version": 1,
                        "dataset": source,
                        "sample_mode": "one_representative_per_exact_category",
                        "encoder": encoder,
                        "model_type": encoder,
                        "model_path": str(Path(str(pva.feature_manifests[encoder]["model_path"])).resolve(strict=True)),
                        "model_fingerprint": pva.feature_manifests[encoder]["model_fingerprint"],
                        "model_count": len(bundle.records),
                        "ordered_image_fingerprint": ordered_fp,
                        "feature_source": "frozen_prior_cache" if source in {"PV-A", "Artiverse", "Articraft-10K", "PartNet-Mobility"} else "current_run",
                    },
                )

    independent: dict[str, dict[str, np.ndarray]] = {source: {} for source in SOURCE_ORDER}
    joint: dict[str, np.ndarray] = {}
    tsne_info: dict[str, Any] = {}
    for encoder in ENCODERS:
        joint_input = np.vstack([source_features[source][encoder] for source in SOURCE_ORDER])
        print(f"[tsne] {encoder} joint n={len(joint_input)}", flush=True)
        joint[encoder], joint_info = _compute_tsne(joint_input, perplexity=args.perplexity, seed=args.seed, max_iter=args.tsne_max_iter, jobs=args.tsne_jobs, verbose=args.tsne_verbose)
        tsne_info[encoder] = {"joint": joint_info, "independent": {}}
        for source in SOURCE_ORDER:
            print(f"[tsne] {encoder} {source} n={len(bundles[source].records)}", flush=True)
            independent[source][encoder], info = _compute_tsne(source_features[source][encoder], perplexity=args.perplexity, seed=args.seed, max_iter=args.tsne_max_iter, jobs=args.tsne_jobs, verbose=args.tsne_verbose)
            tsne_info[encoder]["independent"][source] = info

    atomic_json(output_dir / "dataset_counts.json", {"schema_version": 1, "counts": {source: len(bundles[source].records) for source in SOURCE_ORDER}, "official_counts": {source: {"models": bundles[source].official_model_count, "categories": bundles[source].official_category_count} for source in SOURCE_ORDER}})
    all_artifacts: list[Path] = [output_dir / "dataset_counts.json"]
    for source in SOURCE_ORDER:
        bundle = bundles[source]
        rows = []
        colors = category_colors(len(bundle.records))
        for index, (record, color) in enumerate(zip(bundle.records, colors, strict=True), start=1):
            rows.append({"category_id": f"{source[:2].upper()}{index:04d}", "category": record.category, "dataset_id": record.dataset_id, "ordinal": record.ordinal, "source_path": str(record.source_path), "urdf_path": str(record.urdf_path or ""), "image_path": str(record.image_path), "image_bytes": record.image_bytes, "image_sha256": record.image_sha256, "color": color})
        atomic_csv(output_dir / f"{bundles[source].key}_index.csv", rows)
        all_artifacts.append(output_dir / f"{bundles[source].key}_index.csv")
        for encoder in ENCODERS:
            save_independent_plot(independent[source][encoder], bundle, output_dir / encoder / f"{bundle.key}_one_shot_tsne.png", encoder=encoder, dpi=args.plot_dpi)
            write_coords(output_dir / encoder / f"{bundle.key}_one_shot_tsne_coordinates.csv", independent[source][encoder], bundle, prefix=source[:2].upper())
            atomic_json(output_dir / encoder / f"{bundle.key}_tsne_manifest.json", tsne_info[encoder]["independent"][source])
            all_artifacts.extend([output_dir / encoder / f"{bundle.key}_one_shot_features.npy", output_dir / encoder / f"{bundle.key}_feature_manifest.json", output_dir / encoder / f"{bundle.key}_one_shot_tsne.png", output_dir / encoder / f"{bundle.key}_one_shot_tsne_coordinates.csv", output_dir / encoder / f"{bundle.key}_tsne_manifest.json"])

    save_joint_plot(joint, bundles, output_dir / "joint_seven_source_comparison.png", dpi=args.plot_dpi, title="Uniform Blender comparison: seven class-level sources")
    all_artifacts.append(output_dir / "joint_seven_source_comparison.png")
    for encoder in ENCODERS:
        write_joint_coords(output_dir / encoder / "joint_seven_source_tsne_coordinates.csv", joint[encoder], bundles)
        atomic_json(output_dir / encoder / "joint_tsne_manifest.json", tsne_info[encoder]["joint"])
        all_artifacts.extend([output_dir / encoder / "joint_seven_source_tsne_coordinates.csv", output_dir / encoder / "joint_tsne_manifest.json"])
    metrics = compute_metrics(source_features, joint, neighbor_fraction=args.neighbor_fraction)
    atomic_json(output_dir / "high_dimensional_metrics.json", metrics)
    all_artifacts.append(output_dir / "high_dimensional_metrics.json")
    gap_rows = source_gap_rows(metrics)
    atomic_csv(output_dir / "source_gap_summary.csv", gap_rows)
    save_source_gap_heatmap(
        gap_rows, output_dir / "source_gap_heatmaps.png", dpi=args.plot_dpi
    )
    all_artifacts.extend(
        [output_dir / "source_gap_summary.csv", output_dir / "source_gap_heatmaps.png"]
    )
    contract = {"schema_version": 1, "artifact_contract": "seven_source_uniform_comparison_v1", "counts": {source: len(bundles[source].records) for source in SOURCE_ORDER}, "joint_total": sum(len(bundles[source].records) for source in SOURCE_ORDER), "official_counts": {source: {"models": bundles[source].official_model_count, "categories": bundles[source].official_category_count} for source in SOURCE_ORDER}, "inputs": {source: {**{key: value for key, value in bundle.receipts.items()}, "root": str(bundle.root), "config": bundle.config} for source, bundle in bundles.items()}, "feature_extraction": {"device": args.device, "batch_size": args.batch_size, "num_workers": args.num_workers, "use_amp": bool(args.use_amp), "model_fingerprints": {encoder: pva.feature_manifests[encoder]["model_fingerprint"] for encoder in ENCODERS}}, "projection": {"perplexity": args.perplexity, "seed": args.seed, "tsne_max_iter": args.tsne_max_iter, "tsne_jobs": args.tsne_jobs, "neighbor_fraction": args.neighbor_fraction}, "implementation": {"script": str(SCRIPT), "script_sha256": sha256_file(SCRIPT), "four_source_script_sha256": sha256_file(FOUR_SOURCE_SCRIPT)}, "limits": ["Counts are class-level representatives, not semantically matched categories.", "AOC uses 660 loadable categories: viable preferred, loads_only fallback; 127 broken-only categories are excluded.", "PhysX keeps 132 exact raw labels, including spacing/casing variants.", "t-SNE is visualization only; high-dimensional cosine metrics are primary."]}
    atomic_json(output_dir / "run_contract.json", contract)
    all_artifacts.append(output_dir / "run_contract.json")
    audit = audit_outputs(output_dir, bundles=bundles, features=source_features, coords=joint, artifact_paths=all_artifacts)
    atomic_json(output_dir / "final_audit.json", audit)
    if not audit["pass"]:
        raise RuntimeError("seven-source output audit failed")
    manifest = {"schema_version": 1, "contract": contract, "audit": audit, "metrics": metrics, "tsne": tsne_info}
    atomic_json(output_dir / "run_manifest.json", manifest)
    return {"output_dir": str(output_dir), "counts": contract["counts"], "joint_total": contract["joint_total"], "audit_pass": True, "metrics": metrics}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pva-dir", type=Path, default=DEFAULT_PVA_DIR)
    parser.add_argument("--artiverse-render-root", type=Path, default=DEFAULT_ARTIVERSE_RENDER_ROOT)
    parser.add_argument("--artiverse-comparison-dir", type=Path, default=DEFAULT_ARTIVERSE_DIR)
    parser.add_argument("--articraft-render-root", type=Path, default=DEFAULT_ARTICRAFT_RENDER_ROOT)
    parser.add_argument("--partnet-render-root", type=Path, default=DEFAULT_PARTNET_RENDER_ROOT)
    parser.add_argument("--infinigen-render-root", type=Path, default=DEFAULT_INFINIGEN_RENDER_ROOT)
    parser.add_argument("--aoc-render-root", type=Path, default=DEFAULT_AOC_RENDER_ROOT)
    parser.add_argument("--physx-render-root", type=Path, default=DEFAULT_PHYSX_RENDER_ROOT)
    parser.add_argument("--prior-comparison-dir", type=Path, default=DEFAULT_PRIOR_COMPARISON_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--num-workers", type=int, default=8)
    parser.add_argument("--perplexity", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--tsne-max-iter", type=int, default=1000)
    parser.add_argument("--tsne-jobs", type=int, default=8)
    parser.add_argument("--tsne-verbose", type=int, default=1)
    parser.add_argument("--plot-dpi", type=int, default=180)
    parser.add_argument("--neighbor-fraction", type=float, default=0.06)
    parser.add_argument("--force-extract", action="store_true")
    parser.add_argument("--use-amp", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--skip-artiverse-glb-hash-verification", action="store_true")
    parser.add_argument("--allow-count-drift", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(sys.argv[1:] if argv is None else argv)
    result = run(args)
    print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
