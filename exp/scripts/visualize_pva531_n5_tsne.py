#!/usr/bin/env python3
"""Image-level DINOv2/CLIP t-SNE for the audited PV-A 531 x 5 render set.

Every rendered asset is one t-SNE sample.  The five samples belonging to one
generator class share the frozen class color used by the earlier PV-A plot.
DINOv2 and CLIP are embedded independently; their feature spaces and t-SNE
coordinate systems are never mixed.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
import re
import shutil
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

for _thread_variable in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_thread_variable, "1")

import numpy as np
from PIL import Image


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
HELPER_SCRIPT = SCRIPT.with_name("visualize_pva531_picture_tsne.py")
DEFAULT_INDEX_CSV = REPO_ROOT / "template_maps" / "generator_picture_index.csv"
DEFAULT_RENDER_ROOT = REPO_ROOT / "exp" / "uniform531_n5_studio_256_v1"
DEFAULT_COLOR_INDEX = (
    REPO_ROOT / "exp" / "runtime" / "pva531_uniform_tsne"
    / "generator_class_color_index.csv"
)
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exp" / "pva531_n5_tsne"
DEFAULT_DINO_MODEL = Path(
    "/root/.cache/huggingface/hub/models--facebook--dinov2-base/"
    "snapshots/f9e44c814b77203eaa57a6bdbbd535f21ede1415"
)
DEFAULT_CLIP_MODEL = Path(
    "/root/.cache/huggingface/hub/models--openai--clip-vit-base-patch32/"
    "snapshots/3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"
)
EXPECTED_CLASS_COUNT = 531
EXPECTED_PER_CLASS = 5
EXPECTED_SAMPLE_COUNT = EXPECTED_CLASS_COUNT * EXPECTED_PER_CLASS
RENDER_CONTRACT = "pva531_n5_uniform_studio_v1"
VALID_RENDER_STATUSES = {"rendered", "reused_valid"}


@dataclass(frozen=True, slots=True)
class GeneratorRecord:
    generator_index: str
    generator_name: str
    source_type: str
    picture_category: str
    picture_label: str
    image_paths: tuple[Path, ...]


@dataclass(frozen=True, slots=True)
class SampleRecord:
    ordinal: int
    render_key: str
    generator_index: str
    generator_name: str
    sample_index: int
    source_type: str
    picture_category: str
    asset_id: str
    seed: int
    rank_sha256: str
    asset_dir: Path
    urdf_sha256: str
    package_content_sha256: str
    image_path: Path
    png_bytes: int
    png_sha256: str


@dataclass(frozen=True, slots=True)
class DatasetBundle:
    records: tuple[GeneratorRecord, ...]
    samples: tuple[SampleRecord, ...]
    raw_image_paths: tuple[Path, ...]
    raw_path_to_index: dict[Path, int]
    summary: dict[str, Any]


_HELPER: Any | None = None


def _helper() -> Any:
    """Load the established local feature extractor and PCA+t-SNE routine."""
    global _HELPER
    if _HELPER is None:
        name = "_pva531_n5_feature_helper"
        spec = importlib.util.spec_from_file_location(name, HELPER_SCRIPT)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot load feature helper: {HELPER_SCRIPT}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        _HELPER = module
    return _HELPER


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _receipt(path: Path) -> dict[str, Any]:
    path = path.expanduser().resolve(strict=True)
    return {
        "path": str(path),
        "sha256": sha256_file(path),
        "size_bytes": path.stat().st_size,
    }


def _load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"unreadable JSON object: {path}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return payload


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"CSV has no data rows: {path}")
    return rows


def _atomic_write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _atomic_write_csv(
    path: Path,
    fieldnames: Sequence[str],
    rows: Sequence[Mapping[str, Any]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def _atomic_save_npy(path: Path, array: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("wb") as stream:
        np.save(stream, array, allow_pickle=False)
    temporary.replace(path)


def _strict_feature_cache_valid(
    model_dir: Path,
    requested_contract: Mapping[str, Any],
) -> bool:
    """Require both the numerical contract and bytes receipt before cache reuse."""
    feature_path = model_dir / "raw_image_features.npy"
    manifest_path = model_dir / "feature_manifest.json"
    if not feature_path.is_file() or not manifest_path.is_file():
        return False
    try:
        manifest = _load_json(manifest_path)
        receipt = manifest.get("raw_feature_receipt")
        if manifest.get("strict_cache_contract") != dict(requested_contract):
            return False
        if not isinstance(receipt, dict):
            return False
        return (
            receipt.get("sha256") == sha256_file(feature_path)
            and receipt.get("size_bytes") == feature_path.stat().st_size
        )
    except (OSError, ValueError):
        return False


def _seal_feature_cache(
    model_dir: Path,
    requested_contract: Mapping[str, Any],
) -> dict[str, Any]:
    feature_path = model_dir / "raw_image_features.npy"
    manifest_path = model_dir / "feature_manifest.json"
    manifest = _load_json(manifest_path)
    receipt = _receipt(feature_path)
    manifest["strict_cache_contract"] = dict(requested_contract)
    manifest["raw_feature_receipt"] = receipt
    _atomic_write_json(manifest_path, manifest)
    return receipt


def _normalize_source_type(value: str) -> str:
    value = value.strip()
    if value in {"builtin_no_picture", "articraft_builtin_dataset_no_picture"}:
        return "builtin_no_picture"
    if value == "picture_backed":
        return value
    raise ValueError(f"unsupported source_type {value!r}")


def _parse_int(value: Any, *, field: str) -> int:
    try:
        return int(str(value))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid integer {field}: {value!r}") from exc


def _inside_root(value: str, root: Path, *, field: str) -> Path:
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = root / candidate
    candidate = candidate.resolve(strict=True)
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{field} resolves outside render root: {candidate}") from exc
    return candidate


def _validate_sample_indices(
    rows: Sequence[Mapping[str, str]],
    *,
    expected_per_class: int,
    source: Path,
) -> None:
    grouped: dict[str, list[int]] = defaultdict(list)
    for row in rows:
        grouped[(row.get("generator_index") or "").strip()].append(
            _parse_int(row.get("sample_index"), field="sample_index")
        )
    expected = list(range(1, expected_per_class + 1))
    invalid = {
        key: sorted(values)
        for key, values in grouped.items()
        if sorted(values) != expected
    }
    if invalid:
        example_key, values = next(iter(invalid.items()))
        raise ValueError(
            f"sample_index must be exact 1..{expected_per_class} once per class in "
            f"{source}; {example_key} has {values}"
        )


def discover_n5_render_records(
    index_csv: Path,
    *,
    render_root: Path,
    strict_counts: bool = True,
    expected_per_class: int = EXPECTED_PER_CLASS,
) -> DatasetBundle:
    """Load and fully receipt-check the 531-class, five-render PV-A cohort."""
    if expected_per_class < 1:
        raise ValueError("expected_per_class must be positive")
    index_csv = index_csv.expanduser().resolve(strict=True)
    render_root = render_root.expanduser().resolve(strict=True)
    config_path = render_root / "render_config.json"
    roster_path = render_root / "render_roster.csv"
    manifest_path = render_root / "render_manifest.csv"
    for required in (config_path, roster_path, manifest_path):
        if not required.is_file():
            raise FileNotFoundError(required)

    config = _load_json(config_path)
    if config.get("render_contract") != RENDER_CONTRACT:
        raise ValueError(f"unexpected render contract in {config_path}")
    if config.get("schema_version") != 1:
        raise ValueError(f"expected schema_version 1 in {config_path}")
    if _parse_int(config.get("per_class"), field="config.per_class") != expected_per_class:
        raise ValueError("render config per_class does not match expected_per_class")
    if config.get("index_csv_sha256") != sha256_file(index_csv):
        raise ValueError("render config/index CSV SHA mismatch")
    if config.get("render_roster_sha256") != sha256_file(roster_path):
        raise ValueError("render config/render roster SHA mismatch")
    declared_roster = Path(str(config.get("render_roster") or "")).expanduser().resolve()
    if declared_roster != roster_path:
        raise ValueError(f"render config roster path mismatch: {declared_roster} != {roster_path}")
    input_receipt = config.get("input_receipt")
    if not isinstance(input_receipt, dict) or re.fullmatch(
        r"[0-9a-f]{64}", str(input_receipt.get("asset_and_dependency_sha256") or "")
    ) is None:
        raise ValueError("render config has no valid input dependency receipt")

    index_rows = _read_csv(index_csv)
    roster_rows = _read_csv(roster_path)
    manifest_rows = _read_csv(manifest_path)
    if strict_counts and len(index_rows) != EXPECTED_CLASS_COUNT:
        raise ValueError(f"expected {EXPECTED_CLASS_COUNT} classes, found {len(index_rows)}")
    expected_sample_count = len(index_rows) * expected_per_class
    if len(roster_rows) != expected_sample_count or len(manifest_rows) != expected_sample_count:
        raise ValueError(
            "render roster/manifest sample count mismatch: "
            f"expected {expected_sample_count}, roster={len(roster_rows)}, "
            f"manifest={len(manifest_rows)}"
        )
    if _parse_int(config.get("class_count"), field="config.class_count") != len(index_rows):
        raise ValueError("render config class_count does not match index CSV")
    if _parse_int(config.get("asset_count"), field="config.asset_count") != expected_sample_count:
        raise ValueError("render config asset_count does not match roster")
    _validate_sample_indices(
        roster_rows, expected_per_class=expected_per_class, source=roster_path
    )
    _validate_sample_indices(
        manifest_rows, expected_per_class=expected_per_class, source=manifest_path
    )

    index_by_id: dict[str, dict[str, str]] = {}
    index_order: list[str] = []
    for row_number, row in enumerate(index_rows, start=2):
        generator_index = (row.get("generator_index") or "").strip()
        generator_name = (row.get("generator_name") or "").strip()
        if not generator_index or not generator_name:
            raise ValueError(f"missing generator identity at {index_csv}:{row_number}")
        if generator_index in index_by_id:
            raise ValueError(f"duplicate generator_index {generator_index} in {index_csv}")
        _normalize_source_type(row.get("source_type") or "")
        index_by_id[generator_index] = row
        index_order.append(generator_index)

    manifest_by_key: dict[str, dict[str, str]] = {}
    for row in manifest_rows:
        key = (row.get("render_key") or "").strip()
        if not key or key in manifest_by_key:
            raise ValueError(f"missing or duplicate render_key {key!r} in {manifest_path}")
        if (row.get("status") or "").strip() not in VALID_RENDER_STATUSES:
            raise ValueError(f"render {key} is not successful")
        manifest_by_key[key] = row

    roster_by_class: dict[str, list[dict[str, str]]] = defaultdict(list)
    seen_keys: set[str] = set()
    seen_paths: set[Path] = set()
    for ordinal, row in enumerate(roster_rows, start=1):
        key = (row.get("render_key") or "").strip()
        generator_index = (row.get("generator_index") or "").strip()
        if key in seen_keys or not key:
            raise ValueError(f"missing or duplicate render_key {key!r} in {roster_path}")
        seen_keys.add(key)
        if generator_index not in index_by_id:
            raise ValueError(f"unknown generator_index {generator_index} in {roster_path}")
        if _parse_int(row.get("ordinal"), field="ordinal") != ordinal:
            raise ValueError(f"non-contiguous ordinal at render roster row {ordinal + 1}")
        manifest_row = manifest_by_key.get(key)
        if manifest_row is None:
            raise ValueError(f"render_key {key} is absent from {manifest_path}")
        for field in (
            "generator_index", "generator_name", "sample_index", "source_type",
            "picture_category", "asset_id", "seed", "rank_sha256", "asset_dir",
            "urdf_sha256", "package_content_sha256", "output_path",
        ):
            if (row.get(field) or "").strip() != (manifest_row.get(field) or "").strip():
                raise ValueError(f"manifest/roster {field} mismatch for {key}")
        image_path = _inside_root(row.get("output_path") or "", render_root, field="output_path")
        if image_path in seen_paths:
            raise ValueError(f"duplicate output image path: {image_path}")
        seen_paths.add(image_path)
        expected_name = f"{key}__{row.get('generator_name')}.png"
        if image_path.name != expected_name:
            raise ValueError(f"unexpected image filename for {key}: {image_path.name}")
        roster_by_class[generator_index].append(row)

    if set(roster_by_class) != set(index_by_id):
        raise ValueError("render roster generator coverage does not match index CSV")

    samples: list[SampleRecord] = []
    records: list[GeneratorRecord] = []
    image_sizes: set[tuple[int, int]] = set()
    seen_png_hashes: set[str] = set()
    ordered_png_digest = hashlib.sha256()
    manifest_offsets = {row["render_key"]: row for row in manifest_rows}
    for generator_index in index_order:
        index_row = index_by_id[generator_index]
        generator_name = (index_row.get("generator_name") or "").strip()
        source_type = _normalize_source_type(index_row.get("source_type") or "")
        class_rows = sorted(
            roster_by_class[generator_index],
            key=lambda row: _parse_int(row.get("sample_index"), field="sample_index"),
        )
        asset_ids = [(row.get("asset_id") or "").strip() for row in class_rows]
        asset_dirs = [
            str(Path(row.get("asset_dir") or "").expanduser().resolve())
            for row in class_rows
        ]
        if len(set(asset_ids)) != expected_per_class or any(not value for value in asset_ids):
            raise ValueError(
                f"asset_id must be unique for all {expected_per_class} samples of "
                f"{generator_index}"
            )
        if len(set(asset_dirs)) != expected_per_class:
            raise ValueError(
                f"asset_dir must be unique for all {expected_per_class} samples of "
                f"{generator_index}"
            )
        image_paths: list[Path] = []
        for row in class_rows:
            key = (row.get("render_key") or "").strip()
            if (row.get("generator_name") or "").strip() != generator_name:
                raise ValueError(f"generator identity mismatch for {generator_index}")
            if _normalize_source_type(row.get("source_type") or "") != source_type:
                raise ValueError(f"source_type mismatch for {generator_index}")
            manifest_row = manifest_offsets[key]
            image_path = _inside_root(row.get("output_path") or "", render_root, field="output_path")
            png_bytes = _parse_int(manifest_row.get("png_bytes"), field="png_bytes")
            if image_path.stat().st_size != png_bytes:
                raise ValueError(f"PNG byte size mismatch for {key}")
            png_sha256 = (manifest_row.get("png_sha256") or "").strip()
            if re.fullmatch(r"[0-9a-f]{64}", png_sha256) is None:
                raise ValueError(f"invalid PNG SHA for {key}")
            if sha256_file(image_path) != png_sha256:
                raise ValueError(f"PNG SHA mismatch for {key}")
            if png_sha256 in seen_png_hashes:
                raise ValueError(f"duplicate PNG content SHA across render samples: {png_sha256}")
            seen_png_hashes.add(png_sha256)
            with Image.open(image_path) as image:
                image.verify()
                image_sizes.add(tuple(image.size))
            ordered_png_digest.update(key.encode("utf-8"))
            ordered_png_digest.update(b"\0")
            ordered_png_digest.update(png_sha256.encode("ascii"))
            ordered_png_digest.update(b"\n")
            sample = SampleRecord(
                ordinal=_parse_int(row.get("ordinal"), field="ordinal"),
                render_key=key,
                generator_index=generator_index,
                generator_name=generator_name,
                sample_index=_parse_int(row.get("sample_index"), field="sample_index"),
                source_type=source_type,
                picture_category=(row.get("picture_category") or "").strip()
                or ("builtin_no_picture" if source_type == "builtin_no_picture" else "unknown"),
                asset_id=(row.get("asset_id") or "").strip(),
                seed=_parse_int(row.get("seed"), field="seed"),
                rank_sha256=(row.get("rank_sha256") or "").strip(),
                asset_dir=Path(row.get("asset_dir") or "").expanduser().resolve(),
                urdf_sha256=(row.get("urdf_sha256") or "").strip(),
                package_content_sha256=(row.get("package_content_sha256") or "").strip(),
                image_path=image_path,
                png_bytes=png_bytes,
                png_sha256=png_sha256,
            )
            samples.append(sample)
            image_paths.append(image_path)
        records.append(
            GeneratorRecord(
                generator_index=generator_index,
                generator_name=generator_name,
                source_type=source_type,
                picture_category=(index_row.get("picture_category") or "").strip()
                or ("builtin_no_picture" if source_type == "builtin_no_picture" else "unknown"),
                picture_label=(index_row.get("picture_label") or "").strip(),
                image_paths=tuple(image_paths),
            )
        )

    resolution = _parse_int(config.get("resolution"), field="config.resolution")
    if image_sizes != {(resolution, resolution)}:
        raise ValueError(
            f"render image dimensions {sorted(image_sizes)} disagree with {resolution}x{resolution}"
        )
    samples.sort(key=lambda sample: sample.ordinal)
    raw_paths = tuple(sample.image_path for sample in samples)
    source_counts = Counter(sample.source_type for sample in samples)
    summary = {
        "input_mode": "uniform_blender_531_classes_x_5_assets",
        "render_contract": RENDER_CONTRACT,
        "render_root": str(render_root),
        "index_csv": _receipt(index_csv),
        "render_config": _receipt(config_path),
        "render_roster": _receipt(roster_path),
        "render_manifest": _receipt(manifest_path),
        "class_count": len(records),
        "per_class": expected_per_class,
        "sample_count": len(samples),
        "raw_unique_image_count": len(set(raw_paths)),
        "unique_image_path_count": len(set(raw_paths)),
        "unique_png_content_count": len(seen_png_hashes),
        "source_type_counts": dict(sorted(source_counts.items())),
        "ordered_render_png_receipts_sha256": ordered_png_digest.hexdigest(),
        "resolution": resolution,
        "samples": config.get("samples"),
        "renderer_sha256": config.get("renderer_sha256"),
        "blender_version": config.get("blender_version"),
        "studio": config.get("studio"),
        "sample_unit": "one uniformly rendered asset",
        "class_unit": "PV-A generator_index",
    }
    return DatasetBundle(
        records=tuple(records),
        samples=tuple(samples),
        raw_image_paths=raw_paths,
        raw_path_to_index={path: index for index, path in enumerate(raw_paths)},
        summary=summary,
    )


def build_class_color_map(
    records: Sequence[GeneratorRecord],
    *,
    color_index_csv: Path = DEFAULT_COLOR_INDEX,
) -> dict[str, str]:
    """Align the frozen one-color-per-generator mapping by ID and name."""
    color_index_csv = color_index_csv.expanduser().resolve(strict=True)
    rows = _read_csv(color_index_csv)
    by_id: dict[str, tuple[str, str]] = {}
    colors: set[str] = set()
    for row in rows:
        generator_index = (row.get("generator_index") or "").strip()
        generator_name = (row.get("generator_name") or "").strip()
        color = (row.get("color_hex") or "").strip().lower()
        if not generator_index or generator_index in by_id:
            raise ValueError(f"duplicate or missing generator identity in {color_index_csv}")
        if re.fullmatch(r"#[0-9a-f]{6}", color) is None:
            raise ValueError(f"invalid class color for {generator_index}: {color!r}")
        if color in colors:
            raise ValueError(f"class color is not unique: {color}")
        by_id[generator_index] = (generator_name, color)
        colors.add(color)
    record_ids = [record.generator_index for record in records]
    if len(record_ids) != len(set(record_ids)) or set(record_ids) != set(by_id):
        raise ValueError("class color mapping coverage does not align with generator records")
    mapping: dict[str, str] = {}
    for record in records:
        mapped_name, color = by_id[record.generator_index]
        if mapped_name != record.generator_name:
            raise ValueError(
                f"generator name identity does not align for {record.generator_index}: "
                f"{mapped_name!r} != {record.generator_name!r}"
            )
        mapping[record.generator_index] = color
    return mapping


def audit_tsne_outputs(
    bundle: DatasetBundle,
    color_map: Mapping[str, str],
    coordinates_by_encoder: Mapping[str, np.ndarray],
) -> dict[str, Any]:
    """Reject coordinate/count/color drift before any visualization is published."""
    sample_count = len(bundle.samples)
    class_ids = [record.generator_index for record in bundle.records]
    if set(color_map) != set(class_ids) or len(color_map) != len(class_ids):
        raise ValueError("color mapping count or coverage does not match classes")
    colors = list(color_map.values())
    if len(set(colors)) != len(class_ids) or not all(
        re.fullmatch(r"#[0-9a-fA-F]{6}", color) for color in colors
    ):
        raise ValueError("color mapping must contain one unique valid color per class")
    if not coordinates_by_encoder:
        raise ValueError("no encoder coordinates supplied")
    for encoder, coordinates in coordinates_by_encoder.items():
        array = np.asarray(coordinates)
        if array.ndim != 2 or array.shape[1] != 2 or array.shape[0] != sample_count:
            raise ValueError(
                f"coordinate sample count/shape mismatch for {encoder}: "
                f"{array.shape} != ({sample_count}, 2)"
            )
        if not np.isfinite(array).all():
            raise ValueError(f"coordinates for {encoder} must be finite (no NaN/Inf)")
    per_class = Counter(sample.generator_index for sample in bundle.samples)
    expected_paths = tuple(sample.image_path for sample in bundle.samples)
    checks = {
        "sample_count": len(bundle.raw_image_paths) == sample_count,
        "raw_path_alignment": bundle.raw_image_paths == expected_paths,
        "unique_image_paths": len(set(bundle.raw_image_paths)) == sample_count,
        "class_count": len(class_ids) == len(set(class_ids)),
        "class_sample_coverage": set(per_class) == set(class_ids),
        "one_unique_color_per_class": len(set(colors)) == len(class_ids),
        "finite_coordinates": all(
            np.isfinite(np.asarray(value)).all() for value in coordinates_by_encoder.values()
        ),
        "coordinate_count": all(
            np.asarray(value).shape == (sample_count, 2)
            for value in coordinates_by_encoder.values()
        ),
    }
    audit = {
        "pass": all(checks.values()),
        "checks": checks,
        "counts": {
            "sample_count": sample_count,
            "class_count": len(class_ids),
            "color_count": len(colors),
            "unique_color_count": len(set(colors)),
            "encoder_count": len(coordinates_by_encoder),
            "samples_per_class": sorted(set(per_class.values())),
        },
    }
    if not audit["pass"]:
        failed = [key for key, passed in checks.items() if not passed]
        raise ValueError(f"t-SNE output audit failed: {failed}")
    return audit


def _sample_rows(
    bundle: DatasetBundle,
    color_map: Mapping[str, str],
    coordinates: np.ndarray | None = None,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for raw_index, sample in enumerate(bundle.samples):
        row: dict[str, Any] = {
            "raw_index": raw_index,
            "ordinal": sample.ordinal,
            "render_key": sample.render_key,
            "generator_index": sample.generator_index,
            "generator_name": sample.generator_name,
            "sample_index": sample.sample_index,
            "source_type": sample.source_type,
            "picture_category": sample.picture_category,
            "asset_id": sample.asset_id,
            "seed": sample.seed,
            "color_hex": color_map[sample.generator_index],
            "image_path": str(sample.image_path),
            "png_bytes": sample.png_bytes,
            "png_sha256": sample.png_sha256,
        }
        if coordinates is not None:
            row["tsne_x"] = format(float(coordinates[raw_index, 0]), ".9g")
            row["tsne_y"] = format(float(coordinates[raw_index, 1]), ".9g")
        rows.append(row)
    return rows


def _centroids_2d(
    bundle: DatasetBundle,
    coordinates: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    class_to_index = {
        record.generator_index: index for index, record in enumerate(bundle.records)
    }
    sample_class_indices = np.asarray(
        [class_to_index[sample.generator_index] for sample in bundle.samples], dtype=np.int32
    )
    centers = np.empty((len(bundle.records), 2), dtype=np.float32)
    for class_index in range(len(bundle.records)):
        centers[class_index] = coordinates[sample_class_indices == class_index].mean(axis=0)
    return centers, sample_class_indices


def _centroid_rows(
    bundle: DatasetBundle,
    color_map: Mapping[str, str],
    centers: np.ndarray,
) -> list[dict[str, Any]]:
    return [
        {
            "generator_index": record.generator_index,
            "generator_name": record.generator_name,
            "source_type": record.source_type,
            "picture_category": record.picture_category,
            "sample_count": len(record.image_paths),
            "color_hex": color_map[record.generator_index],
            "centroid_tsne_x": format(float(center[0]), ".9g"),
            "centroid_tsne_y": format(float(center[1]), ".9g"),
        }
        for record, center in zip(bundle.records, centers, strict=True)
    ]


def _style_axis(axis: Any, *, title: str) -> None:
    axis.set_title(title, fontsize=15, pad=11)
    axis.set_xlabel("t-SNE 1")
    axis.set_ylabel("t-SNE 2")
    axis.spines[["top", "right"]].set_visible(False)
    axis.grid(color="#d9d9d9", linewidth=0.45, alpha=0.50)
    axis.set_axisbelow(True)


def _draw_sample_axis(
    axis: Any,
    coordinates: np.ndarray,
    centers: np.ndarray,
    sample_class_indices: np.ndarray,
    class_colors: Sequence[str],
    *,
    title: str,
) -> None:
    from matplotlib.collections import LineCollection

    sample_colors = [class_colors[index] for index in sample_class_indices]
    segments = np.stack((centers[sample_class_indices], coordinates), axis=1)
    axis.add_collection(
        LineCollection(
            segments,
            colors=sample_colors,
            linewidths=0.25,
            alpha=0.12,
            rasterized=True,
            zorder=1,
        )
    )
    axis.scatter(
        coordinates[:, 0],
        coordinates[:, 1],
        c=sample_colors,
        s=12,
        alpha=0.55,
        edgecolors="none",
        rasterized=True,
        zorder=2,
    )
    axis.scatter(
        centers[:, 0],
        centers[:, 1],
        c=list(class_colors),
        s=26,
        alpha=1.0,
        edgecolors="#1d1d1d",
        linewidths=0.25,
        rasterized=True,
        zorder=3,
    )
    _style_axis(axis, title=title)


def _draw_centroid_axis(
    axis: Any,
    centers: np.ndarray,
    class_colors: Sequence[str],
    *,
    title: str,
) -> None:
    axis.scatter(
        centers[:, 0], centers[:, 1], c=list(class_colors), s=28, alpha=1.0,
        edgecolors="#1d1d1d", linewidths=0.2, rasterized=True,
    )
    _style_axis(axis, title=title)


def _save_figure(figure: Any, output_path: Path, *, dpi: int) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(f".{output_path.name}.tmp")
    figure.savefig(temporary, format="png", dpi=dpi, facecolor="white")
    temporary.replace(output_path)


def _save_encoder_plot(
    coordinates: np.ndarray,
    centers: np.ndarray,
    sample_class_indices: np.ndarray,
    class_colors: Sequence[str],
    output_path: Path,
    *,
    encoder_label: str,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    figure, axis = plt.subplots(figsize=(14, 10), dpi=dpi)
    figure.subplots_adjust(left=0.07, right=0.985, top=0.91, bottom=0.09)
    _draw_sample_axis(
        axis, coordinates, centers, sample_class_indices, class_colors,
        title=f"{encoder_label}: {len(coordinates):,} rendered assets",
    )
    handles = [
        Line2D([], [], linestyle="none", marker="o", markersize=4,
               markerfacecolor="#777777", markeredgecolor="none", alpha=0.6,
               label="Rendered asset"),
        Line2D([], [], linestyle="none", marker="o", markersize=6,
               markerfacecolor="#777777", markeredgecolor="#1d1d1d",
               markeredgewidth=0.4, label="2D class centroid"),
        Line2D([], [], color="#777777", linewidth=0.5, alpha=0.5,
               label="Centroid guide"),
    ]
    axis.legend(handles=handles, loc="best", frameon=True, fontsize=8)
    _save_figure(figure, output_path, dpi=dpi)
    plt.close(figure)


def _save_comparison_plot(
    bundle: DatasetBundle,
    coordinates_by_encoder: Mapping[str, np.ndarray],
    centers_by_encoder: Mapping[str, np.ndarray],
    sample_class_indices: np.ndarray,
    class_colors: Sequence[str],
    encoder_labels: Mapping[str, str],
    output_path: Path,
    *,
    dpi: int,
    centroids_only: bool,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    encoder_keys = list(coordinates_by_encoder)
    figure, axes = plt.subplots(1, len(encoder_keys), figsize=(24, 10), dpi=dpi)
    axes = np.atleast_1d(axes)
    figure.subplots_adjust(left=0.045, right=0.985, top=0.89, bottom=0.08, wspace=0.15)
    if centroids_only:
        figure.suptitle(
            f"PV-A class centroids within the {len(bundle.samples):,}-sample t-SNE "
            f"({len(bundle.records)} classes)",
            fontsize=19, y=0.965,
        )
    else:
        figure.suptitle(
            f"PV-A uniform renders: {len(bundle.samples)} assets, "
            f"{len(bundle.records)} classes, {bundle.summary['per_class']} assets per class",
            fontsize=19, y=0.965,
        )
    for axis, key in zip(axes, encoder_keys, strict=True):
        if centroids_only:
            _draw_centroid_axis(
                axis, centers_by_encoder[key], class_colors, title=encoder_labels[key]
            )
        else:
            _draw_sample_axis(
                axis, coordinates_by_encoder[key], centers_by_encoder[key],
                sample_class_indices, class_colors, title=encoder_labels[key],
            )
    figure.text(
        0.5, 0.018,
        "The same PV-A generator class uses the same frozen color in both independent embeddings.",
        ha="center", fontsize=8, color="#4a4a4a",
    )
    _save_figure(figure, output_path, dpi=dpi)
    plt.close(figure)


def _save_source_plot(
    bundle: DatasetBundle,
    coordinates_by_encoder: Mapping[str, np.ndarray],
    encoder_labels: Mapping[str, str],
    output_path: Path,
    *,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    source_colors = {
        "picture_backed": "#2a78a6",
        "builtin_no_picture": "#d05a3a",
    }
    colors = [source_colors[sample.source_type] for sample in bundle.samples]
    keys = list(coordinates_by_encoder)
    figure, axes = plt.subplots(1, len(keys), figsize=(24, 10), dpi=dpi)
    axes = np.atleast_1d(axes)
    figure.subplots_adjust(left=0.045, right=0.985, top=0.89, bottom=0.09, wspace=0.15)
    figure.suptitle("PV-A uniform-render t-SNE by generator source", fontsize=19, y=0.965)
    for axis, key in zip(axes, keys, strict=True):
        coordinates = coordinates_by_encoder[key]
        axis.scatter(
            coordinates[:, 0], coordinates[:, 1], c=colors, s=13, alpha=0.62,
            edgecolors="none", rasterized=True,
        )
        _style_axis(axis, title=encoder_labels[key])
    handles = [
        Line2D([], [], linestyle="none", marker="o", markersize=6,
               markerfacecolor=color, markeredgecolor="none", label=source)
        for source, color in source_colors.items()
    ]
    axes[-1].legend(handles=handles, loc="best", frameon=True, fontsize=8)
    _save_figure(figure, output_path, dpi=dpi)
    plt.close(figure)


def _save_color_key(
    records: Sequence[GeneratorRecord],
    color_map: Mapping[str, str],
    output_path: Path,
    *,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    columns = 6
    rows = math.ceil(len(records) / columns)
    figure = plt.figure(figsize=(24, 18), dpi=dpi, facecolor="white")
    figure.text(
        0.025, 0.985, f"PV-A generator class color key (n={len(records)})",
        ha="left", va="top", fontsize=15,
    )
    x_start, x_end, y_top, y_bottom = 0.025, 0.985, 0.958, 0.025
    column_width = (x_end - x_start) / columns
    y_step = (y_top - y_bottom) / max(1, rows - 1)
    for index, record in enumerate(records):
        column, row = divmod(index, rows)
        x = x_start + column * column_width
        y = y_top - row * y_step
        color = color_map[record.generator_index]
        figure.add_artist(
            Rectangle(
                (x, y - 0.0038), 0.008, 0.0076, transform=figure.transFigure,
                facecolor=color, edgecolor="#202020", linewidth=0.25,
            )
        )
        figure.text(
            x + 0.011, y, f"{record.generator_index}  {record.generator_name}",
            ha="left", va="center", fontsize=4.5, color="#252525",
        )
    _save_figure(figure, output_path, dpi=dpi)
    plt.close(figure)


def _audit_written_outputs(
    bundle: DatasetBundle,
    color_map: Mapping[str, str],
    coordinates_by_encoder: Mapping[str, np.ndarray],
    output_dir: Path,
) -> dict[str, Any]:
    """Reload every numerical/table artifact and verify cross-file semantics."""
    audit = audit_tsne_outputs(bundle, color_map, coordinates_by_encoder)
    sample_rows = _read_csv(output_dir / "render_sample_index.csv")
    expected_keys = [sample.render_key for sample in bundle.samples]
    expected_colors = [color_map[sample.generator_index] for sample in bundle.samples]
    audit["checks"].update(
        {
            "sample_index_csv_rows": len(sample_rows) == len(bundle.samples),
            "sample_index_csv_alignment": [row.get("render_key") for row in sample_rows]
            == expected_keys,
            "sample_index_csv_colors": [row.get("color_hex") for row in sample_rows]
            == expected_colors,
        }
    )

    class_indices = {
        record.generator_index: np.asarray(
            [
                index
                for index, sample in enumerate(bundle.samples)
                if sample.generator_index == record.generator_index
            ],
            dtype=np.int64,
        )
        for record in bundle.records
    }
    model_details: dict[str, Any] = {}
    for encoder, in_memory_coordinates in coordinates_by_encoder.items():
        model_dir = output_dir / encoder
        raw_features = np.load(model_dir / "raw_image_features.npy", allow_pickle=False)
        class_features = np.load(model_dir / "class_features.npy", allow_pickle=False)
        coordinates = np.load(model_dir / "image_tsne_coordinates.npy", allow_pickle=False)
        centers = np.load(model_dir / "class_centroids_2d.npy", allow_pickle=False)
        coordinate_rows = _read_csv(model_dir / "image_tsne_coordinates.csv")
        centroid_rows = _read_csv(model_dir / "class_centroids_2d.csv")
        feature_manifest = _load_json(model_dir / "feature_manifest.json")

        raw_norms = (
            np.linalg.norm(raw_features, axis=1)
            if raw_features.ndim == 2 and raw_features.shape[0] == len(bundle.samples)
            else np.asarray([], dtype=np.float32)
        )
        recomputed_class_features: list[np.ndarray] = []
        recomputed_centers: list[np.ndarray] = []
        if raw_norms.size and coordinates.shape == (len(bundle.samples), 2):
            for record in bundle.records:
                indices = class_indices[record.generator_index]
                mean = raw_features[indices].mean(axis=0)
                mean_norm = float(np.linalg.norm(mean))
                if mean_norm <= 0:
                    raise ValueError(f"zero class feature mean for {encoder}/{record.generator_index}")
                recomputed_class_features.append(mean / mean_norm)
                recomputed_centers.append(coordinates[indices].mean(axis=0))
        recomputed_class_array = (
            np.stack(recomputed_class_features).astype(np.float32, copy=False)
            if recomputed_class_features
            else np.empty((0, 0), dtype=np.float32)
        )
        recomputed_center_array = (
            np.stack(recomputed_centers).astype(np.float32, copy=False)
            if recomputed_centers
            else np.empty((0, 2), dtype=np.float32)
        )
        csv_coordinates = np.asarray(
            [
                [float(row.get("tsne_x") or "nan"), float(row.get("tsne_y") or "nan")]
                for row in coordinate_rows
            ],
            dtype=np.float32,
        )
        csv_centers = np.asarray(
            [
                [
                    float(row.get("centroid_tsne_x") or "nan"),
                    float(row.get("centroid_tsne_y") or "nan"),
                ]
                for row in centroid_rows
            ],
            dtype=np.float32,
        )
        receipt = feature_manifest.get("raw_feature_receipt")
        prefix = f"{encoder}_"
        encoder_checks = {
            prefix + "raw_feature_shape": raw_features.ndim == 2
            and raw_features.shape[0] == len(bundle.samples),
            prefix + "raw_feature_dtype": raw_features.dtype == np.float32,
            prefix + "raw_features_finite": bool(np.isfinite(raw_features).all()),
            prefix + "raw_features_l2_normalized": raw_norms.size > 0
            and bool(np.allclose(raw_norms, 1.0, rtol=1e-5, atol=1e-6)),
            prefix + "class_feature_shape": class_features.shape
            == (len(bundle.records), raw_features.shape[1]),
            prefix + "class_features_finite": bool(np.isfinite(class_features).all()),
            prefix + "class_feature_recomputation": class_features.shape
            == recomputed_class_array.shape
            and bool(np.allclose(class_features, recomputed_class_array, rtol=1e-5, atol=1e-6)),
            prefix + "coordinate_npy_alignment": coordinates.dtype == np.float32
            and np.array_equal(coordinates, np.asarray(in_memory_coordinates)),
            prefix + "coordinate_csv_rows": len(coordinate_rows) == len(bundle.samples),
            prefix + "coordinate_csv_identity": [row.get("render_key") for row in coordinate_rows]
            == expected_keys,
            prefix + "coordinate_csv_values": csv_coordinates.shape == coordinates.shape
            and bool(np.allclose(csv_coordinates, coordinates, rtol=1e-7, atol=1e-5)),
            prefix + "centroid_npy_shape": centers.shape == (len(bundle.records), 2),
            prefix + "centroid_recomputation": centers.shape == recomputed_center_array.shape
            and bool(np.allclose(centers, recomputed_center_array, rtol=1e-6, atol=1e-6)),
            prefix + "centroid_csv_rows": len(centroid_rows) == len(bundle.records),
            prefix + "centroid_csv_identity": [row.get("generator_index") for row in centroid_rows]
            == [record.generator_index for record in bundle.records],
            prefix + "centroid_csv_values": csv_centers.shape == centers.shape
            and bool(np.allclose(csv_centers, centers, rtol=1e-7, atol=1e-5)),
            prefix + "feature_cache_receipt": isinstance(receipt, dict)
            and receipt.get("sha256") == sha256_file(model_dir / "raw_image_features.npy")
            and receipt.get("size_bytes") == (model_dir / "raw_image_features.npy").stat().st_size,
        }
        audit["checks"].update(encoder_checks)
        model_details[encoder] = {
            "raw_feature_shape": list(raw_features.shape),
            "class_feature_shape": list(class_features.shape),
            "coordinate_shape": list(coordinates.shape),
            "centroid_shape": list(centers.shape),
            "raw_feature_l2_norm_min": float(raw_norms.min()) if raw_norms.size else None,
            "raw_feature_l2_norm_max": float(raw_norms.max()) if raw_norms.size else None,
            "coordinate_csv_max_abs_error": (
                float(np.max(np.abs(csv_coordinates - coordinates)))
                if csv_coordinates.shape == coordinates.shape else None
            ),
        }
    audit["model_details"] = model_details
    audit["pass"] = all(audit["checks"].values())
    if not audit["pass"]:
        failed = [key for key, passed in audit["checks"].items() if not passed]
        raise ValueError(f"written output semantic audit failed: {failed}")
    return audit


def _runtime_environment() -> dict[str, Any]:
    import platform

    import PIL
    import sklearn
    import torch
    import transformers

    cuda_devices = [
        torch.cuda.get_device_name(index) for index in range(torch.cuda.device_count())
    ] if torch.cuda.is_available() else []
    return {
        "python": platform.python_version(),
        "numpy": np.__version__,
        "pillow": PIL.__version__,
        "scikit_learn": sklearn.__version__,
        "torch": torch.__version__,
        "transformers": transformers.__version__,
        "cuda_runtime": torch.version.cuda,
        "cuda_available": torch.cuda.is_available(),
        "cuda_devices": cuda_devices,
        "torch_deterministic_algorithms": torch.are_deterministic_algorithms_enabled(),
    }


def _write_release_receipt(output_dir: Path, paths: Sequence[Path]) -> dict[str, Any]:
    receipts: dict[str, dict[str, Any]] = {}
    digest = hashlib.sha256()
    for path in sorted({path.resolve(strict=True) for path in paths}):
        relative = str(path.relative_to(output_dir))
        receipt = _receipt(path)
        receipts[relative] = receipt
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(receipt["sha256"].encode("ascii"))
        digest.update(b"\0")
        digest.update(str(receipt["size_bytes"]).encode("ascii"))
        digest.update(b"\n")
    payload = {
        "schema_version": 1,
        "artifact_count": len(receipts),
        "aggregate_sha256": digest.hexdigest(),
        "artifacts": receipts,
    }
    _atomic_write_json(output_dir / "release_receipt.json", payload)
    return payload


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.plot_dpi < 40:
        raise ValueError("plot_dpi must be at least 40")
    if not args.models or len(args.models) != len(set(args.models)):
        raise ValueError("models must be a non-empty list without duplicates")
    bundle = discover_n5_render_records(
        args.index_csv,
        render_root=args.render_root,
        strict_counts=not args.allow_count_drift,
        expected_per_class=args.expected_per_class,
    )
    color_index_csv = args.color_index_csv.expanduser().resolve(strict=True)
    color_map = build_class_color_map(bundle.records, color_index_csv=color_index_csv)
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    _atomic_write_json(output_dir / "dataset_manifest.json", bundle.summary)
    shutil.copy2(color_index_csv, output_dir / "generator_class_color_index.csv")
    sample_index_rows = _sample_rows(bundle, color_map)
    _atomic_write_csv(
        output_dir / "render_sample_index.csv", list(sample_index_rows[0]), sample_index_rows
    )
    _save_color_key(
        bundle.records, color_map, output_dir / "generator_class_color_key.png",
        dpi=args.plot_dpi,
    )

    helper = _helper()
    model_paths = {"dinov2": args.dino_model, "clip": args.clip_model}
    coordinates_by_encoder: dict[str, np.ndarray] = {}
    centers_by_encoder: dict[str, np.ndarray] = {}
    encoder_labels: dict[str, str] = {}
    model_summaries: dict[str, Any] = {}
    sample_class_indices: np.ndarray | None = None
    for model_name in args.models:
        model_path = model_paths[model_name].expanduser().resolve(strict=True)
        model_dir = output_dir / model_name
        strict_cache_contract = {
            "schema_version": 1,
            "model_name": model_name,
            "model_path": str(model_path),
            "feature_helper_sha256": sha256_file(HELPER_SCRIPT),
            "requested_device": args.device,
            "requested_batch_size": args.batch_size,
            "requested_use_amp": bool(args.use_amp),
        }
        strict_cache_valid = _strict_feature_cache_valid(
            model_dir, strict_cache_contract
        )
        effective_force_extract = bool(args.force_extract or not strict_cache_valid)
        print(
            f"[run] {model_name}: processing {len(bundle.samples)} images "
            f"({len(bundle.records)} classes x {args.expected_per_class})",
            flush=True,
        )
        raw_features, class_features, extraction, reused = helper.load_or_extract_features(
            bundle,
            model_name=model_name,
            model_path=model_path,
            output_dir=output_dir,
            batch_size=args.batch_size,
            device=args.device,
            num_workers=args.num_workers,
            use_amp=args.use_amp,
            force_extract=effective_force_extract,
        )
        if extraction.get("model_type") != model_name:
            raise ValueError(
                f"requested {model_name}, but model reports "
                f"model_type={extraction.get('model_type')!r}"
            )
        raw_feature_receipt = _seal_feature_cache(model_dir, strict_cache_contract)
        coordinates, tsne_info = helper.compute_tsne(
            raw_features,
            perplexity=args.perplexity,
            random_state=args.seed,
            max_iter=args.tsne_max_iter,
            n_jobs=args.tsne_jobs,
            verbose=args.tsne_verbose,
        )
        centers, indices = _centroids_2d(bundle, coordinates)
        if sample_class_indices is None:
            sample_class_indices = indices
        elif not np.array_equal(sample_class_indices, indices):
            raise RuntimeError("sample/class alignment changed between encoders")
        _atomic_save_npy(model_dir / "image_tsne_coordinates.npy", coordinates)
        _atomic_save_npy(model_dir / "class_centroids_2d.npy", centers)
        coordinate_rows = _sample_rows(bundle, color_map, coordinates)
        _atomic_write_csv(
            model_dir / "image_tsne_coordinates.csv",
            list(coordinate_rows[0]),
            coordinate_rows,
        )
        centroid_rows = _centroid_rows(bundle, color_map, centers)
        _atomic_write_csv(
            model_dir / "class_centroids_2d.csv", list(centroid_rows[0]), centroid_rows
        )
        encoder_label = (
            "DINOv2-base" if model_name == "dinov2"
            else str(extraction.get("encoder_label") or "CLIP ViT-B/32")
        )
        _save_encoder_plot(
            coordinates, centers, indices,
            [color_map[record.generator_index] for record in bundle.records],
            model_dir / "tsne_by_generator_class.png",
            encoder_label=encoder_label,
            dpi=args.plot_dpi,
        )
        coordinates_by_encoder[model_name] = coordinates
        centers_by_encoder[model_name] = centers
        encoder_labels[model_name] = encoder_label
        model_summaries[model_name] = {
            "encoder_label": encoder_label,
            "model_path": str(model_path),
            "feature_cache_reused": reused,
            "strict_feature_cache_was_valid": strict_cache_valid,
            "effective_force_extract": effective_force_extract,
            "raw_feature_receipt": raw_feature_receipt,
            "raw_feature_shape": list(raw_features.shape),
            "class_feature_shape": list(class_features.shape),
            "feature_extraction": extraction,
            "tsne": tsne_info,
            "artifacts": {
                "raw_image_features": f"{model_name}/raw_image_features.npy",
                "class_features": f"{model_name}/class_features.npy",
                "feature_manifest": f"{model_name}/feature_manifest.json",
                "image_tsne_coordinates": f"{model_name}/image_tsne_coordinates.npy",
                "image_tsne_coordinates_csv": f"{model_name}/image_tsne_coordinates.csv",
                "class_centroids_2d": f"{model_name}/class_centroids_2d.npy",
                "class_centroids_2d_csv": f"{model_name}/class_centroids_2d.csv",
                "plot": f"{model_name}/tsne_by_generator_class.png",
            },
        }
        del raw_features, class_features

    assert sample_class_indices is not None
    class_colors = [color_map[record.generator_index] for record in bundle.records]
    _save_comparison_plot(
        bundle, coordinates_by_encoder, centers_by_encoder, sample_class_indices,
        class_colors, encoder_labels, output_dir / "tsne_generator_class_comparison.png",
        dpi=args.plot_dpi, centroids_only=False,
    )
    _save_comparison_plot(
        bundle, coordinates_by_encoder, centers_by_encoder, sample_class_indices,
        class_colors, encoder_labels, output_dir / "tsne_class_centroid_comparison.png",
        dpi=args.plot_dpi, centroids_only=True,
    )
    _save_source_plot(
        bundle, coordinates_by_encoder, encoder_labels,
        output_dir / "tsne_source_comparison.png", dpi=args.plot_dpi,
    )

    audit = _audit_written_outputs(
        bundle, color_map, coordinates_by_encoder, output_dir
    )
    required_artifacts = [
        output_dir / "dataset_manifest.json",
        output_dir / "render_sample_index.csv",
        output_dir / "generator_class_color_index.csv",
        output_dir / "generator_class_color_key.png",
        output_dir / "tsne_generator_class_comparison.png",
        output_dir / "tsne_class_centroid_comparison.png",
        output_dir / "tsne_source_comparison.png",
    ]
    for model_name in args.models:
        required_artifacts.extend(
            [
                output_dir / model_name / "raw_image_features.npy",
                output_dir / model_name / "class_features.npy",
                output_dir / model_name / "feature_manifest.json",
                output_dir / model_name / "image_tsne_coordinates.npy",
                output_dir / model_name / "image_tsne_coordinates.csv",
                output_dir / model_name / "class_centroids_2d.npy",
                output_dir / model_name / "class_centroids_2d.csv",
                output_dir / model_name / "tsne_by_generator_class.png",
            ]
        )
    audit["checks"]["required_artifacts_exist"] = all(
        path.is_file() and path.stat().st_size > 0 for path in required_artifacts
    )
    audit["pass"] = all(audit["checks"].values())
    audit["artifact_receipts"] = {
        str(path.relative_to(output_dir)): _receipt(path) for path in required_artifacts
    }
    _atomic_write_json(output_dir / "audit.json", audit)
    if not audit["pass"]:
        raise RuntimeError("on-disk t-SNE artifact audit failed")

    manifest = {
        "schema_version": 2,
        "dataset": bundle.summary,
        "models": model_summaries,
        "protocol": {
            "sample_unit": "one uniformly rendered PV-A asset",
            "samples_per_class": args.expected_per_class,
            "color_unit": "one frozen unique RGB value per generator_index",
            "encoder_policy": "DINOv2 and CLIP fitted independently",
            "projection": "L2-normalized feature -> PCA(50) -> 2D t-SNE",
            "coordinate_policy": "raw t-SNE coordinates; no jitter or collision separation",
            "centroid_policy": (
                f"arithmetic mean of each class's {args.expected_per_class} "
                "2D sample coordinates"
            ),
        },
        "interpretation_limits": [
            f"All {len(color_map)} RGB values are unique, but they are not all "
            "perceptually distinguishable.",
            "Colors identify classes; they do not encode semantic similarity.",
            "Distances and axes are not comparable across the independently fitted encoder panels.",
            "t-SNE preserves local neighborhoods imperfectly and does not establish dataset quality alone.",
        ],
        "color_index_source": _receipt(color_index_csv),
        "implementation": {
            "script": _receipt(SCRIPT),
            "feature_helper": _receipt(HELPER_SCRIPT),
        },
        "runtime_environment": _runtime_environment(),
        "config": {
            "output_dir": str(output_dir),
            "device": args.device,
            "batch_size": args.batch_size,
            "num_workers": args.num_workers,
            "use_amp": args.use_amp,
            "perplexity": args.perplexity,
            "seed": args.seed,
            "tsne_max_iter": args.tsne_max_iter,
            "tsne_jobs": args.tsne_jobs,
            "tsne_verbose": args.tsne_verbose,
            "plot_dpi": args.plot_dpi,
            "force_extract": args.force_extract,
        },
        "artifacts": {
            "audit": "audit.json",
            "dataset_manifest": "dataset_manifest.json",
            "render_sample_index": "render_sample_index.csv",
            "class_color_index": "generator_class_color_index.csv",
            "class_color_key": "generator_class_color_key.png",
            "sample_comparison": "tsne_generator_class_comparison.png",
            "centroid_comparison": "tsne_class_centroid_comparison.png",
            "source_comparison": "tsne_source_comparison.png",
            "release_receipt": "release_receipt.json",
        },
    }
    run_manifest_path = output_dir / "run_manifest.json"
    audit_path = output_dir / "audit.json"
    _atomic_write_json(run_manifest_path, manifest)
    _write_release_receipt(
        output_dir,
        [*required_artifacts, audit_path, run_manifest_path],
    )
    print(
        f"[done] {len(bundle.samples)} samples, {len(bundle.records)} classes, "
        f"{len(set(color_map.values()))} unique class colors -> {output_dir}",
        flush=True,
    )
    return manifest


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="DINOv2/CLIP image-level t-SNE for PV-A's 531 x 5 uniform renders."
    )
    parser.add_argument("--index-csv", type=Path, default=DEFAULT_INDEX_CSV)
    parser.add_argument("--render-root", type=Path, default=DEFAULT_RENDER_ROOT)
    parser.add_argument("--color-index-csv", type=Path, default=DEFAULT_COLOR_INDEX)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--dino-model", type=Path, default=DEFAULT_DINO_MODEL)
    parser.add_argument("--clip-model", type=Path, default=DEFAULT_CLIP_MODEL)
    parser.add_argument(
        "--models", nargs="+", choices=("dinov2", "clip"),
        default=("dinov2", "clip"),
    )
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--num-workers", type=int, default=8)
    parser.add_argument("--no-amp", action="store_false", dest="use_amp")
    parser.set_defaults(use_amp=True)
    parser.add_argument("--perplexity", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--tsne-max-iter", type=int, default=1_000)
    parser.add_argument("--tsne-jobs", type=int, default=min(16, os.cpu_count() or 1))
    parser.add_argument("--tsne-verbose", type=int, choices=(0, 1, 2), default=1)
    parser.add_argument("--plot-dpi", type=int, default=180)
    parser.add_argument("--expected-per-class", type=int, default=EXPECTED_PER_CLASS)
    parser.add_argument("--force-extract", action="store_true")
    parser.add_argument("--allow-count-drift", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    manifest = run(args)
    print(json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
