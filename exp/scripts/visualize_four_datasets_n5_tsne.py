#!/usr/bin/env python3
"""Audited five-shot DINOv2/CLIP comparison for four articulated datasets.

The four Blender cohorts use one frozen studio contract.  Every available
class is shown in the all-class projections.  Quantitative comparisons and
the joint projection use only classes with five distinct rendered assets, so
short categories are never padded with duplicate images.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import os
import re
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

for _variable in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_variable, "1")

import numpy as np
from PIL import Image, __version__ as PILLOW_VERSION


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
FEATURE_HELPER = SCRIPT.with_name("visualize_pva531_picture_tsne.py")
PALETTE_HELPER = SCRIPT.with_name("plot_pva531_generator_colors.py")
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exp/pva_artiverse_articraft_partnet_n5_tsne"
DEFAULT_PVA_FEATURE_DIR = REPO_ROOT / "exp/pva531_n5_tsne"
DEFAULT_PVA_COLOR_INDEX = (
    REPO_ROOT / "exp/runtime/pva531_uniform_tsne/generator_class_color_index.csv"
)
DEFAULT_DINO_MODEL = Path(
    "/root/.cache/huggingface/hub/models--facebook--dinov2-base/"
    "snapshots/f9e44c814b77203eaa57a6bdbbd535f21ede1415"
)
DEFAULT_CLIP_MODEL = Path(
    "/root/.cache/huggingface/hub/models--openai--clip-vit-base-patch32/"
    "snapshots/3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"
)
SUCCESS_STATUSES = {"rendered", "reused_valid"}
ENCODERS = ("dinov2", "clip")
ENCODER_LABELS = {"dinov2": "DINOv2-base", "clip": "CLIP ViT-B/32"}
DATASET_COLORS = {
    "pva": "#246b8e",
    "artiverse": "#c84c3a",
    "articraft10k": "#668b3c",
    "partnet_mobility": "#d19a27",
}
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


@dataclass(frozen=True, slots=True)
class DatasetSpec:
    key: str
    name: str
    root: Path
    expected_classes: int
    expected_samples: int
    expected_strict_classes: int


@dataclass(frozen=True, slots=True)
class Sample:
    dataset_key: str
    dataset_name: str
    ordinal: int
    render_key: str
    class_id: str
    class_name: str
    sample_index: int
    asset_id: str
    source_type: str
    image_path: Path
    png_bytes: int
    png_sha256: str

    @property
    def class_key(self) -> str:
        return f"{self.dataset_key}::{self.class_id}"


@dataclass(frozen=True, slots=True)
class Cohort:
    spec: DatasetSpec
    samples: tuple[Sample, ...]
    class_order: tuple[str, ...]
    class_names: Mapping[str, str]
    class_counts: Mapping[str, int]
    config: Mapping[str, Any]
    summary: Mapping[str, Any]
    receipts: Mapping[str, Mapping[str, Any]]

    @property
    def strict_class_ids(self) -> tuple[str, ...]:
        return tuple(class_id for class_id in self.class_order if self.class_counts[class_id] == 5)

    @property
    def strict_samples(self) -> tuple[Sample, ...]:
        eligible = set(self.strict_class_ids)
        return tuple(sample for sample in self.samples if sample.class_id in eligible)


_MODULE_CACHE: dict[str, Any] = {}


def _load_module(path: Path, name: str) -> Any:
    cached = _MODULE_CACHE.get(name)
    if cached is not None:
        return cached
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import helper: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    _MODULE_CACHE[name] = module
    return module


def _feature_helper() -> Any:
    return _load_module(FEATURE_HELPER, "_four_n5_feature_helper")


def _palette_helper() -> Any:
    return _load_module(PALETTE_HELPER, "_four_n5_palette_helper")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def file_receipt(path: Path) -> dict[str, Any]:
    path = path.expanduser().resolve(strict=True)
    return {"path": str(path), "size_bytes": path.stat().st_size, "sha256": sha256_file(path)}


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"CSV has no rows: {path}")
    return rows


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
    fields = list(rows[0])
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def atomic_npy(path: Path, value: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("wb") as stream:
        np.save(stream, np.asarray(value), allow_pickle=False)
    temporary.replace(path)


def _parse_int(value: Any, field: str) -> int:
    try:
        return int(str(value))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid integer {field}: {value!r}") from exc


def _nonempty(row: Mapping[str, Any], *fields: str) -> str:
    for field in fields:
        value = str(row.get(field) or "").strip()
        if value:
            return value
    raise ValueError(f"none of {fields!r} is populated")


def _resolve_inside(root: Path, value: Any, field: str) -> Path:
    candidate = Path(str(value)).expanduser()
    if not candidate.is_absolute():
        candidate = root / candidate
    candidate = candidate.resolve(strict=True)
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{field} escapes render root {root}: {candidate}") from exc
    return candidate


def _validate_png(path: Path, expected_bytes: int, expected_sha256: str, resolution: int) -> None:
    if path.stat().st_size != expected_bytes:
        raise ValueError(f"PNG byte receipt mismatch: {path}")
    with path.open("rb") as stream:
        if stream.read(8) != PNG_SIGNATURE:
            raise ValueError(f"invalid PNG signature: {path}")
    if sha256_file(path) != expected_sha256:
        raise ValueError(f"PNG SHA-256 receipt mismatch: {path}")
    with Image.open(path) as image:
        image.verify()
        if image.size != (resolution, resolution):
            raise ValueError(f"PNG dimensions differ from render contract: {path} {image.size}")


def _studio_contract(config: Mapping[str, Any]) -> dict[str, Any]:
    shared_renderer = str(config.get("shared_renderer_sha256") or "").strip()
    renderer = shared_renderer or str(config.get("renderer_sha256") or "").strip()
    return {
        "resolution": _parse_int(config.get("resolution"), "resolution"),
        "cycles_samples": _parse_int(config.get("samples"), "samples"),
        "studio": config.get("studio"),
        "effective_shared_renderer_sha256": renderer,
        "blender_version": str(config.get("blender_version") or ""),
    }


def load_cohort(spec: DatasetSpec, *, verify_pngs: bool = True) -> Cohort:
    root = spec.root.expanduser().resolve(strict=True)
    config_path = root / "render_config.json"
    roster_path = root / "render_roster.csv"
    manifest_path = root / "render_manifest.csv"
    summary_path = root / "render_summary.json"
    config = read_json(config_path)
    summary = read_json(summary_path)
    if _parse_int(summary.get("failure_count", -1), "failure_count") != 0:
        raise ValueError(f"render summary contains failures: {summary_path}")

    roster_rows = read_csv(roster_path)
    manifest_rows = read_csv(manifest_path)
    manifest_by_key: dict[str, dict[str, str]] = {}
    for row in manifest_rows:
        key = _nonempty(row, "render_key")
        if key in manifest_by_key:
            raise ValueError(f"duplicate render_key in {manifest_path}: {key}")
        manifest_by_key[key] = row
    ordered_roster = sorted(roster_rows, key=lambda row: _parse_int(row.get("ordinal"), "ordinal"))
    if [_parse_int(row.get("ordinal"), "ordinal") for row in ordered_roster] != list(
        range(1, len(ordered_roster) + 1)
    ):
        raise ValueError(f"roster ordinals are not contiguous: {roster_path}")

    samples: list[Sample] = []
    for row in ordered_roster:
        render_key = _nonempty(row, "render_key")
        result = manifest_by_key.get(render_key)
        if result is None:
            raise ValueError(f"render manifest is missing {render_key}")
        if str(result.get("status") or "") not in SUCCESS_STATUSES:
            raise ValueError(f"unsuccessful render {render_key}: {result.get('status')!r}")
        for identity_field in (
            "ordinal", "asset_id", "sample_index", "generator_index", "generator_name"
        ):
            if str(row.get(identity_field) or "") != str(result.get(identity_field) or ""):
                raise ValueError(f"roster/manifest {identity_field} mismatch for {render_key}")
        for receipt_field in ("output_path", "png_bytes", "png_sha256"):
            roster_value = str(row.get(receipt_field) or "").strip()
            manifest_value = str(result.get(receipt_field) or "").strip()
            if roster_value and manifest_value and roster_value != manifest_value:
                raise ValueError(f"roster/manifest {receipt_field} mismatch for {render_key}")

        output_value = result.get("output_path") or row.get("output_path")
        image_path = _resolve_inside(root, output_value, "output_path")
        png_bytes = _parse_int(result.get("png_bytes") or row.get("png_bytes"), "png_bytes")
        png_sha256 = str(result.get("png_sha256") or row.get("png_sha256") or "").strip()
        if SHA256_RE.fullmatch(png_sha256) is None:
            raise ValueError(f"invalid PNG SHA-256 for {render_key}")
        class_id = _nonempty(row, "generator_index", "class_id", "category")
        class_name = _nonempty(row, "generator_name", "category")
        if verify_pngs:
            _validate_png(image_path, png_bytes, png_sha256, int(config["resolution"]))
        samples.append(
            Sample(
                dataset_key=spec.key,
                dataset_name=spec.name,
                ordinal=_parse_int(row.get("ordinal"), "ordinal"),
                render_key=render_key,
                class_id=class_id,
                class_name=class_name,
                sample_index=_parse_int(row.get("sample_index"), "sample_index"),
                asset_id=_nonempty(row, "asset_id"),
                source_type=str(row.get("source_type") or "").strip(),
                image_path=image_path,
                png_bytes=png_bytes,
                png_sha256=png_sha256,
            )
        )

    if len(samples) != spec.expected_samples:
        raise ValueError(f"{spec.name}: {len(samples)} samples != {spec.expected_samples}")
    if len({sample.render_key for sample in samples}) != len(samples):
        raise ValueError(f"{spec.name}: duplicate render keys")
    if len({sample.image_path for sample in samples}) != len(samples):
        raise ValueError(f"{spec.name}: duplicate image paths")
    if len({(sample.class_id, sample.asset_id) for sample in samples}) != len(samples):
        raise ValueError(f"{spec.name}: duplicate asset within a class")

    class_order = tuple(dict.fromkeys(sample.class_id for sample in samples))
    class_names: dict[str, str] = {}
    class_counts = Counter(sample.class_id for sample in samples)
    for sample in samples:
        prior = class_names.setdefault(sample.class_id, sample.class_name)
        if prior != sample.class_name:
            raise ValueError(f"{spec.name}: class ID {sample.class_id} has multiple names")
    if len(class_order) != spec.expected_classes:
        raise ValueError(f"{spec.name}: {len(class_order)} classes != {spec.expected_classes}")
    if any(count < 1 or count > 5 for count in class_counts.values()):
        raise ValueError(f"{spec.name}: class sample counts must be in [1, 5]")
    for class_id in class_order:
        indices = sorted(sample.sample_index for sample in samples if sample.class_id == class_id)
        if indices != list(range(1, len(indices) + 1)):
            raise ValueError(f"{spec.name}/{class_id}: non-contiguous sample indices {indices}")
    strict_count = sum(count == 5 for count in class_counts.values())
    if strict_count != spec.expected_strict_classes:
        raise ValueError(
            f"{spec.name}: {strict_count} strict n=5 classes != {spec.expected_strict_classes}"
        )

    receipts = {
        "render_config": file_receipt(config_path),
        "render_roster": file_receipt(roster_path),
        "render_manifest": file_receipt(manifest_path),
        "render_summary": file_receipt(summary_path),
    }
    declared_roster_sha = str(config.get("render_roster_sha256") or "").strip()
    if declared_roster_sha and declared_roster_sha != receipts["render_roster"]["sha256"]:
        raise ValueError(f"render roster binding mismatch: {config_path}")
    return Cohort(
        spec=DatasetSpec(
            spec.key,
            spec.name,
            root,
            spec.expected_classes,
            spec.expected_samples,
            spec.expected_strict_classes,
        ),
        samples=tuple(samples),
        class_order=class_order,
        class_names=class_names,
        class_counts=dict(class_counts),
        config=config,
        summary=summary,
        receipts=receipts,
    )


def _ordered_sample_fingerprint(samples: Sequence[Sample]) -> str:
    return canonical_sha256(
        [
            {
                "dataset": sample.dataset_key,
                "class_id": sample.class_id,
                "sample_index": sample.sample_index,
                "asset_id": sample.asset_id,
                "image_path": str(sample.image_path),
                "png_bytes": sample.png_bytes,
                "png_sha256": sample.png_sha256,
            }
            for sample in samples
        ]
    )


def _load_frozen_pva_colors(path: Path, cohort: Cohort) -> dict[str, str]:
    rows = read_csv(path.expanduser().resolve(strict=True))
    by_id = {
        str(row.get("generator_index") or "").strip(): (
            str(row.get("generator_name") or "").strip(),
            str(row.get("color_hex") or "").strip().lower(),
        )
        for row in rows
    }
    if set(by_id) != set(cohort.class_order):
        raise ValueError("frozen PV-A class-color coverage mismatch")
    result: dict[str, str] = {}
    for class_id in cohort.class_order:
        name, color = by_id[class_id]
        if name != cohort.class_names[class_id] or re.fullmatch(r"#[0-9a-f]{6}", color) is None:
            raise ValueError(f"invalid frozen PV-A color identity: {class_id}")
        result[f"{cohort.spec.key}::{class_id}"] = color
    if len(set(result.values())) != len(result):
        raise ValueError("frozen PV-A colors are not unique")
    return result


def build_color_map(cohorts: Sequence[Cohort], pva_color_index: Path) -> dict[str, str]:
    pva = next(cohort for cohort in cohorts if cohort.spec.key == "pva")
    colors = _load_frozen_pva_colors(pva_color_index, pva)
    external_class_count = sum(
        len(cohort.class_order) for cohort in cohorts if cohort.spec.key != "pva"
    )
    candidates = _palette_helper().build_unique_palette(len(colors) + external_class_count + 256)
    unused = iter(color for color in candidates if color not in set(colors.values()))
    for cohort in cohorts:
        if cohort.spec.key == "pva":
            continue
        for class_id in cohort.class_order:
            colors[f"{cohort.spec.key}::{class_id}"] = next(unused)
    expected = sum(len(cohort.class_order) for cohort in cohorts)
    if len(colors) != expected or len(set(colors.values())) != expected:
        raise ValueError("global one-color-per-class mapping is not unique")
    return colors


def _normalize(features: np.ndarray) -> np.ndarray:
    matrix = np.asarray(features, dtype=np.float32)
    if matrix.ndim != 2 or not np.isfinite(matrix).all():
        raise ValueError(f"invalid feature matrix: {matrix.shape}")
    norms = np.linalg.norm(matrix, axis=1, keepdims=True)
    if np.any(norms <= 0):
        raise ValueError("zero feature vector")
    return matrix / norms


def _valid_feature_matrix(path: Path, expected_rows: int) -> np.ndarray | None:
    try:
        matrix = np.load(path, allow_pickle=False)
        if matrix.dtype != np.float32 or matrix.ndim != 2 or matrix.shape[0] != expected_rows:
            return None
        if not np.isfinite(matrix).all():
            return None
        norms = np.linalg.norm(matrix, axis=1)
        if not np.allclose(norms, 1.0, rtol=1e-4, atol=1e-5):
            return None
        return matrix
    except (OSError, ValueError):
        return None


def _load_pva_feature_cache(
    encoder: str,
    samples: Sequence[Sample],
    model_path: Path,
    pva_feature_dir: Path,
) -> tuple[np.ndarray, dict[str, Any]] | None:
    helper = _feature_helper()
    model_dir = pva_feature_dir / encoder
    feature_path = model_dir / "raw_image_features.npy"
    manifest_path = model_dir / "feature_manifest.json"
    if not feature_path.is_file() or not manifest_path.is_file():
        return None
    try:
        manifest = read_json(manifest_path)
        receipt = manifest.get("raw_feature_receipt")
        paths = tuple(sample.image_path for sample in samples)
        expected_path_fingerprint = helper._paths_fingerprint(paths)
        matrix = _valid_feature_matrix(feature_path, len(samples))
        if (
            matrix is None
            or str(manifest.get("model_path")) != str(model_path.resolve(strict=True))
            or manifest.get("model_fingerprint") != helper._model_fingerprint(model_path)
            or manifest.get("raw_paths_fingerprint") != expected_path_fingerprint
            or not isinstance(receipt, dict)
            or receipt.get("sha256") != sha256_file(feature_path)
            or receipt.get("size_bytes") != feature_path.stat().st_size
        ):
            return None
        return matrix, {"mode": "verified_pva_cache", "source": file_receipt(feature_path)}
    except (OSError, ValueError, TypeError):
        return None


def load_or_extract_features(
    encoder: str,
    samples: Sequence[Sample],
    cohorts: Sequence[Cohort],
    *,
    model_path: Path,
    output_dir: Path,
    pva_feature_dir: Path,
    batch_size: int,
    device: str,
    num_workers: int,
    use_amp: bool,
    force_extract: bool,
) -> tuple[np.ndarray, dict[str, Any], bool]:
    helper = _feature_helper()
    encoder_dir = output_dir / "features" / encoder
    feature_path = encoder_dir / "raw_image_features.npy"
    manifest_path = encoder_dir / "feature_manifest.json"
    model_path = model_path.expanduser().resolve(strict=True)
    expected = {
        "schema_version": 1,
        "encoder": encoder,
        "model_path": str(model_path),
        "model_fingerprint": helper._model_fingerprint(model_path),
        "sample_count": len(samples),
        "ordered_sample_fingerprint": _ordered_sample_fingerprint(samples),
        "feature_helper_sha256": sha256_file(FEATURE_HELPER),
        "runtime_versions": {
            "numpy": np.__version__,
            "torch": importlib.metadata.version("torch"),
            "transformers": importlib.metadata.version("transformers"),
            "pillow": PILLOW_VERSION,
        },
        "preprocessing": {
            "image_processor_use_fast": False,
            "alpha_composite_background_rgb": [255, 255, 255],
        },
    }
    if not force_extract and feature_path.is_file() and manifest_path.is_file():
        try:
            old = read_json(manifest_path)
            matrix = _valid_feature_matrix(feature_path, len(samples))
            receipt = old.get("raw_feature_receipt")
            if (
                matrix is not None
                and all(old.get(key) == value for key, value in expected.items())
                and isinstance(receipt, dict)
                and receipt.get("sha256") == sha256_file(feature_path)
                and receipt.get("size_bytes") == feature_path.stat().st_size
            ):
                print(f"[features] reused {feature_path}", flush=True)
                return matrix, old, True
        except (OSError, ValueError, TypeError):
            pass

    pva_count = len(cohorts[0].samples)
    pva_cache = None if force_extract else _load_pva_feature_cache(
        encoder, samples[:pva_count], model_path, pva_feature_dir
    )
    source_details: list[dict[str, Any]] = []
    if pva_cache is None:
        matrix, extraction = helper.extract_image_features(
            tuple(sample.image_path for sample in samples),
            model_path=model_path,
            batch_size=batch_size,
            device=device,
            num_workers=num_workers,
            use_amp=use_amp,
        )
        source_details.append({"mode": "extracted_all", "extraction": extraction})
    else:
        pva_matrix, cache_details = pva_cache
        external_samples = samples[pva_count:]
        external_matrix, extraction = helper.extract_image_features(
            tuple(sample.image_path for sample in external_samples),
            model_path=model_path,
            batch_size=batch_size,
            device=device,
            num_workers=num_workers,
            use_amp=use_amp,
        )
        matrix = np.concatenate((pva_matrix, external_matrix), axis=0).astype(np.float32)
        source_details.extend((cache_details, {"mode": "extracted_external", "extraction": extraction}))
    matrix = _normalize(matrix).astype(np.float32, copy=False)
    atomic_npy(feature_path, matrix)
    manifest = {
        **expected,
        "dtype": str(matrix.dtype),
        "feature_dim": int(matrix.shape[1]),
        "sources": source_details,
    }
    manifest["raw_feature_receipt"] = file_receipt(feature_path)
    atomic_json(manifest_path, manifest)
    return matrix, manifest, False


def load_or_compute_tsne(
    features: np.ndarray,
    *,
    tag: str,
    encoder: str,
    output_dir: Path,
    feature_receipt_sha256: str,
    source_indices: Sequence[int],
    perplexity: float,
    seed: int,
    max_iter: int,
    n_jobs: int,
    verbose: int,
    force: bool,
) -> tuple[np.ndarray, dict[str, Any], bool]:
    coordinate_dir = output_dir / "coordinates" / encoder
    coordinate_path = coordinate_dir / f"{tag}.npy"
    manifest_path = coordinate_dir / f"{tag}.json"
    index_sha = canonical_sha256(list(map(int, source_indices)))
    expected = {
        "schema_version": 1,
        "encoder": encoder,
        "tag": tag,
        "sample_count": int(features.shape[0]),
        "feature_dim": int(features.shape[1]),
        "source_feature_sha256": feature_receipt_sha256,
        "source_indices_sha256": index_sha,
        "tsne_helper_sha256": sha256_file(FEATURE_HELPER),
        "runtime_versions": {
            "numpy": np.__version__,
            "scikit_learn": importlib.metadata.version("scikit-learn"),
        },
        "parameters": {
            "perplexity": float(perplexity),
            "seed": int(seed),
            "max_iter": int(max_iter),
            "n_jobs": int(n_jobs),
        },
    }
    if not force and coordinate_path.is_file() and manifest_path.is_file():
        try:
            old = read_json(manifest_path)
            coordinates = np.load(coordinate_path, allow_pickle=False)
            receipt = old.get("coordinate_receipt")
            if (
                all(old.get(key) == value for key, value in expected.items())
                and coordinates.dtype == np.float32
                and coordinates.shape == (len(features), 2)
                and np.isfinite(coordinates).all()
                and isinstance(receipt, dict)
                and receipt.get("sha256") == sha256_file(coordinate_path)
                and receipt.get("size_bytes") == coordinate_path.stat().st_size
            ):
                print(f"[tsne] reused {coordinate_path}", flush=True)
                return coordinates, old, True
        except (OSError, ValueError, TypeError):
            pass
    print(f"[tsne] {encoder}/{tag}: {len(features)} samples", flush=True)
    coordinates, details = _feature_helper().compute_tsne(
        features,
        perplexity=perplexity,
        random_state=seed,
        max_iter=max_iter,
        n_jobs=n_jobs,
        verbose=verbose,
    )
    coordinates = np.asarray(coordinates, dtype=np.float32)
    atomic_npy(coordinate_path, coordinates)
    manifest = {**expected, "effective": details, "coordinate_receipt": file_receipt(coordinate_path)}
    atomic_json(manifest_path, manifest)
    return coordinates, manifest, False


def separation_metrics(features: np.ndarray, class_keys: Sequence[str]) -> dict[str, float | int]:
    matrix = _normalize(features)
    labels = np.asarray(class_keys, dtype=object)
    counts = Counter(labels.tolist())
    if not counts or set(counts.values()) != {5}:
        raise ValueError("strict feature metrics require exactly five samples per class")
    cosine_similarity = matrix @ matrix.T
    similarity = cosine_similarity.copy()
    np.fill_diagonal(similarity, -np.inf)
    nearest = np.argmax(similarity, axis=1)
    top1 = float(np.mean(labels[nearest] == labels))
    top4_indices = np.argpartition(similarity, -4, axis=1)[:, -4:]
    top4_fraction = float(np.mean(labels[top4_indices] == labels[:, None]))
    same = labels[:, None] == labels[None, :]
    np.fill_diagonal(same, False)
    positive_similarity = np.where(same, similarity, -np.inf)
    negative_similarity = np.where(~same, similarity, -np.inf)
    np.fill_diagonal(negative_similarity, -np.inf)
    nearest_positive = np.max(positive_similarity, axis=1)
    nearest_negative = np.max(negative_similarity, axis=1)
    upper_same = np.triu(same, k=1)
    mean_within_distance = float(np.mean(1.0 - similarity[upper_same]))
    cosine_distance = np.maximum(0.0, 1.0 - cosine_similarity)
    class_order = tuple(counts)
    class_to_index = {class_key: index for index, class_key in enumerate(class_order)}
    label_indices = np.asarray([class_to_index[value] for value in labels], dtype=np.int32)
    mean_distance_to_class = np.column_stack(
        [cosine_distance[:, labels == class_key].mean(axis=1) for class_key in class_order]
    )
    within_distance = np.sum(cosine_distance * same, axis=1) / 4.0
    mean_distance_to_class[np.arange(len(labels)), label_indices] = np.inf
    nearest_other_class_distance = np.min(mean_distance_to_class, axis=1)
    denominator = np.maximum(within_distance, nearest_other_class_distance)
    silhouette_values = np.divide(
        nearest_other_class_distance - within_distance,
        denominator,
        out=np.zeros_like(within_distance),
        where=denominator > 0,
    )
    silhouette = float(np.mean(silhouette_values))
    return {
        "class_count": len(counts),
        "sample_count": len(labels),
        "top1_same_class_rate": top1,
        "top4_same_class_fraction": top4_fraction,
        "cosine_silhouette": silhouette,
        "mean_within_class_cosine_distance": mean_within_distance,
        "mean_nearest_positive_cosine_similarity": float(np.mean(nearest_positive)),
        "mean_nearest_negative_cosine_similarity": float(np.mean(nearest_negative)),
        "mean_nearest_positive_minus_negative_margin": float(
            np.mean(nearest_positive - nearest_negative)
        ),
    }


def _sample_rows(
    samples: Sequence[Sample],
    color_map: Mapping[str, str],
    *,
    global_indices: Sequence[int] | None = None,
    coordinates: np.ndarray | None = None,
) -> list[dict[str, Any]]:
    if global_indices is None:
        global_indices = range(len(samples))
    rows: list[dict[str, Any]] = []
    for row_index, (global_index, sample) in enumerate(zip(global_indices, samples, strict=True)):
        row: dict[str, Any] = {
            "global_index": global_index,
            "dataset": sample.dataset_name,
            "dataset_key": sample.dataset_key,
            "dataset_ordinal": sample.ordinal,
            "render_key": sample.render_key,
            "class_id": sample.class_id,
            "class_name": sample.class_name,
            "class_key": sample.class_key,
            "sample_index": sample.sample_index,
            "asset_id": sample.asset_id,
            "source_type": sample.source_type,
            "color_hex": color_map[sample.class_key],
            "image_path": str(sample.image_path),
            "png_bytes": sample.png_bytes,
            "png_sha256": sample.png_sha256,
        }
        if coordinates is not None:
            row["tsne_x"] = format(float(coordinates[row_index, 0]), ".9g")
            row["tsne_y"] = format(float(coordinates[row_index, 1]), ".9g")
        rows.append(row)
    return rows


def _class_centers(samples: Sequence[Sample], coordinates: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    class_order = tuple(dict.fromkeys(sample.class_key for sample in samples))
    class_to_index = {key: index for index, key in enumerate(class_order)}
    labels = np.asarray([class_to_index[sample.class_key] for sample in samples], dtype=np.int32)
    centers = np.stack(
        [coordinates[labels == index].mean(axis=0) for index in range(len(class_order))]
    ).astype(np.float32)
    return centers, labels


def _style_axis(axis: Any, title: str) -> None:
    axis.set_title(title, fontsize=11, pad=7)
    axis.set_xlabel("t-SNE 1", fontsize=8)
    axis.set_ylabel("t-SNE 2", fontsize=8)
    axis.tick_params(labelsize=7)
    axis.spines[["top", "right"]].set_visible(False)
    axis.grid(color="#d8d8d8", linewidth=0.4, alpha=0.45)
    axis.set_axisbelow(True)


def _draw_class_axis(
    axis: Any,
    coordinates: np.ndarray,
    samples: Sequence[Sample],
    color_map: Mapping[str, str],
    title: str,
) -> None:
    from matplotlib.collections import LineCollection

    centers, labels = _class_centers(samples, coordinates)
    class_order = tuple(dict.fromkeys(sample.class_key for sample in samples))
    class_colors = [color_map[key] for key in class_order]
    sample_colors = [class_colors[index] for index in labels]
    segments = np.stack((centers[labels], coordinates), axis=1)
    axis.add_collection(
        LineCollection(
            segments,
            colors=sample_colors,
            linewidths=0.22,
            alpha=0.10,
            rasterized=True,
            zorder=1,
        )
    )
    axis.scatter(
        coordinates[:, 0], coordinates[:, 1], c=sample_colors, s=8, alpha=0.62,
        edgecolors="none", rasterized=True, zorder=2,
    )
    axis.scatter(
        centers[:, 0], centers[:, 1], c=class_colors, s=16, alpha=1.0,
        edgecolors="#202020", linewidths=0.18, rasterized=True, zorder=3,
    )
    _style_axis(axis, title)


def save_independent_grid(
    cohorts: Sequence[Cohort],
    coordinates: Mapping[tuple[str, str, str], np.ndarray],
    color_map: Mapping[str, str],
    output_path: Path,
    *,
    mode: str,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(len(cohorts), 2, figsize=(18, 22), dpi=dpi)
    figure.subplots_adjust(left=0.06, right=0.985, top=0.935, bottom=0.045, hspace=0.28, wspace=0.14)
    mode_title = "All available classes" if mode == "all" else "Strict five-distinct-assets classes"
    figure.suptitle(f"Uniform Blender renders: {mode_title}", fontsize=18, y=0.978)
    for row_index, cohort in enumerate(cohorts):
        samples = cohort.samples if mode == "all" else cohort.strict_samples
        for column_index, encoder in enumerate(ENCODERS):
            title = (
                f"{cohort.spec.name} | {ENCODER_LABELS[encoder]}\n"
                f"{len(set(sample.class_id for sample in samples))} classes, {len(samples):,} renders"
            )
            _draw_class_axis(
                axes[row_index, column_index],
                coordinates[(cohort.spec.key, encoder, mode)],
                samples,
                color_map,
                title,
            )
    figure.text(
        0.5,
        0.012,
        "Each panel is fitted independently. Class colors are fixed across encoders; absolute panel positions are not comparable.",
        ha="center",
        fontsize=8,
        color="#404040",
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(f".{output_path.name}.tmp")
    figure.savefig(temporary, format="png", dpi=dpi, facecolor="white")
    temporary.replace(output_path)
    plt.close(figure)


def save_joint_grid(
    strict_samples: Sequence[Sample],
    coordinates: Mapping[str, np.ndarray],
    output_path: Path,
    *,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    figure, axes = plt.subplots(1, 2, figsize=(20, 9), dpi=dpi)
    figure.subplots_adjust(left=0.055, right=0.985, top=0.88, bottom=0.10, wspace=0.14)
    figure.suptitle("Joint t-SNE of strict n=5 uniform-render cohorts", fontsize=18, y=0.96)
    point_colors = [DATASET_COLORS[sample.dataset_key] for sample in strict_samples]
    for axis, encoder in zip(axes, ENCODERS, strict=True):
        points = coordinates[encoder]
        axis.scatter(
            points[:, 0], points[:, 1], c=point_colors, s=8, alpha=0.56,
            edgecolors="none", rasterized=True,
        )
        _style_axis(axis, ENCODER_LABELS[encoder])
    counts = Counter(sample.dataset_key for sample in strict_samples)
    names = dict.fromkeys(sample.dataset_key for sample in strict_samples)
    for sample in strict_samples:
        names[sample.dataset_key] = sample.dataset_name
    handles = [
        Line2D(
            [], [], linestyle="none", marker="o", markersize=6,
            markerfacecolor=DATASET_COLORS[key], markeredgecolor="none",
            label=f"{names[key]} ({counts[key]:,})",
        )
        for key in names
    ]
    axes[-1].legend(handles=handles, loc="best", frameon=True, fontsize=8)
    figure.text(
        0.5,
        0.025,
        "Dataset color shows visual-domain overlap; it does not measure category quality.",
        ha="center",
        fontsize=8,
        color="#404040",
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(f".{output_path.name}.tmp")
    figure.savefig(temporary, format="png", dpi=dpi, facecolor="white")
    temporary.replace(output_path)
    plt.close(figure)


def save_metrics_plot(rows: Sequence[Mapping[str, Any]], output_path: Path, *, dpi: int) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    metrics = (
        ("top1_same_class_rate", "Top-1 same-class NN", (0.0, 1.0)),
        ("top4_same_class_fraction", "Top-4 same-class fraction", (0.0, 1.0)),
        ("cosine_silhouette", "Cosine silhouette", None),
        ("mean_nearest_positive_minus_negative_margin", "Nearest positive-negative margin", None),
    )
    dataset_names = list(dict.fromkeys(str(row["dataset"]) for row in rows))
    x = np.arange(len(dataset_names), dtype=np.float32)
    width = 0.36
    figure, axes = plt.subplots(2, 2, figsize=(15, 10), dpi=dpi)
    figure.subplots_adjust(left=0.08, right=0.985, top=0.88, bottom=0.10, hspace=0.34, wspace=0.20)
    figure.suptitle("Strict n=5 separation in original feature space", fontsize=17, y=0.96)
    for axis, (metric, title, limits) in zip(axes.flat, metrics, strict=True):
        for encoder_index, encoder in enumerate(ENCODERS):
            values = [
                float(next(row[metric] for row in rows if row["dataset"] == name and row["encoder"] == encoder))
                for name in dataset_names
            ]
            axis.bar(
                x + (encoder_index - 0.5) * width,
                values,
                width,
                color="#347a9f" if encoder == "dinov2" else "#d05b42",
                label=ENCODER_LABELS[encoder],
            )
        axis.set_title(title, fontsize=11)
        axis.set_xticks(x, dataset_names, rotation=12, ha="right", fontsize=8)
        axis.tick_params(axis="y", labelsize=8)
        axis.spines[["top", "right"]].set_visible(False)
        axis.grid(axis="y", color="#d8d8d8", linewidth=0.45, alpha=0.55)
        axis.set_axisbelow(True)
        if limits is not None:
            axis.set_ylim(*limits)
    axes[0, 0].legend(frameon=True, fontsize=8)
    figure.text(
        0.5,
        0.018,
        "All classes contain five distinct assets. Dataset taxonomies and class counts differ, so values are descriptive rather than a quality ranking.",
        ha="center",
        fontsize=8,
        color="#404040",
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(f".{output_path.name}.tmp")
    figure.savefig(temporary, format="png", dpi=dpi, facecolor="white")
    temporary.replace(output_path)
    plt.close(figure)


def _dataset_specs(args: argparse.Namespace) -> tuple[DatasetSpec, ...]:
    return (
        DatasetSpec("pva", "PV-A", args.pva_render_root, 531, 2655, 531),
        DatasetSpec("artiverse", "Artiverse", args.artiverse_render_root, 84, 368, 65),
        DatasetSpec("articraft10k", "Articraft-10K", args.articraft_render_root, 244, 1193, 236),
        DatasetSpec("partnet_mobility", "PartNet-Mobility", args.partnet_render_root, 46, 230, 46),
    )


def dependency_preflight() -> None:
    required = ("sklearn", "torch", "transformers")
    missing = [name for name in required if importlib.util.find_spec(name) is None]
    if missing:
        raise RuntimeError(
            f"Python environment {sys.executable} is missing required packages: "
            f"{', '.join(missing)}"
        )


def run(args: argparse.Namespace) -> dict[str, Any]:
    dependency_preflight()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    cohorts = tuple(load_cohort(spec, verify_pngs=not args.skip_png_hash_audit) for spec in _dataset_specs(args))
    studio_contracts = {cohort.spec.key: _studio_contract(cohort.config) for cohort in cohorts}
    if len({canonical_sha256(contract) for contract in studio_contracts.values()}) != 1:
        raise ValueError(f"render studio contracts differ: {studio_contracts}")

    all_samples = tuple(sample for cohort in cohorts for sample in cohort.samples)
    global_index = {sample.render_key: index for index, sample in enumerate(all_samples)}
    if len(global_index) != len(all_samples):
        raise ValueError("render_key is not globally unique across datasets")
    strict_samples = tuple(sample for cohort in cohorts for sample in cohort.strict_samples)
    strict_global_indices = [global_index[sample.render_key] for sample in strict_samples]
    color_map = build_color_map(cohorts, args.pva_color_index)

    atomic_csv(output_dir / "render_sample_index.csv", _sample_rows(all_samples, color_map))
    color_rows: list[dict[str, Any]] = []
    class_ordinal = 0
    for cohort in cohorts:
        for class_id in cohort.class_order:
            class_ordinal += 1
            class_key = f"{cohort.spec.key}::{class_id}"
            color_rows.append(
                {
                    "global_class_ordinal": class_ordinal,
                    "dataset": cohort.spec.name,
                    "dataset_key": cohort.spec.key,
                    "class_id": class_id,
                    "class_name": cohort.class_names[class_id],
                    "class_key": class_key,
                    "sample_count": cohort.class_counts[class_id],
                    "strict_n5_eligible": cohort.class_counts[class_id] == 5,
                    "color_hex": color_map[class_key],
                }
            )
    atomic_csv(output_dir / "class_color_index.csv", color_rows)
    dataset_rows = [
        {
            "dataset": cohort.spec.name,
            "class_count": len(cohort.class_order),
            "render_count": len(cohort.samples),
            "strict_n5_class_count": len(cohort.strict_class_ids),
            "strict_n5_render_count": len(cohort.strict_samples),
            "short_class_count": len(cohort.class_order) - len(cohort.strict_class_ids),
            "per_class_count_values": ",".join(map(str, sorted(set(cohort.class_counts.values())))),
            "render_root": str(cohort.spec.root),
        }
        for cohort in cohorts
    ]
    atomic_csv(output_dir / "dataset_summary.csv", dataset_rows)

    feature_models = {"dinov2": args.dino_model, "clip": args.clip_model}
    independent_coordinates: dict[tuple[str, str, str], np.ndarray] = {}
    joint_coordinates: dict[str, np.ndarray] = {}
    metric_rows: list[dict[str, Any]] = []
    encoder_manifests: dict[str, Any] = {}
    offset_by_dataset: dict[str, int] = {}
    running_offset = 0
    for cohort in cohorts:
        offset_by_dataset[cohort.spec.key] = running_offset
        running_offset += len(cohort.samples)

    for encoder in ENCODERS:
        features, feature_manifest, feature_reused = load_or_extract_features(
            encoder,
            all_samples,
            cohorts,
            model_path=feature_models[encoder],
            output_dir=output_dir,
            pva_feature_dir=args.pva_feature_dir.expanduser().resolve(strict=True),
            batch_size=args.batch_size,
            device=args.device,
            num_workers=args.num_workers,
            use_amp=args.use_amp,
            force_extract=args.force_extract,
        )
        feature_sha = str(feature_manifest["raw_feature_receipt"]["sha256"])
        encoder_details: dict[str, Any] = {
            "feature_cache_reused": feature_reused,
            "feature_manifest": feature_manifest,
            "datasets": {},
        }
        for cohort in cohorts:
            offset = offset_by_dataset[cohort.spec.key]
            all_indices = list(range(offset, offset + len(cohort.samples)))
            all_features = features[all_indices]
            all_coords, all_tsne_manifest, all_reused = load_or_compute_tsne(
                all_features,
                tag=f"{cohort.spec.key}_all",
                encoder=encoder,
                output_dir=output_dir,
                feature_receipt_sha256=feature_sha,
                source_indices=all_indices,
                perplexity=args.perplexity,
                seed=args.seed,
                max_iter=args.tsne_max_iter,
                n_jobs=args.tsne_jobs,
                verbose=args.tsne_verbose,
                force=args.force_tsne,
            )
            independent_coordinates[(cohort.spec.key, encoder, "all")] = all_coords
            atomic_csv(
                output_dir / "coordinates" / encoder / f"{cohort.spec.key}_all.csv",
                _sample_rows(cohort.samples, color_map, global_indices=all_indices, coordinates=all_coords),
            )
            eligible = set(cohort.strict_class_ids)
            strict_local_indices = [
                index for index, sample in enumerate(cohort.samples) if sample.class_id in eligible
            ]
            strict_indices = [offset + index for index in strict_local_indices]
            strict_features = features[strict_indices]
            if len(strict_indices) == len(all_indices):
                strict_coords = all_coords
                strict_tsne_manifest = {
                    "aliased_to_all_cohort": True,
                    **all_tsne_manifest,
                }
                strict_reused = all_reused
            else:
                strict_coords, strict_tsne_manifest, strict_reused = load_or_compute_tsne(
                    strict_features,
                    tag=f"{cohort.spec.key}_strict_n5",
                    encoder=encoder,
                    output_dir=output_dir,
                    feature_receipt_sha256=feature_sha,
                    source_indices=strict_indices,
                    perplexity=args.perplexity,
                    seed=args.seed,
                    max_iter=args.tsne_max_iter,
                    n_jobs=args.tsne_jobs,
                    verbose=args.tsne_verbose,
                    force=args.force_tsne,
                )
            independent_coordinates[(cohort.spec.key, encoder, "strict_n5")] = strict_coords
            atomic_csv(
                output_dir / "coordinates" / encoder / f"{cohort.spec.key}_strict_n5.csv",
                _sample_rows(
                    cohort.strict_samples,
                    color_map,
                    global_indices=strict_indices,
                    coordinates=strict_coords,
                ),
            )
            metrics = separation_metrics(
                strict_features, [sample.class_key for sample in cohort.strict_samples]
            )
            metric_rows.append(
                {
                    "dataset": cohort.spec.name,
                    "dataset_key": cohort.spec.key,
                    "encoder": encoder,
                    "encoder_label": ENCODER_LABELS[encoder],
                    **metrics,
                }
            )
            encoder_details["datasets"][cohort.spec.key] = {
                "all_tsne": all_tsne_manifest,
                "all_tsne_cache_reused": all_reused,
                "strict_n5_tsne": strict_tsne_manifest,
                "strict_n5_tsne_cache_reused": strict_reused,
                "strict_n5_metrics": metrics,
            }

        strict_features = features[strict_global_indices]
        joint_coords, joint_manifest, joint_reused = load_or_compute_tsne(
            strict_features,
            tag="joint_strict_n5",
            encoder=encoder,
            output_dir=output_dir,
            feature_receipt_sha256=feature_sha,
            source_indices=strict_global_indices,
            perplexity=args.perplexity,
            seed=args.seed,
            max_iter=args.tsne_max_iter,
            n_jobs=args.tsne_jobs,
            verbose=args.tsne_verbose,
            force=args.force_tsne,
        )
        joint_coordinates[encoder] = joint_coords
        atomic_csv(
            output_dir / "coordinates" / encoder / "joint_strict_n5.csv",
            _sample_rows(
                strict_samples,
                color_map,
                global_indices=strict_global_indices,
                coordinates=joint_coords,
            ),
        )
        encoder_details["joint_strict_n5_tsne"] = joint_manifest
        encoder_details["joint_strict_n5_tsne_cache_reused"] = joint_reused
        encoder_manifests[encoder] = encoder_details
        del features

    atomic_csv(output_dir / "strict_n5_feature_metrics.csv", metric_rows)
    atomic_json(output_dir / "strict_n5_feature_metrics.json", {"rows": metric_rows})
    save_independent_grid(
        cohorts,
        independent_coordinates,
        color_map,
        output_dir / "tsne_independent_all_classes.png",
        mode="all",
        dpi=args.plot_dpi,
    )
    save_independent_grid(
        cohorts,
        independent_coordinates,
        color_map,
        output_dir / "tsne_independent_strict_n5.png",
        mode="strict_n5",
        dpi=args.plot_dpi,
    )
    save_joint_grid(
        strict_samples,
        joint_coordinates,
        output_dir / "tsne_joint_strict_n5_by_dataset.png",
        dpi=args.plot_dpi,
    )
    save_metrics_plot(
        metric_rows,
        output_dir / "feature_space_metrics_strict_n5.png",
        dpi=args.plot_dpi,
    )

    required = [
        output_dir / "render_sample_index.csv",
        output_dir / "class_color_index.csv",
        output_dir / "dataset_summary.csv",
        output_dir / "strict_n5_feature_metrics.csv",
        output_dir / "strict_n5_feature_metrics.json",
        output_dir / "tsne_independent_all_classes.png",
        output_dir / "tsne_independent_strict_n5.png",
        output_dir / "tsne_joint_strict_n5_by_dataset.png",
        output_dir / "feature_space_metrics_strict_n5.png",
    ]
    for encoder in ENCODERS:
        required.extend(
            (
                output_dir / "features" / encoder / "raw_image_features.npy",
                output_dir / "features" / encoder / "feature_manifest.json",
                output_dir / "coordinates" / encoder / "joint_strict_n5.npy",
                output_dir / "coordinates" / encoder / "joint_strict_n5.csv",
            )
        )
    audit_checks = {
        "same_blender_studio_contract": len(
            {canonical_sha256(contract) for contract in studio_contracts.values()}
        ) == 1,
        "expected_dataset_count": len(cohorts) == 4,
        "expected_all_sample_count": len(all_samples) == 4446,
        "expected_all_class_count": len(color_map) == 905,
        "one_global_unique_color_per_class": len(set(color_map.values())) == len(color_map),
        "expected_strict_sample_count": len(strict_samples) == 4390,
        "expected_strict_class_count": sum(len(cohort.strict_class_ids) for cohort in cohorts) == 878,
        "all_coordinates_finite": all(
            np.isfinite(value).all() and value.ndim == 2 and value.shape[1] == 2
            for value in independent_coordinates.values()
        ),
        "joint_coordinates_finite": all(
            value.shape == (len(strict_samples), 2) and np.isfinite(value).all()
            for value in joint_coordinates.values()
        ),
        "metric_rows_complete": len(metric_rows) == len(cohorts) * len(ENCODERS),
        "required_artifacts_exist": all(path.is_file() and path.stat().st_size > 0 for path in required),
    }
    audit = {
        "pass": all(audit_checks.values()),
        "checks": audit_checks,
        "counts": {
            "all_samples": len(all_samples),
            "all_classes": len(color_map),
            "strict_n5_samples": len(strict_samples),
            "strict_n5_classes": sum(len(cohort.strict_class_ids) for cohort in cohorts),
            "unique_class_colors": len(set(color_map.values())),
        },
        "studio_contract": next(iter(studio_contracts.values())),
        "png_hash_audit_performed": not args.skip_png_hash_audit,
        "dataset_receipts": {cohort.spec.key: cohort.receipts for cohort in cohorts},
        "artifact_receipts": {str(path.relative_to(output_dir)): file_receipt(path) for path in required},
    }
    atomic_json(output_dir / "audit.json", audit)
    if not audit["pass"]:
        failed = [key for key, passed in audit_checks.items() if not passed]
        raise RuntimeError(f"output audit failed: {failed}")

    run_manifest = {
        "schema_version": 1,
        "analysis_contract": "four_dataset_uniform_render_max5_and_strict_n5_v1",
        "output_dir": str(output_dir),
        "datasets": dataset_rows,
        "render_studio_contract": next(iter(studio_contracts.values())),
        "encoders": encoder_manifests,
        "parameters": {
            "device": args.device,
            "batch_size": args.batch_size,
            "num_workers": args.num_workers,
            "use_amp": args.use_amp,
            "perplexity": args.perplexity,
            "seed": args.seed,
            "tsne_max_iter": args.tsne_max_iter,
            "tsne_jobs": args.tsne_jobs,
            "plot_dpi": args.plot_dpi,
            "skip_png_hash_audit": args.skip_png_hash_audit,
        },
        "interpretation_limits": [
            "t-SNE is a qualitative local-neighborhood visualization, not a dataset-quality score.",
            "Independent t-SNE panels have unrelated coordinate systems.",
            "The four datasets use different taxonomies, class granularity, and class counts.",
            "Strict n=5 metrics exclude 19 short Artiverse classes and 8 short Articraft-10K classes.",
            "High class separability can reflect rendering consistency or taxonomy granularity, not asset quality alone.",
        ],
        "scripts": {
            "driver": file_receipt(SCRIPT),
            "feature_helper": file_receipt(FEATURE_HELPER),
            "palette_helper": file_receipt(PALETTE_HELPER),
        },
        "artifacts": {
            "audit": "audit.json",
            "sample_index": "render_sample_index.csv",
            "class_color_index": "class_color_index.csv",
            "dataset_summary": "dataset_summary.csv",
            "metrics_csv": "strict_n5_feature_metrics.csv",
            "all_class_tsne": "tsne_independent_all_classes.png",
            "strict_n5_tsne": "tsne_independent_strict_n5.png",
            "joint_tsne": "tsne_joint_strict_n5_by_dataset.png",
            "metrics_plot": "feature_space_metrics_strict_n5.png",
        },
    }
    atomic_json(output_dir / "run_manifest.json", run_manifest)
    receipt_paths = sorted(
        path
        for path in output_dir.rglob("*")
        if path.is_file() and path != output_dir / "release_receipt.json"
    )
    release_receipt = {
        "schema_version": 1,
        "artifact_count": len(receipt_paths),
        "artifacts": {
            str(path.relative_to(output_dir)): file_receipt(path) for path in receipt_paths
        },
    }
    release_receipt["artifact_set_sha256"] = canonical_sha256(release_receipt["artifacts"])
    atomic_json(output_dir / "release_receipt.json", release_receipt)
    print(
        f"[done] {len(all_samples):,} renders, {len(color_map)} classes, "
        f"{len(strict_samples):,} strict n=5 samples -> {output_dir}",
        flush=True,
    )
    return run_manifest


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pva-render-root", type=Path,
        default=REPO_ROOT / "exp/uniform531_n5_studio_256_v1",
    )
    parser.add_argument(
        "--artiverse-render-root", type=Path,
        default=REPO_ROOT / "exp/artiverse_n5_uniform_studio_256_v1",
    )
    parser.add_argument(
        "--articraft-render-root", type=Path,
        default=REPO_ROOT / "exp/articraft10k_uniform_n5_studio_256_v1",
    )
    parser.add_argument(
        "--partnet-render-root", type=Path,
        default=REPO_ROOT / "exp/partnet_mobility_uniform_n5_studio_256_v1",
    )
    parser.add_argument("--pva-feature-dir", type=Path, default=DEFAULT_PVA_FEATURE_DIR)
    parser.add_argument("--pva-color-index", type=Path, default=DEFAULT_PVA_COLOR_INDEX)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--dino-model", type=Path, default=DEFAULT_DINO_MODEL)
    parser.add_argument("--clip-model", type=Path, default=DEFAULT_CLIP_MODEL)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--num-workers", type=int, default=8)
    parser.add_argument("--no-amp", action="store_false", dest="use_amp")
    parser.set_defaults(use_amp=True)
    parser.add_argument("--perplexity", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--tsne-max-iter", type=int, default=1000)
    parser.add_argument("--tsne-jobs", type=int, default=min(16, os.cpu_count() or 1))
    parser.add_argument("--tsne-verbose", type=int, choices=(0, 1, 2), default=1)
    parser.add_argument("--plot-dpi", type=int, default=180)
    parser.add_argument("--force-extract", action="store_true")
    parser.add_argument("--force-tsne", action="store_true")
    parser.add_argument("--skip-png-hash-audit", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    manifest = run(args)
    print(json.dumps(manifest["artifacts"], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
