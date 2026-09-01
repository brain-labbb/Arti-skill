#!/usr/bin/env python3
"""Extract ViT image features and visualize supported datasets with t-SNE."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

for _thread_variable in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_thread_variable, "1")

import numpy as np
from PIL import Image


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_DATA_DIR = REPO_ROOT / "exp" / "artiverse" / "data"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exp" / "runtime" / "artiverse_vit_tsne_dinov2_base"
DEFAULT_PICTURE_DATA_DIR = REPO_ROOT / "articraft_data" / "picture"
DEFAULT_PICTURE_OUTPUT_DIR = REPO_ROOT / "exp" / "runtime" / "articraft_picture_vit_tsne"
DEFAULT_MODEL_SNAPSHOT = Path(
    "/root/.cache/huggingface/hub/models--facebook--dinov2-base/"
    "snapshots/f9e44c814b77203eaa57a6bdbbd535f21ede1415"
)
DEFAULT_MODEL = DEFAULT_MODEL_SNAPSHOT
DEFAULT_TSNE_JOBS = min(64, os.cpu_count() or 1)
PICTURE_UNMAPPED_CATEGORY = "0611"
PICTURE_UNMAPPED_LABEL = "0611 (unmapped batch)"


@dataclass(frozen=True, slots=True)
class RenderSample:
    image_path: Path
    category: str
    source: str
    model_id: str
    view_id: str


@dataclass(frozen=True, slots=True)
class PictureSample:
    image_path: Path
    category: str
    subcategory: str
    image_id: str


@dataclass(frozen=True, slots=True)
class ModelSample:
    category: str
    source: str
    model_id: str
    view_count: int


@dataclass(frozen=True, slots=True)
class PipelineConfig:
    data_dir: Path
    output_dir: Path
    model_name_or_path: str | Path
    batch_size: int
    device: str
    num_workers: int
    use_amp: bool
    expected_views: int
    requested_perplexity: float
    random_state: int
    tsne_max_iter: int
    plot_dpi: int
    tsne_verbose: int
    force_extract: bool
    dataset_format: str = "artiverse"
    tsne_jobs: int = DEFAULT_TSNE_JOBS


class FeatureCacheValidationError(RuntimeError):
    """Raised when a feature cache exists but cannot be safely reused."""


def resolve_dataset_paths(
    dataset_format: str,
    data_dir: Path | None,
    output_dir: Path | None,
) -> tuple[Path, Path]:
    """Resolve format-specific defaults while preserving explicit paths."""
    if dataset_format == "artiverse":
        default_data_dir = DEFAULT_DATA_DIR
        default_output_dir = DEFAULT_OUTPUT_DIR
    elif dataset_format == "picture":
        default_data_dir = DEFAULT_PICTURE_DATA_DIR
        default_output_dir = DEFAULT_PICTURE_OUTPUT_DIR
    else:
        raise ValueError(f"unsupported dataset format: {dataset_format}")
    return data_dir or default_data_dir, output_dir or default_output_dir


def _validate_output_dataset_format(output_dir: Path, dataset_format: str) -> None:
    """Reject reusing a run directory that belongs to another dataset format."""
    manifest_path = output_dir / "run_manifest.json"
    if not manifest_path.is_file():
        return
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"existing run manifest is unreadable: {manifest_path}") from exc
    config = manifest.get("config") if isinstance(manifest, dict) else None
    existing_format = (
        config.get("dataset_format", "artiverse") if isinstance(config, dict) else "artiverse"
    )
    if existing_format != dataset_format:
        raise ValueError(
            f"output directory belongs to {existing_format}, not {dataset_format}: {output_dir}"
        )


def discover_render_samples(data_dir: Path) -> list[RenderSample]:
    """Return reference renders under data/{category}/{source}/{model}/imgs."""
    data_dir = data_dir.resolve()
    samples: list[RenderSample] = []
    for image_path in sorted(data_dir.glob("*/*/*/imgs/*.png")):
        resolved_image_path = image_path.resolve(strict=True)
        try:
            resolved_image_path.relative_to(data_dir)
        except ValueError as exc:
            raise ValueError(f"render path resolves outside data directory: {image_path}") from exc
        relative = image_path.relative_to(data_dir)
        category, source, model_id, image_directory, filename = relative.parts
        if image_directory != "imgs":
            continue
        samples.append(
            RenderSample(
                image_path=resolved_image_path,
                category=category,
                source=source,
                model_id=model_id,
                view_id=Path(filename).stem,
            )
        )
    return samples


def validate_artiverse_samples(
    samples: Sequence[RenderSample],
    *,
    expected_views: int,
) -> dict[str, int]:
    """Validate Artiverse model/view structure and return dataset counts."""
    if expected_views < 1:
        raise ValueError("expected_views must be positive")
    if not samples:
        raise ValueError("no Artiverse reference renders were found")
    grouped: dict[tuple[str, str, str], set[str]] = {}
    for sample in samples:
        key = (sample.category, sample.source, sample.model_id)
        views = grouped.setdefault(key, set())
        if sample.view_id in views:
            raise ValueError(f"duplicate view {sample.view_id} for model {'/'.join(key)}")
        views.add(sample.view_id)
    invalid = [(key, len(views)) for key, views in grouped.items() if len(views) != expected_views]
    if invalid:
        preview = ", ".join(f"{'/'.join(key)}={count}" for key, count in invalid[:5])
        raise ValueError(
            f"expected {expected_views} views for every model; "
            f"found {len(invalid)} invalid models ({preview})"
        )
    return {
        "image_count": len(samples),
        "model_count": len(grouped),
        "category_count": len({sample.category for sample in samples}),
        "source_count": len({sample.source for sample in samples}),
        "views_per_model": expected_views,
    }


def discover_picture_samples(data_dir: Path) -> list[PictureSample]:
    """Return images under picture/{category}/{subcategory}/{image-id}.png."""
    data_dir = data_dir.resolve()
    samples: list[PictureSample] = []
    for image_path in sorted(data_dir.glob("*/*/*.png")):
        resolved_image_path = image_path.resolve(strict=True)
        try:
            resolved_image_path.relative_to(data_dir)
        except ValueError as exc:
            raise ValueError(f"image path resolves outside data directory: {image_path}") from exc
        category, subcategory, filename = image_path.relative_to(data_dir).parts
        samples.append(
            PictureSample(
                image_path=resolved_image_path,
                category=category,
                subcategory=subcategory,
                image_id=Path(filename).stem,
            )
        )
    return samples


def validate_picture_samples(samples: Sequence[PictureSample]) -> dict[str, int]:
    """Validate picture identities and return image/category counts."""
    if not samples:
        raise ValueError("no Articraft picture images were found")
    identities: set[tuple[str, str, str]] = set()
    for sample in samples:
        identity = (sample.category, sample.subcategory, sample.image_id)
        if identity in identities:
            raise ValueError(f"duplicate picture identity {'/'.join(identity)}")
        identities.add(identity)
    categories = {sample.category for sample in samples}
    return {
        "image_count": len(samples),
        "category_count": len(categories),
        "semantic_category_count": len(categories - {PICTURE_UNMAPPED_CATEGORY}),
        "subcategory_count": len(
            {(sample.category, sample.subcategory) for sample in samples}
        ),
        "unmapped_batch_image_count": sum(
            sample.category == PICTURE_UNMAPPED_CATEGORY for sample in samples
        ),
    }


def build_picture_leaf_plot_labels(samples: Sequence[PictureSample]) -> list[str]:
    """Return one scoped label for every picture leaf subcategory."""
    return [f"{sample.category}/{sample.subcategory}" for sample in samples]


def build_picture_taxonomy_summary(samples: Sequence[PictureSample]) -> dict[str, int]:
    """Count both parent directories and scoped leaf directories in picture taxonomy."""
    categories = {sample.category for sample in samples}
    leaf_subcategories = {
        (sample.category, sample.subcategory)
        for sample in samples
    }
    semantic_categories = categories - {PICTURE_UNMAPPED_CATEGORY}
    semantic_leaf_subcategories = {
        key for key in leaf_subcategories if key[0] != PICTURE_UNMAPPED_CATEGORY
    }
    return {
        "category_count": len(categories),
        "semantic_category_count": len(semantic_categories),
        "leaf_subcategory_count": len(leaf_subcategories),
        "semantic_leaf_subcategory_count": len(semantic_leaf_subcategories),
        "taxonomy_node_count": len(categories) + len(leaf_subcategories),
        "semantic_taxonomy_node_count": (
            len(semantic_categories) + len(semantic_leaf_subcategories)
        ),
    }


def build_picture_subcategory_plot_labels(
    samples: Sequence[PictureSample],
    *,
    min_count: int,
) -> list[str]:
    """Collapse unmapped and sparse subcategories into interpretable plot labels."""
    if min_count < 1:
        raise ValueError("min_count must be positive")
    counts: dict[tuple[str, str], int] = {}
    for sample in samples:
        key = (sample.category, sample.subcategory)
        counts[key] = counts.get(key, 0) + 1
    sparse_label = f"Other subcategories (n<{min_count})"
    labels: list[str] = []
    for sample in samples:
        key = (sample.category, sample.subcategory)
        if sample.category == PICTURE_UNMAPPED_CATEGORY:
            labels.append(PICTURE_UNMAPPED_LABEL)
        elif counts[key] < min_count:
            labels.append(sparse_label)
        else:
            labels.append(f"{sample.category}/{sample.subcategory}")
    return labels


def _sample_fingerprint(
    samples: Sequence[RenderSample | PictureSample],
    data_dir: Path,
) -> str:
    digest = hashlib.sha256()
    data_dir = data_dir.resolve()
    for sample in samples:
        resolved = sample.image_path.resolve()
        try:
            path_value = resolved.relative_to(data_dir).as_posix()
        except ValueError as exc:
            raise ValueError(f"render path resolves outside data directory: {sample.image_path}") from exc
        file_info = resolved.stat()
        if isinstance(sample, PictureSample):
            content_digest = hashlib.sha256()
            with resolved.open("rb") as stream:
                for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                    content_digest.update(block)
            identity = (
                sample.category,
                sample.subcategory,
                sample.image_id,
                content_digest.hexdigest(),
            )
        else:
            identity = (sample.category, sample.source, sample.model_id, sample.view_id)
        row = "\t".join(
            (path_value, *identity, str(file_info.st_size), str(file_info.st_mtime_ns))
        )
        digest.update(row.encode("utf-8"))
        digest.update(b"\n")
    return digest.hexdigest()


def _model_fingerprint(model_name_or_path: str | Path) -> str:
    model_path = Path(model_name_or_path)
    if not model_path.is_dir():
        raise FileNotFoundError(f"local model directory does not exist: {model_path}")
    digest = hashlib.sha256()
    files = sorted(path for path in model_path.rglob("*") if path.is_file())
    if not files:
        raise ValueError(f"local model directory contains no files: {model_path}")
    for path in files:
        digest.update(path.relative_to(model_path).as_posix().encode("utf-8"))
        digest.update(b"\0")
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def save_feature_cache(
    features: np.ndarray,
    samples: Sequence[RenderSample | PictureSample],
    *,
    output_dir: Path,
    data_dir: Path,
    model_name_or_path: str | Path,
    extraction_info: dict[str, Any],
) -> dict[str, Any]:
    """Atomically save image features and a provenance manifest."""
    features = np.asarray(features, dtype=np.float32)
    if features.ndim != 2 or features.shape[0] != len(samples):
        raise ValueError("feature cache shape does not match samples")
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": 1,
        "data_dir": str(data_dir.resolve()),
        "model_name_or_path": str(model_name_or_path),
        "model_fingerprint": _model_fingerprint(model_name_or_path),
        "sample_fingerprint": _sample_fingerprint(samples, data_dir),
        "sample_count": int(features.shape[0]),
        "feature_dim": int(features.shape[1]),
        "dtype": str(features.dtype),
        "feature_file": "image_features.npy",
        "extraction": extraction_info,
    }
    feature_path = output_dir / "image_features.npy"
    feature_temporary = output_dir / ".image_features.npy.tmp"
    with feature_temporary.open("wb") as stream:
        np.save(stream, features, allow_pickle=False)
    feature_temporary.replace(feature_path)
    manifest_path = output_dir / "feature_manifest.json"
    manifest_temporary = output_dir / ".feature_manifest.json.tmp"
    manifest_temporary.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    manifest_temporary.replace(manifest_path)
    return manifest


def load_feature_cache(
    *,
    output_dir: Path,
    samples: Sequence[RenderSample | PictureSample],
    data_dir: Path,
    model_name_or_path: str | Path,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Load a feature cache after checking its model and ordered sample set."""
    manifest_path = output_dir / "feature_manifest.json"
    feature_path = output_dir / "image_features.npy"
    if not manifest_path.is_file() or not feature_path.is_file():
        raise FileNotFoundError("feature cache is incomplete")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise FeatureCacheValidationError("feature cache manifest is unreadable") from exc
    if not isinstance(manifest, dict):
        raise FeatureCacheValidationError("feature cache manifest is not an object")
    if manifest.get("schema_version") != 1:
        raise FeatureCacheValidationError("unsupported feature cache schema")
    if manifest.get("data_dir") != str(data_dir.resolve()):
        raise FeatureCacheValidationError("feature cache data directory differs")
    if manifest.get("model_name_or_path") != str(model_name_or_path):
        raise FeatureCacheValidationError("feature cache model differs")
    if manifest.get("model_fingerprint") != _model_fingerprint(model_name_or_path):
        raise FeatureCacheValidationError("feature cache model fingerprint differs")
    expected_fingerprint = _sample_fingerprint(samples, data_dir)
    if manifest.get("sample_fingerprint") != expected_fingerprint:
        raise FeatureCacheValidationError("feature cache sample fingerprint differs")
    try:
        features = np.load(feature_path, allow_pickle=False)
    except (OSError, ValueError) as exc:
        raise FeatureCacheValidationError("feature cache array is unreadable") from exc
    expected_shape = (manifest.get("sample_count"), manifest.get("feature_dim"))
    if features.shape != expected_shape or features.shape[0] != len(samples):
        raise FeatureCacheValidationError("feature cache shape differs from manifest")
    if str(features.dtype) != manifest.get("dtype"):
        raise FeatureCacheValidationError("feature cache dtype differs from manifest")
    return np.asarray(features), manifest


def load_rgb_image(path: Path) -> Image.Image:
    """Load an image as RGB, compositing transparent pixels onto white."""
    with Image.open(path) as image:
        image.load()
        has_alpha = "A" in image.getbands() or "transparency" in image.info
        if not has_alpha:
            return image.convert("RGB")
        rgba = image.convert("RGBA")
        background = Image.new("RGBA", rgba.size, (255, 255, 255, 255))
        return Image.alpha_composite(background, rgba).convert("RGB")


def _l2_normalize(features: np.ndarray) -> np.ndarray:
    features = np.asarray(features, dtype=np.float32)
    norms = np.linalg.norm(features, axis=1, keepdims=True)
    if np.any(norms == 0):
        raise ValueError("cannot normalize zero-length feature vectors")
    return features / norms


def aggregate_model_features(
    image_features: np.ndarray,
    samples: Sequence[RenderSample],
) -> tuple[np.ndarray, list[ModelSample]]:
    """Average normalized view embeddings into one normalized model embedding."""
    image_features = np.asarray(image_features, dtype=np.float32)
    if image_features.ndim != 2:
        raise ValueError("image_features must be a two-dimensional array")
    if image_features.shape[0] != len(samples):
        raise ValueError("feature and sample counts differ")

    normalized = _l2_normalize(image_features)
    grouped: dict[tuple[str, str, str], list[int]] = {}
    for index, sample in enumerate(samples):
        grouped.setdefault((sample.category, sample.source, sample.model_id), []).append(index)

    model_features: list[np.ndarray] = []
    models: list[ModelSample] = []
    for (category, source, model_id), indices in grouped.items():
        model_features.append(normalized[indices].mean(axis=0))
        models.append(
            ModelSample(
                category=category,
                source=source,
                model_id=model_id,
                view_count=len(indices),
            )
        )
    if not model_features:
        raise ValueError("no model features to aggregate")
    return _l2_normalize(np.stack(model_features)), models


def extract_vit_features(
    samples: Sequence[RenderSample | PictureSample],
    *,
    model_name_or_path: str | Path,
    batch_size: int,
    device: str,
    num_workers: int,
    use_amp: bool,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Extract normalized image embeddings with a Hugging Face vision transformer."""
    import torch
    from transformers import AutoImageProcessor, AutoModel

    if not samples:
        raise ValueError("no render samples were provided")
    if batch_size < 1:
        raise ValueError("batch_size must be positive")
    if num_workers < 1:
        raise ValueError("num_workers must be positive")
    resolved_device = "cuda:0" if device == "auto" and torch.cuda.is_available() else device
    if resolved_device == "auto":
        resolved_device = "cpu"
    torch_device = torch.device(resolved_device)
    if torch_device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is unavailable")

    model_path = Path(model_name_or_path)
    if not model_path.is_dir():
        raise FileNotFoundError(f"local model directory does not exist: {model_path}")
    model_location = str(model_path.resolve())
    processor = AutoImageProcessor.from_pretrained(model_location, local_files_only=True)
    model = AutoModel.from_pretrained(model_location, local_files_only=True)
    model.eval().to(torch_device)
    amp_enabled = bool(use_amp and torch_device.type == "cuda")
    model_type = str(getattr(model.config, "model_type", type(model).__name__))
    if model_type == "clip":
        patch_size = int(model.config.vision_config.patch_size)
        encoder_label = f"CLIP ViT-B/{patch_size}"
        feature_source = "visual_projection"
    else:
        encoder_label = "DINOv2" if model_type == "dinov2" else model_type
        feature_source = "cls_token"

    batches: list[np.ndarray] = []
    with ThreadPoolExecutor(max_workers=num_workers) as loader_pool:
        for start in range(0, len(samples), batch_size):
            batch_samples = samples[start : start + batch_size]
            images = list(loader_pool.map(load_rgb_image, (sample.image_path for sample in batch_samples)))
            inputs = processor(images=images, return_tensors="pt")
            pixel_values = inputs["pixel_values"].to(torch_device, non_blocking=True)
            with torch.inference_mode():
                with torch.autocast(
                    device_type=torch_device.type,
                    dtype=torch.float16,
                    enabled=amp_enabled,
                ):
                    if model_type == "clip":
                        vision_outputs = model.vision_model(pixel_values=pixel_values)
                        image_features = model.visual_projection(vision_outputs.pooler_output)
                    else:
                        outputs = model(pixel_values=pixel_values)
                        image_features = outputs.last_hidden_state[:, 0]
                image_features = torch.nn.functional.normalize(image_features.float(), dim=1)
            batches.append(image_features.cpu().numpy())
            processed = min(start + batch_size, len(samples))
            print(f"[vit] extracted {processed}/{len(samples)} images", flush=True)

    features = np.concatenate(batches, axis=0).astype(np.float32, copy=False)
    info = {
        "model_name_or_path": model_location,
        "model_type": model_type,
        "encoder_label": encoder_label,
        "feature_source": feature_source,
        "sample_count": int(features.shape[0]),
        "feature_dim": int(features.shape[1]),
        "device": str(torch_device),
        "amp": amp_enabled,
        "batch_size": batch_size,
        "num_workers": num_workers,
    }
    return features, info


def encoder_label_from_extraction(extraction: dict[str, Any]) -> str:
    """Return a display label, including for caches written before labels existed."""
    explicit_label = extraction.get("encoder_label")
    if isinstance(explicit_label, str) and explicit_label.strip():
        return explicit_label.strip()
    model_type = extraction.get("model_type")
    if model_type == "dinov2":
        return "DINOv2"
    if model_type == "clip":
        return "CLIP"
    if isinstance(model_type, str) and model_type.strip():
        return model_type.strip()
    return "ViT"


def compute_tsne(
    features: np.ndarray,
    *,
    requested_perplexity: float,
    random_state: int,
    max_iter: int,
    verbose: int = 0,
    n_jobs: int = -1,
) -> tuple[np.ndarray, dict[str, Any]]:
    """PCA-project features and compute a deterministic two-dimensional t-SNE."""
    from sklearn.decomposition import PCA
    from sklearn.manifold import TSNE

    features = np.asarray(features, dtype=np.float32)
    if features.ndim != 2:
        raise ValueError("features must be a two-dimensional array")
    sample_count, feature_count = features.shape
    if sample_count < 3:
        raise ValueError("t-SNE requires at least three samples")
    if requested_perplexity <= 0:
        raise ValueError("perplexity must be positive")
    if max_iter < 250:
        raise ValueError("max_iter must be at least 250")

    perplexity = min(float(requested_perplexity), max(1.0, (sample_count - 1) / 3.0))
    pca_components = min(50, sample_count - 1, feature_count)
    projected = PCA(n_components=pca_components, random_state=random_state).fit_transform(features)
    coordinates = TSNE(
        n_components=2,
        perplexity=perplexity,
        early_exaggeration=12.0,
        learning_rate="auto",
        max_iter=max_iter,
        init="pca",
        method="barnes_hut",
        angle=0.5,
        random_state=random_state,
        verbose=verbose,
        n_jobs=n_jobs,
    ).fit_transform(projected)
    info = {
        "perplexity": perplexity,
        "pca_components": pca_components,
        "random_state": random_state,
        "max_iter": max_iter,
        "n_jobs": n_jobs,
    }
    return np.asarray(coordinates, dtype=np.float32), info


def save_embedding_plot(
    coordinates: np.ndarray,
    labels: Sequence[str],
    output_path: Path,
    *,
    title: str,
    point_size: float = 7.0,
    alpha: float = 0.65,
    legend_max_labels: int = 20,
    dpi: int = 180,
    label_color_overrides: dict[str, str] | None = None,
    distinct_colors: bool = False,
) -> dict[str, str]:
    """Save a category-colored scatter plot for two-dimensional coordinates."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import to_hex

    coordinates = np.asarray(coordinates, dtype=np.float32)
    if coordinates.ndim != 2 or coordinates.shape[1] != 2:
        raise ValueError("coordinates must have shape (n_samples, 2)")
    if coordinates.shape[0] != len(labels):
        raise ValueError("coordinate and label counts differ")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    unique_labels = sorted(set(labels))
    color_map = plt.get_cmap("turbo", max(2, len(unique_labels)))
    if distinct_colors:
        from matplotlib.colors import hsv_to_rgb

        label_count = len(unique_labels)
        hues = np.arange(label_count, dtype=np.float64) / max(1, label_count)
        hsv_values = np.column_stack(
            (
                hues,
                np.full(label_count, 0.68, dtype=np.float64),
                np.full(label_count, 0.86, dtype=np.float64),
            )
        )
        palette = [to_hex(color) for color in hsv_to_rgb(hsv_values)]
    else:
        palette = [to_hex(color_map(index)) for index in range(len(unique_labels))]
    color_overrides = label_color_overrides or {}
    colors_by_label: dict[str, str] = {}
    figure, axis = plt.subplots(figsize=(14, 11), dpi=dpi)
    for label_index, label in enumerate(unique_labels):
        indices = np.fromiter((value == label for value in labels), dtype=bool)
        color = color_overrides.get(label, palette[label_index])
        colors_by_label[label] = to_hex(color)
        axis.scatter(
            coordinates[indices, 0],
            coordinates[indices, 1],
            s=point_size,
            alpha=alpha,
            color=color,
            edgecolors="none",
            label=label,
            rasterized=True,
        )
    axis.set_title(title, fontsize=17, pad=14)
    axis.set_xlabel("t-SNE 1")
    axis.set_ylabel("t-SNE 2")
    axis.spines[["top", "right"]].set_visible(False)
    axis.grid(color="#d9d9d9", linewidth=0.45, alpha=0.55)
    if len(unique_labels) <= legend_max_labels:
        legend_columns = max(1, math.ceil(len(unique_labels) / 28))
        axis.legend(
            loc="center left",
            bbox_to_anchor=(1.01, 0.5),
            frameon=False,
            markerscale=2.0,
            fontsize=8,
            ncol=legend_columns,
            columnspacing=1.0,
            handletextpad=0.45,
        )
        figure.tight_layout(rect=(0.0, 0.0, 0.82, 1.0))
    else:
        figure.tight_layout()
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)
    if distinct_colors and len(set(colors_by_label.values())) != len(colors_by_label):
        raise RuntimeError("distinct color palette produced duplicate label colors")
    return colors_by_label


def write_coordinates_csv(
    coordinates: np.ndarray,
    records: Iterable[dict[str, Any]],
    output_path: Path,
) -> None:
    """Write t-SNE coordinates and their sample metadata."""
    rows = list(records)
    coordinates = np.asarray(coordinates, dtype=np.float32)
    if coordinates.shape != (len(rows), 2):
        raise ValueError("coordinate and record counts differ")
    metadata_fields = list(rows[0]) if rows else []
    fieldnames = ["tsne_x", "tsne_y", *metadata_fields]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for coordinate, record in zip(coordinates, rows, strict=True):
            writer.writerow(
                {
                    "tsne_x": format(float(coordinate[0]), ".9g"),
                    "tsne_y": format(float(coordinate[1]), ".9g"),
                    **record,
                }
            )


def write_picture_class_index(
    samples: Sequence[PictureSample],
    labels: Sequence[str],
    colors_by_label: dict[str, str],
    output_path: Path,
) -> None:
    """Write the complete leaf-class/color mapping used by a picture plot."""
    if len(samples) != len(labels):
        raise ValueError("picture sample and label counts differ")
    counts: dict[str, int] = {}
    label_parts: dict[str, tuple[str, str]] = {}
    for sample, label in zip(samples, labels, strict=True):
        counts[label] = counts.get(label, 0) + 1
        label_parts[label] = (sample.category, sample.subcategory)
    missing_colors = sorted(set(counts) - set(colors_by_label))
    if missing_colors:
        raise ValueError(f"missing colors for picture classes: {missing_colors[:3]}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["label", "category", "subcategory", "image_count", "color"],
        )
        writer.writeheader()
        for label in sorted(counts):
            category, subcategory = label_parts[label]
            writer.writerow(
                {
                    "label": label,
                    "category": category,
                    "subcategory": subcategory,
                    "image_count": counts[label],
                    "color": colors_by_label[label],
                }
            )


def create_visualizations(
    image_features: np.ndarray,
    samples: Sequence[RenderSample],
    *,
    data_dir: Path,
    output_dir: Path,
    requested_perplexity: float,
    random_state: int,
    max_iter: int,
    encoder_label: str = "DINOv2",
    plot_dpi: int = 180,
    tsne_verbose: int = 0,
    tsne_jobs: int = -1,
) -> dict[str, Any]:
    """Create image-level and model-level t-SNE artifacts."""
    if len(samples) != len(image_features):
        raise ValueError("feature and sample counts differ")
    if plot_dpi < 40:
        raise ValueError("plot_dpi must be at least 40")
    data_dir = data_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    image_features = _l2_normalize(np.asarray(image_features, dtype=np.float32))
    model_features, models = aggregate_model_features(image_features, samples)
    np.save(output_dir / "model_features.npy", model_features, allow_pickle=False)

    print(f"[tsne] reducing {len(samples)} image embeddings", flush=True)
    image_coordinates, image_tsne_info = compute_tsne(
        image_features,
        requested_perplexity=requested_perplexity,
        random_state=random_state,
        max_iter=max_iter,
        verbose=tsne_verbose,
        n_jobs=tsne_jobs,
    )
    print(f"[tsne] reducing {len(models)} model embeddings", flush=True)
    model_coordinates, model_tsne_info = compute_tsne(
        model_features,
        requested_perplexity=requested_perplexity,
        random_state=random_state,
        max_iter=max_iter,
        verbose=tsne_verbose,
        n_jobs=tsne_jobs,
    )

    image_records = []
    for sample in samples:
        try:
            image_path = sample.image_path.resolve().relative_to(data_dir).as_posix()
        except ValueError:
            image_path = str(sample.image_path)
        image_records.append(
            {
                "category": sample.category,
                "source": sample.source,
                "model_id": sample.model_id,
                "view_id": sample.view_id,
                "image_path": image_path,
            }
        )
    model_records = [
        {
            "category": model.category,
            "source": model.source,
            "model_id": model.model_id,
            "view_count": model.view_count,
        }
        for model in models
    ]
    write_coordinates_csv(
        image_coordinates,
        image_records,
        output_dir / "image_tsne_coordinates.csv",
    )
    write_coordinates_csv(
        model_coordinates,
        model_records,
        output_dir / "model_tsne_coordinates.csv",
    )

    view_counts = sorted({model.view_count for model in models})
    view_description = str(view_counts[0]) if len(view_counts) == 1 else "multi"
    plot_specs = [
        (
            image_coordinates,
            [sample.category for sample in samples],
            "image_tsne_by_category.png",
            f"Artiverse {encoder_label} t-SNE: images by category (n={len(samples):,})",
            3.0,
            0.42,
            100,
        ),
        (
            image_coordinates,
            [sample.source for sample in samples],
            "image_tsne_by_source.png",
            f"Artiverse {encoder_label} t-SNE: images by source (n={len(samples):,})",
            3.0,
            0.42,
            20,
        ),
        (
            model_coordinates,
            [model.category for model in models],
            "model_tsne_by_category.png",
            f"Artiverse {encoder_label} t-SNE: {view_description}-view model means (n={len(models):,})",
            10.0,
            0.68,
            100,
        ),
        (
            model_coordinates,
            [model.source for model in models],
            "model_tsne_by_source.png",
            f"Artiverse {encoder_label} t-SNE: model means by source (n={len(models):,})",
            10.0,
            0.68,
            20,
        ),
    ]
    for coordinates, labels, filename, title, point_size, alpha, legend_limit in plot_specs:
        save_embedding_plot(
            coordinates,
            labels,
            output_dir / filename,
            title=title,
            point_size=point_size,
            alpha=alpha,
            legend_max_labels=legend_limit,
            dpi=plot_dpi,
        )

    summary = {
        "schema_version": 1,
        "encoder_label": encoder_label,
        "image_count": len(samples),
        "model_count": len(models),
        "category_count": len({sample.category for sample in samples}),
        "source_count": len({sample.source for sample in samples}),
        "image_tsne": image_tsne_info,
        "model_tsne": model_tsne_info,
        "artifacts": {
            "image_coordinates": "image_tsne_coordinates.csv",
            "image_plot_by_category": "image_tsne_by_category.png",
            "image_plot_by_source": "image_tsne_by_source.png",
            "model_features": "model_features.npy",
            "model_coordinates": "model_tsne_coordinates.csv",
            "model_plot_by_category": "model_tsne_by_category.png",
            "model_plot_by_source": "model_tsne_by_source.png",
        },
    }
    (output_dir / "visualization_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    return summary


def create_picture_visualizations(
    image_features: np.ndarray,
    samples: Sequence[PictureSample],
    *,
    data_dir: Path,
    output_dir: Path,
    requested_perplexity: float,
    random_state: int,
    max_iter: int,
    encoder_label: str = "DINOv2",
    plot_dpi: int = 180,
    tsne_verbose: int = 0,
    tsne_jobs: int = -1,
) -> dict[str, Any]:
    """Create image-only t-SNE artifacts for Articraft reference pictures."""
    if len(samples) != len(image_features):
        raise ValueError("feature and sample counts differ")
    if plot_dpi < 40:
        raise ValueError("plot_dpi must be at least 40")
    data_dir = data_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    image_features = _l2_normalize(np.asarray(image_features, dtype=np.float32))

    print(f"[tsne] reducing {len(samples)} picture embeddings", flush=True)
    image_coordinates, image_tsne_info = compute_tsne(
        image_features,
        requested_perplexity=requested_perplexity,
        random_state=random_state,
        max_iter=max_iter,
        verbose=tsne_verbose,
        n_jobs=tsne_jobs,
    )

    image_records = []
    for sample in samples:
        try:
            image_path = sample.image_path.resolve().relative_to(data_dir).as_posix()
        except ValueError:
            image_path = str(sample.image_path)
        image_records.append(
            {
                "category": sample.category,
                "subcategory": sample.subcategory,
                "image_id": sample.image_id,
                "image_path": image_path,
            }
        )
    write_coordinates_csv(
        image_coordinates,
        image_records,
        output_dir / "image_tsne_coordinates.csv",
    )

    category_labels = [
        (
            PICTURE_UNMAPPED_LABEL
            if sample.category == PICTURE_UNMAPPED_CATEGORY
            else sample.category
        )
        for sample in samples
    ]
    subcategory_label_min_count = 5
    grouped_subcategory_labels = build_picture_subcategory_plot_labels(
        samples,
        min_count=subcategory_label_min_count,
    )
    leaf_subcategory_labels = build_picture_leaf_plot_labels(samples)
    taxonomy_summary = build_picture_taxonomy_summary(samples)
    save_embedding_plot(
        image_coordinates,
        category_labels,
        output_dir / "image_tsne_by_category.png",
        title=(
            f"Articraft picture {encoder_label} t-SNE: images by top-level category "
            f"(n={len(samples):,})"
        ),
        point_size=11.0,
        alpha=0.7,
        legend_max_labels=100,
        dpi=plot_dpi,
        label_color_overrides={PICTURE_UNMAPPED_LABEL: "#777777"},
    )
    leaf_colors = save_embedding_plot(
        image_coordinates,
        leaf_subcategory_labels,
        output_dir / "image_tsne_by_subcategory.png",
        title=(
            f"Articraft picture {encoder_label} t-SNE: all leaf classes "
            f"(leaf classes={taxonomy_summary['leaf_subcategory_count']:,}; "
            f"taxonomy nodes={taxonomy_summary['taxonomy_node_count']:,}; "
            f"n={len(samples):,})"
        ),
        point_size=11.0,
        alpha=0.7,
        legend_max_labels=100,
        dpi=plot_dpi,
        distinct_colors=True,
    )
    write_picture_class_index(
        samples,
        leaf_subcategory_labels,
        leaf_colors,
        output_dir / "image_tsne_class_index.csv",
    )
    save_embedding_plot(
        image_coordinates,
        grouped_subcategory_labels,
        output_dir / "image_tsne_by_subcategory_grouped.png",
        title=(
            f"Articraft picture {encoder_label} t-SNE: grouped subcategories "
            f"(n>={subcategory_label_min_count}; n={len(samples):,})"
        ),
        point_size=11.0,
        alpha=0.7,
        legend_max_labels=100,
        dpi=plot_dpi,
        label_color_overrides={
            PICTURE_UNMAPPED_LABEL: "#777777",
            f"Other subcategories (n<{subcategory_label_min_count})": "#c7c7c7",
        },
    )

    summary = {
        "schema_version": 1,
        "dataset_format": "picture",
        "encoder_label": encoder_label,
        "image_count": len(samples),
        **taxonomy_summary,
        # Keep the original field name as an alias for downstream consumers.
        "subcategory_count": taxonomy_summary["leaf_subcategory_count"],
        "subcategory_label_min_count": subcategory_label_min_count,
        "displayed_subcategory_label_count": len(set(leaf_subcategory_labels)),
        "displayed_grouped_subcategory_label_count": len(
            set(grouped_subcategory_labels)
        ),
        "unmapped_batch_image_count": sum(
            sample.category == PICTURE_UNMAPPED_CATEGORY for sample in samples
        ),
        "image_tsne": image_tsne_info,
        "artifacts": {
            "image_coordinates": "image_tsne_coordinates.csv",
            "image_plot_by_category": "image_tsne_by_category.png",
            "image_plot_by_subcategory": "image_tsne_by_subcategory.png",
            "image_plot_by_subcategory_grouped": "image_tsne_by_subcategory_grouped.png",
            "image_class_index": "image_tsne_class_index.csv",
        },
    }
    (output_dir / "visualization_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    return summary


def run_pipeline(config: PipelineConfig) -> dict[str, Any]:
    """Run discovery, feature extraction/cache loading, and visualization."""
    data_dir = config.data_dir.resolve(strict=True)
    output_dir = config.output_dir.resolve()
    _validate_output_dataset_format(output_dir, config.dataset_format)
    if config.dataset_format == "artiverse":
        samples: list[RenderSample] | list[PictureSample] = discover_render_samples(data_dir)
        dataset_summary = validate_artiverse_samples(
            samples,
            expected_views=config.expected_views,
        )
        print(
            "[dataset] "
            f"{dataset_summary['image_count']} images, "
            f"{dataset_summary['model_count']} models, "
            f"{dataset_summary['category_count']} categories, "
            f"{dataset_summary['source_count']} sources",
            flush=True,
        )
    elif config.dataset_format == "picture":
        samples = discover_picture_samples(data_dir)
        dataset_summary = validate_picture_samples(samples)
        dataset_summary.update(build_picture_taxonomy_summary(samples))
        print(
            "[dataset] "
            f"{dataset_summary['image_count']} pictures, "
            f"{dataset_summary['category_count']} top-level categories, "
            f"{dataset_summary['leaf_subcategory_count']} leaf subcategories, "
            f"{dataset_summary['taxonomy_node_count']} taxonomy nodes",
            flush=True,
        )
    else:
        raise ValueError(f"unsupported dataset format: {config.dataset_format}")
    output_dir.mkdir(parents=True, exist_ok=True)

    cache_reused = False
    if not config.force_extract:
        try:
            image_features, feature_manifest = load_feature_cache(
                output_dir=output_dir,
                samples=samples,
                data_dir=data_dir,
                model_name_or_path=config.model_name_or_path,
            )
            cache_reused = True
            print(f"[vit] reused {output_dir / 'image_features.npy'}", flush=True)
        except FileNotFoundError:
            cache_reused = False
        except FeatureCacheValidationError as exc:
            cache_reused = False
            print(f"[vit] rejected stale feature cache: {exc}", flush=True)
    if not cache_reused:
        image_features, extraction_info = extract_vit_features(
            samples,
            model_name_or_path=config.model_name_or_path,
            batch_size=config.batch_size,
            device=config.device,
            num_workers=config.num_workers,
            use_amp=config.use_amp,
        )
        feature_manifest = save_feature_cache(
            image_features,
            samples,
            output_dir=output_dir,
            data_dir=data_dir,
            model_name_or_path=config.model_name_or_path,
            extraction_info=extraction_info,
        )

    encoder_label = encoder_label_from_extraction(feature_manifest["extraction"])
    if config.dataset_format == "picture":
        visualization_summary = create_picture_visualizations(
            image_features,
            samples,
            data_dir=data_dir,
            output_dir=output_dir,
            requested_perplexity=config.requested_perplexity,
            random_state=config.random_state,
            max_iter=config.tsne_max_iter,
            encoder_label=encoder_label,
            plot_dpi=config.plot_dpi,
            tsne_verbose=config.tsne_verbose,
            tsne_jobs=config.tsne_jobs,
        )
    else:
        visualization_summary = create_visualizations(
            image_features,
            samples,
            data_dir=data_dir,
            output_dir=output_dir,
            requested_perplexity=config.requested_perplexity,
            random_state=config.random_state,
            max_iter=config.tsne_max_iter,
            encoder_label=encoder_label,
            plot_dpi=config.plot_dpi,
            tsne_verbose=config.tsne_verbose,
            tsne_jobs=config.tsne_jobs,
        )
    run_manifest = {
        "schema_version": 1,
        "dataset": dataset_summary,
        "feature_cache_reused": cache_reused,
        "feature_manifest": feature_manifest,
        "visualization": visualization_summary,
        "config": {
            "dataset_format": config.dataset_format,
            "data_dir": str(data_dir),
            "output_dir": str(output_dir),
            "model_name_or_path": str(config.model_name_or_path),
            "batch_size": config.batch_size,
            "device": config.device,
            "num_workers": config.num_workers,
            "use_amp": config.use_amp,
            "expected_views": config.expected_views,
            "requested_perplexity": config.requested_perplexity,
            "random_state": config.random_state,
            "tsne_max_iter": config.tsne_max_iter,
            "plot_dpi": config.plot_dpi,
            "tsne_verbose": config.tsne_verbose,
            "tsne_jobs": config.tsne_jobs,
            "force_extract": config.force_extract,
        },
    }
    (output_dir / "run_manifest.json").write_text(
        json.dumps(run_manifest, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    return run_manifest


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Extract vision-transformer image features and create t-SNE plots."
    )
    parser.add_argument(
        "--dataset-format",
        choices=("artiverse", "picture"),
        default="artiverse",
        help="Input layout: Artiverse multi-view renders or Articraft reference pictures.",
    )
    parser.add_argument("--data-dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--model", dest="model_name_or_path", default=DEFAULT_MODEL)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--device", default="auto", help="Torch device, for example auto, cpu, or cuda:0.")
    parser.add_argument("--num-workers", type=int, default=min(16, os.cpu_count() or 1))
    parser.add_argument("--no-amp", action="store_false", dest="use_amp", help="Disable CUDA fp16 autocast.")
    parser.add_argument("--expected-views", type=int, default=16)
    parser.add_argument("--perplexity", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--tsne-max-iter", type=int, default=1_000)
    parser.add_argument("--plot-dpi", type=int, default=180)
    parser.add_argument("--tsne-verbose", type=int, choices=(0, 1, 2), default=1)
    parser.add_argument(
        "--tsne-jobs",
        type=int,
        default=DEFAULT_TSNE_JOBS,
        help="CPU workers for t-SNE neighbor search.",
    )
    parser.add_argument(
        "--force-extract",
        action="store_true",
        help="Ignore a compatible feature cache and extract ViT features again.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    data_dir, output_dir = resolve_dataset_paths(
        args.dataset_format,
        args.data_dir,
        args.output_dir,
    )
    config = PipelineConfig(
        data_dir=data_dir,
        output_dir=output_dir,
        model_name_or_path=args.model_name_or_path,
        batch_size=args.batch_size,
        device=args.device,
        num_workers=args.num_workers,
        use_amp=args.use_amp,
        expected_views=args.expected_views,
        requested_perplexity=args.perplexity,
        random_state=args.seed,
        tsne_max_iter=args.tsne_max_iter,
        plot_dpi=args.plot_dpi,
        tsne_verbose=args.tsne_verbose,
        force_extract=args.force_extract,
        dataset_format=args.dataset_format,
        tsne_jobs=args.tsne_jobs,
    )
    manifest = run_pipeline(config)
    print(json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
