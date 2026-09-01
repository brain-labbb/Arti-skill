#!/usr/bin/env python3
"""Aggregate Artiverse model embeddings by category and visualize the centroids."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

for _thread_variable in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_thread_variable, "1")

import numpy as np
from PIL import Image, ImageStat


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_DINOV2_DIR = REPO_ROOT / "exp" / "runtime" / "artiverse_vit_tsne_dinov2_base"
DEFAULT_CLIP_DIR = REPO_ROOT / "exp" / "runtime" / "artiverse_vit_tsne_clip_vit_b32"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exp" / "runtime" / "artiverse84_category_tsne"
DEFAULT_TSNE_JOBS = min(64, os.cpu_count() or 1)


@dataclass(frozen=True, slots=True)
class ModelRecord:
    category: str
    source: str
    model_id: str
    view_count: int

    @property
    def identity(self) -> tuple[str, str, str]:
        return (self.category, self.source, self.model_id)


@dataclass(frozen=True, slots=True)
class CategoryRecord:
    category_id: int
    category: str
    model_count: int
    source_count: int
    source_counts: dict[str, int]


@dataclass(frozen=True, slots=True)
class EncoderRun:
    key: str
    label: str
    run_dir: Path
    model_features: np.ndarray
    records: tuple[ModelRecord, ...]
    receipt: dict[str, Any]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def l2_normalize(features: np.ndarray) -> np.ndarray:
    features = np.asarray(features, dtype=np.float32)
    if features.ndim != 2:
        raise ValueError("features must be a two-dimensional array")
    if not np.isfinite(features).all():
        raise ValueError("features contain non-finite values")
    norms = np.linalg.norm(features, axis=1, keepdims=True)
    if np.any(norms == 0):
        raise ValueError("cannot normalize zero-length feature vectors")
    return np.asarray(features / norms, dtype=np.float32)


def _encoder_label(manifest: dict[str, Any], key: str) -> str:
    extraction = manifest.get("feature_manifest", {}).get("extraction", {})
    explicit = extraction.get("encoder_label")
    if isinstance(explicit, str) and explicit.strip():
        return explicit.strip()
    if key == "dinov2":
        return "DINOv2"
    if key == "clip":
        return "CLIP ViT-B/32"
    return key


def load_encoder_run(run_dir: Path, *, key: str) -> EncoderRun:
    """Load model embeddings and their ordered metadata from one frozen run."""
    run_dir = run_dir.resolve(strict=True)
    manifest_path = run_dir / "run_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"missing source run manifest: {manifest_path}")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"unreadable source run manifest: {manifest_path}") from exc
    if not isinstance(manifest, dict):
        raise ValueError(f"source run manifest is not an object: {manifest_path}")
    extraction = manifest.get("feature_manifest", {}).get("extraction", {})
    expected_model_types = {"dinov2": "dinov2", "clip": "clip"}
    expected_model_type = expected_model_types.get(key)
    if expected_model_type is None:
        raise ValueError(f"unsupported encoder key: {key}")
    if extraction.get("model_type") != expected_model_type:
        raise ValueError(
            f"source run model_type does not match {key}: {extraction.get('model_type')!r}"
        )

    artifacts = manifest.get("visualization", {}).get("artifacts", {})
    feature_name = artifacts.get("model_features", "model_features.npy")
    coordinates_name = artifacts.get("model_coordinates", "model_tsne_coordinates.csv")
    if Path(feature_name).name != feature_name or Path(coordinates_name).name != coordinates_name:
        raise ValueError("source artifact names must be plain filenames")
    feature_path = run_dir / feature_name
    coordinates_path = run_dir / coordinates_name
    if not feature_path.is_file() or not coordinates_path.is_file():
        raise FileNotFoundError(f"source model artifacts are incomplete: {run_dir}")

    try:
        model_features = np.load(feature_path, allow_pickle=False)
    except (OSError, ValueError) as exc:
        raise ValueError(f"unreadable source model features: {feature_path}") from exc
    model_features = np.asarray(model_features)
    if model_features.ndim != 2 or model_features.shape[0] < 3:
        raise ValueError(f"invalid source model feature shape: {model_features.shape}")
    if not np.issubdtype(model_features.dtype, np.floating):
        raise ValueError(f"source model features are not floating point: {model_features.dtype}")
    if not np.isfinite(model_features).all():
        raise ValueError("source model features contain non-finite values")

    records: list[ModelRecord] = []
    identities: set[tuple[str, str, str]] = set()
    with coordinates_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"category", "source", "model_id", "view_count"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"source coordinate table lacks required fields: {coordinates_path}")
        for row_number, row in enumerate(reader, start=2):
            category = row["category"].strip()
            source = row["source"].strip()
            model_id = row["model_id"].strip()
            if not category or not source or not model_id:
                raise ValueError(f"empty model identity at {coordinates_path}:{row_number}")
            try:
                view_count = int(row["view_count"])
            except ValueError as exc:
                raise ValueError(
                    f"invalid view_count at {coordinates_path}:{row_number}"
                ) from exc
            if view_count < 1:
                raise ValueError(f"non-positive view_count at {coordinates_path}:{row_number}")
            record = ModelRecord(category, source, model_id, view_count)
            if record.identity in identities:
                raise ValueError(f"duplicate model identity in {coordinates_path}: {record.identity}")
            identities.add(record.identity)
            records.append(record)

    if model_features.shape[0] != len(records):
        raise ValueError(
            f"source feature/metadata count mismatch: {model_features.shape[0]} != {len(records)}"
        )
    dataset = manifest.get("dataset", {})
    expected_models = dataset.get("model_count")
    expected_categories = dataset.get("category_count")
    observed_categories = len({record.category for record in records})
    if expected_models is not None and expected_models != len(records):
        raise ValueError("source manifest model count differs from model artifacts")
    if expected_categories is not None and expected_categories != observed_categories:
        raise ValueError("source manifest category count differs from model artifacts")

    norms = np.linalg.norm(model_features.astype(np.float32, copy=False), axis=1)
    receipt = {
        "run_dir": str(run_dir),
        "run_manifest": {
            "path": str(manifest_path),
            "sha256": sha256_file(manifest_path),
            "size_bytes": manifest_path.stat().st_size,
        },
        "model_features": {
            "path": str(feature_path),
            "sha256": sha256_file(feature_path),
            "size_bytes": feature_path.stat().st_size,
            "shape": list(model_features.shape),
            "dtype": str(model_features.dtype),
            "finite": True,
            "norm_min": float(norms.min()),
            "norm_max": float(norms.max()),
        },
        "model_metadata": {
            "path": str(coordinates_path),
            "sha256": sha256_file(coordinates_path),
            "size_bytes": coordinates_path.stat().st_size,
            "row_count": len(records),
            "category_count": observed_categories,
        },
    }
    return EncoderRun(
        key=key,
        label=_encoder_label(manifest, key),
        run_dir=run_dir,
        model_features=np.asarray(model_features, dtype=np.float32),
        records=tuple(records),
        receipt=receipt,
    )


def align_encoder_runs(runs: Sequence[EncoderRun]) -> list[EncoderRun]:
    """Return encoder runs in one canonical model order after exact roster matching."""
    if not runs:
        raise ValueError("at least one encoder run is required")
    identity_sets = [{record.identity for record in run.records} for run in runs]
    reference = identity_sets[0]
    for run, identities in zip(runs[1:], identity_sets[1:], strict=True):
        if identities != reference:
            missing = sorted(reference - identities)[:3]
            extra = sorted(identities - reference)[:3]
            raise ValueError(
                f"encoder model rosters differ for {run.key}: missing={missing}, extra={extra}"
            )
    canonical_identities = sorted(reference)
    aligned: list[EncoderRun] = []
    reference_views: dict[tuple[str, str, str], int] | None = None
    for run in runs:
        index = {record.identity: position for position, record in enumerate(run.records)}
        positions = [index[identity] for identity in canonical_identities]
        records = tuple(run.records[position] for position in positions)
        view_counts = {record.identity: record.view_count for record in records}
        if reference_views is None:
            reference_views = view_counts
        elif view_counts != reference_views:
            raise ValueError(f"encoder view counts differ for {run.key}")
        aligned.append(
            EncoderRun(
                key=run.key,
                label=run.label,
                run_dir=run.run_dir,
                model_features=np.asarray(run.model_features[positions], dtype=np.float32),
                records=records,
                receipt=run.receipt,
            )
        )
    return aligned


def aggregate_category_features(
    model_features: np.ndarray,
    records: Sequence[ModelRecord],
) -> tuple[np.ndarray, list[CategoryRecord]]:
    """Average normalized model embeddings into one normalized category centroid."""
    if len(model_features) != len(records):
        raise ValueError("model feature and metadata counts differ")
    normalized = l2_normalize(model_features)
    grouped: dict[str, list[int]] = {}
    for index, record in enumerate(records):
        grouped.setdefault(record.category, []).append(index)
    if len(grouped) < 3:
        raise ValueError("category t-SNE requires at least three categories")

    centroids: list[np.ndarray] = []
    category_records: list[CategoryRecord] = []
    for category_id, category in enumerate(sorted(grouped), start=1):
        indices = grouped[category]
        centroids.append(normalized[indices].mean(axis=0))
        source_counts = dict(sorted(Counter(records[index].source for index in indices).items()))
        category_records.append(
            CategoryRecord(
                category_id=category_id,
                category=category,
                model_count=len(indices),
                source_count=len(source_counts),
                source_counts=source_counts,
            )
        )
    return l2_normalize(np.stack(centroids)), category_records


def compute_tsne(
    features: np.ndarray,
    *,
    requested_perplexity: float,
    random_state: int,
    max_iter: int,
    n_jobs: int,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Apply PCA50 and deterministic Barnes-Hut t-SNE to category features."""
    from sklearn.decomposition import PCA
    from sklearn.manifold import TSNE

    features = l2_normalize(features)
    sample_count, feature_count = features.shape
    if requested_perplexity <= 0:
        raise ValueError("perplexity must be positive")
    if max_iter < 250:
        raise ValueError("max_iter must be at least 250")
    perplexity = min(float(requested_perplexity), float(sample_count - 1))
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
        verbose=0,
        n_jobs=n_jobs,
    ).fit_transform(projected)
    return np.asarray(coordinates, dtype=np.float32), {
        "input_sample_count": sample_count,
        "requested_perplexity": float(requested_perplexity),
        "perplexity": perplexity,
        "pca_components": pca_components,
        "random_state": random_state,
        "max_iter": max_iter,
        "n_jobs": n_jobs,
    }


def _atomic_write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    _atomic_write_text(
        path,
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
    )


def _atomic_save_npy(path: Path, array: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("wb") as stream:
        np.save(stream, np.asarray(array, dtype=np.float32), allow_pickle=False)
    temporary.replace(path)


def _atomic_write_csv(path: Path, fieldnames: Sequence[str], rows: Sequence[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def _category_colors(category_count: int) -> list[str]:
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib.colors import to_hex
    import matplotlib.pyplot as plt

    color_map = plt.get_cmap("turbo")
    return [to_hex(color_map(value)) for value in np.linspace(0.03, 0.97, category_count)]


def _draw_embedding_axis(
    axis: Any,
    coordinates: np.ndarray,
    category_records: Sequence[CategoryRecord],
    colors: Sequence[str],
    *,
    title: str,
    point_size: float,
) -> None:
    from matplotlib.colors import to_rgb

    for coordinate, record, color in zip(coordinates, category_records, colors, strict=True):
        axis.scatter(
            [coordinate[0]],
            [coordinate[1]],
            s=point_size,
            color=color,
            edgecolors="#202020",
            linewidths=0.45,
            zorder=2,
        )
        axis.text(
            float(coordinate[0]),
            float(coordinate[1]),
            f"{record.category_id:02d}",
            ha="center",
            va="center",
            fontsize=4.8,
            color=(
                "#ffffff"
                if sum(weight * channel for weight, channel in zip(
                    (0.2126, 0.7152, 0.0722), to_rgb(color), strict=True
                )) < 0.48
                else "#111111"
            ),
            zorder=3,
        )
    axis.set_title(title, fontsize=15, pad=12)
    axis.set_xlabel("t-SNE 1")
    axis.set_ylabel("t-SNE 2")
    axis.spines[["top", "right"]].set_visible(False)
    axis.grid(color="#d9d9d9", linewidth=0.45, alpha=0.55)


def _add_category_key(
    figure: Any,
    category_records: Sequence[CategoryRecord],
    colors: Sequence[str],
    *,
    x_start: float,
    x_end: float,
    y_top: float,
    y_bottom: float,
    columns: int,
    fontsize: float,
) -> None:
    rows = math.ceil(len(category_records) / columns)
    x_step = (x_end - x_start) / columns
    y_step = (y_top - y_bottom) / max(1, rows - 1)
    for index, (record, color) in enumerate(zip(category_records, colors, strict=True)):
        column = index // rows
        row = index % rows
        x = x_start + column * x_step
        y = y_top - row * y_step
        figure.text(
            x,
            y,
            f"{record.category_id:02d}",
            color=color,
            fontsize=fontsize,
            fontweight="bold",
            ha="left",
            va="center",
        )
        figure.text(
            x + 0.014,
            y,
            f"{record.category}  (n={record.model_count})",
            color="#303030",
            fontsize=fontsize,
            ha="left",
            va="center",
        )


def _save_figure_atomic(figure: Any, path: Path, *, dpi: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    figure.savefig(temporary, format="png", dpi=dpi, facecolor="white")
    temporary.replace(path)


def save_encoder_plot(
    coordinates: np.ndarray,
    category_records: Sequence[CategoryRecord],
    colors: Sequence[str],
    output_path: Path,
    *,
    encoder_label: str,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(19, 12), dpi=dpi)
    figure.subplots_adjust(left=0.06, right=0.63, top=0.91, bottom=0.08)
    _draw_embedding_axis(
        axis,
        coordinates,
        category_records,
        colors,
        title=f"Artiverse {encoder_label} t-SNE: category centroids (n={len(category_records)})",
        point_size=110.0,
    )
    figure.text(
        0.06,
        0.025,
        "Each point is the normalized mean of all 16-view model embeddings in one category.",
        fontsize=8,
        color="#4a4a4a",
        ha="left",
    )
    _add_category_key(
        figure,
        category_records,
        colors,
        x_start=0.655,
        x_end=0.985,
        y_top=0.89,
        y_bottom=0.08,
        columns=3,
        fontsize=6.4,
    )
    _save_figure_atomic(figure, output_path, dpi=dpi)
    plt.close(figure)


def save_comparison_plot(
    encoder_results: Sequence[dict[str, Any]],
    category_records: Sequence[CategoryRecord],
    colors: Sequence[str],
    output_path: Path,
    *,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, len(encoder_results), figsize=(24, 14), dpi=dpi)
    axes = np.atleast_1d(axes)
    figure.subplots_adjust(left=0.045, right=0.985, top=0.90, bottom=0.27, wspace=0.16)
    figure.suptitle(
        f"Artiverse t-SNE: {len(category_records)} category centroids",
        fontsize=19,
        y=0.965,
    )
    for axis, result in zip(axes, encoder_results, strict=True):
        _draw_embedding_axis(
            axis,
            result["coordinates"],
            category_records,
            colors,
            title=result["label"],
            point_size=105.0,
        )
    _add_category_key(
        figure,
        category_records,
        colors,
        x_start=0.045,
        x_end=0.985,
        y_top=0.225,
        y_bottom=0.045,
        columns=6,
        fontsize=6.2,
    )
    figure.text(
        0.5,
        0.012,
        "Category IDs and colors match across panels. t-SNE is fitted independently per encoder; axes are not aligned.",
        fontsize=8,
        color="#4a4a4a",
        ha="center",
    )
    _save_figure_atomic(figure, output_path, dpi=dpi)
    plt.close(figure)


def _coordinate_rows(
    coordinates: np.ndarray,
    category_records: Sequence[CategoryRecord],
) -> list[dict[str, Any]]:
    return [
        {
            "category_id": f"{record.category_id:02d}",
            "category": record.category,
            "model_count": record.model_count,
            "source_count": record.source_count,
            "source_counts_json": json.dumps(record.source_counts, sort_keys=True),
            "tsne_x": format(float(coordinate[0]), ".9g"),
            "tsne_y": format(float(coordinate[1]), ".9g"),
        }
        for coordinate, record in zip(coordinates, category_records, strict=True)
    ]


def audit_outputs(
    output_dir: Path,
    encoder_results: Sequence[dict[str, Any]],
    *,
    category_records: Sequence[CategoryRecord],
) -> dict[str, Any]:
    category_count = len(category_records)
    expected_metadata = [
        (
            f"{record.category_id:02d}",
            record.category,
            str(record.model_count),
            str(record.source_count),
        )
        for record in category_records
    ]
    checks: dict[str, bool] = {}
    artifacts: dict[str, dict[str, Any]] = {}
    category_index_path = output_dir / "category_index.csv"
    with category_index_path.open("r", encoding="utf-8", newline="") as stream:
        category_index_rows = list(csv.DictReader(stream))
    checks["category_index_rows"] = len(category_index_rows) == category_count
    checks["category_index_unique_ids"] = len(
        {row["category_id"] for row in category_index_rows}
    ) == category_count
    checks["category_index_metadata"] = [
        (row["category_id"], row["category"], row["model_count"], row["source_count"])
        for row in category_index_rows
    ] == expected_metadata
    artifacts["category_index.csv"] = {
        "sha256": sha256_file(category_index_path),
        "size_bytes": category_index_path.stat().st_size,
    }
    expected_pngs = [output_dir / "tsne_category_comparison.png"]
    for result in encoder_results:
        encoder_dir = output_dir / result["key"]
        feature_path = encoder_dir / "category_features.npy"
        coordinate_path = encoder_dir / "tsne_coordinates.csv"
        plot_path = encoder_dir / "tsne_by_category.png"
        features = np.load(feature_path, allow_pickle=False)
        with coordinate_path.open("r", encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        checks[f"{result['key']}_feature_shape"] = features.shape == tuple(
            result["category_feature_shape"]
        )
        checks[f"{result['key']}_feature_finite"] = bool(np.isfinite(features).all())
        checks[f"{result['key']}_feature_normalized"] = bool(
            np.allclose(np.linalg.norm(features, axis=1), 1.0, rtol=2e-5, atol=2e-5)
        )
        checks[f"{result['key']}_coordinate_rows"] = len(rows) == category_count
        checks[f"{result['key']}_coordinate_metadata"] = [
            (row["category_id"], row["category"], row["model_count"], row["source_count"])
            for row in rows
        ] == expected_metadata
        checks[f"{result['key']}_coordinate_finite"] = all(
            math.isfinite(float(row[axis])) for row in rows for axis in ("tsne_x", "tsne_y")
        )
        expected_pngs.append(plot_path)
        for path in (feature_path, coordinate_path):
            artifacts[str(path.relative_to(output_dir))] = {
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
    for path in expected_pngs:
        artifact_key = path.relative_to(output_dir).with_suffix("").as_posix().replace("/", "_")
        with Image.open(path) as image:
            image.load()
            gray = image.convert("L")
            standard_deviation = ImageStat.Stat(gray).stddev[0]
            checks[f"{artifact_key}_dimensions"] = (
                image.width >= 1_000 and image.height >= 700
            )
            checks[f"{artifact_key}_nonblank"] = standard_deviation > 1.0
            artifacts[str(path.relative_to(output_dir))] = {
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
                "width": image.width,
                "height": image.height,
                "grayscale_std": standard_deviation,
            }
    manifest_path = output_dir / "run_manifest.json"
    artifacts["run_manifest.json"] = {
        "sha256": sha256_file(manifest_path),
        "size_bytes": manifest_path.stat().st_size,
    }
    return {
        "schema_version": 1,
        "pass": all(checks.values()),
        "checks": checks,
        "artifacts": artifacts,
    }


def run(
    *,
    dinov2_dir: Path,
    clip_dir: Path,
    output_dir: Path,
    requested_perplexity: float,
    random_state: int,
    max_iter: int,
    n_jobs: int,
    plot_dpi: int,
) -> dict[str, Any]:
    if plot_dpi < 40:
        raise ValueError("plot_dpi must be at least 40")
    output_dir = output_dir.resolve()
    dinov2_dir = dinov2_dir.resolve(strict=True)
    clip_dir = clip_dir.resolve(strict=True)
    for source_dir in (dinov2_dir, clip_dir):
        if (
            output_dir == source_dir
            or output_dir.is_relative_to(source_dir)
            or source_dir.is_relative_to(output_dir)
        ):
            raise ValueError(f"output and source directories must be disjoint: {source_dir}")
    source_runs = align_encoder_runs(
        [
            load_encoder_run(dinov2_dir, key="dinov2"),
            load_encoder_run(clip_dir, key="clip"),
        ]
    )
    model_records = source_runs[0].records
    model_count = len(model_records)
    expected_views = sorted({record.view_count for record in model_records})
    if expected_views != [16]:
        raise ValueError(f"expected exactly 16 views per Artiverse model, found {expected_views}")

    encoder_results: list[dict[str, Any]] = []
    reference_categories: list[CategoryRecord] | None = None
    for source_run in source_runs:
        category_features, category_records = aggregate_category_features(
            source_run.model_features,
            source_run.records,
        )
        if reference_categories is None:
            reference_categories = category_records
        elif category_records != reference_categories:
            raise ValueError("encoder category aggregates differ")
        coordinates, tsne_info = compute_tsne(
            category_features,
            requested_perplexity=requested_perplexity,
            random_state=random_state,
            max_iter=max_iter,
            n_jobs=n_jobs,
        )
        encoder_dir = output_dir / source_run.key
        feature_path = encoder_dir / "category_features.npy"
        coordinate_path = encoder_dir / "tsne_coordinates.csv"
        plot_path = encoder_dir / "tsne_by_category.png"
        _atomic_save_npy(feature_path, category_features)
        coordinate_rows = _coordinate_rows(coordinates, category_records)
        _atomic_write_csv(coordinate_path, list(coordinate_rows[0]), coordinate_rows)
        encoder_results.append(
            {
                "key": source_run.key,
                "label": source_run.label,
                "coordinates": coordinates,
                "category_feature_shape": list(category_features.shape),
                "tsne": tsne_info,
                "source_receipt": source_run.receipt,
                "artifacts": {
                    "category_features": str(feature_path.relative_to(output_dir)),
                    "coordinates": str(coordinate_path.relative_to(output_dir)),
                    "plot": str(plot_path.relative_to(output_dir)),
                },
            }
        )

    assert reference_categories is not None
    colors = _category_colors(len(reference_categories))
    for result in encoder_results:
        save_encoder_plot(
            result["coordinates"],
            reference_categories,
            colors,
            output_dir / result["artifacts"]["plot"],
            encoder_label=result["label"],
            dpi=plot_dpi,
        )
    save_comparison_plot(
        encoder_results,
        reference_categories,
        colors,
        output_dir / "tsne_category_comparison.png",
        dpi=plot_dpi,
    )

    index_rows = [
        {
            "category_id": f"{record.category_id:02d}",
            "category": record.category,
            "model_count": record.model_count,
            "source_count": record.source_count,
            "source_counts_json": json.dumps(record.source_counts, sort_keys=True),
            "color": color,
        }
        for record, color in zip(reference_categories, colors, strict=True)
    ]
    _atomic_write_csv(output_dir / "category_index.csv", list(index_rows[0]), index_rows)

    serializable_results = []
    for result in encoder_results:
        serializable = {key: value for key, value in result.items() if key != "coordinates"}
        serializable_results.append(serializable)
    manifest = {
        "schema_version": 1,
        "protocol": {
            "sample_unit": "one Artiverse category centroid",
            "model_feature_input": "normalized mean of 16 normalized view embeddings",
            "category_aggregation": "equal-weight mean of normalized model embeddings, then L2 normalization",
            "coordinate_policy": "PCA50 followed by an independently fitted t-SNE for each encoder",
            "source_2d_coordinates_used_for_aggregation": False,
        },
        "interpretation_limits": [
            "Category centroids suppress within-category variation.",
            "Categories have unequal support and remain equally weighted in t-SNE.",
            "This category-centroid plot is not directly comparable to a one-view-per-class plot.",
        ],
        "implementation": {
            "script_path": str(SCRIPT),
            "script_sha256": sha256_file(SCRIPT),
            "script_size_bytes": SCRIPT.stat().st_size,
        },
        "dataset": {
            "model_count": model_count,
            "category_count": len(reference_categories),
            "source_count": len({record.source for record in model_records}),
            "views_per_model": (
                expected_views[0] if len(expected_views) == 1 else expected_views
            ),
            "models_per_category_min": min(record.model_count for record in reference_categories),
            "models_per_category_max": max(record.model_count for record in reference_categories),
        },
        "config": {
            "dinov2_dir": str(dinov2_dir.resolve()),
            "clip_dir": str(clip_dir.resolve()),
            "output_dir": str(output_dir),
            "requested_perplexity": requested_perplexity,
            "random_state": random_state,
            "max_iter": max_iter,
            "n_jobs": n_jobs,
            "plot_dpi": plot_dpi,
        },
        "encoders": serializable_results,
        "artifacts": {
            "category_index": "category_index.csv",
            "comparison_plot": "tsne_category_comparison.png",
        },
    }
    _atomic_write_json(output_dir / "run_manifest.json", manifest)
    audit = audit_outputs(
        output_dir,
        encoder_results,
        category_records=reference_categories,
    )
    _atomic_write_json(output_dir / "final_audit.json", audit)
    if not audit["pass"]:
        failed = [name for name, passed in audit["checks"].items() if not passed]
        raise RuntimeError(f"output audit failed: {failed}")
    return {"manifest": manifest, "audit": audit}


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Aggregate frozen Artiverse model features by class and draw t-SNE plots."
    )
    parser.add_argument("--dinov2-dir", type=Path, default=DEFAULT_DINOV2_DIR)
    parser.add_argument("--clip-dir", type=Path, default=DEFAULT_CLIP_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--perplexity", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--tsne-max-iter", type=int, default=1_000)
    parser.add_argument("--tsne-jobs", type=int, default=DEFAULT_TSNE_JOBS)
    parser.add_argument("--plot-dpi", type=int, default=180)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    result = run(
        dinov2_dir=args.dinov2_dir,
        clip_dir=args.clip_dir,
        output_dir=args.output_dir,
        requested_perplexity=args.perplexity,
        random_state=args.seed,
        max_iter=args.tsne_max_iter,
        n_jobs=args.tsne_jobs,
        plot_dpi=args.plot_dpi,
    )
    print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
