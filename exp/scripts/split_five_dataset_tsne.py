#!/usr/bin/env python3
"""Create per-dataset t-SNE figures from the audited five-dataset output.

This is a plotting-only companion to ``visualize_five_datasets_n5_tsne.py``.
It reads the already verified coordinate CSV files and never reruns rendering,
feature extraction, or t-SNE.  The source output directory is left untouched;
figures and a small manifest are written below ``individual_tsne`` by default.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import OrderedDict
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


DATASETS: tuple[tuple[str, str], ...] = (
    ("pva", "PV-A"),
    ("artiverse", "Artiverse"),
    ("articraft10k", "Articraft-10K"),
    ("partnet_mobility", "PartNet-Mobility"),
    ("lam", "LAM"),
)
ENCODERS: tuple[tuple[str, str], ...] = (
    ("dinov2", "DINOv2-base"),
    ("clip", "CLIP ViT-B/32"),
)
MODES: tuple[tuple[str, str], ...] = (
    ("all", "All available classes"),
    ("strict_n5", "Strict five-distinct-assets classes"),
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_coordinates(path: Path) -> tuple[list[dict[str, str]], np.ndarray]:
    rows: list[dict[str, str]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            "dataset_key",
            "class_key",
            "class_name",
            "color_hex",
            "tsne_x",
            "tsne_y",
        }
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(f"coordinate CSV is missing fields: {path}")
        for row in reader:
            rows.append(dict(row))
    if not rows:
        raise ValueError(f"coordinate CSV is empty: {path}")
    try:
        coordinates = np.asarray(
            [[float(row["tsne_x"]), float(row["tsne_y"])] for row in rows],
            dtype=np.float32,
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid coordinates: {path}") from exc
    if coordinates.shape != (len(rows), 2) or not np.isfinite(coordinates).all():
        raise ValueError(f"non-finite coordinates: {path}")
    return rows, coordinates


def _load_data(source_dir: Path) -> dict[tuple[str, str, str], tuple[list[dict[str, str]], np.ndarray]]:
    data: dict[tuple[str, str, str], tuple[list[dict[str, str]], np.ndarray]] = {}
    for dataset_key, _dataset_name in DATASETS:
        for encoder, _encoder_name in ENCODERS:
            for mode, _mode_name in MODES:
                path = source_dir / "coordinates" / encoder / f"{dataset_key}_{mode}.csv"
                path = path.resolve(strict=True)
                rows, coordinates = _read_coordinates(path)
                dataset_values = {row["dataset_key"] for row in rows}
                if dataset_values != {dataset_key}:
                    raise ValueError(f"dataset mismatch in {path}: {dataset_values}")
                class_keys = [row["class_key"] for row in rows]
                if len(set(class_keys)) > len(rows):
                    # Duplicate samples are fine; duplicate class keys are expected.
                    pass
                data[(dataset_key, encoder, mode)] = (rows, coordinates)
    return data


def _draw_axis(axis: Any, rows: Sequence[Mapping[str, str]], coordinates: np.ndarray, title: str) -> None:
    from matplotlib.collections import LineCollection

    grouped: OrderedDict[str, list[int]] = OrderedDict()
    for index, row in enumerate(rows):
        grouped.setdefault(str(row["class_key"]), []).append(index)
    centers = np.asarray(
        [coordinates[indices].mean(axis=0) for indices in grouped.values()],
        dtype=np.float32,
    )
    class_colors = [str(rows[indices[0]]["color_hex"]) for indices in grouped.values()]
    sample_colors = [
        class_colors[group_index]
        for group_index, indices in enumerate(grouped.values())
        for _ in indices
    ]
    # Build segments in sample order so each point is connected to its class center.
    segments = np.asarray(
        [
            (centers[group_index], coordinates[sample_index])
            for group_index, indices in enumerate(grouped.values())
            for sample_index in indices
        ],
        dtype=np.float32,
    )
    if len(segments):
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
        coordinates[:, 0],
        coordinates[:, 1],
        c=sample_colors,
        s=8,
        alpha=0.64,
        edgecolors="none",
        rasterized=True,
        zorder=2,
    )
    axis.scatter(
        centers[:, 0],
        centers[:, 1],
        c=class_colors,
        s=17,
        alpha=1.0,
        edgecolors="#202020",
        linewidths=0.18,
        rasterized=True,
        zorder=3,
    )
    axis.set_title(title, fontsize=11, pad=7)
    axis.set_xlabel("t-SNE 1", fontsize=8)
    axis.set_ylabel("t-SNE 2", fontsize=8)
    axis.tick_params(labelsize=7)
    axis.spines[["top", "right"]].set_visible(False)
    axis.grid(color="#d8d8d8", linewidth=0.4, alpha=0.45)
    axis.set_axisbelow(True)


def _save_figure(figure: Any, path: Path, dpi: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    figure.savefig(temporary, format="png", dpi=dpi, facecolor="white")
    temporary.replace(path)


def _plot_dataset(
    dataset_key: str,
    dataset_name: str,
    data: Mapping[tuple[str, str, str], tuple[list[dict[str, str]], np.ndarray]],
    destination: Path,
    *,
    dpi: int,
) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    written: list[Path] = []
    # One compact two-panel file per cohort/mode is convenient for direct use.
    for mode, mode_label in MODES:
        figure, axes = plt.subplots(1, 2, figsize=(16, 7), dpi=dpi)
        axes = np.asarray(axes, dtype=object).reshape(2)
        figure.subplots_adjust(left=0.06, right=0.985, top=0.86, bottom=0.10, wspace=0.14)
        figure.suptitle(f"{dataset_name}: {mode_label}", fontsize=17, y=0.95)
        for axis, (encoder, encoder_label) in zip(axes, ENCODERS, strict=True):
            rows, coordinates = data[(dataset_key, encoder, mode)]
            classes = len({row["class_key"] for row in rows})
            _draw_axis(axis, rows, coordinates, f"{encoder_label} | {classes} classes, {len(rows):,} renders")
        figure.text(
            0.5,
            0.025,
            "The two encoder panels are independently fitted; absolute t-SNE positions are not comparable.",
            ha="center",
            fontsize=8,
            color="#404040",
        )
        path = destination / mode / f"{dataset_key}.png"
        _save_figure(figure, path, dpi)
        plt.close(figure)
        written.append(path)

    # A single four-panel overview is the requested one-file-per-dataset view.
    figure, axes = plt.subplots(2, 2, figsize=(16, 13), dpi=dpi)
    axes = np.asarray(axes, dtype=object).reshape(2, 2)
    figure.subplots_adjust(left=0.06, right=0.985, top=0.91, bottom=0.075, hspace=0.26, wspace=0.14)
    figure.suptitle(f"{dataset_name}: independent t-SNE by encoder and cohort", fontsize=18, y=0.965)
    for row_index, (mode, mode_label) in enumerate(MODES):
        for column_index, (encoder, encoder_label) in enumerate(ENCODERS):
            rows, coordinates = data[(dataset_key, encoder, mode)]
            classes = len({row["class_key"] for row in rows})
            _draw_axis(
                axes[row_index, column_index],
                rows,
                coordinates,
                f"{mode_label} | {encoder_label}\n{classes} classes, {len(rows):,} renders",
            )
    figure.text(
        0.5,
        0.018,
        "Each panel is fitted independently; use class colors and local neighborhoods, not absolute panel positions, for interpretation.",
        ha="center",
        fontsize=8,
        color="#404040",
    )
    path = destination / f"{dataset_key}.png"
    _save_figure(figure, path, dpi)
    plt.close(figure)
    written.append(path)
    return written


def run(source_dir: Path, destination: Path, dpi: int) -> dict[str, Any]:
    source_dir = source_dir.expanduser().resolve(strict=True)
    destination = destination.expanduser().resolve()
    data = _load_data(source_dir)
    written: list[Path] = []
    for dataset_key, dataset_name in DATASETS:
        written.extend(_plot_dataset(dataset_key, dataset_name, data, destination, dpi=dpi))
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "source_dir": str(source_dir),
        "destination": str(destination),
        "datasets": {key: name for key, name in DATASETS},
        "encoders": {key: name for key, name in ENCODERS},
        "modes": {key: name for key, name in MODES},
        "artifacts": {},
    }
    for path in sorted(written):
        relative = str(path.relative_to(destination))
        manifest["artifacts"][relative] = {
            "path": str(path),
            "size_bytes": path.stat().st_size,
            "sha256": _sha256(path),
        }
    destination.mkdir(parents=True, exist_ok=True)
    manifest_path = destination / "manifest.json"
    temporary = manifest_path.with_name(f".{manifest_path.name}.tmp")
    temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(manifest_path)
    print(f"[done] wrote {len(written)} PNGs to {destination}", flush=True)
    return manifest


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "pva_artiverse_articraft_partnet_lam_n5_tsne",
        help="Existing five-dataset t-SNE output directory.",
    )
    parser.add_argument(
        "--destination",
        type=Path,
        default=None,
        help="Destination directory (default: SOURCE_DIR/individual_tsne).",
    )
    parser.add_argument("--dpi", type=int, default=180)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    source_dir = args.source_dir.expanduser().resolve(strict=True)
    destination = args.destination or (source_dir / "individual_tsne")
    run(source_dir, destination, args.dpi)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
