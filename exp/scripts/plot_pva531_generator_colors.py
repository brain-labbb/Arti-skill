#!/usr/bin/env python3
"""Plot frozen PV-A t-SNE coordinates with one unique color per generator class."""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

for _thread_variable in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_thread_variable, "1")

import numpy as np
from PIL import Image, ImageStat


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_INPUT_DIR = REPO_ROOT / "exp" / "runtime" / "pva531_uniform_tsne"
EXPECTED_GENERATOR_COUNT = 531
ENCODER_KEYS = ("dinov2", "clip")
DISPLAY_MINIMUM_SEPARATION = 0.007


@dataclass(frozen=True, slots=True)
class GeneratorRecord:
    generator_index: str
    generator_name: str
    source_type: str
    picture_category: str
    picture_label: str


@dataclass(frozen=True, slots=True)
class EncoderCoordinates:
    key: str
    label: str
    coordinates: np.ndarray
    coordinate_path: Path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


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


def _atomic_write_csv(path: Path, fieldnames: Sequence[str], rows: Sequence[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def _srgb_to_oklab(rgb: np.ndarray) -> np.ndarray:
    """Convert an array of sRGB triples in [0, 1] to OKLab."""
    rgb = np.asarray(rgb, dtype=np.float64)
    linear = np.where(
        rgb <= 0.04045,
        rgb / 12.92,
        ((rgb + 0.055) / 1.055) ** 2.4,
    )
    red, green, blue = linear.T
    light = 0.4122214708 * red + 0.5363325363 * green + 0.0514459929 * blue
    medium = 0.2119034982 * red + 0.6806995451 * green + 0.1073969566 * blue
    short = 0.0883024619 * red + 0.2817188376 * green + 0.6299787005 * blue
    light_root, medium_root, short_root = np.cbrt(light), np.cbrt(medium), np.cbrt(short)
    return np.column_stack(
        (
            0.2104542553 * light_root
            + 0.7936177850 * medium_root
            - 0.0040720468 * short_root,
            1.9779984951 * light_root
            - 2.4285922050 * medium_root
            + 0.4505937099 * short_root,
            0.0259040371 * light_root
            + 0.7827717662 * medium_root
            - 0.8086757660 * short_root,
        )
    )


def build_unique_palette(count: int) -> list[str]:
    """Select deterministic, exactly unique colors by greedy OKLab separation."""
    if count < 1:
        raise ValueError("color count must be positive")
    levels = np.arange(24, 240, 16, dtype=np.uint8)
    candidates_rgb = np.asarray(
        list(itertools.product(levels, repeat=3)),
        dtype=np.uint8,
    )
    candidates_oklab = _srgb_to_oklab(candidates_rgb.astype(np.float64) / 255.0)
    chroma = np.linalg.norm(candidates_oklab[:, 1:], axis=1)
    visible = (
        (candidates_oklab[:, 0] >= 0.35)
        & (candidates_oklab[:, 0] <= 0.82)
        & (chroma >= 0.035)
    )
    candidates_rgb = candidates_rgb[visible]
    candidates_oklab = candidates_oklab[visible]
    chroma = chroma[visible]
    if count > len(candidates_rgb):
        raise ValueError(f"requested {count} colors from only {len(candidates_rgb)} candidates")

    selected: list[int] = []
    minimum_distance = np.full(len(candidates_rgb), np.inf, dtype=np.float64)
    for color_index in range(count):
        candidate_index = (
            int(np.argmax(chroma))
            if color_index == 0
            else int(np.argmax(minimum_distance))
        )
        selected.append(candidate_index)
        distances = np.linalg.norm(
            candidates_oklab - candidates_oklab[candidate_index],
            axis=1,
        )
        minimum_distance = np.minimum(minimum_distance, distances)
        minimum_distance[selected] = -1.0
    colors = [
        f"#{red:02x}{green:02x}{blue:02x}"
        for red, green, blue in candidates_rgb[selected]
    ]
    if len(set(colors)) != count:
        raise RuntimeError("palette generation produced duplicate RGB colors")
    return colors


def separate_display_coordinates(
    coordinates: np.ndarray,
    *,
    minimum_separation: float = DISPLAY_MINIMUM_SEPARATION,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Minimally move near-coincident points in normalized display space."""
    coordinates = np.asarray(coordinates, dtype=np.float64)
    if coordinates.ndim != 2 or coordinates.shape[1] != 2:
        raise ValueError("coordinates must have shape (n, 2)")
    if len(coordinates) < 2 or not np.isfinite(coordinates).all():
        raise ValueError("at least two finite coordinates are required")
    if minimum_separation <= 0:
        raise ValueError("minimum separation must be positive")
    coordinate_minimum = coordinates.min(axis=0)
    coordinate_span = np.ptp(coordinates, axis=0)
    if np.any(coordinate_span == 0):
        raise ValueError("coordinate axes must have non-zero spans")
    original_normalized = (coordinates - coordinate_minimum) / coordinate_span
    display_normalized = np.empty_like(original_normalized)
    moved_count = 0

    for point_index, original in enumerate(original_normalized):
        if (
            point_index == 0
            or np.linalg.norm(display_normalized[:point_index] - original, axis=1).min()
            >= minimum_separation
        ):
            display = original
        else:
            display = None
            phase = (point_index * 0.6180339887498949) % 1.0 * 2.0 * np.pi
            for radius_step in range(1, 41):
                radius = minimum_separation * radius_step / 4.0
                angles = phase + np.arange(48, dtype=np.float64) * (2.0 * np.pi / 48.0)
                candidates = original + radius * np.column_stack(
                    (np.cos(angles), np.sin(angles))
                )
                candidate_distances = np.linalg.norm(
                    candidates[:, None, :] - display_normalized[None, :point_index, :],
                    axis=2,
                ).min(axis=1)
                valid = np.flatnonzero(candidate_distances >= minimum_separation)
                if len(valid):
                    best = valid[np.argmax(candidate_distances[valid])]
                    display = candidates[best]
                    break
            if display is None:
                raise RuntimeError(f"could not separate display point {point_index}")
            moved_count += 1
        display_normalized[point_index] = display

    pairwise_distances = np.linalg.norm(
        display_normalized[:, None, :] - display_normalized[None, :, :],
        axis=2,
    )
    pairwise_distances[np.eye(len(coordinates), dtype=bool)] = np.inf
    displacement = np.linalg.norm(display_normalized - original_normalized, axis=1)
    display_coordinates = coordinate_minimum + display_normalized * coordinate_span
    return np.asarray(display_coordinates, dtype=np.float32), {
        "minimum_separation_requested": minimum_separation,
        "minimum_separation_observed": float(pairwise_distances.min()),
        "moved_point_count": moved_count,
        "unchanged_point_count": len(coordinates) - moved_count,
        "maximum_normalized_displacement": float(displacement.max()),
        "mean_normalized_displacement": float(displacement.mean()),
    }


def load_inputs(
    input_dir: Path,
    *,
    expected_count: int | None,
) -> tuple[list[GeneratorRecord], list[EncoderCoordinates], dict[str, Any]]:
    """Load and exactly align the roster with both frozen coordinate tables."""
    input_dir = input_dir.resolve(strict=True)
    manifest_path = input_dir / "run_manifest.json"
    roster_path = input_dir / "generator_roster_resolved.csv"
    if not manifest_path.is_file() or not roster_path.is_file():
        raise FileNotFoundError(f"PV-A t-SNE input is incomplete: {input_dir}")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"unreadable PV-A run manifest: {manifest_path}") from exc
    if not isinstance(manifest, dict):
        raise ValueError("PV-A run manifest is not an object")

    with roster_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required_roster = {
        "generator_index",
        "generator_name",
        "source_type",
        "picture_category",
        "picture_label",
    }
    if not rows or not required_roster.issubset(rows[0]):
        raise ValueError("generator roster lacks required fields")
    records = [
        GeneratorRecord(
            generator_index=row["generator_index"].strip(),
            generator_name=row["generator_name"].strip(),
            source_type=row["source_type"].strip(),
            picture_category=row["picture_category"].strip(),
            picture_label=row["picture_label"].strip(),
        )
        for row in rows
    ]
    if any(not record.generator_index or not record.generator_name for record in records):
        raise ValueError("generator roster contains an empty identity")
    identities = [(record.generator_index, record.generator_name) for record in records]
    if len(set(identities)) != len(records):
        raise ValueError("generator roster contains duplicate identities")
    expected_indices = [f"G{index:04d}" for index in range(1, len(records) + 1)]
    if [record.generator_index for record in records] != expected_indices:
        raise ValueError("generator roster indices are not contiguous and ordered")
    if expected_count is not None and len(records) != expected_count:
        raise ValueError(f"expected {expected_count} generators, found {len(records)}")
    manifest_count = manifest.get("dataset", {}).get("generator_count")
    if manifest_count != len(records):
        raise ValueError("run manifest generator count differs from roster")

    encoders: list[EncoderCoordinates] = []
    coordinate_receipts: dict[str, Any] = {}
    model_summaries = manifest.get("models", {})
    for key in ENCODER_KEYS:
        coordinate_path = input_dir / key / "tsne_coordinates.csv"
        if not coordinate_path.is_file():
            raise FileNotFoundError(f"missing {key} coordinate table: {coordinate_path}")
        with coordinate_path.open("r", encoding="utf-8", newline="") as stream:
            coordinate_rows = list(csv.DictReader(stream))
        required_coordinates = {"tsne_x", "tsne_y", "generator_index", "generator_name"}
        if not coordinate_rows or not required_coordinates.issubset(coordinate_rows[0]):
            raise ValueError(f"{key} coordinate table lacks required fields")
        coordinate_identities = [
            (row["generator_index"].strip(), row["generator_name"].strip())
            for row in coordinate_rows
        ]
        if coordinate_identities != identities:
            raise ValueError(f"{key} coordinates are not exactly aligned to the generator roster")
        try:
            coordinates = np.asarray(
                [
                    [float(row["tsne_x"]), float(row["tsne_y"])]
                    for row in coordinate_rows
                ],
                dtype=np.float32,
            )
        except ValueError as exc:
            raise ValueError(f"{key} coordinates contain a non-numeric value") from exc
        if coordinates.shape != (len(records), 2) or not np.isfinite(coordinates).all():
            raise ValueError(f"{key} coordinates have an invalid shape or value")
        summary = model_summaries.get(key, {})
        label = str(summary.get("encoder_label") or key)
        encoders.append(EncoderCoordinates(key, label, coordinates, coordinate_path))
        coordinate_receipts[key] = {
            "path": str(coordinate_path),
            "sha256": sha256_file(coordinate_path),
            "size_bytes": coordinate_path.stat().st_size,
            "shape": list(coordinates.shape),
        }

    receipt = {
        "run_manifest": {
            "path": str(manifest_path),
            "sha256": sha256_file(manifest_path),
            "size_bytes": manifest_path.stat().st_size,
        },
        "generator_roster": {
            "path": str(roster_path),
            "sha256": sha256_file(roster_path),
            "size_bytes": roster_path.stat().st_size,
            "row_count": len(records),
        },
        "coordinates": coordinate_receipts,
    }
    return records, encoders, receipt


def _draw_axis(
    axis: Any,
    coordinates: np.ndarray,
    colors: Sequence[str],
    *,
    title: str,
) -> None:
    axis.scatter(
        coordinates[:, 0],
        coordinates[:, 1],
        c=list(colors),
        s=28,
        alpha=1.0,
        edgecolors="#1d1d1d",
        linewidths=0.18,
        rasterized=True,
    )
    axis.set_title(title, fontsize=15, pad=11)
    axis.set_xlabel("t-SNE 1")
    axis.set_ylabel("t-SNE 2")
    axis.spines[["top", "right"]].set_visible(False)
    axis.grid(color="#d9d9d9", linewidth=0.45, alpha=0.55)


def _save_figure_atomic(figure: Any, path: Path, *, dpi: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    figure.savefig(temporary, format="png", dpi=dpi, facecolor="white")
    temporary.replace(path)


def save_encoder_plot(
    encoder: EncoderCoordinates,
    display_coordinates: np.ndarray,
    colors: Sequence[str],
    output_path: Path,
    *,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(14, 10), dpi=dpi)
    figure.subplots_adjust(left=0.08, right=0.98, top=0.91, bottom=0.10)
    _draw_axis(
        axis,
        display_coordinates,
        colors,
        title=f"{encoder.label} t-SNE: {len(colors)} PV-A generator classes",
    )
    figure.text(
        0.5,
        0.025,
        (
            "Every point has a unique class color. Near-coincident positions are minimally "
            "separated for visibility."
        ),
        ha="center",
        fontsize=8,
        color="#4a4a4a",
    )
    _save_figure_atomic(figure, output_path, dpi=dpi)
    plt.close(figure)


def save_comparison_plot(
    encoders: Sequence[EncoderCoordinates],
    display_coordinates: dict[str, np.ndarray],
    colors: Sequence[str],
    output_path: Path,
    *,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, len(encoders), figsize=(24, 10), dpi=dpi)
    axes = np.atleast_1d(axes)
    figure.subplots_adjust(left=0.045, right=0.985, top=0.89, bottom=0.09, wspace=0.15)
    figure.suptitle(
        f"PV-A t-SNE: {len(colors)} generator classes with unique colors",
        fontsize=19,
        y=0.965,
    )
    for axis, encoder in zip(axes, encoders, strict=True):
        _draw_axis(axis, display_coordinates[encoder.key], colors, title=encoder.label)
    figure.text(
        0.5,
        0.02,
        (
            "The same generator uses the same color in both panels. Near-coincident points "
            "are minimally separated for visibility."
        ),
        ha="center",
        fontsize=8,
        color="#4a4a4a",
    )
    _save_figure_atomic(figure, output_path, dpi=dpi)
    plt.close(figure)


def save_color_key(
    records: Sequence[GeneratorRecord],
    colors: Sequence[str],
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
        0.025,
        0.985,
        f"PV-A generator class color key (n={len(records)})",
        ha="left",
        va="top",
        fontsize=15,
    )
    x_start, x_end = 0.025, 0.985
    y_top, y_bottom = 0.958, 0.025
    column_width = (x_end - x_start) / columns
    y_step = (y_top - y_bottom) / max(1, rows - 1)
    for index, (record, color) in enumerate(zip(records, colors, strict=True)):
        column = index // rows
        row = index % rows
        x = x_start + column * column_width
        y = y_top - row * y_step
        figure.add_artist(
            Rectangle(
                (x, y - 0.0038),
                0.008,
                0.0076,
                transform=figure.transFigure,
                facecolor=color,
                edgecolor="#202020",
                linewidth=0.25,
            )
        )
        figure.text(
            x + 0.011,
            y,
            f"{record.generator_index}  {record.generator_name}",
            ha="left",
            va="center",
            fontsize=4.5,
            color="#252525",
        )
    _save_figure_atomic(figure, output_path, dpi=dpi)
    plt.close(figure)


def audit_outputs(
    input_dir: Path,
    records: Sequence[GeneratorRecord],
    colors: Sequence[str],
    encoders: Sequence[EncoderCoordinates],
) -> dict[str, Any]:
    checks: dict[str, bool] = {
        "generator_count": len(records) == len(colors),
        "unique_generator_identities": len(
            {(record.generator_index, record.generator_name) for record in records}
        )
        == len(records),
        "unique_color_count": len(set(colors)) == len(records),
        "valid_hex_colors": all(
            len(color) == 7 and color.startswith("#")
            for color in colors
        ),
    }
    index_path = input_dir / "generator_class_color_index.csv"
    with index_path.open("r", encoding="utf-8", newline="") as stream:
        index_rows = list(csv.DictReader(stream))
    checks["color_index_rows"] = len(index_rows) == len(records)
    checks["color_index_alignment"] = [
        (row["generator_index"], row["generator_name"], row["color_hex"])
        for row in index_rows
    ] == [
        (record.generator_index, record.generator_name, color)
        for record, color in zip(records, colors, strict=True)
    ]

    artifacts: dict[str, dict[str, Any]] = {
        "generator_class_color_index.csv": {
            "sha256": sha256_file(index_path),
            "size_bytes": index_path.stat().st_size,
        }
    }
    expected_identities = [
        (record.generator_index, record.generator_name) for record in records
    ]
    for encoder in encoders:
        plot_coordinate_path = (
            input_dir / encoder.key / "tsne_generator_class_plot_coordinates.csv"
        )
        with plot_coordinate_path.open("r", encoding="utf-8", newline="") as stream:
            plot_coordinate_rows = list(csv.DictReader(stream))
        checks[f"{encoder.key}_plot_coordinate_rows"] = (
            len(plot_coordinate_rows) == len(records)
        )
        checks[f"{encoder.key}_plot_coordinate_alignment"] = [
            (row["generator_index"], row["generator_name"])
            for row in plot_coordinate_rows
        ] == expected_identities
        try:
            original_coordinates = np.asarray(
                [
                    [float(row["original_tsne_x"]), float(row["original_tsne_y"])]
                    for row in plot_coordinate_rows
                ],
                dtype=np.float32,
            )
            display_coordinates = np.asarray(
                [
                    [float(row["display_tsne_x"]), float(row["display_tsne_y"])]
                    for row in plot_coordinate_rows
                ],
                dtype=np.float32,
            )
        except (KeyError, ValueError):
            original_coordinates = np.empty((0, 2), dtype=np.float32)
            display_coordinates = np.empty((0, 2), dtype=np.float32)
        checks[f"{encoder.key}_plot_coordinate_originals"] = bool(
            original_coordinates.shape == encoder.coordinates.shape
            and np.allclose(
                original_coordinates,
                encoder.coordinates,
                rtol=0.0,
                atol=1e-6,
            )
        )
        checks[f"{encoder.key}_plot_coordinate_finite"] = bool(
            display_coordinates.shape == encoder.coordinates.shape
            and np.isfinite(display_coordinates).all()
        )
        artifacts[str(plot_coordinate_path.relative_to(input_dir))] = {
            "sha256": sha256_file(plot_coordinate_path),
            "size_bytes": plot_coordinate_path.stat().st_size,
            "row_count": len(plot_coordinate_rows),
        }
    expected_rgb = {
        (int(color[1:3], 16) << 16)
        | (int(color[3:5], 16) << 8)
        | int(color[5:7], 16)
        for color in colors
    }
    png_paths = [
        input_dir / "tsne_generator_class_comparison.png",
        input_dir / "generator_class_color_key.png",
        *[
            input_dir / encoder.key / "tsne_by_generator_class.png"
            for encoder in encoders
        ],
    ]
    for path in png_paths:
        artifact_key = path.relative_to(input_dir).with_suffix("").as_posix().replace("/", "_")
        with Image.open(path) as image:
            image.load()
            rgb = np.asarray(image.convert("RGB"), dtype=np.uint32)
            encoded = (rgb[:, :, 0] << 16) | (rgb[:, :, 1] << 8) | rgb[:, :, 2]
            visible_colors = expected_rgb.intersection(np.unique(encoded).tolist())
            standard_deviation = ImageStat.Stat(image.convert("L")).stddev[0]
            checks[f"{artifact_key}_dimensions"] = image.width >= 800 and image.height >= 600
            checks[f"{artifact_key}_nonblank"] = standard_deviation > 1.0
            checks[f"{artifact_key}_all_palette_colors_visible"] = (
                len(visible_colors) == len(colors)
            )
            artifacts[str(path.relative_to(input_dir))] = {
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
                "width": image.width,
                "height": image.height,
                "grayscale_std": standard_deviation,
                "visible_palette_color_count": len(visible_colors),
                "expected_palette_color_count": len(colors),
            }
            if path.name == "tsne_generator_class_comparison.png":
                midpoint = image.width // 2
                panel_counts = []
                for panel in (encoded[:, :midpoint], encoded[:, midpoint:]):
                    panel_counts.append(
                        len(expected_rgb.intersection(np.unique(panel).tolist()))
                    )
                checks["comparison_left_panel_all_palette_colors_visible"] = (
                    panel_counts[0] == len(colors)
                )
                checks["comparison_right_panel_all_palette_colors_visible"] = (
                    panel_counts[1] == len(colors)
                )
                artifacts[str(path.relative_to(input_dir))][
                    "panel_visible_palette_color_counts"
                ] = panel_counts
    manifest_path = input_dir / "generator_class_color_manifest.json"
    artifacts["generator_class_color_manifest.json"] = {
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
    input_dir: Path,
    expected_count: int | None,
    plot_dpi: int,
) -> dict[str, Any]:
    if plot_dpi < 40:
        raise ValueError("plot_dpi must be at least 40")
    input_dir = input_dir.resolve(strict=True)
    records, encoders, input_receipt = load_inputs(
        input_dir,
        expected_count=expected_count,
    )
    colors = build_unique_palette(len(records))
    display_coordinates: dict[str, np.ndarray] = {}
    display_adjustments: dict[str, dict[str, Any]] = {}
    for encoder in encoders:
        adjusted, adjustment = separate_display_coordinates(encoder.coordinates)
        display_coordinates[encoder.key] = adjusted
        display_adjustments[encoder.key] = adjustment
        rows = [
            {
                "generator_index": record.generator_index,
                "generator_name": record.generator_name,
                "original_tsne_x": format(float(original[0]), ".9g"),
                "original_tsne_y": format(float(original[1]), ".9g"),
                "display_tsne_x": format(float(display[0]), ".9g"),
                "display_tsne_y": format(float(display[1]), ".9g"),
            }
            for record, original, display in zip(
                records,
                encoder.coordinates,
                adjusted,
                strict=True,
            )
        ]
        _atomic_write_csv(
            input_dir / encoder.key / "tsne_generator_class_plot_coordinates.csv",
            list(rows[0]),
            rows,
        )
    index_rows = [
        {
            "generator_index": record.generator_index,
            "generator_name": record.generator_name,
            "source_type": record.source_type,
            "picture_category": record.picture_category,
            "picture_label": record.picture_label,
            "color_hex": color,
            "color_rgb": ",".join(
                str(int(color[offset : offset + 2], 16))
                for offset in (1, 3, 5)
            ),
        }
        for record, color in zip(records, colors, strict=True)
    ]
    _atomic_write_csv(
        input_dir / "generator_class_color_index.csv",
        list(index_rows[0]),
        index_rows,
    )
    for encoder in encoders:
        save_encoder_plot(
            encoder,
            display_coordinates[encoder.key],
            colors,
            input_dir / encoder.key / "tsne_by_generator_class.png",
            dpi=plot_dpi,
        )
    save_comparison_plot(
        encoders,
        display_coordinates,
        colors,
        input_dir / "tsne_generator_class_comparison.png",
        dpi=plot_dpi,
    )
    save_color_key(
        records,
        colors,
        input_dir / "generator_class_color_key.png",
        dpi=plot_dpi,
    )

    manifest = {
        "schema_version": 1,
        "dataset": {
            "generator_count": len(records),
            "color_count": len(colors),
            "unique_color_count": len(set(colors)),
            "encoder_count": len(encoders),
        },
        "protocol": {
            "sample_unit": "one PV-A generator class per point",
            "color_unit": "one exact RGB color per generator class",
            "color_order": "ascending generator_index, shared across encoders",
            "palette": (
                "greedy farthest-point selection in OKLab from a bounded 8-bit sRGB grid"
            ),
            "coordinate_policy": "reuse audited frozen t-SNE coordinates without refitting",
            "display_collision_policy": (
                "deterministic minimum separation in per-axis normalized coordinates; "
                "original coordinates are never overwritten"
            ),
        },
        "interpretation_limits": [
            "All RGB values are unique, but 531 colors are not all perceptually distinguishable.",
            "Colors identify singleton generator classes and do not encode semantic groups.",
            "DINOv2 and CLIP t-SNE coordinates were fitted independently.",
            "A small display-only displacement separates near-coincident points.",
        ],
        "implementation": {
            "script_path": str(SCRIPT),
            "script_sha256": sha256_file(SCRIPT),
            "script_size_bytes": SCRIPT.stat().st_size,
        },
        "input_receipt": input_receipt,
        "display_adjustments": display_adjustments,
        "artifacts": {
            "class_color_index": "generator_class_color_index.csv",
            "class_color_key": "generator_class_color_key.png",
            "comparison_plot": "tsne_generator_class_comparison.png",
            "encoder_plots": {
                encoder.key: f"{encoder.key}/tsne_by_generator_class.png"
                for encoder in encoders
            },
            "plot_coordinates": {
                encoder.key: f"{encoder.key}/tsne_generator_class_plot_coordinates.csv"
                for encoder in encoders
            },
        },
        "config": {
            "input_dir": str(input_dir),
            "expected_count": expected_count,
            "plot_dpi": plot_dpi,
        },
    }
    _atomic_write_json(input_dir / "generator_class_color_manifest.json", manifest)
    audit = audit_outputs(input_dir, records, colors, encoders)
    _atomic_write_json(input_dir / "generator_class_color_audit.json", audit)
    if not audit["pass"]:
        failed = [name for name, passed in audit["checks"].items() if not passed]
        raise RuntimeError(f"class-color output audit failed: {failed}")
    return {"manifest": manifest, "audit": audit}


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot PV-A t-SNE with one unique color for every generator class."
    )
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    parser.add_argument("--plot-dpi", type=int, default=180)
    parser.add_argument(
        "--allow-count-drift",
        action="store_true",
        help="Allow a non-531 roster for a custom or smoke-test input directory.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    result = run(
        input_dir=args.input_dir,
        expected_count=None if args.allow_count_drift else EXPECTED_GENERATOR_COUNT,
        plot_dpi=args.plot_dpi,
    )
    print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
