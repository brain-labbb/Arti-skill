#!/usr/bin/env python3
"""Joint class-level t-SNE for the 531 generator classes.

The input roster is ``template_maps/generator_picture_index.csv``.  The 99
``articraft_builtin_dataset_no_picture`` rows use their PV-A representative
render.  The 432 picture-backed rows use all PNGs in their mapped picture
directory and are represented by the L2-normalized mean of those image
embeddings.  This keeps the visualization at class level instead of allowing
directories with more reference images to dominate the plot.

DINOv2 and CLIP are run independently because their embedding spaces are not
calibrated and must not be concatenated before t-SNE.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import re
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

for _thread_variable in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_thread_variable, "1")

import numpy as np
from PIL import Image


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_INDEX_CSV = REPO_ROOT / "template_maps" / "generator_picture_index.csv"
DEFAULT_PICTURE_ROOT = REPO_ROOT / "articraft_data" / "picture"
DEFAULT_BUILTIN_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/PV-A/renders/builtin99_representatives"
)
DEFAULT_UNIFORM_RENDER_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/PV-A/renders/uniform531_studio_256_v1"
)
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exp" / "runtime" / "pva531_picture_tsne"
DEFAULT_DINO_MODEL = Path(
    "/root/.cache/huggingface/hub/models--facebook--dinov2-base/"
    "snapshots/f9e44c814b77203eaa57a6bdbbd535f21ede1415"
)
DEFAULT_CLIP_MODEL = Path(
    "/root/.cache/huggingface/hub/models--openai--clip-vit-base-patch32/"
    "snapshots/3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"
)
EXPECTED_GENERATOR_COUNT = 531
EXPECTED_BUILTIN_COUNT = 99
EXPECTED_PICTURE_BACKED_COUNT = 432


@dataclass(frozen=True, slots=True)
class GeneratorRecord:
    """One row in the generator roster and its source images."""

    generator_index: str
    generator_name: str
    source_type: str
    picture_category: str
    picture_label: str
    picture_dir: Path | None
    image_paths: tuple[Path, ...]
    alias_of_generator_index: str | None = None

    @property
    def class_label(self) -> str:
        return f"{self.generator_index} {self.generator_name}"


@dataclass(frozen=True, slots=True)
class DatasetBundle:
    records: tuple[GeneratorRecord, ...]
    raw_image_paths: tuple[Path, ...]
    raw_path_to_index: dict[Path, int]
    summary: dict[str, Any]


def _resolve_inside(path: Path, root: Path, *, must_exist: bool = True) -> Path:
    """Resolve a path and reject symlinks that escape its declared root."""
    root = root.expanduser().resolve(strict=True)
    candidate = (root / path).resolve(strict=must_exist)
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"path resolves outside root {root}: {path}") from exc
    if must_exist and not candidate.exists():
        raise FileNotFoundError(candidate)
    return candidate


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _picture_relative_path(value: str) -> Path:
    """Convert the CSV's repo-relative picture path to a picture-root path."""
    relative = Path(value)
    expected_prefix = ("articraft_data", "picture")
    if relative.parts[:2] == expected_prefix:
        relative = Path(*relative.parts[2:])
    if not relative.parts or relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"invalid picture_source_path in roster: {value!r}")
    return relative


def _builtin_image_path(generator_name: str, builtin_root: Path) -> Path:
    """Find the deterministic seed-0000 representative for a builtin row."""
    filename = f"{generator_name}__seed_0000.png"
    try:
        candidate = _resolve_inside(Path(filename), builtin_root)
    except FileNotFoundError:
        candidate = builtin_root / filename
    if candidate.is_file():
        return candidate.resolve(strict=True)

    # Keep a useful fallback for older PV-A runs that did not have the copied
    # builtin99 directory yet.
    fallback_root = builtin_root.parent / "representatives"
    try:
        fallback = _resolve_inside(Path(filename), fallback_root)
    except FileNotFoundError:
        fallback = fallback_root / filename
    if fallback.is_file():
        return fallback.resolve(strict=True)
    raise FileNotFoundError(
        f"no seed-0000 representative for {generator_name!r}; checked {candidate} and {fallback}"
    )


def discover_generator_records(
    index_csv: Path,
    *,
    picture_root: Path = DEFAULT_PICTURE_ROOT,
    builtin_root: Path = DEFAULT_BUILTIN_ROOT,
    strict_counts: bool = True,
) -> DatasetBundle:
    """Read the 531-row roster and resolve only images belonging to those rows."""
    index_csv = index_csv.expanduser().resolve(strict=True)
    picture_root = picture_root.expanduser().resolve(strict=True)
    builtin_root = builtin_root.expanduser().resolve(strict=True)
    with index_csv.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty generator roster: {index_csv}")

    records: list[GeneratorRecord] = []
    first_by_picture_dir: dict[Path, GeneratorRecord] = {}
    for row_number, row in enumerate(rows, start=2):
        index = (row.get("generator_index") or "").strip()
        name = (row.get("generator_name") or "").strip()
        source = (row.get("source_type") or "").strip()
        if not index or not name:
            raise ValueError(f"missing generator identity at CSV line {row_number}")
        if source == "articraft_builtin_dataset_no_picture":
            image_path = _builtin_image_path(name, builtin_root)
            record = GeneratorRecord(
                generator_index=index,
                generator_name=name,
                source_type="builtin_no_picture",
                picture_category="builtin_no_picture",
                picture_label="",
                picture_dir=None,
                image_paths=(image_path,),
            )
        elif source == "picture_backed":
            source_value = (row.get("picture_source_path") or "").strip()
            if not source_value:
                raise ValueError(f"picture-backed row has no picture path at line {row_number}")
            relative = _picture_relative_path(source_value)
            picture_dir = _resolve_inside(relative, picture_root)
            if not picture_dir.is_dir():
                raise NotADirectoryError(picture_dir)
            image_paths = tuple(
                path.resolve(strict=True)
                for path in sorted(picture_dir.glob("*.png"))
                if path.is_file()
            )
            if not image_paths:
                raise ValueError(f"no PNG images in mapped picture directory: {picture_dir}")
            for image_path in image_paths:
                try:
                    image_path.relative_to(picture_root)
                except ValueError as exc:
                    raise ValueError(f"picture path escapes picture root: {image_path}") from exc
            previous = first_by_picture_dir.get(picture_dir)
            record = GeneratorRecord(
                generator_index=index,
                generator_name=name,
                source_type="picture_backed",
                picture_category=(row.get("picture_category") or "").strip() or "unknown",
                picture_label=(row.get("picture_label") or "").strip(),
                picture_dir=picture_dir,
                image_paths=image_paths,
                alias_of_generator_index=(previous.generator_index if previous else None),
            )
            if previous is None:
                first_by_picture_dir[picture_dir] = record
        else:
            raise ValueError(f"unsupported source_type {source!r} at CSV line {row_number}")
        records.append(record)

    builtin_count = sum(record.source_type == "builtin_no_picture" for record in records)
    picture_count = sum(record.source_type == "picture_backed" for record in records)
    if strict_counts and (
        len(records) != EXPECTED_GENERATOR_COUNT
        or builtin_count != EXPECTED_BUILTIN_COUNT
        or picture_count != EXPECTED_PICTURE_BACKED_COUNT
    ):
        raise ValueError(
            "unexpected roster counts: "
            f"total={len(records)}, builtin={builtin_count}, picture={picture_count}; "
            f"expected {EXPECTED_GENERATOR_COUNT}/{EXPECTED_BUILTIN_COUNT}/{EXPECTED_PICTURE_BACKED_COUNT}"
        )

    raw_paths: list[Path] = []
    raw_path_to_index: dict[Path, int] = {}
    for record in records:
        for image_path in record.image_paths:
            image_path = image_path.resolve(strict=True)
            if image_path not in raw_path_to_index:
                raw_path_to_index[image_path] = len(raw_paths)
                raw_paths.append(image_path)

    duplicate_aliases = [record for record in records if record.alias_of_generator_index]
    summary = {
        "index_csv": str(index_csv),
        "picture_root": str(picture_root),
        "builtin_root": str(builtin_root),
        "sample_unit": "generator_class",
        "picture_image_policy": "all_png_l2_normalized_mean",
        "generator_count": len(records),
        "builtin_no_picture_count": builtin_count,
        "picture_backed_count": picture_count,
        "unique_picture_directory_count": len(first_by_picture_dir),
        "unique_picture_image_count": len(
            {
                image_path
                for record in records
                if record.source_type == "picture_backed"
                for image_path in record.image_paths
            }
        ),
        "raw_unique_image_count": len(raw_paths),
        "picture_backed_raw_image_count": sum(
            len(record.image_paths) for record in records if record.source_type == "picture_backed"
        ),
        "shared_picture_directory_alias_count": len(duplicate_aliases),
        "shared_picture_directory_aliases": [
            {
                "generator_index": record.generator_index,
                "generator_name": record.generator_name,
                "alias_of_generator_index": record.alias_of_generator_index,
                "picture_dir": str(record.picture_dir),
            }
            for record in duplicate_aliases
        ],
    }
    return DatasetBundle(tuple(records), tuple(raw_paths), raw_path_to_index, summary)


def discover_uniform_render_records(
    index_csv: Path,
    *,
    render_root: Path,
    strict_counts: bool = True,
) -> DatasetBundle:
    """Resolve one audited, uniformly rendered seed_0000 PNG per generator."""
    index_csv = index_csv.expanduser().resolve(strict=True)
    render_root = render_root.expanduser().resolve(strict=True)
    config_path = render_root / "render_config.json"
    manifest_path = render_root / "render_manifest.csv"
    if not config_path.is_file() or not manifest_path.is_file():
        raise FileNotFoundError(
            f"uniform render root needs render_config.json and render_manifest.csv: {render_root}"
        )
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config.get("render_contract") != "pva531_uniform_studio_v1":
        raise ValueError(f"unexpected render contract in {config_path}")
    if config.get("schema_version") != 2 or not isinstance(config.get("input_receipt"), dict):
        raise ValueError(f"uniform render config is not a schema-v2 input-audited contract: {config_path}")
    input_receipt = config["input_receipt"]
    if (
        re.fullmatch(r"[0-9a-f]{64}", str(input_receipt.get("asset_and_dependency_sha256")))
        is None
        or int(input_receipt.get("file_count", 0)) < 1
        or int(input_receipt.get("total_bytes", 0)) < 1
    ):
        raise ValueError(f"invalid input receipt in {config_path}")
    declared_index_hash = config.get("index_csv_sha256")
    actual_index_hash = _file_sha256(index_csv)
    if declared_index_hash != actual_index_hash:
        raise ValueError(
            f"render config/index CSV SHA mismatch: {declared_index_hash} != {actual_index_hash}"
        )
    with manifest_path.open("r", encoding="utf-8", newline="") as stream:
        render_rows = list(csv.DictReader(stream))
    render_indices = [(row.get("generator_index") or "").strip() for row in render_rows]
    if len(render_indices) != len(set(render_indices)):
        raise ValueError(f"duplicate generator indices in {manifest_path}")
    successful = {
        (row.get("generator_index") or "").strip(): row
        for row in render_rows
        if (row.get("status") or "").strip() in {"rendered", "reused_valid"}
    }
    with index_csv.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if strict_counts and len(rows) != EXPECTED_GENERATOR_COUNT:
        raise ValueError(f"expected {EXPECTED_GENERATOR_COUNT} generators, found {len(rows)}")
    if len(render_rows) != len(rows):
        raise ValueError(
            f"uniform render manifest/roster count mismatch: {len(render_rows)} != {len(rows)}"
        )
    if int(config.get("generator_count", -1)) != len(rows):
        raise ValueError(
            f"render config/roster count mismatch: {config.get('generator_count')} != {len(rows)}"
        )

    records: list[GeneratorRecord] = []
    image_sizes: set[tuple[int, int]] = set()
    for row_number, row in enumerate(rows, start=2):
        index = (row.get("generator_index") or "").strip()
        name = (row.get("generator_name") or "").strip()
        roster_source = (row.get("source_type") or "").strip()
        if not index or not name:
            raise ValueError(f"missing generator identity at CSV line {row_number}")
        source_type = (
            "builtin_no_picture"
            if roster_source == "articraft_builtin_dataset_no_picture"
            else roster_source
        )
        if source_type not in {"builtin_no_picture", "picture_backed"}:
            raise ValueError(f"unsupported source_type {roster_source!r} at CSV line {row_number}")
        manifest_row = successful.get(index)
        if manifest_row is None or manifest_row.get("generator_name") != name:
            raise ValueError(f"no successful uniform render manifest row for {index} {name}")
        expected_name = f"{index}__{name}__seed_0000.png"
        image_path = _resolve_inside(Path(expected_name), render_root)
        declared_output = Path(manifest_row.get("output_path") or "").expanduser().resolve()
        if declared_output != image_path:
            raise ValueError(
                f"manifest/output mismatch for {index}: {declared_output} != {image_path}"
            )
        actual_bytes = image_path.stat().st_size
        try:
            declared_bytes = int(manifest_row.get("png_bytes") or -1)
        except ValueError as exc:
            raise ValueError(f"invalid png_bytes for {index}") from exc
        if declared_bytes != actual_bytes:
            raise ValueError(
                f"manifest/PNG size mismatch for {index}: {declared_bytes} != {actual_bytes}"
            )
        declared_hash = (manifest_row.get("png_sha256") or "").strip()
        actual_hash = _file_sha256(image_path)
        if declared_hash != actual_hash:
            raise ValueError(
                f"manifest/PNG SHA mismatch for {index}: {declared_hash} != {actual_hash}"
            )
        with Image.open(image_path) as image:
            image.verify()
            image_sizes.add(tuple(image.size))
        records.append(
            GeneratorRecord(
                generator_index=index,
                generator_name=name,
                source_type=source_type,
                picture_category=(row.get("picture_category") or "").strip()
                or ("builtin_no_picture" if source_type == "builtin_no_picture" else "unknown"),
                picture_label=(row.get("picture_label") or "").strip(),
                picture_dir=None,
                image_paths=(image_path,),
            )
        )

    builtin_count = sum(record.source_type == "builtin_no_picture" for record in records)
    picture_count = sum(record.source_type == "picture_backed" for record in records)
    if strict_counts and (
        len(records) != EXPECTED_GENERATOR_COUNT
        or builtin_count != EXPECTED_BUILTIN_COUNT
        or picture_count != EXPECTED_PICTURE_BACKED_COUNT
    ):
        raise ValueError(
            "unexpected roster counts: "
            f"total={len(records)}, builtin={builtin_count}, picture={picture_count}"
        )
    if len(image_sizes) != 1:
        raise ValueError(f"uniform render images have mixed dimensions: {sorted(image_sizes)}")
    configured_size = (int(config.get("resolution", -1)),) * 2
    if image_sizes != {configured_size}:
        raise ValueError(
            f"render dimensions {sorted(image_sizes)} disagree with config {configured_size}"
        )

    raw_paths = tuple(record.image_paths[0] for record in records)
    summary = {
        "index_csv": str(index_csv),
        "input_mode": "uniform_blender_seed_0000",
        "render_root": str(render_root),
        "render_config": str(config_path),
        "render_config_schema_version": config.get("schema_version"),
        "render_config_sha256": _file_sha256(config_path),
        "render_manifest": str(manifest_path),
        "render_contract": config["render_contract"],
        "renderer_sha256": config.get("renderer_sha256"),
        "asset_and_dependency_sha256": input_receipt.get("asset_and_dependency_sha256"),
        "blender_version": config.get("blender_version"),
        "resolution": config.get("resolution"),
        "samples": config.get("samples"),
        "studio": config.get("studio"),
        "sample_unit": "generator_class",
        "image_policy": "one_uniform_blender_render_per_generator",
        "generator_count": len(records),
        "builtin_no_picture_count": builtin_count,
        "picture_backed_count": picture_count,
        "raw_unique_image_count": len(raw_paths),
        "uniform_image_dimensions": list(next(iter(image_sizes))),
    }
    return DatasetBundle(
        tuple(records),
        raw_paths,
        {path: index for index, path in enumerate(raw_paths)},
        summary,
    )


def load_rgb_image(path: Path) -> Image.Image:
    """Load RGB and composite alpha over white for consistent preprocessing."""
    with Image.open(path) as image:
        image.load()
        if "A" not in image.getbands() and "transparency" not in image.info:
            return image.convert("RGB")
        rgba = image.convert("RGBA")
        background = Image.new("RGBA", rgba.size, (255, 255, 255, 255))
        return Image.alpha_composite(background, rgba).convert("RGB")


def l2_normalize(features: np.ndarray) -> np.ndarray:
    features = np.asarray(features, dtype=np.float32)
    if features.ndim != 2:
        raise ValueError(f"expected a 2D feature matrix, got {features.shape}")
    norms = np.linalg.norm(features, axis=1, keepdims=True)
    if np.any(norms <= 0):
        raise ValueError("feature matrix contains a zero vector")
    return features / norms


def aggregate_class_features(
    raw_features: np.ndarray,
    bundle: DatasetBundle,
) -> np.ndarray:
    """Mean each class's normalized image vectors, then normalize the means."""
    normalized = l2_normalize(raw_features)
    centers: list[np.ndarray] = []
    for record in bundle.records:
        indices = [bundle.raw_path_to_index[path.resolve()] for path in record.image_paths]
        centers.append(normalized[indices].mean(axis=0))
    return l2_normalize(np.stack(centers, axis=0))


def _model_fingerprint(model_path: Path) -> str:
    """Fingerprint the exact local model files, including weight contents."""
    model_path = model_path.resolve(strict=True)
    files = sorted(path for path in model_path.iterdir() if path.is_file())
    if not files:
        raise ValueError(f"model directory has no files: {model_path}")
    digest = hashlib.sha256()
    for path in files:
        stat = path.stat()
        digest.update(path.name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(stat.st_size).encode("ascii"))
        digest.update(b"\0")
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                digest.update(block)
        digest.update(b"\n")
    return digest.hexdigest()


def extract_image_features(
    image_paths: Sequence[Path],
    *,
    model_path: Path,
    batch_size: int,
    device: str,
    num_workers: int,
    use_amp: bool,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Extract normalized image embeddings using a local Hugging Face vision model."""
    if not image_paths:
        raise ValueError("cannot extract features for an empty image list")
    if batch_size < 1 or num_workers < 1:
        raise ValueError("batch_size and num_workers must be positive")

    import torch
    from transformers import AutoImageProcessor, AutoModel

    model_path = model_path.expanduser().resolve(strict=True)
    if not model_path.is_dir():
        raise NotADirectoryError(model_path)
    resolved_device = device
    if device == "auto":
        resolved_device = "cuda:0" if torch.cuda.is_available() else "cpu"
    torch_device = torch.device(resolved_device)
    if torch_device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(f"CUDA device requested but unavailable: {resolved_device}")

    processor = AutoImageProcessor.from_pretrained(
        str(model_path), local_files_only=True, use_fast=False
    )
    model = AutoModel.from_pretrained(str(model_path), local_files_only=True)
    model.eval().to(torch_device)
    model_type = str(getattr(model.config, "model_type", type(model).__name__)).lower()
    if model_type == "clip":
        feature_source = "vision_model.visual_projection"
        vision_config = getattr(model.config, "vision_config", None)
        patch_size = getattr(vision_config, "patch_size", None)
        if patch_size:
            hidden_size = getattr(vision_config, "hidden_size", None)
            variant = "B" if hidden_size == 768 else "L" if hidden_size == 1024 else ""
            encoder_label = f"CLIP ViT-{variant}/{patch_size}" if variant else f"CLIP ViT/{patch_size}"
        else:
            encoder_label = "CLIP"
    elif model_type == "dinov2":
        feature_source = "last_hidden_state.cls_token"
        encoder_label = "DINOv2"
    else:
        raise ValueError(f"unsupported model_type {model_type!r}; expected dinov2 or clip")

    amp_enabled = bool(use_amp and torch_device.type == "cuda")
    batches: list[np.ndarray] = []
    with ThreadPoolExecutor(max_workers=num_workers) as loader_pool:
        for start in range(0, len(image_paths), batch_size):
            batch_paths = image_paths[start : start + batch_size]
            images = list(loader_pool.map(load_rgb_image, batch_paths))
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
                        batch_features = model.visual_projection(vision_outputs.pooler_output)
                    else:
                        outputs = model(pixel_values=pixel_values)
                        batch_features = outputs.last_hidden_state[:, 0]
                batch_features = torch.nn.functional.normalize(batch_features.float(), dim=1)
            batches.append(batch_features.cpu().numpy())
            processed = min(start + batch_size, len(image_paths))
            print(f"[features] {encoder_label}: {processed}/{len(image_paths)} images", flush=True)

    features = np.concatenate(batches, axis=0).astype(np.float32, copy=False)
    info = {
        "model_type": model_type,
        "encoder_label": encoder_label,
        "feature_source": feature_source,
        "feature_dim": int(features.shape[1]),
        "image_count": int(features.shape[0]),
        "device": str(torch_device),
        "amp": amp_enabled,
        "batch_size": batch_size,
        "num_workers": num_workers,
        "image_processor_use_fast": False,
        "alpha_composite_background_rgb": [255, 255, 255],
    }
    del model, processor
    if torch_device.type == "cuda":
        torch.cuda.empty_cache()
    return features, info


def _paths_fingerprint(paths: Sequence[Path]) -> str:
    """Fingerprint ordered input paths and bytes for strict cache reuse."""
    digest = hashlib.sha256()
    for path in paths:
        path = path.resolve(strict=True)
        stat = path.stat()
        digest.update(str(path).encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(stat.st_size).encode("ascii"))
        digest.update(b"\0")
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                digest.update(block)
        digest.update(b"\n")
    return digest.hexdigest()


def load_or_extract_features(
    bundle: DatasetBundle,
    *,
    model_name: str,
    model_path: Path,
    output_dir: Path,
    batch_size: int,
    device: str,
    num_workers: int,
    use_amp: bool,
    force_extract: bool,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any], bool]:
    """Load a compatible raw cache or extract it, then build class centers."""
    model_dir = output_dir / model_name
    model_dir.mkdir(parents=True, exist_ok=True)
    raw_path = model_dir / "raw_image_features.npy"
    manifest_path = model_dir / "feature_manifest.json"
    model_path = model_path.expanduser().resolve(strict=True)
    model_fingerprint = _model_fingerprint(model_path)
    path_fingerprint = _paths_fingerprint(bundle.raw_image_paths)
    expected = {
        "schema_version": 2,
        "model_path": str(model_path),
        "model_fingerprint": model_fingerprint,
        "raw_image_count": len(bundle.raw_image_paths),
        "raw_paths_fingerprint": path_fingerprint,
        "preprocessing": {
            "image_processor_use_fast": False,
            "alpha_composite_background_rgb": [255, 255, 255],
        },
    }
    reused = False
    extraction_info: dict[str, Any]
    raw_features: np.ndarray | None = None
    if not force_extract and raw_path.is_file() and manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            raw_features = np.load(raw_path, allow_pickle=False)
            if (
                isinstance(manifest, dict)
                and all(manifest.get(key) == value for key, value in expected.items())
                and isinstance(manifest.get("extraction"), dict)
                and raw_features.ndim == 2
                and raw_features.shape[0] == len(bundle.raw_image_paths)
                and raw_features.shape[1] == manifest["extraction"].get("feature_dim")
                and raw_features.dtype == np.float32
                and np.isfinite(raw_features).all()
            ):
                extraction_info = dict(manifest.get("extraction") or {})
                reused = True
                print(f"[features] reused {raw_path}", flush=True)
            else:
                raw_features = None
        except (OSError, ValueError, json.JSONDecodeError):
            raw_features = None
    if raw_features is None:
        raw_features, extraction_info = extract_image_features(
            bundle.raw_image_paths,
            model_path=model_path,
            batch_size=batch_size,
            device=device,
            num_workers=num_workers,
            use_amp=use_amp,
        )
        temporary = model_dir / ".raw_image_features.npy.tmp"
        with temporary.open("wb") as stream:
            np.save(stream, raw_features, allow_pickle=False)
        temporary.replace(raw_path)
        manifest = {**expected, "dtype": str(raw_features.dtype), "extraction": extraction_info}
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
            encoding="utf-8",
        )

    class_features = aggregate_class_features(raw_features, bundle)
    np.save(model_dir / "class_features.npy", class_features, allow_pickle=False)
    return raw_features, class_features, extraction_info, reused


def compute_tsne(
    features: np.ndarray,
    *,
    perplexity: float,
    random_state: int,
    max_iter: int,
    n_jobs: int,
    verbose: int,
) -> tuple[np.ndarray, dict[str, Any]]:
    """PCA then deterministic two-dimensional t-SNE."""
    from sklearn.decomposition import PCA
    from sklearn.manifold import TSNE

    features = l2_normalize(features)
    count, dimension = features.shape
    if count < 3:
        raise ValueError("t-SNE requires at least three classes")
    if perplexity <= 0 or max_iter < 250:
        raise ValueError("perplexity must be positive and max_iter must be at least 250")
    actual_perplexity = min(float(perplexity), max(1.0, (count - 1) / 3.0))
    pca_components = min(50, count - 1, dimension)
    projected = PCA(n_components=pca_components, random_state=random_state).fit_transform(features)
    kwargs = dict(
        n_components=2,
        perplexity=actual_perplexity,
        early_exaggeration=12.0,
        learning_rate="auto",
        init="pca",
        method="barnes_hut",
        angle=0.5,
        random_state=random_state,
        verbose=verbose,
        n_jobs=n_jobs,
    )
    try:
        reducer = TSNE(max_iter=max_iter, **kwargs)
    except TypeError:  # scikit-learn < 1.5
        reducer = TSNE(n_iter=max_iter, **kwargs)
    coordinates = np.asarray(reducer.fit_transform(projected), dtype=np.float32)
    return coordinates, {
        "sample_count": count,
        "input_dim": dimension,
        "pca_components": pca_components,
        "perplexity": actual_perplexity,
        "random_state": random_state,
        "max_iter": max_iter,
        "n_jobs": n_jobs,
    }


def _records_for_csv(bundle: DatasetBundle) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for record in bundle.records:
        picture_dir = str(record.picture_dir) if record.picture_dir else ""
        rows.append(
            {
                "generator_index": record.generator_index,
                "generator_name": record.generator_name,
                "source_type": record.source_type,
                "picture_category": record.picture_category,
                "picture_label": record.picture_label,
                "picture_dir": picture_dir,
                "image_count": len(record.image_paths),
                "alias_of_generator_index": record.alias_of_generator_index or "",
                "representative_image": str(record.image_paths[0]),
            }
        )
    return rows


def write_coordinates_csv(
    coordinates: np.ndarray,
    bundle: DatasetBundle,
    output_path: Path,
) -> None:
    rows = _records_for_csv(bundle)
    coordinates = np.asarray(coordinates, dtype=np.float32)
    if coordinates.shape != (len(rows), 2):
        raise ValueError("coordinate and record counts differ")
    fields = ["tsne_x", "tsne_y", *rows[0].keys()]
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for point, row in zip(coordinates, rows, strict=True):
            writer.writerow(
                {
                    "tsne_x": format(float(point[0]), ".9g"),
                    "tsne_y": format(float(point[1]), ".9g"),
                    **row,
                }
            )


def _save_source_plot(coordinates: np.ndarray, bundle: DatasetBundle, path: Path, *, title: str, dpi: int) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    source_specs = {
        "picture_backed": ("#2878b5", "o", "picture-backed"),
        "builtin_no_picture": ("#d1495b", "D", "PV-A builtin (no picture)"),
    }
    figure, axis = plt.subplots(figsize=(12, 9), dpi=dpi)
    for source, (color, marker, label) in source_specs.items():
        indices = np.array([record.source_type == source for record in bundle.records], dtype=bool)
        axis.scatter(
            coordinates[indices, 0],
            coordinates[indices, 1],
            s=31 if source == "picture_backed" else 42,
            alpha=0.78,
            color=color,
            marker=marker,
            edgecolors="white" if source == "builtin_no_picture" else "none",
            linewidths=0.35,
            label=f"{label} (n={int(indices.sum())})",
            rasterized=True,
        )
    axis.set_title(title)
    axis.set_xlabel("t-SNE 1")
    axis.set_ylabel("t-SNE 2")
    axis.grid(color="#d8d8d8", linewidth=0.45, alpha=0.6)
    axis.spines[["top", "right"]].set_visible(False)
    axis.legend(frameon=False, loc="best")
    figure.tight_layout()
    figure.savefig(path, bbox_inches="tight")
    plt.close(figure)


def _save_category_plot(coordinates: np.ndarray, bundle: DatasetBundle, path: Path, *, title: str, dpi: int) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = [
        record.picture_category if record.source_type == "picture_backed" else "builtin_no_picture"
        for record in bundle.records
    ]
    unique_labels = sorted(set(labels))
    palette = plt.get_cmap("turbo", max(2, len(unique_labels)))
    figure, axis = plt.subplots(figsize=(14, 10), dpi=dpi)
    for label_index, label in enumerate(unique_labels):
        mask = np.array([value == label for value in labels], dtype=bool)
        is_builtin = label == "builtin_no_picture"
        axis.scatter(
            coordinates[mask, 0],
            coordinates[mask, 1],
            s=42 if is_builtin else 28,
            alpha=0.82,
            color="#d1495b" if is_builtin else palette(label_index),
            marker="D" if is_builtin else "o",
            edgecolors="white" if is_builtin else "none",
            linewidths=0.35,
            label=f"PV-A builtin (n={int(mask.sum())})" if is_builtin else f"{label} (n={int(mask.sum())})",
            rasterized=True,
        )
    axis.set_title(title)
    axis.set_xlabel("t-SNE 1")
    axis.set_ylabel("t-SNE 2")
    axis.grid(color="#d8d8d8", linewidth=0.45, alpha=0.6)
    axis.spines[["top", "right"]].set_visible(False)
    axis.legend(
        frameon=False,
        loc="center left",
        bbox_to_anchor=(1.01, 0.5),
        fontsize=8,
        ncol=2,
        markerscale=1.25,
    )
    figure.tight_layout(rect=(0, 0, 0.82, 1))
    figure.savefig(path, bbox_inches="tight")
    plt.close(figure)


def _save_builtin_label_plot(coordinates: np.ndarray, bundle: DatasetBundle, path: Path, *, title: str, dpi: int) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    picture_mask = np.array(
        [record.source_type == "picture_backed" for record in bundle.records], dtype=bool
    )
    builtin_mask = ~picture_mask
    figure, axis = plt.subplots(figsize=(14, 10), dpi=dpi)
    axis.scatter(
        coordinates[picture_mask, 0],
        coordinates[picture_mask, 1],
        s=22,
        alpha=0.20,
        color="#8aa6b8",
        label="picture-backed (context)",
        rasterized=True,
    )
    axis.scatter(
        coordinates[builtin_mask, 0],
        coordinates[builtin_mask, 1],
        s=46,
        alpha=0.94,
        color="#d1495b",
        marker="D",
        edgecolors="white",
        linewidths=0.45,
        label="PV-A builtin",
        rasterized=True,
    )
    for point, record in zip(coordinates[builtin_mask], (r for r in bundle.records if r.source_type != "picture_backed"), strict=True):
        axis.annotate(
            record.generator_index,
            (float(point[0]), float(point[1])),
            xytext=(3, 3),
            textcoords="offset points",
            fontsize=5.8,
            alpha=0.9,
        )
    axis.set_title(title)
    axis.set_xlabel("t-SNE 1")
    axis.set_ylabel("t-SNE 2")
    axis.grid(color="#d8d8d8", linewidth=0.45, alpha=0.6)
    axis.spines[["top", "right"]].set_visible(False)
    axis.legend(frameon=False, loc="best")
    figure.tight_layout()
    figure.savefig(path, bbox_inches="tight")
    plt.close(figure)


def _save_source_comparison(
    output_dir: Path,
    model_names: Sequence[str],
    *,
    output_path: Path,
) -> None:
    """Make a compact side-by-side overview of the encoder source plots."""
    source_images: list[Image.Image] = []
    try:
        for model_name in model_names:
            source_path = output_dir / model_name / "tsne_by_source.png"
            if not source_path.is_file():
                continue
            with Image.open(source_path) as image:
                copy = image.convert("RGB")
            copy.thumbnail((1200, 900), Image.Resampling.LANCZOS)
            source_images.append(copy)
        if not source_images:
            return
        gap = 24
        canvas_width = sum(image.width for image in source_images) + gap * (len(source_images) - 1)
        canvas_height = max(image.height for image in source_images)
        canvas = Image.new("RGB", (canvas_width, canvas_height), "white")
        offset = 0
        for image in source_images:
            canvas.paste(image, (offset, 0))
            offset += image.width + gap
        output_path.parent.mkdir(parents=True, exist_ok=True)
        canvas.save(output_path)
    finally:
        for image in source_images:
            image.close()


def visualize_model(
    bundle: DatasetBundle,
    class_features: np.ndarray,
    *,
    model_name: str,
    extraction_info: dict[str, Any],
    output_dir: Path,
    perplexity: float,
    random_state: int,
    max_iter: int,
    tsne_jobs: int,
    tsne_verbose: int,
    plot_dpi: int,
) -> dict[str, Any]:
    model_dir = output_dir / model_name
    model_dir.mkdir(parents=True, exist_ok=True)
    coordinates, tsne_info = compute_tsne(
        class_features,
        perplexity=perplexity,
        random_state=random_state,
        max_iter=max_iter,
        n_jobs=tsne_jobs,
        verbose=tsne_verbose,
    )
    np.save(model_dir / "class_features.npy", class_features, allow_pickle=False)
    np.save(model_dir / "tsne_coordinates.npy", coordinates, allow_pickle=False)
    write_coordinates_csv(coordinates, bundle, model_dir / "tsne_coordinates.csv")
    encoder_label = str(extraction_info.get("encoder_label") or model_name)
    uniform_input = bundle.summary.get("input_mode") == "uniform_blender_seed_0000"
    source_title = "roster origin" if uniform_input else "image source"
    category_title = "mapped object category" if uniform_input else "picture category"
    _save_source_plot(
        coordinates,
        bundle,
        model_dir / "tsne_by_source.png",
        title=(
            f"{encoder_label} t-SNE: {len(bundle.records)} uniformly rendered classes "
            f"by {source_title}"
        ),
        dpi=plot_dpi,
    )
    _save_category_plot(
        coordinates,
        bundle,
        model_dir / "tsne_by_picture_category.png",
        title=f"{encoder_label} t-SNE: classes by {category_title}",
        dpi=plot_dpi,
    )
    _save_builtin_label_plot(
        coordinates,
        bundle,
        model_dir / "tsne_builtin_labels.png",
        title=f"{encoder_label} t-SNE: builtin generator IDs highlighted",
        dpi=plot_dpi,
    )
    summary = {
        "encoder_label": encoder_label,
        "feature_dim": int(class_features.shape[1]),
        "class_count": int(class_features.shape[0]),
        "tsne": tsne_info,
        "artifacts": {
            "raw_image_features": "raw_image_features.npy",
            "class_features": "class_features.npy",
            "tsne_coordinates": "tsne_coordinates.npy",
            "tsne_coordinates_csv": "tsne_coordinates.csv",
            "tsne_by_source": "tsne_by_source.png",
            "tsne_by_picture_category": "tsne_by_picture_category.png",
            "tsne_builtin_labels": "tsne_builtin_labels.png",
        },
    }
    (model_dir / "visualization_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    return summary


def _slug_model_name(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    if not value:
        raise ValueError("model name cannot be empty")
    return value


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Extract DINOv2/CLIP features and t-SNE the 531 mapped generator classes."
    )
    parser.add_argument("--index-csv", type=Path, default=DEFAULT_INDEX_CSV)
    parser.add_argument("--picture-root", type=Path, default=DEFAULT_PICTURE_ROOT)
    parser.add_argument("--builtin-root", type=Path, default=DEFAULT_BUILTIN_ROOT)
    parser.add_argument(
        "--uniform-render-root",
        type=Path,
        help=(
            "Use one audited Blender seed_0000 render per generator from this root "
            "instead of the mixed reference-picture/builtin inputs."
        ),
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--dino-model", type=Path, default=DEFAULT_DINO_MODEL)
    parser.add_argument("--clip-model", type=Path, default=DEFAULT_CLIP_MODEL)
    parser.add_argument(
        "--models",
        nargs="+",
        choices=("dinov2", "clip"),
        default=("dinov2", "clip"),
        help="Encoders to run (default: both).",
    )
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--device", default="auto", help="Torch device, e.g. auto, cpu, cuda:0.")
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--no-amp", action="store_false", dest="use_amp")
    parser.set_defaults(use_amp=True)
    parser.add_argument("--perplexity", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--tsne-max-iter", type=int, default=1_000)
    parser.add_argument("--tsne-jobs", type=int, default=min(8, os.cpu_count() or 1))
    parser.add_argument("--tsne-verbose", type=int, choices=(0, 1, 2), default=1)
    parser.add_argument("--plot-dpi", type=int, default=180)
    parser.add_argument("--force-extract", action="store_true")
    parser.add_argument(
        "--allow-count-drift",
        action="store_true",
        help="Allow a non-531 roster when testing on a custom CSV.",
    )
    return parser


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.uniform_render_root is not None:
        bundle = discover_uniform_render_records(
            args.index_csv,
            render_root=args.uniform_render_root,
            strict_counts=not args.allow_count_drift,
        )
    else:
        bundle = discover_generator_records(
            args.index_csv,
            picture_root=args.picture_root,
            builtin_root=args.builtin_root,
            strict_counts=not args.allow_count_drift,
        )
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "dataset_manifest.json").write_text(
        json.dumps(bundle.summary, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    # Keep the full roster alongside the coordinates so a plot is auditable
    # without reopening the source CSV.
    with (output_dir / "generator_roster_resolved.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        rows = _records_for_csv(bundle)
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (output_dir / "raw_image_index.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=["raw_index", "image_path"])
        writer.writeheader()
        writer.writerows(
            {"raw_index": index, "image_path": str(path)}
            for index, path in enumerate(bundle.raw_image_paths)
        )

    model_paths = {"dinov2": args.dino_model, "clip": args.clip_model}
    model_summaries: dict[str, Any] = {}
    for model_name in args.models:
        model_path = model_paths[model_name].expanduser().resolve(strict=True)
        print(
            f"[run] {model_name}: {len(bundle.records)} classes, "
            f"{len(bundle.raw_image_paths)} unique source images",
            flush=True,
        )
        raw_features, class_features, extraction_info, reused = load_or_extract_features(
            bundle,
            model_name=model_name,
            model_path=model_path,
            output_dir=output_dir,
            batch_size=args.batch_size,
            device=args.device,
            num_workers=args.num_workers,
            use_amp=args.use_amp,
            force_extract=args.force_extract,
        )
        summary = visualize_model(
            bundle,
            class_features,
            model_name=model_name,
            extraction_info=extraction_info,
            output_dir=output_dir,
            perplexity=args.perplexity,
            random_state=args.seed,
            max_iter=args.tsne_max_iter,
            tsne_jobs=args.tsne_jobs,
            tsne_verbose=args.tsne_verbose,
            plot_dpi=args.plot_dpi,
        )
        summary["feature_cache_reused"] = reused
        summary["model_path"] = str(model_path)
        model_summaries[model_name] = summary
        # Drop the matrices before loading the next encoder.
        del raw_features, class_features

    comparison_path = output_dir / "tsne_source_comparison.png"
    _save_source_comparison(output_dir, tuple(model_summaries), output_path=comparison_path)

    manifest = {
        "schema_version": 1,
        "dataset": bundle.summary,
        "models": model_summaries,
        "artifacts": {
            "dataset_manifest": "dataset_manifest.json",
            "generator_roster": "generator_roster_resolved.csv",
            "raw_image_index": "raw_image_index.csv",
            "source_comparison": "tsne_source_comparison.png",
        },
        "config": {
            "uniform_render_root": (
                str(args.uniform_render_root.expanduser().resolve())
                if args.uniform_render_root is not None
                else None
            ),
            "batch_size": args.batch_size,
            "device": args.device,
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
    }
    (output_dir / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    manifest = run(args)
    print(json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
