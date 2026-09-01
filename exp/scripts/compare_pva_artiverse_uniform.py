#!/usr/bin/env python3
"""Compare PV-A and Artiverse from one audited Blender studio protocol.

PV-A contributes 531 frozen generator classes, one seed-0000 render per
class.  The primary Artiverse protocol contributes 84 content-independent
representatives, one per category, selected from the official 3,544-model
universe by a frozen identity-hash rule.  The unselected Artiverse models are
used only to validate the selection universe and report category support; no
image or feature from them enters the primary comparison.  A full-model mode
is retained only for compatibility.

Each encoder is projected independently.  Joint plots concatenate PV-A and
Artiverse in the encoder's original feature space before PCA50 + t-SNE; no
independently fitted two-dimensional coordinates are overlaid.
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
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

for _thread_variable in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_thread_variable, "1")

import numpy as np
from PIL import Image, ImageStat


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
PVA_HELPER_SCRIPT = REPO_ROOT / "exp" / "scripts" / "visualize_pva531_picture_tsne.py"
DEFAULT_PVA_DIR = REPO_ROOT / "exp" / "runtime" / "pva531_uniform_tsne"
DEFAULT_ARTIVERSE_RENDER_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/Artiverse/renders/"
    "uniform84_one_per_category_studio_256_v1"
)
DEFAULT_OUTPUT_DIR = (
    REPO_ROOT / "exp" / "runtime" / "pva_artiverse_uniform_one_shot_comparison"
)
EXPECTED_PVA_COUNT = 531
EXPECTED_ARTIVERSE_MODEL_COUNT = 3544
EXPECTED_ARTIVERSE_CATEGORY_COUNT = 84
EXPECTED_ARTIVERSE_CONTRACT = "artiverse_uniform_studio_v1"
EXPECTED_PVA_CONTRACT = "pva531_uniform_studio_v1"
SUCCESS_STATUSES = {"rendered", "reused_valid"}
ENCODERS = ("dinov2", "clip")
ENCODER_MODEL_TYPES = {"dinov2": "dinov2", "clip": "clip"}
SHA256_RE = re.compile(r"[0-9a-f]{64}")


@dataclass(frozen=True, slots=True)
class PvaRecord:
    generator_index: str
    generator_name: str
    source_type: str
    picture_category: str

    @property
    def identity(self) -> tuple[str, str]:
        return (self.generator_index, self.generator_name)


@dataclass(frozen=True, slots=True)
class PvaBundle:
    root: Path
    records: tuple[PvaRecord, ...]
    features: dict[str, np.ndarray]
    feature_manifests: dict[str, dict[str, Any]]
    run_manifest: dict[str, Any]
    render_config: dict[str, Any]
    receipts: dict[str, Any]


@dataclass(frozen=True, slots=True)
class ArtiverseModel:
    ordinal: int
    category: str
    source: str
    model_id: str
    manifest_root: str
    glb_path: Path
    image_path: Path
    image_sha256: str
    image_bytes: int
    category_one_shot: bool

    @property
    def identity(self) -> tuple[str, str, str]:
        return (self.category, self.source, self.model_id)


@dataclass(frozen=True, slots=True)
class ArtiverseRenderBundle:
    root: Path
    records: tuple[ArtiverseModel, ...]
    config: dict[str, Any]
    categories: tuple[str, ...]
    universe_model_count: int
    universe_source_counts: dict[str, dict[str, int]]
    one_shot_only: bool
    render_manifest_path: Path
    dataset_manifest_path: Path
    receipts: dict[str, Any]


@dataclass(frozen=True, slots=True)
class CategoryRecord:
    category_id: int
    category: str
    model_count: int
    source_count: int
    source_counts: dict[str, int]
    one_shot_model_index: int
    one_shot_source: str
    one_shot_model_id: str
    one_shot_manifest_root: str


_PVA_HELPER: Any | None = None


def _pva_helper() -> Any:
    """Load the existing extractor/t-SNE implementation without copying it."""
    global _PVA_HELPER
    if _PVA_HELPER is None:
        name = "_pva531_uniform_compare_helper"
        spec = importlib.util.spec_from_file_location(name, PVA_HELPER_SCRIPT)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot load PV-A helper: {PVA_HELPER_SCRIPT}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        _PVA_HELPER = module
    return _PVA_HELPER


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _file_receipt(path: Path) -> dict[str, Any]:
    path = path.resolve(strict=True)
    return {"path": str(path), "sha256": sha256_file(path), "size_bytes": path.stat().st_size}


def _load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"unreadable JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return payload


def _atomic_write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _atomic_write_csv(path: Path, fieldnames: Sequence[str], rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fieldnames))
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def _atomic_save_npy(path: Path, array: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("wb") as stream:
        np.save(stream, np.asarray(array), allow_pickle=False)
    temporary.replace(path)


def l2_normalize(features: np.ndarray) -> np.ndarray:
    features = np.asarray(features, dtype=np.float32)
    if features.ndim != 2 or not np.isfinite(features).all():
        raise ValueError("features must be a finite two-dimensional matrix")
    norms = np.linalg.norm(features, axis=1, keepdims=True)
    if np.any(norms <= 0):
        raise ValueError("features contain a zero vector")
    return np.asarray(features / norms, dtype=np.float32)


def _load_feature_matrix(path: Path, *, rows: int, dimension: int) -> np.ndarray:
    try:
        matrix = np.load(path, allow_pickle=False)
    except (OSError, ValueError) as exc:
        raise ValueError(f"unreadable feature matrix: {path}") from exc
    if matrix.shape != (rows, dimension) or matrix.dtype != np.float32:
        raise ValueError(
            f"unexpected feature matrix at {path}: {matrix.shape}/{matrix.dtype}; "
            f"expected {(rows, dimension)}/float32"
        )
    if not np.isfinite(matrix).all():
        raise ValueError(f"non-finite feature matrix: {path}")
    if not np.allclose(np.linalg.norm(matrix, axis=1), 1.0, rtol=2e-5, atol=2e-5):
        raise ValueError(f"feature rows are not L2 normalized: {path}")
    return np.asarray(matrix, dtype=np.float32)


def _resolve_declared_file(value: Any, *, parent: Path, field: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"missing path field {field}")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = parent / path
    return path.resolve(strict=True)


def load_pva_frozen(root: Path, *, strict_counts: bool = True) -> PvaBundle:
    """Load the frozen PV-A class matrices and verify their exact model snapshots."""
    root = root.expanduser().resolve(strict=True)
    roster_path = root / "generator_roster_resolved.csv"
    run_manifest_path = root / "run_manifest.json"
    if not roster_path.is_file() or not run_manifest_path.is_file():
        raise FileNotFoundError(f"incomplete PV-A frozen run: {root}")
    with roster_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = {"generator_index", "generator_name", "source_type", "picture_category"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f"PV-A roster lacks required fields: {roster_path}")
    records: list[PvaRecord] = []
    identities: set[tuple[str, str]] = set()
    for ordinal, row in enumerate(rows, start=1):
        index = row["generator_index"].strip()
        name = row["generator_name"].strip()
        if not index or not name:
            raise ValueError(f"empty PV-A identity at roster row {ordinal + 1}")
        if strict_counts and index != f"G{ordinal:04d}":
            raise ValueError(f"PV-A roster order mismatch at row {ordinal + 1}: {index}")
        record = PvaRecord(index, name, row["source_type"].strip(), row["picture_category"].strip())
        if record.identity in identities:
            raise ValueError(f"duplicate PV-A identity: {record.identity}")
        identities.add(record.identity)
        records.append(record)
    if strict_counts and len(records) != EXPECTED_PVA_COUNT:
        raise ValueError(f"expected {EXPECTED_PVA_COUNT} PV-A rows, found {len(records)}")

    run_manifest = _load_json(run_manifest_path)
    dataset = run_manifest.get("dataset")
    if not isinstance(dataset, dict):
        raise ValueError("PV-A run manifest has no dataset object")
    if dataset.get("render_contract") != EXPECTED_PVA_CONTRACT:
        raise ValueError(f"unexpected PV-A render contract: {dataset.get('render_contract')!r}")
    if dataset.get("input_mode") != "uniform_blender_seed_0000":
        raise ValueError("PV-A run is not the uniform Blender seed_0000 run")
    if int(dataset.get("generator_count", -1)) != len(records):
        raise ValueError("PV-A run manifest and roster counts differ")
    render_config_path = _resolve_declared_file(
        dataset.get("render_config"), parent=root, field="dataset.render_config"
    )
    declared_render_hash = dataset.get("render_config_sha256")
    if declared_render_hash != sha256_file(render_config_path):
        raise ValueError("PV-A render config SHA mismatch")
    render_config = _load_json(render_config_path)
    if render_config.get("schema_version") != 2 or render_config.get("render_contract") != EXPECTED_PVA_CONTRACT:
        raise ValueError("PV-A render config is not the audited schema-v2 contract")
    if int(render_config.get("generator_count", -1)) != len(records):
        raise ValueError("PV-A render config and roster counts differ")
    if not isinstance(render_config.get("studio"), dict):
        raise ValueError("PV-A render config has no studio contract")
    pva_renderer_path = _resolve_declared_file(
        render_config.get("renderer"), parent=render_config_path.parent, field="PV-A renderer"
    )
    pva_renderer_hash = sha256_file(pva_renderer_path)
    if render_config.get("renderer_sha256") != pva_renderer_hash:
        raise ValueError("PV-A renderer file no longer matches its frozen SHA")

    features: dict[str, np.ndarray] = {}
    feature_manifests: dict[str, dict[str, Any]] = {}
    receipts: dict[str, Any] = {
        "roster": _file_receipt(roster_path),
        "run_manifest": _file_receipt(run_manifest_path),
        "render_config": _file_receipt(render_config_path),
        "renderer": _file_receipt(pva_renderer_path),
        "encoders": {},
    }
    helper = _pva_helper()
    for key in ENCODERS:
        encoder_dir = root / key
        manifest_path = encoder_dir / "feature_manifest.json"
        feature_path = encoder_dir / "class_features.npy"
        manifest = _load_json(manifest_path)
        extraction = manifest.get("extraction")
        if manifest.get("schema_version") != 2 or not isinstance(extraction, dict):
            raise ValueError(f"invalid PV-A {key} feature manifest")
        if extraction.get("model_type") != ENCODER_MODEL_TYPES[key]:
            raise ValueError(f"PV-A {key} model_type mismatch")
        preprocessing = manifest.get("preprocessing")
        if preprocessing != {
            "image_processor_use_fast": False,
            "alpha_composite_background_rgb": [255, 255, 255],
        }:
            raise ValueError(f"PV-A {key} preprocessing contract mismatch")
        if int(manifest.get("raw_image_count", -1)) != len(records):
            raise ValueError(f"PV-A {key} feature count differs from roster")
        dimension = int(extraction.get("feature_dim", 0))
        if dimension < 1:
            raise ValueError(f"PV-A {key} feature dimension is invalid")
        model_path = _resolve_declared_file(
            manifest.get("model_path"), parent=encoder_dir, field=f"{key}.model_path"
        )
        actual_model_fingerprint = helper._model_fingerprint(model_path)
        if manifest.get("model_fingerprint") != actual_model_fingerprint:
            raise ValueError(f"PV-A {key} local model snapshot fingerprint mismatch")
        matrix = _load_feature_matrix(feature_path, rows=len(records), dimension=dimension)
        features[key] = matrix
        feature_manifests[key] = manifest
        receipts["encoders"][key] = {
            "feature_manifest": _file_receipt(manifest_path),
            "class_features": {
                **_file_receipt(feature_path),
                "shape": list(matrix.shape),
                "dtype": str(matrix.dtype),
            },
            "model_path": str(model_path),
            "model_fingerprint": actual_model_fingerprint,
        }
    return PvaBundle(
        root=root,
        records=tuple(records),
        features=features,
        feature_manifests=feature_manifests,
        run_manifest=run_manifest,
        render_config=render_config,
        receipts=receipts,
    )


def _manifest_roots(dataset_manifest: Mapping[str, Any]) -> tuple[str, ...]:
    if dataset_manifest.get("format") != "artiverse-data-tar-gz-chunks-v1":
        raise ValueError("unsupported Artiverse dataset manifest format")
    chunks = dataset_manifest.get("chunks")
    if not isinstance(chunks, list) or not chunks:
        raise ValueError("Artiverse dataset manifest has no chunks")
    roots: list[str] = []
    for chunk in chunks:
        if not isinstance(chunk, dict) or not isinstance(chunk.get("roots"), list):
            raise ValueError("invalid Artiverse dataset chunk")
        roots.extend(str(value) for value in chunk["roots"])
    if len(roots) != len(set(roots)):
        raise ValueError("duplicate roots in Artiverse dataset manifest")
    if int(dataset_manifest.get("model_count", -1)) != len(roots):
        raise ValueError("Artiverse dataset manifest model count differs from roots")
    return tuple(roots)


def _parse_bool(value: Any, *, field: str) -> bool:
    normalized = str(value).strip().lower()
    if normalized in {"true", "1"}:
        return True
    if normalized in {"false", "0"}:
        return False
    raise ValueError(f"invalid boolean {field}: {value!r}")


def _inside(root: Path, path: Path, *, field: str) -> Path:
    root = root.resolve(strict=True)
    path = path.expanduser().resolve(strict=True)
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{field} escapes {root}: {path}") from exc
    return path


def load_artiverse_uniform(
    root: Path,
    *,
    pva_render_config: Mapping[str, Any],
    strict_counts: bool = True,
    verify_glb_hashes: bool = True,
    one_shot_only: bool = False,
) -> ArtiverseRenderBundle:
    """Load full renders or only the 84 frozen representatives.

    Selected-only mode uses the official 3,544-root manifest solely as an
    identity universe.  It never resolves, hashes, or reads an unselected GLB
    or PNG.
    """
    root = root.expanduser().resolve(strict=True)
    config_path = root / "render_config.json"
    manifest_path = root / "render_manifest.csv"
    one_shot_path = root / "category_one_shot_roster.csv"
    if not config_path.is_file() or not manifest_path.is_file() or not one_shot_path.is_file():
        raise FileNotFoundError(f"incomplete Artiverse uniform render root: {root}")
    config = _load_json(config_path)
    if config.get("schema_version") != 1 or config.get("render_contract") != EXPECTED_ARTIVERSE_CONTRACT:
        raise ValueError("unexpected Artiverse uniform render contract")
    for field in ("resolution", "samples", "studio", "blender_version"):
        if config.get(field) != pva_render_config.get(field):
            raise ValueError(f"Artiverse/PV-A render contract mismatch for {field}")
    if not isinstance(config.get("input_receipt"), dict):
        raise ValueError("Artiverse render config has no input receipt")
    for field in ("pose_policy", "material_policy"):
        if not isinstance(config.get(field), str) or not config[field].strip():
            raise ValueError(f"Artiverse render config has no {field}")
    renderer_path = _resolve_declared_file(
        config.get("renderer"), parent=config_path.parent, field="Artiverse renderer"
    )
    shared_renderer_path = _resolve_declared_file(
        config.get("shared_renderer"), parent=config_path.parent, field="shared_renderer"
    )
    renderer_hash = sha256_file(renderer_path)
    shared_renderer_hash = sha256_file(shared_renderer_path)
    if config.get("renderer_sha256") != renderer_hash:
        raise ValueError("Artiverse renderer file no longer matches its frozen SHA")
    if config.get("shared_renderer_sha256") != shared_renderer_hash:
        raise ValueError("Artiverse shared renderer file no longer matches its frozen SHA")
    if shared_renderer_hash != pva_render_config.get("renderer_sha256"):
        raise ValueError("Artiverse did not use the frozen PV-A shared renderer helper")

    dataset_manifest_path = _resolve_declared_file(
        config.get("dataset_manifest"), parent=root, field="dataset_manifest"
    )
    if config.get("dataset_manifest_sha256") != sha256_file(dataset_manifest_path):
        raise ValueError("Artiverse dataset manifest SHA mismatch")
    dataset_manifest = _load_json(dataset_manifest_path)
    official_roots = _manifest_roots(dataset_manifest)
    if int(config.get("model_count", -1)) != len(official_roots):
        raise ValueError("Artiverse render config model count differs from official roster")
    if strict_counts and len(official_roots) != EXPECTED_ARTIVERSE_MODEL_COUNT:
        raise ValueError(
            f"expected {EXPECTED_ARTIVERSE_MODEL_COUNT} Artiverse models, found {len(official_roots)}"
        )

    universe_categories: list[str] = []
    universe_source_counts: dict[str, Counter[str]] = {}
    winners: dict[str, str] = {}
    official_ordinals: dict[str, int] = {}
    for ordinal, manifest_root in enumerate(official_roots, start=1):
        parts = Path(manifest_root).parts
        if len(parts) != 4 or parts[0] != "data":
            raise ValueError(f"invalid official Artiverse manifest root: {manifest_root}")
        _, category, source, _model_id = parts
        if category not in universe_source_counts:
            universe_categories.append(category)
            universe_source_counts[category] = Counter()
        universe_source_counts[category][source] += 1
        official_ordinals[manifest_root] = ordinal
        previous = winners.get(category)
        if previous is None or (
            hashlib.sha256(manifest_root.encode("utf-8")).hexdigest(), manifest_root
        ) < (hashlib.sha256(previous.encode("utf-8")).hexdigest(), previous):
            winners[category] = manifest_root
    expected_selected_roots = tuple(
        root_value
        for root_value in official_roots
        if winners[Path(root_value).parts[1]] == root_value
    )
    expected_manifest_roots = expected_selected_roots if one_shot_only else official_roots

    data_root = _resolve_declared_file(config.get("data_root"), parent=root, field="data_root")
    with manifest_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = {
        "ordinal", "category", "source", "model_id", "manifest_root", "glb_path",
        "glb_bytes", "glb_sha256", "identity_sha256", "category_one_shot",
        "output_path", "status", "png_bytes", "png_sha256",
    }
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f"Artiverse render manifest lacks required fields: {manifest_path}")
    if len(rows) != len(expected_manifest_roots):
        mode = "one-shot roster" if one_shot_only else "official roster"
        raise ValueError(f"Artiverse render manifest and {mode} counts differ")

    resolution = int(config["resolution"])
    records: list[ArtiverseModel] = []
    seen_identities: set[tuple[str, str, str]] = set()
    seen_outputs: set[Path] = set()
    for row_number, (row, official_root) in enumerate(
        zip(rows, expected_manifest_roots, strict=True), start=2
    ):
        try:
            declared_ordinal = int(row["ordinal"])
        except ValueError as exc:
            raise ValueError(f"invalid Artiverse ordinal at manifest row {row_number}") from exc
        if (
            declared_ordinal != official_ordinals[official_root]
            or row["manifest_root"].strip() != official_root
        ):
            raise ValueError(f"Artiverse ordered roster mismatch at manifest row {row_number}")
        parts = Path(official_root).parts
        if len(parts) != 4 or parts[0] != "data":
            raise ValueError(f"invalid official Artiverse manifest root: {official_root}")
        _, category, source, model_id = parts
        if (row["category"].strip(), row["source"].strip(), row["model_id"].strip()) != (
            category, source, model_id
        ):
            raise ValueError(f"Artiverse identity fields disagree with {official_root}")
        identity = (category, source, model_id)
        if identity in seen_identities:
            raise ValueError(f"duplicate Artiverse model identity: {identity}")
        seen_identities.add(identity)
        if row["status"].strip() not in SUCCESS_STATUSES:
            raise ValueError(f"Artiverse render is not successful: {official_root}")

        glb_path = _inside(data_root, Path(row["glb_path"]), field="glb_path")
        expected_glb = (data_root / category / source / model_id / f"{model_id}.segmented.glb").resolve()
        if glb_path != expected_glb:
            raise ValueError(f"Artiverse GLB path mismatch for {official_root}")
        try:
            declared_glb_bytes = int(row["glb_bytes"])
        except ValueError as exc:
            raise ValueError(f"invalid GLB size for {official_root}") from exc
        if glb_path.stat().st_size != declared_glb_bytes:
            raise ValueError(f"Artiverse GLB size mismatch for {official_root}")
        glb_hash = row["glb_sha256"].strip()
        if SHA256_RE.fullmatch(glb_hash) is None:
            raise ValueError(f"invalid GLB SHA for {official_root}")
        if verify_glb_hashes and sha256_file(glb_path) != glb_hash:
            raise ValueError(f"Artiverse GLB SHA mismatch for {official_root}")
        if SHA256_RE.fullmatch(row["identity_sha256"].strip()) is None:
            raise ValueError(f"invalid identity SHA for {official_root}")

        image_path = _inside(root, Path(row["output_path"]), field="output_path")
        expected_image = (root / category / source / model_id / "imgs" / "000.png").resolve()
        if image_path != expected_image or image_path in seen_outputs:
            raise ValueError(f"Artiverse output path mismatch or duplicate for {official_root}")
        seen_outputs.add(image_path)
        try:
            image_bytes = int(row["png_bytes"])
        except ValueError as exc:
            raise ValueError(f"invalid PNG size for {official_root}") from exc
        image_hash = row["png_sha256"].strip()
        if image_path.stat().st_size != image_bytes or SHA256_RE.fullmatch(image_hash) is None:
            raise ValueError(f"Artiverse PNG receipt invalid for {official_root}")
        if sha256_file(image_path) != image_hash:
            raise ValueError(f"Artiverse PNG SHA mismatch for {official_root}")
        with Image.open(image_path) as image:
            image.verify()
            if image.size != (resolution, resolution):
                raise ValueError(f"Artiverse PNG dimensions mismatch for {official_root}")
        records.append(
            ArtiverseModel(
                ordinal=declared_ordinal,
                category=category,
                source=source,
                model_id=model_id,
                manifest_root=official_root,
                glb_path=glb_path,
                image_path=image_path,
                image_sha256=image_hash,
                image_bytes=image_bytes,
                category_one_shot=_parse_bool(
                    row["category_one_shot"], field=f"category_one_shot[{official_root}]"
                ),
            )
        )

    categories = tuple(universe_categories)
    if int(config.get("category_count", -1)) != len(categories):
        raise ValueError("Artiverse render config category count differs from manifest")
    if strict_counts and len(categories) != EXPECTED_ARTIVERSE_CATEGORY_COUNT:
        raise ValueError(
            f"expected {EXPECTED_ARTIVERSE_CATEGORY_COUNT} Artiverse categories, found {len(categories)}"
        )
    one_shot = [record for record in records if record.category_one_shot]
    if len(one_shot) != len(categories) or Counter(record.category for record in one_shot) != Counter(categories):
        raise ValueError("Artiverse one-shot flags are not exactly one per category")
    for category in categories:
        candidates = [record for record in records if record.category == category]
        expected_root = winners[category]
        selected = next(record for record in candidates if record.category_one_shot)
        if selected.manifest_root != expected_root:
            raise ValueError(f"Artiverse one-shot policy mismatch for category {category}")

    with one_shot_path.open("r", encoding="utf-8", newline="") as stream:
        one_shot_rows = list(csv.DictReader(stream))
    if len(one_shot_rows) != len(one_shot):
        raise ValueError("Artiverse one-shot roster row count mismatch")
    roster_roots = [row.get("manifest_root", "").strip() for row in one_shot_rows]
    if roster_roots != list(expected_selected_roots):
        raise ValueError("Artiverse one-shot roster differs from official hash selection")
    roster_identities = [
        (row.get("category", "").strip(), row.get("source", "").strip(), row.get("model_id", "").strip())
        for row in one_shot_rows
    ]
    if roster_identities != [record.identity for record in one_shot]:
        raise ValueError("Artiverse one-shot roster order differs from main manifest")

    receipts = {
        "render_config": _file_receipt(config_path),
        "render_manifest": _file_receipt(manifest_path),
        "category_one_shot_roster": _file_receipt(one_shot_path),
        "dataset_manifest": _file_receipt(dataset_manifest_path),
        "renderer": _file_receipt(renderer_path),
        "shared_renderer": _file_receipt(shared_renderer_path),
        "ordered_image_receipt_sha256": hashlib.sha256(
            "\n".join(
                f"{record.manifest_root}\0{record.image_bytes}\0{record.image_sha256}"
                for record in records
            ).encode("utf-8")
        ).hexdigest(),
    }
    return ArtiverseRenderBundle(
        root=root,
        records=tuple(records),
        config=config,
        categories=categories,
        universe_model_count=len(official_roots),
        universe_source_counts={
            category: dict(sorted(counts.items()))
            for category, counts in universe_source_counts.items()
        },
        one_shot_only=one_shot_only,
        render_manifest_path=manifest_path,
        dataset_manifest_path=dataset_manifest_path,
        receipts=receipts,
    )


def aggregate_artiverse_features(
    model_features: np.ndarray,
    bundle: ArtiverseRenderBundle,
) -> tuple[np.ndarray, np.ndarray, tuple[CategoryRecord, ...]]:
    """Return full category centroids, frozen one-shot rows, and metadata."""
    model_features = l2_normalize(model_features)
    if model_features.shape[0] != len(bundle.records):
        raise ValueError("Artiverse model features and ordered roster differ")
    full_features: list[np.ndarray] = []
    one_shot_features: list[np.ndarray] = []
    category_records: list[CategoryRecord] = []
    for category_id, category in enumerate(bundle.categories, start=1):
        indices = [index for index, record in enumerate(bundle.records) if record.category == category]
        if not indices:
            raise AssertionError(f"category has no models: {category}")
        selected = [index for index in indices if bundle.records[index].category_one_shot]
        if len(selected) != 1:
            raise ValueError(f"category does not have exactly one one-shot model: {category}")
        full_features.append(model_features[indices].mean(axis=0))
        one_shot_features.append(model_features[selected[0]])
        source_counts = bundle.universe_source_counts.get(category)
        if not isinstance(source_counts, dict) or not source_counts:
            source_counts = dict(Counter(bundle.records[index].source for index in indices))
        model_count = sum(source_counts.values())
        selected_record = bundle.records[selected[0]]
        category_records.append(
            CategoryRecord(
                category_id=category_id,
                category=category,
                model_count=model_count,
                source_count=len(source_counts),
                source_counts=dict(sorted(source_counts.items())),
                one_shot_model_index=selected[0],
                one_shot_source=selected_record.source,
                one_shot_model_id=selected_record.model_id,
                one_shot_manifest_root=selected_record.manifest_root,
            )
        )
    return (
        l2_normalize(np.stack(full_features)),
        l2_normalize(np.stack(one_shot_features)),
        tuple(category_records),
    )


def _ordered_image_fingerprint(records: Sequence[ArtiverseModel]) -> str:
    digest = hashlib.sha256()
    for record in records:
        digest.update(record.manifest_root.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(record.image_path).encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(record.image_bytes).encode("ascii"))
        digest.update(b"\0")
        digest.update(record.image_sha256.encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def load_or_extract_artiverse_features(
    bundle: ArtiverseRenderBundle,
    *,
    encoder: str,
    pva_feature_manifest: Mapping[str, Any],
    output_dir: Path,
    batch_size: int,
    device: str,
    num_workers: int,
    use_amp: bool,
    force_extract: bool,
) -> tuple[np.ndarray, dict[str, Any], bool]:
    """Extract with the exact local snapshot pinned by the PV-A frozen run."""
    if encoder not in ENCODERS:
        raise ValueError(f"unsupported encoder: {encoder}")
    helper = _pva_helper()
    encoder_dir = output_dir / encoder
    encoder_dir.mkdir(parents=True, exist_ok=True)
    feature_path = encoder_dir / (
        "artiverse_one_shot_features.npy"
        if bundle.one_shot_only
        else "artiverse_model_features.npy"
    )
    manifest_path = encoder_dir / "feature_manifest.json"
    model_path = Path(str(pva_feature_manifest["model_path"])).expanduser().resolve(strict=True)
    model_fingerprint = helper._model_fingerprint(model_path)
    if model_fingerprint != pva_feature_manifest.get("model_fingerprint"):
        raise ValueError(f"{encoder} model snapshot no longer matches PV-A")
    expected = {
        "schema_version": 1,
        "encoder": encoder,
        "model_type": ENCODER_MODEL_TYPES[encoder],
        "model_path": str(model_path),
        "model_fingerprint": model_fingerprint,
        "model_count": len(bundle.records),
        "artiverse_sample_mode": "one_shot_only" if bundle.one_shot_only else "full",
        "official_universe_model_count": bundle.universe_model_count,
        "ordered_image_fingerprint": _ordered_image_fingerprint(bundle.records),
        "artiverse_render_manifest_sha256": bundle.receipts["render_manifest"]["sha256"],
        "preprocessing": {
            "image_processor_use_fast": False,
            "alpha_composite_background_rgb": [255, 255, 255],
        },
    }
    reused = False
    extraction: dict[str, Any]
    matrix: np.ndarray | None = None
    if not force_extract and feature_path.is_file() and manifest_path.is_file():
        try:
            manifest = _load_json(manifest_path)
            extraction_value = manifest.get("extraction")
            dimension = int(extraction_value.get("feature_dim", 0)) if isinstance(extraction_value, dict) else 0
            if all(manifest.get(key) == value for key, value in expected.items()):
                matrix = _load_feature_matrix(feature_path, rows=len(bundle.records), dimension=dimension)
                extraction = dict(extraction_value)
                if extraction.get("model_type") != ENCODER_MODEL_TYPES[encoder]:
                    matrix = None
                else:
                    reused = True
                    print(f"[features] reused {feature_path}", flush=True)
        except (OSError, ValueError, TypeError):
            matrix = None
    if matrix is None:
        image_paths = tuple(record.image_path for record in bundle.records)
        matrix, extraction = helper.extract_image_features(
            image_paths,
            model_path=model_path,
            batch_size=batch_size,
            device=device,
            num_workers=num_workers,
            use_amp=use_amp,
        )
        matrix = l2_normalize(matrix)
        if extraction.get("model_type") != ENCODER_MODEL_TYPES[encoder]:
            raise ValueError(f"extracted {encoder} model_type differs from PV-A")
        if int(extraction.get("feature_dim", -1)) != int(
            pva_feature_manifest["extraction"]["feature_dim"]
        ):
            raise ValueError(f"extracted {encoder} feature dimension differs from PV-A")
        _atomic_save_npy(feature_path, matrix)
        _atomic_write_json(
            manifest_path,
            {**expected, "dtype": str(matrix.dtype), "extraction": extraction},
        )
    return matrix, extraction, reused


def _category_colors(count: int) -> tuple[str, ...]:
    """Generate exact-unique, deterministic colors for category identity."""
    import colorsys

    colors: list[str] = []
    for index in range(count):
        hue = (index * 0.6180339887498949) % 1.0
        saturation = 0.58 + 0.16 * ((index % 3) / 2.0)
        value = 0.70 + 0.22 * (((index // 3) % 3) / 2.0)
        red, green, blue = colorsys.hsv_to_rgb(hue, saturation, value)
        color = f"#{round(red * 255):02x}{round(green * 255):02x}{round(blue * 255):02x}"
        if color in colors:
            # This branch is not expected for 84 entries, but keeps exact uniqueness explicit.
            color = f"#{(index * 2654435761 & 0xFFFFFF):06x}"
        colors.append(color)
    if len(set(colors)) != count:
        raise AssertionError("category palette is not unique")
    return tuple(colors)


def _compute_tsne(
    features: np.ndarray,
    *,
    perplexity: float,
    random_state: int,
    max_iter: int,
    n_jobs: int,
    verbose: int,
) -> tuple[np.ndarray, dict[str, Any]]:
    helper = _pva_helper()
    return helper.compute_tsne(
        features,
        perplexity=perplexity,
        random_state=random_state,
        max_iter=max_iter,
        n_jobs=n_jobs,
        verbose=verbose,
    )


def _model_coordinate_rows(
    coordinates: np.ndarray,
    records: Sequence[ArtiverseModel],
) -> list[dict[str, Any]]:
    return [
        {
            "tsne_x": format(float(point[0]), ".9g"),
            "tsne_y": format(float(point[1]), ".9g"),
            "model_index": index,
            "category": record.category,
            "source": record.source,
            "model_id": record.model_id,
            "manifest_root": record.manifest_root,
            "category_one_shot": str(record.category_one_shot).lower(),
        }
        for index, (point, record) in enumerate(zip(coordinates, records, strict=True))
    ]


def _category_coordinate_rows(
    coordinates: np.ndarray,
    records: Sequence[CategoryRecord],
    *,
    representation: str,
) -> list[dict[str, Any]]:
    return [
        {
            "tsne_x": format(float(point[0]), ".9g"),
            "tsne_y": format(float(point[1]), ".9g"),
            "category_id": f"C{record.category_id:02d}",
            "category": record.category,
            "representation": representation,
            "support_count": record.model_count if representation == "full" else 1,
            "full_model_count": record.model_count,
            "one_shot_source": record.one_shot_source,
            "one_shot_model_id": record.one_shot_model_id,
            "one_shot_manifest_root": record.one_shot_manifest_root,
        }
        for point, record in zip(coordinates, records, strict=True)
    ]


def _joint_coordinate_rows(
    coordinates: np.ndarray,
    pva_records: Sequence[PvaRecord],
    art_records: Sequence[CategoryRecord],
    *,
    representation: str,
) -> list[dict[str, Any]]:
    if coordinates.shape != (len(pva_records) + len(art_records), 2):
        raise ValueError("joint coordinate count mismatch")
    rows: list[dict[str, Any]] = []
    for index, (point, record) in enumerate(zip(coordinates[: len(pva_records)], pva_records, strict=True)):
        rows.append(
            {
                "tsne_x": format(float(point[0]), ".9g"),
                "tsne_y": format(float(point[1]), ".9g"),
                "joint_index": index,
                "dataset": "PV-A",
                "representation": "one_seed_per_generator",
                "class_id": record.generator_index,
                "class_name": record.generator_name,
                "support_count": 1,
                "source": record.source_type,
                "source_identity": "",
            }
        )
    offset = len(pva_records)
    for local_index, (point, record) in enumerate(
        zip(coordinates[offset:], art_records, strict=True)
    ):
        rows.append(
            {
                "tsne_x": format(float(point[0]), ".9g"),
                "tsne_y": format(float(point[1]), ".9g"),
                "joint_index": offset + local_index,
                "dataset": "Artiverse",
                "representation": representation,
                "class_id": f"C{record.category_id:02d}",
                "class_name": record.category,
                "support_count": record.model_count if representation == "full" else 1,
                "source": record.one_shot_source if representation == "one_shot" else "multi_source",
                "source_identity": (
                    record.one_shot_manifest_root if representation == "one_shot" else ""
                ),
            }
        )
    return rows


def _save_artiverse_plot(
    coordinates: np.ndarray,
    labels: Sequence[str],
    categories: Sequence[str],
    colors: Sequence[str],
    path: Path,
    *,
    title: str,
    annotate: bool,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    color_by_category = dict(zip(categories, colors, strict=True))
    figure, axis = plt.subplots(figsize=(13, 9), dpi=dpi)
    point_colors = [color_by_category[label] for label in labels]
    size = 45 if len(labels) <= 100 else 17
    axis.scatter(
        coordinates[:, 0], coordinates[:, 1], s=size, c=point_colors,
        alpha=0.82, edgecolors="white" if len(labels) <= 100 else "none",
        linewidths=0.4, rasterized=True,
    )
    if annotate:
        category_ids = {category: f"C{index:02d}" for index, category in enumerate(categories, start=1)}
        for point, label in zip(coordinates, labels, strict=True):
            axis.annotate(category_ids[label], point, xytext=(4, 3), textcoords="offset points", fontsize=6.2)
    axis.set_title(title)
    axis.set_xlabel("t-SNE 1")
    axis.set_ylabel("t-SNE 2")
    axis.grid(color="#d8d8d8", linewidth=0.45, alpha=0.55)
    axis.spines[["top", "right"]].set_visible(False)
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def _save_joint_plot(
    coordinates: np.ndarray,
    pva_count: int,
    path: Path,
    *,
    encoder_label: str,
    representation: str,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(12, 9), dpi=dpi)
    axis.scatter(
        coordinates[:pva_count, 0], coordinates[:pva_count, 1], s=25,
        color="#d1495b", marker="o", alpha=0.70, edgecolors="none",
        label=f"PV-A generator classes (n={pva_count})", rasterized=True,
    )
    axis.scatter(
        coordinates[pva_count:, 0], coordinates[pva_count:, 1], s=49,
        color="#2878b5", marker="^", alpha=0.88, edgecolors="white", linewidths=0.35,
        label=f"Artiverse {representation} categories (n={len(coordinates) - pva_count})",
        rasterized=True,
    )
    axis.set_title(f"Joint {encoder_label} t-SNE: PV-A vs Artiverse ({representation})")
    axis.set_xlabel("joint t-SNE 1")
    axis.set_ylabel("joint t-SNE 2")
    axis.grid(color="#d8d8d8", linewidth=0.45, alpha=0.55)
    axis.spines[["top", "right"]].set_visible(False)
    axis.legend(frameon=False, loc="best")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def _cosine_distances(features: np.ndarray) -> np.ndarray:
    normalized = l2_normalize(features)
    distances = 1.0 - normalized @ normalized.T
    distances = np.clip(distances, 0.0, 2.0)
    np.fill_diagonal(distances, 0.0)
    return distances


def _summary(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    if values.size == 0 or not np.isfinite(values).all():
        raise ValueError("cannot summarize empty or non-finite values")
    return {
        "count": int(values.size),
        "mean": float(values.mean()),
        "std": float(values.std()),
        "min": float(values.min()),
        "q10": float(np.quantile(values, 0.10)),
        "median": float(np.median(values)),
        "q90": float(np.quantile(values, 0.90)),
        "max": float(values.max()),
    }


def _spearman(left: np.ndarray, right: np.ndarray) -> float:
    from scipy.stats import spearmanr

    result = spearmanr(np.asarray(left).reshape(-1), np.asarray(right).reshape(-1))
    statistic = float(result.statistic)
    if not math.isfinite(statistic):
        raise ValueError("Spearman correlation is not finite")
    return statistic


def _neighbor_overlap_from_distances(
    first: np.ndarray,
    second: np.ndarray,
    *,
    fraction: float,
) -> dict[str, Any]:
    if first.shape != second.shape or first.ndim != 2 or first.shape[0] != first.shape[1]:
        raise ValueError("neighbor distance matrices must be equal square matrices")
    count = first.shape[0]
    if count < 3 or not 0 < fraction < 1:
        raise ValueError("neighbor overlap needs at least three samples and a valid fraction")
    k = max(1, min(count - 1, round(fraction * (count - 1))))
    first_work = first.copy()
    second_work = second.copy()
    np.fill_diagonal(first_work, np.inf)
    np.fill_diagonal(second_work, np.inf)
    first_neighbors = np.argsort(first_work, axis=1, kind="stable")[:, :k]
    second_neighbors = np.argsort(second_work, axis=1, kind="stable")[:, :k]
    per_sample = np.asarray(
        [len(set(left) & set(right)) / k for left, right in zip(first_neighbors, second_neighbors, strict=True)],
        dtype=np.float64,
    )
    chance = k / (count - 1)
    adjusted = (float(per_sample.mean()) - chance) / (1.0 - chance) if chance < 1 else 1.0
    return {
        "fraction": fraction,
        "k": k,
        "raw_overlap": float(per_sample.mean()),
        "chance_overlap": chance,
        "chance_adjusted_overlap": adjusted,
        "per_sample": _summary(per_sample),
    }


def encoder_agreement_metrics(
    dinov2_features: np.ndarray,
    clip_features: np.ndarray,
    *,
    neighbor_fraction: float,
) -> dict[str, Any]:
    if dinov2_features.shape[0] != clip_features.shape[0]:
        raise ValueError("DINOv2 and CLIP sample counts differ")
    dino_distances = _cosine_distances(dinov2_features)
    clip_distances = _cosine_distances(clip_features)
    upper = np.triu_indices(len(dino_distances), 1)
    return {
        "sample_count": len(dino_distances),
        "pair_count": len(upper[0]),
        "pairwise_cosine_distance_spearman": _spearman(
            dino_distances[upper], clip_distances[upper]
        ),
        "neighbor_agreement": _neighbor_overlap_from_distances(
            dino_distances, clip_distances, fraction=neighbor_fraction
        ),
        "dinov2_pairwise_cosine_distance": _summary(dino_distances[upper]),
        "clip_pairwise_cosine_distance": _summary(clip_distances[upper]),
    }


def _directional_knn_mixing(
    distances: np.ndarray,
    *,
    query_slice: slice,
    other_start: int,
    k: int,
    expected_other_fraction: float,
) -> dict[str, Any]:
    query_indices = np.arange(distances.shape[0])[query_slice]
    fractions: list[float] = []
    for query_index in query_indices:
        row = distances[query_index].copy()
        row[query_index] = np.inf
        neighbors = np.argsort(row, kind="stable")[:k]
        fractions.append(float(np.mean(neighbors >= other_start)))
    values = np.asarray(fractions, dtype=np.float64)
    adjusted = (
        (float(values.mean()) - expected_other_fraction) / (1.0 - expected_other_fraction)
        if expected_other_fraction < 1
        else 1.0
    )
    return {
        "raw_other_source_fraction": float(values.mean()),
        "random_mixing_expectation": expected_other_fraction,
        "chance_adjusted_mixing": adjusted,
        "per_query": _summary(values),
    }


def cross_dataset_metrics(
    pva_features: np.ndarray,
    artiverse_features: np.ndarray,
    *,
    neighbor_fraction: float,
) -> dict[str, Any]:
    """Measure high-dimensional cross-source proximity within one encoder."""
    from sklearn.metrics import silhouette_samples

    pva = l2_normalize(pva_features)
    artiverse = l2_normalize(artiverse_features)
    if pva.shape[1] != artiverse.shape[1]:
        raise ValueError("cross-dataset feature dimensions differ")
    similarities = pva @ artiverse.T
    pva_nearest_index = np.argmax(similarities, axis=1)
    art_nearest_index = np.argmax(similarities, axis=0)
    pva_nearest_distance = 1.0 - similarities[np.arange(len(pva)), pva_nearest_index]
    art_nearest_distance = 1.0 - similarities[art_nearest_index, np.arange(len(artiverse))]
    mutual_pairs = [
        (int(pva_index), int(art_index))
        for pva_index, art_index in enumerate(pva_nearest_index)
        if art_nearest_index[art_index] == pva_index
    ]

    combined = np.vstack([pva, artiverse])
    combined_distances = _cosine_distances(combined)
    total = len(combined)
    k = max(1, min(total - 2, round(neighbor_fraction * (total - 1))))
    pva_mixing = _directional_knn_mixing(
        combined_distances,
        query_slice=slice(0, len(pva)),
        other_start=len(pva),
        k=k,
        expected_other_fraction=len(artiverse) / (total - 1),
    )
    # The helper above counts indices >= other_start.  For Artiverse queries the
    # opposite source is PV-A, so calculate this direction explicitly.
    art_fractions: list[float] = []
    for query_index in range(len(pva), total):
        row = combined_distances[query_index].copy()
        row[query_index] = np.inf
        neighbors = np.argsort(row, kind="stable")[:k]
        art_fractions.append(float(np.mean(neighbors < len(pva))))
    art_values = np.asarray(art_fractions, dtype=np.float64)
    art_expected = len(pva) / (total - 1)
    art_mixing = {
        "raw_other_source_fraction": float(art_values.mean()),
        "random_mixing_expectation": art_expected,
        "chance_adjusted_mixing": (float(art_values.mean()) - art_expected) / (1.0 - art_expected),
        "per_query": _summary(art_values),
    }
    labels = np.concatenate(
        [np.zeros(len(pva), dtype=np.int8), np.ones(len(artiverse), dtype=np.int8)]
    )
    silhouette = silhouette_samples(combined, labels, metric="cosine")
    silhouette_pva = float(np.mean(silhouette[: len(pva)]))
    silhouette_art = float(np.mean(silhouette[len(pva) :]))
    return {
        "pva_count": len(pva),
        "artiverse_count": len(artiverse),
        "nearest_other_source_cosine_distance": {
            "pva_to_artiverse": _summary(pva_nearest_distance),
            "artiverse_to_pva": _summary(art_nearest_distance),
        },
        "mutual_cross_source_nearest_neighbors": {
            "pair_count": len(mutual_pairs),
            "rate_over_pva": len(mutual_pairs) / len(pva),
            "rate_over_artiverse": len(mutual_pairs) / len(artiverse),
        },
        "source_silhouette_cosine": {
            "pva_mean": silhouette_pva,
            "artiverse_mean": silhouette_art,
            "source_macro_mean": (silhouette_pva + silhouette_art) / 2.0,
            "interpretation": "higher means a larger rendered-source/domain gap, not better quality",
        },
        "knn_source_mixing": {
            "fraction": neighbor_fraction,
            "k": k,
            "pva_queries": pva_mixing,
            "artiverse_queries": art_mixing,
            "source_macro_chance_adjusted": (
                pva_mixing["chance_adjusted_mixing"] + art_mixing["chance_adjusted_mixing"]
            )
            / 2.0,
        },
    }


def support_sensitivity_metrics(
    full_features: np.ndarray,
    one_shot_features: np.ndarray,
    *,
    neighbor_fraction: float,
) -> dict[str, Any]:
    full = l2_normalize(full_features)
    one_shot = l2_normalize(one_shot_features)
    if full.shape != one_shot.shape:
        raise ValueError("full and one-shot feature shapes differ")
    full_distances = _cosine_distances(full)
    one_shot_distances = _cosine_distances(one_shot)
    upper = np.triu_indices(len(full), 1)
    matched = 1.0 - np.sum(full * one_shot, axis=1)
    return {
        "sample_count": len(full),
        "matched_category_cosine_distance": _summary(matched),
        "pairwise_structure_spearman": _spearman(
            full_distances[upper], one_shot_distances[upper]
        ),
        "neighbor_agreement": _neighbor_overlap_from_distances(
            full_distances, one_shot_distances, fraction=neighbor_fraction
        ),
    }


def projection_trustworthiness(
    features: np.ndarray,
    coordinates: np.ndarray,
    *,
    neighbor_fraction: float,
) -> dict[str, Any]:
    from sklearn.manifold import trustworthiness

    count = len(features)
    k = max(1, min((count - 1) // 2, round(neighbor_fraction * (count - 1))))
    score = trustworthiness(
        l2_normalize(features), np.asarray(coordinates, dtype=np.float32),
        n_neighbors=k, metric="cosine",
    )
    return {"sample_count": count, "k": k, "score": float(score)}


def compute_high_dimensional_metrics(
    pva_features: Mapping[str, np.ndarray],
    art_full: Mapping[str, np.ndarray],
    art_one_shot: Mapping[str, np.ndarray],
    joint_coordinates: Mapping[str, Mapping[str, np.ndarray]],
    *,
    neighbor_fraction: float,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "schema_version": 1,
        "protocol": {
            "distance": "cosine distance on L2-normalized high-dimensional embeddings",
            "encoder_agreement": "Spearman over all upper-triangle pair distances",
            "neighbor_fraction": neighbor_fraction,
            "neighbor_overlap_adjustment": "(observed - k/(n-1)) / (1 - k/(n-1))",
            "cross_dataset_direction_policy": "report both directions, then equal-source macro",
            "tsne_policy": "visualization only; projection trustworthiness is reported separately",
        },
        "encoder_agreement": {
            "pva": encoder_agreement_metrics(
                pva_features["dinov2"], pva_features["clip"],
                neighbor_fraction=neighbor_fraction,
            ),
            "artiverse_full": encoder_agreement_metrics(
                art_full["dinov2"], art_full["clip"],
                neighbor_fraction=neighbor_fraction,
            ),
            "artiverse_one_shot": encoder_agreement_metrics(
                art_one_shot["dinov2"], art_one_shot["clip"],
                neighbor_fraction=neighbor_fraction,
            ),
        },
        "per_encoder": {},
    }
    for encoder in ENCODERS:
        result["per_encoder"][encoder] = {
            "pva_vs_artiverse_full": cross_dataset_metrics(
                pva_features[encoder], art_full[encoder], neighbor_fraction=neighbor_fraction
            ),
            "pva_vs_artiverse_one_shot": cross_dataset_metrics(
                pva_features[encoder], art_one_shot[encoder], neighbor_fraction=neighbor_fraction
            ),
            "artiverse_support_sensitivity": support_sensitivity_metrics(
                art_full[encoder], art_one_shot[encoder], neighbor_fraction=neighbor_fraction
            ),
            "joint_projection_trustworthiness": {
                representation: projection_trustworthiness(
                    np.vstack([pva_features[encoder], art_full[encoder] if representation == "full" else art_one_shot[encoder]]),
                    joint_coordinates[encoder][representation],
                    neighbor_fraction=neighbor_fraction,
                )
                for representation in ("full", "one_shot")
            },
        }
    return result


def compute_one_shot_high_dimensional_metrics(
    pva_features: Mapping[str, np.ndarray],
    art_one_shot: Mapping[str, np.ndarray],
    joint_coordinates: Mapping[str, Mapping[str, np.ndarray]],
    *,
    neighbor_fraction: float,
) -> dict[str, Any]:
    """Compute only metrics supported by the official 84-image protocol."""
    result: dict[str, Any] = {
        "schema_version": 1,
        "artiverse_mode": "one_shot_only",
        "protocol": {
            "distance": "cosine distance on L2-normalized high-dimensional embeddings",
            "encoder_agreement": "Spearman over all upper-triangle pair distances",
            "neighbor_fraction": neighbor_fraction,
            "neighbor_overlap_adjustment": "(observed - k/(n-1)) / (1 - k/(n-1))",
            "cross_dataset_direction_policy": "report both directions, then equal-source macro",
            "tsne_policy": "visualization only; projection trustworthiness is reported separately",
        },
        "encoder_agreement": {
            "pva": encoder_agreement_metrics(
                pva_features["dinov2"], pva_features["clip"],
                neighbor_fraction=neighbor_fraction,
            ),
            "artiverse_one_shot": encoder_agreement_metrics(
                art_one_shot["dinov2"], art_one_shot["clip"],
                neighbor_fraction=neighbor_fraction,
            ),
        },
        "per_encoder": {},
    }
    for encoder in ENCODERS:
        joint_features = np.vstack([pva_features[encoder], art_one_shot[encoder]])
        result["per_encoder"][encoder] = {
            "pva_vs_artiverse_one_shot": cross_dataset_metrics(
                pva_features[encoder], art_one_shot[encoder],
                neighbor_fraction=neighbor_fraction,
            ),
            "joint_projection_trustworthiness": {
                "one_shot": projection_trustworthiness(
                    joint_features, joint_coordinates[encoder]["one_shot"],
                    neighbor_fraction=neighbor_fraction,
                )
            },
        }
    return result


def _save_joint_comparison(
    joint_coordinates: Mapping[str, Mapping[str, np.ndarray]],
    *,
    pva_count: int,
    labels: Mapping[str, str],
    representations: Sequence[str],
    path: Path,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(
        2, len(representations),
        figsize=(11 * len(representations), 16), dpi=dpi, squeeze=False,
    )
    for row, encoder in enumerate(ENCODERS):
        for column, representation in enumerate(representations):
            coordinates = joint_coordinates[encoder][representation]
            axis = axes[row, column]
            axis.scatter(
                coordinates[:pva_count, 0], coordinates[:pva_count, 1],
                s=18, color="#d1495b", marker="o", alpha=0.68, edgecolors="none",
                label=f"PV-A (n={pva_count})", rasterized=True,
            )
            axis.scatter(
                coordinates[pva_count:, 0], coordinates[pva_count:, 1],
                s=42, color="#2878b5", marker="^", alpha=0.88,
                edgecolors="white", linewidths=0.3,
                label=f"Artiverse {representation} (n={len(coordinates) - pva_count})",
                rasterized=True,
            )
            axis.set_title(f"{labels[encoder]}: Artiverse {representation}")
            axis.set_xlabel("joint t-SNE 1")
            axis.set_ylabel("joint t-SNE 2")
            axis.grid(color="#d8d8d8", linewidth=0.4, alpha=0.52)
            axis.spines[["top", "right"]].set_visible(False)
            axis.legend(frameon=False, loc="best")
    figure.suptitle(
        "Uniform Blender comparison: PV-A generator classes and Artiverse categories",
        fontsize=18,
    )
    figure.text(
        0.5, 0.012,
        "Each panel is fitted jointly in one encoder space. Separate panels are not coordinate-aligned.",
        ha="center", fontsize=10, color="#444444",
    )
    figure.tight_layout(rect=(0.0, 0.035, 1.0, 0.965))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def _write_input_indices(
    output_dir: Path,
    art_bundle: ArtiverseRenderBundle,
    category_records: Sequence[CategoryRecord],
    colors: Sequence[str],
) -> dict[str, str]:
    model_rows = [
        {
            "model_index": index,
            "ordinal": record.ordinal,
            "category": record.category,
            "source": record.source,
            "model_id": record.model_id,
            "manifest_root": record.manifest_root,
            "category_one_shot": str(record.category_one_shot).lower(),
            "image_path": str(record.image_path),
            "image_bytes": record.image_bytes,
            "image_sha256": record.image_sha256,
        }
        for index, record in enumerate(art_bundle.records)
    ]
    _atomic_write_csv(output_dir / "artiverse_model_index.csv", list(model_rows[0]), model_rows)
    category_rows = [
        {
            "category_id": f"C{record.category_id:02d}",
            "category": record.category,
            "model_count": record.model_count,
            "source_count": record.source_count,
            "source_counts_json": json.dumps(record.source_counts, sort_keys=True),
            "one_shot_model_index": record.one_shot_model_index,
            "one_shot_source": record.one_shot_source,
            "one_shot_model_id": record.one_shot_model_id,
            "one_shot_manifest_root": record.one_shot_manifest_root,
            "color": color,
        }
        for record, color in zip(category_records, colors, strict=True)
    ]
    _atomic_write_csv(output_dir / "artiverse_category_index.csv", list(category_rows[0]), category_rows)
    one_shot_rows = [
        {
            "category_id": row["category_id"],
            "category": row["category"],
            "model_index": row["one_shot_model_index"],
            "source": row["one_shot_source"],
            "model_id": row["one_shot_model_id"],
            "manifest_root": row["one_shot_manifest_root"],
            "color": row["color"],
        }
        for row in category_rows
    ]
    _atomic_write_csv(output_dir / "artiverse_one_shot_index.csv", list(one_shot_rows[0]), one_shot_rows)
    return {
        "artiverse_model_index": "artiverse_model_index.csv",
        "artiverse_category_index": "artiverse_category_index.csv",
        "artiverse_one_shot_index": "artiverse_one_shot_index.csv",
    }


def _finite_json(value: Any) -> bool:
    if isinstance(value, dict):
        return all(_finite_json(item) for item in value.values())
    if isinstance(value, list):
        return all(_finite_json(item) for item in value)
    if isinstance(value, float):
        return math.isfinite(value)
    return True


def build_stable_run_contract(
    pva: PvaBundle,
    artiverse: ArtiverseRenderBundle,
    args: argparse.Namespace,
) -> dict[str, Any]:
    """Return the immutable scientific/artifact identity of one output root."""
    return {
        "schema_version": 1,
        "artifact_contract": "pva_artiverse_uniform_comparison_v1",
        "artiverse_mode": "one_shot_only" if artiverse.one_shot_only else "full",
        "implementation": {
            "script_sha256": sha256_file(SCRIPT),
            "feature_tsne_helper_sha256": sha256_file(PVA_HELPER_SCRIPT),
        },
        "inputs": {
            "pva_root": str(pva.root),
            "pva_roster_sha256": pva.receipts["roster"]["sha256"],
            "pva_run_manifest_sha256": pva.receipts["run_manifest"]["sha256"],
            "pva_render_config_sha256": pva.receipts["render_config"]["sha256"],
            "pva_encoders": {
                encoder: {
                    "feature_manifest_sha256": pva.receipts["encoders"][encoder]["feature_manifest"]["sha256"],
                    "class_features_sha256": pva.receipts["encoders"][encoder]["class_features"]["sha256"],
                    "model_fingerprint": pva.receipts["encoders"][encoder]["model_fingerprint"],
                }
                for encoder in ENCODERS
            },
            "artiverse_render_root": str(artiverse.root),
            "artiverse_render_config_sha256": artiverse.receipts["render_config"]["sha256"],
            "artiverse_render_manifest_sha256": artiverse.receipts["render_manifest"]["sha256"],
            "artiverse_one_shot_roster_sha256": artiverse.receipts["category_one_shot_roster"]["sha256"],
            "artiverse_dataset_manifest_sha256": artiverse.receipts["dataset_manifest"]["sha256"],
            "artiverse_ordered_image_receipt_sha256": artiverse.receipts["ordered_image_receipt_sha256"],
        },
        "projection_and_metrics": {
            "perplexity": args.perplexity,
            "seed": args.seed,
            "tsne_max_iter": args.tsne_max_iter,
            "tsne_jobs": args.tsne_jobs,
            "tsne_verbose": args.tsne_verbose,
            "neighbor_fraction": args.neighbor_fraction,
            "plot_dpi": args.plot_dpi,
        },
        "counts": {
            "pva": len(pva.records),
            "artiverse_render_samples_used": len(artiverse.records),
            "artiverse_official_universe_models": artiverse.universe_model_count,
            "artiverse_categories": len(artiverse.categories),
        },
    }


def validate_output_contract(output_dir: Path, expected: Mapping[str, Any]) -> None:
    """Reject mixing artifacts from a different frozen run before any write."""
    if not output_dir.exists():
        return
    if not output_dir.is_dir():
        raise ValueError(f"output path is not a directory: {output_dir}")
    entries = list(output_dir.iterdir())
    if not entries:
        return
    contract_path = output_dir / "run_contract.json"
    if contract_path.is_file():
        observed = _load_json(contract_path)
    else:
        manifest_path = output_dir / "run_manifest.json"
        if not manifest_path.is_file():
            raise ValueError(
                f"non-empty output has no recognized run contract: {output_dir}; use a new directory"
            )
        manifest = _load_json(manifest_path)
        observed = manifest.get("stable_run_contract")
        if not isinstance(observed, dict):
            raise ValueError(
                f"existing run manifest has no stable run contract: {manifest_path}; use a new directory"
            )
    if observed != expected:
        raise ValueError(
            f"output directory belongs to a different frozen input/configuration: {output_dir}; "
            "use a new directory"
        )


def audit_outputs(
    output_dir: Path,
    *,
    pva_count: int,
    model_count: int,
    category_count: int,
    feature_dimensions: Mapping[str, int],
    artifact_paths: Sequence[Path],
) -> dict[str, Any]:
    checks: dict[str, bool] = {}
    artifacts: dict[str, dict[str, Any]] = {}
    expected_array_rows = {
        "artiverse_model_features.npy": model_count,
        "artiverse_category_features.npy": category_count,
        "artiverse_one_shot_features.npy": category_count,
    }
    expected_csv_rows = {
        "artiverse_model_index.csv": model_count,
        "artiverse_category_index.csv": category_count,
        "artiverse_one_shot_index.csv": category_count,
        "artiverse_model_tsne_coordinates.csv": model_count,
        "artiverse_category_tsne_coordinates.csv": category_count,
        "artiverse_one_shot_tsne_coordinates.csv": category_count,
        "joint_full_tsne_coordinates.csv": pva_count + category_count,
        "joint_one_shot_tsne_coordinates.csv": pva_count + category_count,
    }
    for path in artifact_paths:
        relative = path.relative_to(output_dir).as_posix()
        if not path.is_file():
            checks[f"exists:{relative}"] = False
            continue
        checks[f"exists:{relative}"] = True
        receipt: dict[str, Any] = {
            "sha256": sha256_file(path),
            "size_bytes": path.stat().st_size,
        }
        if path.suffix == ".npy":
            array = np.load(path, allow_pickle=False)
            expected_rows = expected_array_rows.get(path.name)
            encoder = path.parent.name
            expected_dimension = feature_dimensions.get(encoder)
            checks[f"shape:{relative}"] = (
                expected_rows is not None
                and expected_dimension is not None
                and array.shape == (expected_rows, expected_dimension)
            )
            checks[f"finite:{relative}"] = bool(np.isfinite(array).all())
            checks[f"normalized:{relative}"] = bool(
                array.ndim == 2
                and np.allclose(np.linalg.norm(array, axis=1), 1.0, rtol=2e-5, atol=2e-5)
            )
            receipt.update({"shape": list(array.shape), "dtype": str(array.dtype)})
        elif path.suffix == ".csv":
            with path.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            expected_rows = expected_csv_rows.get(path.name)
            if expected_rows is not None:
                checks[f"rows:{relative}"] = len(rows) == expected_rows
            receipt["row_count"] = len(rows)
        elif path.suffix == ".png":
            with Image.open(path) as image:
                image.load()
                standard_deviation = ImageStat.Stat(image.convert("L")).stddev[0]
                checks[f"dimensions:{relative}"] = image.width >= 1000 and image.height >= 700
                checks[f"nonblank:{relative}"] = standard_deviation > 1.0
                receipt.update(
                    {"width": image.width, "height": image.height, "grayscale_std": standard_deviation}
                )
        elif path.name == "high_dimensional_metrics.json":
            metrics = _load_json(path)
            checks[f"finite:{relative}"] = _finite_json(metrics)
        artifacts[relative] = receipt
    return {
        "schema_version": 1,
        "pass": all(checks.values()),
        "checks": checks,
        "artifacts": artifacts,
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.batch_size < 1 or args.num_workers < 1 or args.plot_dpi < 40:
        raise ValueError("batch size/workers must be positive and plot DPI must be at least 40")
    if not 0 < args.neighbor_fraction < 1:
        raise ValueError("neighbor fraction must be between zero and one")
    output_dir = args.output_dir.expanduser().resolve()
    pva = load_pva_frozen(args.pva_dir, strict_counts=not args.allow_count_drift)
    artiverse = load_artiverse_uniform(
        args.artiverse_render_root,
        pva_render_config=pva.render_config,
        strict_counts=not args.allow_count_drift,
        verify_glb_hashes=not args.skip_glb_hash_verification,
        one_shot_only=args.artiverse_one_shot_only,
    )
    for input_root in (pva.root, artiverse.root):
        if output_dir == input_root or output_dir.is_relative_to(input_root) or input_root.is_relative_to(output_dir):
            raise ValueError(f"output directory overlaps an input root: {input_root}")
    stable_contract = build_stable_run_contract(pva, artiverse, args)
    validate_output_contract(output_dir, stable_contract)
    output_dir.mkdir(parents=True, exist_ok=True)
    contract_path = output_dir / "run_contract.json"
    _atomic_write_json(contract_path, stable_contract)

    # The category metadata is encoder-independent.  A synthetic unit matrix is
    # sufficient to derive it without selecting features from either encoder.
    _, _, category_records = aggregate_artiverse_features(
        np.eye(len(artiverse.records), dtype=np.float32), artiverse
    )
    colors = _category_colors(len(category_records))
    top_artifacts = _write_input_indices(output_dir, artiverse, category_records, colors)

    art_full_features: dict[str, np.ndarray] = {}
    art_one_shot_features: dict[str, np.ndarray] = {}
    joint_coordinates: dict[str, dict[str, np.ndarray]] = {}
    encoder_results: dict[str, Any] = {}
    feature_dimensions: dict[str, int] = {}
    encoder_labels: dict[str, str] = {}
    artifact_paths: list[Path] = [contract_path, *[output_dir / value for value in top_artifacts.values()]]

    for encoder in ENCODERS:
        print(
            f"[run] extracting/loading Artiverse {encoder} "
            f"{'one-shot' if artiverse.one_shot_only else 'model'} features",
            flush=True,
        )
        model_features, extraction, reused = load_or_extract_artiverse_features(
            artiverse,
            encoder=encoder,
            pva_feature_manifest=pva.feature_manifests[encoder],
            output_dir=output_dir,
            batch_size=args.batch_size,
            device=args.device,
            num_workers=args.num_workers,
            use_amp=args.use_amp,
            force_extract=args.force_extract,
        )
        full_features, one_shot_features, observed_categories = aggregate_artiverse_features(
            model_features, artiverse
        )
        if observed_categories != category_records:
            raise AssertionError("Artiverse category metadata changed across encoders")
        encoder_dir = output_dir / encoder
        full_path = encoder_dir / "artiverse_category_features.npy"
        one_shot_path = encoder_dir / "artiverse_one_shot_features.npy"
        _atomic_save_npy(one_shot_path, one_shot_features)
        model_path = encoder_dir / "artiverse_model_features.npy"
        if not artiverse.one_shot_only:
            _atomic_save_npy(full_path, full_features)
            art_full_features[encoder] = full_features
        art_one_shot_features[encoder] = one_shot_features
        feature_dimensions[encoder] = model_features.shape[1]
        label = str(extraction.get("encoder_label") or ("DINOv2" if encoder == "dinov2" else "CLIP"))
        encoder_labels[encoder] = label
        feature_artifacts = (
            [one_shot_path]
            if artiverse.one_shot_only
            else [model_path, full_path, one_shot_path]
        )
        artifact_paths.extend([*feature_artifacts, encoder_dir / "feature_manifest.json"])

        independent_specs = (
            (("one_shot", one_shot_features, [record.category for record in category_records]),)
            if artiverse.one_shot_only
            else (
                ("model", model_features, [record.category for record in artiverse.records]),
                ("category", full_features, [record.category for record in category_records]),
                ("one_shot", one_shot_features, [record.category for record in category_records]),
            )
        )
        independent_results: dict[str, Any] = {}
        for representation, features, labels in independent_specs:
            print(f"[tsne] {encoder} Artiverse {representation} n={len(features)}", flush=True)
            coordinates, tsne_info = _compute_tsne(
                features,
                perplexity=args.perplexity,
                random_state=args.seed,
                max_iter=args.tsne_max_iter,
                n_jobs=args.tsne_jobs,
                verbose=args.tsne_verbose,
            )
            coordinate_path = encoder_dir / f"artiverse_{representation}_tsne_coordinates.csv"
            if representation == "model":
                rows = _model_coordinate_rows(coordinates, artiverse.records)
            else:
                rows = _category_coordinate_rows(
                    coordinates, category_records,
                    representation="full" if representation == "category" else "one_shot",
                )
            _atomic_write_csv(coordinate_path, list(rows[0]), rows)
            plot_path = encoder_dir / f"artiverse_{representation}_tsne.png"
            _save_artiverse_plot(
                coordinates, labels, artiverse.categories, colors, plot_path,
                title=f"Artiverse uniform {label} t-SNE: {representation} (n={len(features):,})",
                annotate=representation != "model", dpi=args.plot_dpi,
            )
            artifact_paths.extend([coordinate_path, plot_path])
            independent_results[representation] = {
                "tsne": tsne_info,
                "coordinates": str(coordinate_path.relative_to(output_dir)),
                "plot": str(plot_path.relative_to(output_dir)),
            }

        joint_coordinates[encoder] = {}
        joint_results: dict[str, Any] = {}
        joint_specs = (
            (("one_shot", one_shot_features),)
            if artiverse.one_shot_only
            else (("full", full_features), ("one_shot", one_shot_features))
        )
        for representation, art_features in joint_specs:
            joint_features = np.vstack([pva.features[encoder], art_features])
            print(f"[tsne] {encoder} joint {representation} n={len(joint_features)}", flush=True)
            coordinates, tsne_info = _compute_tsne(
                joint_features,
                perplexity=args.perplexity,
                random_state=args.seed,
                max_iter=args.tsne_max_iter,
                n_jobs=args.tsne_jobs,
                verbose=args.tsne_verbose,
            )
            joint_coordinates[encoder][representation] = coordinates
            coordinate_path = encoder_dir / f"joint_{representation}_tsne_coordinates.csv"
            rows = _joint_coordinate_rows(
                coordinates, pva.records, category_records, representation=representation
            )
            _atomic_write_csv(coordinate_path, list(rows[0]), rows)
            plot_path = encoder_dir / f"joint_{representation}_tsne.png"
            _save_joint_plot(
                coordinates, len(pva.records), plot_path,
                encoder_label=label, representation=representation, dpi=args.plot_dpi,
            )
            artifact_paths.extend([coordinate_path, plot_path])
            joint_results[representation] = {
                "tsne": tsne_info,
                "coordinates": str(coordinate_path.relative_to(output_dir)),
                "plot": str(plot_path.relative_to(output_dir)),
            }
        encoder_results[encoder] = {
            "label": label,
            "feature_dim": model_features.shape[1],
            "feature_cache_reused": reused,
            "independent": independent_results,
            "joint": joint_results,
        }

    if artiverse.one_shot_only:
        metrics = compute_one_shot_high_dimensional_metrics(
            pva.features, art_one_shot_features, joint_coordinates,
            neighbor_fraction=args.neighbor_fraction,
        )
    else:
        metrics = compute_high_dimensional_metrics(
            pva.features, art_full_features, art_one_shot_features, joint_coordinates,
            neighbor_fraction=args.neighbor_fraction,
        )
    metrics_path = output_dir / "high_dimensional_metrics.json"
    _atomic_write_json(metrics_path, metrics)
    comparison_path = output_dir / "joint_source_comparison.png"
    _save_joint_comparison(
        joint_coordinates, pva_count=len(pva.records), labels=encoder_labels,
        representations=("one_shot",) if artiverse.one_shot_only else ("full", "one_shot"),
        path=comparison_path, dpi=args.plot_dpi,
    )
    artifact_paths.extend([metrics_path, comparison_path])

    manifest = {
        "schema_version": 1,
        "stable_run_contract": stable_contract,
        "protocol": {
            "render": "identical camera, lights, background, resolution, samples, Blender version, and studio values",
            "pva_sample_unit": "one seed_0000 render per generator class",
            "artiverse_render_unit": (
                "one hash-selected uniform render per category"
                if artiverse.one_shot_only
                else "one uniform render per official model"
            ),
            "artiverse_full_category": (
                None
                if artiverse.one_shot_only
                else "equal-weight mean of normalized model embeddings, then L2 normalization"
            ),
            "artiverse_one_shot": "content-independent representative frozen by render manifest",
            "joint_projection": "concatenate high-dimensional features per encoder, L2, PCA50, then one fitted t-SNE",
            "encoder_spaces_combined": False,
            "artiverse_mode": "one_shot_only" if artiverse.one_shot_only else "full",
        },
        "interpretation_limits": [
            *(
                [
                    "The primary Artiverse result uses one identity-hash-selected representative per category; category support in the 3,544-model universe is metadata only.",
                    "One representative per category avoids support averaging but has representative-selection variance.",
                ]
                if artiverse.one_shot_only
                else [
                    "The full Artiverse category centroids have unequal support (one to many models), while each PV-A generator has one seed.",
                    "The one-shot Artiverse result is a support-matched sensitivity view with higher representative variance.",
                ]
            ),
            "t-SNE axes and global geometry are not comparable across independently fitted panels.",
            "Dataset-source separation can reflect material and asset-domain differences, not semantic diversity alone.",
        ],
        "inputs": {"pva": pva.receipts, "artiverse": artiverse.receipts},
        "dataset": {
            "pva_generator_count": len(pva.records),
            "artiverse_render_samples_used": len(artiverse.records),
            "artiverse_official_universe_model_count": artiverse.universe_model_count,
            "artiverse_category_count": len(category_records),
            "artiverse_models_per_category_min": min(record.model_count for record in category_records),
            "artiverse_models_per_category_max": max(record.model_count for record in category_records),
        },
        "render_contract_match": {
            "pva_contract": pva.render_config["render_contract"],
            "artiverse_contract": artiverse.config["render_contract"],
            "matched_fields": ["resolution", "samples", "studio", "blender_version"],
            "values_equal": True,
        },
        "implementation": {
            "script": _file_receipt(SCRIPT),
            "reused_feature_tsne_script": _file_receipt(PVA_HELPER_SCRIPT),
        },
        "config": {
            "pva_dir": str(pva.root),
            "artiverse_render_root": str(artiverse.root),
            "output_dir": str(output_dir),
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
            "neighbor_fraction": args.neighbor_fraction,
            "force_extract": args.force_extract,
            "skip_glb_hash_verification": args.skip_glb_hash_verification,
            "artiverse_one_shot_only": args.artiverse_one_shot_only,
        },
        "encoders": encoder_results,
        "artifacts": {
            **top_artifacts,
            "run_contract": str(contract_path.relative_to(output_dir)),
            "high_dimensional_metrics": str(metrics_path.relative_to(output_dir)),
            "joint_source_comparison": str(comparison_path.relative_to(output_dir)),
        },
    }
    manifest_path = output_dir / "run_manifest.json"
    _atomic_write_json(manifest_path, manifest)
    artifact_paths.append(manifest_path)
    audit = audit_outputs(
        output_dir,
        pva_count=len(pva.records),
        model_count=len(artiverse.records),
        category_count=len(category_records),
        feature_dimensions=feature_dimensions,
        artifact_paths=artifact_paths,
    )
    audit_path = output_dir / "final_audit.json"
    _atomic_write_json(audit_path, audit)
    if not audit["pass"]:
        failed = [name for name, passed in audit["checks"].items() if not passed]
        raise RuntimeError(f"output audit failed: {failed}")
    return {"manifest": manifest, "audit": audit}


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pva-dir", type=Path, default=DEFAULT_PVA_DIR)
    parser.add_argument(
        "--artiverse-render-root", type=Path, default=DEFAULT_ARTIVERSE_RENDER_ROOT
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--artiverse-one-shot-only",
        action="store_true",
        dest="artiverse_one_shot_only",
        help="Use only the 84 frozen hash-selected category representatives (default).",
    )
    mode.add_argument(
        "--artiverse-full",
        action="store_false",
        dest="artiverse_one_shot_only",
        help="Compatibility mode: require and use all 3,544 uniform renders.",
    )
    parser.set_defaults(artiverse_one_shot_only=True)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--num-workers", type=int, default=min(16, os.cpu_count() or 1))
    parser.add_argument("--no-amp", action="store_false", dest="use_amp")
    parser.set_defaults(use_amp=True)
    parser.add_argument("--perplexity", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--tsne-max-iter", type=int, default=1_000)
    parser.add_argument("--tsne-jobs", type=int, default=min(64, os.cpu_count() or 1))
    parser.add_argument("--tsne-verbose", type=int, choices=(0, 1, 2), default=1)
    parser.add_argument("--plot-dpi", type=int, default=180)
    parser.add_argument("--neighbor-fraction", type=float, default=0.06)
    parser.add_argument("--force-extract", action="store_true")
    parser.add_argument(
        "--skip-glb-hash-verification", action="store_true",
        help="Trust per-GLB render receipts after checking sizes; PNG hashes are always verified.",
    )
    parser.add_argument(
        "--allow-count-drift", action="store_true",
        help="Allow compact synthetic rosters for tests; all other contracts remain strict.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        result = run(build_argument_parser().parse_args(argv))
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
