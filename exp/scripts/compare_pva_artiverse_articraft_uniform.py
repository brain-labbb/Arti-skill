#!/usr/bin/env python3
"""Compare PV-A, Artiverse, and Articraft-10K under one Blender studio.

The comparison is deliberately class-level.  PV-A contributes one frozen
seed-0000 image for each of its 531 generator classes, Artiverse contributes
one frozen representative for each of its 84 categories, and Articraft-10K
contributes one identity-hash-selected representative for each of the 244
categories in the complete 10,787-row official roster.  The feature spaces
are kept separate for DINOv2 and CLIP; each joint plot is fitted once from
the corresponding high-dimensional matrices (L2 -> PCA50 -> t-SNE).

This file is an analysis driver.  It reuses the audited PV-A/Artiverse loader,
model snapshots, feature extractor, and t-SNE implementation instead of
copying those contracts.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from PIL import Image, ImageStat

for _thread_variable in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_thread_variable, "1")


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
BASE_COMPARE_SCRIPT = REPO_ROOT / "exp/scripts/compare_pva_artiverse_uniform.py"
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
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exp/runtime/pva_artiverse_articraft_uniform_comparison"
ENCODERS = ("dinov2", "clip")
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


@dataclass(frozen=True, slots=True)
class ArticraftRecord:
    ordinal: int
    asset_id: str
    category: str
    cohort_origin: str
    package_path: Path
    image_path: Path
    image_bytes: int
    image_sha256: str
    category_one_shot: bool


@dataclass(frozen=True, slots=True)
class ArticraftBundle:
    root: Path
    records: tuple[ArticraftRecord, ...]
    categories: tuple[str, ...]
    config: dict[str, Any]
    render_manifest_path: Path
    roster_path: Path
    official_roster_path: Path
    official_model_count: int
    official_category_count: int
    receipts: dict[str, Any]


_BASE: Any | None = None


def _base() -> Any:
    global _BASE
    if _BASE is None:
        name = "_pva_artiverse_uniform_base_for_three_source"
        spec = importlib.util.spec_from_file_location(name, BASE_COMPARE_SCRIPT)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot load comparison helper: {BASE_COMPARE_SCRIPT}")
        module = importlib.util.module_from_spec(spec)
        # Make the dynamically loaded module visible during execution;
        # dataclasses resolves the class module through sys.modules.
        sys.modules[name] = module
        spec.loader.exec_module(module)
        _BASE = module
    return _BASE


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_receipt(path: Path) -> dict[str, Any]:
    path = path.resolve(strict=True)
    return {"path": str(path), "sha256": sha256_file(path), "size_bytes": path.stat().st_size}


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
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
    fields = list(rows[0].keys())
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def atomic_npy(path: Path, matrix: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("wb") as stream:
        np.save(stream, np.asarray(matrix), allow_pickle=False)
    temporary.replace(path)


def _resolve_path(value: Any, *, parent: Path, field: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"missing path field {field}")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = parent / path
    return path.resolve(strict=True)


def _inside(root: Path, path: Path, *, field: str) -> Path:
    root = root.resolve(strict=True)
    path = path.expanduser().resolve(strict=True)
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{field} escapes {root}: {path}") from exc
    return path


def _canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    ).hexdigest()


def _read_official_rows(path: Path) -> list[dict[str, Any]]:
    """Read either the merged manifest JSON or its JSONL roster."""
    if path.suffix.lower() == ".jsonl":
        rows: list[dict[str, Any]] = []
        with path.open(encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise ValueError(f"invalid roster row {line_number}: {path}")
                rows.append(value)
        return rows
    value = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(value, dict) and isinstance(value.get("rows"), list):
        rows = value["rows"]
    elif isinstance(value, list):
        rows = value
    else:
        raise ValueError(f"official roster has no rows: {path}")
    if not all(isinstance(row, dict) for row in rows):
        raise ValueError(f"official roster contains a non-object row: {path}")
    return [dict(row) for row in rows]


def _selection_key(row: Mapping[str, Any]) -> tuple[str, str]:
    identity = str(row.get("asset_id") or row.get("record_id") or row.get("model_id") or "").strip()
    if not identity:
        raise ValueError("official Articraft row has no asset identity")
    # The identity itself is the immutable, content-independent tie-breaker.
    return hashlib.sha256(identity.encode("utf-8")).hexdigest(), identity


def _parse_bool(value: Any, *, field: str) -> bool:
    normalized = str(value).strip().lower()
    if normalized in {"true", "1"}:
        return True
    if normalized in {"false", "0"}:
        return False
    raise ValueError(f"invalid boolean {field}: {value!r}")


def _row_image_path(root: Path, row: Mapping[str, Any]) -> Path:
    value = row.get("image_path") or row.get("output_path")
    if not isinstance(value, str) or not value.strip():
        # The renderer's stable layout is category/asset_id/imgs/000.png.
        value = str(Path(str(row["category"])) / str(row["asset_id"]) / "imgs/000.png")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = root / path
    return _inside(root, path, field="Articraft image_path")


def load_articraft_uniform(
    root: Path,
    *,
    pva_render_config: Mapping[str, Any],
    strict_counts: bool = True,
) -> ArticraftBundle:
    root = root.expanduser().resolve(strict=True)
    config_path = root / "render_config.json"
    manifest_path = root / "render_manifest.csv"
    roster_path = root / "category_one_shot_roster.csv"
    for path in (config_path, manifest_path, roster_path):
        if not path.is_file():
            raise FileNotFoundError(f"incomplete Articraft uniform root: {path}")
    config = load_json(config_path)
    contract = str(config.get("render_contract", ""))
    if not contract.startswith("articraft") or "uniform" not in contract:
        raise ValueError(f"unexpected Articraft render contract: {contract!r}")
    for field in ("resolution", "samples", "studio", "blender_version"):
        if config.get(field) != pva_render_config.get(field):
            raise ValueError(f"Articraft/PV-A render contract mismatch for {field}")
    if strict_counts and (
        int(config.get("model_count", -1)) != 10_787
        or int(config.get("category_count", -1)) != 244
    ):
        raise ValueError("Articraft render config does not describe the complete merged roster")
    if strict_counts and (
        int(config.get("selected_count", -1)) != 244
        or int(config.get("selected_category_count", -1)) != 244
    ):
        raise ValueError("Articraft render config is not the complete one-shot category selection")
    selection = config.get("selection")
    if not isinstance(selection, dict) or selection.get("one_shot_only") is not True:
        raise ValueError("Articraft render config is not in one-shot mode")
    for path_field, hash_field in (
        ("renderer", "renderer_sha256"),
        ("shared_renderer", "shared_renderer_sha256"),
    ):
        implementation_path = _resolve_path(
            config.get(path_field), parent=config_path.parent, field=path_field
        )
        if config.get(hash_field) != sha256_file(implementation_path):
            raise ValueError(f"Articraft {path_field} file no longer matches its frozen SHA")
    if config.get("shared_renderer_sha256") != pva_render_config.get("renderer_sha256"):
        raise ValueError("Articraft did not use the frozen PV-A shared studio renderer")

    official_value = config.get("official_roster") or config.get("dataset_manifest") or config.get("roster")
    official_path = _resolve_path(official_value, parent=config_path.parent, field="official_roster")
    declared_hash = config.get("official_roster_sha256") or config.get("dataset_manifest_sha256")
    if declared_hash and declared_hash != sha256_file(official_path):
        raise ValueError("Articraft official roster SHA mismatch")
    official_rows = _read_official_rows(official_path)
    if not official_rows:
        raise ValueError("Articraft official roster is empty")
    official_categories = {str(row.get("category") or row.get("category_slug") or "").strip() for row in official_rows}
    official_categories.discard("")
    if strict_counts and len(official_rows) != 10_787:
        raise ValueError(f"expected 10,787 official Articraft rows, found {len(official_rows)}")
    if strict_counts and len(official_categories) != 244:
        raise ValueError(f"expected 244 official Articraft categories, found {len(official_categories)}")

    winners: dict[str, Mapping[str, Any]] = {}
    for row in official_rows:
        category = str(row.get("category") or row.get("category_slug") or "").strip()
        if not category:
            raise ValueError("official Articraft row has no category")
        previous = winners.get(category)
        if previous is None or _selection_key(row) < _selection_key(previous):
            winners[category] = row

    with manifest_path.open("r", encoding="utf-8", newline="") as stream:
        rendered_rows = list(csv.DictReader(stream))
    if not rendered_rows:
        raise ValueError("Articraft render manifest is empty")
    if strict_counts and len(rendered_rows) != 244:
        raise ValueError(f"expected 244 Articraft render receipts, found {len(rendered_rows)}")
    by_identity: dict[str, dict[str, Any]] = {}
    for row in rendered_rows:
        identity = str(row.get("asset_id") or row.get("record_id") or "").strip()
        if not identity or identity in by_identity:
            raise ValueError(f"duplicate/empty Articraft render identity: {identity!r}")
        by_identity[identity] = row

    selected: list[ArticraftRecord] = []
    for category in sorted(winners):
        official = winners[category]
        identity = str(official.get("asset_id") or official.get("record_id") or "").strip()
        row = by_identity.get(identity)
        if row is None:
            raise ValueError(f"missing rendered winner for Articraft category {category}: {identity}")
        official_ordinal = int(official.get("ordinal", -1))
        row_ordinal = int(row.get("ordinal", -1))
        expected_identity_sha = _selection_key(official)[0]
        official_origin = str(official.get("cohort_origin") or "").strip()
        binding_checks = {
            "category": (str(row.get("category") or "").strip(), category),
            "ordinal": (row_ordinal, official_ordinal),
            "cohort_origin": (str(row.get("cohort_origin") or "").strip(), official_origin),
            "identity_sha256": (
                str(row.get("identity_sha256") or "").strip(),
                expected_identity_sha,
            ),
            "urdf_sha256": (
                str(row.get("urdf_sha256") or "").strip(),
                str(official.get("primary_urdf_sha256") or "").strip(),
            ),
            "urdf_bytes": (
                int(row.get("urdf_bytes", -1)),
                int(official.get("primary_urdf_size", official.get("primary_urdf_bytes", -1))),
            ),
            "package_binding_sha256": (
                str(row.get("package_binding_sha256") or "").strip(),
                str(official.get("package_binding_sha256") or "").strip(),
            ),
        }
        for field, (actual, expected) in binding_checks.items():
            if actual != expected:
                raise ValueError(
                    f"Articraft winner {field} differs from the official roster: {identity}"
                )
        if not _parse_bool(row.get("category_one_shot"), field="category_one_shot"):
            raise ValueError(f"Articraft winner is not marked category_one_shot: {identity}")
        official_source = _resolve_path(
            official.get("source_path"), parent=official_path.parent, field="official source_path"
        )
        row_source = _resolve_path(
            row.get("source_path"), parent=config_path.parent, field="render source_path"
        )
        if row_source != official_source:
            raise ValueError(f"Articraft winner source_path differs from the official roster: {identity}")
        official_urdf = _resolve_path(
            official.get("primary_urdf_path"),
            parent=official_path.parent,
            field="official primary_urdf_path",
        )
        row_urdf = _resolve_path(
            row.get("urdf_path"), parent=config_path.parent, field="render urdf_path"
        )
        if row_urdf != official_urdf:
            raise ValueError(f"Articraft winner urdf_path differs from the official roster: {identity}")
        status = str(row.get("status", "")).strip()
        if status not in {"rendered", "reused_valid", "success"}:
            raise ValueError(f"Articraft winner has non-success status: {category}/{status}")
        image_path = _row_image_path(root, row)
        expected_image_path = _inside(
            root,
            root / category / identity / "imgs" / "000.png",
            field="expected Articraft image_path",
        )
        if image_path != expected_image_path:
            raise ValueError(f"Articraft winner output path differs from the stable layout: {identity}")
        if not image_path.is_file() or image_path.read_bytes()[:8] != PNG_SIGNATURE:
            raise ValueError(f"missing/invalid Articraft PNG: {image_path}")
        with Image.open(image_path) as image:
            image.load()
            if image.size != (int(config["resolution"]), int(config["resolution"])):
                raise ValueError(f"Articraft PNG dimensions differ from the render contract: {image_path}")
            if image.mode not in {"RGB", "RGBA"}:
                raise ValueError(f"Articraft PNG has an unexpected color mode: {image_path}")
        actual_bytes = image_path.stat().st_size
        actual_sha = sha256_file(image_path)
        declared_bytes = int(row.get("png_bytes") or row.get("image_bytes") or actual_bytes)
        declared_sha = str(row.get("png_sha256") or row.get("image_sha256") or actual_sha)
        if declared_bytes != actual_bytes or declared_sha != actual_sha:
            raise ValueError(f"Articraft PNG receipt mismatch: {image_path}")
        selected.append(
            ArticraftRecord(
                ordinal=row_ordinal,
                asset_id=identity,
                category=category,
                cohort_origin=official_origin,
                package_path=official_source,
                image_path=image_path,
                image_bytes=actual_bytes,
                image_sha256=actual_sha,
                category_one_shot=True,
            )
        )
    if strict_counts and len(selected) != 244:
        raise ValueError(f"expected 244 Articraft selected rows, found {len(selected)}")

    # The compact one-shot roster is an auditable second source of the exact
    # selected identities.  It is intentionally checked against the official
    # hash winners rather than trusted as the selector.
    with roster_path.open("r", encoding="utf-8", newline="") as stream:
        compact = list(csv.DictReader(stream))
    compact_ids = [
        str(row.get("asset_id") or row.get("record_id") or row.get("one_shot_asset_id") or "").strip()
        for row in compact
    ]
    selected_ids = [record.asset_id for record in selected]
    if compact_ids != selected_ids:
        raise ValueError("Articraft one-shot roster order does not match official identity-hash winners")

    ordered_receipt = hashlib.sha256(
        "\n".join(f"{r.category}\0{r.asset_id}\0{r.image_bytes}\0{r.image_sha256}" for r in selected).encode("utf-8")
    ).hexdigest()
    return ArticraftBundle(
        root=root,
        records=tuple(selected),
        categories=tuple(record.category for record in selected),
        config=config,
        render_manifest_path=manifest_path,
        roster_path=roster_path,
        official_roster_path=official_path,
        official_model_count=len(official_rows),
        official_category_count=len(official_categories),
        receipts={
            "render_config": file_receipt(config_path),
            "render_manifest": file_receipt(manifest_path),
            "category_one_shot_roster": file_receipt(roster_path),
            "official_roster": file_receipt(official_path),
            "ordered_image_receipt_sha256": ordered_receipt,
        },
    )


def l2_normalize(features: np.ndarray) -> np.ndarray:
    return _base().l2_normalize(features)


def _compute_tsne(
    features: np.ndarray,
    *,
    perplexity: float,
    random_state: int,
    max_iter: int,
    n_jobs: int,
    verbose: int,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Use the audited PCA-then-t-SNE implementation from the base driver."""

    helper = _base()._pva_helper()
    return helper.compute_tsne(
        features,
        perplexity=perplexity,
        random_state=random_state,
        max_iter=max_iter,
        n_jobs=n_jobs,
        verbose=verbose,
    )


def _load_matrix(path: Path, *, rows: int, dimension: int | None = None) -> np.ndarray:
    matrix = np.load(path, allow_pickle=False)
    if matrix.ndim != 2 or matrix.shape[0] != rows or matrix.dtype != np.float32:
        raise ValueError(f"unexpected feature matrix {path}: {matrix.shape}/{matrix.dtype}")
    if dimension is not None and matrix.shape[1] != dimension:
        raise ValueError(f"feature dimension mismatch at {path}")
    if not np.isfinite(matrix).all():
        raise ValueError(f"non-finite features: {path}")
    if not np.allclose(np.linalg.norm(matrix, axis=1), 1.0, rtol=2e-5, atol=2e-5):
        raise ValueError(f"features are not L2 normalized: {path}")
    return np.asarray(matrix, dtype=np.float32)


def _load_artiverse_cached(
    comparison_dir: Path,
    *,
    pva: Any,
    artiverse: Any,
) -> dict[str, np.ndarray]:
    result: dict[str, np.ndarray] = {}
    base = _base()
    ordered_image_fingerprint = base._ordered_image_fingerprint(artiverse.records)
    audit_path = comparison_dir / "final_audit.json"
    audit = load_json(audit_path)
    audit_artifacts = audit.get("artifacts")
    if audit.get("pass") is not True or not isinstance(audit_artifacts, dict):
        raise ValueError("Artiverse cached comparison does not have a passing artifact audit")
    for encoder in ENCODERS:
        feature_path = comparison_dir / encoder / "artiverse_one_shot_features.npy"
        manifest_path = comparison_dir / encoder / "feature_manifest.json"
        if not feature_path.is_file() or not manifest_path.is_file():
            raise FileNotFoundError(f"missing Artiverse cached features: {feature_path}")
        manifest = load_json(manifest_path)
        extraction = manifest.get("extraction")
        expected_preprocessing = {
            "image_processor_use_fast": False,
            "alpha_composite_background_rgb": [255, 255, 255],
        }
        dimension = int(pva.features[encoder].shape[1])
        if (
            manifest.get("schema_version") != 1
            or manifest.get("encoder") != encoder
            or manifest.get("model_type") != encoder
            or manifest.get("artiverse_sample_mode") != "one_shot_only"
            or manifest.get("preprocessing") != expected_preprocessing
            or not isinstance(extraction, dict)
            or extraction.get("model_type") != encoder
            or extraction.get("feature_source")
            != pva.feature_manifests[encoder]["extraction"]["feature_source"]
            or int(extraction.get("feature_dim", -1)) != dimension
        ):
            raise ValueError(f"Artiverse {encoder} cache extraction contract mismatch")
        expected_fp = pva.feature_manifests[encoder].get("model_fingerprint")
        if manifest.get("model_fingerprint") != expected_fp:
            raise ValueError(f"Artiverse {encoder} cache uses a different model snapshot")
        if int(manifest.get("model_count", -1)) != len(artiverse.records):
            raise ValueError(f"Artiverse {encoder} cache count differs from the render roster")
        if manifest.get("artiverse_render_manifest_sha256") != artiverse.receipts["render_manifest"]["sha256"]:
            raise ValueError(f"Artiverse {encoder} cache uses a different render manifest")
        if manifest.get("ordered_image_fingerprint") != ordered_image_fingerprint:
            raise ValueError(f"Artiverse {encoder} cache uses a different ordered image roster")
        for relative, path in (
            (f"{encoder}/artiverse_one_shot_features.npy", feature_path),
            (f"{encoder}/feature_manifest.json", manifest_path),
        ):
            receipt = audit_artifacts.get(relative)
            if not isinstance(receipt, dict) or receipt.get("sha256") != sha256_file(path):
                raise ValueError(f"Artiverse audited artifact receipt mismatch: {relative}")
        result[encoder] = _load_matrix(
            feature_path,
            rows=len(artiverse.records),
            dimension=dimension,
        )
    return result


def _extract_articraft(
    bundle: ArticraftBundle,
    *,
    pva: Any,
    output_dir: Path,
    batch_size: int,
    device: str,
    num_workers: int,
    use_amp: bool,
    force_extract: bool,
) -> dict[str, np.ndarray]:
    base = _base()
    helper = base._pva_helper()
    helper_path = Path(base.PVA_HELPER_SCRIPT).resolve(strict=True)
    import torch

    resolved_device = device
    if device == "auto":
        resolved_device = "cuda:0" if torch.cuda.is_available() else "cpu"
    torch_device = torch.device(resolved_device)
    if torch_device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(f"CUDA device requested but unavailable: {resolved_device}")
    execution_policy = {
        "requested_device": device,
        "resolved_device": str(torch_device),
        "requested_use_amp": bool(use_amp),
        "amp_enabled": bool(use_amp and torch_device.type == "cuda"),
        "batch_size": batch_size,
        "num_workers": num_workers,
    }
    features: dict[str, np.ndarray] = {}
    for encoder in ENCODERS:
        enc_dir = output_dir / encoder
        enc_dir.mkdir(parents=True, exist_ok=True)
        feature_path = enc_dir / "articraft_one_shot_features.npy"
        manifest_path = enc_dir / "articraft_feature_manifest.json"
        model_manifest = pva.feature_manifests[encoder]
        model_path = Path(str(model_manifest["model_path"])).resolve(strict=True)
        expected_fp = model_manifest["model_fingerprint"]
        ordered_fingerprint = hashlib.sha256(
            "\n".join(f"{r.category}\0{r.asset_id}\0{r.image_bytes}\0{r.image_sha256}" for r in bundle.records).encode("utf-8")
        ).hexdigest()
        expected = {
            "schema_version": 1,
            "encoder": encoder,
            "model_type": encoder,
            "model_path": str(model_path),
            "model_fingerprint": expected_fp,
            "model_count": len(bundle.records),
            "ordered_image_fingerprint": ordered_fingerprint,
            "render_manifest_sha256": bundle.receipts["render_manifest"]["sha256"],
            "feature_helper_path": str(helper_path),
            "feature_helper_sha256": sha256_file(helper_path),
            "execution_policy": execution_policy,
            "preprocessing": {
                "image_processor_use_fast": False,
                "alpha_composite_background_rgb": [255, 255, 255],
            },
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
            image_paths = tuple(record.image_path for record in bundle.records)
            matrix, extraction = helper.extract_image_features(
                image_paths,
                model_path=model_path,
                batch_size=batch_size,
                device=device,
                num_workers=num_workers,
                use_amp=use_amp,
            )
            matrix = l2_normalize(matrix).astype(np.float32, copy=False)
            if extraction.get("model_type") != encoder:
                raise ValueError(f"extracted {encoder} model type mismatch")
            if extraction.get("device") != execution_policy["resolved_device"]:
                raise ValueError(f"extracted {encoder} device differs from the requested policy")
            if bool(extraction.get("amp")) != execution_policy["amp_enabled"]:
                raise ValueError(f"extracted {encoder} AMP mode differs from the requested policy")
            if matrix.shape[1] != pva.features[encoder].shape[1]:
                raise ValueError(f"extracted {encoder} dimension mismatch")
            atomic_npy(feature_path, matrix)
            atomic_json(manifest_path, {**expected, "dtype": str(matrix.dtype), "extraction": extraction})
        features[encoder] = matrix
    return features


def category_colors(count: int) -> tuple[str, ...]:
    import colorsys

    values: list[str] = []
    for index in range(count):
        hue = (index * 0.6180339887498949) % 1.0
        saturation = 0.56 + 0.20 * ((index % 4) / 3.0)
        value = 0.70 + 0.22 * (((index // 4) % 3) / 2.0)
        rgb = colorsys.hsv_to_rgb(hue, saturation, value)
        values.append("#%02x%02x%02x" % tuple(round(channel * 255) for channel in rgb))
    if len(set(values)) != count:
        raise AssertionError("category palette is not unique")
    return tuple(values)


def save_articraft_plot(
    coordinates: np.ndarray,
    categories: Sequence[str],
    colors: Sequence[str],
    path: Path,
    *,
    title: str,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(16, 12), dpi=dpi)
    axis.scatter(
        coordinates[:, 0], coordinates[:, 1], s=46, c=list(colors), alpha=0.84,
        edgecolors="white", linewidths=0.35, rasterized=True,
    )
    for index, (point, _category) in enumerate(zip(coordinates, categories, strict=True), start=1):
        axis.annotate(f"C{index:03d}", point, xytext=(3, 2), textcoords="offset points", fontsize=4.2)
    axis.set_title(title)
    axis.set_xlabel("t-SNE 1")
    axis.set_ylabel("t-SNE 2")
    axis.grid(color="#d8d8d8", linewidth=0.4, alpha=0.55)
    axis.spines[["top", "right"]].set_visible(False)
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def save_three_source_plot(
    coordinates: Mapping[str, np.ndarray],
    counts: Mapping[str, int],
    path: Path,
    *,
    labels: Mapping[str, str],
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    specs = {
        "PV-A": ("#d1495b", "o", 18, 0.64),
        "Artiverse": ("#2878b5", "^", 40, 0.86),
        "Articraft-10K": ("#2a9d5b", "s", 35, 0.82),
    }
    figure, axes = plt.subplots(1, 2, figsize=(18, 9), dpi=dpi, squeeze=False)
    for axis, encoder in zip(axes[0], ENCODERS, strict=True):
        start = 0
        for source in ("PV-A", "Artiverse", "Articraft-10K"):
            end = start + counts[source]
            color, marker, size, alpha = specs[source]
            points = coordinates[encoder][start:end]
            axis.scatter(
                points[:, 0], points[:, 1], s=size, color=color, marker=marker,
                alpha=alpha, edgecolors="white" if source != "PV-A" else "none",
                linewidths=0.3, label=f"{labels[source]} (n={counts[source]})", rasterized=True,
            )
            start = end
        axis.set_title(f"{encoder.upper() if encoder == 'clip' else 'DINOv2'} joint t-SNE")
        axis.set_xlabel("joint t-SNE 1")
        axis.set_ylabel("joint t-SNE 2")
        axis.grid(color="#d8d8d8", linewidth=0.4, alpha=0.52)
        axis.spines[["top", "right"]].set_visible(False)
        axis.legend(frameon=False, loc="best")
    figure.suptitle("Uniform Blender comparison: PV-A, Artiverse, and Articraft-10K", fontsize=18)
    figure.text(0.5, 0.012, "Each encoder is fitted jointly in its high-dimensional feature space; axes are not cross-encoder coordinates.", ha="center", fontsize=10, color="#444444")
    figure.tight_layout(rect=(0.0, 0.035, 1.0, 0.95))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def write_coordinates(
    path: Path,
    coordinates: np.ndarray,
    records: Sequence[ArticraftRecord],
) -> None:
    rows = [
        {
            "tsne_x": format(float(point[0]), ".9g"),
            "tsne_y": format(float(point[1]), ".9g"),
            "category_id": f"C{index:03d}",
            "category": record.category,
            "asset_id": record.asset_id,
            "cohort_origin": record.cohort_origin,
            "image_path": str(record.image_path),
        }
        for index, (point, record) in enumerate(zip(coordinates, records, strict=True), start=1)
    ]
    atomic_csv(path, rows)


def write_joint_coordinates(
    path: Path,
    coordinates: np.ndarray,
    *,
    pva_records: Sequence[Any],
    artiverse_records: Sequence[Any],
    articraft_records: Sequence[ArticraftRecord],
) -> None:
    rows: list[dict[str, Any]] = []
    source_specs = (
        ("PV-A", pva_records),
        ("Artiverse", artiverse_records),
        ("Articraft-10K", articraft_records),
    )
    offset = 0
    for source, records in source_specs:
        source_coordinates = coordinates[offset : offset + len(records)]
        for local, (point, record) in enumerate(zip(source_coordinates, records, strict=True)):
            if source == "PV-A":
                class_id = record.generator_index
                class_name = record.generator_name
                origin = record.source_type
            elif source == "Artiverse":
                class_id = f"C{local + 1:02d}"
                class_name = record.category
                origin = getattr(record, "one_shot_source", "") or getattr(record, "source", "")
            else:
                class_id = f"C{local + 1:03d}"
                class_name = record.category
                origin = record.cohort_origin
            rows.append({
                "tsne_x": format(float(point[0]), ".9g"),
                "tsne_y": format(float(point[1]), ".9g"),
                "joint_index": offset + local,
                "dataset": source,
                "class_id": class_id,
                "class_name": class_name,
                "source": origin,
            })
        offset += len(records)
    atomic_csv(path, rows)


def multi_source_metrics(features: Mapping[str, np.ndarray], names: Sequence[str], *, neighbor_fraction: float) -> dict[str, Any]:
    base = _base()
    normalized = [l2_normalize(features[name]) for name in names]
    combined = np.vstack(normalized)
    distances = base._cosine_distances(combined)
    from sklearn.metrics import silhouette_samples

    labels = np.concatenate([np.full(len(features[name]), index, dtype=np.int8) for index, name in enumerate(names)])
    silhouettes = silhouette_samples(combined, labels, metric="cosine")
    k = max(1, min(len(combined) - 2, round(neighbor_fraction * (len(combined) - 1))))
    source_ranges: dict[str, tuple[int, int]] = {}
    cursor = 0
    for name in names:
        source_ranges[name] = (cursor, cursor + len(features[name]))
        cursor += len(features[name])
    mixing: dict[str, Any] = {}
    for name in names:
        start, end = source_ranges[name]
        fractions: dict[str, list[float]] = {other: [] for other in names}
        for query in range(start, end):
            row = distances[query].copy()
            row[query] = np.inf
            neighbors = np.argsort(row, kind="stable")[:k]
            for other in names:
                os_, oe = source_ranges[other]
                fractions[other].append(float(np.mean((neighbors >= os_) & (neighbors < oe))))
        expected = {
            other: (len(features[other]) - (1 if other == name else 0)) / (len(combined) - 1)
            for other in names
        }
        mixing[name] = {
            "k": k,
            "mean_neighbor_source_fraction": {other: float(np.mean(values)) for other, values in fractions.items()},
            "random_expectation": expected,
        }
    return {
        "counts": {name: len(features[name]) for name in names},
        "source_silhouette_cosine": {
            name: float(np.mean(silhouettes[source_ranges[name][0]:source_ranges[name][1]])) for name in names
        },
        "knn_source_mixing": mixing,
        "k": k,
    }


def named_cross_dataset_metrics(
    first_features: np.ndarray,
    second_features: np.ndarray,
    *,
    first_name: str,
    second_name: str,
    neighbor_fraction: float,
) -> dict[str, Any]:
    """Relabel the two-source base metrics with the actual dataset names."""

    raw = _base().cross_dataset_metrics(
        first_features,
        second_features,
        neighbor_fraction=neighbor_fraction,
    )
    nearest = raw["nearest_other_source_cosine_distance"]
    mutual = raw["mutual_cross_source_nearest_neighbors"]
    silhouette = raw["source_silhouette_cosine"]
    mixing = raw["knn_source_mixing"]
    return {
        "source_counts": {
            first_name: raw["pva_count"],
            second_name: raw["artiverse_count"],
        },
        "nearest_other_source_cosine_distance": {
            f"{first_name}_to_{second_name}": nearest["pva_to_artiverse"],
            f"{second_name}_to_{first_name}": nearest["artiverse_to_pva"],
        },
        "mutual_cross_source_nearest_neighbors": {
            "pair_count": mutual["pair_count"],
            f"rate_over_{first_name}": mutual["rate_over_pva"],
            f"rate_over_{second_name}": mutual["rate_over_artiverse"],
        },
        "source_silhouette_cosine": {
            first_name: silhouette["pva_mean"],
            second_name: silhouette["artiverse_mean"],
            "source_macro_mean": silhouette["source_macro_mean"],
            "interpretation": silhouette["interpretation"],
        },
        "knn_source_mixing": {
            "fraction": mixing["fraction"],
            "k": mixing["k"],
            f"{first_name}_queries": mixing["pva_queries"],
            f"{second_name}_queries": mixing["artiverse_queries"],
            "source_macro_chance_adjusted": mixing["source_macro_chance_adjusted"],
        },
    }


def compute_metrics(
    pva_features: Mapping[str, np.ndarray],
    artiverse_features: Mapping[str, np.ndarray],
    articraft_features: Mapping[str, np.ndarray],
    joint_coordinates: Mapping[str, np.ndarray],
    *,
    neighbor_fraction: float,
) -> dict[str, Any]:
    base = _base()
    output: dict[str, Any] = {
        "schema_version": 1,
        "protocol": {
            "distance": "cosine distance on L2-normalized high-dimensional embeddings",
            "joint_projection": "concatenate three sources per encoder, PCA50, then one fitted t-SNE",
            "neighbor_fraction": neighbor_fraction,
            "tsne_policy": "visualization only; projection trustworthiness is reported separately",
        },
        "encoder_agreement": {
            "pva": base.encoder_agreement_metrics(pva_features["dinov2"], pva_features["clip"], neighbor_fraction=neighbor_fraction),
            "artiverse": base.encoder_agreement_metrics(artiverse_features["dinov2"], artiverse_features["clip"], neighbor_fraction=neighbor_fraction),
            "articraft10k": base.encoder_agreement_metrics(articraft_features["dinov2"], articraft_features["clip"], neighbor_fraction=neighbor_fraction),
        },
        "per_encoder": {},
    }
    for encoder in ENCODERS:
        source_features = {"PV-A": pva_features[encoder], "Artiverse": artiverse_features[encoder], "Articraft-10K": articraft_features[encoder]}
        output["per_encoder"][encoder] = {
            "pva_vs_artiverse": named_cross_dataset_metrics(
                pva_features[encoder], artiverse_features[encoder],
                first_name="pva", second_name="artiverse",
                neighbor_fraction=neighbor_fraction,
            ),
            "pva_vs_articraft10k": named_cross_dataset_metrics(
                pva_features[encoder], articraft_features[encoder],
                first_name="pva", second_name="articraft10k",
                neighbor_fraction=neighbor_fraction,
            ),
            "artiverse_vs_articraft10k": named_cross_dataset_metrics(
                artiverse_features[encoder], articraft_features[encoder],
                first_name="artiverse", second_name="articraft10k",
                neighbor_fraction=neighbor_fraction,
            ),
            "three_source": multi_source_metrics(source_features, tuple(source_features), neighbor_fraction=neighbor_fraction),
            "joint_projection_trustworthiness": base.projection_trustworthiness(
                np.vstack([pva_features[encoder], artiverse_features[encoder], articraft_features[encoder]]),
                joint_coordinates[encoder], neighbor_fraction=neighbor_fraction,
            ),
        }
    return output


def audit_outputs(
    output_dir: Path,
    *,
    dimensions: Mapping[str, int],
    counts: Mapping[str, int],
    artifact_paths: Sequence[Path],
) -> dict[str, Any]:
    checks: dict[str, bool] = {}
    artifacts: dict[str, dict[str, Any]] = {}
    expected_arrays = {
        "articraft_one_shot_features.npy": counts["Articraft-10K"],
    }
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
            expected_rows = expected_arrays.get(path.name)
            checks[f"shape:{rel}"] = expected_rows is not None and matrix.shape == (expected_rows, dimensions[encoder])
            checks[f"finite:{rel}"] = bool(np.isfinite(matrix).all())
            checks[f"normalized:{rel}"] = bool(np.allclose(np.linalg.norm(matrix, axis=1), 1.0, rtol=2e-5, atol=2e-5))
            receipt.update({"shape": list(matrix.shape), "dtype": str(matrix.dtype)})
        elif path.suffix == ".csv":
            with path.open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            expected = None
            if path.name == "articraft_index.csv":
                expected = counts["Articraft-10K"]
                checks[f"unique_categories:{rel}"] = len({row.get("category") for row in rows}) == expected
                checks[f"unique_colors:{rel}"] = len({row.get("color") for row in rows}) == expected
            elif path.name == "articraft_one_shot_tsne_coordinates.csv":
                expected = counts["Articraft-10K"]
            elif path.name in {"joint_coordinates.csv", "joint_three_source_tsne_coordinates.csv"}:
                expected = sum(counts.values())
            if expected is not None:
                checks[f"rows:{rel}"] = len(rows) == expected
            receipt["row_count"] = len(rows)
        elif path.suffix == ".png":
            with Image.open(path) as image:
                image.load()
                std = ImageStat.Stat(image.convert("L")).stddev[0]
                checks[f"dimensions:{rel}"] = image.width >= 1000 and image.height >= 700
                checks[f"nonblank:{rel}"] = std > 1.0
                receipt.update({"width": image.width, "height": image.height, "grayscale_std": std})
        elif path.name == "high_dimensional_metrics.json":
            checks[f"finite:{rel}"] = _finite_json(load_json(path))
        artifacts[rel] = receipt
    return {"schema_version": 1, "pass": all(checks.values()), "checks": checks, "artifacts": artifacts}


def _finite_json(value: Any) -> bool:
    if isinstance(value, dict):
        return all(_finite_json(item) for item in value.values())
    if isinstance(value, list):
        return all(_finite_json(item) for item in value)
    if isinstance(value, float):
        return math.isfinite(value)
    return True


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.batch_size < 1 or args.num_workers < 1 or args.plot_dpi < 80:
        raise ValueError("batch size/workers must be positive and plot DPI must be at least 80")
    if not 0 < args.neighbor_fraction < 1:
        raise ValueError("neighbor fraction must be between zero and one")
    base = _base()
    output_dir = args.output_dir.expanduser().resolve()
    artiverse_comparison_dir = args.artiverse_comparison_dir.expanduser().resolve(strict=True)
    pva = base.load_pva_frozen(args.pva_dir, strict_counts=not args.allow_count_drift)
    artiverse = base.load_artiverse_uniform(
        args.artiverse_render_root,
        pva_render_config=pva.render_config,
        strict_counts=not args.allow_count_drift,
        verify_glb_hashes=not args.skip_artiverse_glb_hash_verification,
        one_shot_only=True,
    )
    articraft = load_articraft_uniform(
        args.articraft_render_root,
        pva_render_config=pva.render_config,
        strict_counts=not args.allow_count_drift,
    )
    for input_root in (pva.root, artiverse.root, articraft.root, artiverse_comparison_dir):
        if output_dir == input_root or output_dir.is_relative_to(input_root) or input_root.is_relative_to(output_dir):
            raise ValueError(f"output directory overlaps input root: {input_root}")
    output_dir.mkdir(parents=True, exist_ok=True)

    artiverse_features = _load_artiverse_cached(
        artiverse_comparison_dir,
        pva=pva,
        artiverse=artiverse,
    )
    articraft_features = _extract_articraft(
        articraft, pva=pva, output_dir=output_dir, batch_size=args.batch_size, device=args.device,
        num_workers=args.num_workers, use_amp=args.use_amp, force_extract=args.force_extract,
    )

    joint_coordinates: dict[str, np.ndarray] = {}
    independent_coordinates: dict[str, np.ndarray] = {}
    tsne_info: dict[str, Any] = {}
    for encoder in ENCODERS:
        print(f"[tsne] {encoder} Articraft-10K n={len(articraft.records)}", flush=True)
        independent_coordinates[encoder], independent_info = _compute_tsne(
            articraft_features[encoder], perplexity=args.perplexity, random_state=args.seed,
            max_iter=args.tsne_max_iter, n_jobs=args.tsne_jobs, verbose=args.tsne_verbose,
        )
        print(f"[tsne] {encoder} joint n={len(pva.records)+len(artiverse.records)+len(articraft.records)}", flush=True)
        joint_features = np.vstack([pva.features[encoder], artiverse_features[encoder], articraft_features[encoder]])
        joint_coordinates[encoder], joint_info = _compute_tsne(
            joint_features, perplexity=args.perplexity, random_state=args.seed,
            max_iter=args.tsne_max_iter, n_jobs=args.tsne_jobs, verbose=args.tsne_verbose,
        )
        tsne_info[encoder] = {"independent_articraft": independent_info, "joint": joint_info}

    colors = category_colors(len(articraft.records))
    index_rows = [
        {
            "category_id": f"C{index:03d}", "category": record.category, "asset_id": record.asset_id,
            "cohort_origin": record.cohort_origin, "ordinal": record.ordinal,
            "package_path": str(record.package_path), "image_path": str(record.image_path),
            "image_bytes": record.image_bytes, "image_sha256": record.image_sha256, "color": color,
        }
        for index, (record, color) in enumerate(zip(articraft.records, colors, strict=True), start=1)
    ]
    atomic_csv(output_dir / "articraft_index.csv", index_rows)
    atomic_json(output_dir / "articraft_selection_summary.json", {
        "official_model_count": articraft.official_model_count,
        "official_category_count": articraft.official_category_count,
        "rendered_category_count": len(articraft.records),
        "cohort_origin_counts": dict(sorted(Counter(r.cohort_origin for r in articraft.records).items())),
        "categories": list(articraft.categories),
    })
    for encoder in ENCODERS:
        save_articraft_plot(
            independent_coordinates[encoder], articraft.categories, colors,
            output_dir / encoder / "articraft_one_shot_tsne.png",
            title=f"Articraft-10K uniform {encoder}: one representative per category (n={len(articraft.records)})",
            dpi=args.plot_dpi,
        )
        write_coordinates(output_dir / encoder / "articraft_one_shot_tsne_coordinates.csv", independent_coordinates[encoder], articraft.records)
        atomic_json(output_dir / encoder / "tsne_manifest.json", tsne_info[encoder])
        write_joint_coordinates(
            output_dir / encoder / "joint_three_source_tsne_coordinates.csv", joint_coordinates[encoder],
            pva_records=pva.records, artiverse_records=artiverse.records, articraft_records=articraft.records,
        )
    # Save the two-panel source comparison after both coordinates exist.
    save_three_source_plot(
        joint_coordinates,
        {"PV-A": len(pva.records), "Artiverse": len(artiverse.records), "Articraft-10K": len(articraft.records)},
        output_dir / "joint_three_source_comparison.png",
        labels={"PV-A": "PV-A", "Artiverse": "Artiverse", "Articraft-10K": "Articraft-10K"},
        dpi=args.plot_dpi,
    )
    for encoder in ENCODERS:
        atomic_json(output_dir / encoder / "joint_tsne_manifest.json", tsne_info[encoder]["joint"])

    metrics = compute_metrics(
        pva.features, artiverse_features, articraft_features, joint_coordinates,
        neighbor_fraction=args.neighbor_fraction,
    )
    atomic_json(output_dir / "high_dimensional_metrics.json", metrics)
    source_feature_receipts = {
        encoder: {
            "pva_feature_sha256": pva.receipts["encoders"][encoder]["class_features"]["sha256"],
            "artiverse_feature_sha256": sha256_file(
                artiverse_comparison_dir / encoder / "artiverse_one_shot_features.npy"
            ),
            "artiverse_feature_manifest_sha256": sha256_file(
                artiverse_comparison_dir / encoder / "feature_manifest.json"
            ),
            "articraft_feature_manifest_sha256": sha256_file(
                output_dir / encoder / "articraft_feature_manifest.json"
            ),
            "model_fingerprint": pva.feature_manifests[encoder]["model_fingerprint"],
        }
        for encoder in ENCODERS
    }
    stable = {
        "schema_version": 1,
        "artifact_contract": "pva_artiverse_articraft_uniform_comparison_v1",
        "counts": {
            "PV-A": len(pva.records),
            "Artiverse": len(artiverse.records),
            "Articraft-10K": len(articraft.records),
            "Articraft-10K_official_models": articraft.official_model_count,
            "Articraft-10K_official_categories": articraft.official_category_count,
        },
        "inputs": {
            "pva_root": str(pva.root),
            "pva_roster_sha256": pva.receipts["roster"]["sha256"],
            "pva_run_manifest_sha256": pva.receipts["run_manifest"]["sha256"],
            "artiverse_root": str(artiverse.root),
            "artiverse_render_manifest_sha256": artiverse.receipts["render_manifest"]["sha256"],
            "artiverse_ordered_image_receipt_sha256": artiverse.receipts["ordered_image_receipt_sha256"],
            "artiverse_comparison_dir": str(artiverse_comparison_dir),
            "artiverse_comparison_audit_sha256": sha256_file(
                artiverse_comparison_dir / "final_audit.json"
            ),
            "articraft_root": str(articraft.root),
            "articraft_render_config_sha256": articraft.receipts["render_config"]["sha256"],
            "articraft_render_manifest_sha256": articraft.receipts["render_manifest"]["sha256"],
            "articraft_roster_sha256": articraft.receipts["category_one_shot_roster"]["sha256"],
            "articraft_official_roster_sha256": articraft.receipts["official_roster"]["sha256"],
            "articraft_ordered_image_receipt_sha256": articraft.receipts["ordered_image_receipt_sha256"],
            "source_features": source_feature_receipts,
        },
        "implementation": {
            "script_sha256": sha256_file(SCRIPT),
            "base_compare_script_sha256": sha256_file(BASE_COMPARE_SCRIPT),
            "feature_tsne_helper_sha256": sha256_file(base.PVA_HELPER_SCRIPT),
        },
        "projection": {
            "perplexity": args.perplexity,
            "seed": args.seed,
            "tsne_max_iter": args.tsne_max_iter,
            "tsne_jobs": args.tsne_jobs,
            "neighbor_fraction": args.neighbor_fraction,
        },
        "feature_extraction_request": {
            "device": args.device,
            "batch_size": args.batch_size,
            "num_workers": args.num_workers,
            "use_amp": bool(args.use_amp),
        },
        "protocol": "one identity-hash-selected representative per Articraft category; identical PV-A studio values; same frozen DINOv2 and CLIP snapshots",
    }
    atomic_json(output_dir / "run_contract.json", stable)
    artifacts = [
        output_dir / "run_contract.json", output_dir / "articraft_index.csv", output_dir / "articraft_selection_summary.json",
        output_dir / "high_dimensional_metrics.json", output_dir / "joint_three_source_comparison.png",
    ]
    for encoder in ENCODERS:
        artifacts.extend([
            output_dir / encoder / "articraft_feature_manifest.json",
            output_dir / encoder / "articraft_one_shot_features.npy",
            output_dir / encoder / "articraft_one_shot_tsne.png",
            output_dir / encoder / "articraft_one_shot_tsne_coordinates.csv",
            output_dir / encoder / "joint_three_source_tsne_coordinates.csv",
            output_dir / encoder / "tsne_manifest.json",
            output_dir / encoder / "joint_tsne_manifest.json",
        ])
    audit = audit_outputs(
        output_dir, dimensions={encoder: int(pva.features[encoder].shape[1]) for encoder in ENCODERS},
        counts={"PV-A": len(pva.records), "Artiverse": len(artiverse.records), "Articraft-10K": len(articraft.records)},
        artifact_paths=artifacts,
    )
    atomic_json(output_dir / "final_audit.json", audit)
    if not audit["pass"]:
        raise RuntimeError("Articraft comparison output audit failed")
    manifest = {
        "schema_version": 1, "stable_run_contract": stable, "audit": audit,
        "articraft": {"config": articraft.receipts["render_config"], "manifest": articraft.receipts["render_manifest"], "roster": articraft.receipts["category_one_shot_roster"], "official_roster": articraft.receipts["official_roster"]},
        "encoders": {
            encoder: {
                "feature_dim": int(pva.features[encoder].shape[1]),
                "articraft_feature": file_receipt(
                    output_dir / encoder / "articraft_one_shot_features.npy"
                ),
                "articraft_feature_manifest": file_receipt(
                    output_dir / encoder / "articraft_feature_manifest.json"
                ),
                "tsne": tsne_info[encoder],
            }
            for encoder in ENCODERS
        },
        "interpretation_limits": ["Articraft-10K has 244 categories while PV-A has 531 generator classes and Artiverse has 84 categories; point counts are not semantic-equivalent.", "One representative per category has selection variance.", "t-SNE is visualization only; use high-dimensional cosine metrics for source-gap claims.", "The 244-category Articraft universe includes the 791-row GitHub supplement; cohort origin is recorded per selected row."],
    }
    atomic_json(output_dir / "run_manifest.json", manifest)
    return {"output_dir": str(output_dir), "counts": stable["counts"], "audit_pass": audit["pass"], "metrics": metrics}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pva-dir", type=Path, default=DEFAULT_PVA_DIR)
    parser.add_argument("--artiverse-render-root", type=Path, default=DEFAULT_ARTIVERSE_RENDER_ROOT)
    parser.add_argument("--artiverse-comparison-dir", type=Path, default=DEFAULT_ARTIVERSE_DIR)
    parser.add_argument("--articraft-render-root", type=Path, default=DEFAULT_ARTICRAFT_RENDER_ROOT)
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
    if argv is None:
        argv = os.sys.argv[1:]
    result = run(build_parser().parse_args(argv))
    print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
