#!/usr/bin/env python3
"""Compare PV-A, Artiverse, Articraft-10K, and PartNet-Mobility.

All four inputs are class-level, one-image-per-class renders made with the
same frozen Blender studio.  DINOv2 and CLIP remain separate feature spaces.
For each encoder this driver creates a PartNet-only t-SNE and one joint t-SNE
over 531 + 84 + 244 + 46 = 905 points.  Source-gap claims are computed in the
original L2-normalized feature space; t-SNE is visualization only.

The existing three-source comparison remains the authority for PV-A,
Artiverse, Articraft-10K, model loading, image preprocessing, and t-SNE.  This
file adds only the PartNet-Mobility release/render contract and four-source
outputs.
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
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from PIL import Image, ImageStat


for _thread_variable in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_thread_variable, "1")


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
THREE_SOURCE_SCRIPT = REPO_ROOT / "exp/scripts/compare_pva_artiverse_articraft_uniform.py"
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
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exp/runtime/pva_artiverse_articraft_partnet_uniform_comparison"

ENCODERS = ("dinov2", "clip")
SOURCE_ORDER = ("PV-A", "Artiverse", "Articraft-10K", "PartNet-Mobility")
SOURCE_KEYS = {
    "PV-A": "pva",
    "Artiverse": "artiverse",
    "Articraft-10K": "articraft10k",
    "PartNet-Mobility": "partnet_mobility",
}
EXPECTED_RELEASE_ASSETS = 2_347
EXPECTED_RELEASE_CATEGORIES = 46
EXPECTED_DATASET_MANIFEST_SHA256 = "0ee8fac27ab97d3ec4a4914693ce14324bd7820f3c3f39a2e3bee64278377c5b"
EXPECTED_MANIFEST_CONTENT_SHA256 = "e8c5ccc17c50489b1d9c01c2ab03f3068e510fffc191dfcf5f05e0182a1de1c8"
EXPECTED_ROSTER_SHA256 = "ae54292dc68b313f7b05def4c961f9fa520ae3e735770a32d2bca4a34e9cee4e"
EXPECTED_CANDIDATE_POOL_SHA256 = "0203a510202510cea7e469048e84b133bd65ccbc6e1e3aa90c9bfeea7807959d"
EXPECTED_CATEGORY_MAPPING_SHA256 = "4f4d0414d452df37cb4d50b0226c0004bdbded5b9ef5cdee7e7693d57f99ecad"
EXPECTED_SELECTION_SHA256 = "75071bcfbd93d1dbf9dc4c19509d5900b0af5ab6585c1c0bb9cfa9c04bf6ab2a"
SUCCESS_STATUSES = {"rendered", "reused_valid"}
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True, slots=True)
class PartNetRecord:
    ordinal: int
    dataset_id: str
    category: str
    package_path: Path
    urdf_path: Path
    package_file_count: int
    package_total_bytes: int
    package_content_manifest_sha256: str
    package_binding_sha256: str
    image_path: Path
    image_bytes: int
    image_sha256: str
    category_one_shot: bool

    @property
    def asset_id(self) -> str:
        return self.dataset_id


@dataclass(frozen=True, slots=True)
class PartNetBundle:
    root: Path
    records: tuple[PartNetRecord, ...]
    categories: tuple[str, ...]
    config: dict[str, Any]
    render_manifest_path: Path
    roster_path: Path
    dataset_root: Path
    dataset_manifest_path: Path
    official_model_count: int
    official_category_count: int
    receipts: dict[str, Any]


_THREE: Any | None = None


def _three() -> Any:
    global _THREE
    if _THREE is None:
        name = "_pva_artiverse_articraft_uniform_base_for_four_source"
        spec = importlib.util.spec_from_file_location(name, THREE_SOURCE_SCRIPT)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot load three-source comparison: {THREE_SOURCE_SCRIPT}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        _THREE = module
    return _THREE


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_hash(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


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


def _inside(root: Path, path: Path, *, field: str) -> Path:
    root = root.expanduser().resolve(strict=True)
    path = path.expanduser().resolve(strict=True)
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{field} escapes {root}: {path}") from exc
    return path


def _resolve_path(value: Any, *, parent: Path, field: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"missing path field {field}")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = parent / path
    return path.resolve(strict=True)


def _parse_bool(value: Any, *, field: str) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"true", "1"}:
        return True
    if normalized in {"false", "0"}:
        return False
    raise ValueError(f"invalid boolean {field}: {value!r}")


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"CSV is empty: {path}")
    return rows


def _load_module(path: Path, expected_sha256: str, label: str) -> Any:
    module_name = f"_partnet_uniform_{label}_{expected_sha256}"
    existing = sys.modules.get(module_name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import PartNet {label}: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(module_name, None)
        raise
    return module


def _winner_items(items: Sequence[Any]) -> tuple[Any, ...]:
    winners: dict[str, Any] = {}
    for item in items:
        current = winners.get(item.category)
        if current is None or (item.identity_sha256, item.asset_id) < (
            current.identity_sha256,
            current.asset_id,
        ):
            winners[item.category] = item
    return tuple(winners[category] for category in sorted(winners))


def _dataset_id_order(value: str) -> tuple[int, int | str]:
    """Match the numeric ordering used by the frozen PartNet release roster."""

    return (0, int(value)) if value.isdigit() else (1, value)


def _row_identity(row: Mapping[str, Any]) -> str:
    dataset_id = str(row.get("dataset_id") or row.get("asset_id") or "").strip()
    other = str(row.get("asset_id") or row.get("dataset_id") or "").strip()
    if not dataset_id or dataset_id != other:
        raise ValueError("PartNet render row has missing or conflicting dataset identity")
    return dataset_id


def _row_image_path(root: Path, row: Mapping[str, Any]) -> Path:
    value = row.get("image_path") or row.get("output_path")
    if not isinstance(value, str) or not value.strip():
        value = str(Path(str(row["category"])) / _row_identity(row) / "imgs/000.png")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = root / path
    return _inside(root, path, field="PartNet image_path")


def load_partnet_uniform(
    root: Path,
    *,
    pva_render_config: Mapping[str, Any],
    strict_counts: bool = True,
) -> PartNetBundle:
    root = root.expanduser().resolve(strict=True)
    config_path = root / "render_config.json"
    manifest_path = root / "render_manifest.csv"
    roster_path = root / "category_one_shot_roster.csv"
    for path in (config_path, manifest_path, roster_path):
        if not path.is_file() or path.is_symlink():
            raise FileNotFoundError(f"incomplete PartNet uniform root: {path}")
    config = load_json(config_path)
    contract = str(config.get("render_contract") or "").lower()
    if "partnet" not in contract or "uniform" not in contract:
        raise ValueError(f"unexpected PartNet render contract: {contract!r}")
    if config.get("dataset") not in {"PartNet-Mobility", "partnet_mobility"}:
        raise ValueError(f"unexpected PartNet dataset identity: {config.get('dataset')!r}")
    for field in ("resolution", "samples", "studio", "blender_version"):
        if config.get(field) != pva_render_config.get(field):
            raise ValueError(f"PartNet/PV-A render contract mismatch for {field}")
    for field in ("pose_policy", "material_policy"):
        if not isinstance(config.get(field), str) or not str(config[field]).strip():
            raise ValueError(f"PartNet render config has no {field}")
    configured_output_root = _resolve_path(
        config.get("output_root"), parent=root, field="output_root"
    )
    if configured_output_root != root:
        raise ValueError("PartNet render config output root mismatch")

    config_files: dict[str, Path] = {}
    for field in ("driver", "renderer", "base_renderer", "shared_renderer"):
        path = _resolve_path(config.get(field), parent=root, field=field)
        declared = config.get(f"{field}_sha256")
        if not isinstance(declared, str) or SHA256_RE.fullmatch(declared) is None:
            raise ValueError(f"PartNet {field} SHA-256 receipt is malformed")
        if sha256_file(path) != declared:
            raise ValueError(f"PartNet {field} no longer matches its frozen SHA-256")
        config_files[field] = path
    if config.get("shared_renderer_sha256") != pva_render_config.get("renderer_sha256"):
        raise ValueError("PartNet did not use the frozen PV-A shared studio renderer")

    driver = _load_module(
        config_files["driver"],
        str(config["driver_sha256"]),
        "render_driver",
    )
    dataset_manifest_path = _resolve_path(
        config.get("dataset_manifest"),
        parent=root,
        field="dataset_manifest",
    )
    if dataset_manifest_path.is_symlink() or not dataset_manifest_path.is_file():
        raise ValueError("PartNet full-release manifest must be a regular non-symlink file")
    dataset_manifest_sha256 = sha256_file(dataset_manifest_path)
    if dataset_manifest_sha256 != config.get("dataset_manifest_sha256"):
        raise ValueError("PartNet full-release manifest file receipt mismatch")
    if strict_counts and dataset_manifest_sha256 != EXPECTED_DATASET_MANIFEST_SHA256:
        raise ValueError("PartNet render does not bind the expected full-release manifest")
    official_manifest, official_rows = driver._load_manifest(
        dataset_manifest_path,
        strict_counts=strict_counts,
    )
    if config.get("dataset_manifest_content_sha256") != official_manifest.get(
        "manifest_content_sha256"
    ):
        raise ValueError("PartNet manifest content receipt mismatch")
    if config.get("dataset_roster_sha256") != official_manifest.get("roster_sha256"):
        raise ValueError("PartNet manifest roster receipt mismatch")
    if strict_counts and (
        official_manifest.get("manifest_content_sha256") != EXPECTED_MANIFEST_CONTENT_SHA256
        or official_manifest.get("roster_sha256") != EXPECTED_ROSTER_SHA256
    ):
        raise ValueError("PartNet official manifest logical receipts differ from the frozen release")
    dataset_root = driver._source_root(official_manifest)
    configured_source_root = _resolve_path(
        config.get("source_root"),
        parent=root,
        field="source_root",
    )
    if configured_source_root != dataset_root:
        raise ValueError("PartNet source root differs from its full-release manifest")
    all_items = driver.load_render_items(
        dataset_manifest_path,
        output_root=root,
        strict_counts=strict_counts,
        validate_inputs=False,
    )
    if int(config.get("model_count", -1)) != len(all_items):
        raise ValueError("PartNet render model count differs from its complete release")
    category_count = len({item.category for item in all_items})
    if int(config.get("category_count", -1)) != category_count:
        raise ValueError("PartNet render category count differs from its complete release")
    ordered_items = sorted(all_items, key=lambda item: _dataset_id_order(item.asset_id))
    ids = [item.asset_id for item in ordered_items]
    mapping_rows = [
        {"dataset_id": item.asset_id, "category": item.category}
        for item in ordered_items
    ]
    candidate_pool_sha256 = _canonical_hash(ids)
    category_mapping_sha256 = _canonical_hash(mapping_rows)
    if strict_counts and (
        len(all_items) != EXPECTED_RELEASE_ASSETS
        or category_count != EXPECTED_RELEASE_CATEGORIES
        or candidate_pool_sha256 != EXPECTED_CANDIDATE_POOL_SHA256
        or category_mapping_sha256 != EXPECTED_CATEGORY_MAPPING_SHA256
    ):
        raise ValueError("PartNet full-release identity/category universe differs from expected")

    winners = _winner_items(all_items)
    expected_selection = driver._selection_receipt(winners)
    if strict_counts and expected_selection["identity_category_sha256"] != EXPECTED_SELECTION_SHA256:
        raise ValueError("PartNet identity-hash winner universe differs from the official release")
    selection = config.get("selection")
    if not isinstance(selection, Mapping) or selection.get("one_shot_only") is not True:
        raise ValueError("PartNet render config is not one-shot-only")
    if selection.get("selected_receipt") != expected_selection:
        raise ValueError("PartNet selected receipt differs from the independently derived winners")
    if int(config.get("selected_count", -1)) != len(winners):
        raise ValueError("PartNet selected count differs from its category winners")
    if int(config.get("selected_category_count", -1)) != len(winners):
        raise ValueError("PartNet selected category count differs from its category winners")

    rendered_rows = _read_csv(manifest_path)
    rendered_by_id: dict[str, dict[str, str]] = {}
    for row in rendered_rows:
        dataset_id = _row_identity(row)
        if dataset_id in rendered_by_id:
            raise ValueError(f"duplicate PartNet render identity: {dataset_id}")
        rendered_by_id[dataset_id] = row
    if len(rendered_rows) != len(winners):
        raise ValueError(f"expected {len(winners)} PartNet render receipts, found {len(rendered_rows)}")

    official_by_id = {str(row["asset_id"]): row for row in official_rows}
    records: list[PartNetRecord] = []
    for winner in winners:
        row = rendered_by_id.get(winner.asset_id)
        if row is None:
            raise ValueError(f"missing rendered PartNet category winner: {winner.category}")
        if str(row.get("category") or "") != winner.category:
            raise ValueError(f"PartNet category drift for {winner.asset_id}")
        try:
            ordinal = int(row.get("ordinal", -1))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"PartNet ordinal is invalid: {winner.asset_id}") from exc
        if ordinal != winner.ordinal:
            raise ValueError(f"PartNet ordinal drift for {winner.asset_id}")
        if str(row.get("identity_sha256") or "") != winner.identity_sha256:
            raise ValueError(f"PartNet identity SHA mismatch: {winner.asset_id}")
        if not _parse_bool(row.get("category_one_shot"), field="category_one_shot"):
            raise ValueError(f"PartNet winner is not marked category_one_shot: {winner.asset_id}")
        source = _resolve_path(
            row.get("source_path") or row.get("package_path"),
            parent=root,
            field="source_path",
        )
        urdf = _resolve_path(row.get("urdf_path"), parent=root, field="urdf_path")
        if source != winner.source_path or urdf != winner.urdf_path:
            raise ValueError(f"PartNet winner source binding drift: {winner.asset_id}")
        try:
            urdf_bytes = int(row.get("urdf_bytes", -1))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"PartNet URDF byte receipt is invalid: {winner.asset_id}") from exc
        urdf_sha256 = str(row.get("urdf_sha256") or "")
        if urdf_bytes != winner.urdf_bytes or urdf_sha256 != winner.urdf_sha256:
            raise ValueError(f"PartNet URDF receipt mismatch: {winner.asset_id}")
        official_row = official_by_id[winner.asset_id]
        official_binding = official_row.get("package_binding")
        if not isinstance(official_binding, Mapping):
            raise ValueError(f"PartNet official package receipt is missing: {winner.asset_id}")
        official_projection = {
            "ordinal": official_row.get("ordinal"),
            "category": official_row.get("category"),
            "asset_id": official_row.get("asset_id"),
            "source_path": official_row.get("source_path"),
            "source_relative_path": official_row.get("source_relative_path"),
            "urdf_path": official_row.get("primary_urdf_path"),
            "urdf_relative_path": official_row.get("primary_urdf_relative_path"),
            "urdf_bytes": official_row.get(
                "primary_urdf_bytes", official_row.get("primary_urdf_size")
            ),
            "urdf_sha256": official_row.get("primary_urdf_sha256"),
            "package_file_count": official_binding.get("file_count"),
            "package_total_bytes": official_binding.get("total_bytes"),
            "package_content_manifest_sha256": official_binding.get(
                "content_manifest_sha256"
            ),
            "package_binding_sha256": official_row.get("package_binding_sha256"),
            "identity_sha256": hashlib.sha256(winner.asset_id.encode("utf-8")).hexdigest(),
        }
        winner_projection = {
            "ordinal": winner.ordinal,
            "category": winner.category,
            "asset_id": winner.asset_id,
            "source_path": str(winner.source_path),
            "source_relative_path": winner.source_relative_path,
            "urdf_path": str(winner.urdf_path),
            "urdf_relative_path": winner.urdf_relative_path,
            "urdf_bytes": winner.urdf_bytes,
            "urdf_sha256": winner.urdf_sha256,
            "package_file_count": winner.package_file_count,
            "package_total_bytes": winner.package_total_bytes,
            "package_content_manifest_sha256": winner.package_content_manifest_sha256,
            "package_binding_sha256": winner.package_binding_sha256,
            "identity_sha256": winner.identity_sha256,
        }
        if winner_projection != official_projection:
            raise ValueError(f"PartNet selected item differs from official manifest: {winner.asset_id}")
        validated_binding = driver._validate_package_files(official_row, source)
        if validated_binding != winner.package_binding_sha256:
            raise ValueError(f"PartNet selected package binding mismatch: {winner.asset_id}")
        expected_row = {
            "ordinal": winner.ordinal,
            "category": winner.category,
            "asset_id": winner.asset_id,
            "source_path": str(winner.source_path),
            "source_relative_path": winner.source_relative_path,
            "urdf_path": str(winner.urdf_path),
            "urdf_relative_path": winner.urdf_relative_path,
            "urdf_bytes": winner.urdf_bytes,
            "urdf_sha256": winner.urdf_sha256,
            "package_file_count": winner.package_file_count,
            "package_total_bytes": winner.package_total_bytes,
            "package_content_manifest_sha256": winner.package_content_manifest_sha256,
            "package_binding_sha256": winner.package_binding_sha256,
            "identity_sha256": winner.identity_sha256,
            "category_one_shot": True,
            "output_path": str(winner.output_path),
        }
        for field, expected_value in expected_row.items():
            if str(row.get(field) or "") != str(expected_value):
                raise ValueError(f"PartNet render manifest differs from frozen winner for {field}")
        if str(row.get("status") or "") not in SUCCESS_STATUSES:
            raise ValueError(f"PartNet winner has non-success status: {winner.asset_id}")

        image_path = _row_image_path(root, row)
        expected_image = root / winner.category / winner.asset_id / "imgs" / "000.png"
        if image_path != expected_image.resolve(strict=True):
            raise ValueError(f"PartNet winner output path differs from the stable layout: {winner.asset_id}")
        if image_path.read_bytes()[: len(PNG_SIGNATURE)] != PNG_SIGNATURE:
            raise ValueError(f"PartNet image is not a PNG: {image_path}")
        with Image.open(image_path) as image:
            image.load()
            if image.size != (int(config["resolution"]), int(config["resolution"])):
                raise ValueError(f"PartNet PNG dimensions differ from render contract: {image_path}")
            if image.mode not in {"RGB", "RGBA"}:
                raise ValueError(f"PartNet PNG color mode is invalid: {image_path}")
        try:
            image_bytes = int(row.get("png_bytes", row.get("image_bytes", -1)))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"PartNet PNG byte receipt is invalid: {winner.asset_id}") from exc
        image_sha256 = str(row.get("png_sha256") or row.get("image_sha256") or "")
        if image_bytes != image_path.stat().st_size or image_sha256 != sha256_file(image_path):
            raise ValueError(f"PartNet PNG receipt mismatch: {winner.asset_id}")

        try:
            renderer_result = json.loads(str(row.get("renderer_result") or "{}"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"PartNet renderer_result is invalid: {winner.asset_id}") from exc
        shared_receipt = renderer_result.get("shared_renderer") if isinstance(renderer_result, dict) else None
        if not isinstance(shared_receipt, Mapping) or (
            shared_receipt.get("path") != str(config_files["shared_renderer"])
            or shared_receipt.get("sha256") != config.get("shared_renderer_sha256")
        ):
            raise ValueError(f"PartNet shared renderer receipt mismatch: {winner.asset_id}")
        base_receipt = renderer_result.get("base_renderer") if isinstance(renderer_result, dict) else None
        if not isinstance(base_receipt, Mapping) or (
            base_receipt.get("path") != str(config_files["base_renderer"])
            or base_receipt.get("sha256") != config.get("base_renderer_sha256")
        ):
            raise ValueError(f"PartNet base renderer receipt mismatch: {winner.asset_id}")
        if renderer_result.get("material_policy") != config.get("material_policy"):
            raise ValueError(f"PartNet renderer material policy mismatch: {winner.asset_id}")
        try:
            rendered_source = _resolve_path(
                renderer_result.get("asset_dir"), parent=root, field="renderer_result.asset_dir"
            )
            rendered_output = _resolve_path(
                renderer_result.get("output"), parent=root, field="renderer_result.output"
            )
        except (OSError, ValueError) as exc:
            raise ValueError(
                f"PartNet renderer input/output receipt is invalid: {winner.asset_id}"
            ) from exc
        if rendered_source != source or rendered_output != image_path:
            raise ValueError(f"PartNet renderer input/output receipt mismatch: {winner.asset_id}")

        records.append(
            PartNetRecord(
                ordinal=winner.ordinal,
                dataset_id=winner.asset_id,
                category=winner.category,
                package_path=winner.source_path,
                urdf_path=winner.urdf_path,
                package_file_count=winner.package_file_count,
                package_total_bytes=winner.package_total_bytes,
                package_content_manifest_sha256=winner.package_content_manifest_sha256,
                package_binding_sha256=winner.package_binding_sha256,
                image_path=image_path,
                image_bytes=image_bytes,
                image_sha256=image_sha256,
                category_one_shot=True,
            )
        )

    roster_rows = _read_csv(roster_path)
    if len(roster_rows) != len(records):
        raise ValueError("PartNet one-shot roster row count mismatch")
    expected_categories = [record.category for record in records]
    if [str(row.get("category") or "") for row in roster_rows] != expected_categories:
        raise ValueError("PartNet one-shot roster category order mismatch")
    for roster_row, record in zip(roster_rows, records, strict=True):
        if _row_identity(roster_row) != record.dataset_id:
            raise ValueError("PartNet one-shot roster identity mismatch")
        manifest_row = rendered_by_id[record.dataset_id]
        for field in driver.ROSTER_FIELDS:
            if str(roster_row.get(field) or "") != str(manifest_row.get(field) or ""):
                raise ValueError(f"PartNet one-shot roster differs from manifest for {field}")

    ordered_images = [
        {
            "category": record.category,
            "dataset_id": record.dataset_id,
            "image_bytes": record.image_bytes,
            "image_sha256": record.image_sha256,
        }
        for record in records
    ]
    selected_package_rows = [
        {
            "category": record.category,
            "dataset_id": record.dataset_id,
            "package_file_count": record.package_file_count,
            "package_total_bytes": record.package_total_bytes,
            "package_content_manifest_sha256": record.package_content_manifest_sha256,
            "package_binding_sha256": record.package_binding_sha256,
        }
        for record in records
    ]
    receipts = {
        "render_config": file_receipt(config_path),
        "render_manifest": file_receipt(manifest_path),
        "category_one_shot_roster": file_receipt(roster_path),
        "dataset_manifest": file_receipt(dataset_manifest_path),
        "release": {
            "asset_count": len(all_items),
            "category_count": category_count,
            "candidate_pool_sha256": candidate_pool_sha256,
            "category_mapping_sha256": category_mapping_sha256,
            "manifest_content_sha256": official_manifest["manifest_content_sha256"],
            "roster_sha256": official_manifest["roster_sha256"],
        },
        "selection": expected_selection,
        "selected_package_receipt_sha256": _canonical_hash(selected_package_rows),
        "ordered_image_receipt_sha256": _canonical_hash(ordered_images),
    }
    return PartNetBundle(
        root=root,
        records=tuple(records),
        categories=tuple(expected_categories),
        config=config,
        render_manifest_path=manifest_path,
        roster_path=roster_path,
        dataset_root=dataset_root,
        dataset_manifest_path=dataset_manifest_path,
        official_model_count=len(all_items),
        official_category_count=category_count,
        receipts=receipts,
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


def _extract_partnet(
    bundle: PartNetBundle,
    *,
    pva: Any,
    output_dir: Path,
    batch_size: int,
    device: str,
    num_workers: int,
    use_amp: bool,
    force_extract: bool,
) -> dict[str, np.ndarray]:
    three = _three()
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
    execution_policy = {
        "requested_device": device,
        "resolved_device": str(torch_device),
        "requested_use_amp": bool(use_amp),
        "amp_enabled": bool(use_amp and torch_device.type == "cuda"),
        "batch_size": batch_size,
        "num_workers": num_workers,
    }
    ordered_fingerprint = hashlib.sha256(
        "\n".join(
            f"{record.category}\0{record.dataset_id}\0{record.image_bytes}\0{record.image_sha256}"
            for record in bundle.records
        ).encode("utf-8")
    ).hexdigest()
    features: dict[str, np.ndarray] = {}
    for encoder in ENCODERS:
        enc_dir = output_dir / encoder
        enc_dir.mkdir(parents=True, exist_ok=True)
        feature_path = enc_dir / "partnet_one_shot_features.npy"
        manifest_path = enc_dir / "partnet_feature_manifest.json"
        model_manifest = pva.feature_manifests[encoder]
        model_path = Path(str(model_manifest["model_path"])).resolve(strict=True)
        expected = {
            "schema_version": 1,
            "dataset": "PartNet-Mobility",
            "sample_mode": "one_identity_hash_winner_per_exact_model_cat",
            "encoder": encoder,
            "model_type": encoder,
            "model_path": str(model_path),
            "model_fingerprint": model_manifest["model_fingerprint"],
            "model_count": len(bundle.records),
            "ordered_image_fingerprint": ordered_fingerprint,
            "render_config_sha256": bundle.receipts["render_config"]["sha256"],
            "render_manifest_sha256": bundle.receipts["render_manifest"]["sha256"],
            "one_shot_roster_sha256": bundle.receipts["category_one_shot_roster"]["sha256"],
            "dataset_manifest_sha256": bundle.receipts["dataset_manifest"]["sha256"],
            "manifest_content_sha256": bundle.receipts["release"]["manifest_content_sha256"],
            "roster_sha256": bundle.receipts["release"]["roster_sha256"],
            "candidate_pool_sha256": bundle.receipts["release"]["candidate_pool_sha256"],
            "category_mapping_sha256": bundle.receipts["release"]["category_mapping_sha256"],
            "selected_package_receipt_sha256": bundle.receipts["selected_package_receipt_sha256"],
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
                if all(old.get(key) == value for key, value in expected.items()):
                    matrix = _load_matrix(
                        feature_path,
                        rows=len(bundle.records),
                        dimension=int(pva.features[encoder].shape[1]),
                    )
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
            if extraction.get("model_type") != encoder:
                raise ValueError(f"extracted PartNet {encoder} model type mismatch")
            if extraction.get("device") != execution_policy["resolved_device"]:
                raise ValueError(f"extracted PartNet {encoder} device differs from requested policy")
            if bool(extraction.get("amp")) != execution_policy["amp_enabled"]:
                raise ValueError(f"extracted PartNet {encoder} AMP differs from requested policy")
            if matrix.shape[1] != pva.features[encoder].shape[1]:
                raise ValueError(f"extracted PartNet {encoder} feature dimension mismatch")
            atomic_npy(feature_path, matrix)
            atomic_json(
                manifest_path,
                {**expected, "dtype": str(matrix.dtype), "extraction": extraction},
            )
        features[encoder] = matrix
    return features


def _compute_tsne(
    features: np.ndarray,
    *,
    perplexity: float,
    random_state: int,
    max_iter: int,
    n_jobs: int,
    verbose: int,
) -> tuple[np.ndarray, dict[str, Any]]:
    return _three()._compute_tsne(
        features,
        perplexity=perplexity,
        random_state=random_state,
        max_iter=max_iter,
        n_jobs=n_jobs,
        verbose=verbose,
    )


def category_colors(count: int) -> tuple[str, ...]:
    return tuple(_three().category_colors(count))


def save_partnet_plot(
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

    if coordinates.shape != (len(categories), 2) or len(colors) != len(categories):
        raise ValueError("PartNet plot inputs have inconsistent lengths")
    figure, axis = plt.subplots(figsize=(16, 12), dpi=dpi)
    axis.scatter(
        coordinates[:, 0],
        coordinates[:, 1],
        s=58,
        c=list(colors),
        alpha=0.86,
        edgecolors="white",
        linewidths=0.4,
        rasterized=True,
    )
    for index, point in enumerate(coordinates, start=1):
        axis.annotate(f"P{index:02d}", point, xytext=(4, 3), textcoords="offset points", fontsize=6.0)
    axis.set_title(title)
    axis.set_xlabel("t-SNE 1")
    axis.set_ylabel("t-SNE 2")
    axis.grid(color="#d8d8d8", linewidth=0.4, alpha=0.55)
    axis.spines[["top", "right"]].set_visible(False)
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def save_four_source_plot(
    coordinates: Mapping[str, np.ndarray],
    counts: Mapping[str, int],
    path: Path,
    *,
    dpi: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    specs = {
        "PV-A": ("#d1495b", "o", 18, 0.62),
        "Artiverse": ("#2878b5", "^", 40, 0.84),
        "Articraft-10K": ("#2a9d5b", "s", 35, 0.80),
        "PartNet-Mobility": ("#8a5aa8", "D", 42, 0.88),
    }
    expected = sum(counts[source] for source in SOURCE_ORDER)
    figure, axes = plt.subplots(1, 2, figsize=(18, 9), dpi=dpi, squeeze=False)
    for axis, encoder in zip(axes[0], ENCODERS, strict=True):
        if coordinates[encoder].shape != (expected, 2):
            raise ValueError(f"four-source coordinate shape mismatch for {encoder}")
        start = 0
        for source in SOURCE_ORDER:
            end = start + counts[source]
            color, marker, size, alpha = specs[source]
            points = coordinates[encoder][start:end]
            axis.scatter(
                points[:, 0],
                points[:, 1],
                s=size,
                color=color,
                marker=marker,
                alpha=alpha,
                edgecolors="white" if source != "PV-A" else "none",
                linewidths=0.3,
                label=f"{source} (n={counts[source]})",
                rasterized=True,
            )
            start = end
        axis.set_title(f"{encoder.upper() if encoder == 'clip' else 'DINOv2'} joint t-SNE")
        axis.set_xlabel("joint t-SNE 1")
        axis.set_ylabel("joint t-SNE 2")
        axis.grid(color="#d8d8d8", linewidth=0.4, alpha=0.52)
        axis.spines[["top", "right"]].set_visible(False)
        axis.legend(frameon=False, loc="best")
    figure.suptitle(
        "Uniform Blender comparison: PV-A, Artiverse, Articraft-10K, and PartNet-Mobility",
        fontsize=18,
    )
    figure.text(
        0.5,
        0.012,
        "Each encoder is fitted jointly in its own high-dimensional feature space; axes are not cross-encoder coordinates.",
        ha="center",
        fontsize=10,
        color="#444444",
    )
    figure.tight_layout(rect=(0.0, 0.035, 1.0, 0.95))
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)
    plt.close(figure)


def write_partnet_coordinates(
    path: Path,
    coordinates: np.ndarray,
    records: Sequence[PartNetRecord],
) -> None:
    if coordinates.shape != (len(records), 2):
        raise ValueError("PartNet coordinate count mismatch")
    rows = [
        {
            "tsne_x": format(float(point[0]), ".9g"),
            "tsne_y": format(float(point[1]), ".9g"),
            "category_id": f"P{index:02d}",
            "category": record.category,
            "dataset_id": record.dataset_id,
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
    articraft_records: Sequence[Any],
    partnet_records: Sequence[PartNetRecord],
) -> None:
    source_specs = (
        ("PV-A", pva_records),
        ("Artiverse", artiverse_records),
        ("Articraft-10K", articraft_records),
        ("PartNet-Mobility", partnet_records),
    )
    expected = sum(len(records) for _, records in source_specs)
    if coordinates.shape != (expected, 2):
        raise ValueError("joint four-source coordinate count mismatch")
    rows: list[dict[str, Any]] = []
    offset = 0
    for source, records in source_specs:
        source_coordinates = coordinates[offset : offset + len(records)]
        for local, (point, record) in enumerate(zip(source_coordinates, records, strict=True)):
            if source == "PV-A":
                class_id = record.generator_index
                class_name = record.generator_name
                origin = record.source_type
                source_identity = ""
            elif source == "Artiverse":
                class_id = f"C{local + 1:02d}"
                class_name = record.category
                origin = getattr(record, "one_shot_source", "") or getattr(record, "source", "")
                source_identity = getattr(record, "one_shot_manifest_root", "")
            elif source == "Articraft-10K":
                class_id = f"C{local + 1:03d}"
                class_name = record.category
                origin = record.cohort_origin
                source_identity = record.asset_id
            else:
                class_id = f"P{local + 1:02d}"
                class_name = record.category
                origin = "official_partnet_mobility_v0"
                source_identity = record.dataset_id
            rows.append(
                {
                    "tsne_x": format(float(point[0]), ".9g"),
                    "tsne_y": format(float(point[1]), ".9g"),
                    "joint_index": offset + local,
                    "dataset": source,
                    "class_id": class_id,
                    "class_name": class_name,
                    "source": origin,
                    "source_identity": source_identity,
                }
            )
        offset += len(records)
    atomic_csv(path, rows)


def compute_metrics(
    source_features: Mapping[str, Mapping[str, np.ndarray]],
    joint_coordinates: Mapping[str, np.ndarray],
    *,
    neighbor_fraction: float,
) -> dict[str, Any]:
    if tuple(source_features) != SOURCE_ORDER:
        raise ValueError(f"source feature order must be {SOURCE_ORDER!r}")
    three = _three()
    base = three._base()
    output: dict[str, Any] = {
        "schema_version": 1,
        "protocol": {
            "distance": "cosine distance on L2-normalized high-dimensional embeddings",
            "joint_projection": "concatenate four sources per encoder, PCA50, then one fitted t-SNE",
            "neighbor_fraction": neighbor_fraction,
            "tsne_policy": "visualization only; projection trustworthiness is reported separately",
        },
        "encoder_agreement": {
            SOURCE_KEYS[source]: base.encoder_agreement_metrics(
                source_features[source]["dinov2"],
                source_features[source]["clip"],
                neighbor_fraction=neighbor_fraction,
            )
            for source in SOURCE_ORDER
        },
        "per_encoder": {},
    }
    for encoder in ENCODERS:
        encoder_sources = {source: source_features[source][encoder] for source in SOURCE_ORDER}
        pairwise: dict[str, Any] = {}
        for first, second in itertools.combinations(SOURCE_ORDER, 2):
            first_key = SOURCE_KEYS[first]
            second_key = SOURCE_KEYS[second]
            pairwise[f"{first_key}_vs_{second_key}"] = three.named_cross_dataset_metrics(
                encoder_sources[first],
                encoder_sources[second],
                first_name=first_key,
                second_name=second_key,
                neighbor_fraction=neighbor_fraction,
            )
        combined = np.vstack([encoder_sources[source] for source in SOURCE_ORDER])
        output["per_encoder"][encoder] = {
            "pairwise": pairwise,
            "four_source": three.multi_source_metrics(
                encoder_sources,
                SOURCE_ORDER,
                neighbor_fraction=neighbor_fraction,
            ),
            "joint_projection_trustworthiness": base.projection_trustworthiness(
                combined,
                joint_coordinates[encoder],
                neighbor_fraction=neighbor_fraction,
            ),
        }
    return output


def _finite_json(value: Any) -> bool:
    if isinstance(value, dict):
        return all(_finite_json(item) for item in value.values())
    if isinstance(value, list):
        return all(_finite_json(item) for item in value)
    if isinstance(value, float):
        return math.isfinite(value)
    return True


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
        "partnet_one_shot_features.npy": counts["PartNet-Mobility"],
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
            matrix = np.load(path, allow_pickle=False)
            encoder = path.parent.name
            expected_rows = expected_arrays.get(path.name)
            checks[f"shape:{relative}"] = (
                expected_rows is not None and matrix.shape == (expected_rows, dimensions[encoder])
            )
            checks[f"finite:{relative}"] = bool(np.isfinite(matrix).all())
            checks[f"normalized:{relative}"] = bool(
                np.allclose(np.linalg.norm(matrix, axis=1), 1.0, rtol=2e-5, atol=2e-5)
            )
            receipt.update({"shape": list(matrix.shape), "dtype": str(matrix.dtype)})
        elif path.suffix == ".csv":
            rows = _read_csv(path)
            expected_rows: int | None = None
            if path.name == "partnet_index.csv":
                expected_rows = counts["PartNet-Mobility"]
                checks[f"unique_categories:{relative}"] = (
                    len({row.get("category") for row in rows}) == expected_rows
                )
                checks[f"unique_colors:{relative}"] = len({row.get("color") for row in rows}) == expected_rows
            elif path.name == "partnet_one_shot_tsne_coordinates.csv":
                expected_rows = counts["PartNet-Mobility"]
            elif path.name == "joint_four_source_tsne_coordinates.csv":
                expected_rows = sum(counts.values())
                observed = Counter(row.get("dataset") for row in rows)
                checks[f"source_counts:{relative}"] = observed == Counter(counts)
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
                    {
                        "width": image.width,
                        "height": image.height,
                        "grayscale_std": standard_deviation,
                    }
                )
        elif path.name == "high_dimensional_metrics.json":
            metrics = load_json(path)
            checks[f"finite:{relative}"] = _finite_json(metrics)
            for encoder in ENCODERS:
                pairs = metrics.get("per_encoder", {}).get(encoder, {}).get("pairwise", {})
                checks[f"six_pairs:{encoder}"] = len(pairs) == 6
        artifacts[relative] = receipt
    return {"schema_version": 1, "pass": all(checks.values()), "checks": checks, "artifacts": artifacts}


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.batch_size < 1 or args.num_workers < 1 or args.plot_dpi < 80:
        raise ValueError("batch size/workers must be positive and plot DPI must be at least 80")
    if not 0 < args.neighbor_fraction < 1:
        raise ValueError("neighbor fraction must be between zero and one")
    three = _three()
    base = three._base()
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
    articraft = three.load_articraft_uniform(
        args.articraft_render_root,
        pva_render_config=pva.render_config,
        strict_counts=not args.allow_count_drift,
    )
    partnet = load_partnet_uniform(
        args.partnet_render_root,
        pva_render_config=pva.render_config,
        strict_counts=not args.allow_count_drift,
    )
    input_roots = (
        pva.root,
        artiverse.root,
        articraft.root,
        partnet.root,
        artiverse_comparison_dir,
    )
    for input_root in input_roots:
        if (
            output_dir == input_root
            or output_dir.is_relative_to(input_root)
            or input_root.is_relative_to(output_dir)
        ):
            raise ValueError(f"output directory overlaps input root: {input_root}")
    output_dir.mkdir(parents=True, exist_ok=True)

    artiverse_features = three._load_artiverse_cached(
        artiverse_comparison_dir,
        pva=pva,
        artiverse=artiverse,
    )
    articraft_features = three._extract_articraft(
        articraft,
        pva=pva,
        output_dir=output_dir,
        batch_size=args.batch_size,
        device=args.device,
        num_workers=args.num_workers,
        use_amp=args.use_amp,
        force_extract=args.force_extract,
    )
    partnet_features = _extract_partnet(
        partnet,
        pva=pva,
        output_dir=output_dir,
        batch_size=args.batch_size,
        device=args.device,
        num_workers=args.num_workers,
        use_amp=args.use_amp,
        force_extract=args.force_extract,
    )
    source_features: dict[str, Mapping[str, np.ndarray]] = {
        "PV-A": pva.features,
        "Artiverse": artiverse_features,
        "Articraft-10K": articraft_features,
        "PartNet-Mobility": partnet_features,
    }
    counts = {
        "PV-A": len(pva.records),
        "Artiverse": len(artiverse.records),
        "Articraft-10K": len(articraft.records),
        "PartNet-Mobility": len(partnet.records),
    }

    independent_coordinates: dict[str, np.ndarray] = {}
    joint_coordinates: dict[str, np.ndarray] = {}
    tsne_info: dict[str, Any] = {}
    for encoder in ENCODERS:
        print(f"[tsne] {encoder} PartNet-Mobility n={len(partnet.records)}", flush=True)
        independent_coordinates[encoder], independent_info = _compute_tsne(
            partnet_features[encoder],
            perplexity=args.perplexity,
            random_state=args.seed,
            max_iter=args.tsne_max_iter,
            n_jobs=args.tsne_jobs,
            verbose=args.tsne_verbose,
        )
        joint_features = np.vstack([source_features[source][encoder] for source in SOURCE_ORDER])
        print(f"[tsne] {encoder} joint n={len(joint_features)}", flush=True)
        joint_coordinates[encoder], joint_info = _compute_tsne(
            joint_features,
            perplexity=args.perplexity,
            random_state=args.seed,
            max_iter=args.tsne_max_iter,
            n_jobs=args.tsne_jobs,
            verbose=args.tsne_verbose,
        )
        tsne_info[encoder] = {"independent_partnet": independent_info, "joint": joint_info}

    colors = category_colors(len(partnet.records))
    index_rows = [
        {
            "category_id": f"P{index:02d}",
            "category": record.category,
            "dataset_id": record.dataset_id,
            "ordinal": record.ordinal,
            "package_path": str(record.package_path),
            "urdf_path": str(record.urdf_path),
            "package_file_count": record.package_file_count,
            "package_total_bytes": record.package_total_bytes,
            "package_content_manifest_sha256": record.package_content_manifest_sha256,
            "package_binding_sha256": record.package_binding_sha256,
            "image_path": str(record.image_path),
            "image_bytes": record.image_bytes,
            "image_sha256": record.image_sha256,
            "color": color,
        }
        for index, (record, color) in enumerate(zip(partnet.records, colors, strict=True), start=1)
    ]
    atomic_csv(output_dir / "partnet_index.csv", index_rows)
    atomic_json(
        output_dir / "partnet_selection_summary.json",
        {
            "official_model_count": partnet.official_model_count,
            "paper_reported_model_count": 2_346,
            "frozen_full_release_manifest_model_count": partnet.official_model_count,
            "official_category_count": partnet.official_category_count,
            "rendered_category_count": len(partnet.records),
            "selection": partnet.receipts["selection"],
            "categories": list(partnet.categories),
        },
    )
    for encoder in ENCODERS:
        save_partnet_plot(
            independent_coordinates[encoder],
            partnet.categories,
            colors,
            output_dir / encoder / "partnet_one_shot_tsne.png",
            title=(
                f"PartNet-Mobility uniform {encoder}: one representative per category "
                f"(n={len(partnet.records)})"
            ),
            dpi=args.plot_dpi,
        )
        write_partnet_coordinates(
            output_dir / encoder / "partnet_one_shot_tsne_coordinates.csv",
            independent_coordinates[encoder],
            partnet.records,
        )
        atomic_json(output_dir / encoder / "partnet_tsne_manifest.json", tsne_info[encoder]["independent_partnet"])
        write_joint_coordinates(
            output_dir / encoder / "joint_four_source_tsne_coordinates.csv",
            joint_coordinates[encoder],
            pva_records=pva.records,
            artiverse_records=artiverse.records,
            articraft_records=articraft.records,
            partnet_records=partnet.records,
        )
        atomic_json(output_dir / encoder / "joint_tsne_manifest.json", tsne_info[encoder]["joint"])
    save_four_source_plot(
        joint_coordinates,
        counts,
        output_dir / "joint_four_source_comparison.png",
        dpi=args.plot_dpi,
    )

    metrics = compute_metrics(
        source_features,
        joint_coordinates,
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
            "articraft_feature_sha256": sha256_file(
                output_dir / encoder / "articraft_one_shot_features.npy"
            ),
            "articraft_feature_manifest_sha256": sha256_file(
                output_dir / encoder / "articraft_feature_manifest.json"
            ),
            "partnet_feature_sha256": sha256_file(
                output_dir / encoder / "partnet_one_shot_features.npy"
            ),
            "partnet_feature_manifest_sha256": sha256_file(
                output_dir / encoder / "partnet_feature_manifest.json"
            ),
            "model_fingerprint": pva.feature_manifests[encoder]["model_fingerprint"],
        }
        for encoder in ENCODERS
    }
    stable = {
        "schema_version": 1,
        "artifact_contract": "pva_artiverse_articraft_partnet_uniform_comparison_v1",
        "counts": {
            **counts,
            "joint_total": sum(counts.values()),
            "Articraft-10K_official_models": articraft.official_model_count,
            "Articraft-10K_official_categories": articraft.official_category_count,
            "PartNet-Mobility_official_models": partnet.official_model_count,
            "PartNet-Mobility_official_categories": partnet.official_category_count,
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
            "partnet_root": str(partnet.root),
            "partnet_dataset_root": str(partnet.dataset_root),
            "partnet_render_config_sha256": partnet.receipts["render_config"]["sha256"],
            "partnet_render_manifest_sha256": partnet.receipts["render_manifest"]["sha256"],
            "partnet_roster_sha256": partnet.receipts["category_one_shot_roster"]["sha256"],
            "partnet_dataset_manifest_sha256": partnet.receipts["dataset_manifest"]["sha256"],
            "partnet_manifest_content_sha256": partnet.receipts["release"]["manifest_content_sha256"],
            "partnet_dataset_roster_sha256": partnet.receipts["release"]["roster_sha256"],
            "partnet_candidate_pool_sha256": partnet.receipts["release"]["candidate_pool_sha256"],
            "partnet_category_mapping_sha256": partnet.receipts["release"]["category_mapping_sha256"],
            "partnet_selected_package_receipt_sha256": partnet.receipts["selected_package_receipt_sha256"],
            "partnet_ordered_image_receipt_sha256": partnet.receipts["ordered_image_receipt_sha256"],
            "source_features": source_feature_receipts,
        },
        "implementation": {
            "script_sha256": sha256_file(SCRIPT),
            "three_source_compare_script_sha256": sha256_file(THREE_SOURCE_SCRIPT),
            "two_source_compare_script_sha256": sha256_file(three.BASE_COMPARE_SCRIPT),
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
        "protocol": (
            "one identity-hash-selected representative per Articraft and PartNet category; "
            "one frozen PV-A generator image and one frozen Artiverse category representative; "
            "identical Blender studio values and frozen DINOv2/CLIP snapshots"
        ),
    }
    atomic_json(output_dir / "run_contract.json", stable)

    artifacts = [
        output_dir / "run_contract.json",
        output_dir / "partnet_index.csv",
        output_dir / "partnet_selection_summary.json",
        output_dir / "high_dimensional_metrics.json",
        output_dir / "joint_four_source_comparison.png",
    ]
    for encoder in ENCODERS:
        artifacts.extend(
            [
                output_dir / encoder / "articraft_feature_manifest.json",
                output_dir / encoder / "articraft_one_shot_features.npy",
                output_dir / encoder / "partnet_feature_manifest.json",
                output_dir / encoder / "partnet_one_shot_features.npy",
                output_dir / encoder / "partnet_one_shot_tsne.png",
                output_dir / encoder / "partnet_one_shot_tsne_coordinates.csv",
                output_dir / encoder / "partnet_tsne_manifest.json",
                output_dir / encoder / "joint_four_source_tsne_coordinates.csv",
                output_dir / encoder / "joint_tsne_manifest.json",
            ]
        )
    audit = audit_outputs(
        output_dir,
        dimensions={encoder: int(pva.features[encoder].shape[1]) for encoder in ENCODERS},
        counts=counts,
        artifact_paths=artifacts,
    )
    atomic_json(output_dir / "final_audit.json", audit)
    if not audit["pass"]:
        raise RuntimeError("four-source comparison output audit failed")

    manifest = {
        "schema_version": 1,
        "stable_run_contract": stable,
        "audit": audit,
        "partnet": {
            "config": partnet.receipts["render_config"],
            "manifest": partnet.receipts["render_manifest"],
            "roster": partnet.receipts["category_one_shot_roster"],
            "dataset_manifest": partnet.receipts["dataset_manifest"],
            "release": partnet.receipts["release"],
            "selection": partnet.receipts["selection"],
        },
        "encoders": {
            encoder: {
                "feature_dim": int(pva.features[encoder].shape[1]),
                "partnet_feature": file_receipt(
                    output_dir / encoder / "partnet_one_shot_features.npy"
                ),
                "partnet_feature_manifest": file_receipt(
                    output_dir / encoder / "partnet_feature_manifest.json"
                ),
                "tsne": tsne_info[encoder],
            }
            for encoder in ENCODERS
        },
        "interpretation_limits": [
            "The four sources have 531, 84, 244, and 46 class points; those class systems are not semantically equivalent.",
            "One representative per category has selection variance.",
            "t-SNE is visualization only; high-dimensional cosine metrics support source-gap claims.",
            "The frozen PartNet-Mobility full-release manifest contains 2,347 numeric assets although the paper reports 2,346; its file, content, roster, and selected-package receipts are authoritative here.",
            "PartNet native OBJ/MTL appearance and dataset-specific modeling conventions may contribute to source separation.",
        ],
    }
    atomic_json(output_dir / "run_manifest.json", manifest)
    return {
        "output_dir": str(output_dir),
        "counts": stable["counts"],
        "audit_pass": audit["pass"],
        "metrics": metrics,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pva-dir", type=Path, default=DEFAULT_PVA_DIR)
    parser.add_argument("--artiverse-render-root", type=Path, default=DEFAULT_ARTIVERSE_RENDER_ROOT)
    parser.add_argument("--artiverse-comparison-dir", type=Path, default=DEFAULT_ARTIVERSE_DIR)
    parser.add_argument("--articraft-render-root", type=Path, default=DEFAULT_ARTICRAFT_RENDER_ROOT)
    parser.add_argument("--partnet-render-root", type=Path, default=DEFAULT_PARTNET_RENDER_ROOT)
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
