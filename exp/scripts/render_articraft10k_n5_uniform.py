#!/usr/bin/env python3
"""Render a deterministic, variable-support five-sample Articraft panel.

The frozen Articraft-10K merged roster contains 244 categories, but seven of
those categories have fewer than five distinct assets.  This driver selects
``min(5, support)`` distinct assets per category by the audited identity hash
order.  It never pads a short category with duplicate images.  The first
ranked asset is reused from the audited one-shot render when its complete
receipt and studio contract match; all remaining assets are rendered by the
same Blender worker.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import time
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping, Sequence


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
BASE_DRIVER_PATH = REPO_ROOT / "exp/scripts/render_articraft10k_uniform.py"
DEFAULT_DATASET_MANIFEST = (
    REPO_ROOT
    / "exp/runtime/articraft_github_merged_10787_20260829/rosters/merged/full_release_manifest.json"
)
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "exp/articraft10k_uniform_n5_studio_256_v1"
DEFAULT_REUSE_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/Articraft-10K/renders/"
    "uniform244_one_per_category_studio_256_v1"
)
SAMPLES_PER_CATEGORY = 5


def _load_base_driver() -> Any:
    spec = importlib.util.spec_from_file_location("_articraft10k_uniform_base_n5", BASE_DRIVER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load Articraft base driver: {BASE_DRIVER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


BASE = _load_base_driver()


def _sha256(path: Path) -> str:
    return BASE._sha256(path)


def _canonical_sha256(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    ).hexdigest()


def _atomic_write(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{time.time_ns()}.tmp")
    try:
        temporary.write_text(payload, encoding="utf-8")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _write_json(path: Path, payload: Any) -> None:
    _atomic_write(path, json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True) + "\n")


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fields: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{time.time_ns()}.tmp")
    try:
        with temporary.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
            writer.writeheader()
            for row in rows:
                writer.writerow({field: row.get(field, "") for field in fields})
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def select_samples(
    all_items: Sequence[Any],
    *,
    output_root: Path,
    samples_per_category: int = SAMPLES_PER_CATEGORY,
) -> tuple[tuple[Any, ...], dict[str, Any]]:
    """Select distinct assets per category and assign stable class/sample IDs."""

    if samples_per_category < 1:
        raise ValueError("samples_per_category must be positive")
    grouped: dict[str, list[Any]] = defaultdict(list)
    for item in all_items:
        grouped[str(item.category)].append(item)
    categories = sorted(grouped)
    selected: list[Any] = []
    per_class: dict[str, int] = {}
    short_categories: list[dict[str, Any]] = []
    ordinal = 1
    for class_number, category in enumerate(categories, 1):
        candidates = sorted(
            grouped[category],
            key=lambda item: (str(item.identity_sha256), str(item.asset_id)),
        )
        count = min(samples_per_category, len(candidates))
        if count < 1:
            raise ValueError(f"category {category!r} has no assets")
        per_class[category] = count
        if count < samples_per_category:
            short_categories.append(
                {"class_id": f"A{class_number:04d}", "category": category, "support": len(candidates), "selected": count}
            )
        class_id = f"A{class_number:04d}"
        for sample_index, original in enumerate(candidates[:count], 1):
            output_path = output_root / category / str(original.asset_id) / "imgs" / "000.png"
            # The base worker uses these two fields for receipts and log names.
            item = replace(
                original,
                ordinal=ordinal,
                output_path=output_path,
                category_one_shot=(sample_index == 1),
            )
            # Keep wrapper metadata out of the base dataclass; attach it in the
            # parallel tuple below instead of mutating the frozen base object.
            selected.append(
                _Sampled(
                    item=item,
                    class_id=class_id,
                    class_name=category,
                    sample_index=sample_index,
                    ordinal=ordinal,
                    render_key=f"{class_id}__S{sample_index:02d}__{original.asset_id}",
                    category_support=len(candidates),
                    balanced_n5_eligible=len(candidates) >= samples_per_category,
                )
            )
            ordinal += 1
    stats = {
        "class_count": len(categories),
        "asset_count": len(selected),
        "samples_per_category_target": samples_per_category,
        "per_class_count_values": sorted(set(per_class.values())),
        "balanced_n5_eligible": sum(count >= samples_per_category for count in per_class.values()),
        "short_category_count": len(short_categories),
        "short_categories": short_categories,
    }
    return tuple(selected), stats


class _Sampled:
    __slots__ = (
        "item",
        "class_id",
        "class_name",
        "sample_index",
        "ordinal",
        "render_key",
        "category_support",
        "balanced_n5_eligible",
    )

    def __init__(
        self,
        *,
        item: Any,
        class_id: str,
        class_name: str,
        sample_index: int,
        ordinal: int,
        render_key: str,
        category_support: int,
        balanced_n5_eligible: bool,
    ) -> None:
        self.item = item
        self.class_id = class_id
        self.class_name = class_name
        self.sample_index = sample_index
        self.ordinal = ordinal
        self.render_key = render_key
        self.category_support = category_support
        self.balanced_n5_eligible = balanced_n5_eligible


ROSTER_FIELDS = [
    "ordinal",
    "render_key",
    "generator_index",
    "generator_name",
    "class_id",
    "category",
    "sample_index",
    "category_support",
    "balanced_n5_eligible",
    "source_type",
    "asset_id",
    "cohort_origin",
    "source_path",
    "urdf_path",
    "urdf_sha256",
    "package_binding_sha256",
    "identity_sha256",
    "output_path",
    "png_bytes",
    "png_sha256",
]
MANIFEST_FIELDS = ROSTER_FIELDS + [
    "gpu",
    "status",
    "elapsed_seconds",
    "started_at",
    "finished_at",
    "error",
    "renderer_result",
]


def _row(sample: _Sampled) -> dict[str, Any]:
    item = sample.item
    return {
        "ordinal": sample.ordinal,
        "render_key": sample.render_key,
        "generator_index": sample.class_id,
        "generator_name": sample.class_name,
        "class_id": sample.class_id,
        "category": sample.class_name,
        "sample_index": sample.sample_index,
        "category_support": sample.category_support,
        "balanced_n5_eligible": sample.balanced_n5_eligible,
        "source_type": "articraft10k",
        "asset_id": item.asset_id,
        "cohort_origin": item.cohort_origin,
        "source_path": str(item.source_path),
        "urdf_path": str(item.urdf_path),
        "urdf_sha256": item.urdf_sha256,
        "package_binding_sha256": item.package_binding_sha256,
        "identity_sha256": item.identity_sha256,
        "output_path": str(item.output_path),
    }


def _load_prior(path: Path) -> dict[str, dict[str, Any]]:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {str(row.get("asset_id") or ""): dict(row) for row in rows if row.get("asset_id")}


def _baseline_receipts(reuse_root: Path) -> dict[str, dict[str, Any]]:
    path = reuse_root / "render_manifest.csv"
    return _load_prior(path)


def _validate_reuse_contract(
    reuse_root: Path,
    *,
    dataset_manifest: Path,
    renderer: Path,
    shared_renderer: Path,
    resolution: int,
    samples: int,
) -> dict[str, Any]:
    """Require the reused panel to be the exact audited base contract."""

    root = reuse_root.expanduser().resolve(strict=True)
    config_path = root / "render_config.json"
    manifest_path = root / "render_manifest.csv"
    if not config_path.is_file() or not manifest_path.is_file():
        raise ValueError(f"reuse root lacks its audited receipts: {root}")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    expected = {
        "render_contract": "articraft10k_uniform_studio_v1",
        "dataset": "Articraft-10K",
        "dataset_manifest_sha256": _sha256(dataset_manifest),
        "renderer_sha256": _sha256(renderer),
        "shared_renderer_sha256": _sha256(shared_renderer),
        "resolution": resolution,
        "samples": samples,
        "studio": BASE._studio_contract(),
        "selected_count": BASE.EXPECTED_CATEGORY_COUNT,
        "selected_category_count": BASE.EXPECTED_CATEGORY_COUNT,
    }
    for field, value in expected.items():
        if config.get(field) != value:
            raise ValueError(f"reuse render contract mismatch for {field}: {config_path}")
    selection = config.get("selection")
    if not isinstance(selection, Mapping) or selection.get("one_shot_only") is not True:
        raise ValueError(f"reuse root is not the audited one-shot selection: {config_path}")
    return {
        "root": str(root),
        "render_config": str(config_path),
        "render_config_sha256": _sha256(config_path),
        "render_manifest": str(manifest_path),
        "render_manifest_sha256": _sha256(manifest_path),
    }


def _reuse_one_shot(
    sample: _Sampled,
    *,
    reuse_root: Path,
    baseline: Mapping[str, Any],
    resolution: int,
) -> dict[str, Any] | None:
    """Copy a verified rank-one image into the n5 output location."""

    if sample.sample_index != 1:
        return None
    receipt = baseline.get(str(sample.item.asset_id))
    if not receipt or str(receipt.get("status")) not in BASE.SUCCESS_STATUSES:
        return None
    source = Path(str(receipt.get("output_path") or "")).expanduser()
    if not source.is_file() or not BASE._valid_png(source, resolution):
        return None
    try:
        source.resolve(strict=True).relative_to(reuse_root.resolve(strict=True))
    except (OSError, ValueError):
        return None
    if str(receipt.get("category") or "") != sample.class_name:
        return None
    if str(receipt.get("asset_id") or "") != str(sample.item.asset_id):
        return None
    if str(receipt.get("urdf_sha256") or "") != str(sample.item.urdf_sha256):
        return None
    if str(receipt.get("package_binding_sha256") or "") != str(sample.item.package_binding_sha256):
        return None
    declared_hash = str(receipt.get("png_sha256") or "")
    if len(declared_hash) != 64 or _sha256(source) != declared_hash:
        return None
    target = sample.item.output_path
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() or target.is_symlink():
        if not BASE._valid_png(target, resolution) or _sha256(target) != declared_hash:
            return None
    else:
        try:
            os.link(source, target)
        except OSError:
            shutil.copy2(source, target)
    now = BASE._utc_now()
    result = _row(sample)
    result.update(
        {
            "status": "reused_valid",
            "elapsed_seconds": 0.0,
            "png_bytes": target.stat().st_size,
            "png_sha256": declared_hash,
            "started_at": now,
            "finished_at": now,
            "error": "",
            "renderer_result": {
                "reuse_source": str(source),
                "reuse_source_manifest": str(reuse_root / "render_manifest.csv"),
                "reuse_source_status": str(receipt.get("status")),
            },
        }
    )
    return result


def _render_one(
    sample: _Sampled,
    *,
    args: argparse.Namespace,
    gpu: str,
    prior: Mapping[str, Any] | None,
) -> dict[str, Any]:
    worker_args = copy.copy(args)
    worker_args.gpu = gpu
    result = BASE._render_one(
        sample.item,
        args=worker_args,
        blender=args.blender,
        renderer=args.renderer,
        shared_renderer=args.shared_renderer,
        shared_renderer_sha256=args.shared_renderer_sha256,
        reuse_receipt=prior,
    )
    enriched = _row(sample)
    enriched.update({key: value for key, value in result.items() if key not in enriched})
    enriched["gpu"] = gpu
    return enriched


def _build_config(
    args: argparse.Namespace,
    *,
    all_items: Sequence[Any],
    selected: Sequence[_Sampled],
    stats: Mapping[str, Any],
    reuse_receipt: Mapping[str, Any] | None,
) -> dict[str, Any]:
    manifest = args.dataset_manifest.expanduser().resolve(strict=True)
    return {
        "schema_version": 1,
        "render_contract": "articraft10k_n5_uniform_studio_v1",
        "sampling_contract": "articraft10k_category_min5_identity_hash_v1",
        "dataset": "Articraft-10K",
        "dataset_manifest": str(manifest),
        "dataset_manifest_sha256": _sha256(manifest),
        "full_model_count": len(all_items),
        "class_count": int(stats["class_count"]),
        "selected_count": len(selected),
        "selected_category_count": len({sample.class_name for sample in selected}),
        "samples_per_category_target": args.samples_per_category,
        "per_class_count_values": list(stats["per_class_count_values"]),
        "balanced_n5_eligible": int(stats["balanced_n5_eligible"]),
        "short_category_count": int(stats["short_category_count"]),
        "short_categories": list(stats["short_categories"]),
        "selection_rule": "first min(5, support) by (SHA256(asset_id UTF-8), asset_id) within category",
        "driver": str(SCRIPT),
        "driver_sha256": _sha256(SCRIPT),
        "base_driver": str(BASE_DRIVER_PATH),
        "base_driver_sha256": _sha256(BASE_DRIVER_PATH),
        "renderer": str(args.renderer),
        "renderer_sha256": _sha256(args.renderer),
        "shared_renderer": str(args.shared_renderer),
        "shared_renderer_sha256": args.shared_renderer_sha256,
        "blender": str(args.blender),
        "blender_version": BASE._blender_version(args.blender),
        "output_root": str(args.output_root),
        "reuse": dict(reuse_receipt) if reuse_receipt else None,
        "resolution": args.resolution,
        "samples": args.samples,
        "studio": BASE._studio_contract(),
        "pose_policy": "URDF rest pose; all movable joint coordinates are zero",
        "material_policy": "URDF visual rgba; fixed neutral fallback for missing colors",
        "gpu_visibility": list(args.gpus),
        "workers_per_gpu": args.workers_per_gpu,
    }


def _parse_gpus(values: Sequence[str]) -> tuple[str, ...]:
    result: list[str] = []
    for value in values:
        result.extend(part.strip() for part in str(value).split(",") if part.strip())
    if not result or any(not value.isdigit() for value in result):
        raise ValueError("--gpus requires one or more numeric GPU indices")
    if len(set(result)) != len(result):
        raise ValueError("--gpus contains duplicate indices")
    return tuple(result)


def finalize_existing(output_root: Path) -> dict[str, Any]:
    """Audit completed artifacts and bind their final hashes into the run receipts."""

    root = output_root.expanduser().resolve(strict=True)
    config_path = root / "render_config.json"
    summary_path = root / "render_summary.json"
    roster_path = root / "render_roster.csv"
    manifest_path = root / "render_manifest.csv"
    for path in (config_path, summary_path, roster_path, manifest_path):
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"finalize input is missing or unsafe: {path}")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    with roster_path.open("r", encoding="utf-8", newline="") as stream:
        roster_rows = list(csv.DictReader(stream))
    with manifest_path.open("r", encoding="utf-8", newline="") as stream:
        manifest_rows = list(csv.DictReader(stream))
    expected_count = int(summary.get("selected_count", -1))
    if expected_count < 1 or len(roster_rows) != expected_count or len(manifest_rows) != expected_count:
        raise ValueError("final roster/manifest row count differs from selected_count")
    roster_by_key = {str(row.get("render_key") or ""): row for row in roster_rows}
    manifest_by_key = {str(row.get("render_key") or ""): row for row in manifest_rows}
    if "" in roster_by_key or len(roster_by_key) != expected_count or set(roster_by_key) != set(manifest_by_key):
        raise ValueError("final roster/manifest render keys are missing, duplicated, or different")
    if len({row.get("asset_id") for row in roster_rows}) != expected_count:
        raise ValueError("final roster contains duplicate asset identities")
    if len({row.get("output_path") for row in roster_rows}) != expected_count:
        raise ValueError("final roster contains duplicate output paths")
    for render_key, roster_row in roster_by_key.items():
        manifest_row = manifest_by_key[render_key]
        for field in ROSTER_FIELDS:
            if str(roster_row.get(field) or "") != str(manifest_row.get(field) or ""):
                raise ValueError(f"final roster/manifest field mismatch: {render_key}:{field}")
        if manifest_row.get("status") not in BASE.SUCCESS_STATUSES or manifest_row.get("error"):
            raise ValueError(f"final manifest contains a failed row: {render_key}")
        output = Path(str(roster_row["output_path"])).expanduser().resolve(strict=True)
        try:
            output.relative_to(root)
        except ValueError as exc:
            raise ValueError(f"final output escapes render root: {output}") from exc
        if not BASE._valid_png(output, int(config.get("resolution", -1))):
            raise ValueError(f"final output is not a valid PNG: {output}")
        if output.stat().st_size != int(roster_row.get("png_bytes") or -1):
            raise ValueError(f"final PNG byte receipt mismatch: {output}")
        if _sha256(output) != str(roster_row.get("png_sha256") or ""):
            raise ValueError(f"final PNG hash receipt mismatch: {output}")

    roster_sha256 = _sha256(roster_path)
    manifest_sha256 = _sha256(manifest_path)
    config.update(
        {
            "driver_sha256": _sha256(SCRIPT),
            "render_roster": str(roster_path),
            "render_roster_rows": len(roster_rows),
            "render_roster_sha256": roster_sha256,
            "render_manifest": str(manifest_path),
            "render_manifest_rows": len(manifest_rows),
            "render_manifest_sha256": manifest_sha256,
        }
    )
    _write_json(config_path, config)
    config_sha256 = _sha256(config_path)
    summary.update(
        {
            "driver_sha256": config["driver_sha256"],
            "render_roster_sha256": roster_sha256,
            "render_manifest_sha256": manifest_sha256,
            "render_config_sha256": config_sha256,
            "finalized": True,
        }
    )
    _write_json(summary_path, summary)
    return summary


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.samples_per_category < 1 or args.resolution < 64 or args.samples < 1:
        raise ValueError("samples_per_category >= 1, resolution >= 64, and samples >= 1 are required")
    if args.workers_per_gpu < 1 or args.timeout_seconds <= 0:
        raise ValueError("workers_per_gpu >= 1 and timeout_seconds > 0 are required")
    args.gpus = _parse_gpus(args.gpus)
    args.output_root = args.output_root.expanduser().resolve()
    if args.finalize_only:
        return finalize_existing(args.output_root)
    args.dataset_manifest = args.dataset_manifest.expanduser().resolve(strict=True)
    args.renderer = args.renderer.expanduser().resolve(strict=True)
    args.shared_renderer = args.shared_renderer.expanduser().resolve(strict=True)
    args.blender = args.blender.expanduser().resolve(strict=True)
    args.reuse_root = args.reuse_root.expanduser().resolve() if args.reuse_root else None
    all_items = BASE.load_render_items(
        args.dataset_manifest,
        output_root=args.output_root,
        strict_counts=not args.allow_count_drift,
        validate_inputs=False,
    )
    selected, stats = select_samples(
        all_items,
        output_root=args.output_root,
        samples_per_category=args.samples_per_category,
    )
    if args.categories:
        wanted = set(args.categories)
        missing = sorted(wanted - {sample.class_name for sample in selected})
        if missing:
            raise ValueError(f"unknown --categories: {', '.join(missing)}")
        selected = tuple(sample for sample in selected if sample.class_name in wanted)
    if args.limit is not None:
        if args.limit < 1:
            raise ValueError("--limit must be positive")
        selected = selected[: args.limit]
    if not selected:
        raise ValueError("selection contains no Articraft assets")
    if args.require_five and any(value < args.samples_per_category for value in stats["per_class_count_values"]):
        raise ValueError("one or more Articraft categories have fewer than the requested samples")

    args.shared_renderer_sha256 = _sha256(args.shared_renderer)
    reuse_receipt = (
        _validate_reuse_contract(
            args.reuse_root,
            dataset_manifest=args.dataset_manifest,
            renderer=args.renderer,
            shared_renderer=args.shared_renderer,
            resolution=args.resolution,
            samples=args.samples,
        )
        if args.reuse_root
        else None
    )
    config = _build_config(
        args,
        all_items=all_items,
        selected=selected,
        stats=stats,
        reuse_receipt=reuse_receipt,
    )
    config_path = args.output_root / "render_config.json"
    if config_path.is_file() and not args.force:
        previous = json.loads(config_path.read_text(encoding="utf-8"))
        comparable = {key: value for key, value in config.items() if key not in {"gpu_visibility", "workers_per_gpu"}}
        old_comparable = {key: value for key, value in previous.items() if key not in {"gpu_visibility", "workers_per_gpu"}}
        if comparable != old_comparable:
            raise ValueError(f"output root contains a different n5 contract: {config_path}")
    else:
        if args.output_root.is_dir() and any(args.output_root.iterdir()) and not args.force:
            raise ValueError(f"non-empty output root has no matching render_config.json: {args.output_root}")
        _write_json(config_path, config)

    roster_rows = [_row(sample) for sample in selected]
    _write_csv(args.output_root / "render_roster.csv", roster_rows, ROSTER_FIELDS)

    # Validate selected package bindings before rendering, matching the base
    # driver's fail-closed release policy.
    if not args.dry_run:
        _manifest_obj, manifest_rows = BASE._load_manifest(
            args.dataset_manifest, strict_counts=not args.allow_count_drift
        )
        rows_by_id = {str(row.get("asset_id")): row for row in manifest_rows}
        for sample in selected:
            row = rows_by_id.get(str(sample.item.asset_id))
            if row is None:
                raise ValueError(f"selected asset missing from manifest: {sample.item.asset_id}")
            if BASE._validate_package_files(row, sample.item.source_path) != sample.item.package_binding_sha256:
                raise ValueError(f"selected package binding drift: {sample.item.asset_id}")
            if sample.item.urdf_path.stat().st_size != sample.item.urdf_bytes or _sha256(sample.item.urdf_path) != sample.item.urdf_sha256:
                raise ValueError(f"selected URDF receipt drift: {sample.item.asset_id}")

    if args.dry_run:
        summary = {"status": "dry_run", **stats, "selected_count": len(selected), "roster": str(args.output_root / "render_roster.csv")}
        _write_json(args.output_root / "render_summary.json", summary)
        return summary

    baseline_receipts = _baseline_receipts(args.reuse_root) if args.reuse_root else {}
    manifest_path = args.output_root / "render_manifest.csv"
    prior = _load_prior(manifest_path)
    results: list[dict[str, Any]] = []
    pending: list[_Sampled] = []
    for sample in selected:
        reused = _reuse_one_shot(
            sample,
            reuse_root=args.reuse_root,
            baseline=baseline_receipts,
            resolution=args.resolution,
        ) if args.reuse_root else None
        if reused is not None and not args.force:
            results.append(reused)
        else:
            pending.append(sample)
    print(
        f"[render] {len(selected)} selected ({len(results)} reused, {len(pending)} Blender) / "
        f"{stats['class_count']} categories -> {args.output_root}",
        flush=True,
    )
    state_path = args.output_root / "render_state.jsonl"
    with state_path.open("a", encoding="utf-8") as state_stream:
        with ThreadPoolExecutor(max_workers=args.workers_per_gpu * len(args.gpus)) as pool:
            futures = {
                pool.submit(
                    _render_one,
                    sample,
                    args=args,
                    gpu=args.gpus[index % len(args.gpus)],
                    prior=prior.get(str(sample.item.asset_id)),
                ): sample
                for index, sample in enumerate(pending)
            }
            for done, future in enumerate(as_completed(futures), 1):
                result = future.result()
                results.append(result)
                state_stream.write(json.dumps(result, sort_keys=True, ensure_ascii=True) + "\n")
                state_stream.flush()
                print(
                    f"[render] {done}/{len(pending)} {result.get('category')}/{result.get('asset_id')} {result.get('status')}",
                    flush=True,
                )
    _write_csv(manifest_path, results, MANIFEST_FIELDS)
    _write_csv(args.output_root / "render_roster.csv", results, ROSTER_FIELDS)
    failures = [result for result in results if result.get("status") not in BASE.SUCCESS_STATUSES]
    valid_count = sum(BASE._valid_png(sample.item.output_path, args.resolution) for sample in selected)
    summary = {
        "schema_version": 1,
        "render_contract": config["render_contract"],
        "sampling_contract": config["sampling_contract"],
        "dataset_manifest": str(args.dataset_manifest),
        "full_model_count": len(all_items),
        "class_count": stats["class_count"],
        "selected_count": len(selected),
        "selected_category_count": len({sample.class_name for sample in selected}),
        "samples_per_category_target": args.samples_per_category,
        "per_class_count_values": stats["per_class_count_values"],
        "balanced_n5_eligible": stats["balanced_n5_eligible"],
        "short_category_count": stats["short_category_count"],
        "short_categories": stats["short_categories"],
        "rendered_count": sum(result.get("status") == "rendered" for result in results),
        "reused_valid_count": sum(result.get("status") == "reused_valid" for result in results),
        "failure_count": len(failures),
        "selected_valid_png_count": valid_count,
        "selected_complete": valid_count == len(selected) and not failures,
        "failure_asset_ids": [str(result.get("asset_id")) for result in failures],
        "manifest": str(manifest_path),
        "roster": str(args.output_root / "render_roster.csv"),
        "config": str(config_path),
    }
    _write_json(args.output_root / "render_summary.json", summary)
    if failures:
        raise RuntimeError(f"{len(failures)} Articraft render(s) failed")
    return finalize_existing(args.output_root)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-manifest", type=Path, default=DEFAULT_DATASET_MANIFEST)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--reuse-root", type=Path, default=DEFAULT_REUSE_ROOT)
    parser.add_argument("--renderer", type=Path, default=BASE.DEFAULT_RENDERER)
    parser.add_argument("--shared-renderer", type=Path, default=BASE.DEFAULT_SHARED_RENDERER)
    parser.add_argument("--blender", type=Path, default=BASE.DEFAULT_BLENDER)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--samples-per-category", type=int, default=SAMPLES_PER_CATEGORY)
    parser.add_argument("--gpus", nargs="+", default=["6", "7"])
    parser.add_argument("--workers-per-gpu", type=int, default=2)
    parser.add_argument("--timeout-seconds", type=float, default=900.0)
    parser.add_argument("--categories", nargs="+")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--allow-count-drift", action="store_true")
    parser.add_argument("--require-five", action="store_true")
    parser.add_argument("--finalize-only", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        run(build_argument_parser().parse_args(argv))
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
