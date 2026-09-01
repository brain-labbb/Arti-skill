#!/usr/bin/env python3
"""Render up to five deterministic Artiverse assets per category.

The Artiverse release contains 3,544 models in 84 categories.  Some of the
categories contain fewer than five distinct models, so this driver selects
``min(5, support(category))`` without replacement.  The selection is ranked
by the content-independent identity hash used by the audited one-shot
renderer.  Images are rendered with exactly the same opaque studio worker as
``render_artiverse_uniform.py`` and are written below ``arti-skill/exp``.

The resulting roster is intentionally suitable for the shared feature/t-SNE
loader: every row has a class id, class name, sample index, source type,
asset id, output path, and (after rendering) PNG byte/hash receipts.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import os
import shutil
import sys
import time
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Mapping, Sequence


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
BASE_DRIVER_PATH = REPO_ROOT / "exp/scripts/render_artiverse_uniform.py"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "exp/artiverse_n5_uniform_studio_256_v1"
DEFAULT_REUSE_ROOTS = (
    Path("/mnt/zsn/data/particulate/datasets/Artiverse/renders/uniform3544_studio_256_v1"),
    Path("/mnt/zsn/data/particulate/datasets/Artiverse/renders/uniform84_one_per_category_studio_256_v1"),
)
SAMPLES_PER_CATEGORY = 5


def _load_base_driver() -> Any:
    """Load the audited driver without requiring ``exp`` to be installed."""

    spec = importlib.util.spec_from_file_location("_artiverse_uniform_base", BASE_DRIVER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load base Artiverse renderer: {BASE_DRIVER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


BASE = _load_base_driver()


@dataclass(frozen=True, slots=True)
class SelectedItem:
    """One selected asset plus its stable class/sample identity."""

    item: Any
    source_ordinal: int
    class_id: str
    sample_index: int
    render_key: str

    @property
    def category(self) -> str:
        return str(self.item.category)

    @property
    def asset_id(self) -> str:
        return str(self.item.model_id)


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _atomic_write_text(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{time.time_ns()}.tmp")
    try:
        temporary.write_text(payload, encoding="utf-8")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _write_json(path: Path, payload: Any) -> None:
    _atomic_write_text(path, json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True) + "\n")


def _write_csv(path: Path, fields: Sequence[str], rows: Sequence[Mapping[str, Any]]) -> None:
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


def _class_index(items: Sequence[Any]) -> dict[str, str]:
    categories = sorted({str(item.category) for item in items})
    return {category: f"C{index:03d}" for index, category in enumerate(categories, start=1)}


def select_items(
    items: Sequence[Any],
    *,
    output_root: Path,
    samples_per_category: int = SAMPLES_PER_CATEGORY,
    balanced_only: bool = False,
) -> tuple[SelectedItem, ...]:
    """Select a deterministic, no-replacement panel from the full roster."""

    if samples_per_category < 1:
        raise ValueError("samples_per_category must be positive")
    classes = _class_index(items)
    grouped: dict[str, list[Any]] = defaultdict(list)
    for item in items:
        grouped[str(item.category)].append(item)

    selected: list[SelectedItem] = []
    ordinal = 0
    for category in sorted(grouped):
        candidates = sorted(
            grouped[category],
            key=lambda item: (str(item.identity_sha256), str(item.manifest_root)),
        )
        if balanced_only and len(candidates) < samples_per_category:
            continue
        count = min(samples_per_category, len(candidates))
        for sample_index, original in enumerate(candidates[:count], start=1):
            ordinal += 1
            output = (
                output_root
                / category
                / str(original.source)
                / str(original.model_id)
                / "imgs"
                / "000.png"
            )
            # The base worker uses ordinal for logs and receipts.  Re-number
            # the selected roster contiguously while retaining source_ordinal.
            cloned = replace(original, ordinal=ordinal, output_path=output)
            class_id = classes[category]
            render_key = (
                f"{class_id}__S{sample_index:02d}__{category}__"
                f"{original.source}__{original.model_id}"
            )
            selected.append(
                SelectedItem(
                    item=cloned,
                    source_ordinal=int(original.ordinal),
                    class_id=class_id,
                    sample_index=sample_index,
                    render_key=render_key,
                )
            )
    if len({entry.render_key for entry in selected}) != len(selected):
        raise ValueError("selection produced duplicate render keys")
    if len({entry.item.output_path for entry in selected}) != len(selected):
        raise ValueError("selection produced duplicate output paths")
    return tuple(selected)


ROSTER_FIELDS = [
    "ordinal",
    "render_key",
    "generator_index",
    "generator_name",
    "sample_index",
    "source_type",
    "asset_id",
    "output_path",
    "png_bytes",
    "png_sha256",
    # Provenance fields retained for source/audit checks.
    "source_ordinal",
    "source",
    "manifest_root",
    "identity_sha256",
    "glb_path",
    "glb_bytes",
    "glb_sha256",
]

RESULT_FIELDS = ROSTER_FIELDS + [
    "status",
    "elapsed_seconds",
    "started_at",
    "finished_at",
    "error",
    "imported_cameras_removed",
    "imported_lights_removed",
    "renderer_result",
    "reuse_source_path",
    "reuse_source_manifest",
]


def _entry_row(entry: SelectedItem, *, png_bytes: Any = "", png_sha256: Any = "") -> dict[str, Any]:
    item = entry.item
    return {
        "ordinal": entry.item.ordinal,
        "render_key": entry.render_key,
        "generator_index": entry.class_id,
        "generator_name": entry.category,
        "sample_index": entry.sample_index,
        "source_type": str(item.source),
        "asset_id": entry.asset_id,
        "output_path": str(item.output_path),
        "png_bytes": png_bytes,
        "png_sha256": png_sha256,
        "source_ordinal": entry.source_ordinal,
        "source": str(item.source),
        "manifest_root": str(item.manifest_root),
        "identity_sha256": str(item.identity_sha256),
        "glb_path": str(item.glb_path),
        "glb_bytes": "",
        "glb_sha256": "",
    }


def _input_receipt(selected: Sequence[SelectedItem]) -> tuple[dict[str, Any], dict[str, tuple[int, str]]]:
    base_items = [entry.item for entry in selected]
    return BASE._input_receipt(base_items)


def _shortfall_summary(items: Sequence[Any], samples_per_category: int) -> tuple[dict[str, int], int]:
    counts = Counter(str(item.category) for item in items)
    shortfalls = {
        category: count
        for category, count in sorted(counts.items())
        if count < samples_per_category
    }
    eligible = sum(count >= samples_per_category for count in counts.values())
    return shortfalls, eligible


def _build_config(
    args: argparse.Namespace,
    *,
    all_items: Sequence[Any],
    selected: Sequence[SelectedItem],
    renderer: Path,
    shared_renderer: Path,
    blender: Path,
    input_receipt: Mapping[str, Any],
    roster_path: Path,
) -> dict[str, Any]:
    counts = Counter(entry.category for entry in selected)
    shortfalls, eligible = _shortfall_summary(all_items, args.samples_per_category)
    return {
        "schema_version": 1,
        "render_contract": "artiverse_n5_uniform_studio_v1",
        "sampling_contract": "artiverse_category_hash_rank_max5_v1",
        "dataset": "Artiverse",
        "dataset_manifest": str(args.dataset_manifest.expanduser().resolve(strict=True)),
        "dataset_manifest_sha256": BASE._sha256(args.dataset_manifest.expanduser().resolve(strict=True)),
        "data_root": str(args.data_root.expanduser().resolve(strict=True)),
        "output_root": str(args.output_root),
        "full_model_count": len(all_items),
        "full_category_count": len({str(item.category) for item in all_items}),
        "class_count": len({entry.category for entry in selected}),
        "selected_count": len(selected),
        "selected_category_count": len(counts),
        "per_class_target": args.samples_per_category,
        "per_class_count_values": sorted(set(counts.values())),
        "balanced_n5_eligible": eligible,
        "shortfall_categories": shortfalls,
        "balanced_only": bool(args.balanced_only),
        "selection_rule": "first min(5, support(category)) by (SHA256(manifest_root UTF-8), manifest_root)",
        "render_roster": str(roster_path),
        "render_roster_sha256": BASE._sha256(roster_path),
        "driver": str(SCRIPT),
        "driver_sha256": BASE._sha256(SCRIPT),
        "base_driver": str(BASE_DRIVER_PATH),
        "base_driver_sha256": BASE._sha256(BASE_DRIVER_PATH),
        "renderer": str(renderer),
        "renderer_sha256": BASE._sha256(renderer),
        "shared_renderer": str(shared_renderer),
        "shared_renderer_sha256": BASE._sha256(shared_renderer),
        "blender": str(blender),
        "blender_version": BASE._blender_version(blender),
        "input_receipt": dict(input_receipt),
        "resolution": args.resolution,
        "samples": args.samples,
        "studio": BASE._studio_contract(),
        "pose_policy": "canonical transforms embedded in segmented.glb",
        "material_policy": "native glTF materials and textures; imported cameras and lights removed",
        "image_layout": "category/source/model_id/imgs/000.png",
        "gpu_visibility": list(args.gpus),
        "workers_per_gpu": args.workers_per_gpu,
        "timeout_seconds": args.timeout_seconds,
        "reuse_roots": [str(path.expanduser().resolve()) for path in args.reuse_root],
    }


def _stable_config(config: Mapping[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in config.items()
        if key
        not in {
            "gpu_visibility",
            "workers_per_gpu",
            "timeout_seconds",
            "blender_version",
            "render_roster_sha256",
        }
    }


def _write_roster(path: Path, selected: Sequence[SelectedItem], receipts: Mapping[str, tuple[int, str]]) -> None:
    rows: list[dict[str, Any]] = []
    for entry in selected:
        row = _entry_row(entry)
        glb_bytes, glb_sha256 = receipts[str(entry.item.manifest_root)]
        row.update({"glb_bytes": glb_bytes, "glb_sha256": glb_sha256})
        rows.append(row)
    _write_csv(path, ROSTER_FIELDS, rows)


def _valid_receipt_png(path: Path, row: Mapping[str, Any], resolution: int) -> bool:
    try:
        declared_bytes = int(row.get("png_bytes") or -1)
    except (TypeError, ValueError):
        return False
    declared_hash = str(row.get("png_sha256") or "")
    if not path.is_file() or not BASE._valid_png(path, resolution):
        return False
    if path.stat().st_size != declared_bytes or len(declared_hash) != 64:
        return False
    return BASE._sha256(path) == declared_hash


def _read_external_reuse(
    roots: Sequence[Path],
    *,
    selected: Sequence[SelectedItem],
    renderer: Path,
    shared_renderer: Path,
    resolution: int,
    samples: int,
) -> dict[str, dict[str, Any]]:
    """Read old same-contract manifests and return valid rows by manifest root."""

    selected_by_root = {str(entry.item.manifest_root): entry for entry in selected}
    found: dict[str, dict[str, Any]] = {}
    expected_studio = BASE._studio_contract()
    for raw_root in roots:
        root = raw_root.expanduser().resolve()
        config_path = root / "render_config.json"
        manifest_path = root / "render_manifest.csv"
        if not config_path.is_file() or not manifest_path.is_file():
            continue
        try:
            config = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        # A copied PNG is only valid for this run when the entire studio
        # contract and both renderer receipts match.
        if (
            config.get("studio") != expected_studio
            or str(config.get("renderer_sha256") or "") != BASE._sha256(renderer)
            or str(config.get("shared_renderer_sha256") or "") != BASE._sha256(shared_renderer)
            or int(config.get("resolution", -1)) != resolution
            or int(config.get("samples", -1)) != samples
        ):
            continue
        try:
            rows = list(csv.DictReader(manifest_path.open("r", encoding="utf-8", newline="")))
        except (OSError, csv.Error):
            continue
        for row in rows:
            identity = str(row.get("manifest_root") or "")
            entry = selected_by_root.get(identity)
            if entry is None or identity in found or row.get("status") not in BASE.SUCCESS_STATUSES:
                continue
            if any(
                str(row.get(field) or "") != expected
                for field, expected in (
                    ("category", entry.category),
                    ("source", str(entry.item.source)),
                    ("model_id", entry.asset_id),
                    ("identity_sha256", str(entry.item.identity_sha256)),
                )
            ):
                continue
            try:
                source_path = Path(str(row.get("output_path") or "")).expanduser().resolve()
                source_path.relative_to(root)
            except (OSError, ValueError):
                continue
            if not _valid_receipt_png(source_path, row, resolution):
                continue
            candidate = dict(row)
            candidate["reuse_source_path"] = str(source_path)
            candidate["reuse_source_manifest"] = str(manifest_path)
            found[identity] = candidate
    return found


def _copy_reused(
    entry: SelectedItem,
    source_row: Mapping[str, Any],
    *,
    input_receipt: tuple[int, str],
    output_root: Path,
    resolution: int,
) -> dict[str, Any] | None:
    """Copy a verified old-contract PNG atomically into this output root."""

    source = Path(str(source_row.get("reuse_source_path") or "")).expanduser().resolve()
    target = entry.item.output_path.resolve()
    expected_glb_bytes, expected_glb_sha = input_receipt
    if str(source_row.get("glb_sha256") or "") not in {"", expected_glb_sha}:
        return None
    try:
        target.relative_to(output_root.resolve())
        source.stat()
    except (OSError, ValueError):
        return None
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.{os.getpid()}.{time.time_ns()}.tmp")
    try:
        shutil.copyfile(source, temporary)
        if not BASE._valid_png(temporary, resolution):
            temporary.unlink(missing_ok=True)
            return None
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
    result = {
        **_entry_row(entry, png_bytes=target.stat().st_size, png_sha256=BASE._sha256(target)),
        "status": "reused_valid",
        "elapsed_seconds": 0.0,
        "started_at": BASE._utc_now(),
        "finished_at": BASE._utc_now(),
        "error": "",
        "imported_cameras_removed": source_row.get("imported_cameras_removed", 0),
        "imported_lights_removed": source_row.get("imported_lights_removed", 0),
        "renderer_result": None,
        "reuse_source_path": str(source),
        "reuse_source_manifest": str(source_row.get("reuse_source_manifest") or ""),
    }
    result.update({"glb_bytes": expected_glb_bytes, "glb_sha256": expected_glb_sha})
    return result


def _read_prior(path: Path, selected: Sequence[SelectedItem]) -> dict[str, dict[str, Any]]:
    if not path.is_file():
        return {}
    selected_keys = {entry.render_key for entry in selected}
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    prior: dict[str, dict[str, Any]] = {}
    for row in rows:
        key = str(row.get("render_key") or "")
        if key in selected_keys:
            if key in prior:
                raise ValueError(f"duplicate prior render receipt: {key}")
            prior[key] = dict(row)
    return prior


def _enrich_result(entry: SelectedItem, raw: Mapping[str, Any]) -> dict[str, Any]:
    output = entry.item.output_path
    row = _entry_row(
        entry,
        png_bytes=raw.get("png_bytes", output.stat().st_size if output.is_file() else 0),
        png_sha256=raw.get("png_sha256", BASE._sha256(output) if output.is_file() else ""),
    )
    row.update(
        {
            "glb_bytes": raw.get("glb_bytes", ""),
            "glb_sha256": raw.get("glb_sha256", ""),
            "status": raw.get("status", "failed"),
            "elapsed_seconds": raw.get("elapsed_seconds", 0),
            "started_at": raw.get("started_at", ""),
            "finished_at": raw.get("finished_at", ""),
            "error": raw.get("error", ""),
            "imported_cameras_removed": raw.get("imported_cameras_removed", 0),
            "imported_lights_removed": raw.get("imported_lights_removed", 0),
            "renderer_result": raw.get("renderer_result"),
            "reuse_source_path": raw.get("reuse_source_path", ""),
            "reuse_source_manifest": raw.get("reuse_source_manifest", ""),
        }
    )
    return row


def _render_selected(
    entry: SelectedItem,
    *,
    args: argparse.Namespace,
    renderer: Path,
    blender: Path,
    input_receipt: tuple[int, str],
    prior: Mapping[str, Any] | None,
    gpu: str,
) -> dict[str, Any]:
    worker_args = argparse.Namespace(**vars(args))
    worker_args.gpu = gpu
    raw = BASE._render_one(
        entry.item,
        args=worker_args,
        blender=blender,
        renderer=renderer,
        input_receipt=input_receipt,
        reuse_receipt=dict(prior) if prior else None,
    )
    return _enrich_result(entry, raw)


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.samples_per_category < 1 or args.resolution < 64 or args.samples < 1:
        raise ValueError("samples_per_category >= 1, resolution >= 64, and samples >= 1 are required")
    if args.workers_per_gpu < 1 or args.timeout_seconds <= 0:
        raise ValueError("workers_per_gpu >= 1 and timeout_seconds > 0 are required")
    if not args.gpus or any(not str(gpu).isdigit() for gpu in args.gpus):
        raise ValueError("--gpus requires one or more numeric GPU indices")
    if len(set(args.gpus)) != len(args.gpus):
        raise ValueError("--gpus contains duplicate indices")

    args.output_root = args.output_root.expanduser().resolve()
    manifest = args.dataset_manifest.expanduser().resolve(strict=True)
    data_root = args.data_root.expanduser().resolve(strict=True)
    renderer = args.renderer.expanduser().resolve(strict=True)
    shared_renderer = args.shared_renderer.expanduser().resolve(strict=True)
    blender = args.blender.expanduser().resolve(strict=True)
    all_items = BASE.load_render_items(
        manifest,
        data_root=data_root,
        output_root=args.output_root,
        strict_counts=not args.allow_count_drift,
    )
    selected = select_items(
        all_items,
        output_root=args.output_root,
        samples_per_category=args.samples_per_category,
        balanced_only=args.balanced_only,
    )
    if args.categories:
        wanted = set(args.categories)
        known = {entry.category for entry in selected}
        missing = sorted(wanted - known)
        if missing:
            raise ValueError(f"unknown --categories: {', '.join(missing)}")
        selected = tuple(entry for entry in selected if entry.category in wanted)
    if args.limit is not None:
        if args.limit < 1:
            raise ValueError("--limit must be positive")
        selected = selected[: args.limit]
    if not selected:
        raise ValueError("selection contains no Artiverse assets")

    args.output_root.mkdir(parents=True, exist_ok=True)
    (args.output_root / "logs").mkdir(exist_ok=True)
    input_receipt, per_item_receipts = _input_receipt(selected)
    roster_path = args.output_root / "render_roster.csv"
    _write_roster(roster_path, selected, per_item_receipts)
    config = _build_config(
        args,
        all_items=all_items,
        selected=selected,
        renderer=renderer,
        shared_renderer=shared_renderer,
        blender=blender,
        input_receipt=input_receipt,
        roster_path=roster_path,
    )
    config_path = args.output_root / "render_config.json"
    if config_path.is_file() and not args.force:
        previous = json.loads(config_path.read_text(encoding="utf-8"))
        if _stable_config(previous) != _stable_config(config):
            raise ValueError(f"existing render_config.json does not match requested run: {config_path}")
    else:
        _write_json(config_path, config)

    if args.dry_run:
        return {"status": "dry_run", "config": config}

    manifest_path = args.output_root / "render_manifest.csv"
    prior = _read_prior(manifest_path, selected)
    results_by_key: dict[str, dict[str, Any]] = {}
    # First reuse rows from an earlier invocation in this output root.
    if not args.force:
        by_key = {entry.render_key: entry for entry in selected}
        for key, row in prior.items():
            entry = by_key[key]
            expected = per_item_receipts[str(entry.item.manifest_root)]
            if (
                row.get("status") in BASE.SUCCESS_STATUSES
                and str(row.get("output_path") or "") == str(entry.item.output_path)
                and str(row.get("glb_sha256") or "") in {"", expected[1]}
                and _valid_receipt_png(entry.item.output_path, row, args.resolution)
            ):
                results_by_key[key] = dict(row)

        external = _read_external_reuse(
            args.reuse_root,
            selected=selected,
            renderer=renderer,
            shared_renderer=shared_renderer,
            resolution=args.resolution,
            samples=args.samples,
        )
        for entry in selected:
            if entry.render_key in results_by_key:
                continue
            old = external.get(str(entry.item.manifest_root))
            if old is None:
                continue
            copied = _copy_reused(
                entry,
                old,
                input_receipt=per_item_receipts[str(entry.item.manifest_root)],
                output_root=args.output_root,
                resolution=args.resolution,
            )
            if copied is not None:
                results_by_key[entry.render_key] = copied

    pending = [entry for entry in selected if entry.render_key not in results_by_key]
    print(
        f"[render] {len(selected)} selected ({len({entry.category for entry in selected})} categories); "
        f"{len(results_by_key)} reused, {len(pending)} to render -> {args.output_root}",
        flush=True,
    )
    with ThreadPoolExecutor(max_workers=len(args.gpus) * args.workers_per_gpu) as pool:
        futures = {
            pool.submit(
                _render_selected,
                entry,
                args=args,
                renderer=renderer,
                blender=blender,
                input_receipt=per_item_receipts[str(entry.item.manifest_root)],
                prior=None,
                gpu=args.gpus[index % len(args.gpus)],
            ): entry
            for index, entry in enumerate(pending)
        }
        for done, future in enumerate(as_completed(futures), start=1):
            entry = futures[future]
            result = future.result()
            results_by_key[entry.render_key] = result
            _write_csv(
                manifest_path,
                RESULT_FIELDS,
                [results_by_key[key] for key in sorted(results_by_key, key=lambda key: int(results_by_key[key]["ordinal"]))],
            )
            print(
                f"[render] {done}/{len(pending)} {entry.render_key} {result['status']} "
                f"({result.get('elapsed_seconds', 0)}s)",
                flush=True,
            )

    ordered_results = [
        results_by_key[entry.render_key]
        for entry in sorted(selected, key=lambda value: int(value.item.ordinal))
        if entry.render_key in results_by_key
    ]
    _write_csv(manifest_path, RESULT_FIELDS, ordered_results)
    # Publish the final feature-loader-facing roster with complete PNG
    # receipts, then refresh its content hash in the run contract.
    _write_csv(roster_path, ROSTER_FIELDS, ordered_results)
    config["render_roster_sha256"] = BASE._sha256(roster_path)
    _write_json(config_path, config)
    failures = [row for row in ordered_results if row.get("status") not in BASE.SUCCESS_STATUSES]
    valid_count = sum(BASE._valid_png(entry.item.output_path, args.resolution) for entry in selected)
    per_class_valid = Counter(
        entry.category for entry in selected if BASE._valid_png(entry.item.output_path, args.resolution)
    )
    counts = Counter(entry.category for entry in selected)
    shortfalls, eligible = _shortfall_summary(all_items, args.samples_per_category)
    summary = {
        "schema_version": 1,
        "render_contract": "artiverse_n5_uniform_studio_v1",
        "sampling_contract": config["sampling_contract"],
        "full_model_count": len(all_items),
        "full_category_count": len({str(item.category) for item in all_items}),
        "class_count": len(counts),
        "selected_count": len(selected),
        "selected_category_count": len(counts),
        "per_class_target": args.samples_per_category,
        "per_class_count_values": sorted(set(counts.values())),
        "balanced_n5_eligible": eligible,
        "shortfall_categories": shortfalls,
        "rendered_count": sum(row.get("status") == "rendered" for row in ordered_results),
        "reused_valid_count": sum(row.get("status") == "reused_valid" for row in ordered_results),
        "failure_count": len(failures),
        "valid_png_count": valid_count,
        "categories_with_target_valid": sum(
            count == args.samples_per_category for count in per_class_valid.values()
        ),
        "categories_with_any_valid": len(per_class_valid),
        "manifest": str(manifest_path),
        "roster": str(roster_path),
        "config": str(config_path),
        "failure_render_keys": [str(row.get("render_key")) for row in failures],
    }
    _write_json(args.output_root / "render_summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    if failures:
        raise RuntimeError(f"{len(failures)} Artiverse render(s) failed")
    return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-manifest", type=Path, default=BASE.DEFAULT_DATASET_MANIFEST)
    parser.add_argument("--data-root", type=Path, default=BASE.DEFAULT_DATA_ROOT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--renderer", type=Path, default=BASE.DEFAULT_RENDERER)
    parser.add_argument("--shared-renderer", type=Path, default=BASE.DEFAULT_SHARED_RENDERER)
    parser.add_argument("--blender", type=Path, default=BASE.DEFAULT_BLENDER)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--samples-per-category", type=int, default=SAMPLES_PER_CATEGORY)
    parser.add_argument("--gpus", nargs="+", default=["0", "1"])
    parser.add_argument("--workers-per-gpu", type=int, default=2)
    parser.add_argument("--timeout-seconds", type=float, default=900.0)
    parser.add_argument("--categories", nargs="+")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--reuse-root", type=Path, action="append", default=list(DEFAULT_REUSE_ROOTS))
    parser.add_argument("--balanced-only", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--allow-count-drift", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        run(build_argument_parser().parse_args(argv))
    except (OSError, ValueError, RuntimeError, TypeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
