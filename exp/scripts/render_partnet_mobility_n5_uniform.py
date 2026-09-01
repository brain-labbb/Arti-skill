#!/usr/bin/env python3
"""Render five deterministic PartNet-Mobility assets per category.

The full-release roster and the audited one-shot renderer define the source
contract.  This driver only changes the category sampling cardinality: assets
within each category are ranked by ``(SHA256(asset_id UTF-8), asset_id)`` and
the first five are rendered with the existing uniform studio renderer.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping, Sequence


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
BASE_DRIVER_PATH = REPO_ROOT / "exp/scripts/render_partnet_mobility_uniform.py"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "exp/partnet_mobility_uniform_n5_studio_256_v1"
SAMPLES_PER_CATEGORY = 5


def _load_base_driver() -> Any:
    spec = importlib.util.spec_from_file_location("_partnet_uniform_base", BASE_DRIVER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load base PartNet renderer: {BASE_DRIVER_PATH}")
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolves the module name through sys.modules while creating
    # the base RenderItem type.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


BASE = _load_base_driver()


def _canonical_sha256(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    ).hexdigest()


def _select_five(items: Sequence[Any], *, samples_per_category: int = SAMPLES_PER_CATEGORY) -> tuple[Any, ...]:
    """Select a deterministic, balanced category panel from a full roster."""

    if samples_per_category < 1:
        raise ValueError("samples_per_category must be positive")
    by_category: dict[str, list[Any]] = {}
    for item in items:
        by_category.setdefault(str(item.category), []).append(item)
    selected: list[Any] = []
    for category in sorted(by_category):
        candidates = sorted(
            by_category[category], key=lambda item: (str(item.identity_sha256), str(item.asset_id))
        )
        if len(candidates) < samples_per_category:
            raise ValueError(
                f"category {category!r} has {len(candidates)} assets; "
                f"cannot select {samples_per_category}"
            )
        for sample_index, item in enumerate(candidates[:samples_per_category], 1):
            # Keep the original asset identity and source bindings while giving
            # each sampled asset a unique output path and ordinal for receipts.
            output_path = (
                Path(item.output_path).parents[3]
                / category
                / str(item.asset_id)
                / "imgs"
                / "000.png"
            )
            # The base loader's output root is replaced below by ``output_root``;
            # use a private marker here and rewrite it in ``with_output_root``.
            selected.append((item, sample_index, output_path))
    return tuple(selected)


def _renderability_error(item: Any) -> str:
    """Return a deterministic structural error before invoking Blender."""

    root = Path(item.source_path).expanduser().resolve(strict=True)
    try:
        robot = ET.parse(item.urdf_path).getroot()
    except (ET.ParseError, OSError) as exc:
        return f"cannot parse mobility.urdf: {exc}"
    mesh_count = 0
    for mesh in robot.findall(".//visual/geometry/mesh"):
        mesh_count += 1
        filename = str(mesh.get("filename") or "").strip()
        if not filename or "\\" in filename:
            return f"invalid mesh filename: {filename!r}"
        relative = Path(filename)
        if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
            return f"uncontained mesh filename: {filename!r}"
        candidate = root / relative
        try:
            resolved = candidate.resolve(strict=True)
            resolved.relative_to(root)
        except (OSError, ValueError):
            return f"missing or uncontained mesh: {filename}"
        if not resolved.is_file() or resolved.suffix.lower() != ".obj":
            return f"mesh is not a regular OBJ: {filename}"
    if mesh_count == 0 and not robot.findall(".//visual/geometry/*"):
        return "URDF has no visual geometry"
    return ""


def _select_renderable_five(
    items: Sequence[Any], *, samples_per_category: int = SAMPLES_PER_CATEGORY
) -> tuple[tuple[Any, ...], tuple[dict[str, str], ...]]:
    """Hash-rank assets after a deterministic, outcome-independent preflight."""

    if samples_per_category < 1:
        raise ValueError("samples_per_category must be positive")
    by_category: dict[str, list[Any]] = {}
    for item in items:
        by_category.setdefault(str(item.category), []).append(item)
    selected: list[tuple[Any, int, Path]] = []
    exclusions: list[dict[str, str]] = []
    for category in sorted(by_category):
        candidates = sorted(
            by_category[category], key=lambda item: (str(item.identity_sha256), str(item.asset_id))
        )
        accepted: list[Any] = []
        for item in candidates:
            error = _renderability_error(item)
            if error:
                exclusions.append(
                    {"category": category, "asset_id": str(item.asset_id), "reason": error}
                )
                continue
            accepted.append(item)
            if len(accepted) == samples_per_category:
                break
        if len(accepted) < samples_per_category:
            raise ValueError(
                f"category {category!r} has only {len(accepted)} structurally renderable assets; "
                f"cannot select {samples_per_category}"
            )
        for sample_index, item in enumerate(accepted, 1):
            selected.append((item, sample_index, Path(item.output_path)))
    return tuple(selected), tuple(exclusions)


def with_output_root(item: Any, output_root: Path) -> Any:
    """Clone a base RenderItem with the n5 output location."""

    output = output_root / str(item.category) / str(item.asset_id) / "imgs" / "000.png"
    return replace(item, output_path=output)


def _materialize_panel(
    selected: Sequence[tuple[Any, int, Path]], output_root: Path
) -> tuple[tuple[tuple[Any, int], ...], dict[str, int]]:
    """Assign contiguous 1-based panel ordinals and retain source ordinals."""

    materialized: list[tuple[Any, int]] = []
    source_ordinals: dict[str, int] = {}
    for panel_ordinal, (original, rank, _path) in enumerate(selected, 1):
        source_ordinals[str(original.asset_id)] = int(original.ordinal)
        cloned = replace(
            with_output_root(original, output_root), ordinal=panel_ordinal
        )
        materialized.append((cloned, rank))
    return tuple(materialized), source_ordinals


def _sample_rows(selected: Sequence[tuple[Any, int, Path]], output_root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for item, sample_index, _ in selected:
        cloned = with_output_root(item, output_root)
        row = BASE._item_row(cloned)
        row.update(
            {
                "sample_index": sample_index,
                "sample_key": f"{item.category}:{sample_index}",
                "selection_rank": f"{item.identity_sha256}:{item.asset_id}",
            }
        )
        rows.append(row)
    return rows


def _common_row(
    item: Any,
    *,
    ordinal: int,
    source_ordinal: int,
    sample_index: int,
    category_index: int,
) -> dict[str, Any]:
    """Return the shared n5 roster fields consumed by the t-SNE loader."""

    row = BASE._item_row(item)
    category = str(item.category)
    asset_id = str(item.asset_id)
    row.update(
        {
            "ordinal": ordinal,
            "source_ordinal": source_ordinal,
            "render_key": f"partnet_mobility:{category}:{asset_id}:{sample_index}",
            "generator_index": category_index,
            "class_id": category_index,
            "generator_name": category,
            "source_type": "partnet_mobility",
            "sample_index": sample_index,
            "sample_key": f"{category}:{sample_index}",
            "selection_rank": f"{item.identity_sha256}:{asset_id}",
        }
    )
    return row


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fields: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{time.time_ns()}.tmp")
    try:
        with temporary.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
            writer.writeheader()
            normalized = []
            for row in rows:
                output: dict[str, Any] = {}
                for field in fields:
                    value = row.get(field, "")
                    if isinstance(value, (dict, list)):
                        value = json.dumps(value, sort_keys=True, ensure_ascii=True)
                    output[field] = value
                normalized.append(output)
            writer.writerows(normalized)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{time.time_ns()}.tmp")
    try:
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True) + "\n", encoding="utf-8")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _read_prior(path: Path) -> dict[str, dict[str, Any]]:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    result: dict[str, dict[str, Any]] = {}
    for row in rows:
        key = str(row.get("asset_id") or "")
        if key:
            result[key] = dict(row)
    return result


def _render_one(
    item: Any,
    *,
    sample_index: int,
    category_index: int,
    source_ordinal: int,
    args: argparse.Namespace,
    renderer: Path,
    base_renderer: Path,
    shared_renderer: Path,
    blender: Path,
    prior: Mapping[str, Any] | None,
) -> dict[str, Any]:
    # The audited base worker already validates the two support-renderer
    # receipts and atomically installs a valid PNG.
    result = BASE._render_one(
        item,
        args=args,
        blender=blender,
        renderer=renderer,
        base_renderer=base_renderer,
        base_renderer_sha256=str(args.base_renderer_sha256),
        shared_renderer=shared_renderer,
        shared_renderer_sha256=str(args.shared_renderer_sha256),
        reuse_receipt=prior,
    )
    result.update(
        {
            "source_ordinal": source_ordinal,
            "render_key": f"partnet_mobility:{item.category}:{item.asset_id}:{sample_index}",
            "generator_index": category_index,
            "class_id": category_index,
            "generator_name": str(item.category),
            "source_type": "partnet_mobility",
            "sample_index": sample_index,
            "sample_key": f"{item.category}:{sample_index}",
            "selection_rank": f"{item.identity_sha256}:{item.asset_id}",
        }
    )
    return result


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.samples_per_category < 1 or args.resolution < 64 or args.samples < 1:
        raise ValueError("samples_per_category >= 1, resolution >= 64, and samples >= 1 are required")
    if args.workers < 1 or args.timeout_seconds <= 0:
        raise ValueError("workers >= 1 and timeout_seconds > 0 are required")
    output_root = args.output_root.expanduser().resolve()
    args.output_root = output_root
    manifest = args.dataset_manifest.expanduser().resolve(strict=True)
    renderer = args.renderer.expanduser().resolve(strict=True)
    base_renderer = args.base_renderer.expanduser().resolve(strict=True)
    shared_renderer = args.shared_renderer.expanduser().resolve(strict=True)
    blender = args.blender.expanduser().resolve(strict=True)

    all_items = BASE.load_render_items(
        manifest,
        output_root=output_root,
        strict_counts=not args.allow_count_drift,
        # Validate the selected five per class below.  Hashing all 2,347
        # packages here would add substantial startup cost without changing
        # the frozen selection contract.
        validate_inputs=False,
    )
    selected_raw, preflight_exclusions = _select_renderable_five(
        all_items, samples_per_category=args.samples_per_category
    )
    categories_sorted = sorted({str(item.category) for item, _rank, _path in selected_raw})
    category_indices = {name: index for index, name in enumerate(categories_sorted)}
    indexed, source_ordinals = _materialize_panel(selected_raw, output_root)
    if args.categories:
        wanted = set(args.categories)
        known = {str(item.category) for item, _rank in indexed}
        missing = sorted(wanted - known)
        if missing:
            raise ValueError(f"unknown --categories: {', '.join(missing)}")
        indexed = tuple((item, rank) for item, rank in indexed if str(item.category) in wanted)
    if args.limit is not None:
        if args.limit < 1:
            raise ValueError("--limit must be positive")
        indexed = indexed[: args.limit]
    if not indexed:
        raise ValueError("selection contains no assets")

    frozen_manifest, frozen_rows = BASE._load_manifest(
        manifest, strict_counts=not args.allow_count_drift
    )
    if not args.skip_input_validation:
        rows_by_id = {str(row.get("asset_id")): row for row in frozen_rows}
        for item, _rank in indexed:
            actual_binding = BASE._validate_package_files(rows_by_id[str(item.asset_id)], item.source_path)
            if actual_binding != item.package_binding_sha256:
                raise ValueError(f"selected package binding drift: {item.asset_id}")
            if (
                item.urdf_path.stat().st_size != item.urdf_bytes
                or BASE._sha256(item.urdf_path) != item.urdf_sha256
            ):
                raise ValueError(f"selected URDF receipt drift: {item.asset_id}")

    args.base_renderer_sha256 = BASE._sha256(base_renderer)
    args.shared_renderer_sha256 = BASE._sha256(shared_renderer)
    config = {
        "schema_version": 1,
        "render_contract": "partnet_mobility_uniform_studio_v1",
        "sampling_contract": "partnet_mobility_category_n5_hash_rank_v1",
        "dataset": BASE.EXPECTED_DATASET,
        "dataset_manifest": str(manifest),
        "dataset_manifest_sha256": BASE._sha256(manifest),
        "dataset_manifest_content_sha256": frozen_manifest["manifest_content_sha256"],
        "dataset_roster_sha256": frozen_manifest["roster_sha256"],
        "release_provenance": BASE._release_provenance_receipt(
            required=not args.allow_count_drift
        ),
        "full_model_count": len(all_items),
        "full_category_count": len({str(item.category) for item in all_items}),
        "class_count": len({str(item.category) for item in all_items}),
        "samples_per_category": args.samples_per_category,
        "per_class_count_values": [args.samples_per_category],
        "balanced_n5_eligible": all(
            sum(str(item.category) == category for item in all_items) >= args.samples_per_category
            for category in {str(item.category) for item in all_items}
        ),
        "balanced_n5_eligible_class_count": len(
            {
                category
                for category in {str(item.category) for item in all_items}
                if sum(str(item.category) == category for item in all_items)
                >= args.samples_per_category
            }
        ),
        "selected_count": len(indexed),
        "selected_category_count": len({str(item.category) for item, _ in indexed}),
        "selection_rule": (
            "first N structurally renderable assets by "
            "(SHA256(asset_id UTF-8), asset_id) within category"
        ),
        "structural_preflight": (
            "XML parse; every visual mesh filename resolves to a contained regular OBJ; "
            "at least one visual geometry"
        ),
        "preflight_exclusions": list(preflight_exclusions),
        "selection_sha256": _canonical_sha256(
            [
                {
                    "category": str(item.category),
                    "asset_id": str(item.asset_id),
                    "sample_index": rank,
                    "identity_sha256": str(item.identity_sha256),
                }
                for item, rank in indexed
            ]
        ),
        "input_validation": (
            "selected package exact manifest closure, byte count, and SHA-256"
            if not args.skip_input_validation
            else "explicitly skipped"
        ),
        "output_root": str(output_root),
        "driver": str(SCRIPT),
        "driver_sha256": BASE._sha256(SCRIPT),
        "renderer": str(renderer),
        "renderer_sha256": BASE._sha256(renderer),
        "base_renderer": str(base_renderer),
        "base_renderer_sha256": args.base_renderer_sha256,
        "shared_renderer": str(shared_renderer),
        "shared_renderer_sha256": args.shared_renderer_sha256,
        "blender": str(blender),
        "blender_version": BASE._blender_version(blender),
        "resolution": args.resolution,
        "samples": args.samples,
        "studio": BASE._studio_contract(),
        "pose_policy": "URDF rest pose; all movable joint coordinates are zero",
        "material_policy": BASE.MATERIAL_POLICY,
    }
    config_path = output_root / "render_config.json"
    if config_path.exists() and not args.force:
        old = json.loads(config_path.read_text(encoding="utf-8"))
        post_run_fields = {"blender_version", "render_roster", "render_roster_sha256"}
        comparable = {k: v for k, v in config.items() if k not in post_run_fields}
        old_comparable = {k: v for k, v in old.items() if k not in post_run_fields}
        if comparable != old_comparable:
            raise ValueError(f"existing render_config.json does not match requested run: {config_path}")
    else:
        _write_json(config_path, config)

    roster_rows = []
    for item, rank in indexed:
        row = _common_row(
            item,
            ordinal=int(item.ordinal),
            source_ordinal=source_ordinals[str(item.asset_id)],
            sample_index=rank,
            category_index=category_indices[str(item.category)],
        )
        roster_rows.append(row)
    roster_fields = BASE.ROSTER_FIELDS + [
        "source_ordinal",
        "render_key",
        "generator_index",
        "class_id",
        "generator_name",
        "source_type",
        "sample_index",
        "sample_key",
        "selection_rank",
        "png_bytes",
        "png_sha256",
    ]
    _write_csv(output_root / "render_roster.csv", roster_rows, roster_fields)

    manifest_path = output_root / "render_manifest.csv"
    prior = _read_prior(manifest_path)
    results: list[dict[str, Any]] = []
    print(f"[render] {len(indexed)} assets ({len({str(x.category) for x, _ in indexed})} categories) -> {output_root}", flush=True)
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {
            pool.submit(
                _render_one,
                item,
                sample_index=rank,
                category_index=category_indices[str(item.category)],
                source_ordinal=source_ordinals[str(item.asset_id)],
                args=args,
                renderer=renderer,
                base_renderer=base_renderer,
                shared_renderer=shared_renderer,
                blender=blender,
                prior=prior.get(str(item.asset_id)),
            ): (item, rank)
            for item, rank in indexed
        }
        for done, future in enumerate(as_completed(futures), 1):
            result = future.result()
            results.append(result)
            print(f"[render] {done}/{len(indexed)} {result['category']}/{result['asset_id']} {result['status']}", flush=True)

    result_fields = roster_fields + ["status", "elapsed_seconds", "started_at", "finished_at", "error", "renderer_result"]
    ordered_results = sorted(results, key=lambda row: int(row["ordinal"]))
    _write_csv(manifest_path, ordered_results, result_fields)
    # The final roster is also a self-contained image index for feature
    # extraction, including the immutable PNG byte and hash receipts.
    roster_path = output_root / "render_roster.csv"
    _write_csv(roster_path, ordered_results, roster_fields)
    roster_sha256 = BASE._sha256(roster_path)
    config["render_roster"] = str(roster_path)
    config["render_roster_sha256"] = roster_sha256
    _write_json(config_path, config)
    failures = [row for row in ordered_results if row.get("status") not in BASE.SUCCESS_STATUSES]
    summary = {
        "schema_version": 1,
        "render_contract": "partnet_mobility_uniform_studio_v1",
        "sampling_contract": config["sampling_contract"],
        "full_model_count": len(all_items),
        "full_category_count": len({str(item.category) for item in all_items}),
        "class_count": len({str(item.category) for item in all_items}),
        "samples_per_category": args.samples_per_category,
        "per_class_count_values": [args.samples_per_category],
        "balanced_n5_eligible": all(
            sum(str(item.category) == category for item in all_items) >= args.samples_per_category
            for category in {str(item.category) for item in all_items}
        ),
        "balanced_n5_eligible_class_count": len(
            {
                category
                for category in {str(item.category) for item in all_items}
                if sum(str(item.category) == category for item in all_items)
                >= args.samples_per_category
            }
        ),
        "preflight_exclusion_count": len(preflight_exclusions),
        "preflight_exclusions": list(preflight_exclusions),
        "selected_count": len(indexed),
        "selected_category_count": len({str(item.category) for item, _ in indexed}),
        "rendered_count": sum(row.get("status") == "rendered" for row in ordered_results),
        "reused_valid_count": sum(row.get("status") == "reused_valid" for row in ordered_results),
        "failure_count": len(failures),
        "failure_asset_ids": [str(row.get("asset_id")) for row in failures],
        "manifest": str(manifest_path),
        "roster": str(roster_path),
        "render_roster_sha256": roster_sha256,
        "config": str(config_path),
    }
    _write_json(output_root / "render_summary.json", summary)
    if failures:
        raise RuntimeError(f"{len(failures)} PartNet-Mobility render(s) failed")
    return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-manifest", type=Path, default=BASE.DEFAULT_DATASET_MANIFEST)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--renderer", type=Path, default=BASE.DEFAULT_RENDERER)
    parser.add_argument("--base-renderer", type=Path, default=BASE.DEFAULT_BASE_RENDERER)
    parser.add_argument("--shared-renderer", type=Path, default=BASE.DEFAULT_SHARED_RENDERER)
    parser.add_argument("--blender", type=Path, default=BASE.DEFAULT_BLENDER)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--samples-per-category", type=int, default=SAMPLES_PER_CATEGORY)
    parser.add_argument("--gpu", default="7")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--timeout-seconds", type=float, default=900.0)
    parser.add_argument("--categories", nargs="+")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--allow-count-drift", action="store_true")
    parser.add_argument("--skip-input-validation", action="store_true")
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
