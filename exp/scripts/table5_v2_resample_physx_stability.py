#!/usr/bin/env python3
"""Resample ONLY physx_mobility with a NEW seed (stability check).

PhysX-Mobility is re-selected from its frozen complete original-release roster
(2024 candidates, 1<=movable_joints<=20 filter, seeded SHA256 rank) using a new
selection seed. The other seven datasets are copied unchanged from the bound
eight-dataset cohort. Output is a full eight-dataset cohort manifest compatible
with the frozen revision-2 prepare/runtime/aggregate pipeline.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import tempfile
from collections import Counter
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping, Sequence

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_v2_resample_five_full_release as _five  # noqa: E402
from table5_v2_sample_n200 import (  # noqa: E402
    EXTENDED_DATASET_SLUGS,
    PROTOCOL_ID,
    SAMPLE_SIZE,
    canonical_sha256,
    dataset_eligibility_reason,
    selection_rank,
    validate_manifest,
)
from table5_n200_manifest import _parse_urdf, sha256_file  # noqa: E402
from table123_full_release_common import load_roster  # noqa: E402

NEW_SEED = "arti-skill-table5-v2-physx-stability-resample-20260830"
SLUG = "physx_mobility"


def _runtime_id(slug: str, raw: Mapping[str, Any]) -> str:
    asset_id = str(raw["asset_id"])
    prefix = _five.RUNTIME_ID_PREFIXES.get(slug)
    if prefix is None:
        return asset_id
    ordinal = raw.get("ordinal")
    if isinstance(ordinal, bool) or not isinstance(ordinal, int) or ordinal < 0:
        raise _five.ResampleError(f"{slug}/{asset_id} has an invalid source ordinal")
    return f"{prefix}_{ordinal:04d}"


def materialize_row(
    slug: str,
    raw: Mapping[str, Any],
    *,
    order: int,
    rank: str,
    roster_path: Path,
    roster: Mapping[str, Any],
    seed: str,
) -> dict[str, Any]:
    asset_id = str(raw["asset_id"])
    package = Path(str(raw["source_path"])).resolve(strict=False)
    urdf = Path(str(raw["primary_urdf_path"])).resolve(strict=False)
    expected_hash = str(raw["primary_urdf_sha256"])
    issues: list[str] = []
    observed_hash: str | None = None
    try:
        observed_hash = sha256_file(urdf.resolve(strict=True))
        if observed_hash != expected_hash:
            issues.append("source_urdf_sha256_mismatch")
    except OSError as error:
        issues.append(f"source_urdf_unavailable:{type(error).__name__}:{error}")
    parsed = _parse_urdf(package, urdf)
    issues.extend(str(issue) for issue in parsed["issues"])
    source_hash = canonical_sha256(raw)
    row: dict[str, Any] = {
        "dataset_slug": slug,
        "dataset_name": _five.DATASET_DISPLAY_NAMES[slug],
        "dataset_id": _runtime_id(slug, raw),
        "asset_id": asset_id,
        "category": str(raw.get("category") or raw.get("raw_category") or "N/E"),
        "package_root": str(package),
        "urdf_path": str(urdf),
        "urdf_sha256": expected_hash,
        "xml_counts": deepcopy(raw["xml_counts"]),
        "joint_tree": parsed["joint_tree"],
        "scalar_joints": parsed["scalar_joints"],
        "materialization_xml_counts": parsed["xml_counts"],
        "preflight": {
            "status": "failed" if issues else "pass",
            "issues": sorted(set(issues)),
            "warnings": [],
            "simulator_eligible": not issues,
        },
        "source_full_release": {
            "manifest_path": str(roster_path),
            "manifest_content_sha256": roster["manifest_content_sha256"],
            "roster_sha256": roster["roster_sha256"],
            "roster_jsonl_sha256": roster["roster_jsonl_sha256"],
            "source_ordinal": raw["ordinal"],
            "source_row_sha256": source_hash,
            "package_binding_sha256": raw.get("package_binding_sha256"),
            "primary_urdf_size": raw.get("primary_urdf_size"),
            "observed_primary_urdf_sha256": observed_hash,
        },
        "cohort_selection": {
            "protocol_id": PROTOCOL_ID,
            "seed": seed,
            "selection_order": order,
            "rank_sha256": rank,
            "source_row_sha256": source_hash,
        },
    }
    row["row_sha256"] = canonical_sha256(row, exclude_fields=("row_sha256",))
    return row


def select_dataset(
    slug: str,
    roster_path: Path,
    roster: Mapping[str, Any],
    seed: str,
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    universe_hash = str(roster["roster_sha256"])
    eligible: list[tuple[str, str, Mapping[str, Any]]] = []
    excluded: list[dict[str, Any]] = []
    exclusion_counts: Counter[str] = Counter()
    for raw in roster["rows"]:
        asset_id = str(raw["asset_id"])
        joint_count = raw.get("joint_count")
        reason = dataset_eligibility_reason(slug, raw.get("category"), asset_id, joint_count)
        rank = selection_rank(slug, asset_id, universe_sha256=universe_hash, seed=seed)
        if reason is not None:
            exclusion_counts[reason] += 1
            excluded.append(
                {
                    "dataset_slug": slug,
                    "asset_id": asset_id,
                    "category": raw.get("category"),
                    "movable_joint_count": joint_count,
                    "reason": reason,
                    "rank_sha256": rank,
                }
            )
            continue
        eligible.append((rank, asset_id, raw))
    eligible.sort(key=lambda item: (item[0], item[1]))
    if len(eligible) < SAMPLE_SIZE:
        raise _five.ResampleError(f"{slug} has only {len(eligible)} eligible assets")
    rows: list[dict[str, Any]] = []
    selected: list[dict[str, Any]] = []
    for order, (rank, asset_id, raw) in enumerate(eligible[:SAMPLE_SIZE]):
        row = materialize_row(
            slug, raw, order=order, rank=rank, roster_path=roster_path, roster=roster, seed=seed
        )
        rows.append(row)
        selected.append(
            {
                "dataset_slug": slug,
                "selection_order": order,
                "asset_id": asset_id,
                "category": row["category"],
                "movable_joint_count": raw["joint_count"],
                "rank_sha256": rank,
                "source_ordinal": raw["ordinal"],
                "urdf_path": row["urdf_path"],
                "urdf_sha256": row["urdf_sha256"],
            }
        )
    roster_jsonl = roster_path.with_name("full_release_roster.jsonl")
    dataset = {
        "dataset_slug": slug,
        "dataset_name": _five.DATASET_DISPLAY_NAMES[slug],
        "universe": {
            "kind": "frozen_complete_original_release_roster",
            "path": str(roster_jsonl),
            "sha256": universe_hash,
            "manifest_path": str(roster_path),
            "manifest_content_sha256": roster["manifest_content_sha256"],
            "roster_jsonl_sha256": roster["roster_jsonl_sha256"],
            "candidate_count": len(roster["rows"]),
            "source_dataset": roster["dataset"],
        },
        "selection": {
            "eligible_count": len(eligible),
            "excluded_count": len(excluded),
            "exclusion_counts": dict(sorted(exclusion_counts.items())),
            "selected_count": len(rows),
            "ordered_asset_ids_sha256": canonical_sha256([r["asset_id"] for r in rows]),
            "ordered_ranks_sha256": canonical_sha256(
                [r["cohort_selection"]["rank_sha256"] for r in rows]
            ),
        },
        "rows": rows,
    }
    return dataset, selected, excluded


def validate_physx_stability_cohort(
    manifest: Mapping[str, Any], seed: str, expected_slugs: Sequence[str] | None = None
) -> None:
    """Validate the resampled cohort, accepting the NEW seed for physx only."""
    if expected_slugs is None:
        expected_slugs = list(EXTENDED_DATASET_SLUGS)
    if manifest.get("schema_version") != "table5_v2_core200_manifest_v1":
        raise _five.ResampleError("manifest schema mismatch")
    protocol_value = manifest.get("protocol")
    if not isinstance(protocol_value, Mapping):
        raise _five.ResampleError("manifest protocol is missing")
    protocol_hash = canonical_sha256(protocol_value, exclude_fields=("protocol_sha256",))
    if (
        protocol_value.get("protocol_sha256") != protocol_hash
        or manifest.get("protocol_sha256") != protocol_hash
    ):
        raise _five.ResampleError("protocol hash mismatch")
    datasets = manifest.get("datasets")
    if not isinstance(datasets, list) or [
        d.get("dataset_slug") for d in datasets
    ] != list(expected_slugs):
        raise _five.ResampleError("dataset order mismatch")
    total = 0
    for dataset in datasets:
        slug = str(dataset["dataset_slug"])
        rows = dataset.get("rows")
        if not isinstance(rows, list) or len(rows) != SAMPLE_SIZE:
            raise _five.ResampleError(f"{slug} does not contain exactly {SAMPLE_SIZE} rows")
        universe_sha256 = str(dataset["universe"]["sha256"])
        ranks: list[str] = []
        seen: set[str] = set()
        for order, row in enumerate(rows):
            asset_id = str(row.get("asset_id") or "")
            if not asset_id or asset_id in seen:
                raise _five.ResampleError(f"{slug} duplicate/invalid asset_id")
            seen.add(asset_id)
            counts = row.get("xml_counts")
            joint_count = counts.get("movable_joints") if isinstance(counts, Mapping) else None
            if dataset_eligibility_reason(slug, row.get("category"), asset_id, joint_count) is not None:
                raise _five.ResampleError(f"{slug}/{asset_id} violates eligibility")
            selection = row.get("cohort_selection")
            if not isinstance(selection, Mapping):
                raise _five.ResampleError(f"{slug}/{asset_id} lacks selection binding")
            rank = str(selection.get("rank_sha256") or "")
            if slug == SLUG:
                expected_rank = selection_rank(
                    slug, asset_id, universe_sha256=universe_sha256, seed=seed
                )
            else:
                expected_rank = selection_rank(slug, asset_id, universe_sha256=universe_sha256)
            if rank != expected_rank or selection.get("selection_order") != order:
                raise _five.ResampleError(f"{slug}/{asset_id} rank/order binding mismatch")
            if row.get("row_sha256") != canonical_sha256(row, exclude_fields=("row_sha256",)):
                raise _five.ResampleError(f"{slug}/{asset_id} row hash mismatch")
            ranks.append(rank)
            total += 1
        if ranks != sorted(ranks):
            raise _five.ResampleError(f"{slug} rows are not rank-sorted")
        if dataset["selection"].get("ordered_asset_ids_sha256") != canonical_sha256(
            [r["asset_id"] for r in rows]
        ):
            raise _five.ResampleError(f"{slug} ordered asset hash mismatch")
        if dataset["selection"].get("ordered_ranks_sha256") != canonical_sha256(ranks):
            raise _five.ResampleError(f"{slug} ordered ranks hash mismatch")
    if manifest.get("total_rows") != total:
        raise _five.ResampleError("cohort total_rows mismatch")
    cohort_hash = canonical_sha256(
        [
            {
                "dataset_slug": d["dataset_slug"],
                "universe_sha256": d["universe"]["sha256"],
                "ordered_asset_ids_sha256": d["selection"]["ordered_asset_ids_sha256"],
                "ordered_ranks_sha256": d["selection"]["ordered_ranks_sha256"],
            }
            for d in datasets
        ]
    )
    if manifest.get("cohort_sha256") != cohort_hash:
        raise _five.ResampleError("cohort hash mismatch")
    if manifest.get("manifest_sha256") != canonical_sha256(
        manifest, exclude_fields=("manifest_sha256",)
    ):
        raise _five.ResampleError("cohort manifest hash mismatch")


def build(base_cohort_path: Path, roster_root: Path, output: Path, seed: str) -> dict[str, Any]:
    output = output.resolve(strict=False)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.work.", dir=output.parent))
    try:
        base = _five._read_json(base_cohort_path.resolve(strict=True))
        validate_manifest(base)
        base_by_slug = {str(d["dataset_slug"]): d for d in base["datasets"]}
        directory, expected_dataset, expected_count = _five.ROSTER_SPECS[SLUG]
        roster_path = (roster_root / directory / "full_release_manifest.json").resolve(strict=True)
        roster = load_roster(roster_path, expected_dataset=expected_dataset, verify_sources=False)
        if roster["N_eval"] != expected_count:
            raise _five.ResampleError(f"{SLUG} count changed: {roster['N_eval']} != {expected_count}")
        dataset, selected, excluded = select_dataset(SLUG, roster_path, roster, seed)
        datasets = [
            dataset if slug == SLUG else deepcopy(base_by_slug[slug])
            for slug in EXTENDED_DATASET_SLUGS
        ]
        protocol = deepcopy(base["protocol"])
        manifest: dict[str, Any] = {
            "schema_version": base["schema_version"],
            "protocol": protocol,
            "protocol_sha256": protocol["protocol_sha256"],
            "sample_size_per_dataset": SAMPLE_SIZE,
            "dataset_count": len(datasets),
            "total_rows": SAMPLE_SIZE * len(datasets),
            "ordered_dataset_slugs": list(EXTENDED_DATASET_SLUGS),
            "datasets": datasets,
        }
        manifest["cohort_sha256"] = canonical_sha256(
            [
                {
                    "dataset_slug": d["dataset_slug"],
                    "universe_sha256": d["universe"]["sha256"],
                    "ordered_asset_ids_sha256": d["selection"]["ordered_asset_ids_sha256"],
                    "ordered_ranks_sha256": d["selection"]["ordered_ranks_sha256"],
                }
                for d in datasets
            ]
        )
        manifest["manifest_sha256"] = canonical_sha256(
            manifest, exclude_fields=("manifest_sha256",)
        )
        validate_physx_stability_cohort(manifest, seed)
        _five._atomic_json(staging / "cohort_manifest.json", manifest)
        _five._atomic_json(
            staging / "selection_summary.json",
            {
                "schema_version": "table5_v2_physx_stability_selection_v1",
                "base_cohort_path": str(base_cohort_path.resolve(strict=True)),
                "base_manifest_sha256": base["manifest_sha256"],
                "protocol_id": PROTOCOL_ID,
                "selection_seed": seed,
                "resampled_dataset_slugs": [SLUG],
                "preserved_dataset_slugs": [s for s in EXTENDED_DATASET_SLUGS if s != SLUG],
                "dataset": {"dataset_slug": dataset["dataset_slug"], "universe": dataset["universe"], "selection": dataset["selection"]},
            },
        )
        _five._write_jsonl(staging / "selected_assets.jsonl", selected)
        _five._write_jsonl(staging / "excluded_assets.jsonl", excluded)
        os.replace(staging, output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return manifest


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-cohort", type=Path, default=_five.DEFAULT_BASE_COHORT)
    parser.add_argument("--roster-root", type=Path, default=_five.DEFAULT_ROSTER_ROOT)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seed", type=str, default=NEW_SEED)
    args = parser.parse_args(argv)
    manifest = build(args.base_cohort, args.roster_root, args.out, args.seed)
    physx = next(d for d in manifest["datasets"] if d["dataset_slug"] == SLUG)
    print(json.dumps({
        "out": str(args.out.resolve()),
        "manifest_sha256": manifest["manifest_sha256"],
        "seed": args.seed,
        "physx_selected": physx["selection"]["selected_count"],
        "physx_eligible": physx["selection"]["eligible_count"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
