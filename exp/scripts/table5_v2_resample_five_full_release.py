#!/usr/bin/env python3
"""Resample five Table 5 baselines from their complete frozen releases.

Articraft-10K, Infinigen-Sim, and PV-A are copied exactly from the bound
eight-dataset cohort. LAM, Artiverse, PartNet-Mobility, PhysX-Mobility, and
SketchMobility are selected from their Table 1/2/3 full-release rosters.
Selection precedes URDF parsing and never uses evaluation outcomes.
"""

from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any, Mapping, Sequence

from table123_full_release_common import ManifestError, load_roster
from table5_n200_manifest import _parse_urdf, canonical_sha256, sha256_file
from table5_v2_sample_n200 import (
    DATASET_DISPLAY_NAMES,
    EXTENDED_DATASET_SLUGS,
    MAX_MOVABLE_JOINTS,
    MIN_MOVABLE_JOINTS,
    PROTOCOL_ID,
    PVA_SLUG,
    SAMPLE_SIZE,
    SELECTION_SEED,
    dataset_eligibility_reason,
    selection_rank,
    validate_manifest,
)


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
DEFAULT_BASE_COHORT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_core200_articraft10787_infinigen_paired_official/cohort_manifest.json"
)
DEFAULT_ROSTER_ROOT = REPO_ROOT / "exp/runtime/table123_full_release_20260825"
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_core200_five_full_release_articraft10787_infinigen_paired_official"
)

RESAMPLED_SLUGS = (
    "lam_released_outputs",
    "artiverse",
    "partnet_mobility",
    "physx_mobility",
    "sketchmobility",
)
PRESERVED_SLUGS = ("articraft_10k", "infinigen_sim", PVA_SLUG)
ROSTER_SPECS = {
    "lam_released_outputs": ("lam", "LAM", 3217),
    "artiverse": ("artiverse", "Artiverse", 3544),
    "partnet_mobility": ("partnet", "PartNet-Mobility", 2347),
    "physx_mobility": ("physx", "PhysX-Mobility", 2024),
    "sketchmobility": ("sketch", "SketchMobility", 4956),
}
RUNTIME_ID_PREFIXES = {
    "lam_released_outputs": "lam",
    "artiverse": "artiverse",
    "sketchmobility": "sketch",
}


class ResampleError(RuntimeError):
    """Raised when a full-release cohort cannot be frozen safely."""


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ResampleError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise ResampleError(f"JSON root is not an object: {path}")
    return value


def _atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (
        json.dumps(value, ensure_ascii=True, allow_nan=False, sort_keys=True, indent=2)
        + "\n"
    )
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _write_jsonl(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(
                json.dumps(
                    row,
                    ensure_ascii=True,
                    allow_nan=False,
                    sort_keys=True,
                    separators=(",", ":"),
                )
                + "\n"
            )


def _runtime_id(slug: str, raw: Mapping[str, Any]) -> str:
    asset_id = str(raw["asset_id"])
    prefix = RUNTIME_ID_PREFIXES.get(slug)
    if prefix is None:
        return asset_id
    ordinal = raw.get("ordinal")
    if isinstance(ordinal, bool) or not isinstance(ordinal, int) or ordinal < 0:
        raise ResampleError(f"{slug}/{asset_id} has an invalid source ordinal")
    return f"{prefix}_{ordinal:04d}"


def _materialize_selected_row(
    slug: str,
    raw: Mapping[str, Any],
    *,
    order: int,
    rank: str,
    roster_path: Path,
    roster: Mapping[str, Any],
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
        "dataset_name": DATASET_DISPLAY_NAMES[slug],
        "dataset_id": _runtime_id(slug, raw),
        "asset_id": asset_id,
        "category": str(raw.get("category") or raw.get("raw_category") or "N/E"),
        "package_root": str(package),
        "urdf_path": str(urdf),
        "urdf_sha256": expected_hash,
        # Eligibility is bound to the frozen full-release row, not reparsing.
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
            "seed": SELECTION_SEED,
            "selection_order": order,
            "rank_sha256": rank,
            "source_row_sha256": source_hash,
        },
    }
    row["row_sha256"] = canonical_sha256(row, exclude_fields=("row_sha256",))
    return row


def _select_dataset(
    slug: str,
    roster_path: Path,
    roster: Mapping[str, Any],
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    universe_hash = str(roster["roster_sha256"])
    eligible: list[tuple[str, str, Mapping[str, Any]]] = []
    excluded: list[dict[str, Any]] = []
    exclusion_counts: Counter[str] = Counter()
    for raw in roster["rows"]:
        asset_id = str(raw["asset_id"])
        joint_count = raw.get("joint_count")
        reason = dataset_eligibility_reason(
            slug, raw.get("category"), asset_id, joint_count
        )
        rank = selection_rank(slug, asset_id, universe_sha256=universe_hash)
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
        raise ResampleError(
            f"{slug} has only {len(eligible)} eligible full-release assets"
        )

    rows: list[dict[str, Any]] = []
    selected: list[dict[str, Any]] = []
    for order, (rank, asset_id, raw) in enumerate(eligible[:SAMPLE_SIZE]):
        row = _materialize_selected_row(
            slug,
            raw,
            order=order,
            rank=rank,
            roster_path=roster_path,
            roster=roster,
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
        "dataset_name": DATASET_DISPLAY_NAMES[slug],
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
            "ordered_asset_ids_sha256": canonical_sha256(
                [row["asset_id"] for row in rows]
            ),
            "ordered_ranks_sha256": canonical_sha256(
                [row["cohort_selection"]["rank_sha256"] for row in rows]
            ),
        },
        "rows": rows,
    }
    return dataset, selected, excluded


def _updated_protocol(
    base: Mapping[str, Any], datasets: Sequence[Mapping[str, Any]]
) -> dict[str, Any]:
    protocol = deepcopy(dict(base))
    protocol["selection_rule"] = (
        "for every dataset filter the frozen source universe by "
        "1 <= movable_joint_count <= 20; for PV-A only, also apply fence, "
        "sofa-bed, and public-toilet semantic exclusions; sort eligible "
        "identities by the shared seeded SHA256 rank and take the first 200 "
        "without replacement"
    )
    protocol["baseline_candidate_universe"] = (
        "Articraft-10K uses its frozen 10,787-record complete roster; "
        "Infinigen-Sim uses the full official URDF/MJCF identity intersection; "
        "LAM, Artiverse, PartNet-Mobility, PhysX-Mobility, and SketchMobility "
        "use their frozen complete original-release rosters"
    )
    eligibility = deepcopy(dict(protocol.get("eligibility", {})))
    eligibility["minimum_movable_joint_count_inclusive"] = MIN_MOVABLE_JOINTS
    eligibility["maximum_movable_joint_count_inclusive"] = MAX_MOVABLE_JOINTS
    eligibility["semantic_exclusions_apply_only_to"] = PVA_SLUG
    protocol["eligibility"] = eligibility
    protocol["full_release_candidate_universes"] = {
        dataset["dataset_slug"]: {
            "candidate_count": dataset["universe"]["candidate_count"],
            "manifest_content_sha256": dataset["universe"][
                "manifest_content_sha256"
            ],
            "roster_sha256": dataset["universe"]["sha256"],
            "roster_jsonl_sha256": dataset["universe"]["roster_jsonl_sha256"],
        }
        for dataset in datasets
        if dataset["dataset_slug"] in RESAMPLED_SLUGS
    }
    protocol["protocol_sha256"] = canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    return protocol


def build(base_cohort_path: Path, roster_root: Path, output: Path) -> dict[str, Any]:
    output = output.resolve(strict=False)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.work.", dir=output.parent))
    try:
        base = _read_json(base_cohort_path.resolve(strict=True))
        validate_manifest(base)
        base_by_slug = {
            str(dataset["dataset_slug"]): dataset for dataset in base["datasets"]
        }
        if tuple(base_by_slug) != EXTENDED_DATASET_SLUGS:
            raise ResampleError("base cohort is not the expected eight-dataset cohort")

        replacements: dict[str, dict[str, Any]] = {}
        all_selected: list[dict[str, Any]] = []
        all_excluded: list[dict[str, Any]] = []
        for slug in RESAMPLED_SLUGS:
            directory, expected_dataset, expected_count = ROSTER_SPECS[slug]
            roster_path = (roster_root / directory / "full_release_manifest.json").resolve(
                strict=True
            )
            roster = load_roster(
                roster_path,
                expected_dataset=expected_dataset,
                verify_sources=False,
            )
            if roster["N_eval"] != expected_count:
                raise ResampleError(
                    f"{slug} full-release count changed: "
                    f"{roster['N_eval']} != {expected_count}"
                )
            dataset, selected, excluded = _select_dataset(slug, roster_path, roster)
            replacements[slug] = dataset
            all_selected.extend(selected)
            all_excluded.extend(excluded)

        datasets = [
            replacements.get(slug, deepcopy(base_by_slug[slug]))
            for slug in EXTENDED_DATASET_SLUGS
        ]
        for slug in PRESERVED_SLUGS:
            observed = next(
                dataset for dataset in datasets if dataset["dataset_slug"] == slug
            )
            if observed != base_by_slug[slug]:
                raise ResampleError(f"preserved dataset changed unexpectedly: {slug}")

        protocol = _updated_protocol(base["protocol"], datasets)
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
                    "dataset_slug": dataset["dataset_slug"],
                    "universe_sha256": dataset["universe"]["sha256"],
                    "ordered_asset_ids_sha256": dataset["selection"][
                        "ordered_asset_ids_sha256"
                    ],
                    "ordered_ranks_sha256": dataset["selection"][
                        "ordered_ranks_sha256"
                    ],
                }
                for dataset in datasets
            ]
        )
        manifest["manifest_sha256"] = canonical_sha256(
            manifest, exclude_fields=("manifest_sha256",)
        )
        validate_manifest(manifest)

        _atomic_json(staging / "cohort_manifest.json", manifest)
        _atomic_json(
            staging / "selection_summary.json",
            {
                "schema_version": "table5_v2_five_full_release_selection_v1",
                "base_cohort_path": str(base_cohort_path.resolve(strict=True)),
                "base_manifest_sha256": base["manifest_sha256"],
                "protocol_id": PROTOCOL_ID,
                "selection_seed": SELECTION_SEED,
                "semantic_exclusions_apply_only_to": PVA_SLUG,
                "resampled_dataset_slugs": list(RESAMPLED_SLUGS),
                "preserved_dataset_slugs": list(PRESERVED_SLUGS),
                "datasets": [
                    {
                        "dataset_slug": dataset["dataset_slug"],
                        "universe": dataset["universe"],
                        "selection": dataset["selection"],
                    }
                    for dataset in datasets
                ],
            },
        )
        _write_jsonl(staging / "selected_assets.jsonl", all_selected)
        _write_jsonl(staging / "excluded_assets.jsonl", all_excluded)
        os.replace(staging, output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return manifest


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-cohort", type=Path, default=DEFAULT_BASE_COHORT)
    parser.add_argument("--roster-root", type=Path, default=DEFAULT_ROSTER_ROOT)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    try:
        manifest = build(
            arguments.base_cohort, arguments.roster_root, arguments.out
        )
    except (FileExistsError, ManifestError, ResampleError, OSError) as error:
        parser.error(str(error))
    print(
        json.dumps(
            {
                "cohort": str((arguments.out / "cohort_manifest.json").resolve()),
                "cohort_sha256": manifest["cohort_sha256"],
                "manifest_sha256": manifest["manifest_sha256"],
                "protocol_sha256": manifest["protocol_sha256"],
                "dataset_count": manifest["dataset_count"],
                "total_rows": manifest["total_rows"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
