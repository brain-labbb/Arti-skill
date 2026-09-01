#!/usr/bin/env python3
"""Freeze the Table 5 v2 Core-200 cohort for six baselines and PV-A.

The six baseline candidate universes are their already-frozen N=800 parent
rosters.  PV-A uses its frozen full-release roster.  Selection uses only the
declared scope filters and a deterministic SHA256 rank; simulator outcomes,
preflight status, and previous Table 5 results never affect membership.
"""

from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import heapq
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
from typing import Any, Mapping, Sequence, TextIO

from table5_n200_manifest import (
    DATASET_NAMES,
    DATASET_ORDER,
    PARENT_CONTAINERS,
    _parse_urdf,
    build_manifest as build_baseline_manifest,
    canonical_sha256,
    sha256_file,
)


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
SAMPLE_SIZE = 200
BASELINE_PARENT_SIZE = 800
MIN_MOVABLE_JOINTS = 1
MAX_MOVABLE_JOINTS = 20
SELECTION_SEED = "arti-skill-table5-v2-core200-20260828"
PROTOCOL_ID = "table5-v2-core200-hash-sample-v1"
PVA_SLUG = "pva"
PVA_NAME = "Ours (PV-A)"
INFINIGEN_SLUG = "infinigen_sim"
INFINIGEN_NAME = "Infinigen-Sim"
PVA_ROSTER = (
    REPO_ROOT
    / "exp/runtime/pva_table1234_full_release_20260826/roster/full_release_roster.jsonl"
)
PVA_ROSTER_RECEIPT = PVA_ROSTER.with_name("roster_manifest.json")
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/table5_v2_core200_seed20260828"
DATASET_SLUGS = (*DATASET_ORDER, PVA_SLUG)
EXTENDED_DATASET_SLUGS = (*DATASET_ORDER, INFINIGEN_SLUG, PVA_SLUG)
DATASET_DISPLAY_NAMES = {**DATASET_NAMES, PVA_SLUG: PVA_NAME}

# These PV-A-only scope exclusions are not observed failure modes. Matching is
# done after lower-casing and replacing every non-alphanumeric run with an
# underscore.
SEMANTIC_EXCLUSIONS = {
    "fence": ("fence", "fences"),
    "sofa_bed": (
        "sofa_bed",
        "sofa_beds",
        "sofabed",
        "sofabeds",
        "folding_sofa",
        "folding_sofas",
    ),
    "public_toilet": (
        "public_toilet",
        "public_toilets",
        "public_restroom",
        "public_restrooms",
        "compound_restroom",
        "compound_restrooms",
    ),
}


class CohortError(ValueError):
    """Raised when the cohort cannot be frozen without changing the protocol."""


def normalize_label(value: object) -> str:
    """Return the exact normalized label used by the semantic scope filter."""

    return re.sub(r"_+", "_", re.sub(r"[^a-z0-9]+", "_", str(value).lower())).strip("_")


def semantic_exclusion(category: object, asset_id: object) -> str | None:
    """Return a declared semantic exclusion, if one matches category/identity."""

    normalized = normalize_label(f"{category} {asset_id}")
    padded = f"_{normalized}_"
    for reason, patterns in SEMANTIC_EXCLUSIONS.items():
        for pattern in patterns:
            if f"_{pattern}_" in padded:
                return reason
    return None


def eligibility_reason(
    category: object, asset_id: object, movable_joint_count: object
) -> str | None:
    """Return the first frozen exclusion reason, or None for an eligible row."""

    semantic = semantic_exclusion(category, asset_id)
    if semantic is not None:
        return semantic
    if isinstance(movable_joint_count, bool) or not isinstance(
        movable_joint_count, int
    ):
        return "movable_joint_count_unavailable"
    if movable_joint_count < MIN_MOVABLE_JOINTS:
        return "movable_joint_count_lt_1"
    if movable_joint_count > MAX_MOVABLE_JOINTS:
        return "movable_joint_count_gt_20"
    return None


def dataset_eligibility_reason(
    dataset_slug: str,
    category: object,
    asset_id: object,
    movable_joint_count: object,
) -> str | None:
    """Return the dataset-specific frozen exclusion reason, if any.

    The movable-joint scope applies to every dataset. The three semantic scope
    exclusions are part of PV-A's release definition only.
    """

    if dataset_slug == PVA_SLUG:
        return eligibility_reason(category, asset_id, movable_joint_count)
    if isinstance(movable_joint_count, bool) or not isinstance(
        movable_joint_count, int
    ):
        return "movable_joint_count_unavailable"
    if movable_joint_count < MIN_MOVABLE_JOINTS:
        return "movable_joint_count_lt_1"
    if movable_joint_count > MAX_MOVABLE_JOINTS:
        return "movable_joint_count_gt_20"
    return None


def selection_rank(
    dataset_slug: str,
    asset_id: str,
    *,
    universe_sha256: str,
    seed: str = SELECTION_SEED,
) -> str:
    """Return the deterministic random rank for one candidate identity."""

    identity = "\0".join((PROTOCOL_ID, seed, universe_sha256, dataset_slug, asset_id))
    return hashlib.sha256(identity.encode("utf-8")).hexdigest()


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def _write_jsonl_row(handle: TextIO, value: Mapping[str, Any]) -> None:
    handle.write(json.dumps(value, sort_keys=True, ensure_ascii=True) + "\n")


def _movable_joint_count(row: Mapping[str, Any]) -> int | None:
    counts = row.get("xml_counts")
    if not isinstance(counts, Mapping):
        return None
    value = counts.get("movable_joints")
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _selected_baseline_row(
    raw: Mapping[str, Any], *, selection_order: int, rank: str
) -> dict[str, Any]:
    row = deepcopy(dict(raw))
    source_row_sha256 = row.get("row_sha256")
    row["cohort_selection"] = {
        "protocol_id": PROTOCOL_ID,
        "seed": SELECTION_SEED,
        "selection_order": selection_order,
        "rank_sha256": rank,
        "source_row_sha256": source_row_sha256,
    }
    row["row_sha256"] = canonical_sha256(row, exclude_fields=("row_sha256",))
    return row


def select_baselines(
    exclusion_handle: TextIO,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Materialize each frozen N=800 baseline pool and select Core-200."""

    parent_manifest = build_baseline_manifest(BASELINE_PARENT_SIZE)
    datasets: list[dict[str, Any]] = []
    selected_ledger: list[dict[str, Any]] = []
    for dataset in parent_manifest["datasets"]:
        slug = str(dataset["dataset_slug"])
        universe_sha256 = str(dataset["parent"]["sha256"])
        eligible: list[tuple[str, str, Mapping[str, Any]]] = []
        exclusion_counts: Counter[str] = Counter()
        for raw in dataset["rows"]:
            asset_id = str(raw["asset_id"])
            category = str(raw.get("category") or "N/E")
            joint_count = _movable_joint_count(raw)
            reason = dataset_eligibility_reason(slug, category, asset_id, joint_count)
            rank = selection_rank(slug, asset_id, universe_sha256=universe_sha256)
            if reason is not None:
                exclusion_counts[reason] += 1
                _write_jsonl_row(
                    exclusion_handle,
                    {
                        "asset_id": asset_id,
                        "category": category,
                        "dataset_slug": slug,
                        "movable_joint_count": joint_count,
                        "reason": reason,
                        "rank_sha256": rank,
                    },
                )
                continue
            eligible.append((rank, asset_id, raw))
        eligible.sort(key=lambda item: (item[0], item[1]))
        if len(eligible) < SAMPLE_SIZE:
            raise CohortError(
                f"{slug} has only {len(eligible)} eligible candidates; "
                f"{SAMPLE_SIZE} required"
            )
        selected_rows: list[dict[str, Any]] = []
        for order, (rank, asset_id, raw) in enumerate(eligible[:SAMPLE_SIZE]):
            selected_rows.append(
                _selected_baseline_row(raw, selection_order=order, rank=rank)
            )
            selected_ledger.append(
                {
                    "asset_id": asset_id,
                    "category": str(raw.get("category") or "N/E"),
                    "dataset_slug": slug,
                    "movable_joint_count": _movable_joint_count(raw),
                    "rank_sha256": rank,
                    "selection_order": order,
                    "urdf_path": raw.get("urdf_path"),
                    "urdf_sha256": raw.get("urdf_sha256"),
                }
            )
        datasets.append(
            {
                "dataset_slug": slug,
                "dataset_name": DATASET_DISPLAY_NAMES[slug],
                "universe": {
                    "kind": "frozen_n800_parent_roster",
                    "path": dataset["parent"]["path"],
                    "sha256": universe_sha256,
                    "row_container": PARENT_CONTAINERS[slug],
                    "candidate_count": len(dataset["rows"]),
                },
                "selection": {
                    "eligible_count": len(eligible),
                    "excluded_count": sum(exclusion_counts.values()),
                    "exclusion_counts": dict(sorted(exclusion_counts.items())),
                    "selected_count": len(selected_rows),
                    "ordered_asset_ids_sha256": canonical_sha256(
                        [row["asset_id"] for row in selected_rows]
                    ),
                    "ordered_ranks_sha256": canonical_sha256(
                        [
                            row["cohort_selection"]["rank_sha256"]
                            for row in selected_rows
                        ]
                    ),
                },
                "rows": selected_rows,
            }
        )
    return datasets, selected_ledger


def _pva_receipt() -> tuple[dict[str, Any], str, int]:
    try:
        receipt = json.loads(PVA_ROSTER_RECEIPT.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CohortError(f"cannot load PV-A roster receipt: {error}") from error
    roster = receipt.get("roster")
    if not isinstance(roster, Mapping):
        raise CohortError("PV-A roster receipt has no roster binding")
    expected_hash = roster.get("sha256")
    expected_count = roster.get("row_count")
    if not isinstance(expected_hash, str) or len(expected_hash) != 64:
        raise CohortError("PV-A roster receipt SHA256 is invalid")
    if isinstance(expected_count, bool) or not isinstance(expected_count, int):
        raise CohortError("PV-A roster receipt row_count is invalid")
    return receipt, expected_hash, expected_count


def _push_smallest(
    heap: list[tuple[int, str, dict[str, Any]]],
    *,
    rank: str,
    asset_id: str,
    row: dict[str, Any],
) -> None:
    """Keep only the SAMPLE_SIZE rows with the lexically smallest rank."""

    entry = (-int(rank, 16), asset_id, row)
    if len(heap) < SAMPLE_SIZE:
        heapq.heappush(heap, entry)
        return
    if entry[0] > heap[0][0]:
        heapq.heapreplace(heap, entry)


def _selected_pva_row(
    raw: Mapping[str, Any], *, selection_order: int, rank: str, roster_hash: str
) -> dict[str, Any]:
    package = Path(str(raw["source_path"])).resolve(strict=False)
    urdf = Path(str(raw["primary_urdf_path"])).resolve(strict=False)
    parsed = _parse_urdf(package, urdf)
    source_row_sha256 = canonical_sha256(raw)
    row: dict[str, Any] = {
        "dataset_slug": PVA_SLUG,
        "dataset_name": PVA_NAME,
        "asset_id": str(raw["asset_id"]),
        "dataset_id": f"pva_{selection_order:04d}",
        "category": str(raw.get("category") or raw.get("slug") or "N/E"),
        "package_root": str(package),
        "urdf_path": str(urdf),
        "urdf_sha256": raw.get("primary_urdf_sha256"),
        "physics_json_path": str(package / "physics.json"),
        "joint_tree": parsed["joint_tree"],
        "scalar_joints": parsed["scalar_joints"],
        "xml_counts": parsed["xml_counts"],
        "preflight": {
            "status": "passed" if not parsed["issues"] else "failed",
            "issues": parsed["issues"],
        },
        "source_roster": {
            "path": str(PVA_ROSTER.resolve(strict=False)),
            "sha256": roster_hash,
            "ordinal": raw.get("ordinal"),
            "row_sha256": source_row_sha256,
            "package_content_manifest_sha256": raw.get("package_binding_sha256"),
            "archive_name": raw.get("archive_name"),
            "archive_sha256": raw.get("archive_sha256"),
        },
        "cohort_selection": {
            "protocol_id": PROTOCOL_ID,
            "seed": SELECTION_SEED,
            "selection_order": selection_order,
            "rank_sha256": rank,
            "source_row_sha256": source_row_sha256,
        },
    }
    row["row_sha256"] = canonical_sha256(row, exclude_fields=("row_sha256",))
    return row


def select_pva(
    exclusion_handle: TextIO,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Stream the PV-A full-release roster and select its Core-200 rows."""

    receipt, expected_hash, expected_count = _pva_receipt()
    digest = hashlib.sha256()
    observed_count = 0
    eligible_count = 0
    exclusion_counts: Counter[str] = Counter()
    heap: list[tuple[int, str, dict[str, Any]]] = []
    seen_ids: set[str] = set()
    with PVA_ROSTER.open("rb") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            digest.update(raw_line)
            try:
                raw = json.loads(raw_line)
            except json.JSONDecodeError as error:
                raise CohortError(
                    f"PV-A roster line {line_number} is invalid JSON: {error}"
                ) from error
            if not isinstance(raw, dict):
                raise CohortError(f"PV-A roster line {line_number} is not an object")
            observed_count += 1
            asset_id = str(raw.get("asset_id") or "")
            if not asset_id or asset_id in seen_ids:
                raise CohortError(
                    f"PV-A roster contains invalid/duplicate asset_id: {asset_id!r}"
                )
            seen_ids.add(asset_id)
            category = str(raw.get("category") or raw.get("slug") or "N/E")
            joint_count = _movable_joint_count(raw)
            reason = eligibility_reason(category, asset_id, joint_count)
            rank = selection_rank(PVA_SLUG, asset_id, universe_sha256=expected_hash)
            if reason is not None:
                exclusion_counts[reason] += 1
                _write_jsonl_row(
                    exclusion_handle,
                    {
                        "asset_id": asset_id,
                        "category": category,
                        "dataset_slug": PVA_SLUG,
                        "movable_joint_count": joint_count,
                        "reason": reason,
                        "rank_sha256": rank,
                    },
                )
                continue
            eligible_count += 1
            _push_smallest(heap, rank=rank, asset_id=asset_id, row=raw)
    observed_hash = digest.hexdigest()
    if observed_hash != expected_hash:
        raise CohortError(
            f"PV-A roster SHA256 mismatch: {observed_hash} != {expected_hash}"
        )
    if observed_count != expected_count:
        raise CohortError(
            f"PV-A roster row count mismatch: {observed_count} != {expected_count}"
        )
    if eligible_count < SAMPLE_SIZE or len(heap) != SAMPLE_SIZE:
        raise CohortError(
            f"PV-A has only {eligible_count} eligible candidates; {SAMPLE_SIZE} required"
        )
    ranked = sorted(
        (
            (
                selection_rank(
                    PVA_SLUG,
                    str(row["asset_id"]),
                    universe_sha256=expected_hash,
                ),
                str(row["asset_id"]),
                row,
            )
            for _, _, row in heap
        ),
        key=lambda item: (item[0], item[1]),
    )
    selected_rows: list[dict[str, Any]] = []
    selected_ledger: list[dict[str, Any]] = []
    for order, (rank, asset_id, raw) in enumerate(ranked):
        row = _selected_pva_row(
            raw,
            selection_order=order,
            rank=rank,
            roster_hash=expected_hash,
        )
        selected_rows.append(row)
        selected_ledger.append(
            {
                "asset_id": asset_id,
                "category": row["category"],
                "dataset_slug": PVA_SLUG,
                "movable_joint_count": _movable_joint_count(row),
                "rank_sha256": rank,
                "selection_order": order,
                "urdf_path": row["urdf_path"],
                "urdf_sha256": row["urdf_sha256"],
            }
        )
    dataset = {
        "dataset_slug": PVA_SLUG,
        "dataset_name": PVA_NAME,
        "universe": {
            "kind": "frozen_full_release_roster",
            "path": str(PVA_ROSTER.resolve(strict=False)),
            "sha256": expected_hash,
            "receipt_path": str(PVA_ROSTER_RECEIPT.resolve(strict=False)),
            "receipt_sha256": sha256_file(PVA_ROSTER_RECEIPT),
            "candidate_count": observed_count,
            "source_manifest": receipt.get("source_bindings", {}).get(
                "source_manifest"
            ),
            "source_manifest_sha256": receipt.get("source_bindings", {}).get(
                "source_manifest_sha256"
            ),
        },
        "selection": {
            "eligible_count": eligible_count,
            "excluded_count": sum(exclusion_counts.values()),
            "exclusion_counts": dict(sorted(exclusion_counts.items())),
            "selected_count": len(selected_rows),
            "ordered_asset_ids_sha256": canonical_sha256(
                [row["asset_id"] for row in selected_rows]
            ),
            "ordered_ranks_sha256": canonical_sha256(
                [row["cohort_selection"]["rank_sha256"] for row in selected_rows]
            ),
        },
        "rows": selected_rows,
    }
    return dataset, selected_ledger


def protocol() -> dict[str, Any]:
    value: dict[str, Any] = {
        "protocol_id": PROTOCOL_ID,
        "sample_size_per_dataset": SAMPLE_SIZE,
        "selection_seed": SELECTION_SEED,
        "selection_rule": (
            "for every dataset filter the frozen source universe by "
            "1 <= movable_joint_count <= 20; for PV-A only, also apply the "
            "declared semantic scope exclusions; sort eligible identities by "
            "seeded SHA256 rank and take the first 200 without replacement"
        ),
        "baseline_candidate_universe": (
            "each method's already-frozen N=800 roster, materialized without "
            "filtering on parse, preflight, or simulator outcomes"
        ),
        "pva_candidate_universe": "the frozen 302,440-asset full-release roster",
        "eligibility": {
            "minimum_movable_joint_count_inclusive": MIN_MOVABLE_JOINTS,
            "maximum_movable_joint_count_inclusive": MAX_MOVABLE_JOINTS,
            "semantic_exclusions_apply_only_to": PVA_SLUG,
            "normalized_semantic_exclusions": {
                key: list(values) for key, values in SEMANTIC_EXCLUSIONS.items()
            },
            "normalization": (
                "lowercase category plus asset_id; replace each non-alphanumeric "
                "run by underscore; match complete normalized token sequence"
            ),
        },
        "replacement": False,
        "outcome_filtering": False,
        "failure_replacement": False,
        "historical_result_fields_used": [],
    }
    value["protocol_sha256"] = canonical_sha256(
        value, exclude_fields=("protocol_sha256",)
    )
    return value


def build(output: Path) -> dict[str, Any]:
    output = output.resolve(strict=False)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.work.", dir=output.parent))
    try:
        excluded_path = staging / "excluded_assets.jsonl"
        with excluded_path.open("w", encoding="utf-8") as exclusion_handle:
            baseline_datasets, baseline_ledger = select_baselines(exclusion_handle)
            pva_dataset, pva_ledger = select_pva(exclusion_handle)
        datasets = [*baseline_datasets, pva_dataset]
        selected_ledger = [*baseline_ledger, *pva_ledger]
        frozen_protocol = protocol()
        manifest: dict[str, Any] = {
            "schema_version": "table5_v2_core200_manifest_v1",
            "protocol": frozen_protocol,
            "protocol_sha256": frozen_protocol["protocol_sha256"],
            "sample_size_per_dataset": SAMPLE_SIZE,
            "dataset_count": len(DATASET_SLUGS),
            "total_rows": SAMPLE_SIZE * len(DATASET_SLUGS),
            "ordered_dataset_slugs": list(DATASET_SLUGS),
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
        _write_json(staging / "manifest.json", manifest)
        with (staging / "selected_assets.jsonl").open("w", encoding="utf-8") as handle:
            for row in selected_ledger:
                _write_jsonl_row(handle, row)
        summary = {
            "schema_version": "table5_v2_core200_summary_v1",
            "cohort_sha256": manifest["cohort_sha256"],
            "manifest_sha256": manifest["manifest_sha256"],
            "datasets": [
                {
                    "dataset_slug": dataset["dataset_slug"],
                    "candidate_count": dataset["universe"]["candidate_count"],
                    **dataset["selection"],
                }
                for dataset in datasets
            ],
        }
        _write_json(staging / "summary.json", summary)
        (staging / "README.md").write_text(
            _summary_markdown(manifest), encoding="utf-8"
        )
        os.replace(staging, output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return manifest


def _summary_markdown(manifest: Mapping[str, Any]) -> str:
    lines = [
        "# Table 5 v2 Core-200 cohort\n",
        "\n",
        "Selection is frozen before simulation and never uses previous runtime outcomes. ",
        "All seven datasets use `1 <= movable joints <= 20` and the same declared ",
        "semantic exclusions.\n\n",
        "| Dataset | Candidate | Eligible | Excluded | Selected |\n",
        "|---|---:|---:|---:|---:|\n",
    ]
    for dataset in manifest["datasets"]:
        lines.append(
            f"| {dataset['dataset_name']} | {dataset['universe']['candidate_count']} "
            f"| {dataset['selection']['eligible_count']} "
            f"| {dataset['selection']['excluded_count']} "
            f"| {dataset['selection']['selected_count']} |\n"
        )
    lines.extend(
        [
            "\n",
            f"- Protocol: `{manifest['protocol']['protocol_id']}`\n",
            f"- Seed: `{manifest['protocol']['selection_seed']}`\n",
            f"- Cohort SHA256: `{manifest['cohort_sha256']}`\n",
            f"- Manifest SHA256: `{manifest['manifest_sha256']}`\n",
        ]
    )
    return "".join(lines)


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    if manifest.get("schema_version") != "table5_v2_core200_manifest_v1":
        raise CohortError("manifest schema mismatch")
    protocol_value = manifest.get("protocol")
    if not isinstance(protocol_value, Mapping):
        raise CohortError("manifest protocol is missing")
    protocol_hash = canonical_sha256(
        protocol_value, exclude_fields=("protocol_sha256",)
    )
    if (
        protocol_value.get("protocol_sha256") != protocol_hash
        or manifest.get("protocol_sha256") != protocol_hash
    ):
        raise CohortError("protocol hash mismatch")
    datasets = manifest.get("datasets")
    if not isinstance(datasets, list):
        raise CohortError("manifest datasets are missing")
    observed_order = [dataset.get("dataset_slug") for dataset in datasets]
    if observed_order not in (
        list(DATASET_SLUGS),
        list(EXTENDED_DATASET_SLUGS),
    ):
        raise CohortError("dataset order mismatch")
    all_identities: list[list[str]] = []
    for dataset in datasets:
        slug = str(dataset["dataset_slug"])
        rows = dataset.get("rows")
        if not isinstance(rows, list) or len(rows) != SAMPLE_SIZE:
            raise CohortError(f"{slug} does not contain exactly {SAMPLE_SIZE} rows")
        ranks: list[str] = []
        asset_ids: list[str] = []
        seen: set[str] = set()
        for order, row in enumerate(rows):
            if not isinstance(row, Mapping):
                raise CohortError(f"{slug} selected row {order} is invalid")
            asset_id = str(row.get("asset_id") or "")
            if not asset_id or asset_id in seen:
                raise CohortError(f"{slug} duplicate/invalid asset_id: {asset_id!r}")
            seen.add(asset_id)
            joint_count = _movable_joint_count(row)
            if (
                dataset_eligibility_reason(
                    slug, row.get("category"), asset_id, joint_count
                )
                is not None
            ):
                raise CohortError(f"{slug}/{asset_id} violates frozen eligibility")
            selection = row.get("cohort_selection")
            if not isinstance(selection, Mapping):
                raise CohortError(f"{slug}/{asset_id} lacks selection binding")
            rank = str(selection.get("rank_sha256") or "")
            expected_rank = selection_rank(
                slug,
                asset_id,
                universe_sha256=str(dataset["universe"]["sha256"]),
            )
            if rank != expected_rank or selection.get("selection_order") != order:
                raise CohortError(f"{slug}/{asset_id} rank/order binding mismatch")
            ranks.append(rank)
            asset_ids.append(asset_id)
            all_identities.append([slug, asset_id])
        if ranks != sorted(ranks):
            raise CohortError(f"{slug} selected rows are not rank-sorted")
        if dataset["selection"].get("ordered_asset_ids_sha256") != canonical_sha256(
            asset_ids
        ):
            raise CohortError(f"{slug} ordered asset hash mismatch")
        if dataset["selection"].get("ordered_ranks_sha256") != canonical_sha256(ranks):
            raise CohortError(f"{slug} ordered rank hash mismatch")
    if manifest.get("total_rows") != len(all_identities):
        raise CohortError("manifest total_rows mismatch")
    cohort_hash = canonical_sha256(
        [
            {
                "dataset_slug": dataset["dataset_slug"],
                "universe_sha256": dataset["universe"]["sha256"],
                "ordered_asset_ids_sha256": dataset["selection"][
                    "ordered_asset_ids_sha256"
                ],
                "ordered_ranks_sha256": dataset["selection"]["ordered_ranks_sha256"],
            }
            for dataset in datasets
        ]
    )
    if manifest.get("cohort_sha256") != cohort_hash:
        raise CohortError("cohort hash mismatch")
    manifest_hash = canonical_sha256(manifest, exclude_fields=("manifest_sha256",))
    if manifest.get("manifest_sha256") != manifest_hash:
        raise CohortError("manifest hash mismatch")


def _source_universe_file_sha256(universe: Mapping[str, Any]) -> str:
    """Return the byte-level hash for the universe file bound by ``path``."""

    value = universe.get("roster_jsonl_sha256", universe.get("sha256"))
    if not isinstance(value, str) or len(value) != 64:
        raise CohortError("source universe file SHA256 is malformed")
    return value


def verify(path: Path, *, verify_universe_files: bool = True) -> dict[str, Any]:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CohortError(f"cannot load manifest: {error}") from error
    validate_manifest(manifest)
    if verify_universe_files:
        for dataset in manifest["datasets"]:
            universe = dataset["universe"]
            observed = sha256_file(Path(str(universe["path"])))
            expected = _source_universe_file_sha256(universe)
            if observed != expected:
                raise CohortError(
                    f"{dataset['dataset_slug']} source universe file SHA256 mismatch"
                )
    return manifest


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--verify",
        type=Path,
        help="verify an existing manifest instead of building a new cohort",
    )
    parser.add_argument(
        "--skip-source-hashes",
        action="store_true",
        help="with --verify, skip re-hashing the bound source universe files",
    )
    arguments = parser.parse_args(argv)
    try:
        if arguments.verify is not None:
            manifest = verify(
                arguments.verify,
                verify_universe_files=not arguments.skip_source_hashes,
            )
            output = arguments.verify
        else:
            manifest = build(arguments.out)
            output = arguments.out / "manifest.json"
    except (CohortError, FileExistsError, OSError) as error:
        parser.error(str(error))
    print(
        json.dumps(
            {
                "cohort_sha256": manifest["cohort_sha256"],
                "manifest": str(output.resolve(strict=False)),
                "manifest_sha256": manifest["manifest_sha256"],
                "total_rows": manifest["total_rows"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
