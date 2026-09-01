#!/usr/bin/env python3
"""Prepare entrypoint for the diagnostic Articraft resample cohort.

Delegates to the frozen revision-2 prepare implementation, but replaces the
cohort loader with a validator that understands the diagnostic quality-
stratified selection protocol (diagnostic seed and per-stratum hash ranks)
for the articraft_10k dataset while still enforcing the frozen bindings for
the seven copied datasets.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_v2_prepare as _core  # noqa: E402
import table5_v2_prepare_r2 as _r2  # noqa: E402
import table5_v2_sample_n200 as _sample  # noqa: E402
from table5_v2_articraft_diagnostic_resample import (  # noqa: E402
    DIAGNOSTIC_PROTOCOL_ID,
    DIAGNOSTIC_SEED,
    diagnostic_rank,
)


def _movable_joint_count(row: Mapping[str, Any]) -> int | None:
    counts = row.get("xml_counts")
    if not isinstance(counts, Mapping):
        return None
    value = counts.get("movable_joints")
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _validate_diagnostic_cohort(manifest: Mapping[str, Any]) -> None:
    if manifest.get("schema_version") != "table5_v2_core200_manifest_v1":
        raise _core.PrepareError("cohort schema mismatch")
    protocol_value = manifest.get("protocol")
    if not isinstance(protocol_value, Mapping):
        raise _core.PrepareError("cohort protocol is missing")
    protocol_hash = _core.canonical_sha256(
        protocol_value, exclude_fields=("protocol_sha256",)
    )
    if (
        protocol_value.get("protocol_sha256") != protocol_hash
        or manifest.get("protocol_sha256") != protocol_hash
    ):
        raise _core.PrepareError("cohort protocol hash mismatch")
    datasets = manifest.get("datasets")
    if not isinstance(datasets, list) or [
        dataset.get("dataset_slug") for dataset in datasets
    ] != list(_sample.EXTENDED_DATASET_SLUGS):
        raise _core.PrepareError("cohort dataset order mismatch")
    all_identities: list[list[str]] = []
    for dataset in datasets:
        slug = str(dataset["dataset_slug"])
        rows = dataset.get("rows")
        if not isinstance(rows, list) or len(rows) != _sample.SAMPLE_SIZE:
            raise _core.PrepareError(f"{slug} does not contain exactly 200 rows")
        universe_sha256 = str(dataset["universe"]["sha256"])
        ranks: list[str] = []
        seen: set[str] = set()
        for order, row in enumerate(rows):
            asset_id = str(row.get("asset_id") or "")
            if not asset_id or asset_id in seen:
                raise _core.PrepareError(f"{slug} duplicate/invalid asset_id")
            seen.add(asset_id)
            joint_count = _movable_joint_count(row)
            if (
                _sample.dataset_eligibility_reason(
                    slug, row.get("category"), asset_id, joint_count
                )
                is not None
            ):
                raise _core.PrepareError(f"{slug}/{asset_id} violates eligibility")
            selection = row.get("cohort_selection")
            if not isinstance(selection, Mapping):
                raise _core.PrepareError(f"{slug}/{asset_id} lacks selection binding")
            rank = str(selection.get("rank_sha256") or "")
            if slug == "articraft_10k":
                if selection.get("protocol_id") != DIAGNOSTIC_PROTOCOL_ID:
                    raise _core.PrepareError(
                        f"{slug}/{asset_id} not selected by the diagnostic protocol"
                    )
                stratum = str(
                    row.get("source_provenance", {}).get("diagnostic_stratum") or ""
                )
                if stratum not in ("F", "C", "I", "R"):
                    raise _core.PrepareError(
                        f"{slug}/{asset_id} missing diagnostic stratum"
                    )
                expected_rank = diagnostic_rank(stratum, asset_id, universe_sha256)
            else:
                expected_rank = _sample.selection_rank(
                    slug, asset_id, universe_sha256=universe_sha256
                )
            if rank != expected_rank or selection.get("selection_order") != order:
                raise _core.PrepareError(f"{slug}/{asset_id} rank/order mismatch")
            if row.get("row_sha256") != _core.canonical_sha256(
                row, exclude_fields=("row_sha256",)
            ):
                raise _core.PrepareError(f"{slug}/{asset_id} row hash mismatch")
            ranks.append(rank)
            all_identities.append([slug, asset_id])
        if ranks != sorted(ranks):
            raise _core.PrepareError(f"{slug} rows are not rank-sorted")
        if dataset["selection"].get("ordered_asset_ids_sha256") != _core.canonical_sha256(
            [row["asset_id"] for row in rows]
        ):
            raise _core.PrepareError(f"{slug} ordered asset hash mismatch")
        if dataset["selection"].get("ordered_ranks_sha256") != _core.canonical_sha256(
            ranks
        ):
            raise _core.PrepareError(f"{slug} ordered ranks hash mismatch")
    if manifest.get("total_rows") != len(all_identities):
        raise _core.PrepareError("cohort total_rows mismatch")
    cohort_hash = _core.canonical_sha256(
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
        raise _core.PrepareError("cohort hash mismatch")
    manifest_hash = _core.canonical_sha256(manifest, exclude_fields=("manifest_sha256",))
    if manifest.get("manifest_sha256") != manifest_hash:
        raise _core.PrepareError("cohort manifest hash mismatch")


def _load_cohort(path: Path) -> dict[str, Any]:
    try:
        cohort = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise _core.PrepareError(f"cannot load cohort: {error}") from error
    if not isinstance(cohort, dict):
        raise _core.PrepareError("cohort must be a JSON object")
    _validate_diagnostic_cohort(cohort)
    return cohort


def main(argv: Sequence[str] | None = None) -> int:
    _r2.install()
    _core._load_cohort = _load_cohort
    return _core.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
