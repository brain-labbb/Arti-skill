#!/usr/bin/env python3
"""Compose a fixed eight-dataset Table 5 v2 targeted-smoke manifest."""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import table5_n200_runtime as runtime
import table5_v2_prepare as prepare
from table5_n200_manifest import canonical_sha256, sha256_file
from table5_v2_resample_five_full_release import (
    DEFAULT_OUTPUT as DEFAULT_COHORT_DIR,
    PRESERVED_SLUGS,
    RESAMPLED_SLUGS,
)


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
DEFAULT_COHORT = DEFAULT_COHORT_DIR / "cohort_manifest.json"
DEFAULT_FIVE_PREPARED = (
    REPO_ROOT
    / "exp/runtime/table5_v2_five_full_release_genesis_smoke_v1/prepared/manifest.json"
)
DEFAULT_REUSED_PREPARED = (
    REPO_ROOT
    / "exp/runtime/table5_v2_core200_prepared_articraft10787_infinigen_released_native_official/manifest.json"
)
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_eight_full_release_three_sim_smoke_v1/prepared/manifest.json"
)
SMOKE_ROWS_PER_DATASET = 2


class ComposeError(RuntimeError):
    """Raised when smoke rows cannot be bound to the new frozen cohort."""


def _prepared_hash(datasets: Sequence[Mapping[str, Any]]) -> str:
    return canonical_sha256(
        [
            {
                "dataset_slug": dataset["dataset_slug"],
                "rows": [
                    {
                        "asset_id": row["asset_id"],
                        "row_sha256": row["row_sha256"],
                        "urdf_sha256": row["urdf_sha256"],
                        "physics_receipt_sha256": row["physics"].get(
                            "receipt_sha256",
                            row["physics"].get("physics_plan_sha256"),
                        ),
                        "bbox_receipt_sha256": row["bounding_box"][
                            "receipt_sha256"
                        ],
                    }
                    for row in dataset["rows"]
                ],
            }
            for dataset in datasets
        ]
    )


def _by_slug(groups: Sequence[Mapping[str, Any]]) -> dict[str, Mapping[str, Any]]:
    result = {str(group["dataset_slug"]): group for group in groups}
    if len(result) != len(groups):
        raise ComposeError("duplicate dataset slug")
    return result


def compose(
    cohort_path: Path,
    five_prepared_path: Path,
    reused_prepared_path: Path,
    output_path: Path,
) -> dict[str, Any]:
    output_path = output_path.resolve(strict=False)
    if output_path.exists():
        raise FileExistsError(output_path)

    cohort_path = cohort_path.resolve(strict=True)
    cohort = prepare._load_cohort(cohort_path)
    cohort_by_slug = _by_slug(cohort["datasets"])

    five_prepared_path = five_prepared_path.resolve(strict=True)
    five_bundle = runtime.load_manifest(five_prepared_path)
    five = five_bundle.raw
    if five.get("formal_result_eligible") is not False:
        raise ComposeError("five-dataset input is not marked smoke-only")
    if five.get("source_cohort", {}).get("manifest_sha256") != cohort["manifest_sha256"]:
        raise ComposeError("five-dataset input is bound to another cohort")
    five_by_slug = _by_slug(five["datasets"])
    if tuple(five_by_slug) != RESAMPLED_SLUGS:
        raise ComposeError("five-dataset input order/content mismatch")

    reused_prepared_path = reused_prepared_path.resolve(strict=True)
    reused = prepare.verify(reused_prepared_path, verify_files=False)
    reused_by_slug = _by_slug(reused["datasets"])
    reused_cohort_path = Path(str(reused["source_cohort"]["path"])).resolve(
        strict=True
    )
    reused_cohort = prepare._load_cohort(reused_cohort_path)
    reused_cohort_by_slug = _by_slug(reused_cohort["datasets"])
    for slug in PRESERVED_SLUGS:
        if cohort_by_slug[slug] != reused_cohort_by_slug[slug]:
            raise ComposeError(f"preserved cohort changed: {slug}")

    datasets: list[dict[str, Any]] = []
    for source_dataset in cohort["datasets"]:
        slug = str(source_dataset["dataset_slug"])
        prepared_source = (
            five_by_slug[slug] if slug in RESAMPLED_SLUGS else reused_by_slug[slug]
        )
        rows = deepcopy(list(prepared_source["rows"][:SMOKE_ROWS_PER_DATASET]))
        expected_ids = [
            str(row["asset_id"])
            for row in source_dataset["rows"][:SMOKE_ROWS_PER_DATASET]
        ]
        if [str(row["asset_id"]) for row in rows] != expected_ids:
            raise ComposeError(f"prepared smoke identities differ from cohort: {slug}")
        datasets.append(
            {
                "dataset_slug": slug,
                "dataset_name": source_dataset["dataset_name"],
                "selection": {
                    "kind": "first_two_rows_of_frozen_sha256_rank",
                    "source_selection": deepcopy(source_dataset["selection"]),
                    "selected_count": len(rows),
                    "ordered_asset_ids": expected_ids,
                },
                "rows": rows,
                "preparation_summary": {
                    "row_count": len(rows),
                    "simulator_eligible_count": sum(
                        bool(row["preflight"]["simulator_eligible"]) for row in rows
                    ),
                    "physics_ready_count": sum(
                        row["physics"]["status"] == "ready" for row in rows
                    ),
                    "bbox_available_count": sum(
                        row["bounding_box"]["status"] == "available" for row in rows
                    ),
                },
            }
        )

    protocol = prepare._protocol(cohort)
    manifest: dict[str, Any] = {
        "schema_version": prepare.SCHEMA_VERSION,
        "manifest_kind": "table5_v2_eight_dataset_targeted_smoke_manifest_v1",
        "formal_result_eligible": False,
        "source_cohort": {
            "path": str(cohort_path),
            "manifest_sha256": cohort["manifest_sha256"],
            "cohort_sha256": cohort["cohort_sha256"],
        },
        "smoke_composition": {
            "script_path": str(SCRIPT_PATH),
            "script_sha256": sha256_file(SCRIPT_PATH),
            "resampled_prepared_path": str(five_prepared_path),
            "resampled_prepared_manifest_sha256": five["manifest_sha256"],
            "reused_prepared_path": str(reused_prepared_path),
            "reused_prepared_manifest_sha256": reused["manifest_sha256"],
            "reused_source_cohort_manifest_sha256": reused_cohort[
                "manifest_sha256"
            ],
            "resampled_dataset_slugs": list(RESAMPLED_SLUGS),
            "preserved_dataset_slugs": list(PRESERVED_SLUGS),
        },
        "protocol": protocol,
        "protocol_sha256": protocol["protocol_sha256"],
        "sample_size": SMOKE_ROWS_PER_DATASET,
        "dataset_count": len(datasets),
        "total_rows": sum(len(dataset["rows"]) for dataset in datasets),
        "ordered_dataset_slugs": [dataset["dataset_slug"] for dataset in datasets],
        "datasets": datasets,
    }
    manifest["prepared_cohort_sha256"] = _prepared_hash(datasets)
    manifest["manifest_sha256"] = canonical_sha256(
        manifest, exclude_fields=("manifest_sha256",)
    )
    prepare.atomic_write_json(output_path, manifest)
    runtime.load_manifest(output_path)
    return manifest


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cohort", type=Path, default=DEFAULT_COHORT)
    parser.add_argument("--five-prepared", type=Path, default=DEFAULT_FIVE_PREPARED)
    parser.add_argument("--reused-prepared", type=Path, default=DEFAULT_REUSED_PREPARED)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    try:
        manifest = compose(
            arguments.cohort,
            arguments.five_prepared,
            arguments.reused_prepared,
            arguments.out,
        )
    except (ComposeError, FileExistsError, OSError, prepare.PrepareError) as error:
        parser.error(str(error))
    print(
        json.dumps(
            {
                "manifest": str(arguments.out.resolve()),
                "manifest_sha256": manifest["manifest_sha256"],
                "prepared_cohort_sha256": manifest["prepared_cohort_sha256"],
                "dataset_count": manifest["dataset_count"],
                "total_rows": manifest["total_rows"],
                "formal_result_eligible": manifest["formal_result_eligible"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
