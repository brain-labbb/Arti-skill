#!/usr/bin/env python3
"""Prepare two fixed smoke rows for each resampled Table 5 baseline."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import table5_n200_runtime as runtime
import table5_v2_prepare as prepare
from table5_n200_manifest import canonical_sha256
from table5_v2_resample_five_full_release import (
    DEFAULT_OUTPUT as DEFAULT_COHORT_DIR,
    RESAMPLED_SLUGS,
)


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
DEFAULT_COHORT = DEFAULT_COHORT_DIR / "cohort_manifest.json"
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_five_full_release_genesis_smoke_v1/prepared"
)
SMOKE_ROWS_PER_DATASET = 2


class SmokePrepareError(RuntimeError):
    """Raised when the fixed targeted smoke manifest cannot be prepared."""


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


def build(cohort_path: Path, output: Path, *, workers: int) -> dict[str, Any]:
    if isinstance(workers, bool) or not isinstance(workers, int) or workers < 1:
        raise SmokePrepareError("workers must be a positive integer")
    output = output.resolve(strict=False)
    manifest_path = output / "manifest.json"
    if manifest_path.exists():
        raise FileExistsError(manifest_path)
    output.mkdir(parents=True, exist_ok=True)

    cohort_path = cohort_path.resolve(strict=True)
    cohort = prepare._load_cohort(cohort_path)
    source = {
        str(dataset["dataset_slug"]): dataset for dataset in cohort["datasets"]
    }
    jobs: list[tuple[str, str, int, Mapping[str, Any]]] = []
    for slug in RESAMPLED_SLUGS:
        dataset = source[slug]
        for order, raw in enumerate(dataset["rows"][:SMOKE_ROWS_PER_DATASET]):
            jobs.append((slug, str(dataset["dataset_name"]), order, raw))

    rows_by_key: dict[tuple[str, int], dict[str, Any]] = {}
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(
                prepare._prepare_row,
                raw,
                dataset_slug=slug,
                dataset_name=name,
                selection_order=order,
                output=output,
            ): (slug, order)
            for slug, name, order, raw in jobs
        }
        for future in as_completed(futures):
            rows_by_key[futures[future]] = future.result()

    datasets: list[dict[str, Any]] = []
    for slug in RESAMPLED_SLUGS:
        source_dataset = source[slug]
        rows = [
            rows_by_key[(slug, order)]
            for order in range(SMOKE_ROWS_PER_DATASET)
        ]
        datasets.append(
            {
                "dataset_slug": slug,
                "dataset_name": source_dataset["dataset_name"],
                "selection": {
                    "kind": "first_two_rows_of_frozen_sha256_rank",
                    "source_selection": deepcopy(source_dataset["selection"]),
                    "selected_count": len(rows),
                    "ordered_asset_ids": [row["asset_id"] for row in rows],
                },
                "rows": rows,
                "preparation_summary": {
                    "row_count": len(rows),
                    "simulator_eligible_count": sum(
                        bool(row["preflight"]["simulator_eligible"])
                        for row in rows
                    ),
                    "physics_ready_count": sum(
                        row["physics"]["status"] == "ready" for row in rows
                    ),
                    "bbox_available_count": sum(
                        row["bounding_box"]["status"] == "available"
                        for row in rows
                    ),
                },
            }
        )

    protocol = prepare._protocol(cohort)
    manifest: dict[str, Any] = {
        "schema_version": prepare.SCHEMA_VERSION,
        "manifest_kind": "table5_v2_targeted_smoke_manifest_v1",
        "formal_result_eligible": False,
        "source_cohort": {
            "path": str(cohort_path),
            "manifest_sha256": cohort["manifest_sha256"],
            "cohort_sha256": cohort["cohort_sha256"],
        },
        "protocol": protocol,
        "protocol_sha256": protocol["protocol_sha256"],
        "sample_size": SMOKE_ROWS_PER_DATASET,
        "dataset_count": len(datasets),
        "total_rows": sum(len(dataset["rows"]) for dataset in datasets),
        "ordered_dataset_slugs": list(RESAMPLED_SLUGS),
        "datasets": datasets,
    }
    manifest["prepared_cohort_sha256"] = _prepared_hash(datasets)
    manifest["manifest_sha256"] = canonical_sha256(
        manifest, exclude_fields=("manifest_sha256",)
    )
    prepare.atomic_write_json(manifest_path, manifest)
    runtime.load_manifest(manifest_path)
    prepare.atomic_write_json(
        output / "preparation_summary.json",
        {
            "schema_version": "table5_v2_targeted_smoke_preparation_summary_v1",
            "formal_result_eligible": False,
            "manifest_sha256": manifest["manifest_sha256"],
            "prepared_cohort_sha256": manifest["prepared_cohort_sha256"],
            "datasets": [
                {
                    "dataset_slug": dataset["dataset_slug"],
                    **dataset["preparation_summary"],
                }
                for dataset in datasets
            ],
        },
    )
    return manifest


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cohort", type=Path, default=DEFAULT_COHORT)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=5)
    arguments = parser.parse_args(argv)
    try:
        manifest = build(arguments.cohort, arguments.out, workers=arguments.workers)
    except (FileExistsError, OSError, SmokePrepareError, prepare.PrepareError) as error:
        parser.error(str(error))
    print(
        json.dumps(
            {
                "manifest": str((arguments.out / "manifest.json").resolve()),
                "manifest_sha256": manifest["manifest_sha256"],
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
