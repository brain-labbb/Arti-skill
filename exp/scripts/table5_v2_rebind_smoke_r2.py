#!/usr/bin/env python3
"""Bind an existing smoke-only prepared manifest to revision-2 metrics.

This does not prepare or alter any asset row. It creates a new smoke-only
manifest whose protocol and implementation hashes point at the r2 evaluator.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import table5_v2_prepare as prepare_core
import table5_v2_prepare_r2 as prepare_r2
import table5_v2_runtime as runtime_core
import table5_v2_runtime_r2 as runtime_r2
from table5_n200_manifest import canonical_sha256, sha256_file
from table5_v2_sample_n200 import verify as verify_cohort


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
DEFAULT_COHORT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_core200_five_full_release_articraft10787_infinigen_paired_official/cohort_manifest.json"
)
DEFAULT_SOURCE = (
    REPO_ROOT
    / "exp/runtime/table5_v2_eight_full_release_three_sim_smoke_v1/prepared/manifest.json"
)
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_eight_full_release_three_sim_smoke_r2/prepared/manifest.json"
)


class RebindError(RuntimeError):
    """Raised when the smoke inputs cannot be rebound without changing rows."""


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RebindError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise RebindError(f"JSON root is not an object: {path}")
    return value


def _by_slug(groups: Sequence[Mapping[str, Any]]) -> dict[str, Mapping[str, Any]]:
    result = {str(group["dataset_slug"]): group for group in groups}
    if len(result) != len(groups):
        raise RebindError("duplicate dataset slug")
    return result


def _verify_sources(manifest: Mapping[str, Any]) -> None:
    for dataset in manifest["datasets"]:
        slug = str(dataset["dataset_slug"])
        for row in dataset["rows"]:
            if row.get("row_sha256") != canonical_sha256(
                row, exclude_fields=("row_sha256",)
            ):
                raise RebindError(
                    f"prepared row hash mismatch: {slug}/{row.get('asset_id')}"
                )
            sources = row.get("simulator_sources")
            if not isinstance(sources, Mapping):
                raise RebindError(
                    f"simulator sources missing: {slug}/{row.get('asset_id')}"
                )
            for simulator in runtime_core.SIMULATORS:
                source = sources.get(simulator)
                if not isinstance(source, Mapping):
                    raise RebindError(
                        f"{simulator} source missing: {slug}/{row.get('asset_id')}"
                    )
                path = Path(str(source.get("path") or "")).resolve(strict=True)
                if sha256_file(path) != source.get("sha256"):
                    raise RebindError(
                        f"{simulator} source hash mismatch: {slug}/{row.get('asset_id')}"
                    )


def rebind(cohort_path: Path, source_path: Path, output_path: Path) -> dict[str, Any]:
    cohort_path = cohort_path.resolve(strict=True)
    source_path = source_path.resolve(strict=True)
    output_path = output_path.resolve(strict=False)
    if output_path.exists():
        raise FileExistsError(output_path)

    cohort = verify_cohort(cohort_path, verify_universe_files=True)
    source_bundle = runtime_core._runtime.load_manifest(source_path)
    source = source_bundle.raw
    if source.get("formal_result_eligible") is not False:
        raise RebindError("source manifest is not smoke-only")
    if source.get("source_cohort", {}).get("manifest_sha256") != cohort.get(
        "manifest_sha256"
    ):
        raise RebindError("source smoke manifest is bound to another cohort")
    if source.get("total_rows") != 16 or source.get("dataset_count") != 8:
        raise RebindError("source smoke manifest is not the expected 8 x 2 cohort")

    cohort_by_slug = _by_slug(cohort["datasets"])
    source_by_slug = _by_slug(source["datasets"])
    if tuple(source_by_slug) != tuple(cohort_by_slug):
        raise RebindError("source smoke dataset order differs from the cohort")
    for slug, dataset in source_by_slug.items():
        observed = [str(row["asset_id"]) for row in dataset["rows"]]
        expected = [
            str(row["asset_id"])
            for row in cohort_by_slug[slug]["rows"][: len(observed)]
        ]
        if observed != expected or len(observed) != 2:
            raise RebindError(f"source smoke identities differ from the cohort: {slug}")
    _verify_sources(source)

    rebound = deepcopy(source)
    rebound["manifest_kind"] = "table5_v2_eight_dataset_targeted_smoke_manifest_r2"
    rebound["formal_result_eligible"] = False
    rebound["protocol"] = prepare_r2._protocol(cohort)
    rebound["protocol_sha256"] = rebound["protocol"]["protocol_sha256"]
    rebound["smoke_protocol_rebinding"] = {
        "schema_version": "table5_v2_smoke_protocol_rebinding_v1",
        "source_manifest_path": str(source_path),
        "source_manifest_sha256": source["manifest_sha256"],
        "script_path": str(SCRIPT_PATH),
        "script_sha256": sha256_file(SCRIPT_PATH),
        "asset_rows_changed": False,
    }
    rebound.pop("manifest_sha256", None)
    rebound["manifest_sha256"] = canonical_sha256(
        rebound, exclude_fields=("manifest_sha256",)
    )
    prepare_core.atomic_write_json(output_path, rebound)

    runtime_r2.install()
    checked = runtime_r2._core._runtime.load_manifest(output_path).raw
    if checked["manifest_sha256"] != rebound["manifest_sha256"]:
        raise RebindError("written r2 smoke manifest failed runtime validation")
    return rebound


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cohort", type=Path, default=DEFAULT_COHORT)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    try:
        manifest = rebind(arguments.cohort, arguments.source, arguments.out)
    except (FileExistsError, OSError, RebindError) as error:
        parser.error(str(error))
    print(
        json.dumps(
            {
                "formal_result_eligible": manifest["formal_result_eligible"],
                "manifest": str(arguments.out.resolve()),
                "manifest_sha256": manifest["manifest_sha256"],
                "protocol_sha256": manifest["protocol_sha256"],
                "total_rows": manifest["total_rows"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
