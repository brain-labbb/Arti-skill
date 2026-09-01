#!/usr/bin/env python3
"""Fail-closed Table 4b runner for the frozen PartNet-Mobility N=800 cohort."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_urdf_table4b_artiverse as base  # noqa: E402


SCHEMA_VERSION = "table4b-partnet-mobility/v1"
PROTOCOL_ID = "table4b_partnet_mobility_table4cohort_n800_salt20260813_v1"
DATASET = "PartNet-Mobility"
CLASSIFICATION = "FORMAL"
SOURCE_MANIFEST = (
    REPO
    / "exp/runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json"
)
EXPECTED_SOURCE_MANIFEST_SHA256 = (
    "2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900"
)
EXPECTED_MANIFEST_ORDERED_IDS_SHA256 = (
    "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883"
)
EXPECTED_ORDERED_IDS_SHA256 = (
    "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883"
)
DATASET_ROOT = REPO / "exp/PartNet-Mobility/data/dataset"
N_EVAL = 800
FORMAL_OUTPUT_NAME = (
    "table4b_urdf_partnet_mobility_table4cohort_n800_salt20260813_{timestamp}"
)
SMOKE_OUTPUT_NAME = "table4b_urdf_partnet_mobility_smoke_n{n}_{timestamp}"
SUMMARY_TITLE = (
    "Table 4b - PartNet-Mobility (frozen Table 4 cohort, N=800; "
    "exact collision-representation geometry)"
)
SELECTION_POLICY = (
    "all .items[].dataset_id entries from the frozen PartNet-Mobility Table 4 "
    "manifest in existing order; no resampling, replacement, or result-based filtering"
)


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def load_source_manifest() -> dict[str, Any]:
    payload = SOURCE_MANIFEST.read_bytes()
    digest = sha256_bytes(payload)
    if digest != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise SystemExit(f"source manifest sha256 mismatch: {digest}")
    manifest = json.loads(payload)
    if manifest.get("protocol_id") != "urdf_sim_ready_table4_partnet_mobility_n800_v1":
        raise SystemExit("source manifest protocol_id mismatch")
    if manifest.get("ordered_selected_ids_sha256") != EXPECTED_MANIFEST_ORDERED_IDS_SHA256:
        raise SystemExit("source manifest ordered_selected_ids_sha256 mismatch")
    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != N_EVAL:
        raise SystemExit(f"source manifest must contain exactly {N_EVAL} items")
    if int(manifest.get("sample_size", -1)) != N_EVAL:
        raise SystemExit("source manifest sample_size mismatch")
    dataset_root = Path(str(manifest.get("dataset_root"))).resolve(strict=True)
    if dataset_root != DATASET_ROOT.resolve(strict=True):
        raise SystemExit(f"source manifest dataset_root mismatch: {dataset_root}")
    ids = [str(item["dataset_id"]) for item in items]
    ordered = sha256_bytes(
        json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    )
    if ordered != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit(f"ordered dataset id sha256 mismatch: {ordered}")
    if len(set(ids)) != N_EVAL:
        raise SystemExit("source manifest dataset_id values must be unique")
    for index, item in enumerate(items):
        if int(item.get("order", -1)) != index:
            raise SystemExit(f"item order field mismatch at index {index}")
    return manifest


def build_jobs(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    dataset_root = Path(str(manifest["dataset_root"])).resolve(strict=True)
    expected_root = DATASET_ROOT.resolve(strict=True)
    if dataset_root != expected_root:
        raise SystemExit(f"dataset root mismatch: {dataset_root}")

    jobs: list[dict[str, Any]] = []
    for index, item in enumerate(manifest["items"]):
        dataset_id = str(item["dataset_id"])
        if Path(dataset_id).name != dataset_id or dataset_id in {"", ".", ".."}:
            raise SystemExit(f"dataset_id is not a package basename: {dataset_id!r}")
        package = (dataset_root / dataset_id).resolve(strict=True)
        try:
            package.relative_to(dataset_root)
        except ValueError as error:
            raise SystemExit(f"package escapes frozen PartNet-Mobility root: {package}") from error
        if not package.is_dir() or package.name != dataset_id:
            raise SystemExit(f"dataset_id/package mismatch at index {index}")
        urdf_path = (package / "mobility.urdf").resolve(strict=True)
        try:
            urdf_path.relative_to(package)
        except ValueError as error:
            raise SystemExit(
                f"URDF escapes frozen PartNet-Mobility package: {urdf_path}"
            ) from error
        jobs.append(
            {
                "selection_index": index,
                "dataset_id": dataset_id,
                "asset_id": dataset_id,
                "category": str(item.get("category", "unavailable")),
                "package": str(package),
                "urdf_path": str(urdf_path),
                "expected_urdf_sha256": str(item["urdf_sha256"]),
                "input_identity_sha256": str(item["input_identity_sha256"]),
            }
        )
    return jobs


def verify_run(
    manifest: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
    aggregates: Mapping[str, Any],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        checks.append({"check": name, "pass": bool(ok), "detail": detail})

    source_items = manifest["items"]
    check(
        "source_manifest_sha256",
        sha256_bytes(SOURCE_MANIFEST.read_bytes()) == EXPECTED_SOURCE_MANIFEST_SHA256,
    )
    ids = [str(item["dataset_id"]) for item in source_items]
    ordered = sha256_bytes(
        json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    )
    check("ordered_ids_sha256", ordered == EXPECTED_ORDERED_IDS_SHA256, ordered)
    check("record_count", len(records) == N_EVAL, str(len(records)))
    check(
        "frozen_order_preserved",
        all(
            int(record["selection_index"]) == index
            and str(record["dataset_id"]) == str(source_items[index]["dataset_id"])
            and str(record["asset_id"]) == str(source_items[index]["dataset_id"])
            for index, record in enumerate(records)
        ),
    )
    check(
        "urdf_identity_matches_frozen_manifest",
        all(
            (record.get("urdf_sha256") is None and record.get("status") != "completed")
            or record.get("urdf_sha256")
            == str(source_items[int(record["selection_index"])] ["urdf_sha256"])
            for record in records
        ),
    )
    recomputed = base.aggregate(records)
    check(
        "aggregate_recomputation_matches",
        json.dumps(recomputed, sort_keys=True)
        == json.dumps(dict(aggregates), sort_keys=True),
    )
    return {
        "all_pass": all(item["pass"] for item in checks),
        "check_count": len(checks),
        "checks": checks,
    }


def output_directory_name(mode: str, n: int, timestamp: str) -> str:
    if mode == "formal":
        return FORMAL_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    if mode == "smoke":
        return SMOKE_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    raise ValueError(f"unsupported mode: {mode!r}")


def _configure_base() -> None:
    overrides = {
        "SCRIPT": SCRIPT,
        "SCHEMA_VERSION": SCHEMA_VERSION,
        "PROTOCOL_ID": PROTOCOL_ID,
        "DATASET": DATASET,
        "CLASSIFICATION": CLASSIFICATION,
        "SOURCE_MANIFEST": SOURCE_MANIFEST,
        "EXPECTED_SOURCE_MANIFEST_SHA256": EXPECTED_SOURCE_MANIFEST_SHA256,
        "EXPECTED_ORDERED_IDS_SHA256": EXPECTED_ORDERED_IDS_SHA256,
        "N_EVAL": N_EVAL,
        "CPU_AFFINITY_ENV": "TABLE4B_PARTNET_MOBILITY_CPU_AFFINITY",
        "FORMAL_OUTPUT_NAME": FORMAL_OUTPUT_NAME,
        "SMOKE_OUTPUT_NAME": SMOKE_OUTPUT_NAME,
        "SUMMARY_TITLE": SUMMARY_TITLE,
        "SELECTION_POLICY": SELECTION_POLICY,
        "load_source_manifest": load_source_manifest,
        "build_jobs": build_jobs,
        "verify_run": verify_run,
        "output_directory_name": output_directory_name,
    }
    for name, value in overrides.items():
        setattr(base, name, value)


def main() -> int:
    _configure_base()
    return base.main()


_configure_base()

if __name__ == "__main__":
    raise SystemExit(main())
