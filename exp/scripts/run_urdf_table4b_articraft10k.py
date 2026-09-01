#!/usr/bin/env python3
"""Fail-closed Table 4b runner for the frozen Articraft-10K N=800 cohort."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_urdf_table4b_artiverse as base  # noqa: E402


SCHEMA_VERSION = "table4b-articraft10k/v1"
PROTOCOL_ID = "table4b_articraft10k_table2cohort_n800_seed20260813_v1"
DATASET = "Articraft-10K"
CLASSIFICATION = "FORMAL"
SOURCE_MANIFEST = (
    REPO
    / "exp/runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json"
)
EXPECTED_SOURCE_MANIFEST_SHA256 = (
    "13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d"
)
EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256 = (
    "576852cb6da00775e1c51360b82b4be40e0a614e4fb0cfb1bae066912eed56a3"
)
EXPECTED_ORDERED_IDS_SHA256 = (
    "79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784"
)
DATASET_PACKAGES_ROOT = REPO / "exp/Articraft-10K/released_urdf"
N_EVAL = 800
FORMAL_OUTPUT_NAME = (
    "table4b_urdf_articraft10k_table2cohort_n800_seed20260813_{timestamp}"
)
SMOKE_OUTPUT_NAME = "table4b_urdf_articraft10k_smoke_n{n}_{timestamp}"
SUMMARY_TITLE = (
    "Table 4b - Articraft-10K (frozen Table 2 cohort, N=800; "
    "exact collision-representation geometry)"
)
SELECTION_POLICY = (
    "all .records[].package entries from the frozen Table 2 Articraft-10K manifest in "
    "existing order; no resampling, replacement, or result-based filtering"
)


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return sha256_bytes(payload.encode("utf-8"))


def compute_package_binding(package: Path) -> dict[str, Any]:
    package = package.resolve(strict=True)
    if not package.is_dir():
        raise NotADirectoryError(package)
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if child.is_symlink():
                raise ValueError(f"package contains directory symlink: {child.relative_to(package)}")
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            resolved = path.resolve(strict=True)
            try:
                resolved.relative_to(package)
            except ValueError as error:
                raise ValueError(f"package file escapes package: {relative}") from error
            if not resolved.is_file():
                raise ValueError(f"package entry is not a regular file: {relative}")
            payload = resolved.read_bytes()
            files.append(
                {"path": relative, "bytes": len(payload), "sha256": sha256_bytes(payload)}
            )
    return {
        "file_count": len(files),
        "total_bytes": sum(item["bytes"] for item in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def verify_package_binding(package: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    observed = compute_package_binding(package)
    if observed != dict(expected):
        raise ValueError(f"package binding mismatch: {package}")
    return observed


def load_source_manifest() -> dict[str, Any]:
    payload = SOURCE_MANIFEST.read_bytes()
    digest = sha256_bytes(payload)
    if digest != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise SystemExit(f"source manifest sha256 mismatch: {digest}")
    manifest = json.loads(payload)
    if manifest.get("manifest_content_sha256") != EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256:
        raise SystemExit("source manifest content sha256 mismatch")
    records = manifest.get("records")
    if not isinstance(records, list) or len(records) != N_EVAL:
        raise SystemExit(f"source manifest must contain exactly {N_EVAL} records")
    ids = [str(record["asset_id"]) for record in records]
    ordered = sha256_bytes(json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode())
    if ordered != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit(f"ordered asset id sha256 mismatch: {ordered}")
    for index, record in enumerate(records):
        if int(record.get("selection_index", -1)) != index:
            raise SystemExit(f"record selection_index mismatch at index {index}")
    return manifest


def build_jobs(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    jobs: list[dict[str, Any]] = []
    packages_root = DATASET_PACKAGES_ROOT.resolve(strict=True)
    for index, record in enumerate(manifest["records"]):
        package = Path(str(record["package"])).resolve(strict=True)
        try:
            package.relative_to(packages_root)
        except ValueError as error:
            raise SystemExit(f"package escapes frozen Articraft root: {package}") from error
        if package.name != str(record["asset_id"]):
            raise SystemExit(f"package/asset_id mismatch at index {index}")
        verify_package_binding(package, record["package_binding"])
        urdf_path = package / "model.urdf"
        jobs.append(
            {
                "selection_index": index,
                "dataset_id": str(record["asset_id"]),
                "asset_id": str(record["asset_id"]),
                "category": "unavailable",
                "package": str(package),
                "urdf_path": str(urdf_path),
                "expected_urdf_sha256": str(record["model_urdf_sha256"]),
                "input_identity_sha256": str(
                    record["package_binding"]["content_manifest_sha256"]
                ),
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

    source_records = manifest["records"]
    check(
        "source_manifest_sha256",
        sha256_bytes(SOURCE_MANIFEST.read_bytes()) == EXPECTED_SOURCE_MANIFEST_SHA256,
    )
    ids = [str(record["asset_id"]) for record in source_records]
    ordered = sha256_bytes(json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode())
    check("ordered_ids_sha256", ordered == EXPECTED_ORDERED_IDS_SHA256, ordered)
    check("record_count", len(records) == N_EVAL, str(len(records)))
    check(
        "frozen_order_preserved",
        all(
            int(record["selection_index"]) == index
            and str(record["asset_id"]) == str(source_records[index]["asset_id"])
            for index, record in enumerate(records)
        ),
    )
    check(
        "urdf_identity_matches_frozen_manifest",
        all(
            (record.get("urdf_sha256") is None and record.get("status") != "completed")
            or record.get("urdf_sha256")
            == str(source_records[int(record["selection_index"])] ["model_urdf_sha256"])
            for record in records
        ),
    )
    recomputed = base.aggregate(records)
    check(
        "aggregate_recomputation_matches",
        json.dumps(recomputed, sort_keys=True) == json.dumps(dict(aggregates), sort_keys=True),
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
        "CPU_AFFINITY_ENV": "TABLE4B_ARTICRAFT10K_CPU_AFFINITY",
        "FORMAL_OUTPUT_NAME": FORMAL_OUTPUT_NAME,
        "SMOKE_OUTPUT_NAME": SMOKE_OUTPUT_NAME,
        "SUMMARY_TITLE": SUMMARY_TITLE,
        "SELECTION_POLICY": SELECTION_POLICY,
        "load_source_manifest": load_source_manifest,
        "build_jobs": build_jobs,
        "verify_run": verify_run,
    }
    for name, value in overrides.items():
        setattr(base, name, value)


def main() -> int:
    _configure_base()
    return base.main()


_configure_base()

if __name__ == "__main__":
    raise SystemExit(main())
