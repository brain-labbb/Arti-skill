#!/usr/bin/env python3
"""Fail-closed Table 4b runner for Brain-500 + deterministic PV-A-300."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_urdf_table4b_artiverse as base  # noqa: E402


SCHEMA_VERSION = "table4b-ours-brain500-pva300/v1"
PROTOCOL_ID = "table4b_ours_brain500_pva300_n800_v1"
DATASET = "Ours-800"
CLASSIFICATION = "FORMAL"
SOURCE_MANIFEST = Path(
    "/root/.cache/torch/arti-skill/"
    "urdf_table4_ours_800_mixed_n800_20260823T075110Z/frozen_manifest.json"
)
COHORT_MANIFEST = Path(
    "/root/.cache/torch/arti-skill/ours_pva_800_cohort_v2/manifest.json"
)
EXPECTED_SOURCE_MANIFEST_SHA256 = (
    "b06d08204019d80d00f95779757d3522d3dcb23b4fbe760c722560e4d217520c"
)
EXPECTED_SOURCE_CONTENT_SHA256 = (
    "fc5f5142fa21c857cf6bc0180bba66d7d507af5b588a38a18e308375a36a4f0f"
)
EXPECTED_COHORT_MANIFEST_SHA256 = (
    "014ac091edf84037a12b044226f384722187167ebc7c47330d51e3b717399b53"
)
EXPECTED_ORDERED_IDS_SHA256 = (
    "f297d85c0de7b00411be80fdfa3b12e71ec21723a4bc0e619df87f229624db39"
)
N_EVAL = 800
FORMAL_OUTPUT_NAME = "table4b_urdf_ours_brain500_pva300_n800_{timestamp}"
SMOKE_OUTPUT_NAME = "table4b_urdf_ours_brain500_pva300_smoke_n{n}_{timestamp}"
SUMMARY_TITLE = (
    "Table 4b - Ours-800 (Brain-500 + deterministic PV-A-300; "
    "exact collision-representation geometry)"
)
SELECTION_POLICY = (
    "all 800 frozen mixed Table 4 items in existing order; no resampling, "
    "replacement, or result filtering"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def ordered_ids_sha256(ids: list[str]) -> str:
    return hashlib.sha256(
        json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode()
    ).hexdigest()


def _manifest_self_hash(manifest: Mapping[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def _load_cohort_assets() -> dict[str, dict[str, Any]]:
    if sha256_file(COHORT_MANIFEST) != EXPECTED_COHORT_MANIFEST_SHA256:
        raise SystemExit("cohort manifest sha256 mismatch")
    manifest = json.loads(COHORT_MANIFEST.read_text(encoding="utf-8"))
    assets = manifest.get("assets")
    if not isinstance(assets, list) or len(assets) != N_EVAL:
        raise SystemExit("cohort manifest must contain 800 assets")
    if ordered_ids_sha256([str(row["dataset_id"]) for row in assets]) != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit("cohort ordered dataset IDs mismatch")
    return {str(row["dataset_id"]): row for row in assets}


def load_source_manifest() -> dict[str, Any]:
    if sha256_file(SOURCE_MANIFEST) != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise SystemExit("source manifest sha256 mismatch")
    manifest = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))
    if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
        raise SystemExit("source manifest self-hash mismatch")
    if manifest.get("manifest_content_sha256") != EXPECTED_SOURCE_CONTENT_SHA256:
        raise SystemExit("source manifest content hash mismatch")
    if manifest.get("protocol_id") != "urdf_sim_ready_table4_ours_brain500_pva300_n800_v1":
        raise SystemExit("source protocol mismatch")
    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != N_EVAL:
        raise SystemExit(f"source manifest must contain {N_EVAL} items")
    ids = [str(item["dataset_id"]) for item in items]
    if ordered_ids_sha256(ids) != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit("ordered dataset IDs mismatch")
    if [item.get("source_component") for item in items[:500]] != ["Brain-500"] * 500:
        raise SystemExit("source Brain component order mismatch")
    if [item.get("source_component") for item in items[500:]] != ["PV-A-300"] * 300:
        raise SystemExit("source PV-A component order mismatch")
    return manifest


def _referenced_mesh_paths(package: Path, urdf: Path) -> list[Path]:
    root = ET.parse(urdf).getroot()
    references = sorted(
        {
            str(mesh.get("filename", "")).replace("\\", "/")
            for mesh in root.findall("link/visual/geometry/mesh")
            + root.findall("link/collision/geometry/mesh")
            if mesh.get("filename")
        }
    )
    paths: list[Path] = []
    package = package.resolve(strict=True)
    for reference in references:
        if "://" in reference or Path(reference).is_absolute():
            raise SystemExit(f"unsafe Table 4b mesh reference: {reference}")
        path = (package / reference).resolve(strict=True)
        try:
            path.relative_to(package)
        except ValueError as error:
            raise SystemExit(f"Table 4b mesh escapes package: {reference}") from error
        paths.append(path)
    return paths


def _validate_relevant_package_files(
    asset_id: str, package: Path, urdf: Path, cohort_row: Mapping[str, Any]
) -> None:
    binding = cohort_row.get("package_binding")
    if not isinstance(binding, dict) or not isinstance(binding.get("files"), list):
        raise SystemExit(f"missing package binding: {asset_id}")
    files = binding["files"]
    if canonical_sha256(files) != binding.get("content_manifest_sha256"):
        raise SystemExit(f"package binding self-hash mismatch: {asset_id}")
    by_path = {str(row["path"]): row for row in files}
    paths = [urdf, *_referenced_mesh_paths(package, urdf)]
    for path in paths:
        relative = path.relative_to(package).as_posix()
        expected = by_path.get(relative)
        if expected is None:
            raise SystemExit(f"unbound Table 4b input: {asset_id}:{relative}")
        if path.is_symlink() or not path.is_file():
            raise SystemExit(f"invalid Table 4b input file: {asset_id}:{relative}")
        if path.stat().st_size != expected.get("bytes") or sha256_file(path) != expected.get("sha256"):
            raise SystemExit(f"Table 4b input binding mismatch: {asset_id}:{relative}")


def build_jobs(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    cohort_assets = _load_cohort_assets()
    jobs: list[dict[str, Any]] = []
    for index, item in enumerate(manifest["items"]):
        if int(item.get("order", -1)) != index:
            raise SystemExit(f"source order mismatch at {index}")
        asset_id = str(item["dataset_id"])
        cohort_row = cohort_assets.get(asset_id)
        if cohort_row is None:
            raise SystemExit(f"asset absent from cohort binding: {asset_id}")
        package = Path(str(item.get("package", ""))).resolve(strict=True)
        if package != Path(str(cohort_row.get("package", ""))).resolve(strict=True):
            raise SystemExit(f"package path binding mismatch: {asset_id}")
        expected_relative = Path(package.name) / "model.urdf"
        if Path(str(item.get("primary_urdf_relpath", ""))) != expected_relative:
            raise SystemExit(f"source URDF relative path mismatch: {asset_id}")
        urdf = (package / "model.urdf").resolve(strict=True)
        if urdf.parent != package or sha256_file(urdf) != item.get("urdf_sha256"):
            raise SystemExit(f"URDF binding mismatch: {asset_id}")
        if (
            item.get("package_binding_content_manifest_sha256")
            != cohort_row.get("package_binding", {}).get("content_manifest_sha256")
        ):
            raise SystemExit(f"package content identity mismatch: {asset_id}")
        _validate_relevant_package_files(asset_id, package, urdf, cohort_row)
        jobs.append(
            {
                "selection_index": index,
                "dataset_id": asset_id,
                "asset_id": asset_id,
                "category": str(item["category"]),
                "source_component": str(item["source_component"]),
                "package": str(package),
                "urdf_path": str(urdf),
                "expected_urdf_sha256": str(item["urdf_sha256"]),
                "input_identity_sha256": str(item["input_identity_sha256"]),
                "package_binding_content_manifest_sha256": str(
                    item["package_binding_content_manifest_sha256"]
                ),
            }
        )
    return jobs


def verify_run(
    manifest: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
    aggregates: Mapping[str, Any],
) -> dict[str, Any]:
    checks = [
        {
            "check": "record_count",
            "pass": len(records) == N_EVAL,
            "detail": str(len(records)),
        },
        {
            "check": "frozen_order_preserved",
            "pass": all(
                record.get("dataset_id") == manifest["items"][index]["dataset_id"]
                for index, record in enumerate(records)
            ),
            "detail": "",
        },
        {
            "check": "aggregate_recomputation_matches",
            "pass": base.aggregate(records) == dict(aggregates),
            "detail": "",
        },
    ]
    return {
        "all_pass": all(check["pass"] for check in checks),
        "check_count": len(checks),
        "checks": checks,
    }


def output_directory_name(mode: str, n: int, timestamp: str) -> str:
    if mode == "formal":
        return FORMAL_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    if mode == "smoke":
        return SMOKE_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    raise ValueError(f"unsupported mode: {mode}")


def _configure_base() -> None:
    values = {
        "SCRIPT": SCRIPT,
        "SCHEMA_VERSION": SCHEMA_VERSION,
        "PROTOCOL_ID": PROTOCOL_ID,
        "DATASET": DATASET,
        "CLASSIFICATION": CLASSIFICATION,
        "SOURCE_MANIFEST": SOURCE_MANIFEST,
        "EXPECTED_SOURCE_MANIFEST_SHA256": EXPECTED_SOURCE_MANIFEST_SHA256,
        "EXPECTED_ORDERED_IDS_SHA256": EXPECTED_ORDERED_IDS_SHA256,
        "N_EVAL": N_EVAL,
        "CPU_AFFINITY_ENV": "TABLE4B_OURS_800_CPU_AFFINITY",
        "FORMAL_OUTPUT_NAME": FORMAL_OUTPUT_NAME,
        "SMOKE_OUTPUT_NAME": SMOKE_OUTPUT_NAME,
        "SUMMARY_TITLE": SUMMARY_TITLE,
        "SELECTION_POLICY": SELECTION_POLICY,
        "load_source_manifest": load_source_manifest,
        "build_jobs": build_jobs,
        "verify_run": verify_run,
        "output_directory_name": output_directory_name,
    }
    for name, value in values.items():
        setattr(base, name, value)


def main() -> int:
    _configure_base()
    return base.main()


_configure_base()


if __name__ == "__main__":
    raise SystemExit(main())
