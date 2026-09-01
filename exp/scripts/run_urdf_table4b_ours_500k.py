#!/usr/bin/env python3
"""Fail-closed Table 4b runner for the frozen Ours-500K N=500 cohort."""

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


SCHEMA_VERSION = "table4b-ours-500k/v1"
PROTOCOL_ID = "table4b_ours_500k_table1cohort_n500_v1"
DATASET = "Ours-500K"
CLASSIFICATION = "FORMAL"
SOURCE_MANIFEST = REPO / "exp/runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z/frozen_manifest.json"
EXPECTED_SOURCE_MANIFEST_SHA256 = "1b29d868112dcda326a08f8e3439d6b96c65833b99cc33af3bfcdb58fb4c2e24"
EXPECTED_ORDERED_IDS_SHA256 = "dcd19530ff3a3546fa149db58f331a042d5c3326f1b4fa1e5580914952e79289"
DATASET_ROOT = REPO / "exp/Brain/extracted/arti_cabinet_drawer_geometry_500_20260813"
N_EVAL = 500
FORMAL_OUTPUT_NAME = "table4b_urdf_ours_500k_table1cohort_n500_{timestamp}"
SMOKE_OUTPUT_NAME = "table4b_urdf_ours_500k_smoke_n{n}_{timestamp}"
SUMMARY_TITLE = "Table 4b - Ours-500K (frozen full N=500 cohort; exact collision-representation geometry)"
SELECTION_POLICY = "all 500 frozen Table 4 items in existing order; no resampling, replacement, or result filtering"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def ordered_ids_sha256(ids: list[str]) -> str:
    payload = json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode()
    return hashlib.sha256(payload).hexdigest()


def load_source_manifest() -> dict[str, Any]:
    if sha256_file(SOURCE_MANIFEST) != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise SystemExit("source manifest sha256 mismatch")
    manifest = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))
    items = manifest.get("items")
    if manifest.get("protocol_id") != "urdf_sim_ready_table4_ours_500k_table2_n500_v1":
        raise SystemExit("source protocol mismatch")
    if not isinstance(items, list) or len(items) != N_EVAL:
        raise SystemExit(f"source manifest must contain {N_EVAL} items")
    ids = [str(item["dataset_id"]) for item in items]
    if ordered_ids_sha256(ids) != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit("ordered dataset IDs mismatch")
    return manifest


def build_jobs(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    root = DATASET_ROOT.resolve(strict=True)
    jobs: list[dict[str, Any]] = []
    for index, item in enumerate(manifest["items"]):
        if int(item.get("order", -1)) != index:
            raise SystemExit(f"source order mismatch at {index}")
        asset_id = str(item["dataset_id"])
        package = (root / str(item["asset_root_relpath"])).resolve(strict=True)
        try:
            package.relative_to(root)
        except ValueError as error:
            raise SystemExit(f"package escapes Brain root: {asset_id}") from error
        urdf = (root / str(item["primary_urdf_relpath"])).resolve(strict=True)
        if urdf.parent != package or sha256_file(urdf) != item.get("urdf_sha256"):
            raise SystemExit(f"URDF binding mismatch: {asset_id}")
        for mesh in item.get("collision_mesh_files", []):
            if mesh.get("exists"):
                path = (package / str(mesh["resolved_relpath"])).resolve(strict=True)
                if sha256_file(path) != mesh.get("sha256") or path.stat().st_size != mesh.get("size_bytes"):
                    raise SystemExit(f"collision mesh binding mismatch: {asset_id}")
        jobs.append({
            "selection_index": index,
            "dataset_id": asset_id,
            "asset_id": asset_id,
            "category": str(item["category"]),
            "package": str(package),
            "urdf_path": str(urdf),
            "expected_urdf_sha256": str(item["urdf_sha256"]),
            "input_identity_sha256": str(item["input_identity_sha256"]),
        })
    return jobs


def verify_run(manifest: Mapping[str, Any], records: Sequence[Mapping[str, Any]], aggregates: Mapping[str, Any]) -> dict[str, Any]:
    checks = [
        {"check": "record_count", "pass": len(records) == N_EVAL, "detail": str(len(records))},
        {"check": "frozen_order_preserved", "pass": all(record.get("dataset_id") == manifest["items"][i]["dataset_id"] for i, record in enumerate(records)), "detail": ""},
        {"check": "aggregate_recomputation_matches", "pass": base.aggregate(records) == dict(aggregates), "detail": ""},
    ]
    return {"all_pass": all(check["pass"] for check in checks), "check_count": len(checks), "checks": checks}


def output_directory_name(mode: str, n: int, timestamp: str) -> str:
    if mode == "formal":
        return FORMAL_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    if mode == "smoke":
        return SMOKE_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    raise ValueError(f"unsupported mode: {mode}")


def _configure_base() -> None:
    for name, value in {
        "SCRIPT": SCRIPT, "SCHEMA_VERSION": SCHEMA_VERSION, "PROTOCOL_ID": PROTOCOL_ID,
        "DATASET": DATASET, "CLASSIFICATION": CLASSIFICATION, "SOURCE_MANIFEST": SOURCE_MANIFEST,
        "EXPECTED_SOURCE_MANIFEST_SHA256": EXPECTED_SOURCE_MANIFEST_SHA256,
        "EXPECTED_ORDERED_IDS_SHA256": EXPECTED_ORDERED_IDS_SHA256, "N_EVAL": N_EVAL,
        "CPU_AFFINITY_ENV": "TABLE4B_OURS_500K_CPU_AFFINITY", "FORMAL_OUTPUT_NAME": FORMAL_OUTPUT_NAME,
        "SMOKE_OUTPUT_NAME": SMOKE_OUTPUT_NAME, "SUMMARY_TITLE": SUMMARY_TITLE,
        "SELECTION_POLICY": SELECTION_POLICY, "load_source_manifest": load_source_manifest,
        "build_jobs": build_jobs, "verify_run": verify_run,
    }.items():
        setattr(base, name, value)


def main() -> int:
    _configure_base()
    return base.main()


_configure_base()

if __name__ == "__main__":
    raise SystemExit(main())
