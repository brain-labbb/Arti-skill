#!/usr/bin/env python3
"""Table 4b exact-geometry adapter for frozen SketchMobility N=800."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shutil
import sys
from typing import Any, Mapping, Sequence


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_urdf_table4b_artiverse as base  # noqa: E402
from exp.scripts import sketchmobility_supplementary_common as common  # noqa: E402


SCHEMA_VERSION = "table4b-sketchmobility/v1"
PROTOCOL_ID = "table4b-sketchmobility-table1-cohort-n800-v1"
DATASET = "SketchMobility"
CLASSIFICATION = "FORMAL"
SOURCE_MANIFEST = common.DEFAULT_TABLE4_RECEIPT / "manifest.json"
EXPECTED_SOURCE_MANIFEST_SHA256 = common.EXPECTED_TABLE4_MANIFEST_SHA256
EXPECTED_ORDERED_IDS_SHA256 = common.EXPECTED_ORDERED_ASSET_IDS_SHA256
N_EVAL = common.FORMAL_N_EVAL
REQUIRE_SMOKE_RECEIPT = True
FORMAL_OUTPUT_NAME = "table4b_urdf_sketch_mobility_table1cohort_n800_{timestamp}"
SMOKE_OUTPUT_NAME = "table4b_urdf_sketch_mobility_smoke_n{n}_{timestamp}"
SUMMARY_TITLE = "Table 4b - SketchMobility (frozen Table 1 cohort, N=800)"
SELECTION_POLICY = (
    "exact frozen SketchMobility Table 4 items in original order; no resampling, "
    "replacement, or result-based filtering"
)
_BASE_RUN_CHILD = base.run_child


def load_source_manifest() -> dict[str, Any]:
    cohort = common.load_frozen_cohort(formal=True)
    manifest = dict(cohort["manifest"])
    manifest["dataset_root"] = str(common.DEFAULT_DATASET_ROOT.resolve(strict=True))
    return manifest


def build_jobs(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    dataset_root = Path(str(manifest["dataset_root"])).resolve(strict=True)
    jobs: list[dict[str, Any]] = []
    for index, item in enumerate(manifest["items"]):
        asset_id = str(item["asset_id"])
        package = (dataset_root / asset_id).resolve(strict=True)
        package.relative_to(dataset_root)
        urdf_path = (package / "mobility.urdf").resolve(strict=True)
        urdf_path.relative_to(package)
        jobs.append(
            {
                "selection_index": index,
                "dataset_id": asset_id,
                "asset_id": asset_id,
                "category": str(item["category"]),
                "package": str(package),
                "urdf_path": str(urdf_path),
                "expected_urdf_sha256": str(item["urdf_sha256"]),
                "expected_package_content_manifest_sha256": str(
                    item["package_content_manifest_sha256"]
                ),
                "input_identity_sha256": str(item["input_identity_sha256"]),
            }
        )
    return jobs


def _audit_job(job: Mapping[str, Any]) -> None:
    common.audit_package(
        {
            "asset_id": job["asset_id"],
            "urdf_sha256": job["expected_urdf_sha256"],
            "package_content_manifest_sha256": job[
                "expected_package_content_manifest_sha256"
            ],
        },
        common.DEFAULT_DATASET_ROOT,
    )


def validate_jobs(jobs: Sequence[Mapping[str, Any]], workers: int) -> None:
    with ThreadPoolExecutor(max_workers=workers) as executor:
        list(executor.map(_audit_job, jobs))


def run_child(job_path: Path, result_path: Path) -> int:
    job = json.loads(job_path.read_text(encoding="utf-8"))
    _audit_job(job)
    return_code = _BASE_RUN_CHILD(job_path, result_path)
    record = json.loads(result_path.read_text(encoding="utf-8"))
    record["expected_package_content_manifest_sha256"] = job[
        "expected_package_content_manifest_sha256"
    ]
    record["package_content_manifest_sha256"] = job[
        "expected_package_content_manifest_sha256"
    ]
    base.atomic_write_json(result_path, record)
    return return_code


def snapshot_sources(outdir: Path) -> dict[str, str]:
    sources = [
        SCRIPT,
        Path(base.__file__).resolve(),
        Path(base.geometry.__file__).resolve(),
        Path(common.__file__).resolve(),
        REPO / "exp/scripts/verify_table4b_urdf_sketch_mobility.py",
    ]
    root = outdir / "source_snapshots"
    hashes: dict[str, str] = {}
    for source in sources:
        relative = source.relative_to(REPO)
        destination = root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        digest = common.sha256_file(source)
        if destination.exists():
            if common.sha256_file(destination) != digest:
                raise RuntimeError(f"source snapshot drift: {relative}")
        else:
            shutil.copyfile(source, destination)
        hashes[relative.as_posix()] = digest
    base.CHILD_SOURCE_ROOT = root
    base.SCRIPT = root / SCRIPT.relative_to(REPO)
    return hashes


def verify_run(
    manifest: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
    aggregates: Mapping[str, Any],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def check(name: str, passed: bool, detail: str = "") -> None:
        checks.append({"check": name, "pass": bool(passed), "detail": detail})

    items = manifest["items"]
    ids = [str(item["asset_id"]) for item in items]
    ordered = common.canonical_sha256(ids)
    check(
        "source_manifest_sha256",
        common.sha256_file(SOURCE_MANIFEST) == EXPECTED_SOURCE_MANIFEST_SHA256,
    )
    check("ordered_ids_sha256", ordered == EXPECTED_ORDERED_IDS_SHA256, ordered)
    check("record_count", len(records) == N_EVAL, str(len(records)))
    check(
        "frozen_order_preserved",
        all(
            int(record["selection_index"]) == index
            and str(record["asset_id"]) == ids[index]
            and str(record["dataset_id"]) == ids[index]
            for index, record in enumerate(records)
        ),
    )
    check(
        "urdf_identity_matches_frozen_manifest",
        all(
            (record.get("urdf_sha256") is None and record.get("status") != "completed")
            or record.get("urdf_sha256")
            == str(items[int(record["selection_index"])]["urdf_sha256"])
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
        "all_pass": all(row["pass"] for row in checks),
        "check_count": len(checks),
        "checks": checks,
    }


def output_directory_name(mode: str, n: int, timestamp: str) -> str:
    if mode == "formal":
        return FORMAL_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    if mode == "smoke":
        return SMOKE_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    raise ValueError(f"unsupported mode: {mode!r}")


def validate_smoke_receipt(path: Path | None) -> dict[str, Any]:
    if path is None:
        raise ValueError("formal mode requires an exact N=5 smoke receipt")
    root = path.resolve(strict=True)
    summary_path = root / "summary.json"
    manifest_path = root / "manifest.json"
    frozen_config_path = root / "frozen_config.json"
    standalone_path = root / "standalone_verification.json"
    records_path = root / "asset_records.jsonl"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    run_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    frozen_config = json.loads(frozen_config_path.read_text(encoding="utf-8"))
    with records_path.open(encoding="utf-8") as stream:
        records = [json.loads(line) for line in stream if line.strip()]
    authority = common.load_frozen_cohort(formal=True)["rows"][:5]
    expected_ids = [str(row["asset_id"]) for row in authority]
    observed = {
        "mode": summary.get("mode"),
        "classification": summary.get("classification"),
        "n_eval": summary.get("cohort", {}).get("n_eval"),
        "record_count": len(records),
        "ordered_ids": [record.get("asset_id") for record in records],
        "workers": frozen_config.get("execution", {}).get("workers"),
        "timeout": frozen_config.get("execution", {}).get(
            "child_timeout_seconds"
        ),
        "runner_sha256": frozen_config.get("runner_identity", {}).get(
            "runner_script_sha256"
        ),
        "manifest_protocol": run_manifest.get("protocol_id"),
        "manifest_mode": run_manifest.get("mode"),
        "metrics_match": json.dumps(base.aggregate(records), sort_keys=True)
        == json.dumps(summary.get("metrics", {}), sort_keys=True),
    }
    expected = {
        "mode": "smoke",
        "classification": "SMOKE",
        "n_eval": 5,
        "record_count": 5,
        "ordered_ids": expected_ids,
        "workers": base.WORKERS,
        "timeout": base.CHILD_TIMEOUT_SECONDS,
        "runner_sha256": common.sha256_file(SCRIPT),
        "manifest_protocol": PROTOCOL_ID,
        "manifest_mode": "smoke",
        "metrics_match": True,
    }
    if observed != expected:
        raise ValueError(f"smoke receipt N=5 configuration mismatch: {observed}")
    if not standalone_path.is_file():
        raise ValueError("smoke receipt standalone verification is missing")
    from exp.scripts import verify_table4b_urdf_sketch_mobility as verifier

    replay = verifier.verify_output(root, write=False)
    stored = json.loads(standalone_path.read_text(encoding="utf-8"))
    if replay.get("status") != "PASS" or stored.get("status") != "PASS":
        raise ValueError("smoke receipt standalone verifier replay failed")
    return {
        "path": str(root),
        "summary_sha256": common.sha256_file(summary_path),
        "manifest_sha256": common.sha256_file(manifest_path),
        "frozen_config_sha256": common.sha256_file(frozen_config_path),
        "asset_records_sha256": common.sha256_file(records_path),
        "standalone_verification_sha256": common.sha256_file(standalone_path),
        "ordered_asset_ids_sha256": common.canonical_sha256(expected_ids),
    }


def finalize_receipt(outdir: Path) -> None:
    from exp.scripts import verify_table4b_urdf_sketch_mobility as verifier

    result = verifier.verify_output(outdir, write=True)
    if result.get("status") != "PASS":
        raise RuntimeError("Table 4b standalone receipt verification failed")


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
        "REQUIRE_SMOKE_RECEIPT": REQUIRE_SMOKE_RECEIPT,
        "FORMAL_OUTPUT_NAME": FORMAL_OUTPUT_NAME,
        "SMOKE_OUTPUT_NAME": SMOKE_OUTPUT_NAME,
        "SUMMARY_TITLE": SUMMARY_TITLE,
        "SELECTION_POLICY": SELECTION_POLICY,
        "load_source_manifest": load_source_manifest,
        "build_jobs": build_jobs,
        "verify_run": verify_run,
        "output_directory_name": output_directory_name,
        "validate_smoke_receipt": validate_smoke_receipt,
        "validate_jobs": validate_jobs,
        "run_child": run_child,
        "snapshot_sources": snapshot_sources,
        "finalize_receipt": finalize_receipt,
    }
    for name, value in overrides.items():
        setattr(base, name, value)


def main() -> int:
    _configure_base()
    return base.main()


_configure_base()

if __name__ == "__main__":
    raise SystemExit(main())
