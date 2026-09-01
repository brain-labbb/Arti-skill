#!/usr/bin/env python3
"""Table 4a Genesis adapter for the frozen SketchMobility N=800 cohort."""

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

from exp.scripts import run_urdf_table4a_partnet_mobility as base  # noqa: E402
from exp.scripts import sketchmobility_supplementary_common as common  # noqa: E402


SCHEMA_VERSION = "table4a-sketchmobility/v1"
PROTOCOL_ID = "table4a-sketchmobility-table1-cohort-n800-v1"
DATASET = "SketchMobility"
CLASSIFICATION = "FORMAL"
SOURCE_MANIFEST = common.DEFAULT_TABLE4_RECEIPT / "manifest.json"
EXPECTED_SOURCE_MANIFEST_SHA256 = common.EXPECTED_TABLE4_MANIFEST_SHA256
EXPECTED_ORDERED_IDS_SHA256 = common.EXPECTED_ORDERED_ASSET_IDS_SHA256
TABLE2_RECORDS = common.DEFAULT_TABLE2_RECEIPT / "asset_records.jsonl"
TABLE3_RECORDS = common.DEFAULT_TABLE3_RECEIPT / "asset_records.jsonl"
TABLE4_ASSET_RECORDS = common.DEFAULT_TABLE4_RECEIPT / "asset_records.jsonl"
TABLE4_STATE_RECORDS = common.DEFAULT_TABLE4_RECEIPT / "state_records.jsonl"
EXPECTED_TABLE2_RECORDS_SHA256 = "03b6d5e0d335052f123664a7a85dcdbc33ffbad8143ffb4bb62560e9b44ea2d1"
EXPECTED_TABLE3_RECORDS_SHA256 = "13124125cbdef565efc95c7526e052576aead73fa6499d7b0b81bcc0490a24f7"
EXPECTED_TABLE4_ASSET_RECORDS_SHA256 = "6b51d10a094bea63d20829cf16a4a4034b5cbe31ebdc3852617fc7690ebed58a"
EXPECTED_TABLE4_STATE_RECORDS_SHA256 = "91a1b9b676436f5ff753c0fec6f1dfcc9f4e1c32b60cad1172c14f1ce5c12a40"
N_EVAL = common.FORMAL_N_EVAL
J_EVAL = common.FORMAL_J_EVAL
EXPECTED_CATEGORY_COUNT = 67
WORKERS = 4
CHILD_TIMEOUT_SECONDS = 3600.0
REQUIRE_SMOKE_RECEIPT = True
FORMAL_OUTPUT_NAME = "table4a_urdf_sketch_mobility_table1cohort_n800_{timestamp}"
SMOKE_OUTPUT_NAME = "table4a_urdf_sketch_mobility_smoke_n{n}_{timestamp}"

_BASE_BUILD_JOBS = base.build_jobs
_BASE_SPAWN_CHILDREN = base.spawn_children
_BASE_RUN_CHILD = base.run_child


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def load_source_manifest() -> dict[str, Any]:
    cohort = common.load_frozen_cohort(formal=True)
    manifest = dict(cohort["manifest"])
    manifest["dataset_root"] = str(common.DEFAULT_DATASET_ROOT.resolve(strict=True))
    return manifest


def load_table3_joint_pass() -> tuple[dict[str, dict[str, bool]], int]:
    if common.sha256_file(TABLE3_RECORDS) != EXPECTED_TABLE3_RECORDS_SHA256:
        raise ValueError("Table 3 records byte drift")
    records = _load_jsonl(TABLE3_RECORDS)
    cohort = common.load_frozen_cohort(formal=True)["rows"]
    if len(records) != N_EVAL:
        raise ValueError("Table 3 record count drift")
    result: dict[str, dict[str, bool]] = {}
    joint_total = 0
    for expected, record in zip(cohort, records, strict=True):
        asset_id = str(expected["asset_id"])
        if record.get("asset_id") != asset_id:
            raise ValueError("Table 3 cohort order drift")
        expected_joints = sorted(
            expected.get("joint_specs", []), key=lambda row: int(row["xml_index"])
        )
        observed_joints = record.get("joints", [])
        if [
            (joint.get("joint_name"), joint.get("joint_type"))
            for joint in observed_joints
        ] != [(joint["name"], joint["type"]) for joint in expected_joints]:
            raise ValueError(f"Table 3 joint identity drift: {asset_id}")
        passes = {
            str(joint["joint_name"]): bool(joint["joint_level_pass"])
            for joint in record.get("joints", [])
        }
        result[asset_id] = passes
        joint_total += len(passes)
    if joint_total != J_EVAL:
        raise ValueError(f"Table 3 joint denominator drift: {joint_total}")
    return result, joint_total


def load_table4_strict_pass() -> dict[str, bool]:
    if (
        common.sha256_file(TABLE4_ASSET_RECORDS)
        != EXPECTED_TABLE4_ASSET_RECORDS_SHA256
    ):
        raise ValueError("Table 4 asset records byte drift")
    records = _load_jsonl(TABLE4_ASSET_RECORDS)
    cohort = common.load_frozen_cohort(formal=True)["rows"]
    if len(records) != N_EVAL:
        raise ValueError("Table 4 asset record count drift")
    for expected, record in zip(cohort, records, strict=True):
        if (
            record.get("asset_id") != expected["asset_id"]
            or record.get("package_content_manifest_sha256")
            != expected["package_content_manifest_sha256"]
            or record.get("input_identity_sha256")
            != expected["input_identity_sha256"]
            or int(record.get("movable_dof_count", -1))
            != int(expected["movable_dof_count"])
        ):
            raise ValueError(f"Table 4 asset identity drift: {expected['asset_id']}")
    return {
        str(record["asset_id"]): bool(record["strict_collision_pass"])
        for record in records
    }


def load_table4_state_hashes() -> dict[tuple[str, str, int], str]:
    if (
        common.sha256_file(TABLE4_STATE_RECORDS)
        != EXPECTED_TABLE4_STATE_RECORDS_SHA256
    ):
        raise ValueError("Table 4 state records byte drift")
    index: dict[tuple[str, str, int], str] = {}
    with TABLE4_STATE_RECORDS.open(encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("phase") == "single_joint_sweep":
                key = (
                    str(record["asset_id"]),
                    str(record["joint_name"]),
                    int(record["sample_index"]),
                )
                if key in index:
                    raise ValueError(f"duplicate Table 4 state identity: {key}")
                index[key] = str(record["joint_values_sha256"])
    gate = _collision_gate()
    expected_keys = {
        (str(row["asset_id"]), str(joint["name"]), sample_index)
        for row in common.load_frozen_cohort(formal=True)["rows"]
        if gate[str(row["asset_id"])]
        for joint in row.get("joint_specs", [])
        for sample_index in range(base.SINGLE_SAMPLES)
    }
    if set(index) != expected_keys:
        raise ValueError("Table 4 single-joint state identity closure drift")
    return index


def _collision_gate() -> dict[str, bool]:
    if common.sha256_file(TABLE2_RECORDS) != EXPECTED_TABLE2_RECORDS_SHA256:
        raise ValueError("Table 2 records byte drift")
    records = _load_jsonl(TABLE2_RECORDS)
    cohort = common.load_frozen_cohort(formal=True)["rows"]
    if len(records) != N_EVAL:
        raise ValueError("Table 2 record count drift")
    gate: dict[str, bool] = {}
    for expected, record in zip(cohort, records, strict=True):
        asset_id = str(expected["asset_id"])
        if record.get("asset_id") != asset_id:
            raise ValueError("Table 2 cohort order drift")
        if (
            record.get("package_content_manifest_sha256")
            != expected["package_content_manifest_sha256"]
            or record.get("primary_urdf_sha256") != expected["urdf_sha256"]
        ):
            raise ValueError(f"Table 2 asset identity drift: {asset_id}")
        gate[asset_id] = bool(
            record.get("metrics", {}).get("collision_coverage", {}).get("pass")
        )
    if sum(gate.values()) != 311:
        raise ValueError(f"Table 2 collision gate drift: {sum(gate.values())}")
    return gate


def build_jobs(
    manifest: Mapping[str, Any],
    table3_pass: Mapping[str, Mapping[str, bool]],
    state_hashes: Mapping[tuple[str, str, int], str],
) -> list[dict[str, Any]]:
    gate = _collision_gate()
    jobs = _BASE_BUILD_JOBS(manifest, table3_pass, state_hashes)
    authority = {
        str(row["asset_id"]): row
        for row in common.load_frozen_cohort(formal=True)["rows"]
    }
    for job in jobs:
        row = authority[str(job["dataset_id"])]
        job["genesis_eligible"] = gate[str(job["dataset_id"])]
        job["expected_package_content_manifest_sha256"] = row[
            "package_content_manifest_sha256"
        ]
    return jobs


def gate_failed_record(job: Mapping[str, Any]) -> dict[str, Any]:
    record = base._failed_asset_record(
        job, "table2_collision_coverage_incomplete"
    )
    record["expected_package_content_manifest_sha256"] = job[
        "expected_package_content_manifest_sha256"
    ]
    record["package_content_manifest_sha256"] = job[
        "expected_package_content_manifest_sha256"
    ]
    return record


def _audit_job(job: Mapping[str, Any]) -> None:
    common.audit_package(
        {
            "asset_id": job["dataset_id"],
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
        Path(base.lam4a.__file__).resolve(),
        Path(base.lam4a.static.__file__).resolve(),
        Path(base.lam4a.geometry.__file__).resolve(),
        Path(base.lam4a.verifier.__file__).resolve(),
        Path(common.__file__).resolve(),
        REPO / "exp/scripts/verify_table4a_urdf_sketch_mobility.py",
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


def spawn_children(
    jobs: Sequence[Mapping[str, Any]],
    outdir: Path,
    *,
    workers: int,
    timeout_seconds: float,
) -> list[dict[str, Any]]:
    eligible = [job for job in jobs if bool(job.get("genesis_eligible"))]
    measured = _BASE_SPAWN_CHILDREN(
        eligible, outdir, workers=workers, timeout_seconds=timeout_seconds
    )
    by_index = {int(record["selection_index"]): record for record in measured}
    records = [
        by_index[int(job["selection_index"])]
        if bool(job.get("genesis_eligible"))
        else gate_failed_record(job)
        for job in jobs
    ]
    return records


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
    assets_path = root / "asset_records.jsonl"
    joints_path = root / "joint_records.jsonl"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    run_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    frozen_config = json.loads(frozen_config_path.read_text(encoding="utf-8"))
    asset_records = _load_jsonl(assets_path)
    joint_rows = _load_jsonl(joints_path)
    by_index: dict[int, list[dict[str, Any]]] = {}
    for joint in joint_rows:
        by_index.setdefault(int(joint["selection_index"]), []).append(joint)
    records = [
        {
            **record,
            "joint_records": sorted(
                by_index.get(int(record["selection_index"]), []),
                key=lambda row: int(row.get("dof_position", 10**9)),
            ),
        }
        for record in asset_records
    ]
    authority = common.load_frozen_cohort(formal=True)["rows"][:5]
    expected_ids = [str(row["asset_id"]) for row in authority]
    recomputed = base.aggregate(records, load_table4_strict_pass())
    observed = {
        "mode": summary.get("mode"),
        "classification": summary.get("classification"),
        "n_eval": summary.get("cohort", {}).get("n_eval"),
        "record_count": len(records),
        "ordered_ids": [record.get("dataset_id") for record in records],
        "workers": frozen_config.get("execution", {}).get("workers"),
        "timeout": frozen_config.get("execution", {}).get(
            "child_timeout_seconds"
        ),
        "runner_sha256": frozen_config.get("runner_identity", {}).get(
            "runner_script_sha256"
        ),
        "manifest_protocol": run_manifest.get("protocol_id"),
        "manifest_mode": run_manifest.get("mode"),
        "metrics_match": common.canonical_sha256(recomputed)
        == common.canonical_sha256(summary.get("metrics", {})),
    }
    expected = {
        "mode": "smoke",
        "classification": "SMOKE",
        "n_eval": 5,
        "record_count": 5,
        "ordered_ids": expected_ids,
        "workers": WORKERS,
        "timeout": CHILD_TIMEOUT_SECONDS,
        "runner_sha256": common.sha256_file(SCRIPT),
        "manifest_protocol": PROTOCOL_ID,
        "manifest_mode": "smoke",
        "metrics_match": True,
    }
    if observed != expected:
        raise ValueError(f"smoke receipt N=5 configuration mismatch: {observed}")
    if not standalone_path.is_file():
        raise ValueError("smoke receipt standalone verification is missing")
    from exp.scripts import verify_table4a_urdf_sketch_mobility as verifier

    replay = verifier.verify_output(root, write=False)
    stored = json.loads(standalone_path.read_text(encoding="utf-8"))
    if replay.get("status") != "PASS" or stored.get("status") != "PASS":
        raise ValueError("smoke receipt standalone verifier replay failed")
    return {
        "path": str(root),
        "summary_sha256": common.sha256_file(summary_path),
        "manifest_sha256": common.sha256_file(manifest_path),
        "frozen_config_sha256": common.sha256_file(frozen_config_path),
        "asset_records_sha256": common.sha256_file(assets_path),
        "joint_records_sha256": common.sha256_file(joints_path),
        "standalone_verification_sha256": common.sha256_file(standalone_path),
        "ordered_asset_ids_sha256": common.canonical_sha256(expected_ids),
    }


def finalize_receipt(outdir: Path) -> None:
    from exp.scripts import verify_table4a_urdf_sketch_mobility as verifier

    result = verifier.verify_output(outdir, write=True)
    if result.get("status") != "PASS":
        raise RuntimeError("Table 4a standalone receipt verification failed")


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
        "TABLE3_RECORDS": TABLE3_RECORDS,
        "TABLE4_ASSET_RECORDS": TABLE4_ASSET_RECORDS,
        "TABLE4_STATE_RECORDS": TABLE4_STATE_RECORDS,
        "N_EVAL": N_EVAL,
        "J_EVAL": J_EVAL,
        "EXPECTED_CATEGORY_COUNT": EXPECTED_CATEGORY_COUNT,
        "WORKERS": WORKERS,
        "CHILD_TIMEOUT_SECONDS": CHILD_TIMEOUT_SECONDS,
        "REQUIRE_SMOKE_RECEIPT": REQUIRE_SMOKE_RECEIPT,
        "FORMAL_OUTPUT_NAME": FORMAL_OUTPUT_NAME,
        "SMOKE_OUTPUT_NAME": SMOKE_OUTPUT_NAME,
        "load_source_manifest": load_source_manifest,
        "load_table3_joint_pass": load_table3_joint_pass,
        "load_table4_strict_pass": load_table4_strict_pass,
        "load_table4_state_hashes": load_table4_state_hashes,
        "build_jobs": build_jobs,
        "spawn_children": spawn_children,
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
