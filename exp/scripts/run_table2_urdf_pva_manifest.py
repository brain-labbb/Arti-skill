#!/usr/bin/env python3
"""Run the frozen Table 2 URDF audit on the PV-A custom cohort.

The PV-A cohort is category-stratified (five assets per category), with the
fence and Ferris-wheel rows replaced by the highest declared movable-joint
counts.  This adapter owns only cohort/provenance handling; the nine Table 2
metrics and the fresh-interpreter child protocol are shared with the existing
Table 2 runner.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import time
from typing import Any
import uuid


REPO = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = REPO / "exp/PV-A-per-class-n5-max-joints/manifest.json"
DEFAULT_TABLE1_RECEIPT = REPO / "exp/runtime/table1_pva_per_class_n5_max_joints"
DEFAULT_OUTPUT = REPO / "exp/runtime/table2_pva_per_class_n5_max_joints"
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
TABLE2_RUNNER_PATH = Path(__file__).resolve()
TABLE2_BASE_RUNNER_PATH = REPO / "exp/scripts/run_table2_urdf_ours_500k.py"
TABLE2_EVALUATOR_PATH = REPO / "exp/scripts/run_table2_urdf_articraft.py"
EXPECTED_N = 2_655
N_RELEASE = 302_440
RELEASE_CATEGORIES = 531
DATASET_LABEL = "PV-A-per-class-n5-max-joints"
COHORT_TYPE = "CATEGORY_STRATIFIED_N5_WITH_FENCE_FERRIS_MAX_JOINT_OVERRIDES"
ASSET_TIMEOUT_SECONDS = 120.0
TABLE1_ARTIFACT_NAMES = {
    "manifest.json",
    "asset_records.jsonl",
    "summary.json",
    "report.md",
    "timing.json",
}


def _load_module(path: Path, name: str):
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# Reuse the established child-process protocol.  Redirect its SCRIPT_PATH so
# children attest this adapter's runner hash and re-enter through this file.
BASE = _load_module(TABLE2_BASE_RUNNER_PATH, "table2_pva_manifest_child_shared")
BASE.SCRIPT_PATH = TABLE2_RUNNER_PATH
TABLE2 = BASE.TABLE2


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    return TABLE2.sha256_file(path)


def canonical_sha256(value: Any) -> str:
    return TABLE2.canonical_sha256(value)


def _validate_sha256(value: Any, field: str) -> str:
    value = str(value)
    if len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
        raise ValueError(f"{field} is not a lowercase SHA-256")
    return value


def _validate_package_binding(row: dict[str, Any], *, live: bool) -> None:
    package_raw = row.get("package")
    if not isinstance(package_raw, str) or not Path(package_raw).is_absolute():
        raise ValueError(f"package path is not absolute: {package_raw!r}")
    package = Path(package_raw)
    if not package.is_dir():
        raise ValueError(f"package directory is missing: {package}")
    if row.get("primary_urdf_relative_path") != "model.urdf":
        raise ValueError("only model.urdf is supported as the primary URDF")
    binding = row.get("package_binding")
    if not isinstance(binding, dict):
        raise ValueError("package_binding is missing")
    expected_urdf = _validate_sha256(row.get("urdf_sha256"), "urdf_sha256")
    urdf = package / "model.urdf"
    if live:
        if sha256_file(urdf) != expected_urdf:
            raise ValueError(f"primary URDF hash mismatch: {row.get('dataset_id')}")
        if TABLE2.package_binding(package) != binding:
            raise ValueError(f"package binding mismatch: {row.get('dataset_id')}")


def load_cohort(
    path: Path, *, expected_n: int, validate_packages: bool = False
) -> dict[str, Any]:
    """Load and validate the immutable PV-A selection manifest."""

    path = path.resolve(strict=True)
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError("cohort manifest must be an object")
    content_hash = manifest.get("manifest_content_sha256")
    if content_hash != TABLE2.manifest_self_hash(manifest):
        raise ValueError("cohort manifest self-hash mismatch")
    assets = manifest.get("assets")
    if not isinstance(assets, list) or len(assets) != expected_n:
        raise ValueError(
            f"cohort asset count mismatch: {len(assets) if isinstance(assets, list) else None}"
        )
    if manifest.get("n_eval") != expected_n:
        raise ValueError("cohort n_eval mismatch")
    if expected_n == EXPECTED_N:
        if manifest.get("dataset") != "PV-A-per-class-n5":
            raise ValueError("unexpected frozen PV-A cohort dataset")
        if manifest.get("protocol_id") != "pva-per-class-n5-fence-ferris-max-movable-joints-v1":
            raise ValueError("unexpected frozen PV-A cohort protocol")
        if manifest.get("class_count") != RELEASE_CATEGORIES:
            raise ValueError("frozen PV-A class_count must be 531")
        if manifest.get("per_class") != 5:
            raise ValueError("frozen PV-A per_class must be 5")

    category_counts: Counter[str] = Counter()
    identifiers: list[str] = []
    normalized: list[dict[str, Any]] = []
    for index, raw in enumerate(assets):
        if not isinstance(raw, dict):
            raise ValueError(f"asset row {index} is not an object")
        row = dict(raw)
        if row.get("selection_index") != index:
            raise ValueError(f"selection index mismatch at {index}")
        dataset_id = row.get("dataset_id")
        if not isinstance(dataset_id, str) or not dataset_id:
            raise ValueError(f"missing dataset_id at {index}")
        category = row.get("category")
        if not isinstance(category, str) or not category:
            raise ValueError(f"missing category at {index}")
        identifiers.append(dataset_id)
        category_counts[category] += 1
        _validate_package_binding(row, live=validate_packages)
        normalized.append(row)
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("dataset_id values are not unique")
    if manifest.get("class_count") != len(category_counts):
        raise ValueError("cohort class_count disagrees with assets")
    per_class = manifest.get("per_class")
    if not isinstance(per_class, int) or per_class < 1:
        raise ValueError("cohort per_class is invalid")
    if set(category_counts.values()) != {per_class}:
        raise ValueError("cohort per_class disagrees with assets")
    return {
        "path": path,
        "file_sha256": sha256_file(path),
        "content_sha256": content_hash,
        "manifest": manifest,
        "assets": normalized,
        "category_counts": dict(sorted(category_counts.items())),
    }


def _validate_receipt_artifacts(receipt: Path) -> dict[str, Any]:
    artifact_path = receipt / "artifact_manifest.json"
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    files = artifact.get("files")
    if not isinstance(files, dict) or not files:
        raise ValueError("Table 1 artifact manifest is empty")
    if set(files) != TABLE1_ARTIFACT_NAMES:
        raise ValueError("Table 1 artifact manifest file set is unexpected")
    for name, expected in files.items():
        if Path(str(name)).name != str(name):
            raise ValueError(f"unsafe Table 1 artifact name: {name}")
        path = receipt / str(name)
        if not path.is_file():
            raise ValueError(f"Table 1 artifact is missing: {name}")
        if path.stat().st_size != expected.get("bytes"):
            raise ValueError(f"Table 1 artifact size mismatch: {name}")
        if sha256_file(path) != expected.get("sha256"):
            raise ValueError(f"Table 1 artifact hash mismatch: {name}")
    return artifact


def load_table1_receipt(receipt_dir: Path, cohort: dict[str, Any]) -> dict[str, Any]:
    """Cross-check Table 1's frozen receipt against the source cohort."""

    receipt = receipt_dir.resolve(strict=True)
    manifest_path = receipt / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != "table1_pva_manifest_v1":
        raise ValueError("unexpected Table 1 receipt schema")
    manifest_sha256 = sha256_file(manifest_path)
    if manifest.get("source_manifest_sha256") != cohort["file_sha256"]:
        raise ValueError("Table 1 source manifest file hash does not match cohort")
    if manifest.get("source_manifest_content_sha256") != cohort["content_sha256"]:
        raise ValueError("Table 1 source manifest content hash does not match cohort")
    protocol_path = Path(str(manifest.get("protocol", ""))).resolve()
    if protocol_path != DEFAULT_PROTOCOL.resolve():
        raise ValueError("Table 1 protocol path does not match the requested protocol")
    if manifest.get("protocol_sha256") != sha256_file(DEFAULT_PROTOCOL):
        raise ValueError("Table 1 protocol hash is stale")
    if manifest.get("N_release") != N_RELEASE or manifest.get("N_eval") != EXPECTED_N:
        raise ValueError("Table 1 cohort denominator mismatch")
    table1_rows = manifest.get("assets")
    if not isinstance(table1_rows, list) or len(table1_rows) != EXPECTED_N:
        raise ValueError("Table 1 manifest asset count mismatch")
    if [row.get("selection_index") for row in table1_rows] != list(range(EXPECTED_N)):
        raise ValueError("Table 1 selection order is not contiguous")

    records_path = receipt / "asset_records.jsonl"
    records: dict[str, dict[str, Any]] = {}
    with records_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            asset_id = str(record.get("asset_id", ""))
            if not asset_id or asset_id in records:
                raise ValueError("Table 1 asset records are not unique")
            if record.get("status") != "EVALUATED":
                raise ValueError(f"Table 1 record is not fully evaluated: {asset_id}")
            records[asset_id] = record
    if len(records) != EXPECTED_N:
        raise ValueError("Table 1 asset record count mismatch")

    for index, (source, receipt_row) in enumerate(zip(cohort["assets"], table1_rows)):
        dataset_id = str(source["dataset_id"])
        if receipt_row.get("dataset_id") != dataset_id:
            raise ValueError(f"Table 1 dataset identity mismatch at {index}")
        if receipt_row.get("asset_id") != source.get("asset_id"):
            raise ValueError(f"Table 1 source asset key mismatch at {index}")
        if receipt_row.get("selection_rank") != index + 1:
            raise ValueError(f"Table 1 selection rank mismatch at {index}")
        if receipt_row.get("raw_category") != source.get("category"):
            raise ValueError(f"Table 1 category mismatch at {index}")
        for field in ("package", "package_binding"):
            if receipt_row.get(field) != source.get(field):
                raise ValueError(f"Table 1 {field} mismatch at {index}")
        if receipt_row.get("primary_urdf_sha256") != source.get("urdf_sha256"):
            raise ValueError(f"Table 1 URDF hash mismatch at {index}")
        record = records.get(dataset_id)
        if record is None or record.get("selection_index") != index:
            raise ValueError(f"Table 1 record order mismatch at {index}")
        if record.get("primary_urdf_sha256") != source.get("urdf_sha256"):
            raise ValueError(f"Table 1 record URDF hash mismatch at {index}")

    summary = json.loads((receipt / "summary.json").read_text(encoding="utf-8"))
    if summary.get("status_counts") != {"EVALUATED": EXPECTED_N}:
        raise ValueError("Table 1 summary contains non-evaluated assets")
    if summary.get("cohort", {}).get("N_eval") != EXPECTED_N:
        raise ValueError("Table 1 summary N_eval mismatch")
    if summary.get("cohort", {}).get("N_parse") != EXPECTED_N:
        raise ValueError("Table 1 summary parse coverage mismatch")
    artifact = _validate_receipt_artifacts(receipt)
    return {
        "path": receipt,
        "manifest": manifest,
        "manifest_sha256": manifest_sha256,
        "manifest_content_sha256": TABLE2.manifest_self_hash(manifest),
        "asset_records_sha256": sha256_file(records_path),
        "summary_sha256": sha256_file(receipt / "summary.json"),
        "artifact_manifest_sha256": sha256_file(receipt / "artifact_manifest.json"),
        "artifact_manifest": artifact,
    }


def build_jobs(
    rows: list[dict[str, Any]],
    *,
    manifest_content_sha256: str,
    run_standard_parser: bool,
) -> list[dict[str, Any]]:
    """Build jobs accepted by the shared fresh-interpreter runner."""

    evaluator = BASE.evaluator_binding()
    environment = BASE.environment_binding()
    jobs: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        dataset_id = str(row["dataset_id"])
        package = Path(str(row["package"])).resolve(strict=True)
        urdf = package / "model.urdf"
        urdf_sha256 = sha256_file(urdf)
        if urdf_sha256 != str(row["urdf_sha256"]):
            raise ValueError(f"model.urdf hash mismatch for {dataset_id}")
        binding = row.get("package_binding")
        if not isinstance(binding, dict):
            raise ValueError(f"package binding missing for {dataset_id}")
        jobs.append(
            {
                # dataset_id is globally unique; bare seed names are not.
                "asset_id": dataset_id,
                "raw_category": str(row["category"]),
                "seed_name": str(row.get("asset_id", "")),
                "asset_root": str(package),
                "primary_urdf": str(urdf),
                "table1_selection_rank": index + 1,
                "package": str(package),
                "package_binding": binding,
                "run_standard_parser": bool(run_standard_parser),
                "primary_urdf_relative_path": "model.urdf",
                "model_urdf_sha256": urdf_sha256,
                "primary_urdf_sha256": urdf_sha256,
                "manifest_content_sha256": manifest_content_sha256,
                "evaluator_binding": evaluator,
                "environment_binding": environment,
            }
        )
    return jobs


def _artifact_manifest(root: Path, names: tuple[str, ...]) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "created_at_utc": utc_now(),
        "files": {
            name: {
                "bytes": (root / name).stat().st_size,
                "sha256": sha256_file(root / name),
            }
            for name in names
        },
    }


def _verify_artifact_manifest(
    root: Path, expected_names: tuple[str, ...] | None = None
) -> None:
    artifact = json.loads((root / "artifact_manifest.json").read_text(encoding="utf-8"))
    files = artifact.get("files")
    if not isinstance(files, dict) or not files:
        raise ValueError("artifact manifest has no files")
    if expected_names is not None and set(files) != set(expected_names):
        raise ValueError("artifact manifest file set is unexpected")
    for name, expected in files.items():
        if Path(str(name)).name != str(name):
            raise ValueError(f"unsafe artifact name: {name}")
        path = root / str(name)
        if not path.is_file():
            raise ValueError(f"staged artifact missing: {name}")
        if path.stat().st_size != expected.get("bytes"):
            raise ValueError(f"staged artifact size mismatch: {name}")
        if sha256_file(path) != expected.get("sha256"):
            raise ValueError(f"staged artifact hash mismatch: {name}")


def summary_markdown(
    summary: dict[str, Any], manifest: dict[str, Any], timing: dict[str, Any]
) -> str:
    lines = [
        "# PV-A per-class N=5 Table 2 URDF audit",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            f"Custom cohort: `{COHORT_TYPE}`; N_release={N_RELEASE}, "
            f"N_eval={summary['n_eval']}, {RELEASE_CATEGORIES} observed categories."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    for name in TABLE2.METRIC_NAMES:
        result = summary["metrics"][name]
        rate = "N/A" if result["rate"] is None else f"{100 * result['rate']:.2f}%"
        lines.append(f"| {name} | {result['passed']} / {result['denominator']} ({rate}) |")
    macro = summary["category_macro"]
    if macro.get("state") == "evaluated":
        lines.extend(
            [
                "",
                (
                    f"Category macro: unweighted mean over {macro['category_count']} "
                    "raw categories; all frozen assets and failures retained."
                ),
                "",
                "| Metric | Category macro |",
                "|---|---:|",
            ]
        )
        for name in TABLE2.METRIC_NAMES:
            rate = macro["metrics"][name]["rate"]
            lines.append(f"| {name} | {'N/A' if rate is None else f'{100 * rate:.2f}%'} |")
    lines.extend(
        [
            "",
            f"Wall time: {timing['wall_time_seconds']:.6f} seconds.",
            f"Status counts: `{json.dumps(summary['status_counts'], sort_keys=True)}`.",
            "",
        ]
    )
    return "\n".join(lines)


def _make_manifest(
    *,
    cohort: dict[str, Any],
    receipt: dict[str, Any],
    rows: list[dict[str, Any]],
    args: argparse.Namespace,
    started_at: str,
    protocol_binding: dict[str, str],
    environment: dict[str, Any],
) -> dict[str, Any]:
    child_environment = BASE.environment_binding()
    config = {
        **TABLE2.EVALUATOR_CONFIG,
        "dataset": DATASET_LABEL,
        "workers": args.workers,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "selection_algorithm": (
            "existing frozen PV-A category-stratified manifest order; no resampling, "
            "replacement, or outcome-based reselection"
        ),
        "package_staging_policy": (
            "no staging: each manifest package is audited in place, with package content "
            "binding re-verified before and after each fresh child audit"
        ),
    }
    evaluation = {
        "metrics": list(TABLE2.METRIC_NAMES),
        "standard_parser": "urdfpy" if not args.no_standard_parser else "disabled",
        "standard_parser_version": "0.0.22" if not args.no_standard_parser else None,
        "workers": args.workers,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "gpu_required": False,
        "denominator_policy": "all frozen selected assets, including errors",
        "category_macro_policy": (
            "unweighted mean of per-category asset rates; all frozen assets and failures retained"
        ),
        "config": config,
        "config_sha256": canonical_sha256(config),
        "evaluator_path": str(TABLE2_EVALUATOR_PATH.resolve()),
        "evaluator_sha256": sha256_file(TABLE2_EVALUATOR_PATH.resolve()),
        "runner_path": str(TABLE2_RUNNER_PATH),
        "runner_sha256": sha256_file(TABLE2_RUNNER_PATH),
        "shared_child_runner_path": str(TABLE2_BASE_RUNNER_PATH.resolve()),
        "shared_child_runner_sha256": sha256_file(TABLE2_BASE_RUNNER_PATH),
        "environment": environment,
        "environment_sha256": canonical_sha256(environment),
        "child_environment_binding": child_environment,
        "child_environment_binding_sha256": canonical_sha256(child_environment),
        **protocol_binding,
    }
    assets: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        assets.append(
            {
                **row,
                "asset_id": str(row["dataset_id"]),
                "raw_category": str(row["category"]),
                "seed_name": str(row.get("asset_id", "")),
                "selection_rank": index + 1,
                "primary_urdf_sha256": str(row["urdf_sha256"]),
            }
        )
    manifest = {
        "schema_version": "table2_pva_manifest_v1",
        "created_at_utc": started_at,
        "dataset": DATASET_LABEL,
        "classification": "CUSTOM_COHORT_EVIDENCE" if args.limit is None else "SMOKE",
        "cohort_type": COHORT_TYPE,
        "N_release": N_RELEASE,
        "N_eval": len(rows),
        "release_raw_category_count": RELEASE_CATEGORIES,
        "source": {
            "cohort_manifest": str(cohort["path"]),
            "cohort_manifest_sha256": cohort["file_sha256"],
            "cohort_manifest_content_sha256": cohort["content_sha256"],
            "source_cohort_manifest": cohort["manifest"].get("source_cohort_manifest"),
            "source_cohort_manifest_sha256": cohort["manifest"].get(
                "source_cohort_manifest_sha256"
            ),
            "table1_receipt": str(receipt["path"]),
            "table1_manifest_sha256": receipt["manifest_sha256"],
            "table1_manifest_content_sha256": receipt["manifest_content_sha256"],
            "table1_asset_records_sha256": receipt["asset_records_sha256"],
            "table1_summary_sha256": receipt["summary_sha256"],
            "table1_artifact_manifest_sha256": receipt["artifact_manifest_sha256"],
            "release_asset_count": N_RELEASE,
            "release_category_count": RELEASE_CATEGORIES,
        },
        "evaluation": evaluation,
        "assets": assets,
    }
    manifest["manifest_content_sha256"] = TABLE2.manifest_self_hash(manifest)
    return manifest


def run(args: argparse.Namespace) -> dict[str, Any]:
    wall_started = time.perf_counter()
    started_at = utc_now()
    input_started = time.perf_counter()
    cohort = load_cohort(
        args.input_manifest, expected_n=args.expected_n, validate_packages=args.validate_packages
    )
    receipt = load_table1_receipt(args.table1_receipt, cohort)
    rows = cohort["assets"][: args.limit] if args.limit is not None else cohort["assets"]
    input_seconds = time.perf_counter() - input_started

    output = args.output.absolute()
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)

    with BASE.SHARED._output_lock(output):
        staging = output.parent / f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}"
        staging.mkdir(parents=False, exist_ok=False)
        try:
            protocol = args.protocol.resolve(strict=True)
            if protocol != TABLE2.PROTOCOL_PATH.resolve():
                raise ValueError("custom protocol path is unsupported; use the frozen protocol path")
            protocol_binding = TABLE2.freeze_protocol_snapshot(staging)
            TABLE2.validate_protocol_snapshot_binding(staging, protocol_binding)
            environment = TABLE2.environment_metadata(
                not args.no_standard_parser, args.workers
            )
            run_manifest = _make_manifest(
                cohort=cohort,
                receipt=receipt,
                rows=rows,
                args=args,
                started_at=started_at,
                protocol_binding=protocol_binding,
                environment=environment,
            )
            TABLE2.atomic_write_json(staging / "manifest.json", run_manifest)
            TABLE2.atomic_write_json(staging / "environment.json", environment)

            job_started = time.perf_counter()
            jobs = build_jobs(
                rows,
                manifest_content_sha256=run_manifest["manifest_content_sha256"],
                run_standard_parser=not args.no_standard_parser,
            )
            job_seconds = time.perf_counter() - job_started

            completed = 0

            def on_result(record: dict[str, Any]) -> None:
                nonlocal completed
                completed += 1
                if completed == 1 or completed % 100 == 0 or completed == len(jobs):
                    print(
                        json.dumps(
                            {
                                "event": "asset_completed",
                                "completed": completed,
                                "total": len(jobs),
                                "asset_id": record.get("asset_id"),
                                "status": record.get("status"),
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )

            evaluation_started = time.perf_counter()
            records = BASE.execute_audit_jobs(
                jobs,
                worker_scratch=staging / ".worker_scratch",
                timeout_seconds=args.asset_timeout_seconds,
                max_workers=args.workers,
                on_result=on_result,
            )
            evaluation_seconds = time.perf_counter() - evaluation_started
            TABLE2.atomic_write_jsonl(staging / "asset_records.jsonl", records)

            finalize_started = time.perf_counter()
            summary = TABLE2.aggregate_records(
                records, len(rows), category_field="raw_category"
            )
            summary.update(
                {
                    "dataset": DATASET_LABEL,
                    "classification": run_manifest["classification"],
                    "completed_at_utc": utc_now(),
                    "cohort": {
                        "N_release": N_RELEASE,
                        "N_eval": len(rows),
                        "release_raw_categories": RELEASE_CATEGORIES,
                        "eval_raw_categories": len({row["category"] for row in rows}),
                        "cohort_type": COHORT_TYPE,
                        "cohort_manifest_sha256": cohort["file_sha256"],
                        "table1_manifest_sha256": receipt["manifest_sha256"],
                    },
                }
            )
            TABLE2.atomic_write_json(staging / "summary.json", summary)

            timing = {
                "schema_version": "experiment-timing/v1",
                "table": "Table 2",
                "started_at_utc": started_at,
                "completed_at_utc": utc_now(),
                "wall_time_seconds": time.perf_counter() - wall_started,
                "input_validation_seconds": input_seconds,
                "job_construction_seconds": job_seconds,
                "evaluation_seconds": evaluation_seconds,
                "artifact_finalize_seconds": 0.0,
                "workers": args.workers,
                "asset_timeout_seconds": args.asset_timeout_seconds,
                "n_eval": len(rows),
                "measurement_endpoint": "before_atomic_publish",
            }
            timing["artifact_finalize_seconds"] = time.perf_counter() - finalize_started
            TABLE2.atomic_write_json(staging / "timing.json", timing)
            TABLE2.atomic_write_text(
                staging / "summary.md", summary_markdown(summary, run_manifest, timing)
            )
            artifact_names = (
                "manifest.json",
                "asset_records.jsonl",
                "summary.json",
                "summary.md",
                "environment.json",
                TABLE2.PROTOCOL_SNAPSHOT_NAME,
                "timing.json",
            )
            TABLE2.atomic_write_json(
                staging / "artifact_manifest.json",
                _artifact_manifest(staging, artifact_names),
            )
            timing.update(
                {
                    "completed_at_utc": utc_now(),
                    "wall_time_seconds": time.perf_counter() - wall_started,
                    "artifact_finalize_seconds": time.perf_counter() - finalize_started,
                }
            )
            # Finalize all receipts before the atomic directory rename so a
            # post-publish write failure cannot leave a partial output tree.
            TABLE2.atomic_write_json(staging / "timing.json", timing)
            TABLE2.atomic_write_text(
                staging / "summary.md", summary_markdown(summary, run_manifest, timing)
            )
            TABLE2.atomic_write_json(
                staging / "artifact_manifest.json",
                _artifact_manifest(staging, artifact_names),
            )
            _verify_artifact_manifest(staging, artifact_names)
            os.rename(staging, output)
            return {"summary": summary, "timing": timing, "output": output}
        finally:
            if staging.exists():
                shutil.rmtree(staging)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-manifest", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--table1-receipt", type=Path, default=DEFAULT_TABLE1_RECEIPT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--expected-n", type=int, default=EXPECTED_N)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--asset-timeout-seconds", type=float, default=ASSET_TIMEOUT_SECONDS)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--no-standard-parser", action="store_true")
    parser.add_argument(
        "--validate-packages",
        action="store_true",
        help="recompute every package binding during parent input validation",
    )
    args = parser.parse_args(argv)
    if args.expected_n != EXPECTED_N:
        parser.error(f"--expected-n is frozen at {EXPECTED_N}")
    if args.workers < 1 or args.asset_timeout_seconds <= 0:
        parser.error("workers and timeout must be positive")
    if args.limit is not None and (args.limit < 1 or args.limit > EXPECTED_N):
        parser.error(f"--limit must be between 1 and {EXPECTED_N}")
    return args


def main() -> int:
    argv = list(sys.argv[1:])
    if argv and argv[0] == "--internal-child-job":
        if len(argv) != 4 or argv[2] != "--internal-child-result":
            print("ERROR: invalid internal child invocation", file=sys.stderr)
            return 2
        try:
            return BASE.run_child(Path(argv[1]), Path(argv[3]))
        except BASE.FatalRuntimeBindingError as exc:
            print(f"FATAL_RUNTIME_BINDING: {exc}", file=sys.stderr)
            return BASE.RUNTIME_BINDING_FATAL_EXIT_CODE
        except Exception as exc:  # noqa: BLE001
            print(f"ERROR: internal child failed: {type(exc).__name__}: {exc}", file=sys.stderr)
            return 1

    args = parse_args(argv)
    result = run(args)
    print(
        json.dumps(
            {
                "status": "COMPLETE",
                "output": str(result["output"]),
                "n_eval": result["timing"]["n_eval"],
                "wall_time_seconds": result["timing"]["wall_time_seconds"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
