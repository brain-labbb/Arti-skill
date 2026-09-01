#!/usr/bin/env python3
"""Evaluate Table 2 on the frozen SketchMobility Table 1 N=800 cohort.

This runner is a dataset-specific provenance adapter. It does not implement
Table 2 metrics: every asset is audited by the unchanged shared core in
``run_table2_urdf_articraft.py`` through the existing fresh-interpreter,
owned-process-group harness in ``run_table2_urdf_ours_500k.py``.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import platform
import shutil
import sys
from typing import Any
import uuid


REPO = Path(__file__).resolve().parents[2]
SCRIPT_PATH = Path(__file__).resolve()
DEFAULT_DATASET_ROOT = REPO / "exp/SketchMobility"
DEFAULT_TABLE1_RECEIPT = (
    REPO / "exp/runtime/table1_sketch_mobility_rerun_20260821T021838Z"
)
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
TABLE1_RUNNER_PATH = REPO / "exp/scripts/run_table1_sketch_mobility.py"
TABLE2_EVALUATOR_PATH = REPO / "exp/scripts/run_table2_urdf_articraft.py"
CHILD_HARNESS_PATH = REPO / "exp/scripts/run_table2_urdf_ours_500k.py"
DATASET_LABEL = "SketchMobility"
EXPECTED_N_RELEASE = 4956
EXPECTED_N_EVAL = 800
EXPECTED_RELEASE_CATEGORY_COUNT = 70
EXPECTED_EVAL_CATEGORY_COUNT = 67
EXPECTED_SELECTION_PROTOCOL = "sketch-mobility-table1-global-sample-v1"
EXPECTED_SEED = "arti-skill-table1-sketch-mobility-n800-v1"
EXPECTED_TABLE1_MANIFEST_SHA256 = (
    "081e9e9125f8945cad67a751949e659f6d4e73817704c07cd3fcd4b657ffc696"
)
EXPECTED_TABLE1_RECORDS_SHA256 = (
    "4c7dc19d2a0558e07e6a0f42ce12bd96e7a8b199849c885374a3e396f7b16cca"
)
EXPECTED_RELEASE_ROSTER_SHA256 = (
    "9b3f3776162e59baa0b73996b398f97bc52b632b966cedef7bb83ff99acdb765"
)
EXPECTED_RELEASE_MANIFEST_SHA256 = (
    "5b4b0891bafeba5029e5e1dd71042e5be8543eccb2990edfbc9aba71a1ac56fb"
)
EXPECTED_RELEASE_UNIVERSE_SHA256 = (
    "a9c128d24ab9cba03d593ecff17f1e7284ef1f74f09f69939cbef7c0cc8af346"
)
EXPECTED_RELEASE_CHUNKS = (
    "sketchmobility_data-00001-of-00003.tar.gz",
    "sketchmobility_data-00002-of-00003.tar.gz",
    "sketchmobility_data-00003-of-00003.tar.gz",
)
ASSET_TIMEOUT_SECONDS = 120.0
FORMAL_WORKERS = 4
FORMAL_PYTHON_VERSION = "3.12.3"
FORMAL_URDFPY_VERSION = "0.0.22"


def _load_module(path: Path, name: str):
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load required module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


TABLE1 = _load_module(TABLE1_RUNNER_PATH, "table2_sketch_table1_shared")
HARNESS = _load_module(CHILD_HARNESS_PATH, "table2_sketch_child_harness_shared")
TABLE2 = HARNESS.TABLE2
SHARED = TABLE1.SHARED


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _safe_asset_package(dataset_root: Path, asset_id: str) -> Path:
    relative = PurePosixPath(asset_id)
    if relative.is_absolute() or len(relative.parts) != 4:
        raise ValueError(f"invalid SketchMobility asset path: {asset_id!r}")
    if relative.parts[0] != "data" or any(part in {"", ".", ".."} for part in relative.parts):
        raise ValueError(f"invalid SketchMobility asset path: {asset_id!r}")
    package = (dataset_root / Path(*relative.parts)).resolve(strict=True)
    package.relative_to(dataset_root)
    if not package.is_dir() or package.is_symlink():
        raise ValueError(f"asset package is not a regular directory: {asset_id}")
    return package


def _load_jsonl_by_asset(path: Path) -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSONL at line {line_number}: {path}") from exc
            asset_id = record.get("asset_id")
            if not isinstance(asset_id, str) or not asset_id or asset_id in records:
                raise ValueError(f"invalid or duplicate asset ID at line {line_number}: {path}")
            records[asset_id] = record
    return records


def load_table1_cohort(receipt_dir: Path, dataset_root: Path) -> dict[str, Any]:
    """Load and bind the exact frozen Table 1 SketchMobility cohort."""
    receipt_dir = receipt_dir.resolve(strict=True)
    dataset_root = dataset_root.resolve(strict=True)
    manifest_path = receipt_dir / "manifest.json"
    records_path = receipt_dir / "asset_records.jsonl"
    roster_path = receipt_dir / "release_roster.jsonl"
    observed_hashes = {
        "manifest": TABLE2.sha256_file(manifest_path),
        "asset_records": TABLE2.sha256_file(records_path),
        "release_roster": TABLE2.sha256_file(roster_path),
    }
    expected_hashes = {
        "manifest": EXPECTED_TABLE1_MANIFEST_SHA256,
        "asset_records": EXPECTED_TABLE1_RECORDS_SHA256,
        "release_roster": EXPECTED_RELEASE_ROSTER_SHA256,
    }
    if observed_hashes != expected_hashes:
        raise ValueError(
            f"Table 1 receipt hash mismatch: {observed_hashes} != {expected_hashes}"
        )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    required_identity = {
        "dataset": DATASET_LABEL,
        "N_release": EXPECTED_N_RELEASE,
        "N_eval": EXPECTED_N_EVAL,
        "seed": EXPECTED_SEED,
        "selection_protocol": EXPECTED_SELECTION_PROTOCOL,
        "release_manifest_sha256": EXPECTED_RELEASE_MANIFEST_SHA256,
        "release_universe_sha256": EXPECTED_RELEASE_UNIVERSE_SHA256,
        "release_roster_sha256": EXPECTED_RELEASE_ROSTER_SHA256,
    }
    observed_identity = {key: manifest.get(key) for key in required_identity}
    if observed_identity != required_identity:
        raise ValueError(
            f"Table 1 cohort identity mismatch: {observed_identity} != {required_identity}"
        )
    raw_assets = manifest.get("assets")
    if not isinstance(raw_assets, list) or len(raw_assets) != EXPECTED_N_EVAL:
        raise ValueError("Table 1 manifest must contain exactly 800 assets")
    table1_records = _load_jsonl_by_asset(records_path)
    if len(table1_records) != EXPECTED_N_EVAL:
        raise ValueError("Table 1 asset records must contain exactly 800 assets")

    rows: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    for expected_rank, raw in enumerate(raw_assets, start=1):
        if not isinstance(raw, dict):
            raise ValueError(f"Table 1 asset row {expected_rank} is not an object")
        asset_id = raw.get("asset_id")
        if not isinstance(asset_id, str) or asset_id in seen_ids:
            raise ValueError(f"invalid or duplicate Table 1 asset ID at rank {expected_rank}")
        seen_ids.add(asset_id)
        if raw.get("selection_rank") != expected_rank:
            raise ValueError(f"Table 1 selection rank mismatch: {asset_id}")
        expected_asset_id = "/".join(
            (
                "data",
                str(raw.get("source")),
                str(raw.get("category")),
                str(raw.get("display_id")),
            )
        )
        if asset_id != expected_asset_id:
            raise ValueError(f"Table 1 asset metadata/path mismatch: {asset_id}")
        source_category = f"{raw.get('source')}/{raw.get('category')}"
        if raw.get("source_category") != source_category:
            raise ValueError(f"Table 1 source category mismatch: {asset_id}")
        record = table1_records.get(asset_id)
        if record is None:
            raise ValueError(f"Table 1 record missing: {asset_id}")
        if record.get("status") not in {
            "EVALUATED",
            "EVALUATED_FINGERPRINT_INCOMPLETE",
        }:
            raise ValueError(f"Table 1 record is not retained/evaluated: {asset_id}")
        primary_hash = str(raw.get("mobility_urdf_sha256") or "")
        if record.get("primary_urdf_sha256") != primary_hash:
            raise ValueError(f"Table 1 primary URDF binding mismatch: {asset_id}")

        package = _safe_asset_package(dataset_root, asset_id)
        primary = package / "mobility.urdf"
        annotation = package / "annotation.json"
        if primary.is_symlink() or annotation.is_symlink():
            raise ValueError(f"selected asset contains a required-file symlink: {asset_id}")
        if TABLE2.sha256_file(primary) != primary_hash:
            raise ValueError(f"live mobility.urdf hash mismatch: {asset_id}")
        if TABLE2.sha256_file(annotation) != raw.get("annotation_sha256"):
            raise ValueError(f"live annotation.json hash mismatch: {asset_id}")
        rows.append(
            {
                **raw,
                "primary_urdf_sha256": primary_hash,
                "asset_root": asset_id,
                "raw_category": source_category,
                "seed_name": str(raw["display_id"]),
                "primary_urdf": f"{asset_id}/mobility.urdf",
            }
        )

    if set(table1_records) != seen_ids:
        raise ValueError("Table 1 manifest and asset-record asset sets differ")
    return {
        "manifest": manifest,
        "manifest_sha256": observed_hashes["manifest"],
        "asset_records_sha256": observed_hashes["asset_records"],
        "release_roster_sha256": observed_hashes["release_roster"],
        "rows": rows,
        "release_asset_count": EXPECTED_N_RELEASE,
        "release_category_count": EXPECTED_RELEASE_CATEGORY_COUNT,
    }


def build_jobs(
    rows: list[dict[str, Any]],
    dataset_root: Path,
    *,
    manifest_content_sha256: str,
    run_standard_parser: bool,
) -> list[dict[str, Any]]:
    """Bind selected packages and build jobs for the frozen child harness."""
    dataset_root = dataset_root.resolve(strict=True)
    evaluator = HARNESS.evaluator_binding()
    environment = HARNESS.environment_binding()
    jobs: list[dict[str, Any]] = []
    for row in rows:
        asset_id = str(row["asset_id"])
        package = _safe_asset_package(dataset_root, asset_id)
        package_binding = TABLE2.package_binding(package)
        urdf_sha256 = TABLE2.sha256_file(package / "mobility.urdf")
        if urdf_sha256 != str(row["primary_urdf_sha256"]):
            raise ValueError(f"mobility.urdf hash mismatch for {asset_id}")
        jobs.append(
            {
                "asset_id": asset_id,
                "raw_category": str(row["source_category"]),
                "seed_name": str(row["display_id"]),
                "asset_root": asset_id,
                "primary_urdf": f"{asset_id}/mobility.urdf",
                "table1_selection_rank": int(row["selection_rank"]),
                "package": str(package),
                "package_binding": package_binding,
                "run_standard_parser": run_standard_parser,
                "primary_urdf_relative_path": "mobility.urdf",
                "model_urdf_sha256": urdf_sha256,
                "primary_urdf_sha256": urdf_sha256,
                "manifest_content_sha256": manifest_content_sha256,
                "evaluator_binding": evaluator,
                "environment_binding": environment,
            }
        )
    return jobs


def validate_release_manifest_identity(dataset_root: Path) -> str:
    manifest_path = dataset_root / "dataset_chunks/manifest.json"
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise ValueError(f"release manifest is not a regular file: {manifest_path}")
    observed = TABLE2.sha256_file(manifest_path)
    if observed != EXPECTED_RELEASE_MANIFEST_SHA256:
        raise ValueError(
            "release manifest hash mismatch: "
            f"{observed} != {EXPECTED_RELEASE_MANIFEST_SHA256}"
        )
    return observed


def _reverify_full_release(dataset_root: Path, output: Path) -> dict[str, Any]:
    release_manifest_sha256 = validate_release_manifest_identity(dataset_root)
    release_manifest, identities = TABLE1.load_release_manifest(dataset_root)
    chunk_names = tuple(str(row.get("filename") or "") for row in release_manifest["chunks"])
    if chunk_names != EXPECTED_RELEASE_CHUNKS:
        raise ValueError(f"unexpected release chunks: {chunk_names}")
    universe_bytes = "".join(
        f"{row['asset_id']}\n" for row in sorted(identities, key=lambda row: row["asset_id"])
    ).encode("utf-8")
    release_universe_sha256 = hashlib.sha256(universe_bytes).hexdigest()
    if release_universe_sha256 != EXPECTED_RELEASE_UNIVERSE_SHA256:
        raise ValueError(
            "live release universe hash mismatch: "
            f"{release_universe_sha256} != {EXPECTED_RELEASE_UNIVERSE_SHA256}"
        )
    chunk_verification = TABLE1.verify_chunks(dataset_root, release_manifest)
    roster = TABLE1.build_release_roster(dataset_root, identities)
    roster_path = output / "release_roster_reverified.jsonl"
    TABLE1.SHARED.write_jsonl(roster_path, roster)
    roster_hash = TABLE2.sha256_file(roster_path)
    if roster_hash != EXPECTED_RELEASE_ROSTER_SHA256:
        raise ValueError(
            f"live release roster hash mismatch: {roster_hash} != {EXPECTED_RELEASE_ROSTER_SHA256}"
        )
    status_counts = dict(Counter(str(row["roster_status"]) for row in roster))
    if status_counts != {"COMPLETE": EXPECTED_N_RELEASE}:
        raise ValueError(f"live release roster is incomplete: {status_counts}")
    return {
        "release_manifest_sha256": release_manifest_sha256,
        "release_universe_sha256": release_universe_sha256,
        "release_roster_sha256": roster_hash,
        "release_roster_status_counts": status_counts,
        "chunk_verification": chunk_verification,
    }


def _summary_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    lines = [
        "# SketchMobility Table 2 URDF audit",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            f"Frozen cohort: N={summary['n_eval']} in exact Table 1 selection-rank order; "
            "no resampling, replacement or outcome filtering."
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
                    "Category macro average: unweighted mean over "
                    f"{macro['category_count']} observed source/category groups."
                ),
                "",
                "| Metric | Category macro |",
                "|---|---:|",
            ]
        )
        for name in TABLE2.METRIC_NAMES:
            rate = macro["metrics"][name]["rate"]
            lines.append(f"| {name} | {'N/A' if rate is None else f'{100 * rate:.2f}%'} |")
    lines.append("")
    return "\n".join(lines)


def _run_to_output(args: argparse.Namespace, output: Path) -> dict[str, Any]:
    dataset_root = args.dataset_root.resolve(strict=True)
    started_at = utc_now()
    cohort = load_table1_cohort(args.table1_receipt, dataset_root)
    rows = cohort["rows"][: args.limit] if args.limit is not None else cohort["rows"]
    expected_rows = args.limit if args.limit is not None else EXPECTED_N_EVAL
    if len(rows) != expected_rows:
        raise ValueError("frozen cohort row count mismatch")

    release_reverification = None
    if args.limit is None:
        release_reverification = _reverify_full_release(dataset_root, output)

    environment = TABLE2.environment_metadata(not args.no_standard_parser, args.workers)
    config = {
        **TABLE2.EVALUATOR_CONFIG,
        "dataset": DATASET_LABEL,
        "workers": args.workers,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "selection_algorithm": (
            "exact existing Table 1 SketchMobility manifest .assets[] order; "
            "no resampling/reselection"
        ),
        "package_staging_policy": (
            "no staging: each data/{source}/{category}/{display_id} directory is an "
            "already self-contained package with mobility.urdf and meshes/; full package "
            "content binding is reverified before and after each audit"
        ),
        "category_policy": "exact declared source/category pair; no semantic merging",
    }
    protocol_binding = TABLE2.freeze_protocol_snapshot(output)
    evaluation = {
        "command": [str(SCRIPT_PATH), *sys.argv[1:]],
        "config": config,
        "config_sha256": TABLE2.canonical_sha256(config),
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "workers": args.workers,
        "metrics": list(TABLE2.METRIC_NAMES),
        "denominator_policy": config["denominator_policy"],
        "parse_rate_scope": config["parse_rate_scope"],
        "resource_validation_scope": config["resource_validation_scope"],
        "evaluator_path": str(TABLE2_EVALUATOR_PATH.resolve()),
        "evaluator_sha256": TABLE2.sha256_file(TABLE2_EVALUATOR_PATH),
        "orchestrator_path": str(SCRIPT_PATH),
        "orchestrator_sha256": TABLE2.sha256_file(SCRIPT_PATH),
        "child_harness_path": str(CHILD_HARNESS_PATH.resolve()),
        "child_harness_sha256": TABLE2.sha256_file(CHILD_HARNESS_PATH),
        "table1_runner_path": str(TABLE1_RUNNER_PATH.resolve()),
        "table1_runner_sha256": TABLE2.sha256_file(TABLE1_RUNNER_PATH),
        "environment": environment,
        "environment_sha256": TABLE2.canonical_sha256(environment),
        **protocol_binding,
    }
    assets = [
        {
            "selection_index": int(row["selection_rank"]) - 1,
            "selection_rank": int(row["selection_rank"]),
            "selection_hash": str(row["selection_hash"]),
            "asset_id": str(row["asset_id"]),
            "source": str(row["source"]),
            "category": str(row["category"]),
            "source_category": str(row["source_category"]),
            "display_id": str(row["display_id"]),
            "primary_urdf_relative_path": "mobility.urdf",
            "primary_urdf_sha256": str(row["primary_urdf_sha256"]),
            "package": str(_safe_asset_package(dataset_root, str(row["asset_id"]))),
        }
        for row in rows
    ]
    run_manifest = {
        "schema_version": "1.0.0-sketch-mobility",
        "created_at": started_at,
        "dataset": DATASET_LABEL,
        "mode": "formal" if args.limit is None else "smoke",
        "classification": "FORMAL" if args.limit is None else "SMOKE",
        "source": {
            "dataset_root": str(dataset_root),
            "table1_receipt": str(args.table1_receipt.resolve(strict=True)),
            "table1_manifest_sha256": cohort["manifest_sha256"],
            "table1_asset_records_sha256": cohort["asset_records_sha256"],
            "table1_release_roster_sha256": cohort["release_roster_sha256"],
            "table1_protocol_sha256": cohort["manifest"]["protocol_sha256"],
            "selection_protocol": EXPECTED_SELECTION_PROTOCOL,
            "seed": EXPECTED_SEED,
            "cohort_type": cohort["manifest"]["cohort_type"],
            "release_reverification": release_reverification,
        },
        "evaluation": evaluation,
        "n_requested": expected_rows,
        "n_eval": len(rows),
        "assets": assets,
    }
    run_manifest["manifest_content_sha256"] = TABLE2.manifest_self_hash(run_manifest)
    TABLE2.atomic_write_json(output / "manifest.json", run_manifest)
    TABLE2.atomic_write_json(output / "environment.json", environment)

    jobs = build_jobs(
        rows,
        dataset_root,
        manifest_content_sha256=run_manifest["manifest_content_sha256"],
        run_standard_parser=not args.no_standard_parser,
    )
    checkpoint_counts: Counter[str] = Counter()

    def checkpoint(record: dict[str, Any]) -> None:
        checkpoint_counts[str(record.get("status"))] += 1
        TABLE2.atomic_write_json(
            output / "checkpoint.json",
            {
                "updated_at": utc_now(),
                "completed": sum(checkpoint_counts.values()),
                "total": len(jobs),
                "status_counts": dict(sorted(checkpoint_counts.items())),
            },
        )

    records = HARNESS.execute_audit_jobs(
        jobs,
        worker_scratch=output / ".worker_scratch",
        timeout_seconds=args.asset_timeout_seconds,
        max_workers=args.workers,
        on_result=checkpoint,
    )
    TABLE2.atomic_write_jsonl(output / "asset_records.jsonl", records)
    summary = TABLE2.aggregate_records(records, len(rows), category_field="raw_category")
    summary["dataset"] = DATASET_LABEL
    summary["classification"] = run_manifest["classification"]
    summary["created_at"] = started_at
    summary["completed_at"] = utc_now()
    summary["cohort"] = {
        "N_release": EXPECTED_N_RELEASE,
        "N_eval": len(rows),
        "release_raw_categories": EXPECTED_RELEASE_CATEGORY_COUNT,
        "eval_raw_categories": len({str(row["source_category"]) for row in rows}),
        "cohort_type": cohort["manifest"]["cohort_type"],
        "table1_manifest_sha256": cohort["manifest_sha256"],
    }
    TABLE2.atomic_write_json(output / "summary.json", summary)
    TABLE2.atomic_write_text(output / "summary.md", _summary_markdown(summary, run_manifest))

    artifact_files = [
        "manifest.json",
        "asset_records.jsonl",
        "summary.json",
        "summary.md",
        "environment.json",
        "checkpoint.json",
        TABLE2.PROTOCOL_SNAPSHOT_NAME,
    ]
    if release_reverification is not None:
        artifact_files.append("release_roster_reverified.jsonl")
    artifact_manifest = {
        "schema_version": 1,
        "created_at_utc": utc_now(),
        "files": {
            name: {
                "bytes": (output / name).stat().st_size,
                "sha256": TABLE2.sha256_file(output / name),
            }
            for name in artifact_files
        },
    }
    TABLE2.atomic_write_json(output / "artifact_manifest.json", artifact_manifest)
    return summary


def _allocate_final_version(output: Path) -> tuple[Path, Path | None]:
    runs_root = output.parent / f".{output.name}.runs"
    runs_root.mkdir(parents=True, exist_ok=True)
    old_version: Path | None = None
    if output.is_symlink():
        old_version = output.resolve(strict=True)
        try:
            old_version.relative_to(runs_root.resolve())
        except ValueError as error:
            raise RuntimeError(
                f"existing output pointer is outside its managed run directory: {output}"
            ) from error
    elif output.exists():
        raise RuntimeError(
            f"existing output is not an atomic run pointer; choose a new output path: {output}"
        )

    version = runs_root / f"run.{uuid.uuid4().hex}"
    version.mkdir(parents=False, exist_ok=False)
    return version, old_version


def _publish_final_version(
    version: Path,
    output: Path,
    old_version: Path | None,
) -> None:
    temporary_pointer = output.parent / f".{output.name}.pointer.{uuid.uuid4().hex}"
    try:
        temporary_pointer.symlink_to(os.path.relpath(version, output.parent))
        os.replace(temporary_pointer, output)
    except BaseException:
        if temporary_pointer.is_symlink():
            temporary_pointer.unlink()
        raise
    if old_version is not None and old_version != version:
        shutil.rmtree(old_version)


def run(args: argparse.Namespace) -> dict[str, Any]:
    output = args.output.absolute()
    dataset_root = args.dataset_root.resolve()
    try:
        output.relative_to(dataset_root)
    except ValueError:
        pass
    else:
        raise ValueError("output must not be inside the SketchMobility dataset root")
    with SHARED._output_lock(output):
        version, old_version = _allocate_final_version(output)
        try:
            summary = _run_to_output(args, version)
            manifest = json.loads((version / "manifest.json").read_text(encoding="utf-8"))
            TABLE2.validate_protocol_snapshot_binding(version, manifest["evaluation"])
            SHARED._verify_staged_artifacts(version)
            _publish_final_version(version, output, old_version)
            return summary
        finally:
            if version.exists() and not (
                output.is_symlink() and output.resolve(strict=True) == version
            ):
                shutil.rmtree(version)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--table1-receipt", type=Path, default=DEFAULT_TABLE1_RECEIPT)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--workers", type=int, default=FORMAL_WORKERS)
    parser.add_argument("--asset-timeout-seconds", type=float, default=ASSET_TIMEOUT_SECONDS)
    parser.add_argument("--no-standard-parser", action="store_true")
    return parser.parse_args(argv)


def validate_formal_runtime(python_version: str, urdfpy_version: str) -> None:
    if python_version != FORMAL_PYTHON_VERSION:
        raise ValueError(f"formal mode requires Python {FORMAL_PYTHON_VERSION}")
    if urdfpy_version != FORMAL_URDFPY_VERSION:
        raise ValueError(f"formal mode requires urdfpy {FORMAL_URDFPY_VERSION}")


def _validate_args(args: argparse.Namespace) -> None:
    if args.workers <= 0 or args.asset_timeout_seconds <= 0:
        raise ValueError("--workers and --asset-timeout-seconds must be positive")
    if args.limit is not None and args.limit <= 0:
        raise ValueError("--limit must be positive")
    if args.limit is None:
        required = {
            "dataset_root": DEFAULT_DATASET_ROOT.resolve(strict=False),
            "table1_receipt": DEFAULT_TABLE1_RECEIPT.resolve(strict=False),
            "workers": FORMAL_WORKERS,
            "asset_timeout_seconds": ASSET_TIMEOUT_SECONDS,
            "no_standard_parser": False,
        }
        observed = {
            "dataset_root": args.dataset_root.resolve(strict=False),
            "table1_receipt": args.table1_receipt.resolve(strict=False),
            "workers": args.workers,
            "asset_timeout_seconds": args.asset_timeout_seconds,
            "no_standard_parser": args.no_standard_parser,
        }
        if observed != required:
            raise ValueError(f"formal mode requires exactly {required}; observed {observed}")
        validate_formal_runtime(
            platform.python_version(),
            importlib.metadata.version("urdfpy"),
        )


def main() -> int:
    args = parse_args()
    _validate_args(args)
    if args.output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        suffix = f"smoke_n{args.limit}" if args.limit is not None else "n800"
        args.output = REPO / (
            f"exp/runtime/table2_urdf_sketch_mobility_table1cohort_{suffix}_{stamp}"
        )
    summary = run(args)
    print(
        json.dumps(
            {
                "state": "COMPLETE",
                "n_eval": summary["n_eval"],
                "strict_urdf_pass": summary["strict_urdf_pass"]["passed"],
                "output": str(args.output.absolute()),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
