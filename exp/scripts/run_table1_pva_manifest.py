#!/usr/bin/env python3
"""Run Table 1 structural evaluation on the frozen PV-A per-class cohort."""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import time
from typing import Any
import uuid
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = REPO / "exp/PV-A-per-class-n5-max-joints/manifest.json"
DEFAULT_OUTPUT = REPO / "exp/runtime/table1_pva_per_class_n5_max_joints"
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
DATASET_LABEL = "PV-A-per-class-n5-max-joints"
N_RELEASE = 302_440
RELEASE_CATEGORIES = 531
EXPECTED_N = 2_655
COHORT_TYPE = "CATEGORY_STRATIFIED_N5_WITH_FENCE_FERRIS_MAX_JOINT_OVERRIDES"


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


SHARED = _load_module(
    Path(__file__).with_name("run_table1_artiverse.py"),
    "table1_pva_structural_shared",
)
PACKAGE_HELPERS = _load_module(
    Path(__file__).with_name("prepare_ours_pva_800.py"),
    "table1_pva_package_helpers",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
    return hashlib.sha256(payload.encode()).hexdigest()


def validate_asset_binding(row: dict[str, Any]) -> None:
    package = Path(str(row["package"]))
    if not package.is_absolute() or not package.is_dir():
        raise ValueError(f"invalid package path: {package}")
    relative = str(row.get("primary_urdf_relative_path", ""))
    if relative != "model.urdf":
        raise ValueError(f"unsupported primary URDF path: {relative!r}")
    urdf = package / relative
    if PACKAGE_HELPERS.sha256_file(urdf) != row.get("urdf_sha256"):
        raise ValueError(f"URDF binding mismatch: {row['dataset_id']}")
    live = PACKAGE_HELPERS.package_binding(package)
    if live != row.get("package_binding"):
        raise ValueError(f"package binding mismatch: {row['dataset_id']}")


def load_cohort(
    path: Path, *, expected_n: int, validate_packages: bool
) -> dict[str, Any]:
    path = path.resolve(strict=True)
    manifest = json.loads(path.read_text(encoding="utf-8"))
    expected_hash = manifest.get("manifest_content_sha256")
    unhashed = dict(manifest)
    unhashed.pop("manifest_content_sha256", None)
    if expected_hash != canonical_sha256(unhashed):
        raise ValueError("cohort manifest self-hash mismatch")
    assets = manifest.get("assets")
    if not isinstance(assets, list) or len(assets) != expected_n:
        raise ValueError(f"cohort asset count mismatch: {len(assets) if isinstance(assets, list) else None}")
    if int(manifest.get("n_eval", -1)) != expected_n:
        raise ValueError("cohort n_eval mismatch")
    ids: list[str] = []
    category_counts: Counter[str] = Counter()
    for index, raw in enumerate(assets):
        row = dict(raw)
        if row.get("selection_index") != index:
            raise ValueError(f"selection index mismatch at {index}")
        dataset_id = str(row.get("dataset_id", ""))
        if not dataset_id:
            raise ValueError(f"missing dataset_id at {index}")
        ids.append(dataset_id)
        category = str(row.get("category", ""))
        if not category:
            raise ValueError(f"missing category at {index}")
        category_counts[category] += 1
        if validate_packages:
            validate_asset_binding(row)
    if len(ids) != len(set(ids)):
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
        "file_sha256": PACKAGE_HELPERS.sha256_file(path),
        "content_sha256": expected_hash,
        "manifest": manifest,
        "assets": [dict(row) for row in assets],
    }


def evaluate_asset(row: dict[str, Any]) -> dict[str, Any]:
    dataset_id = str(row["dataset_id"])
    record: dict[str, Any] = {
        "asset_id": dataset_id,
        "dataset_id": dataset_id,
        "source_asset_id": str(row.get("asset_id", "")),
        "raw_category": str(row["category"]),
        "selection_index": int(row["selection_index"]),
        "package": str(row["package"]),
        "status": None,
        "parse_success": False,
        "link_count": None,
        "joint_count": None,
        "joint_type_counts": None,
        "non_fixed_joint_count": None,
        "valid_tree": False,
        "topology_hash": None,
        "fingerprint_complete": False,
        "package_fingerprint": None,
        "referenced_resource_count": None,
        "missing_resources": [],
        "error": None,
    }
    try:
        validate_asset_binding(row)
        urdf = Path(row["package"]) / "model.urdf"
        record["primary_urdf_sha256"] = PACKAGE_HELPERS.sha256_file(urdf)
        record.update(SHARED.analyze_urdf(urdf))
        record["parse_success"] = True
        fingerprint = SHARED.fingerprint_package(urdf)
        record["fingerprint_complete"] = bool(fingerprint["complete"])
        record["package_fingerprint"] = fingerprint["fingerprint"]
        record["referenced_resource_count"] = fingerprint["resource_count"]
        record["missing_resources"] = fingerprint["missing_resources"]
        if fingerprint["complete"]:
            record["status"] = "EVALUATED"
        else:
            record["status"] = "EVALUATED_FINGERPRINT_INCOMPLETE"
            record["error"] = "simulation resource closure incomplete"
    except (ET.ParseError, OSError, ValueError, KeyError) as error:
        record["status"] = "ASSET_EVALUATION_FAILED"
        record["error"] = f"{type(error).__name__}: {error}"
    except Exception as error:  # fail closed
        record["status"] = "ASSET_EVALUATION_FAILED"
        record["error"] = f"{type(error).__name__}: {error}"
    return record


def report(summary: dict[str, Any], timing: dict[str, Any]) -> str:
    cohort = summary["cohort"]
    return "\n".join(
        [
            "# PV-A per-class N=5 Table 1",
            "",
            f"Custom cohort: `{COHORT_TYPE}`. This is not the protocol headline shared-category cohort.",
            "",
            f"- N_release: {cohort['N_release']}",
            f"- N_eval: {cohort['N_eval']}",
            f"- Categories (release / eval): {cohort['release_raw_categories']} / {cohort['eval_raw_categories']}",
            f"- Links/asset (mean / median / P90): {summary['links_per_asset']['mean']:.2f} / {summary['links_per_asset']['median']} / {summary['links_per_asset']['p90_nearest_rank']}",
            f"- Movable joints/asset (mean / median / P90): {summary['movable_joints_per_asset']['mean']:.2f} / {summary['movable_joints_per_asset']['median']} / {summary['movable_joints_per_asset']['p90_nearest_rank']}",
            f"- Multi-joint assets: {summary['multi_joint_assets']['rate']}",
            f"- Unique topologies: {summary['unique_topologies']['rate']} (n={summary['unique_topologies']['denominator']})",
            f"- Exact duplicate rate: {summary['exact_duplicate_rate']['rate']} (n={summary['exact_duplicate_rate']['denominator']})",
            f"- Wall time: {timing['wall_time_seconds']:.6f} seconds",
            "",
        ]
    )


def run(args: argparse.Namespace) -> dict[str, Any]:
    wall_started = time.perf_counter()
    started_at = utc_now()
    input_started = time.perf_counter()
    cohort = load_cohort(
        args.input_manifest,
        expected_n=args.expected_n,
        validate_packages=False,
    )
    rows = cohort["assets"][: args.limit] if args.limit is not None else cohort["assets"]
    for row in rows:
        validate_asset_binding(row)
    input_seconds = time.perf_counter() - input_started

    output = args.output.absolute()
    if output.exists():
        raise FileExistsError(output)
    staging = output.parent / f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}"
    staging.mkdir(parents=True, exist_ok=False)
    try:
        protocol = args.protocol.resolve(strict=True)
        run_manifest = {
            "schema_version": "table1_pva_manifest_v1",
            "dataset": DATASET_LABEL,
            "classification": "CUSTOM_COHORT_EVIDENCE" if args.limit is None else "SMOKE",
            "cohort_type": COHORT_TYPE,
            "N_release": N_RELEASE,
            "N_eval": len(rows),
            "release_raw_category_count": RELEASE_CATEGORIES,
            "source_manifest": str(cohort["path"]),
            "source_manifest_sha256": cohort["file_sha256"],
            "source_manifest_content_sha256": cohort["content_sha256"],
            "protocol": str(protocol),
            "protocol_sha256": PACKAGE_HELPERS.sha256_file(protocol),
            "runner": str(Path(__file__).resolve()),
            "runner_sha256": PACKAGE_HELPERS.sha256_file(Path(__file__).resolve()),
            "shared_evaluator": str(Path(__file__).with_name("run_table1_artiverse.py").resolve()),
            "shared_evaluator_sha256": PACKAGE_HELPERS.sha256_file(Path(__file__).with_name("run_table1_artiverse.py")),
            "workers": args.workers,
            "started_at_utc": started_at,
            "assets": [
                {
                    **row,
                    "selection_rank": index + 1,
                    "primary_urdf_sha256": row["urdf_sha256"],
                    "raw_category": row["category"],
                }
                for index, row in enumerate(rows)
            ],
        }
        SHARED.write_json(staging / "manifest.json", run_manifest)

        evaluation_started = time.perf_counter()
        if args.workers == 1:
            records = [evaluate_asset(row) for row in rows]
        else:
            with ThreadPoolExecutor(max_workers=args.workers) as executor:
                records = list(executor.map(evaluate_asset, rows))
        evaluation_seconds = time.perf_counter() - evaluation_started
        SHARED.write_jsonl(staging / "asset_records.jsonl", records)

        finalize_started = time.perf_counter()
        summary = SHARED.aggregate_records(
            records,
            release_asset_count=N_RELEASE,
            release_category_count=RELEASE_CATEGORIES,
        )
        summary["status_counts"] = dict(sorted(Counter(str(row["status"]) for row in records).items()))
        summary["dataset"] = DATASET_LABEL
        summary["classification"] = run_manifest["classification"]
        summary["cohort"]["cohort_type"] = COHORT_TYPE
        summary["completed_at_utc"] = utc_now()
        SHARED.write_json(staging / "summary.json", summary)
        artifact_finalize_seconds = time.perf_counter() - finalize_started
        completed_at = utc_now()
        timing = {
            "schema_version": "experiment-timing/v1",
            "table": "Table 1",
            "started_at_utc": started_at,
            "completed_at_utc": completed_at,
            "wall_time_seconds": time.perf_counter() - wall_started,
            "input_validation_seconds": input_seconds,
            "evaluation_seconds": evaluation_seconds,
            "artifact_finalize_seconds": artifact_finalize_seconds,
            "workers": args.workers,
            "n_eval": len(rows),
            "measurement_endpoint": "provisional_before_atomic_publish",
        }
        SHARED.write_json(staging / "timing.json", timing)
        (staging / "report.md").write_text(report(summary, timing), encoding="utf-8")
        artifact_names = (
            "manifest.json",
            "asset_records.jsonl",
            "summary.json",
            "report.md",
            "timing.json",
        )
        SHARED.write_json(
            staging / "artifact_manifest.json",
            {
                "schema_version": 1,
                "files": {
                    name: {
                        "bytes": (staging / name).stat().st_size,
                        "sha256": PACKAGE_HELPERS.sha256_file(staging / name),
                    }
                    for name in artifact_names
                },
            },
        )
        os.rename(staging, output)
        timing.update(
            {
                "completed_at_utc": utc_now(),
                "wall_time_seconds": time.perf_counter() - wall_started,
                "artifact_finalize_seconds": time.perf_counter() - finalize_started,
                "measurement_endpoint": (
                    "immediately_after_atomic_publish_before_final_timing_receipt_update"
                ),
            }
        )
        SHARED.write_json(output / "timing.json", timing)
        (output / "report.md").write_text(report(summary, timing), encoding="utf-8")
        SHARED.write_json(
            output / "artifact_manifest.json",
            {
                "schema_version": 1,
                "files": {
                    name: {
                        "bytes": (output / name).stat().st_size,
                        "sha256": PACKAGE_HELPERS.sha256_file(output / name),
                    }
                    for name in artifact_names
                },
            },
        )
        return {"summary": summary, "timing": timing, "output": output}
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-manifest", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--expected-n", type=int, default=EXPECTED_N)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--limit", type=int)
    args = parser.parse_args(argv)
    if args.workers < 1 or args.expected_n < 1 or (args.limit is not None and args.limit < 1):
        parser.error("counts and workers must be positive")
    if args.expected_n != EXPECTED_N:
        parser.error(f"--expected-n is frozen at {EXPECTED_N}")
    return args


def main() -> int:
    args = parse_args()
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
