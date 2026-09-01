#!/usr/bin/env python3
"""Evaluate Table 1 on the frozen PartNet-Mobility N=800 cohort."""

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
import sys
from typing import Any
import uuid
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
DEFAULT_DATASET_ROOT = REPO / "exp/PartNet-Mobility"
DEFAULT_INPUT_MANIFEST = (
    REPO
    / "exp/runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json"
)
DEFAULT_OUTPUT = REPO / "exp/runtime/table1_partnet_mobility"
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
EXPECTED_PROTOCOL_ID = "urdf_sim_ready_table4_partnet_mobility_n800_v1"
EXPECTED_RELEASE_CATEGORY_COUNT = 46


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


SHARED = _load_module(
    Path(__file__).with_name("run_table1_artiverse.py"),
    "table1_structural_metrics_shared",
)
TABLE2 = _load_module(
    Path(__file__).with_name("run_table2_urdf_articraft.py"),
    "table2_urdf_frozen_cohort_shared",
)
TABLE4 = _load_module(
    Path(__file__).with_name("run_urdf_table4_partnet_mobility.py"),
    "table4_partnet_frozen_contract_shared",
)


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _reject_package_symlinks(package: Path) -> None:
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            path = current / name
            if path.is_symlink():
                raise ValueError(
                    f"package contains directory symlink: {path.relative_to(package)}"
                )
        for name in file_names:
            path = current / name
            if path.is_symlink():
                raise ValueError(
                    f"package contains file symlink: {path.relative_to(package)}"
                )


def _top_level_urdf(package: Path) -> Path:
    candidates = sorted(
        path for path in package.iterdir() if path.suffix.lower() == ".urdf"
    )
    if len(candidates) != 1 or candidates[0].name != "mobility.urdf":
        raise ValueError(
            f"package must contain exactly one top-level URDF named mobility.urdf: {package.name}"
        )
    primary = candidates[0]
    if primary.is_symlink() or not primary.is_file():
        raise ValueError(f"mobility.urdf must be a regular non-symlink file: {package.name}")
    return primary


def _release_categories(source_root: Path) -> tuple[dict[str, str], str]:
    rows: list[dict[str, str]] = []
    categories: dict[str, str] = {}
    release_ids = sorted(
        (path.name for path in source_root.iterdir() if path.is_dir()),
        key=int,
    )
    for dataset_id in release_ids:
        meta_path = source_root / dataset_id / "meta.json"
        if meta_path.is_symlink() or not meta_path.is_file():
            raise ValueError(f"meta.json must be a regular non-symlink file: {dataset_id}")
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f"meta.json is unavailable or invalid: {dataset_id}") from error
        category = str(meta.get("model_cat", "")).strip()
        if not category:
            raise ValueError(f"meta.json model_cat is missing: {dataset_id}")
        categories[dataset_id] = category
        rows.append({"dataset_id": dataset_id, "category": category})
    return categories, _canonical_sha256(rows)


def _validate_selection_contract(manifest: dict[str, Any], expected_n: int) -> None:
    if manifest.get("protocol_id") != EXPECTED_PROTOCOL_ID:
        raise ValueError("unexpected PartNet-Mobility frozen protocol")
    if manifest.get("status") != "FROZEN" or manifest.get("qualification_smoke") is not False:
        raise ValueError("PartNet-Mobility cohort must be a formal frozen selection")
    if manifest.get("sample_size") != expected_n:
        raise ValueError(f"expected exactly {expected_n} frozen PartNet-Mobility items")
    selection = manifest.get("selection_policy", {})
    if (
        selection.get("outcome_based_filtering") is not False
        or selection.get("selected_failures_retained_without_replacement") is not True
    ):
        raise ValueError("PartNet-Mobility selection permits outcome-based replacement")
    boundary = manifest.get("cohort_boundary", {})
    if (
        boundary.get("is_full_release_cohort") is not False
        or boundary.get("is_shared_category_balanced_cohort") is not False
    ):
        raise ValueError("PartNet-Mobility frozen cohort boundary is inconsistent")


def _validate_release_category_count(count: int, *, formal: bool) -> None:
    if formal and count != EXPECTED_RELEASE_CATEGORY_COUNT:
        raise ValueError(
            f"formal PartNet-Mobility release must contain exactly "
            f"{EXPECTED_RELEASE_CATEGORY_COUNT} release categories"
        )


def _validate_frozen_asset(identity: dict[str, Any]) -> None:
    package = Path(identity["package"])
    _reject_package_symlinks(package)
    primary = _top_level_urdf(package)
    if SHARED.sha256_file(primary) != identity["primary_urdf_sha256"]:
        raise ValueError("mobility.urdf changed after cohort freeze")
    meta = json.loads((package / "meta.json").read_text(encoding="utf-8"))
    if str(meta.get("model_cat", "")).strip() != identity["raw_category"]:
        raise ValueError("category metadata mismatch after cohort freeze")
    TABLE4.validate_frozen_asset_files(identity["source_item"], package)


def _validate_asset(identity: dict[str, Any]) -> None:
    _validate_frozen_asset(identity)
    package = Path(identity["package"])
    if TABLE2.package_binding(package) != identity["package_binding"]:
        raise ValueError("package changed after Table 1 snapshot")


def load_frozen_cohort(
    manifest_path: Path,
    *,
    source_root: Path,
    expected_n: int,
    formal: bool,
) -> dict[str, Any]:
    manifest_path = manifest_path.resolve(strict=True)
    manifest_bytes = manifest_path.read_bytes()
    try:
        manifest = json.loads(manifest_bytes)
    except json.JSONDecodeError as error:
        raise ValueError(f"input manifest is invalid: {manifest_path}") from error
    _validate_selection_contract(manifest, expected_n)
    loaded = TABLE2.load_partnet_mobility_cohort(
        source_root.resolve(strict=True),
        manifest_path,
        formal=formal,
    )
    if len(loaded["assets"]) != expected_n:
        raise ValueError(f"expected exactly {expected_n} loaded PartNet-Mobility assets")

    categories, category_mapping_sha256 = _release_categories(
        source_root.resolve(strict=True)
    )
    release_category_count = len(set(categories.values()))
    _validate_release_category_count(release_category_count, formal=formal)
    assets: list[dict[str, Any]] = []
    for asset in loaded["assets"]:
        dataset_id = str(asset["dataset_id"])
        package = Path(asset["package"])
        if categories.get(dataset_id) != asset["category"]:
            raise ValueError(f"category metadata mismatch: {dataset_id}")
        source_item = asset["source_item"]
        if source_item.get("package_audit_success") is not True:
            raise ValueError(f"frozen package audit was not successful: {dataset_id}")
        if source_item.get("missing_core_files") not in (None, []):
            raise ValueError(f"frozen package has missing core files: {dataset_id}")
        if source_item.get("selection_digest") != TABLE4.selection_digest(dataset_id):
            raise ValueError(f"selection digest mismatch: {dataset_id}")
        identity = {
            "asset_id": dataset_id,
            "dataset_id": dataset_id,
            "selection_index": int(asset["selection_index"]),
            "raw_category": str(asset["category"]),
            "package": str(package),
            "primary_urdf_relative_path": "mobility.urdf",
            "primary_urdf_sha256": str(source_item["urdf_sha256"]),
            "frozen_item_sha256": str(asset["frozen_item_sha256"]),
            "input_identity_sha256": source_item.get("input_identity_sha256"),
            "source_item": source_item,
        }
        _validate_frozen_asset(identity)
        identity["package_binding"] = TABLE2.package_binding(package)
        assets.append(identity)

    return {
        "manifest": manifest,
        "manifest_file_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
        "source_root": loaded["source_root"],
        "release_asset_count": loaded["release_asset_count"],
        "release_category_count": release_category_count,
        "eval_category_count": len({row["raw_category"] for row in assets}),
        "category_mapping_sha256": category_mapping_sha256,
        "items_sha256": loaded["items_sha256"],
        "ordered_selected_ids_sha256": loaded["ordered_selected_ids_sha256"],
        "candidate_pool_identity_sha256": loaded[
            "candidate_pool_identity_sha256"
        ],
        "archive": loaded["archive"],
        "inventory": loaded["inventory"],
        "inventory_sha256": loaded["inventory_sha256"],
        "assets": assets,
    }


def _record_template(identity: dict[str, Any]) -> dict[str, Any]:
    return {
        "asset_id": identity["asset_id"],
        "dataset_id": identity["dataset_id"],
        "selection_index": identity["selection_index"],
        "raw_category": identity["raw_category"],
        "package": identity["package"],
        "primary_urdf_relative_path": identity["primary_urdf_relative_path"],
        "primary_urdf_sha256": identity["primary_urdf_sha256"],
        "package_content_manifest_sha256": identity["package_binding"][
            "content_manifest_sha256"
        ],
        "frozen_item_sha256": identity["frozen_item_sha256"],
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


def _fingerprint_observation(urdf_path: Path) -> dict[str, Any]:
    try:
        return {"result": SHARED.fingerprint_package(urdf_path), "error": None}
    except Exception as error:
        return {
            "result": None,
            "error": f"{type(error).__name__}: {error}",
        }


def evaluate_package(identity: dict[str, Any]) -> dict[str, Any]:
    record = _record_template(identity)
    urdf_path = Path(identity["package"]) / identity["primary_urdf_relative_path"]
    try:
        _validate_asset(identity)
        record.update(SHARED.analyze_urdf(urdf_path))
        record["parse_success"] = True
    except (ET.ParseError, OSError, ValueError, RuntimeError) as error:
        failed = _record_template(identity)
        failed["status"] = "EVALUATION_FAILED"
        failed["error"] = f"{type(error).__name__}: {error}"
        return failed

    first = _fingerprint_observation(urdf_path)
    try:
        _validate_asset(identity)
        second = _fingerprint_observation(urdf_path)
        if second != first:
            raise ValueError("simulation-resource closure changed during evaluation")
        _validate_asset(identity)
    except (OSError, ValueError, RuntimeError) as error:
        failed = _record_template(identity)
        failed["status"] = "EVALUATION_FAILED"
        failed["error"] = f"{type(error).__name__}: {error}"
        return failed

    if first["error"] is not None:
        record["status"] = "EVALUATED_FINGERPRINT_INCOMPLETE"
        record["error"] = first["error"]
        return record

    fingerprint = first["result"]
    record["fingerprint_complete"] = bool(fingerprint["complete"])
    record["package_fingerprint"] = fingerprint["fingerprint"]
    record["referenced_resource_count"] = fingerprint["resource_count"]
    record["missing_resources"] = fingerprint["missing_resources"]
    if record["fingerprint_complete"]:
        record["status"] = "EVALUATED"
    else:
        record["status"] = "EVALUATED_FINGERPRINT_INCOMPLETE"
        record["error"] = "one or more referenced simulation resources are unavailable"
    return record


def _evaluate_package_fail_closed(identity: dict[str, Any]) -> dict[str, Any]:
    try:
        return evaluate_package(identity)
    except Exception as error:
        record = _record_template(identity)
        record["status"] = "EVALUATION_FAILED"
        record["error"] = f"{type(error).__name__}: {error}"
        return record


def aggregate_partnet_records(
    records: list[dict[str, Any]],
    *,
    release_asset_count: int,
    release_category_count: int,
) -> dict[str, Any]:
    summary = SHARED.aggregate_records(
        records,
        release_asset_count=release_asset_count,
        release_category_count=release_category_count,
    )
    summary["cohort"][
        "cohort_type"
    ] = "FROZEN_HASH_RANKED_SAMPLE_NOT_CATEGORY_BALANCED"
    return summary


def _percentage(value: float | None) -> str:
    return "N/E" if value is None else f"{100.0 * value:.2f}%"


def _report(summary: dict[str, Any], run_manifest: dict[str, Any]) -> str:
    cohort = summary["cohort"]
    links = summary["links_per_asset"]
    movable = summary["movable_joints_per_asset"]
    multi = summary["multi_joint_assets"]
    topology = summary["unique_topologies"]
    duplicate = summary["exact_duplicate_rate"]
    return "\n".join(
        (
            "# PartNet-Mobility Table 1",
            "",
            "| Dataset | N_release | N_eval | Categories (release / eval) | Links/Asset (mean / median / P90) | Movable Joints/Asset (mean / median / P90) | Multi-joint Assets | Unique Topologies | Exact Duplicate Rate |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
            (
                f"| PartNet-Mobility | {cohort['N_release']:,} | {cohort['N_eval']} | "
                f"{cohort['release_raw_categories']} / {cohort['eval_raw_categories']} | "
                f"{links['mean']:.2f} / {links['median']:g} / {links['p90_nearest_rank']} | "
                f"{movable['mean']:.2f} / {movable['median']:g} / {movable['p90_nearest_rank']} | "
                f"{_percentage(multi['rate'])} | "
                f"{_percentage(topology['rate'])} (n={topology['denominator']}) | "
                f"{_percentage(duplicate['rate'])} (n={duplicate['denominator']}) |"
            ),
            "",
            f"Frozen manifest: `{run_manifest['input_manifest_file_sha256']}`.",
            "",
        )
    )


def _run_to_output(args: argparse.Namespace, output: Path) -> dict[str, Any]:
    dataset_root = args.dataset_root.resolve(strict=True)
    source_root = (dataset_root / "data/dataset").resolve(strict=True)
    cohort = load_frozen_cohort(
        args.input_manifest,
        source_root=source_root,
        expected_n=args.expected_n,
        formal=True,
    )
    assets = cohort["assets"]
    source_inventory = cohort["inventory"] or {}
    inventory_source = source_inventory.get("source", {})
    run_manifest = {
        "schema_version": 1,
        "dataset": "PartNet-Mobility",
        "release_status": source_inventory.get(
            "status", "LOCAL_COMPLETE_PROVENANCE_LIMITED"
        ),
        "paper_reported_assets": 2346,
        "paper_reported_categories": 46,
        "N_release": cohort["release_asset_count"],
        "release_raw_category_count": cohort["release_category_count"],
        "N_eval": len(assets),
        "eval_raw_category_count": cohort["eval_category_count"],
        "cohort_type": "FROZEN_HASH_RANKED_SAMPLE_NOT_CATEGORY_BALANCED",
        "selection_policy": cohort["manifest"]["selection_policy"],
        "input_manifest": str(args.input_manifest.resolve()),
        "input_manifest_file_sha256": cohort["manifest_file_sha256"],
        "items_sha256": cohort["items_sha256"],
        "ordered_selected_ids_sha256": cohort["ordered_selected_ids_sha256"],
        "candidate_pool_identity_sha256": cohort[
            "candidate_pool_identity_sha256"
        ],
        "dataset_root": str(dataset_root),
        "source_root": cohort["source_root"],
        "source_repo_id": inventory_source.get("repo_id"),
        "source_revision": inventory_source.get("revision"),
        "source_bytes_authenticated_against_revision": (
            source_inventory.get("verification", {}).get(
                "local_bytes_authenticated_against_hf_revision"
            )
        ),
        "inventory_sha256": cohort["inventory_sha256"],
        "archive": cohort["archive"],
        "category_mapping_policy": "exact dataset_id join to meta.json model_cat",
        "category_mapping_sha256": cohort["category_mapping_sha256"],
        "protocol": str(args.protocol.resolve()),
        "protocol_sha256": SHARED.sha256_file(args.protocol.resolve()),
        "runner": str(Path(__file__).resolve()),
        "runner_sha256": SHARED.sha256_file(Path(__file__).resolve()),
        "shared_metric_runner": str(Path(SHARED.__file__).resolve()),
        "shared_metric_runner_sha256": SHARED.sha256_file(Path(SHARED.__file__).resolve()),
        "frozen_cohort_loader": str(Path(TABLE2.__file__).resolve()),
        "frozen_cohort_loader_sha256": SHARED.sha256_file(Path(TABLE2.__file__).resolve()),
        "frozen_contract_runner": str(Path(TABLE4.__file__).resolve()),
        "frozen_contract_runner_sha256": SHARED.sha256_file(Path(TABLE4.__file__).resolve()),
        "topology_protocol": SHARED.TOPOLOGY_PROTOCOL,
        "fingerprint_protocol": SHARED.FINGERPRINT_PROTOCOL,
        "workers": args.workers,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "assets": [
            {
                key: row[key]
                for key in (
                    "selection_index",
                    "asset_id",
                    "raw_category",
                    "package",
                    "primary_urdf_relative_path",
                    "primary_urdf_sha256",
                    "frozen_item_sha256",
                    "input_identity_sha256",
                    "package_binding",
                )
            }
            for row in assets
        ],
    }
    SHARED.write_json(output / "manifest.json", run_manifest)

    if args.workers == 1:
        records = [_evaluate_package_fail_closed(row) for row in assets]
    else:
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            records = list(executor.map(_evaluate_package_fail_closed, assets))
    SHARED.write_jsonl(output / "asset_records.jsonl", records)
    summary = aggregate_partnet_records(
        records,
        release_asset_count=cohort["release_asset_count"],
        release_category_count=cohort["release_category_count"],
    )
    summary["status_counts"] = dict(
        sorted(Counter(str(row["status"]) for row in records).items())
    )
    summary["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
    SHARED.write_json(output / "summary.json", summary)
    SHARED._atomic_write_text(output / "report.md", _report(summary, run_manifest))

    artifact_files = ("manifest.json", "asset_records.jsonl", "summary.json", "report.md")
    artifact_manifest = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "files": {
            name: {
                "bytes": (output / name).stat().st_size,
                "sha256": SHARED.sha256_file(output / name),
            }
            for name in artifact_files
        },
    }
    SHARED.write_json(output / "artifact_manifest.json", artifact_manifest)
    return summary


def run(args: argparse.Namespace) -> dict[str, Any]:
    output = args.output.absolute()
    with SHARED._output_lock(output):
        staging = output.parent / (
            f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}"
        )
        staging.mkdir(parents=True, exist_ok=False)
        try:
            summary = _run_to_output(args, staging)
            SHARED._verify_staged_artifacts(staging)
            SHARED._publish_staged_output(staging, output)
            return summary
        finally:
            if staging.exists():
                import shutil

                shutil.rmtree(staging)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--input-manifest", type=Path, default=DEFAULT_INPUT_MANIFEST)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--expected-n", type=int, default=800)
    parser.add_argument("--workers", type=int, default=4)
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    if args.expected_n <= 0 or args.workers <= 0:
        raise SystemExit("--expected-n and --workers must be positive")
    summary = run(args)
    print(
        json.dumps(
            {
                "state": "COMPLETE",
                "N_release": summary["cohort"]["N_release"],
                "N_eval": summary["cohort"]["N_eval"],
                "N_parse": summary["cohort"]["N_parse"],
                "output": str(args.output.absolute()),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
