#!/usr/bin/env python3
"""Evaluate Articraft-10K Table 1 on an existing frozen package manifest."""

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
import subprocess
import sys
from typing import Any
import uuid
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE_ROOT = REPO / "exp/Articraft-10K/released_urdf"
DEFAULT_INPUT_MANIFEST = (
    REPO
    / "exp/runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json"
)
DEFAULT_CATEGORY_RECORDS_ROOT = REPO / "exp/baselines/Articraft-10K-official/records"
DEFAULT_OUTPUT = REPO / "exp/runtime/table1_articraft"
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"


def _load_shared_runner():
    path = Path(__file__).with_name("run_table1_artiverse.py")
    name = "table1_structural_metrics_shared"
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load shared Table 1 metrics: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


SHARED = _load_shared_runner()


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return _canonical_sha256(payload)


def _package_file_manifest(package: Path) -> list[dict[str, Any]]:
    try:
        package = package.resolve(strict=True)
    except OSError as error:
        raise ValueError(f"package is unavailable: {package}") from error
    if not package.is_dir():
        raise ValueError(f"package is not a directory: {package}")
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if child.is_symlink():
                raise ValueError(
                    f"package contains directory symlink: {child.relative_to(package)}"
                )
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            try:
                canonical = path.resolve(strict=True)
                canonical.relative_to(package)
            except (OSError, ValueError) as error:
                raise ValueError(f"package file escapes package: {relative}") from error
            if not canonical.is_file():
                raise ValueError(f"package entry is not a regular file: {relative}")
            rows.append(
                {
                    "path": relative,
                    "bytes": canonical.stat().st_size,
                    "sha256": SHARED.sha256_file(canonical),
                }
            )
    return rows


def _package_binding(package: Path) -> dict[str, Any]:
    files = _package_file_manifest(package)
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": _canonical_sha256(files),
    }


def _category_mapping(
    asset_ids: list[str],
    category_records_root: Path,
) -> tuple[dict[str, str], str]:
    categories: dict[str, str] = {}
    rows: list[dict[str, str]] = []
    for asset_id in asset_ids:
        path = category_records_root / asset_id / "record.json"
        try:
            record = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f"category record is unavailable or invalid: {asset_id}") from error
        if record.get("record_id") != asset_id:
            raise ValueError(f"category record identity mismatch: {asset_id}")
        category = str(record.get("category_slug", "")).strip()
        if not category:
            raise ValueError(f"category_slug is missing: {asset_id}")
        categories[asset_id] = category
        rows.append({"asset_id": asset_id, "category_slug": category})
    return categories, _canonical_sha256(rows)


def load_frozen_cohort(
    manifest_path: Path,
    *,
    source_root: Path,
    category_records_root: Path,
    expected_n: int,
) -> dict[str, Any]:
    try:
        manifest_bytes = manifest_path.read_bytes()
        manifest = json.loads(manifest_bytes)
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"input manifest is unavailable or invalid: {manifest_path}") from error
    if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
        raise ValueError("input manifest self-hash mismatch")
    if (
        manifest.get("dataset") != "Articraft-10K"
        or manifest.get("classification") != "FORMAL"
        or manifest.get("mode") != "formal"
    ):
        raise ValueError("input manifest is not a formal Articraft-10K cohort")

    source_root = source_root.resolve(strict=True)
    if Path(str(manifest.get("source", {}).get("root", ""))).resolve() != source_root:
        raise ValueError("input manifest source root does not match --source-root")
    release_entries = list(source_root.iterdir())
    symlink_children = sorted(path.name for path in release_entries if path.is_symlink())
    if symlink_children:
        raise ValueError(
            f"release root contains symlink child: {symlink_children[0]}"
        )
    release_ids = sorted(path.name for path in release_entries if path.is_dir())
    source = manifest.get("source", {})
    if len(release_ids) != source.get("release_asset_count"):
        raise ValueError("release asset count differs from the frozen manifest")
    if _canonical_sha256(release_ids) != source.get("release_asset_ids_sha256"):
        raise ValueError("release asset universe differs from the frozen manifest")

    records = manifest.get("records")
    selection = manifest.get("selection", {})
    if not isinstance(records, list) or len(records) != expected_n:
        raise ValueError(f"expected exactly {expected_n} frozen records")
    if selection.get("n_eval") != expected_n:
        raise ValueError("selection n_eval differs from the requested frozen size")
    asset_ids = [str(row.get("asset_id", "")) for row in records]
    if not all(asset_ids) or len(set(asset_ids)) != len(asset_ids):
        raise ValueError("frozen asset identities must be unique and nonempty")
    if _canonical_sha256(asset_ids) != selection.get("selected_asset_ids_sha256"):
        raise ValueError("selected asset identity hash mismatch")
    if selection.get("selection_order_preserved") is not True:
        raise ValueError("input manifest does not preserve selection order")
    if selection.get("outcome_based_reselection") is not False:
        raise ValueError("input manifest permits outcome-based reselection")

    categories, category_mapping_sha256 = _category_mapping(
        release_ids, category_records_root.resolve(strict=True)
    )
    assets: list[dict[str, Any]] = []
    for index, row in enumerate(records):
        asset_id = asset_ids[index]
        if row.get("selection_index") != index:
            raise ValueError(f"selection index mismatch: {asset_id}")
        try:
            package = Path(str(row.get("package", ""))).resolve(strict=True)
            relative = package.relative_to(source_root)
        except (OSError, ValueError) as error:
            raise ValueError(f"package escapes or is absent from source root: {asset_id}") from error
        if len(relative.parts) != 1 or package.name != asset_id:
            raise ValueError(f"package identity/path mismatch: {asset_id}")
        urdf_path = package / "model.urdf"
        if not urdf_path.is_file():
            raise ValueError(f"model.urdf is missing: {asset_id}")
        if SHARED.sha256_file(urdf_path) != row.get("model_urdf_sha256"):
            raise ValueError(f"model.urdf hash mismatch: {asset_id}")
        binding = _package_binding(package)
        if binding != row.get("package_binding"):
            raise ValueError(f"package binding mismatch: {asset_id}")
        assets.append(
            {
                "asset_id": asset_id,
                "selection_index": index,
                "raw_category": categories[asset_id],
                "package": str(package),
                "model_urdf_sha256": row["model_urdf_sha256"],
                "package_binding": row["package_binding"],
            }
        )

    return {
        "manifest": manifest,
        "manifest_file_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
        "release_asset_count": len(release_ids),
        "release_category_count": len(set(categories.values())),
        "eval_category_count": len({categories[asset_id] for asset_id in asset_ids}),
        "category_mapping_sha256": category_mapping_sha256,
        "assets": assets,
    }


def _record_template(identity: dict[str, Any]) -> dict[str, Any]:
    return {
        "asset_id": identity["asset_id"],
        "selection_index": identity["selection_index"],
        "raw_category": identity["raw_category"],
        "package": identity["package"],
        "model_urdf_sha256": identity["model_urdf_sha256"],
        "package_content_manifest_sha256": identity["package_binding"][
            "content_manifest_sha256"
        ],
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


def evaluate_package(identity: dict[str, Any]) -> dict[str, Any]:
    record = _record_template(identity)
    package = Path(identity["package"])
    urdf_path = package / "model.urdf"
    try:
        if SHARED.sha256_file(urdf_path) != identity["model_urdf_sha256"]:
            raise ValueError("model.urdf changed after cohort freeze")
        analysis = SHARED.analyze_urdf(urdf_path)
        record.update(analysis)
        record["parse_success"] = True
    except (ET.ParseError, OSError, ValueError) as error:
        failed = _record_template(identity)
        failed["status"] = "EVALUATION_FAILED"
        failed["error"] = f"{type(error).__name__}: {error}"
        return failed

    try:
        fingerprint = SHARED.fingerprint_package(urdf_path)
        record["fingerprint_complete"] = bool(fingerprint["complete"])
        record["package_fingerprint"] = fingerprint["fingerprint"]
        record["referenced_resource_count"] = fingerprint["resource_count"]
        record["missing_resources"] = fingerprint["missing_resources"]
    except Exception as error:
        record["error"] = f"{type(error).__name__}: {error}"

    try:
        if _package_binding(package) != identity["package_binding"]:
            raise ValueError("package changed during evaluation")
    except (OSError, ValueError) as error:
        failed = _record_template(identity)
        failed["status"] = "EVALUATION_FAILED"
        failed["error"] = f"{type(error).__name__}: {error}"
        return failed
    if record["fingerprint_complete"]:
        record["status"] = "EVALUATED"
    else:
        record["status"] = "EVALUATED_FINGERPRINT_INCOMPLETE"
        if record["error"] is None:
            record["error"] = "one or more referenced package resources are unavailable"
    return record


def _evaluate_package_fail_closed(identity: dict[str, Any]) -> dict[str, Any]:
    try:
        return evaluate_package(identity)
    except Exception as error:
        record = _record_template(identity)
        record["status"] = "EVALUATION_FAILED"
        record["error"] = f"{type(error).__name__}: {error}"
        return record


def aggregate_articraft_records(
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
    summary["cohort"]["cohort_type"] = "FROZEN_RANDOM_SAMPLE_NOT_CATEGORY_BALANCED"
    return summary


def _git_revision(path: Path) -> str | None:
    revision = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        check=False,
        capture_output=True,
        text=True,
    )
    if revision.returncode != 0:
        return None
    status = subprocess.run(
        ["git", "-C", str(path), "status", "--porcelain=v1", "--untracked-files=all"],
        check=False,
        capture_output=True,
        text=True,
    )
    if status.returncode != 0:
        raise ValueError(f"cannot inspect category checkout status: {path}")
    if status.stdout.strip():
        raise ValueError(f"category checkout has uncommitted changes: {path}")
    return revision.stdout.strip()


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
            "# Articraft-10K Table 1",
            "",
            "| Dataset | N_release | N_eval | Categories (release / eval) | Links/Asset (mean / median / P90) | Movable Joints/Asset (mean / median / P90) | Multi-joint Assets | Unique Topologies | Exact Duplicate Rate |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
            (
                f"| Articraft-10K | {cohort['N_release']:,} | {cohort['N_eval']} | "
                f"{cohort['release_raw_categories']} / {cohort['eval_raw_categories']} | "
                f"{links['mean']:.2f} / {links['median']:g} / {links['p90_nearest_rank']} | "
                f"{movable['mean']:.2f} / {movable['median']:g} / {movable['p90_nearest_rank']} | "
                f"{_percentage(multi['rate'])} | "
                f"{_percentage(topology['rate'])} (n={topology['denominator']}) | "
                f"{_percentage(duplicate['rate'])} (n={duplicate['denominator']}) |"
            ),
            "",
            (
                f"Frozen manifest: `{run_manifest['input_manifest_content_sha256']}`; "
                f"seed {run_manifest['seed']}."
            ),
            "",
        )
    )


def _run_to_output(args: argparse.Namespace, output: Path) -> dict[str, Any]:
    cohort = load_frozen_cohort(
        args.input_manifest.resolve(),
        source_root=args.source_root.resolve(),
        category_records_root=args.category_records_root.resolve(),
        expected_n=args.expected_n,
    )
    input_manifest = cohort["manifest"]
    started_at = datetime.now(timezone.utc).isoformat()
    assets = cohort["assets"]
    run_manifest = {
        "schema_version": 1,
        "dataset": "Articraft-10K",
        "release_status": "VERIFIED_COMPLETE",
        "paper_reported_assets": 10018,
        "paper_reported_categories": 245,
        "N_release": cohort["release_asset_count"],
        "release_raw_category_count": cohort["release_category_count"],
        "N_eval": len(assets),
        "eval_raw_category_count": cohort["eval_category_count"],
        "cohort_type": "FROZEN_RANDOM_SAMPLE_NOT_CATEGORY_BALANCED",
        "seed": input_manifest["selection"]["seed"],
        "selection_protocol": input_manifest["selection"]["algorithm"],
        "selected_asset_ids_sha256": input_manifest["selection"][
            "selected_asset_ids_sha256"
        ],
        "input_manifest": str(args.input_manifest.resolve()),
        "input_manifest_file_sha256": cohort["manifest_file_sha256"],
        "input_manifest_content_sha256": input_manifest["manifest_content_sha256"],
        "source_root": str(args.source_root.resolve()),
        "source_repo_id": input_manifest["source"].get("repo_id"),
        "source_revision": input_manifest["source"].get("revision"),
        "category_records_root": str(args.category_records_root.resolve()),
        "category_records_revision": _git_revision(args.category_records_root.resolve()),
        "category_mapping_policy": "exact asset_id join to official record.json category_slug",
        "category_mapping_sha256": cohort["category_mapping_sha256"],
        "protocol": str(args.protocol.resolve()),
        "protocol_sha256": SHARED.sha256_file(args.protocol.resolve()),
        "runner": str(Path(__file__).resolve()),
        "runner_sha256": SHARED.sha256_file(Path(__file__).resolve()),
        "shared_metric_runner": str(Path(SHARED.__file__).resolve()),
        "shared_metric_runner_sha256": SHARED.sha256_file(Path(SHARED.__file__).resolve()),
        "topology_protocol": SHARED.TOPOLOGY_PROTOCOL,
        "fingerprint_protocol": SHARED.FINGERPRINT_PROTOCOL,
        "workers": args.workers,
        "started_at_utc": started_at,
        "assets": [
            {
                key: row[key]
                for key in (
                    "selection_index",
                    "asset_id",
                    "raw_category",
                    "package",
                    "model_urdf_sha256",
                )
            }
            | {
                "package_content_manifest_sha256": row["package_binding"][
                    "content_manifest_sha256"
                ]
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
    summary = aggregate_articraft_records(
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
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--input-manifest", type=Path, default=DEFAULT_INPUT_MANIFEST)
    parser.add_argument(
        "--category-records-root", type=Path, default=DEFAULT_CATEGORY_RECORDS_ROOT
    )
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
