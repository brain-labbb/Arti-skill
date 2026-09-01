#!/usr/bin/env python3
"""Evaluate the frozen supplementary Infinite Mobility cohort for Table 1."""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import platform
import shutil
import sys
import tempfile
from typing import Any
import uuid

from infinite_mobility_table123_common import (
    APPROVED_FACTORIES,
    APPROVED_RECOVERY_IDENTITIES,
    DEFAULT_OUTPUT as DEFAULT_COHORT,
    FORMAL_COUNT,
    FORMAL_SEEDS,
    canonical_sha256,
    manifest_self_hash,
    output_lock,
    publish_staged,
    regular_file,
    scan_package,
    sha256_file,
    verify_cohort_manifest,
)
from run_table1_artiverse import (
    FINGERPRINT_PROTOCOL,
    TOPOLOGY_PROTOCOL,
    aggregate_records,
    analyze_urdf,
    fingerprint_package,
)


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/table1_infinite_mobility"
PROTOCOL_PATH = REPO_ROOT / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
SHARED_EVALUATOR = SCRIPT.parent / "run_table1_artiverse.py"


def validate_contract(*, n_release: int, n_eval: int, limit: int | None, formal: bool) -> None:
    if formal and limit is not None:
        raise ValueError("formal Table 1 does not permit --limit")
    if formal and (n_release != FORMAL_COUNT or n_eval != FORMAL_COUNT):
        raise ValueError("formal Table 1 cohort must contain exactly 720 assets")


def validate_formal_manifest(manifest: dict[str, Any]) -> None:
    if manifest.get("N_release") != FORMAL_COUNT or manifest.get("N_eval") != FORMAL_COUNT:
        raise ValueError("formal cohort declarations must contain 720")
    if tuple(manifest.get("factory_order", [])) != APPROVED_FACTORIES or tuple(manifest.get("seeds", [])) != FORMAL_SEEDS:
        raise ValueError("formal cohort matrix is not the approved factory/seed matrix")
    rows = manifest.get("assets")
    if not isinstance(rows, list) or len(rows) != FORMAL_COUNT:
        raise ValueError("formal cohort must contain exactly 720 rows")
    expected = {f"{factory}/seed_{seed:03d}" for factory in APPROVED_FACTORIES for seed in FORMAL_SEEDS}
    if {row.get("asset_id") for row in rows if isinstance(row, dict)} != expected:
        raise ValueError("formal identity matrix mismatch")
    for row in rows:
        if not isinstance(row, dict) or row.get("asset_id") != f"{row.get('factory')}/seed_{int(row.get('seed', -1)):03d}" or row.get("raw_category") != row.get("factory"):
            raise ValueError("formal identity fields are inconsistent")
    timeout_rows = [row for row in rows if row.get("original_status") == "TIMEOUT"]
    if len(timeout_rows) != 7 or {row["asset_id"] for row in timeout_rows} != set(APPROVED_RECOVERY_IDENTITIES):
        raise ValueError("formal recovery identity matrix mismatch")
    required_provenance = {"original_record_path", "original_record_sha256", "recovery_record_path", "recovery_record_sha256"}
    if any(
        not row.get("recovery_used")
        or not isinstance(row.get("recovery_provenance"), dict)
        or not required_provenance.issubset(row["recovery_provenance"])
        for row in timeout_rows
    ):
        raise ValueError("formal recovery provenance is incomplete")
    if sum(row.get("original_status") == "PASS" for row in rows) != 713:
        raise ValueError("formal cohort must retain 713 primary PASS rows")
    for index, row in enumerate(rows, start=1):
        if row.get("selection_index") != index or row.get("source") != ("recovery" if row.get("recovery_used") else "primary"):
            raise ValueError("formal selection provenance is inconsistent")
        if not isinstance(row.get("declared_joint_count_hint"), int) or row["declared_joint_count_hint"] < 0:
            raise ValueError("formal declared joint hint is invalid")


def _load_manifest(path: Path, *, formal: bool = False) -> dict[str, Any]:
    manifest = verify_cohort_manifest(path, formal=formal)
    if formal:
        validate_formal_manifest(manifest)
    return manifest


def _failed(row: dict[str, Any], error: Exception) -> dict[str, Any]:
    return {
        **row, "status": "FAILED", "error": f"{type(error).__name__}: {error}", "parse_success": False,
        "link_count": 0, "joint_count": 0, "joint_type_counts": {}, "non_fixed_joint_count": 0,
        "valid_tree": False, "topology_hash": None, "fingerprint_complete": False,
        "package_fingerprint": None, "fingerprint_resource_count": 0, "fingerprint_missing_resources": [],
    }


def _verify_row(row: dict[str, Any]) -> tuple[Path, Path, dict[str, Any]]:
    package = Path(str(row["package_path"])).absolute()
    if package.is_symlink() or not package.is_dir():
        raise ValueError(f"invalid package for {row['asset_id']}")
    urdf_relpath = str(row["urdf_relpath"])
    urdf = regular_file(package / urdf_relpath, label="URDF")
    try:
        urdf.relative_to(package.resolve(strict=True))
    except ValueError as error:
        raise ValueError(f"URDF path escapes package for {row['asset_id']}") from error
    scanned = scan_package(package)
    urdf_item = scanned["files_by_path"].get(urdf_relpath)
    if urdf_item is None or urdf_item["sha256"] != row.get("primary_urdf_sha256"):
        raise ValueError(f"primary URDF hash drift for {row['asset_id']}")
    if scanned["package_binding"] != row.get("package_binding"):
        raise ValueError(f"package binding drift for {row['asset_id']}")
    if scanned["baseline_package_sha256"] != row.get("baseline_package_sha256"):
        raise ValueError(f"baseline package hash drift for {row['asset_id']}")
    return package, urdf, scanned


def _evaluate_row(row: dict[str, Any]) -> dict[str, Any]:
    try:
        _, urdf, _ = _verify_row(row)
        analysis = analyze_urdf(urdf)
        fingerprint = fingerprint_package(urdf)
        return {
            **row, "status": "COMPLETED", "error": None, "parse_success": True, **analysis,
            "fingerprint_complete": fingerprint["complete"], "package_fingerprint": fingerprint["fingerprint"],
            "fingerprint_resource_count": fingerprint["resource_count"], "fingerprint_missing_resources": fingerprint["missing_resources"],
        }
    except Exception as error:
        return _failed(row, error)


def evaluate_cohort(manifest_path: Path, *, workers: int = 4, limit: int | None = None) -> list[dict[str, Any]]:
    manifest = _load_manifest(manifest_path)
    rows = list(manifest["assets"])
    if limit is not None:
        if limit <= 0:
            raise ValueError("--limit must be positive")
        rows = rows[:limit]
    if workers <= 0:
        raise ValueError("workers must be positive")
    if workers == 1:
        records = [_evaluate_row(row) for row in rows]
    else:
        with ThreadPoolExecutor(max_workers=workers) as executor:
            records = list(executor.map(_evaluate_row, rows))
    if len(records) != len(rows):
        raise RuntimeError("evaluation lost frozen assets")
    return records


def _write_json(path: Path, value: Any) -> None:
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    candidate = Path(temporary)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True, ensure_ascii=True)
            handle.write("\n"); handle.flush(); os.fsync(handle.fileno())
        candidate.replace(path)
    except BaseException:
        candidate.unlink(missing_ok=True); raise


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    candidate = Path(temporary)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n")
            handle.flush(); os.fsync(handle.fileno())
        candidate.replace(path)
    except BaseException:
        candidate.unlink(missing_ok=True); raise


def _protocol_bytes() -> bytes:
    return regular_file(PROTOCOL_PATH, label="evaluation protocol").read_bytes()


def _run_manifest(source: dict[str, Any], cohort_path: Path, protocol_bytes: bytes, *, workers: int, limit: int | None, formal: bool, n_eval: int) -> dict[str, Any]:
    evaluator = {
        "runner_path": str(SCRIPT), "runner_sha256": sha256_file(SCRIPT),
        "shared_evaluator_path": str(SHARED_EVALUATOR), "shared_evaluator_sha256": sha256_file(SHARED_EVALUATOR),
        "topology_protocol": TOPOLOGY_PROTOCOL, "fingerprint_protocol": FINGERPRINT_PROTOCOL,
        "python_executable": sys.executable, "python_version": sys.version, "platform": platform.platform(),
    }
    return {
        "schema_version": 2, "dataset": "Infinite Mobility", "release_status": "SUPPLEMENTARY_FULL_GENERATED_COHORT",
        "cohort_type": "SUPPLEMENTARY_FULL_GENERATED_COHORT_NOT_OFFICIAL_FINITE_RELEASE",
        "N_release": int(source["N_release"]), "N_eval": n_eval, "formal": formal, "limit": limit, "workers": workers,
        "cohort_manifest": str(cohort_path.resolve()), "cohort_manifest_sha256": sha256_file(cohort_path),
        "cohort_manifest_content_sha256": source["manifest_content_sha256"], "source_bindings": source.get("source_bindings", []),
        "protocol_source_path": str(PROTOCOL_PATH), "protocol_source_sha256_at_freeze": hashlib_sha256(protocol_bytes),
        "protocol_snapshot_sha256": hashlib_sha256(protocol_bytes), "evaluator_binding": evaluator,
    }


def hashlib_sha256(value: bytes) -> str:
    import hashlib
    return hashlib.sha256(value).hexdigest()


def _render_report(summary: dict[str, Any], manifest: dict[str, Any], source: dict[str, Any]) -> str:
    cohort = summary["cohort"]
    links = summary["links_per_asset"]
    movable = summary["movable_joints_per_asset"]
    multi = summary["multi_joint_assets"]
    topology = summary["unique_topologies"]
    duplicate = summary["exact_duplicate_rate"]
    macro = summary["category_macro"]

    def number(value: Any, decimals: int = 3) -> str:
        return "N/E" if value is None else f"{value:.{decimals}f}" if isinstance(value, float) else str(value)

    def rate(value: float | None) -> str:
        return "N/E" if value is None else f"{100.0 * value:.2f}%"

    return "\n".join((
        "# Infinite Mobility Table 1: Dataset Scale and Structural Diversity", "",
        "## Frozen cohort", "",
        "- This is a supplementary full generated cohort, not an official finite release.",
        "- Identity policy: all obtained factory/seed identities are retained; recovery is an immutable pre-freeze overlay for original TIMEOUT cases; no post-freeze reselection.",
        f"- `N_release`: {cohort['N_release']}; `N_eval`: {cohort['N_eval']}; raw factories: {cohort['release_raw_categories']} / {cohort['eval_raw_categories']}.",
        f"- Cohort manifest SHA-256: `{manifest['cohort_manifest_sha256']}`.", "",
        "## Table 1 result", "",
        "| Dataset | Cohort | N_release | N_eval | Factories (release / eval) | Links/Asset (mean / median / P90) | Movable Joints/Asset (mean / median / P90) | Multi-joint Assets | Unique Topologies | Exact Duplicate Rate |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| Infinite Mobility | supplementary full generated | {cohort['N_release']} | {cohort['N_eval']} | {cohort['release_raw_categories']} / {cohort['eval_raw_categories']} | {number(links['mean'])} / {number(links['median'])} / {number(links['p90_nearest_rank'])} (n={links['denominator']}) | {number(movable['mean'])} / {number(movable['median'])} / {number(movable['p90_nearest_rank'])} (n={movable['denominator']}) | {multi['numerator']} / {multi['denominator']} ({rate(multi['rate'])}) | {topology['unique']} / {topology['denominator']} ({rate(topology['rate'])}); coverage {topology['denominator']} / {topology['coverage_denominator']} ({rate(topology['coverage_rate'])}) | {duplicate['duplicate_excess']} / {duplicate['denominator']} ({rate(duplicate['rate'])}); coverage {duplicate['denominator']} / {duplicate['coverage_denominator']} ({rate(duplicate['coverage_rate'])}) |",
        "", "## Diagnostics", "",
        f"- XML parse coverage: {cohort['N_parse']} / {cohort['N_eval']} ({rate(cohort['N_parse'] / cohort['N_eval'] if cohort['N_eval'] else None)}).",
        f"- Declared joint types: `{json.dumps(summary['declared_joint_type_counts'], sort_keys=True)}`.",
        f"- Category macro across {macro['category_count']} factories: multi-joint {rate(macro['multi_joint_assets_rate'])}; unique topology {rate(macro['unique_topologies_rate'])}; duplicate {rate(macro['exact_duplicate_rate'])}.",
        f"- Status counts: `{json.dumps(summary['status_counts'], sort_keys=True)}`.",
        "", "## Provenance", "",
        f"- Original primary PASS rows: {sum(row.get('original_status') == 'PASS' for row in source['assets'])}.",
        f"- Original TIMEOUT recovery overlays: {sum(row.get('recovery_used') for row in source['assets'])}.",
        "- Topology and duplicate rates retain their shared evaluator denominators and coverage definitions; failed assets remain in `N_eval`.", "",
    ))


def run(*, cohort_manifest: Path, output: Path, workers: int, limit: int | None, formal: bool) -> dict[str, Any]:
    with output_lock(output):
        source = _load_manifest(cohort_manifest, formal=formal)
        rows = list(source["assets"]); selected = rows if limit is None else rows[:limit]
        validate_contract(n_release=int(source["N_release"]), n_eval=len(selected), limit=limit, formal=formal)
        protocol_bytes = _protocol_bytes()
        records = evaluate_cohort(cohort_manifest, workers=workers, limit=limit)
        summary = aggregate_records(records, release_asset_count=int(source["N_release"]), release_category_count=len(source["factory_order"]))
        summary["status_counts"] = dict(sorted(Counter(row["status"] for row in records).items()))
        summary["supplementary_cohort"] = True
        manifest = _run_manifest(source, cohort_manifest, protocol_bytes, workers=workers, limit=limit, formal=formal, n_eval=len(records))
        manifest["manifest_content_sha256"] = manifest_self_hash(manifest)
        staging = output.parent / f".{output.name}.staging.{uuid.uuid4().hex}"; staging.mkdir()
        try:
            _write_json(staging / "manifest.json", manifest)
            _write_jsonl(staging / "asset_records.jsonl", records)
            _write_json(staging / "summary.json", summary)
            snapshot = staging / "protocol_snapshot.md"
            snapshot.write_bytes(protocol_bytes)
            if snapshot.read_bytes() != protocol_bytes or sha256_file(snapshot) != manifest["protocol_snapshot_sha256"]:
                raise RuntimeError("protocol snapshot verification failed")
            (staging / "report.md").write_text(_render_report(summary, manifest, source), encoding="utf-8")
            names = ("manifest.json", "asset_records.jsonl", "summary.json", "report.md", "protocol_snapshot.md")
            _write_json(staging / "artifact_manifest.json", {"schema_version": 1, "files": {name: {"bytes": (staging / name).stat().st_size, "sha256": sha256_file(staging / name)} for name in names}})
            publish_staged(staging, output)
        except BaseException:
            shutil.rmtree(staging, ignore_errors=True); raise
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cohort-manifest", type=Path, default=DEFAULT_COHORT / "manifest.json")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT); parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--limit", type=int); parser.add_argument("--formal", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    summary = run(cohort_manifest=args.cohort_manifest, output=args.output, workers=args.workers, limit=args.limit, formal=args.formal)
    print(json.dumps({"state": "COMPLETE", "N_eval": summary["cohort"]["N_eval"], "output": str(args.output.absolute())}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
