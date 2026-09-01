#!/usr/bin/env python3
"""Freeze and materialize the Table 5 Articraft-200 cohort from 10,787 records.

The source universe is the frozen Articraft GitHub merged roster.  Selection is
performed before compilation and never uses compile or simulator outcomes.  All
selected records are materialized through the official Articraft compiler in
full/validated mode, so the resulting URDFs carry both visual and collision
geometry.  Existing full/validated packages from the 10,787 merge are reused.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

from table5_n200_manifest import _parse_urdf
from table5_v2_sample_n200 import (
    DATASET_ORDER,
    DATASET_SLUGS,
    PVA_SLUG,
    PROTOCOL_ID,
    SAMPLE_SIZE,
    SELECTION_SEED,
    SEMANTIC_EXCLUSIONS,
    canonical_sha256,
    dataset_eligibility_reason,
    normalize_label,
    selection_rank,
    validate_manifest,
)


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
EXP_ROOT = REPO_ROOT / "exp"
DEFAULT_MERGED_MANIFEST = (
    EXP_ROOT
    / "runtime/articraft_github_merged_10787_20260827/rosters/merged/full_release_manifest.json"
)
DEFAULT_MERGED_ROSTER = DEFAULT_MERGED_MANIFEST.with_name("full_release_roster.jsonl")
DEFAULT_SOURCE_ROOT = EXP_ROOT / "Articraft-10K-github"
DEFAULT_BASE_COHORT = EXP_ROOT / "runtime/table5_v2_core200_seed20260828/manifest.json"
DEFAULT_OUTPUT = (
    EXP_ROOT / "runtime/table5_v2_articraft_github_core200_visual_collision_official"
)
COMPILE_HARNESS = EXP_ROOT / "baselines/Articraft-paper-harness"
COMPILE_PYTHON = COMPILE_HARNESS / ".venv/bin/python"
COMPILE_SCRIPT = COMPILE_HARNESS / "cli/compile_record.py"
COMPAT_SCRIPT = SCRIPT_PATH.with_name("articraft_compile_compat.py")
COMPILE_TIMEOUT_SECONDS = 900.0


class ResampleError(RuntimeError):
    """Raised when the frozen Articraft-200 contract cannot be maintained."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write(path: Path, value: Any) -> None:
    path = path.resolve(strict=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(
        value, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False
    ) + "\n"
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_text(path: Path, value: str) -> None:
    path = path.resolve(strict=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(value)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ResampleError(f"cannot load JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise ResampleError(f"JSON root is not an object: {path}")
    return value


def _source_rows(manifest_path: Path) -> tuple[list[dict[str, Any]], str, str]:
    manifest = _load_json(manifest_path)
    rows = manifest.get("rows")
    if not isinstance(rows, list) or len(rows) != 10_787:
        raise ResampleError("merged Articraft manifest must contain N_eval=10,787 rows")
    roster_path = manifest_path.with_name("full_release_roster.jsonl")
    if not roster_path.is_file():
        raise ResampleError(f"merged Articraft roster is missing: {roster_path}")
    roster_hash = sha256_file(roster_path)
    if manifest.get("roster_jsonl_sha256") != roster_hash:
        raise ResampleError("merged Articraft roster SHA256 mismatch")
    return rows, roster_hash, str(manifest.get("manifest_content_sha256") or "")


def _candidate_reason(row: Mapping[str, Any]) -> str | None:
    category = str(row.get("category") or row.get("raw_category") or "N/E")
    asset_id = str(row.get("asset_id") or "")
    joint_count = row.get("joint_count")
    if isinstance(joint_count, bool) or not isinstance(joint_count, int):
        return "movable_joint_count_unavailable"
    return dataset_eligibility_reason(
        "articraft_10k", category, asset_id, joint_count
    )


def select_rows(
    rows: Sequence[Mapping[str, Any]], *, universe_hash: str
) -> tuple[list[dict[str, Any]], Counter[str], int]:
    ranked: list[tuple[str, str, Mapping[str, Any]]] = []
    excluded: Counter[str] = Counter()
    for raw in rows:
        reason = _candidate_reason(raw)
        asset_id = str(raw.get("asset_id") or "")
        rank = selection_rank(
            "articraft_10k", asset_id, universe_sha256=universe_hash
        )
        if reason is not None:
            excluded[reason] += 1
            continue
        ranked.append((rank, asset_id, raw))
    ranked.sort(key=lambda item: (item[0], item[1]))
    if len(ranked) < SAMPLE_SIZE:
        raise ResampleError(
            f"only {len(ranked)} eligible Articraft records; {SAMPLE_SIZE} required"
        )
    return [deepcopy(dict(item[2])) for item in ranked[:SAMPLE_SIZE]], excluded, len(ranked)


def _package_is_full_visual_collision(package: Path, asset_id: str) -> bool:
    urdf = package / "model.urdf"
    report = package / "compile_report.json"
    if not urdf.is_file() or urdf.is_symlink() or not report.is_file():
        return False
    try:
        payload = _load_json(report)
        metrics = payload.get("metrics")
        if not isinstance(metrics, Mapping):
            return False
        if not (
            payload.get("status") == "success"
            and payload.get("record_id") == asset_id
            and metrics.get("compile_level") == "full"
            and metrics.get("validation_level") == "full"
        ):
            return False
        root = ET.parse(urdf).getroot()
        visual = sum(len(link.findall("visual")) for link in root.findall("link"))
        collision = sum(len(link.findall("collision")) for link in root.findall("link"))
        return visual > 0 and collision > 0
    except (OSError, ET.ParseError, ValueError):
        return False


def _existing_package(raw: Mapping[str, Any], asset_id: str) -> Path | None:
    source_path = raw.get("source_path")
    if isinstance(source_path, str):
        package = Path(source_path)
        if _package_is_full_visual_collision(package, asset_id):
            return package.resolve(strict=True)
    return None


def _compile_one(
    *,
    asset_id: str,
    source_root: Path,
    staging_data: Path,
    timeout_seconds: float,
) -> dict[str, Any]:
    package = staging_data / "cache/record_materialization" / asset_id
    if _package_is_full_visual_collision(package, asset_id):
        return {"asset_id": asset_id, "status": "success", "origin": "resume"}
    source = (source_root / "records" / asset_id).resolve(strict=True)
    records = staging_data / "records"
    records.mkdir(parents=True, exist_ok=True)
    link = records / asset_id
    if link.exists() or link.is_symlink():
        if not link.is_symlink() or link.resolve(strict=True) != source:
            raise ResampleError(f"staging source binding mismatch: {link}")
    else:
        link.symlink_to(source, target_is_directory=True)
    # Articraft's official default is full+validate with geometry-QC findings
    # downgraded to warnings.  Table 5 must preserve that released export
    # behavior; strict geometry QC is not an extra selection gate here.
    command = [
        str(COMPILE_PYTHON),
        str(COMPILE_SCRIPT),
        asset_id,
        "--repo-root",
        str(COMPILE_HARNESS),
        "--data-dir",
        str(staging_data),
        "--target",
        "full",
        "--validate",
    ]
    environment = os.environ.copy()
    environment.update(
        {
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        }
    )
    completed = subprocess.run(
        command,
        cwd=COMPILE_HARNESS,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout_seconds,
        check=False,
    )
    log_root = staging_data.parent.parent / "compile_logs"
    atomic_text(log_root / f"{asset_id}.stdout.txt", completed.stdout)
    atomic_text(log_root / f"{asset_id}.stderr.txt", completed.stderr)
    if _package_is_full_visual_collision(package, asset_id):
        return {
            "asset_id": asset_id,
            "status": "success",
            "origin": "executed",
            "returncode": completed.returncode,
            "compile_policy": "full_validate_official_nonblocking_geom_qc",
        }
    # A few GitHub records call legacy warning helpers with signatures that are
    # incompatible with the checked-out SDK.  The wrapper only aliases those
    # APIs and still invokes the official non-blocking export path; it does not
    # repair or filter geometry.
    fallback_command = [
        str(COMPILE_PYTHON),
        str(COMPAT_SCRIPT),
        asset_id,
        "--repo-root",
        str(COMPILE_HARNESS),
        "--data-dir",
        str(staging_data),
        "--target",
        "full",
        "--validate",
    ]
    fallback = subprocess.run(
        fallback_command,
        cwd=COMPILE_HARNESS,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout_seconds,
        check=False,
    )
    atomic_text(log_root / f"{asset_id}.fallback.stdout.txt", fallback.stdout)
    atomic_text(log_root / f"{asset_id}.fallback.stderr.txt", fallback.stderr)
    if _package_is_full_visual_collision(package, asset_id):
        return {
            "asset_id": asset_id,
            "status": "success",
            "origin": "compatibility_fallback",
            "returncode": fallback.returncode,
            "compile_policy": "full_validate_compat_nonblocking_geom_qc",
        }
    return {
        "asset_id": asset_id,
        "status": "failed",
        "origin": "executed",
        "returncode": fallback.returncode,
        "reason": "official and compatibility export did not produce full visual+collision URDF",
    }


def compile_selected(
    selected: Sequence[Mapping[str, Any]],
    *,
    output: Path,
    source_root: Path,
    workers: int,
    timeout_seconds: float,
) -> dict[str, Path]:
    staging_data = output / "staging/data"
    staging_data.mkdir(parents=True, exist_ok=True)
    packages: dict[str, Path] = {}
    jobs: list[tuple[str, Path | None]] = []
    for raw in selected:
        asset_id = str(raw["asset_id"])
        existing = _existing_package(raw, asset_id)
        if existing is not None:
            packages[asset_id] = existing
        else:
            jobs.append((asset_id, None))
    statuses: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(
                _compile_one,
                asset_id=asset_id,
                source_root=source_root,
                staging_data=staging_data,
                timeout_seconds=timeout_seconds,
            ): asset_id
            for asset_id, _ in jobs
        }
        for future in as_completed(futures):
            asset_id = futures[future]
            try:
                status = future.result()
            except Exception as error:  # preserve the asset in the denominator
                status = {
                    "asset_id": asset_id,
                    "status": "failed",
                    "origin": "executed",
                    "reason": f"{type(error).__name__}: {error}",
                }
            statuses.append(status)
            if status["status"] == "success":
                packages[asset_id] = (
                    staging_data / "cache/record_materialization" / asset_id
                ).resolve(strict=True)
    for raw in selected:
        asset_id = str(raw["asset_id"])
        if asset_id in packages:
            continue
        package = staging_data / "cache/record_materialization" / asset_id
        package.mkdir(parents=True, exist_ok=True)
        placeholder = f"ARTICRAFT_TABLE5_COMPILE_FAILED record_id={asset_id}\n"
        atomic_text(package / "model.urdf", placeholder)
        if not (package / "compile_report.json").is_file():
            atomic_write(
                package / "compile_report.json",
                {
                    "schema_version": 1,
                    "record_id": asset_id,
                    "status": "failure",
                    "checks_run": ["compile_urdf"],
                    "metrics": {"compile_level": "full", "validation_level": "full"},
                },
            )
        packages[asset_id] = package.resolve(strict=True)
    atomic_write(
        output / "compile_status.json",
        {
            "schema_version": "table5_v2_articraft_compile_status_v1",
            "selected_count": len(selected),
            "reused_count": len(selected) - len(jobs),
            "executed_count": len(jobs),
            "success_count": sum(_package_is_full_visual_collision(p, aid) for aid, p in packages.items()),
            "failure_count": sum(not _package_is_full_visual_collision(p, aid) for aid, p in packages.items()),
            "records": sorted(statuses, key=lambda row: str(row["asset_id"])),
        },
    )
    return packages


def _raw_compiled_row(
    source: Mapping[str, Any],
    *,
    package: Path,
    order: int,
    rank: str,
    universe_hash: str,
    merged_manifest_hash: str,
) -> dict[str, Any]:
    asset_id = str(source["asset_id"])
    urdf = package / "model.urdf"
    parsed = _parse_urdf(package, urdf)
    joint_count = int(source["joint_count"])
    counts = parsed.get("xml_counts")
    if not isinstance(counts, Mapping):
        counts = {"movable_joints": joint_count}
    else:
        counts = dict(counts)
        counts.setdefault("movable_joints", joint_count)
    source_row_hash = canonical_sha256(source)
    raw: dict[str, Any] = {
        "dataset_slug": "articraft_10k",
        "dataset_name": "Articraft-10K",
        "dataset_id": f"articraft_{order:04d}",
        "asset_id": asset_id,
        "category": str(source.get("category") or source.get("raw_category") or "N/E"),
        "package_root": str(package),
        "urdf_path": str(urdf),
        "urdf_sha256": sha256_file(urdf),
        "xml_counts": counts,
        "source_provenance": {
            "cohort_origin": source.get("cohort_origin"),
            "github_record_path": str(DEFAULT_SOURCE_ROOT / "records" / asset_id),
            "merged_manifest_sha256": merged_manifest_hash,
            "merged_manifest_row_sha256": source_row_hash,
            "universe_roster_sha256": universe_hash,
        },
        "cohort_selection": {
            "protocol_id": PROTOCOL_ID,
            "seed": SELECTION_SEED,
            "selection_order": order,
            "rank_sha256": rank,
            "source_row_sha256": source_row_hash,
        },
    }
    raw["row_sha256"] = canonical_sha256(raw, exclude_fields=("row_sha256",))
    return raw


def build_cohort(
    *,
    selected: Sequence[Mapping[str, Any]],
    packages: Mapping[str, Path],
    excluded: Counter[str],
    eligible_count: int,
    universe_path: Path,
    universe_hash: str,
    merged_manifest_path: Path,
    merged_manifest_hash: str,
    base_cohort_path: Path,
    output: Path,
) -> Path:
    base = _load_json(base_cohort_path)
    datasets = deepcopy(base.get("datasets"))
    if not isinstance(datasets, list) or [d.get("dataset_slug") for d in datasets] != list(DATASET_SLUGS):
        raise ResampleError("base Table 5 v2 cohort does not have the expected seven datasets")
    art_index = next(i for i, d in enumerate(datasets) if d.get("dataset_slug") == "articraft_10k")
    art_rows: list[dict[str, Any]] = []
    for order, source in enumerate(selected):
        asset_id = str(source["asset_id"])
        rank = selection_rank("articraft_10k", asset_id, universe_sha256=universe_hash)
        art_rows.append(
            _raw_compiled_row(
                source,
                package=packages[asset_id],
                order=order,
                rank=rank,
                universe_hash=universe_hash,
                merged_manifest_hash=merged_manifest_hash,
            )
        )
    datasets[art_index] = {
        "dataset_slug": "articraft_10k",
        "dataset_name": "Articraft-10K",
        "universe": {
            "kind": "frozen_articraft_github_merged_10787_roster",
            "path": str(universe_path.resolve(strict=True)),
            "sha256": universe_hash,
            "manifest_path": str(merged_manifest_path.resolve(strict=True)),
            "manifest_content_sha256": merged_manifest_hash,
            "candidate_count": 10_787,
        },
        "selection": {
            "eligible_count": eligible_count,
            "excluded_count": sum(excluded.values()),
            "exclusion_counts": dict(sorted(excluded.items())),
            "selected_count": len(art_rows),
            "ordered_asset_ids_sha256": canonical_sha256([r["asset_id"] for r in art_rows]),
            "ordered_ranks_sha256": canonical_sha256([r["cohort_selection"]["rank_sha256"] for r in art_rows]),
        },
        "rows": art_rows,
    }
    protocol = deepcopy(base["protocol"])
    protocol["baseline_candidate_universe"] = (
        "Articraft-10K uses the frozen 10,787-record GitHub merged roster; "
        "the other baseline datasets use their bound N=800 parent rosters"
    )
    protocol["articraft_candidate_universe"] = {
        "kind": "frozen_articraft_github_merged_10787_roster",
        "candidate_count": 10_787,
        "roster_sha256": universe_hash,
        "merged_manifest_content_sha256": merged_manifest_hash,
    }
    protocol["articraft_export_policy"] = (
        "selected Articraft records are re-materialized with the official "
        "full+validate compiler; geometry QC is non-blocking and findings are "
        "retained as warnings; output URDFs carry visual+collision geometry"
    )
    protocol["protocol_sha256"] = canonical_sha256(protocol, exclude_fields=("protocol_sha256",))
    manifest: dict[str, Any] = {
        "schema_version": base["schema_version"],
        "protocol": protocol,
        "protocol_sha256": protocol["protocol_sha256"],
        "sample_size_per_dataset": SAMPLE_SIZE,
        "dataset_count": len(datasets),
        "total_rows": SAMPLE_SIZE * len(datasets),
        "ordered_dataset_slugs": list(DATASET_SLUGS),
        "datasets": datasets,
    }
    manifest["cohort_sha256"] = canonical_sha256(
        [
            {
                "dataset_slug": d["dataset_slug"],
                "universe_sha256": d["universe"]["sha256"],
                "ordered_asset_ids_sha256": d["selection"]["ordered_asset_ids_sha256"],
                "ordered_ranks_sha256": d["selection"]["ordered_ranks_sha256"],
            }
            for d in datasets
        ]
    )
    manifest["manifest_sha256"] = canonical_sha256(manifest, exclude_fields=("manifest_sha256",))
    validate_manifest(manifest)
    destination = output / "cohort_manifest.json"
    atomic_write(destination, manifest)
    return destination


def run(args: argparse.Namespace) -> dict[str, Any]:
    output = args.out.resolve(strict=False)
    output.mkdir(parents=True, exist_ok=True)
    merged_path = args.merged_manifest.resolve(strict=True)
    rows, universe_hash, merged_manifest_hash = _source_rows(merged_path)
    selected, excluded, eligible_count = select_rows(rows, universe_hash=universe_hash)
    atomic_write(
        output / "selection.json",
        {
            "schema_version": "table5_v2_articraft_github_selection_v1",
            "source_manifest": str(merged_path),
            "source_manifest_content_sha256": merged_manifest_hash,
            "source_roster": str(merged_path.with_name("full_release_roster.jsonl")),
            "source_roster_sha256": universe_hash,
            "candidate_count": len(rows),
            "eligible_count": eligible_count,
            "excluded_count": sum(excluded.values()),
            "exclusion_counts": dict(sorted(excluded.items())),
            "selected_count": len(selected),
            "selected": [
                {
                    "asset_id": row["asset_id"],
                    "category": row.get("category") or row.get("raw_category"),
                    "joint_count": row.get("joint_count"),
                    "cohort_origin": row.get("cohort_origin"),
                    "rank_sha256": selection_rank(
                        "articraft_10k", str(row["asset_id"]), universe_sha256=universe_hash
                    ),
                }
                for row in selected
            ],
        },
    )
    packages = compile_selected(
        selected,
        output=output,
        source_root=args.source_root.resolve(strict=True),
        workers=args.workers,
        timeout_seconds=args.compile_timeout_seconds,
    )
    cohort_path = build_cohort(
        selected=selected,
        packages=packages,
        excluded=excluded,
        eligible_count=eligible_count,
        universe_path=merged_path.with_name("full_release_roster.jsonl"),
        universe_hash=universe_hash,
        merged_manifest_path=merged_path,
        merged_manifest_hash=merged_manifest_hash,
        base_cohort_path=args.base_cohort.resolve(strict=True),
        output=output,
    )
    return {
        "cohort": str(cohort_path),
        "candidate_count": len(rows),
        "eligible_count": eligible_count,
        "selected_count": len(selected),
        "excluded": dict(sorted(excluded.items())),
        "cohort_sha256": _load_json(cohort_path)["cohort_sha256"],
        "manifest_sha256": _load_json(cohort_path)["manifest_sha256"],
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--merged-manifest", type=Path, default=DEFAULT_MERGED_MANIFEST)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--base-cohort", type=Path, default=DEFAULT_BASE_COHORT)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--compile-timeout-seconds", type=float, default=COMPILE_TIMEOUT_SECONDS)
    args = parser.parse_args(argv)
    if args.workers < 1:
        parser.error("--workers must be positive")
    if not COMPILE_PYTHON.is_file() or not COMPILE_SCRIPT.is_file() or not COMPAT_SCRIPT.is_file():
        parser.error("official Articraft compiler environment is unavailable")
    try:
        result = run(args)
    except (OSError, ResampleError, ValueError, ET.ParseError) as error:
        parser.error(str(error))
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
