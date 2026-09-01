#!/usr/bin/env python3
"""Evaluate Table 1 on the frozen Ours-500K acquired release sample (Brain).

Cohort source
-------------
The Ours-500K sample under evaluation is the locally acquired, hash-verified
release archive ``exp/Brain/arti_cabinet_drawer_geometry_500_20260813.zip``
(ModelScope dataset ``Brain``).  The archive contains
``arti_cabinet_drawer_geometry_500_20260813/<category>/<seed_N>/`` asset
packages, each with ``model.urdf``, ``appearance.json``, ``physics.json`` and
an ``assets/`` resource closure (meshes + materials/textures).

The acquired sample holds 500 assets, which is below the N=800 sample size
used for larger releases; the frozen cohort is therefore the FULL acquired
release roster (no subsampling, no replacement, no outcome filtering):
``N_release = N_eval = 500``.

Metrics reuse the frozen shared Table 1 structural evaluator
(``run_table1_artiverse.py`` loaded as ``table1_structural_metrics_shared``),
exactly as the Artiverse / Articraft / LAM / PartNet-Mobility / PhysX-Mobility
Table 1 runs do:

- ``Links/Asset`` and ``Movable Joints/Asset``: XML element counts;
- ``Multi-joint Assets``: assets with >= 2 non-fixed declared joints;
- ``Unique Topologies``: graph hash of the canonical rooted joint tree
  (``rooted-joint-tree-v1``);
- ``Exact Duplicate Rate``: fingerprint of the canonical URDF plus its
  recursive simulation-resource closure (``simulation-package-fingerprint-v2``).
  The fingerprint containment root is the per-asset package directory, which
  natively contains the full resource closure in the release layout.

All failures count against the intent-to-evaluate denominator (fail closed).
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
from typing import Any
import uuid
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
DEFAULT_DATASET_ROOT = (
    REPO / "exp/Brain/extracted/arti_cabinet_drawer_geometry_500_20260813"
)
DEFAULT_ARCHIVE = REPO / "exp/Brain/arti_cabinet_drawer_geometry_500_20260813.zip"
DEFAULT_ARCHIVE_SHA256_FILE = (
    REPO / "exp/Brain/arti_cabinet_drawer_geometry_500_20260813.zip.sha256"
)
DEFAULT_OUTPUT = REPO / "exp/runtime/table1_ours_500k"
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
DATASET_LABEL = "Ours-500K"
RELEASE_STATUS = "ACQUIRED_RELEASE_SAMPLE_BRAIN_MODELSCOPE"
PAPER_REPORTED_ASSETS = "10K"
COHORT_TYPE = "FULL_ACQUIRED_RELEASE_SAMPLE_NO_SUBSAMPLING"
ROSTER_PROTOCOL = "ours500k-table1-roster-v1"
EXPECTED_ARCHIVE_SHA256 = (
    "ffedf5bd90ae5eb96a061d0e127b700915ed6c221eeb7c5afe282b7249bfbd66"
)


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


def _error_text(error: BaseException) -> str:
    return f"{type(error).__name__}: {error}"


def sha256_file(path: Path) -> str:
    return SHARED.sha256_file(path)


def verify_archive_binding(archive: Path, sha_file: Path) -> dict[str, Any]:
    """Verify the acquired archive against its published SHA-256 sidecar."""
    archive = archive.resolve(strict=True)
    declared = sha_file.read_text(encoding="utf-8").split()
    if len(declared) < 1:
        raise ValueError(f"invalid sha256 sidecar: {sha_file}")
    declared_sha = declared[0].strip().lower()
    if declared_sha != EXPECTED_ARCHIVE_SHA256:
        raise ValueError(
            f"sidecar sha256 {declared_sha!r} does not match frozen expected value"
        )
    computed = sha256_file(archive)
    if computed != EXPECTED_ARCHIVE_SHA256:
        raise ValueError(
            f"archive sha256 mismatch: computed {computed!r}, expected {EXPECTED_ARCHIVE_SHA256!r}"
        )
    return {
        "archive": str(archive),
        "archive_bytes": archive.stat().st_size,
        "archive_sha256": computed,
        "sha256_sidecar": str(sha_file.resolve(strict=True)),
        "sha256_sidecar_declared": declared_sha,
        "binding": "ARCHIVE_BYTE_HASH_MATCHES_PUBLISHED_SIDECAR",
    }


def build_release_roster(dataset_root: Path) -> list[dict[str, Any]]:
    """Census the acquired release roster: <category>/<seed_N> asset packages."""
    dataset_root = dataset_root.resolve(strict=True)
    if not dataset_root.is_dir():
        raise ValueError(f"dataset root missing: {dataset_root}")
    identities: list[dict[str, Any]] = []
    problems: list[str] = []
    for category_dir in sorted(
        path for path in dataset_root.iterdir() if path.is_dir()
    ):
        category = category_dir.name
        seed_dirs = sorted(
            path for path in category_dir.iterdir() if path.is_dir()
        )
        if not seed_dirs:
            problems.append(f"EMPTY_CATEGORY:{category}")
        for seed_dir in seed_dirs:
            urdf_path = seed_dir / "model.urdf"
            identity = {
                "asset_id": f"{category}/{seed_dir.name}",
                "raw_category": category,
                "seed_name": seed_dir.name,
                "asset_root": f"{category}/{seed_dir.name}",
                "primary_urdf": f"{category}/{seed_dir.name}/model.urdf",
                "urdf_present": urdf_path.is_file(),
            }
            identities.append(identity)
    if problems:
        raise ValueError(f"roster problems detected: {problems}")
    if not identities:
        raise ValueError("release roster is empty")
    asset_ids = [row["asset_id"] for row in identities]
    if len(set(asset_ids)) != len(asset_ids):
        raise ValueError("release roster asset ids are not unique")
    return identities


def freeze_roster(identities: list[dict[str, Any]], archive_binding: dict[str, Any]) -> str:
    """Deterministic roster hash binding every asset identity to the archive."""
    payload_lines = [
        ROSTER_PROTOCOL,
        archive_binding["archive_sha256"],
        str(len(identities)),
    ]
    for row in sorted(identities, key=lambda item: item["asset_id"]):
        payload_lines.append(
            "\0".join((row["asset_id"], row["raw_category"], row["seed_name"]))
        )
    return hashlib.sha256("\n".join(payload_lines).encode("utf-8")).hexdigest()


def evaluate_asset(dataset_root: Path, identity: dict[str, Any]) -> dict[str, Any]:
    record: dict[str, Any] = {
        **identity,
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
    asset_root = (dataset_root / str(identity["asset_root"])).resolve(strict=False)
    try:
        asset_root.relative_to(dataset_root.resolve())
    except ValueError:
        record["status"] = "INVALID_ASSET_ROOT"
        record["error"] = f"asset root escapes dataset root: {asset_root}"
        return record
    if not asset_root.is_dir():
        record["status"] = "MISSING_ASSET_ROOT"
        record["error"] = f"missing asset root: {identity['asset_root']}"
        return record

    urdf_path = asset_root / "model.urdf"
    record["primary_urdf_sha256"] = None
    if not urdf_path.is_file():
        record["status"] = "MISSING_PRIMARY_URDF"
        record["error"] = "expected model.urdf in asset root; not found"
        return record
    try:
        record["primary_urdf_sha256"] = sha256_file(urdf_path)
    except OSError as error:
        record["status"] = "URDF_READ_FAILED"
        record["error"] = _error_text(error)
        return record
    try:
        analysis = SHARED.analyze_urdf(urdf_path)
    except (ET.ParseError, OSError, ValueError) as error:
        record["status"] = "URDF_PARSE_FAILED"
        record["error"] = _error_text(error)
        return record
    record.update(analysis)
    record["parse_success"] = True

    try:
        fingerprint = SHARED.fingerprint_package(urdf_path)
    except (ET.ParseError, OSError, ValueError) as error:
        record["status"] = "FINGERPRINT_FAILED"
        record["error"] = _error_text(error)
        return record
    record["fingerprint_complete"] = bool(fingerprint["complete"])
    record["package_fingerprint"] = fingerprint["fingerprint"]
    record["referenced_resource_count"] = fingerprint["resource_count"]
    record["missing_resources"] = fingerprint["missing_resources"]
    if fingerprint["complete"]:
        record["status"] = "EVALUATED"
    else:
        record["status"] = "EVALUATED_FINGERPRINT_INCOMPLETE"
        record["error"] = "one or more referenced package resources are unavailable"
    return record


def evaluate_asset_fail_closed(
    dataset_root: Path, identity: dict[str, Any]
) -> dict[str, Any]:
    try:
        return evaluate_asset(dataset_root, identity)
    except Exception as error:  # fail closed; keep intent-to-evaluate denominator
        return {
            **identity,
            "status": "ASSET_EVALUATION_FAILED",
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
            "error": _error_text(error),
        }


def _report(summary: dict[str, Any], run_manifest: dict[str, Any]) -> str:
    cohort = summary["cohort"]
    links = summary["links_per_asset"]
    movable = summary["movable_joints_per_asset"]
    multi = summary["multi_joint_assets"]
    topology = summary["unique_topologies"]
    duplicate = summary["exact_duplicate_rate"]
    macro = summary["category_macro"]

    def percentage(value: float | None) -> str:
        return "N/E" if value is None else f"{100.0 * value:.2f}%"

    def statistic(value: float | int | None, *, decimals: int | None = None) -> str:
        if value is None:
            return "N/E"
        if decimals is None:
            return str(value)
        return f"{value:.{decimals}f}"

    return "\n".join(
        (
            "# Ours-500K Table 1: Dataset Scale and Structural Diversity",
            "",
            "## Frozen cohort",
            "",
            f"- Acquired release archive: `{run_manifest['archive_binding']['archive']}`.",
            f"- Archive SHA-256: `{run_manifest['archive_binding']['archive_sha256']}` (matches published sidecar).",
            f"- Roster hash ({ROSTER_PROTOCOL}): `{run_manifest['roster_sha256']}`.",
            f"- `N_release`: {cohort['N_release']} assets across {cohort['release_raw_categories']} raw categories.",
            f"- `N_eval`: {cohort['N_eval']} assets (full acquired roster; below the N=800 sample size used for larger releases; no subsampling).",
            f"- Cohort type: {COHORT_TYPE}; this is not the shared-category balanced cohort.",
            "",
            "## Table 1 result",
            "",
            "| Dataset / Outputs | Paper-reported Assets | N_release | N_eval | #Categories (release / eval) | Links/Asset (mean / median / P90) | Movable Joints/Asset (mean / median / P90) | Multi-joint Assets | Unique Topologies | Exact Duplicate Rate |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
            (
                f"| {DATASET_LABEL} | {PAPER_REPORTED_ASSETS} | {cohort['N_release']} | {cohort['N_eval']} | "
                f"{cohort['release_raw_categories']} / {cohort['eval_raw_categories']} | "
                f"{statistic(links['mean'], decimals=2)} / "
                f"{statistic(links['median'])} / "
                f"{statistic(links['p90_nearest_rank'])} | "
                f"{statistic(movable['mean'], decimals=2)} / "
                f"{statistic(movable['median'])} / "
                f"{statistic(movable['p90_nearest_rank'])} | "
                f"{percentage(multi['rate'])} | "
                f"{percentage(topology['rate'])} (n={topology['denominator']}) | "
                f"{percentage(duplicate['rate'])} (n={duplicate['denominator']}) |"
            ),
            "",
            "## Diagnostics",
            "",
            f"- XML parse coverage: {cohort['N_parse']} / {cohort['N_eval']} ({percentage(cohort['N_parse'] / cohort['N_eval'] if cohort['N_eval'] else None)}).",
            f"- Category macro over {macro['category_count']} raw categories: multi-joint {percentage(macro['multi_joint_assets_rate'])}; unique topologies {percentage(macro['unique_topologies_rate'])} over {macro['unique_topologies_evaluable_categories']} evaluable categories; exact duplicate rate {percentage(macro['exact_duplicate_rate'])} over {macro['exact_duplicate_evaluable_categories']} evaluable categories.",
            f"- Unique topologies: {topology['unique']} unique hashes; coverage {topology['denominator']} / {topology['coverage_denominator']} ({percentage(topology['coverage_rate'])}).",
            f"- Exact duplicates: excess {duplicate['duplicate_excess']}; assets in duplicate clusters {duplicate['assets_in_duplicate_clusters']} / {duplicate['denominator']} ({percentage(duplicate['assets_in_duplicate_clusters_rate'])}); {duplicate['duplicate_cluster_count']} clusters, maximum size {duplicate['max_cluster_size']}; coverage {duplicate['denominator']} / {duplicate['coverage_denominator']} ({percentage(duplicate['coverage_rate'])}).",
            f"- Declared joint type counts: {json.dumps(summary['declared_joint_type_counts'], sort_keys=True)}.",
            f"- Status counts: {json.dumps(summary['status_counts'], sort_keys=True)}.",
            "- Movable-joint counts include all declared XML joints except literal `fixed`, including exporter extension types; this is not a runtime-valid DoF count.",
            "- Unique-topology rate is defined over valid rooted trees only; coverage against `N_eval` is reported separately.",
            "- Exact duplicate rate uses canonicalized URDF plus the recursively resolved simulation resource closure; incomplete closures are not treated as unique.",
            "",
        )
    )


@contextmanager
def _output_lock(output: Path):
    output.parent.mkdir(parents=True, exist_ok=True)
    lock_path = output.parent / f".{output.name}.lock"
    handle = open(lock_path, "a+", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.close()
        raise RuntimeError(f"another run holds the output lock: {lock_path}")
    try:
        yield
    finally:
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        handle.close()


def _verify_staged_artifacts(output: Path) -> None:
    for name in ("manifest.json", "asset_records.jsonl", "summary.json", "report.md"):
        path = output / name
        if not path.is_file() or path.stat().st_size == 0:
            raise RuntimeError(f"staged artifact missing or empty: {name}")


def _publish_staged_output(staging: Path, output: Path) -> None:
    if output.exists():
        raise RuntimeError(f"output already exists; refusing to overwrite: {output}")
    os.rename(staging, output)


def run(args: argparse.Namespace) -> dict[str, Any]:
    output = args.output.absolute()
    dataset_root = args.dataset_root.resolve(strict=True)
    try:
        output.relative_to(dataset_root)
    except ValueError:
        pass
    else:
        raise ValueError("output must not be inside the dataset root")

    with _output_lock(output):
        staging = output.parent / (
            f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}"
        )
        staging.mkdir(parents=False, exist_ok=False)
        try:
            summary = _run_to_output(args, staging)
            _verify_staged_artifacts(staging)
            _publish_staged_output(staging, output)
            return summary
        finally:
            if staging.exists():
                shutil.rmtree(staging)


def _run_to_output(args: argparse.Namespace, output: Path) -> dict[str, Any]:
    dataset_root = args.dataset_root.resolve(strict=True)
    protocol = args.protocol.resolve(strict=True)

    archive_binding = verify_archive_binding(args.archive, args.archive_sha256_file)
    identities = build_release_roster(dataset_root)
    if len(identities) != args.expected_release_count:
        raise ValueError(
            f"release roster size {len(identities)} != expected {args.expected_release_count}"
        )
    roster_sha256 = freeze_roster(identities, archive_binding)
    categories = sorted({row["raw_category"] for row in identities})

    started_at = datetime.now(timezone.utc).isoformat()
    run_manifest: dict[str, Any] = {
        "schema_version": "table1_ours_500k_manifest_v1",
        "dataset": DATASET_LABEL,
        "release_status": RELEASE_STATUS,
        "paper_reported_assets": PAPER_REPORTED_ASSETS,
        "N_release": len(identities),
        "N_eval": len(identities),
        "release_raw_category_count": len(categories),
        "release_raw_categories": categories,
        "sample_size_policy": (
            "acquired roster (500) is below the N=800 sample size used for larger "
            "releases; the full acquired roster enters evaluation with no "
            "subsampling, replacement, or outcome filtering"
        ),
        "cohort_type": COHORT_TYPE,
        "missing_or_failed_assets_retained": True,
        "roster_protocol": ROSTER_PROTOCOL,
        "roster_sha256": roster_sha256,
        "archive_binding": archive_binding,
        "dataset_root": str(dataset_root),
        "protocol": str(protocol),
        "protocol_sha256": sha256_file(protocol),
        "runner": str(Path(__file__).resolve()),
        "runner_sha256": sha256_file(Path(__file__).resolve()),
        "shared_evaluator": str(
            Path(__file__).with_name("run_table1_artiverse.py").resolve()
        ),
        "shared_evaluator_sha256": sha256_file(
            Path(__file__).with_name("run_table1_artiverse.py")
        ),
        "topology_protocol": SHARED.TOPOLOGY_PROTOCOL,
        "fingerprint_protocol": SHARED.FINGERPRINT_PROTOCOL,
        "movable_joint_policy": (
            "all declared XML joint elements whose normalized type is not literal fixed; "
            "includes exporter extension types and does not imply runtime-valid DoFs"
        ),
        "topology_denominator_policy": (
            "unique topology hashes divided by assets with valid rooted joint trees; "
            "coverage is reported against N_eval"
        ),
        "duplicate_denominator_policy": (
            "duplicate excess divided by assets with complete simulation-package fingerprints; "
            "coverage is reported against N_eval"
        ),
        "p90_definition": "nearest-rank: sorted_values[ceil(0.90 * n) - 1]",
        "workers": args.workers,
        "python_executable": sys.executable,
        "python_version": sys.version,
        "started_at_utc": started_at,
        "assets": [
            {**row, "selection_rank": rank}
            for rank, row in enumerate(
                sorted(identities, key=lambda item: item["asset_id"]), start=1
            )
        ],
    }
    SHARED.write_json(output / "manifest.json", run_manifest)

    if args.workers == 1:
        records = [evaluate_asset_fail_closed(dataset_root, row) for row in identities]
    else:
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            records = list(
                executor.map(
                    lambda row: evaluate_asset_fail_closed(dataset_root, row),
                    identities,
                )
            )
    # canonical record order: sorted by asset_id
    records.sort(key=lambda row: str(row["asset_id"]))
    SHARED.write_jsonl(output / "asset_records.jsonl", records)

    summary = SHARED.aggregate_records(
        records,
        release_asset_count=len(identities),
        release_category_count=len(categories),
    )
    summary["status_counts"] = dict(
        sorted(Counter(str(row["status"]) for row in records).items())
    )
    summary["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
    SHARED.write_json(output / "summary.json", summary)
    report_text = _report(summary, run_manifest)
    (output / "report.md").write_text(report_text, encoding="utf-8")

    artifact_files = ("manifest.json", "asset_records.jsonl", "summary.json", "report.md")
    artifact_manifest = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "files": {
            name: {
                "bytes": (output / name).stat().st_size,
                "sha256": sha256_file(output / name),
            }
            for name in artifact_files
        },
    }
    SHARED.write_json(output / "artifact_manifest.json", artifact_manifest)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument(
        "--archive-sha256-file", type=Path, default=DEFAULT_ARCHIVE_SHA256_FILE
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--expected-release-count", type=int, default=500)
    parser.add_argument("--workers", type=int, default=8)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.workers <= 0:
        raise SystemExit("--workers must be positive")
    summary = run(args)
    print(
        json.dumps(
            {
                "state": "COMPLETE",
                "output": str(args.output.resolve()),
                "N_release": summary["cohort"]["N_release"],
                "N_eval": summary["cohort"]["N_eval"],
                "N_parse": summary["cohort"]["N_parse"],
                "status_counts": summary["status_counts"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
