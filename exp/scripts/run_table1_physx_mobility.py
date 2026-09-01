#!/usr/bin/env python3
"""Evaluate Table 1 on the frozen PhysX-Mobility N=800 cohort.

The cohort is the frozen, source-bound Table 5 receipt set
(``exp/runtime/table5_physx_mobility_n800_v2``): canonical protocol +
manifest + receipt marker, all hash-bound to the official
``Caoza/PhysX-Mobility`` archive. Table 1 metrics are declared-layer
structural statistics over the official released URDFs:

- ``Links/Asset`` and ``Movable Joints/Asset``: XML element counts;
- ``Multi-joint Assets``: assets with >= 2 non-fixed declared joints;
- ``Unique Topologies``: graph hash of the canonical rooted joint tree;
- ``Exact Duplicate Rate``: fingerprint of the canonical URDF plus its
  recursive simulation-resource closure.

PhysX-Mobility release geometry places resources in ``partseg/`` as a
sibling of the flat ``urdf/`` directory, so the URDF directory itself
cannot contain the resource closure. The fingerprint containment root is
therefore a per-asset staging root that replicates the release geometry
byte-for-byte (hash-verified); canonicalization and hashing follow the
shared ``simulation-package-fingerprint-v2`` protocol exactly.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Any
import uuid
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
DEFAULT_DATASET_ROOT = REPO / "exp/PhysX-Mobility/extracted/PhysX_mobility"
DEFAULT_RECEIPT_SET = REPO / "exp/runtime/table5_physx_mobility_n800_v2"
DEFAULT_OUTPUT = REPO / "exp/runtime/table1_physx_mobility"
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
DEFAULT_TABLE5_COMMON = (
    Path("/mnt/zsn/lyb/arti-skill-table5-physx-n800/exp/scripts/table5_physx_common.py")
)
EXPECTED_MANIFEST_SCHEMA = "table5_physx_mobility_manifest_v1"
EXPECTED_RECEIPT_SCHEMA = "table5_physx_mobility_receipt_set_v1"
EXPECTED_RELEASE_ASSET_COUNT = 2024
EXPECTED_RELEASE_CATEGORY_COUNT = 132
COHORT_TYPE = "FROZEN_HASH_RANKED_SAMPLE_NOT_CATEGORY_BALANCED"


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
TABLE5 = _load_module(
    DEFAULT_TABLE5_COMMON,
    "table5_physx_mobility_frozen_contract_shared",
)


def _error_text(error: BaseException) -> str:
    return f"{type(error).__name__}: {error}"


def _normalize_category(label: str) -> str:
    """Supplementary-only label folding: strip all whitespace, casefold."""
    return "".join(str(label).split()).casefold()


def load_receipt_set(receipt_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Validate and return the frozen Table 5 (protocol, manifest) pair."""
    receipt_dir = receipt_dir.resolve(strict=True)
    marker_path = receipt_dir / "receipt_set.json"
    try:
        marker = json.loads(marker_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"receipt marker is unavailable or invalid: {error}") from error
    if marker.get("schema_version") != EXPECTED_RECEIPT_SCHEMA:
        raise ValueError(f"unexpected receipt marker schema: {marker.get('schema_version')!r}")
    protocol, manifest = TABLE5.validate_receipt_set(receipt_dir)
    if manifest.get("schema_version") != EXPECTED_MANIFEST_SCHEMA:
        raise ValueError(f"unexpected manifest schema: {manifest.get('schema_version')!r}")
    return protocol, manifest


def release_roster(dataset_root: Path, *, expected_count: int) -> dict[str, Any]:
    """Census the full release closure: IDs and declared category labels."""
    candidates = TABLE5.collect_candidate_ids(dataset_root, expected_count=expected_count)
    categories: dict[int, str] = {}
    for dataset_id in candidates:
        finaljson_path = dataset_root / "finaljson" / f"{dataset_id}.json"
        try:
            category = json.loads(finaljson_path.read_text(encoding="utf-8"))["category"]
        except (OSError, json.JSONDecodeError, KeyError) as error:
            raise ValueError(f"roster category unavailable for {dataset_id}") from error
        if not isinstance(category, str) or not category.strip():
            raise ValueError(f"roster category invalid for {dataset_id}")
        categories[dataset_id] = category
    return {"candidate_ids": candidates, "categories": categories}


def validate_cohort(
    manifest: dict[str, Any],
    protocol: dict[str, Any],
    dataset_root: Path,
    *,
    verify_archive: bool,
) -> None:
    """Re-derive the frozen selection and verify every row binding on disk."""
    dataset_root = dataset_root.resolve(strict=True)
    TABLE5.validate_manifest(manifest, dataset_root, protocol=protocol)
    if verify_archive:
        TABLE5.validate_source_bindings(manifest, dataset_root, official=True)


def load_formal_cohort(
    receipt_dir: Path,
    dataset_root: Path,
    *,
    expected_n: int,
    verify_archive: bool = True,
) -> dict[str, Any]:
    protocol, manifest = load_receipt_set(receipt_dir)
    selection = manifest.get("selection", {})
    if selection.get("candidate_count") != EXPECTED_RELEASE_ASSET_COUNT:
        raise ValueError(
            f"release closure must contain exactly {EXPECTED_RELEASE_ASSET_COUNT} assets"
        )
    if selection.get("selected_count") != expected_n or len(manifest.get("rows", [])) != expected_n:
        raise ValueError(f"expected exactly {expected_n} frozen PhysX-Mobility rows")
    validate_cohort(manifest, protocol, dataset_root, verify_archive=verify_archive)
    roster = release_roster(dataset_root, expected_count=EXPECTED_RELEASE_ASSET_COUNT)
    release_category_count = len(set(roster["categories"].values()))
    if release_category_count != EXPECTED_RELEASE_CATEGORY_COUNT:
        raise ValueError(
            "formal PhysX-Mobility release must contain exactly "
            f"{EXPECTED_RELEASE_CATEGORY_COUNT} declared raw category labels; "
            f"found {release_category_count}"
        )
    for row in manifest["rows"]:
        if roster["categories"][int(row["dataset_id"])] != row["category"]:
            raise ValueError(f"category metadata mismatch: {row['dataset_id']}")
    assets = [_identity(row) for row in manifest["rows"]]
    return {
        "protocol": protocol,
        "manifest": manifest,
        "roster": roster,
        "release_asset_count": len(roster["candidate_ids"]),
        "release_category_count": release_category_count,
        "assets": assets,
    }


def _identity(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "selection_index": int(row["rank"]) - 1,
        "asset_id": str(row["dataset_id"]),
        "dataset_id": int(row["dataset_id"]),
        "rank": int(row["rank"]),
        "rank_sha256": str(row["rank_sha256"]),
        "raw_category": str(row["category"]),
        "urdf_relative_path": str(row["urdf_relative_path"]),
        "urdf_sha256": str(row["urdf_sha256"]),
        "finaljson_sha256": str(row["finaljson_sha256"]),
        "resource_sha256": str(row["resource_sha256"]),
        "manifest_row_sha256": TABLE5.canonical_sha256(row),
        "declared_collision_elements": int(row["xml_counts"]["collision_elements"]),
    }


def _record_template(identity: dict[str, Any]) -> dict[str, Any]:
    return {
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


def stage_package(
    dataset_root: Path,
    row: dict[str, Any],
    staging_root: Path,
) -> tuple[Path, Path]:
    """Copy the asset byte-for-byte into a staging root mirroring release geometry."""
    dataset_root = dataset_root.resolve()
    asset_root = staging_root / str(row["dataset_id"])
    items = [(str(row["urdf_relative_path"]), str(row["urdf_sha256"]))]
    items.extend(
        (str(resource["relative_path"]), str(resource["sha256"]))
        for resource in row["resources"]
    )
    seen: set[str] = set()
    for relative, expected_sha256 in items:
        if relative in seen:
            raise ValueError(f"duplicate resource binding: {relative}")
        seen.add(relative)
        source = dataset_root / relative
        target = asset_root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        if SHARED.sha256_file(target) != expected_sha256:
            raise ValueError(f"staged byte mismatch: {relative}")
    urdf_path = asset_root / str(row["urdf_relative_path"])
    return asset_root, urdf_path


def fingerprint_package_with_root(urdf_path: Path, package_root: Path) -> dict[str, Any]:
    """Shared fingerprint protocol with an explicit containment root.

    Identical to ``SHARED.fingerprint_package`` except that the containment
    root is passed explicitly, because PhysX-Mobility mesh references
    (``./../partseg/...``) resolve to a sibling of the ``urdf/`` directory.
    """
    package_root = package_root.resolve()
    root = copy.deepcopy(ET.parse(urdf_path).getroot())
    resources: set[Path] = set()
    missing: set[str] = set()
    cache: dict[Path, str | None] = {}

    for resource_node in [*root.findall(".//mesh"), *root.findall(".//texture")]:
        reference = resource_node.attrib.get("filename", "")
        resource = SHARED._contained_resource(package_root, urdf_path.parent, reference)
        if resource is None:
            missing.add(f"UNRESOLVED:{reference}")
            continue
        digest = SHARED._resource_digest(
            resource,
            package_root=package_root,
            cache=cache,
            active=set(),
            resources=resources,
            missing=missing,
        )
        if digest is not None:
            resource_node.attrib["filename"] = f"sha256:{digest}"

    if missing:
        return {
            "complete": False,
            "fingerprint": None,
            "resource_count": len(resources),
            "missing_resources": sorted(missing),
        }

    canonical = SHARED._canonical_urdf(root)
    fingerprint = hashlib.sha256(
        f"{SHARED.FINGERPRINT_PROTOCOL}\0{canonical}".encode("utf-8")
    ).hexdigest()
    return {
        "complete": True,
        "fingerprint": fingerprint,
        "resource_count": len(resources),
        "missing_resources": [],
    }


def evaluate_asset(
    dataset_root: Path,
    row: dict[str, Any],
    staging_root: Path,
) -> dict[str, Any]:
    identity = _identity(row)
    record = _record_template(identity)
    urdf_path = dataset_root.resolve() / str(row["urdf_relative_path"])
    try:
        record.update(SHARED.analyze_urdf(urdf_path))
        record["parse_success"] = True
        xml_counts = row["xml_counts"]
        observed = (
            record["link_count"],
            record["joint_count"],
            record["non_fixed_joint_count"],
        )
        frozen = (
            int(xml_counts["links"]),
            int(xml_counts["joints"]),
            int(xml_counts["movable_joints"]),
        )
        if observed != frozen:
            raise ValueError(
                f"declared XML counts drifted from frozen metadata: {observed} != {frozen}"
            )
    except (ET.ParseError, OSError, ValueError, RuntimeError) as error:
        failed = _record_template(identity)
        failed["status"] = "EVALUATION_FAILED"
        failed["error"] = _error_text(error)
        return failed

    asset_root: Path | None = None
    try:
        try:
            asset_root, staged_urdf = stage_package(dataset_root, row, staging_root)
            observation = fingerprint_package_with_root(staged_urdf, asset_root)
        finally:
            if asset_root is not None:
                shutil.rmtree(asset_root, ignore_errors=True)
    except (OSError, ValueError, RuntimeError) as error:
        record["status"] = "EVALUATION_FAILED"
        record["error"] = _error_text(error)
        return record

    record["fingerprint_complete"] = bool(observation["complete"])
    record["package_fingerprint"] = observation["fingerprint"]
    record["referenced_resource_count"] = observation["resource_count"]
    record["missing_resources"] = observation["missing_resources"]
    if record["fingerprint_complete"]:
        record["status"] = "EVALUATED"
    else:
        record["status"] = "EVALUATED_FINGERPRINT_INCOMPLETE"
        record["error"] = "one or more referenced simulation resources are unavailable"
    return record


def _evaluate_asset_fail_closed(
    dataset_root: Path,
    row: dict[str, Any],
    staging_root: Path,
) -> dict[str, Any]:
    try:
        return evaluate_asset(dataset_root, row, staging_root)
    except Exception as error:  # fail closed, keep the denominator intact
        record = _record_template(_identity(row))
        record["status"] = "EVALUATION_FAILED"
        record["error"] = _error_text(error)
        return record


def aggregate_physx_records(
    records: list[dict[str, Any]],
    *,
    release_asset_count: int,
    release_category_count: int,
    roster_categories: dict[int, str],
) -> dict[str, Any]:
    summary = SHARED.aggregate_records(
        records,
        release_asset_count=release_asset_count,
        release_category_count=release_category_count,
    )
    summary["cohort"]["cohort_type"] = COHORT_TYPE
    eval_labels = [str(row["raw_category"]) for row in records]
    summary["category_census"] = {
        "label_source": "official finaljson category field, exact declared strings",
        "normalization_rule": "supplementary only: strip all whitespace then casefold",
        "release_raw_categories": release_category_count,
        "release_normalized_categories": len(
            {_normalize_category(label) for label in roster_categories.values()}
        ),
        "eval_raw_categories": len(set(eval_labels)),
        "eval_normalized_categories": len(
            {_normalize_category(label) for label in eval_labels}
        ),
        "release_raw_category_labels": sorted(set(roster_categories.values())),
    }
    summary["declared_collision_element_total"] = sum(
        int(row.get("declared_collision_elements") or 0) for row in records
    )
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
    census = summary["category_census"]
    return "\n".join(
        (
            "# PhysX-Mobility Table 1",
            "",
            "| Dataset | N_release | N_eval | Categories (release / eval) | Links/Asset (mean / median / P90) | Movable Joints/Asset (mean / median / P90) | Multi-joint Assets | Unique Topologies | Exact Duplicate Rate |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
            (
                f"| PhysX-Mobility | {cohort['N_release']:,} | {cohort['N_eval']} | "
                f"{census['release_raw_categories']} / {census['eval_raw_categories']} | "
                f"{links['mean']:.2f} / {links['median']:g} / {links['p90_nearest_rank']} | "
                f"{movable['mean']:.2f} / {movable['median']:g} / {movable['p90_nearest_rank']} | "
                f"{_percentage(multi['rate'])} | "
                f"{_percentage(topology['rate'])} (n={topology['denominator']}) | "
                f"{_percentage(duplicate['rate'])} (n={duplicate['denominator']}) |"
            ),
            "",
            (
                f"Frozen receipt set: manifest `{run_manifest['input']['manifest_file_sha256']}`, "
                f"cohort `{run_manifest['input']['manifest_cohort_sha256']}`, "
                f"protocol `{run_manifest['input']['manifest_protocol_sha256']}`."
            ),
            (
                "Category labels are exact declared finaljson strings; "
                f"normalized (whitespace-stripped, casefolded) counts are "
                f"{census['release_normalized_categories']} release / "
                f"{census['eval_normalized_categories']} eval (supplementary only)."
            ),
            "",
        )
    )


def _run_to_output(args: argparse.Namespace, output: Path) -> dict[str, Any]:
    dataset_root = args.dataset_root.resolve(strict=True)
    receipt_dir = args.receipt_set.resolve(strict=True)
    started_at = datetime.now(timezone.utc).isoformat()

    cohort = load_formal_cohort(
        receipt_dir,
        dataset_root,
        expected_n=args.expected_n,
        verify_archive=not args.skip_archive_binding,
    )
    manifest = cohort["manifest"]
    assets = cohort["assets"]
    rows_by_id = {int(row["dataset_id"]): row for row in manifest["rows"]}

    staging_root = Path(
        tempfile.mkdtemp(prefix="table1_physx_mobility_staging.", dir=args.staging_parent)
    )
    try:
        run_manifest = {
            "schema_version": 1,
            "created_at_utc": started_at,
            "dataset": "Caoza/PhysX-Mobility",
            "dataset_root": str(dataset_root),
            "input": {
                "receipt_set": str(receipt_dir),
                "receipt_set_marker_sha256": SHARED.sha256_file(receipt_dir / "receipt_set.json"),
                "protocol_file_sha256": SHARED.sha256_file(receipt_dir / "protocol.json"),
                "manifest_file_sha256": SHARED.sha256_file(receipt_dir / "manifest.json"),
                "manifest_schema_version": manifest["schema_version"],
                "manifest_cohort_sha256": manifest["cohort_sha256"],
                "manifest_protocol_sha256": manifest["protocol_sha256"],
                "selection": manifest["selection"],
                "source_receipt": manifest["source_receipt"],
                "canonical_protocol_file": str(TABLE5.CANONICAL_PROTOCOL_PATH),
                "canonical_protocol_file_sha256": SHARED.sha256_file(
                    TABLE5.CANONICAL_PROTOCOL_PATH
                ),
                "canonical_protocol_file_expected_sha256": TABLE5.CANONICAL_PROTOCOL_FILE_SHA256,
                "archive_binding_verified": not args.skip_archive_binding,
            },
            "cohort_type": COHORT_TYPE,
            "protocol": str(args.protocol.resolve()),
            "protocol_sha256": SHARED.sha256_file(args.protocol.resolve()),
            "runner": str(Path(__file__).resolve()),
            "runner_sha256": SHARED.sha256_file(Path(__file__).resolve()),
            "shared_metric_runner": str(Path(SHARED.__file__).resolve()),
            "shared_metric_runner_sha256": SHARED.sha256_file(Path(SHARED.__file__).resolve()),
            "frozen_contract_runner": str(Path(TABLE5.__file__).resolve()),
            "frozen_contract_runner_sha256": SHARED.sha256_file(Path(TABLE5.__file__).resolve()),
            "topology_protocol": SHARED.TOPOLOGY_PROTOCOL,
            "fingerprint_protocol": SHARED.FINGERPRINT_PROTOCOL,
            "fingerprint_containment_policy": (
                "per-asset staging root replicating release geometry; required because "
                "official PhysX-Mobility mesh references resolve to a sibling of the "
                "urdf/ directory; canonicalization and hashing identical to the shared "
                "fingerprint protocol"
            ),
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
            "assets": assets,
        }
        SHARED.write_json(output / "manifest.json", run_manifest)

        if args.workers == 1:
            records = [
                _evaluate_asset_fail_closed(dataset_root, rows_by_id[asset["dataset_id"]], staging_root)
                for asset in assets
            ]
        else:
            with ThreadPoolExecutor(max_workers=args.workers) as executor:
                records = list(
                    executor.map(
                        lambda asset: _evaluate_asset_fail_closed(
                            dataset_root, rows_by_id[asset["dataset_id"]], staging_root
                        ),
                        assets,
                    )
                )
    finally:
        shutil.rmtree(staging_root, ignore_errors=True)

    SHARED.write_jsonl(output / "asset_records.jsonl", records)
    summary = aggregate_physx_records(
        records,
        release_asset_count=cohort["release_asset_count"],
        release_category_count=cohort["release_category_count"],
        roster_categories=cohort["roster"]["categories"],
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
    dataset_root = args.dataset_root.resolve()
    try:
        output.relative_to(dataset_root)
    except ValueError:
        pass
    else:
        raise ValueError("output must not be inside the PhysX-Mobility dataset root")

    with SHARED._output_lock(output):
        staging = output.parent / (
            f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}"
        )
        staging.mkdir(parents=False, exist_ok=False)
        try:
            summary = _run_to_output(args, staging)
            SHARED._verify_staged_artifacts(staging)
            SHARED._publish_staged_output(staging, output)
            return summary
        finally:
            if staging.exists():
                shutil.rmtree(staging)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--receipt-set", type=Path, default=DEFAULT_RECEIPT_SET)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--expected-n", type=int, default=800)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument(
        "--staging-parent",
        type=Path,
        default=None,
        help="parent directory for per-asset staging (default: system temp)",
    )
    parser.add_argument(
        "--skip-archive-binding",
        action="store_true",
        help="skip byte-level re-verification against the official source archive",
    )
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
