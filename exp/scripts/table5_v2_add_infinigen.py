#!/usr/bin/env python3
"""Append an official released-format Infinigen-Sim Core-200 cohort.

The candidate universe is the identity intersection of the official URDF and
MJCF releases.  Genesis and PyBullet consume URDF, while MuJoCo consumes the
official MJCF for the exact same asset identity.  Selection never depends on
parsing or simulator outcomes.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

from table5_n200_manifest import _parse_urdf, canonical_sha256, sha256_file
from table5_v2_sample_n200 import (
    EXTENDED_DATASET_SLUGS,
    INFINIGEN_NAME,
    INFINIGEN_SLUG,
    PROTOCOL_ID,
    SAMPLE_SIZE,
    SELECTION_SEED,
    dataset_eligibility_reason,
    selection_rank,
    validate_manifest,
)


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
DEFAULT_SOURCE_ROOT = REPO_ROOT / "exp/Infinigen-Sim"
DEFAULT_BASE_COHORT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_articraft_github_core200_visual_collision_official/cohort_manifest.json"
)
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_core200_articraft10787_infinigen_paired_official"
)
UNIVERSE_FILENAME = "infinigen_paired_universe.jsonl"


class InfinigenCohortError(RuntimeError):
    """Raised when released sources or the frozen cohort are inconsistent."""


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise InfinigenCohortError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise InfinigenCohortError(f"JSON root is not an object: {path}")
    return value


def _atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (
        json.dumps(value, ensure_ascii=True, allow_nan=False, sort_keys=True, indent=2)
        + "\n"
    )
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _single_source(package: Path, suffix: str) -> Path:
    files = sorted(path for path in package.glob(f"*{suffix}") if path.is_file())
    if len(files) != 1:
        raise InfinigenCohortError(
            f"expected exactly one {suffix} source in {package}, found {len(files)}"
        )
    return files[0].resolve(strict=True)


def _discover(root: Path, representation: str, suffix: str) -> dict[tuple[str, str], Path]:
    representation_root = (root / representation).resolve(strict=True)
    found: dict[tuple[str, str], Path] = {}
    for package in sorted(
        path
        for category in representation_root.iterdir()
        if category.is_dir()
        for path in category.iterdir()
        if path.is_dir()
    ):
        key = (package.parent.name, package.name)
        if key in found:
            raise InfinigenCohortError(f"duplicate {representation} identity: {key}")
        found[key] = _single_source(package, suffix)
    return found


def _urdf_schema(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise InfinigenCohortError(f"URDF root is not robot: {path}")
    links = [str(node.get("name") or "") for node in root.findall("link")]
    joints = []
    for node in root.findall("joint"):
        kind = str(node.get("type") or "").lower()
        if kind == "fixed":
            continue
        joints.append({"name": str(node.get("name") or ""), "type": kind})
    return {"links": links, "joints": joints}


def _mjcf_schema(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    if root.tag != "mujoco":
        raise InfinigenCohortError(f"MJCF root is not mujoco: {path}")
    bodies = [str(node.get("name") or "") for node in root.findall(".//body")]
    joints: list[dict[str, str]] = []
    type_map = {"hinge": "revolute", "slide": "prismatic"}
    for node in root.findall(".//joint"):
        kind = str(node.get("type") or "hinge").lower()
        joints.append(
            {
                "name": str(node.get("name") or ""),
                "type": type_map.get(kind, kind),
            }
        )
    fixed_base = root.find(".//freejoint") is None and all(
        joint.get("type", "hinge").lower() != "free"
        for joint in root.findall(".//joint")
    )
    return {"bodies": bodies, "joints": joints, "fixed_base": fixed_base}


def _paired_receipt(
    item: tuple[tuple[str, str], Path, Path]
) -> dict[str, Any]:
    (category, local_id), urdf, mjcf = item
    urdf_schema = _urdf_schema(urdf)
    mjcf_schema = _mjcf_schema(mjcf)
    urdf_joints = {joint["name"]: joint["type"] for joint in urdf_schema["joints"]}
    mjcf_joints = {joint["name"]: joint["type"] for joint in mjcf_schema["joints"]}
    urdf_links = sorted(name for name in urdf_schema["links"] if name != "world")
    mjcf_bodies = sorted(name for name in mjcf_schema["bodies"] if name != "object")
    return {
        "asset_id": f"{category}/{local_id}",
        "category": category,
        "local_asset_id": local_id,
        "movable_joint_count": len(urdf_joints),
        "urdf_path": str(urdf),
        "urdf_sha256": sha256_file(urdf),
        "urdf_joint_names": sorted(urdf_joints),
        "urdf_link_names_without_world": urdf_links,
        "mjcf_path": str(mjcf),
        "mjcf_sha256": sha256_file(mjcf),
        "mjcf_joint_names": sorted(mjcf_joints),
        "mjcf_body_names_without_object": mjcf_bodies,
        "cross_format_agreement": {
            "joint_names": sorted(urdf_joints) == sorted(mjcf_joints),
            "joint_types": urdf_joints == mjcf_joints,
            "link_body_names": urdf_links == mjcf_bodies,
            "fixed_base": bool(mjcf_schema["fixed_base"]),
        },
    }


def build_paired_universe(
    source_root: Path, destination: Path, *, workers: int
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    urdfs = _discover(source_root, "urdf", ".urdf")
    mjcfs = _discover(source_root, "mjcf", ".xml")
    paired = sorted(set(urdfs) & set(mjcfs))
    if not paired:
        raise InfinigenCohortError("Infinigen URDF/MJCF intersection is empty")
    items = [(key, urdfs[key], mjcfs[key]) for key in paired]
    with ThreadPoolExecutor(max_workers=workers) as executor:
        rows = list(executor.map(_paired_receipt, items))
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(
                json.dumps(
                    row, ensure_ascii=True, allow_nan=False, sort_keys=True, separators=(",", ":")
                )
                + "\n"
            )
    counts = {
        "urdf_identity_count": len(urdfs),
        "mjcf_identity_count": len(mjcfs),
        "paired_identity_count": len(paired),
        "urdf_only_count": len(set(urdfs) - set(mjcfs)),
        "mjcf_only_count": len(set(mjcfs) - set(urdfs)),
        "joint_name_agreement_count": sum(
            row["cross_format_agreement"]["joint_names"] for row in rows
        ),
        "joint_type_agreement_count": sum(
            row["cross_format_agreement"]["joint_types"] for row in rows
        ),
        "link_body_name_agreement_count": sum(
            row["cross_format_agreement"]["link_body_names"] for row in rows
        ),
        "fixed_base_mjcf_count": sum(
            row["cross_format_agreement"]["fixed_base"] for row in rows
        ),
    }
    return rows, counts


def _selected_row(
    raw: Mapping[str, Any], *, order: int, rank: str, universe_hash: str
) -> dict[str, Any]:
    urdf = Path(str(raw["urdf_path"])).resolve(strict=True)
    mjcf = Path(str(raw["mjcf_path"])).resolve(strict=True)
    package = urdf.parent
    parsed = _parse_urdf(package, urdf)
    source_hash = canonical_sha256(raw)
    row: dict[str, Any] = {
        "dataset_slug": INFINIGEN_SLUG,
        "dataset_name": INFINIGEN_NAME,
        "dataset_id": f"infinigen_{order:04d}",
        "asset_id": str(raw["asset_id"]),
        "category": str(raw["category"]),
        "package_root": str(package),
        "urdf_path": str(urdf),
        "urdf_sha256": str(raw["urdf_sha256"]),
        "joint_tree": parsed["joint_tree"],
        "scalar_joints": parsed["scalar_joints"],
        "xml_counts": parsed["xml_counts"],
        "preflight": {
            "status": "passed" if not parsed["issues"] else "failed",
            "issues": parsed["issues"],
        },
        "simulator_sources": {
            "genesis": {
                "format": "urdf",
                "representation": "official_released_urdf",
                "path": str(urdf),
                "sha256": str(raw["urdf_sha256"]),
                "package_root": str(package),
            },
            "pybullet": {
                "format": "urdf",
                "representation": "official_released_urdf",
                "path": str(urdf),
                "sha256": str(raw["urdf_sha256"]),
                "package_root": str(package),
            },
            "mujoco": {
                "format": "mjcf",
                "representation": "official_released_mjcf",
                "path": str(mjcf),
                "sha256": str(raw["mjcf_sha256"]),
                "package_root": str(mjcf.parent),
            },
        },
        "source_provenance": {
            "paired_universe_sha256": universe_hash,
            "paired_universe_row_sha256": source_hash,
            "cross_format_agreement": deepcopy(raw["cross_format_agreement"]),
        },
        "cohort_selection": {
            "protocol_id": PROTOCOL_ID,
            "seed": SELECTION_SEED,
            "selection_order": order,
            "rank_sha256": rank,
            "source_row_sha256": source_hash,
        },
    }
    row["row_sha256"] = canonical_sha256(row, exclude_fields=("row_sha256",))
    return row


def _select(
    universe: Sequence[Mapping[str, Any]], *, universe_hash: str
) -> tuple[list[dict[str, Any]], Counter[str], int]:
    eligible: list[tuple[str, str, Mapping[str, Any]]] = []
    excluded: Counter[str] = Counter()
    for raw in universe:
        asset_id = str(raw["asset_id"])
        reason = dataset_eligibility_reason(
            INFINIGEN_SLUG,
            raw.get("category"),
            asset_id,
            raw.get("movable_joint_count"),
        )
        rank = selection_rank(INFINIGEN_SLUG, asset_id, universe_sha256=universe_hash)
        if reason is not None:
            excluded[reason] += 1
        else:
            eligible.append((rank, asset_id, raw))
    eligible.sort(key=lambda item: (item[0], item[1]))
    if len(eligible) < SAMPLE_SIZE:
        raise InfinigenCohortError(
            f"only {len(eligible)} eligible paired assets; {SAMPLE_SIZE} required"
        )
    selected = [
        _selected_row(raw, order=order, rank=rank, universe_hash=universe_hash)
        for order, (rank, _asset_id, raw) in enumerate(eligible[:SAMPLE_SIZE])
    ]
    return selected, excluded, len(eligible)


def build(
    base_cohort_path: Path,
    source_root: Path,
    output: Path,
    *,
    workers: int,
) -> dict[str, Any]:
    output = output.resolve(strict=False)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.work.", dir=output.parent))
    try:
        universe_path = staging / UNIVERSE_FILENAME
        universe, universe_counts = build_paired_universe(
            source_root.resolve(strict=True), universe_path, workers=workers
        )
        universe_hash = sha256_file(universe_path)
        selected, excluded, eligible_count = _select(
            universe, universe_hash=universe_hash
        )
        base = _read_json(base_cohort_path.resolve(strict=True))
        validate_manifest(base)
        datasets = deepcopy(base["datasets"])
        if [dataset["dataset_slug"] for dataset in datasets] != list(
            EXTENDED_DATASET_SLUGS[:6] + (EXTENDED_DATASET_SLUGS[-1],)
        ):
            raise InfinigenCohortError("base cohort is not the expected seven-dataset cohort")
        infinigen_dataset = {
            "dataset_slug": INFINIGEN_SLUG,
            "dataset_name": INFINIGEN_NAME,
            "universe": {
                "kind": "official_released_urdf_mjcf_identity_intersection",
                "path": str(output / UNIVERSE_FILENAME),
                "sha256": universe_hash,
                "candidate_count": len(universe),
                **universe_counts,
            },
            "selection": {
                "eligible_count": eligible_count,
                "excluded_count": sum(excluded.values()),
                "exclusion_counts": dict(sorted(excluded.items())),
                "selected_count": len(selected),
                "ordered_asset_ids_sha256": canonical_sha256(
                    [row["asset_id"] for row in selected]
                ),
                "ordered_ranks_sha256": canonical_sha256(
                    [row["cohort_selection"]["rank_sha256"] for row in selected]
                ),
            },
            "rows": selected,
        }
        datasets.insert(len(datasets) - 1, infinigen_dataset)
        protocol = deepcopy(base["protocol"])
        protocol["baseline_candidate_universe"] = (
            "Articraft-10K uses its frozen 10,787-record merged roster; "
            "Infinigen-Sim uses the full official URDF/MJCF identity intersection; "
            "the other baselines use their bound N=800 parent rosters"
        )
        protocol["infinigen_candidate_universe"] = {
            "kind": "official_released_urdf_mjcf_identity_intersection",
            "candidate_count": len(universe),
            "paired_universe_sha256": universe_hash,
            **universe_counts,
        }
        protocol["simulator_source_policy"] = {
            "default": "released URDF through each simulator's importer",
            "infinigen_sim": {
                "genesis": "official released URDF",
                "pybullet": "official released URDF",
                "mujoco": "official released MJCF",
            },
            "native_format_exception": (
                "when a dataset officially releases the same asset identity in the target "
                "simulator's native format, use that released representation"
            ),
            "canonical_metric_schema": "released URDF joint/link schema",
            "claim_boundary": (
                "released cross-simulator readiness, not a same-URDF importer comparison"
            ),
        }
        protocol["protocol_sha256"] = canonical_sha256(
            protocol, exclude_fields=("protocol_sha256",)
        )
        manifest: dict[str, Any] = {
            "schema_version": base["schema_version"],
            "protocol": protocol,
            "protocol_sha256": protocol["protocol_sha256"],
            "sample_size_per_dataset": SAMPLE_SIZE,
            "dataset_count": len(datasets),
            "total_rows": SAMPLE_SIZE * len(datasets),
            "ordered_dataset_slugs": list(EXTENDED_DATASET_SLUGS),
            "datasets": datasets,
        }
        manifest["cohort_sha256"] = canonical_sha256(
            [
                {
                    "dataset_slug": dataset["dataset_slug"],
                    "universe_sha256": dataset["universe"]["sha256"],
                    "ordered_asset_ids_sha256": dataset["selection"][
                        "ordered_asset_ids_sha256"
                    ],
                    "ordered_ranks_sha256": dataset["selection"][
                        "ordered_ranks_sha256"
                    ],
                }
                for dataset in datasets
            ]
        )
        manifest["manifest_sha256"] = canonical_sha256(
            manifest, exclude_fields=("manifest_sha256",)
        )
        validate_manifest(manifest)
        _atomic_json(staging / "cohort_manifest.json", manifest)
        _atomic_json(
            staging / "selection.json",
            {
                "schema_version": "table5_v2_infinigen_paired_selection_v1",
                "source_root": str(source_root.resolve(strict=True)),
                "paired_universe_path": str(output / UNIVERSE_FILENAME),
                "paired_universe_sha256": universe_hash,
                **universe_counts,
                "eligible_count": eligible_count,
                "excluded_count": sum(excluded.values()),
                "exclusion_counts": dict(sorted(excluded.items())),
                "selected_count": len(selected),
                "selected": [
                    {
                        "selection_order": row["cohort_selection"]["selection_order"],
                        "asset_id": row["asset_id"],
                        "category": row["category"],
                        "movable_joint_count": row["xml_counts"]["movable_joints"],
                        "rank_sha256": row["cohort_selection"]["rank_sha256"],
                        "urdf_path": row["simulator_sources"]["genesis"]["path"],
                        "urdf_sha256": row["simulator_sources"]["genesis"]["sha256"],
                        "mjcf_path": row["simulator_sources"]["mujoco"]["path"],
                        "mjcf_sha256": row["simulator_sources"]["mujoco"]["sha256"],
                    }
                    for row in selected
                ],
            },
        )
        os.replace(staging, output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return manifest


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-cohort", type=Path, default=DEFAULT_BASE_COHORT)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=16)
    args = parser.parse_args(argv)
    if args.workers < 1:
        parser.error("--workers must be positive")
    try:
        manifest = build(
            args.base_cohort, args.source_root, args.out, workers=args.workers
        )
    except (FileExistsError, InfinigenCohortError, OSError, ET.ParseError) as error:
        parser.error(str(error))
    print(
        json.dumps(
            {
                "cohort": str((args.out / "cohort_manifest.json").resolve()),
                "cohort_sha256": manifest["cohort_sha256"],
                "manifest_sha256": manifest["manifest_sha256"],
                "dataset_count": manifest["dataset_count"],
                "total_rows": manifest["total_rows"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
