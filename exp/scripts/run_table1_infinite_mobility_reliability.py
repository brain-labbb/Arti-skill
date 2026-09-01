#!/usr/bin/env python3
"""Audit Infinite Mobility for Nano3D Table 1B without regenerating assets.

The frozen generation run used a 300-second timeout.  Table 1B uses the same
180-second per-seed budget as PV-A, so this audit deterministically reclassifies
the original per-case telemetry: a recorded PASS taking more than 180 seconds
is a failure.  This is a telemetry sensitivity analysis, not a fresh 180-second
rerun.

The original runner's gate is deliberately called structural package QC here.
It checks process exit, URDF/tree validity, and mesh references.  It does not
implement the PV-A Full-QC contract, and the exported packages have no native
collision elements.  Full-QC-derived cells therefore fail closed as N/E.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import os
import platform
import re
import statistics
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
EXP_ROOT = REPO_ROOT / "exp"
SOURCE_ROOT = EXP_ROOT / "runtime/infinite_mobility_v1"
RECOVERY_ROOT = EXP_ROOT / "runtime/infinite_mobility_timeout_recovery_v1"
PROTOCOL_PATH = EXP_ROOT / "reference/infinite_mobility_protocol_v1.json"
MATCHED_PATH = EXP_ROOT / "reference/infinite_mobility_hierarchy_common_categories_v1.json"
OFFICIAL_ROOT = EXP_ROOT / "baselines/Infinite-Mobility-official"
CACHE_ROOT = REPO_ROOT / ".cache/Infinite-Mobility"
BLENDER_PATH = REPO_ROOT / ".cache/blender-3.6.0-linux-x64/blender"
GENERATION_RUNNER = EXP_ROOT / "scripts/run_infinite_mobility_baseline.py"
BLENDER_WORKER = EXP_ROOT / "scripts/infinite_mobility_blender_worker.py"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/table1_reliability/infinite_mobility"

MAIN_TIMEOUT_SECONDS = 180.0
ORIGINAL_TIMEOUT_SECONDS = 300.0
RECOVERY_TIMEOUT_SECONDS = 900.0
EXPECTED_OFFICIAL_REMOTE = "https://github.com/InternRobotics/Infinite-Mobility.git"
EXPECTED_OFFICIAL_COMMIT = "5f5961736fcf5b7a6e6fd0a9b3b7af586f39e151"
EXPECTED_OFFICIAL_TREE = "e4dbd72b1d6e38a5c13f8a9f0130385b4c98524f"

README_FACTORY_RE = re.compile(r"^\|\s*([A-Za-z0-9_]+Factory)\s*\|", re.MULTILINE)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_json(payload: Any) -> str:
    encoded = json.dumps(
        payload, ensure_ascii=True, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def dump_json(path: Path, payload: Any) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def source_tree_sha256(root: Path) -> str:
    digest = hashlib.sha256()
    files = sorted(
        path for path in root.rglob("*.py") if "__pycache__" not in path.parts
    )
    for path in files:
        relative = path.relative_to(root).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def git_output(root: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(root), *args], text=True
    ).strip()


def git_tracked_paths(root: Path) -> list[str]:
    raw = subprocess.check_output(
        ["git", "-C", str(root), "ls-files", "-z"]
    )
    return [item.decode("utf-8") for item in raw.split(b"\0") if item]


def tracked_snapshot_sha256(root: Path, paths: Iterable[str]) -> tuple[str, int, int]:
    digest = hashlib.sha256()
    file_count = 0
    byte_count = 0
    for name in paths:
        path = root / name
        if not path.is_file():
            raise FileNotFoundError(path)
        encoded = name.encode("utf-8")
        size = path.stat().st_size
        digest.update(len(encoded).to_bytes(8, "big"))
        digest.update(encoded)
        digest.update(size.to_bytes(8, "big"))
        digest.update(bytes.fromhex(sha256_file(path)))
        file_count += 1
        byte_count += size
    return digest.hexdigest(), file_count, byte_count


def package_sha256(package_dir: Path) -> str:
    digest = hashlib.sha256()
    files = sorted(path for path in package_dir.rglob("*") if path.is_file())
    for path in files:
        if path.name in {"stdout.log", "stderr.log", "record.json"}:
            continue
        relative = path.relative_to(package_dir).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def resolve_mesh_path(package_dir: Path, urdf_path: Path, filename: str) -> Path:
    candidate = Path(filename)
    if candidate.is_absolute():
        return candidate
    urdf_relative = urdf_path.parent / candidate
    if urdf_relative.exists():
        return urdf_relative
    return REPO_ROOT / candidate


def validate_structural_package(package_dir: Path) -> dict[str, Any]:
    result: dict[str, Any] = {
        "urdf_count": 0,
        "valid_urdf": False,
        "valid_tree": False,
        "mesh_references_valid": False,
        "portable_relative_paths": False,
        "link_count": 0,
        "joint_count": 0,
        "movable_joint_count": 0,
        "visual_count": 0,
        "collision_count": 0,
        "mesh_reference_count": 0,
        "errors": [],
    }
    urdfs = sorted(package_dir.rglob("scene.urdf")) if package_dir.is_dir() else []
    result["urdf_count"] = len(urdfs)
    if len(urdfs) != 1:
        result["errors"].append(f"expected one scene.urdf, found {len(urdfs)}")
        result["structural_package_pass"] = False
        return result

    urdf_path = urdfs[0]
    result["urdf_path"] = urdf_path.relative_to(package_dir).as_posix()
    try:
        robot = ET.parse(urdf_path).getroot()
    except (ET.ParseError, OSError) as exc:
        result["errors"].append(f"URDF parse failed: {exc}")
        result["structural_package_pass"] = False
        return result
    if robot.tag != "robot":
        result["errors"].append(f"root tag is {robot.tag!r}, expected 'robot'")
        result["structural_package_pass"] = False
        return result

    links = [node.get("name", "") for node in robot.findall("link")]
    joints = robot.findall("joint")
    result["link_count"] = len(links)
    result["joint_count"] = len(joints)
    result["movable_joint_count"] = sum(
        node.get("type") not in {None, "fixed"} for node in joints
    )
    result["visual_count"] = len(robot.findall(".//visual"))
    result["collision_count"] = len(robot.findall(".//collision"))
    if links and all(links) and len(set(links)) == len(links):
        result["valid_urdf"] = True
    else:
        result["errors"].append("links are empty, unnamed, or duplicated")

    children: dict[str, list[str]] = {name: [] for name in links}
    indegree: Counter[str] = Counter()
    endpoint_error = False
    for joint in joints:
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent_name = parent_node.get("link", "") if parent_node is not None else ""
        child_name = child_node.get("link", "") if child_node is not None else ""
        if parent_name not in children or child_name not in children:
            endpoint_error = True
            result["errors"].append(
                f"joint {joint.get('name', '')!r} has an unknown endpoint"
            )
            continue
        children[parent_name].append(child_name)
        indegree[child_name] += 1

    roots = [name for name in links if indegree[name] == 0]
    if not endpoint_error and len(roots) == 1 and all(indegree[name] <= 1 for name in links):
        visited: set[str] = set()
        active: set[str] = set()

        def walk(node: str) -> bool:
            if node in active:
                return False
            if node in visited:
                return True
            active.add(node)
            for child in children[node]:
                if not walk(child):
                    return False
            active.remove(node)
            visited.add(node)
            return True

        result["valid_tree"] = walk(roots[0]) and len(visited) == len(links)
    if not result["valid_tree"]:
        result["errors"].append(
            f"link graph is not a single-root connected tree (roots={len(roots)})"
        )

    mesh_nodes = robot.findall(".//mesh")
    result["mesh_reference_count"] = len(mesh_nodes)
    mesh_errors: list[str] = []
    relative_only = True
    for node in mesh_nodes:
        filename = node.get("filename", "")
        relative_only &= bool(filename) and not Path(filename).is_absolute()
        mesh_path = (
            resolve_mesh_path(package_dir, urdf_path, filename)
            if filename
            else urdf_path
        )
        try:
            mesh_path.resolve().relative_to(package_dir.resolve())
        except ValueError:
            mesh_errors.append(f"mesh escapes package directory: {filename}")
            continue
        if not mesh_path.is_file() or mesh_path.stat().st_size == 0:
            mesh_errors.append(f"missing or empty mesh: {filename}")
    result["mesh_references_valid"] = bool(mesh_nodes) and not mesh_errors
    result["portable_relative_paths"] = relative_only and bool(mesh_nodes)
    result["errors"].extend(mesh_errors[:20])
    result["structural_package_pass"] = all(
        result[key] for key in ("valid_urdf", "valid_tree", "mesh_references_valid")
    )
    return result


def audit_case(
    source_root: Path,
    record: dict[str, Any],
    rehash: bool,
) -> dict[str, Any]:
    factory = record["factory"]
    seed = int(record["seed"])
    package_dir = source_root / "cases" / factory / f"seed_{seed:03d}" / "package"
    structural = validate_structural_package(package_dir)
    expected_hash = record.get("package_sha256")
    actual_hash = None
    hash_matches = None
    if expected_hash and package_dir.is_dir() and rehash:
        actual_hash = package_sha256(package_dir)
        hash_matches = actual_hash == expected_hash
    elif expected_hash:
        hash_matches = None

    recorded_validation = record.get("validation") or {}
    structural_fields = (
        "valid_urdf",
        "valid_tree",
        "mesh_references_valid",
        "portable_relative_paths",
        "link_count",
        "joint_count",
        "movable_joint_count",
        "visual_count",
        "collision_count",
        "mesh_reference_count",
    )
    validation_matches = all(
        structural.get(key) == recorded_validation.get(key) for key in structural_fields
    )
    original_pass = record.get("status") == "PASS"
    structural_verified = (
        original_pass
        and recorded_validation.get("strict_pass") is True
        and structural.get("structural_package_pass") is True
        and validation_matches
        and (hash_matches is not False)
    )
    elapsed = float(record.get("elapsed_seconds", 0.0))
    compile_180 = structural_verified and elapsed <= MAIN_TIMEOUT_SECONDS

    if compile_180:
        failure_reason = None
    elif record.get("status") != "PASS":
        failure_reason = f"original_{int(ORIGINAL_TIMEOUT_SECONDS)}s_{str(record.get('status')).lower()}"
    elif elapsed > MAIN_TIMEOUT_SECONDS:
        failure_reason = "recorded_elapsed_exceeds_180s"
    elif not structural.get("structural_package_pass"):
        failure_reason = "structural_package_reaudit_failed"
    elif not validation_matches:
        failure_reason = "recorded_validation_mismatch"
    elif hash_matches is False:
        failure_reason = "package_hash_mismatch"
    else:
        failure_reason = "record_not_strict_pass"

    return {
        "identity": f"{factory}__seed_{seed:03d}",
        "factory": factory,
        "seed": seed,
        "elapsed_seconds": elapsed,
        "original_status": record.get("status"),
        "original_300s_structural_package_qc_pass": structural_verified,
        "main_180s_seed_compile_pass": compile_180,
        "main_180s_failure_reason": failure_reason,
        "recorded_package_sha256": expected_hash,
        "recomputed_package_sha256": actual_hash,
        "package_hash_matches": hash_matches,
        "recorded_validation_matches_reaudit": validation_matches,
        "reaudit": structural,
    }


def audit_recovery_case(case: dict[str, Any], rehash: bool) -> dict[str, Any]:
    record_path = REPO_ROOT / case["recovery_record"]
    record = json.loads(record_path.read_text(encoding="utf-8"))
    package_dir = record_path.parent / "package"
    structural = validate_structural_package(package_dir)
    actual_hash = package_sha256(package_dir) if rehash else None
    expected_hash = case["package_sha256"]
    return {
        "identity": f"{case['factory']}__seed_{int(case['seed']):03d}",
        "factory": case["factory"],
        "seed": int(case["seed"]),
        "status": record.get("status"),
        "elapsed_seconds": float(record.get("elapsed_seconds", 0.0)),
        "structural_package_pass": structural.get("structural_package_pass") is True,
        "recorded_package_sha256": expected_hash,
        "recomputed_package_sha256": actual_hash,
        "package_hash_matches": None if not rehash else actual_hash == expected_hash,
        "record_sha256_matches": sha256_file(record_path) == case["recovery_record_sha256"],
        "reaudit": structural,
    }


def metric(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": numerator,
        "denominator": denominator,
        "value": numerator / denominator if denominator else None,
    }


def state_metric(state: str, display: str, reason: str, **extra: Any) -> dict[str, Any]:
    result: dict[str, Any] = {
        "state": state,
        "display": display,
        "numerator": None,
        "denominator": None,
        "value": None,
        "reason": reason,
    }
    result.update(extra)
    return result


def summarize_cohort(outcomes: list[dict[str, Any]], factory_order: list[str]) -> dict[str, Any]:
    by_factory: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in outcomes:
        by_factory[row["factory"]].append(row)

    factory_records: list[dict[str, Any]] = []
    for factory in factory_order:
        rows = sorted(by_factory[factory], key=lambda item: item["seed"])
        compile_180 = sum(row["main_180s_seed_compile_pass"] for row in rows)
        structural_300 = sum(
            row["original_300s_structural_package_qc_pass"] for row in rows
        )
        factory_records.append(
            {
                "factory": factory,
                "requested": len(rows),
                "seed_compile_180s": compile_180,
                "all_36_seed_compile_180s": len(rows) == 36 and compile_180 == 36,
                "structural_package_qc_300s": structural_300,
                "all_36_structural_package_qc_300s": len(rows) == 36
                and structural_300 == 36,
                "failed_seeds_180s": [
                    row["seed"] for row in rows if not row["main_180s_seed_compile_pass"]
                ],
                "failed_seeds_300s": [
                    row["seed"]
                    for row in rows
                    if not row["original_300s_structural_package_qc_pass"]
                ],
            }
        )

    compile_180 = sum(row["main_180s_seed_compile_pass"] for row in outcomes)
    structural_300 = sum(
        row["original_300s_structural_package_qc_pass"] for row in outcomes
    )
    return {
        "factory_count": len(factory_order),
        "seeds_per_factory": 36,
        "requested_cases": len(outcomes),
        "seed_compile_180s": metric(compile_180, len(outcomes)),
        "compile_only_all_36_180s": metric(
            sum(row["all_36_seed_compile_180s"] for row in factory_records),
            len(factory_records),
        ),
        "structural_package_qc_300s": metric(structural_300, len(outcomes)),
        "structural_package_qc_all_36_300s": metric(
            sum(row["all_36_structural_package_qc_300s"] for row in factory_records),
            len(factory_records),
        ),
        "failure_status_counts_180s": dict(
            sorted(
                Counter(
                    row["main_180s_failure_reason"]
                    for row in outcomes
                    if not row["main_180s_seed_compile_pass"]
                ).items()
            )
        ),
        "failures_180s": [
            {
                "identity": row["identity"],
                "factory": row["factory"],
                "seed": row["seed"],
                "original_status": row["original_status"],
                "elapsed_seconds": row["elapsed_seconds"],
                "reason": row["main_180s_failure_reason"],
            }
            for row in outcomes
            if not row["main_180s_seed_compile_pass"]
        ],
        "factory_records": factory_records,
    }


def make_manifest(
    protocol: dict[str, Any],
    matched: dict[str, Any],
    input_hashes: dict[str, str],
) -> dict[str, Any]:
    factories = list(protocol["factories"])
    seeds = list(protocol["seeds"])
    matched_categories = [
        {
            "category_id": row["category_id"],
            "common_category": row["common_category"],
            "factory": row["factory"],
            "selection_reason": row["selection_reason"],
        }
        for row in matched["categories"]
    ]
    matched_factories = [row["factory"] for row in matched_categories]
    return {
        "schema_version": 1,
        "protocol_id": "nano3d_table1b_infinite_mobility_reliability_v1",
        "method": "Infinite Mobility",
        "evaluation_unit": "one public factory at one integer seed",
        "public_main_cohort": {
            "scope": "all factories listed in the pinned official README",
            "factory_count": len(factories),
            "factories": factories,
            "seeds_per_factory": len(seeds),
            "seeds": seeds,
            "requested_count": len(factories) * len(seeds),
            "requested_identities": [
                f"{factory}__seed_{seed:03d}" for factory in factories for seed in seeds
            ],
        },
        "matched_supplementary_cohort": {
            "scope": "five common categories frozen by the existing hierarchy protocol",
            "categories": matched_categories,
            "factory_count": len(matched_factories),
            "factories": matched_factories,
            "seeds_per_factory": len(seeds),
            "seeds": seeds,
            "requested_count": len(matched_factories) * len(seeds),
            "requested_identities": [
                f"{factory}__seed_{seed:03d}"
                for factory in matched_factories
                for seed in seeds
            ],
        },
        "timeout_contract": {
            "main_seconds": MAIN_TIMEOUT_SECONDS,
            "main_policy": (
                "deterministic reclassification of original per-case telemetry; "
                "recorded PASS must also have elapsed_seconds <= 180"
            ),
            "fresh_180s_rerun": False,
            "original_generation_seconds": ORIGINAL_TIMEOUT_SECONDS,
            "recovery_availability_seconds": RECOVERY_TIMEOUT_SECONDS,
            "recovery_counts_for_main_reliability": False,
        },
        "metric_contract": {
            "seed_compile": (
                "process exited successfully within the timeout and saved one parseable, "
                "nonempty URDF/tree/mesh package whose frozen hash revalidates"
            ),
            "seed_full_qc": {
                "state": "not_evaluable",
                "display": "N/E",
                "required": [
                    "author tests",
                    "compiler baseline policies",
                    "overlap policy",
                    "rigid/disconnected policy",
                    "motion-QC coverage",
                ],
                "reason": (
                    "the baseline adapter did not execute these checks and exported zero "
                    "native collision elements"
                ),
            },
            "all_36_pass": (
                "Table 1B headline is Full-QC-derived and is N/E; an explicitly labeled "
                "compile-only 36/36 diagnostic is reported separately"
            ),
            "corner_pass": {
                "state": "not_reported_under_exact_protocol",
                "display": "N/R",
                "reason": (
                    "no output-independent factory parameter-domain manifest or semantic "
                    "mapping to PV-A authored/legacy corners is available"
                ),
            },
            "regression_retention": {
                "state": "not_evaluable",
                "display": "N/E",
                "reason": "no frozen pre/post-repair factory pair and retained-seed protocol",
            },
        },
        "input_hashes": input_hashes,
    }


def make_report(summary: dict[str, Any], provenance: dict[str, Any]) -> str:
    public = summary["public_main_cohort"]
    matched = summary["matched_supplementary_cohort"]
    main_compile = public["seed_compile_180s"]
    main_all = public["compile_only_all_36_180s"]
    matched_compile = matched["seed_compile_180s"]
    matched_all = matched["compile_only_all_36_180s"]
    structural = public["structural_package_qc_300s"]
    structural_all = public["structural_package_qc_all_36_300s"]
    recovery = summary["recovery_900s_availability"]

    lines = [
        "# Table 1B: Infinite Mobility reliability audit",
        "",
        f"- Status: `{summary['status']}`",
        "- Main cohort: 20 pinned public factories x seeds 0-35 = 720 requested cases.",
        "- Main timeout: 180 s/seed, aligned to PV-A.",
        "- Main result is a deterministic reclassification of real per-case telemetry, not a fresh 180 s rerun.",
        "- Full QC and Full-QC-derived 36/36 are `N/E`; Corner Pass is `N/R`.",
        "",
        "## Candidate Table 1B row",
        "",
        "| Method | Templates | Seeds / Template | Total Seeds | Seed Compile (180 s) | Seed Full QC | 36/36 Pass | Corner Pass | Regression Retention |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        (
            "| Infinite Mobility (public factory manifest; telemetry sensitivity) "
            f"| 20 | 36 (0-35) | 720 | {main_compile['numerator']}/{main_compile['denominator']} = "
            f"{100 * main_compile['value']:.1f}% | N/E | N/E | N/R | N/E |"
        ),
        "",
        (
            f"Compile-only diagnostic: {main_all['numerator']}/{main_all['denominator']} = "
            f"{100 * main_all['value']:.1f}% factories pass all 36 seeds at the 180 s telemetry threshold. "
            "This is not the headline Full-QC `36/36 Pass` cell."
        ),
        "",
        "## Denominators and sensitivity",
        "",
        "| Scope | Timeout/evidence | Seed/package pass | Compile-only all-seed factories | Interpretation |",
        "|---|---:|---:|---:|---|",
        (
            f"| Public 20-factory main | 180 s telemetry threshold | {main_compile['numerator']}/{main_compile['denominator']} "
            f"({100 * main_compile['value']:.1f}%) | {main_all['numerator']}/{main_all['denominator']} "
            f"({100 * main_all['value']:.1f}%) | Table 1B aligned timeout; not a fresh rerun |"
        ),
        (
            f"| Public 20-factory original | 300 s actual run | {structural['numerator']}/{structural['denominator']} "
            f"({100 * structural['value']:.1f}%) | {structural_all['numerator']}/{structural_all['denominator']} "
            f"({100 * structural_all['value']:.1f}%) | Structural package QC only |"
        ),
        (
            f"| Public availability overlay | 900 s recovery of original timeouts | "
            f"{recovery['available_packages']['numerator']}/{recovery['available_packages']['denominator']} "
            f"({100 * recovery['available_packages']['value']:.1f}%) | 20/20 package availability | "
            "Artifact coverage sensitivity only; excluded from main reliability |"
        ),
        (
            f"| Matched five-category supplement | 180 s telemetry threshold | "
            f"{matched_compile['numerator']}/{matched_compile['denominator']} "
            f"({100 * matched_compile['value']:.1f}%) | {matched_all['numerator']}/{matched_all['denominator']} "
            f"({100 * matched_all['value']:.1f}%) | Five frozen common-category factories |"
        ),
        "",
        "The 300 s gate verifies process exit, one parseable URDF, a connected acyclic tree, and existing nonempty mesh references. It does not run author tests, PV-A compiler policies, overlap checks, or motion-QC. Across the 713 original PASS packages the re-audit found zero native collision elements, so collision-free or motion-QC claims cannot be made from this export.",
        "",
        "## 180-second failures",
        "",
        "| Factory | Seed | Original status | Recorded elapsed (s) | Main reason |",
        "|---|---:|---|---:|---|",
    ]
    for row in public["failures_180s"]:
        lines.append(
            f"| {row['factory']} | {row['seed']} | {row['original_status']} | "
            f"{row['elapsed_seconds']:.3f} | {row['reason']} |"
        )

    lines.extend(
        [
            "",
            "## Provenance and integrity",
            "",
            f"- Official repository: `{provenance['official_release']['remote']}`",
            f"- Official commit: `{provenance['official_release']['commit']}`",
            f"- Official Git tree: `{provenance['official_release']['tree']}`",
            f"- Actual generation snapshot Python-tree SHA-256: `{provenance['generation_snapshot']['python_tree_sha256']}`",
            f"- Official snapshot Python-tree SHA-256: `{provenance['official_release']['python_tree_sha256']}`",
            f"- Release-equivalent tracked snapshot: `{provenance['generation_snapshot']['release_equivalent_to_official_tracked_files']}`",
            f"- Source generation manifest SHA-256: `{provenance['inputs']['generation_manifest']['sha256']}`",
            f"- Source records SHA-256: `{provenance['inputs']['generation_records']['sha256']}`",
            f"- Frozen public protocol SHA-256: `{provenance['inputs']['public_protocol']['sha256']}`",
            f"- Matched-category protocol SHA-256: `{provenance['inputs']['matched_protocol']['sha256']}`",
            f"- Blender binary SHA-256: `{provenance['runtime']['blender_sha256']}`",
            f"- Package hash re-audit: {summary['integrity']['original_pass_package_hash_matches']['numerator']}/{summary['integrity']['original_pass_package_hash_matches']['denominator']}",
            f"- Structural package re-audit: {summary['integrity']['original_pass_structural_reaudit']['numerator']}/{summary['integrity']['original_pass_structural_reaudit']['denominator']}",
            f"- Self-check: {summary['self_check_passed']}/{summary['self_check_total']} PASS",
            "",
            "The actual generation cache is a snapshot inside the parent workspace repository, not an independent Infinite Mobility Git checkout. The official commit is attributed only because all 6,444 official tracked files and the Python source tree match by content; both pins are retained separately.",
            "",
            "## Commands",
            "",
            "```bash",
            provenance["command"],
            "sha256sum exp/runtime/table1_reliability/infinite_mobility/{manifest.json,provenance.json,case_outcomes.jsonl,summary.json,self_check.json,report.md}",
            "```",
            "",
            "Artifact SHA-256 values are in `artifact_hashes.json`.",
        ]
    )
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--hash-workers", type=int, default=4)
    parser.add_argument(
        "--skip-package-rehash",
        action="store_true",
        help="Debug only: skip 19 GB package content rehash; final audit should not use this",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    try:
        output.relative_to((EXP_ROOT / "runtime").resolve())
    except ValueError as exc:
        raise SystemExit("--output must be inside exp/runtime") from exc
    if args.hash_workers < 1:
        raise SystemExit("--hash-workers must be positive")
    output.mkdir(parents=True, exist_ok=True)

    required_files = [
        PROTOCOL_PATH,
        MATCHED_PATH,
        SOURCE_ROOT / "manifest.json",
        SOURCE_ROOT / "records.json",
        RECOVERY_ROOT / "recovery_manifest.json",
        GENERATION_RUNNER,
        BLENDER_WORKER,
        BLENDER_PATH,
        OFFICIAL_ROOT / "README.md",
        OFFICIAL_ROOT / "LICENSE",
    ]
    for path in required_files:
        if not path.is_file():
            raise FileNotFoundError(path)
    if not CACHE_ROOT.is_dir() or not OFFICIAL_ROOT.is_dir():
        raise FileNotFoundError("Infinite Mobility source snapshot missing")

    protocol = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
    matched_protocol = json.loads(MATCHED_PATH.read_text(encoding="utf-8"))
    generation_manifest = json.loads(
        (SOURCE_ROOT / "manifest.json").read_text(encoding="utf-8")
    )
    records = json.loads((SOURCE_ROOT / "records.json").read_text(encoding="utf-8"))
    recovery_manifest = json.loads(
        (RECOVERY_ROOT / "recovery_manifest.json").read_text(encoding="utf-8")
    )

    input_files = {
        "audit_runner": Path(__file__).resolve(),
        "public_protocol": PROTOCOL_PATH,
        "matched_protocol": MATCHED_PATH,
        "generation_manifest": SOURCE_ROOT / "manifest.json",
        "generation_records": SOURCE_ROOT / "records.json",
        "recovery_manifest": RECOVERY_ROOT / "recovery_manifest.json",
        "generation_runner": GENERATION_RUNNER,
        "blender_worker": BLENDER_WORKER,
    }
    input_hashes = {name: sha256_file(path) for name, path in input_files.items()}
    manifest = make_manifest(protocol, matched_protocol, input_hashes)
    dump_json(output / "manifest.json", manifest)

    official_toplevel = Path(git_output(OFFICIAL_ROOT, "rev-parse", "--show-toplevel"))
    official_commit = git_output(OFFICIAL_ROOT, "rev-parse", "HEAD")
    official_tree = git_output(OFFICIAL_ROOT, "rev-parse", "HEAD^{tree}")
    remote = git_output(OFFICIAL_ROOT, "remote", "get-url", "origin")
    tracked_paths = git_tracked_paths(OFFICIAL_ROOT)
    official_snapshot_hash, tracked_count, tracked_bytes = tracked_snapshot_sha256(
        OFFICIAL_ROOT, tracked_paths
    )
    cache_snapshot_hash, cache_tracked_count, cache_tracked_bytes = tracked_snapshot_sha256(
        CACHE_ROOT, tracked_paths
    )
    official_python_hash = source_tree_sha256(OFFICIAL_ROOT)
    cache_python_hash = source_tree_sha256(CACHE_ROOT)

    run_command = (
        "python exp/scripts/run_table1_infinite_mobility_reliability.py "
        "--output exp/runtime/table1_reliability/infinite_mobility --hash-workers "
        f"{args.hash_workers}"
    )
    provenance = {
        "schema_version": 1,
        "generated_at": utc_now(),
        "command": run_command,
        "official_release": {
            "path": rel(OFFICIAL_ROOT),
            "is_independent_git_checkout": official_toplevel.resolve()
            == OFFICIAL_ROOT.resolve(),
            "remote": remote,
            "commit": official_commit,
            "tree": official_tree,
            "license": "BSD-3-Clause",
            "readme_sha256": sha256_file(OFFICIAL_ROOT / "README.md"),
            "license_sha256": sha256_file(OFFICIAL_ROOT / "LICENSE"),
            "python_tree_sha256": official_python_hash,
            "tracked_snapshot_sha256": official_snapshot_hash,
            "tracked_file_count": tracked_count,
            "tracked_file_bytes": tracked_bytes,
        },
        "generation_snapshot": {
            "path": rel(CACHE_ROOT),
            "is_independent_git_checkout": False,
            "git_commit_attribution": None,
            "git_commit_attribution_reason": (
                "cache directory has no repository boundary; pin content separately"
            ),
            "python_tree_sha256": cache_python_hash,
            "recorded_python_tree_sha256": generation_manifest[
                "baseline_source_tree_sha256"
            ],
            "official_tracked_snapshot_sha256_recomputed_from_cache": cache_snapshot_hash,
            "official_tracked_file_count_found": cache_tracked_count,
            "official_tracked_file_bytes_found": cache_tracked_bytes,
            "release_equivalent_to_official_tracked_files": (
                cache_snapshot_hash == official_snapshot_hash
                and cache_tracked_count == tracked_count
                and cache_tracked_bytes == tracked_bytes
                and cache_python_hash == official_python_hash
            ),
            "excluded_runtime_state": [".nano3d_runs", "__pycache__"],
        },
        "inputs": {
            name: {"path": rel(path), "sha256": input_hashes[name]}
            for name, path in input_files.items()
        },
        "runtime": {
            "python": sys.version,
            "platform": platform.platform(),
            "blender_path": rel(BLENDER_PATH),
            "blender_sha256": sha256_file(BLENDER_PATH),
            "package_rehash_enabled": not args.skip_package_rehash,
            "hash_workers": args.hash_workers,
        },
        "source_run": {
            "run_started_at": generation_manifest["run_started_at"],
            "workers": generation_manifest["workers"],
            "timeout_seconds": generation_manifest["timeout_seconds"],
            "texture_resolution": generation_manifest["texture_resolution"],
            "parts_file_count": generation_manifest["parts_file_count"],
        },
    }
    dump_json(output / "provenance.json", provenance)

    audit_started = time.monotonic()
    outcomes: list[dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.hash_workers) as pool:
        futures = [
            pool.submit(audit_case, SOURCE_ROOT, record, not args.skip_package_rehash)
            for record in records
        ]
        for index, future in enumerate(concurrent.futures.as_completed(futures), 1):
            outcomes.append(future.result())
            if index % 100 == 0 or index == len(futures):
                print(f"audited original packages: {index}/{len(futures)}", flush=True)
    outcomes.sort(key=lambda row: (row["factory"], row["seed"]))

    recovery_cases = recovery_manifest["cases"]
    recovery_audits: list[dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.hash_workers) as pool:
        recovery_audits = list(
            pool.map(
                lambda case: audit_recovery_case(
                    case, not args.skip_package_rehash
                ),
                recovery_cases,
            )
        )
    recovery_audits.sort(key=lambda row: (row["factory"], row["seed"]))

    with (output / "case_outcomes.jsonl").open("w", encoding="utf-8") as stream:
        for row in outcomes:
            stream.write(json.dumps(row, ensure_ascii=True, sort_keys=True) + "\n")

    factories = list(protocol["factories"])
    seeds = list(protocol["seeds"])
    requested_identities = {
        (factory, seed) for factory in factories for seed in seeds
    }
    record_identities = [(row["factory"], int(row["seed"])) for row in records]
    matched_factories = [row["factory"] for row in matched_protocol["categories"]]
    matched_outcomes = [row for row in outcomes if row["factory"] in matched_factories]

    public_summary = summarize_cohort(outcomes, factories)
    matched_summary = summarize_cohort(matched_outcomes, matched_factories)
    pass_outcomes = [row for row in outcomes if row["original_status"] == "PASS"]
    recovery_pass_count = sum(
        row["status"] == "PASS"
        and row["structural_package_pass"]
        and row["record_sha256_matches"]
        and row["package_hash_matches"] is not False
        for row in recovery_audits
    )
    original_pass_count = sum(
        row["original_300s_structural_package_qc_pass"] for row in outcomes
    )
    elapsed_passes = [
        row["elapsed_seconds"]
        for row in outcomes
        if row["original_300s_structural_package_qc_pass"]
    ]

    full_qc = state_metric(
        "not_evaluable",
        "N/E",
        (
            "the generation adapter did not execute author tests, compiler policies, "
            "overlap/rigid/disconnected policies, or motion-QC; all original PASS exports "
            "contain zero native collision elements"
        ),
        missing_prerequisites=[
            "author tests",
            "compiler baseline policies",
            "overlap policy",
            "rigid/disconnected policy",
            "motion-QC coverage",
            "native collision geometry",
        ],
    )
    all_36_full_qc = state_metric(
        "not_evaluable",
        "N/E",
        "the template-level headline requires 36/36 Full-QC outcomes, which are N/E",
    )
    corner = state_metric(
        "not_reported_under_exact_protocol",
        "N/R",
        (
            "no output-independent Infinite Mobility parameter-domain manifest or semantic "
            "mapping to PV-A project-native corners is available; random seeds are not corners"
        ),
    )
    regression = state_metric(
        "not_evaluable",
        "N/E",
        "no frozen pre/post-repair factory pair or retained-seed regression protocol",
    )
    public_summary["seed_full_qc"] = full_qc
    public_summary["all_36_full_qc"] = all_36_full_qc
    public_summary["corner_pass"] = corner
    public_summary["regression_retention"] = regression
    matched_summary["seed_full_qc"] = full_qc
    matched_summary["all_36_full_qc"] = all_36_full_qc
    matched_summary["corner_pass"] = corner
    matched_summary["regression_retention"] = regression

    checks: list[dict[str, Any]] = []

    def check(name: str, passed: bool, details: Any) -> None:
        checks.append({"name": name, "passed": bool(passed), "details": details})

    readme_factories = README_FACTORY_RE.findall(
        (OFFICIAL_ROOT / "README.md").read_text(encoding="utf-8")
    )
    check(
        "official release pin",
        remote == EXPECTED_OFFICIAL_REMOTE
        and official_commit == EXPECTED_OFFICIAL_COMMIT
        and official_tree == EXPECTED_OFFICIAL_TREE,
        {"remote": remote, "commit": official_commit, "tree": official_tree},
    )
    check(
        "public factories exactly match official README",
        readme_factories == factories and len(factories) == 20,
        {"readme_factories": readme_factories, "protocol_factories": factories},
    )
    check(
        "seed list exactly 0-35",
        seeds == list(range(36)),
        {"seeds": seeds},
    )
    check(
        "generation manifest matches frozen public protocol",
        generation_manifest["factories"] == factories
        and generation_manifest["seeds"] == seeds
        and generation_manifest["timeout_seconds"] == ORIGINAL_TIMEOUT_SECONDS,
        {
            "factory_count": len(generation_manifest["factories"]),
            "seed_count": len(generation_manifest["seeds"]),
            "timeout_seconds": generation_manifest["timeout_seconds"],
        },
    )
    check(
        "requested identities are complete and unique",
        len(record_identities) == 720
        and len(set(record_identities)) == 720
        and set(record_identities) == requested_identities,
        {
            "records": len(record_identities),
            "unique_records": len(set(record_identities)),
            "expected": len(requested_identities),
        },
    )
    check(
        "generation snapshot Python tree matches frozen manifest",
        cache_python_hash == generation_manifest["baseline_source_tree_sha256"],
        {
            "actual": cache_python_hash,
            "recorded": generation_manifest["baseline_source_tree_sha256"],
        },
    )
    check(
        "generation snapshot is content-equivalent to official tracked release",
        provenance["generation_snapshot"][
            "release_equivalent_to_official_tracked_files"
        ],
        {
            "official_snapshot": official_snapshot_hash,
            "cache_snapshot": cache_snapshot_hash,
            "tracked_files": tracked_count,
        },
    )
    check(
        "all original PASS structural packages revalidate",
        all(row["reaudit"]["structural_package_pass"] for row in pass_outcomes),
        {
            "revalidated": sum(
                row["reaudit"]["structural_package_pass"] for row in pass_outcomes
            ),
            "expected": len(pass_outcomes),
        },
    )
    check(
        "all original PASS package hashes match",
        args.skip_package_rehash
        or all(row["package_hash_matches"] is True for row in pass_outcomes),
        {
            "rehash_enabled": not args.skip_package_rehash,
            "matches": sum(row["package_hash_matches"] is True for row in pass_outcomes),
            "expected": len(pass_outcomes),
        },
    )
    check(
        "recorded validation matches structural re-audit",
        all(row["recorded_validation_matches_reaudit"] for row in outcomes),
        {
            "matches": sum(row["recorded_validation_matches_reaudit"] for row in outcomes),
            "expected": len(outcomes),
        },
    )
    check(
        "original 300s status denominator preserved",
        original_pass_count == 713
        and Counter(row["original_status"] for row in outcomes)
        == Counter({"PASS": 713, "TIMEOUT": 7}),
        {
            "structural_pass": original_pass_count,
            "statuses": dict(Counter(row["original_status"] for row in outcomes)),
        },
    )
    check(
        "180s main telemetry classification is exact",
        public_summary["seed_compile_180s"]["numerator"] == 702
        and public_summary["compile_only_all_36_180s"]["numerator"] == 16
        and len(public_summary["failures_180s"]) == 18,
        {
            "seed_compile": public_summary["seed_compile_180s"],
            "compile_only_all_36": public_summary["compile_only_all_36_180s"],
            "failure_count": len(public_summary["failures_180s"]),
        },
    )
    check(
        "matched five-category denominator and classification are exact",
        len(matched_factories) == 5
        and len(matched_outcomes) == 180
        and matched_summary["seed_compile_180s"]["numerator"] == 170
        and matched_summary["compile_only_all_36_180s"]["numerator"] == 4,
        {
            "factories": matched_factories,
            "seed_compile": matched_summary["seed_compile_180s"],
            "compile_only_all_36": matched_summary["compile_only_all_36_180s"],
        },
    )
    check(
        "all 900s recovery packages revalidate",
        recovery_pass_count == 7,
        {"passes": recovery_pass_count, "expected": 7},
    )
    check(
        "native collision geometry is absent",
        sum(row["reaudit"]["collision_count"] for row in pass_outcomes) == 0,
        {
            "collision_elements": sum(
                row["reaudit"]["collision_count"] for row in pass_outcomes
            ),
            "packages": len(pass_outcomes),
        },
    )
    check(
        "Full-QC-derived cells fail closed",
        full_qc["display"] == "N/E"
        and all_36_full_qc["display"] == "N/E"
        and full_qc["numerator"] is None
        and all_36_full_qc["numerator"] is None,
        {"seed_full_qc": full_qc["display"], "all_36": all_36_full_qc["display"]},
    )
    check(
        "corner protocol remains N/R",
        corner["display"] == "N/R" and corner["numerator"] is None,
        {"corner_pass": corner["display"], "reason": corner["reason"]},
    )

    overall_pass = all(row["passed"] for row in checks)
    summary: dict[str, Any] = {
        "schema_version": 1,
        "protocol_id": manifest["protocol_id"],
        "generated_at": utc_now(),
        "status": "COMPLETE" if overall_pass else "INTEGRITY_FAILURE",
        "method": "Infinite Mobility",
        "main_result_kind": "deterministic_180s_telemetry_sensitivity_not_fresh_rerun",
        "public_main_cohort": public_summary,
        "matched_supplementary_cohort": matched_summary,
        "recovery_900s_availability": {
            "counts_for_main_reliability": False,
            "recovered_original_timeout_packages": metric(recovery_pass_count, 7),
            "available_packages": metric(original_pass_count + recovery_pass_count, 720),
            "audits": recovery_audits,
        },
        "integrity": {
            "package_rehash_enabled": not args.skip_package_rehash,
            "original_pass_structural_reaudit": metric(
                sum(row["reaudit"]["structural_package_pass"] for row in pass_outcomes),
                len(pass_outcomes),
            ),
            "original_pass_package_hash_matches": metric(
                sum(row["package_hash_matches"] is True for row in pass_outcomes),
                len(pass_outcomes),
            )
            if not args.skip_package_rehash
            else state_metric("not_run", "N/E", "package rehash explicitly skipped"),
            "original_pass_record_validation_matches": metric(
                sum(row["recorded_validation_matches_reaudit"] for row in pass_outcomes),
                len(pass_outcomes),
            ),
            "native_collision_element_total": sum(
                row["reaudit"]["collision_count"] for row in pass_outcomes
            ),
        },
        "elapsed_telemetry_original_structural_passes": {
            "denominator": len(elapsed_passes),
            "mean_seconds": statistics.fmean(elapsed_passes),
            "median_seconds": statistics.median(elapsed_passes),
            "max_seconds": max(elapsed_passes),
            "over_180_seconds": sum(value > MAIN_TIMEOUT_SECONDS for value in elapsed_passes),
        },
        "audit_wall_seconds": time.monotonic() - audit_started,
        "self_check_passed": sum(row["passed"] for row in checks),
        "self_check_total": len(checks),
    }
    self_check = {
        "protocol_id": manifest["protocol_id"],
        "generated_at": utc_now(),
        "overall_pass": overall_pass,
        "passed": sum(row["passed"] for row in checks),
        "total": len(checks),
        "checks": checks,
    }
    dump_json(output / "summary.json", summary)
    dump_json(output / "self_check.json", self_check)
    (output / "report.md").write_text(
        make_report(summary, provenance), encoding="utf-8"
    )

    artifact_names = [
        "manifest.json",
        "provenance.json",
        "case_outcomes.jsonl",
        "summary.json",
        "self_check.json",
        "report.md",
    ]
    artifact_hashes = {
        "schema_version": 1,
        "files": {
            name: {"sha256": sha256_file(output / name), "bytes": (output / name).stat().st_size}
            for name in artifact_names
        },
    }
    artifact_hashes["bundle_sha256"] = sha256_json(artifact_hashes["files"])
    dump_json(output / "artifact_hashes.json", artifact_hashes)

    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"artifact bundle sha256: {artifact_hashes['bundle_sha256']}")
    return 0 if overall_pass and not args.skip_package_rehash else 1


if __name__ == "__main__":
    raise SystemExit(main())
