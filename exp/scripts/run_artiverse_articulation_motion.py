#!/usr/bin/env python3
"""Evaluate the frozen Artiverse Table 6 N=100 semantic motion cohort.

The runner consumes, verifies, and never resamples the static audit artifacts.
It provides a discrete PyBullet collision proxy and an exact-FCL adaptive path
certificate. Composite annotations are scored by semantic pid/DoF, rather than
by the number of proxy joints in the exported URDF.
"""

from __future__ import annotations

import argparse
import concurrent.futures
from collections import Counter, defaultdict
import copy
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Iterable
import xml.etree.ElementTree as ET


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_ARTIVERSE = EXP_ROOT / "artiverse"
DEFAULT_STATIC_ROOT = EXP_ROOT / "runtime/table6_artiverse_reference_v1"
DEFAULT_OUTPUT = DEFAULT_STATIC_ROOT / "motion"
DEFAULT_PROTOCOL = EXP_ROOT / "reference/table6_artiverse_articulation_reference_v1.json"
DEFAULT_AMENDMENT = EXP_ROOT / "reference/table6_artiverse_contact_predicate_amendment_v1.json"
DEFAULT_BACKEND_PYTHON = EXP_ROOT.parent / "arti-template/.venv/bin/python"

EXPECTED_COHORT_PROTOCOL = "nano3d_table6_artiverse_motion_cohort_v1"
EXPECTED_PROTOCOL = "nano3d_table6_artiverse_articulation_reference_v1"
EXPECTED_AMENDMENT = "nano3d_table6_artiverse_contact_predicate_amendment_v1"
INVALIDATED_DIAGNOSTIC = {
    "protocol_sha256": "ebf053343ffd6df92dec801e9ba3a5bab189ca90077eb5467918c5dea8401ddb",
    "cohort_manifest_sha256": "7f6321ba472d070fcffa2559fbd02d1c8a164ee1ea9088ee91036151b956840d",
    "runner_sha256": "8acc11a9cb1b5f198904152333d495475ab0db93b9e431bfc30f4fb4aef5ca68",
    "run_config_sha256": "c966b6340a5d4f6c0899a9dadfd04e4b74207587fa939d42e2359daf69def49f",
    "summary_sha256": "99e9a44c40aed1c58aad90429e15c9c3862839622f01f0d8cdd64b8f56d136f7",
}
EXPECTED_RELEASE_MANIFEST_SHA256 = (
    "8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586"
)
COHORT_SIZE = 100
SINGLE_STATES = 11
MULTI_STATES = 64
PENETRATION_TOLERANCE_M = 1e-6
ADAPTIVE_MAX_DEPTH = 4
ABSOLUTE_CLEARANCE_M = 0.002
CLEARANCE_BBOX_FRACTION = 0.005
SCRIPT = Path(__file__).resolve()
BACKEND_LABELS = {
    "proxy": "pybullet_discrete_proxy",
    "strict": "exact_fcl_adaptive_strict",
}
BACKEND_DISTRIBUTIONS = ("numpy", "scipy", "pybullet", "python-fcl", "trimesh")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase",
        choices=("preflight", "proxy", "strict", "_worker"),
        default="preflight",
    )
    parser.add_argument("--artiverse-root", type=Path, default=DEFAULT_ARTIVERSE)
    parser.add_argument("--static-root", type=Path, default=DEFAULT_STATIC_ROOT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=None)
    parser.add_argument("--backend-python", type=Path, default=DEFAULT_BACKEND_PYTHON)
    parser.add_argument("--worker-input", type=Path)
    parser.add_argument("--worker-output", type=Path)
    return parser.parse_args()


def contained(path: Path, *, must_exist: bool) -> Path:
    resolved = path.resolve(strict=must_exist)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def backend_executable(path: Path) -> Path:
    candidate = path if path.is_absolute() else Path.cwd() / path
    candidate.absolute().relative_to(WORKSPACE.absolute())
    if not candidate.is_file() or not os.access(candidate, os.X_OK):
        raise ValueError(f"backend Python is not executable: {candidate}")
    return candidate.absolute()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_json(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True, default=str) + "\n")
    os.replace(temporary, path)


def module_available(name: str, python: Path | None = None) -> bool:
    if python is None:
        return importlib.util.find_spec(name) is not None
    completed = subprocess.run(
        [str(python), "-c", f"import importlib.util; raise SystemExit(importlib.util.find_spec({name!r}) is None)"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=30,
        check=False,
    )
    return completed.returncode == 0


def asset_key(identity: str) -> str:
    return hashlib.sha256(identity.encode()).hexdigest()[:24]


def backend_label(backend: str) -> str:
    return BACKEND_LABELS.get(backend, backend)


def local_backend_environment() -> dict[str, Any]:
    versions: dict[str, str | None] = {}
    for distribution in BACKEND_DISTRIBUTIONS:
        try:
            versions[distribution] = importlib.metadata.version(distribution)
        except importlib.metadata.PackageNotFoundError:
            versions[distribution] = None
    return {
        "python_executable": sys.executable,
        "python_version": sys.version,
        "distribution_versions": versions,
    }


def probe_backend_environment(python: Path) -> dict[str, Any]:
    code = """
import importlib.metadata
import json
import sys

names = ("numpy", "scipy", "pybullet", "python-fcl", "trimesh")
versions = {}
for name in names:
    try:
        versions[name] = importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        versions[name] = None
print(json.dumps({
    "python_executable": sys.executable,
    "python_version": sys.version,
    "distribution_versions": versions,
}, sort_keys=True))
"""
    completed = subprocess.run(
        [str(python), "-c", code],
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"cannot probe backend Python environment: {completed.stderr[-2000:]}"
        )
    return json.loads(completed.stdout)


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def close(left: float, right: float) -> bool:
    return math.isclose(left, right, rel_tol=1e-6, abs_tol=1e-8)


def validate_frozen_inputs(
    artiverse: Path,
    static_root: Path,
    protocol: Path,
    backend_python: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    paths = {
        "readiness": static_root / "readiness.json",
        "static_records": static_root / "static_asset_records.jsonl",
        "static_summary": static_root / "static_summary.json",
        "cohort": static_root / "motion_cohort_manifest.json",
        "protocol": protocol,
        "amendment": DEFAULT_AMENDMENT,
        "release_manifest": artiverse / "dataset_chunks/manifest.json",
    }
    missing = [name for name, path in paths.items() if not path.is_file()]
    dependencies = {
        "proxy": {
            name: module_available(name, backend_python)
            for name in ("numpy", "scipy", "pybullet")
        },
        "strict": {
            name: module_available(name, backend_python)
            for name in ("numpy", "scipy", "pybullet", "fcl", "trimesh")
        },
    }
    if missing:
        return (
            {
                "state": "BLOCKED",
                "blockers": [f"missing_{name}" for name in missing],
                "static_artifacts_present": False,
                "dependencies": dependencies,
                "backend_python": str(backend_python),
                "motion_execution_authorized": False,
            },
            [],
        )

    readiness = read_json(paths["readiness"])
    summary = read_json(paths["static_summary"])
    cohort = read_json(paths["cohort"])
    amendment = read_json(paths["amendment"])
    frozen_records = read_jsonl(paths["static_records"])
    release_manifest_hash = sha256_file(paths["release_manifest"])
    protocol_hash = sha256_file(protocol)
    amendment_hash = sha256_file(paths["amendment"])
    static_records_hash = sha256_file(paths["static_records"])
    static_summary_hash = sha256_file(paths["static_summary"])
    readiness_hash = sha256_file(paths["readiness"])
    cohort_hash = sha256_file(paths["cohort"])

    checks = {
        "readiness_ready": readiness.get("state") == "READY",
        "readiness_static_authorized": readiness.get("static_audit_authorized") is True,
        "static_asset_denominator": summary.get("asset_denominator") == 3544,
        "cohort_protocol": cohort.get("protocol_id") == EXPECTED_COHORT_PROTOCOL,
        "cohort_intent_size": cohort.get("intent_size") == COHORT_SIZE,
        "cohort_asset_count": len(cohort.get("assets", [])) == COHORT_SIZE,
        "cohort_unique_identity": len(
            {asset.get("identity") for asset in cohort.get("assets", [])}
        )
        == COHORT_SIZE,
        "selection_salt": cohort.get("selection_salt")
        == "nano3d-table6-artiverse-reference-v1",
        "selection_is_outcome_blind": cohort.get("selection_uses_motion_or_collision_outcome")
        is False,
        "failure_replacement_disabled": cohort.get("failure_replacement") is False,
        "protocol_id": read_json(protocol).get("protocol_id") == EXPECTED_PROTOCOL,
        "protocol_sha256": cohort.get("source_protocol_sha256") == protocol_hash,
        "amendment_id": amendment.get("amendment_id") == EXPECTED_AMENDMENT,
        "amendment_protocol_id": amendment.get("applies_to_protocol_id")
        == EXPECTED_PROTOCOL,
        "amendment_protocol_sha256": amendment.get("applies_to_protocol_sha256")
        == protocol_hash,
        "amendment_invalidated_diagnostic": all(
            amendment.get("invalidated_diagnostic", {}).get(key) == value
            for key, value in INVALIDATED_DIAGNOSTIC.items()
        ),
        "amendment_invalidated_excluded": amendment.get(
            "invalidated_diagnostic", {}
        ).get("state")
        == "INVALIDATED_DIAGNOSTIC_EXCLUDED"
        and amendment.get("invalidated_diagnostic", {}).get(
            "publication_authorized"
        )
        is False,
        "amendment_cohort_lock": amendment.get("cohort_lock", {}).get(
            "selection_change_allowed"
        )
        is False
        and amendment.get("cohort_lock", {}).get(
            "identity_order_must_match_invalidated_diagnostic"
        )
        is True
        and amendment.get("cohort_lock", {}).get("failure_replacement_allowed")
        is False
        and amendment.get("cohort_lock", {}).get("canonical_digest")
        == "SHA-256 of JSON with sorted keys and compact separators"
        and amendment.get("cohort_lock", {}).get("ordered_identity_sha256")
        == sha256_json([asset.get("identity") for asset in cohort.get("assets", [])])
        and amendment.get("cohort_lock", {}).get(
            "ordered_index_identity_selection_sha256"
        )
        == sha256_json(
            [
                {
                    key: asset.get(key)
                    for key in ("cohort_index", "identity", "selection_hash")
                }
                for asset in cohort.get("assets", [])
            ]
        ),
        "release_manifest_sha256": release_manifest_hash
        == EXPECTED_RELEASE_MANIFEST_SHA256
        == cohort.get("source_release_manifest_sha256"),
        "static_records_sha256": cohort.get("source_static_asset_records_sha256")
        == static_records_hash,
        "static_summary_sha256": cohort.get("source_static_summary_sha256")
        == static_summary_hash,
        "readiness_sha256": cohort.get("source_readiness_sha256") == readiness_hash,
        "static_record_count": len(frozen_records) == 3544,
    }
    records_by_identity = {record.get("identity"): record for record in frozen_records}
    verified_assets: list[dict[str, Any]] = []
    asset_errors: list[str] = []
    if all(checks.values()):
        for frozen in cohort["assets"]:
            identity = str(frozen.get("identity"))
            static = records_by_identity.get(identity)
            if static is None:
                asset_errors.append(f"{identity}: missing static record")
                continue
            if not static.get("static_motion_eligible"):
                asset_errors.append(f"{identity}: no longer static-motion eligible")
                continue
            if frozen.get("semantic_joints") != static["annotation"].get("semantic_joints"):
                asset_errors.append(f"{identity}: frozen semantic-joint mapping drift")
                continue
            expected_coordinates = [
                coordinate
                for mapping in static["urdf"].get("mapping_rows", [])
                for coordinate in mapping.get("exported_coordinates", [])
            ]
            if frozen.get("exported_coordinates") != expected_coordinates:
                asset_errors.append(f"{identity}: frozen exported-coordinate mapping drift")
                continue
            if frozen.get("unsupported_annotation_types") != static["annotation"].get(
                "unsupported_types"
            ):
                asset_errors.append(f"{identity}: frozen unsupported-type inventory drift")
                continue
            if frozen.get("annotation_missing_pid_count") != static["annotation"].get(
                "missing_pid_count"
            ):
                asset_errors.append(f"{identity}: frozen missing-pid count drift")
                continue
            urdf_rel = frozen.get("urdf_path")
            articulation_rel = frozen.get("articulation_path")
            if not isinstance(urdf_rel, str) or not isinstance(articulation_rel, str):
                asset_errors.append(f"{identity}: missing frozen native paths")
                continue
            try:
                urdf = contained(WORKSPACE / urdf_rel, must_exist=True)
                articulation = contained(WORKSPACE / articulation_rel, must_exist=True)
                urdf.relative_to(artiverse)
                articulation.relative_to(artiverse)
            except (FileNotFoundError, ValueError) as exc:
                asset_errors.append(f"{identity}: native path invalid: {exc}")
                continue
            if sha256_file(urdf) != frozen.get("urdf_sha256"):
                asset_errors.append(f"{identity}: native URDF SHA-256 drift")
                continue
            if sha256_file(articulation) != frozen.get("articulation_sha256"):
                asset_errors.append(f"{identity}: annotation SHA-256 drift")
                continue
            inventory = frozen.get("collision_mesh_inventory")
            if not isinstance(inventory, list) or not inventory:
                asset_errors.append(f"{identity}: missing frozen collision inventory")
                continue
            if sha256_json(inventory) != frozen.get("collision_inventory_sha256"):
                asset_errors.append(f"{identity}: collision inventory digest mismatch")
                continue
            inventory_valid = True
            for item in inventory:
                try:
                    mesh = contained(WORKSPACE / str(item["path"]), must_exist=True)
                    mesh.relative_to(artiverse)
                    if mesh.stat().st_size != item["bytes"] or sha256_file(mesh) != item["sha256"]:
                        raise ValueError("size or SHA-256 drift")
                except (KeyError, FileNotFoundError, ValueError) as exc:
                    asset_errors.append(f"{identity}: collision inventory invalid: {exc}")
                    inventory_valid = False
                    break
            if not inventory_valid:
                continue
            for key in (
                "raw_annotation_record_count",
                "semantic_joint_count",
                "semantic_dof_count",
                "exported_movable_joint_element_count",
            ):
                static_value = (
                    static["annotation"].get("raw_record_count")
                    if key == "raw_annotation_record_count"
                    else static["annotation"].get(key)
                    if key in {"semantic_joint_count", "semantic_dof_count"}
                    else static["urdf"].get("movable_joint_element_count")
                )
                if frozen.get(key) != static_value:
                    asset_errors.append(f"{identity}: frozen denominator drift for {key}")
                    inventory_valid = False
                    break
            if inventory_valid:
                verified_assets.append({**frozen, "native_urdf": str(urdf)})
    checks["all_cohort_assets_verified"] = len(verified_assets) == COHORT_SIZE
    blockers = [name for name, passed in checks.items() if not passed]
    if asset_errors:
        blockers.append("frozen_asset_content")
    state = "READY" if not blockers else "BLOCKED"
    return (
        {
            "state": state,
            "blockers": blockers,
            "checks": checks,
            "asset_error_count": len(asset_errors),
            "asset_error_examples": asset_errors[:25],
            "cohort_manifest_sha256": cohort_hash,
            "static_asset_records_sha256": static_records_hash,
            "protocol_sha256": protocol_hash,
            "amendment_sha256": amendment_hash,
            "release_manifest_sha256": release_manifest_hash,
            "verified_asset_count": len(verified_assets),
            "dependencies": dependencies,
            "backend_python": str(backend_python),
            "motion_execution_authorized": state == "READY",
        },
        verified_assets,
    )


def coordinate_by_component(
    exported: list[dict[str, Any]],
    *,
    component: str | None = None,
    joint_type: str | None = None,
) -> dict[str, Any] | None:
    matches = [
        item
        for item in exported
        if (component is None or item.get("component") == component)
        and (joint_type is None or item.get("type") == joint_type)
    ]
    return matches[0] if len(matches) == 1 else None


def valid_range(value: Any) -> tuple[float, float] | None:
    if (
        isinstance(value, list)
        and len(value) == 2
        and finite(value[0])
        and finite(value[1])
        and float(value[0]) <= float(value[1])
    ):
        return float(value[0]), float(value[1])
    return None


def export_range(coordinate: dict[str, Any]) -> tuple[float, float] | None:
    lower, upper = coordinate.get("lower"), coordinate.get("upper")
    if finite(lower) and finite(upper) and float(lower) <= float(upper):
        return float(lower), float(upper)
    return None


def same_range(left: tuple[float, float], right: tuple[float, float]) -> bool:
    return close(left[0], right[0]) and close(left[1], right[1])


def compile_motion_plan(asset: dict[str, Any]) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    semantic_coordinates: list[dict[str, Any]] = []
    semantic_joint_groups: dict[str, list[str]] = {}
    exported_by_pid: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for coordinate in asset.get("exported_coordinates", []):
        exported_by_pid[str(coordinate.get("pid"))].append(coordinate)
    if asset.get("unsupported_annotation_types"):
        errors.append(
            "unsupported raw annotation types: "
            + ",".join(sorted(str(value) for value in asset["unsupported_annotation_types"]))
        )
    if int(asset.get("annotation_missing_pid_count", 0)):
        errors.append(
            f"raw annotation records missing pid: {asset['annotation_missing_pid_count']}"
        )

    for semantic in asset.get("semantic_joints", []):
        pid = str(semantic.get("pid"))
        joint_type = str(semantic.get("type"))
        exported = exported_by_pid.get(pid, [])
        ranges = [valid_range(value) for value in semantic.get("coordinate_ranges", [])]
        coordinate_ids: list[str] = []
        if not semantic.get("axis_metadata_valid"):
            errors.append(f"pid={pid}: invalid or incomplete axis metadata")
        if not semantic.get("origin_metadata_valid"):
            message = f"pid={pid}: invalid or incomplete annotation origin metadata"
            if joint_type == "universal":
                errors.append(message)
            else:
                warnings.append(
                    message + "; executing the frozen native URDF origin"
                )
        if not semantic.get("range_metadata_valid"):
            errors.append(f"pid={pid}: invalid or incomplete range metadata")

        def add_direct(
            component: str,
            semantic_range: tuple[float, float] | None,
            exported_coordinate: dict[str, Any] | None,
        ) -> None:
            coordinate_id = f"{pid}:{component}"
            if semantic_range is None:
                errors.append(f"pid={pid} {component}: invalid semantic range")
                return
            if exported_coordinate is None:
                errors.append(f"pid={pid} {component}: exported coordinate is not unique")
                return
            native_range = export_range(exported_coordinate)
            if native_range is None or not same_range(semantic_range, native_range):
                errors.append(
                    f"pid={pid} {component}: semantic/export range mismatch "
                    f"{semantic_range} vs {native_range}"
                )
                return
            coordinate_ids.append(coordinate_id)
            semantic_coordinates.append(
                {
                    "coordinate_id": coordinate_id,
                    "pid": pid,
                    "semantic_joint_type": joint_type,
                    "component": component,
                    "lower": semantic_range[0],
                    "upper": semantic_range[1],
                    "execution_mapping": [
                        {
                            "joint_name": exported_coordinate["name"],
                            "formula": "identity",
                        }
                    ],
                }
            )

        if joint_type in {"revolute", "prismatic", "continuous"}:
            add_direct(
                joint_type,
                ranges[0] if ranges else None,
                coordinate_by_component(exported, joint_type=joint_type),
            )
        elif joint_type == "cylindrical":
            add_direct(
                "rotation",
                ranges[0] if len(ranges) > 0 else None,
                coordinate_by_component(exported, component="rotation")
                or coordinate_by_component(exported, joint_type="continuous"),
            )
            add_direct(
                "translation",
                ranges[1] if len(ranges) > 1 else None,
                coordinate_by_component(exported, component="translation")
                or coordinate_by_component(exported, joint_type="prismatic"),
            )
        elif joint_type == "universal":
            add_direct(
                "axis1",
                ranges[0] if len(ranges) > 0 else None,
                coordinate_by_component(exported, component="axis1"),
            )
            add_direct(
                "axis2",
                ranges[1] if len(ranges) > 1 else None,
                coordinate_by_component(exported, component="axis2"),
            )
        elif joint_type == "screw":
            rotation_range = ranges[0] if len(ranges) > 0 else None
            translation_range = ranges[1] if len(ranges) > 1 else None
            source = coordinate_by_component(exported, joint_type="revolute")
            if rotation_range is None or translation_range is None or source is None:
                errors.append(f"pid={pid} screw: incomplete semantic/export mapping")
            else:
                native_rotation_range = export_range(source)
                try:
                    pitch = float(source["screw_pitch"])
                    trans_min = float(source["trans_min"])
                    trans_max = float(source["trans_max"])
                except (KeyError, TypeError, ValueError):
                    errors.append(f"pid={pid} screw: missing export coupling metadata")
                else:
                    delta_theta = rotation_range[1] - rotation_range[0]
                    implied_pitch = (
                        (translation_range[1] - translation_range[0]) / delta_theta
                        if abs(delta_theta) > 1e-12
                        else math.nan
                    )
                    if (
                        not all(finite(value) for value in (pitch, trans_min, trans_max, implied_pitch))
                        or native_rotation_range is None
                        or not same_range(rotation_range, native_rotation_range)
                        or not same_range(translation_range, (trans_min, trans_max))
                        or not math.isclose(pitch, implied_pitch, rel_tol=1e-5, abs_tol=1e-9)
                    ):
                        errors.append(f"pid={pid} screw: inconsistent pitch/translation metadata")
                    else:
                        coordinate_id = f"{pid}:screw"
                        coordinate_ids.append(coordinate_id)
                        semantic_coordinates.append(
                            {
                                "coordinate_id": coordinate_id,
                                "pid": pid,
                                "semantic_joint_type": joint_type,
                                "component": "coupled_rotation_translation",
                                "lower": rotation_range[0],
                                "upper": rotation_range[1],
                                "translation_lower": translation_range[0],
                                "translation_upper": translation_range[1],
                                "pitch": pitch,
                                "native_joint_name": source["name"],
                                "execution_mapping": [],
                            }
                        )
        else:
            errors.append(f"pid={pid}: unsupported semantic type {joint_type!r}")
        semantic_joint_groups[pid] = coordinate_ids

    if len(semantic_joint_groups) != int(asset["semantic_joint_count"]):
        errors.append("semantic joint denominator does not match frozen value")
    if len(semantic_coordinates) != int(asset["semantic_dof_count"]):
        errors.append("semantic DoF denominator does not match frozen value")
    return {
        "identity": asset["identity"],
        "native_urdf": asset["native_urdf"],
        "native_urdf_sha256": asset["urdf_sha256"],
        "raw_annotation_record_count": asset["raw_annotation_record_count"],
        "semantic_joint_count": asset["semantic_joint_count"],
        "semantic_dof_count": asset["semantic_dof_count"],
        "exported_movable_joint_element_count": asset["exported_movable_joint_element_count"],
        "semantic_coordinates": semantic_coordinates,
        "semantic_joint_groups": semantic_joint_groups,
        "has_screw": any(
            coordinate["semantic_joint_type"] == "screw"
            for coordinate in semantic_coordinates
        ),
        "warnings": warnings,
        "errors": errors,
    }


def materialize_execution_urdf(plan: dict[str, Any], output_dir: Path) -> Path:
    native = Path(plan["native_urdf"])
    if not plan["has_screw"]:
        return native
    tree = ET.parse(native)
    robot = tree.getroot()
    coordinates = {
        coordinate["native_joint_name"]: coordinate
        for coordinate in plan["semantic_coordinates"]
        if coordinate["semantic_joint_type"] == "screw"
    }
    for joint in list(robot.findall("joint")):
        coordinate = coordinates.get(str(joint.get("name")))
        if coordinate is None:
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        axis = joint.find("axis")
        origin = joint.find("origin")
        limit = joint.find("limit")
        if parent is None or child is None or axis is None or limit is None:
            raise ValueError(f"cannot expand screw joint {joint.get('name')}")
        proxy_name = f"{child.get('link')}__artiverse_screw_proxy"
        if robot.find(f"link[@name='{proxy_name}']") is not None:
            raise ValueError(f"duplicate screw proxy link {proxy_name}")
        proxy = ET.Element("link", {"name": proxy_name})
        inertial = ET.SubElement(proxy, "inertial")
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": "1e-06"})
        ET.SubElement(
            inertial,
            "inertia",
            {"ixx": "1e-09", "ixy": "0", "ixz": "0", "iyy": "1e-09", "iyz": "0", "izz": "1e-09"},
        )
        robot.append(proxy)

        base_name = str(joint.get("name"))
        translation_name = f"{base_name}__screw_translation"
        rotation_name = f"{base_name}__screw_rotation"
        translation = ET.Element(
            "joint",
            {
                "name": translation_name,
                "type": "prismatic",
                "artiverse_original_type": "screw",
                "artiverse_pid": str(coordinate["pid"]),
                "artiverse_component": "translation",
            },
        )
        ET.SubElement(translation, "parent", {"link": str(parent.get("link"))})
        ET.SubElement(translation, "child", {"link": proxy_name})
        translation.append(copy.deepcopy(origin) if origin is not None else ET.Element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"}))
        translation.append(copy.deepcopy(axis))
        ET.SubElement(
            translation,
            "limit",
            {
                "lower": str(coordinate["translation_lower"]),
                "upper": str(coordinate["translation_upper"]),
                "effort": str(limit.get("effort", "100")),
                "velocity": str(limit.get("velocity", "5")),
            },
        )
        rotation = ET.Element(
            "joint",
            {
                "name": rotation_name,
                "type": "revolute",
                "artiverse_original_type": "screw",
                "artiverse_pid": str(coordinate["pid"]),
                "artiverse_component": "rotation",
            },
        )
        ET.SubElement(rotation, "parent", {"link": proxy_name})
        ET.SubElement(rotation, "child", {"link": str(child.get("link"))})
        ET.SubElement(rotation, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        rotation.append(copy.deepcopy(axis))
        rotation.append(copy.deepcopy(limit))
        index = list(robot).index(joint)
        robot.remove(joint)
        robot.insert(index, translation)
        robot.insert(index + 1, rotation)
        coordinate["execution_mapping"] = [
            {"joint_name": translation_name, "formula": "screw_translation"},
            {"joint_name": rotation_name, "formula": "identity"},
        ]

    native_parent = native.parent.resolve()
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        mesh_path = Path(filename.removeprefix("package://"))
        if not mesh_path.is_absolute():
            mesh_path = (native_parent / mesh_path).resolve()
        mesh.set("filename", str(mesh_path))
    output_dir.mkdir(parents=True, exist_ok=True)
    execution = output_dir / "model.urdf"
    ET.indent(tree, space="  ")
    tree.write(execution, encoding="utf-8", xml_declaration=True)
    return execution


def semantic_to_execution(
    plan: dict[str, Any], semantic_values: tuple[float, ...]
) -> dict[str, float]:
    result: dict[str, float] = {}
    for coordinate, value in zip(plan["semantic_coordinates"], semantic_values):
        for mapping in coordinate["execution_mapping"]:
            if mapping["formula"] == "identity":
                mapped = value
            elif mapping["formula"] == "screw_translation":
                fraction = (value - coordinate["lower"]) / (
                    coordinate["upper"] - coordinate["lower"]
                )
                mapped = coordinate["translation_lower"] + fraction * (
                    coordinate["translation_upper"] - coordinate["translation_lower"]
                )
            else:
                raise ValueError(f"unknown execution formula: {mapping['formula']}")
            result[mapping["joint_name"]] = float(mapped)
    return result


class DisjointSet:
    def __init__(self, values: Iterable[str]) -> None:
        self.parent = {value: value for value in values}

    def find(self, value: str) -> str:
        parent = self.parent[value]
        if parent != value:
            self.parent[value] = self.find(parent)
        return self.parent[value]

    def union(self, left: str, right: str) -> None:
        left_root, right_root = self.find(left), self.find(right)
        if left_root != right_root:
            self.parent[right_root] = left_root


def collision_policy(urdf: Path) -> dict[str, Any]:
    root = ET.parse(urdf).getroot()
    links = {str(link.get("name")) for link in root.findall("link")}
    dsu = DisjointSet(links)
    movable_groups: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"joint {joint.get('name')} has missing endpoint")
        parent_name, child_name = str(parent.get("link")), str(child.get("link"))
        if joint.get("type") == "fixed":
            dsu.union(parent_name, child_name)
        else:
            group = joint.get("artiverse_pid") or f"joint:{joint.get('name')}"
            movable_groups[str(group)].append((parent_name, child_name))
    adjacent_links: list[tuple[str, str]] = []
    for group, edges in movable_groups.items():
        parents = {parent for parent, _ in edges}
        children = {child for _, child in edges}
        outer_parents = sorted(parents - children)
        outer_children = sorted(children - parents)
        if len(outer_parents) != 1 or len(outer_children) != 1:
            raise ValueError(f"motion group {group} is not one serial semantic interface")
        adjacent_links.append((outer_parents[0], outer_children[0]))
    adjacency = {
        frozenset((dsu.find(parent), dsu.find(child)))
        for parent, child in adjacent_links
    }
    return {"dsu": dsu, "adjacency": adjacency, "links": links}


def bind_pybullet(pb: Any, body: int, plan: dict[str, Any]) -> dict[str, Any]:
    by_name: dict[str, int] = {}
    link_by_index: dict[int, str] = {}
    base_name_raw = pb.getBodyInfo(body)[0]
    link_by_index[-1] = (
        base_name_raw.decode() if isinstance(base_name_raw, bytes) else str(base_name_raw)
    )
    for index in range(pb.getNumJoints(body)):
        info = pb.getJointInfo(body, index)
        joint_name = info[1].decode() if isinstance(info[1], bytes) else str(info[1])
        link_name = info[12].decode() if isinstance(info[12], bytes) else str(info[12])
        by_name[joint_name] = index
        link_by_index[index] = link_name
    expected = {
        mapping["joint_name"]
        for coordinate in plan["semantic_coordinates"]
        for mapping in coordinate["execution_mapping"]
    }
    missing = sorted(expected - set(by_name))
    if missing:
        raise ValueError(f"execution joint mapping missing: {missing}")
    return {"joint_by_name": by_name, "link_by_index": link_by_index}


def semantic_defaults(plan: dict[str, Any]) -> tuple[float, ...]:
    return tuple(
        (float(coordinate["lower"]) + float(coordinate["upper"])) / 2.0
        for coordinate in plan["semantic_coordinates"]
    )


def single_paths(plan: dict[str, Any]) -> list[dict[str, Any]]:
    import numpy as np

    defaults = semantic_defaults(plan)
    paths: list[dict[str, Any]] = []
    for position, coordinate in enumerate(plan["semantic_coordinates"]):
        configs = []
        for value in np.linspace(coordinate["lower"], coordinate["upper"], SINGLE_STATES):
            config = list(defaults)
            config[position] = float(value)
            configs.append(tuple(config))
        paths.append({"coordinate": coordinate, "configs": configs})
    return paths


def sobol_path(plan: dict[str, Any]) -> list[tuple[float, ...]]:
    if int(plan["semantic_dof_count"]) <= 1:
        return []
    from scipy.stats import qmc

    unit = qmc.Sobol(d=int(plan["semantic_dof_count"]), scramble=False).random_base2(m=6)
    return [
        tuple(
            coordinate["lower"]
            + float(value) * (coordinate["upper"] - coordinate["lower"])
            for coordinate, value in zip(plan["semantic_coordinates"], row)
        )
        for row in unit
    ]


def filtered_contacts(
    contacts: list[Any],
    link_by_index: dict[int, str],
    policy: dict[str, Any],
) -> list[Any]:
    dsu: DisjointSet = policy["dsu"]
    adjacency: set[frozenset[str]] = policy["adjacency"]
    retained = []
    for contact in contacts:
        left_name = link_by_index.get(int(contact[3]))
        right_name = link_by_index.get(int(contact[4]))
        if left_name is None or right_name is None:
            raise ValueError(
                f"PyBullet contact references unknown link index: {contact[3]}, {contact[4]}"
            )
        if left_name not in policy["links"] or right_name not in policy["links"]:
            raise ValueError(
                f"PyBullet contact link is absent from collision policy: "
                f"{left_name!r}, {right_name!r}"
            )
        left_cluster, right_cluster = dsu.find(left_name), dsu.find(right_name)
        if left_cluster == right_cluster:
            continue
        if frozenset((left_cluster, right_cluster)) in adjacency:
            continue
        retained.append(contact)
    return retained


def base_result(asset: dict[str, Any], plan: dict[str, Any], backend: str) -> dict[str, Any]:
    return {
        "identity": asset["identity"],
        "asset_key": asset_key(asset["identity"]),
        "backend": backend_label(backend),
        "motion_plan_sha256": sha256_json(plan),
        "state": "FAIL",
        "reason": None,
        "raw_annotation_record_count": plan["raw_annotation_record_count"],
        "semantic_joint_count": plan["semantic_joint_count"],
        "semantic_dof_count": plan["semantic_dof_count"],
        "exported_movable_joint_element_count": plan["exported_movable_joint_element_count"],
        "single_state_count": 0,
        "multi_state_count": 0,
        "load_success": False,
        "drive_success": False,
        "joint_pass_count": 0,
        "asset_collision_free": False,
        "asset_penetration_free": False,
        "evaluation_complete": False,
        "evaluated_collision_state_count": 0,
        "collision_state_count": 0,
        "penetration_state_count": 0,
        "retained_contact_state_count": 0,
        "within_tolerance_contact_state_count": 0,
        "retained_contact_count": 0,
        "penetrating_contact_count": 0,
        "within_tolerance_contact_count": 0,
        "minimum_contact_distance_m": None,
        "max_penetration_m": 0.0,
        "first_penetration_witness": None,
        "deepest_penetration_witness": None,
        "metadata_warnings": plan["warnings"],
        "issues": [],
    }


def run_proxy_asset(asset: dict[str, Any], execution_root: Path) -> dict[str, Any]:
    import pybullet as pb

    plan = compile_motion_plan(asset)
    result = base_result(asset, plan, "proxy")
    if plan["errors"]:
        result.update(state="NOT_EVALUABLE", reason="SEMANTIC_PLAN_INVALID", issues=plan["errors"])
        return result
    execution = materialize_execution_urdf(plan, execution_root / asset_key(asset["identity"]))
    result["motion_plan_sha256"] = sha256_json(plan)
    result["execution_urdf"] = str(execution)
    result["execution_urdf_sha256"] = sha256_file(execution)
    policy = collision_policy(execution)
    client = pb.connect(pb.DIRECT)
    body: int | None = None
    try:
        include_parent_flag = getattr(pb, "URDF_USE_SELF_COLLISION_INCLUDE_PARENT", None)
        if not isinstance(include_parent_flag, int) or include_parent_flag == 0:
            raise RuntimeError(
                "PyBullet lacks nonzero URDF_USE_SELF_COLLISION_INCLUDE_PARENT"
            )
        flags = (
            pb.URDF_USE_INERTIA_FROM_FILE
            | pb.URDF_USE_SELF_COLLISION
            | include_parent_flag
        )
        result["pybullet_load_flags"] = int(flags)
        body = pb.loadURDF(str(execution), useFixedBase=True, flags=flags)
        result["load_success"] = True
        binding = bind_pybullet(pb, body, plan)

        def evaluate(
            config: tuple[float, ...],
            *,
            trajectory: str,
            state_index: int,
            coordinate: dict[str, Any] | None = None,
        ) -> dict[str, Any]:
            values = semantic_to_execution(plan, config)
            for name, value in values.items():
                pb.resetJointState(body, binding["joint_by_name"][name], value)
            pb.performCollisionDetection()
            contacts = filtered_contacts(
                pb.getContactPoints(bodyA=body, bodyB=body),
                binding["link_by_index"],
                policy,
            )
            distances = [float(contact[8]) for contact in contacts]
            if not all(finite(distance) for distance in distances):
                raise RuntimeError("non-finite PyBullet contactDistance")
            penetrating = [
                contact
                for contact in contacts
                if max(0.0, -float(contact[8])) > PENETRATION_TOLERANCE_M
            ]
            within_tolerance = [
                contact for contact in contacts if contact not in penetrating
            ]
            readback_ok = all(
                close(float(pb.getJointState(body, binding["joint_by_name"][name])[0]), value)
                for name, value in values.items()
            )
            max_penetration = max(
                [max(0.0, -float(contact[8])) for contact in contacts] or [0.0]
            )
            state = {
                "penetration_free": not penetrating,
                "readback_ok": readback_ok,
                "retained_contact_count": len(contacts),
                "penetrating_contact_count": len(penetrating),
                "within_tolerance_contact_count": len(within_tolerance),
                "minimum_contact_distance_m": min(distances, default=None),
                "max_penetration_m": max_penetration,
            }
            result["evaluated_collision_state_count"] += 1
            result["retained_contact_state_count"] += int(bool(contacts))
            result["within_tolerance_contact_state_count"] += int(
                bool(contacts) and not penetrating
            )
            result["collision_state_count"] += int(bool(penetrating))
            result["penetration_state_count"] += int(bool(penetrating))
            result["retained_contact_count"] += len(contacts)
            result["penetrating_contact_count"] += len(penetrating)
            result["within_tolerance_contact_count"] += len(within_tolerance)
            if distances:
                minimum = min(distances)
                previous = result["minimum_contact_distance_m"]
                result["minimum_contact_distance_m"] = (
                    minimum if previous is None else min(float(previous), minimum)
                )

            def witness(contact: Any) -> dict[str, Any]:
                link_a, link_b = int(contact[3]), int(contact[4])
                distance = float(contact[8])
                return {
                    "trajectory": trajectory,
                    "state_index": state_index,
                    "coordinate_id": coordinate.get("coordinate_id") if coordinate else None,
                    "semantic_pid": coordinate.get("pid") if coordinate else None,
                    "semantic_config_sha256": sha256_json(list(config)),
                    "link_index_a": link_a,
                    "link_index_b": link_b,
                    "link_name_a": binding["link_by_index"].get(link_a),
                    "link_name_b": binding["link_by_index"].get(link_b),
                    "contact_distance_m": distance,
                    "penetration_depth_m": max(0.0, -distance),
                    "penetration_tolerance_m": PENETRATION_TOLERANCE_M,
                }

            if penetrating:
                first = witness(penetrating[0])
                deepest = witness(min(penetrating, key=lambda contact: float(contact[8])))
                if result["first_penetration_witness"] is None:
                    result["first_penetration_witness"] = first
                previous = result["deepest_penetration_witness"]
                if (
                    previous is None
                    or deepest["penetration_depth_m"]
                    > previous["penetration_depth_m"]
                ):
                    result["deepest_penetration_witness"] = deepest
            return state

        all_penetration_free = True
        all_drive = True
        nominal = evaluate(
            semantic_defaults(plan),
            trajectory="nominal_midrange",
            state_index=0,
        )
        all_penetration_free = (
            all_penetration_free and nominal["penetration_free"]
        )
        all_drive = all_drive and nominal["readback_ok"]
        result["nominal_midrange_state_penetration_free"] = nominal[
            "penetration_free"
        ]
        result["nominal_midrange_state_retained_contact_count"] = nominal[
            "retained_contact_count"
        ]
        result["max_penetration_m"] = nominal["max_penetration_m"]
        coordinate_results: list[dict[str, Any]] = []
        for path in single_paths(plan):
            coordinate_penetration_free = True
            coordinate_drive = True
            retained_state_count = 0
            penetration_state_count = 0
            for state_index, config in enumerate(path["configs"]):
                state = evaluate(
                    config,
                    trajectory="single_coordinate",
                    state_index=state_index,
                    coordinate=path["coordinate"],
                )
                result["single_state_count"] += 1
                result["max_penetration_m"] = max(
                    result["max_penetration_m"], state["max_penetration_m"]
                )
                retained_state_count += int(bool(state["retained_contact_count"]))
                penetration_state_count += int(not state["penetration_free"])
                coordinate_penetration_free = (
                    coordinate_penetration_free and state["penetration_free"]
                )
                coordinate_drive = coordinate_drive and state["readback_ok"]
            coordinate_results.append(
                {
                    "coordinate_id": path["coordinate"]["coordinate_id"],
                    "semantic_pid": path["coordinate"]["pid"],
                    "state_count": len(path["configs"]),
                    "retained_contact_state_count": retained_state_count,
                    "penetration_state_count": penetration_state_count,
                    "penetration_free": coordinate_penetration_free,
                    "drive_success": coordinate_drive,
                    "pass": coordinate_penetration_free and coordinate_drive,
                }
            )
            all_penetration_free = (
                all_penetration_free and coordinate_penetration_free
            )
            all_drive = all_drive and coordinate_drive
        multi = sobol_path(plan)
        multi_penetration_free = True
        multi_drive = True
        multi_retained_state_count = 0
        multi_penetration_state_count = 0
        for state_index, config in enumerate(multi):
            state = evaluate(
                config,
                trajectory="sobol_multi",
                state_index=state_index,
            )
            result["multi_state_count"] += 1
            result["max_penetration_m"] = max(
                result["max_penetration_m"], state["max_penetration_m"]
            )
            multi_retained_state_count += int(bool(state["retained_contact_count"]))
            multi_penetration_state_count += int(not state["penetration_free"])
            multi_penetration_free = (
                multi_penetration_free and state["penetration_free"]
            )
            multi_drive = multi_drive and state["readback_ok"]
            all_penetration_free = all_penetration_free and state["penetration_free"]
            all_drive = all_drive and state["readback_ok"]
        result["single_coordinate_results"] = coordinate_results
        coordinate_by_id = {
            row["coordinate_id"]: row for row in coordinate_results
        }
        result["semantic_joint_single_sweep_results"] = [
            {
                "semantic_pid": pid,
                "coordinate_ids": coordinate_ids,
                "pass": all(coordinate_by_id[item]["pass"] for item in coordinate_ids),
            }
            for pid, coordinate_ids in plan["semantic_joint_groups"].items()
        ]
        result["multi_sobol_result"] = {
            "state_count": len(multi),
            "retained_contact_state_count": multi_retained_state_count,
            "penetration_state_count": multi_penetration_state_count,
            "penetration_free": multi_penetration_free,
            "drive_success": multi_drive,
        }
        result["drive_success"] = all_drive
        result["joint_pass_count"] = sum(
            row["pass"] for row in result["semantic_joint_single_sweep_results"]
        )
        result["asset_penetration_free"] = all_penetration_free
        result["asset_collision_free"] = all_penetration_free
        result["evaluation_complete"] = True
        result["state"] = "PASS" if all_drive and all_penetration_free else "FAIL"
        if result["state"] == "PASS":
            result["reason"] = None
        elif not all_drive and not all_penetration_free:
            result["reason"] = "DRIVE_READBACK_AND_PENETRATION_FAILURE"
        elif not all_drive:
            result["reason"] = "DRIVE_READBACK_FAILURE"
        else:
            result["reason"] = "PENETRATION_BEYOND_TOLERANCE_OBSERVED"
        result["collision_policy"] = (
            "PyBullet self-collision with parent contacts enabled, followed by fixed-cluster "
            "and semantic-pid parent-child filtering after transparent composite-proxy "
            "collapse; failure iff retained contactDistance < -1e-6 m"
        )
    except Exception as exc:  # noqa: BLE001
        result["state"] = "NOT_EVALUABLE"
        result["issues"].append(f"{type(exc).__name__}: {exc}")
        result["reason"] = "LOAD_DRIVE_OR_BACKEND_NOT_EVALUABLE"
    finally:
        if body is not None:
            pb.removeBody(body)
        pb.disconnect(client)
    return result


def vector(raw: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if not raw:
        return default
    return tuple(float(value) for value in raw.split())


def rpy_matrix(np: Any, rpy: tuple[float, float, float]) -> Any:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=float)
    return rz @ ry @ rx


def transform_matrix(
    np: Any, xyz: tuple[float, float, float], rpy: tuple[float, float, float]
) -> Any:
    result = np.eye(4)
    result[:3, :3] = rpy_matrix(np, rpy)
    result[:3, 3] = xyz
    return result


def quaternion_matrix(np: Any, pb: Any, position: Any, quaternion: Any) -> Any:
    result = np.eye(4)
    result[:3, :3] = np.asarray(pb.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
    result[:3, 3] = np.asarray(position)
    return result


def load_exact_shapes(
    np: Any,
    fcl: Any,
    trimesh: Any,
    urdf: Path,
    link_names: set[str],
) -> tuple[list[dict[str, Any]], list[str]]:
    root = ET.parse(urdf).getroot()
    shapes: list[dict[str, Any]] = []
    issues: list[str] = []
    for link in root.findall("link"):
        name = str(link.get("name"))
        if name not in link_names:
            issues.append(f"missing Bullet link mapping: {name}")
            continue
        for index, collision in enumerate(link.findall("collision")):
            origin = collision.find("origin")
            local = transform_matrix(
                np,
                vector(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0)),
                vector(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0)),
            )
            geometry = collision.find("geometry")
            if geometry is None:
                issues.append(f"{name}[{index}]: missing geometry")
                continue
            try:
                mesh = geometry.find("mesh")
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                sphere = geometry.find("sphere")
                if mesh is not None:
                    mesh_path = Path(str(mesh.get("filename")))
                    if not mesh_path.is_absolute():
                        mesh_path = (urdf.parent / mesh_path).resolve()
                    loaded = trimesh.load(mesh_path, force="scene", process=False)
                    if isinstance(loaded, trimesh.Scene):
                        geometries = [item.copy() for item in loaded.geometry.values()]
                        if not geometries:
                            raise ValueError("empty mesh scene")
                        loaded = trimesh.util.concatenate(geometries)
                    vertices = np.asarray(loaded.vertices, dtype=np.float64) * np.asarray(
                        vector(mesh.get("scale"), (1.0, 1.0, 1.0))
                    )
                    faces = np.asarray(loaded.faces, dtype=np.int32)
                    if not len(vertices) or not len(faces):
                        raise ValueError("empty collision mesh")
                    model = fcl.BVHModel()
                    model.beginModel(len(vertices), len(faces))
                    model.addSubModel(vertices, faces)
                    model.endModel()
                    shape = model
                    radius = float(np.linalg.norm(vertices, axis=1).max())
                elif box is not None:
                    size = vector(box.get("size"), (0.0, 0.0, 0.0))
                    shape = fcl.Box(*size)
                    radius = float(np.linalg.norm(np.asarray(size) / 2.0))
                elif cylinder is not None:
                    cylinder_radius = float(str(cylinder.get("radius")))
                    cylinder_length = float(str(cylinder.get("length")))
                    shape = fcl.Cylinder(cylinder_radius, cylinder_length)
                    radius = math.hypot(cylinder_radius, cylinder_length / 2.0)
                elif sphere is not None:
                    radius = float(str(sphere.get("radius")))
                    shape = fcl.Sphere(radius)
                else:
                    raise ValueError("unsupported collision geometry")
                shapes.append(
                    {
                        "shape_id": f"{name}[{index}]",
                        "link_name": name,
                        "local": local,
                        "geometry": shape,
                        "radius": radius,
                    }
                )
            except Exception as exc:  # noqa: BLE001
                issues.append(f"{name}[{index}]: {type(exc).__name__}: {exc}")
    return shapes, issues


def exact_pairs(
    shapes: list[dict[str, Any]], policy: dict[str, Any]
) -> list[tuple[int, int]]:
    dsu: DisjointSet = policy["dsu"]
    adjacency: set[frozenset[str]] = policy["adjacency"]
    pairs = []
    for left in range(len(shapes)):
        for right in range(left + 1, len(shapes)):
            left_cluster = dsu.find(shapes[left]["link_name"])
            right_cluster = dsu.find(shapes[right]["link_name"])
            if left_cluster == right_cluster:
                continue
            if frozenset((left_cluster, right_cluster)) in adjacency:
                continue
            pairs.append((left, right))
    return pairs


def shape_motion_weights(
    urdf: Path,
    shapes: list[dict[str, Any]],
    execution_joint_names: set[str],
) -> list[dict[str, float]]:
    root = ET.parse(urdf).getroot()
    parent_joint_by_child: dict[str, dict[str, Any]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"joint {joint.get('name')} has missing endpoint")
        origin = joint.find("origin")
        limit = joint.find("limit")
        lower = upper = 0.0
        if joint.get("type") == "prismatic":
            if limit is None:
                raise ValueError(f"prismatic joint {joint.get('name')} has no limit")
            lower, upper = float(str(limit.get("lower"))), float(str(limit.get("upper")))
        parent_joint_by_child[str(child.get("link"))] = {
            "name": str(joint.get("name")),
            "type": str(joint.get("type")),
            "parent": str(parent.get("link")),
            "origin_length": math.sqrt(
                sum(
                    component * component
                    for component in vector(
                        origin.get("xyz") if origin is not None else None,
                        (0.0, 0.0, 0.0),
                    )
                )
            ),
            "maximum_prismatic_excursion": max(abs(lower), abs(upper)),
        }

    weights: list[dict[str, float]] = []
    for shape in shapes:
        # This upper-bounds every shape point's distance to each ancestor axis:
        # local collision offset/radius plus all downstream joint offsets and
        # possible prismatic excursions. Triangle inequality keeps it valid
        # across arbitrary downstream rotations.
        downstream_radius = float(shape["radius"]) + float(
            math.sqrt(sum(float(value) ** 2 for value in shape["local"][:3, 3]))
        )
        current = shape["link_name"]
        item_weights: dict[str, float] = {}
        seen: set[str] = set()
        while current in parent_joint_by_child:
            if current in seen:
                raise ValueError(f"cycle while deriving motion weights at link {current}")
            seen.add(current)
            joint = parent_joint_by_child[current]
            name, joint_type = joint["name"], joint["type"]
            if name in execution_joint_names:
                if joint_type in {"revolute", "continuous"}:
                    item_weights[name] = downstream_radius
                elif joint_type == "prismatic":
                    item_weights[name] = 1.0
                else:
                    raise ValueError(f"unsupported execution joint type {joint_type!r}")
            downstream_radius += joint["origin_length"]
            if joint_type == "prismatic":
                downstream_radius += joint["maximum_prismatic_excursion"]
            current = joint["parent"]
        weights.append(item_weights)
    return weights


def execution_motion_bound(
    weights: dict[str, float],
    start_values: dict[str, float],
    end_values: dict[str, float],
) -> float:
    return sum(
        multiplier * abs(end_values[name] - start_values[name])
        for name, multiplier in weights.items()
    )


def run_strict_asset(asset: dict[str, Any], execution_root: Path) -> dict[str, Any]:
    import fcl
    import numpy as np
    import pybullet as pb
    import trimesh

    plan = compile_motion_plan(asset)
    result = base_result(asset, plan, "strict")
    if plan["errors"]:
        result.update(state="NOT_EVALUABLE", reason="SEMANTIC_PLAN_INVALID", issues=plan["errors"])
        return result
    execution = materialize_execution_urdf(plan, execution_root / asset_key(asset["identity"]))
    result["motion_plan_sha256"] = sha256_json(plan)
    result["execution_urdf"] = str(execution)
    result["execution_urdf_sha256"] = sha256_file(execution)
    policy = collision_policy(execution)
    client = pb.connect(pb.DIRECT)
    body: int | None = None
    started = time.monotonic()
    try:
        body = pb.loadURDF(str(execution), useFixedBase=True)
        result["load_success"] = True
        binding = bind_pybullet(pb, body, plan)
        link_names = set(binding["link_by_index"].values())
        shapes, shape_issues = load_exact_shapes(np, fcl, trimesh, execution, link_names)
        result["shape_load_issues"] = shape_issues
        if shape_issues or not shapes:
            result.update(
                state="NOT_EVALUABLE",
                reason="EXACT_ENGINE_SHAPE_LOAD_INCOMPATIBLE",
                issues=shape_issues or ["no collision shapes"],
            )
            return result
        pairs = exact_pairs(shapes, policy)
        result["collision_shape_count"] = len(shapes)
        result["eligible_shape_pair_count"] = len(pairs)
        index_by_link = {name: index for index, name in binding["link_by_index"].items()}
        lower_bounds = []
        upper_bounds = []
        for link_name in {shape["link_name"] for shape in shapes}:
            low, high = pb.getAABB(body, index_by_link[link_name])
            lower_bounds.append(np.asarray(low, dtype=float))
            upper_bounds.append(np.asarray(high, dtype=float))
        bbox_diagonal = float(
            np.linalg.norm(np.max(upper_bounds, axis=0) - np.min(lower_bounds, axis=0))
        )
        clearance_trigger = max(
            ABSOLUTE_CLEARANCE_M, CLEARANCE_BBOX_FRACTION * bbox_diagonal
        )
        result["bbox_diagonal_m"] = bbox_diagonal
        result["clearance_trigger_m"] = clearance_trigger
        execution_joint_names = {
            mapping["joint_name"]
            for coordinate in plan["semantic_coordinates"]
            for mapping in coordinate["execution_mapping"]
        }
        motion_weights = shape_motion_weights(execution, shapes, execution_joint_names)
        state_cache: dict[tuple[float, ...], dict[str, Any]] = {}

        def transforms(config: tuple[float, ...]) -> list[Any]:
            values = semantic_to_execution(plan, config)
            for name, value in values.items():
                pb.resetJointState(body, binding["joint_by_name"][name], value)
            for name, value in values.items():
                actual = float(pb.getJointState(body, binding["joint_by_name"][name])[0])
                if not close(actual, value):
                    raise RuntimeError(
                        f"joint readback mismatch for {name}: requested={value}, actual={actual}"
                    )
            world: dict[str, Any] = {}
            base_pos, base_quat = pb.getBasePositionAndOrientation(body)
            world[binding["link_by_index"][-1]] = quaternion_matrix(
                np, pb, base_pos, base_quat
            )
            for index, link_name in binding["link_by_index"].items():
                if index < 0:
                    continue
                state = pb.getLinkState(body, index, computeForwardKinematics=True)
                world[link_name] = quaternion_matrix(np, pb, state[4], state[5])
            return [world[shape["link_name"]] @ shape["local"] for shape in shapes]

        def fcl_transform(matrix: Any) -> Any:
            return fcl.Transform(matrix[:3, :3], matrix[:3, 3])

        def evaluate(config: tuple[float, ...]) -> dict[str, Any]:
            key = tuple(round(float(value), 12) for value in config)
            if key in state_cache:
                return state_cache[key]
            matrices = transforms(key)
            pair_distances: dict[tuple[int, int], float] = {}
            collisions = []
            minimum = math.inf
            penetration = 0.0
            for left, right in pairs:
                object_left = fcl.CollisionObject(
                    shapes[left]["geometry"], fcl_transform(matrices[left])
                )
                object_right = fcl.CollisionObject(
                    shapes[right]["geometry"], fcl_transform(matrices[right])
                )
                distance_result = fcl.DistanceResult()
                distance = float(
                    fcl.distance(
                        object_left,
                        object_right,
                        fcl.DistanceRequest(
                            enable_nearest_points=True, enable_signed_distance=True
                        ),
                        distance_result,
                    )
                )
                pair_distances[(left, right)] = max(0.0, distance)
                minimum = min(minimum, max(0.0, distance))
                collision_result = fcl.CollisionResult()
                count = int(
                    fcl.collide(
                        object_left,
                        object_right,
                        fcl.CollisionRequest(num_max_contacts=8, enable_contact=True),
                        collision_result,
                    )
                )
                if count:
                    depths = [float(contact.penetration_depth) for contact in collision_result.contacts]
                    penetration = max(penetration, max(depths, default=0.0))
                    collisions.append((left, right))
            value = {
                "collision_free": not collisions,
                "minimum_clearance_m": minimum,
                "max_penetration_m": penetration,
                "pair_distances": pair_distances,
                "transforms": matrices,
            }
            state_cache[key] = value
            return value

        interval_count = 0
        adaptive_count = 0
        uncertified_count = 0
        uncertified_examples: list[dict[str, Any]] = []

        def certify(start: tuple[float, ...], end: tuple[float, ...], depth: int = 0) -> tuple[bool, bool]:
            nonlocal interval_count, adaptive_count, uncertified_count
            interval_count += 1
            start_state, end_state = evaluate(start), evaluate(end)
            if not start_state["collision_free"] or not end_state["collision_free"]:
                return True, False
            start_values = semantic_to_execution(plan, start)
            end_values = semantic_to_execution(plan, end)
            uncertain = []
            for left, right in pairs:
                clearance = min(
                    start_state["pair_distances"][(left, right)],
                    end_state["pair_distances"][(left, right)],
                )
                bound = execution_motion_bound(
                    motion_weights[left], start_values, end_values
                ) + execution_motion_bound(
                    motion_weights[right], start_values, end_values
                )
                if clearance <= max(bound + 1e-9, clearance_trigger):
                    uncertain.append((left, right))
            if not uncertain:
                return False, True
            if depth < ADAPTIVE_MAX_DEPTH:
                adaptive_count += 1
                midpoint = tuple((left + right) / 2.0 for left, right in zip(start, end))
                left_hit, left_ok = certify(start, midpoint, depth + 1)
                right_hit, right_ok = certify(midpoint, end, depth + 1)
                return left_hit or right_hit, left_ok and right_ok
            uncertified_count += len(uncertain)
            for left, right in uncertain:
                if len(uncertified_examples) >= 32:
                    break
                uncertified_examples.append(
                    {
                        "depth": depth,
                        "shape_a": shapes[left]["shape_id"],
                        "shape_b": shapes[right]["shape_id"],
                        "start": list(start),
                        "end": list(end),
                    }
                )
            return False, False

        defaults = semantic_defaults(plan)
        evaluate(defaults)
        joint_coordinate_results: dict[str, list[bool]] = defaultdict(list)
        any_collision = False
        all_certified = True
        for path in single_paths(plan):
            coordinate_collision = False
            coordinate_certified = True
            configs = path["configs"]
            result["single_state_count"] += len(configs)
            for index in range(len(configs) - 1):
                observed, certified = certify(configs[index], configs[index + 1])
                coordinate_collision = coordinate_collision or observed
                coordinate_certified = coordinate_certified and certified
            coordinate_pass = coordinate_certified and not coordinate_collision
            joint_coordinate_results[path["coordinate"]["pid"]].append(coordinate_pass)
            any_collision = any_collision or coordinate_collision
            all_certified = all_certified and coordinate_certified
        multi = sobol_path(plan)
        result["multi_state_count"] = len(multi)
        multi_collision = False
        multi_certified = True
        for index in range(len(multi) - 1):
            observed, certified = certify(multi[index], multi[index + 1])
            multi_collision = multi_collision or observed
            multi_certified = multi_certified and certified
        any_collision = any_collision or multi_collision
        all_certified = all_certified and multi_certified
        result["joint_pass_count"] = sum(
            len(values) == len(plan["semantic_joint_groups"][pid]) and all(values)
            for pid, values in joint_coordinate_results.items()
        )
        result["drive_success"] = True
        result["evaluated_interval_count_including_adaptive"] = interval_count
        result["adaptive_interval_count"] = adaptive_count
        result["uncertified_pair_interval_count"] = uncertified_count
        result["uncertified_examples"] = uncertified_examples
        result["discrete_collision_state_count"] = sum(
            not state["collision_free"] for state in state_cache.values()
        )
        result["evaluated_collision_state_count"] = len(state_cache)
        result["collision_state_count"] = result["discrete_collision_state_count"]
        result["minimum_clearance_m"] = min(
            (
                state["minimum_clearance_m"]
                for state in state_cache.values()
                if not math.isinf(state["minimum_clearance_m"])
            ),
            default=None,
        )
        result["max_penetration_m"] = max(
            (state["max_penetration_m"] for state in state_cache.values()),
            default=0.0,
        )
        result["asset_collision_free"] = not any_collision and all_certified
        result["evaluation_complete"] = True
        result["state"] = "PASS" if result["asset_collision_free"] else "FAIL"
        result["reason"] = (
            None
            if result["state"] == "PASS"
            else "COLLISION_OBSERVED_OR_INTERVAL_UNCERTIFIED"
        )
        result["elapsed_s"] = time.monotonic() - started
        result["collision_policy"] = (
            "exact native collision shapes, excluding same fixed cluster and semantic-pid "
            "parent-child adjacency after transparent composite-proxy collapse"
        )
        result["continuous_certificate"] = (
            "endpoint exact-FCL clearance versus URDF ancestor-chain Hausdorff motion bound; "
            "adaptive midpoint bisection to depth 4"
        )
    except Exception as exc:  # noqa: BLE001
        result["state"] = "NOT_EVALUABLE"
        result["issues"].append(f"{type(exc).__name__}: {exc}")
        result["reason"] = "LOAD_DRIVE_OR_BACKEND_NOT_EVALUABLE"
    finally:
        if body is not None:
            pb.removeBody(body)
        pb.disconnect(client)
    return result


def worker(input_path: Path, output_path: Path) -> int:
    payload = read_json(input_path)
    asset = payload["asset"]
    backend = payload["backend"]
    runner_hash = str(payload.get("runner_sha256"))
    cohort_hash = str(payload.get("cohort_manifest_sha256"))
    protocol_hash = str(payload.get("protocol_sha256"))
    amendment_hash = str(payload.get("amendment_sha256"))
    run_config_hash = str(payload.get("run_config_sha256"))
    run_config = payload.get("run_config")
    execution_root = contained(Path(payload["execution_root"]), must_exist=False)
    if runner_hash != sha256_file(SCRIPT):
        plan = compile_motion_plan(asset)
        result = base_result(asset, plan, backend)
        result["state"] = "NOT_EVALUABLE"
        result["reason"] = "WORKER_RUNNER_SHA_MISMATCH"
        result["issues"] = ["worker script does not match frozen runner SHA-256"]
        result["runner_sha256"] = runner_hash
        result["cohort_manifest_sha256"] = cohort_hash
        result["protocol_sha256"] = protocol_hash
        result["amendment_sha256"] = amendment_hash
        result["run_config_sha256"] = run_config_hash
        result["worker_input_sha256"] = sha256_file(input_path)
        write_json(output_path, result)
        return 2
    if not isinstance(run_config, dict) or sha256_json(run_config) != run_config_hash:
        plan = compile_motion_plan(asset)
        result = base_result(asset, plan, backend)
        result["state"] = "NOT_EVALUABLE"
        result["reason"] = "WORKER_RUN_CONFIG_SHA_MISMATCH"
        result["issues"] = ["worker run config does not match its SHA-256"]
        result["runner_sha256"] = runner_hash
        result["cohort_manifest_sha256"] = cohort_hash
        result["protocol_sha256"] = protocol_hash
        result["amendment_sha256"] = amendment_hash
        result["run_config_sha256"] = run_config_hash
        result["worker_input_sha256"] = sha256_file(input_path)
        write_json(output_path, result)
        return 2
    if (
        protocol_hash != run_config.get("protocol_sha256")
        or amendment_hash != run_config.get("amendment_sha256")
    ):
        plan = compile_motion_plan(asset)
        result = base_result(asset, plan, backend)
        result["state"] = "NOT_EVALUABLE"
        result["reason"] = "WORKER_PROTOCOL_AMENDMENT_MISMATCH"
        result["issues"] = [
            "worker protocol/amendment provenance does not match frozen run config"
        ]
        result["runner_sha256"] = runner_hash
        result["cohort_manifest_sha256"] = cohort_hash
        result["protocol_sha256"] = protocol_hash
        result["amendment_sha256"] = amendment_hash
        result["run_config_sha256"] = run_config_hash
        result["worker_input_sha256"] = sha256_file(input_path)
        write_json(output_path, result)
        return 2
    if run_config.get("backend_environment") != local_backend_environment():
        plan = compile_motion_plan(asset)
        result = base_result(asset, plan, backend)
        result["state"] = "NOT_EVALUABLE"
        result["reason"] = "WORKER_BACKEND_ENVIRONMENT_MISMATCH"
        result["issues"] = ["worker environment does not match frozen run config"]
        result["runner_sha256"] = runner_hash
        result["cohort_manifest_sha256"] = cohort_hash
        result["protocol_sha256"] = protocol_hash
        result["amendment_sha256"] = amendment_hash
        result["run_config_sha256"] = run_config_hash
        result["worker_input_sha256"] = sha256_file(input_path)
        write_json(output_path, result)
        return 2
    try:
        result = (
            run_proxy_asset(asset, execution_root)
            if backend == "proxy"
            else run_strict_asset(asset, execution_root)
        )
    except Exception as exc:  # noqa: BLE001
        plan = compile_motion_plan(asset)
        result = base_result(asset, plan, backend)
        result["state"] = "NOT_EVALUABLE"
        result["reason"] = "UNHANDLED_WORKER_FAILURE"
        result["issues"] = [f"{type(exc).__name__}: {exc}"]
    result["runner_sha256"] = runner_hash
    result["cohort_manifest_sha256"] = cohort_hash
    result["protocol_sha256"] = protocol_hash
    result["amendment_sha256"] = amendment_hash
    result["run_config_sha256"] = run_config_hash
    result["worker_input_sha256"] = sha256_file(input_path)
    write_json(output_path, result)
    return 0


def synthetic_not_evaluable(
    asset: dict[str, Any],
    backend: str,
    reason: str,
    *,
    runner_hash: str,
    cohort_hash: str,
    run_config_hash: str,
    run_config: dict[str, Any],
) -> dict[str, Any]:
    plan = compile_motion_plan(asset)
    result = base_result(asset, plan, backend)
    result.update(state="NOT_EVALUABLE", reason=reason)
    result["runner_sha256"] = runner_hash
    result["cohort_manifest_sha256"] = cohort_hash
    result["protocol_sha256"] = run_config["protocol_sha256"]
    result["amendment_sha256"] = run_config["amendment_sha256"]
    result["run_config_sha256"] = run_config_hash
    return result


def run_one(
    asset: dict[str, Any],
    backend: str,
    run_root: Path,
    timeout: float,
    cohort_hash: str,
    runner_hash: str,
    run_config_hash: str,
    run_config: dict[str, Any],
    backend_python: Path,
) -> dict[str, Any]:
    key = asset_key(asset["identity"])
    input_path = run_root / "worker_inputs" / f"{key}.json"
    result_path = run_root / "assets" / f"{key}.json"
    payload = {
        "asset": asset,
        "backend": backend,
        "cohort_manifest_sha256": cohort_hash,
        "runner_sha256": runner_hash,
        "protocol_sha256": run_config["protocol_sha256"],
        "amendment_sha256": run_config["amendment_sha256"],
        "run_config_sha256": run_config_hash,
        "run_config": run_config,
        "timeout_s": timeout,
        "backend_python": str(backend_python),
        "execution_root": str(run_root / "execution_views"),
    }
    write_json(input_path, payload)
    input_hash = sha256_file(input_path)
    if result_path.is_file():
        cached = read_json(result_path)
        if cached.get("worker_input_sha256") == input_hash:
            return cached
        result_path.unlink()
    try:
        completed = subprocess.run(
            [
                str(backend_python),
                str(SCRIPT),
                "--phase",
                "_worker",
                "--worker-input",
                str(input_path),
                "--worker-output",
                str(result_path),
            ],
            cwd=EXP_ROOT.parent,
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
            env={
                **os.environ,
                "OPENBLAS_NUM_THREADS": "1",
                "OMP_NUM_THREADS": "1",
                "MKL_NUM_THREADS": "1",
                "NUMEXPR_NUM_THREADS": "1",
            },
        )
    except subprocess.TimeoutExpired:
        result = synthetic_not_evaluable(
            asset,
            backend,
            "TIMEOUT",
            runner_hash=runner_hash,
            cohort_hash=cohort_hash,
            run_config_hash=run_config_hash,
            run_config=run_config,
        )
        if backend == "strict":
            result["state"] = "FAIL"
            result["reason"] = "TIMEOUT_COUNTS_AS_FAIL"
        result["worker_input_sha256"] = input_hash
        write_json(result_path, result)
        return result
    if result_path.is_file():
        result = read_json(result_path)
        if (
            completed.returncode == 0
            and result.get("worker_input_sha256") == input_hash
        ):
            return result
        result_path.unlink()
    result = synthetic_not_evaluable(
        asset,
        backend,
        "WORKER_EXIT_OR_RESULT_BINDING_FAILURE",
        runner_hash=runner_hash,
        cohort_hash=cohort_hash,
        run_config_hash=run_config_hash,
        run_config=run_config,
    )
    result["worker_returncode"] = completed.returncode
    result["worker_stderr"] = completed.stderr[-5000:]
    result["worker_input_sha256"] = input_hash
    write_json(result_path, result)
    return result


def aggregate_results(
    assets: list[dict[str, Any]],
    records: list[dict[str, Any]],
    backend: str,
    readiness: dict[str, Any],
    runner_hash: str,
    run_config_hash: str,
) -> dict[str, Any]:
    expected_by_identity = {str(asset["identity"]): asset for asset in assets}
    record_identities = [str(record.get("identity")) for record in records]
    if len(expected_by_identity) != len(assets):
        raise ValueError("frozen cohort contains duplicate identities")
    if len(records) != len(assets) or set(record_identities) != set(expected_by_identity):
        raise ValueError("worker result identities do not match the frozen cohort")
    if len(set(record_identities)) != len(record_identities):
        raise ValueError("worker results contain duplicate identities")
    valid_states = {"PASS", "FAIL", "NOT_EVALUABLE"}
    expected_backend = backend_label(backend)
    for record in records:
        identity = str(record["identity"])
        asset = expected_by_identity[identity]
        if record.get("state") not in valid_states:
            raise ValueError(f"{identity}: invalid result state {record.get('state')!r}")
        if record.get("backend") != expected_backend:
            raise ValueError(f"{identity}: backend label mismatch")
        if record.get("runner_sha256") != runner_hash:
            raise ValueError(f"{identity}: runner SHA-256 binding mismatch")
        if record.get("cohort_manifest_sha256") != readiness["cohort_manifest_sha256"]:
            raise ValueError(f"{identity}: cohort SHA-256 binding mismatch")
        if record.get("protocol_sha256") != readiness["protocol_sha256"]:
            raise ValueError(f"{identity}: protocol SHA-256 binding mismatch")
        if record.get("amendment_sha256") != readiness["amendment_sha256"]:
            raise ValueError(f"{identity}: amendment SHA-256 binding mismatch")
        if record.get("run_config_sha256") != run_config_hash:
            raise ValueError(f"{identity}: run-config SHA-256 binding mismatch")
        for key in (
            "raw_annotation_record_count",
            "semantic_joint_count",
            "semantic_dof_count",
            "exported_movable_joint_element_count",
        ):
            if record.get(key) != asset.get(key):
                raise ValueError(f"{identity}: result denominator drift for {key}")
        complete = record.get("evaluation_complete") is True
        if complete != (record.get("state") in {"PASS", "FAIL"}):
            strict_timeout = (
                backend == "strict"
                and record.get("state") == "FAIL"
                and record.get("reason") == "TIMEOUT_COUNTS_AS_FAIL"
                and not complete
            )
            if not strict_timeout:
                raise ValueError(f"{identity}: state/evaluation-complete mismatch")
        if backend == "proxy" and complete:
            dofs = int(record["semantic_dof_count"])
            joints = int(record["semantic_joint_count"])
            expected_single = SINGLE_STATES * dofs
            expected_multi = MULTI_STATES if dofs > 1 else 0
            expected_queries = 1 + expected_single + expected_multi
            if int(record["single_state_count"]) != expected_single:
                raise ValueError(f"{identity}: single-state denominator mismatch")
            if int(record["multi_state_count"]) != expected_multi:
                raise ValueError(f"{identity}: Sobol-state denominator mismatch")
            if int(record["evaluated_collision_state_count"]) != expected_queries:
                raise ValueError(f"{identity}: collision-query denominator mismatch")
            penetration_states = int(record["penetration_state_count"])
            retained_states = int(record["retained_contact_state_count"])
            within_tolerance_states = int(
                record["within_tolerance_contact_state_count"]
            )
            if int(record["collision_state_count"]) != penetration_states:
                raise ValueError(f"{identity}: collision/penetration state drift")
            if not (
                0 <= penetration_states <= retained_states <= expected_queries
                and 0 <= within_tolerance_states <= retained_states
            ):
                raise ValueError(f"{identity}: invalid contact-state containment")
            if retained_states != penetration_states + within_tolerance_states:
                raise ValueError(f"{identity}: contact-state decomposition mismatch")
            retained_contacts = int(record["retained_contact_count"])
            penetrating_contacts = int(record["penetrating_contact_count"])
            within_tolerance_contacts = int(record["within_tolerance_contact_count"])
            if retained_contacts != penetrating_contacts + within_tolerance_contacts:
                raise ValueError(f"{identity}: retained-contact decomposition mismatch")
            coordinate_rows = record.get("single_coordinate_results")
            pid_rows = record.get("semantic_joint_single_sweep_results")
            if not isinstance(coordinate_rows, list) or len(coordinate_rows) != dofs:
                raise ValueError(f"{identity}: scalar-coordinate result denominator drift")
            if not isinstance(pid_rows, list) or len(pid_rows) != joints:
                raise ValueError(f"{identity}: semantic-pid result denominator drift")
            if int(record["joint_pass_count"]) != sum(
                bool(row.get("pass")) for row in pid_rows
            ):
                raise ValueError(f"{identity}: semantic-joint pass count drift")
            witness = record.get("deepest_penetration_witness")
            first_witness = record.get("first_penetration_witness")
            penetration_failure = record.get("reason") in {
                "PENETRATION_BEYOND_TOLERANCE_OBSERVED",
                "DRIVE_READBACK_AND_PENETRATION_FAILURE",
            }
            if penetration_failure:
                if witness is None or first_witness is None:
                    raise ValueError(f"{identity}: penetration failure lacks witness")
                if float(witness.get("penetration_depth_m", 0.0)) <= PENETRATION_TOLERANCE_M:
                    raise ValueError(f"{identity}: penetration witness is within tolerance")
                witness_depth = float(witness["penetration_depth_m"])
                witness_distance = float(witness["contact_distance_m"])
                if not close(witness_depth, -witness_distance):
                    raise ValueError(f"{identity}: penetration witness sign drift")
                if not close(witness_depth, float(record["max_penetration_m"])):
                    raise ValueError(f"{identity}: deepest witness is not asset maximum")
                if float(witness["penetration_tolerance_m"]) != PENETRATION_TOLERANCE_M:
                    raise ValueError(f"{identity}: penetration witness tolerance drift")
                if not witness.get("link_name_a") or not witness.get("link_name_b"):
                    raise ValueError(f"{identity}: penetration witness lacks link names")
                if penetration_states <= 0:
                    raise ValueError(f"{identity}: penetration failure lacks failing state")
            elif witness is not None or first_witness is not None or penetration_states:
                raise ValueError(f"{identity}: non-penetration outcome has penetration evidence")
    counts = Counter(record["state"] for record in records)
    complete_records = [record for record in records if record["evaluation_complete"]]
    complete_counts = Counter(record["state"] for record in complete_records)
    denominators = {
        "motion_intent_assets": len(assets),
        "motion_evaluable_assets": len(complete_records),
        "pass_assets": counts["PASS"],
        "fail_assets": counts["FAIL"],
        "not_evaluable_assets": counts["NOT_EVALUABLE"],
        "completed_fail_assets": complete_counts["FAIL"],
        "intent_fail_without_complete_evaluation_assets": (
            counts["FAIL"] - complete_counts["FAIL"]
        ),
        "raw_annotation_records": sum(asset["raw_annotation_record_count"] for asset in assets),
        "semantic_joints": sum(asset["semantic_joint_count"] for asset in assets),
        "semantic_dofs": sum(asset["semantic_dof_count"] for asset in assets),
        "exported_movable_joint_elements": sum(
            asset["exported_movable_joint_element_count"] for asset in assets
        ),
        "evaluable_semantic_joints": sum(
            record["semantic_joint_count"]
            for record in complete_records
        ),
        "evaluable_semantic_dofs": sum(
            record["semantic_dof_count"]
            for record in complete_records
        ),
    }
    return {
        "protocol_id": f"nano3d_table6_artiverse_{backend}_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "backend": backend,
        "backend_python": readiness["backend_python"],
        "runner_sha256": runner_hash,
        "run_config_sha256": run_config_hash,
        "protocol_sha256": readiness["protocol_sha256"],
        "amendment_sha256": readiness["amendment_sha256"],
        "cohort_manifest_sha256": readiness["cohort_manifest_sha256"],
        "denominators": denominators,
        "load_success_assets": sum(bool(record["load_success"]) for record in records),
        "drive_success_assets": sum(bool(record["drive_success"]) for record in records),
        "metadata_warning_assets": sum(bool(record["metadata_warnings"]) for record in records),
        "metadata_warning_count": sum(len(record["metadata_warnings"]) for record in records),
        "joint_pass_count": sum(int(record["joint_pass_count"]) for record in records),
        "joint_pass_definition": (
            "semantic pid passes iff every scalar coordinate passes its 11-state "
            "single-coordinate sweep; the 64-state Sobol path contributes only to "
            "the asset collision result"
        ),
        "single_state_count": sum(int(record["single_state_count"]) for record in records),
        "multi_state_count": sum(int(record["multi_state_count"]) for record in records),
        "evaluated_collision_state_count": sum(
            int(record["evaluated_collision_state_count"]) for record in records
        ),
        "evaluated_collision_state_definition": (
            "PyBullet configuration queries including repeated configurations"
            if backend == "proxy"
            else "unique cached exact-FCL configurations"
        ),
        "collision_state_count": sum(
            int(record["collision_state_count"]) for record in records
        ),
        "penetration_state_count": sum(
            int(record["penetration_state_count"]) for record in records
        ),
        "retained_contact_state_count": sum(
            int(record["retained_contact_state_count"]) for record in records
        ),
        "within_tolerance_contact_state_count": sum(
            int(record["within_tolerance_contact_state_count"])
            for record in records
        ),
        "retained_contact_count": sum(
            int(record["retained_contact_count"]) for record in records
        ),
        "penetrating_contact_count": sum(
            int(record["penetrating_contact_count"]) for record in records
        ),
        "within_tolerance_contact_count": sum(
            int(record["within_tolerance_contact_count"]) for record in records
        ),
        "penetration_tolerance_m": PENETRATION_TOLERANCE_M,
        "penetration_failure_definition": (
            "retained PyBullet contactDistance < -1e-6 m after fixed-cluster and "
            "semantic-pid adjacency filtering"
        ),
        "penetration_witness_assets": sum(
            record["deepest_penetration_witness"] is not None for record in records
        ),
        "minimum_contact_distance_m": min(
            (
                float(record["minimum_contact_distance_m"])
                for record in records
                if record["minimum_contact_distance_m"] is not None
            ),
            default=None,
        ),
        "result_reason_counts": dict(
            sorted(Counter(str(record.get("reason")) for record in records).items())
        ),
        "asset_pass_rate_over_intent": counts["PASS"] / len(assets),
        "asset_pass_rate_over_evaluable": (
            complete_counts["PASS"] / len(complete_records)
            if complete_records
            else None
        ),
        "not_evaluable_reasons": dict(
            sorted(
                Counter(
                    str(record.get("reason"))
                    for record in records
                    if record["state"] == "NOT_EVALUABLE"
                ).items()
            )
        ),
        "semantic_accuracy_boundary": {
            "joint_type_accuracy": "N/A_NO_INDEPENDENT_GOLD",
            "joint_recall": "N/A_NO_INDEPENDENT_GOLD",
            "parent_child_accuracy": "N/A_NO_INDEPENDENT_GOLD",
            "axis_origin_limit_accuracy": "N/A_NO_INDEPENDENT_GOLD",
            "collision_result": (
                "discrete penetration-only proxy on frozen semantic trajectories"
                if backend == "proxy"
                else "exact-FCL adaptive continuous certificate on frozen evaluated paths"
            ),
        },
    }


def motion_report(summary: dict[str, Any]) -> str:
    denominators = summary["denominators"]
    label = (
        "discrete PyBullet collision proxy"
        if summary["backend"] == "proxy"
        else "exact-FCL adaptive continuous path certificate"
    )
    lines = [
        "# Artiverse Table 6 motion audit",
        "",
        f"Backend: {label}",
        "",
        "## Frozen denominators",
        "",
        f"- motion-intent assets: {denominators['motion_intent_assets']}",
        f"- evaluable assets: {denominators['motion_evaluable_assets']}",
        f"- pass / fail / not evaluable: {denominators['pass_assets']} / {denominators['fail_assets']} / {denominators['not_evaluable_assets']}",
        f"- raw annotation records: {denominators['raw_annotation_records']}",
        f"- semantic joints: {denominators['semantic_joints']}",
        f"- semantic DoFs: {denominators['semantic_dofs']}",
        f"- exported movable URDF elements: {denominators['exported_movable_joint_elements']}",
        "",
        "## Results",
        "",
        f"- load success: {summary['load_success_assets']}/{denominators['motion_intent_assets']}",
        f"- drive success: {summary['drive_success_assets']}/{denominators['motion_intent_assets']}",
        f"- semantic-joint path pass: {summary['joint_pass_count']}/{denominators['evaluable_semantic_joints']}",
        f"- asset pass over intent: {denominators['pass_assets']}/{denominators['motion_intent_assets']}",
        f"- penetration states (>1e-6 m): {summary['penetration_state_count']}/{summary['evaluated_collision_state_count']}",
        f"- retained-contact states within tolerance: {summary['within_tolerance_contact_state_count']}/{summary['evaluated_collision_state_count']}",
        "",
        "## Claim boundary",
        "",
        "The result applies only to the frozen 11-state single-DoF paths and 64-state Sobol path sequence. A state fails only when retained PyBullet contactDistance is below -1e-6 m; contact manifolds within tolerance are diagnostic only. Composite proxy links are collapsed for semantic parent-child collision exclusions. It is not independent gold for joint type, recall, parent-child, axis, origin, or limits.",
    ]
    return "\n".join(lines) + "\n"


def execute_backend(
    assets: list[dict[str, Any]],
    backend: str,
    output: Path,
    workers: int,
    timeout: float,
    readiness: dict[str, Any],
    backend_python: Path,
) -> int:
    runner_hash = sha256_file(SCRIPT)
    cohort_hash = str(readiness["cohort_manifest_sha256"])
    backend_environment = probe_backend_environment(backend_python)
    run_config = {
        "schema_version": "1.0.0",
        "backend": backend,
        "backend_label": backend_label(backend),
        "timeout_s_per_asset": float(timeout),
        "single_joint_states": SINGLE_STATES,
        "multi_joint_sobol_states": MULTI_STATES,
        "sobol_scramble": False,
        "proxy_penetration_tolerance_m": (
            PENETRATION_TOLERANCE_M if backend == "proxy" else None
        ),
        "proxy_penetration_predicate": (
            "retained contactDistance < -proxy_penetration_tolerance_m"
            if backend == "proxy"
            else None
        ),
        "proxy_self_collision_flags": (
            "URDF_USE_INERTIA_FROM_FILE | URDF_USE_SELF_COLLISION | "
            "URDF_USE_SELF_COLLISION_INCLUDE_PARENT"
            if backend == "proxy"
            else None
        ),
        "proxy_contact_filter": (
            "exclude same fixed cluster and semantic-pid parent-child adjacency after "
            "transparent composite-proxy collapse"
            if backend == "proxy"
            else None
        ),
        "proxy_witness_policy": (
            "store first and deepest penetration witness plus per-coordinate/pid results"
            if backend == "proxy"
            else None
        ),
        "adaptive_max_depth": ADAPTIVE_MAX_DEPTH if backend == "strict" else None,
        "absolute_clearance_m": ABSOLUTE_CLEARANCE_M if backend == "strict" else None,
        "clearance_bbox_fraction": (
            CLEARANCE_BBOX_FRACTION if backend == "strict" else None
        ),
        "cohort_manifest_sha256": cohort_hash,
        "protocol_sha256": readiness["protocol_sha256"],
        "amendment_sha256": readiness["amendment_sha256"],
        "runner_sha256": runner_hash,
        "backend_python_argument": str(backend_python),
        "backend_environment": backend_environment,
    }
    run_config_hash = sha256_json(run_config)
    run_id = f"{cohort_hash[:12]}_{runner_hash[:12]}_{run_config_hash[:12]}"
    backend_root = output / backend
    run_root = backend_root / "runs" / run_id
    backend_root.mkdir(parents=True, exist_ok=True)
    lock_path = backend_root / f"{run_id}.lock"
    lock_handle = lock_path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock_handle.close()
        raise RuntimeError(f"run is already active: {run_id}") from exc
    started_at = datetime.now(timezone.utc).isoformat()
    run_state_path = backend_root / "run_state.json"
    run_state = {
        "state": "RUNNING",
        "backend": backend,
        "run_id": run_id,
        "run_root": str(run_root.relative_to(WORKSPACE)),
        "started_at": started_at,
        "cohort_manifest_sha256": cohort_hash,
        "runner_sha256": runner_hash,
        "run_config_sha256": run_config_hash,
        "intent_size": len(assets),
    }
    write_json(run_state_path, run_state)
    write_json(run_root / "run_config.json", run_config)
    write_json(run_root / "run_state.json", run_state)
    for name in ("asset_records.jsonl", "summary.json", "report.md"):
        canonical = backend_root / name
        if canonical.is_file():
            canonical.unlink()
    required = readiness["dependencies"][backend]
    try:
        if not all(required.values()):
            records = [
                synthetic_not_evaluable(
                    asset,
                    backend,
                    "BACKEND_DEPENDENCY_UNAVAILABLE",
                    runner_hash=runner_hash,
                    cohort_hash=cohort_hash,
                    run_config_hash=run_config_hash,
                    run_config=run_config,
                )
                for asset in assets
            ]
        else:
            records = []
            with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, workers)) as pool:
                futures = {
                    pool.submit(
                        run_one,
                        asset,
                        backend,
                        run_root,
                        timeout,
                        cohort_hash,
                        runner_hash,
                        run_config_hash,
                        run_config,
                        backend_python,
                    ): asset
                    for asset in assets
                }
                for index, future in enumerate(concurrent.futures.as_completed(futures), 1):
                    record = future.result()
                    records.append(record)
                    print(
                        f"{index}/{len(assets)} {record['identity']} {record['state']}",
                        flush=True,
                    )
        if sha256_file(SCRIPT) != runner_hash:
            raise RuntimeError("runner changed during execution")
        records.sort(key=lambda record: record["identity"])
        summary = aggregate_results(
            assets,
            records,
            backend,
            readiness,
            runner_hash,
            run_config_hash,
        )
        summary["run_id"] = run_id
        summary["run_root"] = str(run_root.relative_to(WORKSPACE))
        summary["run_state"] = "COMPLETE"
        report_text = motion_report(summary)
        run_records = run_root / "asset_records.jsonl"
        run_summary = run_root / "summary.json"
        run_report = run_root / "report.md"
        write_jsonl(run_records, records)
        write_json(run_summary, summary)
        run_report.parent.mkdir(parents=True, exist_ok=True)
        temporary_report = run_report.with_suffix(".md.tmp")
        temporary_report.write_text(report_text, encoding="utf-8")
        os.replace(temporary_report, run_report)
        artifact_hashes = {
            "asset_records.jsonl": sha256_file(run_records),
            "summary.json": sha256_file(run_summary),
            "report.md": sha256_file(run_report),
        }
        completed_state = {
            **run_state,
            "state": "COMPLETE",
            "completed_at": datetime.now(timezone.utc).isoformat(),
            "artifact_sha256": artifact_hashes,
        }
        write_json(run_root / "run_state.json", completed_state)
        completed_state["run_state_sha256"] = sha256_file(run_root / "run_state.json")
        write_jsonl(backend_root / "asset_records.jsonl", records)
        write_json(backend_root / "summary.json", summary)
        canonical_report = backend_root / "report.md"
        temporary_report = canonical_report.with_suffix(".md.tmp")
        temporary_report.write_text(report_text, encoding="utf-8")
        os.replace(temporary_report, canonical_report)
        write_json(run_state_path, completed_state)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0
    except Exception as exc:
        failed_state = {
            **run_state,
            "state": "FAILED",
            "failed_at": datetime.now(timezone.utc).isoformat(),
            "error": f"{type(exc).__name__}: {exc}",
        }
        write_json(run_root / "run_state.json", failed_state)
        write_json(run_state_path, failed_state)
        raise
    finally:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
        lock_handle.close()


def main() -> int:
    args = parse_args()
    if args.phase == "_worker":
        if args.worker_input is None or args.worker_output is None:
            raise ValueError("_worker requires --worker-input and --worker-output")
        return worker(
            contained(args.worker_input, must_exist=True),
            contained(args.worker_output, must_exist=False),
        )

    artiverse = contained(args.artiverse_root, must_exist=True)
    static_root = contained(args.static_root, must_exist=True)
    protocol = contained(args.protocol, must_exist=True)
    backend_python = backend_executable(args.backend_python)
    output = contained(args.output, must_exist=False)
    runtime_root = contained(EXP_ROOT / "runtime", must_exist=True)
    output.relative_to(runtime_root)
    if output == static_root:
        raise ValueError("motion output may not overwrite the static artifact root")
    if output == artiverse or output.is_relative_to(artiverse):
        raise ValueError("motion output may not overlap the read-only Artiverse payload")
    output.mkdir(parents=True, exist_ok=True)
    readiness, assets = validate_frozen_inputs(
        artiverse, static_root, protocol, backend_python
    )
    write_json(output / "motion_readiness.json", readiness)
    print(json.dumps({"state": readiness["state"], "blockers": readiness["blockers"]}))
    if readiness["state"] != "READY":
        return 2
    if args.phase == "preflight":
        return 0
    backend = args.phase
    timeout = args.timeout if args.timeout is not None else (600.0 if backend == "proxy" else 3600.0)
    return execute_backend(
        assets,
        backend,
        output,
        args.workers,
        timeout,
        readiness,
        backend_python,
    )


if __name__ == "__main__":
    raise SystemExit(main())
