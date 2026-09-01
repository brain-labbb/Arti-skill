#!/usr/bin/env python3
"""Run the paper-aligned articulation evaluation for the frozen 33-asset cohort.

The Nova3D paper reports a 12-asset/59-joint articulation case study.  This
harness keeps the same separations while using the locally frozen 33 assets:

* Tier 1: native movable-joint exposure and joints per asset.
* Tier 2: static URDF metadata audit and generic-range finding.
* Tier 3: 11 samples over each nonzero declared single-joint range plus 64
  deterministic Sobol configurations for assets with >1 functional-motion
  joint. Bounded zero-width joints remain in the declared-joint count but are
  excluded from motion-state denominators.

The physics result is deliberately called a proxy.  There is no local joint
semantic gold/spec, so type accuracy, joint recall, parent-child accuracy,
axis-on-moving-part, and rest-pose-frozen cannot be claimed from these URDFs.
The harness never edits source assets; all outputs are under exp/runtime.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from statistics import mean
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_nano3d_asset_pilot import asset_identity  # noqa: E402


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = WORKSPACE / "exp/runtime/nano3d_articulation_paper"
DEFAULT_INPUT_MANIFEST = (
    WORKSPACE / "exp/runtime/nano3d_glb_n33/input_packages/input_manifest.json"
)
SINGLE_SAMPLES = 11
MULTI_SAMPLES = 64
GENERIC_REVOLUTE_DEG = 300.0
ZERO_WIDTH_TOLERANCE = 1e-12
PENETRATION_TOLERANCE_M = 1e-6
RESET_POSE_TOLERANCE = 1e-9
COLLISION_POLICY = (
    "URDF_USE_SELF_COLLISION | URDF_USE_SELF_COLLISION_EXCLUDE_PARENT"
)
SELF_COLLISION_SMOKE_URDF = """<?xml version="1.0"?>
<robot name="self_collision_flag_smoke">
  <link name="base"/>
  <link name="left"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="right"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <joint name="left_joint" type="fixed"><parent link="base"/><child link="left"/></joint>
  <joint name="right_joint" type="fixed"><parent link="base"/><child link="right"/></joint>
</robot>
"""


def load_dependencies() -> tuple[Any | None, Any | None, Any | None, str | None]:
    try:
        import numpy as np

        # urdfpy 0.0.22 uses aliases removed in newer NumPy releases.  This
        # compatibility shim is process-local and does not edit the package.
        for name, value in {"float": float, "int": int}.items():
            if not hasattr(np, name):
                setattr(np, name, value)
        from urdfpy import URDF
        from scipy.stats import qmc
        import pybullet
    except Exception as exc:  # noqa: BLE001
        return None, None, None, f"{type(exc).__name__}: {exc}"
    return np, URDF, pybullet, None


NP, URDF_CLASS, PYBULLET, DEPENDENCY_ERROR = load_dependencies()


def pybullet_collision_flags() -> int | None:
    if PYBULLET is None:
        return None
    return int(
        PYBULLET.URDF_USE_INERTIA_FROM_FILE
        | PYBULLET.URDF_USE_SELF_COLLISION
        | PYBULLET.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
    )


def run_self_collision_flag_smoke(output: Path) -> dict[str, Any]:
    fixture = output / "self_collision_flag_smoke.urdf"
    fixture.write_text(SELF_COLLISION_SMOKE_URDF, encoding="utf-8")
    if PYBULLET is None:
        payload = {
            "protocol": "nano3d_pybullet_self_collision_flag_smoke_v1",
            "status": "BLOCKED",
            "dependency_error": DEPENDENCY_ERROR,
        }
    else:
        policies = {
            "none": 0,
            "self_collision": PYBULLET.URDF_USE_SELF_COLLISION,
            "exclude_parent_only": PYBULLET.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            "self_plus_exclude_parent": (
                PYBULLET.URDF_USE_SELF_COLLISION
                | PYBULLET.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
            ),
        }
        contacts: dict[str, int] = {}
        client = PYBULLET.connect(PYBULLET.DIRECT)
        try:
            for name, flags in policies.items():
                body = PYBULLET.loadURDF(str(fixture), useFixedBase=True, flags=flags)
                PYBULLET.performCollisionDetection()
                contacts[name] = len(PYBULLET.getContactPoints(bodyA=body, bodyB=body))
                PYBULLET.removeBody(body)
        finally:
            PYBULLET.disconnect(client)
        passed = (
            contacts["none"] == 0
            and contacts["exclude_parent_only"] == 0
            and contacts["self_collision"] > 0
            and contacts["self_plus_exclude_parent"] > 0
        )
        payload = {
            "protocol": "nano3d_pybullet_self_collision_flag_smoke_v1",
            "status": "PASS" if passed else "FAIL",
            "pybullet_api_version": int(PYBULLET.getAPIVersion()),
            "fixture": fixture.name,
            "fixture_design": "two overlapping sibling collision boxes attached to a common fixed parent",
            "contacts": contacts,
            "conclusion": (
                "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT does not enable self-collision; "
                "URDF_USE_SELF_COLLISION must also be set."
            ),
        }
    (output / "self_collision_flag_smoke.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    if payload["status"] != "PASS":
        raise RuntimeError(f"self-collision flag smoke failed: {payload}")
    return payload


def finite_vector(node: ET.Element | None, field: str) -> bool:
    if node is None:
        return True
    raw = node.attrib.get(field)
    if not raw:
        return True
    try:
        values = [float(x) for x in raw.split()]
    except ValueError:
        return False
    return len(values) == 3 and all(math.isfinite(x) for x in values)


def vector_norm(node: ET.Element | None) -> float | None:
    if node is None or not node.attrib.get("xyz"):
        return None
    try:
        values = [float(x) for x in node.attrib["xyz"].split()]
    except ValueError:
        return None
    return math.sqrt(sum(x * x for x in values)) if len(values) == 3 else None


def parse_joint_metadata(asset_dir: Path) -> list[dict[str, Any]]:
    root = ET.parse(asset_dir / "model.urdf").getroot()
    links = {node.attrib.get("name", "") for node in root.findall("link")}
    rows: list[dict[str, Any]] = []
    for node in root.findall("joint"):
        joint_type = node.attrib.get("type", "")
        if joint_type == "fixed":
            continue
        parent_node = node.find("parent")
        child_node = node.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        axis = node.find("axis")
        origin = node.find("origin")
        limit = node.find("limit")
        lower: float | None = None
        upper: float | None = None
        if limit is not None:
            try:
                lower = float(limit.attrib["lower"])
                upper = float(limit.attrib["upper"])
            except (KeyError, TypeError, ValueError):
                lower = upper = None
        bounded = (
            joint_type in {"revolute", "prismatic"}
            and lower is not None
            and upper is not None
            and math.isfinite(lower)
            and math.isfinite(upper)
            and lower <= upper
        )
        span = upper - lower if bounded else None
        zero_width_bounded = bool(
            bounded and span is not None and abs(span) <= ZERO_WIDTH_TOLERANCE
        )
        functional_motion = bool(
            joint_type == "continuous"
            or (bounded and span is not None and span > ZERO_WIDTH_TOLERANCE)
        )
        axis_ok = (
            axis is not None
            and finite_vector(axis, "xyz")
            and (vector_norm(axis) or 0.0) > 1e-9
        )
        origin_ok = origin is None or (finite_vector(origin, "xyz") and finite_vector(origin, "rpy"))
        rows.append(
            {
                "name": node.attrib.get("name", ""),
                "type": joint_type,
                "parent": parent,
                "child": child,
                "parent_child_structurally_valid": parent in links and child in links,
                "axis_metadata_valid": axis_ok,
                "origin_metadata_valid": origin_ok,
                "bounded": bounded,
                "lower": lower,
                "upper": upper,
                "declared_span": span,
                "zero_width_bounded": zero_width_bounded,
                "functional_motion": functional_motion,
                "generic_revolute_range": bool(
                    joint_type == "revolute"
                    and bounded
                    and math.degrees(upper - lower) >= GENERIC_REVOLUTE_DEG  # type: ignore[operator]
                ),
            }
        )
    return rows


def parse_physical_metadata(asset_dir: Path) -> dict[str, int | bool]:
    root = ET.parse(asset_dir / "model.urdf").getroot()
    links = root.findall("link")
    valid_inertial = 0
    collision = 0
    complete = 0
    for link in links:
        has_collision = bool(link.findall("collision"))
        collision += int(has_collision)
        inertial = link.find("inertial")
        inertial_ok = False
        if inertial is not None:
            mass = inertial.find("mass")
            tensor = inertial.find("inertia")
            try:
                mass_value = float(mass.attrib["value"]) if mass is not None else math.nan
                diagonal = [
                    float(tensor.attrib[key]) if tensor is not None else math.nan
                    for key in ("ixx", "iyy", "izz")
                ]
                inertial_ok = (
                    math.isfinite(mass_value)
                    and mass_value > 0
                    and all(math.isfinite(value) and value > 0 for value in diagonal)
                )
            except (KeyError, TypeError, ValueError):
                inertial_ok = False
        valid_inertial += int(inertial_ok)
        complete += int(inertial_ok and has_collision)
    return {
        "link_count": len(links),
        "valid_inertial_link_count": valid_inertial,
        "collision_link_count": collision,
        "physical_metadata_complete_link_count": complete,
        "physical_metadata_complete_asset": bool(links) and complete == len(links),
    }


def joint_samples(meta: dict[str, Any]) -> list[float]:
    if meta["bounded"]:
        return [float(x) for x in NP.linspace(meta["lower"], meta["upper"], SINGLE_SAMPLES)]
    # A continuous joint has no finite declared endpoints.  The fixed
    # operational proxy samples one full turn, including both wrap endpoints.
    return [float(x) for x in NP.linspace(-math.pi, math.pi, SINGLE_SAMPLES)]


def sobol_configurations(metadata: list[dict[str, Any]]) -> list[list[float]]:
    from scipy.stats import qmc

    sampler = qmc.Sobol(d=len(metadata), scramble=False)
    unit = sampler.random_base2(m=6)
    configurations: list[list[float]] = []
    for row in unit:
        values: list[float] = []
        for u, meta in zip(row, metadata):
            if meta["bounded"]:
                values.append(float(meta["lower"] + u * (meta["upper"] - meta["lower"])))
            else:
                values.append(float(-math.pi + u * 2.0 * math.pi))
        configurations.append(values)
    return configurations


def reset_and_detect(
    body: int, joint_indices: list[int], values: list[float]
) -> tuple[int, int, float, float, list[dict[str, Any]]]:
    for index, value in zip(joint_indices, values):
        PYBULLET.resetJointState(body, index, value)
    PYBULLET.performCollisionDetection()
    pose_error = max(
        [
            abs(float(PYBULLET.getJointState(body, index)[0]) - value)
            for index, value in zip(joint_indices, values)
        ]
        or [0.0]
    )
    if pose_error > RESET_POSE_TOLERANCE:
        raise RuntimeError(
            f"reset pose changed during collision detection: {pose_error} > {RESET_POSE_TOLERANCE}"
        )
    contacts = PYBULLET.getContactPoints(bodyA=body, bodyB=body)
    max_penetration = 0.0
    penetrating_contact_count = 0
    pairs: list[dict[str, Any]] = []
    for contact in contacts:
        distance = float(contact[8])
        penetration_depth = max(0.0, -distance)
        max_penetration = max(max_penetration, penetration_depth)
        penetrating = penetration_depth > PENETRATION_TOLERANCE_M
        penetrating_contact_count += int(penetrating)
        if len(pairs) < 8:
            pairs.append(
                {
                    "link_a": int(contact[3]),
                    "link_b": int(contact[4]),
                    "contact_distance": distance,
                    "penetration_depth": penetration_depth,
                    "penetrating_beyond_tolerance": penetrating,
                }
            )
    return len(contacts), penetrating_contact_count, max_penetration, pose_error, pairs


def run_asset(
    asset_dir: Path, frozen_identity: dict[str, Any] | None = None
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    identity = frozen_identity or asset_identity(asset_dir)
    metadata = parse_joint_metadata(asset_dir)
    physical_metadata = parse_physical_metadata(asset_dir)
    result: dict[str, Any] = {
        **identity,
        **physical_metadata,
        "urdfpy_load_success": False,
        "pybullet_load_success": False,
        "physics_collision_policy": COLLISION_POLICY,
        "physics_pose_evaluation": (
            "motors disabled; resetJointState; performCollisionDetection; no simulation step"
        ),
        "penetration_tolerance_m": PENETRATION_TOLERANCE_M,
        "single_joint_sample_count": 0,
        "single_joint_contact_free_state_count": 0,
        "single_joint_penetration_free_state_count": 0,
        "multi_joint_configuration_count": 0,
        "multi_joint_contact_free_state_count": 0,
        "multi_joint_penetration_free_state_count": 0,
        "joint_count": len(metadata),
        "functional_motion_joint_count": sum(row["functional_motion"] for row in metadata),
        "zero_width_bounded_joint_count": sum(row["zero_width_bounded"] for row in metadata),
        "zero_width_bounded_joint_names": [
            row["name"] for row in metadata if row["zero_width_bounded"]
        ],
        "non_evaluable_motion_joint_count": sum(
            not row["functional_motion"] and not row["zero_width_bounded"] for row in metadata
        ),
        "non_evaluable_motion_joint_names": [
            row["name"]
            for row in metadata
            if not row["functional_motion"] and not row["zero_width_bounded"]
        ],
        "revolute_count": sum(row["type"] == "revolute" for row in metadata),
        "prismatic_count": sum(row["type"] == "prismatic" for row in metadata),
        "continuous_count": sum(row["type"] == "continuous" for row in metadata),
        "axis_metadata_valid_count": sum(row["axis_metadata_valid"] for row in metadata),
        "origin_metadata_valid_count": sum(row["origin_metadata_valid"] for row in metadata),
        "bounded_joint_count": sum(row["bounded"] for row in metadata),
        "valid_bounded_limit_count": sum(row["bounded"] for row in metadata),
        "parent_child_structurally_valid_count": sum(
            row["parent_child_structurally_valid"] for row in metadata
        ),
        "generic_revolute_count": sum(row["generic_revolute_range"] for row in metadata),
        "motor_disabled_joint_count": 0,
        "joint_single_sweep_contact_free_pass_count": 0,
        "joint_single_sweep_penetration_free_pass_count": 0,
        "joint_single_sweep_total": sum(row["functional_motion"] for row in metadata),
        "asset_single_sweep_contact_free_pass": False,
        "asset_single_sweep_penetration_free_pass": False,
        "asset_full_range_contact_free_pass": False,
        "asset_full_range_penetration_free_pass": False,
        "rest_state_contact_free": None,
        "rest_state_penetration_free": None,
        "rest_pose_frozen": "not_evaluable_missing_pre_articulation_artifact",
        "min_clearance": None,
        "max_penetration_depth": 0.0,
        "max_reset_pose_error": 0.0,
        "issues": [],
    }
    state_records: list[dict[str, Any]] = []

    if URDF_CLASS is not None:
        try:
            URDF_CLASS.load(str(asset_dir / "model.urdf"))
            result["urdfpy_load_success"] = True
        except Exception as exc:  # noqa: BLE001
            result["issues"].append(f"urdfpy_load: {type(exc).__name__}: {exc}")
    if PYBULLET is None:
        result["issues"].append(f"pybullet_unavailable: {DEPENDENCY_ERROR}")
        return result, state_records

    client = PYBULLET.connect(PYBULLET.DIRECT)
    body: int | None = None
    try:
        flags = pybullet_collision_flags()
        assert flags is not None
        body = PYBULLET.loadURDF(str(asset_dir / "model.urdf"), useFixedBase=True, flags=flags)
        result["pybullet_load_success"] = True
        info_by_name: dict[str, tuple[int, int]] = {}
        for index in range(PYBULLET.getNumJoints(body)):
            info = PYBULLET.getJointInfo(body, index)
            name = info[1].decode("utf-8") if isinstance(info[1], bytes) else str(info[1])
            info_by_name[name] = (index, int(info[2]))
        aligned = [row for row in metadata if row["name"] in info_by_name]
        if len(aligned) != len(metadata):
            result["issues"].append(f"joint_mapping_missing: {len(metadata) - len(aligned)}")
        joint_indices = [info_by_name[row["name"]][0] for row in aligned]
        for index in joint_indices:
            PYBULLET.setJointMotorControl2(
                body,
                index,
                controlMode=PYBULLET.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
            )
        result["motor_disabled_joint_count"] = len(joint_indices)
        defaults = [
            float((row["lower"] + row["upper"]) / 2.0)
            if row["bounded"]
            else 0.0
            for row in aligned
        ]
        motion_positions = [
            position for position, row in enumerate(aligned) if row["functional_motion"]
        ]
        motion_metadata = [aligned[position] for position in motion_positions]

        contact_count, penetrating_count, penetration, pose_error, pairs = reset_and_detect(
            body, joint_indices, defaults
        )
        contact_free = contact_count == 0
        penetration_free = penetrating_count == 0
        result["rest_state_contact_free"] = contact_free
        result["rest_state_penetration_free"] = penetration_free
        result["max_penetration_depth"] = max(result["max_penetration_depth"], penetration)
        result["max_reset_pose_error"] = max(result["max_reset_pose_error"], pose_error)
        state_records.append(
            {
                **identity,
                "phase": "rest",
                "sample_index": 0,
                "contact_free": contact_free,
                "penetration_free": penetration_free,
                "contact_count": contact_count,
                "penetrating_contact_count": penetrating_count,
                "penetration_tolerance_m": PENETRATION_TOLERANCE_M,
                "max_penetration_depth": penetration,
                "reset_pose_max_abs_error": pose_error,
                "contact_pairs": pairs,
            }
        )

        all_single_contact_free = True
        all_single_penetration_free = True
        for joint_position, meta in zip(motion_positions, motion_metadata):
            joint_contact_free = True
            joint_penetration_free = True
            for sample_index, value in enumerate(joint_samples(meta)):
                values = list(defaults)
                values[joint_position] = value
                contact_count, penetrating_count, penetration, pose_error, pairs = reset_and_detect(
                    body, joint_indices, values
                )
                contact_free = contact_count == 0
                penetration_free = penetrating_count == 0
                joint_contact_free = joint_contact_free and contact_free
                joint_penetration_free = joint_penetration_free and penetration_free
                all_single_contact_free = all_single_contact_free and contact_free
                all_single_penetration_free = all_single_penetration_free and penetration_free
                result["single_joint_sample_count"] += 1
                result["single_joint_contact_free_state_count"] += int(contact_free)
                result["single_joint_penetration_free_state_count"] += int(penetration_free)
                result["max_penetration_depth"] = max(result["max_penetration_depth"], penetration)
                result["max_reset_pose_error"] = max(result["max_reset_pose_error"], pose_error)
                state_records.append(
                    {
                        **identity,
                        "phase": "single_joint_sweep",
                        "joint_name": meta["name"],
                        "joint_type": meta["type"],
                        "sample_index": sample_index,
                        "sample_count": SINGLE_SAMPLES,
                        "value": value,
                        "contact_free": contact_free,
                        "penetration_free": penetration_free,
                        "contact_count": contact_count,
                        "penetrating_contact_count": penetrating_count,
                        "penetration_tolerance_m": PENETRATION_TOLERANCE_M,
                        "max_penetration_depth": penetration,
                        "reset_pose_max_abs_error": pose_error,
                        "contact_pairs": pairs,
                    }
                )
            result["joint_single_sweep_contact_free_pass_count"] += int(joint_contact_free)
            result["joint_single_sweep_penetration_free_pass_count"] += int(
                joint_penetration_free
            )
        result["asset_single_sweep_contact_free_pass"] = all_single_contact_free
        result["asset_single_sweep_penetration_free_pass"] = all_single_penetration_free

        all_full_contact_free = all_single_contact_free
        all_full_penetration_free = all_single_penetration_free
        if len(motion_metadata) > 1:
            for sample_index, motion_values in enumerate(sobol_configurations(motion_metadata)):
                values = list(defaults)
                for position, value in zip(motion_positions, motion_values):
                    values[position] = value
                contact_count, penetrating_count, penetration, pose_error, pairs = reset_and_detect(
                    body, joint_indices, values
                )
                contact_free = contact_count == 0
                penetration_free = penetrating_count == 0
                all_full_contact_free = all_full_contact_free and contact_free
                all_full_penetration_free = all_full_penetration_free and penetration_free
                result["multi_joint_configuration_count"] += 1
                result["multi_joint_contact_free_state_count"] += int(contact_free)
                result["multi_joint_penetration_free_state_count"] += int(penetration_free)
                result["max_penetration_depth"] = max(result["max_penetration_depth"], penetration)
                result["max_reset_pose_error"] = max(result["max_reset_pose_error"], pose_error)
                state_records.append(
                    {
                        **identity,
                        "phase": "multi_joint_sobol",
                        "sample_index": sample_index,
                        "sample_count": MULTI_SAMPLES,
                        "values": motion_values,
                        "joint_names": [row["name"] for row in motion_metadata],
                        "contact_free": contact_free,
                        "penetration_free": penetration_free,
                        "contact_count": contact_count,
                        "penetrating_contact_count": penetrating_count,
                        "penetration_tolerance_m": PENETRATION_TOLERANCE_M,
                        "max_penetration_depth": penetration,
                        "reset_pose_max_abs_error": pose_error,
                        "contact_pairs": pairs,
                    }
                )
        result["asset_full_range_contact_free_pass"] = all_full_contact_free
        result["asset_full_range_penetration_free_pass"] = all_full_penetration_free
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"pybullet_sweep: {type(exc).__name__}: {exc}")
    finally:
        if body is not None:
            PYBULLET.removeBody(body)
        PYBULLET.disconnect(client)

    return result, state_records


def fraction(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_frozen_assets(manifest_path: Path) -> list[tuple[dict[str, Any], Path]]:
    rows = json.loads(manifest_path.read_text(encoding="utf-8"))
    if len(rows) != 33:
        raise RuntimeError(f"expected 33 frozen assets in {manifest_path}, found {len(rows)}")
    assets: list[tuple[dict[str, Any], Path]] = []
    seen: set[str] = set()
    for row in rows:
        asset_id = str(row["asset_id"])
        if asset_id in seen:
            raise RuntimeError(f"duplicate frozen asset id: {asset_id}")
        seen.add(asset_id)
        package = Path(row["copied_package"]).resolve()
        urdf = package / "model.urdf"
        if not urdf.is_file() or not (package / "assets").is_dir():
            raise RuntimeError(f"incomplete frozen package: {package}")
        actual_hash = sha256_file(urdf)
        if actual_hash != row["model_urdf_sha256"]:
            raise RuntimeError(f"frozen URDF hash mismatch: {urdf}")
        identity = asset_identity(Path(row["source_package"]).resolve())
        if identity["asset_id"] != asset_id:
            raise RuntimeError(f"asset identity mismatch for {asset_id}")
        identity["evaluation_path"] = str(package)
        identity["model_urdf_sha256"] = actual_hash
        assets.append((identity, package))
    return assets


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--input-manifest", type=Path, default=DEFAULT_INPUT_MANIFEST)
    args = parser.parse_args()
    output = args.output.resolve()
    experiment_root = (WORKSPACE / "exp").resolve()
    if output != experiment_root and experiment_root not in output.parents:
        raise RuntimeError(f"output must stay under {experiment_root}: {output}")
    output.mkdir(parents=True, exist_ok=True)
    collision_flag_smoke = run_self_collision_flag_smoke(output)
    input_manifest = args.input_manifest.resolve()
    selected = read_frozen_assets(input_manifest)
    asset_records: list[dict[str, Any]] = []
    state_records: list[dict[str, Any]] = []
    for identity, asset_dir in selected:
        record, states = run_asset(asset_dir, identity)
        asset_records.append(record)
        state_records.extend(states)

    joint_total = sum(row["joint_count"] for row in asset_records)
    bounded_total = sum(row["bounded_joint_count"] for row in asset_records)
    generic_revolute = sum(row["generic_revolute_count"] for row in asset_records)
    functional_joint_total = sum(row["functional_motion_joint_count"] for row in asset_records)
    zero_width_joint_total = sum(row["zero_width_bounded_joint_count"] for row in asset_records)
    non_evaluable_motion_joint_total = sum(
        row["non_evaluable_motion_joint_count"] for row in asset_records
    )
    single_total = sum(row["single_joint_sample_count"] for row in asset_records)
    single_contact_free = sum(
        row["single_joint_contact_free_state_count"] for row in asset_records
    )
    single_penetration_free = sum(
        row["single_joint_penetration_free_state_count"] for row in asset_records
    )
    multi_total = sum(row["multi_joint_configuration_count"] for row in asset_records)
    multi_contact_free = sum(
        row["multi_joint_contact_free_state_count"] for row in asset_records
    )
    multi_penetration_free = sum(
        row["multi_joint_penetration_free_state_count"] for row in asset_records
    )
    full_total = single_total + multi_total
    full_contact_free = single_contact_free + multi_contact_free
    full_penetration_free = single_penetration_free + multi_penetration_free
    articulated_assets = sum(row["joint_count"] > 0 for row in asset_records)
    link_total = sum(row["link_count"] for row in asset_records)
    inertial_total = sum(row["valid_inertial_link_count"] for row in asset_records)
    collision_link_total = sum(row["collision_link_count"] for row in asset_records)
    physical_complete_total = sum(
        row["physical_metadata_complete_link_count"] for row in asset_records
    )
    summary = {
        "protocol": "nano3d_articulation_paper_aligned_v3",
        "paper_reference": "Nova3D arXiv:2607.22738v1 Section 10, Tables 15-16",
        "selection": {
            "asset_count": len(asset_records),
            "joint_count": joint_total,
            "asset_manifest": str(input_manifest),
            "asset_manifest_sha256": sha256_file(input_manifest),
        },
        "sampling": {
            "single_joint_states_per_joint": SINGLE_SAMPLES,
            "multi_joint_sobol_states_per_asset_with_gt_1_functional_motion_joint": (
                MULTI_SAMPLES
            ),
            "declared_movable_joints": joint_total,
            "functional_motion_joints": functional_joint_total,
            "zero_width_bounded_joints_excluded_from_motion_sweep": zero_width_joint_total,
            "zero_width_nominal_repeated_states_excluded": (
                zero_width_joint_total * SINGLE_SAMPLES
            ),
            "non_evaluable_motion_joints_excluded": non_evaluable_motion_joint_total,
            "zero_width_tolerance": ZERO_WIDTH_TOLERANCE,
            "multi_joint_assets": sum(
                row["functional_motion_joint_count"] > 1 for row in asset_records
            ),
            "collision_policy": COLLISION_POLICY,
            "pose_evaluation": (
                "disable movable-joint motors; resetJointState; "
                "performCollisionDetection; no stepSimulation"
            ),
            "motor_control_policy": "VELOCITY_CONTROL, targetVelocity=0, force=0",
            "motor_disabled_declared_joints": (
                f"{sum(row['motor_disabled_joint_count'] for row in asset_records)}"
                f"/{joint_total}"
            ),
            "reset_pose_max_abs_error_tolerance": RESET_POSE_TOLERANCE,
            "contact_free_definition": "zero PyBullet self-contact points",
            "penetration_free_definition": (
                "no self-contact with penetration depth greater than tolerance"
            ),
            "penetration_tolerance_m": PENETRATION_TOLERANCE_M,
            "primary_collision_proxy": "penetration_free",
            "pybullet_load_flags": pybullet_collision_flags(),
            "pybullet_api_version": collision_flag_smoke["pybullet_api_version"],
            "self_collision_flag_smoke": collision_flag_smoke["status"],
            "ccd": False,
        },
        "tier1": {
            "articulable_asset_rate": f"{articulated_assets}/{len(asset_records)}",
            "articulable_asset_rate_value": fraction(articulated_assets, len(asset_records)),
            "joints_per_asset_mean": fraction(joint_total, len(asset_records)),
            "native_joint_exposure": f"{joint_total}/{joint_total}",
        },
        "tier2_static": {
            "joint_type_accuracy": "N/A: missing frozen joint semantic gold/spec",
            "joint_recall": "N/A: missing frozen expected-joint gold/spec",
            "parent_child_accuracy": "N/A: structural edges available, correctness gold absent",
            "axis_metadata_valid": f"{sum(row['axis_metadata_valid_count'] for row in asset_records)}/{joint_total}",
            "origin_metadata_valid": f"{sum(row['origin_metadata_valid_count'] for row in asset_records)}/{joint_total}",
            "bounded_limit_valid": f"{sum(row['valid_bounded_limit_count'] for row in asset_records)}/{bounded_total}",
            "generic_revolute_range": f"{generic_revolute}/{sum(row['revolute_count'] for row in asset_records)}",
            "continuous_unbounded_joints": sum(row["continuous_count"] for row in asset_records),
        },
        "tier3_functional_proxy": {
            "primary_metric": "penetration_free at 1e-6 m tolerance",
            "declared_movable_joints": joint_total,
            "functional_motion_joints": functional_joint_total,
            "zero_width_bounded_joints_excluded": zero_width_joint_total,
            "non_evaluable_motion_joints_excluded": non_evaluable_motion_joint_total,
            "single_joint_states": single_total,
            "single_joint_contact_free_states": f"{single_contact_free}/{single_total}",
            "single_joint_contact_free_rate": fraction(single_contact_free, single_total),
            "single_joint_penetration_free_states": f"{single_penetration_free}/{single_total}",
            "single_joint_penetration_free_rate": fraction(
                single_penetration_free, single_total
            ),
            "single_joint_contact_present_within_penetration_tolerance": (
                single_penetration_free - single_contact_free
            ),
            "joint_single_sweep_contact_free_pass": (
                f"{sum(row['joint_single_sweep_contact_free_pass_count'] for row in asset_records)}"
                f"/{functional_joint_total}"
            ),
            "joint_single_sweep_penetration_free_pass": (
                f"{sum(row['joint_single_sweep_penetration_free_pass_count'] for row in asset_records)}"
                f"/{functional_joint_total}"
            ),
            "multi_joint_sobol_states": multi_total,
            "multi_joint_contact_free_states": f"{multi_contact_free}/{multi_total}",
            "multi_joint_contact_free_rate": fraction(multi_contact_free, multi_total),
            "multi_joint_penetration_free_states": f"{multi_penetration_free}/{multi_total}",
            "multi_joint_penetration_free_rate": fraction(
                multi_penetration_free, multi_total
            ),
            "multi_joint_contact_present_within_penetration_tolerance": (
                multi_penetration_free - multi_contact_free
            ),
            "full_range_states": full_total,
            "full_range_contact_free_states": f"{full_contact_free}/{full_total}",
            "full_range_contact_free_rate": fraction(full_contact_free, full_total),
            "full_range_penetration_free_states": f"{full_penetration_free}/{full_total}",
            "full_range_penetration_free_rate": fraction(
                full_penetration_free, full_total
            ),
            "full_range_contact_present_within_penetration_tolerance": (
                full_penetration_free - full_contact_free
            ),
            "rest_state_contact_free_assets": (
                f"{sum(row['rest_state_contact_free'] is True for row in asset_records)}"
                f"/{len(asset_records)}"
            ),
            "rest_state_penetration_free_assets": (
                f"{sum(row['rest_state_penetration_free'] is True for row in asset_records)}"
                f"/{len(asset_records)}"
            ),
            "asset_single_sweep_contact_free_pass": (
                f"{sum(row['asset_single_sweep_contact_free_pass'] for row in asset_records)}"
                f"/{len(asset_records)}"
            ),
            "asset_single_sweep_penetration_free_pass": (
                f"{sum(row['asset_single_sweep_penetration_free_pass'] for row in asset_records)}"
                f"/{len(asset_records)}"
            ),
            "asset_full_range_contact_free_proxy": (
                f"{sum(row['asset_full_range_contact_free_pass'] for row in asset_records)}"
                f"/{len(asset_records)}"
            ),
            "asset_full_range_penetration_free_proxy": (
                f"{sum(row['asset_full_range_penetration_free_pass'] for row in asset_records)}"
                f"/{len(asset_records)}"
            ),
            "max_reset_pose_abs_error": max(
                row["max_reset_pose_error"] for row in asset_records
            ),
            "max_penetration_depth_m": max(
                row["max_penetration_depth"] for row in asset_records
            ),
            "joint_geometric_validity": "N/A: paper metric also requires axis-on-moving-part semantic judgment; collision-only proxy is reported above",
            "asset_geometric_validity": "N/A: paper metric also requires axis-on-moving-part semantic judgment; collision-only proxy is reported above",
            "rest_pose_frozen": "N/A: no pre-articulation artifact pair",
            "continuous_collision_detection": "not_run",
        },
        "evaluator_integrity": {
            "urdfpy_load_success": (
                f"{sum(row['urdfpy_load_success'] for row in asset_records)}"
                f"/{len(asset_records)}"
            ),
            "pybullet_load_success": (
                f"{sum(row['pybullet_load_success'] for row in asset_records)}"
                f"/{len(asset_records)}"
            ),
            "assets_with_issues": sum(bool(row["issues"]) for row in asset_records),
            "motor_disabled_declared_joints": (
                f"{sum(row['motor_disabled_joint_count'] for row in asset_records)}"
                f"/{joint_total}"
            ),
            "state_record_count": len(state_records),
            "expected_state_record_count": len(asset_records) + full_total,
            "max_reset_pose_abs_error": max(
                row["max_reset_pose_error"] for row in asset_records
            ),
            "reset_pose_check_pass": all(
                row["max_reset_pose_error"] <= RESET_POSE_TOLERANCE
                for row in asset_records
            ),
            "self_collision_flag_smoke": collision_flag_smoke["status"],
        },
        "panel_c_simulation_preflight": {
            "links": link_total,
            "valid_inertial_links": inertial_total,
            "collision_links": collision_link_total,
            "physical_metadata_complete_links": physical_complete_total,
            "physical_metadata_complete_link_rate": fraction(
                physical_complete_total, link_total
            ),
            "physical_metadata_complete_assets": sum(
                row["physical_metadata_complete_asset"] for row in asset_records
            ),
            "mujoco_l5": "not_run",
            "genesis_l5": "not_run",
            "pybullet_l5": "not_run; tier3 is a reset-and-detect collision proxy",
            "isaac_sim_l5": "not_run",
            "four_simulator_all_pass": None,
            "rest_stable": None,
            "worst_state_stable": None,
        },
        "limitations": [
            "This is a 33-asset local existing-export evaluation, not the paper's 12-asset generated case study.",
            "Self-collision excludes direct parent-child pairs to avoid unavoidable joint-interface contacts.",
            "PyBullet reset-pose collision detection is used; no dynamics step or continuous collision detection.",
            "Penetration-free uses a fixed 1e-6 m depth tolerance; contact-free is reported separately.",
            "No joint spec/gold is available for semantic type, recall, parent-child correctness, or axis-on-moving-part.",
            "Only links with positive finite mass/inertia diagonal and collision geometry count as physical-metadata complete.",
            "The four-simulator L5 protocol has not been run.",
        ],
    }
    (output / "asset_records.json").write_text(json.dumps(asset_records, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output / "state_records.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in state_records), encoding="utf-8"
    )
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    report = [
        "# Nano3D Articulation paper-aligned evaluation",
        "",
        f"- assets: {len(asset_records)}; movable joints: {joint_total}; mean joints/asset: {joint_total / len(asset_records):.3f}",
        f"- declared movable joints: {joint_total}; functional-motion joints: {functional_joint_total}; zero-width bounded excluded: {zero_width_joint_total}",
        f"- single-joint penetration-free: {single_penetration_free}/{single_total}; contact-free: {single_contact_free}/{single_total}",
        f"- multi-joint penetration-free: {multi_penetration_free}/{multi_total}; contact-free: {multi_contact_free}/{multi_total}",
        f"- full penetration-free proxy: {full_penetration_free}/{full_total}; contact-free: {full_contact_free}/{full_total}",
        f"- per-joint single-sweep penetration-free: {sum(row['joint_single_sweep_penetration_free_pass_count'] for row in asset_records)}/{functional_joint_total}; contact-free: {sum(row['joint_single_sweep_contact_free_pass_count'] for row in asset_records)}/{functional_joint_total}",
        f"- asset full-range penetration-free proxy: {sum(row['asset_full_range_penetration_free_pass'] for row in asset_records)}/{len(asset_records)}; contact-free: {sum(row['asset_full_range_contact_free_pass'] for row in asset_records)}/{len(asset_records)}",
        "",
        "Semantic type/recall/parent-child/axis-on-moving-part and rest-pose-frozen remain N/A because the local cohort has no frozen joint gold or pre-articulation pair.",
        "",
        f"All collision results disable motors, reset the requested pose, and call performCollisionDetection with {COLLISION_POLICY}; no simulation step is taken. Penetration-free means no depth above {PENETRATION_TOLERANCE_M:g} m; contact-free means zero contact points. These are not CCD or full physical-validity claims.",
    ]
    (output / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"outputs={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
