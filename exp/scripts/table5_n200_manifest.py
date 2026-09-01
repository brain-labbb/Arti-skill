#!/usr/bin/env python3
"""Build the six-dataset Table 5 N=200 intent-to-evaluate manifest.

The cohort is a deterministic prefix of each already-frozen N=800 parent
roster.  Rows are selected before any XML parsing or preflight work; failed
rows are retained and are never replaced.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
EXP_ROOT = REPO_ROOT / "exp"
DEFAULT_SAMPLE_SIZE = 200

DATASET_ORDER = (
    "articraft_10k",
    "lam_released_outputs",
    "artiverse",
    "partnet_mobility",
    "physx_mobility",
    "sketchmobility",
)

DATASET_NAMES = {
    "articraft_10k": "Articraft-10K",
    "lam_released_outputs": "LAM released outputs",
    "artiverse": "Artiverse",
    "partnet_mobility": "PartNet-Mobility",
    "physx_mobility": "PhysX-Mobility",
    "sketchmobility": "SketchMobility",
}

DEFAULT_PARENT_PATHS = {
    "articraft_10k": EXP_ROOT
    / "runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json",
    "lam_released_outputs": EXP_ROOT
    / "runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/manifest.json",
    "artiverse": EXP_ROOT / "runtime/table5_artiverse_table1_n800_gpu_v2/manifest.json",
    "partnet_mobility": EXP_ROOT
    / "runtime/table5_partnet_mobility_table4cohort_n800_v1/manifest.json",
    "physx_mobility": EXP_ROOT / "runtime/table5_physx_mobility_n800_v2/manifest.json",
    "sketchmobility": EXP_ROOT
    / "runtime/table1_sketch_mobility_rerun_20260821T021838Z/manifest.json",
}

DEFAULT_DATASET_ROOTS = {
    "articraft_10k": EXP_ROOT / "Articraft-10K/released_urdf",
    "lam_released_outputs": EXP_ROOT / "Articulated-Object-Code/released_outputs",
    "artiverse": EXP_ROOT / "artiverse",
    "partnet_mobility": EXP_ROOT / "PartNet-Mobility/data/dataset",
    "physx_mobility": EXP_ROOT / "PhysX-Mobility/extracted/PhysX_mobility",
    "sketchmobility": EXP_ROOT / "SketchMobility",
}

DEFAULT_ARTICRAFT_CATEGORY_RECORDS_ROOT = EXP_ROOT / "Articraft-10K-github/records"

DEFAULT_UPSTREAM_PATHS = {
    "articraft_10k": {
        "table2": EXP_ROOT
        / "runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/asset_records.jsonl",
        "table3": EXP_ROOT
        / "runtime/urdf_table3_articraft10k_table2_n800_20260814T040300Z/asset_records.jsonl",
        "table4": EXP_ROOT
        / "runtime/urdf_table4_articraft10k_n800_20260814/asset_records.json",
    },
    "lam_released_outputs": {
        "table2": EXP_ROOT
        / "runtime/table2_urdf_lam_table3cohort_n800_seed20260813_20260814T081000Z/asset_records.jsonl",
        "table3": EXP_ROOT
        / "runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl",
        "table4": EXP_ROOT / "runtime/urdf_table4_lam_n800_20260814/asset_records.json",
    },
    "sketchmobility": {
        "table2": EXP_ROOT
        / "runtime/table2_urdf_sketch_mobility_table1cohort_n800_20260821T035015Z/asset_records.jsonl",
        "table3": EXP_ROOT
        / "runtime/urdf_table3_sketch_mobility_table1cohort_n800_20260821T062050Z/asset_records.jsonl",
        "table4": EXP_ROOT
        / "runtime/urdf_table4_sketch_mobility_table1cohort_n800_20260821T090554Z/asset_records.jsonl",
    },
}

PARENT_CONTAINERS = {
    "articraft_10k": "records",
    "lam_released_outputs": "records",
    "artiverse": "rows",
    "partnet_mobility": "rows",
    "physx_mobility": "rows",
    "sketchmobility": "assets",
}

REUSED_TABLE5_DATASETS = {
    "artiverse",
    "partnet_mobility",
    "physx_mobility",
}

SCALAR_JOINT_TYPES = {"revolute", "continuous", "prismatic"}
SUPPORTED_JOINT_TYPES = SCALAR_JOINT_TYPES | {"fixed"}
NA = "N/E"


class ManifestError(ValueError):
    """Raised when a parent roster cannot be bound without changing its cohort."""


def canonical_json_bytes(value: Any, *, exclude_fields: Sequence[str] = ()) -> bytes:
    excluded = set(exclude_fields)

    def filtered(item: Any) -> Any:
        if isinstance(item, dict):
            return {
                key: filtered(child)
                for key, child in item.items()
                if key not in excluded
            }
        if isinstance(item, list):
            return [filtered(child) for child in item]
        return item

    return json.dumps(
        filtered(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def canonical_sha256(value: Any, *, exclude_fields: Sequence[str] = ()) -> str:
    return hashlib.sha256(
        canonical_json_bytes(value, exclude_fields=exclude_fields)
    ).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _frozen_protocol(sample_size: int) -> dict[str, Any]:
    """Return the dataset-neutral Table 5 runtime contract."""

    runtime_script = SCRIPT_PATH.with_name("table5_n200_runtime.py")
    evaluator_script = SCRIPT_PATH.with_name("run_table5_sketch_mobility.py")
    for implementation_path in (runtime_script, evaluator_script):
        if not implementation_path.is_file():
            raise ManifestError(
                f"Table 5 implementation source is missing: {implementation_path}"
            )
    protocol: dict[str, Any] = {
        "schema_version": "table5_six_dataset_generic_protocol_v1",
        "protocol_id": "table5-six-dataset-prefix-cohort-v1",
        "selection_rule": (
            "for each dataset, take parent roster stored rows [0:sample_size] "
            "before preflight; never filter, resample, replace, or retry-select"
        ),
        "default_sample_size": DEFAULT_SAMPLE_SIZE,
        "sample_size": sample_size,
        "claim_boundary": {
            "release_level_contact_runtime_diagnostic": True,
            "strict_collision": (
                "reuse a bound upstream strict_collision_pass when available; "
                "otherwise report not evaluable and never infer it from this run"
            ),
            "strict_sim_ready": (
                "logical conjunction of available bound Table 2/3/4 strict gates, "
                "all-three runtime pass, and frozen cross-simulator thresholds; "
                "missing required evidence is not a pass"
            ),
            "not_evidence_of": [
                "semantic_joint_correctness",
                "calibrated_real_world_dynamics",
                "continuous_collision_freedom",
                "real_world_fidelity",
            ],
        },
        "runtime": {
            "base": "fixed",
            "contacts": "enabled",
            "gravity_m_per_s2": [0, 0, -9.81],
            "timestep_s": {"numerator": 1, "denominator": 240},
            "solver_iterations": 50,
            "random_seed": 20260813,
            "child_timeout_s": 300,
            "reset_repetitions": 3,
            "initial_scalar_joint_state": "declared_midpoint",
            "thread_caps": {
                "mkl": 1,
                "numexpr": 1,
                "omp": 1,
                "openblas": 1,
                "taichi": 1,
                "veclib": 1,
            },
            "passive_settling": {"seconds": 1.0, "steps": 240},
            "actuation": {
                "eligibility": {
                    "types": ["revolute", "prismatic"],
                    "bounded": True,
                    "require_finite_lower_upper": True,
                    "require_lower_less_than_upper": True,
                    "require_positive_finite_effort_velocity": True,
                },
                "normalization": {
                    "position": "(q - lower) / (upper - lower)",
                    "speed": "qdot / declared_velocity",
                    "position_error": (
                        "target_normalized - measured_normalized_position"
                    ),
                },
                "effort_controller": {
                    "formula": (
                        "tau = effort * clip(kp * normalized_position_error - "
                        "kd * normalized_speed, -1, 1)"
                    ),
                    "kp": 2.0,
                    "kd": 0.2,
                    "clip": [-1.0, 1.0],
                },
                "one_joint_at_a_time": True,
                "other_scalar_joints": "midpoint",
                "trajectory": {
                    "shape": "minimum_jerk",
                    "initial_normalized": 0.0,
                    "target_normalized": 1.0,
                    "ramp_steps": 240,
                    "hold_steps": 120,
                },
                "unsupported_asset_result": "fail_closed",
            },
            "limit_enforcement": {
                "reset": "midpoint",
                "targets_normalized": [-0.1, 1.1],
                "steps_each": 120,
                "tolerance": {
                    "absolute_si": 1e-6,
                    "relative_declared_range": 0.001,
                },
            },
        },
        "metrics": {
            "load": {
                "require_exact_link_joint_counts_and_names": True,
                "fixed_structure_folding": "strict_fail",
            },
            "reset": {
                "all_repetitions": True,
                "require_finite_scalar_state": True,
            },
            "settling": {
                "max_normalized_displacement": 0.05,
                "max_normalized_speed_per_s": 10.0,
                "require_finite_states": True,
            },
            "actuation": {
                "min_declared_range_fraction": 0.9,
                "no_supported_or_any_unsupported_movable": "fail_closed",
            },
            "limit_enforcement": {"all_supported_bounded_joints": True},
            "constraint_drift": {
                "reference": "URDF_FK_at_measured_joint_states",
                "translation_over_bbox_diagonal_max": 0.001,
                "rotation_rad_max": 0.01,
                "missing_bbox": "not_evaluable_fail_closed_for_strict_pass",
            },
            "simulator_pass": {
                "logical_and": [
                    "load",
                    "reset",
                    "settling",
                    "actuation",
                    "limit_enforcement",
                    "constraint_drift",
                ]
            },
            "strict_collision": {
                "denominator": sample_size,
                "fail_closed": True,
                "source": "manifest.strict_gates.table4.strict_collision_pass",
                "missing_evidence": "not_evaluable",
            },
            "strict_sim_ready": {
                "denominator": sample_size,
                "fail_closed": True,
                "logical_and": [
                    "manifest.strict_gates.table2.strict_urdf_pass",
                    "manifest.strict_gates.table3.strict_kinematic_pass",
                    "manifest.strict_gates.table4.strict_collision_pass",
                    "all_three_runtime_pass",
                    "cross_simulator_thresholds",
                ],
                "missing_evidence": "not_evaluable",
            },
        },
        "cross_simulator": {
            "all_three_denominator": sample_size,
            "failure_treatment": (
                "preflight, missing mapping, unsupported, timeout, crash, or "
                "failed simulator remain explicit and cannot improve strict rate"
            ),
            "pairing": {
                "dataset_key": "manifest.dataset_slug",
                "asset_key": "manifest.asset_id",
                "runtime_asset_key": "manifest.dataset_id",
                "joint_key": "urdf_joint_name",
            },
            "sample_cadence_steps": 12,
            "trajectory_samples": 31,
            "joint_rmse": {
                "position": "normalized_joint_position",
                "sample_steps": list(range(0, 361, 12)),
                "separate_denominators": ["revolute", "prismatic"],
            },
            "pose_comparison": {
                "aggregate": "maximum_across_available_simulator_pairs",
                "frame_alignment": "shared_named_root",
                "link_scope": "reachable_descendant_urdf_child_links",
                "translation_error": (
                    "norm(delta_translation) / object_bounding_box_diagonal"
                ),
                "rotation_error": "quaternion_angular_error_rad",
                "missing_bbox": "not_evaluable",
            },
            "thresholds": {
                "normalized_joint_rmse": 0.1,
                "translation_over_bbox_diagonal": 0.02,
                "rotation_rad": 0.1,
            },
            "type_denominators": ["revolute", "prismatic", "link"],
            "strict_runtime_relationship": (
                "missing or failed pairs cannot improve strict rates"
            ),
        },
        "adapters": {
            "pybullet": {
                "version": "3.2.7",
                "connection": "DIRECT",
                "contacts": "enabled",
                "control": "torque",
                "default_motors": "disabled",
                "fixed_link_merge": False,
                "ignore_collision": False,
                "inertials": "source_URDF",
                "maintain_link_order": False,
                "self_collision": True,
                "visual_shapes": "ignored_after_manifest_resource_validation",
            },
            "genesis": {
                "version": "1.3.1",
                "backend": "cuda",
                "precision": "float32",
                "contacts": "enabled",
                "control": "direct_DOF_force",
                "collision": True,
                "enable_neutral_collision": True,
                "fixed": True,
                "fixed_link_merge": False,
                "recompute_inertia": False,
                "requires_jac_and_IK": False,
                "self_collision": True,
                "substeps": 1,
                "visualization": False,
                "logging": "retain_source_mesh_and_inertia_warnings",
                "gpu_binding": {
                    "policy": "runtime_explicit_exactly_one_visible_device",
                    "visible_device_index": 0,
                    "identity_frozen_at_execution": True,
                    "required_receipt_fields": [
                        "physical_device_index",
                        "gpu_uuid",
                        "device_name",
                        "total_memory_mib",
                        "driver_version",
                        "torch_device_name",
                        "torch_version",
                        "torch_cuda_version",
                    ],
                },
            },
            "mujoco": {
                "version": "3.10.0",
                "importer": "native_URDF",
                "contacts": "enabled",
                "control": "direct_qfrc",
                "self_collision": True,
                "fixed_structure": (
                    "folding_recorded_as_strict_load_failure_mapped_movable_"
                    "diagnostics_continue"
                ),
            },
        },
        "implementation": {
            "runtime_script": str(runtime_script),
            "runtime_script_sha256": sha256_file(runtime_script),
            "evaluator_script": str(evaluator_script),
            "evaluator_script_sha256": sha256_file(evaluator_script),
            "reuse": ["evaluate_asset", "PyBulletAdapter", "MuJoCoAdapter"],
        },
        "artifacts": {
            "retry": "none",
            "terminal_records": "atomic_per_asset",
            "timeout_retention": "terminal_failure",
            "preflight_failure_retention": (
                "terminal_failure_without_replacement_or_denominator_removal"
            ),
            "resume_binding": [
                "schema_version",
                "identity.dataset_slug",
                "identity.dataset_name",
                "identity.dataset_id",
                "identity.asset_id",
                "identity.simulator",
                "identity.manifest_sha256",
                "identity.protocol_sha256",
                "identity.row_sha256",
                "identity.urdf_path",
                "identity.urdf_sha256",
                "identity.executable",
                "identity.worker_source_sha256",
                "identity.timeout_s",
                "identity.effective_workers",
                "identity.cuda_visible_devices",
            ],
            "publication": {
                "receipt_set_marker": "receipt_set.json",
                "marker_binds": ["protocol_sha256", "manifest_sha256"],
                "marker_written_last": True,
            },
        },
    }
    protocol["protocol_sha256"] = canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    return protocol


def _load_parent(
    path: Path, container: str
) -> tuple[Path, str, dict[str, Any], list[Any]]:
    canonical = path.resolve(strict=True)
    file_hash = sha256_file(canonical)
    try:
        value = json.loads(canonical.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ManifestError(f"invalid parent JSON: {canonical}: {error}") from error
    if not isinstance(value, dict):
        raise ManifestError(f"parent manifest is not an object: {canonical}")
    rows = value.get(container)
    if not isinstance(rows, list):
        raise ManifestError(f"parent manifest has no {container!r} list: {canonical}")
    return canonical, file_hash, value, rows


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _children(element: ET.Element, name: str) -> list[ET.Element]:
    return [child for child in element if _local_name(child.tag) == name]


def _first_child(element: ET.Element, name: str) -> ET.Element | None:
    return next((child for child in element if _local_name(child.tag) == name), None)


def _finite_float(raw: str | None) -> float | None:
    if raw is None:
        return None
    try:
        value = float(raw)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def _vector(
    element: ET.Element | None,
    attribute: str,
    default: tuple[float, float, float],
) -> list[float]:
    raw = element.get(attribute) if element is not None else None
    if raw is None:
        return list(default)
    values = [float(item) for item in raw.split()]
    if len(values) != 3 or not all(math.isfinite(item) for item in values):
        raise ManifestError(f"invalid finite 3-vector: {raw!r}")
    return values


def _normalized_axis(values: list[float]) -> list[float]:
    magnitude = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(magnitude) or magnitude <= 0:
        raise ManifestError("joint axis is zero or non-finite")
    return [value / magnitude for value in values]


def _validate_joint_tree(
    link_names: list[str], joints: list[dict[str, Any]]
) -> tuple[dict[str, Any] | None, str | None]:
    try:
        if not link_names or any(not name for name in link_names):
            raise ManifestError("link names must be non-empty")
        if len(set(link_names)) != len(link_names):
            raise ManifestError("link names must be unique")
        links = set(link_names)
        children: set[str] = set()
        joint_names: set[str] = set()
        for joint in joints:
            if not joint["name"] or joint["name"] in joint_names:
                raise ManifestError("joint names must be unique and non-empty")
            if joint["type"] not in SUPPORTED_JOINT_TYPES:
                raise ManifestError(f"unsupported joint type: {joint['type']!r}")
            if (
                joint["parent"] not in links
                or joint["child"] not in links
                or joint["parent"] == joint["child"]
                or joint["child"] in children
            ):
                raise ManifestError("invalid joint parent/child structure")
            joint_names.add(joint["name"])
            children.add(joint["child"])
        roots = sorted(links - children)
        if len(roots) != 1:
            raise ManifestError("joint tree must have exactly one root")
        reached = {roots[0]}
        while True:
            expanded = reached | {
                joint["child"] for joint in joints if joint["parent"] in reached
            }
            if expanded == reached:
                break
            reached = expanded
        if reached != links:
            raise ManifestError("joint tree is disconnected or cyclic")
        return {"links": link_names, "root_links": roots, "joints": joints}, None
    except ManifestError as error:
        return None, str(error)


def _resource_issues(
    package_root: Path, urdf_path: Path, robot: ET.Element
) -> list[str]:
    issues: set[str] = set()
    package = package_root.resolve(strict=False)
    for element in robot.iter():
        if _local_name(element.tag) != "mesh":
            continue
        reference = (element.get("filename") or "").strip()
        raw = Path(reference)
        if not reference or raw.is_absolute() or "://" in reference:
            issues.add(f"unsafe_mesh_reference:{reference}")
            continue
        resolved = (urdf_path.parent / raw).resolve(strict=False)
        try:
            resolved.relative_to(package)
        except ValueError:
            issues.add(f"escaping_mesh_reference:{reference}")
            continue
        if resolved.is_symlink() or not resolved.is_file():
            issues.add(f"missing_mesh_resource:{reference}")
    return sorted(issues)


def _parse_urdf(package_root: Path, urdf_path: Path) -> dict[str, Any]:
    """Parse static metadata without importing or stepping a simulator."""

    result: dict[str, Any] = {
        "joint_tree": None,
        "scalar_joints": [],
        "xml_counts": None,
        "issues": [],
    }
    try:
        if urdf_path.is_symlink() or not urdf_path.is_file():
            raise ManifestError("URDF is missing or symlinked")
        robot = ET.parse(urdf_path).getroot()
        if _local_name(robot.tag) != "robot":
            raise ManifestError("URDF root element is not robot")
        link_nodes = _children(robot, "link")
        joint_nodes = _children(robot, "joint")
        link_names = [(node.get("name") or "").strip() for node in link_nodes]
        joints: list[dict[str, Any]] = []
        for node in joint_nodes:
            parent = _first_child(node, "parent")
            child = _first_child(node, "child")
            origin = _first_child(node, "origin")
            limit = _first_child(node, "limit")
            joint_type = (node.get("type") or "").strip().lower()
            axis = _normalized_axis(
                _vector(_first_child(node, "axis"), "xyz", (1.0, 0.0, 0.0))
            )
            joints.append(
                {
                    "name": (node.get("name") or "").strip(),
                    "type": joint_type,
                    "parent": (
                        (parent.get("link") or "").strip() if parent is not None else ""
                    ),
                    "child": (
                        (child.get("link") or "").strip() if child is not None else ""
                    ),
                    "origin_xyz": _vector(origin, "xyz", (0.0, 0.0, 0.0)),
                    "origin_rpy": _vector(origin, "rpy", (0.0, 0.0, 0.0)),
                    "axis": axis,
                    "fk_supported": joint_type in SUPPORTED_JOINT_TYPES,
                    "lower": _finite_float(
                        limit.get("lower") if limit is not None else None
                    ),
                    "upper": _finite_float(
                        limit.get("upper") if limit is not None else None
                    ),
                    "effort": _finite_float(
                        limit.get("effort") if limit is not None else None
                    ),
                    "velocity": _finite_float(
                        limit.get("velocity") if limit is not None else None
                    ),
                }
            )
        tree, tree_issue = _validate_joint_tree(link_names, joints)
        issues = _resource_issues(package_root, urdf_path, robot)
        if tree_issue is not None:
            issues.append(f"invalid_joint_tree:{tree_issue}")
        scalar_joints = [
            joint for joint in joints if joint["type"] in SCALAR_JOINT_TYPES
        ]
        result.update(
            {
                "joint_tree": tree,
                "scalar_joints": scalar_joints,
                "xml_counts": {
                    "links": len(link_nodes),
                    "joints": len(joint_nodes),
                    "fixed_joints": sum(joint["type"] == "fixed" for joint in joints),
                    "movable_joints": len(scalar_joints),
                    "visual_elements": sum(
                        _local_name(item.tag) == "visual" for item in robot.iter()
                    ),
                    "collision_elements": sum(
                        _local_name(item.tag) == "collision" for item in robot.iter()
                    ),
                    "inertial_elements": sum(
                        _local_name(item.tag) == "inertial" for item in robot.iter()
                    ),
                },
                "issues": sorted(set(issues)),
            }
        )
    except (ET.ParseError, OSError, ValueError, ManifestError) as error:
        result["issues"] = [f"urdf_parse_failure:{type(error).__name__}:{error}"]
    return result


def _unavailable_strict_gates(reason: str) -> dict[str, Any]:
    return {
        "table2": {
            "strict_urdf_pass": None,
            "availability": "not_available",
            "reason": reason,
        },
        "table3": {
            "strict_kinematic_pass": None,
            "availability": "not_available",
            "reason": reason,
        },
        "table4": {
            "strict_collision_pass": None,
            "availability": "not_available",
            "reason": reason,
        },
    }


def _normalized_strict_gates(source: Any) -> dict[str, Any]:
    definitions = {
        "table2": "strict_urdf_pass",
        "table3": "strict_kinematic_pass",
        "table4": "strict_collision_pass",
    }
    raw = source if isinstance(source, Mapping) else {}
    normalized: dict[str, Any] = {}
    for table, field in definitions.items():
        original = raw.get(table)
        gate = deepcopy(original) if isinstance(original, Mapping) else {}
        value = gate.get(field)
        if isinstance(value, bool):
            gate.update({"availability": "available", "reason": None})
        else:
            gate[field] = None
            gate.update(
                {
                    "availability": "not_available",
                    "reason": "not bound by the reused Table 5 source row",
                }
            )
        normalized[table] = gate
    return normalized


def _preflight(
    issues: Sequence[str], *, warnings: Sequence[str] = ()
) -> dict[str, Any]:
    unique_issues = sorted(set(issues))
    unique_warnings = sorted(set(warnings))
    return {
        "status": "failed" if unique_issues else "pass",
        "issues": unique_issues,
        "warnings": unique_warnings,
        "simulator_eligible": not unique_issues,
    }


def _parent_binding(path: Path, file_hash: str, order: int, raw: Any) -> dict[str, Any]:
    return {
        "path": str(path),
        "sha256": file_hash,
        "order": order,
        "row_sha256": canonical_sha256(raw),
    }


def _read_record_file(path: Path) -> list[dict[str, Any]]:
    if path.suffix == ".jsonl":
        rows: list[dict[str, Any]] = []
        with path.open("r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, 1):
                if not line.strip():
                    continue
                try:
                    value = json.loads(line)
                except json.JSONDecodeError as error:
                    raise ManifestError(
                        f"invalid upstream JSONL at {path}:{line_number}"
                    ) from error
                if not isinstance(value, dict):
                    raise ManifestError(
                        f"non-object upstream JSONL row at {path}:{line_number}"
                    )
                rows.append(value)
        return rows
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ManifestError(f"invalid upstream JSON: {path}") from error
    if not isinstance(value, list) or not all(isinstance(row, dict) for row in value):
        raise ManifestError(f"upstream JSON must be an array of objects: {path}")
    return value


def _upstream_key(slug: str, record: Mapping[str, Any]) -> str | None:
    fields = (
        ("asset_key", "asset_id", "dataset_id")
        if slug == "lam_released_outputs"
        else ("asset_id", "dataset_id", "asset_key")
    )
    for field in fields:
        value = record.get(field)
        if isinstance(value, (str, int)) and not isinstance(value, bool):
            return str(value)
    return None


def _index_upstream_rows(
    slug: str, rows: Sequence[Mapping[str, Any]]
) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for record in rows:
        key = _upstream_key(slug, record)
        if key is None:
            raise ManifestError(f"{slug} upstream record has no dataset key")
        if key in result:
            raise ManifestError(f"{slug} upstream has duplicate dataset key: {key}")
        result[key] = dict(record)
    return result


def _load_upstream_bundle(slug: str, table: str, path: Path) -> dict[str, Any]:
    bundle: dict[str, Any] = {
        "path": str(path.resolve(strict=False)),
        "sha256": None,
        "record_count": 0,
        "records": {},
        "auxiliary_manifest": None,
        "error": None,
    }
    try:
        canonical = path.resolve(strict=True)
        rows = _read_record_file(canonical)
        bundle.update(
            {
                "path": str(canonical),
                "sha256": sha256_file(canonical),
                "record_count": len(rows),
                "records": _index_upstream_rows(slug, rows),
            }
        )
        if table == "table4":
            auxiliary_path = canonical.parent / "manifest.json"
            if auxiliary_path.is_file():
                auxiliary = json.loads(auxiliary_path.read_text(encoding="utf-8"))
                items = auxiliary.get("items") if isinstance(auxiliary, dict) else None
                if isinstance(items, list) and all(
                    isinstance(item, dict) for item in items
                ):
                    bundle["auxiliary_manifest"] = {
                        "path": str(auxiliary_path.resolve(strict=True)),
                        "sha256": sha256_file(auxiliary_path.resolve(strict=True)),
                        "items": _index_upstream_rows(slug, items),
                    }
    except (OSError, json.JSONDecodeError, ManifestError) as error:
        bundle["records"] = {}
        bundle["error"] = f"{type(error).__name__}:{error}"
    return bundle


def _resolved_upstream_configuration(
    overrides: Mapping[str, Mapping[str, Path]] | None,
) -> dict[str, dict[str, Path]]:
    result = {
        slug: {table: Path(path) for table, path in paths.items()}
        for slug, paths in DEFAULT_UPSTREAM_PATHS.items()
    }
    if not overrides:
        return result
    unknown_slugs = set(overrides) - set(DEFAULT_UPSTREAM_PATHS)
    if unknown_slugs:
        raise ManifestError(
            f"unknown upstream dataset slugs: {sorted(unknown_slugs)!r}"
        )
    for slug, paths in overrides.items():
        unknown_tables = set(paths) - {"table2", "table3", "table4"}
        if unknown_tables:
            raise ManifestError(
                f"unknown upstream tables for {slug}: {sorted(unknown_tables)!r}"
            )
        result[slug].update({table: Path(path) for table, path in paths.items()})
    return result


def _load_generated_upstreams(
    paths: Mapping[str, Mapping[str, Path]],
) -> dict[str, dict[str, dict[str, Any]]]:
    return {
        slug: {
            table: _load_upstream_bundle(slug, table, Path(paths[slug][table]))
            for table in ("table2", "table3", "table4")
        }
        for slug in ("articraft_10k", "lam_released_outputs", "sketchmobility")
    }


def _record_urdf_hash(record: Mapping[str, Any]) -> str | None:
    for field in (
        "urdf_sha256",
        "primary_urdf_sha256",
        "model_urdf_sha256",
    ):
        value = record.get(field)
        if isinstance(value, str) and len(value) == 64:
            return value
    return None


def _bound_upstream_record(
    bundle: Mapping[str, Any], key: str, expected_urdf_hash: str
) -> tuple[dict[str, Any] | None, str | None, str | None]:
    if bundle.get("error") is not None:
        return None, None, f"upstream artifact unavailable: {bundle['error']}"
    records = bundle.get("records")
    record = records.get(key) if isinstance(records, Mapping) else None
    if not isinstance(record, dict):
        return None, None, "dataset key absent from upstream artifact"
    bound_hash = _record_urdf_hash(record)
    if bound_hash is None:
        auxiliary = bundle.get("auxiliary_manifest")
        items = auxiliary.get("items") if isinstance(auxiliary, Mapping) else None
        item = items.get(key) if isinstance(items, Mapping) else None
        if not isinstance(item, Mapping):
            return None, None, "upstream record has no URDF hash binding"
        if record.get("input_identity_sha256") is not None and record.get(
            "input_identity_sha256"
        ) != item.get("input_identity_sha256"):
            return None, None, "upstream result/manifest input identity mismatch"
        if record.get("package_content_manifest_sha256") is not None and record.get(
            "package_content_manifest_sha256"
        ) != item.get("package_content_manifest_sha256"):
            return None, None, "upstream result/manifest package binding mismatch"
        bound_hash = _record_urdf_hash(item)
    if bound_hash != expected_urdf_hash:
        return None, bound_hash, "upstream URDF hash mismatch"
    return record, bound_hash, None


def _gate_evidence(
    table: str,
    field: str,
    bundle: Mapping[str, Any],
    key: str,
    expected_urdf_hash: str,
) -> dict[str, Any]:
    record, bound_hash, error = _bound_upstream_record(bundle, key, expected_urdf_hash)
    if record is None:
        return {
            field: None,
            "availability": "not_available",
            "reason": error,
        }
    value = record.get(field)
    if not isinstance(value, bool):
        return {
            field: None,
            "availability": "not_available",
            "reason": f"bound upstream record has no boolean {field}",
        }
    evidence: dict[str, Any] = {
        field: value,
        "availability": "available",
        "reason": None,
        "record_sha256": canonical_sha256(record),
        "source_path": bundle["path"],
        "source_sha256": bundle["sha256"],
        "dataset_key": key,
        "urdf_sha256": bound_hash,
    }
    if table == "table2" and isinstance(record.get("metrics"), Mapping):
        evidence["subgates"] = {
            name: metric.get("pass")
            for name, metric in record["metrics"].items()
            if isinstance(metric, Mapping) and isinstance(metric.get("pass"), bool)
        }
    if table == "table3" and isinstance(record.get("tree_valid"), bool):
        evidence["tree_valid"] = record["tree_valid"]
    if table == "table4":
        for name in ("load_success", "measurement_complete"):
            if isinstance(record.get(name), bool):
                evidence[name] = record[name]
    return evidence


def _bound_generated_evidence(
    upstream: Mapping[str, Mapping[str, Any]],
    key: str,
    expected_urdf_hash: Any,
) -> tuple[dict[str, Any], float | None, dict[str, Any]]:
    if not isinstance(expected_urdf_hash, str) or len(expected_urdf_hash) != 64:
        return (
            _unavailable_strict_gates("parent roster has no valid URDF hash"),
            None,
            {
                "status": "not_evaluable",
                "diagonal_m": None,
                "protocol": None,
                "reason": "parent roster has no valid URDF hash",
            },
        )
    gates = {
        "table2": _gate_evidence(
            "table2",
            "strict_urdf_pass",
            upstream["table2"],
            key,
            expected_urdf_hash,
        ),
        "table3": _gate_evidence(
            "table3",
            "strict_kinematic_pass",
            upstream["table3"],
            key,
            expected_urdf_hash,
        ),
        "table4": _gate_evidence(
            "table4",
            "strict_collision_pass",
            upstream["table4"],
            key,
            expected_urdf_hash,
        ),
    }
    table4_record, _, table4_error = _bound_upstream_record(
        upstream["table4"], key, expected_urdf_hash
    )
    diagonal = (
        table4_record.get("object_bbox_diagonal_m")
        if isinstance(table4_record, Mapping)
        else None
    )
    if (
        isinstance(diagonal, (int, float))
        and not isinstance(diagonal, bool)
        and math.isfinite(float(diagonal))
        and float(diagonal) > 0
    ):
        value = float(diagonal)
        return (
            gates,
            value,
            {
                "status": "available",
                "diagonal_m": value,
                "protocol": "pybullet_q0_collision_shape_union_aabb_v1",
                "reason": None,
                "source_path": upstream["table4"]["path"],
                "source_sha256": upstream["table4"]["sha256"],
                "record_sha256": canonical_sha256(table4_record),
            },
        )
    return (
        gates,
        None,
        {
            "status": "not_evaluable",
            "diagonal_m": None,
            "protocol": None,
            "reason": table4_error
            or "bound Table 4 record has no positive finite diagonal",
        },
    )


def _upstream_receipts(
    upstream: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    receipts: dict[str, Any] = {}
    for table, bundle in upstream.items():
        receipt = {
            "path": bundle["path"],
            "sha256": bundle["sha256"],
            "record_count": bundle["record_count"],
            "availability": (
                "available" if bundle.get("error") is None else "not_available"
            ),
            "reason": bundle.get("error"),
        }
        auxiliary = bundle.get("auxiliary_manifest")
        if isinstance(auxiliary, Mapping):
            receipt["auxiliary_manifest_path"] = auxiliary["path"]
            receipt["auxiliary_manifest_sha256"] = auxiliary["sha256"]
        receipts[table] = receipt
    return receipts


def _actual_hash(path: Path, expected: Any, issues: list[str]) -> str | None:
    if path.is_symlink() or not path.is_file():
        issues.append("urdf_missing_or_symlinked")
        return None
    observed = sha256_file(path)
    if isinstance(expected, str) and expected and observed != expected:
        issues.append("urdf_sha256_mismatch")
    return observed


def _read_articraft_category(
    records_root: Path, asset_id: str
) -> tuple[str, str | None]:
    path = records_root / asset_id / "record.json"
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return NA, f"category_record_unavailable:{type(error).__name__}"
    category = record.get("category_slug")
    if (
        record.get("record_id") != asset_id
        or not isinstance(category, str)
        or not category
    ):
        return NA, "category_record_identity_or_category_invalid"
    return category, None


def _generated_row(
    slug: str,
    raw: Mapping[str, Any],
    *,
    order: int,
    parent_path: Path,
    parent_hash: str,
    dataset_root: Path,
    category_records_root: Path,
    upstream: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    warnings: list[str] = []
    if slug == "articraft_10k":
        asset_id = str(raw.get("asset_id") or f"parent_order_{order:04d}")
        dataset_id = f"articraft_{order:04d}"
        upstream_key = asset_id
        package = Path(str(raw.get("package") or dataset_root / asset_id))
        urdf = package / "model.urdf"
        expected_hash = raw.get("model_urdf_sha256")
        category, warning = _read_articraft_category(category_records_root, asset_id)
        if warning:
            warnings.append(warning)
    elif slug == "lam_released_outputs":
        asset_id = str(raw.get("asset_key") or f"parent_order_{order:04d}")
        dataset_id = f"lam_{order:04d}"
        upstream_key = asset_id
        urdf = Path(
            str(
                raw.get("urdf_path")
                or dataset_root / str(raw.get("rel_path", "")) / "generated.urdf"
            )
        )
        package = urdf.parent
        expected_hash = raw.get("urdf_sha256")
        category = str(raw.get("category") or NA)
    elif slug == "sketchmobility":
        asset_id = str(raw.get("asset_id") or f"parent_order_{order:04d}")
        dataset_id = f"sketch_{order:04d}"
        upstream_key = asset_id
        package = dataset_root / asset_id
        urdf = package / "mobility.urdf"
        expected_hash = raw.get("mobility_urdf_sha256")
        category = str(raw.get("source_category") or raw.get("category") or NA)
    else:
        raise ManifestError(f"unsupported generated-row dataset: {slug}")

    package = package.resolve(strict=False)
    urdf = urdf.resolve(strict=False)
    issues: list[str] = []
    observed_hash = _actual_hash(urdf, expected_hash, issues)
    parsed = _parse_urdf(package, urdf)
    issues.extend(parsed["issues"])
    if not isinstance(expected_hash, str) or len(expected_hash) != 64:
        warnings.append("parent_urdf_sha256_unavailable")
    strict_gates, diagonal, bounding_box = _bound_generated_evidence(
        upstream, upstream_key, expected_hash
    )
    if diagonal is None:
        warnings.append("bounding_box_diagonal_not_bound_by_upstream_evidence")
    scalar_joints = parsed["scalar_joints"] if parsed["joint_tree"] is not None else []
    row: dict[str, Any] = {
        "dataset_slug": slug,
        "dataset_name": DATASET_NAMES[slug],
        "asset_id": asset_id,
        "dataset_id": dataset_id,
        "category": category,
        "package_root": str(package),
        "urdf_path": str(urdf),
        "urdf_sha256": observed_hash if observed_hash is not None else expected_hash,
        "source_parent": _parent_binding(parent_path, parent_hash, order, raw),
        "joint_tree": parsed["joint_tree"],
        "scalar_joints": scalar_joints,
        "xml_counts": parsed["xml_counts"],
        "bounding_box": bounding_box,
        "bounding_box_diagonal": diagonal,
        "strict_gates": strict_gates,
        "preflight": _preflight(issues, warnings=warnings),
    }
    if parsed["joint_tree"] is None and parsed["scalar_joints"]:
        row["unvalidated_scalar_joints"] = parsed["scalar_joints"]
    row["row_sha256"] = canonical_sha256(row, exclude_fields=("row_sha256",))
    return row


def _reused_row(
    slug: str,
    raw: Mapping[str, Any],
    *,
    order: int,
    parent_path: Path,
    parent_hash: str,
    dataset_root: Path,
) -> dict[str, Any]:
    row = deepcopy(dict(raw))
    source_dataset_id = row.get("dataset_id")
    asset_id = str(
        row.get("asset_id") or source_dataset_id or f"parent_order_{order:04d}"
    )
    if slug == "physx_mobility":
        package = dataset_root
    else:
        package = dataset_root / str(
            row.get("package_relative_path") or source_dataset_id
        )
    urdf = dataset_root / str(row.get("urdf_relative_path") or "")
    package = package.resolve(strict=False)
    urdf = urdf.resolve(strict=False)
    issues: list[str] = []
    observed_hash = _actual_hash(urdf, row.get("urdf_sha256"), issues)
    if row.get("joint_tree") is None:
        issues.append("joint_tree_unavailable")
    scalar = row.get("scalar_joints")
    if not isinstance(scalar, list):
        issues.append("scalar_joints_unavailable")
        scalar = []
    unvalidated_scalar: list[Any] | None = None
    if row.get("joint_tree") is None and scalar:
        unvalidated_scalar = scalar
        scalar = []
    existing_preflight = row.get("preflight")
    if isinstance(existing_preflight, Mapping):
        existing_issues = existing_preflight.get("issues")
        if isinstance(existing_issues, list):
            issues.extend(str(issue) for issue in existing_issues)
    original_row_hash = row.get("row_sha256")
    row.update(
        {
            "dataset_slug": slug,
            "dataset_name": DATASET_NAMES[slug],
            "asset_id": asset_id,
            "dataset_id": str(
                source_dataset_id if source_dataset_id is not None else asset_id
            ),
            "category": str(row.get("category") or row.get("raw_category") or NA),
            "package_root": str(package),
            "urdf_path": str(urdf),
            "urdf_sha256": (
                observed_hash if observed_hash is not None else row.get("urdf_sha256")
            ),
            "source_parent": _parent_binding(parent_path, parent_hash, order, raw),
            "scalar_joints": scalar,
            "strict_gates": _normalized_strict_gates(row.get("strict_gates")),
            "preflight": _preflight(issues),
        }
    )
    diagonal = row.get("bounding_box_diagonal")
    if not (
        isinstance(diagonal, (int, float))
        and not isinstance(diagonal, bool)
        and math.isfinite(float(diagonal))
        and float(diagonal) > 0
    ):
        row["bounding_box_diagonal"] = None
        row["bounding_box"] = {
            "status": "not_evaluable",
            "diagonal_m": None,
            "protocol": None,
            "reason": "reused Table 5 row contains no finite positive diagonal",
        }
    if isinstance(original_row_hash, str):
        row["source_table5_row_sha256"] = original_row_hash
    if unvalidated_scalar is not None:
        row["unvalidated_scalar_joints"] = unvalidated_scalar
    row["row_sha256"] = canonical_sha256(row, exclude_fields=("row_sha256",))
    return row


def _resolved_configuration(
    defaults: Mapping[str, Path], overrides: Mapping[str, Path] | None
) -> dict[str, Path]:
    result = {key: Path(value) for key, value in defaults.items()}
    if overrides:
        unknown = set(overrides) - set(defaults)
        if unknown:
            raise ManifestError(f"unknown dataset slugs: {sorted(unknown)!r}")
        result.update({key: Path(value) for key, value in overrides.items()})
    return result


def build_manifest(
    sample_size: int = DEFAULT_SAMPLE_SIZE,
    *,
    parent_paths: Mapping[str, Path] | None = None,
    dataset_roots: Mapping[str, Path] | None = None,
    articraft_category_records_root: Path | None = None,
    upstream_paths: Mapping[str, Mapping[str, Path]] | None = None,
) -> dict[str, Any]:
    """Build and return the deterministic six-dataset cohort manifest.

    Optional path mappings exist for focused fixture tests.  They do not alter
    the selection rule: each dataset always uses the exact stored prefix.
    """

    if (
        isinstance(sample_size, bool)
        or not isinstance(sample_size, int)
        or sample_size <= 0
    ):
        raise ManifestError("sample_size must be a positive integer")
    parents = _resolved_configuration(DEFAULT_PARENT_PATHS, parent_paths)
    roots = _resolved_configuration(DEFAULT_DATASET_ROOTS, dataset_roots)
    upstream_configuration = _resolved_upstream_configuration(upstream_paths)
    generated_upstreams = _load_generated_upstreams(upstream_configuration)
    category_root = Path(
        articraft_category_records_root or DEFAULT_ARTICRAFT_CATEGORY_RECORDS_ROOT
    ).resolve(strict=False)

    datasets: list[dict[str, Any]] = []
    for slug in DATASET_ORDER:
        container = PARENT_CONTAINERS[slug]
        parent_path, parent_hash, parent, raw_rows = _load_parent(
            parents[slug], container
        )
        if len(raw_rows) < sample_size:
            raise ManifestError(
                f"{slug} parent has {len(raw_rows)} rows, fewer than sample_size={sample_size}"
            )

        # Membership is frozen here, before parsing, hashing, or preflight.
        selected = raw_rows[:sample_size]
        rows: list[dict[str, Any]] = []
        for order, raw in enumerate(selected):
            if not isinstance(raw, Mapping):
                raise ManifestError(f"{slug} parent row {order} is not an object")
            if slug in REUSED_TABLE5_DATASETS:
                row = _reused_row(
                    slug,
                    raw,
                    order=order,
                    parent_path=parent_path,
                    parent_hash=parent_hash,
                    dataset_root=roots[slug],
                )
            else:
                row = _generated_row(
                    slug,
                    raw,
                    order=order,
                    parent_path=parent_path,
                    parent_hash=parent_hash,
                    dataset_root=roots[slug],
                    category_records_root=category_root,
                    upstream=generated_upstreams[slug],
                )
            rows.append(row)

        ordered_asset_ids = [row["asset_id"] for row in rows]
        ordered_bindings = [
            {
                "order": row["source_parent"]["order"],
                "asset_id": row["asset_id"],
                "dataset_id": row["dataset_id"],
                "urdf_sha256": row["urdf_sha256"],
                "parent_row_sha256": row["source_parent"]["row_sha256"],
            }
            for row in rows
        ]
        dataset_entry: dict[str, Any] = {
            "dataset_slug": slug,
            "dataset_name": DATASET_NAMES[slug],
            "parent": {
                "path": str(parent_path),
                "sha256": parent_hash,
                "row_container": container,
                "parent_count": len(raw_rows),
                "declared_n_eval": parent.get("N_eval")
                or parent.get("selection", {}).get("n_eval")
                or parent.get("selection", {}).get("selected_count"),
            },
            "selection": {
                "rule": "exact stored prefix [0:sample_size]",
                "selected_count": len(rows),
                "ordered_subset_sha256": canonical_sha256(ordered_asset_ids),
                "ordered_row_bindings_sha256": canonical_sha256(ordered_bindings),
                "replacement": False,
                "outcome_filtering": False,
                "preflight_before_selection": False,
                "retained_preflight_failures": sum(
                    row["preflight"]["status"] == "failed" for row in rows
                ),
            },
            "rows": rows,
        }
        if slug in generated_upstreams:
            dataset_entry["upstream_evidence"] = _upstream_receipts(
                generated_upstreams[slug]
            )
        datasets.append(dataset_entry)

    protocol = _frozen_protocol(sample_size)
    manifest: dict[str, Any] = {
        "schema_version": "table5_six_dataset_prefix_manifest_v1",
        "protocol": protocol,
        "protocol_sha256": protocol["protocol_sha256"],
        "sample_size": sample_size,
        "dataset_count": len(datasets),
        "total_rows": sample_size * len(datasets),
        "ordered_dataset_slugs": list(DATASET_ORDER),
        "datasets": datasets,
    }
    manifest["cohort_sha256"] = canonical_sha256(
        [
            {
                "dataset_slug": dataset["dataset_slug"],
                "parent_sha256": dataset["parent"]["sha256"],
                "ordered_subset_sha256": dataset["selection"]["ordered_subset_sha256"],
                "ordered_row_bindings_sha256": dataset["selection"][
                    "ordered_row_bindings_sha256"
                ],
            }
            for dataset in datasets
        ]
    )
    manifest["manifest_sha256"] = canonical_sha256(
        manifest, exclude_fields=("manifest_sha256",)
    )
    return manifest


def validate_manifest(
    manifest: Mapping[str, Any], *, verify_parent_files: bool = True
) -> None:
    """Validate self-hashes, parent bindings, and exact-prefix invariants."""

    if not isinstance(manifest, Mapping):
        raise ManifestError("manifest must be an object")
    if manifest.get("schema_version") != "table5_six_dataset_prefix_manifest_v1":
        raise ManifestError("manifest schema_version mismatch")
    sample_size = manifest.get("sample_size")
    if (
        isinstance(sample_size, bool)
        or not isinstance(sample_size, int)
        or sample_size <= 0
    ):
        raise ManifestError("manifest sample_size must be a positive integer")
    protocol = manifest.get("protocol")
    if not isinstance(protocol, Mapping):
        raise ManifestError("manifest protocol is missing")
    expected_protocol_hash = canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    if (
        protocol.get("protocol_sha256") != expected_protocol_hash
        or manifest.get("protocol_sha256") != expected_protocol_hash
    ):
        raise ManifestError("protocol self-hash mismatch")
    if manifest.get("manifest_sha256") != canonical_sha256(
        manifest, exclude_fields=("manifest_sha256",)
    ):
        raise ManifestError("manifest self-hash mismatch")

    datasets = manifest.get("datasets")
    if not isinstance(datasets, list) or len(datasets) != len(DATASET_ORDER):
        raise ManifestError("manifest must contain all six datasets")
    observed_order = [
        dataset.get("dataset_slug") if isinstance(dataset, Mapping) else None
        for dataset in datasets
    ]
    if observed_order != list(DATASET_ORDER):
        raise ManifestError("dataset order mismatch")
    if manifest.get("ordered_dataset_slugs") != list(DATASET_ORDER):
        raise ManifestError("ordered_dataset_slugs mismatch")
    if manifest.get("dataset_count") != len(DATASET_ORDER):
        raise ManifestError("dataset_count mismatch")
    if manifest.get("total_rows") != sample_size * len(DATASET_ORDER):
        raise ManifestError("total_rows mismatch")

    cohort_bindings: list[dict[str, Any]] = []
    for dataset in datasets:
        if not isinstance(dataset, Mapping):
            raise ManifestError("dataset entry must be an object")
        slug = str(dataset["dataset_slug"])
        parent = dataset.get("parent")
        selection = dataset.get("selection")
        rows = dataset.get("rows")
        if not isinstance(parent, Mapping) or not isinstance(selection, Mapping):
            raise ManifestError(f"{slug} parent/selection receipt is missing")
        if not isinstance(rows, list) or len(rows) != sample_size:
            raise ManifestError(f"{slug} row count does not equal sample_size")
        if selection.get("selected_count") != sample_size:
            raise ManifestError(f"{slug} selected_count mismatch")
        if selection.get("replacement") is not False:
            raise ManifestError(f"{slug} replacement must be false")
        if selection.get("outcome_filtering") is not False:
            raise ManifestError(f"{slug} outcome_filtering must be false")
        if selection.get("preflight_before_selection") is not False:
            raise ManifestError(f"{slug} preflight_before_selection must be false")
        parent_path = Path(str(parent.get("path", "")))
        parent_hash = parent.get("sha256")
        if not isinstance(parent_hash, str) or len(parent_hash) != 64:
            raise ManifestError(f"{slug} parent SHA256 is malformed")
        if verify_parent_files:
            try:
                observed_parent_hash = sha256_file(parent_path.resolve(strict=True))
            except OSError as error:
                raise ManifestError(f"{slug} parent file is unavailable") from error
            if observed_parent_hash != parent_hash:
                raise ManifestError(f"{slug} parent file SHA256 mismatch")

        asset_ids: list[str] = []
        ordered_bindings: list[dict[str, Any]] = []
        runtime_ids: set[str] = set()
        for order, row in enumerate(rows):
            if not isinstance(row, Mapping):
                raise ManifestError(f"{slug} row {order} is not an object")
            if row.get("dataset_slug") != slug:
                raise ManifestError(f"{slug} row {order} dataset_slug mismatch")
            asset_id = row.get("asset_id")
            runtime_id = row.get("dataset_id")
            if not isinstance(asset_id, str) or not asset_id:
                raise ManifestError(f"{slug} row {order} asset_id is invalid")
            if not isinstance(runtime_id, str) or not runtime_id:
                raise ManifestError(f"{slug} row {order} dataset_id is invalid")
            if runtime_id in runtime_ids:
                raise ManifestError(f"{slug} has duplicate dataset_id: {runtime_id}")
            runtime_ids.add(runtime_id)
            source = row.get("source_parent")
            if not isinstance(source, Mapping):
                raise ManifestError(f"{slug} row {order} source_parent is missing")
            if (
                source.get("path") != parent.get("path")
                or source.get("sha256") != parent_hash
                or source.get("order") != order
            ):
                raise ManifestError(f"{slug} row {order} parent binding mismatch")
            if row.get("row_sha256") != canonical_sha256(
                row, exclude_fields=("row_sha256",)
            ):
                raise ManifestError(f"{slug} row {order} self-hash mismatch")
            gates = row.get("strict_gates")
            if not isinstance(gates, Mapping):
                raise ManifestError(f"{slug} row {order} strict_gates is missing")
            for table, field in (
                ("table2", "strict_urdf_pass"),
                ("table3", "strict_kinematic_pass"),
                ("table4", "strict_collision_pass"),
            ):
                gate = gates.get(table)
                if not isinstance(gate, Mapping):
                    raise ManifestError(f"{slug} row {order} {table} gate is missing")
                value = gate.get(field)
                if value is not None and not isinstance(value, bool):
                    raise ManifestError(f"{slug} row {order} {field} is malformed")
                if isinstance(value, bool) and gate.get("availability") != "available":
                    raise ManifestError(
                        f"{slug} row {order} {field} availability mismatch"
                    )
                if value is None and gate.get("availability") != "not_available":
                    raise ManifestError(
                        f"{slug} row {order} {field} availability mismatch"
                    )
            diagonal = row.get("bounding_box_diagonal")
            if diagonal is not None and not (
                isinstance(diagonal, (int, float))
                and not isinstance(diagonal, bool)
                and math.isfinite(float(diagonal))
                and float(diagonal) > 0
            ):
                raise ManifestError(f"{slug} row {order} bbox diagonal is malformed")
            asset_ids.append(asset_id)
            ordered_bindings.append(
                {
                    "order": order,
                    "asset_id": asset_id,
                    "dataset_id": runtime_id,
                    "urdf_sha256": row.get("urdf_sha256"),
                    "parent_row_sha256": source.get("row_sha256"),
                }
            )
        if selection.get("ordered_subset_sha256") != canonical_sha256(asset_ids):
            raise ManifestError(f"{slug} ordered subset hash mismatch")
        if selection.get("ordered_row_bindings_sha256") != canonical_sha256(
            ordered_bindings
        ):
            raise ManifestError(f"{slug} ordered row binding hash mismatch")
        expected_failures = sum(
            isinstance(row.get("preflight"), Mapping)
            and row["preflight"].get("status") == "failed"
            for row in rows
        )
        if selection.get("retained_preflight_failures") != expected_failures:
            raise ManifestError(f"{slug} retained preflight failure count mismatch")
        cohort_bindings.append(
            {
                "dataset_slug": slug,
                "parent_sha256": parent_hash,
                "ordered_subset_sha256": selection["ordered_subset_sha256"],
                "ordered_row_bindings_sha256": selection["ordered_row_bindings_sha256"],
            }
        )
    if manifest.get("cohort_sha256") != canonical_sha256(cohort_bindings):
        raise ManifestError("cohort_sha256 mismatch")


def write_manifest(path: Path, manifest: Mapping[str, Any]) -> None:
    output = path.resolve(strict=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(
                manifest,
                handle,
                indent=2,
                sort_keys=True,
                ensure_ascii=True,
                allow_nan=False,
            )
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, output)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Build the six-dataset Table 5 prefix cohort without running a simulator"
        )
    )
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--sample-size", type=int, default=DEFAULT_SAMPLE_SIZE)
    parser.add_argument(
        "--articraft-parent",
        type=Path,
        help=(
            "frozen Articraft parent roster selected from the GitHub release "
            "manifest; defaults to the historical Table 2 cohort"
        ),
    )
    parser.add_argument(
        "--articraft-category-records-root",
        type=Path,
        default=DEFAULT_ARTICRAFT_CATEGORY_RECORDS_ROOT,
    )
    arguments = parser.parse_args(argv)
    try:
        parent_paths = (
            {"articraft_10k": arguments.articraft_parent}
            if arguments.articraft_parent is not None
            else None
        )
        manifest = build_manifest(
            sample_size=arguments.sample_size,
            parent_paths=parent_paths,
            articraft_category_records_root=arguments.articraft_category_records_root,
        )
        validate_manifest(manifest)
        write_manifest(arguments.out, manifest)
    except (ManifestError, OSError) as error:
        parser.error(str(error))
    print(
        json.dumps(
            {
                "out": str(arguments.out.resolve(strict=False)),
                "sample_size": manifest["sample_size"],
                "total_rows": manifest["total_rows"],
                "cohort_sha256": manifest["cohort_sha256"],
                "manifest_sha256": manifest["manifest_sha256"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
