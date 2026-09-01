#!/usr/bin/env python3
"""Evaluate one Table 4 v4 asset without loading a physics simulator."""

from __future__ import annotations

import argparse
from collections import defaultdict
import datetime as dt
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from typing import Any, Mapping, Sequence


SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
EXP_ROOT = SCRIPT.parents[1]
SAMPLING_CORE_SCRIPT = SCRIPT_DIR / "run_urdf_table4_partnet_mobility.py"
ORACLE_SCRIPT = SCRIPT_DIR / "table4_simulator_free_oracle.py"
CONTACT_POLICY_SCRIPT = SCRIPT_DIR / "table4_intended_contact_policy.py"
DEFAULT_CONTACT_REGISTRY = (
    EXP_ROOT / "reference" / "pva_table4_intended_contacts_v1.json"
)
DEFAULT_CHILD_PYTHON = SCRIPT.parents[2] / "arti-template" / ".venv" / "bin" / "python"

PROTOCOL_ID = "urdf_sim_ready_table4_simulator_free_v4"
RESULT_SCHEMA = "table4_simulator_free_result_v4"
STATE_SCHEMA = "table4_simulator_free_state_v4"
SUMMARY_SCHEMA = "table4_simulator_free_summary_v4"
SAMPLING_PROTOCOL = "mimic_aware_independent_sampling_v2"
SINGLE_SAMPLES = 21
SOBOL_SAMPLES = 64
SOBOL_SEED = 20260813
PENETRATION_THRESHOLD_M = 1e-6
MAX_CONTACT_EVIDENCE_PER_STATE = 8
BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT = "N/E"
MEASURED_COLLISION_GEOMETRY = "MEASURED"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def _python_launcher(path: Path) -> Path:
    launcher = Path(os.path.abspath(os.fspath(path)))
    if not launcher.is_file() or not os.access(launcher, os.X_OK):
        raise ValueError(f"child Python launcher is not executable: {launcher}")
    return launcher


def _atomic_json(path: Path, value: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.is_symlink():
        raise ValueError(f"refusing to overwrite symlink: {path}")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(
                value,
                stream,
                sort_keys=True,
                indent=2,
                ensure_ascii=True,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _load_module(name: str, path: Path) -> Any:
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def sampling_core() -> Any:
    return _load_module("table4_v4_sampling_core", SAMPLING_CORE_SCRIPT)


def geometry_oracle() -> Any:
    return _load_module("table4_simulator_free_oracle", ORACLE_SCRIPT)


def contact_policy() -> Any:
    return _load_module("table4_intended_contact_policy", CONTACT_POLICY_SCRIPT)


def runtime_identity() -> dict[str, Any]:
    identity = dict(geometry_oracle().runtime_identity())
    identity.update(
        {
            "evaluator_source_sha256": sha256_file(SCRIPT),
            "sampling_core_source_sha256": sha256_file(SAMPLING_CORE_SCRIPT),
            "contact_policy_source_sha256": sha256_file(CONTACT_POLICY_SCRIPT),
        }
    )
    return identity


def _source_hashes() -> dict[str, str]:
    return {
        "simulator_free_runner": sha256_file(SCRIPT),
        "simulator_free_oracle": sha256_file(ORACLE_SCRIPT),
        "sampling_core": sha256_file(SAMPLING_CORE_SCRIPT),
        "intended_contact_policy": sha256_file(CONTACT_POLICY_SCRIPT),
    }


def _verify_job_source_bindings(job: Mapping[str, Any]) -> None:
    expected = job.get("execution_source_hashes")
    if not isinstance(expected, Mapping):
        raise ValueError("simulator-free execution source hashes are missing")
    observed = _source_hashes()
    if dict(expected) != observed:
        changed = sorted(
            key
            for key in set(expected) | set(observed)
            if expected.get(key) != observed.get(key)
        )
        raise ValueError("simulator-free execution source drift: " + ", ".join(changed))
    if job.get("execution_source_hashes_sha256") != canonical_sha256(observed):
        raise ValueError("simulator-free execution source hash-map digest mismatch")


def _validate_external_binding(job: Mapping[str, Any]) -> None:
    binding = job.get("kinematic_constraint_binding")
    binding_sha256 = job.get("kinematic_constraint_binding_sha256")
    constraints = job.get("external_joint_constraints", [])
    if binding is None:
        if binding_sha256 is not None or constraints:
            raise ValueError("external joint constraints have no immutable binding")
        return
    if not isinstance(binding, Mapping):
        raise ValueError("kinematic constraint binding is not an object")
    payload = dict(binding)
    declared = payload.pop("binding_content_sha256", None)
    computed = canonical_sha256(payload)
    if declared != computed or binding_sha256 != computed:
        raise ValueError("kinematic constraint binding hash mismatch")
    if binding.get("external_joint_constraints") != constraints:
        raise ValueError("kinematic constraint binding payload mismatch")


def _load_bound_contact_registry(
    job: Mapping[str, Any], *, urdf: Path
) -> tuple[dict[str, Any], dict[str, Any]]:
    registry_path_value = job.get("intended_contact_registry_path")
    if not isinstance(registry_path_value, str) or not registry_path_value:
        raise ValueError("intended-contact registry path is missing")
    registry_path = Path(registry_path_value).resolve(strict=True)
    if not registry_path.is_file() or registry_path.is_symlink():
        raise ValueError("intended-contact registry must be a regular file")
    observed_hash = sha256_file(registry_path)
    if observed_hash != job.get("intended_contact_registry_sha256"):
        raise ValueError("intended-contact registry hash drift")
    policy = contact_policy()
    registry = policy.validate_registry(_load_json(registry_path))
    bound = policy.bind_registry_for_asset(
        registry,
        dataset=str(job.get("dataset") or ""),
        asset_id=str(job.get("asset_id", job.get("dataset_id")) or ""),
        urdf=urdf,
    )
    return registry, bound


def _empty_record(
    job: Mapping[str, Any], issue: str, *, status: str = "error"
) -> dict[str, Any]:
    declared = int(job.get("expected_movable_joints", 0) or 0)
    independent = int(job.get("independent_dof_count", declared) or 0)
    result: dict[str, Any] = {
        "schema_version": RESULT_SCHEMA,
        "protocol_id": job.get("protocol_id", PROTOCOL_ID),
        "evaluator_protocol_id": job.get("evaluator_protocol_id", PROTOCOL_ID),
        "dataset": job.get("dataset"),
        "dataset_id": str(job.get("dataset_id")),
        "asset_id": str(job.get("asset_id", job.get("dataset_id"))),
        "order": int(job.get("order", 0)),
        "category": job.get("category", ""),
        "package": job.get("package"),
        "urdf_path": job.get("urdf_path"),
        "expected_primary_urdf_sha256": job.get("expected_primary_urdf_sha256"),
        "expected_movable_joints": declared,
        "sampling_protocol": job.get("sampling_protocol", SAMPLING_PROTOCOL),
        "joint_sampling_plan_sha256": job.get("joint_sampling_plan_sha256"),
        "independent_dof_count": independent,
        "range_evaluable_independent_dof_count": int(
            job.get("range_evaluable_independent_dof_count", 0) or 0
        ),
        "mimic_joint_count": int(job.get("mimic_joint_count", 0) or 0),
        "native_mimic_joint_count": int(job.get("native_mimic_joint_count", 0) or 0),
        "external_joint_constraint_count": int(
            job.get("external_joint_constraint_count", 0) or 0
        ),
        "fixed_root_joint_count": int(job.get("fixed_root_joint_count", 0) or 0),
        "kinematic_constraint_binding": job.get("kinematic_constraint_binding"),
        "kinematic_constraint_binding_sha256": job.get(
            "kinematic_constraint_binding_sha256"
        ),
        "input_identity_sha256": job.get("input_identity_sha256"),
        "collision_oracle": job.get("collision_oracle"),
        "execution_source_hashes": job.get("execution_source_hashes"),
        "execution_source_hashes_sha256": job.get("execution_source_hashes_sha256"),
        "child_runtime_identity_sha256": job.get("child_runtime_identity_sha256"),
        "intended_contact_registry_path": job.get("intended_contact_registry_path"),
        "intended_contact_registry_sha256": job.get("intended_contact_registry_sha256"),
        "status": status,
        "collision_metric_status": BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT,
        "native_collision_elements": 0,
        "load_success": False,
        "measurement_complete": False,
        "movable_dof_count": declared,
        "range_evaluable_dof_count": 0,
        "object_bbox_diagonal_m": None,
        "rest_state_expected": 1,
        "rest_state_executed": 0,
        "single_state_expected": int(
            job.get("single_state_expected", SINGLE_SAMPLES * independent) or 0
        ),
        "single_state_executed": 0,
        "sobol_state_expected": int(
            job.get("sobol_state_expected", SOBOL_SAMPLES if independent else 0) or 0
        ),
        "sobol_state_executed": 0,
        "rest_all_pair_cf": False,
        "rest_non_adjacent_cf": False,
        "single_joint_sweep_cf": False,
        "multi_joint_sobol_cf": False,
        "strict_collision_pass": False,
        "raw_rest_all_pair_cf": False,
        "raw_rest_non_adjacent_cf": False,
        "raw_single_joint_sweep_cf": False,
        "raw_multi_joint_sobol_cf": False,
        "raw_strict_penetration_proxy_pass": False,
        "raw_rest_all_pair_intersection_free": False,
        "raw_rest_non_adjacent_intersection_free": False,
        "raw_single_joint_intersection_free": False,
        "raw_sobol_intersection_free": False,
        "raw_strict_intersection_free_pass": False,
        "adjusted_rest_all_pair_intersection_free": False,
        "adjusted_rest_non_adjacent_intersection_free": False,
        "adjusted_single_joint_intersection_free": False,
        "adjusted_sobol_intersection_free": False,
        "adjusted_strict_intersection_free_pass": False,
        "rest_non_adjacent_free": 0,
        "single_non_adjacent_free": 0,
        "sobol_non_adjacent_free": 0,
        "raw_rest_non_adjacent_free": 0,
        "raw_single_non_adjacent_free": 0,
        "raw_sobol_non_adjacent_free": 0,
        "joint_single_sweep_cf_passed": 0,
        "max_penetration_m": None,
        "max_penetration_normalized": None,
        "raw_max_penetration_depth_proxy_m": None,
        "raw_max_penetration_depth_proxy_normalized": None,
        "adjusted_max_penetration_depth_proxy_m": None,
        "adjusted_max_penetration_depth_proxy_normalized": None,
        "max_penetration_depth_proxy_m": None,
        "max_penetration_depth_proxy_normalized": None,
        "max_penetration_depth_proxy_view": "contact_adjusted",
        "contact_depth_semantics": "fcl_contact_depth_proxy_not_global_mesh_mtd",
        "state_records": [],
        "state_records_sha256": canonical_sha256([]),
        "state_records_count": 0,
        "issues": [issue],
        "runtime_identity": None,
        "final_runtime_identity": None,
        "runtime_integrity_fatal": False,
        "execution_source_integrity_fatal": False,
    }
    for field in (
        "package_root_binding_content_sha256",
        "execution_input_sha256",
        "evaluation_package_path",
        "evaluation_urdf_path",
        "evaluation_package_relative_path",
        "evaluation_urdf_relative_path",
        "package_binding_sha256",
    ):
        if field in job:
            result[field] = job.get(field)
    return result


def _state_view(
    observation: Mapping[str, Any], *, non_adjacent: bool
) -> dict[str, bool]:
    prefix = "non_adjacent" if non_adjacent else "all_pair"
    return {
        "raw_penetration_free": int(
            observation[f"raw_{prefix}_illegal_penetration_count"]
        )
        == 0,
        "adjusted_penetration_free": int(
            observation[f"{prefix}_illegal_penetration_count"]
        )
        == 0,
        "raw_intersection_free": int(observation[f"{prefix}_intersection_count"]) == 0,
        "adjusted_intersection_free": int(
            observation[f"adjusted_{prefix}_intersection_count"]
        )
        == 0,
    }


def evaluate_job(job: Mapping[str, Any]) -> dict[str, Any]:
    result = _empty_record(job, "evaluation_not_completed")
    states: list[dict[str, Any]] = []
    urdf = Path(str(job.get("evaluation_urdf_path", job.get("urdf_path", ""))))
    package = Path(str(job.get("evaluation_package_path", job.get("package", ""))))
    expected_hash = str(job.get("expected_primary_urdf_sha256") or "")
    try:
        if job.get("evaluator_protocol_id") != PROTOCOL_ID:
            raise ValueError("simulator-free evaluator protocol ID mismatch")
        if job.get("sampling_protocol") != SAMPLING_PROTOCOL:
            raise ValueError("simulator-free sampling protocol mismatch")
        _verify_job_source_bindings(job)
        _validate_external_binding(job)
        identity = runtime_identity()
        identity_sha256 = canonical_sha256(identity)
        if identity_sha256 != job.get("child_runtime_identity_sha256"):
            raise RuntimeError(
                "simulator-free child runtime identity drift before evaluation"
            )
        if not identity.get("simulator_free_runtime_verified"):
            raise RuntimeError(
                "simulator module loaded in v4 child: "
                + ", ".join(identity.get("simulator_modules_loaded", []))
            )
        result["runtime_identity"] = identity
        if not urdf.is_file():
            raise FileNotFoundError(f"primary URDF missing: {urdf}")
        if expected_hash and sha256_file(urdf) != expected_hash:
            raise ValueError("primary URDF hash drift before evaluation")
        _registry, bound_registry = _load_bound_contact_registry(job, urdf=urdf)
        result["intended_contact_policy_id"] = bound_registry["policy_id"]
        result["bound_intended_contact_registration_count"] = len(
            bound_registry["entries"]
        )

        core = sampling_core()
        joints = core.parse_urdf_joints(urdf)
        names = [str(row.get("name") or "") for row in joints]
        if any(not name for name in names) or len(names) != len(set(names)):
            raise ValueError("movable URDF joint names must be non-empty and unique")
        plan = core.compile_joint_sampling_plan(
            joints,
            external_joint_constraints=job.get("external_joint_constraints", []),
        )
        if job.get("joint_sampling_plan_sha256") != plan["plan_sha256"]:
            raise ValueError("joint sampling plan hash mismatch")
        independent_rows = list(plan["independent_joints"])
        independent_dof = int(plan["independent_dof_count"])
        model = geometry_oracle().load_model(urdf, package)
        model_joint_names = [joint.name for joint in model.movable_joints]
        if model_joint_names != names:
            raise ValueError("analytic FK joint order differs from sampling-core order")
        if len(joints) != int(job.get("expected_movable_joints", -1)):
            raise ValueError("roster movable-joint count mismatch")

        result.update(
            {
                "native_collision_elements": len(model.collision_elements),
                "link_count": len(model.links),
                "geometry_kinds": sorted(
                    element.geometry_kind for element in model.collision_elements
                ),
                "movable_dof_count": len(joints),
                "range_evaluable_dof_count": sum(
                    bool(row.get("range_evaluable")) for row in joints
                ),
                "independent_dof_count": independent_dof,
                "range_evaluable_independent_dof_count": int(
                    plan["range_evaluable_independent_dof_count"]
                ),
                "mimic_joint_count": int(plan["mimic_joint_count"]),
                "native_mimic_joint_count": int(
                    plan.get("native_mimic_joint_count", 0)
                ),
                "external_joint_constraint_count": int(
                    plan.get("external_joint_constraint_count", 0)
                ),
                "fixed_root_joint_count": int(plan["fixed_root_joint_count"]),
                "load_success": True,
            }
        )
        if not model.collision_elements:
            result["status"] = "blocked"
            result["collision_metric_status"] = BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT
            result["issues"] = ["no_native_collision_geometry"]
            return result

        rest_values = core.expand_joint_values(plan, [0.0] * independent_dof)
        result["object_bbox_diagonal_m"] = model.object_bbox_diagonal_m(rest_values)
        result["single_state_expected"] = SINGLE_SAMPLES * independent_dof
        result["sobol_state_expected"] = SOBOL_SAMPLES if independent_dof else 0
        if int(job.get("single_state_expected", -1)) != result["single_state_expected"]:
            raise ValueError("single-state denominator mismatch")
        if int(job.get("sobol_state_expected", -1)) != result["sobol_state_expected"]:
            raise ValueError("Sobol-state denominator mismatch")

        def observe(
            values: Sequence[float],
            phase: str,
            sample_index: int,
            joint_name: str | None = None,
        ) -> dict[str, Any]:
            observation = model.observe(
                values,
                phase=phase,
                dataset=str(job["dataset"]),
                asset_id=str(job.get("asset_id", job["dataset_id"])),
                urdf_sha256=expected_hash or sha256_file(urdf),
                intended_contact_registry=bound_registry,
                penetration_threshold_m=PENETRATION_THRESHOLD_M,
                max_recorded_contacts=MAX_CONTACT_EVIDENCE_PER_STATE,
            )
            non_adjacent_view = _state_view(observation, non_adjacent=True)
            all_pair_view = _state_view(observation, non_adjacent=False)
            state = {
                "schema_version": STATE_SCHEMA,
                "protocol_id": job["protocol_id"],
                "evaluator_protocol_id": PROTOCOL_ID,
                "dataset": job["dataset"],
                "dataset_id": str(job["dataset_id"]),
                "order": int(job["order"]),
                "phase": phase,
                "sample_index": sample_index,
                "joint_name": joint_name,
                "joint_values_sha256": core.canonical_sha256(
                    [float(value) for value in values]
                ),
                "fk_backend": "analytic_urdf_tree_v1",
                "collision_oracle": geometry_oracle().ORACLE_POLICY_ID,
                "execution_source_hashes_sha256": job["execution_source_hashes_sha256"],
                "kinematic_constraint_binding_sha256": job.get(
                    "kinematic_constraint_binding_sha256"
                ),
                "intended_contact_registry_sha256": job[
                    "intended_contact_registry_sha256"
                ],
                "raw_penetration_proxy_free": non_adjacent_view["raw_penetration_free"],
                "adjusted_penetration_proxy_free": non_adjacent_view[
                    "adjusted_penetration_free"
                ],
                "raw_intersection_free": non_adjacent_view["raw_intersection_free"],
                "adjusted_intersection_free": non_adjacent_view[
                    "adjusted_intersection_free"
                ],
                "raw_all_pair_penetration_proxy_free": all_pair_view[
                    "raw_penetration_free"
                ],
                "adjusted_all_pair_penetration_proxy_free": all_pair_view[
                    "adjusted_penetration_free"
                ],
                "raw_all_pair_intersection_free": all_pair_view[
                    "raw_intersection_free"
                ],
                "adjusted_all_pair_intersection_free": all_pair_view[
                    "adjusted_intersection_free"
                ],
                "raw_metric_max_penetration_depth_proxy_m": float(
                    observation["raw_non_adjacent_max_penetration_proxy_m"]
                ),
                "adjusted_metric_max_penetration_depth_proxy_m": float(
                    observation["non_adjacent_max_penetration_m"]
                ),
                # Compatibility alias.  The explicit view field on the asset
                # record prevents this adjusted proxy from being read as raw.
                "metric_max_penetration_m": float(
                    observation["non_adjacent_max_penetration_m"]
                ),
                **observation,
            }
            states.append(state)
            return state

        rest = observe(rest_values, "rest", 0)
        result["rest_state_executed"] = 1
        result["raw_rest_all_pair_cf"] = bool(
            rest["raw_all_pair_penetration_proxy_free"]
        )
        result["raw_rest_non_adjacent_cf"] = bool(rest["raw_penetration_proxy_free"])
        result["rest_all_pair_cf"] = bool(
            rest["adjusted_all_pair_penetration_proxy_free"]
        )
        result["rest_non_adjacent_cf"] = bool(rest["adjusted_penetration_proxy_free"])
        result["raw_rest_all_pair_intersection_free"] = bool(
            rest["raw_all_pair_intersection_free"]
        )
        result["raw_rest_non_adjacent_intersection_free"] = bool(
            rest["raw_intersection_free"]
        )
        result["adjusted_rest_all_pair_intersection_free"] = bool(
            rest["adjusted_all_pair_intersection_free"]
        )
        result["adjusted_rest_non_adjacent_intersection_free"] = bool(
            rest["adjusted_intersection_free"]
        )
        result["rest_non_adjacent_free"] = int(result["rest_non_adjacent_cf"])
        result["raw_rest_non_adjacent_free"] = int(result["raw_rest_non_adjacent_cf"])

        raw_single_ok = True
        adjusted_single_ok = True
        raw_single_intersection_ok = True
        adjusted_single_intersection_ok = True
        for position, joint in enumerate(independent_rows):
            if not bool(joint.get("sampling_range_evaluable")):
                continue
            joint_adjusted_ok = True
            for sample_index, value in enumerate(core.single_joint_values(joint)):
                independent_values = [0.0] * independent_dof
                independent_values[position] = float(value)
                values = core.expand_joint_values(plan, independent_values)
                state = observe(
                    values,
                    "single_joint_sweep",
                    sample_index,
                    str(joint["name"]),
                )
                result["single_state_executed"] += 1
                raw_free = bool(state["raw_penetration_proxy_free"])
                adjusted_free = bool(state["adjusted_penetration_proxy_free"])
                raw_intersection_free = bool(state["raw_intersection_free"])
                adjusted_intersection_free = bool(state["adjusted_intersection_free"])
                raw_single_ok = raw_single_ok and raw_free
                adjusted_single_ok = adjusted_single_ok and adjusted_free
                raw_single_intersection_ok = (
                    raw_single_intersection_ok and raw_intersection_free
                )
                adjusted_single_intersection_ok = (
                    adjusted_single_intersection_ok and adjusted_intersection_free
                )
                joint_adjusted_ok = joint_adjusted_ok and adjusted_free
                result["raw_single_non_adjacent_free"] += int(raw_free)
                result["single_non_adjacent_free"] += int(adjusted_free)
            result["joint_single_sweep_cf_passed"] += int(joint_adjusted_ok)

        raw_sobol_ok = True
        adjusted_sobol_ok = True
        raw_sobol_intersection_ok = True
        adjusted_sobol_intersection_ok = True
        if independent_rows and all(
            bool(row.get("sampling_range_evaluable")) for row in independent_rows
        ):
            for sample_index, sampled in enumerate(
                core.sobol_joint_values(independent_rows, seed=SOBOL_SEED)
            ):
                values = core.expand_joint_values(
                    plan, [float(value) for value in sampled]
                )
                state = observe(values, "multi_joint_sobol", sample_index)
                result["sobol_state_executed"] += 1
                raw_free = bool(state["raw_penetration_proxy_free"])
                adjusted_free = bool(state["adjusted_penetration_proxy_free"])
                raw_intersection_free = bool(state["raw_intersection_free"])
                adjusted_intersection_free = bool(state["adjusted_intersection_free"])
                raw_sobol_ok = raw_sobol_ok and raw_free
                adjusted_sobol_ok = adjusted_sobol_ok and adjusted_free
                raw_sobol_intersection_ok = (
                    raw_sobol_intersection_ok and raw_intersection_free
                )
                adjusted_sobol_intersection_ok = (
                    adjusted_sobol_intersection_ok and adjusted_intersection_free
                )
                result["raw_sobol_non_adjacent_free"] += int(raw_free)
                result["sobol_non_adjacent_free"] += int(adjusted_free)

        result["measurement_complete"] = bool(
            result["range_evaluable_independent_dof_count"] == independent_dof
            and result["rest_state_executed"] == result["rest_state_expected"]
            and result["single_state_executed"] == result["single_state_expected"]
            and result["sobol_state_executed"] == result["sobol_state_expected"]
        )
        if independent_dof == 0:
            raw_single_ok = adjusted_single_ok = True
            raw_single_intersection_ok = adjusted_single_intersection_ok = True
            raw_sobol_ok = adjusted_sobol_ok = False
            raw_sobol_intersection_ok = adjusted_sobol_intersection_ok = False
            result["zero_dof_fail_closed"] = True
            result["joint_single_sweep_cf_passed"] = 0

        result["raw_single_joint_sweep_cf"] = bool(
            result["measurement_complete"] and raw_single_ok
        )
        result["single_joint_sweep_cf"] = bool(
            result["measurement_complete"] and adjusted_single_ok
        )
        result["raw_multi_joint_sobol_cf"] = bool(
            result["measurement_complete"] and raw_sobol_ok
        )
        result["multi_joint_sobol_cf"] = bool(
            result["measurement_complete"] and adjusted_sobol_ok
        )
        result["raw_single_joint_intersection_free"] = bool(
            result["measurement_complete"] and raw_single_intersection_ok
        )
        result["adjusted_single_joint_intersection_free"] = bool(
            result["measurement_complete"] and adjusted_single_intersection_ok
        )
        result["raw_sobol_intersection_free"] = bool(
            result["measurement_complete"] and raw_sobol_intersection_ok
        )
        result["adjusted_sobol_intersection_free"] = bool(
            result["measurement_complete"] and adjusted_sobol_intersection_ok
        )
        result["raw_strict_penetration_proxy_pass"] = bool(
            result["measurement_complete"]
            and result["raw_rest_non_adjacent_cf"]
            and result["raw_single_joint_sweep_cf"]
            and result["raw_multi_joint_sobol_cf"]
        )
        result["strict_collision_pass"] = bool(
            result["measurement_complete"]
            and result["rest_non_adjacent_cf"]
            and result["single_joint_sweep_cf"]
            and result["multi_joint_sobol_cf"]
        )
        result["raw_strict_intersection_free_pass"] = bool(
            result["measurement_complete"]
            and result["raw_rest_non_adjacent_intersection_free"]
            and result["raw_single_joint_intersection_free"]
            and result["raw_sobol_intersection_free"]
        )
        result["adjusted_strict_intersection_free_pass"] = bool(
            result["measurement_complete"]
            and result["adjusted_rest_non_adjacent_intersection_free"]
            and result["adjusted_single_joint_intersection_free"]
            and result["adjusted_sobol_intersection_free"]
        )
        if independent_dof == 0:
            result["raw_strict_penetration_proxy_pass"] = False
            result["strict_collision_pass"] = False
            result["raw_strict_intersection_free_pass"] = False
            result["adjusted_strict_intersection_free_pass"] = False

        raw_proxy_max = max(
            (
                float(state["raw_metric_max_penetration_depth_proxy_m"])
                for state in states
            ),
            default=0.0,
        )
        adjusted_proxy_max = max(
            (
                float(state["adjusted_metric_max_penetration_depth_proxy_m"])
                for state in states
            ),
            default=0.0,
        )
        diagonal = float(result["object_bbox_diagonal_m"])
        result["raw_max_penetration_depth_proxy_m"] = raw_proxy_max
        result["raw_max_penetration_depth_proxy_normalized"] = raw_proxy_max / diagonal
        result["adjusted_max_penetration_depth_proxy_m"] = adjusted_proxy_max
        result["adjusted_max_penetration_depth_proxy_normalized"] = (
            adjusted_proxy_max / diagonal
        )
        result["max_penetration_depth_proxy_m"] = adjusted_proxy_max
        result["max_penetration_depth_proxy_normalized"] = adjusted_proxy_max / diagonal
        # Exact maximum penetration remains N/E for arbitrary triangle meshes.
        result["max_penetration_m"] = None
        result["max_penetration_normalized"] = None
        result["collision_metric_status"] = MEASURED_COLLISION_GEOMETRY
        result["status"] = "completed" if result["measurement_complete"] else "error"
        result["issues"] = (
            [] if result["measurement_complete"] else ["measurement_incomplete"]
        )
    except BaseException as exc:  # child boundary is intentionally fail-closed
        result["status"] = "error"
        result["issues"] = [f"{type(exc).__name__}: {exc}"]
        result["measurement_complete"] = False
        if result.get("native_collision_elements", 0):
            result["collision_metric_status"] = "partial" if states else "N/E"
    finally:
        if expected_hash and urdf.is_file() and sha256_file(urdf) != expected_hash:
            result["status"] = "error"
            result["measurement_complete"] = False
            result["issues"] = ["primary URDF hash drift after evaluation"]
        try:
            _verify_job_source_bindings(job)
        except BaseException as exc:
            result["status"] = "error"
            result["measurement_complete"] = False
            result["execution_source_integrity_fatal"] = True
            result["issues"] = [f"execution_source_integrity_fatal: {exc}"]
        try:
            final_identity = runtime_identity()
            result["final_runtime_identity"] = final_identity
            if canonical_sha256(final_identity) != job.get(
                "child_runtime_identity_sha256"
            ):
                raise RuntimeError(
                    "simulator-free child runtime identity drift during evaluation"
                )
            if not final_identity.get("simulator_free_runtime_verified"):
                raise RuntimeError("simulator module loaded during v4 evaluation")
        except BaseException as exc:
            result["status"] = "error"
            result["measurement_complete"] = False
            result["runtime_integrity_fatal"] = True
            result["issues"] = [f"runtime_integrity_fatal: {exc}"]
    result["state_records"] = states
    result["state_records_count"] = len(states)
    result["state_records_sha256"] = canonical_sha256(states)
    expected_states = (
        int(result.get("rest_state_expected", 0))
        + int(result.get("single_state_expected", 0))
        + int(result.get("sobol_state_expected", 0))
    )
    result["unexecuted_state_count"] = max(0, expected_states - len(states))
    return result


def validate_result_binding(result: Mapping[str, Any], job: Mapping[str, Any]) -> None:
    required = {
        "schema_version": RESULT_SCHEMA,
        "protocol_id": job.get("protocol_id"),
        "evaluator_protocol_id": PROTOCOL_ID,
        "dataset": job.get("dataset"),
        "dataset_id": str(job.get("dataset_id")),
        "order": int(job.get("order", 0)),
        "sampling_protocol": SAMPLING_PROTOCOL,
        "joint_sampling_plan_sha256": job.get("joint_sampling_plan_sha256"),
        "input_identity_sha256": job.get("input_identity_sha256"),
        "collision_oracle": job.get("collision_oracle"),
        "execution_source_hashes_sha256": job.get("execution_source_hashes_sha256"),
        "child_runtime_identity_sha256": job.get("child_runtime_identity_sha256"),
        "intended_contact_registry_sha256": job.get("intended_contact_registry_sha256"),
    }
    for field, expected in required.items():
        if result.get(field) != expected:
            raise ValueError(f"simulator-free child binding mismatch: {field}")
    for field in ("runtime_integrity_fatal", "execution_source_integrity_fatal"):
        if not isinstance(result.get(field), bool):
            raise ValueError(f"simulator-free child {field} must be boolean")
        if result[field] and result.get("status") != "error":
            raise ValueError(f"simulator-free child {field} requires error status")
    for field in ("runtime_identity", "final_runtime_identity"):
        identity = result.get(field)
        if identity is None and str(result.get("status")) in {"timeout", "error"}:
            continue
        if not isinstance(identity, Mapping):
            raise ValueError(f"simulator-free child {field} is invalid")
        if canonical_sha256(identity) != job.get(
            "child_runtime_identity_sha256"
        ) and not bool(result.get("runtime_integrity_fatal")):
            raise ValueError(f"simulator-free child {field} binding mismatch")
    states = result.get("state_records")
    if not isinstance(states, list) or any(
        not isinstance(state, Mapping) for state in states
    ):
        raise ValueError("simulator-free child states are invalid")
    if result.get("state_records_count") != len(states):
        raise ValueError("simulator-free child state count mismatch")
    if result.get("state_records_sha256") != canonical_sha256(states):
        raise ValueError("simulator-free child state hash mismatch")


def _child_main(job_path: Path, result_path: Path) -> int:
    try:
        job = _load_json(job_path)
        result = evaluate_job(job)
    except BaseException as exc:
        try:
            job = _load_json(job_path)
            result = _empty_record(job, f"child_exception: {type(exc).__name__}: {exc}")
        except BaseException:
            return 1
    _atomic_json(result_path, result)
    return 0


def _kill_group(process: subprocess.Popen[bytes]) -> None:
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except (OSError, AttributeError):
        process.terminate()
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except (OSError, AttributeError):
            process.kill()
        process.wait(timeout=5.0)


def execute_child(
    job: Mapping[str, Any],
    output: Path,
    timeout_seconds: float,
    *,
    python: Path = DEFAULT_CHILD_PYTHON,
) -> dict[str, Any]:
    python = _python_launcher(python)
    output = Path(output)
    index = int(job["order"])
    jobs_dir = output / "child_jobs"
    children_dir = output / "children"
    logs_dir = output / "child_logs"
    for directory in (jobs_dir, children_dir, logs_dir):
        directory.mkdir(parents=True, exist_ok=True)
    job_path = jobs_dir / f"{index:06d}.json"
    result_path = children_dir / f"{index:06d}.json"
    log_path = logs_dir / f"{index:06d}.log"
    _atomic_json(job_path, dict(job))
    result_path.unlink(missing_ok=True)
    environment = dict(os.environ)
    environment.update(
        {
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
            "PYTHONDONTWRITEBYTECODE": "1",
        }
    )
    command = [
        str(python),
        str(SCRIPT),
        "--child",
        "--job",
        str(job_path),
        "--result",
        str(result_path),
    ]
    with log_path.open("wb") as log:
        process = subprocess.Popen(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            env=environment,
            start_new_session=True,
        )
        try:
            return_code = process.wait(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            _kill_group(process)
            return _empty_record(
                job,
                f"asset_timeout_after_{timeout_seconds:g}_seconds",
                status="timeout",
            )
    if return_code != 0 or not result_path.is_file():
        return _empty_record(
            job, f"child_failed_returncode_{return_code}", status="error"
        )
    result = _load_json(result_path)
    validate_result_binding(result, job)
    return result


def _fraction(passed: int, denominator: int) -> dict[str, Any]:
    return {
        "passed": int(passed),
        "denominator": int(denominator),
        "rate": passed / denominator if denominator else None,
    }


def aggregate_records(
    records: Sequence[Mapping[str, Any]], n_eval: int, j_eval: int
) -> dict[str, Any]:
    if len(records) != n_eval:
        raise ValueError(f"record count mismatch: {len(records)} != {n_eval}")
    if any(bool(record.get("runtime_integrity_fatal")) for record in records):
        raise RuntimeError("refusing to aggregate a run that loaded a simulator module")
    if any(bool(record.get("execution_source_integrity_fatal")) for record in records):
        raise RuntimeError("refusing to aggregate a run with execution-source drift")
    keys = (
        "raw_rest_all_pair_cf",
        "rest_all_pair_cf",
        "raw_rest_non_adjacent_cf",
        "rest_non_adjacent_cf",
        "raw_single_joint_sweep_cf",
        "single_joint_sweep_cf",
        "raw_multi_joint_sobol_cf",
        "multi_joint_sobol_cf",
        "raw_strict_penetration_proxy_pass",
        "strict_collision_pass",
        "raw_rest_non_adjacent_intersection_free",
        "adjusted_rest_non_adjacent_intersection_free",
        "raw_single_joint_intersection_free",
        "adjusted_single_joint_intersection_free",
        "raw_sobol_intersection_free",
        "adjusted_sobol_intersection_free",
        "raw_strict_intersection_free_pass",
        "adjusted_strict_intersection_free_pass",
    )
    passed = {key: 0 for key in keys}
    status_counts: dict[str, int] = defaultdict(int)
    expected_states = executed_states = 0
    collision_assets = measured_assets = 0
    raw_proxy_values: list[float] = []
    adjusted_proxy_values: list[float] = []
    raw_free_states = adjusted_free_states = 0
    raw_free_single_states = adjusted_free_single_states = 0
    expected_single_states = 0
    bound_registrations = 0
    for record in records:
        status_counts[str(record.get("status", "error"))] += 1
        native = int(record.get("native_collision_elements", 0) or 0) > 0
        collision_assets += int(native)
        measured_assets += int(native and bool(record.get("measurement_complete")))
        bound_registrations += int(
            record.get("bound_intended_contact_registration_count", 0) or 0
        )
        expected_states += sum(
            int(record.get(f"{phase}_state_expected", 0) or 0)
            for phase in ("rest", "single", "sobol")
        )
        executed_states += sum(
            int(record.get(f"{phase}_state_executed", 0) or 0)
            for phase in ("rest", "single", "sobol")
        )
        for key in keys:
            passed[key] += int(bool(record.get(key, False)))
        record_raw_single = int(record.get("raw_single_non_adjacent_free", 0) or 0)
        record_adjusted_single = int(record.get("single_non_adjacent_free", 0) or 0)
        raw_free_single_states += record_raw_single
        adjusted_free_single_states += record_adjusted_single
        expected_single_states += int(record.get("single_state_expected", 0) or 0)
        raw_free_states += (
            int(record.get("raw_rest_non_adjacent_free", 0) or 0)
            + record_raw_single
            + int(record.get("raw_sobol_non_adjacent_free", 0) or 0)
        )
        adjusted_free_states += (
            int(record.get("rest_non_adjacent_free", 0) or 0)
            + record_adjusted_single
            + int(record.get("sobol_non_adjacent_free", 0) or 0)
        )
        for field, destination in (
            ("raw_max_penetration_depth_proxy_normalized", raw_proxy_values),
            (
                "adjusted_max_penetration_depth_proxy_normalized",
                adjusted_proxy_values,
            ),
        ):
            proxy = record.get(field)
            if proxy is None:
                continue
            try:
                value = float(proxy)
            except (TypeError, ValueError):
                continue
            if math.isfinite(value) and value >= 0.0:
                destination.append(value)

    metrics = {key: _fraction(passed[key], n_eval) for key in keys}
    raw_collision_states = max(0, expected_states - raw_free_states)
    adjusted_collision_states = max(0, expected_states - adjusted_free_states)
    common_state_coverage = {
        "executed_states": executed_states,
        "unexecuted_states": max(0, expected_states - executed_states),
    }
    metrics["raw_collision_state_rate"] = {
        "collision_states": raw_collision_states,
        "denominator": expected_states,
        "rate": raw_collision_states / expected_states if expected_states else None,
        **common_state_coverage,
    }
    metrics["collision_state_rate"] = {
        "collision_states": adjusted_collision_states,
        "denominator": expected_states,
        "rate": (
            adjusted_collision_states / expected_states if expected_states else None
        ),
        **common_state_coverage,
    }
    metrics["raw_collision_free_range"] = {
        "passed_states": raw_free_single_states,
        "denominator": expected_single_states,
        "rate": (
            raw_free_single_states / expected_single_states
            if expected_single_states
            else None
        ),
    }
    metrics["collision_free_range"] = {
        "passed_states": adjusted_free_single_states,
        "denominator": expected_single_states,
        "rate": (
            adjusted_free_single_states / expected_single_states
            if expected_single_states
            else None
        ),
    }
    if expected_single_states == 0:
        for key in ("raw_collision_free_range", "collision_free_range"):
            metrics[key].update(
                {
                    "status": "N/E",
                    "reason": "cohort_has_no_independent_single_joint_states",
                }
            )
    metrics["max_penetration"] = {
        "status": "N/E",
        "reason": "arbitrary_triangle_mesh_global_mtd_not_available",
    }

    def proxy_metric(values: Sequence[float], *, view: str) -> dict[str, Any]:
        return {
            "status": (
                "COMPLETE"
                if measured_assets == n_eval
                else "PARTIAL"
                if values
                else "N/E"
            ),
            "maximum_observed_normalized": max(values) if values else None,
            "observed_assets": len(values),
            "measured_assets": measured_assets,
            "denominator": n_eval,
            "view": view,
            "semantics": "fcl_contact_depth_proxy_not_global_mesh_mtd",
        }

    metrics["raw_max_penetration_depth_proxy"] = proxy_metric(
        raw_proxy_values, view="raw"
    )
    metrics["adjusted_max_penetration_depth_proxy"] = proxy_metric(
        adjusted_proxy_values, view="contact_adjusted"
    )
    metrics["max_penetration_depth_proxy"] = dict(
        metrics["adjusted_max_penetration_depth_proxy"]
    )
    if collision_assets == 0:
        for key in (
            *keys,
            "raw_collision_state_rate",
            "collision_state_rate",
            "raw_collision_free_range",
            "collision_free_range",
        ):
            metrics[key] = {
                "status": "N/E",
                "reason": "cohort_has_no_native_collision_geometry",
                "denominator": n_eval,
            }
    metrics["aor"] = {
        "status": "N/E",
        "reason": "exact_overlap_volume_backend_not_registered",
    }
    overall_status = (
        "COMPLETE"
        if all(str(row.get("status")) == "completed" for row in records)
        else "COMPLETE_WITH_RETAINED_FAILURES"
    )
    return {
        "schema_version": SUMMARY_SCHEMA,
        "protocol_id": PROTOCOL_ID,
        "status": overall_status,
        "n_eval": n_eval,
        "j_eval": j_eval,
        "collision_geometry_assets": collision_assets,
        "measured_assets": measured_assets,
        "status_counts": dict(sorted(status_counts.items())),
        "state_records_expected": expected_states,
        "state_records_executed": executed_states,
        "bound_intended_contact_registration_count": bound_registrations,
        "metrics": metrics,
        "claim_boundary": {
            "physics_simulator_used": False,
            "forward_kinematics": "analytic_urdf_tree_v1",
            "collision_backend": "python_fcl_0.7",
            "continuous_collision_detection": "not_run",
            "dynamics_validity": "not_evaluated",
            "exact_global_mesh_penetration_depth": "not_available",
            "raw_and_contact_adjusted_results_both_reported": True,
        },
    }


def report_text(summary: Mapping[str, Any], dataset: str) -> str:
    metrics = summary["metrics"]

    def format_fraction(key: str) -> str:
        value = metrics[key]
        if value.get("status") == "N/E" or value.get("rate") is None:
            return "N/E"
        numerator = value.get(
            "passed", value.get("collision_states", value.get("passed_states"))
        )
        return f"{numerator} / {value['denominator']} ({100.0 * value['rate']:.3f}%)"

    def format_proxy(key: str) -> str:
        value = metrics[key]
        if value.get("status") == "N/E":
            return "N/E"
        return (
            f"{float(value['maximum_observed_normalized']):.6f} "
            f"({value['observed_assets']} / {value['denominator']} observed; "
            f"{value['status']})"
        )

    rows = (
        ("raw_rest_all_pair_cf", "Raw rest all-pair penetration-proxy CF"),
        ("rest_all_pair_cf", "Contact-adjusted rest all-pair penetration-proxy CF"),
        ("raw_rest_non_adjacent_cf", "Raw rest penetration-proxy CF"),
        ("rest_non_adjacent_cf", "Contact-adjusted rest penetration-proxy CF"),
        ("raw_single_joint_sweep_cf", "Raw single-joint penetration-proxy CF"),
        ("single_joint_sweep_cf", "Contact-adjusted single-joint penetration-proxy CF"),
        ("raw_multi_joint_sobol_cf", "Raw Sobol penetration-proxy CF"),
        ("multi_joint_sobol_cf", "Contact-adjusted Sobol penetration-proxy CF"),
        ("raw_strict_penetration_proxy_pass", "Raw strict penetration-proxy pass"),
        ("strict_collision_pass", "Contact-adjusted strict penetration-proxy pass"),
        ("raw_strict_intersection_free_pass", "Raw strict intersection-free pass"),
        (
            "adjusted_strict_intersection_free_pass",
            "Contact-adjusted strict intersection-free pass",
        ),
        ("raw_collision_state_rate", "Raw collision-state rate"),
        ("collision_state_rate", "Contact-adjusted collision-state rate"),
        ("raw_collision_free_range", "Raw collision-free range"),
        ("collision_free_range", "Contact-adjusted collision-free range"),
    )
    lines = [
        f"# {dataset}: Table 4 v4 simulator-free evaluation",
        "",
        f"Status: **{summary['status']}**",
        "",
        f"N_eval: {summary['n_eval']}  \\",
        f"J_eval: {summary['j_eval']}",
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    lines.extend(f"| {label} | {format_fraction(key)} |" for key, label in rows)
    lines.extend(
        (
            "| Raw maximum normalized penetration-depth proxy | "
            + format_proxy("raw_max_penetration_depth_proxy")
            + " |",
            "| Contact-adjusted maximum normalized penetration-depth proxy | "
            + format_proxy("adjusted_max_penetration_depth_proxy")
            + " |",
            "| Exact global maximum penetration | N/E |",
            "| AOR | N/E |",
        )
    )
    lines.extend(
        [
            "",
            "No physics simulator is loaded. Exact global mesh penetration depth and AOR remain N/E; the compatibility depth value is explicitly a python-fcl contact-depth proxy.",
            "",
        ]
    )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--job", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--result", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--runtime-identity", action="store_true")
    args = parser.parse_args(argv)
    if args.runtime_identity:
        print(json.dumps(runtime_identity(), sort_keys=True))
        return 0
    if args.child:
        if args.job is None or args.result is None:
            parser.error("--child requires --job and --result")
        return _child_main(args.job, args.result)
    parser.error("this runner is invoked through the v4 dataset adapter")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "DEFAULT_CHILD_PYTHON",
    "DEFAULT_CONTACT_REGISTRY",
    "PROTOCOL_ID",
    "RESULT_SCHEMA",
    "STATE_SCHEMA",
    "SUMMARY_SCHEMA",
    "aggregate_records",
    "canonical_sha256",
    "evaluate_job",
    "execute_child",
    "report_text",
    "runtime_identity",
    "sha256_file",
    "validate_result_binding",
]
