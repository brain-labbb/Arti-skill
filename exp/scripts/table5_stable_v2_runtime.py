#!/usr/bin/env python3
"""Evaluate long-horizon multi-pose hinge stability for frozen Table 5 cohorts."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_v2_runtime as _core  # noqa: E402
import table5_v2_runtime_compat as _compat  # noqa: E402
import table5_v2_runtime_r2 as _r2  # noqa: E402


EVIDENCE_SCHEMA = "table5_stable_v2_evidence_v1"
RECEIPT_SCHEMA = "table5_stable_v2_receipt_v1"
DRIFT_SCHEMA = "table5_stable_v2_constraint_drift_v1"
PROTOCOL_SCHEMA = "table5_stable_v2_protocol_v1"
PROTOCOL_ID = "table5-stable-v2-hinge-long-horizon-20260830"
INITIAL_FRACTIONS = (0.25, 0.5, 0.75)
TIMESTEP_S = 1.0 / 240.0
MAXIMUM_STEPS = 2400
DRIFT_SAMPLE_EVERY_STEPS = 12
RESET_POSITION_TOLERANCE_RANGE = 0.001
RESET_SPEED_TOLERANCE_RANGE_PER_S = 0.001
MAX_LIMIT_VIOLATION_RANGE = 0.005
MAX_REVOLUTE_SPEED_RAD_PER_S = math.radians(300.0)
MAX_PRISMATIC_SPEED_M_PER_S = 5.0

RuntimeErrorV2 = _core.RuntimeErrorV2
_ORIGINAL_TERMINAL_RECORD = _r2._ORIGINAL_TERMINAL_RECORD
_INSTALLED = False


def _stable_protocol() -> dict[str, Any]:
    protocol: dict[str, Any] = {
        "schema_version": PROTOCOL_SCHEMA,
        "protocol_id": PROTOCOL_ID,
        "initial_normalized_positions": list(INITIAL_FRACTIONS),
        "timestep_s": TIMESTEP_S,
        "maximum_steps": MAXIMUM_STEPS,
        "control": "zero_applied_joint_force",
        "environment": {
            "gravity_m_per_s2": [0.0, 0.0, -9.81],
            "base": "fixed",
            "contacts": "enabled",
            "self_collision": "enabled",
            "solver_iterations": 50,
            "asset_physics": "manifest_bound_released_or_injected_fields",
        },
        "eligible_joint_types": ["revolute", "prismatic"],
        "requires_nonempty_mapped_eligible_joint_set": True,
        "requires_all_trials": True,
        "requires_full_horizon": True,
        "rest_state_required": False,
        "constraint_drift": {
            "schema_version": DRIFT_SCHEMA,
            "criterion": "canonical_urdf_fk_at_measured_joint_states",
            "sample_every_steps": DRIFT_SAMPLE_EVERY_STEPS,
            "samples_per_joint_per_trial": MAXIMUM_STEPS // DRIFT_SAMPLE_EVERY_STEPS,
            "position_normalization": "object_bounding_box_diagonal",
            "rotation_error": "quaternion_geodesic_angle",
            "stable_pass_gate": False,
        },
        "thresholds": {
            "reset_position_error_over_range_max": RESET_POSITION_TOLERANCE_RANGE,
            "reset_speed_over_range_per_s_max": RESET_SPEED_TOLERANCE_RANGE_PER_S,
            "limit_violation_over_range_max": MAX_LIMIT_VIOLATION_RANGE,
            "revolute_speed_rad_per_s_max": MAX_REVOLUTE_SPEED_RAD_PER_S,
            "prismatic_speed_m_per_s_max": MAX_PRISMATIC_SPEED_M_PER_S,
        },
        "implementation": {
            "runtime_script": str(SCRIPT_PATH),
            "runtime_script_sha256": _core.sha256_file(SCRIPT_PATH),
            "runtime_core_script": str(Path(_core.__file__).resolve()),
            "runtime_core_script_sha256": _core.sha256_file(
                Path(_core.__file__).resolve()
            ),
            "orchestrator_script": str(Path(_core._runtime.__file__).resolve()),
            "orchestrator_script_sha256": _core.sha256_file(
                Path(_core._runtime.__file__).resolve()
            ),
            "compat_script": str(Path(_compat.__file__).resolve()),
            "compat_script_sha256": _core.sha256_file(Path(_compat.__file__).resolve()),
            "native_load_observer_script": str(Path(_r2.__file__).resolve()),
            "native_load_observer_script_sha256": _core.sha256_file(
                Path(_r2.__file__).resolve()
            ),
            "physics_runtime_script": str(
                Path(_core._physics_runtime.__file__).resolve()
            ),
            "physics_runtime_script_sha256": _core.sha256_file(
                Path(_core._physics_runtime.__file__).resolve()
            ),
            "physics_overlay_script": str(
                SCRIPT_PATH.with_name("table5_pva_physics.py")
            ),
            "physics_overlay_script_sha256": _core.sha256_file(
                SCRIPT_PATH.with_name("table5_pva_physics.py")
            ),
            "adapter_script": str(Path(_core._legacy.__file__).resolve()),
            "adapter_script_sha256": _core.sha256_file(
                Path(_core._legacy.__file__).resolve()
            ),
        },
    }
    protocol["protocol_sha256"] = _core._runtime.canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    return protocol


def _validate_source_protocol(protocol: Any) -> dict[str, Any]:
    if not isinstance(protocol, dict):
        raise RuntimeErrorV2("source protocol must be an object")
    if (
        protocol.get("schema_version") != _r2.PROTOCOL_SCHEMA
        or protocol.get("protocol_id") != _r2.PROTOCOL_ID
    ):
        raise RuntimeErrorV2("source protocol identity mismatch")
    for section in (
        "runtime",
        "v2_metrics",
        "cross_simulator",
        "adapters",
        "cohort_binding",
        "implementation",
    ):
        if not isinstance(protocol.get(section), Mapping):
            raise RuntimeErrorV2(f"source protocol {section} is missing")

    expected_hash = protocol.get("protocol_sha256")
    observed_hash = _core._runtime.canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    if expected_hash != observed_hash:
        raise RuntimeErrorV2("source protocol SHA256 self-check failed")

    source_runtime = protocol["runtime"]
    timestep = source_runtime.get("timestep_s")
    if not isinstance(timestep, Mapping):
        raise RuntimeErrorV2("source protocol timestep is missing")
    numerator, denominator = timestep.get("numerator"), timestep.get("denominator")
    if numerator != 1 or denominator != 240:
        raise RuntimeErrorV2("Stable v2 requires the frozen 1/240 s timestep")
    if source_runtime.get("base") != "fixed":
        raise RuntimeErrorV2("Stable v2 requires a fixed base")
    if source_runtime.get("contacts") != "enabled":
        raise RuntimeErrorV2("Stable v2 requires contacts")
    if source_runtime.get("gravity_m_per_s2") != [0, 0, -9.81]:
        raise RuntimeErrorV2("Stable v2 requires the frozen gravity vector")
    if source_runtime.get("solver_iterations") != 50:
        raise RuntimeErrorV2("Stable v2 requires 50 solver iterations")
    timeout = source_runtime.get("child_timeout_s")
    if (
        not isinstance(timeout, (int, float))
        or isinstance(timeout, bool)
        or not math.isfinite(float(timeout))
        or float(timeout) < len(INITIAL_FRACTIONS) * MAXIMUM_STEPS * TIMESTEP_S
    ):
        raise RuntimeErrorV2("source protocol child timeout is too short")

    metrics = protocol["v2_metrics"]
    if metrics.get("metric_semantics_id") != _r2.METRIC_SEMANTICS_ID:
        raise RuntimeErrorV2("source protocol metric semantics mismatch")
    adapters = protocol["adapters"]
    expected_versions = {"genesis": "1.3.1", "pybullet": "3.2.7", "mujoco": "3.10.0"}
    for simulator, version in expected_versions.items():
        adapter = adapters.get(simulator)
        if not isinstance(adapter, Mapping) or adapter.get("version") != version:
            raise RuntimeErrorV2(f"source protocol {simulator} adapter mismatch")
    for field, value in protocol["implementation"].items():
        if field.endswith("_sha256") and (
            not isinstance(value, str) or _core._runtime.SHA256.fullmatch(value) is None
        ):
            raise RuntimeErrorV2(f"source implementation receipt is malformed: {field}")
    return deepcopy(protocol)


def _identity(*args: Any, **kwargs: Any) -> dict[str, Any]:
    identity = _core._v2_identity(*args, **kwargs)
    stable_protocol = _stable_protocol()
    identity["stable_v2_revision"] = {
        "protocol_id": PROTOCOL_ID,
        "protocol_sha256": stable_protocol["protocol_sha256"],
        "entrypoint_sha256": _core.sha256_file(SCRIPT_PATH),
    }
    return identity


def _eligible_joints(
    row: Mapping[str, Any], mapped_names: Sequence[str]
) -> dict[str, dict[str, Any]]:
    mapped = set(mapped_names)
    result: dict[str, dict[str, Any]] = {}
    for joint in row.get("scalar_joints", []):
        if not isinstance(joint, Mapping):
            continue
        name = joint.get("name")
        lower, upper = joint.get("lower"), joint.get("upper")
        if (
            isinstance(name, str)
            and name in mapped
            and joint.get("type") in {"revolute", "prismatic"}
            and _core._finite(lower)
            and _core._finite(upper)
            and float(lower) < float(upper)
        ):
            result[name] = {
                "type": str(joint["type"]),
                "lower": float(lower),
                "upper": float(upper),
                "range": float(upper) - float(lower),
                "child": str(joint.get("child") or ""),
            }
    return result


def _initial_positions(
    mapped_names: Sequence[str],
    eligible: Mapping[str, Mapping[str, Any]],
    fraction: float,
) -> dict[str, float]:
    positions = {name: 0.0 for name in mapped_names}
    for name, bounds in eligible.items():
        positions[name] = float(bounds["lower"]) + fraction * float(bounds["range"])
    return positions


def _mapping_unchanged(
    adapter: Any, row: Mapping[str, Any], initial_mapping: Mapping[str, Any]
) -> bool:
    final_mapping = _core._mapping_receipt(adapter, row)
    return all(
        final_mapping.get(field) == initial_mapping.get(field)
        for field in (
            "mapped_canonical_scalar_joint_names",
            "observed_link_names",
            "observed_joint_names",
        )
    )


def _drift_accumulator(
    eligible: Mapping[str, Mapping[str, Any]],
) -> dict[str, dict[str, Any]]:
    return {
        name: {
            "joint_name": name,
            "joint_type": joint["type"],
            "child_link_name": joint["child"],
            "attempted_samples": 0,
            "position_evaluated_samples": 0,
            "rotation_evaluated_samples": 0,
            "max_position_error_over_bbox": None,
            "max_rotation_error_rad": None,
            "not_evaluable_reason_counts": {},
        }
        for name, joint in eligible.items()
    }


def _record_not_evaluable(accumulator: dict[str, Any], reason: str) -> None:
    reasons = accumulator["not_evaluable_reason_counts"]
    reasons[reason] = reasons.get(reason, 0) + 1


def _update_constraint_drift(
    row: Mapping[str, Any],
    state: Mapping[str, Any],
    poses: Mapping[str, Any],
    eligible: Mapping[str, Mapping[str, Any]],
    accumulators: Mapping[str, dict[str, Any]],
) -> None:
    bbox = row.get("bounding_box_diagonal")
    bbox_available = _core._finite(bbox) and float(bbox) > 0.0
    measured = {
        name: float(sample["q"])
        for name, sample in state.items()
        if isinstance(sample, Mapping) and _core._finite(sample.get("q"))
    }
    joint_tree = row.get("joint_tree")
    if not isinstance(joint_tree, Mapping):
        for accumulator in accumulators.values():
            accumulator["attempted_samples"] += 1
            _record_not_evaluable(accumulator, "joint_tree_unavailable")
        return

    for name, joint in eligible.items():
        accumulator = accumulators[name]
        accumulator["attempted_samples"] += 1
        child = str(joint["child"])
        if not child:
            _record_not_evaluable(accumulator, "child_link_unavailable")
            continue
        try:
            expected, errors = _core._legacy.fk_link_pose_branch(
                dict(joint_tree), measured, child
            )
            observed = poses.get(child)
            if errors or not _core._finite_pose(expected):
                _record_not_evaluable(accumulator, "canonical_fk_unavailable")
                continue
            if not _core._finite_pose(observed):
                _record_not_evaluable(accumulator, "observed_pose_unavailable")
                continue

            rotation_error = _core._legacy.quaternion_angular_error(
                expected["rotation"], observed["rotation"]
            )
            accumulator["rotation_evaluated_samples"] += 1
            accumulator["max_rotation_error_rad"] = max(
                float(accumulator["max_rotation_error_rad"] or 0.0),
                rotation_error,
            )
            if not bbox_available:
                _record_not_evaluable(accumulator, "bbox_unavailable")
                continue
            position_error = _core._position_error(
                expected["translation"], observed["translation"]
            ) / float(bbox)
            accumulator["position_evaluated_samples"] += 1
            accumulator["max_position_error_over_bbox"] = max(
                float(accumulator["max_position_error_over_bbox"] or 0.0),
                position_error,
            )
        except Exception as error:
            _record_not_evaluable(
                accumulator, f"drift_evaluation_error:{type(error).__name__}"
            )


def _constraint_drift_trial(
    accumulators: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    return {
        "schema_version": DRIFT_SCHEMA,
        "criterion": "canonical_urdf_fk_at_measured_joint_states",
        "sample_every_steps": DRIFT_SAMPLE_EVERY_STEPS,
        "joints": [deepcopy(dict(accumulators[name])) for name in sorted(accumulators)],
    }


def _combined_constraint_drift(
    trials: Sequence[Mapping[str, Any]],
    eligible: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    expected_samples = (
        len(INITIAL_FRACTIONS) * MAXIMUM_STEPS // DRIFT_SAMPLE_EVERY_STEPS
    )
    combined: list[dict[str, Any]] = []
    for name in sorted(eligible):
        rows = [
            joint
            for trial in trials
            for joint in trial.get("constraint_drift", {}).get("joints", [])
            if isinstance(joint, Mapping) and joint.get("joint_name") == name
        ]
        position_samples = sum(
            int(joint.get("position_evaluated_samples", 0)) for joint in rows
        )
        rotation_samples = sum(
            int(joint.get("rotation_evaluated_samples", 0)) for joint in rows
        )
        reasons: dict[str, int] = {}
        for joint in rows:
            for reason, count in joint.get("not_evaluable_reason_counts", {}).items():
                reasons[str(reason)] = reasons.get(str(reason), 0) + int(count)
        position_values = [
            float(joint["max_position_error_over_bbox"])
            for joint in rows
            if _core._finite(joint.get("max_position_error_over_bbox"))
        ]
        rotation_values = [
            float(joint["max_rotation_error_rad"])
            for joint in rows
            if _core._finite(joint.get("max_rotation_error_rad"))
        ]
        combined.append(
            {
                "joint_name": name,
                "joint_type": eligible[name]["type"],
                "child_link_name": eligible[name]["child"],
                "expected_samples": expected_samples,
                "position_evaluated_samples": position_samples,
                "rotation_evaluated_samples": rotation_samples,
                "position_fully_evaluated": position_samples == expected_samples,
                "rotation_fully_evaluated": rotation_samples == expected_samples,
                "max_position_error_over_bbox": (
                    max(position_values) if position_values else None
                ),
                "max_rotation_error_rad": (
                    max(rotation_values) if rotation_values else None
                ),
                "not_evaluable_reason_counts": dict(sorted(reasons.items())),
            }
        )
    return {
        "schema_version": DRIFT_SCHEMA,
        "criterion": "maximum_across_three_long_horizon_trials",
        "sample_every_steps": DRIFT_SAMPLE_EVERY_STEPS,
        "expected_samples_per_joint": expected_samples,
        "joints": combined,
    }


def _trial(
    adapter: Any,
    row: Mapping[str, Any],
    initial_mapping: Mapping[str, Any],
    eligible: Mapping[str, Mapping[str, Any]],
    fraction: float,
) -> dict[str, Any]:
    mapped_names = list(initial_mapping["mapped_canonical_scalar_joint_names"])
    observed_names = list(initial_mapping["observed_link_names"])
    target_positions = _initial_positions(mapped_names, eligible, fraction)
    trace = hashlib.sha256()
    steps_completed = 0
    max_limit_violation = 0.0
    max_revolute_speed = 0.0
    max_prismatic_speed = 0.0
    reset_position_error = 0.0
    reset_speed = 0.0
    reset_state_finite = False
    reset_poses_finite = False
    mapping_unchanged = False
    drift_accumulators = _drift_accumulator(eligible)
    error: dict[str, str] | None = None

    try:
        adapter.reset(target_positions)
        reset_state = adapter.state()
        reset_poses = adapter.link_poses()
        reset_state_finite = _r2._finite_state(reset_state, mapped_names)
        reset_poses_finite = _r2._finite_poses(reset_poses, observed_names)
        if reset_state_finite:
            reset_position_error = max(
                abs(float(reset_state[name]["q"]) - target_positions[name])
                / float(bounds["range"])
                for name, bounds in eligible.items()
            )
            reset_speed = max(
                abs(float(reset_state[name]["qdot"])) / float(bounds["range"])
                for name, bounds in eligible.items()
            )
        reset_ok = bool(
            reset_state_finite
            and reset_poses_finite
            and reset_position_error <= RESET_POSITION_TOLERANCE_RANGE
            and reset_speed <= RESET_SPEED_TOLERANCE_RANGE_PER_S
        )
        if reset_ok:
            for step in range(1, MAXIMUM_STEPS + 1):
                adapter.step({name: 0.0 for name in mapped_names})
                state = adapter.state()
                poses = adapter.link_poses()
                state_finite = _r2._finite_state(state, mapped_names)
                poses_finite = _r2._finite_poses(poses, observed_names)
                trace.update(
                    json.dumps(
                        {
                            "step": step,
                            "state": state,
                            "poses": poses,
                            "state_finite": state_finite,
                            "poses_finite": poses_finite,
                        },
                        ensure_ascii=True,
                        allow_nan=False,
                        sort_keys=True,
                        separators=(",", ":"),
                    ).encode("utf-8")
                )
                steps_completed = step
                if not state_finite or not poses_finite:
                    break

                if step % DRIFT_SAMPLE_EVERY_STEPS == 0:
                    _update_constraint_drift(
                        row, state, poses, eligible, drift_accumulators
                    )

                for name, bounds in eligible.items():
                    sample = state[name]
                    joint_range = float(bounds["range"])
                    normalized_q = (
                        float(sample["q"]) - float(bounds["lower"])
                    ) / joint_range
                    absolute_speed = abs(float(sample["qdot"]))
                    violation = max(-normalized_q, normalized_q - 1.0, 0.0)
                    max_limit_violation = max(max_limit_violation, violation)
                    if bounds["type"] == "revolute":
                        max_revolute_speed = max(max_revolute_speed, absolute_speed)
                    else:
                        max_prismatic_speed = max(max_prismatic_speed, absolute_speed)

        mapping_unchanged = _mapping_unchanged(adapter, row, initial_mapping)
    except BaseException as caught:
        error = {
            "exception_type": type(caught).__name__,
            "message": str(caught)[-_core.TAIL_LIMIT :],
        }

    checks = {
        "reset_state_finite": reset_state_finite,
        "reset_poses_finite": reset_poses_finite,
        "reset_position_within_tolerance": (
            reset_position_error <= RESET_POSITION_TOLERANCE_RANGE
        ),
        "reset_speed_within_tolerance": (
            reset_speed <= RESET_SPEED_TOLERANCE_RANGE_PER_S
        ),
        "full_horizon_finite_rollout": (
            steps_completed == MAXIMUM_STEPS and error is None
        ),
        "limit_compliant": max_limit_violation <= MAX_LIMIT_VIOLATION_RANGE,
        "joint_speed_bounded": bool(
            max_revolute_speed <= MAX_REVOLUTE_SPEED_RAD_PER_S
            and max_prismatic_speed <= MAX_PRISMATIC_SPEED_M_PER_S
        ),
        "mapping_unchanged": mapping_unchanged,
    }
    return {
        "initial_normalized_position": fraction,
        "passed": all(checks.values()),
        "checks": checks,
        "steps_completed": steps_completed,
        "simulated_seconds": steps_completed * TIMESTEP_S,
        "reset_position_error_over_range_max": reset_position_error,
        "reset_speed_over_range_per_s_max": reset_speed,
        "limit_violation_over_range_max": max_limit_violation,
        "revolute_speed_rad_per_s_max": max_revolute_speed,
        "prismatic_speed_m_per_s_max": max_prismatic_speed,
        "constraint_drift": _constraint_drift_trial(drift_accumulators),
        "trace_sha256": trace.hexdigest(),
        "error": error,
    }


def stable_v2_rollout(
    adapter: Any, row: Mapping[str, Any], initial_mapping: Mapping[str, Any]
) -> dict[str, Any]:
    protocol = _stable_protocol()
    mapped_names = list(initial_mapping["mapped_canonical_scalar_joint_names"])
    eligible = _eligible_joints(row, mapped_names)
    trials = (
        [
            _trial(adapter, row, initial_mapping, eligible, fraction)
            for fraction in INITIAL_FRACTIONS
        ]
        if eligible
        else []
    )
    checks = {
        "nonempty_mapped_eligible_joint_set": bool(eligible),
        "all_trials_completed_and_stable": bool(trials)
        and all(trial["passed"] for trial in trials),
    }
    receipt: dict[str, Any] = {
        "schema_version": RECEIPT_SCHEMA,
        "protocol_id": PROTOCOL_ID,
        "protocol_sha256": protocol["protocol_sha256"],
        "passed": all(checks.values()),
        "checks": checks,
        "mapped_eligible_joint_count": len(eligible),
        "mapped_eligible_joint_names": sorted(eligible),
        "trials": trials,
        "constraint_drift": _combined_constraint_drift(trials, eligible),
    }
    receipt["receipt_sha256"] = _core._runtime.canonical_sha256(
        receipt, exclude_fields=("receipt_sha256",)
    )
    return receipt


def _checkpoint(path: Path, response: Mapping[str, Any]) -> None:
    _core._runtime.atomic_write_json(path, response)


def worker_main(request_path: Path, response_path: Path) -> int:
    adapter: Any | None = None
    response: dict[str, Any] = {
        "metrics": _core._false_legacy_metrics(),
        "stable_v2": {
            "schema_version": EVIDENCE_SCHEMA,
            "protocol": _stable_protocol(),
            "source": None,
            "native_import": {"passed": False},
            "dof_mapping": None,
            "physics": None,
            "rollout": {"passed": False},
        },
    }
    evidence = response["stable_v2"]
    try:
        request = _core._read_json(request_path, "worker request")
        if request.get("schema_version") != _core._runtime.WORKER_REQUEST_SCHEMA:
            raise RuntimeErrorV2("worker request schema mismatch")
        simulator = str(request.get("simulator"))
        if simulator not in _core.SIMULATORS:
            raise RuntimeErrorV2("worker simulator is invalid")
        row = deepcopy(request.get("row"))
        source_protocol = deepcopy(request.get("protocol"))
        if not isinstance(row, dict) or not isinstance(source_protocol, dict):
            raise RuntimeErrorV2("worker row/protocol is malformed")
        _validate_source_protocol(source_protocol)
        source = _core._simulator_source(row, simulator)
        source_path = Path(str(request["urdf_path"])).resolve(strict=True)
        if source_path != Path(source["path"]).resolve(strict=True):
            raise RuntimeErrorV2("worker source path differs from prepared binding")
        if _core.sha256_file(source_path) != source["sha256"]:
            raise RuntimeErrorV2("worker source SHA256 differs from prepared binding")
        evidence["source"] = source

        def record_native_import(receipt: dict[str, Any]) -> None:
            evidence["native_import"] = receipt
            _checkpoint(response_path, response)

        observed: _r2._NativeLoadObserver | None = None
        with _r2._observe_native_load(
            simulator, source, record_native_import
        ) as observed:
            adapter = _core._runtime._make_adapter(
                simulator, source_path, row, source_protocol
            )
        if observed.receipt is None:
            raise RuntimeErrorV2("native asset load was not observed")

        mapping = _core._mapping_receipt(adapter, row)
        evidence["dof_mapping"] = mapping
        evidence["physics"] = _core._physics_receipt(simulator, adapter, row)
        evidence["rollout"] = stable_v2_rollout(adapter, row, mapping)
        device_receipt = getattr(adapter, "device_receipt", None)
        if device_receipt is not None:
            response["device_receipt"] = deepcopy(device_receipt)
    except BaseException as error:
        response["worker_error"] = f"{type(error).__name__}: {error}"
        response["traceback_tail"] = traceback.format_exc()[-_core.TAIL_LIMIT :]
    finally:
        if adapter is not None:
            try:
                adapter.close()
            except BaseException as error:
                response["close_error"] = f"{type(error).__name__}: {error}"
    _checkpoint(response_path, response)
    return 0


def _terminal_record(*args: Any, **kwargs: Any) -> dict[str, Any]:
    outcome = args[1] if len(args) > 1 else kwargs.get("outcome")
    record = _ORIGINAL_TERMINAL_RECORD(*args, **kwargs)
    raw = getattr(outcome, "response", None)
    stable = raw.get("stable_v2") if isinstance(raw, Mapping) else None
    if record.get("terminal_status") != "completed" and isinstance(stable, Mapping):
        record["evaluation"]["stable_v2"] = deepcopy(dict(stable))
    return record


def install() -> None:
    global _INSTALLED
    if _INSTALLED:
        return
    _compat.install()
    _core.SCRIPT_PATH = SCRIPT_PATH
    _core._validate_v2_protocol = _validate_source_protocol
    _core._runtime._validate_protocol = _validate_source_protocol
    _core.worker_main = worker_main
    _core._runtime._identity = _identity
    _core._runtime._preflight_failure = _r2._r2_preflight_failure
    _core._runtime._terminal_record = _terminal_record
    _INSTALLED = True


def main(argv: Sequence[str] | None = None) -> int:
    install()
    return _core.main(argv)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeErrorV2 as error:
        print(f"table5_stable_v2_runtime: {error}", file=sys.stderr)
        raise SystemExit(2)
