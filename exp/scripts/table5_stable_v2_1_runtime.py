#!/usr/bin/env python3
"""Evaluate neutral-state long-horizon numerical validity for Table 5 cohorts."""

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

import table5_stable_v2_runtime as _strict  # noqa: E402


_core = _strict._core
_compat = _strict._compat
_r2 = _strict._r2

EVIDENCE_SCHEMA = "table5_stable_v2_1_evidence_v1"
RECEIPT_SCHEMA = "table5_stable_v2_1_receipt_v1"
DRIFT_SCHEMA = _strict.DRIFT_SCHEMA
PROTOCOL_SCHEMA = "table5_stable_v2_1_protocol_v1"
PROTOCOL_ID = "table5-finite-rollout-v2.1-neutral-long-horizon-20260830"
RESET_REPETITIONS = 3
TIMESTEP_S = 1.0 / 240.0
MAXIMUM_STEPS = 2400
DRIFT_SAMPLE_EVERY_STEPS = 12
RESET_POSITION_TOLERANCE_RANGE = 0.001
RESET_SPEED_TOLERANCE_RANGE_PER_S = 0.001
RESET_ABSOLUTE_TOLERANCE = 1.0e-5
STRICT_LIMIT_DIAGNOSTIC_RANGE = 0.005
STRICT_REVOLUTE_SPEED_DIAGNOSTIC_RAD_PER_S = math.radians(300.0)
STRICT_PRISMATIC_SPEED_DIAGNOSTIC_M_PER_S = 5.0

RuntimeErrorV2 = _core.RuntimeErrorV2
_ORIGINAL_TERMINAL_RECORD = _strict._ORIGINAL_TERMINAL_RECORD
_INSTALLED = False


def _stable_protocol() -> dict[str, Any]:
    strict_implementation = _strict._stable_protocol()["implementation"]
    protocol: dict[str, Any] = {
        "schema_version": PROTOCOL_SCHEMA,
        "protocol_id": PROTOCOL_ID,
        "initial_state": {
            "policy": "urdf_neutral_zero_clamped_to_declared_bounded_range",
            "repetitions": RESET_REPETITIONS,
            "cross_joint_fractional_combinations": False,
            "rationale": "functional joint dependencies are outside numerical stability",
        },
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
        "finite_rollout_gate": {
            "requires_all_repetitions": True,
            "requires_full_horizon": True,
            "requires_finite_mapped_joint_states": True,
            "requires_finite_observed_link_poses": True,
            "requires_unchanged_mapping": True,
            "requires_nonempty_bounded_joint_set": False,
            "limit_and_speed_thresholds": "diagnostic_only",
        },
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
            "reset_unbounded_absolute_error_max": RESET_ABSOLUTE_TOLERANCE,
        },
        "supplementary_strict_diagnostics": {
            "limit_violation_over_range_max": STRICT_LIMIT_DIAGNOSTIC_RANGE,
            "revolute_speed_rad_per_s_max": (
                STRICT_REVOLUTE_SPEED_DIAGNOSTIC_RAD_PER_S
            ),
            "prismatic_speed_m_per_s_max": (STRICT_PRISMATIC_SPEED_DIAGNOSTIC_M_PER_S),
            "stable_pass_gate": False,
        },
        "implementation": {
            **strict_implementation,
            "runtime_script": str(SCRIPT_PATH),
            "runtime_script_sha256": _core.sha256_file(SCRIPT_PATH),
            "strict_helper_script": str(Path(_strict.__file__).resolve()),
            "strict_helper_script_sha256": _core.sha256_file(
                Path(_strict.__file__).resolve()
            ),
        },
    }
    protocol["protocol_sha256"] = _core._runtime.canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    return protocol


def _validate_source_protocol(protocol: Any) -> dict[str, Any]:
    return _strict._validate_source_protocol(protocol)


def _identity(*args: Any, **kwargs: Any) -> dict[str, Any]:
    identity = _core._v2_identity(*args, **kwargs)
    stable_protocol = _stable_protocol()
    identity["finite_rollout_v2_1_revision"] = {
        "protocol_id": PROTOCOL_ID,
        "protocol_sha256": stable_protocol["protocol_sha256"],
        "entrypoint_sha256": _core.sha256_file(SCRIPT_PATH),
    }
    return identity


def _mapped_joint_specs(
    row: Mapping[str, Any], mapped_names: Sequence[str]
) -> dict[str, dict[str, Any]]:
    mapped = set(mapped_names)
    result: dict[str, dict[str, Any]] = {}
    for joint in row.get("scalar_joints", []):
        if not isinstance(joint, Mapping):
            continue
        name = joint.get("name")
        if not isinstance(name, str) or name not in mapped:
            continue
        lower, upper = joint.get("lower"), joint.get("upper")
        bounded = bool(
            _core._finite(lower)
            and _core._finite(upper)
            and float(lower) < float(upper)
        )
        result[name] = {
            "type": str(joint.get("type") or ""),
            "bounded": bounded,
            "lower": float(lower) if bounded else None,
            "upper": float(upper) if bounded else None,
            "range": float(upper) - float(lower) if bounded else None,
        }
    return result


def _neutral_positions(specs: Mapping[str, Mapping[str, Any]]) -> dict[str, float]:
    positions: dict[str, float] = {}
    for name, spec in specs.items():
        if spec["bounded"]:
            positions[name] = min(max(0.0, float(spec["lower"])), float(spec["upper"]))
        else:
            positions[name] = 0.0
    return positions


def _reset_errors(
    state: Mapping[str, Mapping[str, Any]],
    specs: Mapping[str, Mapping[str, Any]],
    targets: Mapping[str, float],
) -> tuple[float, float, float, float]:
    bounded_position = 0.0
    bounded_speed = 0.0
    unbounded_position = 0.0
    unbounded_speed = 0.0
    for name, spec in specs.items():
        sample = state[name]
        position_error = abs(float(sample["q"]) - targets[name])
        speed = abs(float(sample["qdot"]))
        if spec["bounded"]:
            joint_range = float(spec["range"])
            bounded_position = max(bounded_position, position_error / joint_range)
            bounded_speed = max(bounded_speed, speed / joint_range)
        else:
            unbounded_position = max(unbounded_position, position_error)
            unbounded_speed = max(unbounded_speed, speed)
    return bounded_position, bounded_speed, unbounded_position, unbounded_speed


def _combined_constraint_drift(
    trials: Sequence[Mapping[str, Any]],
    eligible: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    expected_samples = RESET_REPETITIONS * MAXIMUM_STEPS // DRIFT_SAMPLE_EVERY_STEPS
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
        "criterion": "maximum_across_three_neutral_long_horizon_trials",
        "sample_every_steps": DRIFT_SAMPLE_EVERY_STEPS,
        "expected_samples_per_joint": expected_samples,
        "joints": combined,
    }


def _trial(
    adapter: Any,
    row: Mapping[str, Any],
    initial_mapping: Mapping[str, Any],
    specs: Mapping[str, Mapping[str, Any]],
    eligible: Mapping[str, Mapping[str, Any]],
    repetition: int,
) -> dict[str, Any]:
    mapped_names = list(initial_mapping["mapped_canonical_scalar_joint_names"])
    observed_names = list(initial_mapping["observed_link_names"])
    target_positions = _neutral_positions(specs)
    trace = hashlib.sha256()
    steps_completed = 0
    max_limit_violation = 0.0
    max_revolute_speed = 0.0
    max_prismatic_speed = 0.0
    reset_position_error = 0.0
    reset_speed = 0.0
    reset_unbounded_position_error = 0.0
    reset_unbounded_speed = 0.0
    reset_state_finite = False
    reset_poses_finite = False
    mapping_unchanged = False
    drift_accumulators = _strict._drift_accumulator(eligible)
    error: dict[str, str] | None = None

    try:
        adapter.reset(target_positions)
        reset_state = adapter.state()
        reset_poses = adapter.link_poses()
        reset_state_finite = _r2._finite_state(reset_state, mapped_names)
        reset_poses_finite = _r2._finite_poses(reset_poses, observed_names)
        if reset_state_finite:
            (
                reset_position_error,
                reset_speed,
                reset_unbounded_position_error,
                reset_unbounded_speed,
            ) = _reset_errors(reset_state, specs, target_positions)
        reset_ok = bool(
            reset_state_finite
            and reset_poses_finite
            and reset_position_error <= RESET_POSITION_TOLERANCE_RANGE
            and reset_speed <= RESET_SPEED_TOLERANCE_RANGE_PER_S
            and reset_unbounded_position_error <= RESET_ABSOLUTE_TOLERANCE
            and reset_unbounded_speed <= RESET_ABSOLUTE_TOLERANCE
        )
        if reset_ok:
            for step in range(1, MAXIMUM_STEPS + 1):
                adapter.step({name: 0.0 for name in mapped_names})
                state = adapter.state()
                poses = adapter.link_poses()
                state_finite = _r2._finite_state(state, mapped_names)
                poses_finite = _r2._finite_poses(poses, observed_names)
                if not state_finite or not poses_finite:
                    trace.update(f"nonfinite:{step}".encode("ascii"))
                    break
                trace.update(
                    json.dumps(
                        {"step": step, "state": state, "poses": poses},
                        ensure_ascii=True,
                        allow_nan=False,
                        sort_keys=True,
                        separators=(",", ":"),
                    ).encode("utf-8")
                )
                steps_completed = step

                if step % DRIFT_SAMPLE_EVERY_STEPS == 0:
                    _strict._update_constraint_drift(
                        row, state, poses, eligible, drift_accumulators
                    )

                for name, spec in specs.items():
                    sample = state[name]
                    absolute_speed = abs(float(sample["qdot"]))
                    if spec["bounded"]:
                        normalized_q = (
                            float(sample["q"]) - float(spec["lower"])
                        ) / float(spec["range"])
                        max_limit_violation = max(
                            max_limit_violation,
                            -normalized_q,
                            normalized_q - 1.0,
                            0.0,
                        )
                    if spec["type"] in {"revolute", "continuous"}:
                        max_revolute_speed = max(max_revolute_speed, absolute_speed)
                    elif spec["type"] == "prismatic":
                        max_prismatic_speed = max(max_prismatic_speed, absolute_speed)

        mapping_unchanged = _strict._mapping_unchanged(adapter, row, initial_mapping)
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
            and reset_unbounded_position_error <= RESET_ABSOLUTE_TOLERANCE
        ),
        "reset_speed_within_tolerance": (
            reset_speed <= RESET_SPEED_TOLERANCE_RANGE_PER_S
            and reset_unbounded_speed <= RESET_ABSOLUTE_TOLERANCE
        ),
        "full_horizon_finite_rollout": (
            steps_completed == MAXIMUM_STEPS and error is None
        ),
        "mapping_unchanged": mapping_unchanged,
    }
    diagnostics = {
        "strict_limit_compliant": (
            max_limit_violation <= STRICT_LIMIT_DIAGNOSTIC_RANGE
        ),
        "strict_joint_speed_bounded": bool(
            max_revolute_speed <= STRICT_REVOLUTE_SPEED_DIAGNOSTIC_RAD_PER_S
            and max_prismatic_speed <= STRICT_PRISMATIC_SPEED_DIAGNOSTIC_M_PER_S
        ),
    }
    return {
        "repetition": repetition,
        "initial_state_policy": "urdf_neutral_zero_clamped_to_bounded_range",
        "passed": all(checks.values()),
        "checks": checks,
        "supplementary_strict_diagnostics": diagnostics,
        "steps_completed": steps_completed,
        "simulated_seconds": steps_completed * TIMESTEP_S,
        "reset_position_error_over_range_max": reset_position_error,
        "reset_speed_over_range_per_s_max": reset_speed,
        "reset_unbounded_position_error_max": reset_unbounded_position_error,
        "reset_unbounded_speed_max": reset_unbounded_speed,
        "limit_violation_over_range_max": max_limit_violation,
        "revolute_speed_rad_per_s_max": max_revolute_speed,
        "prismatic_speed_m_per_s_max": max_prismatic_speed,
        "constraint_drift": _strict._constraint_drift_trial(drift_accumulators),
        "trace_sha256": trace.hexdigest(),
        "error": error,
    }


def stable_v2_1_rollout(
    adapter: Any, row: Mapping[str, Any], initial_mapping: Mapping[str, Any]
) -> dict[str, Any]:
    protocol = _stable_protocol()
    mapped_names = list(initial_mapping["mapped_canonical_scalar_joint_names"])
    specs = _mapped_joint_specs(row, mapped_names)
    eligible = _strict._eligible_joints(row, mapped_names)
    trials = [
        _trial(adapter, row, initial_mapping, specs, eligible, repetition)
        for repetition in range(1, RESET_REPETITIONS + 1)
    ]
    checks = {
        "all_neutral_trials_completed_and_finite": bool(trials)
        and all(trial["passed"] for trial in trials)
    }
    receipt: dict[str, Any] = {
        "schema_version": RECEIPT_SCHEMA,
        "protocol_id": PROTOCOL_ID,
        "protocol_sha256": protocol["protocol_sha256"],
        "passed": all(checks.values()),
        "checks": checks,
        "mapped_joint_count": len(specs),
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
        evidence["rollout"] = stable_v2_1_rollout(adapter, row, mapping)
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
        print(f"table5_stable_v2_1_runtime: {error}", file=sys.stderr)
        raise SystemExit(2)
