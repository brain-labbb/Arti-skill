#!/usr/bin/env python3
"""Evaluate fixed-root passive joint settling for the frozen Table 5 cohorts."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback
from typing import Any, Mapping, Sequence
from xml.etree import ElementTree


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_stable_v2_1_runtime as _finite  # noqa: E402
import table5_v2_runtime as _core  # noqa: E402
import table5_v2_runtime_compat as _compat  # noqa: E402
import table5_v2_runtime_r2 as _r2  # noqa: E402


EVIDENCE_SCHEMA = "table5_passive_stability_evidence_v1"
RECEIPT_SCHEMA = "table5_passive_stability_receipt_v1"
PROTOCOL_SCHEMA = "table5_passive_stability_protocol_v1"
PROTOCOL_ID = "table5-passive-joint-settle-exploratory-20260831"
TIMESTEP_S = 1.0 / 240.0
MAXIMUM_STEPS = 2400
SETTLE_WINDOW_STEPS = 240
TARGET_FRACTION = 0.75
CONTINUOUS_TARGET_RAD = math.pi / 2.0
MAX_NORMALIZED_SPEED_P95_PER_S = 0.01
MAX_NORMALIZED_WINDOW_SPAN = 0.01
MAX_NORMALIZED_LIMIT_VIOLATION = 0.01
MIN_ASSET_SETTLE_FRACTION = 0.80
DEFAULT_REVOLUTE_DAMPING = 0.1
DEFAULT_PRISMATIC_DAMPING = 1.0

RuntimeErrorPassive = _core.RuntimeErrorV2
_ORIGINAL_TERMINAL_RECORD = _r2._ORIGINAL_TERMINAL_RECORD
_INSTALLED = False


def _protocol() -> dict[str, Any]:
    protocol: dict[str, Any] = {
        "schema_version": PROTOCOL_SCHEMA,
        "protocol_id": PROTOCOL_ID,
        "classification": "exploratory",
        "environment": {
            "timestep_s": TIMESTEP_S,
            "maximum_steps": MAXIMUM_STEPS,
            "gravity_m_per_s2": [0.0, 0.0, -9.81],
            "base": "fixed",
            "contacts": "enabled",
            "self_collision": "enabled",
            "control": "zero_applied_joint_force",
            "asset_physics": "manifest_bound_released_or_injected_fields",
            "joint_damping": {
                "policy": "preserve_authored_else_type_default",
                "default_revolute_or_continuous": DEFAULT_REVOLUTE_DAMPING,
                "default_prismatic": DEFAULT_PRISMATIC_DAMPING,
            },
        },
        "trials": {
            "policy": "one_canonical_scalar_joint_released_per_trial",
            "other_joint_state": "urdf_neutral_zero_clamped_to_bounded_range",
            "bounded_target_fraction": TARGET_FRACTION,
            "continuous_target_rad": CONTINUOUS_TARGET_RAD,
            "requires_full_horizon": True,
        },
        "settle_gate": {
            "window_steps": SETTLE_WINDOW_STEPS,
            "window_seconds": SETTLE_WINDOW_STEPS * TIMESTEP_S,
            "maximum_normalized_speed_p95_per_s": MAX_NORMALIZED_SPEED_P95_PER_S,
            "maximum_normalized_position_span": MAX_NORMALIZED_WINDOW_SPAN,
            "maximum_normalized_limit_violation": MAX_NORMALIZED_LIMIT_VIOLATION,
            "minimum_asset_settle_fraction": MIN_ASSET_SETTLE_FRACTION,
            "invalid_trials_count_as_not_settled": True,
        },
        "endpoint": {
            "bounded_normalization": "declared_joint_range",
            "continuous_normalization": "two_pi_with_wrapped_distance",
            "cross_simulator_reduction": "median_of_three_pairwise_distances_then_p95",
        },
        "implementation": {
            "runtime_script": str(SCRIPT_PATH),
            "runtime_script_sha256": _core.sha256_file(SCRIPT_PATH),
            "prepared_runtime_script": str(Path(_core.__file__).resolve()),
            "prepared_runtime_script_sha256": _core.sha256_file(
                Path(_core.__file__).resolve()
            ),
            "compatibility_script": str(Path(_compat.__file__).resolve()),
            "compatibility_script_sha256": _core.sha256_file(
                Path(_compat.__file__).resolve()
            ),
        },
    }
    protocol["protocol_sha256"] = _core._runtime.canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    return protocol


def _validate_source_protocol(protocol: Any) -> dict[str, Any]:
    return _finite._validate_source_protocol(protocol)


def _identity(*args: Any, **kwargs: Any) -> dict[str, Any]:
    identity = _core._v2_identity(*args, **kwargs)
    protocol = _protocol()
    identity["passive_stability_revision"] = {
        "protocol_id": PROTOCOL_ID,
        "protocol_sha256": protocol["protocol_sha256"],
        "entrypoint_sha256": _core.sha256_file(SCRIPT_PATH),
    }
    return identity


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _joint_specs(
    row: Mapping[str, Any], mapped_names: Sequence[str]
) -> dict[str, dict[str, Any]]:
    mapped = set(mapped_names)
    result: dict[str, dict[str, Any]] = {}
    for joint in row.get("scalar_joints", []):
        if not isinstance(joint, Mapping):
            continue
        name = joint.get("name")
        kind = joint.get("type")
        if not isinstance(name, str) or name not in mapped:
            continue
        if kind not in {"revolute", "continuous", "prismatic"}:
            continue
        lower, upper = joint.get("lower"), joint.get("upper")
        bounded = (
            _finite_number(lower)
            and _finite_number(upper)
            and float(lower) < float(upper)
        )
        if kind == "prismatic" and not bounded:
            continue
        if bounded:
            scale = float(upper) - float(lower)
            neutral = min(max(0.0, float(lower)), float(upper))
            target = float(lower) + TARGET_FRACTION * scale
            distance_mode = "linear"
        else:
            scale = 2.0 * math.pi
            neutral = 0.0
            target = CONTINUOUS_TARGET_RAD
            distance_mode = "wrapped_angle"
        result[name] = {
            "joint_name": name,
            "joint_type": str(kind),
            "bounded": bool(bounded),
            "lower": float(lower) if bounded else None,
            "upper": float(upper) if bounded else None,
            "scale": scale,
            "neutral": neutral,
            "target": target,
            "distance_mode": distance_mode,
        }
    return result


def _authored_damping(source_path: Path) -> dict[str, float]:
    try:
        root = ElementTree.parse(source_path).getroot()
    except (OSError, ElementTree.ParseError):
        return {}
    result: dict[str, float] = {}
    for joint in root.findall(".//joint"):
        name = joint.get("name")
        dynamics = joint.find("dynamics")
        value = dynamics.get("damping") if dynamics is not None else None
        if not isinstance(name, str) or value is None:
            continue
        try:
            damping = float(value)
        except ValueError:
            continue
        if math.isfinite(damping) and damping >= 0.0:
            result[name] = damping
    return result


def _apply_damping(
    simulator: str,
    adapter: Any,
    row: Mapping[str, Any],
    mapping: Mapping[str, Any],
    source_path: Path,
) -> dict[str, Any]:
    authored = _authored_damping(source_path)
    kinds = {
        str(joint["name"]): str(joint["type"])
        for joint in row.get("scalar_joints", [])
        if isinstance(joint, Mapping)
        and isinstance(joint.get("name"), str)
        and joint.get("type") in {"revolute", "continuous", "prismatic"}
    }
    entries: list[dict[str, Any]] = []
    values: dict[str, float] = {}
    for name in mapping["mapped_canonical_scalar_joint_names"]:
        kind = kinds.get(name)
        if kind not in {"revolute", "continuous", "prismatic"}:
            continue
        source = "authored" if name in authored else "type_default"
        value = authored.get(
            name,
            DEFAULT_PRISMATIC_DAMPING
            if kind == "prismatic"
            else DEFAULT_REVOLUTE_DAMPING,
        )
        values[name] = value
        entries.append(
            {
                "joint_name": name,
                "joint_type": kind,
                "damping": value,
                "source": source,
            }
        )
    if simulator == "pybullet":
        for name, value in values.items():
            adapter.p.changeDynamics(
                adapter.body,
                adapter.joint_indices[name],
                jointDamping=value,
                physicsClientId=adapter.client,
            )
    elif simulator == "mujoco":
        for name, value in values.items():
            _qpos, dof = adapter.mapping[name]
            adapter.model.dof_damping[dof] = value
    elif simulator == "genesis":
        if values:
            names = sorted(values)
            indices = [adapter.mapping[name] for name in names]
            adapter.entity.set_dofs_damping([values[name] for name in names], indices)
    else:
        raise RuntimeErrorPassive(f"unsupported damping adapter: {simulator}")
    receipt: dict[str, Any] = {
        "policy": "preserve_authored_else_type_default",
        "joint_count": len(entries),
        "authored_count": sum(entry["source"] == "authored" for entry in entries),
        "defaulted_count": sum(entry["source"] == "type_default" for entry in entries),
        "joints": entries,
    }
    receipt["receipt_sha256"] = _core._runtime.canonical_sha256(
        receipt, exclude_fields=("receipt_sha256",)
    )
    return receipt


def trial_specs(
    row: Mapping[str, Any], mapped_names: Sequence[str] | None = None
) -> list[dict[str, Any]]:
    names = (
        list(mapped_names)
        if mapped_names is not None
        else [
            str(joint["name"])
            for joint in row.get("scalar_joints", [])
            if isinstance(joint, Mapping) and isinstance(joint.get("name"), str)
        ]
    )
    specs = _joint_specs(row, names)
    if not specs:
        return [
            {
                "trial_id": "neutral",
                "target_joint_name": None,
                "target_joint_type": None,
                "target_scale": None,
                "target_distance_mode": None,
            }
        ]
    return [
        {
            "trial_id": f"joint:{name}",
            "target_joint_name": name,
            "target_joint_type": specs[name]["joint_type"],
            "target_scale": specs[name]["scale"],
            "target_distance_mode": specs[name]["distance_mode"],
        }
        for name in sorted(specs)
    ]


def normalized_distance(left: float, right: float, spec: Mapping[str, Any]) -> float:
    difference = abs(float(left) - float(right))
    if spec.get("distance_mode") == "wrapped_angle":
        difference = abs((difference + math.pi) % (2.0 * math.pi) - math.pi)
    return difference / float(spec["scale"])


def _percentile(values: Sequence[float], percentile: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return math.inf
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile / 100.0
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _window_joint_assessment(
    samples: Sequence[Mapping[str, Mapping[str, float]]],
    spec: Mapping[str, Any],
) -> dict[str, Any]:
    name = str(spec["joint_name"])
    positions = [float(sample[name]["q"]) for sample in samples]
    speeds = [abs(float(sample[name]["qdot"])) for sample in samples]
    scale = float(spec["scale"])
    if spec.get("distance_mode") == "wrapped_angle":
        unwrapped = [positions[0]]
        for position in positions[1:]:
            delta = (position - unwrapped[-1] + math.pi) % (2.0 * math.pi) - math.pi
            unwrapped.append(unwrapped[-1] + delta)
        positions = unwrapped
    normalized_speed_max = max(speeds, default=math.inf) / scale
    normalized_speed_p95 = _percentile(speeds, 95.0) / scale
    normalized_span = (
        (max(positions) - min(positions)) / scale if positions else math.inf
    )
    limit_violation = 0.0
    if spec.get("bounded"):
        lower, upper = float(spec["lower"]), float(spec["upper"])
        limit_violation = max(
            max((lower - position) / scale for position in positions),
            max((position - upper) / scale for position in positions),
            0.0,
        )
    passed = bool(
        normalized_speed_p95 <= MAX_NORMALIZED_SPEED_P95_PER_S
        and normalized_span <= MAX_NORMALIZED_WINDOW_SPAN
    )
    return {
        "joint_name": name,
        "joint_type": spec["joint_type"],
        "scale": scale,
        "distance_mode": spec["distance_mode"],
        "maximum_normalized_speed_per_s": normalized_speed_max,
        "normalized_speed_p95_per_s": normalized_speed_p95,
        "normalized_window_position_span": normalized_span,
        "maximum_normalized_limit_violation": limit_violation,
        "passed": passed,
    }


def _initial_positions(
    specs: Mapping[str, Mapping[str, Any]], target_name: str | None
) -> dict[str, float]:
    positions = {name: float(spec["neutral"]) for name, spec in specs.items()}
    if target_name is not None:
        positions[target_name] = float(specs[target_name]["target"])
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


def _trial(
    adapter: Any,
    row: Mapping[str, Any],
    mapping: Mapping[str, Any],
    specs: Mapping[str, Mapping[str, Any]],
    trial_spec: Mapping[str, Any],
) -> dict[str, Any]:
    mapped_names = list(mapping["mapped_canonical_scalar_joint_names"])
    observed_names = list(mapping["observed_link_names"])
    target_name = trial_spec.get("target_joint_name")
    positions = _initial_positions(specs, str(target_name) if target_name else None)
    samples: deque[dict[str, dict[str, float]]] = deque(
        maxlen=SETTLE_WINDOW_STEPS
    )
    steps_completed = 0
    reset_finite = False
    reset_accurate = False
    mapping_unchanged = False
    error: dict[str, str] | None = None
    trace = hashlib.sha256()
    try:
        adapter.reset(positions)
        state = adapter.state()
        poses = adapter.link_poses()
        reset_finite = bool(
            _r2._finite_state(state, mapped_names)
            and _r2._finite_poses(poses, observed_names)
        )
        reset_accurate = reset_finite and all(
            normalized_distance(float(state[name]["q"]), value, specs[name]) <= 0.001
            for name, value in positions.items()
        )
        if reset_finite and reset_accurate:
            for step in range(1, MAXIMUM_STEPS + 1):
                adapter.step({name: 0.0 for name in mapped_names})
                state = adapter.state()
                poses = adapter.link_poses()
                if not _r2._finite_state(state, mapped_names) or not _r2._finite_poses(
                    poses, observed_names
                ):
                    break
                normalized_state = {
                    name: {
                        "q": float(state[name]["q"]),
                        "qdot": float(state[name]["qdot"]),
                    }
                    for name in mapped_names
                }
                samples.append(normalized_state)
                if step % 12 == 0:
                    trace.update(
                        json.dumps(
                            {"step": step, "state": normalized_state},
                            ensure_ascii=True,
                            allow_nan=False,
                            sort_keys=True,
                            separators=(",", ":"),
                        ).encode("utf-8")
                    )
                steps_completed = step
        mapping_unchanged = _mapping_unchanged(adapter, row, mapping)
    except BaseException as caught:
        error = {
            "exception_type": type(caught).__name__,
            "message": str(caught)[-_core.TAIL_LIMIT :],
        }

    full_horizon = steps_completed == MAXIMUM_STEPS and error is None
    complete_window = len(samples) == SETTLE_WINDOW_STEPS
    assessments = (
        [_window_joint_assessment(list(samples), specs[name]) for name in sorted(specs)]
        if complete_window
        else []
    )
    valid = bool(
        reset_finite
        and reset_accurate
        and full_horizon
        and complete_window
        and mapping_unchanged
    )
    settled = bool(valid and assessments and all(item["passed"] for item in assessments))
    final_state = deepcopy(samples[-1]) if samples else None
    target_endpoint = None
    if target_name is not None and isinstance(final_state, Mapping):
        target = specs[str(target_name)]
        target_endpoint = {
            "joint_name": str(target_name),
            "joint_type": target["joint_type"],
            "q": float(final_state[str(target_name)]["q"]),
            "qdot": float(final_state[str(target_name)]["qdot"]),
            "scale": float(target["scale"]),
            "distance_mode": target["distance_mode"],
        }
    return {
        **deepcopy(dict(trial_spec)),
        "valid": valid,
        "settled": settled,
        "checks": {
            "reset_state_and_poses_finite": reset_finite,
            "reset_position_within_normalized_tolerance": reset_accurate,
            "full_horizon_finite": full_horizon,
            "complete_settle_window": complete_window,
            "mapping_unchanged": mapping_unchanged,
            "all_joints_settled": bool(
                assessments and all(item["passed"] for item in assessments)
            ),
        },
        "steps_completed": steps_completed,
        "simulated_seconds": steps_completed * TIMESTEP_S,
        "joint_assessments": assessments,
        "final_state": final_state,
        "target_endpoint": target_endpoint,
        "trace_sha256": trace.hexdigest(),
        "error": error,
    }


def passive_stability_rollout(
    adapter: Any, row: Mapping[str, Any], mapping: Mapping[str, Any]
) -> dict[str, Any]:
    protocol = _protocol()
    mapped_names = list(mapping["mapped_canonical_scalar_joint_names"])
    specs = _joint_specs(row, mapped_names)
    planned = trial_specs(row, mapped_names)
    trials = [
        _trial(adapter, row, mapping, specs, trial_spec) for trial_spec in planned
    ]
    settled_count = sum(trial["settled"] is True for trial in trials)
    valid_count = sum(trial["valid"] is True for trial in trials)
    planned_count = len(planned)
    settle_fraction = settled_count / planned_count if planned_count else 0.0
    valid_fraction = valid_count / planned_count if planned_count else 0.0
    complete_mapping = not mapping.get("unmapped_canonical_scalar_joint_names")
    receipt: dict[str, Any] = {
        "schema_version": RECEIPT_SCHEMA,
        "protocol_id": PROTOCOL_ID,
        "protocol_sha256": protocol["protocol_sha256"],
        "asset_passed": bool(
            complete_mapping
            and specs
            and planned_count > 0
            and settle_fraction >= MIN_ASSET_SETTLE_FRACTION
        ),
        "complete_mapping": complete_mapping,
        "planned_trial_count": planned_count,
        "valid_trial_count": valid_count,
        "settled_trial_count": settled_count,
        "valid_trial_fraction": valid_fraction,
        "settle_fraction": settle_fraction,
        "mapped_joint_count": len(mapped_names),
        "eligible_joint_count": len(specs),
        "planned_trial_ids": [str(item["trial_id"]) for item in planned],
        "trials": trials,
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
        "passive_stability": {
            "schema_version": EVIDENCE_SCHEMA,
            "protocol": _protocol(),
            "source": None,
            "native_import": {"passed": False},
            "dof_mapping": None,
            "physics": None,
            "damping": None,
            "rollout": {"asset_passed": False},
        },
    }
    evidence = response["passive_stability"]
    try:
        request = _core._read_json(request_path, "worker request")
        if request.get("schema_version") != _core._runtime.WORKER_REQUEST_SCHEMA:
            raise RuntimeErrorPassive("worker request schema mismatch")
        simulator = str(request.get("simulator"))
        if simulator not in _core.SIMULATORS:
            raise RuntimeErrorPassive("worker simulator is invalid")
        row = deepcopy(request.get("row"))
        source_protocol = deepcopy(request.get("protocol"))
        if not isinstance(row, dict) or not isinstance(source_protocol, dict):
            raise RuntimeErrorPassive("worker row/protocol is malformed")
        _validate_source_protocol(source_protocol)
        source = _core._simulator_source(row, simulator)
        source_path = Path(str(request["urdf_path"])).resolve(strict=True)
        if source_path != Path(source["path"]).resolve(strict=True):
            raise RuntimeErrorPassive(
                "worker source path differs from prepared binding"
            )
        if _core.sha256_file(source_path) != source["sha256"]:
            raise RuntimeErrorPassive("worker source SHA256 differs from prepared binding")
        evidence["source"] = source

        def record_native_import(receipt: dict[str, Any]) -> None:
            evidence["native_import"] = receipt
            _checkpoint(response_path, response)

        with _r2._observe_native_load(simulator, source, record_native_import) as observed:
            adapter = _core._runtime._make_adapter(
                simulator, source_path, row, source_protocol
            )
        if observed.receipt is None:
            raise RuntimeErrorPassive("native asset load was not observed")
        mapping = _core._mapping_receipt(adapter, row)
        evidence["dof_mapping"] = mapping
        evidence["physics"] = _core._physics_receipt(simulator, adapter, row)
        evidence["damping"] = _apply_damping(
            simulator, adapter, row, mapping, source_path
        )
        evidence["rollout"] = passive_stability_rollout(adapter, row, mapping)
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
    evidence = raw.get("passive_stability") if isinstance(raw, Mapping) else None
    if record.get("terminal_status") != "completed" and isinstance(evidence, Mapping):
        record["evaluation"]["passive_stability"] = deepcopy(dict(evidence))
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
    except RuntimeErrorPassive as error:
        print(f"table5_passive_stability_runtime: {error}", file=sys.stderr)
        raise SystemExit(2)
