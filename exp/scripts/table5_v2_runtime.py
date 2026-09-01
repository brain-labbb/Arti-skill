#!/usr/bin/env python3
"""Run one simulator against the frozen Table 5 v2 prepared manifest.

The parent preserves one terminal record per selected asset.  This module reuses
the established adapters and dynamic trace evaluator, while adding explicit v2
import, DoF mapping, physics readback, FK-probe, and finite-rollout evidence.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import uuid
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
REPO_ROOT = SCRIPT_PATH.parents[2]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import run_table5_sketch_mobility as _legacy  # noqa: E402
import table5_n200_runtime as _runtime  # noqa: E402
import table5_pva_physics_n200_runtime as _physics_runtime  # noqa: E402
from table5_pva_physics import load_plan  # noqa: E402
from table5_v2_prepare import (  # noqa: E402
    PVA_PHYSICS_POLICY_ID,
    PROTOCOL_ID,
    PROTOCOL_SCHEMA,
    SCHEMA_VERSION as PREPARED_SCHEMA,
    sha256_file,
)


V2_EVIDENCE_SCHEMA = "table5_v2_runtime_evidence_v2"
SIMULATORS = ("genesis", "pybullet", "mujoco")
DEFAULT_EXECUTABLES = {
    "genesis": "/mnt/zsn/miniconda3/envs/genesis-main/bin/python",
    "pybullet": str(REPO_ROOT / "exp/.venv_low_medium/bin/python"),
    "mujoco": "/mnt/zsn/miniconda3/bin/python",
}
DEFAULT_WORKERS = {"genesis": 1, "pybullet": 8, "mujoco": 8}
TAIL_LIMIT = _runtime.TAIL_LIMIT
PROCESS_TERM_GRACE_S = _runtime.PROCESS_TERM_GRACE_S
_GENERIC_IDENTITY = _runtime._identity


RuntimeErrorV2 = _runtime.RuntimeContractError


def _simulator_source(row: Mapping[str, Any], simulator: str) -> dict[str, Any]:
    sources = row.get("simulator_sources")
    source = sources.get(simulator) if isinstance(sources, Mapping) else None
    if not isinstance(source, Mapping):
        source = {
            "format": "urdf",
            "representation": "released_or_prepared_canonical_urdf",
            "path": row.get("urdf_path"),
            "sha256": row.get("urdf_sha256"),
            "package_root": row.get("package_root"),
        }
    source_format = str(source.get("format") or "").lower()
    representation = str(source.get("representation") or "released")
    supported = source_format == "urdf" or (
        simulator == "mujoco"
        and source_format == "mjcf"
        and representation == "official_released_mjcf"
    )
    if not supported:
        raise RuntimeErrorV2(
            f"unsupported {simulator} source format: {source_format!r}"
        )
    source_path = Path(str(source.get("path") or "")).resolve(strict=False)
    source_hash = source.get("sha256")
    if not isinstance(source_hash, str) or len(source_hash) != 64:
        raise RuntimeErrorV2(f"{simulator} source SHA256 is malformed")
    return {
        "format": source_format,
        "representation": representation,
        "path": str(source_path),
        "sha256": source_hash,
        "package_root": str(source.get("package_root") or source_path.parent),
    }


def _v2_identity(
    bundle: _runtime.ManifestBundle,
    row: Mapping[str, Any],
    *,
    simulator: str,
    executable: str,
    timeout_s: float,
    gpu_binding: str | None,
    effective_workers: int,
) -> dict[str, Any]:
    identity = _GENERIC_IDENTITY(
        bundle,
        row,
        simulator=simulator,
        executable=executable,
        timeout_s=timeout_s,
        gpu_binding=gpu_binding,
        effective_workers=effective_workers,
    )
    source = _simulator_source(row, simulator)
    # Keep the historical urdf_* identity fields for the generic orchestrator;
    # in v2 they bind the actual engine input selected by the frozen manifest.
    identity["urdf_path"] = source["path"]
    identity["urdf_sha256"] = source["sha256"]
    identity["source_path"] = source["path"]
    identity["source_sha256"] = source["sha256"]
    identity["source_format"] = source["format"]
    identity["source_representation"] = source["representation"]
    identity["source_package_root"] = source["package_root"]
    identity["worker_source_sha256"] = sha256_file(SCRIPT_PATH)
    identity["prepared_manifest_schema"] = PREPARED_SCHEMA
    identity["prepared_cohort_sha256"] = bundle.raw.get("prepared_cohort_sha256")
    identity["source_cohort_manifest_sha256"] = bundle.raw.get(
        "source_cohort", {}
    ).get("manifest_sha256")
    identity["canonical_urdf_sha256"] = row.get("urdf_sha256")
    identity["physics_policy_id"] = row.get("physics", {}).get("policy_id")
    identity["physics_receipt_sha256"] = row.get("physics", {}).get(
        "receipt_sha256", row.get("physics", {}).get("physics_plan_sha256")
    )
    identity["collision_bbox_receipt_sha256"] = row.get("bounding_box", {}).get(
        "receipt_sha256"
    )
    return identity


def _v2_preflight_failure(row: Mapping[str, Any], source_path: Path) -> str | None:
    preflight = row["preflight"]
    if preflight["simulator_eligible"] is False:
        issues = preflight.get("issues")
        detail = ",".join(str(item) for item in issues) if isinstance(issues, list) else ""
        return f"manifest_preflight_failed{': ' + detail if detail else ''}"
    matching = []
    for simulator in SIMULATORS:
        source = _simulator_source(row, simulator)
        if Path(source["path"]).resolve(strict=False) == source_path.resolve(strict=False):
            matching.append(source)
    if not matching:
        return f"source_not_bound_for_any_simulator: {source_path}"
    if not source_path.is_file():
        return f"source_missing: {source_path}"
    try:
        observed = sha256_file(source_path)
    except OSError as error:
        return f"source_unreadable: {type(error).__name__}: {error}"
    if all(observed != source["sha256"] for source in matching):
        return f"source_hash_mismatch: observed {observed}"
    return None


# The generic orchestration looks this up dynamically and then validates exact
# identity equality on resume.  Patch only the identity constructor, not its IO,
# fail-closed terminal logic, or concurrency controls.
_runtime._identity = _v2_identity
_runtime._preflight_failure = _v2_preflight_failure


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeErrorV2(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise RuntimeErrorV2(f"{label} must be an object")
    return value


def _validate_v2_protocol(protocol: Mapping[str, Any]) -> None:
    if (
        protocol.get("schema_version") != PROTOCOL_SCHEMA
        or protocol.get("protocol_id") != PROTOCOL_ID
    ):
        raise RuntimeErrorV2("prepared v2 protocol identity mismatch")
    metrics = protocol.get("v2_metrics")
    if not isinstance(metrics, Mapping):
        raise RuntimeErrorV2("v2 metric contract is missing")
    if metrics.get("reported_percentile") != 95:
        raise RuntimeErrorV2("v2 reported percentile is not frozen at P95")
    implementation = protocol.get("implementation")
    if not isinstance(implementation, Mapping):
        raise RuntimeErrorV2("v2 implementation receipt is missing")
    bindings = {
        "v2_runtime_script_sha256": SCRIPT_PATH,
        "evaluator_script_sha256": Path(_legacy.__file__).resolve(),
        "pva_physics_script_sha256": SCRIPT_PATH.with_name(
            "table5_pva_physics.py"
        ),
        "pva_physics_runtime_script_sha256": SCRIPT_PATH.with_name(
            "table5_pva_physics_n200_runtime.py"
        ),
    }
    for field, path in bindings.items():
        if implementation.get(field) != sha256_file(path):
            raise RuntimeErrorV2(f"v2 implementation hash mismatch: {path}")


def _finite(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _finite_pose(pose: Any) -> bool:
    return bool(
        isinstance(pose, Mapping)
        and isinstance(pose.get("translation"), list)
        and len(pose["translation"]) == 3
        and all(_finite(value) for value in pose["translation"])
        and isinstance(pose.get("rotation"), list)
        and len(pose["rotation"]) == 4
        and all(_finite(value) for value in pose["rotation"])
    )


def _mapping_receipt(adapter: Any, row: Mapping[str, Any]) -> dict[str, Any]:
    declared = [
        str(joint["name"])
        for joint in row.get("scalar_joints", [])
        if isinstance(joint, Mapping) and isinstance(joint.get("name"), str)
    ]
    mapped = sorted(set(declared) & set(adapter.mapped_joint_names))
    receipt = {
        "declared_canonical_scalar_joint_names": declared,
        "mapped_canonical_scalar_joint_names": mapped,
        "unmapped_canonical_scalar_joint_names": sorted(set(declared) - set(mapped)),
        "observed_link_names": sorted(adapter.observed_link_names),
        "observed_joint_names": sorted(adapter.observed_joint_names),
        "declared_count": len(declared),
        "mapped_count": len(mapped),
    }
    receipt["receipt_sha256"] = _runtime.canonical_sha256(receipt)
    return receipt


def _eligible_joints(row: Mapping[str, Any], mapped_names: Sequence[str]) -> list[dict[str, Any]]:
    support = _legacy.classify_support(
        [dict(joint) for joint in row.get("scalar_joints", [])], mapped_names
    )
    return [dict(item) for item in support.values() if item["eligible"]]


def _first_step(adapter: Any, eligible: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    positions = {
        str(joint["name"]): (float(joint["lower"]) + float(joint["upper"])) / 2.0
        for joint in eligible
    }
    adapter.reset(positions)
    adapter.step({name: 0.0 for name in positions})
    state = adapter.state()
    poses = adapter.link_poses()
    expected_state_names = set(adapter.mapped_joint_names)
    finite_state = bool(
        isinstance(state, Mapping)
        and expected_state_names <= set(state)
        and all(
        isinstance(sample, Mapping)
        and _finite(sample.get("q"))
        and _finite(sample.get("qdot"))
        for sample in state.values()
        )
    )
    finite_poses = bool(
        isinstance(poses, Mapping)
        and set(adapter.observed_link_names) <= set(poses)
        and all(_finite_pose(pose) for pose in poses.values())
    )
    return {
        "passed": finite_state and finite_poses,
        "finite_state": finite_state,
        "finite_link_poses": finite_poses,
        "observed_state_count": len(state),
        "observed_pose_count": len(poses),
    }


def _physics_receipt(simulator: str, adapter: Any, row: Mapping[str, Any]) -> dict[str, Any]:
    physics = row.get("physics")
    if not isinstance(physics, Mapping):
        raise RuntimeErrorV2("row physics receipt is missing")
    result: dict[str, Any] = {
        "status": str(physics.get("status")),
        "policy_id": physics.get("policy_id"),
        "source_receipt_sha256": physics.get(
            "receipt_sha256", physics.get("physics_plan_sha256")
        ),
        "simulator": simulator,
        "adapter_application": None,
        "loaded_source": _simulator_source(row, simulator),
    }
    if physics.get("status") != "ready":
        # Native-track evaluation preserves released fields and lets the
        # simulator fill only fields that are absent from the released asset.
        result["status"] = "native_fallback"
        result["reason"] = physics.get("reason", "released physics field missing")
        result["fallback_policy"] = "simulator_native_missing_fields_only"
        result["receipt_sha256"] = _runtime.canonical_sha256(
            result, exclude_fields=("receipt_sha256",)
        )
        return result
    if physics.get("policy_id") == PVA_PHYSICS_POLICY_ID:
        checked_files: dict[str, str] = {}
        for label, path_field, hash_field in (
            ("source_urdf", "source_urdf_path", "source_urdf_sha256"),
            ("physics_sidecar", "physics_sidecar_path", "physics_sidecar_sha256"),
            ("physics_plan", "physics_plan_path", "physics_plan_sha256"),
            ("injected_urdf", "injected_urdf_path", "injected_urdf_sha256"),
        ):
            path = Path(str(physics[path_field])).resolve(strict=True)
            observed_hash = sha256_file(path)
            if observed_hash != physics[hash_field]:
                raise RuntimeErrorV2(f"{label} hash mismatch before physics application")
            checked_files[label] = observed_hash
        plan = load_plan(
            Path(str(physics["physics_plan_path"])).resolve(strict=True),
            source_urdf_sha256=str(physics["source_urdf_sha256"]),
            physics_sha256=str(physics["physics_sidecar_sha256"]),
            injected_urdf_sha256=str(physics["injected_urdf_sha256"]),
        )
        result["adapter_application"] = _physics_runtime._apply_physics(
            simulator, adapter, plan
        )
        result["checked_file_sha256"] = checked_files
    else:
        loaded_source = result["loaded_source"]
        result["adapter_application"] = {
            "policy": "released URDF inertials consumed by the importer",
            "loaded_source_sha256": loaded_source["sha256"],
            "canonical_urdf_sha256": row.get("urdf_sha256"),
        }
    result["receipt_sha256"] = _runtime.canonical_sha256(
        result, exclude_fields=("receipt_sha256",)
    )
    return result


def _position_error(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def fk_probe(adapter: Any, row: Mapping[str, Any], protocol: Mapping[str, Any]) -> dict[str, Any]:
    """Compare adapter poses against canonical URDF FK at direct-reset states."""

    mapping = _mapping_receipt(adapter, row)
    eligible = _eligible_joints(row, mapping["mapped_canonical_scalar_joint_names"])
    alphas = list(protocol["v2_metrics"]["fk_probe_alphas"])
    bbox = row.get("bounding_box_diagonal")
    bbox_available = _finite(bbox) and float(bbox) > 0.0
    midpoints = {
        joint["name"]: (float(joint["lower"]) + float(joint["upper"])) / 2.0
        for joint in eligible
    }
    samples: list[dict[str, Any]] = []
    for tested in eligible:
        descendants = _legacy._descendant_names(
            dict(row["joint_tree"]), str(tested["child"])
        )
        for alpha in alphas:
            commanded = dict(midpoints)
            commanded[tested["name"]] = float(tested["lower"]) + float(alpha) * (
                float(tested["upper"]) - float(tested["lower"])
            )
            try:
                adapter.reset(commanded)
                state = adapter.state()
                observed = adapter.link_poses()
                measured = {
                    name: float(sample["q"])
                    for name, sample in state.items()
                    if isinstance(sample, Mapping) and _finite(sample.get("q"))
                }
                for link_name in descendants:
                    expected, errors = _legacy.fk_link_pose_branch(
                        dict(row["joint_tree"]), measured, link_name
                    )
                    pose = observed.get(link_name)
                    if errors or not _finite_pose(expected) or not _finite_pose(pose):
                        samples.append(
                            {
                                "joint_name": tested["name"],
                                "alpha": float(alpha),
                                "link_name": link_name,
                                "status": "not_evaluable",
                                "reason": "canonical_or_observed_pose_unavailable",
                            }
                        )
                        continue
                    position_m = _position_error(
                        expected["translation"], pose["translation"]
                    )
                    samples.append(
                        {
                            "joint_name": tested["name"],
                            "alpha": float(alpha),
                            "link_name": link_name,
                            "status": "evaluated" if bbox_available else "not_evaluable",
                            "position_error_m": position_m,
                            "position_error_over_bbox": (
                                position_m / float(bbox) if bbox_available else None
                            ),
                            "rotation_error_rad": _legacy.quaternion_angular_error(
                                expected["rotation"], pose["rotation"]
                            ),
                            "reason": None if bbox_available else "bbox_unavailable",
                        }
                    )
            except Exception as error:
                for link_name in descendants:
                    samples.append(
                        {
                            "joint_name": tested["name"],
                            "alpha": float(alpha),
                            "link_name": link_name,
                            "status": "not_evaluable",
                            "reason": f"{type(error).__name__}: {error}",
                        }
                    )
    return {
        "policy": protocol["cross_simulator"]["fk_probe"],
        "eligible_joint_count": len(eligible),
        "candidate_sample_count": len(samples),
        "evaluated_sample_count": sum(
            sample["status"] == "evaluated" for sample in samples
        ),
        "samples": samples,
    }


def _false_legacy_metrics() -> dict[str, bool]:
    return _runtime._false_metrics()


def _v2_block(import_success: bool = False) -> dict[str, Any]:
    return {
        "schema_version": V2_EVIDENCE_SCHEMA,
        "import": {"passed": import_success},
        "dof_mapping": None,
        "physics": None,
        "fk_probe": None,
        "stable_rollout": {"passed": False},
    }


def _full_finite_rollout(
    evaluation: Mapping[str, Any],
    adapter: Any,
    row: Mapping[str, Any],
    protocol: Mapping[str, Any],
    initial_mapping: Mapping[str, Any],
) -> dict[str, Any]:
    """Recompute the v2 stability gate from retained finite evidence."""

    diagnostics = evaluation.get("diagnostics")
    if not isinstance(diagnostics, Mapping):
        return {"passed": False, "reason": "complete diagnostics are missing"}
    reset_rows = diagnostics.get("reset")
    settling = diagnostics.get("settling")
    actuation_rows = diagnostics.get("actuation")
    limit_rows = diagnostics.get("limit_enforcement")
    expected_steps = int(protocol["runtime"]["actuation"]["trajectory"]["ramp_steps"]) + int(
        protocol["runtime"]["actuation"]["trajectory"]["hold_steps"]
    )
    expected_limit_steps = int(protocol["runtime"]["limit_enforcement"]["steps_each"])
    eligible_names = {
        str(joint["name"])
        for joint in _eligible_joints(
            row, initial_mapping["mapped_canonical_scalar_joint_names"]
        )
    }

    reset_finite = bool(
        isinstance(reset_rows, list)
        and len(reset_rows) == int(protocol["runtime"]["reset_repetitions"])
        and all(
            isinstance(item, Mapping)
            and item.get("finite") is True
            and item.get("error") is None
            for item in reset_rows
        )
    )
    settling_finite = bool(
        isinstance(settling, Mapping)
        and settling.get("finite") is True
        and settling.get("steps_completed")
        == int(protocol["runtime"]["passive_settling"]["steps"])
        and settling.get("error") is None
    )

    actuation_by_name = {
        str(item["joint_name"]): item
        for item in actuation_rows or []
        if isinstance(item, Mapping) and isinstance(item.get("joint_name"), str)
    }
    actuation_finite = bool(
        isinstance(actuation_rows, list)
        and set(actuation_by_name) == eligible_names
        and all(
            item.get("finite") is True
            and item.get("steps_completed") == expected_steps
            and item.get("error") is None
            and isinstance(item.get("full_measured_positions_q"), list)
            and len(item["full_measured_positions_q"]) == expected_steps + 1
            and all(_finite(value) for value in item["full_measured_positions_q"])
            and isinstance(item.get("constraint_drift"), Mapping)
            and item["constraint_drift"].get("finite") is True
            and item["constraint_drift"].get("steps_compared") == expected_steps
            and not item.get("missing_descendant_link_names")
            for item in actuation_by_name.values()
        )
    )

    limits_by_name = {
        str(item["joint_name"]): item
        for item in limit_rows or []
        if isinstance(item, Mapping) and isinstance(item.get("joint_name"), str)
    }
    limits_finite = bool(
        isinstance(limit_rows, list)
        and set(limits_by_name) == eligible_names
        and all(
            isinstance(item.get("targets"), list)
            and len(item["targets"])
            == len(protocol["runtime"]["limit_enforcement"]["targets_normalized"])
            and all(
                isinstance(target, Mapping)
                and target.get("finite") is True
                and target.get("steps_completed") == expected_limit_steps
                and target.get("error") is None
                and _finite(target.get("minimum_q"))
                and _finite(target.get("maximum_q"))
                for target in item["targets"]
            )
            for item in limits_by_name.values()
        )
    )

    final_mapping = _mapping_receipt(adapter, row)
    mapping_unchanged = all(
        final_mapping.get(field) == initial_mapping.get(field)
        for field in (
            "mapped_canonical_scalar_joint_names",
            "observed_link_names",
            "observed_joint_names",
        )
    )
    final_state = adapter.state()
    final_poses = adapter.link_poses()
    final_state_finite = bool(
        isinstance(final_state, Mapping)
        and set(initial_mapping["mapped_canonical_scalar_joint_names"])
        <= set(final_state)
        and all(
            isinstance(sample, Mapping)
            and _finite(sample.get("q"))
            and _finite(sample.get("qdot"))
            for sample in final_state.values()
        )
    )
    final_poses_finite = bool(
        isinstance(final_poses, Mapping)
        and set(initial_mapping["observed_link_names"]) <= set(final_poses)
        and all(_finite_pose(pose) for pose in final_poses.values())
    )
    checks = {
        "reset_finite": reset_finite,
        "settling_finite": settling_finite,
        "actuation_finite": actuation_finite,
        "limit_trials_finite": limits_finite,
        "mapping_unchanged": mapping_unchanged,
        "final_state_finite": final_state_finite,
        "final_link_poses_finite": final_poses_finite,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "eligible_bounded_joint_count": len(eligible_names),
        "final_mapping_receipt_sha256": final_mapping["receipt_sha256"],
        "criterion": "complete retained rollout with finite states and link poses",
    }


def worker_main(request_path: Path, response_path: Path) -> int:
    adapter: Any | None = None
    response: dict[str, Any] = {"metrics": _false_legacy_metrics(), "v2": _v2_block()}
    v2 = response["v2"]
    try:
        request = _read_json(request_path, "worker request")
        if request.get("schema_version") != _runtime.WORKER_REQUEST_SCHEMA:
            raise RuntimeErrorV2("worker request schema mismatch")
        simulator = request.get("simulator")
        if simulator not in SIMULATORS:
            raise RuntimeErrorV2("worker simulator is invalid")
        row = deepcopy(request.get("row"))
        protocol = deepcopy(request.get("protocol"))
        if not isinstance(row, dict) or not isinstance(protocol, dict):
            raise RuntimeErrorV2("worker row/protocol is malformed")
        _runtime._validate_protocol(protocol)
        _validate_v2_protocol(protocol)
        source = _simulator_source(row, str(simulator))
        source_path = Path(str(request["urdf_path"])).resolve(strict=True)
        if source_path != Path(source["path"]).resolve(strict=True):
            raise RuntimeErrorV2("worker source path differs from the prepared binding")
        if sha256_file(source_path) != source["sha256"]:
            raise RuntimeErrorV2("worker source SHA256 differs from the prepared binding")
        v2["source"] = source
        adapter = _runtime._make_adapter(simulator, source_path, row, protocol)
        mapping = _mapping_receipt(adapter, row)
        v2["dof_mapping"] = mapping
        physics_receipt = _physics_receipt(simulator, adapter, row)
        v2["physics"] = physics_receipt
        eligible = _eligible_joints(
            row, mapping["mapped_canonical_scalar_joint_names"]
        )
        first_step = _first_step(adapter, eligible)
        v2["import"] = {
            "passed": bool(first_step["passed"]),
            "first_step": first_step,
            "mapping_receipt_sha256": mapping["receipt_sha256"],
        }
        if not first_step["passed"]:
            raise RuntimeErrorV2("first simulation step produced non-finite evidence")
        v2["fk_probe"] = fk_probe(adapter, row, protocol)
        try:
            evaluation = _legacy.evaluate_asset(adapter, row, protocol)
        except _legacy.DiagnosticFailure as error:
            response["diagnostic_failure"] = deepcopy(error.evidence)
        else:
            response.update(evaluation)
            v2 = response.setdefault("v2", v2)
            v2["stable_rollout"] = _full_finite_rollout(
                response, adapter, row, protocol, mapping
            )
        response["v2"] = v2
        device_receipt = getattr(adapter, "device_receipt", None)
        if device_receipt is not None:
            response["device_receipt"] = deepcopy(device_receipt)
    except _legacy.DiagnosticFailure as error:
        response["diagnostic_failure"] = deepcopy(error.evidence)
        response["metrics"] = _false_legacy_metrics()
        response["v2"] = v2
    except BaseException as error:
        response["worker_error"] = f"{type(error).__name__}: {error}"
        response["traceback_tail"] = traceback.format_exc()[-TAIL_LIMIT:]
        response["metrics"] = _false_legacy_metrics()
        response["v2"] = v2
    finally:
        if adapter is not None:
            try:
                adapter.close()
            except BaseException as error:
                response["close_error"] = f"{type(error).__name__}: {error}"
    _runtime.atomic_write_json(response_path, response)
    return 0


def _terminate_group(process: subprocess.Popen[Any]) -> None:
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=PROCESS_TERM_GRACE_S)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def _tail(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")[-TAIL_LIMIT:]
    except OSError:
        return ""


def spawn_worker_process(
    *,
    request: dict[str, Any],
    executable: str,
    timeout_s: float,
    gpu_binding: str | None,
    work_root: Path,
) -> _runtime.WorkerOutcome:
    run_id = f"{request['row']['dataset_id']}-{uuid.uuid4().hex}"
    request_path = work_root / ".worker_requests" / f"{run_id}.json"
    response_path = work_root / ".worker_responses" / f"{run_id}.json"
    stdout_path = work_root / "worker_logs" / f"{run_id}.stdout.log"
    stderr_path = work_root / "worker_logs" / f"{run_id}.stderr.log"
    _runtime.atomic_write_json(request_path, request)
    response_path.parent.mkdir(parents=True, exist_ok=True)
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    worker_executable = os.path.abspath(os.path.expanduser(executable))
    command = [
        worker_executable,
        str(SCRIPT_PATH),
        "worker",
        "--request",
        str(request_path),
        "--response",
        str(response_path),
    ]
    environment = os.environ.copy()
    caps = request["protocol"].get("runtime", {}).get("thread_caps", {})
    for key, name in {
        "omp": "OMP_NUM_THREADS",
        "mkl": "MKL_NUM_THREADS",
        "openblas": "OPENBLAS_NUM_THREADS",
        "numexpr": "NUMEXPR_NUM_THREADS",
        "veclib": "VECLIB_MAXIMUM_THREADS",
        "taichi": "TI_NUM_THREADS",
    }.items():
        value = caps.get(key)
        if isinstance(value, int) and not isinstance(value, bool) and value > 0:
            environment[name] = str(value)
    if request["simulator"] == "genesis" and gpu_binding is not None:
        environment["CUDA_VISIBLE_DEVICES"] = gpu_binding
        environment.setdefault("CUDA_DEVICE_ORDER", "PCI_BUS_ID")
    started = time.monotonic()
    try:
        with stdout_path.open("wb") as stdout_handle, stderr_path.open(
            "wb"
        ) as stderr_handle:
            process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=stdout_handle,
                stderr=stderr_handle,
                env=environment,
                start_new_session=True,
            )
            timed_out = False
            try:
                process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                timed_out = True
                _terminate_group(process)
    except Exception as error:
        return _runtime.WorkerOutcome(
            duration_s=time.monotonic() - started,
            parent_error=f"{type(error).__name__}: {error}",
            stdout_tail=_tail(stdout_path),
            stderr_tail=_tail(stderr_path),
            command=command,
        )
    response: dict[str, Any] | None = None
    response_error: str | None = None
    if response_path.is_file():
        try:
            loaded = json.loads(response_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                response = loaded
            else:
                response_error = "worker response is not an object"
        except (OSError, json.JSONDecodeError) as error:
            response_error = f"cannot parse worker response: {error}"
    else:
        response_error = "worker response is missing"
    return _runtime.WorkerOutcome(
        returncode=process.returncode,
        timed_out=timed_out,
        duration_s=time.monotonic() - started,
        response=response,
        response_error=response_error,
        stdout_tail=_tail(stdout_path),
        stderr_tail=_tail(stderr_path),
        command=command,
    )


def run(
    prepared: Path,
    output: Path,
    *,
    simulator: str,
    workers: int,
    executable: str,
    datasets: Sequence[str] | None,
    gpu_bindings: Sequence[str | None] | None,
) -> dict[str, Any]:
    if simulator not in SIMULATORS:
        raise RuntimeErrorV2(f"unsupported simulator: {simulator}")
    # Keep a virtualenv entry point intact.  Resolving its symlink to the base
    # interpreter can silently drop simulator packages (notably PyBullet).
    executable_path = Path(os.path.abspath(os.path.expanduser(executable)))
    if not executable_path.is_file():
        raise RuntimeErrorV2(f"simulator Python is unavailable: {executable_path}")
    manifest = _read_json(prepared, "prepared manifest")
    if manifest.get("schema_version") != PREPARED_SCHEMA:
        raise RuntimeErrorV2("prepared manifest schema mismatch")
    _validate_v2_protocol(manifest.get("protocol", {}))
    timeout = float(manifest["protocol"]["runtime"]["child_timeout_s"])
    return _runtime.run_manifest(
        prepared,
        output,
        datasets=datasets,
        simulators=(simulator,),
        workers={simulator: workers},
        executables={simulator: str(executable_path)},
        gpu_bindings=gpu_bindings,
        timeout_s=timeout,
        launcher=spawn_worker_process,
    )


def _csv(value: str | None) -> list[str] | None:
    if value is None:
        return None
    result = [item.strip() for item in value.split(",") if item.strip()]
    return result or None


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run_parser = commands.add_parser("run")
    run_parser.add_argument("--prepared", type=Path, required=True)
    run_parser.add_argument("--out", type=Path, required=True)
    run_parser.add_argument("--simulator", choices=SIMULATORS, required=True)
    run_parser.add_argument("--workers", type=int)
    run_parser.add_argument("--executable")
    run_parser.add_argument("--datasets")
    run_parser.add_argument("--gpus", help="comma-separated Genesis CUDA tokens")
    worker_parser = commands.add_parser("worker", help=argparse.SUPPRESS)
    worker_parser.add_argument("--request", type=Path, required=True)
    worker_parser.add_argument("--response", type=Path, required=True)
    arguments = parser.parse_args(argv)
    if arguments.command == "worker":
        return worker_main(arguments.request, arguments.response)
    simulator = arguments.simulator
    workers = arguments.workers or DEFAULT_WORKERS[simulator]
    executable = arguments.executable or DEFAULT_EXECUTABLES[simulator]
    gpus = _csv(arguments.gpus)
    summary = run(
        arguments.prepared,
        arguments.out,
        simulator=simulator,
        workers=workers,
        executable=executable,
        datasets=_csv(arguments.datasets),
        gpu_bindings=gpus,
    )
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeErrorV2 as error:
        print(f"table5_v2_runtime: {error}", file=sys.stderr)
        raise SystemExit(2)
