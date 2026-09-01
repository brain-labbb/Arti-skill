#!/usr/bin/env python3
"""Run the revision-2 Table 5 evaluator without changing frozen v1 sources.

Revision 2 separates native import, passive numerical stability, DoF mapping,
and articulated trajectory diagnostics.  The Genesis fixed-root mapping remains
a receipt-bound adapter normalization and never changes observed link/joint sets.
"""

from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
import hashlib
import importlib
import json
import math
from numbers import Integral
from pathlib import Path
import sys
import traceback
from typing import Any, Callable, Iterator, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_v2_runtime as _core  # noqa: E402
import table5_v2_runtime_compat as _compat  # noqa: E402


PROTOCOL_SCHEMA = "table5_v2_runtime_protocol_v2"
PROTOCOL_ID = "table5-v2-readiness-portability-v2"
METRIC_SEMANTICS_ID = "table5-v2-native-import-passive-stability-r2"
V2_EVIDENCE_SCHEMA = "table5_v2_runtime_evidence_v3"
IMPORT_RECEIPT_SCHEMA = "table5_v2_native_import_receipt_v1"
STABLE_RECEIPT_SCHEMA = "table5_v2_passive_stable_rollout_v1"

RuntimeErrorV2 = _core.RuntimeErrorV2
_ORIGINAL_IDENTITY = _core._v2_identity
_ORIGINAL_TERMINAL_RECORD = _core._runtime._terminal_record
_INSTALLED = False


def _hash(path: Path) -> str:
    return _core.sha256_file(path)


def _validate_protocol(protocol: Mapping[str, Any]) -> None:
    _core._runtime._validate_protocol(protocol)
    if (
        protocol.get("schema_version") != PROTOCOL_SCHEMA
        or protocol.get("protocol_id") != PROTOCOL_ID
    ):
        raise RuntimeErrorV2("revision-2 protocol identity mismatch")
    metrics = protocol.get("v2_metrics")
    if not isinstance(metrics, Mapping):
        raise RuntimeErrorV2("revision-2 metric contract is missing")
    passive = metrics.get("passive_stable_rollout")
    if (
        metrics.get("metric_semantics_id") != METRIC_SEMANTICS_ID
        or metrics.get("reported_percentile") != 95
        or not isinstance(passive, Mapping)
        or not isinstance(passive.get("steps"), int)
        or isinstance(passive.get("steps"), bool)
        or int(passive["steps"]) <= 0
        or passive.get("control") != "zero_applied_joint_force"
    ):
        raise RuntimeErrorV2("revision-2 metric semantics are malformed")
    implementation = protocol.get("implementation")
    if not isinstance(implementation, Mapping):
        raise RuntimeErrorV2("revision-2 implementation receipt is missing")
    bindings = {
        "v2_runtime_script_sha256": SCRIPT_PATH,
        "v2_runtime_core_script_sha256": Path(_core.__file__).resolve(),
        "genesis_compat_script_sha256": Path(_compat.__file__).resolve(),
        "aggregate_script_sha256": SCRIPT_PATH.with_name("table5_v2_aggregate_r2.py"),
        "evaluator_script_sha256": Path(_core._legacy.__file__).resolve(),
        "pva_physics_script_sha256": SCRIPT_PATH.with_name("table5_pva_physics.py"),
        "pva_physics_runtime_script_sha256": SCRIPT_PATH.with_name(
            "table5_pva_physics_n200_runtime.py"
        ),
    }
    for field, path in bindings.items():
        if not path.is_file() or implementation.get(field) != _hash(path):
            raise RuntimeErrorV2(f"revision-2 implementation hash mismatch: {path}")


def _identity(*args: Any, **kwargs: Any) -> dict[str, Any]:
    identity = _ORIGINAL_IDENTITY(*args, **kwargs)
    simulator = str(kwargs.get("simulator"))
    identity["runtime_revision"] = {
        "metric_semantics_id": METRIC_SEMANTICS_ID,
        "entrypoint_sha256": _hash(SCRIPT_PATH),
        "core_runtime_sha256": _hash(Path(_core.__file__).resolve()),
        "genesis_compat_sha256": (
            _hash(Path(_compat.__file__).resolve()) if simulator == "genesis" else None
        ),
    }
    return identity


def _r2_preflight_failure(row: Mapping[str, Any], source_path: Path) -> str | None:
    """Fail before native load only when the frozen engine input cannot be read."""

    matching: list[Mapping[str, Any]] = []
    for simulator in _core.SIMULATORS:
        try:
            source = _core._simulator_source(row, simulator)
        except (OSError, RuntimeErrorV2):
            continue
        if Path(source["path"]).resolve(strict=False) == source_path.resolve(
            strict=False
        ):
            matching.append(source)
    if not matching:
        return f"source_not_bound_for_any_simulator: {source_path}"
    if not source_path.is_file():
        return f"source_missing: {source_path}"
    try:
        observed = _hash(source_path)
    except OSError as error:
        return f"source_unreadable: {type(error).__name__}: {error}"
    if all(observed != source["sha256"] for source in matching):
        return f"source_hash_mismatch: observed {observed}"
    return None


def _v2_block() -> dict[str, Any]:
    return {
        "schema_version": V2_EVIDENCE_SCHEMA,
        "metric_semantics_id": METRIC_SEMANTICS_ID,
        "source": None,
        "import": {"passed": False},
        "dof_mapping": None,
        "physics": None,
        "stable_rollout": {"passed": False},
        "fk_probe": None,
    }


def _native_import_receipt(
    simulator: str,
    source: Mapping[str, Any],
) -> dict[str, Any]:
    operation = {
        "genesis": "Scene.add_entity+Scene.build",
        "pybullet": "loadURDF",
        "mujoco": "MjModel.from_xml_path",
    }[simulator]
    receipt: dict[str, Any] = {
        "schema_version": IMPORT_RECEIPT_SCHEMA,
        "passed": True,
        "criterion": "native_simulator_asset_load",
        "simulator": simulator,
        "native_load_operation": operation,
        "source_format": source["format"],
        "source_representation": source["representation"],
        "source_sha256": source["sha256"],
    }
    receipt["receipt_sha256"] = _core._runtime.canonical_sha256(
        receipt, exclude_fields=("receipt_sha256",)
    )
    return receipt


class _NativeLoadObserver:
    """Record the native load boundary without changing arguments or results."""

    def __init__(
        self,
        simulator: str,
        source: Mapping[str, Any],
        on_success: Callable[[dict[str, Any]], None],
    ) -> None:
        self.simulator = simulator
        self.source = source
        self.on_success = on_success
        self.receipt: dict[str, Any] | None = None

    def mark_success(self) -> None:
        if self.receipt is not None:
            return
        self.receipt = _native_import_receipt(self.simulator, self.source)
        self.on_success(deepcopy(self.receipt))


@contextmanager
def _observe_native_load(
    simulator: str,
    source: Mapping[str, Any],
    on_success: Callable[[dict[str, Any]], None],
) -> Iterator[_NativeLoadObserver]:
    """Observe only the official simulator load call used by the frozen adapters."""

    observer = _NativeLoadObserver(simulator, source, on_success)
    if simulator == "pybullet":
        module = importlib.import_module("pybullet")
        original = module.loadURDF

        def observed_load_urdf(*args: Any, **kwargs: Any) -> Any:
            result = original(*args, **kwargs)
            if (
                isinstance(result, Integral)
                and not isinstance(result, bool)
                and result >= 0
            ):
                observer.mark_success()
            return result

        module.loadURDF = observed_load_urdf
        try:
            yield observer
        finally:
            if module.loadURDF is observed_load_urdf:
                module.loadURDF = original
        return

    if simulator == "mujoco":
        module = importlib.import_module("mujoco")
        original = module.MjModel

        class ObservedMjModel:
            @staticmethod
            def from_xml_path(*args: Any, **kwargs: Any) -> Any:
                result = original.from_xml_path(*args, **kwargs)
                if result is not None:
                    observer.mark_success()
                return result

        module.MjModel = ObservedMjModel
        try:
            yield observer
        finally:
            if module.MjModel is ObservedMjModel:
                module.MjModel = original
        return

    if simulator == "genesis":
        module = importlib.import_module("genesis")
        original = module.Scene

        class ObservedScene:
            def __init__(self, *args: Any, **kwargs: Any) -> None:
                object.__setattr__(self, "_scene", original(*args, **kwargs))
                object.__setattr__(self, "_entity_added", False)

            def add_entity(self, *args: Any, **kwargs: Any) -> Any:
                result = self._scene.add_entity(*args, **kwargs)
                object.__setattr__(self, "_entity_added", True)
                return result

            def build(self, *args: Any, **kwargs: Any) -> Any:
                result = self._scene.build(*args, **kwargs)
                if self._entity_added:
                    observer.mark_success()
                return result

            def __getattr__(self, name: str) -> Any:
                return getattr(self._scene, name)

            def __setattr__(self, name: str, value: Any) -> None:
                setattr(self._scene, name, value)

        module.Scene = ObservedScene
        try:
            yield observer
        finally:
            if module.Scene is ObservedScene:
                module.Scene = original
        return

    raise RuntimeErrorV2(f"unsupported native-load observer simulator: {simulator}")


def _finite_state(state: Any, mapped_names: Sequence[str]) -> bool:
    return bool(
        isinstance(state, Mapping)
        and set(mapped_names) <= set(state)
        and all(
            isinstance(sample, Mapping)
            and _core._finite(sample.get("q"))
            and _core._finite(sample.get("qdot"))
            for sample in state.values()
        )
    )


def _finite_poses(poses: Any, observed_names: Sequence[str]) -> bool:
    return bool(
        observed_names
        and isinstance(poses, Mapping)
        and set(observed_names) <= set(poses)
        and all(_core._finite_pose(pose) for pose in poses.values())
    )


def _passive_initial_positions(
    adapter: Any, row: Mapping[str, Any]
) -> dict[str, float]:
    joints = {
        str(joint["name"]): joint
        for joint in row.get("scalar_joints", [])
        if isinstance(joint, Mapping) and isinstance(joint.get("name"), str)
    }
    positions: dict[str, float] = {}
    for name in adapter.mapped_joint_names:
        joint = joints.get(name, {})
        lower, upper = joint.get("lower"), joint.get("upper")
        if (
            _core._finite(lower)
            and _core._finite(upper)
            and float(lower) < float(upper)
        ):
            positions[name] = (float(lower) + float(upper)) / 2.0
        else:
            positions[name] = 0.0
    return positions


def passive_stable_rollout(
    adapter: Any,
    row: Mapping[str, Any],
    protocol: Mapping[str, Any],
    initial_mapping: Mapping[str, Any],
) -> dict[str, Any]:
    """Run the same zero-force finite-state rollout for every imported asset."""

    required_steps = int(protocol["v2_metrics"]["passive_stable_rollout"]["steps"])
    mapped_names = list(initial_mapping["mapped_canonical_scalar_joint_names"])
    observed_names = list(initial_mapping["observed_link_names"])
    trace = hashlib.sha256()
    steps_completed = 0
    finite_state_steps = 0
    finite_pose_steps = 0
    error: dict[str, str] | None = None
    reset_completed = False
    try:
        adapter.reset(_passive_initial_positions(adapter, row))
        reset_completed = True
        for step in range(1, required_steps + 1):
            adapter.step({name: 0.0 for name in mapped_names})
            state = adapter.state()
            poses = adapter.link_poses()
            state_finite = _finite_state(state, mapped_names)
            poses_finite = _finite_poses(poses, observed_names)
            snapshot = {
                "step": step,
                "state": state,
                "poses": poses,
                "finite_state": state_finite,
                "finite_poses": poses_finite,
            }
            trace.update(
                json.dumps(
                    snapshot,
                    ensure_ascii=True,
                    allow_nan=False,
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
            )
            steps_completed = step
            finite_state_steps += int(state_finite)
            finite_pose_steps += int(poses_finite)
            if not state_finite or not poses_finite:
                break
    except BaseException as caught:
        error = {
            "exception_type": type(caught).__name__,
            "message": str(caught)[-_core.TAIL_LIMIT :],
        }

    final_mapping: Mapping[str, Any] | None = None
    try:
        final_mapping = _core._mapping_receipt(adapter, row)
    except BaseException as caught:
        if error is None:
            error = {
                "exception_type": type(caught).__name__,
                "message": str(caught)[-_core.TAIL_LIMIT :],
            }
    mapping_unchanged = bool(
        isinstance(final_mapping, Mapping)
        and all(
            final_mapping.get(field) == initial_mapping.get(field)
            for field in (
                "mapped_canonical_scalar_joint_names",
                "observed_link_names",
                "observed_joint_names",
            )
        )
    )
    checks = {
        "reset_completed": reset_completed,
        "all_steps_completed": steps_completed == required_steps,
        "all_mapped_states_finite": finite_state_steps == required_steps,
        "all_observed_link_poses_finite": finite_pose_steps == required_steps,
        "mapping_unchanged": mapping_unchanged,
    }
    receipt: dict[str, Any] = {
        "schema_version": STABLE_RECEIPT_SCHEMA,
        "passed": all(checks.values()) and error is None,
        "criterion": "fixed_step_zero_force_finite_passive_rollout",
        "control": "zero_applied_joint_force",
        "required_steps": required_steps,
        "steps_completed": steps_completed,
        "finite_state_steps": finite_state_steps,
        "finite_pose_steps": finite_pose_steps,
        "mapped_dof_count": len(mapped_names),
        "observed_link_count": len(observed_names),
        "checks": checks,
        "trace_sha256": trace.hexdigest(),
        "error": error,
    }
    receipt["receipt_sha256"] = _core._runtime.canonical_sha256(
        receipt, exclude_fields=("receipt_sha256",)
    )
    return receipt


def _checkpoint(response_path: Path, response: Mapping[str, Any]) -> None:
    _core._runtime.atomic_write_json(response_path, response)


def worker_main(request_path: Path, response_path: Path) -> int:
    adapter: Any | None = None
    response: dict[str, Any] = {
        "metrics": _core._false_legacy_metrics(),
        "v2": _v2_block(),
    }
    v2 = response["v2"]
    try:
        request = _core._read_json(request_path, "worker request")
        if request.get("schema_version") != _core._runtime.WORKER_REQUEST_SCHEMA:
            raise RuntimeErrorV2("worker request schema mismatch")
        simulator = request.get("simulator")
        if simulator not in _core.SIMULATORS:
            raise RuntimeErrorV2("worker simulator is invalid")
        row = deepcopy(request.get("row"))
        protocol = deepcopy(request.get("protocol"))
        if not isinstance(row, dict) or not isinstance(protocol, dict):
            raise RuntimeErrorV2("worker row/protocol is malformed")
        _validate_protocol(protocol)
        source = _core._simulator_source(row, str(simulator))
        source_path = Path(str(request["urdf_path"])).resolve(strict=True)
        if source_path != Path(source["path"]).resolve(strict=True):
            raise RuntimeErrorV2("worker source path differs from the prepared binding")
        if _hash(source_path) != source["sha256"]:
            raise RuntimeErrorV2(
                "worker source SHA256 differs from the prepared binding"
            )
        v2["source"] = source

        def record_native_import(receipt: dict[str, Any]) -> None:
            v2["import"] = receipt
            _checkpoint(response_path, response)

        observed: _NativeLoadObserver | None = None
        try:
            with _observe_native_load(
                str(simulator), source, record_native_import
            ) as observed:
                adapter = _core._runtime._make_adapter(
                    simulator, source_path, row, protocol
                )
        except Exception as error:
            receipt = observed.receipt if observed is not None else None
            if (
                row.get("joint_tree") is None
                and isinstance(receipt, Mapping)
                and receipt.get("passed") is True
            ):
                # Frozen canonical schema cannot represent this asset
                # (preflight already recorded invalid_joint_tree).  Keep the
                # successful native import receipt and terminate as a
                # classified canonical-mapping failure; never force a pass.
                preflight = row.get("preflight")
                response["diagnostic_failure"] = {
                    "stage": "canonical_mapping",
                    "reason": "CANONICAL_MAPPING_FAIL",
                    "message": (
                        "CANONICAL_MAPPING_FAIL: canonical joint tree is "
                        "unavailable after a successful native import"
                    ),
                    "preflight_status": (
                        preflight.get("status")
                        if isinstance(preflight, Mapping)
                        else None
                    ),
                    "preflight_issues": (
                        list(preflight.get("issues") or [])
                        if isinstance(preflight, Mapping)
                        else []
                    ),
                    "error": f"{type(error).__name__}: {error}",
                }
                response["metrics"] = _core._false_legacy_metrics()
                response["v2"] = v2
                _checkpoint(response_path, response)
                return 0
            raise
        if observed.receipt is None:
            raise RuntimeErrorV2(
                "adapter returned without an observed native asset-load success"
            )

        mapping = _core._mapping_receipt(adapter, row)
        v2["dof_mapping"] = mapping
        v2["physics"] = _core._physics_receipt(str(simulator), adapter, row)
        v2["stable_rollout"] = passive_stable_rollout(adapter, row, protocol, mapping)
        _checkpoint(response_path, response)

        v2["fk_probe"] = _core.fk_probe(adapter, row, protocol)
        try:
            evaluation = _core._legacy.evaluate_asset(adapter, row, protocol)
        except _core._legacy.DiagnosticFailure as error:
            response["diagnostic_failure"] = deepcopy(error.evidence)
        else:
            response.update(evaluation)
        response["v2"] = v2
        device_receipt = getattr(adapter, "device_receipt", None)
        if device_receipt is not None:
            response["device_receipt"] = deepcopy(device_receipt)
    except _core._legacy.DiagnosticFailure as error:
        response["diagnostic_failure"] = deepcopy(error.evidence)
        response["metrics"] = _core._false_legacy_metrics()
        response["v2"] = v2
    except BaseException as error:
        response["worker_error"] = f"{type(error).__name__}: {error}"
        response["traceback_tail"] = traceback.format_exc()[-_core.TAIL_LIMIT :]
        response["metrics"] = _core._false_legacy_metrics()
        response["v2"] = v2
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
    v2 = raw.get("v2") if isinstance(raw, Mapping) else None
    if (
        record.get("terminal_status") != "completed"
        and isinstance(v2, Mapping)
        and v2.get("schema_version") == V2_EVIDENCE_SCHEMA
    ):
        record["evaluation"]["v2"] = deepcopy(dict(v2))
    return record


def install() -> None:
    global _INSTALLED
    if _INSTALLED:
        return
    _compat.install()
    _core.SCRIPT_PATH = SCRIPT_PATH
    _core.V2_EVIDENCE_SCHEMA = V2_EVIDENCE_SCHEMA
    _core._validate_v2_protocol = _validate_protocol
    _core.worker_main = worker_main
    _core._runtime._identity = _identity
    _core._runtime._preflight_failure = _r2_preflight_failure
    _core._runtime._terminal_record = _terminal_record
    _INSTALLED = True


def main(argv: Sequence[str] | None = None) -> int:
    install()
    return _core.main(argv)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeErrorV2 as error:
        print(f"table5_v2_runtime_r2: {error}", file=sys.stderr)
        raise SystemExit(2)
