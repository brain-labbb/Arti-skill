#!/usr/bin/env python3
"""Run the PV-A N=200 Table 5 protocol with audited physics.json injection."""

from __future__ import annotations

import argparse
import copy
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
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_n200_runtime as _runtime  # noqa: E402
import run_table5_sketch_mobility as _legacy  # noqa: E402
from table5_pva_physics import (  # noqa: E402
    PLAN_SCHEMA,
    POLICY_ID,
    SIDECAR_SCHEMA,
    PhysicsInjectionError,
    load_plan,
    load_sidecar,
    sha256_file,
)


PROTOCOL_SCHEMA = "table5_pva_physics_protocol_v2"
PROTOCOL_ID = "table5-pva-prefix-physics-injection-v2"
RECEIPT_SCHEMA = "table5_pva_physics_runtime_receipt_v1"

RuntimeContractError = _runtime.RuntimeContractError
WorkerOutcome = _runtime.WorkerOutcome
SIMULATORS = _runtime.SIMULATORS
WORKER_REQUEST_SCHEMA = _runtime.WORKER_REQUEST_SCHEMA
DEFAULT_TIMEOUT_S = _runtime.DEFAULT_TIMEOUT_S
TAIL_LIMIT = _runtime.TAIL_LIMIT

_GENERIC_IDENTITY = _runtime._identity
_GENERIC_PREFLIGHT = _runtime._preflight_failure


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise RuntimeContractError(f"{label} must be an object")
    return value


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or _runtime.SHA256.fullmatch(value) is None:
        raise RuntimeContractError(f"{label} must be a SHA-256 digest")
    return value


def _validate_protocol(protocol: Any) -> dict[str, Any]:
    normalized = _runtime._validate_protocol(protocol)
    if normalized.get("schema_version") != PROTOCOL_SCHEMA:
        raise RuntimeContractError("physics protocol schema mismatch")
    if normalized.get("protocol_id") != PROTOCOL_ID:
        raise RuntimeContractError("physics protocol id mismatch")
    injection = _mapping(normalized.get("physics_injection"), "physics_injection")
    expected = {
        "required_sidecar_schema": SIDECAR_SCHEMA,
        "policy_id": POLICY_ID,
        "derived_plan_schema": PLAN_SCHEMA,
    }
    for field, value in expected.items():
        if injection.get(field) != value:
            raise RuntimeContractError(f"physics protocol {field} mismatch")
    implementation = _mapping(normalized.get("implementation"), "implementation")
    sources = {
        SCRIPT_PATH: implementation.get("physics_runtime_script_sha256"),
        SCRIPT_PATH.with_name("table5_pva_physics.py"): implementation.get(
            "physics_overlay_script_sha256"
        ),
    }
    for path, expected_hash in sources.items():
        if _sha(expected_hash, str(path)) != sha256_file(path):
            raise RuntimeContractError(f"physics implementation hash mismatch: {path}")
    return normalized


def _validate_physics_row(row: Mapping[str, Any]) -> None:
    source = _mapping(row.get("source_urdf"), "row.source_urdf")
    sidecar = _mapping(row.get("physics_sidecar"), "row.physics_sidecar")
    injection = _mapping(row.get("physics_injection"), "row.physics_injection")
    for value, label in (
        (source.get("sha256"), "source_urdf.sha256"),
        (sidecar.get("sha256"), "physics_sidecar.sha256"),
        (sidecar.get("model_urdf_sha256"), "physics_sidecar.model_urdf_sha256"),
        (injection.get("plan_sha256"), "physics_injection.plan_sha256"),
        (injection.get("injected_urdf_sha256"), "physics_injection.injected_urdf_sha256"),
    ):
        _sha(value, label)
    for value, label in (
        (source.get("path"), "source_urdf.path"),
        (sidecar.get("path"), "physics_sidecar.path"),
        (injection.get("plan_path"), "physics_injection.plan_path"),
        (injection.get("injected_urdf_path"), "physics_injection.injected_urdf_path"),
    ):
        if not isinstance(value, str) or not value:
            raise RuntimeContractError(f"{label} must be a path")
    if source["sha256"] != sidecar["model_urdf_sha256"]:
        raise RuntimeContractError("physics sidecar/source row binding mismatch")
    if sidecar.get("schema_version") != SIDECAR_SCHEMA:
        raise RuntimeContractError("physics sidecar row schema mismatch")
    if injection.get("policy_id") != POLICY_ID:
        raise RuntimeContractError("physics injection row policy mismatch")
    if injection.get("plan_schema_version") != PLAN_SCHEMA:
        raise RuntimeContractError("physics injection row plan schema mismatch")
    if row.get("urdf_sha256") != injection["injected_urdf_sha256"]:
        raise RuntimeContractError("runtime URDF is not the injected URDF")


def _physics_identity(
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
    source = row["source_urdf"]
    sidecar = row["physics_sidecar"]
    injection = row["physics_injection"]
    identity.update(
        {
            "worker_source_sha256": sha256_file(SCRIPT_PATH),
            "source_urdf_path": source["path"],
            "source_urdf_sha256": source["sha256"],
            "physics_path": sidecar["path"],
            "physics_sha256": sidecar["sha256"],
            "physics_plan_path": injection["plan_path"],
            "physics_plan_sha256": injection["plan_sha256"],
            "physics_policy_id": injection["policy_id"],
        }
    )
    return identity


def _checked_file(path_value: Any, hash_value: Any, label: str) -> Path:
    path = Path(str(path_value)).resolve()
    if not path.is_file():
        raise RuntimeContractError(f"{label}_missing: {path}")
    observed = sha256_file(path)
    if observed != hash_value:
        raise RuntimeContractError(
            f"{label}_hash_mismatch: expected {hash_value}, observed {observed}"
        )
    return path


def _physics_preflight(row: Mapping[str, Any], urdf_path: Path) -> str | None:
    generic = _GENERIC_PREFLIGHT(row, urdf_path)
    if generic is not None:
        return generic
    try:
        _validate_physics_row(row)
        source = row["source_urdf"]
        sidecar = row["physics_sidecar"]
        injection = row["physics_injection"]
        source_path = _checked_file(source["path"], source["sha256"], "source_urdf")
        physics_path = _checked_file(
            sidecar["path"], sidecar["sha256"], "physics_sidecar"
        )
        plan_path = _checked_file(
            injection["plan_path"], injection["plan_sha256"], "physics_plan"
        )
        load_sidecar(physics_path, source_urdf_sha256=source["sha256"])
        load_plan(
            plan_path,
            source_urdf_sha256=source["sha256"],
            physics_sha256=sidecar["sha256"],
            injected_urdf_sha256=row["urdf_sha256"],
        )
        if source_path == urdf_path:
            raise RuntimeContractError("source and injected URDF paths must differ")
    except (OSError, RuntimeContractError, PhysicsInjectionError) as error:
        return f"physics_preflight_failed: {type(error).__name__}: {error}"
    return None


def _friction_by_link(plan: Mapping[str, Any]) -> dict[str, float]:
    result: dict[str, float] = {}
    for row in plan["links"]:
        name = row.get("link_name")
        value = row.get("dynamic_friction_coefficient")
        if value is None and row.get("collision_count") == 0:
            continue
        if (
            not isinstance(name, str)
            or not name
            or name in result
            or isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) < 0.0
        ):
            raise RuntimeContractError("physics plan contains invalid link friction")
        result[name] = float(value)
    return result


def _receipt(
    *,
    simulator: str,
    plan: Mapping[str, Any],
    requested: Mapping[str, float],
    observed: Mapping[str, float],
    application_granularity: str,
    applied_collision_geom_count: int | None = None,
) -> dict[str, Any]:
    if set(requested) != set(observed):
        raise RuntimeContractError("physics friction receipt has incomplete coverage")
    maximum_error = max(
        (abs(requested[name] - observed[name]) for name in requested), default=0.0
    )
    receipt = {
        "schema_version": RECEIPT_SCHEMA,
        "policy_id": plan["policy_id"],
        "simulator": simulator,
        "field": "dynamic_friction_coefficient",
        "application_granularity": application_granularity,
        "link_count": len(requested),
        "requested_link_values_sha256": _runtime.canonical_sha256(dict(requested)),
        "observed_link_values_sha256": _runtime.canonical_sha256(dict(observed)),
        "maximum_readback_absolute_error": maximum_error,
        "readback_tolerance": 1.0e-12,
        "readback_pass": maximum_error <= 1.0e-12,
        "unsupported_fields_not_applied": [
            "youngs_modulus_pa",
            "poissons_ratio",
            "static_friction_coefficient",
            "restitution_coefficient",
            "joint_damping",
            "joint_friction",
        ],
    }
    if applied_collision_geom_count is not None:
        receipt["applied_collision_geom_count"] = applied_collision_geom_count
    if not receipt["readback_pass"]:
        raise RuntimeContractError(
            f"{simulator} friction readback mismatch: {maximum_error}"
        )
    return receipt


def _apply_pybullet(adapter: Any, plan: Mapping[str, Any]) -> dict[str, Any]:
    requested = _friction_by_link(plan)
    indices = {adapter.root_name: -1}
    count = adapter.p.getNumJoints(adapter.body, physicsClientId=adapter.client)
    for index in range(count):
        info = adapter.p.getJointInfo(adapter.body, index, physicsClientId=adapter.client)
        indices[info[12].decode("utf-8", errors="strict")] = index
    if not set(requested) <= set(indices):
        raise RuntimeContractError(
            "PyBullet link set differs from the physics injection plan"
        )
    observed: dict[str, float] = {}
    for name in sorted(requested):
        adapter.p.changeDynamics(
            adapter.body,
            indices[name],
            lateralFriction=requested[name],
            physicsClientId=adapter.client,
        )
        dynamics = adapter.p.getDynamicsInfo(
            adapter.body, indices[name], physicsClientId=adapter.client
        )
        observed[name] = float(dynamics[1])
    return _receipt(
        simulator="pybullet",
        plan=plan,
        requested=requested,
        observed=observed,
        application_granularity="link_changeDynamics_lateralFriction",
    )


def _apply_mujoco(adapter: Any, plan: Mapping[str, Any]) -> dict[str, Any]:
    requested = _friction_by_link(plan)
    observed_by_link: dict[str, float] = {}
    geom_count = 0
    plan_by_link = {row["link_name"]: row for row in plan["links"]}
    for link_name in sorted(requested):
        geom_values: list[float] = []
        for collision in plan_by_link[link_name]["collisions"]:
            geom_name = collision["injected_geom_name"]
            geom_id = adapter.mujoco.mj_name2id(
                adapter.model, adapter.mujoco.mjtObj.mjOBJ_GEOM, geom_name
            )
            if geom_id < 0:
                raise RuntimeContractError(f"MuJoCo collision geom is missing: {geom_name}")
            adapter.model.geom_friction[geom_id, 0] = requested[link_name]
            geom_values.append(float(adapter.model.geom_friction[geom_id, 0]))
            geom_count += 1
        if not geom_values:
            raise RuntimeContractError(f"MuJoCo link has no collision geom: {link_name}")
        if max(geom_values) - min(geom_values) > 1.0e-12:
            raise RuntimeContractError(f"MuJoCo link friction is inconsistent: {link_name}")
        observed_by_link[link_name] = geom_values[0]
    return _receipt(
        simulator="mujoco",
        plan=plan,
        requested=requested,
        observed=observed_by_link,
        application_granularity="collision_geom_sliding_component_from_link_value",
        applied_collision_geom_count=geom_count,
    )


def _apply_genesis(adapter: Any, plan: Mapping[str, Any]) -> dict[str, Any]:
    requested = _friction_by_link(plan)
    links = adapter.links
    if not set(requested) <= set(links):
        raise RuntimeContractError("Genesis link set differs from the physics injection plan")
    observed: dict[str, float] = {}
    for name in sorted(requested):
        link = links[name]
        link.set_friction(requested[name])
        geoms = list(link.geoms)
        if not geoms:
            raise RuntimeContractError(f"Genesis link has no collision geom: {name}")
        values = [float(geom.friction) for geom in geoms]
        if max(values) - min(values) > 1.0e-12:
            raise RuntimeContractError(f"Genesis link friction is inconsistent: {name}")
        observed[name] = values[0]
    return _receipt(
        simulator="genesis",
        plan=plan,
        requested=requested,
        observed=observed,
        application_granularity="rigid_link_set_friction",
    )


def _apply_physics(simulator: str, adapter: Any, plan: Mapping[str, Any]) -> dict[str, Any]:
    function = {
        "pybullet": _apply_pybullet,
        "mujoco": _apply_mujoco,
        "genesis": _apply_genesis,
    }.get(simulator)
    if function is None:
        raise RuntimeContractError(f"unsupported simulator: {simulator}")
    return function(adapter, plan)


def _make_adapter(
    simulator: str,
    raw_urdf_path: Path,
    row: dict[str, Any],
    protocol: dict[str, Any],
) -> Any:
    return _runtime._make_adapter(simulator, raw_urdf_path, row, protocol)


def worker_main(request_path: Path, response_path: Path) -> int:
    adapter: Any | None = None
    response: dict[str, Any]
    try:
        request = _runtime._read_json(request_path, "worker request")
        if request.get("schema_version") != WORKER_REQUEST_SCHEMA:
            raise RuntimeContractError("worker request schema mismatch")
        simulator = request.get("simulator")
        if simulator not in SIMULATORS:
            raise RuntimeContractError("worker request simulator is invalid")
        row = copy.deepcopy(request["row"])
        _validate_physics_row(row)
        protocol = _validate_protocol(request["protocol"])
        raw_urdf_path = Path(request["urdf_path"]).resolve(strict=True)
        injection = row["physics_injection"]
        plan = load_plan(
            Path(injection["plan_path"]).resolve(strict=True),
            source_urdf_sha256=row["source_urdf"]["sha256"],
            physics_sha256=row["physics_sidecar"]["sha256"],
            injected_urdf_sha256=row["urdf_sha256"],
        )
        missing_bbox = not _runtime._finite_positive(row.get("bounding_box_diagonal"))
        original_bbox = row.get("bounding_box_diagonal")
        if missing_bbox:
            row["bounding_box_diagonal"] = 1.0
        adapter = _make_adapter(simulator, raw_urdf_path, row, protocol)
        injection_receipt = _apply_physics(simulator, adapter, plan)
        injection_receipt["physics_plan_sha256"] = injection["plan_sha256"]
        response = _runtime.evaluate_asset(adapter, row, protocol)
        response["physics_injection_receipt"] = injection_receipt
        if missing_bbox:
            response["metrics"]["constraint_drift"] = False
            response["metrics"]["simulator_pass"] = False
            response["missing_bbox_normalizer"] = True
            response.setdefault("diagnostics", {})["missing_bbox_normalizer"] = {
                "reason": "missing_bbox_normalizer",
                "source_value": original_bbox,
                "substitute_used_only_for_evaluator_execution": 1.0,
            }
        device_receipt = getattr(adapter, "device_receipt", None)
        if device_receipt is not None:
            response["device_receipt"] = copy.deepcopy(device_receipt)
    except _legacy.DiagnosticFailure as error:
        response = {
            "diagnostic_failure": copy.deepcopy(error.evidence),
            "metrics": _runtime._false_metrics(),
        }
    except BaseException as error:
        response = {
            "worker_error": f"{type(error).__name__}: {error}",
            "traceback_tail": traceback.format_exc()[-TAIL_LIMIT:],
        }
    finally:
        if adapter is not None:
            try:
                adapter.close()
            except BaseException as error:
                response.setdefault("close_error", f"{type(error).__name__}: {error}")
    _runtime.atomic_write_json(response_path, response)
    return 0


def spawn_worker_process(
    *,
    request: dict[str, Any],
    executable: str,
    timeout_s: float,
    gpu_binding: str | None,
    work_root: Path,
) -> WorkerOutcome:
    run_id = f"{request['row']['dataset_id']}-{uuid.uuid4().hex}"
    request_path = work_root / ".worker_requests" / f"{run_id}.json"
    response_path = work_root / ".worker_responses" / f"{run_id}.json"
    stdout_path = work_root / "worker_logs" / f"{run_id}.stdout.log"
    stderr_path = work_root / "worker_logs" / f"{run_id}.stderr.log"
    _runtime.atomic_write_json(request_path, request)
    response_path.parent.mkdir(parents=True, exist_ok=True)
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        _runtime._resolve_executable(executable),
        str(SCRIPT_PATH),
        "worker",
        "--request",
        str(request_path),
        "--response",
        str(response_path),
    ]
    environment = os.environ.copy()
    thread_caps = request["protocol"].get("runtime", {}).get("thread_caps", {})
    thread_names = {
        "omp": "OMP_NUM_THREADS",
        "mkl": "MKL_NUM_THREADS",
        "openblas": "OPENBLAS_NUM_THREADS",
        "numexpr": "NUMEXPR_NUM_THREADS",
        "veclib": "VECLIB_MAXIMUM_THREADS",
        "taichi": "TI_NUM_THREADS",
    }
    for key, env_name in thread_names.items():
        value = thread_caps.get(key)
        if isinstance(value, int) and not isinstance(value, bool) and value > 0:
            environment[env_name] = str(value)
    if request["simulator"] == "genesis":
        if gpu_binding is not None:
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
                _runtime._terminate_group(process)
    except Exception as error:
        return WorkerOutcome(
            duration_s=time.monotonic() - started,
            parent_error=f"{type(error).__name__}: {error}",
            stdout_tail=_runtime._tail(stdout_path),
            stderr_tail=_runtime._tail(stderr_path),
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
                response_error = "worker response is not a JSON object"
        except (OSError, json.JSONDecodeError) as error:
            response_error = f"cannot parse worker response: {error}"
    else:
        response_error = "worker response file is missing"
    return WorkerOutcome(
        returncode=process.returncode,
        timed_out=timed_out,
        duration_s=time.monotonic() - started,
        response=response,
        response_error=response_error,
        stdout_tail=_runtime._tail(stdout_path),
        stderr_tail=_runtime._tail(stderr_path),
        command=command,
    )


def run_manifest(
    manifest: Path | str,
    runtime_root: Path | str,
    *,
    datasets: Sequence[str] | None = None,
    simulators: Sequence[str] = SIMULATORS,
    workers: Mapping[str, int] | None = None,
    executables: Mapping[str, str] | None = None,
    gpu_bindings: Sequence[str | None] | None = None,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    launcher: Any = spawn_worker_process,
) -> dict[str, Any]:
    bundle = _runtime.load_manifest(manifest)
    if bundle.raw.get("schema_version") != "table5_pva_physics_n200_manifest_v2":
        raise RuntimeContractError("physics manifest schema mismatch")
    _validate_protocol(bundle.protocol)
    for dataset in bundle.datasets:
        for row in dataset.rows:
            _validate_physics_row(row)
    previous_identity = _runtime._identity
    previous_preflight = _runtime._preflight_failure
    _runtime._identity = _physics_identity
    _runtime._preflight_failure = _physics_preflight
    try:
        return _runtime.run_manifest(
            manifest,
            runtime_root,
            datasets=datasets,
            simulators=simulators,
            workers=workers,
            executables=executables,
            gpu_bindings=gpu_bindings,
            timeout_s=timeout_s,
            launcher=launcher,
        )
    finally:
        _runtime._identity = previous_identity
        _runtime._preflight_failure = previous_preflight


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True, metavar="{run}")
    run = commands.add_parser("run", help="run selected physics-bound intents")
    run.add_argument("--manifest", type=Path, required=True)
    run.add_argument("--runtime-root", "--out", dest="runtime_root", type=Path, required=True)
    run.add_argument("--datasets", help="comma-separated dataset slugs; default: all")
    run.add_argument("--simulators", default=",".join(SIMULATORS))
    run.add_argument("--workers", action="append")
    run.add_argument("--executables", action="append")
    run.add_argument("--gpus", action="append")
    run.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    worker = commands.add_parser("worker", help=argparse.SUPPRESS)
    worker.add_argument("--request", type=Path, required=True, help=argparse.SUPPRESS)
    worker.add_argument("--response", type=Path, required=True, help=argparse.SUPPRESS)
    commands._choices_actions = [
        action for action in commands._choices_actions if action.dest != "worker"
    ]
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "worker":
        return worker_main(args.request, args.response)
    selected_datasets = _runtime._parse_csv(args.datasets)
    selected_simulators = _runtime._parse_csv(args.simulators)
    if selected_simulators is None:
        raise RuntimeContractError("--simulators is empty")
    workers = _runtime._parse_assignments(args.workers, default=1, converter=int)
    if any(
        not isinstance(value, int) or isinstance(value, bool) or value < 1
        for value in workers.values()
    ):
        raise RuntimeContractError("worker counts must be positive integers")
    executables = _runtime._parse_assignments(
        args.executables, default=sys.executable, converter=str
    )
    summary = run_manifest(
        args.manifest,
        args.runtime_root,
        datasets=selected_datasets,
        simulators=selected_simulators,
        workers=workers,
        executables=executables,
        gpu_bindings=_runtime._gpu_bindings(args.gpus),
        timeout_s=args.timeout,
    )
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeContractError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
