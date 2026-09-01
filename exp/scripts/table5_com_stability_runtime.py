#!/usr/bin/env python3
"""Evaluate Genesis center-of-mass stability with locked joints and a free root."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import traceback
from typing import Any, Mapping, Sequence

import numpy as np
from scipy.spatial import ConvexHull


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_stable_v2_1_runtime as _finite  # noqa: E402
import table5_tipover_runtime as _tipover  # noqa: E402
import table5_v2_runtime as _core  # noqa: E402
import table5_v2_runtime_compat as _compat  # noqa: E402
import table5_v2_runtime_r2 as _r2  # noqa: E402


EVIDENCE_SCHEMA = "table5_com_stability_evidence_v1"
RECEIPT_SCHEMA = "table5_com_stability_receipt_v1"
PROTOCOL_SCHEMA = "table5_com_stability_protocol_v1"
PROTOCOL_ID = "table5-genesis-com-static-stability-20260901-v2"
COLLISION_POLICY_ENV = "TABLE5_COM_COLLISION_POLICY"
RECOMPUTE_INERTIA_ENV = "TABLE5_COM_RECOMPUTE_INERTIA"
COLLISION_POLICY = os.environ.get(COLLISION_POLICY_ENV, "source_only")
RECOMPUTE_INERTIA = os.environ.get(RECOMPUTE_INERTIA_ENV, "0") == "1"
TIMESTEP_S = 1.0 / 240.0
HORIZON_STEPS = 2400
CONTACT_WINDOW_STEPS = 120
CONTACT_TILT_LIMIT_DEG = 5.0
CONTACT_Z_TOLERANCE_M = 0.01
CONTACT_DEDUPLICATION_M = 1.0e-5
MIN_SUPPORT_AREA_M2 = 1.0e-8
STATIC_MARGIN_TOLERANCE_M = 1.0e-5
FINAL_WINDOW_STEPS = 240
GRAVITY_M_PER_S2 = 9.81
SURFACE_FRICTION = 0.8
INITIAL_CLEARANCE_M = 0.002
TIP_ANGLE_DEG = 60.0
FINAL_TIP_ANGLE_DEG = 15.0
MAX_ROOT_DROP_HEIGHTS = 1.0

RuntimeErrorCom = _core.RuntimeErrorV2
_ORIGINAL_TERMINAL_RECORD = _r2._ORIGINAL_TERMINAL_RECORD


def _protocol() -> dict[str, Any]:
    protocol: dict[str, Any] = {
        "schema_version": PROTOCOL_SCHEMA,
        "protocol_id": PROTOCOL_ID,
        "classification": "diagnostic" if RECOMPUTE_INERTIA else "strict",
        "simulator": "genesis",
        "environment": {
            "timestep_s": TIMESTEP_S,
            "horizon_steps": HORIZON_STEPS,
            "horizon_seconds": HORIZON_STEPS * TIMESTEP_S,
            "gravity_m_per_s2": [0.0, 0.0, -GRAVITY_M_PER_S2],
            "surface": "infinite_z_zero_plane",
            "surface_friction": SURFACE_FRICTION,
            "asset_friction": SURFACE_FRICTION,
            "initial_clearance_m": INITIAL_CLEARANCE_M,
            "contact_solver_iterations": 50,
        },
        "configuration": {
            "root": "free",
            "root_orientation": "canonical_identity",
            "root_translation": "canonical_collision_aabb_bottom_aligned_to_plane",
            "joints": "all_revolute_continuous_prismatic_hard_locked_at_q_zero",
            "fixed_link_merge": True,
        },
        "support_region": {
            "source": "Genesis_plane_contact_positions_during_initial_contact_window",
            "contact_window_seconds": CONTACT_WINDOW_STEPS * TIMESTEP_S,
            "contact_tilt_limit_deg": CONTACT_TILT_LIMIT_DEG,
            "contact_z_tolerance_m": CONTACT_Z_TOLERANCE_M,
            "polygon": "convex_hull_of_deduplicated_contact_positions_xy",
        },
        "metrics": {
            "com_support_margin": "signed_com_to_support_polygon_boundary_distance_normalized_by_polygon_diameter",
            "com_static_stability": "com_support_margin_nonnegative_and_full_10_second_gravity_rollout_passes",
        },
        "pass_gate": {
            "maximum_transient_tilt_deg": TIP_ANGLE_DEG,
            "maximum_final_window_tilt_deg": FINAL_TIP_ANGLE_DEG,
            "maximum_root_drop_in_bbox_heights": MAX_ROOT_DROP_HEIGHTS,
            "requires_full_finite_horizon": True,
        },
        "physics": {
            "policy": "use_Genesis_finalized_link_mass_and_COM_after_import",
            "recompute_inertia": RECOMPUTE_INERTIA,
        },
        "geometry": {
            "collision_policy": COLLISION_POLICY,
            "visual_fallback": COLLISION_POLICY != "source_only",
        },
        "implementation": {
            "runtime_script": str(SCRIPT_PATH),
            "runtime_script_sha256": _core.sha256_file(SCRIPT_PATH),
            "tipover_runtime_script": str(Path(_tipover.__file__).resolve()),
            "tipover_runtime_script_sha256": _core.sha256_file(Path(_tipover.__file__).resolve()),
        },
    }
    protocol["protocol_sha256"] = _core._runtime.canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    return protocol


def _as_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=float)


def _contact_positions(entity: Any) -> np.ndarray:
    contacts = entity.get_contacts()
    positions = _as_numpy(contacts.get("position"))
    if positions.ndim == 3 and positions.shape[0] == 1:
        positions = positions[0]
    if positions.ndim == 1 and positions.size == 3:
        positions = positions.reshape(1, 3)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise RuntimeErrorCom(f"Genesis contact positions have invalid shape: {positions.shape}")
    return positions[np.isfinite(positions).all(axis=1)]


def _entity_com(entity: Any) -> tuple[np.ndarray, float]:
    positions = _as_numpy(entity.get_links_pos(ref="link_com", relative=False))
    if positions.ndim == 3 and positions.shape[0] == 1:
        positions = positions[0]
    if positions.ndim == 1 and positions.size == 3:
        positions = positions.reshape(1, 3)
    masses = _as_numpy(entity.get_links_inertial_mass()).reshape(-1)
    if positions.ndim != 2 or positions.shape[1] != 3 or len(positions) != len(masses):
        raise RuntimeErrorCom(
            f"Genesis link COM/mass shapes are invalid: {positions.shape}, {masses.shape}"
        )
    if not np.isfinite(positions).all() or not np.isfinite(masses).all() or np.any(masses <= 0.0):
        raise RuntimeErrorCom("Genesis finalized COM or mass is non-finite/non-positive")
    total_mass = float(masses.sum())
    return (positions * masses[:, None]).sum(axis=0) / total_mass, total_mass


def _deduplicate_points(points: Sequence[Sequence[float]]) -> np.ndarray:
    if not points:
        return np.empty((0, 2), dtype=float)
    values = np.asarray(points, dtype=float)
    values = values[np.isfinite(values).all(axis=1)]
    if not len(values):
        return np.empty((0, 2), dtype=float)
    quantized = np.round(values / CONTACT_DEDUPLICATION_M).astype(np.int64)
    _, indices = np.unique(quantized, axis=0, return_index=True)
    return values[np.sort(indices)]


def _support_polygon(points_xy: np.ndarray) -> tuple[np.ndarray, float]:
    if len(points_xy) < 3:
        return np.empty((0, 2), dtype=float), 0.0
    try:
        hull = ConvexHull(points_xy)
    except Exception:
        return np.empty((0, 2), dtype=float), 0.0
    polygon = points_xy[hull.vertices]
    area = float(hull.volume)
    if not math.isfinite(area) or area < MIN_SUPPORT_AREA_M2:
        return np.empty((0, 2), dtype=float), 0.0
    return polygon, area


def _point_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    edge = end - start
    denominator = float(np.dot(edge, edge))
    if denominator <= 0.0:
        return float(np.linalg.norm(point - start))
    factor = float(np.dot(point - start, edge) / denominator)
    factor = max(0.0, min(1.0, factor))
    return float(np.linalg.norm(point - (start + factor * edge)))


def signed_support_margin(com_xy: Sequence[float], polygon: Sequence[Sequence[float]]) -> float:
    point = np.asarray(com_xy, dtype=float)
    vertices = np.asarray(polygon, dtype=float)
    if point.shape != (2,) or vertices.ndim != 2 or vertices.shape[1] != 2 or len(vertices) < 3:
        raise ValueError("a 2D COM point and a non-degenerate polygon are required")
    cross_products = []
    distances = []
    for index, start in enumerate(vertices):
        end = vertices[(index + 1) % len(vertices)]
        edge = end - start
        cross_products.append(float(edge[0] * (point - start)[1] - edge[1] * (point - start)[0]))
        distances.append(_point_segment_distance(point, start, end))
    inside = all(value >= -STATIC_MARGIN_TOLERANCE_M for value in cross_products)
    distance = min(distances)
    return distance if inside else -distance


def _polygon_diameter(polygon: np.ndarray) -> float:
    if len(polygon) < 2:
        return 0.0
    differences = polygon[:, None, :] - polygon[None, :, :]
    return float(np.linalg.norm(differences, axis=-1).max())


class GenesisComRuntime(_tipover.GenesisTipOverRuntime):
    def gravity_trial_with_com(self) -> dict[str, Any]:
        self.scene.reset()
        initial_position = _tipover._values(self.entity.get_pos(relative=True))
        if len(initial_position) != 3:
            raise RuntimeErrorCom("Genesis initial root position has an invalid shape")
        trace = hashlib.sha256()
        tilt_samples: list[float] = []
        root_z_samples: list[float] = []
        contact_points: list[np.ndarray] = []
        first_contact_step: int | None = None
        first_contact_com: np.ndarray | None = None
        total_mass: float | None = None
        steps_completed = 0
        error: dict[str, str] | None = None
        try:
            for step in range(1, HORIZON_STEPS + 1):
                self.scene.step()
                quaternion = _tipover._values(self.entity.get_quat(relative=True))
                position = _tipover._values(self.entity.get_pos(relative=True))
                if len(quaternion) != 4 or len(position) != 3:
                    raise RuntimeErrorCom("Genesis root pose has an invalid shape")
                current_tilt = _tipover.tilt_deg(quaternion)
                tilt_samples.append(current_tilt)
                root_z_samples.append(position[2])
                if step <= CONTACT_WINDOW_STEPS and current_tilt <= CONTACT_TILT_LIMIT_DEG:
                    observed = _contact_positions(self.entity)
                    observed = observed[np.abs(observed[:, 2]) <= CONTACT_Z_TOLERANCE_M]
                    if len(observed):
                        if first_contact_step is None:
                            first_contact_step = step
                            first_contact_com, total_mass = _entity_com(self.entity)
                        contact_points.extend(observed[:, :2])
                if step % _tipover.SAMPLE_STRIDE == 0:
                    trace.update(
                        json.dumps(
                            {"step": step, "position": position, "quaternion": quaternion},
                            sort_keys=True,
                            separators=(",", ":"),
                            allow_nan=False,
                        ).encode("ascii")
                    )
                steps_completed = step
        except BaseException as caught:
            error = {
                "exception_type": type(caught).__name__,
                "message": str(caught)[-_core.TAIL_LIMIT :],
            }
        full_horizon = steps_completed == HORIZON_STEPS and error is None
        final_window = tilt_samples[-FINAL_WINDOW_STEPS:]
        maximum_tilt = max(tilt_samples) if tilt_samples else None
        maximum_final_tilt = max(final_window) if final_window else None
        minimum_root_z = min(root_z_samples) if root_z_samples else None
        root_drop_heights = (
            max(0.0, initial_position[2] - minimum_root_z) / self.bbox["height_m"]
            if minimum_root_z is not None
            else None
        )
        rollout_passed = bool(
            full_horizon
            and maximum_tilt is not None
            and maximum_tilt <= TIP_ANGLE_DEG
            and maximum_final_tilt is not None
            and maximum_final_tilt <= FINAL_TIP_ANGLE_DEG
            and root_drop_heights is not None
            and root_drop_heights <= MAX_ROOT_DROP_HEIGHTS
        )
        deduplicated = _deduplicate_points(contact_points)
        polygon, area = _support_polygon(deduplicated)
        margin_m: float | None = None
        normalized_margin: float | None = None
        if first_contact_com is not None and len(polygon):
            margin_m = signed_support_margin(first_contact_com[:2], polygon)
            diameter = _polygon_diameter(polygon)
            if diameter > 0.0:
                normalized_margin = margin_m / diameter
        support_available = margin_m is not None and normalized_margin is not None
        static_passed = bool(
            support_available
            and margin_m >= -STATIC_MARGIN_TOLERANCE_M
            and rollout_passed
        )
        return {
            "valid": full_horizon,
            "steps_completed": steps_completed,
            "simulated_seconds": steps_completed * TIMESTEP_S,
            "rollout_passed": rollout_passed,
            "maximum_tilt_deg": maximum_tilt,
            "maximum_final_window_tilt_deg": maximum_final_tilt,
            "minimum_root_z_m": minimum_root_z,
            "root_drop_in_bbox_heights": root_drop_heights,
            "trace_sha256": trace.hexdigest(),
            "error": error,
            "com_support": {
                "available": support_available,
                "total_mass_kg": total_mass,
                "center_of_mass_world_m": (
                    [float(value) for value in first_contact_com]
                    if first_contact_com is not None
                    else None
                ),
                "first_contact_step": first_contact_step,
                "contact_point_count": int(len(deduplicated)),
                "support_polygon_area_m2": area,
                "support_polygon_diameter_m": _polygon_diameter(polygon),
                "support_polygon_vertices_xy_m": [
                    [float(value) for value in vertex] for vertex in polygon
                ],
                "signed_margin_m": margin_m,
                "normalized_signed_margin": normalized_margin,
            },
        }


def _identity(*args: Any, **kwargs: Any) -> dict[str, Any]:
    identity = _core._v2_identity(*args, **kwargs)
    protocol = _protocol()
    identity["com_stability_revision"] = {
        "protocol_id": PROTOCOL_ID,
        "protocol_sha256": protocol["protocol_sha256"],
        "entrypoint_sha256": _core.sha256_file(SCRIPT_PATH),
    }
    return identity


def worker_main(request_path: Path, response_path: Path) -> int:
    runtime: GenesisComRuntime | None = None
    response: dict[str, Any] = {
        "metrics": _core._false_legacy_metrics(),
        "com_stability": {
            "schema_version": EVIDENCE_SCHEMA,
            "protocol": _protocol(),
            "source": None,
            "joint_lock": None,
            "rollout": None,
        },
    }
    evidence = response["com_stability"]
    try:
        request = _core._read_json(request_path, "worker request")
        if request.get("schema_version") != _core._runtime.WORKER_REQUEST_SCHEMA:
            raise RuntimeErrorCom("worker request schema mismatch")
        if request.get("simulator") != "genesis":
            raise RuntimeErrorCom("COM stability worker supports Genesis only")
        row = deepcopy(request.get("row"))
        source_protocol = deepcopy(request.get("protocol"))
        if not isinstance(row, dict) or not isinstance(source_protocol, dict):
            raise RuntimeErrorCom("worker row/protocol is malformed")
        _finite._validate_source_protocol(source_protocol)
        source = _core._simulator_source(row, "genesis")
        source_path = Path(str(request["urdf_path"])).resolve(strict=True)
        if source_path != Path(source["path"]).resolve(strict=True):
            raise RuntimeErrorCom("worker source path differs from prepared binding")
        if _core.sha256_file(source_path) != source["sha256"]:
            raise RuntimeErrorCom("worker source SHA256 differs from prepared binding")
        evidence["source"] = source
        with __import__("tempfile").TemporaryDirectory(prefix="table5_com_stability_") as directory:
            locked_path = Path(directory) / "model.locked.urdf"
            evidence["joint_lock"] = _tipover.make_locked_urdf(
                source_path,
                Path(str(source.get("package_root") or source_path.parent)),
                locked_path,
                collision_policy=COLLISION_POLICY,
            )
            _core._runtime.atomic_write_json(response_path, response)
            runtime = GenesisComRuntime(
                locked_path,
                row,
                source_protocol,
                recompute_inertia=RECOMPUTE_INERTIA,
            )
            evidence["native_import"] = {
                "passed": True,
                "operation": "Scene.add_entity+Scene.build",
                "locked_source_sha256": evidence["joint_lock"]["locked_sha256"],
            }
            evidence["rollout"] = runtime.gravity_trial_with_com()
            response["device_receipt"] = deepcopy(runtime.device_receipt)
            runtime.close()
            runtime = None
    except BaseException as error:
        response["worker_error"] = f"{type(error).__name__}: {error}"
        response["traceback_tail"] = traceback.format_exc()[-_core.TAIL_LIMIT :]
    finally:
        if runtime is not None:
            try:
                runtime.close()
            except BaseException as error:
                response["close_error"] = f"{type(error).__name__}: {error}"
    _core._runtime.atomic_write_json(response_path, response)
    return 0


def _terminal_record(*args: Any, **kwargs: Any) -> dict[str, Any]:
    outcome = args[1] if len(args) > 1 else kwargs.get("outcome")
    record = _ORIGINAL_TERMINAL_RECORD(*args, **kwargs)
    raw = getattr(outcome, "response", None)
    evidence = raw.get("com_stability") if isinstance(raw, Mapping) else None
    if record.get("terminal_status") != "completed" and isinstance(evidence, Mapping):
        record["evaluation"]["com_stability"] = deepcopy(dict(evidence))
    return record


def install() -> None:
    _compat.install()
    _core.SCRIPT_PATH = SCRIPT_PATH
    _core._runtime._validate_protocol = _finite._validate_source_protocol
    _core.worker_main = worker_main
    _core._runtime._identity = _identity
    _core._runtime._preflight_failure = _r2._r2_preflight_failure
    _core._runtime._terminal_record = _terminal_record
    _tipover.SCRIPT_PATH = SCRIPT_PATH


def _configure_mode(collision_policy: str | None, recompute_inertia: bool) -> None:
    global COLLISION_POLICY, RECOMPUTE_INERTIA
    if collision_policy is not None:
        if collision_policy not in _tipover.COLLISION_POLICIES:
            raise RuntimeErrorCom(f"unknown collision policy: {collision_policy}")
        COLLISION_POLICY = collision_policy
        os.environ[COLLISION_POLICY_ENV] = collision_policy
    RECOMPUTE_INERTIA = bool(recompute_inertia)
    os.environ[RECOMPUTE_INERTIA_ENV] = "1" if RECOMPUTE_INERTIA else "0"


def run(
    prepared: Path,
    output: Path,
    *,
    workers: int,
    executable: str,
    datasets: Sequence[str] | None,
    gpu_bindings: Sequence[str | None] | None,
) -> dict[str, Any]:
    executable_path = Path(os.path.abspath(os.path.expanduser(executable)))
    if not executable_path.is_file():
        raise RuntimeErrorCom(f"simulator Python is unavailable: {executable_path}")
    manifest = _core._read_json(prepared, "prepared manifest")
    if manifest.get("schema_version") != _core.PREPARED_SCHEMA:
        raise RuntimeErrorCom("prepared manifest schema mismatch")
    _finite._validate_source_protocol(manifest.get("protocol", {}))
    timeout = float(manifest["protocol"]["runtime"]["child_timeout_s"])
    return _core._runtime.run_manifest(
        prepared,
        output,
        datasets=datasets,
        simulators=("genesis",),
        workers={"genesis": workers},
        executables={"genesis": str(executable_path)},
        gpu_bindings=gpu_bindings,
        timeout_s=timeout,
        launcher=_core.spawn_worker_process,
    )


def _csv(value: str | None) -> list[str] | None:
    if value is None:
        return None
    result = [item.strip() for item in value.split(",") if item.strip()]
    return result or None


def main(argv: Sequence[str] | None = None) -> int:
    import argparse

    install()
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run_parser = commands.add_parser("run")
    run_parser.add_argument("--prepared", type=Path, required=True)
    run_parser.add_argument("--out", type=Path, required=True)
    run_parser.add_argument("--workers", type=int, default=1)
    run_parser.add_argument("--executable", required=True)
    run_parser.add_argument("--datasets")
    run_parser.add_argument("--gpus", required=True)
    run_parser.add_argument(
        "--collision-policy",
        choices=sorted(_tipover.COLLISION_POLICIES),
        default=None,
    )
    run_parser.add_argument("--recompute-inertia", action="store_true")
    worker_parser = commands.add_parser("worker", help=argparse.SUPPRESS)
    worker_parser.add_argument("--request", type=Path, required=True)
    worker_parser.add_argument("--response", type=Path, required=True)
    arguments = parser.parse_args(argv)
    if arguments.command == "worker":
        return worker_main(arguments.request, arguments.response)
    _configure_mode(arguments.collision_policy, arguments.recompute_inertia)
    summary = run(
        arguments.prepared,
        arguments.out,
        workers=arguments.workers,
        executable=arguments.executable,
        datasets=_csv(arguments.datasets),
        gpu_bindings=_csv(arguments.gpus),
    )
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
