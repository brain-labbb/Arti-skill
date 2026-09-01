#!/usr/bin/env python3
"""Evaluate Genesis tip-over stability with a free root and locked joints."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import traceback
from typing import Any, Mapping, Sequence
from urllib.parse import unquote, urlparse
from xml.etree import ElementTree


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_stable_v2_1_runtime as _finite  # noqa: E402
import table5_v2_runtime as _core  # noqa: E402
import table5_v2_runtime_compat as _compat  # noqa: E402
import table5_v2_runtime_r2 as _r2  # noqa: E402


EVIDENCE_SCHEMA = "table5_tipover_stability_evidence_v1"
RECEIPT_SCHEMA = "table5_tipover_stability_receipt_v1"
PROTOCOL_SCHEMA = "table5_tipover_stability_protocol_v1"
PROTOCOL_ID = "table5-genesis-tipover-locked-joints-exploratory-20260831"
TIMESTEP_S = 1.0 / 240.0
HORIZON_STEPS = 2400
PERTURBATION_STEP = 480
SAMPLE_STRIDE = 4
FINAL_WINDOW_STEPS = 240
GRAVITY_M_PER_S2 = 9.81
SURFACE_FRICTION = 0.8
INITIAL_CLEARANCE_M = 0.002
NORMALIZED_SPEED_KICK = 0.05
NORMALIZED_ANGULAR_SPEED_KICK = 0.20
TIP_ANGLE_DEG = 60.0
FINAL_TIP_ANGLE_DEG = 15.0
MAX_ROOT_DROP_HEIGHTS = 1.0
PERTURBATION_DIRECTIONS = {
    "+x": (1.0, 0.0),
    "-x": (-1.0, 0.0),
    "+y": (0.0, 1.0),
    "-y": (0.0, -1.0),
}

RuntimeErrorTipOver = _core.RuntimeErrorV2
_ORIGINAL_TERMINAL_RECORD = _r2._ORIGINAL_TERMINAL_RECORD
_INSTALLED = False


def _protocol() -> dict[str, Any]:
    protocol: dict[str, Any] = {
        "schema_version": PROTOCOL_SCHEMA,
        "protocol_id": PROTOCOL_ID,
        "classification": "exploratory",
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
            "self_collision": "disabled_after_hard_joint_lock",
        },
        "configuration": {
            "root": "free",
            "root_orientation": "canonical_identity",
            "root_translation": "canonical_collision_aabb_bottom_aligned_to_plane",
            "joints": "all_revolute_continuous_prismatic_hard_locked_at_q_zero",
            "fixed_link_merge": True,
            "support_semantics": "canonical_bottom_aligned_stress_test",
        },
        "trials": {
            "gravity_only": ["gravity"],
            "perturbed": list(PERTURBATION_DIRECTIONS),
            "perturbation_step": PERTURBATION_STEP,
            "perturbation_time_s": PERTURBATION_STEP * TIMESTEP_S,
            "perturbation": {
                "kind": "instantaneous_high_push_equivalent_root_velocity_increment",
                "linear_normalization": "delta_v / sqrt(g * canonical_bbox_height)",
                "normalized_linear_magnitude": NORMALIZED_SPEED_KICK,
                "angular_normalization": "delta_omega * sqrt(canonical_bbox_height / g)",
                "normalized_angular_magnitude": NORMALIZED_ANGULAR_SPEED_KICK,
            },
        },
        "pass_gate": {
            "maximum_transient_tilt_deg": TIP_ANGLE_DEG,
            "maximum_final_window_tilt_deg": FINAL_TIP_ANGLE_DEG,
            "maximum_root_drop_in_bbox_heights": MAX_ROOT_DROP_HEIGHTS,
            "requires_full_finite_horizon": True,
            "tip_over_stability": "gravity_trial_passes",
            "perturbed_tip_over_stability": "gravity_and_all_four_perturbation_trials_pass",
        },
        "physics": {
            "policy": "preserve_released_or_prepared_inertials_else_genesis_native_missing_field_fallback",
            "recompute_inertia": False,
        },
        "implementation": {
            "runtime_script": str(SCRIPT_PATH),
            "runtime_script_sha256": _core.sha256_file(SCRIPT_PATH),
            "prepared_runtime_script": str(Path(_core.__file__).resolve()),
            "prepared_runtime_script_sha256": _core.sha256_file(Path(_core.__file__).resolve()),
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
    identity["tipover_stability_revision"] = {
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


def _values(value: Any) -> list[float]:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    if hasattr(value, "tolist"):
        value = value.tolist()
    while isinstance(value, list) and len(value) == 1 and isinstance(value[0], list):
        value = value[0]
    if not isinstance(value, (list, tuple)):
        raise RuntimeErrorTipOver("Genesis state is not an array")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise RuntimeErrorTipOver("Genesis state contains a non-finite value")
    return result


def _mesh_path(filename: str, source_path: Path, package_root: Path) -> Path:
    if filename.startswith("file://"):
        parsed = urlparse(filename)
        return Path(unquote(parsed.path)).resolve()
    if filename.startswith("package://"):
        relative = filename[len("package://") :]
        parts = Path(relative).parts
        candidates = [package_root / relative]
        if parts and package_root.name == parts[0]:
            candidates.insert(0, package_root.joinpath(*parts[1:]))
        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()
        return candidates[0].resolve()
    path = Path(filename)
    return path.resolve() if path.is_absolute() else (source_path.parent / path).resolve()


COLLISION_POLICIES = {
    "source_only",
    "per_link_visual_fallback",
    "robust_visual_collision",
}
DEGENERATE_MESH_MIN_THICKNESS_M = 1.0e-4


def _rpy_matrix(rpy: Sequence[float]) -> Any:
    import numpy as np

    roll, pitch, yaw = (float(value) for value in rpy)
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ],
        dtype=float,
    )


def _mesh_vertices(path: Path, scale: Sequence[float]) -> np.ndarray:
    import numpy as np
    import trimesh

    loaded = trimesh.load(path, force="scene", process=False)
    if not isinstance(loaded, trimesh.Scene):
        loaded = trimesh.Scene(loaded)
    vertices = [
        np.asarray(geometry.vertices, dtype=float) * np.asarray(scale, dtype=float)
        for geometry in loaded.geometry.values()
    ]
    if not vertices:
        return np.empty((0, 3), dtype=float)
    return np.vstack(vertices)


def _repair_degenerate_mesh_geometry(
    geometry_element: ElementTree.Element,
    source_path: Path,
    package_root: Path,
    collision_element: ElementTree.Element,
) -> bool:
    import numpy as np

    mesh = geometry_element.find("mesh")
    if mesh is None or not mesh.get("filename"):
        return False
    try:
        scale_values = [float(value) for value in str(mesh.get("scale", "1 1 1")).split()]
        if len(scale_values) != 3:
            return False
        mesh_path = _mesh_path(str(mesh.get("filename")), source_path, package_root)
        vertices = _mesh_vertices(mesh_path, scale_values)
    except Exception:
        return False
    if len(vertices) < 4 or not np.isfinite(vertices).all():
        degenerate = True
    else:
        centered = vertices - vertices.mean(axis=0)
        degenerate = np.linalg.matrix_rank(centered) < 3
    if not degenerate or len(vertices) == 0:
        return False
    minimum = vertices.min(axis=0)
    maximum = vertices.max(axis=0)
    extent = maximum - minimum
    nonzero_extent = float(np.max(extent))
    if not math.isfinite(nonzero_extent) or nonzero_extent <= 0.0:
        return False
    thickness = max(DEGENERATE_MESH_MIN_THICKNESS_M, nonzero_extent * 1.0e-3)
    extent = np.maximum(extent, thickness)
    center = (minimum + maximum) * 0.5
    origin = collision_element.find("origin")
    if origin is None:
        origin = ElementTree.Element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        collision_element.insert(0, origin)
    rotation, translation = _origin_values(origin)
    adjusted_translation = translation + rotation @ center
    origin.set("xyz", " ".join(f"{float(value):.9g}" for value in adjusted_translation))
    for child in list(geometry_element):
        geometry_element.remove(child)
    ElementTree.SubElement(
        geometry_element,
        "box",
        {"size": " ".join(f"{float(value):.9g}" for value in extent)},
    )
    return True


def _origin_values(element: ElementTree.Element) -> tuple[Any, Any]:
    import numpy as np

    xyz = [float(value) for value in str(element.get("xyz", "0 0 0")).split()]
    rpy = [float(value) for value in str(element.get("rpy", "0 0 0")).split()]
    if len(xyz) != 3 or len(rpy) != 3:
        raise RuntimeErrorTipOver("URDF origin must contain three xyz and rpy values")
    return _rpy_matrix(rpy), np.asarray(xyz, dtype=float)


def make_locked_urdf(
    source_path: Path,
    package_root: Path,
    output_path: Path,
    *,
    collision_policy: str = "source_only",
) -> dict[str, Any]:
    if collision_policy not in COLLISION_POLICIES:
        raise RuntimeErrorTipOver(f"unknown collision policy: {collision_policy}")
    try:
        tree = ElementTree.parse(source_path)
    except (OSError, ElementTree.ParseError) as error:
        raise RuntimeErrorTipOver(f"cannot parse source URDF: {error}") from error
    root = tree.getroot()
    locked_names: list[str] = []
    for joint in root.findall(".//joint"):
        if joint.get("type") not in {"revolute", "continuous", "prismatic"}:
            continue
        name = joint.get("name")
        if isinstance(name, str):
            locked_names.append(name)
        joint.set("type", "fixed")
        for tag in ("axis", "limit", "dynamics", "mimic", "safety_controller", "calibration"):
            for child in list(joint.findall(tag)):
                joint.remove(child)
    visual_fallback_links: list[str] = []
    visual_fallback_geometry_count = 0
    repaired_degenerate_meshes = 0
    for link in root.findall("link"):
        collisions = link.findall("collision")
        visuals = link.findall("visual")
        if collision_policy != "source_only" and not collisions and visuals:
            for visual in visuals:
                collision = deepcopy(visual)
                collision.tag = "collision"
                for material in list(collision.findall("material")):
                    collision.remove(material)
                link.append(collision)
                visual_fallback_geometry_count += 1
            name = link.get("name")
            if isinstance(name, str):
                visual_fallback_links.append(name)
    rewritten_meshes = 0
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        mesh.set("filename", str(_mesh_path(filename, source_path, package_root)))
        rewritten_meshes += 1
    if collision_policy == "robust_visual_collision":
        for collision in root.findall(".//collision"):
            geometry = collision.find("geometry")
            if geometry is not None:
                repaired_degenerate_meshes += int(
                    _repair_degenerate_mesh_geometry(
                        geometry, source_path, package_root, collision
                    )
                )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    receipt: dict[str, Any] = {
        "policy": "movable_joints_to_fixed_at_urdf_q_zero",
        "collision_policy": collision_policy,
        "visual_fallback_links": sorted(visual_fallback_links),
        "visual_fallback_geometry_count": visual_fallback_geometry_count,
        "repaired_degenerate_meshes": repaired_degenerate_meshes,
        "source_path": str(source_path),
        "source_sha256": _core.sha256_file(source_path),
        "locked_path": str(output_path),
        "locked_sha256": _core.sha256_file(output_path),
        "locked_joint_names": sorted(locked_names),
        "locked_joint_count": len(locked_names),
        "rewritten_mesh_count": rewritten_meshes,
    }
    receipt["receipt_sha256"] = _core._runtime.canonical_sha256(
        receipt, exclude_fields=("receipt_sha256", "locked_path")
    )
    return receipt


def canonical_bbox(row: Mapping[str, Any]) -> dict[str, float]:
    bbox = row.get("bounding_box")
    if not isinstance(bbox, Mapping) or bbox.get("status") != "available":
        raise RuntimeErrorTipOver("canonical collision bounding box is unavailable")
    minimum = bbox.get("minimum_xyz_m")
    maximum = bbox.get("maximum_xyz_m")
    if not (
        isinstance(minimum, list)
        and isinstance(maximum, list)
        and len(minimum) == 3
        and len(maximum) == 3
        and all(_finite_number(value) for value in minimum + maximum)
    ):
        raise RuntimeErrorTipOver("canonical collision bounding box is malformed")
    height = float(maximum[2]) - float(minimum[2])
    if height <= 0.0:
        raise RuntimeErrorTipOver("canonical collision bounding box has non-positive height")
    return {
        "minimum_z_m": float(minimum[2]),
        "maximum_z_m": float(maximum[2]),
        "height_m": height,
        "initial_root_z_m": INITIAL_CLEARANCE_M - float(minimum[2]),
    }


def perturbation_delta_v(height_m: float) -> float:
    if not math.isfinite(height_m) or height_m <= 0.0:
        raise ValueError("height_m must be finite and positive")
    return NORMALIZED_SPEED_KICK * math.sqrt(GRAVITY_M_PER_S2 * height_m)


def perturbation_delta_omega(height_m: float) -> float:
    if not math.isfinite(height_m) or height_m <= 0.0:
        raise ValueError("height_m must be finite and positive")
    return NORMALIZED_ANGULAR_SPEED_KICK * math.sqrt(GRAVITY_M_PER_S2 / height_m)


def tilt_deg(quaternion_wxyz: Sequence[float]) -> float:
    if len(quaternion_wxyz) != 4:
        raise ValueError("quaternion must contain four values")
    w, x, y, z = (float(value) for value in quaternion_wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("quaternion norm must be finite and positive")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


class GenesisTipOverRuntime:
    def __init__(
        self,
        locked_path: Path,
        row: Mapping[str, Any],
        source_protocol: Mapping[str, Any],
        *,
        recompute_inertia: bool = False,
    ) -> None:
        import genesis as gs
        import torch

        expected_version = source_protocol["adapters"]["genesis"].get("version")
        if isinstance(expected_version, str) and expected_version:
            _core._legacy._distribution_version(["genesis-world", "genesis"], expected_version)
        if not torch.cuda.is_available() or torch.cuda.device_count() != 1:
            raise RuntimeErrorTipOver("Genesis worker must have exactly one visible CUDA device")
        physical_token = os.environ.get("CUDA_VISIBLE_DEVICES")
        if not physical_token or "," in physical_token:
            raise RuntimeErrorTipOver("Genesis worker lacks an exact CUDA_VISIBLE_DEVICES binding")
        query = subprocess.run(
            [
                "nvidia-smi",
                f"--id={physical_token}",
                "--query-gpu=index,uuid,name,memory.total,driver_version",
                "--format=csv,noheader,nounits",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=15,
        )
        fields = [item.strip() for item in query.stdout.strip().split(",")]
        if len(fields) != 5:
            raise RuntimeErrorTipOver("nvidia-smi returned malformed GPU identity")
        self.device_receipt = {
            "binding_policy": "current_cuda_visible_devices",
            "physical_device_token": physical_token,
            "physical_device_index": int(fields[0]),
            "gpu_uuid": fields[1],
            "device_name": fields[2],
            "total_memory_mib": int(fields[3]),
            "driver_version": fields[4],
            "logical_device": "cuda:0",
            "torch_device_name": str(torch.cuda.get_device_name(0)),
        }
        self.gs = gs
        self.bbox = canonical_bbox(row)
        self.recompute_inertia = bool(recompute_inertia)
        gs.init(
            backend=gs.cuda,
            precision="32",
            seed=int(source_protocol["runtime"].get("random_seed", 0)),
            logging_level=logging.WARNING,
        )
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(
                dt=TIMESTEP_S,
                substeps=1,
                gravity=(0.0, 0.0, -GRAVITY_M_PER_S2),
            ),
            rigid_options=gs.options.RigidOptions(
                enable_collision=True,
                enable_self_collision=False,
                enable_neutral_collision=False,
                iterations=50,
            ),
            show_viewer=False,
        )
        material = gs.materials.Rigid(friction=SURFACE_FRICTION)
        self.scene.add_entity(gs.morphs.Plane(), material=material)
        self.entity = self.scene.add_entity(
            gs.morphs.URDF(
                file=str(locked_path),
                fixed=False,
                pos=(0.0, 0.0, self.bbox["initial_root_z_m"]),
                visualization=False,
                collision=True,
                merge_fixed_links=True,
                requires_jac_and_IK=False,
                recompute_inertia=self.recompute_inertia,
                align=False,
            ),
            material=material,
        )
        self.scene.build()
        if int(self.entity.n_dofs) != 6:
            raise RuntimeErrorTipOver(
                f"hard-locked free asset must expose exactly six root DoFs, observed {self.entity.n_dofs}"
            )

    def close(self) -> None:
        try:
            self.scene.destroy()
        finally:
            self.gs.destroy()

    def trial(self, trial_id: str, direction: tuple[float, float] | None) -> dict[str, Any]:
        self.scene.reset()
        initial_position = _values(self.entity.get_pos(relative=True))
        if len(initial_position) != 3:
            raise RuntimeErrorTipOver("Genesis initial root position has an invalid shape")
        trace = hashlib.sha256()
        tilt_samples: list[float] = []
        root_z_samples: list[float] = []
        steps_completed = 0
        kick_m_per_s = 0.0
        angular_kick_rad_per_s = 0.0
        error: dict[str, str] | None = None
        try:
            for step in range(1, HORIZON_STEPS + 1):
                if direction is not None and step == PERTURBATION_STEP + 1:
                    velocity = _values(self.entity.get_dofs_velocity())
                    if len(velocity) != 6:
                        raise RuntimeErrorTipOver("free root velocity does not contain six DoFs")
                    kick_m_per_s = perturbation_delta_v(self.bbox["height_m"])
                    angular_kick_rad_per_s = perturbation_delta_omega(self.bbox["height_m"])
                    velocity[0] += direction[0] * kick_m_per_s
                    velocity[1] += direction[1] * kick_m_per_s
                    velocity[3] -= direction[1] * angular_kick_rad_per_s
                    velocity[4] += direction[0] * angular_kick_rad_per_s
                    self.entity.set_dofs_velocity(velocity)
                self.scene.step()
                quaternion = _values(self.entity.get_quat(relative=True))
                position = _values(self.entity.get_pos(relative=True))
                if len(quaternion) != 4 or len(position) != 3:
                    raise RuntimeErrorTipOver("Genesis root pose has an invalid shape")
                current_tilt = tilt_deg(quaternion)
                tilt_samples.append(current_tilt)
                root_z_samples.append(position[2])
                if step % SAMPLE_STRIDE == 0:
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
        initial_root_z = initial_position[2]
        root_drop_heights = (
            max(0.0, initial_root_z - minimum_root_z) / self.bbox["height_m"]
            if minimum_root_z is not None
            else None
        )
        passed = bool(
            full_horizon
            and maximum_tilt is not None
            and maximum_tilt <= TIP_ANGLE_DEG
            and maximum_final_tilt is not None
            and maximum_final_tilt <= FINAL_TIP_ANGLE_DEG
            and root_drop_heights is not None
            and root_drop_heights <= MAX_ROOT_DROP_HEIGHTS
        )
        return {
            "trial_id": trial_id,
            "kind": "gravity_only" if direction is None else "horizontal_perturbation",
            "direction_xy": list(direction) if direction is not None else None,
            "kick_m_per_s": kick_m_per_s,
            "angular_kick_rad_per_s": angular_kick_rad_per_s,
            "normalized_kick": NORMALIZED_SPEED_KICK if direction is not None else 0.0,
            "normalized_angular_kick": (
                NORMALIZED_ANGULAR_SPEED_KICK if direction is not None else 0.0
            ),
            "valid": full_horizon,
            "passed": passed,
            "steps_completed": steps_completed,
            "simulated_seconds": steps_completed * TIMESTEP_S,
            "maximum_tilt_deg": maximum_tilt,
            "maximum_final_window_tilt_deg": maximum_final_tilt,
            "minimum_root_z_m": minimum_root_z,
            "root_drop_in_bbox_heights": root_drop_heights,
            "trace_sha256": trace.hexdigest(),
            "error": error,
        }


def evaluate_tipover(runtime: GenesisTipOverRuntime, row: Mapping[str, Any]) -> dict[str, Any]:
    gravity = runtime.trial("gravity", None)
    perturbed = [
        runtime.trial(name, direction)
        for name, direction in PERTURBATION_DIRECTIONS.items()
    ]
    trials = [gravity, *perturbed]
    physics = row.get("physics")
    receipt: dict[str, Any] = {
        "schema_version": RECEIPT_SCHEMA,
        "protocol_id": PROTOCOL_ID,
        "protocol_sha256": _protocol()["protocol_sha256"],
        "tip_over_stability_passed": gravity["passed"] is True,
        "perturbed_tip_over_stability_passed": bool(
            gravity["passed"] is True and all(trial["passed"] is True for trial in perturbed)
        ),
        "valid_trial_count": sum(trial["valid"] is True for trial in trials),
        "planned_trial_count": len(trials),
        "canonical_bbox": deepcopy(runtime.bbox),
        "physics_source_status": physics.get("status") if isinstance(physics, Mapping) else None,
        "physics_policy_id": physics.get("policy_id") if isinstance(physics, Mapping) else None,
        "trials": trials,
    }
    receipt["receipt_sha256"] = _core._runtime.canonical_sha256(
        receipt, exclude_fields=("receipt_sha256",)
    )
    return receipt


def _checkpoint(path: Path, response: Mapping[str, Any]) -> None:
    _core._runtime.atomic_write_json(path, response)


def worker_main(request_path: Path, response_path: Path) -> int:
    runtime: GenesisTipOverRuntime | None = None
    response: dict[str, Any] = {
        "metrics": _core._false_legacy_metrics(),
        "tipover_stability": {
            "schema_version": EVIDENCE_SCHEMA,
            "protocol": _protocol(),
            "source": None,
            "joint_lock": None,
            "rollout": {
                "tip_over_stability_passed": False,
                "perturbed_tip_over_stability_passed": False,
            },
        },
    }
    evidence = response["tipover_stability"]
    try:
        request = _core._read_json(request_path, "worker request")
        if request.get("schema_version") != _core._runtime.WORKER_REQUEST_SCHEMA:
            raise RuntimeErrorTipOver("worker request schema mismatch")
        if request.get("simulator") != "genesis":
            raise RuntimeErrorTipOver("tip-over worker supports Genesis only")
        row = deepcopy(request.get("row"))
        source_protocol = deepcopy(request.get("protocol"))
        if not isinstance(row, dict) or not isinstance(source_protocol, dict):
            raise RuntimeErrorTipOver("worker row/protocol is malformed")
        _validate_source_protocol(source_protocol)
        source = _core._simulator_source(row, "genesis")
        source_path = Path(str(request["urdf_path"])).resolve(strict=True)
        if source_path != Path(source["path"]).resolve(strict=True):
            raise RuntimeErrorTipOver("worker source path differs from prepared binding")
        if _core.sha256_file(source_path) != source["sha256"]:
            raise RuntimeErrorTipOver("worker source SHA256 differs from prepared binding")
        evidence["source"] = source
        with tempfile.TemporaryDirectory(prefix="table5_tipover_") as directory:
            locked_path = Path(directory) / "model.locked.urdf"
            lock_receipt = make_locked_urdf(
                source_path,
                Path(str(source.get("package_root") or source_path.parent)),
                locked_path,
            )
            evidence["joint_lock"] = lock_receipt
            _checkpoint(response_path, response)
            runtime = GenesisTipOverRuntime(locked_path, row, source_protocol)
            evidence["native_import"] = {
                "passed": True,
                "operation": "Scene.add_entity+Scene.build",
                "locked_source_sha256": lock_receipt["locked_sha256"],
            }
            evidence["rollout"] = evaluate_tipover(runtime, row)
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
    _checkpoint(response_path, response)
    return 0


def _terminal_record(*args: Any, **kwargs: Any) -> dict[str, Any]:
    outcome = args[1] if len(args) > 1 else kwargs.get("outcome")
    record = _ORIGINAL_TERMINAL_RECORD(*args, **kwargs)
    raw = getattr(outcome, "response", None)
    evidence = raw.get("tipover_stability") if isinstance(raw, Mapping) else None
    if record.get("terminal_status") != "completed" and isinstance(evidence, Mapping):
        record["evaluation"]["tipover_stability"] = deepcopy(dict(evidence))
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
    except RuntimeErrorTipOver as error:
        print(f"table5_tipover_runtime: {error}", file=sys.stderr)
        raise SystemExit(2)
