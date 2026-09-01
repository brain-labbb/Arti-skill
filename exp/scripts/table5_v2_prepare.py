#!/usr/bin/env python3
"""Prepare canonical, physics-bound inputs for the frozen Table 5 v2 cohort.

This command performs no simulator import or stepping.  It validates the
frozen cohort, derives one collision-space scale receipt per asset, audits
released inertials, and compiles PV-A physics.json sidecars into injected URDFs.
The prepared manifest is written last, so its presence is the completion marker.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
REPO_ROOT = SCRIPT_PATH.parents[2]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import run_table5_sketch_mobility as _evaluator  # noqa: E402
import table5_n200_manifest as _base  # noqa: E402
import table5_v2_sample_n200 as _sample  # noqa: E402
from table5_pva_physics import (  # noqa: E402
    PLAN_SCHEMA,
    POLICY_ID as PVA_PHYSICS_POLICY_ID,
    SIDECAR_SCHEMA,
    PhysicsInjectionError,
    build_injected_asset,
)


SCHEMA_VERSION = "table5_v2_prepared_manifest_v1"
PROTOCOL_SCHEMA = "table5_v2_runtime_protocol_v1"
PROTOCOL_ID = "table5-v2-readiness-portability-v1"
PREPARE_POLICY_ID = "table5-v2-canonical-prepare-v1"
BBOX_POLICY_ID = "canonical-urdf-q0-object-aabb-collision-preferred-v1"
BASELINE_PHYSICS_POLICY_ID = "released-valid-urdf-inertial-v1"
DEFAULT_COHORT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_core200_articraft10787_infinigen_paired_official/cohort_manifest.json"
)
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_core200_prepared_articraft10787_infinigen_released_native_official"
)
DEFAULT_WORKERS = 8
SIMULATORS = ("genesis", "pybullet", "mujoco")
FK_ALPHAS = (0.0, 0.25, 0.5, 0.75, 1.0)
class PrepareError(ValueError):
    """Raised when the frozen cohort or preparation contract is malformed."""


def canonical_sha256(value: Any, *, exclude_fields: Iterable[str] = ()) -> str:
    return _base.canonical_sha256(value, exclude_fields=tuple(exclude_fields))


def sha256_file(path: Path) -> str:
    return _base.sha256_file(path)


def atomic_write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (
        json.dumps(
            value,
            ensure_ascii=True,
            allow_nan=False,
            sort_keys=True,
            indent=2,
        )
        + "\n"
    )
    descriptor, temporary = tempfile.mkstemp(
        dir=path.parent, prefix=f".{path.name}.", suffix=".tmp"
    )
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


def _finite_positive(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        and float(value) > 0.0
    )


def _vector(text: str | None, default: Sequence[float]) -> np.ndarray:
    if text is None:
        return np.asarray(default, dtype=float)
    values = [float(item) for item in text.split()]
    if len(values) != len(default) or not np.isfinite(values).all():
        raise PrepareError(f"invalid vector: {text!r}")
    return np.asarray(values, dtype=float)


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _quat_matrix(quaternion_wxyz: Sequence[float]) -> np.ndarray:
    q = np.asarray(quaternion_wxyz, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise PrepareError("invalid FK quaternion")
    norm = float(np.linalg.norm(q))
    if norm <= 0.0:
        raise PrepareError("zero FK quaternion")
    w, x, y, z = q / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def _origin_transform(element: ET.Element) -> np.ndarray:
    origin = element.find("origin")
    xyz = _vector(origin.get("xyz") if origin is not None else None, (0, 0, 0))
    rpy = _vector(origin.get("rpy") if origin is not None else None, (0, 0, 0))
    return _transform(_rpy_matrix(rpy), xyz)


def _resolve_mesh(filename: str, urdf_path: Path, package_root: Path) -> Path:
    candidates: list[Path] = []
    if filename.startswith("file://"):
        candidates.append(Path(filename[7:]))
    elif filename.startswith("package://"):
        suffix = filename[len("package://") :]
        parts = Path(suffix).parts
        candidates.append(package_root / suffix)
        if len(parts) > 1:
            candidates.append(package_root / Path(*parts[1:]))
        candidates.append(urdf_path.parent / Path(*parts[1:]))
    else:
        raw = Path(filename)
        candidates.append(raw if raw.is_absolute() else urdf_path.parent / raw)
        candidates.append(package_root / raw)
    for candidate in candidates:
        resolved = candidate.resolve(strict=False)
        if resolved.is_file() and not resolved.is_symlink():
            return resolved
    raise FileNotFoundError(f"collision mesh is unavailable: {filename}")


def _mesh_vertices(path: Path, scale: np.ndarray) -> np.ndarray:
    import trimesh

    loaded = trimesh.load(path, process=False)
    meshes = (
        list(loaded.geometry.values())
        if isinstance(loaded, trimesh.Scene)
        else [loaded]
    )
    vertices = [
        np.asarray(mesh.vertices, dtype=float) * scale
        for mesh in meshes
        if hasattr(mesh, "vertices") and len(mesh.vertices)
    ]
    if not vertices:
        raise PrepareError(f"collision mesh has no vertices: {path}")
    result = np.concatenate(vertices, axis=0)
    if result.ndim != 2 or result.shape[1] != 3 or not np.isfinite(result).all():
        raise PrepareError(f"collision mesh has invalid vertices: {path}")
    return result


def _geometry_vertices(
    geometry: ET.Element, *, urdf_path: Path, package_root: Path
) -> tuple[np.ndarray, dict[str, Any]]:
    children = list(geometry)
    if len(children) != 1:
        raise PrepareError("collision geometry must contain exactly one shape")
    shape = children[0]
    if shape.tag == "box":
        size = _vector(shape.get("size"), (0, 0, 0))
        if np.any(size <= 0):
            raise PrepareError("box size must be positive")
        half = size / 2.0
        vertices = np.asarray(
            [
                [x, y, z]
                for x in (-half[0], half[0])
                for y in (-half[1], half[1])
                for z in (-half[2], half[2])
            ]
        )
        return vertices, {"kind": "box"}
    if shape.tag == "sphere":
        radius = float(shape.get("radius", "nan"))
        if not math.isfinite(radius) or radius <= 0:
            raise PrepareError("sphere radius must be positive")
        vertices = np.asarray(
            [
                [x, y, z]
                for x in (-radius, radius)
                for y in (-radius, radius)
                for z in (-radius, radius)
            ]
        )
        return vertices, {"kind": "sphere"}
    if shape.tag == "cylinder":
        radius = float(shape.get("radius", "nan"))
        length = float(shape.get("length", "nan"))
        if not all(math.isfinite(value) and value > 0 for value in (radius, length)):
            raise PrepareError("cylinder dimensions must be positive")
        vertices = np.asarray(
            [
                [radius * math.cos(2 * math.pi * index / 64),
                 radius * math.sin(2 * math.pi * index / 64), z]
                for index in range(64)
                for z in (-length / 2.0, length / 2.0)
            ]
        )
        return vertices, {"kind": "cylinder", "radial_samples": 64}
    if shape.tag == "mesh":
        filename = shape.get("filename")
        if not filename:
            raise PrepareError("mesh has no filename")
        scale = _vector(shape.get("scale"), (1, 1, 1))
        if np.any(scale <= 0):
            raise PrepareError("mesh scale must be positive")
        path = _resolve_mesh(filename, urdf_path, package_root)
        return _mesh_vertices(path, scale), {
            "kind": "mesh",
            "path": str(path),
            "sha256": sha256_file(path),
        }
    raise PrepareError(f"unsupported collision geometry: {shape.tag}")


def derive_collision_bbox(
    urdf_path: Path,
    package_root: Path,
    joint_tree: Mapping[str, Any],
) -> dict[str, Any]:
    """Derive a shared q=0 object AABB without a simulator importer."""

    try:
        root = ET.parse(urdf_path).getroot()
        scalar_positions = {
            str(joint["name"]): 0.0
            for joint in joint_tree["joints"]
            if joint.get("type") in _base.SCALAR_JOINT_TYPES
        }
        poses = _evaluator.fk_link_poses(dict(joint_tree), scalar_positions)
        collision_count = sum(len(link.findall("collision")) for link in root.findall("link"))
        geometry_role = "collision" if collision_count else "visual"
        all_vertices: list[np.ndarray] = []
        geometry_receipts: list[dict[str, Any]] = []
        for link in root.findall("link"):
            link_name = link.get("name")
            pose = poses.get(str(link_name))
            if pose is None:
                raise PrepareError(f"FK pose unavailable for link: {link_name}")
            link_transform = _transform(
                _quat_matrix(pose["rotation"]),
                np.asarray(pose["translation"], dtype=float),
            )
            for geometry_index, element in enumerate(link.findall(geometry_role)):
                geometry = element.find("geometry")
                if geometry is None:
                    raise PrepareError(f"{geometry_role} geometry missing: {link_name}")
                vertices, receipt = _geometry_vertices(
                    geometry, urdf_path=urdf_path, package_root=package_root
                )
                transform = link_transform @ _origin_transform(element)
                homogeneous = np.column_stack(
                    (vertices, np.ones(vertices.shape[0], dtype=float))
                )
                world = (transform @ homogeneous.T).T[:, :3]
                if not np.isfinite(world).all():
                    raise PrepareError("transformed collision vertices are non-finite")
                all_vertices.append(world)
                geometry_receipts.append(
                    {
                        "link_name": link_name,
                        "geometry_role": geometry_role,
                        "geometry_index": geometry_index,
                        **receipt,
                    }
                )
        if not all_vertices:
            raise PrepareError("URDF contains no collision or visual geometry")
        points = np.concatenate(all_vertices, axis=0)
        lower = points.min(axis=0)
        upper = points.max(axis=0)
        diagonal = float(np.linalg.norm(upper - lower))
        if not math.isfinite(diagonal) or diagonal <= 0:
            raise PrepareError("collision AABB diagonal is not positive")
        value = {
            "status": "available",
            "policy_id": BBOX_POLICY_ID,
            "configuration": "canonical_q_zero",
            "geometry_role": geometry_role,
            "geometry_selection": (
                "all collision geometries when present; otherwise all visual geometries"
            ),
            "minimum_xyz_m": [float(item) for item in lower],
            "maximum_xyz_m": [float(item) for item in upper],
            "diagonal_m": diagonal,
            "geometry_count": len(geometry_receipts),
            "geometry_receipts_sha256": canonical_sha256(geometry_receipts),
        }
    except Exception as error:  # retain asset membership and explicit coverage
        value = {
            "status": "not_available",
            "policy_id": BBOX_POLICY_ID,
            "configuration": "canonical_q_zero",
            "diagonal_m": None,
            "reason": f"{type(error).__name__}: {error}",
        }
    value["receipt_sha256"] = canonical_sha256(
        value, exclude_fields=("receipt_sha256",)
    )
    return value


def audit_urdf_inertials(urdf_path: Path, joint_tree: Mapping[str, Any]) -> dict[str, Any]:
    """Audit released inertials; a fixed root link is allowed to omit one."""

    root = ET.parse(urdf_path).getroot()
    root_links = set(joint_tree.get("root_links", []))
    rows: list[dict[str, Any]] = []
    required_valid = True
    for link in root.findall("link"):
        name = str(link.get("name") or "")
        required = name not in root_links
        valid, details = _valid_inertial(link)
        if required:
            required_valid &= valid
        rows.append(
            {
                "link_name": name,
                "required_for_fixed_base_dynamics": required,
                "valid": valid,
                "details": details,
            }
        )
    receipt = {
        "policy_id": BASELINE_PHYSICS_POLICY_ID,
        "status": "ready" if required_valid else "blocked",
        "root_links_exempt_because_base_is_fixed": sorted(root_links),
        "link_count": len(rows),
        "required_link_count": sum(row["required_for_fixed_base_dynamics"] for row in rows),
        "valid_required_link_count": sum(
            row["required_for_fixed_base_dynamics"] and row["valid"] for row in rows
        ),
        "links": rows,
    }
    if not required_valid:
        receipt["reason"] = "invalid_or_missing_required_link_inertial"
        receipt["invalid_required_link_names"] = [
            row["link_name"]
            for row in rows
            if row["required_for_fixed_base_dynamics"] and not row["valid"]
        ]
    receipt["receipt_sha256"] = canonical_sha256(
        receipt, exclude_fields=("receipt_sha256",)
    )
    return receipt


def _valid_inertial(link: ET.Element) -> tuple[bool, dict[str, Any] | None]:
    inertials = link.findall("inertial")
    if len(inertials) != 1:
        return False, {
            "reason": "inertial_element_count_must_equal_one",
            "inertial_element_count": len(inertials),
        }
    inertial = inertials[0]
    try:
        mass_node = inertial.find("mass")
        inertia_node = inertial.find("inertia")
        if mass_node is None or inertia_node is None:
            return False, None
        mass = float(mass_node.get("value", "nan"))
        values = {
            key: float(inertia_node.get(key, "nan"))
            for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        }
        matrix = np.asarray(
            [
                [values["ixx"], values["ixy"], values["ixz"]],
                [values["ixy"], values["iyy"], values["iyz"]],
                [values["ixz"], values["iyz"], values["izz"]],
            ]
        )
        origin = inertial.find("origin")
        center = _vector(origin.get("xyz") if origin is not None else None, (0, 0, 0))
        eigenvalues = np.linalg.eigvalsh(matrix)
        valid = bool(
            math.isfinite(mass)
            and mass > 0
            and np.isfinite(center).all()
            and np.isfinite(matrix).all()
            and eigenvalues[0] > 1.0e-12
            and eigenvalues[2] <= eigenvalues[0] + eigenvalues[1] + 1.0e-9
        )
        return valid, {
            "mass_kg": mass,
            "center_of_mass_xyz": [float(item) for item in center],
            "inertia_eigenvalues_kg_m2": [float(item) for item in eigenvalues],
        }
    except (TypeError, ValueError, PrepareError, np.linalg.LinAlgError):
        return False, None


def _protocol(cohort: Mapping[str, Any]) -> dict[str, Any]:
    protocol = _base._frozen_protocol(_sample.SAMPLE_SIZE)
    protocol.update(
        {
            "schema_version": PROTOCOL_SCHEMA,
            "protocol_id": PROTOCOL_ID,
            "cohort_binding": {
                "schema_version": cohort["schema_version"],
                "cohort_sha256": cohort["cohort_sha256"],
                "manifest_sha256": cohort["manifest_sha256"],
                "selection_protocol_sha256": cohort["protocol_sha256"],
            },
            "preparation": {
                "policy_id": PREPARE_POLICY_ID,
                "bbox_policy_id": BBOX_POLICY_ID,
                "baseline_physics_policy_id": BASELINE_PHYSICS_POLICY_ID,
                "pva_physics_policy_id": PVA_PHYSICS_POLICY_ID,
                "pva_sidecar_schema": SIDECAR_SCHEMA,
                "pva_plan_schema": PLAN_SCHEMA,
            },
            "v2_metrics": {
                "asset_denominator": _sample.SAMPLE_SIZE,
                "table5a_simulator": "genesis",
                "import_success": (
                    "adapter creation, physics initialization, first step, and "
                    "canonical mapping receipt are present"
                ),
                "dof_coverage": "mapped canonical scalar DoFs / declared scalar DoFs",
                "stable_rollout": (
                    "asset completes the full rollout with finite states; missing physics fields use simulator-native fallback"
                ),
                "tracking_nrmse": "RMSE(q-q_target)/(upper-lower)",
                "limit_violation": (
                    "max(0, lower-q, q-upper)/(upper-lower), max over time per joint"
                ),
                "fk_probe_alphas": list(FK_ALPHAS),
                "fk_position_error": "Euclidean position error / collision AABB diagonal",
                "fk_rotation_error": "SO(3) geodesic error in radians",
                "reported_percentile": 95,
                "continuous_metrics_require_coverage": True,
                "bootstrap": {
                    "unit": "asset cluster",
                    "resamples": 2000,
                    "confidence_level": 0.95,
                    "seed": "table5-v2-bootstrap-20260828",
                },
            },
        }
    )
    protocol["runtime"]["child_timeout_s"] = 900
    protocol["runtime"]["random_seed"] = 20260828
    protocol["cross_simulator"]["fk_probe"] = {
        "gravity": "off_by_direct_reset_without_step",
        "contact_response": "not_stepped",
        "alphas": list(FK_ALPHAS),
        "reference": "canonical_URDF_FK_at_measured_joint_states",
    }
    protocol["simulator_source_policy"] = deepcopy(
        cohort["protocol"].get(
            "simulator_source_policy",
            {
                "default": "released URDF through each simulator's importer",
                "canonical_metric_schema": "released URDF joint/link schema",
            },
        )
    )
    protocol["adapters"]["mujoco"]["importer"] = (
        "official released MJCF when explicitly bound to the same asset identity; "
        "otherwise released or prepared URDF"
    )
    generic_runtime_script = SCRIPT_PATH.with_name("table5_n200_runtime.py")
    v2_runtime_script = SCRIPT_PATH.with_name("table5_v2_runtime.py")
    aggregate_script = SCRIPT_PATH.with_name("table5_v2_aggregate.py")
    required_sources = [
        SCRIPT_PATH,
        generic_runtime_script,
        v2_runtime_script,
        aggregate_script,
        Path(_evaluator.__file__).resolve(),
        SCRIPT_PATH.with_name("table5_pva_physics.py"),
        SCRIPT_PATH.with_name("table5_pva_physics_n200_runtime.py"),
    ]
    missing = [str(path) for path in required_sources if not path.is_file()]
    if missing:
        raise PrepareError(f"v2 implementation is incomplete: {missing}")
    protocol["implementation"] = {
        "prepare_script": str(SCRIPT_PATH),
        "prepare_script_sha256": sha256_file(SCRIPT_PATH),
        # The generic runner validates these two historical field names.
        "runtime_script": str(generic_runtime_script),
        "runtime_script_sha256": sha256_file(generic_runtime_script),
        "v2_runtime_script": str(v2_runtime_script),
        "v2_runtime_script_sha256": sha256_file(v2_runtime_script),
        "aggregate_script": str(aggregate_script),
        "aggregate_script_sha256": sha256_file(aggregate_script),
        "evaluator_script": str(Path(_evaluator.__file__).resolve()),
        "evaluator_script_sha256": sha256_file(Path(_evaluator.__file__).resolve()),
        "pva_physics_script": str(SCRIPT_PATH.with_name("table5_pva_physics.py")),
        "pva_physics_script_sha256": sha256_file(
            SCRIPT_PATH.with_name("table5_pva_physics.py")
        ),
        "pva_physics_runtime_script": str(
            SCRIPT_PATH.with_name("table5_pva_physics_n200_runtime.py")
        ),
        "pva_physics_runtime_script_sha256": sha256_file(
            SCRIPT_PATH.with_name("table5_pva_physics_n200_runtime.py")
        ),
    }
    protocol["protocol_sha256"] = canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    return protocol


def _load_cohort(path: Path) -> dict[str, Any]:
    try:
        cohort = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise PrepareError(f"cannot load cohort: {error}") from error
    if not isinstance(cohort, dict):
        raise PrepareError("cohort must be a JSON object")
    _sample.validate_manifest(cohort)
    return cohort


def _prepared_simulator_sources(
    raw: Mapping[str, Any],
    *,
    effective_urdf: Path,
    effective_hash: str | None,
    source_package: Path,
    issues: list[str],
) -> dict[str, dict[str, Any]]:
    declared = raw.get("simulator_sources")
    declared_sources = declared if isinstance(declared, Mapping) else {}
    sources: dict[str, dict[str, Any]] = {}
    for simulator in SIMULATORS:
        candidate = declared_sources.get(simulator)
        if not isinstance(candidate, Mapping):
            sources[simulator] = {
                "format": "urdf",
                "representation": "released_or_prepared_canonical_urdf",
                "path": str(effective_urdf),
                "sha256": effective_hash,
                "package_root": str(source_package),
            }
            continue
        source_format = str(candidate.get("format") or "").lower()
        representation = str(candidate.get("representation") or "released")
        supported = source_format == "urdf" or (
            simulator == "mujoco"
            and source_format == "mjcf"
            and representation == "official_released_mjcf"
        )
        if not supported:
            issues.append(f"{simulator}_source_format_unsupported:{source_format}")
        source_path = Path(str(candidate.get("path") or "")).resolve(strict=False)
        expected_hash = candidate.get("sha256")
        try:
            source_path = source_path.resolve(strict=True)
            observed_hash = sha256_file(source_path)
            if observed_hash != expected_hash:
                issues.append(f"{simulator}_source_sha256_mismatch")
        except OSError as error:
            observed_hash = None
            issues.append(
                f"{simulator}_source_unavailable:{type(error).__name__}:{error}"
            )
        sources[simulator] = {
            "format": source_format,
            "representation": representation,
            "path": str(source_path),
            "sha256": observed_hash,
            "package_root": str(
                Path(str(candidate.get("package_root") or source_path.parent)).resolve(
                    strict=False
                )
            ),
        }
    return sources


def _prepare_row(
    raw: Mapping[str, Any],
    *,
    dataset_slug: str,
    dataset_name: str,
    selection_order: int,
    output: Path,
) -> dict[str, Any]:
    source_urdf = Path(str(raw["urdf_path"])).resolve(strict=False)
    source_package = Path(str(raw["package_root"])).resolve(strict=False)
    issues: list[str] = []
    warnings: list[str] = []
    source_hash: str | None = None
    try:
        source_urdf = source_urdf.resolve(strict=True)
        source_hash = sha256_file(source_urdf)
        if source_hash != raw.get("urdf_sha256"):
            issues.append("source_urdf_sha256_mismatch")
    except OSError as error:
        issues.append(f"source_urdf_unavailable:{type(error).__name__}:{error}")
    parsed = _base._parse_urdf(source_package, source_urdf)
    issues.extend(str(item) for item in parsed["issues"])
    joint_tree = parsed["joint_tree"]
    scalar_joints = parsed["scalar_joints"] if isinstance(joint_tree, dict) else []

    effective_urdf = source_urdf
    physics: dict[str, Any]
    if dataset_slug == _sample.PVA_SLUG and source_hash is not None:
        physics_path = source_package / "physics.json"
        asset_root = output / "canonical_assets" / dataset_slug / f"pva_{selection_order:04d}"
        injected_urdf = asset_root / "model.physics.urdf"
        plan_path = asset_root / "physics_plan.json"
        try:
            plan = build_injected_asset(
                source_urdf=source_urdf,
                physics_path=physics_path,
                destination_urdf=injected_urdf,
                plan_path=plan_path,
            )
            effective_urdf = injected_urdf.resolve(strict=True)
            physics = {
                "status": "ready",
                "policy_id": PVA_PHYSICS_POLICY_ID,
                "source_urdf_path": str(source_urdf),
                "source_urdf_sha256": source_hash,
                "physics_sidecar_path": str(physics_path.resolve(strict=True)),
                "physics_sidecar_sha256": plan["physics_sha256"],
                "physics_plan_path": str(plan_path.resolve(strict=True)),
                "physics_plan_sha256": plan["plan_sha256"],
                "injected_urdf_path": str(effective_urdf),
                "injected_urdf_sha256": plan["injected_urdf_sha256"],
                "derived_inertial_link_count": plan["derived_inertial_link_count"],
                "preserved_inertial_link_count": plan[
                    "preserved_inertial_link_count"
                ],
                "convex_hull_fallback_collision_count": plan[
                    "convex_hull_fallback_collision_count"
                ],
                "appearance_only_collision_count": plan[
                    "appearance_only_collision_count"
                ],
                "collisionless_valid_inertial_link_count": plan[
                    "collisionless_valid_inertial_link_count"
                ],
            }
        except (OSError, PhysicsInjectionError, ValueError) as error:
            physics = {
                "status": "blocked",
                "policy_id": PVA_PHYSICS_POLICY_ID,
                "source_urdf_path": str(source_urdf),
                "source_urdf_sha256": source_hash,
                "physics_sidecar_path": str(physics_path.resolve(strict=False)),
                "reason": f"{type(error).__name__}: {error}",
            }
            warnings.append("physics_compile_blocked")
    elif isinstance(joint_tree, Mapping) and source_urdf.is_file():
        try:
            physics = audit_urdf_inertials(source_urdf, joint_tree)
        except (OSError, ET.ParseError, ValueError) as error:
            physics = {
                "status": "blocked",
                "policy_id": BASELINE_PHYSICS_POLICY_ID,
                "reason": f"{type(error).__name__}: {error}",
            }
        if physics["status"] != "ready":
            warnings.append("released_physics_blocked")
    else:
        physics = {
            "status": "blocked",
            "policy_id": (
                PVA_PHYSICS_POLICY_ID
                if dataset_slug == _sample.PVA_SLUG
                else BASELINE_PHYSICS_POLICY_ID
            ),
            "reason": "source URDF or canonical joint tree unavailable",
        }

    effective_hash = sha256_file(effective_urdf) if effective_urdf.is_file() else None
    simulator_sources = _prepared_simulator_sources(
        raw,
        effective_urdf=effective_urdf,
        effective_hash=effective_hash,
        source_package=source_package,
        issues=issues,
    )
    effective_parsed = _base._parse_urdf(source_package, effective_urdf)
    if effective_parsed["joint_tree"] is not None:
        joint_tree = effective_parsed["joint_tree"]
        scalar_joints = effective_parsed["scalar_joints"]
    bbox = (
        derive_collision_bbox(effective_urdf, source_package, joint_tree)
        if effective_urdf.is_file() and isinstance(joint_tree, Mapping)
        else {
            "status": "not_available",
            "policy_id": BBOX_POLICY_ID,
            "configuration": "canonical_q_zero",
            "diagonal_m": None,
            "reason": "canonical URDF or joint tree unavailable",
        }
    )
    if "receipt_sha256" not in bbox:
        bbox["receipt_sha256"] = canonical_sha256(
            bbox, exclude_fields=("receipt_sha256",)
        )
    if bbox["status"] != "available":
        warnings.append("collision_bbox_not_available")
    dataset_id = (
        f"pva_{selection_order:04d}"
        if dataset_slug == _sample.PVA_SLUG
        else str(raw["dataset_id"])
    )
    row: dict[str, Any] = {
        "dataset_slug": dataset_slug,
        "dataset_name": dataset_name,
        "dataset_id": dataset_id,
        "asset_id": str(raw["asset_id"]),
        "category": str(raw.get("category") or "N/E"),
        "selection_order": selection_order,
        "package_root": str(source_package),
        "urdf_path": str(effective_urdf),
        "urdf_sha256": effective_hash,
        "simulator_sources": simulator_sources,
        "source_urdf": {
            "path": str(source_urdf),
            "sha256": source_hash,
            "source_cohort_row_sha256": raw.get("row_sha256"),
        },
        "joint_tree": joint_tree,
        "scalar_joints": scalar_joints,
        "xml_counts": effective_parsed["xml_counts"],
        "bounding_box_diagonal": bbox.get("diagonal_m"),
        "bounding_box": bbox,
        "physics": physics,
        "collision": {
            "policy": (
                "official released simulator-specific representation when bound; "
                "otherwise the effective canonical URDF"
            ),
            "bbox_receipt_sha256": bbox["receipt_sha256"],
        },
        "preflight": {
            "status": "failed" if issues else "pass",
            "issues": sorted(set(issues)),
            "warnings": sorted(set(warnings)),
            "simulator_eligible": not issues,
        },
        "cohort_selection": deepcopy(raw.get("cohort_selection")),
    }
    row["row_sha256"] = canonical_sha256(row, exclude_fields=("row_sha256",))
    return row


def build_manifest(
    cohort_path: Path,
    output: Path,
    *,
    workers: int = DEFAULT_WORKERS,
) -> dict[str, Any]:
    if not isinstance(workers, int) or isinstance(workers, bool) or workers < 1:
        raise PrepareError("workers must be a positive integer")
    output = output.resolve(strict=False)
    if (output / "manifest.json").exists():
        raise FileExistsError(output / "manifest.json")
    output.mkdir(parents=True, exist_ok=True)
    cohort = _load_cohort(cohort_path.resolve(strict=True))
    jobs: list[tuple[int, str, str, int, Mapping[str, Any]]] = []
    global_order = 0
    for dataset in cohort["datasets"]:
        for selection_order, raw in enumerate(dataset["rows"]):
            jobs.append(
                (
                    global_order,
                    str(dataset["dataset_slug"]),
                    str(dataset["dataset_name"]),
                    selection_order,
                    raw,
                )
            )
            global_order += 1
    results: dict[int, dict[str, Any]] = {}
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(
                _prepare_row,
                raw,
                dataset_slug=slug,
                dataset_name=name,
                selection_order=selection_order,
                output=output,
            ): global_index
            for global_index, slug, name, selection_order, raw in jobs
        }
        for future in as_completed(futures):
            results[futures[future]] = future.result()
    prepared_datasets: list[dict[str, Any]] = []
    cursor = 0
    for source_dataset in cohort["datasets"]:
        count = len(source_dataset["rows"])
        rows = [results[index] for index in range(cursor, cursor + count)]
        cursor += count
        prepared_datasets.append(
            {
                "dataset_slug": source_dataset["dataset_slug"],
                "dataset_name": source_dataset["dataset_name"],
                "selection": deepcopy(source_dataset["selection"]),
                "rows": rows,
                "preparation_summary": {
                    "row_count": len(rows),
                    "simulator_eligible_count": sum(
                        row["preflight"]["simulator_eligible"] for row in rows
                    ),
                    "physics_ready_count": sum(
                        row["physics"]["status"] == "ready" for row in rows
                    ),
                    "bbox_available_count": sum(
                        row["bounding_box"]["status"] == "available" for row in rows
                    ),
                },
            }
        )
    protocol = _protocol(cohort)
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "prepare_policy_id": PREPARE_POLICY_ID,
        "source_cohort": {
            "path": str(cohort_path.resolve(strict=True)),
            "manifest_sha256": cohort["manifest_sha256"],
            "cohort_sha256": cohort["cohort_sha256"],
        },
        "protocol": protocol,
        "protocol_sha256": protocol["protocol_sha256"],
        "sample_size": _sample.SAMPLE_SIZE,
        "dataset_count": len(prepared_datasets),
        "total_rows": len(jobs),
        "ordered_dataset_slugs": [
            dataset["dataset_slug"] for dataset in prepared_datasets
        ],
        "datasets": prepared_datasets,
    }
    manifest["prepared_cohort_sha256"] = canonical_sha256(
        [
            {
                "dataset_slug": dataset["dataset_slug"],
                "rows": [
                    {
                        "asset_id": row["asset_id"],
                        "row_sha256": row["row_sha256"],
                        "urdf_sha256": row["urdf_sha256"],
                        "physics_receipt_sha256": row["physics"].get(
                            "receipt_sha256",
                            row["physics"].get("physics_plan_sha256"),
                        ),
                        "bbox_receipt_sha256": row["bounding_box"][
                            "receipt_sha256"
                        ],
                    }
                    for row in dataset["rows"]
                ],
            }
            for dataset in prepared_datasets
        ]
    )
    manifest["manifest_sha256"] = canonical_sha256(
        manifest, exclude_fields=("manifest_sha256",)
    )
    validate_manifest(manifest, verify_files=True)
    atomic_write_json(output / "manifest.json", manifest)
    atomic_write_json(
        output / "preparation_summary.json",
        {
            "schema_version": "table5_v2_preparation_summary_v1",
            "manifest_sha256": manifest["manifest_sha256"],
            "prepared_cohort_sha256": manifest["prepared_cohort_sha256"],
            "datasets": [
                {
                    "dataset_slug": dataset["dataset_slug"],
                    **dataset["preparation_summary"],
                }
                for dataset in prepared_datasets
            ],
        },
    )
    return manifest


def validate_manifest(manifest: Mapping[str, Any], *, verify_files: bool) -> None:
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise PrepareError("prepared manifest schema mismatch")
    protocol = manifest.get("protocol")
    if not isinstance(protocol, Mapping):
        raise PrepareError("prepared manifest protocol is missing")
    expected_protocol = canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    if (
        protocol.get("protocol_sha256") != expected_protocol
        or manifest.get("protocol_sha256") != expected_protocol
    ):
        raise PrepareError("prepared protocol hash mismatch")
    datasets = manifest.get("datasets")
    valid_counts = {
        len(_sample.DATASET_SLUGS),
        len(_sample.EXTENDED_DATASET_SLUGS),
    }
    if not isinstance(datasets, list) or len(datasets) not in valid_counts:
        raise PrepareError("prepared manifest has an unsupported dataset count")
    if [dataset.get("dataset_slug") for dataset in datasets] != manifest.get(
        "ordered_dataset_slugs"
    ):
        raise PrepareError("prepared dataset order mismatch")
    total = 0
    for dataset in datasets:
        rows = dataset.get("rows")
        slug = dataset.get("dataset_slug")
        if not isinstance(rows, list) or len(rows) != _sample.SAMPLE_SIZE:
            raise PrepareError(f"{slug} prepared row count mismatch")
        total += len(rows)
        for order, row in enumerate(rows):
            if row.get("selection_order") != order:
                raise PrepareError(f"{slug} selection order mismatch")
            if row.get("row_sha256") != canonical_sha256(
                row, exclude_fields=("row_sha256",)
            ):
                raise PrepareError(f"{slug}/{order} row hash mismatch")
            urdf_hash = row.get("urdf_sha256")
            if verify_files and isinstance(urdf_hash, str):
                if sha256_file(Path(row["urdf_path"])) != urdf_hash:
                    raise PrepareError(f"{slug}/{order} canonical URDF hash mismatch")
            if verify_files:
                source = row.get("source_urdf")
                if isinstance(source, Mapping) and isinstance(source.get("sha256"), str):
                    if sha256_file(Path(str(source["path"]))) != source["sha256"]:
                        raise PrepareError(f"{slug}/{order} source URDF hash mismatch")
                simulator_sources = row.get("simulator_sources")
                if not isinstance(simulator_sources, Mapping) or set(
                    simulator_sources
                ) != set(SIMULATORS):
                    raise PrepareError(f"{slug}/{order} simulator sources are malformed")
                for simulator, simulator_source in simulator_sources.items():
                    expected = simulator_source.get("sha256")
                    source_path = simulator_source.get("path")
                    if isinstance(expected, str) and sha256_file(
                        Path(str(source_path))
                    ) != expected:
                        raise PrepareError(
                            f"{slug}/{order} {simulator} source hash mismatch"
                        )
                physics = row.get("physics")
                if isinstance(physics, Mapping) and physics.get("status") == "ready":
                    for path_field, hash_field, label in (
                        ("physics_sidecar_path", "physics_sidecar_sha256", "sidecar"),
                        ("physics_plan_path", "physics_plan_sha256", "physics plan"),
                    ):
                        expected = physics.get(hash_field)
                        if isinstance(expected, str) and sha256_file(
                            Path(str(physics[path_field]))
                        ) != expected:
                            raise PrepareError(
                                f"{slug}/{order} {label} hash mismatch"
                            )
    if total != manifest.get("total_rows"):
        raise PrepareError("prepared total_rows mismatch")
    if manifest.get("manifest_sha256") != canonical_sha256(
        manifest, exclude_fields=("manifest_sha256",)
    ):
        raise PrepareError("prepared manifest hash mismatch")


def verify(path: Path, *, verify_files: bool = True) -> dict[str, Any]:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise PrepareError(f"cannot load prepared manifest: {error}") from error
    if not isinstance(manifest, dict):
        raise PrepareError("prepared manifest must be an object")
    validate_manifest(manifest, verify_files=verify_files)
    return manifest


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cohort", type=Path, default=DEFAULT_COHORT)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--verify", type=Path)
    parser.add_argument("--skip-file-hashes", action="store_true")
    arguments = parser.parse_args(argv)
    try:
        if arguments.verify is not None:
            manifest = verify(
                arguments.verify, verify_files=not arguments.skip_file_hashes
            )
            output = arguments.verify
        else:
            manifest = build_manifest(
                arguments.cohort, arguments.out, workers=arguments.workers
            )
            output = arguments.out / "manifest.json"
    except (PrepareError, FileExistsError, OSError) as error:
        parser.error(str(error))
    print(
        json.dumps(
            {
                "manifest": str(output.resolve(strict=False)),
                "manifest_sha256": manifest["manifest_sha256"],
                "prepared_cohort_sha256": manifest["prepared_cohort_sha256"],
                "protocol_sha256": manifest["protocol_sha256"],
                "total_rows": manifest["total_rows"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
