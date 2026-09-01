#!/usr/bin/env python3
"""Run the frozen URDF Sim-Ready Table 4 protocol on Ours-500K (Brain).

Cohort: the exact frozen Ours-500K Table 2 formal manifest
(``table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z``): all 500
acquired assets in ``selection_index`` order, no resampling/replacement.

Frozen evaluation contract (identical to the completed Artiverse /
Articraft-10K / LAM / PartNet-Mobility Table 4 runs):

- rest state: every evaluable joint at ``q = 0``;
- single-joint sweep: K=21 states per joint (Table 3 protocol);
- multi-joint: R=64 Sobol configurations per asset with >=1 movable DoF
  (seed 20260813); zero-DoF assets keep their asset-level denominators;
- pair policy: all-pair and non-adjacent (direct parent-child excluded);
  surface contact allowed, penetration > 1e-6 m illegal;
- scale: q0 collision-shape union AABB diagonal in PyBullet
  (``pybullet_q0_collision_shape_union_aabb_v1``);
- PyBullet load flags: URDF_USE_INERTIA_FROM_FILE | URDF_USE_SELF_COLLISION
  | URDF_USE_SELF_COLLISION_INCLUDE_PARENT | URDF_IGNORE_VISUAL_SHAPES,
  useFixedBase=True;
- every asset runs in a fresh child interpreter with timeout; failures,
  timeouts and unexecuted states stay in the intent-to-evaluate denominators
  (fail closed).

All metrics are aggregated by the shared collision core
(``run_urdf_table4_partnet_mobility.py``, SHA-pinned), whose
``summarize_records`` implements the fail-closed state denominators.
AOR is reported N/E (no stable exact overlap-volume implementation).
"""

from __future__ import annotations

import argparse
from concurrent.futures import as_completed, ThreadPoolExecutor
from datetime import datetime, timezone
import importlib.util
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import tempfile
from typing import Any
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
CORE_SCRIPT = REPO / "exp/scripts/run_urdf_table4_partnet_mobility.py"
DATASET_LABEL = "Ours-500K"
DATASET_ROOT = (
    REPO / "exp/Brain/extracted/arti_cabinet_drawer_geometry_500_20260813"
)
DEFAULT_TABLE2_MANIFEST = (
    REPO
    / "exp/runtime/table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z/manifest.json"
)
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
PROTOCOL_SNAPSHOT_NAME = "protocol_document_at_freeze.md"

PROTOCOL_ID = "urdf_sim_ready_table4_ours_500k_table2_n500_v1"
SAMPLE_SIZE = 500
EXPECTED_N_RELEASE = 500
EXPECTED_TABLE2_MANIFEST_FILE_SHA256 = (
    "f6f2eb2e9a5a0b257d2843674e987946a9d014274348784018540772f2660b71"
)
EXPECTED_TABLE2_MANIFEST_CONTENT_SHA256 = (
    "8b1bdb53bef17ac104bb42daa331899dfa498020be131fe5727f16df0f0427fa"
)
EXPECTED_ARCHIVE_SHA256 = (
    "ffedf5bd90ae5eb96a061d0e127b700915ed6c221eeb7c5afe282b7249bfbd66"
)
EXPECTED_CORE_SHA256 = (
    "e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0"
)
SCALE_PROTOCOL = "pybullet_q0_collision_shape_union_aabb_v1"
SINGLE_SAMPLES = 21
SOBOL_SAMPLES = 64
SOBOL_SEED = 20260813
PENETRATION_THRESHOLD_M = 1e-6
RESET_TOLERANCE = 1e-9
CHILD_TIMEOUT_SECONDS = 900.0
DEFAULT_WORKERS = 4
AUDIT_WORKERS = 4

IDENTITY_FIELDS = (
    "asset_id",
    "raw_category",
    "seed_name",
    "asset_root_relpath",
    "selection_rank",
)
FROZEN_INPUT_FIELDS = (
    "protocol_id",
    "order",
    "dataset_id",
    *IDENTITY_FIELDS,
    "category",
    "package_audit_success",
    "audit_issue",
    "primary_urdf_relpath",
    "urdf_sha256",
    "valid_tree",
    "movable_dof_count",
    "range_evaluable_dof_count",
    "joint_specs_sha256",
    "collision_mesh_inventory_sha256",
    "missing_collision_mesh_reference_count",
    "unsafe_collision_mesh_reference_count",
    "scale_derivation_sha256",
    "object_bbox_diagonal_m",
    "rest_state_expected",
    "single_state_expected",
    "sobol_state_expected",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    digest = __import__("hashlib").sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_core():
    if sha256_file(CORE_SCRIPT) != EXPECTED_CORE_SHA256:
        raise RuntimeError("collision core SHA256 does not match the frozen pin")
    spec = importlib.util.spec_from_file_location("urdf_table4_partnet_core", CORE_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import collision core: {CORE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def canonical_sha256(value: Any) -> str:
    return _load_core().canonical_sha256(value)


def _manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def atomic_json(path: Path, value: Any) -> None:
    _load_core().atomic_json(path, value)


def atomic_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    _load_core().atomic_jsonl(path, rows)


def atomic_text(path: Path, value: str) -> None:
    _load_core().atomic_text(path, value)


def _valid_tree(urdf_path: Path) -> bool:
    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError:
        return False
    if root.tag != "robot":
        return False
    links = [node.attrib.get("name", "").strip() for node in root.findall("link")]
    link_set = set(links)
    if not links or "" in link_set or len(link_set) != len(links):
        return False
    indegree = {name: 0 for name in links}
    adjacency: dict[str, list[str]] = {name: [] for name in links}
    for joint in root.findall("joint"):
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "").strip() if parent_node is not None else ""
        child = child_node.attrib.get("link", "").strip() if child_node is not None else ""
        if parent not in link_set or child not in link_set or parent == child:
            return False
        adjacency[parent].append(child)
        indegree[child] += 1
    roots = [name for name in links if indegree[name] == 0]
    if len(roots) != 1 or any(count > 1 for count in indegree.values()):
        return False
    if len(root.findall("joint")) != len(links) - 1:
        return False
    seen: set[str] = set()
    stack = [roots[0]]
    while stack:
        node = stack.pop()
        if node in seen:
            return False
        seen.add(node)
        stack.extend(adjacency[node])
    return len(seen) == len(links)


def collision_mesh_inventory(
    asset_root: Path, urdf_path: Path
) -> tuple[list[dict[str, Any]], int]:
    package_root = urdf_path.parent.resolve(strict=True)
    asset_root = asset_root.resolve(strict=True)
    root = ET.parse(urdf_path).getroot()
    references = sorted(
        {
            mesh.get("filename", "").replace("\\", "/")
            for mesh in root.findall("link/collision/geometry/mesh")
            if mesh.get("filename")
        }
    )
    inventory: list[dict[str, Any]] = []
    unsafe = 0
    for reference in references:
        path: Path | None = None
        safe = bool(reference) and "://" not in reference and not Path(reference).is_absolute()
        if safe:
            candidate = (package_root / reference).resolve(strict=False)
            try:
                candidate.relative_to(package_root)
                candidate.relative_to(asset_root)
                path = candidate
            except ValueError:
                safe = False
        if not safe:
            unsafe += 1
        exists = bool(path is not None and path.is_file())
        inventory.append(
            {
                "path": reference,
                "safe": safe,
                "resolved_relpath": (
                    path.relative_to(asset_root).as_posix() if path is not None else None
                ),
                "exists": exists,
                "is_symlink": bool(path is not None and path.is_symlink()),
                "size_bytes": path.stat().st_size if exists and path is not None else None,
                "sha256": sha256_file(path) if exists and path is not None else None,
            }
        )
    return inventory, unsafe


def derive_collision_aabb(urdf_path: Path) -> dict[str, Any]:
    import pybullet as bullet

    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        body = bullet.loadURDF(
            str(urdf_path),
            useFixedBase=True,
            flags=flags,
            physicsClientId=client,
        )
        for joint_index in range(bullet.getNumJoints(body, physicsClientId=client)):
            bullet.resetJointState(
                body,
                joint_index,
                0.0,
                targetVelocity=0.0,
                physicsClientId=client,
            )
        collision_links = []
        bounds = []
        for link_index in range(-1, bullet.getNumJoints(body, physicsClientId=client)):
            shapes = bullet.getCollisionShapeData(
                body, link_index, physicsClientId=client
            )
            if not shapes:
                continue
            lower, upper = bullet.getAABB(body, link_index, physicsClientId=client)
            values = [*lower, *upper]
            if not all(math.isfinite(float(value)) for value in values):
                raise ValueError("collision AABB contains non-finite values")
            collision_links.append(link_index)
            bounds.append((tuple(map(float, lower)), tuple(map(float, upper))))
        if not bounds:
            raise ValueError("loaded URDF has no collision shapes")
        minimum = [min(row[0][axis] for row in bounds) for axis in range(3)]
        maximum = [max(row[1][axis] for row in bounds) for axis in range(3)]
        diagonal = math.sqrt(
            sum((high - low) ** 2 for low, high in zip(minimum, maximum))
        )
        if not math.isfinite(diagonal) or diagonal <= 0.0:
            raise ValueError("collision AABB diagonal is not positive finite")
        return {
            "protocol": SCALE_PROTOCOL,
            "status": "PASS",
            "joint_state": "q=0 for every simulator joint",
            "minimum_m": minimum,
            "maximum_m": maximum,
            "diagonal_m": diagonal,
            "collision_link_indices": collision_links,
            "load_flags": flags,
        }
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)


def _empty_audit() -> dict[str, Any]:
    return {
        "package_audit_success": False,
        "audit_issue": None,
        "primary_urdf_relpath": None,
        "urdf_sha256": None,
        "valid_tree": False,
        "movable_dof_count": 0,
        "range_evaluable_dof_count": 0,
        "joint_specs": [],
        "joint_specs_sha256": canonical_sha256([]),
        "collision_mesh_files": [],
        "collision_mesh_inventory_sha256": canonical_sha256([]),
        "missing_collision_mesh_reference_count": 0,
        "unsafe_collision_mesh_reference_count": 0,
        "scale_derivation": {"protocol": SCALE_PROTOCOL, "status": "N/E"},
        "scale_derivation_sha256": None,
        "object_bbox_diagonal_m": None,
    }


def audit_asset(dataset_root: Path, asset_row: dict[str, Any]) -> dict[str, Any]:
    result = _empty_audit()
    try:
        dataset_root = dataset_root.resolve(strict=True)
        asset_root = (dataset_root / str(asset_row["asset_root"])).resolve(strict=True)
        asset_root.relative_to(dataset_root)
        urdf_path = asset_root / "model.urdf"
        if not urdf_path.is_file() or urdf_path.is_symlink():
            raise FileNotFoundError("model.urdf missing or symlinked")
        result["primary_urdf_relpath"] = urdf_path.relative_to(dataset_root).as_posix()
        result["urdf_sha256"] = sha256_file(urdf_path)
        if result["urdf_sha256"] != str(asset_row["primary_urdf_sha256"]):
            raise ValueError("model.urdf hash differs from the frozen Table 2 binding")
        core = _load_core()
        joints = core.parse_urdf_joints(urdf_path)
        result["joint_specs"] = joints
        result["joint_specs_sha256"] = canonical_sha256(joints)
        result["movable_dof_count"] = len(joints)
        result["range_evaluable_dof_count"] = sum(
            bool(row["range_evaluable"]) for row in joints
        )
        result["valid_tree"] = _valid_tree(urdf_path)
        inventory, unsafe = collision_mesh_inventory(asset_root, urdf_path)
        result["collision_mesh_files"] = inventory
        result["collision_mesh_inventory_sha256"] = canonical_sha256(inventory)
        result["unsafe_collision_mesh_reference_count"] = unsafe
        result["missing_collision_mesh_reference_count"] = sum(
            not row["exists"] for row in inventory
        )
        if unsafe:
            raise ValueError(f"unsafe collision mesh reference count: {unsafe}")
        if result["missing_collision_mesh_reference_count"]:
            raise FileNotFoundError(
                "missing collision mesh reference count: "
                f"{result['missing_collision_mesh_reference_count']}"
            )
        if not result["valid_tree"]:
            raise ValueError("URDF joint graph is not a valid rooted tree")
        scale = derive_collision_aabb(urdf_path)
        result["scale_derivation"] = scale
        result["scale_derivation_sha256"] = canonical_sha256(scale)
        result["object_bbox_diagonal_m"] = scale["diagonal_m"]
        result["package_audit_success"] = True
    except Exception as exc:  # noqa: BLE001
        result["audit_issue"] = f"{type(exc).__name__}: {exc}"
    return result


def current_runtime_identity() -> dict[str, Any]:
    import importlib.metadata

    import pybullet as bullet

    core = _load_core()
    pybullet_path = Path(bullet.__file__).resolve(strict=True)
    return {
        "python_executable": sys.executable,
        "python_version": platform_version(),
        "pybullet_module": str(pybullet_path),
        "pybullet_module_sha256": sha256_file(pybullet_path),
        "pybullet_api_version": bullet.getAPIVersion(),
        "adapter_runner": str(SCRIPT),
        "adapter_runner_sha256": sha256_file(SCRIPT),
        "collision_core": str(CORE_SCRIPT),
        "collision_core_sha256": sha256_file(CORE_SCRIPT),
        "hostname": __import__("socket").gethostname(),
        "core_runtime": core.current_runtime_identity(),
    }


def platform_version() -> str:
    import platform

    return platform.python_version()


def load_cohort(table2_manifest_path: Path) -> dict[str, Any]:
    table2_manifest_path = table2_manifest_path.resolve(strict=True)
    file_sha = sha256_file(table2_manifest_path)
    cohort = json.loads(table2_manifest_path.read_text(encoding="utf-8"))
    core = _load_core()
    if cohort.get("manifest_content_sha256") != _manifest_self_hash(cohort):
        raise ValueError("Table 2 manifest self-hash mismatch")
    if file_sha != EXPECTED_TABLE2_MANIFEST_FILE_SHA256:
        raise ValueError("Table 2 manifest file hash mismatch")
    if cohort.get("manifest_content_sha256") != EXPECTED_TABLE2_MANIFEST_CONTENT_SHA256:
        raise ValueError("Table 2 manifest content hash mismatch")
    if (
        cohort.get("dataset") != DATASET_LABEL
        or cohort.get("classification") != "FORMAL"
        or cohort.get("mode") != "formal"
    ):
        raise ValueError("cohort manifest must be the formal Ours-500K Table 2 run")
    source = cohort.get("source", {})
    if source.get("archive_binding", {}).get("archive_sha256") != EXPECTED_ARCHIVE_SHA256:
        raise ValueError("Table 2 archive binding mismatch")
    rows = cohort.get("assets")
    if not isinstance(rows, list) or len(rows) != SAMPLE_SIZE:
        raise ValueError("Table 2 cohort asset count mismatch")
    return {
        "file_sha256": file_sha,
        "content_sha256": cohort["manifest_content_sha256"],
        "source": source,
        "rows": sorted(rows, key=lambda row: int(row["selection_index"])),
    }


def build_frozen_items(
    cohort_rows: list[dict[str, Any]],
    audits: dict[str, dict[str, Any]],
    runtime_identity: dict[str, Any],
) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for order, row in enumerate(cohort_rows):
        asset_id = str(row["asset_id"])
        audit = audits[asset_id]
        movable = int(audit["movable_dof_count"])
        item = {
            "protocol_id": PROTOCOL_ID,
            "order": order,
            "dataset_id": asset_id,
            "asset_id": asset_id,
            "raw_category": str(row["raw_category"]),
            "seed_name": str(row["seed_name"]),
            "asset_root_relpath": str(row["asset_root"]),
            "selection_rank": int(row["selection_index"]) + 1,
            "category": str(row["raw_category"]),
            "package_audit_success": bool(audit["package_audit_success"]),
            "audit_issue": audit["audit_issue"],
            "primary_urdf_relpath": audit["primary_urdf_relpath"],
            "urdf_sha256": audit["urdf_sha256"],
            "valid_tree": bool(audit["valid_tree"]),
            "movable_dof_count": movable,
            "range_evaluable_dof_count": int(audit["range_evaluable_dof_count"]),
            "joint_specs_sha256": audit["joint_specs_sha256"],
            "collision_mesh_files": audit["collision_mesh_files"],
            "collision_mesh_inventory_sha256": audit[
                "collision_mesh_inventory_sha256"
            ],
            "missing_collision_mesh_reference_count": int(
                audit["missing_collision_mesh_reference_count"]
            ),
            "unsafe_collision_mesh_reference_count": int(
                audit["unsafe_collision_mesh_reference_count"]
            ),
            "scale_derivation": audit["scale_derivation"],
            "scale_derivation_sha256": audit["scale_derivation_sha256"],
            "object_bbox_diagonal_m": audit["object_bbox_diagonal_m"],
            "rest_state_expected": 1,
            "single_state_expected": SINGLE_SAMPLES * movable,
            "sobol_state_expected": SOBOL_SAMPLES if movable > 0 else 0,
            "runtime_binding": runtime_identity,
        }
        item["input_identity_sha256"] = canonical_sha256(
            {key: item[key] for key in FROZEN_INPUT_FIELDS}
        )
        items.append(item)
    return items


def validate_frozen_source_snapshot(item: dict[str, Any], dataset_root: Path) -> None:
    dataset_root = dataset_root.resolve(strict=True)
    urdf_path = dataset_root / str(item["primary_urdf_relpath"])
    if not urdf_path.is_file() or urdf_path.is_symlink():
        raise FileNotFoundError("primary URDF missing or symlinked")
    if sha256_file(urdf_path) != item["urdf_sha256"]:
        raise ValueError("primary URDF changed after freeze")
    asset_root = (dataset_root / str(item["asset_root_relpath"])).resolve(strict=True)
    asset_root.relative_to(dataset_root)
    inventory, unsafe = collision_mesh_inventory(asset_root, urdf_path)
    if unsafe:
        raise ValueError(f"unsafe collision mesh reference count: {unsafe}")
    if canonical_sha256(inventory) != item["collision_mesh_inventory_sha256"]:
        raise ValueError("collision mesh inventory changed after freeze")


def evaluate_asset(item: dict[str, Any], dataset_root: Path) -> dict[str, Any]:
    import pybullet as bullet

    core = _load_core()
    result = core.failure_record(item, "evaluation_not_completed")
    result["issues"] = []
    result["state_records"] = []
    result["runner_sha256"] = sha256_file(SCRIPT)
    result["collision_core_sha256"] = sha256_file(CORE_SCRIPT)
    result["runtime_identity"] = current_runtime_identity()
    try:
        validate_frozen_source_snapshot(item, dataset_root)
    except Exception as exc:  # noqa: BLE001
        result["issues"] = [f"{type(exc).__name__}: {exc}"]
        return result
    if not item.get("package_audit_success"):
        result["issues"] = [str(item.get("audit_issue") or "package_audit_failed")]
        return result
    urdf_path = dataset_root.resolve(strict=True) / str(item["primary_urdf_relpath"])
    joints = core.parse_urdf_joints(urdf_path)
    result["movable_dof_count"] = len(joints)
    result["range_evaluable_dof_count"] = sum(
        bool(row["range_evaluable"]) for row in joints
    )
    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        body = bullet.loadURDF(
            str(urdf_path),
            useFixedBase=True,
            flags=flags,
            physicsClientId=client,
        )
        result["load_success"] = True
        simulator_by_name: dict[str, int] = {}
        for index in range(bullet.getNumJoints(body, physicsClientId=client)):
            info = bullet.getJointInfo(body, index, physicsClientId=client)
            name = info[1].decode("utf-8") if isinstance(info[1], bytes) else str(info[1])
            simulator_by_name[name] = index
        missing_names = [row["name"] for row in joints if row["name"] not in simulator_by_name]
        if missing_names:
            raise RuntimeError(f"simulator joint mapping missing: {missing_names}")
        joint_indices = [simulator_by_name[row["name"]] for row in joints]
        for index in joint_indices:
            bullet.setJointMotorControl2(
                body,
                index,
                controlMode=bullet.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
                physicsClientId=client,
            )
        direct_pairs = core._direct_parent_pairs(bullet, body, client)
        rest_values = [0.0] * len(joints)

        def observe(
            values: list[float],
            phase: str,
            sample_index: int,
            joint_name: str | None = None,
        ) -> dict[str, Any]:
            collision, readback_error = core._reset_and_observe(
                bullet,
                body,
                client,
                joint_indices,
                values,
                direct_pairs,
            )
            metric_key = (
                "all_pair_max_penetration_m"
                if phase == "rest"
                else "non_adjacent_max_penetration_m"
            )
            state = {
                "dataset_id": item["dataset_id"],
                **{key: item[key] for key in IDENTITY_FIELDS},
                "category": item["category"],
                "protocol_id": item["protocol_id"],
                "order": item["order"],
                "input_identity_sha256": item["input_identity_sha256"],
                "phase": phase,
                "sample_index": sample_index,
                "joint_name": joint_name,
                "joint_values_sha256": canonical_sha256(values),
                "reset_readback_max_abs_error": readback_error,
                "metric_max_penetration_m": float(collision[metric_key]),
                **collision,
            }
            result["state_records"].append(state)
            return state

        rest = observe(rest_values, "rest", 0)
        result["rest_state_executed"] = 1
        result["rest_all_pair_cf"] = rest["all_pair_illegal_penetration_count"] == 0
        result["rest_non_adjacent_cf"] = (
            rest["non_adjacent_illegal_penetration_count"] == 0
        )
        result["rest_non_adjacent_free"] = int(result["rest_non_adjacent_cf"])

        joint_sweep_passes = 0
        for joint_position, row in enumerate(joints):
            if not row["range_evaluable"]:
                result["issues"].append(f"joint_range_not_evaluable:{row['name']}")
                continue
            joint_free = True
            for sample_index, value in enumerate(core.single_joint_values(row)):
                values = list(rest_values)
                values[joint_position] = value
                state = observe(values, "single_joint_sweep", sample_index, str(row["name"]))
                result["single_state_executed"] += 1
                free = state["non_adjacent_illegal_penetration_count"] == 0
                result["single_non_adjacent_free"] += int(free)
                joint_free = joint_free and free
            joint_sweep_passes += int(joint_free)
        result["joint_single_sweep_cf_passed"] = joint_sweep_passes

        if joints and all(row["range_evaluable"] for row in joints):
            for sample_index, values in enumerate(core.sobol_joint_values(joints)):
                state = observe(values, "multi_joint_sobol", sample_index)
                result["sobol_state_executed"] += 1
                result["sobol_non_adjacent_free"] += int(
                    state["non_adjacent_illegal_penetration_count"] == 0
                )

        result["single_joint_sweep_cf"] = bool(
            result["single_state_executed"] == result["single_state_expected"]
            and result["single_non_adjacent_free"] == result["single_state_expected"]
        )
        result["multi_joint_sobol_cf"] = bool(
            result["movable_dof_count"] > 0
            and result["range_evaluable_dof_count"] == result["movable_dof_count"]
            and result["sobol_state_executed"] == result["sobol_state_expected"]
            and result["sobol_non_adjacent_free"] == result["sobol_state_expected"]
        )
        expected_total = sum(
            int(result[f"{phase}_state_expected"])
            for phase in ("rest", "single", "sobol")
        )
        executed_total = sum(
            int(result[f"{phase}_state_executed"])
            for phase in ("rest", "single", "sobol")
        )
        result["measurement_complete"] = bool(
            result["range_evaluable_dof_count"] == result["movable_dof_count"]
            and executed_total == expected_total
        )
        result["strict_collision_pass"] = bool(
            result["measurement_complete"]
            and result["rest_non_adjacent_cf"]
            and result["single_joint_sweep_cf"]
            and result["multi_joint_sobol_cf"]
        )
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"{type(exc).__name__}: {exc}")
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)
    states = result["state_records"]
    if states:
        result["max_penetration_m"] = max(
            float(state["metric_max_penetration_m"]) for state in states
        )
        result["max_penetration_normalized"] = (
            float(result["max_penetration_m"])
            / float(result["object_bbox_diagonal_m"])
        )
        result["max_reset_readback_error"] = max(
            float(state["reset_readback_max_abs_error"]) for state in states
        )
    result["state_records_sha256"] = canonical_sha256(states)
    return result


def run_child(item_path: Path, dataset_root: Path, result_path: Path) -> int:
    item = json.loads(item_path.resolve(strict=True).read_text(encoding="utf-8"))
    binding = item.get("runtime_binding") or {}
    if binding.get("python_executable") != sys.executable:
        print(
            f"FATAL_RUNTIME_BINDING: python {sys.executable} != {binding.get('python_executable')}",
            file=sys.stderr,
        )
        return 86
    result = evaluate_asset(item, dataset_root)
    atomic_json(result_path, result)
    return 0


def verify_result_against_item(item: dict[str, Any], result: dict[str, Any]) -> None:
    if result.get("dataset_id") != item["dataset_id"]:
        raise ValueError("result dataset_id mismatch")
    if result.get("input_identity_sha256") != item["input_identity_sha256"]:
        raise ValueError("result input identity mismatch")
    states = result.get("state_records") or []
    counters: dict[str, dict[str, int]] = {
        "rest": {"executed": 0, "free": 0},
        "single_joint_sweep": {"executed": 0, "free": 0},
        "multi_joint_sobol": {"executed": 0, "free": 0},
    }
    for state in states:
        phase = str(state.get("phase"))
        if phase not in counters:
            raise ValueError(f"unknown state phase: {phase}")
        counters[phase]["executed"] += 1
        if int(state.get("non_adjacent_illegal_penetration_count", -1)) == 0:
            counters[phase]["free"] += 1
    if counters["rest"]["executed"] != int(result.get("rest_state_executed", -1)):
        raise ValueError("rest executed counter mismatch")
    if counters["single_joint_sweep"]["executed"] != int(result.get("single_state_executed", -1)):
        raise ValueError("single executed counter mismatch")
    if counters["multi_joint_sobol"]["executed"] != int(result.get("sobol_state_executed", -1)):
        raise ValueError("sobol executed counter mismatch")
    if counters["single_joint_sweep"]["free"] != int(result.get("single_non_adjacent_free", -1)):
        raise ValueError("single free counter mismatch")
    if counters["multi_joint_sobol"]["free"] != int(result.get("sobol_non_adjacent_free", -1)):
        raise ValueError("sobol free counter mismatch")
    rest_free_expected = int(result.get("rest_non_adjacent_free", -1))
    if counters["rest"]["free"] != rest_free_expected:
        raise ValueError("rest free counter mismatch")
    if states:
        recomputed_max = max(
            float(state["metric_max_penetration_m"]) for state in states
        )
        if abs(recomputed_max - float(result.get("max_penetration_m", float("nan")))) > 0:
            raise ValueError("max penetration mismatch")
    recomputed_hash = canonical_sha256(states)
    if recomputed_hash != result.get("state_records_sha256"):
        raise ValueError("state_records hash mismatch")
    expected_total = (
        int(result["rest_state_expected"])
        + int(result["single_state_expected"])
        + int(result["sobol_state_expected"])
    )
    executed_total = (
        int(result["rest_state_executed"])
        + int(result["single_state_executed"])
        + int(result["sobol_state_executed"])
    )
    if executed_total > expected_total:
        raise ValueError("executed states exceed frozen expectation")


def run_one_subprocess(
    item: dict[str, Any],
    scratch: Path,
    timeout_seconds: float,
) -> dict[str, Any]:
    core = _load_core()
    job_root = Path(tempfile.mkdtemp(prefix="job_", dir=scratch))
    item_path = job_root / "item.json"
    result_path = job_root / "result.json"
    atomic_json(item_path, item)
    environment = dict(os.environ)
    environment.update(
        {
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        }
    )
    process = subprocess.Popen(
        [
            sys.executable,
            str(SCRIPT),
            "--internal-child",
            str(item_path),
            str(DATASET_ROOT),
            str(result_path),
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        start_new_session=True,
        env=environment,
    )
    import shutil as _shutil

    try:
        try:
            _stdout, stderr = process.communicate(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.communicate(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate()
            result = core.failure_record(item, f"child_timeout_after_{timeout_seconds:g}s", timed_out=True)
            result["runner_sha256"] = sha256_file(SCRIPT)
            result["collision_core_sha256"] = sha256_file(CORE_SCRIPT)
            result["runtime_identity"] = current_runtime_identity()
            return result
        if process.returncode != 0 or not result_path.is_file():
            detail = (stderr or b"").decode("utf-8", errors="replace")[-4000:]
            result = core.failure_record(
                item, f"child_exit_{process.returncode}: {detail}"
            )
            result["runner_sha256"] = sha256_file(SCRIPT)
            result["collision_core_sha256"] = sha256_file(CORE_SCRIPT)
            result["runtime_identity"] = current_runtime_identity()
            return result
        return json.loads(result_path.read_text(encoding="utf-8"))
    finally:
        _shutil.rmtree(job_root, ignore_errors=True)


def summarize(
    manifest: dict[str, Any], records: list[dict[str, Any]]
) -> dict[str, Any]:
    core = _load_core()
    summary = core.summarize_records(
        {
            "sample_size": len(records),
            "protocol_id": PROTOCOL_ID,
            "cohort_label": manifest["cohort_label"],
        },
        records,
    )
    summary["metrics"]["max_penetration"]["normalization"] = (
        "PyBullet q0 collision-shape union AABB diagonal "
        f"({SCALE_PROTOCOL})"
    )
    summary["dataset"] = DATASET_LABEL
    summary["n_release"] = EXPECTED_N_RELEASE
    summary["n_eval"] = len(records)
    summary["manifest_content_sha256"] = manifest["manifest_content_sha256"]
    summary["completed_at"] = utc_now()
    return summary


def report_text(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    metrics = summary["metrics"]

    def pct(row: dict[str, Any], key: str = "passed") -> str:
        if row.get("rate") is None:
            return "N/E"
        return f"{row[key]} / {row['denominator']} ({100 * row['rate']:.3f}%)"

    csr = metrics["collision_state_rate"]
    cfr = metrics["collision_free_range"]
    maxp = metrics["max_penetration"]
    lines = [
        "# Ours-500K Table 4: Collision and Mechanical Clearance",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        f"Frozen cohort: N_eval={summary['n_eval']} (full acquired roster, Table 2 manifest order); "
        f"rest q=0; single-joint K={SINGLE_SAMPLES}; Sobol R={SOBOL_SAMPLES} (seed {SOBOL_SEED}); "
        f"penetration threshold {PENETRATION_THRESHOLD_M} m; scale protocol `{SCALE_PROTOCOL}`.",
        "",
        "| Metric | Result |",
        "|---|---:|",
        f"| Rest All-pair CF | {pct(metrics['rest_all_pair_cf'])} |",
        f"| Rest Non-adjacent CF | {pct(metrics['rest_non_adjacent_cf'])} |",
        f"| Single-joint Sweep CF | {pct(metrics['single_joint_sweep_cf'])} |",
        f"| Multi-joint Sobol CF | {pct(metrics['multi_joint_sobol_cf'])} |",
        (
            f"| Collision-state Rate | {csr['collision_states']} / {csr['denominator']} "
            f"({100 * csr['rate']:.3f}%) |"
        ),
        "| AOR | N/E |",
        (
            f"| Max Penetration | {maxp['maximum_observed_normalized']:.6f} "
            f"({maxp['fully_measured_assets']} / {maxp['denominator']} measured; {maxp['status']}) |"
            if maxp["maximum_observed_normalized"] is not None
            else "| Max Penetration | N/E |"
        ),
        (
            f"| Collision-free Range | {cfr['passed_states']} / {cfr['denominator']} "
            f"({100 * cfr['rate']:.3f}%) |"
        ),
        f"| Strict Collision Pass | {pct(metrics['strict_collision_pass'])} |",
        "",
        "Collision-state Rate uses the fail-closed denominator: unexecuted states count as non-free.",
        "AOR is N/E because no stable exact overlap-volume implementation was run; bounding-box overlap is not substituted.",
        "Discrete sweeps do not constitute CCD, joint semantic correctness, or dynamics validity.",
        "",
    ]
    return "\n".join(lines)


def run(args: argparse.Namespace) -> Path:
    if args.workers <= 0:
        raise ValueError("--workers must be positive")
    table2_manifest_path = args.table2_manifest.resolve(strict=True)
    cohort = load_cohort(table2_manifest_path)
    rows = cohort["rows"] if args.limit is None else cohort["rows"][: args.limit]

    if args.output is None:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        suffix = f"smoke_n{args.limit}" if args.limit is not None else "n500"
        args.output = REPO / f"exp/runtime/urdf_table4_ours_500k_table2_{suffix}_{timestamp}"
    output = args.output.resolve(strict=False)
    if args.resume:
        if not output.is_dir():
            raise FileNotFoundError(f"resume output does not exist: {output}")
    else:
        output.mkdir(parents=True, exist_ok=False)

    manifest_path = output / "frozen_manifest.json"
    records_path = output / "asset_records.jsonl"
    checkpoint_path = output / "checkpoint.json"
    scratch = output / ".worker_scratch"
    scratch.mkdir(exist_ok=True)
    core = _load_core()

    if args.resume:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
            raise RuntimeError("resume manifest self-hash mismatch")
    else:
        # Phase A: freeze protocol snapshot, then audit all assets (no result inspection).
        snapshot = output / PROTOCOL_SNAPSHOT_NAME
        snapshot.write_bytes(PROTOCOL_DOCUMENT.read_bytes())

        runtime_identity = current_runtime_identity()
        audits: dict[str, dict[str, Any]] = {}
        if args.audit_workers <= 1:
            for row in rows:
                audits[str(row["asset_id"])] = audit_asset(DATASET_ROOT, row)
        else:
            with ThreadPoolExecutor(max_workers=args.audit_workers) as executor:
                futures = {
                    executor.submit(audit_asset, DATASET_ROOT, row): row for row in rows
                }
                for future in as_completed(futures):
                    row = futures[future]
                    audits[str(row["asset_id"])] = future.result()
        items = build_frozen_items(rows, audits, runtime_identity)
        manifest = {
            "schema_version": "table4_ours_500k_frozen_manifest_v1",
            "created_at": utc_now(),
            "dataset": DATASET_LABEL,
            "classification": "FORMAL" if args.limit is None else "SMOKE",
            "cohort_label": (
                "Ours-500K full acquired roster (Table 2 formal manifest order)"
            ),
            "protocol_id": PROTOCOL_ID,
            "protocol_document_at_freeze_sha256": sha256_file(snapshot),
            "source": {
                "dataset_root": str(DATASET_ROOT),
                "table2_manifest_path": str(table2_manifest_path),
                "table2_manifest_file_sha256": cohort["file_sha256"],
                "table2_manifest_content_sha256": cohort["content_sha256"],
                "archive_sha256": EXPECTED_ARCHIVE_SHA256,
                "cohort_type": str(cohort["source"].get("cohort_type", "")),
                "n_release": EXPECTED_N_RELEASE,
                "n_eval": len(items),
                "category_count": len({item["category"] for item in items}),
            },
            "evaluation": {
                "protocol_path": str(PROTOCOL_DOCUMENT),
                "adapter_path": str(SCRIPT),
                "adapter_sha256": sha256_file(SCRIPT),
                "core_path": str(CORE_SCRIPT),
                "core_sha256": sha256_file(CORE_SCRIPT),
                "child_python": sys.executable,
                "runtime_identity": runtime_identity,
                "single_samples": SINGLE_SAMPLES,
                "sobol_samples": SOBOL_SAMPLES,
                "sobol_seed": SOBOL_SEED,
                "penetration_threshold_m": PENETRATION_THRESHOLD_M,
                "reset_tolerance": RESET_TOLERANCE,
                "scale_protocol": SCALE_PROTOCOL,
                "pair_policy": (
                    "all-pair and non-adjacent (direct parent-child excluded); "
                    "surface contact allowed; penetration > threshold illegal; "
                    "no method-specific allowlist"
                ),
                "rest_state_policy": "q=0 for every evaluable joint",
                "child_timeout_seconds": args.child_timeout_seconds,
                "workers": args.workers,
            },
            "items": items,
            "selection": {
                "algorithm": "exact Table 2 manifest .assets[] order; optional smoke prefix only",
                "selected_asset_ids_sha256": canonical_sha256(
                    [item["asset_id"] for item in items]
                ),
                "ordered_identities_sha256": canonical_sha256(
                    [
                        {key: item[key] for key in FROZEN_INPUT_FIELDS}
                        for item in items
                    ]
                ),
            },
        }
        manifest["manifest_content_sha256"] = _manifest_self_hash(manifest)
        atomic_json(manifest_path, manifest)
        records_path.touch(exist_ok=False)
        atomic_json(
            checkpoint_path,
            {
                "state": "frozen",
                "completed": 0,
                "remaining": len(items),
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "updated_at": utc_now(),
            },
        )

    items = manifest["items"]
    n_eval = len(items)
    existing = []
    if records_path.is_file():
        existing = [
            json.loads(line) for line in records_path.read_text().splitlines() if line
        ]
    by_key: dict[str, dict[str, Any]] = {}
    for record in existing:
        key = str(record.get("dataset_id"))
        if key in by_key:
            raise RuntimeError(f"duplicate resume record: {key}")
        if record.get("input_identity_sha256") != next(
            item["input_identity_sha256"] for item in items if item["asset_id"] == key
        ):
            raise RuntimeError(f"resume record binding mismatch: {key}")
        by_key[key] = record
    pending = [item for item in items if item["asset_id"] not in by_key]

    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(
                run_one_subprocess, item, scratch, args.child_timeout_seconds
            ): item
            for item in pending
        }
        for future in as_completed(futures):
            item = futures[future]
            try:
                result = future.result()
            except Exception as exc:  # noqa: BLE001
                result = core.failure_record(
                    item, f"parent_worker_exception: {type(exc).__name__}: {exc}"
                )
                result["runner_sha256"] = sha256_file(SCRIPT)
                result["collision_core_sha256"] = sha256_file(CORE_SCRIPT)
            verify_result_against_item(item, result)
            by_key[item["asset_id"]] = result
            with records_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(result, ensure_ascii=False, sort_keys=True) + "\n")
            completed = len(by_key)
            atomic_json(
                checkpoint_path,
                {
                    "state": "running" if completed < n_eval else "aggregating",
                    "completed": completed,
                    "remaining": n_eval - completed,
                    "last_completed_asset": item["asset_id"],
                    "manifest_content_sha256": manifest["manifest_content_sha256"],
                    "updated_at": utc_now(),
                },
            )
            print(f"[{completed}/{n_eval}] {item['asset_id']} done", flush=True)

    ordered = [by_key[item["asset_id"]] for item in items]
    summary = summarize(manifest, ordered)
    atomic_json(output / "summary.json", summary)
    atomic_text(output / "report.md", report_text(summary, manifest))
    state_rows = []
    for result in ordered:
        state_rows.extend(result.get("state_records") or [])
    atomic_jsonl(output / "state_records.jsonl", state_rows)
    verification = verify_run(manifest, ordered, summary)
    atomic_json(output / "verification.json", verification)
    atomic_json(
        checkpoint_path,
        {
            "state": "complete",
            "completed": n_eval,
            "remaining": 0,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "updated_at": utc_now(),
        },
    )
    return output


def verify_run(
    manifest: dict[str, Any],
    records: list[dict[str, Any]],
    summary: dict[str, Any],
) -> dict[str, Any]:
    checks: dict[str, bool] = {}
    checks["record_count_matches_frozen_items"] = len(records) == len(manifest["items"])
    checks["record_order_matches_frozen_items"] = all(
        record["dataset_id"] == item["asset_id"]
        for record, item in zip(records, manifest["items"])
    )
    checks["input_identity_bindings_match"] = all(
        record.get("input_identity_sha256") == item["input_identity_sha256"]
        for record, item in zip(records, manifest["items"])
    )
    for record, item in zip(records, manifest["items"]):
        verify_result_against_item(item, record)
    checks["state_counter_replay_all_records"] = True
    expected_states = sum(
        int(item["rest_state_expected"])
        + int(item["single_state_expected"])
        + int(item["sobol_state_expected"])
        for item in manifest["items"]
    )
    executed_states = sum(
        int(record["rest_state_executed"])
        + int(record["single_state_executed"])
        + int(record["sobol_state_executed"])
        for record in records
    )
    checks["executed_states_le_expected"] = executed_states <= expected_states
    metrics = summary["metrics"]
    recomputed_strict = sum(
        bool(record["strict_collision_pass"]) for record in records
    )
    checks["strict_pass_consistent"] = (
        metrics["strict_collision_pass"]["passed"] == recomputed_strict
    )
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "expected_states": expected_states,
        "executed_states": executed_states,
        "verified_at": utc_now(),
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--table2-manifest", type=Path, default=DEFAULT_TABLE2_MANIFEST)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--audit-workers", type=int, default=AUDIT_WORKERS)
    parser.add_argument(
        "--child-timeout-seconds", type=float, default=CHILD_TIMEOUT_SECONDS
    )
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--internal-child", nargs=3, default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.internal_child is not None:
        item_path, dataset_root, result_path = args.internal_child
        return run_child(Path(item_path), Path(dataset_root), Path(result_path))
    output = run(args)
    print(json.dumps({"status": "completed", "output": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
