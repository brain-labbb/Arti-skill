#!/usr/bin/env python3
"""Independently verify a published SketchMobility Table 5 result."""

from __future__ import annotations

import argparse
from collections import Counter
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
from typing import Any, Iterable, Mapping
import xml.etree.ElementTree as ET


SIMULATORS = ("pybullet", "genesis", "mujoco")
SIMULATOR_PAIRS = (
    ("pybullet", "genesis"),
    ("pybullet", "mujoco"),
    ("genesis", "mujoco"),
)
METRICS = (
    "load",
    "reset",
    "settling",
    "actuation",
    "limit_enforcement",
    "constraint_drift",
    "simulator_pass",
)
AND_METRICS = METRICS[:-1]
TABLE5A_FIELDS = (*METRICS, "strict_collision_pass")
TABLE5B_RATE_FIELDS = (
    "per_simulator_pass",
    "all_three_load",
    "all_three_runtime_pass",
    "strict_urdf_pass",
    "strict_kinematic_pass",
    "strict_collision_pass",
    "strict_consistency",
    "strict_sim_ready",
)
TERMINAL_STATUSES = {
    "completed",
    "preflight_failure",
    "diagnostic_failure",
    "timeout",
    "native_crash",
    "missing_response",
    "malformed_response",
    "worker_error",
    "parent_error",
}
ADAPTER_NAMES = {
    "pybullet": "PyBulletAdapter",
    "genesis": "GenesisAdapter",
    "mujoco": "MuJoCoAdapter",
}
REPO_ROOT = Path(__file__).resolve().parents[2]
CANONICAL_PROTOCOL_PATH = (
    REPO_ROOT / "exp/reference/table5_sketch_mobility_n800_protocol_v1.json"
)
CANONICAL_PROTOCOL_FILE_SHA256 = (
    "32632da34a5830b62e49dbafff081f346ba83c0ba9f38d99e6f2a6f7a53a3325"
)
FORMAL_TABLE1_MANIFEST_SHA256 = (
    "081e9e9125f8945cad67a751949e659f6d4e73817704c07cd3fcd4b657ffc696"
)
FORMAL_RELEASE_MANIFEST_SHA256 = (
    "5b4b0891bafeba5029e5e1dd71042e5be8543eccb2990edfbc9aba71a1ac56fb"
)
FORMAL_RELEASE_UNIVERSE_SHA256 = (
    "a9c128d24ab9cba03d593ecff17f1e7284ef1f74f09f69939cbef7c0cc8af346"
)
FORMAL_ORDERED_IDENTITY_SHA256 = (
    "f7cd81b2c6ef85c915582a911e3040fbc330a90e626aafdf71a9c5919ab1402f"
)
FORMAL_ORDERED_MANIFEST_ROOT_SHA256 = (
    "a88506e1da8e7e8b61a740965dea2faba4e9ab8280f47417e17550024b6dde17"
)
FORMAL_ORDERED_PACKAGE_BINDING_SHA256 = (
    "5fa3622502d74feacffd327b61c7a43f7c30d6d6109d4439d79651a39a39805d"
)
FORMAL_ORDERED_URDF_BINDING_SHA256 = (
    "4fee367147d3f83482ee34959723bf12539c22472153b10dce252d36967d2f86"
)
FORMAL_COHORT_SIZE = 800
FORMAL_PACKAGE_FILE_COUNT = 31403
FORMAL_PACKAGE_TOTAL_BYTES = 2510671298
FORMAL_PREFLIGHT_FAILURES = 489
FORMAL_CATEGORY_COUNT = 67
AGGREGATE_FILES = {
    "aggregate_set.json",
    "table5.json",
    "failure_inventory.json",
    "report.md",
    "self_check.json",
}
AGGREGATE_HASH_FILES = {
    "table5.json",
    "failure_inventory.json",
    "report.md",
    "self_check.json",
}
SELF_CHECK_HASH_FILES = {"table5.json", "failure_inventory.json", "report.md"}
SELF_CHECK_NAMES = {
    "phase_separation",
    "intent_unique_selected",
    "formal_full_cohort",
    "full_n_denominators",
    "all_three_denominators",
    "metric_numerators_recomputed",
    "intersections_recomputed",
    "strict_rates_recomputed",
    "strict_formula_recomputed",
    "asset_outcomes_recomputed",
    "category_reconciliation",
    "category_micro_reconciles",
    "category_macro_reconciles",
    "category_metadata_reconciles",
    "inventory_reasons_complete",
    "finite_json",
}
IDENTITY_FIELDS = {
    "dataset_id",
    "asset_id",
    "manifest_root",
    "order",
    "selection_rank",
    "selection_hash",
    "simulator",
    "adapter_name",
    "adapter_version",
    "adapter_config_sha256",
    "adapter_implementation_sha256",
    "parent_gpu_hardware_sha256",
    "urdf_sha256",
    "package_content_manifest_sha256",
    "effective_workers",
    "protocol_sha256",
    "cohort_sha256",
    "manifest_row_sha256",
}
TERMINAL_FIELDS = {
    "schema_version",
    "run_phase",
    "terminal",
    "terminal_status",
    "identity",
    "provenance",
    "process",
    "source_contract",
    "load",
    "support",
    "metrics",
    "diagnostics",
}
IMPLEMENTATION_RECEIPT_FIELDS = {
    "schema_version",
    "simulator",
    "distribution_name",
    "distribution_version",
    "distribution_metadata_sha256",
    "distribution_record_sha256",
    "direct_url",
    "direct_url_sha256",
    "module_name",
    "module_origin",
    "module_origin_sha256",
    "evaluator_source",
    "evaluator_source_sha256",
    "git",
}
PROVENANCE_FIELDS = {
    "urdf_relative_path",
    "manifest_row_sha256",
    "adapter_implementation_receipt",
    "device_receipt",
    "parent_gpu_receipt",
    "source_binding_receipt",
}
RUNTIME_INPUT_FIELDS = {
    "present",
    "effective_workers",
    "intent_count",
    "terminal_count",
    "complete",
    "record_file_hashes",
    "record_set_sha256",
    "summary_sha256",
    "progress_sha256",
    "adapter_implementation_sha256",
}
FROZEN_GENESIS_GPU_HARDWARE = {
    "physical_device_index": 3,
    "gpu_uuid": "GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7",
    "device_name": "NVIDIA L20X",
    "total_memory_mib": 143771,
    "driver_version": "570.172.08",
    "compute_capability": "8.9",
}
FROZEN_GENESIS_GPU_BINDING = {
    "cuda_visible_devices": "3",
    "physical_device_index": 3,
    "visible_device_index": 0,
    "gpu_uuid": "GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7",
}
FROZEN_GENESIS_TORCH_DEVICE = {
    "torch_device_name": "NVIDIA L20X",
    "torch_total_memory_bytes": 150121021440,
    "torch_total_memory_mib": 143166,
    "torch_compute_capability": "9.0",
}
FROZEN_GENESIS_SOFTWARE = {
    "torch_version": "2.8.0+cu128",
    "torch_cuda_version": "12.8",
    "cudnn_version": 91002,
    "quadrants_version": "1.2.0",
    "python_version": "3.12.13",
}
TAIL_LIMIT = 8192
WARNING_ENTRY_LIMIT = 100
WARNING_MESSAGE_LIMIT = 1024
PYBULLET_MAX_ACTUAL_STATE_JOINTS = 127
MANIFEST_ROW_FIELDS = {
    "dataset_id",
    "order",
    "asset_id",
    "manifest_root",
    "raw_category",
    "category",
    "source",
    "selection_hash",
    "selection_rank",
    "package_relative_path",
    "package_binding",
    "package_content_manifest_sha256",
    "urdf_relative_path",
    "urdf_sha256",
    "link_names",
    "joint_names",
    "joints",
    "scalar_joints",
    "joint_tree",
    "joint_tree_issue",
    "resources",
    "resource_sha256",
    "xml_counts",
    "collision",
    "inertia",
    "visual",
    "bounding_box",
    "bounding_box_diagonal",
    "strict_gates",
    "preflight",
    "row_sha256",
}


class VerificationContractError(ValueError):
    """Raised when a publication violates the independent contract."""


def canonical_json_bytes(
    value: Any, *, exclude_fields: Iterable[str] = ()
) -> bytes:
    excluded = set(exclude_fields)

    def filtered(item: Any) -> Any:
        if isinstance(item, dict):
            return {
                key: filtered(child)
                for key, child in item.items()
                if key not in excluded
            }
        if isinstance(item, list):
            return [filtered(child) for child in item]
        return item

    return json.dumps(
        filtered(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def canonical_sha256(
    value: Any, *, exclude_fields: Iterable[str] = ()
) -> str:
    return hashlib.sha256(
        canonical_json_bytes(value, exclude_fields=exclude_fields)
    ).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _read_json_object(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise VerificationContractError(f"not a regular JSON file: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise VerificationContractError(f"malformed JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise VerificationContractError(f"expected JSON object: {path}")
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise VerificationContractError(f"non-finite JSON {path}: {error}") from error
    return value


def _rate(passed: int, denominator: int) -> dict[str, int | float]:
    return {
        "passed": passed,
        "denominator": denominator,
        "percentage": 0.0 if denominator == 0 else 100.0 * passed / denominator,
    }


def _canonical_protocol_errors(protocol: Any) -> list[str]:
    errors: list[str] = []
    try:
        if sha256_file(CANONICAL_PROTOCOL_PATH) != CANONICAL_PROTOCOL_FILE_SHA256:
            errors.append("canonical protocol source file hash mismatch")
        canonical = _read_json_object(CANONICAL_PROTOCOL_PATH)
        expected = copy.deepcopy(canonical)
        expected["protocol_sha256"] = canonical_sha256(
            expected, exclude_fields={"protocol_sha256", "generated_at"}
        )
        if protocol != expected:
            errors.append("published protocol differs from the canonical protocol")
        if not isinstance(protocol, dict) or protocol.get(
            "protocol_sha256"
        ) != canonical_sha256(
            protocol, exclude_fields={"protocol_sha256", "generated_at"}
        ):
            errors.append("published protocol semantic self-hash mismatch")
    except (OSError, TypeError, ValueError, KeyError) as error:
        errors.append(f"canonical protocol closure failed: {error}")
    return errors


def _phase_intent(
    *, phase: str, table5: Mapping[str, Any], rows: list[dict[str, Any]]
) -> tuple[list[str], list[dict[str, Any]]]:
    if phase not in {"qualification", "formal"}:
        raise VerificationContractError(f"invalid phase: {phase}")
    row_ids = [row.get("dataset_id") for row in rows]
    if any(not isinstance(dataset_id, str) for dataset_id in row_ids):
        raise VerificationContractError("manifest contains malformed dataset IDs")
    row_by_id = {row["dataset_id"]: row for row in rows}
    if len(row_by_id) != len(rows):
        raise VerificationContractError("manifest contains duplicate dataset IDs")
    intent = table5.get("intent")
    if not isinstance(intent, Mapping) or set(intent) != {
        "count",
        "dataset_ids",
        "manifest_roots",
    }:
        raise VerificationContractError("Table 5 intent fields are malformed")
    ids = intent.get("dataset_ids")
    roots = intent.get("manifest_roots")
    if (
        not isinstance(ids, list)
        or not ids
        or not all(isinstance(dataset_id, str) for dataset_id in ids)
        or len(ids) != len(set(ids))
        or intent.get("count") != len(ids)
        or not isinstance(roots, list)
        or len(roots) != len(ids)
    ):
        raise VerificationContractError("Table 5 intent roster is malformed")
    if any(dataset_id not in row_by_id for dataset_id in ids):
        raise VerificationContractError("Table 5 intent contains an unknown dataset ID")
    selected = set(ids)
    ordered = [dataset_id for dataset_id in row_ids if dataset_id in selected]
    if ids != ordered:
        raise VerificationContractError("Table 5 intent is not in manifest order")
    intent_rows = [row_by_id[dataset_id] for dataset_id in ids]
    if roots != [row.get("manifest_root") for row in intent_rows]:
        raise VerificationContractError("Table 5 intent root binding mismatch")
    if phase == "formal" and (
        len(rows) != FORMAL_COHORT_SIZE or ids != row_ids
    ):
        raise VerificationContractError("formal intent is not the full N=800 cohort")
    return ids, intent_rows


def _collect_tree_files(root: Path) -> list[dict[str, Any]]:
    resolved = Path(root).resolve(strict=True)
    if not resolved.is_dir():
        raise VerificationContractError(f"not a directory: {root}")
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(
        resolved, followlinks=False
    ):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            if (current / name).is_symlink():
                raise VerificationContractError(
                    f"directory symlink in source tree: {current / name}"
                )
        for name in file_names:
            path = current / name
            if path.is_symlink() or not path.is_file():
                raise VerificationContractError(
                    f"non-regular file in source tree: {path}"
                )
            canonical = path.resolve(strict=True)
            canonical.relative_to(resolved)
            files.append(
                {
                    "path": canonical.relative_to(resolved).as_posix(),
                    "bytes": canonical.stat().st_size,
                    "sha256": sha256_file(canonical),
                }
            )
    return files


def _package_binding(package: Path) -> dict[str, Any]:
    files = _collect_tree_files(package)
    return {
        "file_count": len(files),
        "total_bytes": sum(item["bytes"] for item in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def _artifact_set(root: Path) -> dict[str, Any]:
    resolved = Path(root).resolve(strict=True)
    files = _collect_tree_files(resolved)
    return {
        "root": str(resolved),
        "file_count": len(files),
        "total_bytes": sum(item["bytes"] for item in files),
        "files": files,
        "artifact_set_sha256": canonical_sha256(files),
    }


def _read_jsonl_by_id(path: Path, key: str) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as error:
                raise VerificationContractError(
                    f"malformed JSONL {path}:{line_number}: {error}"
                ) from error
            if not isinstance(value, dict) or not isinstance(value.get(key), str):
                raise VerificationContractError(
                    f"malformed JSONL identity {path}:{line_number}"
                )
            identity = value[key]
            if identity in result:
                raise VerificationContractError(
                    f"duplicate JSONL identity {identity!r} in {path}"
                )
            result[identity] = value
    return result


def _upstream_gate_evidence(
    upstream: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    table2 = upstream["table2"]
    table3 = upstream["table3"]
    table4 = upstream["table4"]
    metrics = table2.get("metrics")
    subgates: dict[str, bool] = {}
    if not isinstance(metrics, Mapping):
        raise VerificationContractError("Table 2 metrics are missing")
    for name in (
        "parse_rate",
        "resource_resolution",
        "finite_fields",
        "valid_tree",
        "valid_joint_spec",
        "collision_coverage",
        "inertial_coverage",
        "inertia_validity",
    ):
        metric = metrics.get(name)
        if not isinstance(metric, Mapping) or not isinstance(metric.get("pass"), bool):
            raise VerificationContractError(f"Table 2 gate is malformed: {name}")
        subgates[name] = metric["pass"]
    for value, field, label in (
        (table2, "strict_urdf_pass", "Table 2"),
        (table3, "strict_kinematic_pass", "Table 3"),
        (table3, "tree_valid", "Table 3"),
        (table4, "strict_collision_pass", "Table 4"),
        (table4, "load_success", "Table 4"),
        (table4, "measurement_complete", "Table 4"),
    ):
        if not isinstance(value.get(field), bool):
            raise VerificationContractError(f"{label} {field} is not boolean")
    return {
        "table2": {
            "strict_urdf_pass": table2["strict_urdf_pass"],
            "subgates": subgates,
            "record_sha256": canonical_sha256(table2),
        },
        "table3": {
            "strict_kinematic_pass": table3["strict_kinematic_pass"],
            "tree_valid": table3["tree_valid"],
            "record_sha256": canonical_sha256(table3),
        },
        "table4": {
            "strict_collision_pass": table4["strict_collision_pass"],
            "load_success": table4["load_success"],
            "measurement_complete": table4["measurement_complete"],
            "record_sha256": canonical_sha256(table4),
        },
    }


def _xml_float_attribute(element: ET.Element | None, name: str) -> float | None:
    if element is None or element.get(name) is None:
        return None
    try:
        value = float(element.get(name, ""))
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def _xml_vector_attribute(
    element: ET.Element | None,
    name: str,
    default: tuple[float, float, float],
) -> list[float]:
    text = element.get(name) if element is not None else None
    if text is None:
        return list(default)
    try:
        values = [float(item) for item in text.split()]
    except ValueError as error:
        raise VerificationContractError(f"invalid URDF {name} vector") from error
    if len(values) != 3 or not all(math.isfinite(item) for item in values):
        raise VerificationContractError(f"invalid URDF {name} vector")
    return values


def _xml_normalized(values: list[float]) -> list[float]:
    magnitude = math.sqrt(sum(item * item for item in values))
    if not math.isfinite(magnitude) or magnitude <= 0:
        raise VerificationContractError("joint axis must be finite and non-zero")
    return [item / magnitude for item in values]


def _xml_joint_tree(
    link_names: list[str], joints: list[dict[str, Any]]
) -> dict[str, Any]:
    if (
        not link_names
        or any(not name for name in link_names)
        or len(set(link_names)) != len(link_names)
    ):
        raise VerificationContractError("link names must be unique and non-empty")
    links = set(link_names)
    child_links: set[str] = set()
    joint_names: set[str] = set()
    for joint in joints:
        if not joint["name"] or joint["name"] in joint_names:
            raise VerificationContractError("joint names must be unique and non-empty")
        if (
            joint["parent"] not in links
            or joint["child"] not in links
            or joint["parent"] == joint["child"]
            or joint["child"] in child_links
        ):
            raise VerificationContractError(
                "joint tree has invalid parent/child structure"
            )
        joint_names.add(joint["name"])
        child_links.add(joint["child"])
    roots = sorted(links - child_links)
    if len(roots) != 1:
        raise VerificationContractError("joint tree must have exactly one root")
    reachable = {roots[0]}
    while True:
        expanded = reachable | {
            joint["child"] for joint in joints if joint["parent"] in reachable
        }
        if expanded == reachable:
            break
        reachable = expanded
    if len(reachable) != len(links):
        raise VerificationContractError("joint tree is disconnected or cyclic")
    return {"links": link_names, "root_links": roots, "joints": joints}


def _fresh_urdf_structure(urdf_path: Path) -> dict[str, Any]:
    urdf = Path(urdf_path).resolve(strict=True)
    package = urdf.parent.resolve(strict=True)
    try:
        robot = ET.parse(urdf).getroot()
    except ET.ParseError as error:
        raise VerificationContractError(f"invalid URDF XML: {urdf_path}") from error
    if robot.tag != "robot":
        raise VerificationContractError("URDF root must be robot")
    link_nodes = list(robot.findall("link"))
    joint_nodes = list(robot.findall("joint"))
    link_names = [node.get("name", "").strip() for node in link_nodes]
    joints: list[dict[str, Any]] = []
    for joint in joint_nodes:
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        joint_type = joint.get("type", "").strip().lower()
        try:
            axis = _xml_normalized(
                _xml_vector_attribute(
                    joint.find("axis"), "xyz", (1.0, 0.0, 0.0)
                )
            )
        except VerificationContractError:
            axis = [float("nan"), float("nan"), float("nan")]
        origin = joint.find("origin")
        limit = joint.find("limit")
        joints.append(
            {
                "name": joint.get("name", "").strip(),
                "type": joint_type,
                "parent": (
                    parent_node.get("link", "").strip()
                    if parent_node is not None
                    else ""
                ),
                "child": (
                    child_node.get("link", "").strip()
                    if child_node is not None
                    else ""
                ),
                "origin_xyz": _xml_vector_attribute(
                    origin, "xyz", (0.0, 0.0, 0.0)
                ),
                "origin_rpy": _xml_vector_attribute(
                    origin, "rpy", (0.0, 0.0, 0.0)
                ),
                "axis": axis,
                "fk_supported": joint_type
                in {"fixed", "revolute", "continuous", "prismatic"},
                "lower": _xml_float_attribute(limit, "lower"),
                "upper": _xml_float_attribute(limit, "upper"),
                "effort": _xml_float_attribute(limit, "effort"),
                "velocity": _xml_float_attribute(limit, "velocity"),
            }
        )
    try:
        if any(not all(math.isfinite(value) for value in joint["axis"]) for joint in joints):
            raise VerificationContractError("joint axis must be finite and non-zero")
        tree = _xml_joint_tree(link_names, joints)
        tree_issue = None
    except VerificationContractError as error:
        tree = None
        tree_issue = str(error)
    scalar_joints = [
        joint
        for joint in joints
        if joint["type"] in {"revolute", "continuous", "prismatic"}
    ]
    resources: dict[str, dict[str, Any]] = {}
    collision_links = 0
    inertial_links = 0
    visual_links = 0
    inertial_rows: list[dict[str, Any]] = []
    for link in link_nodes:
        collisions = list(link.findall("collision"))
        visuals = list(link.findall("visual"))
        inertials = list(link.findall("inertial"))
        collision_links += int(bool(collisions))
        visual_links += int(bool(visuals))
        inertial_links += int(bool(inertials))
        for usage, elements in (("collision", collisions), ("visual", visuals)):
            meshes = [
                mesh
                for element in elements
                for mesh in element.findall(".//mesh")
            ]
            for mesh in meshes:
                reference = mesh.get("filename", "")
                raw = Path(reference)
                if not reference or raw.is_absolute() or "://" in reference:
                    raise VerificationContractError(
                        f"unsafe URDF mesh reference: {reference!r}"
                    )
                resource_path = (urdf.parent / raw).resolve(strict=False)
                try:
                    relative = resource_path.relative_to(package).as_posix()
                except ValueError as error:
                    raise VerificationContractError(
                        f"URDF mesh reference escapes package: {reference}"
                    ) from error
                if resource_path.is_symlink() or not resource_path.is_file():
                    raise VerificationContractError(
                        f"URDF mesh resource is missing or symlinked: {relative}"
                    )
                resource = resources.setdefault(
                    relative,
                    {
                        "relative_path": relative,
                        "sha256": sha256_file(resource_path),
                        "usages": [],
                        "occurrences": 0,
                    },
                )
                if usage not in resource["usages"]:
                    resource["usages"].append(usage)
                resource["occurrences"] += 1
        for inertial in inertials:
            inertia = inertial.find("inertia")
            inertial_rows.append(
                {
                    "link": link.get("name", ""),
                    "mass": _xml_float_attribute(inertial.find("mass"), "value"),
                    "inertia": {
                        key: _xml_float_attribute(inertia, key)
                        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
                    },
                }
            )
    resource_rows = [
        {**resources[key], "usages": sorted(resources[key]["usages"])}
        for key in sorted(resources)
    ]
    return {
        "link_names": link_names,
        "joint_names": [joint["name"] for joint in joints],
        "joints": joints,
        "scalar_joints": scalar_joints,
        "joint_tree": tree,
        "joint_tree_issue": tree_issue,
        "resources": resource_rows,
        "resource_sha256": canonical_sha256(resource_rows),
        "xml_counts": {
            "links": len(link_nodes),
            "joints": len(joint_nodes),
            "fixed_joints": sum(joint["type"] == "fixed" for joint in joints),
            "movable_joints": len(scalar_joints),
            "visual_elements": len(robot.findall(".//visual")),
            "collision_elements": len(robot.findall(".//collision")),
            "inertial_elements": len(robot.findall(".//inertial")),
        },
        "collision": {
            "covered_links": collision_links,
            "denominator_links": len(link_nodes),
            "full_link_coverage": collision_links == len(link_nodes),
            "element_count": len(robot.findall(".//collision")),
            "resource_sha256": canonical_sha256(
                [row for row in resource_rows if "collision" in row["usages"]]
            ),
        },
        "inertia": {
            "covered_links": inertial_links,
            "denominator_links": len(link_nodes),
            "full_link_coverage": inertial_links == len(link_nodes),
            "elements": inertial_rows,
            "elements_sha256": canonical_sha256(inertial_rows),
        },
        "visual": {
            "covered_links": visual_links,
            "denominator_links": len(link_nodes),
            "full_link_coverage": visual_links == len(link_nodes),
            "element_count": len(robot.findall(".//visual")),
        },
    }


def _fresh_row_source_errors(
    row: Any,
    *,
    table1_asset: Any,
    upstream: Mapping[str, Mapping[str, Any]],
    dataset_root: Path,
    order: int,
) -> list[str]:
    errors: list[str] = []
    try:
        if not isinstance(row, dict) or not isinstance(table1_asset, Mapping):
            return [f"manifest/Table 1 row is malformed at order {order}"]
        dataset_id = f"sketch_{order:04d}"
        if set(row) != MANIFEST_ROW_FIELDS:
            errors.append(f"{dataset_id} manifest row fields are not exact")
        asset_id = table1_asset.get("asset_id")
        expected_identity = {
            "dataset_id": dataset_id,
            "order": order,
            "asset_id": asset_id,
            "manifest_root": asset_id,
            "selection_rank": order + 1,
            "selection_hash": table1_asset.get("selection_hash"),
            "source": table1_asset.get("source"),
            "package_relative_path": asset_id,
            "urdf_relative_path": f"{asset_id}/mobility.urdf",
        }
        for field, expected in expected_identity.items():
            if row.get(field) != expected:
                errors.append(f"{dataset_id} source identity mismatch: {field}")
        if row.get("row_sha256") != canonical_sha256(
            row, exclude_fields={"row_sha256"}
        ):
            errors.append(f"{dataset_id} row self-hash mismatch")
        if not isinstance(asset_id, str) or not asset_id.startswith("data/"):
            return errors + [f"{dataset_id} Table 1 asset identity is malformed"]
        dataset = Path(dataset_root).resolve(strict=True)
        package = (dataset / asset_id).resolve(strict=True)
        package.relative_to(dataset)
        fresh_package = _package_binding(package)
        if row.get("package_binding") != fresh_package:
            errors.append(f"{dataset_id} package binding differs from fresh files")
        package_hash = fresh_package["content_manifest_sha256"]
        if row.get("package_content_manifest_sha256") != package_hash:
            errors.append(f"{dataset_id} package content hash mismatch")
        urdf_path = package / "mobility.urdf"
        urdf_hash = sha256_file(urdf_path)
        if row.get("urdf_sha256") != urdf_hash:
            errors.append(f"{dataset_id} URDF hash differs from fresh file")
        if table1_asset.get("mobility_urdf_sha256") != urdf_hash:
            errors.append(f"{dataset_id} Table 1 URDF binding mismatch")
        if set(upstream) != {"table2", "table3", "table4"}:
            return errors + [f"{dataset_id} upstream record set is incomplete"]
        table2, table3, table4 = (
            upstream["table2"],
            upstream["table3"],
            upstream["table4"],
        )
        provenance_checks = (
            (table2.get("asset_id") == asset_id, "Table 2 identity"),
            (table2.get("table1_selection_rank") == order + 1, "Table 2 rank"),
            (table3.get("asset_id") == asset_id, "Table 3 identity"),
            (table3.get("selection_rank") == order + 1, "Table 3 rank"),
            (table3.get("selection_hash") == table1_asset.get("selection_hash"), "Table 3 selection hash"),
            (table4.get("dataset_id") == asset_id, "Table 4 dataset identity"),
            (table4.get("asset_id") == asset_id, "Table 4 asset identity"),
            (table4.get("order") == order, "Table 4 order"),
            (table4.get("selection_rank") == order + 1, "Table 4 rank"),
            (table4.get("selection_hash") == table1_asset.get("selection_hash"), "Table 4 selection hash"),
        )
        for condition, label in provenance_checks:
            if not condition:
                errors.append(f"{dataset_id} {label} mismatch")
        for name, record in upstream.items():
            if record.get("package_content_manifest_sha256") != package_hash:
                errors.append(f"{dataset_id} {name} package binding mismatch")
        if table2.get("primary_urdf_sha256") != urdf_hash:
            errors.append(f"{dataset_id} Table 2 URDF binding mismatch")
        if table3.get("urdf_sha256") != urdf_hash:
            errors.append(f"{dataset_id} Table 3 URDF binding mismatch")
        fresh_structure = _fresh_urdf_structure(urdf_path)
        for field, expected in fresh_structure.items():
            if row.get(field) != expected:
                errors.append(
                    f"{dataset_id} {field} differs from independently parsed URDF"
                )
        raw_category = table3.get("raw_category")
        if (
            not isinstance(raw_category, str)
            or row.get("raw_category") != raw_category
            or row.get("category") != raw_category
            or table2.get("raw_category") != raw_category
            or table4.get("raw_category") != raw_category
        ):
            errors.append(f"{dataset_id} raw category binding mismatch")
        if row.get("strict_gates") != _upstream_gate_evidence(upstream):
            errors.append(f"{dataset_id} strict gates differ from upstream records")
        bbox = table4.get("object_bbox_diagonal_m")
        bbox_available = _finite_number(bbox) and float(bbox) > 0
        expected_bbox = {
            "status": "available" if bbox_available else "not_available",
            "diagonal_m": float(bbox) if bbox_available else None,
            "protocol": "pybullet_q0_collision_shape_union_aabb_v1",
        }
        if row.get("bounding_box") != expected_bbox:
            errors.append(f"{dataset_id} bounding-box receipt mismatch")
        if row.get("bounding_box_diagonal") != (
            float(bbox) if bbox_available else None
        ):
            errors.append(f"{dataset_id} bounding-box binding mismatch")
        issues: list[str] = []
        if row.get("joint_tree") is None:
            issues.append("invalid_joint_graph")
        if not bbox_available:
            issues.append("missing_bounding_box")
        expected_preflight = {
            "status": "pass" if not issues else "failed",
            "issues": issues,
            "simulator_eligible": not issues,
        }
        if row.get("preflight") != expected_preflight:
            errors.append(f"{dataset_id} preflight binding mismatch")
    except (OSError, TypeError, ValueError, KeyError) as error:
        errors.append(f"fresh row source closure failed at order {order}: {error}")
    return errors


def _manifest_core_errors(
    protocol: Any, manifest: Any, table1_manifest: Path
) -> list[str]:
    errors: list[str] = []
    try:
        if not isinstance(protocol, dict) or not isinstance(manifest, dict):
            return ["protocol or manifest is not a JSON object"]
        if set(manifest) != {
            "schema_version",
            "source_receipt",
            "selection",
            "upstream_artifacts",
            "protocol_sha256",
            "rows",
            "cohort_sha256",
        }:
            errors.append("manifest top-level fields are not exact")
        if manifest.get("schema_version") != "table5_sketch_mobility_manifest_v1":
            errors.append("manifest schema mismatch")
        if manifest.get("protocol_sha256") != protocol.get("protocol_sha256"):
            errors.append("manifest protocol binding mismatch")
        if manifest.get("cohort_sha256") != canonical_sha256(
            manifest, exclude_fields={"cohort_sha256", "generated_at"}
        ):
            errors.append("manifest cohort self-hash mismatch")
        rows = manifest.get("rows")
        if not isinstance(rows, list) or len(rows) != FORMAL_COHORT_SIZE:
            return errors + ["manifest does not contain the frozen 800 rows"]
        if not all(isinstance(row, dict) for row in rows):
            return errors + ["manifest contains a malformed row"]
        expected_ids = [f"sketch_{index:04d}" for index in range(FORMAL_COHORT_SIZE)]
        if [row.get("dataset_id") for row in rows] != expected_ids:
            errors.append("manifest runtime ID order is not sketch_0000..sketch_0799")
        source = manifest.get("source_receipt")
        selection = manifest.get("selection")
        if not isinstance(source, Mapping) or not isinstance(selection, Mapping):
            return errors + ["manifest source/selection receipt is malformed"]
        source_expected = {
            "dataset": "SketchMobility",
            "release_status": "RELEASED_MIXED_LICENSE_SECONDARY_CURATION",
            "N_release": 4956,
            "N_eval": 800,
            "table1_manifest_sha256": FORMAL_TABLE1_MANIFEST_SHA256,
            "release_manifest_sha256": FORMAL_RELEASE_MANIFEST_SHA256,
            "release_universe_sha256": FORMAL_RELEASE_UNIVERSE_SHA256,
        }
        for field, expected in source_expected.items():
            if source.get(field) != expected:
                errors.append(f"manifest source receipt mismatch: {field}")
        if sha256_file(Path(table1_manifest).resolve(strict=True)) != FORMAL_TABLE1_MANIFEST_SHA256:
            errors.append("Table 1 manifest fresh file hash mismatch")
        selection_expected = {
            "candidate_count": 800,
            "selected_count": 800,
            "identity_authority": "manifest_root",
            "internal_id": "sketch_<zero_based_order:04d>",
            "ordered_identity_sha256": FORMAL_ORDERED_IDENTITY_SHA256,
            "ordered_manifest_root_sha256": FORMAL_ORDERED_MANIFEST_ROOT_SHA256,
            "ordered_package_binding_sha256": FORMAL_ORDERED_PACKAGE_BINDING_SHA256,
            "ordered_urdf_binding_sha256": FORMAL_ORDERED_URDF_BINDING_SHA256,
            "replacement": "never",
            "outcome_filtering": False,
            "retained_preflight_failures": FORMAL_PREFLIGHT_FAILURES,
        }
        for field, expected in selection_expected.items():
            if selection.get(field) != expected:
                errors.append(f"manifest selection mismatch: {field}")
    except (OSError, TypeError, ValueError, KeyError) as error:
        errors.append(f"manifest core validation failed: {error}")
    return errors


def _fresh_source_closure_errors(
    protocol: dict[str, Any], manifest: dict[str, Any], table1_manifest: Path
) -> list[str]:
    errors = _manifest_core_errors(protocol, manifest, table1_manifest)
    if errors:
        return errors
    try:
        rows = manifest["rows"]
        table1_path = Path(table1_manifest).resolve(strict=True)
        table1 = _read_json_object(table1_path)
        for field, expected in {
            "dataset": "SketchMobility",
            "selection_protocol": "sketch-mobility-table1-global-sample-v1",
            "seed": "arti-skill-table1-sketch-mobility-n800-v1",
            "N_release": 4956,
            "N_eval": 800,
            "release_manifest_sha256": FORMAL_RELEASE_MANIFEST_SHA256,
            "release_universe_sha256": FORMAL_RELEASE_UNIVERSE_SHA256,
        }.items():
            if table1.get(field) != expected:
                errors.append(f"Table 1 source mismatch: {field}")
        assets = table1.get("assets")
        if not isinstance(assets, list) or len(assets) != FORMAL_COHORT_SIZE:
            return errors + ["Table 1 source does not contain 800 assets"]
        source = manifest["source_receipt"]
        if Path(source["table1_manifest_path"]).resolve(strict=True) != table1_path:
            errors.append("manifest Table 1 path does not resolve to supplied input")
        dataset_root = Path(source["dataset_root"]).resolve(strict=True)
        if sha256_file(dataset_root / source["release_manifest_path"]) != FORMAL_RELEASE_MANIFEST_SHA256:
            errors.append("release manifest fresh file hash mismatch")
        receipts = manifest.get("upstream_artifacts")
        if not isinstance(receipts, Mapping) or set(receipts) != {
            "table2",
            "table3",
            "table4",
        }:
            return errors + ["manifest upstream artifact receipt set is malformed"]
        upstream_maps: dict[str, dict[str, dict[str, Any]]] = {}
        for name, key in (("table2", "asset_id"), ("table3", "asset_id"), ("table4", "dataset_id")):
            receipt = receipts[name]
            root = Path(receipt["root"]).resolve(strict=True)
            if receipt != _artifact_set(root):
                errors.append(f"{name} artifact set differs from fresh files")
            upstream_maps[name] = _read_jsonl_by_id(root / "asset_records.jsonl", key)
        asset_ids = [asset.get("asset_id") for asset in assets]
        for name, records in upstream_maps.items():
            if set(records) != set(asset_ids):
                errors.append(f"{name} upstream roster differs from Table 1")
        if errors:
            return errors
        for order, (row, asset) in enumerate(zip(rows, assets)):
            asset_id = asset.get("asset_id") if isinstance(asset, Mapping) else None
            if not isinstance(asset_id, str):
                errors.append(f"Table 1 asset is malformed at order {order}")
                continue
            errors.extend(
                _fresh_row_source_errors(
                    row,
                    table1_asset=asset,
                    upstream={name: records[asset_id] for name, records in upstream_maps.items()},
                    dataset_root=dataset_root,
                    order=order,
                )
            )
            if len(errors) >= 50:
                return errors[:50] + ["fresh source errors truncated"]
        package_meta = [
            {
                "manifest_root": row["manifest_root"],
                "package_content_manifest_sha256": row["package_content_manifest_sha256"],
            }
            for row in rows
        ]
        urdf_meta = [
            {
                "manifest_root": row["manifest_root"],
                "urdf_relpath": row["urdf_relative_path"],
                "sha256": row["urdf_sha256"],
            }
            for row in rows
        ]
        recomputed = {
            "ordered_identity_sha256": canonical_sha256(
                [
                    {
                        "asset_id": row["asset_id"],
                        "selection_hash": row["selection_hash"],
                        "selection_rank": row["selection_rank"],
                    }
                    for row in rows
                ]
            ),
            "ordered_manifest_root_sha256": canonical_sha256([row["manifest_root"] for row in rows]),
            "ordered_package_binding_sha256": canonical_sha256(package_meta),
            "ordered_urdf_binding_sha256": canonical_sha256(urdf_meta),
        }
        for field, observed in recomputed.items():
            if manifest["selection"].get(field) != observed:
                errors.append(f"manifest selection recomputation mismatch: {field}")
        if sum(row["package_binding"]["file_count"] for row in rows) != FORMAL_PACKAGE_FILE_COUNT:
            errors.append("fresh package file-count total mismatch")
        if sum(row["package_binding"]["total_bytes"] for row in rows) != FORMAL_PACKAGE_TOTAL_BYTES:
            errors.append("fresh package byte total mismatch")
        if sum(not row["preflight"]["simulator_eligible"] for row in rows) != FORMAL_PREFLIGHT_FAILURES:
            errors.append("preflight failure count mismatch")
        if len({row["raw_category"] for row in rows}) != FORMAL_CATEGORY_COUNT:
            errors.append("raw category count mismatch")
    except (OSError, TypeError, ValueError, KeyError, IndexError) as error:
        errors.append(f"fresh manifest/upstream closure failed: {error}")
    return errors


def _source_contract(row: Mapping[str, Any]) -> dict[str, Any]:
    tree = row.get("joint_tree")
    if isinstance(tree, Mapping):
        links = tree.get("links")
        joints = tree.get("joints")
    else:
        links = row.get("link_names")
        joints = row.get("joints")
    if not isinstance(links, list) or not isinstance(joints, list) or not all(
        isinstance(joint, Mapping)
        and isinstance(joint.get("name"), str)
        and isinstance(joint.get("type"), str)
        for joint in joints
    ):
        raise VerificationContractError("manifest row source contract is malformed")
    return {
        "expected_link_names": sorted(links),
        "expected_joint_names": sorted(joint["name"] for joint in joints),
        "expected_fixed_joint_names": sorted(
            joint["name"] for joint in joints if joint["type"] == "fixed"
        ),
        "expected_movable_joint_names": sorted(
            joint["name"] for joint in joints if joint["type"] != "fixed"
        ),
        "bounding_box_diagonal": row.get("bounding_box_diagonal"),
    }


def _implementation_receipt_errors(
    receipt: Any, *, simulator: str, protocol: Mapping[str, Any]
) -> list[str]:
    errors: list[str] = []
    try:
        if not isinstance(receipt, dict) or set(receipt) != IMPLEMENTATION_RECEIPT_FIELDS:
            return ["adapter implementation receipt fields are malformed"]
        if receipt.get("schema_version") != "table5_adapter_implementation_receipt_v2":
            errors.append("adapter implementation receipt schema mismatch")
        if receipt.get("simulator") != simulator or receipt.get(
            "distribution_version"
        ) != protocol["adapters"][simulator]["version"]:
            errors.append("adapter implementation simulator/version mismatch")
        for field in (
            "distribution_metadata_sha256",
            "distribution_record_sha256",
            "module_origin_sha256",
            "evaluator_source_sha256",
        ):
            if not _is_sha256(receipt.get(field)):
                errors.append(f"adapter implementation hash is malformed: {field}")
        direct_url = receipt.get("direct_url")
        direct_hash = receipt.get("direct_url_sha256")
        if (direct_url is None) != (direct_hash is None) or (
            direct_hash is not None and not _is_sha256(direct_hash)
        ):
            errors.append("adapter direct-url binding is malformed")
        for path_field, hash_field in (
            ("module_origin", "module_origin_sha256"),
            ("evaluator_source", "evaluator_source_sha256"),
        ):
            path_value = receipt.get(path_field)
            if not isinstance(path_value, str) or not path_value:
                errors.append(f"adapter implementation path is malformed: {path_field}")
                continue
            path = Path(path_value).resolve(strict=True)
            if sha256_file(path) != receipt.get(hash_field):
                errors.append(f"adapter implementation fresh hash mismatch: {path_field}")
        git = receipt.get("git")
        if not isinstance(git, dict) or set(git) != {
            "root",
            "head",
            "dirty",
            "status_sha256",
        }:
            errors.append("adapter implementation git receipt is malformed")
    except (OSError, TypeError, ValueError, KeyError) as error:
        errors.append(f"adapter implementation receipt validation failed: {error}")
    return errors


def _normalized_gpu_uuid(value: Any) -> str:
    if not isinstance(value, str) or not value:
        raise VerificationContractError("GPU UUID is missing")
    normalized = value.lower()
    if normalized.startswith("gpu-"):
        normalized = normalized[4:]
    normalized = normalized.replace("-", "")
    if len(normalized) != 32 or any(
        character not in "0123456789abcdef" for character in normalized
    ):
        raise VerificationContractError("GPU UUID is malformed")
    return normalized


def _child_gpu_errors(receipt: Any) -> list[str]:
    fields = {
        "schema_version",
        *FROZEN_GENESIS_GPU_HARDWARE,
        "used_memory_mib",
        "free_memory_mib",
        "utilization_percent",
        "compute_pids",
        "worker_pid",
    }
    if not isinstance(receipt, dict) or set(receipt) != fields:
        return ["Genesis child GPU receipt fields are malformed"]
    errors: list[str] = []
    worker_pid = receipt.get("worker_pid")
    if (
        receipt.get("schema_version")
        != "table5_sketch_mobility_child_gpu_gate_receipt_v1"
    ):
        errors.append("Genesis child GPU receipt schema mismatch")
    if {
        key: receipt.get(key) for key in FROZEN_GENESIS_GPU_HARDWARE
    } != FROZEN_GENESIS_GPU_HARDWARE:
        errors.append("Genesis child GPU hardware mismatch")
    if not isinstance(worker_pid, int) or isinstance(worker_pid, bool) or worker_pid <= 0:
        errors.append("Genesis child GPU worker PID is malformed")
    if not all(
        isinstance(receipt.get(field), int)
        and not isinstance(receipt.get(field), bool)
        and receipt[field] >= 0
        for field in ("used_memory_mib", "free_memory_mib", "utilization_percent")
    ):
        errors.append("Genesis child GPU telemetry is malformed")
    pids = receipt.get("compute_pids")
    if not isinstance(pids, list) or pids != sorted(set(pids)) or not all(
        isinstance(pid, int) and not isinstance(pid, bool) and pid > 0 for pid in pids
    ):
        errors.append("Genesis child GPU PID receipt is malformed")
    elif isinstance(worker_pid, int) and not isinstance(worker_pid, bool):
        if not set(pids).issubset({worker_pid}):
            errors.append("Genesis child GPU contains an external compute PID")
    if not errors and (
        receipt["used_memory_mib"] > 1024
        or receipt["free_memory_mib"] < 32768
        or receipt["utilization_percent"] != 0
    ):
        errors.append("Genesis child GPU receipt violates the idle gate")
    return errors


def _device_receipt_errors(
    receipt: Any, *, simulator: str, completed: bool
) -> list[str]:
    if simulator != "genesis":
        return [] if receipt is None else ["CPU terminal contains a device receipt"]
    if receipt is None:
        return ["completed Genesis terminal lacks a device receipt"] if completed else []
    fields = {
        "schema_version",
        "backend",
        "logical_device",
        "logical_device_count",
        "cuda_visible_devices",
        "cuda_device_order",
        "physical_device_index",
        "visible_device_index",
        "nvidia_smi_gpu_uuid",
        "torch_gpu_uuid",
        "normalized_gpu_uuid",
        "nvidia_smi_device_name",
        "nvidia_smi_total_memory_mib",
        "driver_version",
        "nvidia_smi_compute_capability",
        *FROZEN_GENESIS_TORCH_DEVICE,
        *FROZEN_GENESIS_SOFTWARE,
        "child_gpu_gate_receipt",
    }
    if not isinstance(receipt, dict) or set(receipt) != fields:
        return ["Genesis device receipt fields are malformed"]
    errors = [
        f"device receipt: {error}"
        for error in _child_gpu_errors(receipt.get("child_gpu_gate_receipt"))
    ]
    try:
        normalized_torch = _normalized_gpu_uuid(receipt.get("torch_gpu_uuid"))
        normalized_frozen = _normalized_gpu_uuid(
            FROZEN_GENESIS_GPU_BINDING["gpu_uuid"]
        )
    except VerificationContractError as error:
        errors.append(f"Genesis device UUID is malformed: {error}")
        normalized_torch = normalized_frozen = None
    expected = {
        "schema_version": "table5_sketch_mobility_genesis_device_receipt_v1",
        "backend": "cuda",
        "logical_device": "cuda:0",
        "logical_device_count": 1,
        "cuda_visible_devices": FROZEN_GENESIS_GPU_BINDING[
            "cuda_visible_devices"
        ],
        "cuda_device_order": "PCI_BUS_ID",
        "physical_device_index": FROZEN_GENESIS_GPU_BINDING[
            "physical_device_index"
        ],
        "visible_device_index": FROZEN_GENESIS_GPU_BINDING["visible_device_index"],
        "nvidia_smi_gpu_uuid": FROZEN_GENESIS_GPU_BINDING["gpu_uuid"],
        "normalized_gpu_uuid": normalized_frozen,
        "nvidia_smi_device_name": FROZEN_GENESIS_GPU_HARDWARE["device_name"],
        "nvidia_smi_total_memory_mib": FROZEN_GENESIS_GPU_HARDWARE[
            "total_memory_mib"
        ],
        "driver_version": FROZEN_GENESIS_GPU_HARDWARE["driver_version"],
        "nvidia_smi_compute_capability": FROZEN_GENESIS_GPU_HARDWARE[
            "compute_capability"
        ],
        **FROZEN_GENESIS_TORCH_DEVICE,
        **FROZEN_GENESIS_SOFTWARE,
    }
    if any(receipt.get(field) != value for field, value in expected.items()):
        errors.append("Genesis device receipt does not match frozen GPU binding")
    if normalized_torch is not None and receipt.get("normalized_gpu_uuid") != normalized_torch:
        errors.append("Genesis device torch UUID binding mismatch")
    return errors


def _parent_gpu_errors(receipt: Any) -> list[str]:
    fields = {
        "schema_version",
        *FROZEN_GENESIS_GPU_HARDWARE,
        "used_memory_mib",
        "free_memory_mib",
        "utilization_percent",
        "compute_pids",
    }
    if not isinstance(receipt, dict) or set(receipt) != fields:
        return ["Genesis parent GPU receipt fields are malformed"]
    errors: list[str] = []
    if receipt.get("schema_version") != "table5_sketch_mobility_parent_gpu_gate_receipt_v1":
        errors.append("Genesis parent GPU receipt schema mismatch")
    if {
        key: receipt.get(key) for key in FROZEN_GENESIS_GPU_HARDWARE
    } != FROZEN_GENESIS_GPU_HARDWARE:
        errors.append("Genesis parent GPU hardware mismatch")
    if not all(
        isinstance(receipt.get(field), int)
        and not isinstance(receipt.get(field), bool)
        and receipt[field] >= 0
        for field in ("used_memory_mib", "free_memory_mib", "utilization_percent")
    ):
        errors.append("Genesis parent GPU telemetry is malformed")
    pids = receipt.get("compute_pids")
    if not isinstance(pids, list) or pids != sorted(set(pids)) or not all(
        isinstance(pid, int) and not isinstance(pid, bool) and pid > 0 for pid in pids
    ):
        errors.append("Genesis parent GPU PID receipt is malformed")
    if not errors and (
        receipt["used_memory_mib"] > 1024
        or receipt["free_memory_mib"] < 32768
        or receipt["utilization_percent"] != 0
        or bool(pids)
    ):
        errors.append("Genesis parent GPU receipt violates the idle gate")
    return errors


def _warning_list_errors(value: Any) -> list[str]:
    if not isinstance(value, list) or len(value) > WARNING_ENTRY_LIMIT:
        return ["retained warnings are malformed"]
    errors: list[str] = []
    for warning in value:
        if not isinstance(warning, dict) or set(warning) != {
            "logger",
            "level",
            "message",
        }:
            errors.append("retained warning fields are malformed")
            continue
        if any(
            not isinstance(warning[field], str)
            or len(warning[field]) > WARNING_MESSAGE_LIMIT
            for field in ("logger", "level", "message")
        ):
            errors.append("retained warning value is malformed")
    return errors


def _process_evidence_errors(
    status: Any, process: Any, diagnostics: Any
) -> list[str]:
    fields = {
        "exit_code",
        "signal",
        "timeout",
        "stdout_tail",
        "stderr_tail",
        "retained_warnings",
    }
    if not isinstance(process, dict) or set(process) != fields:
        return ["terminal process evidence fields are malformed"]
    errors: list[str] = []
    exit_code = process.get("exit_code")
    signal_value = process.get("signal")
    timeout = process.get("timeout")
    if not (
        exit_code is None
        or isinstance(exit_code, int)
        and not isinstance(exit_code, bool)
    ):
        errors.append("terminal process exit code is malformed")
    if not (
        signal_value is None
        or isinstance(signal_value, int)
        and not isinstance(signal_value, bool)
        and signal_value > 0
    ):
        errors.append("terminal process signal is malformed")
    if not isinstance(timeout, bool):
        errors.append("terminal process timeout flag is malformed")
    if isinstance(exit_code, int) and not isinstance(exit_code, bool):
        if exit_code < 0 and signal_value != -exit_code:
            errors.append("terminal process signal is inconsistent")
        elif exit_code >= 0 and signal_value is not None:
            errors.append("terminal process has an unexpected signal")
    for field in ("stdout_tail", "stderr_tail"):
        value = process.get(field)
        if not isinstance(value, str) or len(value) > TAIL_LIMIT:
            errors.append(f"terminal process {field} is malformed")
    warnings = process.get("retained_warnings")
    errors.extend(_warning_list_errors(warnings))
    diagnostic_warnings = (
        diagnostics.get("warnings") if isinstance(diagnostics, Mapping) else None
    )
    if warnings != diagnostic_warnings:
        errors.append("terminal process warnings differ from diagnostics")
    if errors:
        return errors
    clean_exit = exit_code == 0 and signal_value is None and timeout is False
    if status in {"completed", "diagnostic_failure"} and not clean_exit:
        errors.append(f"{status} terminal process evidence is inconsistent")
    if status == "timeout" and timeout is not True:
        errors.append("timeout terminal process evidence is inconsistent")
    if status == "native_crash" and not (
        timeout is False and isinstance(exit_code, int) and exit_code != 0
    ):
        errors.append("native-crash terminal process evidence is inconsistent")
    if status in {"malformed_response", "missing_response", "worker_error"} and not clean_exit:
        errors.append(f"{status} terminal process evidence is inconsistent")
    if status in {"parent_error", "preflight_failure"} and not (
        exit_code is None and signal_value is None and timeout is False
    ):
        errors.append(f"{status} terminal process evidence is inconsistent")
    return errors


def _unsupported_reason(
    joint: Mapping[str, Any], mapped_names: set[str]
) -> str | None:
    if joint.get("type") not in {"revolute", "prismatic"}:
        return "unsupported_joint_type"
    lower, upper = joint.get("lower"), joint.get("upper")
    if not _finite_number(lower) or not _finite_number(upper):
        return "missing_or_nonfinite_limit"
    if float(lower) >= float(upper):
        return "nonpositive_range"
    effort = joint.get("effort")
    if not _finite_number(effort) or float(effort) <= 0:
        return "missing_or_nonpositive_effort"
    velocity = joint.get("velocity")
    if not _finite_number(velocity) or float(velocity) <= 0:
        return "missing_or_nonpositive_velocity"
    if joint.get("name") not in mapped_names:
        return "missing_runtime_mapping"
    return None


def _classify_support(
    joints: Iterable[Mapping[str, Any]], mapped_names: Iterable[str]
) -> dict[str, dict[str, Any]]:
    mapped = set(mapped_names)
    support: dict[str, dict[str, Any]] = {}
    for joint in joints:
        name = joint.get("name")
        if not isinstance(name, str) or not name or name in support:
            raise VerificationContractError("manifest movable joints are malformed")
        reason = _unsupported_reason(joint, mapped)
        support[name] = {
            "name": name,
            "type": joint.get("type"),
            "parent": joint.get("parent"),
            "child": joint.get("child"),
            "lower": joint.get("lower"),
            "upper": joint.get("upper"),
            "effort": joint.get("effort"),
            "velocity": joint.get("velocity"),
            "runtime_mapped": name in mapped,
            "eligible": reason is None,
            "unsupported_reason": reason,
        }
    return support


def _completed_load_support(
    load: Any, support: Any, row: Mapping[str, Any]
) -> dict[str, dict[str, Any]]:
    tree = row.get("joint_tree")
    if not isinstance(tree, Mapping):
        raise VerificationContractError("completed row lacks a valid joint tree")
    tree_links, tree_joints = tree.get("links"), tree.get("joints")
    if not isinstance(tree_links, list) or not isinstance(tree_joints, list):
        raise VerificationContractError("completed row joint tree is malformed")
    if not isinstance(load, dict) or not isinstance(support, dict):
        raise VerificationContractError("completed load/support sections are malformed")
    expected_links = sorted(tree_links)
    expected_joints = sorted(joint["name"] for joint in tree_joints)
    expected_fixed = sorted(
        joint["name"] for joint in tree_joints if joint["type"] == "fixed"
    )
    observed_links = load.get("observed_link_names")
    observed_joints = load.get("observed_joint_names")
    mapped_scalar = load.get("mapped_scalar_joint_names")
    if not all(
        isinstance(values, list)
        and all(isinstance(name, str) for name in values)
        for values in (observed_links, observed_joints, mapped_scalar)
    ):
        raise VerificationContractError("completed load name lists are malformed")
    if (
        observed_links != sorted(set(observed_links))
        or observed_joints != sorted(set(observed_joints))
        or mapped_scalar != sorted(set(mapped_scalar))
    ):
        raise VerificationContractError("completed runtime names are not canonical")
    movable = [joint for joint in tree_joints if joint["type"] != "fixed"]
    expected_support = _classify_support(movable, mapped_scalar)
    supported_names = sorted(
        entry["name"] for entry in expected_support.values() if entry["eligible"]
    )
    unsupported_names = sorted(
        entry["name"] for entry in expected_support.values() if not entry["eligible"]
    )
    expected_load = {
        "created": True,
        "expected_link_names": expected_links,
        "observed_link_names": observed_links,
        "missing_link_names": sorted(
            (Counter(expected_links) - Counter(observed_links)).elements()
        ),
        "extra_link_names": sorted(
            (Counter(observed_links) - Counter(expected_links)).elements()
        ),
        "expected_joint_names": expected_joints,
        "observed_joint_names": observed_joints,
        "missing_joint_names": sorted(
            (Counter(expected_joints) - Counter(observed_joints)).elements()
        ),
        "extra_joint_names": sorted(
            (Counter(observed_joints) - Counter(expected_joints)).elements()
        ),
        "expected_fixed_joint_names": expected_fixed,
        "missing_fixed_joint_names": sorted(set(expected_fixed) - set(observed_joints)),
        "fixed_structure_preserved": not bool(
            set(expected_fixed) - set(observed_joints)
        ),
        "mapped_supported_joint_names": supported_names,
        "mapped_scalar_joint_names": mapped_scalar,
        "strict_load": observed_links == expected_links
        and observed_joints == expected_joints,
    }
    if load != expected_load:
        raise VerificationContractError("completed load differs from manifest row")
    expected_support_section = {
        "joints": list(expected_support.values()),
        "supported_joint_names": supported_names,
        "unsupported_joint_names": unsupported_names,
        "any_unsupported_movable_joint": bool(unsupported_names),
        "no_supported_movable_joint": not bool(supported_names),
    }
    if support != expected_support_section:
        raise VerificationContractError("completed support partition is inconsistent")
    return expected_support


def _finite_state(state: Any, names: Iterable[str]) -> bool:
    if not isinstance(state, dict):
        return False
    return all(
        name in state
        and isinstance(state[name], dict)
        and _finite_number(state[name].get("q"))
        and _finite_number(state[name].get("qdot"))
        for name in names
    )


def _settling_recomputed(
    states: list[dict[str, dict[str, float]]],
    support: Mapping[str, Mapping[str, Any]],
    mapped_names: Iterable[str],
    *,
    max_displacement: float,
    max_speed: float,
) -> dict[str, Any]:
    supported = [entry for entry in support.values() if entry["eligible"]]
    maximum_displacement = 0.0
    maximum_speed = 0.0
    mapped = list(mapped_names)
    finite = all(_finite_state(state, mapped) for state in states)
    displacement_within = True
    speed_within = True
    for state in states:
        for joint in supported:
            sample = state.get(joint["name"], {})
            if not _finite_number(sample.get("q")) or not _finite_number(
                sample.get("qdot")
            ):
                finite = False
                continue
            span = float(joint["upper"]) - float(joint["lower"])
            midpoint = (float(joint["lower"]) + float(joint["upper"])) / 2.0
            displacement = abs(float(sample["q"]) - midpoint) / span
            speed = abs(float(sample["qdot"])) / span
            displacement_within &= (
                midpoint - max_displacement * span
                <= float(sample["q"])
                <= midpoint + max_displacement * span
            )
            speed_within &= (
                -max_speed * span
                <= float(sample["qdot"])
                <= max_speed * span
            )
            maximum_displacement = max(maximum_displacement, displacement)
            maximum_speed = max(maximum_speed, speed)
    return {
        "passed": finite and displacement_within and speed_within,
        "finite": finite,
        "max_normalized_displacement": maximum_displacement,
        "max_normalized_speed_per_s": maximum_speed,
    }


def _validate_terminal_pose(pose: Any) -> None:
    if not isinstance(pose, dict) or set(pose) != {"translation", "rotation"}:
        raise VerificationContractError("descendant pose fields are malformed")
    translation, rotation = pose.get("translation"), pose.get("rotation")
    if not (
        isinstance(translation, list)
        and len(translation) == 3
        and all(_finite_number(value) for value in translation)
        and isinstance(rotation, list)
        and len(rotation) == 4
        and all(_finite_number(value) for value in rotation)
    ):
        raise VerificationContractError("descendant pose values are malformed")
    norm = math.sqrt(sum(float(value) ** 2 for value in rotation))
    if not math.isfinite(norm) or norm <= 0:
        raise VerificationContractError("descendant pose quaternion is malformed")


def _completed_diagnostic_metrics(
    load: Mapping[str, Any],
    support: Mapping[str, Any],
    diagnostics: Any,
    protocol: Mapping[str, Any],
    row: Mapping[str, Any],
    support_map: Mapping[str, Mapping[str, Any]],
) -> dict[str, bool]:
    if not isinstance(diagnostics, dict) or set(diagnostics) != {
        "reset",
        "settling",
        "actuation",
        "limit_enforcement",
        "warnings",
        "errors",
    }:
        raise VerificationContractError("completed diagnostic fields are not exact")
    warning_errors = _warning_list_errors(diagnostics.get("warnings"))
    if warning_errors:
        raise VerificationContractError(warning_errors[0])
    supported_names = support.get("supported_joint_names")
    unsupported_names = support.get("unsupported_joint_names")
    if not isinstance(supported_names, list) or not isinstance(
        unsupported_names, list
    ):
        raise VerificationContractError("completed support names are malformed")
    if (
        supported_names != sorted(set(supported_names))
        or unsupported_names != sorted(set(unsupported_names))
        or set(supported_names) & set(unsupported_names)
        or set(support_map) != set(supported_names) | set(unsupported_names)
    ):
        raise VerificationContractError("completed support partition is malformed")
    reset_rows = diagnostics.get("reset")
    reset_count = int(protocol["runtime"]["reset_repetitions"])
    if not isinstance(reset_rows, list) or len(reset_rows) != reset_count:
        raise VerificationContractError(
            "completed reset diagnostics have wrong repetition count"
        )
    mapped_names = load.get("mapped_scalar_joint_names")
    if not isinstance(mapped_names, list):
        raise VerificationContractError("completed mapped joint names are malformed")
    reset_pass = True
    for repetition, reset_row in enumerate(reset_rows, start=1):
        if not isinstance(reset_row, dict) or set(reset_row) != {
            "repetition",
            "state",
            "finite",
            "error",
        }:
            raise VerificationContractError("completed reset row fields are malformed")
        if reset_row.get("repetition") != repetition:
            raise VerificationContractError("completed reset repetition is malformed")
        expected_finite = _finite_state(reset_row.get("state"), mapped_names)
        if reset_row.get("finite") is not expected_finite:
            raise VerificationContractError("completed reset finite flag is inconsistent")
        error = reset_row.get("error")
        if error is not None and not isinstance(error, str):
            raise VerificationContractError("completed reset error is malformed")
        reset_pass &= expected_finite and error is None
    settling = diagnostics.get("settling")
    passive_steps = int(protocol["runtime"]["passive_settling"]["steps"])
    settling_fields = {
        "passed",
        "finite",
        "max_normalized_displacement",
        "max_normalized_speed_per_s",
        "state_trace",
        "steps_completed",
        "error",
    }
    if not isinstance(settling, dict) or set(settling) != settling_fields:
        raise VerificationContractError("completed settling fields are malformed")
    if settling.get("steps_completed") != passive_steps:
        raise VerificationContractError("completed settling step count is malformed")
    state_trace = settling.get("state_trace")
    if not isinstance(state_trace, list) or len(state_trace) != passive_steps:
        raise VerificationContractError("completed settling trace length is malformed")
    recomputed_settling = _settling_recomputed(
        state_trace,
        support_map,
        mapped_names,
        max_displacement=float(
            protocol["metrics"]["settling"]["max_normalized_displacement"]
        ),
        max_speed=float(
            protocol["metrics"]["settling"]["max_normalized_speed_per_s"]
        ),
    )
    if any(
        settling.get(field) != recomputed_settling[field]
        for field in (
            "finite",
            "passed",
            "max_normalized_displacement",
            "max_normalized_speed_per_s",
        )
    ) or settling.get("error") is not None:
        raise VerificationContractError("completed settling summary is inconsistent")
    actuation_rows = diagnostics.get("actuation")
    limit_rows = diagnostics.get("limit_enforcement")
    if not isinstance(actuation_rows, list) or not isinstance(limit_rows, list):
        raise VerificationContractError("completed diagnostic lists are malformed")
    if sorted(
        item.get("joint_name") for item in actuation_rows if isinstance(item, dict)
    ) != sorted(supported_names):
        raise VerificationContractError(
            "completed actuation roster differs from supported joints"
        )
    if sorted(
        item.get("joint_name") for item in limit_rows if isinstance(item, dict)
    ) != sorted(supported_names):
        raise VerificationContractError(
            "completed limit roster differs from supported joints"
        )
    exact_steps = list(range(0, 361, 12))
    actuation_pass = bool(supported_names) and not bool(unsupported_names)
    drift_pass = bool(supported_names) and not bool(unsupported_names)
    actuation_fields = {
        "joint_name",
        "joint_type",
        "passed",
        "finite",
        "realized_range_fraction",
        "minimum_q",
        "maximum_q",
        "steps_completed",
        "trajectory",
        "full_measured_positions_q",
        "final_descendant_root_frame_poses",
        "missing_descendant_link_names",
        "constraint_drift",
        "error",
    }
    drift_fields = {
        "passed",
        "finite",
        "max_translation_over_bbox_diagonal",
        "max_rotation_error_rad",
        "steps_compared",
        "normalized_translation_errors",
        "rotation_errors_rad",
        "unsupported_ancestor_joint_names",
    }
    for actuation in actuation_rows:
        if not isinstance(actuation, dict) or set(actuation) != actuation_fields:
            raise VerificationContractError("completed actuation fields are malformed")
        joint_name = actuation.get("joint_name")
        joint = support_map.get(joint_name)
        if joint is None or actuation.get("joint_type") != joint.get("type"):
            raise VerificationContractError("completed actuation joint binding mismatch")
        trajectory = actuation.get("trajectory")
        if not isinstance(trajectory, dict) or set(trajectory) != {
            "sample_steps",
            "normalized_positions",
        } or trajectory.get("sample_steps") != exact_steps:
            raise VerificationContractError("completed sparse trajectory is malformed")
        samples = trajectory.get("normalized_positions")
        full_positions = actuation.get("full_measured_positions_q")
        if not (
            isinstance(samples, list)
            and len(samples) == len(exact_steps)
            and all(_finite_number(value) for value in samples)
            and isinstance(full_positions, list)
            and len(full_positions) == 361
            and all(_finite_number(value) for value in full_positions)
        ):
            raise VerificationContractError("completed actuation traces are malformed")
        if actuation.get("steps_completed") != 360:
            raise VerificationContractError("completed actuation step count is malformed")
        if not all(
            isinstance(actuation.get(field), bool) for field in ("passed", "finite")
        ):
            raise VerificationContractError("completed actuation flags are malformed")
        reported = (
            actuation.get("realized_range_fraction"),
            actuation.get("minimum_q"),
            actuation.get("maximum_q"),
        )
        if not all(_finite_number(value) for value in reported):
            raise VerificationContractError("completed actuation range is malformed")
        span = float(joint["upper"]) - float(joint["lower"])
        minimum_q = min(float(value) for value in full_positions)
        maximum_q = max(float(value) for value in full_positions)
        realized = (maximum_q - minimum_q) / span
        if reported != (realized, minimum_q, maximum_q):
            raise VerificationContractError("completed actuation range differs from trace")
        expected_samples = [
            (float(full_positions[step]) - float(joint["lower"])) / span
            for step in exact_steps
        ]
        if samples != expected_samples:
            raise VerificationContractError("completed sparse trajectory differs from trace")
        actuation_error = actuation.get("error")
        if actuation_error is not None and not isinstance(actuation_error, str):
            raise VerificationContractError("completed actuation error is malformed")
        expected_actuation = bool(
            actuation["finite"]
            and realized
            >= float(
                protocol["metrics"]["actuation"]["min_declared_range_fraction"]
            )
            and actuation_error is None
        )
        if actuation["passed"] is not expected_actuation:
            raise VerificationContractError("completed actuation pass is inconsistent")
        actuation_pass &= actuation["passed"]
        poses = actuation.get("final_descendant_root_frame_poses")
        missing = actuation.get("missing_descendant_link_names")
        if not (
            isinstance(poses, dict)
            and isinstance(missing, list)
            and all(isinstance(name, str) for name in missing)
            and len(missing) == len(set(missing))
            and not set(poses) & set(missing)
        ):
            raise VerificationContractError("completed descendant pose partition is malformed")
        descendants = set(_descendant_names(row, str(joint["child"])))
        if set(poses) | set(missing) != descendants:
            raise VerificationContractError("completed descendant poses differ from topology")
        for link_name, pose in poses.items():
            if not isinstance(link_name, str):
                raise VerificationContractError("completed descendant pose name is malformed")
            _validate_terminal_pose(pose)
        drift = actuation.get("constraint_drift")
        if not isinstance(drift, dict) or set(drift) != drift_fields:
            raise VerificationContractError("completed drift fields are malformed")
        unsupported_ancestors = drift.get("unsupported_ancestor_joint_names")
        translation_trace = drift.get("normalized_translation_errors")
        rotation_trace = drift.get("rotation_errors_rad")
        if not (
            isinstance(unsupported_ancestors, list)
            and all(isinstance(name, str) for name in unsupported_ancestors)
            and unsupported_ancestors == sorted(set(unsupported_ancestors))
            and isinstance(translation_trace, list)
            and len(translation_trace) == 360
            and all(_finite_number(value) and float(value) >= 0 for value in translation_trace)
            and isinstance(rotation_trace, list)
            and len(rotation_trace) == 360
            and all(_finite_number(value) and float(value) >= 0 for value in rotation_trace)
        ):
            raise VerificationContractError("completed drift traces are malformed")
        maximum_translation = max(float(value) for value in translation_trace)
        maximum_rotation = max(float(value) for value in rotation_trace)
        if not (
            drift.get("finite") is True
            and drift.get("max_translation_over_bbox_diagonal") == maximum_translation
            and drift.get("max_rotation_error_rad") == maximum_rotation
            and drift.get("steps_compared") == 360
        ):
            raise VerificationContractError("completed drift summary differs from traces")
        expected_drift = bool(
            maximum_translation
            <= float(
                protocol["metrics"]["constraint_drift"][
                    "translation_over_bbox_diagonal_max"
                ]
            )
            and maximum_rotation
            <= float(protocol["metrics"]["constraint_drift"]["rotation_rad_max"])
            and not missing
            and not unsupported_ancestors
            and actuation_error is None
        )
        if drift.get("passed") is not expected_drift:
            raise VerificationContractError("completed drift pass is inconsistent")
        drift_pass &= expected_drift
    expected_errors = [
        item["error"] for item in actuation_rows if item.get("error") is not None
    ]
    if diagnostics.get("errors") != expected_errors:
        raise VerificationContractError("completed diagnostic errors are inconsistent")
    limit_pass = bool(supported_names) and not bool(unsupported_names)
    target_fields = {
        "target_normalized",
        "passed",
        "finite",
        "tolerance",
        "minimum_q",
        "maximum_q",
        "steps_completed",
        "error",
    }
    for limit_row in limit_rows:
        if not isinstance(limit_row, dict) or set(limit_row) != {
            "joint_name",
            "targets",
        }:
            raise VerificationContractError("completed limit row fields are malformed")
        joint = support_map.get(limit_row.get("joint_name"))
        targets = limit_row.get("targets")
        if joint is None or not (
            isinstance(targets, list)
            and len(targets) == 2
            and all(isinstance(entry, dict) and set(entry) == target_fields for entry in targets)
            and [entry.get("target_normalized") for entry in targets] == [-0.1, 1.1]
        ):
            raise VerificationContractError("completed limit targets are malformed")
        tolerance = max(
            float(protocol["runtime"]["limit_enforcement"]["tolerance"]["absolute_si"]),
            float(
                protocol["runtime"]["limit_enforcement"]["tolerance"][
                    "relative_declared_range"
                ]
            )
            * (float(joint["upper"]) - float(joint["lower"])),
        )
        for target in targets:
            if target.get("steps_completed") != int(
                protocol["runtime"]["limit_enforcement"]["steps_each"]
            ):
                raise VerificationContractError("completed limit step count is malformed")
            expected_limit = bool(
                target.get("finite") is True
                and _finite_number(target.get("minimum_q"))
                and _finite_number(target.get("maximum_q"))
                and target["minimum_q"] >= float(joint["lower"]) - tolerance
                and target["maximum_q"] <= float(joint["upper"]) + tolerance
                and target.get("tolerance") == tolerance
                and target.get("error") is None
            )
            if not isinstance(target.get("passed"), bool) or target["passed"] is not expected_limit:
                raise VerificationContractError("completed limit pass is inconsistent")
            limit_pass &= expected_limit
    return {
        "load": bool(load["strict_load"]),
        "reset": bool(reset_pass),
        "settling": bool(recomputed_settling["passed"]),
        "actuation": bool(actuation_pass),
        "limit_enforcement": bool(limit_pass),
        "constraint_drift": bool(drift_pass),
    }


def _failure_load_support(row: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    source = _source_contract(row)
    tree = row.get("joint_tree")
    joints = tree.get("joints") if isinstance(tree, Mapping) else row.get("joints")
    if not isinstance(joints, list):
        raise VerificationContractError("failure row joint metadata is malformed")
    movable = [joint for joint in joints if joint.get("type") != "fixed"]
    support = _classify_support(movable, set())
    expected_load = {
        "created": False,
        "expected_link_names": source["expected_link_names"],
        "observed_link_names": [],
        "missing_link_names": source["expected_link_names"],
        "extra_link_names": [],
        "expected_joint_names": source["expected_joint_names"],
        "observed_joint_names": [],
        "missing_joint_names": source["expected_joint_names"],
        "extra_joint_names": [],
        "expected_fixed_joint_names": source["expected_fixed_joint_names"],
        "missing_fixed_joint_names": source["expected_fixed_joint_names"],
        "fixed_structure_preserved": False,
        "mapped_supported_joint_names": [],
        "mapped_scalar_joint_names": [],
        "strict_load": False,
    }
    expected_support = {
        "joints": list(support.values()),
        "supported_joint_names": [],
        "unsupported_joint_names": sorted(support),
        "any_unsupported_movable_joint": bool(support),
        "no_supported_movable_joint": True,
    }
    return expected_load, expected_support


def _diagnostic_failure_errors(
    evidence: Any,
    *,
    row: Mapping[str, Any],
    protocol: Mapping[str, Any],
    simulator: str,
) -> list[str]:
    required = {
        "reason",
        "stage",
        "operation",
        "joint_name",
        "steps_completed",
        "exception_type",
        "message",
        "observed_joint_count",
        "supported_max_joint_count",
        "full_measured_positions_q",
        "trajectory",
    }
    if not isinstance(evidence, dict) or set(evidence) != required:
        return ["diagnostic failure evidence fields are not exact"]
    errors: list[str] = []
    for field in ("reason", "stage", "operation", "exception_type", "message"):
        if not isinstance(evidence[field], str) or not evidence[field]:
            errors.append(f"diagnostic failure {field} is malformed")
    if isinstance(evidence.get("message"), str) and len(evidence["message"]) > TAIL_LIMIT:
        errors.append("diagnostic failure message is too long")
    if evidence.get("stage") not in {
        "adapter_initialization",
        "reset",
        "settling",
        "actuation",
        "limit_enforcement",
    }:
        errors.append("diagnostic failure stage is malformed")
    tree = row.get("joint_tree")
    joints = tree.get("joints") if isinstance(tree, Mapping) else None
    if not isinstance(joints, list):
        errors.append("diagnostic failure row topology is malformed")
        joints = []
    movable_names = {
        joint.get("name") for joint in joints if joint.get("type") != "fixed"
    }
    joint_name = evidence.get("joint_name")
    if joint_name is not None and (
        not isinstance(joint_name, str) or joint_name not in movable_names
    ):
        errors.append("diagnostic failure joint name is malformed")
    steps_completed = evidence.get("steps_completed")
    if (
        not isinstance(steps_completed, int)
        or isinstance(steps_completed, bool)
        or steps_completed < 0
    ):
        errors.append("diagnostic failure step count is malformed")
    trace = evidence.get("full_measured_positions_q")
    trajectory = evidence.get("trajectory")
    if not isinstance(trace, list) or not all(_finite_number(value) for value in trace):
        errors.append("diagnostic failure finite prefix is malformed")
        trace = []
    if not isinstance(trajectory, dict) or set(trajectory) != {
        "sample_steps",
        "normalized_positions",
    }:
        errors.append("diagnostic failure trajectory fields are malformed")
        sample_steps: Any = []
        samples: Any = []
    else:
        sample_steps = trajectory.get("sample_steps")
        samples = trajectory.get("normalized_positions")
        if not (
            isinstance(sample_steps, list)
            and all(
                isinstance(step, int) and not isinstance(step, bool)
                for step in sample_steps
            )
            and sample_steps == sorted(set(sample_steps))
            and isinstance(samples, list)
            and len(samples) == len(sample_steps)
            and all(_finite_number(value) for value in samples)
        ):
            errors.append("diagnostic failure sparse trajectory is malformed")
            sample_steps, samples = [], []
    observed_count = evidence.get("observed_joint_count")
    supported_count = evidence.get("supported_max_joint_count")
    counts_absent = observed_count is None and supported_count is None
    counts_present = all(
        isinstance(value, int) and not isinstance(value, bool) and value >= 0
        for value in (observed_count, supported_count)
    )
    if not counts_absent and not counts_present:
        errors.append("diagnostic failure capacity counts are malformed")
    reason = evidence.get("reason")
    stage = evidence.get("stage")
    operation = evidence.get("operation")
    if reason == "pybullet_actual_state_link_limit":
        if not (
            simulator == "pybullet"
            and counts_present
            and stage == "adapter_initialization"
            and operation == "getJointState"
            and joint_name is None
            and observed_count >= PYBULLET_MAX_ACTUAL_STATE_JOINTS + 1
            and observed_count == row.get("xml_counts", {}).get("joints")
            and supported_count == PYBULLET_MAX_ACTUAL_STATE_JOINTS
            and steps_completed == 0
            and trace == []
            and sample_steps == []
            and samples == []
            and evidence.get("exception_type") == "RuntimeContractError"
            and evidence.get("message")
            == "PyBullet actual-state API does not support multibodies with 128 or more joints"
        ):
            errors.append("diagnostic failure PyBullet capacity evidence is inconsistent")
    elif reason == "simulator_diagnostic_interrupted":
        allowed_operations = {
            "reset": {"reset", "state"},
            "settling": {"reset", "step", "state"},
            "actuation": {"reset", "step", "state"},
            "limit_enforcement": {"reset", "step", "state"},
        }
        if (
            not counts_absent
            or stage not in allowed_operations
            or operation not in allowed_operations.get(stage, set())
        ):
            errors.append("diagnostic failure interruption evidence is inconsistent")
    elif reason == "simulator_asset_load_rejected":
        native_load_operations = {
            "pybullet": "loadURDF",
            "mujoco": "MjModel.from_xml_path",
            "genesis": "scene.add_entity_or_build",
        }
        if not (
            counts_absent
            and stage == "adapter_initialization"
            and operation == native_load_operations.get(simulator)
            and joint_name is None
            and steps_completed == 0
            and trace == []
            and sample_steps == []
            and samples == []
        ):
            errors.append("diagnostic failure asset-load evidence is inconsistent")
    else:
        errors.append("diagnostic failure reason is unknown")
    scalar = row.get("scalar_joints")
    scalar = scalar if isinstance(scalar, list) else []
    if stage == "actuation" and isinstance(steps_completed, int):
        joint = next(
            (item for item in scalar if item.get("name") == joint_name), None
        )
        if joint is None or steps_completed > 360:
            errors.append("diagnostic failure actuation evidence is inconsistent")
        elif trace:
            expected_steps = [
                step
                for step in protocol["cross_simulator"]["joint_rmse"]["sample_steps"]
                if step <= steps_completed
            ]
            span = float(joint["upper"]) - float(joint["lower"])
            expected_samples = [
                (float(trace[step]) - float(joint["lower"])) / span
                for step in expected_steps
            ] if len(trace) == steps_completed + 1 else None
            if (
                len(trace) != steps_completed + 1
                or sample_steps != expected_steps
                or samples != expected_samples
            ):
                errors.append("diagnostic failure actuation prefix is inconsistent")
        elif steps_completed != 0 or sample_steps or samples:
            errors.append("diagnostic failure empty actuation prefix is inconsistent")
    elif stage == "limit_enforcement" and isinstance(steps_completed, int):
        if (
            joint_name is None
            or steps_completed
            > int(protocol["runtime"]["limit_enforcement"]["steps_each"])
            or len(trace) != steps_completed
            or sample_steps
            or samples
        ):
            errors.append("diagnostic failure limit evidence is inconsistent")
    elif stage == "settling" and isinstance(steps_completed, int):
        if (
            steps_completed > int(protocol["runtime"]["passive_settling"]["steps"])
            or trace
            or sample_steps
            or samples
        ):
            errors.append("diagnostic failure settling evidence is inconsistent")
    elif stage in {"adapter_initialization", "reset"}:
        if steps_completed != 0 or trace or sample_steps or samples:
            errors.append("diagnostic failure pre-diagnostic evidence is inconsistent")
    try:
        json.dumps(evidence, allow_nan=False)
    except (TypeError, ValueError) as error:
        errors.append(f"diagnostic failure evidence is not finite JSON: {error}")
    return errors


def _failure_terminal_evidence_errors(
    record: Mapping[str, Any],
    *,
    row: Mapping[str, Any],
    protocol: Mapping[str, Any],
    simulator: str,
) -> list[str]:
    errors: list[str] = []
    status = record.get("terminal_status")
    expected_load, expected_support = _failure_load_support(row)
    if record.get("load") != expected_load:
        errors.append("failure load differs from the closed template")
    if record.get("support") != expected_support:
        errors.append("failure support differs from the closed template")
    diagnostics = record.get("diagnostics")
    base_fields = {
        "reset",
        "settling",
        "actuation",
        "limit_enforcement",
        "warnings",
        "errors",
    }
    expected_fields = set(base_fields)
    if status == "diagnostic_failure":
        expected_fields.add("diagnostic_failure")
    if status == "preflight_failure":
        expected_fields.add("preflight")
    if not isinstance(diagnostics, dict) or set(diagnostics) != expected_fields:
        return errors + ["failure diagnostic fields are not exact"]
    if (
        diagnostics.get("reset") != []
        or diagnostics.get("settling") != {}
        or diagnostics.get("actuation") != []
        or diagnostics.get("limit_enforcement") != []
        or _warning_list_errors(diagnostics.get("warnings"))
        or not isinstance(diagnostics.get("errors"), list)
        or len(diagnostics["errors"]) != 1
        or not isinstance(diagnostics["errors"][0], str)
    ):
        errors.append("failure diagnostics differ from the closed template")
        return errors
    if status == "diagnostic_failure":
        evidence = diagnostics.get("diagnostic_failure")
        errors.extend(
            _diagnostic_failure_errors(
                evidence, row=row, protocol=protocol, simulator=simulator
            )
        )
        if isinstance(evidence, Mapping) and diagnostics["errors"] != [
            f"diagnostic_failure: {evidence.get('message')}"
        ]:
            errors.append("diagnostic failure error message is inconsistent")
    elif status == "preflight_failure":
        if diagnostics.get("preflight") != row.get("preflight"):
            errors.append("preflight failure evidence differs from manifest")
        issues = row.get("preflight", {}).get("issues")
        expected_message = ",".join(issues) if isinstance(issues, list) else ""
        if diagnostics["errors"] != [f"preflight_failure: {expected_message}"]:
            errors.append("preflight failure error message is inconsistent")
    elif not diagnostics["errors"][0].startswith(f"{status}: "):
        errors.append("failure error message/status is inconsistent")
    return errors


def _terminal_record_errors(
    record: Any,
    *,
    row: Mapping[str, Any],
    simulator: str,
    phase: str,
    protocol: Mapping[str, Any],
    manifest: Mapping[str, Any],
) -> list[str]:
    errors: list[str] = []
    label = f"{simulator}/{row.get('dataset_id')}"
    try:
        if not isinstance(record, dict) or set(record) != TERMINAL_FIELDS:
            return [f"{label} terminal top-level fields are malformed"]
        if (
            record.get("schema_version")
            != "table5_sketch_mobility_runtime_asset_v1"
            or record.get("run_phase") != phase
            or record.get("terminal") is not True
            or record.get("terminal_status") not in TERMINAL_STATUSES
        ):
            errors.append(f"{label} terminal schema/phase/status mismatch")
        status = record.get("terminal_status")
        identity = record.get("identity")
        if not isinstance(identity, dict) or set(identity) != IDENTITY_FIELDS:
            return errors + [f"{label} terminal identity fields are malformed"]
        workers = identity.get("effective_workers")
        if not isinstance(workers, int) or isinstance(workers, bool) or workers < 1:
            errors.append(f"{label} effective worker count is malformed")
            workers = 1
        if simulator == "genesis" and workers != 1:
            errors.append(f"{label} Genesis effective workers must equal one")
        provenance = record.get("provenance")
        receipt = (
            provenance.get("adapter_implementation_receipt")
            if isinstance(provenance, Mapping)
            else None
        )
        errors.extend(
            f"{label}: {error}"
            for error in _implementation_receipt_errors(
                receipt, simulator=simulator, protocol=protocol
            )
        )
        expected_identity = {
            "dataset_id": row.get("dataset_id"),
            "asset_id": row.get("asset_id"),
            "manifest_root": row.get("manifest_root"),
            "order": row.get("order"),
            "selection_rank": row.get("selection_rank"),
            "selection_hash": row.get("selection_hash"),
            "simulator": simulator,
            "adapter_name": ADAPTER_NAMES[simulator],
            "adapter_version": protocol["adapters"][simulator]["version"],
            "adapter_config_sha256": canonical_sha256(
                {
                    "adapter": protocol["adapters"][simulator],
                    "runtime": protocol["runtime"],
                    "effective_child_timeout_s": float(
                        protocol["runtime"]["child_timeout_s"]
                    ),
                    "effective_workers": workers,
                }
            ),
            "adapter_implementation_sha256": canonical_sha256(receipt),
            "parent_gpu_hardware_sha256": (
                canonical_sha256(FROZEN_GENESIS_GPU_HARDWARE)
                if simulator == "genesis"
                else None
            ),
            "urdf_sha256": row.get("urdf_sha256"),
            "package_content_manifest_sha256": row.get(
                "package_content_manifest_sha256"
            ),
            "effective_workers": workers,
            "protocol_sha256": manifest.get("protocol_sha256"),
            "cohort_sha256": manifest.get("cohort_sha256"),
            "manifest_row_sha256": canonical_sha256(row),
        }
        if identity != expected_identity:
            errors.append(f"{label} terminal identity does not match manifest/protocol")
        metrics = record.get("metrics")
        if not isinstance(metrics, dict) or set(metrics) != set(METRICS) or not all(
            isinstance(metrics.get(metric), bool) for metric in METRICS
        ):
            errors.append(f"{label} terminal metric fields are not exact booleans")
        else:
            if metrics["simulator_pass"] is not all(
                metrics[metric] for metric in AND_METRICS
            ):
                errors.append(f"{label} simulator_pass is not the six-metric AND")
            if status != "completed" and any(metrics.values()):
                errors.append(f"{label} failure terminal contains passing metrics")
        for section in ("load", "support", "diagnostics", "process", "provenance"):
            if not isinstance(record.get(section), dict):
                errors.append(f"{label} terminal section is malformed: {section}")
        if isinstance(record.get("load"), dict) and isinstance(metrics, dict):
            if metrics.get("load") is not record["load"].get("strict_load"):
                errors.append(f"{label} load metric/detail mismatch")
        if record.get("source_contract") != _source_contract(row):
            errors.append(f"{label} source contract differs from manifest row")
        preflight = row.get("preflight")
        eligible = (
            isinstance(preflight, Mapping)
            and preflight.get("simulator_eligible") is True
        )
        if (status == "preflight_failure") == eligible:
            errors.append(f"{label} terminal/preflight eligibility mismatch")
        if not isinstance(provenance, dict) or set(provenance) != PROVENANCE_FIELDS:
            errors.append(f"{label} terminal provenance fields are malformed")
        else:
            manifest_row_hash = canonical_sha256(row)
            if (
                provenance.get("urdf_relative_path") != row.get("urdf_relative_path")
                or provenance.get("manifest_row_sha256") != manifest_row_hash
            ):
                errors.append(f"{label} terminal provenance row binding mismatch")
            expected_source_receipt = {
                "schema_version": "table5_sketch_mobility_worker_source_binding_receipt_v1",
                "status": "pass",
                "urdf_relative_path": row.get("urdf_relative_path"),
                "urdf_sha256": row.get("urdf_sha256"),
                "package_relative_path": row.get("package_relative_path"),
                "package_content_manifest_sha256": row.get(
                    "package_content_manifest_sha256"
                ),
                "package_binding": row.get("package_binding"),
            }
            source_receipt = provenance.get("source_binding_receipt")
            if status in {"completed", "diagnostic_failure"}:
                if source_receipt != expected_source_receipt:
                    errors.append(f"{label} worker source receipt is missing or mismatched")
            elif source_receipt is not None and source_receipt != expected_source_receipt:
                errors.append(f"{label} optional worker source receipt is mismatched")
            parent_gpu = provenance.get("parent_gpu_receipt")
            if simulator == "genesis":
                if parent_gpu is None and status != "preflight_failure":
                    errors.append(f"{label} Genesis terminal lacks parent GPU receipt")
                elif parent_gpu is not None:
                    errors.extend(
                        f"{label}: {error}" for error in _parent_gpu_errors(parent_gpu)
                    )
            elif parent_gpu is not None:
                errors.append(f"{label} CPU terminal contains a GPU receipt")
            errors.extend(
                f"{label}: {error}"
                for error in _device_receipt_errors(
                    provenance.get("device_receipt"),
                    simulator=simulator,
                    completed=status == "completed",
                )
            )
        errors.extend(
            f"{label}: {error}"
            for error in _process_evidence_errors(
                status, record.get("process"), record.get("diagnostics")
            )
        )
        if status == "completed":
            support_map = _completed_load_support(
                record.get("load"), record.get("support"), row
            )
            recomputed_metrics = _completed_diagnostic_metrics(
                record["load"],
                record["support"],
                record.get("diagnostics"),
                protocol,
                row,
                support_map,
            )
            recomputed_metrics["simulator_pass"] = all(
                recomputed_metrics[name] for name in AND_METRICS
            )
            if metrics != recomputed_metrics:
                errors.append(f"{label} terminal metrics differ from diagnostics")
        elif status in TERMINAL_STATUSES:
            errors.extend(
                f"{label}: {error}"
                for error in _failure_terminal_evidence_errors(
                    record,
                    row=row,
                    protocol=protocol,
                    simulator=simulator,
                )
            )
        json.dumps(record, allow_nan=False)
    except (OSError, TypeError, ValueError, KeyError, IndexError) as error:
        errors.append(f"terminal record validation failed for {label}: {error}")
    return errors


def _simulator_runtime_receipt_errors(
    simulator_root: Path,
    runtime_input: Any,
    *,
    simulator: str,
    phase: str,
    intent_ids: list[str],
    records: Mapping[str, Mapping[str, Any]],
) -> list[str]:
    errors: list[str] = []
    try:
        if not isinstance(runtime_input, dict) or set(runtime_input) != RUNTIME_INPUT_FIELDS:
            return [f"{simulator} runtime receipt fields are malformed"]
        assets_root = Path(simulator_root) / "assets"
        files = sorted(assets_root.glob("*.json")) if assets_root.is_dir() else []
        if [path.name for path in files] != [f"{dataset_id}.json" for dataset_id in intent_ids]:
            errors.append(f"{simulator} runtime file roster differs from intent")
        fresh_hashes = {path.name: sha256_file(path) for path in files}
        if runtime_input.get("record_file_hashes") != fresh_hashes:
            errors.append(f"{simulator} runtime record-file hash closure mismatch")
        record_rows = [
            {"filename": name, "sha256": fresh_hashes[name]}
            for name in sorted(fresh_hashes, key=lambda item: Path(item).stem)
        ]
        if runtime_input.get("record_set_sha256") != canonical_sha256(record_rows):
            errors.append(f"{simulator} runtime record-set hash mismatch")
        for name in ("summary.json", "progress.json"):
            field = name.removesuffix(".json") + "_sha256"
            path = Path(simulator_root) / name
            if (
                not path.is_file()
                or path.is_symlink()
                or runtime_input.get(field) != sha256_file(path)
            ):
                errors.append(f"{simulator} runtime {name} hash mismatch")
        count = len(intent_ids)
        if not (
            runtime_input.get("present") is True
            and runtime_input.get("intent_count") == count
            and runtime_input.get("terminal_count") == count
            and runtime_input.get("complete") is True
        ):
            errors.append(f"{simulator} runtime receipt is incomplete")
        implementations = {
            record.get("identity", {}).get("adapter_implementation_sha256")
            for record in records.values()
        }
        if len(implementations) != 1 or runtime_input.get(
            "adapter_implementation_sha256"
        ) != next(iter(implementations), None):
            errors.append(f"{simulator} runtime implementation hash mismatch")
        workers = runtime_input.get("effective_workers")
        if not isinstance(workers, int) or isinstance(workers, bool) or workers < 1:
            errors.append(f"{simulator} runtime worker receipt is malformed")
        record_workers = {
            record.get("identity", {}).get("effective_workers")
            for record in records.values()
        }
        if record_workers != {workers}:
            errors.append(
                f"{simulator} runtime worker receipt differs from terminal records"
            )
        if simulator == "genesis" and workers != 1:
            errors.append("Genesis runtime worker receipt must equal one")
        if phase not in {"formal", "qualification"}:
            errors.append(f"{simulator} runtime phase is invalid")
    except (OSError, TypeError, ValueError, KeyError) as error:
        errors.append(f"{simulator} runtime receipt validation failed: {error}")
    return errors


def _runtime_summary_errors(
    summary: Any,
    progress: Any,
    *,
    simulator: str,
    phase: str,
    intent_count: int,
    records: Mapping[str, Mapping[str, Any]],
) -> list[str]:
    status_counts: dict[str, int] = {}
    metric_counts = {metric: 0 for metric in METRICS}
    workers = set()
    for record in records.values():
        status = record.get("terminal_status")
        status_counts[status] = status_counts.get(status, 0) + 1
        workers.add(record.get("identity", {}).get("effective_workers"))
        for metric in METRICS:
            metric_counts[metric] += record.get("metrics", {}).get(metric) is True
    effective_workers = next(iter(workers), None) if len(workers) == 1 else None
    expected = {
        "schema_version": "table5_sketch_mobility_runtime_summary_v1",
        "run_phase": phase,
        "simulator": simulator,
        "effective_workers": effective_workers,
        "intent_count": intent_count,
        "terminal_count": len(records),
        "remaining_count": intent_count - len(records),
        "complete": len(records) == intent_count,
        "terminal_status_counts": status_counts,
        "metric_pass_counts": metric_counts,
        "metric_denominator": intent_count,
    }
    errors: list[str] = []
    if summary != expected:
        errors.append(f"{simulator} summary differs from terminal recomputation")
    expected_progress = copy.deepcopy(expected)
    expected_progress["schema_version"] = "table5_sketch_mobility_runtime_progress_v1"
    if progress != expected_progress:
        errors.append(f"{simulator} progress differs from terminal recomputation")
    return errors


def _strict_supported_joints(row: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    tree = row.get("joint_tree")
    if tree is None and row.get("preflight", {}).get("simulator_eligible") is False:
        return []
    joints = tree.get("joints") if isinstance(tree, Mapping) else None
    if not isinstance(joints, list):
        raise VerificationContractError("manifest joint tree is malformed")
    return [
        joint
        for joint in joints
        if isinstance(joint, Mapping)
        and joint.get("type") in {"revolute", "prismatic"}
        and _finite_number(joint.get("lower"))
        and _finite_number(joint.get("upper"))
        and float(joint["lower"]) < float(joint["upper"])
        and _finite_number(joint.get("effort"))
        and float(joint["effort"]) > 0
        and _finite_number(joint.get("velocity"))
        and float(joint["velocity"]) > 0
    ]


def _movable_joints(row: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    tree = row.get("joint_tree")
    if tree is None and row.get("preflight", {}).get("simulator_eligible") is False:
        return []
    joints = tree.get("joints") if isinstance(tree, Mapping) else None
    if not isinstance(joints, list) or not all(
        isinstance(joint, Mapping) for joint in joints
    ):
        raise VerificationContractError("manifest movable-joint surface is malformed")
    return [joint for joint in joints if joint.get("type") != "fixed"]


def _supported_by_name(record: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    support = record.get("support")
    joints = support.get("joints") if isinstance(support, Mapping) else None
    if not isinstance(joints, list):
        return {}
    return {
        joint["name"]: joint
        for joint in joints
        if isinstance(joint, Mapping) and isinstance(joint.get("name"), str)
    }


def _actuation_by_name(record: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    diagnostics = record.get("diagnostics")
    entries = diagnostics.get("actuation") if isinstance(diagnostics, Mapping) else None
    if not isinstance(entries, list):
        return {}
    return {
        entry["joint_name"]: entry
        for entry in entries
        if isinstance(entry, Mapping) and isinstance(entry.get("joint_name"), str)
    }


def _descendant_names(row: Mapping[str, Any], child: str) -> list[str]:
    tree = row.get("joint_tree")
    joints = tree.get("joints") if isinstance(tree, Mapping) else None
    if not isinstance(joints, list):
        raise VerificationContractError("manifest descendant tree is malformed")
    reachable = {child}
    while True:
        expanded = reachable | {
            joint["child"]
            for joint in joints
            if isinstance(joint, Mapping)
            and joint.get("parent") in reachable
            and isinstance(joint.get("child"), str)
        }
        if expanded == reachable:
            return sorted(reachable)
        reachable = expanded


def _quaternion_error(left: Any, right: Any) -> float:
    if not (
        isinstance(left, list)
        and isinstance(right, list)
        and len(left) == len(right) == 4
        and all(_finite_number(value) for value in left + right)
    ):
        raise VerificationContractError("pose quaternion is malformed")
    left_norm = math.sqrt(sum(float(value) ** 2 for value in left))
    right_norm = math.sqrt(sum(float(value) ** 2 for value in right))
    if left_norm <= 0 or right_norm <= 0:
        raise VerificationContractError("pose quaternion has zero norm")
    dot = abs(
        sum(
            float(a) / left_norm * (float(b) / right_norm)
            for a, b in zip(left, right)
        )
    )
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _pose_error(
    left: Mapping[str, Any], right: Mapping[str, Any], diagonal: float
) -> tuple[float, float]:
    lt, rt = left.get("translation"), right.get("translation")
    if not (
        isinstance(lt, list)
        and isinstance(rt, list)
        and len(lt) == len(rt) == 3
        and all(_finite_number(value) for value in lt + rt)
        and _finite_number(diagonal)
        and diagonal > 0
    ):
        raise VerificationContractError("pose translation/scale is malformed")
    translation = math.sqrt(
        sum((float(a) - float(b)) ** 2 for a, b in zip(lt, rt))
    ) / diagonal
    return translation, _quaternion_error(left.get("rotation"), right.get("rotation"))


def _strict_consistency(
    protocol: Mapping[str, Any],
    row: Mapping[str, Any],
    records_by_simulator: Mapping[str, Mapping[str, Mapping[str, Any]]],
    dataset_id: str,
) -> bool:
    thresholds = protocol["cross_simulator"]["thresholds"]
    joint_threshold = float(thresholds["normalized_joint_rmse"])
    translation_threshold = float(thresholds["translation_over_bbox_diagonal"])
    rotation_threshold = float(thresholds["rotation_rad"])
    sample_steps = protocol["cross_simulator"]["joint_rmse"]["sample_steps"]
    consistency = True
    records: dict[str, Mapping[str, Any]] = {}
    for simulator in SIMULATORS:
        record = records_by_simulator.get(simulator, {}).get(dataset_id)
        if record is None:
            consistency = False
            continue
        identity = record.get("identity", {})
        if not (
            identity.get("dataset_id") == dataset_id
            and identity.get("asset_id") == row.get("manifest_root")
            and identity.get("manifest_root") == row.get("manifest_root")
        ):
            consistency = False
            continue
        records[simulator] = record
        if record.get("terminal_status") != "completed" or record.get(
            "metrics", {}
        ).get("simulator_pass") is not True:
            consistency = False
    supported_names = {joint["name"] for joint in _strict_supported_joints(row)}
    for joint in _movable_joints(row):
        for record in records.values():
            if record.get("terminal_status") != "completed":
                continue
            support = _supported_by_name(record).get(joint.get("name"))
            if not (
                joint.get("name") in supported_names
                and isinstance(support, Mapping)
                and support.get("type") == joint.get("type")
                and support.get("eligible") is True
                and support.get("runtime_mapped") is True
            ):
                consistency = False
    for joint in _strict_supported_joints(row):
        traces: dict[str, list[float]] = {}
        poses: dict[str, Mapping[str, Any]] = {}
        descendants = _descendant_names(row, joint["child"])
        for simulator, record in records.items():
            if record.get("terminal_status") != "completed":
                continue
            support = _supported_by_name(record).get(joint["name"])
            if not (
                isinstance(support, Mapping)
                and support.get("type") == joint["type"]
                and support.get("eligible") is True
                and support.get("runtime_mapped") is True
            ):
                consistency = False
                continue
            diagnostic = _actuation_by_name(record).get(joint["name"])
            trajectory = (
                diagnostic.get("trajectory")
                if isinstance(diagnostic, Mapping)
                else None
            )
            values = (
                trajectory.get("normalized_positions")
                if isinstance(trajectory, Mapping)
                else None
            )
            if not (
                isinstance(trajectory, Mapping)
                and trajectory.get("sample_steps") == sample_steps
                and isinstance(values, list)
                and len(values) == len(sample_steps)
                and all(_finite_number(value) for value in values)
            ):
                consistency = False
                continue
            traces[simulator] = [float(value) for value in values]
            pose_map = diagnostic.get("final_descendant_root_frame_poses")
            poses[simulator] = pose_map if isinstance(pose_map, Mapping) else {}
            if any(descendant not in poses[simulator] for descendant in descendants):
                consistency = False
        if set(traces) != set(SIMULATORS):
            consistency = False
        else:
            for left, right in SIMULATOR_PAIRS:
                rmse = math.sqrt(
                    statistics.fmean(
                        (a - b) ** 2 for a, b in zip(traces[left], traces[right])
                    )
                )
                if rmse > joint_threshold:
                    consistency = False
        for descendant in descendants:
            if not all(
                descendant in poses.get(simulator, {}) for simulator in SIMULATORS
            ):
                consistency = False
                continue
            for left, right in SIMULATOR_PAIRS:
                translation, rotation = _pose_error(
                    poses[left][descendant],
                    poses[right][descendant],
                    float(row["bounding_box_diagonal"]),
                )
                if translation > translation_threshold or rotation > rotation_threshold:
                    consistency = False
    return consistency


def _recompute_headline_tables(
    *,
    protocol: Mapping[str, Any],
    rows_by_id: Mapping[str, Mapping[str, Any]],
    records_by_simulator: Mapping[str, Mapping[str, Mapping[str, Any]]],
    intent_ids: list[str],
) -> tuple[dict[str, Any], dict[str, Any], dict[str, dict[str, bool]]]:
    denominator = len(intent_ids)
    outcomes: dict[str, dict[str, bool]] = {}
    for dataset_id in intent_ids:
        row = rows_by_id[dataset_id]
        runtime = all(
            records_by_simulator.get(simulator, {})
            .get(dataset_id, {})
            .get("metrics", {})
            .get("simulator_pass")
            is True
            for simulator in SIMULATORS
        )
        consistency = _strict_consistency(
            protocol, row, records_by_simulator, dataset_id
        )
        strict_urdf = row["strict_gates"]["table2"]["strict_urdf_pass"] is True
        strict_kinematic = (
            row["strict_gates"]["table3"]["strict_kinematic_pass"] is True
        )
        strict_collision = (
            row["strict_gates"]["table4"]["strict_collision_pass"] is True
        )
        outcomes[dataset_id] = {
            "strict_urdf_pass": strict_urdf,
            "strict_kinematic_pass": strict_kinematic,
            "strict_collision_pass": strict_collision,
            "all_three_runtime_pass": runtime,
            "strict_consistency": consistency,
            "strict_sim_ready": all(
                (strict_urdf, strict_kinematic, strict_collision, runtime, consistency)
            ),
        }
    table5a: dict[str, Any] = {}
    for simulator in SIMULATORS:
        node = {
            metric: _rate(
                sum(
                    records_by_simulator.get(simulator, {})
                    .get(dataset_id, {})
                    .get("metrics", {})
                    .get(metric)
                    is True
                    for dataset_id in intent_ids
                ),
                denominator,
            )
            for metric in METRICS
        }
        node["strict_collision_pass"] = _rate(
            sum(
                outcomes[dataset_id]["strict_collision_pass"]
                for dataset_id in intent_ids
            ),
            denominator,
        )
        table5a[simulator] = node
    table5b = {
        "per_simulator_pass": {
            simulator: copy.deepcopy(table5a[simulator]["simulator_pass"])
            for simulator in SIMULATORS
        },
        "all_three_load": _rate(
            sum(
                all(
                    records_by_simulator.get(simulator, {})
                    .get(dataset_id, {})
                    .get("metrics", {})
                    .get("load")
                    is True
                    for simulator in SIMULATORS
                )
                for dataset_id in intent_ids
            ),
            denominator,
        ),
        "all_three_runtime_pass": _rate(
            sum(
                outcomes[dataset_id]["all_three_runtime_pass"]
                for dataset_id in intent_ids
            ),
            denominator,
        ),
        **{
            metric: _rate(
                sum(outcomes[dataset_id][metric] for dataset_id in intent_ids),
                denominator,
            )
            for metric in (
                "strict_urdf_pass",
                "strict_kinematic_pass",
                "strict_collision_pass",
                "strict_consistency",
                "strict_sim_ready",
            )
        },
    }
    return table5a, table5b, outcomes


def _recompute_categories(
    *,
    protocol: Mapping[str, Any],
    rows_by_id: Mapping[str, Mapping[str, Any]],
    records_by_simulator: Mapping[str, Mapping[str, Mapping[str, Any]]],
    intent_ids: list[str],
    outcomes: Mapping[str, Mapping[str, bool]],
) -> dict[str, Any]:
    del protocol
    groups: dict[str, list[str]] = {}
    for dataset_id in intent_ids:
        category = rows_by_id[dataset_id].get("raw_category")
        if not isinstance(category, str) or not category:
            raise VerificationContractError("manifest raw category is malformed")
        groups.setdefault(category, []).append(dataset_id)

    def category_metrics(ids: list[str]) -> dict[str, Any]:
        denominator = len(ids)
        table5a: dict[str, Any] = {}
        for simulator in SIMULATORS:
            table5a[simulator] = {
                metric: _rate(
                    sum(
                        records_by_simulator.get(simulator, {})
                        .get(dataset_id, {})
                        .get("metrics", {})
                        .get(metric)
                        is True
                        for dataset_id in ids
                    ),
                    denominator,
                )
                for metric in METRICS
            }
            table5a[simulator]["strict_collision_pass"] = _rate(
                sum(outcomes[dataset_id]["strict_collision_pass"] for dataset_id in ids),
                denominator,
            )
        table5b = {
            "per_simulator_pass": {
                simulator: copy.deepcopy(table5a[simulator]["simulator_pass"])
                for simulator in SIMULATORS
            },
            "all_three_load": _rate(
                sum(
                    all(
                        records_by_simulator.get(simulator, {})
                        .get(dataset_id, {})
                        .get("metrics", {})
                        .get("load")
                        is True
                        for simulator in SIMULATORS
                    )
                    for dataset_id in ids
                ),
                denominator,
            ),
            "all_three_runtime_pass": _rate(
                sum(outcomes[dataset_id]["all_three_runtime_pass"] for dataset_id in ids),
                denominator,
            ),
            **{
                metric: _rate(
                    sum(outcomes[dataset_id][metric] for dataset_id in ids),
                    denominator,
                )
                for metric in (
                    "strict_urdf_pass",
                    "strict_kinematic_pass",
                    "strict_collision_pass",
                    "strict_consistency",
                    "strict_sim_ready",
                )
            },
        }
        return {"table5a": table5a, "table5b": table5b}

    micro = category_metrics(intent_ids)
    group_rows: list[dict[str, Any]] = []
    for category, ids in groups.items():
        warnings: list[str] = []
        if len(ids) == 1:
            warnings.append("singleton")
        if len(ids) < 5:
            warnings.append("small_group")
        group_rows.append(
            {
                "category": category,
                "size": len(ids),
                "warnings": warnings,
                **category_metrics(ids),
            }
        )
    macro = {
        "table5a": {
            simulator: {
                metric: {
                    "category_count": len(group_rows),
                    "percentage": (
                        statistics.fmean(
                            group["table5a"][simulator][metric]["percentage"]
                            for group in group_rows
                        )
                        if group_rows
                        else None
                    ),
                }
                for metric in TABLE5A_FIELDS
            }
            for simulator in SIMULATORS
        },
        "table5b": {
            "per_simulator_pass": {
                simulator: {
                    "category_count": len(group_rows),
                    "percentage": (
                        statistics.fmean(
                            group["table5b"]["per_simulator_pass"][simulator][
                                "percentage"
                            ]
                            for group in group_rows
                        )
                        if group_rows
                        else None
                    ),
                }
                for simulator in SIMULATORS
            },
            **{
                metric: {
                    "category_count": len(group_rows),
                    "percentage": (
                        statistics.fmean(
                            group["table5b"][metric]["percentage"]
                            for group in group_rows
                        )
                        if group_rows
                        else None
                    ),
                }
                for metric in TABLE5B_RATE_FIELDS
                if metric != "per_simulator_pass"
            },
        },
    }
    return {
        "headline": "micro",
        "small_group_threshold": 5,
        "category_count": len(group_rows),
        "micro": micro,
        "macro": macro,
        "groups": group_rows,
    }


def _category_errors(observed: Any, expected: Mapping[str, Any]) -> list[str]:
    return [] if observed == expected else ["category report differs from recomputation"]


def _distribution(values: list[float]) -> dict[str, int | float | None]:
    ordered = sorted(values)
    p95 = ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)] if ordered else None
    return {
        "population_max": max(values) if values else None,
        "mean": statistics.fmean(values) if values else None,
        "median": statistics.median(values) if values else None,
        "p95": p95,
    }


def _joint_exclusion_reason(
    record: Mapping[str, Any] | None, joint_name: str, joint_type: str
) -> str | None:
    if record is None:
        return "missing_record"
    if record.get("terminal_status") != "completed":
        return f"terminal_{record.get('terminal_status')}"
    support = _supported_by_name(record).get(joint_name)
    if support is None:
        return "missing_support_entry"
    if support.get("type") != joint_type:
        return "joint_type_mismatch"
    if support.get("eligible") is not True:
        reason = support.get("unsupported_reason")
        return (
            reason
            if isinstance(reason, str) and reason
            else "unsupported_without_reason"
        )
    if support.get("runtime_mapped") is not True:
        return "eligible_not_runtime_mapped"
    return None


def _recompute_joint_rmse(
    protocol: Mapping[str, Any],
    rows_by_id: Mapping[str, Mapping[str, Any]],
    records: Mapping[str, Mapping[str, Mapping[str, Any]]],
    intent_ids: list[str],
) -> dict[str, Any]:
    steps = protocol["cross_simulator"]["joint_rmse"]["sample_steps"]
    threshold = float(
        protocol["cross_simulator"]["thresholds"]["normalized_joint_rmse"]
    )
    values: dict[str, list[float]] = {"revolute": [], "prismatic": []}
    candidates = {"revolute": 0, "prismatic": 0}
    evaluable = {"revolute": 0, "prismatic": 0}
    pairs = {"revolute": 0, "prismatic": 0}
    for dataset_id in intent_ids:
        row = rows_by_id[dataset_id]
        tree = row.get("joint_tree")
        if tree is None and row.get("preflight", {}).get("simulator_eligible") is False:
            continue
        for joint in tree["joints"]:
            joint_type = joint.get("type")
            if joint_type not in values:
                continue
            candidates[joint_type] += 1
            traces: dict[str, list[float]] = {}
            for simulator in SIMULATORS:
                record = records[simulator].get(dataset_id)
                if _joint_exclusion_reason(record, joint["name"], joint_type) is not None:
                    continue
                diagnostic = _actuation_by_name(record).get(joint["name"])
                trajectory = (
                    diagnostic.get("trajectory")
                    if isinstance(diagnostic, Mapping)
                    else None
                )
                positions = (
                    trajectory.get("normalized_positions")
                    if isinstance(trajectory, Mapping)
                    else None
                )
                if not (
                    isinstance(trajectory, Mapping)
                    and trajectory.get("sample_steps") == steps
                    and isinstance(positions, list)
                    and len(positions) == len(steps)
                    and all(_finite_number(position) for position in positions)
                ):
                    raise VerificationContractError(
                        "eligible joint trajectory is malformed"
                    )
                traces[simulator] = [float(position) for position in positions]
            pair_values = [
                math.sqrt(
                    statistics.fmean(
                        (a - b) ** 2 for a, b in zip(traces[left], traces[right])
                    )
                )
                for left, right in SIMULATOR_PAIRS
                if left in traces and right in traces
            ]
            if pair_values:
                evaluable[joint_type] += 1
                pairs[joint_type] += len(pair_values)
                values[joint_type].append(max(pair_values))
    return {
        joint_type: {
            "candidate_units": candidates[joint_type],
            "evaluable_units": evaluable[joint_type],
            "available_pairs": pairs[joint_type],
            "excluded_units": candidates[joint_type] - evaluable[joint_type],
            "threshold": threshold,
            "units_within_threshold": sum(
                value <= threshold for value in values[joint_type]
            ),
            **_distribution(values[joint_type]),
        }
        for joint_type in ("revolute", "prismatic")
    }


def _recompute_link_pose(
    protocol: Mapping[str, Any],
    rows_by_id: Mapping[str, Mapping[str, Any]],
    records: Mapping[str, Mapping[str, Mapping[str, Any]]],
    intent_ids: list[str],
) -> dict[str, Any]:
    thresholds = protocol["cross_simulator"]["thresholds"]
    translation_threshold = float(thresholds["translation_over_bbox_diagonal"])
    rotation_threshold = float(thresholds["rotation_rad"])
    translation_values: list[float] = []
    rotation_values: list[float] = []
    candidates = evaluable = pairs = 0
    for dataset_id in intent_ids:
        row = rows_by_id[dataset_id]
        tree = row.get("joint_tree")
        if tree is None and row.get("preflight", {}).get("simulator_eligible") is False:
            continue
        diagonal = row.get("bounding_box_diagonal")
        for joint in tree["joints"]:
            joint_type = joint.get("type")
            if joint_type not in {"revolute", "prismatic"}:
                continue
            for descendant in _descendant_names(row, joint["child"]):
                candidates += 1
                if diagonal is None:
                    continue
                poses: dict[str, Mapping[str, Any]] = {}
                for simulator in SIMULATORS:
                    record = records[simulator].get(dataset_id)
                    if _joint_exclusion_reason(record, joint["name"], joint_type) is not None:
                        continue
                    diagnostic = _actuation_by_name(record).get(joint["name"])
                    pose_map = (
                        diagnostic.get("final_descendant_root_frame_poses")
                        if isinstance(diagnostic, Mapping)
                        else None
                    )
                    missing = (
                        diagnostic.get("missing_descendant_link_names")
                        if isinstance(diagnostic, Mapping)
                        else None
                    )
                    if not isinstance(pose_map, Mapping) or not isinstance(missing, list):
                        raise VerificationContractError(
                            "eligible descendant pose surface is malformed"
                        )
                    if descendant in pose_map:
                        poses[simulator] = pose_map[descendant]
                    elif descendant not in missing:
                        raise VerificationContractError(
                            "descendant pose lacks explicit result"
                        )
                pair_values = [
                    _pose_error(poses[left], poses[right], float(diagonal))
                    for left, right in SIMULATOR_PAIRS
                    if left in poses and right in poses
                ]
                if pair_values:
                    evaluable += 1
                    pairs += len(pair_values)
                    translation_values.append(max(value[0] for value in pair_values))
                    rotation_values.append(max(value[1] for value in pair_values))
    return {
        "evaluation_unit": ["asset", "tested_joint", "descendant_link"],
        "candidate_units": candidates,
        "evaluable_units": evaluable,
        "available_pairs": pairs,
        "excluded_units": candidates - evaluable,
        "translation_over_bbox_diagonal": {
            "threshold": translation_threshold,
            "units_within_threshold": sum(
                value <= translation_threshold for value in translation_values
            ),
            **_distribution(translation_values),
        },
        "rotation_rad": {
            "threshold": rotation_threshold,
            "units_within_threshold": sum(
                value <= rotation_threshold for value in rotation_values
            ),
            **_distribution(rotation_values),
        },
    }


def _joint_inventory_count(
    protocol: Mapping[str, Any],
    rows_by_id: Mapping[str, Mapping[str, Any]],
    records: Mapping[str, Mapping[str, Mapping[str, Any]]],
    intent_ids: list[str],
) -> int:
    threshold = float(
        protocol["cross_simulator"]["thresholds"]["normalized_joint_rmse"]
    )
    count = 0
    for dataset_id in intent_ids:
        row = rows_by_id[dataset_id]
        tree = row.get("joint_tree")
        if tree is None and row.get("preflight", {}).get("simulator_eligible") is False:
            continue
        for joint in tree["joints"]:
            joint_type = joint.get("type")
            if joint_type not in {"revolute", "prismatic"}:
                continue
            traces: dict[str, list[float]] = {}
            for simulator in SIMULATORS:
                record = records.get(simulator, {}).get(dataset_id)
                reason = _joint_exclusion_reason(record, joint["name"], joint_type)
                if reason is not None:
                    count += 1
                    continue
                diagnostic = _actuation_by_name(record).get(joint["name"])
                trajectory = diagnostic.get("trajectory")
                traces[simulator] = [
                    float(value) for value in trajectory["normalized_positions"]
                ]
            pair_values = [
                math.sqrt(
                    statistics.fmean(
                        (a - b) ** 2
                        for a, b in zip(traces[left], traces[right])
                    )
                )
                for left, right in SIMULATOR_PAIRS
                if left in traces and right in traces
            ]
            count += sum(value > threshold for value in pair_values)
            if pair_values:
                count += max(pair_values) > threshold
            else:
                count += 1
    return count


def _pose_inventory_count(
    protocol: Mapping[str, Any],
    rows_by_id: Mapping[str, Mapping[str, Any]],
    records: Mapping[str, Mapping[str, Mapping[str, Any]]],
    intent_ids: list[str],
) -> int:
    thresholds = protocol["cross_simulator"]["thresholds"]
    translation_threshold = float(thresholds["translation_over_bbox_diagonal"])
    rotation_threshold = float(thresholds["rotation_rad"])
    count = 0
    for dataset_id in intent_ids:
        row = rows_by_id[dataset_id]
        tree = row.get("joint_tree")
        if tree is None and row.get("preflight", {}).get("simulator_eligible") is False:
            continue
        diagonal = row.get("bounding_box_diagonal")
        for joint in tree["joints"]:
            joint_type = joint.get("type")
            if joint_type not in {"revolute", "prismatic"}:
                continue
            for descendant in _descendant_names(row, joint["child"]):
                if diagonal is None:
                    count += 1
                    continue
                poses: dict[str, Mapping[str, Any]] = {}
                for simulator in SIMULATORS:
                    record = records.get(simulator, {}).get(dataset_id)
                    reason = _joint_exclusion_reason(
                        record, joint["name"], joint_type
                    )
                    if reason is None:
                        diagnostic = _actuation_by_name(record).get(joint["name"])
                        pose_map = diagnostic.get("final_descendant_root_frame_poses")
                        missing = diagnostic.get("missing_descendant_link_names")
                        if descendant in pose_map:
                            poses[simulator] = pose_map[descendant]
                        elif descendant in missing:
                            reason = "missing_descendant_link_pose"
                        else:
                            raise VerificationContractError(
                                "eligible pose lacks an explicit result"
                            )
                    if reason is not None:
                        count += 1
                pair_values = [
                    _pose_error(poses[left], poses[right], float(diagonal))
                    for left, right in SIMULATOR_PAIRS
                    if left in poses and right in poses
                ]
                count += sum(
                    translation > translation_threshold
                    or rotation > rotation_threshold
                    for translation, rotation in pair_values
                )
                if pair_values:
                    count += (
                        max(value[0] for value in pair_values) > translation_threshold
                        or max(value[1] for value in pair_values) > rotation_threshold
                    )
                else:
                    count += 1
    return count


def _strict_inventory_count(
    protocol: Mapping[str, Any],
    rows_by_id: Mapping[str, Mapping[str, Any]],
    records_by_simulator: Mapping[str, Mapping[str, Mapping[str, Any]]],
    intent_ids: list[str],
) -> int:
    thresholds = protocol["cross_simulator"]["thresholds"]
    joint_threshold = float(thresholds["normalized_joint_rmse"])
    translation_threshold = float(thresholds["translation_over_bbox_diagonal"])
    rotation_threshold = float(thresholds["rotation_rad"])
    sample_steps = protocol["cross_simulator"]["joint_rmse"]["sample_steps"]
    count = 0
    for dataset_id in intent_ids:
        row = rows_by_id[dataset_id]
        records: dict[str, Mapping[str, Any]] = {}
        for simulator in SIMULATORS:
            record = records_by_simulator.get(simulator, {}).get(dataset_id)
            if record is None:
                count += 1
                continue
            identity = record.get("identity", {})
            if not (
                identity.get("dataset_id") == dataset_id
                and identity.get("asset_id") == row.get("manifest_root")
                and identity.get("manifest_root") == row.get("manifest_root")
            ):
                count += 1
                continue
            records[simulator] = record
            if record.get("terminal_status") != "completed":
                count += 1
                continue
            if record.get("metrics", {}).get("simulator_pass") is not True:
                count += 1
        strict_supported_names = {
            joint["name"] for joint in _strict_supported_joints(row)
        }
        for joint in _movable_joints(row):
            for simulator in SIMULATORS:
                record = records.get(simulator)
                if record is None or record.get("terminal_status") != "completed":
                    continue
                support = _supported_by_name(record).get(joint.get("name"))
                if not (
                    joint.get("name") in strict_supported_names
                    and isinstance(support, Mapping)
                    and support.get("type") == joint.get("type")
                    and support.get("eligible") is True
                    and support.get("runtime_mapped") is True
                ):
                    count += 1
        for joint in _strict_supported_joints(row):
            traces: dict[str, list[float]] = {}
            poses: dict[str, Mapping[str, Any]] = {}
            descendants = _descendant_names(row, joint["child"])
            for simulator in SIMULATORS:
                record = records.get(simulator)
                if record is None or record.get("terminal_status") != "completed":
                    continue
                support = _supported_by_name(record).get(joint["name"])
                if not (
                    isinstance(support, Mapping)
                    and support.get("type") == joint["type"]
                    and support.get("eligible") is True
                    and support.get("runtime_mapped") is True
                ):
                    count += 1
                    continue
                diagnostic = _actuation_by_name(record).get(joint["name"])
                trajectory = (
                    diagnostic.get("trajectory")
                    if isinstance(diagnostic, Mapping)
                    else None
                )
                values = (
                    trajectory.get("normalized_positions")
                    if isinstance(trajectory, Mapping)
                    else None
                )
                if not (
                    isinstance(trajectory, Mapping)
                    and trajectory.get("sample_steps") == sample_steps
                    and isinstance(values, list)
                    and len(values) == len(sample_steps)
                    and all(_finite_number(value) for value in values)
                ):
                    count += 1
                    continue
                traces[simulator] = [float(value) for value in values]
                pose_map = diagnostic.get("final_descendant_root_frame_poses")
                pose_map = pose_map if isinstance(pose_map, Mapping) else {}
                poses[simulator] = pose_map
                count += sum(descendant not in pose_map for descendant in descendants)
            if set(traces) == set(SIMULATORS):
                count += sum(
                    math.sqrt(
                        statistics.fmean(
                            (a - b) ** 2
                            for a, b in zip(traces[left], traces[right])
                        )
                    )
                    > joint_threshold
                    for left, right in SIMULATOR_PAIRS
                )
            for descendant in descendants:
                if not all(
                    descendant in poses.get(simulator, {}) for simulator in SIMULATORS
                ):
                    continue
                count += sum(
                    (
                        lambda pair: pair[0] > translation_threshold
                        or pair[1] > rotation_threshold
                    )(
                        _pose_error(
                            poses[left][descendant],
                            poses[right][descendant],
                            float(row["bounding_box_diagonal"]),
                        )
                    )
                    for left, right in SIMULATOR_PAIRS
                )
    return count


def _recompute_diagnostic_counts(
    *,
    protocol: Mapping[str, Any],
    rows_by_id: Mapping[str, Mapping[str, Any]],
    records_by_simulator: Mapping[str, Mapping[str, Mapping[str, Any]]],
    intent_ids: list[str],
) -> dict[str, Any]:
    joint = _recompute_joint_rmse(
        protocol, rows_by_id, records_by_simulator, intent_ids
    )
    poses = _recompute_link_pose(
        protocol, rows_by_id, records_by_simulator, intent_ids
    )
    drift_count = sum(
        len(record.get("diagnostics", {}).get("actuation", []))
        for simulator in SIMULATORS
        for dataset_id in intent_ids
        for record in [records_by_simulator.get(simulator, {}).get(dataset_id, {})]
        if record.get("terminal_status") == "completed"
    )
    return {
        "joint_rmse": {
            joint_type: {
                key: joint[joint_type][key]
                for key in (
                    "candidate_units",
                    "evaluable_units",
                    "available_pairs",
                    "excluded_units",
                )
            }
            for joint_type in ("revolute", "prismatic")
        },
        "link_pose": {
            key: poses[key]
            for key in (
                "candidate_units",
                "evaluable_units",
                "available_pairs",
                "excluded_units",
            )
        },
        "joint_exclusion_entries": _joint_inventory_count(
            protocol, rows_by_id, records_by_simulator, intent_ids
        ),
        "pose_exclusion_entries": _pose_inventory_count(
            protocol, rows_by_id, records_by_simulator, intent_ids
        ),
        "constraint_drift_entries": drift_count,
        "strict_consistency_entries": _strict_inventory_count(
            protocol, rows_by_id, records_by_simulator, intent_ids
        ),
    }


def _recompute_record_inventory(
    *,
    rows_by_id: Mapping[str, Mapping[str, Any]],
    records_by_simulator: Mapping[str, Mapping[str, Mapping[str, Any]]],
    intent_ids: list[str],
) -> list[dict[str, Any]]:
    inventory: list[dict[str, Any]] = []
    for simulator in SIMULATORS:
        records = records_by_simulator.get(simulator, {})
        for dataset_id in intent_ids:
            if dataset_id not in records:
                row = rows_by_id[dataset_id]
                inventory.append(
                    {
                        "dataset_id": dataset_id,
                        "asset_id": row["manifest_root"],
                        "manifest_root": row["manifest_root"],
                        "simulator": simulator,
                        "reason": "missing_record",
                    }
                )
        for dataset_id, record in sorted(records.items()):
            if dataset_id not in intent_ids:
                continue
            failures = [
                metric for metric in METRICS if record["metrics"][metric] is False
            ]
            if record.get("terminal_status") == "completed" and not failures:
                continue
            row = rows_by_id[dataset_id]
            entry = {
                "dataset_id": dataset_id,
                "asset_id": row["manifest_root"],
                "manifest_root": row["manifest_root"],
                "simulator": simulator,
                "reason": (
                    "terminal_failure"
                    if record.get("terminal_status") != "completed"
                    else "strict_metric_failure"
                ),
                "terminal_status": record.get("terminal_status"),
                "failed_metrics": failures,
            }
            if record.get("terminal_status") == "diagnostic_failure":
                entry["diagnostic_failure"] = copy.deepcopy(
                    record["diagnostics"]["diagnostic_failure"]
                )
            inventory.append(entry)
    return inventory


def _record_inventory_errors(observed: Any, expected: list[dict[str, Any]]) -> list[str]:
    return (
        []
        if observed == expected
        else ["failure record inventory differs from terminal recomputation"]
    )


def _diagnostic_count_errors(
    expected: Mapping[str, Any], inventory: Any, observed: Any
) -> list[str]:
    errors: list[str] = []
    if observed != expected:
        errors.append("aggregate diagnostic counts differ from independent recomputation")
    if not isinstance(inventory, Mapping):
        return errors + ["failure inventory is malformed"]
    for section, count_field in (
        ("joint_diagnostics", "joint_exclusion_entries"),
        ("pose_diagnostics", "pose_exclusion_entries"),
        ("constraint_drift", "constraint_drift_entries"),
        ("strict_consistency", "strict_consistency_entries"),
    ):
        rows = inventory.get(section)
        if not isinstance(rows, list) or len(rows) != expected[count_field]:
            errors.append(f"failure inventory count mismatch: {section}")
    return errors


def _expected_report_markdown(table5: Mapping[str, Any]) -> str:
    label = (
        "complete formal"
        if table5["formal_claim_complete"]
        else table5["state"]
    )
    lines = [
        "# SketchMobility Table 5",
        "",
        f"- Phase: `{table5['run_phase']}` ({table5['report_kind']})",
        f"- State: `{label}`",
        f"- full-N denominator: `{table5['intent']['count']}`",
        "- Scope: the exact ordered SketchMobility Table 1 N=800 cohort with contact-enabled simulator adapters.",
        "- Strict Sim-ready joins frozen Table 2/3/4 gates by exact manifest_root and requires all-three runtime plus strict consistency.",
        "- Cross-simulator agreement is simulator evidence, not real-world dynamics validation.",
        "",
        "## Table 5a",
        "",
        "| Simulator | Load | Reset | Settling | Actuation | Limits | Drift pass | Simulator pass | Strict collision |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for simulator in SIMULATORS:
        values = table5["table5a"][simulator]
        cells = [
            f"{values[metric]['passed']}/{values[metric]['denominator']} ({values[metric]['percentage']:.3f}%)"
            for metric in METRICS
        ]
        strict_collision = values["strict_collision_pass"]
        cells.append(
            f"{strict_collision['passed']}/{strict_collision['denominator']} "
            f"({strict_collision['percentage']:.3f}%)"
        )
        lines.append(f"| {simulator} | " + " | ".join(cells) + " |")
    table5b = table5["table5b"]
    lines.extend(
        [
            "",
            "## Table 5b",
            "",
            f"All-three Load: {table5b['all_three_load']['passed']}/{table5b['all_three_load']['denominator']} "
            f"({table5b['all_three_load']['percentage']:.3f}%).",
            "",
            f"All-three Runtime Pass: {table5b['all_three_runtime_pass']['passed']}/"
            f"{table5b['all_three_runtime_pass']['denominator']} "
            f"({table5b['all_three_runtime_pass']['percentage']:.3f}%).",
            "",
            f"Upstream Strict URDF: {table5b['strict_urdf_pass']['passed']}/"
            f"{table5b['strict_urdf_pass']['denominator']} "
            f"({table5b['strict_urdf_pass']['percentage']:.3f}%).",
            "",
            f"Upstream Strict Kinematic: {table5b['strict_kinematic_pass']['passed']}/"
            f"{table5b['strict_kinematic_pass']['denominator']} "
            f"({table5b['strict_kinematic_pass']['percentage']:.3f}%).",
            "",
            f"Upstream Strict Collision: {table5b['strict_collision_pass']['passed']}/"
            f"{table5b['strict_collision_pass']['denominator']} "
            f"({table5b['strict_collision_pass']['percentage']:.3f}%).",
            "",
            f"Strict consistency: {table5b['strict_consistency']['passed']}/"
            f"{table5b['strict_consistency']['denominator']} "
            f"({table5b['strict_consistency']['percentage']:.3f}%).",
            "",
            f"Strict Sim-ready: {table5b['strict_sim_ready']['passed']}/"
            f"{table5b['strict_sim_ready']['denominator']} "
            f"({table5b['strict_sim_ready']['percentage']:.3f}%).",
            "",
            "Diagnostic evaluable denominators are separate from the full-N strict rates:",
            "",
            f"- Revolute joint RMSE: {table5b['joint_rmse']['revolute']['evaluable_units']} evaluable units; "
            f"population max {table5b['joint_rmse']['revolute']['population_max']}.",
            f"- Prismatic joint RMSE: {table5b['joint_rmse']['prismatic']['evaluable_units']} evaluable units; "
            f"population max {table5b['joint_rmse']['prismatic']['population_max']}.",
            f"- Link-pose error: {table5b['link_pose_error']['evaluable_units']} diagnostic evaluable units.",
            "",
            "## Categories",
            "",
            "The micro headline uses all intent assets. The macro supplement is the unweighted mean over exact raw category strings.",
            "",
            f"Small-group warning threshold: < {table5['categories']['small_group_threshold']} assets.",
            "",
        ]
    )
    if table5["state"] == "incomplete":
        lines.extend(
            [
                "> INCOMPLETE: this report preserves intent denominators but is not a completed formal Table 5 claim.",
                "",
            ]
        )
    return "\n".join(lines)


def _report_errors(report: Any, table5: Mapping[str, Any]) -> list[str]:
    if not isinstance(report, str):
        return ["aggregate report is not text"]
    return (
        []
        if report == _expected_report_markdown(table5)
        else ["aggregate report differs from the deterministic Table 5 template"]
    )


def _aggregate_marker_errors(
    aggregate_root: Path,
    marker: Any,
    *,
    phase: str,
    protocol_sha256: str,
    cohort_sha256: str,
) -> list[str]:
    errors: list[str] = []
    try:
        if not isinstance(marker, dict) or set(marker) != {
            "schema_version",
            "run_phase",
            "protocol_sha256",
            "cohort_sha256",
            "file_hashes",
        }:
            return ["aggregate publication marker fields are not exact"]
        if (
            marker.get("schema_version")
            != "table5_sketch_mobility_aggregate_publication_v1"
            or marker.get("run_phase") != phase
            or marker.get("protocol_sha256") != protocol_sha256
            or marker.get("cohort_sha256") != cohort_sha256
        ):
            errors.append("aggregate publication marker binding mismatch")
        hashes = marker.get("file_hashes")
        if not isinstance(hashes, dict) or set(hashes) != AGGREGATE_HASH_FILES:
            errors.append("aggregate publication marker hash keys are not exact")
        else:
            for name in sorted(AGGREGATE_HASH_FILES):
                path = Path(aggregate_root) / name
                if (
                    path.is_symlink()
                    or not path.is_file()
                    or hashes[name] != sha256_file(path)
                ):
                    errors.append(f"aggregate file hash mismatch: {name}")
    except (OSError, TypeError, ValueError, KeyError) as error:
        errors.append(f"aggregate marker validation failed: {error}")
    return errors


def _verification_result(
    *, root: Path, phase: str, checks: dict[str, bool], errors: list[str]
) -> dict[str, Any]:
    return {
        "schema_version": "table5_sketch_mobility_independent_verification_v1",
        "status": "PASS" if checks and all(checks.values()) else "FAIL",
        "phase": phase,
        "run_root": str(root),
        "checks": checks,
        "errors": errors,
    }


def verify_publication(
    run_root: Path, *, phase: str, table1_manifest: Path
) -> dict[str, Any]:
    root = Path(run_root).resolve()
    table1_manifest = Path(table1_manifest).resolve()
    checks: dict[str, bool] = {}
    errors: list[str] = []

    def set_check(name: str, observed_errors: list[str]) -> None:
        checks[name] = not observed_errors
        errors.extend(observed_errors)

    def read(path: Path, label: str) -> dict[str, Any] | None:
        try:
            return _read_json_object(path)
        except (OSError, TypeError, ValueError) as error:
            errors.append(f"{label}: {error}")
            return None

    def read_text(path: Path, label: str) -> str | None:
        try:
            if path.is_symlink() or not path.is_file():
                raise VerificationContractError(f"missing regular file: {path}")
            return path.read_text(encoding="utf-8")
        except (OSError, TypeError, ValueError) as error:
            errors.append(f"{label}: {error}")
            return None

    try:
        if phase not in {"qualification", "formal"}:
            raise VerificationContractError(f"invalid phase: {phase}")
        marker = read(root / "receipt_set.json", "receipt marker")
        protocol = read(root / "protocol.json", "protocol")
        manifest = read(root / "manifest.json", "manifest")
        receipt_errors: list[str] = []
        if marker is None or protocol is None or manifest is None:
            receipt_errors.append("receipt set is incomplete")
        else:
            if set(marker) != {
                "schema_version",
                "protocol_sha256",
                "manifest_sha256",
            }:
                receipt_errors.append("receipt marker fields are not exact")
            if marker.get("schema_version") != "table5_sketch_mobility_receipt_set_v1":
                receipt_errors.append("receipt marker schema mismatch")
            if marker.get("protocol_sha256") != sha256_file(root / "protocol.json"):
                receipt_errors.append("receipt marker protocol file hash mismatch")
            if marker.get("manifest_sha256") != sha256_file(root / "manifest.json"):
                receipt_errors.append("receipt marker manifest file hash mismatch")
        set_check("receipt_set_hashes", receipt_errors)
        set_check(
            "canonical_protocol_closure",
            _canonical_protocol_errors(protocol)
            if protocol is not None
            else ["protocol unavailable"],
        )
        core_errors = (
            _manifest_core_errors(protocol, manifest, table1_manifest)
            if protocol is not None and manifest is not None
            else ["manifest core unavailable"]
        )
        set_check("formal_manifest_roster", core_errors)
        rows = manifest.get("rows") if isinstance(manifest, dict) else None
        rows_valid = (
            isinstance(rows, list)
            and len(rows) == FORMAL_COHORT_SIZE
            and all(isinstance(row, dict) for row in rows)
        )
        checks["runtime_id_order"] = bool(
            rows_valid
            and [row.get("dataset_id") for row in rows]
            == [f"sketch_{index:04d}" for index in range(FORMAL_COHORT_SIZE)]
        )
        checks["table1_cohort_order"] = False

        aggregate_root = root / "aggregate" / phase
        aggregate_set = read(aggregate_root / "aggregate_set.json", "aggregate marker")
        table5 = read(aggregate_root / "table5.json", "Table 5 aggregate")
        inventory = read(
            aggregate_root / "failure_inventory.json", "failure inventory"
        )
        self_check = read(aggregate_root / "self_check.json", "aggregate self-check")
        report = read_text(aggregate_root / "report.md", "aggregate report")
        aggregate_available = all(
            item is not None
            for item in (aggregate_set, table5, inventory, self_check, report)
        )

        if rows_valid and isinstance(table5, dict):
            try:
                intent_ids, intent_rows = _phase_intent(
                    phase=phase, table5=table5, rows=rows
                )
                set_check("phase_intent_roster", [])
            except (TypeError, ValueError, KeyError) as error:
                intent_ids, intent_rows = [], []
                set_check("phase_intent_roster", [str(error)])
        elif phase == "formal" and rows_valid:
            intent_rows = list(rows)
            intent_ids = [row["dataset_id"] for row in rows]
            set_check("phase_intent_roster", ["formal Table 5 intent is unavailable"])
        else:
            intent_ids, intent_rows = [], []
            set_check("phase_intent_roster", ["cannot determine publication intent"])

        row_by_id = (
            {row["dataset_id"]: row for row in rows} if rows_valid else {}
        )
        records_by_simulator: dict[str, dict[str, dict[str, Any]]] = {}
        summaries_complete = bool(intent_ids)
        runtime_receipt = (
            table5.get("receipt", {}).get("runtime_inputs")
            if isinstance(table5, dict)
            else None
        )
        deep_validation = bool(aggregate_available and not core_errors and intent_ids)
        for simulator in SIMULATORS:
            simulator_root = root / phase / simulator
            summary = read(simulator_root / "summary.json", f"{simulator} summary")
            progress = read(simulator_root / "progress.json", f"{simulator} progress")
            assets_root = simulator_root / "assets"
            files = sorted(assets_root.glob("*.json")) if assets_root.is_dir() else []
            roster_errors = []
            if [path.stem for path in files] != intent_ids:
                roster_errors.append(
                    f"{simulator} terminal roster is incomplete or out of order"
                )
            set_check(f"{simulator}_terminal_roster", roster_errors)
            records: dict[str, dict[str, Any]] = {}
            terminal_errors: list[str] = []
            if deep_validation:
                for path in files:
                    record = read(path, f"{simulator}/{path.name}")
                    if record is None:
                        terminal_errors.append(f"{simulator}/{path.name} is malformed")
                        continue
                    records[path.stem] = record
                    row = row_by_id.get(path.stem)
                    if row is None:
                        terminal_errors.append(
                            f"{simulator}/{path.name} is outside manifest"
                        )
                        continue
                    terminal_errors.extend(
                        _terminal_record_errors(
                            record,
                            row=row,
                            simulator=simulator,
                            phase=phase,
                            protocol=protocol,
                            manifest=manifest,
                        )
                    )
            else:
                terminal_errors.append(
                    f"{simulator} deep terminal validation deferred for incomplete publication"
                )
            set_check(f"{simulator}_terminal_records", terminal_errors)
            summary_errors = (
                _runtime_summary_errors(
                    summary,
                    progress,
                    simulator=simulator,
                    phase=phase,
                    intent_count=len(intent_ids),
                    records=records,
                )
                if deep_validation and summary is not None and progress is not None
                else (
                    []
                    if summary is not None
                    and summary.get("complete") is True
                    and summary.get("remaining_count") == 0
                    and summary.get("intent_count") == len(intent_ids)
                    and summary.get("terminal_count") == len(intent_ids)
                    and progress is not None
                    else [f"{simulator} summary/progress is incomplete"]
                )
            )
            set_check(f"{simulator}_summary_reaggregation", summary_errors)
            complete = not roster_errors and not summary_errors and (
                not terminal_errors or not deep_validation
            )
            summaries_complete = summaries_complete and complete
            records_by_simulator[simulator] = records
            runtime_errors: list[str] = []
            if deep_validation:
                if not isinstance(runtime_receipt, Mapping) or simulator not in runtime_receipt:
                    runtime_errors.append(
                        f"{simulator} aggregate runtime receipt is missing"
                    )
                else:
                    runtime_errors.extend(
                        _simulator_runtime_receipt_errors(
                            simulator_root,
                            runtime_receipt[simulator],
                            simulator=simulator,
                            phase=phase,
                            intent_ids=intent_ids,
                            records=records,
                        )
                    )
            else:
                runtime_errors.append(
                    f"{simulator} runtime receipt unavailable for incomplete publication"
                )
            set_check(f"{simulator}_runtime_receipt_closure", runtime_errors)
        set_check(
            "all_simulator_summaries_complete",
            []
            if summaries_complete
            else ["one or more simulator summaries are incomplete"],
        )

        aggregate_errors: list[str] = []
        if not aggregate_available:
            aggregate_errors.append("aggregate publication is missing")
        else:
            if {path.name for path in aggregate_root.iterdir()} != AGGREGATE_FILES:
                aggregate_errors.append("aggregate publication file set is not exact")
            expected_formal = phase == "formal"
            if not (
                set(table5)
                == {
                    "schema_version",
                    "method",
                    "dataset",
                    "run_phase",
                    "report_kind",
                    "state",
                    "formal_claim_complete",
                    "intent",
                    "receipt",
                    "claim_boundary",
                    "table5a",
                    "table5b",
                    "asset_outcomes",
                    "categories",
                }
                and
                table5.get("schema_version")
                == "table5_sketch_mobility_aggregate_v1"
                and table5.get("method")
                == "SketchMobility Table 1 cohort contact-enabled cross-simulator readiness"
                and table5.get("dataset") == "SketchMobility"
                and table5.get("run_phase") == phase
                and table5.get("state") == "complete"
                and table5.get("formal_claim_complete") is expected_formal
                and table5.get("report_kind")
                == ("formal" if expected_formal else "non_formal")
                and table5.get("claim_boundary")
                == {
                    "contact_enabled_runtime_diagnostic": True,
                    "upstream_gates_joined_by_manifest_root": True,
                    "strict_consistency_requires_all_three_simulators": True,
                    "strict_sim_ready_formula": (
                        "strict_urdf AND strict_kinematic AND strict_collision AND "
                        "all_three_runtime_pass AND strict_consistency"
                    ),
                }
            ):
                aggregate_errors.append("Table 5 aggregate schema/claim state mismatch")
            inventory_fields = {
                "schema_version",
                "run_phase",
                "records",
                "joint_diagnostics",
                "pose_diagnostics",
                "constraint_drift",
                "strict_consistency",
            }
            if not (
                inventory.get("schema_version")
                == "table5_sketch_mobility_failure_inventory_v1"
                and inventory.get("run_phase") == phase
                and set(inventory) == inventory_fields
                and all(
                    isinstance(inventory.get(name), list)
                    for name in inventory_fields - {"schema_version", "run_phase"}
                )
            ):
                aggregate_errors.append("failure inventory schema/sections mismatch")
            observed_checks = self_check.get("checks")
            if not (
                set(self_check)
                == {
                    "schema_version",
                    "run_phase",
                    "coverage",
                    "completion_rule",
                    "completion",
                    "input_receipt",
                    "selected_count",
                    "category_accounting",
                    "checks",
                    "diagnostic_counts",
                    "passed",
                    "publication_schema_version",
                    "output_hashes",
                }
                and
                self_check.get("schema_version")
                == "table5_sketch_mobility_aggregate_self_check_v1"
                and self_check.get("publication_schema_version")
                == "table5_sketch_mobility_aggregate_publication_v1"
                and self_check.get("run_phase") == phase
                and self_check.get("selected_count") == len(intent_ids)
                and self_check.get("completion_rule")
                == (
                    "every intent asset has exactly one valid terminal record in every simulator and "
                    "the exact runtime summary/progress receipts are complete"
                )
                and self_check.get("completion")
                == {"state": "complete", "formal_claim_complete": expected_formal}
                and self_check.get("passed") is True
                and isinstance(observed_checks, dict)
                and set(observed_checks) == SELF_CHECK_NAMES
                and all(value is True for value in observed_checks.values())
                and self_check.get("input_receipt") == table5.get("receipt")
            ):
                aggregate_errors.append("aggregate self-check is not exact/passing")
            if deep_validation:
                expected_coverage: dict[str, Any] = {}
                for simulator in SIMULATORS:
                    status_counts: dict[str, int] = {}
                    for record in records_by_simulator[simulator].values():
                        status = record.get("terminal_status")
                        status_counts[status] = status_counts.get(status, 0) + 1
                    expected_coverage[simulator] = {
                        "intent_count": len(intent_ids),
                        "seen_count": len(records_by_simulator[simulator]),
                        "missing_count": 0,
                        "missing_ids": [],
                        "duplicate_count": 0,
                        "terminal_status_counts": dict(sorted(status_counts.items())),
                    }
                if self_check.get("coverage") != expected_coverage:
                    aggregate_errors.append(
                        "aggregate self-check coverage differs from terminal records"
                    )
            output_hashes = self_check.get("output_hashes")
            if not isinstance(output_hashes, dict) or set(output_hashes) != SELF_CHECK_HASH_FILES:
                aggregate_errors.append("aggregate self-check output hashes are not exact")
            else:
                for name in SELF_CHECK_HASH_FILES:
                    if output_hashes[name] != sha256_file(aggregate_root / name):
                        aggregate_errors.append(
                            f"self-check output hash mismatch: {name}"
                        )
        set_check("aggregate_publication_complete", aggregate_errors)

        marker_errors = (
            _aggregate_marker_errors(
                aggregate_root,
                aggregate_set,
                phase=phase,
                protocol_sha256=(manifest or {}).get("protocol_sha256", ""),
                cohort_sha256=(manifest or {}).get("cohort_sha256", ""),
            )
            if aggregate_set is not None and manifest is not None
            else ["aggregate publication marker unavailable"]
        )
        set_check("aggregate_file_hashes", marker_errors)

        aggregate_receipt_errors: list[str] = []
        if isinstance(table5, dict) and isinstance(manifest, dict):
            receipt = table5.get("receipt")
            expected_receipt_fields = {
                "protocol_sha256",
                "cohort_sha256",
                "receipt_set_sha256",
                "protocol_file_sha256",
                "manifest_file_sha256",
                "receipt_root",
                "dataset_root",
                "table1_manifest",
                "upstream_roots",
                "runtime_inputs",
            }
            if not isinstance(receipt, dict) or set(receipt) != expected_receipt_fields:
                aggregate_receipt_errors.append("Table 5 receipt fields are not exact")
            else:
                expected_bindings = {
                    "protocol_sha256": manifest.get("protocol_sha256"),
                    "cohort_sha256": manifest.get("cohort_sha256"),
                    "receipt_set_sha256": sha256_file(root / "receipt_set.json"),
                    "protocol_file_sha256": sha256_file(root / "protocol.json"),
                    "manifest_file_sha256": sha256_file(root / "manifest.json"),
                }
                for field, expected in expected_bindings.items():
                    if receipt.get(field) != expected:
                        aggregate_receipt_errors.append(
                            f"Table 5 receipt mismatch: {field}"
                        )
                for field, expected in (
                    ("receipt_root", root),
                    ("table1_manifest", table1_manifest),
                ):
                    try:
                        matched = Path(receipt.get(field)).resolve(strict=True) == expected
                    except (OSError, TypeError, ValueError):
                        matched = False
                    if not matched:
                        aggregate_receipt_errors.append(
                            f"Table 5 receipt mismatch: {field}"
                        )
                try:
                    if Path(receipt.get("dataset_root")).resolve(strict=True) != Path(
                        manifest["source_receipt"]["dataset_root"]
                    ).resolve(strict=True):
                        aggregate_receipt_errors.append(
                            "Table 5 dataset-root receipt mismatch"
                        )
                except (OSError, TypeError, ValueError, KeyError):
                    aggregate_receipt_errors.append(
                        "Table 5 dataset-root receipt is malformed"
                    )
                upstream_roots = receipt.get("upstream_roots")
                if not isinstance(upstream_roots, dict) or set(upstream_roots) != {
                    "table2",
                    "table3",
                    "table4",
                }:
                    aggregate_receipt_errors.append(
                        "Table 5 upstream-root receipt is malformed"
                    )
                else:
                    for name in upstream_roots:
                        try:
                            observed = Path(upstream_roots[name]).resolve(strict=True)
                            expected = Path(
                                manifest["upstream_artifacts"][name]["root"]
                            ).resolve(strict=True)
                            if observed != expected:
                                aggregate_receipt_errors.append(
                                    f"Table 5 upstream-root receipt mismatch: {name}"
                                )
                        except (OSError, TypeError, ValueError, KeyError):
                            aggregate_receipt_errors.append(
                                f"Table 5 upstream-root receipt is malformed: {name}"
                            )
        else:
            aggregate_receipt_errors.append("Table 5 receipt unavailable")
        set_check("aggregate_receipt_closure", aggregate_receipt_errors)

        if deep_validation and summaries_complete:
            fresh_errors = _fresh_source_closure_errors(
                protocol, manifest, table1_manifest
            )
            set_check("fresh_manifest_upstream_closure", fresh_errors)
            checks["table1_cohort_order"] = not fresh_errors
        else:
            set_check(
                "fresh_manifest_upstream_closure",
                ["fresh source closure deferred because publication inputs are incomplete"],
            )

        table5a_errors: list[str] = []
        table5b_errors: list[str] = []
        category_errors: list[str] = []
        diagnostic_count_errors: list[str] = []
        record_inventory_errors: list[str] = []
        report_errors: list[str] = []
        if deep_validation and summaries_complete:
            recomputed_a, recomputed_b, outcomes = _recompute_headline_tables(
                protocol=protocol,
                rows_by_id=row_by_id,
                records_by_simulator=records_by_simulator,
                intent_ids=intent_ids,
            )
            if table5.get("table5a") != recomputed_a:
                table5a_errors.append(
                    "Table 5a differs from independent record recomputation"
                )
            observed_b = table5.get("table5b")
            expected_b_keys = {
                "per_simulator_pass",
                "all_three_load",
                "all_three_runtime_pass",
                "strict_urdf_pass",
                "strict_kinematic_pass",
                "strict_collision_pass",
                "strict_consistency",
                "strict_sim_ready",
                "joint_rmse",
                "link_pose_error",
            }
            if not isinstance(observed_b, dict) or set(observed_b) != expected_b_keys:
                table5b_errors.append("Table 5b fields are not exact")
            else:
                for field, expected in recomputed_b.items():
                    if observed_b.get(field) != expected:
                        table5b_errors.append(
                            f"Table 5b recomputation mismatch: {field}"
                        )
                if observed_b.get("joint_rmse") != _recompute_joint_rmse(
                    protocol, row_by_id, records_by_simulator, intent_ids
                ):
                    table5b_errors.append("Table 5b joint RMSE differs from records")
                if observed_b.get("link_pose_error") != _recompute_link_pose(
                    protocol, row_by_id, records_by_simulator, intent_ids
                ):
                    table5b_errors.append(
                        "Table 5b link-pose error differs from records"
                    )
            expected_outcomes = [
                {
                    "dataset_id": dataset_id,
                    "asset_id": row_by_id[dataset_id]["manifest_root"],
                    "manifest_root": row_by_id[dataset_id]["manifest_root"],
                    **outcomes[dataset_id],
                }
                for dataset_id in intent_ids
            ]
            if table5.get("asset_outcomes") != expected_outcomes:
                table5b_errors.append(
                    "Table 5 asset outcomes differ from recomputation"
                )
            expected_categories = _recompute_categories(
                protocol=protocol,
                rows_by_id=row_by_id,
                records_by_simulator=records_by_simulator,
                intent_ids=intent_ids,
                outcomes=outcomes,
            )
            category_errors.extend(
                _category_errors(table5.get("categories"), expected_categories)
            )
            expected_category_accounting = {
                "small_group_threshold": expected_categories[
                    "small_group_threshold"
                ],
                "category_count": expected_categories["category_count"],
                "groups": [
                    {
                        "category": group["category"],
                        "size": group["size"],
                        "warnings": group["warnings"],
                    }
                    for group in expected_categories["groups"]
                ],
            }
            if self_check.get("category_accounting") != expected_category_accounting:
                category_errors.append(
                    "aggregate category accounting differs from recomputation"
                )
            expected_diagnostic_counts = _recompute_diagnostic_counts(
                protocol=protocol,
                rows_by_id=row_by_id,
                records_by_simulator=records_by_simulator,
                intent_ids=intent_ids,
            )
            diagnostic_count_errors.extend(
                _diagnostic_count_errors(
                    expected_diagnostic_counts,
                    inventory,
                    self_check.get("diagnostic_counts"),
                )
            )
            record_inventory_errors.extend(
                _record_inventory_errors(
                    inventory.get("records"),
                    _recompute_record_inventory(
                        rows_by_id=row_by_id,
                        records_by_simulator=records_by_simulator,
                        intent_ids=intent_ids,
                    ),
                )
            )
            report_errors.extend(_report_errors(report, table5))
        else:
            table5a_errors.append("Table 5a cannot be recomputed from incomplete inputs")
            table5b_errors.append("Table 5b cannot be recomputed from incomplete inputs")
            category_errors.append(
                "category report cannot be recomputed from incomplete inputs"
            )
            diagnostic_count_errors.append(
                "diagnostic counts cannot be recomputed from incomplete inputs"
            )
            record_inventory_errors.append(
                "failure record inventory cannot be recomputed from incomplete inputs"
            )
            report_errors.append(
                "aggregate report cannot be verified from incomplete inputs"
            )
        set_check("table5a_reaggregation", table5a_errors)
        set_check("table5b_full_reaggregation", table5b_errors)
        set_check("category_full_reaggregation", category_errors)
        set_check("diagnostic_inventory_count_reaggregation", diagnostic_count_errors)
        set_check("failure_record_inventory_reaggregation", record_inventory_errors)
        set_check("report_template_reaggregation", report_errors)
        checks["table5b_strict_gate_reaggregation"] = checks[
            "table5b_full_reaggregation"
        ]
    except Exception as error:
        checks["structured_validation"] = False
        errors.append(
            f"validation contract exception: {type(error).__name__}: {error}"
        )
    else:
        checks["structured_validation"] = True
    return _verification_result(root=root, phase=phase, checks=checks, errors=errors)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", required=True, type=Path)
    parser.add_argument("--phase", required=True, choices=("qualification", "formal"))
    parser.add_argument("--table1-manifest", required=True, type=Path)
    args = parser.parse_args()
    result = verify_publication(
        args.run_root, phase=args.phase, table1_manifest=args.table1_manifest
    )
    print(json.dumps(result, sort_keys=True, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
