#!/usr/bin/env python3
"""Freeze provenance and reset/readback evidence for the PhysX-Omni Table 6 pilot.

This is an attributable exploratory method pilot, not a semantic benchmark.
The eight identities were fixed before outputs were observed in the recorded
session, but no pre-run local selection artifact survives.  The script keeps
that reproducibility failure explicit and never treats zero collision geometry
as a vacuous collision pass.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
from typing import Any
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
DEFAULT_ROOT = REPO / "exp/runtime/table6_physx_omni_method_pilot"
ADAPTER = REPO / ".cache/table6_sources/physx_omni_method/source_snapshot_46fa1cd0_adapter"
DEPENDENCY_INTEGRITY = DEFAULT_ROOT / "dependency_integrity.json"
PARTNET = REPO.parent / "PartNet_Mobility/data/dataset"
LOW_MEDIUM_PYTHON = REPO / "exp/.venv_low_medium/bin/python"
SOURCE_COMMIT = "46fa1cd0b6883d4d14431d51c3326ef80a85ef64"
MODEL_REVISION = "765cd275839f88333cb754f1c6c0b8d3887a3b2c"
TRELLIS_REVISION = "25e0d31ffbebe4b5a97464dd851910efc3002d96"
PROCESSOR_REVISION = "cc594898137f460bfe9f0759e9844b3ce807cfb5"
TEST_LIST_REVISION = "7c87bcc252b33cdc10c8583c20ac899136729ad5"
FROZEN = [
    ("4627", "Display", "full"),
    ("6813", "Clock", "full"),
    ("102001", "Oven", "full"),
    ("103593", "Phone", "full"),
    ("9918", "Laptop", "full"),
    ("102187", "TrashCan", "smoke"),
    ("103514", "Toaster", "full"),
    ("102916", "Pen", "full"),
]
MOVABLE = {"revolute", "continuous", "prismatic"}
TIMESTAMP = re.compile(r"\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}),\d+\]")
RESET_TOLERANCE = 1e-9


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def iso_mtime(path: Path) -> str:
    return datetime.fromtimestamp(path.stat().st_mtime, timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def read_key_values(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}
    rows: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            rows[key.strip()] = value.strip()
    return rows


def log_timestamps(paths: list[Path]) -> dict[str, str]:
    values: list[str] = []
    for path in paths:
        if not path.is_file():
            continue
        for match in TIMESTAMP.finditer(path.read_text(encoding="utf-8", errors="replace")):
            values.append(match.group(1).replace(" ", "T") + "Z")
    return {
        "log_first_timestamp_utc": min(values) if values else "unknown",
        "log_last_timestamp_utc": max(values) if values else "unknown",
    }


def read_stage_timing(log_dir: Path, stage: str) -> dict[str, Any]:
    status = read_key_values(log_dir / f"{stage}.status")
    started_path = log_dir / f"{stage}.started_utc"
    finished_path = log_dir / f"{stage}.finished_utc"
    logs = [log_dir / f"{stage}.method.log", log_dir / f"{stage}.stderr"]
    mtimes = [iso_mtime(path) for path in logs if path.is_file()]
    result: dict[str, Any] = {
        "started_utc": started_path.read_text(encoding="utf-8").strip() if started_path.is_file() else "unknown",
        "finished_utc": finished_path.read_text(encoding="utf-8").strip() if finished_path.is_file() else "unknown",
        "wall_seconds": int(status["wall_seconds"]) if status.get("wall_seconds", "").isdigit() else "unknown",
        "exit_code": int(status["exit_code"]) if re.fullmatch(r"-?\d+", status.get("exit_code", "")) else "unknown",
        "log_mtime_utc": max(mtimes) if mtimes else "unknown",
    }
    result.update(log_timestamps(logs))
    return result


def inventory(root: Path) -> list[dict[str, Any]]:
    if not root.is_dir():
        return []
    rows = []
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        rows.append({
            "path": path.relative_to(root).as_posix(),
            "size": path.stat().st_size,
            "sha256": sha256_file(path),
        })
    return rows


def stage_output_inventory(root: Path, stage: str) -> list[dict[str, Any]]:
    prefixes = {
        "stage1": ("allind.npy", "basic_info.txt", "cond_img.png", "coord_", "ind_"),
        "stage1.retry": ("allind.npy", "basic_info.txt", "cond_img.png", "coord_", "ind_"),
        "stage2": ("objs/",),
        "stage3": ("basic_info.json", "basic.urdf", "basic.xml", "desert.png"),
    }
    accepted = prefixes[stage]
    return [
        row for row in inventory(root)
        if any(row["path"] == prefix or row["path"].startswith(prefix) for prefix in accepted)
    ]


def output_cardinality(root: Path) -> dict[str, Any]:
    declared = sorted(root.glob("ind_*.npy"))
    glbs = sorted(root.glob("objs/*/*.glb"))
    objs = sorted(root.glob("objs/*/*.obj"))
    textures = sorted(root.glob("objs/*/material_0.png"))
    expected = len(declared)
    return {
        "declared_part_count": expected,
        "glb_count": len(glbs),
        "obj_count": len(objs),
        "texture_count": len(textures),
        "stage2_cardinality_match": bool(expected and len(glbs) == len(objs) == len(textures) == expected),
    }


def vector(node: ET.Element | None, key: str, *, default: list[float] | None = None) -> list[float] | None:
    if node is None or node.get(key) is None:
        return default
    try:
        values = [float(item) for item in node.get(key, "").split()]
    except ValueError:
        return None
    return values if len(values) == 3 and all(math.isfinite(item) for item in values) else None


def graph_valid(links: set[str], edges: list[tuple[str, str]]) -> bool:
    indegree = Counter(child for _, child in edges)
    roots = [link for link in links if indegree[link] == 0]
    adjacency: dict[str, list[str]] = defaultdict(list)
    for parent, child in edges:
        adjacency[parent].append(child)
    reached: set[str] = set()
    queue = deque(roots)
    while queue:
        current = queue.popleft()
        if current in reached:
            continue
        reached.add(current)
        queue.extend(adjacency[current])
    return bool(
        len(roots) == 1
        and len(edges) == len(links) - 1
        and len(reached) == len(links)
        and all(value <= 1 for value in indegree.values())
    )


def audit_urdf(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError("URDF root is not robot")
    link_nodes = root.findall("link")
    link_names = [node.get("name", "") for node in link_nodes]
    links = set(link_names)
    edges: list[tuple[str, str]] = []
    joints: list[dict[str, Any]] = []
    for node in root.findall("joint"):
        parent_node = node.find("parent")
        child_node = node.find("child")
        parent = parent_node.get("link", "") if parent_node is not None else ""
        child = child_node.get("link", "") if child_node is not None else ""
        joint_type = node.get("type", "")
        edges.append((parent, child))
        if joint_type not in MOVABLE:
            continue
        axis = vector(node.find("axis"), "xyz")
        origin_xyz = vector(node.find("origin"), "xyz", default=[0.0, 0.0, 0.0])
        origin_rpy = vector(node.find("origin"), "rpy", default=[0.0, 0.0, 0.0])
        limit = node.find("limit")
        lower = upper = None
        if joint_type != "continuous" and limit is not None:
            try:
                lower = float(limit.get("lower", "nan"))
                upper = float(limit.get("upper", "nan"))
            except ValueError:
                pass
        bounded_valid = bool(
            joint_type == "continuous"
            or lower is not None and upper is not None
            and math.isfinite(lower) and math.isfinite(upper) and lower <= upper
        )
        joints.append({
            "name": node.get("name", ""),
            "type": joint_type,
            "parent": parent,
            "child": child,
            "endpoints_valid": parent in links and child in links,
            "axis_valid": axis is not None and sum(item * item for item in axis) > 1e-18,
            "origin_valid": origin_xyz is not None and origin_rpy is not None,
            "bounded_limit_valid": bounded_valid,
            "lower": lower,
            "upper": upper,
            "generic_revolute": bool(
                joint_type == "revolute" and bounded_valid
                and math.degrees(float(upper) - float(lower)) >= 300.0
            ),
        })
    visual_mesh_refs = [node.get("filename", "") for node in root.findall(".//visual/geometry/mesh")]
    missing_visual = [name for name in visual_mesh_refs if not (path.parent / name).resolve().is_file()]
    collision_count = len(root.findall(".//collision"))
    inertial_count = 0
    placeholder_count = 0
    for link in link_nodes:
        inertial = link.find("inertial")
        try:
            mass = float(inertial.find("mass").get("value"))  # type: ignore[union-attr]
            tensor = inertial.find("inertia")  # type: ignore[union-attr]
            diagonal = [float(tensor.get(key)) for key in ("ixx", "iyy", "izz")]  # type: ignore[union-attr]
            off_diagonal = [float(tensor.get(key)) for key in ("ixy", "ixz", "iyz")]  # type: ignore[union-attr]
            valid = mass > 0 and all(item > 0 and math.isfinite(item) for item in diagonal)
            inertial_count += int(valid)
            placeholder_count += int(valid and mass == 1 and diagonal == [1, 1, 1] and off_diagonal == [0, 0, 0])
        except (AttributeError, TypeError, ValueError):
            pass
    return {
        "xml_parse_success": True,
        "link_count": len(link_nodes),
        "joint_count": len(root.findall("joint")),
        "movable_joint_count": len(joints),
        "joint_type_counts": dict(sorted(Counter(row["type"] for row in joints).items())),
        "valid_tree": bool(len(links) == len(link_names) and all(link_names) and graph_valid(links, edges)),
        "parent_child_structurally_valid_count": sum(row["endpoints_valid"] for row in joints),
        "axis_metadata_valid_count": sum(row["axis_valid"] for row in joints),
        "origin_metadata_valid_count": sum(row["origin_valid"] for row in joints),
        "bounded_limit_valid_count": sum(row["bounded_limit_valid"] for row in joints),
        "generic_revolute_count": sum(row["generic_revolute"] for row in joints),
        "revolute_count": sum(row["type"] == "revolute" for row in joints),
        "continuous_count": sum(row["type"] == "continuous" for row in joints),
        "visual_mesh_reference_count": len(visual_mesh_refs),
        "missing_visual_mesh_reference_count": len(missing_visual),
        "missing_visual_mesh_references": missing_visual,
        "collision_element_count": collision_count,
        "collision_evaluation_status": "EVALUABLE" if collision_count else "N/A_ZERO_COLLISION_GEOMETRY",
        "valid_inertial_link_count": inertial_count,
        "placeholder_inertial_link_count": placeholder_count,
        "joints": joints,
    }


def run_pybullet_audit(urdf: Path, output: Path) -> dict[str, Any]:
    command = [str(LOW_MEDIUM_PYTHON), str(SCRIPT), "--child", str(urdf), str(output)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=180)
    if result.returncode != 0 or not output.is_file():
        return {
            "status": "FAIL",
            "exit_code": result.returncode,
            "stderr": result.stderr[-4000:],
            "stdout": result.stdout[-4000:],
        }
    return json.loads(output.read_text(encoding="utf-8"))


def child_audit(urdf: Path, output: Path) -> None:
    import pybullet as pybullet

    client = pybullet.connect(pybullet.DIRECT)
    try:
        flags = pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
        body = pybullet.loadURDF(str(urdf), useFixedBase=True, flags=flags, physicsClientId=client)
        movable = []
        for index in range(pybullet.getNumJoints(body, physicsClientId=client)):
            info = pybullet.getJointInfo(body, index, physicsClientId=client)
            if info[2] not in (pybullet.JOINT_REVOLUTE, pybullet.JOINT_PRISMATIC):
                continue
            pybullet.setJointMotorControl2(
                body, index, pybullet.VELOCITY_CONTROL, force=0, physicsClientId=client
            )
            lower, upper = float(info[8]), float(info[9])
            if info[2] == pybullet.JOINT_REVOLUTE and upper < lower:
                lower, upper = -math.pi, math.pi
            if not math.isfinite(lower) or not math.isfinite(upper) or upper < lower:
                continue
            movable.append((index, info[1].decode("utf-8", errors="replace"), lower, upper))
        states = []
        for index, name, lower, upper in movable:
            targets = [lower + (upper - lower) * fraction for fraction in (0.0, 0.5, 1.0)]
            max_error = 0.0
            for target in targets:
                pybullet.resetJointState(body, index, target, physicsClientId=client)
                pybullet.performCollisionDetection(physicsClientId=client)
                actual = float(pybullet.getJointState(body, index, physicsClientId=client)[0])
                max_error = max(max_error, abs(actual - target))
            states.append({"joint_name": name, "state_count": len(targets), "max_readback_error": max_error})
        collision_shapes = sum(
            len(pybullet.getCollisionShapeData(body, link, physicsClientId=client) or [])
            for link in [-1] + list(range(pybullet.getNumJoints(body, physicsClientId=client)))
        )
        payload = {
            "status": "PASS",
            "protocol": "disable motors; resetJointState; performCollisionDetection; readback; no stepSimulation",
            "load_success": True,
            "movable_joint_count": len(movable),
            "state_count": sum(row["state_count"] for row in states),
            "max_readback_error": max((row["max_readback_error"] for row in states), default=0.0),
            "readback_pass": all(row["max_readback_error"] <= RESET_TOLERANCE for row in states),
            "collision_shape_count": collision_shapes,
            "collision_evaluation_status": "EVALUABLE" if collision_shapes else "N/A_ZERO_COLLISION_GEOMETRY",
            "joint_records": states,
        }
        write_json(output, payload)
    finally:
        pybullet.disconnect(client)


def find_method_log(root: Path, item_id: str, stage: str, source: str) -> list[Path]:
    log_dir = root / source / "logs" if source == "smoke" else root / "full/logs" / item_id
    return [log_dir / f"{stage}.method.log", log_dir / f"{stage}.stderr"]


def stage_record(root: Path, item_id: str, source: str, stage: str, status: str, marker: str) -> dict[str, Any]:
    log_dir = root / source / "logs" if source == "smoke" else root / "full/logs" / item_id
    timing = read_stage_timing(log_dir, stage)
    paths = find_method_log(root, item_id, stage, source)
    text = "\n".join(path.read_text(encoding="utf-8", errors="replace") for path in paths if path.is_file())
    artifacts = stage_output_inventory(
        (root / "smoke/generated" if source == "smoke" else root / "full/generated") / item_id,
        stage,
    )
    log_inventory = [
        {
            "path": str(path.relative_to(root)),
            "size": path.stat().st_size,
            "sha256": sha256_file(path),
        }
        for path in paths if path.is_file()
    ]
    invocation = log_dir / f"{stage}.invocation.txt"
    invocation_record = (
        {
            "path": str(invocation.relative_to(root)),
            "size": invocation.stat().st_size,
            "sha256": sha256_file(invocation),
        }
        if invocation.is_file() else None
    )
    return {
        "item_id": item_id,
        "stage": stage,
        "status": status,
        "method_success_marker": marker in text,
        "expected_marker": marker,
        **timing,
        "log_files": log_inventory,
        "invocation_record": invocation_record,
        "output_inventory_sha256": canonical_sha256(artifacts),
        "output_file_count": len(artifacts),
    }


def main(root: Path) -> int:
    root = root.resolve()
    dependency = json.loads((root / "dependency_integrity.json").read_text(encoding="utf-8"))
    frozen_rows = [{"item_id": item_id, "category": category, "source": source} for item_id, category, source in FROZEN]
    independent_audit_path = root / "independent_protocol_audit.json"
    independent_audit_markdown_path = root / "independent_protocol_audit.md"
    independent_audit = json.loads(independent_audit_path.read_text(encoding="utf-8"))
    manifest = {
        "protocol": "nano3d_table6_physx_omni_exploratory_attributable_pilot_v1",
        "intent_count": 8,
        "identity_rows": frozen_rows,
        "identity_rows_sha256": canonical_sha256(frozen_rows),
        "identity_fixed_before_observed_outputs_per_session_execution_record": True,
        "failure_policy": "retain all eight intended IDs; no outcome replacement",
        "selection_reproducibility": "NOT_EVIDENCED",
        "selection_reproducibility_check": "FAIL",
        "selection_gate_severity": "HIGH",
        "selection_limitation": "no pre-run local selection artifact survives; do not claim salt argmin or formal outcome-independent benchmark sampling",
        "benchmark_positioning": "exploratory attributable pilot on IDs from an official mobility test list",
        "held_out_claim": False,
        "held_out_gate": "FAIL_HIGH_NO_TRAINING_EXCLUSION_MANIFEST",
        "training_overlap_risk": "POSSIBLE: official training configuration names PhysX-Mobility; exclusion of these IDs is not evidenced",
        "independent_semantic_gold": False,
        "publication_gate": False,
        "publication_gate_status": "FAIL_CLOSED_EXPLORATORY_ONLY",
        "source_commit": SOURCE_COMMIT,
        "model_revision": MODEL_REVISION,
        "trellis_revision": TRELLIS_REVISION,
        "processor_revision": PROCESSOR_REVISION,
        "test_list_revision": TEST_LIST_REVISION,
        "dependency_integrity_status": dependency.get("status"),
        "dependency_integrity_pass_count": dependency.get("pass_count"),
        "dependency_integrity_total_count": dependency.get("total_count"),
        "qwen_processor_primary_manifest_covered": False,
        "qwen_processor_separate_blob_check": "PASS_11_OF_11",
        "operational_adapter_disclosure": {
            "pristine_official_tracked_files_match": "334/334",
            "modified_source_file_count": 5,
            "purpose": "deterministic ordering/seeds, local dependency routing, checked subprocesses, and fail-fast evidence",
        },
        "independent_protocol_audit_sha256": sha256_file(independent_audit_path),
        "independent_protocol_audit_markdown_sha256": sha256_file(independent_audit_markdown_path),
        "independent_protocol_audit_verdict": independent_audit.get("overall_verdict"),
    }
    write_json(root / "manifest.json", manifest)

    stage_rows: list[dict[str, Any]] = []
    asset_rows: list[dict[str, Any]] = []
    successes = {"4627", "6813", "103593", "9918", "102187", "103514", "102916"}
    for item_id, category, source in FROZEN:
        generated = (root / "smoke/generated" if source == "smoke" else root / "full/generated") / item_id
        if item_id == "102001":
            row = stage_record(root, item_id, source, "stage1", "INCOMPLETE_RETAINED", "success: 102001")
            row["failure_reason"] = "shared batch was interrupted after nine part outputs; no allind.npy or success marker; finish and wall time are unknown"
            row["partial_output_retained"] = True
            stage_rows.append(row)
            asset_rows.append({
                "item_id": item_id,
                "category": category,
                "intent_status": "RETAINED_FAILURE",
                "method_output_status": "N/E_STAGE1_INCOMPLETE",
                "static_status": "N/E",
                "pybullet_status": "N/E",
                "output_inventory": inventory(generated),
            })
            continue
        stage1 = stage_record(root, item_id, source, "stage1", "PASS", f"success: {item_id}")
        if source == "smoke":
            # The smoke log is the direct method stderr and has the marker.
            stage1["method_success_marker"] = any(
                f"success: {item_id}" in path.read_text(encoding="utf-8", errors="replace")
                for path in find_method_log(root, item_id, "stage1", source) if path.is_file()
            )
        if item_id == "102916":
            retry_dir = root / "full/logs/102916"
            stage1 = stage_record(root, item_id, source, "stage1.retry", "PASS_AFTER_HARNESS_RETRY", f"success: {item_id}")
            stage1["prior_harness_failure"] = {
                "exit_code": read_key_values(retry_dir / "stage1.status").get("exit_code", "unknown"),
                "reason": "relative 1vlm_demo.py path was absent from the per-ID cwd; not a method failure",
            }
        stage_rows.append(stage1)
        stage_rows.append(stage_record(root, item_id, source, "stage2", "PASS", f"success: {item_id}"))
        stage_rows.append(stage_record(root, item_id, source, "stage3", "PASS", f"complete: {item_id}"))

        urdf = generated / "basic.urdf"
        asset: dict[str, Any] = {
            "item_id": item_id,
            "category": category,
            "intent_status": "SUCCESS" if item_id in successes else "RETAINED_FAILURE",
            "method_output_status": "ATTRIBUTABLE_COMPLETE",
            "output_inventory": inventory(generated),
            "output_cardinality": output_cardinality(generated),
            "semantic_diagnostic": (
                {
                    "upstream_category": "TrashCan",
                    "generated_name": "Double Door Refrigerator",
                    "generated_category": "Appliance",
                    "exact_category_consistency": False,
                    "interpretation": "structural generation success only; not semantic success",
                }
                if item_id == "102187" else "N/E_NO_FROZEN_INDEPENDENT_ADJUDICATION"
            ),
        }
        try:
            static = audit_urdf(urdf)
            asset["static_status"] = "PASS"
            asset["urdf_sha256"] = sha256_file(urdf)
            asset["static"] = static
            child_output = root / f"pybullet_{item_id}.json"
            asset["pybullet"] = run_pybullet_audit(urdf, child_output)
            asset["pybullet_status"] = asset["pybullet"].get("status", "FAIL")
        except Exception as error:
            asset["static_status"] = "FAIL"
            asset["static_error"] = f"{type(error).__name__}: {error}"
            asset["pybullet_status"] = "N/E"
        asset_rows.append(asset)

    write_json(root / "stage_records.json", stage_rows)
    write_json(root / "asset_records.json", asset_rows)
    complete = [row for row in asset_rows if row.get("method_output_status") == "ATTRIBUTABLE_COMPLETE"]
    static_ok = [row for row in complete if row.get("static_status") == "PASS"]
    pybullet_ok = [row for row in complete if row.get("pybullet_status") == "PASS"]
    static_rows = [row["static"] for row in static_ok]
    movable = sum(row["movable_joint_count"] for row in static_rows)
    collisions = sum(row["collision_element_count"] for row in static_rows)
    summary = {
        "protocol": manifest["protocol"],
        "status": "EXPLORATORY_PARTIAL",
        "publication_gate": False,
        "publication_gate_reason": "HIGH gates fail for selection reproducibility and held-out/training-exclusion evidence",
        "intent_assets": 8,
        "attributable_complete_assets": len(complete),
        "retained_failed_assets": 8 - len(complete),
        "static_pass_assets": len(static_ok),
        "pybullet_load_reset_readback_pass_assets": len(pybullet_ok),
        "declared_movable_joints": movable,
        "mean_movable_joints_per_complete_asset": movable / len(complete) if complete else None,
        "parent_child_structurally_valid_joints": sum(row["parent_child_structurally_valid_count"] for row in static_rows),
        "axis_metadata_valid_joints": sum(row["axis_metadata_valid_count"] for row in static_rows),
        "origin_metadata_valid_joints": sum(row["origin_metadata_valid_count"] for row in static_rows),
        "bounded_limit_metadata_valid_joints": sum(row["bounded_limit_valid_count"] for row in static_rows),
        "generic_revolute_joints": sum(row["generic_revolute_count"] for row in static_rows),
        "revolute_joints": sum(row["revolute_count"] for row in static_rows),
        "continuous_joints": sum(row["continuous_count"] for row in static_rows),
        "collision_elements": collisions,
        "collision_metrics": "N/A_ZERO_COLLISION_GEOMETRY" if not collisions else "N/E_NOT_RUN",
        "continuous_collision_detection": "N/A_ZERO_COLLISION_GEOMETRY" if not collisions else "N/E_NOT_RUN",
        "valid_inertial_links": sum(row["valid_inertial_link_count"] for row in static_rows),
        "uniform_placeholder_inertial_links": sum(row["placeholder_inertial_link_count"] for row in static_rows),
        "physical_inertia_fidelity": "N/A_UNIFORM_PLACEHOLDERS",
        "reset_readback_states": sum(row.get("pybullet", {}).get("state_count", 0) for row in pybullet_ok),
        "max_reset_readback_error": max((row.get("pybullet", {}).get("max_readback_error", 0.0) for row in pybullet_ok), default=None),
        "selection_reproducibility": "NOT_EVIDENCED",
        "selection_reproducibility_check": "FAIL",
        "selection_gate_severity": "HIGH",
        "held_out_claim": False,
        "held_out_gate": "FAIL_HIGH_NO_TRAINING_EXCLUSION_MANIFEST",
        "qwen_processor_primary_manifest_covered": False,
        "qwen_processor_separate_blob_check": "PASS_11_OF_11",
        "independent_semantic_gold": False,
        "semantic_joint_type_accuracy": "N/A",
        "expected_joint_recall": "N/A",
        "semantic_parent_child_accuracy": "N/A",
        "semantic_axis_accuracy": "N/A",
        "semantic_origin_accuracy": "N/A",
        "semantic_limit_accuracy": "N/A",
        "stage2_cardinality_match_assets": sum(row.get("output_cardinality", {}).get("stage2_cardinality_match", False) for row in complete),
        "observed_semantic_diagnostic": "102187 upstream TrashCan -> generated Double Door Refrigerator; aggregate semantic accuracy remains N/A",
        "manifest_sha256": sha256_file(root / "manifest.json"),
        "stage_records_sha256": sha256_file(root / "stage_records.json"),
        "asset_records_sha256": sha256_file(root / "asset_records.json"),
        "independent_protocol_audit_sha256": manifest["independent_protocol_audit_sha256"],
        "independent_protocol_audit_markdown_sha256": manifest["independent_protocol_audit_markdown_sha256"],
    }
    write_json(root / "summary.json", summary)

    terminal_audit = independent_audit.get("terminal_evidence_audit", {})
    audited_stage = terminal_audit.get("stage_records", {})
    audited_assets = terminal_audit.get("asset_records", {})
    checks = {
        "manifest_intent_is_eight": manifest["intent_count"] == 8,
        "identity_categories_are_unique": len({row["category"] for row in frozen_rows}) == 8,
        "dependency_integrity_passes": dependency.get("status") == "PASS" and dependency.get("pass_count") == dependency.get("total_count") == 45,
        "failed_id_retained": any(row["item_id"] == "102001" and row["intent_status"] == "RETAINED_FAILURE" for row in asset_rows),
        "no_outcome_replacement": {row["item_id"] for row in asset_rows} == {row[0] for row in FROZEN},
        "all_successes_have_three_stage_records": all(sum(row["item_id"] == item_id for row in stage_rows) == 3 for item_id in successes),
        "all_complete_outputs_parse": len(static_ok) == len(complete),
        "all_complete_outputs_pybullet_pass": len(pybullet_ok) == len(complete),
        "all_complete_stage2_cardinalities_match": all(row["output_cardinality"]["stage2_cardinality_match"] for row in complete),
        "all_stage2_markers_present": all(row["method_success_marker"] for row in stage_rows if row["stage"] == "stage2"),
        "all_stage3_markers_present": all(row["method_success_marker"] for row in stage_rows if row["stage"] == "stage3"),
        "all_complete_outputs_have_zero_collision": collisions == 0,
        "collision_result_is_na": summary["collision_metrics"] == "N/A_ZERO_COLLISION_GEOMETRY",
        "semantic_gold_is_not_claimed": not manifest["independent_semantic_gold"],
        "held_out_is_not_claimed": not manifest["held_out_claim"],
        "selection_reproducibility_not_evidenced": manifest["selection_reproducibility"] == "NOT_EVIDENCED",
        "independent_audit_is_terminal": independent_audit.get("protocol") == "table6_physx_omni_method_independent_protocol_audit_v2_terminal",
        "independent_audit_record_hashes_match": (
            audited_stage.get("sha256") == summary["stage_records_sha256"]
            and audited_assets.get("sha256") == summary["asset_records_sha256"]
        ),
        "independent_audit_terminal_denominators_match": (
            audited_stage.get("stage1_intent_count") == summary["intent_assets"]
            and audited_stage.get("stage1_success_count") == summary["attributable_complete_assets"]
            and audited_stage.get("stage1_incomplete_count") == summary["retained_failed_assets"]
            and audited_assets.get("attributable_complete_count") == summary["attributable_complete_assets"]
            and audited_assets.get("static_valid_tree_count") == summary["static_pass_assets"]
            and audited_assets.get("pybullet_load_reset_readback_pass_count") == summary["pybullet_load_reset_readback_pass_assets"]
            and audited_assets.get("collision_evaluable_asset_count") == 0
            and audited_assets.get("independent_semantic_gold_asset_count") == 0
        ),
    }
    formal_gates = {
        "selection_reproducibility": False,
        "held_out_training_exclusion": False,
        "qwen_processor_in_primary_dependency_manifest": False,
        "runtime_environment_frozen": False,
        "per_item_stage1_invocation_complete": False,
        "independent_semantic_gold": False,
        "nonvacuous_collision_geometry": collisions > 0,
    }
    execution_checks_passed = all(checks.values())
    self_check = {
        "protocol": "nano3d_table6_physx_omni_exploratory_pilot_self_check_v1",
        "status": "EXPLORATORY_PARTIAL_FAIL_CLOSED",
        "passed": False,
        "publication_gate": False,
        "execution_checks_passed": execution_checks_passed,
        "checks": checks,
        "formal_publication_gates": formal_gates,
        "selection_reproducibility_check": "FAIL",
        "selection_reproducibility_note": "HIGH gate failure: this prevents formal benchmark-cohort positioning but does not invalidate attributable execution evidence.",
        "manifest_sha256": summary["manifest_sha256"],
        "stage_records_sha256": summary["stage_records_sha256"],
        "asset_records_sha256": summary["asset_records_sha256"],
        "independent_protocol_audit_sha256": summary["independent_protocol_audit_sha256"],
        "independent_protocol_audit_markdown_sha256": summary["independent_protocol_audit_markdown_sha256"],
        "summary_sha256": sha256_file(root / "summary.json"),
    }
    write_json(root / "self_check.json", self_check)

    report = f"""# PhysX-Omni Table 6 exploratory method pilot

## Positioning

Status: `EXPLORATORY_PARTIAL`; `publication_gate=false`. This is an attributable exploratory pilot on eight identities from a PhysX-Anything mobility test list. It is **not** a held-out claim: the released PhysX-Omni training configuration names PhysX-Mobility, and public evidence does not establish that these IDs were excluded. It also has no independent semantic joint gold.

The session record fixed all eight IDs before outputs were observed and retained the failed ID without replacement. However, no pre-run local selection artifact survives, so `selection_reproducibility=NOT_EVIDENCED` and the reproducibility check is `FAIL`. Do not describe this as a reproducible salt-argmin or formal outcome-independent benchmark cohort.

## Result

- Intent: 8 assets / 8 PartNet categories.
- Attributable complete outputs: {summary['attributable_complete_assets']}/8; retained incomplete: {summary['retained_failed_assets']}/8 (`102001`, Stage1 interrupted after nine part outputs without `allind.npy` or a success marker).
- Static XML: {summary['static_pass_assets']}/{summary['attributable_complete_assets']} complete outputs parse as valid trees.
- Articulation: {summary['declared_movable_joints']} movable joints, mean {summary['mean_movable_joints_per_complete_asset']:.3f} per complete asset.
- Structural metadata: parent/child {summary['parent_child_structurally_valid_joints']}/{summary['declared_movable_joints']}; axis {summary['axis_metadata_valid_joints']}/{summary['declared_movable_joints']}; origin {summary['origin_metadata_valid_joints']}/{summary['declared_movable_joints']}; bounded/continuous limit metadata {summary['bounded_limit_metadata_valid_joints']}/{summary['declared_movable_joints']}.
- PyBullet reset/readback: {summary['pybullet_load_reset_readback_pass_assets']}/{summary['attributable_complete_assets']} complete outputs; {summary['reset_readback_states']} states; max error {summary['max_reset_readback_error']} m/rad-equivalent joint coordinate units. Protocol disables motors, then uses `resetJointState -> performCollisionDetection -> readback`, with no `stepSimulation`.
- Collision: {summary['collision_elements']} collision elements, therefore penetration/full-range/CCD are `N/A`, not a vacuous pass.
- Inertia: {summary['uniform_placeholder_inertial_links']}/{summary['valid_inertial_links']} syntactically valid inertials are uniform placeholders; physical inertia fidelity is `N/A`.
- Semantic diagnostic: upstream `102187` is TrashCan, but its generated output calls itself `Double Door Refrigerator` / `Appliance`. This is retained as a structural generation success and an observed semantic mismatch, never as semantic accuracy.
- Independent semantic type/recall/parent/axis/origin/limit accuracy: `N/A`.

## Provenance

- Source commit: `{SOURCE_COMMIT}`. Execution used a disclosed five-file operational adapter for deterministic ordering/seeds, explicit local dependency routing, checked subprocesses, and fail-fast evidence; this was not an untouched-checkout execution.
- PhysX-Omni model revision: `{MODEL_REVISION}`; TRELLIS revision: `{TRELLIS_REVISION}`; Qwen processor revision: `{PROCESSOR_REVISION}`.
- Dependency integrity: original manifest {dependency.get('pass_count')}/{dependency.get('total_count')} `PASS`. It omits the invoked Qwen processor/tokenizer directory; the independent audit separately verifies 11/11 files at `{PROCESSOR_REVISION}`, so primary-manifest coverage remains a formal `FAIL`.
- Per-stage explicit times are retained when present. Missing historical start/finish/wall fields remain `unknown`; method-log timestamps and file mtimes are separate evidence fields and are not promoted into exact wall times.
- Terminal evidence chain: independent audit JSON SHA-256 `{summary['independent_protocol_audit_sha256']}`; audit Markdown SHA-256 `{summary['independent_protocol_audit_markdown_sha256']}`; `manifest.json` SHA-256 `{summary['manifest_sha256']}`; `summary.json` SHA-256 `{sha256_file(root / 'summary.json')}`. The independent audit pins `stage_records.json` SHA-256 `{summary['stage_records_sha256']}` and `asset_records.json` SHA-256 `{summary['asset_records_sha256']}`.

See `manifest.json`, `stage_records.json`, `asset_records.json`, `summary.json`, and `self_check.json` for machine-readable denominators, hashes, and limitations.
"""
    (root / "report.md").write_text(report, encoding="utf-8")
    self_check["report_sha256"] = sha256_file(root / "report.md")
    write_json(root / "self_check.json", self_check)
    # The runner succeeds when execution evidence was generated consistently;
    # publication gates remain false and are intentionally not conflated with
    # process exit status.
    return 0 if execution_checks_passed else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--child", nargs=2, metavar=("URDF", "OUTPUT"))
    arguments = parser.parse_args()
    if arguments.child:
        child_audit(Path(arguments.child[0]).resolve(), Path(arguments.child[1]).resolve())
    else:
        sys.exit(main(arguments.root))
