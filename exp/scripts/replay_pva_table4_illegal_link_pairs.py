#!/usr/bin/env python3
"""Low-resource diagnostic replay of PV-A Table 4 illegal link pairs."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
import sys
import tempfile
from typing import Any, Mapping
import zlib

SCRIPT = Path(__file__).resolve()
EXP = SCRIPT.parent.parent
if str(SCRIPT.parent) not in sys.path:
    sys.path.insert(0, str(SCRIPT.parent))
import run_urdf_table4_partnet_mobility as core

V1_DB = EXP / "runtime/pva_table1234_full_release_20260826/evaluation/results.sqlite3"
V2_ROOT = Path("/tmp/pva_table4_mimic_aware_full_release_20260827")
OUTPUT = EXP / "runtime/pva_table4_illegal_link_pair_replay_20260827.json"
PRIORITY = (
    "Stationary_Folder",
    "pictureX_0611_drawing_compass_with_adjustable_legs",
    "rack_and_pinion_slider",
    "standing_desk_with_synchronous_telescoping_legs_and_articulated_controls",
    "watch", "Bench_Wood_Swing", "Door_folding_door", "Chair_Folding_chair",
    "Industrial_Ore_crusher_jaw", "pictureX_0611_Cabinet_with_doors",
    "pictureX_0611_Cabinet_with_doors_and_drawers",
    "pictureX_0611_Industrial_rolling_work_table",
)
DEPTH_EQUIVALENCE_TOLERANCE_M = 1e-12


def canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def states(blob: bytes) -> list[dict[str, Any]]:
    return [json.loads(line) for line in zlib.decompress(blob).splitlines()]


def classify_replay(source_count: int, replay_count: int, depth_delta: float) -> str:
    if source_count == replay_count and depth_delta <= DEPTH_EQUIVALENCE_TOLERANCE_M:
        return "exact"
    if depth_delta <= DEPTH_EQUIVALENCE_TOLERANCE_M:
        return "manifold_count_only"
    return "partial_non_equivalent"


def self_hash_valid(artifact: Mapping[str, Any]) -> bool:
    payload = dict(artifact); declared = payload.pop("artifact_content_sha256", None)
    return declared == hashlib.sha256(canonical(payload).encode()).hexdigest()


def verify_manifest_self_hash(manifest: Mapping[str, Any]) -> dict[str, Any]:
    payload = dict(manifest)
    declared = payload.pop("manifest_content_sha256", None)
    computed = hashlib.sha256(canonical(payload).encode()).hexdigest()
    if declared != computed:
        raise RuntimeError("v2 manifest self-hash mismatch")
    return {"declared": declared, "computed": computed, "match": True}


def verify_runtime_contract(
    expected: Mapping[str, Any], observed: Mapping[str, Any]
) -> dict[str, Any]:
    expected_keys = set(expected)
    observed_keys = set(observed)
    if expected_keys != observed_keys:
        missing = sorted(expected_keys - observed_keys)
        extra = sorted(observed_keys - expected_keys)
        raise RuntimeError(f"runtime identity field-set mismatch: missing={missing}, extra={extra}")
    matches = {key: observed[key] == expected[key] for key in sorted(expected)}
    if not all(matches.values()):
        raise RuntimeError(f"runtime identity mismatch: {matches}")
    return {"field_matches": matches, "exact_field_set_match": True, "all_fields_match": True}


def mirror_urdf(row: Mapping[str, Any], physical_root: Path) -> Path:
    relative = Path(str(row["primary_urdf_relative_path"]))
    parts = relative.parts[1:] if relative.parts and relative.parts[0] == "extracted" else relative.parts
    path = (physical_root / Path(*parts)).resolve(strict=True)
    if sha256_file(path) != row["primary_urdf_sha256"]:
        raise RuntimeError(f"mirror URDF hash mismatch: {path}")
    return path


def schedule(urdf: Path, version: str) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    joints = core.parse_urdf_joints(urdf)
    plan = core.compile_joint_sampling_plan(joints) if version == "v2" else None
    independent = list(plan["independent_joints"]) if plan else joints
    values_by_state: list[list[float]] = []
    descriptors: list[dict[str, Any]] = []
    def add(phase: str, index: int, joint: str | None, independent_values: list[float]) -> None:
        values = core.expand_joint_values(plan, independent_values) if plan else independent_values
        values_by_state.append([float(value) for value in values])
        descriptors.append({"phase": phase, "sample_index": index, "joint_name": joint, "joint_values_sha256": core.canonical_sha256(values)})
    add("rest", 0, None, [0.0] * len(independent))
    for position, joint in enumerate(independent):
        if not bool(joint.get("sampling_range_evaluable", joint.get("range_evaluable"))):
            continue
        for index, value in enumerate(core.single_joint_values(joint)):
            vector = [0.0] * len(independent); vector[position] = float(value)
            add("single_joint_sweep", index, str(joint["name"]), vector)
    if independent and all(bool(row.get("sampling_range_evaluable", row.get("range_evaluable"))) for row in independent):
        for index, vector in enumerate(core.sobol_joint_values(independent, seed=core.SOBOL_SEED)):
            add("multi_joint_sobol", index, None, [float(value) for value in vector])
    return descriptors, values_by_state


def replay(urdf: Path, descriptors: list[dict[str, Any]], values: list[list[float]], targets: list[tuple[int, dict[str, Any]]]) -> list[dict[str, Any]]:
    import pybullet as bullet
    client = bullet.connect(bullet.DIRECT)
    body = None
    output = []
    try:
        flags = int(bullet.URDF_USE_INERTIA_FROM_FILE | bullet.URDF_USE_SELF_COLLISION | bullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT | bullet.URDF_IGNORE_VISUAL_SHAPES)
        body = bullet.loadURDF(str(urdf), useFixedBase=True, flags=flags, physicsClientId=client)
        by_name = {}
        link_names = {-1: bullet.getBodyInfo(body, physicsClientId=client)[0].decode("utf-8", "replace")}
        for index in range(bullet.getNumJoints(body, physicsClientId=client)):
            info = bullet.getJointInfo(body, index, physicsClientId=client)
            by_name[info[1].decode()] = index
            link_names[index] = info[12].decode("utf-8", "replace")
        parsed = core.parse_urdf_joints(urdf)
        joint_indices = [by_name[str(row["name"])] for row in parsed]
        for index in joint_indices:
            bullet.setJointMotorControl2(body, index, bullet.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0, physicsClientId=client)
        direct = core._direct_parent_pairs(bullet, body, client)
        for state_index, source_state in targets:
            descriptor = descriptors[state_index]
            for field in ("phase", "sample_index", "joint_name"):
                if descriptor[field] != source_state.get(field):
                    raise RuntimeError(f"regenerated state descriptor mismatch: {field}")
            if descriptor["joint_values_sha256"] != source_state["joint_values_sha256"]:
                raise RuntimeError("regenerated state joint_values_sha256 mismatch")
            for joint_index, value in zip(joint_indices, values[state_index], strict=True):
                bullet.resetJointState(body, joint_index, value, targetVelocity=0.0, physicsClientId=client)
            bullet.performCollisionDetection(physicsClientId=client)
            illegal = []
            for contact in bullet.getContactPoints(bodyA=body, bodyB=body, physicsClientId=client):
                a, b = int(contact[3]), int(contact[4]); depth = max(0.0, -float(contact[8]))
                if frozenset((a, b)) in direct or depth <= core.PENETRATION_THRESHOLD_M:
                    continue
                illegal.append({"link_a_index": a, "link_a_name": link_names.get(a), "link_b_index": b, "link_b_name": link_names.get(b), "penetration_depth_m": depth})
            replayed_max = max((row["penetration_depth_m"] for row in illegal), default=0.0)
            depth_delta = abs(replayed_max - float(source_state["non_adjacent_max_penetration_m"]))
            replay_class = classify_replay(
                int(source_state["non_adjacent_illegal_penetration_count"]),
                len(illegal), depth_delta,
            )
            output.append({
                "phase": descriptor["phase"], "state_index": state_index,
                "sample_index": descriptor["sample_index"], "joint_name": descriptor["joint_name"],
                "joint_values": values[state_index], "joint_values_sha256": descriptor["joint_values_sha256"],
                "source_non_adjacent_illegal_count": source_state["non_adjacent_illegal_penetration_count"],
                "source_non_adjacent_max_penetration_m": source_state["non_adjacent_max_penetration_m"],
                "replayed_illegal_contact_count": len(illegal),
                "replayed_max_penetration_m": replayed_max,
                "illegal_count_exact_match": len(illegal) == source_state["non_adjacent_illegal_penetration_count"],
                "maximum_depth_abs_delta_m": depth_delta,
                "replay_classification": replay_class,
                "illegal_contacts": illegal,
            })
        return output
    finally:
        if body is not None: bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)


def collect(database: Path, version: str, physical_root: Path) -> dict[str, Any]:
    uri = f"{database.resolve(strict=True).as_uri()}?mode=ro" + ("&immutable=1" if version == "v1" else "")
    connection = sqlite3.connect(uri, uri=True); connection.execute("PRAGMA query_only=ON"); connection.execute("BEGIN")
    roster_connection = connection if version == "v1" else sqlite3.connect(f"{V1_DB.resolve().as_uri()}?mode=ro&immutable=1", uri=True)
    output = []
    try:
        record_col, blob_col = ("table4_json", "table4_states_zlib") if version == "v1" else ("record_json", "states_zlib")
        watermark = None
        if version == "v2":
            digest = hashlib.sha256(); count = 0; maximum = None
            for ordinal, asset_id, record_text, blob, state_count, worker_status in connection.execute(
                f"SELECT ordinal,asset_id,{record_col},{blob_col},state_count,worker_status FROM results ORDER BY ordinal"
            ):
                count += 1; maximum = int(ordinal)
                for value in (str(ordinal).encode(), str(asset_id).encode(), str(record_text).encode(), bytes(blob), str(state_count).encode(), str(worker_status).encode()):
                    digest.update(len(value).to_bytes(8, "big")); digest.update(value)
            watermark = {"row_count": count, "max_ordinal": maximum, "framed_full_row_content_sha256": digest.hexdigest(), "framing": "uint64-be length + ordinal/asset_id/record_text/compressed_state_blob/state_count/worker_status"}
        for category in PRIORITY:
            if version == "v1":
                query = f"""SELECT r.ordinal,r.{record_col},r.{blob_col} FROM results r
                  JOIN assets a USING(ordinal) WHERE a.category=? AND
                  COALESCE(json_extract(r.{record_col},'$.strict_collision_pass'),0)=0
                  AND json_extract(r.{record_col},'$.measurement_complete')=1 ORDER BY r.ordinal LIMIT 1"""
                row = connection.execute(query, (category,)).fetchone()
            else:
                query = f"""SELECT ordinal,{record_col},{blob_col} FROM results WHERE
                  json_extract({record_col},'$.category')=? AND
                  COALESCE(json_extract({record_col},'$.strict_collision_pass'),0)=0
                  AND json_extract({record_col},'$.measurement_complete')=1 ORDER BY ordinal LIMIT 1"""
                row = connection.execute(query, (category,)).fetchone()
            if row is None: continue
            ordinal, record_text, blob = row
            roster = json.loads(roster_connection.execute("SELECT row_json FROM assets WHERE ordinal=?", (ordinal,)).fetchone()[0])
            record = json.loads(record_text); source_states = states(blob)
            urdf = mirror_urdf(roster, physical_root)
            descriptors, values = schedule(urdf, version)
            if len(descriptors) != len(source_states): raise RuntimeError("schedule/state count mismatch")
            targets = []
            seen = set()
            for index, state in enumerate(source_states):
                phase = state["phase"]
                if phase not in seen and int(state["non_adjacent_illegal_penetration_count"]) > 0:
                    targets.append((index, state)); seen.add(phase)
            state_identity = [
                {field: state.get(field) for field in ("phase", "sample_index", "joint_name", "joint_values_sha256")}
                for state in source_states
            ]
            output.append({"version": version, "ordinal": record["order"], "asset_id": record["dataset_id"], "category": category, "urdf": str(urdf), "urdf_sha256": roster["primary_urdf_sha256"], "source_provenance": {"record_text_sha256": hashlib.sha256(str(record_text).encode()).hexdigest(), "compressed_state_blob_sha256": hashlib.sha256(bytes(blob)).hexdigest(), "source_state_identity_sha256": hashlib.sha256(canonical(state_identity).encode()).hexdigest(), "source_state_count": len(source_states)}, "states": replay(urdf, descriptors, values, targets)})
        found = sorted(result["category"] for result in output)
        return {"results": output, "selection": {"requested_categories": list(PRIORITY), "found_categories": found, "missing_categories": sorted(set(PRIORITY) - set(found))}, "watermark": watermark}
    finally:
        connection.rollback(); connection.close()
        if roster_connection is not connection: roster_connection.close()


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--output", type=Path, default=OUTPUT); parser.add_argument("--v2-root", type=Path, default=V2_ROOT)
    args = parser.parse_args(); manifest_path = args.v2_root / "manifest.json"; manifest = json.loads(manifest_path.read_text()); manifest_self_hash = verify_manifest_self_hash(manifest); physical = Path(manifest["package_root_binding"]["physical_root"])
    import pybullet as bullet
    import scipy
    expected_runtime = manifest["runtime_identity"]
    observed_runtime = {
        "collision_core_sha256": sha256_file(Path(core.__file__).resolve()),
        "python_executable": str(Path(sys.executable).resolve()),
        "python_version": sys.version,
        "runner_sha256": sha256_file(SCRIPT.parent / "run_table4_full_release.py"),
        "scipy_version": scipy.__version__,
        "pybullet_api_version": bullet.getAPIVersion(),
        "pybullet_module": str(Path(bullet.__file__).resolve()),
        "pybullet_module_sha256": sha256_file(Path(bullet.__file__).resolve()),
    }
    runtime_verification = verify_runtime_contract(expected_runtime, observed_runtime)
    receipt_path = V1_DB.parent / "full_release_receipt.json"
    receipt = json.loads(receipt_path.read_text())
    receipt_payload = dict(receipt); declared_receipt_hash = receipt_payload.pop("receipt_content_sha256")
    if hashlib.sha256(canonical(receipt_payload).encode()).hexdigest() != declared_receipt_hash:
        raise RuntimeError("v1 receipt self-hash mismatch")
    v1_database_sha = sha256_file(V1_DB)
    if v1_database_sha != receipt["result_database_sha256"]:
        raise RuntimeError("v1 database/receipt SHA256 mismatch")
    v1 = collect(V1_DB, "v1", physical); v2 = collect(args.v2_root / "results.sqlite3", "v2", physical)
    results = v1["results"] + v2["results"]
    replayed_states = [state for result in results for state in result["states"]]
    if not replayed_states or any(not state["illegal_contacts"] for state in replayed_states):
        raise RuntimeError("diagnostic self-check: every selected state must reproduce an illegal contact")
    if any(contact["penetration_depth_m"] <= core.PENETRATION_THRESHOLD_M for state in replayed_states for contact in state["illegal_contacts"]):
        raise RuntimeError("diagnostic self-check: emitted contact is not illegal")
    classifications = {name: sum(state["replay_classification"] == name for state in replayed_states) for name in ("exact", "manifold_count_only", "partial_non_equivalent")}
    artifact = {"schema_version": "pva_table4_illegal_link_pair_diagnostic_replay_v2", "resource_policy": {"processes": 1, "worker_pool": False, "recommended_thread_environment": {"OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}}, "inputs": {"v1_database": str(V1_DB), "v1_database_sha256": v1_database_sha, "v1_receipt": str(receipt_path), "v1_receipt_file_sha256": sha256_file(receipt_path), "v1_receipt_content_sha256": declared_receipt_hash, "v1_database_receipt_binding_verified": True, "v2_database": str(args.v2_root / "results.sqlite3"), "v2_manifest": str(manifest_path), "v2_manifest_file_sha256": sha256_file(manifest_path), "v2_manifest_self_hash": manifest_self_hash, "v2_selection_watermark": v2["watermark"], "physical_mirror": str(physical)}, "runtime_identity": {"expected": expected_runtime, "observed": observed_runtime, **runtime_verification, "runner_source_path": str(SCRIPT.parent / "run_table4_full_release.py"), "runner_source_verified": True}, "selection": {"v1": v1["selection"], "v2": v2["selection"]}, "summary": {"asset_replays": len(results), "state_replays": len(replayed_states), "replay_classification_counts": classifications, "depth_equivalence_tolerance_m": DEPTH_EQUIVALENCE_TOLERANCE_M, "classification_semantics": {"exact": "illegal count equal and maximum-depth delta <= tolerance", "manifold_count_only": "illegal count differs but maximum-depth delta <= tolerance", "partial_non_equivalent": "maximum-depth delta exceeds tolerance; proves an illegal pair at the bound state but is not a complete equivalent replay"}}, "results": results, "self_checks": {"all_selected_states_reproduced_illegal_contacts": True, "all_emitted_depths_exceed_threshold": True, "source_phase_sample_joint_checked_before_each_replay": True, "joint_value_hash_checked_before_each_replay": True, "source_record_blob_and_state_identity_hashed": True}, "implementation": {"script": str(SCRIPT), "script_sha256": sha256_file(SCRIPT)}}
    artifact["artifact_content_sha256"] = hashlib.sha256(canonical(artifact).encode()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True); fd, name = tempfile.mkstemp(prefix=f".{args.output.name}.", dir=args.output.parent)
    with os.fdopen(fd, "w", encoding="ascii") as stream: stream.write(canonical(artifact) + "\n")
    os.replace(name, args.output); print(canonical(artifact)); return 0


if __name__ == "__main__": raise SystemExit(main())
