#!/usr/bin/env python3
"""Independent semantic verifier for SketchMobility Table 4 receipts."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import sys
from typing import Any, Callable
import xml.etree.ElementTree as ET

from scipy.stats import qmc


SCRIPT_PATH = Path(__file__).resolve()
PROTOCOL_ID = "urdf-sim-ready-table4-sketch-mobility-table1-cohort-v1"
SOURCE_PIN_SET_ID = "table4-sketch-mobility-source-pins-v1"
DATASET_NAME = "SketchMobility"
FORMAL_N_EVAL = 800
FORMAL_J_EVAL = 1824
FORMAL_N_RELEASE = 4956
SINGLE_SAMPLES = 21
SOBOL_SAMPLES = 64
SOBOL_SEED = 20260813
ZERO_WIDTH_TOLERANCE = 1e-12
FORMAL_PYTHON_VERSION = "3.12.3"
FORMAL_PYBULLET_VERSION = "3.2.7"
FORMAL_PYBULLET_API_VERSION = 202010061
FORMAL_NUMPY_VERSION = "2.5.1"
FORMAL_SCIPY_VERSION = "1.18.0"
FORMAL_COLLISION_HARNESS_SHA256 = "e09bd4b7050bfb0e854f2462a27fdbc40ef3d59bb71040b7c73576a0fec3b84f"
FORMAL_COLLISION_CORE_SHA256 = "e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0"
FORMAL_TABLE1_MANIFEST_SHA256 = "081e9e9125f8945cad67a751949e659f6d4e73817704c07cd3fcd4b657ffc696"
FORMAL_TABLE2_MANIFEST_SHA256 = "0be3e21f079bd86ba9ab680f1d709dd676b623bea01d8e43a3db85943a64a8e5"
FORMAL_TABLE2_RECORDS_SHA256 = "03b6d5e0d335052f123664a7a85dcdbc33ffbad8143ffb4bb62560e9b44ea2d1"
FORMAL_TABLE3_MANIFEST_SHA256 = "0f90fbdec03cf4be69dc2b870b2aa7eaa3c00de93e49c005394e402907276f4a"
FORMAL_TABLE3_CONTENT_SHA256 = "3b151753d17c77e68e1ee7993bdf643867b0a1bef9142df3b17abca1fbda6ab8"
FORMAL_TABLE3_RECORDS_SHA256 = "13124125cbdef565efc95c7526e052576aead73fa6499d7b0b81bcc0490a24f7"
SOURCE_FILENAMES = {
    "adapter": "run_table4_urdf_sketch_mobility.py",
    "independent_verifier": "verify_table4_urdf_sketch_mobility.py",
    "collision_harness": "run_urdf_table4_ours_500k.py",
    "collision_core": "run_urdf_table4_partnet_mobility.py",
    "table3_adapter": "run_table3_urdf_sketch_mobility.py",
    "table3_child_harness": "run_urdf_table3_ours_500k.py",
    "table3_core": "run_urdf_table3_lam.py",
    "table3_verifier": "verify_table3_urdf_sketch_mobility.py",
    "source_pins": "table4_sketch_mobility_source_pins_v1.json",
}
IDENTITY_FIELDS = (
    "asset_id",
    "raw_category",
    "source",
    "selection_hash",
    "selection_rank",
)
FROZEN_INPUT_FIELDS = (
    "protocol_id",
    "order",
    "dataset_id",
    *IDENTITY_FIELDS,
    "category",
    "asset_root_relpath",
    "primary_urdf_relpath",
    "urdf_sha256",
    "package_content_manifest_sha256",
    "package_audit_success",
    "audit_issue",
    "valid_tree",
    "movable_dof_count",
    "range_evaluable_dof_count",
    "joint_specs_sha256",
    "joint_specs",
    "collision_mesh_inventory_sha256",
    "missing_collision_mesh_reference_count",
    "unsafe_collision_mesh_reference_count",
    "scale_derivation_sha256",
    "object_bbox_diagonal_m",
    "rest_state_expected",
    "single_state_expected",
    "sobol_state_expected",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise ValueError(f"blank JSONL row in {path} at line {line_number}")
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"non-object JSONL row in {path} at line {line_number}")
            rows.append(value)
    return rows


def atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def validate_source_bundle(output: Path, manifest: dict[str, Any]) -> dict[str, str]:
    declared = manifest["evaluation"]["source_snapshots"]
    if set(declared) != set(SOURCE_FILENAMES):
        raise ValueError("source snapshot roles mismatch")
    observed: dict[str, str] = {}
    for role, filename in SOURCE_FILENAMES.items():
        target = output / "source_snapshot" / filename
        if target.is_symlink() or not target.is_file():
            raise ValueError(f"source snapshot missing or symlinked: {role}")
        if Path(str(declared[role]["path"])).resolve(strict=True) != target.resolve(
            strict=True
        ):
            raise ValueError(f"source snapshot path mismatch: {role}")
        observed[role] = sha256_file(target)
        if observed[role] != declared[role]["sha256"]:
            raise ValueError(f"source snapshot hash mismatch: {role}")
    pin_path = output / "source_snapshot" / SOURCE_FILENAMES["source_pins"]
    pins = read_json(pin_path)
    if (
        pins.get("schema_version") != 1
        or pins.get("pin_set_id") != SOURCE_PIN_SET_ID
        or pins.get("sources")
        != {role: digest for role, digest in observed.items() if role != "source_pins"}
    ):
        raise ValueError("source pin set does not close the executable snapshots")
    if (
        manifest["evaluation"].get("source_pin_set_id") != SOURCE_PIN_SET_ID
        or manifest["evaluation"].get("source_pin_set_sha256")
        != observed["source_pins"]
    ):
        raise ValueError("manifest source pin binding mismatch")
    if observed["independent_verifier"] != sha256_file(SCRIPT_PATH):
        raise ValueError("executing verifier differs from frozen verifier")
    return observed


def _safe_package(dataset_root: Path, asset_id: str) -> Path:
    relative = PurePosixPath(asset_id)
    if (
        relative.is_absolute()
        or len(relative.parts) != 4
        or relative.parts[0] != "data"
        or any(part in {"", ".", ".."} for part in relative.parts)
    ):
        raise ValueError(f"invalid asset ID: {asset_id}")
    cursor = dataset_root
    for part in relative.parts:
        cursor = cursor / part
        if cursor.is_symlink():
            raise ValueError(f"asset path contains symlink: {asset_id}")
    package = cursor.resolve(strict=True)
    package.relative_to(dataset_root)
    if not package.is_dir():
        raise ValueError(f"asset package is not a directory: {asset_id}")
    return package


def package_binding(package: Path) -> dict[str, Any]:
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(
        package, followlinks=False
    ):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            if (current / name).is_symlink():
                raise ValueError("package contains directory symlink")
        for name in file_names:
            path = current / name
            if path.is_symlink() or not path.is_file():
                raise ValueError("package contains non-regular file")
            files.append(
                {
                    "path": path.relative_to(package).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
    return {
        "files": files,
        "file_count": len(files),
        "total_bytes": sum(int(row["bytes"]) for row in files),
        "content_manifest_sha256": canonical_sha256(files),
    }


def collision_inventory(package: Path, urdf_path: Path) -> tuple[list[dict[str, Any]], int]:
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
            candidate = (urdf_path.parent / reference).resolve(strict=False)
            try:
                candidate.relative_to(urdf_path.parent)
                candidate.relative_to(package)
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
                    path.relative_to(package).as_posix() if path is not None else None
                ),
                "exists": exists,
                "is_symlink": bool(path is not None and path.is_symlink()),
                "size_bytes": path.stat().st_size if exists and path is not None else None,
                "sha256": sha256_file(path) if exists and path is not None else None,
            }
        )
    return inventory, unsafe


def joint_interval(row: dict[str, Any]) -> tuple[float, float]:
    if row["type"] == "continuous":
        return -math.pi, math.pi
    lower = row.get("lower")
    upper = row.get("upper")
    if not isinstance(lower, (int, float)) or not isinstance(upper, (int, float)):
        raise ValueError("joint lacks finite limits")
    lower_f, upper_f = float(lower), float(upper)
    if not math.isfinite(lower_f) or not math.isfinite(upper_f) or upper_f - lower_f <= ZERO_WIDTH_TOLERANCE:
        raise ValueError("joint has invalid range")
    return lower_f, upper_f


def parse_joints(urdf_path: Path) -> list[dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    rows: list[dict[str, Any]] = []
    for xml_index, node in enumerate(root.findall("joint")):
        joint_type = str(node.get("type", ""))
        if joint_type == "fixed":
            continue
        limit = node.find("limit")
        lower = upper = None
        if joint_type in {"revolute", "prismatic"} and limit is not None:
            try:
                lower = float(limit.get("lower", ""))
                upper = float(limit.get("upper", ""))
            except ValueError:
                lower = upper = None
        row = {
            "xml_index": xml_index,
            "name": node.get("name", f"joint_{xml_index}"),
            "type": joint_type,
            "lower": lower,
            "upper": upper,
        }
        try:
            joint_interval(row)
            row["range_evaluable"] = True
        except ValueError:
            row["range_evaluable"] = False
        rows.append(row)
    return rows


def validate_packages(manifest: dict[str, Any]) -> bool:
    dataset_root = Path(manifest["source"]["dataset_root"]).resolve(strict=True)
    for item in manifest["items"]:
        package = _safe_package(dataset_root, str(item["asset_id"]))
        binding = package_binding(package)
        if binding["content_manifest_sha256"] != item["package_content_manifest_sha256"]:
            raise ValueError(f"package binding mismatch: {item['asset_id']}")
        urdf_path = dataset_root / item["primary_urdf_relpath"]
        if sha256_file(urdf_path) != item["urdf_sha256"]:
            raise ValueError(f"URDF binding mismatch: {item['asset_id']}")
        joints = parse_joints(urdf_path)
        if joints != item["joint_specs"] or canonical_sha256(joints) != item["joint_specs_sha256"]:
            raise ValueError(f"joint spec mismatch: {item['asset_id']}")
        inventory, unsafe = collision_inventory(package, urdf_path)
        if unsafe != int(item["unsafe_collision_mesh_reference_count"]):
            raise ValueError(f"unsafe inventory mismatch: {item['asset_id']}")
        if canonical_sha256(inventory) != item["collision_mesh_inventory_sha256"]:
            raise ValueError(f"collision inventory mismatch: {item['asset_id']}")
    return True


def validate_upstream(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    source = manifest["source"]
    table1 = Path(source["table1_receipt"]) / "manifest.json"
    table2 = Path(source["table2_manifest"])
    table2_records_path = table2.parent / "asset_records.jsonl"
    table3_root = Path(source["table3_receipt"])
    table3 = table3_root / "manifest.json"
    table3_records_path = table3_root / "asset_records.jsonl"
    observed_hashes = {
        "table1": sha256_file(table1),
        "table2": sha256_file(table2),
        "table2_records": sha256_file(table2_records_path),
        "table3": sha256_file(table3),
        "table3_records": sha256_file(table3_records_path),
    }
    expected_hashes = {
        "table1": FORMAL_TABLE1_MANIFEST_SHA256,
        "table2": FORMAL_TABLE2_MANIFEST_SHA256,
        "table2_records": FORMAL_TABLE2_RECORDS_SHA256,
        "table3": FORMAL_TABLE3_MANIFEST_SHA256,
        "table3_records": FORMAL_TABLE3_RECORDS_SHA256,
    }
    if observed_hashes != expected_hashes:
        raise ValueError("fixed upstream hash mismatch")
    table2_manifest = read_json(table2)
    table3_manifest = read_json(table3)
    if table3_manifest.get("manifest_content_sha256") != manifest_self_hash(table3_manifest):
        raise ValueError("Table 3 manifest self-hash mismatch")
    if table3_manifest.get("manifest_content_sha256") != FORMAL_TABLE3_CONTENT_SHA256:
        raise ValueError("Table 3 content hash mismatch")
    table2_rows = table2_manifest["assets"]
    table2_records = read_jsonl(table2_records_path)
    table3_frozen = table3_manifest["records"]
    table3_records = read_jsonl(table3_records_path)
    count = int(manifest["sample_size"])
    if not all(len(rows) == FORMAL_N_EVAL for rows in (table2_rows, table2_records, table3_frozen, table3_records)):
        raise ValueError("upstream denominator mismatch")
    for rank, (item, t2, t2r, t3, t3r) in enumerate(
        zip(
            manifest["items"],
            table2_rows[:count],
            table2_records[:count],
            table3_frozen[:count],
            table3_records[:count],
            strict=True,
        ),
        start=1,
    ):
        expected = {
            "asset_id": t2["asset_id"],
            "selection_rank": rank,
            "selection_hash": t2["selection_hash"],
            "category": t2["source_category"],
            "urdf_sha256": t2["primary_urdf_sha256"],
            "package_content_manifest_sha256": t2r[
                "package_content_manifest_sha256"
            ],
        }
        for row in (item, t3, t3r):
            if {key: row.get(key) for key in expected} != expected:
                raise ValueError(f"upstream row mismatch at rank {rank}")
        if item["input_identity_sha256"] != canonical_sha256(
            {key: item.get(key) for key in FROZEN_INPUT_FIELDS}
        ):
            raise ValueError(f"item identity hash mismatch at rank {rank}")
    return table2_rows[:count]


def state_identities(item: dict[str, Any]) -> list[dict[str, Any]]:
    joints = item["joint_specs"]
    rest = [0.0] * len(joints)
    identities = [
        {
            "phase": "rest",
            "joint_name": None,
            "sample_index": 0,
            "joint_values_sha256": canonical_sha256(rest),
        }
    ]
    for position, joint in enumerate(joints):
        if not joint["range_evaluable"]:
            continue
        lower, upper = joint_interval(joint)
        for index in range(SINGLE_SAMPLES):
            values = list(rest)
            values[position] = lower + index * (upper - lower) / (SINGLE_SAMPLES - 1)
            identities.append(
                {
                    "phase": "single_joint_sweep",
                    "joint_name": joint["name"],
                    "sample_index": index,
                    "joint_values_sha256": canonical_sha256(values),
                }
            )
    if joints and all(joint["range_evaluable"] for joint in joints):
        intervals = [joint_interval(joint) for joint in joints]
        unit = qmc.Sobol(d=len(joints), scramble=True, seed=SOBOL_SEED).random_base2(m=6)
        for index, vector in enumerate(unit):
            values = [
                float(lower + scalar * (upper - lower))
                for scalar, (lower, upper) in zip(vector, intervals)
            ]
            identities.append(
                {
                    "phase": "multi_joint_sobol",
                    "joint_name": None,
                    "sample_index": index,
                    "joint_values_sha256": canonical_sha256(values),
                }
            )
    return identities


def validate_record(item: dict[str, Any], record: dict[str, Any], states: list[dict[str, Any]], pins: dict[str, str]) -> None:
    frozen_expected = {
        "protocol_id": item["protocol_id"],
        "order": item["order"],
        "dataset_id": item["dataset_id"],
        "category": item["category"],
        "input_identity_sha256": item["input_identity_sha256"],
        "movable_dof_count": item["movable_dof_count"],
        "range_evaluable_dof_count": item["range_evaluable_dof_count"],
        "rest_state_expected": item["rest_state_expected"],
        "single_state_expected": item["single_state_expected"],
        "sobol_state_expected": item["sobol_state_expected"],
        "object_bbox_diagonal_m": item["object_bbox_diagonal_m"],
        "runner_sha256": pins["adapter"],
        "collision_core_sha256": pins["collision_core"],
    }
    if {key: record.get(key) for key in frozen_expected} != frozen_expected:
        raise ValueError("record frozen binding mismatch")
    for key in (
        "asset_id",
        "selection_hash",
        "selection_rank",
        "package_content_manifest_sha256",
        "effective_child_environment_sha256",
    ):
        if record.get(key) != item.get(key):
            raise ValueError(f"record provenance mismatch: {key}")
    executed = {
        "rest": int(record["rest_state_executed"]),
        "single": int(record["single_state_executed"]),
        "sobol": int(record["sobol_state_executed"]),
    }
    expected = {
        "rest": int(record["rest_state_expected"]),
        "single": int(record["single_state_expected"]),
        "sobol": int(record["sobol_state_expected"]),
    }
    if any(not 0 <= executed[key] <= expected[key] for key in expected):
        raise ValueError("executed state counter outside denominator")
    if len(states) != sum(executed.values()):
        raise ValueError("state count mismatch")
    observed_identities = [
        {
            "phase": state.get("phase"),
            "joint_name": state.get("joint_name"),
            "sample_index": state.get("sample_index"),
            "joint_values_sha256": state.get("joint_values_sha256"),
        }
        for state in states
    ]
    if observed_identities != state_identities(item)[: len(states)]:
        raise ValueError("state identity does not match frozen sampling sequence")
    phase_map = {
        "rest": "rest",
        "single": "single_joint_sweep",
        "sobol": "multi_joint_sobol",
    }
    for key, phase in phase_map.items():
        if sum(state["phase"] == phase for state in states) != executed[key]:
            raise ValueError(f"state phase count mismatch: {phase}")
    if any(state.get("dataset_id") != item["dataset_id"] for state in states):
        raise ValueError("state dataset binding mismatch")
    for state in states:
        for prefix in ("all_pair", "non_adjacent"):
            contacts = int(state[f"{prefix}_contact_count"])
            illegal = int(state[f"{prefix}_illegal_penetration_count"])
            maximum = float(state[f"{prefix}_max_penetration_m"])
            if not 0 <= illegal <= contacts or maximum < 0:
                raise ValueError("invalid collision counters")
        metric_key = "all_pair_max_penetration_m" if state["phase"] == "rest" else "non_adjacent_max_penetration_m"
        if not math.isclose(float(state["metric_max_penetration_m"]), float(state[metric_key]), rel_tol=0.0, abs_tol=1e-15):
            raise ValueError("state metric penetration policy mismatch")
    free = {
        "rest": sum(state["non_adjacent_illegal_penetration_count"] == 0 for state in states if state["phase"] == "rest"),
        "single": sum(state["non_adjacent_illegal_penetration_count"] == 0 for state in states if state["phase"] == "single_joint_sweep"),
        "sobol": sum(state["non_adjacent_illegal_penetration_count"] == 0 for state in states if state["phase"] == "multi_joint_sobol"),
    }
    if free["rest"] != int(record["rest_non_adjacent_free"]) or free["single"] != int(record["single_non_adjacent_free"]) or free["sobol"] != int(record["sobol_non_adjacent_free"]):
        raise ValueError("free-state counter mismatch")
    if canonical_sha256(states) != record["state_records_sha256"]:
        raise ValueError("state digest mismatch")
    load_success = bool(record["load_success"])
    measurement_complete = bool(
        load_success
        and int(record["range_evaluable_dof_count"]) == int(record["movable_dof_count"])
        and sum(executed.values()) == sum(expected.values())
    )
    expected_flags = {
        "measurement_complete": measurement_complete,
        "rest_all_pair_cf": bool(load_success and executed["rest"] == expected["rest"] and all(int(state["all_pair_illegal_penetration_count"]) == 0 for state in states if state["phase"] == "rest")),
        "rest_non_adjacent_cf": bool(load_success and executed["rest"] == expected["rest"] and free["rest"] == expected["rest"]),
        "single_joint_sweep_cf": bool(load_success and executed["single"] == expected["single"] and free["single"] == expected["single"]),
        "multi_joint_sobol_cf": bool(int(record["movable_dof_count"]) > 0 and int(record["range_evaluable_dof_count"]) == int(record["movable_dof_count"]) and executed["sobol"] == expected["sobol"] and free["sobol"] == expected["sobol"]),
    }
    expected_flags["strict_collision_pass"] = bool(
        expected_flags["measurement_complete"]
        and expected_flags["rest_non_adjacent_cf"]
        and expected_flags["single_joint_sweep_cf"]
        and expected_flags["multi_joint_sobol_cf"]
    )
    if any(bool(record[key]) != value for key, value in expected_flags.items()):
        raise ValueError("derived asset flag mismatch")
    by_joint: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for state in states:
        if state["phase"] == "single_joint_sweep":
            by_joint[str(state["joint_name"])].append(state)
    joint_passes = sum(
        len(rows) == SINGLE_SAMPLES
        and all(int(state["non_adjacent_illegal_penetration_count"] == 0) for state in rows)
        for rows in by_joint.values()
    )
    if joint_passes != int(record["joint_single_sweep_cf_passed"]):
        raise ValueError("joint sweep pass count mismatch")
    if states:
        maximum = max(float(state["metric_max_penetration_m"]) for state in states)
        if not math.isclose(maximum, float(record["max_penetration_m"]), rel_tol=0.0, abs_tol=1e-15):
            raise ValueError("max penetration mismatch")
        if not math.isclose(maximum / float(record["object_bbox_diagonal_m"]), float(record["max_penetration_normalized"]), rel_tol=1e-12, abs_tol=1e-15):
            raise ValueError("normalized penetration mismatch")
    elif record.get("max_penetration_m") is not None:
        raise ValueError("empty state stream has penetration value")


def record_closure(output: Path, manifest: dict[str, Any], pins: dict[str, str]) -> list[dict[str, Any]]:
    records = read_jsonl(output / "asset_records.jsonl")
    states = read_jsonl(output / "state_records.jsonl")
    if len(records) != len(manifest["items"]):
        raise ValueError("record denominator mismatch")
    by_asset: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for state in states:
        by_asset[str(state.get("dataset_id"))].append(state)
    full: list[dict[str, Any]] = []
    for item, record in zip(manifest["items"], records, strict=True):
        asset_states = by_asset.pop(str(item["dataset_id"]), [])
        validate_record(item, record, asset_states, pins)
        full.append({**record, "state_records": asset_states})
    if by_asset:
        raise ValueError("state rows contain unknown assets")
    return full


def rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def pass_metric(records: list[dict[str, Any]], key: str) -> dict[str, Any]:
    passed = sum(bool(record[key]) for record in records)
    return {"passed": passed, "denominator": len(records), "rate": rate(passed, len(records))}


def core_aggregate(records: list[dict[str, Any]], cohort_label: str) -> dict[str, Any]:
    rest_expected = sum(int(row["rest_state_expected"]) for row in records)
    single_expected = sum(int(row["single_state_expected"]) for row in records)
    sobol_expected = sum(int(row["sobol_state_expected"]) for row in records)
    total_expected = rest_expected + single_expected + sobol_expected
    rest_free = sum(int(row["rest_non_adjacent_free"]) for row in records)
    single_free = sum(int(row["single_non_adjacent_free"]) for row in records)
    sobol_free = sum(int(row["sobol_non_adjacent_free"]) for row in records)
    executed = sum(int(row["rest_state_executed"]) + int(row["single_state_executed"]) + int(row["sobol_state_executed"]) for row in records)
    observed_collision = executed - rest_free - single_free - sobol_free
    maximum_values = [float(row["max_penetration_normalized"]) for row in records if row.get("max_penetration_normalized") is not None]
    fully_measured = [float(row["max_penetration_normalized"]) for row in records if row.get("max_penetration_normalized") is not None and row["measurement_complete"]]
    metrics = {
        "rest_all_pair_cf": pass_metric(records, "rest_all_pair_cf"),
        "rest_non_adjacent_cf": pass_metric(records, "rest_non_adjacent_cf"),
        "single_joint_sweep_cf": pass_metric(records, "single_joint_sweep_cf"),
        "multi_joint_sobol_cf": pass_metric(records, "multi_joint_sobol_cf"),
        "collision_state_rate": {
            "collision_states": total_expected - rest_free - single_free - sobol_free,
            "denominator": total_expected,
            "rate": rate(total_expected - rest_free - single_free - sobol_free, total_expected),
            "definition": "fail-closed collision-or-unexecuted configurations / frozen expected configurations",
            "observed_collision_states": observed_collision,
            "executed_states": executed,
            "unexecuted_states": total_expected - executed,
            "observed_collision_rate_executed": rate(observed_collision, executed),
        },
        "aor": {"status": "N/E", "reason": "no stable exact overlap-volume implementation; bounding-box overlap is not substituted"},
        "max_penetration": {
            "maximum_observed_normalized": max(maximum_values) if maximum_values else None,
            "observed_assets": len(maximum_values),
            "fully_measured_assets": len(fully_measured),
            "denominator": len(records),
            "status": "COMPLETE" if len(fully_measured) == len(records) else "PARTIAL",
            "normalization": "release bounding_box.json diagonal",
        },
        "collision_free_range": {"passed_states": single_free, "denominator": single_expected, "rate": rate(single_free, single_expected)},
        "strict_collision_pass": pass_metric(records, "strict_collision_pass"),
    }
    by_category: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in records:
        by_category[str(row.get("category") or "__PACKAGE_AUDIT_FAILED__")].append(row)
    category_results = {category: core_aggregate(rows, category)["metrics"] for category, rows in sorted(by_category.items())} if len(by_category) > 1 else {}
    category_macro: dict[str, float | None] = {}
    if category_results:
        for key in ("rest_all_pair_cf", "rest_non_adjacent_cf", "single_joint_sweep_cf", "multi_joint_sobol_cf", "strict_collision_pass"):
            values = [row[key]["rate"] for row in category_results.values() if row[key]["rate"] is not None]
            category_macro[key] = sum(values) / len(values) if values else None
        values = [row["collision_free_range"]["rate"] for row in category_results.values() if row["collision_free_range"]["rate"] is not None]
        category_macro["collision_free_range"] = sum(values) / len(values) if values else None
    return {
        "protocol_id": PROTOCOL_ID,
        "status": "COMPLETE" if all(row["measurement_complete"] for row in records) else "COMPLETE_WITH_RETAINED_FAILURES",
        "cohort": {
            "label": cohort_label,
            "selected": len(records),
            "load_success": sum(bool(row["load_success"]) for row in records),
            "measurement_complete": sum(bool(row["measurement_complete"]) for row in records),
            "category_count": len(by_category),
            "child_timeouts": sum(bool(row["child_timed_out"]) for row in records),
        },
        "metrics": metrics,
        "category_macro": category_macro,
        "category_results": category_results,
        "claim_boundary": {
            "continuous_collision_detection": "not_run",
            "semantic_joint_correctness": "not_evaluated",
            "physical_dynamics_validity": "not_evaluated",
            "full_release_result": False,
            "shared_category_balanced_result": False,
        },
    }


def aggregate_summary(manifest: dict[str, Any], records: list[dict[str, Any]], completed_at: str) -> dict[str, Any]:
    summary = core_aggregate(records, manifest["cohort_label"])
    summary["metrics"]["max_penetration"]["normalization"] = (
        "PyBullet q0 collision-shape union AABB diagonal "
        "(pybullet_q0_collision_shape_union_aabb_v1)"
    )
    summary.update(
        {
            "dataset": DATASET_NAME,
            "n_release": FORMAL_N_RELEASE,
            "n_eval": len(records),
            "j_eval": sum(int(item["movable_dof_count"]) for item in manifest["items"]),
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "completed_at": completed_at,
        }
    )
    return summary


def report_text(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    metrics = summary["metrics"]

    def fraction(metric: dict[str, Any], numerator: str = "passed") -> str:
        if metric.get("rate") is None:
            return "N/E"
        return f"{metric[numerator]} / {metric['denominator']} ({100.0 * metric['rate']:.3f}%)"

    maximum = metrics["max_penetration"]
    max_text = "N/E" if maximum["maximum_observed_normalized"] is None else f"{maximum['maximum_observed_normalized']:.6f} ({maximum['fully_measured_assets']} / {maximum['denominator']} measured; {maximum['status']})"
    return "\n".join(
        [
            "# SketchMobility Table 4: Collision and Mechanical Clearance",
            "",
            f"Run classification: **{manifest['classification']}**.",
            "",
            f"Exact frozen cohort: N_eval={summary['n_eval']}, J_eval={summary['j_eval']}; rest q=0; single-joint K={SINGLE_SAMPLES}; Sobol R={SOBOL_SAMPLES} (seed {SOBOL_SEED}); threshold 1e-06 m.",
            "",
            "| Metric | Result |",
            "|---|---:|",
            f"| Rest All-pair CF | {fraction(metrics['rest_all_pair_cf'])} |",
            f"| Rest Non-adjacent CF | {fraction(metrics['rest_non_adjacent_cf'])} |",
            f"| Single-joint Sweep CF | {fraction(metrics['single_joint_sweep_cf'])} |",
            f"| Multi-joint Sobol CF | {fraction(metrics['multi_joint_sobol_cf'])} |",
            f"| Collision-state Rate | {fraction(metrics['collision_state_rate'], 'collision_states')} |",
            "| AOR | N/E |",
            f"| Max Penetration | {max_text} |",
            f"| Collision-free Range | {fraction(metrics['collision_free_range'], 'passed_states')} |",
            f"| Strict Collision Pass | {fraction(metrics['strict_collision_pass'])} |",
            "",
            "Unexecuted states remain in the denominator and count as non-free. AOR is N/E; no bounding-box overlap substitute was used.",
            "",
        ]
    )


def verify_artifacts(output: Path) -> bool:
    symlinks = [path.relative_to(output).as_posix() for path in output.rglob("*") if path.is_symlink()]
    if symlinks:
        raise ValueError(f"receipt contains symlink: {sorted(symlinks)}")
    artifact_path = output / "artifact_manifest.json"
    if not artifact_path.is_file():
        return True
    artifact = read_json(artifact_path)
    expected = {"artifact_manifest.json"}
    for entry in artifact["artifacts"]:
        path = output / entry["path"]
        if not path.is_file() or path.stat().st_size != int(entry["bytes"]) or sha256_file(path) != entry["sha256"]:
            raise ValueError(f"artifact mismatch: {entry['path']}")
        expected.add(entry["path"])
    observed = {path.relative_to(output).as_posix() for path in output.rglob("*") if path.is_file()}
    if expected != observed:
        raise ValueError("artifact file set mismatch")
    return True


def verify_output(output_root: Path, *, write_receipt: bool) -> dict[str, Any]:
    output = output_root.resolve(strict=True)
    checks: dict[str, bool] = {}
    errors: dict[str, str] = {}

    def check(name: str, operation: Callable[[], Any]) -> Any:
        try:
            value = operation()
            if value is False:
                raise ValueError("check returned false")
            checks[name] = True
            return value
        except Exception as exc:  # noqa: BLE001
            checks[name] = False
            errors[name] = f"{type(exc).__name__}: {exc}"
            return None

    manifest = check("manifest_readable", lambda: read_json(output / "manifest.json"))
    if not isinstance(manifest, dict):
        manifest = {}
    check("manifest_self_hash", lambda: manifest.get("manifest_content_sha256") == manifest_self_hash(manifest))
    check("protocol_identity", lambda: manifest.get("protocol_id") == PROTOCOL_ID and manifest.get("dataset") == DATASET_NAME)
    pins = check("source_bundle_pinned", lambda: validate_source_bundle(output, manifest))
    if not isinstance(pins, dict):
        pins = {}
    runtime = manifest.get("evaluation", {}).get("runtime_identity", {})
    check(
        "formal_runtime_identity",
        lambda: {
            "python_version": runtime.get("python_version"),
            "pybullet_version": runtime.get("pybullet_version"),
            "pybullet_api_version": runtime.get("pybullet_api_version"),
            "numpy_version": runtime.get("numpy_version"),
            "scipy_version": runtime.get("scipy_version"),
            "collision_harness_sha256": runtime.get("collision_harness_sha256"),
            "collision_core_sha256": runtime.get("collision_core_sha256"),
        }
        == {
            "python_version": FORMAL_PYTHON_VERSION,
            "pybullet_version": FORMAL_PYBULLET_VERSION,
            "pybullet_api_version": FORMAL_PYBULLET_API_VERSION,
            "numpy_version": FORMAL_NUMPY_VERSION,
            "scipy_version": FORMAL_SCIPY_VERSION,
            "collision_harness_sha256": FORMAL_COLLISION_HARNESS_SHA256,
            "collision_core_sha256": FORMAL_COLLISION_CORE_SHA256,
        },
    )
    check("exact_upstream_cohort", lambda: validate_upstream(manifest))
    check("all_live_packages_match", lambda: validate_packages(manifest))
    records = check("record_and_state_semantic_closure", lambda: record_closure(output, manifest, pins))
    if not isinstance(records, list):
        records = []
    summary = check("summary_readable", lambda: read_json(output / "summary.json"))
    if not isinstance(summary, dict):
        summary = {}
    recomputed = check(
        "summary_reaggregation_available",
        lambda: aggregate_summary(manifest, records, str(summary.get("completed_at"))) if records else (_ for _ in ()).throw(ValueError("records unavailable")),
    )
    check("summary_matches_reaggregation", lambda: summary == recomputed)
    check("report_matches_reaggregation", lambda: (output / "report.md").read_text(encoding="utf-8") == report_text(recomputed, manifest))
    checkpoint = check("checkpoint_readable", lambda: read_json(output / "checkpoint.json"))
    check("checkpoint_complete", lambda: checkpoint.get("state") == "complete" and int(checkpoint.get("completed", -1)) == int(manifest["sample_size"]) and int(checkpoint.get("remaining", -1)) == 0 and checkpoint.get("manifest_content_sha256") == manifest.get("manifest_content_sha256"))
    pair = check("pair_policy_receipt_readable", lambda: read_json(output / "pair_policy_smoke.json"))
    check("pair_policy_semantics", lambda: pair.get("status") == "PASS" and int(pair.get("all_pair_illegal_penetration_count", 0)) > 0 and int(pair.get("non_adjacent_illegal_penetration_count", -1)) == 0 and int(pair.get("pybullet_api_version", -1)) == FORMAL_PYBULLET_API_VERSION)
    check("artifact_manifest_closes", lambda: verify_artifacts(output))
    formal_evaluation = bool(manifest.get("classification") == "FORMAL" and int(manifest.get("sample_size", -1)) == FORMAL_N_EVAL and summary.get("j_eval") == FORMAL_J_EVAL)
    check("classification_consistent", lambda: manifest.get("classification") in {"FORMAL", "NON_FORMAL_SMOKE"})
    receipt = {
        "schema_version": "table4-sketch-mobility-verification/v2",
        "status": "PASS" if checks and all(checks.values()) else "FAIL",
        "formal_evaluation": formal_evaluation,
        "checks": checks,
        "errors": errors,
        "check_count": len(checks),
        "passed_check_count": sum(checks.values()),
        "manifest_content_sha256": manifest.get("manifest_content_sha256"),
    }
    if write_receipt:
        atomic_json(output / "verification.json", receipt)
    return receipt


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--write-receipt", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    receipt = verify_output(args.output_root, write_receipt=args.write_receipt)
    print(json.dumps(receipt, sort_keys=True))
    return 0 if receipt["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
