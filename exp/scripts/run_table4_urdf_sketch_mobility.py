#!/usr/bin/env python3
"""Run frozen URDF Sim-Ready Table 4 on SketchMobility's fixed N=800 cohort."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import tempfile
from typing import Any

import numpy
import scipy


SCRIPT_PATH = Path(__file__).resolve()
REPO = SCRIPT_PATH.parents[2]
TABLE3_ADAPTER_PATH = SCRIPT_PATH.with_name("run_table3_urdf_sketch_mobility.py")
COLLISION_HARNESS_PATH = SCRIPT_PATH.with_name("run_urdf_table4_ours_500k.py")
COLLISION_CORE_PATH = SCRIPT_PATH.with_name("run_urdf_table4_partnet_mobility.py")
VERIFIER_PATH = SCRIPT_PATH.with_name("verify_table4_urdf_sketch_mobility.py")
PROTOCOL_PATH = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
DEFAULT_DATASET_ROOT = REPO / "exp/SketchMobility"
DEFAULT_TABLE1_RECEIPT = REPO / "exp/runtime/table1_sketch_mobility_rerun_20260821T021838Z"
DEFAULT_TABLE2_RECEIPT = REPO / "exp/runtime/table2_urdf_sketch_mobility_table1cohort_n800_20260821T035015Z"
DEFAULT_TABLE2_MANIFEST = DEFAULT_TABLE2_RECEIPT / "manifest.json"
DEFAULT_TABLE3_RECEIPT = REPO / "exp/runtime/urdf_table3_sketch_mobility_table1cohort_n800_20260821T062050Z"
DEFAULT_SOURCE_PINS = REPO / "exp/reference/table4_sketch_mobility_source_pins_v1.json"

DATASET_NAME = "SketchMobility"
PROTOCOL_ID = "urdf-sim-ready-table4-sketch-mobility-table1-cohort-v1"
SOURCE_PIN_SET_ID = "table4-sketch-mobility-source-pins-v1"
FORMAL_N_RELEASE = 4956
FORMAL_N_EVAL = 800
FORMAL_J_EVAL = 1824
FORMAL_TABLE2_MANIFEST_SHA256 = "0be3e21f079bd86ba9ab680f1d709dd676b623bea01d8e43a3db85943a64a8e5"
FORMAL_TABLE2_RECORDS_SHA256 = "03b6d5e0d335052f123664a7a85dcdbc33ffbad8143ffb4bb62560e9b44ea2d1"
FORMAL_TABLE3_MANIFEST_SHA256 = "0f90fbdec03cf4be69dc2b870b2aa7eaa3c00de93e49c005394e402907276f4a"
FORMAL_TABLE3_MANIFEST_CONTENT_SHA256 = "3b151753d17c77e68e1ee7993bdf643867b0a1bef9142df3b17abca1fbda6ab8"
FORMAL_TABLE3_RECORDS_SHA256 = "13124125cbdef565efc95c7526e052576aead73fa6499d7b0b81bcc0490a24f7"
FORMAL_TABLE3_ADAPTER_SHA256 = "18522b8237e94a2419a4339d98681cc6d88459a17f6117dd9a8f2d4a5649d79d"
FORMAL_COLLISION_HARNESS_SHA256 = "e09bd4b7050bfb0e854f2462a27fdbc40ef3d59bb71040b7c73576a0fec3b84f"
FORMAL_COLLISION_CORE_SHA256 = "e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0"
FORMAL_PYTHON_VERSION = "3.12.3"
FORMAL_PYBULLET_VERSION = "3.2.7"
FORMAL_PYBULLET_API_VERSION = 202010061
FORMAL_NUMPY_VERSION = "2.5.1"
FORMAL_SCIPY_VERSION = "1.18.0"
SINGLE_SAMPLES = 21
SOBOL_SAMPLES = 64
SOBOL_SEED = 20260813
PENETRATION_THRESHOLD_M = 1e-6
RESET_TOLERANCE = 1e-9
DEFAULT_WORKERS = 4
DEFAULT_AUDIT_WORKERS = 4
DEFAULT_TIMEOUT_SECONDS = 900.0
PROTOCOL_SNAPSHOT_NAME = "protocol_snapshot.md"
SOURCE_SNAPSHOT_DIR = "source_snapshot"
RECORD_JOURNAL_DIR = ".record_journal"
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
EFFECTIVE_CHILD_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
    "PYTHONDONTWRITEBYTECODE": "1",
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


def _load_module(path: Path, name: str):
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import required module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


TABLE3 = _load_module(TABLE3_ADAPTER_PATH, "table4_sketch_table3_adapter")
COLLISION = _load_module(COLLISION_HARNESS_PATH, "table4_sketch_collision_harness")


def _configure_collision_module() -> None:
    COLLISION.SCRIPT = SCRIPT_PATH
    COLLISION.CORE_SCRIPT = COLLISION_CORE_PATH
    COLLISION.DATASET_ROOT = DEFAULT_DATASET_ROOT
    COLLISION.DATASET_LABEL = DATASET_NAME
    COLLISION.PROTOCOL_ID = PROTOCOL_ID
    COLLISION.EXPECTED_N_RELEASE = FORMAL_N_RELEASE
    COLLISION.SAMPLE_SIZE = FORMAL_N_EVAL
    COLLISION.IDENTITY_FIELDS = IDENTITY_FIELDS
    COLLISION.FROZEN_INPUT_FIELDS = FROZEN_INPUT_FIELDS


_configure_collision_module()


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


def _manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def load_frozen_cohort(
    table3_receipt: Path,
    table2_manifest: Path,
    table1_receipt: Path,
    dataset_root: Path,
    *,
    formal: bool,
) -> dict[str, Any]:
    """Cross-bind Table 3 membership to the exact Table 1/2 authority."""
    table3_receipt = table3_receipt.resolve(strict=True)
    table2_manifest = table2_manifest.resolve(strict=True)
    table1_receipt = table1_receipt.resolve(strict=True)
    dataset_root = dataset_root.resolve(strict=True)
    table3_manifest_path = table3_receipt / "manifest.json"
    table3_records_path = table3_receipt / "asset_records.jsonl"
    table2_records_path = table2_manifest.parent / "asset_records.jsonl"

    if formal:
        observed = {
            "table2_manifest": sha256_file(table2_manifest),
            "table2_records": sha256_file(table2_records_path),
            "table3_manifest": sha256_file(table3_manifest_path),
            "table3_records": sha256_file(table3_records_path),
        }
        expected = {
            "table2_manifest": FORMAL_TABLE2_MANIFEST_SHA256,
            "table2_records": FORMAL_TABLE2_RECORDS_SHA256,
            "table3_manifest": FORMAL_TABLE3_MANIFEST_SHA256,
            "table3_records": FORMAL_TABLE3_RECORDS_SHA256,
        }
        if observed != expected:
            raise ValueError(f"formal upstream receipt hash mismatch: {observed}")

    table3_manifest = json.loads(table3_manifest_path.read_text(encoding="utf-8"))
    if table3_manifest.get("manifest_content_sha256") != _manifest_self_hash(table3_manifest):
        raise ValueError("Table 3 manifest self-hash mismatch")
    if formal and table3_manifest.get("manifest_content_sha256") != FORMAL_TABLE3_MANIFEST_CONTENT_SHA256:
        raise ValueError("Table 3 manifest content hash mismatch")
    if table3_manifest.get("dataset") != DATASET_NAME or table3_manifest.get("classification") != "FORMAL":
        raise ValueError("Table 3 receipt is not the formal SketchMobility run")

    upstream = TABLE3.load_frozen_cohort(
        table2_manifest, table1_receipt, dataset_root, formal=False
    )
    table2_records = read_jsonl(table2_records_path)
    table3_records = read_jsonl(table3_records_path)
    frozen_rows = table3_manifest.get("records")
    if not isinstance(frozen_rows, list):
        raise ValueError("Table 3 manifest records are missing")
    if not (
        len(upstream["rows"])
        == len(table2_records)
        == len(frozen_rows)
        == len(table3_records)
        == FORMAL_N_EVAL
    ):
        raise ValueError("upstream cohort record count mismatch")

    rows: list[dict[str, Any]] = []
    declared_joint_count = 0
    for rank, (table2_row, table2_record, frozen, record) in enumerate(
        zip(
            upstream["rows"],
            table2_records,
            frozen_rows,
            table3_records,
            strict=True,
        ),
        start=1,
    ):
        expected = {
            "asset_id": table2_row.get("asset_id"),
            "selection_rank": rank,
            "selection_index": rank - 1,
            "selection_hash": table2_row.get("selection_hash"),
            "category": table2_row.get("source_category"),
            "package": table2_row.get("package"),
            "urdf_sha256": table2_row.get("primary_urdf_sha256"),
            "package_content_manifest_sha256": table2_record.get(
                "package_content_manifest_sha256"
            ),
        }
        observed_frozen = {key: frozen.get(key) for key in expected}
        observed_record = {key: record.get(key) for key in expected}
        if observed_frozen != expected or observed_record != expected:
            raise ValueError(f"Table 3 record binding mismatch at rank {rank}")
        if record.get("manifest_content_sha256") != table3_manifest.get(
            "manifest_content_sha256"
        ):
            raise ValueError(f"Table 3 record manifest binding mismatch at rank {rank}")
        if record.get("status") != "completed":
            raise ValueError(f"Table 3 record is not completed at rank {rank}")
        if frozen.get("package_binding", {}).get("content_manifest_sha256") != expected[
            "package_content_manifest_sha256"
        ]:
            raise ValueError(f"Table 3 package binding mismatch at rank {rank}")
        declared = int(record.get("declared_joint_count", -1))
        if declared < 0 or declared != int(frozen.get("declared_joint_count_hint", -2)):
            raise ValueError(f"Table 3 declared joint count mismatch at rank {rank}")
        declared_joint_count += declared
        rows.append(
            {
                **expected,
                "source": table2_row.get("source"),
                "raw_category": table2_row.get("source_category"),
                "package_binding": frozen["package_binding"],
                "declared_joint_count": declared,
                "table3_strict_kinematic_pass": bool(
                    record.get("strict_kinematic_pass")
                ),
                "table2_collision_coverage_pass": bool(
                    table2_record.get("metrics", {})
                    .get("collision_coverage", {})
                    .get("pass")
                ),
            }
        )
    if formal and declared_joint_count != FORMAL_J_EVAL:
        raise ValueError("formal declared joint denominator mismatch")
    return {
        "rows": rows,
        "declared_joint_count": declared_joint_count,
        "n_release": FORMAL_N_RELEASE,
        "table3_manifest": table3_manifest,
        "hashes": {
            "table2_manifest": sha256_file(table2_manifest),
            "table2_records": sha256_file(table2_records_path),
            "table3_manifest": sha256_file(table3_manifest_path),
            "table3_records": sha256_file(table3_records_path),
        },
    }


def audit_asset(row: dict[str, Any], dataset_root: Path) -> dict[str, Any]:
    dataset_root = dataset_root.resolve(strict=True)
    package = TABLE3._safe_package(dataset_root, str(row["asset_id"]))
    package_binding = TABLE3._package_binding(package)
    if package_binding != row["package_binding"]:
        raise ValueError(f"full package binding drift: {row['asset_id']}")
    urdf_path = package / "mobility.urdf"
    if sha256_file(urdf_path) != row["urdf_sha256"]:
        raise ValueError(f"primary URDF binding drift: {row['asset_id']}")

    joints = COLLISION._load_core().parse_urdf_joints(urdf_path)
    inventory, unsafe = COLLISION.collision_mesh_inventory(package, urdf_path)
    missing = sum(not item["exists"] for item in inventory)
    valid_tree = COLLISION._valid_tree(urdf_path)
    result: dict[str, Any] = {
        **row,
        "asset_root_relpath": str(row["asset_id"]),
        "primary_urdf_relative_path": "mobility.urdf",
        "primary_urdf_relpath": f"{row['asset_id']}/mobility.urdf",
        "urdf_sha256": sha256_file(urdf_path),
        "package_content_manifest_sha256": package_binding[
            "content_manifest_sha256"
        ],
        "movable_dof_count": len(joints),
        "range_evaluable_dof_count": sum(bool(item["range_evaluable"]) for item in joints),
        "joint_specs": joints,
        "joint_specs_sha256": canonical_sha256(joints),
        "collision_mesh_files": inventory,
        "collision_mesh_inventory_sha256": canonical_sha256(inventory),
        "missing_collision_mesh_reference_count": missing,
        "unsafe_collision_mesh_reference_count": unsafe,
        "valid_tree": valid_tree,
        "package_audit_success": False,
        "audit_issue": None,
        "scale_derivation": {"protocol": COLLISION.SCALE_PROTOCOL, "status": "N/E"},
        "object_bbox_diagonal_m": None,
    }
    issues: list[str] = []
    if unsafe:
        issues.append(f"unsafe_collision_mesh_reference_count:{unsafe}")
    if missing:
        issues.append(f"missing_collision_mesh_reference_count:{missing}")
    if not valid_tree:
        issues.append("invalid_rooted_tree")
    if not row["table2_collision_coverage_pass"]:
        issues.append("table2_collision_coverage_incomplete")
    if len(joints) != int(row["declared_joint_count"]):
        issues.append("declared_joint_count_drift")
    if not issues:
        try:
            scale = COLLISION.derive_collision_aabb(urdf_path)
            result["scale_derivation"] = scale
            result["object_bbox_diagonal_m"] = scale["diagonal_m"]
            result["package_audit_success"] = True
        except Exception as exc:  # noqa: BLE001
            issues.append(f"{type(exc).__name__}: {exc}")
    result["audit_issue"] = ";".join(issues) if issues else None
    result["scale_derivation_sha256"] = canonical_sha256(result["scale_derivation"])
    return result


def current_runtime_identity() -> dict[str, Any]:
    import pybullet

    return {
        "python_executable": sys.executable,
        "python_version": platform.python_version(),
        "pybullet_version": importlib.metadata.version("pybullet"),
        "pybullet_api_version": int(pybullet.getAPIVersion()),
        "pybullet_module_sha256": sha256_file(Path(pybullet.__file__).resolve(strict=True)),
        "numpy_version": numpy.__version__,
        "scipy_version": scipy.__version__,
        "collision_harness_sha256": sha256_file(COLLISION_HARNESS_PATH),
        "collision_core_sha256": sha256_file(COLLISION_CORE_PATH),
    }


def validate_formal_runtime(identity: dict[str, Any]) -> None:
    expected = {
        "python_version": FORMAL_PYTHON_VERSION,
        "pybullet_version": FORMAL_PYBULLET_VERSION,
        "pybullet_api_version": FORMAL_PYBULLET_API_VERSION,
        "numpy_version": FORMAL_NUMPY_VERSION,
        "scipy_version": FORMAL_SCIPY_VERSION,
        "collision_harness_sha256": FORMAL_COLLISION_HARNESS_SHA256,
        "collision_core_sha256": FORMAL_COLLISION_CORE_SHA256,
    }
    observed = {key: identity.get(key) for key in expected}
    if observed != expected:
        raise ValueError(f"formal runtime mismatch: {observed} != {expected}")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def atomic_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n")
    temporary.replace(path)


def atomic_text(path: Path, value: str) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def freeze_protocol_snapshot(output: Path) -> dict[str, str]:
    target = output / PROTOCOL_SNAPSHOT_NAME
    target.write_bytes(PROTOCOL_PATH.read_bytes())
    return {
        "path": str(target),
        "sha256": sha256_file(target),
        "source_path": str(PROTOCOL_PATH),
        "source_sha256_at_freeze": sha256_file(PROTOCOL_PATH),
    }


def _executable_source_paths() -> dict[str, Path]:
    return {
        "adapter": SCRIPT_PATH,
        "independent_verifier": VERIFIER_PATH,
        "collision_harness": COLLISION_HARNESS_PATH,
        "collision_core": COLLISION_CORE_PATH,
        "table3_adapter": TABLE3_ADAPTER_PATH,
        "table3_child_harness": TABLE3_ADAPTER_PATH.with_name(
            "run_urdf_table3_ours_500k.py"
        ),
        "table3_core": TABLE3_ADAPTER_PATH.with_name("run_urdf_table3_lam.py"),
        "table3_verifier": TABLE3_ADAPTER_PATH.with_name(
            "verify_table3_urdf_sketch_mobility.py"
        ),
    }


def _source_paths() -> dict[str, Path]:
    return {**_executable_source_paths(), "source_pins": DEFAULT_SOURCE_PINS}


def validate_source_pins(
    pin_path: Path, *, source_paths: dict[str, Path] | None = None
) -> dict[str, Any]:
    pin_path = pin_path.resolve(strict=True)
    value = json.loads(pin_path.read_text(encoding="utf-8"))
    if (
        not isinstance(value, dict)
        or value.get("schema_version") != 1
        or value.get("pin_set_id") != SOURCE_PIN_SET_ID
    ):
        raise ValueError("source pin set identity mismatch")
    declared = value.get("sources")
    paths = source_paths or _executable_source_paths()
    if not isinstance(declared, dict) or set(declared) != set(paths):
        raise ValueError("source pin roles mismatch")
    observed = {role: sha256_file(path.resolve(strict=True)) for role, path in paths.items()}
    if observed != declared:
        raise ValueError(f"source pin mismatch: {observed} != {declared}")
    return value


def freeze_source_snapshots(output: Path) -> dict[str, dict[str, str]]:
    snapshot_root = output / SOURCE_SNAPSHOT_DIR
    snapshot_root.mkdir(parents=True, exist_ok=False)
    result: dict[str, dict[str, str]] = {}
    for role, source in _source_paths().items():
        target = snapshot_root / source.name
        target.write_bytes(source.read_bytes())
        result[role] = {
            "path": str(target),
            "sha256": sha256_file(target),
            "source_path": str(source),
        }
    return result


def validate_source_snapshots(
    output: Path,
    declared: dict[str, dict[str, str]],
    *,
    require_live_match: bool,
) -> None:
    if set(declared) != set(SOURCE_FILENAMES):
        raise RuntimeError("source snapshot roles mismatch")
    snapshot_paths: dict[str, Path] = {}
    for role, filename in SOURCE_FILENAMES.items():
        entry = declared[role]
        target = output / SOURCE_SNAPSHOT_DIR / filename
        if Path(str(entry.get("path"))).resolve(strict=True) != target.resolve(strict=True):
            raise RuntimeError(f"source snapshot path mismatch: {role}")
        digest = sha256_file(target)
        if digest != entry.get("sha256"):
            raise RuntimeError(f"source snapshot hash mismatch: {role}")
        snapshot_paths[role] = target
    validate_source_pins(
        snapshot_paths["source_pins"],
        source_paths={
            role: path for role, path in snapshot_paths.items() if role != "source_pins"
        },
    )
    if require_live_match:
        validate_source_pins(DEFAULT_SOURCE_PINS)
        live = _source_paths()
        for role, path in live.items():
            if sha256_file(path.resolve(strict=True)) != declared[role]["sha256"]:
                raise RuntimeError(f"live source differs from snapshot: {role}")


def build_items(
    rows: list[dict[str, Any]],
    audits: dict[str, dict[str, Any]],
    runtime_identity: dict[str, Any],
    *,
    formal: bool,
) -> list[dict[str, Any]]:
    child_environment_sha256 = canonical_sha256(EFFECTIVE_CHILD_ENVIRONMENT)
    items: list[dict[str, Any]] = []
    for order, row in enumerate(rows):
        audit = audits[str(row["asset_id"])]
        movable = int(audit["movable_dof_count"])
        item = {
            "protocol_id": PROTOCOL_ID,
            "order": order,
            "dataset_id": str(row["asset_id"]),
            "asset_id": str(row["asset_id"]),
            "raw_category": str(row["raw_category"]),
            "source": str(row["source"]),
            "selection_hash": str(row["selection_hash"]),
            "selection_rank": int(row["selection_rank"]),
            "category": str(row["category"]),
            "asset_root_relpath": str(audit["asset_root_relpath"]),
            "primary_urdf_relpath": str(audit["primary_urdf_relpath"]),
            "urdf_sha256": str(audit["urdf_sha256"]),
            "package_content_manifest_sha256": str(
                audit["package_content_manifest_sha256"]
            ),
            "package_audit_success": bool(audit["package_audit_success"]),
            "audit_issue": audit["audit_issue"],
            "valid_tree": bool(audit["valid_tree"]),
            "movable_dof_count": movable,
            "range_evaluable_dof_count": int(audit["range_evaluable_dof_count"]),
            "joint_specs_sha256": audit["joint_specs_sha256"],
            "joint_specs": audit["joint_specs"],
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
            "formal": formal,
            "effective_child_environment_sha256": child_environment_sha256,
        }
        item["input_identity_sha256"] = canonical_sha256(
            {key: item.get(key) for key in FROZEN_INPUT_FIELDS}
        )
        items.append(item)
    return items


def validate_live_package(item: dict[str, Any], dataset_root: Path) -> None:
    package = TABLE3._safe_package(dataset_root.resolve(strict=True), item["asset_id"])
    binding = TABLE3._package_binding(package)
    if binding["content_manifest_sha256"] != item["package_content_manifest_sha256"]:
        raise ValueError("full package changed after freeze")
    urdf_path = dataset_root.resolve(strict=True) / item["primary_urdf_relpath"]
    if sha256_file(urdf_path) != item["urdf_sha256"]:
        raise ValueError("primary URDF changed after freeze")
    inventory, unsafe = COLLISION.collision_mesh_inventory(package, urdf_path)
    if unsafe or canonical_sha256(inventory) != item["collision_mesh_inventory_sha256"]:
        raise ValueError("collision mesh inventory changed after freeze")


def _decorate_result(item: dict[str, Any], result: dict[str, Any]) -> dict[str, Any]:
    result.update(
        {
            "asset_id": item["asset_id"],
            "raw_category": item["raw_category"],
            "source": item["source"],
            "selection_hash": item["selection_hash"],
            "selection_rank": item["selection_rank"],
            "package_content_manifest_sha256": item[
                "package_content_manifest_sha256"
            ],
            "effective_child_environment_sha256": item[
                "effective_child_environment_sha256"
            ],
        }
    )
    return result


def evaluate_asset(item: dict[str, Any], dataset_root: Path) -> dict[str, Any]:
    _configure_collision_module()
    try:
        validate_live_package(item, dataset_root)
    except Exception as exc:  # noqa: BLE001
        result = COLLISION._load_core().failure_record(
            item, f"source_binding_failure: {type(exc).__name__}: {exc}"
        )
        result["runner_sha256"] = sha256_file(SCRIPT_PATH)
        result["collision_core_sha256"] = sha256_file(COLLISION_CORE_PATH)
        result["runtime_identity"] = COLLISION.current_runtime_identity()
        return _decorate_result(item, result)
    result = COLLISION.evaluate_asset(item, dataset_root)
    return _decorate_result(item, result)


def run_child(item_path: Path, dataset_root: Path, result_path: Path) -> int:
    item = json.loads(item_path.resolve(strict=True).read_text(encoding="utf-8"))
    binding = item.get("runtime_binding", {})
    if binding.get("python_executable") != sys.executable:
        print("FATAL_RUNTIME_BINDING: python executable differs", file=sys.stderr)
        return 86
    identity = current_runtime_identity()
    if item.get("formal"):
        validate_formal_runtime(identity)
    result = evaluate_asset(item, dataset_root)
    atomic_json(result_path, result)
    return 0


def verify_result_against_item(item: dict[str, Any], result: dict[str, Any]) -> None:
    core = COLLISION._load_core()
    states = result.get("state_records")
    if not isinstance(states, list) or not core._result_counters_valid(
        result, states, item
    ):
        raise ValueError("child state sequence or result derivation mismatch")
    expected = {
        "asset_id": item["asset_id"],
        "selection_hash": item["selection_hash"],
        "selection_rank": item["selection_rank"],
        "package_content_manifest_sha256": item[
            "package_content_manifest_sha256"
        ],
        "effective_child_environment_sha256": item[
            "effective_child_environment_sha256"
        ],
        "runner_sha256": sha256_file(SCRIPT_PATH),
        "collision_core_sha256": sha256_file(COLLISION_CORE_PATH),
    }
    observed = {key: result.get(key) for key in expected}
    if observed != expected:
        raise ValueError(f"child result provenance mismatch: {observed}")


def _run_one_subprocess(
    item: dict[str, Any],
    scratch: Path,
    timeout_seconds: float,
    frozen_adapter: Path,
    dataset_root: Path,
) -> dict[str, Any]:
    job_root = Path(tempfile.mkdtemp(prefix="job_", dir=scratch))
    item_path = job_root / "item.json"
    result_path = job_root / "result.json"
    atomic_json(item_path, item)
    environment = dict(os.environ)
    environment.update(EFFECTIVE_CHILD_ENVIRONMENT)
    process = subprocess.Popen(
        [
            sys.executable,
            str(frozen_adapter),
            "--internal-child",
            str(item_path),
            str(dataset_root),
            str(result_path),
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        start_new_session=True,
        env=environment,
    )
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
            result = COLLISION._load_core().failure_record(
                item, f"child_timeout_after_{timeout_seconds:g}s", timed_out=True
            )
            result["runner_sha256"] = sha256_file(SCRIPT_PATH)
            result["collision_core_sha256"] = sha256_file(COLLISION_CORE_PATH)
            result["runtime_identity"] = COLLISION.current_runtime_identity()
            return _decorate_result(item, result)
        if process.returncode != 0 or not result_path.is_file():
            detail = (stderr or b"").decode("utf-8", errors="replace")[-4000:]
            result = COLLISION._load_core().failure_record(
                item, f"child_exit_{process.returncode}: {detail}"
            )
            result["runner_sha256"] = sha256_file(SCRIPT_PATH)
            result["collision_core_sha256"] = sha256_file(COLLISION_CORE_PATH)
            result["runtime_identity"] = COLLISION.current_runtime_identity()
            return _decorate_result(item, result)
        return json.loads(result_path.read_text(encoding="utf-8"))
    finally:
        shutil.rmtree(job_root, ignore_errors=True)


def _journal_path(journal: Path, item: dict[str, Any]) -> Path:
    return journal / f"{int(item['selection_rank']):04d}.json"


def load_journal_records(
    journal: Path, items: list[dict[str, Any]]
) -> dict[str, dict[str, Any]]:
    by_rank = {int(item["selection_rank"]): item for item in items}
    records: dict[str, dict[str, Any]] = {}
    for path in sorted(journal.glob("*.json")):
        try:
            rank = int(path.stem)
        except ValueError as exc:
            raise RuntimeError(f"invalid journal filename: {path.name}") from exc
        item = by_rank.get(rank)
        if item is None:
            raise RuntimeError(f"journal rank outside frozen cohort: {rank}")
        record = json.loads(path.read_text(encoding="utf-8"))
        verify_result_against_item(item, record)
        records[item["asset_id"]] = record
    return records


def _execute_jobs(
    items: list[dict[str, Any]],
    journal: Path,
    scratch: Path,
    *,
    workers: int,
    timeout_seconds: float,
    frozen_adapter: Path,
    dataset_root: Path,
    checkpoint: Path,
    manifest_hash: str,
    stop_after: int | None = None,
) -> list[dict[str, Any]]:
    records = load_journal_records(journal, items)
    pending = [item for item in items if item["asset_id"] not in records]
    newly_completed = 0
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(
                _run_one_subprocess,
                item,
                scratch,
                timeout_seconds,
                frozen_adapter,
                dataset_root,
            ): item
            for item in pending
        }
        for future in as_completed(futures):
            item = futures[future]
            try:
                record = future.result()
            except Exception as exc:  # noqa: BLE001
                record = COLLISION._load_core().failure_record(
                    item, f"parent_worker_exception: {type(exc).__name__}: {exc}"
                )
                record["runner_sha256"] = sha256_file(SCRIPT_PATH)
                record["collision_core_sha256"] = sha256_file(COLLISION_CORE_PATH)
                record["runtime_identity"] = COLLISION.current_runtime_identity()
                record = _decorate_result(item, record)
            verify_result_against_item(item, record)
            atomic_json(_journal_path(journal, item), record)
            records[item["asset_id"]] = record
            newly_completed += 1
            completed = len(records)
            atomic_json(
                checkpoint,
                {
                    "state": "running" if completed < len(items) else "aggregating",
                    "completed": completed,
                    "remaining": len(items) - completed,
                    "manifest_content_sha256": manifest_hash,
                    "updated_at": utc_now(),
                },
            )
            print(f"table4 {completed}/{len(items)} rank={item['selection_rank']}", flush=True)
            if stop_after is not None and newly_completed >= stop_after:
                for other in futures:
                    other.cancel()
                raise KeyboardInterrupt("test interruption")
    return [records[item["asset_id"]] for item in items]


def aggregate_summary(
    manifest: dict[str, Any],
    records: list[dict[str, Any]],
    *,
    completed_at: str | None = None,
) -> dict[str, Any]:
    core = COLLISION._load_core()
    summary = core.summarize_records(
        {
            "sample_size": len(records),
            "protocol_id": PROTOCOL_ID,
            "cohort_label": manifest["cohort_label"],
        },
        records,
    )
    summary["dataset"] = DATASET_NAME
    summary["n_release"] = FORMAL_N_RELEASE
    summary["n_eval"] = len(records)
    summary["j_eval"] = sum(int(item["movable_dof_count"]) for item in manifest["items"])
    summary["manifest_content_sha256"] = manifest["manifest_content_sha256"]
    summary["metrics"]["max_penetration"]["normalization"] = (
        "PyBullet q0 collision-shape union AABB diagonal "
        f"({COLLISION.SCALE_PROTOCOL})"
    )
    summary["completed_at"] = completed_at or utc_now()
    return summary


def report_text(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    metrics = summary["metrics"]

    def fraction(metric: dict[str, Any], numerator: str = "passed") -> str:
        if metric.get("rate") is None:
            return "N/E"
        return (
            f"{metric[numerator]} / {metric['denominator']} "
            f"({100.0 * metric['rate']:.3f}%)"
        )

    maximum = metrics["max_penetration"]
    max_text = "N/E"
    if maximum["maximum_observed_normalized"] is not None:
        max_text = (
            f"{maximum['maximum_observed_normalized']:.6f} "
            f"({maximum['fully_measured_assets']} / {maximum['denominator']} measured; "
            f"{maximum['status']})"
        )
    return "\n".join(
        [
            "# SketchMobility Table 4: Collision and Mechanical Clearance",
            "",
            f"Run classification: **{manifest['classification']}**.",
            "",
            f"Exact frozen cohort: N_eval={summary['n_eval']}, J_eval={summary['j_eval']}; "
            f"rest q=0; single-joint K={SINGLE_SAMPLES}; Sobol R={SOBOL_SAMPLES} "
            f"(seed {SOBOL_SEED}); threshold {PENETRATION_THRESHOLD_M} m.",
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
            "Unexecuted states remain in the denominator and count as non-free. "
            "AOR is N/E; no bounding-box overlap substitute was used.",
            "",
        ]
    )


def _write_artifact_manifest(output: Path) -> None:
    declared = []
    for path in sorted(output.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"receipt contains symlink: {path.relative_to(output)}")
        if not path.is_file() or path.name == "artifact_manifest.json":
            continue
        relative = path.relative_to(output).as_posix()
        if relative.startswith(RECORD_JOURNAL_DIR + "/") or relative.startswith(
            ".worker_scratch/"
        ):
            continue
        declared.append(
            {
                "path": relative,
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    atomic_json(
        output / "artifact_manifest.json",
        {"schema_version": 1, "artifacts": declared},
    )


def verify_artifacts(output: Path) -> None:
    manifest_path = output / "artifact_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    declared = manifest.get("artifacts")
    if not isinstance(declared, list):
        raise ValueError("artifact manifest is malformed")
    expected_paths = {"artifact_manifest.json"}
    for entry in declared:
        relative = str(entry["path"])
        path = output / relative
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"declared artifact missing: {relative}")
        if path.stat().st_size != int(entry["bytes"]) or sha256_file(path) != entry[
            "sha256"
        ]:
            raise ValueError(f"declared artifact binding mismatch: {relative}")
        expected_paths.add(relative)
    symlinks = [
        path.relative_to(output).as_posix()
        for path in output.rglob("*")
        if path.is_symlink()
    ]
    if symlinks:
        raise ValueError(f"receipt contains symlink: {sorted(symlinks)}")
    observed_paths = {
        path.relative_to(output).as_posix()
        for path in output.rglob("*")
        if path.is_file()
    }
    if observed_paths != expected_paths:
        raise ValueError(
            "unexpected receipt files: "
            f"unexpected={sorted(observed_paths - expected_paths)}, "
            f"missing={sorted(expected_paths - observed_paths)}"
        )


def _run_independent_verifier(output: Path, manifest: dict[str, Any]) -> dict[str, Any]:
    verifier = Path(
        manifest["evaluation"]["source_snapshots"]["independent_verifier"]["path"]
    )
    environment = dict(os.environ)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    process = subprocess.run(
        [
            sys.executable,
            str(verifier),
            "--output-root",
            str(output),
            "--write-receipt",
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        env=environment,
    )
    if process.returncode != 0:
        stdout = process.stdout.decode("utf-8", errors="replace")[-4000:]
        stderr = process.stderr.decode("utf-8", errors="replace")[-4000:]
        raise RuntimeError(
            f"independent verifier failed: stdout={stdout!r} stderr={stderr!r}"
        )
    receipt = json.loads((output / "verification.json").read_text(encoding="utf-8"))
    if receipt.get("status") != "PASS":
        raise RuntimeError("independent verifier did not pass")
    return receipt


def _build_manifest(
    args: argparse.Namespace,
    loaded: dict[str, Any],
    items: list[dict[str, Any]],
    protocol: dict[str, str],
    sources: dict[str, dict[str, str]],
    runtime_identity: dict[str, Any],
) -> dict[str, Any]:
    manifest = {
        "schema_version": "table4-sketch-mobility/v1",
        "created_at": utc_now(),
        "dataset": DATASET_NAME,
        "classification": "FORMAL" if args.mode == "formal" else "NON_FORMAL_SMOKE",
        "protocol_id": PROTOCOL_ID,
        "cohort_label": "SketchMobility exact frozen Table 1/2/3 N=800 cohort",
        "sample_size": len(items),
        "source": {
            "dataset_root": str(args.dataset_root.resolve(strict=True)),
            "table1_receipt": str(args.table1_receipt.resolve(strict=True)),
            "table2_manifest": str(args.table2_manifest.resolve(strict=True)),
            "table3_receipt": str(args.table3_receipt.resolve(strict=True)),
            "table2_manifest_sha256": loaded["hashes"]["table2_manifest"],
            "table2_records_sha256": loaded["hashes"]["table2_records"],
            "table3_manifest_sha256": loaded["hashes"]["table3_manifest"],
            "table3_records_sha256": loaded["hashes"]["table3_records"],
            "n_release": FORMAL_N_RELEASE,
            "declared_joint_count": loaded["declared_joint_count"],
        },
        "evaluation": {
            "protocol": protocol,
            "source_snapshots": sources,
            "adapter_sha256": sources["adapter"]["sha256"],
            "collision_harness_sha256": sources["collision_harness"]["sha256"],
            "collision_core_sha256": sources["collision_core"]["sha256"],
            "independent_verifier_sha256": sources["independent_verifier"]["sha256"],
            "source_pin_set_id": SOURCE_PIN_SET_ID,
            "source_pin_set_sha256": sources["source_pins"]["sha256"],
            "runtime_identity": runtime_identity,
            "effective_child_environment": EFFECTIVE_CHILD_ENVIRONMENT,
            "effective_child_environment_sha256": canonical_sha256(
                EFFECTIVE_CHILD_ENVIRONMENT
            ),
            "workers": args.workers,
            "audit_workers": args.audit_workers,
            "child_timeout_seconds": args.child_timeout_seconds,
            "rest_state_policy": "q=0 for every evaluable joint",
            "single_samples": SINGLE_SAMPLES,
            "sobol_samples": SOBOL_SAMPLES,
            "sobol_seed": SOBOL_SEED,
            "penetration_threshold_m": PENETRATION_THRESHOLD_M,
            "pair_policy": "all-pair and non-adjacent; direct parent-child excluded for headline; no allowlist",
            "geometry_policy": "collision geometry only; no visual fallback",
        },
        "selection": {
            "algorithm": "exact Table 3 manifest and record rank order; smoke uses prefix only",
            "selected_asset_ids_sha256": canonical_sha256(
                [item["asset_id"] for item in items]
            ),
            "outcome_based_reselection": False,
        },
        "items": items,
    }
    if args.mode == "formal":
        manifest["source"]["smoke_receipt"] = smoke_receipt_binding(
            args.smoke_receipt
        )
    manifest["manifest_content_sha256"] = _manifest_self_hash(manifest)
    return manifest


def _validate_resume(
    args: argparse.Namespace, output: Path, manifest: dict[str, Any]
) -> None:
    if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
        raise RuntimeError("resume manifest self-hash mismatch")
    expected_class = "FORMAL" if args.mode == "formal" else "NON_FORMAL_SMOKE"
    if manifest.get("classification") != expected_class:
        raise RuntimeError("resume mode mismatch")
    expected_count = FORMAL_N_EVAL if args.limit is None else args.limit
    if manifest.get("sample_size") != expected_count:
        raise RuntimeError("resume sample size mismatch")
    validate_source_snapshots(
        output,
        manifest["evaluation"]["source_snapshots"],
        require_live_match=False,
    )


def _run_to_output(args: argparse.Namespace, output: Path, *, resume: bool) -> dict[str, Any]:
    manifest_path = output / "manifest.json"
    journal = output / RECORD_JOURNAL_DIR
    scratch = output / ".worker_scratch"
    checkpoint = output / "checkpoint.json"
    if resume:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        _validate_resume(args, output, manifest)
    else:
        protocol = freeze_protocol_snapshot(output)
        sources = freeze_source_snapshots(output)
        runtime_identity = current_runtime_identity()
        if args.mode == "formal":
            validate_formal_runtime(runtime_identity)
        loaded = load_frozen_cohort(
            args.table3_receipt,
            args.table2_manifest,
            args.table1_receipt,
            args.dataset_root,
            formal=True,
        )
        rows = loaded["rows"] if args.limit is None else loaded["rows"][: args.limit]
        audits: dict[str, dict[str, Any]] = {}
        with ThreadPoolExecutor(max_workers=args.audit_workers) as executor:
            futures = {
                executor.submit(audit_asset, row, args.dataset_root): row for row in rows
            }
            for future in as_completed(futures):
                row = futures[future]
                audits[str(row["asset_id"])] = future.result()
        items = build_items(
            rows,
            audits,
            runtime_identity,
            formal=args.mode == "formal",
        )
        manifest = _build_manifest(
            args, loaded, items, protocol, sources, runtime_identity
        )
        atomic_json(manifest_path, manifest)
        journal.mkdir(exist_ok=False)
        atomic_json(
            checkpoint,
            {
                "state": "frozen",
                "completed": 0,
                "remaining": len(items),
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "updated_at": utc_now(),
            },
        )
        COLLISION._load_core().run_pair_policy_smoke(output)

    scratch.mkdir(exist_ok=True)
    items = manifest["items"]
    frozen_adapter = Path(
        manifest["evaluation"]["source_snapshots"]["adapter"]["path"]
    )
    records = _execute_jobs(
        items,
        journal,
        scratch,
        workers=args.workers,
        timeout_seconds=args.child_timeout_seconds,
        frozen_adapter=frozen_adapter,
        dataset_root=args.dataset_root.resolve(strict=True),
        checkpoint=checkpoint,
        manifest_hash=manifest["manifest_content_sha256"],
    )
    summary = aggregate_summary(manifest, records)
    asset_records: list[dict[str, Any]] = []
    state_records: list[dict[str, Any]] = []
    for record in records:
        copy = dict(record)
        state_records.extend(copy.pop("state_records", []))
        asset_records.append(copy)
    atomic_jsonl(output / "asset_records.jsonl", asset_records)
    atomic_jsonl(output / "state_records.jsonl", state_records)
    atomic_json(output / "summary.json", summary)
    atomic_text(output / "report.md", report_text(summary, manifest))
    atomic_json(
        checkpoint,
        {
            "state": "complete",
            "completed": len(items),
            "remaining": 0,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "updated_at": utc_now(),
        },
    )
    shutil.rmtree(scratch, ignore_errors=True)
    shutil.rmtree(journal, ignore_errors=True)
    verification = _run_independent_verifier(output, manifest)
    _write_artifact_manifest(output)
    verify_artifacts(output)
    return {"summary": summary, "verification": verification, "manifest": manifest}


def run(args: argparse.Namespace) -> dict[str, Any]:
    validate_contract(args)
    output = args.output.resolve(strict=False)
    with TABLE3._output_lock(output):
        if args.resume:
            version, resume_index = TABLE3._load_resume_version(output)
        else:
            version, resume_index = TABLE3._allocate_fresh_version(output)
        result = _run_to_output(args, version, resume=args.resume)
        TABLE3._publish_final_version(version, output, resume_index)
    return result


def smoke_receipt_binding(path: Path | None) -> dict[str, Any]:
    if path is None:
        raise ValueError("formal mode requires a passing smoke receipt")
    receipt = path.resolve(strict=True)
    verify_artifacts(receipt)
    manifest_path = receipt / "manifest.json"
    verification_path = receipt / "verification.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    verification = json.loads(verification_path.read_text(encoding="utf-8"))
    expected_config = {
        "protocol_id": PROTOCOL_ID,
        "classification": "NON_FORMAL_SMOKE",
        "sample_size": 5,
        "single_samples": SINGLE_SAMPLES,
        "sobol_samples": SOBOL_SAMPLES,
        "sobol_seed": SOBOL_SEED,
        "penetration_threshold_m": PENETRATION_THRESHOLD_M,
        "child_timeout_seconds": DEFAULT_TIMEOUT_SECONDS,
        "source_pin_set_sha256": sha256_file(DEFAULT_SOURCE_PINS),
    }
    observed = {
        "protocol_id": manifest.get("protocol_id"),
        "classification": manifest.get("classification"),
        "sample_size": manifest.get("sample_size"),
        "single_samples": manifest.get("evaluation", {}).get("single_samples"),
        "sobol_samples": manifest.get("evaluation", {}).get("sobol_samples"),
        "sobol_seed": manifest.get("evaluation", {}).get("sobol_seed"),
        "penetration_threshold_m": manifest.get("evaluation", {}).get(
            "penetration_threshold_m"
        ),
        "child_timeout_seconds": manifest.get("evaluation", {}).get(
            "child_timeout_seconds"
        ),
        "source_pin_set_sha256": manifest.get("evaluation", {}).get(
            "source_pin_set_sha256"
        ),
    }
    if observed != expected_config:
        raise ValueError(f"smoke receipt configuration mismatch: {observed}")
    frozen_verifier = Path(
        manifest["evaluation"]["source_snapshots"]["independent_verifier"][
            "path"
        ]
    )
    environment = dict(os.environ)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    replay = subprocess.run(
        [
            sys.executable,
            str(frozen_verifier),
            "--output-root",
            str(receipt),
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        env=environment,
    )
    try:
        replay_receipt = json.loads(replay.stdout.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("smoke receipt semantic verification produced no receipt") from exc
    if (
        replay.returncode != 0
        or replay_receipt.get("status") != "PASS"
        or not all(replay_receipt.get("checks", {}).values())
    ):
        detail = replay.stderr.decode("utf-8", errors="replace")[-2000:]
        raise ValueError(
            "smoke receipt semantic verification failed: "
            f"{replay_receipt.get('errors', {})}; stderr={detail}"
        )
    verify_artifacts(receipt)
    if verification.get("status") != "PASS" or not all(
        verification.get("checks", {}).values()
    ):
        raise ValueError("formal mode requires a passing smoke receipt")
    return {
        "path": str(receipt),
        "manifest_sha256": sha256_file(manifest_path),
        "manifest_content_sha256": manifest.get("manifest_content_sha256"),
        "verification_sha256": sha256_file(verification_path),
        "artifact_manifest_sha256": sha256_file(
            receipt / "artifact_manifest.json"
        ),
        "source_pin_set_sha256": expected_config["source_pin_set_sha256"],
    }


def validate_contract(args: argparse.Namespace) -> None:
    if args.output is None:
        raise ValueError("parent run requires --output")
    if args.workers <= 0 or args.audit_workers <= 0:
        raise ValueError("workers must be positive")
    if args.child_timeout_seconds <= 0:
        raise ValueError("child timeout must be positive")
    if args.limit is not None and not 0 < args.limit <= FORMAL_N_EVAL:
        raise ValueError(f"limit must be in [1, {FORMAL_N_EVAL}]")
    if args.mode == "formal":
        if args.limit is not None:
            raise ValueError("formal mode does not accept --limit")
        expected = {
            "dataset_root": DEFAULT_DATASET_ROOT.resolve(strict=True),
            "table1_receipt": DEFAULT_TABLE1_RECEIPT.resolve(strict=True),
            "table2_manifest": DEFAULT_TABLE2_MANIFEST.resolve(strict=True),
            "table3_receipt": DEFAULT_TABLE3_RECEIPT.resolve(strict=True),
        }
        observed = {
            "dataset_root": args.dataset_root.resolve(strict=True),
            "table1_receipt": args.table1_receipt.resolve(strict=True),
            "table2_manifest": args.table2_manifest.resolve(strict=True),
            "table3_receipt": args.table3_receipt.resolve(strict=True),
        }
        if (
            observed != expected
            or args.workers != DEFAULT_WORKERS
            or args.audit_workers != DEFAULT_AUDIT_WORKERS
            or args.child_timeout_seconds != DEFAULT_TIMEOUT_SECONDS
        ):
            raise ValueError("formal mode requires frozen formal configuration")
        if not args.resume:
            validate_source_pins(DEFAULT_SOURCE_PINS)
            smoke_receipt_binding(args.smoke_receipt)
    elif not args.resume:
        validate_source_pins(DEFAULT_SOURCE_PINS)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), default="formal")
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--table1-receipt", type=Path, default=DEFAULT_TABLE1_RECEIPT)
    parser.add_argument("--table2-manifest", type=Path, default=DEFAULT_TABLE2_MANIFEST)
    parser.add_argument("--table3-receipt", type=Path, default=DEFAULT_TABLE3_RECEIPT)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--audit-workers", type=int, default=DEFAULT_AUDIT_WORKERS)
    parser.add_argument(
        "--child-timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS
    )
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--smoke-receipt", type=Path, default=None)
    parser.add_argument("--internal-child", nargs=3, default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.internal_child is not None:
        item_path, dataset_root, result_path = args.internal_child
        return run_child(Path(item_path), Path(dataset_root), Path(result_path))
    result = run(args)
    print(
        json.dumps(
            {
                "status": "completed",
                "n_eval": result["summary"]["n_eval"],
                "verification": result["verification"]["status"],
                "output": str(args.output),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
