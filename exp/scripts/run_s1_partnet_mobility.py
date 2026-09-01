#!/usr/bin/env python3
"""Run Supplementary Table S1 on the frozen PartNet-Mobility Table 4 cohort."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import platform
import sys
import tempfile
import time
from typing import Any, Mapping


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import lam_supplementary_static as static_atoms  # noqa: E402


DATASET = "PartNet-Mobility"
SCHEMA_VERSION = "supplementary-s1-partnet-mobility/v1"
PROTOCOL_ID = "s1_partnet_mobility_table4cohort_n800_salt20260813_v1"
N_EVAL = 800
TABLE4_PROTOCOL_ID = "urdf_sim_ready_table4_partnet_mobility_n800_v1"
TABLE4_RUN = REPO / "exp/runtime/urdf_table4_partnet_mobility_n800_20260813"
SOURCE_MANIFEST = TABLE4_RUN / "frozen_manifest.json"
TABLE4_ASSET_RECORDS = TABLE4_RUN / "asset_records.json"
TABLE4_STATE_RECORDS = TABLE4_RUN / "state_records.jsonl"
TABLE4_VERIFICATION = TABLE4_RUN / "verification.json"
DATASET_ROOT = REPO / "exp/PartNet-Mobility/data/dataset"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
VERIFIER_PATH = SCRIPT.with_name("verify_s1_partnet_mobility.py")
STATIC_ATOMS_PATH = Path(static_atoms.__file__).resolve()
DEFAULT_OUTPUT_PARENT = REPO / "exp/runtime"
DEFAULT_WORKERS = 16

EXPECTED_SOURCE_MANIFEST_SHA256 = "2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900"
EXPECTED_ASSET_RECORDS_SHA256 = "bdbfa385a74e44bd7662cba8f2c15ffbe3d664dfc0953722b8b37c44400430dc"
EXPECTED_STATE_RECORDS_SHA256 = "c72728ecfde2b0b6248da7048936b5bf52cb4f4cef0ae7438cf960e27895618c"
EXPECTED_TABLE4_VERIFICATION_SHA256 = "fe036bd6e2025d2a0c8b9aa14694b8affa4d3b0d6bf34b096dd93da111cc8e79"
EXPECTED_ORDERED_IDS_SHA256 = "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883"
EXPECTED_FORMAL_STRICT_PASSED = 567
EXPECTED_STATIC_ATOMS_SHA256 = "4701415dad8a5c0a434c16887979bcb70c250ba0b25772014e8db73789098e5f"

PAIR_POLICY = {
    "eligible_pairs": "distinct source-URDF links with collision geometry",
    "shared_topology_exclusion": "exclude_direct_parent_child",
    "method_specific_allowance": "none in headline",
    "surface_contact_allowed": True,
    "penetration_threshold_m": 1e-6,
}


class ProtocolViolation(RuntimeError):
    """Raised when a frozen input or identity binding has drifted."""


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def ordered_ids_sha256(values: list[str]) -> str:
    return canonical_sha256(values)


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def timestamp_tag() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def atomic_write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_write_json(path: Path, value: Any) -> None:
    atomic_write_text(path, json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False) + "\n")


def load_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"{label} is unavailable or invalid: {path}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"{label} root must be an object: {path}")
    return payload


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSONL at {path}:{line_number}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"JSONL row must be an object at {path}:{line_number}")
            rows.append(row)
    return rows


def nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a non-negative integer")
    return value


def ratio(passed: int, denominator: int) -> dict[str, int | float]:
    return {
        "passed": passed,
        "denominator": denominator,
        "rate": passed / denominator if denominator else 0.0,
    }


def _resolved_child(root: Path, relative: str, *, field: str) -> Path:
    safe = static_atoms.safe_package_relative_path(relative, field=field)
    candidate = root.joinpath(*safe.parts)
    if candidate.is_symlink():
        raise ValueError(f"{field} is a symlink: {relative}")
    resolved = candidate.resolve(strict=True)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{field} escapes package: {relative}") from exc
    if not resolved.is_file():
        raise ValueError(f"{field} is not a regular file: {relative}")
    return resolved


def load_cohort(
    manifest_path: Path,
    dataset_root: Path,
    *,
    formal: bool,
    limit: int | None = None,
) -> dict[str, Any]:
    manifest_path = manifest_path.resolve(strict=True)
    dataset_root = dataset_root.resolve(strict=True)
    observed_manifest_sha = sha256_file(manifest_path)
    if formal and observed_manifest_sha != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise ProtocolViolation("formal frozen manifest SHA256 mismatch")
    manifest = load_json_object(manifest_path, "frozen manifest")
    items = manifest.get("items")
    if manifest.get("status") != "FROZEN" or not isinstance(items, list):
        raise ValueError("cohort manifest is not a frozen item list")
    if Path(str(manifest.get("dataset_root", ""))).resolve(strict=True) != dataset_root:
        raise ValueError("cohort dataset root mismatch")
    if formal:
        if manifest.get("protocol_id") != TABLE4_PROTOCOL_ID:
            raise ProtocolViolation("formal Table 4 protocol identity mismatch")
        if manifest.get("sample_size") != N_EVAL or len(items) != N_EVAL:
            raise ProtocolViolation(f"formal cohort must contain exactly {N_EVAL} items")
        if manifest.get("ordered_selected_ids_sha256") != EXPECTED_ORDERED_IDS_SHA256:
            raise ProtocolViolation("formal ordered selected IDs declaration mismatch")
        if limit is not None:
            raise ValueError("formal cohort cannot be limited")
    if limit is not None:
        if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= len(items):
            raise ValueError("cohort limit is outside available items")
        items = items[:limit]

    records: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, raw in enumerate(items):
        if not isinstance(raw, Mapping):
            raise ValueError(f"cohort item {index} is not an object")
        dataset_id = raw.get("dataset_id")
        if not isinstance(dataset_id, str) or not dataset_id or dataset_id in seen:
            raise ValueError(f"invalid or duplicate dataset_id at item {index}")
        if Path(dataset_id).name != dataset_id or "/" in dataset_id or "\\" in dataset_id:
            raise ValueError(f"unsafe dataset_id at item {index}: {dataset_id!r}")
        seen.add(dataset_id)
        if raw.get("order") != index:
            raise ValueError(f"cohort order mismatch for {dataset_id}")
        package_candidate = dataset_root / dataset_id
        if package_candidate.is_symlink():
            raise ValueError(f"package is a symlink: {dataset_id}")
        package = package_candidate.resolve(strict=True)
        try:
            package.relative_to(dataset_root)
        except ValueError as exc:
            raise ValueError(f"package escapes dataset root: {dataset_id}") from exc
        if package.is_symlink() or not package.is_dir():
            raise ValueError(f"invalid package directory: {dataset_id}")
        urdf = _resolved_child(package, "mobility.urdf", field="primary_urdf")
        urdf_sha = sha256_file(urdf)
        if urdf_sha != raw.get("urdf_sha256"):
            raise ValueError(f"URDF SHA256 mismatch: {dataset_id}")
        meshes = raw.get("collision_mesh_files")
        if not isinstance(meshes, list):
            raise ValueError(f"collision mesh inventory missing: {dataset_id}")
        for mesh in meshes:
            if not isinstance(mesh, Mapping) or not isinstance(mesh.get("exists"), bool):
                raise ValueError(f"invalid collision mesh binding: {dataset_id}")
            relative = mesh.get("path")
            if not isinstance(relative, str):
                raise ValueError(f"invalid collision mesh path: {dataset_id}")
            if mesh["exists"]:
                path = _resolved_child(package, relative, field="collision_mesh")
                if sha256_file(path) != mesh.get("sha256") or path.stat().st_size != mesh.get("size_bytes"):
                    raise ValueError(f"collision mesh binding mismatch: {dataset_id}: {relative}")
            else:
                safe = static_atoms.safe_package_relative_path(relative, field="collision_mesh")
                candidate = package.joinpath(*safe.parts)
                if os.path.lexists(candidate) or mesh.get("sha256") is not None or mesh.get("size_bytes") is not None:
                    raise ValueError(f"declared-missing collision mesh binding mismatch: {dataset_id}: {relative}")
        record = {
            "selection_index": index,
            "asset_id": dataset_id,
            "dataset_id": dataset_id,
            "category": raw.get("category"),
            "package": str(package),
            "primary_urdf_relative_path": "mobility.urdf",
            "urdf_sha256_expected": urdf_sha,
            "collision_mesh_files_expected": [dict(mesh) for mesh in meshes],
            "table4_input_identity_sha256": raw.get("input_identity_sha256"),
            "movable_dof_count": raw.get("movable_dof_count"),
            "range_evaluable_dof_count": raw.get("range_evaluable_dof_count"),
            "rest_state_expected": raw.get("rest_state_expected"),
            "single_state_expected": raw.get("single_state_expected"),
            "sobol_state_expected": raw.get("sobol_state_expected"),
            "table4_protocol_id": raw.get("protocol_id", manifest.get("protocol_id")),
        }
        record["s1_input_identity_sha256"] = canonical_sha256(record)
        records.append(record)
    ids = [row["dataset_id"] for row in records]
    if formal and ordered_ids_sha256(ids) != EXPECTED_ORDERED_IDS_SHA256:
        raise ProtocolViolation("formal ordered dataset IDs SHA256 mismatch")
    return {
        "manifest_path": str(manifest_path),
        "manifest_file_sha256": observed_manifest_sha,
        "dataset_root": str(dataset_root),
        "ordered_dataset_ids_sha256": ordered_ids_sha256(ids),
        "records": records,
    }


def _reaggregate_table4_asset(
    item: Mapping[str, Any], asset: Mapping[str, Any], states: list[dict[str, Any]]
) -> dict[str, Any]:
    dataset_id = str(item["dataset_id"])
    if canonical_sha256(states) != asset.get("state_records_sha256"):
        raise ValueError(f"state records SHA256 mismatch: {dataset_id}")
    expected = {
        "rest": nonnegative_int(item.get("rest_state_expected"), "rest expected"),
        "single_joint_sweep": nonnegative_int(item.get("single_state_expected"), "single expected"),
        "multi_joint_sobol": nonnegative_int(item.get("sobol_state_expected"), "sobol expected"),
    }
    by_phase: dict[str, list[dict[str, Any]]] = {phase: [] for phase in expected}
    state_identities: set[tuple[Any, Any, Any]] = set()
    for state in states:
        if state.get("dataset_id") != dataset_id or state.get("category") != item.get("category"):
            raise ValueError(f"Table 4 state identity mismatch: {dataset_id}")
        phase = state.get("phase")
        if phase not in by_phase:
            raise ValueError(f"unknown Table 4 state phase: {dataset_id}: {phase!r}")
        nonnegative_int(state.get("non_adjacent_illegal_penetration_count"), "illegal penetration count")
        sample_index = nonnegative_int(state.get("sample_index"), "state sample_index")
        identity = (phase, state.get("joint_name"), sample_index)
        if identity in state_identities:
            raise ValueError(f"duplicate Table 4 state identity: {dataset_id}: {identity}")
        state_identities.add(identity)
        by_phase[str(phase)].append(state)
    executed = {phase: len(rows) for phase, rows in by_phase.items()}
    free = {
        phase: sum(row["non_adjacent_illegal_penetration_count"] == 0 for row in rows)
        for phase, rows in by_phase.items()
    }
    movable = nonnegative_int(item.get("movable_dof_count"), "movable DoF count")
    evaluable = nonnegative_int(item.get("range_evaluable_dof_count"), "range-evaluable DoF count")
    rest_pass = executed["rest"] == expected["rest"] and free["rest"] == expected["rest"]
    single_pass = (
        executed["single_joint_sweep"] == expected["single_joint_sweep"]
        and free["single_joint_sweep"] == expected["single_joint_sweep"]
    )
    sobol_pass = (
        movable > 0
        and evaluable == movable
        and executed["multi_joint_sobol"] == expected["multi_joint_sobol"]
        and free["multi_joint_sobol"] == expected["multi_joint_sobol"]
    )
    complete = evaluable == movable and sum(executed.values()) == sum(expected.values())
    strict = bool(complete and rest_pass and single_pass and sobol_pass)
    recorded = {
        "rest_state_executed": executed["rest"],
        "rest_state_expected": expected["rest"],
        "rest_non_adjacent_free": free["rest"],
        "rest_non_adjacent_cf": rest_pass,
        "single_state_executed": executed["single_joint_sweep"],
        "single_state_expected": expected["single_joint_sweep"],
        "single_non_adjacent_free": free["single_joint_sweep"],
        "single_joint_sweep_cf": single_pass,
        "sobol_state_executed": executed["multi_joint_sobol"],
        "sobol_state_expected": expected["multi_joint_sobol"],
        "sobol_non_adjacent_free": free["multi_joint_sobol"],
        "multi_joint_sobol_cf": sobol_pass,
        "measurement_complete": complete,
        "strict_collision_pass": strict,
    }
    for field, value in recorded.items():
        if asset.get(field) != value:
            raise ValueError(f"Table 4 asset {field} mismatch: {dataset_id}")
    return {
        "asset_id": dataset_id,
        "dataset_id": dataset_id,
        "selection_index": item["selection_index"],
        "measurement_complete": complete,
        "strict_collision_pass": strict,
        "state_record_count": len(states),
        "state_records_sha256": asset["state_records_sha256"],
        "table4_asset_record_sha256": canonical_sha256(asset),
    }


def load_table4_evidence(
    manifest_path: Path,
    asset_records_path: Path,
    state_records_path: Path,
    cohort: Mapping[str, Any],
    *,
    formal: bool,
    verification_path: Path | None = None,
) -> dict[str, Any]:
    manifest_path = manifest_path.resolve(strict=True)
    asset_records_path = asset_records_path.resolve(strict=True)
    state_records_path = state_records_path.resolve(strict=True)
    hashes = {
        "manifest": sha256_file(manifest_path),
        "assets": sha256_file(asset_records_path),
        "states": sha256_file(state_records_path),
    }
    if formal:
        expected = {
            "manifest": EXPECTED_SOURCE_MANIFEST_SHA256,
            "assets": EXPECTED_ASSET_RECORDS_SHA256,
            "states": EXPECTED_STATE_RECORDS_SHA256,
        }
        if hashes != expected:
            raise ProtocolViolation("formal Table 4 artifact SHA256 mismatch")
        if verification_path is None:
            verification_path = TABLE4_VERIFICATION
        verification_path = verification_path.resolve(strict=True)
        if sha256_file(verification_path) != EXPECTED_TABLE4_VERIFICATION_SHA256:
            raise ProtocolViolation("formal Table 4 verification SHA256 mismatch")
        receipt = load_json_object(verification_path, "Table 4 verification")
        bound = receipt.get("artifact_sha256")
        if (
            receipt.get("status") != "PASS"
            or not isinstance(bound, Mapping)
            or bound.get("frozen_manifest.json") != hashes["manifest"]
            or bound.get("asset_records.json") != hashes["assets"]
            or bound.get("state_records.jsonl") != hashes["states"]
        ):
            raise ProtocolViolation("Table 4 verification does not bind formal artifacts")

    manifest = load_json_object(manifest_path, "Table 4 manifest")
    assets = json.loads(asset_records_path.read_text(encoding="utf-8"))
    states = load_jsonl(state_records_path)
    items = manifest.get("items")
    sources = cohort.get("records")
    if not isinstance(items, list) or not isinstance(assets, list) or not isinstance(sources, list):
        raise ValueError("Table 4 evidence roots are invalid")
    if formal and manifest.get("protocol_id") != TABLE4_PROTOCOL_ID:
        raise ProtocolViolation("formal Table 4 protocol identity mismatch")
    if not formal:
        count = len(sources)
        if len(items) < count or len(assets) < count:
            raise ValueError("Table 4 evidence is shorter than smoke cohort")
        items = items[:count]
        assets = assets[:count]
        selected_ids = {row["dataset_id"] for row in sources}
        states = [row for row in states if row.get("dataset_id") in selected_ids]
    if len(items) != len(sources) or len(assets) != len(sources):
        raise ValueError("Table 4 evidence denominator mismatch")

    states_by_id: dict[str, list[dict[str, Any]]] = {row["dataset_id"]: [] for row in sources}
    seen_order: list[str] = []
    for state in states:
        dataset_id = state.get("dataset_id")
        if dataset_id not in states_by_id:
            raise ValueError(f"unknown Table 4 state dataset_id: {dataset_id!r}")
        if not seen_order or seen_order[-1] != dataset_id:
            if dataset_id in seen_order:
                raise ValueError("Table 4 state records are not grouped in cohort order")
            seen_order.append(str(dataset_id))
        states_by_id[str(dataset_id)].append(state)
    expected_order = [row["dataset_id"] for row in sources if states_by_id[row["dataset_id"]]]
    if seen_order != expected_order:
        raise ValueError("Table 4 state records do not follow cohort order")

    records: list[dict[str, Any]] = []
    for index, (source, item, asset) in enumerate(zip(sources, items, assets, strict=True)):
        if not isinstance(item, Mapping) or not isinstance(asset, Mapping):
            raise ValueError(f"Table 4 item or asset is invalid at index {index}")
        for field in ("dataset_id", "category", "movable_dof_count", "range_evaluable_dof_count"):
            if item.get(field) != source.get(field):
                raise ValueError(f"Table 4 cohort {field} mismatch at index {index}")
        if item.get("order") != index or asset.get("order") != index:
            raise ValueError(f"Table 4 order mismatch at index {index}")
        for field in ("dataset_id", "category", "input_identity_sha256", "protocol_id"):
            if asset.get(field) != item.get(field):
                raise ValueError(f"Table 4 asset {field} mismatch at index {index}")
        records.append(_reaggregate_table4_asset(source, asset, states_by_id[source["dataset_id"]]))
    strict_passed = sum(bool(row["strict_collision_pass"]) for row in records)
    if formal and strict_passed != EXPECTED_FORMAL_STRICT_PASSED:
        raise ProtocolViolation("formal Table 4 strict-pass aggregate mismatch")
    return {
        "protocol_id": manifest.get("protocol_id"),
        "manifest_path": str(manifest_path),
        "manifest_file_sha256": hashes["manifest"],
        "asset_records_path": str(asset_records_path),
        "asset_records_file_sha256": hashes["assets"],
        "state_records_path": str(state_records_path),
        "state_records_file_sha256": hashes["states"],
        "strict_passed": strict_passed,
        "denominator": len(records),
        "state_record_count": len(states),
        "records": records,
    }


def evidence_inventory_for_package(source: Mapping[str, Any]) -> dict[str, Any]:
    package = Path(str(source["package"]))
    candidates: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            if (current / name).is_symlink():
                raise ValueError(f"package contains directory symlink: {source['dataset_id']}: {name}")
        for name in file_names:
            path = current / name
            lower = name.lower()
            kinds: list[str] = []
            if path.suffix.lower() == ".json" and static_atoms.RECEIPT_NAME_RE.search(lower):
                kinds.append("mechanical_receipt")
            if lower in static_atoms.REBUILD_RECIPE_NAMES:
                kinds.append("rebuild_recipe")
            if path.suffix.lower() == ".json" and static_atoms.ALLOWANCE_NAME_RE.search(lower):
                kinds.append("allowance_registry")
            if not kinds:
                continue
            if path.is_symlink():
                raise ValueError(f"evidence candidate is a symlink: {source['dataset_id']}: {name}")
            resolved = path.resolve(strict=True)
            try:
                relative = resolved.relative_to(package)
            except ValueError as exc:
                raise ValueError(f"evidence candidate escapes package: {source['dataset_id']}: {name}") from exc
            candidates.append(
                {
                    "path": relative.as_posix(),
                    "kinds": kinds,
                    "sha256": sha256_file(resolved),
                    "bytes": resolved.stat().st_size,
                }
            )
    candidates.sort(key=lambda row: (row["path"], row["kinds"]))
    return {
        "selection_index": source["selection_index"],
        "dataset_id": source["dataset_id"],
        "package": source["package"],
        "candidates": candidates,
        "candidate_count": len(candidates),
        "candidates_sha256": canonical_sha256(candidates),
    }


def _inventory_payload(source: dict[str, Any]) -> dict[str, Any]:
    return evidence_inventory_for_package(source)


def build_evidence_inventory(sources: list[dict[str, Any]], *, workers: int) -> dict[str, Any]:
    if workers == 1:
        records = [evidence_inventory_for_package(source) for source in sources]
    else:
        with multiprocessing.get_context("fork").Pool(processes=workers) as pool:
            records = pool.map(_inventory_payload, sources)
    return {
        "schema_version": "s1-evidence-candidate-inventory/v1",
        "protocol_id": PROTOCOL_ID,
        "record_count": len(records),
        "candidate_count": sum(row["candidate_count"] for row in records),
        "records": records,
        "records_sha256": canonical_sha256(records),
    }


def audit_release_evidence(source: Mapping[str, Any]) -> dict[str, Any]:
    package = Path(str(source["package"])).resolve(strict=True)
    audit = static_atoms.audit_lam_package(
        package,
        urdf_relative_path=str(source["primary_urdf_relative_path"]),
        asset_id=str(source["dataset_id"]),
        expected_movable_joints=nonnegative_int(source.get("movable_dof_count"), "movable DoF count"),
    )
    dataset_id = str(source["dataset_id"])
    if audit.get("status") != "completed" or audit.get("urdf_sha256") != source.get("urdf_sha256_expected"):
        raise ValueError(f"S1 static audit or URDF binding failed: {dataset_id}")
    closure = audit.get("resource_closure")
    evidence = audit.get("s1_evidence")
    if not isinstance(closure, Mapping):
        raise ValueError(f"resource closure unavailable: {dataset_id}")
    if not isinstance(evidence, Mapping):
        raise ValueError(f"S1 release evidence missing: {dataset_id}")
    receipt = evidence.get("receipt")
    rebuild = evidence.get("rebuild")
    allowance = evidence.get("allowance")
    if not all(isinstance(value, Mapping) for value in (receipt, rebuild, allowance)):
        raise ValueError(f"S1 evidence component missing: {dataset_id}")
    if allowance.get("status") != "COMPLETE":
        raise ValueError(f"allowance topology is not evaluable: {dataset_id}")
    receipt_bound = bool(receipt.get("receipt_bound_asset"))
    rebuild_eligible = bool(rebuild.get("eligible_asset"))
    if closure.get("complete") is not True and receipt_bound:
        raise ValueError(f"valid receipt cannot bind an incomplete resource closure: {dataset_id}")
    return {
        "resource_closure": dict(closure),
        "receipt": dict(receipt),
        "receipt_replay": {
            "eligible_receipt_count": int(receipt.get("valid_mechanical_receipt_count", 0)),
            "attempted": 0,
            "passed": False,
            "status": "VALID_RECEIPT_NOT_REPLAYED" if receipt_bound else "NO_VALID_RECEIPT",
        },
        "rebuild": {
            **dict(rebuild),
            "status": "ELIGIBLE_NOT_RUN" if rebuild_eligible else "N/E",
        },
        "allowance": dict(allowance),
        "issues": list(audit.get("issues", [])),
    }


def _audit_payload(source: dict[str, Any]) -> dict[str, Any]:
    return audit_release_evidence(source)


def build_s1_asset_record(
    source: Mapping[str, Any], evidence: Mapping[str, Any], table4: Mapping[str, Any]
) -> dict[str, Any]:
    dataset_id = str(source["dataset_id"])
    if table4.get("dataset_id") != dataset_id or table4.get("selection_index") != source.get("selection_index"):
        raise ValueError(f"Table 4 result identity mismatch: {dataset_id}")
    allowance = evidence.get("allowance")
    receipt = evidence.get("receipt")
    replay = evidence.get("receipt_replay")
    rebuild = evidence.get("rebuild")
    if not all(isinstance(value, Mapping) for value in (allowance, receipt, replay, rebuild)):
        raise ValueError(f"incomplete S1 evidence: {dataset_id}")
    registered = nonnegative_int(allowance.get("registered_excluded_pair_count"), "registered pair count")
    eligible = nonnegative_int(allowance.get("eligible_nonadjacent_pair_count"), "eligible pair count")
    if registered:
        raise ValueError("non-empty registered allowance requires pair-specific replay")
    strict = bool(table4.get("strict_collision_pass"))
    return {
        "selection_index": source["selection_index"],
        "asset_id": dataset_id,
        "dataset_id": dataset_id,
        "category": source.get("category"),
        "package": source["package"],
        "primary_urdf_relative_path": source["primary_urdf_relative_path"],
        "urdf_sha256": source["urdf_sha256_expected"],
        "table4_input_identity_sha256": source["table4_input_identity_sha256"],
        "s1_input_identity_sha256": source["s1_input_identity_sha256"],
        "resource_closure": evidence["resource_closure"],
        "release_receipt_bound": bool(receipt.get("receipt_bound_asset")),
        "release_receipt_replay_pass": bool(replay.get("passed")),
        "receipt_replay_status": replay.get("status"),
        "deterministic_rebuild_eligible": bool(rebuild.get("eligible_asset")),
        "deterministic_rebuild_match": False,
        "deterministic_rebuild_status": rebuild.get("status"),
        "registered_excluded_pair_count": registered,
        "eligible_nonadjacent_pair_count": eligible,
        "strict_collision_pass_no_method_allowance": strict,
        "strict_collision_pass_registered_allowance": strict,
        "table4_measurement_complete": bool(table4.get("measurement_complete")),
        "table4_state_record_count": table4.get("state_record_count"),
        "table4_state_records_sha256": table4.get("state_records_sha256"),
        "table4_asset_record_sha256": table4.get("table4_asset_record_sha256"),
        "release_evidence": dict(evidence),
        "status": "completed",
    }


def aggregate_s1(records: list[Mapping[str, Any]]) -> dict[str, Any]:
    denominator = len(records)
    receipt_bound = sum(bool(row.get("release_receipt_bound")) for row in records)
    replayed = sum(bool(row.get("release_receipt_replay_pass")) for row in records)
    rebuild_eligible = sum(bool(row.get("deterministic_rebuild_eligible")) for row in records)
    rebuild_match = sum(bool(row.get("deterministic_rebuild_match")) for row in records)
    registered_pairs = sum(nonnegative_int(row.get("registered_excluded_pair_count"), "registered pair count") for row in records)
    eligible_pairs = sum(nonnegative_int(row.get("eligible_nonadjacent_pair_count"), "eligible pair count") for row in records)
    strict = sum(bool(row.get("strict_collision_pass_no_method_allowance")) for row in records)
    registered_strict = sum(bool(row.get("strict_collision_pass_registered_allowance")) for row in records)
    rebuild_metric: dict[str, Any] = {
        "status": "N/E" if rebuild_eligible == 0 else "NOT_RUN",
        "passed": None if rebuild_eligible == 0 else rebuild_match,
        "denominator": rebuild_eligible,
        "rate": None if rebuild_eligible == 0 else rebuild_match / rebuild_eligible,
        "eligible_assets": rebuild_eligible,
        "asset_denominator": denominator,
    }
    return {
        "receipt_bound_assets": ratio(receipt_bound, denominator),
        "receipt_replay_pass": ratio(replayed, denominator),
        "deterministic_rebuild_match": rebuild_metric,
        "allowance_density": {
            "registered_pairs": registered_pairs,
            "eligible_pairs": eligible_pairs,
            "rate": registered_pairs / eligible_pairs if eligible_pairs else None,
        },
        "strict_pass_no_method_allowance": ratio(strict, denominator),
        "registered_allowance_gain_pp": {
            "value": 100.0 * (registered_strict - strict) / denominator if denominator else 0.0,
            "registered_passed": registered_strict,
            "no_allowance_passed": strict,
            "denominator": denominator,
        },
    }


def render_summary(summary: Mapping[str, Any]) -> str:
    metrics = summary["metrics"]

    def fraction(metric: Mapping[str, Any]) -> str:
        return f"{metric['passed']} / {metric['denominator']} ({100.0 * metric['rate']:.2f}%)"

    rebuild = metrics["deterministic_rebuild_match"]
    allowance = metrics["allowance_density"]
    return "\n".join(
        [
            "# Supplementary Table S1: PartNet-Mobility",
            "",
            f"- Protocol: `{summary['protocol_id']}`",
            f"- Status: `{summary['status']}`",
            f"- N_eval: {summary['n_eval']}",
            "",
            "| Metric | Result |",
            "|---|---:|",
            f"| Receipt-bound Assets | {fraction(metrics['receipt_bound_assets'])} |",
            f"| Receipt Replay Pass | {fraction(metrics['receipt_replay_pass'])} |",
            f"| Deterministic Rebuild Match | {rebuild['status']} ({rebuild['eligible_assets']} / {rebuild['asset_denominator']} eligible) |",
            f"| Allowance Density | {allowance['registered_pairs']} / {allowance['eligible_pairs']} ({100.0 * (allowance['rate'] or 0.0):.2f}%) |",
            f"| Strict Pass (No Method-specific Allowance) | {fraction(metrics['strict_pass_no_method_allowance'])} |",
            f"| Registered-allowance Gain | {metrics['registered_allowance_gain_pp']['value']:.2f} pp |",
            "",
        ]
    )


def run_evaluation(
    *,
    output: Path,
    cohort_manifest: Path,
    dataset_root: Path,
    table4_manifest: Path,
    table4_asset_records: Path,
    table4_state_records: Path,
    protocol_document: Path,
    formal: bool,
    workers: int,
    limit: int | None = None,
    table4_verification: Path | None = None,
) -> dict[str, Any]:
    if workers < 1:
        raise ValueError("workers must be positive")
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    started_at = utc_now()
    protocol_document = protocol_document.resolve(strict=True)
    protocol_bytes = protocol_document.read_bytes()
    code_identity = {
        "runner_path": str(SCRIPT),
        "runner_sha256": sha256_file(SCRIPT),
        "verifier_path": str(VERIFIER_PATH),
        "verifier_sha256": sha256_file(VERIFIER_PATH),
        "static_atoms_path": str(STATIC_ATOMS_PATH),
        "static_atoms_sha256": sha256_file(STATIC_ATOMS_PATH),
    }
    if formal and code_identity["static_atoms_sha256"] != EXPECTED_STATIC_ATOMS_SHA256:
        raise ProtocolViolation("formal static S1 atom SHA256 mismatch")
    protocol_snapshot = output / "protocol_snapshot.md"
    atomic_write_text(protocol_snapshot, protocol_bytes.decode("utf-8"))
    cohort = load_cohort(cohort_manifest, dataset_root, formal=formal, limit=limit)
    sources = cohort["records"]
    evidence_inventory = build_evidence_inventory(sources, workers=workers)
    evidence_inventory_path = output / "evidence_inventory.json"
    atomic_write_json(evidence_inventory_path, evidence_inventory)
    table4 = load_table4_evidence(
        table4_manifest,
        table4_asset_records,
        table4_state_records,
        cohort,
        formal=formal,
        verification_path=table4_verification,
    )
    if workers == 1:
        evidence_rows = [audit_release_evidence(row) for row in sources]
    else:
        with multiprocessing.get_context("fork").Pool(processes=workers) as pool:
            evidence_rows = pool.map(_audit_payload, sources)
    records = [
        build_s1_asset_record(source, evidence, strict)
        for source, evidence, strict in zip(sources, evidence_rows, table4["records"], strict=True)
    ]
    metrics = aggregate_s1(records)
    if formal:
        if metrics["receipt_bound_assets"]["passed"] != 0:
            raise ProtocolViolation("formal release unexpectedly contains a valid mechanical receipt")
        if metrics["deterministic_rebuild_match"]["eligible_assets"] != 0:
            raise ProtocolViolation("formal release unexpectedly contains an eligible rebuild recipe")
        if metrics["allowance_density"]["registered_pairs"] != 0:
            raise ProtocolViolation("formal release unexpectedly contains a registered allowance")
        if metrics["strict_pass_no_method_allowance"]["passed"] != EXPECTED_FORMAL_STRICT_PASSED:
            raise ProtocolViolation("formal strict-pass result drifted")

    if protocol_document.read_bytes() != protocol_bytes:
        raise ProtocolViolation("protocol document changed during evaluation")
    current_code_identity = {
        key: (value if key.endswith("_path") else sha256_file(Path(code_identity[key.replace("_sha256", "_path")])))
        for key, value in code_identity.items()
    }
    if current_code_identity != code_identity:
        raise ProtocolViolation("evaluator code changed during evaluation")
    if build_evidence_inventory(sources, workers=workers) != evidence_inventory:
        raise ProtocolViolation("evidence candidate inventory changed during evaluation")
    config = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "classification": "FORMAL" if formal else "SMOKE",
        "n_eval": len(records),
        "intended_n_eval": N_EVAL,
        "workers": workers,
        "limit": limit,
        "pair_policy": PAIR_POLICY,
        "source": {
            "cohort_manifest": cohort["manifest_path"],
            "cohort_manifest_sha256": cohort["manifest_file_sha256"],
            "dataset_root": cohort["dataset_root"],
            "ordered_dataset_ids_sha256": cohort["ordered_dataset_ids_sha256"],
            "table4_manifest": table4["manifest_path"],
            "table4_manifest_sha256": table4["manifest_file_sha256"],
            "table4_asset_records": table4["asset_records_path"],
            "table4_asset_records_sha256": table4["asset_records_file_sha256"],
            "table4_state_records": table4["state_records_path"],
            "table4_state_records_sha256": table4["state_records_file_sha256"],
            "protocol_document": str(protocol_document),
            "protocol_snapshot_sha256": sha256_file(protocol_snapshot),
            "evidence_inventory": str(evidence_inventory_path.resolve()),
            "evidence_inventory_sha256": sha256_file(evidence_inventory_path),
        },
        "code_identity": code_identity,
        "runtime": {
            "python": sys.version,
            "python_executable": sys.executable,
            "platform": platform.platform(),
        },
    }
    frozen_config = output / "frozen_config.json"
    atomic_write_json(frozen_config, config)
    asset_records_path = output / "asset_records.jsonl"
    atomic_write_text(asset_records_path, "".join(canonical_json(record) + "\n" for record in records))
    summary = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "classification": config["classification"],
        "status": "COMPLETE",
        "started_at": started_at,
        "completed_at": utc_now(),
        "wall_time_seconds": time.monotonic() - started,
        "n_eval": len(records),
        "terminal_records": len(records),
        "error_records": 0,
        "table4_state_record_count": table4["state_record_count"],
        "metrics": metrics,
    }
    summary_path = output / "summary.json"
    summary_md_path = output / "summary.md"
    atomic_write_json(summary_path, summary)
    atomic_write_text(summary_md_path, render_summary(summary))
    artifacts = {}
    for path in (frozen_config, protocol_snapshot, evidence_inventory_path, asset_records_path, summary_path, summary_md_path):
        artifacts[path.name] = {"sha256": sha256_file(path), "bytes": path.stat().st_size}
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "classification": config["classification"],
        "created_at": utc_now(),
        "n_eval": len(records),
        "artifacts": artifacts,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    atomic_write_json(output / "manifest.json", manifest)
    return summary


def default_output(formal: bool) -> Path:
    label = "table4cohort_n800_salt20260813" if formal else "smoke"
    return DEFAULT_OUTPUT_PARENT / f"s1_partnet_mobility_{label}_{timestamp_tag()}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args(argv)
    formal = not args.smoke
    if formal and args.limit is not None:
        parser.error("--limit requires --smoke")
    output = args.output or default_output(formal)
    summary = run_evaluation(
        output=output,
        cohort_manifest=SOURCE_MANIFEST,
        dataset_root=DATASET_ROOT,
        table4_manifest=SOURCE_MANIFEST,
        table4_asset_records=TABLE4_ASSET_RECORDS,
        table4_state_records=TABLE4_STATE_RECORDS,
        table4_verification=TABLE4_VERIFICATION,
        protocol_document=PROTOCOL_DOCUMENT,
        formal=formal,
        workers=args.workers,
        limit=args.limit,
    )
    print(json.dumps({"output": str(output), "status": summary["status"], "metrics": summary["metrics"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
