#!/usr/bin/env python3
"""Evaluate Supplementary Table S1 on the frozen Articraft-10K cohort."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
from typing import Any, Mapping
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import lam_supplementary_static as static_atoms  # noqa: E402

DATASET = "Articraft-10K"
SCHEMA_VERSION = "s1-articraft10k-run/v1"
PROTOCOL_ID = "s1_articraft10k_table2cohort_n800_seed20260813_v1"
N_EVAL = 800
SOURCE_MANIFEST = (
    REPO
    / "exp/runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json"
)
DATASET_ROOT = REPO / "exp/Articraft-10K"
EXPECTED_SOURCE_MANIFEST_SHA256 = (
    "13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d"
)
EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256 = (
    "576852cb6da00775e1c51360b82b4be40e0a614e4fb0cfb1bae066912eed56a3"
)
EXPECTED_ORDERED_ASSET_IDS_SHA256 = (
    "79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784"
)
TABLE4_RUN = REPO / "exp/runtime/urdf_table4_articraft10k_n800_20260814"
TABLE4_MANIFEST = TABLE4_RUN / "frozen_manifest.json"
TABLE4_ASSET_RECORDS = TABLE4_RUN / "asset_records.json"
TABLE4_STATE_RECORDS = TABLE4_RUN / "state_records.jsonl"
EXPECTED_TABLE4_MANIFEST_SHA256 = (
    "6b4275cf3da29244af70c04acecd87094f0c158dee992db20b04e90c05292c20"
)
EXPECTED_TABLE4_ASSET_RECORDS_SHA256 = (
    "b732a53a464a8aeebb74799d5ec737de75f3cca377c9a5b274a5dd35adbe301b"
)
EXPECTED_TABLE4_STATE_RECORDS_SHA256 = (
    "6efd4031ecebf74f30f8d3ec3c312ae2faf1b521322b5d4a8b57bb732177ac8b"
)
EXPECTED_TABLE4_PROTOCOL_ID = "urdf_sim_ready_table4_articraft10k_n800_v1"
EXPECTED_FORMAL_STRICT_PASSED = 147
STATIC_ATOMS_PATH = Path(static_atoms.__file__).resolve()
VERIFIER_PATH = SCRIPT.with_name("verify_s1_articraft10k.py")
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
CATEGORY_RECORDS_ROOT = REPO / "exp/baselines/Articraft-10K-official/records"
DEFAULT_OUTPUT_PARENT = REPO / "exp/runtime"
DEFAULT_WORKERS = 4
EXPECTED_STATIC_ATOMS_SHA256 = (
    "4701415dad8a5c0a434c16887979bcb70c250ba0b25772014e8db73789098e5f"
)
EXPECTED_FORMAL_ELIGIBLE_PAIRS = 3040
EXPECTED_FORMAL_OFFICIAL_SOURCE_MANIFEST_SHA256 = (
    "11a37014d2d73782f502f2043b589915f663e16094470366529d5e944d777f47"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSONL at {path}:{line_number}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"JSONL row must be an object at {path}:{line_number}")
            rows.append(row)
    return rows


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def atomic_write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
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
    atomic_write_text(
        path,
        json.dumps(
            value,
            indent=2,
            sort_keys=True,
            ensure_ascii=True,
            allow_nan=False,
        )
        + "\n",
    )


def package_binding(package: Path) -> dict[str, Any]:
    package = package.resolve(strict=True)
    if not package.is_dir():
        raise ValueError(f"package is not a directory: {package}")
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if child.is_symlink():
                raise ValueError(
                    f"package contains directory symlink: {child.relative_to(package)}"
                )
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            resolved = path.resolve(strict=True)
            try:
                resolved.relative_to(package)
            except ValueError as exc:
                raise ValueError(f"package file escapes package: {relative}") from exc
            if not resolved.is_file():
                raise ValueError(f"package entry is not a regular file: {relative}")
            files.append(
                {
                    "path": relative,
                    "bytes": resolved.stat().st_size,
                    "sha256": sha256_file(resolved),
                }
            )
    return {
        "file_count": len(files),
        "total_bytes": sum(int(row["bytes"]) for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def _validate_record(
    record: Mapping[str, Any],
    index: int,
    release_root: Path,
) -> dict[str, Any]:
    asset_id = record.get("asset_id")
    if not isinstance(asset_id, str) or not asset_id:
        raise ValueError(f"invalid asset_id at selection index {index}")
    if record.get("selection_index") != index:
        raise ValueError(f"selection_index mismatch at index {index}")
    package = Path(str(record.get("package", ""))).resolve(strict=True)
    try:
        package.relative_to(release_root)
    except ValueError as exc:
        raise ValueError(f"package escapes release root at index {index}") from exc
    if package.name != asset_id:
        raise ValueError(f"package/asset_id mismatch at index {index}")
    observed_binding = package_binding(package)
    if observed_binding != record.get("package_binding"):
        raise ValueError(f"package binding mismatch: {asset_id}")
    urdf_path = package / "model.urdf"
    observed_urdf_sha256 = sha256_file(urdf_path)
    if observed_urdf_sha256 != record.get("model_urdf_sha256"):
        raise ValueError(f"model.urdf SHA256 mismatch: {asset_id}")
    return {
        **dict(record),
        "package": str(package),
        "package_content_manifest_sha256": observed_binding[
            "content_manifest_sha256"
        ],
    }


def load_cohort(
    manifest_path: Path,
    dataset_root: Path,
    *,
    formal: bool,
    limit: int | None = None,
) -> dict[str, Any]:
    manifest_path = manifest_path.resolve(strict=True)
    dataset_root = dataset_root.resolve(strict=True)
    if formal and sha256_file(manifest_path) != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise RuntimeError("formal source manifest file SHA256 mismatch")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("dataset") != DATASET:
        raise ValueError("source manifest dataset mismatch")
    records_raw = manifest.get("records")
    if not isinstance(records_raw, list):
        raise ValueError("source manifest records must be a list")
    if limit is not None:
        if formal:
            raise ValueError("formal cohort cannot be limited")
        if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= len(
            records_raw
        ):
            raise ValueError("cohort limit is outside the available records")
        records_raw = records_raw[:limit]
    if formal:
        if len(records_raw) != N_EVAL:
            raise RuntimeError(f"formal cohort must contain exactly {N_EVAL} records")
        if (
            manifest.get("manifest_content_sha256")
            != EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256
        ):
            raise RuntimeError("formal source manifest content SHA256 mismatch")
    release_root = (dataset_root / "released_urdf").resolve(strict=True)
    records = [
        _validate_record(record, index, release_root)
        for index, record in enumerate(records_raw)
    ]
    ordered_ids_sha256 = canonical_sha256([record["asset_id"] for record in records])
    if formal and ordered_ids_sha256 != EXPECTED_ORDERED_ASSET_IDS_SHA256:
        raise RuntimeError("formal ordered asset IDs SHA256 mismatch")
    return {
        "manifest_path": str(manifest_path),
        "manifest_file_sha256": sha256_file(manifest_path),
        "manifest_content_sha256": manifest.get("manifest_content_sha256"),
        "ordered_asset_ids_sha256": ordered_ids_sha256,
        "records": records,
    }


def _load_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"{label} is unavailable or invalid: {path}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"{label} root must be an object: {path}")
    return payload


def _official_model_py_evidence(
    records_root: Path,
    asset_id: str,
    compile_report: Mapping[str, Any],
) -> dict[str, Any]:
    record_root = (records_root / asset_id).resolve(strict=True)
    record_path = record_root / "record.json"
    record = _load_json_object(record_path, "official record")
    if record.get("record_id") != asset_id:
        raise ValueError(f"official record identity mismatch: {asset_id}")
    artifacts = record.get("artifacts")
    hashes = record.get("hashes")
    raw_model_path = artifacts.get("model_py") if isinstance(artifacts, Mapping) else None
    expected_hash = (
        hashes.get("model_py_sha256") if isinstance(hashes, Mapping) else None
    )
    compile_metrics = compile_report.get("metrics")
    fingerprint_inputs = (
        compile_metrics.get("fingerprint_inputs")
        if isinstance(compile_metrics, Mapping)
        else None
    )
    compile_hash = (
        fingerprint_inputs.get("model_py_sha256")
        if isinstance(fingerprint_inputs, Mapping)
        else None
    )
    declared = isinstance(raw_model_path, str) and bool(raw_model_path)
    exists = False
    observed_hash: str | None = None
    normalized_path: str | None = None
    if declared:
        relative = static_atoms.safe_package_relative_path(
            raw_model_path, field="official_model_py"
        )
        candidate = record_root.joinpath(*relative.parts)
        normalized_path = relative.as_posix()
        if candidate.is_symlink():
            raise ValueError(f"official model.py is a symlink: {asset_id}")
        if candidate.is_file():
            resolved = candidate.resolve(strict=True)
            try:
                resolved.relative_to(record_root)
            except ValueError as exc:
                raise ValueError(f"official model.py escapes record root: {asset_id}") from exc
            exists = True
            observed_hash = sha256_file(resolved)
    return {
        "record_json_path": str(record_path.resolve(strict=True)),
        "record_json_sha256": sha256_file(record_path),
        "declared": declared,
        "path": normalized_path,
        "exists": exists,
        "expected_sha256": expected_hash,
        "observed_sha256": observed_hash,
        "compile_report_model_py_sha256": compile_hash,
        "metadata_hash_match": bool(
            isinstance(expected_hash, str)
            and isinstance(compile_hash, str)
            and expected_hash == compile_hash
        ),
        "content_hash_match": bool(
            exists
            and isinstance(expected_hash, str)
            and observed_hash == expected_hash
        ),
    }


def audit_release_evidence(
    cohort_record: Mapping[str, Any],
    category_records_root: Path,
) -> dict[str, Any]:
    asset_id = str(cohort_record["asset_id"])
    package = Path(str(cohort_record["package"])).resolve(strict=True)
    static_record = static_atoms.audit_lam_package(
        package,
        urdf_relative_path="model.urdf",
        asset_id=asset_id,
    )
    if static_record.get("status") != "completed":
        raise ValueError(f"S1 static audit failed for {asset_id}: {static_record['issues']}")
    if static_record.get("urdf_sha256") != cohort_record.get("model_urdf_sha256"):
        raise ValueError(f"S1 static audit URDF identity mismatch: {asset_id}")
    s1 = static_record.get("s1_evidence")
    if not isinstance(s1, Mapping):
        raise ValueError(f"S1 static evidence missing: {asset_id}")

    compile_path = package / "compile_report.json"
    compile_report = _load_json_object(compile_path, "compile report")
    if compile_report.get("record_id") != asset_id:
        raise ValueError(f"compile report identity mismatch: {asset_id}")
    overlap_allowances = compile_report.get("overlap_allowances")
    if not isinstance(overlap_allowances, list):
        raise ValueError(f"compile report overlap_allowances must be a list: {asset_id}")

    static_allowance = s1.get("allowance")
    if not isinstance(static_allowance, Mapping):
        raise ValueError(f"static allowance evidence missing: {asset_id}")
    eligible_count = static_allowance.get("eligible_nonadjacent_pair_count")
    static_registered = static_allowance.get("registered_excluded_pair_count")
    if not isinstance(eligible_count, int) or eligible_count < 0:
        raise ValueError(f"eligible pair count unavailable: {asset_id}")
    if not isinstance(static_registered, int) or static_registered < 0:
        raise ValueError(f"registered allowance count unavailable: {asset_id}")
    compile_pairs, compile_pair_issues = static_atoms._extract_allowance_pairs(
        {"excluded_non_adjacent_pairs": overlap_allowances}
    )
    if compile_pair_issues:
        raise ValueError(
            f"compile report allowance registry invalid for {asset_id}: "
            + "; ".join(compile_pair_issues)
        )
    try:
        urdf_root = ET.parse(package / "model.urdf").getroot()
    except (OSError, ET.ParseError) as exc:
        raise ValueError(f"allowance topology is unavailable: {asset_id}") from exc
    eligible_pairs, topology_issues = static_atoms._eligible_nonadjacent_pairs(
        urdf_root
    )
    if topology_issues or len(eligible_pairs) != eligible_count:
        raise ValueError(f"allowance topology evidence mismatch: {asset_id}")
    invalid_compile_pairs = sorted(compile_pairs - eligible_pairs)
    if invalid_compile_pairs:
        left, right = invalid_compile_pairs[0]
        raise ValueError(
            "compile report allowance pair is not eligible: "
            f"{asset_id}: {left}::{right}"
        )

    release_registry_pairs: set[tuple[str, str]] = set()
    release_registry_paths: list[str] = []
    for record in static_allowance.get("records", []):
        if not isinstance(record, Mapping) or record.get("valid") is not True:
            continue
        raw_path = record.get("path")
        if not isinstance(raw_path, str):
            raise ValueError(f"allowance registry path is invalid: {asset_id}")
        relative = static_atoms.safe_package_relative_path(
            raw_path, field="allowance_registry"
        )
        registry_path = package.joinpath(*relative.parts).resolve(strict=True)
        try:
            registry_path.relative_to(package)
        except ValueError as exc:
            raise ValueError(f"allowance registry escapes package: {asset_id}") from exc
        registry_payload = _load_json_object(registry_path, "allowance registry")
        registry_pairs, registry_issues = static_atoms._extract_allowance_pairs(
            registry_payload
        )
        if registry_issues or not registry_pairs <= eligible_pairs:
            raise ValueError(f"allowance registry evidence mismatch: {asset_id}")
        release_registry_pairs.update(registry_pairs)
        release_registry_paths.append(relative.as_posix())
    if len(release_registry_pairs) != static_registered:
        raise ValueError(f"static allowance registry count mismatch: {asset_id}")
    registered_count = len(release_registry_pairs | compile_pairs)

    receipt_raw = s1.get("receipt")
    rebuild_raw = s1.get("rebuild")
    if not isinstance(receipt_raw, Mapping) or not isinstance(rebuild_raw, Mapping):
        raise ValueError(f"receipt or rebuild evidence missing: {asset_id}")
    official_model_py = _official_model_py_evidence(
        Path(category_records_root), asset_id, compile_report
    )
    rebuild_eligible = bool(rebuild_raw.get("eligible_asset"))
    return {
        "asset_id": asset_id,
        "resource_closure": static_record["resource_closure"],
        "receipt": {
            **dict(receipt_raw),
            "receipt_bound_asset": bool(receipt_raw.get("receipt_bound_asset")),
        },
        "compile_report": {
            "path": "compile_report.json",
            "sha256": sha256_file(compile_path),
            "mechanical_receipt": False,
            "status": compile_report.get("status"),
            "overlap_allowances": overlap_allowances,
        },
        "rebuild": {
            **dict(rebuild_raw),
            "status": "ELIGIBLE_NOT_RUN" if rebuild_eligible else "N/E",
            "eligible_asset": rebuild_eligible,
            "official_model_py": official_model_py,
        },
        "allowance": {
            **dict(static_allowance),
            "status": "COMPLETE",
            "registered_excluded_pair_count": registered_count,
            "registry_sources": [
                "compile_report.json",
                *release_registry_paths,
            ],
        },
        "issues": list(static_record.get("issues", [])),
    }


def _nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a non-negative integer")
    return value


def _require_identity(
    source: Mapping[str, Any],
    observed: Mapping[str, Any],
    fields: tuple[str, ...],
    label: str,
) -> None:
    for field in fields:
        if observed.get(field) != source.get(field):
            raise ValueError(f"{label} {field} mismatch")


def _reaggregate_table4_asset(
    item: Mapping[str, Any],
    asset: Mapping[str, Any],
    states: list[dict[str, Any]],
) -> dict[str, Any]:
    asset_id = str(item["asset_id"])
    if canonical_sha256(states) != asset.get("state_records_sha256"):
        raise ValueError(f"state records SHA256 mismatch: {asset_id}")
    expected_by_phase = {
        "rest": _nonnegative_int(item.get("rest_state_expected"), "rest expected"),
        "single_joint_sweep": _nonnegative_int(
            item.get("single_state_expected"), "single expected"
        ),
        "multi_joint_sobol": _nonnegative_int(
            item.get("sobol_state_expected"), "sobol expected"
        ),
    }
    rows_by_phase = {phase: [] for phase in expected_by_phase}
    for state in states:
        _require_identity(
            item,
            state,
            ("protocol_id", "asset_id", "selection_index", "order"),
            f"Table 4 state {asset_id}",
        )
        phase = state.get("phase")
        if phase not in rows_by_phase:
            raise ValueError(f"unknown Table 4 state phase for {asset_id}: {phase!r}")
        _nonnegative_int(
            state.get("non_adjacent_illegal_penetration_count"),
            f"illegal penetration count for {asset_id}",
        )
        rows_by_phase[str(phase)].append(state)
    executed_by_phase = {
        phase: len(phase_rows) for phase, phase_rows in rows_by_phase.items()
    }
    free_by_phase = {
        phase: sum(
            row["non_adjacent_illegal_penetration_count"] == 0
            for row in phase_rows
        )
        for phase, phase_rows in rows_by_phase.items()
    }
    movable = _nonnegative_int(item.get("movable_dof_count"), "movable DoF count")
    range_evaluable = _nonnegative_int(
        item.get("range_evaluable_dof_count"), "range-evaluable DoF count"
    )
    rest_pass = bool(
        executed_by_phase["rest"] == expected_by_phase["rest"]
        and free_by_phase["rest"] == expected_by_phase["rest"]
    )
    single_pass = bool(
        executed_by_phase["single_joint_sweep"]
        == expected_by_phase["single_joint_sweep"]
        and free_by_phase["single_joint_sweep"]
        == expected_by_phase["single_joint_sweep"]
    )
    sobol_pass = bool(
        movable > 0
        and range_evaluable == movable
        and executed_by_phase["multi_joint_sobol"]
        == expected_by_phase["multi_joint_sobol"]
        and free_by_phase["multi_joint_sobol"]
        == expected_by_phase["multi_joint_sobol"]
    )
    measurement_complete = bool(
        range_evaluable == movable
        and sum(executed_by_phase.values()) == sum(expected_by_phase.values())
    )
    strict_pass = bool(
        measurement_complete and rest_pass and single_pass and sobol_pass
    )
    recorded_fields = {
        "rest_state_executed": executed_by_phase["rest"],
        "rest_non_adjacent_free": free_by_phase["rest"],
        "rest_non_adjacent_cf": rest_pass,
        "single_state_executed": executed_by_phase["single_joint_sweep"],
        "single_non_adjacent_free": free_by_phase["single_joint_sweep"],
        "single_joint_sweep_cf": single_pass,
        "sobol_state_executed": executed_by_phase["multi_joint_sobol"],
        "sobol_non_adjacent_free": free_by_phase["multi_joint_sobol"],
        "multi_joint_sobol_cf": sobol_pass,
        "measurement_complete": measurement_complete,
        "strict_collision_pass": strict_pass,
    }
    for field, expected in recorded_fields.items():
        if asset.get(field) != expected:
            raise ValueError(f"Table 4 asset {field} mismatch: {asset_id}")
    for phase, field in (
        ("rest", "rest_state_expected"),
        ("single_joint_sweep", "single_state_expected"),
        ("multi_joint_sobol", "sobol_state_expected"),
    ):
        if asset.get(field) != expected_by_phase[phase]:
            raise ValueError(f"Table 4 asset {field} mismatch: {asset_id}")
    return {
        "asset_id": asset_id,
        "selection_index": item["selection_index"],
        "strict_collision_pass": strict_pass,
        "measurement_complete": measurement_complete,
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
) -> dict[str, Any]:
    manifest_path = manifest_path.resolve(strict=True)
    asset_records_path = asset_records_path.resolve(strict=True)
    state_records_path = state_records_path.resolve(strict=True)
    if formal:
        expected_files = (
            (manifest_path, EXPECTED_TABLE4_MANIFEST_SHA256),
            (asset_records_path, EXPECTED_TABLE4_ASSET_RECORDS_SHA256),
            (state_records_path, EXPECTED_TABLE4_STATE_RECORDS_SHA256),
        )
        for path, expected_hash in expected_files:
            if sha256_file(path) != expected_hash:
                raise RuntimeError(f"formal Table 4 artifact SHA256 mismatch: {path}")
    manifest = _load_json_object(manifest_path, "Table 4 manifest")
    assets_raw = json.loads(asset_records_path.read_text(encoding="utf-8"))
    if not isinstance(assets_raw, list):
        raise ValueError("Table 4 asset records root must be a list")
    states_raw = load_jsonl(state_records_path)
    items = manifest.get("items")
    cohort_records = cohort.get("records")
    if not isinstance(items, list) or not isinstance(cohort_records, list):
        raise ValueError("Table 4 items or cohort records are unavailable")
    if not formal:
        prefix_size = len(cohort_records)
        if len(items) < prefix_size or len(assets_raw) < prefix_size:
            raise ValueError("Table 4 evidence is shorter than the smoke cohort")
        items = items[:prefix_size]
        assets_raw = assets_raw[:prefix_size]
        states_raw = [
            row
            for row in states_raw
            if isinstance(row.get("selection_index"), int)
            and not isinstance(row.get("selection_index"), bool)
            and 0 <= row["selection_index"] < prefix_size
        ]
    if len(items) != len(cohort_records) or len(assets_raw) != len(cohort_records):
        raise ValueError("Table 4 evidence denominator mismatch")
    if formal and manifest.get("sample_size") != len(cohort_records):
        raise ValueError("Table 4 manifest sample_size mismatch")
    if formal and manifest.get("protocol_id") != EXPECTED_TABLE4_PROTOCOL_ID:
        raise RuntimeError("formal Table 4 protocol identity mismatch")

    states_by_index: dict[int, list[dict[str, Any]]] = {
        index: [] for index in range(len(cohort_records))
    }
    previous_index = -1
    for state in states_raw:
        index = _nonnegative_int(state.get("selection_index"), "state selection_index")
        if index not in states_by_index:
            raise ValueError(f"unknown Table 4 state selection_index: {index}")
        if index < previous_index:
            raise ValueError("Table 4 state records are not in cohort order")
        previous_index = index
        states_by_index[index].append(state)

    records: list[dict[str, Any]] = []
    for index, (source, item, asset) in enumerate(
        zip(cohort_records, items, assets_raw, strict=True)
    ):
        if not isinstance(item, Mapping) or not isinstance(asset, Mapping):
            raise ValueError(f"Table 4 item or asset row is invalid at index {index}")
        _require_identity(
            source,
            item,
            (
                "asset_id",
                "selection_index",
                "model_urdf_sha256",
                "package_content_manifest_sha256",
            ),
            f"Table 4 manifest item {index}",
        )
        _require_identity(
            item,
            asset,
            (
                "protocol_id",
                "asset_id",
                "selection_index",
                "order",
                "model_urdf_sha256",
                "package_content_manifest_sha256",
                "movable_dof_count",
                "range_evaluable_dof_count",
            ),
            f"Table 4 asset record {index}",
        )
        records.append(
            _reaggregate_table4_asset(item, asset, states_by_index[index])
        )
    strict_passed = sum(record["strict_collision_pass"] for record in records)
    if formal and strict_passed != EXPECTED_FORMAL_STRICT_PASSED:
        raise RuntimeError("formal Table 4 strict-pass aggregate mismatch")
    return {
        "protocol_id": manifest.get("protocol_id"),
        "manifest_path": str(manifest_path),
        "manifest_file_sha256": sha256_file(manifest_path),
        "asset_records_path": str(asset_records_path),
        "asset_records_file_sha256": sha256_file(asset_records_path),
        "state_records_path": str(state_records_path),
        "state_records_file_sha256": sha256_file(state_records_path),
        "strict_passed": strict_passed,
        "denominator": len(records),
        "state_record_count": len(states_raw),
        "records": records,
    }


def build_s1_asset_record(
    cohort_record: Mapping[str, Any],
    release_evidence: Mapping[str, Any],
    table4_record: Mapping[str, Any],
) -> dict[str, Any]:
    asset_id = str(cohort_record["asset_id"])
    selection_index = cohort_record["selection_index"]
    if release_evidence.get("asset_id") != asset_id:
        raise ValueError(f"release evidence identity mismatch: {asset_id}")
    if (
        table4_record.get("asset_id") != asset_id
        or table4_record.get("selection_index") != selection_index
    ):
        raise ValueError(f"Table 4 evidence identity mismatch: {asset_id}")
    receipt = release_evidence.get("receipt")
    rebuild = release_evidence.get("rebuild")
    allowance = release_evidence.get("allowance")
    resource_closure = release_evidence.get("resource_closure")
    if not all(
        isinstance(value, Mapping)
        for value in (receipt, rebuild, allowance, resource_closure)
    ):
        raise ValueError(f"incomplete release evidence: {asset_id}")
    eligible_pairs = _nonnegative_int(
        allowance.get("eligible_nonadjacent_pair_count"),
        f"eligible pair count for {asset_id}",
    )
    registered_pairs = _nonnegative_int(
        allowance.get("registered_excluded_pair_count"),
        f"registered allowance count for {asset_id}",
    )
    if registered_pairs:
        raise ValueError(
            "non-empty registered allowance requires pair-specific replay: "
            f"{asset_id}"
        )
    receipt_bound = bool(receipt.get("receipt_bound_asset"))
    rebuild_eligible = bool(rebuild.get("eligible_asset"))
    strict_pass = bool(table4_record.get("strict_collision_pass"))
    return {
        "schema_version": "s1-articraft10k-asset/v1",
        "asset_id": asset_id,
        "selection_index": selection_index,
        "package": cohort_record["package"],
        "model_urdf_sha256": cohort_record["model_urdf_sha256"],
        "package_content_manifest_sha256": cohort_record[
            "package_content_manifest_sha256"
        ],
        "source_cohort_record_sha256": canonical_sha256(dict(cohort_record)),
        "resource_closure_sha256": resource_closure.get("sha256"),
        "release_receipt_bound": receipt_bound,
        "release_receipt_replay_pass": False,
        "receipt_replay_status": (
            "VALID_RECEIPT_NOT_REPLAYED" if receipt_bound else "NO_VALID_RECEIPT"
        ),
        "deterministic_rebuild_eligible": rebuild_eligible,
        "deterministic_rebuild_match": False,
        "deterministic_rebuild_status": (
            "ELIGIBLE_NOT_RUN" if rebuild_eligible else "N/E"
        ),
        "eligible_non_adjacent_pair_count": eligible_pairs,
        "registered_method_allowance_pair_count": registered_pairs,
        "strict_collision_pass_no_method_allowance": strict_pass,
        "strict_collision_pass_registered_allowance": strict_pass,
        "table4_measurement_complete": bool(
            table4_record.get("measurement_complete")
        ),
        "table4_state_record_count": table4_record["state_record_count"],
        "table4_state_records_sha256": table4_record["state_records_sha256"],
        "table4_asset_record_sha256": table4_record[
            "table4_asset_record_sha256"
        ],
        "release_evidence": dict(release_evidence),
        "terminal": True,
        "status": "completed",
    }


def _rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def aggregate_s1(records: list[Mapping[str, Any]]) -> dict[str, Any]:
    denominator = len(records)
    receipt_bound = sum(bool(row["release_receipt_bound"]) for row in records)
    receipt_replayed = sum(
        bool(row["release_receipt_replay_pass"]) for row in records
    )
    rebuild_eligible = sum(
        bool(row["deterministic_rebuild_eligible"]) for row in records
    )
    rebuild_matched = sum(
        bool(row["deterministic_rebuild_match"]) for row in records
    )
    registered_pairs = sum(
        _nonnegative_int(
            row["registered_method_allowance_pair_count"],
            "registered allowance pair count",
        )
        for row in records
    )
    eligible_pairs = sum(
        _nonnegative_int(
            row["eligible_non_adjacent_pair_count"], "eligible pair count"
        )
        for row in records
    )
    no_allowance_passed = sum(
        bool(row["strict_collision_pass_no_method_allowance"])
        for row in records
    )
    registered_passed = sum(
        bool(row["strict_collision_pass_registered_allowance"])
        for row in records
    )
    rebuild_summary: dict[str, Any] = {
        "status": "N/E" if rebuild_eligible == 0 else "NOT_RUN",
        "passed": None if rebuild_eligible == 0 else rebuild_matched,
        "denominator": rebuild_eligible,
        "rate": None if rebuild_eligible == 0 else _rate(rebuild_matched, rebuild_eligible),
        "eligible_assets": rebuild_eligible,
        "asset_denominator": denominator,
    }
    if rebuild_eligible == 0:
        rebuild_summary["reason"] = (
            "no frozen public build recipe with complete inputs in the release evidence"
        )
    return {
        "receipt_bound_assets": {
            "passed": receipt_bound,
            "denominator": denominator,
            "rate": _rate(receipt_bound, denominator),
        },
        "receipt_replay_pass": {
            "passed": receipt_replayed,
            "denominator": denominator,
            "rate": _rate(receipt_replayed, denominator),
        },
        "deterministic_rebuild_match": rebuild_summary,
        "allowance_density": {
            "registered_pairs": registered_pairs,
            "eligible_pairs": eligible_pairs,
            "rate": _rate(registered_pairs, eligible_pairs),
        },
        "strict_pass_no_method_allowance": {
            "passed": no_allowance_passed,
            "denominator": denominator,
            "rate": _rate(no_allowance_passed, denominator),
        },
        "registered_allowance_gain_pp": {
            "value": (
                (registered_passed - no_allowance_passed) * 100.0 / denominator
                if denominator
                else None
            ),
            "registered_passed": registered_passed,
            "no_allowance_passed": no_allowance_passed,
            "denominator": denominator,
        },
    }


def official_source_manifest_sha256(records: list[Mapping[str, Any]]) -> str:
    bindings: list[dict[str, Any]] = []
    for row in records:
        release_evidence = row.get("release_evidence")
        rebuild = (
            release_evidence.get("rebuild")
            if isinstance(release_evidence, Mapping)
            else None
        )
        official = (
            rebuild.get("official_model_py")
            if isinstance(rebuild, Mapping)
            else None
        )
        if not isinstance(official, Mapping):
            raise ValueError(f"official source evidence missing: {row.get('asset_id')}")
        bindings.append(
            {
                "asset_id": row.get("asset_id"),
                "selection_index": row.get("selection_index"),
                "record_json_sha256": official.get("record_json_sha256"),
                "model_py_path": official.get("path"),
                "model_py_exists": official.get("exists"),
                "model_py_sha256": official.get("observed_sha256"),
            }
        )
    return canonical_sha256(bindings)


def _format_rate(metric: Mapping[str, Any]) -> str:
    passed = metric["passed"]
    denominator = metric["denominator"]
    rate = metric["rate"]
    return f"{passed} / {denominator} ({float(rate) * 100.0:.2f}%)"


def render_summary_markdown(summary: Mapping[str, Any]) -> str:
    metrics = summary["metrics"]
    rebuild = metrics["deterministic_rebuild_match"]
    rebuild_text = (
        "N/E"
        if rebuild["status"] == "N/E"
        else _format_rate(rebuild)
    )
    allowance = metrics["allowance_density"]
    allowance_rate = allowance["rate"]
    allowance_text = (
        "N/E"
        if allowance_rate is None
        else f"{allowance['registered_pairs']} / {allowance['eligible_pairs']} "
        f"({float(allowance_rate) * 100.0:.2f}%)"
    )
    gain = metrics["registered_allowance_gain_pp"]["value"]
    lines = [
        "# Supplementary Table S1 - Articraft-10K",
        "",
        f"Protocol: `{summary['protocol_id']}`",
        "",
        "| Metric | Result |",
        "|---|---:|",
        f"| Receipt-bound Assets | {_format_rate(metrics['receipt_bound_assets'])} |",
        f"| Receipt Replay Pass | {_format_rate(metrics['receipt_replay_pass'])} |",
        f"| Deterministic Rebuild Match | {rebuild_text} |",
        f"| Allowance Density | {allowance_text} |",
        "| Strict Pass (No Method-specific Allowance) | "
        f"{_format_rate(metrics['strict_pass_no_method_allowance'])} |",
        f"| Registered-allowance Gain | {float(gain):.2f} pp |",
        "",
    ]
    return "\n".join(lines)


def _artifact_binding(path: Path) -> dict[str, Any]:
    return {"bytes": path.stat().st_size, "sha256": sha256_file(path)}


def _runtime_identity() -> dict[str, Any]:
    return {
        "python_executable": sys.executable,
        "python_version": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "platform": platform.platform(),
        "gpu_required": False,
    }


def run_evaluation(
    *,
    output: Path,
    cohort_manifest: Path,
    dataset_root: Path,
    category_records_root: Path,
    table4_manifest: Path,
    table4_asset_records: Path,
    table4_state_records: Path,
    protocol_document: Path,
    formal: bool,
    workers: int,
    limit: int | None = None,
) -> dict[str, Any]:
    if isinstance(workers, bool) or not isinstance(workers, int) or workers < 1:
        raise ValueError("workers must be a positive integer")
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    protocol_document = Path(protocol_document).resolve(strict=True)
    category_records_root = Path(category_records_root).resolve(strict=True)
    if formal and sha256_file(STATIC_ATOMS_PATH) != EXPECTED_STATIC_ATOMS_SHA256:
        raise RuntimeError("formal static-atoms SHA256 mismatch")
    started_at = utc_now()
    cohort = load_cohort(
        Path(cohort_manifest), Path(dataset_root), formal=formal, limit=limit
    )
    table4 = load_table4_evidence(
        Path(table4_manifest),
        Path(table4_asset_records),
        Path(table4_state_records),
        cohort,
        formal=formal,
    )

    release_evidence_by_index: list[dict[str, Any] | None] = [
        None for _ in cohort["records"]
    ]
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(
                audit_release_evidence, record, category_records_root
            ): index
            for index, record in enumerate(cohort["records"])
        }
        for future in as_completed(futures):
            index = futures[future]
            release_evidence_by_index[index] = future.result()
    if any(record is None for record in release_evidence_by_index):
        raise RuntimeError("release evidence did not cover the entire cohort")
    records = [
        build_s1_asset_record(source, release, table4_record)
        for source, release, table4_record in zip(
            cohort["records"],
            release_evidence_by_index,
            table4["records"],
            strict=True,
        )
        if release is not None
    ]
    metrics = aggregate_s1(records)
    official_source_manifest = official_source_manifest_sha256(records)
    if formal:
        if len(records) != N_EVAL:
            raise RuntimeError("formal S1 asset denominator mismatch")
        if metrics["receipt_bound_assets"]["passed"] != 0:
            raise RuntimeError("formal release unexpectedly contains valid receipts")
        if metrics["deterministic_rebuild_match"]["eligible_assets"] != 0:
            raise RuntimeError("formal release unexpectedly contains a rebuild recipe")
        allowance = metrics["allowance_density"]
        if (
            allowance["registered_pairs"] != 0
            or allowance["eligible_pairs"] != EXPECTED_FORMAL_ELIGIBLE_PAIRS
        ):
            raise RuntimeError("formal allowance aggregate mismatch")
        if (
            metrics["strict_pass_no_method_allowance"]["passed"]
            != EXPECTED_FORMAL_STRICT_PASSED
        ):
            raise RuntimeError("formal strict-pass aggregate mismatch")
        if (
            official_source_manifest
            != EXPECTED_FORMAL_OFFICIAL_SOURCE_MANIFEST_SHA256
        ):
            raise RuntimeError("formal official-source manifest mismatch")

    completed_at = utc_now()
    frozen_config = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "classification": "FORMAL" if formal else "SMOKE",
        "dataset": DATASET,
        "cohort": {
            "n_eval": len(records),
            "selection_policy": (
                "all .records[].package entries from the frozen Table 2 manifest "
                "in existing order; no resampling, replacement, or result filtering"
            ),
            "source_manifest": cohort["manifest_path"],
            "source_manifest_file_sha256": cohort["manifest_file_sha256"],
            "source_manifest_content_sha256": cohort[
                "manifest_content_sha256"
            ],
            "ordered_asset_ids_sha256": cohort["ordered_asset_ids_sha256"],
        },
        "release_sources": {
            "category_records_root": str(category_records_root),
            "official_source_manifest_sha256": official_source_manifest,
        },
        "table4_source": {
            key: value
            for key, value in table4.items()
            if key.endswith("sha256")
            or key.endswith("_path")
            or key in {"protocol_id", "state_record_count"}
        },
        "metric_operationalization": {
            "receipt": (
                "release package receipt must bind URDF, recursive resource closure, "
                "protocol, runner, pair policy, thresholds, and conclusion; compile_report "
                "is explicitly excluded"
            ),
            "receipt_replay": "missing or non-replayable receipt fails closed",
            "rebuild": (
                "eligible only with a frozen recipe, runner identity, complete bound inputs, "
                "and declared canonical output fingerprint"
            ),
            "allowance": (
                "compile_report.overlap_allowances plus valid release registry files; "
                "eligible denominator is source-link pairs excluding direct parent-child"
            ),
            "strict_pass": (
                "independent reaggregation of frozen unified PyBullet Table 4 rest, "
                "single-joint, and Sobol raw state records; penetration threshold 1e-6 m; "
                "direct parent-child excluded; no method-specific allowance"
            ),
            "registered_allowance": (
                "empty registry is exactly equivalent to no allowance; a non-empty registry "
                "requires pair-specific replay and is rejected by this frozen run"
            ),
        },
        "execution": {"workers": workers, "runtime": _runtime_identity()},
        "code_identity": {
            "runner_path": str(SCRIPT),
            "runner_sha256": sha256_file(SCRIPT),
            "verifier_path": str(VERIFIER_PATH),
            "verifier_sha256": sha256_file(VERIFIER_PATH),
            "static_atoms_path": str(STATIC_ATOMS_PATH),
            "static_atoms_sha256": sha256_file(STATIC_ATOMS_PATH),
        },
        "protocol_document": {
            "path": str(protocol_document),
            "sha256_at_freeze": sha256_file(protocol_document),
        },
        "frozen_at": started_at,
    }
    summary = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "classification": "FORMAL" if formal else "SMOKE",
        "dataset": DATASET,
        "status": "completed",
        "n_eval": len(records),
        "started_at": started_at,
        "completed_at": completed_at,
        "metrics": metrics,
    }
    atomic_write_text(
        output / "protocol_snapshot.md",
        protocol_document.read_text(encoding="utf-8"),
    )
    atomic_write_json(output / "frozen_config.json", frozen_config)
    atomic_write_text(
        output / "asset_records.jsonl",
        "".join(
            json.dumps(
                record,
                sort_keys=True,
                ensure_ascii=True,
                allow_nan=False,
            )
            + "\n"
            for record in records
        ),
    )
    atomic_write_json(output / "summary.json", summary)
    atomic_write_text(output / "summary.md", render_summary_markdown(summary))
    artifact_names = (
        "protocol_snapshot.md",
        "frozen_config.json",
        "asset_records.jsonl",
        "summary.json",
        "summary.md",
    )
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "classification": summary["classification"],
        "dataset": DATASET,
        "created_at": completed_at,
        "n_eval": len(records),
        "cohort": frozen_config["cohort"],
        "release_sources": frozen_config["release_sources"],
        "table4_source": frozen_config["table4_source"],
        "code_identity": frozen_config["code_identity"],
        "artifacts": {
            name: _artifact_binding(output / name) for name in artifact_names
        },
    }
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    atomic_write_json(output / "manifest.json", manifest)
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), required=True)
    parser.add_argument("--n", type=int)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--output", type=Path)
    return parser.parse_args(argv)


def requested_n(args: argparse.Namespace) -> int:
    if args.n is not None:
        return int(args.n)
    return N_EVAL if args.mode == "formal" else 5


def validate_args(args: argparse.Namespace) -> None:
    n = requested_n(args)
    if isinstance(args.workers, bool) or not isinstance(args.workers, int) or args.workers < 1:
        raise ValueError("workers must be a positive integer")
    if args.mode == "formal":
        if n != N_EVAL:
            raise ValueError("formal mode requires n=800")
        if args.workers != DEFAULT_WORKERS:
            raise ValueError("formal mode requires workers=4")
    elif not 1 <= n <= N_EVAL:
        raise ValueError("smoke n must be in [1, 800]")


def default_output(mode: str, n: int) -> Path:
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    if mode == "formal":
        name = (
            "s1_articraft10k_table2cohort_n800_seed20260813_" + timestamp
        )
    else:
        name = f"s1_articraft10k_smoke_n{n}_{timestamp}"
    return DEFAULT_OUTPUT_PARENT / name


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        validate_args(args)
        n = requested_n(args)
        output = args.output if args.output is not None else default_output(args.mode, n)
        output = Path(output).resolve()
        run_evaluation(
            output=output,
            cohort_manifest=SOURCE_MANIFEST,
            dataset_root=DATASET_ROOT,
            category_records_root=CATEGORY_RECORDS_ROOT,
            table4_manifest=TABLE4_MANIFEST,
            table4_asset_records=TABLE4_ASSET_RECORDS,
            table4_state_records=TABLE4_STATE_RECORDS,
            protocol_document=PROTOCOL_DOCUMENT,
            formal=args.mode == "formal",
            workers=args.workers,
            limit=None if args.mode == "formal" else n,
        )
        command = [sys.executable, str(VERIFIER_PATH), "--run", str(output)]
        if args.mode == "formal":
            command.append("--formal")
        result = subprocess.run(command, check=False)
        if result.returncode != 0:
            raise RuntimeError(f"independent verifier exited {result.returncode}")
    except Exception as exc:  # noqa: BLE001
        print(f"S1 evaluation failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
