#!/usr/bin/env python3
"""Seal the external evidence bundle for the PV-A Table 4 v2 publication."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
from typing import Any, Mapping


SCRIPT = Path(__file__).resolve()
EXP_ROOT = SCRIPT.parents[1]
if str(SCRIPT.parent) not in sys.path:
    sys.path.insert(0, str(SCRIPT.parent))
import check_pva_table4_mimic_aware_full_release as checker
SCHEMA = "pva_table4_mimic_aware_publication_bundle_receipt_v2"
RUN_SCHEMA = "pva_table4_mimic_aware_full_release_run_v2"
RUN_RECEIPT_SCHEMA = "pva_table4_mimic_aware_full_release_receipt_v1"
COPY_RECEIPT_SCHEMA = "pva_table4_mimic_aware_atomic_copy_receipt_v1"
CHECK_SCHEMA = "pva_table4_mimic_aware_automation_check_v1"
PARITY_SCHEMA = "pva_table4_v1_v2_prefix_parity_audit_v2"
COHORT_SCHEMA = "ours500_pva_table4_label_matched_cohort_audit_v2"
FAILURE_SCHEMA = "pva_table4_v1_failure_diagnosis_v1"
REPLAY_SCHEMA = "pva_table4_illegal_link_pair_diagnostic_replay_v2"
MIRROR_MANIFEST_SCHEMA = "pva_local_archive_stage_manifest_v1"
MIRROR_ARTIFACT_SCHEMA = "pva_local_archive_stage_artifact_manifest_v1"
MIRROR_SUMMARY_SCHEMA = "pva_local_archive_stage_summary_v1"
SAMPLING_PROTOCOL = "mimic_aware_independent_sampling_v2"
PROTOCOL_ID = "urdf_sim_ready_table4_pva_full_release_v2"
PUBLISHED_SOURCE_FILES = (
    "manifest.json", "protocol_snapshot.md", "records.jsonl", "asset_records.jsonl",
    "state_records.jsonl", "summary.json", "summary.md", "checkpoint.json",
    "results.sqlite3", "artifact_manifest.json", "full_release_receipt.json", "progress.json",
)
INTERNAL_COPY_RECEIPT = "publication_copy_receipt.json"
IMPLEMENTATIONS = {
    "core": EXP_ROOT / "scripts" / "run_urdf_table4_partnet_mobility.py",
    "runner": EXP_ROOT / "scripts" / "run_table4_full_release.py",
    "wrapper": EXP_ROOT / "scripts" / "run_pva_table4_mimic_aware_full_release.py",
    "checker": EXP_ROOT / "scripts" / "check_pva_table4_mimic_aware_full_release.py",
    "parity": EXP_ROOT / "scripts" / "audit_pva_table4_v1_v2_prefix_parity.py",
    "publisher": EXP_ROOT / "scripts" / "publish_pva_table4_mimic_aware_full_release.py",
    "sealer": SCRIPT,
    "cohort_auditor": EXP_ROOT / "scripts" / "audit_ours500_pva_table4_cohort.py",
    "failure_diagnoser": EXP_ROOT / "scripts" / "diagnose_pva_table4_v1_failures.py",
    "illegal_link_replayer": EXP_ROOT / "scripts" / "replay_pva_table4_illegal_link_pairs.py",
}
_LOADED_JSON_BINDINGS: dict[str, tuple[int, str]] = {}


class SealError(ValueError):
    pass


def canonical_text(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def self_hash(value: Mapping[str, Any], field: str) -> str:
    payload = dict(value); payload.pop(field, None)
    return hashlib.sha256(canonical_text(payload).encode("utf-8")).hexdigest()


def regular_file(path: Path, label: str) -> Path:
    path = Path(path).absolute()
    try:
        mode = path.lstat().st_mode
    except FileNotFoundError as error:
        raise SealError(f"missing {label}: {path}") from error
    if stat.S_ISLNK(mode) or not stat.S_ISREG(mode):
        raise SealError(f"{label} is not a regular non-symlink file: {path}")
    if path.resolve(strict=True) != path:
        raise SealError(f"{label} has a symlinked or non-canonical path component: {path}")
    return path


def regular_dir(path: Path, label: str) -> Path:
    path = Path(path).absolute()
    try:
        mode = path.lstat().st_mode
    except FileNotFoundError as error:
        raise SealError(f"missing {label}: {path}") from error
    if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
        raise SealError(f"{label} is not a non-symlink directory: {path}")
    if path.resolve(strict=True) != path:
        raise SealError(f"{label} has a symlinked or non-canonical path component: {path}")
    return path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    path = regular_file(path, "hashed input")
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    with os.fdopen(descriptor, "rb") as stream:
        opened = os.fstat(stream.fileno())
        if not stat.S_ISREG(opened.st_mode):
            raise SealError(f"hashed input changed type: {path}")
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
        closed = os.fstat(stream.fileno())
    if (opened.st_dev, opened.st_ino, opened.st_size, opened.st_mtime_ns) != (closed.st_dev, closed.st_ino, closed.st_size, closed.st_mtime_ns):
        raise SealError(f"hashed input changed while reading: {path}")
    return digest.hexdigest()


def read_file_bytes(path: Path, label: str) -> bytes:
    path = regular_file(path, label); descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    with os.fdopen(descriptor, "rb") as stream:
        opened = os.fstat(stream.fileno()); data = stream.read(); closed = os.fstat(stream.fileno())
    if not stat.S_ISREG(opened.st_mode) or (opened.st_dev, opened.st_ino, opened.st_size, opened.st_mtime_ns) != (closed.st_dev, closed.st_ino, closed.st_size, closed.st_mtime_ns):
        raise SealError(f"{label} changed while reading: {path}")
    return data


def file_binding(path: Path) -> dict[str, Any]:
    path = regular_file(path, "bound input")
    digest = hashlib.sha256(); descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    with os.fdopen(descriptor, "rb") as stream:
        opened = os.fstat(stream.fileno())
        if not stat.S_ISREG(opened.st_mode): raise SealError(f"bound input changed type: {path}")
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""): digest.update(block)
        closed = os.fstat(stream.fileno())
    if (opened.st_dev, opened.st_ino, opened.st_size, opened.st_mtime_ns) != (closed.st_dev, closed.st_ino, closed.st_size, closed.st_mtime_ns):
        raise SealError(f"bound input changed while hashing: {path}")
    value = {"path": str(path), "bytes": opened.st_size, "sha256": digest.hexdigest()}
    loaded = _LOADED_JSON_BINDINGS.get(str(path))
    if loaded is not None and loaded != (value["bytes"], value["sha256"]):
        raise SealError(f"JSON input changed after semantic validation: {path}")
    return value


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        path = regular_file(path, label); data = read_file_bytes(path, label)
        observed = (len(data), hashlib.sha256(data).hexdigest())
        prior = _LOADED_JSON_BINDINGS.setdefault(str(path), observed)
        if prior != observed:
            raise SealError(f"{label} changed between semantic reads")
        value = json.loads(data.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise SealError(f"invalid JSON: {label}") from error
    if not isinstance(value, dict):
        raise SealError(f"{label} must be a JSON object")
    return value


def require_self_hash(value: Mapping[str, Any], field: str, label: str) -> None:
    if value.get(field) != self_hash(value, field):
        raise SealError(f"{label} self-hash mismatch")


def safe_child(root: Path, relative_text: Any, label: str) -> Path:
    if not isinstance(relative_text, str):
        raise SealError(f"{label} path is not a string")
    relative = Path(relative_text)
    if relative.is_absolute() or relative == Path(".") or ".." in relative.parts:
        raise SealError(f"unsafe relative path in {label}")
    current = root
    for part in relative.parts:
        current = current / part
        regular_file(current, label) if current == root / relative else regular_dir(current, label)
    return regular_file(root / relative, label)


def verify_published(published: Path) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    published = regular_dir(published, "published run")
    manifest = load_json(published / "manifest.json", "run manifest")
    receipt = load_json(published / "full_release_receipt.json", "run receipt")
    if manifest.get("schema_version") != RUN_SCHEMA or receipt.get("schema_version") != RUN_RECEIPT_SCHEMA:
        raise SealError("run manifest/receipt schema mismatch")
    require_self_hash(manifest, "manifest_content_sha256", "run manifest")
    require_self_hash(receipt, "receipt_content_sha256", "run receipt")
    if manifest.get("classification") != "FORMAL_FULL_RELEASE" or manifest.get("limit") is not None:
        raise SealError("run is not an unlimited FORMAL_FULL_RELEASE")
    if receipt.get("classification") != "FORMAL_FULL_RELEASE":
        raise SealError("run receipt is not FORMAL_FULL_RELEASE")
    if type(manifest.get("N_eval")) is not int or manifest["N_eval"] <= 0 or type(manifest.get("J_eval")) is not int or manifest["J_eval"] < 0 or type(manifest.get("category_count")) is not int or manifest["category_count"] <= 0:
        raise SealError("run N/J/category denominators are invalid")
    for key in ("N_eval", "J_eval", "category_count", "sampling_protocol", "protocol_id"):
        if receipt.get(key) != manifest.get(key):
            raise SealError(f"run receipt/manifest mismatch: {key}")
    if manifest.get("sampling_protocol") != SAMPLING_PROTOCOL or manifest.get("protocol_id") != PROTOCOL_ID:
        raise SealError("run protocol mismatch")
    files = {}
    for name in PUBLISHED_SOURCE_FILES:
        files[name] = file_binding(published / name)
    expected_receipt_hashes = {
        "manifest_sha256": "manifest.json", "records_sha256": "records.jsonl",
        "state_records_sha256": "state_records.jsonl", "summary_sha256": "summary.json",
        "artifact_manifest_sha256": "artifact_manifest.json", "result_database_sha256": "results.sqlite3",
    }
    for field, name in expected_receipt_hashes.items():
        if receipt.get(field) != files[name]["sha256"]:
            raise SealError(f"run receipt binding mismatch: {field}")
    return manifest, receipt, files


def verify_copy_receipt(path: Path, published: Path, published_files: Mapping[str, Any], implementation: Mapping[str, Any]) -> dict[str, Any]:
    external = load_json(path, "external copy receipt")
    internal_path = published / INTERNAL_COPY_RECEIPT
    internal = load_json(internal_path, "internal copy receipt")
    if read_file_bytes(path, "external copy receipt") != read_file_bytes(internal_path, "internal copy receipt"):
        raise SealError("internal/external copy receipts differ")
    if external.get("schema_version") != COPY_RECEIPT_SCHEMA:
        raise SealError("copy receipt schema mismatch")
    require_self_hash(external, "copy_receipt_content_sha256", "copy receipt")
    if Path(str(external.get("destination", ""))).absolute() != published.absolute():
        raise SealError("copy receipt destination mismatch")
    rows = external.get("files")
    if not isinstance(rows, list) or len(rows) != len(PUBLISHED_SOURCE_FILES):
        raise SealError("copy receipt must bind exactly twelve source files")
    observed = {}
    for row in rows:
        if not isinstance(row, Mapping) or set(row) != {"path", "bytes", "sha256"}:
            raise SealError("invalid copy receipt file row")
        name = row.get("path")
        if not isinstance(name, str) or name in observed:
            raise SealError("duplicate/invalid copy receipt path")
        observed[name] = {"bytes": row.get("bytes"), "sha256": row.get("sha256")}
    expected = {name: {"bytes": value["bytes"], "sha256": value["sha256"]} for name, value in published_files.items()}
    if observed != expected:
        raise SealError("copy receipt twelve-file binding mismatch")
    if external.get("publisher_source_sha256") != implementation["publisher"]["sha256"] or external.get("checker_source_sha256") != implementation["checker"]["sha256"]:
        raise SealError("copy receipt implementation binding mismatch")
    return external


def verify_checker(path: Path, published: Path, manifest: Mapping[str, Any], receipt: Mapping[str, Any]) -> dict[str, Any]:
    report = load_json(path, "automation checker report")
    if report.get("schema_version") != CHECK_SCHEMA or report.get("all_pass") is not True:
        raise SealError("automation checker did not pass")
    if (report.get("N_eval"), report.get("J_eval")) != (manifest.get("N_eval"), manifest.get("J_eval")):
        raise SealError("checker N/J mismatch")
    if Path(str(report.get("output", ""))).absolute() != published.absolute():
        raise SealError("checker output binding mismatch")
    if report.get("status") != receipt.get("status"):
        raise SealError("checker/run status mismatch")
    try:
        observed = checker.check(published)
    except BaseException as error:
        raise SealError(f"independent checker rerun failed: {error}") from error
    if canonical_text(observed) != canonical_text(report):
        raise SealError("automation checker report differs from independent rerun")
    return report


def verify_parity(path: Path, run_receipt: Mapping[str, Any], published_files: Mapping[str, Any], implementation: Mapping[str, Any]) -> dict[str, Any]:
    report = load_json(path, "full parity report")
    if report.get("schema_version") != PARITY_SCHEMA or report.get("all_pass") is not True:
        raise SealError("full parity report did not pass")
    require_self_hash(report, "audit_content_sha256", "full parity report")
    status = report.get("publication_status")
    if not isinstance(status, Mapping) or status.get("final_publication_eligible") is not True or status.get("interim_non_durable_active_snapshot") is not False:
        raise SealError("parity report is not final-publication eligible")
    inputs = report.get("inputs")
    if not isinstance(inputs, Mapping) or inputs.get("v2_full_release_receipt_verified") is not True:
        raise SealError("parity report is not receipt-bound")
    if inputs.get("v2_full_release_receipt_content_sha256") != run_receipt.get("receipt_content_sha256"):
        raise SealError("parity/run receipt content binding mismatch")
    if inputs.get("v2_full_release_receipt_file_sha256") != published_files["full_release_receipt.json"]["sha256"]:
        raise SealError("parity/run receipt file binding mismatch")
    if inputs.get("v2_database_full_file_sha256") != published_files["results.sqlite3"]["sha256"]:
        raise SealError("parity/run database binding mismatch")
    parity_implementation = report.get("implementation")
    if not isinstance(parity_implementation, Mapping) or parity_implementation.get("script_sha256") != implementation["parity"]["sha256"]:
        raise SealError("parity implementation binding mismatch")
    return report


def verify_hashed_artifact(path: Path, schema: str, hash_field: str, label: str) -> dict[str, Any]:
    value = load_json(path, label)
    if value.get("schema_version") != schema:
        raise SealError(f"{label} schema mismatch")
    require_self_hash(value, hash_field, label)
    return value


def verify_cohort(path: Path, implementation: Mapping[str, Any], parity_inputs: Mapping[str, Any]) -> dict[str, Any]:
    value = verify_hashed_artifact(path, COHORT_SCHEMA, "audit_content_sha256", "cohort audit")
    validation = value.get("validation")
    required = {
        "ours_manifest_self_hash_verified", "ours_frozen_row_identities_verified",
        "ours_primary_urdf_payload_sha256_verified", "ours_primary_urdf_regular_non_symlink_verified",
        "ours_state_semantics_independently_replayed", "ours_load_child_worker_success_verified",
        "ours_reset_readback_tolerance_verified", "ours_formal_summary_recomputed",
        "ours_formal_verification_bound", "ours_result_closure_verified",
        "pva_execution_manifest_self_hash_verified", "pva_receipt_self_hash_verified",
        "pva_database_receipt_sha256_verified", "pva_database_opened_mode_ro",
        "pva_roster_row_sha256_verified", "pva_table4_input_identities_verified",
        "label_match_identity_closure_verified", "declared_protocol_fields_aligned",
        "category_partition_closure_verified",
    }
    if not isinstance(validation, Mapping) or any(validation.get(key) is not True for key in required):
        raise SealError("cohort validation gates are incomplete")
    if value.get("implementation", {}).get("script_sha256") != implementation["cohort_auditor"]["sha256"]:
        raise SealError("cohort implementation binding mismatch")
    pva = value.get("inputs", {}).get("current_pva_v1", {})
    if pva.get("full_release_receipt", {}).get("file_sha256") != parity_inputs.get("v1_full_release_receipt_file_sha256") or pva.get("full_release_receipt", {}).get("receipt_content_sha256") != parity_inputs.get("v1_full_release_receipt_content_sha256") or pva.get("results_database", {}).get("file_sha256") != parity_inputs.get("v1_database_full_file_sha256"):
        raise SealError("cohort/parity v1 evidence binding mismatch")
    return value


def verify_failure(path: Path, implementation: Mapping[str, Any], parity_inputs: Mapping[str, Any]) -> dict[str, Any]:
    value = verify_hashed_artifact(path, FAILURE_SCHEMA, "artifact_content_sha256", "failure diagnosis")
    strict = value.get("strict_summary"); inputs = value.get("inputs")
    if not isinstance(strict, Mapping) or type(strict.get("denominator")) is not int or strict["denominator"] <= 0 or strict.get("failed") != strict["denominator"] - strict.get("passed", -1):
        raise SealError("failure diagnosis strict-summary gate mismatch")
    if not isinstance(inputs, Mapping) or not isinstance(inputs.get("database_sha256"), str) or len(inputs["database_sha256"]) != 64:
        raise SealError("failure diagnosis source binding missing")
    if value.get("implementation", {}).get("script_sha256") != implementation["failure_diagnoser"]["sha256"]:
        raise SealError("failure diagnosis implementation binding mismatch")
    receipt_path = regular_file(Path(str(inputs.get("receipt", ""))), "failure source receipt")
    if inputs.get("database_sha256") != parity_inputs.get("v1_database_full_file_sha256") or sha256_file(receipt_path) != parity_inputs.get("v1_full_release_receipt_file_sha256"):
        raise SealError("failure/parity v1 evidence binding mismatch")
    return value


def verify_replay(path: Path, manifest: Mapping[str, Any], published_files: Mapping[str, Any], implementation: Mapping[str, Any], parity_inputs: Mapping[str, Any]) -> dict[str, Any]:
    value = verify_hashed_artifact(path, REPLAY_SCHEMA, "artifact_content_sha256", "illegal-link replay")
    checks = value.get("self_checks"); runtime = value.get("runtime_identity"); inputs = value.get("inputs")
    required_checks = {"all_selected_states_reproduced_illegal_contacts", "all_emitted_depths_exceed_threshold", "source_phase_sample_joint_checked_before_each_replay", "joint_value_hash_checked_before_each_replay", "source_record_blob_and_state_identity_hashed"}
    if not isinstance(checks, Mapping) or set(checks) != required_checks or any(item is not True for item in checks.values()):
        raise SealError("illegal-link replay self-checks did not all pass")
    if not isinstance(runtime, Mapping) or runtime.get("all_fields_match") is not True or runtime.get("exact_field_set_match") is not True:
        raise SealError("illegal-link replay runtime identity mismatch")
    expected_runtime = runtime.get("expected")
    if not isinstance(expected_runtime, Mapping) or expected_runtime.get("runner_sha256") != implementation["runner"]["sha256"] or expected_runtime.get("collision_core_sha256") != implementation["core"]["sha256"] or runtime.get("runner_source_verified") is not True:
        raise SealError("illegal-link replay runner/core binding mismatch")
    manifest_self_hash = inputs.get("v2_manifest_self_hash") if isinstance(inputs, Mapping) else None
    manifest_declared = manifest_self_hash.get("declared") if isinstance(manifest_self_hash, Mapping) else manifest_self_hash
    manifest_match = manifest_self_hash.get("match") is True if isinstance(manifest_self_hash, Mapping) else True
    if not isinstance(inputs, Mapping) or manifest_declared != manifest.get("manifest_content_sha256") or not manifest_match or inputs.get("v2_manifest_file_sha256") != published_files["manifest.json"]["sha256"]:
        raise SealError("illegal-link replay v2 manifest binding mismatch")
    watermark = inputs.get("v2_selection_watermark"); summary = value.get("summary")
    if not isinstance(watermark, Mapping) or type(watermark.get("row_count")) is not int or watermark["row_count"] <= 0 or not isinstance(watermark.get("framed_full_row_content_sha256"), str) or not isinstance(summary, Mapping) or type(summary.get("asset_replays")) is not int or summary["asset_replays"] <= 0 or type(summary.get("state_replays")) is not int or summary["state_replays"] <= 0:
        raise SealError("illegal-link replay selection/summary closure mismatch")
    if inputs.get("v1_database_sha256") != parity_inputs.get("v1_database_full_file_sha256") or inputs.get("v1_receipt_file_sha256") != parity_inputs.get("v1_full_release_receipt_file_sha256") or inputs.get("v1_receipt_content_sha256") != parity_inputs.get("v1_full_release_receipt_content_sha256"):
        raise SealError("replay/parity v1 evidence binding mismatch")
    if value.get("implementation", {}).get("script_sha256") != implementation["illegal_link_replayer"]["sha256"]:
        raise SealError("illegal-link replay implementation binding mismatch")
    return value


def verify_mirror(root: Path) -> dict[str, Any]:
    root = regular_dir(root, "mirror metadata")
    manifest_path, summary_path, artifact_path = root / "manifest.json", root / "summary.json", root / "artifact_manifest.json"
    manifest = verify_hashed_artifact(manifest_path, MIRROR_MANIFEST_SCHEMA, "manifest_content_sha256", "mirror manifest")
    summary = verify_hashed_artifact(summary_path, MIRROR_SUMMARY_SCHEMA, "summary_content_sha256", "mirror summary")
    artifact = verify_hashed_artifact(artifact_path, MIRROR_ARTIFACT_SCHEMA, "artifact_manifest_content_sha256", "mirror artifact manifest")
    if summary.get("status") != "COMPLETE" or summary.get("manifest_content_sha256") != manifest.get("manifest_content_sha256"):
        raise SealError("mirror summary is incomplete or unbound")
    archive_count = manifest.get("archive_count")
    if type(archive_count) is not int or archive_count < 0 or summary.get("archive_count") != archive_count:
        raise SealError("mirror archive count binding mismatch")
    rows = artifact.get("files")
    if not isinstance(rows, list) or not rows:
        raise SealError("mirror artifact manifest has no files")
    files = {}
    for row in rows:
        if not isinstance(row, Mapping) or set(row) != {"path", "bytes", "sha256"}:
            raise SealError("invalid mirror artifact row")
        path = safe_child(root, row.get("path"), "mirror artifact")
        name = str(row["path"])
        if name in files or path.stat().st_size != row.get("bytes") or sha256_file(path) != row.get("sha256"):
            raise SealError(f"mirror artifact closure mismatch: {name}")
        files[name] = file_binding(path)
    for required in ("manifest.json", "summary.json", "archive_records.jsonl"):
        if required not in files:
            raise SealError(f"mirror artifact closure missing {required}")
    records_path = root / "archive_records.jsonl"
    record_count = read_file_bytes(records_path, "mirror archive records").count(b"\n")
    marker_count = sum(1 for name in files if name.startswith("markers/") and name.endswith(".done.json"))
    if record_count != archive_count or marker_count != archive_count or len(files) != archive_count + 3:
        raise SealError("mirror records/markers/artifact count closure mismatch")
    return {"root": str(root.absolute()), "manifest": file_binding(manifest_path), "summary": file_binding(summary_path), "artifact_manifest": file_binding(artifact_path), "declared_file_count": len(files), "declared_files_content_sha256": hashlib.sha256(canonical_text(files).encode()).hexdigest()}


def verify_protocol_documents(manifest: Mapping[str, Any], published_files: Mapping[str, Any], protocol_doc: Path) -> dict[str, Any]:
    protocol = manifest.get("protocol")
    if not isinstance(protocol, Mapping):
        raise SealError("run manifest protocol binding is missing")
    snapshot = published_files.get("protocol_snapshot.md")
    if not isinstance(snapshot, Mapping):
        raise SealError("published run-start protocol snapshot binding is missing")
    if protocol.get("snapshot") != "protocol_snapshot.md":
        raise SealError("run manifest protocol snapshot path mismatch")
    if protocol.get("snapshot_sha256") != snapshot.get("sha256"):
        raise SealError("run manifest protocol snapshot SHA-256 mismatch")
    post_run_report = file_binding(Path(protocol_doc))
    if protocol.get("source_document") != post_run_report["path"]:
        raise SealError("post-run report is not the run-start source document")
    return {"run_start_snapshot": dict(snapshot), "post_run_report": post_run_report}


def seal(*, published: Path, copy_receipt: Path, checker_report: Path, parity_report: Path,
         cohort_audit: Path, failure_diagnosis: Path, illegal_link_replay: Path,
         mirror_metadata: Path, protocol_doc: Path, output: Path) -> Path:
    _LOADED_JSON_BINDINGS.clear()
    published = Path(published).absolute(); output = Path(output).absolute()
    mirror_metadata = Path(mirror_metadata).absolute()
    resolved_output_parent = output.parent.resolve(strict=True)
    for protected in (published.resolve(strict=True), mirror_metadata.resolve(strict=True)):
        if resolved_output_parent == protected or protected in resolved_output_parent.parents:
            raise SealError("output must be external to published and mirror inputs")
    manifest, run_receipt, published_files = verify_published(published)
    implementations = {name: file_binding(path) for name, path in IMPLEMENTATIONS.items()}
    source_hashes = manifest.get("source_hashes")
    if not isinstance(source_hashes, Mapping) or source_hashes.get("adapter") != implementations["wrapper"]["sha256"] or source_hashes.get("table4_runner") != implementations["runner"]["sha256"] or source_hashes.get("table4_core") != implementations["core"]["sha256"]:
        raise SealError("run manifest implementation binding mismatch")
    copy_value = verify_copy_receipt(Path(copy_receipt), published, published_files, implementations)
    verify_checker(Path(checker_report), published, manifest, run_receipt)
    parity_value = verify_parity(Path(parity_report), run_receipt, published_files, implementations)
    parity_inputs = parity_value["inputs"]
    verify_cohort(Path(cohort_audit), implementations, parity_inputs)
    verify_failure(Path(failure_diagnosis), implementations, parity_inputs)
    verify_replay(Path(illegal_link_replay), manifest, published_files, implementations, parity_inputs)
    mirror = verify_mirror(mirror_metadata)
    protocol_documents = verify_protocol_documents(manifest, published_files, Path(protocol_doc))
    inputs = {
        "published_run": {"path": str(published), "N_eval": manifest["N_eval"], "J_eval": manifest["J_eval"], "category_count": manifest["category_count"], "sampling_protocol": SAMPLING_PROTOCOL, "protocol_id": PROTOCOL_ID, "files": published_files},
        "copy_receipt": file_binding(Path(copy_receipt)), "internal_copy_receipt": file_binding(published / INTERNAL_COPY_RECEIPT), "automation_checker_report": file_binding(Path(checker_report)),
        "full_parity_report": file_binding(Path(parity_report)), "cohort_audit": file_binding(Path(cohort_audit)),
        "failure_diagnosis": file_binding(Path(failure_diagnosis)), "illegal_link_replay": file_binding(Path(illegal_link_replay)),
        "mirror_metadata": mirror, "protocol_documents": protocol_documents,
    }
    if inputs["copy_receipt"]["sha256"] != inputs["internal_copy_receipt"]["sha256"] or inputs["copy_receipt"]["bytes"] != inputs["internal_copy_receipt"]["bytes"]:
        raise SealError("internal/external copy receipt changed after validation")
    # Recheck every directly bound file before committing the receipt.
    for group in inputs.values():
        if isinstance(group, Mapping) and "path" in group and "sha256" in group:
            if file_binding(Path(str(group["path"]))) != group:
                raise SealError("input changed while publication receipt was assembled")
    value: dict[str, Any] = {"schema_version": SCHEMA, "publication_ready": True, "inputs": inputs, "implementation": implementations, "copy_receipt_content_sha256": copy_value["copy_receipt_content_sha256"]}
    value["publication_receipt_content_sha256"] = self_hash(value, "publication_receipt_content_sha256")
    def recheck_bindings(node: Any) -> None:
        if isinstance(node, Mapping):
            if set(("path", "bytes", "sha256")).issubset(node):
                expected = {"path": node["path"], "bytes": node["bytes"], "sha256": node["sha256"]}
                if file_binding(Path(str(node["path"]))) != expected:
                    raise SealError("bound input changed while receipt was assembled")
            for child in node.values(): recheck_bindings(child)
        elif isinstance(node, list):
            for child in node: recheck_bindings(child)
    recheck_bindings(value)
    parent = regular_dir(output.parent, "output parent")
    if output.exists() or output.is_symlink():
        raise SealError(f"output already exists: {output}")
    data = (canonical_text(value) + "\n").encode("ascii")
    descriptor = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data); stream.flush(); os.fsync(stream.fileno())
        directory_fd = os.open(parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try: os.fsync(directory_fd)
        finally: os.close(directory_fd)
    except BaseException:
        # Preserve a created output for forensic inspection; never overwrite/delete.
        raise
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--published", type=Path, required=True); parser.add_argument("--copy-receipt", type=Path, required=True)
    parser.add_argument("--checker-report", type=Path, required=True); parser.add_argument("--parity-report", type=Path, required=True)
    parser.add_argument("--cohort-audit", type=Path, required=True); parser.add_argument("--failure-diagnosis", type=Path, required=True)
    parser.add_argument("--illegal-link-replay", type=Path, required=True); parser.add_argument("--mirror-metadata", type=Path, required=True)
    parser.add_argument("--protocol-doc", type=Path, required=True, help="post-run report document; path must match manifest.protocol.source_document"); parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv); seal(**vars(args)); print(args.output); return 0


if __name__ == "__main__":
    raise SystemExit(main())
