from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "seal_pva_table4_mimic_aware_publication.py"
SPEC = importlib.util.spec_from_file_location("publication_sealer", SCRIPT)
assert SPEC and SPEC.loader
sealer = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(sealer)


@pytest.fixture(autouse=True)
def stub_checker(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sealer.checker, "check", lambda output: {"schema_version": sealer.CHECK_SCHEMA, "all_pass": True, "output": str(Path(output).absolute()), "N_eval": 2, "J_eval": 3, "status": "COMPLETE"})


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(sealer.canonical_text(value) + "\n", encoding="ascii")


def sealed(value: dict, field: str) -> dict:
    value[field] = sealer.self_hash(value, field); return value


def binding(path: Path) -> dict:
    return {"path": path.name, "bytes": path.stat().st_size, "sha256": sealer.sha256_file(path)}


def protocol_documents_fixture(tmp_path: Path) -> tuple[dict, dict, Path, Path]:
    root = tmp_path / "protocol_documents"; root.mkdir()
    snapshot = root / "protocol_snapshot.md"; snapshot.write_text("run-start protocol\n", encoding="ascii")
    report = root / "protocol.md"; report.write_text("run-start protocol\npost-run report\n", encoding="ascii")
    manifest = {"protocol": {"source_document": str(report.resolve()), "snapshot": "protocol_snapshot.md", "snapshot_sha256": sealer.sha256_file(snapshot)}}
    published_files = {"protocol_snapshot.md": sealer.file_binding(snapshot)}
    return manifest, published_files, snapshot, report


def fixture(tmp_path: Path) -> dict[str, Path]:
    published = tmp_path / "published"; published.mkdir()
    snapshot = published / "protocol_snapshot.md"; snapshot.write_text("run-start protocol\n", encoding="ascii")
    protocol = tmp_path / "protocol.md"; protocol.write_text("run-start protocol\npost-run report\n", encoding="ascii")
    impl = {name: sealer.file_binding(path) for name, path in sealer.IMPLEMENTATIONS.items()}
    manifest = sealed({"schema_version": sealer.RUN_SCHEMA, "classification": "FORMAL_FULL_RELEASE", "limit": None, "N_eval": 2, "J_eval": 3, "category_count": 1, "sampling_protocol": sealer.SAMPLING_PROTOCOL, "protocol_id": sealer.PROTOCOL_ID, "protocol": {"source_document": str(protocol.resolve()), "snapshot": "protocol_snapshot.md", "snapshot_sha256": sealer.sha256_file(snapshot)}, "source_hashes": {"adapter": impl["wrapper"]["sha256"], "table4_runner": impl["runner"]["sha256"], "table4_core": impl["core"]["sha256"]}}, "manifest_content_sha256")
    write_json(published / "manifest.json", manifest)
    for name, data in {"records.jsonl": b"{}\n{}\n", "asset_records.jsonl": b"{}\n{}\n", "state_records.jsonl": b"{}\n", "summary.md": b"ok\n", "results.sqlite3": b"db"}.items():
        (published / name).write_bytes(data)
    write_json(published / "summary.json", sealed({"schema_version": "pva_table4_mimic_aware_summary_v2", "status": "COMPLETE"}, "summary_content_sha256"))
    write_json(published / "checkpoint.json", sealed({"schema_version": "pva_table4_mimic_aware_checkpoint_v1", "state": "complete"}, "checkpoint_content_sha256"))
    write_json(published / "progress.json", {"schema_version": "pva_table4_mimic_aware_progress_v1", "status": "COMPLETE"})
    nine = sealer.PUBLISHED_SOURCE_FILES[:9]
    artifact = sealed({"schema_version": "pva_table4_mimic_aware_artifacts_v1", "artifacts": [binding(published / name) for name in nine]}, "artifact_manifest_content_sha256")
    write_json(published / "artifact_manifest.json", artifact)
    receipt = sealed({"schema_version": sealer.RUN_RECEIPT_SCHEMA, "classification": "FORMAL_FULL_RELEASE", "N_eval": 2, "J_eval": 3, "category_count": 1, "sampling_protocol": sealer.SAMPLING_PROTOCOL, "protocol_id": sealer.PROTOCOL_ID, "status": "COMPLETE", "manifest_sha256": sealer.sha256_file(published / "manifest.json"), "records_sha256": sealer.sha256_file(published / "records.jsonl"), "state_records_sha256": sealer.sha256_file(published / "state_records.jsonl"), "summary_sha256": sealer.sha256_file(published / "summary.json"), "artifact_manifest_sha256": sealer.sha256_file(published / "artifact_manifest.json"), "result_database_sha256": sealer.sha256_file(published / "results.sqlite3")}, "receipt_content_sha256")
    write_json(published / "full_release_receipt.json", receipt)
    rows = []
    for name in sealer.PUBLISHED_SOURCE_FILES:
        path = published / name; rows.append({"path": name, "bytes": path.stat().st_size, "sha256": sealer.sha256_file(path)})
    copy_value = sealed({"schema_version": sealer.COPY_RECEIPT_SCHEMA, "source": str(tmp_path / "source"), "destination": str(published.absolute()), "files": rows, "publisher_source_sha256": impl["publisher"]["sha256"], "checker_source_sha256": impl["checker"]["sha256"]}, "copy_receipt_content_sha256")
    internal = published / sealer.INTERNAL_COPY_RECEIPT; external = tmp_path / "copy.json"
    write_json(internal, copy_value); external.write_bytes(internal.read_bytes())
    checker = tmp_path / "checker.json"; write_json(checker, {"schema_version": sealer.CHECK_SCHEMA, "all_pass": True, "output": str(published.absolute()), "N_eval": 2, "J_eval": 3, "status": "COMPLETE"})
    parity = tmp_path / "parity.json"
    v1_receipt = tmp_path / "v1_receipt.json"; write_json(v1_receipt, {"schema_version": "fixture"}); v1_db_hash = "b" * 64; v1_receipt_hash = sealer.sha256_file(v1_receipt); v1_content_hash = "c" * 64
    parity_value = {"schema_version": sealer.PARITY_SCHEMA, "all_pass": True, "publication_status": {"final_publication_eligible": True, "interim_non_durable_active_snapshot": False}, "inputs": {"v2_full_release_receipt_verified": True, "v2_full_release_receipt_content_sha256": receipt["receipt_content_sha256"], "v2_full_release_receipt_file_sha256": sealer.sha256_file(published / "full_release_receipt.json"), "v2_database_full_file_sha256": sealer.sha256_file(published / "results.sqlite3"), "v1_database_full_file_sha256": v1_db_hash, "v1_full_release_receipt_file_sha256": v1_receipt_hash, "v1_full_release_receipt_content_sha256": v1_content_hash}, "implementation": {"script_sha256": impl["parity"]["sha256"]}}
    write_json(parity, sealed(parity_value, "audit_content_sha256"))
    artifacts = {}
    cohort_validation = {key: True for key in {"ours_manifest_self_hash_verified", "ours_frozen_row_identities_verified", "ours_primary_urdf_payload_sha256_verified", "ours_primary_urdf_regular_non_symlink_verified", "ours_state_semantics_independently_replayed", "ours_load_child_worker_success_verified", "ours_reset_readback_tolerance_verified", "ours_formal_summary_recomputed", "ours_formal_verification_bound", "ours_result_closure_verified", "pva_execution_manifest_self_hash_verified", "pva_receipt_self_hash_verified", "pva_database_receipt_sha256_verified", "pva_database_opened_mode_ro", "pva_roster_row_sha256_verified", "pva_table4_input_identities_verified", "label_match_identity_closure_verified", "declared_protocol_fields_aligned", "category_partition_closure_verified"}}
    cohort_value = {"schema_version": sealer.COHORT_SCHEMA, "validation": cohort_validation, "inputs": {"current_pva_v1": {"full_release_receipt": {"file_sha256": v1_receipt_hash, "receipt_content_sha256": v1_content_hash}, "results_database": {"file_sha256": v1_db_hash}}}, "implementation": {"script_sha256": impl["cohort_auditor"]["sha256"]}}
    artifacts["cohort"] = tmp_path / "cohort.json"; write_json(artifacts["cohort"], sealed(cohort_value, "audit_content_sha256"))
    failure_value = {"schema_version": sealer.FAILURE_SCHEMA, "strict_summary": {"denominator": 2, "passed": 1, "failed": 1}, "inputs": {"database_sha256": v1_db_hash, "receipt": str(v1_receipt)}, "implementation": {"script_sha256": impl["failure_diagnoser"]["sha256"]}}
    artifacts["failure"] = tmp_path / "failure.json"; write_json(artifacts["failure"], sealed(failure_value, "artifact_content_sha256"))
    replay_checks = {key: True for key in {"all_selected_states_reproduced_illegal_contacts", "all_emitted_depths_exceed_threshold", "source_phase_sample_joint_checked_before_each_replay", "joint_value_hash_checked_before_each_replay", "source_record_blob_and_state_identity_hashed"}}
    replay_value = {"schema_version": sealer.REPLAY_SCHEMA, "self_checks": replay_checks, "runtime_identity": {"all_fields_match": True, "exact_field_set_match": True, "runner_source_verified": True, "expected": {"runner_sha256": impl["runner"]["sha256"], "collision_core_sha256": impl["core"]["sha256"]}}, "inputs": {"v1_database_sha256": v1_db_hash, "v1_receipt_file_sha256": v1_receipt_hash, "v1_receipt_content_sha256": v1_content_hash, "v2_manifest_self_hash": {"declared": manifest["manifest_content_sha256"], "computed": manifest["manifest_content_sha256"], "match": True}, "v2_manifest_file_sha256": sealer.sha256_file(published / "manifest.json"), "v2_selection_watermark": {"row_count": 2, "framed_full_row_content_sha256": "d" * 64}}, "summary": {"asset_replays": 1, "state_replays": 1}, "implementation": {"script_sha256": impl["illegal_link_replayer"]["sha256"]}}
    artifacts["replay"] = tmp_path / "replay.json"; write_json(artifacts["replay"], sealed(replay_value, "artifact_content_sha256"))
    mirror = tmp_path / "mirror"; mirror.mkdir()
    mirror_manifest = sealed({"schema_version": sealer.MIRROR_MANIFEST_SCHEMA, "archive_count": 1}, "manifest_content_sha256"); write_json(mirror / "manifest.json", mirror_manifest)
    (mirror / "archive_records.jsonl").write_text("{}\n", encoding="ascii")
    mirror_summary = sealed({"schema_version": sealer.MIRROR_SUMMARY_SCHEMA, "status": "COMPLETE", "archive_count": 1, "manifest_content_sha256": mirror_manifest["manifest_content_sha256"]}, "summary_content_sha256"); write_json(mirror / "summary.json", mirror_summary)
    marker = mirror / "markers" / "one.done.json"; marker.parent.mkdir(); write_json(marker, {"schema_version": "fixture"})
    mirror_rows = []
    for name in ("manifest.json", "archive_records.jsonl", "summary.json", "markers/one.done.json"):
        path = mirror / name; mirror_rows.append({"path": name, "bytes": path.stat().st_size, "sha256": sealer.sha256_file(path)})
    write_json(mirror / "artifact_manifest.json", sealed({"schema_version": sealer.MIRROR_ARTIFACT_SCHEMA, "files": mirror_rows}, "artifact_manifest_content_sha256"))
    return {"published": published, "copy_receipt": external, "checker_report": checker, "parity_report": parity, "cohort_audit": artifacts["cohort"], "failure_diagnosis": artifacts["failure"], "illegal_link_replay": artifacts["replay"], "mirror_metadata": mirror, "protocol_doc": protocol, "output": tmp_path / "publication.json"}


def test_seals_complete_bundle_with_self_hash_and_implementation_bindings(tmp_path: Path) -> None:
    args = fixture(tmp_path); output = sealer.seal(**args); value = json.loads(output.read_text())
    assert value["schema_version"] == sealer.SCHEMA == "pva_table4_mimic_aware_publication_bundle_receipt_v2"
    assert value["publication_ready"] is True
    assert value["publication_receipt_content_sha256"] == sealer.self_hash(value, "publication_receipt_content_sha256")
    assert set(value["implementation"]) == set(sealer.IMPLEMENTATIONS)
    assert len(value["inputs"]["published_run"]["files"]) == 12
    assert value["inputs"]["mirror_metadata"]["declared_file_count"] == 4
    protocol_documents = value["inputs"]["protocol_documents"]
    assert set(protocol_documents) == {"run_start_snapshot", "post_run_report"}
    assert all(set(binding) == {"path", "bytes", "sha256"} for binding in protocol_documents.values())
    assert protocol_documents["run_start_snapshot"] == value["inputs"]["published_run"]["files"]["protocol_snapshot.md"]
    assert protocol_documents["post_run_report"] == sealer.file_binding(args["protocol_doc"])
    assert protocol_documents["run_start_snapshot"]["sha256"] != protocol_documents["post_run_report"]["sha256"]
    assert "protocol_document" not in value["inputs"]


def test_output_is_no_clobber(tmp_path: Path) -> None:
    args = fixture(tmp_path); args["output"].write_text("keep", encoding="ascii")
    with pytest.raises(sealer.SealError, match="already exists"): sealer.seal(**args)
    assert args["output"].read_text() == "keep"


def test_rejects_interim_or_ineligible_parity(tmp_path: Path) -> None:
    args = fixture(tmp_path); path = args["parity_report"]; value = json.loads(path.read_text())
    value["publication_status"]["interim_non_durable_active_snapshot"] = True
    value["audit_content_sha256"] = sealer.self_hash(value, "audit_content_sha256"); write_json(path, value)
    with pytest.raises(sealer.SealError, match="eligible"): sealer.seal(**args)


def test_rejects_copy_receipt_twelve_file_tamper(tmp_path: Path) -> None:
    args = fixture(tmp_path); path = args["copy_receipt"]; value = json.loads(path.read_text()); value["files"][0]["sha256"] = "0" * 64
    value["copy_receipt_content_sha256"] = sealer.self_hash(value, "copy_receipt_content_sha256"); write_json(path, value); (args["published"] / sealer.INTERNAL_COPY_RECEIPT).write_bytes(path.read_bytes())
    with pytest.raises(sealer.SealError, match="twelve-file"): sealer.seal(**args)


def test_rejects_mirror_closure_tamper(tmp_path: Path) -> None:
    args = fixture(tmp_path); (args["mirror_metadata"] / "archive_records.jsonl").write_text("tampered\n", encoding="ascii")
    with pytest.raises(sealer.SealError, match="mirror artifact closure"): sealer.seal(**args)


def test_rejects_symlinked_evidence(tmp_path: Path) -> None:
    args = fixture(tmp_path); target = args["failure_diagnosis"]; external = tmp_path / "external.json"; external.write_bytes(target.read_bytes()); target.unlink(); target.symlink_to(external)
    with pytest.raises(sealer.SealError, match="non-symlink"): sealer.seal(**args)


def test_accepts_unchanged_post_run_document(tmp_path: Path) -> None:
    args = fixture(tmp_path); snapshot = args["published"] / "protocol_snapshot.md"
    args["protocol_doc"].write_bytes(snapshot.read_bytes())
    value = json.loads(sealer.seal(**args).read_text())
    protocol_documents = value["inputs"]["protocol_documents"]
    assert protocol_documents["run_start_snapshot"]["sha256"] == protocol_documents["post_run_report"]["sha256"]


def test_rejects_missing_manifest_protocol_binding(tmp_path: Path) -> None:
    _, published_files, _, report = protocol_documents_fixture(tmp_path)
    with pytest.raises(sealer.SealError, match="protocol binding is missing"):
        sealer.verify_protocol_documents({}, published_files, report)


def test_rejects_manifest_protocol_snapshot_path_mismatch(tmp_path: Path) -> None:
    manifest, published_files, _, report = protocol_documents_fixture(tmp_path)
    manifest["protocol"]["snapshot"] = "other.md"
    with pytest.raises(sealer.SealError, match="snapshot path mismatch"):
        sealer.verify_protocol_documents(manifest, published_files, report)


def test_rejects_manifest_protocol_snapshot_hash_mismatch(tmp_path: Path) -> None:
    manifest, published_files, _, report = protocol_documents_fixture(tmp_path)
    manifest["protocol"]["snapshot_sha256"] = "0" * 64
    with pytest.raises(sealer.SealError, match="snapshot SHA-256 mismatch"):
        sealer.verify_protocol_documents(manifest, published_files, report)


def test_rejects_post_run_report_from_different_path_even_with_same_content(tmp_path: Path) -> None:
    manifest, published_files, _, report = protocol_documents_fixture(tmp_path)
    other = tmp_path / "other.md"; other.write_bytes(report.read_bytes())
    manifest["protocol"]["source_document"] = str(other.resolve())
    with pytest.raises(sealer.SealError, match="run-start source document"):
        sealer.verify_protocol_documents(manifest, published_files, report)


def test_rejects_symlinked_post_run_report(tmp_path: Path) -> None:
    manifest, published_files, _, report = protocol_documents_fixture(tmp_path)
    link = tmp_path / "protocol-link.md"; link.symlink_to(report)
    manifest["protocol"]["source_document"] = str(link.absolute())
    with pytest.raises(sealer.SealError, match="non-symlink"):
        sealer.verify_protocol_documents(manifest, published_files, link)


def test_rejects_post_run_report_changed_while_receipt_is_assembled(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    args = fixture(tmp_path); report = args["protocol_doc"].absolute(); original = sealer.file_binding
    changed = False

    def racing_file_binding(path: Path) -> dict:
        nonlocal changed
        observed = original(path)
        if Path(path).absolute() == report and not changed:
            changed = True
            report.write_text("changed after initial binding\n", encoding="ascii")
        return observed

    monkeypatch.setattr(sealer, "file_binding", racing_file_binding)
    with pytest.raises(sealer.SealError, match="bound input changed while receipt was assembled"):
        sealer.seal(**args)


def test_cli_help_describes_post_run_report(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit) as error:
        sealer.main(["--help"])
    assert error.value.code == 0
    assert "post-run report document" in capsys.readouterr().out


def test_rejects_output_inside_published_input(tmp_path: Path) -> None:
    args = fixture(tmp_path); args["output"] = args["published"] / "publication.json"
    with pytest.raises(sealer.SealError, match="external"): sealer.seal(**args)


def test_rejects_forged_checker_report(tmp_path: Path) -> None:
    args = fixture(tmp_path); report = json.loads(args["checker_report"].read_text()); report["state_records"] = 999; write_json(args["checker_report"], report)
    with pytest.raises(sealer.SealError, match="independent rerun"): sealer.seal(**args)


def test_rejects_cohort_with_failed_validation_gate(tmp_path: Path) -> None:
    args = fixture(tmp_path); path = args["cohort_audit"]; value = json.loads(path.read_text()); value["validation"]["pva_database_receipt_sha256_verified"] = False
    value["audit_content_sha256"] = sealer.self_hash(value, "audit_content_sha256"); write_json(path, value)
    with pytest.raises(sealer.SealError, match="validation gates"): sealer.seal(**args)


def test_rejects_replay_not_bound_to_published_manifest(tmp_path: Path) -> None:
    args = fixture(tmp_path); path = args["illegal_link_replay"]; value = json.loads(path.read_text()); value["inputs"]["v2_manifest_self_hash"] = "0" * 64
    value["artifact_content_sha256"] = sealer.self_hash(value, "artifact_content_sha256"); write_json(path, value)
    with pytest.raises(sealer.SealError, match="manifest binding"): sealer.seal(**args)
