from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "publish_pva_table4_mimic_aware_full_release.py"
SPEC = importlib.util.spec_from_file_location("pva_table4_publisher", SCRIPT)
assert SPEC and SPEC.loader
publisher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(publisher)


@pytest.fixture(autouse=True)
def _stub_full_checker(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(publisher.checker, "check", lambda _root: {"all_pass": True})


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n", encoding="utf-8")


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _seal(value: dict, field: str) -> dict:
    value[field] = publisher._self_hash(value, field)
    return value


def _fixture(tmp_path: Path) -> Path:
    root = tmp_path / "source"
    root.mkdir()
    manifest = _seal(
        {
            "schema_version": publisher.MANIFEST_SCHEMA,
            "classification": "FORMAL_FULL_RELEASE",
            "limit": None,
            "N_eval": 2,
            "J_eval": 3,
            "category_count": 1,
            "sampling_protocol": publisher.SAMPLING_PROTOCOL,
            "protocol_id": publisher.PROTOCOL_ID,
        },
        "manifest_content_sha256",
    )
    _write_json(root / "manifest.json", manifest)
    (root / "protocol_snapshot.md").write_text("frozen protocol\n", encoding="utf-8")
    (root / "records.jsonl").write_bytes(b"{}\n{}\n")
    (root / "asset_records.jsonl").write_bytes((root / "records.jsonl").read_bytes())
    (root / "state_records.jsonl").write_bytes(b"{}\n")
    summary = _seal({"schema_version": "pva_table4_mimic_aware_summary_v2", "status": "COMPLETE"}, "summary_content_sha256")
    _write_json(root / "summary.json", summary)
    (root / "summary.md").write_text("complete\n", encoding="utf-8")
    (root / "results.sqlite3").write_bytes(b"sealed database fixture")
    checkpoint = _seal(
        {
            "schema_version": publisher.CHECKPOINT_SCHEMA,
            "state": "complete",
            "records": 2,
            "records_sha256": _sha(root / "records.jsonl"),
            "state_records_sha256": _sha(root / "state_records.jsonl"),
            "summary_sha256": _sha(root / "summary.json"),
            "manifest_content_sha256": manifest["manifest_content_sha256"],
        },
        "checkpoint_content_sha256",
    )
    _write_json(root / "checkpoint.json", checkpoint)
    artifacts = [
        {"path": name, "bytes": (root / name).stat().st_size, "sha256": _sha(root / name)}
        for name in publisher.SEALED_ARTIFACTS
    ]
    artifact = _seal(
        {"schema_version": publisher.ARTIFACT_SCHEMA, "artifacts": artifacts},
        "artifact_manifest_content_sha256",
    )
    _write_json(root / "artifact_manifest.json", artifact)
    receipt = _seal(
        {
            "schema_version": publisher.RECEIPT_SCHEMA,
            "classification": "FORMAL_FULL_RELEASE",
            "N_eval": 2,
            "J_eval": 3,
            "category_count": 1,
            "sampling_protocol": publisher.SAMPLING_PROTOCOL,
            "protocol_id": publisher.PROTOCOL_ID,
            "manifest": "manifest.json",
            "manifest_sha256": _sha(root / "manifest.json"),
            "records": "records.jsonl",
            "records_sha256": _sha(root / "records.jsonl"),
            "state_records": "state_records.jsonl",
            "state_records_sha256": _sha(root / "state_records.jsonl"),
            "summary": "summary.json",
            "summary_sha256": _sha(root / "summary.json"),
            "artifact_manifest": "artifact_manifest.json",
            "artifact_manifest_sha256": _sha(root / "artifact_manifest.json"),
            "result_database": "results.sqlite3",
            "result_database_sha256": _sha(root / "results.sqlite3"),
        },
        "receipt_content_sha256",
    )
    _write_json(root / "full_release_receipt.json", receipt)
    _write_json(
        root / "progress.json",
        {
            "schema_version": publisher.PROGRESS_SCHEMA,
            "status": "COMPLETE",
            "completed_assets": 2,
            "total_assets": 2,
        },
    )
    return root


def _reseal_artifact(root: Path) -> None:
    value = json.loads((root / "artifact_manifest.json").read_text())
    for entry in value["artifacts"]:
        path = root / entry["path"]
        entry.update(bytes=path.stat().st_size, sha256=_sha(path))
    value["artifact_manifest_content_sha256"] = publisher._self_hash(value, "artifact_manifest_content_sha256")
    _write_json(root / "artifact_manifest.json", value)


def test_publish_copies_exact_set_and_writes_bound_receipt(tmp_path: Path) -> None:
    source = _fixture(tmp_path)
    before = {name: _sha(source / name) for name in publisher.PUBLISHED_FILES}
    destination = tmp_path / "published"
    receipt_path = publisher.publish(source, destination)

    assert {path.name for path in destination.iterdir()} == set(publisher.PUBLISHED_FILES) | {publisher.INTERNAL_COPY_RECEIPT}
    assert {name: _sha(source / name) for name in publisher.PUBLISHED_FILES} == before
    assert {name: _sha(destination / name) for name in publisher.PUBLISHED_FILES} == before
    receipt = json.loads(receipt_path.read_text())
    assert receipt["source"] == str(source.absolute())
    assert receipt["destination"] == str(destination.absolute())
    assert len(receipt["files"]) == 12
    assert receipt["copy_receipt_content_sha256"] == publisher._self_hash(receipt, "copy_receipt_content_sha256")
    assert receipt["publisher_source_sha256"] == _sha(SCRIPT)
    assert receipt["checker_source_sha256"] == _sha(publisher.CHECKER_SOURCE)
    assert receipt["checker_all_pass"] is True
    assert (destination / publisher.INTERNAL_COPY_RECEIPT).read_bytes() == receipt_path.read_bytes()
    assert (destination / publisher.INTERNAL_COPY_RECEIPT).stat().st_ino == receipt_path.stat().st_ino
    assert not (tmp_path / ".published.publish-stage").exists()


@pytest.mark.parametrize("status", ["RUNNING", "complete", None])
def test_rejects_non_complete_progress(tmp_path: Path, status: str | None) -> None:
    source = _fixture(tmp_path)
    progress = json.loads((source / "progress.json").read_text())
    progress["status"] = status
    _write_json(source / "progress.json", progress)
    with pytest.raises(publisher.PublicationError, match="progress"):
        publisher.publish(source, tmp_path / "published")


def test_rejects_tampered_checkpoint_even_if_artifact_is_resealed(tmp_path: Path) -> None:
    source = _fixture(tmp_path)
    checkpoint = json.loads((source / "checkpoint.json").read_text())
    checkpoint["records"] = 1
    _write_json(source / "checkpoint.json", checkpoint)
    _reseal_artifact(source)
    with pytest.raises(publisher.PublicationError, match="checkpoint self-hash"):
        publisher.publish(source, tmp_path / "published")


def test_rejects_receipt_database_hash_tamper(tmp_path: Path) -> None:
    source = _fixture(tmp_path)
    receipt = json.loads((source / "full_release_receipt.json").read_text())
    receipt["result_database_sha256"] = "0" * 64
    receipt["receipt_content_sha256"] = publisher._self_hash(receipt, "receipt_content_sha256")
    _write_json(source / "full_release_receipt.json", receipt)
    with pytest.raises(publisher.PublicationError, match="result_database"):
        publisher.publish(source, tmp_path / "published")


def test_rejects_nonempty_wal_but_allows_zero_length_wal(tmp_path: Path) -> None:
    source = _fixture(tmp_path)
    wal = source / "results.sqlite3-wal"
    wal.write_bytes(b"active")
    with pytest.raises(publisher.PublicationError, match="sidecar"):
        publisher.publish(source, tmp_path / "published")
    wal.write_bytes(b"")
    publisher.publish(source, tmp_path / "published")


def test_rejects_dangling_wal_symlink(tmp_path: Path) -> None:
    source = _fixture(tmp_path)
    (source / "results.sqlite3-wal").symlink_to(tmp_path / "missing")
    with pytest.raises(publisher.PublicationError, match="sidecar"):
        publisher.publish(source, tmp_path / "published")


def test_rejects_nonempty_shm(tmp_path: Path) -> None:
    source = _fixture(tmp_path)
    (source / "results.sqlite3-shm").write_bytes(b"active")
    with pytest.raises(publisher.PublicationError, match="sidecar"):
        publisher.publish(source, tmp_path / "published")


def test_rejects_source_change_during_copy_and_preserves_stage(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = _fixture(tmp_path)
    original = publisher.shutil.copyfileobj
    changed = False

    def mutate_after_first_copy(reader, writer, length):
        nonlocal changed
        original(reader, writer, length)
        if not changed:
            changed = True
            (source / "summary.md").write_text("changed during copy\n", encoding="utf-8")

    monkeypatch.setattr(publisher.shutil, "copyfileobj", mutate_after_first_copy)
    with pytest.raises(publisher.PublicationError, match="post-copy verification|source changed"):
        publisher.publish(source, tmp_path / "published")
    assert not (tmp_path / "published").exists()
    assert (tmp_path / ".published.publish-stage").is_dir()


def test_rejects_symlinked_artifact(tmp_path: Path) -> None:
    source = _fixture(tmp_path)
    external = tmp_path / "external"
    external.write_bytes((source / "summary.md").read_bytes())
    (source / "summary.md").unlink()
    (source / "summary.md").symlink_to(external)
    with pytest.raises(publisher.PublicationError, match="non-symlink"):
        publisher.publish(source, tmp_path / "published")


@pytest.mark.parametrize("existing", ["destination", "stage", "receipt"])
def test_existing_targets_fail_closed_without_deletion(tmp_path: Path, existing: str) -> None:
    source = _fixture(tmp_path)
    destination = tmp_path / "published"
    stage = tmp_path / ".published.publish-stage"
    receipt = tmp_path / "published.copy_receipt.json"
    target = {"destination": destination, "stage": stage, "receipt": receipt}[existing]
    if existing == "receipt":
        target.write_text("keep", encoding="utf-8")
    else:
        target.mkdir()
        (target / "keep").write_text("keep", encoding="utf-8")
    with pytest.raises(publisher.PublicationError, match="already exists"):
        publisher.publish(source, destination)
    assert target.exists()
    assert (target / "keep").exists() if target.is_dir() else target.read_text() == "keep"


def test_rejects_smoke_manifest_even_when_counts_are_complete(tmp_path: Path) -> None:
    source = _fixture(tmp_path)
    manifest = json.loads((source / "manifest.json").read_text())
    manifest["classification"] = "SMOKE"
    manifest["limit"] = 2
    manifest["manifest_content_sha256"] = publisher._self_hash(manifest, "manifest_content_sha256")
    _write_json(source / "manifest.json", manifest)
    _reseal_artifact(source)
    with pytest.raises(publisher.PublicationError, match="FORMAL_FULL_RELEASE"):
        publisher.publish(source, tmp_path / "published")


def test_rejects_checker_failure_before_creating_stage(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = _fixture(tmp_path)
    monkeypatch.setattr(publisher.checker, "check", lambda _root: {"all_pass": False})
    with pytest.raises(publisher.PublicationError, match="checker"):
        publisher.publish(source, tmp_path / "published")
    assert not (tmp_path / ".published.publish-stage").exists()


def test_rejects_destination_inside_source(tmp_path: Path) -> None:
    source = _fixture(tmp_path)
    with pytest.raises(publisher.PublicationError, match="read-only source"):
        publisher.publish(source, source / "published")


def test_rejects_copy_receipt_aliasing_destination(tmp_path: Path) -> None:
    source = _fixture(tmp_path)
    destination = tmp_path / "published"
    with pytest.raises(publisher.PublicationError, match="alias"):
        publisher.publish(source, destination, copy_receipt=destination)


def test_destination_race_is_no_clobber_and_preserves_stage(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = _fixture(tmp_path)
    destination = tmp_path / "published"
    calls = 0

    def checker_with_race(_root):
        nonlocal calls
        calls += 1
        if calls == 2:
            destination.mkdir()
            (destination / "keep").write_text("keep", encoding="utf-8")
        return {"all_pass": True}

    monkeypatch.setattr(publisher.checker, "check", checker_with_race)
    with pytest.raises(publisher.PublicationError, match="appeared"):
        publisher.publish(source, destination)
    assert (destination / "keep").read_text() == "keep"
    assert (tmp_path / ".published.publish-stage").is_dir()


def test_receipt_race_is_no_clobber_but_internal_receipt_recovers_publication(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = _fixture(tmp_path)
    destination = tmp_path / "published"
    receipt_path = tmp_path / "published.copy_receipt.json"
    original_link = publisher.os.link

    def race_link(src, dst):
        Path(dst).write_text("keep", encoding="utf-8")
        return original_link(src, dst)

    monkeypatch.setattr(publisher.os, "link", race_link)
    with pytest.raises(publisher.PublicationError, match="receipt appeared"):
        publisher.publish(source, destination)
    assert receipt_path.read_text() == "keep"
    internal = json.loads((destination / publisher.INTERNAL_COPY_RECEIPT).read_text())
    assert internal["destination"] == str(destination.absolute())
    assert internal["copy_receipt_content_sha256"] == publisher._self_hash(internal, "copy_receipt_content_sha256")


def test_rejects_checker_source_change_during_final_stage_check(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = _fixture(tmp_path)
    destination = tmp_path / "published"
    checker_source = tmp_path / "checker.py"
    checker_source.write_text("original\n", encoding="utf-8")
    monkeypatch.setattr(publisher, "CHECKER_SOURCE", checker_source)
    calls = 0

    def checker_that_changes_after_final_check(_root):
        nonlocal calls
        calls += 1
        if calls == 3:
            checker_source.write_text("changed\n", encoding="utf-8")
        return {"all_pass": True}

    monkeypatch.setattr(publisher.checker, "check", checker_that_changes_after_final_check)
    with pytest.raises(publisher.PublicationError, match="source changed"):
        publisher.publish(source, destination)
    assert not destination.exists()
    assert (tmp_path / ".published.publish-stage").is_dir()


def test_rename_noreplace_uses_empty_directory_reservation_when_flag_unsupported(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "source"
    destination = tmp_path / "destination"
    source.mkdir()
    (source / "payload").write_text("kept", encoding="utf-8")

    def unsupported(_source: Path, _destination: Path) -> None:
        raise OSError(publisher.errno.EINVAL, "unsupported rename flag")

    monkeypatch.setattr(publisher, "_renameat2_noreplace", unsupported)
    source_inode = source.stat().st_ino
    publisher._rename_noreplace(source, destination)

    assert not source.exists()
    assert destination.stat().st_ino == source_inode
    assert (destination / "payload").read_text() == "kept"


def test_reservation_fallback_does_not_clobber_existing_destination(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "source"
    destination = tmp_path / "destination"
    source.mkdir()
    destination.mkdir()
    (source / "payload").write_text("source", encoding="utf-8")
    (destination / "keep").write_text("destination", encoding="utf-8")

    def unsupported(_source: Path, _destination: Path) -> None:
        raise OSError(publisher.errno.EINVAL, "unsupported rename flag")

    monkeypatch.setattr(publisher, "_renameat2_noreplace", unsupported)
    with pytest.raises(publisher.PublicationError, match="appeared"):
        publisher._rename_noreplace(source, destination)

    assert (source / "payload").read_text() == "source"
    assert (destination / "keep").read_text() == "destination"


def test_recover_failed_install_rebinds_publisher_and_preserves_old_receipt(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = _fixture(tmp_path)
    destination = tmp_path / "published"
    original_install = publisher._rename_noreplace

    def failed_install(_source: Path, _destination: Path) -> None:
        raise OSError(publisher.errno.EINVAL, "unsupported rename flag")

    monkeypatch.setattr(publisher, "_rename_noreplace", failed_install)
    with pytest.raises(OSError, match="unsupported rename flag"):
        publisher.publish(source, destination)

    stage = tmp_path / ".published.publish-stage"
    old_receipt_path = stage / publisher.INTERNAL_COPY_RECEIPT
    old_receipt_sha256 = _sha(old_receipt_path)
    prior_publisher_sha256 = _sha(SCRIPT)

    def unsupported(_source: Path, _destination: Path) -> None:
        raise OSError(publisher.errno.EINVAL, "unsupported rename flag")

    monkeypatch.setattr(publisher, "_rename_noreplace", original_install)
    monkeypatch.setattr(publisher, "_renameat2_noreplace", unsupported)
    receipt_path = publisher.recover_failed_install(
        destination,
        prior_publisher_sha256=prior_publisher_sha256,
        trigger_errno=publisher.errno.EINVAL,
    )

    backup = tmp_path / "published.failed_rename_receipt.json"
    recovered = json.loads(receipt_path.read_text())
    assert _sha(backup) == old_receipt_sha256
    assert recovered["publisher_source_sha256"] == _sha(SCRIPT)
    assert recovered["failed_install_recovery"]["prior_copy_receipt"]["sha256"] == old_receipt_sha256
    assert recovered["copy_receipt_content_sha256"] == publisher._self_hash(
        recovered, "copy_receipt_content_sha256"
    )
    assert (destination / publisher.INTERNAL_COPY_RECEIPT).stat().st_ino == receipt_path.stat().st_ino
    assert not stage.exists()


def test_recovery_rejects_stage_change_after_final_hash(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = _fixture(tmp_path)
    destination = tmp_path / "published"

    def failed_install(_source: Path, _destination: Path) -> None:
        raise OSError(publisher.errno.EINVAL, "unsupported rename flag")

    monkeypatch.setattr(publisher, "_rename_noreplace", failed_install)
    with pytest.raises(OSError):
        publisher.publish(source, destination)
    monkeypatch.undo()

    original_validate = publisher._validate_stage_file_rows
    calls = 0

    def mutate_after_final_hash(stage: Path, rows):
        nonlocal calls
        calls += 1
        result = original_validate(stage, rows)
        if calls == 2:
            (stage / "summary.md").write_text("changed after final hash\n", encoding="utf-8")
        return result

    monkeypatch.setattr(publisher, "_validate_stage_file_rows", mutate_after_final_hash)
    with pytest.raises(publisher.PublicationError, match="changed before installation"):
        publisher.recover_failed_install(
            destination,
            prior_publisher_sha256=_sha(SCRIPT),
            trigger_errno=publisher.errno.EINVAL,
        )
    assert not destination.exists()
    assert (tmp_path / ".published.publish-stage").is_dir()


def test_recovery_rejects_path_aliases(tmp_path: Path) -> None:
    destination = tmp_path / "published"
    with pytest.raises(publisher.PublicationError, match="pairwise distinct"):
        publisher.recover_failed_install(
            destination,
            prior_publisher_sha256=_sha(SCRIPT),
            trigger_errno=publisher.errno.EINVAL,
            copy_receipt=destination,
        )


def test_recovery_completes_external_receipt_link_after_install_crash(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = _fixture(tmp_path)
    destination = tmp_path / "published"

    def failed_install(_source: Path, _destination: Path) -> None:
        raise OSError(publisher.errno.EINVAL, "unsupported rename flag")

    monkeypatch.setattr(publisher, "_rename_noreplace", failed_install)
    with pytest.raises(OSError):
        publisher.publish(source, destination)
    monkeypatch.undo()

    original_link = publisher.os.link
    calls = 0

    def fail_external_link(source_path, destination_path):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError(publisher.errno.EIO, "simulated crash before external receipt")
        return original_link(source_path, destination_path)

    monkeypatch.setattr(publisher.os, "link", fail_external_link)
    with pytest.raises(OSError, match="simulated crash"):
        publisher.recover_failed_install(
            destination,
            prior_publisher_sha256=_sha(SCRIPT),
            trigger_errno=publisher.errno.EINVAL,
        )
    assert destination.is_dir()
    receipt_path = tmp_path / "published.copy_receipt.json"
    assert not receipt_path.exists()

    monkeypatch.setattr(publisher.os, "link", original_link)
    completed = publisher.recover_failed_install(
        destination,
        prior_publisher_sha256=_sha(SCRIPT),
        trigger_errno=publisher.errno.EINVAL,
    )
    assert completed == receipt_path
    assert os.path.samefile(
        destination / publisher.INTERNAL_COPY_RECEIPT, receipt_path
    )
