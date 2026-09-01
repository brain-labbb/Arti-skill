from __future__ import annotations

import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tarfile
from typing import Any

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "stage_pva_archives_local.py"
SPEC = importlib.util.spec_from_file_location("stage_pva_archives_local_test", SCRIPT)
assert SPEC and SPEC.loader
stage = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = stage
SPEC.loader.exec_module(stage)


pytestmark = pytest.mark.skipif(
    not stage.shutil.which("zstd") or not stage.shutil.which("tar"),
    reason="GNU tar and zstd are required",
)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _make_archive(
    archives: Path, name: str, roots: dict[str, dict[str, bytes]]
) -> Path:
    source = archives.parent / f"source-{name}"
    source.mkdir()
    for root_name, files in roots.items():
        for relative, content in files.items():
            path = source / root_name / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
    archive = archives / name
    subprocess.run(
        [
            "tar",
            "--zstd",
            "-cf",
            str(archive),
            "-C",
            str(source),
            *sorted(roots),
        ],
        check=True,
    )
    return archive


def _make_raw_archive(
    archives: Path,
    name: str,
    members: list[tuple[tarfile.TarInfo, bytes | None]],
    *,
    trailing_zero_bytes: int = 0,
) -> Path:
    raw = archives.parent / f"{name}.tar"
    with tarfile.open(raw, "w") as archive:
        for info, content in members:
            archive.addfile(info, io.BytesIO(content) if content is not None else None)
    if trailing_zero_bytes:
        with raw.open("ab") as stream:
            stream.write(b"\0" * trailing_zero_bytes)
    destination = archives / name
    subprocess.run(
        ["zstd", "-q", "-f", str(raw), "-o", str(destination)], check=True
    )
    return destination


def test_audit_drains_zstd_after_tar_end_blocks(tmp_path: Path) -> None:
    archives = tmp_path / "archives"
    archives.mkdir()
    info = tarfile.TarInfo("seed_0000/model.urdf")
    info.size = 5
    archive = _make_raw_archive(
        archives,
        "padded.tar.zst",
        [(info, b"robot")],
        trailing_zero_bytes=2 * 1024 * 1024,
    )

    audit = stage._audit_archive(archive)

    assert audit.tar_member_count == 1
    assert audit.regular_file_count == 1
    assert audit.total_regular_bytes == 5


def test_resume_manifest_accepts_only_known_stream_drain_fix() -> None:
    requested = {
        "schema_version": stage.MANIFEST_SCHEMA,
        "source_code_sha256": "f" * 64,
        "archive_count": 1,
    }
    requested["manifest_content_sha256"] = stage._self_hash(
        requested, "manifest_content_sha256"
    )
    observed = dict(requested)
    observed["source_code_sha256"] = next(
        iter(stage.RESUME_COMPATIBLE_SOURCE_CODE_SHA256)
    )
    observed["manifest_content_sha256"] = stage._self_hash(
        observed, "manifest_content_sha256"
    )

    assert stage._resume_manifest_compatible(observed, requested) is True
    observed["source_code_sha256"] = "e" * 64
    observed["manifest_content_sha256"] = stage._self_hash(
        observed, "manifest_content_sha256"
    )
    assert stage._resume_manifest_compatible(observed, requested) is False


def _source_evaluation(
    tmp_path: Path, archive_rows: list[tuple[str, str, str]]
) -> Path:
    source = tmp_path / "evaluation"
    source.mkdir()
    database = source / "results.sqlite3"
    connection = sqlite3.connect(database)
    connection.executescript(
        """
        CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);
        CREATE TABLE assets(
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            category TEXT NOT NULL,
            joint_count INTEGER NOT NULL,
            row_sha256 TEXT NOT NULL,
            row_json TEXT NOT NULL
        );
        """
    )
    categories = {category for _, category, _ in archive_rows}
    meta = {
        "schema_version": stage.SOURCE_DB_SCHEMA,
        "asset_import_state": "COMPLETE",
        "selected_n": len(archive_rows),
        "selected_category_count": len(categories),
    }
    connection.executemany(
        "INSERT INTO meta VALUES(?, ?)",
        [(key, stage._canonical_text(value)) for key, value in meta.items()],
    )
    for ordinal, (archive_name, category, archive_sha256) in enumerate(archive_rows):
        row: dict[str, Any] = {
            "ordinal": ordinal,
            "asset_id": f"PV-A/{category}/seed_{ordinal:04d}",
            "category": category,
            "raw_category": category,
            "archive_name": archive_name,
            "archive_sha256": archive_sha256,
        }
        text = stage._canonical_text(row)
        connection.execute(
            "INSERT INTO assets VALUES(?, ?, ?, ?, ?, ?)",
            (
                ordinal,
                row["asset_id"],
                category,
                1,
                hashlib.sha256(text.encode()).hexdigest(),
                text,
            ),
        )
    connection.commit()
    connection.close()

    receipt = {
        "schema_version": stage.SOURCE_RECEIPT_SCHEMA,
        "N_eval": len(archive_rows),
        "eval_category_count": len(categories),
        "result_database": "results.sqlite3",
        "result_database_sha256": "d" * 64,
    }
    receipt["receipt_content_sha256"] = stage._self_hash(
        receipt, "receipt_content_sha256"
    )
    (source / "full_release_receipt.json").write_text(
        json.dumps(receipt), encoding="utf-8"
    )
    return source


def test_stages_grouped_archives_and_seals_resumable_closure(tmp_path: Path) -> None:
    archives = tmp_path / "archives"
    archives.mkdir()
    first = _make_archive(
        archives,
        "alpha_part00.tar.zst",
        {"seed_0000": {"model.urdf": b"alpha-0"}},
    )
    second = _make_archive(
        archives,
        "alpha_part01.tar.zst",
        {"seed_0001": {"model.urdf": b"alpha-1"}},
    )
    third = _make_archive(
        archives,
        "beta.tar.zst",
        {"seed_0000": {"model.urdf": b"beta"}},
    )
    source = _source_evaluation(
        tmp_path,
        [
            (first.name, "alpha", _sha(first)),
            (second.name, "alpha", _sha(second)),
            (third.name, "beta", _sha(third)),
        ],
    )
    output = tmp_path / "output"

    artifact_path = stage.stage_pva_archives(
        source, archives, output, workers=3
    )
    assert (output / "extracted/alpha/seed_0000/model.urdf").read_bytes() == b"alpha-0"
    assert (output / "extracted/alpha/seed_0001/model.urdf").read_bytes() == b"alpha-1"
    assert (output / "extracted/beta/seed_0000/model.urdf").read_bytes() == b"beta"
    assert len(list((output / "markers").glob("*.done.json"))) == 3
    assert not list((output / "markers").glob("*.inprogress.json"))
    summary = json.loads((output / "summary.json").read_text())
    assert summary["status"] == "COMPLETE"
    assert summary["archive_count"] == 3
    artifact = json.loads(artifact_path.read_text())
    assert artifact["artifact_manifest_content_sha256"] == stage._self_hash(
        artifact, "artifact_manifest_content_sha256"
    )

    before = artifact_path.read_bytes()
    assert stage.stage_pva_archives(
        source, archives, output, workers=1, resume=True
    ).read_bytes() == before


@pytest.mark.parametrize("member_kind", ["traversal", "symlink"])
def test_rejects_unsafe_members_before_extraction(
    tmp_path: Path, member_kind: str
) -> None:
    archives = tmp_path / "archives"
    archives.mkdir()
    if member_kind == "traversal":
        info = tarfile.TarInfo("../escaped.txt")
        info.size = 6
        content: bytes | None = b"escape"
    else:
        info = tarfile.TarInfo("seed_0000/link")
        info.type = tarfile.SYMTYPE
        info.linkname = "../../escaped.txt"
        content = None
    archive = _make_raw_archive(archives, "unsafe.tar.zst", [(info, content)])
    source = _source_evaluation(
        tmp_path, [(archive.name, "unsafe", _sha(archive))]
    )
    output = tmp_path / "output"

    with pytest.raises(ValueError, match="unsafe tar member|unsupported tar member"):
        stage.stage_pva_archives(source, archives, output, workers=1)
    assert not (tmp_path / "escaped.txt").exists()
    assert not list((output / "markers").glob("*.done.json"))


def test_inprogress_marker_recovers_after_extraction_interruption(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archives = tmp_path / "archives"
    archives.mkdir()
    archive = _make_archive(
        archives,
        "alpha.tar.zst",
        {"seed_0000": {"model.urdf": b"recoverable"}},
    )
    source = _source_evaluation(
        tmp_path, [(archive.name, "alpha", _sha(archive))]
    )
    output = tmp_path / "output"
    real_extract = stage._extract_archive
    calls = 0

    def interrupted(archive_path: Path, category_dir: Path) -> None:
        nonlocal calls
        calls += 1
        real_extract(archive_path, category_dir)
        raise RuntimeError("simulated interruption")

    monkeypatch.setattr(stage, "_extract_archive", interrupted)
    with pytest.raises(RuntimeError, match="simulated interruption"):
        stage.stage_pva_archives(source, archives, output, workers=1)
    assert (output / "markers/alpha.tar.zst.inprogress.json").is_file()
    assert (output / "extracted/alpha/seed_0000/model.urdf").is_file()

    monkeypatch.setattr(stage, "_extract_archive", real_extract)
    stage.stage_pva_archives(source, archives, output, workers=1, resume=True)
    assert calls == 1
    assert (output / "markers/alpha.tar.zst.done.json").is_file()
    assert not (output / "markers/alpha.tar.zst.inprogress.json").exists()


def test_resume_rejects_archive_changed_after_done_marker(tmp_path: Path) -> None:
    archives = tmp_path / "archives"
    archives.mkdir()
    archive = _make_archive(
        archives,
        "alpha.tar.zst",
        {"seed_0000": {"model.urdf": b"bound"}},
    )
    source = _source_evaluation(
        tmp_path, [(archive.name, "alpha", _sha(archive))]
    )
    output = tmp_path / "output"
    stage.stage_pva_archives(source, archives, output, workers=1)
    archive.write_bytes(archive.read_bytes() + b"tampered")

    with pytest.raises(ValueError, match="archive SHA256 mismatch"):
        stage.stage_pva_archives(source, archives, output, workers=1, resume=True)


def test_source_mapping_rejects_archive_category_conflict(tmp_path: Path) -> None:
    archives = tmp_path / "archives"
    archives.mkdir()
    archive = _make_archive(
        archives,
        "shared.tar.zst",
        {"seed_0000": {"model.urdf": b"shared"}},
    )
    source = _source_evaluation(
        tmp_path,
        [
            (archive.name, "alpha", _sha(archive)),
            (archive.name, "beta", _sha(archive)),
        ],
    )
    with pytest.raises(ValueError, match="conflicting source binding"):
        stage.stage_pva_archives(source, archives, tmp_path / "output", workers=1)
