from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import sqlite3

import pytest


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "check_pva_table4_mimic_aware_full_release.py"
)
SPEC = importlib.util.spec_from_file_location("pva_table4_checker", SCRIPT)
assert SPEC and SPEC.loader
checker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _sealed_database(tmp_path: Path) -> Path:
    database = tmp_path / "results.sqlite3"
    connection = sqlite3.connect(database)
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("CREATE TABLE evidence(value INTEGER NOT NULL)")
    connection.execute("INSERT INTO evidence VALUES (7)")
    connection.commit()
    connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    connection.close()
    return database


def test_immutable_connection_does_not_create_or_change_sidecars(tmp_path: Path) -> None:
    database = _sealed_database(tmp_path)
    shm = Path(f"{database}-shm")
    shm.write_bytes(b"stale volatile index")
    before = {
        path.name: (path.stat().st_size, _sha256(path), path.stat().st_mtime_ns)
        for path in tmp_path.iterdir()
    }

    connection = checker._connect_sealed_immutable(database, _sha256(database))
    try:
        assert connection.execute("SELECT value FROM evidence").fetchone() == (7,)
        assert connection.execute("PRAGMA query_only").fetchone() == (1,)
    finally:
        connection.close()

    after = {
        path.name: (path.stat().st_size, _sha256(path), path.stat().st_mtime_ns)
        for path in tmp_path.iterdir()
    }
    assert after == before


def test_immutable_uri_and_query_only_are_explicit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    database = _sealed_database(tmp_path)
    observed: dict[str, object] = {}
    real_connect = checker.sqlite3.connect

    def capture(database_uri: str, **kwargs):
        observed.update(uri=database_uri, uri_flag=kwargs.get("uri"))
        return real_connect(database_uri, **kwargs)

    monkeypatch.setattr(checker.sqlite3, "connect", capture)
    connection = checker._connect_sealed_immutable(database, _sha256(database))
    connection.close()
    assert str(observed["uri"]).endswith("?mode=ro&immutable=1")
    assert observed["uri_flag"] is True


@pytest.mark.parametrize("suffix", ["-wal", "-journal"])
def test_nonempty_durable_sidecar_is_rejected(tmp_path: Path, suffix: str) -> None:
    database = _sealed_database(tmp_path)
    Path(f"{database}{suffix}").write_bytes(b"uncheckpointed")
    with pytest.raises(checker.CheckError, match="sidecar is non-empty"):
        checker._connect_sealed_immutable(database, _sha256(database))


def test_database_hash_mismatch_is_rejected(tmp_path: Path) -> None:
    database = _sealed_database(tmp_path)
    with pytest.raises(checker.CheckError, match="SHA-256 mismatch"):
        checker._connect_sealed_immutable(database, "0" * 64)


@pytest.mark.parametrize("suffix", ["", "-wal", "-journal", "-shm"])
def test_symlinked_database_or_sidecar_is_rejected(
    tmp_path: Path, suffix: str
) -> None:
    database = _sealed_database(tmp_path)
    target = tmp_path / "external"
    target.write_bytes(b"")
    candidate = Path(f"{database}{suffix}")
    if suffix:
        candidate.unlink(missing_ok=True)
    else:
        database.unlink()
    candidate.symlink_to(target)
    with pytest.raises(checker.CheckError, match="non-symlink|unsafe"):
        checker._connect_sealed_immutable(database, _sha256(target))
