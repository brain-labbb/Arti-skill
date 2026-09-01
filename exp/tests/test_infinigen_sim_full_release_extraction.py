from __future__ import annotations

import io
import sys
import tarfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import extract_infinigen_sim_full_release as extractor


def _tar(path: Path, member: tarfile.TarInfo, payload: bytes = b"x") -> None:
    with tarfile.open(path, "w:gz") as archive:
        archive.addfile(member, io.BytesIO(payload) if member.isfile() else None)


def test_secure_extraction_rejects_traversal(tmp_path: Path) -> None:
    archive = tmp_path / "bad.tar.gz"
    member = tarfile.TarInfo("../escape.txt"); member.size = 1
    _tar(archive, member)
    with pytest.raises(ValueError, match="traversal"):
        extractor.extract_archives_securely(tmp_path, tmp_path / "out")


def test_secure_extraction_rejects_symlink_and_device(tmp_path: Path) -> None:
    archive = tmp_path / "bad.tar.gz"
    member = tarfile.TarInfo("link"); member.type = tarfile.SYMTYPE; member.linkname = "target"
    _tar(archive, member)
    with pytest.raises(ValueError, match="link"):
        extractor.extract_archives_securely(tmp_path, tmp_path / "out")


def test_secure_extraction_extracts_normal_archive_and_returns_hashes(tmp_path: Path) -> None:
    archive = tmp_path / "one.tar.gz"
    member = tarfile.TarInfo("asset/model.urdf"); member.size = len(b"<robot/>")
    _tar(archive, member, b"<robot/>")
    result = extractor.extract_archives_securely(tmp_path, tmp_path / "out")
    assert (tmp_path / "out/one/asset/model.urdf").read_text() == "<robot/>"
    assert result["archives"][0]["sha256"]
