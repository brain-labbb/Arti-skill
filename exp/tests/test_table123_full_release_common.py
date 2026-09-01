from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import table123_full_release_common as common


def _row(root: Path, asset_id: str, joints: list[str] | None = None) -> dict:
    package = root / asset_id
    package.mkdir(parents=True, exist_ok=True)
    urdf = package / "robot.urdf"
    urdf.write_text(f"<robot name='{asset_id}'/>\n", encoding="utf-8")
    return {
        "asset_id": asset_id,
        "source_path": str(package),
        "source_relative_path": asset_id,
        "primary_urdf_path": str(urdf),
        "primary_urdf_relative_path": f"{asset_id}/robot.urdf",
        "non_fixed_joints": joints or [],
    }


def test_freeze_roster_sorts_rows_and_sets_dynamic_denominators(tmp_path: Path) -> None:
    rows = [_row(tmp_path / "src", "b", ["j1"]), _row(tmp_path / "src", "a", ["j1", "j2"])]
    manifest = common.freeze_roster(
        reversed(rows),
        tmp_path / "full_release_manifest.json",
        dataset="Fixture",
        source_bindings=[{"name": "fixture", "sha256": "a" * 64}],
    )

    assert [row["asset_id"] for row in manifest["rows"]] == ["a", "b"]
    assert [row["ordinal"] for row in manifest["rows"]] == [0, 1]
    assert manifest["N_eval"] == 2
    assert manifest["J_eval"] == 3
    assert manifest["manifest_content_sha256"] == common.canonical_sha256(
        {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    )


def test_freeze_roster_rejects_duplicate_asset_ids(tmp_path: Path) -> None:
    rows = [_row(tmp_path / "src", "same"), _row(tmp_path / "src", "same")]
    with pytest.raises(ValueError, match="duplicate"):
        common.freeze_roster(rows, tmp_path / "manifest.json", dataset="Fixture", source_bindings=[])


def test_roster_rejects_path_escape_and_symlink(tmp_path: Path) -> None:
    source = tmp_path / "src"
    source.mkdir()
    outside = tmp_path / "outside.urdf"
    outside.write_text("<robot/>", encoding="utf-8")
    escaping = _row(source, "escape")
    escaping["primary_urdf_relative_path"] = "../outside.urdf"
    with pytest.raises(ValueError, match="relative path"):
        common.freeze_roster([escaping], tmp_path / "escape.json", dataset="Fixture", source_bindings=[])

    target = source / "target"
    target.mkdir()
    (target / "robot.urdf").write_text("<robot/>", encoding="utf-8")
    link = source / "link"
    link.symlink_to(target, target_is_directory=True)
    linked = _row(source, "linked")
    linked["source_path"] = str(link)
    linked["primary_urdf_path"] = str(link / "robot.urdf")
    with pytest.raises(ValueError, match="symlink"):
        common.freeze_roster([linked], tmp_path / "linked.json", dataset="Fixture", source_bindings=[])


def test_verify_roster_detects_primary_urdf_hash_drift(tmp_path: Path) -> None:
    row = _row(tmp_path / "src", "asset")
    path = tmp_path / "full_release_manifest.json"
    common.freeze_roster([row], path, dataset="Fixture", source_bindings=[])
    Path(row["primary_urdf_path"]).write_text("changed", encoding="utf-8")
    with pytest.raises(ValueError, match="URDF hash"):
        common.verify_roster(path)


def test_verify_roster_checks_hashed_manifest_source_binding(tmp_path: Path) -> None:
    row = _row(tmp_path / "src", "asset")
    receipt = tmp_path / "source-receipt.json"
    receipt.write_text("{\"revision\":\"r1\"}\n", encoding="utf-8")
    path = tmp_path / "full_release_manifest.json"
    common.freeze_roster(
        [row],
        path,
        dataset="Fixture",
        source_bindings=[
            {
                "name": "receipt",
                "path": str(receipt),
                "sha256": common.sha256_file(receipt),
            }
        ],
    )
    common.load_roster(path, verify_sources=True)
    receipt.write_text("{\"revision\":\"r2\"}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="source binding"):
        common.load_roster(path, verify_sources=True)


def test_checkpoint_and_artifact_manifest_are_atomic_and_verified(tmp_path: Path) -> None:
    checkpoint = tmp_path / "checkpoint.json"
    common.write_checkpoint(checkpoint, {"next_ordinal": 2})
    saved = json.loads(checkpoint.read_text(encoding="utf-8"))
    assert saved["next_ordinal"] == 2
    assert saved["checkpoint_content_sha256"] == common.canonical_sha256(
        {"next_ordinal": 2}
    )

    payload = tmp_path / "records.jsonl"
    payload.write_text("{}\n", encoding="utf-8")
    artifact = {
        "schema_version": "table123_artifact_manifest_v1",
        "artifacts": [
            {
                "path": payload.name,
                "size": payload.stat().st_size,
                "sha256": common.sha256_file(payload),
            }
        ],
    }
    artifact["artifact_manifest_content_sha256"] = common.canonical_sha256(artifact)
    (tmp_path / "artifact_manifest.json").write_text(
        json.dumps(artifact, sort_keys=True), encoding="utf-8"
    )
    common.verify_artifacts(tmp_path)
    payload.write_text("xx\n", encoding="utf-8")
    with pytest.raises(ValueError, match="hash"):
        common.verify_artifacts(tmp_path)
