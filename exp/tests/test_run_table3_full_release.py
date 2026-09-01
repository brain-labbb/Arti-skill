from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import run_table3_full_release as runner
import table123_full_release_common as common


def _urdf(root: Path, name: str, *, movable: bool = True, joint_type: str = "revolute") -> Path:
    package = root / name
    package.mkdir(parents=True, exist_ok=True)
    if movable:
        body = (
            "<robot name='fixture'><link name='base'/><link name='tip'/>"
            f"<joint name='joint' type='{joint_type}'><parent link='base'/>"
            "<child link='tip'/><axis xyz='0 0 1'/><limit lower='-1' upper='1'/>"
            "</joint></robot>"
        )
    else:
        body = "<robot name='fixture'><link name='base'/></robot>"
    path = package / "model.urdf"
    path.write_text(body, encoding="utf-8")
    return path


def _manifest(tmp_path: Path, rows: list[dict]) -> Path:
    manifest_path = tmp_path / "full_release_manifest.json"
    common.freeze_roster(rows, manifest_path, dataset="Fixture", source_bindings=[])
    return manifest_path


def _row(path: Path, asset_id: str, joints: list[str]) -> dict:
    return {
        "asset_id": asset_id,
        "source_path": str(path.parent),
        "source_relative_path": asset_id,
        "primary_urdf_path": str(path),
        "primary_urdf_relative_path": f"{asset_id}/model.urdf",
        "non_fixed_joints": joints,
    }


def test_evaluate_row_uses_exact_k21_and_preserves_identity(tmp_path: Path) -> None:
    path = _urdf(tmp_path, "hinge")
    record = runner.evaluate_row(_row(path, "hinge", ["joint"]))
    assert record["asset_id"] == "hinge"
    assert record["sample_count_expected"] == 21 or record["joints"][0]["sample_count_expected"] == 21
    assert record["ordinal"] is None or isinstance(record["ordinal"], int)


def test_aggregate_uses_dynamic_asset_and_joint_denominators(tmp_path: Path) -> None:
    fixed = _urdf(tmp_path, "fixed", movable=False)
    hinge = _urdf(tmp_path, "hinge")
    rows = [_row(fixed, "fixed", []), _row(hinge, "hinge", ["joint"])]
    manifest_path = _manifest(tmp_path, rows)
    manifest = common.load_roster(manifest_path)
    records = [runner.evaluate_row(row) for row in manifest["rows"]]
    summary = runner.aggregate_full_release(records, manifest)
    assert summary["n_eval"] == 2
    assert summary["j_eval"] == 1
    assert summary["status_counts"]


def test_unsupported_joint_is_explicit_fail_closed(tmp_path: Path) -> None:
    path = _urdf(tmp_path, "floating", joint_type="floating")
    record = runner.evaluate_row(_row(path, "floating", ["joint"]))
    assert record["strict_kinematic_pass"] is False
    assert record["status"] in {"error", "completed"}
    assert record.get("error") or any(not joint.get("joint_level_pass", False) for joint in record.get("joints", []))


def test_physx_unsupported_joint_is_excluded_from_supported_denominator(tmp_path: Path) -> None:
    package = tmp_path / "physx"
    package.mkdir()
    path = package / "model.urdf"
    path.write_text(
        "<robot name='fixture'><link name='base'/><link name='tip'/><link name='lid'/>"
        "<joint name='hinge' type='revolute'><parent link='base'/><child link='tip'/>"
        "<axis xyz='0 0 1'/><limit lower='-1' upper='1'/></joint>"
        "<joint name='floating' type='floating'><parent link='tip'/><child link='lid'/>"
        "</joint></robot>",
        encoding="utf-8",
    )
    row = _row(path, "physx", ["hinge"])
    row.update({
        "joint_count": 1,
        "unsupported_joint_count": 1,
        "declared_joint_count_all": 2,
        "primary_urdf_sha256": common.sha256_file(path),
    })
    record = runner.evaluate_row(row)
    assert record["status"] == "completed"
    assert record["declared_joint_count"] == 1
    assert record["unsupported_joint_count"] == 1


def test_runner_rejects_legacy_n800_manifest(tmp_path: Path) -> None:
    legacy = tmp_path / "legacy.json"
    legacy.write_text(json.dumps({"N_eval": 800, "rows": []}), encoding="utf-8")
    with pytest.raises((ValueError, RuntimeError)):
        runner.run_full_release(legacy, tmp_path / "out", workers=1, timeout_seconds=5)


def test_run_publishes_ordered_records_and_checkpoint(tmp_path: Path) -> None:
    first = _urdf(tmp_path, "z-last")
    second = _urdf(tmp_path, "a-first", movable=False)
    manifest_path = _manifest(
        tmp_path,
        [_row(first, "z-last", ["joint"]), _row(second, "a-first", [])],
    )
    output = runner.run_full_release(manifest_path, tmp_path / "out", workers=2, timeout_seconds=5)
    records = [json.loads(line) for line in (output / "records.jsonl").read_text().splitlines()]
    assert [row["asset_id"] for row in records] == ["a-first", "z-last"]
    assert all(row["status"] == "completed" for row in records)
    checkpoint = json.loads((output / "checkpoint.json").read_text())
    assert checkpoint["state"] == "complete"
    common.verify_artifacts(output)
