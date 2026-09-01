from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "run_table4_full_release.py"
spec = importlib.util.spec_from_file_location("table4_full_release_runner", SCRIPT)
assert spec and spec.loader
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _urdf(*, collision: bool) -> str:
    collision_xml = (
        "<collision><geometry><box size=\"0.2 0.2 0.2\"/></geometry></collision>"
        if collision
        else ""
    )
    return f'''<?xml version="1.0"?>
<robot name="fixture">
  <link name="base">{collision_xml}</link>
  <link name="link1">{collision_xml}</link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
'''


def _manifest(tmp_path: Path, *, collision: bool) -> Path:
    package = tmp_path / ("with_collision" if collision else "without_collision")
    package.mkdir()
    urdf = package / "model.urdf"
    urdf.write_text(_urdf(collision=collision), encoding="utf-8")
    row = {
        "asset_id": "fixture/0",
        "category": "fixture",
        "joint_count": 1,
        "non_fixed_joints": [{"name": "hinge", "type": "revolute"}],
        "source_path": str(package),
        "primary_urdf_path": str(urdf),
        "primary_urdf_relative_path": "model.urdf",
        "primary_urdf_sha256": _sha(urdf),
    }
    manifest = {
        "schema_version": runner.ROSTER_SCHEMA_VERSION,
        "dataset": "fixture",
        "N_eval": 1,
        "J_eval": 1,
        "rows": [row],
    }
    manifest["manifest_content_sha256"] = runner.canonical_sha256(
        {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    )
    path = tmp_path / ("manifest_collision.json" if collision else "manifest_blocked.json")
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def test_build_jobs_keeps_absolute_source_binding_and_frozen_joint_count(tmp_path: Path) -> None:
    path = _manifest(tmp_path, collision=True)
    manifest, jobs = runner.load_roster(path, dataset="fixture")
    assert manifest["N_eval"] == 1
    assert jobs[0]["asset_id"] == "fixture/0"
    assert Path(jobs[0]["primary_urdf_path"]).is_absolute()
    assert jobs[0]["expected_movable_joints"] == 1


def test_no_native_collision_is_blocked_and_never_vacuous_pass(tmp_path: Path) -> None:
    path = _manifest(tmp_path, collision=False)
    _manifest_obj, jobs = runner.load_roster(path, dataset="fixture")
    result = runner.evaluate_job(jobs[0])
    assert result["status"] == "blocked"
    assert result["collision_metric_status"] == runner.BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT
    assert result["strict_collision_pass"] is None
    assert result["state_records"] == []
    assert result["unexecuted_state_count"] == 86
    assert "execution_input_sha256" not in result

    summary = runner.aggregate_records([result], 1, 1)
    assert summary["status"] == "BLOCKED"
    assert summary["metrics"]["rest_all_pair_cf"]["status"] == "N/E"
    assert summary["metrics"]["collision_state_rate"]["status"] == "N/E"


@pytest.mark.parametrize(
    "reference",
    (
        "file:///tmp/mesh.obj",
        "/tmp/mesh.obj",
        "C:/tmp/mesh.obj",
    ),
)
def test_collision_inventory_rejects_absolute_and_file_uri_meshes(
    tmp_path: Path, reference: str
) -> None:
    package = tmp_path / "package"
    package.mkdir()
    urdf = package / "model.urdf"
    urdf.write_text(
        f'''<robot name="unsafe">
  <link name="base">
    <collision><geometry><mesh filename="{reference}"/></geometry></collision>
  </link>
</robot>
''',
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="absolute/file URI"):
        runner.collision_inventory(package, urdf)


def test_collision_fixture_runs_complete_frozen_state_plan(tmp_path: Path) -> None:
    path = _manifest(tmp_path, collision=True)
    _manifest_obj, jobs = runner.load_roster(path, dataset="fixture")
    result = runner.evaluate_job(jobs[0])
    assert result["status"] == "completed"
    assert result["load_success"] is True
    assert result["collision_metric_status"] == "MEASURED"
    assert result["rest_state_executed"] == 1
    assert result["single_state_executed"] == 21
    assert result["sobol_state_executed"] == 64
    assert len(result["state_records"]) == 86
    assert result["measurement_complete"] is True
    assert result["max_penetration_normalized"] is not None


def test_run_dataset_emits_resumable_receipt_and_combined_receipt(tmp_path: Path) -> None:
    path = _manifest(tmp_path, collision=False)
    output = tmp_path / "run"
    runner.run_dataset(path, output, dataset="fixture", workers=1, timeout_seconds=30)
    for name in (
        "manifest.json", "records.jsonl", "state_records.jsonl", "asset_records.json",
        "summary.json", "summary.md", "checkpoint.json", "artifact_manifest.json",
    ):
        assert (output / name).is_file(), name
    # A second invocation must be an explicit resume and preserve the terminal
    # blocked record rather than rerunning it.
    runner.run_dataset(path, output, dataset="fixture", workers=1, timeout_seconds=30, resume=True)
    receipt = runner.write_combined_receipt({"fixture": output}, tmp_path)
    payload = json.loads(receipt.read_text(encoding="utf-8"))
    assert payload["schema_version"] == runner.RECEIPT_SCHEMA_VERSION
    assert payload["methods"][0]["N_eval"] == 1


def test_complete_resume_reuses_frozen_protocol_after_source_update(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A sealed run remains resumable when the live protocol is later edited."""
    path = _manifest(tmp_path, collision=False)
    output = tmp_path / "run"
    runner.run_dataset(path, output, dataset="fixture", workers=1, timeout_seconds=30)

    protocol_path = (runner.EXP_ROOT / "URDF-Sim-Ready-Automatic-Evaluation.md").resolve()
    original_hash = runner.sha256_file

    def changed_protocol_hash(value: Path) -> str:
        if Path(value).resolve() == protocol_path:
            return "f" * 64
        return original_hash(value)

    monkeypatch.setattr(runner, "sha256_file", changed_protocol_hash)
    runner.run_dataset(path, output, dataset="fixture", workers=1, timeout_seconds=30, resume=True)


def test_resume_accepts_provenance_superset_and_path_aliases() -> None:
    sealed = [
        {"name": "extracted_root", "path": "/data/extracted", "sha256": "a" * 64},
        {"name": "archive_receipt", "path": "/data/receipt.json", "sha256": "b" * 64},
    ]
    requested = [
        # The roster used the historical Infinigen-Sim alias for this path.
        {"name": "Infinigen-Sim", "path": "/data/extracted"},
        {"name": "archive_receipt", "path": "/data/receipt.json", "sha256": "b" * 64},
    ]
    assert runner._source_bindings_compatible(sealed, requested)
    assert not runner._source_bindings_compatible(
        sealed,
        [{"name": "archive_receipt", "path": "/other/receipt.json", "sha256": "b" * 64}],
    )
    assert not runner._source_bindings_compatible(
        sealed,
        [{"name": "archive_receipt", "path": "/data/receipt.json", "sha256": "c" * 64}],
    )


def test_manifest_rejects_joint_count_drift(tmp_path: Path) -> None:
    path = _manifest(tmp_path, collision=True)
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["rows"][0]["joint_count"] = 2
    payload["manifest_content_sha256"] = runner.canonical_sha256(
        {key: value for key, value in payload.items() if key != "manifest_content_sha256"}
    )
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="J_eval"):
        runner.load_roster(path, dataset="fixture")


def test_zero_dof_repair_rebuilds_summary_without_rerunning_states(tmp_path: Path) -> None:
    """The repair utility changes only vacuous asset-level booleans."""
    import importlib.util

    repair_path = SCRIPT.with_name("repair_table4_zero_dof.py")
    repair_spec = importlib.util.spec_from_file_location("table4_zero_dof_repair", repair_path)
    assert repair_spec and repair_spec.loader
    repair_module = importlib.util.module_from_spec(repair_spec)
    repair_spec.loader.exec_module(repair_module)

    path = _manifest(tmp_path, collision=True)
    output = tmp_path / "run"
    runner.run_dataset(path, output, dataset="fixture", workers=1, timeout_seconds=30)
    rows = [json.loads(line) for line in (output / "records.jsonl").read_text().splitlines()]
    rows[0]["movable_dof_count"] = 0
    rows[0]["expected_movable_joints"] = 0
    rows[0]["single_state_expected"] = 0
    rows[0]["sobol_state_expected"] = 0
    rows[0]["single_joint_sweep_cf"] = True
    rows[0]["multi_joint_sobol_cf"] = True
    rows[0]["strict_collision_pass"] = True
    (output / "records.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    (output / "asset_records.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    (output / "asset_records.json").write_text(json.dumps(rows))
    result = repair_module.repair(output)
    assert result["changed_count"] == 1
    repaired = json.loads((output / "records.jsonl").read_text().splitlines()[0])
    assert repaired["single_joint_sweep_cf"] is True
    assert repaired["multi_joint_sobol_cf"] is False
    assert repaired["strict_collision_pass"] is False
    assert json.loads((output / "summary.json").read_text())["repair"]["changed_count"] == 1
