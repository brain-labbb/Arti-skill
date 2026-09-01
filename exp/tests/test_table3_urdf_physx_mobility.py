from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_table3_urdf_physx_mobility.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("run_table3_urdf_physx_mobility", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


OBJ_TEXT = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"


def _physx_urdf(dataset_id: int, *, extra_joints: str = "") -> str:
    return f"""<robot name="scene">
  <link name="l_world"/>
  <link name="l_0">
    <visual><geometry><mesh filename="./../partseg/{dataset_id}/objs/{dataset_id}.obj" scale="1 1 1"/></geometry></visual>
  </link>
  <link name="l_1"/>
  <joint name="j_revolute" type="revolute">
    <parent link="l_world"/><child link="l_0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j_prismatic" type="prismatic">
    <parent link="l_0"/><child link="l_1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="0.5" effort="1" velocity="1"/>
  </joint>
{extra_joints}</robot>
"""


def _write_dataset(root: Path, dataset_id: int, *, urdf_text: str, obj_text: str = OBJ_TEXT) -> dict:
    (root / "urdf").mkdir(parents=True, exist_ok=True)
    (root / "finaljson").mkdir(parents=True, exist_ok=True)
    objs = root / "partseg" / str(dataset_id) / "objs"
    objs.mkdir(parents=True, exist_ok=True)
    urdf_path = root / "urdf" / f"{dataset_id}.urdf"
    urdf_path.write_text(urdf_text, encoding="utf-8")
    obj_path = objs / f"{dataset_id}.obj"
    obj_path.write_text(obj_text, encoding="utf-8")
    (root / "finaljson" / f"{dataset_id}.json").write_text(
        json.dumps({"category": "Fixture"}), encoding="utf-8"
    )
    return {
        "dataset_id": dataset_id,
        "rank": 1,
        "rank_sha256": "rank",
        "category": "Fixture",
        "urdf_relative_path": f"urdf/{dataset_id}.urdf",
        "urdf_sha256": _sha256_file(urdf_path),
        "finaljson_sha256": _sha256_file(root / "finaljson" / f"{dataset_id}.json"),
        "resource_sha256": "resources",
        "resources": [
            {
                "relative_path": f"partseg/{dataset_id}/objs/{dataset_id}.obj",
                "sha256": _sha256_file(obj_path),
            }
        ],
        "xml_counts": {"links": 3, "joints": 2, "fixed_joints": 0, "movable_joints": 2, "collision_elements": 0},
    }


def _build_job(runner, tmp_path: Path, row: dict, name: str) -> dict:
    staged = runner.build_eval_package(tmp_path / "dataset", row, tmp_path / f"staging_{name}")
    entry = staged
    return {
        "asset_key": str(row["dataset_id"]),
        "dataset_id": row["dataset_id"],
        "selection_index": 0,
        "rank": 1,
        "rank_sha256": "rank",
        "category": "Fixture",
        "eval_package": str(entry["eval_package"]),
        "eval_urdf": str(entry["eval_urdf"]),
        "eval_urdf_sha256": entry["eval_urdf_sha256"],
        "eval_package_binding": entry["eval_package_binding"],
        "urdf_sha256": row["urdf_sha256"],
        "manifest_row_sha256": "row",
        "manifest_content_sha256": "manifest",
        "declared_joint_count_hint": row["xml_counts"]["movable_joints"],
        "samples": 21,
    }, entry


def test_build_eval_package_relativizes_and_preserves_declarations(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    row = _write_dataset(root, 10, urdf_text=_physx_urdf(10))

    entry = runner.build_eval_package(root, row, tmp_path / "staging")

    assert entry["reference_map"] == [
        {"original": "./../partseg/10/objs/10.obj", "relativized": "partseg/10/objs/10.obj"}
    ]
    rewritten = entry["eval_urdf"]
    assert rewritten.is_file()
    record = runner.CORE.evaluate_urdf(rewritten, "10", samples=21, declared_joint_count_hint=2)
    assert record["status"] == "completed"
    assert record["kinematic_scale_status"] == "COMPLETE"
    assert record["tree_valid"] is True
    assert [joint["joint_level_pass"] for joint in record["joints"]] == [True, True]
    assert record["strict_kinematic_pass"] is True


def test_build_eval_package_rejects_byte_drift(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    row = _write_dataset(root, 10, urdf_text=_physx_urdf(10))
    (root / "partseg/10/objs/10.obj").write_text("v 9 9 9\n", encoding="utf-8")

    with pytest.raises(ValueError, match="staged byte mismatch"):
        runner.build_eval_package(root, row, tmp_path / "staging")


def test_child_job_end_to_end_all_pass(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    row = _write_dataset(root, 10, urdf_text=_physx_urdf(10))
    job, _entry = _build_job(runner, tmp_path, row, "a")

    job_root = tmp_path / "jobdir" / "job_000000"
    job_root.mkdir(parents=True)
    (job_root / "job.json").write_text(json.dumps(job, sort_keys=True), encoding="utf-8")
    completed = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--internal-job",
            str(job_root / "job.json"),
            "--internal-result",
            str(job_root / "result.json"),
        ],
        capture_output=True,
        text=True,
        timeout=180,
    )
    assert completed.returncode == 0, completed.stderr

    record = json.loads((job_root / "result.json").read_text(encoding="utf-8"))
    assert record["status"] == "completed"
    assert record["result_origin"] == "child_attested"
    assert record["strict_kinematic_pass"] is True
    assert record["declared_joint_count"] == 2
    assert record["category"] == "Fixture"
    assert record["eval_urdf_sha256"] == job["eval_urdf_sha256"]
    assert record["manifest_content_sha256"] == "manifest"
    runner.validate_record(job, record)


def test_floating_joint_asset_fails_closed(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    extra = """  <link name="l_f"/>
  <joint name="j_floating" type="floating">
    <parent link="l_1"/><child link="l_f"/>
  </joint>
"""
    row = _write_dataset(root, 20, urdf_text=_physx_urdf(20, extra_joints=extra))
    row["xml_counts"] = {**row["xml_counts"], "links": 4, "joints": 3, "movable_joints": 3}

    entry = runner.build_eval_package(root, row, tmp_path / "staging")
    record = runner.CORE.evaluate_urdf(entry["eval_urdf"], "20", samples=21, declared_joint_count_hint=3)

    assert record["status"] == "error"
    assert "unsupported joint type" in record["error"]
    assert record["declared_joint_count"] == 3
    assert len(record["joints"]) == 3
    assert all(joint["joint_level_pass"] is False for joint in record["joints"])
    assert record["strict_kinematic_pass"] is False


def test_degenerate_motion_fails_joint(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    urdf_text = _physx_urdf(30).replace('upper="1"', 'upper="0.000000000001"', 1)
    row = _write_dataset(root, 30, urdf_text=urdf_text)

    entry = runner.build_eval_package(root, row, tmp_path / "staging")
    record = runner.CORE.evaluate_urdf(entry["eval_urdf"], "30", samples=21, declared_joint_count_hint=2)

    assert record["status"] == "completed"
    revolute = record["joints"][0]
    assert revolute["non_degenerate_motion_pass"] is False
    assert revolute["joint_level_pass"] is False
    assert record["joints"][1]["joint_level_pass"] is True
    assert record["strict_kinematic_pass"] is False


def test_harness_timeout_is_fail_closed(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    row = _write_dataset(root, 40, urdf_text=_physx_urdf(40))
    job, _entry = _build_job(runner, tmp_path, row, "t")
    job["internal_test_action"] = "sleep"
    job["sleep"] = 10.0

    records = runner.execute_jobs(
        [job],
        worker_scratch=tmp_path / "scratch",
        timeout_seconds=0.7,
        max_workers=1,
    )

    assert len(records) == 1
    record = records[0]
    assert record["status"] == "timeout"
    assert record["result_origin"] == "parent_synthesized"
    assert record["strict_kinematic_pass"] is False
    assert len(record["joints"]) == record["declared_joint_count"] == 2
    assert all(joint["joint_level_pass"] is False for joint in record["joints"])
    assert record["worker_evidence"]["timed_out"] is True


def test_harness_end_to_end_aggregation(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    row_a = _write_dataset(root, 10, urdf_text=_physx_urdf(10))
    urdf_b = _physx_urdf(20).replace('upper="1"', 'upper="0.000000000001"', 1)
    row_b = _write_dataset(root, 20, urdf_text=urdf_b)

    entries = {}
    jobs = []
    for row in (row_a, row_b):
        entry = runner.build_eval_package(root, row, tmp_path / f"staging_{row['dataset_id']}")
        entries[row["dataset_id"]] = entry
        jobs.append(
            {
                "asset_key": str(row["dataset_id"]),
                "dataset_id": row["dataset_id"],
                "selection_index": row["dataset_id"],
                "rank": 1,
                "rank_sha256": "rank",
                "category": "Fixture",
                "eval_package": str(entry["eval_package"]),
                "eval_urdf": str(entry["eval_urdf"]),
                "eval_urdf_sha256": entry["eval_urdf_sha256"],
                "eval_package_binding": entry["eval_package_binding"],
                "urdf_sha256": row["urdf_sha256"],
                "manifest_row_sha256": "row",
                "manifest_content_sha256": "manifest",
                "declared_joint_count_hint": 2,
                "samples": 21,
            }
        )

    records = runner.execute_jobs(
        jobs,
        worker_scratch=tmp_path / "scratch",
        timeout_seconds=180.0,
        max_workers=2,
    )
    assert [record["status"] for record in records] == ["completed", "completed"]

    summary = runner.CORE.aggregate_records(records, 2)
    assert summary["n_eval"] == 2
    assert summary["j_eval"] == 4
    assert summary["metrics"]["valid_range"]["passed"] == 4
    assert summary["metrics"]["joint_sweep_success"]["passed"] == 4
    assert summary["metrics"]["non_degenerate_motion"]["passed"] == 3
    assert summary["metrics"]["subtree_consistency"]["passed"] == 4
    assert summary["metrics"]["joint_level_pass"]["passed"] == 3
    assert summary["metrics"]["fk_roundtrip_error"]["status"] == "COMPLETE"
    assert summary["metrics"]["fk_roundtrip_error"]["measured_joint_count"] == 4
    assert summary["metrics"]["strict_kinematic_pass"]["passed"] == 1
    assert summary["category_macro"]["state"] == "evaluated"
