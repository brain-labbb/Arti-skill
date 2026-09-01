from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_table4_urdf_physx_mobility.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("run_table4_urdf_physx_mobility", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


ZERO_COLLISION_URDF = """<robot name="scene">
  <link name="l_world">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="1.0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="l_0">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="1.0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <visual><geometry><mesh filename="./../partseg/10/objs/10.obj" scale="1 1 1"/></geometry></visual>
  </link>
  <joint name="j_0" type="revolute">
    <parent link="l_world"/><child link="l_0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""

ZERO_DOF_URDF = """<robot name="scene">
  <link name="l_world">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="1.0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="l_0">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="1.0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <joint name="j_f" type="fixed">
    <parent link="l_world"/><child link="l_0"/>
  </joint>
</robot>
"""

PENETRATING_URDF = """<robot name="pen">
  <link name="l_world">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="1.0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <link name="l_mid">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="1.0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="l_leaf">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="1.0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <joint name="j0" type="fixed">
    <parent link="l_world"/><child link="l_mid"/>
  </joint>
  <joint name="j1" type="revolute">
    <parent link="l_mid"/><child link="l_leaf"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def _write_dataset(root: Path, dataset_id: int, *, urdf_text: str, with_obj: bool = False) -> dict:
    (root / "urdf").mkdir(parents=True, exist_ok=True)
    urdf_path = root / "urdf" / f"{dataset_id}.urdf"
    urdf_path.write_text(urdf_text, encoding="utf-8")
    resources = []
    if with_obj:
        objs = root / "partseg" / str(dataset_id) / "objs"
        objs.mkdir(parents=True, exist_ok=True)
        obj_path = objs / f"{dataset_id}.obj"
        obj_path.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")
        resources.append({"relative_path": f"partseg/{dataset_id}/objs/{dataset_id}.obj", "sha256": _sha256_file(obj_path)})
    movable = sum(1 for line in urdf_text.splitlines() if 'type="revolute"' in line or 'type="prismatic"' in line)
    return {
        "dataset_id": dataset_id,
        "rank": 1,
        "rank_sha256": "rank",
        "category": "Fixture",
        "bounding_box_diagonal": 2.0,
        "urdf_relative_path": f"urdf/{dataset_id}.urdf",
        "urdf_sha256": _sha256_file(urdf_path),
        "finaljson_sha256": "finaljson",
        "resource_sha256": "resources",
        "resources": resources,
        "xml_counts": {
            "links": urdf_text.count("<link name="),
            "joints": urdf_text.count("<joint name="),
            "fixed_joints": urdf_text.count('type="fixed"'),
            "movable_joints": movable,
            "collision_elements": urdf_text.count("<collision>"),
        },
    }


def _stage_and_build_item(runner, tmp_path: Path, row: dict, name: str) -> dict:
    staged = runner.stage_package(tmp_path / "dataset", row, tmp_path / f"staging_{name}")
    return runner.build_item(row, staged, order=0)


def _run_child(runner, tmp_path: Path, item: dict, name: str = "job_000000") -> dict:
    job_root = tmp_path / f"child_{name}"
    job_root.mkdir(parents=True)
    job_path = job_root / "job.json"
    result_path = job_root / "result.json"
    job_path.write_text(json.dumps(item, sort_keys=True), encoding="utf-8")
    completed = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--internal-job",
            str(job_path),
            "--internal-result",
            str(result_path),
        ],
        capture_output=True,
        text=True,
        timeout=240,
    )
    assert completed.returncode == 0, completed.stderr
    return json.loads(result_path.read_text(encoding="utf-8"))


def test_stage_package_rejects_byte_drift(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    row = _write_dataset(root, 10, urdf_text=ZERO_COLLISION_URDF, with_obj=True)
    (root / "partseg/10/objs/10.obj").write_text("v 9 9 9\n", encoding="utf-8")
    with pytest.raises(ValueError, match="staged byte mismatch"):
        runner.stage_package(root, row, tmp_path / "staging")


def test_zero_collision_asset_is_vacuously_collision_free(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    row = _write_dataset(root, 10, urdf_text=ZERO_COLLISION_URDF, with_obj=True)
    item = _stage_and_build_item(runner, tmp_path, row, "zc")

    record = _run_child(runner, tmp_path, item)

    assert record["result_origin"] == "child_attested"
    assert record["load_success"] is True
    assert record["movable_dof_count"] == 1
    assert record["range_evaluable_dof_count"] == 1
    assert record["rest_all_pair_cf"] is True
    assert record["rest_non_adjacent_cf"] is True
    assert record["single_joint_sweep_cf"] is True
    assert record["multi_joint_sobol_cf"] is True
    assert record["measurement_complete"] is True
    assert record["strict_collision_pass"] is True
    assert record["max_penetration_normalized"] == 0.0
    states = record["state_records"]
    assert len(states) == 1 + 21 + 64
    assert all(state["all_pair_contact_count"] == 0 for state in states)
    assert all(state["non_adjacent_contact_count"] == 0 for state in states)
    runner.validate_result(item, record)


def test_zero_dof_asset_strict_fail_closed(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    row = _write_dataset(root, 20, urdf_text=ZERO_DOF_URDF)
    item = _stage_and_build_item(runner, tmp_path, row, "zd")

    record = _run_child(runner, tmp_path, item)

    assert record["load_success"] is True
    assert record["movable_dof_count"] == 0
    assert record["rest_non_adjacent_cf"] is True
    assert record["multi_joint_sobol_cf"] is False
    assert record["measurement_complete"] is True
    assert record["strict_collision_pass"] is False
    assert len(record["state_records"]) == 1
    runner.validate_result(item, record)


def test_penetrating_asset_fails_collision_checks(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    row = _write_dataset(root, 30, urdf_text=PENETRATING_URDF)
    item = _stage_and_build_item(runner, tmp_path, row, "pen")

    record = _run_child(runner, tmp_path, item)

    assert record["load_success"] is True
    assert record["rest_all_pair_cf"] is False
    assert record["rest_non_adjacent_cf"] is False
    assert record["single_joint_sweep_cf"] is False
    assert record["multi_joint_sobol_cf"] is False
    assert record["strict_collision_pass"] is False
    assert record["max_penetration_normalized"] > 0.0
    rest_state = record["state_records"][0]
    assert rest_state["all_pair_contact_count"] > 0
    assert rest_state["non_adjacent_contact_count"] > 0
    assert rest_state["non_adjacent_illegal_penetration_count"] > 0
    runner.validate_result(item, record)


def test_harness_timeout_is_fail_closed(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    row = _write_dataset(root, 40, urdf_text=ZERO_DOF_URDF)
    item = _stage_and_build_item(runner, tmp_path, row, "to")
    item["internal_test_action"] = "sleep"
    item["sleep"] = 10.0

    records = runner.execute_jobs(
        [item], worker_scratch=tmp_path / "scratch", timeout_seconds=0.7, max_workers=1
    )
    assert len(records) == 1
    record = records[0]
    assert record["child_timed_out"] is True
    assert record["strict_collision_pass"] is False
    assert record["rest_state_executed"] == 0
    assert record["result_origin"] == "parent_synthesized"
    assert record["worker_evidence"]["timed_out"] is True


def test_harness_end_to_end_aggregation(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    row_a = _write_dataset(root, 10, urdf_text=ZERO_COLLISION_URDF, with_obj=True)
    row_b = _write_dataset(root, 20, urdf_text=ZERO_DOF_URDF)
    items = [
        _stage_and_build_item(runner, tmp_path, row_a, "a"),
        _stage_and_build_item(runner, tmp_path, row_b, "b"),
    ]
    items[1]["order"] = 1

    records = runner.execute_jobs(
        items, worker_scratch=tmp_path / "scratch", timeout_seconds=240.0, max_workers=2
    )
    assert [record["load_success"] for record in records] == [True, True]

    summary = runner.load_core().summarize_records(
        {"sample_size": 2, "protocol_id": runner.PROTOCOL_ID}, records
    )
    metrics = summary["metrics"]
    assert metrics["strict_collision_pass"]["passed"] == 1
    assert metrics["strict_collision_pass"]["denominator"] == 2
    # asset A: 1 rest + 21 single + 64 sobol; asset B: 1 rest
    assert metrics["collision_state_rate"]["denominator"] == 87
    assert metrics["collision_state_rate"]["collision_states"] == 0
    assert metrics["max_penetration"]["maximum_observed_normalized"] == 0.0
    assert metrics["max_penetration"]["status"] == "COMPLETE"
    assert summary["status"] == "COMPLETE"
