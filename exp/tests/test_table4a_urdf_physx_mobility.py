from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_table4a_urdf_physx_mobility.py"
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))


def _load_runner():
    spec = importlib.util.spec_from_file_location("run_table4a_urdf_physx_mobility", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


ZERO_COLLISION_URDF = """<robot name="scene">
  <link name="l_world">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="1.0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="l_0">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="1.0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <joint name="j_revolute" type="revolute">
    <parent link="l_world"/><child link="l_0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
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


def _write_package(tmp_path: Path, name: str, urdf_text: str) -> Path:
    pkg = tmp_path / "packages" / name
    (pkg / "urdf").mkdir(parents=True)
    urdf = pkg / "urdf" / "model.urdf"
    urdf.write_text(urdf_text, encoding="utf-8")
    return pkg


def _base_job(runner, pkg: Path, *, expected_dof: int, joints: list[dict]) -> dict:
    urdf = pkg / "urdf" / "model.urdf"
    return {
        "selection_index": 0,
        "dataset_id": "999999",
        "rank": 1,
        "category": "Fixture",
        "package": str(pkg),
        "package_binding": runner.package_binding(pkg),
        "urdf_path": str(urdf),
        "expected_urdf_sha256": runner.lam4a.sha256_file(urdf),
        "manifest_row_sha256": "fixture-row",
        "input_identity_sha256": "fixture-row",
        "expected_movable_dof": expected_dof,
        "expected_state_count": 21 * expected_dof,
        "joints": joints,
        "genesis_cache_path": str(pkg.parent / f"cache_{pkg.name}"),
        "template_cache_path": None,
    }


def _run_child(runner, tmp_path: Path, job: dict, name: str) -> dict:
    job_root = tmp_path / f"child_{name}"
    job_root.mkdir(parents=True)
    job_path = job_root / "job.json"
    result_path = job_root / "result.json"
    job_path.write_text(json.dumps(job, sort_keys=True), encoding="utf-8")
    import os
    env = dict(os.environ)
    env["GS_CACHE_FILE_PATH"] = str(job["genesis_cache_path"])
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    completed = subprocess.run(
        [sys.executable, str(RUNNER), "--child", "--job", str(job_path), "--result", str(result_path)],
        capture_output=True,
        text=True,
        timeout=600,
        cwd=str(REPO),
        env=env,
    )
    assert completed.returncode == 0, completed.stderr[-2000:]
    return json.loads(result_path.read_text(encoding="utf-8"))


def test_child_zero_collision_asset_vacuous_pass(tmp_path: Path) -> None:
    runner = _load_runner()
    pkg = _write_package(tmp_path, "zc", ZERO_COLLISION_URDF)
    values = [0.0 + i * (1.0 - 0.0) / 20 for i in range(21)]
    references = [runner.lam4a.canonical_sha256([v]) for v in values]
    job = _base_job(
        runner,
        pkg,
        expected_dof=1,
        joints=[
            {
                "name": "j_revolute",
                "type": "revolute",
                "lower": 0.0,
                "upper": 1.0,
                "xml_index": 0,
                "dof_position": 0,
                "range_evaluable": True,
                "values": values,
                "state_hash_references": references,
                "table3_joint_level_pass": True,
            }
        ],
    )

    record = _run_child(runner, tmp_path, job, "zc")

    assert record["status"] == "completed", record["issues"]
    assert record["load_success"] is True
    assert record["mapping_status"] == "COMPLETE"
    assert record["eligible_pair_count"] == 0
    assert record["source_collision_element_count"] == 0
    assert record["states_executed"] == 21
    assert record["state_hash_cross_check"] == {"verified": 21, "mismatch": 0, "no_reference": 0}
    joint = record["joint_records"][0]
    assert joint["full_range_cf_pass"] is True
    assert joint["limit_reachable"] is True
    assert joint["safe_dof"] == 1
    assert all(s["illegal_collision"] is False for s in joint["state_summaries"])


def test_child_penetrating_asset_fails(tmp_path: Path) -> None:
    runner = _load_runner()
    pkg = _write_package(tmp_path, "pen", PENETRATING_URDF)
    values = [0.0 + i * (1.0 - 0.0) / 20 for i in range(21)]
    job = _base_job(
        runner,
        pkg,
        expected_dof=1,
        joints=[
            {
                "name": "j1",
                "type": "revolute",
                "lower": 0.0,
                "upper": 1.0,
                "xml_index": 1,
                "dof_position": 0,
                "range_evaluable": True,
                "values": values,
                "state_hash_references": [None] * 21,
                "table3_joint_level_pass": True,
            }
        ],
    )

    record = _run_child(runner, tmp_path, job, "pen")

    assert record["status"] == "completed", record["issues"]
    assert record["eligible_pair_count"] >= 1
    joint = record["joint_records"][0]
    assert joint["illegal_states"] > 0
    assert joint["full_range_cf_pass"] is False
    assert joint["limit_reachable"] is False
    assert joint["safe_dof"] == 0
    assert record["state_hash_cross_check"]["no_reference"] == 21


def test_child_dof_mismatch_fail_closed(tmp_path: Path) -> None:
    runner = _load_runner()
    pkg = _write_package(tmp_path, "dof", ZERO_COLLISION_URDF)
    job = _base_job(runner, pkg, expected_dof=5, joints=[])

    record = _run_child(runner, tmp_path, job, "dof")

    assert record["status"] == "error"
    assert record["load_success"] is False
    assert any("GenesisAdapterError" in issue for issue in record["issues"])


def test_stage_package_rejects_byte_drift(tmp_path: Path) -> None:
    runner = _load_runner()
    root = tmp_path / "dataset"
    (root / "urdf").mkdir(parents=True)
    (root / "partseg/10/objs").mkdir(parents=True)
    urdf = root / "urdf/10.urdf"
    urdf.write_text(ZERO_COLLISION_URDF, encoding="utf-8")
    obj = root / "partseg/10/objs/10.obj"
    obj.write_text("v 0 0 0\n", encoding="utf-8")
    row = {
        "dataset_id": 10,
        "urdf_relative_path": "urdf/10.urdf",
        "urdf_sha256": runner.lam4a.sha256_file(urdf),
        "resources": [{"relative_path": "partseg/10/objs/10.obj", "sha256": runner.lam4a.sha256_file(obj)}],
    }
    obj.write_text("v 9 9 9\n", encoding="utf-8")
    with pytest.raises(ValueError, match="staged byte mismatch"):
        runner.stage_package(root, row, tmp_path / "staging")


def test_aggregate_and_dof_bins(tmp_path: Path) -> None:
    runner = _load_runner()

    def completed_record(dataset_id: str, n_joints: int, safe: int) -> dict:
        joints = []
        for i in range(n_joints):
            joints.append(
                {
                    "joint_name": f"j{i}",
                    "joint_type": "revolute",
                    "dof_position": i,
                    "xml_index": i,
                    "states_intended": 21,
                    "states_executed": 21,
                    "illegal_states": 0,
                    "full_range_cf_pass": True,
                    "limit_endpoints_intended": 2,
                    "limit_endpoints_executed": 2,
                    "limit_reachable": True,
                    "table3_joint_level_pass": i < safe,
                    "safe_dof": int(i < safe),
                    "issues": [],
                }
            )
        return {
            "dataset_id": dataset_id,
            "category": "Fixture",
            "selection_index": int(dataset_id),
            "expected_movable_dof": n_joints,
            "status": "completed",
            "states_intended": 21 * n_joints,
            "states_executed": 21 * n_joints,
            "state_hash_cross_check": {"verified": 21 * n_joints, "mismatch": 0, "no_reference": 0},
            "joint_records": joints,
        }

    failed = runner._failed_asset_record(
        {
            "selection_index": 2,
            "dataset_id": "30",
            "category": "Fixture",
            "package": "/tmp/none",
            "urdf_path": "/tmp/none/model.urdf",
            "expected_urdf_sha256": "x",
            "expected_movable_dof": 2,
            "expected_state_count": 42,
            "joints": [
                {"name": "a", "type": "revolute", "dof_position": 0, "xml_index": 0, "table3_joint_level_pass": True},
                {"name": "b", "type": "revolute", "dof_position": 1, "xml_index": 1, "table3_joint_level_pass": False},
            ],
        },
        issue="probe_failure",
    )
    records = [completed_record("10", 1, 1), completed_record("20", 3, 2), failed]
    table4_strict = {"10": True, "20": False, "30": False}

    agg = runner.aggregate(records, table4_strict)
    assert agg["joint_level_full_range_cf"] == {"numerator": 4, "denominator": 6, "percent": 66.6667}
    assert agg["collision_safe_dof_retention"]["numerator"] == 3
    assert agg["collision_safe_dof_retention"]["denominator"] == runner.J_EVAL
    assert agg["executable_cf_dof_per_asset"]["n_assets"] == 3
    assert agg["executable_cf_dof_per_asset"]["total_safe_dof"] == 3
    assert agg["limit_reachability"]["numerator"] == 4
    assert agg["limit_reachability"]["denominator"] == 6
    assert agg["normalized_clearance_p5"]["status"] == "N/E"
    assert agg["normalized_clearance_p5"]["coverage_status"] == "PARTIAL"
    bins = agg["dof_bins"]
    assert bins["1"]["n_eval"] == 1 and bins["1"]["strict_pass"] == 1
    assert bins["2--3"]["n_eval"] == 2 and bins["2--3"]["strict_pass"] == 0
    assert bins["0"]["n_eval"] == 0
