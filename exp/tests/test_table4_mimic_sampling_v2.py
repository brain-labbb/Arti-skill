from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


EXP_ROOT = Path(__file__).parents[1]


def _load(name: str):
    path = EXP_ROOT / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = _load("run_urdf_table4_partnet_mobility")
runner = _load("run_table4_full_release")
checker = _load("check_pva_table4_mimic_aware_full_release")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _mimic_urdf() -> str:
    return """<robot name="mimic_fixture">
  <link name="base">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="driver_link">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="follower_link">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="driver" type="revolute">
    <parent link="base"/><child link="driver_link"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="follower" type="revolute">
    <parent link="driver_link"/><child link="follower_link"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-0.4" upper="0.4" effort="1" velocity="1"/>
    <mimic joint="driver" multiplier="0.5" offset="0.1"/>
  </joint>
</robot>
"""


def _nonzero_offset_bbox_urdf() -> str:
    return """<robot name="mimic_bbox_fixture">
  <link name="base">
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <link name="driver_link">
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <link name="follower_link">
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <joint name="driver" type="prismatic">
    <parent link="base"/><child link="driver_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="1" velocity="1"/>
  </joint>
  <joint name="follower" type="prismatic">
    <parent link="driver_link"/><child link="follower_link"/>
    <axis xyz="1 0 0"/>
    <limit lower="1.5" upper="2.5" effort="1" velocity="1"/>
    <mimic joint="driver" multiplier="1" offset="2"/>
  </joint>
</robot>
"""


def _zero_width_root_urdf() -> str:
    return """<robot name="zero_width_fixture">
  <link name="base">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="fixed_root_link">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="sampled_root_link">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="fixed_root" type="revolute">
    <parent link="base"/><child link="fixed_root_link"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="0" effort="1" velocity="1"/>
  </joint>
  <joint name="sampled_root" type="prismatic">
    <parent link="fixed_root_link"/><child link="sampled_root_link"/>
    <origin xyz="1 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-0.2" upper="0.2" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def _manifest_for_urdf(
    tmp_path: Path,
    urdf_text: str,
    joints: list[tuple[str, str]],
    *,
    asset_id: str,
) -> Path:
    package = tmp_path / "package"
    package.mkdir()
    urdf = package / "model.urdf"
    urdf.write_text(urdf_text, encoding="utf-8")
    row = {
        "asset_id": asset_id,
        "category": "fixture",
        "joint_count": len(joints),
        "non_fixed_joints": [
            {"name": name, "type": joint_type} for name, joint_type in joints
        ],
        "source_path": str(package),
        "primary_urdf_path": str(urdf),
        "primary_urdf_relative_path": "model.urdf",
        "primary_urdf_sha256": _sha256(urdf),
    }
    value = {
        "schema_version": runner.ROSTER_SCHEMA,
        "dataset": "fixture",
        "N_eval": 1,
        "J_eval": len(joints),
        "rows": [row],
    }
    value["manifest_content_sha256"] = runner.canonical_sha256(value)
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def _manifest(tmp_path: Path) -> Path:
    return _manifest_for_urdf(
        tmp_path,
        _mimic_urdf(),
        [("driver", "revolute"), ("follower", "revolute")],
        asset_id="fixture/mimic",
    )


def test_sampling_plan_composes_chains_and_intersects_follower_limits() -> None:
    joints = [
        {"name": "root", "type": "revolute", "lower": -2.0, "upper": 2.0, "range_evaluable": True, "mimic": None},
        {"name": "middle", "type": "revolute", "lower": -1.5, "upper": 2.5, "range_evaluable": True, "mimic": {"joint": "root", "multiplier": 2.0, "offset": 0.5}},
        {"name": "tip", "type": "revolute", "lower": -0.5, "upper": 0.5, "range_evaluable": True, "mimic": {"joint": "middle", "multiplier": -0.5, "offset": 0.25}},
    ]

    plan = core.compile_joint_sampling_plan(joints)

    assert plan["independent_dof_count"] == 1
    assert plan["mimic_joint_count"] == 2
    assert plan["independent_joints"][0]["sampling_lower"] == pytest.approx(-0.5)
    assert plan["independent_joints"][0]["sampling_upper"] == pytest.approx(0.5)
    assert core.expand_joint_values(plan, [0.4]) == pytest.approx([0.4, 1.3, -0.4])


def test_external_affine_constraint_uses_one_independent_rack_dof() -> None:
    joints = [
        {
            "name": "rack_slide",
            "type": "prismatic",
            "lower": -0.03,
            "upper": 0.03,
            "range_evaluable": True,
            "mimic": None,
        },
        {
            "name": "pinion_spin",
            "type": "revolute",
            "lower": -0.6,
            "upper": 0.6,
            "range_evaluable": True,
            "mimic": None,
        },
    ]
    constraint = {
        "constraint_id": "reviewed-rack:pinion_spin",
        "driver_joint": "rack_slide",
        "follower_joint": "pinion_spin",
        "multiplier": 20.0,
        "offset": 0.0,
    }

    plan = core.compile_joint_sampling_plan(
        joints, external_joint_constraints=[constraint]
    )

    assert plan["independent_dof_count"] == 1
    assert plan["native_mimic_joint_count"] == 0
    assert plan["external_joint_constraint_count"] == 1
    assert plan["mimic_joint_count"] == 1
    assert plan["independent_joints"][0]["name"] == "rack_slide"
    assert core.expand_joint_values(plan, [0.02]) == pytest.approx([0.02, 0.4])
    assert joints[1]["mimic"] is None


def test_external_affine_constraint_cannot_override_native_mimic() -> None:
    joints = [
        {
            "name": "driver",
            "type": "revolute",
            "lower": -1.0,
            "upper": 1.0,
            "range_evaluable": True,
            "mimic": None,
        },
        {
            "name": "follower",
            "type": "revolute",
            "lower": -1.0,
            "upper": 1.0,
            "range_evaluable": True,
            "mimic": {"joint": "driver", "multiplier": 1.0, "offset": 0.0},
        },
    ]
    external = {
        "constraint_id": "must-not-override",
        "driver_joint": "driver",
        "follower_joint": "follower",
        "multiplier": 2.0,
        "offset": 0.0,
    }

    with pytest.raises(ValueError, match="override native mimic"):
        core.compile_joint_sampling_plan(
            joints, external_joint_constraints=[external]
        )


def test_zero_width_root_is_fixed_while_positive_width_root_is_sampled() -> None:
    joints = [
        {
            "name": "fixed_root",
            "type": "revolute",
            "lower": 0.0,
            "upper": 0.0,
            "range_evaluable": False,
            "mimic": None,
        },
        {
            "name": "free_root",
            "type": "prismatic",
            "lower": -1.0,
            "upper": 1.0,
            "range_evaluable": True,
            "mimic": None,
        },
        {
            "name": "follower",
            "type": "prismatic",
            "lower": -5.0,
            "upper": 5.0,
            "range_evaluable": True,
            "mimic": {
                "joint": "free_root",
                "multiplier": 2.0,
                "offset": 0.25,
            },
        },
    ]

    plan = core.compile_joint_sampling_plan(joints)

    assert plan["independent_dof_count"] == 1
    assert plan["range_evaluable_independent_dof_count"] == 1
    assert [row["name"] for row in plan["independent_joints"]] == ["free_root"]
    assert core.expand_joint_values(plan, [0.0]) == pytest.approx([0.0, 0.0, 0.25])
    assert core.expand_joint_values(plan, [0.75]) == pytest.approx(
        [0.0, 0.75, 1.75]
    )


def test_zero_width_follower_pins_root_and_expands_fixed_state() -> None:
    joints = [
        {
            "name": "root",
            "type": "revolute",
            "lower": -2.0,
            "upper": 2.0,
            "range_evaluable": True,
            "mimic": None,
        },
        {
            "name": "follower",
            "type": "revolute",
            "lower": 1.25,
            "upper": 1.25,
            "range_evaluable": False,
            "mimic": {
                "joint": "root",
                "multiplier": 2.0,
                "offset": 0.25,
            },
        },
    ]

    plan = core.compile_joint_sampling_plan(joints)

    assert plan["independent_dof_count"] == 0
    assert plan["range_evaluable_independent_dof_count"] == 0
    assert plan["independent_joints"] == []
    assert plan["mimic_joint_count"] == 1
    assert core.expand_joint_values(plan, []) == pytest.approx([0.5, 1.25])


def test_incompatible_zero_width_follower_remains_non_evaluable() -> None:
    joints = [
        {
            "name": "root",
            "type": "revolute",
            "lower": -1.0,
            "upper": 1.0,
            "range_evaluable": True,
            "mimic": None,
        },
        {
            "name": "follower",
            "type": "revolute",
            "lower": 3.0,
            "upper": 3.0,
            "range_evaluable": False,
            "mimic": {
                "joint": "root",
                "multiplier": 1.0,
                "offset": 0.0,
            },
        },
    ]

    plan = core.compile_joint_sampling_plan(joints)

    assert plan["independent_dof_count"] == 1
    assert plan["range_evaluable_independent_dof_count"] == 0
    assert [row["name"] for row in plan["independent_joints"]] == ["root"]
    assert plan["independent_joints"][0]["sampling_range_evaluable"] is False
    assert plan["independent_joints"][0]["zero_baseline_evaluable"] is False


@pytest.mark.parametrize(
    "joints, match",
    [
        ([{"name": "a", "type": "revolute", "lower": -1.0, "upper": 1.0, "mimic": {"joint": "missing", "multiplier": 1.0, "offset": 0.0}}], "missing joint"),
        ([{"name": "a", "type": "revolute", "lower": -1.0, "upper": 1.0, "mimic": {"joint": "b", "multiplier": 1.0, "offset": 0.0}}, {"name": "b", "type": "revolute", "lower": -1.0, "upper": 1.0, "mimic": {"joint": "a", "multiplier": 1.0, "offset": 0.0}}], "cycle"),
    ],
)
def test_sampling_plan_rejects_invalid_mimic_graphs(joints, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        core.compile_joint_sampling_plan(joints)


def test_sampling_plan_fails_closed_for_invalid_limits_or_zero_baseline() -> None:
    invalid_follower = [
        {"name": "root", "type": "revolute", "lower": -1.0, "upper": 1.0, "range_evaluable": True, "mimic": None},
        {"name": "follower", "type": "revolute", "lower": 1.0, "upper": -1.0, "range_evaluable": False, "mimic": {"joint": "root", "multiplier": 1.0, "offset": 0.0}},
    ]
    shifted_follower = [
        {"name": "root", "type": "revolute", "lower": -2.0, "upper": 2.0, "range_evaluable": True, "mimic": None},
        {"name": "follower", "type": "revolute", "lower": 1.0, "upper": 2.0, "range_evaluable": True, "mimic": {"joint": "root", "multiplier": 1.0, "offset": 0.0}},
    ]

    invalid_plan = core.compile_joint_sampling_plan(invalid_follower)
    shifted_plan = core.compile_joint_sampling_plan(shifted_follower)

    assert invalid_plan["range_evaluable_independent_dof_count"] == 0
    assert shifted_plan["independent_joints"][0]["sampling_lower"] == pytest.approx(1.0)
    assert shifted_plan["independent_joints"][0]["sampling_upper"] == pytest.approx(2.0)
    assert shifted_plan["independent_joints"][0]["zero_baseline_evaluable"] is False
    assert shifted_plan["range_evaluable_independent_dof_count"] == 0


def test_v2_runner_samples_only_independent_dof_and_expands_full_state(tmp_path: Path) -> None:
    manifest_path = _manifest(tmp_path)
    _manifest_value, jobs = runner.load_roster(
        manifest_path,
        dataset="fixture",
        sampling_protocol=runner.SAMPLING_PROTOCOL_V2,
    )
    job = jobs[0]

    assert job["independent_dof_count"] == 1
    assert job["mimic_joint_count"] == 1
    assert job["single_state_expected"] == 21
    assert job["sobol_state_expected"] == 64
    assert job["joint_sampling_plan_sha256"]

    result = runner.evaluate_job(job)

    assert result["status"] == "completed"
    assert result["measurement_complete"] is True
    assert result["independent_dof_count"] == 1
    assert result["single_state_executed"] == 21
    assert result["sobol_state_executed"] == 64
    assert len(result["state_records"]) == 86
    assert {state["joint_name"] for state in result["state_records"] if state["phase"] == "single_joint_sweep"} == {"driver"}
    assert all(state["schema_version"] == "table4_state_v2" for state in result["state_records"])
    first_single = next(
        state for state in result["state_records"] if state["phase"] == "single_joint_sweep"
    )
    assert first_single["joint_values_sha256"] == core.canonical_sha256([-1.0, -0.4])
    source_row = json.loads(manifest_path.read_text())["rows"][0]
    plan = checker._plan_metadata(source_row)
    for state in result["state_records"]:
        expected_values = checker._expected_joint_values(
            plan,
            state["phase"],
            state["sample_index"],
            state["joint_name"],
        )
        assert state["joint_values_sha256"] == checker._canonical_sha256(
            expected_values
        )


def test_v2_runner_excludes_fixed_root_from_state_denominators(
    tmp_path: Path,
) -> None:
    manifest_path = _manifest_for_urdf(
        tmp_path,
        _zero_width_root_urdf(),
        [("fixed_root", "revolute"), ("sampled_root", "prismatic")],
        asset_id="fixture/zero-width-root",
    )
    _manifest_value, jobs = runner.load_roster(
        manifest_path,
        dataset="fixture",
        sampling_protocol=runner.SAMPLING_PROTOCOL_V2,
    )

    job = jobs[0]
    assert job["independent_dof_count"] == 1
    assert job["fixed_root_joint_count"] == 1
    assert job["mimic_joint_count"] == 0
    assert job["single_state_expected"] == 21
    assert job["sobol_state_expected"] == 64

    result = runner.evaluate_job(job)

    assert result["status"] == "completed"
    assert result["measurement_complete"] is True
    assert result["fixed_root_joint_count"] == 1
    assert result["single_state_executed"] == 21
    assert result["sobol_state_executed"] == 64
    assert len(result["state_records"]) == 86
    assert {
        state["joint_name"]
        for state in result["state_records"]
        if state["phase"] == "single_joint_sweep"
    } == {"sampled_root"}


def test_v2_bbox_scale_uses_expanded_nonzero_offset_rest_state(
    tmp_path: Path,
) -> None:
    bullet = pytest.importorskip("pybullet")
    manifest_path = _manifest_for_urdf(
        tmp_path,
        _nonzero_offset_bbox_urdf(),
        [("driver", "prismatic"), ("follower", "prismatic")],
        asset_id="fixture/nonzero-offset-bbox",
    )
    _manifest_value, jobs = runner.load_roster(
        manifest_path,
        dataset="fixture",
        sampling_protocol=runner.SAMPLING_PROTOCOL_V2,
    )

    result = runner.evaluate_job(jobs[0])

    assert result["status"] == "completed"
    rest_state = next(
        state for state in result["state_records"] if state["phase"] == "rest"
    )
    assert rest_state["joint_values_sha256"] == core.canonical_sha256([0.0, 2.0])

    client = bullet.connect(bullet.DIRECT)
    body = None
    try:
        flags = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        urdf_path = Path(jobs[0]["urdf_path"])
        body = bullet.loadURDF(
            str(urdf_path),
            useFixedBase=True,
            flags=flags,
            physicsClientId=client,
        )
        default_zero_scale = runner._derive_scale(bullet, body, client)
        simulator_by_name = {}
        for index in range(bullet.getNumJoints(body, physicsClientId=client)):
            info = bullet.getJointInfo(body, index, physicsClientId=client)
            name = info[1].decode("utf-8")
            simulator_by_name[name] = index
        for name, value in (("driver", 0.0), ("follower", 2.0)):
            bullet.resetJointState(
                body,
                simulator_by_name[name],
                value,
                physicsClientId=client,
            )
        bullet.performCollisionDetection(physicsClientId=client)
        expanded_rest_scale = runner._derive_scale(bullet, body, client)
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)

    assert expanded_rest_scale > default_zero_scale * 4.0
    assert result["object_bbox_diagonal_m"] == pytest.approx(
        expanded_rest_scale, abs=1e-9
    )
    assert result["object_bbox_diagonal_m"] != pytest.approx(
        default_zero_scale, abs=1e-9
    )

    source_row = json.loads(manifest_path.read_text())["rows"][0]
    plan = checker._plan_metadata(source_row)
    checker._validate_states(
        result,
        result["state_records"],
        0,
        plan,
        require_normalization=True,
    )

    bad_rest_receipt = dict(result)
    bad_rest_receipt["normalization_joint_values_sha256"] = "0" * 64
    with pytest.raises(checker.CheckError, match="normalization rest-state hash"):
        checker._validate_states(
            bad_rest_receipt,
            result["state_records"],
            0,
            plan,
            require_normalization=True,
        )

    bad_states = [dict(state) for state in result["state_records"]]
    bad_states[0]["metric_max_penetration_m"] += 1e-3
    bad_raw_metric = dict(result)
    bad_raw_metric["state_records_sha256"] = checker._canonical_sha256(bad_states)
    with pytest.raises(checker.CheckError, match="metric penetration policy"):
        checker._validate_states(bad_raw_metric, bad_states, 0, plan)

    bad_normalized = dict(result)
    bad_normalized["max_penetration_normalized"] += 1.0
    with pytest.raises(checker.CheckError, match="max_penetration_normalized"):
        checker._validate_states(
            bad_normalized,
            result["state_records"],
            0,
            plan,
            require_normalization=True,
        )


def test_v1_runner_keeps_historical_independent_joint_sampling(tmp_path: Path) -> None:
    _manifest_value, jobs = runner.load_roster(
        _manifest(tmp_path), dataset="fixture"
    )
    assert jobs[0]["sampling_protocol"] == runner.SAMPLING_PROTOCOL_V1
    assert jobs[0]["single_state_expected"] == 42
    assert jobs[0]["sobol_state_expected"] == 64
