from __future__ import annotations

from pathlib import Path

import pytest

from agent.templates.drone import (
    DroneConfig,
    build_drone,
    build_seeded_drone,
    config_from_seed,
    resolve_config,
    run_drone_tests,
)
from sdk import ArticulationType
from sdk._core.v0.testing import TestContext as SDKTestContext


def _assert_template_qc_passes(config: DroneConfig) -> None:
    model = build_drone(config)
    report = run_drone_tests(model, config)
    assert report.passed, report.failures


def test_template_line_floor() -> None:
    template_path = Path("agent/templates/drone.py")
    assert len(template_path.read_text().splitlines()) >= 1000


def test_seed_reproducibility() -> None:
    assert config_from_seed(5) == config_from_seed(5)
    assert config_from_seed(5) != config_from_seed(6)
    assert build_seeded_drone(5).name == "seeded_drone_5"


def test_default_seed_domain_stays_multirotor_core() -> None:
    families = {config_from_seed(seed).airframe_family for seed in range(60)}
    assert families <= {"quad_folding", "hex_multirotor", "flat_quad_gimbal"}


def test_hex_multirotor_has_six_radial_propellers() -> None:
    config = DroneConfig(airframe_family="hex_multirotor")
    resolved = resolve_config(config)
    model = build_drone(config)
    spin_joints = [
        joint for joint in model.articulations if joint.name.startswith("propeller_spin_")
    ]
    fold_joints = [joint for joint in model.articulations if joint.name.startswith("arm_fold_")]

    assert resolved.rotor_count == 6
    assert len(spin_joints) == 6
    assert len(fold_joints) == 6
    assert all(tuple(joint.axis) == (0.0, 0.0, 1.0) for joint in spin_joints)


def test_quad_folding_arm_hinges_start_on_body_rim() -> None:
    config = DroneConfig(airframe_family="quad_folding")
    resolved = resolve_config(config)
    model = build_drone(config)
    fold_joints = [joint for joint in model.articulations if joint.name.startswith("arm_fold_")]

    assert len(fold_joints) == resolved.rotor_count
    for joint in fold_joints:
        x, y, _ = joint.origin.xyz
        assert abs((x * x + y * y) ** 0.5 - resolved.body_mount_radius) < 0.12
        assert joint.articulation_type == ArticulationType.REVOLUTE


def test_gimbal_chain_axes_are_semantic() -> None:
    config = DroneConfig(airframe_family="flat_quad_gimbal", gimbal_enabled=True)
    model = build_drone(config)
    yaw = model.get_articulation("gimbal_yaw")
    tilt = model.get_articulation("gimbal_tilt")
    roll = model.get_articulation("gimbal_roll")

    assert yaw.articulation_type == ArticulationType.REVOLUTE
    assert tilt.articulation_type == ArticulationType.REVOLUTE
    assert roll.articulation_type == ArticulationType.REVOLUTE
    assert tuple(yaw.axis) == (0.0, 0.0, 1.0)
    assert tuple(tilt.axis) == (0.0, 1.0, 0.0)
    assert tuple(roll.axis) == (1.0, 0.0, 0.0)


def test_review_gated_vtol_downgrades_in_default_domain() -> None:
    resolved = resolve_config(
        DroneConfig(
            airframe_family="fixed_wing_vtol",
            seed_domain="multirotor_core",
        )
    )
    assert resolved.airframe_family == "quad_folding"


@pytest.mark.parametrize("seed", range(12))
def test_seeded_configs_pass_template_qc(seed: int) -> None:
    _assert_template_qc_passes(config_from_seed(seed))


def test_strict_current_pose_overlap_seed_zero() -> None:
    model = build_drone(config_from_seed(0))
    ctx = SDKTestContext(model)
    ctx.fail_if_parts_overlap_in_current_pose(overlap_tol=0.003, overlap_volume_tol=1e-7)
    assert ctx.report().passed
