from __future__ import annotations

import pytest

from agent.templates.cctv_mast_with_pantilt_camera_head import (
    CctvMastWithPantiltCameraHeadConfig,
    build_cctv_mast_camera,
    build_seeded_cctv_mast_camera,
    config_from_seed,
    resolve_config,
    run_cctv_mast_camera_tests,
)
from sdk import TestContext as ArticraftTestContext


def test_seed_reproducibility() -> None:
    assert config_from_seed(31) == config_from_seed(31)
    assert config_from_seed(31) != config_from_seed(32)
    assert build_seeded_cctv_mast_camera(31).name == "seeded_cctv_mast_camera_31"


def test_seeded_configs_stay_on_freestanding_pantilt_camera() -> None:
    configs = [config_from_seed(seed) for seed in range(40)]
    assert {cfg.mount_layout for cfg in configs} == {"freestanding_pole", "side_arm_pole"}
    assert {cfg.mast_profile for cfg in configs} == {"round_pole"}
    assert {cfg.arm_style for cfg in configs} <= {"none", "side_arm"}
    assert {cfg.telescoping_stage_count for cfg in configs} == {0}
    assert {cfg.pan_range_mode for cfg in configs} == {"limited"}
    assert {cfg.camera_style for cfg in configs} <= {"bullet", "box"}
    assert {cfg.pan_head_style for cfg in configs} <= {"bearing_can", "compact_socket"}


def test_validator_for_stable_core() -> None:
    config = CctvMastWithPantiltCameraHeadConfig(
        mount_layout="freestanding_pole", mast_profile="round_pole", camera_style="bullet"
    )
    model = build_cctv_mast_camera(config)
    report = run_cctv_mast_camera_tests(model, config)
    assert report.passed, report.failures


def test_constraints_and_reject_cases() -> None:
    with pytest.raises(ValueError, match="telescoping layouts"):
        resolve_config(
            CctvMastWithPantiltCameraHeadConfig(
                mount_layout="telescoping_column",
                mast_profile="square_tube",
                telescoping_stage_count=0,
            )
        )
    with pytest.raises(ValueError, match="corner_wall_bracket"):
        resolve_config(
            CctvMastWithPantiltCameraHeadConfig(
                mount_layout="corner_wall_bracket", mast_profile="round_pole"
            )
        )
    with pytest.raises(ValueError, match="tilt_range"):
        resolve_config(CctvMastWithPantiltCameraHeadConfig(tilt_range=(0.6, -0.4)))


def test_supported_side_arm_and_legacy_mount_coercion() -> None:
    side_arm = resolve_config(CctvMastWithPantiltCameraHeadConfig(mount_layout="side_arm_pole"))
    assert side_arm.mount_layout == "side_arm_pole"
    assert side_arm.arm_style == "side_arm"
    assert side_arm.telescoping_stage_count == 0

    mobile = resolve_config(
        CctvMastWithPantiltCameraHeadConfig(
            mount_layout="mobile_trailer_mast",
            mast_profile="trailer_tube_stack",
            telescoping_stage_count=3,
            camera_style="ptz_pod",
            pan_head_style="yoke_bridge",
            lens_style="dome_window",
            arm_style="braced_arm",
            pan_range_mode="continuous",
        )
    )

    assert mobile.mount_layout == "freestanding_pole"
    assert mobile.mast_profile == "round_pole"
    assert mobile.telescoping_stage_count == 0
    assert mobile.camera_style == "bullet"
    assert mobile.pan_head_style == "bearing_can"
    assert mobile.arm_style == "none"
    assert mobile.pan_range_mode == "limited"
    assert mobile.lens_style == "hooded_barrel"


def test_part_details_for_stable_core() -> None:
    default = build_cctv_mast_camera(CctvMastWithPantiltCameraHeadConfig())
    support_visuals = {v.name for v in default.get_part("support").visuals}
    pan_visuals = {v.name for v in default.get_part("pan_head").visuals}
    camera_visuals = {v.name for v in default.get_part("camera_head").visuals}
    assert {"base_plate_bolt_0", "lower_pole_collar", "upper_pole_collar"}.issubset(support_visuals)
    assert {"pan_bearing_top_ring", "pan_ring_bolt_0", "tilt_axis_outer_cap_0"}.issubset(
        pan_visuals
    )
    assert {"bullet_camera_top_spine", "rear_cable_gland", "sunshield_rear_lip"}.issubset(
        camera_visuals
    )


def test_seeded_models_pass_strict_geometry_qc() -> None:
    for seed in range(13):
        ctx = ArticraftTestContext(build_seeded_cctv_mast_camera(seed))
        ctx.check_model_valid()
        report = ctx.report()
        assert report.passed, report.failures
