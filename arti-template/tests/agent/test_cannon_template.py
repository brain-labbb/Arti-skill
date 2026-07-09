from __future__ import annotations

import pytest

from agent.templates.cannon import (
    CannonConfig,
    build_cannon,
    build_seeded_cannon,
    config_from_seed,
    resolve_config,
    run_cannon_tests,
)
from sdk import ArticulationType
from sdk import TestContext as ArticraftTestContext


def test_seed_config_is_reproducible() -> None:
    assert config_from_seed(11) == config_from_seed(11)
    assert config_from_seed(11) != config_from_seed(12)


def test_rejects_invalid_layout_and_ranges() -> None:
    with pytest.raises(ValueError, match="Unsupported mount_layout"):
        resolve_config(CannonConfig(mount_layout="pipe_stand"))  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="wheel_count"):
        resolve_config(CannonConfig(wheel_count=3))
    with pytest.raises(ValueError, match="elevation_range"):
        resolve_config(CannonConfig(elevation_range_deg=(20.0, 10.0)))


def test_default_wheeled_cannon_matches_validator() -> None:
    config = CannonConfig()
    model = build_cannon(config)
    report = run_cannon_tests(model, config)
    assert report.passed, report.failures
    assert model.get_articulation("barrel_elevation").articulation_type == ArticulationType.REVOLUTE
    assert (
        model.get_articulation("left_wheel_spin").articulation_type == ArticulationType.CONTINUOUS
    )
    barrel_visuals = {visual.name for visual in model.get_part("barrel").visuals}
    support_visuals = {visual.name for visual in model.get_part("support").visuals}
    assert {"muzzle_bore_shadow", "vent_touch_hole_plate", "lifting_dolphin"}.issubset(
        barrel_visuals
    )
    assert {"left_trunnion_cap_strap", "trail_lunette_handle"}.issubset(support_visuals)


def test_optional_layout_constraints() -> None:
    naval = CannonConfig(mount_layout="naval_slide", barrel_profile="short_carronade")
    naval_model = build_cannon(naval)
    assert run_cannon_tests(naval_model, naval).passed
    assert naval_model.get_articulation("recoil_slide").axis == (-1.0, 0.0, 0.0)
    assert naval_model.get_articulation("traverse_joint").axis == (0.0, 0.0, 1.0)
    assert naval_model.get_articulation("recoil_slide").parent == "yoke_or_rotating_carriage"
    assert naval_model.get_articulation("barrel_elevation").parent == "recoil_carriage"

    support_visuals = {visual.name for visual in naval_model.get_part("support").visuals}
    rotating_visuals = {
        visual.name for visual in naval_model.get_part("yoke_or_rotating_carriage").visuals
    }
    assert "slide_rail" not in support_visuals
    assert {"deck_plank_seam_0", "deck_traverse_ring"}.issubset(support_visuals)
    assert {"port_slide_rail", "starboard_slide_rail"}.issubset(rotating_visuals)

    mortar = CannonConfig(mount_layout="trestle_mortar", barrel_profile="squat_mortar")
    mortar_model = build_cannon(mortar)
    assert run_cannon_tests(mortar_model, mortar).passed
    assert (
        mortar_model.get_articulation("wedge_slide").articulation_type == ArticulationType.PRISMATIC
    )


def test_four_wheel_field_carriage_uses_four_attached_wheels() -> None:
    config = CannonConfig(mount_layout="wheeled_field_carriage", wheel_count=4)
    model = build_cannon(config)
    report = run_cannon_tests(model, config)

    assert report.passed, report.failures
    for name in ("rear_left", "rear_right", "front_left", "front_right"):
        assert model.get_part(f"{name}_wheel") is not None
        assert (
            model.get_articulation(f"{name}_wheel_spin").articulation_type
            == ArticulationType.CONTINUOUS
        )

    spoke_angles = {
        visual.origin.rpy
        for visual in model.get_part("rear_left_wheel").visuals
        if visual.name and visual.name.startswith("spoke_")
    }
    assert len(spoke_angles) > 1
    wheel_visuals = {visual.name for visual in model.get_part("rear_left_wheel").visuals}
    assert {"hub_cap", "rim_rivet_0"}.issubset(wheel_visuals)


def test_traverse_joint_origin_sits_on_bearing_not_inside_barbette() -> None:
    config = CannonConfig(mount_layout="barbette_platform")
    model = build_cannon(config)
    assert model.get_articulation("traverse_joint").origin.xyz[2] == pytest.approx(0.42)


def test_diversity_parameters_are_observable() -> None:
    configs = [config_from_seed(seed) for seed in range(60)]
    assert len({cfg.mount_layout for cfg in configs}) >= 4
    assert len({cfg.barrel_profile for cfg in configs}) >= 3
    assert len({cfg.wheel_style for cfg in configs}) >= 2
    assert len({cfg.material_style for cfg in configs}) >= 3


def test_build_seeded_cannon_runs_template_tests() -> None:
    model = build_seeded_cannon(7)
    config = config_from_seed(7)
    report = run_cannon_tests(model, config)
    assert report.passed, report.failures


def test_seed_0_to_2_pass_strict_contact_qc() -> None:
    for seed in range(3):
        ctx = ArticraftTestContext(build_seeded_cannon(seed))
        ctx.check_model_valid()
        ctx.fail_if_isolated_parts()
        ctx.fail_if_parts_overlap_in_current_pose(
            overlap_tol=0.003,
            overlap_volume_tol=1e-7,
        )
        report = ctx.report()
        assert report.passed, report.failures
