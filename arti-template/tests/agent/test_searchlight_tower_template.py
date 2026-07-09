from __future__ import annotations

import math

from agent.templates.searchlight_tower import (
    PALETTES,
    SearchlightTowerConfig,
    build_searchlight_tower,
    build_seeded_searchlight_tower,
    config_from_seed,
    resolve_config,
    run_searchlight_tower_tests,
)


def test_seed_reproducibility() -> None:
    assert config_from_seed(17) == config_from_seed(17)
    assert config_from_seed(17) != config_from_seed(18)
    assert build_seeded_searchlight_tower(17).name == "seeded_searchlight_tower_17"


def test_seed0_matches_anchor_skeleton() -> None:
    model = build_searchlight_tower(config_from_seed(0))
    assert [p.name for p in model.parts] == ["base_tower", "yoke_stage", "lamp_head"]
    joints = {(a.parent, a.child, a.articulation_type.name) for a in model.articulations}
    assert joints == {
        ("base_tower", "yoke_stage", "REVOLUTE"),
        ("yoke_stage", "lamp_head", "REVOLUTE"),
    }
    base_visuals = {v.name for v in model.get_part("base_tower").visuals}
    assert all(f"mast_brace_{i}" in base_visuals for i in range(4))


def test_seeded_subdomain_covers_enums() -> None:
    configs = [config_from_seed(s) for s in range(50)]
    assert {c.root_style for c in configs} == {
        "pole_mast",
        "lattice_tower",
        "tripod_legged",
        "exposed_pole_braced",
        "desktop_fold",
    }
    assert {c.yoke_style for c in configs} == {
        "two_arm_box",
        "trunnion_yoke_mesh",
        "split_arm_collar",
        "turret",
    }
    assert {c.head_shell_style for c in configs} == {"primitive_cylinder", "lathe_shell"}
    assert {c.pan_joint_type for c in configs} == {"revolute", "continuous"}
    assert {c.material_style for c in configs} == set(PALETTES)


def test_identity_and_joint_axes() -> None:
    config = SearchlightTowerConfig()
    model = build_searchlight_tower(config)
    report = run_searchlight_tower_tests(model, config)
    assert report.passed, report.failures
    pan = model.get_articulation("pan_axis")
    tilt = model.get_articulation("tilt_axis")
    assert tuple(pan.axis) == (0.0, 0.0, 1.0)
    assert abs(tilt.axis[2]) < 1e-9 and abs(tilt.axis[1]) > 0.9
    lamp_visuals = {v.name for v in model.get_part("lamp_head").visuals}
    assert {"front_lens", "front_bezel", "pivot_hub"}.issubset(lamp_visuals)


def test_resolve_clamps_invalid_tilt_range() -> None:
    config = SearchlightTowerConfig(tilt_lower=0.9, tilt_upper=0.2)
    resolved = resolve_config(config)
    assert resolved.tilt_lower < resolved.tilt_upper


def test_continuous_pan_branch_builds() -> None:
    config = SearchlightTowerConfig(pan_joint_type="continuous")
    model = build_searchlight_tower(config)
    pan = model.get_articulation("pan_axis")
    assert pan.articulation_type.name == "CONTINUOUS"


def test_all_head_shell_styles_build() -> None:
    for style in ("primitive_cylinder", "lathe_shell"):
        config = SearchlightTowerConfig(head_shell_style=style)
        model = build_searchlight_tower(config)
        report = run_searchlight_tower_tests(model, config)
        assert report.passed, (style, report.failures)


def test_lamp_lens_uses_translucent_material() -> None:
    model = build_searchlight_tower(SearchlightTowerConfig())
    lens = model.get_part("lamp_head").get_visual("front_lens")
    assert lens.material is not None
    assert lens.material.rgba[3] < 1.0
    assert math.isfinite(lens.material.rgba[3])
