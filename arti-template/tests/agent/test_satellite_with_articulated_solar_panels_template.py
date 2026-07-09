from __future__ import annotations

import pytest

from agent.templates.satellite_with_articulated_solar_panels import (
    PALETTES,
    SatelliteConfig,
    build_satellite_with_articulated_solar_panels,
    config_from_seed,
    run_satellite_with_articulated_solar_panels_tests,
    slot_choices_for_seed,
)


def _used_material_names(model) -> set[str]:
    return {
        visual.material
        for part in model.parts
        for visual in part.visuals
        if isinstance(visual.material, str)
    }


def test_all_satellite_palettes_define_material_roles() -> None:
    expected_roles = {
        "antenna",
        "blanket",
        "bus",
        "bus_alt",
        "dark",
        "deck",
        "foil",
        "lens",
        "marking",
        "radiator",
        "radiator_louver",
        "sensor",
        "solar",
        "solar_alt",
        "solar_grid",
        "structure",
        "thermal",
        "thruster",
        "thruster_inner",
    }

    assert len(PALETTES) >= 6
    for palette in PALETTES.values():
        assert expected_roles <= set(palette)
        assert all(len(rgba) == 4 for rgba in palette.values())
        assert all(0.0 <= channel <= 1.0 for rgba in palette.values() for channel in rgba)


def test_default_satellite_uses_expanded_material_roles() -> None:
    model = build_satellite_with_articulated_solar_panels(SatelliteConfig())
    registered = {material.name for material in model.materials}
    used = _used_material_names(model)

    expected_used_roles = {
        "antenna",
        "blanket",
        "bus",
        "bus_alt",
        "dark",
        "deck",
        "foil",
        "lens",
        "marking",
        "radiator",
        "radiator_louver",
        "solar",
        "solar_alt",
        "solar_grid",
        "structure",
        "thermal",
        "thruster",
        "thruster_inner",
    }
    assert expected_used_roles <= registered
    assert expected_used_roles <= used


@pytest.mark.parametrize("theme", sorted(PALETTES))
def test_each_palette_theme_builds_with_registered_materials(theme: str) -> None:
    model = build_satellite_with_articulated_solar_panels(SatelliteConfig(palette_theme=theme))
    registered = {material.name for material in model.materials}
    used = _used_material_names(model)

    assert set(PALETTES[theme]) == registered
    assert used <= registered
    assert "palette_theme" in {slot for slot, _choice in model.meta["slot_choices"]}


def test_seeded_satellites_cover_all_palette_themes() -> None:
    sampled_themes = {config_from_seed(seed).palette_theme for seed in range(80)}

    assert sampled_themes == set(PALETTES)
    assert {choice for slot, choice in slot_choices_for_seed(10) if slot == "palette_theme"} == {
        config_from_seed(10).palette_theme
    }


@pytest.mark.parametrize("seed", range(12))
def test_seeded_configs_pass_template_qc(seed: int) -> None:
    config = config_from_seed(seed)
    model = build_satellite_with_articulated_solar_panels(config)
    report = run_satellite_with_articulated_solar_panels_tests(model, config)

    assert report.passed, report.failures
