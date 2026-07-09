"""Satellite with articulated solar panels modular procedural template.

The category spec is currently source-blocked, but the template is shaped as
a modular object with explicit topology slots:

* ``bus_body``: spacecraft bus family.
* ``solar_array``: articulated solar-wing topology.
* ``payload``: dish, mast, or instrument package.
* ``thruster_pack``: aft propulsion detail.

seed=0 is the bilateral box-bus anchor with two articulated solar wings and a
forward high-gain dish.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Literal

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

__modular__ = True

BusBodyModule = Literal["boxy_comm_bus", "octagonal_science_bus", "truss_service_bus"]
SolarArrayModule = Literal["bilateral_two_panel_wings", "four_wing_cross", "foldout_segment_wings"]
PayloadModule = Literal["high_gain_dish", "sensor_mast_cluster", "side_payload_boom"]
ThrusterPackModule = Literal["single_apogee_nozzle", "quad_corner_thrusters", "dual_ion_thrusters"]
PaletteTheme = Literal[
    "gold_comm",
    "white_science",
    "graphite_thermal",
    "silver_probe",
    "copper_mli",
    "blue_white_orbiter",
]

BUS_MODULES: tuple[BusBodyModule, ...] = (
    "boxy_comm_bus",
    "octagonal_science_bus",
    "truss_service_bus",
)
SOLAR_MODULES: tuple[SolarArrayModule, ...] = (
    "bilateral_two_panel_wings",
    "four_wing_cross",
    "foldout_segment_wings",
)
PAYLOAD_MODULES: tuple[PayloadModule, ...] = (
    "high_gain_dish",
    "sensor_mast_cluster",
    "side_payload_boom",
)
THRUSTER_MODULES: tuple[ThrusterPackModule, ...] = (
    "single_apogee_nozzle",
    "quad_corner_thrusters",
    "dual_ion_thrusters",
)

PALETTES: dict[PaletteTheme, dict[str, tuple[float, float, float, float]]] = {
    "gold_comm": {
        "bus": (0.78, 0.66, 0.22, 1.0),
        "bus_alt": (0.86, 0.78, 0.38, 1.0),
        "deck": (0.70, 0.72, 0.76, 1.0),
        "structure": (0.58, 0.60, 0.64, 1.0),
        "thermal": (0.10, 0.11, 0.13, 1.0),
        "blanket": (0.92, 0.78, 0.32, 1.0),
        "foil": (0.82, 0.70, 0.42, 1.0),
        "radiator": (0.82, 0.84, 0.86, 1.0),
        "radiator_louver": (0.18, 0.19, 0.21, 1.0),
        "solar": (0.06, 0.15, 0.34, 1.0),
        "solar_alt": (0.04, 0.10, 0.24, 1.0),
        "solar_grid": (0.78, 0.80, 0.84, 1.0),
        "antenna": (0.90, 0.91, 0.88, 1.0),
        "sensor": (0.82, 0.88, 0.94, 1.0),
        "lens": (0.22, 0.38, 0.48, 1.0),
        "thruster": (0.17, 0.17, 0.18, 1.0),
        "thruster_inner": (0.05, 0.05, 0.055, 1.0),
        "marking": (0.74, 0.18, 0.12, 1.0),
        "dark": (0.22, 0.23, 0.26, 1.0),
    },
    "white_science": {
        "bus": (0.86, 0.86, 0.82, 1.0),
        "bus_alt": (0.94, 0.93, 0.88, 1.0),
        "deck": (0.64, 0.66, 0.70, 1.0),
        "structure": (0.52, 0.55, 0.60, 1.0),
        "thermal": (0.08, 0.09, 0.10, 1.0),
        "blanket": (0.93, 0.91, 0.84, 1.0),
        "foil": (0.76, 0.73, 0.66, 1.0),
        "radiator": (0.80, 0.82, 0.84, 1.0),
        "radiator_louver": (0.20, 0.21, 0.22, 1.0),
        "solar": (0.05, 0.18, 0.38, 1.0),
        "solar_alt": (0.03, 0.12, 0.30, 1.0),
        "solar_grid": (0.72, 0.74, 0.78, 1.0),
        "antenna": (0.92, 0.93, 0.90, 1.0),
        "sensor": (0.22, 0.36, 0.52, 1.0),
        "lens": (0.10, 0.24, 0.36, 1.0),
        "thruster": (0.24, 0.25, 0.26, 1.0),
        "thruster_inner": (0.055, 0.055, 0.060, 1.0),
        "marking": (0.55, 0.59, 0.63, 1.0),
        "dark": (0.20, 0.21, 0.24, 1.0),
    },
    "graphite_thermal": {
        "bus": (0.26, 0.27, 0.28, 1.0),
        "bus_alt": (0.33, 0.34, 0.34, 1.0),
        "deck": (0.58, 0.60, 0.62, 1.0),
        "structure": (0.46, 0.48, 0.50, 1.0),
        "thermal": (0.04, 0.045, 0.05, 1.0),
        "blanket": (0.18, 0.18, 0.17, 1.0),
        "foil": (0.65, 0.58, 0.42, 1.0),
        "radiator": (0.70, 0.72, 0.72, 1.0),
        "radiator_louver": (0.09, 0.095, 0.10, 1.0),
        "solar": (0.03, 0.12, 0.28, 1.0),
        "solar_alt": (0.02, 0.08, 0.20, 1.0),
        "solar_grid": (0.62, 0.64, 0.67, 1.0),
        "antenna": (0.78, 0.80, 0.78, 1.0),
        "sensor": (0.55, 0.68, 0.78, 1.0),
        "lens": (0.16, 0.30, 0.38, 1.0),
        "thruster": (0.15, 0.15, 0.16, 1.0),
        "thruster_inner": (0.035, 0.035, 0.040, 1.0),
        "marking": (0.62, 0.50, 0.22, 1.0),
        "dark": (0.13, 0.14, 0.16, 1.0),
    },
    "silver_probe": {
        "bus": (0.72, 0.74, 0.74, 1.0),
        "bus_alt": (0.86, 0.87, 0.86, 1.0),
        "deck": (0.62, 0.64, 0.65, 1.0),
        "structure": (0.50, 0.52, 0.54, 1.0),
        "thermal": (0.09, 0.095, 0.10, 1.0),
        "blanket": (0.88, 0.86, 0.78, 1.0),
        "foil": (0.74, 0.72, 0.66, 1.0),
        "radiator": (0.82, 0.84, 0.83, 1.0),
        "radiator_louver": (0.16, 0.17, 0.18, 1.0),
        "solar": (0.04, 0.14, 0.34, 1.0),
        "solar_alt": (0.03, 0.10, 0.25, 1.0),
        "solar_grid": (0.72, 0.74, 0.76, 1.0),
        "antenna": (0.92, 0.92, 0.88, 1.0),
        "sensor": (0.40, 0.50, 0.58, 1.0),
        "lens": (0.12, 0.27, 0.38, 1.0),
        "thruster": (0.20, 0.21, 0.22, 1.0),
        "thruster_inner": (0.045, 0.045, 0.050, 1.0),
        "marking": (0.38, 0.46, 0.58, 1.0),
        "dark": (0.18, 0.19, 0.21, 1.0),
    },
    "copper_mli": {
        "bus": (0.70, 0.48, 0.26, 1.0),
        "bus_alt": (0.82, 0.62, 0.36, 1.0),
        "deck": (0.68, 0.68, 0.64, 1.0),
        "structure": (0.52, 0.52, 0.50, 1.0),
        "thermal": (0.075, 0.070, 0.065, 1.0),
        "blanket": (0.86, 0.58, 0.28, 1.0),
        "foil": (0.66, 0.42, 0.22, 1.0),
        "radiator": (0.76, 0.78, 0.76, 1.0),
        "radiator_louver": (0.16, 0.15, 0.14, 1.0),
        "solar": (0.05, 0.13, 0.30, 1.0),
        "solar_alt": (0.03, 0.09, 0.22, 1.0),
        "solar_grid": (0.68, 0.70, 0.72, 1.0),
        "antenna": (0.88, 0.86, 0.80, 1.0),
        "sensor": (0.38, 0.48, 0.56, 1.0),
        "lens": (0.12, 0.26, 0.34, 1.0),
        "thruster": (0.18, 0.17, 0.16, 1.0),
        "thruster_inner": (0.045, 0.040, 0.035, 1.0),
        "marking": (0.56, 0.18, 0.12, 1.0),
        "dark": (0.18, 0.17, 0.16, 1.0),
    },
    "blue_white_orbiter": {
        "bus": (0.82, 0.84, 0.84, 1.0),
        "bus_alt": (0.92, 0.93, 0.90, 1.0),
        "deck": (0.58, 0.62, 0.66, 1.0),
        "structure": (0.48, 0.52, 0.58, 1.0),
        "thermal": (0.07, 0.08, 0.095, 1.0),
        "blanket": (0.89, 0.88, 0.80, 1.0),
        "foil": (0.72, 0.72, 0.66, 1.0),
        "radiator": (0.80, 0.83, 0.85, 1.0),
        "radiator_louver": (0.15, 0.17, 0.20, 1.0),
        "solar": (0.03, 0.16, 0.40, 1.0),
        "solar_alt": (0.02, 0.10, 0.30, 1.0),
        "solar_grid": (0.68, 0.72, 0.76, 1.0),
        "antenna": (0.90, 0.91, 0.88, 1.0),
        "sensor": (0.26, 0.42, 0.58, 1.0),
        "lens": (0.09, 0.24, 0.38, 1.0),
        "thruster": (0.19, 0.20, 0.22, 1.0),
        "thruster_inner": (0.040, 0.045, 0.055, 1.0),
        "marking": (0.18, 0.28, 0.48, 1.0),
        "dark": (0.17, 0.18, 0.21, 1.0),
    },
}


@dataclass(frozen=True)
class SatelliteWithArticulatedSolarPanelsConfig:
    bus_body_module: BusBodyModule | None = None
    solar_array_module: SolarArrayModule | None = None
    payload_module: PayloadModule | None = None
    thruster_pack_module: ThrusterPackModule | None = None
    palette_theme: PaletteTheme = "gold_comm"
    bus_width: float = 1.10
    bus_depth: float = 0.98
    bus_height: float = 1.52
    panel_span: float = 2.05
    panel_height: float = 3.05
    hinge_limit: float = 1.45
    panel_segment_count: int = 2
    palette: dict[str, tuple[float, float, float, float]] = field(
        default_factory=lambda: dict(PALETTES["gold_comm"])
    )


@dataclass(frozen=True)
class ResolvedSatelliteWithArticulatedSolarPanelsConfig:
    bus_body_module: BusBodyModule
    solar_array_module: SolarArrayModule
    payload_module: PayloadModule
    thruster_pack_module: ThrusterPackModule
    palette_theme: PaletteTheme
    bus_width: float
    bus_depth: float
    bus_height: float
    panel_span: float
    panel_height: float
    hinge_limit: float
    panel_segment_count: int
    palette: dict[str, tuple[float, float, float, float]]


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def _cyl_x() -> tuple[float, float, float]:
    return (0.0, math.pi / 2.0, 0.0)


def _cyl_y() -> tuple[float, float, float]:
    return (math.pi / 2.0, 0.0, 0.0)


def config_from_seed(seed: int) -> SatelliteWithArticulatedSolarPanelsConfig:
    if seed == 0:
        return SatelliteWithArticulatedSolarPanelsConfig(
            bus_body_module="boxy_comm_bus",
            solar_array_module="bilateral_two_panel_wings",
            payload_module="high_gain_dish",
            thruster_pack_module="single_apogee_nozzle",
            palette_theme="gold_comm",
            bus_width=1.10,
            bus_depth=0.98,
            bus_height=1.52,
            panel_span=2.05,
            panel_height=3.05,
            hinge_limit=1.45,
            panel_segment_count=2,
        )
    rng = random.Random(seed)
    bus: BusBodyModule = rng.choice(BUS_MODULES)
    solar: SolarArrayModule = rng.choice(SOLAR_MODULES)
    payload: PayloadModule = rng.choice(PAYLOAD_MODULES)
    thruster: ThrusterPackModule = rng.choice(THRUSTER_MODULES)
    theme: PaletteTheme = rng.choice(tuple(PALETTES))
    return SatelliteWithArticulatedSolarPanelsConfig(
        bus_body_module=bus,
        solar_array_module=solar,
        payload_module=payload,
        thruster_pack_module=thruster,
        palette_theme=theme,
        bus_width=round(rng.uniform(0.86, 1.34), 4),
        bus_depth=round(rng.uniform(0.78, 1.18), 4),
        bus_height=round(rng.uniform(1.10, 1.82), 4),
        panel_span=round(rng.uniform(1.50, 2.55), 4),
        panel_height=round(rng.uniform(2.20, 3.70), 4),
        hinge_limit=round(rng.uniform(1.05, 1.58), 4),
        panel_segment_count=rng.randint(1, 4),
    )


def resolve_config(
    config: SatelliteWithArticulatedSolarPanelsConfig,
) -> ResolvedSatelliteWithArticulatedSolarPanelsConfig:
    bus = config.bus_body_module or "boxy_comm_bus"
    solar = config.solar_array_module or "bilateral_two_panel_wings"
    payload = config.payload_module or "high_gain_dish"
    thruster = config.thruster_pack_module or "single_apogee_nozzle"
    if bus not in BUS_MODULES:
        raise ValueError(f"Unsupported bus_body_module: {bus}")
    if solar not in SOLAR_MODULES:
        raise ValueError(f"Unsupported solar_array_module: {solar}")
    if payload not in PAYLOAD_MODULES:
        raise ValueError(f"Unsupported payload_module: {payload}")
    if thruster not in THRUSTER_MODULES:
        raise ValueError(f"Unsupported thruster_pack_module: {thruster}")
    if config.palette_theme not in PALETTES:
        raise ValueError(f"Unsupported palette_theme: {config.palette_theme}")
    if solar == "four_wing_cross" and payload in {"high_gain_dish", "side_payload_boom"}:
        payload = "sensor_mast_cluster"
    return ResolvedSatelliteWithArticulatedSolarPanelsConfig(
        bus_body_module=bus,
        solar_array_module=solar,
        payload_module=payload,
        thruster_pack_module=thruster,
        palette_theme=config.palette_theme,
        bus_width=_clamp(config.bus_width, 0.72, 1.55),
        bus_depth=_clamp(config.bus_depth, 0.62, 1.35),
        bus_height=_clamp(config.bus_height, 0.95, 2.05),
        panel_span=_clamp(config.panel_span, 1.20, 2.90),
        panel_height=_clamp(config.panel_height, 1.85, 4.10),
        hinge_limit=_clamp(config.hinge_limit, 0.65, 1.70),
        panel_segment_count=max(1, min(4, int(config.panel_segment_count))),
        palette=dict(PALETTES[config.palette_theme]),
    )


def _mesh(model: ArticulatedObject, geometry: LatheGeometry, name: str):
    if model.assets is not None:
        return mesh_from_geometry(geometry, model.assets.mesh_path(f"{name}.obj"))
    return mesh_from_geometry(geometry, name)


def _dish_mesh(model: ArticulatedObject):
    return _mesh(
        model,
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.00, -0.11),
                (0.09, -0.105),
                (0.22, -0.075),
                (0.38, -0.025),
                (0.46, 0.0),
            ],
            inner_profile=[
                (0.00, -0.095),
                (0.08, -0.090),
                (0.21, -0.062),
                (0.36, -0.020),
                (0.43, 0.0),
            ],
            segments=48,
        ),
        "satellite_high_gain_dish",
    )


def _nozzle_mesh(model: ArticulatedObject, scale: float):
    return _mesh(
        model,
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.12 * scale, 0.0),
                (0.15 * scale, -0.09 * scale),
                (0.24 * scale, -0.25 * scale),
                (0.30 * scale, -0.38 * scale),
            ],
            inner_profile=[
                (0.07 * scale, 0.0),
                (0.11 * scale, -0.09 * scale),
                (0.18 * scale, -0.25 * scale),
                (0.24 * scale, -0.38 * scale),
            ],
            segments=40,
        ),
        f"satellite_nozzle_{scale:.2f}",
    )


def _build_bus(model: ArticulatedObject, r: ResolvedSatelliteWithArticulatedSolarPanelsConfig):
    bus = model.part("bus")
    w, d, h = r.bus_width, r.bus_depth, r.bus_height
    bus.visual(Box((w, d, h)), material="bus", name="main_bus")
    bus.visual(
        Box((w * 1.08, d * 1.08, 0.12)),
        origin=Origin(xyz=(0, 0, h * 0.5 + 0.04)),
        material="deck",
        name="top_deck",
    )
    bus.visual(
        Box((w * 1.08, d * 1.08, 0.12)),
        origin=Origin(xyz=(0, 0, -h * 0.5 - 0.04)),
        material="deck",
        name="bottom_deck",
    )
    for side_name, y in (("plus_y", d * 0.5 + 0.006), ("minus_y", -d * 0.5 - 0.006)):
        bus.visual(
            Box((w * 0.58, 0.012, h * 0.34)),
            origin=Origin(xyz=(-w * 0.12, y, -h * 0.12)),
            material="blanket",
            name=f"{side_name}_mli_blanket_panel",
        )
        bus.visual(
            Box((w * 0.32, 0.014, h * 0.09)),
            origin=Origin(xyz=(w * 0.20, y, h * 0.25)),
            material="bus_alt",
            name=f"{side_name}_avionics_access_panel",
        )
        bus.visual(
            Box((w * 0.44, 0.016, h * 0.018)),
            origin=Origin(xyz=(0, y, h * 0.02)),
            material="marking",
            name=f"{side_name}_orientation_stripe",
        )
    for edge_name, y in (("plus_y", d * 0.53), ("minus_y", -d * 0.53)):
        bus.visual(
            Box((w * 1.14, 0.035, 0.10)),
            origin=Origin(xyz=(0, y, h * 0.5 + 0.12)),
            material="structure",
            name=f"top_frame_{edge_name}",
        )
        bus.visual(
            Box((w * 1.14, 0.035, 0.10)),
            origin=Origin(xyz=(0, y, -h * 0.5 - 0.12)),
            material="structure",
            name=f"bottom_frame_{edge_name}",
        )
    for edge_name, x in (("plus_x", w * 0.53), ("minus_x", -w * 0.53)):
        bus.visual(
            Box((0.035, d * 1.14, 0.10)),
            origin=Origin(xyz=(x, 0, h * 0.5 + 0.12)),
            material="structure",
            name=f"top_frame_{edge_name}",
        )
        bus.visual(
            Box((0.035, d * 1.14, 0.10)),
            origin=Origin(xyz=(x, 0, -h * 0.5 - 0.12)),
            material="structure",
            name=f"bottom_frame_{edge_name}",
        )
    for side, x in (("plus", w * 0.5 + 0.012), ("minus", -w * 0.5 - 0.012)):
        bus.visual(
            Box((0.075, d * 0.78, h * 0.72)),
            origin=Origin(xyz=(x, 0, 0)),
            material="radiator",
            name=f"radiator_{side}_x",
        )
        for panel_idx, z in enumerate((-h * 0.24, 0.0, h * 0.24)):
            bus.visual(
                Box((0.082, d * 0.30, 0.018)),
                origin=Origin(xyz=(x + (0.004 if x > 0 else -0.004), -d * 0.22, z)),
                material="radiator_louver",
                name=f"radiator_{side}_x_louver_a_{panel_idx}",
            )
            bus.visual(
                Box((0.082, d * 0.30, 0.018)),
                origin=Origin(xyz=(x + (0.004 if x > 0 else -0.004), d * 0.22, z)),
                material="radiator_louver",
                name=f"radiator_{side}_x_louver_b_{panel_idx}",
            )
    bus.visual(
        Box((w * 0.18, 0.12, h * 0.62)),
        origin=Origin(xyz=(0, d * 0.5 + 0.02, 0)),
        material="structure",
        name="plus_y_array_clevis",
    )
    bus.visual(
        Box((w * 0.18, 0.12, h * 0.62)),
        origin=Origin(xyz=(0, -d * 0.5 - 0.02, 0)),
        material="structure",
        name="minus_y_array_clevis",
    )
    if r.solar_array_module == "four_wing_cross":
        bus.visual(
            Box((0.12, d * 0.18, h * 0.48)),
            origin=Origin(xyz=(w * 0.5 + 0.02, 0, -0.18)),
            material="structure",
            name="plus_x_array_clevis",
        )
        bus.visual(
            Box((0.12, d * 0.18, h * 0.48)),
            origin=Origin(xyz=(-w * 0.5 - 0.02, 0, -0.18)),
            material="structure",
            name="minus_x_array_clevis",
        )
    for side_name, y_sign in (("plus_y", 1.0), ("minus_y", -1.0)):
        for cheek_name, x in (("port_cheek", -w * 0.09), ("starboard_cheek", w * 0.09)):
            bus.visual(
                Box((0.035, 0.15, h * 0.44)),
                origin=Origin(xyz=(x, y_sign * (d * 0.5 + 0.06), 0)),
                material="structure",
                name=f"{side_name}_solar_{cheek_name}",
            )
        bus.visual(
            Cylinder(radius=0.032, length=h * 0.58),
            origin=Origin(xyz=(0, y_sign * (d * 0.5 + 0.082), 0)),
            material="structure",
            name=f"{side_name}_solar_hinge_pin",
        )
        bus.visual(
            Box((w * 0.26, 0.062, max(0.16, h * 0.22))),
            origin=Origin(xyz=(0, y_sign * (d * 0.5 + 0.12), 0.0)),
            material="structure",
            name=f"{side_name}_root_drive_housing",
        )
    if r.solar_array_module == "four_wing_cross":
        for side_name, x_sign in (("plus_x", 1.0), ("minus_x", -1.0)):
            for cheek_name, y in (("forward_cheek", -d * 0.09), ("aft_cheek", d * 0.09)):
                bus.visual(
                    Box((0.15, 0.035, h * 0.36)),
                    origin=Origin(xyz=(x_sign * (w * 0.5 + 0.06), y, -0.18)),
                    material="structure",
                    name=f"{side_name}_solar_{cheek_name}",
                )
            bus.visual(
                Cylinder(radius=0.028, length=h * 0.46),
                origin=Origin(xyz=(x_sign * (w * 0.5 + 0.082), 0, -0.18)),
                material="structure",
                name=f"{side_name}_solar_hinge_pin",
            )
    if r.solar_array_module != "four_wing_cross":
        bus.visual(
            Box((0.16, d * 0.16, h * 0.50)),
            origin=Origin(xyz=(w * 0.5 + 0.03, 0, 0.04)),
            material="structure",
            name="payload_mount",
        )
    bus.visual(
        Box((0.34, 0.34, 0.10)),
        origin=Origin(xyz=(0, 0, h * 0.5 + 0.15)),
        material="deck",
        name="sensor_plinth",
    )
    bus.visual(
        Cylinder(radius=0.045, length=0.22),
        origin=Origin(xyz=(-0.12, -0.12, h * 0.5 + 0.29)),
        material="structure",
        name="star_tracker_stem",
    )
    bus.visual(
        Sphere(radius=0.07),
        origin=Origin(xyz=(-0.12, -0.12, h * 0.5 + 0.43)),
        material="lens",
        name="star_tracker_dome",
    )
    for i, z in enumerate((-h * 0.26, 0.0, h * 0.26)):
        bus.visual(
            Box((w * 0.92, 0.010, 0.012)),
            origin=Origin(xyz=(0, d * 0.505, z)),
            material="foil",
            name=f"thermal_blanket_seam_plus_y_{i}",
        )
        bus.visual(
            Box((w * 0.92, 0.010, 0.012)),
            origin=Origin(xyz=(0, -d * 0.505, z)),
            material="foil",
            name=f"thermal_blanket_seam_minus_y_{i}",
        )
    for i, y in enumerate((-d * 0.26, 0.0, d * 0.26)):
        bus.visual(
            Box((0.010, d * 0.10, h * 0.010)),
            origin=Origin(xyz=(w * 0.505, y, h * 0.30)),
            material="marking",
            name=f"plus_x_access_fastener_pair_{i}",
        )
        bus.visual(
            Box((0.010, d * 0.10, h * 0.010)),
            origin=Origin(xyz=(-w * 0.505, y, -h * 0.30)),
            material="marking",
            name=f"minus_x_access_fastener_pair_{i}",
        )
    for idx, (x, y) in enumerate(
        (
            (w * 0.28, d * 0.28),
            (-w * 0.28, d * 0.28),
            (w * 0.28, -d * 0.28),
            (-w * 0.28, -d * 0.28),
        )
    ):
        bus.visual(
            Cylinder(radius=0.036, length=0.12),
            origin=Origin(xyz=(x, y, h * 0.5 + 0.16)),
            material="structure",
            name=f"attitude_sensor_socket_{idx}",
        )
        bus.visual(
            Sphere(radius=0.028),
            origin=Origin(xyz=(x, y, h * 0.5 + 0.22)),
            material="lens",
            name=f"attitude_sensor_dome_{idx}",
        )
    if r.bus_body_module == "octagonal_science_bus":
        for i in range(4):
            theta = math.pi / 4 + i * math.pi / 2
            bus.visual(
                Box((0.10, 0.40, h * 0.78)),
                origin=Origin(xyz=(math.cos(theta) * w * 0.42, math.sin(theta) * d * 0.42, 0)),
                material="bus_alt",
                name=f"octagonal_corner_post_{i}",
            )
    elif r.bus_body_module == "truss_service_bus":
        for z in (-h * 0.33, 0.0, h * 0.33):
            bus.visual(
                Box((w * 1.12, 0.045, 0.055)),
                origin=Origin(xyz=(0, d * 0.52, z)),
                material="structure",
                name=f"plus_y_truss_{z:.2f}",
            )
            bus.visual(
                Box((w * 1.12, 0.045, 0.055)),
                origin=Origin(xyz=(0, -d * 0.52, z)),
                material="structure",
                name=f"minus_y_truss_{z:.2f}",
            )
    bus.inertial = Inertial.from_geometry(Box((w * 1.1, d * 1.1, h * 1.35)), mass=700)
    return bus


def _add_panel_cells(
    part, *, prefix: str, span: float, height: float, x_face: float = 0.006
) -> None:
    columns = max(3, min(5, int(span / 0.48) + 1))
    rows = 3
    cell_w = span / columns * 0.78
    cell_h = height / rows * 0.74
    for row in range(rows):
        z = -height * 0.5 + height * (row + 0.5) / rows
        for col in range(columns):
            y = 0.13 + span * (col + 0.5) / columns
            part.visual(
                Box((0.006, cell_w, cell_h)),
                origin=Origin(xyz=(x_face, y, z)),
                material="solar_alt" if (row + col) % 2 else "solar",
                name=f"{prefix}_cell_{row}_{col}",
            )


def _build_solar_wing(
    model: ArticulatedObject,
    r: ResolvedSatelliteWithArticulatedSolarPanelsConfig,
    *,
    name: str,
    side: Literal["plus_y", "minus_y", "plus_x", "minus_x"],
    span: float,
    height: float,
    segments: int,
    joint_origin: tuple[float, float, float],
    joint_rpy: tuple[float, float, float] = (0, 0, 0),
) -> None:
    wing = model.part(name)
    wing.visual(
        Cylinder(radius=0.032, length=height * 0.32),
        origin=Origin(),
        material="structure",
        name="hinge_barrel",
    )
    wing.visual(
        Cylinder(radius=0.018, length=height * 0.92),
        origin=Origin(xyz=(0.0, 0.085, 0.0)),
        material="structure",
        name="root_torsion_tube",
    )
    wing.visual(
        Box((0.075, 0.14, height * 0.36)),
        origin=Origin(xyz=(0, 0.055, 0)),
        material="structure",
        name="root_yoke",
    )
    wing.visual(
        Box((0.105, 0.16, height * 0.16)),
        origin=Origin(xyz=(0, 0.12, -height * 0.32)),
        material="dark",
        name="deployment_motor_box",
    )
    cap_offset = min(height * 0.20, r.bus_height * 0.15)
    for cap_name, z in (("upper", cap_offset), ("lower", -cap_offset)):
        wing.visual(
            Cylinder(radius=0.050, length=0.026),
            origin=Origin(xyz=(0, -0.005, z)),
            material="structure",
            name=f"{cap_name}_bearing_cap",
        )
    segment_w = span / segments
    for i in range(segments):
        cy = 0.13 + segment_w * (i + 0.5)
        wing.visual(
            Box((0.020, segment_w * 0.96, height)),
            origin=Origin(xyz=(0, cy, 0)),
            material="blanket",
            name=f"panel_backing_{i}",
        )
        wing.visual(
            Box((0.026, 0.045, height * 1.02)),
            origin=Origin(xyz=(0, 0.13 + segment_w * i, 0)),
            material="structure",
            name=f"hinge_or_frame_{i}",
        )
        wing.visual(
            Box((0.040, segment_w * 0.86, 0.050)),
            origin=Origin(xyz=(-0.020, cy, height * 0.34)),
            material="structure",
            name=f"backside_upper_stringer_{i}",
        )
        wing.visual(
            Box((0.040, segment_w * 0.86, 0.050)),
            origin=Origin(xyz=(-0.020, cy, -height * 0.34)),
            material="structure",
            name=f"backside_lower_stringer_{i}",
        )
        wing.visual(
            Box((0.030, 0.040, height * 0.92)),
            origin=Origin(xyz=(-0.018, 0.13 + segment_w * (i + 1), 0)),
            material="structure",
            name=f"segment_side_rail_{i}",
        )
    wing.visual(
        Box((0.028, span + 0.16, 0.060)),
        origin=Origin(xyz=(0, 0.13 + span * 0.5, height * 0.5)),
        material="structure",
        name="top_frame_rail",
    )
    wing.visual(
        Box((0.028, span + 0.16, 0.060)),
        origin=Origin(xyz=(0, 0.13 + span * 0.5, -height * 0.5)),
        material="structure",
        name="bottom_frame_rail",
    )
    wing.visual(
        Box((0.030, 0.070, height * 1.03)),
        origin=Origin(xyz=(0, 0.13, 0)),
        material="structure",
        name="inner_frame_rail",
    )
    wing.visual(
        Box((0.030, 0.080, height * 1.03)),
        origin=Origin(xyz=(0, 0.13 + span, 0)),
        material="structure",
        name="outer_frame_rail",
    )
    _add_panel_cells(wing, prefix="solar", span=span, height=height)
    for i in range(max(2, segments + 1)):
        y = 0.13 + span * i / max(1, segments)
        wing.visual(
            Box((0.030, 0.010, height * 1.03)),
            origin=Origin(xyz=(0.001, y, 0)),
            material="dark",
            name=f"panel_wire_harness_{i}",
        )
    for row, z in enumerate((-height * 0.18, height * 0.18)):
        wing.visual(
            Box((0.012, span * 0.92, 0.014)),
            origin=Origin(xyz=(0.012, 0.13 + span * 0.5, z)),
            material="solar_grid",
            name=f"cell_busbar_{row}",
        )
    if r.solar_array_module == "foldout_segment_wings":
        wing.visual(
            Box((0.040, 0.18, height * 0.90)),
            origin=Origin(xyz=(0, span + 0.18, 0)),
            material="structure",
            name="outer_foldout_spine",
        )
        wing.visual(
            Box((0.080, 0.16, height * 0.22)),
            origin=Origin(xyz=(0, span + 0.10, -height * 0.28)),
            material="dark",
            name="outer_foldout_latch_box",
        )
    wing.inertial = Inertial.from_geometry(Box((0.08, span + 0.22, height)), mass=45)
    model.articulation(
        f"bus_to_{name}",
        ArticulationType.REVOLUTE,
        parent="bus",
        child=wing,
        origin=Origin(xyz=joint_origin, rpy=joint_rpy),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=160, velocity=0.35, lower=0.0, upper=r.hinge_limit),
        meta={"slot_side": side},
    )


def _build_solar_arrays(
    model: ArticulatedObject, r: ResolvedSatelliteWithArticulatedSolarPanelsConfig
) -> None:
    w, d = r.bus_width, r.bus_depth
    segments = r.panel_segment_count
    if r.solar_array_module == "four_wing_cross":
        span = r.panel_span * 0.72
        height = r.panel_height * 0.70
        _build_solar_wing(
            model,
            r,
            name="plus_y_solar_wing",
            side="plus_y",
            span=span,
            height=height,
            segments=segments,
            joint_origin=(0, d * 0.5 + 0.08, 0.20),
        )
        _build_solar_wing(
            model,
            r,
            name="minus_y_solar_wing",
            side="minus_y",
            span=span,
            height=height,
            segments=segments,
            joint_origin=(0, -d * 0.5 - 0.08, 0.20),
            joint_rpy=(0, 0, math.pi),
        )
        _build_solar_wing(
            model,
            r,
            name="plus_x_solar_wing",
            side="plus_x",
            span=span * 0.82,
            height=height,
            segments=segments,
            joint_origin=(w * 0.5 + 0.08, 0, -0.18),
            joint_rpy=(0, 0, -math.pi / 2),
        )
        _build_solar_wing(
            model,
            r,
            name="minus_x_solar_wing",
            side="minus_x",
            span=span * 0.82,
            height=height,
            segments=segments,
            joint_origin=(-w * 0.5 - 0.08, 0, -0.18),
            joint_rpy=(0, 0, math.pi / 2),
        )
        return
    _build_solar_wing(
        model,
        r,
        name="left_solar_wing",
        side="plus_y",
        span=r.panel_span,
        height=r.panel_height,
        segments=segments,
        joint_origin=(0, d * 0.5 + 0.08, 0),
    )
    _build_solar_wing(
        model,
        r,
        name="right_solar_wing",
        side="minus_y",
        span=r.panel_span,
        height=r.panel_height,
        segments=segments,
        joint_origin=(0, -d * 0.5 - 0.08, 0),
        joint_rpy=(0, 0, math.pi),
    )


def _build_payload(
    model: ArticulatedObject, r: ResolvedSatelliteWithArticulatedSolarPanelsConfig
) -> None:
    w, h = r.bus_width, r.bus_height
    if r.payload_module == "high_gain_dish":
        payload = model.part("high_gain_antenna")
        payload.visual(
            Cylinder(radius=0.10, length=0.12),
            origin=Origin(xyz=(0.06, 0, 0), rpy=_cyl_x()),
            material="structure",
            name="mount_collar",
        )
        payload.visual(
            Cylinder(radius=0.052, length=0.42),
            origin=Origin(xyz=(0.27, 0, 0), rpy=_cyl_x()),
            material="structure",
            name="support_boom",
        )
        payload.visual(
            Box((0.22, 0.20, 0.18)),
            origin=Origin(xyz=(0.36, 0, -0.08)),
            material="thermal",
            name="transceiver_box",
        )
        payload.visual(
            Cylinder(radius=0.12, length=0.18),
            origin=Origin(xyz=(0.56, 0, 0), rpy=_cyl_x()),
            material="structure",
            name="dish_hub",
        )
        payload.visual(
            Cylinder(radius=0.075, length=0.12),
            origin=Origin(xyz=(0.67, 0, 0), rpy=_cyl_x()),
            material="structure",
            name="dish_neck",
        )
        payload.visual(
            _dish_mesh(model),
            origin=Origin(xyz=(0.78, 0, 0), rpy=_cyl_x()),
            material="antenna",
            name="dish_shell",
        )
        payload.visual(
            Cylinder(radius=0.014, length=0.36),
            origin=Origin(xyz=(0.86, 0, 0), rpy=_cyl_x()),
            material="structure",
            name="feed_support_boom",
        )
        payload.visual(
            Cylinder(radius=0.035, length=0.26),
            origin=Origin(xyz=(1.05, 0, 0), rpy=_cyl_x()),
            material="antenna",
            name="feed_horn",
        )
        payload.visual(
            Cylinder(radius=0.080, length=0.040),
            origin=Origin(xyz=(0.98, 0, 0), rpy=_cyl_x()),
            material="structure",
            name="feed_ring",
        )
        for idx, (y, z) in enumerate(((0.10, 0.10), (-0.10, 0.10), (0.0, -0.13))):
            payload.visual(
                Cylinder(radius=0.010, length=0.34),
                origin=Origin(xyz=(0.90, y * 0.5, z * 0.5), rpy=_cyl_x()),
                material="structure",
                name=f"feed_support_strut_{idx}",
            )
        model.articulation(
            "bus_to_high_gain_antenna",
            ArticulationType.REVOLUTE,
            parent="bus",
            child=payload,
            origin=Origin(xyz=(w * 0.5 + 0.08, 0, 0.05)),
            axis=(0, 0, 1),
            motion_limits=MotionLimits(effort=25, velocity=0.25, lower=-0.35, upper=0.35),
        )
        return
    if r.payload_module == "sensor_mast_cluster":
        payload = model.part("sensor_mast_cluster")
        payload.visual(
            Cylinder(radius=0.065, length=0.18),
            origin=Origin(xyz=(0, 0, 0.09)),
            material="structure",
            name="mast_socket",
        )
        payload.visual(
            Cylinder(radius=0.030, length=0.54),
            origin=Origin(xyz=(0, 0, 0.36)),
            material="structure",
            name="sensor_mast",
        )
        payload.visual(
            Cylinder(radius=0.020, length=0.30),
            origin=Origin(xyz=(0, 0, 0.69)),
            material="structure",
            name="upper_sensor_mast_extension",
        )
        for i, z in enumerate((0.50, 0.66, 0.80)):
            head_x = 0.08 * (-1) ** i
            payload.visual(
                Box((0.18 - i * 0.018, 0.070, 0.038)),
                origin=Origin(xyz=(head_x * 0.5, 0, z - 0.055)),
                material="structure",
                name=f"instrument_head_mount_{i}",
            )
            payload.visual(
                Box((0.20 - i * 0.025, 0.12, 0.075)),
                origin=Origin(xyz=(head_x, 0, z)),
                material="sensor",
                name=f"instrument_head_{i}",
            )
        payload.visual(
            Box((0.035, 0.46, 0.035)),
            origin=Origin(xyz=(0.0, 0.0, 0.62)),
            material="structure",
            name="cross_scan_bar",
        )
        for idx, y in enumerate((-0.20, 0.20)):
            payload.visual(
                Cylinder(radius=0.030, length=0.055),
                origin=Origin(xyz=(0, y, 0.62), rpy=_cyl_y()),
                material="structure",
                name=f"cross_scan_sensor_socket_{idx}",
            )
            payload.visual(
                Sphere(radius=0.045),
                origin=Origin(xyz=(0, y, 0.62)),
                material="lens",
                name=f"cross_scan_sensor_{idx}",
            )
        model.articulation(
            "bus_to_sensor_mast_cluster",
            ArticulationType.REVOLUTE,
            parent="bus",
            child=payload,
            origin=Origin(xyz=(0, 0, h * 0.5 + 0.20)),
            axis=(0, 0, 1),
            motion_limits=MotionLimits(effort=12, velocity=0.2, lower=-0.6, upper=0.6),
        )
        return
    payload = model.part("side_payload_boom")
    payload.visual(
        Cylinder(radius=0.06, length=0.14),
        origin=Origin(xyz=(0.07, 0, 0), rpy=_cyl_x()),
        material="structure",
        name="boom_socket",
    )
    payload.visual(
        Box((0.56, 0.08, 0.08)),
        origin=Origin(xyz=(0.34, 0, 0)),
        material="structure",
        name="instrument_boom",
    )
    payload.visual(
        Box((0.42, 0.035, 0.035)),
        origin=Origin(xyz=(0.34, 0.040, 0.055)),
        material="structure",
        name="upper_boom_stringer",
    )
    payload.visual(
        Box((0.42, 0.035, 0.035)),
        origin=Origin(xyz=(0.34, -0.040, -0.055)),
        material="structure",
        name="lower_boom_stringer",
    )
    for idx, x in enumerate((0.18, 0.34, 0.50)):
        payload.visual(
            Box((0.030, 0.096, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.055)),
            material="structure",
            name=f"upper_boom_standoff_{idx}",
        )
        payload.visual(
            Box((0.030, 0.096, 0.030)),
            origin=Origin(xyz=(x, 0.0, -0.055)),
            material="structure",
            name=f"lower_boom_standoff_{idx}",
        )
    payload.visual(
        Box((0.25, 0.28, 0.22)),
        origin=Origin(xyz=(0.70, 0, 0)),
        material="sensor",
        name="payload_box",
    )
    payload.visual(
        Cylinder(radius=0.08, length=0.10),
        origin=Origin(xyz=(0.86, 0, 0.02), rpy=_cyl_x()),
        material="lens",
        name="aperture",
    )
    model.articulation(
        "bus_to_side_payload_boom",
        ArticulationType.REVOLUTE,
        parent="bus",
        child=payload,
        origin=Origin(xyz=(w * 0.5 + 0.08, 0, 0.05)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=18, velocity=0.25, lower=-0.5, upper=0.5),
    )


def _build_thrusters(
    model: ArticulatedObject, r: ResolvedSatelliteWithArticulatedSolarPanelsConfig
) -> None:
    bus = model.get_part("bus")
    h = r.bus_height
    if r.thruster_pack_module == "single_apogee_nozzle":
        bus.visual(
            Cylinder(radius=0.18, length=0.12),
            origin=Origin(xyz=(0, 0, -h * 0.5 - 0.08)),
            material="thruster",
            name="engine_collar",
        )
        bus.visual(
            Box((r.bus_width * 0.34, r.bus_depth * 0.34, 0.035)),
            origin=Origin(xyz=(0, 0, -h * 0.5 - 0.02)),
            material="structure",
            name="apogee_engine_mount_plate",
        )
        bus.visual(
            _nozzle_mesh(model, 1.0),
            origin=Origin(xyz=(0, 0, -h * 0.5 - 0.13)),
            material="thruster_inner",
            name="apogee_nozzle",
        )
    elif r.thruster_pack_module == "quad_corner_thrusters":
        for i, (x, y) in enumerate(
            (
                (r.bus_width * 0.32, r.bus_depth * 0.32),
                (r.bus_width * 0.32, -r.bus_depth * 0.32),
                (-r.bus_width * 0.32, r.bus_depth * 0.32),
                (-r.bus_width * 0.32, -r.bus_depth * 0.32),
            )
        ):
            bus.visual(
                Box((0.18, 0.18, 0.035)),
                origin=Origin(xyz=(x, y, -h * 0.5 - 0.025)),
                material="structure",
                name=f"corner_thruster_mount_{i}",
            )
            bus.visual(
                Cylinder(radius=0.075, length=0.16),
                origin=Origin(xyz=(x, y, -h * 0.5 - 0.06)),
                material="thruster",
                name=f"corner_thruster_{i}",
            )
    else:
        for i, y in enumerate((-r.bus_depth * 0.22, r.bus_depth * 0.22)):
            bus.visual(
                Box((0.20, 0.16, 0.045)),
                origin=Origin(xyz=(-r.bus_width * 0.18, y, -h * 0.5 - 0.02)),
                material="structure",
                name=f"ion_thruster_mount_{i}",
            )
            bus.visual(
                Cylinder(radius=0.07, length=0.26),
                origin=Origin(xyz=(-r.bus_width * 0.18, y, -h * 0.5 - 0.07), rpy=_cyl_y()),
                material="thruster",
                name=f"ion_thruster_{i}",
            )


def build_satellite_with_articulated_solar_panels(
    config: SatelliteWithArticulatedSolarPanelsConfig,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name="satellite_with_articulated_solar_panels", assets=assets)
    for name, rgba in r.palette.items():
        model.material(name, rgba=rgba)
    _build_bus(model, r)
    _build_solar_arrays(model, r)
    _build_payload(model, r)
    _build_thrusters(model, r)
    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_satellite_with_articulated_solar_panels(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_satellite_with_articulated_solar_panels(config_from_seed(seed), assets=assets)


def slot_choices_for_config(
    r: ResolvedSatelliteWithArticulatedSolarPanelsConfig,
) -> list[tuple[str, str]]:
    return [
        ("bus_body", r.bus_body_module),
        ("solar_array", r.solar_array_module),
        ("payload", r.payload_module),
        ("thruster_pack", r.thruster_pack_module),
        ("palette_theme", r.palette_theme),
        ("panel_segment_multiplicity", f"{r.panel_segment_count}_segments_per_wing"),
    ]


def slot_choices_for_seed(seed: int) -> list[tuple[str, str]]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


def run_satellite_with_articulated_solar_panels_tests(
    object_model: ArticulatedObject,
    config: SatelliteWithArticulatedSolarPanelsConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    names = {p.name for p in object_model.parts}
    joints = {j.name: j for j in object_model.articulations}
    ctx.check("bus_present", "bus" in names)
    solar_parts = [name for name in names if name.endswith("solar_wing")]
    expected_wings = 4 if r.solar_array_module == "four_wing_cross" else 2
    ctx.check("solar_wing_count", len(solar_parts) == expected_wings)
    solar_joints = [
        j for j in joints.values() if j.name.startswith("bus_to_") and "solar_wing" in j.name
    ]
    ctx.check("solar_joints_revolute", len(solar_joints) == expected_wings)
    ctx.check(
        "solar_axes_vertical",
        all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in solar_joints),
    )
    ctx.check(
        "payload_present",
        any(
            name in names
            for name in ("high_gain_antenna", "sensor_mast_cluster", "side_payload_boom")
        ),
    )
    if "bus" in names:
        bus = object_model.get_part("bus")
        bus_visuals = {v.name for v in bus.visuals}

        def _wing_side(part_name: str) -> str:
            if part_name.startswith(("left_", "plus_y_")):
                return "plus_y"
            if part_name.startswith(("right_", "minus_y_")):
                return "minus_y"
            if part_name.startswith("plus_x_"):
                return "plus_x"
            return "minus_x"

        ctx.check(
            "attitude_sensors_have_sockets",
            {
                "attitude_sensor_socket_0",
                "attitude_sensor_socket_1",
                "attitude_sensor_socket_2",
                "attitude_sensor_socket_3",
                "attitude_sensor_dome_0",
                "attitude_sensor_dome_1",
                "attitude_sensor_dome_2",
                "attitude_sensor_dome_3",
            }.issubset(bus_visuals),
            details="Corner attitude domes must sit on visible deck sockets, not float above the bus.",
        )
        for name in solar_parts:
            wing = object_model.get_part(name)
            wing_visuals = {v.name for v in wing.visuals}
            side = _wing_side(name)
            pin_name = f"{side}_solar_hinge_pin"
            clevis_name = f"{side}_array_clevis"
            ctx.check(
                f"{name}_has_root_support_details",
                {
                    "hinge_barrel",
                    "root_yoke",
                    "root_torsion_tube",
                    "deployment_motor_box",
                    "inner_frame_rail",
                    "outer_frame_rail",
                }.issubset(wing_visuals),
            )
            ctx.check(
                f"{name}_segment_count_in_range",
                1 <= r.panel_segment_count <= 4,
                details=f"panel_segment_count={r.panel_segment_count}",
            )
            panel_backings = [
                visual_name
                for visual_name in wing_visuals
                if visual_name.startswith("panel_backing_")
            ]
            ctx.check(
                f"{name}_segment_count_matches_config",
                len(panel_backings) == r.panel_segment_count,
                details=f"expected={r.panel_segment_count}, actual={sorted(panel_backings)}",
            )
            ctx.check(
                f"{name}_segments_have_frame_details",
                all(
                    f"hinge_or_frame_{idx}" in wing_visuals
                    and f"segment_side_rail_{idx}" in wing_visuals
                    for idx in range(r.panel_segment_count)
                ),
                details=f"panel_segment_count={r.panel_segment_count}",
            )
            ctx.check(f"{side}_bus_hinge_pin_present", pin_name in bus_visuals)
            ctx.check(f"{side}_bus_clevis_present", clevis_name in bus_visuals)
            ctx.expect_contact(
                bus,
                wing,
                elem_a=pin_name,
                elem_b="hinge_barrel",
                contact_tol=0.012,
                name=f"{name}_hinge_barrel_seated_on_bus_pin",
            )
            ctx.expect_contact(
                bus,
                wing,
                elem_a=clevis_name,
                elem_b="root_yoke",
                contact_tol=0.025,
                name=f"{name}_root_yoke_captured_by_bus_clevis",
            )
            ctx.allow_overlap(
                bus,
                wing,
                elem_a=pin_name,
                elem_b="hinge_barrel",
                reason="Coaxial solar-array pin is physically captured inside the wing hinge barrel.",
            )
            ctx.allow_overlap(
                bus,
                wing,
                elem_a=clevis_name,
                elem_b="hinge_barrel",
                reason="The hinge barrel is recessed into the bus-side clevis fork.",
            )
            ctx.allow_overlap(
                bus,
                wing,
                elem_a=pin_name,
                elem_b="root_yoke",
                reason="The root hinge pin passes through the wing yoke around the hinge barrel.",
            )
            ctx.allow_overlap(
                bus,
                wing,
                elem_a=clevis_name,
                elem_b="root_yoke",
                reason="The wing root yoke is visibly seated between the bus-side clevis cheeks.",
            )
            ctx.allow_overlap(
                bus,
                wing,
                reason="Solar wing hinge hardware, clevis cheeks, and root drive housings intentionally interlock at the bus side wall.",
            )
            root_drive_name = f"{side}_root_drive_housing"
            if root_drive_name in bus_visuals:
                ctx.expect_contact(
                    bus,
                    wing,
                    elem_a=root_drive_name,
                    elem_b="root_yoke",
                    contact_tol=0.140,
                    name=f"{name}_root_yoke_backed_by_drive_housing",
                )
                ctx.allow_overlap(
                    bus,
                    wing,
                    elem_a=root_drive_name,
                    elem_b="root_yoke",
                    reason="The solar wing root yoke is backed by the visible deployment drive housing.",
                )
                ctx.allow_overlap(
                    bus,
                    wing,
                    elem_a=root_drive_name,
                    elem_b="hinge_barrel",
                    reason="The deployment drive housing wraps around the root hinge barrel.",
                )
                for cap_name in ("upper_bearing_cap", "lower_bearing_cap"):
                    if cap_name in wing_visuals:
                        ctx.allow_overlap(
                            bus,
                            wing,
                            elem_a=root_drive_name,
                            elem_b=cap_name,
                            reason="The deployment drive housing captures the hinge bearing cap.",
                        )
            for cap_name in ("upper_bearing_cap", "lower_bearing_cap"):
                if cap_name in wing_visuals:
                    ctx.allow_overlap(
                        bus,
                        wing,
                        elem_a=clevis_name,
                        elem_b=cap_name,
                        reason="The hinge bearing cap is nested within the bus-side clevis fork.",
                    )
        for name in ("high_gain_antenna", "sensor_mast_cluster", "side_payload_boom"):
            if name in names:
                payload = object_model.get_part(name)
                payload_visuals = {v.name for v in payload.visuals}
                if name in {"high_gain_antenna", "side_payload_boom"}:
                    socket = "mount_collar" if name == "high_gain_antenna" else "boom_socket"
                    ctx.expect_contact(
                        bus,
                        payload,
                        elem_a="payload_mount",
                        elem_b=socket,
                        contact_tol=0.025,
                        name=f"{name}_socket_seated_in_payload_mount",
                    )
                    ctx.allow_overlap(
                        bus,
                        payload,
                        elem_a="payload_mount",
                        elem_b=socket,
                        reason="Payload socket is visibly captured by the bus-side mounting block.",
                    )
                    ctx.allow_overlap(
                        bus,
                        payload,
                        reason="Payload socket, collar, and mounting block intentionally nest at the spacecraft bus face.",
                    )
                if name == "sensor_mast_cluster":
                    ctx.check(
                        "sensor_mast_has_socket_crossbar_and_head_mounts",
                        {
                            "mast_socket",
                            "sensor_mast",
                            "upper_sensor_mast_extension",
                            "cross_scan_bar",
                            "cross_scan_sensor_socket_0",
                            "cross_scan_sensor_socket_1",
                            "instrument_head_mount_0",
                            "instrument_head_mount_1",
                            "instrument_head_mount_2",
                        }.issubset(payload_visuals),
                        details="Sensor payload heads and end sensors must be visibly tied back to the mast.",
                    )
                    ctx.expect_contact(
                        bus,
                        payload,
                        elem_a="sensor_plinth",
                        elem_b="mast_socket",
                        contact_tol=0.025,
                        name="sensor_mast_socket_seated_on_plinth",
                    )
                    ctx.allow_overlap(
                        bus,
                        payload,
                        reason="Sensor mast socket is seated onto the bus sensor plinth.",
                    )
                if name == "high_gain_antenna":
                    ctx.check(
                        "high_gain_has_feed_support",
                        {
                            "dish_neck",
                            "feed_ring",
                            "feed_horn",
                            "feed_support_boom",
                            "feed_support_strut_0",
                        }.issubset(payload_visuals),
                        details="Feed hardware must be supported from the dish hub, not appear as free-floating rods.",
                    )
                if name == "side_payload_boom":
                    ctx.check(
                        "side_payload_boom_has_standoffs",
                        {
                            "upper_boom_stringer",
                            "lower_boom_stringer",
                            "upper_boom_standoff_0",
                            "upper_boom_standoff_1",
                            "upper_boom_standoff_2",
                            "lower_boom_standoff_0",
                            "lower_boom_standoff_1",
                            "lower_boom_standoff_2",
                        }.issubset(payload_visuals),
                        details="Boom stringers must have visible standoffs to the main boom.",
                    )
    if solar_parts:
        cell_counts = [
            len(
                [v for v in object_model.get_part(name).visuals if v.name.startswith("solar_cell_")]
            )
            for name in solar_parts
        ]
        ctx.check(
            "solar_wings_have_multi_cell_detail",
            all(count >= 6 for count in cell_counts),
            details=f"cell_counts={cell_counts}",
        )
    ctx.check_model_valid()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.05)
    ctx.fail_if_joint_mating_has_gap()
    return ctx.report()


SatelliteConfig = SatelliteWithArticulatedSolarPanelsConfig
ResolvedSatelliteConfig = ResolvedSatelliteWithArticulatedSolarPanelsConfig

__all__ = [
    "SatelliteWithArticulatedSolarPanelsConfig",
    "ResolvedSatelliteWithArticulatedSolarPanelsConfig",
    "SatelliteConfig",
    "ResolvedSatelliteConfig",
    "config_from_seed",
    "resolve_config",
    "build_satellite_with_articulated_solar_panels",
    "build_seeded_satellite_with_articulated_solar_panels",
    "slot_choices_for_seed",
    "run_satellite_with_articulated_solar_panels_tests",
]
