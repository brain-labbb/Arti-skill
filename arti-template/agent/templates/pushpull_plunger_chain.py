"""Modular procedural template for push-pull plunger chains."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, replace
from typing import Literal

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

__modular__ = True

SupportStyle = Literal["compact_bridge", "fork_sleeve", "rail_bed", "bored_stack"]
PlungerProfile = Literal["rectangular_rod", "round_stepped", "carried_sleeve", "stepped_ejector"]
TopologyFamily = Literal["single_tab", "single_lever", "two_stage_clevis", "three_stage_clevis"]
TerminalOutput = Literal["hinged_tab", "fork_lever", "terminal_clevis", "push_pad"]
MaterialStyle = Literal["dark_shop", "zinc_black", "green_bronze"]

TOPOLOGIES: tuple[TopologyFamily, ...] = (
    "single_tab",
    "single_lever",
    "two_stage_clevis",
    "three_stage_clevis",
)
MATERIAL_STYLES: tuple[MaterialStyle, ...] = ("dark_shop", "zinc_black", "green_bronze")

PALETTES: dict[MaterialStyle, dict[str, tuple[float, float, float, float]]] = {
    "dark_shop": {
        "frame": (0.22, 0.24, 0.26, 1.0),
        "guide": (0.56, 0.58, 0.60, 1.0),
        "rod": (0.76, 0.78, 0.80, 1.0),
        "accent": (0.78, 0.42, 0.14, 1.0),
        "pin": (0.07, 0.075, 0.08, 1.0),
    },
    "zinc_black": {
        "frame": (0.08, 0.085, 0.09, 1.0),
        "guide": (0.68, 0.70, 0.73, 1.0),
        "rod": (0.12, 0.13, 0.14, 1.0),
        "accent": (0.58, 0.20, 0.14, 1.0),
        "pin": (0.50, 0.52, 0.55, 1.0),
    },
    "green_bronze": {
        "frame": (0.18, 0.32, 0.23, 1.0),
        "guide": (0.63, 0.50, 0.28, 1.0),
        "rod": (0.72, 0.74, 0.70, 1.0),
        "accent": (0.88, 0.58, 0.15, 1.0),
        "pin": (0.10, 0.13, 0.10, 1.0),
    },
}


@dataclass(frozen=True)
class PushPullPlungerChainConfig:
    topology: TopologyFamily | None = None
    material_style: MaterialStyle = "dark_shop"
    scale: float = 1.0
    stroke_scale: float = 1.0
    stage_spacing: float = 0.36
    rod_radius: float = 0.012
    name: str = "pushpull_plunger_chain"


@dataclass(frozen=True)
class ResolvedConfig:
    topology: TopologyFamily
    support_style: SupportStyle
    plunger_profile: PlungerProfile
    terminal_output: TerminalOutput
    material_style: MaterialStyle
    stage_count: int
    scale: float
    stroke_scale: float
    stage_spacing: float
    rod_radius: float
    axis_z: float
    name: str


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _pick(value, choices):
    return value if value in choices else choices[0]


def config_from_seed(seed: int) -> PushPullPlungerChainConfig:
    rng = random.Random(seed)
    return PushPullPlungerChainConfig(
        topology=rng.choice(TOPOLOGIES),
        material_style=rng.choice(MATERIAL_STYLES),
        scale=rng.uniform(0.86, 1.22),
        stroke_scale=rng.uniform(0.80, 1.20),
        stage_spacing=rng.uniform(0.31, 0.43),
        rod_radius=rng.uniform(0.009, 0.016),
        name=f"seeded_pushpull_plunger_chain_{seed}",
    )


def resolve_config(config: PushPullPlungerChainConfig | None = None) -> ResolvedConfig:
    cfg = config or PushPullPlungerChainConfig()
    topology = _pick(cfg.topology, TOPOLOGIES)
    material_style = _pick(cfg.material_style, MATERIAL_STYLES)
    if topology == "single_tab":
        support_style: SupportStyle = "compact_bridge"
        plunger_profile: PlungerProfile = "rectangular_rod"
        terminal_output: TerminalOutput = "hinged_tab"
        stage_count = 1
    elif topology == "single_lever":
        support_style = "fork_sleeve"
        plunger_profile = "round_stepped"
        terminal_output = "fork_lever"
        stage_count = 1
    elif topology == "two_stage_clevis":
        support_style = "bored_stack"
        plunger_profile = "stepped_ejector"
        terminal_output = "terminal_clevis"
        stage_count = 2
    else:
        support_style = "rail_bed"
        plunger_profile = "carried_sleeve"
        terminal_output = "terminal_clevis"
        stage_count = 3
    scale = _clamp(cfg.scale, 0.75, 1.35)
    stage_spacing = _clamp(cfg.stage_spacing, 0.26, 0.50)
    axis_z = 0.075 * scale
    if support_style == "rail_bed":
        stage_spacing = max(stage_spacing, 0.62)
        axis_z = 0.165 * scale
    elif support_style == "bored_stack":
        stage_spacing = max(stage_spacing, 0.42)
        axis_z = 0.110 * scale
    return ResolvedConfig(
        topology=topology,
        support_style=support_style,
        plunger_profile=plunger_profile,
        terminal_output=terminal_output,
        material_style=material_style,
        stage_count=stage_count,
        scale=scale,
        stroke_scale=_clamp(cfg.stroke_scale, 0.70, 1.30),
        stage_spacing=stage_spacing,
        rod_radius=_clamp(cfg.rod_radius, 0.007, 0.020),
        axis_z=axis_z,
        name=cfg.name or "pushpull_plunger_chain",
    )


def with_overrides(
    config: PushPullPlungerChainConfig, **kwargs: object
) -> PushPullPlungerChainConfig:
    return replace(config, **kwargs)


def slot_choices_for_config(config: PushPullPlungerChainConfig | ResolvedConfig):
    r = config if isinstance(config, ResolvedConfig) else resolve_config(config)
    return (
        ("support", r.support_style),
        ("plunger_profile", r.plunger_profile),
        ("topology", r.topology),
        ("stage_count", f"{r.stage_count}_stage"),
        ("terminal_output", r.terminal_output),
        ("material_palette", r.material_style),
    )


def slot_choices_for_seed(seed: int):
    return slot_choices_for_config(config_from_seed(seed))


def _mat(model: ArticulatedObject, r: ResolvedConfig, key: str):
    cache = model.meta.setdefault("_material_cache", {})
    name = f"pushpull_{key}"
    if name not in cache:
        cache[name] = model.material(name, rgba=PALETTES[r.material_style][key])
    return cache[name]


def _x_cylinder(
    part, *, name: str, x: float, radius: float, length: float, material, z: float = 0.0
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_cylinder(
    part, *, name: str, xyz: tuple[float, float, float], radius: float, length: float, material
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_support(model: ArticulatedObject, r: ResolvedConfig):
    frame = model.part("support_frame")
    frame_mat = _mat(model, r, "frame")
    guide_mat = _mat(model, r, "guide")
    s = r.scale
    z = r.axis_z
    if r.support_style == "compact_bridge":
        frame.visual(
            Box((0.18 * s, 0.070 * s, 0.016 * s)),
            origin=Origin(xyz=(0.02 * s, 0.0, 0.008 * s)),
            material=frame_mat,
            name="foot",
        )
        frame.visual(
            Box((0.145 * s, 0.064 * s, 0.014 * s)),
            origin=Origin(xyz=(0.02 * s, 0.0, z + 0.026 * s)),
            material=guide_mat,
            name="top_bar",
        )
        for x in (-0.052 * s, 0.082 * s):
            for y in (-0.026 * s, 0.026 * s):
                frame.visual(
                    Box((0.014 * s, 0.010 * s, 0.112 * s)),
                    origin=Origin(xyz=(x, y, 0.072 * s)),
                    material=frame_mat,
                    name=f"post_{x:.2f}_{y:.2f}",
                )
        frame.visual(
            Box((0.012 * s, 0.052 * s, 0.030 * s)),
            origin=Origin(xyz=(0.095 * s, 0.0, z)),
            material=frame_mat,
            name="front_ear_bridge",
        )
    elif r.support_style == "fork_sleeve":
        frame.visual(
            Box((0.32 * s, 0.240 * s, 0.018 * s)),
            origin=Origin(xyz=(0.05 * s, 0.0, 0.009 * s)),
            material=frame_mat,
            name="base_bridge",
        )
        for y in (-0.034 * s, 0.034 * s):
            frame.visual(
                Box((0.18 * s, 0.012 * s, 0.064 * s)),
                origin=Origin(xyz=(0.035 * s, y, 0.046 * s)),
                material=frame_mat,
                name=f"fork_cheek_{y:.2f}",
            )
        _x_cylinder(
            frame,
            name="guide_sleeve",
            x=0.014 * s,
            radius=0.023 * s,
            length=0.130 * s,
            material=guide_mat,
            z=z,
        )
        frame.visual(
            Box((0.120 * s, 0.060 * s, z)),
            origin=Origin(xyz=(0.014 * s, 0.0, z / 2.0)),
            material=frame_mat,
            name="sleeve_saddle",
        )
        for y in (0.067 * s, 0.105 * s):
            frame.visual(
                Box((0.030 * s, 0.016 * s, 0.170 * s)),
                origin=Origin(xyz=(0.185 * s, y, 0.085 * s)),
                material=frame_mat,
                name=f"lever_ear_{y:.2f}",
            )
    elif r.support_style == "rail_bed":
        frame.visual(
            Box((1.08 * s, 0.230 * s, 0.034 * s)),
            origin=Origin(xyz=(0.18 * s, 0.0, 0.017 * s)),
            material=frame_mat,
            name="long_base_plate",
        )
        frame.visual(
            Box((1.00 * s, 0.048 * s, 0.032 * s)),
            origin=Origin(xyz=(0.20 * s, 0.0, 0.050 * s)),
            material=guide_mat,
            name="center_rail",
        )
        for y in (-0.084 * s, 0.084 * s):
            frame.visual(
                Box((0.96 * s, 0.024 * s, 0.034 * s)),
                origin=Origin(xyz=(0.20 * s, y, 0.051 * s)),
                material=guide_mat,
                name=f"side_rail_{y:.2f}",
            )
        _x_cylinder(
            frame,
            name="fixed_guide_sleeve",
            x=-0.28 * s,
            radius=0.038 * s,
            length=0.185 * s,
            material=guide_mat,
            z=z,
        )
        pedestal_h = max(0.070 * s, z - 0.038 * s)
        frame.visual(
            Box((0.19 * s, 0.090 * s, pedestal_h)),
            origin=Origin(xyz=(-0.28 * s, 0.0, pedestal_h / 2.0)),
            material=frame_mat,
            name="sleeve_pedestal",
        )
    else:
        frame.visual(
            Box((0.46 * s, 0.135 * s, 0.020 * s)),
            origin=Origin(xyz=(0.02 * s, 0.0, 0.010 * s)),
            material=frame_mat,
            name="bored_base",
        )
        frame.visual(
            Box((0.024 * s, 0.105 * s, 0.112 * s)),
            origin=Origin(xyz=(-0.17 * s, 0.0, 0.076 * s)),
            material=frame_mat,
            name="rear_bored_plate",
        )
        for x, radius, length in (
            (-0.06 * s, 0.032 * s, 0.23 * s),
            (0.12 * s, 0.023 * s, 0.19 * s),
        ):
            _x_cylinder(
                frame,
                name=f"half_guide_{x:.2f}",
                x=x,
                radius=radius,
                length=length,
                material=guide_mat,
                z=z,
            )
        frame.visual(
            Box((0.024 * s, 0.090 * s, 0.104 * s)),
            origin=Origin(xyz=(0.235 * s, 0.0, 0.072 * s)),
            material=frame_mat,
            name="front_yoke",
        )
    return frame


def _add_plunger_part(model: ArticulatedObject, r: ResolvedConfig, index: int):
    part = model.part(f"plunger_{index}")
    rod_mat = _mat(model, r, "rod")
    guide_mat = _mat(model, r, "guide")
    pin_mat = _mat(model, r, "pin")
    s = r.scale
    rad = r.rod_radius * (0.82**index)
    length = (0.33 + 0.08 * (r.stage_count - index)) * s
    if r.plunger_profile == "rectangular_rod":
        part.visual(
            Box((length, 0.018 * s, 0.014 * s)),
            origin=Origin(xyz=(length / 2.0, 0.0, 0.0)),
            material=rod_mat,
            name="rectangular_shank",
        )
        part.visual(
            Box((0.045 * s, 0.034 * s, 0.026 * s)),
            origin=Origin(xyz=(length + 0.022 * s, 0.0, 0.0)),
            material=rod_mat,
            name="wide_tip",
        )
    elif r.plunger_profile == "round_stepped":
        _x_cylinder(
            part, name="tail", x=0.040 * s, radius=rad * 0.72, length=0.080 * s, material=rod_mat
        )
        _x_cylinder(
            part, name="collar", x=0.088 * s, radius=rad * 1.55, length=0.016 * s, material=pin_mat
        )
        _x_cylinder(
            part,
            name="main_rod",
            x=0.096 * s + length / 2.0,
            radius=rad,
            length=length,
            material=rod_mat,
        )
        _x_cylinder(
            part,
            name="front_tip",
            x=0.096 * s + length + 0.026 * s,
            radius=rad * 1.25,
            length=0.052 * s,
            material=rod_mat,
        )
    elif r.plunger_profile == "carried_sleeve":
        _x_cylinder(part, name="rod", x=length / 2.0, radius=rad, length=length, material=rod_mat)
        _x_cylinder(
            part,
            name="front_collar",
            x=length + 0.020 * s,
            radius=rad * 1.6,
            length=0.040 * s,
            material=pin_mat,
        )
        _x_cylinder(
            part,
            name="carried_guide_sleeve",
            x=length + 0.145 * s,
            radius=rad * 2.35,
            length=0.180 * s,
            material=guide_mat,
        )
        frame_drop_z = -0.042 * s
        part.visual(
            Box((0.120 * s, 0.038 * s, 0.046 * s)),
            origin=Origin(xyz=(length + 0.100 * s, 0.0, frame_drop_z * 0.52)),
            material=pin_mat,
            name="sleeve_drop_web",
        )
    else:
        _x_cylinder(
            part,
            name="stepped_rear",
            x=0.080 * s,
            radius=rad * 1.25,
            length=0.160 * s,
            material=rod_mat,
        )
        _x_cylinder(
            part,
            name="stepped_front",
            x=0.260 * s,
            radius=rad * 0.82,
            length=0.200 * s,
            material=rod_mat,
        )
        _x_cylinder(
            part,
            name="stop_collar",
            x=0.190 * s,
            radius=rad * 1.65,
            length=0.024 * s,
            material=pin_mat,
        )
    return part


def _plunger_front_extent(r: ResolvedConfig, index: int) -> float:
    s = r.scale
    length = (0.33 + 0.08 * (r.stage_count - index)) * s
    if r.plunger_profile == "rectangular_rod":
        return length + 0.045 * s
    if r.plunger_profile == "round_stepped":
        return 0.096 * s + length + 0.052 * s
    if r.plunger_profile == "carried_sleeve":
        return length + 0.235 * s
    return 0.360 * s


def _support_slide_exit_x(r: ResolvedConfig) -> float:
    s = r.scale
    if r.support_style == "compact_bridge":
        return 0.101 * s
    if r.support_style == "fork_sleeve":
        return (0.014 + 0.065) * s
    if r.support_style == "rail_bed":
        return (-0.28 + 0.185 / 2.0) * s
    return (0.235 + 0.024 / 2.0) * s


def _plunger_rear_extent(r: ResolvedConfig) -> float:
    return 0.0


def _add_terminal(model: ArticulatedObject, r: ResolvedConfig, parent, origin_x: float):
    accent = _mat(model, r, "accent")
    pin = _mat(model, r, "pin")
    s = r.scale
    if r.terminal_output == "hinged_tab":
        tab = model.part("hinged_tab")
        _y_cylinder(
            tab,
            name="hinge_barrel",
            xyz=(0.0, 0.0, 0.0),
            radius=0.011 * s,
            length=0.030 * s,
            material=accent,
        )
        tab.visual(
            Box((0.010 * s, 0.040 * s, 0.055 * s)),
            origin=Origin(xyz=(0.014 * s, 0.0, -0.032 * s)),
            material=accent,
            name="tab_plate",
        )
        tab.visual(
            Box((0.028 * s, 0.024 * s, 0.007 * s)),
            origin=Origin(xyz=(0.025 * s, 0.0, -0.062 * s)),
            material=pin,
            name="finger_lip",
        )
        model.articulation(
            "plunger_to_hinged_tab",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=tab,
            origin=Origin(xyz=(origin_x, 0.0, 0.020 * s)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=0.95),
        )
    elif r.terminal_output == "fork_lever":
        lever = model.part("fork_lever")
        _y_cylinder(
            lever,
            name="pivot_hub",
            xyz=(0.0, 0.0, 0.0),
            radius=0.018 * s,
            length=0.026 * s,
            material=accent,
        )
        lever.visual(
            Box((0.095 * s, 0.018 * s, 0.020 * s)),
            origin=Origin(xyz=(0.050 * s, 0.0, 0.030 * s), rpy=(0.0, -0.20, 0.0)),
            material=accent,
            name="cam_arm",
        )
        lever.visual(
            Box((0.030 * s, 0.026 * s, 0.036 * s)),
            origin=Origin(xyz=(0.016 * s, 0.0, 0.006 * s)),
            material=pin,
            name="cam_lobe",
        )
        model.articulation(
            "support_to_fork_lever",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=lever,
            origin=Origin(xyz=(origin_x, 0.086 * s, 0.095 * s)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-0.30, upper=0.82),
        )
    else:
        clevis = model.part("terminal_clevis")
        clevis.visual(
            Box((0.056 * s, 0.064 * s, 0.048 * s)),
            origin=Origin(xyz=(0.028 * s, 0.0, 0.0)),
            material=accent,
            name="clevis_bridge",
        )
        for y in (-0.030 * s, 0.030 * s):
            clevis.visual(
                Box((0.070 * s, 0.012 * s, 0.052 * s)),
                origin=Origin(xyz=(0.085 * s, y, 0.0)),
                material=accent,
                name=f"clevis_ear_{y:.2f}",
            )
        _y_cylinder(
            clevis,
            name="clevis_pin",
            xyz=(0.092 * s, 0.0, 0.0),
            radius=0.006 * s,
            length=0.074 * s,
            material=pin,
        )
        model.articulation(
            "plunger_to_terminal_clevis",
            ArticulationType.FIXED,
            parent=parent,
            child=clevis,
            origin=Origin(xyz=(origin_x, 0.0, 0.0)),
        )


def build_pushpull_plunger_chain(
    config: PushPullPlungerChainConfig | None = None, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=r.name, assets=assets)
    support = _add_support(model, r)
    model.meta["module_slots"] = dict(slot_choices_for_config(r))
    model.meta["interface_specs"] = {
        "support.slide_axis": {"axis": (1.0, 0.0, 0.0), "origin": "guide centerline"},
        "plunger.terminal_mount": {
            "parent": f"plunger_{r.stage_count - 1}",
            "type": r.terminal_output,
        },
    }
    model.meta["mating_contracts"] = [
        {
            "parent": "support_frame",
            "child": "plunger_0",
            "joint": "PRISMATIC",
            "axis": (1.0, 0.0, 0.0),
        }
    ]

    previous = support
    origin_x = _support_slide_exit_x(r) - _plunger_rear_extent(r)
    strokes = []
    for i in range(r.stage_count):
        plunger = _add_plunger_part(model, r, i)
        if i == 0:
            joint_x = origin_x
            joint_z = r.axis_z
        else:
            joint_x = _plunger_front_extent(r, i - 1) - _plunger_rear_extent(r)
            joint_z = 0.0
        joint_origin = Origin(xyz=(joint_x, 0.0, joint_z))
        upper = (0.080 - min(i, 2) * 0.015) * r.stroke_scale * r.scale
        strokes.append(upper)
        model.articulation(
            f"slide_{i}",
            ArticulationType.PRISMATIC,
            parent=previous,
            child=plunger,
            origin=joint_origin,
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=240.0 - i * 35.0, velocity=0.25, lower=0.0, upper=upper
            ),
        )
        previous = plunger

    terminal_x = _plunger_front_extent(r, r.stage_count - 1)
    if r.terminal_output == "fork_lever":
        _add_terminal(model, r, support, 0.185 * r.scale)
    else:
        _add_terminal(model, r, previous, terminal_x)
    model.meta["stroke_limits"] = strokes
    return model


def build_seeded_pushpull_plunger_chain(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_pushpull_plunger_chain(config_from_seed(seed), assets=assets)


def run_pushpull_plunger_chain_tests(
    object_model: ArticulatedObject, config: PushPullPlungerChainConfig | None = None
) -> TestReport:
    ctx = TestContext(object_model)
    prismatic = [
        j for j in object_model.articulations if j.articulation_type == ArticulationType.PRISMATIC
    ]
    revolute = [
        j for j in object_model.articulations if j.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "has at least one prismatic plunger slide",
        bool(prismatic),
        details=f"joints={len(prismatic)}",
    )
    ctx.check(
        "all prismatic slides use the push-pull x axis",
        all(tuple(j.axis) == (1.0, 0.0, 0.0) for j in prismatic),
        details=[tuple(j.axis) for j in prismatic],
    )
    slots = object_model.meta.get("module_slots", {})
    topology = slots.get("topology")
    if topology in {"single_tab", "single_lever"}:
        ctx.check(
            "single output families include a revolute tab or lever",
            bool(revolute),
            details=f"revolute={len(revolute)}",
        )
    if topology in {"two_stage_clevis", "three_stage_clevis"}:
        expected = 2 if topology == "two_stage_clevis" else 3
        ctx.check(
            "telescoping family has expected serial prismatic count",
            len(prismatic) == expected,
            details=f"expected={expected} got={len(prismatic)}",
        )
        ctx.check(
            "telescoping family includes terminal clevis",
            object_model.get_part("terminal_clevis") is not None,
        )
    return ctx.report()


object_model = build_pushpull_plunger_chain()


def run_tests() -> TestReport:
    return run_pushpull_plunger_chain_tests(object_model)


__all__ = [
    "PushPullPlungerChainConfig",
    "build_pushpull_plunger_chain",
    "build_seeded_pushpull_plunger_chain",
    "config_from_seed",
    "object_model",
    "run_pushpull_plunger_chain_tests",
    "run_tests",
    "slot_choices_for_seed",
]
