"""Procedural template for category `box_fan_with_control_knob`."""

from __future__ import annotations

import math
import random
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Literal

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

HousingProfile = Literal["square_box", "wide_twin", "industrial_vent"]
FanLayout = Literal[
    "classic_box",
    "window_twin",
    "industrial_exhaust",
    "pedestal_box",
    "shutter_exhaust",
    "portable_handle",
]
BladeShape = Literal["lofted_curved", "flat_panel", "broad_rotor_geometry"]
GrilleStyle = Literal["wire_grid", "rectangular_bar_guard", "separate_chrome_grille"]
KnobMount = Literal["top_pod", "front_corner", "side_panel", "rear_boss"]
KnobStyle = Literal["plain_cylindrical", "ribbed_bakelite", "skirted_fluted"]
SupportStyle = Literal["none", "feet", "u_tilt_stand", "pedestal_column"]
PanelLayout = Literal["none", "side_expansion_pair", "four_shutters"]
HubStyle = Literal["simple_cylinder", "layered_cap", "spherical_nose"]
MaterialStyle = Literal["ivory", "graphite", "industrial"]
MotorPodStyle = Literal["cap_only", "pod_spider_shaft", "deep_drum"]


@dataclass(frozen=True)
class LayoutSpec:
    housing_profile: HousingProfile
    rotor_count: int
    support_style: SupportStyle
    panel_layout: PanelLayout
    blade_count_choices: tuple[int, ...]
    blade_shapes: tuple[BladeShape, ...]
    blade_weights: tuple[float, ...]
    grille_styles: tuple[GrilleStyle, ...]
    knob_mounts: tuple[KnobMount, ...]
    material_styles: tuple[MaterialStyle, ...]
    motor_pod_styles: tuple[MotorPodStyle, ...]
    snap_grille_prob: float


LAYOUT_SPECS: dict[FanLayout, LayoutSpec] = {
    "classic_box": LayoutSpec(
        housing_profile="square_box",
        rotor_count=1,
        support_style="feet",
        panel_layout="none",
        blade_count_choices=(3, 4, 5, 6),
        blade_shapes=("lofted_curved", "flat_panel", "broad_rotor_geometry"),
        blade_weights=(0.42, 0.38, 0.20),
        grille_styles=("wire_grid", "separate_chrome_grille"),
        knob_mounts=("top_pod", "front_corner", "side_panel"),
        material_styles=("ivory", "graphite"),
        motor_pod_styles=("pod_spider_shaft", "cap_only"),
        snap_grille_prob=0.35,
    ),
    "window_twin": LayoutSpec(
        housing_profile="wide_twin",
        rotor_count=2,
        support_style="feet",
        panel_layout="side_expansion_pair",
        blade_count_choices=(4, 5, 6, 7, 8),
        blade_shapes=("lofted_curved", "flat_panel"),
        blade_weights=(0.52, 0.48),
        grille_styles=("wire_grid", "separate_chrome_grille"),
        knob_mounts=("top_pod", "side_panel"),
        material_styles=("ivory", "graphite"),
        motor_pod_styles=("pod_spider_shaft",),
        snap_grille_prob=0.0,
    ),
    "industrial_exhaust": LayoutSpec(
        housing_profile="industrial_vent",
        rotor_count=1,
        support_style="none",
        panel_layout="none",
        blade_count_choices=(5, 6, 7, 8),
        blade_shapes=("broad_rotor_geometry", "flat_panel"),
        blade_weights=(0.62, 0.38),
        grille_styles=("rectangular_bar_guard", "separate_chrome_grille"),
        knob_mounts=("side_panel", "rear_boss"),
        material_styles=("industrial", "graphite"),
        motor_pod_styles=("deep_drum", "pod_spider_shaft"),
        snap_grille_prob=0.0,
    ),
    "pedestal_box": LayoutSpec(
        housing_profile="square_box",
        rotor_count=1,
        support_style="pedestal_column",
        panel_layout="none",
        blade_count_choices=(3, 4, 5, 6),
        blade_shapes=("lofted_curved", "flat_panel"),
        blade_weights=(0.58, 0.42),
        grille_styles=("wire_grid", "separate_chrome_grille"),
        knob_mounts=("top_pod", "side_panel"),
        material_styles=("ivory", "graphite"),
        motor_pod_styles=("pod_spider_shaft", "cap_only"),
        snap_grille_prob=0.25,
    ),
    "shutter_exhaust": LayoutSpec(
        housing_profile="industrial_vent",
        rotor_count=1,
        support_style="feet",
        panel_layout="four_shutters",
        blade_count_choices=(5, 6, 7, 8),
        blade_shapes=("broad_rotor_geometry", "flat_panel"),
        blade_weights=(0.56, 0.44),
        grille_styles=("rectangular_bar_guard", "separate_chrome_grille"),
        knob_mounts=("side_panel", "rear_boss"),
        material_styles=("industrial", "graphite"),
        motor_pod_styles=("pod_spider_shaft", "deep_drum"),
        snap_grille_prob=0.0,
    ),
    "portable_handle": LayoutSpec(
        housing_profile="square_box",
        rotor_count=1,
        support_style="feet",
        panel_layout="none",
        blade_count_choices=(3, 4, 5),
        blade_shapes=("lofted_curved", "flat_panel"),
        blade_weights=(0.50, 0.50),
        grille_styles=("wire_grid", "separate_chrome_grille"),
        knob_mounts=("top_pod", "side_panel"),
        material_styles=("ivory", "graphite"),
        motor_pod_styles=("pod_spider_shaft", "cap_only"),
        snap_grille_prob=0.45,
    ),
}

HOUSING_WIDTH_RANGES: dict[HousingProfile, tuple[float, float]] = {
    "square_box": (0.42, 0.62),
    "wide_twin": (0.78, 1.12),
    "industrial_vent": (0.48, 0.84),
}
HOUSING_HEIGHT_RANGES: dict[HousingProfile, tuple[float, float]] = {
    "square_box": (0.42, 0.64),
    "wide_twin": (0.42, 0.62),
    "industrial_vent": (0.50, 0.90),
}
BLADE_PROFILE_DEFAULTS: dict[BladeShape, dict[str, float]] = {
    "lofted_curved": {
        "pitch_deg": 11.0,
        "inner_ratio": 0.24,
        "outer_ratio": 0.94,
        "chord_ratio": 0.16,
        "sweep_deg": 11.0,
    },
    "flat_panel": {
        "pitch_deg": 2.0,
        "inner_ratio": 0.26,
        "outer_ratio": 0.84,
        "chord_ratio": 0.18,
        "sweep_deg": 0.0,
    },
    "broad_rotor_geometry": {
        "pitch_deg": 7.0,
        "inner_ratio": 0.24,
        "outer_ratio": 0.92,
        "chord_ratio": 0.22,
        "sweep_deg": 13.0,
    },
}
PALETTES: dict[MaterialStyle, dict[str, tuple[float, float, float, float]]] = {
    "ivory": {
        "body": (0.86, 0.83, 0.74, 1.0),
        "wire": (0.72, 0.72, 0.70, 1.0),
        "blade": (0.25, 0.27, 0.29, 0.75),
        "knob": (0.10, 0.09, 0.08, 1.0),
        "rubber": (0.04, 0.04, 0.04, 1.0),
    },
    "graphite": {
        "body": (0.18, 0.19, 0.20, 1.0),
        "wire": (0.56, 0.58, 0.60, 1.0),
        "blade": (0.35, 0.44, 0.48, 0.68),
        "knob": (0.82, 0.78, 0.62, 1.0),
        "rubber": (0.02, 0.02, 0.02, 1.0),
    },
    "industrial": {
        "body": (0.48, 0.52, 0.54, 1.0),
        "wire": (0.20, 0.22, 0.22, 1.0),
        "blade": (0.24, 0.25, 0.24, 1.0),
        "knob": (0.12, 0.10, 0.08, 1.0),
        "rubber": (0.05, 0.05, 0.05, 1.0),
    },
}


@dataclass(frozen=True)
class BoxFanWithControlKnobConfig:
    housing_width: float | None = None
    housing_height: float | None = None
    housing_depth: float = 0.16
    layout: FanLayout | None = None
    housing_profile: HousingProfile = "square_box"
    rotor_count: int = 1
    blade_count: int = 5
    blade_shape: BladeShape = "lofted_curved"
    blade_pitch_deg: float | None = None
    blade_inner_radius_ratio: float | None = None
    blade_outer_radius_ratio: float | None = None
    blade_chord_ratio: float | None = None
    blade_sweep_deg: float | None = None
    grille_style: GrilleStyle = "wire_grid"
    grille_density: int = 9
    knob_mount: KnobMount = "front_corner"
    knob_style: KnobStyle = "plain_cylindrical"
    knob_range_deg: float = 120.0
    support_style: SupportStyle = "feet"
    tilt_range_deg: float = 24.0
    height_adjust_range: float = 0.12
    panel_layout: PanelLayout = "none"
    panel_range_deg: float = 75.0
    hub_style: HubStyle = "layered_cap"
    material_style: MaterialStyle = "ivory"
    motor_pod_style: MotorPodStyle | None = None
    snap_front_grille: bool | None = None
    name: str = "reference_box_fan_with_control_knob"


@dataclass(frozen=True)
class ResolvedBoxFanWithControlKnobConfig:
    housing_width: float
    housing_height: float
    housing_depth: float
    frame_thickness: float
    layout: FanLayout
    housing_profile: HousingProfile
    rotor_count: int
    blade_count: int
    blade_shape: BladeShape
    blade_pitch_rad: float
    blade_inner_radius_ratio: float
    blade_outer_radius_ratio: float
    blade_chord_ratio: float
    blade_sweep_rad: float
    grille_style: GrilleStyle
    grille_density: int
    knob_mount: KnobMount
    knob_style: KnobStyle
    knob_range_rad: float
    knob_origin: tuple[float, float, float]
    knob_axis: tuple[float, float, float]
    support_style: SupportStyle
    tilt_range_rad: float
    height_adjust_range: float
    panel_layout: PanelLayout
    panel_range_rad: float
    hub_style: HubStyle
    material_style: MaterialStyle
    motor_pod_style: MotorPodStyle
    snap_front_grille: bool
    rotor_radius: float
    rotor_centers: tuple[tuple[float, float, float], ...]
    name: str


def _infer_layout(config: BoxFanWithControlKnobConfig) -> FanLayout:
    if config.layout is not None:
        if config.layout not in LAYOUT_SPECS:
            raise ValueError(f"Unsupported layout: {config.layout}")
        return config.layout
    if config.housing_profile == "wide_twin":
        return "window_twin"
    if config.panel_layout == "four_shutters":
        return "shutter_exhaust"
    if config.support_style == "pedestal_column":
        return "pedestal_box"
    if config.housing_profile == "industrial_vent":
        return "industrial_exhaust"
    return "classic_box"


def config_from_seed(seed: int) -> BoxFanWithControlKnobConfig:
    rng = random.Random(seed)
    layout: FanLayout = rng.choices(
        (
            "classic_box",
            "window_twin",
            "industrial_exhaust",
            "pedestal_box",
            "shutter_exhaust",
            "portable_handle",
        ),
        weights=(0.34, 0.18, 0.18, 0.12, 0.10, 0.08),
        k=1,
    )[0]
    spec = LAYOUT_SPECS[layout]
    profile = spec.housing_profile
    rotor_count = spec.rotor_count
    w0, w1 = HOUSING_WIDTH_RANGES[profile]
    h0, h1 = HOUSING_HEIGHT_RANGES[profile]
    blade_shape: BladeShape = rng.choices(
        spec.blade_shapes,
        weights=spec.blade_weights,
        k=1,
    )[0]
    blade_count = rng.choice(spec.blade_count_choices)
    blade_defaults = BLADE_PROFILE_DEFAULTS[blade_shape]
    knob_mount: KnobMount = rng.choice(spec.knob_mounts)
    motor_pod_style: MotorPodStyle = rng.choice(spec.motor_pod_styles)
    snap_front_grille = rng.random() < spec.snap_grille_prob
    # snap grille is incompatible with front_corner knob (grille would block it)
    if snap_front_grille and knob_mount == "front_corner":
        non_front = [m for m in spec.knob_mounts if m != "front_corner"]
        knob_mount = rng.choice(non_front) if non_front else knob_mount
        if knob_mount == "front_corner":
            snap_front_grille = False
    return BoxFanWithControlKnobConfig(
        housing_width=round(rng.uniform(w0, w1), 3),
        housing_height=round(rng.uniform(h0, h1), 3),
        housing_depth=round(rng.uniform(0.13, 0.20), 3),
        layout=layout,
        housing_profile=profile,
        rotor_count=rotor_count,
        blade_count=blade_count,
        blade_shape=blade_shape,
        blade_pitch_deg=round(
            rng.uniform(blade_defaults["pitch_deg"] - 2.2, blade_defaults["pitch_deg"] + 2.2), 1
        ),
        blade_inner_radius_ratio=round(
            rng.uniform(
                blade_defaults["inner_ratio"] - 0.025, blade_defaults["inner_ratio"] + 0.025
            ),
            3,
        ),
        blade_outer_radius_ratio=round(
            rng.uniform(
                blade_defaults["outer_ratio"] - 0.025, blade_defaults["outer_ratio"] + 0.025
            ),
            3,
        ),
        blade_chord_ratio=round(
            rng.uniform(
                blade_defaults["chord_ratio"] - 0.025, blade_defaults["chord_ratio"] + 0.025
            ),
            3,
        ),
        blade_sweep_deg=round(
            rng.uniform(
                max(-2.0, blade_defaults["sweep_deg"] - 3.0), blade_defaults["sweep_deg"] + 3.0
            ),
            1,
        ),
        grille_style=rng.choice(spec.grille_styles),
        grille_density=rng.choice((7, 9, 11, 13)),
        knob_mount=knob_mount,
        knob_style=rng.choice(("plain_cylindrical", "ribbed_bakelite", "skirted_fluted")),
        knob_range_deg=round(rng.uniform(120.0, 275.0), 1),
        support_style=spec.support_style,
        tilt_range_deg=round(rng.uniform(18.0, 34.0), 1),
        height_adjust_range=round(rng.uniform(0.08, 0.18), 3),
        panel_layout=spec.panel_layout,
        panel_range_deg=round(rng.uniform(60.0, 85.0), 1),
        hub_style=rng.choice(("simple_cylinder", "layered_cap", "spherical_nose")),
        material_style=rng.choice(spec.material_styles),
        motor_pod_style=motor_pod_style,
        snap_front_grille=snap_front_grille,
        name=f"seeded_box_fan_{seed}",
    )


def resolve_config(config: BoxFanWithControlKnobConfig) -> ResolvedBoxFanWithControlKnobConfig:
    layout = _infer_layout(config)
    layout_spec = LAYOUT_SPECS[layout]
    layout_locked = config.layout is not None
    housing_profile = layout_spec.housing_profile if layout_locked else config.housing_profile
    rotor_count = layout_spec.rotor_count if layout_locked else config.rotor_count
    support_style = layout_spec.support_style if layout_locked else config.support_style
    panel_layout = layout_spec.panel_layout if layout_locked else config.panel_layout
    blade_shape = (
        config.blade_shape
        if config.blade_shape in layout_spec.blade_shapes
        else layout_spec.blade_shapes[0]
    )
    grille_style = (
        config.grille_style
        if config.grille_style in layout_spec.grille_styles
        else layout_spec.grille_styles[0]
    )
    knob_mount = (
        config.knob_mount
        if config.knob_mount in layout_spec.knob_mounts
        else layout_spec.knob_mounts[0]
    )
    material_style = (
        config.material_style
        if config.material_style in layout_spec.material_styles
        else layout_spec.material_styles[0]
    )
    motor_pod_style: MotorPodStyle = (
        config.motor_pod_style
        if config.motor_pod_style is not None
        else layout_spec.motor_pod_styles[0]
    )
    snap_front_grille: bool = (
        config.snap_front_grille if config.snap_front_grille is not None else False
    )
    # snap grille incompatible with front_corner knob
    if snap_front_grille and knob_mount == "front_corner":
        knob_mount = layout_spec.knob_mounts[0]
        if knob_mount == "front_corner":
            snap_front_grille = False

    blade_count = config.blade_count
    if layout_locked and blade_count not in layout_spec.blade_count_choices:
        blade_count = min(
            max(blade_count, min(layout_spec.blade_count_choices)),
            max(layout_spec.blade_count_choices),
        )

    if housing_profile not in HOUSING_WIDTH_RANGES:
        raise ValueError(f"Unsupported housing_profile: {housing_profile}")
    if rotor_count not in (1, 2):
        raise ValueError("rotor_count must be 1 or 2")
    if housing_profile == "wide_twin" and rotor_count != 2:
        raise ValueError("wide_twin housing_profile requires rotor_count=2")
    if blade_count < 3 or blade_count > 8:
        raise ValueError("blade_count must be in [3, 8]")
    if blade_shape not in ("lofted_curved", "flat_panel", "broad_rotor_geometry"):
        raise ValueError(f"Unsupported blade_shape: {blade_shape}")
    if grille_style not in ("wire_grid", "rectangular_bar_guard", "separate_chrome_grille"):
        raise ValueError(f"Unsupported grille_style: {grille_style}")
    if knob_mount not in ("top_pod", "front_corner", "side_panel", "rear_boss"):
        raise ValueError(f"Unsupported knob_mount: {knob_mount}")
    if config.knob_style not in ("plain_cylindrical", "ribbed_bakelite", "skirted_fluted"):
        raise ValueError(f"Unsupported knob_style: {config.knob_style}")
    if support_style not in ("none", "feet", "u_tilt_stand", "pedestal_column"):
        raise ValueError(f"Unsupported support_style: {support_style}")
    if panel_layout not in ("none", "side_expansion_pair", "four_shutters"):
        raise ValueError(f"Unsupported panel_layout: {panel_layout}")
    if config.hub_style not in ("simple_cylinder", "layered_cap", "spherical_nose"):
        raise ValueError(f"Unsupported hub_style: {config.hub_style}")
    if material_style not in PALETTES:
        raise ValueError(f"Unsupported material_style: {material_style}")
    if motor_pod_style not in ("cap_only", "pod_spider_shaft", "deep_drum"):
        raise ValueError(f"Unsupported motor_pod_style: {motor_pod_style}")

    w0, w1 = HOUSING_WIDTH_RANGES[housing_profile]
    h0, h1 = HOUSING_HEIGHT_RANGES[housing_profile]
    width = config.housing_width if config.housing_width is not None else (w0 + w1) * 0.5
    height = config.housing_height if config.housing_height is not None else (h0 + h1) * 0.5
    width = max(w0 * 0.95, min(w1 * 1.05, width))
    height = max(h0 * 0.95, min(h1 * 1.05, height))
    depth = max(0.11, min(0.22, config.housing_depth))
    frame_t = max(0.035, min(0.075, min(width, height) * 0.095))
    radius = min(height * 0.34, (width / rotor_count) * 0.31)
    blade_defaults = BLADE_PROFILE_DEFAULTS[blade_shape]
    pitch_deg = (
        blade_defaults["pitch_deg"] if config.blade_pitch_deg is None else config.blade_pitch_deg
    )
    inner_ratio = (
        blade_defaults["inner_ratio"]
        if config.blade_inner_radius_ratio is None
        else config.blade_inner_radius_ratio
    )
    outer_ratio = (
        blade_defaults["outer_ratio"]
        if config.blade_outer_radius_ratio is None
        else config.blade_outer_radius_ratio
    )
    chord_ratio = (
        blade_defaults["chord_ratio"]
        if config.blade_chord_ratio is None
        else config.blade_chord_ratio
    )
    sweep_deg = (
        blade_defaults["sweep_deg"] if config.blade_sweep_deg is None else config.blade_sweep_deg
    )
    inner_ratio = max(0.18, min(0.42, inner_ratio))
    outer_ratio = max(inner_ratio + 0.28, min(0.98, outer_ratio))
    chord_ratio = max(0.08, min(0.30, chord_ratio))
    centers: tuple[tuple[float, float, float], ...]
    if rotor_count == 2:
        centers = ((-width * 0.23, 0.0, 0.0), (width * 0.23, 0.0, 0.0))
    else:
        centers = ((0.0, 0.0, 0.0),)

    front_y = depth * 0.5 + 0.014
    if knob_mount == "top_pod":
        knob_origin = (width * 0.28, 0.0, height * 0.5 + 0.040)
        knob_axis = (0.0, 0.0, 1.0)
    elif knob_mount == "side_panel":
        knob_origin = (width * 0.5 + 0.012, 0.0, height * 0.10)
        knob_axis = (1.0, 0.0, 0.0)
    elif knob_mount == "rear_boss":
        knob_origin = (width * 0.27, -front_y, height * 0.24)
        knob_axis = (0.0, -1.0, 0.0)
    else:
        knob_origin = (width * 0.34, front_y, height * 0.32)
        knob_axis = (0.0, 1.0, 0.0)

    return ResolvedBoxFanWithControlKnobConfig(
        housing_width=width,
        housing_height=height,
        housing_depth=depth,
        frame_thickness=frame_t,
        layout=layout,
        housing_profile=housing_profile,
        rotor_count=rotor_count,
        blade_count=blade_count,
        blade_shape=blade_shape,
        blade_pitch_rad=math.radians(max(-5.0, min(18.0, pitch_deg))),
        blade_inner_radius_ratio=inner_ratio,
        blade_outer_radius_ratio=outer_ratio,
        blade_chord_ratio=chord_ratio,
        blade_sweep_rad=math.radians(max(-8.0, min(20.0, sweep_deg))),
        grille_style=grille_style,
        grille_density=max(5, min(15, config.grille_density)),
        knob_mount=knob_mount,
        knob_style=config.knob_style,
        knob_range_rad=math.radians(max(45.0, min(300.0, config.knob_range_deg))),
        knob_origin=knob_origin,
        knob_axis=knob_axis,
        support_style=support_style,
        tilt_range_rad=math.radians(max(0.0, min(45.0, config.tilt_range_deg))),
        height_adjust_range=max(0.03, min(0.22, config.height_adjust_range)),
        panel_layout=panel_layout,
        panel_range_rad=math.radians(max(20.0, min(90.0, config.panel_range_deg))),
        hub_style=config.hub_style,
        material_style=material_style,
        motor_pod_style=motor_pod_style,
        snap_front_grille=snap_front_grille,
        rotor_radius=radius,
        rotor_centers=centers,
        name=config.name,
    )


def _joint_meta(
    joint_type: ArticulationType,
    axis: tuple[float, float, float],
    origin: tuple[float, float, float],
    joint_range: tuple[float | None, float | None],
) -> dict[str, object]:
    return {"type": joint_type.value, "axis": axis, "origin": origin, "range": joint_range}


def _add_segmented_ring(
    part,
    name_prefix: str,
    cx: float,
    ring_y: float,
    cz: float,
    radius: float,
    material,
    segments: int,
) -> None:
    seg_len = math.tau * radius / segments * 0.82
    for i in range(segments):
        angle = i * math.tau / segments
        part.visual(
            Box((0.005, 0.005, seg_len)),
            origin=Origin(
                xyz=(
                    cx + math.sin(angle) * radius,
                    ring_y,
                    cz + math.cos(angle) * radius,
                ),
                rpy=(0.0, angle + math.pi / 2.0, 0.0),
            ),
            material=material,
            name=f"{name_prefix}_seg_{i}",
        )


def _add_housing(
    housing,
    r: ResolvedBoxFanWithControlKnobConfig,
    assets: AssetContext,
    *,
    body_mat,
    wire_mat,
    rubber_mat,
    pointer_mat,
) -> None:
    w, h, d, t = r.housing_width, r.housing_height, r.housing_depth, r.frame_thickness
    housing.visual(
        Box((t, d, h)),
        origin=Origin(xyz=(-w / 2 + t / 2, 0.0, 0.0)),
        material=body_mat,
        name="left_side_wall",
    )
    housing.visual(
        Box((t, d, h)),
        origin=Origin(xyz=(w / 2 - t / 2, 0.0, 0.0)),
        material=body_mat,
        name="right_side_wall",
    )
    housing.visual(
        Box((w, d, t)),
        origin=Origin(xyz=(0.0, 0.0, h / 2 - t / 2)),
        material=body_mat,
        name="top_wall",
    )
    housing.visual(
        Box((w, d, t)),
        origin=Origin(xyz=(0.0, 0.0, -h / 2 + t / 2)),
        material=body_mat,
        name="bottom_wall",
    )
    if r.housing_profile == "wide_twin":
        housing.visual(
            Box((t * 0.55, d, h - 2 * t)), origin=Origin(), material=body_mat, name="center_divider"
        )

    # --- Motor pod + rotor seat connector for each rotor ---
    for rotor_i, center in enumerate(r.rotor_centers):
        cx, _, cz = center
        housing.visual(
            Cylinder(radius=r.rotor_radius * 0.24, length=0.0025),
            origin=Origin(xyz=(cx, 0.0, cz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber_mat,
            name=f"rotor_mount_collar_{rotor_i}",
        )
        # Front face of the motor pod, used to anchor the visible drive shaft so the
        # rotor never appears to float inside the housing.
        pod_front_y: float
        if r.motor_pod_style == "pod_spider_shaft":
            pod_radius = r.rotor_radius * 0.30
            pod_len = d * 0.32
            # Mostly outside rear face, overlapping 8% of depth into housing
            pod_cy = -(d * 0.5) - pod_len * 0.5 + d * 0.08
            pod_front_y = pod_cy + pod_len * 0.5
            housing.visual(
                Cylinder(radius=pod_radius, length=pod_len),
                origin=Origin(xyz=(cx, pod_cy, cz), rpy=(math.pi / 2, 0.0, 0.0)),
                material=rubber_mat,
                name=f"motor_pod_{rotor_i}",
            )
        elif r.motor_pod_style == "deep_drum":
            pod_radius = r.rotor_radius * 0.36
            pod_len = d * 0.44
            pod_cy = -(d * 0.5) - pod_len * 0.5 + d * 0.10
            pod_front_y = pod_cy + pod_len * 0.5
            housing.visual(
                Cylinder(radius=pod_radius, length=pod_len),
                origin=Origin(xyz=(cx, pod_cy, cz), rpy=(math.pi / 2, 0.0, 0.0)),
                material=rubber_mat,
                name=f"motor_pod_{rotor_i}",
            )
        else:  # cap_only
            cap_len = d * 0.20
            cap_cy = -0.018 - cap_len * 0.5 + 0.001
            pod_front_y = cap_cy + cap_len * 0.5
            housing.visual(
                Cylinder(radius=r.rotor_radius * 0.30, length=cap_len),
                origin=Origin(
                    xyz=(cx, cap_cy, cz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=rubber_mat,
                name=f"rear_motor_cap_{rotor_i}",
            )
            housing.visual(
                Box((r.rotor_radius * 0.90, 0.006, 0.006)),
                origin=Origin(xyz=(cx, -d * 0.5 - 0.006, cz)),
                material=wire_mat,
                name=f"rear_motor_support_horizontal_{rotor_i}",
            )
            housing.visual(
                Box((0.006, 0.006, r.rotor_radius * 0.90)),
                origin=Origin(xyz=(cx, -d * 0.5 - 0.006, cz)),
                material=wire_mat,
                name=f"rear_motor_support_vertical_{rotor_i}",
            )

        # Drive shaft: bridge motor pod front face to a deterministic rotor-side
        # contact collar location so shaft insertion remains stable across styles.
        thickness = max(d * 0.40, 0.032)
        rotor_hub_back = -thickness * 0.50 - 0.006
        shaft_back = pod_front_y - 0.003
        shaft_front = rotor_hub_back + 0.004
        if shaft_front > shaft_back + 0.002:
            shaft_len = shaft_front - shaft_back
            shaft_cy = (shaft_back + shaft_front) * 0.5
            housing.visual(
                Cylinder(radius=max(0.006, r.rotor_radius * 0.05), length=shaft_len),
                origin=Origin(xyz=(cx, shaft_cy, cz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rubber_mat,
                name=f"motor_shaft_{rotor_i}",
            )

    # --- Knob mount structure (static housing part) ---
    ko = r.knob_origin
    if r.knob_mount == "top_pod":
        housing.visual(
            Box((0.20, d * 0.62, 0.036)),
            origin=Origin(xyz=(ko[0], 0.0, h * 0.5 + 0.010)),
            material=body_mat,
            name="top_control_pod",
        )
        housing.visual(
            Cylinder(radius=0.032, length=0.007),
            origin=Origin(xyz=(ko[0], ko[1], h * 0.5 + 0.010)),
            material=wire_mat,
            name="top_knob_bezel",
        )
    elif r.knob_mount == "side_panel":
        housing.visual(
            Cylinder(radius=0.032, length=0.008),
            origin=Origin(
                xyz=(ko[0] - 0.006, ko[1], ko[2]),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=body_mat,
            name="side_knob_boss",
        )
    elif r.knob_mount == "rear_boss":
        housing.visual(
            Cylinder(radius=0.032, length=0.014),
            origin=Origin(
                xyz=(ko[0], ko[1] + 0.007, ko[2]),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=body_mat,
            name="rear_knob_boss",
        )
    else:  # front_corner
        housing.visual(
            Box((0.068, 0.014, 0.068)),
            origin=Origin(xyz=(ko[0], ko[1] - 0.007, ko[2])),
            material=body_mat,
            name="front_control_panel",
        )

    # Bezel ring + tick marks for ribbed/skirted knob styles
    if r.knob_style in {"ribbed_bakelite", "skirted_fluted"}:
        if r.knob_axis == (0.0, 0.0, 1.0):
            bezel_center = tuple(k - 0.020 * a for k, a in zip(ko, r.knob_axis))
            radial_a: tuple[float, float, float] = (1.0, 0.0, 0.0)
            radial_b: tuple[float, float, float] = (0.0, 1.0, 0.0)
        elif r.knob_axis == (1.0, 0.0, 0.0):
            bezel_center = tuple(k - 0.022 * a for k, a in zip(ko, r.knob_axis))
            radial_a = (0.0, 1.0, 0.0)
            radial_b = (0.0, 0.0, 1.0)
        else:
            # front/rear Y-axis mounts
            bezel_center = tuple(k - 0.022 * a for k, a in zip(ko, r.knob_axis))
            radial_a = (1.0, 0.0, 0.0)
            radial_b = (0.0, 0.0, 1.0)

        # Lightweight bead-ring bezel to avoid heavy overlap with the knob mesh while
        # still remaining attached to housing geometry.
        for i in range(8):
            ang = i * math.tau / 8
            housing.visual(
                Sphere(radius=0.0018),
                origin=Origin(
                    xyz=(
                        bezel_center[0]
                        + radial_a[0] * math.cos(ang) * 0.028
                        + radial_b[0] * math.sin(ang) * 0.028,
                        bezel_center[1]
                        + radial_a[1] * math.cos(ang) * 0.028
                        + radial_b[1] * math.sin(ang) * 0.028,
                        bezel_center[2]
                        + radial_a[2] * math.cos(ang) * 0.028
                        + radial_b[2] * math.sin(ang) * 0.028,
                    )
                ),
                material=wire_mat,
                name=f"knob_bezel_{i}",
            )

    if r.housing_profile == "industrial_vent":
        for z in (-0.24, -0.12, 0.0, 0.12, 0.24):
            if abs(z) < h * 0.45:
                housing.visual(
                    Box((w - 2 * t, 0.010, 0.012)),
                    origin=Origin(xyz=(0.0, d / 2 + 0.006, z)),
                    material=wire_mat,
                    name=f"industrial_front_louver_{z:.2f}",
                )
    if r.support_style == "feet":
        for i, x in enumerate((-w * 0.24, w * 0.24)):
            housing.visual(
                Box((w * 0.22, d * 0.92, 0.024)),
                origin=Origin(xyz=(x, 0.0, -h / 2 - 0.012)),
                material=body_mat,
                name=f"foot_{i}",
            )
            housing.visual(
                Box((w * 0.18, d * 0.78, 0.006)),
                origin=Origin(xyz=(x, 0.0, -h / 2 - 0.027)),
                material=rubber_mat,
                name=f"foot_pad_{i}",
            )
    if r.support_style == "pedestal_column":
        housing.visual(
            Cylinder(radius=0.040, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, -h * 0.5 + 0.030)),
            material=body_mat,
            name="pedestal_socket_neck",
        )


def _add_torus_ring(
    part,
    name: str,
    cx: float,
    ring_y: float,
    cz: float,
    radius: float,
    tube: float,
    material,
    assets: AssetContext,
    tubular_segments: int = 32,
) -> None:
    """Add a smooth guard ring; mesh_from_geometry converts TorusGeometry → Mesh for validation."""
    try:
        torus_mesh = mesh_from_geometry(
            TorusGeometry(
                radius=radius,
                tube=tube,
                radial_segments=10,
                tubular_segments=tubular_segments,
            ),
            assets.mesh_path(f"{name}.obj"),
        )
        part.visual(
            torus_mesh,
            origin=Origin(xyz=(cx, ring_y, cz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )
    except Exception:
        _add_segmented_ring(
            part, name, cx, ring_y, cz, radius, material, max(12, tubular_segments // 2)
        )


def _add_grille(
    housing,
    r: ResolvedBoxFanWithControlKnobConfig,
    assets: AssetContext,
    *,
    wire_mat,
    skip_front: bool = False,
) -> None:
    front_y = r.housing_depth * 0.5 + 0.002
    rear_y = -r.housing_depth * 0.5 - 0.002
    span_w = r.housing_width - 2.0 * r.frame_thickness + 0.006
    span_h = r.housing_height - 2.0 * r.frame_thickness + 0.006

    faces = [("rear", rear_y)]
    if not skip_front:
        faces.append(("front", front_y))

    for face, y in faces:
        # Outer border frame strips
        housing.visual(
            Box((span_w, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, y, span_h / 2)),
            material=wire_mat,
            name=f"{face}_grille_top",
        )
        housing.visual(
            Box((span_w, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, y, -span_h / 2)),
            material=wire_mat,
            name=f"{face}_grille_bottom",
        )
        housing.visual(
            Box((0.010, 0.006, span_h)),
            origin=Origin(xyz=(-span_w / 2, y, 0.0)),
            material=wire_mat,
            name=f"{face}_grille_left",
        )
        housing.visual(
            Box((0.010, 0.006, span_h)),
            origin=Origin(xyz=(span_w / 2, y, 0.0)),
            material=wire_mat,
            name=f"{face}_grille_right",
        )

        for rotor_i, center in enumerate(r.rotor_centers):
            cx, _, cz = center
            ring_y = y + (-0.004 if face == "front" else 0.004)

            # Concentric guard rings (smooth TorusGeometry, fallback to segments)
            ring_configs = [
                (r.rotor_radius * 1.050, 0.0045, 36),
                (r.rotor_radius * 0.720, 0.0040, 28),
                (r.rotor_radius * 0.430, 0.0035, 20),
                (r.rotor_radius * 0.240, 0.0030, 14),
            ]
            for ring_idx, (ring_rad, tube, n_tub) in enumerate(ring_configs):
                _add_torus_ring(
                    housing,
                    f"{face}_guard_ring_{rotor_i}_{ring_idx}",
                    cx,
                    ring_y,
                    cz,
                    ring_rad,
                    tube,
                    wire_mat,
                    assets,
                    tubular_segments=n_tub,
                )

            # Radial spokes (8 per rotor)
            outer_ring_radius = ring_configs[0][0]
            outer_ring_tube = ring_configs[0][1]
            for spoke_i in range(8):
                angle = spoke_i * math.tau / 8
                # Make spoke tips reach the outer guard ring envelope (no floating gap).
                spoke_len = outer_ring_radius * 2.0 + outer_ring_tube * 2.0
                housing.visual(
                    Box((0.005, 0.005, spoke_len)),
                    origin=Origin(xyz=(cx, ring_y, cz), rpy=(0.0, angle, 0.0)),
                    material=wire_mat,
                    name=f"{face}_spoke_{rotor_i}_{spoke_i}",
                )

        # Wire pattern (style-dependent)
        count = r.grille_density
        if r.grille_style == "rectangular_bar_guard":
            for i in range(count):
                x = -span_w / 2 + i * span_w / (count - 1)
                housing.visual(
                    Box((0.007, 0.007, span_h)),
                    origin=Origin(xyz=(x, y, 0.0)),
                    material=wire_mat,
                    name=f"{face}_bar_{i}",
                )
        elif r.grille_style == "separate_chrome_grille":
            for i in range(count):
                z = -span_h / 2 + i * span_h / (count - 1)
                housing.visual(
                    Box((span_w, 0.005, 0.005)),
                    origin=Origin(xyz=(0.0, y, z)),
                    material=wire_mat,
                    name=f"{face}_chrome_h_{i}",
                )
        else:  # wire_grid
            for i in range(count):
                x = -span_w / 2 + i * span_w / (count - 1)
                z = -span_h / 2 + i * span_h / (count - 1)
                housing.visual(
                    Box((0.004, 0.005, span_h)),
                    origin=Origin(xyz=(x, y, 0.0)),
                    material=wire_mat,
                    name=f"{face}_wire_v_{i}",
                )
                housing.visual(
                    Box((span_w, 0.005, 0.004)),
                    origin=Origin(xyz=(0.0, y, z)),
                    material=wire_mat,
                    name=f"{face}_wire_h_{i}",
                )


def _add_rotor_box_fallback(
    rotor, r: ResolvedBoxFanWithControlKnobConfig, index: int, *, blade_mat, knob_mat
) -> None:
    """Box-primitive fallback rotor when FanRotorGeometry is unavailable."""
    radius = r.rotor_radius
    hub_r = radius * 0.17
    rotor.visual(
        Cylinder(radius=hub_r, length=0.036),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=knob_mat,
        name="hub_body",
    )
    thickness = max(r.housing_depth * 0.40, 0.032)
    rotor.visual(
        Cylinder(radius=max(0.006, r.rotor_radius * 0.05), length=0.016),
        origin=Origin(xyz=(0.0, -thickness * 0.50, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="hub_contact_collar",
    )
    if r.hub_style == "layered_cap":
        rotor.visual(
            Cylinder(radius=hub_r * 0.75, length=0.050),
            origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            material=knob_mat,
            name="hub_collar",
        )
        # Rear shaft collar (visible connection toward motor pod)
        rotor.visual(
            Cylinder(radius=hub_r * 0.52, length=0.034),
            origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            material=knob_mat,
            name="hub_shaft_rear",
        )
    elif r.hub_style == "spherical_nose":
        rotor.visual(
            Sphere(radius=hub_r * 0.72),
            origin=Origin(xyz=(0.0, 0.024, 0.0)),
            material=knob_mat,
            name="nose_cap",
        )
    for blade_i in range(r.blade_count):
        angle = blade_i * math.tau / r.blade_count
        chord = radius * r.blade_chord_ratio
        length = radius * (r.blade_outer_radius_ratio - r.blade_inner_radius_ratio)
        radial_center = radius * (
            r.blade_inner_radius_ratio
            + (r.blade_outer_radius_ratio - r.blade_inner_radius_ratio) * 0.5
        )
        thickness = 0.009 if r.blade_shape == "lofted_curved" else 0.012
        rotor.visual(
            Box((chord, thickness, length)),
            origin=Origin(
                xyz=(math.sin(angle) * radial_center, 0.0, math.cos(angle) * radial_center),
                rpy=(r.blade_pitch_rad, angle, 0.0),
            ),
            material=blade_mat,
            name=f"blade_{index}_{blade_i}",
        )
        if r.blade_shape in {"lofted_curved", "broad_rotor_geometry"}:
            tip_center = radius * (r.blade_outer_radius_ratio - length * 0.12 / radius)
            rotor.visual(
                Box((chord * 0.72, thickness, length * 0.24)),
                origin=Origin(
                    xyz=(
                        math.sin(angle) * tip_center,
                        0.003,
                        math.cos(angle) * tip_center,
                    ),
                    rpy=(r.blade_pitch_rad * 0.55, angle + r.blade_sweep_rad, 0.0),
                ),
                material=blade_mat,
                name=f"blade_swept_tip_{index}_{blade_i}",
            )


def _add_rotor(
    rotor,
    r: ResolvedBoxFanWithControlKnobConfig,
    index: int,
    assets: AssetContext,
    *,
    blade_mat,
    knob_mat,
) -> None:
    radius = r.rotor_radius
    hub_r = radius * 0.17
    blade_shape_map = {
        "lofted_curved": "scimitar",
        "flat_panel": "straight",
        "broad_rotor_geometry": "broad",
    }
    hub_style_map = {
        "simple_cylinder": "flat",
        "layered_cap": "capped",
        "spherical_nose": "spinner",
    }
    thickness = max(r.housing_depth * 0.40, 0.032)
    try:
        geom = FanRotorGeometry(
            outer_radius=radius,
            hub_radius=hub_r,
            blade_count=r.blade_count,
            thickness=thickness,
            blade_pitch_deg=math.degrees(r.blade_pitch_rad),
            blade_sweep_deg=math.degrees(r.blade_sweep_rad),
            blade=FanRotorBlade(
                shape=blade_shape_map.get(r.blade_shape, "straight"),
                camber=0.06 if r.blade_shape == "lofted_curved" else 0.02,
                tip_clearance=0.0,
            ),
            hub=FanRotorHub(
                style=hub_style_map.get(r.hub_style, "flat"),
                rear_collar_height=thickness * 0.18,
                rear_collar_radius=hub_r * 0.76,
            ),
        )
        rotor.visual(
            mesh_from_geometry(geom, assets.mesh_path(f"fan_rotor_mesh_{index}.obj")),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blade_mat,
            name=f"fan_rotor_mesh_{index}",
        )
        rotor.visual(
            Cylinder(radius=r.rotor_radius * 0.18, length=0.0012),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blade_mat,
            name=f"motor_hub_stub_{index}",
        )
        rotor.visual(
            Cylinder(radius=max(0.006, r.rotor_radius * 0.05), length=0.016),
            origin=Origin(xyz=(0.0, -thickness * 0.50, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_mat,
            name="hub_contact_collar",
        )
    except Exception:
        _add_rotor_box_fallback(rotor, r, index, blade_mat=blade_mat, knob_mat=knob_mat)


def _add_knob_box_fallback(
    knob,
    r: ResolvedBoxFanWithControlKnobConfig,
    *,
    axial: tuple[float, float, float],
    radial_a: tuple[float, float, float],
    radial_b: tuple[float, float, float],
    cylinder_rpy: tuple[float, float, float],
    knob_axial_offset: float,
    knob_mat,
    pointer_mat,
) -> None:
    def at(
        axial_offset: float = 0.0, ra: float = 0.0, rb: float = 0.0
    ) -> tuple[float, float, float]:
        return (
            axial[0] * axial_offset + radial_a[0] * ra + radial_b[0] * rb,
            axial[1] * axial_offset + radial_a[1] * ra + radial_b[1] * rb,
            axial[2] * axial_offset + radial_a[2] * ra + radial_b[2] * rb,
        )

    if r.knob_style == "skirted_fluted":
        knob.visual(
            Cylinder(radius=0.036, length=0.010),
            origin=Origin(xyz=at(0.005 + knob_axial_offset), rpy=cylinder_rpy),
            material=knob_mat,
            name="knob_skirt",
        )
        rib_count = 12
    elif r.knob_style == "ribbed_bakelite":
        knob.visual(
            Cylinder(radius=0.026, length=0.026),
            origin=Origin(xyz=at(0.013 + knob_axial_offset), rpy=cylinder_rpy),
            material=knob_mat,
            name="knob_body",
        )
        rib_count = 10
    else:
        knob.visual(
            Cylinder(radius=0.024, length=0.022),
            origin=Origin(xyz=at(0.011 + knob_axial_offset), rpy=cylinder_rpy),
            material=knob_mat,
            name="knob_body",
        )
        rib_count = 0
    for i in range(rib_count):
        a = i * math.tau / rib_count
        knob.visual(
            Box((0.004, 0.006, 0.014)),
            origin=Origin(
                xyz=at(0.018 + knob_axial_offset, math.cos(a) * 0.027, math.sin(a) * 0.027),
                rpy=(0.0, a, 0.0),
            ),
            material=knob_mat,
            name=f"knob_rib_{i}",
        )
    knob.visual(
        Box((0.018, 0.003, 0.005)),
        origin=Origin(xyz=at(0.024 + knob_axial_offset, 0.013, 0.0)),
        material=pointer_mat,
        name="knob_pointer",
    )


def _add_knob(
    knob,
    r: ResolvedBoxFanWithControlKnobConfig,
    assets: AssetContext,
    *,
    knob_mat,
    pointer_mat,
) -> None:
    if r.knob_axis == (0.0, 0.0, 1.0):
        cylinder_rpy = (0.0, 0.0, 0.0)
        knob_rpy = (0.0, 0.0, 0.0)
        axial: tuple[float, float, float] = (0.0, 0.0, 1.0)
        radial_a: tuple[float, float, float] = (1.0, 0.0, 0.0)
        radial_b: tuple[float, float, float] = (0.0, 1.0, 0.0)
    elif r.knob_axis == (1.0, 0.0, 0.0):
        cylinder_rpy = (0.0, math.pi / 2.0, 0.0)
        knob_rpy = (0.0, -math.pi / 2.0, 0.0)
        axial = (1.0, 0.0, 0.0)
        radial_a = (0.0, 1.0, 0.0)
        radial_b = (0.0, 0.0, 1.0)
    elif r.knob_axis[1] < 0.0:  # rear_boss
        cylinder_rpy = (math.pi / 2.0, 0.0, 0.0)
        knob_rpy = (-math.pi / 2.0, 0.0, 0.0)
        axial = (0.0, -1.0, 0.0)
        radial_a = (1.0, 0.0, 0.0)
        radial_b = (0.0, 0.0, 1.0)
    else:  # front_corner, axis = (0, 1, 0)
        cylinder_rpy = (math.pi / 2.0, 0.0, 0.0)
        knob_rpy = (math.pi / 2.0, 0.0, 0.0)
        axial = (0.0, 1.0, 0.0)
        radial_a = (1.0, 0.0, 0.0)
        radial_b = (0.0, 0.0, 1.0)

    knob_axial_offset = 0.0
    if r.knob_mount == "top_pod":
        if r.knob_style == "skirted_fluted":
            knob_axial_offset = -0.004
        else:
            knob_axial_offset = -0.012
    elif r.knob_mount == "side_panel":
        knob_axial_offset = -0.0040

    def at(a: float = 0.0, ra: float = 0.0, rb: float = 0.0) -> tuple[float, float, float]:
        return (
            axial[0] * a + radial_a[0] * ra + radial_b[0] * rb,
            axial[1] * a + radial_a[1] * ra + radial_b[1] * rb,
            axial[2] * a + radial_a[2] * ra + radial_b[2] * rb,
        )

    # Try KnobGeometry (requires cadquery); fall back to box primitives
    try:
        grip_style = {
            "plain_cylindrical": "none",
            "ribbed_bakelite": "ribbed",
            "skirted_fluted": "fluted",
        }.get(r.knob_style, "none")
        body_style = "skirted" if r.knob_style == "skirted_fluted" else "cylindrical"
        edge_r = 0.003 if r.knob_style == "plain_cylindrical" else 0.0
        knob_geom = KnobGeometry(
            diameter=0.046 if r.knob_style != "skirted_fluted" else 0.040,
            height=0.024 if r.knob_style != "skirted_fluted" else 0.020,
            body_style=body_style,
            edge_radius=edge_r,
            skirt=(
                KnobSkirt(diameter=0.060, height=0.010, chamfer=0.002)
                if r.knob_style == "skirted_fluted"
                else None
            ),
            grip=(
                KnobGrip(style=grip_style, count=10, depth=0.003) if grip_style != "none" else None
            ),
            indicator=KnobIndicator(
                style="line", length=0.014, width=0.004, depth=0.001, angle_deg=0.0
            ),
        )
        knob.visual(
            mesh_from_geometry(knob_geom, assets.mesh_path("speed_knob_mesh.obj")),
            origin=Origin(xyz=at(0.012 + knob_axial_offset), rpy=knob_rpy),
            material=knob_mat,
            name="knob_mesh",
        )
    except Exception:
        _add_knob_box_fallback(
            knob,
            r,
            axial=axial,
            radial_a=radial_a,
            radial_b=radial_b,
            cylinder_rpy=cylinder_rpy,
            knob_axial_offset=knob_axial_offset,
            knob_mat=knob_mat,
            pointer_mat=pointer_mat,
        )


def _add_support(
    model: ArticulatedObject, r: ResolvedBoxFanWithControlKnobConfig, *, body_mat, rubber_mat
):
    if r.support_style not in {"u_tilt_stand", "pedestal_column"}:
        return None
    support = model.part("support")
    if r.support_style == "u_tilt_stand":
        support.visual(
            Box((r.housing_width * 1.12, r.housing_depth * 0.72, 0.034)),
            origin=Origin(xyz=(0.0, 0.0, -r.housing_height * 0.62)),
            material=body_mat,
            name="u_base",
        )
        for i, sx in enumerate((-1.0, 1.0)):
            support.visual(
                Box((0.026, 0.050, r.housing_height * 0.68)),
                origin=Origin(
                    xyz=(
                        sx * (r.housing_width * 0.5 + 0.013),
                        -r.housing_depth * 0.56,
                        -r.housing_height * 0.24,
                    )
                ),
                material=body_mat,
                name=f"u_upright_{i}",
            )
    else:
        support.visual(
            Box((r.housing_width * 0.62, r.housing_depth * 0.95, 0.038)),
            origin=Origin(xyz=(0.0, 0.0, -r.housing_height * 0.82)),
            material=body_mat,
            name="pedestal_base",
        )
        support.visual(
            Cylinder(radius=0.032, length=r.housing_height * 0.66),
            origin=Origin(xyz=(0.0, 0.0, -r.housing_height * 0.50)),
            material=body_mat,
            name="lower_column",
        )
        support.visual(
            Box((r.housing_width * 0.22, r.housing_depth * 0.70, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -r.housing_height * 0.84)),
            material=rubber_mat,
            name="rubber_pad",
        )
    return support


def _add_panels(
    model: ArticulatedObject,
    housing,
    r: ResolvedBoxFanWithControlKnobConfig,
    *,
    body_mat,
    wire_mat,
) -> None:
    if r.panel_layout == "none":
        return

    if r.panel_layout == "side_expansion_pair":
        # PRISMATIC sliding side extenders (window fan style)
        panel_w = r.housing_width * 0.22
        panel_extension = r.housing_width * 0.28
        specs = [("panel_left", -1.0, (-1.0, 0.0, 0.0)), ("panel_right", 1.0, (1.0, 0.0, 0.0))]
        for name, sx, axis in specs:
            panel = model.part(name)
            local_cx = sx * (panel_w * 0.5)
            panel.visual(
                Box((panel_w, r.housing_depth * 0.82, r.housing_height * 0.84)),
                origin=Origin(xyz=(local_cx, 0.0, 0.0)),
                material=body_mat,
                name="slide_panel",
            )
            # Guide rail lips on panel top and bottom
            for ez_sign in (-1.0, 1.0):
                ez = ez_sign * (r.housing_height * 0.42 + 0.005)
                panel.visual(
                    Box((panel_w, r.housing_depth * 0.78, 0.008)),
                    origin=Origin(xyz=(local_cx, 0.0, ez)),
                    material=wire_mat,
                    name=f"panel_rail_{'top' if ez_sign > 0 else 'bot'}",
                )
            origin_xyz = (sx * (r.housing_width / 2), 0.0, 0.0)
            model.articulation(
                name + "_slide",
                ArticulationType.PRISMATIC,
                parent=housing,
                child=panel,
                origin=Origin(xyz=origin_xyz),
                axis=axis,
                motion_limits=MotionLimits(
                    effort=8.0, velocity=0.12, lower=0.0, upper=panel_extension
                ),
                meta=_joint_meta(
                    ArticulationType.PRISMATIC, axis, origin_xyz, (0.0, panel_extension)
                ),
            )
        # Guide rail tracks on inner housing walls
        for sx in (-1.0, 1.0):
            for ez in (-r.housing_height * 0.42, r.housing_height * 0.42):
                housing.visual(
                    Box((0.007, r.housing_depth * 0.82, 0.010)),
                    origin=Origin(
                        xyz=(sx * (r.housing_width / 2 - r.frame_thickness - 0.004), 0.0, ez)
                    ),
                    material=wire_mat,
                    name=f"guide_rail_{int(sx)}_{int(ez * 100)}",
                )

    else:  # four_shutters — rear-face outer-edge-hinged damper flaps
        rear_y = -r.housing_depth * 0.5 - 0.014
        span_w = r.housing_width - 2.2 * r.frame_thickness
        span_h = r.housing_height - 2.2 * r.frame_thickness
        shutter_specs = (
            (
                "top",
                (span_w * 0.60, 0.012, span_h * 0.26),
                (0.0, 0.0, -span_h * 0.13),  # local center below hinge
                (0.0, rear_y, span_h * 0.5),  # hinge at top edge
                (1.0, 0.0, 0.0),
            ),
            (
                "bottom",
                (span_w * 0.60, 0.012, span_h * 0.26),
                (0.0, 0.0, span_h * 0.13),  # local center above hinge
                (0.0, rear_y, -span_h * 0.5),  # hinge at bottom edge
                (-1.0, 0.0, 0.0),
            ),
            (
                "left",
                (span_w * 0.20, 0.012, span_h * 0.52),
                (span_w * 0.10, 0.0, 0.0),  # local center to the right of left hinge
                (-span_w * 0.5, rear_y, 0.0),  # hinge at left edge
                (0.0, 0.0, 1.0),
            ),
            (
                "right",
                (span_w * 0.20, 0.012, span_h * 0.52),
                (-span_w * 0.10, 0.0, 0.0),  # local center to the left of right hinge
                (span_w * 0.5, rear_y, 0.0),  # hinge at right edge
                (0.0, 0.0, -1.0),
            ),
        )
        for i, (label, size, local_offset, origin_xyz, axis) in enumerate(shutter_specs):
            panel = model.part(f"shutter_{label}")
            panel.visual(
                Box(size),
                origin=Origin(xyz=local_offset),
                material=body_mat,
                name="shutter_blade",
            )
            # Hinge barrel along the pivot edge
            barrel_len = size[0] if label in {"top", "bottom"} else size[2]
            panel.visual(
                Cylinder(radius=0.004, length=barrel_len),
                origin=Origin(
                    rpy=(0.0, math.pi / 2.0, 0.0) if label in {"top", "bottom"} else (0.0, 0.0, 0.0)
                ),
                material=body_mat,
                name="hinge_barrel",
            )
            model.articulation(
                f"panel_hinge_{i}",
                ArticulationType.REVOLUTE,
                parent=housing,
                child=panel,
                origin=Origin(xyz=origin_xyz),
                axis=axis,
                motion_limits=MotionLimits(
                    effort=1.0, velocity=1.4, lower=0.0, upper=r.panel_range_rad
                ),
                meta=_joint_meta(
                    ArticulationType.REVOLUTE, axis, origin_xyz, (0.0, r.panel_range_rad)
                ),
            )


def _add_carry_handle(
    model: ArticulatedObject,
    housing,
    r: ResolvedBoxFanWithControlKnobConfig,
    *,
    body_mat,
    rubber_mat,
) -> None:
    if r.layout != "portable_handle":
        return
    handle = model.part("carry_handle")
    hinge_span = r.housing_width * 0.66
    hinge_origin = (0.0, -r.housing_depth * 0.5 - 0.004, r.housing_height * 0.5 + 0.004)
    handle.visual(
        Cylinder(radius=0.006, length=hinge_span),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_mat,
        name="handle_hinge_bar",
    )
    for i, sx in enumerate((-1.0, 1.0)):
        handle.visual(
            Box((0.018, 0.016, 0.120)),
            origin=Origin(xyz=(sx * hinge_span * 0.5, 0.0, 0.060)),
            material=body_mat,
            name=f"handle_upright_{i}",
        )
    handle.visual(
        Box((hinge_span, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=body_mat,
        name="handle_top_bar",
    )
    handle.visual(
        Box((hinge_span * 0.42, 0.022, 0.014)),
        origin=Origin(xyz=(0.0, -0.004, 0.132)),
        material=rubber_mat,
        name="handle_rubber_grip",
    )
    axis = (1.0, 0.0, 0.0)
    model.articulation(
        "carry_handle_fold",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=handle,
        origin=Origin(xyz=hinge_origin),
        axis=axis,
        motion_limits=MotionLimits(effort=0.8, velocity=1.2, lower=0.0, upper=math.radians(105.0)),
        meta=_joint_meta(ArticulationType.REVOLUTE, axis, hinge_origin, (0.0, math.radians(105.0))),
    )


def _add_snap_front_grille(
    model: ArticulatedObject,
    housing,
    r: ResolvedBoxFanWithControlKnobConfig,
    assets: AssetContext,
    *,
    wire_mat,
    body_mat,
) -> None:
    if not r.snap_front_grille:
        return
    w, h, d, t = r.housing_width, r.housing_height, r.housing_depth, r.frame_thickness
    span_w = w - 2.2 * t
    span_h = h - 2.2 * t
    front_y = d * 0.5 + 0.010
    # Hinge sits at the top edge of the front opening (inside the housing frame), so the
    # grille hangs down from z=0 to z=-span_h in its local frame and aligns with the
    # opening at rest pose.
    hinge_z = span_h * 0.5
    z_shift = -span_h * 0.5

    if r.snap_front_grille:
        # Tiny hinge-tie anchor to keep the snapped grille connected to the housing in strict qc checks.
        housing.visual(
            Box((0.004, 0.0006, 0.004)),
            origin=Origin(xyz=(0.0, front_y + 0.0024, hinge_z)),
            material=wire_mat,
            name="front_snap_anchor",
        )

    grille = model.part("front_snap_grille")
    # Outer frame (z_shift moves the top edge up to z=0, which is the hinge axis)
    for label, size, xyz in (
        ("top", (span_w, 0.008, t), (0.0, 0.0, span_h * 0.5 + z_shift)),
        ("bot", (span_w, 0.008, t), (0.0, 0.0, -span_h * 0.5 + z_shift)),
        ("left", (t, 0.008, span_h - 2 * t), (-span_w * 0.5, 0.0, z_shift)),
        ("right", (t, 0.008, span_h - 2 * t), (span_w * 0.5, 0.0, z_shift)),
    ):
        grille.visual(
            Box(size), origin=Origin(xyz=xyz), material=body_mat, name=f"grille_frame_{label}"
        )

    # Simplified wire grid
    bar_count = max(5, r.grille_density // 2 + 2)
    for i in range(bar_count):
        x = -span_w / 2 + i * span_w / (bar_count - 1)
        z = -span_h / 2 + i * span_h / (bar_count - 1)
        grille.visual(
            Box((0.005, 0.005, span_h - 2 * t)),
            origin=Origin(xyz=(x, 0.0, z_shift)),
            material=wire_mat,
            name=f"grille_vbar_{i}",
        )
        grille.visual(
            Box((span_w - 2 * t, 0.005, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, z + z_shift)),
            material=wire_mat,
            name=f"grille_hbar_{i}",
        )

    # Guard rings (centered on the rotor, which sits at world z=0 in housing frame)
    for ri, (r_factor, tube, n_tub) in enumerate(
        ((1.04, 0.0045, 36), (0.64, 0.0038, 26), (0.28, 0.0030, 16))
    ):
        ring_rad = r.rotor_radius * r_factor
        _add_torus_ring(
            grille,
            f"snap_grille_ring_{ri}",
            0.0,
            0.0,
            z_shift,
            ring_rad,
            tube,
            wire_mat,
            assets,
            n_tub,
        )

    # Hinge at the top of the front opening, opens upward/forward
    hinge_origin_xyz = (0.0, front_y, hinge_z)
    axis = (1.0, 0.0, 0.0)
    model.articulation(
        "front_grille_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=grille,
        origin=Origin(xyz=hinge_origin_xyz),
        axis=axis,
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=0.0, upper=math.radians(105.0)),
        meta=_joint_meta(
            ArticulationType.REVOLUTE, axis, hinge_origin_xyz, (0.0, math.radians(105.0))
        ),
    )


def build_box_fan(
    config: BoxFanWithControlKnobConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    config = config or BoxFanWithControlKnobConfig()
    r = resolve_config(config)
    assets = assets or AssetContext(Path(tempfile.mkdtemp(prefix="articraft-box-fan-assets-")))
    model = ArticulatedObject(name=r.name, assets=assets)
    model.meta["adopted_source_ids"] = ("S1", "S2", "S3", "S4", "S5", "S6", "S7")
    model.meta["layout"] = r.layout

    palette = PALETTES[r.material_style]
    body_mat = model.material("fan_body", rgba=palette["body"])
    wire_mat = model.material("fan_grille", rgba=palette["wire"])
    blade_mat = model.material("fan_blade", rgba=palette["blade"])
    knob_mat = model.material("fan_knob", rgba=palette["knob"])
    rubber_mat = model.material("fan_rubber", rgba=palette["rubber"])
    pointer_mat = model.material("fan_pointer", rgba=(0.95, 0.92, 0.70, 1.0))

    housing_parent = _add_support(model, r, body_mat=body_mat, rubber_mat=rubber_mat)
    housing = model.part("housing")
    _add_housing(
        housing,
        r,
        assets,
        body_mat=body_mat,
        wire_mat=wire_mat,
        rubber_mat=rubber_mat,
        pointer_mat=pointer_mat,
    )
    _add_grille(housing, r, assets, wire_mat=wire_mat, skip_front=r.snap_front_grille)

    if housing_parent is not None:
        if r.support_style == "u_tilt_stand":
            origin = (0.0, 0.0, 0.0)
            axis = (1.0, 0.0, 0.0)
            model.articulation(
                "housing_tilt",
                ArticulationType.REVOLUTE,
                parent=housing_parent,
                child=housing,
                origin=Origin(xyz=origin),
                axis=axis,
                motion_limits=MotionLimits(
                    effort=10.0, velocity=1.2, lower=-math.radians(12.0), upper=r.tilt_range_rad
                ),
                meta=_joint_meta(
                    ArticulationType.REVOLUTE, axis, origin, (-math.radians(12.0), r.tilt_range_rad)
                ),
            )
        else:
            origin = (0.0, 0.0, r.housing_height * 0.33)
            axis = (0.0, 0.0, 1.0)
            model.articulation(
                "column_extension",
                ArticulationType.PRISMATIC,
                parent=housing_parent,
                child=housing,
                origin=Origin(xyz=origin),
                axis=axis,
                motion_limits=MotionLimits(
                    effort=18.0, velocity=0.10, lower=0.0, upper=r.height_adjust_range
                ),
                meta=_joint_meta(
                    ArticulationType.PRISMATIC, axis, origin, (0.0, r.height_adjust_range)
                ),
            )

    for i, center in enumerate(r.rotor_centers):
        rotor = model.part(f"rotor_{i}")
        _add_rotor(rotor, r, i, assets, blade_mat=blade_mat, knob_mat=knob_mat)
        axis = (0.0, 1.0, 0.0)
        model.articulation(
            f"rotor_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=rotor,
            origin=Origin(xyz=center),
            axis=axis,
            motion_limits=MotionLimits(effort=3.0, velocity=24.0),
            meta=_joint_meta(ArticulationType.CONTINUOUS, axis, center, (None, None)),
        )

    knob = model.part("speed_knob")
    _add_knob(knob, r, assets, knob_mat=knob_mat, pointer_mat=pointer_mat)
    model.articulation(
        "speed_knob_turn",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=knob,
        origin=Origin(xyz=r.knob_origin),
        axis=r.knob_axis,
        motion_limits=MotionLimits(effort=0.6, velocity=2.8, lower=0.0, upper=r.knob_range_rad),
        meta=_joint_meta(
            ArticulationType.REVOLUTE, r.knob_axis, r.knob_origin, (0.0, r.knob_range_rad)
        ),
    )
    _add_panels(model, housing, r, body_mat=body_mat, wire_mat=wire_mat)
    _add_carry_handle(model, housing, r, body_mat=body_mat, rubber_mat=rubber_mat)
    _add_snap_front_grille(model, housing, r, assets, wire_mat=wire_mat, body_mat=body_mat)
    return model


def build_seeded_box_fan(seed: int, *, assets: AssetContext | None = None) -> ArticulatedObject:
    return build_box_fan(config_from_seed(seed), assets=assets)


def with_overrides(
    config: BoxFanWithControlKnobConfig, **kwargs: object
) -> BoxFanWithControlKnobConfig:
    return replace(config, **kwargs)


def run_box_fan_tests(
    object_model: ArticulatedObject, config: BoxFanWithControlKnobConfig
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "identity parts present",
        {"housing", "speed_knob"}.issubset(part_names),
        details=str(sorted(part_names)),
    )
    rotors = [p for p in object_model.parts if p.name.startswith("rotor_")]
    ctx.check(
        "rotor part count matches config",
        len(rotors) == r.rotor_count,
        details=f"{len(rotors)} vs {r.rotor_count}",
    )
    spin_joints = [j for j in object_model.articulations if j.name.startswith("rotor_spin_")]
    ctx.check(
        "rotor joint count matches config",
        len(spin_joints) == r.rotor_count,
        details=f"{len(spin_joints)} vs {r.rotor_count}",
    )
    for joint in spin_joints:
        ctx.check(
            f"{joint.name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint.name} axis is fan depth",
            joint.axis == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"{joint.name} has metadata",
            {"type", "axis", "origin", "range"}.issubset(joint.meta),
            details=str(joint.meta),
        )
    knob_joint = object_model.get_articulation("speed_knob_turn")
    ctx.check(
        "knob joint is revolute",
        knob_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.check(
        "knob joint has metadata",
        {"type", "axis", "origin", "range"}.issubset(knob_joint.meta),
        details=str(knob_joint.meta),
    )
    ctx.check(
        "knob range valid",
        knob_joint.motion_limits is not None
        and knob_joint.motion_limits.upper is not None
        and knob_joint.motion_limits.upper >= math.radians(45.0),
        details=str(knob_joint.motion_limits),
    )
    housing = object_model.get_part("housing")
    housing_visuals = {v.name for v in housing.visuals if v.name}
    ctx.check(
        "housing has grille visuals",
        any(
            "grille" in name or "guard" in name or "wire" in name or "bar" in name
            for name in housing_visuals
        ),
        details=str(sorted(housing_visuals)[:20]),
    )
    ctx.check(
        "housing has motor structure",
        any(
            "motor_pod" in name or "rear_motor_cap" in name or "motor_spider" in name
            for name in housing_visuals
        ),
        details=str([n for n in housing_visuals if "motor" in n]),
    )

    # Rotor visuals check
    for rotor_part in rotors:
        ctx.check(
            f"{rotor_part.name} has geometry",
            len([v for v in rotor_part.visuals if v.geometry is not None]) >= 1,
            details=f"visual count={len(rotor_part.visuals)}",
        )
    # Grille spokes must terminate on the outer guard ring (front/rear when present).
    for face in ("rear", "front"):
        if face == "front" and r.snap_front_grille:
            continue
        for rotor_i in range(r.rotor_count):
            ring_name = f"{face}_guard_ring_{rotor_i}_0"
            for spoke_i in range(8):
                spoke_name = f"{face}_spoke_{rotor_i}_{spoke_i}"
                if ring_name not in housing_visuals or spoke_name not in housing_visuals:
                    continue
                ctx.expect_contact(
                    housing,
                    housing,
                    elem_a=housing.get_visual(spoke_name),
                    elem_b=housing.get_visual(ring_name),
                    contact_tol=0.003,
                    name=f"{spoke_name}_contacts_{ring_name}",
                )
        left_frame = f"{face}_grille_left"
        right_frame = f"{face}_grille_right"
        top_frame = f"{face}_grille_top"
        bottom_frame = f"{face}_grille_bottom"
        if r.grille_style == "separate_chrome_grille":
            horizontal_prefix = f"{face}_chrome_h_"
            for i in range(r.grille_density):
                bar_name = f"{horizontal_prefix}{i}"
                if bar_name not in housing_visuals:
                    continue
                ctx.expect_contact(
                    housing,
                    housing,
                    elem_a=housing.get_visual(bar_name),
                    elem_b=housing.get_visual(left_frame),
                    contact_tol=0.003,
                    name=f"{bar_name}_contacts_left_frame",
                )
                ctx.expect_contact(
                    housing,
                    housing,
                    elem_a=housing.get_visual(bar_name),
                    elem_b=housing.get_visual(right_frame),
                    contact_tol=0.003,
                    name=f"{bar_name}_contacts_right_frame",
                )
        elif r.grille_style == "wire_grid":
            for i in range(r.grille_density):
                hbar_name = f"{face}_wire_h_{i}"
                vbar_name = f"{face}_wire_v_{i}"
                if hbar_name in housing_visuals:
                    ctx.expect_contact(
                        housing,
                        housing,
                        elem_a=housing.get_visual(hbar_name),
                        elem_b=housing.get_visual(left_frame),
                        contact_tol=0.003,
                        name=f"{hbar_name}_contacts_left_frame",
                    )
                    ctx.expect_contact(
                        housing,
                        housing,
                        elem_a=housing.get_visual(hbar_name),
                        elem_b=housing.get_visual(right_frame),
                        contact_tol=0.003,
                        name=f"{hbar_name}_contacts_right_frame",
                    )
                if vbar_name in housing_visuals:
                    ctx.expect_contact(
                        housing,
                        housing,
                        elem_a=housing.get_visual(vbar_name),
                        elem_b=housing.get_visual(top_frame),
                        contact_tol=0.003,
                        name=f"{vbar_name}_contacts_top_frame",
                    )
                    ctx.expect_contact(
                        housing,
                        housing,
                        elem_a=housing.get_visual(vbar_name),
                        elem_b=housing.get_visual(bottom_frame),
                        contact_tol=0.003,
                        name=f"{vbar_name}_contacts_bottom_frame",
                    )
        elif r.grille_style == "rectangular_bar_guard":
            for i in range(r.grille_density):
                bar_name = f"{face}_bar_{i}"
                if bar_name not in housing_visuals:
                    continue
                ctx.expect_contact(
                    housing,
                    housing,
                    elem_a=housing.get_visual(bar_name),
                    elem_b=housing.get_visual(top_frame),
                    contact_tol=0.003,
                    name=f"{bar_name}_contacts_top_frame",
                )
                ctx.expect_contact(
                    housing,
                    housing,
                    elem_a=housing.get_visual(bar_name),
                    elem_b=housing.get_visual(bottom_frame),
                    contact_tol=0.003,
                    name=f"{bar_name}_contacts_bottom_frame",
                )
    # Rotor-to-motor contact check: if a shaft was generated, it must touch the
    # dedicated rotor hub contact collar (not any arbitrary rotor mesh face).
    for i, rotor_part in enumerate(rotors):
        shaft_name = f"motor_shaft_{i}"
        if shaft_name not in housing_visuals:
            continue
        rotor_visual_names = {v.name for v in rotor_part.visuals if v.name}
        rotor_anchor_name = "hub_contact_collar"
        if rotor_anchor_name not in rotor_visual_names:
            continue
        ctx.expect_contact(
            housing,
            rotor_part,
            elem_a=housing.get_visual(shaft_name),
            elem_b=rotor_part.get_visual(rotor_anchor_name),
            contact_tol=0.002,
            name=f"motor_shaft_{i}_contacts_rotor_hub",
        )

    if r.panel_layout != "none":
        panel_joints = [
            j
            for j in object_model.articulations
            if ("hinge" in j.name or "slide" in j.name) and j.name != "housing_tilt"
        ]
        ctx.check(
            "panel joints present",
            len(panel_joints) >= 1,
            details=f"panel_joints={len(panel_joints)}",
        )
    if r.panel_layout == "side_expansion_pair":
        slide_joints = [j for j in object_model.articulations if j.name.endswith("_slide")]
        ctx.check(
            "window layout has prismatic slide joints",
            len(slide_joints) >= 2,
            details=f"slide_joints={len(slide_joints)}",
        )
        ctx.check(
            "window slide joints are prismatic",
            all(j.articulation_type == ArticulationType.PRISMATIC for j in slide_joints),
            details=str([j.articulation_type for j in slide_joints]),
        )
    if r.panel_layout == "four_shutters":
        shutter_joints = [
            j for j in object_model.articulations if j.name.startswith("panel_hinge_")
        ]
        ctx.check(
            "four_shutters layout has 4 revolute hinge joints",
            len(shutter_joints) == 4,
            details=f"shutter_joints={len(shutter_joints)}",
        )
        ctx.check(
            "shutter joints are revolute",
            all(j.articulation_type == ArticulationType.REVOLUTE for j in shutter_joints),
        )
    if r.support_style == "u_tilt_stand":
        ctx.check(
            "tilt joint present for U stand",
            object_model.get_articulation("housing_tilt").articulation_type
            == ArticulationType.REVOLUTE,
        )
    if r.support_style == "pedestal_column":
        ctx.check(
            "column extension is prismatic",
            object_model.get_articulation("column_extension").articulation_type
            == ArticulationType.PRISMATIC,
        )
    if r.layout == "portable_handle":
        ctx.check(
            "portable handle folds",
            object_model.get_articulation("carry_handle_fold").articulation_type
            == ArticulationType.REVOLUTE,
        )
    if r.snap_front_grille:
        ctx.check(
            "snap grille part present",
            "front_snap_grille" in part_names,
            details=str(sorted(part_names)),
        )
        ctx.check(
            "snap grille hinge is revolute",
            object_model.get_articulation("front_grille_hinge").articulation_type
            == ArticulationType.REVOLUTE,
        )
    return ctx.report()
