from __future__ import annotations

import math
import random
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

MountLayout = Literal[
    "wheeled_field_carriage",
    "deck_swivel_yoke",
    "naval_slide",
    "trestle_mortar",
    "barbette_platform",
]
BarrelProfile = Literal["long_smoothbore", "short_carronade", "squat_mortar", "tapered_swivel"]
WheelStyle = Literal["none", "solid_iron", "spoked_wood", "iron_shod_wood"]
TraverseStyle = Literal["none", "post_swivel", "deck_ring", "barbette_bearing"]
WedgeProfile = Literal["fixed_block", "sliding_wedge"]
MaterialStyle = Literal["bronze_wood", "black_iron_wood", "weathered_iron", "coastal_concrete"]

MATERIAL_PALETTES: dict[
    MaterialStyle,
    tuple[
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
    ],
] = {
    "bronze_wood": ((0.48, 0.28, 0.12, 1.0), (0.56, 0.36, 0.16, 1.0), (0.18, 0.16, 0.13, 1.0)),
    "black_iron_wood": (
        (0.32, 0.22, 0.12, 1.0),
        (0.05, 0.052, 0.05, 1.0),
        (0.025, 0.026, 0.026, 1.0),
    ),
    "weathered_iron": (
        (0.45, 0.42, 0.36, 1.0),
        (0.05, 0.052, 0.05, 1.0),
        (0.025, 0.026, 0.026, 1.0),
    ),
    "coastal_concrete": ((0.48, 0.48, 0.44, 1.0), (0.15, 0.16, 0.17, 1.0), (0.60, 0.58, 0.52, 1.0)),
}


@dataclass(frozen=True)
class CannonConfig:
    mount_layout: MountLayout = "wheeled_field_carriage"
    barrel_profile: BarrelProfile = "long_smoothbore"
    barrel_length: float | None = None
    wheel_style: WheelStyle = "spoked_wood"
    wheel_count: int = 2
    traverse_style: TraverseStyle = "none"
    recoil_enabled: bool | None = None
    wedge_enabled: bool | None = None
    wedge_profile: WedgeProfile = "sliding_wedge"
    elevation_range_deg: tuple[float, float] = (-6.0, 22.0)
    recoil_travel: float = 0.36
    wedge_travel: float = 0.245
    material_style: MaterialStyle = "black_iron_wood"
    name: str = "reference_cannon"


@dataclass(frozen=True)
class ResolvedCannonConfig:
    mount_layout: MountLayout
    barrel_profile: BarrelProfile
    barrel_length: float
    barrel_radius: float
    barrel_origin_z: float
    trunnion_half_track: float
    wheel_style: WheelStyle
    wheel_count: int
    wheel_radius: float
    wheel_width: float
    wheel_track: float
    traverse_style: TraverseStyle
    recoil_enabled: bool
    wedge_enabled: bool
    wedge_profile: WedgeProfile
    elevation_range_rad: tuple[float, float]
    recoil_travel: float
    wedge_travel: float
    support_width: float
    support_length: float
    support_height: float
    material_style: MaterialStyle
    name: str


def config_from_seed(seed: int) -> CannonConfig:
    rng = random.Random(seed)
    mount_layout: MountLayout = rng.choices(
        (
            "wheeled_field_carriage",
            "deck_swivel_yoke",
            "naval_slide",
            "trestle_mortar",
            "barbette_platform",
        ),
        weights=(0.34, 0.18, 0.20, 0.16, 0.12),
        k=1,
    )[0]
    profile_by_layout: dict[MountLayout, tuple[BarrelProfile, ...]] = {
        "wheeled_field_carriage": ("long_smoothbore", "tapered_swivel"),
        "deck_swivel_yoke": ("tapered_swivel", "short_carronade"),
        "naval_slide": ("short_carronade", "long_smoothbore"),
        "trestle_mortar": ("squat_mortar",),
        "barbette_platform": ("long_smoothbore", "short_carronade"),
    }
    barrel_profile = rng.choice(profile_by_layout[mount_layout])
    return CannonConfig(
        mount_layout=mount_layout,
        barrel_profile=barrel_profile,
        barrel_length=round(rng.uniform(0.70, 2.10), 3),
        wheel_style=rng.choice(("solid_iron", "spoked_wood", "iron_shod_wood")),
        wheel_count=rng.choice((2, 4)),
        traverse_style=rng.choice(("post_swivel", "deck_ring", "barbette_bearing", "none")),
        recoil_enabled=None,
        wedge_enabled=None,
        material_style=rng.choice(
            ("bronze_wood", "black_iron_wood", "weathered_iron", "coastal_concrete")
        ),
        name=f"seeded_cannon_{seed}",
    )


def resolve_config(config: CannonConfig) -> ResolvedCannonConfig:
    if config.mount_layout not in {
        "wheeled_field_carriage",
        "deck_swivel_yoke",
        "naval_slide",
        "trestle_mortar",
        "barbette_platform",
    }:
        raise ValueError(f"Unsupported mount_layout: {config.mount_layout}")
    if config.barrel_profile not in {
        "long_smoothbore",
        "short_carronade",
        "squat_mortar",
        "tapered_swivel",
    }:
        raise ValueError(f"Unsupported barrel_profile: {config.barrel_profile}")
    if config.wheel_style not in {"none", "solid_iron", "spoked_wood", "iron_shod_wood"}:
        raise ValueError(f"Unsupported wheel_style: {config.wheel_style}")
    if config.traverse_style not in {"none", "post_swivel", "deck_ring", "barbette_bearing"}:
        raise ValueError(f"Unsupported traverse_style: {config.traverse_style}")
    if config.wedge_profile not in {"fixed_block", "sliding_wedge"}:
        raise ValueError(f"Unsupported wedge_profile: {config.wedge_profile}")
    if config.material_style not in MATERIAL_PALETTES:
        raise ValueError(f"Unsupported material_style: {config.material_style}")
    if config.wheel_count not in {2, 4}:
        raise ValueError("wheel_count must be 2 or 4")

    profile_lengths = {
        "long_smoothbore": (1.25, 2.20, 0.115),
        "short_carronade": (0.75, 1.35, 0.145),
        "squat_mortar": (0.65, 0.95, 0.210),
        "tapered_swivel": (0.85, 1.45, 0.085),
    }
    min_len, max_len, radius = profile_lengths[config.barrel_profile]
    barrel_length = (
        config.barrel_length if config.barrel_length is not None else (min_len + max_len) * 0.5
    )
    barrel_length = max(min_len, min(max_len, barrel_length))

    support_dims = {
        "wheeled_field_carriage": (1.75, 0.88, 0.72, 0.82),
        "deck_swivel_yoke": (0.52, 0.46, 0.64, 0.68),
        "naval_slide": (1.85, 0.82, 0.28, 0.62),
        "trestle_mortar": (1.36, 0.98, 0.70, 0.62),
        "barbette_platform": (1.45, 1.45, 0.52, 0.78),
    }
    support_length, support_width, support_height, barrel_z = support_dims[config.mount_layout]
    wheel_style = config.wheel_style if config.mount_layout == "wheeled_field_carriage" else "none"
    traverse_style = config.traverse_style
    if config.mount_layout == "deck_swivel_yoke" and traverse_style == "none":
        traverse_style = "post_swivel"
    if config.mount_layout == "naval_slide" and traverse_style == "none":
        traverse_style = "deck_ring"
    if config.mount_layout == "barbette_platform" and traverse_style == "none":
        traverse_style = "barbette_bearing"
    if config.mount_layout in {"wheeled_field_carriage", "trestle_mortar"}:
        traverse_style = "none"

    recoil_enabled = config.recoil_enabled
    if recoil_enabled is None:
        recoil_enabled = config.mount_layout == "naval_slide"
    wedge_enabled = config.wedge_enabled
    if wedge_enabled is None:
        wedge_enabled = config.mount_layout == "trestle_mortar"

    lower_deg, upper_deg = config.elevation_range_deg
    if lower_deg >= upper_deg:
        raise ValueError("elevation_range_deg lower must be below upper")
    lower_deg = max(-10.0, min(5.0, lower_deg))
    upper_deg = max(12.0, min(40.0, upper_deg))
    trunnion_half_track = support_width * 0.38
    if config.mount_layout == "wheeled_field_carriage":
        trunnion_half_track = support_width * 0.5 - 0.13
    elif config.mount_layout == "deck_swivel_yoke":
        trunnion_half_track = 0.11
    elif config.mount_layout == "naval_slide":
        trunnion_half_track = support_width * 0.19
    elif config.mount_layout == "trestle_mortar":
        trunnion_half_track = 0.35

    wheel_radius = 0.33 if config.wheel_count == 4 else 0.43
    wheel_width = 0.12

    return ResolvedCannonConfig(
        mount_layout=config.mount_layout,
        barrel_profile=config.barrel_profile,
        barrel_length=barrel_length,
        barrel_radius=radius,
        barrel_origin_z=barrel_z,
        trunnion_half_track=trunnion_half_track,
        wheel_style=wheel_style,
        wheel_count=config.wheel_count,
        wheel_radius=wheel_radius,
        wheel_width=wheel_width,
        wheel_track=support_width + 0.36,
        traverse_style=traverse_style,
        recoil_enabled=bool(recoil_enabled),
        wedge_enabled=bool(wedge_enabled),
        wedge_profile=config.wedge_profile,
        elevation_range_rad=(math.radians(lower_deg), math.radians(upper_deg)),
        recoil_travel=max(0.08, min(0.36, config.recoil_travel)),
        wedge_travel=max(0.05, min(0.245, config.wedge_travel)),
        support_width=support_width,
        support_length=support_length,
        support_height=support_height,
        material_style=config.material_style,
        name=config.name,
    )


def _cylinder_along_x(radius: float, length: float, center_x: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(center_x - length / 2.0, 0.0, 0.0))
        .circle(radius)
        .extrude(length)
    )


def _frustum_along_x(
    radius_a: float, radius_b: float, length: float, start_x: float
) -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(start_x, 0.0, 0.0))
        .circle(radius_a)
        .workplane(offset=length)
        .circle(radius_b)
        .loft(combine=True)
    )


def _build_barrel_mesh(r: ResolvedCannonConfig) -> cq.Workplane:
    radius = r.barrel_radius
    length = r.barrel_length
    if r.barrel_profile == "squat_mortar":
        body = _frustum_along_x(radius * 1.10, radius * 0.90, length * 0.70, -length * 0.30)
        body = body.union(_cylinder_along_x(radius * 1.20, length * 0.32, -length * 0.38))
        body = body.union(_cylinder_along_x(radius * 1.08, length * 0.18, length * 0.38))
        body = body.union(_frustum_along_x(radius * 1.08, radius * 1.24, 0.14, length * 0.44))
    elif r.barrel_profile == "tapered_swivel":
        body = _frustum_along_x(radius * 1.12, radius * 0.72, length, -length * 0.52)
        body = body.union(_cylinder_along_x(radius * 1.22, length * 0.28, -length * 0.58))
        body = body.union(_frustum_along_x(radius * 0.72, radius * 0.90, 0.14, length * 0.46))
    elif r.barrel_profile == "short_carronade":
        body = _frustum_along_x(radius * 1.18, radius * 0.88, length * 0.80, -length * 0.30)
        body = body.union(_cylinder_along_x(radius * 1.32, length * 0.30, -length * 0.48))
        body = body.union(_cylinder_along_x(radius * 0.96, length * 0.20, length * 0.42))
        body = body.union(_frustum_along_x(radius * 0.96, radius * 1.18, 0.14, length * 0.50))
    else:
        body = _frustum_along_x(radius * 1.35, radius * 0.92, length * 0.72, -length * 0.44)
        body = body.union(_cylinder_along_x(radius * 1.44, length * 0.32, -length * 0.60))
        body = body.union(_cylinder_along_x(radius * 0.92, length * 0.14, length * 0.30))
        body = body.union(_cylinder_along_x(radius * 1.08, length * 0.22, length * 0.42))
        body = body.union(_frustum_along_x(radius * 1.08, radius * 1.22, 0.15, length * 0.52))
    bore = _cylinder_along_x(radius * 0.46, length * 1.12, length * 0.44)
    return body.cut(bore)


def _visual_box(part, size, xyz, material, name: str, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _visual_cylinder(
    part, radius: float, length: float, xyz, material, name: str, rpy=(0.0, 0.0, 0.0)
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _build_wheeled_carriage(part, r: ResolvedCannonConfig, wood, iron, accent) -> None:
    rail_pitch = -math.atan2(0.46, 1.65)
    cheek_y = r.trunnion_half_track + 0.085
    for idx, side, rname, cname, saddle_name in (
        (0, -1.0, "side_rail_0", "cheek_block_0", "trunnion_saddle_0"),
        (1, 1.0, "side_rail_1", "cheek_block_1", "trunnion_saddle_1"),
    ):
        y = side * cheek_y
        _visual_box(
            part, (1.72, 0.14, 0.17), (-0.175, y, 0.45), wood, rname, rpy=(0.0, rail_pitch, 0.0)
        )
        _visual_box(
            part,
            (0.38, 0.15, 0.16),
            (0.02, side * (r.trunnion_half_track + 0.075), r.barrel_origin_z - 0.235),
            wood,
            cname,
            rpy=(0.0, rail_pitch, 0.0),
        )
        _visual_box(
            part,
            (0.24, 0.17, 0.16),
            (0.0, y, r.barrel_origin_z - 0.075),
            wood,
            saddle_name,
        )
    _visual_box(
        part,
        (0.22, 0.86, 0.13),
        (0.48, 0.0, 0.50),
        wood,
        "front_transom",
        rpy=(0.0, rail_pitch, 0.0),
    )
    _visual_box(
        part,
        (0.20, 0.82, 0.13),
        (-0.72, 0.0, 0.30),
        wood,
        "rear_transom",
        rpy=(0.0, rail_pitch, 0.0),
    )
    _visual_box(part, (0.36, 0.74, 0.14), (-1.02, 0.0, 0.07), accent, "rear_skid")
    _visual_box(
        part,
        (0.52, 0.18, 0.12),
        (-0.70, 0.0, 0.23),
        wood,
        "trail_tongue",
        rpy=(0.0, rail_pitch, 0.0),
    )
    axle_xs = (-0.88, 0.18) if r.wheel_count == 4 else (-0.58,)
    for index, axle_x in enumerate(axle_xs):
        _visual_cylinder(
            part,
            0.058,
            r.wheel_track - 0.10,
            (axle_x, 0.0, r.wheel_radius),
            wood,
            f"wheel_axle_{index}",
            rpy=(math.pi / 2, 0.0, 0.0),
        )
        for side_sign in (-1.0, 1.0):
            cap_y = side_sign * (r.wheel_track * 0.5 + r.wheel_width * 1.05 - 0.025)
            _visual_cylinder(
                part,
                0.080,
                0.05,
                (axle_x, cap_y, r.wheel_radius),
                iron,
                f"axle_cap_{index}_{0 if side_sign < 0 else 1}",
                rpy=(math.pi / 2, 0.0, 0.0),
            )
    for side, y in (("left", cheek_y + 0.058), ("right", -cheek_y - 0.058)):
        _visual_box(
            part,
            (0.58, 0.026, 0.034),
            (-0.08, y, r.barrel_origin_z - 0.075),
            iron,
            f"{side}_trunnion_cap_strap",
        )
    # Keep the lunette hardware physically attached to the trail assembly.
    _visual_cylinder(
        part,
        0.042,
        0.32,
        (-1.36, 0.0, 0.07),
        iron,
        "trail_lunette_handle",
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    _visual_cylinder(
        part,
        0.078,
        0.04,
        (-1.54, 0.0, 0.07),
        iron,
        "trail_lunette_ring",
        rpy=(0.0, math.pi / 2.0, 0.0),
    )


def _build_support(part, r: ResolvedCannonConfig, wood, iron, accent) -> None:
    if r.mount_layout == "wheeled_field_carriage":
        _build_wheeled_carriage(part, r, wood, iron, accent)
    elif r.mount_layout == "deck_swivel_yoke":
        _visual_cylinder(part, 0.20, 0.05, (0.0, 0.0, 0.025), iron, "base_flange")
        _visual_cylinder(
            part, 0.07, r.support_height, (0.0, 0.0, r.support_height * 0.5), iron, "post_column"
        )
        _visual_cylinder(
            part, 0.10, 0.05, (0.0, 0.0, r.support_height + 0.025), accent, "post_head"
        )
    elif r.mount_layout == "naval_slide":
        _visual_box(
            part,
            (r.support_length + 0.40, r.support_width + 0.30, 0.08),
            (0.45, 0.0, 0.04),
            wood,
            "deck_section",
        )
        for i, y in enumerate((-0.44, -0.22, 0.0, 0.22, 0.44)):
            _visual_box(
                part,
                (r.support_length + 0.46, 0.010, 0.006),
                (0.45, y, 0.083),
                accent,
                f"deck_plank_seam_{i}",
            )
        _visual_cylinder(part, 0.42, 0.035, (0.0, 0.0, 0.0625), iron, "deck_traverse_ring")
    elif r.mount_layout == "trestle_mortar":
        for side, y in (("port", -0.38), ("starboard", 0.38)):
            _visual_box(
                part, (r.support_length, 0.12, 0.12), (0.0, y, 0.10), wood, f"{side}_bed_rail"
            )
            _visual_box(part, (0.42, 0.06, 0.34), (0.0, y, 0.42), wood, f"{side}_trunnion_cheek")
            _visual_box(
                part, (0.50, 0.025, 0.040), (0.0, y, 0.61), iron, f"{side}_iron_cheek_strap"
            )
        for x in (-0.48, 0.0, 0.48):
            _visual_box(
                part,
                (0.12, 0.86, 0.08),
                (x, 0.0, 0.08),
                wood,
                f"cross_tie_{x:.2f}".replace("-", "m").replace(".", "_"),
            )
        _visual_box(part, (0.80, 0.05, 0.035), (0.0, 0.0, 0.18), iron, "dovetail_slot")
    else:
        _visual_cylinder(part, 0.78, 0.12, (0.0, 0.0, 0.06), accent, "barbette_ring")
        _visual_cylinder(part, 0.45, 0.25, (0.0, 0.0, 0.20), accent, "concrete_pedestal")
        _visual_cylinder(part, 0.22, 0.08, (0.0, 0.0, 0.38), iron, "bearing_race")


def _build_barrel(part, r: ResolvedCannonConfig, iron, accent, bore_shadow, assets) -> None:
    part.visual(
        mesh_from_cadquery(
            _build_barrel_mesh(r),
            "barrel_shell",
            assets=assets,
            tolerance=0.0015,
            angular_tolerance=0.08,
        ),
        material=iron,
        name="barrel_shell",
    )
    part.visual(
        Cylinder(radius=r.barrel_radius * 0.32, length=_trunnion_pin_half_span(r) * 2.0),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="trunnion_pin",
    )
    part.visual(
        Cylinder(radius=r.barrel_radius * 0.38, length=0.14),
        origin=Origin(xyz=(-r.barrel_length * 0.60, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="cascabel_neck",
    )
    part.visual(
        Sphere(radius=r.barrel_radius * 0.52),
        origin=Origin(xyz=(-r.barrel_length * 0.68, 0.0, 0.0)),
        material=iron,
        name="cascabel_knob",
    )
    part.visual(
        Cylinder(radius=r.barrel_radius * 0.46, length=0.018),
        origin=Origin(xyz=(r.barrel_length * 0.62, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bore_shadow,
        name="muzzle_bore_shadow",
    )
    _visual_box(
        part,
        (0.075, 0.050, 0.020),
        (-r.barrel_length * 0.18, 0.0, r.barrel_radius * 0.92),
        accent,
        "vent_touch_hole_plate",
    )
    _visual_box(
        part,
        (0.11, 0.028, 0.030),
        (r.barrel_length * 0.10, 0.0, r.barrel_radius * 1.03),
        accent,
        "lifting_dolphin",
    )


def _build_wheel(part, r: ResolvedCannonConfig, wood, iron) -> None:
    if r.wheel_style in {"spoked_wood", "iron_shod_wood"}:
        _visual_cylinder(
            part,
            r.wheel_radius * 0.90,
            r.wheel_width * 0.82,
            (0.0, 0.0, 0.0),
            wood,
            "spoked_wheel",
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )
        _visual_cylinder(
            part,
            r.wheel_radius,
            r.wheel_width,
            (0.0, 0.0, 0.0),
            iron,
            "iron_hoop",
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )
        for i in range(8):
            angle = math.tau * i / 8.0
            part.visual(
                Box((r.wheel_radius * 1.45, 0.018, 0.025)),
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, angle, 0.0)),
                material=wood,
                name=f"spoke_{i}",
            )
        _visual_cylinder(
            part,
            r.wheel_radius * 0.18,
            r.wheel_width * 1.05,
            (0.0, 0.0, 0.0),
            iron,
            "wheel_hub",
            rpy=(math.pi / 2, 0.0, 0.0),
        )
        _visual_cylinder(
            part,
            r.wheel_radius * 0.045,
            0.012,
            (r.wheel_radius * 0.35, r.wheel_width * 0.50 + 0.010, 0.0),
            iron,
            "hub_cap",
            rpy=(math.pi / 2, 0.0, 0.0),
        )
        for i in range(8):
            angle = math.tau * i / 8.0
            _visual_cylinder(
                part,
                r.wheel_radius * 0.020,
                0.018,
                (
                    math.cos(angle) * r.wheel_radius * 0.78,
                    0.0,
                    math.sin(angle) * r.wheel_radius * 0.78,
                ),
                iron,
                f"rim_rivet_{i}",
                rpy=(math.pi / 2, 0.0, 0.0),
            )
    else:
        _visual_cylinder(
            part,
            r.wheel_radius,
            r.wheel_width,
            (0.0, 0.0, 0.0),
            iron,
            "wheel_disc",
            rpy=(math.pi / 2, 0.0, 0.0),
        )
        _visual_cylinder(
            part,
            r.wheel_radius * 0.22,
            r.wheel_width * 1.10,
            (0.0, 0.0, 0.0),
            iron,
            "wheel_hub",
            rpy=(math.pi / 2, 0.0, 0.0),
        )
        _visual_cylinder(
            part,
            r.wheel_radius * 0.045,
            0.012,
            (r.wheel_radius * 0.35, r.wheel_width * 0.50 + 0.010, 0.0),
            iron,
            "hub_cap",
            rpy=(math.pi / 2, 0.0, 0.0),
        )
        for i in range(8):
            angle = math.tau * i / 8.0
            _visual_cylinder(
                part,
                r.wheel_radius * 0.020,
                0.018,
                (
                    math.cos(angle) * r.wheel_radius * 0.78,
                    0.0,
                    math.sin(angle) * r.wheel_radius * 0.78,
                ),
                iron,
                f"rim_rivet_{i}",
                rpy=(math.pi / 2, 0.0, 0.0),
            )


def _traverse_origin_z(r: ResolvedCannonConfig) -> float:
    if r.mount_layout == "deck_swivel_yoke":
        return r.support_height + 0.03
    if r.mount_layout == "naval_slide":
        return 0.08
    if r.mount_layout == "barbette_platform":
        return 0.42
    return max(0.12, r.support_height * 0.55)


def _trunnion_pin_half_span(r: ResolvedCannonConfig) -> float:
    """Half-span so trunnion pin ends meet wooden bearing faces by layout."""
    if r.recoil_enabled:
        # Recoil cheeks: center at support_width*0.28*1.15, thickness_y=0.05.
        # Inner face sits at center - 0.025.
        return r.support_width * 0.28 * 1.15 - 0.025
    return r.trunnion_half_track


def build_cannon(
    config: CannonConfig | None = None, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    config = config or CannonConfig()
    r = resolve_config(config)
    assets = assets or AssetContext(Path(tempfile.mkdtemp(prefix="articraft-cannon-assets-")))
    model = ArticulatedObject(name=r.name, assets=assets)
    wood_rgba, iron_rgba, accent_rgba = MATERIAL_PALETTES[r.material_style]
    wood = model.material(f"wood_{r.material_style}", rgba=wood_rgba)
    iron = model.material(f"iron_{r.material_style}", rgba=iron_rgba)
    accent = model.material(f"accent_{r.material_style}", rgba=accent_rgba)
    bore_shadow = model.material("sooty_bore_shadow", rgba=(0.005, 0.004, 0.003, 1.0))

    support = model.part("support")
    _build_support(support, r, wood, iron, accent)
    barrel_parent = support
    barrel_origin = Origin(xyz=(0.0, 0.0, r.barrel_origin_z))

    if r.traverse_style != "none":
        yoke = model.part("yoke_or_rotating_carriage")
        if r.mount_layout == "naval_slide":
            _visual_cylinder(yoke, 0.36, 0.045, (0.0, 0.0, 0.0225), iron, "deck_ring_turntable")
            for y, side in ((-0.32, "starboard"), (0.32, "port")):
                _visual_box(
                    yoke,
                    (r.support_length, 0.10, 0.08),
                    (0.45, y, 0.04),
                    accent,
                    f"{side}_slide_rail",
                )
                _visual_box(
                    yoke,
                    (r.support_length - 0.14, 0.055, 0.035),
                    (0.48, y, 0.0975),
                    accent,
                    f"{side}_rail_cap",
                )
            _visual_box(
                yoke, (0.18, r.support_width, 0.08), (-0.40, 0.0, 0.04), wood, "aft_transom"
            )
            _visual_box(
                yoke, (0.16, r.support_width, 0.08), (1.28, 0.0, 0.04), wood, "front_transom"
            )
        else:
            _visual_cylinder(yoke, 0.18, 0.08, (0.0, 0.0, 0.04), iron, "swivel_collar")
            for y, side in (
                (r.trunnion_half_track + 0.025, "left"),
                (-r.trunnion_half_track - 0.025, "right"),
            ):
                # Arm lower face sits on swivel collar top face (z=0.08 in yoke-local space).
                _visual_box(yoke, (0.18, 0.05, 0.34), (0.0, y, 0.25), accent, f"{side}_yoke_arm")
        model.articulation(
            "traverse_joint",
            ArticulationType.CONTINUOUS
            if r.traverse_style == "post_swivel"
            else ArticulationType.REVOLUTE,
            parent=support,
            child=yoke,
            origin=Origin(xyz=(0.0, 0.0, _traverse_origin_z(r))),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=700.0, velocity=0.8, lower=-math.radians(55), upper=math.radians(55)
            ),
            meta={
                "type": "continuous" if r.traverse_style == "post_swivel" else "revolute",
                "axis": (0.0, 0.0, 1.0),
                "origin": "pedestal/deck ring center",
                "range": "continuous or +/-55deg",
            },
        )
        barrel_parent = yoke
        barrel_origin = Origin(xyz=(0.0, 0.0, 0.24))

    if r.recoil_enabled:
        recoil = model.part("recoil_carriage")
        for y in (-r.support_width * 0.28, r.support_width * 0.28):
            _visual_box(
                recoil,
                (0.80, 0.08, 0.08),
                (0.0, y, 0.06),
                wood,
                f"runner_{abs(y):.2f}".replace(".", "_"),
            )
            _visual_box(
                recoil,
                (0.70, 0.05, 0.32),
                (0.0, y * 1.15, 0.26),
                wood,
                f"trunnion_cheek_{abs(y):.2f}".replace(".", "_"),
            )
        recoil_origin_z = 0.095 if r.mount_layout == "naval_slide" else 0.12
        model.articulation(
            "recoil_slide",
            ArticulationType.PRISMATIC,
            parent=barrel_parent,
            child=recoil,
            origin=Origin(xyz=(0.35, 0.0, recoil_origin_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1800.0, velocity=1.2, lower=0.0, upper=r.recoil_travel
            ),
            meta={
                "type": "prismatic",
                "axis": (-1.0, 0.0, 0.0),
                "origin": "slide rail start",
                "range": (0.0, r.recoil_travel),
            },
        )
        barrel_parent = recoil
        barrel_origin = Origin(xyz=(0.0, 0.0, 0.32))

    barrel = model.part("barrel")
    _build_barrel(barrel, r, iron, accent, bore_shadow, assets)
    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=barrel_parent,
        child=barrel,
        origin=barrel_origin,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.8,
            lower=r.elevation_range_rad[0],
            upper=r.elevation_range_rad[1],
        ),
        meta={
            "type": "revolute",
            "axis": (0.0, -1.0, 0.0),
            "origin": "trunnion center",
            "range": r.elevation_range_rad,
        },
    )

    if r.wheel_style != "none":
        wheel_center_y = r.wheel_track * 0.5 - 0.05 + r.wheel_width * 0.525
        if r.wheel_count == 4:
            wheel_positions = (
                ("rear_left", -0.88, wheel_center_y),
                ("rear_right", -0.88, -wheel_center_y),
                ("front_left", 0.18, wheel_center_y),
                ("front_right", 0.18, -wheel_center_y),
            )
        else:
            wheel_positions = (
                ("left", -0.58, wheel_center_y),
                ("right", -0.58, -wheel_center_y),
            )
        for side, x, y in wheel_positions:
            wheel = model.part(f"{side}_wheel")
            _build_wheel(wheel, r, wood, iron)
            model.articulation(
                f"{side}_wheel_spin",
                ArticulationType.CONTINUOUS,
                parent=support,
                child=wheel,
                origin=Origin(xyz=(x, y, r.wheel_radius)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=500.0, velocity=8.0),
                meta={
                    "type": "continuous",
                    "axis": (0.0, 1.0, 0.0),
                    "origin": "wheel center",
                    "range": "continuous",
                },
            )

    if r.wedge_enabled:
        wedge = model.part("elevation_wedge")
        _visual_box(wedge, (0.42, 0.24, 0.12), (-0.16, 0.0, 0.08), wood, "wedge_body")
        _visual_box(wedge, (0.04, 0.22, 0.08), (0.08, 0.0, 0.14), iron, "strike_plate")
        model.articulation(
            "wedge_slide",
            ArticulationType.PRISMATIC,
            parent=support,
            child=wedge,
            origin=Origin(xyz=(-0.20, 0.0, 0.1765)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=260.0, velocity=0.2, lower=0.0, upper=r.wedge_travel),
            meta={
                "type": "prismatic",
                "axis": (1.0, 0.0, 0.0),
                "origin": "trestle bed slot",
                "range": (0.0, r.wedge_travel),
            },
        )

    return model


def run_cannon_tests(object_model: ArticulatedObject, config: CannonConfig) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}
    ctx.check(
        "required_parts",
        {"support", "barrel"}.issubset(part_names),
        details=str(sorted(part_names)),
    )

    barrel_joint = object_model.get_articulation("barrel_elevation")
    ctx.check(
        "barrel_joint_is_revolute",
        barrel_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(barrel_joint.articulation_type),
    )
    ctx.check(
        "barrel_joint_axis_is_trunnion_axis",
        tuple(round(v, 6) for v in barrel_joint.axis) == (0.0, -1.0, 0.0),
        details=str(barrel_joint.axis),
    )
    ctx.check(
        "barrel_joint_has_metadata",
        {"type", "axis", "origin", "range"}.issubset(barrel_joint.meta),
        details=str(barrel_joint.meta),
    )
    limits = barrel_joint.motion_limits
    ctx.check(
        "barrel_range_matches_resolved",
        limits is not None
        and limits.lower == r.elevation_range_rad[0]
        and limits.upper == r.elevation_range_rad[1],
        details=str(limits),
    )

    barrel_visuals = {v.name for v in object_model.get_part("barrel").visuals}
    ctx.check(
        "barrel_identity_visuals",
        {"barrel_shell", "cascabel_knob", "trunnion_pin"}.issubset(barrel_visuals),
        details=str(sorted(barrel_visuals)),
    )

    if r.wheel_style != "none":
        expected_wheels = (
            {"left_wheel", "right_wheel"}
            if r.wheel_count == 2
            else {"rear_left_wheel", "rear_right_wheel", "front_left_wheel", "front_right_wheel"}
        )
        expected_wheel_joints = {f"{name}_spin" for name in expected_wheels}
        ctx.check(
            "wheel_parts_exist",
            expected_wheels.issubset(part_names),
            details=str(sorted(part_names)),
        )
        ctx.check(
            "wheel_spin_joints_exist",
            expected_wheel_joints.issubset(joint_names),
            details=str(sorted(joint_names)),
        )
        if r.wheel_style in {"spoked_wood", "iron_shod_wood"}:
            wheel_name = "left_wheel" if r.wheel_count == 2 else "rear_left_wheel"
            wheel_visuals = {v.name for v in object_model.get_part(wheel_name).visuals}
            ctx.check(
                "spoked_wheel_mesh_present",
                "spoked_wheel" in wheel_visuals and "iron_hoop" in wheel_visuals,
                details=str(sorted(wheel_visuals)),
            )

    if r.traverse_style != "none":
        traverse = object_model.get_articulation("traverse_joint")
        ctx.check(
            "traverse_axis_vertical",
            tuple(round(v, 6) for v in traverse.axis) == (0.0, 0.0, 1.0),
            details=str(traverse.axis),
        )
        if r.mount_layout in {"deck_swivel_yoke", "barbette_platform"}:
            yoke = object_model.get_part("yoke_or_rotating_carriage")
            barrel = object_model.get_part("barrel")
            left_arm = yoke.get_visual("left_yoke_arm")
            right_arm = yoke.get_visual("right_yoke_arm")
            trunnion_pin = barrel.get_visual("trunnion_pin")
            geometry_ok = (
                isinstance(left_arm.geometry, Box)
                and isinstance(right_arm.geometry, Box)
                and isinstance(trunnion_pin.geometry, Cylinder)
            )
            ctx.check("yoke_arm_and_trunnion_geometry_types", geometry_ok)
            if geometry_ok:
                pin_half_span_y = trunnion_pin.geometry.length * 0.5
                left_inner_face_y = left_arm.origin.xyz[1] - left_arm.geometry.size[1] * 0.5
                right_inner_face_y = right_arm.origin.xyz[1] + right_arm.geometry.size[1] * 0.5
                ctx.check(
                    "left_yoke_arm_contacts_trunnion_pin",
                    abs(left_inner_face_y - pin_half_span_y) <= 1e-4,
                    details=f"left_inner_face_y={left_inner_face_y:.6f} pin_half_span_y={pin_half_span_y:.6f}",
                )
                ctx.check(
                    "right_yoke_arm_contacts_trunnion_pin",
                    abs(right_inner_face_y + pin_half_span_y) <= 1e-4,
                    details=f"right_inner_face_y={right_inner_face_y:.6f} pin_half_span_y={pin_half_span_y:.6f}",
                )
    if r.recoil_enabled:
        recoil = object_model.get_articulation("recoil_slide")
        ctx.check(
            "recoil_prismatic",
            recoil.articulation_type == ArticulationType.PRISMATIC,
            details=str(recoil.articulation_type),
        )
        ctx.check(
            "recoil_axis_parallel_rails",
            tuple(round(v, 6) for v in recoil.axis) == (-1.0, 0.0, 0.0),
            details=str(recoil.axis),
        )
        if r.mount_layout == "naval_slide":
            yoke = object_model.get_part("yoke_or_rotating_carriage")
            recoil_part = object_model.get_part("recoil_carriage")
            rail_cap = next(
                (v for v in yoke.visuals if v.name and v.name.endswith("_rail_cap")), None
            )
            runner = next(
                (v for v in recoil_part.visuals if v.name and v.name.startswith("runner_")), None
            )
            barrel = object_model.get_part("barrel")
            trunnion_pin = barrel.get_visual("trunnion_pin")
            cheek = next(
                (v for v in recoil_part.visuals if v.name and v.name.startswith("trunnion_cheek_")),
                None,
            )
            geometry_ok = (
                rail_cap is not None
                and runner is not None
                and isinstance(rail_cap.geometry, Box)
                and isinstance(runner.geometry, Box)
            )
            ctx.check("naval_recoil_runner_geometry_types", geometry_ok)
            if geometry_ok:
                rail_cap_top_z = (
                    recoil.origin.xyz[2] * 0.0
                    + rail_cap.origin.xyz[2]
                    + rail_cap.geometry.size[2] * 0.5
                )
                runner_bottom_z = (
                    recoil.origin.xyz[2] + runner.origin.xyz[2] - runner.geometry.size[2] * 0.5
                )
                ctx.check(
                    "naval_recoil_runner_contacts_rail_cap",
                    abs(runner_bottom_z - rail_cap_top_z) <= 1e-4,
                    details=f"runner_bottom_z={runner_bottom_z:.6f} rail_cap_top_z={rail_cap_top_z:.6f}",
                )
            trunnion_geometry_ok = (
                isinstance(trunnion_pin.geometry, Cylinder) and isinstance(cheek.geometry, Box)
                if cheek is not None
                else False
            )
            ctx.check("naval_trunnion_pin_cheek_geometry_types", trunnion_geometry_ok)
            if trunnion_geometry_ok:
                pin_half_span_y = trunnion_pin.geometry.length * 0.5
                cheek_inner_face_y = abs(cheek.origin.xyz[1]) - cheek.geometry.size[1] * 0.5
                ctx.check(
                    "naval_trunnion_pin_contacts_cheek_inner_faces",
                    abs(pin_half_span_y - cheek_inner_face_y) <= 1e-4,
                    details=f"pin_half_span_y={pin_half_span_y:.6f} cheek_inner_face_y={cheek_inner_face_y:.6f}",
                )
    if r.wedge_enabled:
        wedge = object_model.get_articulation("wedge_slide")
        ctx.check(
            "wedge_part_and_joint",
            "elevation_wedge" in part_names
            and wedge.articulation_type == ArticulationType.PRISMATIC,
            details=str(sorted(part_names)),
        )
    if r.mount_layout == "wheeled_field_carriage":
        support = object_model.get_part("support")
        barrel = object_model.get_part("barrel")
        rear_skid = support.get_visual("rear_skid")
        handle = support.get_visual("trail_lunette_handle")
        ring = support.get_visual("trail_lunette_ring")
        trunnion_pin = barrel.get_visual("trunnion_pin")
        cheek_block = support.get_visual("cheek_block_0")
        geometry_ok = (
            isinstance(rear_skid.geometry, Box)
            and isinstance(handle.geometry, Cylinder)
            and isinstance(ring.geometry, Cylinder)
        )
        ctx.check("trail_lunette_geometry_types", geometry_ok)
        if geometry_ok:
            rear_skid_tail_x = rear_skid.origin.xyz[0] - rear_skid.geometry.size[0] * 0.5
            handle_front_x = handle.origin.xyz[0] + handle.geometry.length * 0.5
            handle_tail_x = handle.origin.xyz[0] - handle.geometry.length * 0.5
            ring_inner_x = ring.origin.xyz[0] + ring.geometry.length * 0.5
            ctx.check(
                "trail_lunette_handle_attaches_to_rear_skid",
                abs(handle_front_x - rear_skid_tail_x) <= 1e-4,
                details=f"handle_front_x={handle_front_x:.6f} rear_skid_tail_x={rear_skid_tail_x:.6f}",
            )
            ctx.check(
                "trail_lunette_ring_attaches_to_handle",
                abs(ring_inner_x - handle_tail_x) <= 1e-4,
                details=f"ring_inner_x={ring_inner_x:.6f} handle_tail_x={handle_tail_x:.6f}",
            )
        trunnion_geometry_ok = isinstance(trunnion_pin.geometry, Cylinder) and isinstance(
            cheek_block.geometry, Box
        )
        ctx.check("wheeled_trunnion_pin_cheek_geometry_types", trunnion_geometry_ok)
        if trunnion_geometry_ok:
            pin_half_span_y = trunnion_pin.geometry.length * 0.5
            cheek_inner_face_y = abs(cheek_block.origin.xyz[1]) - cheek_block.geometry.size[1] * 0.5
            ctx.check(
                "wheeled_trunnion_pin_contacts_cheek_inner_faces",
                abs(pin_half_span_y - cheek_inner_face_y) <= 1e-4,
                details=f"pin_half_span_y={pin_half_span_y:.6f} cheek_inner_face_y={cheek_inner_face_y:.6f}",
            )

    ctx.check(
        "diversity_parameters_present",
        r.mount_layout
        in {
            "wheeled_field_carriage",
            "deck_swivel_yoke",
            "naval_slide",
            "trestle_mortar",
            "barbette_platform",
        }
        and r.barrel_profile
        in {"long_smoothbore", "short_carronade", "squat_mortar", "tapered_swivel"},
        details=f"{r.mount_layout}, {r.barrel_profile}",
    )
    return ctx.report()


def build_seeded_cannon(seed: int, *, assets: AssetContext | None = None) -> ArticulatedObject:
    return build_cannon(config_from_seed(seed), assets=assets)
