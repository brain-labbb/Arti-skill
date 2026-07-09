from __future__ import annotations

import math
from typing import Iterable

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


ALUMINUM = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.74, 1.0))
DARK_METAL = Material("dark_hinge_hardware", rgba=(0.08, 0.085, 0.085, 1.0))
BLACK_PLASTIC = Material("black_plastic_feet", rgba=(0.01, 0.01, 0.01, 1.0))
PALE_PLASTIC = Material("pale_hinge_blocks", rgba=(0.78, 0.80, 0.80, 1.0))
HANGER_MATERIAL = Material("empty_gray_hangers", rgba=(0.38, 0.37, 0.35, 1.0))


def _as_vec(point: Iterable[float]) -> tuple[float, float, float]:
    x, y, z = point
    return (float(x), float(y), float(z))


def _cylinder_between(
    start: Iterable[float],
    end: Iterable[float],
    radius: float,
    *,
    radial_segments: int = 20,
) -> MeshGeometry:
    """Closed circular tube between two local points."""
    p0 = _as_vec(start)
    p1 = _as_vec(end)
    vx = p1[0] - p0[0]
    vy = p1[1] - p0[1]
    vz = p1[2] - p0[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 1.0e-8:
        raise ValueError("tube endpoints must be distinct")

    geom = CylinderGeometry(radius, length, radial_segments=radial_segments, closed=True)
    direction = (vx / length, vy / length, vz / length)
    z_axis = (0.0, 0.0, 1.0)
    dot = max(-1.0, min(1.0, direction[2]))
    angle = math.acos(dot)
    axis = (-direction[1], direction[0], 0.0)
    axis_len = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
    if axis_len > 1.0e-8:
        geom.rotate((axis[0] / axis_len, axis[1] / axis_len, axis[2] / axis_len), angle)
    elif direction[2] < 0.0:
        geom.rotate((1.0, 0.0, 0.0), math.pi)
    # If direction is already +Z, no rotation is necessary.
    geom.translate((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    return geom


def _tube_visual(
    part,
    name: str,
    start: Iterable[float],
    end: Iterable[float],
    radius: float,
    material: Material = ALUMINUM,
    *,
    radial_segments: int = 20,
) -> None:
    part.visual(
        mesh_from_geometry(
            _cylinder_between(start, end, radius, radial_segments=radial_segments),
            name,
        ),
        material=material,
        name=name,
    )


def _merged_tubes(
    segments: Iterable[tuple[tuple[float, float, float], tuple[float, float, float], float]],
    *,
    radial_segments: int = 20,
) -> MeshGeometry:
    merged = MeshGeometry()
    for start, end, radius in segments:
        merged.merge(_cylinder_between(start, end, radius, radial_segments=radial_segments))
    return merged


def _straight_frame_visual(part, name: str, segments, material=ALUMINUM) -> None:
    part.visual(
        mesh_from_geometry(_merged_tubes(segments, radial_segments=18), name),
        material=material,
        name=name,
    )


def _hanger_geometry() -> MeshGeometry:
    """One empty hanger, centered on a bar-axis pivot and free to sway about +X."""
    geom = MeshGeometry()
    hook = tube_from_spline_points(
        [
            (0.000, 0.000, 0.040),
            (0.000, -0.026, 0.035),
            (0.000, -0.040, 0.006),
            (0.000, -0.027, -0.030),
            (0.000, -0.006, -0.055),
        ],
        radius=0.0035,
        samples_per_segment=12,
        radial_segments=12,
        cap_ends=True,
    )
    geom.merge(hook)
    geom.merge(
        _merged_tubes(
            [
                ((0.0, -0.006, -0.055), (0.0, -0.135, -0.285), 0.004),
                ((0.0, -0.006, -0.055), (0.0, 0.135, -0.285), 0.004),
                ((0.0, -0.135, -0.285), (0.0, 0.135, -0.285), 0.004),
                ((0.0, -0.020, -0.160), (0.0, 0.020, -0.160), 0.003),
            ],
            radial_segments=12,
        )
    )
    return geom


def _add_leg_frame(
    model: ArticulatedObject,
    root,
    *,
    name: str,
    y_sign: float,
    y_hinge: float,
) -> None:
    leg = model.part(name)
    # Child frame is the long folding hinge line.  The leg pair is connected by
    # top and lower cross tubes, so the visual reads as one folding support.
    segments = [
        ((-0.88, 0.000, 0.000), (-0.88, 0.315 * y_sign, -0.745), 0.015),
        ((0.88, 0.000, 0.000), (0.88, 0.315 * y_sign, -0.745), 0.015),
        ((-0.88, 0.000, 0.000), (0.88, 0.000, 0.000), 0.012),
        ((-0.88, 0.315 * y_sign, -0.745), (0.88, 0.315 * y_sign, -0.745), 0.011),
        ((-0.88, 0.155 * y_sign, -0.365), (0.88, 0.155 * y_sign, -0.365), 0.008),
    ]
    _straight_frame_visual(leg, "folding_tube_frame", segments, ALUMINUM)
    for i, x in enumerate((-0.88, 0.88)):
        leg.visual(
            Box((0.075, 0.045, 0.035)),
            origin=Origin(xyz=(x, 0.330 * y_sign, -0.760)),
            material=BLACK_PLASTIC,
            name=f"rubber_foot_{i}",
        )

    model.articulation(
        f"frame_to_{name}",
        ArticulationType.REVOLUTE,
        parent=root,
        child=leg,
        origin=Origin(xyz=(0.0, y_hinge, 0.775)),
        axis=(1.0 * y_sign, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=0.95),
    )


def _add_wing(
    model: ArticulatedObject,
    root,
    *,
    name: str,
    y_sign: float,
    y_hinge: float,
    z_hinge: float = 1.035,
    wing_scale: float = 1.0,
    bar_count: int = 6,
    lower_limit: float = -0.55,
    upper_limit: float = 0.85,
) -> None:
    wing = model.part(name)
    # An inclined rectangular drying wing: bar_count parallel clothes bars plus
    # two side rails.  The inner edge deliberately clears the fixed hinge rail.
    # wing_scale controls the outward reach and rise relative to the baseline.
    outer_y = 0.48 * wing_scale * y_sign
    outer_z = 0.205 * wing_scale
    inner_step_y = 0.030 * wing_scale * y_sign
    connect_y = 0.085 * wing_scale * y_sign
    connect_z = 0.025 * wing_scale
    segments = [
        ((-0.86, 0.000, 0.000), (0.86, 0.000, 0.000), 0.009),
        ((-0.70, 0.000, 0.000), (-0.70, connect_y, connect_z), 0.008),
        ((0.70, 0.000, 0.000), (0.70, connect_y, connect_z), 0.008),
        ((-0.86, inner_step_y, 0.000), (-0.86, outer_y, outer_z), 0.010),
        ((0.86, inner_step_y, 0.000), (0.86, outer_y, outer_z), 0.010),
    ]
    bar_span = 0.480 * wing_scale - 0.030
    for i in range(bar_count):
        t = (i + 1) / bar_count
        y = (0.030 * wing_scale + bar_span * t) * y_sign
        z = outer_z * t
        radius = 0.011 if (i == 0 or i == bar_count - 1) else 0.008
        segments.append(((-0.86, y, z), (0.86, y, z), radius))
    _straight_frame_visual(wing, "wing_tube_frame", segments, ALUMINUM)
    for i, x in enumerate((-0.89, 0.89)):
        wing.visual(
            Box((0.055, 0.040, 0.032)),
            origin=Origin(xyz=(x, outer_y + 0.012 * y_sign, outer_z)),
            material=BLACK_PLASTIC,
            name=f"outer_cap_{i}",
        )

    model.articulation(
        f"frame_to_{name}",
        ArticulationType.REVOLUTE,
        parent=root,
        child=wing,
        origin=Origin(xyz=(0.0, y_hinge, z_hinge)),
        axis=(1.0 * y_sign, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=lower_limit, upper=upper_limit),
    )


# ---------------------------------------------------------------------------
# Wing configuration table — upper pair (full-size) + lower secondary tier.
# All four wings share the same _add_wing helper and uniform REVOLUTE policy.
# ---------------------------------------------------------------------------
_WING_CONFIGS = [
    dict(name="upper_front_wing", y_sign=1.0, y_hinge=0.18, z_hinge=1.035,
         wing_scale=1.0, bar_count=6, lower_limit=-0.55, upper_limit=0.85),
    dict(name="upper_rear_wing", y_sign=-1.0, y_hinge=-0.18, z_hinge=1.035,
         wing_scale=1.0, bar_count=6, lower_limit=-0.55, upper_limit=0.85),
    dict(name="lower_front_wing", y_sign=1.0, y_hinge=0.27, z_hinge=0.800,
         wing_scale=0.60, bar_count=4, lower_limit=-0.30, upper_limit=0.70),
    dict(name="lower_rear_wing", y_sign=-1.0, y_hinge=-0.27, z_hinge=0.800,
         wing_scale=0.60, bar_count=4, lower_limit=-0.30, upper_limit=0.70),
]

# Maps each wing to the main-frame rail it hinges on (for overlap allowances).
_WING_HINGE_RAILS = {
    "upper_front_wing": "front_hinge_rail",
    "upper_rear_wing": "rear_hinge_rail",
    "lower_front_wing": "front_lower_bar",
    "lower_rear_wing": "rear_lower_bar",
}


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="freestanding_laundry_drying_rack",
        meta={
            "category_note": "Reference and category both indicate a freestanding laundry drying rack; no mismatch suspected."
        },
    )

    rack = model.part("main_frame")

    # Long fixed central rack, proportioned like a household floor-standing rack.
    for y, z, r, name in [
        (0.00, 1.075, 0.013, "top_center_rail"),
        (0.18, 1.035, 0.012, "front_hinge_rail"),
        (-0.18, 1.035, 0.012, "rear_hinge_rail"),
        (0.00, 0.785, 0.012, "lower_towel_rail"),
        (0.27, 0.800, 0.010, "front_lower_bar"),
        (-0.27, 0.800, 0.010, "rear_lower_bar"),
    ]:
        _tube_visual(rack, name, (-0.88, y, z), (0.88, y, z), r, ALUMINUM)

    # End cross members and light hinge plates tie all fixed bars into one rigid body.
    for x in (-0.88, 0.88):
        _tube_visual(rack, f"end_top_cross_{x:+.0f}", (x, -0.20, 1.035), (x, 0.20, 1.035), 0.010, ALUMINUM)
        _tube_visual(rack, f"end_lower_cross_{x:+.0f}", (x, -0.29, 0.800), (x, 0.29, 0.800), 0.010, ALUMINUM)
        _tube_visual(rack, f"end_front_strut_{x:+.0f}", (x, 0.18, 1.035), (x, 0.27, 0.800), 0.009, ALUMINUM)
        _tube_visual(rack, f"end_rear_strut_{x:+.0f}", (x, -0.18, 1.035), (x, -0.27, 0.800), 0.009, ALUMINUM)
        _tube_visual(rack, f"end_center_post_{x:+.0f}", (x, 0.00, 1.075), (x, 0.00, 0.785), 0.010, ALUMINUM)
        rack.visual(
            Box((0.050, 0.070, 0.090)),
            origin=Origin(xyz=(x, 0.0, 0.785)),
            material=PALE_PLASTIC,
            name=f"center_hinge_plate_{x:+.0f}",
        )
        # Upper wing hinge blocks (at the upper hinge rail level)
        for y in (-0.18, 0.18):
            rack.visual(
                Box((0.045, 0.035, 0.040)),
                origin=Origin(xyz=(x, y, 1.035)),
                material=PALE_PLASTIC,
                name=f"wing_hinge_block_{x:+.0f}_{y:+.0f}",
            )
        # Lower wing hinge blocks (at the lower bar level)
        for y in (-0.27, 0.27):
            rack.visual(
                Box((0.040, 0.030, 0.028)),
                origin=Origin(xyz=(x, y, 0.814)),
                material=PALE_PLASTIC,
                name=f"lower_wing_hinge_block_{x:+.0f}_{y:+.0f}",
            )
        # Leg hinge blocks
        for y in (-0.27, 0.27):
            rack.visual(
                Box((0.050, 0.043, 0.035)),
                origin=Origin(xyz=(x, y, 0.785)),
                material=PALE_PLASTIC,
                name=f"leg_hinge_block_{x:+.0f}_{y:+.0f}",
            )
        for y, z, cap_name in [
            (0.18, 1.035, "front_top"),
            (-0.18, 1.035, "rear_top"),
            (0.27, 0.800, "front_lower"),
            (-0.27, 0.800, "rear_lower"),
        ]:
            rack.visual(
                Box((0.030, 0.030, 0.030)),
                origin=Origin(xyz=(x + (0.028 if x > 0 else -0.028), y, z)),
                material=BLACK_PLASTIC,
                name=f"{cap_name}_end_cap_{x:+.0f}",
            )

    # Vertical center-plane stabilizer struts connecting the lower towel rail
    # to the top center rail.  Positioned at y=0 (frame centerline) so they
    # never cross any wing hinge bar or leg frame path.
    for x_pos in (-0.70, 0.70):
        _tube_visual(rack, f"center_strut_{x_pos:+.1f}", (x_pos, 0.000, 0.785), (x_pos, 0.000, 1.075), 0.006, DARK_METAL, radial_segments=14)
        for z, tag in ((0.785, "lower"), (1.075, "upper")):
            rack.visual(
                Box((0.024, 0.016, 0.016)),
                origin=Origin(xyz=(x_pos, 0.000, z)),
                material=DARK_METAL,
                name=f"strut_clip_{x_pos:+.1f}_{tag}",
            )

    _add_leg_frame(model, rack, name="front_legs", y_sign=1.0, y_hinge=0.285)
    _add_leg_frame(model, rack, name="rear_legs", y_sign=-1.0, y_hinge=-0.285)

    # Four hinged drying wings emitted through a shared loop with uniform joint policy.
    for cfg in _WING_CONFIGS:
        _add_wing(model, rack, **cfg)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rack = object_model.get_part("main_frame")
    front_legs = object_model.get_part("front_legs")
    rear_legs = object_model.get_part("rear_legs")

    wing_names = [cfg["name"] for cfg in _WING_CONFIGS]
    wings = {name: object_model.get_part(name) for name in wing_names}

    # --- Overlap allowances: wing inner bars coaxial with their hinge rails ---
    for wing_name, rail_name in _WING_HINGE_RAILS.items():
        ctx.allow_overlap(
            wing_name,
            "main_frame",
            elem_a="wing_tube_frame",
            elem_b=rail_name,
            reason=f"The {wing_name} inner tube is intentionally coaxial with the fixed hinge rail to represent a captured folding hinge.",
        )

    # --- Overlap allowances: lower wing bars pass near the leg hinge blocks ---
    # The lower wing hinge line (y=±0.27, z=0.800) shares the frame end region
    # with the leg hinge blocks (x=±0.88, y=±0.27, z=0.785).  This is realistic
    # hardware proximity on a folding rack.
    for wing_name, x_code, y_code in [
        ("lower_front_wing", "-1", "+0"),
        ("lower_front_wing", "+1", "+0"),
        ("lower_rear_wing", "-1", "-0"),
        ("lower_rear_wing", "+1", "-0"),
    ]:
        ctx.allow_overlap(
            wing_name,
            "main_frame",
            elem_a="wing_tube_frame",
            elem_b=f"leg_hinge_block_{x_code}_{y_code}",
            reason=f"The {wing_name} bar passes near the leg hinge block at the frame end; local hardware proximity at the shared hinge level.",
        )

    # --- Overlap allowances: leg tubes captured in hinge blocks ---
    for leg_name, y_code in (("front_legs", "+0"), ("rear_legs", "-0")):
        for x_code in ("-1", "+1"):
            ctx.allow_overlap(
                leg_name,
                "main_frame",
                elem_a="folding_tube_frame",
                elem_b=f"leg_hinge_block_{x_code}_{y_code}",
                reason="The folding leg top tube is locally captured in the plastic hinge block.",
            )

    # --- Structural counts: 1 frame + 2 leg frames + 4 wings = 7 parts, 6 joints ---
    ctx.check(
        "rack has folding supports and four hinged drying wings (two tiers)",
        len(object_model.parts) == 7 and len(object_model.articulations) == 6,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )

    # --- Leg frame geometry ---
    ctx.expect_overlap(
        front_legs,
        rear_legs,
        axes="x",
        min_overlap=1.3,
        elem_a="folding_tube_frame",
        elem_b="folding_tube_frame",
        name="opposing folding leg frames span the rack length",
    )
    ctx.expect_gap(
        rack,
        front_legs,
        axis="z",
        min_gap=0.70,
        positive_elem="lower_towel_rail",
        negative_elem="rubber_foot_0",
        name="front legs sit well below the drying rails",
    )
    ctx.expect_gap(
        rack,
        rear_legs,
        axis="z",
        min_gap=0.70,
        positive_elem="lower_towel_rail",
        negative_elem="rubber_foot_0",
        name="rear legs sit well below the drying rails",
    )

    # --- Upper wings share the long hinge span ---
    ctx.expect_overlap(
        wings["upper_front_wing"],
        rack,
        axes="x",
        min_overlap=1.6,
        elem_a="wing_tube_frame",
        elem_b="front_hinge_rail",
        name="upper front wing shares the long hinge span",
    )
    ctx.expect_overlap(
        wings["upper_rear_wing"],
        rack,
        axes="x",
        min_overlap=1.6,
        elem_a="wing_tube_frame",
        elem_b="rear_hinge_rail",
        name="upper rear wing shares the long hinge span",
    )

    # --- Lower wings share the lower bar hinge span ---
    ctx.expect_overlap(
        wings["lower_front_wing"],
        rack,
        axes="x",
        min_overlap=1.6,
        elem_a="wing_tube_frame",
        elem_b="front_lower_bar",
        name="lower front wing shares the lower bar hinge span",
    )
    ctx.expect_overlap(
        wings["lower_rear_wing"],
        rack,
        axes="x",
        min_overlap=1.6,
        elem_a="wing_tube_frame",
        elem_b="rear_lower_bar",
        name="lower rear wing shares the lower bar hinge span",
    )

    # --- Articulation references ---
    upper_front_joint = object_model.get_articulation("frame_to_upper_front_wing")
    upper_rear_joint = object_model.get_articulation("frame_to_upper_rear_wing")
    lower_front_joint = object_model.get_articulation("frame_to_lower_front_wing")
    lower_rear_joint = object_model.get_articulation("frame_to_lower_rear_wing")
    front_leg_joint = object_model.get_articulation("frame_to_front_legs")

    # --- Upper wing motion: positive angle lifts the outer drying wings ---
    rest_upper_front = ctx.part_world_aabb(wings["upper_front_wing"])
    with ctx.pose({upper_front_joint: 0.55, upper_rear_joint: 0.55}):
        moved_upper_front = ctx.part_world_aabb(wings["upper_front_wing"])
        moved_upper_rear = ctx.part_world_aabb(wings["upper_rear_wing"])
    ctx.check(
        "positive upper wing hinge motion lifts the outer drying wings",
        rest_upper_front is not None
        and moved_upper_front is not None
        and moved_upper_rear is not None
        and moved_upper_front[1][2] > rest_upper_front[1][2] + 0.06
        and moved_upper_rear[1][2] > 1.25,
        details=f"rest={rest_upper_front}, moved_front={moved_upper_front}, moved_rear={moved_upper_rear}",
    )

    # --- Lower tier wing motion (variant-specific axis check) ---
    # Proves that lower_front_wing and lower_rear_wing have real REVOLUTE joints
    # anchored on the lower bar hinge line and that positive q lifts them.
    rest_lower_front = ctx.part_world_aabb(wings["lower_front_wing"])
    with ctx.pose({lower_front_joint: 0.50, lower_rear_joint: 0.50}):
        moved_lower_front = ctx.part_world_aabb(wings["lower_front_wing"])
        moved_lower_rear = ctx.part_world_aabb(wings["lower_rear_wing"])
    ctx.check(
        "lower_front_wing and lower_rear_wing articulate on their own lower-tier hinge line",
        rest_lower_front is not None
        and moved_lower_front is not None
        and moved_lower_rear is not None
        and moved_lower_front[1][2] > rest_lower_front[1][2] + 0.02
        and moved_lower_rear[1][2] > 0.85,
        details=f"rest={rest_lower_front}, moved_front={moved_lower_front}, moved_rear={moved_lower_rear}",
    )

    # --- Lower tier wings sit below upper tier at rest ---
    ctx.check(
        "lower tier wings are vertically below upper tier wings at rest",
        rest_lower_front is not None
        and rest_upper_front is not None
        and rest_lower_front[1][2] < rest_upper_front[0][2],
        details=f"lower_max_z={rest_lower_front[1][2] if rest_lower_front else None}, upper_min_z={rest_upper_front[0][2] if rest_upper_front else None}",
    )

    # --- Leg folding motion ---
    leg_rest = ctx.part_world_aabb(front_legs)
    with ctx.pose({front_leg_joint: 0.55}):
        leg_moved = ctx.part_world_aabb(front_legs)
    ctx.check(
        "front folding leg joint raises the rubber feet",
        leg_rest is not None and leg_moved is not None and leg_moved[0][2] > leg_rest[0][2] + 0.05,
        details=f"rest={leg_rest}, moved={leg_moved}",
    )

    return ctx.report()


object_model = build_object_model()
