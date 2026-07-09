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
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
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

    # Swivel caster wheels at each leg tip. Each wheel is a separate part
    # mounted via a CONTINUOUS revolute joint at the leg-tip contact point,
    # so the rack becomes a mobile floor rack while keeping folding legs.
    wheel_radius = 0.025
    wheel_width = 0.018
    for i, x in enumerate((-0.88, 0.88)):
        wheel_part = model.part(f"{name}_caster_wheel_{i}")
        wheel_geom = WheelGeometry(
            wheel_radius,
            wheel_width,
            rim=WheelRim(
                inner_radius=0.018,
                flange_height=0.003,
                flange_thickness=0.002,
                bead_seat_depth=0.001,
            ),
            hub=WheelHub(radius=0.008, width=0.014, cap_style="flat"),
            bore=WheelBore(style="round", diameter=0.006),
        )
        wheel_part.visual(
            mesh_from_geometry(wheel_geom, f"caster_wheel_{i}"),
            material=DARK_METAL,
            name=f"caster_wheel_{i}",
        )
        # Wheel spins about local X (like a rolling wheel on the floor).
        model.articulation(
            f"{name}_to_caster_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=leg,
            child=wheel_part,
            origin=Origin(xyz=(x, 0.330 * y_sign, -0.760)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=10.0),
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
) -> None:
    wing = model.part(name)
    # An inclined rectangular drying wing: five parallel clothes bars plus two
    # side rails.  The inner edge deliberately clears the fixed hinge rail.
    outer_y = 0.48 * y_sign
    outer_z = 0.205
    segments = [
        ((-0.86, 0.000, 0.000), (0.86, 0.000, 0.000), 0.009),
        ((-0.70, 0.000, 0.000), (-0.70, 0.085 * y_sign, 0.025), 0.008),
        ((0.70, 0.000, 0.000), (0.70, 0.085 * y_sign, 0.025), 0.008),
        ((-0.86, 0.030 * y_sign, 0.000), (-0.86, outer_y, outer_z), 0.010),
        ((0.86, 0.030 * y_sign, 0.000), (0.86, outer_y, outer_z), 0.010),
    ]
    for i, t in enumerate((0.12, 0.30, 0.48, 0.66, 0.84, 1.0)):
        y = (0.030 + (0.480 - 0.030) * t) * y_sign
        z = outer_z * t
        radius = 0.011 if i in (0, 5) else 0.008
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
        origin=Origin(xyz=(0.0, y_hinge, 1.035)),
        axis=(1.0 * y_sign, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.55, upper=0.85),
    )


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
        for y in (-0.18, 0.18):
            rack.visual(
                Box((0.045, 0.035, 0.040)),
                origin=Origin(xyz=(x, y, 1.035)),
                material=PALE_PLASTIC,
                name=f"wing_hinge_block_{x:+.0f}_{y:+.0f}",
            )
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

    # Small scissor-style stabilizer links on each side, as visible in the reference.
    for y_sign, side_name in ((1.0, "front"), (-1.0, "rear")):
        y = 0.285 * y_sign
        lower_y = 0.270 * y_sign
        upper_y = 0.000
        _tube_visual(rack, f"{side_name}_brace_upper", (-0.70, lower_y, 0.800), (-0.30, upper_y, 1.075), 0.006, DARK_METAL, radial_segments=14)
        _tube_visual(rack, f"{side_name}_brace_lower", (0.70, lower_y, 0.800), (0.30, upper_y, 1.075), 0.006, DARK_METAL, radial_segments=14)
        for x in (-0.70, 0.70):
            is_lower = abs(x) > 0.5
            rack.visual(
                Box((0.032, 0.016, 0.016)),
                origin=Origin(xyz=(x, lower_y if is_lower else upper_y, 0.800 if is_lower else 1.035)),
                material=DARK_METAL,
                name=f"{side_name}_brace_rivet_{x:+.1f}",
            )

    _add_leg_frame(model, rack, name="front_legs", y_sign=1.0, y_hinge=0.285)
    _add_leg_frame(model, rack, name="rear_legs", y_sign=-1.0, y_hinge=-0.285)
    _add_wing(model, rack, name="front_wing", y_sign=1.0, y_hinge=0.18)
    _add_wing(model, rack, name="rear_wing", y_sign=-1.0, y_hinge=-0.18)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rack = object_model.get_part("main_frame")
    front_legs = object_model.get_part("front_legs")
    rear_legs = object_model.get_part("rear_legs")
    front_wing = object_model.get_part("front_wing")
    rear_wing = object_model.get_part("rear_wing")

    for wing_name, rail_name in (
        ("front_wing", "front_hinge_rail"),
        ("rear_wing", "rear_hinge_rail"),
    ):
        ctx.allow_overlap(
            wing_name,
            "main_frame",
            elem_a="wing_tube_frame",
            elem_b=rail_name,
            reason="The wing inner tube is intentionally coaxial with the fixed hinge rail to represent a captured folding hinge.",
        )

    for leg_name, y_code in (("front_legs", "+0"), ("rear_legs", "-0")):
        for x_code in ("-1", "+1"):
            ctx.allow_overlap(
                leg_name,
                "main_frame",
                elem_a="folding_tube_frame",
                elem_b=f"leg_hinge_block_{x_code}_{y_code}",
                reason="The folding leg top tube is locally captured in the plastic hinge block.",
            )
    
    # Caster wheels are mounted at leg tips, intentionally overlapping the leg frame
    for leg_name in ("front_legs", "rear_legs"):
        for i in (0, 1):
            wheel_part_name = f"{leg_name}_caster_wheel_{i}"
            ctx.allow_overlap(
                leg_name,
                wheel_part_name,
                elem_a="folding_tube_frame",
                elem_b=f"caster_wheel_{i}",
                reason="Caster wheel is mounted at the leg tip contact point, representing a swivel caster attachment.",
            )

    # Verify caster wheel structure: 5 base parts + 4 caster wheels = 9 parts total
    # Joints: 4 original (2 legs + 2 wings) + 4 caster wheel joints = 8 total
    ctx.check(
        "rack has folding supports, hinged wings, and caster wheels",
        len(object_model.parts) == 9 and len(object_model.articulations) == 8,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )
    
    # Verify each caster wheel has a CONTINUOUS joint
    for leg_name in ("front_legs", "rear_legs"):
        for i in (0, 1):
            wheel_part_name = f"{leg_name}_caster_wheel_{i}"
            joint_name = f"{leg_name}_to_caster_wheel_{i}"
            wheel_part = object_model.get_part(wheel_part_name)
            wheel_joint = object_model.get_articulation(joint_name)
            ctx.check(
                f"{wheel_part_name} has CONTINUOUS rolling joint",
                wheel_joint.articulation_type == "continuous",
                details=f"joint_type={wheel_joint.articulation_type}",
            )
    
    ctx.expect_overlap(
        front_legs,
        rear_legs,
        axes="x",
        min_overlap=1.3,
        elem_a="folding_tube_frame",
        elem_b="folding_tube_frame",
        name="opposing folding leg frames span the rack length",
    )
    # Verify caster wheels are at ground level, well below the drying rails
    for leg_name in ("front_legs", "rear_legs"):
        for i in (0, 1):
            wheel_part_name = f"{leg_name}_caster_wheel_{i}"
            wheel_part = object_model.get_part(wheel_part_name)
            ctx.expect_gap(
                rack,
                wheel_part,
                axis="z",
                min_gap=0.70,
                positive_elem="lower_towel_rail",
                negative_elem=f"caster_wheel_{i}",
                name=f"{wheel_part_name} sits well below the drying rails",
            )
    ctx.expect_overlap(
        front_wing,
        rack,
        axes="x",
        min_overlap=1.6,
        elem_a="wing_tube_frame",
        elem_b="front_hinge_rail",
        name="front wing shares the long hinge span",
    )
    ctx.expect_overlap(
        rear_wing,
        rack,
        axes="x",
        min_overlap=1.6,
        elem_a="wing_tube_frame",
        elem_b="rear_hinge_rail",
        name="rear wing shares the long hinge span",
    )

    front_joint = object_model.get_articulation("frame_to_front_wing")
    rear_joint = object_model.get_articulation("frame_to_rear_wing")
    front_leg_joint = object_model.get_articulation("frame_to_front_legs")
    rest_front = ctx.part_world_aabb(front_wing)
    with ctx.pose({front_joint: 0.55, rear_joint: 0.55}):
        moved_front = ctx.part_world_aabb(front_wing)
        moved_rear = ctx.part_world_aabb(rear_wing)
    ctx.check(
        "positive wing hinge motion lifts the outer drying wings",
        rest_front is not None
        and moved_front is not None
        and moved_rear is not None
        and moved_front[1][2] > rest_front[1][2] + 0.06
        and moved_rear[1][2] > 1.25,
        details=f"rest_front={rest_front}, moved_front={moved_front}, moved_rear={moved_rear}",
    )

    leg_rest = ctx.part_world_aabb(front_legs)
    with ctx.pose({front_leg_joint: 0.55}):
        leg_moved = ctx.part_world_aabb(front_legs)
    ctx.check(
        "front folding leg joint raises the caster wheels",
        leg_rest is not None and leg_moved is not None and leg_moved[0][2] > leg_rest[0][2] + 0.05,
        details=f"rest={leg_rest}, moved={leg_moved}",
    )

    return ctx.report()


object_model = build_object_model()
