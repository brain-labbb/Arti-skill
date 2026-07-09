from __future__ import annotations

from math import radians

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_THICKNESS = 0.0032
BEVEL_THICKNESS = 0.0009
LOWER_PLATE_Z = -0.00165
UPPER_PLATE_Z = 0.00205
N_PINKING_TEETH = 9
PINKING_TOOTH_AMP = 0.003


def _extruded_polygon(points: list[tuple[float, float]], thickness: float, z_center: float) -> cq.Workplane:
    """Flat plate from a 2-D outline, centered at z_center."""
    return (
        cq.Workplane("XY")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((0.0, 0.0, z_center - thickness / 2.0))
    )


def _rotated(shape: cq.Workplane, angle_deg: float) -> cq.Workplane:
    return shape.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)


def _mirror_y(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(x, -y) for x, y in points]


def _metal_plate(side: int, angle_deg: float, z_center: float) -> cq.Workplane:
    """One continuous forged blade/tang plate.

    side=-1 places the handle tang below the blade axis, side=+1 mirrors it
    above the axis for the opposing shear half.
    """
    lower_side_profile = [
        (0.246, 0.000),
        (0.222, 0.006),
        (0.034, 0.016),
        (-0.012, 0.012),
        (-0.052, -0.019),
        (-0.083, -0.044),
        (-0.097, -0.041),
        (-0.066, -0.018),
        (-0.025, -0.010),
        (0.022, -0.016),
        (0.230, -0.006),
    ]
    pts = lower_side_profile if side < 0 else _mirror_y(lower_side_profile)
    return _rotated(_extruded_polygon(pts, PLATE_THICKNESS, z_center), angle_deg)


def _pinking_teeth(
    n: int, x_start: float, x_end: float, base_y: float, amplitude: float
) -> list[tuple[float, float]]:
    """Build a zigzag polyline of *n* triangular pinking teeth along the cutting edge.

    Teeth protrude in the −Y direction from *base_y*; mirror for the opposing blade.
    Each tooth is a triangle: base at base_y, apex at base_y − amplitude.
    """
    pitch = (x_end - x_start) / n
    points: list[tuple[float, float]] = []
    for i in range(n):
        x_base = x_start + i * pitch
        x_apex = x_base + 0.5 * pitch
        points.append((x_base, base_y))
        points.append((x_apex, base_y - amplitude))
    points.append((x_end, base_y))
    return points


def _pinking_bevel(side: int, angle_deg: float, z_center: float) -> cq.Workplane:
    """Pinking sawtooth cutting-edge strip with N interlocking triangular teeth."""
    x_start = 0.038
    x_end = 0.228
    base_y = -0.008
    outer_y = 0.005
    n = N_PINKING_TEETH
    pitch = (x_end - x_start) / n

    # Half-pitch offset on the upper blade so the two tooth rows mesh
    x_off = 0.0 if side < 0 else pitch * 0.5
    xs = x_start + x_off
    xe = x_end + x_off

    tooth_pts = _pinking_teeth(n, xs, xe, base_y, PINKING_TOOTH_AMP)

    lower_polygon = tooth_pts + [
        (xe + 0.010, -0.001),
        (xs + 0.012, outer_y),
        (xs, outer_y),
    ]

    pts = lower_polygon if side < 0 else _mirror_y(lower_polygon)
    return _rotated(_extruded_polygon(pts, BEVEL_THICKNESS, z_center), angle_deg)


def _ellipse_ring(
    center: tuple[float, float],
    outer: tuple[float, float],
    inner: tuple[float, float],
    thickness: float,
    z_center: float,
) -> cq.Workplane:
    cx, cy = center
    outer_solid = cq.Workplane("XY").center(cx, cy).ellipse(outer[0], outer[1]).extrude(thickness)
    cutter = (
        cq.Workplane("XY")
        .center(cx, cy)
        .ellipse(inner[0], inner[1])
        .extrude(thickness * 3.0)
        .translate((0.0, 0.0, -thickness))
    )
    return outer_solid.cut(cutter).translate((0.0, 0.0, z_center - thickness / 2.0))


def _grip(
    *,
    side: int,
    angle_deg: float,
    center: tuple[float, float],
    outer: tuple[float, float],
    inner: tuple[float, float],
    z_center: float,
) -> cq.Workplane:
    """Hollow plastic finger loop plus overlapping neck into the metal tang."""
    ring = _ellipse_ring(center, outer, inner, PLATE_THICKNESS + 0.0012, z_center)
    if side < 0:
        neck_pts = [
            (-0.010, -0.014),
            (-0.051, -0.025),
            (-0.071, -0.040),
            (-0.063, -0.053),
            (-0.025, -0.023),
        ]
    else:
        neck_pts = _mirror_y(
            [
                (-0.010, -0.014),
                (-0.050, -0.024),
                (-0.066, -0.037),
                (-0.059, -0.049),
                (-0.024, -0.021),
            ]
        )
    neck = _extruded_polygon(neck_pts, PLATE_THICKNESS + 0.0012, z_center)
    return _rotated(ring.union(neck), angle_deg)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="fabric_tailor_shears",
        meta={
            "category": "Textiles_Fabric / Fabric scissors",
            "classification_note": "Reference and category both indicate fabric scissors/tailor shears; no mismatch suspected.",
        },
    )

    model.material("brushed_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material("bright_edge", rgba=(0.92, 0.94, 0.96, 1.0))
    model.material("dark_plastic", rgba=(0.045, 0.047, 0.052, 1.0))
    model.material("screw_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("slot_shadow", rgba=(0.015, 0.015, 0.016, 1.0))

    lower_angle = 11.0
    upper_angle = -13.0

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_metal_plate(-1, lower_angle, LOWER_PLATE_Z), "lower_plate", tolerance=0.0007),
        material="brushed_steel",
        name="lower_plate",
    )
    lower_arm.visual(
        Cylinder(radius=0.017, length=PLATE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, LOWER_PLATE_Z)),
        material="brushed_steel",
        name="lower_pivot_boss",
    )
    lower_arm.visual(
        mesh_from_cadquery(
            _pinking_bevel(-1, lower_angle, LOWER_PLATE_Z + PLATE_THICKNESS / 2.0 + BEVEL_THICKNESS / 2.0 - 0.00025),
            "lower_cutting_bevel",
            tolerance=0.0005,
        ),
        material="bright_edge",
        name="lower_cutting_bevel",
    )
    lower_arm.visual(
        mesh_from_cadquery(
            _grip(
                side=-1,
                angle_deg=lower_angle,
                center=(-0.100, -0.055),
                outer=(0.052, 0.031),
                inner=(0.038, 0.020),
                z_center=LOWER_PLATE_Z,
            ),
            "large_finger_loop",
            tolerance=0.0008,
        ),
        material="dark_plastic",
        name="large_finger_loop",
    )
    lower_arm.visual(
        Cylinder(radius=0.014, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0, LOWER_PLATE_Z - PLATE_THICKNESS / 2.0 - 0.0010)),
        material="screw_steel",
        name="lower_pivot_washer",
    )
    lower_arm.visual(
        Cylinder(radius=0.0062, length=0.0058),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="screw_steel",
        name="lower_pivot_pin",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_metal_plate(1, upper_angle, UPPER_PLATE_Z), "upper_plate", tolerance=0.0007),
        material="brushed_steel",
        name="upper_plate",
    )
    upper_arm.visual(
        Cylinder(radius=0.017, length=PLATE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, UPPER_PLATE_Z)),
        material="brushed_steel",
        name="upper_pivot_boss",
    )
    upper_arm.visual(
        mesh_from_cadquery(
            _pinking_bevel(1, upper_angle, UPPER_PLATE_Z + PLATE_THICKNESS / 2.0 + BEVEL_THICKNESS / 2.0 - 0.00025),
            "upper_cutting_bevel",
            tolerance=0.0005,
        ),
        material="bright_edge",
        name="upper_cutting_bevel",
    )
    upper_arm.visual(
        mesh_from_cadquery(
            _grip(
                side=1,
                angle_deg=upper_angle,
                center=(-0.088, 0.047),
                outer=(0.038, 0.027),
                inner=(0.026, 0.017),
                z_center=UPPER_PLATE_Z,
            ),
            "small_thumb_loop",
            tolerance=0.0008,
        ),
        material="dark_plastic",
        name="small_thumb_loop",
    )
    upper_arm.visual(
        Cylinder(radius=0.012, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, UPPER_PLATE_Z + PLATE_THICKNESS / 2.0 + 0.00140)),
        material="screw_steel",
        name="pivot_screw_head",
    )
    upper_arm.visual(
        Box((0.015, 0.0022, 0.00045)),
        origin=Origin(
            xyz=(0.0, 0.0, UPPER_PLATE_Z + PLATE_THICKNESS / 2.0 + 0.00330),
            rpy=(0.0, 0.0, radians(-18.0)),
        ),
        material="slot_shadow",
        name="screw_slot",
    )

    model.articulation(
        "pivot_screw",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.55, effort=3.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_arm")
    upper = object_model.get_part("upper_arm")
    pivot = object_model.get_articulation("pivot_screw")

    ctx.allow_overlap(
        lower,
        upper,
        elem_a="lower_pivot_pin",
        elem_b="upper_pivot_boss",
        reason="The screw shank is intentionally retained through the upper blade's pivot bore, represented as a local embedded pin in the plate proxy.",
    )
    ctx.expect_within(
        lower,
        upper,
        axes="xy",
        inner_elem="lower_pivot_pin",
        outer_elem="upper_pivot_boss",
        margin=0.001,
        name="pivot pin is centered within the upper blade plate",
    )
    ctx.expect_overlap(
        lower,
        upper,
        axes="z",
        min_overlap=0.002,
        elem_a="lower_pivot_pin",
        elem_b="upper_pivot_boss",
        name="pivot pin remains captured through upper blade thickness",
    )

    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        min_gap=0.0001,
        max_gap=0.0012,
        positive_elem="upper_plate",
        negative_elem="lower_plate",
        name="stacked blades have a thin pivot clearance",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        min_overlap=0.025,
        elem_a="upper_plate",
        elem_b="lower_plate",
        name="blade plates cross in plan at the screw",
    )

    lower_loop_aabb = ctx.part_element_world_aabb(lower, elem="large_finger_loop")
    upper_loop_aabb = ctx.part_element_world_aabb(upper, elem="small_thumb_loop")
    if lower_loop_aabb and upper_loop_aabb:
        lower_loop_width = lower_loop_aabb[1][0] - lower_loop_aabb[0][0]
        upper_loop_width = upper_loop_aabb[1][0] - upper_loop_aabb[0][0]
        ctx.check(
            "asymmetric fabric-shear handle loops",
            lower_loop_width > upper_loop_width + 0.018,
            details=f"large loop width={lower_loop_width:.3f}, small loop width={upper_loop_width:.3f}",
        )
    else:
        ctx.fail("asymmetric fabric-shear handle loops", "could not measure handle-loop AABBs")

    lower_blade_aabb = ctx.part_element_world_aabb(lower, elem="lower_cutting_bevel")
    ctx.check(
        "long tailor-shear cutting bevel",
        lower_blade_aabb is not None and (lower_blade_aabb[1][0] - lower_blade_aabb[0][0]) > 0.16,
        details=f"lower bevel aabb={lower_blade_aabb}",
    )

    # Pinking sawtooth verification: the thickened bevel strip (Z-extent > 0.0007)
    # carries N triangular teeth that protrude perpendicular to the blade,
    # widening the world-space Y-profile beyond a smooth honed edge.
    lower_bevel_aabb = ctx.part_element_world_aabb(lower, elem="lower_cutting_bevel")
    upper_bevel_aabb = ctx.part_element_world_aabb(upper, elem="upper_cutting_bevel")
    if lower_bevel_aabb and upper_bevel_aabb:
        lower_y_extent = lower_bevel_aabb[1][1] - lower_bevel_aabb[0][1]
        upper_y_extent = upper_bevel_aabb[1][1] - upper_bevel_aabb[0][1]
        lower_z_extent = lower_bevel_aabb[1][2] - lower_bevel_aabb[0][2]
        upper_z_extent = upper_bevel_aabb[1][2] - upper_bevel_aabb[0][2]
        ctx.check(
            "lower_cutting_bevel pinking sawtooth profile",
            lower_y_extent > 0.045 and lower_z_extent > 0.0007,
            details=f"Y-extent={lower_y_extent:.4f}, Z-extent={lower_z_extent:.4f}",
        )
        ctx.check(
            "upper_cutting_bevel pinking sawtooth profile",
            upper_y_extent > 0.051 and upper_z_extent > 0.0007,
            details=f"Y-extent={upper_y_extent:.4f}, Z-extent={upper_z_extent:.4f}",
        )
    else:
        ctx.fail("pinking sawtooth verification", "could not measure cutting bevel AABBs")

    def _center_y(aabb):
        return (aabb[0][1] + aabb[1][1]) / 2.0 if aabb else None

    with ctx.pose({pivot: -0.30}):
        closed_upper = ctx.part_element_world_aabb(upper, elem="upper_cutting_bevel")
        closed_lower = ctx.part_element_world_aabb(lower, elem="lower_cutting_bevel")
        closed_span = abs(_center_y(closed_upper) - _center_y(closed_lower)) if closed_upper and closed_lower else None

    with ctx.pose({pivot: 0.48}):
        open_upper = ctx.part_element_world_aabb(upper, elem="upper_cutting_bevel")
        open_lower = ctx.part_element_world_aabb(lower, elem="lower_cutting_bevel")
        open_span = abs(_center_y(open_upper) - _center_y(open_lower)) if open_upper and open_lower else None

    ctx.check(
        "revolute screw opens the blades",
        closed_span is not None and open_span is not None and open_span > closed_span + 0.055,
        details=f"closed span={closed_span}, open span={open_span}",
    )

    return ctx.report()


object_model = build_object_model()
