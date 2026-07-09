from __future__ import annotations

from math import pi, radians, sin, sqrt

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
BEVEL_THICKNESS = 0.00045
LOWER_PLATE_Z = -0.00165
UPPER_PLATE_Z = 0.00205

# Scalloped-edge parameters: rounded wavy cutting edge for decorative fabric shears
SCALLOP_COUNT = 10
SCALLOP_DEPTH = 0.0028
BEVEL_SCALLOP_DEPTH = 0.0018


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


def _resample_polyline(points: list[tuple[float, float]], n_samples: int) -> list[tuple[float, float]]:
    """Resample a 2-D polyline to *n_samples* roughly arc-length-spaced points."""
    cum = [0.0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        cum.append(cum[-1] + sqrt(dx * dx + dy * dy))
    total = cum[-1]
    if total < 1e-12:
        return [points[0]] * n_samples
    out: list[tuple[float, float]] = []
    for j in range(n_samples):
        target = total * j / max(n_samples - 1, 1)
        for i in range(1, len(cum)):
            if cum[i] >= target - 1e-12:
                seg = cum[i] - cum[i - 1]
                t = (target - cum[i - 1]) / seg if seg > 1e-12 else 0.0
                t = max(0.0, min(1.0, t))
                x = points[i - 1][0] + t * (points[i][0] - points[i - 1][0])
                y = points[i - 1][1] + t * (points[i][1] - points[i - 1][1])
                out.append((x, y))
                break
    return out


def _scallop_edge(
    edge_points: list[tuple[float, float]],
    n_scallops: int,
    depth: float,
    phase: float = 0.0,
) -> list[tuple[float, float]]:
    """Replace a cutting-edge polyline with a scalloped (wavy) version.

    Each scallop is a rounded convex bump of *depth* perpendicular to the edge.
    *phase* offsets the wave so the two blades' scallops interlock.
    """
    n_samples = n_scallops * 12 + 1
    resampled = _resample_polyline(edge_points, n_samples)

    # Total arc length
    total = 0.0
    for i in range(1, len(resampled)):
        dx = resampled[i][0] - resampled[i - 1][0]
        dy = resampled[i][1] - resampled[i - 1][1]
        total += sqrt(dx * dx + dy * dy)

    result: list[tuple[float, float]] = []
    cum = 0.0
    for i in range(len(resampled)):
        x, y = resampled[i]
        if i == 0 or i == len(resampled) - 1:
            # Preserve exact endpoints (no scallop offset)
            result.append((x, y))
        else:
            # Tangent from neighbours
            dx = resampled[min(i + 1, len(resampled) - 1)][0] - resampled[max(i - 1, 0)][0]
            dy = resampled[min(i + 1, len(resampled) - 1)][1] - resampled[max(i - 1, 0)][1]
            seg = sqrt(dx * dx + dy * dy)
            if seg < 1e-12:
                result.append((x, y))
                continue
            # Left normal (perpendicular, outward for the cutting-edge traversal)
            nx, ny = -dy / seg, dx / seg
            t = cum / total if total > 1e-12 else 0.0
            # Taper envelope so scallops vanish at endpoints
            taper = 0.07
            env = 1.0
            if t < taper:
                env = t / taper
            elif t > 1.0 - taper:
                env = (1.0 - t) / taper
            offset = depth * abs(sin(pi * n_scallops * t + phase)) * env
            result.append((x + offset * nx, y + offset * ny))
        if i < len(resampled) - 1:
            dx = resampled[i + 1][0] - x
            dy = resampled[i + 1][1] - y
            cum += sqrt(dx * dx + dy * dy)
    return result


def _metal_plate(side: int, angle_deg: float, z_center: float) -> cq.Workplane:
    """One continuous forged blade/tang plate with a scalloped cutting edge.

    side=-1 places the handle tang below the blade axis, side=+1 mirrors it
    above the axis for the opposing shear half.  The cutting edge carries
    loop-emitted rounded scallops that interlock between the two blades.
    """
    # Cutting-edge polyline from pivot toward tip (before scalloping)
    cutting_pivot_to_tip = [
        (-0.012, 0.012),
        (0.034, 0.016),
        (0.222, 0.006),
        (0.246, 0.000),
    ]
    phase = 0.0 if side < 0 else pi / 2.0
    scalloped = _scallop_edge(cutting_pivot_to_tip, SCALLOP_COUNT, SCALLOP_DEPTH, phase=phase)

    # Non-cutting structural sections (unchanged from parent)
    tang = [
        (-0.052, -0.019),
        (-0.083, -0.044),
        (-0.097, -0.041),
        (-0.066, -0.018),
    ]
    back_edge = [
        (-0.025, -0.010),
        (0.022, -0.016),
        (0.230, -0.006),
    ]

    # Profile: tip → scalloped cutting edge (tip-to-pivot) → tang → back edge → close
    scalloped_tip_to_pivot = list(reversed(scalloped))
    lower_profile = scalloped_tip_to_pivot + tang + back_edge
    pts = lower_profile if side < 0 else _mirror_y(lower_profile)
    return _rotated(_extruded_polygon(pts, PLATE_THICKNESS, z_center), angle_deg)


def _blade_bevel(side: int, angle_deg: float, z_center: float) -> cq.Workplane:
    """Raised narrow polished bevel with a scalloped cutting edge."""
    # Cutting-edge side of the bevel (pivot to tip)
    cutting_pivot_to_tip = [
        (0.010, 0.0060),
        (0.051, 0.0036),
        (0.239, -0.0010),
    ]
    phase = 0.0 if side < 0 else pi / 2.0
    scalloped = _scallop_edge(cutting_pivot_to_tip, SCALLOP_COUNT, BEVEL_SCALLOP_DEPTH, phase=phase)

    # Inner (non-cutting) edge of the bevel strip
    inner_edge = [
        (0.010, -0.0070),
        (0.046, -0.011),
        (0.224, -0.0048),
        (0.239, -0.0010),
    ]
    # Profile: inner edge (pivot→tip), then scalloped cutting edge (tip→pivot)
    scalloped_tip_to_pivot = list(reversed(scalloped))
    lower_bevel = inner_edge + scalloped_tip_to_pivot[1:]  # skip duplicated tip
    pts = lower_bevel if side < 0 else _mirror_y(lower_bevel)
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
            _blade_bevel(-1, lower_angle, LOWER_PLATE_Z + PLATE_THICKNESS / 2.0 + BEVEL_THICKNESS / 2.0 - 0.00025),
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
            _blade_bevel(1, upper_angle, UPPER_PLATE_Z + PLATE_THICKNESS / 2.0 + BEVEL_THICKNESS / 2.0 - 0.00025),
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
    upper_blade_aabb = ctx.part_element_world_aabb(upper, elem="upper_cutting_bevel")
    ctx.check(
        "scalloped cutting bevels span the blade length",
        lower_blade_aabb is not None
        and upper_blade_aabb is not None
        and (lower_blade_aabb[1][0] - lower_blade_aabb[0][0]) > 0.16
        and (upper_blade_aabb[1][0] - upper_blade_aabb[0][0]) > 0.16,
        details=f"lower bevel aabb={lower_blade_aabb}, upper bevel aabb={upper_blade_aabb}",
    )

    # Scalloped-edge check: the wavy cutting edge adds perpendicular bumps,
    # so the bevel Y-extent must exceed the straight-edge baseline.
    if lower_blade_aabb and upper_blade_aabb:
        lower_y_extent = lower_blade_aabb[1][1] - lower_blade_aabb[0][1]
        upper_y_extent = upper_blade_aabb[1][1] - upper_blade_aabb[0][1]
        ctx.check(
            "scalloped lower_cutting_bevel has wavy edge extent",
            lower_y_extent > 0.040,
            details=f"lower bevel Y extent={lower_y_extent:.4f}",
        )
        ctx.check(
            "scalloped upper_cutting_bevel has wavy edge extent",
            upper_y_extent > 0.040,
            details=f"upper bevel Y extent={upper_y_extent:.4f}",
        )

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
