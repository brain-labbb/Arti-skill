from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Variant 29: Playground seesaw with A-frame support, molded seats, and
# rounded handle grips.
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Central A-frame support: two angled tube legs per side meeting at the apex,
#   a horizontal cross-brace, and flat axle bracket plates at the top.
# - The rocking beam is a 3.0 m mustard-yellow steel bar (80 x 40 mm) with a
#   pivot sleeve + triangular gusset at center, a molded seat with raised lip
#   per end, a rounded-grip handle, and a curved rubber bumper under each tip.
# - Single revolute joint at the apex, axis (0, 1, 0), +/- 20 degrees.
#   Positive q lowers the +X end (right-hand rule about +Y).
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76  # axle height (~0.8 m tall at the pivot)

# A-frame geometry
AFOOT_X = 0.55  # half-spread of the A-frame feet along beam direction
AFOOT_Y = 0.32  # half-spread of feet laterally
A_APEX_Z = PIVOT_Z
TUBE_R = 0.025  # ~50 mm diameter tube
BRACE_Z = 0.38  # cross-brace height

AXLE_R = 0.016
AXLE_LEN = 0.24

# Beam-local frame: origin at the axle center; the bar bottom sits 50 mm above.
BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0  # 0.07
BAR_TOP = BAR_BOT + BEAM_T  # 0.09

SEAT_X = 1.28
HANDLE_X = 1.02
BUMPER_X = 1.42
TILT = math.radians(20.0)


def _aframe_leg_points(side_x: float, side_y: float) -> list[tuple[float, float, float]]:
    """One A-frame leg: from ground foot to the apex."""
    return [
        (side_x * AFOOT_X, side_y * AFOOT_Y, 0.02),
        (side_x * AFOOT_X * 0.6, side_y * AFOOT_Y * 0.5, A_APEX_Z * 0.5),
        (0.0, 0.0, A_APEX_Z - 0.02),
    ]


def _brace_points() -> list[tuple[float, float, float]]:
    """Horizontal cross-brace connecting the two A-frame sides at mid-height."""
    return [
        (-AFOOT_X * 0.6, -AFOOT_Y * 0.5, BRACE_Z),
        (AFOOT_X * 0.6, -AFOOT_Y * 0.5, BRACE_Z),
    ]


def _brace_points_back() -> list[tuple[float, float, float]]:
    """Rear horizontal cross-brace."""
    return [
        (-AFOOT_X * 0.6, AFOOT_Y * 0.5, BRACE_Z),
        (AFOOT_X * 0.6, AFOOT_Y * 0.5, BRACE_Z),
    ]


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline, plane across the beam (YZ)."""
    half_w = 0.038
    leg_bot = BAR_TOP - 0.010  # rod tip embedded in the beam bar
    arc_z = 0.28
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.190),
    ]
    for k in range(7):  # semicircular top bend
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.190))
    pts.append((x, half_w, leg_bot))
    return pts


def _grip_points(x: float) -> list[tuple[float, float, float]]:
    """Centerline of the rounded grip sleeve over the handle top arc."""
    half_w = 0.038
    arc_z = 0.28
    pts: list[tuple[float, float, float]] = []
    for k in range(7):  # same arc as the handle top
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    return pts


def _bumper_geometry(x: float, index: int):
    """Curved tire-section bumper: half-annulus shell extruded across the beam."""
    r_out = 0.065
    r_in = 0.048
    profile: list[tuple[float, float]] = []
    n = 20
    for k in range(n + 1):  # outer arc, bottom half (pi .. 2*pi)
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):  # inner arc back (2*pi .. pi)
        a = 2.0 * math.pi - math.pi * k / n
        profile.append((r_in * math.cos(a), r_in * math.sin(a)))
    geom = ExtrudeGeometry(profile, 0.10, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(x, 0.0, BAR_BOT + 0.002)
    return mesh_from_geometry(geom, f"seesaw_bumper_{index}")


def _gusset_geometry():
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def _molded_seat_shape() -> cq.Workplane:
    """Molded seat: a shallow dish with a raised lip rim.

    Built as a rounded-rectangle plate with a concave depression scooped out
    of the top and a raised lip rim around the perimeter.
    """
    seat_w = 0.26  # width (along beam, X)
    seat_d = 0.22  # depth (across beam, Y)
    plate_h = 0.018  # total plate thickness
    dish_depth = 0.008
    lip_h = 0.012  # raised lip above the dish
    lip_w = 0.014  # lip wall thickness

    # Start with a solid rounded-rect plate
    plate = (
        cq.Workplane("XY")
        .box(seat_w, seat_d, plate_h)
        .edges("|Z")
        .fillet(0.025)
    )

    # Scoop a concave dish from the top: use a large sphere to cut
    dish_radius = 0.30  # large radius gives a shallow dish
    dish_center_z = plate_h / 2.0 + dish_radius - dish_depth
    plate = (
        plate
        .faces(">Z")
        .workplane()
        .center(0, 0)
        .circle(seat_w * 0.38)
        .cutBlind(-dish_depth)
    )

    # Build the raised lip as a perimeter wall on top of the plate
    lip_outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, plate_h / 2.0))
        .box(seat_w, seat_d, lip_h)
        .edges("|Z")
        .fillet(0.025)
    )
    lip_inner = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, plate_h / 2.0))
        .box(seat_w - 2 * lip_w, seat_d - 2 * lip_w, lip_h + 0.004)
        .edges("|Z")
        .fillet(0.018)
    )
    lip_ring = lip_outer.cut(lip_inner)

    seat = plate.union(lip_ring)
    return seat


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_playground_seesaw")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    dark_green = model.material("molded_seat_green", rgba=(0.18, 0.32, 0.20, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("aframe_base")

    # Four A-frame legs: two per side (front +Y, back -Y)
    leg_configs = [
        (+1.0, +1.0),  # front-right leg
        (+1.0, -1.0),  # back-right leg
        (-1.0, +1.0),  # front-left leg
        (-1.0, -1.0),  # back-left leg
    ]
    for i, (sx, sy) in enumerate(leg_configs):
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _aframe_leg_points(sx, sy),
                    radius=TUBE_R,
                    samples_per_segment=8,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"aframe_leg_{i}",
            ),
            material=galvanized,
            name=f"leg_{i}",
        )

    # Front and back cross-braces at mid-height
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _brace_points(),
                radius=TUBE_R * 0.7,
                samples_per_segment=6,
                radial_segments=14,
                cap_ends=True,
            ),
            "aframe_brace_front",
        ),
        material=galvanized,
        name="brace_front",
    )
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _brace_points_back(),
                radius=TUBE_R * 0.7,
                samples_per_segment=6,
                radial_segments=14,
                cap_ends=True,
            ),
            "aframe_brace_back",
        ),
        material=galvanized,
        name="brace_back",
    )

    # Axle bracket plates: flat steel plates on each side of the apex
    # that carry the axle bolt through drilled holes
    bracket_w = 0.07
    bracket_h = 0.09
    bracket_t = 0.008
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Box((bracket_w, bracket_t, bracket_h)),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.01), A_APEX_Z - 0.01),
            ),
            material=rust,
            name=f"axle_bracket_{i}",
        )
        # Bracket bolt/nut at the top of each bracket plate
        base.visual(
            Cylinder(radius=0.008, length=0.014),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.01), A_APEX_Z + 0.025),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rust,
            name=f"bracket_bolt_{i}",
        )

    # Pivot axle bolt through both bracket plates, axis along Y
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, A_APEX_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_axle",
    )
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.006), A_APEX_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rust,
            name=f"axle_nut_{i}",
        )

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    beam.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=mustard, name="gusset_plate")

    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar",
    )

    # Rust streak patches wrapping the painted bar
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # Molded seats with raised lips at each end
    seat_mesh = mesh_from_cadquery(_molded_seat_shape(), "molded_seat")
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            seat_mesh,
            origin=Origin(xyz=(side * SEAT_X, 0.0, BAR_TOP + 0.009)),
            material=dark_green,
            name=f"seat_{i}",
        )

    # End fittings: handle with rounded grip + bumper
    for i, side in enumerate((1.0, -1.0)):
        # Handle rod
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.009,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"seesaw_handle_{i}",
            ),
            material=pale_steel,
            name=f"handle_{i}",
        )
        # Rounded grip sleeve over the handle top arc
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _grip_points(side * HANDLE_X),
                    radius=0.018,
                    samples_per_segment=8,
                    radial_segments=20,
                    cap_ends=True,
                ),
                f"seesaw_grip_{i}",
            ),
            material=grip_rubber,
            name=f"grip_{i}",
        )
        # Tire-section bumper
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

    # -------------------------------------------------------------- joint ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, A_APEX_Z)),
        axis=(0.0, 1.0, 0.0),  # positive q lowers the +X end
        motion_limits=MotionLimits(effort=200.0, velocity=2.5, lower=-TILT, upper=TILT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("aframe_base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")

    # --- Structural: A-frame support with axle brackets ---
    # Confirm A-frame legs exist and reach the ground
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "A-frame base feet rest on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.03,
        details=f"base aabb={base_box}",
    )
    # Axle brackets present at the apex
    for i in range(2):
        bracket = ctx.part_element_world_aabb(base, elem=f"axle_bracket_{i}")
        ctx.check(
            f"axle_bracket_{i} is present near the apex",
            bracket is not None and bracket[0][2] > 0.60,
            details=f"bracket aabb={bracket}",
        )

    # --- Pivot mechanism ---
    # The beam's pivot sleeve intentionally captures the base axle bolt.
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    # The A-frame legs converge at the apex where the pivot sleeve wraps the axle.
    # Each leg tip meets the pivot area; this small local overlap is intentional.
    for i in range(4):
        ctx.allow_overlap(
            beam,
            base,
            elem_a="pivot_sleeve",
            elem_b=f"leg_{i}",
            reason=f"A-frame leg_{i} converges to the apex to support the pivot bracket.",
        )
    ctx.expect_contact(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        name="pivot sleeve is seated on the axle bolt",
    )
    ctx.expect_within(
        beam,
        base,
        axes="y",
        inner_elem="pivot_sleeve",
        outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve stays inside the axle span",
    )

    # Beam bar rides above the A-frame legs (measured away from the apex).
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="brace_front",
        min_gap=0.02,
        max_gap=0.50,
        name="beam bar clears the cross-brace",
    )

    # Joint configuration: horizontal Y axis, +/- 20 degree rocking limits.
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to the beam",
        abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "rocking limits are about +/- 20 degrees",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Non-fixed joint verification: articulated pose moves beam ends ---
    rest_bumper = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        tilted_bumper = ctx.part_element_world_aabb(beam, elem="bumper_0")
    ctx.check(
        "revolute joint is non-fixed: beam end moves when articulated",
        rest_bumper is not None
        and tilted_bumper is not None
        and abs(rest_bumper[0][2] - tilted_bumper[0][2]) > 0.10,
        details=f"rest_z={rest_bumper[0][2] if rest_bumper else None}, tilted_z={tilted_bumper[0][2] if tilted_bumper else None}",
    )

    # --- Hero geometry: beam scale ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # --- Molded seats with raised lips ---
    for i in range(2):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"seat_{i} is a molded seat seated on the beam bar top",
            seat is not None
            and bar_box is not None
            and abs(seat[0][2] - bar_box[1][2]) < 0.005  # seat bottom flush with bar top
            and seat[1][2] > bar_box[1][2] + 0.01,  # seat rises above bar
            details=f"seat aabb={seat}",
        )
        # Molded seat should have meaningful height from the lip
        ctx.check(
            f"seat_{i} has raised lip (height > 0.02 m)",
            seat is not None and (seat[1][2] - seat[0][2]) > 0.020,
            details=f"seat height={seat[1][2] - seat[0][2] if seat else None}",
        )

    # --- Rounded handle grips ---
    for i in range(2):
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        grip = ctx.part_element_world_aabb(beam, elem=f"grip_{i}")
        ctx.check(
            f"handle_{i} stands about 0.25 m above the beam",
            handle is not None
            and bar_box is not None
            and handle[1][2] > bar_box[1][2] + 0.18
            and handle[0][2] < bar_box[1][2],
            details=f"handle aabb={handle}",
        )
        ctx.check(
            f"grip_{i} is present at the handle top arc",
            grip is not None
            and handle is not None
            and grip[1][2] > handle[0][2] + 0.15
            and grip[0][2] < handle[1][2],
            details=f"grip aabb={grip}",
        )

    # --- Bumpers ---
    for i in range(2):
        bumper = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")
        ctx.check(
            f"bumper_{i} hangs below the beam tip",
            bumper is not None
            and bar_box is not None
            and bumper[0][2] < bar_box[0][2]
            and min(abs(bumper[0][0]), abs(bumper[1][0])) > 1.3,
            details=f"bumper aabb={bumper}",
        )

    # --- Decisive pose checks: rocking alternately lowers each end ---
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "positive rock lowers the +X end near the ground",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.40
            and down_b0[0][2] > 0.0,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_b1 is not None and up_b1[0][2] > 1.0,
            details=f"raised bumper aabb={up_b1}",
        )
    with ctx.pose({pivot: -TILT}):
        down_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "negative rock lowers the -X end near the ground",
            down_b1 is not None and 0.0 < down_b1[0][2] < 0.32,
            details=f"tilted bumper aabb={down_b1}",
        )

    return ctx.report()


object_model = build_object_model()
