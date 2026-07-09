from __future__ import annotations

import math

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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Compact backyard seesaw with triangular supports
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two triangular A-frame supports (tubular steel, ~36 mm dia), one on each
#   side of the beam (±Y), each with two legs converging to the apex.
# - A horizontal crossbar ties the two apexes together.
# - Rubber ground pads under each of the four feet.
# - The rocking beam is a 2.0 m steel bar (70 x 35 mm) painted teal-green,
#   with a pivot sleeve + gusset plate at center, wooden seats, grab handles,
#   and small rubber bumpers at each end.
# - Single revolute joint at the apex, axis (0, 1, 0), +/- 20 degrees.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.00  # 2.0 m total beam
BEAM_W = 0.070
BEAM_T = 0.035
PIVOT_Z = 0.55  # compact backyard pivot height

# Triangular support geometry
LEG_SPREAD = 0.25  # feet are 0.50m apart in X
Y_SEP = 0.115  # support frames at ±Y from center
PAD_TOP = 0.012  # top of rubber pad
TUBE_R = 0.018  # ~36 mm diameter tube

AXLE_R = 0.014
AXLE_LEN = 2.0 * Y_SEP + 0.04  # spans between supports with slight overhang

# Beam-local frame: origin at the axle center; the bar bottom sits above.
BAR_BOT = 0.040
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_X = 0.82
HANDLE_X = 0.62
BUMPER_X = 0.92
TILT = math.radians(20.0)

# Ground pad dimensions
PAD_W = 0.10
PAD_L = 0.10
PAD_T = 0.012


def _leg_points(foot_x: float, side_y: float) -> list[tuple[float, float, float]]:
    """Straight tube from foot to apex for one leg of a triangular support."""
    return [
        (foot_x, side_y, PAD_TOP),
        (foot_x * 0.5, side_y, PIVOT_Z * 0.5 + PAD_TOP * 0.5),
        (0.0, side_y, PIVOT_Z),
    ]


def _crossbrace_points(side_y: float) -> list[tuple[float, float, float]]:
    """Horizontal cross-brace tube connecting the two legs at mid-height."""
    mid_z = PIVOT_Z * 0.38 + PAD_TOP
    mid_x = LEG_SPREAD * 0.62
    return [
        (-mid_x, side_y, mid_z),
        (mid_x, side_y, mid_z),
    ]


def _bumper_geometry(x: float, index: int):
    """Small rubber bumper pad under beam tip."""
    profile: list[tuple[float, float]] = []
    n = 16
    r_out = 0.045
    r_in = 0.032
    for k in range(n + 1):
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):
        a = 2.0 * math.pi - math.pi * k / n
        profile.append((r_in * math.cos(a), r_in * math.sin(a)))
    geom = ExtrudeGeometry(profile, 0.08, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(x, 0.0, BAR_BOT + 0.002)
    return mesh_from_geometry(geom, f"bumper_{index}")


def _gusset_geometry():
    """Triangular gusset plate joining beam bar to pivot sleeve."""
    profile = [(-0.08, 0.040), (0.08, 0.040), (0.0, 0.015)]
    geom = ExtrudeGeometry(profile, 0.016, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "gusset_plate")


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab handle rod, plane across the beam (YZ)."""
    half_w = 0.030
    leg_bot = BAR_TOP - 0.008
    arc_z = 0.22
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.155),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.155))
    pts.append((x, half_w, leg_bot))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backyard_seesaw")

    # Materials
    steel_tube = model.material("painted_steel", rgba=(0.30, 0.42, 0.38, 1.0))
    teal_paint = model.material("teal_green_paint", rgba=(0.15, 0.45, 0.40, 1.0))
    rust = model.material("rust_accent", rgba=(0.45, 0.28, 0.14, 1.0))
    wood = model.material("pine_wood", rgba=(0.65, 0.50, 0.30, 1.0))
    rubber = model.material("black_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.25, 0.27, 1.0))
    bright_red = model.material("safety_red", rgba=(0.72, 0.15, 0.12, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("triangular_base")

    # Two triangular A-frame supports, one per side (±Y)
    for i, side in enumerate((1.0, -1.0)):
        sy = side * Y_SEP
        # Left leg (negative X foot)
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _leg_points(-LEG_SPREAD, sy),
                    radius=TUBE_R,
                    samples_per_segment=4,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"left_leg_{i}",
            ),
            material=steel_tube,
            name=f"left_leg_{i}",
        )
        # Right leg (positive X foot)
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _leg_points(LEG_SPREAD, sy),
                    radius=TUBE_R,
                    samples_per_segment=4,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"right_leg_{i}",
            ),
            material=steel_tube,
            name=f"right_leg_{i}",
        )
        # Cross brace at mid-height
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _crossbrace_points(sy),
                    radius=TUBE_R * 0.7,
                    samples_per_segment=4,
                    radial_segments=12,
                    cap_ends=True,
                ),
                f"crossbrace_{i}",
            ),
            material=steel_tube,
            name=f"crossbrace_{i}",
        )

    # Horizontal crossbar connecting the two A-frame supports below the apex
    crossbar_z = PIVOT_Z - 0.045
    base.visual(
        Cylinder(radius=TUBE_R, length=2.0 * Y_SEP),
        origin=Origin(xyz=(0.0, 0.0, crossbar_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_tube,
        name="apex_crossbar",
    )

    # Pivot axle bolt through the apex crossbar
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_axle",
    )
    # Axle nuts
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.020, length=0.010),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.004), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"axle_nut_{i}",
        )

    # Rubber ground pads under each of the four feet
    for i, (fx, fy) in enumerate(
        [(-LEG_SPREAD, Y_SEP), (-LEG_SPREAD, -Y_SEP),
         (LEG_SPREAD, Y_SEP), (LEG_SPREAD, -Y_SEP)]
    ):
        base.visual(
            Box((PAD_W, PAD_L, PAD_T)),
            origin=Origin(xyz=(fx, fy, PAD_T / 2.0)),
            material=rubber,
            name=f"ground_pad_{i}",
        )

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    # Pivot sleeve (bushing around the axle)
    beam.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_sleeve",
    )
    # Gusset plate
    beam.visual(_gusset_geometry(), material=teal_paint, name="gusset_plate")

    # Main beam bar
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=teal_paint,
        name="beam_bar",
    )

    # End fittings for each side
    for i, side in enumerate((1.0, -1.0)):
        # Wooden seat plate
        beam.visual(
            Box((0.26, 0.20, 0.018)),
            origin=Origin(xyz=(side * SEAT_X, 0.0, BAR_TOP + 0.009)),
            material=wood,
            name=f"seat_{i}",
        )
        # Grab handle
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.008,
                    samples_per_segment=6,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"handle_{i}",
            ),
            material=bright_red,
            name=f"handle_{i}",
        )
        # Rubber bumper under tip
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
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.0, lower=-TILT, upper=TILT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("triangular_base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")

    # --- Variant-specific: triangular supports and ground pads ---

    # Triangular base exists and has the expected structure
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "triangular base is grounded (feet near Z=0)",
        base_box is not None and -0.005 <= base_box[0][2] <= 0.020,
        details=f"base aabb={base_box}",
    )

    # Rubber ground pads exist under support legs
    pad_found = 0
    for i in range(4):
        pad_aabb = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        if pad_aabb is not None and pad_aabb[0][2] < 0.020:
            pad_found += 1
    ctx.check(
        "four rubber ground pads under support legs",
        pad_found == 4,
        details=f"pads found at ground level: {pad_found}",
    )

    # Compact size: beam about 2.0m (not 3.0m)
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    ctx.check(
        "beam is about 2.0 m long (compact backyard)",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 2.0) < 0.05,
        details=f"bar aabb={bar_box}",
    )

    # Pivot height about 0.55m (compact)
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "pivot is about 0.55 m high (compact backyard)",
        axle_box is not None and 0.45 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.62,
        details=f"axle aabb={axle_box}",
    )

    # --- Pivot joint checks ---

    # Sleeve captures the axle (intentional overlap)
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        name="pivot sleeve seated on axle",
    )
    ctx.expect_within(
        beam,
        base,
        axes="y",
        inner_elem="pivot_sleeve",
        outer_elem="pivot_axle",
        margin=0.002,
        name="pivot sleeve stays inside axle span",
    )

    # Beam bar clears the apex crossbar
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="apex_crossbar",
        min_gap=0.002,
        max_gap=0.08,
        name="beam bar clears the apex crossbar",
    )

    # Joint axis and limits
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to beam",
        abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "rocking limits are +/- 20 degrees",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Seats, handles, bumpers ---
    for i in range(2):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        bumper = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")
        ctx.check(
            f"seat_{i} sits on beam bar top",
            seat is not None
            and bar_box is not None
            and abs(seat[0][2] - bar_box[1][2]) < 0.010  # seat bottom at bar top
            and seat[1][2] > bar_box[1][2],  # seat extends above bar
            details=f"seat aabb={seat}, bar top={bar_box[1][2]}",
        )
        ctx.check(
            f"handle_{i} stands above beam",
            handle is not None
            and bar_box is not None
            and handle[1][2] > bar_box[1][2] + 0.12
            and handle[0][2] < bar_box[1][2],
            details=f"handle aabb={handle}",
        )
        ctx.check(
            f"bumper_{i} hangs below beam tip",
            bumper is not None
            and bar_box is not None
            and bumper[0][2] < bar_box[0][2]
            and min(abs(bumper[0][0]), abs(bumper[1][0])) > 0.80,
            details=f"bumper aabb={bumper}",
        )

    # --- Decisive pose checks: rocking alternately lowers each end ---
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "positive rock lowers +X end",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.20
            and down_b0[0][2] > -0.05,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises -X end",
            up_b1 is not None and up_b1[0][2] > 0.70,
            details=f"raised bumper aabb={up_b1}",
        )
    with ctx.pose({pivot: -TILT}):
        down_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "negative rock lowers -X end",
            down_b1 is not None and -0.05 < down_b1[0][2] < 0.30,
            details=f"tilted bumper aabb={down_b1}",
        )

    return ctx.report()


object_model = build_object_model()
