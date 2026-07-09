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
# - Two triangular A-frame supports (welded steel tube) with rubber ground
#   pads under each foot.
# - Rocking beam (2.0 m) with wooden seats, grab handles, gusset plate.
# - Rubber end bumpers on short prismatic joints (vertical compression).
# - Fixed safety bump stops on the beam underside, inboard of the bumpers.
# - Revolute joint at apex, axis (0, 1, 0), +/- 20 degrees.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.00   # 2.0 m total beam
BEAM_W = 0.07
BEAM_T = 0.035
PIVOT_Z = 0.55     # pivot height

# Beam bar in beam part frame (origin at axle center)
BAR_BOT = 0.04
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

# Triangular support frame
SPREAD_X = 0.38    # half-spread of feet along X
FRAME_Y = 0.14     # Y offset of each side frame
TUBE_R = 0.018     # ~36 mm diameter tube
FOOT_Z = 0.030     # tube foot center (sits on pad)

# Rubber ground pads
PAD_W = 0.08
PAD_D = 0.08
PAD_T = 0.012

# Seats and handles
SEAT_X = 0.78
HANDLE_X = 0.58

# Rubber end bumpers (prismatic compression)
BUMPER_X = 0.92
BUMPER_W = 0.08
BUMPER_D = 0.10
BUMPER_H = 0.05
BUMPER_TRAVEL = 0.030

# Safety bump stops (fixed, on beam underside, inboard of bumpers)
STOP_X = 0.85
STOP_W = 0.04
STOP_D = 0.06
STOP_H = 0.035

# Pivot axle
AXLE_R = 0.014
AXLE_LEN = 0.20

# Joint limits
TILT = math.radians(20.0)


def _leg_points(foot_x: float, side_y: float) -> list[tuple[float, float, float]]:
    """One leg of the triangular support: foot to apex."""
    return [
        (foot_x, side_y, FOOT_Z),
        (foot_x * 0.5, side_y, (FOOT_Z + PIVOT_Z) * 0.5),
        (0.0, side_y, PIVOT_Z),
    ]


def _base_tube_points(foot_x: float) -> list[tuple[float, float, float]]:
    """Horizontal base tube connecting both side frames at one foot position."""
    return [
        (foot_x, -FRAME_Y, FOOT_Z),
        (foot_x, FRAME_Y, FOOT_Z),
    ]


def _apex_cross_points() -> list[tuple[float, float, float]]:
    """Cross tube at the apex connecting both side frames."""
    return [
        (0.0, -FRAME_Y, PIVOT_Z),
        (0.0, FRAME_Y, PIVOT_Z),
    ]


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline, plane across the beam (YZ)."""
    half_w = 0.030
    leg_bot = BAR_TOP - 0.008
    arc_z = BAR_TOP + 0.20
    mid_z = arc_z - 0.06
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, mid_z),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, mid_z))
    pts.append((x, half_w, leg_bot))
    return pts


def _gusset_geometry():
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    # Bottom vertex clears the apex cross tube top (PIVOT_Z + TUBE_R = 0.568)
    profile = [(-0.08, BAR_BOT), (0.08, BAR_BOT), (0.0, 0.022)]
    geom = ExtrudeGeometry(profile, 0.016, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "gusset_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backyard_seesaw")

    # -- materials --
    green_paint = model.material("green_paint", rgba=(0.18, 0.42, 0.22, 1.0))
    blue_paint = model.material("blue_paint", rgba=(0.16, 0.32, 0.58, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.22, 0.25, 1.0))
    zinc = model.material("zinc_plated", rgba=(0.68, 0.68, 0.62, 1.0))
    wood = model.material("sealed_wood", rgba=(0.58, 0.42, 0.25, 1.0))
    rubber = model.material("black_rubber", rgba=(0.09, 0.09, 0.09, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.85, 0.38, 0.08, 1.0))

    # --------------------------------------------------------------- frame ---
    frame = model.part("frame")

    # Triangular support legs: 4 legs (2 per side frame)
    tube_idx = 0
    for side_y in (FRAME_Y, -FRAME_Y):
        for foot_x in (SPREAD_X, -SPREAD_X):
            frame.visual(
                mesh_from_geometry(
                    tube_from_spline_points(
                        _leg_points(foot_x, side_y),
                        radius=TUBE_R,
                        samples_per_segment=8,
                        radial_segments=16,
                        cap_ends=True,
                    ),
                    f"support_leg_{tube_idx}",
                ),
                material=green_paint,
                name=f"support_leg_{tube_idx}",
            )
            tube_idx += 1

    # Base cross tubes (connecting side frames at each foot X position)
    for i, foot_x in enumerate((SPREAD_X, -SPREAD_X)):
        frame.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _base_tube_points(foot_x),
                    radius=TUBE_R,
                    samples_per_segment=4,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"base_cross_{i}",
            ),
            material=green_paint,
            name=f"base_cross_{i}",
        )

    # Apex cross tube
    frame.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _apex_cross_points(),
                radius=TUBE_R,
                samples_per_segment=4,
                radial_segments=16,
                cap_ends=True,
            ),
            "apex_cross",
        ),
        material=green_paint,
        name="apex_cross",
    )

    # Pivot axle bolt through the apex
    frame.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_axle",
    )
    # Axle nuts
    for i, side in enumerate((1.0, -1.0)):
        frame.visual(
            Cylinder(radius=0.020, length=0.010),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.004), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"axle_nut_{i}",
        )

    # Rubber ground pads (one under each foot)
    pad_idx = 0
    for side_y in (FRAME_Y, -FRAME_Y):
        for foot_x in (SPREAD_X, -SPREAD_X):
            frame.visual(
                Box((PAD_W, PAD_D, PAD_T)),
                origin=Origin(xyz=(foot_x, side_y, PAD_T / 2.0)),
                material=rubber,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # ---------------------------------------------------------------- beam ---
    beam = model.part("beam")

    # Pivot sleeve (bushing around the axle)
    beam.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_sleeve",
    )

    # Gusset plate
    beam.visual(_gusset_geometry(), material=blue_paint, name="gusset_plate")

    # Beam bar
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=blue_paint,
        name="beam_bar",
    )

    # Per-end fittings: seats, handles, safety bump stops
    for i, side in enumerate((1.0, -1.0)):
        # Wooden seat plate
        beam.visual(
            Box((0.24, 0.18, 0.016)),
            origin=Origin(xyz=(side * SEAT_X, 0.0, BAR_TOP + 0.008)),
            material=wood,
            name=f"seat_{i}",
        )

        # Grab handle
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.008,
                    samples_per_segment=8,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"handle_tube_{i}",
            ),
            material=zinc,
            name=f"handle_{i}",
        )

        # Safety bump stop (fixed rubber block on beam underside, inboard of bumper)
        beam.visual(
            Box((STOP_W, STOP_D, STOP_H)),
            origin=Origin(xyz=(side * STOP_X, 0.0, BAR_BOT - STOP_H / 2.0)),
            material=safety_orange,
            name=f"bump_stop_{i}",
        )

    # ----------------------------------------------- bumper parts (prismatic) ---
    for i, side in enumerate((1.0, -1.0)):
        bumper = model.part(f"end_bumper_{i}")
        bumper.visual(
            Box((BUMPER_W, BUMPER_D, BUMPER_H)),
            origin=Origin(xyz=(0.0, 0.0, -BUMPER_H / 2.0)),
            material=rubber,
            name=f"bumper_block_{i}",
        )

    # ---------------------------------------------------------- articulations ---

    # Main pivot (revolute)
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    # Bumper compression joints (prismatic, vertical)
    for i, side in enumerate((1.0, -1.0)):
        model.articulation(
            f"bumper_slide_{i}",
            ArticulationType.PRISMATIC,
            parent=beam,
            child=f"end_bumper_{i}",
            origin=Origin(xyz=(side * BUMPER_X, 0.0, BAR_BOT)),
            axis=(0.0, 0.0, 1.0),  # positive q = upward compression
            motion_limits=MotionLimits(
                effort=50.0, velocity=0.5, lower=0.0, upper=BUMPER_TRAVEL
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    beam = object_model.get_part("beam")
    bumper_0 = object_model.get_part("end_bumper_0")
    bumper_1 = object_model.get_part("end_bumper_1")
    pivot = object_model.get_articulation("beam_pivot")
    slide_0 = object_model.get_articulation("bumper_slide_0")
    slide_1 = object_model.get_articulation("bumper_slide_1")

    # --- Pivot sleeve/axle overlap (bushing) ---
    ctx.allow_overlap(
        beam,
        frame,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    ctx.allow_overlap(
        beam,
        frame,
        elem_a="pivot_sleeve",
        elem_b="apex_cross",
        reason="Pivot sleeve wraps around the apex cross tube at the saddle; the cross tube structurally carries the pivot.",
    )
    ctx.expect_contact(
        beam,
        frame,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        name="pivot sleeve is seated on the axle bolt",
    )
    ctx.expect_within(
        beam,
        frame,
        axes="y",
        inner_elem="pivot_sleeve",
        outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve stays inside the axle span",
    )

    # --- Triangular support structure ---
    for i in range(4):
        leg_aabb = ctx.part_element_world_aabb(frame, elem=f"support_leg_{i}")
        ctx.check(
            f"support_leg_{i} exists and spans from ground to near apex",
            leg_aabb is not None
            and leg_aabb[0][2] < 0.06
            and leg_aabb[1][2] > PIVOT_Z - 0.05,
            details=f"leg aabb={leg_aabb}",
        )

    # Verify triangular arrangement: legs from different sides cross in X
    leg0 = ctx.part_element_world_aabb(frame, elem="support_leg_0")
    leg2 = ctx.part_element_world_aabb(frame, elem="support_leg_2")
    ctx.check(
        "triangular supports form opposing A-frames",
        leg0 is not None
        and leg2 is not None
        and leg0[0][1] != leg2[0][1],  # different Y positions
        details=f"leg0_y={leg0[0][1] if leg0 else None}, leg2_y={leg2[0][1] if leg2 else None}",
    )

    # --- Rubber ground pads ---
    for i in range(4):
        pad_aabb = ctx.part_element_world_aabb(frame, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground_pad_{i} rests on the ground",
            pad_aabb is not None
            and -0.005 <= pad_aabb[0][2] <= 0.005
            and pad_aabb[1][2] - pad_aabb[0][2] < 0.020,
            details=f"pad aabb={pad_aabb}",
        )

    # Ground pads contact support legs
    ctx.expect_gap(
        frame,
        frame,
        axis="z",
        positive_elem="support_leg_0",
        negative_elem="ground_pad_0",
        min_gap=-0.002,
        max_gap=0.015,
        name="support leg sits on ground pad",
    )

    # --- Revolute joint configuration ---
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

    # --- Prismatic bumper compression joints ---
    for slide, name_suffix in ((slide_0, "0"), (slide_1, "1")):
        s_ax = slide.axis
        ctx.check(
            f"bumper_slide_{name_suffix} axis is vertical (Z)",
            abs(s_ax[0]) < 1e-9 and abs(s_ax[1]) < 1e-9 and abs(s_ax[2] - 1.0) < 1e-9,
            details=f"axis={s_ax}",
        )
        s_lim = slide.motion_limits
        ctx.check(
            f"bumper_slide_{name_suffix} has short compression travel",
            s_lim is not None
            and s_lim.lower is not None
            and s_lim.upper is not None
            and abs(s_lim.lower) < 1e-6
            and 0.020 <= s_lim.upper <= 0.040,
            details=f"limits=({s_lim.lower}, {s_lim.upper})",
        )
        ctx.check(
            f"bumper_slide_{name_suffix} is PRISMATIC",
            slide.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={slide.articulation_type}",
        )

    # --- Safety bump stops ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    for i in range(2):
        stop_aabb = ctx.part_element_world_aabb(beam, elem=f"bump_stop_{i}")
        ctx.check(
            f"bump_stop_{i} hangs below the beam bar",
            stop_aabb is not None
            and bar_box is not None
            and stop_aabb[0][2] < bar_box[0][2],
            details=f"stop aabb={stop_aabb}",
        )

    # --- Beam scale ---
    ctx.check(
        "beam is about 2.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 2.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )

    # Frame feet on ground
    frame_box = ctx.part_world_aabb(frame)
    ctx.check(
        "frame rests on the ground via pads",
        frame_box is not None and -0.01 <= frame_box[0][2] <= 0.01,
        details=f"frame aabb={frame_box}",
    )

    # Pivot height
    axle_box = ctx.part_element_world_aabb(frame, elem="pivot_axle")
    ctx.check(
        "pivot axle sits about 0.55 m high",
        axle_box is not None and 0.50 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.60,
        details=f"axle aabb={axle_box}",
    )

    # --- Per-end fittings ---
    for i in range(2):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        ctx.check(
            f"seat_{i} is on the beam bar top",
            seat is not None
            and bar_box is not None
            and seat[0][2] > bar_box[1][2] - 0.005
            and seat[1][2] > bar_box[1][2],
            details=f"seat aabb={seat}",
        )
        ctx.check(
            f"handle_{i} stands above the beam",
            handle is not None
            and bar_box is not None
            and handle[1][2] > bar_box[1][2] + 0.14
            and handle[0][2] < bar_box[1][2],
            details=f"handle aabb={handle}",
        )

    # --- Bumper position checks ---
    for i, bumper_part in enumerate((bumper_0, bumper_1)):
        bumper_aabb = ctx.part_element_world_aabb(bumper_part, elem=f"bumper_block_{i}")
        ctx.check(
            f"bumper_block_{i} hangs below beam near the tip",
            bumper_aabb is not None
            and bar_box is not None
            and bumper_aabb[0][2] < bar_box[0][2]
            and min(abs(bumper_aabb[0][0]), abs(bumper_aabb[1][0])) > 0.80,
            details=f"bumper aabb={bumper_aabb}",
        )

    # --- Decisive pose: rocking ---
    # Use the beam bar to track rocking (bumpers are separate child parts).
    rest_bar = ctx.part_element_world_aabb(beam, elem="beam_bar")
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")
    with ctx.pose({pivot: TILT}):
        down_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
        up_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")
        ctx.check(
            "positive rock lowers the +X end",
            rest_seat0 is not None
            and down_seat0 is not None
            and down_seat0[0][2] < rest_seat0[0][2] - 0.20,
            details=f"rest={rest_seat0}, tilted={down_seat0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_seat1 is not None
            and rest_seat1 is not None
            and up_seat1[0][2] > rest_seat1[0][2] + 0.20,
            details=f"raised seat aabb={up_seat1}",
        )

    with ctx.pose({pivot: -TILT}):
        down_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")
        ctx.check(
            "negative rock lowers the -X end",
            down_seat1 is not None
            and rest_seat1 is not None
            and down_seat1[0][2] < rest_seat1[0][2] - 0.20,
            details=f"tilted seat aabb={down_seat1}",
        )

    # --- Decisive pose: bumper compression ---
    rest_z0 = ctx.part_element_world_aabb("end_bumper_0", elem="bumper_block_0")
    with ctx.pose({slide_0: BUMPER_TRAVEL}):
        compressed_z0 = ctx.part_element_world_aabb("end_bumper_0", elem="bumper_block_0")
        ctx.check(
            "bumper compression raises the bumper block (positive Z axis)",
            rest_z0 is not None
            and compressed_z0 is not None
            and compressed_z0[0][2] > rest_z0[0][2] + 0.020,
            details=f"rest={rest_z0}, compressed={compressed_z0}",
        )

    return ctx.report()


object_model = build_object_model()
