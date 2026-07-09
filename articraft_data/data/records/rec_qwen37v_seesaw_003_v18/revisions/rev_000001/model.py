from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Compact backyard seesaw (~1.5 m long, ~0.55 m tall).
# World: X along the beam length, Y lateral, Z up.
# ---------------------------------------------------------------------------
BEAM_HALF = 0.75          # half-length of beam (1.5 m total)
BEAM_R = 0.025            # beam tube radius (50 mm dia)

PIVOT_Z = 0.46            # pivot axis = beam centreline height
APEX_Z = 0.34             # height where the A-frame legs converge

FRAME_SPREAD = 0.28       # lateral foot spread of the A-frame
LEG_R = 0.018             # leg tube radius (36 mm)
BRACE_R = 0.013           # cross-brace tube radius

CROSS_H = 0.18            # cross-brace height

PAD_SIDE = 0.11           # rubber pad side length
PAD_THICK = 0.018         # rubber pad thickness

SEAT_X = 0.62             # seat centre along beam (local X)
SEAT_SIZE = (0.26, 0.20, 0.014)

HANDLE_X = 0.68           # handle post position
HANDLE_H = 0.20           # handle post height above beam
BAR_W = 0.16              # handlebar crossbar width

ROCK_LIMIT = 0.262        # ~15 degrees each way


def _leg_at(h: float) -> float:
    """Lateral Y position of an A-frame leg at height *h* (linear interpolation)."""
    return FRAME_SPREAD * (1.0 - h / APEX_Z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backyard_seesaw")

    model.material("frame_green", rgba=(0.18, 0.42, 0.22, 1.0))
    model.material("beam_yellow", rgba=(0.92, 0.78, 0.12, 1.0))
    model.material("dark_gray", rgba=(0.30, 0.30, 0.32, 1.0))
    model.material("rubber_black", rgba=(0.10, 0.10, 0.10, 1.0))
    model.material("bracket_dark", rgba=(0.15, 0.15, 0.17, 1.0))
    model.material("silver_bolt", rgba=(0.72, 0.73, 0.76, 1.0))

    # ==================================================================
    # Fixed support: A-frame with triangular profile + rubber ground pads.
    # ==================================================================
    frame = model.part("support_frame")

    # --- A-frame legs (two tubes forming inverted-V in the YZ plane) ---
    for i, sy in enumerate((1.0, -1.0)):
        leg_pts = [
            (0.0, sy * FRAME_SPREAD, PAD_THICK),
            (0.0, sy * FRAME_SPREAD * 0.5, APEX_Z * 0.5),
            (0.0, 0.0, APEX_Z),
        ]
        frame.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    leg_pts,
                    radius=LEG_R,
                    samples_per_segment=6,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"leg_{i}",
            ),
            material="frame_green",
            name=f"leg_{i}",
        )

        # Rubber ground pad under each foot.
        frame.visual(
            Box((PAD_SIDE, PAD_SIDE, PAD_THICK)),
            origin=Origin(xyz=(0.0, sy * FRAME_SPREAD, PAD_THICK / 2.0)),
            material="rubber_black",
            name=f"ground_pad_{i}",
        )

    # --- Cross-brace connecting the two legs at mid-height. ---
    brace_y = _leg_at(CROSS_H)
    brace_pts = [
        (0.0, -brace_y, CROSS_H),
        (0.0, 0.0, CROSS_H),
        (0.0, brace_y, CROSS_H),
    ]
    frame.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                brace_pts,
                radius=BRACE_R,
                samples_per_segment=4,
                radial_segments=12,
                cap_ends=True,
            ),
            "cross_brace",
        ),
        material="frame_green",
        name="cross_brace",
    )

    # --- Apex plate where the legs converge. ---
    frame.visual(
        Box((0.10, 0.10, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, APEX_Z + 0.005)),
        material="bracket_dark",
        name="apex_plate",
    )

    # --- Pivot bracket: two cheek plates + bottom web. ---
    BRACKET_H = PIVOT_Z - APEX_Z + 0.04   # bracket spans from apex to above pivot
    BRACKET_CZ = APEX_Z + BRACKET_H / 2.0
    cheek_gap = 0.070  # half-gap between cheeks in Y

    for i, sy in enumerate((1.0, -1.0)):
        frame.visual(
            Box((0.08, 0.008, BRACKET_H)),
            origin=Origin(xyz=(0.0, sy * cheek_gap, BRACKET_CZ)),
            material="bracket_dark",
            name=f"bracket_cheek_{i}",
        )
        # Bearing boss on outer face of each cheek at pivot height.
        frame.visual(
            Cylinder(radius=0.028, length=0.014),
            origin=Origin(
                xyz=(0.0, sy * (cheek_gap + 0.011), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="bracket_dark",
            name=f"pivot_boss_{i}",
        )
        # Bolt heads on each boss.
        for j, ang in enumerate((0.0, 0.5, 1.0, 1.5)):
            dx = 0.016 * math.cos(ang * math.pi * 2)
            dz = 0.016 * math.sin(ang * math.pi * 2)
            frame.visual(
                Cylinder(radius=0.005, length=0.007),
                origin=Origin(
                    xyz=(dx, sy * (cheek_gap + 0.020), PIVOT_Z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_bolt",
                name=f"boss_bolt_{i}_{j}",
            )

    # Bottom web connecting the two cheeks.
    frame.visual(
        Box((0.08, 2.0 * cheek_gap, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, APEX_Z + 0.004)),
        material="bracket_dark",
        name="bracket_web",
    )

    # ==================================================================
    # Rocking beam: straight tube with seats + handles at each end.
    # Part frame sits at the pivot axis so the revolute joint is at origin.
    # ==================================================================
    beam = model.part("beam")

    # Main beam tube (straight, along local X).
    beam_pts = [
        (-BEAM_HALF, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (BEAM_HALF, 0.0, 0.0),
    ]
    beam.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                beam_pts,
                radius=BEAM_R,
                samples_per_segment=6,
                radial_segments=20,
                cap_ends=True,
            ),
            "beam_tube",
        ),
        material="beam_yellow",
        name="beam_tube",
    )

    # Pivot stub descending from beam centre into the bracket.
    beam.visual(
        Cylinder(radius=0.018, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
        material="beam_yellow",
        name="pivot_stub",
    )

    # End caps on the beam tube.
    for i, sx in enumerate((1.0, -1.0)):
        beam.visual(
            Cylinder(radius=BEAM_R + 0.002, length=0.006),
            origin=Origin(
                xyz=(sx * BEAM_HALF, 0.0, 0.0),
                rpy=(0.0, sx * math.pi / 2.0, 0.0),
            ),
            material="dark_gray",
            name=f"end_cap_{i}",
        )

    # Seat + handle assemblies, mirrored at each end.
    for i, s in enumerate((1.0, -1.0)):
        # --- Seat clamp ring around beam ---
        beam.visual(
            Cylinder(radius=BEAM_R + 0.010, length=0.055),
            origin=Origin(
                xyz=(s * SEAT_X, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="dark_gray",
            name=f"seat_clamp_{i}",
        )
        # Clamp bolts.
        for j, sy in enumerate((1.0, -1.0)):
            beam.visual(
                Cylinder(radius=0.005, length=0.010),
                origin=Origin(
                    xyz=(s * SEAT_X, sy * (BEAM_R + 0.014), 0.0),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_bolt",
                name=f"clamp_bolt_{i}_{j}",
            )

        # --- Seat support bracket (flat plate from clamp up to seat) ---
        beam.visual(
            Box((0.16, 0.040, 0.004)),
            origin=Origin(xyz=(s * SEAT_X, 0.0, BEAM_R + 0.002)),
            material="dark_gray",
            name=f"seat_bracket_{i}",
        )

        # --- Seat plate (flat panel sitting on the beam end) ---
        seat_z = BEAM_R + 0.004 + SEAT_SIZE[2] / 2.0
        beam.visual(
            Box(SEAT_SIZE),
            origin=Origin(xyz=(s * SEAT_X, 0.0, seat_z)),
            material="dark_gray",
            name=f"seat_plate_{i}",
        )

        # --- Handle post (vertical tube rising from beam) ---
        post_base = BEAM_R + 0.005
        post_top = post_base + HANDLE_H
        post_pts = [
            (s * HANDLE_X, 0.0, post_base),
            (s * HANDLE_X, 0.0, post_base + HANDLE_H * 0.5),
            (s * HANDLE_X, 0.0, post_top),
        ]
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    post_pts,
                    radius=0.012,
                    samples_per_segment=4,
                    radial_segments=12,
                    cap_ends=True,
                ),
                f"handle_post_{i}",
            ),
            material="beam_yellow",
            name=f"handle_post_{i}",
        )

        # --- Handlebar crossbar ---
        bar_pts = [
            (s * HANDLE_X, -BAR_W / 2.0, post_top),
            (s * HANDLE_X, 0.0, post_top),
            (s * HANDLE_X, BAR_W / 2.0, post_top),
        ]
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    bar_pts,
                    radius=0.010,
                    samples_per_segment=4,
                    radial_segments=10,
                    cap_ends=True,
                ),
                f"handle_bar_{i}",
            ),
            material="beam_yellow",
            name=f"handle_bar_{i}",
        )

        # Rubber grip sleeves on each end of the crossbar.
        for j, gy in enumerate((-1.0, 1.0)):
            beam.visual(
                Cylinder(radius=0.014, length=0.050),
                origin=Origin(
                    xyz=(s * HANDLE_X, gy * (BAR_W / 2.0 - 0.018), post_top),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="rubber_black",
                name=f"grip_{i}_{j}",
            )

    # ==================================================================
    # Articulation: revolute joint at the centre bracket, axis along Y.
    # ==================================================================
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=1.5, lower=-ROCK_LIMIT, upper=ROCK_LIMIT
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")

    # --- Triangular A-frame support structure ---
    leg0 = ctx.part_element_world_aabb(frame, elem="leg_0")
    leg1 = ctx.part_element_world_aabb(frame, elem="leg_1")
    ctx.check(
        "A-frame has two support legs",
        leg0 is not None and leg1 is not None,
        details=f"leg0={leg0}, leg1={leg1}",
    )
    # The legs spread apart laterally (in Y) forming the triangular profile.
    ctx.check(
        "legs spread laterally forming triangular support",
        leg0 is not None
        and leg1 is not None
        and leg0[1][1] > 0.15
        and leg1[0][1] < -0.15,
        details=f"leg0_y={None if leg0 is None else (leg0[0][1], leg0[1][1])}, "
                f"leg1_y={None if leg1 is None else (leg1[0][1], leg1[1][1])}",
    )
    # Both legs reach down to ground level.
    ctx.check(
        "legs reach ground level",
        leg0 is not None and leg1 is not None
        and leg0[0][2] < 0.05 and leg1[0][2] < 0.05,
        details=f"leg0_min_z={None if leg0 is None else leg0[0][2]}, "
                f"leg1_min_z={None if leg1 is None else leg1[0][2]}",
    )
    # Both legs reach up near the apex.
    ctx.check(
        "legs converge at the apex",
        leg0 is not None and leg1 is not None
        and leg0[1][2] > APEX_Z - 0.02
        and leg1[1][2] > APEX_Z - 0.02,
        details=f"leg0_max_z={None if leg0 is None else leg0[1][2]}, "
                f"leg1_max_z={None if leg1 is None else leg1[1][2]}",
    )

    # --- Rubber ground pads under each support leg ---
    pad0 = ctx.part_element_world_aabb(frame, elem="ground_pad_0")
    pad1 = ctx.part_element_world_aabb(frame, elem="ground_pad_1")
    ctx.check(
        "rubber ground pads exist under each leg",
        pad0 is not None and pad1 is not None,
        details=f"pad0={pad0}, pad1={pad1}",
    )
    ctx.check(
        "ground pads at ground level",
        pad0 is not None and pad1 is not None
        and pad0[0][2] < 0.02 and pad1[0][2] < 0.02,
        details=f"pad0_z={None if pad0 is None else (pad0[0][2], pad0[1][2])}, "
                f"pad1_z={None if pad1 is None else (pad1[0][2], pad1[1][2])}",
    )
    # Pads positioned at the feet of the legs (laterally spread).
    ctx.check(
        "ground pads at leg feet positions",
        pad0 is not None and pad1 is not None
        and pad0[1][1] > 0.10 and pad1[0][1] < -0.10,
        details=f"pad0_y={None if pad0 is None else (pad0[0][1], pad0[1][1])}, "
                f"pad1_y={None if pad1 is None else (pad1[0][1], pad1[1][1])}",
    )

    # --- Cross-brace connecting the two legs ---
    brace = ctx.part_element_world_aabb(frame, elem="cross_brace")
    ctx.check(
        "cross-brace connects the two legs",
        brace is not None
        and brace[0][1] < -0.05 and brace[1][1] > 0.05,
        details=f"brace={brace}",
    )

    # --- Pivot stub captured in the bracket ---
    ctx.allow_overlap(
        beam,
        frame,
        elem_a="pivot_stub",
        elem_b="apex_plate",
        reason="The pivot stub descends through the apex plate into the bracket assembly.",
    )
    ctx.allow_overlap(
        beam,
        frame,
        elem_a="pivot_stub",
        elem_b="bracket_web",
        reason="The pivot stub descends through the bracket web to form the rocking axle.",
    )
    ctx.allow_overlap(
        beam,
        frame,
        elem_a="pivot_stub",
        elem_b="leg_0",
        reason="The pivot stub descends past the leg convergence point at the apex.",
    )
    ctx.allow_overlap(
        beam,
        frame,
        elem_a="pivot_stub",
        elem_b="leg_1",
        reason="The pivot stub descends past the leg convergence point at the apex.",
    )
    ctx.expect_overlap(
        beam,
        frame,
        axes="z",
        elem_a="pivot_stub",
        elem_b="apex_plate",
        min_overlap=0.01,
        name="pivot stub inserted through apex plate",
    )
    ctx.expect_within(
        beam,
        frame,
        axes="xy",
        inner_elem="pivot_stub",
        outer_elem="apex_plate",
        margin=0.0,
        name="pivot stub centered on apex plate",
    )

    # --- Beam spans the seesaw length ---
    beam_tube = ctx.part_element_world_aabb(beam, elem="beam_tube")
    ctx.check(
        "beam tube spans the seesaw length",
        beam_tube is not None and (beam_tube[1][0] - beam_tube[0][0]) >= 1.4,
        details=f"beam_tube={beam_tube}",
    )

    # --- Compact overall envelope ---
    beam_aabb = ctx.part_world_aabb(beam)
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "compact length ~1.5 m",
        beam_aabb is not None and 1.3 <= (beam_aabb[1][0] - beam_aabb[0][0]) <= 1.7,
        details=f"beam_aabb={beam_aabb}",
    )
    ctx.check(
        "overall height ~0.55 m",
        beam_aabb is not None and frame_aabb is not None
        and 0.45 <= max(beam_aabb[1][2], frame_aabb[1][2]) <= 0.75,
        details=f"beam={beam_aabb}, frame={frame_aabb}",
    )

    # --- Seats at opposite ends of the beam ---
    seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
    ctx.check(
        "seats at opposite beam ends",
        seat0 is not None and seat1 is not None
        and seat0[0][0] > 0.3 and seat1[1][0] < -0.3,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # --- Joint limits: +/- 15 degrees ---
    lim = pivot.motion_limits
    ctx.check(
        "rocking range ~+/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Non-fixed joint check ---
    ctx.check(
        "beam pivot is a non-fixed revolute joint",
        pivot is not None
        and pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type if pivot else None}",
    )

    # --- Decisive pose: positive tilt raises one seat, lowers the other ---
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        seat1_up = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        rocker_dn = ctx.part_world_aabb(beam)
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None and seat1_up is not None
            and seat0 is not None and seat1 is not None
            and seat0_dn[1][2] < seat0[1][2] - 0.08
            and seat1_up[1][2] > seat1[1][2] + 0.08,
            details=f"seat0_dn={seat0_dn}, seat1_up={seat1_up}",
        )
        ctx.check(
            "beam clears ground at full tilt",
            rocker_dn is not None and rocker_dn[0][2] > 0.005,
            details=f"rocker={rocker_dn}",
        )

    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        rocker_up = ctx.part_world_aabb(beam)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None and seat0 is not None
            and seat0_up[0][2] > seat0[0][2] + 0.08,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "beam clears ground at opposite tilt",
            rocker_up is not None and rocker_up[0][2] > 0.005,
            details=f"rocker={rocker_up}",
        )

    return ctx.report()


object_model = build_object_model()
