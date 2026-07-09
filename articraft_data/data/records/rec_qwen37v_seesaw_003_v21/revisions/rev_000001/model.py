from __future__ import annotations

import math

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

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along the seesaw length, Z up.
# ---------------------------------------------------------------------------
PLANK_LENGTH = 2.40
PLANK_WIDTH = 0.22
PLANK_THICK = 0.040
PIVOT_Z = 0.58                # world height of the rocking axis
ROCK_LIMIT = 0.262            # ~15 degrees each way

# Support frame
LEG_RADIUS = 0.028
LEG_SPREAD_Y = 0.44           # half-spread of legs at ground level
FOOT_RADIUS = 0.048
FOOT_THICK = 0.014
CROSSBAR_RADIUS = 0.018

BRACKET_W = 0.14
BRACKET_D = 0.26              # wider than plank to form side cheeks
BRACKET_H = 0.14
BRACKET_CZ = PIVOT_Z - 0.02   # bracket center, top extends past pivot

# Seats
SEAT_X = 0.92                 # seat center distance from plank center
SEAT_W = 0.28                 # seat width (along plank length)
SEAT_D = 0.26                 # seat depth (across plank width)
SEAT_TOTAL_H = 0.055          # total seat height (base + lip)
SEAT_BASE_T = 0.020           # base plate thickness below the dish
SEAT_LIP_DEPTH = 0.035        # dish depth cut from top
SEAT_LIP_INSET = 0.020        # inset from outer edge for the dish
SEAT_FILLET = 0.025           # outer corner fillet

# Handlebars (past the seat outer edge: SEAT_X + SEAT_W/2 = 1.06)
HANDLE_X = 1.10               # handlebar position along plank
HANDLE_POST_R = 0.014
HANDLE_POST_H = 0.28
HANDLE_GRIP_R = 0.011
HANDLE_GRIP_W = 0.17
HANDLE_CAP_R = 0.017
HANDLE_CAP_T = 0.008
HANDLE_PIVOT_LIMIT = 0.175    # ~10 degrees


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw")

    model.material("red_plank", rgba=(0.82, 0.15, 0.08, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.72, 0.73, 0.75, 1.0))
    model.material("dark_gray", rgba=(0.30, 0.32, 0.34, 1.0))
    model.material("green_seat", rgba=(0.15, 0.55, 0.25, 1.0))
    model.material("rubber_grip", rgba=(0.12, 0.12, 0.13, 1.0))

    # ===================================================================
    # Fixed base: A-frame support with round legs + pivot bracket
    # ===================================================================
    support = model.part("support_frame")

    # Two round legs forming an inverted-V (A-frame), viewed from end.
    # Each leg runs from a ground foot up to the bracket area.
    leg_top_z = BRACKET_CZ - BRACKET_H * 0.25
    for i, sy in enumerate((1.0, -1.0)):
        dz = leg_top_z
        dy = LEG_SPREAD_Y
        leg_len = math.sqrt(dy * dy + dz * dz)
        # Rotation around X to tilt the leg inward
        leg_angle = math.atan2(dy, dz)

        mid_y = sy * dy / 2.0
        mid_z = dz / 2.0

        support.visual(
            Cylinder(radius=LEG_RADIUS, length=leg_len),
            origin=Origin(
                xyz=(0.0, mid_y, mid_z),
                rpy=(sy * leg_angle, 0.0, 0.0),
            ),
            material="light_gray",
            name=f"leg_{i}",
        )
        # Flat rubber foot disk
        support.visual(
            Cylinder(radius=FOOT_RADIUS, length=FOOT_THICK),
            origin=Origin(xyz=(0.0, sy * LEG_SPREAD_Y, FOOT_THICK / 2.0)),
            material="matte_black",
            name=f"foot_{i}",
        )

    # Crossbar connecting the legs partway up
    crossbar_z = leg_top_z * 0.45
    crossbar_half_y = LEG_SPREAD_Y * (1.0 - crossbar_z / leg_top_z)
    crossbar_len = 2.0 * crossbar_half_y

    support.visual(
        Cylinder(radius=CROSSBAR_RADIUS, length=crossbar_len),
        origin=Origin(
            xyz=(0.0, 0.0, crossbar_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="light_gray",
        name="crossbar",
    )

    # Pivot bracket at top of the A-frame
    support.visual(
        Box((BRACKET_W, BRACKET_D, BRACKET_H)),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ)),
        material="matte_black",
        name="pivot_bracket",
    )

    # Pivot pin through the bracket (horizontal, along Y)
    support.visual(
        Cylinder(radius=0.016, length=BRACKET_D + 0.04),
        origin=Origin(
            xyz=(0.0, 0.0, PIVOT_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="dark_gray",
        name="pivot_pin",
    )

    # Bolt heads on bracket side cheeks (below the plank to avoid overlap)
    bolt_z_base = BRACKET_CZ - 0.04
    for i, sy in enumerate((1.0, -1.0)):
        for j, ang in enumerate((0.3, 1.0, 1.7)):
            dx = 0.030 * math.cos(ang * math.pi)
            dz_off = 0.028 * math.sin(ang * math.pi)
            support.visual(
                Cylinder(radius=0.008, length=0.012),
                origin=Origin(
                    xyz=(dx, sy * (BRACKET_D / 2.0 + 0.006), bolt_z_base + dz_off),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="dark_gray",
                name=f"bracket_bolt_{i}_{j}",
            )

    # ===================================================================
    # Plank: flat rectangular board that rocks on the pivot
    # ===================================================================
    plank = model.part("plank")

    # Plank part frame sits at the pivot point; board extends along X.
    plank_cq = (
        cq.Workplane("XY")
        .box(PLANK_LENGTH, PLANK_WIDTH, PLANK_THICK)
        .edges("|Z")
        .fillet(0.008)
    )
    plank.visual(
        mesh_from_cadquery(plank_cq, "plank_board"),
        material="red_plank",
        name="plank_board",
    )

    # Molded seats with raised lips, built as CadQuery solids.
    # Each seat is a rounded box with a rectangular depression cut from
    # the top, leaving raised lip walls all around.
    for i, sx in enumerate((1.0, -1.0)):
        seat_cq = (
            cq.Workplane("XY")
            .box(SEAT_W, SEAT_D, SEAT_TOTAL_H)
            .edges("|Z")
            .fillet(SEAT_FILLET)
            .faces(">Z")
            .workplane()
            .rect(SEAT_W - 2 * SEAT_LIP_INSET, SEAT_D - 2 * SEAT_LIP_INSET)
            .cutBlind(-SEAT_LIP_DEPTH)
        )

        # Seat sits on top of the plank
        seat_z = PLANK_THICK / 2.0 + SEAT_TOTAL_H / 2.0
        plank.visual(
            mesh_from_cadquery(seat_cq, f"molded_seat_{i}"),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, seat_z)),
            material="green_seat",
            name=f"molded_seat_{i}",
        )

    # ===================================================================
    # Handlebars: separate parts that pivot on the plank
    # ===================================================================
    for i, sx in enumerate((1.0, -1.0)):
        hb = model.part(f"handlebar_{i}")

        # Vertical post rising from the plank
        hb.visual(
            Cylinder(radius=HANDLE_POST_R, length=HANDLE_POST_H),
            origin=Origin(xyz=(0.0, 0.0, HANDLE_POST_H / 2.0)),
            material="dark_gray",
            name=f"post_{i}",
        )

        # Horizontal grip bar at the top of the post
        hb.visual(
            Cylinder(radius=HANDLE_GRIP_R, length=HANDLE_GRIP_W),
            origin=Origin(
                xyz=(0.0, 0.0, HANDLE_POST_H),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="dark_gray",
            name=f"grip_bar_{i}",
        )

        # Rubber end caps on the grip bar
        for j, gy in enumerate((1.0, -1.0)):
            hb.visual(
                Cylinder(radius=HANDLE_CAP_R, length=HANDLE_CAP_T),
                origin=Origin(
                    xyz=(0.0, gy * (HANDLE_GRIP_W / 2.0 + HANDLE_CAP_T / 2.0), HANDLE_POST_H),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="rubber_grip",
                name=f"grip_cap_{i}_{j}",
            )

        # Small mounting flange at the base of the post
        hb.visual(
            Cylinder(radius=0.025, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material="matte_black",
            name=f"mount_flange_{i}",
        )

        # Handlebar revolute joint: pivots forward/backward (around X axis)
        # The joint origin is on the plank top surface at the handlebar position
        model.articulation(
            f"handlebar_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=plank,
            child=hb,
            origin=Origin(xyz=(sx * HANDLE_X, 0.0, PLANK_THICK / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=2.0,
                lower=-HANDLE_PIVOT_LIMIT,
                upper=HANDLE_PIVOT_LIMIT,
            ),
        )

    # ===================================================================
    # Main plank pivot: horizontal axis perpendicular to seesaw length
    # ===================================================================
    model.articulation(
        "plank_pivot",
        ArticulationType.REVOLUTE,
        parent=support,
        child=plank,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=1.5,
            lower=-ROCK_LIMIT,
            upper=ROCK_LIMIT,
        ),
    )

    return model


def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    plank = object_model.get_part("plank")
    pivot = object_model.get_articulation("plank_pivot")
    hb0 = object_model.get_part("handlebar_0")
    hb1 = object_model.get_part("handlebar_1")
    hb0_pivot = object_model.get_articulation("handlebar_0_pivot")
    hb1_pivot = object_model.get_articulation("handlebar_1_pivot")

    # --- Classic plank beam (not curved) ---
    board = ctx.part_element_world_aabb(plank, elem="plank_board")
    ctx.check(
        "plank board spans the seesaw length",
        board is not None and (board[1][0] - board[0][0]) >= 2.2,
        details=f"board={board}",
    )
    ctx.check(
        "plank board is flat (minimal z variation)",
        board is not None and (board[1][2] - board[0][2]) < 0.06,
        details=f"board z-range={None if board is None else board[1][2] - board[0][2]}",
    )

    # --- Round support legs ---
    leg0 = ctx.part_element_world_aabb(support, elem="leg_0")
    leg1 = ctx.part_element_world_aabb(support, elem="leg_1")
    ctx.check(
        "two round support legs exist",
        leg0 is not None and leg1 is not None,
        details=f"leg0={leg0}, leg1={leg1}",
    )
    # Legs spread apart at the ground
    ctx.check(
        "support legs spread apart at ground level",
        leg0 is not None
        and leg1 is not None
        and (leg0[1][1] - leg0[0][1]) > 0.02
        and (leg1[1][1] - leg1[0][1]) > 0.02,
        details=f"leg0_y={None if leg0 is None else leg0[1][1] - leg0[0][1]}, leg1_y={None if leg1 is None else leg1[1][1] - leg1[0][1]}",
    )

    # --- Molded seats with raised lips ---
    seat0 = ctx.part_element_world_aabb(plank, elem="molded_seat_0")
    seat1 = ctx.part_element_world_aabb(plank, elem="molded_seat_1")
    ctx.check(
        "two molded seats exist on the plank",
        seat0 is not None and seat1 is not None,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    # Seats have raised lips: total height should exceed the base thickness
    ctx.check(
        "seats have raised lips (taller than flat plate)",
        seat0 is not None
        and seat1 is not None
        and (seat0[1][2] - seat0[0][2]) >= 0.045
        and (seat1[1][2] - seat1[0][2]) >= 0.045,
        details=f"seat0_h={None if seat0 is None else seat0[1][2] - seat0[0][2]}, seat1_h={None if seat1 is None else seat1[1][2] - seat1[0][2]}",
    )
    # Seats are at opposite ends
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seats at opposite ends of the plank",
        seat0 is not None
        and seat1 is not None
        and _cx(seat0) > 0.7
        and _cx(seat1) < -0.7,
        details=f"seat0_cx={None if seat0 is None else _cx(seat0)}, seat1_cx={None if seat1 is None else _cx(seat1)}",
    )
    # Seats sit on top of the plank
    ctx.check(
        "seats mounted on top of the plank",
        seat0 is not None
        and seat1 is not None
        and board is not None
        and seat0[0][2] >= board[1][2] - 0.005
        and seat1[0][2] >= board[1][2] - 0.005,
        details=f"seat0_min_z={None if seat0 is None else seat0[0][2]}, board_top={None if board is None else board[1][2]}",
    )

    # --- Handlebar pivot joints (non-fixed articulations) ---
    hb0_lim = hb0_pivot.motion_limits
    hb1_lim = hb1_pivot.motion_limits
    ctx.check(
        "handlebar_0 has a revolute pivot with nonzero range",
        hb0_lim is not None
        and hb0_lim.lower is not None
        and hb0_lim.upper is not None
        and hb0_lim.upper > hb0_lim.lower,
        details=f"hb0_limits=({None if hb0_lim is None else hb0_lim.lower}, {None if hb0_lim is None else hb0_lim.upper})",
    )
    ctx.check(
        "handlebar_1 has a revolute pivot with nonzero range",
        hb1_lim is not None
        and hb1_lim.lower is not None
        and hb1_lim.upper is not None
        and hb1_lim.upper > hb1_lim.lower,
        details=f"hb1_limits=({None if hb1_lim is None else hb1_lim.lower}, {None if hb1_lim is None else hb1_lim.upper})",
    )

    # Handlebar pivot limits are about ±10 degrees
    ctx.check(
        "handlebar pivot range is about ±10 degrees",
        hb0_lim is not None
        and abs(hb0_lim.lower + HANDLE_PIVOT_LIMIT) < 0.02
        and abs(hb0_lim.upper - HANDLE_PIVOT_LIMIT) < 0.02,
        details=f"hb0_limits=({hb0_lim.lower}, {hb0_lim.upper})",
    )

    # --- Handlebar pose checks: grips above the plank ---
    grip0 = ctx.part_element_world_aabb(hb0, elem="grip_bar_0")
    grip1 = ctx.part_element_world_aabb(hb1, elem="grip_bar_1")
    ctx.check(
        "handlebar grips are above the plank",
        grip0 is not None
        and grip1 is not None
        and board is not None
        and grip0[0][2] > board[1][2]
        and grip1[0][2] > board[1][2],
        details=f"grip0={grip0}, grip1={grip1}",
    )

    # --- Overall envelope ---
    pa = ctx.part_world_aabb(plank)
    sa = ctx.part_world_aabb(support)
    ha0 = ctx.part_world_aabb(hb0)
    ha1 = ctx.part_world_aabb(hb1)
    ctx.check(
        "overall length about 2.4–2.6 m",
        pa is not None and 2.2 <= (pa[1][0] - pa[0][0]) <= 2.8,
        details=f"plank aabb={pa}",
    )
    # Height includes handlebar grips above the plank
    max_z = 0.0
    for a in (pa, sa, ha0, ha1):
        if a is not None:
            max_z = max(max_z, a[1][2])
    ctx.check(
        "overall height about 0.8–1.0 m",
        0.75 <= max_z <= 1.10,
        details=f"max_z={max_z}, plank={pa}, support={sa}, hb0={ha0}, hb1={ha1}",
    )

    # Plank passes through the pivot bracket (intentional captured fit)
    ctx.allow_overlap(
        plank,
        support,
        elem_a="plank_board",
        elem_b="pivot_bracket",
        reason="The plank board passes through the pivot bracket that captures the rocking axle.",
    )
    ctx.allow_overlap(
        plank,
        support,
        elem_a="plank_board",
        elem_b="pivot_pin",
        reason="The pivot pin passes through the plank center to form the rocking axle.",
    )
    ctx.expect_overlap(
        plank,
        support,
        axes="xy",
        elem_a="plank_board",
        elem_b="pivot_bracket",
        min_overlap=0.05,
        name="plank centered in bracket on XY",
    )

    # --- Plank pivot limits ---
    plim = pivot.motion_limits
    ctx.check(
        "plank rocking range about ±15 degrees",
        plim is not None
        and abs(plim.lower + ROCK_LIMIT) < 0.02
        and abs(plim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({plim.lower}, {plim.upper})",
    )

    # --- Decisive pose: plank rocks, seats swap height ---
    base_rest = ctx.part_world_aabb(support)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(plank, elem="molded_seat_0")
        seat1_up = ctx.part_element_world_aabb(plank, elem="molded_seat_1")
        plank_posed = ctx.part_world_aabb(plank)
        base_posed = ctx.part_world_aabb(support)
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None
            and seat1_up is not None
            and seat0 is not None
            and seat1 is not None
            and seat0_dn[1][2] < seat0[1][2] - 0.10
            and seat1_up[0][2] > seat1[0][2] + 0.10,
            details=f"seat0_dn={seat0_dn}, seat1_up={seat1_up}",
        )
        ctx.check(
            "plank clears the ground at full tilt",
            plank_posed is not None and plank_posed[0][2] > 0.005,
            details=f"plank={plank_posed}",
        )
        ctx.check(
            "support frame stays fixed while rocking",
            base_rest is not None
            and base_posed is not None
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )

    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(plank, elem="molded_seat_0")
        plank_up = ctx.part_world_aabb(plank)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None
            and seat0 is not None
            and seat0_up[0][2] > seat0[0][2] + 0.10,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "plank clears ground at opposite tilt",
            plank_up is not None and plank_up[0][2] > 0.005,
            details=f"plank={plank_up}",
        )

    # --- Handlebar pivot pose check ---
    with ctx.pose({hb0_pivot: HANDLE_PIVOT_LIMIT}):
        grip0_posed = ctx.part_element_world_aabb(hb0, elem="grip_bar_0")
        ctx.check(
            "handlebar_0 grip moves when pivoted",
            grip0 is not None
            and grip0_posed is not None
            and abs(grip0_posed[0][1] - grip0[0][1]) > 0.005
            or abs(grip0_posed[1][2] - grip0[1][2]) > 0.005,
            details=f"rest={grip0}, posed={grip0_posed}",
        )

    return ctx.report()


object_model = build_object_model()
