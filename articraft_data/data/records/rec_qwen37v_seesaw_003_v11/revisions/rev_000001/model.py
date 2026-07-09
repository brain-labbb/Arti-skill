from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along seesaw length, Y across, Z up.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.58          # pivot axis height above ground
PLANK_L = 2.40          # plank length
PLANK_W = 0.22          # plank width
PLANK_T = 0.035         # plank thickness

SEAT_X = 0.85           # seat centre X offset from pivot
SEAT_SIZE = 0.30        # seat outer square dimension
SEAT_H = 0.060          # seat total height (base + lip)
SEAT_WALL = 0.022       # seat wall thickness (lip width)
SEAT_FLOOR = 0.012      # seat floor thickness

BACKREST_X = SEAT_SIZE / 2.0   # hinge X offset from seat centre (outboard)
BACKREST_W = 0.26       # backrest width (along X)
BACKREST_H = 0.22       # backrest height
BACKREST_T = 0.020      # backrest thickness

HANDLE_X = 0.58         # handle post X offset from pivot
HANDLE_POST_R = 0.014   # handle post radius
HANDLE_POST_H = 0.30    # handle post height above plank top
HANDLE_GRIP_R = 0.020   # grip capsule radius
HANDLE_GRIP_L = 0.12    # grip capsule cylindrical length

LEG_R = 0.025           # A-frame leg tube radius
LEG_SPREAD_Y = 0.30     # leg base Y spread from centre
FOOT_R = 0.040          # foot pad radius
CROSSBAR_R = 0.018      # crossbar tube radius

ROCK_LIMIT = 0.262      # ~15 degrees
BACKREST_LOWER = 0.0
BACKREST_UPPER = 0.35   # ~20 degrees tilt back

# Derived geometry
LEG_LEN = math.sqrt(LEG_SPREAD_Y ** 2 + PIVOT_Z ** 2)
LEG_ANGLE = math.atan2(LEG_SPREAD_Y, PIVOT_Z)
CROSSBAR_Z = 0.16
CROSSBAR_HALF_Y = LEG_SPREAD_Y * (1.0 - CROSSBAR_Z / PIVOT_Z)
CROSSBAR_LEN = 2.0 * CROSSBAR_HALF_Y
HINGE_Z = PLANK_T / 2.0 + SEAT_H             # backrest hinge Z in plank frame (at seat lip top)


# ---------------------------------------------------------------------------
# Procedural mesh builders
# ---------------------------------------------------------------------------
def _make_bucket_seat() -> object:
    """Molded bucket seat with raised lips (CadQuery solid)."""
    w = SEAT_SIZE
    seat = cq.Workplane("XY").box(w, w, SEAT_H, centered=(True, True, False))
    inner = w - 2.0 * SEAT_WALL
    cut_depth = SEAT_H - SEAT_FLOOR
    seat = (
        seat.faces(">Z").workplane()
        .rect(inner, inner)
        .cutBlind(-cut_depth)
    )
    return seat


def _make_backrest_panel() -> object:
    """Flat backrest panel (CadQuery solid, centred at origin)."""
    return (
        cq.Workplane("XY")
        .box(BACKREST_W, BACKREST_T, BACKREST_H, centered=(True, True, True))
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw_plank")

    # Materials
    model.material("forest_green", rgba=(0.13, 0.42, 0.20, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("bright_yellow", rgba=(0.95, 0.78, 0.10, 1.0))
    model.material("royal_blue", rgba=(0.15, 0.35, 0.70, 1.0))
    model.material("steel_gray", rgba=(0.50, 0.52, 0.55, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))

    # =================================================================
    # Support frame (root, fixed): A-frame round legs + pivot bracket
    # =================================================================
    frame = model.part("support_frame")

    # Two round tube A-frame legs in the YZ plane
    for i, sy in enumerate((-1.0, 1.0)):
        frame.visual(
            Cylinder(radius=LEG_R, length=LEG_LEN),
            origin=Origin(
                xyz=(0.0, sy * LEG_SPREAD_Y / 2.0, PIVOT_Z / 2.0),
                rpy=(sy * LEG_ANGLE, 0.0, 0.0),
            ),
            material="forest_green",
            name=f"leg_{i}",
        )
        # Foot pad
        frame.visual(
            Cylinder(radius=FOOT_R, length=0.012),
            origin=Origin(xyz=(0.0, sy * LEG_SPREAD_Y, 0.006)),
            material="matte_black",
            name=f"foot_pad_{i}",
        )

    # Crossbar between the legs
    frame.visual(
        Cylinder(radius=CROSSBAR_R, length=CROSSBAR_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, CROSSBAR_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="forest_green",
        name="crossbar",
    )

    # Pivot bracket housing at the apex
    frame.visual(
        Box((0.14, 0.12, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z - 0.012)),
        material="matte_black",
        name="pivot_bracket",
    )
    # Pivot axle bosses on each side
    for i, sy in enumerate((-1.0, 1.0)):
        frame.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=Origin(
                xyz=(0.0, sy * 0.069, PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="matte_black",
            name=f"axle_boss_{i}",
        )

    # =================================================================
    # Plank (rocks on pivot): board + seats + handles
    # =================================================================
    plank = model.part("plank")

    # Main plank board
    plank.visual(
        Box((PLANK_L, PLANK_W, PLANK_T)),
        origin=Origin(xyz=(0.0, 0.0, PLANK_T / 2.0)),
        material="bright_yellow",
        name="plank_board",
    )

    # Molded bucket seats with raised lips
    seat_mesh = mesh_from_cadquery(_make_bucket_seat(), "bucket_seat")
    for i, s in enumerate((1.0, -1.0)):
        plank.visual(
            seat_mesh,
            origin=Origin(xyz=(s * SEAT_X, 0.0, PLANK_T / 2.0)),
            material="royal_blue",
            name=f"seat_{i}",
        )

    # Handle posts and rounded grips at each end
    for i, s in enumerate((1.0, -1.0)):
        # Vertical post
        plank.visual(
            Cylinder(radius=HANDLE_POST_R, length=HANDLE_POST_H),
            origin=Origin(
                xyz=(s * HANDLE_X, 0.0, PLANK_T / 2.0 + HANDLE_POST_H / 2.0),
            ),
            material="steel_gray",
            name=f"handle_post_{i}",
        )
        # Horizontal rounded capsule grip (along Y)
        grip = CapsuleGeometry(radius=HANDLE_GRIP_R, length=HANDLE_GRIP_L)
        grip.rotate_x(math.pi / 2.0)  # align along Y
        grip.translate(
            s * HANDLE_X,
            0.0,
            PLANK_T / 2.0 + HANDLE_POST_H + HANDLE_GRIP_R,
        )
        plank.visual(
            mesh_from_geometry(grip, f"handle_grip_{i}"),
            material="rubber_black",
            name=f"handle_grip_{i}",
        )

    # =================================================================
    # Backrests (tilt on revolute joints)
    # =================================================================
    backrest_mesh = mesh_from_cadquery(_make_backrest_panel(), "backrest_panel")

    for i, s in enumerate((1.0, -1.0)):
        br = model.part(f"backrest_{i}")
        br.visual(
            backrest_mesh,
            origin=Origin(xyz=(0.0, 0.0, BACKREST_H / 2.0)),
            material="royal_blue",
            name=f"backrest_panel_{i}",
        )

        # Backrest tilt articulation
        hinge_x = s * (SEAT_X + BACKREST_X)
        axis = (0.0, s, 0.0)  # positive q tilts top away from centre

        model.articulation(
            f"backrest_{i}_tilt",
            ArticulationType.REVOLUTE,
            parent=plank,
            child=br,
            origin=Origin(xyz=(hinge_x, 0.0, HINGE_Z)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=2.0,
                lower=BACKREST_LOWER,
                upper=BACKREST_UPPER,
            ),
        )

    # =================================================================
    # Main plank pivot
    # =================================================================
    model.articulation(
        "plank_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
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


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _aabb_intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(
        a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol
        for i in range(3)
    )


def _aabb_cx(aabb) -> float:
    return 0.5 * (aabb[0][0] + aabb[1][0])


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    plank = object_model.get_part("plank")
    br0 = object_model.get_part("backrest_0")
    br1 = object_model.get_part("backrest_1")
    pivot = object_model.get_articulation("plank_pivot")
    br0_joint = object_model.get_articulation("backrest_0_tilt")
    br1_joint = object_model.get_articulation("backrest_1_tilt")

    # ---- Intentional overlaps ----

    # The plank board sits on the pivot bracket that captures the rocking axle.
    ctx.allow_overlap(
        plank,
        frame,
        elem_a="plank_board",
        elem_b="pivot_bracket",
        reason="The plank board rests on the pivot bracket housing as the rocking fulcrum.",
    )
    # Axle bosses protrude through the plank at the pivot bearings.
    for i in range(2):
        ctx.allow_overlap(
            plank,
            frame,
            elem_a="plank_board",
            elem_b=f"axle_boss_{i}",
            reason="The axle boss protrudes through the plank at the pivot bearing.",
        )
    # Leg apexes enter the bracket housing just below the plank bottom.
    for i in range(2):
        ctx.allow_overlap(
            plank,
            frame,
            elem_a="plank_board",
            elem_b=f"leg_{i}",
            reason="The A-frame leg apex enters the pivot bracket housing below the plank.",
        )
    ctx.expect_gap(
        plank,
        frame,
        axis="z",
        positive_elem="plank_board",
        negative_elem="pivot_bracket",
        max_penetration=0.025,
        name="plank seats on bracket without deep penetration",
    )

    # ---- Variant 11: classic plank seesaw checks ----

    # 1. Plank board spans the seesaw length
    board = ctx.part_element_world_aabb(plank, elem="plank_board")
    ctx.check(
        "plank board spans at least 2.2 m",
        board is not None and (board[1][0] - board[0][0]) >= 2.2,
        details=f"board={board}",
    )

    # 2. Two molded bucket seats at opposite ends
    seat0 = ctx.part_element_world_aabb(plank, elem="seat_0")
    seat1 = ctx.part_element_world_aabb(plank, elem="seat_1")
    ctx.check(
        "seats exist at both ends of the plank",
        seat0 is not None and seat1 is not None
        and _aabb_cx(seat0) > 0.6 and _aabb_cx(seat1) < -0.6,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # 3. Seats have raised lips (Z extent > plank thickness alone)
    ctx.check(
        "molded seats have raised lips (Z extent > 0.04 m)",
        seat0 is not None and seat1 is not None
        and (seat0[1][2] - seat0[0][2]) > 0.04
        and (seat1[1][2] - seat1[0][2]) > 0.04,
        details=f"seat0_dz={None if seat0 is None else seat0[1][2] - seat0[0][2]}, "
                f"seat1_dz={None if seat1 is None else seat1[1][2] - seat1[0][2]}",
    )

    # 4. Seats mirrored about the pivot
    ctx.check(
        "seats mirrored about the pivot",
        seat0 is not None and seat1 is not None
        and abs(_aabb_cx(seat0) + _aabb_cx(seat1)) < 0.03,
        details=f"seat0_cx={None if seat0 is None else _aabb_cx(seat0)}, "
                f"seat1_cx={None if seat1 is None else _aabb_cx(seat1)}",
    )

    # 5. Backrests exist as separate articulated parts
    br0_aabb = ctx.part_world_aabb(br0)
    br1_aabb = ctx.part_world_aabb(br1)
    ctx.check(
        "backrest parts exist at both ends",
        br0_aabb is not None and br1_aabb is not None
        and _aabb_cx(br0_aabb) > 0.6 and _aabb_cx(br1_aabb) < -0.6,
        details=f"br0={br0_aabb}, br1={br1_aabb}",
    )

    # 6. Backrest tilt joints are revolute with non-zero range
    br0_lim = br0_joint.motion_limits
    br1_lim = br1_joint.motion_limits
    ctx.check(
        "backrest_0 tilt has non-zero revolute range",
        br0_lim is not None and br0_lim.upper - br0_lim.lower > 0.1,
        details=f"br0_limits=({br0_lim.lower}, {br0_lim.upper})",
    )
    ctx.check(
        "backrest_1 tilt has non-zero revolute range",
        br1_lim is not None and br1_lim.upper - br1_lim.lower > 0.1,
        details=f"br1_limits=({br1_lim.lower}, {br1_lim.upper})",
    )

    # 7. Backrest tilt moves the backrest top outward (decisive pose)
    with ctx.pose({br0_joint: BACKREST_UPPER}):
        br0_tilted = ctx.part_world_aabb(br0)
        ctx.check(
            "backrest_0 tilts outward (top moves +X)",
            br0_aabb is not None and br0_tilted is not None
            and br0_tilted[1][0] > br0_aabb[1][0] + 0.01,
            details=f"rest={br0_aabb}, tilted={br0_tilted}",
        )
    with ctx.pose({br1_joint: BACKREST_UPPER}):
        br1_tilted = ctx.part_world_aabb(br1)
        ctx.check(
            "backrest_1 tilts outward (top moves -X)",
            br1_aabb is not None and br1_tilted is not None
            and br1_tilted[0][0] < br1_aabb[0][0] - 0.01,
            details=f"rest={br1_aabb}, tilted={br1_tilted}",
        )

    # 8. Rounded handle grips at both ends
    grip0 = ctx.part_element_world_aabb(plank, elem="handle_grip_0")
    grip1 = ctx.part_element_world_aabb(plank, elem="handle_grip_1")
    ctx.check(
        "handle grips present at both ends",
        grip0 is not None and grip1 is not None
        and _aabb_cx(grip0) > 0.3 and _aabb_cx(grip1) < -0.3,
        details=f"grip0={grip0}, grip1={grip1}",
    )
    # Grips extend in Y (across the seesaw, for two-hand grab)
    ctx.check(
        "handle grips span across the seesaw (Y extent > 0.10 m)",
        grip0 is not None and grip1 is not None
        and (grip0[1][1] - grip0[0][1]) > 0.10
        and (grip1[1][1] - grip1[0][1]) > 0.10,
        details=f"grip0_dy={None if grip0 is None else grip0[1][1] - grip0[0][1]}, "
                f"grip1_dy={None if grip1 is None else grip1[1][1] - grip1[0][1]}",
    )

    # 9. Round support legs (A-frame extends in Y)
    frame_aabb = ctx.part_world_aabb(frame)
    leg0 = ctx.part_element_world_aabb(frame, elem="leg_0")
    leg1 = ctx.part_element_world_aabb(frame, elem="leg_1")
    ctx.check(
        "round support legs present",
        leg0 is not None and leg1 is not None,
        details=f"leg0={leg0}, leg1={leg1}",
    )
    ctx.check(
        "A-frame legs spread in Y direction",
        frame_aabb is not None
        and (frame_aabb[1][1] - frame_aabb[0][1]) > 0.40,
        details=f"frame Y span={None if frame_aabb is None else frame_aabb[1][1] - frame_aabb[0][1]}",
    )

    # 10. Legs reach the ground and connect to pivot bracket
    bracket = ctx.part_element_world_aabb(frame, elem="pivot_bracket")
    ctx.check(
        "legs reach ground level",
        leg0 is not None and leg1 is not None
        and leg0[0][2] < 0.02 and leg1[0][2] < 0.02,
        details=f"leg0_min_z={None if leg0 is None else leg0[0][2]}, "
                f"leg1_min_z={None if leg1 is None else leg1[0][2]}",
    )
    ctx.check(
        "legs reach the pivot bracket at the top",
        leg0 is not None and leg1 is not None and bracket is not None
        and leg0[1][2] > bracket[0][2] and leg1[1][2] > bracket[0][2],
        details=f"leg0={leg0}, leg1={leg1}, bracket={bracket}",
    )

    # ---- Main pivot articulation checks ----

    # 11. Main pivot rocking range ~ +/- 15 degrees
    pivot_lim = pivot.motion_limits
    ctx.check(
        "main pivot rocking range about +/- 15 degrees",
        pivot_lim is not None
        and abs(pivot_lim.lower + ROCK_LIMIT) < 0.02
        and abs(pivot_lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({pivot_lim.lower}, {pivot_lim.upper})",
    )

    # 12. Overall envelope
    plank_aabb = ctx.part_world_aabb(plank)
    ctx.check(
        "overall length about 2.4 m",
        plank_aabb is not None and 2.2 <= (plank_aabb[1][0] - plank_aabb[0][0]) <= 2.7,
        details=f"plank_aabb={plank_aabb}",
    )
    ctx.check(
        "overall height about 0.9 m",
        plank_aabb is not None and frame_aabb is not None
        and 0.78 <= max(plank_aabb[1][2], frame_aabb[1][2]) <= 1.0,
        details=f"plank={plank_aabb}, frame={frame_aabb}",
    )

    # 13. Decisive pose: positive rock tilts seat_0 down, seat_1 up
    base_rest = ctx.part_world_aabb(frame)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(plank, elem="seat_0")
        seat1_up = ctx.part_element_world_aabb(plank, elem="seat_1")
        rocker_dn = ctx.part_world_aabb(plank)
        base_posed = ctx.part_world_aabb(frame)
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None and seat1_up is not None
            and seat0 is not None and seat1 is not None
            and seat0_dn[1][2] < seat0[1][2] - 0.10
            and seat1_up[0][2] > seat1[0][2] + 0.10,
            details=f"seat0_dn={seat0_dn}, seat1_up={seat1_up}",
        )
        ctx.check(
            "plank clears ground at full tilt",
            rocker_dn is not None and rocker_dn[0][2] > 0.005,
            details=f"rocker={rocker_dn}",
        )
        ctx.check(
            "support frame stays fixed while rocking",
            base_rest is not None and base_posed is not None
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )

    # 14. Handle posts connect to plank board
    post0 = ctx.part_element_world_aabb(plank, elem="handle_post_0")
    post1 = ctx.part_element_world_aabb(plank, elem="handle_post_1")
    ctx.check(
        "handle posts connect plank to grips",
        _aabb_intersects(post0, board) and _aabb_intersects(post0, grip0)
        and _aabb_intersects(post1, board) and _aabb_intersects(post1, grip1),
        details=f"post0={post0}, post1={post1}",
    )

    # 15. Seats sit on plank board
    ctx.check(
        "seats mounted on plank board",
        _aabb_intersects(seat0, board) and _aabb_intersects(seat1, board),
        details=f"seat0={seat0}, seat1={seat1}, board={board}",
    )

    return ctx.report()


object_model = build_object_model()
