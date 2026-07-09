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
    rounded_rect_profile,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along the seesaw length, Z up.
# ---------------------------------------------------------------------------
PLANK_L = 2.40          # plank beam length
PLANK_W = 0.20          # plank beam width
PLANK_T = 0.040         # plank beam thickness
PIVOT_Z = 0.55          # world height of the pivot axis

LEG_R = 0.032           # support leg tube radius
LEG_SPREAD = 0.38       # half-spread of legs at ground level (Y)
LEG_TOP_Y = 0.12        # leg attachment Y offset at bracket cheek outer
BRACKET_BOTTOM_Z = PIVOT_Z - 0.045  # bracket bottom surface
LEG_DY = LEG_SPREAD - LEG_TOP_Y  # lateral run of each leg
LEG_DZ = BRACKET_BOTTOM_Z         # vertical run of each leg (ground to bracket bottom)
LEG_LEN = math.sqrt(LEG_DY ** 2 + LEG_DZ ** 2)
LEG_ANGLE = math.atan2(LEG_DY, LEG_DZ)  # tilt from vertical
LEG_MID_Y = (LEG_SPREAD + LEG_TOP_Y) / 2.0  # leg center Y magnitude
LEG_MID_Z = LEG_DZ / 2.0                       # leg center Z

CROSSBAR_R = 0.018      # cross-brace radius
CROSS_Z_FRAC = 0.30     # crossbar height as fraction of PIVOT_Z

FOOT_R = 0.055          # foot plate radius
FOOT_T = 0.008          # foot plate thickness

BRACKET_W = 0.26        # bracket width along X (cheeks + gap)
BRACKET_D = 0.14        # bracket depth along Y
BRACKET_H = 0.09        # bracket height along Z
BRACKET_GAP = 0.22      # gap between cheeks (slightly > PLANK_W)
BRACKET_BASE_T = 0.008  # base plate thickness
BRACKET_CZ = PIVOT_Z    # bracket centered on pivot axis

AXLE_R = 0.014          # axle bolt radius
AXLE_HALF = 0.14        # axle half-length along Y (extends past bracket cheeks)

CAP_R = 0.032           # axle cap disc radius
CAP_T = 0.010           # axle cap thickness

SEAT_X = 1.02           # seat center X offset from pivot
SEAT_W = 0.28           # seat width (along Y)
SEAT_D = 0.26           # seat depth (along X)
SEAT_T = 0.012          # seat pan thickness
LIP_T = 0.008           # lip wall thickness
LIP_BACK = 0.058        # back lip height (tallest)
LIP_SIDE = 0.040        # side lip height
LIP_FRONT = 0.025       # front lip height (shortest, for leg clearance)

HANDLE_X = 0.88         # handle post X offset from pivot
HANDLE_POST_H = 0.32    # handle post height above plank top
HANDLE_POST_R = 0.011   # handle post tube radius
GRIP_R = 0.014          # grip bar radius
GRIP_LEN = 0.18         # grip bar length along Y

ROCK_LIMIT = 0.262      # ~15 degrees each way


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw_plank")

    model.material("galvanized_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    model.material("dark_bracket", rgba=(0.18, 0.18, 0.20, 1.0))
    model.material("plank_green", rgba=(0.16, 0.56, 0.24, 1.0))
    model.material("seat_blue", rgba=(0.12, 0.36, 0.68, 1.0))
    model.material("axle_cap_orange", rgba=(0.92, 0.48, 0.06, 1.0))
    model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("silver_bolt", rgba=(0.74, 0.75, 0.78, 1.0))

    # =================================================================
    # Fixed support frame: A-frame legs + bracket + axle + axle caps.
    # =================================================================
    frame = model.part("support_frame")

    # Two round tube legs forming an inverted V (A-frame), spreading in Y.
    for i, sy in enumerate((1.0, -1.0)):
        # Leg center and tilt — legs attach to bracket cheeks, not center.
        leg_cy = sy * LEG_MID_Y
        leg_cz = LEG_MID_Z
        frame.visual(
            Cylinder(radius=LEG_R, length=LEG_LEN),
            origin=Origin(
                xyz=(0.0, leg_cy, leg_cz),
                rpy=(sy * LEG_ANGLE, 0.0, 0.0),
            ),
            material="galvanized_steel",
            name=f"leg_{i}",
        )
        # Foot plate at the ground contact.
        frame.visual(
            Cylinder(radius=FOOT_R, length=FOOT_T),
            origin=Origin(xyz=(0.0, sy * LEG_SPREAD, FOOT_T / 2.0)),
            material="dark_bracket",
            name=f"foot_plate_{i}",
        )

    # Horizontal crossbar between the legs for lateral bracing.
    cross_z = CROSS_Z_FRAC * BRACKET_BOTTOM_Z
    cross_y = LEG_SPREAD - LEG_DY * CROSS_Z_FRAC
    frame.visual(
        Cylinder(radius=CROSSBAR_R, length=2.0 * cross_y),
        origin=Origin(
            xyz=(0.0, 0.0, cross_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="galvanized_steel",
        name="crossbar",
    )

    # Central pivot bracket — two cheek plates with a gap for the plank.
    # Each cheek is on the +Y and -Y sides of the pivot.
    cheek_t = 0.012  # cheek thickness
    cheek_gap = 0.22  # gap between cheeks (slightly > PLANK_W)
    cheek_h = BRACKET_H
    cheek_w = BRACKET_W
    for j, sy in enumerate((1.0, -1.0)):
        cheek_y = sy * (cheek_gap / 2.0 + cheek_t / 2.0)
        frame.visual(
            Box((cheek_w, cheek_t, cheek_h)),
            origin=Origin(xyz=(0.0, cheek_y, BRACKET_CZ)),
            material="dark_bracket",
            name=f"bracket_cheek_{j}",
        )
    # Base plate connecting the cheeks.
    base_w = BRACKET_W
    base_d = cheek_gap + 2.0 * cheek_t
    base_t = 0.008
    frame.visual(
        Box((base_w, base_d, base_t)),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ - cheek_h / 2.0 + base_t / 2.0)),
        material="dark_bracket",
        name="bracket_base",
    )

    # Bolt heads on bracket cheek outer faces.
    cheek_outer_y = cheek_gap / 2.0 + cheek_t
    for i, sy in enumerate((1.0, -1.0)):
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            dx = 0.034 * math.cos(ang * math.pi)
            dz = 0.034 * math.sin(ang * math.pi)
            frame.visual(
                Cylinder(radius=0.007, length=0.008),
                origin=Origin(
                    xyz=(dx, sy * (cheek_outer_y + 0.004), PIVOT_Z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_bolt",
                name=f"bracket_bolt_{i}_{j}",
            )

    # Axle bolt running through the bracket along Y.
    frame.visual(
        Cylinder(radius=AXLE_R, length=2.0 * AXLE_HALF),
        origin=Origin(
            xyz=(0.0, 0.0, PIVOT_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="galvanized_steel",
        name="pivot_axle",
    )

    # Visible axle caps — round discs on each side of the bracket.
    for i, sy in enumerate((1.0, -1.0)):
        frame.visual(
            Cylinder(radius=CAP_R, length=CAP_T),
            origin=Origin(
                xyz=(0.0, sy * (AXLE_HALF + CAP_T / 2.0), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="axle_cap_orange",
            name=f"axle_cap_{i}",
        )

    # =================================================================
    # Rocker: straight plank beam + center hub + mirrored seats + handles.
    # Part frame sits at the pivot axis so the revolute joint needs no offset.
    # =================================================================
    plank = model.part("plank")

    # Main plank beam — straight rectangular timber/metal plank.
    plank.visual(
        Box((PLANK_L, PLANK_W, PLANK_T)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="plank_green",
        name="plank_beam",
    )

    # Seat profile for extrusion.
    seat_profile = rounded_rect_profile(SEAT_D, SEAT_W, 0.030)

    for i, s in enumerate((1.0, -1.0)):
        sx = s * SEAT_X
        plank_top = PLANK_T / 2.0

        # Seat pan — extruded rounded-rect plate sitting on the plank top.
        seat_cz = plank_top + SEAT_T / 2.0
        plank.visual(
            mesh_from_geometry(
                ExtrudeGeometry(seat_profile, SEAT_T, cap=True, center=True),
                f"seat_pan_{i}",
            ),
            origin=Origin(xyz=(sx, 0.0, seat_cz)),
            material="seat_blue",
            name=f"seat_pan_{i}",
        )

        # Raised lip walls around the seat perimeter.
        # The seat depth is along X, width along Y.
        # Back lip: at the outer end (away from center), tallest wall.
        back_x = sx + s * (SEAT_D / 2.0 - LIP_T / 2.0)
        plank.visual(
            Box((LIP_T, SEAT_W - 0.02, LIP_BACK)),
            origin=Origin(
                xyz=(back_x, 0.0, plank_top + SEAT_T + LIP_BACK / 2.0),
            ),
            material="seat_blue",
            name=f"seat_lip_back_{i}",
        )
        # Front lip: at the inner end (toward center), shortest wall.
        front_x = sx - s * (SEAT_D / 2.0 - LIP_T / 2.0)
        plank.visual(
            Box((LIP_T, SEAT_W - 0.02, LIP_FRONT)),
            origin=Origin(
                xyz=(front_x, 0.0, plank_top + SEAT_T + LIP_FRONT / 2.0),
            ),
            material="seat_blue",
            name=f"seat_lip_front_{i}",
        )
        # Side lips: along each side of the seat.
        for j, sy in enumerate((1.0, -1.0)):
            lip_y = sy * (SEAT_W / 2.0 - LIP_T / 2.0)
            plank.visual(
                Box((SEAT_D - 2.0 * LIP_T, LIP_T, LIP_SIDE)),
                origin=Origin(
                    xyz=(sx, lip_y, plank_top + SEAT_T + LIP_SIDE / 2.0),
                ),
                material="seat_blue",
                name=f"seat_lip_side_{i}_{j}",
            )

        # Handle post — vertical tube rising from the plank near the seat.
        post_base_z = plank_top
        post_cz = post_base_z + HANDLE_POST_H / 2.0
        plank.visual(
            Cylinder(radius=HANDLE_POST_R, length=HANDLE_POST_H),
            origin=Origin(xyz=(s * HANDLE_X, 0.0, post_cz)),
            material="galvanized_steel",
            name=f"handle_post_{i}",
        )
        # Horizontal grip bar at the top of the post.
        grip_z = post_base_z + HANDLE_POST_H - GRIP_R
        plank.visual(
            Cylinder(radius=GRIP_R, length=GRIP_LEN),
            origin=Origin(
                xyz=(s * HANDLE_X, 0.0, grip_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="grip_black",
            name=f"handle_grip_{i}",
        )

    # =================================================================
    # Articulation: plank rocks on a single revolute joint at the bracket.
    # Axis along Y (perpendicular to plank length). Positive q tilts +X down.
    # =================================================================
    model.articulation(
        "plank_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=plank,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0, velocity=1.5, lower=-ROCK_LIMIT, upper=ROCK_LIMIT
        ),
    )

    return model


def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    plank = object_model.get_part("plank")
    pivot = object_model.get_articulation("plank_pivot")

    # The pivot axle passes through the plank beam (intentional fit).
    ctx.allow_overlap(
        plank,
        frame,
        elem_a="plank_beam",
        elem_b="pivot_axle",
        reason="The pivot axle passes through the plank beam as the rocking axis.",
    )
    # Plank beam is centered between bracket cheeks (Y direction).
    beam = ctx.part_element_world_aabb(plank, elem="plank_beam")
    cheek0 = ctx.part_element_world_aabb(frame, elem="bracket_cheek_0")
    cheek1 = ctx.part_element_world_aabb(frame, elem="bracket_cheek_1")
    ctx.check(
        "plank beam centered between bracket cheeks",
        cheek0 is not None
        and cheek1 is not None
        and beam is not None
        and cheek0[0][1] > beam[1][1]  # cheek_0 inner face beyond beam +Y edge
        and cheek1[1][1] < beam[0][1],  # cheek_1 inner face beyond beam -Y edge
        details=f"cheek0={cheek0}, cheek1={cheek1}, beam={beam}",
    )
    # Plank beam height overlaps cheek height (both span the pivot region).
    ctx.check(
        "plank beam height overlaps bracket cheek height",
        cheek0 is not None
        and beam is not None
        and beam[1][2] > cheek0[0][2]  # beam top above cheek bottom
        and beam[0][2] < cheek0[1][2],  # beam bottom below cheek top
        details=f"cheek0={cheek0}, beam={beam}",
    )

    # Plank beam spans the seesaw length.
    ctx.check(
        "plank beam spans at least 2.2 m",
        beam is not None and (beam[1][0] - beam[0][0]) >= 2.2,
        details=f"beam={beam}",
    )

    # Overall envelope checks.
    pa = ctx.part_world_aabb(plank)
    fa = ctx.part_world_aabb(frame)
    ctx.check(
        "overall length about 2.6 m",
        pa is not None and 2.3 <= (pa[1][0] - pa[0][0]) <= 2.8,
        details=f"plank aabb={pa}",
    )
    ctx.check(
        "overall height about 0.9 m",
        pa is not None and fa is not None and 0.80 <= max(pa[1][2], fa[1][2]) <= 1.0,
        details=f"plank={pa}, frame={fa}",
    )

    # A-frame legs exist and reach the ground.
    leg0 = ctx.part_element_world_aabb(frame, elem="leg_0")
    leg1 = ctx.part_element_world_aabb(frame, elem="leg_1")
    ctx.check(
        "A-frame legs spread laterally",
        leg0 is not None
        and leg1 is not None
        and abs(0.5 * (leg0[0][1] + leg0[1][1])) > 0.10
        and abs(0.5 * (leg1[0][1] + leg1[1][1])) > 0.10,
        details=f"leg0={leg0}, leg1={leg1}",
    )
    ctx.check(
        "legs reach near ground level",
        leg0 is not None and leg1 is not None
        and leg0[0][2] < 0.05 and leg1[0][2] < 0.05,
        details=f"leg0={leg0}, leg1={leg1}",
    )

    # Axle caps are visible on each side of the bracket cheeks.
    cap0 = ctx.part_element_world_aabb(frame, elem="axle_cap_0")
    cap1 = ctx.part_element_world_aabb(frame, elem="axle_cap_1")
    ctx.check(
        "axle caps extend beyond bracket cheeks on both sides",
        cap0 is not None
        and cap1 is not None
        and cheek0 is not None
        and cheek1 is not None
        and cap0[1][1] > cheek0[1][1]  # cap_0 outer beyond cheek_0 outer
        and cap1[0][1] < cheek1[0][1],  # cap_1 outer beyond cheek_1 outer
        details=f"cap0={cap0}, cap1={cap1}, cheek0={cheek0}, cheek1={cheek1}",
    )

    # Molded seats with raised lips at each end.
    seat0 = ctx.part_element_world_aabb(plank, elem="seat_pan_0")
    seat1 = ctx.part_element_world_aabb(plank, elem="seat_pan_1")
    lip_back_0 = ctx.part_element_world_aabb(plank, elem="seat_lip_back_0")
    lip_back_1 = ctx.part_element_world_aabb(plank, elem="seat_lip_back_1")
    ctx.check(
        "seats at opposite ends of the plank",
        seat0 is not None
        and seat1 is not None
        and 0.5 * (seat0[0][0] + seat0[1][0]) > 0.7
        and 0.5 * (seat1[0][0] + seat1[1][0]) < -0.7,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "seat back lips rise above seat pans",
        lip_back_0 is not None
        and lip_back_1 is not None
        and seat0 is not None
        and seat1 is not None
        and lip_back_0[1][2] > seat0[1][2]
        and lip_back_1[1][2] > seat1[1][2],
        details=f"lip_back_0={lip_back_0}, lip_back_1={lip_back_1}",
    )

    # Handles above the plank near seats.
    grip0 = ctx.part_element_world_aabb(plank, elem="handle_grip_0")
    grip1 = ctx.part_element_world_aabb(plank, elem="handle_grip_1")
    ctx.check(
        "handle grips above the plank beam",
        grip0 is not None
        and grip1 is not None
        and beam is not None
        and grip0[0][2] > beam[1][2]
        and grip1[0][2] > beam[1][2],
        details=f"grip0={grip0}, grip1={grip1}, beam={beam}",
    )

    # Joint limits: about +/- 15 degrees.
    lim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # Decisive pose checks: plank tilts as one body.
    base_rest = ctx.part_world_aabb(frame)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(plank, elem="seat_pan_0")
        seat1_up = ctx.part_element_world_aabb(plank, elem="seat_pan_1")
        plank_posed = ctx.part_world_aabb(plank)
        frame_posed = ctx.part_world_aabb(frame)
        ctx.check(
            "positive tilt lowers seat_0 and raises seat_1",
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
            plank_posed is not None and plank_posed[0][2] > 0.01,
            details=f"plank={plank_posed}",
        )
        ctx.check(
            "support frame stays fixed while rocking",
            base_rest is not None
            and frame_posed is not None
            and abs(base_rest[1][2] - frame_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={frame_posed}",
        )
    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(plank, elem="seat_pan_0")
        plank_up = ctx.part_world_aabb(plank)
        ctx.check(
            "negative tilt raises seat_0",
            seat0_up is not None
            and seat0 is not None
            and seat0_up[0][2] > seat0[0][2] + 0.10,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "plank clears the ground at opposite tilt",
            plank_up is not None and plank_up[0][2] > 0.01,
            details=f"plank={plank_up}",
        )

    return ctx.report()


object_model = build_object_model()
