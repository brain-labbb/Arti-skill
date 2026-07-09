from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X/Y horizontal, Z up.
# Cross seesaw: two perpendicular curved beams, 4 seats.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.38          # world height of the rocking axis
BEAM_R = 0.06           # main tube radius (~120 mm diameter)
BEAM_HALF = 1.15        # half-length of each curved beam
CURVE_C = 0.1285        # parabolic curvature of the banana beam
BEAM_CENTER_Z = 0.16    # beam centerline height at center, relative to pivot

COLLAR_DIST = 0.97      # clamp collar position along the beam from center
SEAT_CENTER_DIST = 1.14
SEAT_Z = 0.062          # seat plate mid-plane, relative to the pivot
PLATE_T = 0.012
HANDLE_DIST = 1.03
HANDLE_Z = 0.552        # handle plate mid-plane, relative to the pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way

PEDESTAL_R = 0.075
PEDESTAL_H = 0.24
BRACKET_SIZE = (0.18, 0.18, 0.19)
BRACKET_CZ = 0.335      # bracket box center height

# Support legs
LEG_ANGLES = [math.pi * 0.25, math.pi * 0.75, math.pi * 1.25, math.pi * 1.75]
LEG_CENTER_R = 0.060    # leg center horizontal distance from origin (inside pedestal)
LEG_CENTER_Z = 0.07     # leg center height
LEG_LENGTH = 0.18       # leg cylinder length
LEG_R = 0.022           # leg cylinder radius
LEG_TILT = 0.82         # tilt angle from vertical (radians, ~47°)
PAD_R = 0.050
PAD_H = 0.016

# Locking pin dimensions
PIN_R = 0.010
PIN_SHAFT_LEN = 0.08
PIN_SLIDE = 0.07        # prismatic travel
PIN_ORIGIN_X = 0.145    # pin rest position (retracted, outside bracket)


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def _make_beam_points(axis_angle: float) -> list[tuple[float, float, float]]:
    """Generate spline points for a curved beam along a direction in XY plane."""
    ca, sa = math.cos(axis_angle), math.sin(axis_angle)
    n = 12
    pts = []
    for k in range(-n, n + 1):
        t = BEAM_HALF * k / n
        x = t * ca
        y = t * sa
        z = _beam_z(t)
        pts.append((x, y, z))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_playground_seesaw")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.11, 1.0))
    model.material("pin_steel", rgba=(0.62, 0.63, 0.65, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: pedestal, bracket, support legs with rubber pads.
    # -----------------------------------------------------------------
    base = model.part("pedestal_mount")

    # Central ground pedestal
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H / 2.0)),
        material="light_gray",
        name="ground_pedestal",
    )

    # Black cast pivot bracket on top (square for cross seesaw)
    base.visual(
        Box(BRACKET_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ)),
        material="matte_black",
        name="pivot_bracket",
    )

    # Pivot bosses on ±Y bracket cheeks only (rocking axis is Y)
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.045, length=0.020),
            origin=Origin(
                xyz=(0.0, sy * 0.099, PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.3, 1.0, 1.7)):
            bx = 0.028 * math.cos(ang * math.pi)
            bz = 0.028 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.007, length=0.010),
                origin=Origin(
                    xyz=(bx, sy * 0.111, PIVOT_Z + bz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
            )

    # Locking pin guide boss on +X side of bracket (small protrusion)
    base.visual(
        Cylinder(radius=0.022, length=0.025),
        origin=Origin(
            xyz=(0.095, 0.0, 0.28),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="matte_black",
        name="pin_guide_boss",
    )

    # Four support legs with rubber ground pads
    for i, la in enumerate(LEG_ANGLES):
        ca, sa = math.cos(la), math.sin(la)
        # Leg center inside pedestal volume for connectivity
        leg_cx = ca * LEG_CENTER_R
        leg_cy = sa * LEG_CENTER_R
        leg_cz = LEG_CENTER_Z
        # Tilt outward: rpy = (0, -tilt, azimuth) makes cylinder tilt outward-down
        base.visual(
            Cylinder(radius=LEG_R, length=LEG_LENGTH),
            origin=Origin(
                xyz=(leg_cx, leg_cy, leg_cz),
                rpy=(0.0, -LEG_TILT, la),
            ),
            material="light_gray",
            name=f"support_leg_{i}",
        )
        # Ground pad at the foot of each leg (outer-bottom end)
        half = LEG_LENGTH / 2.0
        sin_t = math.sin(LEG_TILT)
        cos_t = math.cos(LEG_TILT)
        pad_r = LEG_CENTER_R + half * sin_t
        pad_z = max(0.0, LEG_CENTER_Z - half * cos_t - PAD_H / 2.0)
        base.visual(
            Cylinder(radius=PAD_R, length=PAD_H),
            origin=Origin(xyz=(ca * pad_r, sa * pad_r, pad_z + PAD_H / 2.0)),
            material="rubber_black",
            name=f"ground_pad_{i}",
        )

    # -----------------------------------------------------------------
    # Rocker: two perpendicular curved beams + pivot stub + 4 seat/handle ends.
    # -----------------------------------------------------------------
    rocker = model.part("rocker")

    # Two curved beams: along X (angle=0) and along Y (angle=pi/2)
    for bi, angle in enumerate([0.0, math.pi / 2.0]):
        pts = _make_beam_points(angle)
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    pts,
                    radius=BEAM_R,
                    samples_per_segment=4,
                    radial_segments=28,
                    cap_ends=True,
                ),
                f"beam_tube_{bi}",
            ),
            material="gloss_red_orange",
            name=f"beam_tube_{bi}",
        )

    # Central hub where beams cross
    rocker.visual(
        Cylinder(radius=0.09, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, BEAM_CENTER_Z)),
        material="gloss_red_orange",
        name="cross_hub",
    )

    # Red flare wedge under beam center
    wedge = ConeGeometry(0.09, 0.10, radial_segments=28).rotate_x(math.pi)
    wedge.translate(0.0, 0.0, 0.115)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="gloss_red_orange",
        name="pivot_wedge",
    )

    # Short red stub descending into bracket
    rocker.visual(
        Cylinder(radius=0.048, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="gloss_red_orange",
        name="pivot_stub",
    )

    # Shared profiles
    seat_profile = sample_catmull_rom_spline_2d(
        [
            (0.21, 0.0),
            (0.05, 0.115),
            (-0.10, 0.145),
            (-0.185, 0.10),
            (-0.21, 0.0),
            (-0.185, -0.10),
            (-0.10, -0.145),
            (0.05, -0.115),
        ],
        samples_per_segment=8,
        closed=True,
    )
    grip_outer = rounded_rect_profile(0.18, 0.30, 0.05)
    grip_hole = rounded_rect_profile(0.06, 0.09, 0.02)
    grip_holes = [
        [(hx, hy + 0.075) for hx, hy in grip_hole],
        [(hx, hy - 0.075) for hx, hy in grip_hole],
    ]

    collar_z = _beam_z(COLLAR_DIST)
    slope = 2.0 * CURVE_C * COLLAR_DIST
    tangent = math.atan(slope)

    # 4 ends: +X, -X, +Y, -Y
    end_dirs = [
        (1.0, 0.0, 0.0),           # +X
        (-1.0, 0.0, math.pi),       # -X
        (0.0, 1.0, math.pi / 2.0),  # +Y
        (0.0, -1.0, -math.pi / 2.0),  # -Y
    ]

    for i, (dx, dy, seat_yaw) in enumerate(end_dirs):
        sign = 1.0 if (dx > 0 or dy > 0) else -1.0
        bx, by = dx, dy  # unit direction along beam

        # Clamp collar position
        cx = bx * COLLAR_DIST
        cy = by * COLLAR_DIST

        # Collar orientation aligned to beam tangent
        if abs(bx) > 0.5:
            # X-axis beam: collar tilted around Y
            collar_rpy = (0.0, math.pi / 2.0 - sign * tangent, 0.0)
        else:
            # Y-axis beam: collar tilted around X
            collar_rpy = (-(math.pi / 2.0 - sign * tangent), 0.0, 0.0)
        rocker.visual(
            Cylinder(radius=0.080, length=0.085),
            origin=Origin(xyz=(cx, cy, collar_z), rpy=collar_rpy),
            material="matte_black",
            name=f"clamp_collar_{i}",
        )

        # Collar bolts: positioned on the collar surface
        for j, boff in enumerate((-1.0, 1.0)):
            if abs(bx) > 0.5:
                # X-axis beam: bolts on ±Y sides of collar
                bolt_pos = (cx, boff * 0.082, collar_z)
                bolt_rpy = (math.pi / 2.0, 0.0, 0.0)
            else:
                # Y-axis beam: bolts on ±X sides of collar
                bolt_pos = (boff * 0.082, cy, collar_z)
                bolt_rpy = (0.0, math.pi / 2.0, 0.0)
            rocker.visual(
                Cylinder(radius=0.011, length=0.032),
                origin=Origin(xyz=bolt_pos, rpy=bolt_rpy),
                material="silver_rivet",
                name=f"collar_bolt_{i}_{j}",
            )

        # Drop tube from collar to seat
        seat_x = bx * SEAT_CENTER_DIST
        seat_y = by * SEAT_CENTER_DIST
        drop_pts = [
            (cx, cy, collar_z),
            (bx * 1.05, by * 1.05, 0.185),
            (bx * 1.12, by * 1.12, 0.105),
            (seat_x, seat_y, 0.066),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    drop_pts, radius=0.026, samples_per_segment=10, radial_segments=18
                ),
                f"drop_tube_{i}",
            ),
            material="gloss_red_orange",
            name=f"drop_tube_{i}",
        )

        # Seat plate with rivets
        seat = ExtrudeGeometry(seat_profile, PLATE_T, cap=True, center=True)
        seat.rotate_z(seat_yaw)
        seat.translate(seat_x, seat_y, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat, f"seat_plate_{i}"),
            material="dark_gray_steel",
            name=f"seat_plate_{i}",
        )
        rivet_local = [(0.13, 0.0), (0.0, 0.10), (0.0, -0.10), (-0.13, 0.075), (-0.13, -0.075)]
        cos_y = math.cos(seat_yaw)
        sin_y = math.sin(seat_yaw)
        for j, (lx, ly) in enumerate(rivet_local):
            rx = lx * cos_y - ly * sin_y
            ry = lx * sin_y + ly * cos_y
            rocker.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(seat_x + rx, seat_y + ry, 0.070)),
                material="silver_rivet",
                name=f"seat_rivet_{i}_{j}",
            )

        # Seat stop fin
        rocker.visual(
            Box((0.045, 0.022, 0.04)),
            origin=Origin(xyz=(bx * 1.26, by * 1.26, 0.038)),
            material="matte_black",
            name=f"seat_fin_{i}",
        )

        # Handle post
        handle_x = bx * HANDLE_DIST
        handle_y = by * HANDLE_DIST
        post_pts = [
            (cx, cy, 0.285),
            (bx * 0.985, by * 0.985, 0.40),
            (bx * 1.01, by * 1.01, 0.48),
            (handle_x, handle_y, 0.550),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    post_pts, radius=0.021, samples_per_segment=10, radial_segments=18
                ),
                f"handle_post_{i}",
            ),
            material="gloss_red_orange",
            name=f"handle_post_{i}",
        )

        # Grip plate with hand cutouts
        grip = ExtrudeWithHolesGeometry(grip_outer, grip_holes, PLATE_T, cap=True, center=True)
        grip.rotate_z(seat_yaw)
        grip.translate(handle_x, handle_y, HANDLE_Z)
        rocker.visual(
            mesh_from_geometry(grip, f"handle_plate_{i}"),
            material="dark_gray_steel",
            name=f"handle_plate_{i}",
        )

    # -----------------------------------------------------------------
    # Locking pin: prismatic slide on bracket +X side.
    # At q=0 the pin is retracted (outside bracket); positive q pushes it in.
    # -----------------------------------------------------------------
    lock_pin = model.part("locking_pin")

    # Pin shaft (cylinder along local Z, rotated to horizontal along X)
    lock_pin.visual(
        Cylinder(radius=PIN_R, length=PIN_SHAFT_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="pin_steel",
        name="pin_shaft",
    )
    # Pin head (larger knob at outboard end)
    lock_pin.visual(
        Cylinder(radius=PIN_R * 2.4, length=0.020),
        origin=Origin(xyz=(PIN_SHAFT_LEN / 2.0 + 0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="matte_black",
        name="pin_head",
    )
    # Pin retaining ring at inboard end
    lock_pin.visual(
        Cylinder(radius=PIN_R * 1.8, length=0.006),
        origin=Origin(xyz=(-(PIN_SHAFT_LEN / 2.0 + 0.003), 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="silver_rivet",
        name="pin_retainer",
    )

    # -----------------------------------------------------------------
    # Articulations
    # -----------------------------------------------------------------

    # Main rocking pivot
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0, velocity=1.5, lower=-ROCK_LIMIT, upper=ROCK_LIMIT
        ),
    )

    # Locking pin: prismatic slide along -X (pushing into bracket)
    model.articulation(
        "lock_pin_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_pin,
        origin=Origin(xyz=(PIN_ORIGIN_X, 0.0, 0.28)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.3, lower=0.0, upper=PIN_SLIDE
        ),
    )

    return model


def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("pedestal_mount")
    rocker = object_model.get_part("rocker")
    lock_pin = object_model.get_part("locking_pin")
    pivot = object_model.get_articulation("rocker_pivot")
    pin_joint = object_model.get_articulation("lock_pin_slide")

    # --- Pivot stub captured in bracket ---
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        reason="The red center stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    # --- Pin retainer seated against guide boss at rest ---
    ctx.allow_overlap(
        lock_pin,
        base,
        elem_a="pin_retainer",
        elem_b="pin_guide_boss",
        reason="The pin retainer seats against the guide boss when the locking pin is in the retracted (rest) position.",
    )
    ctx.expect_contact(
        lock_pin,
        base,
        elem_a="pin_retainer",
        elem_b="pin_guide_boss",
        contact_tol=0.010,
        name="pin retainer contacts guide boss at rest",
    )
    ctx.expect_overlap(
        rocker,
        base,
        axes="z",
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        min_overlap=0.04,
        name="pivot stub inserted into bracket",
    )
    ctx.expect_within(
        rocker,
        base,
        axes="xy",
        inner_elem="pivot_stub",
        outer_elem="pivot_bracket",
        margin=0.0,
        name="pivot stub centered in bracket",
    )

    # --- Bracket seated on pedestal ---
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
    )

    # --- Cross beams: two perpendicular beams ---
    beam0 = ctx.part_element_world_aabb(rocker, elem="beam_tube_0")
    beam1 = ctx.part_element_world_aabb(rocker, elem="beam_tube_1")
    ctx.check(
        "beam 0 spans along X axis",
        beam0 is not None and (beam0[1][0] - beam0[0][0]) >= 2.0,
        details=f"beam0={beam0}",
    )
    ctx.check(
        "beam 1 spans along Y axis",
        beam1 is not None and (beam1[1][1] - beam1[0][1]) >= 2.0,
        details=f"beam1={beam1}",
    )
    ctx.check(
        "beams are perpendicular (one along X, one along Y)",
        beam0 is not None and beam1 is not None
        and (beam0[1][0] - beam0[0][0]) > (beam0[1][1] - beam0[0][1]) * 2.0
        and (beam1[1][1] - beam1[0][1]) > (beam1[1][0] - beam1[0][0]) * 2.0,
        details=f"beam0={beam0}, beam1={beam1}",
    )

    # --- Four seats at beam ends ---
    seats = []
    for i in range(4):
        s = ctx.part_element_world_aabb(rocker, elem=f"seat_plate_{i}")
        seats.append(s)
        ctx.check(f"seat {i} exists", s is not None, details=f"seat_plate_{i}={s}")

    ctx.check(
        "seat 0 and seat 1 at opposite X ends",
        seats[0] is not None and seats[1] is not None
        and seats[0][0][0] > 0.7 and seats[1][1][0] < -0.7,
        details=f"seat0={seats[0]}, seat1={seats[1]}",
    )
    ctx.check(
        "seat 2 and seat 3 at opposite Y ends",
        seats[2] is not None and seats[3] is not None
        and seats[2][0][1] > 0.7 and seats[3][1][1] < -0.7,
        details=f"seat2={seats[2]}, seat3={seats[3]}",
    )

    # --- Four handles above seats ---
    for i in range(4):
        g = ctx.part_element_world_aabb(rocker, elem=f"handle_plate_{i}")
        ctx.check(f"handle {i} exists above seat", g is not None, details=f"handle_plate_{i}={g}")

    # --- Rubber ground pads under support legs ---
    pads = []
    for i in range(4):
        p = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        pads.append(p)
        ctx.check(
            f"rubber ground pad {i} exists at ground level",
            p is not None and p[0][2] < 0.02,
            details=f"ground_pad_{i}={p}",
        )

    # Pads distributed around pedestal (at least some offset from center)
    ctx.check(
        "ground pads distributed around pedestal",
        all(p is not None for p in pads)
        and max(abs(0.5 * (p[0][0] + p[1][0])) for p in pads) > 0.05
        and max(abs(0.5 * (p[0][1] + p[1][1])) for p in pads) > 0.05,
        details=f"pads={pads}",
    )

    # Support legs exist and connect pedestal to pads
    legs = []
    for i in range(4):
        lg = ctx.part_element_world_aabb(base, elem=f"support_leg_{i}")
        legs.append(lg)
        ctx.check(f"support leg {i} exists", lg is not None, details=f"support_leg_{i}={lg}")

    # Legs bridge pedestal and pads
    for i in range(4):
        ctx.check(
            f"leg {i} connects pedestal to pad",
            legs[i] is not None and pedestal is not None and pads[i] is not None
            and legs[i][0][2] <= pedestal[0][2] + 0.05
            and legs[i][1][2] >= pads[i][0][2] - 0.01,
            details=f"leg={legs[i]}, pedestal={pedestal}, pad={pads[i]}",
        )

    # --- Locking pin articulation ---
    ctx.check(
        "locking pin has prismatic joint",
        pin_joint is not None
        and pin_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"pin_joint type={pin_joint.articulation_type if pin_joint else None}",
    )
    pin_lim = pin_joint.motion_limits
    ctx.check(
        "locking pin slide range is reasonable",
        pin_lim is not None and pin_lim.lower >= 0.0 and 0.02 <= pin_lim.upper <= 0.12,
        details=f"pin limits=({pin_lim.lower}, {pin_lim.upper})",
    )

    # Pin exists near bracket height
    pin_shaft = ctx.part_element_world_aabb(lock_pin, elem="pin_shaft")
    ctx.check(
        "locking pin shaft exists near bracket",
        pin_shaft is not None and bracket is not None
        and pin_shaft[0][2] > bracket[0][2] - 0.15
        and pin_shaft[1][2] < bracket[1][2] + 0.05,
        details=f"pin_shaft={pin_shaft}, bracket={bracket}",
    )

    # Pin translates when actuated
    pin_rest = ctx.part_world_aabb(lock_pin)
    with ctx.pose({pin_joint: PIN_SLIDE}):
        pin_engaged = ctx.part_world_aabb(lock_pin)
        ctx.check(
            "locking pin translates when slide is actuated",
            pin_rest is not None and pin_engaged is not None
            and abs(pin_engaged[0][0] - pin_rest[0][0]) > 0.02,
            details=f"rest={pin_rest}, engaged={pin_engaged}",
        )

    # --- Rocking pivot ---
    lim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # Pose check: rocking tilts the cross assembly
    base_rest = ctx.part_world_aabb(base)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
        rocker_posed = ctx.part_world_aabb(rocker)
        base_posed = ctx.part_world_aabb(base)
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None and seat1_up is not None
            and seats[0] is not None and seats[1] is not None
            and seat0_dn[1][2] < seats[0][1][2] - 0.10
            and seat1_up[1][2] > seats[1][1][2] + 0.10,
            details=f"seat0_dn={seat0_dn}, seat1_up={seat1_up}",
        )
        ctx.check(
            "rocker clears the ground at full tilt",
            rocker_posed is not None and rocker_posed[0][2] > 0.005,
            details=f"rocker={rocker_posed}",
        )
        ctx.check(
            "pedestal stays fixed while rocking",
            base_rest is not None and base_posed is not None
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )

    # Overall cross-seesaw envelope
    ra = ctx.part_world_aabb(rocker)
    ctx.check(
        "cross seesaw spans about 2.3m in both horizontal directions",
        ra is not None
        and (ra[1][0] - ra[0][0]) >= 2.0
        and (ra[1][1] - ra[0][1]) >= 2.0,
        details=f"rocker aabb={ra}",
    )

    return ctx.report()


object_model = build_object_model()
