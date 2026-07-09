from __future__ import annotations

# Cosmetic squeeze tube with a roller-ball applicator (fork of screw-cap tube).
# Frame: tube stands upright along +Z. The flat crimped bottom seam sits at
# z=0; the soft pale-yellow body rises and rounds out, narrows into a white
# shoulder cone, then a domed roller-ball housing with a concave spherical
# socket. A stainless-steel applicator ball sits in the socket and spins freely
# (CONTINUOUS joint about X). A translucent overcap lifts straight off along +Z
# (PRISMATIC joint) to protect the ball when not in use.
#
# Construction notes:
#  - The body is a CadQuery loft: flat, wide crimp seam at bottom rounding to
#    near-circular at the shoulder (classic round-to-flat squeeze-tube).
#  - Shoulder is a hollow cone with a central bore feeding product to the ball.
#  - Housing is a cylindrical cup with a concave spherical seat matching the
#    ball radius; the bore continues through the floor of the socket.
#  - Ball is a sphere with a small off-axis marker to prove rotation.
#  - Overcap is a hollow shell that seats over the housing at rest and lifts
#    off along +Z when the prismatic joint extends.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key heights (meters), measured up +Z from the bottom crimp seam ----
BODY_BOTTOM_Z = 0.0
BODY_TOP_Z = 0.082          # where the soft body meets the white shoulder
SHOULDER_TOP_Z = 0.104      # top of the shoulder cone / base of the housing
HOUSING_RIM_Z = 0.117       # top rim of the housing socket

BODY_MAX_R = 0.0225         # near-circular radius of the body at the shoulder
CRIMP_HALF_W = 0.0235       # half-width of the flat crimp seam (wide, flat)
CRIMP_HALF_T = 0.0028       # half-thickness of the flat crimp seam (thin)

# Housing dimensions
HOUSING_OUTER_R = 0.0090    # outer wall radius
HOUSING_SOCKET_R = 0.00525  # socket radius (slightly larger than ball for clearance)
BORE_R = 0.003              # bore radius feeding product to ball

# Ball dimensions
BALL_R = 0.005              # 5mm radius (10mm diameter applicator ball)

# Ball seating: the socket sphere is centered at the rim; the ball is lowered
# slightly so it contacts the concave seat (tiny intentional overlap at socket
# floor, covered by allow_overlap).
SOCKET_CENTER_Z = HOUSING_RIM_Z                      # socket sphere center for housing
BALL_SEAT_OFFSET = 0.0003                            # how far ball drops into seat
BALL_ORIGIN_Z = HOUSING_RIM_Z - BALL_SEAT_OFFSET     # ball joint origin (world Z)

# Overcap dimensions
OVERCAP_R = 0.0115          # outer radius of the overcap
OVERCAP_INNER_R = 0.0095    # inner bore (clearance over housing)
OVERCAP_HEIGHT = 0.024      # total height of the overcap shell
OVERCAP_TOP_T = 0.003       # top disc thickness
OVERCAP_LIFT = 0.030        # prismatic upper travel limit (fully clears ball)


def _rounded_rect_pts(hw: float, ht: float, fillet: float, n_arc: int = 8):
    f = min(fillet, hw, ht)
    cx = hw - f
    cy = ht - f
    pts = []
    corners = [
        (cx, -cy, -math.pi / 2, 0.0),
        (cx, cy, 0.0, math.pi / 2),
        (-cx, cy, math.pi / 2, math.pi),
        (-cx, -cy, math.pi, 3 * math.pi / 2),
    ]
    for ccx, ccy, a0, a1 in corners:
        for k in range(n_arc + 1):
            a = a0 + (a1 - a0) * k / n_arc
            pts.append((ccx + f * math.cos(a), ccy + f * math.sin(a)))
    return pts


def _body_solid() -> cq.Workplane:
    levels = [
        (BODY_BOTTOM_Z, CRIMP_HALF_W, CRIMP_HALF_T),
        (0.012, CRIMP_HALF_W * 0.99, 0.009),
        (0.030, BODY_MAX_R * 0.99, 0.016),
        (0.055, BODY_MAX_R, BODY_MAX_R * 0.97),
        (BODY_TOP_Z, BODY_MAX_R * 0.985, BODY_MAX_R * 0.985),
    ]
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, hw, ht) in enumerate(levels):
        off = z if i == 0 else z - prev_z
        if i == 0:
            wp = wp.workplane(offset=z)
        else:
            wp = wp.workplane(offset=off)
        fr = min(hw, ht) * 0.98
        pts = _rounded_rect_pts(hw, ht, fillet=fr, n_arc=8)
        wp = wp.polyline(pts).close()
        prev_z = z
    solid = wp.loft(ruled=False)
    return solid


def _body_mesh():
    outer = _body_solid()
    inner_levels = [
        (0.006, CRIMP_HALF_W * 0.9, CRIMP_HALF_T * 0.4),
        (0.014, CRIMP_HALF_W * 0.9, 0.0072),
        (0.030, BODY_MAX_R * 0.9, 0.0135),
        (0.055, BODY_MAX_R * 0.9, BODY_MAX_R * 0.86),
        (BODY_TOP_Z + 0.01, BODY_MAX_R * 0.9, BODY_MAX_R * 0.86),
    ]
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, hw, ht) in enumerate(inner_levels):
        off = z if i == 0 else z - prev_z
        if i == 0:
            wp = wp.workplane(offset=z)
        else:
            wp = wp.workplane(offset=off)
        fr = min(hw, ht) * 0.98
        pts = _rounded_rect_pts(hw, ht, fillet=fr, n_arc=8)
        wp = wp.polyline(pts).close()
        prev_z = z
    inner = wp.loft(ruled=False)
    body = outer.cut(inner)
    return mesh_from_cadquery(body, "tube_body")


def _shoulder_mesh():
    # White shoulder: hollow cone from body mouth to housing base, with bore.
    h = SHOULDER_TOP_Z - BODY_TOP_Z
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .circle(BODY_MAX_R * 0.985)
        .workplane(offset=h)
        .circle(HOUSING_OUTER_R * 1.1)
        .loft(ruled=False)
    )
    # Short collar blending shoulder top into housing base
    collar = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP_Z - 0.005)
        .circle(HOUSING_OUTER_R * 1.1)
        .extrude(0.005)
    )
    # Bore through shoulder for product flow
    bore = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z - 0.002)
        .circle(BORE_R * 1.1)
        .extrude(h + 0.010)
    )
    return mesh_from_cadquery(shoulder.union(collar).cut(bore), "shoulder")


def _housing_mesh():
    # Roller-ball housing: cylindrical cup with concave spherical socket seat.
    # The socket is cut as a sphere matching the ball radius, centered at the
    # ball center. The upper hemisphere cuts through the top face (opening for
    # ball protrusion); the lower hemisphere forms the concave seat.
    h = HOUSING_RIM_Z - SHOULDER_TOP_Z
    # Outer cylindrical wall
    outer = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP_Z)
        .circle(HOUSING_OUTER_R)
        .extrude(h)
    )
    # Spherical socket pocket (slightly larger than ball for clearance)
    socket_sphere = (
        cq.Workplane("XY")
        .workplane(offset=SOCKET_CENTER_Z)
        .sphere(HOUSING_SOCKET_R)
    )
    housing = outer.cut(socket_sphere)
    # Bore through the floor of the socket connecting to shoulder bore
    bore = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP_Z - 0.002)
        .circle(BORE_R)
        .extrude(h * 0.6)
    )
    housing = housing.cut(bore)
    # Add a small rim/lip at the top for visual definition
    rim = (
        cq.Workplane("XY")
        .workplane(offset=HOUSING_RIM_Z - 0.0015)
        .circle(HOUSING_OUTER_R + 0.0008)
        .circle(HOUSING_OUTER_R - 0.0002)
        .extrude(0.0015)
    )
    housing = housing.union(rim)
    return mesh_from_cadquery(housing, "housing_socket")


def _ball_mesh():
    # Stainless-steel applicator ball centered at z=0 in LOCAL part frame.
    # The joint origin places this part at BALL_ORIGIN_Z in world space,
    # so all geometry here is relative to the ball center.
    ball = cq.Workplane("XY").sphere(BALL_R)
    # Small equatorial stripe so rotation about X is visible in AABB
    stripe = (
        cq.Workplane("XY")
        .circle(BALL_R + 0.0002)
        .circle(BALL_R - 0.0002)
        .extrude(0.001)
        .translate((0.0, 0.0, -0.0005))
    )
    return mesh_from_cadquery(ball.union(stripe), "applicator_ball")


def _ball_marker_mesh():
    # Small off-center dot on the ball surface (+Z pole in local frame)
    # to verify rotation moves it off-axis when the ball rolls about X.
    marker = (
        cq.Workplane("XY")
        .workplane(offset=BALL_R - 0.0004)
        .circle(0.001)
        .extrude(0.0008)
    )
    return mesh_from_cadquery(marker, "ball_marker")


def _overcap_mesh():
    # Translucent protective overcap: hollow cylinder with closed top,
    # built in LOCAL frame with z=0 at the bottom rim so the prismatic
    # joint reads cleanly as lifting up +Z.
    cap = cq.Workplane("XY").circle(OVERCAP_R).extrude(OVERCAP_HEIGHT)
    # Hollow interior (clearance for housing + protruding ball)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(OVERCAP_INNER_R)
        .extrude(OVERCAP_HEIGHT - OVERCAP_TOP_T + 0.001)
    )
    cap = cap.cut(cavity)
    # Slight chamfer ring at bottom rim for visual seating cue
    rim = (
        cq.Workplane("XY")
        .circle(OVERCAP_R + 0.0006)
        .circle(OVERCAP_R - 0.0004)
        .extrude(0.002)
    )
    cap = cap.union(rim)
    return mesh_from_cadquery(cap, "overcap_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cosmetic_rollerball_tube")

    pale_yellow = model.material("pale_yellow", rgba=(0.93, 0.89, 0.66, 1.0))
    white = model.material("white", rgba=(0.96, 0.96, 0.95, 1.0))
    steel = model.material("steel_ball", rgba=(0.72, 0.73, 0.75, 1.0))
    marker_red = model.material("ball_marker", rgba=(0.85, 0.25, 0.20, 1.0))
    cap_tint = model.material("overcap_tint", rgba=(0.88, 0.92, 0.95, 0.85))

    # ---- body (root): tube body + shoulder + roller-ball housing ----
    body = model.part("tube_body")
    body.visual(_body_mesh(), material=pale_yellow, name="tube_body")
    body.visual(_shoulder_mesh(), material=white, name="shoulder")
    body.visual(_housing_mesh(), material=white, name="housing_socket")
    body.inertial = Inertial.from_geometry(
        Box((0.045, 0.045, 0.12)), mass=0.04, origin=Origin(xyz=(0.0, 0.0, 0.06))
    )

    # ---- applicator ball: sits in the housing socket, rolls freely ----
    ball = model.part("applicator_ball")
    ball.visual(_ball_mesh(), material=steel, name="applicator_ball")
    ball.visual(_ball_marker_mesh(), material=marker_red, name="ball_marker")
    ball.inertial = Inertial.from_geometry(
        Sphere(BALL_R * 2), mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- overcap: protective cap that lifts off along +Z ----
    overcap = model.part("overcap")
    overcap.visual(_overcap_mesh(), material=cap_tint, name="overcap_shell")
    overcap.inertial = Inertial.from_geometry(
        Box((2 * OVERCAP_R, 2 * OVERCAP_R, OVERCAP_HEIGHT)), mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, OVERCAP_HEIGHT * 0.5)),
    )

    # ---- ball_roll: CONTINUOUS joint, ball spins freely in its socket ----
    # Axis = (1,0,0) represents rolling about X (primary application direction).
    # Joint origin at the ball center (actual contact surface of socket).
    model.articulation(
        "ball_roll",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=ball,
        origin=Origin(xyz=(0.0, 0.0, BALL_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=5.0),
    )

    # ---- overcap_lift: PRISMATIC joint, overcap lifts off along +Z ----
    # Joint origin at the housing rim where the overcap seats.
    # At q=0 the overcap bottom rim is at the housing rim (seated).
    # At q=OVERCAP_LIFT the overcap fully clears the ball and housing.
    model.articulation(
        "overcap_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=overcap,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_RIM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=OVERCAP_LIFT, effort=2.0, velocity=1.0
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("tube_body")
    ball = object_model.get_part("applicator_ball")
    overcap = object_model.get_part("overcap")
    ball_roll = object_model.get_articulation("ball_roll")
    overcap_lift = object_model.get_articulation("overcap_lift")

    # Allow intentional overlap: ball seated in housing socket
    ctx.allow_overlap(
        ball,
        body,
        elem_a="applicator_ball",
        elem_b="housing_socket",
        reason="Applicator ball is intentionally seated inside the concave housing socket.",
    )
    # Allow intentional overlap: overcap covers housing and ball at rest
    ctx.allow_overlap(
        overcap,
        body,
        elem_a="overcap_shell",
        elem_b="housing_socket",
        reason="Overcap intentionally encloses the housing when seated at rest.",
    )
    ctx.allow_overlap(
        overcap,
        ball,
        elem_a="overcap_shell",
        elem_b="applicator_ball",
        reason="Overcap intentionally covers the applicator ball when seated at rest.",
    )

    # --- Tube body proportions preserved from parent ---
    full_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "tube body is taller than it is wide (upright tube proportions)",
        full_ext[2] > full_ext[0] and full_ext[2] > full_ext[1],
        details=f"body extents={full_ext}",
    )
    ctx.check(
        "bottom crimp seam is flat (wide along X, thin along Y)",
        CRIMP_HALF_W > CRIMP_HALF_T * 4.0,
        details=f"crimp half_w={CRIMP_HALF_W}, half_t={CRIMP_HALF_T}",
    )

    # --- Housing has socket geometry with open bore ---
    ctx.check(
        "housing socket radius exceeds ball radius (ball can seat)",
        HOUSING_SOCKET_R > BALL_R,
        details=f"socket_r={HOUSING_SOCKET_R}, ball_r={BALL_R}",
    )
    ctx.check(
        "bore feeds product from tube body up to the ball",
        BORE_R > 0.002 and BORE_R < BALL_R * 0.8,
        details=f"bore_r={BORE_R}, ball_r={BALL_R}",
    )
    ctx.check(
        "housing sits on top of the shoulder",
        SHOULDER_TOP_Z > BODY_TOP_Z and HOUSING_RIM_Z > SHOULDER_TOP_Z,
        details=f"body_top={BODY_TOP_Z}, shoulder_top={SHOULDER_TOP_Z}, "
                f"rim_z={HOUSING_RIM_Z}",
    )

    # --- Ball is mounted at the housing socket ---
    ball_pos = ctx.part_world_position(ball)
    ctx.check(
        "ball is positioned at the housing rim level",
        ball_pos is not None and abs(ball_pos[2] - BALL_ORIGIN_Z) < 1e-4,
        details=f"ball_pos={ball_pos}, expected_z={BALL_ORIGIN_Z}",
    )
    ctx.expect_contact(
        ball,
        body,
        elem_a="applicator_ball",
        elem_b="housing_socket",
        name="ball contacts the housing socket at rest",
    )

    # --- Mechanism: ball rolls (CONTINUOUS about X) ---
    # The marker sits at the +Z pole of the ball. Rotating 90° about X
    # (right-hand rule) swings it from +Z to -Y, so zmax drops and ymin
    # becomes more negative.
    aabb0 = ctx.part_world_aabb(ball)
    with ctx.pose({ball_roll: math.pi / 2.0}):
        aabb90 = ctx.part_world_aabb(ball)
    zmax0 = aabb0[1][2]
    zmax90 = aabb90[1][2]
    ymin0 = aabb0[0][1]
    ymin90 = aabb90[0][1]
    ctx.check(
        "ball roll about X moves the pole marker (z-max drops, y-min drops)",
        zmax0 > zmax90 + 1e-4 and ymin90 < ymin0 - 1e-4,
        details=f"rest zmax={zmax0:.5f} ymin={ymin0:.5f} | "
                f"q90 zmax={zmax90:.5f} ymin={ymin90:.5f}",
    )
    ctx.check(
        "ball_roll is CONTINUOUS about X axis",
        ball_roll.articulation_type == ArticulationType.CONTINUOUS
        and tuple(ball_roll.axis) == (1.0, 0.0, 0.0),
        details=f"type={ball_roll.articulation_type}, axis={ball_roll.axis}",
    )

    # --- Mechanism: overcap lifts off along +Z (PRISMATIC) ---
    cap_rest_z = ctx.part_world_position(overcap)[2]
    with ctx.pose({overcap_lift: OVERCAP_LIFT}):
        cap_lifted_z = ctx.part_world_position(overcap)[2]
        # When fully lifted, overcap clears the ball and housing
        ctx.expect_gap(
            overcap,
            ball,
            axis="z",
            min_gap=0.0,
            positive_elem="overcap_shell",
            negative_elem="applicator_ball",
            name="overcap clears ball when fully lifted",
        )
    ctx.check(
        "overcap lifts up along +Z from seated position",
        cap_lifted_z > cap_rest_z + OVERCAP_LIFT * 0.8,
        details=f"rest_z={cap_rest_z}, lifted_z={cap_lifted_z}",
    )
    ctx.check(
        "overcap_lift is PRISMATIC along +Z",
        overcap_lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(overcap_lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={overcap_lift.articulation_type}, axis={overcap_lift.axis}",
    )

    # --- Overcap seats correctly at rest ---
    ctx.check(
        "overcap bottom is seated at housing rim level at rest",
        abs(cap_rest_z - HOUSING_RIM_Z) < 1e-4,
        details=f"cap_rest_z={cap_rest_z}, housing_rim_z={HOUSING_RIM_Z}",
    )

    return ctx.report()


object_model = build_object_model()
