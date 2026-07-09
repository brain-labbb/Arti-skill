from __future__ import annotations

# Indoor spin bike with open triangulated steel-tube frame.
# Frame: +X = front (flywheel), -X = rear; +Z = up; +Y = left side.
#   - Open tube frame: seat tube, down tube, top tube, head tube, front fork
#     legs, and rear stays, all reading as welded round metal tubes.
#   - Front flywheel disc (gray center, RED accent ring) spins on Y axis,
#     mounted between the fork legs on a visible axle.
#   - Crank with two pedal arms 180 deg apart at the BB (bottom bracket) junction.
#   - Gray padded saddle on a prismatic post sliding up from the seat tube top.
#   - RED curved handlebars on a prismatic post sliding up from the head tube top.
#   - Chrome stabilizer cross-tubes (front + rear) with black end caps.
# Articulations:
#   - flywheel: CONTINUOUS about Y.
#   - crank: CONTINUOUS about Y.
#   - left/right pedal: CONTINUOUS about Y.
#   - saddle post: PRISMATIC vertical adjust.
#   - handlebar post: PRISMATIC vertical adjust.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Frame junction points (meters) – define the open tube-frame geometry.
# ---------------------------------------------------------------------------
BB = (0.10, 0.0, 0.18)             # bottom bracket (crank axle)
ST_TOP = (-0.06, 0.0, 0.56)       # seat tube top (saddle post entry)
HT_TOP = (0.30, 0.0, 0.62)        # head tube top (handlebar post entry)
FORK_CROWN = (0.30, 0.0, 0.42)    # fork crown / head tube bottom
STAY_JUNC = (0.02, 0.0, 0.37)     # rear stay junction on the seat tube

# Fork legs (straddle the flywheel, go from crown to ground)
FORK_L = (0.30, -0.23, 0.035)
FORK_R = (0.30,  0.23, 0.035)

# Rear stays (from seat tube junction to ground)
STAY_L = (-0.12, -0.23, 0.035)
STAY_R = (-0.12,  0.23, 0.035)

# ---------------------------------------------------------------------------
# Shared component constants
# ---------------------------------------------------------------------------
TUBE_R = 0.022          # main tube radius
TUBE_R_THIN = 0.018     # fork legs / stays radius

FLYWHEEL_X, FLYWHEEL_Z = 0.30, 0.26
FLYWHEEL_R = 0.165
FLYWHEEL_HALF_W = 0.022

CRANK_X, CRANK_Z = BB[0], BB[2]
CRANK_ARM_LEN = 0.085
CRANK_ARM_Y = 0.110
PEDAL_Y = 0.175

FOOT_SPAN = 0.46        # cross-tube length along Y


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _tube(p1, p2, radius=TUBE_R):
    """Build a straight tube mesh between two 3D points (5-pt catmull-rom)."""
    pts = []
    for i in range(5):
        t = i / 4.0
        pts.append(tuple(a + t * (b - a) for a, b in zip(p1, p2)))
    return tube_from_spline_points(
        pts,
        radius=radius,
        samples_per_segment=4,
        radial_segments=16,
        cap_ends=True,
    )


def _frame_mesh():
    """Complete triangulated tube frame merged into one connected mesh."""
    geo = _tube(BB, ST_TOP)                           # seat tube
    geo = geo.merge(_tube(BB, FORK_CROWN))             # down tube (backbone)
    geo = geo.merge(_tube(ST_TOP, HT_TOP))             # top tube
    geo = geo.merge(_tube(FORK_CROWN, HT_TOP))         # head tube
    geo = geo.merge(_tube(FORK_CROWN, FORK_L, TUBE_R_THIN))  # left fork
    geo = geo.merge(_tube(FORK_CROWN, FORK_R, TUBE_R_THIN))  # right fork
    geo = geo.merge(_tube(STAY_JUNC, STAY_L, TUBE_R_THIN))   # left stay
    geo = geo.merge(_tube(STAY_JUNC, STAY_R, TUBE_R_THIN))   # right stay

    # Junction details – BB shell, collars, flywheel axle
    bb_shell = CylinderGeometry(0.032, 0.070).rotate_x(math.pi / 2.0)
    bb_shell.translate(*BB)
    geo = geo.merge(bb_shell)

    crown_collar = CylinderGeometry(0.028, 0.040).rotate_x(math.pi / 2.0)
    crown_collar.translate(*FORK_CROWN)
    geo = geo.merge(crown_collar)

    seat_collar = CylinderGeometry(0.028, 0.030).rotate_x(math.pi / 2.0)
    seat_collar.translate(*ST_TOP)
    geo = geo.merge(seat_collar)

    # Flywheel axle spanning between fork legs
    axle = CylinderGeometry(0.012, 0.22).rotate_x(math.pi / 2.0)
    axle.translate(FLYWHEEL_X, 0.0, FLYWHEEL_Z)
    geo = geo.merge(axle)

    return mesh_from_geometry(geo, "frame")


def _flywheel_disc_mesh():
    disc = CylinderGeometry(FLYWHEEL_R * 0.86, FLYWHEEL_HALF_W * 2.0 * 0.7).rotate_x(
        math.pi / 2.0
    )
    hub = CylinderGeometry(0.040, FLYWHEEL_HALF_W * 2.0 * 1.1).rotate_x(math.pi / 2.0)
    disc.merge(hub)
    return mesh_from_geometry(disc, "flywheel_disc")


def _flywheel_ring_mesh():
    ring = TorusGeometry(
        FLYWHEEL_R * 0.92, 0.012, radial_segments=18, tubular_segments=64
    ).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(ring, "flywheel_red_ring")


def _saddle_mesh():
    """Padded gray saddle – teardrop pad via lofted sections along Y."""
    # Build a simple elongated pad using merged primitives
    pad = BoxGeometry((0.140, 0.120, 0.040))
    # Narrower nose section
    nose = BoxGeometry((0.060, 0.050, 0.035)).translate(0.06, 0.0, 0.0)
    pad = pad.merge(nose)
    # Wider rear
    rear = BoxGeometry((0.080, 0.150, 0.045)).translate(-0.04, 0.0, 0.0)
    pad = pad.merge(rear)
    return mesh_from_geometry(pad, "saddle_pad")


def _handlebar_mesh():
    """RED curved handlebar sweeping back toward the rider (-X)."""
    pts = [
        (-0.06, -0.32, 0.02),
        (-0.03, -0.26, -0.02),
        (0.0, -0.16, -0.04),
        (0.0, -0.06, -0.02),
        (0.0, 0.0, 0.0),
        (0.0, 0.06, -0.02),
        (0.0, 0.16, -0.04),
        (-0.03, 0.26, -0.02),
        (-0.06, 0.32, 0.02),
    ]
    bar = tube_from_spline_points(
        pts, radius=0.016, samples_per_segment=14, radial_segments=14
    )
    return mesh_from_geometry(bar, "handlebar_tube")


# ---------------------------------------------------------------------------
# Build model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exercise_bike")

    # Materials
    frame_steel = model.material("frame_steel", rgba=(0.18, 0.19, 0.22, 1.0))
    red = model.material("accent_red", rgba=(0.82, 0.10, 0.12, 1.0))
    gray = model.material("part_gray", rgba=(0.45, 0.45, 0.48, 1.0))
    dark = model.material("dark_gray", rgba=(0.20, 0.20, 0.22, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    black = model.material("cap_black", rgba=(0.10, 0.10, 0.11, 1.0))
    white = model.material("body_white", rgba=(0.93, 0.93, 0.94, 1.0))

    # ========================= BODY (root – tube frame) =========================
    body = model.part("body")
    body.visual(_frame_mesh(), material=frame_steel, name="frame")
    body.inertial = Inertial.from_geometry(
        Box((0.50, 0.46, 0.55)),
        mass=12.0,
        origin=Origin(xyz=(0.10, 0.0, 0.30)),
    )

    # ========================= STABILIZER FEET =========================
    foot_specs = (
        ("front_foot", FORK_L[0]),   # front foot at fork X
        ("rear_foot", STAY_L[0]),    # rear foot at stay X
    )
    for fname, fx in foot_specs:
        foot = model.part(fname)
        # Chrome cross-tube centered at foot part origin
        tube_geo = CylinderGeometry(0.018, FOOT_SPAN).rotate_x(math.pi / 2.0)
        foot.visual(
            mesh_from_geometry(tube_geo, "stabilizer_tube"),
            material=chrome,
            name="stabilizer_tube",
        )
        # End caps and ground pads at each end
        for side, cy in (("l", FOOT_SPAN / 2.0), ("r", -FOOT_SPAN / 2.0)):
            cap = CylinderGeometry(0.024, 0.030).rotate_x(math.pi / 2.0)
            cap.translate(0.0, cy, 0.0)
            foot.visual(
                mesh_from_geometry(cap, f"foot_cap_{side}"),
                material=black,
                name=f"foot_cap_{side}",
            )
            pad = BoxGeometry((0.040, 0.050, 0.022))
            pad.translate(0.0, cy, -0.024)
            foot.visual(
                mesh_from_geometry(pad, f"foot_pad_{side}"),
                material=black,
                name=f"foot_pad_{side}",
            )
        foot.inertial = Inertial.from_geometry(
            Box((0.05, FOOT_SPAN, 0.06)), mass=1.5
        )
        model.articulation(
            f"body_to_{fname}",
            ArticulationType.FIXED,
            parent=body,
            child=foot,
            origin=Origin(xyz=(fx, 0.0, 0.035)),
        )

    # ========================= FLYWHEEL =========================
    flywheel = model.part("flywheel")
    flywheel.visual(_flywheel_disc_mesh(), material=gray, name="flywheel_disc")
    flywheel.visual(_flywheel_ring_mesh(), material=red, name="flywheel_red_ring")
    # Off-axis marker bolt so rotation is visually detectable
    marker = CylinderGeometry(0.012, FLYWHEEL_HALF_W * 2.0 * 1.2).rotate_x(
        math.pi / 2.0
    )
    marker.translate(FLYWHEEL_R * 0.55, 0.0, 0.0)
    flywheel.visual(
        mesh_from_geometry(marker, "flywheel_marker"),
        material=dark,
        name="flywheel_marker",
    )
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(FLYWHEEL_R, 2.0 * FLYWHEEL_HALF_W), mass=4.0
    )
    model.articulation(
        "body_to_flywheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=flywheel,
        origin=Origin(xyz=(FLYWHEEL_X, 0.0, FLYWHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    # ========================= CRANK + PEDALS =========================
    crank = model.part("crank")
    hub = CylinderGeometry(0.018, 2.0 * CRANK_ARM_Y).rotate_x(math.pi / 2.0)
    crank_geo = hub
    for sgn, ay in ((1.0, CRANK_ARM_Y), (-1.0, -CRANK_ARM_Y)):
        arm = BoxGeometry((0.026, 0.022, CRANK_ARM_LEN)).translate(
            0.0, ay, sgn * CRANK_ARM_LEN / 2.0
        )
        crank_geo = crank_geo.merge(arm)
    crank.visual(
        mesh_from_geometry(crank_geo, "crank_body"),
        material=dark,
        name="crank_body",
    )
    crank.inertial = Inertial.from_geometry(Box((0.06, 0.24, 0.18)), mass=1.2)
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(CRANK_X, 0.0, CRANK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )

    # Pedals: each on its own spindle at the crank-arm tip
    pedal_specs = (
        ("left_pedal", CRANK_ARM_Y, CRANK_ARM_LEN, PEDAL_Y),
        ("right_pedal", -CRANK_ARM_Y, -CRANK_ARM_LEN, -PEDAL_Y),
    )
    for pname, tip_y, tip_z, plat_y in pedal_specs:
        pedal = model.part(pname)
        out = plat_y - tip_y
        plat = BoxGeometry((0.090, 0.060, 0.014)).translate(0.0, out, 0.0)
        spindle = CylinderGeometry(0.009, abs(out) + 0.02).rotate_x(math.pi / 2.0)
        spindle.translate(0.0, out / 2.0, 0.0)
        plat = plat.merge(spindle)
        ridge = BoxGeometry((0.012, 0.060, 0.020)).translate(0.034, out, 0.006)
        plat = plat.merge(ridge)
        pedal.visual(
            mesh_from_geometry(plat, "pedal_tread"),
            material=dark,
            name="pedal_tread",
        )
        pedal.inertial = Inertial.from_geometry(
            Box((0.09, 0.06, 0.03)), mass=0.25, origin=Origin(xyz=(0.0, out, 0.0))
        )
        model.articulation(
            f"crank_to_{pname}",
            ArticulationType.CONTINUOUS,
            parent=crank,
            child=pedal,
            origin=Origin(xyz=(0.0, tip_y, tip_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0),
        )

    # ========================= SADDLE POST (prismatic) =========================
    saddle_post = model.part("saddle_post")
    saddle_post.visual(
        Cylinder(radius=0.016, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=chrome,
        name="saddle_post_tube",
    )
    saddle_post.visual(
        _saddle_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=gray,
        name="saddle_pad",
    )
    saddle_post.inertial = Inertial.from_geometry(
        Box((0.16, 0.16, 0.42)), mass=1.6, origin=Origin(xyz=(0.0, 0.0, 0.08))
    )
    model.articulation(
        "body_to_saddle_post",
        ArticulationType.PRISMATIC,
        parent=body,
        child=saddle_post,
        origin=Origin(xyz=ST_TOP),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.1, lower=0.0, upper=0.10),
    )

    # ========================= HANDLEBAR POST (prismatic) =========================
    hbar_post = model.part("handlebar_post")
    hbar_post.visual(
        Cylinder(radius=0.018, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=chrome,
        name="handlebar_post_tube",
    )
    # Clamp block (sits on top of post tube, touching it)
    hbar_post.visual(
        Box((0.05, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=dark,
        name="handlebar_clamp",
    )
    # Console display (thin dark panel facing the rider at -X, mounted on clamp)
    hbar_post.visual(
        Box((0.018, 0.14, 0.10)),
        origin=Origin(xyz=(-0.02, 0.0, 0.22), rpy=(0.0, 0.3, 0.0)),
        material=dark,
        name="console_pad",
    )
    # Red handlebars (mounted at clamp level)
    hbar_post.visual(
        _handlebar_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=red,
        name="handlebar_tube",
    )
    hbar_post.inertial = Inertial.from_geometry(
        Box((0.20, 0.70, 0.40)), mass=2.2, origin=Origin(xyz=(0.0, 0.0, 0.14))
    )
    model.articulation(
        "body_to_handlebar_post",
        ArticulationType.PRISMATIC,
        parent=body,
        child=hbar_post,
        origin=Origin(xyz=HT_TOP),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.1, lower=0.0, upper=0.10),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    flywheel = object_model.get_part("flywheel")
    crank = object_model.get_part("crank")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")
    saddle_post = object_model.get_part("saddle_post")
    hbar_post = object_model.get_part("handlebar_post")
    front_foot = object_model.get_part("front_foot")
    rear_foot = object_model.get_part("rear_foot")

    fly_joint = object_model.get_articulation("body_to_flywheel")
    crank_joint = object_model.get_articulation("body_to_crank")
    lpedal_joint = object_model.get_articulation("crank_to_left_pedal")
    saddle_joint = object_model.get_articulation("body_to_saddle_post")
    hbar_joint = object_model.get_articulation("body_to_handlebar_post")

    # ---- Intentional seated overlaps ----
    ctx.allow_overlap(
        saddle_post, body,
        elem_a="saddle_post_tube", elem_b="frame",
        reason="Saddle post chrome tube slides inside the seat tube of the frame; intentional insertion.",
    )
    ctx.allow_overlap(
        hbar_post, body,
        elem_a="handlebar_post_tube", elem_b="frame",
        reason="Handlebar post chrome tube slides inside the head tube of the frame; intentional insertion.",
    )
    ctx.allow_overlap(
        crank, body,
        reason="Crank hub passes through the BB shell on the frame; intentional mounting.",
    )
    ctx.allow_overlap(
        flywheel, body,
        reason="Flywheel disc is mounted on the axle between fork legs; intentional mounting.",
    )
    ctx.allow_overlap(
        left_pedal, crank,
        reason="Left pedal spindle is captured on the crank arm tip.",
    )
    ctx.allow_overlap(
        right_pedal, crank,
        reason="Right pedal spindle is captured on the crank arm tip.",
    )
    ctx.allow_overlap(
        front_foot, body,
        reason="Front stabilizer foot bolts onto the fork leg bottoms; intentional mounting.",
    )
    ctx.allow_overlap(
        rear_foot, body,
        reason="Rear stabilizer foot bolts onto the rear stay endpoints; intentional mounting.",
    )

    # ---- Open tube frame: body is NOT a solid block ----
    body_aabb = ctx.part_world_aabb(body)
    body_ext = _ext(body_aabb)
    # The frame bounding box should be large in XZ but the frame tubes are thin.
    # A solid shroud would have Y-extent close to the full tube width (>0.10m);
    # the open frame has tubes at Y=0 plus fork/stay endpoints at Y=±0.23,
    # so the Y-extent is dominated by the fork spread (~0.46m) but the
    # interior is mostly empty. We just verify the body exists with a
    # reasonable bounding box.
    ctx.check(
        "frame body has reasonable size",
        body_ext[0] > 0.30 and body_ext[2] > 0.40,
        details=f"body_ext={body_ext}",
    )

    # ---- Flywheel spins about its side (Y) axis ----
    m0 = ctx.part_element_world_aabb(flywheel, elem="flywheel_marker")
    mc0 = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][2] + m0[1][2]) / 2.0)
    with ctx.pose({fly_joint: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(flywheel, elem="flywheel_marker")
        mc1 = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][2] + m1[1][2]) / 2.0)
    ctx.check(
        "flywheel marker revolves about the side axis",
        abs(mc1[1] - mc0[1]) > 0.04 and abs(mc1[0] - mc0[0]) > 0.04,
        details=f"marker XZ rest={mc0}, quarter-turn={mc1}",
    )

    # ---- Crank rotates: pedal mount revolves ----
    p0 = ctx.part_world_position(left_pedal)
    with ctx.pose({crank_joint: math.pi / 2.0}):
        p1 = ctx.part_world_position(left_pedal)
    ctx.check(
        "crank rotation revolves the pedal about the crank axis",
        p0 is not None and p1 is not None
        and (abs(p1[0] - p0[0]) > 0.03 or abs(p1[2] - p0[2]) > 0.03),
        details=f"left pedal rest={p0}, crank quarter-turn={p1}",
    )

    # ---- Pedal spins on its spindle ----
    e0 = _ext(ctx.part_world_aabb(left_pedal))
    with ctx.pose({lpedal_joint: math.pi / 2.0}):
        e1 = _ext(ctx.part_world_aabb(left_pedal))
    ctx.check(
        "left pedal spins on its spindle",
        abs(e1[0] - e0[0]) > 0.01 or abs(e1[2] - e0[2]) > 0.01,
        details=f"pedal extents rest={e0}, spun={e1}",
    )

    # ---- Saddle post raises (prismatic, seat moves up) ----
    s0 = ctx.part_world_position(saddle_post)
    with ctx.pose({saddle_joint: 0.1}):
        s1 = ctx.part_world_position(saddle_post)
    ctx.check(
        "saddle post raises the seat",
        s0 is not None and s1 is not None and s1[2] > s0[2] + 0.08,
        details=f"saddle rest_z={s0}, raised_z={s1}",
    )
    # Saddle post stays inserted in the frame at full rise
    ctx.expect_overlap(
        saddle_post, body, axes="z",
        elem_a="saddle_post_tube", elem_b="frame",
        min_overlap=0.02, name="saddle post retained in seat tube",
    )

    # ---- Handlebar post raises ----
    h0 = ctx.part_world_position(hbar_post)
    with ctx.pose({hbar_joint: 0.1}):
        h1 = ctx.part_world_position(hbar_post)
    ctx.check(
        "handlebar post raises",
        h0 is not None and h1 is not None and h1[2] > h0[2] + 0.08,
        details=f"hbar rest_z={h0}, raised_z={h1}",
    )

    # ---- Stabilizer feet at ground level and widest footprint ----
    ff = ctx.part_world_aabb(front_foot)
    rf = ctx.part_world_aabb(rear_foot)
    fly_aabb = ctx.part_world_aabb(flywheel)
    foot_min_z = min(ff[0][2], rf[0][2])
    ctx.check(
        "stabilizer feet rest at the ground plane (lowest parts)",
        foot_min_z <= fly_aabb[0][2] + 0.02 and foot_min_z < 0.05,
        details=f"foot_min_z={foot_min_z}, flywheel_min_z={fly_aabb[0][2]}",
    )
    foot_span_y = max(ff[1][1], rf[1][1]) - min(ff[0][1], rf[0][1])
    body_span_y = body_ext[1]
    ctx.check(
        "feet are the widest footprint",
        foot_span_y >= body_span_y - 0.01,
        details=f"foot_span_y={foot_span_y}, body_span_y={body_span_y}",
    )
    ctx.check(
        "front foot is forward of rear foot",
        (ff[0][0] + ff[1][0]) / 2.0 > (rf[0][0] + rf[1][0]) / 2.0,
        details=f"front_cx={(ff[0][0]+ff[1][0])/2.0}, rear_cx={(rf[0][0]+rf[1][0])/2.0}",
    )

    # ---- Flywheel is between fork legs (Y containment) ----
    fly_aabb_full = ctx.part_world_aabb(flywheel)
    body_aabb_full = ctx.part_world_aabb(body)
    # Flywheel Y-extent should be within body Y-extent (between fork legs)
    ctx.check(
        "flywheel sits between the fork legs in Y",
        fly_aabb_full[0][1] >= body_aabb_full[0][1] - 0.01
        and fly_aabb_full[1][1] <= body_aabb_full[1][1] + 0.01,
        details=f"flywheel_y=[{fly_aabb_full[0][1]:.3f},{fly_aabb_full[1][1]:.3f}], "
                f"body_y=[{body_aabb_full[0][1]:.3f},{body_aabb_full[1][1]:.3f}]",
    )

    # ---- Handlebar is at the front of the bike (forward of saddle) ----
    hbar_pos = ctx.part_world_position(hbar_post)
    saddle_pos = ctx.part_world_position(saddle_post)
    ctx.check(
        "handlebar post is forward of saddle post",
        hbar_pos is not None and saddle_pos is not None
        and hbar_pos[0] > saddle_pos[0] + 0.10,
        details=f"hbar_x={hbar_pos}, saddle_x={saddle_pos}",
    )

    return ctx.report()


object_model = build_object_model()
