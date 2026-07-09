from __future__ import annotations

# White upright stationary exercise bike.
# Frame: +X = front (flywheel housing), -X = rear; +Z = up; +Y = left side.
#   - The molded white body sits low at the front and rises toward the rear.
#   - The front flywheel disc (gray center, RED accent ring) spins about the
#     horizontal side axis (Y).
#   - A gray padded saddle on a post rises out of the body (rear-upper).
#   - RED curved handlebars on a vertical post with a small console pad rise
#     at the rear of the body.
#   - A crank with two pedal arms 180 deg apart spins about the crank axis (Y),
#     low at the front; each pedal spins on its own spindle.
#   - Chrome stabilizer cross-tubes (front + rear) with black end caps form the
#     widest footprint on the ground.
# Articulations:
#   - flywheel: CONTINUOUS about its side axis (Y).
#   - crank: CONTINUOUS about the crank axis (Y).
#   - left/right pedal: each CONTINUOUS about its pedal spindle (Y).
#   - saddle post: PRISMATIC vertical adjust (~0.1 m).
#   - handlebar post: PRISMATIC vertical adjust (~0.1 m).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- Key reference coordinates (meters) ----
FLYWHEEL_X = 0.30          # front flywheel center, X
FLYWHEEL_Z = 0.30          # flywheel center height
FLYWHEEL_R = 0.165         # flywheel outer radius
FLYWHEEL_HALF_W = 0.022    # half thickness of the disc along Y
FLYWHEEL_Y = 0.100         # flywheel sits outboard on the +Y side (disc meets body)

CRANK_X = 0.10             # crank axle X (low and behind the flywheel)
CRANK_Z = 0.120            # crank axle height
CRANK_ARM_LEN = 0.085      # crank arm length (axle -> pedal spindle)
CRANK_ARM_Y = 0.110        # crank arm offset along Y (outboard of the body face)
PEDAL_Y = 0.175            # pedal platform offset along Y (outboard)

SADDLE_BASE_Z = 0.50       # top of body where saddle post emerges
HBAR_BASE_Z = 0.46         # top of body region where handlebar post emerges


def _loft(sections) -> cq.Workplane:
    # sections along +Y (XZ planes): ("rect", y, w, h) or ("circle", y, r).
    wp = cq.Workplane("XZ")
    prev = 0.0
    for i, s in enumerate(sections):
        y = s[1]
        off = y if i == 0 else y - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        if s[0] == "circle":
            wp = wp.circle(s[2])
        else:
            wp = wp.rect(s[2], s[3])
        prev = y
    return wp.loft(ruled=False)


def _body_solid() -> cq.Workplane:
    # Molded white shroud: a tall teardrop side profile (front bulge around the
    # flywheel, narrowing up and back to the post column), lofted across Y so it
    # reads as a rounded molded plastic body rather than a slab.
    # Side profile in the XZ plane (extruded thickness controlled by the loft).
    profile_pts = [
        (0.46, 0.22),   # front-top of the flywheel cowl
        (0.47, 0.34),
        (0.42, 0.45),
        (0.30, 0.49),
        (0.10, 0.50),   # rises toward the rear post column
        (-0.05, 0.50),
        (-0.14, 0.46),
        (-0.16, 0.34),
        (-0.12, 0.22),
        (-0.04, 0.14),
        (0.10, 0.11),   # bottom near the crank
        (0.26, 0.12),
        (0.40, 0.15),
    ]
    wire = (
        cq.Workplane("XZ")
        .moveTo(*profile_pts[0])
    )
    for p in profile_pts[1:]:
        wire = wire.lineTo(*p)
    base = wire.close().extrude(0.085, both=True)  # thickness along Y (+/-0.085)
    # Round the molded sides by lofting a thinner upper section is complex; keep
    # the extruded teardrop and fillet the vertical edges for a softer molded read.
    try:
        base = base.edges("|Y").fillet(0.03)
    except Exception:
        pass
    return base


def _flywheel_mesh():
    # Flywheel disc: gray dished center plate.
    disc = CylinderGeometry(FLYWHEEL_R * 0.86, FLYWHEEL_HALF_W * 2.0 * 0.7).rotate_x(
        math.pi / 2.0
    )
    # Raised hub boss in the center.
    hub = CylinderGeometry(0.040, FLYWHEEL_HALF_W * 2.0 * 1.1).rotate_x(math.pi / 2.0)
    disc.merge(hub)
    return mesh_from_geometry(disc, "flywheel_disc")


def _flywheel_ring_mesh():
    # RED accent ring around the disc rim.
    ring = TorusGeometry(
        FLYWHEEL_R * 0.92, 0.012, radial_segments=18, tubular_segments=64
    ).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(ring, "flywheel_red_ring")


def _saddle_mesh():
    # Padded gray saddle: a teardrop pad, wide at the rear, tapering to a nose.
    pad = _loft(
        [
            ("rect", -0.075, 0.060, 0.045),  # left edge near nose
            ("rect", -0.030, 0.110, 0.060),
            ("rect", 0.030, 0.150, 0.060),
            ("rect", 0.075, 0.130, 0.050),   # rear is wide
        ]
    )
    return mesh_from_cadquery(pad, "saddle_pad")


def _handlebar_mesh():
    # RED curved handlebars: a central bar that sweeps up and out to two grips,
    # modeled as one swept tube (symmetric U / ram-horn shape).
    pts = [
        (0.0, -0.34, 0.02),    # left grip tip (forward + out)
        (0.04, -0.28, -0.02),
        (0.05, -0.16, -0.05),
        (0.02, -0.06, -0.03),
        (0.0, 0.0, 0.0),       # center clamp
        (0.02, 0.06, -0.03),
        (0.05, 0.16, -0.05),
        (0.04, 0.28, -0.02),
        (0.0, 0.34, 0.02),     # right grip tip
    ]
    bar = tube_from_spline_points(
        pts, radius=0.016, samples_per_segment=14, radial_segments=14
    )
    return mesh_from_geometry(bar, "handlebar_tube")


def _foot_mesh(length_y: float):
    # Chrome stabilizer cross-tube along Y with black end caps.
    tube = CylinderGeometry(0.018, length_y).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(tube, "stabilizer_tube")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exercise_bike")

    white = model.material("body_white", rgba=(0.93, 0.93, 0.94, 1.0))
    red = model.material("accent_red", rgba=(0.82, 0.10, 0.12, 1.0))
    gray = model.material("part_gray", rgba=(0.45, 0.45, 0.48, 1.0))
    dark = model.material("dark_gray", rgba=(0.20, 0.20, 0.22, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    black = model.material("cap_black", rgba=(0.10, 0.10, 0.11, 1.0))

    # ========================= BODY (root) =========================
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shroud"),
        material=white,
        name="body_shroud",
    )

    # Crank axle boss: a dark hub band where the crank seats. Built as one mesh
    # spanning the body width so it is a single connected element on the body.
    boss = CylinderGeometry(0.030, 0.20).rotate_x(math.pi / 2.0)
    boss.translate(CRANK_X, 0.0, CRANK_Z)
    body.visual(
        mesh_from_geometry(boss, "crank_boss"),
        material=dark,
        name="crank_boss",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.62, 0.18, 0.40)),
        mass=14.0,
        origin=Origin(xyz=(0.16, 0.0, 0.30)),
    )

    # ========================= STABILIZER FEET =========================
    # Front + rear chrome cross-tubes (widest footprint), each connected up to
    # the body by a chrome down-leg, with two black end caps resting on ground.
    # body_bottom_z gives the height the down-leg must reach up to.
    foot_specs = (
        ("front_foot", 0.34, 0.46, 0.155),   # leg rises into the body front
        ("rear_foot", -0.12, 0.46, 0.225),   # leg rises into the body rear
    )
    for fname, fx, flen, leg_top_z in foot_specs:
        foot = model.part(fname)
        # Cross tube on the ground (center at z=0.022).
        foot.visual(
            _foot_mesh(flen),
            origin=Origin(xyz=(fx, 0.0, 0.022)),
            material=chrome,
            name="stabilizer_tube",
        )
        # Vertical down-leg connecting the cross tube up into the body shroud.
        leg = CylinderGeometry(0.020, leg_top_z + 0.05)
        leg.translate(fx, 0.0, (leg_top_z + 0.05) / 2.0)
        foot.visual(
            mesh_from_geometry(leg, "foot_leg"),
            material=chrome,
            name="foot_leg",
        )
        # Black end caps + small ground pads.
        for cy in (flen / 2.0, -flen / 2.0):
            cap = CylinderGeometry(0.024, 0.030).rotate_x(math.pi / 2.0)
            cap.translate(fx, cy - math.copysign(0.012, cy), 0.022)
            foot.visual(
                mesh_from_geometry(cap, f"foot_cap_{'l' if cy > 0 else 'r'}"),
                material=black,
                name=f"foot_cap_{'l' if cy > 0 else 'r'}",
            )
            pad = BoxGeometry((0.040, 0.050, 0.022)).translate(
                fx, cy - math.copysign(0.006, cy), 0.011
            )
            foot.visual(
                mesh_from_geometry(pad, f"foot_pad_{'l' if cy > 0 else 'r'}"),
                material=black,
                name=f"foot_pad_{'l' if cy > 0 else 'r'}",
            )
        foot.inertial = Inertial.from_geometry(
            Box((0.05, flen, leg_top_z)), mass=1.5, origin=Origin(xyz=(fx, 0.0, leg_top_z / 2.0))
        )
        model.articulation(
            f"body_to_{fname}",
            ArticulationType.FIXED,
            parent=body,
            child=foot,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

    # ========================= FLYWHEEL =========================
    flywheel = model.part("flywheel")
    flywheel.visual(_flywheel_mesh(), material=gray, name="flywheel_disc")
    flywheel.visual(_flywheel_ring_mesh(), material=red, name="flywheel_red_ring")
    # Off-axis marker (small bolt) so rotation is detectable.
    marker = CylinderGeometry(0.012, FLYWHEEL_HALF_W * 2.0 * 1.2).rotate_x(math.pi / 2.0)
    marker.translate(FLYWHEEL_R * 0.55, 0.0, 0.0)
    flywheel.visual(mesh_from_geometry(marker, "flywheel_marker"), material=dark, name="flywheel_marker")
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(FLYWHEEL_R, 2.0 * FLYWHEEL_HALF_W),
        mass=4.0,
    )
    # Flywheel center is in front and slightly inside the body front face (+Y side).
    model.articulation(
        "body_to_flywheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=flywheel,
        origin=Origin(xyz=(FLYWHEEL_X, FLYWHEEL_Y, FLYWHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    # ========================= CRANK + PEDALS =========================
    # Crank: a hub with two arms 180 deg apart along the crank axis (Y).
    crank = model.part("crank")
    hub = CylinderGeometry(0.018, 2.0 * CRANK_ARM_Y).rotate_x(math.pi / 2.0)  # axle across body
    crank_geo = hub
    # Two arms 180 deg apart, mounted at the outer ends of the hub.
    for sgn, ay in ((1.0, CRANK_ARM_Y), (-1.0, -CRANK_ARM_Y)):
        arm = BoxGeometry((0.026, 0.022, CRANK_ARM_LEN)).translate(
            0.0, ay, sgn * CRANK_ARM_LEN / 2.0
        )
        crank_geo = crank_geo.merge(arm)
    crank.visual(mesh_from_geometry(crank_geo, "crank_body"), material=dark, name="crank_body")
    crank.inertial = Inertial.from_geometry(
        Box((0.06, 0.24, 0.18)), mass=1.2
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(CRANK_X, 0.0, CRANK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )

    # Pedals: each on its own spindle at the end of a crank arm, 180 deg apart.
    # Left pedal arm points +Z from the axle, right pedal arm points -Z.
    # Each pedal mounts at a crank-arm tip. Tip Y = +/-CRANK_ARM_Y, tip Z =
    # +/-CRANK_ARM_LEN (arms 180 deg apart). The pedal platform sits outboard at
    # PEDAL_Y; a short spindle reaches inboard to the arm tip.
    pedal_specs = (
        ("left_pedal", CRANK_ARM_Y, CRANK_ARM_LEN, PEDAL_Y),
        ("right_pedal", -CRANK_ARM_Y, -CRANK_ARM_LEN, -PEDAL_Y),
    )
    for pname, tip_y, tip_z, plat_y in pedal_specs:
        pedal = model.part(pname)
        out = plat_y - tip_y  # outboard offset of platform from the joint (arm tip)
        # Pedal platform (flat tread) outboard of the spindle.
        plat = BoxGeometry((0.090, 0.060, 0.014)).translate(0.0, out, 0.0)
        # Spindle from the arm tip (y=0 in pedal frame) out to the platform.
        spindle = CylinderGeometry(0.009, abs(out) + 0.02).rotate_x(math.pi / 2.0)
        spindle.translate(0.0, out / 2.0, 0.0)
        plat = plat.merge(spindle)
        # Off-axis marker ridge so pedal spin is detectable.
        ridge = BoxGeometry((0.012, 0.060, 0.020)).translate(0.034, out, 0.006)
        plat = plat.merge(ridge)
        pedal.visual(mesh_from_geometry(plat, "pedal_tread"), material=dark, name="pedal_tread")
        pedal.inertial = Inertial.from_geometry(
            Box((0.09, 0.06, 0.03)), mass=0.25, origin=Origin(xyz=(0.0, out, 0.0))
        )
        # Mount on the crank arm tip: crank frame origin + arm vector.
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
    # A chrome post that slides vertically out of the body, carrying the saddle.
    saddle_post = model.part("saddle_post")
    # Post extends well below the seating plane so it stays inserted at full rise.
    saddle_post.visual(
        Cylinder(radius=0.018, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=chrome,
        name="saddle_post_tube",
    )
    # Gray padded saddle on top of the post.
    saddle_post.visual(
        _saddle_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.16), rpy=(0.0, 0.0, 0.0)),
        material=gray,
        name="saddle_pad",
    )
    saddle_post.inertial = Inertial.from_geometry(
        Box((0.16, 0.16, 0.40)), mass=1.6, origin=Origin(xyz=(0.0, 0.0, 0.08))
    )
    model.articulation(
        "body_to_saddle_post",
        ArticulationType.PRISMATIC,
        parent=body,
        child=saddle_post,
        # Joint at the body's saddle-tube mouth (forward-upper of the column).
        origin=Origin(xyz=(0.06, 0.0, SADDLE_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.1, lower=0.0, upper=0.1),
    )

    # ========================= HANDLEBAR POST (prismatic) =========================
    # A chrome post sliding vertically out of the body front-upper column,
    # carrying the console pad + red handlebars.
    hbar_post = model.part("handlebar_post")
    hbar_post.visual(
        Cylinder(radius=0.020, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=chrome,
        name="handlebar_post_tube",
    )
    # Console pad (white tablet) high up, tilted to face the rider (+X).
    hbar_post.visual(
        Box((0.16, 0.13, 0.024)),
        origin=Origin(xyz=(0.085, 0.0, 0.34), rpy=(0.0, -0.6, 0.0)),
        material=white,
        name="console_pad",
    )
    # Clamp block where the handlebar tube mounts.
    hbar_post.visual(
        Box((0.05, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=dark,
        name="handlebar_clamp",
    )
    # Red curved handlebars mounted above the clamp.
    hbar_post.visual(
        _handlebar_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=red,
        name="handlebar_tube",
    )
    hbar_post.inertial = Inertial.from_geometry(
        Box((0.30, 0.70, 0.60)), mass=2.2, origin=Origin(xyz=(0.04, 0.0, 0.24))
    )
    model.articulation(
        "body_to_handlebar_post",
        ArticulationType.PRISMATIC,
        parent=body,
        child=hbar_post,
        origin=Origin(xyz=(-0.10, 0.0, HBAR_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.1, lower=0.0, upper=0.1),
    )

    return model


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

    # ---- Intentional seated overlaps (post-in-frame, crank-on-boss, flywheel) ----
    ctx.allow_overlap(
        saddle_post, body,
        elem_a="saddle_post_tube", elem_b="body_shroud",
        reason="Saddle post is a chrome tube sliding inside the body's seat-tube; it is intentionally inserted.",
    )
    ctx.allow_overlap(
        hbar_post, body,
        elem_a="handlebar_post_tube", elem_b="body_shroud",
        reason="Handlebar post slides inside the body's head-tube column; intentional insertion.",
    )
    ctx.allow_overlap(
        crank, body,
        reason="Crank hub passes through the body and seats on the dark axle boss; intentional mounting.",
    )
    ctx.allow_overlap(
        flywheel, body,
        reason="Flywheel disc is seated against the body's front cowl on its axle; intentional mounting.",
    )
    # Pedals are captured on the crank-arm tips (spindle nested in the arm).
    ctx.allow_overlap(
        left_pedal, crank,
        reason="Left pedal spindle is intentionally captured on the crank arm tip.",
    )
    ctx.allow_overlap(
        right_pedal, crank,
        reason="Right pedal spindle is intentionally captured on the crank arm tip.",
    )
    # Stabilizer down-legs intentionally rise into the body shroud to mount.
    ctx.allow_overlap(
        front_foot, body,
        elem_a="foot_leg", elem_b="body_shroud",
        reason="Front stabilizer down-leg intentionally enters the body shroud to mount.",
    )
    ctx.allow_overlap(
        rear_foot, body,
        elem_a="foot_leg", elem_b="body_shroud",
        reason="Rear stabilizer down-leg intentionally enters the body shroud to mount.",
    )

    # ---- Flywheel spins about its side (Y) axis: off-axis marker rotates ----
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

    # ---- Crank rotates: pedal mount revolves about the crank axis ----
    p0 = ctx.part_world_position(left_pedal)
    with ctx.pose({crank_joint: math.pi / 2.0}):
        p1 = ctx.part_world_position(left_pedal)
    ctx.check(
        "crank rotation revolves the pedal about the crank axis",
        p0 is not None and p1 is not None
        and (abs(p1[0] - p0[0]) > 0.03 or abs(p1[2] - p0[2]) > 0.03),
        details=f"left pedal rest={p0}, crank quarter-turn={p1}",
    )

    # ---- A pedal spins on its own spindle: tread marker reorients ----
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
    # Saddle post stays inserted in the body at full rise.
    ctx.expect_overlap(
        saddle_post, body, axes="z",
        elem_a="saddle_post_tube", elem_b="body_shroud",
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

    # ---- Base feet form the widest footprint and rest on the ground ----
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
    body_span_y = _ext(ctx.part_world_aabb(body))[1]
    ctx.check(
        "feet are the widest footprint",
        foot_span_y >= body_span_y - 0.01,
        details=f"foot_span_y={foot_span_y}, body_span_y={body_span_y}",
    )
    # Front foot is forward of the rear foot.
    ctx.check(
        "front foot is forward of rear foot",
        (ff[0][0] + ff[1][0]) / 2.0 > (rf[0][0] + rf[1][0]) / 2.0,
        details=f"front_cx={(ff[0][0]+ff[1][0])/2.0}, rear_cx={(rf[0][0]+rf[1][0])/2.0}",
    )

    return ctx.report()


object_model = build_object_model()
