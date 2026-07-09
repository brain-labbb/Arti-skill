from __future__ import annotations

# Dark gray ergonomic trackball mouse.
# Frame: long axis along +X (front/scroll-wheel/buttons at +X, rear palm rest at -X),
# width along Y (centerline y=0), height along +Z (resting base at z=0).
# Scale ~0.12 m long x 0.095 m wide x 0.038 m tall.
# Articulations:
#   - left button:  PRISMATIC press down (-Z), ~1.5 mm.
#   - right button: PRISMATIC press down (-Z), ~1.5 mm.
#   - scroll wheel: CONTINUOUS rotation about its horizontal (Y) axis.
#   - trackball:    CONTINUOUS rotation about X axis (rolling forward/back).

import math

import cadquery as cq
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
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key longitudinal stations (X) ----
BODY_FRONT_X = 0.058
BODY_REAR_X = -0.058
SEAM_FRONT_X = 0.022
BUTTON_BACK_X = -0.006
WHEEL_X = 0.040
TOP_Z = 0.038
BAND_TOP_Z = 0.010

# ---- trackball socket (behind the buttons, on the thumb +Y side) ----
BALL_RADIUS = 0.017
SOCKET_RADIUS = 0.018
BALL_CENTER_X = -0.035
BALL_CENTER_Y = 0.020
BALL_CENTER_Z = 0.028
SOCKET_SURFACE_Z = 0.032


def _loft(sections) -> cq.Workplane:
    # sections: list of ("rect", x, w, h, zc) | ("circle", x, r, zc) in YZ planes
    # along +X. zc is the z-center of the slice.
    wp = cq.Workplane("YZ")
    prev = 0.0
    for i, s in enumerate(sections):
        x = s[1]
        off = x if i == 0 else x - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        if s[0] == "circle":
            zc = s[2] if len(s) <= 3 else s[3]
            r = s[2]
            wp = wp.center(0.0, zc).circle(r).center(0.0, -zc)
        else:
            zc = s[4] if len(s) > 4 else 0.0
            wp = wp.center(0.0, zc).rect(s[2], s[3]).center(0.0, -zc)
        prev = x
    return wp.loft(ruled=False)


def _body_outer() -> cq.Workplane:
    # Wider, flatter trackball housing loft: broader footprint, smooth taper.
    outer = _loft(
        [
            ("rect", 0.058, 0.032, 0.016, 0.008),
            ("rect", 0.042, 0.065, 0.026, 0.013),
            ("rect", 0.022, 0.085, 0.034, 0.017),
            ("rect", 0.000, 0.095, 0.038, 0.019),
            ("rect", -0.025, 0.088, 0.034, 0.017),
            ("rect", -0.050, 0.065, 0.028, 0.014),
            ("rect", -0.058, 0.050, 0.020, 0.010),
        ]
    )
    # Clip below z=0 for a flat base
    base_clip = cq.Workplane("XY").box(0.18, 0.14, 0.10, centered=(True, True, False))
    base_clip = base_clip.translate((0.0, 0.0, 0.0))
    return outer.intersect(base_clip)


def _body_solid() -> cq.Workplane:
    # Trackball housing: dome shell with button bay, wheel slot, and ball socket.
    body = _body_outer()

    # Top button bay: a shallow recess across the top front for the two covers.
    bay = cq.Workplane("XY").box(0.066, 0.060, 0.020, centered=(True, True, False))
    bay = bay.translate((0.018, 0.0, TOP_Z - 0.004))
    body = body.cut(bay)

    # Front scroll-wheel slot: a thin vertical channel between the buttons.
    slot = cq.Workplane("XY").box(0.020, 0.013, 0.030, centered=(True, True, False))
    slot = slot.translate((WHEEL_X, 0.0, TOP_Z - 0.014))
    body = body.cut(slot)

    # Trackball socket: cut a sphere from the body to create the ball cavity.
    socket_cut = cq.Workplane("XY").sphere(SOCKET_RADIUS).translate(
        (BALL_CENTER_X, BALL_CENTER_Y, BALL_CENTER_Z)
    )
    body = body.cut(socket_cut)

    return body


def _band_solid() -> cq.Workplane:
    # Glossy lower band wrapping the bottom of the shell.
    outer = _body_outer()
    slab = cq.Workplane("XY").box(0.18, 0.14, BAND_TOP_Z, centered=(True, True, False))
    band = outer.intersect(slab)
    return band


def _button_cover(side: int) -> cq.Workplane:
    # One top click button cover: curved shell filling half the button bay.
    outer = _body_outer()
    y_lo = 0.0015 if side > 0 else -0.034
    y_hi = 0.034 if side > 0 else -0.0015
    y_c = (y_lo + y_hi) / 2.0
    y_w = y_hi - y_lo
    slab = cq.Workplane("XY").box(
        BODY_FRONT_X - BUTTON_BACK_X + 0.02, y_w, 0.024, centered=(True, True, False)
    )
    x_c = (BODY_FRONT_X + BUTTON_BACK_X) / 2.0
    slab = slab.translate((x_c, y_c, TOP_Z - 0.008))
    cover = outer.intersect(slab)
    slot = cq.Workplane("XY").box(0.024, 0.016, 0.034, centered=(True, True, False))
    slot = slot.translate((WHEEL_X, 0.0, TOP_Z - 0.014))
    cover = cover.cut(slot)
    return cover


def _wheel_mesh():
    # Scroll wheel: ribbed cylinder with off-axis marker fin.
    wheel = CylinderGeometry(0.0075, 0.011, radial_segments=28).rotate_x(math.pi / 2.0)
    for k in range(10):
        a = 2.0 * math.pi * k / 10.0
        rib = BoxGeometry((0.0016, 0.012, 0.0016))
        rib.rotate_y(a)
        rib.translate(0.0072 * math.cos(a), 0.0, 0.0072 * math.sin(a))
        wheel.merge(rib)
    marker = BoxGeometry((0.004, 0.013, 0.0020))
    marker.translate(0.0078, 0.0, 0.0)
    wheel.merge(marker)
    return mesh_from_geometry(wheel, "scroll_wheel")


def _trackball_mesh():
    # Trackball: sphere with visible rotation markers (cross stripes).
    # A stripe along Y protrudes at the Y poles; spinning about X rotates it
    # into the Z direction, making the rotation detectable via extent changes.
    ball = SphereGeometry(BALL_RADIUS, width_segments=32, height_segments=24)
    # Primary marker stripe along Y (perpendicular to spin axis X)
    stripe = BoxGeometry((0.003, BALL_RADIUS * 2.0 + 0.004, 0.003))
    ball.merge(stripe)
    # Secondary cross-stripe along X (parallel to spin axis, for visual interest)
    cross = BoxGeometry((BALL_RADIUS * 2.0 + 0.004, 0.003, 0.003))
    ball.merge(cross)
    return mesh_from_geometry(ball, "trackball")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trackball_mouse")

    body_gray = model.material("body_dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    band_gloss = model.material("band_gloss", rgba=(0.10, 0.10, 0.12, 1.0))
    button_gray = model.material("button_dark_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    wheel_dark = model.material("wheel_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    ball_blue = model.material("trackball_blue", rgba=(0.12, 0.28, 0.62, 1.0))

    # ---- body (root): trackball housing shell + glossy lower band ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=body_gray,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_band_solid(), "lower_band"),
        material=band_gloss,
        name="lower_band",
    )
    # Socket lip ring: a visible torus at the socket opening
    lip_radius = math.sqrt(SOCKET_RADIUS**2 - (SOCKET_SURFACE_Z - BALL_CENTER_Z)**2)
    lip_mesh = mesh_from_geometry(
        TorusGeometry(lip_radius, 0.002, radial_segments=12, tubular_segments=32),
        "socket_lip",
    )
    body.visual(
        lip_mesh,
        material=model.material("lip_dark", rgba=(0.15, 0.15, 0.17, 1.0)),
        origin=Origin(xyz=(BALL_CENTER_X, BALL_CENTER_Y, SOCKET_SURFACE_Z)),
        name="socket_lip",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.12, 0.095, 0.040)), mass=0.12, origin=Origin(xyz=(-0.005, 0.0, 0.015))
    )

    # ---- left & right click buttons: press straight down (-Z) ----
    for name, side in (("left_button", 1), ("right_button", -1)):
        btn = model.part(name)
        btn.visual(
            mesh_from_cadquery(_button_cover(side), f"{name}_cover"),
            material=button_gray,
            name=f"{name}_cover",
        )
        btn.inertial = Inertial.from_geometry(
            Box((0.055, 0.030, 0.020)),
            mass=0.004,
            origin=Origin(xyz=(0.022, side * 0.017, TOP_Z - 0.010)),
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=btn,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=0.0015),
        )

    # ---- scroll wheel: continuous spin about Y, mounted in the front slot ----
    wheel = model.part("scroll_wheel")
    wheel.visual(_wheel_mesh(), material=wheel_dark, name="scroll_wheel")
    wheel.inertial = Inertial.from_geometry(Cylinder(0.0078, 0.011), mass=0.0015)
    WHEEL_Z = TOP_Z - 0.012
    model.articulation(
        "body_to_scroll_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    # ---- trackball: continuous spin about X, seated in the housing socket ----
    trackball = model.part("trackball")
    trackball.visual(_trackball_mesh(), material=ball_blue, name="trackball_ball")
    trackball.inertial = Inertial.from_geometry(Sphere(BALL_RADIUS), mass=0.015)
    model.articulation(
        "body_to_trackball",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=trackball,
        origin=Origin(xyz=(BALL_CENTER_X, BALL_CENTER_Y, BALL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=10.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left = object_model.get_part("left_button")
    right = object_model.get_part("right_button")
    wheel = object_model.get_part("scroll_wheel")
    trackball = object_model.get_part("trackball")
    left_joint = object_model.get_articulation("body_to_left_button")
    right_joint = object_model.get_articulation("body_to_right_button")
    spin = object_model.get_articulation("body_to_scroll_wheel")
    ball_spin = object_model.get_articulation("body_to_trackball")

    # --- body shape: wider flat trackball housing proportions ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is wider than tall (trackball housing)",
        bext[1] > bext[2] + 0.02,
        details=f"body extents={bext}",
    )
    ctx.check(
        "body reads as a flat housing (longer and wider than tall)",
        bext[0] > bext[2] + 0.02 and bext[1] > bext[2],
        details=f"body extents={bext}",
    )

    # --- trackball socket: ball is seated in the housing on the thumb (+Y) side ---
    ctx.allow_overlap(
        trackball,
        body,
        elem_a="trackball_ball",
        elem_b="body_shell",
        reason="Trackball is intentionally seated in the spherical socket cavity of the housing.",
    )
    tb_pos = ctx.part_world_position(trackball)
    ctx.check(
        "trackball is on the thumb (+Y) side of the housing",
        tb_pos is not None and tb_pos[1] > 0.005,
        details=f"trackball pos={tb_pos}",
    )
    # Trackball protrudes above the housing mid-height
    tb_aabb = ctx.part_world_aabb(trackball)
    body_aabb = ctx.part_world_aabb(body)
    body_mid_z = (body_aabb[0][2] + body_aabb[1][2]) / 2.0
    ctx.check(
        "trackball protrudes above the housing mid-height",
        tb_aabb[1][2] > body_mid_z,
        details=f"trackball top z={tb_aabb[1][2]}, body mid z={body_mid_z}",
    )

    # Trackball spin: marker rotates about X, changing Y/Z extents
    rest_ext = _ext(ctx.part_world_aabb(trackball))
    with ctx.pose({ball_spin: math.pi / 2.0}):
        spun_ext = _ext(ctx.part_world_aabb(trackball))
    ctx.check(
        "trackball marker rotates on spin (Y/Z extents change)",
        abs(spun_ext[1] - rest_ext[1]) > 0.001 or abs(spun_ext[2] - rest_ext[2]) > 0.001,
        details=f"rest YZ=({rest_ext[1]},{rest_ext[2]}), spun YZ=({spun_ext[1]},{spun_ext[2]})",
    )

    # --- trackball articulation is continuous type ---
    ctx.check(
        "trackball joint is CONTINUOUS",
        ball_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"trackball joint type={ball_spin.articulation_type}",
    )

    # --- buttons seat into the top bay ---
    for btn, joint, jname in (
        (left, left_joint, "left"),
        (right, right_joint, "right"),
    ):
        ctx.allow_overlap(
            btn,
            body,
            elem_a=f"{btn.name}_cover",
            elem_b="body_shell",
            reason="Button cover is intentionally seated into the recessed top button bay.",
        )
        bpos = ctx.part_element_world_aabb(btn, elem=f"{btn.name}_cover")
        top_z = bpos[1][2]
        ctx.check(
            f"{jname} button is at the top of the body",
            top_z > 0.020,
            details=f"{jname} cover top z={top_z}",
        )
        rest_top = ctx.part_world_aabb(btn)[1][2]
        with ctx.pose({joint: 0.0015}):
            pressed_top = ctx.part_world_aabb(btn)[1][2]
            ctx.expect_overlap(
                btn,
                body,
                axes="xy",
                min_overlap=0.010,
                name=f"{jname} button stays seated when pressed",
            )
        ctx.check(
            f"{jname} button presses down",
            pressed_top < rest_top - 0.0010,
            details=f"rest_top={rest_top}, pressed_top={pressed_top}",
        )

    # --- left button on +Y, right on -Y ---
    lc = ctx.part_element_world_aabb(left, elem="left_button_cover")
    rc = ctx.part_element_world_aabb(right, elem="right_button_cover")
    l_cy = (lc[0][1] + lc[1][1]) / 2.0
    r_cy = (rc[0][1] + rc[1][1]) / 2.0
    ctx.check(
        "left button on +Y, right button on -Y",
        l_cy > 0.0 and r_cy < 0.0,
        details=f"left_cy={l_cy}, right_cy={r_cy}",
    )

    # --- scroll wheel between buttons ---
    wpos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel near front center between the buttons",
        wpos is not None and wpos[0] > 0.02 and abs(wpos[1]) < 0.006,
        details=f"wheel pos={wpos}",
    )
    waabb = ctx.part_world_aabb(wheel)
    w_cy = (waabb[0][1] + waabb[1][1]) / 2.0
    ctx.check(
        "wheel sits between left (+Y) and right (-Y) buttons",
        r_cy < w_cy < l_cy,
        details=f"left_cy={l_cy}, wheel_cy={w_cy}, right_cy={r_cy}",
    )
    ctx.allow_overlap(
        wheel,
        body,
        elem_a="scroll_wheel",
        elem_b="body_shell",
        reason="Scroll wheel is seated in the front slot and partly nested in the body.",
    )

    # --- scroll wheel spin ---
    rest_w = _ext(ctx.part_world_aabb(wheel))
    with ctx.pose({spin: math.pi / 2.0}):
        spun_w = _ext(ctx.part_world_aabb(wheel))
    ctx.check(
        "scroll wheel marker rotates about the axle",
        abs(spun_w[0] - rest_w[0]) > 0.001 or abs(spun_w[2] - rest_w[2]) > 0.001,
        details=f"rest XZ=({rest_w[0]},{rest_w[2]}), spun XZ=({spun_w[0]},{spun_w[2]})",
    )

    return ctx.report()


object_model = build_object_model()
