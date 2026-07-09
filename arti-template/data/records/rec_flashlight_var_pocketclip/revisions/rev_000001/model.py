from __future__ import annotations

# Black tactical flashlight / torch.
# Frame: cylindrical body axis along +X.
#   - Front (HEAD with flared bezel + reflector + LED) at +X.
#   - Rear (tail cap) at -X.
#   - Body centerline at z=0; the side push button sits on +Z near the head.
# Articulations:
#   - focus_head: CONTINUOUS twist about the body (+X) axis (zoom/focus ring).
#       An off-axis bezel crenellation makes the spin detectable.
#   - push_button: PRISMATIC press into the body (-Z), ~1.5mm travel.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

# ----- key longitudinal stations (meters, along +X) -----
TAIL_X = -0.100  # rear face of tail cap
GRIP_BACK_X = -0.080
GRIP_FRONT_X = 0.020
BODY_FRONT_X = 0.060  # front lip of the body tube (where head seats)
HEAD_BACK_X = 0.050  # head sleeve back (overlaps body front lip)
HEAD_FRONT_X = 0.100  # bezel front rim

BODY_R = 0.0175  # body tube radius (~0.035m dia)
HEAD_R = 0.030  # head / bezel radius (~0.060m dia)

BUTTON_X = 0.040  # side button longitudinal station (near the head)


def _loft(sections) -> cq.Workplane:
    # sections: list of ("circle", x, r) along +X (YZ planes).
    wp = cq.Workplane("YZ")
    prev = 0.0
    for i, s in enumerate(sections):
        x = s[1]
        off = x if i == 0 else x - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(s[2])
        prev = x
    return wp.loft(ruled=False)


def _body_solid() -> cq.Workplane:
    # Long aluminum tube: tail cap, knurled grip swell, then a slim mid body
    # that narrows to a front lip where the head seats.
    return _loft(
        [
            ("circle", TAIL_X, 0.018),
            ("circle", TAIL_X + 0.006, 0.020),  # tail cap rim
            ("circle", GRIP_BACK_X, 0.019),
            ("circle", -0.030, 0.0195),  # grip swell
            ("circle", GRIP_FRONT_X, 0.0185),
            ("circle", 0.045, BODY_R),
            ("circle", BODY_FRONT_X, 0.019),  # front lip swells slightly
        ]
    )


def _knurl_band() -> cq.Workplane:
    # Diamond knurled grip: two opposing rib lattices on the grip swell, built
    # as many small angled ridges wrapped around the body surface.
    band = None
    z0 = GRIP_BACK_X + 0.004
    z1 = GRIP_FRONT_X - 0.004
    n = 28
    rib_r = 0.0202
    for sign in (1.0, -1.0):
        for i in range(n):
            ang = 2.0 * math.pi * i / n
            ridge = cq.Workplane("XY").box(
                z1 - z0, 0.0022, 0.0016, centered=(True, True, True)
            )
            ridge = ridge.rotate((0, 0, 0), (1, 0, 0), sign * 18.0)
            ridge = ridge.translate((0.5 * (z0 + z1), 0.0, rib_r))
            ridge = ridge.rotate((0, 0, 0), (1, 0, 0), math.degrees(ang))
            band = ridge if band is None else band.union(ridge)
    return band


def _bezel_mesh():
    # Crenellated bezel ring at the very front of the head: an OPEN annular metal
    # rim (bore aligned with the reflector mouth so you can see into the head),
    # with notch teeth standing proud around the circumference.
    ring = (
        cq.Workplane("YZ")
        .workplane(offset=HEAD_FRONT_X - 0.010)  # x = 0.090
        .circle(HEAD_R)
        .circle(0.0285)  # open bore over the reflector mouth
        .extrude(0.010)  # to x = 0.100
    )
    teeth = 8
    for i in range(teeth):
        ang = 2.0 * math.pi * i / teeth
        ty = (HEAD_R - 0.003) * math.cos(ang)
        tz = (HEAD_R - 0.003) * math.sin(ang)
        tooth = (
            cq.Workplane("YZ")
            .workplane(offset=HEAD_FRONT_X - 0.004)  # x = 0.096
            .center(ty, tz)
            .circle(0.0045)
            .extrude(0.012)  # stands proud of the front rim
        )
        ring = ring.union(tooth)
    return mesh_from_cadquery(ring, "bezel_ring")


def _head_shell_mesh():
    # Flared head body: a sleeve at the back (slips over the body front lip)
    # lofting out to the wide bezel radius at the front.
    head = _loft(
        [
            ("circle", HEAD_BACK_X, 0.0195),  # sleeve over body lip
            ("circle", HEAD_BACK_X + 0.010, 0.021),
            ("circle", 0.075, 0.026),  # flare out
            ("circle", HEAD_FRONT_X - 0.010, HEAD_R),
            ("circle", HEAD_FRONT_X - 0.002, HEAD_R),  # bezel seat
        ]
    )
    # Hollow out a conical reflector cavity that opens at the front face, so the
    # reflector cone and LED inside are visible instead of a solid capped disc.
    bore = (
        cq.Workplane("XZ")
        .moveTo(0.064, 0.0)  # throat (back of cavity)
        .lineTo(0.064, 0.0072)  # throat radius (around the LED)
        .lineTo(0.0995, 0.0285)  # mouth radius just inside the front rim
        .lineTo(0.0995, 0.0)  # close across the open front
        .close()
        .revolve(360, (0, 0, 0), (1, 0, 0))
    )
    head = head.cut(bore)
    return mesh_from_cadquery(head, "head_shell")


def _reflector_mesh():
    # Deep reflector cone recessed inside the head, opening toward +X.
    # Revolved hollow funnel (thin wall) about the +X axis.
    funnel = (
        cq.Workplane("XZ")
        .moveTo(0.066, 0.006)  # throat (small radius near LED)
        .lineTo(0.098, 0.027)  # mouth (wide, near bezel front)
        .lineTo(0.098, 0.029)
        .lineTo(0.064, 0.0075)
        .close()
        .revolve(360, (0, 0, 0), (1, 0, 0))
    )
    return mesh_from_cadquery(funnel, "reflector")


def _pocket_clip_mesh():
    """Spring-steel pocket clip: thin bent strip mounted on the body near the
    tail, running along the -Z side (opposite the push button).  The mount
    section sits flush against the body tube; the arm angles outward then runs
    parallel to the body; the free end curls inward for pocket retention."""
    points = [
        (-0.082, 0.0, -0.0192),  # mount: seated on body surface near tail
        (-0.074, 0.0, -0.0225),  # transition leaving body
        (-0.064, 0.0, -0.0265),  # clearing body surface
        (-0.040, 0.0, -0.0280),  # running along body
        (-0.010, 0.0, -0.0280),  # mid-run
        (0.015, 0.0, -0.0270),   # approaching tip
        (0.024, 0.0, -0.0245),   # tip curl start
        (0.029, 0.0, -0.0210),   # tip curl inward toward body
    ]
    return sweep_profile_along_spline(
        points,
        profile=rounded_rect_profile(0.0012, 0.009, radius=0.0004),
        samples_per_segment=16,
        up_hint=(0.0, 1.0, 0.0),
        cap_profile=True,
    )


def _clip_mount_solid() -> cq.Workplane:
    """Thin clamp band around the body tube where the pocket clip attaches."""
    band_x = -0.080
    band_w = 0.008
    r_inner = 0.0185  # slightly inside body surface for secure mount
    r_outer = 0.0215
    return (
        cq.Workplane("YZ")
        .workplane(offset=band_x - band_w * 0.5)
        .circle(r_outer)
        .circle(r_inner)
        .extrude(band_w)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tactical_flashlight")

    alu = model.material("matte_black_aluminum", rgba=(0.10, 0.10, 0.11, 1.0))
    knurl_mat = model.material("knurl_black", rgba=(0.07, 0.07, 0.08, 1.0))
    bezel_mat = model.material("bezel_metal", rgba=(0.18, 0.18, 0.20, 1.0))
    reflector_mat = model.material("reflector_silver", rgba=(0.62, 0.63, 0.66, 1.0))
    led_mat = model.material("led_green", rgba=(0.30, 0.95, 0.35, 1.0))
    button_mat = model.material("button_rubber", rgba=(0.14, 0.14, 0.15, 1.0))
    clip_mat = model.material("black_oxide_steel", rgba=(0.16, 0.16, 0.18, 1.0))

    # ---------------- body (root): tube + grip knurl + tail cap -------------
    body = model.part("body")

    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=alu,
        name="body_shell",
    )

    # Diamond knurled grip overlay (distinct darker material).
    body.visual(
        mesh_from_cadquery(_knurl_band(), "grip_knurl"),
        material=knurl_mat,
        name="grip_knurl",
    )

    # Tail cap end button (flat disc on the rear face).
    tail_btn = CylinderGeometry(0.012, 0.006, radial_segments=24).rotate_y(math.pi / 2.0)
    tail_btn.translate(TAIL_X - 0.002, 0.0, 0.0)
    body.visual(mesh_from_geometry(tail_btn, "tail_cap"), material=bezel_mat, name="tail_cap")

    # Raised boss that hosts the side push button, on +Z near the head.
    boss = CylinderGeometry(0.0075, 0.006, radial_segments=20)  # axis +Z
    boss.translate(BUTTON_X, 0.0, BODY_R + 0.001)
    body.visual(mesh_from_geometry(boss, "button_boss"), material=alu, name="button_boss")

    # Spring-steel pocket clip: bent strip on -Z side, mounted near the tail.
    body.visual(
        mesh_from_geometry(_pocket_clip_mesh(), "pocket_clip"),
        material=clip_mat,
        name="pocket_clip",
    )
    # Clip mount band: thin annular clamp ring around the body at the clip base.
    body.visual(
        mesh_from_cadquery(_clip_mount_solid(), "clip_mount"),
        material=clip_mat,
        name="clip_mount",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.20, 0.040, 0.040)), mass=0.28, origin=Origin(xyz=(-0.010, 0.0, 0.0))
    )

    # ---------------- focus head: bezel + reflector + LED (twists) ----------
    head = model.part("focus_head")
    head.visual(_head_shell_mesh(), material=alu, name="head_shell")
    head.visual(_bezel_mesh(), material=bezel_mat, name="bezel_ring")
    head.visual(_reflector_mesh(), material=reflector_mat, name="reflector")

    # Central green LED emitter at the reflector throat. A small puck base seats
    # it into the funnel throat (so it is connected, not floating), with the
    # bright green emitter dome on top.
    led = CylinderGeometry(0.0065, 0.0035, radial_segments=20).rotate_y(math.pi / 2.0)
    led.translate(0.0645, 0.0, 0.0)  # base disc spanning the throat opening
    dome = SphereGeometry(0.0035, width_segments=16, height_segments=10)
    dome.translate(0.0665, 0.0, 0.0)
    led.merge(dome)
    head.visual(mesh_from_geometry(led, "led_emitter"), material=led_mat, name="led_emitter")

    # Off-axis marker crenellation: one taller bezel tooth so spin is visible.
    marker = CylinderGeometry(0.005, 0.016, radial_segments=10).rotate_y(math.pi / 2.0)
    marker.translate(HEAD_FRONT_X + 0.003, 0.0, HEAD_R - 0.004)
    head.visual(mesh_from_geometry(marker, "bezel_marker"), material=bezel_mat, name="bezel_marker")

    head.inertial = Inertial.from_geometry(
        Box((0.055, 0.062, 0.062)), mass=0.10, origin=Origin(xyz=(0.078, 0.0, 0.0))
    )

    model.articulation(
        "head_focus_twist",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0),
    )

    # ---------------- side push button: presses into the body --------------
    button = model.part("push_button")
    btn = CylinderGeometry(0.006, 0.006, radial_segments=20)  # axis +Z
    button.visual(mesh_from_geometry(btn, "button_cap"), material=button_mat, name="button_cap")
    button.inertial = Inertial.from_geometry(Box((0.012, 0.012, 0.006)), mass=0.004)

    # Button sits on the boss top, presses inward along -Z.
    model.articulation(
        "button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(BUTTON_X, 0.0, BODY_R + 0.007)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.0015),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    head = object_model.get_part("focus_head")
    button = object_model.get_part("push_button")
    twist = object_model.get_articulation("head_focus_twist")
    press = object_model.get_articulation("button_press")

    # ---- Hero: head wider than grip, body is a long cylinder ----
    shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    body_ext = _ext(shell_aabb)
    head_ext = _ext(ctx.part_world_aabb(head))
    ctx.check(
        "body is a long cylinder (X is the long axis)",
        body_ext[0] > 0.15 and body_ext[0] > 3.0 * body_ext[1],
        details=f"body_shell extents={body_ext}",
    )
    ctx.check(
        "head is wider than the grip body",
        head_ext[1] > body_ext[1] + 0.015 and head_ext[2] > body_ext[2] + 0.015,
        details=f"head={head_ext}, body_shell={body_ext}",
    )

    # ---- Head sits at the front (+X) and is seated over the body front lip ----
    head_pos = ctx.part_world_position(head)
    body_pos = ctx.part_world_position(body)
    ctx.check(
        "head mounted at the front of the body",
        head_pos is not None and body_pos is not None and head_pos[0] >= body_pos[0],
        details=f"head={head_pos}, body={body_pos}",
    )
    ctx.allow_overlap(
        head,
        body,
        elem_a="head_shell",
        elem_b="body_shell",
        reason="Head back sleeve is intentionally slipped over the body front lip.",
    )
    ctx.expect_overlap(
        head, body, axes="x", min_overlap=0.004, name="head sleeve seated over body lip"
    )

    # ---- Reflector + LED are at the front, near the bezel mouth ----
    led_aabb = ctx.part_element_world_aabb(head, elem="led_emitter")
    refl_aabb = ctx.part_element_world_aabb(head, elem="reflector")
    ctx.check(
        "LED emitter is at the front of the head",
        led_aabb is not None and 0.5 * (led_aabb[0][0] + led_aabb[1][0]) > 0.055,
        details=f"led_aabb={led_aabb}",
    )
    ctx.check(
        "reflector funnel is at the front of the head",
        refl_aabb is not None and refl_aabb[1][0] > 0.090,
        details=f"reflector_aabb={refl_aabb}",
    )

    # ---- Focus head twists about the body axis: off-axis marker rotates ----
    # Marker starts on +Z (top). A +quarter turn about +X (right-hand rule)
    # carries +Z toward -Y.
    m0 = ctx.part_element_world_aabb(head, elem="bezel_marker")
    mc0 = (0.5 * (m0[0][1] + m0[1][1]), 0.5 * (m0[0][2] + m0[1][2]))
    with ctx.pose({twist: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(head, elem="bezel_marker")
        mc1 = (0.5 * (m1[0][1] + m1[1][1]), 0.5 * (m1[0][2] + m1[1][2]))
    ctx.check(
        "marker starts near the top (+Z)",
        mc0[1] > 0.018 and abs(mc0[0]) < 0.010,
        details=f"rest marker (y,z)={mc0}",
    )
    ctx.check(
        "focus twist rotates the off-axis marker around the axis",
        mc1[0] < -0.018 and abs(mc1[1]) < 0.010,
        details=f"rest (y,z)={mc0}, quarter-turn (y,z)={mc1}",
    )

    # ---- Side push button presses into the body (z drops) and stays seated ----
    ctx.allow_overlap(
        button,
        body,
        elem_a="button_cap",
        elem_b="button_boss",
        reason="Button cap is seated into the boss bore and depresses into it.",
    )
    ctx.expect_contact(
        button, body, elem_a="button_cap", elem_b="button_boss", name="button seated in boss"
    )
    rest_z = ctx.part_world_position(button)[2]
    with ctx.pose({press: 0.0015}):
        pressed_z = ctx.part_world_position(button)[2]
    ctx.check(
        "push button presses inward (z drops)",
        pressed_z < rest_z - 0.001,
        details=f"rest_z={rest_z}, pressed_z={pressed_z}",
    )
    ctx.check(
        "button is on the near-head side, above the body axis",
        rest_z > BODY_R,
        details=f"button rest_z={rest_z}",
    )

    # ---- Pocket clip: protrudes below body on -Z side, near the tail --------
    clip_aabb = ctx.part_element_world_aabb(body, elem="pocket_clip")
    shell_aabb_clip = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "pocket_clip visual exists on body",
        clip_aabb is not None,
        details="pocket_clip AABB is None",
    )
    if clip_aabb is not None and shell_aabb_clip is not None:
        # Clip must protrude below (more negative Z than) the body tube surface
        clip_z_min = clip_aabb[0][2]
        shell_z_min = shell_aabb_clip[0][2]
        ctx.check(
            "pocket_clip protrudes outside body tube on -Z side",
            clip_z_min < shell_z_min - 0.004,
            details=f"clip_z_min={clip_z_min:.4f}, shell_z_min={shell_z_min:.4f}",
        )
        # Clip is near the tail (rear half of the body)
        clip_x_center = 0.5 * (clip_aabb[0][0] + clip_aabb[1][0])
        ctx.check(
            "pocket_clip is mounted near the tail (rear half of body)",
            clip_x_center < 0.0,
            details=f"clip_x_center={clip_x_center:.4f}",
        )
        # Clip has meaningful length along X (it's a long strip, not a dot)
        clip_x_span = clip_aabb[1][0] - clip_aabb[0][0]
        ctx.check(
            "pocket_clip has meaningful length along body axis",
            clip_x_span > 0.050,
            details=f"clip_x_span={clip_x_span:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
