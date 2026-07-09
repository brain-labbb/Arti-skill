from __future__ import annotations

# Small black wireless computer mouse.
# Frame: long body axis along +X (FRONT/click end at +X, rear palm hump at -X),
# width along Y (left button +Y, right button -Y), height along +Z with the
# base resting at z=0 and the dome rising up. Scale ~0.10 x 0.06 x 0.035 m.
# Articulations:
#   - left_button:  REVOLUTE about a rear hinge (about +Y axis) so the FRONT
#                   tip of the cover dips down ~1.5mm when pressed.
#   - right_button: REVOLUTE about a rear hinge, same press behavior.
#   - scroll_wheel: CONTINUOUS spin about the Y axis (off-axis notch for
#                   spin detectability) sitting in the slot between the buttons.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key longitudinal stations (X) ----
BODY_FRONT_X = 0.050   # nose / front edge of the mouse
BODY_REAR_X = -0.050   # rear palm edge
SPLIT_X = -0.006       # rear end of the button covers / front of palm hump
WHEEL_X = 0.030        # scroll wheel center (between the front buttons)
WHEEL_R = 0.0075       # scroll wheel radius
WHEEL_HALF_W = 0.0045  # scroll wheel half width
SLOT_HALF_W = 0.0058   # half width of the gap between the two buttons


def _loft(sections) -> cq.Workplane:
    # sections: list of ("rect", x, w, h) or ("circle", x, r) along +X (YZ planes).
    wp = cq.Workplane("YZ")
    prev = 0.0
    for i, s in enumerate(sections):
        x = s[1]
        off = x if i == 0 else x - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        if s[0] == "circle":
            wp = wp.circle(s[2])
        else:
            wp = wp.rect(s[2], s[3])
        prev = x
    return wp.loft(ruled=False)


def _body_shell() -> cq.Workplane:
    # Curved mouse shell: a flat-bottomed dome that is tallest near the rear
    # (palm hump) and tapers to a lower, narrower nose at the front. Built as a
    # loft of rectangular sections (each placed with its bottom on z=0), then
    # intersected with a longitudinal arched ridge so the top reads as a dome,
    # and clipped at z=0 for a flat base.
    wp = cq.Workplane("YZ")
    sections = [
        (BODY_REAR_X, 0.044, 0.022),
        (-0.038, 0.056, 0.031),
        (-0.020, 0.060, 0.035),   # tallest / widest palm hump
        (0.004, 0.059, 0.033),
        (0.026, 0.055, 0.026),
        (0.042, 0.046, 0.019),
        (BODY_FRONT_X, 0.034, 0.013),  # low narrow nose
    ]
    prev = None
    for i, (x, w, h) in enumerate(sections):
        off = x if i == 0 else x - prev
        wp = wp.workplane(offset=off)
        # rect drawn in local YZ plane: first arg -> Y span, second -> Z span;
        # shift center up by h/2 so the bottom edge lands on z=0.
        wp = wp.center(0.0, h / 2.0).rect(w, h).center(0.0, -h / 2.0)
        prev = x
    shell = wp.loft(ruled=False)

    # Longitudinal arched ridge (in the XZ plane) giving the dome its curved top.
    ridge = (
        cq.Workplane("XZ")
        .moveTo(BODY_REAR_X, 0.020)
        .threePointArc((-0.020, 0.036), (BODY_FRONT_X, 0.013))
        .lineTo(BODY_FRONT_X, 0.0)
        .lineTo(BODY_REAR_X, 0.0)
        .close()
        .extrude(0.05, both=True)
    )
    shell = shell.intersect(ridge)
    # Clip anything below the base plane so the bottom is flat at z=0.
    base_block = cq.Workplane("XY").box(0.16, 0.10, 0.10, centered=(True, True, False))
    shell = shell.intersect(base_block)
    return shell


def _button_cover(shell: cq.Workplane, side: int) -> cq.Workplane:
    # One front click cover, carved from the SAME shell solid so its underside
    # coincides exactly with the shell top surface (guaranteed seated contact).
    # It is the top skin of the front half of the mouse, on one Y side of the
    # central wheel slot. side = +1 -> left (+Y), -1 -> right (-Y).
    #
    # Region: x in [SPLIT_X, BODY_FRONT_X+eps], this Y side beyond the slot,
    # and the top skin only (a horizontal slab keeps a thin top lid).
    if side > 0:
        y_lo, y_hi = SLOT_HALF_W, 0.06
    else:
        y_lo, y_hi = -0.06, -SLOT_HALF_W
    region = (
        cq.Workplane("XY")
        .box(
            (BODY_FRONT_X + 0.004) - SPLIT_X,
            y_hi - y_lo,
            0.10,
            centered=(False, False, False),
        )
        .translate((SPLIT_X, y_lo, 0.020))  # keep only top skin (z>0.020)
    )
    cover = shell.intersect(region)
    return cover


def _wheel_mesh():
    # Dark scroll wheel: a short cylinder about the Y axis with a ribbed rim and
    # an off-axis radial notch so its spin is visually detectable.
    wheel = CylinderGeometry(WHEEL_R, 2.0 * WHEEL_HALF_W, radial_segments=28).rotate_x(
        math.pi / 2.0
    )
    # tread ribs around the rim
    for k in range(10):
        a = 2.0 * math.pi * k / 10.0
        rib = BoxGeometry((0.0016, 2.0 * WHEEL_HALF_W + 0.0006, 0.003))
        rib.rotate_y(a)
        rib.translate(
            (WHEEL_R - 0.0009) * math.cos(a), 0.0, (WHEEL_R - 0.0009) * math.sin(a)
        )
        wheel.merge(rib)
    # off-axis marker nub: a small tab that protrudes past the rim at one
    # angular position so the wheel's silhouette changes as it spins.
    nub = BoxGeometry((0.0022, 2.0 * WHEEL_HALF_W, 0.004))
    nub.translate(0.0, 0.0, WHEEL_R + 0.0012)  # sticks out at top (+Z) at rest
    wheel.merge(nub)
    return mesh_from_geometry(wheel, "scroll_wheel")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wireless_mouse")

    matte_black = model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    gloss_band = model.material("gloss_band", rgba=(0.16, 0.16, 0.18, 1.0))
    button_black = model.material("button_black", rgba=(0.11, 0.11, 0.12, 1.0))
    wheel_dark = model.material("wheel_dark", rgba=(0.05, 0.05, 0.06, 1.0))

    # ---- body (root): the main rounded shell + a glossy lower band ----
    body = model.part("body")
    shell = _body_shell()
    # Recess a slot for the scroll wheel through the shell top between the
    # buttons. The button lids sit ON the shell top as a separate layer (a
    # seated overlap, justified in tests) so they read as distinct click covers.
    slot = (
        cq.Workplane("XY")
        .box(0.022, 2.0 * SLOT_HALF_W, 0.05, centered=(False, True, False))
        .translate((WHEEL_X - 0.011, 0.0, 0.022))
    )
    body_shell = shell.cut(slot)
    body.visual(
        mesh_from_cadquery(body_shell, "body_shell"), material=matte_black, name="body_shell"
    )

    # Slightly glossy lower band wrapping the base edge of the shell.
    band = _loft(
        [
            ("rect", BODY_REAR_X + 0.002, 0.046, 0.006),
            ("rect", -0.020, 0.061, 0.006),
            ("rect", 0.026, 0.056, 0.006),
            ("rect", BODY_FRONT_X - 0.002, 0.035, 0.006),
        ]
    ).translate((0.0, 0.0, 0.004))
    base_clip = cq.Workplane("XY").box(0.16, 0.10, 0.10, centered=(True, True, False))
    band = band.intersect(base_clip)
    body.visual(mesh_from_cadquery(band, "lower_band"), material=gloss_band, name="lower_band")

    body.inertial = Inertial.from_geometry(
        Box((0.10, 0.060, 0.035)), mass=0.085, origin=Origin(xyz=(-0.006, 0.0, 0.017))
    )

    # ---- scroll wheel: continuous spin about Y, seated in the slot ----
    wheel = model.part("scroll_wheel")
    wheel.visual(_wheel_mesh(), material=wheel_dark, name="scroll_wheel")
    wheel.inertial = Inertial.from_geometry(
        Box((2.0 * WHEEL_R, 2.0 * WHEEL_HALF_W, 2.0 * WHEEL_R)), mass=0.002
    )
    model.articulation(
        "body_to_scroll_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(WHEEL_X, 0.0, 0.027)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=20.0),
    )

    # ---- left / right click covers: revolute press about a rear hinge ----
    # Hinge axis is +Y so the front tip (+X) dips in -Z under positive rotation.
    # Hinge origin at the rear of the cover (SPLIT_X), up at the top of the shell.
    # The cover geometry is authored in world coordinates, so the joint origin is
    # the rotation pivot and the child link frame coincides with world at zero pose.
    hinge_x = SPLIT_X
    hinge_z = 0.030
    press = math.radians(7.0)  # ~1.5mm front-tip drop over the ~0.056m lever
    for name, side in (("left_button", +1), ("right_button", -1)):
        btn = model.part(name)
        # Cover is authored in world coords; the child link frame sits at the
        # hinge origin, so shift the geometry by -hinge so it lands correctly.
        cover = _button_cover(shell, side).translate((-hinge_x, 0.0, -hinge_z))
        btn.visual(
            mesh_from_cadquery(cover, f"{name}_cover"),
            material=button_black,
            name=f"{name}_cover",
        )
        btn.inertial = Inertial.from_geometry(
            Box((0.040, 0.024, 0.012)),
            mass=0.004,
            # centroid expressed in the child (hinge) frame: world ~ hinge.
            origin=Origin(xyz=(0.022 - hinge_x, side * 0.018, 0.030 - hinge_z)),
        )
        # press DOWN: front tip (+X) moves -Z -> positive rotation about +Y.
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=btn,
            origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=press),
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
    left_joint = object_model.get_articulation("body_to_left_button")
    right_joint = object_model.get_articulation("body_to_right_button")
    spin = object_model.get_articulation("body_to_scroll_wheel")

    # --- body is dome-shaped and longer than it is wide ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body longer than wide",
        bext[0] > bext[1] + 0.02,
        details=f"body extents (L,W,H)={bext}",
    )
    ctx.check(
        "body is a low dome (wider/longer than tall)",
        bext[2] < bext[1] and bext[2] < bext[0],
        details=f"body extents (L,W,H)={bext}",
    )

    # --- buttons seat against the body top (covers rest on the shell) ---
    ctx.allow_overlap(
        left, body, elem_a="left_button_cover", elem_b="body_shell",
        reason="Click lid sits flush over the front shell top with a hidden seated overlap.",
    )
    ctx.allow_overlap(
        right, body, elem_a="right_button_cover", elem_b="body_shell",
        reason="Click lid sits flush over the front shell top with a hidden seated overlap.",
    )
    ctx.expect_contact(left, body, name="left button seated on shell")
    ctx.expect_contact(right, body, name="right button seated on shell")

    # --- pressing each button drops its FRONT tip ---
    # The hinge is at the rear; the front nose is the lowest point of the lid,
    # so pressing rotates the front down and its min-Z (and max-Z) both drop.
    def tip_z(part):
        return ctx.part_world_aabb(part)[0][2]  # lowest point = front nose underside

    l_rest = tip_z(left)
    with ctx.pose({left_joint: math.radians(7.0)}):
        l_press = tip_z(left)
    ctx.check(
        "left button presses down (front tip drops)",
        l_press < l_rest - 0.0008,
        details=f"rest_tip_z={l_rest}, pressed_tip_z={l_press}",
    )

    r_rest = tip_z(right)
    with ctx.pose({right_joint: math.radians(7.0)}):
        r_press = tip_z(right)
    ctx.check(
        "right button presses down (front tip drops)",
        r_press < r_rest - 0.0008,
        details=f"rest_tip_z={r_rest}, pressed_tip_z={r_press}",
    )

    # --- wheel sits between the two buttons (Y near 0, between L and R) ---
    def yc(part):
        mn, mx = ctx.part_world_aabb(part)
        return 0.5 * (mn[1] + mx[1])

    def xc(part):
        mn, mx = ctx.part_world_aabb(part)
        return 0.5 * (mn[0] + mx[0])

    wy, ly, ry = yc(wheel), yc(left), yc(right)
    ctx.check(
        "scroll wheel centered between buttons in Y",
        abs(wy) < 0.004 and ry < wy < ly,
        details=f"wheel_yc={wy}, left_yc={ly}, right_yc={ry}",
    )
    ctx.check(
        "scroll wheel at the front click zone",
        xc(wheel) > 0.0,
        details=f"wheel_xc={xc(wheel)}",
    )
    ctx.allow_overlap(
        wheel, body, elem_a="scroll_wheel", elem_b="body_shell",
        reason="Wheel is intentionally recessed into the slot in the shell top.",
    )

    # --- wheel spins about its Y axis: the off-axis notch swings around ---
    n0 = _ext(ctx.part_world_aabb(wheel))
    z0 = ctx.part_world_aabb(wheel)[1][2]  # top of wheel incl. notch
    with ctx.pose({spin: math.pi / 2.0}):
        z90 = ctx.part_world_aabb(wheel)[1][2]
        n90 = _ext(ctx.part_world_aabb(wheel))
    ctx.check(
        "scroll wheel notch swings when spun",
        abs(z90 - z0) > 0.0005 or abs(n90[0] - n0[0]) > 0.0005,
        details=f"rest_topZ={z0}, spun_topZ={z90}, rest_ext={n0}, spun_ext={n90}",
    )

    return ctx.report()


object_model = build_object_model()
