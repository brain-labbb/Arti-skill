from __future__ import annotations

# Dark gray ergonomic wireless mouse.
# Frame: long axis along +X (front/scroll-wheel/buttons at +X, rear palm rest at -X),
# width along Y (centerline y=0), height along +Z (resting base at z=0).
# Scale ~0.11 m long x 0.065 m wide x 0.038 m tall.
# Articulations:
#   - left button:  PRISMATIC press down (-Z), ~1.5 mm.
#   - right button: PRISMATIC press down (-Z), ~1.5 mm.
#   - scroll wheel: CONTINUOUS rotation about its horizontal (Y) axis; an
#       off-axis notch makes the rotation visible. Plus a PRISMATIC middle-click.

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
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key longitudinal stations (X) ----
BODY_FRONT_X = 0.055  # front nose
BODY_REAR_X = -0.055  # rear palm hump
SEAM_FRONT_X = 0.020  # buttons run from the front to roughly mid-body
BUTTON_BACK_X = -0.006  # rear edge of the button covers
WHEEL_X = 0.040  # scroll wheel sits near the front center
TOP_Z = 0.044  # overall height (dome peak)
BAND_TOP_Z = 0.010  # glossy lower band cap height


def _loft(sections) -> cq.Workplane:
    # sections: list of ("rect", x, w, h, zc) | ("circle", x, r, zc) in YZ planes
    # along +X. zc is the z-center of the slice (the YZ workplane is at z=0, so the
    # rect/circle is shifted up by zc within that plane to control dome height).
    wp = cq.Workplane("YZ")
    prev = 0.0
    for i, s in enumerate(sections):
        x = s[1]
        off = x if i == 0 else x - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        # On a "YZ" workplane, local X maps to world Y and local Y maps to world Z.
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
    # Ergonomic dome: narrow rounded nose at the front, swelling to a wide rounded
    # palm hump toward the rear, tallest just behind center. Built as a smooth loft
    # of rounded rectangles (w = full width, h = full height of each YZ slice),
    # then the bottom is flattened by intersecting with a slab so the base is flat.
    outer = _loft(
        [
            ("rect", 0.055, 0.026, 0.018, 0.009),
            ("rect", 0.045, 0.044, 0.028, 0.014),
            ("rect", 0.030, 0.058, 0.036, 0.018),
            ("rect", 0.010, 0.065, 0.044, 0.022),
            ("rect", -0.015, 0.066, 0.046, 0.023),
            ("rect", -0.035, 0.062, 0.042, 0.021),
            ("rect", -0.050, 0.048, 0.032, 0.016),
            ("rect", -0.055, 0.030, 0.020, 0.010),
        ]
    )
    # Round the silhouette: union a tall central blob is unnecessary; instead just
    # keep flat base by clipping below z=0 and above the dome is already lofted.
    base_clip = cq.Workplane("XY").box(0.16, 0.10, 0.10, centered=(True, True, False))
    base_clip = base_clip.translate((0.0, 0.0, 0.0))
    return outer.intersect(base_clip)


def _body_solid() -> cq.Workplane:
    # Body = dome shell. We carve the top button bay (so the two button covers nest
    # flush in a recess) and a front slot for the scroll wheel.
    body = _body_outer()

    # Top button bay: a shallow recess across the top front where the two covers sit.
    bay = cq.Workplane("XY").box(0.066, 0.060, 0.020, centered=(True, True, False))
    bay = bay.translate((0.018, 0.0, TOP_Z - 0.004))
    body = body.cut(bay)

    # Front scroll-wheel slot: a thin vertical channel between the buttons.
    slot = cq.Workplane("XY").box(0.020, 0.013, 0.030, centered=(True, True, False))
    slot = slot.translate((WHEEL_X, 0.0, TOP_Z - 0.014))
    body = body.cut(slot)
    return body


def _band_solid() -> cq.Workplane:
    # Glossy lower band wrapping the bottom of the shell: take the body outer form,
    # intersect with a thin slab near the base, slightly inset so it reads as a
    # separate trim ring rather than part of the matte shell.
    outer = _body_outer()
    slab = cq.Workplane("XY").box(0.16, 0.10, BAND_TOP_Z, centered=(True, True, False))
    band = outer.intersect(slab)
    return band


def _button_cover(side: int) -> cq.Workplane:
    # One top click button: a curved cover filling half of the top button bay,
    # split from its neighbor by a central seam. `side` = +1 (left, +Y) or -1
    # (right, -Y). Built from the body outer dome intersected with a half-bay slab.
    outer = _body_outer()
    # Slab covering this side of the bay, from the seam outward, front to mid-body.
    # Width slab: from a small seam gap to the outer edge.
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
    # Carve out the wheel slot region from the inner front corner of each cover.
    slot = cq.Workplane("XY").box(0.024, 0.016, 0.034, centered=(True, True, False))
    slot = slot.translate((WHEEL_X, 0.0, TOP_Z - 0.014))
    cover = cover.cut(slot)
    return cover


def _wheel_mesh():
    # Scroll wheel: a short ribbed cylinder spinning about Y. An off-axis radial
    # marker fin makes the rotation detectable. Local frame: axle along Y.
    wheel = CylinderGeometry(0.0075, 0.011, radial_segments=28).rotate_x(math.pi / 2.0)
    # Tread ribs around the rim.
    for k in range(10):
        a = 2.0 * math.pi * k / 10.0
        rib = BoxGeometry((0.0016, 0.012, 0.0016))
        rib.rotate_y(a)
        rib.translate(0.0072 * math.cos(a), 0.0, 0.0072 * math.sin(a))
        wheel.merge(rib)
    # Off-axis marker notch: a small fin sticking out at one angular position so a
    # quarter turn visibly moves it.
    marker = BoxGeometry((0.004, 0.013, 0.0020))
    marker.translate(0.0078, 0.0, 0.0)
    wheel.merge(marker)
    return mesh_from_geometry(wheel, "scroll_wheel")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wireless_mouse")

    body_gray = model.material("body_dark_gray", rgba=(0.26, 0.27, 0.29, 1.0))
    band_gloss = model.material("band_gloss", rgba=(0.10, 0.10, 0.12, 1.0))
    button_gray = model.material("button_dark_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    wheel_dark = model.material("wheel_dark", rgba=(0.06, 0.06, 0.07, 1.0))

    # ---- body (root): ergonomic dome shell + glossy lower band ----
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
    body.inertial = Inertial.from_geometry(
        Box((0.11, 0.066, 0.038)), mass=0.09, origin=Origin(xyz=(-0.005, 0.0, 0.015))
    )

    # ---- left & right click buttons: press straight down (-Z) ----
    # side: +1 left (+Y), -1 right (-Y)
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
    # Wheel axle frame centered in the slot. Seated low enough that the wheel rim
    # nests into the body slot walls (intentional small overlap, allowed in tests),
    # so the wheel is physically captured rather than floating.
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

    # --- body shape: longer than wide, and dome-shaped (taller toward the rear) ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is longer than wide",
        bext[0] > bext[1] + 0.02,
        details=f"body extents={bext}",
    )
    ctx.check(
        "body reads as a low dome (wider/longer than tall)",
        bext[0] > bext[2] + 0.04 and bext[1] > bext[2],
        details=f"body extents={bext}",
    )

    # --- buttons seat into the top bay; allow the small nesting overlap ---
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
        # Button sits at the top of the body.
        bpos = ctx.part_element_world_aabb(btn, elem=f"{btn.name}_cover")
        top_z = bpos[1][2]
        ctx.check(
            f"{jname} button is at the top of the body",
            top_z > 0.030,
            details=f"{jname} cover top z={top_z}",
        )
        # Pressing the button moves it straight down and it stays seated.
        rest = _ext(ctx.part_world_aabb(btn))
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
        del rest

    # --- left button is on +Y side, right button on -Y side ---
    lpos = ctx.part_world_position(left)
    rpos = ctx.part_world_position(right)
    lc = ctx.part_element_world_aabb(left, elem="left_button_cover")
    rc = ctx.part_element_world_aabb(right, elem="right_button_cover")
    l_cy = (lc[0][1] + lc[1][1]) / 2.0
    r_cy = (rc[0][1] + rc[1][1]) / 2.0
    ctx.check(
        "left button on +Y, right button on -Y",
        l_cy > 0.0 and r_cy < 0.0,
        details=f"left_cy={l_cy}, right_cy={r_cy}, lpos={lpos}, rpos={rpos}",
    )

    # --- scroll wheel is at the front center, between the two buttons ---
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
    # Wheel rides in the slot at the top of the body (axle near top).
    ctx.allow_overlap(
        wheel,
        body,
        elem_a="scroll_wheel",
        elem_b="body_shell",
        reason="Scroll wheel is seated in the front slot and partly nested in the body.",
    )

    # --- scroll wheel spins about its Y axis: the off-axis marker moves ---
    rest_w = _ext(ctx.part_world_aabb(wheel))
    with ctx.pose({spin: math.pi / 2.0}):
        spun_w = _ext(ctx.part_world_aabb(wheel))
    # A quarter turn about Y swaps the marker's reach between X and Z, changing the
    # X/Z extents of the wheel part.
    ctx.check(
        "scroll wheel marker rotates about the axle (extents change)",
        abs(spun_w[0] - rest_w[0]) > 0.001 or abs(spun_w[2] - rest_w[2]) > 0.001,
        details=f"rest XZ=({rest_w[0]},{rest_w[2]}), spun XZ=({spun_w[0]},{spun_w[2]})",
    )

    return ctx.report()


object_model = build_object_model()
