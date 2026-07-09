from __future__ import annotations

# Purple wireless game controller (Xbox-like gamepad).
# Frame:
#   +X = right (toward the ABXY face buttons)
#   +Y = "up" on the face / toward the top edge that carries the shoulder triggers
#   +Z = out of the top control face (sticks, buttons and dpad press in -Z)
# The glossy purple ergonomic shell has two grip handles. On the top face:
#   - left analog thumbstick (upper-left)   -> REVOLUTE tilt on its post
#   - right analog thumbstick (lower-right) -> REVOLUTE tilt on its post
#   - cross D-pad (lower-left)              -> REVOLUTE rocker
#   - four round ABXY face buttons (right)  -> PRISMATIC press (-Z)
#   - two center menu buttons + center logo (fixed)
# Top edge (+Y):
#   - left + right shoulder triggers        -> REVOLUTE pull about the top hinge

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

# ---- key layout constants (meters) ----
TOP_Z = 0.030  # top control surface plane (where sticks/buttons sit)

# stick centers
L_STICK = (-0.040, 0.018)   # (x, y) left stick, upper-left
R_STICK = (0.030, -0.018)   # (x, y) right stick, lower-right

DPAD_CTR = (-0.030, -0.020)  # (x, y) cross dpad, lower-left
ABXY_CTR = (0.045, 0.020)    # (x, y) center of the four face buttons, upper-right
ABXY_R = 0.016               # radial spacing of the diamond
BTN_R = 0.0075               # face button radius

TRIG_Y = 0.050               # top-edge hinge line (front edge of body)
TRIG_X = 0.045               # |x| of each trigger center


def _body_shell() -> cq.Workplane:
    # Ergonomic gamepad shell: a rounded central body lofted from a wide flat
    # top section down to a narrower base, with two grip horns swept out and
    # down to the lower corners. Built in the XY top-view plane, extruded in Z.
    # Central body silhouette (rounded "wing" shape) via a superellipse-ish
    # rounded rectangle, then grips added as lofted horns.

    # --- central body block (rounded) ---
    # Straight-walled rounded box so vertical edges exist for filleting, with a
    # gently domed top added afterwards as a second loft cap.
    body = (
        cq.Workplane("XY")
        .box(0.150, 0.092, 0.052)
        .edges("|Z")
        .fillet(0.030)
    )
    dome = (
        cq.Workplane("XY")
        .workplane(offset=0.026)
        .rect(0.146, 0.088)
        .workplane(offset=0.008)
        .rect(0.130, 0.072)
        .loft(ruled=False)
    )
    body = body.union(dome)

    # --- two grip horns dropping from the lower-front corners ---
    grips = None
    for sx in (-1.0, 1.0):
        gx = sx * 0.052
        grip = (
            cq.Workplane("XY")
            .center(gx, -0.010)
            .rect(0.040, 0.046)
            .workplane(offset=-0.030)
            .center(sx * 0.012, -0.020)
            .rect(0.036, 0.040)
            .workplane(offset=-0.030)
            .center(sx * 0.010, -0.016)
            .circle(0.018)
            .loft(ruled=False)
        )
        grips = grip if grips is None else grips.union(grip)

    shell = body.union(grips)
    return shell


def _stick_post_and_dish(stick_x: float, stick_y: float):
    # A raised dish ring around the stick base, fixed to the body. It straddles
    # the top surface (z ~ 0.034) so it stays fused to the shell.
    ring = (
        cq.Workplane("XY")
        .workplane(offset=0.026)
        .circle(0.018)
        .circle(0.012)
        .extrude(0.012)
    )
    ring = ring.translate((stick_x, stick_y, 0.0))
    return ring


def _thumbstick_mesh(name: str):
    # Analog stick: a short post topped by a concave grippy cap.
    post = CylinderGeometry(0.0055, 0.014, radial_segments=24)
    post.translate(0.0, 0.0, 0.007)
    cap = CylinderGeometry(0.011, 0.006, radial_segments=28)
    cap.translate(0.0, 0.0, 0.017)
    rim = CylinderGeometry(0.0125, 0.0025, radial_segments=28)
    rim.translate(0.0, 0.0, 0.0195)
    post.merge(cap)
    post.merge(rim)
    return mesh_from_geometry(post, name)


def _dpad_mesh(name: str):
    # Cross / plus shaped rocker built from two crossed bars.
    arm = 0.013
    th = 0.0075
    h = 0.006
    a = BoxGeometry((2 * arm, 2 * th, h))
    b = BoxGeometry((2 * th, 2 * arm, h))
    a.merge(b)
    # slight raised center hub
    hub = CylinderGeometry(0.006, h + 0.002, radial_segments=20)
    a.merge(hub)
    return mesh_from_geometry(a, name)


def _trigger_mesh(name: str):
    # Shoulder trigger: a paddle that hangs down-and-forward from the top-front
    # hinge. Local frame: hinge at origin, paddle drops into -Z and reaches out
    # in +Y. Built as a profile in the YZ plane, extruded along X (the hinge).
    # Pulling the trigger (positive rotation about +X) swings the free lower end
    # up toward the player.
    paddle = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)        # (y, z) at the hinge
        .lineTo(0.014, -0.002)
        .lineTo(0.018, -0.014)
        .lineTo(0.014, -0.024)
        .lineTo(0.0, -0.026)
        .lineTo(-0.004, -0.012)
        .close()
        .extrude(0.044)
    )
    # center the extrusion (along X) about the hinge
    paddle = paddle.translate((-0.022, 0.0, 0.0))
    return mesh_from_cadquery(paddle, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="game_controller")

    purple = model.material("body_purple", rgba=(0.43, 0.30, 0.68, 1.0))
    dark = model.material("dark_gray", rgba=(0.13, 0.13, 0.15, 1.0))
    btn_dark = model.material("button_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    glow = model.material("white_glow", rgba=(0.92, 0.93, 1.0, 1.0))

    # =====================================================================
    # BODY (root)
    # =====================================================================
    body = model.part("body")
    shell = _body_shell()
    body.visual(mesh_from_cadquery(shell, "body_shell"), material=purple, name="body_shell")

    # Recessed dish rings around each thumbstick (fixed trim on the body).
    body.visual(
        mesh_from_cadquery(_stick_post_and_dish(*L_STICK), "left_stick_dish"),
        material=dark,
        name="left_stick_dish",
    )
    body.visual(
        mesh_from_cadquery(_stick_post_and_dish(*R_STICK), "right_stick_dish"),
        material=dark,
        name="right_stick_dish",
    )

    # Center menu buttons (small pills) + round logo, fixed to the body.
    # Center menu buttons: short pills straddling the top surface (z ~ 0.034).
    for i, mx in enumerate((-0.010, 0.010)):
        body.visual(
            Cylinder(0.0035, 0.012),
            origin=Origin(xyz=(mx, 0.030, 0.032)),
            material=dark,
            name=f"menu_btn_{i}",
        )
    # Center logo disc with a glow, straddling the top surface.
    body.visual(
        Cylinder(0.008, 0.012),
        origin=Origin(xyz=(0.0, 0.012, 0.032)),
        material=glow,
        name="logo",
    )

    # Glow accent strips set into the front face of each grip (straddle face).
    for sx in (-1.0, 1.0):
        body.visual(
            Box((0.006, 0.034, 0.010)),
            origin=Origin(xyz=(sx * 0.050, 0.044, -0.012)),
            material=glow,
            name=f"glow_strip_{0 if sx < 0 else 1}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.150, 0.092, 0.060)), mass=0.30, origin=Origin(xyz=(0.0, -0.005, 0.0))
    )

    # =====================================================================
    # THUMBSTICKS (revolute tilt about a horizontal axis)
    # =====================================================================
    for name, (sx, sy), axis in (
        ("left", L_STICK, (0.0, 1.0, 0.0)),
        ("right", R_STICK, (0.0, 1.0, 0.0)),
    ):
        stick = model.part(f"{name}_thumbstick")
        stick.visual(
            _thumbstick_mesh(f"{name}_thumbstick"), material=btn_dark, name=f"{name}_stick"
        )
        stick.inertial = Inertial.from_geometry(
            Box((0.025, 0.025, 0.024)), mass=0.006
        )
        # Pivot at the base of the post on the top surface; tilt swings the cap.
        model.articulation(
            f"body_to_{name}_thumbstick",
            ArticulationType.REVOLUTE,
            parent=body,
            child=stick,
            origin=Origin(xyz=(sx, sy, TOP_Z)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=1.0, velocity=4.0, lower=-0.35, upper=0.35
            ),
        )

    # =====================================================================
    # D-PAD (revolute rocker)
    # =====================================================================
    dpad = model.part("dpad")
    dpad.visual(_dpad_mesh("dpad"), material=btn_dark, name="dpad_cross")
    dpad.inertial = Inertial.from_geometry(Box((0.026, 0.026, 0.008)), mass=0.005)
    model.articulation(
        "body_to_dpad",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dpad,
        origin=Origin(xyz=(DPAD_CTR[0], DPAD_CTR[1], TOP_Z + 0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-0.20, upper=0.20),
    )

    # =====================================================================
    # FACE BUTTONS A/B/X/Y (prismatic press down -Z)
    # diamond layout: Y top, A bottom, X left, B right
    # =====================================================================
    face_layout = {
        "y": (0.0, ABXY_R),
        "a": (0.0, -ABXY_R),
        "x": (-ABXY_R, 0.0),
        "b": (ABXY_R, 0.0),
    }
    for label, (dx, dy) in face_layout.items():
        bx = ABXY_CTR[0] + dx
        by = ABXY_CTR[1] + dy
        btn = model.part(f"face_btn_{label}")
        cap = CylinderGeometry(BTN_R, 0.006, radial_segments=24)
        cap.translate(0.0, 0.0, 0.003)
        # glow letter ring on top
        ring = CylinderGeometry(BTN_R - 0.002, 0.0015, radial_segments=24)
        ring.translate(0.0, 0.0, 0.0065)
        btn.visual(mesh_from_geometry(cap, f"face_btn_{label}_cap"), material=btn_dark,
                   name=f"face_btn_{label}_cap")
        btn.visual(mesh_from_geometry(ring, f"face_btn_{label}_glow"), material=glow,
                   name=f"face_btn_{label}_glow")
        btn.inertial = Inertial.from_geometry(Box((0.015, 0.015, 0.006)), mass=0.002)
        model.articulation(
            f"body_to_face_btn_{label}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=btn,
            origin=Origin(xyz=(bx, by, TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.0015),
        )

    # =====================================================================
    # SHOULDER TRIGGERS (revolute pull about the top-edge hinge)
    # =====================================================================
    for name, sx in (("trigger_l", -1.0), ("trigger_r", 1.0)):
        trig = model.part(name)
        trig.visual(_trigger_mesh(name), material=purple, name=f"{name}_paddle")
        trig.inertial = Inertial.from_geometry(Box((0.048, 0.012, 0.024)), mass=0.006)
        # Hinge along X at the top-front edge; positive q pulls the paddle up/back.
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=trig,
            origin=Origin(xyz=(sx * TRIG_X, TRIG_Y, 0.018)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=0.35),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lstick = object_model.get_part("left_thumbstick")
    rstick = object_model.get_part("right_thumbstick")
    dpad = object_model.get_part("dpad")
    trig_l = object_model.get_part("trigger_l")
    trig_r = object_model.get_part("trigger_r")

    l_joint = object_model.get_articulation("body_to_left_thumbstick")
    r_joint = object_model.get_articulation("body_to_right_thumbstick")
    dpad_joint = object_model.get_articulation("body_to_dpad")
    tl_joint = object_model.get_articulation("body_to_trigger_l")
    tr_joint = object_model.get_articulation("body_to_trigger_r")

    face_labels = ["a", "b", "x", "y"]
    face_btns = {lb: object_model.get_part(f"face_btn_{lb}") for lb in face_labels}

    # --- Seated overlaps: sticks in their dishes, buttons/dpad near the surface ---
    ctx.allow_overlap(
        lstick, body, elem_a="left_stick", elem_b="left_stick_dish",
        reason="Stick post is intentionally seated inside its recessed dish ring.",
    )
    ctx.allow_overlap(
        rstick, body, elem_a="right_stick", elem_b="right_stick_dish",
        reason="Stick post is intentionally seated inside its recessed dish ring.",
    )
    ctx.allow_overlap(
        dpad, body, elem_a="dpad_cross", elem_b="body_shell",
        reason="D-pad rocker hub is seated into a shallow well on the top surface.",
    )
    ctx.allow_overlap(
        trig_l, body, elem_a="trigger_l_paddle", elem_b="body_shell",
        reason="Left trigger paddle wraps and pivots on the body's top-front edge.",
    )
    ctx.allow_overlap(
        trig_r, body, elem_a="trigger_r_paddle", elem_b="body_shell",
        reason="Right trigger paddle wraps and pivots on the body's top-front edge.",
    )

    # --- Both thumbsticks present, mounted on the face, and TILT about their pivot ---
    for name, part, joint, ctr in (
        ("left", lstick, l_joint, L_STICK),
        ("right", rstick, r_joint, R_STICK),
    ):
        pos = ctx.part_world_position(part)
        ctx.check(
            f"{name} thumbstick mounted on the top face",
            pos is not None and pos[2] > TOP_Z - 0.005,
            details=f"{name} stick pos={pos}",
        )
        # Tilt swings the top of the stick sideways in X.
        top0 = ctx.part_world_aabb(part)[1][0]
        with ctx.pose({joint: 0.35}):
            topT = ctx.part_world_aabb(part)[1][0]
        ctx.check(
            f"{name} thumbstick tilts about its pivot",
            abs(topT - top0) > 0.003,
            details=f"{name} stick top-x rest={top0}, tilted={topT}",
        )

    # --- Four face buttons present, in a diamond on the RIGHT side ---
    for lb in face_labels:
        bp = ctx.part_world_position(face_btns[lb])
        ctx.check(
            f"face button {lb.upper()} present on the right",
            bp is not None and bp[0] > 0.02,
            details=f"face_btn_{lb} pos={bp}",
        )

    # --- A sampled face button presses straight down (-Z) ---
    sample = face_btns["a"]
    sample_joint = object_model.get_articulation("body_to_face_btn_a")
    z0 = ctx.part_world_position(sample)[2]
    with ctx.pose({sample_joint: 0.0015}):
        z1 = ctx.part_world_position(sample)[2]
    ctx.check(
        "face button A presses down",
        z1 < z0 - 0.0008,
        details=f"A z rest={z0}, pressed={z1}",
    )

    # --- D-pad articulates (rocker tilt swings it in Z on one side) ---
    dy0 = ctx.part_world_aabb(dpad)[1][2]
    with ctx.pose({dpad_joint: 0.20}):
        dy1 = ctx.part_world_aabb(dpad)[1][2]
    ctx.check(
        "dpad rocks about its hinge",
        abs(dy1 - dy0) > 0.001,
        details=f"dpad top-z rest={dy0}, rocked={dy1}",
    )

    # --- Both triggers pull about the top-edge hinge (paddle swings up in Z) ---
    for name, part, joint in (
        ("trigger_l", trig_l, tl_joint),
        ("trigger_r", trig_r, tr_joint),
    ):
        tz0 = ctx.part_world_aabb(part)[1][2]
        with ctx.pose({joint: 0.35}):
            tz1 = ctx.part_world_aabb(part)[1][2]
        ctx.check(
            f"{name} pulls about the top hinge",
            tz1 > tz0 + 0.002,
            details=f"{name} top-z rest={tz0}, pulled={tz1}",
        )

    # --- Body has two grips: geometry extends to both lower corners in X ---
    bb = ctx.part_world_aabb(body)
    width = bb[1][0] - bb[0][0]
    ctx.check(
        "body spans two grips across X",
        width > 0.12,
        details=f"body width={width}",
    )

    return ctx.report()


object_model = build_object_model()
