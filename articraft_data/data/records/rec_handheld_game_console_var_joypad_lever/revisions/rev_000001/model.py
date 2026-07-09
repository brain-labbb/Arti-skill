from __future__ import annotations

# Blue PSP-style handheld game console (FRONT-face layout).
# Frame:
#   +X = wide axis (right), +Y = tall axis (up), +Z = out of the front face.
#   Glossy blue slab centered at origin: ~0.17 wide (X) x 0.074 tall (Y) x 0.022 thick (Z).
#   Front face at z=+0.011, back at z=-0.011.
# Layout (front face):
#   - large black landscape SCREEN recessed in the center.
#   - tall JOYSTICK LEVER on the LEFT of the screen (-X), with ball-top knob.
#   - four round FACE BUTTONS on the RIGHT of the screen (+X).
#   - round analog NUB below the joystick (low dome on a post).
#   - small START / SELECT / HOME buttons under the screen.
#   - two shoulder buttons (L / R) on the TOP edge.
#   - speaker grilles flanking the screen.
# Articulations:
#   - joystick: REVOLUTE tilt about horizontal (X) axis (~+/-20deg).
#   - 4 face buttons: each PRISMATIC press down (~1.5 mm, -Z).
#   - analog nub: REVOLUTE tilt about a horizontal (X) axis (~+/-15deg).
#   - L / R shoulder buttons: each REVOLUTE press about the top-edge hinge (~0-18deg).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions ----
W = 0.170          # full width (X)
H = 0.074          # full height (Y)
T = 0.022          # full thickness (Z)
FRONT_Z = T / 2.0  # +0.011

SCREEN_W = 0.078
SCREEN_H = 0.044
SCREEN_CX = 0.0
SCREEN_CY = 0.006   # screen sits slightly above center

STICK_CX = -0.064    # joystick center (left grip, clear of screen bezel)
STICK_CY = 0.011
FACE_CX = 0.067     # face-button cluster center (right grip, clear of screen bezel)
FACE_CY = 0.011
NUB_CX = -0.060     # analog nub center (below joystick)
NUB_CY = -0.024

# Joystick lever dimensions
STICK_SHAFT_R = 0.003    # shaft radius
STICK_SHAFT_H = 0.028    # shaft height from pivot
STICK_BALL_R = 0.007     # ball top radius
COLLAR_R = 0.010         # collar outer radius
COLLAR_H = 0.005         # collar height above shell front face


def _slab_solid() -> cq.Workplane:
    # Glossy blue shell: a rounded landscape slab with sculpted bulging grip ends.
    # Central body: rounded box.
    body = (
        cq.Workplane("XY")
        .box(W - 0.030, H, T)
        .edges("|Z")
        .fillet(0.010)
        .edges("|X")
        .fillet(0.004)
    )
    # Sculpted, rounded grip ends (bulge out a touch and round generously).
    for sx in (-1.0, 1.0):
        grip = (
            cq.Workplane("XY")
            .center(sx * (W / 2.0 - 0.020), 0.0)
            .box(0.044, H + 0.004, T + 0.003)
            .edges("|Z")
            .fillet(0.018)
            .edges("|Y")
            .fillet(0.006)
        )
        body = body.union(grip)
    # Recess the central screen well into the front face.
    well = (
        cq.Workplane("XY")
        .workplane(offset=FRONT_Z)
        .center(SCREEN_CX, SCREEN_CY)
        .rect(SCREEN_W + 0.010, SCREEN_H + 0.010)
        .extrude(-0.004)
    )
    body = body.cut(well)
    return body


def _slab_mesh():
    return mesh_from_cadquery(_slab_solid(), "shell")


def _joystick_collar_solid() -> cq.Workplane:
    """Raised pivot collar on the shell front face for the joystick lever.
    A short cylinder with a central bore for the stick shaft.
    Extends from z=0 upward (+Z) in its local frame."""
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
    )
    # Bore hole for the stick shaft
    bore = (
        cq.Workplane("XY")
        .circle(STICK_SHAFT_R + 0.001)
        .extrude(COLLAR_H + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    collar = collar.cut(bore)
    # Fillet the top edge for a rounded look
    collar = collar.edges(">Z").fillet(0.0015)
    return collar


def _joystick_lever_mesh():
    """Build the joystick lever as a lathed form: a slim shaft with ball-top knob.
    The pivot point is at z=0; the shaft extends upward (+Z).
    The shaft base extends slightly below z=0 (-0.002) to seat inside the collar bore."""
    shaft_top = STICK_SHAFT_H
    ball_center_z = shaft_top + STICK_BALL_R * 0.6
    # Build profile as lathe points (radius, z):
    # Shaft extends 7mm below pivot to seat through the collar and into the shell body.
    profile = [
        (0.0, -0.007),                           # center bottom (deep inside shell)
        (STICK_SHAFT_R, -0.007),                 # shaft base outer (embedded in shell)
        (STICK_SHAFT_R, shaft_top - 0.002),      # shaft top before ball
        (STICK_SHAFT_R + 0.001, shaft_top),      # slight flare at neck
        (STICK_BALL_R, ball_center_z - 0.002),   # ball lower tangent
        (STICK_BALL_R, ball_center_z + 0.002),   # ball upper tangent
        (STICK_SHAFT_R + 0.001, ball_center_z + STICK_BALL_R * 0.8),  # above ball neck
        (0.0, ball_center_z + STICK_BALL_R),     # ball top center
    ]
    return LatheGeometry(profile, segments=24)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="handheld_game_console")

    # ---- materials ----
    blue = model.material("glossy_blue", rgba=(0.12, 0.52, 0.92, 1.0))
    black = model.material("screen_black", rgba=(0.03, 0.03, 0.05, 1.0))
    dark = model.material("dark_gray", rgba=(0.16, 0.16, 0.18, 1.0))
    btn_blue = model.material("button_blue", rgba=(0.20, 0.60, 0.95, 1.0))
    silver = model.material("silver", rgba=(0.78, 0.80, 0.83, 1.0))
    stick_mat = model.material("stick_black", rgba=(0.08, 0.08, 0.10, 1.0))

    # ============================================================
    # ROOT: the glossy blue shell + everything fused to it.
    # ============================================================
    body = model.part("body")
    body.visual(_slab_mesh(), material=blue, name="shell")

    # Recessed black SCREEN (the large central landscape face).
    body.visual(
        Box((SCREEN_W, SCREEN_H, 0.004)),
        origin=Origin(xyz=(SCREEN_CX, SCREEN_CY, FRONT_Z - 0.005)),
        material=black,
        name="screen",
    )
    # Thin silver bezel framing the screen (four thin bars).
    bez_t = 0.0020
    half_w = SCREEN_W / 2.0 + 0.003
    half_h = SCREEN_H / 2.0 + 0.003
    for nm, sz, off in (
        ("bezel_top", (SCREEN_W + 0.008, bez_t, 0.003), (0.0, half_h, 0.0)),
        ("bezel_bot", (SCREEN_W + 0.008, bez_t, 0.003), (0.0, -half_h, 0.0)),
        ("bezel_left", (bez_t, SCREEN_H + 0.008, 0.003), (-half_w, 0.0, 0.0)),
        ("bezel_right", (bez_t, SCREEN_H + 0.008, 0.003), (half_w, 0.0, 0.0)),
    ):
        body.visual(
            Box(sz),
            origin=Origin(xyz=(SCREEN_CX + off[0], SCREEN_CY + off[1], FRONT_Z - 0.0035)),
            material=silver,
            name=nm,
        )

    # Speaker grilles flanking the screen (perforated dark patches with ribs).
    for side, sx in (("left", -1.0), ("right", 1.0)):
        gx = sx * 0.044
        spk = mesh_from_geometry(_speaker_grille(), f"speaker_{side}")
        body.visual(
            spk,
            origin=Origin(xyz=(gx, -0.006, FRONT_Z - 0.0015)),
            material=dark,
            name=f"speaker_{side}",
        )

    # START / SELECT small buttons + round HOME button under the screen (static trim).
    for nm, ox in (("select", -0.014), ("start", 0.014)):
        body.visual(
            Box((0.012, 0.005, 0.003)),
            origin=Origin(xyz=(SCREEN_CX + ox, SCREEN_CY - SCREEN_H / 2.0 - 0.010, FRONT_Z - 0.001)),
            material=silver,
            name=f"{nm}_button",
        )
    body.visual(
        Cylinder(0.005, 0.003),
        origin=Origin(xyz=(SCREEN_CX, SCREEN_CY - SCREEN_H / 2.0 - 0.010, FRONT_Z - 0.001)),
        material=silver,
        name="home_button",
    )

    # PSP-style wordmark plate along the bottom front (small dark strip).
    body.visual(
        Box((0.030, 0.005, 0.0015)),
        origin=Origin(xyz=(0.0, -H / 2.0 + 0.006, FRONT_Z - 0.0005)),
        material=silver,
        name="logo_strip",
    )

    # Raised pivot COLLAR on the shell for the joystick lever.
    # The collar sits at the joystick position on the front face.
    body.visual(
        mesh_from_cadquery(_joystick_collar_solid(), "joystick_collar"),
        origin=Origin(xyz=(STICK_CX, STICK_CY, FRONT_Z)),
        material=dark,
        name="joystick_collar",
    )

    # Analog nub POST/well boss on the shell (raised collar around the pivot).
    body.visual(
        Cylinder(0.0085, 0.004),
        origin=Origin(xyz=(NUB_CX, NUB_CY, FRONT_Z - 0.001)),
        material=dark,
        name="nub_collar",
    )
    # Short fixed post stub that the nub pivots on (sits inside the collar).
    body.visual(
        Cylinder(0.0035, 0.008),
        origin=Origin(xyz=(NUB_CX, NUB_CY, FRONT_Z + 0.002)),
        material=dark,
        name="nub_post",
    )

    # Body inertial: approximate the slab.
    body.inertial = Inertial.from_geometry(
        Box((W, H, T)), mass=0.28, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ============================================================
    # JOYSTICK LEVER: tall slim stick with ball top, REVOLUTE tilt.
    # The part frame sits at the pivot point (collar top surface).
    # Geometry extends upward (+Z local = outward from front face).
    # Joint axis is X so positive q tilts the stick toward -Y.
    # ============================================================
    stick = model.part("joystick")
    stick.visual(
        mesh_from_geometry(_joystick_lever_mesh(), "joystick"),
        material=stick_mat,
        name="joystick",
    )
    stick.inertial = Inertial.from_geometry(
        Cylinder(0.007, 0.042), mass=0.008, origin=Origin(xyz=(0.0, 0.0, 0.021))
    )
    model.articulation(
        "body_to_joystick",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stick,
        # Joint origin at the collar top surface (the actual pivot contact).
        origin=Origin(xyz=(STICK_CX, STICK_CY, FRONT_Z + COLLAR_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0,
            lower=-math.radians(20.0), upper=math.radians(20.0),
        ),
    )

    # ============================================================
    # FOUR FACE BUTTONS: each round, PRISMATIC press down (~1.5 mm).
    # Diamond arrangement: top, right, bottom, left within the cluster.
    # ============================================================
    face_offsets = [
        (0.0, 0.0105),    # face_btn_0 (top, triangle)
        (0.0105, 0.0),    # face_btn_1 (right, circle)
        (0.0, -0.0105),   # face_btn_2 (bottom, cross)
        (-0.0105, 0.0),   # face_btn_3 (left, square)
    ]
    for i, (ox, oy) in enumerate(face_offsets):
        btn = model.part(f"face_btn_{i}")
        cap = DomeGeometry(0.0072, radial_segments=24, height_segments=10).scale(1.0, 1.0, 0.45)
        stem = CylinderGeometry(0.0066, 0.005, radial_segments=20)
        stem.translate(0.0, 0.0, -0.0025)
        cap.merge(stem)
        btn.visual(mesh_from_geometry(cap, f"face_btn_{i}"), material=btn_blue, name=f"face_btn_{i}")
        btn.inertial = Inertial.from_geometry(Cylinder(0.0072, 0.006), mass=0.0015)
        model.articulation(
            f"body_to_face_btn_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=btn,
            origin=Origin(xyz=(FACE_CX + ox, FACE_CY + oy, FRONT_Z + 0.001)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=0.0015),
        )

    # ============================================================
    # ANALOG NUB: low dome on a post, REVOLUTE tilt about X axis (~+/-15deg).
    # The nub geometry sits above the joint so a positive tilt swings the top.
    # ============================================================
    nub = model.part("analog_nub")
    nub_dome = DomeGeometry(0.0072, radial_segments=24, height_segments=12).scale(1.0, 1.0, 0.5)
    nub_dome.translate(0.0, 0.0, 0.0055)  # sits above pivot
    nub_stem = CylinderGeometry(0.0028, 0.006, radial_segments=18)
    nub_stem.translate(0.0, 0.0, 0.0025)
    nub_dome.merge(nub_stem)
    nub.visual(mesh_from_geometry(nub_dome, "analog_nub"), material=dark, name="analog_nub")
    nub.inertial = Inertial.from_geometry(Cylinder(0.0072, 0.008), mass=0.003)
    model.articulation(
        "body_to_analog_nub",
        ArticulationType.REVOLUTE,
        parent=body,
        child=nub,
        origin=Origin(xyz=(NUB_CX, NUB_CY, FRONT_Z + 0.0015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0, lower=-math.radians(15.0), upper=math.radians(15.0)
        ),
    )

    # ============================================================
    # SHOULDER BUTTONS (L / R): each REVOLUTE press about the top-edge hinge.
    # Hinge line runs along X near the top-back corner; positive q presses
    # the front lip downward (~0-18deg).
    # ============================================================
    top_y = H / 2.0
    back_z = -T / 2.0
    for side, sx, vname in (("l", -1.0, "shoulder_l"), ("r", 1.0, "shoulder_r")):
        sh = model.part(f"shoulder_{side}")
        # Trigger pad: a wide curved bar; geometry extends forward (+Z) and
        # outward from the hinge so a positive rotation about +X presses it down.
        sh.visual(
            mesh_from_cadquery(_shoulder_solid(), vname),
            material=blue,
            name=vname,
        )
        sh.inertial = Inertial.from_geometry(Box((0.030, 0.010, 0.012)), mass=0.004)
        model.articulation(
            f"body_to_shoulder_{side}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=sh,
            # Hinge sits at the top edge, toward the back, over each end.
            origin=Origin(xyz=(sx * 0.058, top_y - 0.004, back_z + 0.006)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=2.0, lower=0.0, upper=math.radians(18.0)
            ),
        )

    return model


def _speaker_grille():
    # Small oval grille: an array of tiny recessed holes implied by thin ribs.
    g = CylinderGeometry(0.006, 0.001, radial_segments=20)
    for dx in (-0.004, 0.0, 0.004):
        slot = CylinderGeometry(0.0012, 0.0014, radial_segments=10)
        slot.translate(dx, 0.0, 0.0008)
        g.merge(slot)
    return g


def _shoulder_solid() -> cq.Workplane:
    # L/R trigger bar: a curved pad that extends forward from the hinge.
    # Built in the child frame so that, at q=0, it lies along the top edge with
    # its mass forward (+Z) and slightly down (-Y) from the hinge line.
    bar = (
        cq.Workplane("XY")
        .center(0.0, 0.006)
        .box(0.034, 0.012, 0.010)
        .edges("|X")
        .fillet(0.004)
        .edges("|Z")
        .fillet(0.004)
    )
    bar = bar.translate((0.0, -0.002, 0.008))
    return bar


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    stick = object_model.get_part("joystick")
    nub = object_model.get_part("analog_nub")
    f0 = object_model.get_part("face_btn_0")
    f1 = object_model.get_part("face_btn_1")
    sh_l = object_model.get_part("shoulder_l")
    sh_r = object_model.get_part("shoulder_r")

    stick_joint = object_model.get_articulation("body_to_joystick")
    nub_joint = object_model.get_articulation("body_to_analog_nub")
    f0_joint = object_model.get_articulation("body_to_face_btn_0")
    shl_joint = object_model.get_articulation("body_to_shoulder_l")
    shr_joint = object_model.get_articulation("body_to_shoulder_r")

    # ---- seated-overlap allowances (stick in collar, nub on post, buttons, hinges) ----
    ctx.allow_overlap(
        stick, body, elem_a="joystick", elem_b="joystick_collar",
        reason="Joystick lever shaft pivots inside the raised collar bore.",
    )
    ctx.allow_overlap(
        stick, body, elem_a="joystick", elem_b="shell",
        reason="Joystick lever shaft base inserts through the collar into the shell aperture.",
    )
    ctx.allow_overlap(
        nub, body, elem_a="analog_nub", elem_b="nub_collar",
        reason="Analog nub stem sits inside its raised collar boss.",
    )
    ctx.allow_overlap(
        nub, body, elem_a="analog_nub", elem_b="nub_post",
        reason="Analog nub pivots on the fixed post stub nested inside it.",
    )
    for i in range(4):
        fb = object_model.get_part(f"face_btn_{i}")
        ctx.allow_overlap(
            fb, body, elem_a=f"face_btn_{i}", elem_b="shell",
            reason="Face button stem seats into its shell well.",
        )
    ctx.allow_overlap(
        sh_l, body, elem_a="shoulder_l", elem_b="shell",
        reason="Left shoulder trigger wraps over the top-edge hinge of the shell.",
    )
    ctx.allow_overlap(
        sh_r, body, elem_a="shoulder_r", elem_b="shell",
        reason="Right shoulder trigger wraps over the top-edge hinge of the shell.",
    )

    # ---- screen is the large central recessed black face ----
    scr_aabb = ctx.part_element_world_aabb(body, elem="screen")
    (smnx, smny, smnz), (smxx, smxy, smxz) = scr_aabb
    ctx.check(
        "screen is a large landscape central face",
        (smxx - smnx) > 0.06 and (smxy - smny) > 0.03 and (smxx - smnx) > (smxy - smny),
        details=f"screen extents x={smxx-smnx:.3f}, y={smxy-smny:.3f}",
    )
    ctx.check(
        "screen recessed near the front face (z<=front)",
        smxz <= FRONT_Z + 1e-4,
        details=f"screen max z={smxz:.4f} vs front {FRONT_Z:.4f}",
    )

    # ---- joystick left of screen, face buttons right of screen ----
    stick_x = ctx.part_world_position(stick)[0]
    f1_x = ctx.part_world_position(f1)[0]
    ctx.check(
        "joystick is left of the screen",
        stick_x < smnx,
        details=f"stick_x={stick_x:.3f}, screen_min_x={smnx:.3f}",
    )
    ctx.check(
        "face buttons are right of the screen",
        f1_x > smxx,
        details=f"face_btn_1_x={f1_x:.3f}, screen_max_x={smxx:.3f}",
    )

    # ---- joystick lever is tall (extends well above the front face) ----
    stick_aabb = ctx.part_world_aabb(stick)
    stick_height = stick_aabb[1][2] - stick_aabb[0][2]
    ctx.check(
        "joystick lever is tall (at least 25mm)",
        stick_height > 0.025,
        details=f"stick height={stick_height:.3f}",
    )
    stick_zmax = stick_aabb[1][2]
    collar_zmax = ctx.part_element_world_aabb(body, elem="joystick_collar")[1][2]
    ctx.check(
        "joystick ball top extends well above the collar",
        stick_zmax > collar_zmax + 0.020,
        details=f"stick_zmax={stick_zmax:.4f}, collar_zmax={collar_zmax:.4f}",
    )

    # ---- joystick tilts about its horizontal pivot: ball top swings in Y ----
    # The stick extends +Z from the pivot; positive rotation about +X
    # swings the top toward -Y. Track the AABB Y-center.
    stick_rest_aabb = ctx.part_world_aabb(stick)
    stick_rest_ymid = 0.5 * (stick_rest_aabb[0][1] + stick_rest_aabb[1][1])
    with ctx.pose({stick_joint: math.radians(20.0)}):
        stick_tilt_pos = ctx.part_world_aabb(stick)
        stick_pos_ymid = 0.5 * (stick_tilt_pos[0][1] + stick_tilt_pos[1][1])
    with ctx.pose({stick_joint: -math.radians(20.0)}):
        stick_tilt_neg = ctx.part_world_aabb(stick)
        stick_neg_ymid = 0.5 * (stick_tilt_neg[0][1] + stick_tilt_neg[1][1])
    ctx.check(
        "joystick tilts about its horizontal pivot (ball swings in Y)",
        (stick_pos_ymid < stick_rest_ymid - 0.003)
        and (stick_neg_ymid > stick_rest_ymid + 0.003),
        details=f"rest_ymid={stick_rest_ymid:.4f} pos={stick_pos_ymid:.4f} neg={stick_neg_ymid:.4f}",
    )

    # ---- joystick joint origin is at the collar surface, not far away ----
    joint_origin = object_model.get_articulation("body_to_joystick")
    ctx.check(
        "joystick joint origin sits at or above the front face",
        joint_origin.origin.xyz[2] >= FRONT_Z + COLLAR_H - 0.002,
        details=f"joint_z={joint_origin.origin.xyz[2]:.4f}, expected>={FRONT_Z + COLLAR_H:.4f}",
    )

    # ---- a sampled face button presses down (toward -Z) ----
    rest_z = ctx.part_world_position(f0)[2]
    with ctx.pose({f0_joint: 0.0015}):
        pressed_z = ctx.part_world_position(f0)[2]
    ctx.check(
        "face button presses down (-Z)",
        pressed_z < rest_z - 0.0010,
        details=f"rest_z={rest_z:.4f}, pressed_z={pressed_z:.4f}",
    )
    ctx.expect_contact(f0, body, name="face button seated in shell")

    # ---- analog nub tilts about its pivot: dome (above pivot) swings in Y ----
    nub_rest_aabb = ctx.part_world_aabb(nub)
    nub_rest_ymid = 0.5 * (nub_rest_aabb[0][1] + nub_rest_aabb[1][1])
    nub_rest_zmax = nub_rest_aabb[1][2]
    with ctx.pose({nub_joint: math.radians(15.0)}):
        nub_tilt_aabb = ctx.part_world_aabb(nub)
        nub_tilt_ymid = 0.5 * (nub_tilt_aabb[0][1] + nub_tilt_aabb[1][1])
        nub_tilt_zmax = nub_tilt_aabb[1][2]
    with ctx.pose({nub_joint: -math.radians(15.0)}):
        nub_neg_aabb = ctx.part_world_aabb(nub)
        nub_neg_ymid = 0.5 * (nub_neg_aabb[0][1] + nub_neg_aabb[1][1])
    ctx.check(
        "analog nub tilts about its horizontal pivot",
        (nub_tilt_ymid < nub_rest_ymid - 0.0006)
        and (nub_neg_ymid > nub_rest_ymid + 0.0006),
        details=f"rest_ymid={nub_rest_ymid:.4f} pos_ymid={nub_tilt_ymid:.4f} "
        f"neg_ymid={nub_neg_ymid:.4f} (zmax rest={nub_rest_zmax:.4f} pos={nub_tilt_zmax:.4f})",
    )
    ctx.check(
        "analog nub is below the joystick",
        ctx.part_world_position(nub)[1] < ctx.part_world_position(stick)[1],
        details=f"nub_y={ctx.part_world_position(nub)[1]:.3f}, stick_y={ctx.part_world_position(stick)[1]:.3f}",
    )

    # ---- both shoulder buttons press about the top-edge hinge ----
    shl_rest = ctx.part_world_aabb(sh_l)
    with ctx.pose({shl_joint: math.radians(18.0)}):
        shl_press = ctx.part_world_aabb(sh_l)
    ctx.check(
        "left shoulder presses about top hinge",
        shl_press[0][2] < shl_rest[0][2] - 0.0008 or shl_press[0][1] < shl_rest[0][1] - 0.0008,
        details=f"rest_minZ={shl_rest[0][2]:.4f} press_minZ={shl_press[0][2]:.4f} "
        f"rest_minY={shl_rest[0][1]:.4f} press_minY={shl_press[0][1]:.4f}",
    )
    shr_rest = ctx.part_world_aabb(sh_r)
    with ctx.pose({shr_joint: math.radians(18.0)}):
        shr_press = ctx.part_world_aabb(sh_r)
    ctx.check(
        "right shoulder presses about top hinge",
        shr_press[0][2] < shr_rest[0][2] - 0.0008 or shr_press[0][1] < shr_rest[0][1] - 0.0008,
        details=f"rest_minZ={shr_rest[0][2]:.4f} press_minZ={shr_press[0][2]:.4f} "
        f"rest_minY={shr_rest[0][1]:.4f} press_minY={shr_press[0][1]:.4f}",
    )
    # Shoulders sit on the top edge of the body.
    ctx.check(
        "shoulders are near the top edge",
        ctx.part_world_position(sh_l)[1] > 0.02 and ctx.part_world_position(sh_r)[1] > 0.02,
        details=f"shL_y={ctx.part_world_position(sh_l)[1]:.3f}, shR_y={ctx.part_world_position(sh_r)[1]:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
