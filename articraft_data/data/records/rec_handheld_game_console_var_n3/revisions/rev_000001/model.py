from __future__ import annotations

# Blue PSP-style handheld game console (FRONT-face layout).
# Frame:
#   +X = wide axis (right), +Y = tall axis (up), +Z = out of the front face.
#   Glossy blue slab centered at origin: ~0.17 wide (X) x 0.074 tall (Y) x 0.022 thick (Z).
#   Front face at z=+0.011, back at z=-0.011.
# Layout (front face):
#   - large black landscape SCREEN recessed in the center.
#   - cross D-PAD on the LEFT of the screen (-X).
#   - three round FACE BUTTONS on the RIGHT of the screen (+X).
#   - round analog NUB below the D-pad (low dome on a post).
#   - small START / SELECT / HOME buttons under the screen.
#   - two shoulder buttons (L / R) on the TOP edge.
#   - speaker grilles flanking the screen.
# Articulations:
#   - D-pad: PRISMATIC press down (into the shell, -Z).
#   - 3 face buttons: each PRISMATIC press down (~1.5 mm, -Z).
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
    MotionLimits,
    Origin,
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

DPAD_CX = -0.064    # D-pad center (left grip, clear of screen bezel)
DPAD_CY = 0.011
FACE_CX = 0.067     # face-button cluster center (right grip, clear of screen bezel)
FACE_CY = 0.011
NUB_CX = -0.060     # analog nub center (below D-pad)
NUB_CY = -0.024


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="handheld_game_console")

    # ---- materials ----
    blue = model.material("glossy_blue", rgba=(0.12, 0.52, 0.92, 1.0))
    black = model.material("screen_black", rgba=(0.03, 0.03, 0.05, 1.0))
    dark = model.material("dark_gray", rgba=(0.16, 0.16, 0.18, 1.0))
    btn_blue = model.material("button_blue", rgba=(0.20, 0.60, 0.95, 1.0))
    silver = model.material("silver", rgba=(0.78, 0.80, 0.83, 1.0))

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
    # D-PAD: a raised cross, PRISMATIC press into the shell.
    # ============================================================
    dpad = model.part("dpad")
    dpad.visual(mesh_from_cadquery(_dpad_solid(), "dpad"), material=dark, name="dpad")
    dpad.inertial = Inertial.from_geometry(Box((0.026, 0.026, 0.006)), mass=0.006)
    model.articulation(
        "body_to_dpad",
        ArticulationType.PRISMATIC,
        parent=body,
        child=dpad,
        origin=Origin(xyz=(DPAD_CX, DPAD_CY, FRONT_Z + 0.001)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.0025),
    )

    # ============================================================
    # THREE FACE BUTTONS: each round, PRISMATIC press down (~1.5 mm).
    # Equilateral triangle arrangement (120° apart) on the right grip.
    # ============================================================
    _N_FACE = 3
    _FACE_CLUSTER_R = 0.0105  # radius of the triangular layout
    for i in range(_N_FACE):
        angle = math.radians(90.0 + i * 120.0)  # start at top, go CCW
        ox = _FACE_CLUSTER_R * math.cos(angle)
        oy = _FACE_CLUSTER_R * math.sin(angle)
        btn = model.part(f"face_btn_{i}")
        btn.visual(
            mesh_from_geometry(_face_button_mesh(i), f"face_btn_{i}"),
            material=btn_blue,
            name=f"face_btn_{i}",
        )
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
    for side, sx in (("l", -1.0), ("r", 1.0)):
        sh = model.part(f"shoulder_{side}")
        # Trigger pad: a wide curved bar; geometry extends forward (+Z) and
        # outward from the hinge so a positive rotation about +X presses it down.
        sh.visual(
            mesh_from_cadquery(_shoulder_solid(), f"shoulder_{side}"),
            material=blue,
            name=f"shoulder_{side}",
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


def _face_button_mesh(i: int):
    """Shared dome+stem geometry for face button i. Identical across all 3 buttons."""
    cap = DomeGeometry(0.0072, radial_segments=24, height_segments=10).scale(1.0, 1.0, 0.45)
    stem = CylinderGeometry(0.0066, 0.005, radial_segments=20)
    stem.translate(0.0, 0.0, -0.0025)
    cap.merge(stem)
    return cap


def _speaker_grille():
    # Small oval grille: an array of tiny recessed holes implied by thin ribs.
    g = CylinderGeometry(0.006, 0.001, radial_segments=20)
    for dx in (-0.004, 0.0, 0.004):
        slot = CylinderGeometry(0.0012, 0.0014, radial_segments=10)
        slot.translate(dx, 0.0, 0.0008)
        g.merge(slot)
    return g


def _dpad_solid() -> cq.Workplane:
    # Cross-shaped D-pad: two crossed bars with a dished center, raised.
    arm_w = 0.0085
    arm_l = 0.024
    horiz = cq.Workplane("XY").box(arm_l, arm_w, 0.006)
    vert = cq.Workplane("XY").box(arm_w, arm_l, 0.006)
    cross = horiz.union(vert)
    cross = cross.edges("|Z").fillet(0.0018)
    # Slight dished center hub.
    hub = cq.Workplane("XY").workplane(offset=0.003).circle(0.0055).extrude(0.001)
    cross = cross.union(hub)
    return cross


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
    dpad = object_model.get_part("dpad")
    nub = object_model.get_part("analog_nub")
    f0 = object_model.get_part("face_btn_0")
    f1 = object_model.get_part("face_btn_1")
    sh_l = object_model.get_part("shoulder_l")
    sh_r = object_model.get_part("shoulder_r")

    dpad_joint = object_model.get_articulation("body_to_dpad")
    nub_joint = object_model.get_articulation("body_to_analog_nub")
    f0_joint = object_model.get_articulation("body_to_face_btn_0")
    shl_joint = object_model.get_articulation("body_to_shoulder_l")
    shr_joint = object_model.get_articulation("body_to_shoulder_r")

    # ---- seated-overlap allowances (buttons in wells, nub on post, hinges) ----
    ctx.allow_overlap(
        dpad, body, elem_a="dpad", elem_b="shell",
        reason="D-pad base seats into its shell well/aperture.",
    )
    ctx.allow_overlap(
        nub, body, elem_a="analog_nub", elem_b="nub_collar",
        reason="Analog nub stem sits inside its raised collar boss.",
    )
    ctx.allow_overlap(
        nub, body, elem_a="analog_nub", elem_b="nub_post",
        reason="Analog nub pivots on the fixed post stub nested inside it.",
    )
    for i in range(3):
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
    screen = body.get_visual("screen")
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

    # ---- D-pad left of screen, face buttons right of screen ----
    dpad_x = ctx.part_world_position(dpad)[0]
    f1_x = ctx.part_world_position(f1)[0]
    ctx.check(
        "D-pad is left of the screen",
        dpad_x < smnx,
        details=f"dpad_x={dpad_x:.3f}, screen_min_x={smnx:.3f}",
    )
    ctx.check(
        "face buttons are right of the screen",
        f1_x > smxx,
        details=f"face_btn_1_x={f1_x:.3f}, screen_max_x={smxx:.3f}",
    )

    # ---- exactly 3 face buttons (not 4) ----
    face_btns = [object_model.get_part(f"face_btn_{i}") for i in range(3)]
    ctx.check(
        "exactly 3 face buttons exist",
        all(fb is not None for fb in face_btns),
        details="face_btn_0, face_btn_1, face_btn_2 must all exist",
    )
    all_part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no 4th face button",
        "face_btn_3" not in all_part_names,
        details=f"found face_btn_3 in parts list" if "face_btn_3" in all_part_names else "",
    )

    # ---- face buttons form a triangle with roughly equal spacing ----
    fb_positions = [ctx.part_world_position(fb) for fb in face_btns]
    fb_dists = []
    for j in range(3):
        k = (j + 1) % 3
        dx = fb_positions[j][0] - fb_positions[k][0]
        dy = fb_positions[j][1] - fb_positions[k][1]
        fb_dists.append(math.sqrt(dx * dx + dy * dy))
    ctx.check(
        "face buttons form a regular triangle (equal spacing)",
        max(fb_dists) - min(fb_dists) < 0.003,
        details=f"pairwise distances: {[f'{d:.4f}' for d in fb_dists]}",
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

    # ---- D-pad articulates (press down) ----
    dpad_rest_z = ctx.part_world_position(dpad)[2]
    with ctx.pose({dpad_joint: 0.0025}):
        dpad_press_z = ctx.part_world_position(dpad)[2]
    ctx.check(
        "D-pad presses down (-Z)",
        dpad_press_z < dpad_rest_z - 0.0015,
        details=f"rest_z={dpad_rest_z:.4f}, pressed_z={dpad_press_z:.4f}",
    )

    # ---- analog nub tilts about its pivot: dome (above pivot) swings in Y ----
    # The dome sits at +Z above the pivot, so a positive rotation about +X
    # swings its mass toward -Y (and lowers its top). Track the AABB Y-center.
    nub_rest_aabb = ctx.part_world_aabb(nub)
    nub_rest_ymid = 0.5 * (nub_rest_aabb[0][1] + nub_rest_aabb[1][1])
    nub_rest_zmax = nub_rest_aabb[1][2]
    with ctx.pose({nub_joint: math.radians(15.0)}):
        nub_tilt_aabb = ctx.part_world_aabb(nub)
        nub_tilt_ymid = 0.5 * (nub_tilt_aabb[0][1] + nub_tilt_aabb[1][1])
        nub_tilt_zmax = nub_tilt_aabb[1][2]
    # Negative-angle tilt for an unambiguous symmetric check (mass swings +Y).
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
    nub_x = ctx.part_world_position(nub)[0]
    ctx.check(
        "analog nub is below the D-pad",
        ctx.part_world_position(nub)[1] < dpad_x * 0.0 + ctx.part_world_position(dpad)[1],
        details=f"nub_y={ctx.part_world_position(nub)[1]:.3f}",
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
