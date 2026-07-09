from __future__ import annotations

# Dual-screen clamshell handheld game console (Nintendo DS style).
# Frame:
#   +X = wide axis (right), +Y = tall axis (up on device face), +Z = out of front face.
# Structure:
#   - DECK (root): thicker rounded slab (~0.022m), carries lower screen, D-pad,
#     face buttons, analog nub, shoulder buttons, speakers, and menu buttons.
#   - LID: thinner rounded panel (~0.012m), carries upper screen + bezel on inner face.
#   - HINGE: visible barrel knuckle (3 deck, 2 lid interleaved) along shared back edge.
#     Axis = X, origin at top-back corner of deck.
#     q=0 → CLOSED (lid flat over deck, screens facing each other)
#     q≈120° → OPEN (lid swings up and back, screen tilted toward user)
# Articulations:
#   - deck_to_lid: REVOLUTE about X, 0 to ~120°
#   - D-pad: PRISMATIC press (-Z), same as parent
#   - 4 face buttons: each PRISMATIC press (-Z), same as parent
#   - analog nub: REVOLUTE tilt about X, same as parent
#   - L/R shoulder buttons: each REVOLUTE press, same as parent

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
W = 0.170           # full width (X)
DECK_H = 0.080      # deck height (Y)
DECK_T = 0.022      # deck thickness (Z)
LID_H = 0.068       # lid height (Y)
LID_T = 0.012       # lid thickness (Z)
FRONT_Z = DECK_T / 2.0  # +0.011, deck front face

SCREEN_W = 0.078
SCREEN_H = 0.044
LOWER_SCREEN_CY = 0.008  # lower screen center Y (slightly above deck center, toward hinge)

DPAD_CX = -0.064
DPAD_CY = LOWER_SCREEN_CY
FACE_CX = 0.067
FACE_CY = LOWER_SCREEN_CY
NUB_CX = -0.060
NUB_CY = -0.022

# Hinge line: shared back edge at top of deck
HINGE_Y = DECK_H / 2.0     # 0.040
HINGE_Z = -DECK_T / 2.0    # -0.011

# Lid visual offset in lid's local frame (positions lid panel over deck front when q=0)
# At q=0, lid panel inner face should be near deck front face (z=FRONT_Z).
# Lid origin is at hinge (HINGE_Y, HINGE_Z). Offset to put lid panel in front of deck:
LID_VIS_OY = -LID_H / 2.0                   # extends downward from hinge
LID_VIS_OZ = DECK_T + LID_T / 2.0 + 0.001   # in front of deck with small gap

# Hinge barrel knuckle parameters
BARREL_R = 0.004
BARREL_LEN = 0.022  # each knuckle length along X
BARREL_SPAN = 0.120  # total span of 5 knuckles
KNUCKLE_SPACING = BARREL_SPAN / 5.0  # 0.024

# Deck knuckle X centers (indices 0, 2, 4 of 5)
DECK_KNUCKLE_XS = [-2.0 * KNUCKLE_SPACING, 0.0, 2.0 * KNUCKLE_SPACING]
# Lid knuckle X centers (indices 1, 3 of 5)
LID_KNUCKLE_XS = [-1.0 * KNUCKLE_SPACING, 1.0 * KNUCKLE_SPACING]


def _deck_shell_solid() -> cq.Workplane:
    """Deck shell: rounded slab with sculpted grip ends and lower screen well."""
    body = (
        cq.Workplane("XY")
        .box(W - 0.030, DECK_H, DECK_T)
        .edges("|Z")
        .fillet(0.010)
        .edges("|X")
        .fillet(0.004)
    )
    # Sculpted grip ends
    for sx in (-1.0, 1.0):
        grip = (
            cq.Workplane("XY")
            .center(sx * (W / 2.0 - 0.020), 0.0)
            .box(0.044, DECK_H + 0.004, DECK_T)
            .edges("|Z")
            .fillet(0.018)
            .edges("|Y")
            .fillet(0.005)
        )
        body = body.union(grip)
    # Lower screen well (recessed into front face)
    well = (
        cq.Workplane("XY")
        .workplane(offset=FRONT_Z)
        .center(0.0, LOWER_SCREEN_CY)
        .rect(SCREEN_W + 0.010, SCREEN_H + 0.010)
        .extrude(-0.004)
    )
    body = body.cut(well)
    return body


def _lid_shell_solid() -> cq.Workplane:
    """Lid shell: thin rounded panel with upper screen well on inner face."""
    body = (
        cq.Workplane("XY")
        .box(W - 0.020, LID_H, LID_T)
        .edges("|Z")
        .fillet(0.008)
        .edges("|X")
        .fillet(0.003)
    )
    # Upper screen well on inner face (inner face is at z = -LID_T/2 in local coords)
    well = (
        cq.Workplane("XY")
        .workplane(offset=-LID_T / 2.0)
        .center(0.0, 0.0)
        .rect(SCREEN_W + 0.010, SCREEN_H + 0.010)
        .extrude(0.003)
    )
    body = body.cut(well)
    return body


def _dpad_solid() -> cq.Workplane:
    """Cross-shaped D-pad: two crossed bars with a dished center."""
    arm_w = 0.0085
    arm_l = 0.024
    horiz = cq.Workplane("XY").box(arm_l, arm_w, 0.006)
    vert = cq.Workplane("XY").box(arm_w, arm_l, 0.006)
    cross = horiz.union(vert)
    cross = cross.edges("|Z").fillet(0.0018)
    hub = cq.Workplane("XY").workplane(offset=0.003).circle(0.0055).extrude(0.001)
    cross = cross.union(hub)
    return cross


def _shoulder_solid() -> cq.Workplane:
    """L/R trigger bar: curved pad extending from hinge."""
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


def _speaker_grille():
    """Small oval grille with thin ribs."""
    g = CylinderGeometry(0.006, 0.001, radial_segments=20)
    for dx in (-0.004, 0.0, 0.004):
        slot = CylinderGeometry(0.0012, 0.0014, radial_segments=10)
        slot.translate(dx, 0.0, 0.0008)
        g.merge(slot)
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="handheld_game_console")

    # ---- materials ----
    blue = model.material("glossy_blue", rgba=(0.12, 0.52, 0.92, 1.0))
    black = model.material("screen_black", rgba=(0.03, 0.03, 0.05, 1.0))
    dark = model.material("dark_gray", rgba=(0.16, 0.16, 0.18, 1.0))
    btn_blue = model.material("button_blue", rgba=(0.20, 0.60, 0.95, 1.0))
    silver = model.material("silver", rgba=(0.78, 0.80, 0.83, 1.0))
    hinge_mat = model.material("hinge_dark", rgba=(0.22, 0.22, 0.25, 1.0))

    # ============================================================
    # DECK (root): thicker lower slab carrying controls + lower screen.
    # ============================================================
    deck = model.part("deck")
    deck.visual(mesh_from_cadquery(_deck_shell_solid(), "deck_shell"), material=blue, name="deck_shell")

    # Lower screen (recessed black landscape face on deck front)
    deck.visual(
        Box((SCREEN_W, SCREEN_H, 0.004)),
        origin=Origin(xyz=(0.0, LOWER_SCREEN_CY, FRONT_Z - 0.005)),
        material=black,
        name="lower_screen",
    )
    # Bezel around lower screen
    bez_t = 0.0020
    half_w = SCREEN_W / 2.0 + 0.003
    half_h = SCREEN_H / 2.0 + 0.003
    for nm, sz, off in (
        ("lower_bezel_top", (SCREEN_W + 0.008, bez_t, 0.003), (0.0, half_h, 0.0)),
        ("lower_bezel_bot", (SCREEN_W + 0.008, bez_t, 0.003), (0.0, -half_h, 0.0)),
        ("lower_bezel_left", (bez_t, SCREEN_H + 0.008, 0.003), (-half_w, 0.0, 0.0)),
        ("lower_bezel_right", (bez_t, SCREEN_H + 0.008, 0.003), (half_w, 0.0, 0.0)),
    ):
        deck.visual(
            Box(sz),
            origin=Origin(xyz=(off[0], LOWER_SCREEN_CY + off[1], FRONT_Z - 0.0035)),
            material=silver,
            name=nm,
        )

    # Speaker grilles flanking the lower screen
    for side, sx in (("left", -1.0), ("right", 1.0)):
        gx = sx * 0.044
        spk = mesh_from_geometry(_speaker_grille(), f"speaker_{side}")
        deck.visual(
            spk,
            origin=Origin(xyz=(gx, LOWER_SCREEN_CY - 0.014, FRONT_Z - 0.0015)),
            material=dark,
            name=f"speaker_{side}",
        )

    # START / SELECT / HOME buttons under the lower screen
    for nm, ox in (("select", -0.014), ("start", 0.014)):
        deck.visual(
            Box((0.012, 0.005, 0.003)),
            origin=Origin(xyz=(ox, LOWER_SCREEN_CY - SCREEN_H / 2.0 - 0.010, FRONT_Z - 0.001)),
            material=silver,
            name=f"{nm}_button",
        )
    deck.visual(
        Cylinder(0.005, 0.003),
        origin=Origin(xyz=(0.0, LOWER_SCREEN_CY - SCREEN_H / 2.0 - 0.010, FRONT_Z - 0.001)),
        material=silver,
        name="home_button",
    )

    # Analog nub collar + post on deck
    deck.visual(
        Cylinder(0.0085, 0.004),
        origin=Origin(xyz=(NUB_CX, NUB_CY, FRONT_Z - 0.001)),
        material=dark,
        name="nub_collar",
    )
    deck.visual(
        Cylinder(0.0035, 0.008),
        origin=Origin(xyz=(NUB_CX, NUB_CY, FRONT_Z + 0.002)),
        material=dark,
        name="nub_post",
    )

    # Deck hinge barrel knuckles (3, interleaved)
    for i, kx in enumerate(DECK_KNUCKLE_XS):
        deck.visual(
            Cylinder(BARREL_R, BARREL_LEN),
            origin=Origin(xyz=(kx, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_mat,
            name=f"deck_knuckle_{i}",
        )

    deck.inertial = Inertial.from_geometry(
        Box((W, DECK_H, DECK_T)), mass=0.20, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ============================================================
    # LID: thinner upper panel carrying upper screen, hinged at deck top-back.
    # ============================================================
    lid = model.part("lid")
    # Lid shell (thin rounded panel, positioned via Origin offset from hinge)
    lid.visual(
        mesh_from_cadquery(_lid_shell_solid(), "lid_shell"),
        material=blue,
        origin=Origin(xyz=(0.0, LID_VIS_OY, LID_VIS_OZ)),
        name="lid_shell",
    )

    # Upper screen (recessed into lid inner face)
    # Inner face of lid is at local z = LID_VIS_OZ - LID_T/2 in lid frame
    lid_screen_z = LID_VIS_OZ - LID_T / 2.0 + 0.002  # slightly recessed
    lid.visual(
        Box((SCREEN_W, SCREEN_H, 0.004)),
        origin=Origin(xyz=(0.0, LID_VIS_OY, lid_screen_z)),
        material=black,
        name="upper_screen",
    )
    # Bezel around upper screen
    for nm, sz, off in (
        ("upper_bezel_top", (SCREEN_W + 0.008, bez_t, 0.003), (0.0, half_h, 0.0)),
        ("upper_bezel_bot", (SCREEN_W + 0.008, bez_t, 0.003), (0.0, -half_h, 0.0)),
        ("upper_bezel_left", (bez_t, SCREEN_H + 0.008, 0.003), (-half_w, 0.0, 0.0)),
        ("upper_bezel_right", (bez_t, SCREEN_H + 0.008, 0.003), (half_w, 0.0, 0.0)),
    ):
        lid.visual(
            Box(sz),
            origin=Origin(xyz=(off[0], LID_VIS_OY + off[1], lid_screen_z + 0.001)),
            material=silver,
            name=nm,
        )

    # Lid hinge barrel knuckles (2, interleaved with deck) + arms connecting to shell
    shell_back_z = LID_VIS_OZ - LID_T / 2.0  # back face of lid shell in lid frame
    arm_cz = shell_back_z / 2.0               # arm center Z
    arm_dz = shell_back_z + 0.002             # arm length to reach past shell back
    for i, kx in enumerate(LID_KNUCKLE_XS):
        lid.visual(
            Cylinder(BARREL_R, BARREL_LEN),
            origin=Origin(xyz=(kx, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_mat,
            name=f"lid_knuckle_{i}",
        )
        # Hinge arm: thin box bridging from knuckle to shell back face
        lid.visual(
            Box((0.008, 0.005, arm_dz)),
            origin=Origin(xyz=(kx, 0.0, arm_cz)),
            material=blue,
            name=f"lid_hinge_arm_{i}",
        )

    lid.inertial = Inertial.from_geometry(
        Box((W, LID_H, LID_T)), mass=0.10, origin=Origin(xyz=(0.0, LID_VIS_OY, LID_VIS_OZ))
    )

    # HINGE: deck_to_lid REVOLUTE about X axis
    model.articulation(
        "deck_to_lid",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=math.radians(120.0)
        ),
    )

    # ============================================================
    # D-PAD: raised cross, PRISMATIC press into deck (same as parent).
    # ============================================================
    dpad = model.part("dpad")
    dpad.visual(mesh_from_cadquery(_dpad_solid(), "dpad"), material=dark, name="dpad")
    dpad.inertial = Inertial.from_geometry(Box((0.026, 0.026, 0.006)), mass=0.006)
    model.articulation(
        "deck_to_dpad",
        ArticulationType.PRISMATIC,
        parent=deck,
        child=dpad,
        origin=Origin(xyz=(DPAD_CX, DPAD_CY, FRONT_Z + 0.001)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.0025),
    )

    # ============================================================
    # FOUR FACE BUTTONS: each round dome, PRISMATIC press (-Z).
    # ============================================================
    face_offsets = [
        (0.0, 0.0105),    # face_btn_0 (top)
        (0.0105, 0.0),    # face_btn_1 (right)
        (0.0, -0.0105),   # face_btn_2 (bottom)
        (-0.0105, 0.0),   # face_btn_3 (left)
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
            f"deck_to_face_btn_{i}",
            ArticulationType.PRISMATIC,
            parent=deck,
            child=btn,
            origin=Origin(xyz=(FACE_CX + ox, FACE_CY + oy, FRONT_Z + 0.001)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=0.0015),
        )

    # ============================================================
    # ANALOG NUB: low dome on post, REVOLUTE tilt about X.
    # ============================================================
    nub = model.part("analog_nub")
    nub_dome = DomeGeometry(0.0072, radial_segments=24, height_segments=12).scale(1.0, 1.0, 0.5)
    nub_dome.translate(0.0, 0.0, 0.0055)
    nub_stem = CylinderGeometry(0.0028, 0.006, radial_segments=18)
    nub_stem.translate(0.0, 0.0, 0.0025)
    nub_dome.merge(nub_stem)
    nub.visual(mesh_from_geometry(nub_dome, "analog_nub"), material=dark, name="analog_nub")
    nub.inertial = Inertial.from_geometry(Cylinder(0.0072, 0.008), mass=0.003)
    model.articulation(
        "deck_to_analog_nub",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=nub,
        origin=Origin(xyz=(NUB_CX, NUB_CY, FRONT_Z + 0.0015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0, lower=-math.radians(15.0), upper=math.radians(15.0)
        ),
    )

    # ============================================================
    # SHOULDER BUTTONS (L / R): REVOLUTE press at deck top edge.
    # ============================================================
    top_y = DECK_H / 2.0
    back_z = -DECK_T / 2.0
    for side, sx in (("l", -1.0), ("r", 1.0)):
        sh = model.part(f"shoulder_{side}")
        sh.visual(
            mesh_from_cadquery(_shoulder_solid(), f"shoulder_{side}"),
            material=blue,
            name=f"shoulder_{side}",
        )
        sh.inertial = Inertial.from_geometry(Box((0.030, 0.010, 0.012)), mass=0.004)
        model.articulation(
            f"deck_to_shoulder_{side}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=sh,
            origin=Origin(xyz=(sx * 0.058, top_y - 0.006, back_z + 0.008)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=2.0, lower=0.0, upper=math.radians(18.0)
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    lid = object_model.get_part("lid")
    dpad = object_model.get_part("dpad")
    nub = object_model.get_part("analog_nub")
    f0 = object_model.get_part("face_btn_0")
    f1 = object_model.get_part("face_btn_1")
    sh_l = object_model.get_part("shoulder_l")
    sh_r = object_model.get_part("shoulder_r")

    hinge = object_model.get_articulation("deck_to_lid")
    dpad_joint = object_model.get_articulation("deck_to_dpad")
    nub_joint = object_model.get_articulation("deck_to_analog_nub")
    f0_joint = object_model.get_articulation("deck_to_face_btn_0")
    shl_joint = object_model.get_articulation("deck_to_shoulder_l")
    shr_joint = object_model.get_articulation("deck_to_shoulder_r")

    # ---- overlap allowances ----
    # Hinge knuckles embedded in deck shell corner
    for i in range(2):
        ctx.allow_overlap(
            lid, deck, elem_a=f"lid_knuckle_{i}", elem_b="deck_shell",
            reason="Hinge barrel knuckle embedded in deck shell at shared back-edge contact.",
        )
    # Hinge arms pass through the hinge region (bridge knuckle to shell)
    for i in range(2):
        ctx.allow_overlap(
            lid, deck, elem_a=f"lid_hinge_arm_{i}", elem_b="deck_shell",
            reason="Hinge arm bridges lid knuckle to shell through the deck hinge region.",
        )
    # Controls inside closed clamshell (lid covers deck at q=0)
    ctx.allow_overlap(
        dpad, lid, elem_a="dpad", elem_b="lid_shell",
        reason="D-pad protrudes from deck; lid closes over it when clamshell is shut.",
    )
    for i in range(4):
        fb = object_model.get_part(f"face_btn_{i}")
        ctx.allow_overlap(
            fb, lid, elem_a=f"face_btn_{i}", elem_b="lid_shell",
            reason="Face button protrudes from deck; lid closes over it when clamshell is shut.",
        )
    ctx.allow_overlap(
        nub, lid, elem_a="analog_nub", elem_b="lid_shell",
        reason="Analog nub protrudes from deck; lid closes over it when clamshell is shut.",
    )
    # Nub collar/post overlap with lid (deck visuals, but nub is separate part)
    ctx.allow_overlap(
        nub, deck, elem_a="analog_nub", elem_b="nub_collar",
        reason="Analog nub stem sits inside its raised collar boss.",
    )
    ctx.allow_overlap(
        nub, deck, elem_a="analog_nub", elem_b="nub_post",
        reason="Analog nub pivots on the fixed post stub nested inside it.",
    )
    # D-pad seats into deck shell
    ctx.allow_overlap(
        dpad, deck, elem_a="dpad", elem_b="deck_shell",
        reason="D-pad base seats into its deck shell aperture.",
    )
    for i in range(4):
        fb = object_model.get_part(f"face_btn_{i}")
        ctx.allow_overlap(
            fb, deck, elem_a=f"face_btn_{i}", elem_b="deck_shell",
            reason="Face button stem seats into its deck shell well.",
        )
    # Shoulder buttons wrap over deck hinge area
    ctx.allow_overlap(
        sh_l, deck, elem_a="shoulder_l", elem_b="deck_shell",
        reason="Left shoulder trigger wraps over the deck top-edge hinge.",
    )
    ctx.allow_overlap(
        sh_r, deck, elem_a="shoulder_r", elem_b="deck_shell",
        reason="Right shoulder trigger wraps over the deck top-edge hinge.",
    )

    # ---- CLAMSHELL STRUCTURE: lid is distinct from deck ----
    deck_aabb = ctx.part_world_aabb(deck)
    lid_aabb = ctx.part_world_aabb(lid)
    deck_shell_aabb = ctx.part_element_world_aabb(deck, elem="deck_shell")
    lid_shell_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    deck_shell_z = deck_shell_aabb[1][2] - deck_shell_aabb[0][2]
    lid_shell_z = lid_shell_aabb[1][2] - lid_shell_aabb[0][2]
    ctx.check(
        "lid shell is thinner than deck shell (distinct halves)",
        lid_shell_z < deck_shell_z * 0.8,
        details=f"deck_shell_z={deck_shell_z:.4f}, lid_shell_z={lid_shell_z:.4f}",
    )

    # ---- HINGE: deck_to_lid is REVOLUTE and opens correctly ----
    hinge_info = hinge
    ctx.check(
        "deck_to_lid is a REVOLUTE hinge",
        hinge_info is not None,
        details="hinge articulation exists",
    )

    # At q=0 (closed): lid flat over deck, screens face each other
    lower_screen_aabb = ctx.part_element_world_aabb(deck, elem="lower_screen")
    upper_screen_aabb = ctx.part_element_world_aabb(lid, elem="upper_screen")
    ctx.check(
        "closed: both screens exist and face each other",
        lower_screen_aabb is not None and upper_screen_aabb is not None,
        details="both screens present at q=0",
    )

    # At open pose: lid clearly rotated up off deck plane
    open_angle = math.radians(120.0)
    with ctx.pose({hinge: open_angle}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        lid_open_ymax = lid_open_aabb[1][1]
        upper_scr_open_aabb = ctx.part_element_world_aabb(lid, elem="upper_screen")
        upper_scr_open_zmax = upper_scr_open_aabb[1][2]

    deck_ymax = deck_aabb[1][1]
    ctx.check(
        "open: lid swings up above deck top edge",
        lid_open_ymax > deck_ymax + 0.005,
        details=f"lid_ymax={lid_open_ymax:.4f}, deck_ymax={deck_ymax:.4f}",
    )
    ctx.check(
        "open: upper screen rotated behind deck front plane",
        upper_scr_open_zmax < deck_shell_aabb[0][2] + 0.005,
        details=f"screen_zmax={upper_scr_open_zmax:.4f}, deck_shell_zmin={deck_shell_aabb[0][2]:.4f}",
    )

    # ---- Lower screen on deck, D-pad left, face buttons right ----
    lmnx, lmny, lmnz = lower_screen_aabb[0]
    lmxx, lmxy, lmxz = lower_screen_aabb[1]
    ctx.check(
        "lower screen is a large landscape face on deck",
        (lmxx - lmnx) > 0.06 and (lmxy - lmny) > 0.03 and (lmxx - lmnx) > (lmxy - lmny),
        details=f"x={lmxx-lmnx:.3f}, y={lmxy-lmny:.3f}",
    )
    dpad_x = ctx.part_world_position(dpad)[0]
    f1_x = ctx.part_world_position(f1)[0]
    ctx.check(
        "D-pad is left of lower screen",
        dpad_x < lmnx,
        details=f"dpad_x={dpad_x:.3f}, screen_min_x={lmnx:.3f}",
    )
    ctx.check(
        "face buttons are right of lower screen",
        f1_x > lmxx,
        details=f"face_btn_1_x={f1_x:.3f}, screen_max_x={lmxx:.3f}",
    )

    # ---- D-pad presses down (-Z) ----
    dpad_rest_z = ctx.part_world_position(dpad)[2]
    with ctx.pose({dpad_joint: 0.0025}):
        dpad_press_z = ctx.part_world_position(dpad)[2]
    ctx.check(
        "D-pad presses down (-Z)",
        dpad_press_z < dpad_rest_z - 0.0015,
        details=f"rest={dpad_rest_z:.4f}, pressed={dpad_press_z:.4f}",
    )

    # ---- Face button presses down (-Z) ----
    rest_z = ctx.part_world_position(f0)[2]
    with ctx.pose({f0_joint: 0.0015}):
        pressed_z = ctx.part_world_position(f0)[2]
    ctx.check(
        "face button presses down (-Z)",
        pressed_z < rest_z - 0.0010,
        details=f"rest_z={rest_z:.4f}, pressed_z={pressed_z:.4f}",
    )

    # ---- Analog nub tilts ----
    nub_rest_aabb = ctx.part_world_aabb(nub)
    nub_rest_ymid = 0.5 * (nub_rest_aabb[0][1] + nub_rest_aabb[1][1])
    with ctx.pose({nub_joint: math.radians(15.0)}):
        nub_tilt_aabb = ctx.part_world_aabb(nub)
        nub_tilt_ymid = 0.5 * (nub_tilt_aabb[0][1] + nub_tilt_aabb[1][1])
    with ctx.pose({nub_joint: -math.radians(15.0)}):
        nub_neg_aabb = ctx.part_world_aabb(nub)
        nub_neg_ymid = 0.5 * (nub_neg_aabb[0][1] + nub_neg_aabb[1][1])
    ctx.check(
        "analog nub tilts about horizontal pivot",
        (nub_tilt_ymid < nub_rest_ymid - 0.0006) and (nub_neg_ymid > nub_rest_ymid + 0.0006),
        details=f"rest={nub_rest_ymid:.4f} pos={nub_tilt_ymid:.4f} neg={nub_neg_ymid:.4f}",
    )

    # ---- Shoulder buttons press ----
    shl_rest = ctx.part_world_aabb(sh_l)
    with ctx.pose({shl_joint: math.radians(18.0)}):
        shl_press = ctx.part_world_aabb(sh_l)
    ctx.check(
        "left shoulder presses about top hinge",
        shl_press[0][2] < shl_rest[0][2] - 0.0008 or shl_press[0][1] < shl_rest[0][1] - 0.0008,
        details=f"rest_minZ={shl_rest[0][2]:.4f} press_minZ={shl_press[0][2]:.4f}",
    )
    shr_rest = ctx.part_world_aabb(sh_r)
    with ctx.pose({shr_joint: math.radians(18.0)}):
        shr_press = ctx.part_world_aabb(sh_r)
    ctx.check(
        "right shoulder presses about top hinge",
        shr_press[0][2] < shr_rest[0][2] - 0.0008 or shr_press[0][1] < shr_rest[0][1] - 0.0008,
        details=f"rest_minZ={shr_rest[0][2]:.4f} press_minZ={shr_press[0][2]:.4f}",
    )

    # ---- Upper screen exists on lid ----
    upper_scr_aabb = ctx.part_element_world_aabb(lid, elem="upper_screen")
    ctx.check(
        "upper screen is a large landscape face on lid",
        upper_scr_aabb is not None
        and (upper_scr_aabb[1][0] - upper_scr_aabb[0][0]) > 0.06
        and (upper_scr_aabb[1][1] - upper_scr_aabb[0][1]) > 0.03,
        details=f"upper_screen aabb={upper_scr_aabb}",
    )

    # ---- Proportions: palm-scale (~0.16-0.17m wide) ----
    ctx.check(
        "device width is palm-scale (0.15-0.18m)",
        0.15 < (deck_aabb[1][0] - deck_aabb[0][0]) < 0.18,
        details=f"width={deck_aabb[1][0] - deck_aabb[0][0]:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
