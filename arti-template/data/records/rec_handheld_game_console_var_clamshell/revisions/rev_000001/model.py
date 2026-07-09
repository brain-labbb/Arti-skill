from __future__ import annotations

# Clamshell foldable handheld game console (GBA SP style).
# Two distinct halves joined by a revolute barrel hinge along the shared back edge.
#
# Frame:
#   +X = wide axis (right), +Y = up (when open), +Z = toward user (front).
#   Deck (lower half): chunky rounded slab with controls on top surface (+Y).
#   Lid  (upper half): slimmer rounded panel carrying screen on inner face (-Y).
#
# Hinge:
#   Barrel knuckles along X at the shared back-top edge.
#   axis = (-1, 0, 0) so positive q opens the lid upward.
#   q=0   → closed (lid folded flat over deck, screen facing controls).
#   q≈2.09 (120°) → open (lid stands up tilted toward user).
#
# Controls (on deck top surface):
#   - D-pad (left), PRISMATIC press -Y.
#   - 4 face buttons (right), each PRISMATIC press -Y.
#   - Analog nub (below D-pad), REVOLUTE tilt about X.
#   - L/R shoulder buttons (back edge), REVOLUTE press.

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
D_DECK = 0.078      # deck depth (Z, front-to-back)
T_DECK = 0.022      # deck thickness (Y)
D_LID = 0.076       # lid panel depth/height (Z in local → Y when open)
T_LID = 0.012       # lid thickness (Y when closed)

DECK_TOP = T_DECK / 2.0    # +0.011
DECK_FRONT = D_DECK / 2.0  # +0.039
DECK_BACK = -D_DECK / 2.0  # -0.039

# Hinge axis slightly above deck top for control clearance
HINGE_Y = DECK_TOP + 0.004  # 0.015
HINGE_Z = DECK_BACK          # -0.039

# Screen (on lid inner face)
SCREEN_W = 0.078
SCREEN_H = 0.044

# Control positions on deck top surface (X, Z)
DPAD_X = -0.064
DPAD_Z = 0.010
FACE_X = 0.067
FACE_Z = 0.010
NUB_X = -0.060
NUB_Z = -0.020

# Hinge barrel
BARREL_R = 0.005
KNUCKLE_W = 0.010


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------

def _deck_solid() -> cq.Workplane:
    """Chunky rounded slab for the deck (lower control half).
    Centered at origin; controls sit on the +Y face."""
    body = (
        cq.Workplane("XY")
        .box(W - 0.030, T_DECK, D_DECK)
        .edges("|Y")
        .fillet(0.010)
        .edges("|X")
        .fillet(0.004)
    )
    # Sculpted rounded grip ends
    for sx in (-1.0, 1.0):
        grip = (
            cq.Workplane("XY")
            .center(sx * (W / 2.0 - 0.020), 0.0)
            .box(0.044, T_DECK + 0.003, D_DECK * 0.55)
            .edges("|Y")
            .fillet(0.018)
            .edges("|X")
            .fillet(0.005)
        )
        body = body.union(grip)
    return body


def _lid_solid() -> cq.Workplane:
    """Slimmer rounded panel for the lid (upper screen half).
    In local frame: extends from origin (hinge) in +Z,
    thickness in +Y from Y=0 upward."""
    body = (
        cq.Workplane("XY")
        .box(W - 0.020, T_LID, D_LID)
        .edges("|Y")
        .fillet(0.007)
        .edges("|X")
        .fillet(0.003)
    )
    # Shift so hinge end is at Z=0 and inner face at Y=0
    body = body.translate((0.0, T_LID / 2.0, D_LID / 2.0))
    return body


def _dpad_solid() -> cq.Workplane:
    """Cross-shaped D-pad in the XZ plane (for the deck top surface)."""
    arm_w = 0.0085
    arm_l = 0.024
    horiz = cq.Workplane("XY").box(arm_l, 0.006, arm_w)
    vert = cq.Workplane("XY").box(arm_w, 0.006, arm_l)
    cross = horiz.union(vert)
    cross = cross.edges("|Y").fillet(0.0018)
    # Slight dished center hub
    hub = cq.Workplane("XY").workplane(offset=0.003).circle(0.0055).extrude(0.001)
    cross = cross.union(hub)
    return cross


def _shoulder_solid() -> cq.Workplane:
    """L/R trigger bar for the deck back edge."""
    bar = (
        cq.Workplane("XY")
        .box(0.034, 0.006, 0.012)
        .edges("|Y")
        .fillet(0.003)
        .edges("|X")
        .fillet(0.002)
    )
    # Shift so pivot is at the front edge (the back end presses down)
    bar = bar.translate((0.0, 0.0, -0.006))
    return bar


def _speaker_grille():
    """Small grille: array of recessed slots."""
    g = CylinderGeometry(0.006, 0.001, radial_segments=20)
    for dx in (-0.004, 0.0, 0.004):
        slot = CylinderGeometry(0.0012, 0.0014, radial_segments=10)
        slot.translate(dx, 0.0, 0.0008)
        g.merge(slot)
    return g


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamshell_handheld")

    # ---- materials ----
    blue = model.material("glossy_blue", rgba=(0.12, 0.52, 0.92, 1.0))
    black = model.material("screen_black", rgba=(0.03, 0.03, 0.05, 1.0))
    dark = model.material("dark_gray", rgba=(0.16, 0.16, 0.18, 1.0))
    btn_blue = model.material("button_blue", rgba=(0.20, 0.60, 0.95, 1.0))
    silver = model.material("silver", rgba=(0.78, 0.80, 0.83, 1.0))

    # ============================================================
    # DECK: lower control half (root part).
    # ============================================================
    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(_deck_solid(), "deck_shell"),
        material=blue, name="deck_shell",
    )

    # Hinge barrel knuckles on deck (3 knuckles, interleaved with lid's 2)
    deck_knuckle_xs = [-0.020, 0.0, 0.020]
    for i, kx in enumerate(deck_knuckle_xs):
        deck.visual(
            Cylinder(BARREL_R, KNUCKLE_W),
            origin=Origin(
                xyz=(kx, HINGE_Y, HINGE_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),  # rotate Z-axis cylinder to X-axis
            ),
            material=dark,
            name=f"deck_knuckle_{i}",
        )

    # Analog nub collar + post on deck surface
    deck.visual(
        Cylinder(0.0085, 0.004),
        origin=Origin(
            xyz=(NUB_X, DECK_TOP + 0.002, NUB_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),  # Z-axis → Y-axis
        ),
        material=dark, name="nub_collar",
    )
    deck.visual(
        Cylinder(0.0035, 0.006),
        origin=Origin(
            xyz=(NUB_X, DECK_TOP + 0.003, NUB_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark, name="nub_post",
    )

    # Start / Select / Home buttons on deck surface (static trim)
    for nm, ox in (("select", -0.012), ("start", 0.012)):
        deck.visual(
            Box((0.010, 0.003, 0.005)),
            origin=Origin(xyz=(ox, DECK_TOP + 0.001, -0.005)),
            material=silver, name=f"{nm}_button",
        )
    deck.visual(
        Cylinder(0.004, 0.003),
        origin=Origin(
            xyz=(0.0, DECK_TOP + 0.001, -0.005),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=silver, name="home_button",
    )

    # Speaker grilles on deck surface (flanking center)
    for side, sx in (("left", -1.0), ("right", 1.0)):
        gx = sx * 0.044
        spk = mesh_from_geometry(_speaker_grille(), f"speaker_{side}")
        deck.visual(
            spk,
            origin=Origin(
                xyz=(gx, DECK_TOP + 0.0005, -0.008),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark, name=f"speaker_{side}",
        )

    deck.inertial = Inertial.from_geometry(
        Box((W, T_DECK, D_DECK)), mass=0.20, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ============================================================
    # LID: upper screen half, connected via hinge.
    # ============================================================
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=blue, name="lid_shell",
    )

    # Hinge barrel knuckles on lid (2 knuckles)
    lid_knuckle_xs = [-0.010, 0.010]
    for i, kx in enumerate(lid_knuckle_xs):
        lid.visual(
            Cylinder(BARREL_R, KNUCKLE_W),
            origin=Origin(
                xyz=(kx, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark, name=f"lid_knuckle_{i}",
        )

    # Screen on lid inner face (Y=0 in local, recessed slightly)
    lid.visual(
        Box((SCREEN_W, 0.003, SCREEN_H)),
        origin=Origin(xyz=(0.0, -0.001, D_LID / 2.0)),
        material=black, name="screen",
    )

    # Bezel frame around screen (four thin bars)
    bez_t = 0.002
    half_sw = SCREEN_W / 2.0 + 0.003
    half_sh = SCREEN_H / 2.0 + 0.003
    bezel_specs = [
        ("bezel_top", (SCREEN_W + 0.008, 0.003, bez_t), (0.0, 0.0, half_sh)),
        ("bezel_bot", (SCREEN_W + 0.008, 0.003, bez_t), (0.0, 0.0, -half_sh)),
        ("bezel_left", (bez_t, 0.003, SCREEN_H + 0.008), (-half_sw, 0.0, 0.0)),
        ("bezel_right", (bez_t, 0.003, SCREEN_H + 0.008), (half_sw, 0.0, 0.0)),
    ]
    for nm, sz, off in bezel_specs:
        lid.visual(
            Box(sz),
            origin=Origin(xyz=(off[0], -0.0005, D_LID / 2.0 + off[2])),
            material=silver, name=nm,
        )

    # Logo strip on lid outer face (top center when open), seated on surface
    lid.visual(
        Box((0.030, 0.0015, 0.005)),
        origin=Origin(xyz=(0.0, T_LID, D_LID * 0.85)),
        material=silver, name="logo_strip",
    )

    lid.inertial = Inertial.from_geometry(
        Box((W, T_LID, D_LID)), mass=0.12,
        origin=Origin(xyz=(0.0, T_LID / 2.0, D_LID / 2.0)),
    )

    # ---- HINGE: deck to lid, REVOLUTE ----
    model.articulation(
        "deck_to_lid",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )

    # ============================================================
    # D-PAD: raised cross on deck, PRISMATIC press -Y.
    # ============================================================
    dpad = model.part("dpad")
    dpad.visual(
        mesh_from_cadquery(_dpad_solid(), "dpad"),
        material=dark, name="dpad",
    )
    dpad.inertial = Inertial.from_geometry(Box((0.026, 0.006, 0.026)), mass=0.006)
    model.articulation(
        "deck_to_dpad",
        ArticulationType.PRISMATIC,
        parent=deck, child=dpad,
        origin=Origin(xyz=(DPAD_X, DECK_TOP, DPAD_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.0025),
    )

    # ============================================================
    # FOUR FACE BUTTONS: round dome buttons, PRISMATIC press -Y.
    # ============================================================
    face_offsets = [
        (0.0, 0.0105),    # face_btn_0 (top)
        (0.0105, 0.0),    # face_btn_1 (right)
        (0.0, -0.0105),   # face_btn_2 (bottom)
        (-0.0105, 0.0),   # face_btn_3 (left)
    ]
    for i, (ox, oz) in enumerate(face_offsets):
        btn = model.part(f"face_btn_{i}")
        # Dome + stem geometry (built along Z, rotated to Y via visual rpy)
        cap = DomeGeometry(0.0072, radial_segments=24, height_segments=10).scale(1.0, 1.0, 0.45)
        stem = CylinderGeometry(0.0066, 0.005, radial_segments=20)
        stem.translate(0.0, 0.0, -0.0025)
        cap.merge(stem)
        btn.visual(
            mesh_from_geometry(cap, f"face_btn_{i}"),
            material=btn_blue,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),  # dome Z → Y
            name=f"face_btn_{i}",
        )
        btn.inertial = Inertial.from_geometry(Cylinder(0.0072, 0.006), mass=0.0015)
        model.articulation(
            f"deck_to_face_btn_{i}",
            ArticulationType.PRISMATIC,
            parent=deck, child=btn,
            origin=Origin(xyz=(FACE_X + ox, DECK_TOP, FACE_Z + oz)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=0.0015),
        )

    # ============================================================
    # ANALOG NUB: low dome on deck, REVOLUTE tilt about X.
    # ============================================================
    nub = model.part("analog_nub")
    nub_dome = DomeGeometry(0.0072, radial_segments=24, height_segments=12).scale(1.0, 1.0, 0.35)
    nub_dome.translate(0.0, 0.0, 0.003)
    nub_stem = CylinderGeometry(0.0028, 0.004, radial_segments=18)
    nub_stem.translate(0.0, 0.0, 0.001)
    nub_dome.merge(nub_stem)
    nub.visual(
        mesh_from_geometry(nub_dome, "analog_nub"),
        material=dark,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),  # dome Z → Y
        name="analog_nub",
    )
    nub.inertial = Inertial.from_geometry(Cylinder(0.0072, 0.006), mass=0.003)
    model.articulation(
        "deck_to_analog_nub",
        ArticulationType.REVOLUTE,
        parent=deck, child=nub,
        origin=Origin(xyz=(NUB_X, DECK_TOP + 0.001, NUB_Z)),
        axis=(1.0, 0.0, 0.0),  # tilt about X (forward/backward)
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0,
            lower=-math.radians(15.0), upper=math.radians(15.0),
        ),
    )

    # ============================================================
    # SHOULDER BUTTONS (L/R): REVOLUTE press at deck back edge.
    # ============================================================
    for side, sx in (("l", -1.0), ("r", 1.0)):
        sh = model.part(f"shoulder_{side}")
        sh.visual(
            mesh_from_cadquery(_shoulder_solid(), f"shoulder_{side}"),
            material=blue, name=f"shoulder_{side}",
        )
        sh.inertial = Inertial.from_geometry(Box((0.030, 0.006, 0.012)), mass=0.004)
        model.articulation(
            f"deck_to_shoulder_{side}",
            ArticulationType.REVOLUTE,
            parent=deck, child=sh,
            origin=Origin(xyz=(sx * 0.058, DECK_TOP + 0.001, DECK_BACK + 0.010)),
            axis=(-1.0, 0.0, 0.0),  # positive q presses back end down
            motion_limits=MotionLimits(
                effort=2.0, velocity=2.0,
                lower=0.0, upper=math.radians(18.0),
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

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

    # ---- Overlap allowances ----

    # Controls seated in deck
    ctx.allow_overlap(
        dpad, deck, elem_a="dpad", elem_b="deck_shell",
        reason="D-pad base seats into deck surface aperture.",
    )
    ctx.allow_overlap(
        nub, deck, elem_a="analog_nub", elem_b="nub_collar",
        reason="Analog nub stem sits inside its raised collar boss.",
    )
    ctx.allow_overlap(
        nub, deck, elem_a="analog_nub", elem_b="nub_post",
        reason="Analog nub pivots on the fixed post stub.",
    )
    for i in range(4):
        fb = object_model.get_part(f"face_btn_{i}")
        ctx.allow_overlap(
            fb, deck, elem_a=f"face_btn_{i}", elem_b="deck_shell",
            reason="Face button stem seats into deck well.",
        )
    ctx.allow_overlap(
        sh_l, deck, elem_a="shoulder_l", elem_b="deck_shell",
        reason="Left shoulder trigger mounts at deck back edge.",
    )
    ctx.allow_overlap(
        sh_r, deck, elem_a="shoulder_r", elem_b="deck_shell",
        reason="Right shoulder trigger mounts at deck back edge.",
    )

    # Hinge knuckle interleaving: deck knuckles overlap lid shell,
    # lid knuckles overlap deck shell/bracket at the shared pivot.
    for i in range(3):
        ctx.allow_overlap(
            deck, lid, elem_a=f"deck_knuckle_{i}", elem_b="lid_shell",
            reason="Deck hinge knuckle interleaves with lid shell at barrel pivot.",
        )
    for i in range(2):
        ctx.allow_overlap(
            lid, deck, elem_a=f"lid_knuckle_{i}", elem_b="deck_shell",
            reason="Lid hinge knuckle interleaves with deck bracket at barrel pivot.",
        )

    # ---- Hinge test: lid opens from closed to standing up ----
    # The lid part origin is at the hinge, so part_world_position doesn't change.
    # Use AABB center to track the lid panel swinging upward.
    lid_closed_aabb = ctx.part_world_aabb(lid)
    lid_closed_center_y = 0.5 * (lid_closed_aabb[0][1] + lid_closed_aabb[1][1])

    with ctx.pose({hinge: math.radians(90.0)}):
        lid_90_aabb = ctx.part_world_aabb(lid)
        lid_90_center_y = 0.5 * (lid_90_aabb[0][1] + lid_90_aabb[1][1])
    ctx.check(
        "lid at 90° stands above deck",
        lid_90_center_y > DECK_TOP + 0.02,
        details=f"lid_90_center_y={lid_90_center_y:.4f}, deck_top={DECK_TOP:.4f}",
    )

    with ctx.pose({hinge: math.radians(120.0)}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        lid_open_center_y = 0.5 * (lid_open_aabb[0][1] + lid_open_aabb[1][1])
    ctx.check(
        "lid hinge opens: lid center rises when hinge rotates to 120°",
        lid_open_center_y > lid_closed_center_y + 0.01,
        details=f"closed_center_y={lid_closed_center_y:.4f}, open_center_y={lid_open_center_y:.4f}",
    )

    # At open pose, verify the lid is clearly rotated off the deck plane
    deck_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "open lid extends well above deck (not coplanar)",
        lid_open_aabb[1][1] > deck_aabb[1][1] + 0.03,
        details=f"lid_max_y={lid_open_aabb[1][1]:.4f}, deck_max_y={deck_aabb[1][1]:.4f}",
    )

    # ---- Screen is on lid ----
    screen = lid.get_visual("screen")
    ctx.check(
        "screen exists on lid inner face",
        screen is not None,
        details="lid should carry the screen visual",
    )
    # At open pose, screen should be elevated above deck
    with ctx.pose({hinge: math.radians(90.0)}):
        scr_aabb = ctx.part_element_world_aabb(lid, elem="screen")
    ctx.check(
        "screen elevated above deck when lid open at 90°",
        scr_aabb[0][1] > deck_aabb[1][1] - 0.005,
        details=f"screen_min_y={scr_aabb[0][1]:.4f}, deck_max_y={deck_aabb[1][1]:.4f}",
    )

    # ---- D-pad presses down (-Y) ----
    dpad_rest_y = ctx.part_world_position(dpad)[1]
    with ctx.pose({dpad_joint: 0.0025}):
        dpad_press_y = ctx.part_world_position(dpad)[1]
    ctx.check(
        "D-pad presses down (-Y)",
        dpad_press_y < dpad_rest_y - 0.0015,
        details=f"rest_y={dpad_rest_y:.4f}, pressed_y={dpad_press_y:.4f}",
    )

    # ---- Face button presses down (-Y) ----
    f0_rest_y = ctx.part_world_position(f0)[1]
    with ctx.pose({f0_joint: 0.0015}):
        f0_press_y = ctx.part_world_position(f0)[1]
    ctx.check(
        "face button presses down (-Y)",
        f0_press_y < f0_rest_y - 0.0010,
        details=f"rest_y={f0_rest_y:.4f}, pressed_y={f0_press_y:.4f}",
    )
    ctx.expect_contact(f0, deck, name="face button seated in deck")

    # ---- D-pad left of center, face buttons right ----
    dpad_x = ctx.part_world_position(dpad)[0]
    f1_x = ctx.part_world_position(f1)[0]
    ctx.check(
        "D-pad is on left side of deck",
        dpad_x < -0.03,
        details=f"dpad_x={dpad_x:.3f}",
    )
    ctx.check(
        "face buttons are on right side of deck",
        f1_x > 0.03,
        details=f"face_btn_1_x={f1_x:.3f}",
    )

    # ---- Analog nub tilts ----
    nub_rest_aabb = ctx.part_world_aabb(nub)
    nub_rest_zmid = 0.5 * (nub_rest_aabb[0][2] + nub_rest_aabb[1][2])
    with ctx.pose({nub_joint: math.radians(15.0)}):
        nub_tilt_aabb = ctx.part_world_aabb(nub)
        nub_tilt_zmid = 0.5 * (nub_tilt_aabb[0][2] + nub_tilt_aabb[1][2])
    with ctx.pose({nub_joint: -math.radians(15.0)}):
        nub_neg_aabb = ctx.part_world_aabb(nub)
        nub_neg_zmid = 0.5 * (nub_neg_aabb[0][2] + nub_neg_aabb[1][2])
    ctx.check(
        "analog nub tilts about horizontal pivot",
        (nub_tilt_zmid > nub_rest_zmid + 0.0003)
        and (nub_neg_zmid < nub_rest_zmid - 0.0003),
        details=f"rest_zmid={nub_rest_zmid:.4f} pos={nub_tilt_zmid:.4f} neg={nub_neg_zmid:.4f}",
    )

    # ---- Shoulder buttons press (far end at -Z goes down, -Y) ----
    shl_rest = ctx.part_world_aabb(sh_l)
    with ctx.pose({shl_joint: math.radians(18.0)}):
        shl_press = ctx.part_world_aabb(sh_l)
    ctx.check(
        "left shoulder presses down (far end drops)",
        shl_press[0][1] < shl_rest[0][1] - 0.001,
        details=f"rest_minY={shl_rest[0][1]:.4f} press_minY={shl_press[0][1]:.4f}",
    )
    shr_rest = ctx.part_world_aabb(sh_r)
    with ctx.pose({shr_joint: math.radians(18.0)}):
        shr_press = ctx.part_world_aabb(sh_r)
    ctx.check(
        "right shoulder presses down (far end drops)",
        shr_press[0][1] < shr_rest[0][1] - 0.001,
        details=f"rest_minY={shr_rest[0][1]:.4f} press_minY={shr_press[0][1]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
