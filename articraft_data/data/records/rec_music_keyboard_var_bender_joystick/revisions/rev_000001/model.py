"""Compact MIDI keyboard controller (reference: picture/Music/keyboard/001.png).

A 25-mini-key MIDI controller in a black chassis with red side end caps:
- 25 pressable piano keys (15 white naturals with notched profiles + 10 black
  sharps), each hinged at the rear with a small downward press travel.
- 8 backlit drum pads (2 rows x 4 columns) on the top-left of the control deck,
  each with a red bezel and a small prismatic press travel.
- 4 small rotary knobs in a column on the left edge of the deck, each with a
  raised off-axis indicator dot so rotation is visually provable.
- A long recessed display/touch-strip area top-center-right.

Axes: Z up, X spans the width (left/right), the player-facing front is -Y.
"""

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---------------------------------------------------------------- chassis ---
BODY_W = 0.320  # full width including red end caps
BODY_D = 0.180  # front-to-back depth
CAP_W = 0.013  # red end cap thickness
CORE_W = BODY_W - 2.0 * CAP_W  # black core width
CORE_HALF = CORE_W / 2.0  # 0.147

SHELL_Z0, SHELL_Z1 = 0.0, 0.012  # bottom shell
DECK_FRONT_Y = 0.005  # front face of the raised rear control deck
DECK_BACK_Y = BODY_D / 2.0  # 0.090
DECK_TOP_Z = 0.040
BED_TOP_Z = 0.016  # low key-bed floor under the keys
CAP_TOP_Z = 0.042  # end caps sit slightly proud of the deck

# ------------------------------------------------------------------- keys ---
N_WHITE = 15
KEY_PITCH = CORE_W / N_WHITE  # 0.0196
WHITE_W = KEY_PITCH - 0.0010
WHITE_L = 0.093  # rear face at deck front, tip near the body front edge
WHITE_T = 0.011  # white key thickness
WHITE_BOT_Z = 0.022
WHITE_HINGE_Z = WHITE_BOT_Z + WHITE_T / 2.0  # 0.0275

BLACK_W = 0.0105
BLACK_L = 0.056
BLACK_H = 0.0215
BLACK_BOT_Z = 0.024
BLACK_HINGE_Z = BLACK_BOT_Z + BLACK_H / 2.0  # 0.03475
SLOT_W = 0.0125  # gap carved between white keys for each sharp
SLOT_L = 0.058  # notch depth from the key rear

# naturals (0-based) that have a sharp on their right boundary
SHARP_AFTER = (0, 1, 3, 4, 5, 7, 8, 10, 11, 12)

WHITE_PRESS = 0.055  # rad, ~5 mm tip travel
BLACK_PRESS = 0.060  # rad, ~3.4 mm tip travel

# ------------------------------------------------------------------- pads ---
PAD_SIZE = 0.026
PAD_H = 0.005
BEZEL_SIZE = 0.029
BEZEL_H = 0.0016
PAD_COLS_X = (-0.110, -0.0785, -0.047, -0.0155)
PAD_ROWS_Y = (0.0318, 0.0633)
PAD_TRAVEL = 0.0015

# ------------------------------------------------------------------ knobs ---
KNOB_D = 0.013
KNOB_H = 0.009
KNOB_X = -0.135
KNOB_YS = (0.019, 0.038, 0.057, 0.076)
KNOB_RANGE = 2.62  # +/- 150 degrees

# ----------------------------------------------------------- display strip --
STRIP_CX = 0.070
STRIP_HALF_L = 0.062
STRIP_HALF_W = 0.012
STRIP_CY = 0.0475
STRIP_RIM_W = 0.004
STRIP_RIM_H = 0.0025

# ----------------------------------------------------------- joystick ---
JOYSTICK_X = -0.120  # front-left of deck
JOYSTICK_Y = 0.015
JOYSTICK_SOCKET_D = 0.022
JOYSTICK_SOCKET_H = 0.006
JOYSTICK_RING_D = 0.020
JOYSTICK_RING_TUBE = 0.0025
JOYSTICK_STICK_D = 0.006
JOYSTICK_STICK_H = 0.025
JOYSTICK_CAP_D = 0.009
JOYSTICK_TILT = 0.35  # ~20 degrees max tilt


def _white_key_mesh(cut_left: bool, cut_right: bool) -> MeshGeometry:
    """Top-view key profile (rear at y=0, tip at y=-WHITE_L), thickness on Z."""
    xl, xr = -WHITE_W / 2.0, WHITE_W / 2.0
    xlc = -(KEY_PITCH / 2.0 - SLOT_W / 2.0)  # notched rear edge, left side
    xrc = KEY_PITCH / 2.0 - SLOT_W / 2.0  # notched rear edge, right side
    pts: list[tuple[float, float]] = [(xl, -WHITE_L), (xr, -WHITE_L)]
    if cut_right:
        pts += [(xr, -SLOT_L), (xrc, -SLOT_L), (xrc, 0.0)]
    else:
        pts += [(xr, 0.0)]
    if cut_left:
        pts += [(xlc, 0.0), (xlc, -SLOT_L), (xl, -SLOT_L)]
    else:
        pts += [(xl, 0.0)]
    return ExtrudeGeometry(pts, WHITE_T, center=True)


def _black_key_mesh() -> MeshGeometry:
    """Sharp key wedge: vertical rear, slanted front face, thickness on X."""
    setback = 0.010
    profile = [
        (0.0, -BLACK_H / 2.0),
        (0.0, BLACK_H / 2.0),
        (-(BLACK_L - setback), BLACK_H / 2.0),
        (-BLACK_L, -BLACK_H / 2.0),
    ]
    geom = ExtrudeGeometry.from_z0(profile, BLACK_W)
    # (a, b, c) -> (c, a, b): profile-x becomes key Y, profile-y becomes key Z,
    # extrusion becomes the key width on X.
    geom.rotate_y(math.pi / 2.0).rotate_x(math.pi / 2.0)
    geom.translate(-BLACK_W / 2.0, 0.0, 0.0)
    return geom


def _pad_mesh() -> MeshGeometry:
    profile = rounded_rect_profile(PAD_SIZE, PAD_SIZE, 0.004)
    return ExtrudeGeometry.from_z0(profile, PAD_H)


def _knob_geometry() -> KnobGeometry:
    return KnobGeometry(
        KNOB_D,
        KNOB_H,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=24, depth=0.0005),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,
    )


def _joystick_ring_mesh() -> MeshGeometry:
    """Torus gimbal ring."""
    return TorusGeometry(JOYSTICK_RING_D / 2.0, JOYSTICK_RING_TUBE)


def _joystick_stick_mesh() -> MeshGeometry:
    """Vertical stick shaft with spherical cap."""
    shaft = CylinderGeometry(JOYSTICK_STICK_D / 2.0, JOYSTICK_STICK_H)
    cap = SphereGeometry(JOYSTICK_CAP_D / 2.0)
    cap.translate(0.0, 0.0, JOYSTICK_STICK_H / 2.0)
    return shaft.merge(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_midi_keyboard_controller")

    model.material("chassis_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("accent_red", rgba=(0.78, 0.06, 0.08, 1.0))
    model.material("key_white", rgba=(0.93, 0.93, 0.91, 1.0))
    model.material("key_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("pad_rubber", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("strip_glass", rgba=(0.03, 0.03, 0.035, 1.0))
    model.material("knob_gray", rgba=(0.17, 0.17, 0.18, 1.0))

    chassis = model.part("chassis")

    # Bottom shell spanning the black core footprint.
    chassis.visual(
        Box((CORE_W, BODY_D, SHELL_Z1 - SHELL_Z0)),
        origin=Origin(xyz=(0.0, 0.0, (SHELL_Z0 + SHELL_Z1) / 2.0)),
        material="chassis_black",
        name="bottom_shell",
    )
    # Raised rear control deck.
    deck_d = DECK_BACK_Y - DECK_FRONT_Y
    chassis.visual(
        Box((CORE_W, deck_d, DECK_TOP_Z - SHELL_Z1)),
        origin=Origin(
            xyz=(0.0, (DECK_FRONT_Y + DECK_BACK_Y) / 2.0, (SHELL_Z1 + DECK_TOP_Z) / 2.0)
        ),
        material="chassis_black",
        name="control_deck",
    )
    # Low key-bed floor under the keys.
    bed_d = DECK_FRONT_Y - (-BODY_D / 2.0)
    chassis.visual(
        Box((CORE_W, bed_d, BED_TOP_Z - SHELL_Z1)),
        origin=Origin(
            xyz=(0.0, (DECK_FRONT_Y - BODY_D / 2.0) / 2.0, (SHELL_Z1 + BED_TOP_Z) / 2.0)
        ),
        material="chassis_black",
        name="key_bed",
    )
    # Red side end caps.
    for side, sx in (("left", -1.0), ("right", 1.0)):
        chassis.visual(
            Box((CAP_W, BODY_D, CAP_TOP_Z)),
            origin=Origin(xyz=(sx * (CORE_HALF + CAP_W / 2.0), 0.0, CAP_TOP_Z / 2.0)),
            material="accent_red",
            name=f"{side}_end_cap",
        )

    # Red backlit bezel under each drum pad.
    for r, py in enumerate(PAD_ROWS_Y):
        for c, px in enumerate(PAD_COLS_X):
            chassis.visual(
                Box((BEZEL_SIZE, BEZEL_SIZE, BEZEL_H)),
                origin=Origin(xyz=(px, py, DECK_TOP_Z + BEZEL_H / 2.0)),
                material="accent_red",
                name=f"pad_bezel_{r}_{c}",
            )

    # Recessed display / touch-strip area: dark floor inside a raised rim.
    chassis.visual(
        Box((2.0 * STRIP_HALF_L, 2.0 * STRIP_HALF_W, 0.0008)),
        origin=Origin(xyz=(STRIP_CX, STRIP_CY, DECK_TOP_Z + 0.0004)),
        material="strip_glass",
        name="display_strip",
    )
    rim_len = 2.0 * (STRIP_HALF_L + STRIP_RIM_W)
    for tag, oy in (("front", -1.0), ("rear", 1.0)):
        chassis.visual(
            Box((rim_len, STRIP_RIM_W, STRIP_RIM_H)),
            origin=Origin(
                xyz=(
                    STRIP_CX,
                    STRIP_CY + oy * (STRIP_HALF_W + STRIP_RIM_W / 2.0),
                    DECK_TOP_Z + STRIP_RIM_H / 2.0,
                )
            ),
            material="chassis_black",
            name=f"strip_rim_{tag}",
        )
    for tag, ox in (("inner", -1.0), ("outer", 1.0)):
        chassis.visual(
            Box((STRIP_RIM_W, 2.0 * STRIP_HALF_W, STRIP_RIM_H)),
            origin=Origin(
                xyz=(
                    STRIP_CX + ox * (STRIP_HALF_L + STRIP_RIM_W / 2.0),
                    STRIP_CY,
                    DECK_TOP_Z + STRIP_RIM_H / 2.0,
                )
            ),
            material="chassis_black",
            name=f"strip_rim_{tag}",
        )

    # ---------------------------------------------------------- white keys --
    white_meshes: dict[tuple[bool, bool], object] = {}
    for i in range(N_WHITE):
        cut_left = (i - 1) in SHARP_AFTER
        cut_right = i in SHARP_AFTER
        key = (cut_left, cut_right)
        if key not in white_meshes:
            tag = f"white_key_{'l' if cut_left else 'x'}{'r' if cut_right else 'x'}"
            white_meshes[key] = mesh_from_geometry(_white_key_mesh(cut_left, cut_right), tag)

        cx = -CORE_HALF + KEY_PITCH * (i + 0.5)
        part = model.part(f"white_key_{i}")
        part.visual(white_meshes[key], material="key_white", name="key_body")
        model.articulation(
            f"white_key_{i}_press",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=part,
            origin=Origin(xyz=(cx, DECK_FRONT_Y, WHITE_HINGE_Z)),
            # Key extends along -Y from the rear hinge; positive q about +X
            # drops the front tip downward (press).
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=WHITE_PRESS),
        )

    # ---------------------------------------------------------- black keys --
    black_mesh = mesh_from_geometry(_black_key_mesh(), "black_key")
    for j, n in enumerate(SHARP_AFTER):
        cx = -CORE_HALF + KEY_PITCH * (n + 1)  # centered on the natural boundary
        part = model.part(f"black_key_{j}")
        part.visual(black_mesh, material="key_black", name="key_body")
        model.articulation(
            f"black_key_{j}_press",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=part,
            origin=Origin(xyz=(cx, DECK_FRONT_Y, BLACK_HINGE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=BLACK_PRESS),
        )

    # ------------------------------------------------------------ drum pads --
    pad_mesh = mesh_from_geometry(_pad_mesh(), "drum_pad")
    pad_index = 0
    for r, py in enumerate(PAD_ROWS_Y):
        for c, px in enumerate(PAD_COLS_X):
            part = model.part(f"drum_pad_{pad_index}")
            part.visual(pad_mesh, material="pad_rubber", name="pad_cap")
            model.articulation(
                f"drum_pad_{pad_index}_press",
                ArticulationType.PRISMATIC,
                parent=chassis,
                child=part,
                origin=Origin(xyz=(px, py, DECK_TOP_Z + BEZEL_H)),
                # Positive q presses the pad straight down into the bezel.
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=PAD_TRAVEL),
            )
            pad_index += 1

    # -------------------------------------------------------------- knobs ---
    knob_mesh = mesh_from_geometry(_knob_geometry(), "rotary_knob")
    for k, ky in enumerate(KNOB_YS):
        part = model.part(f"knob_{k}")
        part.visual(knob_mesh, material="knob_gray", name="knob_cap")
        model.articulation(
            f"knob_{k}_turn",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=part,
            origin=Origin(xyz=(KNOB_X, ky, DECK_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.5, velocity=6.0, lower=-KNOB_RANGE, upper=KNOB_RANGE
            ),
        )

    # ---------------------------------------------------------- joystick ---
    # Socket base on the deck
    socket_mesh = mesh_from_geometry(
        CylinderGeometry(JOYSTICK_SOCKET_D / 2.0, JOYSTICK_SOCKET_H),
        "joystick_socket"
    )
    chassis.visual(
        socket_mesh,
        origin=Origin(xyz=(JOYSTICK_X, JOYSTICK_Y, DECK_TOP_Z + JOYSTICK_SOCKET_H / 2.0)),
        material="chassis_black",
        name="joystick_socket",
    )
    
    # Gimbal ring (tilts left/right for pitch)
    gimbal = model.part("joystick_gimbal")
    ring_mesh = mesh_from_geometry(_joystick_ring_mesh(), "joystick_ring")
    gimbal.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="knob_gray",
        name="ring",
    )
    model.articulation(
        "joystick_pitch",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=gimbal,
        origin=Origin(xyz=(JOYSTICK_X, JOYSTICK_Y, DECK_TOP_Z + JOYSTICK_SOCKET_H)),
        axis=(0.0, 1.0, 0.0),  # Y-axis for left/right tilt
        motion_limits=MotionLimits(
            effort=1.0, velocity=5.0, lower=-JOYSTICK_TILT, upper=JOYSTICK_TILT
        ),
    )
    
    # Stick (tilts forward/back for modulation)
    stick = model.part("joystick_stick")
    stick_mesh = mesh_from_geometry(_joystick_stick_mesh(), "joystick_stick")
    stick.visual(
        stick_mesh,
        origin=Origin(xyz=(0.0, 0.0, JOYSTICK_STICK_H / 2.0)),
        material="chassis_black",
        name="shaft",
    )
    model.articulation(
        "joystick_mod",
        ArticulationType.REVOLUTE,
        parent=gimbal,
        child=stick,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),  # X-axis for forward/back tilt
        motion_limits=MotionLimits(
            effort=1.0, velocity=5.0, lower=-JOYSTICK_TILT, upper=JOYSTICK_TILT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    whites = [p for p in object_model.parts if p.name.startswith("white_key_")]
    blacks = [p for p in object_model.parts if p.name.startswith("black_key_")]
    pads = [p for p in object_model.parts if p.name.startswith("drum_pad_")]
    knobs = [p for p in object_model.parts if p.name.startswith("knob_")]

    ctx.check("15 white naturals", len(whites) == 15, details=f"got {len(whites)}")
    ctx.check("10 black sharps", len(blacks) == 10, details=f"got {len(blacks)}")
    ctx.check(
        "25 pressable piano keys",
        len(whites) + len(blacks) == 25,
        details=f"got {len(whites) + len(blacks)}",
    )
    ctx.check("8 drum pads", len(pads) == 8, details=f"got {len(pads)}")
    ctx.check("4 rotary knobs", len(knobs) == 4, details=f"got {len(knobs)}")

    # Every key articulation is a limited revolute press starting at rest.
    bad_keys = []
    for i in range(15):
        j = object_model.get_articulation(f"white_key_{i}_press")
        lim = j.motion_limits
        if lim is None or lim.lower != 0.0 or lim.upper is None or lim.upper <= 0.0:
            bad_keys.append(j.name)
    for i in range(10):
        j = object_model.get_articulation(f"black_key_{i}_press")
        lim = j.motion_limits
        if lim is None or lim.lower != 0.0 or lim.upper is None or lim.upper <= 0.0:
            bad_keys.append(j.name)
    ctx.check(
        "key press limits rest at 0 with downward travel", not bad_keys, details=str(bad_keys)
    )

    chassis = object_model.get_part("chassis")
    white_0 = object_model.get_part("white_key_0")
    black_0 = object_model.get_part("black_key_0")
    pad_0 = object_model.get_part("drum_pad_0")

    # Keys clear the low key-bed floor at rest.
    ctx.expect_gap(
        white_0,
        chassis,
        axis="z",
        negative_elem="key_bed",
        min_gap=0.003,
        max_gap=0.010,
        name="white key floats above the key bed",
    )
    # Key rears seat against the control deck front face.
    ctx.expect_contact(
        white_0,
        chassis,
        elem_b="control_deck",
        contact_tol=1e-4,
        name="white key rear seats at the deck",
    )
    ctx.expect_contact(
        black_0,
        chassis,
        elem_b="control_deck",
        contact_tol=1e-4,
        name="black key rear seats at the deck",
    )
    # Pads rest on their red backlit bezels and stay within the deck footprint.
    ctx.expect_contact(
        pad_0,
        chassis,
        elem_b="pad_bezel_0_0",
        contact_tol=1e-4,
        name="drum pad rests on its red bezel",
    )
    ctx.expect_within(
        pad_0,
        chassis,
        axes="xy",
        outer_elem="control_deck",
        name="drum pad stays on the control deck",
    )
    # Knob sits on the deck top.
    knob_0 = object_model.get_part("knob_0")
    ctx.expect_contact(
        knob_0,
        chassis,
        elem_b="control_deck",
        contact_tol=1e-4,
        name="knob base seats on the deck",
    )

    # Decisive pose checks: pressing moves the touch surface downward.
    w_joint = object_model.get_articulation("white_key_0_press")
    rest = ctx.part_world_aabb(white_0)
    with ctx.pose({w_joint: WHITE_PRESS}):
        pressed = ctx.part_world_aabb(white_0)
    ctx.check(
        "pressed white key tip dips down",
        rest is not None and pressed is not None and pressed[0][2] < rest[0][2] - 0.003,
        details=f"rest_min_z={rest[0][2] if rest else None}, "
        f"pressed_min_z={pressed[0][2] if pressed else None}",
    )

    b_joint = object_model.get_articulation("black_key_0_press")
    rest_b = ctx.part_world_aabb(black_0)
    with ctx.pose({b_joint: BLACK_PRESS}):
        pressed_b = ctx.part_world_aabb(black_0)
    ctx.check(
        "pressed black key tip dips down",
        rest_b is not None and pressed_b is not None and pressed_b[0][2] < rest_b[0][2] - 0.002,
        details=f"rest_min_z={rest_b[0][2] if rest_b else None}, "
        f"pressed_min_z={pressed_b[0][2] if pressed_b else None}",
    )

    p_joint = object_model.get_articulation("drum_pad_0_press")
    rest_p = ctx.part_world_aabb(pad_0)
    with ctx.pose({p_joint: PAD_TRAVEL}):
        pressed_p = ctx.part_world_aabb(pad_0)
    ctx.check(
        "pressed drum pad sinks down",
        rest_p is not None
        and pressed_p is not None
        and pressed_p[1][2] < rest_p[1][2] - 0.5 * PAD_TRAVEL,
        details=f"rest_top_z={rest_p[1][2] if rest_p else None}, "
        f"pressed_top_z={pressed_p[1][2] if pressed_p else None}",
    )

    # Knob is a limited vertical-axis rotary control.
    k_joint = object_model.get_articulation("knob_0_turn")
    ctx.check(
        "knob turns about the vertical axis",
        tuple(k_joint.axis) == (0.0, 0.0, 1.0)
        and k_joint.motion_limits is not None
        and k_joint.motion_limits.lower < 0.0 < k_joint.motion_limits.upper,
        details=f"axis={k_joint.axis}, limits={k_joint.motion_limits}",
    )

    # ---------------------------------------------------------- joystick ---
    gimbal = object_model.get_part("joystick_gimbal")
    stick = object_model.get_part("joystick_stick")
    ctx.check("joystick gimbal exists", gimbal is not None, details="missing part")
    ctx.check("joystick stick exists", stick is not None, details="missing part")
    
    pitch_joint = object_model.get_articulation("joystick_pitch")
    mod_joint = object_model.get_articulation("joystick_mod")
    ctx.check("joystick pitch joint exists", pitch_joint is not None, details="missing joint")
    ctx.check("joystick mod joint exists", mod_joint is not None, details="missing joint")
    
    # Pitch joint tilts left/right about Y-axis
    ctx.check(
        "joystick pitch tilts about Y-axis",
        tuple(pitch_joint.axis) == (0.0, 1.0, 0.0)
        and pitch_joint.motion_limits is not None
        and pitch_joint.motion_limits.lower < 0.0 < pitch_joint.motion_limits.upper,
        details=f"axis={pitch_joint.axis}, limits={pitch_joint.motion_limits}",
    )
    
    # Mod joint tilts forward/back about X-axis
    ctx.check(
        "joystick mod tilts about X-axis",
        tuple(mod_joint.axis) == (1.0, 0.0, 0.0)
        and mod_joint.motion_limits is not None
        and mod_joint.motion_limits.lower < 0.0 < mod_joint.motion_limits.upper,
        details=f"axis={mod_joint.axis}, limits={mod_joint.motion_limits}",
    )
    
    # Stick rests centered within the gimbal ring footprint
    ctx.expect_within(
        stick,
        gimbal,
        axes="xy",
        margin=0.002,
        name="joystick stick centers within gimbal",
    )
    
    # Pose test: tilting pitch moves the stick sideways
    rest_aabb = ctx.part_world_aabb(stick)
    with ctx.pose({pitch_joint: JOYSTICK_TILT}):
        tilted_aabb = ctx.part_world_aabb(stick)
    ctx.check(
        "joystick pitch tilts stick left/right",
        rest_aabb is not None and tilted_aabb is not None 
        and abs(tilted_aabb[1][0] - rest_aabb[1][0]) > 0.002,
        details=f"rest_max_x={rest_aabb[1][0] if rest_aabb else None}, "
                f"tilted_max_x={tilted_aabb[1][0] if tilted_aabb else None}",
    )
    
    # Pose test: tilting mod moves the stick forward/back
    with ctx.pose({mod_joint: JOYSTICK_TILT}):
        mod_tilted_aabb = ctx.part_world_aabb(stick)
    ctx.check(
        "joystick mod tilts stick forward/back",
        rest_aabb is not None and mod_tilted_aabb is not None 
        and abs(mod_tilted_aabb[0][1] - rest_aabb[0][1]) > 0.005,
        details=f"rest_min_y={rest_aabb[0][1] if rest_aabb else None}, "
                f"tilted_min_y={mod_tilted_aabb[0][1] if mod_tilted_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
