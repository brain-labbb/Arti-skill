"""Compact MIDI keyboard controller with fader bank.

A 25-mini-key MIDI controller in a black chassis with red side end caps:
- 25 pressable piano keys (15 white naturals with notched profiles + 10 black
  sharps), each hinged at the rear with a small downward press travel.
- 9 linear faders in a horizontal bank on the control deck, each a prismatic
  cap riding a raised slot rail along the front-to-back travel axis.
- A long recessed display/touch-strip area at the rear of the deck.

Axes: Z up, X spans the width (left/right), the player-facing front is -Y.
"""

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
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

# ----------------------------------------------------------------- faders ---
N_FADERS = 9
FADER_SPACING = 0.028  # center-to-center spacing
FADER_X0 = -FADER_SPACING * (N_FADERS - 1) / 2.0  # centered on deck

FADER_SLOT_L = 0.046       # slot track visible length along Y
FADER_SLOT_W = 0.004       # slot groove width on X
FADER_RAIL_W = 0.003       # each raised rail strip width
FADER_RAIL_H = 0.003       # rail height above deck top
FADER_SLOT_FRONT_Y = 0.008 # front of the visible slot track
FADER_SLOT_CY = FADER_SLOT_FRONT_Y + FADER_SLOT_L / 2.0  # 0.031

FADER_CAP_W = 0.020        # cap width (X), wider than the slot
FADER_CAP_D = 0.010        # cap depth (Y, along travel)
FADER_CAP_H = 0.009        # cap height (Z)
FADER_GRIP_W = 0.014       # grip ridge width
FADER_GRIP_D = 0.003       # grip ridge depth
FADER_GRIP_H = 0.003       # grip ridge height above cap top
FADER_TRAVEL = 0.030       # prismatic travel along +Y

# ----------------------------------------------------------- display strip --
STRIP_CX = 0.070
STRIP_HALF_L = 0.062
STRIP_HALF_W = 0.012
STRIP_CY = 0.072            # rear of deck for fader bank clearance
STRIP_RIM_W = 0.004
STRIP_RIM_H = 0.0025


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


def _fader_cap_mesh() -> MeshGeometry:
    """Fader slider cap: rounded-rectangle body extruded along Z."""
    profile = rounded_rect_profile(FADER_CAP_W, FADER_CAP_D, 0.002)
    return ExtrudeGeometry.from_z0(profile, FADER_CAP_H)


def _fader_grip_mesh() -> MeshGeometry:
    """Small grip ridge on top of the fader cap."""
    profile = rounded_rect_profile(FADER_GRIP_W, FADER_GRIP_D, 0.001)
    return ExtrudeGeometry.from_z0(profile, FADER_GRIP_H)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_midi_keyboard_controller")

    model.material("chassis_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("accent_red", rgba=(0.78, 0.06, 0.08, 1.0))
    model.material("key_white", rgba=(0.93, 0.93, 0.91, 1.0))
    model.material("key_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("strip_glass", rgba=(0.03, 0.03, 0.035, 1.0))
    model.material("fader_cap", rgba=(0.20, 0.20, 0.21, 1.0))
    model.material("fader_grip", rgba=(0.88, 0.88, 0.86, 1.0))
    model.material("slot_dark", rgba=(0.04, 0.04, 0.05, 1.0))
    model.material("rail_silver", rgba=(0.55, 0.55, 0.56, 1.0))

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

    # Fader slot rails: dark groove + two raised silver rail strips per fader.
    for i in range(N_FADERS):
        fx = FADER_X0 + i * FADER_SPACING
        # Dark slot groove on the deck surface.
        chassis.visual(
            Box((FADER_SLOT_W, FADER_SLOT_L, 0.001)),
            origin=Origin(xyz=(fx, FADER_SLOT_CY, DECK_TOP_Z + 0.0005)),
            material="slot_dark",
            name=f"fader_slot_{i}",
        )
        # Raised rail strips on each side of the slot.
        for side, sx in (("left", -1.0), ("right", 1.0)):
            chassis.visual(
                Box((FADER_RAIL_W, FADER_SLOT_L, FADER_RAIL_H)),
                origin=Origin(
                    xyz=(
                        fx + sx * (FADER_SLOT_W / 2.0 + FADER_RAIL_W / 2.0),
                        FADER_SLOT_CY,
                        DECK_TOP_Z + FADER_RAIL_H / 2.0,
                    )
                ),
                material="rail_silver",
                name=f"fader_rail_{i}_{side}",
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

    # ----------------------------------------------------------- faders ---
    cap_mesh = mesh_from_geometry(_fader_cap_mesh(), "fader_cap")
    grip_mesh = mesh_from_geometry(_fader_grip_mesh(), "fader_grip")
    for i in range(N_FADERS):
        fx = FADER_X0 + i * FADER_SPACING
        part = model.part(f"fader_{i}")
        # Cap body: bottom at z=0 in part frame (sits on the rail top).
        # Front edge at y=0 in part frame (at the joint origin Y).
        part.visual(
            cap_mesh,
            origin=Origin(xyz=(0.0, FADER_CAP_D / 2.0, 0.0)),
            material="fader_cap",
            name="cap_body",
        )
        # Grip ridge on top of the cap.
        part.visual(
            grip_mesh,
            origin=Origin(xyz=(0.0, FADER_CAP_D / 2.0, FADER_CAP_H)),
            material="fader_grip",
            name="cap_grip",
        )
        model.articulation(
            f"fader_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=part,
            # Joint origin at the front end of the slot, on the rail top surface.
            origin=Origin(xyz=(fx, FADER_SLOT_FRONT_Y, DECK_TOP_Z + FADER_RAIL_H)),
            # Positive q slides the cap toward the rear (+Y).
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=0.10, lower=0.0, upper=FADER_TRAVEL
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    whites = [p for p in object_model.parts if p.name.startswith("white_key_")]
    blacks = [p for p in object_model.parts if p.name.startswith("black_key_")]
    faders = [p for p in object_model.parts if p.name.startswith("fader_")]

    ctx.check("15 white naturals", len(whites) == 15, details=f"got {len(whites)}")
    ctx.check("10 black sharps", len(blacks) == 10, details=f"got {len(blacks)}")
    ctx.check(
        "25 pressable piano keys",
        len(whites) + len(blacks) == 25,
        details=f"got {len(whites) + len(blacks)}",
    )
    ctx.check("9 linear faders", len(faders) == 9, details=f"got {len(faders)}")

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

    # ---- Fader checks ----
    fader_0 = object_model.get_part("fader_0")
    fader_8 = object_model.get_part("fader_8")
    f0_joint = object_model.get_articulation("fader_0_slide")
    f8_joint = object_model.get_articulation("fader_8_slide")

    # Every fader is a prismatic slide with 0-to-positive travel.
    bad_faders = []
    for i in range(N_FADERS):
        j = object_model.get_articulation(f"fader_{i}_slide")
        if j.articulation_type != ArticulationType.PRISMATIC:
            bad_faders.append(f"{j.name}: type={j.articulation_type}")
            continue
        lim = j.motion_limits
        if lim is None or lim.lower != 0.0 or lim.upper is None or lim.upper <= 0.0:
            bad_faders.append(f"{j.name}: limits={lim}")
    ctx.check(
        "all faders are prismatic with 0-to-positive travel",
        not bad_faders,
        details=str(bad_faders),
    )

    # Fader axis is along Y (front-to-back travel).
    ctx.check(
        "fader slide axis is along Y",
        tuple(f0_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={f0_joint.axis}",
    )

    # Fader cap sits above the deck (on the rail) at rest.
    ctx.expect_gap(
        fader_0,
        chassis,
        axis="z",
        positive_elem="cap_body",
        negative_elem="control_deck",
        min_gap=0.001,
        max_gap=0.006,
        name="fader cap sits above the deck on the rail",
    )

    # Fader stays within the deck footprint on XY.
    ctx.expect_within(
        fader_0,
        chassis,
        axes="xy",
        outer_elem="control_deck",
        name="fader 0 stays on the control deck",
    )
    ctx.expect_within(
        fader_8,
        chassis,
        axes="xy",
        outer_elem="control_deck",
        name="fader 8 stays on the control deck",
    )

    # Decisive pose: sliding moves the cap toward the rear (+Y).
    rest_pos = ctx.part_world_position(fader_0)
    with ctx.pose({f0_joint: FADER_TRAVEL}):
        slid_pos = ctx.part_world_position(fader_0)
    ctx.check(
        "fader slides toward rear at max travel",
        rest_pos is not None
        and slid_pos is not None
        and slid_pos[1] > rest_pos[1] + 0.5 * FADER_TRAVEL,
        details=f"rest_y={rest_pos[1] if rest_pos else None}, "
        f"slid_y={slid_pos[1] if slid_pos else None}",
    )

    # Pressed white key tip dips down.
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

    # Pressed black key tip dips down.
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

    return ctx.report()


object_model = build_object_model()
