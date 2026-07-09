"""Silver handheld wireless point-of-sale payment terminal.

Reference: countertop/handheld POS terminal with an angled color display, a
menu key row under the screen, a 12-key numeric keypad plus red/yellow/green
function keys, a rear receipt printer with a flip-open paper cover and paper
roll, a front chip card slot, and a magnetic stripe swipe groove along the
right side.

Frame convention: +X is the front (keypad / chip slot end, toward the user),
+Z is up, +Y is the user's right when facing the keypad. All dimensions are
meters; the real device is about 0.185 m long and 0.082 m wide.
"""

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
BODY_LEN_HALF = 0.0925  # +X front face, -X rear face
BODY_WIDTH_HALF = 0.041
FRONT_DECK_Z = 0.024  # flat keypad deck height
REAR_DECK_Z = 0.050  # flat rear deck around the printer opening
SLOPE_FRONT_X = 0.000  # display slope starts here (deck level)
SLOPE_REAR_X = -0.036  # display slope tops out here (rear deck level)

# printer compartment
CAVITY_X_MIN, CAVITY_X_MAX = -0.086, -0.044
CAVITY_Y_HALF = 0.033
CAVITY_FLOOR_Z = 0.014

# printer cover (authored in hinge-local frame, hinge at rear edge)
HINGE_X = -0.0905
HINGE_Z = REAR_DECK_Z
COVER_LEN = 0.0525  # extends +X (forward) from hinge line
COVER_WIDTH = 0.080
COVER_THICK = 0.0129

# paper roll
ROLL_CENTER = (-0.065, 0.0, 0.030)
ROLL_RADIUS = 0.015
ROLL_LEN = 0.060
AXLE_RADIUS = 0.003
AXLE_LEN = 0.070  # ends embed into the compartment side walls (bearing slots)

# keypad
KEY_SIZE = (0.013, 0.017, 0.004)
KEY_ROWS_X = (0.021, 0.0375, 0.054, 0.0705)  # rows 1-2-3 ... star-0-hash
FN_ROW_X = 0.086  # red / yellow / green row at the front edge
KEY_COLS_Y = (-0.0215, 0.0, 0.0215)
KEY_TRAVEL = 0.0015
KEY_CLEAR = 0.0  # key caps rest exactly on the deck surface

# menu key strip under the display
STRIP_CENTER = (0.0065, 0.0, 0.0246)
STRIP_SIZE = (0.013, 0.064, 0.0018)
MENU_KEY_SIZE = (0.009, 0.011, 0.0035)
MENU_KEYS_Y = (-0.024, -0.008, 0.008, 0.024)

# display slope frame (unit normal of the sloped display face)
_slope_dx = SLOPE_REAR_X - SLOPE_FRONT_X  # -0.036
_slope_dz = REAR_DECK_Z - FRONT_DECK_Z  # +0.026
_slope_len = math.hypot(_slope_dx, _slope_dz)
SLOPE_NORMAL = (_slope_dz / _slope_len, 0.0, -_slope_dx / _slope_len)
SLOPE_PITCH = math.asin(SLOPE_NORMAL[0])  # rpy pitch that maps +Z onto normal
SLOPE_MID = (
    (SLOPE_FRONT_X + SLOPE_REAR_X) / 2.0,
    0.0,
    (FRONT_DECK_Z + REAR_DECK_Z) / 2.0,
)

# display tilt hinge (top-rear edge of the sloped face)
DISPLAY_HINGE = (SLOPE_REAR_X, 0.0, REAR_DECK_Z)
HINGE_BOSS_RADIUS = 0.004
HINGE_BOSS_LEN = 0.064

# display_head local positions (relative to hinge origin at top-rear slope edge)
_BEZEL_LOCAL = (
    SLOPE_MID[0] + SLOPE_NORMAL[0] * 0.00145 - DISPLAY_HINGE[0],
    0.0,
    SLOPE_MID[2] + SLOPE_NORMAL[2] * 0.00145 - DISPLAY_HINGE[2],
)
_SCREEN_LOCAL = (
    SLOPE_MID[0] + SLOPE_NORMAL[0] * 0.0039 - DISPLAY_HINGE[0],
    0.0,
    SLOPE_MID[2] + SLOPE_NORMAL[2] * 0.0039 - DISPLAY_HINGE[2],
)


def _on_slope(normal_offset: float) -> tuple[float, float, float]:
    """Point offset from the display-slope midpoint along its outward normal."""
    return (
        SLOPE_MID[0] + SLOPE_NORMAL[0] * normal_offset,
        0.0,
        SLOPE_MID[2] + SLOPE_NORMAL[2] * normal_offset,
    )


# ---------------------------------------------------------------- geometry


def _housing_shape() -> cq.Workplane:
    """Main chassis: side profile extruded across the width, then detailed."""
    profile = [
        (BODY_LEN_HALF, 0.0),
        (BODY_LEN_HALF, FRONT_DECK_Z),  # front face
        (SLOPE_FRONT_X, FRONT_DECK_Z),  # flat keypad deck
        (SLOPE_REAR_X, REAR_DECK_Z),  # display slope
        (-BODY_LEN_HALF, REAR_DECK_Z),  # flat rear deck (printer area)
        (-BODY_LEN_HALF, 0.0),  # rear face
    ]
    body = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(BODY_WIDTH_HALF, both=True)
    )
    body = body.edges("|Z").fillet(0.008)
    try:
        body = body.edges("|Y").fillet(0.0018)
    except Exception:
        pass  # cosmetic edge softening only

    # receipt-paper compartment (open at the top, closed by the cover)
    cavity = (
        cq.Workplane("XY")
        .box(CAVITY_X_MAX - CAVITY_X_MIN, 2 * CAVITY_Y_HALF, 0.050)
        .translate(((CAVITY_X_MIN + CAVITY_X_MAX) / 2.0, 0.0, CAVITY_FLOOR_Z + 0.025))
    )
    body = body.cut(cavity)

    # chip card slot recessed into the front face
    chip_slot = (
        cq.Workplane("XY")
        .box(0.024, 0.056, 0.004)
        .translate((BODY_LEN_HALF - 0.005, 0.0, 0.010))
    )
    body = body.cut(chip_slot)

    # magnetic stripe swipe groove along the right side
    swipe = (
        cq.Workplane("XY")
        .box(0.170, 0.008, 0.003)
        .translate((0.0, BODY_WIDTH_HALF, 0.0155))
    )
    body = body.cut(swipe)
    return body


def _cover_shape() -> cq.Workplane:
    """Printer cover in hinge-local frame: extends +X, sits on z ~= 0."""
    cover = (
        cq.Workplane("XY")
        .box(COVER_LEN, COVER_WIDTH, COVER_THICK)
        .translate((COVER_LEN / 2.0, 0.0, COVER_THICK / 2.0))
    )
    cover = cover.edges("|Z").fillet(0.006)
    try:
        cover = cover.edges(">Z").fillet(0.004)
    except Exception:
        pass
    return cover


def _keycap_shape(size: tuple[float, float, float]) -> cq.Workplane:
    """Rounded keycap resting on local z=0."""
    sx, sy, sz = size
    cap = cq.Workplane("XY").box(sx, sy, sz)
    cap = cap.edges("|Z").fillet(min(sx, sy) * 0.18)
    try:
        cap = cap.edges(">Z").fillet(sz * 0.22)
    except Exception:
        pass
    return cap.translate((0.0, 0.0, sz / 2.0))


# ---------------------------------------------------------------- model


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pos_payment_terminal")

    silver = Material(name="silver_plastic", rgba=(0.76, 0.77, 0.80, 1.0))
    silver_key = Material(name="silver_key", rgba=(0.83, 0.84, 0.86, 1.0))
    dark = Material(name="dark_plastic", rgba=(0.07, 0.07, 0.08, 1.0))
    screen_blue = Material(name="screen_blue", rgba=(0.13, 0.34, 0.72, 1.0))
    red = Material(name="cancel_red", rgba=(0.76, 0.11, 0.10, 1.0))
    amber = Material(name="clear_yellow", rgba=(0.95, 0.68, 0.10, 1.0))
    green = Material(name="enter_green", rgba=(0.10, 0.62, 0.20, 1.0))
    paper = Material(name="paper_white", rgba=(0.94, 0.94, 0.92, 1.0))
    rubber = Material(name="rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))

    # ----- body (root)
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_housing_shape(), "housing"),
        material=silver,
        name="housing",
    )

    # hinge bracket on the housing at the top-rear of the slope
    body.visual(
        Box((0.008, 0.070, 0.010)),
        origin=Origin(xyz=(SLOPE_REAR_X + 0.004, 0.0, REAR_DECK_Z + 0.004)),
        material=dark,
        name="hinge_bracket",
    )

    # ----- display head (tilting screen, hinged at top-rear of slope)
    display_head = model.part("display_head")
    # pivot boss/barrel at the hinge line
    display_head.visual(
        Cylinder(radius=HINGE_BOSS_RADIUS, length=HINGE_BOSS_LEN),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_boss",
    )
    # bezel plate carrying the lit screen
    display_head.visual(
        Box((0.040, 0.060, 0.0045)),
        origin=Origin(xyz=_BEZEL_LOCAL, rpy=(0.0, SLOPE_PITCH, 0.0)),
        material=dark,
        name="display_bezel",
    )
    display_head.visual(
        Box((0.030, 0.048, 0.0008)),
        origin=Origin(xyz=_SCREEN_LOCAL, rpy=(0.0, SLOPE_PITCH, 0.0)),
        material=screen_blue,
        name="display_screen",
    )
    model.articulation(
        "housing_to_display_head",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display_head,
        origin=Origin(xyz=DISPLAY_HINGE),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=0.6,
        ),
    )

    # dark menu strip below the display, carrying the four menu keys
    body.visual(
        Box(STRIP_SIZE),
        origin=Origin(xyz=STRIP_CENTER),
        material=dark,
        name="menu_strip",
    )

    # dark liner at the back of the chip card slot
    body.visual(
        Box((0.003, 0.054, 0.0038)),
        # slot back wall is at x = 0.0755; embed half the liner into it
        origin=Origin(xyz=(0.0755, 0.0, 0.010)),
        material=dark,
        name="chip_slot_liner",
    )

    # dark liner along the swipe groove floor
    body.visual(
        Box((0.168, 0.002, 0.0028)),
        origin=Origin(xyz=(0.0, 0.0365, 0.0155)),
        material=dark,
        name="swipe_slot_liner",
    )

    # four rubber feet
    for i, (fx, fy) in enumerate(
        [(0.068, -0.026), (0.068, 0.026), (-0.068, -0.026), (-0.068, 0.026)]
    ):
        body.visual(
            Cylinder(radius=0.006, length=0.0032),
            origin=Origin(xyz=(fx, fy, -0.0012)),
            material=rubber,
            name=f"foot_{i}",
        )

    # ----- printer cover (hinged at the rear edge)
    cover = model.part("printer_cover")
    cover.visual(
        mesh_from_cadquery(_cover_shape(), "printer_cover"),
        material=silver,
        name="cover_shell",
    )
    # serrated tear bar along the top front edge of the cover
    cover.visual(
        Box((0.004, 0.070, 0.002)),
        origin=Origin(xyz=(0.0525, 0.0, 0.0125)),
        material=dark,
        name="tear_bar",
    )
    model.articulation(
        "printer_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # cover extends +X from the hinge; -Y lifts its free edge upward
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.7),
    )

    # ----- paper roll on a spindle inside the compartment
    roll = model.part("paper_roll")
    roll.visual(
        Cylinder(radius=ROLL_RADIUS, length=ROLL_LEN),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=paper,
        name="roll_paper",
    )
    roll.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_LEN),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="roll_axle",
    )
    model.articulation(
        "paper_roll_spindle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=roll,
        origin=Origin(xyz=ROLL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=10.0),
    )

    # ----- keypad: 12 numeric keys + 3 colored function keys
    key_mesh = mesh_from_cadquery(_keycap_shape(KEY_SIZE), "keycap")
    key_limits = MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=KEY_TRAVEL)

    grid_names = [
        ("key_1", "key_2", "key_3"),
        ("key_4", "key_5", "key_6"),
        ("key_7", "key_8", "key_9"),
        ("key_star", "key_0", "key_hash"),
    ]
    for row_x, row_names in zip(KEY_ROWS_X, grid_names):
        for col_y, key_name in zip(KEY_COLS_Y, row_names):
            key = model.part(key_name)
            key.visual(key_mesh, material=silver_key, name=f"{key_name}_cap")
            model.articulation(
                f"{key_name}_press",
                ArticulationType.PRISMATIC,
                parent=body,
                child=key,
                origin=Origin(xyz=(row_x, col_y, FRONT_DECK_Z + KEY_CLEAR)),
                axis=(0.0, 0.0, -1.0),  # positive q presses the key down
                motion_limits=key_limits,
            )

    for col_y, key_name, mat in zip(
        KEY_COLS_Y,
        ("cancel_key", "clear_key", "enter_key"),
        (red, amber, green),
    ):
        key = model.part(key_name)
        key.visual(key_mesh, material=mat, name=f"{key_name}_cap")
        model.articulation(
            f"{key_name}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(FN_ROW_X, col_y, FRONT_DECK_Z + KEY_CLEAR)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=key_limits,
        )

    # ----- menu key row under the display
    menu_mesh = mesh_from_cadquery(_keycap_shape(MENU_KEY_SIZE), "menu_keycap")
    strip_top_z = STRIP_CENTER[2] + STRIP_SIZE[2] / 2.0
    menu_limits = MotionLimits(effort=1.5, velocity=0.05, lower=0.0, upper=0.001)
    for i, col_y in enumerate(MENU_KEYS_Y):
        key = model.part(f"menu_key_{i}")
        key.visual(menu_mesh, material=dark, name=f"menu_key_{i}_cap")
        model.articulation(
            f"menu_key_{i}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(STRIP_CENTER[0], col_y, strip_top_z + KEY_CLEAR)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=menu_limits,
        )

    return model


# ---------------------------------------------------------------- tests


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("printer_cover")
    roll = object_model.get_part("paper_roll")
    display_head = object_model.get_part("display_head")
    hinge = object_model.get_articulation("printer_cover_hinge")
    tilt = object_model.get_articulation("housing_to_display_head")

    # ---- intentional overlap allowances ----

    # Paper roll axle ends captured in housing bearing slots
    ctx.allow_overlap(
        "paper_roll",
        "body",
        elem_a="roll_axle",
        elem_b="housing",
        reason="Paper roll axle ends are captured in the housing bearing slots.",
    )

    # Display hinge boss pivot seated into the housing at the slope top edge
    ctx.allow_overlap(
        "display_head",
        "body",
        elem_a="hinge_boss",
        elem_b="housing",
        reason="Display hinge boss pivot is seated into the housing at the slope top edge.",
    )
    ctx.allow_overlap(
        "display_head",
        "body",
        elem_a="hinge_boss",
        elem_b="hinge_bracket",
        reason="Display hinge boss barrel sits inside the housing hinge bracket cradle.",
    )

    # Display bezel plate seated into the housing slope surface
    ctx.allow_overlap(
        "display_head",
        "body",
        elem_a="display_bezel",
        elem_b="housing",
        reason="Display bezel plate is seated into the housing slope surface (~0.8 mm embed).",
    )

    # Hinge boss slightly overlaps printer cover front edge at the pivot
    ctx.allow_overlap(
        "display_head",
        "printer_cover",
        elem_a="hinge_boss",
        elem_b="cover_shell",
        reason="Display hinge boss barrel slightly overlaps the printer cover front edge at the shared pivot region.",
    )

    # ---- printer cover ----

    ctx.expect_contact(cover, body, contact_tol=0.0005, name="cover seats on rear deck")
    ctx.expect_within(
        cover, body, axes="xy", margin=0.003, name="closed cover stays over the body"
    )

    # ---- paper roll ----

    ctx.expect_within(
        roll,
        body,
        axes="xyz",
        inner_elem="roll_paper",
        margin=0.0005,
        name="paper roll sits inside the printer compartment",
    )

    closed_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({hinge: 1.4}):
        open_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "printer cover flips open upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.02,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    # ---- display_head tilt hinge (TARGET structural change) ----

    # Hinge boss contacts the housing bracket at the pivot
    ctx.expect_contact(
        display_head,
        body,
        elem_a="hinge_boss",
        elem_b="hinge_bracket",
        contact_tol=0.004,
        name="display hinge boss contacts housing bracket",
    )

    # Display head carries the expected visuals
    ctx.check(
        "display_head carries bezel, screen, and hinge boss",
        display_head.get_visual("display_bezel") is not None
        and display_head.get_visual("display_screen") is not None
        and display_head.get_visual("hinge_boss") is not None,
    )

    # housing_to_display_head joint exists as a revolute tilt
    ctx.check(
        "housing_to_display_head revolute joint exists",
        object_model.get_articulation("housing_to_display_head") is not None,
    )

    # Positive tilt angle raises the free edge (screen) upward
    rest_aabb = ctx.part_world_aabb(display_head)
    with ctx.pose({tilt: 0.6}):
        tilted_aabb = ctx.part_world_aabb(display_head)
    ctx.check(
        "display head free edge rises when tilted open",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[0][2] > rest_aabb[0][2] + 0.003,
        details=(
            f"rest_min_z={rest_aabb[0][2]:.4f}, "
            f"tilted_min_z={tilted_aabb[0][2]:.4f}"
        ),
    )

    # Screen remains above the keypad deck at rest
    screen_aabb = ctx.part_element_world_aabb(display_head, elem="display_screen")
    ctx.check(
        "display screen sits on the raised angled face",
        screen_aabb is not None and screen_aabb[0][2] > FRONT_DECK_Z + 0.005,
        details=f"screen_aabb={screen_aabb}",
    )

    # ---- keypad ----

    key_names = (
        ["key_1", "key_2", "key_3", "key_4", "key_5", "key_6"]
        + ["key_7", "key_8", "key_9", "key_star", "key_0", "key_hash"]
        + ["cancel_key", "clear_key", "enter_key"]
        + [f"menu_key_{i}" for i in range(4)]
    )
    ctx.check(
        "terminal has 19 pressable keys",
        all(object_model.get_part(n) is not None for n in key_names)
        and len(key_names) == 19,
    )
    for name in ("key_1", "key_hash", "enter_key", "menu_key_0"):
        key_part = object_model.get_part(name)
        ctx.expect_contact(
            key_part, body, contact_tol=0.0005, name=f"{name} rests on the deck"
        )
        ctx.expect_overlap(
            key_part,
            body,
            axes="xy",
            min_overlap=0.004,
            name=f"{name} sits over the deck",
        )

    key_5_joint = object_model.get_articulation("key_5_press")
    rest = ctx.part_world_position("key_5")
    with ctx.pose({key_5_joint: KEY_TRAVEL}):
        pressed = ctx.part_world_position("key_5")
    ctx.check(
        "key 5 presses downward",
        rest is not None and pressed is not None and pressed[2] < rest[2] - 0.0009,
        details=f"rest={rest}, pressed={pressed}",
    )

    # ---- key colors ----

    def _rgba(part_name: str) -> tuple[float, ...]:
        part = object_model.get_part(part_name)
        mat = part.visuals[0].material
        if mat is None or getattr(mat, "rgba", None) is None:
            return (0.0, 0.0, 0.0, 0.0)
        return tuple(mat.rgba)

    r = _rgba("cancel_key")
    y = _rgba("clear_key")
    g = _rgba("enter_key")
    ctx.check("cancel key is red", r[0] > 0.5 and r[1] < 0.3)
    ctx.check("clear key is yellow", y[0] > 0.7 and y[1] > 0.5 and y[2] < 0.3)
    ctx.check("enter key is green", g[1] > 0.4 and g[0] < 0.3)

    # ---- card interfaces ----

    ctx.check(
        "chip card slot liner present",
        body.get_visual("chip_slot_liner") is not None,
    )
    ctx.check(
        "magstripe swipe groove liner present",
        body.get_visual("swipe_slot_liner") is not None,
    )

    return ctx.report()


object_model = build_object_model()
