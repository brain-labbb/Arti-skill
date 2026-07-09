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

# cradle / charging dock
CRADLE_LEN = 0.230
CRADLE_WIDTH = 0.110
CRADLE_HEIGHT = 0.038  # tray top surface height above counter
WALL_THICK = 0.010
WALL_HEIGHT = 0.032  # rear upstand above tray top
WALL_EMBED = 0.003  # wall embed depth into tray for robust boolean union
DOCK_FOOT_POSITIONS = (
    (0.085, -0.038),
    (0.085, 0.038),
    (-0.085, -0.038),
    (-0.085, 0.038),
)

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


def _dock_shape() -> cq.Workplane:
    """Charging cradle: flat tray with a rear upstand wall."""
    tray = (
        cq.Workplane("XY")
        .box(CRADLE_LEN, CRADLE_WIDTH, CRADLE_HEIGHT)
        .translate((0.0, 0.0, CRADLE_HEIGHT / 2.0))
    )
    tray = tray.edges("|Z").fillet(0.008)
    try:
        tray = tray.edges(">Z").fillet(0.003)
    except Exception:
        pass

    # rear upstand wall (embedded slightly into the tray for a robust union)
    wall_h = WALL_HEIGHT + WALL_EMBED
    wall_z = CRADLE_HEIGHT + WALL_HEIGHT / 2.0 - WALL_EMBED / 2.0
    wall = (
        cq.Workplane("XY")
        .box(WALL_THICK, CRADLE_WIDTH - 0.016, wall_h)
        .translate((-CRADLE_LEN / 2.0 + WALL_THICK / 2.0 + 0.004, 0.0, wall_z))
    )
    wall = wall.edges("|Z").fillet(0.004)
    try:
        wall = wall.edges(">Z").fillet(0.003)
    except Exception:
        pass

    # small front lip to locate the terminal
    lip_h = 0.006 + WALL_EMBED
    lip_z = CRADLE_HEIGHT + 0.003 - WALL_EMBED / 2.0
    lip = (
        cq.Workplane("XY")
        .box(WALL_THICK * 0.8, CRADLE_WIDTH - 0.020, lip_h)
        .translate((CRADLE_LEN / 2.0 - WALL_THICK * 0.4 - 0.004, 0.0, lip_z))
    )
    lip = lip.edges("|Z").fillet(0.003)

    return tray.union(wall).union(lip)


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
    dock_plastic = Material(name="dock_plastic", rgba=(0.16, 0.16, 0.18, 1.0))
    contact_metal = Material(name="contact_metal", rgba=(0.82, 0.70, 0.28, 1.0))

    # ----- dock_base (root: countertop charging cradle)
    dock_base = model.part("dock_base")
    dock_base.visual(
        mesh_from_cadquery(_dock_shape(), "dock_cradle"),
        material=dock_plastic,
        name="dock_shell",
    )

    # rubber feet on the underside of the cradle
    for i, (fx, fy) in enumerate(DOCK_FOOT_POSITIONS):
        dock_base.visual(
            Cylinder(radius=0.006, length=0.0032),
            origin=Origin(xyz=(fx, fy, -0.0016)),
            material=rubber,
            name=f"foot_{i}",
        )

    # charging contact pads on the cradle tray top
    for i, cy in enumerate((-0.012, 0.012)):
        dock_base.visual(
            Box((0.008, 0.004, 0.001)),
            origin=Origin(xyz=(-0.030, cy, CRADLE_HEIGHT + 0.0005)),
            material=contact_metal,
            name=f"contact_pad_{i}",
        )

    # ----- body (terminal housing, child of dock_base via tilt joint)
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_housing_shape(), "housing"),
        material=silver,
        name="housing",
    )

    # angled display bezel + lit screen, seated into the sloped face
    bezel_center = _on_slope(0.00145)  # 4.5 mm bezel, ~0.8 mm embedded
    body.visual(
        Box((0.040, 0.060, 0.0045)),
        origin=Origin(xyz=bezel_center, rpy=(0.0, SLOPE_PITCH, 0.0)),
        material=dark,
        name="display_bezel",
    )
    screen_center = _on_slope(0.0039)
    body.visual(
        Box((0.030, 0.048, 0.0008)),
        origin=Origin(xyz=screen_center, rpy=(0.0, SLOPE_PITCH, 0.0)),
        material=screen_blue,
        name="display_screen",
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

    # tilt articulation: terminal reclines against the cradle upstand
    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=dock_base,
        child=body,
        # pivot near the rear of the cradle tray, at the tray top surface
        origin=Origin(xyz=(-0.005, 0.0, CRADLE_HEIGHT)),
        # positive q raises the front (keypad) end, reclining the display
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=0.35),
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
    dock_base = object_model.get_part("dock_base")
    body = object_model.get_part("body")
    cover = object_model.get_part("printer_cover")
    roll = object_model.get_part("paper_roll")
    hinge = object_model.get_articulation("printer_cover_hinge")
    tilt = object_model.get_articulation("base_to_body")

    # The spindle axle ends are intentionally captured in the compartment
    # side walls, standing in for the real snap-in bearing slots.
    ctx.allow_overlap(
        "paper_roll",
        "body",
        elem_a="roll_axle",
        elem_b="housing",
        reason="Paper roll axle ends are captured in the housing bearing slots.",
    )

    # The terminal housing sits on the cradle tray; small mesh intersection
    # at the contact surface is expected for a seated part.
    ctx.allow_overlap(
        "body",
        "dock_base",
        elem_a="housing",
        elem_b="dock_shell",
        reason="Terminal housing is seated on the cradle tray surface.",
    )

    # printer cover seats on the rear deck rim and stays inside the footprint
    ctx.expect_contact(cover, body, contact_tol=0.0005, name="cover seats on rear deck")
    ctx.expect_within(
        cover, body, axes="xy", margin=0.003, name="closed cover stays over the body"
    )

    # paper roll is held inside the compartment
    ctx.expect_within(
        roll,
        body,
        axes="xyz",
        inner_elem="roll_paper",
        margin=0.0005,
        name="paper roll sits inside the printer compartment",
    )

    # cover opens upward to expose the paper compartment
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

    # keypad: 19 pressable keys total, seated on the deck
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

    # pressing a key moves it straight down
    key_5_joint = object_model.get_articulation("key_5_press")
    rest = ctx.part_world_position("key_5")
    with ctx.pose({key_5_joint: KEY_TRAVEL}):
        pressed = ctx.part_world_position("key_5")
    ctx.check(
        "key 5 presses downward",
        rest is not None and pressed is not None and pressed[2] < rest[2] - 0.0009,
        details=f"rest={rest}, pressed={pressed}",
    )

    # colored function keys carry the correct colors
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

    # angled display: screen visual sits above the keypad deck on the slope
    screen_aabb = ctx.part_element_world_aabb(body, elem="display_screen")
    ctx.check(
        "display screen sits on the raised angled face",
        screen_aabb is not None and screen_aabb[0][2] > FRONT_DECK_Z + 0.005,
        details=f"screen_aabb={screen_aabb}",
    )

    # card interfaces are present
    ctx.check(
        "chip card slot liner present",
        body.get_visual("chip_slot_liner") is not None,
    )
    ctx.check(
        "magstripe swipe groove liner present",
        body.get_visual("swipe_slot_liner") is not None,
    )

    # dock_base cradle exists with the charging dock geometry
    ctx.check(
        "dock_base cradle shell present",
        dock_base.get_visual("dock_shell") is not None,
    )
    ctx.check(
        "dock_base has rubber feet",
        dock_base.get_visual("foot_0") is not None
        and dock_base.get_visual("foot_1") is not None
        and dock_base.get_visual("foot_2") is not None
        and dock_base.get_visual("foot_3") is not None,
    )
    ctx.check(
        "dock_base has charging contact pads",
        dock_base.get_visual("contact_pad_0") is not None
        and dock_base.get_visual("contact_pad_1") is not None,
    )

    # terminal body is seated in the cradle (not floating)
    ctx.expect_contact(
        body,
        dock_base,
        contact_tol=0.005,
        name="terminal body contacts the cradle tray",
    )
    ctx.expect_within(
        body,
        dock_base,
        axes="xy",
        margin=0.010,
        name="terminal body sits within the cradle footprint",
    )

    # tilt joint reclines the terminal head
    rest_aabb = ctx.part_world_aabb(body)
    with ctx.pose({tilt: 0.35}):
        tilted_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base_to_body tilt joint reclines the terminal",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][2] > rest_aabb[1][2] + 0.01,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
