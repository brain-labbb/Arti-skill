"""Electronic cash register on a cash-drawer base.

Dark charcoal register (Sharp ER-A347 style) sitting on a matching lockable
cash drawer base: sloped multi-color keypad deck, twin receipt paper rolls,
tiltable rear operator LCD, pole-mounted swiveling customer display, and a
sliding cash drawer with a seven-slot bill till, hinged bill clips, and six
front coin compartments.
"""

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
# Global frame: origin at bottom center of the drawer housing.
# +Y is toward the cashier (front, drawer slide direction), +Z is up.

HOUSING_W = 0.44
HOUSING_D = 0.42
HOUSING_H = 0.115

BODY_W = 0.40
BODY_Y_REAR = -0.19
BODY_Y_FRONT = 0.185
BODY_Z0 = HOUSING_H  # 0.115
REAR_TOP_REAR = 0.272
REAR_TOP_FRONT = 0.268
PRINTER_FRONT_Y = -0.05
DECK_Z_REAR = 0.212
DECK_Z_FRONT = 0.168

# Keyboard deck slope (top surface from (y=-0.05, z=0.212) to (y=0.185, z=0.168)).
DECK_PITCH = math.atan2(DECK_Z_REAR - DECK_Z_FRONT, BODY_Y_FRONT - PRINTER_FRONT_Y)
COS_P = math.cos(DECK_PITCH)
SIN_P = math.sin(DECK_PITCH)

KEY_H = 0.011
KEY_TRAVEL = 0.0025
KEY_SEAT_EMBED = 0.0004  # keycap skirt seats slightly into the deck (allowed)

DRAWER_TRAVEL = 0.30
CLIP_ARM_ANGLE = 0.315  # rad, rest droop of the bill clip arm
BODY_SEAT_EMBED = 0.0005  # register body seats slightly into the housing top


def deck_point(x: float, s: float) -> tuple[float, float, float]:
    """Point on the sloped key deck; s is the down-slope distance from the deck rear edge."""
    return (x, PRINTER_FRONT_Y + s * COS_P, DECK_Z_REAR - s * SIN_P)


def _cq_box(sx: float, sy: float, sz: float, cx: float, cy: float, cz: float) -> cq.Workplane:
    return cq.Workplane("XY").transformed(offset=(cx, cy, cz)).box(sx, sy, sz)


# ---------------------------------------------------------------- CadQuery solids


def _housing_shell() -> cq.Workplane:
    """Hollow drawer housing: bottom, top, rear and side walls; open front."""
    bottom = _cq_box(HOUSING_W, HOUSING_D, 0.012, 0.0, 0.0, 0.006)
    top = _cq_box(HOUSING_W, HOUSING_D, 0.012, 0.0, 0.0, 0.109)
    rear = _cq_box(HOUSING_W, 0.014, 0.095, 0.0, -0.203, 0.0575)
    left = _cq_box(0.014, HOUSING_D, 0.095, -0.213, 0.0, 0.0575)
    right = _cq_box(0.014, HOUSING_D, 0.095, 0.213, 0.0, 0.0575)
    return bottom.union(top).union(rear).union(left).union(right)


def _register_shell() -> cq.Workplane:
    """Main register housing: raised printer section, sloped key deck, roll wells."""
    profile = (
        cq.Workplane("YZ")
        .moveTo(BODY_Y_REAR, BODY_Z0 - BODY_SEAT_EMBED)
        .lineTo(BODY_Y_REAR, REAR_TOP_REAR)
        .lineTo(PRINTER_FRONT_Y, REAR_TOP_FRONT)
        .lineTo(PRINTER_FRONT_Y, DECK_Z_REAR)
        .lineTo(BODY_Y_FRONT, DECK_Z_FRONT)
        .lineTo(BODY_Y_FRONT, BODY_Z0 - BODY_SEAT_EMBED)
        .close()
        .extrude(BODY_W / 2, both=True)
    )
    profile = profile.edges("|Z").fillet(0.006)
    # Twin paper-roll wells cut into the printer section top.
    well_a = _cq_box(0.090, 0.080, 0.098, -0.105, -0.12, 0.261)
    well_b = _cq_box(0.080, 0.080, 0.098, 0.010, -0.12, 0.261)
    return profile.cut(well_a).cut(well_b)


def _till_insert() -> cq.Workplane:
    """Drawer tray: solid block with seven bill slots and six coin compartments."""
    # The tray bottom seats 0.5 mm into the housing floor so it reads as riding on it.
    block = _cq_box(0.40, 0.3935, 0.0645, 0.0, 0.01425, 0.04375)
    # Seven bill compartments (rear half).
    BILL_COUNT = 7
    bill_gap = 0.006
    bill_w = (0.376 - (BILL_COUNT - 1) * bill_gap) / BILL_COUNT
    for i in range(BILL_COUNT):
        cx = -0.188 + bill_w / 2 + i * (bill_w + bill_gap)
        block = block.cut(_cq_box(bill_w, 0.158, 0.060, cx, -0.091, 0.050))
    # Six coin compartments (front half).
    coin_w = (0.376 - 5 * 0.006) / 6
    for i in range(6):
        cx = -0.188 + coin_w / 2 + i * (coin_w + 0.006)
        block = block.cut(_cq_box(coin_w, 0.1955, 0.060, cx, 0.09775, 0.050))
    return block


def _bill_clip() -> cq.Workplane:
    """Hinged bill clip: hinge tube, sloped pressing arm, flat pad. Local origin on hinge axis."""
    tube = cq.Workplane("YZ").circle(0.004).extrude(0.040).translate((-0.020, 0.0, 0.0))
    arm = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.071, 0.0))
        .box(0.028, 0.142, 0.003)
        .rotate((-1.0, 0.0, 0.0), (1.0, 0.0, 0.0), -math.degrees(CLIP_ARM_ANGLE))
    )
    pad = _cq_box(0.042, 0.028, 0.0035, 0.0, 0.135, -0.0455)
    return tube.union(arm).union(pad)


# ---------------------------------------------------------------- model


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electronic_cash_register")

    charcoal = model.material("charcoal", rgba=(0.25, 0.22, 0.20, 1.0))
    charcoal_dark = model.material("charcoal_dark", rgba=(0.17, 0.15, 0.14, 1.0))
    till_gray = model.material("till_gray", rgba=(0.18, 0.18, 0.19, 1.0))
    clip_black = model.material("clip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    key_white = model.material("key_white", rgba=(0.90, 0.89, 0.86, 1.0))
    key_gray = model.material("key_gray", rgba=(0.52, 0.52, 0.54, 1.0))
    key_blue = model.material("key_blue", rgba=(0.28, 0.38, 0.62, 1.0))
    key_red = model.material("key_red", rgba=(0.72, 0.22, 0.22, 1.0))
    key_cyan = model.material("key_cyan", rgba=(0.35, 0.72, 0.78, 1.0))
    key_dark = model.material("key_dark", rgba=(0.20, 0.22, 0.30, 1.0))
    paper_white = model.material("paper_white", rgba=(0.95, 0.95, 0.93, 1.0))
    screen_blue = model.material("screen_blue", rgba=(0.62, 0.72, 0.82, 1.0))
    display_black = model.material("display_black", rgba=(0.05, 0.05, 0.06, 1.0))
    digit_green = model.material("digit_green", rgba=(0.25, 0.95, 0.35, 1.0))
    silver = model.material("silver", rgba=(0.75, 0.75, 0.78, 1.0))

    # ---- root: drawer housing -------------------------------------------
    housing = model.part("drawer_housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "housing_shell"),
        material=charcoal_dark,
        name="housing_shell",
    )

    # ---- register body (fixed on the housing top) -----------------------
    body = model.part("register_body")
    body.visual(
        mesh_from_cadquery(_register_shell(), "register_shell"),
        material=charcoal,
        name="body_shell",
    )
    # Paper rolls resting in the wells (well floors at z=0.212).
    body.visual(
        Cylinder(radius=0.0285, length=0.085),
        origin=Origin(xyz=(-0.105, -0.12, 0.240), rpy=(0.0, math.pi / 2, 0.0)),
        material=paper_white,
        name="receipt_roll",
    )
    body.visual(
        Cylinder(radius=0.0285, length=0.075),
        origin=Origin(xyz=(0.010, -0.12, 0.240), rpy=(0.0, math.pi / 2, 0.0)),
        material=paper_white,
        name="journal_roll",
    )
    # Paper strips rising off the rolls, tilted slightly forward.
    body.visual(
        Box((0.060, 0.0012, 0.075)),
        origin=Origin(xyz=(-0.105, -0.138, 0.2925), rpy=(0.12, 0.0, 0.0)),
        material=paper_white,
        name="receipt_paper",
    )
    body.visual(
        Box((0.050, 0.0012, 0.060)),
        origin=Origin(xyz=(0.010, -0.138, 0.285), rpy=(0.12, 0.0, 0.0)),
        material=paper_white,
        name="journal_paper",
    )
    # Operator LCD mounting bracket on the printer-section top.
    body.visual(
        Box((0.032, 0.020, 0.040)),
        origin=Origin(xyz=(0.10, -0.12, 0.288)),
        material=charcoal,
        name="lcd_bracket",
    )
    # Customer display pole at the rear.
    body.visual(
        Cylinder(radius=0.010, length=0.126),
        origin=Origin(xyz=(-0.10, -0.174, 0.3325)),
        material=display_black,
        name="display_pole",
    )

    model.articulation(
        "housing_to_body",
        ArticulationType.FIXED,
        parent=housing,
        child=body,
        origin=Origin(),
    )

    # ---- cash drawer (prismatic, slides toward the cashier) -------------
    drawer = model.part("cash_drawer")
    drawer.visual(
        Box((0.436, 0.016, 0.111)),
        origin=Origin(xyz=(0.0, 0.2185, 0.0575)),
        material=charcoal_dark,
        name="drawer_front",
    )
    drawer.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.13, 0.2285, 0.045), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=silver,
        name="lock_cylinder",
    )
    drawer.visual(
        mesh_from_cadquery(_till_insert(), "till"),
        material=till_gray,
        name="till",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.4, lower=0.0, upper=DRAWER_TRAVEL),
    )

    # ---- hinged bill clips in the seven bill slots -----------------------
    clip_mesh = mesh_from_cadquery(_bill_clip(), "bill_clip")
    BILL_COUNT = 7
    bill_gap = 0.006
    bill_w = (0.376 - (BILL_COUNT - 1) * bill_gap) / BILL_COUNT
    for i in range(BILL_COUNT):
        cx = -0.188 + bill_w / 2 + i * (bill_w + bill_gap)
        clip = model.part(f"bill_clip_{i}")
        clip.visual(clip_mesh, material=clip_black, name="bill_clip")
        model.articulation(
            f"bill_clip_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=drawer,
            child=clip,
            # Hinge tube captured in the till rear wall, axis across the slot.
            origin=Origin(xyz=(cx, -0.167, 0.070)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.35),
        )

    # ---- keypad ----------------------------------------------------------
    def add_key(name: str, x: float, s: float, kw: float, kd: float, mat) -> None:
        key = model.part(name)
        key.visual(
            Box((kw, kd, KEY_H)),
            origin=Origin(xyz=(0.0, 0.0, KEY_H / 2 - KEY_SEAT_EMBED)),
            material=mat,
            name="keycap",
        )
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=deck_point(x, s), rpy=(-DECK_PITCH, 0.0, 0.0)),
            axis=(0.0, 0.0, -1.0),  # positive q presses the key into the deck
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=KEY_TRAVEL),
        )

    # Left function block: 3 cols x 4 rows, mixed colors.
    fn_colors = [
        [key_blue, key_blue, key_gray],
        [key_gray, key_blue, key_gray],
        [key_red, key_blue, key_gray],
        [key_red, key_blue, key_gray],
    ]
    fn_x = [-0.166, -0.143, -0.120]
    fn_s = [0.075, 0.099, 0.123, 0.147]
    for r in range(4):
        for c in range(3):
            add_key(f"function_key_{r}_{c}", fn_x[c], fn_s[r], 0.017, 0.017, fn_colors[r][c])

    # Two paper-feed keys above the numeric pad.
    add_key("feed_key_0", -0.075, 0.052, 0.030, 0.015, key_gray)
    add_key("feed_key_1", -0.030, 0.052, 0.030, 0.015, key_gray)

    # Numeric pad: 4 cols x 4 rows, white with one gray clear key.
    num_x = [-0.085, -0.058, -0.031, -0.004]
    num_s = [0.085, 0.111, 0.137, 0.163]
    for r in range(4):
        for c in range(4):
            mat = key_gray if (r == 0 and c == 3) else key_white
            add_key(f"numeric_key_{r}_{c}", num_x[c], num_s[r], 0.022, 0.019, mat)

    # Department key grid: 4 cols x 5 rows, blue top row.
    dept_x = [0.040, 0.063, 0.086, 0.109]
    dept_s = [0.060, 0.083, 0.106, 0.129, 0.152]
    for r in range(5):
        for c in range(4):
            mat = key_blue if r == 0 else key_gray
            add_key(f"dept_key_{r}_{c}", dept_x[c], dept_s[r], 0.018, 0.016, mat)

    # Right mode column and the wide cyan total key.
    for i, s in enumerate([0.060, 0.083, 0.106, 0.129]):
        add_key(f"mode_key_{i}", 0.150, s, 0.024, 0.016, key_dark)
    add_key("total_key", 0.150, 0.165, 0.024, 0.030, key_cyan)

    # ---- tiltable operator LCD ------------------------------------------
    lcd = model.part("lcd_display")
    lcd.visual(
        Box((0.120, 0.016, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=display_black,
        name="lcd_housing",
    )
    lcd.visual(
        Box((0.096, 0.002, 0.060)),
        origin=Origin(xyz=(0.0, 0.0085, 0.042)),
        material=screen_blue,
        name="lcd_screen",
    )
    model.articulation(
        "lcd_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lcd,
        origin=Origin(xyz=(0.10, -0.12, 0.305)),
        axis=(1.0, 0.0, 0.0),  # positive q tilts the screen back toward the rear
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-0.40, upper=0.40),
    )

    # ---- pole-mounted customer display (swivels on the pole) ------------
    head = model.part("customer_display_head")
    head.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=display_black,
        name="pole_collar",
    )
    head.visual(
        Box((0.150, 0.032, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=display_black,
        name="head_shell",
    )
    head.visual(
        Box((0.128, 0.0016, 0.030)),
        origin=Origin(xyz=(0.0, 0.0163, 0.040)),
        material=charcoal_dark,
        name="digit_window",
    )
    head.visual(
        Box((0.105, 0.0014, 0.014)),
        origin=Origin(xyz=(0.0, 0.0174, 0.040)),
        material=digit_green,
        name="digit_band",
    )
    model.articulation(
        "display_head_yaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=head,
        origin=Origin(xyz=(-0.10, -0.174, 0.394)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-1.6, upper=1.6),
    )

    return model


# ---------------------------------------------------------------- tests


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("drawer_housing")
    body = object_model.get_part("register_body")
    drawer = object_model.get_part("cash_drawer")
    lcd = object_model.get_part("lcd_display")
    head = object_model.get_part("customer_display_head")
    slide = object_model.get_articulation("drawer_slide")

    # Intentional local captures (hinge-pin / socket style embeds).
    for i in range(7):
        ctx.allow_overlap(
            f"bill_clip_{i}",
            drawer,
            elem_a="bill_clip",
            elem_b="till",
            reason="Bill clip hinge tube is captured in the till rear wall like a hinge pin.",
        )
    ctx.allow_overlap(
        lcd,
        body,
        elem_a="lcd_housing",
        elem_b="lcd_bracket",
        reason="Operator LCD hinge boss seats over the mounting bracket for the tilt joint.",
    )
    ctx.allow_overlap(
        head,
        body,
        elem_a="pole_collar",
        elem_b="display_pole",
        reason="Customer display collar sockets over the pole top for the swivel joint.",
    )
    ctx.allow_overlap(
        drawer,
        housing,
        elem_a="till",
        elem_b="housing_shell",
        reason="Drawer tray bottom rides directly on the housing floor like a drawer slide.",
    )
    for part in object_model.parts:
        if "key" in part.name:
            ctx.allow_overlap(
                part,
                body,
                elem_a="keycap",
                elem_b="body_shell",
                reason="Keycap skirt seats 0.4 mm into the sloped deck like a key well.",
            )

    # Register body seats flush on the drawer housing top.
    ctx.expect_gap(
        body,
        housing,
        axis="z",
        max_penetration=0.001,
        max_gap=0.001,
        name="register body seats on the drawer housing top",
    )

    # Closed drawer nests inside the housing.
    ctx.expect_within(
        drawer,
        housing,
        axes="x",
        margin=0.002,
        name="closed drawer stays within the housing width",
    )
    ctx.expect_within(
        drawer,
        housing,
        axes="z",
        inner_elem="till",
        margin=0.001,
        name="till insert stays below the housing top plate",
    )
    ctx.expect_overlap(
        drawer,
        housing,
        axes="y",
        min_overlap=0.30,
        name="closed drawer is fully nested in the housing",
    )

    # Drawer slides out toward the cashier and stays retained.
    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({slide: DRAWER_TRAVEL}):
        open_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            housing,
            axes="y",
            min_overlap=0.08,
            name="open drawer retains insertion in the housing",
        )
    ctx.check(
        "drawer opens forward",
        rest_pos is not None
        and open_pos is not None
        and open_pos[1] > rest_pos[1] + 0.9 * DRAWER_TRAVEL,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    # Bill clips: seven of them, hinged in the till, lifting upward.
    clips = [p for p in object_model.parts if p.name.startswith("bill_clip_")]
    ctx.check("seven bill clips", len(clips) == 7, details=f"found {len(clips)}")
    ctx.expect_contact(
        "bill_clip_0",
        drawer,
        contact_tol=1e-5,
        name="bill clip hinge engages the till",
    )
    clip_joint = object_model.get_articulation("bill_clip_0_hinge")
    rest_aabb = ctx.part_world_aabb("bill_clip_0")
    with ctx.pose({clip_joint: 1.2}):
        lifted_aabb = ctx.part_world_aabb("bill_clip_0")
    ctx.check(
        "bill clip lifts upward",
        rest_aabb is not None
        and lifted_aabb is not None
        and lifted_aabb[1][2] > rest_aabb[1][2] + 0.05,
        details=f"rest_top={rest_aabb}, lifted_top={lifted_aabb}",
    )

    # Keypad: 55 individual keys resting on the sloped deck; a key press moves inward.
    keys = [p for p in object_model.parts if "key" in p.name]
    ctx.check("55 keypad keys", len(keys) == 55, details=f"found {len(keys)}")
    ctx.expect_contact(
        "numeric_key_0_0",
        body,
        contact_tol=1e-4,
        name="numeric key rests on the key deck",
    )
    key_joint = object_model.get_articulation("numeric_key_1_1_press")
    key_rest = ctx.part_world_position("numeric_key_1_1")
    with ctx.pose({key_joint: KEY_TRAVEL}):
        key_pressed = ctx.part_world_position("numeric_key_1_1")
    ctx.check(
        "key press travels into the deck",
        key_rest is not None
        and key_pressed is not None
        and key_pressed[2] < key_rest[2] - 0.001,
        details=f"rest={key_rest}, pressed={key_pressed}",
    )

    # Operator LCD is seated on its bracket and tilts backward.
    ctx.expect_contact(
        lcd,
        body,
        elem_a="lcd_housing",
        elem_b="lcd_bracket",
        name="LCD housing seats on the mounting bracket",
    )
    tilt = object_model.get_articulation("lcd_tilt")
    lcd_rest = ctx.part_world_aabb(lcd)
    with ctx.pose({tilt: 0.35}):
        lcd_tilted = ctx.part_world_aabb(lcd)
    ctx.check(
        "LCD tilts toward the rear",
        lcd_rest is not None
        and lcd_tilted is not None
        and lcd_tilted[0][1] < lcd_rest[0][1] - 0.01,
        details=f"rest={lcd_rest}, tilted={lcd_tilted}",
    )

    # Customer display head sockets on the pole and swivels around it.
    ctx.expect_contact(
        head,
        body,
        elem_a="pole_collar",
        elem_b="display_pole",
        name="customer display collar captures the pole",
    )
    yaw = object_model.get_articulation("display_head_yaw")
    head_rest = ctx.part_world_aabb(head)
    with ctx.pose({yaw: 1.3}):
        head_turned = ctx.part_world_aabb(head)
    ctx.check(
        "customer display swivels on the pole",
        head_rest is not None
        and head_turned is not None
        and (head_turned[1][0] - head_turned[0][0]) < (head_rest[1][0] - head_rest[0][0]) - 0.03,
        details=f"rest={head_rest}, turned={head_turned}",
    )

    # Twin paper rolls with paper strips rising above the wells.
    for elem in ("receipt_roll", "journal_roll", "receipt_paper", "journal_paper"):
        aabb = ctx.part_element_world_aabb(body, elem=elem)
        ctx.check(f"{elem} present", aabb is not None, details=f"aabb={aabb}")
    paper_aabb = ctx.part_element_world_aabb(body, elem="receipt_paper")
    ctx.check(
        "receipt paper rises above the printer housing",
        paper_aabb is not None and paper_aabb[1][2] > 0.30,
        details=f"aabb={paper_aabb}",
    )

    # Drawer front carries the lock cylinder proud of the panel.
    lock_aabb = ctx.part_element_world_aabb(drawer, elem="lock_cylinder")
    front_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_front")
    ctx.check(
        "lock cylinder sits proud of the drawer front",
        lock_aabb is not None and front_aabb is not None and lock_aabb[1][1] > front_aabb[1][1],
        details=f"lock={lock_aabb}, front={front_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
