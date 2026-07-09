"""Modern POS cash register on a cash-drawer base.

Flat POS terminal base (light gray shell) with an upright tilting operator
screen on a short neck, numeric keypad with total key, single receipt paper
roll, pole-mounted swiveling customer display, and a sliding cash drawer
with a five-slot bill till, hinged bill clips, and six front coin
compartments.
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

# Flat POS control deck
BODY_SEAT_EMBED = 0.0005  # register body seats slightly into the housing top
DECK_W = 0.40
DECK_D = 0.38
DECK_H = 0.050
DECK_Z0 = HOUSING_H  # 0.115
DECK_Z_TOP = DECK_Z0 + DECK_H - BODY_SEAT_EMBED  # actual deck surface: 0.1645
DECK_REAR_Y = -DECK_D / 2  # -0.19

# LCD neck post
NECK_H = 0.10
NECK_W = 0.040
NECK_D = 0.030
NECK_X = 0.06
NECK_Y = -0.13

# Customer display pole
POLE_H = 0.25
POLE_X = -0.12
POLE_Y = -0.15

KEY_H = 0.011
KEY_TRAVEL = 0.0025
KEY_SEAT_EMBED = 0.0004  # keycap skirt seats slightly into the deck (allowed)

DRAWER_TRAVEL = 0.30
CLIP_ARM_ANGLE = 0.315  # rad, rest droop of the bill clip arm


def deck_point(x: float, s: float) -> tuple[float, float, float]:
    """Point on the flat key deck; x is lateral, s is distance from rear edge."""
    return (x, DECK_REAR_Y + s, DECK_Z_TOP)


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


def _pos_deck() -> cq.Workplane:
    """Flat POS control deck: low box with rounded vertical edges."""
    deck = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, DECK_Z0 + DECK_H / 2 - BODY_SEAT_EMBED))
        .box(DECK_W, DECK_D, DECK_H)
    )
    deck = deck.edges("|Z").fillet(0.005)
    return deck


def _till_insert() -> cq.Workplane:
    """Drawer tray: solid block with five bill slots and six coin compartments."""
    block = _cq_box(0.40, 0.3935, 0.0645, 0.0, 0.01425, 0.04375)
    bill_w = 0.0704
    for i in range(5):
        cx = -0.188 + bill_w / 2 + i * (bill_w + 0.006)
        block = block.cut(_cq_box(bill_w, 0.158, 0.060, cx, -0.091, 0.050))
    coin_w = (0.376 - 5 * 0.006) / 6
    for i in range(6):
        cx = -0.188 + coin_w / 2 + i * (coin_w + 0.006)
        block = block.cut(_cq_box(coin_w, 0.1955, 0.060, cx, 0.09775, 0.050))
    return block


def _bill_clip() -> cq.Workplane:
    """Hinged bill clip: hinge tube, sloped pressing arm, flat pad."""
    tube = cq.Workplane("YZ").circle(0.0045).extrude(0.050).translate((-0.025, 0.0, 0.0))
    arm = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.071, 0.0))
        .box(0.030, 0.142, 0.003)
        .rotate((-1.0, 0.0, 0.0), (1.0, 0.0, 0.0), -math.degrees(CLIP_ARM_ANGLE))
    )
    pad = _cq_box(0.052, 0.028, 0.0035, 0.0, 0.135, -0.0455)
    return tube.union(arm).union(pad)


# ---------------------------------------------------------------- model


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electronic_cash_register")

    # Lighter POS palette
    shell_light = model.material("shell_light", rgba=(0.82, 0.82, 0.84, 1.0))
    shell_mid = model.material("shell_mid", rgba=(0.68, 0.68, 0.70, 1.0))
    till_gray = model.material("till_gray", rgba=(0.18, 0.18, 0.19, 1.0))
    clip_black = model.material("clip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    key_white = model.material("key_white", rgba=(0.90, 0.89, 0.86, 1.0))
    key_gray = model.material("key_gray", rgba=(0.52, 0.52, 0.54, 1.0))
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
        material=shell_mid,
        name="housing_shell",
    )

    # ---- register body (flat POS deck fixed on the housing top) ---------
    body = model.part("register_body")
    body.visual(
        mesh_from_cadquery(_pos_deck(), "pos_deck"),
        material=shell_light,
        name="body_shell",
    )
    # LCD support neck rising from the deck rear area.
    body.visual(
        Box((NECK_W, NECK_D, NECK_H)),
        origin=Origin(xyz=(NECK_X, NECK_Y, DECK_Z_TOP + NECK_H / 2)),
        material=shell_mid,
        name="lcd_neck",
    )
    # Single receipt paper roll sitting on the deck (rear-left area).
    body.visual(
        Cylinder(radius=0.028, length=0.080),
        origin=Origin(
            xyz=(-0.10, -0.08, DECK_Z_TOP + 0.028),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=paper_white,
        name="receipt_roll",
    )
    # Paper strip rising from the deck (emerges from behind the roll).
    body.visual(
        Box((0.060, 0.0012, 0.065)),
        origin=Origin(xyz=(-0.10, -0.10, DECK_Z_TOP + 0.0325)),
        material=paper_white,
        name="receipt_paper",
    )
    # Customer display pole rising from the deck rear.
    body.visual(
        Cylinder(radius=0.010, length=POLE_H),
        origin=Origin(xyz=(POLE_X, POLE_Y, DECK_Z_TOP + POLE_H / 2)),
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
        material=shell_mid,
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

    # ---- hinged bill clips in the five bill slots ------------------------
    clip_mesh = mesh_from_cadquery(_bill_clip(), "bill_clip")
    bill_w = 0.0704
    for i in range(5):
        cx = -0.188 + bill_w / 2 + i * (bill_w + 0.006)
        clip = model.part(f"bill_clip_{i}")
        clip.visual(clip_mesh, material=clip_black, name="bill_clip")
        model.articulation(
            f"bill_clip_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=drawer,
            child=clip,
            origin=Origin(xyz=(cx, -0.167, 0.070)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.35),
        )

    # ---- keypad on the flat deck ----------------------------------------
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
            origin=Origin(xyz=deck_point(x, s)),
            axis=(0.0, 0.0, -1.0),  # positive q presses the key into the deck
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=KEY_TRAVEL),
        )

    # Numeric pad: 4 cols x 4 rows, white with one gray clear key.
    num_x = [-0.055, -0.028, -0.001, 0.026]
    num_s = [0.22, 0.246, 0.272, 0.298]
    for r in range(4):
        for c in range(4):
            mat = key_gray if (r == 0 and c == 3) else key_white
            add_key(f"numeric_key_{r}_{c}", num_x[c], num_s[r], 0.022, 0.019, mat)

    # Wide cyan total key to the right of the numeric pad.
    add_key("total_key", 0.080, 0.265, 0.024, 0.030, key_cyan)

    # ---- tiltable operator LCD on the upright neck ----------------------
    lcd = model.part("lcd_display")
    lcd.visual(
        Box((0.150, 0.016, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=display_black,
        name="lcd_housing",
    )
    lcd.visual(
        Box((0.130, 0.002, 0.090)),
        origin=Origin(xyz=(0.0, 0.009, 0.058)),
        material=screen_blue,
        name="lcd_screen",
    )
    model.articulation(
        "lcd_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lcd,
        # Pivot at the top of the neck; screen extends upward from here.
        origin=Origin(xyz=(NECK_X, NECK_Y, DECK_Z_TOP + NECK_H)),
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
        material=shell_mid,
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
        origin=Origin(xyz=(POLE_X, POLE_Y, DECK_Z_TOP + POLE_H)),
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
    for i in range(5):
        ctx.allow_overlap(
            f"bill_clip_{i}",
            drawer,
            elem_a="bill_clip",
            elem_b="till",
            reason="Bill clip hinge tube is captured in the till rear wall like a hinge pin.",
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
                reason="Keycap skirt seats 0.4 mm into the flat deck like a key well.",
            )

    # -- Structural change: register_body is a flat POS deck, not a wedge --
    body_shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    deck_height = body_shell_aabb[1][2] - body_shell_aabb[0][2]
    ctx.check(
        "register_body is a flat POS deck not a tall wedge",
        deck_height < 0.08,
        details=f"body_shell height = {deck_height:.4f}m",
    )

    # LCD screen is mounted upright on the neck above the deck surface.
    lcd_screen_aabb = ctx.part_element_world_aabb(lcd, elem="lcd_screen")
    ctx.check(
        "LCD screen is mounted upright on the neck",
        lcd_screen_aabb is not None and lcd_screen_aabb[0][2] > DECK_Z_TOP + 0.04,
        details=f"lcd_screen bottom z = {lcd_screen_aabb}",
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

    # Bill clips: five of them, hinged in the till, lifting upward.
    clips = [p for p in object_model.parts if p.name.startswith("bill_clip_")]
    ctx.check("five bill clips", len(clips) == 5, details=f"found {len(clips)}")
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

    # Keypad: 17 individual keys (16 numeric + 1 total) on the flat deck.
    keys = [p for p in object_model.parts if "key" in p.name]
    ctx.check("17 keypad keys", len(keys) == 17, details=f"found {len(keys)}")
    ctx.expect_contact(
        "numeric_key_0_0",
        body,
        contact_tol=1e-4,
        name="numeric key rests on the flat deck",
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

    # Operator LCD is seated on its neck and tilts backward.
    ctx.expect_contact(
        lcd,
        body,
        elem_a="lcd_housing",
        elem_b="lcd_neck",
        name="LCD housing seats on the neck top",
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

    # Single receipt roll with paper strip rising above the deck.
    for elem in ("receipt_roll", "receipt_paper"):
        aabb = ctx.part_element_world_aabb(body, elem=elem)
        ctx.check(f"{elem} present", aabb is not None, details=f"aabb={aabb}")
    paper_aabb = ctx.part_element_world_aabb(body, elem="receipt_paper")
    ctx.check(
        "receipt paper rises above the deck",
        paper_aabb is not None and paper_aabb[1][2] > DECK_Z_TOP + 0.03,
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
