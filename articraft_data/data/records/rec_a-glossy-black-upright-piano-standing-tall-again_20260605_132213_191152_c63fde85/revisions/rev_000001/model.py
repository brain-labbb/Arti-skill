from __future__ import annotations

# Glossy black upright piano: a tall vertical cabinet (back, sides, top lid),
# an upper front panel and lower kickboard, a protruding keyboard with a full
# set of black-and-white keys, a fold-down fallboard, three brass pedals near
# the floor, and a base plinth.

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ---------------------------------------------------------------------------
# Footprint / height constants (meters, floor at z=0).
# ---------------------------------------------------------------------------
HALF_W = 0.730           # half cabinet width
SIDE_THK = 0.050
DEPTH_BACK_Y = 0.560     # back of the cabinet
SHELL_BOTTOM_Z = 0.050
SHELL_TOP_Z = 1.190      # top of the side panels (closed lid rests here)

KEYBED_TOP_Z = 0.696
WHITE_KEY_H = 0.024
WHITE_KEY_TOP_Z = KEYBED_TOP_Z + WHITE_KEY_H  # 0.720
BLACK_KEY_H = 0.013

KEYBOARD_FRONT_Y = -0.135
N_WHITE = 52
WHITE_PITCH = 0.0234
WHITE_KEY_W = 0.0218
WHITE_KEY_DEPTH = 0.145

LEDGE_TOP_Z = 0.740
LID_THK = 0.030


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_piano")

    black = model.material("black_gloss", rgba=(0.045, 0.045, 0.055, 1.0))
    black_satin = model.material("black_satin", rgba=(0.080, 0.080, 0.090, 1.0))
    ivory = model.material("key_ivory", rgba=(0.945, 0.935, 0.900, 1.0))
    ebony = model.material("key_ebony", rgba=(0.060, 0.060, 0.070, 1.0))
    brass = model.material("brass", rgba=(0.815, 0.625, 0.205, 1.0))

    body = model.part("body")

    # --- Cabinet shell (back + two sides) ------------------------------------
    body.visual(
        Box((2 * HALF_W, 0.045, SHELL_TOP_Z - SHELL_BOTTOM_Z)),
        origin=Origin(xyz=(0.0, DEPTH_BACK_Y - 0.0225, (SHELL_TOP_Z + SHELL_BOTTOM_Z) / 2.0)),
        material=black,
        name="back_panel",
    )
    for side, sx in (("l", -(HALF_W - SIDE_THK / 2.0)), ("r", HALF_W - SIDE_THK / 2.0)):
        body.visual(
            Box((SIDE_THK, DEPTH_BACK_Y, SHELL_TOP_Z - SHELL_BOTTOM_Z)),
            origin=Origin(xyz=(sx, DEPTH_BACK_Y / 2.0, (SHELL_TOP_Z + SHELL_BOTTOM_Z) / 2.0)),
            material=black,
            name=f"side_panel_{side}",
        )

    # --- Base plinth / toe board ---------------------------------------------
    body.visual(
        Box((2 * HALF_W, DEPTH_BACK_Y, 0.060)),
        origin=Origin(xyz=(0.0, DEPTH_BACK_Y / 2.0, 0.030)),
        material=black,
        name="plinth",
    )

    # --- Upper front panel (large board above the keyboard) ------------------
    body.visual(
        Box((1.37, 0.050, 0.340)),
        origin=Origin(xyz=(0.0, 0.005, 0.970)),
        material=black,
        name="upper_front_panel",
    )
    # decorative top rail above the upper panel
    body.visual(
        Box((1.40, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, 0.005, 1.158)),
        material=black,
        name="top_front_rail",
    )

    # --- Lower front board (kickboard) ---------------------------------------
    body.visual(
        Box((1.37, 0.050, 0.560)),
        origin=Origin(xyz=(0.0, 0.005, 0.340)),
        material=black,
        name="lower_front_board",
    )

    # --- Keyboard shelf (protrudes forward; keys rest on top) ----------------
    body.visual(
        Box((1.32, 0.340, 0.076)),
        origin=Origin(xyz=(0.0, 0.030, KEYBED_TOP_Z - 0.038)),
        material=black_satin,
        name="key_shelf",
    )
    # key blocks / cheeks framing the keyboard
    for side, sx in (("l", -0.665), ("r", 0.665)):
        body.visual(
            Box((0.050, 0.255, 0.180)),
            origin=Origin(xyz=(sx, -0.020, 0.710)),
            material=black,
            name=f"key_block_{side}",
        )
    # ledge behind the keys that the open fallboard lies on
    body.visual(
        Box((1.330, 0.105, 0.040)),
        origin=Origin(xyz=(0.0, 0.0675, LEDGE_TOP_Z - 0.020)),
        material=black,
        name="fallboard_ledge",
    )
    # nameboard strip below the upper panel (above the resting fallboard)
    body.visual(
        Box((1.32, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.018, 0.785)),
        material=black,
        name="nameboard",
    )

    # --- Pedal mount block ----------------------------------------------------
    body.visual(
        Box((0.230, 0.090, 0.110)),
        origin=Origin(xyz=(0.0, -0.020, 0.120)),
        material=black,
        name="pedal_box",
    )

    # =======================================================================
    # Top lid — hinged at the rear, lifts the front edge upward.
    # =======================================================================
    lid_hinge_y = DEPTH_BACK_Y
    lid_hinge_z = SHELL_TOP_Z + LID_THK / 2.0
    lid = model.part("top_lid")
    lid.visual(
        Box((2 * HALF_W, DEPTH_BACK_Y, LID_THK)),
        origin=Origin(xyz=(0.0, -DEPTH_BACK_Y / 2.0, 0.0)),
        material=black,
        name="lid_panel",
    )
    model.articulation(
        "body_to_top_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, lid_hinge_y, lid_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.5, lower=0.0, upper=1.2),
    )

    # =======================================================================
    # Fallboard (key cover) — retracted at rest (keys exposed), then slides
    # straight forward from the ledge to cover the keys.
    # =======================================================================
    fallboard = model.part("fallboard")
    fallboard.visual(
        Box((1.26, 0.095, 0.020)),
        origin=Origin(xyz=(0.0, 0.0475, 0.010)),
        material=black,
        name="fallboard_panel",
    )
    model.articulation(
        "body_to_fallboard",
        ArticulationType.PRISMATIC,
        parent=body,
        child=fallboard,
        origin=Origin(xyz=(0.0, 0.015, LEDGE_TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.25, lower=0.0, upper=0.12),
    )

    # =======================================================================
    # Keyboard — 52 white keys + black keys, each pressing down (prismatic).
    # =======================================================================
    white_centers = [(i - (N_WHITE - 1) / 2.0) * WHITE_PITCH for i in range(N_WHITE)]
    white_key_geom = Box((WHITE_KEY_W, WHITE_KEY_DEPTH, WHITE_KEY_H))
    white_y = KEYBOARD_FRONT_Y + WHITE_KEY_DEPTH / 2.0
    white_z = KEYBED_TOP_Z + WHITE_KEY_H / 2.0
    for i, cx in enumerate(white_centers):
        wk = model.part(f"white_key_{i}")
        wk.visual(white_key_geom, origin=Origin(), material=ivory, name="white_key")
        model.articulation(
            f"body_to_white_key_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=wk,
            origin=Origin(xyz=(cx, white_y, white_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.1, lower=0.0, upper=0.009),
        )

    black_after = {0, 1, 3, 4, 5}
    black_depth = 0.090
    black_key_geom = Box((0.011, black_depth, BLACK_KEY_H))
    black_back_y = white_y + WHITE_KEY_DEPTH / 2.0
    black_y = black_back_y - black_depth / 2.0
    black_z = WHITE_KEY_TOP_Z + BLACK_KEY_H / 2.0
    b_index = 0
    for i in range(N_WHITE - 1):
        if (i % 7) not in black_after:
            continue
        bx = (white_centers[i] + white_centers[i + 1]) / 2.0
        bk = model.part(f"black_key_{b_index}")
        bk.visual(black_key_geom, origin=Origin(), material=ebony, name="black_key")
        model.articulation(
            f"body_to_black_key_{b_index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=bk,
            origin=Origin(xyz=(bx, black_y, black_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.6, velocity=0.1, lower=0.0, upper=0.007),
        )
        b_index += 1

    # =======================================================================
    # Three brass pedals on the pedal box (revolute, press front edge down).
    # =======================================================================
    pedal_depth = 0.110
    pedal_geom = Box((0.030, pedal_depth, 0.012))
    pedal_box_front_y = -0.020 - 0.090 / 2.0  # front face of the pedal box
    for pi, px in enumerate((-0.055, 0.0, 0.055)):
        pedal = model.part(f"pedal_{pi}")
        pedal.visual(
            pedal_geom,
            origin=Origin(xyz=(0.0, -pedal_depth / 2.0, 0.0)),
            material=brass,
            name="pedal",
        )
        model.articulation(
            f"body_to_pedal_{pi}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=pedal,
            origin=Origin(xyz=(px, pedal_box_front_y, 0.120)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=0.18),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("top_lid")
    fallboard = object_model.get_part("fallboard")

    ctx.check(
        "has upper front panel",
        body.get_visual("upper_front_panel") is not None,
        details="missing upper panel",
    )

    # Top lid lifts upward when opened.
    lid_joint = object_model.get_articulation("body_to_top_lid")
    rest_lid = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.1}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "top lid opens upward",
        rest_lid is not None
        and open_lid is not None
        and open_lid[1][2] > rest_lid[1][2] + 0.15,
        details=f"rest={rest_lid}, open={open_lid}",
    )

    # A white key presses straight down.
    wk_joint = object_model.get_articulation("body_to_white_key_25")
    wk = object_model.get_part("white_key_25")
    rest_k = ctx.part_world_position(wk)
    with ctx.pose({wk_joint: 0.009}):
        down_k = ctx.part_world_position(wk)
    ctx.check(
        "white key travels down",
        rest_k is not None and down_k is not None and down_k[2] < rest_k[2] - 0.004,
        details=f"rest={rest_k}, down={down_k}",
    )

    # Fallboard slides forward over the keys.
    fb_joint = object_model.get_articulation("body_to_fallboard")
    rest_fb = ctx.part_world_aabb(fallboard)
    with ctx.pose({fb_joint: 0.12}):
        closed_fb = ctx.part_world_aabb(fallboard)
    ctx.check(
        "fallboard slides forward over keys",
        rest_fb is not None
        and closed_fb is not None
        and closed_fb[0][1] < rest_fb[0][1] - 0.10
        and closed_fb[1][1] < rest_fb[1][1] - 0.10,
        details=f"rest={rest_fb}, closed={closed_fb}",
    )
    ctx.check(
        "fallboard stays level above keyboard instead of rotating into the case",
        closed_fb is not None
        and closed_fb[0][2] > WHITE_KEY_TOP_Z
        and rest_fb is not None
        and abs(closed_fb[0][2] - rest_fb[0][2]) < 1e-6
        and abs(closed_fb[1][2] - rest_fb[1][2]) < 1e-6,
        details=f"white_key_top={WHITE_KEY_TOP_Z}, rest={rest_fb}, closed={closed_fb}",
    )

    # A pedal depresses at the front.
    ped_joint = object_model.get_articulation("body_to_pedal_1")
    pedal = object_model.get_part("pedal_1")
    rest_p = ctx.part_world_aabb(pedal)
    with ctx.pose({ped_joint: 0.18}):
        press_p = ctx.part_world_aabb(pedal)
    ctx.check(
        "pedal depresses",
        rest_p is not None
        and press_p is not None
        and press_p[0][2] < rest_p[0][2] - 0.005,
        details=f"rest={rest_p}, press={press_p}",
    )

    return ctx.report()


object_model = build_object_model()
