from __future__ import annotations

# Glossy black grand piano with a curved wing-shaped rim, a large top lid that
# opens on a lid stick to reveal the cast-iron plate and strings, a full
# black-and-white keyboard, a fold-down fallboard, three brass pedals on a
# central lyre, and three tapered legs on brass casters.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    PianoHingeGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)

# ---------------------------------------------------------------------------
# Footprint / height constants (meters, floor at z=0).
# ---------------------------------------------------------------------------
HALF_W = 0.66          # half body width
CAVITY_FRONT_Y = 0.12  # front of the curved rim cavity
TAIL_Y = 1.86          # tail tip (back)

RIM_BOTTOM_Z = 0.745
RIM_TOP_Z = 1.000
LID_THK = 0.020

KEY_BED_TOP_Z = 0.695
WHITE_KEY_H = 0.024
WHITE_KEY_TOP_Z = KEY_BED_TOP_Z + WHITE_KEY_H  # 0.719
BLACK_KEY_H = 0.013

KEYBOARD_FRONT_Y = -0.165
N_WHITE = 52
WHITE_PITCH = 0.0234
WHITE_KEY_W = 0.0218
WHITE_KEY_DEPTH = 0.150

PLATE_TOP_Z = 0.860
STRING_Z = 0.884

SPLIT_Y = 1.00  # Y where the split lid divides front / rear leaves


def _piano_outline() -> list[tuple[float, float]]:
    """Counter-clockwise top-view outline of the grand piano case."""
    pts: list[tuple[float, float]] = []
    pts.append((-HALF_W, CAVITY_FRONT_Y))  # front-left
    pts.append((HALF_W, CAVITY_FRONT_Y))   # front-right
    pts.append((HALF_W, 0.62))             # right cheek, straight section
    bentside = sample_catmull_rom_spline_2d(
        [
            (HALF_W, 0.62),
            (0.625, 1.05),
            (0.46, 1.45),
            (0.18, 1.74),
            (-0.06, TAIL_Y),
        ],
        samples_per_segment=7,
    )
    pts.extend(bentside[1:])               # bentside curve to tail tip
    pts.append((-HALF_W, 1.52))            # tail back-left -> spine
    return pts


def _centroid(profile: list[tuple[float, float]]) -> tuple[float, float]:
    cx = sum(p[0] for p in profile) / len(profile)
    cy = sum(p[1] for p in profile) / len(profile)
    return cx, cy


def _scale_profile(
    profile: list[tuple[float, float]], factor: float
) -> list[tuple[float, float]]:
    cx, cy = _centroid(profile)
    return [(cx + (x - cx) * factor, cy + (y - cy) * factor) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="grand_piano")

    black = model.material("black_gloss", rgba=(0.045, 0.045, 0.055, 1.0))
    black_satin = model.material("black_satin", rgba=(0.075, 0.075, 0.085, 1.0))
    ivory = model.material("key_ivory", rgba=(0.945, 0.935, 0.900, 1.0))
    ebony = model.material("key_ebony", rgba=(0.060, 0.060, 0.070, 1.0))
    brass = model.material("brass", rgba=(0.815, 0.625, 0.205, 1.0))
    iron_gold = model.material("iron_plate_gold", rgba=(0.720, 0.555, 0.230, 1.0))
    wood_board = model.material("soundboard_wood", rgba=(0.760, 0.560, 0.330, 1.0))
    string_steel = model.material("string_steel", rgba=(0.780, 0.785, 0.800, 1.0))
    string_copper = model.material("string_copper", rgba=(0.730, 0.450, 0.225, 1.0))
    felt_red = model.material("felt_red", rgba=(0.520, 0.090, 0.090, 1.0))

    outline = _piano_outline()
    inner = _scale_profile(outline, 0.90)
    inner_rev = list(reversed(inner))

    case = model.part("case")

    # --- Curved hollow rim wall ---------------------------------------------
    rim_geom = ExtrudeWithHolesGeometry(
        outline,
        [inner_rev],
        RIM_TOP_Z - RIM_BOTTOM_Z,
        cap=True,
        center=True,
    )
    rim_mid_z = (RIM_TOP_Z + RIM_BOTTOM_Z) / 2.0
    case.visual(
        mesh_from_geometry(rim_geom, "case_rim"),
        origin=Origin(xyz=(0.0, 0.0, rim_mid_z)),
        material=black,
        name="rim_wall",
    )

    # --- Bottom board (closes the underside of the rim) ----------------------
    bottom_geom = ExtrudeGeometry(outline, 0.014, cap=True, center=True)
    case.visual(
        mesh_from_geometry(bottom_geom, "case_bottom"),
        origin=Origin(xyz=(0.0, 0.0, RIM_BOTTOM_Z + 0.007)),
        material=black_satin,
        name="bottom_board",
    )

    # --- Soundboard (light wood floor of the cavity) -------------------------
    sb_geom = ExtrudeGeometry(_scale_profile(outline, 0.93), 0.012, cap=True, center=True)
    case.visual(
        mesh_from_geometry(sb_geom, "case_soundboard"),
        origin=Origin(xyz=(0.0, 0.0, 0.792)),
        material=wood_board,
        name="soundboard",
    )

    # --- Cast-iron plate (gold harp) -----------------------------------------
    plate_geom = ExtrudeGeometry(_scale_profile(outline, 0.92), 0.018, cap=True, center=True)
    case.visual(
        mesh_from_geometry(plate_geom, "case_plate"),
        origin=Origin(xyz=(0.0, 0.0, PLATE_TOP_Z - 0.009)),
        material=iron_gold,
        name="cast_iron_plate",
    )

    # --- Pinblock (front) and hitch-pin rail (back) carry the strings --------
    case.visual(
        Box((1.18, 0.075, 0.060)),
        origin=Origin(xyz=(0.0, 0.205, 0.875)),
        material=iron_gold,
        name="pinblock",
    )
    case.visual(
        Box((0.62, 0.060, 0.055)),
        origin=Origin(xyz=(0.0, 1.46, 0.875)),
        material=iron_gold,
        name="hitch_rail",
    )

    # --- Strings (steel treble + copper-wound bass) --------------------------
    n_strings = 26
    for s in range(n_strings):
        t = s / (n_strings - 1)
        x = -0.52 + t * 1.04
        # bass strings (left) longer + thicker copper, treble shorter steel
        front_y = 0.20
        back_y = 1.44 - 0.32 * t
        length = back_y - front_y
        cy = (front_y + back_y) / 2.0
        is_bass = t < 0.28
        radius = 0.0028 if is_bass else 0.0017
        mat = string_copper if is_bass else string_steel
        case.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, cy, STRING_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mat,
            name=f"string_{s}",
        )

    # --- Keybed and keyboard surround ---------------------------------------
    case.visual(
        Box((1.30, 0.215, 0.045)),
        origin=Origin(xyz=(0.0, -0.07, KEY_BED_TOP_Z - 0.0225)),
        material=black_satin,
        name="keybed",
    )
    # key blocks / cheeks on both sides of the keyboard
    for side, sx in (("l", -0.635), ("r", 0.635)):
        case.visual(
            Box((0.050, 0.30, 0.135)),
            origin=Origin(xyz=(sx, -0.03, 0.7275)),
            material=black,
            name=f"key_block_{side}",
        )
    # nameboard behind the keys
    case.visual(
        Box((1.27, 0.030, 0.110)),
        origin=Origin(xyz=(0.0, 0.012, 0.745)),
        material=black,
        name="nameboard",
    )
    # front shelf connecting nameboard region to the rim front
    case.visual(
        Box((1.32, 0.12, 0.022)),
        origin=Origin(xyz=(0.0, 0.060, 0.789)),
        material=black,
        name="front_shelf",
    )
    # keyslip strip in front of the keys
    case.visual(
        Box((1.27, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, KEYBOARD_FRONT_Y - 0.018, 0.700)),
        material=black,
        name="keyslip",
    )

    # --- Legs (tapered) on brass casters ------------------------------------
    leg_positions = [(-0.585, 0.07), (0.585, 0.07), (-0.055, 1.52)]
    for li, (lx, ly) in enumerate(leg_positions):
        case.visual(
            Cylinder(radius=0.046, length=RIM_BOTTOM_Z - 0.06),
            origin=Origin(xyz=(lx, ly, (RIM_BOTTOM_Z - 0.06) / 2.0 + 0.06)),
            material=black,
            name=f"leg_{li}",
        )
        case.visual(
            Cylinder(radius=0.052, length=0.030),
            origin=Origin(xyz=(lx, ly, 0.745 - 0.010)),
            material=black,
            name=f"leg_collar_{li}",
        )
        case.visual(
            Cylinder(radius=0.034, length=0.060),
            origin=Origin(xyz=(lx, ly, 0.030)),
            material=brass,
            name=f"caster_{li}",
        )

    # --- Lyre + pedal box (hangs below the keybed front center) -------------
    # Posts run from inside the keybed (top) down into the pedal box (bottom).
    for side, sx in (("l", -0.060), ("r", 0.060)):
        case.visual(
            Box((0.038, 0.038, 0.470)),
            origin=Origin(xyz=(sx, -0.060, 0.450), rpy=(0.14, 0.0, 0.0)),
            material=black,
            name=f"lyre_post_{side}",
        )
    case.visual(
        Box((0.230, 0.105, 0.075)),
        origin=Origin(xyz=(0.0, -0.090, 0.205)),
        material=black,
        name="pedal_box",
    )

    # --- Music desk (rack standing in front of the strings, behind fallboard)
    case.visual(
        Box((0.86, 0.012, 0.185)),
        origin=Origin(xyz=(0.0, 0.150, 0.900), rpy=(-0.24, 0.0, 0.0)),
        material=black,
        name="music_desk_panel",
    )
    case.visual(
        Box((0.86, 0.040, 0.014)),
        origin=Origin(xyz=(0.0, 0.140, 0.808)),
        material=black,
        name="music_desk_base",
    )

    # =======================================================================
    # Two-leaf split top lid — each leaf hinged on the spine, opens upward.
    # =======================================================================
    hinge_x = -HALF_W
    hinge_z = RIM_TOP_Z + LID_THK / 2.0

    # Recompute bentside spline to find the split intersection on the curve.
    bentside = sample_catmull_rom_spline_2d(
        [
            (HALF_W, 0.62),
            (0.625, 1.05),
            (0.46, 1.45),
            (0.18, 1.74),
            (-0.06, TAIL_Y),
        ],
        samples_per_segment=7,
    )
    split_x = HALF_W
    split_idx = 0
    for i in range(len(bentside) - 1):
        y0, y1 = bentside[i][1], bentside[i + 1][1]
        if y0 <= SPLIT_Y <= y1:
            t = (SPLIT_Y - y0) / (y1 - y0) if y1 != y0 else 0.0
            split_x = bentside[i][0] + t * (bentside[i + 1][0] - bentside[i][0])
            split_idx = i
            break

    # --- Front leaf outline (keyboard side to split line) --------------------
    front_outline: list[tuple[float, float]] = [
        (-HALF_W, CAVITY_FRONT_Y),
        (HALF_W, CAVITY_FRONT_Y),
        (HALF_W, 0.62),
    ]
    front_outline.extend(bentside[1 : split_idx + 1])
    front_outline.append((split_x, SPLIT_Y))
    front_outline.append((-HALF_W, SPLIT_Y))

    # --- Rear leaf outline (split line through bentside to tail) -------------
    rear_outline: list[tuple[float, float]] = [
        (-HALF_W, SPLIT_Y),
        (split_x, SPLIT_Y),
    ]
    rear_outline.extend(bentside[split_idx + 1 :])
    rear_outline.append((-HALF_W, 1.52))

    # Build each leaf as a separate part with its own spine hinge.
    lid_leafs: list[tuple] = []
    for leaf_i, (leaf_outline, leaf_name, hinge_y_leaf, hinge_len) in enumerate(
        [
            (
                front_outline,
                "lid_front",
                (CAVITY_FRONT_Y + SPLIT_Y) / 2.0,
                SPLIT_Y - CAVITY_FRONT_Y,
            ),
            (
                rear_outline,
                "lid_rear",
                (SPLIT_Y + 1.52) / 2.0,
                1.52 - SPLIT_Y,
            ),
        ]
    ):
        leaf_local = [(x - hinge_x, y - hinge_y_leaf) for x, y in leaf_outline]
        leaf_geom = ExtrudeGeometry(leaf_local, LID_THK, cap=True, center=True)
        leaf_part = model.part(leaf_name)
        leaf_part.visual(
            mesh_from_geometry(leaf_geom, f"{leaf_name}_panel"),
            origin=Origin(),
            material=black,
            name=f"{leaf_name}_panel",
        )
        # Visible continuous hinge strip along the spine edge of each leaf.
        hinge_strip_geom = PianoHingeGeometry(
            hinge_len,
            leaf_width_a=0.018,
            leaf_width_b=0.016,
            leaf_thickness=0.0020,
            pin_diameter=0.003,
            knuckle_pitch=0.014,
        )
        leaf_part.visual(
            mesh_from_geometry(hinge_strip_geom, f"{leaf_name}_hinge_strip"),
            origin=Origin(
                xyz=(0.0, 0.0, -LID_THK / 2.0 - 0.002),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=f"{leaf_name}_hinge_strip",
        )
        model.articulation(
            f"case_to_{leaf_name}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=leaf_part,
            origin=Origin(xyz=(hinge_x, hinge_y_leaf, hinge_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=60.0, velocity=1.5, lower=0.0, upper=0.95
            ),
        )
        lid_leafs.append((leaf_part, leaf_name, hinge_y_leaf))

    # =======================================================================
    # Fallboard (key cover) — hinged behind keys, folded open at rest so the
    # keyboard is exposed; it lies flat on the front shelf.
    # =======================================================================
    fallboard = model.part("fallboard")
    fallboard.visual(
        Box((1.27, 0.090, 0.020)),
        # Panel extends back over the shelf; bottom sits on the shelf top (0.800).
        origin=Origin(xyz=(0.0, 0.047, 0.010)),
        material=black,
        name="fallboard_panel",
    )
    model.articulation(
        "case_to_fallboard",
        ArticulationType.REVOLUTE,
        parent=case,
        child=fallboard,
        # Hinge at the front edge of the shelf; q=0 lies flat (keys exposed),
        # positive q swings it up and forward over the keyboard.
        origin=Origin(xyz=(0.0, 0.002, 0.800)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=2.4),
    )

    # =======================================================================
    # Keyboard — 52 white keys + black keys, each pressing down (prismatic).
    # =======================================================================
    white_centers = [(i - (N_WHITE - 1) / 2.0) * WHITE_PITCH for i in range(N_WHITE)]
    white_key_geom = Box((WHITE_KEY_W, WHITE_KEY_DEPTH, WHITE_KEY_H))
    white_y = KEYBOARD_FRONT_Y + WHITE_KEY_DEPTH / 2.0  # front-aligned keys
    # Key bottom rests exactly on the keybed top so each key is supported.
    white_z = KEY_BED_TOP_Z + WHITE_KEY_H / 2.0
    for i, cx in enumerate(white_centers):
        wk = model.part(f"white_key_{i}")
        wk.visual(white_key_geom, origin=Origin(), material=ivory, name="white_key")
        model.articulation(
            f"case_to_white_key_{i}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=wk,
            origin=Origin(xyz=(cx, white_y, white_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.1, lower=0.0, upper=0.009),
        )

    # Black keys sit in the gaps following the repeating 7-white pattern.
    # Each black key rests on the white-key top plane (it overhangs the
    # adjacent white keys), so its bottom is exactly the white key top.
    black_after = {0, 1, 3, 4, 5}
    black_depth = 0.090
    black_key_geom = Box((0.011, black_depth, BLACK_KEY_H))
    black_back_y = white_y + WHITE_KEY_DEPTH / 2.0  # align with white key rear
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
            f"case_to_black_key_{b_index}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=bk,
            origin=Origin(xyz=(bx, black_y, black_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.6, velocity=0.1, lower=0.0, upper=0.007),
        )
        b_index += 1

    # =======================================================================
    # Three brass pedals on the pedal box (revolute, press front edge down).
    # =======================================================================
    pedal_depth = 0.115
    pedal_geom = Box((0.034, pedal_depth, 0.012))
    pedal_box_front_y = -0.090 - 0.105 / 2.0  # front face of the pedal box
    for pi, px in enumerate((-0.052, 0.0, 0.052)):
        pedal = model.part(f"pedal_{pi}")
        # Pedal protrudes forward (-Y) from the box front; back edge touches it.
        pedal.visual(
            pedal_geom,
            origin=Origin(xyz=(0.0, -pedal_depth / 2.0, 0.0)),
            material=brass,
            name="pedal",
        )
        model.articulation(
            f"case_to_pedal_{pi}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=pedal,
            origin=Origin(xyz=(px, pedal_box_front_y, 0.200)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=0.18),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    lid_front = object_model.get_part("lid_front")
    lid_rear = object_model.get_part("lid_rear")
    lid_front_joint = object_model.get_articulation("case_to_lid_front")
    lid_rear_joint = object_model.get_articulation("case_to_lid_rear")
    fallboard = object_model.get_part("fallboard")

    # Hero geometry exists.
    ctx.check(
        "has cast iron plate",
        case.get_visual("cast_iron_plate") is not None,
        details="missing plate",
    )
    ctx.check(
        "has strings",
        case.get_visual("string_0") is not None,
        details="missing strings",
    )

    # Two-leaf split lid: both leaves exist with visible hinge strips.
    ctx.check(
        "has front lid leaf",
        lid_front is not None and lid_front.get_visual("lid_front_panel") is not None,
        details="missing front lid leaf",
    )
    ctx.check(
        "has rear lid leaf",
        lid_rear is not None and lid_rear.get_visual("lid_rear_panel") is not None,
        details="missing rear lid leaf",
    )
    ctx.check(
        "has front lid hinge strip",
        lid_front.get_visual("lid_front_hinge_strip") is not None,
        details="missing front hinge strip",
    )
    ctx.check(
        "has rear lid hinge strip",
        lid_rear.get_visual("lid_rear_hinge_strip") is not None,
        details="missing rear hinge strip",
    )

    # Closed front leaf sits above the strings (does not crush the interior).
    with ctx.pose({lid_front_joint: 0.0}):
        ctx.expect_gap(
            lid_front,
            case,
            axis="z",
            min_gap=0.0,
            positive_elem="lid_front_panel",
            negative_elem="string_10",
            name="closed front leaf clears the strings",
        )

    # Opening the front leaf lifts the free edge upward.
    rest_front_aabb = ctx.part_world_aabb(lid_front)
    with ctx.pose({lid_front_joint: 0.9}):
        open_front_aabb = ctx.part_world_aabb(lid_front)
    ctx.check(
        "front lid leaf opens upward",
        rest_front_aabb is not None
        and open_front_aabb is not None
        and open_front_aabb[1][2] > rest_front_aabb[1][2] + 0.2,
        details=f"rest_top={rest_front_aabb}, open_top={open_front_aabb}",
    )

    # Opening the rear leaf lifts the free edge upward.
    rest_rear_aabb = ctx.part_world_aabb(lid_rear)
    with ctx.pose({lid_rear_joint: 0.9}):
        open_rear_aabb = ctx.part_world_aabb(lid_rear)
    ctx.check(
        "rear lid leaf opens upward",
        rest_rear_aabb is not None
        and open_rear_aabb is not None
        and open_rear_aabb[1][2] > rest_rear_aabb[1][2] + 0.2,
        details=f"rest_top={rest_rear_aabb}, open_top={open_rear_aabb}",
    )

    # A white key presses straight down when actuated.
    wk_joint = object_model.get_articulation("case_to_white_key_20")
    wk = object_model.get_part("white_key_20")
    rest_k = ctx.part_world_position(wk)
    with ctx.pose({wk_joint: 0.009}):
        down_k = ctx.part_world_position(wk)
    ctx.check(
        "white key travels down",
        rest_k is not None and down_k is not None and down_k[2] < rest_k[2] - 0.004,
        details=f"rest={rest_k}, down={down_k}",
    )

    # The fallboard rotates forward to cover the keyboard.
    fb_joint = object_model.get_articulation("case_to_fallboard")
    rest_fb = ctx.part_world_aabb(fallboard)
    with ctx.pose({fb_joint: 2.4}):
        closed_fb = ctx.part_world_aabb(fallboard)
    ctx.check(
        "fallboard folds forward over keys",
        rest_fb is not None
        and closed_fb is not None
        and closed_fb[0][1] < rest_fb[0][1] - 0.05,
        details=f"rest={rest_fb}, closed={closed_fb}",
    )
    ctx.check(
        "fallboard closes above keyboard instead of into the case",
        closed_fb is not None
        and closed_fb[0][2] > WHITE_KEY_TOP_Z
        and rest_fb is not None
        and closed_fb[1][2] > rest_fb[1][2] + 0.035,
        details=f"white_key_top={WHITE_KEY_TOP_Z}, rest={rest_fb}, closed={closed_fb}",
    )

    # A pedal depresses at the front.
    ped_joint = object_model.get_articulation("case_to_pedal_1")
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
