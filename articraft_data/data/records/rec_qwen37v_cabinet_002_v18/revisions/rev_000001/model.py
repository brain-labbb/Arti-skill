from __future__ import annotations

# Corner cabinet with angled front doors (~0.90 m W x 0.42 m D carcass x 0.85 m H).
#
# World: front faces +X (back at x=0, carcass front at x=BD), width along Y
# (centered), height along +Z, grounded at z=0 on four square legs ~0.15 m.
# Matte black wood carcass and doors; smooth silver-gray top overhangs.
#
# Front has two angled doors forming a shallow V that protrudes forward from
# the carcass face. Each door swings outward on a vertical revolute hinge at
# its outer edge. Interior holds two shelves in the main door compartment and
# one shelf in the open upper cubby (visible through the front gap).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

# ---------- key dimensions (meters) ----------
BW = 0.90                    # body width (Y)
BD = 0.42                    # body depth (X), back at x=0
WALL = 0.018                 # carcass panel thickness
LEG_H = 0.150
BODY_BOT = LEG_H
MAIN_TOP_Z = 0.62            # divider between door compartment and open cubby
OPEN_TOP_Z = 0.828           # top of open cubby (bottom of top slab)
TOP_THK = 0.022
H_TOTAL = OPEN_TOP_Z + TOP_THK  # ~0.850

BH_MAIN = MAIN_TOP_Z - BODY_BOT    # main door compartment height (~0.47)
BH_OPEN = OPEN_TOP_Z - MAIN_TOP_Z  # open cubby height (~0.208)

# Front V-shape: doors angle forward from carcass front to a center meeting point.
V_DEPTH = 0.14               # how far the V protrudes past the carcass front
FRONT_X = BD                 # carcass front plane
CENTER_X = BD + V_DEPTH      # where the two doors meet (0.56)

HINGE_Y = BW / 2.0           # hinge at outer face of side panels (±0.45)

# Front frame stile and rail dimensions.
STILE_W = 0.028
RAIL_H = 0.028

# Door panel geometry.
door_dx = V_DEPTH
door_dy = HINGE_Y
DOOR_LEN = math.sqrt(door_dx ** 2 + door_dy ** 2)  # ~0.471
DOOR_THK = 0.020
DOOR_H = BH_MAIN - 2 * RAIL_H - 0.012  # fits within the rail opening with reveals

# Rotation from local +X to the door panel direction in world XY.
LEFT_ANGLE = math.atan2(-door_dy, door_dx)   # ~-1.27 rad
RIGHT_ANGLE = math.atan2(door_dy, door_dx)   # ~+1.27 rad

# Knob: polished silver ball on a short stem (same as parent dresser).
KNOB_R = 0.012
STEM_R = 0.005
STEM_L = 0.014
STEM_TOTAL = STEM_L + 0.004  # total stem cylinder length
EMBED = 0.004                # stem embedment into door panel

# Interior shelves.
SHELF_THK = 0.016
INNER_W = BW - 2 * WALL      # inner width between side panels
INNER_D = BD - WALL           # inner depth from back panel to front

# Legs.
LEG_SQ = 0.045

# Top slab.
OVERHANG = 0.020

# Derived positions.
DOOR_MID_Z = BODY_BOT + BH_MAIN / 2.0

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_cabinet_angled_doors")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    shelf_mat = model.material("shelf_black", rgba=(0.10, 0.10, 0.11, 1.0))

    # =================================================================
    # ROOT: carcass (shell + legs + front frame + shelves + top slab)
    # =================================================================
    carcass = model.part("carcass")
    full_h = OPEN_TOP_Z - BODY_BOT  # full carcass height

    # Back panel.
    carcass.visual(
        Box((WALL, BW, full_h)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT + full_h / 2.0)),
        material=black, name="back_panel",
    )

    # Side panels.
    for tag, s in (("left", 1), ("right", -1)):
        carcass.visual(
            Box((BD, WALL, full_h)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                                BODY_BOT + full_h / 2.0)),
            material=black, name=f"side_panel_{tag}",
        )

    # Bottom board.
    carcass.visual(
        Box((BD - WALL, INNER_W, WALL)),
        origin=Origin(xyz=(WALL + (BD - WALL) / 2.0, 0.0, BODY_BOT + WALL / 2.0)),
        material=black, name="bottom_board",
    )

    # Mid divider board (ceiling of main compartment / floor of open cubby).
    carcass.visual(
        Box((BD - WALL, INNER_W, WALL)),
        origin=Origin(xyz=(WALL + (BD - WALL) / 2.0, 0.0, MAIN_TOP_Z + WALL / 2.0)),
        material=black, name="mid_divider",
    )

    # Front frame stiles (vertical strips flanking the door opening).
    stile_h = BH_MAIN
    for tag, s in (("left", 1), ("right", -1)):
        carcass.visual(
            Box((WALL, STILE_W, stile_h)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                                s * (BW / 2.0 - WALL - STILE_W / 2.0),
                                BODY_BOT + stile_h / 2.0)),
            material=black, name=f"front_stile_{tag}",
        )

    # Front frame bottom rail.
    rail_span = INNER_W - 2 * STILE_W
    carcass.visual(
        Box((WALL, rail_span, RAIL_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, BODY_BOT + RAIL_H / 2.0)),
        material=black, name="front_bottom_rail",
    )

    # Front frame top rail (at top of door opening, bottom of cubby).
    carcass.visual(
        Box((WALL, rail_span, RAIL_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, MAIN_TOP_Z - RAIL_H / 2.0)),
        material=black, name="front_top_rail",
    )

    # Interior shelves in the main door compartment (2 shelves).
    shelf_inner_d = INNER_D - 0.020
    shelf_inner_w = INNER_W - 0.010
    for i in range(2):
        shelf_z = BODY_BOT + WALL + (i + 1) * (BH_MAIN - WALL - 0.010) / 3.0
        carcass.visual(
            Box((shelf_inner_d, shelf_inner_w, SHELF_THK)),
            origin=Origin(xyz=(WALL + shelf_inner_d / 2.0, 0.0, shelf_z)),
            material=shelf_mat, name=f"main_shelf_{i}",
        )

    # Open cubby shelf (visible through the open front gap above the doors).
    open_shelf_z = MAIN_TOP_Z + WALL + BH_OPEN * 0.45
    carcass.visual(
        Box((shelf_inner_d, shelf_inner_w, SHELF_THK)),
        origin=Origin(xyz=(WALL + shelf_inner_d / 2.0, 0.0, open_shelf_z)),
        material=shelf_mat, name="open_shelf",
    )

    # Silver-gray top slab with overhang on all sides.
    top_depth = CENTER_X + 2 * OVERHANG
    top_width = BW + 2 * OVERHANG
    top_cx = CENTER_X / 2.0  # centered on full depth including V protrusion
    carcass.visual(
        Box((top_depth, top_width, TOP_THK)),
        origin=Origin(xyz=(top_cx, 0.0, OPEN_TOP_Z + TOP_THK / 2.0)),
        material=silver_top, name="top_slab",
    )

    # Four square legs.
    for tag, lx, ly in (("front_left", BD - LEG_SQ / 2.0, HINGE_Y - LEG_SQ / 2.0),
                          ("front_right", BD - LEG_SQ / 2.0, -(HINGE_Y - LEG_SQ / 2.0)),
                          ("rear_left", LEG_SQ / 2.0, HINGE_Y - LEG_SQ / 2.0),
                          ("rear_right", LEG_SQ / 2.0, -(HINGE_Y - LEG_SQ / 2.0))):
        carcass.visual(
            Box((LEG_SQ, LEG_SQ, LEG_H + 0.004)),
            origin=Origin(xyz=(lx, ly, (LEG_H + 0.004) / 2.0)),
            material=black, name=f"leg_{tag}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=35.0)

    # =================================================================
    # LEFT DOOR
    # =================================================================
    left_door = model.part("left_door")
    # Door panel: hinge at local origin, panel extends along +X toward center.
    # Visual offset so inner face (y=0) is at the hinge and outer face at +Y.
    left_door.visual(
        Box((DOOR_LEN, DOOR_THK, DOOR_H)),
        origin=Origin(xyz=(DOOR_LEN / 2.0, DOOR_THK / 2.0, 0.0)),
        material=black, name="door_panel",
    )
    # Knob on the outside (+Y) face, near the free edge.
    knob_x = DOOR_LEN - 0.050
    knob_stem_cy = DOOR_THK + (STEM_TOTAL - EMBED) / 2.0
    knob_ball_cy = DOOR_THK + (STEM_TOTAL - EMBED) + KNOB_R - 0.002
    left_door.visual(
        Cylinder(radius=STEM_R, length=STEM_TOTAL),
        origin=Origin(xyz=(knob_x, knob_stem_cy, 0.0),
                       rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver, name="knob_stem",
    )
    left_door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(knob_x, knob_ball_cy, 0.0)),
        material=silver, name="knob_ball",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((DOOR_LEN, DOOR_THK, DOOR_H)), mass=2.5)

    # =================================================================
    # RIGHT DOOR
    # =================================================================
    right_door = model.part("right_door")
    # Mirror: inner face at y=0 (hinge), outer face at -Y.
    right_door.visual(
        Box((DOOR_LEN, DOOR_THK, DOOR_H)),
        origin=Origin(xyz=(DOOR_LEN / 2.0, -DOOR_THK / 2.0, 0.0)),
        material=black, name="door_panel",
    )
    knob_stem_cy_r = -(DOOR_THK + (STEM_TOTAL - EMBED) / 2.0)
    knob_ball_cy_r = -(DOOR_THK + (STEM_TOTAL - EMBED) + KNOB_R - 0.002)
    right_door.visual(
        Cylinder(radius=STEM_R, length=STEM_TOTAL),
        origin=Origin(xyz=(knob_x, knob_stem_cy_r, 0.0),
                       rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver, name="knob_stem",
    )
    right_door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(knob_x, knob_ball_cy_r, 0.0)),
        material=silver, name="knob_ball",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((DOOR_LEN, DOOR_THK, DOOR_H)), mass=2.5)

    # =================================================================
    # ARTICULATIONS
    # =================================================================
    # Left door hinge: vertical axis, positive q opens outward (CCW from above).
    model.articulation(
        "carcass_to_left_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(FRONT_X, HINGE_Y, DOOR_MID_Z),
                       rpy=(0.0, 0.0, LEFT_ANGLE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                    lower=0.0, upper=1.40),
    )

    # Right door hinge: vertical axis, negative direction so positive q opens
    # outward (CW from above for the right side).
    model.articulation(
        "carcass_to_right_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(FRONT_X, -HINGE_Y, DOOR_MID_Z),
                       rpy=(0.0, 0.0, RIGHT_ANGLE)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                    lower=0.0, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_hinge = object_model.get_articulation("carcass_to_left_door")
    right_hinge = object_model.get_articulation("carcass_to_right_door")

    # --- Overall scale (~0.90 W x ~0.85 H, depth includes V protrusion) ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("legs_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_~090", 0.85 < width_y < 1.00,
              details=f"w={width_y:.3f}")
    ctx.check("height_~085", abs(height_z - 0.85) < 0.02,
              details=f"h={height_z:.3f}")

    # --- Two revolute hinges, vertical axes, open outward ---
    ctx.check("left_hinge_revolute",
              left_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("right_hinge_revolute",
              right_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("left_hinge_vertical",
              abs(left_hinge.axis[2]) > 0.99
              and abs(left_hinge.axis[0]) < 0.01
              and abs(left_hinge.axis[1]) < 0.01)
    ctx.check("right_hinge_vertical",
              abs(right_hinge.axis[2]) > 0.99
              and abs(right_hinge.axis[0]) < 0.01
              and abs(right_hinge.axis[1]) < 0.01)
    ctx.check("left_hinge_range",
              abs(left_hinge.motion_limits.lower) < 1e-9
              and left_hinge.motion_limits.upper > 1.0)
    ctx.check("right_hinge_range",
              abs(right_hinge.motion_limits.lower) < 1e-9
              and right_hinge.motion_limits.upper > 1.0)

    # --- Angled doors: closed pose has doors protruding forward of carcass ---
    carcass_front_x = BD
    left_panel = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_panel = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert left_panel is not None and right_panel is not None
    ctx.check("left_door_proud_of_carcass",
              left_panel[1][0] > carcass_front_x + 0.05,
              details=f"left door max x={left_panel[1][0]:.4f}")
    ctx.check("right_door_proud_of_carcass",
              right_panel[1][0] > carcass_front_x + 0.05,
              details=f"right door max x={right_panel[1][0]:.4f}")

    # Doors meet near the center (y~0) at the V apex.
    left_center_y = (left_panel[0][1] + left_panel[1][1]) / 2.0
    right_center_y = (right_panel[0][1] + right_panel[1][1]) / 2.0
    ctx.check("doors_form_v_shape",
              left_panel[0][1] < 0.03 and right_panel[1][1] > -0.03,
              details=f"left min y={left_panel[0][1]:.3f}, right max y={right_panel[1][1]:.3f}")

    # --- Hinges at the outer edges (near the carcass side panels) ---
    left_hinge_pos = left_hinge.origin.xyz if hasattr(left_hinge.origin, 'xyz') else None
    ctx.check("left_hinge_at_outer_edge",
              abs(left_hinge.origin.xyz[1] - HINGE_Y) < 0.01
              and abs(left_hinge.origin.xyz[0] - FRONT_X) < 0.01,
              details=f"hinge y={left_hinge.origin.xyz[1]:.3f}")
    ctx.check("right_hinge_at_outer_edge",
              abs(right_hinge.origin.xyz[1] + HINGE_Y) < 0.01
              and abs(right_hinge.origin.xyz[0] - FRONT_X) < 0.01,
              details=f"hinge y={right_hinge.origin.xyz[1]:.3f}")

    # --- Open pose: doors swing outward (world Y increases for left, decreases for right) ---
    # Use element AABB centers since part origin is at the hinge (doesn't move).
    left_rest_panel = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_rest_panel = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert left_rest_panel is not None and right_rest_panel is not None
    left_rest_cy = (left_rest_panel[0][1] + left_rest_panel[1][1]) / 2.0
    right_rest_cy = (right_rest_panel[0][1] + right_rest_panel[1][1]) / 2.0

    with ctx.pose({left_hinge: left_hinge.motion_limits.upper}):
        left_open_panel = ctx.part_element_world_aabb(left_door, elem="door_panel")
    with ctx.pose({right_hinge: right_hinge.motion_limits.upper}):
        right_open_panel = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert left_open_panel is not None and right_open_panel is not None
    left_open_cy = (left_open_panel[0][1] + left_open_panel[1][1]) / 2.0
    right_open_cy = (right_open_panel[0][1] + right_open_panel[1][1]) / 2.0

    # Left door opens to the left (positive Y when facing cabinet).
    ctx.check("left_door_opens_outward",
              left_open_cy > left_rest_cy + 0.05,
              details=f"rest cy={left_rest_cy:.3f}, open cy={left_open_cy:.3f}")
    # Right door opens to the right (negative Y).
    ctx.check("right_door_opens_outward",
              right_open_cy < right_rest_cy - 0.05,
              details=f"rest cy={right_rest_cy:.3f}, open cy={right_open_cy:.3f}")

    # --- Shelf boards visible in the carcass ---
    shelf0 = ctx.part_element_world_aabb(carcass, elem="main_shelf_0")
    shelf1 = ctx.part_element_world_aabb(carcass, elem="main_shelf_1")
    open_shelf = ctx.part_element_world_aabb(carcass, elem="open_shelf")
    assert shelf0 is not None and shelf1 is not None and open_shelf is not None
    ctx.check("main_shelf_0_is_thin",
              abs((shelf0[1][2] - shelf0[0][2]) - SHELF_THK) < 0.002)
    ctx.check("main_shelves_stacked",
              shelf1[0][2] > shelf0[1][2],
              details=f"shelf0 top={shelf0[1][2]:.3f}, shelf1 bot={shelf1[0][2]:.3f}")
    ctx.check("open_shelf_in_cubby",
              open_shelf[0][2] > MAIN_TOP_Z
              and open_shelf[1][2] < OPEN_TOP_Z,
              details=f"open shelf z=({open_shelf[0][2]:.3f},{open_shelf[1][2]:.3f})")

    # --- Open cubby: the open shelf is visible through the front opening ---
    # The open shelf should be in front of the back panel and not blocked.
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert back is not None
    ctx.check("open_shelf_visible_from_front",
              open_shelf[1][0] > back[1][0] + 0.05
              and open_shelf[0][0] < FRONT_X,
              details=f"shelf x=({open_shelf[0][0]:.3f},{open_shelf[1][0]:.3f})")

    # --- Silver top slab overhangs ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side_l = ctx.part_element_world_aabb(carcass, elem="side_panel_left")
    assert top is not None and side_l is not None
    ctx.check("top_overhang_sides",
              top[1][1] > side_l[1][1] + 0.015
              and top[0][1] < -side_l[1][1] - 0.015,
              details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f})")
    ctx.check("top_is_thin_slab",
              abs((top[1][2] - top[0][2]) - TOP_THK) < 0.002)

    # --- Knobs on doors: polished silver balls proud of the outer face ---
    left_ball = ctx.part_element_world_aabb(left_door, elem="knob_ball")
    right_ball = ctx.part_element_world_aabb(right_door, elem="knob_ball")
    assert left_ball is not None and right_ball is not None
    # The knobs are on the outward face of the angled doors. Since the doors
    # protrude forward of the carcass, the knob balls should be at x > FRONT_X
    # (clearly outside the carcass shell).
    ctx.check("left_knob_outside_carcass",
              left_ball[0][0] > FRONT_X + 0.02,
              details=f"knob min x={left_ball[0][0]:.4f}")
    ctx.check("right_knob_outside_carcass",
              right_ball[0][0] > FRONT_X + 0.02,
              details=f"knob min x={right_ball[0][0]:.4f}")

    return ctx.report()


object_model = build_object_model()
