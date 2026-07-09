from __future__ import annotations

# Storage cabinet variant (~1.70 m W x 0.85 m H x 0.50 m D).
#
# World layout: front faces +X (back of the body at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four square
# legs ~0.15 m tall. Matte black wood carcass; a thin smooth silver-gray top
# slab overhangs the body ~0.02 m on all sides. Front corners carry decorative
# posts carved with a stacked spiral/faceted zigzag pattern continuing into the
# straight front legs.
#
# Front holds three sections:
#   - Left open cubby with 2 shelf boards (3 compartments)
#   - Center closed cabinet with one mirrored door on a side hinge (REVOLUTE)
#   - Right open cubby with 2 shelf boards (3 compartments)
# The door swings outward on a vertical hinge at the left edge of the center
# opening; positive angle opens the door toward +X (outward).

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
W_TOTAL = 1.70
D_TOTAL = 0.50
H_TOTAL = 0.85
OVERHANG = 0.020
TOP_THK = 0.022

BW = W_TOTAL - 2 * OVERHANG      # body width 1.66
BD = D_TOTAL - 2 * OVERHANG      # body depth 0.46
LEG_H = 0.150
BODY_BOT = LEG_H
BODY_TOP = H_TOTAL - TOP_THK     # 0.828
BH = BODY_TOP - BODY_BOT         # body height

WALL = 0.018
INNER_W = BW - 2 * WALL          # 1.624
INNER_D = BD - WALL              # 0.442 (usable depth behind back panel)

# Front opening zone.
ZONE_BOT = BODY_BOT + 0.030      # 0.180
ZONE_TOP = BODY_TOP - 0.017      # 0.811
ZONE_H = ZONE_TOP - ZONE_BOT     # ~0.631

# Section layout along Y.
CUBBY_W = 0.400                  # internal width of each side cubby
DIV_THK = 0.018                  # divider panel thickness

# Divider center Y positions.
DIV_Y_L = -(CUBBY_W + DIV_THK / 2.0)   # left divider
DIV_Y_R = (CUBBY_W + DIV_THK / 2.0)    # right divider

# Cubby internal Y ranges.
CUBBY_L_MIN = -(BW / 2.0 - WALL)        # left cubby left edge
CUBBY_L_MAX = DIV_Y_L - DIV_THK / 2.0   # left cubby right edge
CUBBY_R_MIN = DIV_Y_R + DIV_THK / 2.0   # right cubby left edge
CUBBY_R_MAX = (BW / 2.0 - WALL)         # right cubby right edge

# Center opening Y range.
CENTER_MIN = DIV_Y_L + DIV_THK / 2.0
CENTER_MAX = DIV_Y_R - DIV_THK / 2.0

# Derived widths.
CENTER_W = CENTER_MAX - CENTER_MIN               # 2*CUBBY_W = 0.800
ACTUAL_CUBBY_W = CUBBY_L_MAX - CUBBY_L_MIN       # ~0.394

# Shelves: 2 per cubby at 1/3 and 2/3 of internal height.
SHELF_THK = 0.015
SHELF_DEPTH = INNER_D - 0.010   # slight setback from front
SHELF_Z1 = ZONE_BOT + ZONE_H / 3.0
SHELF_Z2 = ZONE_BOT + 2.0 * ZONE_H / 3.0

# Door dimensions.
DOOR_THK = 0.020
DOOR_W = CENTER_W - 0.006       # small clearance gap
DOOR_H = ZONE_H - 0.006         # small clearance gap
MIRROR_INSET = 0.003            # mirror sits proud of door face
MARGIN = 0.040                  # mirror margin from door edge

# Door hinge at left edge of center opening.
HINGE_X = BD                     # at carcass front face
HINGE_Y = CENTER_MIN             # left edge of center opening
HINGE_Z = (ZONE_BOT + ZONE_TOP) / 2.0

# Carved corner posts (kept from parent).
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG
LEG_SQ = 0.050

# Knob/handle on door.
KNOB_R = 0.0125
STEM_R = 0.0055
STEM_L = 0.014


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_cabinet_cubbies_mirrored_door")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    mirror_mat = model.material("mirror_glass", rgba=(0.85, 0.88, 0.92, 1.0))
    shelf_mat = model.material("shelf_black", rgba=(0.10, 0.10, 0.11, 1.0))

    # ===================================================================
    # ROOT: carcass
    # ===================================================================
    carcass = model.part("carcass")

    # Side panels.
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD, WALL, BH)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                               BODY_BOT + BH / 2.0)),
            material=black,
            name=f"side_panel_{tag}",
        )

    # Back panel.
    carcass.visual(
        Box((WALL, INNER_W, BH)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=black,
        name="back_panel",
    )

    # Bottom board.
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + WALL / 2.0)),
        material=black,
        name="bottom_board",
    )

    # Top stretcher board.
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - WALL / 2.0)),
        material=black,
        name="top_stretcher",
    )

    # Front top rail (above all openings).
    carcass.visual(
        Box((WALL, INNER_W, ZONE_BOT - BODY_BOT - WALL)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (BODY_BOT + WALL + ZONE_BOT) / 2.0)),
        material=black,
        name="front_bottom_rail",
    )
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - ZONE_TOP)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (ZONE_TOP + BODY_TOP) / 2.0)),
        material=black,
        name="front_top_rail",
    )

    # Internal vertical dividers separating cubbies from center section.
    for tag, dy in (("left", DIV_Y_L), ("right", DIV_Y_R)):
        carcass.visual(
            Box((BD - WALL, DIV_THK, ZONE_H)),
            origin=Origin(xyz=(WALL + (BD - WALL) / 2.0, dy,
                               (ZONE_BOT + ZONE_TOP) / 2.0)),
            material=black,
            name=f"divider_{tag}",
        )

    # Shelf boards in side cubbies (2 per cubby, visible through open front).
    # Shelves abut the back panel (x=WALL) and span the full cubby width
    # to contact the side panel and divider inner faces.
    for side_tag, y_min, y_max in (("left", CUBBY_L_MIN, CUBBY_L_MAX),
                                    ("right", CUBBY_R_MIN, CUBBY_R_MAX)):
        cubby_cy = (y_min + y_max) / 2.0
        cubby_w = y_max - y_min
        for si, sz in enumerate((SHELF_Z1, SHELF_Z2)):
            carcass.visual(
                Box((SHELF_DEPTH, cubby_w, SHELF_THK)),
                origin=Origin(xyz=(WALL + SHELF_DEPTH / 2.0,
                                   cubby_cy, sz)),
                material=shelf_mat,
                name=f"shelf_{side_tag}_{si}",
            )

    # Floor boards inside cubbies (bottom shelf surface).
    for side_tag, y_min, y_max in (("left", CUBBY_L_MIN, CUBBY_L_MAX),
                                    ("right", CUBBY_R_MIN, CUBBY_R_MAX)):
        cubby_cy = (y_min + y_max) / 2.0
        cubby_w = y_max - y_min
        carcass.visual(
            Box((SHELF_DEPTH, cubby_w, SHELF_THK)),
            origin=Origin(xyz=(WALL + SHELF_DEPTH / 2.0,
                               cubby_cy, ZONE_BOT + SHELF_THK / 2.0)),
            material=shelf_mat,
            name=f"cubby_floor_{side_tag}",
        )

    # Center section floor board.
    carcass.visual(
        Box((SHELF_DEPTH, CENTER_W, SHELF_THK)),
        origin=Origin(xyz=(WALL + SHELF_DEPTH / 2.0,
                           0.0, ZONE_BOT + SHELF_THK / 2.0)),
        material=shelf_mat,
        name="center_floor",
    )

    # Shelf boards inside center cabinet (visible when door is open).
    for si, sz in enumerate((SHELF_Z1, SHELF_Z2)):
        carcass.visual(
            Box((SHELF_DEPTH, CENTER_W, SHELF_THK)),
            origin=Origin(xyz=(WALL + SHELF_DEPTH / 2.0,
                               0.0, sz)),
            material=shelf_mat,
            name=f"center_shelf_{si}",
        )

    # Silver-gray top slab with overhang.
    carcass.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=silver_top,
        name="top_slab",
    )

    # Carved spiral/zigzag corner posts.
    for ptag, s in (("0", 1), ("1", -1)):
        for i in range(N_SEG):
            odd = i % 2 == 1
            ang = math.radians(25.0) * (-1 if odd else 1) * s
            dx = 0.003 if odd else 0.0
            dy = 0.004 if odd else 0.0
            z0 = BODY_BOT + i * (SEG_H - 0.0015)
            carcass.visual(
                Box((POST_SQ, POST_SQ, SEG_H)),
                origin=Origin(xyz=(POST_CX + dx, s * (POST_CY + dy),
                                   z0 + SEG_H / 2.0),
                              rpy=(0.0, 0.0, ang)),
                material=black_deep,
                name=f"carved_post_{ptag}_seg_{i}",
            )

    # Four straight square legs.
    for tag, lx, ly in (("front_0", POST_CX - 0.012, POST_CY),
                        ("front_1", POST_CX - 0.012, -POST_CY),
                        ("rear_0", 0.030, POST_CY),
                        ("rear_1", 0.030, -POST_CY)):
        carcass.visual(
            Box((LEG_SQ, LEG_SQ, LEG_H + 0.004)),
            origin=Origin(xyz=(lx, ly, (LEG_H + 0.004) / 2.0)),
            material=black,
            name=f"leg_{tag}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=55.0)

    # ===================================================================
    # DOOR: mirrored door on revolute side hinge
    # ===================================================================
    door = model.part("door")

    # Door panel: in local frame, hinge edge at x=0, y=0.
    # Panel extends along +Y (width) and +X (thickness, proud of carcass).
    door.visual(
        Box((DOOR_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(DOOR_THK / 2.0, DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )

    # Mirror panel on the front face of the door (inset from edges).
    mirror_w = DOOR_W - 2 * MARGIN
    mirror_h = DOOR_H - 2 * MARGIN
    door.visual(
        Box((0.004, mirror_w, mirror_h)),
        origin=Origin(xyz=(DOOR_THK + 0.002, DOOR_W / 2.0, 0.0)),
        material=mirror_mat,
        name="mirror_panel",
    )

    # Small handle knob on the free edge (right side) of the door.
    handle_y = DOOR_W - 0.040
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0 + DOOR_THK, handle_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="handle_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004 + DOOR_THK, handle_y, 0.0)),
        material=silver,
        name="handle_ball",
    )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, DOOR_H)), mass=5.0)

    # Door articulation: revolute hinge at left edge of center opening.
    # axis=(0,0,1): positive rotation swings the free edge (+Y) toward +X
    # (outward from the cabinet front).
    model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0,
                                   lower=0.0, upper=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("carcass_to_door")

    # --- Overall dimensions ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("legs_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_170", abs(width_y - 1.70) < 0.02, details=f"w={width_y:.3f}")
    ctx.check("depth_050", abs(depth_x - 0.50) < 0.04, details=f"d={depth_x:.3f}")
    ctx.check("height_085", abs(height_z - 0.85) < 0.01, details=f"h={height_z:.3f}")

    # --- Silver top slab overhangs body ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert top is not None and side is not None and back is not None
    ctx.check(
        "top_overhang_all_sides",
        top[1][1] > side[1][1] + 0.015 and top[0][1] < -side[1][1] - 0.015
        and top[0][0] < back[0][0] - 0.015 and top[1][0] > 0.46 + 0.015,
        details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f}) x=({top[0][0]:.3f},{top[1][0]:.3f})",
    )

    # --- Door joint: revolute, vertical axis, sensible limits ---
    ctx.check("door_joint_is_revolute",
              door_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("door_hinge_axis_vertical",
              abs(door_joint.axis[2]) > 0.99
              and abs(door_joint.axis[0]) < 0.01
              and abs(door_joint.axis[1]) < 0.01)
    ctx.check("door_range_positive",
              door_joint.motion_limits.lower is not None
              and door_joint.motion_limits.upper is not None
              and abs(door_joint.motion_limits.lower) < 1e-9
              and door_joint.motion_limits.upper > 1.0,
              details=f"range=({door_joint.motion_limits.lower},{door_joint.motion_limits.upper})")

    # --- Door closed: panel at carcass front, mirror on front face ---
    door_face = ctx.part_element_world_aabb(door, elem="door_panel")
    mirror = ctx.part_element_world_aabb(door, elem="mirror_panel")
    assert door_face is not None and mirror is not None
    ctx.check("door_at_front_face",
              abs(door_face[0][0] - BD) < 0.005,
              details=f"door back x={door_face[0][0]:.4f}")
    ctx.check("mirror_proud_of_door",
              mirror[0][0] > door_face[1][0] - 0.005,
              details=f"mirror min x={mirror[0][0]:.4f}, door front x={door_face[1][0]:.4f}")
    ctx.check("mirror_within_door_face",
              mirror[0][1] > door_face[0][1] + 0.02
              and mirror[1][1] < door_face[1][1] - 0.02
              and mirror[0][2] > door_face[0][2] + 0.02
              and mirror[1][2] < door_face[1][2] - 0.02,
              details="mirror not inset enough from door edges")

    # --- Handle on door free edge ---
    handle = ctx.part_element_world_aabb(door, elem="handle_ball")
    assert handle is not None
    ctx.check("handle_proud_of_door",
              handle[0][0] > door_face[1][0] + 0.002,
              details=f"handle min x={handle[0][0]:.4f}")

    # --- Door opens outward: positive pose moves door free edge toward +X ---
    rest_pos = ctx.part_world_position(door)
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        open_pos = ctx.part_world_position(door)
        open_face = ctx.part_element_world_aabb(door, elem="door_panel")
    assert rest_pos is not None and open_pos is not None and open_face is not None
    ctx.check("door_opens_outward",
              open_face[1][0] > BD + 0.10,
              details=f"open front x={open_face[1][0]:.4f} vs closed front {BD:.3f}")

    # --- Open side cubbies: shelves visible through front gap ---
    for side in ("left", "right"):
        for si in range(2):
            shelf = ctx.part_element_world_aabb(carcass, elem=f"shelf_{side}_{si}")
            assert shelf is not None
            ctx.check(f"shelf_{side}_{si}_visible_through_front",
                      shelf[1][0] > BD - 0.10,
                      details=f"shelf front x={shelf[1][0]:.4f}")

    # --- Cubby shelves are within carcass width ---
    for side in ("left", "right"):
        for si in range(2):
            ctx.expect_within(
                carcass, carcass, axes="y",
                margin=0.001,
                name=f"shelf_{side}_{si}_within_body",
            )

    # --- Dividers separate sections ---
    div_l = ctx.part_element_world_aabb(carcass, elem="divider_left")
    div_r = ctx.part_element_world_aabb(carcass, elem="divider_right")
    assert div_l is not None and div_r is not None
    ctx.check("dividers_flank_center",
              div_l[1][1] < 0.0 and div_r[0][1] > 0.0,
              details=f"left max y={div_l[1][1]:.3f}, right min y={div_r[0][1]:.3f}")

    # --- Carved posts still present ---
    seg0 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_0")
    assert seg0 is not None
    ctx.check("carved_posts_present", seg0[0][2] < LEG_H + 0.002)

    # --- Handle exists on door ---
    ctx.check("door_has_handle",
              door.get_visual("handle_ball") is not None)

    return ctx.report()


object_model = build_object_model()
