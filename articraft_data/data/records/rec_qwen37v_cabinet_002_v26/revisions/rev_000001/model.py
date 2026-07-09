from __future__ import annotations

# Wide black wooden storage cabinet (~1.70 m W x 0.85 m H x 0.50 m D).
# Variant of the double dresser: open side cubbies with visible shelf boards,
# a closed central cabinet with two hinged doors, a small rotating latch at
# the center door seam, and recessed panel borders on each door face.
#
# World layout: front faces +X (back at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four square
# legs ~0.15 m tall. Matte black wood carcass and door fronts; a thin smooth
# silver-gray top slab overhangs the body ~0.02 m on all sides. The front
# corners carry decorative carved posts continuing into straight legs.

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
BH = BODY_TOP - BODY_BOT

WALL = 0.018
INNER_W = BW - 2 * WALL          # 1.624

# Section layout along Y: left cubby | divider | center cabinet | divider | right cubby
CUBBY_W = 0.350
DIVIDER_W = WALL
CENTER_W = INNER_W - 2 * CUBBY_W - 2 * DIVIDER_W  # ~0.888

# Y boundaries (centered at Y=0)
CUBBY_L_OUT = -INNER_W / 2.0
CUBBY_L_IN = CUBBY_L_OUT + CUBBY_W
DIV_L_IN = CUBBY_L_IN + DIVIDER_W
CENTER_L = DIV_L_IN
CENTER_R = CENTER_L + CENTER_W
DIV_R_IN = CENTER_R + DIVIDER_W
CUBBY_R_OUT = DIV_R_IN
CUBBY_R_IN = CUBBY_R_OUT + CUBBY_W

CUBBY_L_CY = (CUBBY_L_OUT + CUBBY_L_IN) / 2.0
DIV_L_CY = (CUBBY_L_IN + DIV_L_IN) / 2.0
CENTER_CY = (CENTER_L + CENTER_R) / 2.0
DIV_R_CY = (CENTER_R + DIV_R_IN) / 2.0
CUBBY_R_CY = (CUBBY_R_OUT + CUBBY_R_IN) / 2.0

# Front opening zone
ZONE_BOT = BODY_BOT + 0.030
ZONE_TOP = BODY_TOP - 0.017
ZONE_H = ZONE_TOP - ZONE_BOT
REVEAL = 0.004

# Doors
DOOR_THK = 0.020
DOOR_W = (CENTER_W - REVEAL) / 2.0
DOOR_H = ZONE_H
FACE_PROUD = 0.002
FRONT_X = BD

# Recessed panel border
RECESS_MARGIN = 0.035
RECESS_DEPTH = 0.004
RECESS_W = DOOR_W - 2 * RECESS_MARGIN
RECESS_H = DOOR_H - 2 * RECESS_MARGIN

# Latch
LATCH_W = 0.060
LATCH_H = 0.018
LATCH_THK = 0.010
LATCH_Z = (ZONE_BOT + ZONE_TOP) / 2.0

# Shelves
SHELF_THK = 0.016
N_SHELVES = 2

# Carved posts
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG
LEG_SQ = 0.050

# Knobs
KNOB_R = 0.0125
STEM_R = 0.0055
STEM_L = 0.014


def _build_left_door(model: ArticulatedObject, black, recess_mat, silver):
    """Left door: hinge at origin (left edge), panel extends +Y.
    Outer face at local x=0 (facing +X in world when closed).
    Knob near free edge (+Y end) on outer face."""
    door = model.part("left_door")
    # Panel: x from -THK to 0, outer face at x=0
    door.visual(
        Box((DOOR_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(-DOOR_THK / 2.0, DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )
    # Recessed panel on outer face (x=0 side). Border strips proud of recessed center.
    bx = 0.001  # strips sit slightly proud of the panel face
    # Top border
    door.visual(
        Box((RECESS_DEPTH, DOOR_W - 2 * RECESS_MARGIN, RECESS_MARGIN)),
        origin=Origin(xyz=(bx - RECESS_DEPTH / 2.0, DOOR_W / 2.0,
                           DOOR_H / 2.0 - RECESS_MARGIN / 2.0)),
        material=black,
        name="recess_border_top",
    )
    # Bottom border
    door.visual(
        Box((RECESS_DEPTH, DOOR_W - 2 * RECESS_MARGIN, RECESS_MARGIN)),
        origin=Origin(xyz=(bx - RECESS_DEPTH / 2.0, DOOR_W / 2.0,
                           -DOOR_H / 2.0 + RECESS_MARGIN / 2.0)),
        material=black,
        name="recess_border_bottom",
    )
    # Left border (near hinge)
    door.visual(
        Box((RECESS_DEPTH, RECESS_MARGIN, DOOR_H - 2 * RECESS_MARGIN)),
        origin=Origin(xyz=(bx - RECESS_DEPTH / 2.0, RECESS_MARGIN / 2.0, 0.0)),
        material=black,
        name="recess_border_left",
    )
    # Right border (near free edge)
    door.visual(
        Box((RECESS_DEPTH, RECESS_MARGIN, DOOR_H - 2 * RECESS_MARGIN)),
        origin=Origin(xyz=(bx - RECESS_DEPTH / 2.0,
                           DOOR_W - RECESS_MARGIN / 2.0, 0.0)),
        material=black,
        name="recess_border_right",
    )
    # Recessed center panel (slightly behind the border strips)
    door.visual(
        Box((RECESS_DEPTH * 0.5, RECESS_W, RECESS_H)),
        origin=Origin(xyz=(-RECESS_DEPTH * 0.5 - 0.001, DOOR_W / 2.0, 0.0)),
        material=recess_mat,
        name="recessed_center",
    )
    # Knob near free edge on outer face
    knob_y = DOOR_W - 0.040
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0 + 0.002, knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.002, knob_y, 0.0)),
        material=silver,
        name="knob_ball",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, DOOR_H)), mass=3.0)
    return door


def _build_right_door(model: ArticulatedObject, black, recess_mat, silver):
    """Right door: hinge at origin (right edge), panel extends -Y.
    Outer face at local x=0 (facing +X in world when closed).
    Knob near free edge (-Y end) on outer face."""
    door = model.part("right_door")
    # Panel: x from -THK to 0, outer face at x=0; extends from y=-W to y=0
    door.visual(
        Box((DOOR_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(-DOOR_THK / 2.0, -DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )
    # Recessed panel borders on outer face
    bx = 0.001
    door.visual(
        Box((RECESS_DEPTH, DOOR_W - 2 * RECESS_MARGIN, RECESS_MARGIN)),
        origin=Origin(xyz=(bx - RECESS_DEPTH / 2.0, -DOOR_W / 2.0,
                           DOOR_H / 2.0 - RECESS_MARGIN / 2.0)),
        material=black,
        name="recess_border_top",
    )
    door.visual(
        Box((RECESS_DEPTH, DOOR_W - 2 * RECESS_MARGIN, RECESS_MARGIN)),
        origin=Origin(xyz=(bx - RECESS_DEPTH / 2.0, -DOOR_W / 2.0,
                           -DOOR_H / 2.0 + RECESS_MARGIN / 2.0)),
        material=black,
        name="recess_border_bottom",
    )
    # Right border (near hinge at y=0)
    door.visual(
        Box((RECESS_DEPTH, RECESS_MARGIN, DOOR_H - 2 * RECESS_MARGIN)),
        origin=Origin(xyz=(bx - RECESS_DEPTH / 2.0, -RECESS_MARGIN / 2.0, 0.0)),
        material=black,
        name="recess_border_right",
    )
    # Left border (near free edge at y=-W)
    door.visual(
        Box((RECESS_DEPTH, RECESS_MARGIN, DOOR_H - 2 * RECESS_MARGIN)),
        origin=Origin(xyz=(bx - RECESS_DEPTH / 2.0,
                           -DOOR_W + RECESS_MARGIN / 2.0, 0.0)),
        material=black,
        name="recess_border_left",
    )
    # Recessed center panel
    door.visual(
        Box((RECESS_DEPTH * 0.5, RECESS_W, RECESS_H)),
        origin=Origin(xyz=(-RECESS_DEPTH * 0.5 - 0.001, -DOOR_W / 2.0, 0.0)),
        material=recess_mat,
        name="recessed_center",
    )
    # Knob near free edge (-Y end) on outer face
    knob_y = -DOOR_W + 0.040
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0 + 0.002, knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.002, knob_y, 0.0)),
        material=silver,
        name="knob_ball",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, DOOR_H)), mass=3.0)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_storage_cabinet_cubbies")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    recess_mat = model.material("recess_dark", rgba=(0.04, 0.04, 0.045, 1.0))
    shelf_mat = model.material("shelf_black", rgba=(0.10, 0.10, 0.11, 1.0))

    # ===================================================================
    # ROOT: carcass
    # ===================================================================
    carcass = model.part("carcass")

    # Side panels
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD, WALL, BH)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                               BODY_BOT + BH / 2.0)),
            material=black,
            name=f"side_panel_{tag}",
        )

    # Back panel
    carcass.visual(
        Box((WALL, INNER_W, BH)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=black,
        name="back_panel",
    )

    # Bottom board
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + 0.009)),
        material=black,
        name="bottom_board",
    )

    # Top stretcher
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black,
        name="top_stretcher",
    )

    # Front rails (below/above opening zone)
    carcass.visual(
        Box((WALL, INNER_W, ZONE_BOT - BODY_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (BODY_BOT + ZONE_BOT) / 2.0)),
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

    # Vertical dividers between cubbies and center
    for tag, cy in (("left", DIV_L_CY), ("right", DIV_R_CY)):
        carcass.visual(
            Box((BD - WALL, DIVIDER_W, ZONE_H)),
            origin=Origin(xyz=(WALL + (BD - WALL) / 2.0, cy,
                               (ZONE_BOT + ZONE_TOP) / 2.0)),
            material=black,
            name=f"divider_{tag}",
        )

    # Shelf boards in side cubbies
    shelf_depth = BD - 2 * WALL  # from back panel inner face to front rail inner face
    shelf_x_center = WALL + shelf_depth / 2.0
    for cubby_tag, cubby_cy in (("left", CUBBY_L_CY), ("right", CUBBY_R_CY)):
        for i in range(N_SHELVES):
            shelf_z = ZONE_BOT + (i + 1) * ZONE_H / (N_SHELVES + 1)
            carcass.visual(
                Box((shelf_depth, CUBBY_W - 0.004, SHELF_THK)),
                origin=Origin(xyz=(shelf_x_center, cubby_cy, shelf_z)),
                material=shelf_mat,
                name=f"shelf_{cubby_tag}_{i}",
            )

    # Center cabinet internal shelf
    carcass.visual(
        Box((shelf_depth - 0.04, CENTER_W - 0.004, SHELF_THK)),
        origin=Origin(xyz=(shelf_x_center - 0.02, CENTER_CY,
                           (ZONE_BOT + ZONE_TOP) / 2.0)),
        material=shelf_mat,
        name="center_shelf",
    )

    # Silver top slab
    carcass.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=silver_top,
        name="top_slab",
    )

    # Carved corner posts
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

    # Four legs
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
    # DOORS
    # ===================================================================
    left_door = _build_left_door(model, black, recess_mat, silver)
    right_door = _build_right_door(model, black, recess_mat, silver)

    # Left door hinge: at left edge of center opening, front face
    # Door extends +Y from hinge. Outer face at local x=0.
    # At q=0, local x=0 maps to world x = hinge_x.
    # We want the outer face slightly proud of carcass front (FRONT_X).
    hinge_x = FRONT_X + FACE_PROUD
    hinge_z = (ZONE_BOT + ZONE_TOP) / 2.0

    model.articulation(
        "carcass_to_left_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(hinge_x, CENTER_L, hinge_z)),
        # Right-hand about -Z: +Y rotates toward +X (door opens outward)
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                   lower=0.0, upper=math.radians(95)),
    )

    # Right door hinge: at right edge of center opening
    # Door extends -Y from hinge. Outer face at local x=0.
    model.articulation(
        "carcass_to_right_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(hinge_x, CENTER_R, hinge_z)),
        # Right-hand about +Z: -Y rotates toward +X (door opens outward)
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                   lower=0.0, upper=math.radians(95)),
    )

    # ===================================================================
    # LATCH
    # ===================================================================
    latch = model.part("latch")
    latch.visual(
        Box((LATCH_THK, LATCH_W, LATCH_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=silver,
        name="latch_bar",
    )
    latch.visual(
        Cylinder(radius=0.008, length=LATCH_THK + 0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="latch_pivot",
    )
    latch.inertial = Inertial.from_geometry(
        Box((LATCH_THK, LATCH_W, LATCH_H)), mass=0.1)

    # Latch mounted on front face at center seam, proud of doors
    latch_x = hinge_x + DOOR_THK + 0.003
    model.articulation(
        "carcass_to_latch",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=latch,
        origin=Origin(xyz=(latch_x, CENTER_CY, LATCH_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0,
                                   lower=0.0, upper=math.radians(90)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    latch = object_model.get_part("latch")

    left_hinge = object_model.get_articulation("carcass_to_left_door")
    right_hinge = object_model.get_articulation("carcass_to_right_door")
    latch_joint = object_model.get_articulation("carcass_to_latch")

    # --- Grounding and overall scale ---
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

    # --- Silver top overhangs body ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert top is not None and side is not None and back is not None
    ctx.check("top_overhang_all_sides",
              top[1][1] > side[1][1] + 0.015 and top[0][1] < -side[1][1] - 0.015
              and top[0][0] < back[0][0] - 0.015 and top[1][0] > FRONT_X + 0.015,
              details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f})")

    # --- Open side cubbies: shelf boards visible ---
    for cubby in ("left", "right"):
        for i in range(N_SHELVES):
            shelf_name = f"shelf_{cubby}_{i}"
            shelf = ctx.part_element_world_aabb(carcass, elem=shelf_name)
            assert shelf is not None
            ctx.check(f"{shelf_name}_inside_body",
                      shelf[0][0] > 0.01 and shelf[1][0] < BD + 0.01,
                      details=f"x=({shelf[0][0]:.3f},{shelf[1][0]:.3f})")
            ctx.check(f"{shelf_name}_in_zone",
                      shelf[0][2] > ZONE_BOT and shelf[1][2] < ZONE_TOP,
                      details=f"z=({shelf[0][2]:.3f},{shelf[1][2]:.3f})")

    # --- Vertical dividers ---
    for tag in ("left", "right"):
        div = ctx.part_element_world_aabb(carcass, elem=f"divider_{tag}")
        assert div is not None
        ctx.check(f"divider_{tag}_full_height",
                  div[1][2] - div[0][2] > ZONE_H * 0.9,
                  details=f"h={div[1][2] - div[0][2]:.3f}")

    # --- Doors: revolute joints ---
    ctx.check("left_door_revolute",
              left_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("right_door_revolute",
              right_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("left_door_limits_valid",
              abs(left_hinge.motion_limits.lower) < 1e-6
              and left_hinge.motion_limits.upper > math.radians(80))
    ctx.check("right_door_limits_valid",
              abs(right_hinge.motion_limits.lower) < 1e-6
              and right_hinge.motion_limits.upper > math.radians(80))

    # --- Doors: recessed panel borders ---
    for door_name in ("left_door", "right_door"):
        door_part = object_model.get_part(door_name)
        recess = ctx.part_element_world_aabb(door_part, elem="recessed_center")
        assert recess is not None
        rw = recess[1][1] - recess[0][1]
        rh = recess[1][2] - recess[0][2]
        ctx.check(f"{door_name}_recessed_panel",
                  rw > 0.10 and rh > 0.10,
                  details=f"recess=({rw:.3f}x{rh:.3f})")

    # --- Closed pose: doors proud of carcass front ---
    left_face = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_face = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert left_face is not None and right_face is not None
    ctx.check("doors_proud_of_carcass",
              left_face[1][0] > FRONT_X and right_face[1][0] > FRONT_X,
              details=f"L={left_face[1][0]:.4f}, R={right_face[1][0]:.4f}")

    # --- Open pose: doors swing outward ---
    with ctx.pose({left_hinge: left_hinge.motion_limits.upper * 0.5}):
        left_mid = ctx.part_element_world_aabb(left_door, elem="door_panel")
    with ctx.pose({right_hinge: right_hinge.motion_limits.upper * 0.5}):
        right_mid = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert left_mid is not None and right_mid is not None

    # When opened, the door free edge should extend further in +X
    ctx.check("left_door_opens_outward",
              left_mid[1][0] > FRONT_X + 0.05,
              details=f"open face x={left_mid[1][0]:.4f}")
    ctx.check("right_door_opens_outward",
              right_mid[1][0] > FRONT_X + 0.05,
              details=f"open face x={right_mid[1][0]:.4f}")

    # --- Latch: revolute joint at center seam ---
    ctx.check("latch_revolute",
              latch_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("latch_range_90",
              abs(latch_joint.motion_limits.upper - math.radians(90)) < 0.01)

    latch_bar = ctx.part_element_world_aabb(latch, elem="latch_bar")
    assert latch_bar is not None
    ctx.check("latch_at_center",
              abs((latch_bar[0][1] + latch_bar[1][1]) / 2.0 - CENTER_CY) < 0.02)

    # Latch rotation visibly changes orientation
    with ctx.pose({latch_joint: latch_joint.motion_limits.upper}):
        latch_rot = ctx.part_element_world_aabb(latch, elem="latch_bar")
    assert latch_rot is not None
    span0 = latch_bar[1][2] - latch_bar[0][2]  # Z span at rest
    span1 = latch_rot[1][2] - latch_rot[0][2]   # Z span when rotated
    ctx.check("latch_rotates_visibly",
              abs(span0 - span1) > 0.005,
              details=f"z_span rest={span0:.4f} rotated={span1:.4f}")

    # --- Carved posts preserved ---
    seg0 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_0")
    assert seg0 is not None
    ctx.check("carved_posts_present", seg0[0][2] < LEG_H + 0.002)

    return ctx.report()


object_model = build_object_model()
