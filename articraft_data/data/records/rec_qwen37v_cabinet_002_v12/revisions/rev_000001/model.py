from __future__ import annotations

# Low wide dresser cabinet variant (~1.50 m W x 0.75 m H x 0.45 m D).
#
# World layout: front faces +X (back of the body at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four square
# legs ~0.12 m tall. Matte black wood carcass, drawer fronts, and door panels;
# a thin smooth silver-gray top slab overhangs the body ~0.02 m on all sides.
#
# Upper zone: three full-width drawers stacked vertically, each a PRISMATIC
# joint sliding out along +X, range 0 to 0.35 m. Each drawer is a hollow
# open-top tray behind a flat front panel with two silver ball knobs.
#
# Lower zone: two cabinet doors meeting at a center seam, each on a REVOLUTE
# hinge at its outer side edge. Shelf boards inside the cabinet are visible
# through the open gap when doors swing outward. A small rotating latch at
# the center seam locks/unlocks the doors (REVOLUTE, 0 to pi/2).

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
W_TOTAL = 1.50
D_TOTAL = 0.45
H_TOTAL = 0.75
OVERHANG = 0.018
TOP_THK = 0.020

BW = W_TOTAL - 2 * OVERHANG      # body width 1.464
BD = D_TOTAL - 2 * OVERHANG      # body depth 0.414
LEG_H = 0.120
BODY_BOT = LEG_H                 # 0.12
BODY_TOP = H_TOTAL - TOP_THK     # 0.73
BH = BODY_TOP - BODY_BOT         # 0.61

WALL = 0.018
INNER_W = BW - 2 * WALL          # 1.428

REVEAL = 0.008

# Horizontal divider between drawer zone and door zone.
DIVIDER_Z = BODY_BOT + 0.290     # ~0.410
DIVIDER_THK = 0.016

# --- Drawer zone (upper) ---
DRAWER_ZONE_BOT = DIVIDER_Z + DIVIDER_THK  # 0.426
DRAWER_ZONE_TOP = BODY_TOP - 0.018         # 0.712
DRAWER_ZONE_H = DRAWER_ZONE_TOP - DRAWER_ZONE_BOT  # 0.286
FH_DRAWER = (DRAWER_ZONE_H - 2 * REVEAL) / 3.0  # ~0.090

DRAWER_CZ = [
    DRAWER_ZONE_BOT + FH_DRAWER / 2.0,
    DRAWER_ZONE_BOT + FH_DRAWER + REVEAL + FH_DRAWER / 2.0,
    DRAWER_ZONE_BOT + 2 * (FH_DRAWER + REVEAL) + FH_DRAWER / 2.0,
]

# --- Door zone (lower) ---
DOOR_ZONE_BOT = BODY_BOT + 0.030    # 0.150
DOOR_ZONE_TOP = DIVIDER_Z - REVEAL  # 0.402
DOOR_FH = DOOR_ZONE_TOP - DOOR_ZONE_BOT  # ~0.252
DOOR_CZ = (DOOR_ZONE_BOT + DOOR_ZONE_TOP) / 2.0

DOOR_FW = (INNER_W - REVEAL) / 2.0  # each door width
DOOR_CY_LEFT = -(DOOR_FW / 2.0 + REVEAL / 2.0)
DOOR_CY_RIGHT = (DOOR_FW / 2.0 + REVEAL / 2.0)

# Drawer front dimensions.
FACE_THK = 0.018
FACE_PROUD = 0.001
FRONT_X = BD
SLAB_BACK_X = FRONT_X + FACE_PROUD
JOINT_X = SLAB_BACK_X + FACE_THK

TRAVEL = 0.350
TRAY_D = 0.370
TRAY_T = 0.012
TRAY_W = INNER_W - 0.010
TRAY_H = FH_DRAWER - 0.014

# Door panel thickness.
DOOR_THK = 0.018

# Hinge position: doors mount as overlay panels proud of the carcass front.
# The hinge line is set so the door back face clears the carcass front by ~1mm.
HINGE_X = FRONT_X + FACE_PROUD + FACE_THK  # 0.433

# Shelf boards inside the door compartment.
SHELF_COUNT = 2
SHELF_THK = 0.014
SHELF_ZONE_H = DOOR_FH - 0.020
SHELF_SPACING = SHELF_ZONE_H / (SHELF_COUNT + 1)

# Knobs.
KNOB_R = 0.012
STEM_R = 0.005
STEM_L = 0.014

# Latch dimensions.
LATCH_W = 0.040
LATCH_H = 0.016
LATCH_THK = 0.006
LATCH_STEM_R = 0.004
LATCH_STEM_L = 0.030  # long stem to embed into carcass center stile

# Leg dimensions.
LEG_SQ = 0.045


def _build_drawer(model: ArticulatedObject, name: str, front_w: float,
                  front_h: float, tray_w: float, tray_h: float,
                  knob_ys: list[float], black, tray_mat, silver):
    """Drawer: front panel outer surface at local x=0, tray extends toward -X."""
    drawer = model.part(name)

    drawer.visual(
        Box((FACE_THK, front_w, front_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, 0.0)),
        material=black,
        name="front_panel",
    )

    tray_back_x = -(FACE_THK + TRAY_D)
    tray_cx = -(FACE_THK + TRAY_D / 2.0)
    tray_bot = -front_h / 2.0 + 0.012
    drawer.visual(
        Box((TRAY_D, tray_w, TRAY_T)),
        origin=Origin(xyz=(tray_cx, 0.0, tray_bot + TRAY_T / 2.0)),
        material=tray_mat,
        name="tray_bottom",
    )
    wall_h = tray_h - TRAY_T + 0.002
    wall_cz = tray_bot + TRAY_T - 0.002 + wall_h / 2.0
    drawer.visual(
        Box((TRAY_T, tray_w, wall_h)),
        origin=Origin(xyz=(tray_back_x + TRAY_T / 2.0, 0.0, wall_cz)),
        material=tray_mat,
        name="tray_back_wall",
    )
    side_len = TRAY_D + 0.002
    for tag, s in (("0", 1), ("1", -1)):
        drawer.visual(
            Box((side_len, TRAY_T, wall_h)),
            origin=Origin(xyz=(-FACE_THK + 0.002 - side_len / 2.0,
                               s * (tray_w / 2.0 - TRAY_T / 2.0), wall_cz)),
            material=tray_mat,
            name=f"tray_side_wall_{tag}",
        )
    drawer.visual(
        Box((TRAY_T, tray_w, wall_h)),
        origin=Origin(xyz=(-FACE_THK - TRAY_T / 2.0 + 0.002, 0.0, wall_cz)),
        material=tray_mat,
        name="tray_front_wall",
    )

    for i, ky in enumerate(knob_ys):
        drawer.visual(
            Cylinder(radius=STEM_R, length=STEM_L + 0.004),
            origin=Origin(xyz=((STEM_L - 0.004) / 2.0, ky, 0.0),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=silver,
            name=f"knob_stem_{i}",
        )
        drawer.visual(
            Sphere(radius=KNOB_R),
            origin=Origin(xyz=(STEM_L + KNOB_R - 0.004, ky, 0.0)),
            material=silver,
            name=f"knob_ball_{i}",
        )

    drawer.inertial = Inertial.from_geometry(
        Box((TRAY_D, tray_w, tray_h)), mass=3.0)
    return drawer


def _build_door(model: ArticulatedObject, name: str, door_w: float,
                door_h: float, hinge_side: float, black, silver):
    """Door: hinge edge at local y=0, panel extends along local Y toward center.
    Front face at local x=0, panel spans x in [-DOOR_THK, 0].
    hinge_side: +1 if panel extends along +Y, -1 if along -Y."""
    door = model.part(name)

    door.visual(
        Box((DOOR_THK, door_w, door_h)),
        origin=Origin(xyz=(-DOOR_THK / 2.0, hinge_side * door_w / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )

    # Small silver knob near the free edge.
    knob_y = hinge_side * (door_w - 0.040)
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0, knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="door_knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004, knob_y, 0.0)),
        material=silver,
        name="door_knob_ball",
    )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, door_w, door_h)), mass=2.5)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_wide_dresser_cabinet")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    tray_mat = model.material("tray_black", rgba=(0.13, 0.13, 0.14, 1.0))
    shelf_mat = model.material("shelf_wood", rgba=(0.10, 0.10, 0.11, 1.0))

    # ===================================================================
    # ROOT: carcass
    # ===================================================================
    carcass = model.part("carcass")

    # Side panels (stop short of the front face to clear overlay doors).
    side_depth = BD - DOOR_THK - 0.002  # clear the door back face
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((side_depth, WALL, BH)),
            origin=Origin(xyz=(side_depth / 2.0, s * (BW / 2.0 - WALL / 2.0),
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
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + 0.009)),
        material=black,
        name="bottom_board",
    )
    # Top stretcher board.
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black,
        name="top_stretcher",
    )

    # Horizontal divider between drawer zone and door zone.
    carcass.visual(
        Box((BD - WALL, INNER_W, DIVIDER_THK)),
        origin=Origin(xyz=(WALL + (BD - WALL) / 2.0, 0.0,
                           DIVIDER_Z + DIVIDER_THK / 2.0)),
        material=black,
        name="zone_divider",
    )

    # Front face frame: rails and stiles at the front face plane.
    # Bottom rail (below doors).
    carcass.visual(
        Box((WALL, INNER_W, DOOR_ZONE_BOT - BODY_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (BODY_BOT + DOOR_ZONE_BOT) / 2.0)),
        material=black,
        name="front_bottom_rail",
    )
    # Top rail (above drawers).
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - DRAWER_ZONE_TOP)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (DRAWER_ZONE_TOP + BODY_TOP) / 2.0)),
        material=black,
        name="front_top_rail",
    )
    # Front face between drawer zone and door zone (divider face strip).
    carcass.visual(
        Box((WALL, INNER_W, DIVIDER_THK + REVEAL)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           DIVIDER_Z + (DIVIDER_THK + REVEAL) / 2.0)),
        material=black,
        name="front_divider_strip",
    )

    # Side stiles flanking the door opening.
    stile_w = 0.020
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, stile_w, DOOR_FH)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (INNER_W / 2.0 - stile_w / 2.0),
                               DOOR_CZ)),
            material=black,
            name=f"door_stile_{tag}",
        )

    # Center stile behind the door center seam.
    carcass.visual(
        Box((WALL, 0.024, DOOR_FH)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, DOOR_CZ)),
        material=black_deep,
        name="door_center_stile",
    )

    # Drawer runners (dust panels).
    for i, cz in enumerate(DRAWER_CZ):
        tray_bot_z = cz + (-FH_DRAWER / 2.0 + 0.012)
        carcass.visual(
            Box((BD - 0.030, INNER_W, 0.014)),
            origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, 0.0,
                               tray_bot_z - 0.007)),
            material=black_deep,
            name=f"drawer_runner_{i}",
        )

    # Horizontal rails between drawer rows.
    for i in range(2):
        rail_z = DRAWER_ZONE_BOT + (i + 1) * (FH_DRAWER + REVEAL) - REVEAL / 2.0
        carcass.visual(
            Box((0.016, INNER_W, REVEAL)),
            origin=Origin(xyz=(BD - 0.008, 0.0, rail_z)),
            material=black_deep,
            name=f"drawer_row_rail_{i}",
        )

    # --- Shelf boards inside the door compartment ---
    shelf_depth = BD - WALL - 0.010
    shelf_x_center = WALL + shelf_depth / 2.0
    for i in range(SHELF_COUNT):
        shelf_z = DOOR_ZONE_BOT + (i + 1) * SHELF_SPACING
        carcass.visual(
            Box((shelf_depth, INNER_W - 0.004, SHELF_THK)),
            origin=Origin(xyz=(shelf_x_center, 0.0, shelf_z)),
            material=shelf_mat,
            name=f"shelf_board_{i}",
        )

    # Silver-gray top slab.
    carcass.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=silver_top,
        name="top_slab",
    )

    # Four square legs.
    for tag, lx, ly in (("front_left", BD - LEG_SQ / 2.0, BW / 2.0 - LEG_SQ / 2.0),
                         ("front_right", BD - LEG_SQ / 2.0, -(BW / 2.0 - LEG_SQ / 2.0)),
                         ("rear_left", LEG_SQ / 2.0, BW / 2.0 - LEG_SQ / 2.0),
                         ("rear_right", LEG_SQ / 2.0, -(BW / 2.0 - LEG_SQ / 2.0))):
        carcass.visual(
            Box((LEG_SQ, LEG_SQ, LEG_H + 0.004)),
            origin=Origin(xyz=(lx, ly, (LEG_H + 0.004) / 2.0)),
            material=black,
            name=f"leg_{tag}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=45.0)

    # ===================================================================
    # DRAWERS: three PRISMATIC slides along +X
    # ===================================================================
    drawer_knob_ys = [-0.22, 0.22]
    drawer_parts = []
    for i, cz in enumerate(DRAWER_CZ):
        d = _build_drawer(model, f"drawer_{i}", INNER_W - 0.004, FH_DRAWER,
                          TRAY_W, TRAY_H, drawer_knob_ys,
                          black, tray_mat, silver)
        drawer_parts.append((f"drawer_{i}", d, cz))

    for name, d, cz in drawer_parts:
        model.articulation(
            f"carcass_to_{name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=d,
            origin=Origin(xyz=(JOINT_X, 0.0, cz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=0.4,
                                       lower=0.0, upper=TRAVEL),
        )

    # ===================================================================
    # DOORS: two REVOLUTE doors on side hinges
    # ===================================================================
    # Left door: hinge on the left side edge (-Y). Panel extends toward center (+Y).
    left_door = _build_door(model, "door_left", DOOR_FW, DOOR_FH,
                            +1.0, black, silver)
    # Right door: hinge on the right side edge (+Y). Panel extends toward center (-Y).
    right_door = _build_door(model, "door_right", DOOR_FW, DOOR_FH,
                             -1.0, black, silver)

    hinge_y_left = -(INNER_W / 2.0)
    hinge_y_right = INNER_W / 2.0
    hinge_z = DOOR_CZ

    # Left door: panel extends +Y from hinge. Rotation about -Z opens outward (+X).
    model.articulation(
        "carcass_to_door_left",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(HINGE_X, hinge_y_left, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                   lower=0.0, upper=math.radians(95)),
    )

    # Right door: panel extends -Y from hinge. Rotation about +Z opens outward (+X).
    model.articulation(
        "carcass_to_door_right",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(HINGE_X, hinge_y_right, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                   lower=0.0, upper=math.radians(95)),
    )

    # ===================================================================
    # LATCH: rotating turn-latch at center seam
    # ===================================================================
    latch = model.part("latch")

    # Latch bar: horizontal when locked (q=0), vertical when unlocked (q=pi/2).
    latch.visual(
        Box((LATCH_THK, LATCH_W, LATCH_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=silver,
        name="latch_bar",
    )
    # Latch stem: extends back (-X) into the carcass center stile.
    latch.visual(
        Cylinder(radius=LATCH_STEM_R, length=LATCH_STEM_L + 0.004),
        origin=Origin(xyz=(-(LATCH_STEM_L + 0.004) / 2.0, 0.0, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="latch_stem",
    )
    # Small knob on the front of the latch.
    latch.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(LATCH_THK / 2.0 + 0.006, 0.0, 0.0)),
        material=silver,
        name="latch_knob",
    )

    latch.inertial = Inertial.from_geometry(
        Box((LATCH_THK, LATCH_W, LATCH_H)), mass=0.1)

    # Mount the latch on the carcass center stile. The stem embeds into the
    # stile for structural support. The latch bar sits just proud of the doors.
    # Latch origin at: x such that the bar is in front of the door face,
    # and the stem reaches back into the center stile.
    latch_x = HINGE_X + 0.001  # 1mm in front of door front face
    model.articulation(
        "carcass_to_latch",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=latch,
        origin=Origin(xyz=(latch_x, 0.0, DOOR_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0,
                                   lower=0.0, upper=math.radians(90)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    drawer_names = [f"drawer_{i}" for i in range(3)]
    drawers = {n: object_model.get_part(n) for n in drawer_names}
    drawer_joints = {n: object_model.get_articulation(f"carcass_to_{n}")
                     for n in drawer_names}

    door_left = object_model.get_part("door_left")
    door_right = object_model.get_part("door_right")
    latch = object_model.get_part("latch")
    joint_door_left = object_model.get_articulation("carcass_to_door_left")
    joint_door_right = object_model.get_articulation("carcass_to_door_right")
    joint_latch = object_model.get_articulation("carcass_to_latch")

    # --- Latch stem embeds into carcass center stile (intentional) ---
    ctx.allow_overlap(
        "carcass", "latch",
        elem_a="door_center_stile",
        elem_b="latch_stem",
        reason="The latch stem embeds into the carcass center stile for structural mounting.",
    )
    # Latch bar bridges across the door center seam (intentional seated contact).
    ctx.allow_overlap(
        "door_left", "latch",
        elem_a="door_panel",
        elem_b="latch_bar",
        reason="The latch bar sits against the left door edge, bridging the center seam.",
    )
    ctx.allow_overlap(
        "door_right", "latch",
        elem_a="door_panel",
        elem_b="latch_bar",
        reason="The latch bar sits against the right door edge, bridging the center seam.",
    )

    # --- Grounding and overall scale ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("legs_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_near_150", abs(width_y - 1.50) < 0.03,
              details=f"w={width_y:.3f}")
    ctx.check("depth_near_045", abs(depth_x - 0.45) < 0.05,
              details=f"d={depth_x:.3f}")
    ctx.check("height_near_075", abs(height_z - 0.75) < 0.02,
              details=f"h={height_z:.3f}")

    # --- Silver top slab overhangs body ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert top is not None and side is not None and back is not None
    ctx.check("top_overhang_sides",
              top[1][1] > side[1][1] + 0.010 and top[0][1] < side[0][1] - 0.010,
              details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f}), "
                      f"side y=({side[0][1]:.3f},{side[1][1]:.3f})")
    ctx.check("top_overhang_front_back",
              top[0][0] < back[0][0] - 0.010 and top[1][0] > side[1][0] + 0.010,
              details=f"top x=({top[0][0]:.3f},{top[1][0]:.3f})")
    ctx.check("top_is_thin_slab", abs((top[1][2] - top[0][2]) - TOP_THK) < 0.002)

    # --- Three prismatic drawers ---
    ctx.check("three_drawers_exist", len(drawer_joints) == 3)
    for n, j in drawer_joints.items():
        ctx.check(f"{n}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{n}_axis_out_front",
                  j.axis[0] > 0.99 and abs(j.axis[1]) < 0.01
                  and abs(j.axis[2]) < 0.01)
        ctx.check(f"{n}_travel_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - TRAVEL) < 1e-6,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # Drawers stacked vertically.
    f0 = ctx.part_element_world_aabb(drawers["drawer_0"], elem="front_panel")
    f1 = ctx.part_element_world_aabb(drawers["drawer_1"], elem="front_panel")
    f2 = ctx.part_element_world_aabb(drawers["drawer_2"], elem="front_panel")
    assert f0 is not None and f1 is not None and f2 is not None
    ctx.check("drawers_stacked_vertically",
              f2[0][2] > f1[1][2] and f1[0][2] > f0[1][2],
              details=f"z: d0 top={f0[1][2]:.3f}, d1 bot={f1[0][2]:.3f}, "
                      f"d1 top={f1[1][2]:.3f}, d2 bot={f2[0][2]:.3f}")

    # Each drawer has two knobs.
    for n in drawer_names:
        b0 = ctx.part_element_world_aabb(drawers[n], elem="knob_ball_0")
        b1 = ctx.part_element_world_aabb(drawers[n], elem="knob_ball_1")
        assert b0 is not None and b1 is not None
        ctx.check(f"{n}_two_knobs", True)

    # Drawer closed: front proud, tray nested.
    for n, d in drawers.items():
        face = ctx.part_element_world_aabb(d, elem="front_panel")
        tray = ctx.part_element_world_aabb(d, elem="tray_bottom")
        assert face is not None and tray is not None
        ctx.check(f"{n}_front_proud",
                  face[1][0] > FRONT_X and face[1][0] < FRONT_X + 0.030,
                  details=f"face front x={face[1][0]:.4f}")
        ctx.check(f"{n}_tray_nested",
                  tray[1][0] < FRONT_X + 0.002 and tray[0][0] > 0.02,
                  details=f"tray x=({tray[0][0]:.3f},{tray[1][0]:.3f})")
        ctx.expect_within(d, carcass, axes="y", margin=0.002,
                          name=f"{n}_within_carcass_width")

    # Drawer open pose: slides out, rear stays inserted.
    for n in ("drawer_0", "drawer_2"):
        d, j = drawers[n], drawer_joints[n]
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
            rear = ctx.part_element_world_aabb(d, elem="tray_back_wall")
            assert rear is not None
            rear_x = rear[0][0]
        assert rest is not None and out is not None
        ctx.check(f"{n}_slides_forward",
                  abs((out[0] - rest[0]) - TRAVEL) < 1e-6,
                  details=f"dx={out[0] - rest[0]:.4f}")
        ctx.check(f"{n}_retains_insertion",
                  rear_x < FRONT_X - 0.005,
                  details=f"open rear x={rear_x:.4f}")

    # --- Two revolute doors ---
    ctx.check("door_left_revolute",
              joint_door_left.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("door_right_revolute",
              joint_door_right.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("door_left_z_axis",
              abs(joint_door_left.axis[2]) > 0.99,
              details=f"axis={joint_door_left.axis}")
    ctx.check("door_right_z_axis",
              abs(joint_door_right.axis[2]) > 0.99,
              details=f"axis={joint_door_right.axis}")
    ctx.check("door_left_range",
              abs(joint_door_left.motion_limits.lower) < 1e-9
              and joint_door_left.motion_limits.upper > math.radians(80))
    ctx.check("door_right_range",
              abs(joint_door_right.motion_limits.lower) < 1e-9
              and joint_door_right.motion_limits.upper > math.radians(80))

    # Doors closed: panels cover the door zone, meeting at center.
    dl_panel = ctx.part_element_world_aabb(door_left, elem="door_panel")
    dr_panel = ctx.part_element_world_aabb(door_right, elem="door_panel")
    assert dl_panel is not None and dr_panel is not None

    # Left door covers left half of opening.
    ctx.check("door_left_covers_left_half",
              dl_panel[0][1] < -(INNER_W / 4.0) and dl_panel[1][1] > -REVEAL,
              details=f"left door y=({dl_panel[0][1]:.3f},{dl_panel[1][1]:.3f})")
    # Right door covers right half of opening.
    ctx.check("door_right_covers_right_half",
              dr_panel[0][1] < REVEAL and dr_panel[1][1] > INNER_W / 4.0,
              details=f"right door y=({dr_panel[0][1]:.3f},{dr_panel[1][1]:.3f})")

    # Doors open pose: swing outward (front face moves further +X).
    dl_closed_x = dl_panel[1][0]
    dr_closed_x = dr_panel[1][0]
    with ctx.pose({joint_door_left: math.radians(60),
                   joint_door_right: math.radians(60)}):
        dl_open = ctx.part_element_world_aabb(door_left, elem="door_panel")
        dr_open = ctx.part_element_world_aabb(door_right, elem="door_panel")
        assert dl_open is not None and dr_open is not None
        ctx.check("door_left_swings_outward",
                  dl_open[1][0] > dl_closed_x + 0.03,
                  details=f"closed={dl_closed_x:.3f}, open max_x={dl_open[1][0]:.3f}")
        ctx.check("door_right_swings_outward",
                  dr_open[1][0] > dr_closed_x + 0.03,
                  details=f"closed={dr_closed_x:.3f}, open max_x={dr_open[1][0]:.3f}")

    # --- Rotating latch at center seam ---
    ctx.check("latch_revolute",
              joint_latch.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("latch_z_axis",
              abs(joint_latch.axis[2]) > 0.99,
              details=f"axis={joint_latch.axis}")
    ctx.check("latch_90_degree_range",
              abs(joint_latch.motion_limits.lower) < 1e-9
              and abs(joint_latch.motion_limits.upper - math.radians(90)) < 0.01,
              details=f"range=({joint_latch.motion_limits.lower:.3f},"
                      f"{joint_latch.motion_limits.upper:.3f})")

    # Latch at center seam (Y near 0, at door height).
    latch_bar = ctx.part_element_world_aabb(latch, elem="latch_bar")
    assert latch_bar is not None
    latch_cy = (latch_bar[0][1] + latch_bar[1][1]) / 2.0
    latch_cz = (latch_bar[0][2] + latch_bar[1][2]) / 2.0
    ctx.check("latch_at_center_y",
              abs(latch_cy) < 0.020,
              details=f"latch cy={latch_cy:.4f}")
    ctx.check("latch_at_door_height",
              abs(latch_cz - DOOR_CZ) < 0.030,
              details=f"latch cz={latch_cz:.3f}, door_cz={DOOR_CZ:.3f}")

    # Latch rotation: horizontal at q=0, vertical at q=pi/2.
    locked_span_y = latch_bar[1][1] - latch_bar[0][1]
    locked_span_z = latch_bar[1][2] - latch_bar[0][2]
    with ctx.pose({joint_latch: math.radians(90)}):
        latch_unlocked = ctx.part_element_world_aabb(latch, elem="latch_bar")
        assert latch_unlocked is not None
        unlocked_span_y = latch_unlocked[1][1] - latch_unlocked[0][1]
        unlocked_span_z = latch_unlocked[1][2] - latch_unlocked[0][2]
    ctx.check("latch_horizontal_when_locked",
              locked_span_y > locked_span_z,
              details=f"locked y={locked_span_y:.4f}, z={locked_span_z:.4f}")
    ctx.check("latch_vertical_when_unlocked",
              unlocked_span_z > unlocked_span_y,
              details=f"unlocked y={unlocked_span_y:.4f}, z={unlocked_span_z:.4f}")

    # Latch stem contacts carcass (proof of support).
    ctx.expect_contact(carcass, latch,
                       elem_a="door_center_stile",
                       elem_b="latch_stem",
                       contact_tol=0.005,
                       name="latch_stem_contacts_carcass_stile")

    # --- Shelf boards ---
    shelf_0 = ctx.part_element_world_aabb(carcass, elem="shelf_board_0")
    shelf_1 = ctx.part_element_world_aabb(carcass, elem="shelf_board_1")
    assert shelf_0 is not None and shelf_1 is not None
    ctx.check("two_shelves_exist", True)
    ctx.check("shelves_stacked_vertically",
              shelf_1[0][2] > shelf_0[1][2],
              details=f"s0 top={shelf_0[1][2]:.3f}, s1 bot={shelf_1[0][2]:.3f}")
    ctx.check("shelf_0_inside_cabinet",
              shelf_0[1][0] < FRONT_X and shelf_0[0][0] >= WALL - 0.001,
              details=f"shelf x=({shelf_0[0][0]:.3f},{shelf_0[1][0]:.3f})")
    ctx.check("shelf_0_in_door_zone_z",
              shelf_0[0][2] > DOOR_ZONE_BOT and shelf_0[1][2] < DOOR_ZONE_TOP,
              details=f"shelf z=({shelf_0[0][2]:.3f},{shelf_0[1][2]:.3f}), "
                      f"door zone=({DOOR_ZONE_BOT:.3f},{DOOR_ZONE_TOP:.3f})")

    # Drawers are above the doors.
    ctx.check("drawers_above_doors",
              f0[0][2] > dl_panel[1][2] - 0.01,
              details=f"drawer_0 bottom z={f0[0][2]:.3f}, "
                      f"door top z={dl_panel[1][2]:.3f}")

    return ctx.report()


object_model = build_object_model()
