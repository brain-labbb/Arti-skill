from __future__ import annotations

# Tall two-door cabinet with raised plinth base (~0.90 m W x 1.60 m H x 0.45 m D).
#
# World layout: front faces +X (back of body at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on a raised
# plinth base ~0.10 m tall. Matte black wood carcass, doors, and drawer
# fronts; a thin smooth silver-gray top slab overhangs the body ~0.02 m
# on all sides.
#
# Lower section: two wide drawers on independent PRISMATIC joints sliding
# out along +X, range 0 to 0.38 m, each a hollow open-top tray behind a
# flat front panel with a centered silver ball knob.
#
# Upper section: two doors on REVOLUTE joints, hinged at the outer vertical
# edges with visible cylindrical hinge barrels. Left door hinged on the
# left side (axis (0,0,-1)), right door on the right (axis (0,0,1)), both
# opening outward (+X), range 0 to ~1.5 rad. Each door carries one silver
# ball knob near its free edge.

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
W_TOTAL = 0.90           # overall width including top overhang (Y)
D_TOTAL = 0.45           # overall depth including top overhang (X)
H_TOTAL = 1.60           # overall height (Z)
OVERHANG = 0.020         # top slab overhang on all sides
TOP_THK = 0.022          # silver top slab thickness

BW = W_TOTAL - 2 * OVERHANG      # body width 0.86
BD = D_TOTAL - 2 * OVERHANG      # body depth 0.41
PLINTH_H = 0.100                 # raised plinth base height
PLINTH_INSET = 0.018             # plinth inset from body edges
BODY_BOT = PLINTH_H              # body bottom z (0.10)
BODY_TOP = H_TOTAL - TOP_THK     # body top z (1.578)
BH = BODY_TOP - BODY_BOT         # body height ~1.478

WALL = 0.018                     # carcass panel thickness
INNER_W = BW - 2 * WALL          # inner width ~0.824
INNER_D = BD - WALL              # inner depth ~0.392

REVEAL = 0.008                   # gap between adjacent fronts
FACE_THK = 0.018                 # door and drawer front panel thickness

# --- Drawer zone (lower section) ---
DRAWER_RAIL_BOT = 0.025          # bottom rail height above body bottom
DRAWER_ZONE_BOT = BODY_BOT + DRAWER_RAIL_BOT  # 0.125
DRAWER_FH = 0.210                # each drawer front height
DRAWER_ZONE_TOP = DRAWER_ZONE_BOT + 2 * DRAWER_FH + REVEAL  # 0.553

# Divider between drawers and doors
DIVIDER_H = 0.025
DOOR_ZONE_BOT = DRAWER_ZONE_TOP + DIVIDER_H   # 0.578
DOOR_ZONE_TOP = BODY_TOP - 0.020              # 1.558
DOOR_H = DOOR_ZONE_TOP - DOOR_ZONE_BOT        # ~0.980
DOOR_W = (INNER_W - REVEAL) / 2.0             # ~0.408

# Drawer front centers (Z)
CZ_DRAWER_0 = DRAWER_ZONE_BOT + DRAWER_FH / 2.0
CZ_DRAWER_1 = DRAWER_ZONE_BOT + DRAWER_FH + REVEAL + DRAWER_FH / 2.0

# Door center Z
DOOR_CZ = (DOOR_ZONE_BOT + DOOR_ZONE_TOP) / 2.0

# Hinge locations
HINGE_X = BD                             # at carcass front face
LEFT_HINGE_Y = -(BW / 2.0 - WALL)       # inner face of left side panel
RIGHT_HINGE_Y = (BW / 2.0 - WALL)       # inner face of right side panel

# Hinge barrel dimensions
BARREL_R = 0.007
BARREL_LEN = 0.048
BARREL_Z_FRAC = [0.12, 0.50, 0.88]      # fractional heights for 3 barrels

# Drawer dimensions
DRAWER_FW = INNER_W - 0.010             # drawer front width
TRAVEL = 0.380
TRAY_D = 0.395                           # tray depth (retained when open)
TRAY_T = 0.012                           # tray panel thickness

# Knobs: polished silver ball on a short stem
KNOB_R = 0.012
STEM_R = 0.005
STEM_L = 0.014


def _build_drawer(model, name, front_w, front_h, tray_w, tray_h,
                  black, tray_mat, silver):
    """Drawer in local frame: front panel outer surface at local x=0,
    panel spans x in [-FACE_THK, 0], hollow open-top tray extends toward -X."""
    drawer = model.part(name)

    # Flat matte-black front panel
    drawer.visual(
        Box((FACE_THK, front_w, front_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, 0.0)),
        material=black,
        name="front_panel",
    )

    # Hollow open-top tray
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
    tray_back_x = -(FACE_THK + TRAY_D)
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

    # Single centered silver ball knob
    drawer.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0, 0.0, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    drawer.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004, 0.0, 0.0)),
        material=silver,
        name="knob_ball",
    )

    drawer.inertial = Inertial.from_geometry(
        Box((TRAY_D, tray_w, tray_h)), mass=3.5)
    return drawer


def _build_door(model, name, door_w, door_h, hinge_y_sign,
                black, silver):
    """Door in local frame: hinge edge at local origin, panel extends
    in hinge_y_sign * Y direction. Front face at local x=0 to x=FACE_THK."""
    door = model.part(name)

    # Flat matte-black door panel
    door.visual(
        Box((FACE_THK, door_w, door_h)),
        origin=Origin(xyz=(FACE_THK / 2.0, hinge_y_sign * door_w / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )

    # Raised panel detail: a shallow rectangular recess on the front face
    recess_thk = 0.004
    recess_w = door_w - 0.08
    recess_h = door_h - 0.12
    door.visual(
        Box((recess_thk, recess_w, recess_h)),
        origin=Origin(xyz=(FACE_THK + recess_thk / 2.0,
                           hinge_y_sign * door_w / 2.0, 0.0)),
        material=black,
        name="panel_recess",
    )

    # Silver ball knob near the free edge
    knob_y = hinge_y_sign * (door_w - 0.045)
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=(FACE_THK + (STEM_L - 0.004) / 2.0, knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(FACE_THK + STEM_L + KNOB_R - 0.004, knob_y, 0.0)),
        material=silver,
        name="knob_ball",
    )

    door.inertial = Inertial.from_geometry(
        Box((FACE_THK, door_w, door_h)), mass=5.0)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_two_door_cabinet")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    tray_mat = model.material("tray_black", rgba=(0.13, 0.13, 0.14, 1.0))
    hinge_mat = model.material("hinge_brass", rgba=(0.55, 0.50, 0.35, 1.0))

    # ===================================================================
    # ROOT: carcass (hollow shell + plinth + shelves + hinge barrels + top)
    # ===================================================================
    carcass = model.part("carcass")

    # --- Raised plinth base ---
    plinth_w = BW - 2 * PLINTH_INSET
    plinth_d = BD - PLINTH_INSET
    carcass.visual(
        Box((plinth_d, plinth_w, PLINTH_H)),
        origin=Origin(xyz=(plinth_d / 2.0, 0.0, PLINTH_H / 2.0)),
        material=black_deep,
        name="plinth_base",
    )

    # --- Side panels ---
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD, WALL, BH)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                               BODY_BOT + BH / 2.0)),
            material=black,
            name=f"side_panel_{tag}",
        )

    # --- Back panel ---
    carcass.visual(
        Box((WALL, INNER_W, BH)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=black,
        name="back_panel",
    )

    # --- Bottom board ---
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + 0.009)),
        material=black,
        name="bottom_board",
    )

    # --- Top stretcher board ---
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black,
        name="top_stretcher",
    )

    # --- Front bottom rail (below drawers) ---
    carcass.visual(
        Box((WALL, INNER_W, DRAWER_RAIL_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           BODY_BOT + DRAWER_RAIL_BOT / 2.0)),
        material=black,
        name="front_bottom_rail",
    )

    # --- Front top rail (above doors) ---
    top_rail_h = BODY_TOP - DOOR_ZONE_TOP
    carcass.visual(
        Box((WALL, INNER_W, top_rail_h)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           DOOR_ZONE_TOP + top_rail_h / 2.0)),
        material=black,
        name="front_top_rail",
    )

    # --- Horizontal divider between drawer zone and door zone ---
    carcass.visual(
        Box((BD - 0.020, INNER_W, DIVIDER_H)),
        origin=Origin(xyz=(0.020 + (BD - 0.020) / 2.0, 0.0,
                           DRAWER_ZONE_TOP + DIVIDER_H / 2.0)),
        material=black,
        name="zone_divider",
    )

    # --- Center stile between the two doors ---
    carcass.visual(
        Box((WALL, 0.030, DOOR_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, DOOR_CZ)),
        material=black,
        name="door_center_stile",
    )

    # --- Front side stiles flanking the door zone ---
    stile_w = BW / 2.0 - (INNER_W / 2.0) + 0.002
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, stile_w, DOOR_H)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (BW / 2.0 - stile_w / 2.0),
                               DOOR_CZ)),
            material=black,
            name=f"door_side_stile_{tag}",
        )

    # --- Drawer runner panels (dust panels for drawers to ride on) ---
    for cz in (CZ_DRAWER_0, CZ_DRAWER_1):
        tray_bot_z = cz + (-DRAWER_FH / 2.0 + 0.012)
        carcass.visual(
            Box((BD - 0.030, INNER_W, 0.014)),
            origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, 0.0,
                               tray_bot_z - 0.007)),
            material=black_deep,
            name=f"drawer_runner_z{cz:.2f}",
        )

    # --- Interior shelf behind the doors ---
    shelf_z = DOOR_CZ + 0.01
    carcass.visual(
        Box((INNER_D - 0.010, INNER_W - 0.004, 0.016)),
        origin=Origin(xyz=(WALL + (INNER_D - 0.010) / 2.0, 0.0, shelf_z)),
        material=black_deep,
        name="interior_shelf",
    )

    # --- Silver-gray top slab with overhang ---
    carcass.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=silver_top,
        name="top_slab",
    )

    # --- Hinge barrels: 3 per door, visible cylinders at the hinge line ---
    barrel_zs = [DOOR_ZONE_BOT + frac * DOOR_H for frac in BARREL_Z_FRAC]
    for i, bz in enumerate(barrel_zs):
        # Left door hinge barrels
        carcass.visual(
            Cylinder(radius=BARREL_R, length=BARREL_LEN),
            origin=Origin(xyz=(HINGE_X, LEFT_HINGE_Y, bz)),
            material=hinge_mat,
            name=f"hinge_barrel_left_{i}",
        )
        # Right door hinge barrels
        carcass.visual(
            Cylinder(radius=BARREL_R, length=BARREL_LEN),
            origin=Origin(xyz=(HINGE_X, RIGHT_HINGE_Y, bz)),
            material=hinge_mat,
            name=f"hinge_barrel_right_{i}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=45.0)

    # ===================================================================
    # DRAWERS: two independent PRISMATIC slides along +X
    # ===================================================================
    drawer_0 = _build_drawer(model, "drawer_0", DRAWER_FW, DRAWER_FH,
                             INNER_W - 0.020, DRAWER_FH - 0.030,
                             black, tray_mat, silver)
    drawer_1 = _build_drawer(model, "drawer_1", DRAWER_FW, DRAWER_FH,
                             INNER_W - 0.020, DRAWER_FH - 0.030,
                             black, tray_mat, silver)

    # Joint X: front of carcass, accounting for proud front panel
    JOINT_X = BD + FACE_THK

    model.articulation(
        "carcass_to_drawer_0",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=drawer_0,
        origin=Origin(xyz=(JOINT_X, 0.0, CZ_DRAWER_0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.5,
                                   lower=0.0, upper=TRAVEL),
    )
    model.articulation(
        "carcass_to_drawer_1",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=drawer_1,
        origin=Origin(xyz=(JOINT_X, 0.0, CZ_DRAWER_1)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.5,
                                   lower=0.0, upper=TRAVEL),
    )

    # ===================================================================
    # DOORS: two REVOLUTE hinges at outer vertical edges
    # ===================================================================
    left_door = _build_door(model, "left_door", DOOR_W, DOOR_H,
                            1, black, silver)   # extends +Y from hinge
    right_door = _build_door(model, "right_door", DOOR_W, DOOR_H,
                             -1, black, silver)  # extends -Y from hinge

    # Left door: hinged at left side, axis (0,0,-1) → positive q opens outward
    model.articulation(
        "carcass_to_left_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(HINGE_X, LEFT_HINGE_Y, DOOR_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0,
                                   lower=0.0, upper=1.50),
    )

    # Right door: hinged at right side, axis (0,0,1) → positive q opens outward
    model.articulation(
        "carcass_to_right_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(HINGE_X, RIGHT_HINGE_Y, DOOR_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0,
                                   lower=0.0, upper=1.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    drawer_0 = object_model.get_part("drawer_0")
    drawer_1 = object_model.get_part("drawer_1")

    left_hinge = object_model.get_articulation("carcass_to_left_door")
    right_hinge = object_model.get_articulation("carcass_to_right_door")
    drawer_0_joint = object_model.get_articulation("carcass_to_drawer_0")
    drawer_1_joint = object_model.get_articulation("carcass_to_drawer_1")

    # --- Grounding and overall scale ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("plinth_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_090", abs(width_y - 0.90) < 0.02,
              details=f"w={width_y:.3f}")
    ctx.check("depth_045", abs(depth_x - 0.45) < 0.04,
              details=f"d={depth_x:.3f}")
    ctx.check("height_160", abs(height_z - 1.60) < 0.02,
              details=f"h={height_z:.3f}")

    # --- Raised plinth base ---
    plinth = ctx.part_element_world_aabb(carcass, elem="plinth_base")
    assert plinth is not None
    ctx.check("plinth_at_ground",
              abs(plinth[0][2]) < 0.003,
              details=f"plinth bottom z={plinth[0][2]:.4f}")
    ctx.check("plinth_height",
              abs((plinth[1][2] - plinth[0][2]) - PLINTH_H) < 0.005,
              details=f"plinth h={plinth[1][2] - plinth[0][2]:.3f}")

    # --- Silver top slab overhangs ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    assert top is not None and side is not None
    ctx.check("top_overhang_sides",
              top[1][1] > side[1][1] + 0.015
              and top[0][1] < -side[1][1] - 0.015,
              details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f})")
    ctx.check("top_is_thin_slab",
              abs((top[1][2] - top[0][2]) - TOP_THK) < 0.002)

    # --- Two doors: REVOLUTE joints ---
    ctx.check("left_door_revolute",
              left_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("right_door_revolute",
              right_hinge.articulation_type == ArticulationType.REVOLUTE)

    # Door axes: left opens with (0,0,-1), right with (0,0,1)
    ctx.check("left_door_axis",
              abs(left_hinge.axis[2] - (-1.0)) < 0.01
              and abs(left_hinge.axis[0]) < 0.01
              and abs(left_hinge.axis[1]) < 0.01)
    ctx.check("right_door_axis",
              abs(right_hinge.axis[2] - 1.0) < 0.01
              and abs(right_hinge.axis[0]) < 0.01
              and abs(right_hinge.axis[1]) < 0.01)

    # Door limits: 0 to 1.5 rad
    for name, j in (("left", left_hinge), ("right", right_hinge)):
        ctx.check(f"{name}_door_limits",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 1.50) < 0.01,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Visible hinge barrels on carcass ---
    for side_tag in ("left", "right"):
        for i in range(3):
            hb = ctx.part_element_world_aabb(carcass,
                                              elem=f"hinge_barrel_{side_tag}_{i}")
            assert hb is not None
            barrel_diam = max(hb[1][0] - hb[0][0], hb[1][1] - hb[0][1])
            ctx.check(f"hinge_barrel_{side_tag}_{i}_visible",
                      barrel_diam > 2 * BARREL_R - 0.002,
                      details=f"barrel diam={barrel_diam:.4f}")

    # Hinge barrels at the correct hinge Y positions
    hb_left = ctx.part_element_world_aabb(carcass, elem="hinge_barrel_left_1")
    hb_right = ctx.part_element_world_aabb(carcass, elem="hinge_barrel_right_1")
    assert hb_left is not None and hb_right is not None
    left_barrel_cy = (hb_left[0][1] + hb_left[1][1]) / 2.0
    right_barrel_cy = (hb_right[0][1] + hb_right[1][1]) / 2.0
    ctx.check("left_barrels_at_left_edge",
              left_barrel_cy < -0.30,
              details=f"left barrel cy={left_barrel_cy:.3f}")
    ctx.check("right_barrels_at_right_edge",
              right_barrel_cy > 0.30,
              details=f"right barrel cy={right_barrel_cy:.3f}")

    # --- Doors open outward (positive q → +X motion of free edge) ---
    left_panel_rest = ctx.part_element_world_aabb(left_door, elem="door_panel")
    assert left_panel_rest is not None
    with ctx.pose({left_hinge: 1.0}):
        left_panel_open = ctx.part_element_world_aabb(left_door, elem="door_panel")
    assert left_panel_open is not None
    ctx.check("left_door_opens_outward",
              left_panel_open[1][0] > left_panel_rest[1][0] + 0.05,
              details=f"rest max_x={left_panel_rest[1][0]:.3f}, open max_x={left_panel_open[1][0]:.3f}")

    right_panel_rest = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert right_panel_rest is not None
    with ctx.pose({right_hinge: 1.0}):
        right_panel_open = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert right_panel_open is not None
    ctx.check("right_door_opens_outward",
              right_panel_open[1][0] > right_panel_rest[1][0] + 0.05,
              details=f"rest max_x={right_panel_rest[1][0]:.3f}, open max_x={right_panel_open[1][0]:.3f}")

    # --- Two drawers: PRISMATIC joints ---
    ctx.check("drawer_0_prismatic",
              drawer_0_joint.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("drawer_1_prismatic",
              drawer_1_joint.articulation_type == ArticulationType.PRISMATIC)

    for name, j in (("drawer_0", drawer_0_joint), ("drawer_1", drawer_1_joint)):
        ctx.check(f"{name}_axis_outward",
                  j.axis[0] > 0.99 and abs(j.axis[1]) < 0.01
                  and abs(j.axis[2]) < 0.01)
        ctx.check(f"{name}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - TRAVEL) < 1e-6,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Drawers slide forward, rear stays inserted ---
    carcass_front = BD
    for name, d, j in (("drawer_0", drawer_0, drawer_0_joint),
                        ("drawer_1", drawer_1, drawer_1_joint)):
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
            rear = ctx.part_element_world_aabb(d, elem="tray_back_wall")
            assert rear is not None
            rear_x = rear[0][0]
        assert rest is not None and out is not None
        ctx.check(f"{name}_slides_forward",
                  abs((out[0] - rest[0]) - TRAVEL) < 1e-5,
                  details=f"dx={out[0] - rest[0]:.4f}")
        ctx.check(f"{name}_retains_insertion",
                  rear_x < carcass_front - 0.005,
                  details=f"open rear x={rear_x:.4f}")

    # --- Drawers stacked vertically with reveal ---
    f0 = ctx.part_element_world_aabb(drawer_0, elem="front_panel")
    f1 = ctx.part_element_world_aabb(drawer_1, elem="front_panel")
    assert f0 is not None and f1 is not None
    ctx.check("drawers_stacked",
              f1[0][2] > f0[1][2] - 0.002,
              details=f"d0 top={f0[1][2]:.3f}, d1 bot={f1[0][2]:.3f}")
    rev_v = f1[0][2] - f0[1][2]
    ctx.check("drawer_reveal",
              0.004 < rev_v < 0.020,
              details=f"reveal={rev_v:.4f}")

    # --- Drawer zone below door zone ---
    door_panel = ctx.part_element_world_aabb(left_door, elem="door_panel")
    assert door_panel is not None
    ctx.check("drawers_below_doors",
              f1[1][2] < door_panel[0][2] + 0.01,
              details=f"drawer top={f1[1][2]:.3f}, door bot={door_panel[0][2]:.3f}")

    # --- Allow intentional hinge barrel overlap with door edges ---
    ctx.allow_overlap(
        carcass, left_door,
        elem_a="hinge_barrel_left_0", elem_b="door_panel",
        reason="Hinge barrel sits at the door's hinged edge, partially embedded in the door thickness.",
    )
    ctx.allow_overlap(
        carcass, left_door,
        elem_a="hinge_barrel_left_1", elem_b="door_panel",
        reason="Hinge barrel sits at the door's hinged edge, partially embedded in the door thickness.",
    )
    ctx.allow_overlap(
        carcass, left_door,
        elem_a="hinge_barrel_left_2", elem_b="door_panel",
        reason="Hinge barrel sits at the door's hinged edge, partially embedded in the door thickness.",
    )
    ctx.allow_overlap(
        carcass, right_door,
        elem_a="hinge_barrel_right_0", elem_b="door_panel",
        reason="Hinge barrel sits at the door's hinged edge, partially embedded in the door thickness.",
    )
    ctx.allow_overlap(
        carcass, right_door,
        elem_a="hinge_barrel_right_1", elem_b="door_panel",
        reason="Hinge barrel sits at the door's hinged edge, partially embedded in the door thickness.",
    )
    ctx.allow_overlap(
        carcass, right_door,
        elem_a="hinge_barrel_right_2", elem_b="door_panel",
        reason="Hinge barrel sits at the door's hinged edge, partially embedded in the door thickness.",
    )

    # Prove hinge barrels are near the door edges (contact proof)
    for side, door_part, barrel_prefix in (
        ("left", left_door, "hinge_barrel_left"),
        ("right", right_door, "hinge_barrel_right"),
    ):
        for i in range(3):
            ctx.expect_contact(
                carcass, door_part,
                elem_a=f"{barrel_prefix}_{i}", elem_b="door_panel",
                contact_tol=0.010,
                name=f"{barrel_prefix}_{i}_near_door_edge",
            )

    return ctx.report()


object_model = build_object_model()
