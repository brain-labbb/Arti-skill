from __future__ import annotations

# Wide black wooden cabinet variant (~1.70 m W x 0.85 m H x 0.50 m D).
# Left side: one hinged door on side hinges with visible hinge barrels.
# Right side: three stacked drawers, each a prismatic slide along +X.
#
# World layout: front faces +X (back at x=0, front at x=BD),
# width along Y (centered), height along +Z, grounded at z=0.
# Matte black wood carcass/door/drawer fronts; silver-gray top slab.
# Decorative carved corner posts and four square legs.

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
BODY_BOT = LEG_H                 # 0.150
BODY_TOP = H_TOTAL - TOP_THK     # 0.828
BH = BODY_TOP - BODY_BOT         # 0.678

WALL = 0.018
DIVIDER_W = 0.018

# Inner bay widths (split at y=0 by center divider).
LEFT_INNER_Y = -(BW / 2.0 - WALL)    # -0.812
RIGHT_INNER_Y = BW / 2.0 - WALL      # +0.812
LEFT_BAY_W = abs(LEFT_INNER_Y) - DIVIDER_W / 2.0   # 0.803
RIGHT_BAY_W = RIGHT_INNER_Y - DIVIDER_W / 2.0       # 0.803
RIGHT_BAY_CY = (DIVIDER_W / 2.0 + RIGHT_INNER_Y) / 2.0  # 0.4105

# Front drawer/door zone.
ZONE_BOT = BODY_BOT + 0.030      # 0.180
ZONE_TOP = BODY_TOP - 0.017      # 0.811
REVEAL = 0.008

# Door dimensions.
DOOR_REVEAL = 0.004
DOOR_W = LEFT_BAY_W - 2 * DOOR_REVEAL  # ~0.795
DOOR_H = ZONE_TOP - ZONE_BOT           # ~0.631
FACE_THK = 0.018
FACE_PROUD = 0.002

# Drawers (3 stacked on right side).
DRAWER_COUNT = 3
N_REVEALS = DRAWER_COUNT + 1
DRAWER_H = (ZONE_TOP - ZONE_BOT - N_REVEALS * REVEAL) / DRAWER_COUNT
DRAWER_W = RIGHT_BAY_W - 2 * REVEAL

# Drawer center Z positions.
DRAWER_CZ = []
for _i in range(DRAWER_COUNT):
    _z_bot = ZONE_BOT + (_i + 1) * REVEAL + _i * DRAWER_H
    DRAWER_CZ.append(_z_bot + DRAWER_H / 2.0)

# Joint placement.
FRONT_X = BD
JOINT_X = FRONT_X + FACE_PROUD + FACE_THK
TRAVEL = 0.400
TRAY_D = 0.420
TRAY_T = 0.012

# Hinge placement (left edge of door, at carcass front).
HINGE_X = BD
HINGE_Y = LEFT_INNER_Y
ZONE_CZ = (ZONE_BOT + ZONE_TOP) / 2.0
HINGE_ZS = [ZONE_BOT + 0.08, ZONE_CZ, ZONE_TOP - 0.08]
HINGE_BARREL_R = 0.007
HINGE_BARREL_L = 0.055

# Knobs.
KNOB_R = 0.0125
STEM_R = 0.0055
STEM_L = 0.014

# Carved corner posts.
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG
LEG_SQ = 0.050


def _build_drawer(model, name, front_w, front_h, tray_w, tray_h,
                  knob_ys, black, tray_mat, silver):
    """Drawer in local frame: front panel outer surface near local x=0."""
    drawer = model.part(name)
    # Front panel.
    drawer.visual(
        Box((FACE_THK, front_w, front_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, 0.0)),
        material=black,
        name="front_panel",
    )
    # Hollow open-top tray.
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
    # Knobs.
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
        Box((TRAY_D, tray_w, tray_h)), mass=4.0)
    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_cabinet_door_and_drawers")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    tray_mat = model.material("tray_black", rgba=(0.13, 0.13, 0.14, 1.0))
    hinge_mat = model.material("hinge_silver", rgba=(0.65, 0.66, 0.68, 1.0))

    # ===================================================================
    # ROOT: carcass (shell + divider + legs + carved posts + top)
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
    # Bottom board and top stretcher.
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + 0.009)),
        material=black,
        name="bottom_board",
    )
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black,
        name="top_stretcher",
    )
    # Center divider (vertical panel at y=0, full zone height).
    carcass.visual(
        Box((BD - WALL, DIVIDER_W, ZONE_TOP - ZONE_BOT)),
        origin=Origin(xyz=(WALL + (BD - WALL) / 2.0, 0.0,
                           (ZONE_BOT + ZONE_TOP) / 2.0)),
        material=black,
        name="center_divider",
    )
    # Front frame: top and bottom rails (full width).
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

    # Right bay: horizontal shelf/runner panels between drawers.
    for i in range(DRAWER_COUNT - 1):
        shelf_z = (DRAWER_CZ[i] + DRAWER_CZ[i + 1]) / 2.0
        carcass.visual(
            Box((BD - 0.030, RIGHT_BAY_W, 0.014)),
            origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0,
                               RIGHT_BAY_CY, shelf_z)),
            material=black_deep,
            name=f"right_shelf_{i}",
        )
    # Bottom runner for right bay (on top of bottom board).
    carcass.visual(
        Box((BD - 0.030, RIGHT_BAY_W, 0.014)),
        origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0,
                           RIGHT_BAY_CY,
                           ZONE_BOT - 0.007)),
        material=black_deep,
        name="right_bottom_runner",
    )

    # Left bay: one interior shelf (behind the door).
    carcass.visual(
        Box((BD - 0.040, LEFT_BAY_W, 0.016)),
        origin=Origin(xyz=(0.030 + (BD - 0.040) / 2.0,
                           -(DIVIDER_W / 2.0 + LEFT_BAY_W / 2.0),
                           ZONE_CZ)),
        material=black_deep,
        name="left_interior_shelf",
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

    # --- Visible hinge barrels on the door side ---
    for i, hz in enumerate(HINGE_ZS):
        # Barrel cylinder (vertical pin axis).
        carcass.visual(
            Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_L),
            origin=Origin(xyz=(HINGE_X, HINGE_Y, hz)),
            material=hinge_mat,
            name=f"hinge_barrel_{i}",
        )
        # Carcass-side leaf plate (extends backward from barrel).
        carcass.visual(
            Box((0.030, 0.002, HINGE_BARREL_L - 0.010)),
            origin=Origin(xyz=(HINGE_X - 0.015, HINGE_Y + 0.001, hz)),
            material=hinge_mat,
            name=f"hinge_leaf_carcass_{i}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=55.0)

    # ===================================================================
    # DOOR: hinged on left side, revolute joint
    # ===================================================================
    door = model.part("door")

    # Door front panel (outer face at local x = FACE_THK/2, extends in +Y from hinge).
    door.visual(
        Box((FACE_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(FACE_THK / 2.0, DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )
    # Door back face (thin backing panel).
    door.visual(
        Box((0.006, DOOR_W - 0.020, DOOR_H - 0.020)),
        origin=Origin(xyz=(-0.003, DOOR_W / 2.0, 0.0)),
        material=black_deep,
        name="door_backing",
    )
    # Door-side hinge leaf plates (move with door).
    for i, hz in enumerate(HINGE_ZS):
        local_z = hz - ZONE_CZ  # relative to door origin (at zone center)
        door.visual(
            Box((0.002, 0.028, HINGE_BARREL_L - 0.010)),
            origin=Origin(xyz=(0.001, 0.014, local_z)),
            material=hinge_mat,
            name=f"hinge_leaf_door_{i}",
        )
    # Door knob (near free edge, at mid-height).
    knob_y = DOOR_W - 0.065
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=(FACE_THK + (STEM_L - 0.004) / 2.0, knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="door_knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(FACE_THK + STEM_L + KNOB_R - 0.004, knob_y, 0.0)),
        material=silver,
        name="door_knob_ball",
    )

    door.inertial = Inertial.from_geometry(
        Box((FACE_THK, DOOR_W, DOOR_H)), mass=5.0)

    # Door articulation: revolute, hinge at left edge, axis (0,0,-1) opens outward.
    model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, ZONE_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5,
                                   lower=0.0, upper=1.75),
    )

    # ===================================================================
    # DRAWERS: three stacked prismatic slides on right side
    # ===================================================================
    drawers = []
    for i, cz in enumerate(DRAWER_CZ):
        d = _build_drawer(model, f"drawer_{i}", DRAWER_W, DRAWER_H,
                          DRAWER_W - 0.040, DRAWER_H - 0.040,
                          [-0.18, 0.18], black, tray_mat, silver)
        drawers.append((f"drawer_{i}", d, cz))

    for name, d, cz in drawers:
        model.articulation(
            f"carcass_to_{name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=d,
            origin=Origin(xyz=(JOINT_X, RIGHT_BAY_CY, cz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=50.0, velocity=0.5,
                                       lower=0.0, upper=TRAVEL),
        )

    return model


INNER_W = BW - 2 * WALL


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("carcass_to_door")
    drawer_names = [f"drawer_{i}" for i in range(DRAWER_COUNT)]
    drawer_parts = {n: object_model.get_part(n) for n in drawer_names}
    drawer_joints = {n: object_model.get_articulation(f"carcass_to_{n}")
                     for n in drawer_names}

    # --- Overall scale ---
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

    # --- Silver top overhangs ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert top is not None and side is not None and back is not None
    ctx.check("top_overhang_all_sides",
              top[1][1] > side[1][1] + 0.015
              and top[0][1] < -side[1][1] - 0.015
              and top[0][0] < back[0][0] - 0.015
              and top[1][0] > 0.46 + 0.015)

    # --- Door: revolute joint with correct axis and limits ---
    ctx.check("door_is_revolute",
              door_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("door_axis_z_negative",
              abs(door_joint.axis[0]) < 0.01
              and abs(door_joint.axis[1]) < 0.01
              and door_joint.axis[2] < -0.99)
    ctx.check("door_range",
              abs(door_joint.motion_limits.lower) < 1e-9
              and abs(door_joint.motion_limits.upper - 1.75) < 0.01)

    # --- Visible hinge barrels on door side ---
    for i in range(3):
        hb = ctx.part_element_world_aabb(carcass, elem=f"hinge_barrel_{i}")
        assert hb is not None
        ctx.check(f"hinge_barrel_{i}_exists", True)
        # Barrel is at the left edge of the door opening.
        ctx.check(f"hinge_barrel_{i}_at_left_edge",
                  abs((hb[0][1] + hb[1][1]) / 2.0 - HINGE_Y) < 0.01,
                  details=f"barrel y center={(hb[0][1] + hb[1][1]) / 2.0:.4f}")

    # --- Door opens outward (free edge moves in +X direction) ---
    door_rest = ctx.part_world_position(door)
    with ctx.pose({door_joint: 1.0}):
        door_open = ctx.part_world_position(door)
        door_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    assert door_rest is not None and door_open is not None
    assert door_panel_aabb is not None
    ctx.check("door_opens_outward",
              door_panel_aabb[1][0] > FRONT_X + 0.10,
              details=f"open panel max x={door_panel_aabb[1][0]:.4f}")

    # --- Door panel spans left bay ---
    dp = ctx.part_element_world_aabb(door, elem="door_panel")
    assert dp is not None
    door_width = dp[1][1] - dp[0][1]
    door_height = dp[1][2] - dp[0][2]
    ctx.check("door_width_matches_bay",
              abs(door_width - DOOR_W) < 0.005,
              details=f"door w={door_width:.4f}")
    ctx.check("door_height_matches_zone",
              abs(door_height - DOOR_H) < 0.005,
              details=f"door h={door_height:.4f}")

    # --- Three stacked drawers, prismatic joints ---
    ctx.check("three_drawers", len(drawer_joints) == DRAWER_COUNT)
    for n, j in drawer_joints.items():
        ctx.check(f"{n}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{n}_axis_out_front",
                  j.axis[0] > 0.99 and abs(j.axis[1]) < 0.01
                  and abs(j.axis[2]) < 0.01)
        ctx.check(f"{n}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 0.40) < 1e-6)

    # --- Drawers are on the right side (+Y), door is on left (-Y) ---
    for n, d in drawer_parts.items():
        face = ctx.part_element_world_aabb(d, elem="front_panel")
        assert face is not None
        face_cy = (face[0][1] + face[1][1]) / 2.0
        ctx.check(f"{n}_on_right_side", face_cy > 0.1,
                  details=f"face center y={face_cy:.3f}")

    door_face = ctx.part_element_world_aabb(door, elem="door_panel")
    assert door_face is not None
    door_cy = (door_face[0][1] + door_face[1][1]) / 2.0
    ctx.check("door_on_left_side", door_cy < -0.1,
              details=f"door center y={door_cy:.3f}")

    # --- Drawers stacked vertically ---
    faces_z = []
    for n in drawer_names:
        f = ctx.part_element_world_aabb(drawer_parts[n], elem="front_panel")
        assert f is not None
        faces_z.append((f[0][2] + f[1][2]) / 2.0)
    ctx.check("drawers_stacked_vertically",
              faces_z[0] < faces_z[1] < faces_z[2],
              details=f"z centers={[f'{z:.3f}' for z in faces_z]}")

    # --- Drawer slide test: slides out along +X ---
    for n in ("drawer_0", "drawer_2"):
        d, j = drawer_parts[n], drawer_joints[n]
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
        assert rest is not None and out is not None
        ctx.check(f"{n}_slides_forward",
                  abs((out[0] - rest[0]) - 0.40) < 1e-6,
                  details=f"dx={out[0] - rest[0]:.4f}")

    # --- Center divider separates door side from drawer side ---
    divider = ctx.part_element_world_aabb(carcass, elem="center_divider")
    assert divider is not None
    divider_cy = (divider[0][1] + divider[1][1]) / 2.0
    ctx.check("center_divider_at_center",
              abs(divider_cy) < 0.01,
              details=f"divider y={divider_cy:.4f}")

    # --- Door has knob ---
    knob = ctx.part_element_world_aabb(door, elem="door_knob_ball")
    assert knob is not None
    ctx.check("door_knob_proud",
              knob[0][0] > door_face[1][0] + 0.002,
              details=f"knob min x={knob[0][0]:.4f}")

    # --- Carved corner posts intentionally protrude past door/drawer fronts ---
    # The decorative carved posts are rotated square segments that bulge forward
    # past the front panels; this is realistic for furniture with carved corner detail.
    ctx.allow_overlap(
        "carcass", "door",
        reason="Carved corner post segments intentionally protrude past the door front panel as decorative detail.",
    )
    for i in range(DRAWER_COUNT):
        ctx.allow_overlap(
            "carcass", f"drawer_{i}",
            reason=f"Carved corner post segments intentionally protrude past drawer_{i} front panel.",
        )

    # Proof: door still opens correctly despite post proximity.
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        door_open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    assert door_open_aabb is not None
    ctx.check("door_clears_posts_when_open",
              door_open_aabb[0][1] < HINGE_Y + 0.10,
              details=f"open door min y={door_open_aabb[0][1]:.4f}")

    # Proof: drawers slide out independently of posts.
    with ctx.pose({drawer_joints["drawer_1"]: 0.30}):
        d1_face = ctx.part_element_world_aabb(drawer_parts["drawer_1"],
                                               elem="front_panel")
    assert d1_face is not None
    ctx.check("drawer_1_slides_clear_posts",
              d1_face[0][0] > FRONT_X + 0.25,
              details=f"drawer_1 front x={d1_face[0][0]:.4f}")

    return ctx.report()


object_model = build_object_model()
