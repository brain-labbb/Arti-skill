from __future__ import annotations

# Variant 13: wide black wooden cabinet (~1.70 m W x 0.85 m H x 0.50 m D).
# One side has a hinged door (revolute joint, visible hinge barrels),
# the other side has three stacked drawers (prismatic joints, pull handles).
#
# World layout: front faces +X (back at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four
# square legs ~0.15 m tall. Matte black wood carcass; silver-gray top.
# Decorative carved corner posts with stacked spiral/zigzag segments.
#
# Left side (+Y): full-height hinged door, hinge at outer +Y edge,
# revolute around Z, opens outward. Visible hinge barrels on the carcass.
# Right side (-Y): three stacked drawers, each sliding along +X,
# with bar-style pull handles.

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

BW = W_TOTAL - 2 * OVERHANG       # body width 1.66
BD = D_TOTAL - 2 * OVERHANG       # body depth 0.46
LEG_H = 0.150
BODY_BOT = LEG_H
BODY_TOP = H_TOTAL - TOP_THK      # 0.828
BH = BODY_TOP - BODY_BOT          # body height

WALL = 0.018
INNER_W = BW - 2 * WALL

# Front drawer/door zone.
ZONE_BOT = BODY_BOT + 0.030       # 0.180
ZONE_TOP = BODY_TOP - 0.017       # 0.811
ZONE_H = ZONE_TOP - ZONE_BOT      # ~0.631
REVEAL = 0.008

# Front opening half-width (between carved posts).
OPEN_HW = 0.762

FACE_THK = 0.018
FACE_PROUD = 0.0005
FRONT_X = BD                       # 0.46
SLAB_BACK_X = FRONT_X + FACE_PROUD
JOINT_X = SLAB_BACK_X + FACE_THK  # drawer/door frame origin X

TRAVEL = 0.400
TRAY_D = 0.420
TRAY_T = 0.012

# Carved corner posts.
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG

LEG_SQ = 0.050

# Door dimensions.
DOOR_W = OPEN_HW - REVEAL          # ~0.754
DOOR_H = ZONE_H - REVEAL           # full zone minus top reveal
DOOR_CZ = (ZONE_BOT + ZONE_TOP) / 2.0

# Drawer dimensions (3 stacked on -Y side).
DRAWER_W = OPEN_HW - REVEAL        # ~0.754
DRAWER_H = (ZONE_H - 2 * REVEAL) / 3.0  # ~0.205
DRAWER_CY = -OPEN_HW / 2.0         # centered on the drawer side

# Drawer Z centers.
DRAWER_CZ = [
    ZONE_BOT + REVEAL + DRAWER_H / 2.0,
    ZONE_BOT + REVEAL + DRAWER_H + REVEAL + DRAWER_H / 2.0,
    ZONE_BOT + REVEAL + 2 * (DRAWER_H + REVEAL) + DRAWER_H / 2.0,
]

# Hinge barrel dimensions.
HINGE_BARREL_R = 0.008
HINGE_BARREL_H = 0.040
HINGE_Z_FRAC = [0.2, 0.8]         # fraction of zone height for hinge positions

# Pull handle dimensions (bar-style).
HANDLE_BAR_R = 0.005
HANDLE_BAR_L = 0.080
HANDLE_STEM_R = 0.004
HANDLE_STEM_L = 0.018
HANDLE_SPAN = 0.050                # distance between stem centers


def _build_drawer(model, name, front_w, front_h, tray_w, tray_h,
                  black, tray_mat, silver):
    """Drawer: front panel at local x=0, hollow tray extends -X, bar pull handle."""
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

    # Bar-style pull handle (horizontal bar on two stems).
    # Handle centered on the front panel, oriented horizontally.
    for tag, dy in (("0", -HANDLE_SPAN / 2.0), ("1", HANDLE_SPAN / 2.0)):
        drawer.visual(
            Cylinder(radius=HANDLE_STEM_R, length=HANDLE_STEM_L + 0.004),
            origin=Origin(xyz=(HANDLE_STEM_L / 2.0 - 0.002 + FACE_PROUD, dy, 0.0),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=silver,
            name=f"handle_stem_{tag}",
        )
    # Horizontal bar connecting the two stems (center embeds into stem tips).
    drawer.visual(
        Cylinder(radius=HANDLE_BAR_R, length=HANDLE_BAR_L),
        origin=Origin(xyz=(HANDLE_STEM_L + FACE_PROUD, 0.0, 0.0),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="handle_bar",
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

    # Front frame rails (below and above the zone).
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

    # Front side stiles (between opening edge and side panels).
    stile_w = BW / 2.0 - OPEN_HW + 0.004
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, stile_w, ZONE_H)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (BW / 2.0 - stile_w / 2.0),
                               (ZONE_BOT + ZONE_TOP) / 2.0)),
            material=black,
            name=f"front_side_stile_{tag}",
        )

    # Center vertical divider at Y=0, separating door side from drawer side.
    carcass.visual(
        Box((BD - WALL, WALL, ZONE_H)),
        origin=Origin(xyz=((WALL + BD - WALL) / 2.0, 0.0,
                           (ZONE_BOT + ZONE_TOP) / 2.0)),
        material=black,
        name="center_divider",
    )

    # Dust panels (drawer runners) on the drawer side (-Y) only.
    for i, cz in enumerate(DRAWER_CZ):
        tray_bot_z = cz + (-DRAWER_H / 2.0 + 0.012)
        carcass.visual(
            Box((BD - 0.030, OPEN_HW - WALL, 0.014)),
            origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0,
                               -(OPEN_HW + WALL) / 2.0,
                               tray_bot_z - 0.007)),
            material=black_deep,
            name=f"drawer_dust_panel_{i}",
        )

    # Shelf behind door (one shelf in the door compartment).
    shelf_z = (ZONE_BOT + ZONE_TOP) / 2.0
    carcass.visual(
        Box((BD - WALL - 0.020, OPEN_HW - WALL, 0.016)),
        origin=Origin(xyz=((WALL + BD - 0.020) / 2.0,
                           (WALL + OPEN_HW) / 2.0,
                           shelf_z)),
        material=black_deep,
        name="door_shelf",
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

    # --- Visible hinge barrels on the carcass at the door edge (+Y side) ---
    hinge_y = OPEN_HW  # hinge line at outer edge of door opening
    for i, frac in enumerate(HINGE_Z_FRAC):
        hz = ZONE_BOT + frac * ZONE_H
        carcass.visual(
            Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_H),
            origin=Origin(xyz=(BD, hinge_y, hz)),
            material=silver,
            name=f"hinge_barrel_{i}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=60.0)

    # ===================================================================
    # DOOR: hinged panel on the +Y side, revolute around Z
    # ===================================================================
    door = model.part("door")

    # Door panel: origin at hinge edge, panel extends in -Y.
    door.visual(
        Box((FACE_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(-FACE_THK / 2.0, -DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )

    # Door handle: bar-style pull on the free edge (-Y side of door local).
    door_handle_y = -(DOOR_W - 0.060)  # near the free edge
    for tag, dz in (("0", -HANDLE_SPAN / 2.0), ("1", HANDLE_SPAN / 2.0)):
        door.visual(
            Cylinder(radius=HANDLE_STEM_R, length=HANDLE_STEM_L + 0.004),
            origin=Origin(xyz=(HANDLE_STEM_L / 2.0 - 0.002 + FACE_PROUD,
                               door_handle_y, dz),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=silver,
            name=f"door_handle_stem_{tag}",
        )
    # Vertical bar connecting the two vertically separated stems.
    door.visual(
        Cylinder(radius=HANDLE_BAR_R, length=HANDLE_BAR_L),
        origin=Origin(xyz=(HANDLE_STEM_L + FACE_PROUD,
                           door_handle_y, 0.0),
                      rpy=(0.0, 0.0, 0.0)),
        material=silver,
        name="door_handle_bar",
    )

    # Hinge knuckle plates on the door edge (at hinge side, +Y local).
    for i, frac in enumerate(HINGE_Z_FRAC):
        hz_local = (ZONE_BOT + frac * ZONE_H) - DOOR_CZ
        door.visual(
            Box((0.006, 0.022, HINGE_BARREL_H * 0.8)),
            origin=Origin(xyz=(-FACE_THK / 2.0, -0.011, hz_local)),
            material=silver,
            name=f"door_hinge_plate_{i}",
        )

    door.inertial = Inertial.from_geometry(
        Box((FACE_THK, DOOR_W, DOOR_H)), mass=3.5)

    # Door articulation: revolute around Z at the hinge line.
    model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(JOINT_X, hinge_y, DOOR_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0,
                                   lower=0.0, upper=math.pi / 2.0),
    )

    # ===================================================================
    # DRAWERS: three stacked on the -Y side, prismatic along +X
    # ===================================================================
    drawers = []
    for i in range(3):
        d = _build_drawer(
            model, f"drawer_{i}", DRAWER_W, DRAWER_H,
            DRAWER_W - 0.040, DRAWER_H - 0.040,
            black, tray_mat, silver,
        )
        drawers.append((f"drawer_{i}", d, DRAWER_CY, DRAWER_CZ[i]))

    for name, d, cy, cz in drawers:
        model.articulation(
            f"carcass_to_{name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=d,
            origin=Origin(xyz=(JOINT_X, cy, cz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=50.0, velocity=0.5,
                                       lower=0.0, upper=TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    drawer_names = [f"drawer_{i}" for i in range(3)]
    drawers = {n: object_model.get_part(n) for n in drawer_names}
    door_joint = object_model.get_articulation("carcass_to_door")
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

    # --- Silver top overhang ---
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

    # --- Door: revolute joint with visible hinge barrels ---
    ctx.check("door_joint_is_revolute",
              door_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("door_axis_is_z",
              abs(door_joint.axis[2]) > 0.99
              and abs(door_joint.axis[0]) < 0.01
              and abs(door_joint.axis[1]) < 0.01)
    ctx.check("door_range_0_to_90deg",
              abs(door_joint.motion_limits.lower) < 1e-9
              and abs(door_joint.motion_limits.upper - math.pi / 2.0) < 0.01,
              details=f"range=({door_joint.motion_limits.lower:.3f},{door_joint.motion_limits.upper:.3f})")

    # Hinge barrels visible on carcass at the door edge.
    for i in range(len(HINGE_Z_FRAC)):
        hb = ctx.part_element_world_aabb(carcass, elem=f"hinge_barrel_{i}")
        assert hb is not None
        ctx.check(f"hinge_barrel_{i}_at_door_edge",
                  abs(hb[0][1] - OPEN_HW + HINGE_BARREL_R) < 0.005,
                  details=f"barrel min y={hb[0][1]:.4f}")

    # Door panel exists and has correct proportions.
    door_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    assert door_panel is not None
    door_panel_w = door_panel[1][1] - door_panel[0][1]
    door_panel_h = door_panel[1][2] - door_panel[0][2]
    ctx.check("door_panel_width", abs(door_panel_w - DOOR_W) < 0.005,
              details=f"w={door_panel_w:.3f}")
    ctx.check("door_panel_height", abs(door_panel_h - DOOR_H) < 0.005,
              details=f"h={door_panel_h:.3f}")

    # Door handle present.
    handle_bar = ctx.part_element_world_aabb(door, elem="door_handle_bar")
    assert handle_bar is not None
    ctx.check("door_handle_proud_of_panel",
              handle_bar[0][0] > door_panel[1][0] + 0.005,
              details=f"handle min x={handle_bar[0][0]:.4f}, panel max x={door_panel[1][0]:.4f}")

    # Door opens outward: at positive angle, free edge moves toward +X.
    rest_pos = ctx.part_world_position(door)
    with ctx.pose({door_joint: math.pi / 4.0}):
        open_pos = ctx.part_world_position(door)
    assert rest_pos is not None and open_pos is not None
    # The door origin is at the hinge line, which doesn't move much.
    # Check the door panel AABB moves outward instead.
    with ctx.pose({door_joint: 0.0}):
        closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: math.pi / 4.0}):
        open_aabb = ctx.part_world_aabb(door)
    assert closed_aabb is not None and open_aabb is not None
    ctx.check("door_opens_outward",
              open_aabb[1][0] > closed_aabb[1][0] + 0.05,
              details=f"closed max x={closed_aabb[1][0]:.3f}, open max x={open_aabb[1][0]:.3f}")

    # --- Drawers: three prismatic joints ---
    ctx.check("three_drawers", len(drawer_joints) == 3)
    for n, j in drawer_joints.items():
        ctx.check(f"{n}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{n}_axis_out_front",
                  j.axis[0] > 0.99 and abs(j.axis[1]) < 0.01
                  and abs(j.axis[2]) < 0.01)
        ctx.check(f"{n}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 0.40) < 1e-6,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # Drawers are on the -Y side (opposite from door).
    for n, d in drawers.items():
        face = ctx.part_element_world_aabb(d, elem="front_panel")
        assert face is not None
        face_cy = (face[0][1] + face[1][1]) / 2.0
        ctx.check(f"{n}_on_negative_y_side", face_cy < -0.05,
                  details=f"face center y={face_cy:.3f}")

    # Drawers have bar pull handles.
    for n, d in drawers.items():
        bar = ctx.part_element_world_aabb(d, elem="handle_bar")
        assert bar is not None
        ctx.check(f"{n}_has_pull_handle",
                  bar[1][0] > face[1][0] + 0.005,
                  details=f"bar max x={bar[1][0]:.4f}, panel max x={face[1][0]:.4f}")

    # Drawers are stacked vertically with reveals.
    faces = [ctx.part_element_world_aabb(drawers[f"drawer_{i}"], elem="front_panel")
             for i in range(3)]
    assert all(f is not None for f in faces)
    ctx.check("drawers_stacked_vertically",
              faces[2][0][2] > faces[1][1][2] > faces[0][1][2],
              details=f"z ranges: bottom=({faces[0][0][2]:.3f},{faces[0][1][2]:.3f}), "
                      f"mid=({faces[1][0][2]:.3f},{faces[1][1][2]:.3f}), "
                      f"top=({faces[2][0][2]:.3f},{faces[2][1][2]:.3f})")

    # Drawers slide forward and retain insertion.
    for n in drawer_names:
        d, j = drawers[n], drawer_joints[n]
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
            rear = ctx.part_element_world_aabb(d, elem="tray_back_wall")
            assert rear is not None
            rear_x = rear[0][0]
        assert rest is not None and out is not None
        ctx.check(f"{n}_slides_forward",
                  abs((out[0] - rest[0]) - 0.40) < 1e-6,
                  details=f"dx={out[0] - rest[0]:.4f}")
        ctx.check(f"{n}_retains_insertion", rear_x < 0.46 - 0.005,
                  details=f"open rear x={rear_x:.4f}")

    # --- Allow hinge barrel embedding at door edge ---
    for i in range(len(HINGE_Z_FRAC)):
        ctx.allow_overlap(
            carcass, door,
            elem_a=f"hinge_barrel_{i}",
            elem_b="door_panel",
            reason=f"Hinge barrel {i} sits at the door hinge edge, intentionally embedded where the real hinge pin passes through the door.",
        )
    # Proof: hinge barrels are at the door hinge edge.
    for i in range(len(HINGE_Z_FRAC)):
        ctx.expect_contact(
            carcass, door,
            elem_a=f"hinge_barrel_{i}",
            elem_b="door_panel",
            name=f"hinge_barrel_{i}_contacts_door_edge",
        )

    # --- Door and drawers occupy different sides ---
    door_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    drawer0_panel = ctx.part_element_world_aabb(drawers["drawer_0"], elem="front_panel")
    assert door_panel_aabb is not None and drawer0_panel is not None
    ctx.check("door_and_drawers_separated_in_y",
              door_panel_aabb[0][1] > drawer0_panel[1][1] + 0.01,
              details=f"door min y={door_panel_aabb[0][1]:.3f}, drawer max y={drawer0_panel[1][1]:.3f}")

    # --- Carved posts still present ---
    seg0 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_0")
    seg1 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_1")
    assert seg0 is not None and seg1 is not None
    w0 = seg0[1][1] - seg0[0][1]
    ctx.check("post_segments_rotated", w0 > POST_SQ + 0.008,
              details=f"seg aabb width={w0:.4f} vs stock {POST_SQ}")
    ctx.check("post_segments_stacked", abs(seg1[0][2] - seg0[1][2]) < 0.004,
              details=f"seg1 bottom={seg1[0][2]:.4f}, seg0 top={seg0[1][2]:.4f}")

    return ctx.report()


object_model = build_object_model()
