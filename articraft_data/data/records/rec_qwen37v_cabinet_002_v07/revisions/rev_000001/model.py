from __future__ import annotations

# Locker-like steel cabinet (~1.70 m W x 0.85 m H x 0.50 m D).
#
# World layout: front faces +X (back at x=0, front at x=BD), width along Y
# (centered at y=0), height along +Z, grounded at z=0 on four short steel legs.
#
# Left section: a hinged door with visible barrel hinges on the left edge,
# vent slot pattern in the upper area, and a horizontal pull handle.
# Right section: three drawers on independent prismatic slides (+X, 0..0.40 m),
# each with a horizontal bar pull handle.
#
# Materials: light gray powder-coated steel carcass and door, slightly darker
# gray for drawers, brushed steel for handles and hinge barrels.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    Cylinder,
    HingeHolePattern,
    Inertial,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------- key dimensions (meters) ----------
W_TOTAL = 1.70
D_TOTAL = 0.50
H_TOTAL = 0.85

LEG_H = 0.080
LEG_SQ = 0.040

BODY_BOT = LEG_H              # 0.080
TOP_THK = 0.020
BODY_TOP = H_TOTAL - TOP_THK  # 0.830
BH = BODY_TOP - BODY_BOT      # 0.750

WALL = 0.015                   # steel sheet thickness
BD = D_TOTAL - 0.020           # body depth = 0.480
BW = W_TOTAL - 0.020           # body width = 1.680
INNER_W = BW - 2 * WALL        # 1.650

# Center divider splits cabinet into door section (-Y) and drawer section (+Y)
DIVIDER_W = 0.018

# Each section (door side and drawer side) has the same full width:
SECTION_W = BW / 2.0 - DIVIDER_W / 2.0  # 0.840 - 0.009 = 0.831
SECTION_HALF = SECTION_W / 2.0           # 0.4155

# Door section center Y (negative side)
DOOR_CY = -(DIVIDER_W / 2.0 + SECTION_HALF)  # -(0.009 + 0.4155) = -0.4245
# Drawer section center Y (positive side)
DRAWER_CY = (DIVIDER_W / 2.0 + SECTION_HALF)  # 0.4245

# Hinge line Y: inner surface of left side panel
HINGE_Y = -(BW / 2.0 - WALL)   # -0.825

# Door panel width (fills the opening with small clearance)
# Door extends from HINGE_Y to near center divider; avoid divider overlap
DOOR_W = (-DIVIDER_W / 2.0 - 0.004) - HINGE_Y  # (-.009-.004) - (-.825) = 0.812

# Drawer zone (vertical)
DRAWER_ZONE_BOT = BODY_BOT + 0.025
DRAWER_ZONE_TOP = BODY_TOP - 0.025
REVEAL = 0.005
N_DRAWERS = 3
DRAWER_FH = (DRAWER_ZONE_TOP - DRAWER_ZONE_BOT - (N_DRAWERS - 1) * REVEAL) / N_DRAWERS

# Drawer centers (Z)
DRAWER_CZ = [DRAWER_ZONE_BOT + DRAWER_FH / 2.0 + i * (DRAWER_FH + REVEAL)
             for i in range(N_DRAWERS)]

# Drawer front width (must stay within inner opening: divider edge to side panel inner face)
DRAWER_OPENING = (BW / 2.0 - WALL) - (DIVIDER_W / 2.0)  # 0.825 - 0.009 = 0.816
DRAWER_FW = DRAWER_OPENING - 0.008  # reveal gaps on both sides

FACE_THK = 0.015
FRONT_X = BD                    # 0.480

TRAVEL = 0.400
TRAY_D = 0.400
TRAY_T = 0.010

# Hinge parameters: hinges at the front-left edge of the cabinet
HINGE_LENGTH = 0.065
N_HINGES = 3
HINGE_ZS = [BODY_BOT + 0.080,
            (BODY_BOT + BODY_TOP) / 2.0,
            BODY_TOP - 0.080]

# Hinge line: at front face, left inner edge
HINGE_X = BD

# Pull handle dimensions
HANDLE_BAR_L = 0.090
HANDLE_BAR_R = 0.005
HANDLE_STEM_R = 0.004
HANDLE_STEM_L = 0.020


def _build_drawer(model: ArticulatedObject, name: str, front_w: float,
                  front_h: float, tray_w: float, tray_h: float,
                  steel_front, tray_mat, handle_mat):
    """Drawer: front panel outer surface at local x=0, panel spans [-FACE_THK, 0].
    Hollow open-top tray extends toward -X. Pull handle on front."""
    drawer = model.part(name)

    # Flat steel front panel
    drawer.visual(
        Box((FACE_THK, front_w, front_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, 0.0)),
        material=steel_front,
        name="front_panel",
    )

    # --- hollow open-top tray ---
    tray_back_x = -(FACE_THK + TRAY_D)
    tray_cx = -(FACE_THK + TRAY_D / 2.0)
    tray_bot = -front_h / 2.0 + 0.010
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

    # --- Pull handle: horizontal bar on two standoffs ---
    for i, dy in enumerate([-HANDLE_BAR_L / 3.0, HANDLE_BAR_L / 3.0]):
        drawer.visual(
            Cylinder(radius=HANDLE_STEM_R, length=HANDLE_STEM_L),
            origin=Origin(xyz=(HANDLE_STEM_L / 2.0, dy, 0.0),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=handle_mat,
            name=f"handle_stem_{i}",
        )
    # Horizontal bar (along Y)
    drawer.visual(
        Cylinder(radius=HANDLE_BAR_R, length=HANDLE_BAR_L),
        origin=Origin(xyz=(HANDLE_STEM_L + HANDLE_BAR_R, 0.0, 0.0),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="handle_bar",
    )

    drawer.inertial = Inertial.from_geometry(
        Box((TRAY_D, tray_w, tray_h)), mass=3.5)
    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_locker_cabinet")

    # Materials
    steel_body = model.material("steel_body", rgba=(0.55, 0.57, 0.60, 1.0))
    steel_door = model.material("steel_door", rgba=(0.52, 0.54, 0.58, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.48, 0.50, 0.54, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.30, 0.32, 0.35, 1.0))
    handle_mat = model.material("brushed_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    tray_mat = model.material("tray_steel", rgba=(0.40, 0.42, 0.45, 1.0))
    hinge_mat = model.material("hinge_steel", rgba=(0.65, 0.67, 0.70, 1.0))

    # ===================================================================
    # ROOT: carcass (steel shell + legs + top)
    # ===================================================================
    carcass = model.part("carcass")

    # Side panels
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD, WALL, BH)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                               BODY_BOT + BH / 2.0)),
            material=steel_body,
            name=f"side_panel_{tag}",
        )
    # Back panel
    carcass.visual(
        Box((WALL, INNER_W, BH)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=steel_body,
        name="back_panel",
    )
    # Bottom board
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + WALL / 2.0)),
        material=steel_body,
        name="bottom_board",
    )
    # Top stretcher
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - WALL / 2.0)),
        material=steel_body,
        name="top_stretcher",
    )
    # Center divider (separates door section from drawer section)
    carcass.visual(
        Box((BD - 0.020, DIVIDER_W, BH - 0.010)),
        origin=Origin(xyz=(0.010 + (BD - 0.020) / 2.0, 0.0,
                           BODY_BOT + (BH - 0.010) / 2.0 + 0.005)),
        material=steel_body,
        name="center_divider",
    )

    # Front frame rails for drawer section (top/bottom)
    # Bottom rail
    carcass.visual(
        Box((WALL, SECTION_W, DRAWER_ZONE_BOT - BODY_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, DRAWER_CY,
                           (BODY_BOT + DRAWER_ZONE_BOT) / 2.0)),
        material=steel_body,
        name="drawer_bottom_rail",
    )
    # Top rail
    carcass.visual(
        Box((WALL, SECTION_W, BODY_TOP - DRAWER_ZONE_TOP)),
        origin=Origin(xyz=(BD - WALL / 2.0, DRAWER_CY,
                           (DRAWER_ZONE_TOP + BODY_TOP) / 2.0)),
        material=steel_body,
        name="drawer_top_rail",
    )
    # Side stile for drawer section (right side, at the outer panel)
    carcass.visual(
        Box((WALL, WALL, BH - 0.010)),
        origin=Origin(xyz=(BD - WALL / 2.0, BW / 2.0 - WALL / 2.0,
                           BODY_BOT + BH / 2.0)),
        material=steel_body,
        name="drawer_right_stile",
    )

    # Dust panels / runners for drawers
    for i, cz in enumerate(DRAWER_CZ):
        tray_bot_z = cz + (-DRAWER_FH / 2.0 + 0.010)
        # Runner stops before the drawer front panel (runner max x < FRONT_X - FACE_THK)
        runner_len = BD - 0.050
        carcass.visual(
            Box((runner_len, SECTION_W - 0.010, 0.010)),
            origin=Origin(xyz=(0.030 + runner_len / 2.0, DRAWER_CY,
                               tray_bot_z - 0.005)),
            material=steel_dark,
            name=f"drawer_runner_{i}",
        )

    # Front frame for door section (top/bottom rails)
    carcass.visual(
        Box((WALL, SECTION_W, 0.030)),
        origin=Origin(xyz=(BD - WALL / 2.0, DOOR_CY,
                           BODY_BOT + 0.015)),
        material=steel_body,
        name="door_bottom_rail",
    )
    carcass.visual(
        Box((WALL, SECTION_W, 0.030)),
        origin=Origin(xyz=(BD - WALL / 2.0, DOOR_CY,
                           BODY_TOP - 0.015)),
        material=steel_body,
        name="door_top_rail",
    )

    # Top panel (steel, slight overhang)
    carcass.visual(
        Box((D_TOTAL + 0.010, W_TOTAL + 0.010, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=handle_mat,
        name="top_panel",
    )

    # Vent slots on right side panel (upper area)
    vent_side = SlotPatternPanelGeometry(
        (0.180, 0.100),
        0.004,
        slot_size=(0.035, 0.005),
        pitch=(0.045, 0.014),
        frame=0.010,
        corner_radius=0.003,
    )
    vent_side_mesh = mesh_from_geometry(vent_side, "vent_side_right")
    # Place on outer face of right side panel: rotate to face +Y
    carcass.visual(
        vent_side_mesh,
        origin=Origin(xyz=(BD * 0.65, BW / 2.0, BODY_TOP - 0.120),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="vent_side_right",
    )

    # Four steel legs
    for tag, lx, ly in (("front_left", 0.040, -(BW / 2.0 - 0.040)),
                         ("front_right", 0.040, (BW / 2.0 - 0.040)),
                         ("rear_left", BD - 0.040, -(BW / 2.0 - 0.040)),
                         ("rear_right", BD - 0.040, (BW / 2.0 - 0.040))):
        carcass.visual(
            Box((LEG_SQ, LEG_SQ, LEG_H + 0.004)),
            origin=Origin(xyz=(lx, ly, (LEG_H + 0.004) / 2.0)),
            material=steel_dark,
            name=f"leg_{tag}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=45.0)

    # ===================================================================
    # DOOR: hinged on left edge with barrel hinges and vent slots
    # ===================================================================
    door = model.part("door")

    # Door panel: in local frame, hinge edge at y=0, extends in +Y.
    # Front face at local x=0, thickness in -X direction.
    door_h = BH - 0.060
    door.visual(
        Box((FACE_THK, DOOR_W, door_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, DOOR_W / 2.0, 0.0)),
        material=steel_door,
        name="door_panel",
    )

    # Vent slot pattern on upper portion of door
    vent_door_w = DOOR_W - 0.080
    vent_door_h = 0.160
    vent_door = SlotPatternPanelGeometry(
        (vent_door_w, vent_door_h),
        0.004,
        slot_size=(0.040, 0.005),
        pitch=(0.050, 0.014),
        frame=0.010,
        corner_radius=0.003,
    )
    vent_door_mesh = mesh_from_geometry(vent_door, "vent_door")
    # Place on front face of door, upper portion
    door.visual(
        vent_door_mesh,
        origin=Origin(xyz=(0.002, DOOR_W / 2.0, door_h / 2.0 - 0.130),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="vent_door_slots",
    )

    # Pull handle on door (near free edge, vertical bar)
    handle_y = DOOR_W - 0.050  # near free edge
    for i, dz in enumerate([-0.025, 0.025]):
        door.visual(
            Cylinder(radius=HANDLE_STEM_R, length=HANDLE_STEM_L),
            origin=Origin(xyz=(HANDLE_STEM_L / 2.0, handle_y, dz),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=handle_mat,
            name=f"door_handle_stem_{i}",
        )
    # Vertical bar
    door.visual(
        Cylinder(radius=HANDLE_BAR_R, length=0.060),
        origin=Origin(xyz=(HANDLE_STEM_L + HANDLE_BAR_R, handle_y, 0.0)),
        material=handle_mat,
        name="door_handle_bar",
    )

    # Barrel hinges at the hinge edge
    hinge_geom = BarrelHingeGeometry(
        HINGE_LENGTH,
        leaf_width_a=0.018,
        leaf_width_b=0.016,
        leaf_thickness=0.002,
        pin_diameter=0.004,
        knuckle_count=5,
        holes_a=HingeHolePattern(style="round", count=2, diameter=0.003,
                                 edge_margin=0.010),
        holes_b=HingeHolePattern(style="round", count=2, diameter=0.003,
                                 edge_margin=0.010),
    )
    for i, hz in enumerate(HINGE_ZS):
        hinge_mesh = mesh_from_geometry(hinge_geom, f"hinge_barrel_{i}")
        # Hinge at door hinge edge; local z offset relative to door center
        door.visual(
            hinge_mesh,
            origin=Origin(xyz=(0.0, 0.0, hz - (BODY_BOT + BH / 2.0))),
            material=hinge_mat,
            name=f"hinge_barrel_{i}",
        )

    door.inertial = Inertial.from_geometry(
        Box((FACE_THK, DOOR_W, door_h)), mass=5.0)

    # Door articulation: revolute around Z at the hinge line (front-left edge).
    # With axis=(0,0,-1) and door extending in +Y from hinge, positive q swings
    # the free edge outward (+X direction) for an outward-opening door.
    model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, BODY_BOT + BH / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0,
                                   lower=0.0, upper=2.4),
    )

    # ===================================================================
    # DRAWERS: three independent PRISMATIC slides along +X
    # ===================================================================
    drawers = []
    tray_w = DRAWER_FW - 0.020
    tray_h = DRAWER_FH - 0.030
    for i in range(N_DRAWERS):
        name = f"drawer_{i}"
        d = _build_drawer(model, name, DRAWER_FW, DRAWER_FH,
                          tray_w, tray_h,
                          steel_drawer, tray_mat, handle_mat)
        drawers.append((name, d, DRAWER_CY, DRAWER_CZ[i]))

    for name, d, cy, cz in drawers:
        model.articulation(
            f"carcass_to_{name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=d,
            origin=Origin(xyz=(FRONT_X, cy, cz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=50.0, velocity=0.5,
                                       lower=0.0, upper=TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("carcass_to_door")

    drawer_names = [f"drawer_{i}" for i in range(N_DRAWERS)]
    drawer_parts = {n: object_model.get_part(n) for n in drawer_names}
    drawer_joints = {n: object_model.get_articulation(f"carcass_to_{n}")
                     for n in drawer_names}

    # --- Overall dimensions ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_170", abs(width_y - 1.70) < 0.04, details=f"w={width_y:.3f}")
    ctx.check("depth_050", abs(depth_x - 0.50) < 0.04, details=f"d={depth_x:.3f}")
    ctx.check("height_085", abs(height_z - 0.85) < 0.02, details=f"h={height_z:.3f}")
    ctx.check("legs_on_floor", abs(cb[0][2]) < 0.005, details=f"min_z={cb[0][2]:.4f}")

    # --- Door: revolute joint with visible hinge barrels ---
    ctx.check("door_is_revolute",
              door_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("door_axis_z",
              abs(door_joint.axis[2]) > 0.99,
              details=f"axis={door_joint.axis}")
    ctx.check("door_range_positive",
              door_joint.motion_limits.upper > 1.5,
              details=f"upper={door_joint.motion_limits.upper}")

    # Hinge barrels exist
    hinge_0 = door.get_visual("hinge_barrel_0")
    hinge_1 = door.get_visual("hinge_barrel_1")
    hinge_2 = door.get_visual("hinge_barrel_2")
    ctx.check("hinge_barrels_exist",
              hinge_0 is not None and hinge_1 is not None and hinge_2 is not None)

    # Door opens outward: check door panel element position
    panel_rest = ctx.part_element_world_aabb(door, elem="door_panel")
    assert panel_rest is not None
    rest_front_x = panel_rest[1][0]  # max X of panel at rest

    with ctx.pose({door_joint: 1.2}):
        panel_open = ctx.part_element_world_aabb(door, elem="door_panel")
    assert panel_open is not None
    open_front_x = panel_open[1][0]

    ctx.check("door_opens_outward",
              open_front_x > rest_front_x + 0.10,
              details=f"rest_x={rest_front_x:.3f}, open_x={open_front_x:.3f}")

    # --- Vent slots on door ---
    vent = door.get_visual("vent_door_slots")
    ctx.check("door_vent_slots_exist", vent is not None)

    # --- Drawer pull handles ---
    for n in drawer_names:
        bar = object_model.get_part(n).get_visual("handle_bar")
        ctx.check(f"{n}_has_pull_handle", bar is not None)

    # --- Three prismatic drawers ---
    ctx.check("three_drawers", len(drawer_joints) == N_DRAWERS)
    for n, j in drawer_joints.items():
        ctx.check(f"{n}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{n}_axis_forward",
                  j.axis[0] > 0.99 and abs(j.axis[1]) < 0.01 and abs(j.axis[2]) < 0.01)
        ctx.check(f"{n}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - TRAVEL) < 1e-6,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Drawer slide test ---
    for n in drawer_names:
        d, j = drawer_parts[n], drawer_joints[n]
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
            rear = ctx.part_element_world_aabb(d, elem="tray_back_wall")
        assert rest is not None and out is not None and rear is not None
        ctx.check(f"{n}_slides_forward",
                  abs((out[0] - rest[0]) - TRAVEL) < 1e-6,
                  details=f"dx={out[0] - rest[0]:.4f}")
        ctx.check(f"{n}_retains_insertion",
                  rear[0][0] < FRONT_X - 0.005,
                  details=f"open rear x={rear[0][0]:.4f}")

    # --- Door handle exists ---
    door_bar = door.get_visual("door_handle_bar")
    ctx.check("door_has_pull_handle", door_bar is not None)

    # --- Side vent exists ---
    side_vent = carcass.get_visual("vent_side_right")
    ctx.check("side_vent_exists", side_vent is not None)

    # --- Door on drawer section side check: door fills left section ---
    door_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    assert door_panel_aabb is not None
    door_min_y = door_panel_aabb[0][1]
    door_max_y = door_panel_aabb[1][1]
    ctx.check("door_fills_left_section",
              door_min_y < -0.70 and door_max_y > -0.10,
              details=f"door y=({door_min_y:.3f},{door_max_y:.3f})")

    return ctx.report()


object_model = build_object_model()
