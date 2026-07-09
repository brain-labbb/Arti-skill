from __future__ import annotations

# Locker-like steel storage cabinet (~1.70 m W x 0.85 m H x 0.50 m D).
#
# World layout: front faces +X (back at x=0, front opening at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four
# short steel feet. Dark charcoal steel carcass; lighter steel door panels
# with horizontal vent slots. Two upper doors on revolute hinges (visible
# barrel hinges on the door side) and two lower drawers on prismatic slides.

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
WALL = 0.015
TOP_THK = 0.018
FEET_H = 0.040

BW = W_TOTAL
BD = D_TOTAL
BODY_BOT = FEET_H
BODY_TOP = H_TOTAL - TOP_THK
BH = BODY_TOP - BODY_BOT
INNER_W = BW - 2 * WALL

# Door zone (upper section)
DOOR_BOT_Z = BODY_BOT + 0.280
DOOR_TOP_Z = BODY_TOP - 0.010
DOOR_H = DOOR_TOP_Z - DOOR_BOT_Z
DOOR_CENTER_Z = (DOOR_BOT_Z + DOOR_TOP_Z) / 2.0
CENTER_GAP = 0.008
DOOR_W = (INNER_W - CENTER_GAP) / 2.0
DOOR_THK = 0.016
DOOR_STANDOFF = 0.004  # gap between door back face and carcass front edge

# Drawer zone (lower section)
DRAWER_BOT_Z = BODY_BOT + 0.020
DRAWER_TOP_Z = DOOR_BOT_Z - 0.010
DRAWER_H = DRAWER_TOP_Z - DRAWER_BOT_Z
DRAWER_CENTER_Z = (DRAWER_BOT_Z + DRAWER_TOP_Z) / 2.0
DRAWER_W = (INNER_W - CENTER_GAP) / 2.0

# Drawer slide
FACE_THK = 0.016
TRAVEL = 0.380
TRAY_D = 0.420
TRAY_T = 0.010

FRONT_X = BD  # carcass front opening plane

# Hinges
HINGE_LENGTH = 0.080
HINGE_LEAF_W_A = 0.022
HINGE_LEAF_W_B = 0.018
HINGE_LEAF_THK = 0.0024
HINGE_PIN_D = 0.004
HINGE_KNUCKLE_OD = 0.010

# Handle
HANDLE_R = 0.008
HANDLE_L = 0.090

# Feet
FOOT_W = 0.040
FOOT_D = 0.040


def _build_drawer(model, name, front_w, front_h, tray_w, tray_h,
                  steel_dark, steel_tray, handle_mat):
    """Drawer: front panel outer face at local x=0, tray extends toward -X."""
    drawer = model.part(name)

    # Flat steel front panel
    drawer.visual(
        Box((FACE_THK, front_w, front_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="front_panel",
    )

    # Hollow open-top tray
    tray_back_x = -(FACE_THK + TRAY_D)
    tray_cx = -(FACE_THK + TRAY_D / 2.0)
    tray_bot = -front_h / 2.0 + 0.010
    drawer.visual(
        Box((TRAY_D, tray_w, TRAY_T)),
        origin=Origin(xyz=(tray_cx, 0.0, tray_bot + TRAY_T / 2.0)),
        material=steel_tray,
        name="tray_bottom",
    )
    wall_h = tray_h - TRAY_T + 0.002
    wall_cz = tray_bot + TRAY_T - 0.002 + wall_h / 2.0
    drawer.visual(
        Box((TRAY_T, tray_w, wall_h)),
        origin=Origin(xyz=(tray_back_x + TRAY_T / 2.0, 0.0, wall_cz)),
        material=steel_tray,
        name="tray_back_wall",
    )
    side_len = TRAY_D + 0.002
    for tag, s in (("0", 1), ("1", -1)):
        drawer.visual(
            Box((side_len, TRAY_T, wall_h)),
            origin=Origin(xyz=(-FACE_THK + 0.002 - side_len / 2.0,
                               s * (tray_w / 2.0 - TRAY_T / 2.0), wall_cz)),
            material=steel_tray,
            name=f"tray_side_wall_{tag}",
        )
    drawer.visual(
        Box((TRAY_T, tray_w, wall_h)),
        origin=Origin(xyz=(-FACE_THK - TRAY_T / 2.0 + 0.002, 0.0, wall_cz)),
        material=steel_tray,
        name="tray_front_wall",
    )

    # Horizontal bar handle
    drawer.visual(
        Cylinder(radius=HANDLE_R, length=HANDLE_L),
        origin=Origin(xyz=(0.008, 0.0, 0.0),
                      rpy=(0.0, 0.0, math.pi / 2.0)),
        material=handle_mat,
        name="handle",
    )
    for tag, dy in (("0", -0.032), ("1", 0.032)):
        drawer.visual(
            Cylinder(radius=0.005, length=0.012),
            origin=Origin(xyz=(0.004, dy, 0.0),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=handle_mat,
            name=f"handle_mount_{tag}",
        )

    drawer.inertial = Inertial.from_geometry(
        Box((TRAY_D, tray_w, tray_h)), mass=3.0)
    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_locker_cabinet")

    steel_dark = model.material("steel_dark", rgba=(0.25, 0.27, 0.30, 1.0))
    steel_light = model.material("steel_light", rgba=(0.55, 0.58, 0.62, 1.0))
    steel_top = model.material("steel_top", rgba=(0.42, 0.44, 0.48, 1.0))
    steel_tray = model.material("steel_tray", rgba=(0.35, 0.37, 0.40, 1.0))
    handle_mat = model.material("handle_chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    hinge_mat = model.material("hinge_steel", rgba=(0.50, 0.52, 0.55, 1.0))
    vent_mat = model.material("vent_dark", rgba=(0.18, 0.20, 0.22, 1.0))

    # ===================================================================
    # CARCASS (root)
    # ===================================================================
    carcass = model.part("carcass")

    # Side panels
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD, WALL, BH)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                               BODY_BOT + BH / 2.0)),
            material=steel_dark,
            name=f"side_panel_{tag}",
        )

    # Back panel
    carcass.visual(
        Box((WALL, INNER_W, BH)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=steel_dark,
        name="back_panel",
    )

    # Bottom board
    carcass.visual(
        Box((BD - WALL, INNER_W, WALL)),
        origin=Origin(xyz=(WALL + (BD - WALL) / 2.0, 0.0,
                           BODY_BOT + WALL / 2.0)),
        material=steel_dark,
        name="bottom_board",
    )

    # Horizontal divider between doors and drawers
    carcass.visual(
        Box((BD - WALL, INNER_W, WALL)),
        origin=Origin(xyz=(WALL + (BD - WALL) / 2.0, 0.0, DOOR_BOT_Z)),
        material=steel_dark,
        name="divider_board",
    )

    # Top panel
    carcass.visual(
        Box((BD, BW, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=steel_top,
        name="top_panel",
    )

    # Front frame: top rail above doors
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - DOOR_TOP_Z)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (DOOR_TOP_Z + BODY_TOP) / 2.0)),
        material=steel_dark,
        name="front_top_rail",
    )
    # Front frame: bottom rail below drawers
    carcass.visual(
        Box((WALL, INNER_W, DRAWER_BOT_Z - BODY_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (BODY_BOT + DRAWER_BOT_Z) / 2.0)),
        material=steel_dark,
        name="front_bottom_rail",
    )

    # Center stile between doors (thin vertical strip at center)
    carcass.visual(
        Box((WALL, CENTER_GAP + 0.010, DOOR_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, DOOR_CENTER_Z)),
        material=steel_dark,
        name="door_center_stile",
    )
    # Center stile between drawers
    carcass.visual(
        Box((WALL, CENTER_GAP + 0.010, DRAWER_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, DRAWER_CENTER_Z)),
        material=steel_dark,
        name="drawer_center_stile",
    )

    # Drawer runner shelf (top surface contacts tray bottom)
    runner_top_z = DRAWER_CENTER_Z + (-DRAWER_H / 2.0 + 0.010)
    runner_cz = runner_top_z - 0.005
    carcass.visual(
        Box((BD - 0.030, INNER_W, 0.010)),
        origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, 0.0, runner_cz)),
        material=steel_dark,
        name="drawer_runner",
    )

    # Four short steel feet
    for tag, fx, fy in (("fl", BD - 0.030, BW / 2.0 - 0.030),
                        ("fr", BD - 0.030, -(BW / 2.0 - 0.030)),
                        ("rl", 0.030, BW / 2.0 - 0.030),
                        ("rr", 0.030, -(BW / 2.0 - 0.030))):
        carcass.visual(
            Box((FOOT_D, FOOT_W, FEET_H + 0.004)),
            origin=Origin(xyz=(fx, fy, (FEET_H + 0.004) / 2.0)),
            material=steel_dark,
            name=f"foot_{tag}",
        )

    # --- Visible barrel hinges (2 per door, mounted on carcass) ---
    hinge_geom = BarrelHingeGeometry(
        HINGE_LENGTH,
        leaf_width_a=HINGE_LEAF_W_A,
        leaf_width_b=HINGE_LEAF_W_B,
        leaf_thickness=HINGE_LEAF_THK,
        pin_diameter=HINGE_PIN_D,
        knuckle_outer_diameter=HINGE_KNUCKLE_OD,
        knuckle_count=5,
        holes_a=HingeHolePattern(
            style="round", count=3, diameter=0.003, edge_margin=0.008),
        holes_b=HingeHolePattern(
            style="round", count=2, diameter=0.003, edge_margin=0.010),
    )
    hinge_mesh = mesh_from_geometry(hinge_geom, "barrel_hinge")

    hinge_y_positions = {
        "left": -(BW / 2.0 - WALL),
        "right": (BW / 2.0 - WALL),
    }
    hinge_z_positions = [DOOR_BOT_Z + 0.060, DOOR_TOP_Z - 0.060]

    for door_tag, hy in hinge_y_positions.items():
        for h_idx, hz in enumerate(hinge_z_positions):
            # Hinge pin axis is vertical (local Z of hinge = world Z).
            # For left door, rotate 180° so leaves face correct direction
            yaw = 0.0 if door_tag == "right" else math.pi
            carcass.visual(
                hinge_mesh,
                origin=Origin(xyz=(FRONT_X, hy, hz),
                              rpy=(0.0, 0.0, yaw)),
                material=hinge_mat,
                name=f"hinge_{door_tag}_{h_idx}",
            )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=45.0)

    # ===================================================================
    # DOORS (revolute, vertical hinge axis)
    # ===================================================================
    # Door part frame at the hinge pin line. At q=0 the door is closed.
    # Door panel extends from hinge outward along local Y.

    door_data = []
    for door_tag, s_val, hinge_y in [
        ("left", 1, -(BW / 2.0 - WALL)),
        ("right", -1, (BW / 2.0 - WALL)),
    ]:
        door = model.part(f"door_{door_tag}")

        # Door panel: hinge edge at local y=0, extends to y = s_val * DOOR_W
        # Panel front face at local x = DOOR_STANDOFF + DOOR_THK
        # Panel back face at local x = DOOR_STANDOFF
        panel_cx = DOOR_STANDOFF + DOOR_THK / 2.0
        panel_cy = s_val * DOOR_W / 2.0

        door.visual(
            Box((DOOR_THK, DOOR_W, DOOR_H)),
            origin=Origin(xyz=(panel_cx, panel_cy, 0.0)),
            material=steel_light,
            name="door_panel",
        )

        # Vent slot panel on the upper portion of the door
        vent_w = DOOR_W - 0.08
        vent_h = DOOR_H * 0.35
        vent_geom = SlotPatternPanelGeometry(
            (vent_h, vent_w),  # (local X -> becomes Z after rotation, local Y stays)
            0.003,
            slot_size=(0.040, 0.005),
            pitch=(0.050, 0.012),
            frame=0.010,
            slot_angle_deg=0.0,
        )
        vent_mesh = mesh_from_geometry(vent_geom, f"vent_{door_tag}")

        # Rotate vent to face outward (+X in door frame)
        # rpy=(0, pi/2, 0): local X -> Z, local Y -> Y, local Z -> -X
        vent_x = DOOR_STANDOFF + DOOR_THK + 0.002
        vent_y = s_val * DOOR_W / 2.0
        vent_z = 0.08  # above center in door local frame (upper portion)
        door.visual(
            vent_mesh,
            origin=Origin(xyz=(vent_x, vent_y, vent_z),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=vent_mat,
            name="vent_grille",
        )

        # Vertical bar handle near the free edge of the door
        handle_y = s_val * (DOOR_W - 0.045)
        door.visual(
            Cylinder(radius=HANDLE_R, length=HANDLE_L),
            origin=Origin(xyz=(DOOR_STANDOFF + DOOR_THK + 0.010,
                               handle_y, -0.05)),
            material=handle_mat,
            name="door_handle",
        )
        # Handle mounting brackets
        for mt, dz in (("0", -0.030), ("1", 0.030)):
            door.visual(
                Cylinder(radius=0.005, length=0.012),
                origin=Origin(xyz=(DOOR_STANDOFF + DOOR_THK + 0.004,
                                   handle_y, -0.05 + dz),
                              rpy=(0.0, math.pi / 2.0, 0.0)),
                material=handle_mat,
                name=f"handle_mount_{mt}",
            )

        door.inertial = Inertial.from_geometry(
            Box((DOOR_THK, DOOR_W, DOOR_H)), mass=4.0)
        door_data.append((door_tag, door, hinge_y, s_val))

    # Door articulations: revolute around vertical axis (Z)
    for door_tag, door, hinge_y, s_val in door_data:
        # Left door: hinge on left, door extends +Y from hinge.
        # RHR around -Z moves +Y toward +X (outward). Axis = (0,0,-1).
        # Right door: hinge on right, door extends -Y from hinge.
        # RHR around +Z moves -Y toward +X (outward). Axis = (0,0,+1).
        axis = (0.0, 0.0, -1.0 * s_val)
        model.articulation(
            f"carcass_to_door_{door_tag}",
            ArticulationType.REVOLUTE,
            parent=carcass,
            child=door,
            origin=Origin(xyz=(FRONT_X, hinge_y, DOOR_CENTER_Z)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=20.0, velocity=1.0, lower=0.0, upper=1.5),
        )

    # ===================================================================
    # DRAWERS (prismatic, slide outward along +X)
    # ===================================================================
    drawer_cy = [
        -(DRAWER_W / 2.0 + CENTER_GAP / 2.0),
        (DRAWER_W / 2.0 + CENTER_GAP / 2.0),
    ]

    drawers = []
    for i, cy in enumerate(drawer_cy):
        d = _build_drawer(
            model, f"drawer_{i}", DRAWER_W, DRAWER_H,
            DRAWER_W - 0.020, DRAWER_H - 0.020,
            steel_dark, steel_tray, handle_mat)
        drawers.append((f"drawer_{i}", d, cy))

    # Joint origin: drawer frame origin at front panel outer face.
    # At q=0 the panel back face contacts the carcass front opening plane.
    joint_x = FRONT_X + FACE_THK

    for name, d, cy in drawers:
        model.articulation(
            f"carcass_to_{name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=d,
            origin=Origin(xyz=(joint_x, cy, DRAWER_CENTER_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=50.0, velocity=0.5, lower=0.0, upper=TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    door_left = object_model.get_part("door_left")
    door_right = object_model.get_part("door_right")
    drawer_0 = object_model.get_part("drawer_0")
    drawer_1 = object_model.get_part("drawer_1")

    hinge_left = object_model.get_articulation("carcass_to_door_left")
    hinge_right = object_model.get_articulation("carcass_to_door_right")
    slide_0 = object_model.get_articulation("carcass_to_drawer_0")
    slide_1 = object_model.get_articulation("carcass_to_drawer_1")

    # --- Overall scale ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("feet_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_170", abs(width_y - 1.70) < 0.02,
              details=f"w={width_y:.3f}")
    ctx.check("depth_050", abs(depth_x - 0.50) < 0.04,
              details=f"d={depth_x:.3f}")
    ctx.check("height_085", abs(height_z - 0.85) < 0.01,
              details=f"h={height_z:.3f}")

    # --- Steel top panel ---
    top = ctx.part_element_world_aabb(carcass, elem="top_panel")
    assert top is not None
    ctx.check("top_is_thin_slab",
              abs((top[1][2] - top[0][2]) - TOP_THK) < 0.002)

    # --- Door hinge joints: revolute, vertical axis ---
    for jname, j, expected_sign in [
        ("carcass_to_door_left", hinge_left, -1.0),
        ("carcass_to_door_right", hinge_right, 1.0),
    ]:
        ctx.check(f"{jname}_revolute",
                  j.articulation_type == ArticulationType.REVOLUTE)
        ctx.check(f"{jname}_vertical_axis",
                  abs(j.axis[2]) > 0.99
                  and abs(j.axis[0]) < 0.01
                  and abs(j.axis[1]) < 0.01,
                  details=f"axis={j.axis}")
        ctx.check(f"{jname}_axis_sign",
                  j.axis[2] * expected_sign > 0.99,
                  details=f"axis z={j.axis[2]}")
        ctx.check(f"{jname}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and j.motion_limits.upper > 1.0,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Visible hinge barrels on the door side ---
    for door_tag in ("left", "right"):
        for h_idx in range(2):
            hinge_elem = f"hinge_{door_tag}_{h_idx}"
            aabb = ctx.part_element_world_aabb(carcass, elem=hinge_elem)
            ctx.check(f"{hinge_elem}_exists",
                      aabb is not None,
                      details=f"visible barrel hinge on {door_tag} side")

    # --- Doors open outward (positive q moves free edge toward +X) ---
    for door_tag, door, hinge in [
        ("left", door_left, hinge_left),
        ("right", door_right, hinge_right),
    ]:
        panel_rest = ctx.part_element_world_aabb(door, elem="door_panel")
        with ctx.pose({hinge: 0.8}):
            panel_open = ctx.part_element_world_aabb(door, elem="door_panel")
        assert panel_rest is not None and panel_open is not None
        rest_cx = (panel_rest[0][0] + panel_rest[1][0]) / 2.0
        open_cx = (panel_open[0][0] + panel_open[1][0]) / 2.0
        ctx.check(f"door_{door_tag}_opens_outward",
                  open_cx > rest_cx + 0.10,
                  details=f"rest_cx={rest_cx:.3f}, open_cx={open_cx:.3f}")
        # Door handle should be proud of the door panel surface
        handle = ctx.part_element_world_aabb(door, elem="door_handle")
        ctx.check(f"door_{door_tag}_handle_proud",
                  handle is not None and handle[1][0] > panel_rest[1][0] + 0.002,
                  details=f"handle x={handle[1][0] if handle else 'N/A'}")

    # --- Vent slots on doors ---
    for door_tag, door in [("left", door_left), ("right", door_right)]:
        vent = ctx.part_element_world_aabb(door, elem="vent_grille")
        ctx.check(f"door_{door_tag}_has_vent",
                  vent is not None,
                  details=f"vent slots visible on {door_tag} door")

    # --- Drawer prismatic joints: +X axis, 0..TRAVEL ---
    for dname, slide in [("carcass_to_drawer_0", slide_0),
                          ("carcass_to_drawer_1", slide_1)]:
        ctx.check(f"{dname}_prismatic",
                  slide.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{dname}_axis_x",
                  slide.axis[0] > 0.99
                  and abs(slide.axis[1]) < 0.01
                  and abs(slide.axis[2]) < 0.01)
        ctx.check(f"{dname}_range",
                  abs(slide.motion_limits.lower) < 1e-9
                  and abs(slide.motion_limits.upper - TRAVEL) < 1e-6,
                  details=f"range=({slide.motion_limits.lower},{slide.motion_limits.upper})")

    # --- Drawers: front panels proud, trays nested ---
    carcass_front = FRONT_X
    for dname, dpart in [("drawer_0", drawer_0), ("drawer_1", drawer_1)]:
        face = ctx.part_element_world_aabb(dpart, elem="front_panel")
        tray = ctx.part_element_world_aabb(dpart, elem="tray_bottom")
        wall0 = ctx.part_element_world_aabb(dpart, elem="tray_side_wall_0")
        assert face is not None and tray is not None and wall0 is not None
        ctx.check(f"{dname}_front_proud",
                  face[1][0] > carcass_front + 0.001,
                  details=f"face front x={face[1][0]:.4f}")
        ctx.check(f"{dname}_tray_nested",
                  tray[1][0] < carcass_front + 0.005 and tray[0][0] > 0.02,
                  details=f"tray x=({tray[0][0]:.3f},{tray[1][0]:.3f})")
        ctx.check(f"{dname}_tray_open_top",
                  wall0[1][2] > tray[1][2] + 0.04,
                  details=f"wall top={wall0[1][2]:.3f}")
        ctx.expect_within(dpart, carcass, axes="y", margin=0.002,
                          name=f"{dname}_within_carcass_width")

    # --- Drawers slide forward, rear stays inserted ---
    for dname, dpart, slide in [("drawer_0", drawer_0, slide_0),
                                 ("drawer_1", drawer_1, slide_1)]:
        rest = ctx.part_world_position(dpart)
        with ctx.pose({slide: slide.motion_limits.upper}):
            out = ctx.part_world_position(dpart)
            rear = ctx.part_element_world_aabb(dpart, elem="tray_back_wall")
            assert rear is not None
            rear_x = rear[0][0]
        assert rest is not None and out is not None
        ctx.check(f"{dname}_slides_forward",
                  abs((out[0] - rest[0]) - TRAVEL) < 1e-6,
                  details=f"dx={out[0] - rest[0]:.4f}")
        ctx.check(f"{dname}_retains_insertion",
                  rear_x < carcass_front - 0.005,
                  details=f"open rear x={rear_x:.4f}")

    # --- Independence: opening one drawer leaves neighbor shut ---
    with ctx.pose({slide_0: 0.30}):
        nb = ctx.part_element_world_aabb(drawer_1, elem="front_panel")
        assert nb is not None
        rest_nb_x = FRONT_X + FACE_THK + 0.002
        ctx.check("drawers_independent",
                  abs(nb[1][0] - rest_nb_x) < 0.004,
                  details=f"neighbor front x={nb[1][0]:.4f}")

    # --- Non-fixed joint count ---
    all_joints = [hinge_left, hinge_right, slide_0, slide_1]
    non_fixed = sum(1 for j in all_joints
                    if j.articulation_type != ArticulationType.FIXED)
    ctx.check("at_least_one_non_fixed_joint", non_fixed >= 1,
              details=f"non_fixed={non_fixed}")

    # --- Hinge/door overlap allowances ---
    # The barrel hinge leaves are intentionally embedded in the door panel
    # for mechanical connection representation.
    for door_tag, door_part in [("left", door_left), ("right", door_right)]:
        for h_idx in range(2):
            ctx.allow_overlap(
                carcass, door_part,
                elem_a=f"hinge_{door_tag}_{h_idx}",
                elem_b="door_panel",
                reason=(
                    f"Hinge {door_tag}_{h_idx} leaf is intentionally embedded "
                    "in the door panel to represent the mechanical hinge connection."
                ),
            )

    # --- Drawer isolation allowances ---
    # Prismatic joint children may read as disconnected at rest pose when
    # the front panel just contacts the carcass opening. The drawer tray
    # sits on the carcass runner and slides on a prismatic joint.
    ctx.allow_isolated_part(
        drawer_0,
        reason="Prismatic joint child; drawer tray rests on carcass runner and slides outward.",
    )
    ctx.allow_isolated_part(
        drawer_1,
        reason="Prismatic joint child; drawer tray rests on carcass runner and slides outward.",
    )

    # Proof: drawers remain within carcass width at rest and extended
    for dname, dpart, slide in [("drawer_0", drawer_0, slide_0),
                                 ("drawer_1", drawer_1, slide_1)]:
        ctx.expect_within(dpart, carcass, axes="y", margin=0.002,
                          name=f"{dname}_within_carcass_width_rest")
        with ctx.pose({slide: slide.motion_limits.upper}):
            ctx.expect_within(dpart, carcass, axes="y", margin=0.002,
                              name=f"{dname}_within_carcass_width_extended")

    # Proof: hinges are near the door edge (mechanical connection)
    for door_tag in ("left", "right"):
        for h_idx in range(2):
            hinge_aabb = ctx.part_element_world_aabb(
                carcass, elem=f"hinge_{door_tag}_{h_idx}")
            door_aabb = ctx.part_element_world_aabb(
                object_model.get_part(f"door_{door_tag}"), elem="door_panel")
            if hinge_aabb and door_aabb:
                ctx.expect_overlap(
                    carcass, object_model.get_part(f"door_{door_tag}"),
                    axes="z",
                    elem_a=f"hinge_{door_tag}_{h_idx}",
                    elem_b="door_panel",
                    min_overlap=0.04,
                    name=f"hinge_{door_tag}_{h_idx}_overlaps_door_in_z",
                )

    return ctx.report()


object_model = build_object_model()
