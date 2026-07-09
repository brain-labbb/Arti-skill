from __future__ import annotations

# Locker-like steel cabinet (~1.70 m W x 0.85 m H x 0.50 m D).
#
# World layout: front faces +X (back at x≈0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four
# caster-block feet. Steel gray body with horizontal vent slot panels
# on both side faces. One full-height mirrored door on the left side
# swings outward on visible barrel hinges. A vertical pull handle sits
# near the right (free) edge of the door.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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
FOOT_H = 0.050
WALL = 0.018
TOP_THK = 0.020
OVERHANG = 0.010

BODY_BOT = FOOT_H                      # 0.050
BODY_TOP = H_TOTAL - TOP_THK           # 0.830
BH = BODY_TOP - BODY_BOT               # 0.780

BW = W_TOTAL
BD = D_TOTAL

INNER_W = BW - 2 * WALL                # 1.664
INNER_D = BD - WALL                    # 0.482

# Front frame rails and stiles
RAIL_H = 0.040
STILE_W = 0.035
OPEN_BOT = BODY_BOT + RAIL_H           # 0.090
OPEN_TOP = BODY_TOP - RAIL_H           # 0.790
OPEN_H = OPEN_TOP - OPEN_BOT           # 0.700
OPEN_W = INNER_W - 2 * STILE_W         # 1.594

# Door
DOOR_THK = 0.020
DOOR_GAP = 0.003                       # clearance around door in opening
DOOR_W = OPEN_W - 2 * DOOR_GAP         # 1.588
DOOR_H = OPEN_H - 2 * DOOR_GAP         # 0.694
DOOR_SETBACK = 0.001                   # small gap between door back and frame

# Hinge positions (3 hinges along door height)
DOOR_CZ = (OPEN_BOT + OPEN_TOP) / 2.0  # 0.440
HINGE_FRAC = [0.20, 0.50, 0.80]
HINGE_ZS = [BODY_BOT + BH * f for f in HINGE_FRAC]
HINGE_Y = -BW / 2.0 + WALL + STILE_W  # inner edge of left stile
HINGE_BARREL_R = 0.009
HINGE_BARREL_H = 0.045

# Handle
HANDLE_R = 0.008
HANDLE_L = 0.120
HANDLE_STANDOFF = 0.025

# Feet
FOOT_W = 0.060
FOOT_D = 0.060

# Vent slot panel
VENT_W = 0.280    # along depth (X after rotation)
VENT_H = 0.420    # along height (Z after rotation)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_locker_cabinet")

    # --- materials ---
    steel_body = model.material("steel_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.30, 0.32, 0.35, 1.0))
    steel_top = model.material("steel_top", rgba=(0.58, 0.60, 0.63, 1.0))
    mirror = model.material("mirror_face", rgba=(0.82, 0.85, 0.90, 1.0))
    hinge_mat = model.material("hinge_steel", rgba=(0.52, 0.54, 0.57, 1.0))
    foot_mat = model.material("caster_black", rgba=(0.12, 0.12, 0.14, 1.0))
    handle_mat = model.material("chrome_handle", rgba=(0.85, 0.87, 0.90, 1.0))
    vent_mat = model.material("vent_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    frame_mat = model.material("door_frame", rgba=(0.25, 0.27, 0.30, 1.0))

    # ====================================================
    # ROOT: cabinet body
    # ====================================================
    cab = model.part("cabinet")

    # Side panels
    for tag, s in (("0", 1), ("1", -1)):
        cab.visual(
            Box((BD, WALL, BH)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                               BODY_BOT + BH / 2.0)),
            material=steel_body,
            name=f"side_panel_{tag}",
        )

    # Back panel
    cab.visual(
        Box((WALL, INNER_W, BH)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=steel_body,
        name="back_panel",
    )

    # Bottom board
    cab.visual(
        Box((BD - WALL, INNER_W, WALL)),
        origin=Origin(xyz=(WALL + (BD - WALL) / 2.0, 0.0,
                           BODY_BOT + WALL / 2.0)),
        material=steel_dark,
        name="bottom_board",
    )

    # Top stretcher board
    cab.visual(
        Box((BD - WALL, INNER_W, WALL)),
        origin=Origin(xyz=(WALL + (BD - WALL) / 2.0, 0.0,
                           BODY_TOP - WALL / 2.0)),
        material=steel_dark,
        name="top_board",
    )

    # Top panel (slight overhang)
    cab.visual(
        Box((BD + 2 * OVERHANG, BW + 2 * OVERHANG, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=steel_top,
        name="top_panel",
    )

    # Front top rail
    cab.visual(
        Box((WALL, INNER_W, RAIL_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           BODY_TOP - RAIL_H / 2.0)),
        material=steel_body,
        name="front_top_rail",
    )
    # Front bottom rail
    cab.visual(
        Box((WALL, INNER_W, RAIL_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           BODY_BOT + RAIL_H / 2.0)),
        material=steel_body,
        name="front_bottom_rail",
    )
    # Left stile (hinge side)
    cab.visual(
        Box((WALL, STILE_W, OPEN_H)),
        origin=Origin(xyz=(BD - WALL / 2.0,
                           -BW / 2.0 + WALL + STILE_W / 2.0,
                           DOOR_CZ)),
        material=steel_body,
        name="left_stile",
    )
    # Right stile (latch side)
    cab.visual(
        Box((WALL, STILE_W, OPEN_H)),
        origin=Origin(xyz=(BD - WALL / 2.0,
                           BW / 2.0 - WALL - STILE_W / 2.0,
                           DOOR_CZ)),
        material=steel_body,
        name="right_stile",
    )

    # Interior shelf (one shelf, embedded into side panels for connectivity)
    shelf_d = INNER_D - 0.010
    cab.visual(
        Box((shelf_d, INNER_W + 0.004, 0.015)),
        origin=Origin(xyz=(WALL + shelf_d / 2.0, 0.0,
                           BODY_BOT + BH * 0.48)),
        material=steel_dark,
        name="interior_shelf",
    )

    # --- Vent slot panels on side faces ---
    vent_geo = SlotPatternPanelGeometry(
        (VENT_W, VENT_H),
        0.004,
        slot_size=(0.055, 0.007),
        pitch=(0.075, 0.016),
        frame=0.012,
        center=True,
    )
    vent_mesh_r = mesh_from_geometry(vent_geo, "vent_right")
    vent_mesh_l = mesh_from_geometry(vent_geo, "vent_left")

    # Right side: rotate so panel face points +Y (outward)
    # rpy=(-pi/2, 0, 0): local Y→-Z, Z→+Y; panel in XZ, face at +Y
    cab.visual(
        vent_mesh_r,
        origin=Origin(xyz=(BD / 2.0, BW / 2.0 - 0.001, BODY_BOT + BH / 2.0),
                      rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=vent_mat,
        name="vent_right",
    )
    # Left side: rotate so panel face points -Y (outward)
    # rpy=(pi/2, 0, 0): local Y→+Z, Z→-Y; panel in XZ, face at -Y
    cab.visual(
        vent_mesh_l,
        origin=Origin(xyz=(BD / 2.0, -BW / 2.0 + 0.001, BODY_BOT + BH / 2.0),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=vent_mat,
        name="vent_left",
    )

    # --- Hinge barrels at hinge line ---
    hinge_x = BD + 0.002   # barrel center slightly forward of frame face
    for i, hz in enumerate(HINGE_ZS):
        # Barrel knuckle cylinder (pin along Z)
        cab.visual(
            Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_H),
            origin=Origin(xyz=(hinge_x, HINGE_Y, hz)),
            material=hinge_mat,
            name=f"hinge_barrel_{i}",
        )
        # Frame leaf (thin plate on left stile face)
        cab.visual(
            Box((0.028, 0.003, HINGE_BARREL_H * 0.85)),
            origin=Origin(xyz=(BD - 0.004, HINGE_Y + 0.008, hz)),
            material=hinge_mat,
            name=f"hinge_frame_leaf_{i}",
        )

    # --- Caster block feet ---
    foot_inset_x = 0.055
    foot_inset_y = BW / 2.0 - 0.055
    for tag, fx, fy in (
        ("front_left", BD - foot_inset_x, -foot_inset_y),
        ("front_right", BD - foot_inset_x, foot_inset_y),
        ("rear_left", foot_inset_x, -foot_inset_y),
        ("rear_right", foot_inset_x, foot_inset_y),
    ):
        cab.visual(
            Box((FOOT_D, FOOT_W, FOOT_H + 0.004)),
            origin=Origin(xyz=(fx, fy, (FOOT_H + 0.004) / 2.0)),
            material=foot_mat,
            name=f"foot_{tag}",
        )

    cab.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=40.0)

    # ====================================================
    # DOOR: one mirrored door on left-side hinges
    # ====================================================
    door = model.part("door")

    # Main door panel (mirror finish), extends from hinge toward +Y
    door.visual(
        Box((DOOR_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(DOOR_SETBACK + DOOR_THK / 2.0, DOOR_W / 2.0, 0.0)),
        material=mirror,
        name="door_panel",
    )

    # Door frame strips (dark steel border)
    FT = 0.004    # frame strip thickness (proud of door face)
    FW = 0.028    # frame strip width
    # Top frame strip
    door.visual(
        Box((FT, DOOR_W, FW)),
        origin=Origin(xyz=(DOOR_SETBACK + DOOR_THK + FT / 2.0,
                           DOOR_W / 2.0, DOOR_H / 2.0 - FW / 2.0)),
        material=frame_mat,
        name="door_frame_top",
    )
    # Bottom frame strip
    door.visual(
        Box((FT, DOOR_W, FW)),
        origin=Origin(xyz=(DOOR_SETBACK + DOOR_THK + FT / 2.0,
                           DOOR_W / 2.0, -DOOR_H / 2.0 + FW / 2.0)),
        material=frame_mat,
        name="door_frame_bottom",
    )
    # Right frame strip (handle side)
    door.visual(
        Box((FT, FW, DOOR_H - 2 * FW)),
        origin=Origin(xyz=(DOOR_SETBACK + DOOR_THK + FT / 2.0,
                           DOOR_W - FW / 2.0, 0.0)),
        material=frame_mat,
        name="door_frame_right",
    )
    # Left frame strip (hinge side)
    door.visual(
        Box((FT, FW, DOOR_H - 2 * FW)),
        origin=Origin(xyz=(DOOR_SETBACK + DOOR_THK + FT / 2.0,
                           FW / 2.0, 0.0)),
        material=frame_mat,
        name="door_frame_left",
    )

    # Handle: vertical pull bar near right (free) edge
    handle_y = DOOR_W - 0.060
    door.visual(
        Cylinder(radius=HANDLE_R, length=HANDLE_L),
        origin=Origin(xyz=(DOOR_SETBACK + DOOR_THK + HANDLE_STANDOFF,
                           handle_y, 0.0)),
        material=handle_mat,
        name="handle_bar",
    )
    # Handle standoff brackets
    for bi, dz in enumerate((-0.042, 0.042)):
        door.visual(
            Box((HANDLE_STANDOFF, 0.018, 0.018)),
            origin=Origin(xyz=(DOOR_SETBACK + DOOR_THK + HANDLE_STANDOFF / 2.0,
                               handle_y, dz)),
            material=handle_mat,
            name=f"handle_bracket_{bi}",
        )

    # Door-side hinge leaves (thin plates embedded into door panel hinge edge).
    # Position them so they overlap the door panel in both X and Y for
    # connectivity: the door panel spans local y in [0, DOOR_W], so the
    # leaf center Y must be > 0 to land inside the panel.
    for i, hz in enumerate(HINGE_ZS):
        local_z = hz - DOOR_CZ
        door.visual(
            Box((0.024, 0.003, HINGE_BARREL_H * 0.85)),
            origin=Origin(xyz=(DOOR_SETBACK + DOOR_THK / 2.0,
                               0.008, local_z)),
            material=hinge_mat,
            name=f"hinge_door_leaf_{i}",
        )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, DOOR_H)), mass=7.0)

    # ====================================================
    # ARTICULATION: revolute door hinge
    # ====================================================
    # Hinge line at the front-left corner of the opening.
    # axis=(0,0,-1): positive q swings the free edge (+Y) toward +X (outward).
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cab,
        child=door,
        origin=Origin(xyz=(BD, HINGE_Y, DOOR_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5,
            lower=0.0, upper=2.3,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cab = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("cabinet_to_door")

    # --- Grounding and overall scale ---
    cb = ctx.part_world_aabb(cab)
    assert cb is not None
    ctx.check("feet_on_floor", abs(cb[0][2]) < 0.005,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_170", abs(width_y - 1.70) < 0.03,
              details=f"w={width_y:.3f}")
    ctx.check("depth_050", abs(depth_x - 0.50) < 0.04,
              details=f"d={depth_x:.3f}")
    ctx.check("height_085", abs(height_z - 0.85) < 0.02,
              details=f"h={height_z:.3f}")

    # --- Door exists and is revolute ---
    ctx.check("door_is_revolute",
              hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("door_axis_vertical",
              abs(hinge.axis[2]) > 0.99
              and abs(hinge.axis[0]) < 0.02
              and abs(hinge.axis[1]) < 0.02,
              details=f"axis={hinge.axis}")
    ctx.check("door_range_0_to_2_3",
              abs(hinge.motion_limits.lower) < 1e-6
              and abs(hinge.motion_limits.upper - 2.3) < 0.01,
              details=f"range=({hinge.motion_limits.lower},"
                      f"{hinge.motion_limits.upper})")

    # --- Closed pose: door covers the opening ---
    door_aabb = ctx.part_world_aabb(door)
    assert door_aabb is not None
    # Door should span most of the opening width and height
    door_w = door_aabb[1][1] - door_aabb[0][1]
    door_h = door_aabb[1][2] - door_aabb[0][2]
    ctx.check("door_covers_opening_width",
              door_w > OPEN_W * 0.95,
              details=f"door_w={door_w:.3f}, open_w={OPEN_W:.3f}")
    ctx.check("door_covers_opening_height",
              door_h > OPEN_H * 0.95,
              details=f"door_h={door_h:.3f}, open_h={OPEN_H:.3f}")

    # Door panel front face slightly proud of cabinet frame (use panel element,
    # not full-door AABB which includes the handle standoff)
    cab_front_x = BD
    door_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    assert door_panel_aabb is not None
    door_front_x = door_panel_aabb[1][0]
    ctx.check("door_proud_of_frame",
              0.0 < door_front_x - cab_front_x < 0.04,
              details=f"door panel front={door_front_x:.4f}, "
                      f"frame={cab_front_x}")

    # --- Hinge barrels visible at hinge line ---
    for i in range(3):
        barrel = ctx.part_element_world_aabb(cab, elem=f"hinge_barrel_{i}")
        assert barrel is not None
        barrel_cy = (barrel[0][1] + barrel[1][1]) / 2.0
        ctx.check(f"hinge_barrel_{i}_at_hinge_line",
                  abs(barrel_cy - HINGE_Y) < 0.015,
                  details=f"barrel cy={barrel_cy:.4f}, hinge_y={HINGE_Y:.4f}")

    # --- Vent slot panels on both sides ---
    vent_r = ctx.part_element_world_aabb(cab, elem="vent_right")
    vent_l = ctx.part_element_world_aabb(cab, elem="vent_left")
    assert vent_r is not None and vent_l is not None
    ctx.check("vent_right_on_right_side",
              vent_r[0][1] > BW / 2.0 - 0.02,
              details=f"vent_r min y={vent_r[0][1]:.3f}")
    ctx.check("vent_left_on_left_side",
              vent_l[1][1] < -BW / 2.0 + 0.02,
              details=f"vent_l max y={vent_l[1][1]:.3f}")
    # Vents have meaningful area
    vent_r_h = vent_r[1][2] - vent_r[0][2]
    vent_r_w = vent_r[1][0] - vent_r[0][0]
    ctx.check("vent_right_has_area",
              vent_r_h > 0.30 and vent_r_w > 0.20,
              details=f"h={vent_r_h:.3f}, w={vent_r_w:.3f}")

    # --- Feet at ground level ---
    for ft in ("front_left", "front_right", "rear_left", "rear_right"):
        foot = ctx.part_element_world_aabb(cab, elem=f"foot_{ft}")
        assert foot is not None
        ctx.check(f"foot_{ft}_on_ground",
                  foot[0][2] < 0.003,
                  details=f"foot min z={foot[0][2]:.4f}")

    # --- Handle on door free edge ---
    handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    assert handle is not None
    door_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    assert door_panel is not None
    ctx.check("handle_proud_of_door",
              handle[0][0] > door_panel[1][0] + 0.005,
              details=f"handle min x={handle[0][0]:.4f}, "
                      f"door front={door_panel[1][0]:.4f}")
    # Handle near free edge (right side, positive Y)
    handle_cy = (handle[0][1] + handle[1][1]) / 2.0
    door_cy = (door_panel[0][1] + door_panel[1][1]) / 2.0
    ctx.check("handle_near_free_edge",
              handle_cy > door_cy + DOOR_W * 0.30,
              details=f"handle cy={handle_cy:.3f}, door cy={door_cy:.3f}")

    # --- Open pose: door swings outward ---
    # Use the door frame right (free-edge) element to track swing motion,
    # since the door part origin sits at the hinge line and doesn't translate.
    rest_edge = ctx.part_element_world_aabb(door, elem="door_frame_right")
    assert rest_edge is not None
    rest_edge_x = (rest_edge[0][0] + rest_edge[1][0]) / 2.0
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_aabb = ctx.part_world_aabb(door)
        open_edge = ctx.part_element_world_aabb(door, elem="door_frame_right")
    assert open_aabb is not None and open_edge is not None
    open_edge_x = (open_edge[0][0] + open_edge[1][0]) / 2.0

    # The door's free edge should move significantly forward (+X) when opened.
    ctx.check("door_sways_outward",
              open_aabb[1][0] > cab_front_x + 0.30,
              details=f"open max x={open_aabb[1][0]:.4f}")

    # Free edge moves forward when opened
    ctx.check("door_center_moves_forward",
              open_edge_x > rest_edge_x + 0.10,
              details=f"rest edge x={rest_edge_x:.4f}, "
                      f"open edge x={open_edge_x:.4f}")

    # --- Intentional overlap: hinge barrels embed into door edge ---
    ctx.allow_overlap(
        "cabinet", "door",
        elem_a="hinge_barrel_0", elem_b="door_panel",
        reason="Hinge barrel wraps around the pin at the door edge, "
               "creating a small local embed that represents the real "
               "barrel hinge capture.",
    )
    ctx.allow_overlap(
        "cabinet", "door",
        elem_a="hinge_barrel_1", elem_b="door_panel",
        reason="Middle hinge barrel capture at door edge.",
    )
    ctx.allow_overlap(
        "cabinet", "door",
        elem_a="hinge_barrel_2", elem_b="door_panel",
        reason="Upper hinge barrel capture at door edge.",
    )

    # Proof: hinge barrels remain at hinge line even when door opens
    with ctx.pose({hinge: 1.0}):
        for i in range(3):
            barrel_open = ctx.part_element_world_aabb(
                cab, elem=f"hinge_barrel_{i}")
            assert barrel_open is not None
            ctx.check(f"hinge_barrel_{i}_stable_when_open",
                      abs(barrel_open[0][0] - (BD - HINGE_BARREL_R)) < 0.02,
                      details=f"barrel x={barrel_open[0][0]:.4f}")

    return ctx.report()


object_model = build_object_model()
