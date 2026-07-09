from __future__ import annotations

# Narrow bathroom wall cabinet (~0.42 m W x 0.65 m H x 0.14 m D).
# Wall-mounted: bottom of cabinet at z=0, back panel at x=0, front face at x=D.
# Width along Y (centered), height along +Z.
#
# Three vertical sections from bottom to top:
#   1. Lower drawer zone: two stacked drawers, each on a prismatic joint (+X).
#   2. Middle open shelf zone: open front with two shelf boards visible.
#   3. Upper mirror zone: full-width mirrored door on a revolute side hinge.
#
# Carcass is matte white painted wood. Door has a mirror surface. Small gap
# seams surround all moving fronts (drawer reveals and door clearance).

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
CAB_W = 0.420           # overall width (Y)
CAB_D = 0.140           # overall depth (X), back at x=0
CAB_H = 0.650           # overall height (Z)
WALL = 0.014            # carcass panel thickness

# Inner cavity
INNER_W = CAB_W - 2 * WALL
INNER_D = CAB_D - WALL  # back panel takes one WALL, front is open

# Vertical section boundaries (z)
BOT_PANEL_Z = WALL                       # 0.014 - bottom panel top
DRAWER_ZONE_TOP = 0.200                  # top of drawer section
DIVIDER_1_Z = DRAWER_ZONE_TOP            # divider between drawers and shelves
DIVIDER_1_TOP = DIVIDER_1_Z + WALL       # 0.214
SHELF_ZONE_TOP = 0.370                   # top of open shelf section
DIVIDER_2_Z = SHELF_ZONE_TOP             # divider between shelves and mirror
DIVIDER_2_TOP = DIVIDER_2_Z + WALL       # 0.384
MIRROR_ZONE_TOP = CAB_H - WALL           # 0.636 - below top panel

# Drawer section
DRAWER_H = (DRAWER_ZONE_TOP - BOT_PANEL_Z) / 2.0  # each drawer front height ~0.093
REVEAL = 0.003                            # gap seam around moving fronts
DRAWER_0_CY = 0.0                         # bottom drawer center z
DRAWER_0_Z_BOT = BOT_PANEL_Z
DRAWER_0_Z_TOP = BOT_PANEL_Z + DRAWER_H
DRAWER_1_Z_BOT = DRAWER_0_Z_TOP + REVEAL
DRAWER_1_Z_TOP = DRAWER_1_Z_BOT + DRAWER_H
DRAWER_1_CY = (DRAWER_1_Z_BOT + DRAWER_1_Z_TOP) / 2.0

# Drawer front panel
FACE_THK = 0.014
FACE_PROUD = 0.001                        # front panel slightly proud of carcass

# Drawer tray (interior box)
TRAY_D = 0.110                            # tray depth (fits inside cabinet)
TRAY_T = 0.008                            # tray panel thickness
TRAY_W = INNER_W - 0.006                  # slight clearance inside carcass

# Joint position for drawers: front of carcass face
FRONT_X = CAB_D                           # 0.14
JOINT_X = FRONT_X + FACE_PROUD            # drawer joint at proud front face

# Mirror door
DOOR_THK = 0.012                          # door panel thickness
MIRROR_H = MIRROR_ZONE_TOP - DIVIDER_2_TOP  # mirror section height ~0.252
MIRROR_CY = (DIVIDER_2_TOP + MIRROR_ZONE_TOP) / 2.0  # center z
DOOR_W = INNER_W - 2 * REVEAL            # door width with gap seams
DOOR_REVEAL_TOP = REVEAL
DOOR_REVEAL_BOT = REVEAL

# Open shelf zone
SHELF_ZONE_H = SHELF_ZONE_TOP - DIVIDER_1_TOP  # ~0.156
SHELF_BOARD_THK = 0.010

# Knobs
KNOB_R = 0.008
STEM_R = 0.004
STEM_L = 0.010

# Travel for drawers
TRAVEL = 0.100                            # drawers slide out 0.10 m


def _build_drawer(model: ArticulatedObject, name: str, front_w: float,
                  front_h: float, white, tray_mat, chrome):
    """Drawer in local frame: front panel outer surface at local x=0,
    panel extends toward -X. Tray extends behind the front panel."""
    drawer = model.part(name)

    # Flat white front panel
    drawer.visual(
        Box((FACE_THK, front_w, front_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, 0.0)),
        material=white,
        name="front_panel",
    )

    # Hollow open-top tray behind front panel
    tray_cx = -(FACE_THK + TRAY_D / 2.0)
    tray_bot = -front_h / 2.0 + 0.008
    wall_h = front_h - 0.016
    wall_cz = tray_bot + TRAY_T + wall_h / 2.0 - 0.002

    # Tray bottom
    drawer.visual(
        Box((TRAY_D, TRAY_W, TRAY_T)),
        origin=Origin(xyz=(tray_cx, 0.0, tray_bot + TRAY_T / 2.0)),
        material=tray_mat,
        name="tray_bottom",
    )
    # Tray back wall
    drawer.visual(
        Box((TRAY_T, TRAY_W, wall_h)),
        origin=Origin(xyz=(-(FACE_THK + TRAY_D) + TRAY_T / 2.0, 0.0, wall_cz)),
        material=tray_mat,
        name="tray_back_wall",
    )
    # Tray side walls
    side_len = TRAY_D + 0.002
    for tag, s in (("0", 1), ("1", -1)):
        drawer.visual(
            Box((side_len, TRAY_T, wall_h)),
            origin=Origin(xyz=(-(FACE_THK - 0.002) - side_len / 2.0,
                               s * (TRAY_W / 2.0 - TRAY_T / 2.0), wall_cz)),
            material=tray_mat,
            name=f"tray_side_{tag}",
        )
    # Tray front wall (behind the front panel)
    drawer.visual(
        Box((TRAY_T, TRAY_W, wall_h)),
        origin=Origin(xyz=(-(FACE_THK + TRAY_T / 2.0) + 0.002, 0.0, wall_cz)),
        material=tray_mat,
        name="tray_front_wall",
    )

    # Single chrome knob centered on drawer front
    drawer.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.003),
        origin=Origin(xyz=((STEM_L - 0.003) / 2.0, 0.0, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="knob_stem",
    )
    drawer.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.003, 0.0, 0.0)),
        material=chrome,
        name="knob_ball",
    )

    drawer.inertial = Inertial.from_geometry(
        Box((TRAY_D, TRAY_W, front_h)), mass=1.5)
    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_wall_cabinet")

    # Materials
    white = model.material("white_paint", rgba=(0.92, 0.92, 0.90, 1.0))
    white_inner = model.material("white_inner", rgba=(0.88, 0.88, 0.86, 1.0))
    mirror = model.material("mirror_glass", rgba=(0.82, 0.87, 0.92, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    shelf_wood = model.material("light_wood", rgba=(0.82, 0.74, 0.62, 1.0))
    tray_mat = model.material("tray_white", rgba=(0.90, 0.90, 0.88, 1.0))

    # ===================================================================
    # ROOT: carcass (hollow shell with back, sides, top, bottom, dividers)
    # ===================================================================
    carcass = model.part("carcass")

    # Back panel (against wall)
    carcass.visual(
        Box((WALL, CAB_W, CAB_H)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, CAB_H / 2.0)),
        material=white,
        name="back_panel",
    )
    # Side panels
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((CAB_D, WALL, CAB_H)),
            origin=Origin(xyz=(CAB_D / 2.0, s * (CAB_W / 2.0 - WALL / 2.0),
                               CAB_H / 2.0)),
            material=white,
            name=f"side_panel_{tag}",
        )
    # Bottom panel
    carcass.visual(
        Box((CAB_D, INNER_W, WALL)),
        origin=Origin(xyz=(CAB_D / 2.0, 0.0, WALL / 2.0)),
        material=white,
        name="bottom_panel",
    )
    # Top panel
    carcass.visual(
        Box((CAB_D, INNER_W, WALL)),
        origin=Origin(xyz=(CAB_D / 2.0, 0.0, CAB_H - WALL / 2.0)),
        material=white,
        name="top_panel",
    )

    # Horizontal divider 1: between drawers and open shelf
    carcass.visual(
        Box((CAB_D, INNER_W, WALL)),
        origin=Origin(xyz=(CAB_D / 2.0, 0.0, DIVIDER_1_Z + WALL / 2.0)),
        material=white,
        name="divider_1",
    )
    # Horizontal divider 2: between open shelf and mirror zone
    carcass.visual(
        Box((CAB_D, INNER_W, WALL)),
        origin=Origin(xyz=(CAB_D / 2.0, 0.0, DIVIDER_2_Z + WALL / 2.0)),
        material=white,
        name="divider_2",
    )

    # Front frame rails for drawer zone (side stiles between drawers)
    # Vertical center stile between the two drawers is not needed since they
    # are stacked (not side by side). We need front frame elements around
    # the drawer openings.
    # Drawer zone front frame: thin rails at the sides of the drawer opening
    drawer_opening_w = INNER_W
    stile_w = 0.0  # no stiles, drawers span full inner width

    # Shelf boards in the open shelf zone (2 shelves)
    shelf_bot = DIVIDER_1_TOP
    shelf_top_limit = DIVIDER_2_Z
    shelf_span = shelf_top_limit - shelf_bot
    # Two shelf boards dividing the space into 3 cubbies
    for i, frac in enumerate([0.38, 0.70]):
        sz = shelf_bot + frac * shelf_span
        carcass.visual(
            Box((INNER_D - 0.004, INNER_W - 0.004, SHELF_BOARD_THK)),
            origin=Origin(xyz=(WALL + (INNER_D - 0.004) / 2.0, 0.0, sz)),
            material=shelf_wood,
            name=f"shelf_board_{i}",
        )

    # Internal back wall for mirror zone (behind where mirror sits when closed)
    carcass.visual(
        Box((WALL, INNER_W, MIRROR_H)),
        origin=Origin(xyz=(WALL + WALL / 2.0, 0.0, MIRROR_CY)),
        material=white_inner,
        name="mirror_back_wall",
    )

    # Small wall-mount bracket at back (visible mounting plate)
    carcass.visual(
        Box((0.006, 0.20, 0.08)),
        origin=Origin(xyz=(0.003, 0.0, CAB_H - 0.06)),
        material=chrome,
        name="wall_mount_plate",
    )

    # Drawer runners (thin rails inside the carcass for each drawer)
    for tag, z_bot in (("0", DRAWER_0_Z_BOT), ("1", DRAWER_1_Z_BOT)):
        # Each drawer sits on a runner at its bottom
        carcass.visual(
            Box((INNER_D - 0.010, 0.010, 0.006)),
            origin=Origin(xyz=(WALL + (INNER_D - 0.010) / 2.0,
                               INNER_W / 2.0 - 0.010,
                               z_bot + 0.003)),
            material=white_inner,
            name=f"runner_right_{tag}",
        )
        carcass.visual(
            Box((INNER_D - 0.010, 0.010, 0.006)),
            origin=Origin(xyz=(WALL + (INNER_D - 0.010) / 2.0,
                               -(INNER_W / 2.0 - 0.010),
                               z_bot + 0.003)),
            material=white_inner,
            name=f"runner_left_{tag}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((CAB_D, CAB_W, CAB_H)), mass=8.0)

    # ===================================================================
    # DRAWERS: two stacked prismatic slides along +X
    # ===================================================================
    d0 = _build_drawer(model, "drawer_0", INNER_W - 2 * REVEAL,
                       DRAWER_H - REVEAL, white, tray_mat, chrome)
    d1 = _build_drawer(model, "drawer_1", INNER_W - 2 * REVEAL,
                       DRAWER_H - REVEAL, white, tray_mat, chrome)

    # Drawer 0 articulation
    model.articulation(
        "carcass_to_drawer_0",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=d0,
        origin=Origin(xyz=(JOINT_X, 0.0,
                           (DRAWER_0_Z_BOT + DRAWER_0_Z_TOP) / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.3,
                                   lower=0.0, upper=TRAVEL),
    )

    # Drawer 1 articulation
    model.articulation(
        "carcass_to_drawer_1",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=d1,
        origin=Origin(xyz=(JOINT_X, 0.0,
                           (DRAWER_1_Z_BOT + DRAWER_1_Z_TOP) / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.3,
                                   lower=0.0, upper=TRAVEL),
    )

    # ===================================================================
    # MIRROR DOOR: revolute hinge on left side (-Y edge)
    # ===================================================================
    door = model.part("mirror_door")

    # Door panel: in local frame, hinge at local origin, panel extends +Y
    # Front face of door at local x=0, thickness toward -X
    door.visual(
        Box((DOOR_THK, DOOR_W, MIRROR_H - 2 * DOOR_REVEAL_TOP)),
        origin=Origin(xyz=(-DOOR_THK / 2.0, DOOR_W / 2.0, 0.0)),
        material=mirror,
        name="mirror_panel",
    )
    # Door frame backing (slightly smaller, behind mirror)
    door.visual(
        Box((0.006, DOOR_W - 0.010, MIRROR_H - 2 * DOOR_REVEAL_TOP - 0.010)),
        origin=Origin(xyz=(-(DOOR_THK + 0.003), DOOR_W / 2.0, 0.0)),
        material=white,
        name="door_backing",
    )
    # Small chrome handle on right side of door - horizontal bar proud of face
    handle_y = DOOR_W - 0.030
    # Handle bar: thin cylinder along Y, half-embedded into the mirror front face
    door.visual(
        Cylinder(radius=0.005, length=0.050),
        origin=Origin(xyz=(0.003, handle_y, 0.0),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="door_handle",
    )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, MIRROR_H)), mass=2.0)

    # Hinge: at left edge of opening, front face, mid-height of mirror zone
    # Axis (0,0,-1): positive q opens door outward (right edge swings toward +X)
    model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(FRONT_X, -(INNER_W / 2.0), MIRROR_CY)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5,
                                   lower=0.0, upper=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door = object_model.get_part("mirror_door")
    drawer_0 = object_model.get_part("drawer_0")
    drawer_1 = object_model.get_part("drawer_1")
    door_joint = object_model.get_articulation("carcass_to_door")
    dj0 = object_model.get_articulation("carcass_to_drawer_0")
    dj1 = object_model.get_articulation("carcass_to_drawer_1")

    # --- Overall dimensions ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_042", abs(width_y - 0.42) < 0.02,
              details=f"w={width_y:.3f}")
    ctx.check("depth_014", abs(depth_x - 0.14) < 0.02,
              details=f"d={depth_x:.3f}")
    ctx.check("height_065", abs(height_z - 0.65) < 0.02,
              details=f"h={height_z:.3f}")

    # --- Cabinet is wall-mounted style (bottom above floor) ---
    ctx.check("cabinet_bottom_at_origin",
              abs(cb[0][2]) < 0.005,
              details=f"min z={cb[0][2]:.4f}")

    # --- Three distinct articulations: 2 prismatic drawers + 1 revolute door ---
    ctx.check("drawer_0_prismatic",
              dj0.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("drawer_1_prismatic",
              dj1.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("door_revolute",
              door_joint.articulation_type == ArticulationType.REVOLUTE)

    # --- Drawer axes point outward (+X) ---
    ctx.check("drawer_0_axis",
              dj0.axis[0] > 0.99 and abs(dj0.axis[1]) < 0.01
              and abs(dj0.axis[2]) < 0.01)
    ctx.check("drawer_1_axis",
              dj1.axis[0] > 0.99 and abs(dj1.axis[1]) < 0.01
              and abs(dj1.axis[2]) < 0.01)

    # --- Door hinge axis is vertical (Z) ---
    ctx.check("door_axis_vertical",
              abs(door_joint.axis[2]) > 0.99
              and abs(door_joint.axis[0]) < 0.01
              and abs(door_joint.axis[1]) < 0.01)

    # --- Drawer gap seams (reveals around fronts) ---
    # Drawer fronts should be slightly proud of carcass front
    for name, dp in [("drawer_0", drawer_0), ("drawer_1", drawer_1)]:
        face = ctx.part_element_world_aabb(dp, elem="front_panel")
        assert face is not None
        ctx.check(f"{name}_front_proud",
                  0.0 < face[1][0] - FRONT_X < 0.005,
                  details=f"face front x={face[1][0]:.4f}, carcass front={FRONT_X}")

    # Gap between the two drawer fronts (reveal seam)
    face0 = ctx.part_element_world_aabb(drawer_0, elem="front_panel")
    face1 = ctx.part_element_world_aabb(drawer_1, elem="front_panel")
    assert face0 is not None and face1 is not None
    vertical_gap = face1[0][2] - face0[1][2]
    ctx.check("drawer_reveal_seam",
              0.001 < vertical_gap < 0.010,
              details=f"vertical gap={vertical_gap:.4f}")

    # --- Door gap seams around mirror door ---
    door_aabb = ctx.part_element_world_aabb(door, elem="mirror_panel")
    assert door_aabb is not None
    # Door should sit at the front face of the cabinet
    ctx.check("door_at_front_face",
              abs(door_aabb[1][0] - FRONT_X) < 0.005,
              details=f"door front x={door_aabb[1][0]:.4f}")

    # --- Shelf boards visible in open section ---
    shelf0 = ctx.part_element_world_aabb(carcass, elem="shelf_board_0")
    shelf1 = ctx.part_element_world_aabb(carcass, elem="shelf_board_1")
    assert shelf0 is not None and shelf1 is not None
    # Shelves should be between the dividers
    ctx.check("shelf_0_in_open_zone",
              shelf0[0][2] > DIVIDER_1_TOP
              and shelf0[1][2] < DIVIDER_2_Z,
              details=f"shelf0 z=({shelf0[0][2]:.4f},{shelf0[1][2]:.4f})")
    ctx.check("shelf_1_in_open_zone",
              shelf1[0][2] > DIVIDER_1_TOP
              and shelf1[1][2] < DIVIDER_2_Z,
              details=f"shelf1 z=({shelf1[0][2]:.4f},{shelf1[1][2]:.4f})")
    # Shelves should span most of the cabinet width
    shelf0_w = shelf0[1][1] - shelf0[0][1]
    ctx.check("shelves_span_width",
              shelf0_w > INNER_W * 0.90,
              details=f"shelf width={shelf0_w:.3f} vs inner={INNER_W:.3f}")

    # --- Open section has no front panel (shelves visible from front) ---
    # The front of the open zone should not have a blocking panel
    # Verify by checking that shelf front edges are near the carcass front
    ctx.check("shelf_front_near_opening",
              shelf0[1][0] > CAB_D - 0.02,
              details=f"shelf max x={shelf0[1][0]:.4f}")

    # --- Drawer slide test: drawers move outward along +X ---
    for name, dp, jt in [("drawer_0", drawer_0, dj0),
                         ("drawer_1", drawer_1, dj1)]:
        rest = ctx.part_world_position(dp)
        with ctx.pose({jt: jt.motion_limits.upper}):
            opened = ctx.part_world_position(dp)
        assert rest is not None and opened is not None
        ctx.check(f"{name}_slides_outward",
                  abs((opened[0] - rest[0]) - TRAVEL) < 1e-6,
                  details=f"dx={opened[0] - rest[0]:.4f}")

    # --- Door opens outward: positive q swings mirror door open ---
    rest_panel = ctx.part_element_world_aabb(door, elem="mirror_panel")
    assert rest_panel is not None
    rest_cx = (rest_panel[0][0] + rest_panel[1][0]) / 2.0
    with ctx.pose({door_joint: 1.0}):
        open_panel = ctx.part_element_world_aabb(door, elem="mirror_panel")
        assert open_panel is not None
        open_cx = (open_panel[0][0] + open_panel[1][0]) / 2.0
    # When door opens, the panel center should move toward +X (outward)
    ctx.check("door_opens_outward",
              open_cx > rest_cx + 0.02,
              details=f"rest cx={rest_cx:.4f}, open cx={open_cx:.4f}")

    # --- Drawers retain insertion when fully open ---
    for name, dp, jt in [("drawer_0", drawer_0, dj0),
                         ("drawer_1", drawer_1, dj1)]:
        with ctx.pose({jt: jt.motion_limits.upper}):
            rear = ctx.part_element_world_aabb(dp, elem="tray_back_wall")
            assert rear is not None
            ctx.check(f"{name}_retains_insertion",
                      rear[0][0] < FRONT_X - 0.005,
                      details=f"open rear x={rear[0][0]:.4f}")

    # --- Drawers are within carcass width ---
    for name, dp in [("drawer_0", drawer_0), ("drawer_1", drawer_1)]:
        ctx.expect_within(dp, carcass, axes="y", margin=0.002,
                          name=f"{name}_within_carcass_width")

    # --- Mirror door covers the upper section ---
    door_z_range = door_aabb[1][2] - door_aabb[0][2]
    ctx.check("door_covers_mirror_zone",
              door_z_range > MIRROR_H * 0.85,
              details=f"door z range={door_z_range:.3f} vs zone={MIRROR_H:.3f}")

    # --- Door handle is present and proud of mirror surface ---
    handle = ctx.part_element_world_aabb(door, elem="door_handle")
    assert handle is not None
    ctx.check("handle_proud_of_mirror",
              handle[1][0] > door_aabb[1][0] - 0.002,
              details=f"handle max x={handle[1][0]:.4f}")

    return ctx.report()


object_model = build_object_model()
