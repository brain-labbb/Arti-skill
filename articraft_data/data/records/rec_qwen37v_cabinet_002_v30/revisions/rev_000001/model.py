from __future__ import annotations

# Narrow bathroom wall cabinet (~0.42 m W x 0.58 m H x 0.14 m D).
#
# World layout: front faces +X (back at x=0, front at x=D),
# width along Y (centered), height along +Z, bottom at z=0.
# Wall-mounted (no legs). White painted wood carcass.
#
# Upper compartment: mirrored door hinged on the left side (negative Y edge),
# revolute about Z, opens outward 0 to ~1.4 rad.
# Lower compartment: tambour front panel on a prismatic joint sliding sideways
# along +Y (to the right), range 0 to ~0.41 m, revealing shelf boards inside.
#
# Internal shelves: one horizontal divider between upper/lower compartments,
# one shelf inside the upper compartment, one shelf inside the lower
# compartment — all visible through the open gap when the tambour slides aside.

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
W = 0.420          # overall width (Y)
H = 0.580          # overall height (Z)
D = 0.140          # overall depth (X)
WALL = 0.012       # panel thickness

INNER_W = W - 2 * WALL   # 0.396
INNER_D = D - WALL       # 0.128 (back panel, open front)
INNER_H = H - 2 * WALL   # 0.556

# Divider height from cabinet bottom: splits upper/lower compartments.
DIV_Z = 0.290      # divider center z (from bottom of cabinet)
DIV_THK = 0.012    # divider panel thickness

# Upper compartment: from DIV_Z + DIV_THK/2 to top
UPPER_BOT = DIV_Z + DIV_THK / 2.0
UPPER_TOP = H - WALL
UPPER_H = UPPER_TOP - UPPER_BOT  # ~0.278

# Lower compartment: from bottom to DIV_Z - DIV_THK/2
LOWER_BOT = WALL
LOWER_TOP = DIV_Z - DIV_THK / 2.0
LOWER_H = LOWER_TOP - LOWER_BOT  # ~0.266

# Shelf positions (inside compartments)
UPPER_SHELF_Z = UPPER_BOT + UPPER_H * 0.50   # mid-upper
LOWER_SHELF_Z = LOWER_BOT + LOWER_H * 0.45   # slightly below mid-lower
SHELF_THK = 0.010

# Door dimensions (covers upper compartment front opening, below top rail)
DOOR_W = INNER_W          # spans inner width
DOOR_H = UPPER_H - 0.018  # fits below top rail with reveal
DOOR_THK = 0.010          # thin door panel
MIRROR_THK = 0.003        # mirror layer on door front

# Tambour dimensions (covers lower compartment front opening)
TAMBOUR_W = INNER_W       # spans inner width
TAMBOUR_H = LOWER_H       # spans lower compartment height
TAMBOUR_THK = 0.010

# Hinge position: left side of upper compartment (negative Y)
HINGE_Y = -W / 2.0 + WALL   # at the inner left edge

# Tambour slide range
TAMBOUR_TRAVEL = 0.410

# Door frame (front face x coordinate)
FRONT_X = D                 # carcass front plane

# Small handle/knob on door
HANDLE_R = 0.008
HANDLE_STEM_L = 0.015


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_wall_cabinet")

    # Materials
    white_wood = model.material("white_wood", rgba=(0.92, 0.91, 0.89, 1.0))
    white_inner = model.material("white_inner", rgba=(0.88, 0.87, 0.86, 1.0))
    mirror = model.material("mirror_surface", rgba=(0.82, 0.85, 0.88, 1.0))
    chrome = model.material("chrome_handle", rgba=(0.75, 0.76, 0.78, 1.0))
    tambour_mat = model.material("tambour_slats", rgba=(0.85, 0.84, 0.82, 1.0))
    shelf_mat = model.material("shelf_white", rgba=(0.90, 0.89, 0.87, 1.0))

    # ===================================================================
    # ROOT: carcass (hollow box panels + internal shelves)
    # ===================================================================
    carcass = model.part("carcass")

    # Back panel
    carcass.visual(
        Box((WALL, W, H)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, H / 2.0)),
        material=white_wood,
        name="back_panel",
    )

    # Side panels
    for tag, s in (("left", -1), ("right", 1)):
        carcass.visual(
            Box((D, WALL, H)),
            origin=Origin(xyz=(D / 2.0, s * (W / 2.0 - WALL / 2.0), H / 2.0)),
            material=white_wood,
            name=f"side_{tag}",
        )

    # Top panel
    carcass.visual(
        Box((D, INNER_W, WALL)),
        origin=Origin(xyz=(D / 2.0, 0.0, H - WALL / 2.0)),
        material=white_wood,
        name="top_panel",
    )

    # Bottom panel
    carcass.visual(
        Box((D, INNER_W, WALL)),
        origin=Origin(xyz=(D / 2.0, 0.0, WALL / 2.0)),
        material=white_wood,
        name="bottom_panel",
    )

    # Horizontal divider between upper/lower compartments
    carcass.visual(
        Box((INNER_D, INNER_W, DIV_THK)),
        origin=Origin(xyz=(WALL + INNER_D / 2.0, 0.0, DIV_Z)),
        material=white_inner,
        name="divider_shelf",
    )

    # Shelf depth: stops short of the front opening to avoid door/tambour overlap
    SHELF_D = INNER_D - 0.020  # 0.108 — leaves 20mm from front opening
    # Shelf center x: embedded 1mm into back panel for connectivity
    SHELF_CX = WALL - 0.001 + SHELF_D / 2.0

    # Upper internal shelf (contacts back panel and side panels)
    carcass.visual(
        Box((SHELF_D, INNER_W + 0.002, SHELF_THK)),
        origin=Origin(xyz=(SHELF_CX, 0.0, UPPER_SHELF_Z)),
        material=shelf_mat,
        name="upper_shelf",
    )

    # Lower internal shelf (contacts back panel and side panels)
    carcass.visual(
        Box((SHELF_D, INNER_W + 0.002, SHELF_THK)),
        origin=Origin(xyz=(SHELF_CX, 0.0, LOWER_SHELF_Z)),
        material=shelf_mat,
        name="lower_shelf",
    )

    # Front face lip (thin perimeter lip around the front opening, set back
    # from the door/tambour contact plane to avoid overlap)
    lip_h = 0.008
    lip_w = 0.008
    # Top lip (above door opening, set inward from front face)
    carcass.visual(
        Box((lip_w, INNER_W, lip_h)),
        origin=Origin(xyz=(D - WALL - lip_w / 2.0, 0.0,
                           H - WALL - lip_h / 2.0)),
        material=white_wood,
        name="front_top_lip",
    )
    # Bottom lip (below tambour opening)
    carcass.visual(
        Box((lip_w, INNER_W, lip_h)),
        origin=Origin(xyz=(D - WALL - lip_w / 2.0, 0.0,
                           WALL + lip_h / 2.0)),
        material=white_wood,
        name="front_bottom_lip",
    )

    # Tambour guide rails (thin horizontal tracks for the tambour to slide in)
    guide_thk = 0.005
    guide_depth = 0.020
    # Upper tambour guide (at top of lower opening)
    carcass.visual(
        Box((guide_depth, INNER_W + 0.010, guide_thk)),
        origin=Origin(xyz=(D - guide_depth / 2.0, 0.0,
                           LOWER_TOP + guide_thk / 2.0)),
        material=chrome,
        name="tambour_guide_upper",
    )
    # Lower tambour guide (at bottom of lower opening)
    carcass.visual(
        Box((guide_depth, INNER_W + 0.010, guide_thk)),
        origin=Origin(xyz=(D - guide_depth / 2.0, 0.0,
                           LOWER_BOT - guide_thk / 2.0 + WALL)),
        material=chrome,
        name="tambour_guide_lower",
    )

    carcass.inertial = Inertial.from_geometry(
        Box((D, W, H)), mass=5.0)

    # ===================================================================
    # MIRRORED DOOR: revolute hinge on left side, opens outward
    # ===================================================================
    door = model.part("door")

    # Door panel (local frame: hinge edge at local y=0, door extends along +Y)
    # Door panel center is at local y = DOOR_W/2
    door.visual(
        Box((DOOR_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(-DOOR_THK / 2.0, DOOR_W / 2.0, 0.0)),
        material=white_wood,
        name="door_panel",
    )

    # Mirror layer on the outer face of the door
    door.visual(
        Box((MIRROR_THK, DOOR_W - 0.020, DOOR_H - 0.020)),
        origin=Origin(xyz=(MIRROR_THK / 2.0, DOOR_W / 2.0, 0.0)),
        material=mirror,
        name="mirror_face",
    )

    # Small chrome handle on the right edge of the door (stem embedded 4mm into panel)
    stem_embed = 0.004
    stem_cx = -stem_embed + HANDLE_STEM_L / 2.0
    door.visual(
        Cylinder(radius=HANDLE_R, length=HANDLE_STEM_L),
        origin=Origin(xyz=(stem_cx, DOOR_W - 0.025, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="door_handle_stem",
    )
    door.visual(
        Sphere(radius=HANDLE_R * 1.2),
        origin=Origin(xyz=(-stem_embed + HANDLE_STEM_L + HANDLE_R * 0.5,
                           DOOR_W - 0.025, 0.0)),
        material=chrome,
        name="door_handle_knob",
    )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, DOOR_H)), mass=1.5)

    # Hinge articulation: left side of upper opening, vertical axis (Z)
    # The hinge origin is at the left inner edge of the upper opening,
    # at the vertical center of the upper compartment.
    # At q=0, the door is closed (covering the upper opening).
    # Positive q opens the door outward (rotating the free edge toward +X).
    # The door extends along local +Y from the hinge, so rotating about
    # +Z swings it counterclockwise when viewed from above — the free edge
    # (at +Y) moves toward +X. That's correct for opening outward.
    hinge_z = UPPER_BOT + UPPER_H / 2.0
    model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(FRONT_X, HINGE_Y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5,
                                   lower=0.0, upper=1.40),
    )

    # ===================================================================
    # TAMBOUR FRONT: prismatic slide along +Y (sideways to the right)
    # ===================================================================
    tambour = model.part("tambour")

    # Tambour panel: flat panel that covers the lower compartment opening.
    # Local frame origin at the panel center.
    tambour.visual(
        Box((TAMBOUR_THK, TAMBOUR_W, TAMBOUR_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tambour_mat,
        name="tambour_panel",
    )

    # Horizontal slat lines on the tambour (visual detail — thin raised strips)
    n_slats = 8
    slat_spacing = TAMBOUR_H / (n_slats + 1)
    slat_thk = 0.002
    for i in range(n_slats):
        sz = -TAMBOUR_H / 2.0 + (i + 1) * slat_spacing
        tambour.visual(
            Box((slat_thk, TAMBOUR_W - 0.010, 0.004)),
            origin=Origin(xyz=(TAMBOUR_THK / 2.0 + slat_thk / 2.0, 0.0, sz)),
            material=white_wood,
            name=f"tambour_slat_{i}",
        )

    # Small pull handle on the left edge of the tambour
    tambour.visual(
        Box((0.008, 0.030, 0.012)),
        origin=Origin(xyz=(TAMBOUR_THK / 2.0 + 0.004,
                           -TAMBOUR_W / 2.0 + 0.025, 0.0)),
        material=chrome,
        name="tambour_pull",
    )

    tambour.inertial = Inertial.from_geometry(
        Box((TAMBOUR_THK, TAMBOUR_W, TAMBOUR_H)), mass=1.0)

    # Tambour prismatic articulation: slides along +Y (to the right)
    # Joint origin at the center of the lower opening on the front face.
    tambour_cz = LOWER_BOT + LOWER_H / 2.0
    model.articulation(
        "carcass_to_tambour",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=tambour,
        origin=Origin(xyz=(FRONT_X + TAMBOUR_THK / 2.0,
                           0.0, tambour_cz)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.3,
                                   lower=0.0, upper=TAMBOUR_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    tambour = object_model.get_part("tambour")
    door_joint = object_model.get_articulation("carcass_to_door")
    tambour_joint = object_model.get_articulation("carcass_to_tambour")

    # --- Overall proportions: narrow wall cabinet ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_narrow", 0.35 < width_y < 0.50,
              details=f"w={width_y:.3f}")
    ctx.check("depth_shallow", 0.10 < depth_x < 0.20,
              details=f"d={depth_x:.3f}")
    ctx.check("height_moderate", 0.45 < height_z < 0.70,
              details=f"h={height_z:.3f}")

    # --- No legs (wall-mounted): bottom panel near z=0 ---
    ctx.check("no_legs_bottom_at_zero", abs(cb[0][2]) < 0.005,
              details=f"min z={cb[0][2]:.4f}")

    # --- Internal shelves exist ---
    upper_shelf = ctx.part_element_world_aabb(carcass, elem="upper_shelf")
    lower_shelf = ctx.part_element_world_aabb(carcass, elem="lower_shelf")
    divider = ctx.part_element_world_aabb(carcass, elem="divider_shelf")
    assert upper_shelf is not None and lower_shelf is not None and divider is not None
    ctx.check("upper_shelf_exists", True)
    ctx.check("lower_shelf_exists", True)
    ctx.check("divider_exists", True)

    # Shelves are inside the carcass vertically
    ctx.check("shelves_stacked",
              lower_shelf[0][2] < divider[0][2] < upper_shelf[0][2],
              details=f"lower_z={lower_shelf[0][2]:.3f}, div_z={divider[0][2]:.3f}, upper_z={upper_shelf[0][2]:.3f}")

    # --- Mirrored door: revolute joint ---
    ctx.check("door_is_revolute",
              door_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("door_hinge_vertical_axis",
              abs(door_joint.axis[2]) > 0.99,
              details=f"axis={door_joint.axis}")
    ctx.check("door_range_reasonable",
              door_joint.motion_limits.upper > 1.0
              and door_joint.motion_limits.lower == 0.0,
              details=f"range=({door_joint.motion_limits.lower},{door_joint.motion_limits.upper})")

    # Door has a mirror face
    mirror_face = ctx.part_element_world_aabb(door, elem="mirror_face")
    assert mirror_face is not None
    ctx.check("mirror_face_on_door", True)

    # At closed pose, door covers the upper opening
    door_panel_bb = ctx.part_element_world_aabb(door, elem="door_panel")
    assert door_panel_bb is not None
    ctx.check("door_covers_upper_at_rest",
              door_panel_bb[1][2] > DIV_Z and door_panel_bb[0][2] < H - WALL,
              details=f"door z=({door_panel_bb[0][2]:.3f},{door_panel_bb[1][2]:.3f})")

    # At open pose, door swings outward (free edge moves toward +X)
    rest_pos = ctx.part_world_position(door)
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        open_pos = ctx.part_world_position(door)
        door_open_bb = ctx.part_element_world_aabb(door, elem="door_panel")
    assert rest_pos is not None and open_pos is not None and door_open_bb is not None
    # The door origin should move outward in X when opened
    ctx.check("door_opens_outward",
              door_open_bb[1][0] > FRONT_X + 0.02,
              details=f"open door max x={door_open_bb[1][0]:.4f}")

    # --- Tambour: prismatic joint sliding sideways ---
    ctx.check("tambour_is_prismatic",
              tambour_joint.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("tambour_slides_sideways",
              abs(tambour_joint.axis[1]) > 0.99,
              details=f"axis={tambour_joint.axis}")
    ctx.check("tambour_range_reasonable",
              tambour_joint.motion_limits.upper > 0.25
              and tambour_joint.motion_limits.lower == 0.0,
              details=f"range=({tambour_joint.motion_limits.lower},{tambour_joint.motion_limits.upper})")

    # At closed pose, tambour covers the lower opening
    tambour_bb = ctx.part_element_world_aabb(tambour, elem="tambour_panel")
    assert tambour_bb is not None
    ctx.check("tambour_covers_lower_at_rest",
              tambour_bb[0][2] < DIV_Z and tambour_bb[1][2] > WALL + 0.01,
              details=f"tambour z=({tambour_bb[0][2]:.3f},{tambour_bb[1][2]:.3f})")

    # At open pose, tambour slides to the right (+Y)
    tambour_rest = ctx.part_world_position(tambour)
    with ctx.pose({tambour_joint: tambour_joint.motion_limits.upper}):
        tambour_open = ctx.part_world_position(tambour)
        tambour_open_bb = ctx.part_element_world_aabb(tambour, elem="tambour_panel")
    assert tambour_rest is not None and tambour_open is not None
    assert tambour_open_bb is not None
    ctx.check("tambour_slides_right",
              tambour_open[1] > tambour_rest[1] + 0.25,
              details=f"rest y={tambour_rest[1]:.4f}, open y={tambour_open[1]:.4f}")

    # When tambour is open, lower shelves are exposed (tambour panel moved away)
    ctx.check("tambour_exposes_lower_when_open",
              tambour_open_bb[0][1] > W / 2.0 - 0.05,
              details=f"open tambour min y={tambour_open_bb[0][1]:.4f}")

    # --- At least two non-fixed joints ---
    all_joints = [door_joint, tambour_joint]
    non_fixed = [j for j in all_joints
                 if j.articulation_type != ArticulationType.FIXED]
    ctx.check("at_least_two_non_fixed_joints",
              len(non_fixed) >= 2,
              details=f"non_fixed={len(non_fixed)}")

    # --- Shelves visible through open gap: when tambour is open,
    # lower shelf is no longer occluded by the tambour on the Y axis ---
    with ctx.pose({tambour_joint: tambour_joint.motion_limits.upper}):
        shelf_open = ctx.part_element_world_aabb(carcass, elem="lower_shelf")
        assert shelf_open is not None
        # The shelf should still be at its position (carcass didn't move)
        # but the tambour is now out of the way
        ctx.check("lower_shelf_visible_when_tambour_open",
                  shelf_open[1][1] < tambour_open_bb[0][1],
                  details=f"shelf max y={shelf_open[1][1]:.3f}, tambour min y={tambour_open_bb[0][1]:.3f}")

    return ctx.report()


object_model = build_object_model()
