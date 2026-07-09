from __future__ import annotations

# Variant 14: Wide black wooden cabinet with sliding tambour panel and mirrored door.
# Forked from the double dresser into a distinct storage cabinet sibling.
# ~1.70 m W x 0.85 m H x 0.50 m D. Same carved posts, legs, silver-gray top.
#
# Upper compartment: bypass-style sliding tambour panel. A fixed panel covers
# the left half; the tambour panel (set back on a track) covers the right half
# at rest and slides along +Y behind the fixed panel to reveal the right-side
# shelves. PRISMATIC joint, axis=(0,+1,0), travel = half-opening width.
#
# Lower compartment: one mirrored door on a left-side vertical hinge (REVOLUTE
# about +Z), recessed panel borders frame a mirror panel set back from the
# border face. The right half of the lower opening is always open, showing
# internal shelf boards.
#
# Front faces +X, width along Y (centered), height along +Z, grounded at z=0.

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
BH = BODY_TOP - BODY_BOT         # ~0.678

WALL = 0.018
INNER_W = BW - 2 * WALL          # ~1.624

# Front opening zones
ZONE_BOT = BODY_BOT + 0.030      # 0.180
ZONE_TOP = BODY_TOP - 0.017      # 0.811
MID_Z = (ZONE_BOT + ZONE_TOP) / 2.0  # ~0.496

# Upper opening (between mid divider and top rail)
UPPER_BOT = MID_Z + WALL / 2.0   # ~0.505
UPPER_TOP = ZONE_TOP              # 0.811
UPPER_H = UPPER_TOP - UPPER_BOT  # ~0.306

# Lower opening (between bottom rail and mid divider)
LOWER_BOT = ZONE_BOT              # 0.180
LOWER_TOP = MID_Z - WALL / 2.0   # ~0.487
LOWER_H = LOWER_TOP - LOWER_BOT  # ~0.307

# Opening half-width (between stiles)
OPEN_HW = 0.762
FRONT_X = BD                      # 0.46

# --- Tambour (bypass sliding panel) ---
TAMBOUR_W = OPEN_HW - 0.006       # ~0.756, covers right half
TAMBOUR_H = UPPER_H - 0.008      # ~0.298
TAMBOUR_THK = 0.014
TAMBOUR_SETBACK = WALL + 0.004   # set back behind the front frame plane
TAMBOUR_TRAVEL = OPEN_HW          # slides one half-width behind fixed panel

# --- Mirror door ---
DOOR_W = OPEN_HW - 0.006          # ~0.756
DOOR_H = LOWER_H - 0.008         # ~0.299
DOOR_BACK_THK = 0.012
BORDER_THK = 0.006                # proud of back panel
RECESS_BW = 0.035                 # border strip width
MIRROR_THK = 0.003
HINGE_Y = OPEN_HW - 0.003        # left (+Y) edge of lower opening

# --- Shelves ---
SHELF_THK = 0.015
SHELF_D = BD - 2 * WALL - 0.040  # ~0.384, clear of tambour track

# --- Fixed upper panel (left half, part of carcass) ---
FIXED_W = OPEN_HW - 0.003        # ~0.759
FIXED_THK = WALL                  # same as front frame

# --- Carved posts ---
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG
LEG_SQ = 0.050

# Knob
KNOB_R = 0.013
STEM_R = 0.006
STEM_L = 0.016


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_cabinet_tambour_mirror")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    mirror_mat = model.material("mirror_glass", rgba=(0.82, 0.86, 0.90, 1.0))
    shelf_mat = model.material("shelf_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tambour_mat = model.material("tambour_wood", rgba=(0.09, 0.085, 0.08, 1.0))

    # ===================================================================
    # ROOT: carcass (shell, legs, posts, top, shelves, fixed panel)
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
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + WALL / 2.0)),
        material=black,
        name="bottom_board",
    )

    # Top stretcher board
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - WALL / 2.0)),
        material=black,
        name="top_stretcher",
    )

    # Mid-height divider
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, MID_Z)),
        material=black,
        name="mid_divider",
    )

    # Front top rail
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - ZONE_TOP)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (ZONE_TOP + BODY_TOP) / 2.0)),
        material=black,
        name="front_top_rail",
    )

    # Front bottom rail
    carcass.visual(
        Box((WALL, INNER_W, ZONE_BOT - BODY_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (BODY_BOT + ZONE_BOT) / 2.0)),
        material=black,
        name="front_bottom_rail",
    )

    # Front side stiles
    stile_w = BW / 2.0 - OPEN_HW + 0.004
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, stile_w, ZONE_TOP - ZONE_BOT)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (BW / 2.0 - stile_w / 2.0),
                               (ZONE_BOT + ZONE_TOP) / 2.0)),
            material=black,
            name=f"front_stile_{tag}",
        )

    # --- Fixed panel (left half of upper opening, part of carcass) ---
    carcass.visual(
        Box((FIXED_THK, FIXED_W, UPPER_H)),
        origin=Origin(xyz=(BD - WALL / 2.0,
                           OPEN_HW / 2.0 + 0.002,
                           (UPPER_BOT + UPPER_TOP) / 2.0)),
        material=black,
        name="fixed_upper_panel",
    )

    # --- Tambour track rails (horizontal, set back from front face) ---
    # Rails contact the tambour top/bottom edges to provide physical support.
    track_x = BD - TAMBOUR_SETBACK
    for tag, zpos in (("top", UPPER_TOP), ("bot", UPPER_BOT)):
        carcass.visual(
            Box((0.016, 2 * OPEN_HW + 0.010, 0.008)),
            origin=Origin(xyz=(track_x, 0.0, zpos)),
            material=black_deep,
            name=f"tambour_rail_{tag}",
        )

    # --- Shelves in upper compartment ---
    # Shelves span the full inner width (contacting side panels) and embed
    # 1 mm into the back panel for connectivity.
    shelf_x0 = WALL - 0.001  # back edge slightly embedded into back panel
    for i in range(2):
        sz = UPPER_BOT + (i + 1) * UPPER_H / 3.0
        carcass.visual(
            Box((SHELF_D, INNER_W, SHELF_THK)),
            origin=Origin(xyz=(shelf_x0 + SHELF_D / 2.0, 0.0, sz)),
            material=shelf_mat,
            name=f"upper_shelf_{i}",
        )

    # --- Shelves in lower compartment ---
    for i in range(2):
        sz = LOWER_BOT + (i + 1) * LOWER_H / 3.0
        carcass.visual(
            Box((SHELF_D, INNER_W, SHELF_THK)),
            origin=Origin(xyz=(shelf_x0 + SHELF_D / 2.0, 0.0, sz)),
            material=shelf_mat,
            name=f"lower_shelf_{i}",
        )

    # Silver-gray top slab
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
    # TAMBOUR PANEL: slides along +Y behind the fixed panel (bypass)
    # ===================================================================
    tambour = model.part("tambour_panel")

    # Tambour body - horizontal slat pattern
    n_slats = 10
    slat_gap = 0.002
    slat_h = (TAMBOUR_H - (n_slats - 1) * slat_gap) / n_slats
    for i in range(n_slats):
        sz = -TAMBOUR_H / 2.0 + i * (slat_h + slat_gap) + slat_h / 2.0
        tambour.visual(
            Box((TAMBOUR_THK, TAMBOUR_W - 0.004, slat_h)),
            origin=Origin(xyz=(0.0, 0.0, sz)),
            material=tambour_mat,
            name=f"slat_{i}",
        )

    # Backing strip connecting slats (narrower, behind)
    tambour.visual(
        Box((0.005, TAMBOUR_W - 0.040, TAMBOUR_H - 0.010)),
        origin=Origin(xyz=(-TAMBOUR_THK / 2.0 - 0.0025, 0.0, 0.0)),
        material=black_deep,
        name="tambour_backing",
    )

    # Pull handle at bottom center
    tambour.visual(
        Box((0.018, 0.100, 0.012)),
        origin=Origin(xyz=(TAMBOUR_THK / 2.0 + 0.009, 0.0,
                           -TAMBOUR_H / 2.0 + 0.006)),
        material=silver,
        name="tambour_handle",
    )

    tambour.inertial = Inertial.from_geometry(
        Box((TAMBOUR_THK, TAMBOUR_W, TAMBOUR_H)), mass=2.5)

    # Tambour articulation: PRISMATIC along +Y
    # At q=0: tambour centered on right half of upper opening
    # At q=TRAVEL: slides behind fixed panel, revealing right half
    tambour_cx = BD - TAMBOUR_SETBACK
    tambour_cy = -OPEN_HW / 2.0  # right half center
    tambour_cz = (UPPER_BOT + UPPER_TOP) / 2.0

    model.articulation(
        "carcass_to_tambour",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=tambour,
        origin=Origin(xyz=(tambour_cx, tambour_cy, tambour_cz)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.4,
                                   lower=0.0, upper=TAMBOUR_TRAVEL),
    )

    # ===================================================================
    # MIRROR DOOR: revolute on left-side vertical hinge
    # ===================================================================
    door = model.part("mirror_door")

    # Door back panel (structural)
    door.visual(
        Box((DOOR_BACK_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(DOOR_BACK_THK / 2.0, -DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_back",
    )

    # Recessed panel borders (four strips forming a frame, proud of mirror)
    bx = DOOR_BACK_THK + BORDER_THK / 2.0
    inner_w = DOOR_W - 2 * RECESS_BW
    inner_h = DOOR_H - 2 * RECESS_BW

    # Top border
    door.visual(
        Box((BORDER_THK, DOOR_W, RECESS_BW)),
        origin=Origin(xyz=(bx, -DOOR_W / 2.0, DOOR_H / 2.0 - RECESS_BW / 2.0)),
        material=black_deep,
        name="border_top",
    )
    # Bottom border
    door.visual(
        Box((BORDER_THK, DOOR_W, RECESS_BW)),
        origin=Origin(xyz=(bx, -DOOR_W / 2.0, -(DOOR_H / 2.0 - RECESS_BW / 2.0))),
        material=black_deep,
        name="border_bottom",
    )
    # Left border (hinge side, near y=0)
    door.visual(
        Box((BORDER_THK, RECESS_BW, inner_h)),
        origin=Origin(xyz=(bx, -RECESS_BW / 2.0, 0.0)),
        material=black_deep,
        name="border_left",
    )
    # Right border (free edge side, near y=-DOOR_W)
    door.visual(
        Box((BORDER_THK, RECESS_BW, inner_h)),
        origin=Origin(xyz=(bx, -(DOOR_W - RECESS_BW / 2.0), 0.0)),
        material=black_deep,
        name="border_right",
    )

    # Mirror panel (recessed behind borders)
    door.visual(
        Box((MIRROR_THK, inner_w, inner_h)),
        origin=Origin(xyz=(DOOR_BACK_THK + MIRROR_THK / 2.0, -DOOR_W / 2.0, 0.0)),
        material=mirror_mat,
        name="mirror_panel",
    )

    # Door knob on free edge
    knob_y = -(DOOR_W - 0.030)
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0 + DOOR_BACK_THK + BORDER_THK,
                           knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="door_knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(DOOR_BACK_THK + BORDER_THK + STEM_L + KNOB_R - 0.004,
                           knob_y, 0.0)),
        material=silver,
        name="door_knob_ball",
    )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_BACK_THK + BORDER_THK, DOOR_W, DOOR_H)), mass=3.5)

    # Door articulation: REVOLUTE about +Z at left hinge edge
    # Door extends in -Y from hinge. Positive q swings free edge toward +X (outward).
    door_cz = (LOWER_BOT + LOWER_TOP) / 2.0
    model.articulation(
        "carcass_to_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(BD, HINGE_Y, door_cz)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0,
                                   lower=0.0, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    tambour = object_model.get_part("tambour_panel")
    door = object_model.get_part("mirror_door")
    tambour_joint = object_model.get_articulation("carcass_to_tambour")
    door_joint = object_model.get_articulation("carcass_to_door")

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
    assert top is not None and side is not None
    ctx.check("top_overhang",
              top[1][1] > side[1][1] + 0.015 and top[0][1] < -side[1][1] - 0.015,
              details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f})")

    # --- Tambour is PRISMATIC along +Y ---
    ctx.check("tambour_prismatic",
              tambour_joint.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("tambour_axis_y",
              abs(tambour_joint.axis[1] - 1.0) < 0.01
              and abs(tambour_joint.axis[0]) < 0.01
              and abs(tambour_joint.axis[2]) < 0.01)
    ctx.check("tambour_travel_positive",
              tambour_joint.motion_limits.upper > 0.30,
              details=f"upper={tambour_joint.motion_limits.upper:.3f}")

    # --- Tambour slides to reveal right half ---
    rest_pos = ctx.part_world_position(tambour)
    with ctx.pose({tambour_joint: tambour_joint.motion_limits.upper}):
        open_pos = ctx.part_world_position(tambour)
    assert rest_pos is not None and open_pos is not None
    ctx.check("tambour_slides_along_y",
              open_pos[1] > rest_pos[1] + 0.30,
              details=f"rest y={rest_pos[1]:.3f}, open y={open_pos[1]:.3f}")
    # Tambour X stays constant (pure Y slide)
    ctx.check("tambour_no_x_drift",
              abs(open_pos[0] - rest_pos[0]) < 0.002,
              details=f"dx={open_pos[0] - rest_pos[0]:.4f}")

    # --- Tambour has slat pattern ---
    slat0 = ctx.part_element_world_aabb(tambour, elem="slat_0")
    slat9 = ctx.part_element_world_aabb(tambour, elem="slat_9")
    assert slat0 is not None and slat9 is not None
    ctx.check("tambour_has_slats",
              abs(slat9[0][2] - slat0[1][2]) > 0.05,
              details=f"slat0 top={slat0[1][2]:.3f}, slat9 bot={slat9[0][2]:.3f}")

    # --- Mirror door is REVOLUTE about Z ---
    ctx.check("door_revolute",
              door_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("door_axis_z",
              abs(door_joint.axis[2]) > 0.99)
    ctx.check("door_range_sufficient",
              door_joint.motion_limits.upper > 1.0,
              details=f"upper={door_joint.motion_limits.upper:.2f}")

    # --- Door swings outward (free edge moves +X) ---
    door_back_rest = ctx.part_element_world_aabb(door, elem="door_back")
    assert door_back_rest is not None
    rest_front_x = door_back_rest[1][0]
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        door_back_open = ctx.part_element_world_aabb(door, elem="door_back")
        assert door_back_open is not None
        open_max_x = door_back_open[1][0]
    ctx.check("door_sways_outward",
              open_max_x > rest_front_x + 0.05,
              details=f"rest front x={rest_front_x:.3f}, "
                      f"open max x={open_max_x:.3f}")

    # --- Recessed panel borders on door ---
    border_t = ctx.part_element_world_aabb(door, elem="border_top")
    border_b = ctx.part_element_world_aabb(door, elem="border_bottom")
    mirror = ctx.part_element_world_aabb(door, elem="mirror_panel")
    door_back = ctx.part_element_world_aabb(door, elem="door_back")
    assert (border_t is not None and border_b is not None
            and mirror is not None and door_back is not None)

    # Borders are proud of mirror (farther +X)
    ctx.check("borders_proud_of_mirror",
              border_t[1][0] > mirror[1][0] + 0.001,
              details=f"border front x={border_t[1][0]:.4f}, "
                      f"mirror front x={mirror[1][0]:.4f}")

    # Top border is above bottom border
    ctx.check("borders_frame_door",
              border_t[0][2] > border_b[1][2],
              details=f"top bot z={border_t[0][2]:.3f}, "
                      f"bottom top z={border_b[1][2]:.3f}")

    # Mirror is within the door back panel area (Y/Z)
    ctx.expect_within(door, door, axes="y",
                      inner_elem="mirror_panel", outer_elem="door_back",
                      margin=0.001,
                      name="mirror_within_door_y")

    # --- Shelves exist in both compartments ---
    for name in ("upper_shelf_0", "upper_shelf_1", "lower_shelf_0", "lower_shelf_1"):
        shelf = ctx.part_element_world_aabb(carcass, elem=name)
        assert shelf is not None
        ctx.check(f"{name}_exists", shelf[1][2] - shelf[0][2] < 0.030,
                  details=f"thickness={shelf[1][2] - shelf[0][2]:.4f}")

    # Upper shelves within upper compartment Z range
    us0 = ctx.part_element_world_aabb(carcass, elem="upper_shelf_0")
    us1 = ctx.part_element_world_aabb(carcass, elem="upper_shelf_1")
    assert us0 is not None and us1 is not None
    ctx.check("upper_shelves_in_range",
              us0[0][2] > UPPER_BOT and us1[1][2] < UPPER_TOP,
              details=f"z0={us0[0][2]:.3f}, z1={us1[1][2]:.3f}")

    # Lower shelves within lower compartment Z range
    ls0 = ctx.part_element_world_aabb(carcass, elem="lower_shelf_0")
    ls1 = ctx.part_element_world_aabb(carcass, elem="lower_shelf_1")
    assert ls0 is not None and ls1 is not None
    ctx.check("lower_shelves_in_range",
              ls0[0][2] > LOWER_BOT and ls1[1][2] < LOWER_TOP,
              details=f"z0={ls0[0][2]:.3f}, z1={ls1[1][2]:.3f}")

    # --- Fixed upper panel on left half ---
    fixed = ctx.part_element_world_aabb(carcass, elem="fixed_upper_panel")
    assert fixed is not None
    ctx.check("fixed_panel_left_side",
              fixed[0][1] > 0.0,
              details=f"fixed y=({fixed[0][1]:.3f},{fixed[1][1]:.3f})")

    # --- Carved posts present ---
    seg0 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_0")
    assert seg0 is not None
    ctx.check("posts_present", seg0[0][2] < LEG_H + 0.005)

    # --- At least two non-fixed joints ---
    all_joints = [tambour_joint, door_joint]
    non_fixed = [j for j in all_joints
                 if j.articulation_type in (ArticulationType.REVOLUTE,
                                            ArticulationType.PRISMATIC,
                                            ArticulationType.CONTINUOUS)]
    ctx.check("at_least_two_joints", len(non_fixed) >= 2,
              details=f"non_fixed={len(non_fixed)}")

    return ctx.report()


object_model = build_object_model()
