from __future__ import annotations

# Wide black wooden cabinet variant (~1.70 m W x 0.85 m H x 0.50 m D).
#
# Storage cabinet with:
# - Two swing doors (revolute hinges) on the lower front
# - A sliding tambour-style panel on the upper front
# - Interior shelf boards visible through openings
# - Small gap seams around all moving fronts
#
# World layout: front faces +X (back at x=0, front at x=BD),
# width along Y (centered), height along +Z, grounded at z=0.
# Matte black wood carcass; silver-gray overhanging top slab.
# Decorative carved corner posts with stacked spiral/zigzag pattern
# continuing down into four square legs ~0.15 m tall.

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
BODY_BOT = LEG_H                  # 0.150
BODY_TOP = H_TOTAL - TOP_THK     # 0.828
BH = BODY_TOP - BODY_BOT         # 0.678

WALL = 0.018
INNER_W = BW - 2 * WALL          # 1.624
INNER_D = BD - WALL              # 0.442

# Front opening zone
ZONE_BOT = BODY_BOT + 0.030      # 0.180
ZONE_TOP = BODY_TOP - 0.017      # 0.811

# --- Door zone (lower ~57% of opening) ---
DOOR_ZONE_H = 0.380
DOOR_BOT = ZONE_BOT              # 0.180
DOOR_ZONE_TOP = DOOR_BOT + DOOR_ZONE_H  # 0.560
DOOR_GAP_V = 0.000
DOOR_H = DOOR_ZONE_H - 2 * DOOR_GAP_V  # 0.380 (full zone, contacts rails)
DOOR_CZ = DOOR_BOT + DOOR_GAP_V + DOOR_H / 2.0  # 0.370

# --- Divider rail ---
DIVIDER_H = 0.022
DIVIDER_BOT = DOOR_ZONE_TOP      # 0.560
DIVIDER_TOP = DIVIDER_BOT + DIVIDER_H  # 0.582

# --- Tambour zone (upper) ---
TAMB_BOT = DIVIDER_TOP           # 0.582
TAMB_TOP = ZONE_TOP              # 0.811
TAMB_H = TAMB_TOP - TAMB_BOT    # 0.229
TAMB_CZ = (TAMB_BOT + TAMB_TOP) / 2.0  # 0.6965
TAMB_PANEL_H = TAMB_H             # full zone, contacts rails
TAMB_THK = 0.012

# --- Face frame and hinge positions ---
# Hinges sit inward from the carved post inner face (posts extend to ~0.772
# due to ±25° rotation of 0.050 stock) so doors/tambour clear the posts.
STILE_FACE_W = 0.052
HINGE_Y = INNER_W / 2.0 - STILE_FACE_W  # 0.760

DOOR_GAP_H = 0.004
DOOR_W = HINGE_Y - DOOR_GAP_H   # 0.756
FACE_THK = 0.018
FACE_PROUD = 0.000

# Tambour panel covers half the opening, slides to the other half
TAMB_W = HINGE_Y                 # 0.760
TAMB_TRAVEL = HINGE_Y            # 0.760

FRONT_X = BD                     # 0.46

# Carved corner posts
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025      # 0.805
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG

LEG_SQ = 0.050

# Shelf
SHELF_THK = 0.014
SHELF_D = INNER_D - 0.020       # 0.422
SHELF_W = INNER_W - 0.010       # 1.614

# Knobs
KNOB_R = 0.0125
STEM_R = 0.0055
STEM_L = 0.014


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_cabinet_tambour_doors")

    # --- materials ---
    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    shelf_mat = model.material("shelf_black", rgba=(0.10, 0.10, 0.11, 1.0))
    panel_inner = model.material("panel_recessed", rgba=(0.060, 0.060, 0.065, 1.0))

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
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + 0.009)),
        material=black,
        name="bottom_board",
    )

    # Top stretcher board
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black,
        name="top_stretcher",
    )

    # Front bottom rail (below door zone)
    carcass.visual(
        Box((WALL, INNER_W, ZONE_BOT - BODY_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (BODY_BOT + ZONE_BOT) / 2.0)),
        material=black,
        name="front_bottom_rail",
    )

    # Front top rail (above tambour zone)
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - ZONE_TOP)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (ZONE_TOP + BODY_TOP) / 2.0)),
        material=black,
        name="front_top_rail",
    )

    # Horizontal divider rail (between doors and tambour)
    carcass.visual(
        Box((WALL, INNER_W, DIVIDER_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (DIVIDER_BOT + DIVIDER_TOP) / 2.0)),
        material=black,
        name="divider_rail",
    )

    # Face frame side stiles (thin trim at opening edges, door zone only)
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, STILE_FACE_W, DOOR_ZONE_H)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (INNER_W / 2.0 - STILE_FACE_W / 2.0),
                               (DOOR_BOT + DOOR_ZONE_TOP) / 2.0)),
            material=black_deep,
            name=f"door_stile_{tag}",
        )

    # Face frame side stiles (tambour zone)
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, STILE_FACE_W, TAMB_H)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (INNER_W / 2.0 - STILE_FACE_W / 2.0),
                               TAMB_CZ)),
            material=black_deep,
            name=f"tambour_stile_{tag}",
        )

    # Center stile behind door meeting point
    carcass.visual(
        Box((WALL, 0.020, DOOR_H + 2 * DOOR_GAP_V)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, DOOR_CZ)),
        material=black_deep,
        name="center_stile",
    )

    # Silver-gray top slab with overhang
    carcass.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=silver_top,
        name="top_slab",
    )

    # Carved spiral/zigzag corner posts
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

    # Four straight square legs
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

    # Interior shelf boards (visible through open doors and tambour)
    shelf_cx = WALL + SHELF_D / 2.0 + 0.010
    shelf_positions = [
        ("shelf_lower", DOOR_BOT + DOOR_GAP_V + DOOR_H * 0.30),
        ("shelf_upper", DOOR_BOT + DOOR_GAP_V + DOOR_H * 0.70),
        ("shelf_tambour", TAMB_CZ),
    ]
    for shelf_name, shelf_z in shelf_positions:
        carcass.visual(
            Box((SHELF_D, SHELF_W, SHELF_THK)),
            origin=Origin(xyz=(shelf_cx, 0.0, shelf_z)),
            material=shelf_mat,
            name=shelf_name,
        )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=55.0)

    # ===================================================================
    # LEFT DOOR
    # ===================================================================
    left_door = model.part("left_door")

    # Door panel: hinge edge at part frame, extends in +Y
    left_door.visual(
        Box((FACE_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(FACE_THK / 2.0 + FACE_PROUD, DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )

    # Recessed inner panel (Shaker-style detail)
    recess_margin = 0.045
    recess_w = DOOR_W - 2 * recess_margin
    recess_h = DOOR_H - 2 * recess_margin
    left_door.visual(
        Box((0.003, recess_w, recess_h)),
        origin=Origin(xyz=(FACE_THK + FACE_PROUD + 0.0015, DOOR_W / 2.0, 0.0)),
        material=panel_inner,
        name="door_recess",
    )

    # Knob near the free edge
    knob_y = DOOR_W - 0.060
    left_door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0 + FACE_THK + FACE_PROUD,
                           knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    left_door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004 + FACE_THK + FACE_PROUD,
                           knob_y, 0.0)),
        material=silver,
        name="knob_ball",
    )

    left_door.inertial = Inertial.from_geometry(
        Box((FACE_THK, DOOR_W, DOOR_H)), mass=3.0)

    # ===================================================================
    # RIGHT DOOR
    # ===================================================================
    right_door = model.part("right_door")

    right_door.visual(
        Box((FACE_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(FACE_THK / 2.0 + FACE_PROUD, -DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )

    recess_w = DOOR_W - 2 * recess_margin
    recess_h = DOOR_H - 2 * recess_margin
    right_door.visual(
        Box((0.003, recess_w, recess_h)),
        origin=Origin(xyz=(FACE_THK + FACE_PROUD + 0.0015, -DOOR_W / 2.0, 0.0)),
        material=panel_inner,
        name="door_recess",
    )

    knob_y_r = -(DOOR_W - 0.060)
    right_door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0 + FACE_THK + FACE_PROUD,
                           knob_y_r, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    right_door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004 + FACE_THK + FACE_PROUD,
                           knob_y_r, 0.0)),
        material=silver,
        name="knob_ball",
    )

    right_door.inertial = Inertial.from_geometry(
        Box((FACE_THK, DOOR_W, DOOR_H)), mass=3.0)

    # ===================================================================
    # TAMBOUR PANEL
    # ===================================================================
    tambour = model.part("tambour")

    # Main sliding panel
    tambour.visual(
        Box((TAMB_THK, TAMB_W, TAMB_PANEL_H)),
        origin=Origin(xyz=(TAMB_THK / 2.0 + FACE_PROUD, 0.0, 0.0)),
        material=black,
        name="tambour_panel",
    )

    # Horizontal groove lines suggesting tambour slats
    n_grooves = 7
    groove_zone = TAMB_PANEL_H - 0.020
    groove_spacing = groove_zone / max(n_grooves - 1, 1)
    for i in range(n_grooves):
        gz = -groove_zone / 2.0 + i * groove_spacing
        tambour.visual(
            Box((0.001, TAMB_W - 0.020, 0.003)),
            origin=Origin(xyz=(TAMB_THK + FACE_PROUD + 0.0005, 0.0, gz)),
            material=black_deep,
            name=f"tambour_groove_{i}",
        )

    # Pull handle near the trailing edge
    handle_y = TAMB_W / 2.0 - 0.040
    tambour.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0 + TAMB_THK + FACE_PROUD,
                           handle_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="handle_stem",
    )
    tambour.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004 + TAMB_THK + FACE_PROUD,
                           handle_y, 0.0)),
        material=silver,
        name="handle_ball",
    )

    tambour.inertial = Inertial.from_geometry(
        Box((TAMB_THK, TAMB_W, TAMB_PANEL_H)), mass=2.0)

    # ===================================================================
    # ARTICULATIONS
    # ===================================================================

    # Left door: revolute hinge at left edge, axis -Z so positive q opens outward
    model.articulation(
        "carcass_to_left_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(FRONT_X, -HINGE_Y, DOOR_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5,
                                   lower=0.0, upper=1.4),
    )

    # Right door: revolute hinge at right edge, axis +Z so positive q opens outward
    model.articulation(
        "carcass_to_right_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(FRONT_X, HINGE_Y, DOOR_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5,
                                   lower=0.0, upper=1.4),
    )

    # Tambour: prismatic along +Y, slides from left half to right half
    model.articulation(
        "carcass_to_tambour",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=tambour,
        origin=Origin(xyz=(FRONT_X, -(TAMB_W / 2.0), TAMB_CZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.5,
                                   lower=0.0, upper=TAMB_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    tambour = object_model.get_part("tambour")

    left_joint = object_model.get_articulation("carcass_to_left_door")
    right_joint = object_model.get_articulation("carcass_to_right_door")
    tamb_joint = object_model.get_articulation("carcass_to_tambour")

    # --- Overall dimensions ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("legs_on_floor", abs(cb[0][2]) < 0.003,
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

    # --- Silver top slab overhangs ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert top is not None and side is not None and back is not None
    ctx.check(
        "top_overhang_all_sides",
        top[1][1] > side[1][1] + 0.015 and top[0][1] < -side[1][1] - 0.015
        and top[0][0] < back[0][0] - 0.015 and top[1][0] > 0.46 + 0.015,
        details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f}) "
                f"x=({top[0][0]:.3f},{top[1][0]:.3f})",
    )

    # --- Carved posts stacked and rotated ---
    seg0 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_0")
    seg1 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_1")
    assert seg0 is not None and seg1 is not None
    w0 = seg0[1][1] - seg0[0][1]
    ctx.check("post_segments_rotated", w0 > POST_SQ + 0.008,
              details=f"seg aabb width={w0:.4f}")
    ctx.check("post_segments_stacked", abs(seg1[0][2] - seg0[1][2]) < 0.004,
              details=f"seg1 bot={seg1[0][2]:.4f}, seg0 top={seg0[1][2]:.4f}")

    # --- Three non-fixed joints exist ---
    ctx.check("left_door_revolute",
              left_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("right_door_revolute",
              right_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("tambour_prismatic",
              tamb_joint.articulation_type == ArticulationType.PRISMATIC)

    # --- Door hinge axes and limits ---
    ctx.check("left_door_axis_neg_z",
              left_joint.axis[2] < -0.99
              and abs(left_joint.axis[0]) < 0.01
              and abs(left_joint.axis[1]) < 0.01,
              details=f"axis={left_joint.axis}")
    ctx.check("right_door_axis_pos_z",
              right_joint.axis[2] > 0.99
              and abs(right_joint.axis[0]) < 0.01
              and abs(right_joint.axis[1]) < 0.01,
              details=f"axis={right_joint.axis}")
    for jname, j in (("left_door", left_joint), ("right_door", right_joint)):
        ctx.check(f"{jname}_limits",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 1.4) < 0.01,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Tambour axis and travel ---
    ctx.check("tambour_axis_pos_y",
              tamb_joint.axis[1] > 0.99
              and abs(tamb_joint.axis[0]) < 0.01
              and abs(tamb_joint.axis[2]) < 0.01,
              details=f"axis={tamb_joint.axis}")
    ctx.check("tambour_travel",
              abs(tamb_joint.motion_limits.lower) < 1e-9
              and tamb_joint.motion_limits.upper > 0.70,
              details=f"upper={tamb_joint.motion_limits.upper:.3f}")

    # --- Shelves exist inside carcass ---
    for sname in ("shelf_lower", "shelf_upper", "shelf_tambour"):
        shelf_aabb = ctx.part_element_world_aabb(carcass, elem=sname)
        assert shelf_aabb is not None
        ctx.check(f"{sname}_inside_carcass",
                  shelf_aabb[0][0] > 0.01 and shelf_aabb[1][0] < BD - 0.005,
                  details=f"x=({shelf_aabb[0][0]:.3f},{shelf_aabb[1][0]:.3f})")

    # --- Door gap seams at rest (q=0) ---
    left_face = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_face = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert left_face is not None and right_face is not None

    # Center gap between doors
    center_gap = right_face[0][1] - left_face[1][1]
    ctx.check("door_center_gap_seam",
              0.003 < center_gap < 0.020,
              details=f"gap={center_gap:.4f}")

    # Doors proud of carcass face
    carcass_front_x = BD
    ctx.check("left_door_proud",
              left_face[0][0] > carcass_front_x - 0.001,
              details=f"door min x={left_face[0][0]:.4f}")
    ctx.check("right_door_proud",
              right_face[0][0] > carcass_front_x - 0.001,
              details=f"door min x={right_face[0][0]:.4f}")

    # Tambour panel gap seams
    tamb_face = ctx.part_element_world_aabb(tambour, elem="tambour_panel")
    assert tamb_face is not None
    ctx.check("tambour_proud",
              tamb_face[0][0] > carcass_front_x - 0.001,
              details=f"tambour min x={tamb_face[0][0]:.4f}")

    # --- Doors swing outward at positive q ---
    left_knob_rest = ctx.part_element_world_aabb(left_door, elem="knob_ball")
    assert left_knob_rest is not None
    rest_x_left = left_knob_rest[1][0]
    with ctx.pose({left_joint: 1.0}):
        left_knob_open = ctx.part_element_world_aabb(left_door, elem="knob_ball")
        assert left_knob_open is not None
        open_x_left = left_knob_open[1][0]
    ctx.check("left_door_opens_outward",
              open_x_left > rest_x_left + 0.15,
              details=f"rest x={rest_x_left:.3f}, open x={open_x_left:.3f}")

    right_knob_rest = ctx.part_element_world_aabb(right_door, elem="knob_ball")
    assert right_knob_rest is not None
    rest_x_right = right_knob_rest[1][0]
    with ctx.pose({right_joint: 1.0}):
        right_knob_open = ctx.part_element_world_aabb(right_door, elem="knob_ball")
        assert right_knob_open is not None
        open_x_right = right_knob_open[1][0]
    ctx.check("right_door_opens_outward",
              open_x_right > rest_x_right + 0.15,
              details=f"rest x={rest_x_right:.3f}, open x={open_x_right:.3f}")

    # --- Tambour slides along Y at positive q ---
    tamb_rest = ctx.part_element_world_aabb(tambour, elem="tambour_panel")
    assert tamb_rest is not None
    rest_y_center = (tamb_rest[0][1] + tamb_rest[1][1]) / 2.0
    with ctx.pose({tamb_joint: tamb_joint.motion_limits.upper}):
        tamb_open = ctx.part_element_world_aabb(tambour, elem="tambour_panel")
        assert tamb_open is not None
        open_y_center = (tamb_open[0][1] + tamb_open[1][1]) / 2.0
    ctx.check("tambour_slides_along_y",
              open_y_center > rest_y_center + 0.50,
              details=f"rest y={rest_y_center:.3f}, open y={open_y_center:.3f}")

    # --- Tambour reveals shelf when open ---
    shelf_aabb = ctx.part_element_world_aabb(carcass, elem="shelf_tambour")
    assert shelf_aabb is not None
    tamb_rest_center = (tamb_face[0][1] + tamb_face[1][1]) / 2.0
    with ctx.pose({tamb_joint: tamb_joint.motion_limits.upper}):
        tamb_at_open = ctx.part_element_world_aabb(tambour, elem="tambour_panel")
        assert tamb_at_open is not None
        open_center = (tamb_at_open[0][1] + tamb_at_open[1][1]) / 2.0
        # Panel has slid right, leaving the left portion of the opening uncovered
        ctx.check("tambour_reveals_left_shelf",
                  open_center > tamb_rest_center + 0.50
                  and tamb_at_open[0][1] > -0.020,
                  details=f"rest center y={tamb_rest_center:.3f}, "
                          f"open center y={open_center:.3f}, "
                          f"min y={tamb_at_open[0][1]:.3f}")

    # --- Shelf accessible through open doors ---
    left_rest_panel = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_rest_panel = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert left_rest_panel is not None and right_rest_panel is not None
    rest_max_x_left = left_rest_panel[1][0]
    rest_max_x_right = right_rest_panel[1][0]
    with ctx.pose({left_joint: 1.2, right_joint: 1.2}):
        left_swing = ctx.part_element_world_aabb(left_door, elem="door_panel")
        right_swing = ctx.part_element_world_aabb(right_door, elem="door_panel")
        assert left_swing is not None and right_swing is not None
        # Door free edges have swung well outward past the carcass front
        ctx.check("doors_swing_clear_shelf_zone",
                  left_swing[1][0] > rest_max_x_left + 0.20
                  and right_swing[1][0] > rest_max_x_right + 0.20,
                  details=f"left max x: {rest_max_x_left:.3f}->{left_swing[1][0]:.3f}, "
                          f"right max x: {rest_max_x_right:.3f}->{right_swing[1][0]:.3f}")

    return ctx.report()


object_model = build_object_model()
