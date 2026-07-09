from __future__ import annotations

# Variant 16: Wide black wooden cabinet (~1.70 m W x 0.85 m H x 0.50 m D).
#
# Three-section front layout: open side cubbies (-Y and +Y) flanking a closed
# central cabinet. The center has a tambour front that slides sideways along
# the width axis (+Y) on a prismatic joint, revealing interior shelf boards.
# The side cubbies are permanently open, showing their shelf boards through
# the front gap. Small door-gap seams surround the tambour panel when closed.
#
# Retained from parent: matte black wood carcass, silver-gray overhanging top,
# carved spiral/zigzag corner posts, four straight square legs.

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

BW = W_TOTAL - 2 * OVERHANG   # 1.66  body width
BD = D_TOTAL - 2 * OVERHANG   # 0.46  body depth
LEG_H = 0.150
BODY_BOT = LEG_H              # 0.150
BODY_TOP = H_TOTAL - TOP_THK  # 0.828
BH = BODY_TOP - BODY_BOT      # 0.678

WALL = 0.018
INNER_W = BW - 2 * WALL       # 1.624

# Front opening zone (between bottom and top rails)
OPEN_BOT = BODY_BOT + 0.030   # 0.180
OPEN_TOP = BODY_TOP - 0.017   # 0.811
OPEN_H = OPEN_TOP - OPEN_BOT  # 0.631

# Three-section layout along Y (width)
CENTER_W = 0.500              # center cabinet opening width
DIV_W = 0.018                 # divider panel thickness
CUBBY_W = (INNER_W - CENTER_W - 2 * DIV_W) / 2.0  # ~0.544

# Divider center Y positions (symmetric about y=0)
DIV_Y = CENTER_W / 2.0 + DIV_W / 2.0  # 0.259

FRONT_X = BD                  # 0.46  carcass front plane

# Tambour sliding door
TAMBOUR_THK = 0.015
TAMBOUR_GAP = 0.004           # reveal gap (Y sides only; Z filled by tracks)
TAMBOUR_W = CENTER_W - 2 * TAMBOUR_GAP   # 0.492
# Track rails at top/bottom provide the sliding surface and Z contact
TRACK_THK = 0.003             # track rail thickness
TRACK_D = 0.020               # track rail depth (extends forward from front)
# Tambour panel spans between the tracks (Z contact at both track faces)
TAMBOUR_BOT_Z = OPEN_BOT + TRACK_THK     # 0.183
TAMBOUR_TOP_Z = OPEN_TOP - TRACK_THK     # 0.808
TAMBOUR_H = TAMBOUR_TOP_Z - TAMBOUR_BOT_Z  # 0.625
TAMBOUR_CZ = (TAMBOUR_BOT_Z + TAMBOUR_TOP_Z) / 2.0  # 0.4955
# Tambour back face flush with carcass front (tracks bridge the gap)
TAMBOUR_X = FRONT_X + TAMBOUR_THK / 2.0  # 0.4675
TAMBOUR_TRAVEL = 0.52         # enough to clear center opening

# Shelves
SHELF_THK = 0.016
N_SHELVES = 2

# Carved corner posts and legs (from parent)
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025   # 0.805
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG
LEG_SQ = 0.050


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_cabinet_tambour_cubbies")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    shelf_mat = model.material("shelf_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tambour_mat = model.material("tambour_slat", rgba=(0.085, 0.085, 0.09, 1.0))

    # ===================================================================
    # CARCASS (root): hollow shell, dividers, shelves, posts, legs, top
    # ===================================================================
    carcass = model.part("carcass")

    # Side panels
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD, WALL, BH)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                               BODY_BOT + BH / 2.0)),
            material=black, name=f"side_panel_{tag}",
        )

    # Back panel
    carcass.visual(
        Box((WALL, INNER_W, BH)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=black, name="back_panel",
    )

    # Bottom board
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + WALL / 2.0)),
        material=black, name="bottom_board",
    )

    # Top stretcher board
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - WALL / 2.0)),
        material=black, name="top_stretcher",
    )

    # Front bottom rail (below all front openings)
    carcass.visual(
        Box((WALL, INNER_W, OPEN_BOT - BODY_BOT)),
        origin=Origin(xyz=(FRONT_X - WALL / 2.0, 0.0,
                           (BODY_BOT + OPEN_BOT) / 2.0)),
        material=black, name="front_bottom_rail",
    )

    # Front top rail (above all front openings)
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - OPEN_TOP)),
        origin=Origin(xyz=(FRONT_X - WALL / 2.0, 0.0,
                           (OPEN_TOP + BODY_TOP) / 2.0)),
        material=black, name="front_top_rail",
    )

    # Tambour sliding tracks (top and bottom rails at center opening)
    # These extend forward from the carcass front to support the tambour panel.
    track_cx = FRONT_X - WALL + TRACK_D / 2.0  # extends from rail front forward
    for track_tag, track_cz in (
        ("bottom", OPEN_BOT + TRACK_THK / 2.0),
        ("top", OPEN_TOP - TRACK_THK / 2.0),
    ):
        carcass.visual(
            Box((TRACK_D, CENTER_W, TRACK_THK)),
            origin=Origin(xyz=(track_cx, 0.0, track_cz)),
            material=black_deep, name=f"tambour_track_{track_tag}",
        )

    # Vertical dividers separating center from side cubbies
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD - WALL, DIV_W, OPEN_H)),
            origin=Origin(xyz=(WALL + (BD - WALL) / 2.0,
                               s * DIV_Y,
                               (OPEN_BOT + OPEN_TOP) / 2.0)),
            material=black, name=f"divider_{tag}",
        )

    # ----- Shelves -----
    shelf_zs = [OPEN_BOT + OPEN_H * (i + 1) / (N_SHELVES + 1)
                for i in range(N_SHELVES)]

    # Cubby internal bounds
    inner_div_y = DIV_Y + DIV_W / 2.0        # 0.268
    inner_side_y = BW / 2.0 - WALL           # 0.812
    cubby_cy = (inner_div_y + inner_side_y) / 2.0  # 0.540
    cubby_w = inner_side_y - inner_div_y      # 0.544

    shelf_depth = BD - 2 * WALL               # 0.424
    shelf_cx = WALL + shelf_depth / 2.0       # 0.230

    # Side cubby shelves (visible through open front)
    for tag, s in (("0", 1), ("1", -1)):
        for i, sz in enumerate(shelf_zs):
            carcass.visual(
                Box((shelf_depth, cubby_w, SHELF_THK)),
                origin=Origin(xyz=(shelf_cx, s * cubby_cy, sz)),
                material=shelf_mat, name=f"cubby_{tag}_shelf_{i}",
            )

    # Center cabinet shelves (visible when tambour is open)
    for i, sz in enumerate(shelf_zs):
        carcass.visual(
            Box((shelf_depth, CENTER_W, SHELF_THK)),
            origin=Origin(xyz=(shelf_cx, 0.0, sz)),
            material=shelf_mat, name=f"center_shelf_{i}",
        )

    # ----- Silver-gray top slab with overhang -----
    carcass.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=silver_top, name="top_slab",
    )

    # ----- Carved spiral/zigzag corner posts -----
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
                material=black_deep, name=f"carved_post_{ptag}_seg_{i}",
            )

    # ----- Four straight square legs -----
    for tag, lx, ly in (("front_0", POST_CX - 0.012, POST_CY),
                        ("front_1", POST_CX - 0.012, -POST_CY),
                        ("rear_0", 0.030, POST_CY),
                        ("rear_1", 0.030, -POST_CY)):
        carcass.visual(
            Box((LEG_SQ, LEG_SQ, LEG_H + 0.004)),
            origin=Origin(xyz=(lx, ly, (LEG_H + 0.004) / 2.0)),
            material=black, name=f"leg_{tag}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=55.0)

    # ===================================================================
    # TAMBOUR DOOR: slides sideways (+Y) on prismatic joint
    # ===================================================================
    tambour = model.part("tambour")

    # Main tambour panel
    tambour.visual(
        Box((TAMBOUR_THK, TAMBOUR_W, TAMBOUR_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tambour_mat, name="tambour_panel",
    )

    # Horizontal slat grooves to suggest tambour slat construction
    n_slats = 10
    slat_gap = 0.020
    slat_zone = TAMBOUR_H - 2 * slat_gap
    slat_spacing = slat_zone / (n_slats - 1) if n_slats > 1 else slat_zone
    for i in range(n_slats):
        sz = -(TAMBOUR_H / 2.0 - slat_gap) + i * slat_spacing
        tambour.visual(
            Box((0.003, TAMBOUR_W - 0.012, 0.002)),
            # Embed 1 mm into the panel front face for connectivity
            origin=Origin(xyz=(TAMBOUR_THK / 2.0 - 0.001, 0.0, sz)),
            material=black_deep, name=f"slat_groove_{i}",
        )

    # Small silver pull handle on the -Y edge (trailing edge for +Y slide)
    stem_len = 0.014
    handle_y = -(TAMBOUR_W / 2.0 - 0.040)
    tambour.visual(
        Cylinder(radius=0.005, length=stem_len + 0.004),
        origin=Origin(xyz=((stem_len - 0.004) / 2.0, handle_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver, name="handle_stem",
    )
    tambour.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(stem_len + 0.012 - 0.004, handle_y, 0.0)),
        material=silver, name="handle_ball",
    )

    tambour.inertial = Inertial.from_geometry(
        Box((TAMBOUR_THK, TAMBOUR_W, TAMBOUR_H)), mass=3.0)

    # Tambour articulation: prismatic along +Y (sideways slide)
    model.articulation(
        "carcass_to_tambour",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=tambour,
        origin=Origin(xyz=(TAMBOUR_X, 0.0, TAMBOUR_CZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.3,
                                   lower=0.0, upper=TAMBOUR_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    tambour = object_model.get_part("tambour")
    tambour_joint = object_model.get_articulation("carcass_to_tambour")

    # --- Overall dimensions preserved from parent ---
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

    # --- Silver top overhangs ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert top is not None and side is not None and back is not None
    ctx.check("top_overhangs_sides",
              top[1][1] > side[1][1] + 0.015
              and top[0][1] < side[0][1] - 0.015,
              details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f})")
    ctx.check("top_overhangs_front_back",
              top[0][0] < back[0][0] - 0.015
              and top[1][0] > FRONT_X + 0.015,
              details=f"top x=({top[0][0]:.3f},{top[1][0]:.3f})")
    ctx.check("top_is_thin_slab",
              abs((top[1][2] - top[0][2]) - TOP_THK) < 0.002)

    # --- Open side cubbies: shelf boards visible through front opening ---
    for tag in ("0", "1"):
        for i in range(N_SHELVES):
            shelf = ctx.part_element_world_aabb(
                carcass, elem=f"cubby_{tag}_shelf_{i}")
            assert shelf is not None
            # Shelf front edge near the carcass front (visible from outside)
            ctx.check(f"cubby_{tag}_shelf_{i}_near_front",
                      shelf[1][0] > FRONT_X - 0.06,
                      details=f"shelf max x={shelf[1][0]:.3f}")
            # Shelf within the opening height
            ctx.check(f"cubby_{tag}_shelf_{i}_in_opening",
                      shelf[0][2] > OPEN_BOT and shelf[1][2] < OPEN_TOP,
                      details=f"shelf z=({shelf[0][2]:.3f},{shelf[1][2]:.3f})")

    # Cubby shelves are in the correct side zones (not center)
    cs0 = ctx.part_element_world_aabb(carcass, elem="cubby_0_shelf_0")
    cs1 = ctx.part_element_world_aabb(carcass, elem="cubby_1_shelf_0")
    assert cs0 is not None and cs1 is not None
    ctx.check("cubby_0_on_positive_y_side",
              cs0[0][1] > DIV_Y,
              details=f"cubby_0 min y={cs0[0][1]:.3f}")
    ctx.check("cubby_1_on_negative_y_side",
              cs1[1][1] < -DIV_Y,
              details=f"cubby_1 max y={cs1[1][1]:.3f}")

    # --- Center cabinet shelves exist ---
    for i in range(N_SHELVES):
        shelf = ctx.part_element_world_aabb(carcass, elem=f"center_shelf_{i}")
        assert shelf is not None
        ctx.check(f"center_shelf_{i}_in_center_zone",
                  abs(shelf[0][1] + shelf[1][1]) < 0.02,
                  details=f"center y={((shelf[0][1]+shelf[1][1])/2):.4f}")

    # --- Tambour: prismatic along +Y (sideways) ---
    ctx.check("tambour_joint_exists", tambour_joint is not None)
    ctx.check("tambour_prismatic",
              tambour_joint.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("tambour_axis_along_y",
              abs(tambour_joint.axis[1] - 1.0) < 0.01
              and abs(tambour_joint.axis[0]) < 0.01
              and abs(tambour_joint.axis[2]) < 0.01,
              details=f"axis={tambour_joint.axis}")
    ctx.check("tambour_travel_sufficient",
              tambour_joint.motion_limits.upper >= 0.48,
              details=f"upper={tambour_joint.motion_limits.upper:.3f}")
    ctx.check("tambour_lower_at_zero",
              abs(tambour_joint.motion_limits.lower) < 1e-6)

    # --- Closed pose: tambour covers center opening ---
    panel_closed = ctx.part_element_world_aabb(tambour, elem="tambour_panel")
    assert panel_closed is not None

    # Tambour panel Y extent should cover the center opening
    center_opening_half = CENTER_W / 2.0
    ctx.check("tambour_covers_center_y",
              panel_closed[0][1] < -center_opening_half + TAMBOUR_GAP + 0.005
              and panel_closed[1][1] > center_opening_half - TAMBOUR_GAP - 0.005,
              details=f"panel y=({panel_closed[0][1]:.3f},{panel_closed[1][1]:.3f})")

    # Tambour panel Z extent should cover the opening height
    ctx.check("tambour_covers_opening_z",
              panel_closed[0][2] < OPEN_BOT + TAMBOUR_GAP + 0.005
              and panel_closed[1][2] > OPEN_TOP - TAMBOUR_GAP - 0.005,
              details=f"panel z=({panel_closed[0][2]:.3f},{panel_closed[1][2]:.3f})")

    # Center shelves behind the tambour when closed
    cs = ctx.part_element_world_aabb(carcass, elem="center_shelf_0")
    assert cs is not None
    ctx.check("tambour_in_front_of_center_shelves",
              panel_closed[0][0] > cs[1][0],
              details=f"tambour back x={panel_closed[0][0]:.3f}, "
                      f"shelf front x={cs[1][0]:.3f}")

    # --- Gap seams around tambour when closed ---
    # Top gap: between tambour panel top edge and front_top_rail bottom edge
    rail_top = ctx.part_element_world_aabb(carcass, elem="front_top_rail")
    rail_bot = ctx.part_element_world_aabb(carcass, elem="front_bottom_rail")
    assert rail_top is not None and rail_bot is not None
    gap_top = rail_top[0][2] - panel_closed[1][2]
    gap_bot = panel_closed[0][2] - rail_bot[1][2]
    ctx.check("tambour_gap_seam_top",
              0.0 <= gap_top < 0.012,
              details=f"top gap={gap_top:.4f}")
    ctx.check("tambour_gap_seam_bottom",
              0.0 <= gap_bot < 0.012,
              details=f"bottom gap={gap_bot:.4f}")

    # Side gap seams: between tambour panel edges and divider inner faces
    div0 = ctx.part_element_world_aabb(carcass, elem="divider_0")
    div1 = ctx.part_element_world_aabb(carcass, elem="divider_1")
    assert div0 is not None and div1 is not None
    gap_right = div0[0][1] - panel_closed[1][1]  # div0 inner face - panel +Y edge
    gap_left = panel_closed[0][1] - div1[1][1]   # panel -Y edge - div1 inner face
    ctx.check("tambour_gap_seam_right",
              -0.002 <= gap_right < 0.012,
              details=f"right gap={gap_right:.4f}")
    ctx.check("tambour_gap_seam_left",
              -0.002 <= gap_left < 0.012,
              details=f"left gap={gap_left:.4f}")

    # --- Open pose: tambour slides sideways, reveals center ---
    with ctx.pose({tambour_joint: tambour_joint.motion_limits.upper}):
        panel_open = ctx.part_element_world_aabb(tambour, elem="tambour_panel")
        assert panel_open is not None

        # Panel moved substantially in +Y
        ctx.check("tambour_slides_sideways",
                  panel_open[0][1] > 0.20,
                  details=f"open panel min y={panel_open[0][1]:.3f}")

        # Panel no longer covers center opening (trailing edge past +Y opening edge)
        ctx.check("tambour_reveals_center",
                  panel_open[0][1] > center_opening_half - 0.02,
                  details=f"panel min y={panel_open[0][1]:.3f} "
                          f"vs opening edge {center_opening_half:.3f}")

        # Center shelves now exposed (no longer behind tambour in Y)
        # The center shelf at y=0 should not overlap with the open panel
        ctx.check("open_center_shelves_exposed",
                  panel_open[0][1] > 0.05,
                  details=f"panel min y={panel_open[0][1]:.3f}")

    # --- Carved posts and legs preserved ---
    seg0 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_0")
    top_seg = ctx.part_element_world_aabb(
        carcass, elem=f"carved_post_0_seg_{N_SEG - 1}")
    assert seg0 is not None and top_seg is not None
    ctx.check("posts_span_body_height",
              seg0[0][2] < BODY_BOT + 0.002
              and top_seg[1][2] > BODY_TOP - 0.004,
              details=f"post z=({seg0[0][2]:.3f},{top_seg[1][2]:.3f})")
    w0 = seg0[1][1] - seg0[0][1]
    ctx.check("post_segments_rotated", w0 > POST_SQ + 0.008,
              details=f"seg aabb width={w0:.4f}")

    leg = ctx.part_element_world_aabb(carcass, elem="leg_front_0")
    assert leg is not None
    ctx.check("front_leg_to_floor",
              abs(leg[0][2]) < 0.002 and leg[1][2] > LEG_H - 0.002)

    # --- At least one non-fixed joint ---
    all_joints = list(object_model.articulations)
    non_fixed = [j for j in all_joints
                 if j.articulation_type != ArticulationType.FIXED]
    ctx.check("has_nonfixed_joint", len(non_fixed) >= 1,
              details=f"non-fixed joints={len(non_fixed)}")

    return ctx.report()


object_model = build_object_model()
