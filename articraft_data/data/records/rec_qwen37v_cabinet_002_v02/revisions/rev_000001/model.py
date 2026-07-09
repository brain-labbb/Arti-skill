from __future__ import annotations

# Low wide black dresser cabinet variant (~1.70 m W x 0.85 m H x 0.50 m D).
#
# World layout: front faces +X (back at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four
# square legs ~0.15 m tall. Matte black wood carcass; smooth silver-gray
# top slab with overhang. Decorative carved corner posts with stacked
# spiral/zigzag pattern continue into the front legs.
#
# Front layout (3 columns along Y):
#   Right column: 3 drawers stacked vertically, each PRISMATIC along +X.
#   Center column: tambour door sliding sideways along +Y (PRISMATIC),
#                  revealing shelf boards inside when open.
#   Left column:   fixed door panel with recessed panel border detail.
#
# All drawers are hollow open-top trays with proud front panels and
# polished silver ball knobs on short stems.

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
BODY_BOT = LEG_H
BODY_TOP = H_TOTAL - TOP_THK     # 0.828
BH = BODY_TOP - BODY_BOT         # body height

WALL = 0.018                     # carcass panel thickness
INNER_W = BW - 2 * WALL

# ---------- Column layout along Y ----------
# Right column (drawers): y from DIVIDER_R to +BW/2
# Center column (tambour + shelves): y from DIVIDER_L to DIVIDER_R
# Left column (fixed door): y from -BW/2 to DIVIDER_L
COL_W_DRAWER = 0.50             # right column width
COL_W_TAMBOUR = 0.54            # center column width
DIVIDER_R = BW / 2.0 - COL_W_DRAWER   # ~0.33
DIVIDER_L = DIVIDER_R - COL_W_TAMBOUR  # ~-0.21

# Drawer zone (right column)
FRONT_FRAME_BOT = BODY_BOT + 0.025
FRONT_FRAME_TOP = BODY_TOP - 0.018
REVEAL = 0.007

DRAWER_ZONE_H = FRONT_FRAME_TOP - FRONT_FRAME_BOT
DRAWER_FH = (DRAWER_ZONE_H - 2 * REVEAL) / 3.0   # each drawer front height

DRAWER_CY = [
    FRONT_FRAME_BOT + DRAWER_FH / 2.0,                                         # bottom
    FRONT_FRAME_BOT + DRAWER_FH + REVEAL + DRAWER_FH / 2.0,                    # middle
    FRONT_FRAME_BOT + 2 * (DRAWER_FH + REVEAL) - REVEAL + DRAWER_FH / 2.0,    # top
]
# Fix top center calculation
DRAWER_CY[2] = FRONT_FRAME_TOP - DRAWER_FH / 2.0

FACE_THK = 0.018
FACE_PROUD = 0.0005
FRONT_X = BD
SLAB_BACK_X = FRONT_X + FACE_PROUD
JOINT_X = SLAB_BACK_X + FACE_THK

TRAVEL = 0.350
TRAY_D = 0.380
TRAY_T = 0.012

# Tambour zone (center column)
TAMBOUR_ZONE_BOT = BODY_BOT + 0.025
TAMBOUR_ZONE_TOP = BODY_TOP - 0.018
TAMBOUR_H = TAMBOUR_ZONE_TOP - TAMBOUR_ZONE_BOT
TAMBOUR_THK = 0.015
TAMBOUR_TRAVEL = 0.50           # slides sideways to reveal shelves
TAMBOUR_CY = (DIVIDER_L + DIVIDER_R) / 2.0  # center Y of tambour column
TAMBOUR_CZ = (TAMBOUR_ZONE_BOT + TAMBOUR_ZONE_TOP) / 2.0

# Shelf boards inside center column
SHELF_THK = 0.014
N_SHELVES = 3

# Left column door
DOOR_THK = 0.020
DOOR_RECESS_DEPTH = 0.008       # recessed border depth
DOOR_BORDER = 0.040             # border width around the recessed panel

# Carved corner posts
POST_SQ = 0.050
POST_CX = BD - 0.002
POST_CY = BW / 2.0 - 0.025
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG

LEG_SQ = 0.050

# Knobs
KNOB_R = 0.0125
STEM_R = 0.0055
STEM_L = 0.014


def _build_drawer(model, name, front_w, front_h, tray_w, tray_h,
                  knob_ys, black, tray_mat, silver):
    """Drawer: front panel outer surface at local x=0, tray extends toward -X."""
    drawer = model.part(name)

    drawer.visual(
        Box((FACE_THK, front_w, front_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, 0.0)),
        material=black,
        name="front_panel",
    )

    # Hollow open-top tray
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

    # Silver ball knobs on short stems
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
        Box((TRAY_D, tray_w, tray_h)), mass=3.5)
    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dresser_cabinet_tambour_variant")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    tray_mat = model.material("tray_black", rgba=(0.13, 0.13, 0.14, 1.0))
    shelf_mat = model.material("shelf_wood", rgba=(0.10, 0.10, 0.11, 1.0))
    tambour_mat = model.material("tambour_black", rgba=(0.09, 0.09, 0.095, 1.0))
    door_mat = model.material("door_black", rgba=(0.065, 0.065, 0.07, 1.0))
    recess_mat = model.material("recess_dark", rgba=(0.04, 0.04, 0.045, 1.0))

    # ===================================================================
    # ROOT: carcass
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

    # Top stretcher
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black,
        name="top_stretcher",
    )

    # Front bottom rail (full width)
    carcass.visual(
        Box((WALL, INNER_W, FRONT_FRAME_BOT - BODY_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (BODY_BOT + FRONT_FRAME_BOT) / 2.0)),
        material=black,
        name="front_bottom_rail",
    )

    # Front top rail (full width)
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - FRONT_FRAME_TOP)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (FRONT_FRAME_TOP + BODY_TOP) / 2.0)),
        material=black,
        name="front_top_rail",
    )

    # Vertical dividers between columns
    divider_h = FRONT_FRAME_TOP - FRONT_FRAME_BOT
    for tag, dy in (("left", DIVIDER_L), ("right", DIVIDER_R)):
        carcass.visual(
            Box((BD - 0.020, WALL, divider_h)),
            origin=Origin(xyz=(WALL + (BD - 0.020) / 2.0, dy,
                               (FRONT_FRAME_BOT + FRONT_FRAME_TOP) / 2.0)),
            material=black,
            name=f"divider_{tag}",
        )

    # Tambour guide rails: thin strips at top and bottom of center column
    # opening, front face at x=BD to contact the tambour panel back.
    rail_thk = 0.010
    rail_depth = WALL
    rail_w = abs(DIVIDER_R - DIVIDER_L) - 0.004
    for tag, z_edge in (("bottom", TAMBOUR_ZONE_BOT), ("top", TAMBOUR_ZONE_TOP)):
        zc = z_edge + rail_thk / 2.0 if tag == "bottom" else z_edge - rail_thk / 2.0
        carcass.visual(
            Box((rail_depth, rail_w, rail_thk)),
            origin=Origin(xyz=(BD - rail_depth / 2.0, TAMBOUR_CY, zc)),
            material=black_deep,
            name=f"tambour_rail_{tag}",
        )

    # Front stiles between dividers and side panels (fill front frame around openings)
    # Right column: stiles above and between drawers
    right_col_center_y = (DIVIDER_R + BW / 2.0 - WALL) / 2.0

    # Drawer dust panels (runners under each drawer, top surface matches tray bottom)
    for i in range(3):
        tray_bot_z = DRAWER_CY[i] + (-DRAWER_FH / 2.0 + 0.012)
        carcass.visual(
            Box((BD - 0.030, COL_W_DRAWER - 0.010, 0.012)),
            origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, right_col_center_y,
                               tray_bot_z - 0.006)),
            material=black_deep,
            name=f"drawer_dust_{i}",
        )

    # Shelf boards in center column
    shelf_span = BD - 0.040
    shelf_w = abs(DIVIDER_R - DIVIDER_L) - 0.010
    for i in range(N_SHELVES):
        frac = (i + 1) / (N_SHELVES + 1)
        sz = TAMBOUR_ZONE_BOT + frac * (TAMBOUR_ZONE_TOP - TAMBOUR_ZONE_BOT)
        carcass.visual(
            Box((shelf_span, shelf_w, SHELF_THK)),
            origin=Origin(xyz=(0.020 + shelf_span / 2.0,
                               TAMBOUR_CY, sz)),
            material=shelf_mat,
            name=f"shelf_{i}",
        )

    # Left column: fixed door panel with recessed borders
    left_col_center_y = (-BW / 2.0 + WALL + DIVIDER_L) / 2.0
    left_col_w = abs(DIVIDER_L - (-BW / 2.0 + WALL))
    door_h = FRONT_FRAME_TOP - FRONT_FRAME_BOT
    door_cz = (FRONT_FRAME_BOT + FRONT_FRAME_TOP) / 2.0

    # Outer door panel
    carcass.visual(
        Box((DOOR_THK, left_col_w - 0.006, door_h - 0.006)),
        origin=Origin(xyz=(BD - DOOR_THK / 2.0 + FACE_PROUD,
                           left_col_center_y, door_cz)),
        material=door_mat,
        name="door_panel",
    )

    # Recessed inner panel (smaller, set back to create border effect)
    recess_w = left_col_w - 2 * DOOR_BORDER - 0.006
    recess_h = door_h - 2 * DOOR_BORDER - 0.006
    carcass.visual(
        Box((DOOR_THK - DOOR_RECESS_DEPTH, recess_w, recess_h)),
        origin=Origin(xyz=(BD - DOOR_THK / 2.0 + FACE_PROUD + DOOR_RECESS_DEPTH / 2.0,
                           left_col_center_y, door_cz)),
        material=recess_mat,
        name="door_recess",
    )

    # Recessed border strips (4 strips forming the border frame)
    # Top border strip
    carcass.visual(
        Box((DOOR_RECESS_DEPTH, recess_w + 0.004, DOOR_BORDER)),
        origin=Origin(xyz=(BD - DOOR_RECESS_DEPTH / 2.0 + FACE_PROUD,
                           left_col_center_y,
                           door_cz + recess_h / 2.0 + DOOR_BORDER / 2.0)),
        material=door_mat,
        name="door_border_top",
    )
    # Bottom border strip
    carcass.visual(
        Box((DOOR_RECESS_DEPTH, recess_w + 0.004, DOOR_BORDER)),
        origin=Origin(xyz=(BD - DOOR_RECESS_DEPTH / 2.0 + FACE_PROUD,
                           left_col_center_y,
                           door_cz - recess_h / 2.0 - DOOR_BORDER / 2.0)),
        material=door_mat,
        name="door_border_bottom",
    )
    # Left border strip
    carcass.visual(
        Box((DOOR_RECESS_DEPTH, DOOR_BORDER, recess_h)),
        origin=Origin(xyz=(BD - DOOR_RECESS_DEPTH / 2.0 + FACE_PROUD,
                           left_col_center_y - recess_w / 2.0 - DOOR_BORDER / 2.0,
                           door_cz)),
        material=door_mat,
        name="door_border_left",
    )
    # Right border strip
    carcass.visual(
        Box((DOOR_RECESS_DEPTH, DOOR_BORDER, recess_h)),
        origin=Origin(xyz=(BD - DOOR_RECESS_DEPTH / 2.0 + FACE_PROUD,
                           left_col_center_y + recess_w / 2.0 + DOOR_BORDER / 2.0,
                           door_cz)),
        material=door_mat,
        name="door_border_right",
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
    # DRAWERS: 3 independent PRISMATIC slides along +X
    # ===================================================================
    drawer_w = min(COL_W_DRAWER - 0.016, 0.400)  # clear the carved posts
    drawer_tray_w = drawer_w - 0.020
    drawers = []
    for i, cy in enumerate(DRAWER_CY):
        d = _build_drawer(model, f"drawer_{i}", drawer_w, DRAWER_FH,
                          drawer_tray_w, DRAWER_FH - 0.020,
                          [0.0], black, tray_mat, silver)
        drawers.append((f"drawer_{i}", d, right_col_center_y, cy))

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

    # ===================================================================
    # TAMBOUR: slides sideways along +Y
    # ===================================================================
    tambour = model.part("tambour")

    # Tambour panel: a flat board that covers the center column opening.
    # Local frame: panel face in local YZ plane, thickness along X.
    # The panel extends along Y (width = COL_W_TAMBOUR) and Z (height = TAMBOUR_H).
    tambour.visual(
        Box((TAMBOUR_THK, COL_W_TAMBOUR - 0.008, TAMBOUR_H - 0.006)),
        origin=Origin(xyz=(-TAMBOUR_THK / 2.0, 0.0, 0.0)),
        material=tambour_mat,
        name="tambour_panel",
    )

    # Tambour slat lines (decorative horizontal grooves to suggest tambour construction)
    n_slats = 8
    slat_spacing = (TAMBOUR_H - 0.010) / n_slats
    for i in range(n_slats):
        sz = -(TAMBOUR_H - 0.010) / 2.0 + (i + 0.5) * slat_spacing
        tambour.visual(
            Box((0.003, COL_W_TAMBOUR - 0.012, 0.003)),
            origin=Origin(xyz=(0.001, 0.0, sz)),
            material=black_deep,
            name=f"tambour_slat_{i}",
        )

    # Small pull handle on tambour (embedded into the panel face for connectivity)
    tambour.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, COL_W_TAMBOUR / 2.0 - 0.050, 0.0),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="tambour_handle",
    )

    tambour.inertial = Inertial.from_geometry(
        Box((TAMBOUR_THK, COL_W_TAMBOUR, TAMBOUR_H)), mass=2.5)

    # Tambour articulation: PRISMATIC along +Y (slides sideways to the right)
    # At q=0 the tambour covers the center column.
    # At q=TAMBOUR_TRAVEL it slides to the right, revealing shelves.
    # Panel back face sits at x=BD (carcass front plane) for contact.
    model.articulation(
        "carcass_to_tambour",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=tambour,
        origin=Origin(xyz=(BD + TAMBOUR_THK,
                           TAMBOUR_CY, TAMBOUR_CZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.4,
                                   lower=0.0, upper=TAMBOUR_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    drawer_names = [f"drawer_{i}" for i in range(3)]
    drawer_parts = {n: object_model.get_part(n) for n in drawer_names}
    drawer_joints = {n: object_model.get_articulation(f"carcass_to_{n}")
                     for n in drawer_names}
    tambour_part = object_model.get_part("tambour")
    tambour_joint = object_model.get_articulation("carcass_to_tambour")

    # --- Overall dimensions ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("legs_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_170", abs(width_y - 1.70) < 0.02, details=f"w={width_y:.3f}")
    ctx.check("depth_050", abs(depth_x - 0.50) < 0.04, details=f"d={depth_x:.3f}")
    ctx.check("height_085", abs(height_z - 0.85) < 0.01,
              details=f"h={height_z:.3f}")

    # --- Silver top slab overhangs ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert top is not None and side is not None and back is not None
    ctx.check("top_overhang_all_sides",
              top[1][1] > side[1][1] + 0.015 and top[0][1] < -side[1][1] - 0.015
              and top[0][0] < back[0][0] - 0.015 and top[1][0] > 0.46 + 0.015,
              details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f})")
    ctx.check("top_is_thin_slab", abs((top[1][2] - top[0][2]) - TOP_THK) < 0.002)

    # --- Three drawers exist, all PRISMATIC along +X ---
    ctx.check("three_drawers", len(drawer_joints) == 3)
    for n, j in drawer_joints.items():
        ctx.check(f"{n}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{n}_axis_forward",
                  j.axis[0] > 0.99 and abs(j.axis[1]) < 0.01
                  and abs(j.axis[2]) < 0.01)
        ctx.check(f"{n}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - TRAVEL) < 1e-6,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Drawers stacked vertically ---
    f_fronts = [ctx.part_element_world_aabb(drawer_parts[n], elem="front_panel")
                for n in drawer_names]
    assert all(f is not None for f in f_fronts)
    ctx.check("drawers_stacked_vertically",
              f_fronts[2][0][2] > f_fronts[1][1][2] > f_fronts[0][1][2],
              details=f"z ranges: bot=({f_fronts[0][0][2]:.3f},{f_fronts[0][1][2]:.3f}), "
                      f"mid=({f_fronts[1][0][2]:.3f},{f_fronts[1][1][2]:.3f}), "
                      f"top=({f_fronts[2][0][2]:.3f},{f_fronts[2][1][2]:.3f})")

    # --- Drawer fronts proud of carcass ---
    carcass_front = BD
    for n in drawer_names:
        face = ctx.part_element_world_aabb(drawer_parts[n], elem="front_panel")
        assert face is not None
        ctx.check(f"{n}_front_proud",
                  0.0 < face[1][0] - carcass_front < 0.03,
                  details=f"face front x={face[1][0]:.4f}")

    # --- Drawer open pose: slides forward, retains insertion ---
    for n in ("drawer_0", "drawer_2"):
        d, j = drawer_parts[n], drawer_joints[n]
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
            rear = ctx.part_element_world_aabb(d, elem="tray_back_wall")
            assert rear is not None
            rear_x = rear[0][0]
        assert rest is not None and out is not None
        ctx.check(f"{n}_slides_forward",
                  abs((out[0] - rest[0]) - TRAVEL) < 1e-5,
                  details=f"dx={out[0] - rest[0]:.4f}")
        ctx.check(f"{n}_retains_insertion", rear_x < carcass_front - 0.005,
                  details=f"open rear x={rear_x:.4f}")

    # --- Tambour exists and is PRISMATIC along Y (sideways) ---
    ctx.check("tambour_prismatic",
              tambour_joint.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("tambour_axis_sideways",
              abs(tambour_joint.axis[1]) > 0.99
              and abs(tambour_joint.axis[0]) < 0.01
              and abs(tambour_joint.axis[2]) < 0.01,
              details=f"axis={tambour_joint.axis}")
    ctx.check("tambour_range",
              abs(tambour_joint.motion_limits.lower) < 1e-9
              and abs(tambour_joint.motion_limits.upper - TAMBOUR_TRAVEL) < 1e-6,
              details=f"range=({tambour_joint.motion_limits.lower},"
                      f"{tambour_joint.motion_limits.upper})")

    # --- Tambour slides sideways when opened ---
    rest_pos = ctx.part_world_position(tambour_part)
    with ctx.pose({tambour_joint: tambour_joint.motion_limits.upper}):
        open_pos = ctx.part_world_position(tambour_part)
    assert rest_pos is not None and open_pos is not None
    ctx.check("tambour_slides_sideways",
              abs((open_pos[1] - rest_pos[1]) - TAMBOUR_TRAVEL) < 1e-5,
              details=f"dy={open_pos[1] - rest_pos[1]:.4f}")
    # Tambour should not move much in X or Z
    ctx.check("tambour_no_forward_motion",
              abs(open_pos[0] - rest_pos[0]) < 0.005
              and abs(open_pos[2] - rest_pos[2]) < 0.005,
              details=f"dx={open_pos[0] - rest_pos[0]:.4f}, "
                      f"dz={open_pos[2] - rest_pos[2]:.4f}")

    # --- Shelf boards exist inside the center column ---
    for i in range(N_SHELVES):
        shelf = ctx.part_element_world_aabb(carcass, elem=f"shelf_{i}")
        assert shelf is not None
        ctx.check(f"shelf_{i}_exists",
                  shelf[1][2] - shelf[0][2] < 0.025,
                  details=f"thickness={shelf[1][2] - shelf[0][2]:.4f}")
        # Shelves should be between the dividers in Y
        ctx.check(f"shelf_{i}_in_center_column",
                  shelf[0][1] > DIVIDER_L - 0.01
                  and shelf[1][1] < DIVIDER_R + 0.01,
                  details=f"y=({shelf[0][1]:.3f},{shelf[1][1]:.3f})")

    # --- Shelves visible when tambour is open ---
    with ctx.pose({tambour_joint: TAMBOUR_TRAVEL}):
        tambour_aabb = ctx.part_world_aabb(tambour_part)
        shelf_1 = ctx.part_element_world_aabb(carcass, elem="shelf_1")
        assert tambour_aabb is not None and shelf_1 is not None
        # When open, the shelf Y range should not be fully covered by tambour
        ctx.check("shelf_exposed_when_tambour_open",
                  shelf_1[0][1] < tambour_aabb[0][1] or shelf_1[1][1] > tambour_aabb[1][1],
                  details=f"shelf y=({shelf_1[0][1]:.3f},{shelf_1[1][1]:.3f}), "
                          f"tambour y=({tambour_aabb[0][1]:.3f},{tambour_aabb[1][1]:.3f})")

    # --- Recessed panel borders on left door ---
    door_panel = ctx.part_element_world_aabb(carcass, elem="door_panel")
    door_recess = ctx.part_element_world_aabb(carcass, elem="door_recess")
    border_top = ctx.part_element_world_aabb(carcass, elem="door_border_top")
    assert door_panel is not None and door_recess is not None and border_top is not None
    # The recessed panel should be thinner (in X) than the outer door panel
    door_x_thk = door_panel[1][0] - door_panel[0][0]
    recess_x_thk = door_recess[1][0] - door_recess[0][0]
    ctx.check("door_has_recessed_panel",
              recess_x_thk < door_x_thk - 0.002,
              details=f"door thk={door_x_thk:.4f}, recess thk={recess_x_thk:.4f}")
    # Border strips sit proud of the recess
    ctx.check("door_border_top_exists",
              border_top[1][2] - border_top[0][2] > 0.02,
              details=f"border top height={border_top[1][2] - border_top[0][2]:.4f}")

    # --- At least one non-fixed joint ---
    all_joints = [object_model.get_articulation(n)
                  for n in [f"carcass_to_{dn}" for dn in drawer_names]
                  + ["carcass_to_tambour"]]
    non_fixed = [j for j in all_joints
                 if j.articulation_type != ArticulationType.FIXED]
    ctx.check("has_non_fixed_joints", len(non_fixed) >= 1,
              details=f"non_fixed_count={len(non_fixed)}")

    # --- Carved posts exist ---
    seg0 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_0")
    seg1 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_1")
    assert seg0 is not None and seg1 is not None
    w0 = seg0[1][1] - seg0[0][1]
    ctx.check("post_segments_rotated", w0 > POST_SQ + 0.008,
              details=f"seg aabb width={w0:.4f} vs stock {POST_SQ}")

    return ctx.report()


object_model = build_object_model()
