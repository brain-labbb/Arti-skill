from __future__ import annotations

# Variant 24: Wide black wooden cabinet (~1.70 m W x 0.85 m H x 0.50 m D)
# with a sliding tambour-style front panel over an upper open compartment
# containing shelf boards, plus four wide drawers in the lower zone.
#
# World layout: front faces +X (back at x=0, front at x=BD), width along Y
# (centered), height along +Z, grounded at z=0 on four square legs ~0.15 m.
# Carcass and drawer fronts are matte black wood; the silver-gray top slab
# overhangs on all sides. Decorative carved posts at the front corners
# continue into the legs.
#
# Upper zone: open front compartment with two shelf boards, covered by a
# tambour panel (horizontal slats) that slides vertically (+Z) to reveal
# the shelves. Lower zone: 2 rows x 2 wide drawers, each on a prismatic
# joint sliding outward along +X, range 0 to 0.40 m.

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

BW = W_TOTAL - 2 * OVERHANG       # 1.66
BD = D_TOTAL - 2 * OVERHANG       # 0.46
LEG_H = 0.150
BODY_BOT = LEG_H                   # 0.150
BODY_TOP = H_TOTAL - TOP_THK      # 0.828
BH = BODY_TOP - BODY_BOT          # 0.678

WALL = 0.018
INNER_W = BW - 2 * WALL           # 1.624

# ---- Zone split: lower drawers, upper shelves + tambour ----
DIVIDER_THK = 0.018
DIVIDER_Z_CENTER = 0.470
DIVIDER_TOP = DIVIDER_Z_CENTER + DIVIDER_THK / 2.0   # 0.479
DIVIDER_BOT = DIVIDER_Z_CENTER - DIVIDER_THK / 2.0   # 0.461

# Lower drawer zone
LOWER_RAIL_H = 0.030
LOWER_ZONE_BOT = BODY_BOT + LOWER_RAIL_H              # 0.180
LOWER_REVEAL = 0.008
LOWER_ZONE_TOP = DIVIDER_BOT - LOWER_REVEAL           # 0.453
LOWER_FH = (LOWER_ZONE_TOP - LOWER_ZONE_BOT - LOWER_REVEAL) / 2.0  # ~0.133

LOWER_CZ_BOT = LOWER_ZONE_BOT + LOWER_FH / 2.0
LOWER_CZ_TOP = LOWER_ZONE_BOT + LOWER_FH + LOWER_REVEAL + LOWER_FH / 2.0

# Upper shelf/tambour zone
UPPER_ZONE_BOT = DIVIDER_TOP + 0.002                  # 0.481
UPPER_ZONE_TOP = BODY_TOP - 0.025                     # 0.803
UPPER_H = UPPER_ZONE_TOP - UPPER_ZONE_BOT             # 0.322

# Tambour panel
TAMBOUR_THK = 0.012
TAMBOUR_SLAT_H = 0.022
TAMBOUR_SLAT_GAP = 0.003
TAMBOUR_PANEL_H = UPPER_H - 0.010                    # 0.312 (slight clearance)
TAMBOUR_PANEL_W = 1.540                               # fits between posts
TAMBOUR_TRAVEL = 0.220                                # slides up 0.22 m

# Drawer layout (lower zone, 2 rows of 2 wide drawers)
OPEN_HW = 0.762
FW_WIDE = (2 * OPEN_HW - LOWER_REVEAL) / 2.0         # ~0.760
WIDE_CY = [-(FW_WIDE / 2.0 + LOWER_REVEAL / 2.0),
            (FW_WIDE / 2.0 + LOWER_REVEAL / 2.0)]

FACE_THK = 0.018
FACE_PROUD = 0.0005
FRONT_X = BD                                          # 0.46
SLAB_BACK_X = FRONT_X + FACE_PROUD
JOINT_X = SLAB_BACK_X + FACE_THK

TRAVEL = 0.400
TRAY_D = 0.420
TRAY_T = 0.012

# Carved corner posts
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025                           # 0.805
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG

LEG_SQ = 0.050

# Knobs
KNOB_R = 0.0125
STEM_R = 0.0055
STEM_L = 0.014

# Shelves
SHELF_THK = 0.016
N_SHELVES = 2
SHELF_Z = [UPPER_ZONE_BOT + 0.080, UPPER_ZONE_BOT + 0.170]  # 0.561, 0.651


def _build_drawer(model: ArticulatedObject, name: str, front_w: float,
                  front_h: float, tray_w: float, tray_h: float,
                  knob_ys: list[float], black, tray_mat, silver):
    """Drawer: front panel outer surface at local x=0, tray extends -X."""
    drawer = model.part(name)

    drawer.visual(
        Box((FACE_THK, front_w, front_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, 0.0)),
        material=black,
        name="front_panel",
    )

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
        Box((TRAY_D, tray_w, tray_h)), mass=4.0)
    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_cabinet_tambour_shelves")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    tray_mat = model.material("tray_black", rgba=(0.13, 0.13, 0.14, 1.0))
    shelf_mat = model.material("shelf_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tambour_mat = model.material("tambour_black", rgba=(0.09, 0.09, 0.095, 1.0))
    track_mat = model.material("track_dark", rgba=(0.04, 0.04, 0.045, 1.0))

    # ===================================================================
    # ROOT: carcass (shell + legs + posts + top + divider + shelves + tracks)
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

    # Top stretcher (set back from front to clear the tambour path)
    carcass.visual(
        Box((BD - 0.060, INNER_W, 0.018)),
        origin=Origin(xyz=((BD - 0.060) / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black,
        name="top_stretcher",
    )

    # Horizontal divider between lower drawers and upper shelves
    carcass.visual(
        Box((BD - WALL, INNER_W, DIVIDER_THK)),
        origin=Origin(xyz=(WALL + (BD - WALL) / 2.0, 0.0, DIVIDER_Z_CENTER)),
        material=black,
        name="divider_board",
    )

    # ---- Lower zone front frame ----
    carcass.visual(
        Box((WALL, INNER_W, LOWER_RAIL_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           BODY_BOT + LOWER_RAIL_H / 2.0)),
        material=black,
        name="lower_bottom_rail",
    )

    stile_w = BW / 2.0 - OPEN_HW + 0.004
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, stile_w, LOWER_ZONE_TOP - LOWER_ZONE_BOT)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (BW / 2.0 - stile_w / 2.0),
                               (LOWER_ZONE_BOT + LOWER_ZONE_TOP) / 2.0)),
            material=black,
            name=f"lower_front_stile_{tag}",
        )

    carcass.visual(
        Box((0.016, 0.024, LOWER_ZONE_TOP - LOWER_ZONE_BOT)),
        origin=Origin(xyz=(BD - 0.008, 0.0,
                           (LOWER_ZONE_BOT + LOWER_ZONE_TOP) / 2.0)),
        material=black,
        name="lower_center_stile",
    )

    # Dust panels (drawer runners)
    for row_tag, cz in (("bottom", LOWER_CZ_BOT), ("top", LOWER_CZ_TOP)):
        tray_bot_z = cz + (-LOWER_FH / 2.0 + 0.012)
        carcass.visual(
            Box((BD - 0.030, INNER_W, 0.014)),
            origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, 0.0,
                               tray_bot_z - 0.007)),
            material=black_deep,
            name=f"{row_tag}_dust_panel",
        )

    # ---- Upper zone: front frame rails (top and bottom of opening) ----
    carcass.visual(
        Box((WALL, INNER_W, UPPER_ZONE_BOT - DIVIDER_TOP)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (DIVIDER_TOP + UPPER_ZONE_BOT) / 2.0)),
        material=black,
        name="upper_bottom_rail",
    )
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - UPPER_ZONE_TOP)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (UPPER_ZONE_TOP + BODY_TOP) / 2.0)),
        material=black,
        name="upper_top_rail",
    )

    # Upper zone front side stiles (between opening edges and side panels)
    upper_stile_w = BW / 2.0 - TAMBOUR_PANEL_W / 2.0 - WALL
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, upper_stile_w, UPPER_H)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (BW / 2.0 - WALL - upper_stile_w / 2.0),
                               (UPPER_ZONE_BOT + UPPER_ZONE_TOP) / 2.0)),
            material=black,
            name=f"upper_side_stile_{tag}",
        )

    # ---- Tambour tracks (vertical guide strips on inner sides) ----
    track_w = 0.015
    track_d = 0.025
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((track_d, track_w, UPPER_H + 0.020)),
            origin=Origin(xyz=(BD - track_d / 2.0,
                               s * (BW / 2.0 - WALL - track_w / 2.0),
                               (UPPER_ZONE_BOT + UPPER_ZONE_TOP) / 2.0)),
            material=track_mat,
            name=f"tambour_track_{tag}",
        )

    # ---- Shelf boards in upper zone ----
    inner_d = BD - WALL - 0.010
    for i in range(N_SHELVES):
        carcass.visual(
            Box((inner_d, INNER_W - 0.010, SHELF_THK)),
            origin=Origin(xyz=(WALL + inner_d / 2.0, 0.0, SHELF_Z[i])),
            material=shelf_mat,
            name=f"shelf_board_{i}",
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
    # DRAWERS: 4 independent PRISMATIC slides along +X (lower zone)
    # ===================================================================
    drawer_specs = [
        ("lower_bottom_drawer_0", WIDE_CY[0], LOWER_CZ_BOT),
        ("lower_bottom_drawer_1", WIDE_CY[1], LOWER_CZ_BOT),
        ("lower_top_drawer_0", WIDE_CY[0], LOWER_CZ_TOP),
        ("lower_top_drawer_1", WIDE_CY[1], LOWER_CZ_TOP),
    ]
    drawer_parts = {}
    for name, cy, cz in drawer_specs:
        d = _build_drawer(model, name, FW_WIDE, LOWER_FH,
                          0.700, 0.100, [-0.19, 0.19],
                          black, tray_mat, silver)
        drawer_parts[name] = (d, cy, cz)

    for name, (d, cy, cz) in drawer_parts.items():
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
    # TAMBOUR: sliding front panel, PRISMATIC along +Z
    # ===================================================================
    tambour = model.part("tambour")

    # Build horizontal slats filling the panel height
    n_slats = int(TAMBOUR_PANEL_H / (TAMBOUR_SLAT_H + TAMBOUR_SLAT_GAP))
    slat_pitch = TAMBOUR_SLAT_H + TAMBOUR_SLAT_GAP
    slats_total = n_slats * TAMBOUR_SLAT_H + (n_slats - 1) * TAMBOUR_SLAT_GAP
    slat_z0 = -slats_total / 2.0 + TAMBOUR_SLAT_H / 2.0

    for i in range(n_slats):
        slat_z = slat_z0 + i * slat_pitch
        tambour.visual(
            Box((TAMBOUR_THK, TAMBOUR_PANEL_W, TAMBOUR_SLAT_H)),
            origin=Origin(xyz=(0.0, 0.0, slat_z)),
            material=tambour_mat,
            name=f"tambour_slat_{i}",
        )

    # Backing strip connecting slats (thin vertical strip behind the slats)
    tambour.visual(
        Box((0.004, TAMBOUR_PANEL_W - 0.040, TAMBOUR_PANEL_H - 0.020)),
        origin=Origin(xyz=(-TAMBOUR_THK / 2.0 - 0.002, 0.0, 0.0)),
        material=tambour_mat,
        name="tambour_backing",
    )

    # Pull handle at the bottom center
    handle_w = 0.120
    handle_h = 0.015
    handle_d = 0.018
    tambour.visual(
        Box((handle_d, handle_w, handle_h)),
        origin=Origin(xyz=(TAMBOUR_THK / 2.0 + handle_d / 2.0, 0.0,
                           slat_z0 - TAMBOUR_SLAT_H / 2.0 + handle_h / 2.0 + 0.003)),
        material=silver,
        name="tambour_handle",
    )

    tambour.inertial = Inertial.from_geometry(
        Box((TAMBOUR_THK + 0.010, TAMBOUR_PANEL_W, TAMBOUR_PANEL_H)), mass=3.0)

    # Tambour joint: prismatic along +Z (slides up to reveal shelves)
    tambour_cx = BD - TAMBOUR_THK / 2.0
    tambour_cz = (UPPER_ZONE_BOT + UPPER_ZONE_TOP) / 2.0
    model.articulation(
        "carcass_to_tambour",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=tambour,
        origin=Origin(xyz=(tambour_cx, 0.0, tambour_cz)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.3,
                                   lower=0.0, upper=TAMBOUR_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    tambour = object_model.get_part("tambour")
    tambour_joint = object_model.get_articulation("carcass_to_tambour")

    drawer_names = [f"lower_bottom_drawer_{i}" for i in range(2)] + \
                   [f"lower_top_drawer_{i}" for i in range(2)]
    drawer_objs = {n: object_model.get_part(n) for n in drawer_names}
    drawer_joints = {n: object_model.get_articulation(f"carcass_to_{n}")
                     for n in drawer_names}

    # --- Overall dimensions (~1.70 x 0.50 x 0.85 m) ---
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

    # --- Silver top overhang ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert top is not None and side is not None and back is not None
    ctx.check("top_overhang",
              top[1][1] > side[1][1] + 0.015 and top[0][1] < -side[1][1] - 0.015
              and top[0][0] < back[0][0] - 0.015 and top[1][0] > 0.46 + 0.015)

    # --- Divider board separates zones ---
    divider = ctx.part_element_world_aabb(carcass, elem="divider_board")
    assert divider is not None
    ctx.check("divider_near_mid_height",
              abs(divider[0][2] + (divider[1][2] - divider[0][2]) / 2.0 - DIVIDER_Z_CENTER) < 0.005)

    # === TAMBOUR CHECKS ===

    # Tambour is prismatic along vertical (+Z)
    ctx.check("tambour_is_prismatic",
              tambour_joint.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("tambour_axis_vertical",
              abs(tambour_joint.axis[2] - 1.0) < 0.01
              and abs(tambour_joint.axis[0]) < 0.01
              and abs(tambour_joint.axis[1]) < 0.01,
              details=f"axis={tambour_joint.axis}")
    ctx.check("tambour_travel_range",
              abs(tambour_joint.motion_limits.lower) < 1e-9
              and tambour_joint.motion_limits.upper > 0.18,
              details=f"range=({tambour_joint.motion_limits.lower:.3f},"
                      f"{tambour_joint.motion_limits.upper:.3f})")

    # Tambour at rest sits in the upper zone (above divider)
    tambour_rest = ctx.part_world_aabb(tambour)
    assert tambour_rest is not None
    ctx.check("tambour_above_divider",
              tambour_rest[0][2] > divider[1][2] - 0.01,
              details=f"tambour min z={tambour_rest[0][2]:.3f}, "
                      f"divider top={divider[1][2]:.3f}")

    # Tambour has multiple horizontal slats (tambour style)
    slat_count = sum(1 for v in tambour.visuals
                     if v.name.startswith("tambour_slat_"))
    ctx.check("tambour_has_slats", slat_count >= 8,
              details=f"slat_count={slat_count}")

    # Tambour slides up and reveals the shelf area
    shelf0 = ctx.part_element_world_aabb(carcass, elem="shelf_board_0")
    shelf1 = ctx.part_element_world_aabb(carcass, elem="shelf_board_1")
    assert shelf0 is not None and shelf1 is not None

    rest_bottom_z = tambour_rest[0][2]
    with ctx.pose({tambour_joint: tambour_joint.motion_limits.upper}):
        tambour_open = ctx.part_world_aabb(tambour)
        assert tambour_open is not None
        open_bottom_z = tambour_open[0][2]

    ctx.check("tambour_slides_upward",
              open_bottom_z > rest_bottom_z + 0.15,
              details=f"rest bottom={rest_bottom_z:.3f}, "
                      f"open bottom={open_bottom_z:.3f}")
    ctx.check("tambour_reveals_lower_shelf",
              open_bottom_z > shelf0[1][2] + 0.01,
              details=f"open bottom={open_bottom_z:.3f}, "
                      f"shelf0 top={shelf0[1][2]:.3f}")
    ctx.check("tambour_reveals_upper_shelf",
              open_bottom_z > shelf1[1][2] + 0.01,
              details=f"open bottom={open_bottom_z:.3f}, "
                      f"shelf1 top={shelf1[1][2]:.3f}")

    # === SHELF CHECKS ===

    for i in range(N_SHELVES):
        sh = ctx.part_element_world_aabb(carcass, elem=f"shelf_board_{i}")
        assert sh is not None
        sh_thk = sh[1][2] - sh[0][2]
        ctx.check(f"shelf_{i}_thin_board",
                  abs(sh_thk - SHELF_THK) < 0.002,
                  details=f"thickness={sh_thk:.4f}")
        sh_w = sh[1][1] - sh[0][1]
        ctx.check(f"shelf_{i}_spans_width",
                  sh_w > INNER_W * 0.8,
                  details=f"width={sh_w:.3f}")
        # Shelves are above the divider
        ctx.check(f"shelf_{i}_above_divider",
                  sh[0][2] > divider[1][2] - 0.002,
                  details=f"shelf bottom={sh[0][2]:.3f}")
        # Shelves are below the top
        ctx.check(f"shelf_{i}_below_top",
                  sh[1][2] < BODY_TOP + 0.002,
                  details=f"shelf top={sh[1][2]:.3f}")

    # Shelves are vertically separated (shelf 1 above shelf 0)
    ctx.check("shelves_vertically_ordered",
              shelf1[0][2] > shelf0[1][2] + 0.01,
              details=f"shelf0 top={shelf0[1][2]:.3f}, "
                      f"shelf1 bottom={shelf1[0][2]:.3f}")

    # === DRAWER CHECKS ===

    ctx.check("four_drawers", len(drawer_joints) == 4)
    for n, j in drawer_joints.items():
        ctx.check(f"{n}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{n}_axis_outward",
                  j.axis[0] > 0.99 and abs(j.axis[1]) < 0.01
                  and abs(j.axis[2]) < 0.01)
        ctx.check(f"{n}_range_0_to_040",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 0.40) < 1e-6)

    # Drawers are in the lower zone (below divider)
    for n in drawer_names:
        face = ctx.part_element_world_aabb(drawer_objs[n], elem="front_panel")
        assert face is not None
        ctx.check(f"{n}_below_divider",
                  face[1][2] < divider[0][2] + 0.01,
                  details=f"face top z={face[1][2]:.3f}, "
                          f"divider bottom={divider[0][2]:.3f}")

    # Drawer slide test
    for n in ("lower_top_drawer_0", "lower_bottom_drawer_1"):
        d, j = drawer_objs[n], drawer_joints[n]
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
        assert rest is not None and out is not None
        ctx.check(f"{n}_slides_forward",
                  abs((out[0] - rest[0]) - 0.40) < 1e-6,
                  details=f"dx={out[0] - rest[0]:.4f}")

    # Drawers have hollow open-top trays
    for n in drawer_names:
        tray = ctx.part_element_world_aabb(drawer_objs[n], elem="tray_bottom")
        wall = ctx.part_element_world_aabb(drawer_objs[n], elem="tray_side_wall_0")
        assert tray is not None and wall is not None
        ctx.check(f"{n}_open_top_tray",
                  wall[1][2] > tray[1][2] + 0.04,
                  details=f"wall top={wall[1][2]:.3f}, tray top={tray[1][2]:.3f}")

    # Two knobs per wide drawer
    for n in drawer_names:
        b0 = ctx.part_element_world_aabb(drawer_objs[n], elem="knob_ball_0")
        b1 = ctx.part_element_world_aabb(drawer_objs[n], elem="knob_ball_1")
        assert b0 is not None and b1 is not None
        ctx.check(f"{n}_two_knobs", True)

    # --- Carved posts exist and are stacked ---
    seg0 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_0")
    seg1 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_1")
    assert seg0 is not None and seg1 is not None
    w0 = seg0[1][1] - seg0[0][1]
    ctx.check("post_segments_rotated", w0 > POST_SQ + 0.008)
    ctx.check("post_segments_stacked", abs(seg1[0][2] - seg0[1][2]) < 0.004)

    return ctx.report()


object_model = build_object_model()
