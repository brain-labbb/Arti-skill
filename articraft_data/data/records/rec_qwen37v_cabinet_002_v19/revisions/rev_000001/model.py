from __future__ import annotations

# Variant 19: wide black wooden cabinet/dresser (~1.70 m W x 0.85 m H x 0.50 m D)
# forked from the double dresser. Changes:
#   - Lift-up silver-gray lid over shallow top storage (REVOLUTE hinge at back)
#   - Visible barrel hinges on the lid hinge side
#   - Tambour front (horizontal slats) slides sideways on a PRISMATIC joint
#   - Four small top drawers with separate bar pull handles (replacing knobs)
#   - Lower section: open cabinet cavity behind the tambour
#
# World layout: front faces +X (back of the body at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four square
# legs ~0.15 m tall. Matte black wood carcass; silver-gray lid on top.

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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------- key dimensions (meters) ----------
W_TOTAL = 1.70           # overall width including top overhang (Y)
D_TOTAL = 0.50           # overall depth including top overhang (X)
H_TOTAL = 0.85           # overall height (Z)
OVERHANG = 0.020         # lid overhang on all sides
TOP_THK = 0.022          # silver lid slab thickness

BW = W_TOTAL - 2 * OVERHANG      # body width 1.66
BD = D_TOTAL - 2 * OVERHANG      # body depth 0.46
LEG_H = 0.150
BODY_BOT = LEG_H                 # body bottom z = 0.15
BODY_TOP = H_TOTAL - TOP_THK     # body top z = 0.828
BH = BODY_TOP - BODY_BOT         # body height

WALL = 0.018                     # carcass panel thickness
INNER_W = BW - 2 * WALL

# --- Vertical zones ---
# Shallow storage under the lid: top ~0.08 m of body
SHALLOW_BOT = BODY_TOP - 0.080   # 0.748
SHALLOW_DIVIDER = 0.014          # thin divider board thickness

# Drawer zone: below shallow storage, above bottom rail
DRAWER_ZONE_TOP = SHALLOW_BOT - SHALLOW_DIVIDER  # ~0.734
DRAWER_ZONE_BOT = BODY_BOT + 0.030               # 0.180
REVEAL = 0.008

# Four small drawers in one row
FH_DRAWER = (DRAWER_ZONE_TOP - DRAWER_ZONE_BOT) * 0.38  # ~0.211 each
TAMBOUR_ZONE_TOP = DRAWER_ZONE_BOT - REVEAL             # bottom of tambour area
TAMBOUR_ZONE_BOT = DRAWER_ZONE_BOT

# Recalculate: drawers at top of front, tambour below
# Drawer row: single row of 4 small drawers
DRW_TOP = DRAWER_ZONE_TOP - REVEAL
DRW_BOT = DRW_TOP - FH_DRAWER
DRW_CZ = (DRW_TOP + DRW_BOT) / 2.0

# Tambour zone: from bottom rail to below drawers
TMB_TOP = DRW_BOT - REVEAL
TMB_BOT = DRAWER_ZONE_BOT
TMB_H = TMB_TOP - TMB_BOT
TMB_CZ = (TMB_TOP + TMB_BOT) / 2.0

# Drawer-front horizontal layout between the carved corner posts.
OPEN_HW = 0.762                  # half-width of the drawer-front band
FW_SMALL = (2 * OPEN_HW - 3 * REVEAL) / 4.0
SMALL_CY = [-(FW_SMALL * 1.5 + REVEAL * 1.5),
            -(FW_SMALL * 0.5 + REVEAL * 0.5),
            (FW_SMALL * 0.5 + REVEAL * 0.5),
            (FW_SMALL * 1.5 + REVEAL * 1.5)]

FACE_THK = 0.018
FACE_PROUD = 0.0005
FRONT_X = BD                     # carcass front plane (0.46)
SLAB_BACK_X = FRONT_X + FACE_PROUD
JOINT_X = SLAB_BACK_X + FACE_THK

TRAVEL = 0.400
TRAY_D = 0.420
TRAY_T = 0.012

# Carved corner posts.
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG

LEG_SQ = 0.050

# Pull handles: horizontal bar on two stems
HANDLE_BAR_R = 0.005
HANDLE_BAR_LEN = 0.10
HANDLE_STEM_R = 0.004
HANDLE_STEM_LEN = 0.020
HANDLE_SPREAD = 0.035  # half-distance between stems

# Tambour: horizontal slats
TMB_SLAT_COUNT = 10
TMB_SLAT_GAP = 0.003
TMB_SLAT_H = (TMB_H - (TMB_SLAT_COUNT - 1) * TMB_SLAT_GAP) / TMB_SLAT_COUNT
TMB_FACE_THK = 0.014
TMB_TRAVEL = 0.80  # slides sideways 0.80 m

# Hinge barrels
HINGE_BARREL_R = 0.008
HINGE_BARREL_LEN = 0.040
HINGE_COUNT = 3


def _build_drawer(model, name, front_w, front_h, tray_w, tray_h,
                  black, tray_mat, silver):
    """Small drawer with bar pull handle. Local frame: front panel outer
    surface at local x=0, panel spans x in [-FACE_THK, 0], hollow open-top
    tray extends toward -X."""
    drawer = model.part(name)

    # Flat matte-black front panel.
    drawer.visual(
        Box((FACE_THK, front_w, front_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, 0.0)),
        material=black,
        name="front_panel",
    )

    # --- hollow open-top tray ---
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

    # --- bar pull handle (two stems + horizontal bar) ---
    # Stems protrude from front panel along +X
    for tag, dy in (("0", -HANDLE_SPREAD), ("1", HANDLE_SPREAD)):
        drawer.visual(
            Cylinder(radius=HANDLE_STEM_R, length=HANDLE_STEM_LEN + 0.006),
            origin=Origin(xyz=((HANDLE_STEM_LEN - 0.006) / 2.0, dy, 0.0),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=silver,
            name=f"handle_stem_{tag}",
        )
    # Horizontal bar connecting the stems
    drawer.visual(
        Cylinder(radius=HANDLE_BAR_R, length=HANDLE_BAR_LEN),
        origin=Origin(xyz=(HANDLE_STEM_LEN + HANDLE_BAR_R, 0.0, 0.0),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="handle_bar",
    )

    drawer.inertial = Inertial.from_geometry(
        Box((TRAY_D, tray_w, tray_h)), mass=4.0)
    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_cabinet_tambour_lid")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    tray_mat = model.material("tray_black", rgba=(0.13, 0.13, 0.14, 1.0))
    hinge_mat = model.material("hinge_brass", rgba=(0.70, 0.60, 0.30, 1.0))

    # ===================================================================
    # ROOT: carcass
    # ===================================================================
    carcass = model.part("carcass")

    # Side panels.
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD, WALL, BH)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                               BODY_BOT + BH / 2.0)),
            material=black,
            name=f"side_panel_{tag}",
        )
    # Back panel.
    carcass.visual(
        Box((WALL, INNER_W, BH)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=black,
        name="back_panel",
    )
    # Bottom board.
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + 0.009)),
        material=black,
        name="bottom_board",
    )
    # Divider board between tambour cavity and drawer row.
    carcass.visual(
        Box((BD - 0.030, INNER_W, SHALLOW_DIVIDER)),
        origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, 0.0,
                           (DRW_BOT + DRW_TOP) / 2.0 - FH_DRAWER / 2.0 - SHALLOW_DIVIDER / 2.0)),
        material=black,
        name="drawer_divider",
    )
    # Top stretcher / shallow storage floor.
    carcass.visual(
        Box((BD, INNER_W, SHALLOW_DIVIDER)),
        origin=Origin(xyz=(BD / 2.0, 0.0, SHALLOW_BOT - SHALLOW_DIVIDER / 2.0)),
        material=black,
        name="shallow_floor",
    )
    # Shallow storage walls (low rim around the top compartment).
    rim_h = 0.050
    # Front rim of shallow storage.
    carcass.visual(
        Box((WALL, INNER_W, rim_h)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, SHALLOW_BOT + rim_h / 2.0)),
        material=black,
        name="shallow_front_rim",
    )
    # Back rim.
    carcass.visual(
        Box((WALL, INNER_W, rim_h)),
        origin=Origin(xyz=(WALL + WALL / 2.0, 0.0, SHALLOW_BOT + rim_h / 2.0)),
        material=black,
        name="shallow_back_rim",
    )
    # Side rims.
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD - 2 * WALL, WALL, rim_h)),
            origin=Origin(xyz=(WALL + (BD - 2 * WALL) / 2.0,
                               s * (BW / 2.0 - WALL - WALL / 2.0),
                               SHALLOW_BOT + rim_h / 2.0)),
            material=black,
            name=f"shallow_side_rim_{tag}",
        )

    # Front frame rails (below tambour / above drawers).
    carcass.visual(
        Box((WALL, INNER_W, TMB_BOT - BODY_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (BODY_BOT + TMB_BOT) / 2.0)),
        material=black,
        name="front_bottom_rail",
    )
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - SHALLOW_BOT - rim_h)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (SHALLOW_BOT + rim_h + BODY_TOP) / 2.0)),
        material=black,
        name="front_top_rail",
    )
    # Front side stiles.
    stile_w = BW / 2.0 - OPEN_HW + 0.004
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, stile_w, DRW_TOP - DRW_BOT)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (BW / 2.0 - stile_w / 2.0),
                               (DRW_BOT + DRW_TOP) / 2.0)),
            material=black,
            name=f"front_side_stile_{tag}",
        )
    # Center stile behind the central vertical reveal of the small-drawer row.
    carcass.visual(
        Box((0.016, 0.024, DRW_TOP - DRW_BOT)),
        origin=Origin(xyz=(BD - 0.008, 0.0, (DRW_BOT + DRW_TOP) / 2.0)),
        material=black,
        name="front_center_stile",
    )
    # Short stiles behind the two outer reveals of the small-drawer row.
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((0.016, 0.024, FH_DRAWER)),
            origin=Origin(xyz=(BD - 0.008, s * (FW_SMALL + REVEAL), DRW_CZ)),
            material=black,
            name=f"drawer_row_stile_{tag}",
        )
    # Dust panel / runner for the small drawers.
    tray_bot_z = DRW_CZ + (-FH_DRAWER / 2.0 + 0.012)
    carcass.visual(
        Box((BD - 0.030, INNER_W, 0.014)),
        origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, 0.0,
                           tray_bot_z - 0.007)),
        material=black_deep,
        name="drawer_dust_panel",
    )

    # Tambour track rails (top and bottom guide rails for the tambour).
    rail_thk = 0.012
    carcass.visual(
        Box((WALL, INNER_W, rail_thk)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, TMB_TOP + rail_thk / 2.0)),
        material=black_deep,
        name="tambour_top_rail",
    )
    carcass.visual(
        Box((WALL, INNER_W, rail_thk)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, TMB_BOT - rail_thk / 2.0)),
        material=black_deep,
        name="tambour_bottom_rail",
    )
    # Tambour pocket (side recess where the tambour slides into).
    pocket_w = 0.060
    carcass.visual(
        Box((BD - 0.040, pocket_w, TMB_H + 2 * rail_thk)),
        origin=Origin(xyz=(0.020 + (BD - 0.040) / 2.0,
                           BW / 2.0 - WALL - pocket_w / 2.0,
                           (TMB_BOT + TMB_TOP) / 2.0)),
        material=black_deep,
        name="tambour_pocket",
    )

    # Carved spiral/zigzag corner posts.
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

    # Four straight square legs.
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
    # LID: lift-up top with silver-gray surface and hinge barrels
    # ===================================================================
    lid = model.part("lid")

    # The lid panel: silver-gray slab. In local frame, hinge edge at x=0,
    # panel extends along +X toward the front.
    lid.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(D_TOTAL / 2.0, 0.0, TOP_THK / 2.0)),
        material=silver_top,
        name="lid_panel",
    )
    # Thin black lip around the underside edge of the lid.
    lip_h = 0.008
    for tag, dy, w in (("front", 0.0, W_TOTAL), ("back", 0.0, W_TOTAL)):
        pass  # simplified - lid is thin enough
    # Underside lip on all four edges for a finished look.
    lid.visual(
        Box((D_TOTAL - 0.010, 0.008, lip_h)),
        origin=Origin(xyz=(D_TOTAL / 2.0, (W_TOTAL / 2.0 - 0.004), -lip_h / 2.0)),
        material=black,
        name="lid_lip_side_0",
    )
    lid.visual(
        Box((D_TOTAL - 0.010, 0.008, lip_h)),
        origin=Origin(xyz=(D_TOTAL / 2.0, -(W_TOTAL / 2.0 - 0.004), -lip_h / 2.0)),
        material=black,
        name="lid_lip_side_1",
    )
    lid.visual(
        Box((0.008, W_TOTAL - 0.010, lip_h)),
        origin=Origin(xyz=(D_TOTAL - 0.004, 0.0, -lip_h / 2.0)),
        material=black,
        name="lid_lip_front",
    )

    # Hinge barrels on the back edge (hinge side).
    # These are visible brass cylinders that form the hinge knuckle.
    hinge_spacing = BW / (HINGE_COUNT + 1)
    for i in range(HINGE_COUNT):
        hy = -BW / 2.0 + hinge_spacing * (i + 1)
        # Barrel cylinder aligned along Y (the hinge pin axis).
        lid.visual(
            Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_LEN),
            origin=Origin(xyz=(HINGE_BARREL_R * 0.5, hy, 0.0),
                          rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_mat,
            name=f"hinge_barrel_{i}",
        )
        # Small leaf plate connecting barrel to lid underside.
        lid.visual(
            Box((0.025, HINGE_BARREL_LEN + 0.008, 0.002)),
            origin=Origin(xyz=(0.0125 + HINGE_BARREL_R * 0.3, hy, -0.001)),
            material=hinge_mat,
            name=f"hinge_leaf_{i}",
        )

    lid.inertial = Inertial.from_geometry(
        Box((D_TOTAL, W_TOTAL, TOP_THK)), mass=3.0)

    # Lid articulation: REVOLUTE at the back edge of the carcass top.
    # Origin: back-top edge of carcass. The lid hinge edge aligns here.
    # axis=(0, -1, 0): positive rotation lifts the front edge of the lid upward.
    model.articulation(
        "carcass_to_lid",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0,
                                   lower=0.0, upper=1.5),
    )

    # ===================================================================
    # TAMBOUR: horizontal slat front that slides sideways (+Y)
    # ===================================================================
    tambour = model.part("tambour")

    # Tambour slats: horizontal boards stacked vertically with small gaps.
    slat_w = 2 * OPEN_HW  # spans between the corner posts
    for i in range(TMB_SLAT_COUNT):
        sz = TMB_BOT + i * (TMB_SLAT_H + TMB_SLAT_GAP) + TMB_SLAT_H / 2.0
        tambour.visual(
            Box((TMB_FACE_THK, slat_w, TMB_SLAT_H)),
            origin=Origin(xyz=(BD - TMB_FACE_THK / 2.0, 0.0, sz)),
            material=black,
            name=f"tambour_slat_{i}",
        )
    # Backing strip connecting slats (thin flexible backing).
    tambour.visual(
        Box((0.003, slat_w - 0.020, TMB_H - 0.010)),
        origin=Origin(xyz=(BD - TMB_FACE_THK - 0.0015, 0.0,
                           (TMB_BOT + TMB_TOP) / 2.0)),
        material=black_deep,
        name="tambour_backing",
    )
    # Pull handle on the tambour (small recessed grip).
    tambour.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(BD + 0.003, 0.25, (TMB_BOT + TMB_TOP) / 2.0),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="tambour_pull",
    )

    tambour.inertial = Inertial.from_geometry(
        Box((TMB_FACE_THK, slat_w, TMB_H)), mass=5.0)

    # Tambour articulation: PRISMATIC along +Y (slides sideways to the right).
    model.articulation(
        "carcass_to_tambour",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=tambour,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.5,
                                   lower=0.0, upper=TMB_TRAVEL),
    )

    # ===================================================================
    # DRAWERS: four small prismatic slides along +X with bar pull handles
    # ===================================================================
    drawers = []
    for i, cy in enumerate(SMALL_CY):
        d = _build_drawer(model, f"drawer_{i}", FW_SMALL, FH_DRAWER,
                          0.330, 0.160, black, tray_mat, silver)
        drawers.append((f"drawer_{i}", d, cy))

    for name, d, cy in drawers:
        model.articulation(
            f"carcass_to_{name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=d,
            origin=Origin(xyz=(JOINT_X, cy, DRW_CZ)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=50.0, velocity=0.5,
                                       lower=0.0, upper=TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    lid = object_model.get_part("lid")
    tambour = object_model.get_part("tambour")
    drawer_names = [f"drawer_{i}" for i in range(4)]
    drawers = {n: object_model.get_part(n) for n in drawer_names}
    drawer_joints = {n: object_model.get_articulation(f"carcass_to_{n}")
                     for n in drawer_names}
    lid_joint = object_model.get_articulation("carcass_to_lid")
    tambour_joint = object_model.get_articulation("carcass_to_tambour")

    # --- Overall dimensions ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("legs_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    ctx.check("width_170", abs(width_y - 1.70) < 0.02, details=f"w={width_y:.3f}")
    ctx.check("depth_050", abs(depth_x - 0.50) < 0.04, details=f"d={depth_x:.3f}")
    # Height includes the lid sitting on top of the carcass.
    lid_aabb = ctx.part_world_aabb(lid)
    assert lid_aabb is not None
    height_z = lid_aabb[1][2] - cb[0][2]
    ctx.check("height_085", abs(height_z - 0.85) < 0.01,
              details=f"h={height_z:.3f}")

    # Hinge barrels intentionally embed at the hinge line where the lid
    # meets the carcass back panel. This is local mechanical seating.
    ctx.allow_overlap(
        carcass, lid,
        elem_a="back_panel", elem_b="hinge_barrel_0",
        reason="Hinge barrel seats into the back panel at the hinge pivot line.",
    )
    ctx.allow_overlap(
        carcass, lid,
        elem_a="back_panel", elem_b="hinge_barrel_1",
        reason="Hinge barrel seats into the back panel at the hinge pivot line.",
    )
    ctx.allow_overlap(
        carcass, lid,
        elem_a="back_panel", elem_b="hinge_barrel_2",
        reason="Hinge barrel seats into the back panel at the hinge pivot line.",
    )
    # Prove the barrels stay near the hinge line.
    ctx.expect_contact(carcass, lid, elem_a="back_panel", elem_b="hinge_barrel_0",
                       contact_tol=0.015, name="barrel_0_seated_at_hinge")

    # --- Lid: REVOLUTE joint that opens upward ---
    ctx.check("lid_is_revolute",
              lid_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("lid_axis_lateral",
              abs(lid_joint.axis[1]) > 0.99,
              details=f"axis={lid_joint.axis}")
    ctx.check("lid_limits_0_to_15",
              abs(lid_joint.motion_limits.lower) < 1e-9
              and abs(lid_joint.motion_limits.upper - 1.5) < 1e-6)

    # Lid closed pose: sits on top of the carcass.
    lid_closed = ctx.part_element_world_aabb(lid, elem="lid_panel")
    assert lid_closed is not None
    ctx.check("lid_on_top",
              lid_closed[0][2] > BODY_TOP - 0.005,
              details=f"lid bottom z={lid_closed[0][2]:.4f}")

    # Lid open pose: front edge rises.
    rest_z = ctx.part_world_position(lid)
    with ctx.pose({lid_joint: 1.2}):
        open_pos = ctx.part_world_position(lid)
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_panel")
    assert rest_z is not None and open_pos is not None and lid_open is not None
    ctx.check("lid_opens_upward",
              lid_open[1][2] > BODY_TOP + 0.15,
              details=f"open lid top z={lid_open[1][2]:.4f}")

    # --- Hinge barrels exist on the lid ---
    hinge0 = ctx.part_element_world_aabb(lid, elem="hinge_barrel_0")
    hinge1 = ctx.part_element_world_aabb(lid, elem="hinge_barrel_1")
    hinge2 = ctx.part_element_world_aabb(lid, elem="hinge_barrel_2")
    assert hinge0 is not None and hinge1 is not None and hinge2 is not None
    ctx.check("hinge_barrels_exist", True)
    # Hinge barrels are near the back edge (small x).
    ctx.check("hinge_barrels_at_back",
              hinge0[0][0] < 0.05,
              details=f"barrel0 min x={hinge0[0][0]:.4f}")
    # Hinges are spaced apart along Y.
    ctx.check("hinges_spaced",
              abs(hinge2[0][1] - hinge0[0][1]) > 0.20,
              details=f"span y={hinge2[0][1] - hinge0[0][1]:.4f}")

    # --- Tambour: PRISMATIC joint sliding sideways ---
    ctx.check("tambour_is_prismatic",
              tambour_joint.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("tambour_axis_sideways",
              abs(tambour_joint.axis[1]) > 0.99,
              details=f"axis={tambour_joint.axis}")
    ctx.check("tambour_range_080",
              abs(tambour_joint.motion_limits.lower) < 1e-9
              and abs(tambour_joint.motion_limits.upper - TMB_TRAVEL) < 1e-6)

    # Tambour closed: covers the front opening.
    tmb_closed = ctx.part_element_world_aabb(tambour, elem="tambour_slat_0")
    assert tmb_closed is not None
    ctx.check("tambour_covers_front",
              tmb_closed[1][0] > BD - 0.01,
              details=f"tambour front x={tmb_closed[1][0]:.4f}")

    # Tambour open: slides sideways.
    tmb_rest = ctx.part_world_position(tambour)
    with ctx.pose({tambour_joint: TMB_TRAVEL}):
        tmb_open = ctx.part_world_position(tambour)
    assert tmb_rest is not None and tmb_open is not None
    ctx.check("tambour_slides_sideways",
              abs((tmb_open[1] - tmb_rest[1]) - TMB_TRAVEL) < 1e-6,
              details=f"dy={tmb_open[1] - tmb_rest[1]:.4f}")

    # Tambour has multiple visible slats.
    slat_top = ctx.part_element_world_aabb(tambour, elem=f"tambour_slat_{TMB_SLAT_COUNT - 1}")
    assert slat_top is not None
    ctx.check("tambour_has_slats",
              slat_top[1][2] > tmb_closed[0][2] + 0.10,
              details=f"top slat z={slat_top[1][2]:.4f}")

    # --- Four drawers with bar pull handles ---
    ctx.check("four_drawers", len(drawer_joints) == 4)
    for n, j in drawer_joints.items():
        ctx.check(f"{n}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{n}_axis_out_front",
                  j.axis[0] > 0.99 and abs(j.axis[1]) < 0.01
                  and abs(j.axis[2]) < 0.01)
        ctx.check(f"{n}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - TRAVEL) < 1e-6)

    # Drawers have bar pull handles (not knobs).
    for n in drawer_names:
        d = drawers[n]
        bar = ctx.part_element_world_aabb(d, elem="handle_bar")
        assert bar is not None
        ctx.check(f"{n}_has_pull_handle",
                  (bar[1][1] - bar[0][1]) > 0.06,
                  details=f"bar span y={bar[1][1] - bar[0][1]:.4f}")
        # Handle protrudes from the front panel.
        face = ctx.part_element_world_aabb(d, elem="front_panel")
        assert face is not None
        ctx.check(f"{n}_handle_proud",
                  bar[0][0] > face[1][0] - 0.001,
                  details=f"bar min x={bar[0][0]:.4f}, face max x={face[1][0]:.4f}")

    # Drawer closed: fronts proud, trays nested.
    carcass_front = BD
    for n in drawer_names:
        d = drawers[n]
        face = ctx.part_element_world_aabb(d, elem="front_panel")
        tray = ctx.part_element_world_aabb(d, elem="tray_bottom")
        assert face is not None and tray is not None
        ctx.check(f"{n}_front_proud",
                  0.0 < face[1][0] - carcass_front < 0.03,
                  details=f"face front x={face[1][0]:.4f}")
        ctx.check(f"{n}_tray_nested",
                  tray[1][0] < carcass_front + 0.002 and tray[0][0] > 0.02,
                  details=f"tray x=({tray[0][0]:.3f},{tray[1][0]:.3f})")

    # Drawer open: slides out, rear stays inserted.
    for n in ("drawer_1", "drawer_3"):
        d, j = drawers[n], drawer_joints[n]
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
            rear = ctx.part_element_world_aabb(d, elem="tray_back_wall")
        assert rest is not None and out is not None and rear is not None
        ctx.check(f"{n}_slides_forward",
                  abs((out[0] - rest[0]) - TRAVEL) < 1e-6,
                  details=f"dx={out[0] - rest[0]:.4f}")
        ctx.check(f"{n}_retains_insertion",
                  rear[0][0] < carcass_front - 0.005,
                  details=f"open rear x={rear[0][0]:.4f}")

    # Independence: opening one drawer leaves neighbors shut.
    with ctx.pose({drawer_joints["drawer_1"]: 0.30}):
        nb = ctx.part_element_world_aabb(drawers["drawer_0"], elem="front_panel")
        assert nb is not None
        ctx.check("drawers_independent",
                  abs(nb[1][0] - (BD + FACE_PROUD + FACE_THK)) < 0.002,
                  details=f"neighbor front x={nb[1][0]:.4f}")

    # Drawers are in upper zone, tambour in lower zone.
    drw_face = ctx.part_element_world_aabb(drawers["drawer_0"], elem="front_panel")
    tmb_face = ctx.part_element_world_aabb(tambour, elem="tambour_slat_0")
    assert drw_face is not None and tmb_face is not None
    ctx.check("drawers_above_tambour",
              drw_face[0][2] > tmb_face[1][2] - 0.01,
              details=f"drawer bot z={drw_face[0][2]:.3f}, tambour top z={tmb_face[1][2]:.3f}")

    # Non-fixed joints exist (at least lid hinge and tambour slide).
    non_fixed = [a for a in object_model.articulations
                 if a.articulation_type != ArticulationType.FIXED]
    ctx.check("has_non_fixed_joints",
              len(non_fixed) >= 2,
              details=f"non-fixed count={len(non_fixed)}")

    return ctx.report()


object_model = build_object_model()
