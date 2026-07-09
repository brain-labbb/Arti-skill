from __future__ import annotations

# Low wide black dresser cabinet (~1.70 m W x 0.60 m H x 0.50 m D).
#
# Variant 22: forked from the 8-drawer double dresser into a low cabinet with
# three horizontal drawers, a hinged top lid, and interior shelf boards visible
# through the drawer gap seams.
#
# World layout: front faces +X (back of body at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four square
# legs ~0.15 m tall. Matte black wood carcass and drawer fronts; a thin smooth
# silver-gray top lid hinges upward from the rear edge. Front corners carry
# decorative carved posts continuing into the legs.
#
# Three wide drawers slide out along +X on independent prismatic joints,
# range 0 to 0.40 m, each a hollow open-top tray with two silver ball knobs.
# Interior shelf boards are visible through the reveal gaps around the fronts.

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
W_TOTAL = 1.70           # overall width including top overhang (Y)
D_TOTAL = 0.50           # overall depth including top overhang (X)
H_TOTAL = 0.60           # overall height (Z) — low dresser
OVERHANG = 0.020         # top slab overhang on all sides
TOP_THK = 0.022          # silver top lid thickness

BW = W_TOTAL - 2 * OVERHANG      # body width 1.66
BD = D_TOTAL - 2 * OVERHANG      # body depth 0.46
LEG_H = 0.150
BODY_BOT = LEG_H                 # body bottom z
BODY_TOP = H_TOTAL - TOP_THK     # body top z (0.578)
BH = BODY_TOP - BODY_BOT         # body height

WALL = 0.018                     # carcass panel thickness
INNER_W = BW - 2 * WALL

# Front drawer zone (between bottom rail and top rail).
ZONE_BOT = BODY_BOT + 0.025      # 0.175
ZONE_TOP = BODY_TOP - 0.015      # 0.563
REVEAL = 0.008                   # thin gap seam between drawer fronts

# Three equal-height horizontal drawers.
FH = (ZONE_TOP - ZONE_BOT - 2 * REVEAL) / 3.0  # ~0.121

# Row centers (joint origin heights).
CZ = [
    ZONE_BOT + FH / 2.0,
    ZONE_BOT + FH + REVEAL + FH / 2.0,
    ZONE_BOT + 2 * (FH + REVEAL) + FH / 2.0,
]

# Drawer-front horizontal layout between the carved corner posts.
OPEN_HW = 0.762                  # half-width of the drawer-front band
FW = (2 * OPEN_HW - 2 * REVEAL) / 3.0  # ~0.503
DRAWER_CY = [
    -(FW + REVEAL),
    0.0,
    (FW + REVEAL),
]

FACE_THK = 0.018                 # drawer front panel thickness
FACE_PROUD = 0.0005              # clearance behind the proud front panel
FRONT_X = BD                     # carcass front plane (0.46)
SLAB_BACK_X = FRONT_X + FACE_PROUD
JOINT_X = SLAB_BACK_X + FACE_THK  # drawer frame origin = front of slab

TRAVEL = 0.400
TRAY_D = 0.420                   # tray depth
TRAY_T = 0.012

# Carved corner posts.
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025
N_SEG = 10
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG

LEG_SQ = 0.050

# Knobs: polished silver ball on a short stem.
KNOB_R = 0.0125
STEM_R = 0.0055
STEM_L = 0.014

# Shelves inside the carcass (visible through gaps).
SHELF_THK = 0.014


def _build_drawer(model: ArticulatedObject, name: str, front_w: float,
                  front_h: float, tray_w: float, tray_h: float,
                  knob_ys: list[float], black, tray_mat, silver):
    """Drawer in local frame: front panel outer surface at local x=0,
    panel spans x in [-FACE_THK, 0], hollow open-top tray extends toward -X."""
    drawer = model.part(name)

    # Flat matte-black front panel, slightly proud of the carcass face.
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

    # --- silver ball knobs on short stems ---
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
    model = ArticulatedObject(name="low_wide_dresser_cabinet")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    tray_mat = model.material("tray_black", rgba=(0.13, 0.13, 0.14, 1.0))
    shelf_mat = model.material("shelf_wood", rgba=(0.12, 0.11, 0.10, 1.0))

    # ===================================================================
    # ROOT: carcass (hollow shell + legs + carved posts + shelves)
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
    # Top stretcher board (supports the lid from below).
    carcass.visual(
        Box((BD, INNER_W, 0.016)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.008)),
        material=black,
        name="top_stretcher",
    )

    # Front frame rails (below and above the drawer zone).
    carcass.visual(
        Box((WALL, INNER_W, ZONE_BOT - BODY_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (BODY_BOT + ZONE_BOT) / 2.0)),
        material=black,
        name="front_bottom_rail",
    )
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - ZONE_TOP)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (ZONE_TOP + BODY_TOP) / 2.0)),
        material=black,
        name="front_top_rail",
    )

    # Front side stiles (between posts and drawer fronts).
    stile_w = BW / 2.0 - OPEN_HW + 0.004
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, stile_w, ZONE_TOP - ZONE_BOT)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (BW / 2.0 - stile_w / 2.0),
                               (ZONE_BOT + ZONE_TOP) / 2.0)),
            material=black,
            name=f"front_side_stile_{tag}",
        )

    # Vertical dividers between the three drawers (creating gap seams).
    for tag, cy in (("0", -(FW / 2.0 + REVEAL / 2.0)),
                    ("1", (FW / 2.0 + REVEAL / 2.0))):
        carcass.visual(
            Box((WALL, REVEAL, ZONE_TOP - ZONE_BOT)),
            origin=Origin(xyz=(BD - WALL / 2.0, cy,
                               (ZONE_BOT + ZONE_TOP) / 2.0)),
            material=black_deep,
            name=f"front_divider_{tag}",
        )

    # Horizontal divider strips between drawers (gap seam rails).
    for tag, cz_gap in (("0", ZONE_BOT + FH + REVEAL / 2.0),
                        ("1", ZONE_BOT + 2 * FH + 1.5 * REVEAL)):
        carcass.visual(
            Box((WALL, 2 * OPEN_HW, REVEAL)),
            origin=Origin(xyz=(BD - WALL / 2.0, 0.0, cz_gap)),
            material=black_deep,
            name=f"horizontal_gap_rail_{tag}",
        )

    # Dust panels (drawer runners) between each drawer row.
    for i in range(3):
        tray_bot_z = CZ[i] + (-FH / 2.0 + 0.012)
        carcass.visual(
            Box((BD - 0.030, INNER_W, 0.014)),
            origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, 0.0,
                               tray_bot_z - 0.007)),
            material=black_deep,
            name=f"dust_panel_{i}",
        )

    # --- Interior shelf boards (visible through gap seams) ---
    # Two shelves at heights between drawer rows, spanning inner width and depth.
    shelf_depth = BD - 0.040
    for i, sz in enumerate([
        BODY_BOT + BH * 0.35,
        BODY_BOT + BH * 0.65,
    ]):
        carcass.visual(
            Box((shelf_depth, INNER_W - 0.010, SHELF_THK)),
            origin=Origin(xyz=(WALL + 0.020 + shelf_depth / 2.0, 0.0, sz)),
            material=shelf_mat,
            name=f"shelf_board_{i}",
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
        Box((BD, BW, H_TOTAL)), mass=45.0)

    # ===================================================================
    # LID: silver-gray top panel, hinged at rear
    # ===================================================================
    lid = model.part("lid")

    # Lid slab: in lid local frame, origin at hinge line (rear edge).
    # Slab extends forward along +X and has overhang behind hinge.
    lid_cx = BD / 2.0  # center x in lid frame
    lid.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(lid_cx, 0.0, TOP_THK / 2.0)),
        material=silver_top,
        name="lid_slab",
    )
    # Thin hinge barrel reinforcement on top surface near rear edge.
    lid.visual(
        Box((0.040, 0.180, 0.006)),
        origin=Origin(xyz=(0.020, 0.0, TOP_THK + 0.003)),
        material=silver_top,
        name="lid_hinge_reinforce",
    )

    lid.inertial = Inertial.from_geometry(
        Box((D_TOTAL, W_TOTAL, TOP_THK)), mass=3.0)

    # Lid hinge: revolute joint at the rear top edge of the carcass.
    # Axis (0, -1, 0): right-hand rule around -Y lifts +X (front edge) upward.
    model.articulation(
        "carcass_to_lid",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                   lower=0.0, upper=1.30),
    )

    # ===================================================================
    # DRAWERS: three independent PRISMATIC slides along +X
    # ===================================================================
    drawers = []
    for i, cy in enumerate(DRAWER_CY):
        d = _build_drawer(model, f"drawer_{i}", FW, FH,
                          FW - 0.040, FH - 0.025,
                          [-0.15, 0.15], black, tray_mat, silver)
        drawers.append((f"drawer_{i}", d, cy, CZ[i]))

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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    lid = object_model.get_part("lid")
    drawer_names = [f"drawer_{i}" for i in range(3)]
    drawers = {n: object_model.get_part(n) for n in drawer_names}
    drawer_joints = {n: object_model.get_articulation(f"carcass_to_{n}")
                     for n in drawer_names}
    lid_joint = object_model.get_articulation("carcass_to_lid")

    # --- Grounding and overall scale (~1.70 x 0.50 x 0.60 m) ---
    cb = ctx.part_world_aabb(carcass)
    lb = ctx.part_world_aabb(lid)
    assert cb is not None and lb is not None
    ctx.check("legs_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")
    # Overall envelope combines carcass and closed lid.
    overall_min_z = min(cb[0][2], lb[0][2])
    overall_max_z = max(cb[1][2], lb[1][2])
    width_y = max(cb[1][1], lb[1][1]) - min(cb[0][1], lb[0][1])
    depth_x = max(cb[1][0], lb[1][0]) - min(cb[0][0], lb[0][0])
    height_z = overall_max_z - overall_min_z
    ctx.check("width_170", abs(width_y - 1.70) < 0.02,
              details=f"w={width_y:.3f}")
    ctx.check("depth_050", abs(depth_x - 0.50) < 0.04,
              details=f"d={depth_x:.3f}")
    ctx.check("height_060", abs(height_z - 0.60) < 0.02,
              details=f"h={height_z:.3f}")
    ctx.check("is_low_cabinet", height_z < 0.70,
              details=f"h={height_z:.3f}")

    # --- Lid: revolute hinge at rear, opens upward ---
    ctx.check("lid_hinge_revolute",
              lid_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("lid_hinge_axis_lateral",
              abs(lid_joint.axis[1]) > 0.99
              and abs(lid_joint.axis[0]) < 0.01
              and abs(lid_joint.axis[2]) < 0.01,
              details=f"axis={lid_joint.axis}")
    ctx.check("lid_hinge_range",
              abs(lid_joint.motion_limits.lower) < 1e-9
              and lid_joint.motion_limits.upper > 1.0,
              details=f"range=({lid_joint.motion_limits.lower},"
                      f"{lid_joint.motion_limits.upper})")

    # Lid hinge origin is near the rear (back) of the carcass.
    hinge_pos = lid_joint.origin.xyz
    ctx.check("lid_hinge_at_rear",
              hinge_pos[0] < BD * 0.15,
              details=f"hinge x={hinge_pos[0]:.4f}")

    # Closed pose: lid sits at top of carcass.
    lid_slab = ctx.part_element_world_aabb(lid, elem="lid_slab")
    assert lid_slab is not None
    ctx.check("lid_closed_at_top",
              abs(lid_slab[0][2] - BODY_TOP) < 0.005,
              details=f"lid bottom z={lid_slab[0][2]:.4f}")

    # Open pose: lid front edge rises above closed position.
    rest_lid_pos = ctx.part_world_position(lid)
    with ctx.pose({lid_joint: lid_joint.motion_limits.upper}):
        open_lid_pos = ctx.part_world_position(lid)
        open_slab = ctx.part_element_world_aabb(lid, elem="lid_slab")
    assert rest_lid_pos is not None and open_lid_pos is not None
    assert open_slab is not None
    ctx.check("lid_opens_upward",
              open_slab[1][2] > BODY_TOP + 0.10,
              details=f"open top z={open_slab[1][2]:.4f}")

    # --- Three horizontal drawers ---
    ctx.check("three_drawers", len(drawer_joints) == 3)
    for n, j in drawer_joints.items():
        ctx.check(f"{n}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{n}_axis_out_front",
                  j.axis[0] > 0.99 and abs(j.axis[1]) < 0.01
                  and abs(j.axis[2]) < 0.01)
        ctx.check(f"{n}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 0.40) < 1e-6,
                  details=f"range=({j.motion_limits.lower},"
                          f"{j.motion_limits.upper})")

    # Drawer fronts are horizontal (wider than tall).
    for n, d in drawers.items():
        face = ctx.part_element_world_aabb(d, elem="front_panel")
        assert face is not None
        fw = face[1][1] - face[0][1]
        fh = face[1][2] - face[0][2]
        ctx.check(f"{n}_horizontal_front", fw > fh * 2.5,
                  details=f"w={fw:.3f} h={fh:.3f}")

    # Drawer fronts stacked vertically with gap seams.
    faces = [ctx.part_element_world_aabb(drawers[n], elem="front_panel")
             for n in drawer_names]
    assert all(f is not None for f in faces)
    ctx.check("drawers_stacked_vertically",
              faces[2][0][2] > faces[1][1][2] > faces[0][1][2],
              details=f"z ranges: {[(f[0][2], f[1][2]) for f in faces]}")

    # Gap seams between adjacent drawer fronts.
    gap_01 = faces[1][0][2] - faces[0][1][2]
    gap_12 = faces[2][0][2] - faces[1][1][2]
    ctx.check("gap_seam_01", 0.003 < gap_01 < 0.020,
              details=f"gap={gap_01:.4f}")
    ctx.check("gap_seam_12", 0.003 < gap_12 < 0.020,
              details=f"gap={gap_12:.4f}")

    # Each wide drawer has two knobs.
    for n, d in drawers.items():
        b0 = ctx.part_element_world_aabb(d, elem="knob_ball_0")
        b1 = ctx.part_element_world_aabb(d, elem="knob_ball_1")
        assert b0 is not None and b1 is not None
        ctx.check(f"{n}_two_knobs", True)

    # Closed pose: fronts proud, trays nested inside carcass.
    carcass_front = BD
    for n, d in drawers.items():
        face = ctx.part_element_world_aabb(d, elem="front_panel")
        tray = ctx.part_element_world_aabb(d, elem="tray_bottom")
        assert face is not None and tray is not None
        ctx.check(f"{n}_front_proud",
                  0.0 < face[1][0] - carcass_front < 0.03,
                  details=f"face front x={face[1][0]:.4f}")
        ctx.check(f"{n}_tray_nested",
                  tray[1][0] < carcass_front + 0.002 and tray[0][0] > 0.02,
                  details=f"tray x=({tray[0][0]:.3f},{tray[1][0]:.3f})")
        ctx.expect_within(d, carcass, axes="y", margin=0.001,
                          name=f"{n}_within_carcass_width")

    # Open pose: slides 0.40 m out, rear stays inserted.
    for n in drawer_names:
        d, j = drawers[n], drawer_joints[n]
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
            rear = ctx.part_element_world_aabb(d, elem="tray_back_wall")
            assert rear is not None
            rear_x = rear[0][0]
        assert rest is not None and out is not None
        ctx.check(f"{n}_slides_forward",
                  abs((out[0] - rest[0]) - 0.40) < 1e-6,
                  details=f"dx={out[0] - rest[0]:.4f}")
        ctx.check(f"{n}_retains_insertion", rear_x < carcass_front - 0.005,
                  details=f"open rear x={rear_x:.4f}")

    # --- Shelf boards inside the carcass ---
    shelf_0 = ctx.part_element_world_aabb(carcass, elem="shelf_board_0")
    shelf_1 = ctx.part_element_world_aabb(carcass, elem="shelf_board_1")
    assert shelf_0 is not None and shelf_1 is not None
    ctx.check("shelves_inside_body_height",
              shelf_0[0][2] > BODY_BOT and shelf_1[1][2] < BODY_TOP,
              details=f"shelf0 z=({shelf_0[0][2]:.3f},{shelf_0[1][2]:.3f})"
                      f" shelf1 z=({shelf_1[0][2]:.3f},{shelf_1[1][2]:.3f})")
    ctx.check("shelves_are_thin_boards",
              (shelf_0[1][2] - shelf_0[0][2]) < 0.025
              and (shelf_1[1][2] - shelf_1[0][2]) < 0.025)
    ctx.check("shelves_stacked_vertically",
              shelf_1[0][2] > shelf_0[1][2] + 0.01,
              details=f"gap={shelf_1[0][2] - shelf_0[1][2]:.4f}")
    # Shelves span a good portion of the carcass width.
    shelf_w = shelf_0[1][1] - shelf_0[0][1]
    ctx.check("shelves_span_width", shelf_w > INNER_W * 0.8,
              details=f"shelf w={shelf_w:.3f}")

    # --- Carved corner posts still present ---
    seg0 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_0")
    assert seg0 is not None
    top_seg = ctx.part_element_world_aabb(
        carcass, elem=f"carved_post_0_seg_{N_SEG - 1}")
    assert top_seg is not None
    ctx.check("posts_span_body_height",
              seg0[0][2] < LEG_H + 0.002 and top_seg[1][2] > BODY_TOP - 0.01,
              details=f"post z=({seg0[0][2]:.3f},{top_seg[1][2]:.3f})")

    # --- Independence: opening one drawer leaves neighbors shut ---
    with ctx.pose({drawer_joints["drawer_1"]: 0.30}):
        nb = ctx.part_element_world_aabb(drawers["drawer_0"],
                                         elem="front_panel")
        assert nb is not None
        ctx.check("drawers_independent",
                  abs(nb[1][0] - (BD + FACE_PROUD + FACE_THK)) < 0.002,
                  details=f"neighbor front x={nb[1][0]:.4f}")

    return ctx.report()


object_model = build_object_model()
