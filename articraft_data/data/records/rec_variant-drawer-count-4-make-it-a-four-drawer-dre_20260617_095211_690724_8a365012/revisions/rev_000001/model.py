from __future__ import annotations

# Wide black wooden dresser — four-drawer variant (~1.70 m W x 0.85 m H x 0.50 m D).
#
# World layout: front faces +X (back of the body at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four square
# legs ~0.15 m tall. Matte black wood carcass and drawer fronts; a thin smooth
# silver-gray top slab overhangs the body ~0.02 m on all sides. The front
# corners carry decorative posts carved with a stacked spiral/faceted zigzag
# pattern, approximated as a vertical stack of alternately rotated square
# prisms, continuing down into the straight front legs.
#
# Front holds four drawers in two rows of two wide drawers (two knobs each).
# All four drawers are independent PRISMATIC joints sliding out along +X,
# range 0 to 0.40 m, each a hollow open-top tray behind a flat front panel
# that sits slightly proud of the carcass face.

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
H_TOTAL = 0.85           # overall height (Z)
OVERHANG = 0.020         # top slab overhang on all sides
TOP_THK = 0.022          # silver top slab thickness

BW = W_TOTAL - 2 * OVERHANG      # body width 1.66
BD = D_TOTAL - 2 * OVERHANG      # body depth 0.46 (back x=0, front x=BD)
LEG_H = 0.150
BODY_BOT = LEG_H                 # body bottom z
BODY_TOP = H_TOTAL - TOP_THK     # body top z (0.828)
BH = BODY_TOP - BODY_BOT         # body height

WALL = 0.018                     # carcass panel thickness
INNER_W = BW - 2 * WALL

# Front drawer zone (between bottom rail and top rail).
ZONE_BOT = BODY_BOT + 0.030      # 0.180
ZONE_TOP = BODY_TOP - 0.017      # 0.811
REVEAL = 0.008
N_DRAWERS = 4
N_ROWS = 2
N_COLS = 2

# Two rows of wide drawers filling the full zone.
FH_WIDE = (ZONE_TOP - ZONE_BOT - (N_ROWS - 1) * REVEAL) / N_ROWS

# Row centers (joint origin heights), bottom to top.
ROW_CZ = []
for r in range(N_ROWS):
    row_bot = ZONE_BOT + r * (FH_WIDE + REVEAL)
    ROW_CZ.append(row_bot + FH_WIDE / 2.0)

# Drawer-front horizontal layout between the carved corner posts.
OPEN_HW = 0.762                  # half-width of the drawer-front band
FW_WIDE = (2 * OPEN_HW - REVEAL) / 2.0        # ~0.764
WIDE_CY = [-(FW_WIDE / 2.0 + REVEAL / 2.0), (FW_WIDE / 2.0 + REVEAL / 2.0)]

FACE_THK = 0.018                 # drawer front panel thickness
FACE_PROUD = 0.0005              # clearance behind the proud front panel
FRONT_X = BD                     # carcass front plane (0.46)
SLAB_BACK_X = FRONT_X + FACE_PROUD
JOINT_X = SLAB_BACK_X + FACE_THK  # drawer frame origin = front of slab

TRAVEL = 0.400
TRAY_D = 0.420                   # tray depth (keeps rear inserted when open)
TRAY_T = 0.012

# Carved corner posts.
POST_SQ = 0.050                  # square cross-section of each carved segment
POST_CX = 0.452                  # post axis x (bulges slightly proud of front)
POST_CY = BW / 2.0 - 0.025       # post axis |y|
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG  # 1.5 mm vertical embed per seam

LEG_SQ = 0.050

# Knobs: polished silver ball on a short stem.
KNOB_R = 0.0125
STEM_R = 0.0055
STEM_L = 0.014


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
    side_len = TRAY_D + 0.002    # 2 mm embed into the front panel back
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
            # Cylinder long axis is local +Z; pitch by pi/2 to point along +X.
            # 4 mm of the stem embeds into the front panel.
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
    model = ArticulatedObject(name="black_dresser_four_drawers")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    tray_mat = model.material("tray_black", rgba=(0.13, 0.13, 0.14, 1.0))

    # ===================================================================
    # ROOT: carcass (hollow shell + legs + carved posts + silver top)
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
    # Bottom board and top stretcher board of the body.
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + 0.009)),
        material=black,
        name="bottom_board",
    )
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black,
        name="top_stretcher",
    )
    # Front frame rails (below the bottom drawers / above the top drawers).
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
    # Front side stiles filling the strip between drawer fronts and side panels.
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
    # Center stile behind the central vertical reveal between columns.
    carcass.visual(
        Box((0.016, 0.024, ZONE_TOP - ZONE_BOT)),
        origin=Origin(xyz=(BD - 0.008, 0.0, (ZONE_BOT + ZONE_TOP) / 2.0)),
        material=black,
        name="front_center_stile",
    )
    # Dust panels (drawer runners): one below each row.
    for row_idx in range(N_ROWS):
        cz = ROW_CZ[row_idx]
        tray_bot_z = cz + (-FH_WIDE / 2.0 + 0.012)
        carcass.visual(
            Box((BD - 0.030, INNER_W, 0.014)),
            origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, 0.0,
                               tray_bot_z - 0.007)),
            material=black_deep,
            name=f"dust_panel_{row_idx}",
        )

    # Silver-gray top slab with overhang on all sides.
    carcass.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=silver_top,
        name="top_slab",
    )

    # Carved spiral/zigzag corner posts: stacked alternately rotated prisms,
    # with odd segments nudged outward so the silhouette reads as a zigzag.
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

    # Four straight square legs (front legs continue the carved posts).
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
        Box((BD, BW, H_TOTAL)), mass=60.0)

    # ===================================================================
    # DRAWERS: four independent PRISMATIC slides along +X
    # Two rows × two columns, looped with drawer_{i} naming.
    # ===================================================================
    drawers = []
    for i in range(N_DRAWERS):
        row = i // N_COLS
        col = i % N_COLS
        cy = WIDE_CY[col]
        cz = ROW_CZ[row]
        d = _build_drawer(model, f"drawer_{i}", FW_WIDE, FH_WIDE,
                          0.700, FH_WIDE - 0.040,
                          [-0.19, 0.19], black, tray_mat, silver)
        drawers.append((f"drawer_{i}", d, cy, cz))

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
    drawer_names = [f"drawer_{i}" for i in range(N_DRAWERS)]
    parts = {n: object_model.get_part(n) for n in drawer_names}
    joints = {n: object_model.get_articulation(f"carcass_to_{n}")
              for n in drawer_names}

    # --- Grounding and true overall scale (~1.70 x 0.50 x 0.85 m) ---
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

    # --- Silver top slab overhangs the black body on all four sides ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert top is not None and side is not None and back is not None
    ctx.check(
        "top_overhang_all_sides",
        top[1][1] > side[1][1] + 0.015 and top[0][1] < -side[1][1] - 0.015
        and top[0][0] < back[0][0] - 0.015 and top[1][0] > 0.46 + 0.015,
        details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f}) x=({top[0][0]:.3f},{top[1][0]:.3f})",
    )
    ctx.check("top_is_thin_slab", abs((top[1][2] - top[0][2]) - TOP_THK) < 0.002)

    # --- Carved corner posts: stacked alternately rotated prisms ---
    seg0 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_0")
    seg1 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_1")
    assert seg0 is not None and seg1 is not None
    w0 = seg0[1][1] - seg0[0][1]
    ctx.check("post_segments_rotated", w0 > POST_SQ + 0.008,
              details=f"seg aabb width={w0:.4f} vs stock {POST_SQ}")
    ctx.check("post_segments_stacked", abs(seg1[0][2] - seg0[1][2]) < 0.004,
              details=f"seg1 bottom={seg1[0][2]:.4f}, seg0 top={seg0[1][2]:.4f}")
    top_seg = ctx.part_element_world_aabb(
        carcass, elem=f"carved_post_0_seg_{N_SEG - 1}")
    assert top_seg is not None
    ctx.check("posts_span_body_height",
              seg0[0][2] < LEG_H + 0.002 and top_seg[1][2] > BODY_TOP - 0.004,
              details=f"post z=({seg0[0][2]:.3f},{top_seg[1][2]:.3f})")
    # Front legs continue under the carved posts down to the floor.
    leg = ctx.part_element_world_aabb(carcass, elem="leg_front_0")
    assert leg is not None
    ctx.check("front_leg_under_post",
              abs(leg[0][2]) < 0.002 and leg[1][2] > LEG_H - 0.002
              and abs((leg[0][1] + leg[1][1]) / 2.0 - POST_CY) < 0.002,
              details=f"leg z=({leg[0][2]:.3f},{leg[1][2]:.3f})")

    # --- Four independent prismatic drawers, +X, 0..0.40 m ---
    ctx.check("four_drawers", len(joints) == N_DRAWERS)
    for n, j in joints.items():
        ctx.check(f"{n}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{n}_axis_out_front",
                  j.axis[0] > 0.99 and abs(j.axis[1]) < 0.01
                  and abs(j.axis[2]) < 0.01)
        ctx.check(f"{n}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 0.40) < 1e-6,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Front layout: two rows of two wide drawers, thin reveals ---
    fronts = {}
    for n in drawer_names:
        fronts[n] = ctx.part_element_world_aabb(parts[n], elem="front_panel")
    assert all(f is not None for f in fronts.values())

    # All fronts are the same wide-drawer height.
    front_heights = [fronts[n][1][2] - fronts[n][0][2] for n in drawer_names]
    ctx.check("uniform_front_height",
              max(front_heights) - min(front_heights) < 0.002,
              details=f"heights={[f'{h:.3f}' for h in front_heights]}")

    # Top row sits above bottom row.
    ctx.check("rows_stacked",
              fronts["drawer_2"][0][2] > fronts["drawer_0"][1][2],
              details=f"row1 bot={fronts['drawer_2'][0][2]:.3f}, row0 top={fronts['drawer_0'][1][2]:.3f}")

    # Horizontal reveal between columns.
    rev_v = fronts["drawer_1"][0][1] - fronts["drawer_0"][1][1]
    ctx.check("thin_reveal_vertical", 0.004 < rev_v < 0.02,
              details=f"reveal={rev_v:.4f}")

    # Vertical reveal between rows.
    rev_h = fronts["drawer_2"][0][2] - fronts["drawer_0"][1][2]
    ctx.check("thin_reveal_horizontal", 0.004 < rev_h < 0.02,
              details=f"reveal={rev_h:.4f}")

    # Columns aligned across rows.
    ctx.check("columns_aligned",
              abs(fronts["drawer_0"][0][1] - fronts["drawer_2"][0][1]) < 0.002
              and abs(fronts["drawer_1"][1][1] - fronts["drawer_3"][1][1]) < 0.002)

    # --- Closed pose: fronts slightly proud, trays nested, knobs proud ---
    carcass_front = 0.46
    for n, d in parts.items():
        face = ctx.part_element_world_aabb(d, elem="front_panel")
        tray = ctx.part_element_world_aabb(d, elem="tray_bottom")
        ball = ctx.part_element_world_aabb(d, elem="knob_ball_0")
        wall0 = ctx.part_element_world_aabb(d, elem="tray_side_wall_0")
        assert (face is not None and tray is not None and ball is not None
                and wall0 is not None)
        ctx.check(f"{n}_front_proud",
                  0.0 < face[1][0] - carcass_front < 0.03,
                  details=f"face front x={face[1][0]:.4f}")
        ctx.check(f"{n}_tray_nested",
                  tray[1][0] < carcass_front + 0.002 and tray[0][0] > 0.02,
                  details=f"tray x=({tray[0][0]:.3f},{tray[1][0]:.3f})")
        ctx.check(f"{n}_knob_ball_proud", ball[0][0] > face[1][0] + 0.002,
                  details=f"ball min x={ball[0][0]:.4f}")
        # Hollow open-top tray: walls rise well above the tray bottom.
        ctx.check(f"{n}_tray_open_top", wall0[1][2] > tray[1][2] + 0.05,
                  details=f"wall top={wall0[1][2]:.3f}")
        ctx.expect_within(d, carcass, axes="y", margin=0.001,
                          name=f"{n}_within_carcass_width")

    # All wide drawers carry two knobs.
    for n in drawer_names:
        b1 = ctx.part_element_world_aabb(parts[n], elem="knob_ball_1")
        assert b1 is not None
        ctx.check(f"{n}_two_knobs", True)

    # --- Open pose: slides 0.40 m out along +X, rear stays inserted ---
    for n in ("drawer_0", "drawer_2", "drawer_3"):
        d, j = parts[n], joints[n]
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

    # --- Independence: opening one drawer leaves its neighbors shut ---
    with ctx.pose({joints["drawer_0"]: 0.30}):
        nb = ctx.part_element_world_aabb(parts["drawer_1"],
                                         elem="front_panel")
        wb = ctx.part_element_world_aabb(parts["drawer_2"],
                                         elem="front_panel")
        assert nb is not None and wb is not None
        ctx.check("drawers_independent",
                  abs(nb[1][0] - (0.46 + FACE_PROUD + FACE_THK)) < 0.002
                  and abs(wb[1][0] - (0.46 + FACE_PROUD + FACE_THK)) < 0.002,
                  details=f"neighbor fronts x={nb[1][0]:.4f},{wb[1][0]:.4f}")

    return ctx.report()


object_model = build_object_model()
