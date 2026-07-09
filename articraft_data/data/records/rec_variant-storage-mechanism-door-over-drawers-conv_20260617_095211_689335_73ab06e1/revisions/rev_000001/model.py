from __future__ import annotations

# Wide black wooden storage cabinet (~1.70 m W x 0.85 m H x 0.50 m D).
# Variant: door-over-drawers — upper region has a pair of hinged cabinet doors
# (revolute about vertical Z), lower region has two prismatic drawers.
#
# World layout: front faces +X (back of the body at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four square
# legs ~0.15 m tall. Matte black wood carcass and fronts; a thin smooth
# silver-gray top slab overhangs the body ~0.02 m on all sides. The front
# corners carry decorative posts carved with a stacked spiral/faceted zigzag
# pattern, continuing down into the straight front legs.
#
# Front holds two cabinet doors (upper) and two wide drawers (lower).
# Doors are REVOLUTE about Z, hinged at outer vertical edges, opening outward.
# Drawers are PRISMATIC along +X, range 0 to 0.40 m, hollow open-top trays.

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

# Front opening zone (between bottom rail and top rail).
ZONE_BOT = BODY_BOT + 0.030      # 0.180
ZONE_TOP = BODY_TOP - 0.017      # 0.811
REVEAL = 0.008

# Lower drawer row (two wide drawers, same as parent bottom row).
FH_WIDE = 0.230
CZ_BOT = ZONE_BOT + FH_WIDE / 2.0  # 0.295

# Upper door zone: from top of drawer zone + reveal to top of opening.
DOOR_ZONE_BOT = ZONE_BOT + FH_WIDE + REVEAL  # 0.418
DOOR_H = ZONE_TOP - DOOR_ZONE_BOT             # ~0.393
DOOR_CZ = (DOOR_ZONE_BOT + ZONE_TOP) / 2.0   # ~0.6145

# Front opening horizontal layout between the carved corner posts.
OPEN_HW = 0.762                  # half-width of the front opening
FW_WIDE = (2 * OPEN_HW - REVEAL) / 2.0        # ~0.764
DOOR_W = OPEN_HW - REVEAL / 2.0               # ~0.758
WIDE_CY = [-(FW_WIDE / 2.0 + REVEAL / 2.0), (FW_WIDE / 2.0 + REVEAL / 2.0)]

FACE_THK = 0.018                 # front panel thickness
FACE_PROUD = 0.0005              # clearance behind the proud front panel
FRONT_X = BD                     # carcass front plane (0.46)
SLAB_BACK_X = FRONT_X + FACE_PROUD
JOINT_X = SLAB_BACK_X + FACE_THK  # frame origin = front of slab

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


def _build_door(model: ArticulatedObject, name: str, door_w: float,
                door_h: float, direction: int, black, silver):
    """Cabinet door in local frame: hinge at local origin (y=0), panel extends
    along direction * Y. Front panel outer surface at local x=0.
    direction: +1 for left door (extends +Y), -1 for right door (extends -Y)."""
    door = model.part(name)

    # Flat matte-black door panel.
    door.visual(
        Box((FACE_THK, door_w, door_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, direction * door_w / 2.0, 0.0)),
        material=black,
        name="panel",
    )

    # Thin back panel for visible depth when door is open.
    door.visual(
        Box((0.004, door_w - 0.020, door_h - 0.020)),
        origin=Origin(xyz=(-FACE_THK - 0.002,
                           direction * door_w / 2.0, 0.0)),
        material=black,
        name="back_panel",
    )

    # Knob on the free edge side (opposite the hinge).
    knob_y = direction * (door_w - 0.045)
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0, knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem_0",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004, knob_y, 0.0)),
        material=silver,
        name="knob_ball_0",
    )

    door.inertial = Inertial.from_geometry(
        Box((FACE_THK, door_w, door_h)), mass=3.0)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_cabinet_door_over_drawers")

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
    for i, s in ((0, 1), (1, -1)):
        carcass.visual(
            Box((BD, WALL, BH)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                               BODY_BOT + BH / 2.0)),
            material=black,
            name=f"side_panel_{i}",
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
    # Front frame rails (below the bottom drawers / above the doors).
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
    # Front side stiles filling the strip between fronts and side panels.
    stile_w = BW / 2.0 - OPEN_HW + 0.004
    for i, s in ((0, 1), (1, -1)):
        carcass.visual(
            Box((WALL, stile_w, ZONE_TOP - ZONE_BOT)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (BW / 2.0 - stile_w / 2.0),
                               (ZONE_BOT + ZONE_TOP) / 2.0)),
            material=black,
            name=f"front_side_stile_{i}",
        )
    # Center stile behind the central vertical reveal (doors above, drawers below).
    carcass.visual(
        Box((0.016, 0.024, ZONE_TOP - ZONE_BOT)),
        origin=Origin(xyz=(BD - 0.008, 0.0, (ZONE_BOT + ZONE_TOP) / 2.0)),
        material=black,
        name="front_center_stile",
    )
    # Horizontal divider rail between door zone (above) and drawer zone (below).
    divider_h = 0.018
    divider_cz = ZONE_BOT + FH_WIDE + divider_h / 2.0
    carcass.visual(
        Box((WALL, INNER_W, divider_h)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, divider_cz)),
        material=black,
        name="divider_rail",
    )
    # Interior shelf at the divider height (visible when doors are open).
    carcass.visual(
        Box((BD - 0.030, INNER_W, 0.014)),
        origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, 0.0,
                           DOOR_ZONE_BOT - 0.007)),
        material=black_deep,
        name="interior_shelf",
    )
    # Dust panel for the lower drawer row (drawer runners).
    tray_bot_z = CZ_BOT + (-FH_WIDE / 2.0 + 0.012)
    carcass.visual(
        Box((BD - 0.030, INNER_W, 0.014)),
        origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, 0.0,
                           tray_bot_z - 0.007)),
        material=black_deep,
        name="bottom_dust_panel",
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
    for ptag, s in ((0, 1), (1, -1)):
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
    # DOORS: two cabinet doors, REVOLUTE about vertical Z
    # ===================================================================
    # Left door (i=0): hinge at left edge (y = -OPEN_HW), extends +Y
    #   axis = (0, 0, -1): positive q rotates +Y toward +X (outward)
    # Right door (i=1): hinge at right edge (y = +OPEN_HW), extends -Y
    #   axis = (0, 0, +1): positive q rotates -Y toward +X (outward)
    door_configs = [
        # (name, hinge_y, direction, axis_z)
        ("door_0", -OPEN_HW, +1, -1.0),
        ("door_1", +OPEN_HW, -1, +1.0),
    ]
    for dname, hinge_y, direction, axis_z in door_configs:
        door = _build_door(model, dname, DOOR_W, DOOR_H, direction,
                           black, silver)
        model.articulation(
            f"carcass_to_{dname}",
            ArticulationType.REVOLUTE,
            parent=carcass,
            child=door,
            origin=Origin(xyz=(JOINT_X, hinge_y, DOOR_CZ)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=20.0, velocity=1.5,
                                       lower=0.0, upper=1.4),
        )

    # ===================================================================
    # DRAWERS: two wide drawers in the lower row, PRISMATIC along +X
    # ===================================================================
    for i, cy in enumerate(WIDE_CY):
        d = _build_drawer(model, f"drawer_{i}", FW_WIDE, FH_WIDE,
                          0.700, 0.185, [-0.19, 0.19],
                          black, tray_mat, silver)
        model.articulation(
            f"carcass_to_drawer_{i}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=d,
            origin=Origin(xyz=(JOINT_X, cy, CZ_BOT)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=50.0, velocity=0.5,
                                       lower=0.0, upper=TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    door_names = [f"door_{i}" for i in range(2)]
    drawer_names = [f"drawer_{i}" for i in range(2)]

    doors = {n: object_model.get_part(n) for n in door_names}
    drawers = {n: object_model.get_part(n) for n in drawer_names}
    door_joints = {n: object_model.get_articulation(f"carcass_to_{n}")
                   for n in door_names}
    drawer_joints = {n: object_model.get_articulation(f"carcass_to_{n}")
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
    leg = ctx.part_element_world_aabb(carcass, elem="leg_front_0")
    assert leg is not None
    ctx.check("front_leg_under_post",
              abs(leg[0][2]) < 0.002 and leg[1][2] > LEG_H - 0.002
              and abs((leg[0][1] + leg[1][1]) / 2.0 - POST_CY) < 0.002,
              details=f"leg z=({leg[0][2]:.3f},{leg[1][2]:.3f})")

    # --- Two doors: REVOLUTE about Z axis ---
    ctx.check("two_doors", len(door_joints) == 2)
    for n, j in door_joints.items():
        ctx.check(f"{n}_revolute",
                  j.articulation_type == ArticulationType.REVOLUTE)
        ctx.check(f"{n}_axis_vertical",
                  abs(j.axis[2]) > 0.99 and abs(j.axis[0]) < 0.01
                  and abs(j.axis[1]) < 0.01,
                  details=f"axis={j.axis}")
        ctx.check(f"{n}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and j.motion_limits.upper > 1.0,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Two drawers: PRISMATIC along +X, 0..0.40 m ---
    ctx.check("two_drawers", len(drawer_joints) == 2)
    for n, j in drawer_joints.items():
        ctx.check(f"{n}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{n}_axis_out_front",
                  j.axis[0] > 0.99 and abs(j.axis[1]) < 0.01
                  and abs(j.axis[2]) < 0.01)
        ctx.check(f"{n}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 0.40) < 1e-6,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Doors above drawers: vertical layout ---
    door_panels = [ctx.part_element_world_aabb(doors[n], elem="panel")
                   for n in door_names]
    drawer_fronts = [ctx.part_element_world_aabb(drawers[n], elem="front_panel")
                     for n in drawer_names]
    assert all(p is not None for p in door_panels + drawer_fronts)
    # Door bottoms are above drawer tops.
    ctx.check("doors_above_drawers",
              door_panels[0][0][2] > drawer_fronts[0][1][2] + 0.003,
              details=f"door bot z={door_panels[0][0][2]:.4f}, "
                      f"drawer top z={drawer_fronts[0][1][2]:.4f}")

    # --- Two doors meet at center with thin reveal ---
    left_edge = door_panels[0][1][1]   # max Y of left door
    right_edge = door_panels[1][0][1]  # min Y of right door
    ctx.check("doors_center_reveal",
              0.003 < right_edge - left_edge < 0.020,
              details=f"gap={right_edge - left_edge:.4f}")

    # --- Doors and drawers cover the full front opening width ---
    ctx.check("left_door_at_left_edge",
              abs(door_panels[0][0][1] - (-OPEN_HW)) < 0.005,
              details=f"left door min y={door_panels[0][0][1]:.4f}")
    ctx.check("right_door_at_right_edge",
              abs(door_panels[1][1][1] - OPEN_HW) < 0.005,
              details=f"right door max y={door_panels[1][1][1]:.4f}")

    # --- Closed pose: door panels proud, drawer fronts proud, knobs proud ---
    carcass_front = BD
    for n in door_names:
        panel = ctx.part_element_world_aabb(doors[n], elem="panel")
        ball = ctx.part_element_world_aabb(doors[n], elem="knob_ball_0")
        assert panel is not None and ball is not None
        ctx.check(f"{n}_panel_proud",
                  0.0 < panel[1][0] - carcass_front < 0.03,
                  details=f"panel front x={panel[1][0]:.4f}")
        ctx.check(f"{n}_knob_proud",
                  ball[0][0] > panel[1][0] + 0.002,
                  details=f"ball min x={ball[0][0]:.4f}")

    for n in drawer_names:
        face = ctx.part_element_world_aabb(drawers[n], elem="front_panel")
        tray = ctx.part_element_world_aabb(drawers[n], elem="tray_bottom")
        ball = ctx.part_element_world_aabb(drawers[n], elem="knob_ball_0")
        assert face is not None and tray is not None and ball is not None
        ctx.check(f"{n}_front_proud",
                  0.0 < face[1][0] - carcass_front < 0.03,
                  details=f"face front x={face[1][0]:.4f}")
        ctx.check(f"{n}_tray_nested",
                  tray[1][0] < carcass_front + 0.002 and tray[0][0] > 0.02,
                  details=f"tray x=({tray[0][0]:.3f},{tray[1][0]:.3f})")
        ctx.check(f"{n}_knob_proud",
                  ball[0][0] > face[1][0] + 0.002,
                  details=f"ball min x={ball[0][0]:.4f}")
        ctx.expect_within(drawers[n], carcass, axes="y", margin=0.001,
                          name=f"{n}_within_carcass_width")

    # Wide drawers carry two knobs each.
    for n in drawer_names:
        b1 = ctx.part_element_world_aabb(drawers[n], elem="knob_ball_1")
        assert b1 is not None
        ctx.check(f"{n}_two_knobs", True)

    # --- Doors open outward: positive pose moves panel free edge in +X ---
    for n in door_names:
        d = doors[n]
        j = door_joints[n]
        rest_panel = ctx.part_element_world_aabb(d, elem="panel")
        assert rest_panel is not None
        rest_center_x = (rest_panel[0][0] + rest_panel[1][0]) / 2.0
        with ctx.pose({j: j.motion_limits.upper}):
            open_panel = ctx.part_element_world_aabb(d, elem="panel")
            assert open_panel is not None
            open_max_x = open_panel[1][0]
        ctx.check(f"{n}_opens_outward",
                  open_max_x > rest_center_x + 0.10,
                  details=f"rest center x={rest_center_x:.4f}, "
                          f"open max x={open_max_x:.4f}")

    # --- Drawers slide out 0.40 m, rear stays inserted ---
    for n in drawer_names:
        d = drawers[n]
        j = drawer_joints[n]
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

    # --- Independence: opening one door leaves the other closed ---
    with ctx.pose({door_joints["door_0"]: 1.0}):
        d1_panel = ctx.part_element_world_aabb(doors["door_1"], elem="panel")
        assert d1_panel is not None
        ctx.check("doors_independent",
                  abs(d1_panel[1][0] - JOINT_X) < 0.003,
                  details=f"door_1 front x={d1_panel[1][0]:.4f}")

    # --- Drawer independence: opening one leaves the other shut ---
    with ctx.pose({drawer_joints["drawer_0"]: 0.30}):
        d1_face = ctx.part_element_world_aabb(drawers["drawer_1"],
                                              elem="front_panel")
        assert d1_face is not None
        ctx.check("drawers_independent",
                  abs(d1_face[1][0] - JOINT_X) < 0.003,
                  details=f"drawer_1 front x={d1_face[1][0]:.4f}")

    return ctx.report()


object_model = build_object_model()