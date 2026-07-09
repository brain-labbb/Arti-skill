from __future__ import annotations

# Wide black wooden cabinet (~1.70 m W x 0.85 m H x 0.50 m D).
# Variant of the double dresser: glass-framed upper doors and solid lower doors.
#
# World layout: front faces +X (back at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four square
# legs ~0.15 m tall. Matte black wood carcass and door frames; a thin smooth
# silver-gray top slab overhangs the body ~0.02 m on all sides. The front
# corners carry decorative posts carved with a stacked spiral/faceted zigzag
# pattern, continuing down into the straight front legs.
#
# Front holds four doors in two rows: upper row has two glass-framed doors,
# lower row has two solid black wood doors. Each door swings on a revolute
# hinge at its outer side edge, with visible hinge barrels. Positive joint
# angle opens the door outward (toward +X).

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

# Front door zone (between bottom rail and top rail).
ZONE_BOT = BODY_BOT + 0.030      # 0.180
ZONE_TOP = BODY_TOP - 0.017      # 0.811
REVEAL = 0.008
DIVIDER_Z = (ZONE_BOT + ZONE_TOP) / 2.0  # horizontal divider center

# Door heights (with reveals top and bottom of each sub-zone).
UPPER_DH = ZONE_TOP - REVEAL - (DIVIDER_Z + REVEAL)   # upper door height
LOWER_DH = (DIVIDER_Z - REVEAL) - (ZONE_BOT + REVEAL)  # lower door height

# Door vertical centers.
UPPER_CZ = (DIVIDER_Z + REVEAL + ZONE_TOP - REVEAL) / 2.0
LOWER_CZ = (ZONE_BOT + REVEAL + DIVIDER_Z - REVEAL) / 2.0

# Door horizontal layout between the carved corner posts.
OPEN_HW = 0.762                  # half-width of the door-front band
DOOR_W = OPEN_HW - REVEAL / 2.0  # each door width

FACE_THK = 0.018                 # door panel thickness
FACE_PROUD = 0.002               # clearance in front of carcass face
FRONT_X = BD                     # carcass front plane (0.46)
HINGE_X = FRONT_X + FACE_PROUD  # hinge origin X

# Frame bars for glass doors.
FRAME_BAR = 0.040                # frame bar width
GLASS_THK = 0.004                # glass panel thickness

# Carved corner posts.
POST_SQ = 0.050                  # square cross-section of each carved segment
POST_CX = 0.452                  # post axis x (bulges slightly proud of front)
POST_CY = BW / 2.0 - 0.025       # post axis |y|
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG

LEG_SQ = 0.050

# Knobs: polished silver ball on a short stem.
KNOB_R = 0.0125
STEM_R = 0.0055
STEM_L = 0.014

# Hinge barrels.
BARREL_R = 0.008
BARREL_H = 0.035
N_BARRELS_PER_HINGE = 3

# Door swing limit (about 90 degrees).
DOOR_SWING = 1.57


def _build_glass_door(model: ArticulatedObject, name: str, door_w: float,
                      door_h: float, y_sign: int,
                      black, glass, silver):
    """Glass-framed door. Local frame: origin at hinge point, face at x=0,
    door extends in y_sign*Y direction, thickness toward -X."""
    door = model.part(name)
    hw = door_w / 2.0
    hh = door_h / 2.0
    s = y_sign  # +1 for left (hinge at -Y, extends +Y), -1 for right

    # Frame bars (black wood).
    # Top bar.
    door.visual(
        Box((FACE_THK, door_w, FRAME_BAR)),
        origin=Origin(xyz=(-FACE_THK / 2.0, s * hw, hh - FRAME_BAR / 2.0)),
        material=black,
        name="frame_top",
    )
    # Bottom bar.
    door.visual(
        Box((FACE_THK, door_w, FRAME_BAR)),
        origin=Origin(xyz=(-FACE_THK / 2.0, s * hw, -hh + FRAME_BAR / 2.0)),
        material=black,
        name="frame_bottom",
    )
    # Hinge-side bar (at y=0 end).
    door.visual(
        Box((FACE_THK, FRAME_BAR, door_h - 2 * FRAME_BAR)),
        origin=Origin(xyz=(-FACE_THK / 2.0, s * FRAME_BAR / 2.0, 0.0)),
        material=black,
        name="frame_hinge_side",
    )
    # Free-edge bar (at y = s*door_w end).
    door.visual(
        Box((FACE_THK, FRAME_BAR, door_h - 2 * FRAME_BAR)),
        origin=Origin(xyz=(-FACE_THK / 2.0, s * (door_w - FRAME_BAR / 2.0), 0.0)),
        material=black,
        name="frame_free_edge",
    )

    # Glass panel (thin translucent sheet inset within frame).
    glass_w = door_w - 2 * FRAME_BAR
    glass_h = door_h - 2 * FRAME_BAR
    door.visual(
        Box((GLASS_THK, glass_w, glass_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, s * hw, 0.0)),
        material=glass,
        name="glass_panel",
    )

    # Knob on the free edge side.
    knob_y = s * (door_w - FRAME_BAR / 2.0)
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0, knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004, knob_y, 0.0)),
        material=silver,
        name="knob_ball",
    )

    door.inertial = Inertial.from_geometry(
        Box((FACE_THK, door_w, door_h)), mass=2.5)
    return door


def _build_solid_door(model: ArticulatedObject, name: str, door_w: float,
                      door_h: float, y_sign: int,
                      black, silver):
    """Solid wood door. Local frame: origin at hinge point, face at x=0,
    door extends in y_sign*Y direction, thickness toward -X."""
    door = model.part(name)
    hw = door_w / 2.0
    s = y_sign

    # Solid panel.
    door.visual(
        Box((FACE_THK, door_w, door_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0, s * hw, 0.0)),
        material=black,
        name="solid_panel",
    )

    # Knob on the free edge side.
    knob_y = s * (door_w - 0.06)
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0, knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004, knob_y, 0.0)),
        material=silver,
        name="knob_ball",
    )

    door.inertial = Inertial.from_geometry(
        Box((FACE_THK, door_w, door_h)), mass=3.0)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_cabinet_glass_upper_doors")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    glass = model.material("translucent_glass", rgba=(0.75, 0.82, 0.85, 0.30))

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
    # Bottom board.
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + 0.009)),
        material=black,
        name="bottom_board",
    )
    # Top stretcher board.
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black,
        name="top_stretcher",
    )
    # Face frame setback: frame members sit behind the closed door panels.
    # Door back X = HINGE_X - FACE_THK = 0.444; frame front should be behind.
    FF_X = HINGE_X - FACE_THK - WALL / 2.0 - 0.001  # 0.434

    # Front bottom rail.
    carcass.visual(
        Box((WALL, INNER_W, ZONE_BOT - BODY_BOT)),
        origin=Origin(xyz=(FF_X, 0.0,
                           (BODY_BOT + ZONE_BOT) / 2.0)),
        material=black,
        name="front_bottom_rail",
    )
    # Front top rail.
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - ZONE_TOP)),
        origin=Origin(xyz=(FF_X, 0.0,
                           (ZONE_TOP + BODY_TOP) / 2.0)),
        material=black,
        name="front_top_rail",
    )
    # Front side stiles.
    stile_w = BW / 2.0 - OPEN_HW + 0.004
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, stile_w, ZONE_TOP - ZONE_BOT)),
            origin=Origin(xyz=(FF_X,
                               s * (BW / 2.0 - stile_w / 2.0),
                               (ZONE_BOT + ZONE_TOP) / 2.0)),
            material=black,
            name=f"front_side_stile_{tag}",
        )
    # Center vertical stile (between left and right doors).
    carcass.visual(
        Box((WALL, 0.028, ZONE_TOP - ZONE_BOT)),
        origin=Origin(xyz=(FF_X, 0.0,
                           (ZONE_BOT + ZONE_TOP) / 2.0)),
        material=black,
        name="center_stile",
    )
    # Horizontal divider board between upper and lower door zones.
    carcass.visual(
        Box((BD - 0.020, INNER_W, 0.018)),
        origin=Origin(xyz=(0.020 + (BD - 0.020) / 2.0, 0.0, DIVIDER_Z)),
        material=black,
        name="horizontal_divider",
    )

    # Interior shelf (adds storage volume realism).
    carcass.visual(
        Box((BD - 0.040, INNER_W, 0.014)),
        origin=Origin(xyz=(0.030 + (BD - 0.040) / 2.0, 0.0,
                           BODY_BOT + 0.020)),
        material=black_deep,
        name="bottom_shelf",
    )

    # Silver-gray top slab with overhang on all sides.
    carcass.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=silver_top,
        name="top_slab",
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

    # --- Hinge barrels at each of the 4 hinge locations ---
    # Each hinge has a mounting plate and barrel segments.
    hinge_locations = [
        # (hinge_y, cz, door_h) for each hinge
        (-OPEN_HW, UPPER_CZ, UPPER_DH),  # upper left
        (+OPEN_HW, UPPER_CZ, UPPER_DH),  # upper right
        (-OPEN_HW, LOWER_CZ, LOWER_DH),  # lower left
        (+OPEN_HW, LOWER_CZ, LOWER_DH),  # lower right
    ]
    for h_idx, (hy, hcz, hdh) in enumerate(hinge_locations):
        # Hinge mounting plate (connects barrels to carcass front frame).
        # Plate extends from behind the front stile to the hinge line.
        plate_h = hdh * 0.8  # plate spans 80% of door height
        plate_back_x = FF_X + WALL / 2.0 - 0.003  # embed 3mm into stile
        plate_front_x = HINGE_X
        plate_depth = plate_front_x - plate_back_x
        plate_cx = (plate_back_x + plate_front_x) / 2.0
        carcass.visual(
            Box((plate_depth, 0.025, plate_h)),
            origin=Origin(xyz=(plate_cx, hy, hcz)),
            material=silver,
            name=f"hinge_plate_{h_idx}",
        )
        # Barrel segments along the hinge.
        barrel_spacing = hdh / (N_BARRELS_PER_HINGE + 1)
        for b in range(N_BARRELS_PER_HINGE):
            bz = hcz - hdh / 2.0 + barrel_spacing * (b + 1)
            carcass.visual(
                Cylinder(radius=BARREL_R, length=BARREL_H),
                origin=Origin(xyz=(HINGE_X, hy, bz)),
                material=silver,
                name=f"hinge_barrel_{h_idx}_{b}",
            )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=55.0)

    # ===================================================================
    # DOORS: four independent REVOLUTE hinges
    # ===================================================================
    # Upper doors: glass-framed.
    upper_left = _build_glass_door(
        model, "upper_left_door", DOOR_W, UPPER_DH, +1,
        black, glass, silver)
    upper_right = _build_glass_door(
        model, "upper_right_door", DOOR_W, UPPER_DH, -1,
        black, glass, silver)

    # Lower doors: solid wood.
    lower_left = _build_solid_door(
        model, "lower_left_door", DOOR_W, LOWER_DH, +1,
        black, silver)
    lower_right = _build_solid_door(
        model, "lower_right_door", DOOR_W, LOWER_DH, -1,
        black, silver)

    # Articulations: revolute hinges at door side edges.
    # Left doors: hinge at -OPEN_HW, door extends +Y, axis=(0,0,-1)
    #   => positive q swings free edge (+Y) toward +X (outward).
    # Right doors: hinge at +OPEN_HW, door extends -Y, axis=(0,0,1)
    #   => positive q swings free edge (-Y) toward +X (outward).
    door_specs = [
        ("carcass_to_upper_left_door", upper_left,
         -OPEN_HW, UPPER_CZ, (0.0, 0.0, -1.0)),
        ("carcass_to_upper_right_door", upper_right,
         +OPEN_HW, UPPER_CZ, (0.0, 0.0, 1.0)),
        ("carcass_to_lower_left_door", lower_left,
         -OPEN_HW, LOWER_CZ, (0.0, 0.0, -1.0)),
        ("carcass_to_lower_right_door", lower_right,
         +OPEN_HW, LOWER_CZ, (0.0, 0.0, 1.0)),
    ]
    for jname, door_part, hy, hcz, axis in door_specs:
        model.articulation(
            jname,
            ArticulationType.REVOLUTE,
            parent=carcass,
            child=door_part,
            origin=Origin(xyz=(HINGE_X, hy, hcz)),
            axis=axis,
            motion_limits=MotionLimits(effort=20.0, velocity=1.0,
                                       lower=0.0, upper=DOOR_SWING),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    door_names = ["upper_left_door", "upper_right_door",
                  "lower_left_door", "lower_right_door"]
    doors = {n: object_model.get_part(n) for n in door_names}
    joints = {n: object_model.get_articulation(f"carcass_to_{n}")
              for n in door_names}

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

    # --- Four doors, all revolute ---
    ctx.check("four_doors", len(joints) == 4)
    for n, j in joints.items():
        ctx.check(f"{n}_revolute",
                  j.articulation_type == ArticulationType.REVOLUTE)
        # Axis should be vertical (Z component dominant).
        ctx.check(f"{n}_axis_vertical",
                  abs(j.axis[2]) > 0.99 and abs(j.axis[0]) < 0.01
                  and abs(j.axis[1]) < 0.01,
                  details=f"axis={j.axis}")
        ctx.check(f"{n}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - DOOR_SWING) < 0.01,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Upper doors have glass panels, lower doors are solid ---
    for n in ("upper_left_door", "upper_right_door"):
        glass_vis = doors[n].get_visual("glass_panel")
        ctx.check(f"{n}_has_glass", glass_vis is not None)
        frame_vis = doors[n].get_visual("frame_top")
        ctx.check(f"{n}_has_frame", frame_vis is not None)

    for n in ("lower_left_door", "lower_right_door"):
        solid_vis = doors[n].get_visual("solid_panel")
        ctx.check(f"{n}_has_solid_panel", solid_vis is not None)
        # Lower doors should NOT have glass.
        has_glass = any(v.name == "glass_panel" for v in doors[n].visuals)
        ctx.check(f"{n}_no_glass", not has_glass)

    # --- Visible hinge barrels on the carcass ---
    hinge_barrel_count = sum(
        1 for v in carcass.visuals
        if v.name is not None and v.name.startswith("hinge_barrel_"))
    ctx.check("hinge_barrels_present",
              hinge_barrel_count >= 8,
              details=f"found {hinge_barrel_count} hinge barrel visuals")

    # --- Allow intentional overlap: hinge barrels at door/carcass junction ---
    # The hinge barrels sit at the hinge line where the door attaches to the
    # carcass. This overlap is intentional and represents the hinge hardware.
    for n in door_names:
        ctx.allow_overlap(
            carcass, doors[n],
            reason=f"Hinge barrels at the {n} hinge line intentionally overlap with the door panel/frame.",
        )
        # Proof: the door is properly hinged and swings correctly (checked below).

    # --- Closed pose: doors sit at the front face ---
    for n in door_names:
        d = doors[n]
        # Door face is at local x=0, which is at world x=HINGE_X.
        if n.startswith("upper"):
            elem = "frame_top"
        else:
            elem = "solid_panel"
        face = ctx.part_element_world_aabb(d, elem=elem)
        assert face is not None
        ctx.check(f"{n}_front_at_face",
                  abs(face[1][0] - HINGE_X) < 0.005,
                  details=f"face front x={face[1][0]:.4f}, expected ~{HINGE_X:.4f}")

    # --- Open pose: doors swing outward (knob/free edge moves toward +X) ---
    for n in ("upper_left_door", "lower_right_door"):
        d = doors[n]
        j = joints[n]
        rest_knob = ctx.part_element_world_aabb(d, elem="knob_ball")
        with ctx.pose({j: j.motion_limits.upper}):
            open_knob = ctx.part_element_world_aabb(d, elem="knob_ball")
        assert rest_knob is not None and open_knob is not None
        rest_knob_x = (rest_knob[0][0] + rest_knob[1][0]) / 2.0
        open_knob_x = (open_knob[0][0] + open_knob[1][0]) / 2.0
        ctx.check(f"{n}_swings_outward",
                  open_knob_x > rest_knob_x + 0.05,
                  details=f"rest_knob_x={rest_knob_x:.4f}, open_knob_x={open_knob_x:.4f}")

    # --- Upper/lower doors separated by horizontal divider ---
    div_aabb = ctx.part_element_world_aabb(carcass, elem="horizontal_divider")
    upper_aabb = ctx.part_world_aabb(doors["upper_left_door"])
    lower_aabb = ctx.part_world_aabb(doors["lower_left_door"])
    assert div_aabb is not None and upper_aabb is not None and lower_aabb is not None
    ctx.check("divider_between_doors",
              lower_aabb[1][2] < div_aabb[0][2] + 0.02
              and upper_aabb[0][2] > div_aabb[1][2] - 0.02,
              details=f"div z=({div_aabb[0][2]:.3f},{div_aabb[1][2]:.3f}), "
                      f"upper_bot={upper_aabb[0][2]:.3f}, lower_top={lower_aabb[1][2]:.3f}")

    # --- Left/right door hinge sides correct ---
    ul_j = joints["upper_left_door"]
    ur_j = joints["upper_right_door"]
    ctx.check("left_hinge_at_negative_y",
              ul_j.origin.xyz[1] < -0.5,
              details=f"hinge y={ul_j.origin.xyz[1]:.3f}")
    ctx.check("right_hinge_at_positive_y",
              ur_j.origin.xyz[1] > 0.5,
              details=f"hinge y={ur_j.origin.xyz[1]:.3f}")

    # --- All doors have knobs ---
    for n in door_names:
        knob = doors[n].get_visual("knob_ball")
        ctx.check(f"{n}_has_knob", knob is not None)

    return ctx.report()


object_model = build_object_model()
