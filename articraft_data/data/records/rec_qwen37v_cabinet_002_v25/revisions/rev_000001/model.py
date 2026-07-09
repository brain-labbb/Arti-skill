from __future__ import annotations

# Cabinet variant: glass-framed upper doors, solid lower doors, hinged top lid.
# Based on the wide black wooden dresser carcass (~1.70 m W x 0.85 m H x 0.50 m D).
#
# World layout: front faces +X (back at x=0, front at x=BD),
# width along Y (centered), height along +Z, grounded at z=0.
# Matte black wood carcass; silver-gray hinged top lid.
# Front has two sections divided by a middle shelf:
#   Upper: two glass-framed doors with visible hinge barrels
#   Lower: two solid wood doors with pull handles and hinge barrels
# All four doors are REVOLUTE joints hinged at the outer edges, opening outward.
# Top lid is REVOLUTE on a rear hinge line, opening upward.

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

WALL = 0.018
INNER_W = BW - 2 * WALL

# Front opening zone
ZONE_BOT = BODY_BOT + 0.030      # 0.180
ZONE_TOP = BODY_TOP - 0.017      # 0.811
REVEAL = 0.008

# Middle divider shelf
DIVIDER_Z = 0.470
DIVIDER_THK = 0.018

# Upper and lower opening heights
UPPER_OPEN_BOT = DIVIDER_Z + DIVIDER_THK / 2.0   # 0.479
UPPER_OPEN_TOP = ZONE_TOP                          # 0.811
LOWER_OPEN_BOT = ZONE_BOT                          # 0.180
LOWER_OPEN_TOP = DIVIDER_Z - DIVIDER_THK / 2.0    # 0.461

UPPER_DOOR_H = UPPER_OPEN_TOP - UPPER_OPEN_BOT - REVEAL   # ~0.324
LOWER_DOOR_H = LOWER_OPEN_TOP - LOWER_OPEN_BOT - REVEAL   # ~0.273

# Door widths (each door fills half the opening minus center reveal)
OPEN_HW = 0.762                  # half-width of front opening
DOOR_W = OPEN_HW - REVEAL / 2.0  # ~0.758

FACE_THK = 0.018
FACE_PROUD = 0.002

# Glass door frame
FRAME_BAR = 0.038
GLASS_THK = 0.004

# Carved corner posts
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG
LEG_SQ = 0.050

# Top lid
LID_THK = 0.020
LID_DEPTH = D_TOTAL - OVERHANG   # 0.48 (no rear overhang past hinge)

# Hinge barrels
HINGE_R = 0.006
HINGE_H = 0.030

# Pull handle
HANDLE_BAR_W = 0.008
HANDLE_BAR_H = 0.100
HANDLE_STEM_R = 0.004
HANDLE_STEM_L = 0.018

# Computed door center heights
UPPER_CZ = (UPPER_OPEN_BOT + UPPER_OPEN_TOP) / 2.0
LOWER_CZ = (LOWER_OPEN_BOT + LOWER_OPEN_TOP) / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_door_cabinet")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    glass = model.material("cabinet_glass", rgba=(0.75, 0.82, 0.88, 0.35))

    # ===================================================================
    # ROOT: carcass (shell + legs + carved posts + frame + divider)
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
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + 0.009)),
        material=black, name="bottom_board",
    )

    # Top stretcher board
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black, name="top_stretcher",
    )

    # Front bottom rail
    carcass.visual(
        Box((WALL, INNER_W, ZONE_BOT - BODY_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (BODY_BOT + ZONE_BOT) / 2.0)),
        material=black, name="front_bottom_rail",
    )

    # Front top rail
    carcass.visual(
        Box((WALL, INNER_W, BODY_TOP - ZONE_TOP)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (ZONE_TOP + BODY_TOP) / 2.0)),
        material=black, name="front_top_rail",
    )

    # Front center stile (vertical, between left and right doors)
    carcass.visual(
        Box((0.016, 0.024, ZONE_TOP - ZONE_BOT)),
        origin=Origin(xyz=(BD - 0.008, 0.0,
                           (ZONE_BOT + ZONE_TOP) / 2.0)),
        material=black, name="front_center_stile",
    )

    # Front side stiles
    stile_w = BW / 2.0 - OPEN_HW + 0.004
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, stile_w, ZONE_TOP - ZONE_BOT)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (BW / 2.0 - stile_w / 2.0),
                               (ZONE_BOT + ZONE_TOP) / 2.0)),
            material=black, name=f"front_side_stile_{tag}",
        )

    # Middle divider shelf
    carcass.visual(
        Box((BD - 0.030, INNER_W, DIVIDER_THK)),
        origin=Origin(xyz=(0.020 + (BD - 0.030) / 2.0, 0.0, DIVIDER_Z)),
        material=black, name="divider_shelf",
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
                material=black_deep, name=f"carved_post_{ptag}_seg_{i}",
            )

    # Four straight square legs
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
    # UPPER DOORS (glass-framed, 2 doors hinged at outer edges)
    # ===================================================================
    for idx, (side, sy) in enumerate((("left", -1), ("right", 1))):
        door = model.part(f"upper_door_{idx}")
        # dcy = panel center y offset from hinge (extends away from hinge toward center)
        dcy = -sy * DOOR_W / 2.0
        px = FACE_PROUD - FACE_THK / 2.0  # panel center x in local frame

        # Frame: top rail
        door.visual(
            Box((FACE_THK, DOOR_W, FRAME_BAR)),
            origin=Origin(xyz=(px, dcy, UPPER_DOOR_H / 2.0 - FRAME_BAR / 2.0)),
            material=black, name="top_rail",
        )
        # Frame: bottom rail
        door.visual(
            Box((FACE_THK, DOOR_W, FRAME_BAR)),
            origin=Origin(xyz=(px, dcy, -UPPER_DOOR_H / 2.0 + FRAME_BAR / 2.0)),
            material=black, name="bottom_rail",
        )
        # Frame: hinge-side stile (near y=0 local = hinge line)
        hinge_stile_cy = -sy * FRAME_BAR / 2.0
        door.visual(
            Box((FACE_THK, FRAME_BAR, UPPER_DOOR_H - 2 * FRAME_BAR)),
            origin=Origin(xyz=(px, hinge_stile_cy, 0.0)),
            material=black, name="hinge_stile",
        )
        # Frame: free-edge stile
        free_stile_cy = -sy * (DOOR_W - FRAME_BAR / 2.0)
        door.visual(
            Box((FACE_THK, FRAME_BAR, UPPER_DOOR_H - 2 * FRAME_BAR)),
            origin=Origin(xyz=(px, free_stile_cy, 0.0)),
            material=black, name="free_stile",
        )

        # Glass panel (inset within frame)
        glass_w = DOOR_W - 2 * FRAME_BAR
        glass_h = UPPER_DOOR_H - 2 * FRAME_BAR
        door.visual(
            Box((GLASS_THK, glass_w, glass_h)),
            origin=Origin(xyz=(px, dcy, 0.0)),
            material=glass, name="glass_panel",
        )

        # Hinge barrels (2 per door, vertical cylinders at hinge edge)
        for hi, hz in enumerate((UPPER_DOOR_H * 0.30, -UPPER_DOOR_H * 0.30)):
            door.visual(
                Cylinder(radius=HINGE_R, length=HINGE_H),
                origin=Origin(xyz=(0.0, 0.0, hz)),
                material=silver, name=f"hinge_barrel_{hi}",
            )

        door.inertial = Inertial.from_geometry(
            Box((FACE_THK, DOOR_W, UPPER_DOOR_H)), mass=3.0)

    # ===================================================================
    # LOWER DOORS (solid wood, 2 doors with pull handles)
    # ===================================================================
    for idx, (side, sy) in enumerate((("left", -1), ("right", 1))):
        door = model.part(f"lower_door_{idx}")
        dcy = -sy * DOOR_W / 2.0
        px = FACE_PROUD - FACE_THK / 2.0

        # Solid door panel
        door.visual(
            Box((FACE_THK, DOOR_W, LOWER_DOOR_H)),
            origin=Origin(xyz=(px, dcy, 0.0)),
            material=black, name="door_panel",
        )

        # Hinge barrels (2 per door)
        for hi, hz in enumerate((LOWER_DOOR_H * 0.30, -LOWER_DOOR_H * 0.30)):
            door.visual(
                Cylinder(radius=HINGE_R, length=HINGE_H),
                origin=Origin(xyz=(0.0, 0.0, hz)),
                material=silver, name=f"hinge_barrel_{hi}",
            )

        # Pull handle (D-pull: bar on two stems, near the free edge)
        handle_cy = -sy * (DOOR_W - 0.060)
        handle_front_x = FACE_PROUD  # at the door front face

        # Handle bar (vertical thin bar)
        door.visual(
            Box((HANDLE_BAR_W, HANDLE_BAR_W, HANDLE_BAR_H)),
            origin=Origin(xyz=(handle_front_x + HANDLE_STEM_L + HANDLE_BAR_W / 2.0,
                               handle_cy, 0.0)),
            material=silver, name="handle_bar",
        )
        # Handle stems (2, protruding along +X from door face)
        for si, sz in enumerate((-0.030, 0.030)):
            door.visual(
                Cylinder(radius=HANDLE_STEM_R, length=HANDLE_STEM_L),
                origin=Origin(xyz=(handle_front_x + HANDLE_STEM_L / 2.0,
                                   handle_cy, sz),
                              rpy=(0.0, math.pi / 2.0, 0.0)),
                material=silver, name=f"handle_stem_{si}",
            )

        door.inertial = Inertial.from_geometry(
            Box((FACE_THK, DOOR_W, LOWER_DOOR_H)), mass=4.0)

    # ===================================================================
    # TOP LID (silver-gray panel, hinged at rear edge)
    # ===================================================================
    lid = model.part("top_lid")
    lid.visual(
        Box((LID_DEPTH, W_TOTAL, LID_THK)),
        origin=Origin(xyz=(LID_DEPTH / 2.0, 0.0, LID_THK / 2.0)),
        material=silver_top, name="lid_panel",
    )
    # Lid hinge barrels (2, horizontal along Y at the rear hinge line)
    for hi, hy in enumerate((W_TOTAL * 0.30, -W_TOTAL * 0.30)):
        lid.visual(
            Cylinder(radius=HINGE_R, length=HINGE_H),
            origin=Origin(xyz=(0.0, hy, 0.0),
                          rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=silver, name=f"lid_hinge_{hi}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((LID_DEPTH, W_TOTAL, LID_THK)), mass=3.0)

    # ===================================================================
    # ARTICULATIONS
    # ===================================================================

    # Upper doors: revolute at outer hinge edges
    # Left door: hinge at y=-OPEN_HW, axis (0,0,-1) -> +q opens outward (+X)
    model.articulation(
        "carcass_to_upper_door_0",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=model.get_part("upper_door_0"),
        origin=Origin(xyz=(BD, -OPEN_HW, UPPER_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0,
                                   lower=0.0, upper=math.radians(110)),
    )
    # Right door: hinge at y=+OPEN_HW, axis (0,0,1) -> +q opens outward (+X)
    model.articulation(
        "carcass_to_upper_door_1",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=model.get_part("upper_door_1"),
        origin=Origin(xyz=(BD, OPEN_HW, UPPER_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0,
                                   lower=0.0, upper=math.radians(110)),
    )

    # Lower doors: same hinge arrangement
    model.articulation(
        "carcass_to_lower_door_0",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=model.get_part("lower_door_0"),
        origin=Origin(xyz=(BD, -OPEN_HW, LOWER_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0,
                                   lower=0.0, upper=math.radians(110)),
    )
    model.articulation(
        "carcass_to_lower_door_1",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=model.get_part("lower_door_1"),
        origin=Origin(xyz=(BD, OPEN_HW, LOWER_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0,
                                   lower=0.0, upper=math.radians(110)),
    )

    # Top lid: revolute at rear top edge, axis (0,-1,0) -> +q opens upward
    model.articulation(
        "carcass_to_top_lid",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5,
                                   lower=0.0, upper=math.radians(95)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    door_names = ["upper_door_0", "upper_door_1", "lower_door_0", "lower_door_1"]
    doors = {n: object_model.get_part(n) for n in door_names}
    door_joints = {n: object_model.get_articulation(f"carcass_to_{n}")
                   for n in door_names}
    lid = object_model.get_part("top_lid")
    lid_joint = object_model.get_articulation("carcass_to_top_lid")

    # --- Grounding and overall scale ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("legs_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    ctx.check("width_170", abs(width_y - 1.70) < 0.02,
              details=f"w={width_y:.3f}")
    ctx.check("depth_050", abs(depth_x - 0.50) < 0.04,
              details=f"d={depth_x:.3f}")
    # Height measured across the full assembly (carcass + lid at rest).
    lid_aabb = ctx.part_world_aabb(lid)
    assert lid_aabb is not None
    model_min_z = min(cb[0][2], lid_aabb[0][2])
    model_max_z = max(cb[1][2], lid_aabb[1][2])
    height_z = model_max_z - model_min_z
    ctx.check("height_085", abs(height_z - 0.85) < 0.02,
              details=f"h={height_z:.3f}")

    # --- All four doors are REVOLUTE joints ---
    ctx.check("four_doors", len(door_joints) == 4)
    for n, j in door_joints.items():
        ctx.check(f"{n}_revolute",
                  j.articulation_type == ArticulationType.REVOLUTE)
        # Axis is vertical (Z) for door hinges
        ctx.check(f"{n}_vertical_axis",
                  abs(j.axis[2]) > 0.99 and abs(j.axis[0]) < 0.01
                  and abs(j.axis[1]) < 0.01,
                  details=f"axis={j.axis}")
        # Limits: 0 to ~110 degrees
        ctx.check(f"{n}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and j.motion_limits.upper > math.radians(90),
                  details=f"range=({j.motion_limits.lower:.3f},{j.motion_limits.upper:.3f})")

    # --- Top lid is REVOLUTE, opens upward ---
    ctx.check("lid_revolute",
              lid_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("lid_axis_lateral",
              abs(lid_joint.axis[1]) > 0.99 and abs(lid_joint.axis[0]) < 0.01
              and abs(lid_joint.axis[2]) < 0.01,
              details=f"axis={lid_joint.axis}")
    ctx.check("lid_range",
              abs(lid_joint.motion_limits.lower) < 1e-9
              and lid_joint.motion_limits.upper > math.radians(80),
              details=f"upper={lid_joint.motion_limits.upper:.3f}")

    # --- Upper doors have glass panels ---
    for n in ("upper_door_0", "upper_door_1"):
        gp = doors[n].get_visual("glass_panel")
        ctx.check(f"{n}_has_glass", gp is not None)

    # --- Lower doors have solid panels and pull handles ---
    for n in ("lower_door_0", "lower_door_1"):
        dp = doors[n].get_visual("door_panel")
        hb = doors[n].get_visual("handle_bar")
        ctx.check(f"{n}_solid_panel", dp is not None)
        ctx.check(f"{n}_has_handle", hb is not None)

    # --- Hinge barrels visible on all doors ---
    for n in door_names:
        hb0 = doors[n].get_visual("hinge_barrel_0")
        hb1 = doors[n].get_visual("hinge_barrel_1")
        ctx.check(f"{n}_hinge_barrels", hb0 is not None and hb1 is not None)

    # --- Lid hinge barrels visible ---
    lh0 = lid.get_visual("lid_hinge_0")
    lh1 = lid.get_visual("lid_hinge_1")
    ctx.check("lid_hinge_barrels", lh0 is not None and lh1 is not None)

    # --- Closed pose: doors seated at carcass front ---
    carcass_front = BD
    for n in door_names:
        d = doors[n]
        if n.startswith("upper"):
            panel_elem = "top_rail"  # use frame rail as proxy for front position
        else:
            panel_elem = "door_panel"
        face = ctx.part_element_world_aabb(d, elem=panel_elem)
        assert face is not None
        ctx.check(f"{n}_front_near_carcass",
                  abs(face[1][0] - carcass_front) < 0.025,
                  details=f"face front x={face[1][0]:.4f}")

    # --- Upper/lower door vertical separation (divider shelf between them) ---
    upper_face = ctx.part_element_world_aabb(doors["upper_door_0"], elem="bottom_rail")
    lower_face = ctx.part_element_world_aabb(doors["lower_door_0"], elem="door_panel")
    assert upper_face is not None and lower_face is not None
    ctx.check("upper_above_lower",
              upper_face[0][2] > lower_face[1][2] + 0.005,
              details=f"upper bot={upper_face[0][2]:.3f}, lower top={lower_face[1][2]:.3f}")

    # --- Door opening pose: free edge swings outward (+X) ---
    for n in ("upper_door_0", "lower_door_1"):
        d = doors[n]
        j = door_joints[n]
        rest_pos = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            open_pos = ctx.part_world_position(d)
        assert rest_pos is not None and open_pos is not None
        # The door part origin doesn't move (it's at the hinge), but the geometry
        # rotates. Check that the part element (free edge) moved outward.
        rest_face = ctx.part_element_world_aabb(d, elem="hinge_barrel_0")
        with ctx.pose({j: j.motion_limits.upper}):
            open_face = ctx.part_element_world_aabb(d, elem="hinge_barrel_0")
        # Hinge barrels stay near the hinge; check the panel moves instead
        if n.startswith("upper"):
            elem = "free_stile"
        else:
            elem = "handle_bar"
        rest_elem = ctx.part_element_world_aabb(d, elem=elem)
        with ctx.pose({j: j.motion_limits.upper}):
            open_elem = ctx.part_element_world_aabb(d, elem=elem)
        assert rest_elem is not None and open_elem is not None
        ctx.check(f"{n}_opens_outward",
                  open_elem[1][0] > rest_elem[1][0] + 0.05,
                  details=f"rest x={rest_elem[1][0]:.3f}, open x={open_elem[1][0]:.3f}")

    # --- Lid opening pose: front edge rises ---
    lid_rest = ctx.part_element_world_aabb(lid, elem="lid_panel")
    assert lid_rest is not None
    rest_front_z = lid_rest[1][2]  # top of lid at rest
    with ctx.pose({lid_joint: lid_joint.motion_limits.upper}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_panel")
    assert lid_open is not None
    open_max_z = lid_open[1][2]
    ctx.check("lid_opens_upward",
              open_max_z > rest_front_z + 0.10,
              details=f"rest top z={rest_front_z:.3f}, open max z={open_max_z:.3f}")

    # --- Allowances: door hinge barrels overlap carcass stiles (captured barrel) ---
    for n in door_names:
        stile = "front_side_stile_1" if n.endswith("_0") else "front_side_stile_0"
        ctx.allow_overlap(
            "carcass", n,
            elem_a=stile, elem_b="hinge_barrel_0",
            reason=f"Hinge barrel captured in carcass stile at the {n} hinge line.",
        )
        ctx.allow_overlap(
            "carcass", n,
            elem_a=stile, elem_b="hinge_barrel_1",
            reason=f"Hinge barrel captured in carcass stile at the {n} hinge line.",
        )
        # Door panel edge seating overlaps the center stile
        ctx.allow_overlap(
            "carcass", n,
            elem_a="front_center_stile", elem_b=("door_panel" if n.startswith("lower") else "top_rail"),
            reason=f"Door panel seats against the center stile at the free edge.",
        )
        # Upper door free stile seats against the center stile when closed
        if n.startswith("upper"):
            ctx.allow_overlap(
                "carcass", n,
                elem_a="front_center_stile", elem_b="free_stile",
                reason=f"Upper door free stile meets the center stile at the closed position.",
            )

    # Lid hinge barrels overlap carcass back panel and top stretcher
    ctx.allow_overlap(
        "carcass", "top_lid",
        elem_a="back_panel", elem_b="lid_hinge_0",
        reason="Lid hinge barrel captured at the rear hinge line.",
    )
    ctx.allow_overlap(
        "carcass", "top_lid",
        elem_a="back_panel", elem_b="lid_hinge_1",
        reason="Lid hinge barrel captured at the rear hinge line.",
    )
    ctx.allow_overlap(
        "carcass", "top_lid",
        elem_a="top_stretcher", elem_b="lid_hinge_0",
        reason="Lid hinge barrel sits in the top stretcher at the rear hinge pivot.",
    )
    ctx.allow_overlap(
        "carcass", "top_lid",
        elem_a="top_stretcher", elem_b="lid_hinge_1",
        reason="Lid hinge barrel sits in the top stretcher at the rear hinge pivot.",
    )

    # Upper door bottom rails pass the center stile when closed (normal cabinet seating)
    for n in ("upper_door_0", "upper_door_1"):
        ctx.allow_overlap(
            "carcass", n,
            elem_a="front_center_stile", elem_b="bottom_rail",
            reason=f"Upper door bottom rail passes the center stile at the closed position.",
        )

    # --- Proof checks paired with allowances ---
    # Doors remain within carcass width when closed
    for n in door_names:
        ctx.expect_within(doors[n], carcass, axes="y", margin=0.005,
                          name=f"{n}_within_carcass_width")

    # Lid sits on top of carcass when closed
    ctx.expect_gap(lid, carcass, axis="z", min_gap=-0.002, max_gap=0.005,
                   positive_elem="lid_panel", negative_elem="top_stretcher",
                   name="lid_seats_on_carcass_top")

    return ctx.report()


object_model = build_object_model()
