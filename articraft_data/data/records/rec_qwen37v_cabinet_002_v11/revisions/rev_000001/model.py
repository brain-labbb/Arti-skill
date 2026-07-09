from __future__ import annotations

# Tall two-door cabinet variant (~0.80 m W x 1.60 m H x 0.45 m D).
#
# World layout: front faces +X (back of the body at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four caster
# blocks. Matte black wood carcass and door fronts; a thin smooth silver-gray
# lid/top slab overhangs the body on all sides. The front holds two tall doors,
# each with visible brass hinge barrels on the door side and a polished silver
# ball knob near the free edge. A raised plinth base sits above the caster
# blocks. The top lid hinges upward on a rear revolute joint.

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
W_TOTAL = 0.80           # overall width including top overhang (Y)
D_TOTAL = 0.45           # overall depth including top overhang (X)
H_TOTAL = 1.60           # overall height (Z)
OVERHANG = 0.020         # top slab overhang on all sides
LID_THK = 0.022          # lid/top slab thickness

BW = W_TOTAL - 2 * OVERHANG      # body width 0.76
BD = D_TOTAL - 2 * OVERHANG      # body depth 0.41
WALL = 0.018                     # carcass panel thickness
INNER_W = BW - 2 * WALL          # 0.724
INNER_D = BD - WALL              # 0.392 (open front, closed back)

CASTER_H = 0.025                 # caster block height
PLINTH_H = 0.080                 # plinth base height
BODY_BOT = CASTER_H + PLINTH_H   # 0.105
BODY_TOP = H_TOTAL - LID_THK     # 1.578
BH = BODY_TOP - BODY_BOT         # 1.473

# Door dimensions
DOOR_REVEAL = 0.004              # gap around doors
DOOR_H = BH - 2 * DOOR_REVEAL   # door height (~1.465)
DOOR_W = (INNER_W - DOOR_REVEAL) / 2.0  # each door width (~0.360)
DOOR_THK = 0.020                 # door panel thickness

# Hinge barrel dimensions
BARREL_R = 0.008                 # barrel radius
BARREL_H = 0.045                 # barrel height
N_BARRELS = 3                    # barrels per door

# Knob dimensions
KNOB_R = 0.012
STEM_R = 0.005
STEM_L = 0.012

# Plinth
PLINTH_INSET = 0.015             # plinth inset from body edges
CASTER_SQ = 0.040                # caster block size
CASTER_INSET = 0.030             # caster inset from body corners

# Interior shelf
SHELF_THK = 0.016


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_two_door_cabinet")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    hinge_mat = model.material("hinge_brass", rgba=(0.65, 0.55, 0.30, 1.0))
    interior = model.material("interior_white", rgba=(0.85, 0.84, 0.82, 1.0))

    # ===================================================================
    # ROOT: cabinet carcass (hollow shell + plinth + casters + barrels)
    # ===================================================================
    carcass = model.part("cabinet")

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
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + WALL / 2.0)),
        material=black,
        name="bottom_board",
    )

    # Top stretcher board (just under the lid)
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - WALL / 2.0)),
        material=black,
        name="top_stretcher",
    )

    # Front frame rails (thin strips above and below the door opening)
    rail_h = DOOR_REVEAL + 0.008
    carcass.visual(
        Box((WALL, INNER_W, rail_h)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           BODY_BOT + rail_h / 2.0)),
        material=black,
        name="front_bottom_rail",
    )
    carcass.visual(
        Box((WALL, INNER_W, rail_h)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           BODY_TOP - rail_h / 2.0)),
        material=black,
        name="front_top_rail",
    )

    # Center stile (vertical divider behind the door meeting point)
    carcass.visual(
        Box((WALL, 0.028, DOOR_H + 0.010)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=black,
        name="center_stile",
    )

    # Interior back wall (light colored)
    carcass.visual(
        Box((0.003, INNER_W - 0.010, BH - 0.050)),
        origin=Origin(xyz=(WALL + 0.002, 0.0, BODY_BOT + BH / 2.0)),
        material=interior,
        name="interior_back",
    )

    # Interior shelf (middle)
    shelf_z = BODY_BOT + BH * 0.45
    carcass.visual(
        Box((INNER_D - 0.010, INNER_W - 0.010, SHELF_THK)),
        origin=Origin(xyz=(WALL + (INNER_D - 0.010) / 2.0, 0.0, shelf_z)),
        material=interior,
        name="interior_shelf",
    )

    # Plinth base (raised, slightly inset from body edges)
    plinth_w = BW - 2 * PLINTH_INSET
    plinth_d = BD - PLINTH_INSET
    carcass.visual(
        Box((plinth_d, plinth_w, PLINTH_H)),
        origin=Origin(xyz=(PLINTH_INSET + plinth_d / 2.0, 0.0,
                           CASTER_H + PLINTH_H / 2.0)),
        material=black_deep,
        name="plinth_base",
    )

    # Four caster blocks at the corners
    for tag, cx, cy in (("fl", CASTER_INSET, BW / 2.0 - CASTER_INSET),
                        ("fr", CASTER_INSET, -(BW / 2.0 - CASTER_INSET)),
                        ("rl", BD - CASTER_INSET, BW / 2.0 - CASTER_INSET),
                        ("rr", BD - CASTER_INSET, -(BW / 2.0 - CASTER_INSET))):
        carcass.visual(
            Box((CASTER_SQ, CASTER_SQ, CASTER_H)),
            origin=Origin(xyz=(cx, cy, CASTER_H / 2.0)),
            material=black_deep,
            name=f"caster_{tag}",
        )

    # Visible hinge barrels on the door side (brass cylinders, 3 per door)
    barrel_zs = [BODY_BOT + BH * f for f in (0.15, 0.50, 0.85)]
    for door_tag, hinge_y in (("left", -INNER_W / 2.0),
                               ("right", INNER_W / 2.0)):
        for i, bz in enumerate(barrel_zs):
            carcass.visual(
                Cylinder(radius=BARREL_R, length=BARREL_H),
                origin=Origin(xyz=(BD, hinge_y, bz)),
                material=hinge_mat,
                name=f"hinge_barrel_{door_tag}_{i}",
            )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=45.0)

    # ===================================================================
    # LEFT DOOR: hinges on the left side, opens outward
    # ===================================================================
    left_door = model.part("left_door")

    # Door panel: extends along +Y from the hinge line
    left_door.visual(
        Box((DOOR_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(DOOR_THK / 2.0, DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )

    # Knob near the free edge (right side of left door), slightly below center
    knob_y_l = DOOR_W - 0.060
    knob_z_l = -0.100
    stem_cx_l = DOOR_THK + (STEM_L - 0.004) / 2.0
    left_door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=(stem_cx_l, knob_y_l, knob_z_l),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    left_door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(DOOR_THK + STEM_L + KNOB_R, knob_y_l, knob_z_l)),
        material=silver,
        name="knob_ball",
    )

    left_door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, DOOR_H)), mass=3.0)

    # ===================================================================
    # RIGHT DOOR: hinges on the right side, opens outward
    # ===================================================================
    right_door = model.part("right_door")

    # Door panel: extends along -Y from the hinge line
    right_door.visual(
        Box((DOOR_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(DOOR_THK / 2.0, -DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )

    # Knob near the free edge (left side of right door)
    knob_y_r = -(DOOR_W - 0.060)
    stem_cx_r = DOOR_THK + (STEM_L - 0.004) / 2.0
    right_door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=(stem_cx_r, knob_y_r, -0.100),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    right_door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(DOOR_THK + STEM_L + KNOB_R, knob_y_r, -0.100)),
        material=silver,
        name="knob_ball",
    )

    right_door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, DOOR_H)), mass=3.0)

    # ===================================================================
    # LID: top panel, hinges upward from the rear edge
    # ===================================================================
    lid = model.part("lid")

    # Lid panel: extends along +X (forward) from the rear hinge line
    lid.visual(
        Box((D_TOTAL, W_TOTAL, LID_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, LID_THK / 2.0)),
        material=silver_top,
        name="lid_panel",
    )

    lid.inertial = Inertial.from_geometry(
        Box((D_TOTAL, W_TOTAL, LID_THK)), mass=4.0)

    # ===================================================================
    # ARTICULATIONS
    # ===================================================================
    center_z = BODY_BOT + BH / 2.0

    # Left door: hinge at left edge of opening, positive q opens outward (+X).
    # Door extends along +Y from hinge; axis=-Z makes +Y rotate toward +X.
    model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(BD, -INNER_W / 2.0, center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                   lower=0.0, upper=1.5),
    )

    # Right door: hinge at right edge of opening, positive q opens outward (+X).
    # Door extends along -Y from hinge; axis=+Z makes -Y rotate toward +X.
    model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(BD, INNER_W / 2.0, center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                   lower=0.0, upper=1.5),
    )

    # Lid: hinge at rear top edge, positive q lifts the front edge upward (+Z).
    # Lid extends along +X from the hinge; axis=-Y makes +X rotate toward +Z.
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0,
                                   lower=0.0, upper=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    lid = object_model.get_part("lid")

    left_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_door")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")

    # --- Overall dimensions (~0.80 x 0.45 x 1.60 m) ---
    cb = ctx.part_world_aabb(cabinet)
    assert cb is not None
    depth_x = cb[1][0] - cb[0][0]
    ctx.check("depth_045", abs(depth_x - 0.45) < 0.04,
              details=f"d={depth_x:.3f}")

    # Width and height include the lid (widest/tallest part)
    lid_panel_closed = ctx.part_element_world_aabb(lid, elem="lid_panel")
    assert lid_panel_closed is not None
    width_y = lid_panel_closed[1][1] - lid_panel_closed[0][1]
    height_z = lid_panel_closed[1][2] - cb[0][2]
    ctx.check("width_080", abs(width_y - 0.80) < 0.02,
              details=f"w={width_y:.3f}")
    ctx.check("height_160", abs(height_z - 1.60) < 0.02,
              details=f"h={height_z:.3f}")

    # --- Grounding: casters on floor ---
    ctx.check("casters_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")

    # --- Plinth base exists and is above casters ---
    plinth = ctx.part_element_world_aabb(cabinet, elem="plinth_base")
    assert plinth is not None
    ctx.check("plinth_above_casters",
              plinth[0][2] > CASTER_H - 0.003,
              details=f"plinth bottom z={plinth[0][2]:.4f}")
    ctx.check("plinth_is_raised",
              (plinth[1][2] - plinth[0][2]) > PLINTH_H - 0.005,
              details=f"plinth height={plinth[1][2] - plinth[0][2]:.4f}")

    # --- Caster blocks exist at all four corners ---
    for tag in ("fl", "fr", "rl", "rr"):
        caster = ctx.part_element_world_aabb(cabinet, elem=f"caster_{tag}")
        assert caster is not None
        ctx.check(f"caster_{tag}_on_floor",
                  abs(caster[0][2]) < 0.003,
                  details=f"caster {tag} bottom z={caster[0][2]:.4f}")

    # --- Hinge barrels visible on door side ---
    for door_tag in ("left", "right"):
        for i in range(N_BARRELS):
            barrel = ctx.part_element_world_aabb(
                cabinet, elem=f"hinge_barrel_{door_tag}_{i}")
            assert barrel is not None
            barrel_h = barrel[1][2] - barrel[0][2]
            ctx.check(f"hinge_barrel_{door_tag}_{i}_visible",
                      barrel_h > BARREL_H - 0.005,
                      details=f"barrel h={barrel_h:.4f}")

    # --- Allow hinge barrel / door panel overlap (mechanically intentional) ---
    for door_tag, door_part in (("left", left_door), ("right", right_door)):
        for i in range(N_BARRELS):
            ctx.allow_overlap(
                cabinet, door_part,
                elem_a=f"hinge_barrel_{door_tag}_{i}",
                elem_b="door_panel",
                reason=("Hinge barrel is intentionally embedded at the "
                        "door-carcass junction to represent the hinge pivot."),
            )

    # --- Doors are revolute with correct limits ---
    ctx.check("left_door_revolute",
              left_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("right_door_revolute",
              right_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("left_door_limits",
              abs(left_hinge.motion_limits.lower) < 1e-9
              and abs(left_hinge.motion_limits.upper - 1.5) < 0.01,
              details=f"range=({left_hinge.motion_limits.lower},"
                      f"{left_hinge.motion_limits.upper})")
    ctx.check("right_door_limits",
              abs(right_hinge.motion_limits.lower) < 1e-9
              and abs(right_hinge.motion_limits.upper - 1.5) < 0.01,
              details=f"range=({right_hinge.motion_limits.lower},"
                      f"{right_hinge.motion_limits.upper})")

    # --- Lid is revolute with correct limits ---
    ctx.check("lid_revolute",
              lid_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("lid_limits",
              abs(lid_hinge.motion_limits.lower) < 1e-9
              and abs(lid_hinge.motion_limits.upper - 1.4) < 0.01,
              details=f"range=({lid_hinge.motion_limits.lower},"
                      f"{lid_hinge.motion_limits.upper})")

    # --- Closed pose: doors at front, lid on top ---
    left_panel = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_panel = ctx.part_element_world_aabb(right_door, elem="door_panel")
    lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    assert left_panel is not None and right_panel is not None
    assert lid_panel is not None

    # Doors cover the front opening
    ctx.check("left_door_at_front",
              abs(left_panel[1][0] - (BD + DOOR_THK)) < 0.005,
              details=f"left door front x={left_panel[1][0]:.4f}")
    ctx.check("right_door_at_front",
              abs(right_panel[1][0] - (BD + DOOR_THK)) < 0.005,
              details=f"right door front x={right_panel[1][0]:.4f}")

    # Doors are tall (span most of the body height)
    left_door_h = left_panel[1][2] - left_panel[0][2]
    ctx.check("doors_are_tall",
              left_door_h > BH * 0.95,
              details=f"door h={left_door_h:.3f}, body h={BH:.3f}")

    # Lid on top of the carcass
    ctx.check("lid_on_top",
              abs(lid_panel[0][2] - BODY_TOP) < 0.005,
              details=f"lid bottom z={lid_panel[0][2]:.4f}")

    # Lid overhangs the carcass sides
    side = ctx.part_element_world_aabb(cabinet, elem="side_panel_0")
    assert side is not None
    ctx.check("lid_overhangs_sides",
              lid_panel[1][1] > side[1][1] + 0.015
              and lid_panel[0][1] < -side[1][1] - 0.015,
              details=f"lid y=({lid_panel[0][1]:.3f},{lid_panel[1][1]:.3f})")

    # Lid is a thin slab
    ctx.check("lid_is_thin_slab",
              abs((lid_panel[1][2] - lid_panel[0][2]) - LID_THK) < 0.002)

    # --- Knobs exist on each door ---
    for door_name in ("left_door", "right_door"):
        dp = object_model.get_part(door_name)
        ball = ctx.part_element_world_aabb(dp, elem="knob_ball")
        assert ball is not None
        panel = ctx.part_element_world_aabb(dp, elem="door_panel")
        assert panel is not None
        ctx.check(f"{door_name}_knob_proud",
                  ball[0][0] > panel[1][0] + 0.002,
                  details=f"ball min x={ball[0][0]:.4f}, "
                          f"panel max x={panel[1][0]:.4f}")

    # --- Open pose: doors swing outward ---
    rest_left_x = left_panel[1][0]  # front edge of left door panel
    with ctx.pose({left_hinge: left_hinge.motion_limits.upper}):
        open_left_aabb = ctx.part_element_world_aabb(left_door, elem="door_panel")
        assert open_left_aabb is not None
    ctx.check("left_door_opens_outward",
              open_left_aabb[1][0] > rest_left_x + 0.05,
              details=f"rest front x={rest_left_x:.4f}, "
                      f"open front x={open_left_aabb[1][0]:.4f}")

    rest_right_x = right_panel[1][0]
    with ctx.pose({right_hinge: right_hinge.motion_limits.upper}):
        open_right_aabb = ctx.part_element_world_aabb(right_door, elem="door_panel")
        assert open_right_aabb is not None
    ctx.check("right_door_opens_outward",
              open_right_aabb[1][0] > rest_right_x + 0.05,
              details=f"rest front x={rest_right_x:.4f}, "
                      f"open front x={open_right_aabb[1][0]:.4f}")

    # --- Open pose: lid lifts upward ---
    rest_lid_z = lid_panel[1][2]  # top of lid panel
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        assert open_lid_aabb is not None
    ctx.check("lid_opens_upward",
              open_lid_aabb[1][2] > rest_lid_z + 0.05,
              details=f"rest top z={rest_lid_z:.4f}, "
                      f"open top z={open_lid_aabb[1][2]:.4f}")

    # --- Non-fixed joints exist (at least one) ---
    all_joints = [left_hinge, right_hinge, lid_hinge]
    non_fixed = [j for j in all_joints
                 if j.articulation_type != ArticulationType.FIXED]
    ctx.check("has_non_fixed_joints", len(non_fixed) >= 1,
              details=f"non-fixed count={len(non_fixed)}")

    # --- Door independence: opening one door leaves the other closed ---
    with ctx.pose({left_hinge: 1.0}):
        right_panel_open = ctx.part_element_world_aabb(
            right_door, elem="door_panel")
        assert right_panel_open is not None
        ctx.check("doors_independent",
                  abs(right_panel_open[1][0] - (BD + DOOR_THK)) < 0.005,
                  details=f"right door front x={right_panel_open[1][0]:.4f}")

    return ctx.report()


object_model = build_object_model()
