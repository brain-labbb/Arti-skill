from __future__ import annotations

# Wide black wooden storage cabinet variant (~1.70 m W x 0.85 m H x 0.50 m D).
# Variant 29: fork of the double dresser into a door-and-lid cabinet sibling.
#
# World layout: front faces +X (back of the body at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on four caster
# blocks ~0.06 m tall. Matte black wood carcass and door fronts; a thin smooth
# silver-gray lift-up lid on top. The front holds two side-hung doors on
# revolute hinge joints with visible barrel cylinders; the top lid lifts up
# on a rear hinge to reveal a shallow storage compartment.

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
LID_THK = 0.022          # silver lid slab thickness

BW = W_TOTAL - 2 * OVERHANG      # body width 1.66
BD = D_TOTAL - 2 * OVERHANG      # body depth 0.46
WALL = 0.018                     # carcass panel thickness
INNER_W = BW - 2 * WALL          # inner clear width

FEET_H = 0.060                   # caster block height
BODY_BOT = FEET_H                # body bottom z
BODY_TOP = H_TOTAL - LID_THK     # body top z (0.790) where lid sits
BH = BODY_TOP - BODY_BOT         # body height

# Shallow storage compartment under the lid.
SHALLOW_H = 0.065                # height of the shallow storage well
SHELF_Z = BODY_TOP - SHALLOW_H   # top of internal shelf board
FRAME_FRONT_H = SHALLOW_H        # front frame above door opening

# Main door opening zone.
RAIL_BOT_H = 0.035               # bottom rail below door opening
OPEN_BOT = BODY_BOT + RAIL_BOT_H
OPEN_TOP = SHELF_Z               # door opening goes up to the shelf
OPEN_H = OPEN_TOP - OPEN_BOT     # door opening height

# Door geometry.
DOOR_THK = 0.020
DOOR_GAP = 0.006                 # center gap between doors
DOOR_REVEAL = 0.004              # reveal around door edges
DOOR_W = (INNER_W - DOOR_GAP - 2 * DOOR_REVEAL) / 2.0
DOOR_H = OPEN_H - 2 * DOOR_REVEAL
DOOR_CZ = (OPEN_BOT + OPEN_TOP) / 2.0  # door center z

# Hinge barrel dimensions.
BARREL_R = 0.007
BARREL_LEN = 0.045
N_BARRELS = 3

# Caster/feet blocks.
FOOT_W = 0.065
FOOT_D = 0.065

# Knobs: polished silver ball on a short stem (same style as parent).
KNOB_R = 0.014
STEM_R = 0.005
STEM_L = 0.014


def _build_door(model, name, hinge_side, black, silver):
    """Build a door part. hinge_side=-1 for left door (hinge at -Y),
    hinge_side=+1 for right door (hinge at +Y).
    Local frame: origin at hinge axis. Door panel extends toward
    local +Y (left) or -Y (right) and toward +X (outward face)."""
    door = model.part(name)

    # Direction from hinge toward free edge in local Y.
    dy = -hinge_side  # left door (hinge_side=-1): dy=+1, extends +Y

    # Door panel: thickness along X, width along Y, height along Z.
    door.visual(
        Box((DOOR_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(DOOR_THK / 2.0, dy * DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )

    # Knob on the free edge of the door.
    knob_y = dy * (DOOR_W - 0.060)
    # Stem: cylinder along local Z, rotated to point along +X.
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0 + DOOR_THK, knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004 + DOOR_THK, knob_y, 0.0)),
        material=silver,
        name="knob_ball",
    )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, DOOR_H)), mass=3.5)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_cabinet_doors_lid")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_lid", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    hinge_mat = model.material("hinge_brass", rgba=(0.55, 0.50, 0.35, 1.0))
    foot_mat = model.material("caster_black", rgba=(0.10, 0.10, 0.11, 1.0))

    # ===================================================================
    # ROOT: carcass (hollow shell + caster blocks + hinge barrels + lid frame)
    # ===================================================================
    carcass = model.part("carcass")

    # Side panels (full height from BODY_BOT to BODY_TOP).
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

    # Front bottom rail (below door opening).
    carcass.visual(
        Box((WALL, INNER_W, RAIL_BOT_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           BODY_BOT + RAIL_BOT_H / 2.0)),
        material=black,
        name="front_bottom_rail",
    )

    # Front top rail / frame above door opening (shallow storage front wall).
    carcass.visual(
        Box((WALL, INNER_W, FRAME_FRONT_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           OPEN_TOP + FRAME_FRONT_H / 2.0)),
        material=black,
        name="front_top_frame",
    )

    # Internal shelf board separating main compartment from shallow storage.
    carcass.visual(
        Box((BD - 2 * WALL, INNER_W, 0.018)),
        origin=Origin(xyz=(WALL + (BD - 2 * WALL) / 2.0, 0.0,
                           SHELF_Z + 0.009)),
        material=black_deep,
        name="shelf_board",
    )

    # Center vertical divider (separates left and right door compartments).
    divider_h = OPEN_H
    carcass.visual(
        Box((BD - 2 * WALL, WALL, divider_h)),
        origin=Origin(xyz=(WALL + (BD - 2 * WALL) / 2.0, 0.0,
                           OPEN_BOT + divider_h / 2.0)),
        material=black_deep,
        name="center_divider",
    )

    # Top rim strips (thin boards forming the rim the lid sits on).
    # Front rim already covered by front_top_frame extending to BODY_TOP.
    # Back rim:
    carcass.visual(
        Box((WALL, INNER_W, SHALLOW_H)),
        origin=Origin(xyz=(WALL + WALL / 2.0, 0.0,
                           SHELF_Z + SHALLOW_H / 2.0)),
        material=black_deep,
        name="back_rim",
    )
    # Side rims (connecting front to back at the top).
    for tag, s in (("0", 1), ("1", -1)):
        rim_len = BD - 3 * WALL
        carcass.visual(
            Box((rim_len, WALL, SHALLOW_H)),
            origin=Origin(xyz=(2 * WALL + rim_len / 2.0,
                               s * (INNER_W / 2.0 - WALL / 2.0),
                               SHELF_Z + SHALLOW_H / 2.0)),
            material=black_deep,
            name=f"side_rim_{tag}",
        )

    # Crown edge strips providing slight overhang beyond the side panels.
    # These give the carcass the full overall width including overhang.
    crown_thk = 0.012
    crown_h = 0.020
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD + 2 * OVERHANG, crown_thk, crown_h)),
            origin=Origin(xyz=(BD / 2.0,
                               s * (BW / 2.0 + crown_thk / 2.0),
                               BODY_TOP - crown_h / 2.0)),
            material=black,
            name=f"crown_edge_{tag}",
        )
    # Front and back crown edge strips.
    for tag, xf in (("front", BD + OVERHANG), ("rear", -OVERHANG)):
        carcass.visual(
            Box((crown_thk, W_TOTAL, crown_h)),
            origin=Origin(xyz=(xf, 0.0, BODY_TOP - crown_h / 2.0)),
            material=black,
            name=f"crown_edge_{tag}",
        )

    # Visible hinge barrels for left door (on carcass, at the left hinge edge).
    barrel_zs = [OPEN_BOT + DOOR_H * f for f in (0.18, 0.50, 0.82)]
    for i, bz in enumerate(barrel_zs):
        carcass.visual(
            Cylinder(radius=BARREL_R, length=BARREL_LEN),
            origin=Origin(xyz=(BD - WALL, -(INNER_W / 2.0), bz)),
            material=hinge_mat,
            name=f"hinge_barrel_left_{i}",
        )

    # Visible hinge barrels for right door (on carcass, at the right hinge edge).
    for i, bz in enumerate(barrel_zs):
        carcass.visual(
            Cylinder(radius=BARREL_R, length=BARREL_LEN),
            origin=Origin(xyz=(BD - WALL, (INNER_W / 2.0), bz)),
            material=hinge_mat,
            name=f"hinge_barrel_right_{i}",
        )

    # Lid hinge barrels (on carcass rear top edge, axis along Y).
    lid_barrel_ys = [-(INNER_W * 0.3), 0.0, (INNER_W * 0.3)]
    for i, by in enumerate(lid_barrel_ys):
        carcass.visual(
            Cylinder(radius=BARREL_R, length=BARREL_LEN),
            # Rotate cylinder to lie along Y axis.
            origin=Origin(xyz=(WALL + WALL / 2.0, by, BODY_TOP),
                          rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_mat,
            name=f"lid_hinge_barrel_{i}",
        )

    # Four caster/feet blocks at corners.
    foot_positions = [
        ("front_left", BD - FOOT_D / 2.0 - 0.020, BW / 2.0 - FOOT_W / 2.0 - 0.020),
        ("front_right", BD - FOOT_D / 2.0 - 0.020, -(BW / 2.0 - FOOT_W / 2.0 - 0.020)),
        ("rear_left", FOOT_D / 2.0 + 0.020, BW / 2.0 - FOOT_W / 2.0 - 0.020),
        ("rear_right", FOOT_D / 2.0 + 0.020, -(BW / 2.0 - FOOT_W / 2.0 - 0.020)),
    ]
    for ft_name, fx, fy in foot_positions:
        carcass.visual(
            Box((FOOT_D, FOOT_W, FEET_H + 0.004)),
            origin=Origin(xyz=(fx, fy, (FEET_H + 0.004) / 2.0)),
            material=foot_mat,
            name=f"caster_{ft_name}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=55.0)

    # ===================================================================
    # DOORS: two revolute-hinged doors
    # ===================================================================
    left_door = _build_door(model, "left_door", hinge_side=-1, black=black, silver=silver)
    right_door = _build_door(model, "right_door", hinge_side=1, black=black, silver=silver)

    # Left door articulation: hinge at left inner edge, axis (0,0,-1).
    # Positive q opens the free edge (+Y local) toward +X (outward).
    model.articulation(
        "carcass_to_left_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(BD, -(INNER_W / 2.0), DOOR_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5,
                                   lower=0.0, upper=1.50),
    )

    # Right door articulation: hinge at right inner edge, axis (0,0,1).
    # Positive q opens the free edge (-Y local) toward +X (outward).
    model.articulation(
        "carcass_to_right_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(BD, (INNER_W / 2.0), DOOR_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5,
                                   lower=0.0, upper=1.50),
    )

    # ===================================================================
    # LID: lift-up top lid on rear hinge
    # ===================================================================
    lid = model.part("top_lid")
    lid_w = INNER_W - 2 * DOOR_REVEAL
    lid_d = BD - 2 * WALL  # from back inner to front inner
    lid.visual(
        Box((lid_d, lid_w, LID_THK)),
        origin=Origin(xyz=(lid_d / 2.0, 0.0, LID_THK / 2.0)),
        material=silver_top,
        name="lid_panel",
    )
    # Thin lip on the front edge of the lid (grip edge).
    lid.visual(
        Box((0.012, lid_w, LID_THK + 0.006)),
        origin=Origin(xyz=(lid_d - 0.006, 0.0, (LID_THK + 0.006) / 2.0)),
        material=silver,
        name="lid_front_lip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_d, lid_w, LID_THK)), mass=2.5)

    # Lid articulation: hinge at rear top edge, axis (0,-1,0).
    # Positive q lifts the front edge upward.
    model.articulation(
        "carcass_to_lid",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lid,
        origin=Origin(xyz=(2 * WALL, 0.0, BODY_TOP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0,
                                   lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    lid = object_model.get_part("top_lid")

    left_joint = object_model.get_articulation("carcass_to_left_door")
    right_joint = object_model.get_articulation("carcass_to_right_door")
    lid_joint = object_model.get_articulation("carcass_to_lid")

    # --- Overall scale ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("feet_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_170", abs(width_y - 1.70) < 0.03, details=f"w={width_y:.3f}")
    ctx.check("depth_050", abs(depth_x - 0.50) < 0.04, details=f"d={depth_x:.3f}")
    ctx.check("height_085", abs(height_z - 0.85) < 0.02,
              details=f"h={height_z:.3f}")

    # --- Caster blocks at the base ---
    caster_names = ["caster_front_left", "caster_front_right",
                   "caster_rear_left", "caster_rear_right"]
    for cn in caster_names:
        ca = ctx.part_element_world_aabb(carcass, elem=cn)
        assert ca is not None
        ctx.check(f"{cn}_at_floor", ca[0][2] < 0.003,
                  details=f"{cn} min z={ca[0][2]:.4f}")

    # --- Visible hinge barrels on door sides ---
    for side in ("left", "right"):
        for i in range(N_BARRELS):
            bname = f"hinge_barrel_{side}_{i}"
            ba = ctx.part_element_world_aabb(carcass, elem=bname)
            assert ba is not None, f"Missing {bname}"
            # Barrels should be near the front face and at door height.
            ctx.check(f"{bname}_near_front",
                      ba[1][0] > BD - 0.04 and ba[0][0] < BD + 0.01,
                      details=f"{bname} x=({ba[0][0]:.3f},{ba[1][0]:.3f})")

    # --- Doors are revolute joints ---
    ctx.check("left_door_revolute",
              left_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("right_door_revolute",
              right_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("left_door_axis_vertical",
              abs(left_joint.axis[2]) > 0.99,
              details=f"axis={left_joint.axis}")
    ctx.check("right_door_axis_vertical",
              abs(right_joint.axis[2]) > 0.99,
              details=f"axis={right_joint.axis}")

    # --- Doors closed: panels cover the front opening ---
    left_face = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_face = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert left_face is not None and right_face is not None

    # Doors should span the opening height.
    ctx.check("left_door_covers_opening",
              left_face[0][2] < OPEN_BOT + 0.01 and left_face[1][2] > OPEN_TOP - 0.01,
              details=f"left door z=({left_face[0][2]:.3f},{left_face[1][2]:.3f})")
    ctx.check("right_door_covers_opening",
              right_face[0][2] < OPEN_BOT + 0.01 and right_face[1][2] > OPEN_TOP - 0.01,
              details=f"right door z=({right_face[0][2]:.3f},{right_face[1][2]:.3f})")

    # Doors should be near the carcass front face when closed.
    ctx.check("left_door_at_front",
              abs(left_face[1][0] - BD - DOOR_THK) < 0.005,
              details=f"left door front x={left_face[1][0]:.4f}")

    # --- Doors open: swing outward ---
    left_rest = ctx.part_world_position(left_door)
    with ctx.pose({left_joint: left_joint.motion_limits.upper}):
        left_open = ctx.part_world_position(left_door)
        left_open_face = ctx.part_element_world_aabb(left_door, elem="door_panel")
    assert left_rest is not None and left_open is not None and left_open_face is not None
    ctx.check("left_door_opens_outward",
              left_open_face[1][0] > BD + 0.10,
              details=f"open left door max x={left_open_face[1][0]:.3f}")

    right_rest = ctx.part_world_position(right_door)
    with ctx.pose({right_joint: right_joint.motion_limits.upper}):
        right_open = ctx.part_world_position(right_door)
        right_open_face = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert right_rest is not None and right_open is not None and right_open_face is not None
    ctx.check("right_door_opens_outward",
              right_open_face[1][0] > BD + 0.10,
              details=f"open right door max x={right_open_face[1][0]:.3f}")

    # --- Lid: revolute joint at rear, lifts upward ---
    ctx.check("lid_revolute",
              lid_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("lid_axis_lateral",
              abs(lid_joint.axis[1]) > 0.99,
              details=f"axis={lid_joint.axis}")

    # Lid closed: sits on top of the carcass.
    lid_closed = ctx.part_element_world_aabb(lid, elem="lid_panel")
    assert lid_closed is not None
    ctx.check("lid_at_top",
              abs(lid_closed[0][2] - BODY_TOP) < 0.005,
              details=f"lid bottom z={lid_closed[0][2]:.4f}")

    # Lid open: front edge rises.
    lid_rest_pos = ctx.part_world_position(lid)
    with ctx.pose({lid_joint: lid_joint.motion_limits.upper}):
        lid_open_pos = ctx.part_world_position(lid)
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    assert lid_rest_pos is not None and lid_open_pos is not None
    assert lid_open_aabb is not None
    ctx.check("lid_opens_upward",
              lid_open_aabb[1][2] > BODY_TOP + 0.15,
              details=f"open lid max z={lid_open_aabb[1][2]:.3f}")

    # --- Shallow storage: shelf exists below lid ---
    shelf = ctx.part_element_world_aabb(carcass, elem="shelf_board")
    assert shelf is not None
    ctx.check("shelf_below_lid",
              shelf[1][2] < BODY_TOP - 0.01,
              details=f"shelf top z={shelf[1][2]:.3f}")
    ctx.check("shelf_creates_shallow_storage",
              abs(shelf[1][2] - SHELF_Z - 0.018) < 0.005,
              details=f"shelf top z={shelf[1][2]:.3f}, expected={SHELF_Z + 0.018:.3f}")

    # --- Lid hinge barrels visible at rear top ---
    for i in range(3):
        lb = ctx.part_element_world_aabb(carcass, elem=f"lid_hinge_barrel_{i}")
        assert lb is not None
        ctx.check(f"lid_hinge_barrel_{i}_at_rear_top",
                  lb[1][0] < WALL + 0.03 and lb[0][2] > BODY_TOP - 0.02,
                  details=f"barrel x=({lb[0][0]:.3f},{lb[1][0]:.3f}) z=({lb[0][2]:.3f},{lb[1][2]:.3f})")

    # --- At least one non-fixed joint with proper limits ---
    for jname, j in [("left_door", left_joint), ("right_door", right_joint), ("lid", lid_joint)]:
        ctx.check(f"{jname}_has_range",
                  j.motion_limits.upper > 0.5,
                  details=f"{jname} upper={j.motion_limits.upper}")
        ctx.check(f"{jname}_lower_zero",
                  abs(j.motion_limits.lower) < 1e-6,
                  details=f"{jname} lower={j.motion_limits.lower}")

    return ctx.report()


object_model = build_object_model()
