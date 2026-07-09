from __future__ import annotations

# Storage cabinet variant: wide black cabinet (~1.70 m W x 0.85 m H x 0.50 m D)
# with glass-framed upper doors, solid lower doors, a sliding tambour front,
# visible hinge barrels, and caster blocks at the base.
#
# World layout: front faces +X (back at x=0, front at x=BD), width along Y
# (centered), height along +Z, grounded at z=0 on four caster blocks.

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
WALL = 0.018
INNER_W = BW - 2 * WALL          # 1.624

FOOT_H = 0.065                    # caster block height
BODY_BOT = FOOT_H                 # 0.065
BODY_TOP = H_TOTAL - TOP_THK     # 0.828
BH = BODY_TOP - BODY_BOT         # 0.763

FACE_THK = 0.018
FACE_PROUD = 0.002
REVEAL = 0.005

# Zone heights from BODY_BOT upward (total = BH = 0.763):
BOT_RAIL_H = 0.020
LOWER_H = 0.260
MID_RAIL_H = 0.022
TAMBOUR_H = 0.130
UP_RAIL_H = 0.022
UPPER_H = 0.285
TOP_RAIL_H = BH - BOT_RAIL_H - LOWER_H - MID_RAIL_H - TAMBOUR_H - UP_RAIL_H - UPPER_H

# Z boundaries:
Z0 = BODY_BOT
Z1 = Z0 + BOT_RAIL_H              # 0.085
Z2 = Z1 + LOWER_H                 # 0.345
Z3 = Z2 + MID_RAIL_H              # 0.367
Z4 = Z3 + TAMBOUR_H               # 0.497
Z5 = Z4 + UP_RAIL_H               # 0.519
Z6 = Z5 + UPPER_H                 # 0.804
Z7 = Z6 + TOP_RAIL_H              # 0.828 = BODY_TOP

# Door zone centers:
LOWER_CZ = (Z1 + Z2) / 2.0
UPPER_CZ = (Z5 + Z6) / 2.0
TAMBOUR_CZ = (Z3 + Z4) / 2.0

# Door openings: center stile divides the front into left/right halves.
CENTER_STILE_W = 0.024
HALF_OPEN_W = (INNER_W - CENTER_STILE_W) / 2.0   # 0.800
DOOR_W = HALF_OPEN_W - REVEAL                     # 0.795

# Hinge Y positions (slightly inward from side panel inner faces to avoid overlap):
LEFT_HINGE_Y = -INNER_W / 2.0 + 0.008      # -0.804
RIGHT_HINGE_Y = INNER_W / 2.0 - 0.008       # 0.804

# Front frame stile widths:
SIDE_STILE_W = 0.0  # stiles are the side panels themselves at front

FRONT_X = BD                          # carcass front plane

# Hinge barrel dimensions:
HB_R = 0.006
HB_H = 0.024
HB_INSET = 0.030  # distance from door top/bottom edge

# Tambour:
TAMBOUR_THK = 0.012
TAMBOUR_PANEL_W = 0.75
TAMBOUR_PANEL_H = TAMBOUR_H - 0.016  # gap for tracks
TAMBOUR_Y_OFFSET = -(INNER_W / 2.0) + TAMBOUR_PANEL_W / 2.0  # start at left
TAMBOUR_TRAVEL = 0.78

# Knob:
KNOB_R = 0.012
STEM_R = 0.005
STEM_L = 0.012

# Caster:
CASTER_W = 0.050
CASTER_D = 0.050
WHEEL_R = 0.018
WHEEL_W = 0.016

# Glass frame:
FRAME_W = 0.028   # frame member width
FRAME_THK = FACE_THK
GLASS_THK = 0.004


def _build_glass_door(model, name, door_w, door_h, hinge_side):
    """Glass-framed door. hinge_side: 'left' or 'right'.
    Part origin at hinge edge, door extends away from hinge."""
    door = model.part(name)

    # Direction: +1 for left-hinge (door extends +Y), -1 for right-hinge (-Y)
    d = 1 if hinge_side == "left" else -1

    # Frame: 4 border strips
    inner_w = door_w - 2 * FRAME_W
    inner_h = door_h - 2 * FRAME_W

    # Top frame rail
    door.visual(
        Box((FRAME_THK, door_w, FRAME_W)),
        origin=Origin(xyz=(-FRAME_THK / 2.0 + FACE_PROUD, d * door_w / 2.0, door_h / 2.0 - FRAME_W / 2.0)),
        material=model.materials[0],
        name="frame_top",
    )
    # Bottom frame rail
    door.visual(
        Box((FRAME_THK, door_w, FRAME_W)),
        origin=Origin(xyz=(-FRAME_THK / 2.0 + FACE_PROUD, d * door_w / 2.0, -door_h / 2.0 + FRAME_W / 2.0)),
        material=model.materials[0],
        name="frame_bottom",
    )
    # Hinge-side frame stile
    door.visual(
        Box((FRAME_THK, FRAME_W, inner_h)),
        origin=Origin(xyz=(-FRAME_THK / 2.0 + FACE_PROUD, d * FRAME_W / 2.0, 0.0)),
        material=model.materials[0],
        name="frame_hinge_stile",
    )
    # Free-edge frame stile
    door.visual(
        Box((FRAME_THK, FRAME_W, inner_h)),
        origin=Origin(xyz=(-FRAME_THK / 2.0 + FACE_PROUD, d * (door_w - FRAME_W / 2.0), 0.0)),
        material=model.materials[0],
        name="frame_free_stile",
    )

    # Glass panel (semi-transparent, inset within frame)
    door.visual(
        Box((GLASS_THK, inner_w, inner_h)),
        origin=Origin(xyz=(-GLASS_THK / 2.0 + FACE_PROUD, d * door_w / 2.0, 0.0)),
        material=model.materials[4],  # glass material
        name="glass_panel",
    )

    # Cross mullion (vertical divider in the glass)
    door.visual(
        Box((FRAME_THK, 0.012, inner_h)),
        origin=Origin(xyz=(-FRAME_THK / 2.0 + FACE_PROUD, d * door_w / 2.0, 0.0)),
        material=model.materials[0],
        name="mullion_vertical",
    )
    # Horizontal mullion
    door.visual(
        Box((FRAME_THK, inner_w, 0.012)),
        origin=Origin(xyz=(-FRAME_THK / 2.0 + FACE_PROUD, d * door_w / 2.0, 0.0)),
        material=model.materials[0],
        name="mullion_horizontal",
    )

    # Hinge barrels (2 per door, at hinge edge)
    for i, z_off in enumerate([door_h / 2.0 - HB_INSET, -door_h / 2.0 + HB_INSET]):
        door.visual(
            Cylinder(radius=HB_R, length=HB_H),
            origin=Origin(xyz=(0.0, 0.0, z_off),
                          rpy=(0.0, 0.0, 0.0)),  # cylinder along +Z
            material=model.materials[3],  # silver
            name=f"hinge_barrel_{i}",
        )

    # Knob at free edge
    knob_y = d * (door_w - 0.040)
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0 + FACE_PROUD, knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=model.materials[3],
        name="knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004 + FACE_PROUD, knob_y, 0.0)),
        material=model.materials[3],
        name="knob_ball",
    )

    door.inertial = Inertial.from_geometry(
        Box((FACE_THK, door_w, door_h)), mass=3.5)
    return door


# Hinge-barrel-on-door frame offset helper:
# Place hinge barrel at the door hinge edge (local y=0, x=0),
# but offset slightly in X to avoid full overlap with carcass barrel.
HB_DOOR_X_OFFSET = 0.004  # shift door barrel slightly outward


def _build_solid_door(model, name, door_w, door_h, hinge_side):
    """Solid wood door. hinge_side: 'left' or 'right'."""
    door = model.part(name)
    d = 1 if hinge_side == "left" else -1

    # Solid panel
    door.visual(
        Box((FACE_THK, door_w, door_h)),
        origin=Origin(xyz=(-FACE_THK / 2.0 + FACE_PROUD, d * door_w / 2.0, 0.0)),
        material=model.materials[0],
        name="door_panel",
    )

    # Raised center panel detail (thin inset rectangle)
    inset_w = door_w - 0.080
    inset_h = door_h - 0.080
    door.visual(
        Box((0.004, inset_w, inset_h)),
        origin=Origin(xyz=(FACE_PROUD + 0.002, d * door_w / 2.0, 0.0)),
        material=model.materials[1],  # deeper black
        name="raised_panel",
    )

    # Hinge barrels (2 per door)
    for i, z_off in enumerate([door_h / 2.0 - HB_INSET, -door_h / 2.0 + HB_INSET]):
        door.visual(
            Cylinder(radius=HB_R, length=HB_H),
            origin=Origin(xyz=(0.0, 0.0, z_off)),
            material=model.materials[3],
            name=f"hinge_barrel_{i}",
        )

    # Knob at free edge
    knob_y = d * (door_w - 0.040)
    door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=((STEM_L - 0.004) / 2.0 + FACE_PROUD, knob_y, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=model.materials[3],
        name="knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(STEM_L + KNOB_R - 0.004 + FACE_PROUD, knob_y, 0.0)),
        material=model.materials[3],
        name="knob_ball",
    )

    door.inertial = Inertial.from_geometry(
        Box((FACE_THK, door_w, door_h)), mass=4.0)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_storage_cabinet")

    # Materials
    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    glass = model.material("glass_tint", rgba=(0.78, 0.85, 0.88, 0.35))
    caster_mat = model.material("caster_dark", rgba=(0.15, 0.15, 0.16, 1.0))

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

    # Top stretcher board
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black,
        name="top_stretcher",
    )

    # Front frame rails
    # Bottom rail
    carcass.visual(
        Box((WALL, INNER_W, BOT_RAIL_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, Z0 + BOT_RAIL_H / 2.0)),
        material=black,
        name="front_bottom_rail",
    )
    # Mid rail (between lower doors and tambour)
    carcass.visual(
        Box((WALL, INNER_W, MID_RAIL_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, Z2 + MID_RAIL_H / 2.0)),
        material=black,
        name="front_mid_rail",
    )
    # Upper rail (between tambour and upper doors)
    carcass.visual(
        Box((WALL, INNER_W, UP_RAIL_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, Z4 + UP_RAIL_H / 2.0)),
        material=black,
        name="front_upper_rail",
    )
    # Top rail
    carcass.visual(
        Box((WALL, INNER_W, TOP_RAIL_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, Z6 + TOP_RAIL_H / 2.0)),
        material=black,
        name="front_top_rail",
    )

    # Center stile (vertical divider)
    carcass.visual(
        Box((WALL, CENTER_STILE_W, BH)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=black,
        name="center_stile",
    )

    # Internal shelf in lower compartment
    carcass.visual(
        Box((BD - 0.040, INNER_W, 0.016)),
        origin=Origin(xyz=(0.020 + (BD - 0.040) / 2.0, 0.0, (Z1 + Z2) / 2.0)),
        material=black_deep,
        name="lower_shelf",
    )

    # Internal shelf in upper compartment
    carcass.visual(
        Box((BD - 0.040, INNER_W, 0.016)),
        origin=Origin(xyz=(0.020 + (BD - 0.040) / 2.0, 0.0, (Z5 + Z6) / 2.0)),
        material=black_deep,
        name="upper_shelf",
    )

    # Tambour track rails (thin guides at top and bottom of tambour opening)
    track_thk = 0.006
    track_h = 0.008
    carcass.visual(
        Box((track_thk, INNER_W, track_h)),
        origin=Origin(xyz=(BD - track_thk / 2.0, 0.0, Z3 + track_h / 2.0)),
        material=black_deep,
        name="tambour_track_bottom",
    )
    carcass.visual(
        Box((track_thk, INNER_W, track_h)),
        origin=Origin(xyz=(BD - track_thk / 2.0, 0.0, Z4 - track_h / 2.0)),
        material=black_deep,
        name="tambour_track_top",
    )

    # (Hinge barrels are on the door parts only — visible at the hinge edge.)

    # Silver-gray top slab with overhang
    carcass.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=silver_top,
        name="top_slab",
    )

    # Caster blocks at base (4 corners)
    caster_positions = [
        ("front_left", BD - CASTER_D / 2.0 - 0.020, BW / 2.0 - CASTER_W / 2.0 - 0.020),
        ("front_right", BD - CASTER_D / 2.0 - 0.020, -(BW / 2.0 - CASTER_W / 2.0 - 0.020)),
        ("rear_left", CASTER_D / 2.0 + 0.020, BW / 2.0 - CASTER_W / 2.0 - 0.020),
        ("rear_right", CASTER_D / 2.0 + 0.020, -(BW / 2.0 - CASTER_W / 2.0 - 0.020)),
    ]
    for ctag, cx, cy in caster_positions:
        # Block
        carcass.visual(
            Box((CASTER_D, CASTER_W, FOOT_H - 2 * WHEEL_R)),
            origin=Origin(xyz=(cx, cy, 2 * WHEEL_R + (FOOT_H - 2 * WHEEL_R) / 2.0)),
            material=caster_mat,
            name=f"caster_block_{ctag}",
        )
        # Wheel (cylinder oriented along Y)
        carcass.visual(
            Cylinder(radius=WHEEL_R, length=WHEEL_W),
            origin=Origin(xyz=(cx, cy, WHEEL_R),
                          rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name=f"caster_wheel_{ctag}",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=55.0)

    # ===================================================================
    # DOORS: 2 glass-framed upper + 2 solid lower
    # ===================================================================

    # Upper glass-framed doors
    upper_door_0 = _build_glass_door(model, "upper_door_0", DOOR_W, UPPER_H, "left")
    upper_door_1 = _build_glass_door(model, "upper_door_1", DOOR_W, UPPER_H, "right")

    # Lower solid doors
    lower_door_0 = _build_solid_door(model, "lower_door_0", DOOR_W, LOWER_H, "left")
    lower_door_1 = _build_solid_door(model, "lower_door_1", DOOR_W, LOWER_H, "right")

    # ===================================================================
    # TAMBOUR: sliding front panel
    # ===================================================================
    tambour = model.part("tambour")
    tambour.visual(
        Box((TAMBOUR_THK, TAMBOUR_PANEL_W, TAMBOUR_PANEL_H)),
        origin=Origin(xyz=(-TAMBOUR_THK / 2.0 + FACE_PROUD,
                           TAMBOUR_Y_OFFSET, 0.0)),
        material=black,
        name="tambour_panel",
    )
    # Tambour pull handle (small horizontal bar near right edge of panel)
    handle_cx = FACE_PROUD - TAMBOUR_THK / 2.0 + 0.004  # embedded 4mm into panel front
    handle_cy = TAMBOUR_Y_OFFSET + TAMBOUR_PANEL_W / 2.0 - 0.050  # near right edge
    tambour.visual(
        Box((0.008, 0.080, 0.014)),
        origin=Origin(xyz=(handle_cx, handle_cy, 0.0)),
        material=silver,
        name="tambour_handle",
    )
    tambour.inertial = Inertial.from_geometry(
        Box((TAMBOUR_THK, TAMBOUR_PANEL_W, TAMBOUR_PANEL_H)), mass=2.5)

    # ===================================================================
    # ARTICULATIONS
    # ===================================================================

    # Upper doors: revolute around Z axis at hinge edge
    # Left door (hinge at -Y): axis=(0,0,-1) so positive q opens outward (+X)
    model.articulation(
        "carcass_to_upper_door_0",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=upper_door_0,
        origin=Origin(xyz=(FRONT_X, LEFT_HINGE_Y, UPPER_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.4),
    )
    # Right door (hinge at +Y): axis=(0,0,1) so positive q opens outward (+X)
    model.articulation(
        "carcass_to_upper_door_1",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=upper_door_1,
        origin=Origin(xyz=(FRONT_X, RIGHT_HINGE_Y, UPPER_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.4),
    )

    # Lower doors: same hinge logic
    model.articulation(
        "carcass_to_lower_door_0",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lower_door_0,
        origin=Origin(xyz=(FRONT_X, LEFT_HINGE_Y, LOWER_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.4),
    )
    model.articulation(
        "carcass_to_lower_door_1",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lower_door_1,
        origin=Origin(xyz=(FRONT_X, RIGHT_HINGE_Y, LOWER_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.4),
    )

    # Tambour: prismatic along +Y (slides sideways to the right)
    model.articulation(
        "carcass_to_tambour",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=tambour,
        origin=Origin(xyz=(FRONT_X, 0.0, TAMBOUR_CZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.3, lower=0.0, upper=TAMBOUR_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    # --- Overall dimensions ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("casters_on_floor", abs(cb[0][2]) < 0.005,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_170", abs(width_y - 1.70) < 0.02, details=f"w={width_y:.3f}")
    ctx.check("depth_050", abs(depth_x - 0.50) < 0.04, details=f"d={depth_x:.3f}")
    ctx.check("height_085", abs(height_z - 0.85) < 0.01,
              details=f"h={height_z:.3f}")

    # --- Silver top overhangs ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    assert top is not None and side is not None
    ctx.check("top_overhangs_sides",
              top[1][1] > side[1][1] + 0.015 and top[0][1] < -side[1][1] - 0.015,
              details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f})")

    # --- Caster blocks at base ---
    caster_fl = ctx.part_element_world_aabb(carcass, elem="caster_wheel_front_left")
    caster_rr = ctx.part_element_world_aabb(carcass, elem="caster_wheel_rear_right")
    assert caster_fl is not None and caster_rr is not None
    ctx.check("casters_near_floor",
              caster_fl[0][2] < 0.025 and caster_rr[0][2] < 0.025,
              details=f"caster min z: fl={caster_fl[0][2]:.4f}, rr={caster_rr[0][2]:.4f}")

    # --- Articulation types: at least one revolute and one prismatic ---
    door_joints = [
        object_model.get_articulation("carcass_to_upper_door_0"),
        object_model.get_articulation("carcass_to_upper_door_1"),
        object_model.get_articulation("carcass_to_lower_door_0"),
        object_model.get_articulation("carcass_to_lower_door_1"),
    ]
    tambour_joint = object_model.get_articulation("carcass_to_tambour")

    for j in door_joints:
        ctx.check(f"{j.name}_is_revolute",
                  j.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("tambour_is_prismatic",
              tambour_joint.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("tambour_axis_along_y",
              abs(tambour_joint.axis[1]) > 0.99,
              details=f"axis={tambour_joint.axis}")
    ctx.check("tambour_range",
              abs(tambour_joint.motion_limits.lower) < 1e-9
              and tambour_joint.motion_limits.upper > 0.50,
              details=f"range=({tambour_joint.motion_limits.lower},{tambour_joint.motion_limits.upper})")

    # --- Glass-framed upper doors ---
    ud0 = object_model.get_part("upper_door_0")
    ud1 = object_model.get_part("upper_door_1")
    for dp, name in [(ud0, "upper_door_0"), (ud1, "upper_door_1")]:
        glass = ctx.part_element_world_aabb(dp, elem="glass_panel")
        frame_t = ctx.part_element_world_aabb(dp, elem="frame_top")
        frame_b = ctx.part_element_world_aabb(dp, elem="frame_bottom")
        assert glass is not None and frame_t is not None and frame_b is not None
        ctx.check(f"{name}_has_glass", True)
        # Glass is within the frame bounds vertically
        ctx.check(f"{name}_glass_within_frame",
                  glass[0][2] >= frame_b[0][2] - 0.002
                  and glass[1][2] <= frame_t[1][2] + 0.002,
                  details=f"glass z=({glass[0][2]:.3f},{glass[1][2]:.3f}), "
                          f"frame z=({frame_b[0][2]:.3f},{frame_t[1][2]:.3f})")

    # --- Solid lower doors ---
    ld0 = object_model.get_part("lower_door_0")
    ld1 = object_model.get_part("lower_door_1")
    for dp, name in [(ld0, "lower_door_0"), (ld1, "lower_door_1")]:
        panel = ctx.part_element_world_aabb(dp, elem="door_panel")
        assert panel is not None
        panel_h = panel[1][2] - panel[0][2]
        ctx.check(f"{name}_solid_panel", panel_h > LOWER_H - 0.01,
                  details=f"panel h={panel_h:.3f}")

    # --- Hinge barrels visible ---
    for dp, name in [(ud0, "upper_door_0"), (ld0, "lower_door_0"),
                     (ud1, "upper_door_1"), (ld1, "lower_door_1")]:
        hb = ctx.part_element_world_aabb(dp, elem="hinge_barrel_0")
        assert hb is not None
        barrel_h = hb[1][2] - hb[0][2]
        ctx.check(f"{name}_has_hinge_barrels",
                  barrel_h > HB_H - 0.002,
                  details=f"barrel h={barrel_h:.4f}")

    # --- Upper doors are above lower doors ---
    ud0_aabb = ctx.part_world_aabb(ud0)
    ld0_aabb = ctx.part_world_aabb(ld0)
    assert ud0_aabb is not None and ld0_aabb is not None
    ctx.check("upper_above_lower",
              ud0_aabb[0][2] > ld0_aabb[1][2] - 0.02,
              details=f"upper min z={ud0_aabb[0][2]:.3f}, lower max z={ld0_aabb[1][2]:.3f}")

    # --- Tambour exists and has panel ---
    tamb = object_model.get_part("tambour")
    tp = ctx.part_element_world_aabb(tamb, elem="tambour_panel")
    assert tp is not None
    ctx.check("tambour_panel_exists", (tp[1][1] - tp[0][1]) > 0.50,
              details=f"panel width={tp[1][1] - tp[0][1]:.3f}")

    # (No carcass hinge barrel overlap allowances needed — hinge barrels are on door parts only.)

    # --- Doors open outward: positive q moves door panel to higher x ---
    for dp, j_name, panel_elem in [
        (ud0, "carcass_to_upper_door_0", "frame_free_stile"),
        (ld0, "carcass_to_lower_door_0", "door_panel"),
        (ud1, "carcass_to_upper_door_1", "frame_free_stile"),
        (ld1, "carcass_to_lower_door_1", "door_panel"),
    ]:
        j = object_model.get_articulation(j_name)
        rest_aabb = ctx.part_element_world_aabb(dp, elem=panel_elem)
        with ctx.pose({j: 0.5}):
            open_aabb = ctx.part_element_world_aabb(dp, elem=panel_elem)
        assert rest_aabb is not None and open_aabb is not None
        ctx.check(f"{j_name}_opens_outward",
                  open_aabb[1][0] > rest_aabb[1][0] + 0.05,
                  details=f"rest max_x={rest_aabb[1][0]:.4f}, open max_x={open_aabb[1][0]:.4f}")

    # --- Tambour slides sideways along Y ---
    rest_tamb = ctx.part_world_position(tamb)
    with ctx.pose({tambour_joint: tambour_joint.motion_limits.upper}):
        open_tamb = ctx.part_world_position(tamb)
    assert rest_tamb is not None and open_tamb is not None
    dy = open_tamb[1] - rest_tamb[1]
    ctx.check("tambour_slides_sideways",
              abs(dy - TAMBOUR_TRAVEL) < 0.01,
              details=f"dy={dy:.4f}")
    # Tambour stays at same X and Z
    ctx.check("tambour_no_x_drift",
              abs(open_tamb[0] - rest_tamb[0]) < 0.005,
              details=f"dx={open_tamb[0] - rest_tamb[0]:.4f}")

    # --- Non-fixed joint count ---
    all_joints = [object_model.get_articulation(n) for n in
                  ["carcass_to_upper_door_0", "carcass_to_upper_door_1",
                   "carcass_to_lower_door_0", "carcass_to_lower_door_1",
                   "carcass_to_tambour"]]
    non_fixed = [j for j in all_joints if j.articulation_type != ArticulationType.FIXED]
    ctx.check("at_least_one_non_fixed_joint", len(non_fixed) >= 1,
              details=f"non_fixed={len(non_fixed)}")

    return ctx.report()


object_model = build_object_model()
