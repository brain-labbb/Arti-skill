from __future__ import annotations

# Variant 09: wide black wooden cabinet with lift-up lid, double doors,
# rotating centre latch, and visible hinge barrels.
#
# Derived from the parent double dresser: same carcass (~1.70 m W x 0.85 m H
# x 0.50 m D), carved corner posts, and square legs.  The eight drawers are
# replaced by two hinged cabinet doors on the front and a lift-up lid on top
# that reveals a shallow storage compartment.  A small rotating latch at the
# centre seam locks the doors.
#
# World frame: front faces +X (back at x=0, front at x=BD), width along Y
# (centred), height along +Z, grounded at z=0.

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

# ---------- key dimensions (metres) ----------
W_TOTAL = 1.70
D_TOTAL = 0.50
H_TOTAL = 0.85
OVERHANG = 0.020
TOP_THK = 0.022

BW = W_TOTAL - 2 * OVERHANG          # 1.66
BD = D_TOTAL - 2 * OVERHANG          # 0.46
LEG_H = 0.150
BODY_BOT = LEG_H                     # 0.150
BODY_TOP = H_TOTAL - TOP_THK         # 0.828
BH = BODY_TOP - BODY_BOT             # 0.678

WALL = 0.018
INNER_W = BW - 2 * WALL              # 1.624

# Shallow storage compartment under the lid.
SHALLOW_DEPTH = 0.080
SHALLOW_FLOOR_THK = 0.016
DOOR_ZONE_TOP = BODY_TOP - SHALLOW_DEPTH   # 0.748
DOOR_ZONE_BOT = BODY_BOT + 0.030           # 0.180
DOOR_H = DOOR_ZONE_TOP - DOOR_ZONE_BOT     # 0.568
DOOR_CZ = (DOOR_ZONE_BOT + DOOR_ZONE_TOP) / 2.0  # 0.464

REVEAL = 0.008
CENTRE_STILE_W = 0.030

# Hinge Y is set inside the carved-post inner edge so the doors clear the posts.
# Post inner edge ~ POST_CY - max_bulge ≈ 0.805 - 0.037 = 0.768.
HINGE_Y = 0.758
STILE_W = INNER_W / 2.0 - HINGE_Y   # ~0.054
DOOR_W = HINGE_Y - CENTRE_STILE_W / 2.0 - REVEAL  # ~0.735
DOOR_THK = 0.018
FACE_PROUD = 0.002

# Carved corner posts (unchanged from parent).
POST_SQ = 0.050
POST_CX = 0.452
POST_CY = BW / 2.0 - 0.025
N_SEG = 12
SEG_H = (BH + (N_SEG - 1) * 0.0015) / N_SEG
LEG_SQ = 0.050

# Knobs (silver ball on short stem).
KNOB_R = 0.0125
STEM_R = 0.0055
STEM_L = 0.014

# Hinge barrels.
BARREL_R = 0.005
BARREL_LEN = 0.035

# Latch.
LATCH_W = 0.060
LATCH_H = 0.012
LATCH_THK = 0.005
LATCH_BOSS_R = 0.008
LATCH_BOSS_L = 0.010

# Lid.
LID_D = D_TOTAL
LID_W = W_TOTAL
LID_THK = TOP_THK


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_cabinet_lid_doors")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    brass = model.material("hinge_brass", rgba=(0.72, 0.55, 0.22, 1.0))

    # ===================================================================
    # ROOT: carcass
    # ===================================================================
    carcass = model.part("carcass")

    # --- side panels ---
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD, WALL, BH)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                               BODY_BOT + BH / 2.0)),
            material=black,
            name=f"side_panel_{tag}",
        )

    # --- back panel ---
    carcass.visual(
        Box((WALL, INNER_W, BH)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=black,
        name="back_panel",
    )

    # --- bottom board ---
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + 0.009)),
        material=black,
        name="bottom_board",
    )

    # --- top rim (thin frame around lid opening, gives carcass height ~BODY_TOP) ---
    carcass.visual(
        Box((BD, INNER_W, 0.018)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - 0.009)),
        material=black,
        name="top_rim",
    )

    # --- shallow compartment floor ---
    carcass.visual(
        Box((BD, INNER_W, SHALLOW_FLOOR_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0,
                           DOOR_ZONE_TOP + SHALLOW_FLOOR_THK / 2.0)),
        material=black,
        name="shallow_floor",
    )

    # --- front bottom rail ---
    carcass.visual(
        Box((WALL, INNER_W, DOOR_ZONE_BOT - BODY_BOT)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           (BODY_BOT + DOOR_ZONE_BOT) / 2.0)),
        material=black,
        name="front_bottom_rail",
    )

    # --- front side stiles ---
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((WALL, STILE_W, DOOR_H)),
            origin=Origin(xyz=(BD - WALL / 2.0,
                               s * (INNER_W / 2.0 - STILE_W / 2.0),
                               DOOR_CZ)),
            material=black,
            name=f"front_stile_{tag}",
        )

    # --- front centre stile (latch mounts here) ---
    carcass.visual(
        Box((WALL, CENTRE_STILE_W, DOOR_H)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, DOOR_CZ)),
        material=black,
        name="centre_stile",
    )

    # --- door hinge barrels (two per door, on carcass side stile face) ---
    # Barrels are vertical cylinders (along Z) at the hinge axis.
    barrel_zs = [DOOR_ZONE_BOT + 0.080, DOOR_ZONE_TOP - 0.080]
    for dtag, hy in (("left", -HINGE_Y), ("right", HINGE_Y)):
        for i, bz in enumerate(barrel_zs):
            carcass.visual(
                Cylinder(radius=BARREL_R, length=BARREL_LEN),
                origin=Origin(xyz=(BD - WALL / 2.0, hy, bz)),
                material=brass,
                name=f"{dtag}_hinge_barrel_{i}",
            )

    # --- lid hinge barrels (three along back top edge, oriented along Y) ---
    # Position slightly below lid bottom to avoid overlap (center at BODY_TOP - 0.003).
    for i, ly in enumerate([-0.35, 0.0, 0.35]):
        carcass.visual(
            Cylinder(radius=0.004, length=0.040),
            origin=Origin(xyz=(WALL, ly, BODY_TOP - 0.003),
                          rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"lid_hinge_barrel_{i}",
        )

    # --- carved corner posts (same spiral/zigzag pattern as parent) ---
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

    # --- four square legs ---
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
        Box((BD, BW, BH)), mass=55.0)

    # ===================================================================
    # LID  (silver-gray panel, hinged at back top edge)
    # ===================================================================
    lid = model.part("lid")
    # Lid panel extends from hinge along +X in local frame.
    lid.visual(
        Box((LID_D, LID_W, LID_THK)),
        origin=Origin(xyz=(LID_D / 2.0 - OVERHANG, 0.0, LID_THK / 2.0)),
        material=silver_top,
        name="lid_panel",
    )
    # Underside liner.
    lid.visual(
        Box((0.38, 1.58, 0.004)),
        origin=Origin(xyz=(0.23, 0.0, -0.002)),
        material=black,
        name="lid_liner",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_D, LID_W, LID_THK)), mass=3.0)

    # ===================================================================
    # LEFT DOOR
    # ===================================================================
    left_door = model.part("left_door")
    # Door panel: hinge at local origin, extends inward (+Y) and outward (+X face).
    left_door.visual(
        Box((DOOR_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(FACE_PROUD - DOOR_THK / 2.0, DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )
    # Knob near the centre (latch) edge.
    _ky = DOOR_W - 0.050
    left_door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=(FACE_PROUD + (STEM_L - 0.004) / 2.0, _ky, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    left_door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(FACE_PROUD + STEM_L + KNOB_R - 0.004, _ky, 0.0)),
        material=silver,
        name="knob_ball",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, DOOR_H)), mass=5.0)

    # ===================================================================
    # RIGHT DOOR
    # ===================================================================
    right_door = model.part("right_door")
    right_door.visual(
        Box((DOOR_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(FACE_PROUD - DOOR_THK / 2.0, -DOOR_W / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )
    _ky_r = -(DOOR_W - 0.050)
    right_door.visual(
        Cylinder(radius=STEM_R, length=STEM_L + 0.004),
        origin=Origin(xyz=(FACE_PROUD + (STEM_L - 0.004) / 2.0, _ky_r, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="knob_stem",
    )
    right_door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(FACE_PROUD + STEM_L + KNOB_R - 0.004, _ky_r, 0.0)),
        material=silver,
        name="knob_ball",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, DOOR_H)), mass=5.0)

    # ===================================================================
    # LATCH  (small rotating bar at centre seam, mounted on carcass stile)
    # ===================================================================
    latch = model.part("latch")
    # Pivot boss embedded into the centre stile (extends toward -X).
    latch.visual(
        Cylinder(radius=LATCH_BOSS_R, length=LATCH_BOSS_L),
        origin=Origin(xyz=(-LATCH_BOSS_L / 2.0, 0.0, 0.0),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="latch_boss",
    )
    # Locking bar (horizontal at q=0, vertical at q=π/2).
    latch.visual(
        Box((LATCH_THK, LATCH_W, LATCH_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=silver,
        name="latch_bar",
    )
    latch.inertial = Inertial.from_geometry(
        Box((LATCH_THK, LATCH_W, LATCH_H)), mass=0.15)

    # ===================================================================
    # ARTICULATIONS
    # ===================================================================

    # Lid: back top edge, axis -Y → positive q lifts the front edge upward.
    model.articulation(
        "carcass_to_lid",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lid,
        origin=Origin(xyz=(-OVERHANG, 0.0, BODY_TOP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0,
                                   lower=0.0, upper=1.4),
    )

    # Left door: hinge at left inner stile edge, axis -Z → positive q opens outward.
    model.articulation(
        "carcass_to_left_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(BD, -HINGE_Y, DOOR_CZ)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0,
                                   lower=0.0, upper=1.5),
    )

    # Right door: hinge at right inner stile edge, axis +Z → positive q opens outward.
    model.articulation(
        "carcass_to_right_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(BD, HINGE_Y, DOOR_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0,
                                   lower=0.0, upper=1.5),
    )

    # Latch: centre front on stile face, axis +X → bar rotates horizontal→vertical.
    # Origin at the stile front face (BD) so the boss embeds into the stile.
    model.articulation(
        "carcass_to_latch",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=latch,
        origin=Origin(xyz=(BD, 0.0, DOOR_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0,
                                   lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    lid = object_model.get_part("lid")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    latch = object_model.get_part("latch")

    lid_hinge = object_model.get_articulation("carcass_to_lid")
    left_hinge = object_model.get_articulation("carcass_to_left_door")
    right_hinge = object_model.get_articulation("carcass_to_right_door")
    latch_joint = object_model.get_articulation("carcass_to_latch")

    # --- Overall dimensions ---
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
    # Overall height: carcass AABB top + lid thickness (lid sits on top rim).
    carcass_h = cb[1][2] - cb[0][2]
    total_h = cb[1][2] + LID_THK - cb[0][2]
    ctx.check("height_085", abs(total_h - 0.85) < 0.01,
              details=f"carcass_h={carcass_h:.3f}, total_h={total_h:.3f}")

    # --- Lid: REVOLUTE, lateral axis, lifts upward ---
    ctx.check("lid_is_revolute",
              lid_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("lid_axis_lateral",
              abs(lid_hinge.axis[1]) > 0.99 and abs(lid_hinge.axis[0]) < 0.01)
    # Use lid_panel visual to measure opening (part origin is at hinge, doesn't translate).
    lid_panel_closed = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_panel_open = ctx.part_element_world_aabb(lid, elem="lid_panel")
    assert lid_panel_closed is not None and lid_panel_open is not None
    ctx.check("lid_lifts_upward",
              lid_panel_open[1][2] > lid_panel_closed[1][2] + 0.05,
              details=f"closed top={lid_panel_closed[1][2]:.3f}, "
                      f"open top={lid_panel_open[1][2]:.3f}")

    # --- Doors: REVOLUTE, open outward (+X) ---
    for door, hinge, dname in [
        (left_door, left_hinge, "left_door"),
        (right_door, right_hinge, "right_door"),
    ]:
        ctx.check(f"{dname}_is_revolute",
                  hinge.articulation_type == ArticulationType.REVOLUTE)
        face_closed = ctx.part_element_world_aabb(door, elem="door_panel")
        with ctx.pose({hinge: hinge.motion_limits.upper}):
            face_open = ctx.part_element_world_aabb(door, elem="door_panel")
        assert face_closed is not None and face_open is not None
        ctx.check(f"{dname}_opens_outward",
                  face_open[1][0] > face_closed[1][0] + 0.05,
                  details=f"closed front={face_closed[1][0]:.3f}, "
                          f"open front={face_open[1][0]:.3f}")

    # --- Doors seat at carcass front when closed ---
    for door, dname in [(left_door, "left_door"), (right_door, "right_door")]:
        face = ctx.part_element_world_aabb(door, elem="door_panel")
        assert face is not None
        ctx.check(f"{dname}_front_proud",
                  0.0 < face[1][0] - BD < 0.010,
                  details=f"face front x={face[1][0]:.4f}, carcass front={BD}")

    # --- Latch: REVOLUTE, axis along X, bar rotates horizontal→vertical ---
    ctx.check("latch_is_revolute",
              latch_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("latch_axis_depth",
              abs(latch_joint.axis[0]) > 0.99)
    bar_rest = ctx.part_element_world_aabb(latch, elem="latch_bar")
    with ctx.pose({latch_joint: latch_joint.motion_limits.upper}):
        bar_open = ctx.part_element_world_aabb(latch, elem="latch_bar")
    assert bar_rest is not None and bar_open is not None
    rest_y = bar_rest[1][1] - bar_rest[0][1]
    rest_z = bar_rest[1][2] - bar_rest[0][2]
    open_y = bar_open[1][1] - bar_open[0][1]
    open_z = bar_open[1][2] - bar_open[0][2]
    ctx.check("latch_horizontal_at_rest",
              rest_y > rest_z,
              details=f"rest y_span={rest_y:.3f} z_span={rest_z:.3f}")
    ctx.check("latch_vertical_when_unlocked",
              open_z > open_y,
              details=f"open y_span={open_y:.3f} z_span={open_z:.3f}")

    # --- Hinge barrels exist on carcass ---
    for name in ["left_hinge_barrel_0", "left_hinge_barrel_1",
                 "right_hinge_barrel_0", "right_hinge_barrel_1"]:
        ctx.check(f"{name}_exists", carcass.get_visual(name) is not None)
    for name in ["lid_hinge_barrel_0", "lid_hinge_barrel_1", "lid_hinge_barrel_2"]:
        ctx.check(f"{name}_exists", carcass.get_visual(name) is not None)

    # --- Door hinge barrels are near their respective hinge axes ---
    for dtag, hy in [("left", -HINGE_Y), ("right", HINGE_Y)]:
        for i in range(2):
            bb = ctx.part_element_world_aabb(carcass, elem=f"{dtag}_hinge_barrel_{i}")
            assert bb is not None
            barrel_cy = (bb[0][1] + bb[1][1]) / 2.0
            ctx.check(f"{dtag}_barrel_{i}_at_hinge_y",
                      abs(barrel_cy - hy) < 0.003,
                      details=f"barrel cy={barrel_cy:.4f}, hinge y={hy:.4f}")

    # --- Carved posts preserved ---
    seg0 = ctx.part_element_world_aabb(carcass, elem="carved_post_0_seg_0")
    assert seg0 is not None
    ctx.check("posts_span_body", seg0[0][2] < LEG_H + 0.002)

    # --- Allow overlaps ---
    # Hinge barrels are the visible knuckles at the door pivot axes.
    ctx.allow_overlap(
        "carcass", "left_door",
        elem_a="left_hinge_barrel_0", elem_b="door_panel",
        reason="Brass hinge barrel is the visible knuckle at the left door pivot axis.",
    )
    ctx.allow_overlap(
        "carcass", "left_door",
        elem_a="left_hinge_barrel_1", elem_b="door_panel",
        reason="Brass hinge barrel is the visible knuckle at the left door pivot axis.",
    )
    ctx.allow_overlap(
        "carcass", "right_door",
        elem_a="right_hinge_barrel_0", elem_b="door_panel",
        reason="Brass hinge barrel is the visible knuckle at the right door pivot axis.",
    )
    ctx.allow_overlap(
        "carcass", "right_door",
        elem_a="right_hinge_barrel_1", elem_b="door_panel",
        reason="Brass hinge barrel is the visible knuckle at the right door pivot axis.",
    )
    # Latch boss and bar embed into the carcass centre stile (pivot mount).
    ctx.allow_overlap(
        "carcass", "latch",
        elem_a="centre_stile", elem_b="latch_boss",
        reason="Latch pivot boss is embedded into the centre stile as a captured pivot.",
    )
    ctx.allow_overlap(
        "carcass", "latch",
        elem_a="centre_stile", elem_b="latch_bar",
        reason="Latch bar sits against the centre stile face when horizontal.",
    )
    # Latch bar spans both door fronts when locked.
    ctx.allow_overlap(
        "left_door", "latch",
        elem_a="door_panel", elem_b="latch_bar",
        reason="Latch bar overlaps the left door front when locked across the seam.",
    )
    ctx.allow_overlap(
        "right_door", "latch",
        elem_a="door_panel", elem_b="latch_bar",
        reason="Latch bar overlaps the right door front when locked across the seam.",
    )

    # Proof: latch contacts the carcass centre stile (mounted pivot).
    ctx.expect_contact(
        latch, carcass,
        contact_tol=0.005,
        name="latch_mounted_on_carcass",
    )

    # Proof: doors contact carcass near hinges at rest.
    for door, dname in [(left_door, "left_door"), (right_door, "right_door")]:
        ctx.expect_contact(
            door, carcass,
            contact_tol=0.005,
            name=f"{dname}_near_carcass_hinge",
        )

    # Proof: lid contacts carcass at back top edge.
    ctx.expect_contact(
        lid, carcass,
        contact_tol=0.005,
        name="lid_near_carcass_hinge",
    )

    return ctx.report()


object_model = build_object_model()
