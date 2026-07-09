from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Widespread two-handle bathroom faucet in polished gold brass.
# Deck-mounted: three units on a horizontal deck plate.
#
# Frame conventions:
#   - Deck is the horizontal XY plane; slab from z=0 to z=DECK_H.
#   - +Z is up; +Y is "forward" (toward the user at the sink).
#   - Left handle at x=-HANDLE_SPACING, right at +HANDLE_SPACING.
#   - Spout rises from center, curves forward and downward.
# ---------------------------------------------------------------------------

# Deck (mounting surface)
DECK_W = 0.34
DECK_D = 0.14
DECK_H = 0.012

# Layout
HANDLE_SPACING = 0.10

# Spout tube
SPOUT_TUBE_R = 0.015
SPOUT_BORE_R = 0.011
SPOUT_RISE = 0.12
SPOUT_BEND_R = 0.05
SPOUT_DROP_LEN = 0.04

# Spout base
SPOUT_BASE_R = 0.022
SPOUT_BASE_H = 0.035
BEARING_R = 0.018
BEARING_H = 0.004

# Pedestal (tapered frustum)
PED_R_BOT = 0.024
PED_R_TOP = 0.016
PED_H = 0.055

# Lever handle
LEVER_R = 0.008
LEVER_LEN = 0.060
LEVER_COLLAR_R = 0.012
LEVER_COLLAR_H = 0.006

# Seam rings
SEAM_EXTRA = 0.003
SEAM_H = 0.0015

# ── Spout path geometry ──────────────────────────────────────────
# 135° clockwise arc from upward to right-and-downward tangent.
# Arc center at (SPOUT_BEND_R, SPOUT_RISE) in the YZ plane.
_ARC_SWEEP = 3.0 * math.pi / 4.0  # 135°
_END_ANGLE = math.pi - _ARC_SWEEP  # π/4
_MID_ANGLE = math.pi - _ARC_SWEEP / 2.0  # 5π/8

SPOUT_MID_Y = SPOUT_BEND_R + SPOUT_BEND_R * math.cos(_MID_ANGLE)
SPOUT_MID_Z = SPOUT_RISE + SPOUT_BEND_R * math.sin(_MID_ANGLE)
SPOUT_END_Y = SPOUT_BEND_R + SPOUT_BEND_R * math.cos(_END_ANGLE)
SPOUT_END_Z = SPOUT_RISE + SPOUT_BEND_R * math.sin(_END_ANGLE)

# Drop line direction = clockwise tangent at arc end
_DROP_DY = math.sin(_END_ANGLE)
_DROP_DZ = -math.cos(_END_ANGLE)
SPOUT_OUTLET_Y = SPOUT_END_Y + SPOUT_DROP_LEN * _DROP_DY
SPOUT_OUTLET_Z = SPOUT_END_Z + SPOUT_DROP_LEN * _DROP_DZ

# Computed volumes for hollow-bore verification
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_tube() -> cq.Workplane:
    """Hollow gooseneck spout in local frame: origin at tube base, +Z up,
    curve toward +Y.  The path starts along +Z for tangent continuity
    with the vertical riser, then arcs clockwise through 135°."""
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc((SPOUT_MID_Y, SPOUT_MID_Z), (SPOUT_END_Y, SPOUT_END_Z))
        .lineTo(SPOUT_OUTLET_Y, SPOUT_OUTLET_Z)
    )
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -0.005)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc((SPOUT_MID_Y, SPOUT_MID_Z), (SPOUT_END_Y, SPOUT_END_Z))
        .lineTo(
            SPOUT_OUTLET_Y + 0.005 * _DROP_DY,
            SPOUT_OUTLET_Z + 0.005 * _DROP_DZ,
        )
    )
    tube = cq.Workplane("XY").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.005)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )
    unbored = tube
    solid = tube.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_tapered_pedestal() -> cq.Workplane:
    """Tapered frustum pedestal: wider at bottom, narrower at top."""
    return (
        cq.Workplane("XY")
        .circle(PED_R_BOT)
        .workplane(offset=PED_H)
        .circle(PED_R_TOP)
        .loft()
    )


def _build_seam_ring(base_r: float) -> cq.Workplane:
    """Thin annular seam ring at a deck-base interface."""
    return (
        cq.Workplane("XY")
        .circle(base_r + SEAM_EXTRA)
        .circle(base_r)
        .extrude(SEAM_H)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    seam_mat = model.material("seam_bronze", rgba=(0.30, 0.24, 0.10, 1.0))
    deck_mat = model.material("deck_stone", rgba=(0.90, 0.88, 0.85, 1.0))

    # Pre-build shared meshes
    spout_mesh = mesh_from_cadquery(_build_spout_tube(), "spout_tube")
    pedestal_mesh = mesh_from_cadquery(_build_tapered_pedestal(), "tapered_pedestal")
    seam_spout_mesh = mesh_from_cadquery(_build_seam_ring(SPOUT_BASE_R), "seam_spout")
    seam_ped_mesh = mesh_from_cadquery(_build_seam_ring(PED_R_BOT), "seam_pedestal")

    # ── Deck (root, mounting surface) ─────────────────────────────
    deck = model.part("deck")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_H)),
        origin=Origin(xyz=(0.0, 0.0, DECK_H / 2.0)),
        material=deck_mat,
        name="slab",
    )

    # ── Spout base (fixed to deck at center) ──────────────────────
    spout_base = model.part("spout_base")
    spout_base.visual(
        Cylinder(radius=SPOUT_BASE_R, length=SPOUT_BASE_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_BASE_H / 2.0)),
        material=gold,
        name="base_column",
    )
    spout_base.visual(
        Cylinder(radius=BEARING_R, length=BEARING_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_BASE_H + BEARING_H / 2.0)),
        material=gold,
        name="bearing_race",
    )
    spout_base.visual(
        seam_spout_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=seam_mat,
        name="seam_ring",
    )
    model.articulation(
        "deck_to_spout_base",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_base,
        origin=Origin(xyz=(0.0, 0.0, DECK_H)),
    )

    # ── Spout (continuous swivel on vertical axis) ────────────────
    spout = model.part("spout")
    spout.visual(spout_mesh, material=gold, name="tube")
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=spout_base,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_BASE_H + BEARING_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0),
    )

    # ── Valve pedestals (fixed) and lever handles (revolute) ─────
    for side, sx in (("left", -1.0), ("right", 1.0)):
        pedestal = model.part(f"{side}_pedestal")
        pedestal.visual(pedestal_mesh, material=gold, name="tapered_body")
        pedestal.visual(
            seam_ped_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=seam_mat,
            name="seam_ring",
        )
        model.articulation(
            f"deck_to_{side}_pedestal",
            ArticulationType.FIXED,
            parent=deck,
            child=pedestal,
            origin=Origin(xyz=(sx * HANDLE_SPACING, 0.0, DECK_H)),
        )

        lever = model.part(f"{side}_lever")
        # Collar where lever meets pedestal top
        lever.visual(
            Cylinder(radius=LEVER_COLLAR_R, length=LEVER_COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, LEVER_COLLAR_H / 2.0)),
            material=gold,
            name="collar",
        )
        # Lever bar extending forward (+Y) from collar top
        lever.visual(
            Cylinder(radius=LEVER_R, length=LEVER_LEN),
            origin=Origin(
                xyz=(0.0, LEVER_LEN / 2.0, LEVER_COLLAR_H),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=gold,
            name="lever_bar",
        )
        # Rounded tip
        lever.visual(
            Sphere(radius=LEVER_R),
            origin=Origin(xyz=(0.0, LEVER_LEN, LEVER_COLLAR_H)),
            material=gold,
            name="tip",
        )
        model.articulation(
            f"{side}_lever_joint",
            ArticulationType.REVOLUTE,
            parent=pedestal,
            child=lever,
            origin=Origin(xyz=(0.0, 0.0, PED_H)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=3.0,
                lower=-math.pi / 2.0,
                upper=math.pi / 2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout_base = object_model.get_part("spout_base")
    spout = object_model.get_part("spout")
    left_ped = object_model.get_part("left_pedestal")
    right_ped = object_model.get_part("right_pedestal")
    left_lever = object_model.get_part("left_lever")
    right_lever = object_model.get_part("right_lever")

    swivel = object_model.get_articulation("spout_swivel")
    left_joint = object_model.get_articulation("left_lever_joint")
    right_joint = object_model.get_articulation("right_lever_joint")

    # ── Spout swivel: continuous vertical joint ───────────────────
    ctx.check(
        "spout_swivel_is_continuous",
        str(swivel.joint_type).lower().endswith("continuous"),
        f"type={swivel.joint_type}",
    )
    ctx.check(
        "spout_swivel_axis_vertical",
        abs(swivel.axis[0]) < 1e-9
        and abs(swivel.axis[1]) < 1e-9
        and abs(swivel.axis[2] - 1.0) < 1e-9,
        f"axis={swivel.axis}",
    )

    # ── Lever joints: revolute about vertical axis ────────────────
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_is_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ctx.check(
            f"{joint.name}_axis_vertical",
            abs(joint.axis[0]) < 1e-9
            and abs(joint.axis[1]) < 1e-9
            and abs(joint.axis[2] - 1.0) < 1e-9,
            f"axis={joint.axis}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_quarter_turn_range",
            lim is not None
            and abs(lim.lower + math.pi / 2.0) < 1e-6
            and abs(lim.upper - math.pi / 2.0) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # ── Spout is hollow with visible bore ─────────────────────────
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} vs unbored={SPOUT_UNBORED_VOLUME:.3e}",
    )

    # ── Spout rises and curves forward ────────────────────────────
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    spout_frame_z = DECK_H + SPOUT_BASE_H + BEARING_H
    ctx.check(
        "spout_rises_well_above_deck",
        sz1 > spout_frame_z + 0.10,
        f"spout zmax={sz1:.3f}, base top={spout_frame_z:.3f}",
    )
    ctx.check(
        "spout_curves_forward_positive_y",
        sy1 > 0.04,
        f"spout ymax={sy1:.3f}",
    )

    # ── Tapered pedestals: X-width is between top and bottom dia ──
    for ped in (left_ped, right_ped):
        ped_aabb = ctx.part_element_world_aabb(ped, elem="tapered_body")
        assert ped_aabb is not None
        (px0, _, _), (px1, _, _) = ped_aabb
        x_span = px1 - px0
        ctx.check(
            f"{ped.name}_tapered_profile",
            2.0 * PED_R_TOP * 0.9 < x_span < 2.0 * PED_R_BOT * 1.1,
            f"x_span={x_span:.4f}, expected between {2*PED_R_TOP:.4f} and {2*PED_R_BOT:.4f}",
        )

    # ── Lever handles extend forward (not cross handles) ──────────
    for lever in (left_lever, right_lever):
        la = ctx.part_world_aabb(lever)
        assert la is not None
        y_span = la[1][1] - la[0][1]
        ctx.check(
            f"{lever.name}_lever_extends_forward",
            y_span > LEVER_LEN * 0.7,
            f"y_span={y_span:.4f}, expected >= {LEVER_LEN * 0.7:.4f}",
        )

    # ── Seam rings at all three deck bases ────────────────────────
    for p in (spout_base, left_ped, right_ped):
        seam_vis = p.get_visual("seam_ring")
        ctx.check(
            f"{p.name}_has_seam_ring",
            seam_vis is not None,
            f"part {p.name} missing seam_ring visual",
        )

    # Seam rings sit near deck surface
    for p in (spout_base, left_ped, right_ped):
        sa = ctx.part_element_world_aabb(p, elem="seam_ring")
        assert sa is not None
        ctx.check(
            f"{p.name}_seam_near_deck_surface",
            abs(sa[0][2] - DECK_H) < 0.003,
            f"seam zmin={sa[0][2]:.4f}, deck top={DECK_H}",
        )

    # ── Three-piece widespread layout ─────────────────────────────
    lv = ctx.part_world_position(left_ped)
    rv = ctx.part_world_position(right_ped)
    sv = ctx.part_world_position(spout_base)
    assert lv is not None and rv is not None and sv is not None
    ctx.check(
        "three_piece_layout",
        abs(sv[0]) < 0.005 and lv[0] < -0.05 and rv[0] > 0.05,
        f"left_x={lv[0]:.3f}, spout_x={sv[0]:.3f}, right_x={rv[0]:.3f}",
    )

    # ── Spout swivel pose: rotates about vertical, XY stays fixed ─
    rest_pos = ctx.part_world_position(spout)
    rest_aabb = spout_aabb
    with ctx.pose({swivel: math.pi / 2.0}):
        rot_pos = ctx.part_world_position(spout)
        assert rest_pos is not None and rot_pos is not None
        ctx.check(
            "spout_xy_stable_under_swivel",
            abs(rot_pos[0] - rest_pos[0]) < 1e-5
            and abs(rot_pos[1] - rest_pos[1]) < 1e-5,
            f"rest_xy=({rest_pos[0]:.4f},{rest_pos[1]:.4f}), "
            f"rot_xy=({rot_pos[0]:.4f},{rot_pos[1]:.4f})",
        )
        rot_aabb = ctx.part_world_aabb(spout)
        assert rot_aabb is not None
        ctx.check(
            "spout_forward_direction_changes_under_swivel",
            abs(rot_aabb[1][1] - rest_aabb[1][1]) > 0.01
            or abs(rot_aabb[0][0] - rest_aabb[0][0]) > 0.01,
            f"rest ymax={rest_aabb[1][1]:.3f}, rot ymax={rot_aabb[1][1]:.3f}",
        )

    # ── Lever rotation pose: quarter turn moves lever off +Y axis ─
    rest_lever_aabb = ctx.part_world_aabb(left_lever)
    assert rest_lever_aabb is not None
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_lever_aabb = ctx.part_world_aabb(left_lever)
        assert rot_lever_aabb is not None
        ctx.check(
            "left_lever_rotates_off_forward",
            rot_lever_aabb[1][1] < rest_lever_aabb[1][1] - 0.005,
            f"rest ymax={rest_lever_aabb[1][1]:.4f}, "
            f"rot ymax={rot_lever_aabb[1][1]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
