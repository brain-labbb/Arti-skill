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
# Deck-mounted on a horizontal countertop surface.
#
# Frame conventions:
#   - The deck (countertop) is the horizontal XY plane at z = 0.
#     The deck slab occupies z < 0 (below counter).
#   - Everything mounts on top of the deck (z > 0).
#   - The spout projects forward along +Y (toward the user).
#   - Left handle at -X, right handle at +X.
# ---------------------------------------------------------------------------

# Layout
VALVE_SPACING_X = 0.12  # valve centers at x = +/- 0.12

# Deck panel (countertop substrate)
DECK_W = 0.40
DECK_D = 0.16
DECK_T = 0.020

# Spout body: rectangular column with waterfall channel
SPOUT_BASE_W = 0.036    # column width (X)
SPOUT_BASE_D = 0.030    # column depth (Y)
SPOUT_COLUMN_H = 0.095  # column height above base flange
SPOUT_CHANNEL_W = 0.052 # waterfall channel width (wider than column)
SPOUT_CHANNEL_D = 0.055 # channel projection forward (Y)
SPOUT_CHANNEL_H = 0.016 # channel body thickness (Z)
SPOUT_SLOT_W = 0.044    # waterfall slot width
SPOUT_SLOT_H = 0.006    # waterfall slot height
SPOUT_FLANGE_R = 0.025  # base flange radius
SPOUT_FLANGE_T = 0.007  # base flange thickness

# Valve assemblies
VALVE_ESC_R1, VALVE_ESC_T1 = 0.028, 0.007
VALVE_ESC_R2, VALVE_ESC_T2 = 0.022, 0.007
VALVE_BODY_R = 0.012
VALVE_BODY_H = 0.035

# Cross handle (horizontal cross on vertical stem)
STEM_R = 0.005
STEM_H = 0.010  # visible stem above valve body (below hub)
HUB_R = 0.011
HUB_H = 0.016
ARM_R = 0.004
ARM_HALF = 0.042  # half-length of each arm from center (tip-to-tip ~0.084)
ARM_Z_OFFSET = 0.008  # arm center height above stem top

# Seam ring (narrow dark ring at each deck base)
SEAM_THICK = 0.0015  # very thin seam


def _build_waterfall_spout() -> cq.Workplane:
    """Spout: rectangular column with wide flat waterfall channel on top,
    including a horizontal slot at the front lip. Origin at deck level (z=0)."""
    # Base flange
    flange = cq.Workplane("XY").circle(SPOUT_FLANGE_R).extrude(SPOUT_FLANGE_T)

    # Rectangular column
    col_z = SPOUT_FLANGE_T
    column = (
        cq.Workplane("XY")
        .workplane(offset=col_z)
        .rect(SPOUT_BASE_W, SPOUT_BASE_D)
        .extrude(SPOUT_COLUMN_H)
    )

    # Waterfall channel body: wide flat box at top, extending forward
    chan_z = col_z + SPOUT_COLUMN_H
    # Channel extends forward from the column center
    fwd_offset = (SPOUT_CHANNEL_D - SPOUT_BASE_D) / 2.0
    channel = (
        cq.Workplane("XY")
        .workplane(offset=chan_z)
        .center(0.0, fwd_offset)
        .rect(SPOUT_CHANNEL_W, SPOUT_CHANNEL_D)
        .extrude(SPOUT_CHANNEL_H)
    )

    solid = flange.union(column).union(channel)

    # Cut waterfall slot at the front lip of the channel
    # Slot is a horizontal rectangular opening at the front face
    slot_z_center = chan_z + SPOUT_CHANNEL_H / 2.0
    slot_y = SPOUT_BASE_D / 2.0 + SPOUT_CHANNEL_D - SPOUT_BASE_D  # near front edge
    slot_cutter = (
        cq.Workplane("XZ")
        .workplane(offset=slot_y)
        .rect(SPOUT_SLOT_W, SPOUT_SLOT_H)
        .extrude(0.020)
    )
    solid = solid.cut(slot_cutter)

    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_waterfall_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    deck_mat = model.material("countertop_white", rgba=(0.92, 0.92, 0.90, 1.0))
    seam_mat = model.material("seam_dark", rgba=(0.22, 0.18, 0.10, 1.0))

    # --- deck panel (root, countertop substrate) ---
    deck = model.part("deck_panel")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T / 2.0)),
        material=deck_mat,
        name="countertop",
    )

    # --- central spout (fixed) ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_waterfall_spout(), "waterfall_spout"),
        material=gold,
        name="channel",
    )
    # Narrow seam ring at spout deck base
    spout.visual(
        Cylinder(radius=SPOUT_FLANGE_R + 0.003, length=SEAM_THICK),
        origin=Origin(xyz=(0.0, 0.0, SEAM_THICK / 2.0)),
        material=seam_mat,
        name="spout_seam",
    )
    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- valve assemblies (fixed) and cross handles (revolute on vertical stems) ---
    valve_top_z = VALVE_ESC_T1 + VALVE_ESC_T2 + VALVE_BODY_H

    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        # Stepped escutcheon base
        valve.visual(
            Cylinder(radius=VALVE_ESC_R1, length=VALVE_ESC_T1),
            origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_T1 / 2.0)),
            material=gold,
            name="escutcheon_base",
        )
        valve.visual(
            Cylinder(radius=VALVE_ESC_R2, length=VALVE_ESC_T2),
            origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_T1 + VALVE_ESC_T2 / 2.0)),
            material=gold,
            name="escutcheon_step",
        )
        # Valve body (vertical cylinder)
        valve.visual(
            Cylinder(radius=VALVE_BODY_R, length=VALVE_BODY_H),
            origin=Origin(
                xyz=(0.0, 0.0, VALVE_ESC_T1 + VALVE_ESC_T2 + VALVE_BODY_H / 2.0)
            ),
            material=gold,
            name="valve_body",
        )
        # Narrow seam ring at valve deck base
        valve.visual(
            Cylinder(radius=VALVE_ESC_R1 + 0.003, length=SEAM_THICK),
            origin=Origin(xyz=(0.0, 0.0, SEAM_THICK / 2.0)),
            material=seam_mat,
            name="valve_seam",
        )
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * VALVE_SPACING_X, 0.0, 0.0)),
        )

        # --- Cross handle on vertical stem ---
        handle = model.part(f"{side}_handle")

        # Stem (visible portion above valve body)
        handle.visual(
            Cylinder(radius=STEM_R, length=STEM_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_H / 2.0)),
            material=gold,
            name="stem",
        )
        # Knurled hub
        handle.visual(
            Cylinder(radius=HUB_R, length=HUB_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_H + HUB_H / 2.0)),
            material=gold,
            name="hub",
        )
        # Domed cap
        handle.visual(
            Sphere(radius=0.008),
            origin=Origin(xyz=(0.0, 0.0, STEM_H + HUB_H + 0.006)),
            material=gold,
            name="cap",
        )

        # Arms at mid-hub height
        arm_z = STEM_H + HUB_H * 0.5
        # Arm along X
        handle.visual(
            Cylinder(radius=ARM_R, length=ARM_HALF * 2.0),
            origin=Origin(
                xyz=(0.0, 0.0, arm_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=gold,
            name="arm_x",
        )
        # Arm along Y
        handle.visual(
            Cylinder(radius=ARM_R, length=ARM_HALF * 2.0),
            origin=Origin(
                xyz=(0.0, 0.0, arm_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=gold,
            name="arm_y",
        )
        # Sphere tips at arm ends
        for name, pos in (
            ("tip_x_pos", (ARM_HALF, 0.0, arm_z)),
            ("tip_x_neg", (-ARM_HALF, 0.0, arm_z)),
            ("tip_y_pos", (0.0, ARM_HALF, arm_z)),
            ("tip_y_neg", (0.0, -ARM_HALF, arm_z)),
        ):
            handle.visual(
                Sphere(radius=ARM_R * 1.4),
                origin=Origin(xyz=pos),
                material=gold,
                name=name,
            )

        # Stem insert (seats into valve body bore, rotates with handle)
        handle.visual(
            Cylinder(radius=STEM_R, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=gold,
            name="stem_insert",
        )

        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, valve_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_panel")
    spout = object_model.get_part("spout")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # --- Both handle joints are revolute around vertical (Z) axis ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_vertical_axis",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2] - 1.0) < 1e-9,
            f"axis={ax}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_full_turn_range",
            lim is not None
            and abs(lim.lower + math.pi) < 1e-6
            and abs(lim.upper - math.pi) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # --- Spout is a rectangular waterfall channel ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    spout_dx = sx1 - sx0
    spout_dy = sy1 - sy0
    spout_dz = sz1 - sz0
    ctx.check(
        "spout_is_rectangular_channel",
        spout_dx >= SPOUT_CHANNEL_W - 0.005 and spout_dy >= SPOUT_CHANNEL_D - 0.005,
        f"spout extents: x={spout_dx:.3f}, y={spout_dy:.3f}",
    )
    ctx.check(
        "spout_rises_above_deck",
        spout_dz > 0.08,
        f"spout height={spout_dz:.3f}",
    )
    ctx.check(
        "spout_channel_wider_than_column",
        spout_dx > SPOUT_BASE_W,
        f"spout x={spout_dx:.3f} vs column={SPOUT_BASE_W}",
    )

    # --- Spout sits on deck surface (spout above deck, small gap/contact) ---
    ctx.expect_gap(
        spout, deck, axis="z",
        max_gap=0.003, max_penetration=0.003,
        name="spout_base_near_deck_surface",
    )

    # --- Valve placement: flanking the spout symmetrically ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "valves_flank_spout_symmetrically",
        abs(lv[0] + VALVE_SPACING_X) < 1e-6
        and abs(rv[0] - VALVE_SPACING_X) < 1e-6,
        f"left={lv}, right={rv}",
    )
    # Valves sit on deck surface
    ctx.expect_gap(
        left_valve, deck, axis="z",
        max_gap=0.003, max_penetration=0.003,
        name="left_valve_base_near_deck",
    )
    ctx.expect_gap(
        right_valve, deck, axis="z",
        max_gap=0.003, max_penetration=0.003,
        name="right_valve_base_near_deck",
    )

    # --- Handles mounted above valves, XY overlap ---
    ctx.expect_overlap(left_handle, left_valve, axes="xy", min_overlap=0.005)
    ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.005)

    # Handle stem insert seats into valve body (designed local penetration)
    ctx.expect_gap(
        left_handle, left_valve, axis="z",
        min_gap=-0.012, max_gap=0.003,
        name="left_handle_seated_on_valve",
    )

    # --- Narrow seams exist at all three deck bases ---
    ctx.check(
        "spout_has_deck_seam",
        spout.get_visual("spout_seam") is not None,
        "spout_seam visual missing",
    )
    ctx.check(
        "left_valve_has_deck_seam",
        left_valve.get_visual("valve_seam") is not None,
        "valve_seam visual missing",
    )
    ctx.check(
        "right_valve_has_deck_seam",
        right_valve.get_visual("valve_seam") is not None,
        "valve_seam visual missing",
    )

    # --- Pose: handle rotates in horizontal plane around vertical axis ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_dx = rot_aabb[1][0] - rot_aabb[0][0]
        rot_dy = rot_aabb[1][1] - rot_aabb[0][1]
        ctx.check(
            "left_handle_rotates_in_horizontal_plane",
            rot_dx > 0.03 and rot_dy > 0.03,
            f"rotated extents: x={rot_dx:.3f}, y={rot_dy:.3f}",
        )
        cen = ctx.part_world_position(left_handle)
        assert cen is not None
        ctx.check(
            "left_handle_stays_on_valve_axis",
            abs(cen[0] + VALVE_SPACING_X) < 1e-6,
            f"handle x={cen[0]:.4f}, expected {-VALVE_SPACING_X}",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.005)
        # Handle arms should clear the spout
        ctx.expect_gap(
            right_handle, spout, axis="x",
            min_gap=0.005,
            name="right_handle_clears_spout_at_90deg",
        )

    # --- Deck panel top at z=0 ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_top_near_z_zero",
        abs(deck_aabb[1][2]) < 0.002,
        f"deck zmax={deck_aabb[1][2]:.4f}",
    )

    # --- Overall width ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert lh_aabb is not None and rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_reasonable",
        0.22 <= total_w <= 0.35,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- Intentional embedding: stem inserts seat into valve body bores ---
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem_insert"),
        elem_b=left_valve.get_visual("valve_body"),
        reason="handle stem insert seats inside the valve body bore and rotates with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem_insert"),
        elem_b=right_valve.get_visual("valve_body"),
        reason="handle stem insert seats inside the valve body bore and rotates with the handle",
    )

    return ctx.report()


object_model = build_object_model()
