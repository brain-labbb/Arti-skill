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
# Widespread deck-mounted bathroom faucet in polished gold brass.
#
# Frame conventions:
#   - The deck/counter is the horizontal XY plane at z = 0.
#   - The faucet projects upward from the deck along +Z.
#   - Three units mount through the deck: left handle, central spout, right handle.
#   - Spout is a rectangular waterfall channel with hinged aerator at outlet.
#   - Handles are cross-style on pedestals with decorative ring ridges.
# ---------------------------------------------------------------------------

# Layout
HANDLE_SPACING_X = 0.10  # handle centers at x = +/- 0.10
DECK_THICKNESS = 0.012

# Deck plate (mounting substrate)
DECK_W = 0.36
DECK_D = 0.18
DECK_T = DECK_THICKNESS

# Spout - rectangular waterfall channel
SPOUT_BASE_W = 0.050  # width of spout base
SPOUT_BASE_D = 0.040  # depth of spout base
SPOUT_BASE_H = 0.080  # height of spout base pedestal
SPOUT_CHANNEL_W = 0.040  # channel width
SPOUT_CHANNEL_D = 0.120  # channel length (projection)
SPOUT_CHANNEL_H = 0.025  # channel wall height
SPOUT_CHANNEL_WALL_T = 0.005  # channel wall thickness

# Aerator (hinged at spout outlet)
AERATOR_W = 0.038
AERATOR_D = 0.020
AERATOR_T = 0.008


# Handle assemblies
HANDLE_PEDESTAL_R = 0.022
HANDLE_PEDESTAL_H = 0.050
HANDLE_RING_R = 0.025
HANDLE_RING_T = 0.004
HANDLE_CROSS_R = 0.004
HANDLE_CROSS_LEN = 0.090
HUB_R = 0.012
HUB_H = 0.020

# Seam grooves (visual detail at deck base)
SEAM_W = 0.002
SEAM_DEPTH = 0.003


def _build_spout_channel() -> cq.Workplane:
    """Rectangular waterfall channel with open top, base pedestal, and hollow bore."""
    # Base pedestal
    base = (
        cq.Workplane("XY")
        .box(SPOUT_BASE_W, SPOUT_BASE_D, SPOUT_BASE_H, centered=(True, True, False))
    )
    
    # Channel walls (U-shaped channel)
    channel_outer = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_BASE_H)
        .box(SPOUT_CHANNEL_W, SPOUT_CHANNEL_D, SPOUT_CHANNEL_H, centered=(True, False, False))
    )
    
    # Cut out the channel interior
    channel_inner_w = SPOUT_CHANNEL_W - 2 * SPOUT_CHANNEL_WALL_T
    channel_inner_d = SPOUT_CHANNEL_D - SPOUT_CHANNEL_WALL_T
    channel_cut = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_BASE_H + SPOUT_CHANNEL_WALL_T)
        .box(channel_inner_w, channel_inner_d, SPOUT_CHANNEL_H, centered=(True, False, False))
    )
    
    # Water bore through the base
    bore_diameter = 0.015
    bore = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .circle(bore_diameter / 2.0)
        .extrude(SPOUT_CHANNEL_D)
    )
    
    solid = base.union(channel_outer).cut(channel_cut).cut(bore)
    return solid


def _build_aerator() -> cq.Workplane:
    """Small rectangular aerator plate that hinges at the spout outlet."""
    aerator = (
        cq.Workplane("XY")
        .box(AERATOR_W, AERATOR_D, AERATOR_T, centered=(True, True, False))
    )
    # Add some mesh/screen detail (small holes pattern)
    hole_r = 0.0015
    hole_spacing = 0.004
    for i in range(-4, 5):
        for j in range(-2, 3):
            hx = i * hole_spacing
            hy = j * hole_spacing
            if abs(hx) < AERATOR_W / 2.0 - 0.002 and abs(hy) < AERATOR_D / 2.0 - 0.002:
                hole = (
                    cq.Workplane("XY")
                    .workplane(offset=AERATOR_T / 2.0)
                    .center(hx, hy)
                    .circle(hole_r)
                    .extrude(AERATOR_T)
                )
                aerator = aerator.cut(hole)
    return aerator


def _build_handle_pedestal() -> cq.Workplane:
    """Handle pedestal with decorative ring ridges."""
    # Main pedestal cylinder
    pedestal = (
        cq.Workplane("XY")
        .circle(HANDLE_PEDESTAL_R)
        .extrude(HANDLE_PEDESTAL_H)
    )
    
    # Add decorative ring ridges (3 rings at different heights)
    ring_positions = [0.015, 0.025, 0.035]
    for z_pos in ring_positions:
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z_pos)
            .circle(HANDLE_RING_R)
            .extrude(HANDLE_RING_T)
        )
        pedestal = pedestal.union(ring)
    
    return pedestal


def _build_cross_handle() -> cq.Workplane:
    """Four-arm cross handle with central hub."""
    # Central hub
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_H)
    
    # Four spoke arms (two crossing rods)
    arm1 = (
        cq.Workplane("XY")
        .workplane(offset=HUB_H / 2.0)
        .box(HANDLE_CROSS_LEN, 2 * HANDLE_CROSS_R, 2 * HANDLE_CROSS_R, centered=(True, True, True))
    )
    arm2 = (
        cq.Workplane("XY")
        .workplane(offset=HUB_H / 2.0)
        .box(2 * HANDLE_CROSS_R, HANDLE_CROSS_LEN, 2 * HANDLE_CROSS_R, centered=(True, True, True))
    )
    
    # Rounded tips
    tips = []
    half = HANDLE_CROSS_LEN / 2.0
    for dx, dy in [(half, 0), (-half, 0), (0, half), (0, -half)]:
        tip = (
            cq.Workplane("XY")
            .workplane(offset=HUB_H / 2.0)
            .center(dx, dy)
            .sphere(HANDLE_CROSS_R * 1.2)
        )
        tips.append(tip)
    
    solid = hub.union(arm1).union(arm2)
    for tip in tips:
        solid = solid.union(tip)
    
    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_deck_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    deck_gray = model.material("deck_granite", rgba=(0.35, 0.35, 0.38, 1.0))
    seam_dark = model.material("seam_shadow", rgba=(0.15, 0.15, 0.15, 1.0))

    # --- deck plate (root, mounting surface) ---
    deck = model.part("deck_plate")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, DECK_T / 2.0)),
        material=deck_gray,
        name="deck_surface",
    )

    # --- central spout with waterfall channel ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_channel(), "spout_channel"),
        material=gold,
        name="waterfall_channel",
    )
    
    # Seam groove at spout base
    spout.visual(
        Box((SPOUT_BASE_W + 0.004, SPOUT_BASE_D + 0.004, SEAM_DEPTH)),
        origin=Origin(xyz=(0.0, 0.0, SEAM_DEPTH / 2.0)),
        material=seam_dark,
        name="spout_seam",
    )
    
    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, DECK_T)),
    )

    # --- aerator (hinged at spout outlet) ---
    aerator = model.part("aerator")
    # Offset so the inner edge sits just past the channel outlet end
    aerator.visual(
        mesh_from_cadquery(_build_aerator(), "aerator_plate"),
        origin=Origin(xyz=(0.0, AERATOR_D / 2.0, 0.0)),
        material=gold,
        name="aerator_mesh",
    )
    
    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        # Hinge at the outlet end of the channel, axis across the width
        origin=Origin(
            xyz=(0.0, SPOUT_CHANNEL_D, SPOUT_BASE_H + SPOUT_CHANNEL_H - AERATOR_T)
        ),
        axis=(-1.0, 0.0, 0.0),  # rotates around -X so positive angle drops the aerator downward
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.5, lower=0.0, upper=math.pi / 3.0  # 0 to 60 degrees down
        ),
    )

    # --- handle assemblies (left and right) ---
    pedestal_mesh = mesh_from_cadquery(_build_handle_pedestal(), "handle_pedestal")
    cross_mesh = mesh_from_cadquery(_build_cross_handle(), "cross_handle")
    
    for side, sx in [("left", -1.0), ("right", 1.0)]:
        # Valve body (fixed to deck)
        valve = model.part(f"{side}_valve")
        valve.visual(
            pedestal_mesh,
            material=gold,
            name="pedestal",
        )
        
        # Seam groove at handle base
        valve.visual(
            Cylinder(radius=HANDLE_PEDESTAL_R + 0.003, length=SEAM_DEPTH),
            origin=Origin(xyz=(0.0, 0.0, SEAM_DEPTH / 2.0)),
            material=seam_dark,
            name=f"{side}_handle_seam",
        )
        
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPACING_X, 0.0, DECK_T)),
        )

        # Cross handle (revolute on valve)
        handle = model.part(f"{side}_cross_handle")
        handle.visual(
            cross_mesh,
            material=gold,
            name="handle_cross",
        )
        
        # Stem that seats into the valve
        handle.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=gold,
            name="stem",
        )
        
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, HANDLE_PEDESTAL_H + HUB_H / 2.0)),
            axis=(0.0, 0.0, 1.0),  # vertical axis rotation
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_plate")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_cross_handle")
    right_handle = object_model.get_part("right_cross_handle")
    
    aerator_joint = object_model.get_articulation("aerator_hinge")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # --- verify at least one non-fixed joint (aerator hinge) ---
    ctx.check(
        "aerator_hinge_is_revolute",
        str(aerator_joint.joint_type).lower().endswith("revolute"),
        f"type={aerator_joint.joint_type}",
    )
    
    # Aerator hinge axis should be horizontal across spout width (-X axis for downward pivot)
    ax = aerator_joint.axis
    ctx.check(
        "aerator_hinge_axis_across_spout",
        abs(ax[0] + 1.0) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2]) < 1e-9,
        f"axis={ax}",
    )
    
    # Aerator hinge range: 0 to 60 degrees downward
    lim = aerator_joint.motion_limits
    ctx.check(
        "aerator_hinge_range_0_to_60_deg",
        lim is not None and abs(lim.lower) < 1e-6 and abs(lim.upper - math.pi / 3.0) < 1e-6,
        f"limits=({lim.lower}, {lim.upper})",
    )

    # --- handle joints remain revolute ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )

    # --- spout is rectangular waterfall channel (not cylindrical) ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    spout_dx = sx1 - sx0
    spout_dy = sy1 - sy0
    
    ctx.check(
        "spout_is_rectangular_channel",
        0.040 <= spout_dx <= 0.060 and 0.12 <= spout_dy <= 0.16,
        f"spout size=({spout_dx:.3f}, {spout_dy:.3f})",
    )
    
    ctx.check(
        "spout_height_reasonable",
        0.08 <= (sz1 - sz0) <= 0.11,
        f"spout height={sz1 - sz0:.3f}",
    )

    # --- aerator pivots downward when hinge is actuated ---
    aerator_aabb_closed = ctx.part_world_aabb(aerator)
    assert aerator_aabb_closed is not None
    
    with ctx.pose({aerator_joint: math.pi / 6.0}):  # 30 degrees
        aerator_aabb_open = ctx.part_world_aabb(aerator)
        assert aerator_aabb_open is not None
        
        # Aerator minimum z should drop when hinged downward
        ctx.check(
            "aerator_pivots_downward",
            aerator_aabb_open[0][2] < aerator_aabb_closed[0][2] - 0.001,
            f"closed zmin={aerator_aabb_closed[0][2]:.4f}, open zmin={aerator_aabb_open[0][2]:.4f}",
        )
    
    ctx.check(
        "aerator_exists_and_mounted",
        aerator is not None,
        "aerator part not found",
    )

    # --- decorative ring ridges on handle pedestals ---
    # Check that pedestals have reasonable size and are present
    lv_aabb = ctx.part_world_aabb(left_valve)
    rv_aabb = ctx.part_world_aabb(right_valve)
    assert lv_aabb is not None and rv_aabb is not None
    
    ctx.check(
        "handle_pedestals_present",
        0.040 <= (lv_aabb[1][2] - lv_aabb[0][2]) <= 0.060,
        f"left pedestal height={lv_aabb[1][2] - lv_aabb[0][2]:.3f}",
    )

    # --- seams at all three deck bases ---
    # Seams are visual elements, check they exist
    spout_seam = spout.get_visual("spout_seam")
    left_seam = left_valve.get_visual("left_handle_seam")
    right_seam = right_valve.get_visual("right_handle_seam")
    
    ctx.check(
        "deck_base_seams_present",
        spout_seam is not None and left_seam is not None and right_seam is not None,
        "one or more seam visuals missing",
    )

    # --- three-piece widespread layout ---
    lv_pos = ctx.part_world_position(left_valve)
    rv_pos = ctx.part_world_position(right_valve)
    spout_pos = ctx.part_world_position(spout)
    
    assert lv_pos is not None and rv_pos is not None and spout_pos is not None
    
    ctx.check(
        "widespread_three_piece_layout",
        abs(lv_pos[0] + HANDLE_SPACING_X) < 0.005
        and abs(rv_pos[0] - HANDLE_SPACING_X) < 0.005
        and abs(spout_pos[0]) < 0.005,
        f"positions: left={lv_pos[0]:.3f}, spout={spout_pos[0]:.3f}, right={rv_pos[0]:.3f}",
    )

    # --- overall width about 0.30 m ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert lh_aabb is not None and rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    
    ctx.check(
        "overall_width_about_0p30",
        0.28 <= total_w <= 0.32,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- handles rotate about vertical axis ---
    with ctx.pose({left_joint: math.pi / 2.0}):
        lh_rot_aabb = ctx.part_world_aabb(left_handle)
        assert lh_rot_aabb is not None
        # Cross handle should still be roughly same size after 90 deg rotation
        rot_dx = lh_rot_aabb[1][0] - lh_rot_aabb[0][0]
        ctx.check(
            "left_handle_rotates_about_vertical_axis",
            0.080 <= rot_dx <= 0.115,
            f"rotated handle x extent={rot_dx:.3f}",
        )

    # --- deck plate grounded at z=0 ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_plate_grounded",
        abs(deck_aabb[0][2]) < 1e-6,
        f"deck zmin={deck_aabb[0][2]:.4f}",
    )

    # --- handle stems seat into valve bodies (intentional overlap) ---
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("pedestal"),
        reason="handle stem seats into valve pedestal and rotates with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("pedestal"),
        reason="handle stem seats into valve pedestal and rotates with the handle",
    )

    return ctx.report()


object_model = build_object_model()
