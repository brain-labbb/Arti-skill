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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Widespread two-handle deck-mounted bathroom faucet in polished gold brass.
#
# Frame conventions:
#   - The deck/countertop is the horizontal XY plane at z = 0.
#   - Three units mount vertically through the deck: left handle, central
#     spout, right handle, spaced about 0.10 m apart.
#   - Faucet hardware projects upward (+Z) from the deck surface.
#   - Lever handles rotate forward-back about horizontal axes parallel to Y.
# ---------------------------------------------------------------------------

# Layout
DECK_W = 0.38
DECK_D = 0.18
DECK_T = 0.020
UNIT_SPACING_X = 0.10  # centers at x = -0.10, 0, +0.10

# Deck seams (narrow visual seam rings at base interfaces)
SEAM_RING_H = 0.001  # thin seam ring height
SEAM_RING_OVERHANG = 0.002  # how much wider the seam ring is vs base

# Spout - rectangular waterfall channel
SPOUT_BASE_R = 0.018  # base escutcheon radius (within channel footprint)
SPOUT_BASE_H = 0.015  # base height
CHANNEL_W = 0.060  # channel width (X)
CHANNEL_D = 0.040  # channel depth (Y)
CHANNEL_H = 0.035  # channel height (Z)
CHANNEL_WALL_T = 0.004  # channel wall thickness
OUTLET_W = 0.040  # outlet opening width
OUTLET_D = 0.025  # outlet opening depth

# Valve assemblies
VALVE_BASE_R = 0.026
VALVE_BASE_H = 0.015
VALVE_BODY_R = 0.014
VALVE_BODY_H = 0.025

# Lever handles
HANDLE_BASE_R = 0.016
HANDLE_BASE_H = 0.008
HANDLE_SHAFT_R = 0.008
HANDLE_SHAFT_H = 0.020
LEVER_W = 0.012  # lever width
LEVER_H = 0.008  # lever height
LEVER_LEN = 0.065  # lever length from pivot


def _build_spout_channel() -> cq.Workplane:
    """Rectangular waterfall channel with hollow outlet in local frame.
    Base sits on deck (z=0), channel projects upward."""
    # Base escutcheon cylinder
    base = cq.Workplane("XY").circle(SPOUT_BASE_R).extrude(SPOUT_BASE_H)
    
    # Outer channel box
    outer = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_BASE_H)
        .rect(CHANNEL_W, CHANNEL_D)
        .extrude(CHANNEL_H)
    )
    
    # Inner hollow (cut from top, leaving walls and floor)
    inner_w = CHANNEL_W - 2 * CHANNEL_WALL_T
    inner_d = CHANNEL_D - 2 * CHANNEL_WALL_T
    inner_cut = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_BASE_H + CHANNEL_WALL_T)
        .rect(inner_w, inner_d)
        .extrude(CHANNEL_H - CHANNEL_WALL_T + 0.001)
    )
    
    # Outlet opening (cut through front wall at bottom)
    outlet_cut = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_BASE_H + CHANNEL_WALL_T)
        .center(0, -CHANNEL_D / 2.0)
        .rect(OUTLET_W, OUTLET_D)
        .extrude(CHANNEL_H - 2 * CHANNEL_WALL_T)
    )
    
    solid = base.union(outer).cut(inner_cut).cut(outlet_cut)
    return solid


def _build_lever_handle() -> cq.Workplane:
    """Lever handle in local frame: base cylinder, shaft, and elongated lever.
    Joint frame at shaft top, lever extends along +X."""
    # Base plate
    base = cq.Workplane("XY").circle(HANDLE_BASE_R).extrude(HANDLE_BASE_H)
    
    # Vertical shaft
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=HANDLE_BASE_H)
        .circle(HANDLE_SHAFT_R)
        .extrude(HANDLE_SHAFT_H)
    )
    
    # Horizontal lever arm (extends from shaft top along +X)
    lever = (
        cq.Workplane("XY")
        .workplane(offset=HANDLE_BASE_H + HANDLE_SHAFT_H)
        .center(LEVER_LEN / 2.0, 0)
        .rect(LEVER_LEN, LEVER_W)
        .extrude(LEVER_H)
    )
    
    # Rounded lever tip
    tip = (
        cq.Workplane("XY")
        .workplane(offset=HANDLE_BASE_H + HANDLE_SHAFT_H)
        .center(LEVER_LEN, 0)
        .circle(LEVER_W / 2.0)
        .extrude(LEVER_H)
    )
    
    return base.union(shaft).union(lever).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    deck_white = model.material("deck_white", rgba=(0.93, 0.93, 0.90, 1.0))

    # --- deck/countertop (root, mounting substrate) ---
    deck = model.part("deck")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, DECK_T / 2.0)),
        material=deck_white,
        name="countertop",
    )

    # --- central spout (fixed) ---
    spout = model.part("spout")
    # Dark seam ring at deck interface
    seam_dark = model.material("seam_dark", rgba=(0.15, 0.15, 0.15, 1.0))
    spout.visual(
        Cylinder(radius=SPOUT_BASE_R + SEAM_RING_OVERHANG, length=SEAM_RING_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_RING_H / 2.0)),
        material=seam_dark,
        name="deck_seam",
    )
    spout.visual(
        mesh_from_cadquery(_build_spout_channel(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, SEAM_RING_H)),
        material=gold,
        name="channel",
    )
    # Spout base contacts deck directly
    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, DECK_T)),
    )

    # --- valve assemblies (fixed) and lever handles (revolute) ---
    lever_mesh = mesh_from_cadquery(_build_lever_handle(), "lever_handle")
    
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        
        # Dark seam ring at deck interface
        valve.visual(
            Cylinder(radius=VALVE_BASE_R + SEAM_RING_OVERHANG, length=SEAM_RING_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_RING_H / 2.0)),
            material=seam_dark,
            name="deck_seam",
        )
        
        # Base escutcheon
        valve.visual(
            Cylinder(radius=VALVE_BASE_R, length=VALVE_BASE_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_RING_H + VALVE_BASE_H / 2.0)),
            material=gold,
            name="base",
        )
        
        # Valve body cylinder
        valve.visual(
            Cylinder(radius=VALVE_BODY_R, length=VALVE_BODY_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_RING_H + VALVE_BASE_H + VALVE_BODY_H / 2.0)),
            material=gold,
            name="body",
        )
        
        # Valve base contacts deck directly
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * UNIT_SPACING_X, 0.0, DECK_T)),
        )

        handle = model.part(f"{side}_handle")
        handle.visual(lever_mesh, material=gold, name="lever")
        
        # Lever rotates forward-back about horizontal Y axis
        # Joint frame at valve body top so handle base plate sits on it
        joint_z = SEAM_RING_H + VALVE_BASE_H + VALVE_BODY_H
        model.articulation(
            f"{side}_handle_pivot",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, joint_z)),
            axis=(0.0, 1.0, 0.0),  # horizontal axis parallel to Y
            motion_limits=MotionLimits(
                effort=5.0, 
                velocity=3.0, 
                lower=-math.pi / 3.0,  # -60 degrees (backward)
                upper=math.pi / 3.0    # +60 degrees (forward)
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout = object_model.get_part("spout")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_joint = object_model.get_articulation("left_handle_pivot")
    right_joint = object_model.get_articulation("right_handle_pivot")

    # --- joint plan: two independent revolute lever handles, axis horizontal
    # (parallel to Y), range -60..+60 deg for forward-back rotation ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_horizontal_y",
            abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
            f"axis={ax}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_lever_range",
            lim is not None
            and abs(lim.lower + math.pi / 3.0) < 1e-6
            and abs(lim.upper - math.pi / 3.0) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # --- spout geometry: rectangular waterfall channel with hollow outlet ---
    channel_elem = spout.get_visual("channel")
    spout_aabb = ctx.part_element_world_aabb(spout, elem="channel")
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    
    ctx.check(
        "spout_is_rectangular_channel",
        abs((sx1 - sx0) - CHANNEL_W) < 0.005 and abs((sy1 - sy0) - CHANNEL_D) < 0.005,
        f"channel dims x={sx1 - sx0:.3f}, y={sy1 - sy0:.3f} (expected {CHANNEL_W}, {CHANNEL_D})",
    )
    
    ctx.check(
        "spout_channel_height",
        abs((sz1 - sz0) - (SPOUT_BASE_H + CHANNEL_H)) < 0.005,
        f"channel height={sz1 - sz0:.3f}",
    )
    
    # Spout is centered on deck
    spout_pos = ctx.part_world_position(spout)
    assert spout_pos is not None
    ctx.check(
        "spout_centered_on_deck",
        abs(spout_pos[0]) < 0.001 and abs(spout_pos[1]) < 0.001,
        f"spout center={spout_pos}",
    )

    # --- deck seams: narrow visual seam rings at all three base interfaces ---
    # Verify seam rings exist as distinct visuals
    ctx.check(
        "spout_has_deck_seam",
        spout.get_visual("deck_seam") is not None,
        "spout missing deck_seam visual",
    )
    for valve, side in [(left_valve, "left"), (right_valve, "right")]:
        ctx.check(
            f"{side}_valve_has_deck_seam",
            valve.get_visual("deck_seam") is not None,
            f"{side} valve missing deck_seam visual",
        )

    # --- valve placement: flanking the spout at x = +/-0.10 ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "valves_flank_spout_symmetrically",
        abs(lv[0] + UNIT_SPACING_X) < 1e-6
        and abs(rv[0] - UNIT_SPACING_X) < 1e-6
        and abs(lv[1]) < 1e-6
        and abs(rv[1]) < 1e-6,
        f"left={lv}, right={rv}",
    )

    # --- lever handles extend horizontally from valve tops ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert lh_aabb is not None and rh_aabb is not None
    
    ctx.check(
        "left_lever_extends_horizontally",
        (lh_aabb[1][0] - lh_aabb[0][0]) > LEVER_LEN * 0.8,
        f"left lever x extent={lh_aabb[1][0] - lh_aabb[0][0]:.3f}",
    )
    
    ctx.check(
        "right_lever_extends_horizontally",
        (rh_aabb[1][0] - rh_aabb[0][0]) > LEVER_LEN * 0.8,
        f"right lever x extent={rh_aabb[1][0] - rh_aabb[0][0]:.3f}",
    )

    # --- overall width about 0.30 m across the three units ---
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.26 <= total_w <= 0.32,
        f"handle-to-handle width={total_w:.3f}",
    )

    # --- lever rotation proves forward-back motion ---
    rest_aabb = ctx.part_world_aabb(left_handle)
    assert rest_aabb is not None
    rest_z_extent = rest_aabb[1][2] - rest_aabb[0][2]
    
    with ctx.pose({left_joint: math.pi / 6.0}):  # 30 degrees forward
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_z_extent = rot_aabb[1][2] - rot_aabb[0][2]
        
        # When lever rotates forward, the Z extent should change (lever tilts)
        ctx.check(
            "left_lever_rotates_forward",
            abs(rot_z_extent - rest_z_extent) > 0.005,  # Z extent changes when tilted
            f"rest z extent={rest_z_extent:.3f}, rotated z extent={rot_z_extent:.3f}",
        )
        
        # Lever still attached to valve (overlap in XY)
        ctx.expect_overlap(left_handle, left_valve, axes="xy", min_overlap=0.005)

    with ctx.pose({right_joint: -math.pi / 6.0}):  # 30 degrees backward
        # Lever still attached to valve
        ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.005)

    # --- deck panel grounded ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_panel_grounded",
        abs(deck_aabb[0][2]) < 1e-6,
        f"deck zmin={deck_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
