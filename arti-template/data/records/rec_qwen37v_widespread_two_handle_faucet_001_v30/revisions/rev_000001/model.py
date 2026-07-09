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
# Deck-mounted on a horizontal counter surface.
#
# Frame conventions:
#   - The deck surface is the horizontal XY plane at z = 0.
#   - The deck slab occupies z < 0 (below counter).
#   - Z is up; the spout and handles project above the deck.
#   - The spout projects along +Y (toward the user).
#   - Left handle at -X, right handle at +X.
# ---------------------------------------------------------------------------

# Layout
HANDLE_SPACING_X = 0.10  # handle centers at x = +/- 0.10

# Deck (counter surface)
DECK_W = 0.40
DECK_D = 0.18
DECK_T = 0.025

# Spout
SPOUT_TUBE_R = 0.014  # outer radius (~0.028 m diameter)
SPOUT_BORE_R = 0.010  # inner bore radius (visible at outlet)
SPOUT_RISER_H = 0.14  # vertical rise before the curve
SPOUT_BEND_R = 0.055  # bend radius of the forward-and-down curve
SPOUT_FORWARD = 0.10  # forward reach after the bend
SPOUT_DROP_Z = -0.06  # outlet end below the bend apex

# Spout base escutcheon
SPOUT_BASE_R1, SPOUT_BASE_T1 = 0.028, 0.008
SPOUT_BASE_R2, SPOUT_BASE_T2 = 0.022, 0.006

# Valve assemblies
VALVE_BASE_R1, VALVE_BASE_T1 = 0.026, 0.007
VALVE_BASE_R2, VALVE_BASE_T2 = 0.020, 0.005
VALVE_BODY_R = 0.013
VALVE_BODY_H = 0.035  # height of valve body above base

# Cross handle
HANDLE_ROD_R = 0.004
HANDLE_ROD_LEN = 0.090  # tip-to-tip
HUB_R = 0.012
HUB_H = 0.018
KNURL_R = 0.0135
STEM_R = 0.006
STEM_H = 0.014

# Seam ring dimensions
SEAM_R = 0.030  # slightly larger than base escutcheons
SEAM_T = 0.0015  # very thin

# Asymmetric rest angles for cross handles (radians)
LEFT_HANDLE_REST_ANGLE = math.radians(25)  # left handle rotated 25° CW from reference
RIGHT_HANDLE_REST_ANGLE = math.radians(-15)  # right handle rotated 15° CCW

# Computed by build_object_model() for hollow-bore verification in run_tests().
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_solid() -> cq.Workplane:
    """Spout in its local frame: base on XY plane at z=0, rising vertically,
    curving forward (+Y) and downward to an open hollow outlet."""
    # Path in the YZ plane: rise vertically, arc forward and down
    rise_end_z = SPOUT_RISER_H
    # Arc from top of riser, bending forward and down
    arc_mid = (SPOUT_BEND_R * 0.7, rise_end_z - SPOUT_BEND_R * 0.3)
    arc_end = (SPOUT_BEND_R + SPOUT_FORWARD * 0.3, rise_end_z - SPOUT_BEND_R * 0.85)
    outlet_end = (SPOUT_BEND_R + SPOUT_FORWARD, rise_end_z - SPOUT_BEND_R + SPOUT_DROP_Z)

    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, rise_end_z)
        .threePointArc(arc_mid, arc_end)
        .lineTo(outlet_end[0], outlet_end[1])
    )

    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -0.003)
        .lineTo(0.0, rise_end_z)
        .threePointArc(arc_mid, arc_end)
        .lineTo(outlet_end[0], outlet_end[1] - 0.003)
    )

    # Sweep tube along path - XY workplane has +Z normal, matching path start
    tube = cq.Workplane("XY").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    # Base escutcheon (stepped)
    base_outer = cq.Workplane("XY").circle(SPOUT_BASE_R1).extrude(SPOUT_BASE_T1)
    base_step = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_BASE_T1)
        .circle(SPOUT_BASE_R2)
        .extrude(SPOUT_BASE_T2)
    )

    unbored = tube.union(base_outer).union(base_step)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle central hub: axis +Z, bottom face at z=0,
    with a knurled (faceted) middle band and a domed top cap."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_H)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.010)
    )
    dome = cq.Workplane("XY").workplane(offset=HUB_H - 0.002).sphere(0.011)
    return hub.union(knurl).union(dome)


def _add_valve_visuals(valve, gold) -> None:
    """Stepped base escutcheon + cylindrical valve body, axis +Z from
    the deck plane (valve frame origin sits on the deck surface)."""
    valve.visual(
        Cylinder(radius=VALVE_BASE_R1, length=VALVE_BASE_T1),
        origin=Origin(xyz=(0.0, 0.0, VALVE_BASE_T1 / 2.0)),
        material=gold,
        name="escutcheon_base",
    )
    valve.visual(
        Cylinder(radius=VALVE_BASE_R2, length=VALVE_BASE_T2),
        origin=Origin(xyz=(0.0, 0.0, VALVE_BASE_T1 + VALVE_BASE_T2 / 2.0)),
        material=gold,
        name="escutcheon_step",
    )
    valve.visual(
        Cylinder(radius=VALVE_BODY_R, length=VALVE_BODY_H),
        origin=Origin(xyz=(0.0, 0.0, VALVE_BASE_T1 + VALVE_BASE_T2 + VALVE_BODY_H / 2.0)),
        material=gold,
        name="valve_body",
    )


def _add_handle_visuals(handle, hub_mesh, gold, rest_angle: float = 0.0) -> None:
    """Four-arm cross handle rotating around vertical (Z) axis.
    The cross arms lie in the horizontal XY plane.
    rest_angle rotates the whole cross assembly for asymmetric positioning."""
    # Stem turns with the handle; seats down into the valve body.
    valve_top_z = VALVE_BASE_T1 + VALVE_BASE_T2 + VALVE_BODY_H
    handle.visual(
        Cylinder(radius=STEM_R, length=STEM_H),
        origin=Origin(xyz=(0.0, 0.0, -STEM_H / 2.0)),
        material=gold,
        name="stem",
    )
    # Hub on top
    handle.visual(hub_mesh, material=gold, name="hub")

    # Spoke rod plane is at hub mid-height
    spoke_z = HUB_H / 2.0

    # Cross spokes in the horizontal plane, rotated by rest_angle
    cos_a = math.cos(rest_angle)
    sin_a = math.sin(rest_angle)
    half = HANDLE_ROD_LEN / 2.0

    # Spoke pair 1: along the rotated X direction
    dx1 = cos_a * half
    dy1 = sin_a * half
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, spoke_z), rpy=(0.0, math.pi / 2.0, rest_angle)),
        material=gold,
        name="spoke_pair_1",
    )
    # Spoke pair 2: perpendicular to pair 1
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, spoke_z), rpy=(math.pi / 2.0, 0.0, rest_angle)),
        material=gold,
        name="spoke_pair_2",
    )

    # Rounded spoke tips (4 tips)
    for name_suffix, angle_offset in (
        ("tip_a", 0.0),
        ("tip_b", math.pi),
        ("tip_c", math.pi / 2.0),
        ("tip_d", -math.pi / 2.0),
    ):
        a = rest_angle + angle_offset
        tx = half * math.cos(a)
        ty = half * math.sin(a)
        handle.visual(
            Sphere(radius=HANDLE_ROD_R),
            origin=Origin(xyz=(tx, ty, spoke_z)),
            material=gold,
            name=name_suffix,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    seam_dark = model.material("seam_gasket", rgba=(0.15, 0.12, 0.10, 1.0))
    deck_stone = model.material("counter_surface", rgba=(0.88, 0.86, 0.82, 1.0))

    # --- deck (root, counter surface) ---
    deck = model.part("deck")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T / 2.0)),
        material=deck_stone,
        name="slab",
    )

    # --- central spout (fixed to deck) ---
    spout = model.part("spout")
    spout.visual(mesh_from_cadquery(_build_spout_solid(), "spout"), material=gold, name="tube")
    # Seam ring at spout base
    spout.visual(
        Cylinder(radius=SEAM_R, length=SEAM_T),
        origin=Origin(xyz=(0.0, 0.0, -SEAM_T / 2.0)),
        material=seam_dark,
        name="deck_seam",
    )
    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- valve assemblies (fixed) and cross handles (revolute, vertical axis) ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")
    valve_top_z = VALVE_BASE_T1 + VALVE_BASE_T2 + VALVE_BODY_H

    handle_configs = [
        ("left", -1.0, LEFT_HANDLE_REST_ANGLE),
        ("right", 1.0, RIGHT_HANDLE_REST_ANGLE),
    ]

    for side, sx, rest_angle in handle_configs:
        valve = model.part(f"{side}_valve")
        _add_valve_visuals(valve, gold)
        # Seam ring at valve base
        valve.visual(
            Cylinder(radius=SEAM_R, length=SEAM_T),
            origin=Origin(xyz=(0.0, 0.0, -SEAM_T / 2.0)),
            material=seam_dark,
            name="deck_seam",
        )
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPACING_X, 0.0, 0.0)),
        )

        handle = model.part(f"{side}_cross_handle")
        _add_handle_visuals(handle, hub_mesh, gold, rest_angle=rest_angle)
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            # Joint frame at top of valve body; axis is vertical (Z).
            origin=Origin(xyz=(0.0, 0.0, valve_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout = object_model.get_part("spout")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_cross_handle")
    right_handle = object_model.get_part("right_cross_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # --- joint plan: two independent revolute cross handles, vertical axis ---
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

    # --- asymmetric rest angles: handles are angled differently ---
    ctx.check(
        "handles_asymmetrically_angled",
        abs(LEFT_HANDLE_REST_ANGLE - RIGHT_HANDLE_REST_ANGLE) > math.radians(10),
        f"left_rest={math.degrees(LEFT_HANDLE_REST_ANGLE):.1f}deg, "
        f"right_rest={math.degrees(RIGHT_HANDLE_REST_ANGLE):.1f}deg",
    )

    # --- narrow seams at all three deck bases ---
    spout_seam = spout.get_visual("deck_seam")
    left_seam = left_valve.get_visual("deck_seam")
    right_seam = right_valve.get_visual("deck_seam")
    ctx.check(
        "spout_has_deck_seam",
        spout_seam is not None,
        "spout missing deck_seam visual",
    )
    ctx.check(
        "left_valve_has_deck_seam",
        left_seam is not None,
        "left valve missing deck_seam visual",
    )
    ctx.check(
        "right_valve_has_deck_seam",
        right_seam is not None,
        "right valve missing deck_seam visual",
    )

    # --- intentional embedding: handle stems seat into valve body bores ---
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("valve_body"),
        reason="valve stem is seated inside the valve body bore and turns with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("valve_body"),
        reason="valve stem is seated inside the valve body bore and turns with the handle",
    )

    # --- spout geometry: hollow bore ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} m^3 vs unbored={SPOUT_UNBORED_VOLUME:.3e} m^3",
    )

    # --- spout rises above deck ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "spout_rises_above_deck",
        sz1 > 0.10,
        f"spout zmax={sz1:.3f}",
    )
    ctx.check(
        "spout_outlet_drops_below_apex",
        sz0 < sz1 - 0.05,
        f"spout zmin={sz0:.3f}, zmax={sz1:.3f}",
    )

    # --- valve placement: flanking the spout at x = +/-0.10 ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "valves_flank_spout",
        abs(lv[0] + HANDLE_SPACING_X) < 1e-6
        and abs(rv[0] - HANDLE_SPACING_X) < 1e-6,
        f"left={lv}, right={rv}",
    )

    # --- handles sit above deck surface ---
    ctx.expect_gap(left_handle, deck, axis="z", min_gap=0.01)
    ctx.expect_gap(right_handle, deck, axis="z", min_gap=0.01)

    # --- cross handle size: about 0.09 m tip to tip ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    assert lh_aabb is not None
    (hx0, hy0, hz0), (hx1, hy1, hz1) = lh_aabb
    ctx.check(
        "cross_handle_about_0p09_tip_to_tip",
        0.080 <= (hx1 - hx0) <= 0.105 and 0.080 <= (hy1 - hy0) <= 0.105,
        f"handle extents x={hx1 - hx0:.3f}, y={hy1 - hy0:.3f}",
    )

    # --- handle overlap with valve (mounted, not floating) ---
    ctx.expect_overlap(left_handle, left_valve, axes="xy", min_overlap=0.008)
    ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.008)

    # --- overall width about 0.30 m ---
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.26 <= total_w <= 0.34,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- rotation proof: turning handle changes spoke orientation ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        # At q=pi/4, the AABB in XY should still encompass the cross
        rot_dx = rot_aabb[1][0] - rot_aabb[0][0]
        rot_dy = rot_aabb[1][1] - rot_aabb[0][1]
        ctx.check(
            "left_handle_rotates_in_horizontal_plane",
            rot_dx > 0.06 and rot_dy > 0.06,
            f"rotated AABB: dx={rot_dx:.3f}, dy={rot_dy:.3f}",
        )
        # Handle center stays on valve axis
        cen = ctx.part_world_position(left_handle)
        assert cen is not None
        ctx.check(
            "left_handle_stays_on_valve_axis",
            abs(cen[0] + HANDLE_SPACING_X) < 0.002 and abs(cen[1]) < 0.002,
            f"handle origin={cen}",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.008)

    # --- deck grounded ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_grounded_at_z_zero",
        abs(deck_aabb[0][2] + DECK_T) < 1e-6,
        f"deck zmin={deck_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
