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
# Deck-mounted: three units on a horizontal countertop surface.
#
# Frame conventions:
#   - Deck top surface at z = DECK_T (the horizontal mounting plane).
#   - Spout at center (x=0), projects forward along +Y.
#   - Left valve at x = -VALVE_SPACING, right at x = +VALVE_SPACING.
#   - All units rise along +Z from the deck.
# ---------------------------------------------------------------------------

# Layout
DECK_W = 0.38
DECK_D = 0.12
DECK_T = 0.025
VALVE_SPACING = 0.10

# Spout
SPOUT_TUBE_R = 0.015
SPOUT_BORE_R = 0.011
SPOUT_RISE = 0.15
SPOUT_REACH_Y = 0.10
SPOUT_DROP_Z = 0.08
SPOUT_OUTLET_EXT = 0.020
SPOUT_BASE_R = 0.022
SPOUT_BASE_H = 0.035

# Valve assemblies
VALVE_ESC_R = 0.030
VALVE_ESC_H = 0.012
VALVE_BODY_R = 0.014
VALVE_BODY_H = 0.030

# Cross handle
HANDLE_ROD_R = 0.004
HANDLE_ROD_LEN = 0.100
HUB_R = 0.012
HUB_H = 0.022
KNURL_R = 0.0135
STEM_R = 0.006
STEM_LEN = 0.014

# Aerator
AERATOR_R = 0.013
AERATOR_H = 0.015
AERATOR_BORE_R = 0.009

# Seam rings
SEAM_WIDTH = 0.003
SEAM_HEIGHT = 0.001

# Asymmetric handle rest angles (visibly different but balanced)
LEFT_HANDLE_ANGLE = math.radians(30)
RIGHT_HANDLE_ANGLE = math.radians(-20)

# Computed for hollow-bore verification
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0

# Outlet position (computed for aerator joint)
OUTLET_Y = SPOUT_REACH_Y
OUTLET_Z = SPOUT_RISE - SPOUT_DROP_Z - SPOUT_OUTLET_EXT


def _build_spout_solid() -> cq.Workplane:
    """Spout in local frame: origin at deck surface, tube rises along +Z,
    curves forward along +Y and drops to an open hollow outlet."""
    curve_start_z = SPOUT_RISE
    curve_end_y = SPOUT_REACH_Y
    curve_end_z = SPOUT_RISE - SPOUT_DROP_Z
    outlet_end_z = curve_end_z - SPOUT_OUTLET_EXT

    # Mid point for smooth gooseneck arc
    mid_y = curve_end_y * 0.55
    mid_z = curve_start_z - (curve_start_z - curve_end_z) * 0.18

    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, curve_start_z)
        .threePointArc((mid_y, mid_z), (curve_end_y, curve_end_z))
        .lineTo(curve_end_y, outlet_end_z)
    )

    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -0.004)
        .lineTo(0.0, curve_start_z)
        .threePointArc((mid_y, mid_z), (curve_end_y, curve_end_z))
        .lineTo(curve_end_y, outlet_end_z - 0.004)
    )

    tube = cq.Workplane("XY").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.004)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    base = cq.Workplane("XY").circle(SPOUT_BASE_R).extrude(SPOUT_BASE_H)

    unbored = tube.union(base)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle central hub: axis +Z from base, knurled band, domed cap."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_H)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=HUB_H * 0.28)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(HUB_H * 0.44)
    )
    dome = cq.Workplane("XY").workplane(offset=HUB_H).sphere(HUB_R * 0.85)
    return hub.union(knurl).union(dome)


def _build_aerator_solid() -> cq.Workplane:
    """Small cylindrical aerator body with through-bore."""
    body = cq.Workplane("XY").circle(AERATOR_R).extrude(AERATOR_H)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .circle(AERATOR_BORE_R)
        .extrude(AERATOR_H + 0.004)
    )
    # Add a small collar ring at the top (hinge connection)
    collar = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .circle(AERATOR_R + 0.003)
        .circle(AERATOR_R - 0.001)
        .extrude(0.005)
    )
    return body.cut(bore).union(collar)


def _build_seam_ring(inner_r: float, outer_r: float) -> cq.Workplane:
    """Thin annular seam ring for deck-base junction."""
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(SEAM_HEIGHT)
    )


def _add_handle_visuals(handle, hub_mesh, gold, angle_offset: float) -> None:
    """Four-arm cross handle in the handle frame (joint frame at top of valve
    body, axis +Z). Includes stem, hub, spokes at the asymmetric rest angle,
    and rounded tips."""
    # Stem seats downward into the valve body.
    handle.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
        material=gold,
        name="stem",
    )
    # Hub rises from the joint.
    handle.visual(hub_mesh, material=gold, name="hub")

    spoke_z = HUB_H * 0.5
    c = math.cos(angle_offset)
    s = math.sin(angle_offset)

    # Rod originally along X, rotated by angle_offset about Z.
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, spoke_z), rpy=(0.0, math.pi / 2.0, angle_offset)),
        material=gold,
        name="spoke_pair_x",
    )
    # Rod originally along Y, rotated by angle_offset about Z.
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, spoke_z), rpy=(math.pi / 2.0, 0.0, angle_offset)),
        material=gold,
        name="spoke_pair_y",
    )

    half = HANDLE_ROD_LEN / 2.0
    # Tip positions rotated by the asymmetric angle.
    for name, (dx, dy) in (
        ("tip_x_pos", (half * c, half * s)),
        ("tip_x_neg", (-half * c, -half * s)),
        ("tip_y_pos", (-half * s, half * c)),
        ("tip_y_neg", (half * s, -half * c)),
    ):
        handle.visual(
            Sphere(radius=HANDLE_ROD_R),
            origin=Origin(xyz=(dx, dy, spoke_z)),
            material=gold,
            name=name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    deck_mat = model.material("deck_white", rgba=(0.92, 0.92, 0.89, 1.0))
    seam_mat = model.material("dark_seam", rgba=(0.15, 0.12, 0.08, 1.0))

    # --- deck (root, countertop mounting surface) ---
    deck = model.part("deck")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, DECK_T / 2.0)),
        material=deck_mat,
        name="slab",
    )

    # Seam rings on the deck at each base location
    spout_seam_mesh = mesh_from_cadquery(
        _build_seam_ring(SPOUT_BASE_R + 0.0005, SPOUT_BASE_R + 0.0005 + SEAM_WIDTH),
        "spout_seam_ring",
    )
    valve_seam_mesh = mesh_from_cadquery(
        _build_seam_ring(VALVE_ESC_R + 0.0005, VALVE_ESC_R + 0.0005 + SEAM_WIDTH),
        "valve_seam_ring",
    )
    deck.visual(
        spout_seam_mesh,
        origin=Origin(xyz=(0.0, 0.0, DECK_T)),
        material=seam_mat,
        name="spout_seam",
    )
    deck.visual(
        valve_seam_mesh,
        origin=Origin(xyz=(-VALVE_SPACING, 0.0, DECK_T)),
        material=seam_mat,
        name="left_valve_seam",
    )
    deck.visual(
        valve_seam_mesh,
        origin=Origin(xyz=(VALVE_SPACING, 0.0, DECK_T)),
        material=seam_mat,
        name="right_valve_seam",
    )

    # --- central spout (fixed, rises from deck) ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_solid(), "spout"),
        material=gold,
        name="tube",
    )
    # Hinge barrel at the outlet for the aerator pivot
    spout.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(
            xyz=(0.0, OUTLET_Y, OUTLET_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=gold,
        name="hinge_barrel",
    )
    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, DECK_T)),
    )

    # --- aerator (revolute, pivots downward on hinge) ---
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator_solid(), "aerator"),
        # Flip to hang downward from the joint (Z becomes -Z)
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi, 0.0, 0.0)),
        material=gold,
        name="aerator_body",
    )
    model.articulation(
        "spout_to_aerator",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(0.0, OUTLET_Y, OUTLET_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.5, lower=0.0, upper=0.55
        ),
    )

    # --- valve assemblies (fixed) and cross handles (revolute) ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")

    for side, sx, angle in (
        ("left", -1.0, LEFT_HANDLE_ANGLE),
        ("right", 1.0, RIGHT_HANDLE_ANGLE),
    ):
        valve = model.part(f"{side}_valve")
        # Escutcheon disk on the deck
        valve.visual(
            Cylinder(radius=VALVE_ESC_R, length=VALVE_ESC_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_H / 2.0)),
            material=gold,
            name="escutcheon",
        )
        # Valve body rising above escutcheon
        valve.visual(
            Cylinder(radius=VALVE_BODY_R, length=VALVE_BODY_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_H + VALVE_BODY_H / 2.0)),
            material=gold,
            name="valve_body",
        )
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * VALVE_SPACING, 0.0, DECK_T)),
        )

        handle = model.part(f"{side}_cross_handle")
        _add_handle_visuals(handle, hub_mesh, gold, angle)
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_H + VALVE_BODY_H)),
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
    aerator = object_model.get_part("aerator")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_cross_handle")
    right_handle = object_model.get_part("right_cross_handle")

    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")
    aerator_joint = object_model.get_articulation("spout_to_aerator")

    # --- aerator pivot joint: revolute with horizontal axis ---
    ctx.check(
        "aerator_joint_is_revolute",
        str(aerator_joint.joint_type).lower().endswith("revolute"),
        f"type={aerator_joint.joint_type}",
    )
    ax = aerator_joint.axis
    ctx.check(
        "aerator_axis_horizontal_perpendicular_to_spout",
        abs(ax[0] - 1.0) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2]) < 1e-9,
        f"axis={ax}",
    )
    lim = aerator_joint.motion_limits
    ctx.check(
        "aerator_pivot_limits",
        lim is not None
        and abs(lim.lower) < 1e-6
        and 0.3 <= lim.upper <= 0.7,
        f"limits=({lim.lower}, {lim.upper})",
    )

    # --- aerator pivots downward when posed ---
    rest_aabb = ctx.part_world_aabb(aerator)
    assert rest_aabb is not None
    rest_z_min = rest_aabb[0][2]
    with ctx.pose({aerator_joint: 0.4}):
        pivoted_aabb = ctx.part_world_aabb(aerator)
        assert pivoted_aabb is not None
        pivoted_y_max = pivoted_aabb[1][1]
        rest_y_max = rest_aabb[1][1]
        ctx.check(
            "aerator_pivots_forward",
            pivoted_y_max > rest_y_max + 0.002,
            f"rest_y_max={rest_y_max:.4f}, pivoted_y_max={pivoted_y_max:.4f}",
        )

    # --- handle joints: revolute, vertical axis ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        jax = joint.axis
        ctx.check(
            f"{joint.name}_vertical_axis",
            abs(jax[0]) < 1e-9 and abs(jax[1]) < 1e-9 and abs(jax[2] - 1.0) < 1e-9,
            f"axis={jax}",
        )
        jlim = joint.motion_limits
        ctx.check(
            f"{joint.name}_full_turn_range",
            jlim is not None
            and abs(jlim.lower + math.pi) < 1e-6
            and abs(jlim.upper - math.pi) < 1e-6,
            f"limits=({jlim.lower}, {jlim.upper})",
        )

    # --- asymmetric handle rest angles (cross extents differ due to angle) ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert lh_aabb is not None and rh_aabb is not None
    lh_dx = lh_aabb[1][0] - lh_aabb[0][0]
    rh_dx = rh_aabb[1][0] - rh_aabb[0][0]
    lh_dy = lh_aabb[1][1] - lh_aabb[0][1]
    rh_dy = rh_aabb[1][1] - rh_aabb[0][1]
    ctx.check(
        "handles_asymmetric_spoke_extents",
        abs(lh_dx - rh_dx) > 0.003 or abs(lh_dy - rh_dy) > 0.003,
        f"left=({lh_dx:.4f},{lh_dy:.4f}), right=({rh_dx:.4f},{rh_dy:.4f})",
    )

    # --- intentional embedding: stems seat into valve bodies ---
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

    # --- aerator hinge: collar wraps around spout hinge barrel at outlet ---
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a=aerator.get_visual("aerator_body"),
        elem_b=spout.get_visual("tube"),
        reason="aerator collar wraps around the spout hinge barrel at the outlet for the pivot connection",
    )
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a=aerator.get_visual("aerator_body"),
        elem_b=spout.get_visual("hinge_barrel"),
        reason="hinge barrel passes through the aerator collar as the pivot pin",
    )
    # Proof: aerator stays connected to spout at the outlet
    ctx.expect_overlap(
        aerator, spout, axes="xy", min_overlap=0.005,
        name="aerator_stays_at_spout_outlet",
    )

    # --- spout is hollow (bore visible at outlet) ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.95 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} vs unbored={SPOUT_UNBORED_VOLUME:.3e}",
    )

    # --- spout rises from deck and projects forward ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "spout_rises_above_deck",
        sz1 > DECK_T + 0.10,
        f"spout zmax={sz1:.3f}, deck top={DECK_T}",
    )
    ctx.check(
        "spout_projects_forward",
        sy1 > 0.06,
        f"spout ymax={sy1:.3f}",
    )

    # --- seam rings present at all three deck bases ---
    spout_seam = deck.get_visual("spout_seam")
    left_seam = deck.get_visual("left_valve_seam")
    right_seam = deck.get_visual("right_valve_seam")
    ctx.check(
        "three_seam_rings_exist",
        spout_seam is not None and left_seam is not None and right_seam is not None,
        "missing seam visuals on deck",
    )

    # --- overall width about 0.30 m ---
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.27 <= total_w <= 0.33,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- handles flanking spout ---
    lv_pos = ctx.part_world_position(left_valve)
    rv_pos = ctx.part_world_position(right_valve)
    assert lv_pos is not None and rv_pos is not None
    ctx.check(
        "valves_flank_spout",
        abs(lv_pos[0] + VALVE_SPACING) < 1e-4
        and abs(rv_pos[0] - VALVE_SPACING) < 1e-4,
        f"left_x={lv_pos[0]:.4f}, right_x={rv_pos[0]:.4f}",
    )

    # --- handle rotation proof ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        cen = ctx.part_world_position(left_handle)
        assert cen is not None
        ctx.check(
            "left_handle_stays_on_valve_while_rotating",
            abs(cen[0] - lv_pos[0]) < 1e-4 and abs(cen[1] - lv_pos[1]) < 1e-4,
            f"handle center={cen}, valve={lv_pos}",
        )

    # --- deck grounded ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_grounded_at_z0",
        abs(deck_aabb[0][2]) < 1e-6,
        f"deck zmin={deck_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
