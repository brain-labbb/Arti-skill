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
#   - The deck/counter surface is the horizontal XY plane at z = 0.
#   - The deck slab occupies z < 0 (thickness below the surface).
#   - The spout rises along +Z from the deck center and curves forward (-Y).
#   - Left handle at x = -HANDLE_SPREAD_X, right at x = +HANDLE_SPREAD_X.
#   - Viewer-facing side is -Y.
# ---------------------------------------------------------------------------

# Layout
HANDLE_SPREAD_X = 0.10  # handle centers at x = +/- 0.10

# Deck panel (mounting substrate)
DECK_W = 0.38
DECK_D = 0.20
DECK_T = 0.015

# Spout
SPOUT_TUBE_R = 0.014  # outer radius
SPOUT_BORE_R = 0.010  # inner bore radius (visible at outlet)
SPOUT_RISER_H = 0.14  # vertical rise before the curve
SPOUT_BEND_R = 0.05  # bend radius for the gooseneck curve
SPOUT_REACH_Y = 0.08  # how far forward the spout reaches
SPOUT_DROP_Z = 0.04  # outlet end height above deck

# Spout escutcheon base
SPOUT_BASE_R = 0.025
SPOUT_BASE_H = 0.010
SEAM_H = 0.0015  # narrow seam ring height at deck bases

# Handle pedestals (tapered with decorative rings)
PED_BASE_R = 0.022  # bottom radius
PED_TOP_R = 0.015  # top radius
PED_HEIGHT = 0.055  # total pedestal height
PED_RING_H = 0.003  # ring thickness
PED_RING_PROTRUSION = 0.003  # how far rings protrude beyond pedestal surface
PED_RING_POSITIONS = [0.012, 0.030]  # z positions of decorative rings

# Handle lever
LEVER_R = 0.008  # lever cylinder radius
LEVER_LEN = 0.055  # lever arm length from center
HUB_R = 0.012  # central hub radius on top of pedestal
HUB_H = 0.012  # hub height
STEM_R = 0.006  # stem radius (hidden inside pedestal top)
STEM_LEN = 0.015  # stem length

# Handle base escutcheon
HANDLE_BASE_R = 0.024
HANDLE_BASE_H = 0.008

# Computed by build for hollow-bore verification
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_solid() -> cq.Workplane:
    """Spout in local frame: base on XY plane at z=0, rises vertically then
    curves forward (-Y direction) and downward to an open outlet."""
    # Arc geometry: quarter-circle from 90° to 180° around center (0, SPOUT_RISER_H - SPOUT_BEND_R)
    arc_center_z = SPOUT_RISER_H - SPOUT_BEND_R
    s45 = math.sin(math.pi / 4.0)

    arc_start = (0.0, SPOUT_RISER_H)
    arc_mid = (-SPOUT_BEND_R * s45, arc_center_z + SPOUT_BEND_R * s45)
    arc_end = (-SPOUT_BEND_R, arc_center_z)
    outlet_end = (-SPOUT_REACH_Y, SPOUT_DROP_Z)

    # Path in YZ workplane: first coord = Y, second coord = Z
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(arc_start[0], arc_start[1])
        .threePointArc(arc_mid, arc_end)
        .lineTo(outlet_end[0], outlet_end[1])
    )
    # Bore path slightly extended for clean cuts
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -0.003)
        .lineTo(arc_start[0], arc_start[1])
        .threePointArc(arc_mid, arc_end)
        .lineTo(outlet_end[0], outlet_end[1] - 0.003)
    )

    # Sweep tube along path - XY workplane normal is +Z matching path start tangent
    tube = cq.Workplane("XY").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    # Base escutcheon cylinder
    base = cq.Workplane("XY").circle(SPOUT_BASE_R).extrude(SPOUT_BASE_H)

    unbored = tube.union(base)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_pedestal_solid() -> cq.Workplane:
    """Tapered pedestal with decorative ring ridges.
    Local frame: base on XY plane at z=0, rises along +Z."""
    # Tapered body via revolution of trapezoidal profile
    body = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(PED_BASE_R, 0.0)
        .lineTo(PED_TOP_R, PED_HEIGHT)
        .lineTo(0.0, PED_HEIGHT)
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    # Add decorative ring ridges protruding from the tapered surface
    for z_pos in PED_RING_POSITIONS:
        frac = z_pos / PED_HEIGHT
        local_r = PED_BASE_R + (PED_TOP_R - PED_BASE_R) * frac
        ring_outer = local_r + PED_RING_PROTRUSION
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z_pos)
            .circle(ring_outer)
            .circle(local_r - 0.0005)
            .extrude(PED_RING_H)
        )
        body = body.union(ring)
    return body


def _build_lever_solid() -> cq.Workplane:
    """Lever handle in local frame: hub centered at origin, lever arm extending
    along +X. Stem extends downward along -Z. Revolute axis is Z."""
    # Central hub/cap
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_H)
    # Lever arm - horizontal cylinder along +X at mid-hub height
    lever = (
        cq.Workplane("YZ")
        .circle(LEVER_R)
        .extrude(LEVER_LEN)
    ).translate((0, 0, HUB_H / 2.0))
    # Rounded end cap on lever
    end_cap = cq.Workplane("XY").sphere(LEVER_R).translate((LEVER_LEN, 0, HUB_H / 2.0))
    # Stem going down into pedestal
    stem = (
        cq.Workplane("XY")
        .circle(STEM_R)
        .extrude(STEM_LEN)
    ).translate((0, 0, -STEM_LEN))

    return hub.union(lever).union(end_cap).union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_gold_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    deck_mat = model.material("counter_marble", rgba=(0.92, 0.90, 0.88, 1.0))
    seam_mat = model.material("dark_seam", rgba=(0.15, 0.12, 0.10, 1.0))

    # --- deck panel (root, mounting substrate) ---
    deck = model.part("deck_panel")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T / 2.0)),
        material=deck_mat,
        name="slab",
    )

    # --- central spout (fixed) ---
    spout = model.part("spout")
    spout.visual(mesh_from_cadquery(_build_spout_solid(), "spout"), material=gold, name="tube")
    # Narrow seam ring at spout deck base
    spout.visual(
        Cylinder(radius=SPOUT_BASE_R + 0.002, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, -SEAM_H / 2.0)),
        material=seam_mat,
        name="deck_seam",
    )
    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- handle assemblies: pedestal (fixed) + lever (revolute vertical) ---
    pedestal_mesh = mesh_from_cadquery(_build_pedestal_solid(), "pedestal")
    lever_mesh = mesh_from_cadquery(_build_lever_solid(), "lever")

    for side, sx in (("left", -1.0), ("right", 1.0)):
        # Pedestal + base (fixed to deck)
        pedestal = model.part(f"{side}_pedestal")
        pedestal.visual(pedestal_mesh, material=gold, name="body")
        # Base escutcheon ring
        pedestal.visual(
            Cylinder(radius=HANDLE_BASE_R, length=HANDLE_BASE_H),
            origin=Origin(xyz=(0.0, 0.0, HANDLE_BASE_H / 2.0)),
            material=gold,
            name="base_ring",
        )
        # Narrow seam at deck base
        pedestal.visual(
            Cylinder(radius=HANDLE_BASE_R + 0.002, length=SEAM_H),
            origin=Origin(xyz=(0.0, 0.0, -SEAM_H / 2.0)),
            material=seam_mat,
            name="deck_seam",
        )
        model.articulation(
            f"deck_to_{side}_pedestal",
            ArticulationType.FIXED,
            parent=deck,
            child=pedestal,
            origin=Origin(xyz=(sx * HANDLE_SPREAD_X, 0.0, 0.0)),
        )

        # Lever handle (revolute around vertical stem axis)
        handle = model.part(f"{side}_lever_handle")
        handle.visual(lever_mesh, material=gold, name="lever")
        model.articulation(
            f"{side}_handle_joint",
            ArticulationType.REVOLUTE,
            parent=pedestal,
            child=handle,
            # Joint at top of pedestal, axis vertical
            origin=Origin(xyz=(0.0, 0.0, PED_HEIGHT)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0, velocity=2.0, lower=-math.pi / 2.0, upper=math.pi / 2.0
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_panel")
    spout = object_model.get_part("spout")
    left_ped = object_model.get_part("left_pedestal")
    right_ped = object_model.get_part("right_pedestal")
    left_handle = object_model.get_part("left_lever_handle")
    right_handle = object_model.get_part("right_lever_handle")
    left_joint = object_model.get_articulation("left_handle_joint")
    right_joint = object_model.get_articulation("right_handle_joint")

    # --- Variant 04 checks: widespread deck-mounted layout ---

    # Two independent revolute joints around vertical axes
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_is_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_is_vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2] - 1.0) < 1e-9,
            f"axis={ax}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_has_quarter_turn_limits",
            lim is not None
            and abs(lim.lower + math.pi / 2.0) < 1e-6
            and abs(lim.upper - math.pi / 2.0) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # --- Spout is hollow tube with visible bore ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.95 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} m^3 vs unbored={SPOUT_UNBORED_VOLUME:.3e} m^3",
    )

    # --- Three-piece widespread layout: handles flank spout ---
    lp = ctx.part_world_position(left_ped)
    rp = ctx.part_world_position(right_ped)
    sp = ctx.part_world_position(spout)
    assert lp is not None and rp is not None and sp is not None
    ctx.check(
        "widespread_layout_handles_flank_spout",
        lp[0] < sp[0] - 0.05 and rp[0] > sp[0] + 0.05,
        f"left_x={lp[0]:.3f}, spout_x={sp[0]:.3f}, right_x={rp[0]:.3f}",
    )
    ctx.check(
        "widespread_layout_symmetric_spread",
        abs(abs(lp[0]) - HANDLE_SPREAD_X) < 0.005
        and abs(abs(rp[0]) - HANDLE_SPREAD_X) < 0.005,
        f"left={lp[0]:.4f}, right={rp[0]:.4f}, expected=+/-{HANDLE_SPREAD_X}",
    )

    # --- Deck seams present on all three bases ---
    for part_obj, part_name in [
        (spout, "spout"),
        (left_ped, "left_pedestal"),
        (right_ped, "right_pedestal"),
    ]:
        seam = part_obj.get_visual("deck_seam")
        ctx.check(
            f"{part_name}_has_deck_seam",
            seam is not None,
            f"{part_name} missing deck_seam visual",
        )

    # --- Pedestals have decorative ring ridges (tapered body) ---
    for side, ped in (("left", left_ped), ("right", right_ped)):
        ped_aabb = ctx.part_world_aabb(ped)
        assert ped_aabb is not None
        ped_height = ped_aabb[1][2] - ped_aabb[0][2]
        ctx.check(
            f"{side}_pedestal_height_matches_design",
            abs(ped_height - PED_HEIGHT - HANDLE_BASE_H) < 0.010,
            f"{side} pedestal height={ped_height:.4f}, expected~{PED_HEIGHT + HANDLE_BASE_H}",
        )

    # --- Lever handles have cylindrical lever arms ---
    for side, handle in (("left", left_handle), ("right", right_handle)):
        h_aabb = ctx.part_world_aabb(handle)
        assert h_aabb is not None
        h_span_x = h_aabb[1][0] - h_aabb[0][0]
        h_span_y = h_aabb[1][1] - h_aabb[0][1]
        max_span = max(h_span_x, h_span_y)
        ctx.check(
            f"{side}_lever_handle_has_arm_extent",
            max_span >= LEVER_LEN * 0.8,
            f"{side} lever max_span={max_span:.4f}, expected>={LEVER_LEN * 0.8:.4f}",
        )

    # --- Handles mounted on pedestals (connected, not floating) ---
    ctx.expect_overlap(left_handle, left_ped, axes="xy", min_overlap=0.005)
    ctx.expect_overlap(right_handle, right_ped, axes="xy", min_overlap=0.005)

    # --- Intentional overlap: lever stems seat into pedestal tops ---
    ctx.allow_overlap(
        left_handle,
        left_ped,
        elem_a=left_handle.get_visual("lever"),
        elem_b=left_ped.get_visual("body"),
        reason="lever stem is seated inside the pedestal top bore and rotates with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_ped,
        elem_a=right_handle.get_visual("lever"),
        elem_b=right_ped.get_visual("body"),
        reason="lever stem is seated inside the pedestal top bore and rotates with the handle",
    )

    # --- Pose check: lever rotation around vertical axis ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_pos = ctx.part_world_position(left_handle)
        assert rot_pos is not None
        lp_world = ctx.part_world_position(left_ped)
        assert lp_world is not None
        ctx.check(
            "left_handle_stays_on_pedestal_while_rotating",
            abs(rot_pos[0] - lp_world[0]) < 0.005
            and abs(rot_pos[1] - lp_world[1]) < 0.005,
            f"handle_xy=({rot_pos[0]:.4f},{rot_pos[1]:.4f}), ped_xy=({lp_world[0]:.4f},{lp_world[1]:.4f})",
        )

    with ctx.pose({right_joint: -math.pi / 3.0}):
        # Right handle rotates; lever arm swings, proving real articulation
        rh_aabb = ctx.part_world_aabb(right_handle)
        assert rh_aabb is not None
        ctx.check(
            "right_handle_lever_sweeps_when_rotated",
            (rh_aabb[1][0] - rh_aabb[0][0]) > 0.02
            and (rh_aabb[1][1] - rh_aabb[0][1]) > 0.02,
            f"right handle AABB at -60deg: x={rh_aabb[1][0]-rh_aabb[0][0]:.3f}, y={rh_aabb[1][1]-rh_aabb[0][1]:.3f}",
        )

    # --- Deck panel grounded at z=0 ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_top_at_z_zero",
        abs(deck_aabb[1][2]) < 1e-6,
        f"deck zmax={deck_aabb[1][2]:.4f}",
    )

    # --- Spout rises above deck ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    ctx.check(
        "spout_rises_above_deck",
        spout_aabb[1][2] > 0.10,
        f"spout zmax={spout_aabb[1][2]:.3f}, expected>0.10",
    )

    return ctx.report()


object_model = build_object_model()
