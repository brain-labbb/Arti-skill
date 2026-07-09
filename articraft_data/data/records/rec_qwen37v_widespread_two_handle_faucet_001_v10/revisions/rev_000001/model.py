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
# Widespread two-handle deck-mounted bathroom faucet in polished gold brass.
#
# Frame conventions:
#   - World Z is up. The deck plate sits on the XY plane at z = 0.
#   - The spout rises vertically from the deck center and swivels about Z.
#   - Two handle assemblies flank the spout at x = +/- HANDLE_OFFSET_X.
#   - The faucet is ~0.30 m wide overall (handle-tip to handle-tip).
# ---------------------------------------------------------------------------

# Layout
HANDLE_OFFSET_X = 0.10  # handle centers at x = +/- 0.10
DECK_W = 0.34
DECK_D = 0.08
DECK_H = 0.010

# Spout base / swivel collar
SPOUT_BASE_R = 0.022
SPOUT_BASE_H = 0.025
SEAM_GAP = 0.0015  # narrow visible seam at deck base

# Spout tube (gooseneck arc)
SPOUT_TUBE_R = 0.012  # outer radius
SPOUT_BORE_R = 0.0085  # inner bore
SPOUT_RISE = 0.14  # vertical rise before arc
SPOUT_ARC_R = 0.05  # arc radius of the gooseneck bend
SPOUT_REACH = 0.08  # horizontal reach past the apex

# Handle pedestal
PEDESTAL_R = 0.018
PEDESTAL_H = 0.055
PEDESTAL_RING_R = 0.021  # outer radius of decorative rings
PEDESTAL_RING_THICK = 0.004  # ring height
RING_POSITIONS = (0.012, 0.028, 0.044)  # z heights of rings on pedestal

# Cross handle
HANDLE_HUB_R = 0.012
HANDLE_HUB_H = 0.020
HANDLE_SPOKE_R = 0.004
HANDLE_SPOKE_LEN = 0.080  # tip-to-tip ~0.08 m
STEM_R = 0.006
STEM_LEN = 0.012

# Asymmetric handle rest angles (degrees, about Z from forward)
LEFT_ANGLE_DEG = 25.0
RIGHT_ANGLE_DEG = -15.0

# Computed for hollow bore verification
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_gooseneck_spout() -> cq.Workplane:
    """Gooseneck spout in its local frame: base on XY plane at z=0,
    tube rises along +Z, arcs over in the XZ plane, drops to an open outlet."""
    # Path in XZ plane (local x -> world x, local y -> world z on XZ workplane)
    arc_mid = (
        SPOUT_ARC_R * math.sin(math.pi / 4),
        SPOUT_RISE + SPOUT_ARC_R * (1.0 - math.cos(math.pi / 4)),
    )
    arc_end = (SPOUT_ARC_R, SPOUT_RISE)
    outlet_end = (SPOUT_ARC_R + SPOUT_REACH, SPOUT_RISE - 0.04)

    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, SPOUT_BASE_H)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc(arc_mid, arc_end)
        .lineTo(outlet_end[0], outlet_end[1])
    )
    bore_path = (
        cq.Workplane("XZ")
        .moveTo(0.0, SPOUT_BASE_H - 0.003)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc(arc_mid, arc_end)
        .lineTo(outlet_end[0], outlet_end[1] - 0.003)
    )

    tube = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_BASE_H)
        .circle(SPOUT_TUBE_R)
        .sweep(path)
    )
    bore = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_BASE_H - 0.003)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    base = cq.Workplane("XY").circle(SPOUT_BASE_R).extrude(SPOUT_BASE_H)
    # Seam ring at deck interface (thin lip below base)
    seam = (
        cq.Workplane("XY")
        .workplane(offset=-SEAM_GAP)
        .circle(SPOUT_BASE_R + 0.003)
        .extrude(SEAM_GAP)
    )

    unbored = base.union(tube).union(seam)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_pedestal_with_rings() -> cq.Workplane:
    """Handle pedestal cylinder with decorative ring ridges, single fused solid."""
    body = cq.Workplane("XY").circle(PEDESTAL_R).extrude(PEDESTAL_H)
    for rz in RING_POSITIONS:
        ring = (
            cq.Workplane("XY")
            .workplane(offset=rz - PEDESTAL_RING_THICK / 2.0)
            .circle(PEDESTAL_RING_R)
            .extrude(PEDESTAL_RING_THICK)
        )
        body = body.union(ring)
    # Seam lip at base
    seam = (
        cq.Workplane("XY")
        .workplane(offset=-SEAM_GAP)
        .circle(PEDESTAL_R + 0.003)
        .extrude(SEAM_GAP)
    )
    body = body.union(seam)
    return body


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle knurled hub as a single fused CadQuery solid."""
    hub = cq.Workplane("XY").circle(HANDLE_HUB_R).extrude(HANDLE_HUB_H)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .polygon(12, 2.0 * (HANDLE_HUB_R + 0.002))
        .extrude(0.010)
    )
    dome = (
        cq.Workplane("XY")
        .workplane(offset=HANDLE_HUB_H)
        .sphere(HANDLE_HUB_R * 0.80)
    )
    return hub.union(knurl).union(dome)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    dark_seam = model.material("dark_seam_ring", rgba=(0.15, 0.12, 0.08, 1.0))
    deck_mat = model.material("deck_surface", rgba=(0.90, 0.88, 0.85, 1.0))

    # --- deck plate (root, represents counter/sink mounting surface) ---
    deck = model.part("deck_plate")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_H)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_H / 2.0)),
        material=deck_mat,
        name="deck_surface",
    )

    # --- central spout (continuous swivel about Z) ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_gooseneck_spout(), "gooseneck_spout"),
        material=gold,
        name="spout_body",
    )
    model.articulation(
        "deck_to_spout",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0),
    )

    # --- handle assemblies ---
    pedestal_mesh = mesh_from_cadquery(_build_pedestal_with_rings(), "handle_pedestal")
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")

    left_angle_rad = math.radians(LEFT_ANGLE_DEG)
    right_angle_rad = math.radians(RIGHT_ANGLE_DEG)

    for side, sx, angle_rad in [
        ("left", -1.0, left_angle_rad),
        ("right", 1.0, right_angle_rad),
    ]:
        # Pedestal (fixed to deck)
        pedestal = model.part(f"{side}_pedestal")
        pedestal.visual(pedestal_mesh, material=gold, name="pedestal_body")
        model.articulation(
            f"deck_to_{side}_pedestal",
            ArticulationType.FIXED,
            parent=deck,
            child=pedestal,
            origin=Origin(xyz=(sx * HANDLE_OFFSET_X, 0.0, 0.0)),
        )

        # Cross handle (revolute about Z) - separate visuals for connectivity
        handle = model.part(f"{side}_handle")

        # Stem seats into pedestal top
        handle.visual(
            Cylinder(radius=STEM_R, length=STEM_LEN),
            origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
            material=gold,
            name="stem",
        )
        # Hub (knurled central boss)
        handle.visual(hub_mesh, material=gold, name="hub")

        # Spokes at asymmetric rest angle. Build in the handle frame:
        # The cross is rotated by angle_rad about Z from the standard +X/+Y axes.
        half = HANDLE_SPOKE_LEN / 2.0
        spoke_z = HANDLE_HUB_H / 2.0  # spoke axis height

        for i in range(4):
            theta = angle_rad + i * (math.pi / 2.0)
            dx = math.cos(theta)
            dy = math.sin(theta)
            # Spoke cylinder along the (dx, dy) direction, centered on hub axis
            # Use a cylinder oriented along an arbitrary direction:
            # Build along local X, then rotate into place
            spoke_name = f"spoke_{i}"
            handle.visual(
                Cylinder(radius=HANDLE_SPOKE_R, length=HANDLE_SPOKE_LEN),
                origin=Origin(
                    xyz=(0.0, 0.0, spoke_z),
                    # Rotate Z-default cylinder onto the (dx, dy, 0) direction:
                    # first rotate 90° about Y to lay along X,
                    # then rotate theta about Z.
                    rpy=(0.0, math.pi / 2.0, theta),
                ),
                material=gold,
                name=spoke_name,
            )
            # Sphere tip at spoke end
            tip_x = dx * half
            tip_y = dy * half
            handle.visual(
                Sphere(radius=HANDLE_SPOKE_R * 1.3),
                origin=Origin(xyz=(tip_x, tip_y, spoke_z)),
                material=gold,
                name=f"tip_{i}",
            )

        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=pedestal,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_plate")
    spout = object_model.get_part("spout")
    left_pedestal = object_model.get_part("left_pedestal")
    right_pedestal = object_model.get_part("right_pedestal")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")

    swivel = object_model.get_articulation("deck_to_spout")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # --- spout swivel: continuous joint about vertical axis ---
    ctx.check(
        "spout_swivel_is_continuous",
        str(swivel.joint_type).lower().endswith("continuous"),
        f"type={swivel.joint_type}",
    )
    swivel_ax = swivel.axis
    ctx.check(
        "spout_swivel_axis_is_vertical",
        abs(swivel_ax[0]) < 1e-9 and abs(swivel_ax[1]) < 1e-9 and abs(swivel_ax[2] - 1.0) < 1e-9,
        f"axis={swivel_ax}",
    )

    # --- handle joints: revolute about vertical axis ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_vertical",
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

    # --- spout swivel actually rotates the gooseneck ---
    rest_aabb = ctx.part_world_aabb(spout)
    assert rest_aabb is not None
    with ctx.pose({swivel: math.pi / 2.0}):
        rotated_aabb = ctx.part_world_aabb(spout)
    assert rotated_aabb is not None
    # After 90° rotation about Z, X extent should become Y extent and vice versa
    rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
    rest_dy = rest_aabb[1][1] - rest_aabb[0][1]
    rot_dx = rotated_aabb[1][0] - rotated_aabb[0][0]
    rot_dy = rotated_aabb[1][1] - rotated_aabb[0][1]
    ctx.check(
        "spout_swivel_rotates_gooseneck",
        abs(rot_dy - rest_dx) < 0.01 and abs(rot_dx - rest_dy) < 0.01,
        f"rest dx={rest_dx:.3f} dy={rest_dy:.3f}, rotated dx={rot_dx:.3f} dy={rot_dy:.3f}",
    )

    # --- spout is hollow ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.95 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} vs unbored={SPOUT_UNBORED_VOLUME:.3e}",
    )

    # --- spout rises well above deck ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "spout_rises_above_deck",
        sz1 > 0.10,
        f"spout zmax={sz1:.3f}",
    )

    # --- three-piece widespread layout: left, center, right ---
    lp = ctx.part_world_position(left_pedestal)
    rp = ctx.part_world_position(right_pedestal)
    sp = ctx.part_world_position(spout)
    assert lp is not None and rp is not None and sp is not None
    ctx.check(
        "spout_centered_between_handles",
        abs(sp[0]) < 0.005 and lp[0] < sp[0] < rp[0],
        f"left_x={lp[0]:.3f}, spout_x={sp[0]:.3f}, right_x={rp[0]:.3f}",
    )
    ctx.check(
        "handles_flank_at_offset",
        abs(lp[0] + HANDLE_OFFSET_X) < 0.005 and abs(rp[0] - HANDLE_OFFSET_X) < 0.005,
        f"left_x={lp[0]:.3f}, right_x={rp[0]:.3f}",
    )

    # --- asymmetric handle angles: left and right rest angles differ ---
    ctx.check(
        "handles_asymmetrically_angled",
        abs(LEFT_ANGLE_DEG - RIGHT_ANGLE_DEG) > 10.0,
        f"left={LEFT_ANGLE_DEG}°, right={RIGHT_ANGLE_DEG}°",
    )
    # Verify the spoke orientations differ between left and right
    lh_aabb_rest = ctx.part_world_aabb(left_handle)
    rh_aabb_rest = ctx.part_world_aabb(right_handle)
    assert lh_aabb_rest is not None and rh_aabb_rest is not None
    lh_dx = lh_aabb_rest[1][0] - lh_aabb_rest[0][0]
    lh_dy = lh_aabb_rest[1][1] - lh_aabb_rest[0][1]
    rh_dx = rh_aabb_rest[1][0] - rh_aabb_rest[0][0]
    rh_dy = rh_aabb_rest[1][1] - rh_aabb_rest[0][1]
    ctx.check(
        "handle_cross_orientations_differ",
        abs(lh_dx - rh_dx) > 0.002 or abs(lh_dy - rh_dy) > 0.002,
        f"left dx={lh_dx:.3f} dy={lh_dy:.3f}, right dx={rh_dx:.3f} dy={rh_dy:.3f}",
    )

    # --- handles sit on top of pedestals (stem seats into pedestal top, so
    # the handle min_z dips slightly below pedestal max_z by the stem depth) ---
    ctx.expect_gap(
        left_handle, left_pedestal, axis="z",
        max_gap=0.002, max_penetration=STEM_LEN + 0.002,
    )
    ctx.expect_gap(
        right_handle, right_pedestal, axis="z",
        max_gap=0.002, max_penetration=STEM_LEN + 0.002,
    )
    # Prove the handle hub sits at the pedestal top
    ctx.expect_overlap(left_handle, left_pedestal, axes="xy", min_overlap=0.01)
    ctx.expect_overlap(right_handle, right_pedestal, axes="xy", min_overlap=0.01)

    # --- pedestals and spout sit on deck ---
    ctx.expect_gap(spout, deck, axis="z", max_gap=0.005, max_penetration=0.003)
    ctx.expect_gap(left_pedestal, deck, axis="z", max_gap=0.005, max_penetration=0.003)
    ctx.expect_gap(right_pedestal, deck, axis="z", max_gap=0.005, max_penetration=0.003)

    # --- overall width about 0.28-0.34 m ---
    total_w = rh_aabb_rest[1][0] - lh_aabb_rest[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.24 <= total_w <= 0.35,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- handle rotation proof: turning left handle changes its footprint ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_dx = rot_aabb[1][0] - rot_aabb[0][0]
        rot_dy = rot_aabb[1][1] - rot_aabb[0][1]
        ctx.check(
            "left_handle_rotation_changes_footprint",
            abs(rot_dx - lh_dx) > 0.001 or abs(rot_dy - lh_dy) > 0.001,
            f"rest dx={lh_dx:.3f}, rotated dx={rot_dx:.3f}",
        )

    # --- deck grounded ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_plate_grounded",
        abs(deck_aabb[0][2] + DECK_H) < 0.002,
        f"deck zmin={deck_aabb[0][2]:.4f}",
    )

    # --- intentional overlap: stems seat into pedestal tops ---
    ctx.allow_overlap(
        left_handle,
        left_pedestal,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_pedestal.get_visual("pedestal_body"),
        reason="handle stem is seated into the pedestal top bore and turns with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_pedestal,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_pedestal.get_visual("pedestal_body"),
        reason="handle stem is seated into the pedestal top bore and turns with the handle",
    )

    return ctx.report()


object_model = build_object_model()
