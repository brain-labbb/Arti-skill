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
#   - The deck is the horizontal XY plane at z = 0 (slab occupies z < 0).
#   - Everything mounts upward from the deck (+Z direction).
#   - The spout is centered at x=0, a low bridge arch between handles.
#   - Valve assemblies flank at x = +/- VALVE_PITCH_X.
#   - Cross handles rotate about vertical axles (Z axis).
# ---------------------------------------------------------------------------

# Layout
VALVE_PITCH_X = 0.10  # valve centers at x = +/- 0.10 m

# Deck panel (mounting substrate)
DECK_W = 0.36
DECK_D = 0.14
DECK_T = 0.012

# Spout bridge arch
SPOUT_TUBE_R = 0.013       # outer tube radius
SPOUT_BORE_R = 0.009       # inner bore (hollow outlet)
ARCH_RISE_Z = 0.08         # peak height of arch above deck
ARCH_FOOT_X = 0.04         # half-span of the arch footprint
ARCH_BASE_R = 0.016        # base flange radius at each foot

# Valve assemblies
VALVE_ESC_R1, VALVE_ESC_T1 = 0.028, 0.006  # outer escutcheon
VALVE_ESC_R2, VALVE_ESC_T2 = 0.020, 0.006  # inner step
VALVE_BODY_R = 0.013
VALVE_BODY_H = 0.032       # height of valve body above escutcheon
SEAM_H = 0.0015            # seam groove height (narrow deck-base seam)
VALVE_SEAM_R = 0.030       # valve seam ring (slightly wider than escutcheon)
SPOUT_SEAM_R = 0.018       # spout seam ring (slightly wider than base flange)

# Cross handle (horizontal plane, rotating about vertical Z axle)
HANDLE_ROD_R = 0.004
HANDLE_ROD_LEN = 0.082     # rod length (tip-to-tip with sphere tips ~0.09 m)
HUB_R = 0.011
HUB_H = 0.016              # hub height
KNURL_R = 0.0125
STEM_R = 0.006
STEM_DEPTH = 0.010         # stem penetrates into valve body top

# Computed by build_object_model() for hollow-bore verification in run_tests().
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_arch() -> cq.Workplane:
    """Build the central bridge arch spout.

    Local frame: origin at deck level (z=0), arch rises along +Z.
    The arch spans along X axis between two feet at x = +/- ARCH_FOOT_X.
    The tube center is raised by SPOUT_TUBE_R at the feet so the tube
    bottom sits flush on the deck (z=0).
    """
    # Arch path: smooth curve from one foot to the other, peaking at center.
    # Path in XZ plane. The tube center at the feet is at z = SPOUT_TUBE_R
    # so the swept tube bottom touches z = 0.
    z_foot = SPOUT_TUBE_R
    path = (
        cq.Workplane("XZ")
        .moveTo(-ARCH_FOOT_X, z_foot)
        .spline([
            (-ARCH_FOOT_X, z_foot),
            (-ARCH_FOOT_X * 0.65, ARCH_RISE_Z * 0.55),
            (-ARCH_FOOT_X * 0.3, ARCH_RISE_Z * 0.92),
            (0.0, ARCH_RISE_Z),
            (ARCH_FOOT_X * 0.3, ARCH_RISE_Z * 0.92),
            (ARCH_FOOT_X * 0.65, ARCH_RISE_Z * 0.55),
            (ARCH_FOOT_X, z_foot),
        ])
    )

    # Outer tube swept along arch path — profile centered at the path start
    tube = (
        cq.Workplane("YZ")
        .workplane(offset=-ARCH_FOOT_X)
        .center(0.0, z_foot)
        .circle(SPOUT_TUBE_R)
        .sweep(path)
    )

    # Bore path for hollow interior (slightly extended for clean through-cut)
    bore_path = (
        cq.Workplane("XZ")
        .moveTo(-ARCH_FOOT_X - 0.004, z_foot)
        .spline([
            (-ARCH_FOOT_X - 0.004, z_foot),
            (-ARCH_FOOT_X * 0.65, ARCH_RISE_Z * 0.55),
            (-ARCH_FOOT_X * 0.3, ARCH_RISE_Z * 0.92),
            (0.0, ARCH_RISE_Z),
            (ARCH_FOOT_X * 0.3, ARCH_RISE_Z * 0.92),
            (ARCH_FOOT_X * 0.65, ARCH_RISE_Z * 0.55),
            (ARCH_FOOT_X + 0.004, z_foot),
        ])
    )
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=-ARCH_FOOT_X - 0.004)
        .center(0.0, z_foot)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    # Base flanges at each foot: small cylinders from z=0 upward
    base_left = (
        cq.Workplane("XY")
        .center(-ARCH_FOOT_X, 0.0)
        .circle(ARCH_BASE_R)
        .extrude(z_foot + SPOUT_TUBE_R)
    )
    base_right = (
        cq.Workplane("XY")
        .center(ARCH_FOOT_X, 0.0)
        .circle(ARCH_BASE_R)
        .extrude(z_foot + SPOUT_TUBE_R)
    )

    unbored = tube.union(base_left).union(base_right)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_spout_seam_rings() -> cq.Workplane:
    """Thin seam rings at the two spout feet, sitting on the deck."""
    ring_left = (
        cq.Workplane("XY")
        .center(-ARCH_FOOT_X, 0.0)
        .circle(SPOUT_SEAM_R)
        .extrude(SEAM_H)
    )
    ring_right = (
        cq.Workplane("XY")
        .center(ARCH_FOOT_X, 0.0)
        .circle(SPOUT_SEAM_R)
        .extrude(SEAM_H)
    )
    return ring_left.union(ring_right)


def _build_hub_solid_vertical() -> cq.Workplane:
    """Cross-handle central hub with vertical axis (Z).
    Back face at z=0, extending upward along +Z.
    Includes knurled band and domed top cap."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_H)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.003)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.010)
    )
    dome = (
        cq.Workplane("XY")
        .workplane(offset=HUB_H - 0.002)
        .sphere(0.010)
    )
    return hub.union(knurl).union(dome)


def _build_valve_with_seam() -> cq.Workplane:
    """Build valve body with stepped escutcheon and narrow deck-base seam.
    Local frame: origin at deck level, axis along +Z."""
    # Narrow seam ring at deck base (thin dark groove)
    seam = (
        cq.Workplane("XY")
        .circle(VALVE_SEAM_R)
        .extrude(SEAM_H)
    )
    # Outer escutcheon
    esc_outer = (
        cq.Workplane("XY")
        .workplane(offset=SEAM_H)
        .circle(VALVE_ESC_R1)
        .extrude(VALVE_ESC_T1)
    )
    # Inner escutcheon step
    esc_inner = (
        cq.Workplane("XY")
        .workplane(offset=SEAM_H + VALVE_ESC_T1)
        .circle(VALVE_ESC_R2)
        .extrude(VALVE_ESC_T2)
    )
    # Valve body cylinder
    body_base = SEAM_H + VALVE_ESC_T1 + VALVE_ESC_T2
    body = (
        cq.Workplane("XY")
        .workplane(offset=body_base)
        .circle(VALVE_BODY_R)
        .extrude(VALVE_BODY_H)
    )
    return seam.union(esc_outer).union(esc_inner).union(body)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_gold_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    seam_dark = model.material("seam_shadow", rgba=(0.30, 0.24, 0.08, 1.0))
    deck_mat = model.material("countertop", rgba=(0.82, 0.80, 0.78, 1.0))

    # --- Deck panel (root, horizontal mounting surface) ---
    deck = model.part("deck_panel")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T / 2.0)),
        material=deck_mat,
        name="deck_slab",
    )

    # --- Central spout (bridge arch, fixed) ---
    spout = model.part("spout")
    # Seam rings sit below the arch feet
    spout.visual(
        mesh_from_cadquery(_build_spout_seam_rings(), "spout_seam"),
        material=seam_dark,
        name="base_seams",
    )
    # Arch tube sits on top of seam rings
    spout.visual(
        mesh_from_cadquery(_build_spout_arch(), "spout_arch"),
        material=gold,
        name="arch",
    )
    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Valve assemblies (fixed) and cross handles (revolute, vertical axle) ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid_vertical(), "handle_hub")
    valve_mesh = mesh_from_cadquery(_build_valve_with_seam(), "valve_body")

    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        valve.visual(valve_mesh, material=gold, name="valve_assembly")
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * VALVE_PITCH_X, 0.0, 0.0)),
        )

        # Cross handle rotates about vertical axle (Z axis)
        handle = model.part(f"{side}_cross_handle")
        # Stem extends down into the valve body top
        handle.visual(
            Cylinder(radius=STEM_R, length=STEM_DEPTH),
            origin=Origin(xyz=(0.0, 0.0, -STEM_DEPTH / 2.0)),
            material=gold,
            name="stem",
        )
        handle.visual(hub_mesh, material=gold, name="hub")

        # Cross spokes lie in horizontal plane (XY), since rotation axis is Z
        spoke_z = HUB_H / 2.0  # spokes at mid-height of hub
        # Spoke rod along X
        handle.visual(
            Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
            origin=Origin(xyz=(0.0, 0.0, spoke_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gold,
            name="spoke_x",
        )
        # Spoke rod along Y
        handle.visual(
            Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
            origin=Origin(xyz=(0.0, 0.0, spoke_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gold,
            name="spoke_y",
        )
        # Rounded spoke tips (4 tips)
        half = HANDLE_ROD_LEN / 2.0
        for name, (dx, dy) in (
            ("tip_px", (half, 0.0)),
            ("tip_nx", (-half, 0.0)),
            ("tip_py", (0.0, half)),
            ("tip_ny", (0.0, -half)),
        ):
            handle.visual(
                Sphere(radius=HANDLE_ROD_R),
                origin=Origin(xyz=(dx, dy, spoke_z)),
                material=gold,
                name=name,
            )

        # Handle origin at top of valve body; joint frame at valve top face
        valve_top_z = SEAM_H + VALVE_ESC_T1 + VALVE_ESC_T2 + VALVE_BODY_H
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
    left_handle = object_model.get_part("left_cross_handle")
    right_handle = object_model.get_part("right_cross_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # --- Joint plan: two independent revolute cross handles, vertical axis (Z),
    # range -180..+180 deg ---
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

    # --- Intentional embedding: handle stems seat into valve body tops ---
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("valve_assembly"),
        reason="valve stem seats into the valve body bore and turns with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("valve_assembly"),
        reason="valve stem seats into the valve body bore and turns with the handle",
    )

    # --- Spout geometry: hollow bore, bridge arch shape ---
    ctx.check(
        "spout_arch_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} m^3 vs unbored={SPOUT_UNBORED_VOLUME:.3e} m^3",
    )

    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb

    # Arch rises above deck
    ctx.check(
        "spout_arch_rises_above_deck",
        sz1 >= 0.06,
        f"spout peak z={sz1:.3f}",
    )
    # Arch base sits at or near deck level (seam ring starts at z=0)
    ctx.check(
        "spout_base_near_deck_surface",
        abs(sz0) < 0.003,
        f"spout zmin={sz0:.4f}",
    )

    # --- Deck base seams exist on all three units ---
    ctx.check(
        "spout_has_base_seams",
        spout.get_visual("base_seams") is not None,
        "spout missing base_seams visual",
    )
    ctx.check(
        "left_valve_has_seam_in_assembly",
        left_valve.get_visual("valve_assembly") is not None,
        "left valve missing valve_assembly",
    )
    ctx.check(
        "right_valve_has_seam_in_assembly",
        right_valve.get_visual("valve_assembly") is not None,
        "right valve missing valve_assembly",
    )

    # --- Three-piece widespread layout: spout between valves ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    sp = ctx.part_world_position(spout)
    assert lv is not None and rv is not None and sp is not None
    ctx.check(
        "three_piece_widespread_layout",
        lv[0] < sp[0] < rv[0],
        f"left_x={lv[0]:.3f}, spout_x={sp[0]:.3f}, right_x={rv[0]:.3f}",
    )
    ctx.check(
        "valves_symmetric_about_center",
        abs(lv[0] + VALVE_PITCH_X) < 0.003
        and abs(rv[0] - VALVE_PITCH_X) < 0.003,
        f"left_x={lv[0]:.3f}, right_x={rv[0]:.3f}",
    )

    # --- All three units mount on the deck surface (above z=0) ---
    # Spout above deck: positive=spout, negative=deck
    ctx.expect_gap(spout, deck, axis="z", max_gap=0.003, max_penetration=0.003,
                   name="spout_seated_on_deck")
    ctx.expect_gap(left_valve, deck, axis="z", max_gap=0.003, max_penetration=0.003,
                   name="left_valve_seated_on_deck")
    ctx.expect_gap(right_valve, deck, axis="z", max_gap=0.003, max_penetration=0.003,
                   name="right_valve_seated_on_deck")

    # --- Handle rotation about vertical axis ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        # At 45 deg the X extent of the cross shrinks
        rest_aabb = ctx.part_world_aabb(left_handle)  # this is the posed AABB
        rot_x = rot_aabb[1][0] - rot_aabb[0][0]
        ctx.check(
            "left_handle_spokes_rotate_off_axis",
            rot_x < 0.085,
            f"x extent at q=45deg is {rot_x:.3f}",
        )
        # Handle stays above its valve
        cen = ctx.part_world_position(left_handle)
        assert cen is not None
        ctx.check(
            "left_handle_stays_above_valve_while_rotating",
            abs(cen[0] - lv[0]) < 0.002 and abs(cen[1]) < 0.002,
            f"handle center=({cen[0]:.3f},{cen[1]:.3f}), valve=({lv[0]:.3f},{lv[1]:.3f})",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        # Quarter turn: cross maps onto itself
        ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.008,
                          name="right_handle_stays_on_valve_at_quarter_turn")

    # --- Overall width about 0.30 m ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert lh_aabb is not None and rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.24 <= total_w <= 0.34,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- Spout arch bridges between the two valve positions ---
    ctx.check(
        "spout_arch_between_valves",
        lv[0] < sx0 and sx1 < rv[0],
        f"spout x=({sx0:.3f},{sx1:.3f}) should be between valves",
    )

    return ctx.report()


object_model = build_object_model()
