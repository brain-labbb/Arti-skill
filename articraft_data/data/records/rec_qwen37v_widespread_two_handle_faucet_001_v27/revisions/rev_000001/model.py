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
# Widespread two-handle bathroom faucet with high swan-neck swivel spout.
# Polished gold brass finish, deck-mounted (countertop installation).
#
# Frame conventions:
#   - The deck/counter is the horizontal XY plane at z = 0 (slab below).
#   - The faucet projects upward (+Z) from the deck surface.
#   - The spout is central at (0, 0); handles flank at x = +/- VALVE_SPACING.
#   - The spout reach direction is +Y (toward the sink basin).
# ---------------------------------------------------------------------------

# Layout
VALVE_SPACING = 0.10  # valve centers at x = +/- 0.10 from spout

# Deck panel (countertop mounting substrate)
DECK_W = 0.38
DECK_D = 0.12
DECK_T = 0.025

# Spout base pedestal (fixed to deck)
SPOUT_FLANGE_R = 0.028
SPOUT_FLANGE_H = 0.006
SPOUT_POST_R = 0.017
SPOUT_POST_H = 0.028
SPOUT_BASE_TOTAL = SPOUT_FLANGE_H + SPOUT_POST_H  # 0.034 m

# Swan-neck spout tube
SPOUT_TUBE_R = 0.011  # outer tube radius (~22 mm diameter)
SPOUT_BORE_R = 0.007  # inner bore
SPOUT_RISE = 0.16  # vertical rise before curve starts
SPOUT_ARCH_PEAK = 0.25  # peak height above deck
SPOUT_REACH_Y = 0.14  # horizontal reach at outlet
SPOUT_OUTLET_Z = 0.10  # outlet height above deck

# Valve assemblies (deck-mounted)
VALVE_ESC_R1, VALVE_ESC_T1 = 0.028, 0.007  # escutcheon base
VALVE_ESC_R2, VALVE_ESC_T2 = 0.021, 0.007  # escutcheon step
VALVE_BODY_R = 0.013
VALVE_BODY_H = 0.030  # valve body above escutcheon
STEM_COLLAR_R = 0.011
STEM_COLLAR_H = 0.010  # visible stem collar height

# Cross handle (horizontal plane, rotates about vertical axis)
HANDLE_ROD_R = 0.004
HANDLE_ROD_LEN = 0.090  # tip-to-tip span
HANDLE_ROD_PLANE_Z = 0.008  # rod center above handle origin
HUB_R = 0.011
HUB_LEN = 0.020
KNURL_R = 0.012
STEM_R = 0.005
STEM_LEN = STEM_COLLAR_H  # seats fully within collar, stops at valve body top

# Computed for hollow-bore test
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_swan_neck() -> cq.Workplane:
    """Swan-neck spout tube in its local frame.
    Origin at the swivel point (top of the spout base post).
    Z is up, Y is reach direction. Path in the YZ plane.
    The tube rises vertically, arches over in a smooth swan curve,
    and descends to an open outlet with visible bore."""

    # Swan-neck path: (y, z) in YZ workplane
    # 1. Vertical rise
    # 2. Smooth curve arching over the top
    # 3. Descending to outlet
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, SPOUT_RISE)
        # Smooth arch: curve up and over
        .threePointArc(
            (0.04, SPOUT_RISE + 0.05),  # midpoint rising
            (0.08, SPOUT_ARCH_PEAK),    # peak of arch
        )
        .threePointArc(
            (0.12, SPOUT_ARCH_PEAK - 0.03),  # past peak, descending
            (SPOUT_REACH_Y, SPOUT_OUTLET_Z),  # outlet position
        )
    )

    # Bore path follows the same curve but extends slightly past both ends
    # for clean open cuts at inlet and outlet
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -0.004)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc(
            (0.04, SPOUT_RISE + 0.05),
            (0.08, SPOUT_ARCH_PEAK),
        )
        .threePointArc(
            (0.12, SPOUT_ARCH_PEAK - 0.03),
            (SPOUT_REACH_Y, SPOUT_OUTLET_Z - 0.004),
        )
    )

    # Profile circle perpendicular to path start tangent (+Z direction)
    tube = cq.Workplane("XY").circle(SPOUT_TUBE_R).sweep(path)
    bore = cq.Workplane("XY").workplane(offset=-0.004).circle(SPOUT_BORE_R).sweep(bore_path)

    # Small collar ring at the base of the spout where it meets the post
    collar = (
        cq.Workplane("XY")
        .circle(SPOUT_TUBE_R + 0.004)
        .circle(SPOUT_TUBE_R - 0.001)
        .extrude(0.010)
    )

    unbored = tube.union(collar)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_hub() -> cq.Workplane:
    """Cross-handle hub in handle local frame: axis +Z, bottom at z=0.
    Knurled (faceted) middle band with domed cap."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_LEN)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.010)
    )
    dome = cq.Workplane("XY").workplane(offset=HUB_LEN - 0.003).sphere(0.010)
    return hub.union(knurl).union(dome)


def _add_valve_visuals(valve, gold) -> None:
    """Valve assembly visuals: stepped escutcheon, valve body, and visible
    stem collar. All stacked upward (+Z) from the valve frame origin on deck."""
    # Escutcheon base ring
    valve.visual(
        Cylinder(radius=VALVE_ESC_R1, length=VALVE_ESC_T1),
        origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_T1 / 2.0)),
        material=gold,
        name="escutcheon_base",
    )
    # Escutcheon step (smaller raised ring)
    valve.visual(
        Cylinder(radius=VALVE_ESC_R2, length=VALVE_ESC_T2),
        origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_T1 + VALVE_ESC_T2 / 2.0)),
        material=gold,
        name="escutcheon_step",
    )
    esc_top = VALVE_ESC_T1 + VALVE_ESC_T2
    # Valve body cylinder
    valve.visual(
        Cylinder(radius=VALVE_BODY_R, length=VALVE_BODY_H),
        origin=Origin(xyz=(0.0, 0.0, esc_top + VALVE_BODY_H / 2.0)),
        material=gold,
        name="valve_body",
    )
    body_top = esc_top + VALVE_BODY_H
    # Visible stem collar: a ring around the stem above the valve body
    valve.visual(
        Cylinder(radius=STEM_COLLAR_R, length=STEM_COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, body_top + STEM_COLLAR_H / 2.0)),
        material=gold,
        name="stem_collar",
    )


def _add_handle_visuals(handle, hub_mesh, gold) -> None:
    """Cross handle visuals in handle local frame: origin at the bottom of
    the stem (handle joint frame), Z up. Four horizontal spokes with sphere
    tips, knurled hub, and a stem that seats down into the stem collar."""
    # Stem: projects downward from handle into the collar bore
    handle.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
        material=gold,
        name="stem",
    )
    # Hub above the stem
    handle.visual(hub_mesh, material=gold, name="hub")
    # Four-arm cross: two perpendicular horizontal spoke rods
    # Spoke 1: along X axis
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, HANDLE_ROD_PLANE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),  # rotate Z-axis cylinder onto X
        ),
        material=gold,
        name="spoke_x",
    )
    # Spoke 2: along Y axis
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, HANDLE_ROD_PLANE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),  # rotate Z-axis cylinder onto Y
        ),
        material=gold,
        name="spoke_y",
    )
    # Rounded sphere tips at spoke ends
    half = HANDLE_ROD_LEN / 2.0
    for name, (dx, dy) in (
        ("tip_pos_x", (half, 0.0)),
        ("tip_neg_x", (-half, 0.0)),
        ("tip_pos_y", (0.0, half)),
        ("tip_neg_y", (0.0, -half)),
    ):
        handle.visual(
            Sphere(radius=HANDLE_ROD_R),
            origin=Origin(xyz=(dx, dy, HANDLE_ROD_PLANE_Z)),
            material=gold,
            name=name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_swan_neck_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    deck_stone = model.material("countertop_granite", rgba=(0.25, 0.22, 0.20, 1.0))

    # --- deck panel (root, countertop substrate) ---
    deck = model.part("deck")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T / 2.0)),  # top surface at z=0
        material=deck_stone,
        name="slab",
    )

    # --- spout base (fixed pedestal on deck) ---
    spout_base = model.part("spout_base")
    # Flange: wider ring sitting on the deck
    spout_base.visual(
        Cylinder(radius=SPOUT_FLANGE_R, length=SPOUT_FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_FLANGE_H / 2.0)),
        material=gold,
        name="flange",
    )
    # Post: cylindrical riser above the flange
    spout_base.visual(
        Cylinder(radius=SPOUT_POST_R, length=SPOUT_POST_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_FLANGE_H + SPOUT_POST_H / 2.0)),
        material=gold,
        name="post",
    )
    model.articulation(
        "deck_to_spout_base",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_base,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # base sits on deck surface
    )

    # --- swan-neck spout (continuous swivel on the base) ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_swan_neck(), "swan_neck_tube"),
        material=gold,
        name="tube",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=spout_base,
        child=spout,
        # Joint at the top of the spout base post; axis is vertical
        origin=Origin(xyz=(0.0, 0.0, SPOUT_BASE_TOTAL)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0),
    )

    # --- valve assemblies (fixed) and cross handles (revolute) ---
    hub_mesh = mesh_from_cadquery(_build_hub(), "handle_hub")
    handle_joint_z = VALVE_ESC_T1 + VALVE_ESC_T2 + VALVE_BODY_H + STEM_COLLAR_H

    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_visuals(valve, gold)
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * VALVE_SPACING, 0.0, 0.0)),
        )

        handle = model.part(f"{side}_handle")
        _add_handle_visuals(handle, hub_mesh, gold)
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            # Joint at the top of the stem collar; vertical rotation axis
            origin=Origin(xyz=(0.0, 0.0, handle_joint_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout_base = object_model.get_part("spout_base")
    spout = object_model.get_part("spout")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    swivel = object_model.get_articulation("spout_swivel")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # --- swivel joint: continuous type, vertical axis ---
    ctx.check(
        "spout_swivel_is_continuous",
        str(swivel.joint_type).lower().endswith("continuous"),
        f"type={swivel.joint_type}",
    )
    sw_ax = swivel.axis
    ctx.check(
        "spout_swivel_axis_is_vertical",
        abs(sw_ax[0]) < 1e-9 and abs(sw_ax[1]) < 1e-9 and abs(sw_ax[2] - 1.0) < 1e-9,
        f"axis={sw_ax}",
    )

    # --- handle joints: revolute, vertical axis, full-turn range ---
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

    # --- swan neck geometry: tall arch, hollow bore ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} vs unbored={SPOUT_UNBORED_VOLUME:.3e}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "swan_neck_reaches_high_arch",
        sz1 >= 0.22,
        f"spout peak z={sz1:.3f} (expected >= 0.22 m)",
    )
    ctx.check(
        "swan_neck_reach_forward",
        sy1 >= 0.10,
        f"spout reach y={sy1:.3f} (expected >= 0.10 m)",
    )
    ctx.check(
        "spout_base_above_deck",
        sz0 >= -0.005,
        f"spout zmin={sz0:.3f} (base sits at deck level)",
    )

    # --- spout base supports spout on deck ---
    ctx.expect_contact(spout_base, deck, name="spout_base_seats_on_deck")
    ctx.expect_gap(spout, spout_base, axis="z", min_gap=-0.002, max_gap=0.005,
                   name="spout_mounts_on_base_top")

    # --- stem collars present on both valves ---
    for side in ("left", "right"):
        valve = object_model.get_part(f"{side}_valve")
        collar = valve.get_visual("stem_collar")
        ctx.check(
            f"{side}_stem_collar_exists",
            collar is not None,
            "stem_collar visual not found",
        )
    # Stem collars are above the valve body
    lv_aabb = ctx.part_world_aabb(left_valve)
    assert lv_aabb is not None
    ctx.check(
        "left_valve_extends_above_deck_with_collar",
        lv_aabb[1][2] >= 0.04,
        f"left valve peak z={lv_aabb[1][2]:.3f} (expected >= 0.04 m for collar)",
    )

    # --- handle stems seat into stem collars (intentional overlap) ---
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("stem_collar"),
        reason="handle stem is seated inside the stem collar bore and turns with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("stem_collar"),
        reason="handle stem is seated inside the stem collar bore and turns with the handle",
    )

    # --- handle overlap with valve proves mounting ---
    ctx.expect_overlap(left_handle, left_valve, axes="xy", min_overlap=0.005,
                       name="left_handle_mounted_on_valve")
    ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.005,
                       name="right_handle_mounted_on_valve")

    # --- three-piece widespread layout ---
    lp = ctx.part_world_position(left_valve)
    rp = ctx.part_world_position(right_valve)
    sp = ctx.part_world_position(spout_base)
    assert lp is not None and rp is not None and sp is not None
    ctx.check(
        "widespread_three_piece_layout",
        abs(lp[0] + VALVE_SPACING) < 1e-6
        and abs(rp[0] - VALVE_SPACING) < 1e-6
        and abs(sp[0]) < 1e-6,
        f"left_x={lp[0]:.4f}, center_x={sp[0]:.4f}, right_x={rp[0]:.4f}",
    )

    # --- handles are above deck, flanking the spout ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert lh_aabb is not None and rh_aabb is not None
    ctx.check(
        "handles_above_deck",
        lh_aabb[0][2] > 0.03 and rh_aabb[0][2] > 0.03,
        f"left_handle zmin={lh_aabb[0][2]:.3f}, right zmin={rh_aabb[0][2]:.3f}",
    )

    # --- cross handle size: about 0.09 m tip-to-tip ---
    ctx.check(
        "cross_handle_about_0p09_tip_to_tip",
        0.085 <= (lh_aabb[1][0] - lh_aabb[0][0]) <= 0.105
        and 0.085 <= (lh_aabb[1][1] - lh_aabb[0][1]) <= 0.105,
        f"handle x={lh_aabb[1][0] - lh_aabb[0][0]:.3f}, y={lh_aabb[1][1] - lh_aabb[0][1]:.3f}",
    )

    # --- overall width about 0.30 m ---
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.27 <= total_w <= 0.33,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- spout swivel proof: rotating the spout moves the outlet ---
    spout_rest_pos = ctx.part_world_position(spout)
    assert spout_rest_pos is not None
    with ctx.pose({swivel: math.pi / 2.0}):
        spout_rot_pos = ctx.part_world_position(spout)
        assert spout_rot_pos is not None
        # After 90° swivel, the reach direction should shift from +Y to -X
        rot_aabb = ctx.part_world_aabb(spout)
        assert rot_aabb is not None
        ctx.check(
            "spout_swivel_rotates_reach_direction",
            rot_aabb[0][0] < -0.05,  # outlet now extends toward -X
            f"after 90° swivel, spout xmin={rot_aabb[0][0]:.3f} (expected < -0.05)",
        )

    # --- handle rotation proof: spokes rotate in horizontal plane ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        cen = ctx.part_world_position(left_handle)
        assert cen is not None
        ctx.check(
            "left_handle_stays_on_valve_axis_while_rotating",
            abs(cen[0] + VALVE_SPACING) < 1e-6,
            f"handle origin x={cen[0]:.4f} (expected {-VALVE_SPACING})",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.005,
                           name="right_handle_stays_mounted_at_quarter_turn")

    # --- deck grounded at z=0 ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_top_surface_at_z_zero",
        abs(deck_aabb[1][2]) < 1e-6,
        f"deck zmax={deck_aabb[1][2]:.4f}",
    )

    # --- valves seat on deck ---
    ctx.expect_gap(left_valve, deck, axis="z", max_gap=0.001, max_penetration=0.001,
                   name="left_valve_seats_on_deck")
    ctx.expect_gap(right_valve, deck, axis="z", max_gap=0.001, max_penetration=0.001,
                   name="right_valve_seats_on_deck")

    return ctx.report()


object_model = build_object_model()
