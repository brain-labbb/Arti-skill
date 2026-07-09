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
#
# Frame conventions:
#   - Deck plate sits on z = 0 (grounded), top surface at z = DECK_H.
#   - X is left-right; handles at x = ±HANDLE_SPREAD.
#   - Y is front-back; user faces from -Y.
#   - Z is up.
#   - Three-piece layout: left handle | central bridge spout | right handle.
# ---------------------------------------------------------------------------

# Layout
HANDLE_SPREAD = 0.10       # handle centers at x = ±0.10 m
DECK_W = 0.30              # deck plate width
DECK_D = 0.070             # deck plate depth
DECK_H = 0.010             # deck plate thickness

# Bridge spout arch
BRIDGE_HALF_SPAN = 0.060   # arch feet at x = ±0.06
BRIDGE_PEAK_Z = 0.055      # arch peak above deck top
BRIDGE_TUBE_R = 0.013      # arch tube outer radius
BRIDGE_BORE_R = 0.008      # bore radius for outlet
RISER_H = 0.022            # riser height (deck top to arch foot)
RISER_R = 0.013            # riser radius (matches tube)
OUTLET_LEN = 0.016         # downward outlet nozzle length

# Handle pedestals
PEDESTAL_R = 0.016         # pedestal body radius
PEDESTAL_H = 0.050         # pedestal height above deck top
RING_H = 0.003             # decorative ring ridge height
RING_EXTRA = 0.002         # ring extends beyond pedestal radius
RING_ZS = (0.010, 0.022, 0.036)  # ring ridge z-positions above deck top

# Seams at deck bases
SEAM_H = 0.001             # seam thickness
SEAM_EXTRA = 0.003         # seam extends beyond element radius

# Lever handles
LEVER_LEN = 0.075          # lever bar length (from pivot)
LEVER_W = 0.012            # lever bar width (grip)
LEVER_H = 0.008            # lever bar thickness
STEM_R = 0.005             # lever stem radius
STEM_LEN = 0.012           # lever stem length (seats into pedestal)

# Volume tracking for hollow bore verification
ARCH_SOLID_VOLUME: float = 0.0
ARCH_UNBORED_VOLUME: float = 0.0


def _build_bridge_arch() -> cq.Workplane:
    """Bridge arch tube with downward outlet nozzle and visible bore.
    
    Built in local frame where z=0 is deck top. The arch is a swept tube
    along a three-point arc, with a short downward outlet at center bottom.
    """
    global ARCH_SOLID_VOLUME, ARCH_UNBORED_VOLUME

    total_span = 2.0 * BRIDGE_HALF_SPAN
    rel_peak = BRIDGE_PEAK_Z - RISER_H  # peak relative to arch feet

    # Arch path: three-point arc in XZ, starting at origin
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .threePointArc((BRIDGE_HALF_SPAN, rel_peak), (total_span, 0.0))
    )

    # Profile at origin on YZ plane, sweep along path
    arch = cq.Workplane("YZ").circle(BRIDGE_TUBE_R).sweep(path)

    # Translate to center (x: shift by -half_span) and raise to riser height
    arch = arch.translate((-BRIDGE_HALF_SPAN, 0.0, RISER_H))

    # Downward outlet nozzle at center bottom of arch
    # Start inside the arch tube (above centerline) for connectivity
    nozzle_start_z = BRIDGE_PEAK_Z + 0.005
    nozzle_bottom_z = BRIDGE_PEAK_Z - BRIDGE_TUBE_R - OUTLET_LEN
    nozzle_r = BRIDGE_TUBE_R * 0.70
    nozzle_len = nozzle_start_z - nozzle_bottom_z
    nozzle = (
        cq.Workplane("XY")
        .workplane(offset=nozzle_start_z)
        .circle(nozzle_r)
        .extrude(-nozzle_len)
    )

    unbored = arch.union(nozzle)
    ARCH_UNBORED_VOLUME = unbored.val().Volume()

    # Bore through outlet nozzle (visible opening at bottom)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=nozzle_start_z + 0.003)
        .circle(BRIDGE_BORE_R)
        .extrude(-(nozzle_len + 0.006))
    )

    solid = unbored.cut(bore)
    ARCH_SOLID_VOLUME = solid.val().Volume()

    return solid


def _build_pedestal() -> cq.Workplane:
    """Handle pedestal with decorative ring ridges and a cap top.
    
    Built in local frame where z=0 is the pedestal base (deck top).
    """
    body = cq.Workplane("XY").circle(PEDESTAL_R).extrude(PEDESTAL_H)

    # Decorative ring ridges
    for rz in RING_ZS:
        ring = (
            cq.Workplane("XY")
            .workplane(offset=rz - RING_H / 2.0)
            .circle(PEDESTAL_R + RING_EXTRA)
            .extrude(RING_H)
        )
        body = body.union(ring)

    # Slight cap/dome at top
    cap = (
        cq.Workplane("XY")
        .workplane(offset=PEDESTAL_H - 0.001)
        .circle(PEDESTAL_R * 0.95)
        .extrude(0.004)
    )
    body = body.union(cap)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold", rgba=(0.85, 0.66, 0.20, 1.0))
    dark_seam = model.material("seam_shadow", rgba=(0.40, 0.32, 0.10, 1.0))
    stone = model.material("deck_stone", rgba=(0.90, 0.88, 0.84, 1.0))

    # --- Deck plate (root, mounting surface) ---
    deck = model.part("deck")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_H)),
        origin=Origin(xyz=(0.0, 0.0, DECK_H / 2.0)),
        material=stone,
        name="plate",
    )

    # --- Bridge spout (fixed to deck) ---
    spout = model.part("spout")
    bridge_mesh = mesh_from_cadquery(_build_bridge_arch(), "bridge_arch")
    spout.visual(bridge_mesh, material=gold, name="arch")

    # Vertical risers from deck top to arch feet
    riser_z_center = RISER_H / 2.0
    for side_name, sx in (("left", -1.0), ("right", 1.0)):
        spout.visual(
            Cylinder(radius=RISER_R, length=RISER_H),
            origin=Origin(xyz=(sx * BRIDGE_HALF_SPAN, 0.0, riser_z_center)),
            material=gold,
            name=f"{side_name}_riser",
        )
        # Seam ring at riser base
        spout.visual(
            Cylinder(radius=RISER_R + SEAM_EXTRA, length=SEAM_H),
            origin=Origin(xyz=(sx * BRIDGE_HALF_SPAN, 0.0, SEAM_H / 2.0)),
            material=dark_seam,
            name=f"{side_name}_seam",
        )

    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, DECK_H)),
    )

    # --- Handle assemblies (pedestal + lever) ---
    ped_mesh = mesh_from_cadquery(_build_pedestal(), "pedestal")

    for side, sx in (("left", -1.0), ("right", 1.0)):
        # Pedestal (fixed to deck)
        pedestal = model.part(f"{side}_pedestal")
        pedestal.visual(ped_mesh, material=gold, name="body")
        # Seam ring at pedestal base
        pedestal.visual(
            Cylinder(radius=PEDESTAL_R + SEAM_EXTRA, length=SEAM_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2.0)),
            material=dark_seam,
            name="base_seam",
        )

        model.articulation(
            f"deck_to_{side}_pedestal",
            ArticulationType.FIXED,
            parent=deck,
            child=pedestal,
            origin=Origin(xyz=(sx * HANDLE_SPREAD, 0.0, DECK_H)),
        )

        # Lever handle (revolute: forward-back tilt)
        lever = model.part(f"{side}_lever")
        # Lever bar extending upward from pivot
        lever.visual(
            Box((LEVER_W, LEVER_H, LEVER_LEN)),
            origin=Origin(xyz=(0.0, 0.0, LEVER_LEN / 2.0)),
            material=gold,
            name="bar",
        )
        # Rounded cap at lever tip
        lever.visual(
            Sphere(radius=LEVER_W * 0.55),
            origin=Origin(xyz=(0.0, 0.0, LEVER_LEN)),
            material=gold,
            name="cap",
        )
        # Stem seated into pedestal bore
        lever.visual(
            Cylinder(radius=STEM_R, length=STEM_LEN),
            origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
            material=gold,
            name="stem",
        )

        model.articulation(
            f"{side}_lever_pivot",
            ArticulationType.REVOLUTE,
            parent=pedestal,
            child=lever,
            # Pivot at the pedestal top (+ cap height)
            origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H + 0.003)),
            # Axis along X: positive q tilts lever forward (toward -Y / user)
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0, velocity=2.0, lower=0.0, upper=1.2
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout = object_model.get_part("spout")
    left_ped = object_model.get_part("left_pedestal")
    right_ped = object_model.get_part("right_pedestal")
    left_lever = object_model.get_part("left_lever")
    right_lever = object_model.get_part("right_lever")
    left_joint = object_model.get_articulation("left_lever_pivot")
    right_joint = object_model.get_articulation("right_lever_pivot")

    # --- Joint type and axis: two independent revolute lever pivots ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_lateral",
            abs(ax[0] - 1.0) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2]) < 1e-9,
            f"axis={ax}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_forward_tilt_limits",
            lim is not None
            and abs(lim.lower) < 1e-6
            and abs(lim.upper - 1.2) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # --- Lever stems seated into pedestal bores ---
    ctx.allow_overlap(
        left_lever,
        left_ped,
        elem_a=left_lever.get_visual("stem"),
        elem_b=left_ped.get_visual("body"),
        reason="lever stem seats into the pedestal bore and pivots with the handle",
    )
    ctx.allow_overlap(
        right_lever,
        right_ped,
        elem_a=right_lever.get_visual("stem"),
        elem_b=right_ped.get_visual("body"),
        reason="lever stem seats into the pedestal bore and pivots with the handle",
    )

    # Prove stem contact with pedestal
    ctx.expect_contact(
        left_lever,
        left_ped,
        elem_a=left_lever.get_visual("stem"),
        elem_b=left_ped.get_visual("body"),
        contact_tol=0.002,
        name="left_stem_contacts_pedestal",
    )
    ctx.expect_contact(
        right_lever,
        right_ped,
        elem_a=right_lever.get_visual("stem"),
        elem_b=right_ped.get_visual("body"),
        contact_tol=0.002,
        name="right_stem_contacts_pedestal",
    )

    # --- Lever forward tilt proof: positive q moves tip toward user (-Y) ---
    with ctx.pose({left_joint: 0.0}):
        rest_aabb = ctx.part_world_aabb(left_lever)
    with ctx.pose({left_joint: 0.6}):
        tilted_aabb = ctx.part_world_aabb(left_lever)
    assert rest_aabb is not None and tilted_aabb is not None
    # At rest the lever extends upward; its y-min is near the pivot.
    # At tilt=0.6 rad the tip swings forward, so y-min should decrease.
    ctx.check(
        "left_lever_tilts_forward",
        tilted_aabb[0][1] < rest_aabb[0][1] - 0.008,
        f"rest_ymin={rest_aabb[0][1]:.4f}, tilted_ymin={tilted_aabb[0][1]:.4f}",
    )

    # --- Bridge arch spans between handles ---
    arch_aabb = ctx.part_element_world_aabb(spout, elem="arch")
    assert arch_aabb is not None
    (ax0, ay0, az0), (ax1, ay1, az1) = arch_aabb
    ctx.check(
        "bridge_arch_spans_between_handles",
        ax0 < -0.04 and ax1 > 0.04,
        f"arch x extent=({ax0:.3f}, {ax1:.3f})",
    )
    ctx.check(
        "bridge_arch_rises_above_deck",
        az1 > DECK_H + 0.030,
        f"arch peak z={az1:.3f}",
    )

    # --- Hollow bore at outlet ---
    ctx.check(
        "outlet_has_hollow_bore",
        0.0 < ARCH_SOLID_VOLUME < 0.98 * ARCH_UNBORED_VOLUME,
        f"solid={ARCH_SOLID_VOLUME:.3e} vs unbored={ARCH_UNBORED_VOLUME:.3e}",
    )

    # --- Decorative ring ridges widen pedestal beyond body radius ---
    ped_aabb = ctx.part_element_world_aabb(left_ped, elem="body")
    assert ped_aabb is not None
    (px0, py0, pz0), (px1, py1, pz1) = ped_aabb
    ped_width_x = px1 - px0
    body_diameter = 2.0 * PEDESTAL_R
    ctx.check(
        "left_pedestal_has_ring_ridges",
        ped_width_x > body_diameter + RING_EXTRA * 0.5,
        f"pedestal x width={ped_width_x:.4f} vs body diameter={body_diameter:.4f}",
    )

    right_ped_aabb = ctx.part_element_world_aabb(right_ped, elem="body")
    assert right_ped_aabb is not None
    rpw = right_ped_aabb[1][0] - right_ped_aabb[0][0]
    ctx.check(
        "right_pedestal_has_ring_ridges",
        rpw > body_diameter + RING_EXTRA * 0.5,
        f"pedestal x width={rpw:.4f} vs body diameter={body_diameter:.4f}",
    )

    # --- Seams at all three deck bases ---
    ctx.check(
        "left_pedestal_has_base_seam",
        left_ped.get_visual("base_seam") is not None,
    )
    ctx.check(
        "right_pedestal_has_base_seam",
        right_ped.get_visual("base_seam") is not None,
    )
    # Spout has seams at both riser feet
    ctx.check(
        "spout_has_riser_seams",
        spout.get_visual("left_seam") is not None
        and spout.get_visual("right_seam") is not None,
    )

    # --- Three-piece widespread layout ---
    lp = ctx.part_world_position(left_ped)
    rp = ctx.part_world_position(right_ped)
    sp = ctx.part_world_position(spout)
    assert lp is not None and rp is not None and sp is not None
    ctx.check(
        "three_piece_layout_spout_centered",
        abs(sp[0]) < 0.01 and lp[0] < sp[0] - 0.02 and sp[0] + 0.02 < rp[0],
        f"left_x={lp[0]:.3f}, spout_x={sp[0]:.3f}, right_x={rp[0]:.3f}",
    )
    ctx.check(
        "handles_at_correct_spread",
        abs(lp[0] + HANDLE_SPREAD) < 0.005
        and abs(rp[0] - HANDLE_SPREAD) < 0.005,
        f"left_x={lp[0]:.3f}, right_x={rp[0]:.3f}",
    )

    # --- Deck grounded at z=0 ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_grounded",
        abs(deck_aabb[0][2]) < 1e-6,
        f"deck zmin={deck_aabb[0][2]:.4f}",
    )

    # --- Lever stays mounted while tilting ---
    with ctx.pose({right_joint: 1.0}):
        ctx.expect_overlap(
            right_lever,
            right_ped,
            axes="xy",
            min_overlap=0.002,
            name="right_lever_stays_mounted_at_full_tilt",
        )

    # --- Overall width reasonable for widespread faucet ---
    ll_aabb = ctx.part_world_aabb(left_lever)
    rl_aabb = ctx.part_world_aabb(right_lever)
    assert ll_aabb is not None and rl_aabb is not None
    total_w = rl_aabb[1][0] - ll_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.20 <= total_w <= 0.35,
        f"lever-tip to lever-tip width={total_w:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
