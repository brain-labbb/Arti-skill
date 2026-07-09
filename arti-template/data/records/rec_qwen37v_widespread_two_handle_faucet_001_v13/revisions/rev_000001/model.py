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
# Deck-mount style, ~0.30 m wide overall.
#
# Frame conventions:
#   - The deck/counter surface is the horizontal XY plane at z = 0.
#   - Parts project upward (+Z).
#   - The spout curves toward -Y (toward the user).
#   - Left handle at x = -HANDLE_X, right at x = +HANDLE_X.
# ---------------------------------------------------------------------------

# Layout
HANDLE_X = 0.10  # handle centers at x = +/- 0.10

# Deck panel (mounting substrate)
DECK_W = 0.36
DECK_D = 0.18
DECK_T = 0.012

# Spout assembly
SPOUT_BASE_R = 0.025
SPOUT_BASE_H = 0.015
SPOUT_RISER_R = 0.013
SPOUT_RISER_H = 0.12
SPOUT_TUBE_R = 0.013
SPOUT_BORE_R = 0.009
SPOUT_STRAIGHT = 0.10  # horizontal reach from riser top
SPOUT_BEND_R = 0.04
SPOUT_DROP_Z = -0.08  # how far below the bend the outlet drops

# Handle bases
BASE_R = 0.030
BASE_H = 0.015

# Stem collars
COLLAR_R = 0.012
COLLAR_H = 0.028

# Cross handle (axis is Z for deck-mount)
HANDLE_ROD_R = 0.004
HANDLE_ROD_LEN = 0.090  # tip-to-tip ~0.09 m
HUB_R = 0.012
HUB_LEN = 0.022
KNURL_R = 0.0135
STEM_R = 0.006

# Hot/cold cap disks
CAP_R = 0.007
CAP_H = 0.003

# Computed by build for hollow-bore verification
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_solid() -> cq.Workplane:
    """Spout in its local frame: base flange on deck (z=0), riser along +Z,
    then curving outward along -Y and downward to an open outlet.
    The part origin is at the base center on the deck surface."""
    riser_top = SPOUT_BASE_H + SPOUT_RISER_H

    # Build the arm path at origin (Z=0), then translate up to riser_top.
    # Path goes in -Y then curves downward (-Z).
    # Arc center is at (Y=-SPOUT_STRAIGHT, Z=-SPOUT_BEND_R).
    mid_arc = (
        -SPOUT_STRAIGHT - SPOUT_BEND_R * math.sin(math.pi / 4.0),
        -SPOUT_BEND_R + SPOUT_BEND_R * math.cos(math.pi / 4.0),
    )
    end_arc = (
        -SPOUT_STRAIGHT - SPOUT_BEND_R,
        -SPOUT_BEND_R,
    )
    drop_end = (end_arc[0], end_arc[1] + SPOUT_DROP_Z)

    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(-SPOUT_STRAIGHT, 0.0)
        .threePointArc(mid_arc, end_arc)
        .lineTo(drop_end[0], drop_end[1])
    )

    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.004, 0.0)
        .lineTo(-SPOUT_STRAIGHT, 0.0)
        .threePointArc(mid_arc, end_arc)
        .lineTo(drop_end[0], drop_end[1] - 0.003)
    )

    # Sweep profile on ZX workplane (normal +Y), perpendicular to -Y tangent.
    tube = cq.Workplane("ZX").circle(SPOUT_TUBE_R).sweep(path)
    bore = cq.Workplane("ZX").circle(SPOUT_BORE_R).sweep(bore_path)

    # Translate arm up to riser top
    tube = tube.translate((0, 0, riser_top))
    bore = bore.translate((0, 0, riser_top))

    # Base flange
    flange = cq.Workplane("XY").circle(SPOUT_BASE_R).extrude(SPOUT_BASE_H)
    # Riser column
    riser = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_BASE_H)
        .circle(SPOUT_RISER_R)
        .extrude(SPOUT_RISER_H)
    )

    unbored = tube.union(flange).union(riser)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle central hub in the handle frame: axis +Z, base face at
    z=0, with a knurled (faceted) middle band and a chamfered top edge."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_LEN)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.012)
    )
    # Top rim ring to give the hub a finished look (flat top for cap seating)
    top_rim = (
        cq.Workplane("XY")
        .workplane(offset=HUB_LEN - 0.003)
        .circle(HUB_R + 0.001)
        .extrude(0.003)
    )
    return hub.union(knurl).union(top_rim)


def _add_handle_visuals(handle, hub_mesh, gold, cap_material, cap_name) -> None:
    """Cross handle visuals in handle frame (origin at stem top, axis +Z).
    The handle rotates about the Z axis like a deck-mount knob."""
    # Stem (seats into the collar below)
    handle.visual(
        Cylinder(radius=STEM_R, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=gold,
        name="stem",
    )
    # Hub
    handle.visual(hub_mesh, material=gold, name="hub")
    # Cross spokes - two perpendicular rods through the hub
    # Spoke rod along X axis
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, HUB_LEN / 2.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gold,
        name="spoke_x",
    )
    # Spoke rod along Y axis
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, HUB_LEN / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="spoke_y",
    )
    # Rounded spoke tips (4 tips)
    half = HANDLE_ROD_LEN / 2.0
    tip_z = HUB_LEN / 2.0
    for name, (dx, dy) in (
        ("tip_px", (half, 0.0)),
        ("tip_nx", (-half, 0.0)),
        ("tip_py", (0.0, half)),
        ("tip_ny", (0.0, -half)),
    ):
        handle.visual(
            Sphere(radius=HANDLE_ROD_R),
            origin=Origin(xyz=(dx, dy, tip_z)),
            material=gold,
            name=name,
        )
    # Hot/cold cap disk on top of hub - seated into the hub top surface
    handle.visual(
        Cylinder(radius=CAP_R, length=CAP_H),
        origin=Origin(xyz=(0.0, 0.0, HUB_LEN - 0.001 + CAP_H / 2.0)),
        material=cap_material,
        name=cap_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    deck_white = model.material("deck_surface", rgba=(0.92, 0.92, 0.90, 1.0))
    hot_red = model.material("hot_indicator", rgba=(0.80, 0.15, 0.15, 1.0))
    cold_blue = model.material("cold_indicator", rgba=(0.15, 0.30, 0.75, 1.0))

    # --- deck panel (root, mounting substrate) ---
    deck = model.part("deck_panel")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T / 2.0)),
        material=deck_white,
        name="panel",
    )

    # --- central spout (swivels on continuous vertical joint) ---
    spout = model.part("spout")
    spout.visual(mesh_from_cadquery(_build_spout_solid(), "spout_body"), material=gold, name="tube")
    model.articulation(
        "deck_to_spout",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0),
    )

    # --- handle bases (fixed) and cross handles (revolute) ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")

    for side, sx, cap_mat, cap_name in (
        ("left", -1.0, hot_red, "hot_cap"),
        ("right", 1.0, cold_blue, "cold_cap"),
    ):
        # Handle base with stem collar
        base = model.part(f"{side}_base")
        # Round escutcheon base
        base.visual(
            Cylinder(radius=BASE_R, length=BASE_H),
            origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
            material=gold,
            name="escutcheon",
        )
        # Stem collar rising from the base
        base.visual(
            Cylinder(radius=COLLAR_R, length=COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, BASE_H + COLLAR_H / 2.0)),
            material=gold,
            name="stem_collar",
        )
        model.articulation(
            f"deck_to_{side}_base",
            ArticulationType.FIXED,
            parent=deck,
            child=base,
            origin=Origin(xyz=(sx * HANDLE_X, 0.0, 0.0)),
        )

        # Cross handle (revolute about vertical axis)
        handle = model.part(f"{side}_handle")
        _add_handle_visuals(handle, hub_mesh, gold, cap_mat, cap_name)
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=base,
            child=handle,
            # Joint frame at the top of the stem collar
            origin=Origin(xyz=(0.0, 0.0, BASE_H + COLLAR_H)),
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
    left_base = object_model.get_part("left_base")
    right_base = object_model.get_part("right_base")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")
    spout_joint = object_model.get_articulation("deck_to_spout")

    # --- spout swivel: continuous vertical joint ---
    ctx.check(
        "spout_swivel_is_continuous",
        str(spout_joint.joint_type).lower().endswith("continuous"),
        f"type={spout_joint.joint_type}",
    )
    spout_ax = spout_joint.axis
    ctx.check(
        "spout_swivel_axis_is_vertical",
        abs(spout_ax[0]) < 1e-9 and abs(spout_ax[1]) < 1e-9 and abs(spout_ax[2] - 1.0) < 1e-9,
        f"axis={spout_ax}",
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

    # --- stem collars exist under each handle ---
    left_collar = left_base.get_visual("stem_collar")
    right_collar = right_base.get_visual("stem_collar")
    ctx.check(
        "left_stem_collar_exists",
        left_collar is not None,
        "left base missing stem_collar visual",
    )
    ctx.check(
        "right_stem_collar_exists",
        right_collar is not None,
        "right base missing stem_collar visual",
    )

    # --- hot and cold cap disks exist as geometry ---
    hot_cap = left_handle.get_visual("hot_cap")
    cold_cap = right_handle.get_visual("cold_cap")
    ctx.check(
        "hot_cap_disk_exists",
        hot_cap is not None,
        "left handle missing hot_cap visual",
    )
    ctx.check(
        "cold_cap_disk_exists",
        cold_cap is not None,
        "right handle missing cold_cap visual",
    )

    # --- cross handles present with spokes ---
    for handle, side in ((left_handle, "left"), (right_handle, "right")):
        ctx.check(
            f"{side}_handle_has_spokes",
            handle.get_visual("spoke_x") is not None and handle.get_visual("spoke_y") is not None,
            f"{side} handle missing spoke visuals",
        )

    # --- spout is hollow (bore cut through) ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} m^3 vs unbored={SPOUT_UNBORED_VOLUME:.3e} m^3",
    )

    # --- spout rises above the deck and projects outward ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "spout_rises_above_deck",
        sz1 > 0.10,
        f"spout zmax={sz1:.3f}",
    )
    ctx.check(
        "spout_projects_toward_user",
        sy0 < -0.08,
        f"spout ymin={sy0:.3f}",
    )

    # --- handles flank the spout symmetrically ---
    lv = ctx.part_world_position(left_base)
    rv = ctx.part_world_position(right_base)
    assert lv is not None and rv is not None
    ctx.check(
        "handles_flank_spout_symmetrically",
        abs(lv[0] + HANDLE_X) < 1e-4 and abs(rv[0] - HANDLE_X) < 1e-4,
        f"left={lv}, right={rv}",
    )

    # --- handle overlap with base (stem seated in collar) ---
    ctx.allow_overlap(
        left_handle,
        left_base,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_base.get_visual("stem_collar"),
        reason="handle stem seats into the stem collar and rotates with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_base,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_base.get_visual("stem_collar"),
        reason="handle stem seats into the stem collar and rotates with the handle",
    )

    # --- handles project above deck, bases on deck ---
    ctx.expect_gap(left_handle, deck, axis="z", min_gap=0.01, name="left_handle_above_deck")
    ctx.expect_gap(right_handle, deck, axis="z", min_gap=0.01, name="right_handle_above_deck")

    # --- handle rotation proof: spokes rotate off-axis ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_x = rot_aabb[1][0] - rot_aabb[0][0]
        ctx.check(
            "left_handle_spokes_rotate_off_axis",
            rot_x < 0.085,
            f"x extent at q=45deg is {rot_x:.3f} (cross at 45 deg shrinks from ~0.09)",
        )

    # --- spout swivel proof: spout projects in a different direction when rotated ---
    with ctx.pose({spout_joint: math.pi / 2.0}):
        rot_aabb = ctx.part_world_aabb(spout)
        assert rot_aabb is not None
        # After 90 deg rotation about +Z (right-hand rule), -Y becomes +X
        ctx.check(
            "spout_swivels_sideways_at_90deg",
            rot_aabb[1][0] > 0.08,
            f"spout xmax after 90deg swivel = {rot_aabb[1][0]:.3f}",
        )

    # --- overall width about 0.30 m ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert lh_aabb is not None and rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.25 <= total_w <= 0.35,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- deck surface at z=0 (body below the surface) ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_surface_at_z_zero",
        abs(deck_aabb[1][2]) < 0.001,
        f"deck zmax={deck_aabb[1][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
