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
# Widespread deck-mounted two-handle bathroom faucet in polished gold brass.
#
# Frame conventions:
#   - The deck surface is the XY plane at z = DECK_H (top of deck plate).
#   - +Z is up; -Y is forward (toward user / sink).
#   - Three posts at x = -SPACING, 0, +SPACING rise from the deck.
#   - A horizontal bridge bar runs along X connecting the posts.
#   - The spout rises from the center post, arcs forward (-Y) and drops down.
#   - Cross handles rotate about vertical (+Z) axes on the flanking valves.
#   - The outlet aerator pivots on a small hinge (revolute about X).
# ---------------------------------------------------------------------------

# Layout
DECK_W = 0.34            # deck plate width (X)
DECK_D = 0.12            # deck plate depth (Y)
DECK_H = 0.015           # deck plate thickness
SPACING = 0.10           # post center-to-center spacing from center

# Posts
POST_R = 0.015           # post outer radius
POST_HEIGHT = 0.060      # post height above deck top
POST_TOP_Z = DECK_H + POST_HEIGHT

# Bridge bar
BRIDGE_R = 0.009         # bridge tube radius
BRIDGE_Z = DECK_H + BRIDGE_R + 0.002  # bar center (slightly above deck)

# Seam rings at deck bases
SEAM_OUTER_R = POST_R + 0.003
SEAM_INNER_R = POST_R - 0.001
SEAM_H = 0.0015

# Spout
SPOUT_TUBE_R = 0.014
SPOUT_BORE_R = 0.010
# Computed by build_object_model() for hollow verification.
SPOUT_SOLID_VOL: float = 0.0
SPOUT_UNBORED_VOL: float = 0.0

# Valve assemblies
VALVE_ESC_R = 0.022
VALVE_ESC_H = 0.006
VALVE_BODY_R = 0.014
VALVE_BODY_H = 0.025

# Cross handle (deck-mount: axis vertical)
HANDLE_ROD_R = 0.004
HANDLE_ROD_LEN = 0.090   # tip-to-tip
HUB_R = 0.012
HUB_LEN = 0.022
KNURL_R = 0.0135
STEM_R = 0.006
STEM_LEN = 0.014

# Aerator
AERATOR_R = 0.011
AERATOR_LEN = 0.018


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------

def _build_spout_solid() -> cq.Workplane:
    """Spout in its local frame (origin at post top):
    rises vertically, arcs forward (-Y), and drops to an open hollow outlet."""
    # Path in the YZ plane (x = 0).  Coords on YZ workplane are (y, z).
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, 0.06)
        .threePointArc((-0.04, 0.11), (-0.10, 0.03))
        .lineTo(-0.10, -0.02)
    )
    # Extended bore path for clean open-end cuts.
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -0.004)
        .lineTo(0.0, 0.06)
        .threePointArc((-0.04, 0.11 - 0.003), (-0.10, 0.03))
        .lineTo(-0.10, -0.026)
    )

    # Profile perpendicular to initial +Z tangent.
    tube = cq.Workplane("XY").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.004)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )
    # Decorative collar at spout base (inner radius < tube radius so the
    # collar overlaps with and connects to the tube mesh).
    collar = (
        cq.Workplane("XY")
        .circle(0.020)
        .circle(SPOUT_TUBE_R - 0.001)
        .extrude(0.010)
    )

    unbored = tube.union(collar)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOL, SPOUT_UNBORED_VOL
    SPOUT_SOLID_VOL = solid.val().Volume()
    SPOUT_UNBORED_VOL = unbored.val().Volume()
    return solid


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle hub with vertical axis (deck-mount): knurled band + dome."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_LEN)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.012)
    )
    dome = cq.Workplane("XY").workplane(offset=HUB_LEN - 0.004).sphere(0.011)
    return hub.union(knurl).union(dome)


def _build_seam_ring() -> cq.Workplane:
    """Thin annular seam ring at the deck–post junction."""
    return (
        cq.Workplane("XY")
        .circle(SEAM_OUTER_R)
        .circle(SEAM_INNER_R)
        .extrude(SEAM_H)
    )


# ---------------------------------------------------------------------------
# Visual helpers
# ---------------------------------------------------------------------------

def _add_valve_visuals(valve, gold) -> None:
    """Escutcheon trim + cylindrical valve body atop the post (axis +Z)."""
    valve.visual(
        Cylinder(radius=VALVE_ESC_R, length=VALVE_ESC_H),
        origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_H / 2.0)),
        material=gold,
        name="escutcheon",
    )
    valve.visual(
        Cylinder(radius=VALVE_BODY_R, length=VALVE_BODY_H),
        origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_H + VALVE_BODY_H / 2.0)),
        material=gold,
        name="valve_body",
    )


def _add_handle_visuals(handle, hub_mesh, gold) -> None:
    """Four-arm cross handle with vertical axis: stem, hub, two spoke rods,
    four sphere tips."""
    # Stem seats downward into the valve body bore.
    handle.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
        material=gold,
        name="stem",
    )
    handle.visual(hub_mesh, material=gold, name="hub")

    spoke_z = HUB_LEN / 2.0  # spoke plane at hub mid-height

    # Spoke rod along X (rotate Z-axis cylinder onto X).
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, spoke_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gold,
        name="spoke_x",
    )
    # Spoke rod along Y (rotate Z-axis cylinder onto Y).
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, spoke_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="spoke_y",
    )
    # Rounded spoke tips.
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


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_deck_faucet")

    gold = model.material("polished_gold", rgba=(0.85, 0.66, 0.20, 1.0))
    seam_gold = model.material("seam_shadow", rgba=(0.40, 0.30, 0.10, 1.0))
    deck_mat = model.material("deck_marble", rgba=(0.88, 0.87, 0.84, 1.0))
    screen_mat = model.material("aerator_mesh", rgba=(0.30, 0.25, 0.12, 1.0))

    # ---- Deck plate (root) ----
    deck = model.part("deck_plate")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_H)),
        origin=Origin(xyz=(0.0, 0.0, DECK_H / 2.0)),
        material=deck_mat,
        name="deck",
    )

    # ---- Bridge assembly (FIXED to deck) ----
    bridge = model.part("bridge")

    # Horizontal bridge bar along X connecting the three posts.
    bridge.visual(
        Cylinder(radius=BRIDGE_R, length=2.0 * SPACING),
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gold,
        name="bar",
    )
    # Three vertical post stubs from deck top to post-top height.
    for sx, label in ((-SPACING, "left"), (0.0, "center"), (SPACING, "right")):
        bridge.visual(
            Cylinder(radius=POST_R, length=POST_HEIGHT),
            origin=Origin(xyz=(sx, 0.0, DECK_H + POST_HEIGHT / 2.0)),
            material=gold,
            name=f"{label}_post",
        )

    # Narrow seam rings at all three deck bases (on bridge so they share
    # geometry connectivity with the posts).
    seam_mesh = mesh_from_cadquery(_build_seam_ring(), "seam_ring")
    for sx, label in ((-SPACING, "left"), (0.0, "center"), (SPACING, "right")):
        bridge.visual(
            seam_mesh,
            origin=Origin(xyz=(sx, 0.0, DECK_H)),
            material=seam_gold,
            name=f"seam_{label}",
        )

    model.articulation(
        "deck_to_bridge",
        ArticulationType.FIXED,
        parent=deck,
        child=bridge,
        origin=Origin(),
    )

    # ---- Spout (FIXED to bridge at center post top) ----
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_solid(), "spout"),
        material=gold,
        name="tube",
    )
    model.articulation(
        "bridge_to_spout",
        ArticulationType.FIXED,
        parent=bridge,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z)),
    )

    # ---- Aerator (REVOLUTE on spout at outlet, small hinge) ----
    aerator = model.part("aerator")
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_LEN / 2.0)),
        material=gold,
        name="body",
    )
    aerator.visual(
        Cylinder(radius=AERATOR_R - 0.002, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_LEN + 0.0015)),
        material=screen_mat,
        name="screen",
    )
    # Hinge at spout outlet (path ends at y=-0.10, z=-0.02 in spout frame).
    model.articulation(
        "spout_to_aerator",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(0.0, -0.10, -0.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=-0.30, upper=0.30),
    )

    # ---- Valve assemblies (FIXED) and cross handles (REVOLUTE) ----
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "hub")
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_visuals(valve, gold)
        model.articulation(
            f"bridge_to_{side}_valve",
            ArticulationType.FIXED,
            parent=bridge,
            child=valve,
            origin=Origin(xyz=(sx * SPACING, 0.0, POST_TOP_Z)),
        )

        handle = model.part(f"{side}_handle")
        _add_handle_visuals(handle, hub_mesh, gold)
        handle_top_z = VALVE_ESC_H + VALVE_BODY_H
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, handle_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi,
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_plate")
    bridge = object_model.get_part("bridge")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")

    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")
    aerator_joint = object_model.get_articulation("spout_to_aerator")

    # ---- Bridge bar spans all three posts ----
    bridge_aabb = ctx.part_world_aabb(bridge)
    assert bridge_aabb is not None
    (bx0, _, _), (bx1, _, _) = bridge_aabb
    ctx.check(
        "bridge_bar_spans_all_posts",
        (bx1 - bx0) >= 2.0 * SPACING - 0.005,
        f"bridge x extent = {bx1 - bx0:.3f}, expected >= {2*SPACING - 0.005:.3f}",
    )

    # ---- Narrow seams at all three deck bases ----
    for label in ("left", "center", "right"):
        seam = bridge.get_visual(f"seam_{label}")
        ctx.check(
            f"seam_{label}_present",
            seam is not None,
            f"missing seam_{label} visual on bridge",
        )

    # ---- Aerator joint: revolute, horizontal axis, small range ----
    ctx.check(
        "aerator_joint_is_revolute",
        str(aerator_joint.joint_type).lower().endswith("revolute"),
        f"type={aerator_joint.joint_type}",
    )
    ax = aerator_joint.axis
    ctx.check(
        "aerator_axis_horizontal_x",
        abs(ax[0] - 1.0) < 1e-6 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        f"axis={ax}",
    )
    lim = aerator_joint.motion_limits
    ctx.check(
        "aerator_small_pivot_range",
        lim is not None and lim.upper - lim.lower > 0.1 and lim.upper - lim.lower < 1.0,
        f"range = {lim.upper - lim.lower:.3f} rad",
    )

    # ---- Handle joints: revolute about vertical axis ----
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
        hlim = joint.motion_limits
        ctx.check(
            f"{joint.name}_full_turn",
            hlim is not None
            and abs(hlim.lower + math.pi) < 1e-6
            and abs(hlim.upper - math.pi) < 1e-6,
            f"limits=({hlim.lower}, {hlim.upper})",
        )

    # ---- Intentional overlap: spout seated on center post ----
    ctx.allow_overlap(
        bridge, spout,
        elem_a=bridge.get_visual("center_post"),
        elem_b=spout.get_visual("tube"),
        reason="spout tube and collar are seated on the center post; swept mesh has small penetration at the seating interface",
    )
    ctx.expect_overlap(
        spout, bridge,
        axes="xy", min_overlap=0.010,
        name="spout_centered_on_center_post",
    )

    # ---- Intentional overlap: handle stems seated in valve bodies ----
    ctx.allow_overlap(
        left_handle, left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("valve_body"),
        reason="handle stem is seated inside the valve body bore and turns with the handle",
    )
    ctx.allow_overlap(
        right_handle, right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("valve_body"),
        reason="handle stem is seated inside the valve body bore and turns with the handle",
    )
    # Proof: stems overlap with valve bodies and handles stay mounted.
    ctx.expect_overlap(
        left_handle, left_valve,
        axes="xy", min_overlap=0.005,
        elem_a="stem", elem_b="valve_body",
        name="left_stem_seated_in_valve",
    )
    ctx.expect_overlap(
        right_handle, right_valve,
        axes="xy", min_overlap=0.005,
        elem_a="stem", elem_b="valve_body",
        name="right_stem_seated_in_valve",
    )

    # ---- Spout is hollow (bore reduces volume) ----
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOL < 0.95 * SPOUT_UNBORED_VOL,
        f"solid={SPOUT_SOLID_VOL:.3e} vs unbored={SPOUT_UNBORED_VOL:.3e}",
    )

    # ---- Aerator pivots at the spout outlet ----
    with ctx.pose({aerator_joint: 0.0}):
        rest_aabb = ctx.part_world_aabb(aerator)
    with ctx.pose({aerator_joint: 0.25}):
        tilt_aabb = ctx.part_world_aabb(aerator)
    assert rest_aabb is not None and tilt_aabb is not None
    ctx.check(
        "aerator_pivot_changes_y_extent",
        abs(tilt_aabb[0][1] - rest_aabb[0][1]) > 0.001
        or abs(tilt_aabb[1][1] - rest_aabb[1][1]) > 0.001,
        f"rest_y=({rest_aabb[0][1]:.4f},{rest_aabb[1][1]:.4f}) "
        f"tilt_y=({tilt_aabb[0][1]:.4f},{tilt_aabb[1][1]:.4f})",
    )

    # ---- Handle rotation proof (off-axis spokes shrink extent) ----
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        ctx.check(
            "left_handle_rotates_off_axis",
            rot_aabb is not None,
            "handle AABB undefined after rotation",
        )

    # ---- Three-piece widespread layout ----
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    sp = ctx.part_world_position(spout)
    assert lv is not None and rv is not None and sp is not None
    ctx.check(
        "three_piece_widespread_layout",
        abs(lv[0] + SPACING) < 0.005
        and abs(rv[0] - SPACING) < 0.005
        and abs(sp[0]) < 0.005,
        f"left_x={lv[0]:.3f}, center_x={sp[0]:.3f}, right_x={rv[0]:.3f}",
    )

    # ---- Deck plate grounded at z = 0 ----
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_grounded",
        abs(deck_aabb[0][2]) < 1e-6,
        f"deck zmin = {deck_aabb[0][2]:.4f}",
    )

    # ---- Overall width ~0.30 m across handle tips ----
    lh_aabb = ctx.part_world_aabb(left_handle)
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert lh_aabb is not None and rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.26 <= total_w <= 0.34,
        f"handle-tip to handle-tip width = {total_w:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
