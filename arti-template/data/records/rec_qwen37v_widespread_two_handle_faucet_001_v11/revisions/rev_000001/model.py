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
# Deck-mounted with tall gooseneck spout and independent lever handles.
#
# Frame conventions:
#   - Deck surface is the horizontal XY plane at z = 0.
#   - Faucet projects upward (+Z) from the deck.
#   - User faces the faucet from +Y; lever arms extend toward +Y.
#   - Spout centered at x = 0; handles flanking at x = ±HANDLE_SPREAD.
# ---------------------------------------------------------------------------

# Layout
HANDLE_SPREAD = 0.14  # x-distance from center to each valve center

# Deck (mounting substrate / countertop section)
DECK_W = 0.44
DECK_D = 0.24
DECK_T = 0.025

# Spout — gooseneck tube with hollow bore
SPOUT_TUBE_R = 0.014
SPOUT_BORE_R = 0.010
SPOUT_RISE = 0.16
SPOUT_PEAK_Z = 0.25
SPOUT_REACH_Y = 0.14
SPOUT_OUTLET_Z = 0.08
SPOUT_FLANGE_R = 0.024
SPOUT_FLANGE_H = 0.008
SPOUT_SHANK_R = 0.010

# Valve assembly
VALVE_ESC_R = 0.024
VALVE_ESC_H = 0.006
VALVE_BODY_R = 0.012
VALVE_BODY_H = 0.030
COLLAR_STEP_R = 0.016
COLLAR_STEP_H = 0.005
COLLAR_R = 0.014
COLLAR_H = 0.010
VALVE_SHANK_R = 0.008

# Lever handle
HUB_R = 0.013
HUB_H = 0.018
LEVER_LEN = 0.065
LEVER_R = 0.005
TIP_R = 0.007
STEM_R = 0.006
STEM_LEN = 0.020  # penetrates through collar into valve body

# Underside hex nuts
SPOUT_NUT_DIA = 0.026
SPOUT_NUT_H = 0.008
VALVE_NUT_DIA = 0.024
VALVE_NUT_H = 0.008

# Joint height above deck (top of stem collar)
JOINT_Z = VALVE_ESC_H + VALVE_BODY_H + COLLAR_STEP_H + COLLAR_H  # 0.051

# Computed volumes for hollow-bore verification
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_solid() -> cq.Workplane:
    """Gooseneck spout in local frame: base at z=0, rises upward, arches
    forward (+Y), curves down to an open outlet with visible hollow bore."""
    # Path in YZ plane: local x = world Y (forward), local y = world Z (up).
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc(
            (0.05, SPOUT_PEAK_Z),
            (SPOUT_REACH_Y, SPOUT_PEAK_Z - 0.03),
        )
        .lineTo(SPOUT_REACH_Y + 0.02, SPOUT_OUTLET_Z)
    )
    # Profile perpendicular to path start tangent (+Z): XY workplane.
    tube = cq.Workplane("XY").circle(SPOUT_TUBE_R).sweep(path)
    flange = cq.Workplane("XY").circle(SPOUT_FLANGE_R).extrude(SPOUT_FLANGE_H)

    unbored = tube.union(flange)

    # Bore follows the same centerline, slightly extended for clean cuts.
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, SPOUT_FLANGE_H)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc(
            (0.05, SPOUT_PEAK_Z),
            (SPOUT_REACH_Y, SPOUT_PEAK_Z - 0.03),
        )
        .lineTo(SPOUT_REACH_Y + 0.02, SPOUT_OUTLET_Z - 0.003)
    )
    bore = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_FLANGE_H)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_hex_nut(dia: float, height: float) -> cq.Workplane:
    """Hex nut prism extruded from z=0 to z=height."""
    return cq.Workplane("XY").polygon(6, dia).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    deck_mat = model.material("deck_cream", rgba=(0.92, 0.91, 0.87, 1.0))

    # --- deck panel (root, mounting substrate) ---
    deck = model.part("deck_panel")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T / 2.0)),
        material=deck_mat,
        name="deck_surface",
    )

    # --- central gooseneck spout (fixed) ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_solid(), "spout"),
        material=gold,
        name="tube",
    )
    # Shank passes through deck to underside nut.
    spout_shank_len = DECK_T + SPOUT_NUT_H + SPOUT_FLANGE_H
    spout.visual(
        Cylinder(radius=SPOUT_SHANK_R, length=spout_shank_len),
        origin=Origin(
            xyz=(0.0, 0.0, -(DECK_T + SPOUT_NUT_H) + spout_shank_len / 2.0)
        ),
        material=gold,
        name="shank",
    )
    spout_nut_mesh = mesh_from_cadquery(
        _build_hex_nut(SPOUT_NUT_DIA, SPOUT_NUT_H), "spout_nut"
    )
    spout.visual(
        spout_nut_mesh,
        origin=Origin(xyz=(0.0, 0.0, -(DECK_T + SPOUT_NUT_H))),
        material=gold,
        name="underside_nut",
    )
    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- valve assemblies and lever handles ---
    valve_nut_mesh = mesh_from_cadquery(
        _build_hex_nut(VALVE_NUT_DIA, VALVE_NUT_H), "valve_nut"
    )

    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")

        # Escutcheon base (flat ring on deck surface).
        valve.visual(
            Cylinder(radius=VALVE_ESC_R, length=VALVE_ESC_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_H / 2.0)),
            material=gold,
            name="escutcheon",
        )
        # Valve body (cylinder rising from escutcheon).
        valve.visual(
            Cylinder(radius=VALVE_BODY_R, length=VALVE_BODY_H),
            origin=Origin(
                xyz=(0.0, 0.0, VALVE_ESC_H + VALVE_BODY_H / 2.0)
            ),
            material=gold,
            name="valve_body",
        )
        # Stem collar step (wider decorative ring).
        valve.visual(
            Cylinder(radius=COLLAR_STEP_R, length=COLLAR_STEP_H),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    VALVE_ESC_H + VALVE_BODY_H + COLLAR_STEP_H / 2.0,
                )
            ),
            material=gold,
            name="collar_step",
        )
        # Stem collar (narrower ring on top of step).
        valve.visual(
            Cylinder(radius=COLLAR_R, length=COLLAR_H),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    VALVE_ESC_H
                    + VALVE_BODY_H
                    + COLLAR_STEP_H
                    + COLLAR_H / 2.0,
                )
            ),
            material=gold,
            name="collar",
        )
        # Shank through deck to underside nut.
        valve_shank_len = DECK_T + VALVE_NUT_H + VALVE_ESC_H
        valve.visual(
            Cylinder(radius=VALVE_SHANK_R, length=valve_shank_len),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    -(DECK_T + VALVE_NUT_H) + valve_shank_len / 2.0,
                )
            ),
            material=gold,
            name="shank",
        )
        # Underside hex nut.
        valve.visual(
            valve_nut_mesh,
            origin=Origin(xyz=(0.0, 0.0, -(DECK_T + VALVE_NUT_H))),
            material=gold,
            name="underside_nut",
        )

        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPREAD, 0.0, 0.0)),
        )

        # --- lever handle (revolute, forward-back tilt about X axis) ---
        handle = model.part(f"{side}_lever")

        # Hub (cylinder sitting on stem collar).
        handle.visual(
            Cylinder(radius=HUB_R, length=HUB_H),
            origin=Origin(xyz=(0.0, 0.0, HUB_H / 2.0)),
            material=gold,
            name="hub",
        )
        # Lever arm (cylinder along +Y from hub, rotated from Z axis).
        arm_y_offset = LEVER_LEN / 2.0 + HUB_R * 0.5
        handle.visual(
            Cylinder(radius=LEVER_R, length=LEVER_LEN),
            origin=Origin(
                xyz=(0.0, arm_y_offset, HUB_H / 2.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=gold,
            name="lever_arm",
        )
        # Lever tip (rounded sphere at arm end).
        handle.visual(
            Sphere(radius=TIP_R),
            origin=Origin(
                xyz=(0.0, LEVER_LEN + HUB_R * 0.5, HUB_H / 2.0)
            ),
            material=gold,
            name="lever_tip",
        )
        # Stem (extends downward into valve body — intentional overlap).
        handle.visual(
            Cylinder(radius=STEM_R, length=STEM_LEN),
            origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
            material=gold,
            name="stem",
        )

        model.articulation(
            f"{side}_lever_joint",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
            # X axis: positive q tilts lever tip up (+Y → +Z).
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-0.3, upper=1.2
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_panel")
    spout = object_model.get_part("spout")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_lever = object_model.get_part("left_lever")
    right_lever = object_model.get_part("right_lever")
    left_joint = object_model.get_articulation("left_lever_joint")
    right_joint = object_model.get_articulation("right_lever_joint")

    # --- joints: two independent revolute levers, forward-back tilt ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_lateral",
            abs(ax[0] - 1.0) < 1e-9
            and abs(ax[1]) < 1e-9
            and abs(ax[2]) < 1e-9,
            f"axis={ax}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_forward_back_range",
            lim is not None and lim.lower < -0.1 and lim.upper > 0.5,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # --- wider handle spread than parent (0.14 vs 0.10) ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "handles_spread_wider",
        abs(lv[0] + HANDLE_SPREAD) < 1e-6
        and abs(rv[0] - HANDLE_SPREAD) < 1e-6,
        f"left_x={lv[0]:.3f}, right_x={rv[0]:.3f}, spread={HANDLE_SPREAD}",
    )
    ctx.check(
        "handle_centers_above_deck",
        abs(lv[2]) < 1e-6 and abs(rv[2]) < 1e-6,
        f"left_z={lv[2]:.4f}, right_z={rv[2]:.4f}",
    )

    # --- taller spout (peak >= 0.22 m above deck) ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "spout_peak_tall",
        sz1 >= 0.22,
        f"spout z_max={sz1:.3f}",
    )
    # Spout outlet drops below peak (visible downward curve).
    ctx.check(
        "spout_outlet_below_peak",
        sz1 - sz0 > 0.10,
        f"spout z range=({sz0:.3f}, {sz1:.3f})",
    )

    # --- hollow bore verification ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} vs unbored={SPOUT_UNBORED_VOLUME:.3e}",
    )

    # --- lever handles have visible arms extending forward ---
    for lever_part, lever_name in (
        (left_lever, "left"),
        (right_lever, "right"),
    ):
        lever_aabb = ctx.part_world_aabb(lever_part)
        assert lever_aabb is not None
        (lx0, ly0, lz0), (lx1, ly1, lz1) = lever_aabb
        ctx.check(
            f"{lever_name}_lever_has_arm",
            (ly1 - ly0) > 0.04,
            f"{lever_name} lever Y extent={ly1 - ly0:.3f}",
        )

    # --- stem collars visible on each valve ---
    for valve, side in ((left_valve, "left"), (right_valve, "right")):
        collar = valve.get_visual("collar")
        collar_step = valve.get_visual("collar_step")
        ctx.check(
            f"{side}_stem_collar_visible",
            collar is not None and collar_step is not None,
            "stem collar or collar step missing",
        )

    # --- underside nuts below deck ---
    for valve, side in ((left_valve, "left"), (right_valve, "right")):
        nut = valve.get_visual("underside_nut")
        ctx.check(
            f"{side}_underside_nut_exists",
            nut is not None,
            "underside nut missing",
        )
    spout_nut = spout.get_visual("underside_nut")
    ctx.check(
        "spout_underside_nut_exists",
        spout_nut is not None,
        "spout underside nut missing",
    )

    # --- intentional overlaps ---
    # Handle stems pass through collar, collar step, and into valve body.
    for lever, valve, side in (
        (left_lever, left_valve, "left"),
        (right_lever, right_valve, "right"),
    ):
        ctx.allow_overlap(
            lever,
            valve,
            elem_a=lever.get_visual("stem"),
            elem_b=valve.get_visual("collar"),
            reason=f"{side} lever stem passes through the stem collar bore",
        )
        ctx.allow_overlap(
            lever,
            valve,
            elem_a=lever.get_visual("stem"),
            elem_b=valve.get_visual("collar_step"),
            reason=f"{side} lever stem passes through the collar step bore",
        )
        ctx.allow_overlap(
            lever,
            valve,
            elem_a=lever.get_visual("stem"),
            elem_b=valve.get_visual("valve_body"),
            reason=f"{side} lever stem seats into the valve body bore",
        )
    # Shanks pass through deck mounting holes.
    ctx.allow_overlap(
        spout,
        deck,
        elem_a=spout.get_visual("shank"),
        elem_b=deck.get_visual("deck_surface"),
        reason="spout shank passes through the deck mounting hole",
    )
    for valve, side in ((left_valve, "left"), (right_valve, "right")):
        ctx.allow_overlap(
            valve,
            deck,
            elem_a=valve.get_visual("shank"),
            elem_b=deck.get_visual("deck_surface"),
            reason=f"{side} valve shank passes through the deck mounting hole",
        )

    # --- prove stem is actually seated in valve body ---
    ctx.expect_overlap(
        left_lever,
        left_valve,
        axes="z",
        elem_a=left_lever.get_visual("stem"),
        elem_b=left_valve.get_visual("valve_body"),
        min_overlap=0.002,
        name="left_stem_seated_in_valve_body",
    )
    ctx.expect_overlap(
        right_lever,
        right_valve,
        axes="z",
        elem_a=right_lever.get_visual("stem"),
        elem_b=right_valve.get_visual("valve_body"),
        min_overlap=0.002,
        name="right_stem_seated_in_valve_body",
    )

    # --- forward-back rotation proof ---
    rest_aabb_l = ctx.part_world_aabb(left_lever)
    assert rest_aabb_l is not None
    rest_z_max_l = rest_aabb_l[1][2]

    with ctx.pose({left_joint: 0.8}):
        tilted_aabb_l = ctx.part_world_aabb(left_lever)
        assert tilted_aabb_l is not None
        tilted_z_max_l = tilted_aabb_l[1][2]
    ctx.check(
        "left_lever_tilts_up_at_positive_q",
        tilted_z_max_l > rest_z_max_l + 0.01,
        f"rest_z_max={rest_z_max_l:.3f}, tilted_z_max={tilted_z_max_l:.3f}",
    )

    rest_aabb_r = ctx.part_world_aabb(right_lever)
    assert rest_aabb_r is not None
    rest_z_max_r = rest_aabb_r[1][2]

    with ctx.pose({right_joint: 0.8}):
        tilted_aabb_r = ctx.part_world_aabb(right_lever)
        assert tilted_aabb_r is not None
        tilted_z_max_r = tilted_aabb_r[1][2]
    ctx.check(
        "right_lever_tilts_up_at_positive_q",
        tilted_z_max_r > rest_z_max_r + 0.01,
        f"rest_z_max={rest_z_max_r:.3f}, tilted_z_max={tilted_z_max_r:.3f}",
    )

    # --- valves flank spout symmetrically ---
    ctx.check(
        "valves_symmetric_about_center",
        abs(lv[0] + rv[0]) < 1e-6,
        f"left_x={lv[0]:.4f}, right_x={rv[0]:.4f}",
    )

    # --- deck panel positioned correctly ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_top_at_z_zero",
        abs(deck_aabb[1][2]) < 1e-6,
        f"deck z_max={deck_aabb[1][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
