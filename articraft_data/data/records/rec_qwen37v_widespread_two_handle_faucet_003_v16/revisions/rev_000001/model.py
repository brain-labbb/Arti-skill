from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet with bridge bar.

Three-piece widespread layout on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (revolute about column vertical axis, -45..+45 deg),
- hot (left) and cold (right): valve columns topped by cross handles
  (each revolute about its column vertical axis, -90..+90 deg),
- deck-mounted bridge bar visually linking the three posts.

Narrow seams at all three deck bases; decorative ring ridges on the
handle pedestals.  Cross handles rotate around short vertical axles.

All surfaces matte black; red/blue indicator dots on handle hubs.
Modeled at true scale in metres; deck bottom on z = 0.
"""

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

# ----------------------------------------------------------------- globals
DECK_X = 0.46
DECK_Y = 0.18
DECK_T = 0.02

SPREAD_HALF = 0.15  # valve centres at x = ±0.15

# ── centre spout ───────────────────────────────────────────────────────
SPOUT_FLANGE_R = 0.042
SPOUT_FLANGE_H = 0.012
SPOUT_COL_R = 0.025
SPOUT_COL_H = 0.12

# ── gooseneck (spout-part frame, origin at column top) ─────────────────
TUBE_R = 0.0155
RISER_EMBED = 0.03
RISER_TOP = 0.14
ARC_R = 0.062
HOOK_DEG = -12.0
COLLAR_R = 0.020
COLLAR_H = 0.016

# ── valve columns ──────────────────────────────────────────────────────
VALVE_FLANGE_R = 0.036
VALVE_FLANGE_H = 0.010
VALVE_COL_R = 0.0225
VALVE_COL_H = 0.10

# ── bridge bar ─────────────────────────────────────────────────────────
BRIDGE_LEN = 0.26   # full x-extent
BRIDGE_W = 0.016
BRIDGE_H = 0.008

# ── cross handle (handle-part frame, origin at valve column top) ───────
CROSS_STEM_R = 0.009
CROSS_STEM_EMBED = 0.015
CROSS_STEM_RISE = 0.025
CROSS_HUB_R = 0.013
CROSS_HUB_H = 0.008
CROSS_ARM_R = 0.007
CROSS_ARM_HALF = 0.048
DOT_R = 0.0035

# ── seam rings ─────────────────────────────────────────────────────────
SEAM_H = 0.002
SEAM_SPOUT_R = SPOUT_FLANGE_R + 0.003
SEAM_VALVE_R = VALVE_FLANGE_R + 0.003

# ── decorative rings ───────────────────────────────────────────────────
RING_MAJOR_R = VALVE_COL_R + 0.002   # torus centre-line radius
RING_TUBE_R = 0.003                  # torus tube radius
RING_Z_POS = (0.033, 0.066)          # heights on valve column

# ── aerator at gooseneck hook tip ──────────────────────────────────────
ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.016
AERATOR_R = 0.017
_TX = math.sin(math.radians(HOOK_DEG))
_TZ = -math.cos(math.radians(HOOK_DEG))
AERATOR_CY = ARC_END_Y + _TX * (AERATOR_LEN / 2 - 0.004)
AERATOR_CZ = ARC_END_Z + _TZ * (AERATOR_LEN / 2 - 0.004)


# ---------------------------------------------------------- CQ helpers
def _gooseneck_solid() -> cq.Workplane:
    """Swept gooseneck tube: straight riser + forward-down arc."""
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -RISER_EMBED)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (ARC_END_Y, ARC_END_Z))
    )
    profile = cq.Workplane("XY").workplane(offset=-RISER_EMBED).circle(TUBE_R)
    return profile.sweep(path, isFrenet=True)


def _bridge_bar_mesh() -> cq.Workplane:
    """Horizontal bridge bar with slightly rounded vertical edges."""
    return (
        cq.Workplane("XY")
        .box(BRIDGE_LEN, BRIDGE_W, BRIDGE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.002)
    )


# ================================================================= build
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_bridge_bathroom_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.08, 0.08, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.03, 0.03, 0.03, 1.0))

    # ── sink deck ──────────────────────────────────────────────────────
    sink_deck = model.part("sink_deck")
    sink_deck.visual(
        Box((DECK_X, DECK_Y, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, DECK_T / 2)),
        material=deck_stone,
        name="deck_slab",
    )

    # ── bridge bar ────────────────────────────────────────────────────
    bridge_bar = model.part("bridge_bar")
    bridge_bar.visual(
        mesh_from_cadquery(_bridge_bar_mesh(), "bridge_arm"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="bridge_arm",
    )
    model.articulation(
        "deck_to_bridge",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=bridge_bar,
        origin=Origin(xyz=(0.0, 0.0, DECK_T)),
    )

    # ── centre spout base (with seam) ─────────────────────────────────
    spout_base = model.part("spout_base")
    spout_base.visual(
        Cylinder(radius=SEAM_SPOUT_R, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2)),
        material=seam_dark,
        name="deck_seam",
    )
    spout_base.visual(
        Cylinder(radius=SPOUT_FLANGE_R, length=SPOUT_FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_FLANGE_H / 2)),
        material=matte_black,
        name="base_flange",
    )
    spout_base.visual(
        Cylinder(radius=SPOUT_COL_R, length=SPOUT_COL_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_COL_H / 2)),
        material=matte_black,
        name="base_column",
    )
    model.articulation(
        "deck_to_spout_base",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=spout_base,
        origin=Origin(xyz=(0.0, 0.0, DECK_T)),
    )

    # ── gooseneck spout ───────────────────────────────────────────────
    gooseneck_spout = model.part("gooseneck_spout")
    gooseneck_spout.visual(
        mesh_from_cadquery(_gooseneck_solid(), "gooseneck_tube"),
        material=matte_black,
        name="spout_tube",
    )
    gooseneck_spout.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H / 2)),
        material=matte_black,
        name="swivel_collar",
    )
    gooseneck_spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(
            xyz=(0.0, AERATOR_CY, AERATOR_CZ),
            rpy=(math.radians(HOOK_DEG), 0.0, 0.0),
        ),
        material=matte_black,
        name="aerator",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=spout_base,
        child=gooseneck_spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_COL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-math.pi / 4,
            upper=math.pi / 4,
        ),
    )

    # ── valve columns (seams + decorative rings) ──────────────────────
    def _valve_column(name: str) -> object:
        col = model.part(name)
        # Seam ring at deck junction
        col.visual(
            Cylinder(radius=SEAM_VALVE_R, length=SEAM_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2)),
            material=seam_dark,
            name="deck_seam",
        )
        # Flange
        col.visual(
            Cylinder(radius=VALVE_FLANGE_R, length=VALVE_FLANGE_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_FLANGE_H / 2)),
            material=matte_black,
            name="valve_flange",
        )
        # Column body
        col.visual(
            Cylinder(radius=VALVE_COL_R, length=VALVE_COL_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H / 2)),
            material=matte_black,
            name="valve_body",
        )
        # Decorative ring ridges (thin wider cylinders on column surface)
        ring_r = RING_MAJOR_R + RING_TUBE_R  # outer radius
        ring_h = 2 * RING_TUBE_R             # thickness
        for i, z_pos in enumerate(RING_Z_POS):
            col.visual(
                Cylinder(radius=ring_r, length=ring_h),
                origin=Origin(xyz=(0.0, 0.0, z_pos)),
                material=matte_black,
                name=f"deco_ring_{i}",
            )
        return col

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")

    model.articulation(
        "deck_to_hot_valve",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=hot_valve_column,
        origin=Origin(xyz=(-SPREAD_HALF, 0.0, DECK_T)),
    )
    model.articulation(
        "deck_to_cold_valve",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=cold_valve_column,
        origin=Origin(xyz=(SPREAD_HALF, 0.0, DECK_T)),
    )

    # ── cross handles ─────────────────────────────────────────────────
    def _cross_handle(name: str, dot_material: object) -> object:
        handle = model.part(name)
        arm_z = CROSS_STEM_RISE + CROSS_HUB_H / 2
        stem_len = CROSS_STEM_RISE + CROSS_STEM_EMBED

        # Stem (seats into column bore)
        handle.visual(
            Cylinder(radius=CROSS_STEM_R, length=stem_len),
            origin=Origin(xyz=(0.0, 0.0, (CROSS_STEM_RISE - CROSS_STEM_EMBED) / 2)),
            material=matte_black,
            name="handle_stem",
        )
        # Hub
        handle.visual(
            Cylinder(radius=CROSS_HUB_R, length=CROSS_HUB_H),
            origin=Origin(xyz=(0.0, 0.0, arm_z)),
            material=matte_black,
            name="handle_hub",
        )
        # Arm along X
        handle.visual(
            Cylinder(radius=CROSS_ARM_R, length=2 * CROSS_ARM_HALF),
            origin=Origin(xyz=(0.0, 0.0, arm_z), rpy=(0.0, math.pi / 2, 0.0)),
            material=matte_black,
            name="cross_arm_x",
        )
        # Arm along Y
        handle.visual(
            Cylinder(radius=CROSS_ARM_R, length=2 * CROSS_ARM_HALF),
            origin=Origin(xyz=(0.0, 0.0, arm_z), rpy=(math.pi / 2, 0.0, 0.0)),
            material=matte_black,
            name="cross_arm_y",
        )
        # End caps (four arm tips)
        for cap_name, dx, dy in (
            ("cap_x_pos", CROSS_ARM_HALF, 0.0),
            ("cap_x_neg", -CROSS_ARM_HALF, 0.0),
            ("cap_y_pos", 0.0, CROSS_ARM_HALF),
            ("cap_y_neg", 0.0, -CROSS_ARM_HALF),
        ):
            handle.visual(
                Sphere(radius=CROSS_ARM_R),
                origin=Origin(xyz=(dx, dy, arm_z)),
                material=matte_black,
                name=cap_name,
            )
        # Temperature indicator dot on hub front face
        handle.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, CROSS_HUB_R - 0.0005, arm_z)),
            material=dot_material,
            name="indicator_dot",
        )
        return handle

    hot_cross = _cross_handle("hot_cross_handle", hot_red)
    cold_cross = _cross_handle("cold_cross_handle", cold_blue)

    for joint_name, parent_col, child_handle in (
        ("hot_handle_turn", hot_valve_column, hot_cross),
        ("cold_handle_turn", cold_valve_column, cold_cross),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=parent_col,
            child=child_handle,
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=-math.pi / 2,
                upper=math.pi / 2,
            ),
        )

    return model


# ================================================================= tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("sink_deck")
    bridge = object_model.get_part("bridge_bar")
    spout_base = object_model.get_part("spout_base")
    gooseneck = object_model.get_part("gooseneck_spout")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_handle = object_model.get_part("hot_cross_handle")
    cold_handle = object_model.get_part("cold_cross_handle")

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_turn = object_model.get_articulation("hot_handle_turn")
    cold_turn = object_model.get_articulation("cold_handle_turn")
    deck_to_bridge = object_model.get_articulation("deck_to_bridge")

    spout_tube = gooseneck.get_visual("spout_tube")
    base_column = spout_base.get_visual("base_column")
    aerator = gooseneck.get_visual("aerator")

    # ── overlap allowances ────────────────────────────────────────────
    # Bridge bar structurally connects through the three column bases.
    ctx.allow_overlap(
        bridge,
        spout_base,
        reason="bridge bar passes through the centre spout base at the junction",
    )
    ctx.allow_overlap(
        bridge,
        hot_col,
        reason="bridge bar end enters hot valve column base at left junction",
    )
    ctx.allow_overlap(
        bridge,
        cold_col,
        reason="bridge bar end enters cold valve column base at right junction",
    )
    # Gooseneck riser and handle stems seat inside their columns.
    ctx.allow_overlap(
        gooseneck,
        spout_base,
        elem_a=spout_tube,
        elem_b=base_column,
        reason="gooseneck riser tube seats 30 mm into the base column bore",
    )
    for handle, col in ((hot_handle, hot_col), (cold_handle, cold_col)):
        ctx.allow_overlap(
            handle,
            col,
            elem_a=handle.get_visual("handle_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="cross-handle stem seats 15 mm into the valve cartridge bore",
        )

    # ── bridge bar: fixed to deck, spans between the three posts ──────
    ctx.check(
        "bridge_is_fixed_to_deck",
        str(deck_to_bridge.joint_type).lower().endswith("fixed"),
        f"type={deck_to_bridge.joint_type}",
    )
    bridge_aabb = ctx.part_world_aabb(bridge)
    bridge_span = bridge_aabb[1][0] - bridge_aabb[0][0]
    ctx.check(
        "bridge_spans_between_posts",
        bridge_span > 0.20,
        f"bridge x-span = {bridge_span:.3f} m",
    )
    ctx.check(
        "bridge_on_deck_surface",
        abs(bridge_aabb[0][2] - DECK_T) < 0.002,
        f"bridge bottom z = {bridge_aabb[0][2]:.4f}",
    )

    # ── joint plan: types, axes, ranges ───────────────────────────────
    for joint, lim in (
        (spout_swivel, math.pi / 4),
        (hot_turn, math.pi / 2),
        (cold_turn, math.pi / 2),
    ):
        ctx.check(
            f"{joint.name}_is_vertical_revolute",
            str(joint.joint_type).lower().endswith("revolute")
            and tuple(joint.axis) == (0.0, 0.0, 1.0),
            f"axis={joint.axis}",
        )
        ml = joint.motion_limits
        ctx.check(
            f"{joint.name}_range",
            ml is not None
            and abs(ml.lower + lim) < 1e-6
            and abs(ml.upper - lim) < 1e-6,
            f"lower={ml.lower} upper={ml.upper}",
        )

    # ── placement: 0.30 m spread, all three pieces seated on deck ─────
    hot_pos = ctx.part_world_position(hot_col)
    cold_pos = ctx.part_world_position(cold_col)
    spout_pos = ctx.part_world_position(spout_base)
    ctx.check(
        "widespread_0p30_spread",
        abs(hot_pos[0] + 0.15) < 1e-6
        and abs(cold_pos[0] - 0.15) < 1e-6
        and abs(spout_pos[0]) < 1e-6,
        f"hot_x={hot_pos[0]} cold_x={cold_pos[0]} spout_x={spout_pos[0]}",
    )
    for piece in (spout_base, hot_col, cold_col):
        ctx.expect_contact(piece, deck, contact_tol=1e-5)

    deck_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "deck_grounded_at_z0",
        abs(deck_aabb[0][2]) < 1e-6 and abs(deck_aabb[1][2] - DECK_T) < 1e-6,
        f"deck z {deck_aabb[0][2]}..{deck_aabb[1][2]}",
    )

    # ── cross handles: cross-shaped (both arms present) ───────────────
    for handle in (hot_handle, cold_handle):
        with ctx.pose({hot_turn: 0.0, cold_turn: 0.0}):
            arm_x_aabb = ctx.part_element_world_aabb(
                handle, elem=handle.get_visual("cross_arm_x")
            )
            arm_y_aabb = ctx.part_element_world_aabb(
                handle, elem=handle.get_visual("cross_arm_y")
            )
        span_x = arm_x_aabb[1][0] - arm_x_aabb[0][0]
        span_y = arm_y_aabb[1][1] - arm_y_aabb[0][1]
        ctx.check(
            f"{handle.name}_cross_shape",
            span_x > 0.08 and span_y > 0.08,
            f"arm_x span={span_x:.3f} arm_y span={span_y:.3f}",
        )

    # ── seam rings present at all three deck bases ────────────────────
    for part_with_seam in (spout_base, hot_col, cold_col):
        seam = part_with_seam.get_visual("deck_seam")
        ctx.check(
            f"{part_with_seam.name}_has_deck_seam",
            seam is not None,
            "deck_seam visual missing",
        )

    # ── decorative ring ridges on handle pedestals ────────────────────
    for col in (hot_col, cold_col):
        for i in range(len(RING_Z_POS)):
            ring = col.get_visual(f"deco_ring_{i}")
            ctx.check(
                f"{col.name}_deco_ring_{i}",
                ring is not None,
                f"deco_ring_{i} visual missing",
            )

    # ── gooseneck form: rises ~0.32 above deck, outlet ~0.25 above ────
    neck_aabb = ctx.part_world_aabb(gooseneck)
    arc_top_above_deck = neck_aabb[1][2] - DECK_T
    ctx.check(
        "gooseneck_arc_top_height",
        0.28 < arc_top_above_deck < 0.36,
        f"arc top {arc_top_above_deck:.3f} m above deck",
    )
    tip_aabb = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    outlet_above_deck = 0.5 * (tip_aabb[0][2] + tip_aabb[1][2]) - DECK_T
    ctx.check(
        "spout_outlet_about_0p25_above_deck",
        abs(outlet_above_deck - 0.25) < 0.02,
        f"outlet {outlet_above_deck:.3f} m above deck",
    )
    ctx.check(
        "spout_hooks_forward",
        tip_aabb[1][1] > 0.10,
        f"outlet front reach y={tip_aabb[1][1]:.3f}",
    )

    # ── indicator dots: red on hot, blue on cold ──────────────────────
    for handle, mat in ((hot_handle, "hot_red"), (cold_handle, "cold_blue")):
        dot = handle.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(
            f"{handle.name}_dot_material",
            mat_name == mat,
            f"material={mat_name}",
        )

    # ── articulation behaviour: cross handle rotation ─────────────────
    # At q=0 both arms span X and Y; at q=+π/2 the X-arm becomes Y-span
    # and vice-versa, proving rotation about the vertical axis.
    with ctx.pose({hot_turn: 0.0}):
        bar0_x = ctx.part_element_world_aabb(
            hot_handle, elem=hot_handle.get_visual("cross_arm_x")
        )
        bar0_y = ctx.part_element_world_aabb(
            hot_handle, elem=hot_handle.get_visual("cross_arm_y")
        )
    with ctx.pose({hot_turn: math.pi / 2}):
        bar90_x = ctx.part_element_world_aabb(
            hot_handle, elem=hot_handle.get_visual("cross_arm_x")
        )
        bar90_y = ctx.part_element_world_aabb(
            hot_handle, elem=hot_handle.get_visual("cross_arm_y")
        )
    span_x0 = bar0_x[1][0] - bar0_x[0][0]
    span_y0 = bar0_y[1][1] - bar0_y[0][1]
    span_x90 = bar90_x[1][0] - bar90_x[0][0]
    span_y90 = bar90_y[1][1] - bar90_y[0][1]
    ctx.check(
        "hot_cross_rotates_about_vertical",
        span_x0 > 0.08
        and span_y0 > 0.08
        and span_x90 < 0.03
        and span_y90 < 0.03,
        f"q=0 x={span_x0:.3f} y={span_y0:.3f}  q=90 x={span_x90:.3f} y={span_y90:.3f}",
    )

    # ── spout swivel: +45° swings outlet toward -X ────────────────────
    with ctx.pose({spout_swivel: math.pi / 4}):
        tip45 = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    tip45_cx = 0.5 * (tip45[0][0] + tip45[1][0])
    tip45_cz = 0.5 * (tip45[0][2] + tip45[1][2])
    ctx.check(
        "spout_swivels_about_column_axis",
        tip45_cx < -0.06 and abs(tip45_cz - (outlet_above_deck + DECK_T)) < 1e-3,
        f"tip at 45° x={tip45_cx:.3f} z={tip45_cz:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
