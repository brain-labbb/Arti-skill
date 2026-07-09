from __future__ import annotations

"""Matte-black widespread three-piece bathroom faucet set (swan-neck variant).

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a high-curved swan-neck spout
  (continuous swivel about the column's vertical axis),
- hot (left) and cold (right): valve columns topped by T-style lever
  handles (each revolute about its column's vertical axis, -90..+90 deg),
  with visible stem collars (trim rings) at the handle bases.

All faucet surfaces matte black; tiny red/blue indicator dots on the
handle stems. Modeled at true scale in meters; deck bottom on z=0.
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

# ---------------------------------------------------------------- dimensions
DECK_X = 0.46  # deck slab length along the spread axis
DECK_Y = 0.18  # deck slab depth (front/back)
DECK_T = 0.02  # deck slab thickness; deck top at z = DECK_T

SPREAD_HALF = 0.15  # valve columns at x = -0.15 / +0.15 (0.30 m spread)

# Center spout piece
SPOUT_FLANGE_R = 0.042
SPOUT_FLANGE_H = 0.012
SPOUT_COL_R = 0.025
SPOUT_COL_H = 0.12  # column top = joint height above the deck surface

# Swan neck (higher curved profile, in spout part frame, origin at column top)
TUBE_R = 0.0155  # tube radius (slimmer than the column)
RISER_EMBED = 0.03  # hidden engagement into the column below the joint
RISER_TOP = 0.17  # tall riser for the high swan-neck profile
ARC_R = 0.060  # graceful arc radius
HOOK_DEG = -30.0  # pronounced forward-down hook at the outlet
COLLAR_R = 0.020
COLLAR_H = 0.016

# Valve pieces
VALVE_FLANGE_R = 0.036
VALVE_FLANGE_H = 0.010
VALVE_COL_R = 0.0225
VALVE_COL_H = 0.10  # column top = lever joint height above the deck surface

# Stem collar (visible trim ring under each handle, on the valve column top)
STEM_COLLAR_R = 0.028
STEM_COLLAR_H = 0.008

# T-lever (in the lever part frame, origin at valve column top)
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.045
BAR_R = 0.0095
BAR_LEN = 0.12
BAR_CENTER_OFF = 0.025  # bar center offset so the stem sits ~1/3 from one end
DOT_R = 0.0035

ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.016
AERATOR_R = 0.017
# Unit tangent of the arc at the hook end (pointing out of the spout, downward).
_TX = math.sin(math.radians(HOOK_DEG))  # y component
_TZ = -math.cos(math.radians(HOOK_DEG))  # z component
AERATOR_CY = ARC_END_Y + _TX * (AERATOR_LEN / 2 - 0.004)
AERATOR_CZ = ARC_END_Z + _TZ * (AERATOR_LEN / 2 - 0.004)


def _swan_neck_solid() -> cq.Workplane:
    """Swept swan-neck tube: tall riser + high graceful forward-down arc."""
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -RISER_EMBED)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (ARC_END_Y, ARC_END_Z))
    )
    profile = cq.Workplane("XY").workplane(offset=-RISER_EMBED).circle(TUBE_R)
    return profile.sweep(path, isFrenet=True)


def _stem_collar_ring() -> cq.Workplane:
    """Annular trim ring that sits on the valve column top around the stem."""
    return (
        cq.Workplane("XY")
        .circle(STEM_COLLAR_R)
        .extrude(STEM_COLLAR_H)
        .faces(">Z")
        .workplane()
        .circle(STEM_R + 0.001)
        .cutThruAll()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_black_swan_neck_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.08, 0.08, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))

    # ------------------------------------------------------------- sink deck
    sink_deck = model.part("sink_deck")
    sink_deck.visual(
        Box((DECK_X, DECK_Y, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, DECK_T / 2)),
        material=deck_stone,
        name="deck_slab",
    )

    # ----------------------------------------------------- center spout base
    spout_base = model.part("spout_base")
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

    # ------------------------------------------------------- swan-neck spout
    swan_neck_spout = model.part("swan_neck_spout")
    swan_neck_spout.visual(
        mesh_from_cadquery(_swan_neck_solid(), "swan_neck_tube"),
        material=matte_black,
        name="spout_tube",
    )
    swan_neck_spout.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H / 2)),
        material=matte_black,
        name="swivel_collar",
    )
    # Aerator nozzle at the hook tip, aligned with the arc end tangent.
    swan_neck_spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(
            xyz=(0.0, AERATOR_CY, AERATOR_CZ),
            rpy=(math.radians(HOOK_DEG), 0.0, 0.0),
        ),
        material=matte_black,
        name="aerator",
    )

    # Continuous vertical swivel (unlimited rotation, no position bounds)
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=spout_base,
        child=swan_neck_spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_COL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0),
    )

    # --------------------------------------------------- hot / cold valves
    def _valve_column(name: str) -> object:
        col = model.part(name)
        col.visual(
            Cylinder(radius=VALVE_FLANGE_R, length=VALVE_FLANGE_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_FLANGE_H / 2)),
            material=matte_black,
            name="valve_flange",
        )
        col.visual(
            Cylinder(radius=VALVE_COL_R, length=VALVE_COL_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H / 2)),
            material=matte_black,
            name="valve_body",
        )
        # Visible stem collar (trim ring) on top of the valve column,
        # surrounding the handle stem bore.
        col.visual(
            mesh_from_cadquery(_stem_collar_ring(), f"{name}_collar"),
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H)),
            material=matte_black,
            name="stem_collar",
        )
        return col

    def _t_lever(name: str, bar_off: float, dot_material: object) -> object:
        lever = model.part(name)
        lever.visual(
            Cylinder(radius=STEM_R, length=STEM_TOP + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (STEM_TOP - STEM_EMBED) / 2)),
            material=matte_black,
            name="lever_stem",
        )
        # Horizontal T-bar along X; off-center so it overhangs outward.
        lever.visual(
            Cylinder(radius=BAR_R, length=BAR_LEN),
            origin=Origin(xyz=(bar_off, 0.0, STEM_TOP), rpy=(0.0, math.pi / 2, 0.0)),
            material=matte_black,
            name="lever_bar",
        )
        for end in (-1.0, 1.0):
            lever.visual(
                Sphere(radius=BAR_R),
                origin=Origin(xyz=(bar_off + end * BAR_LEN / 2, 0.0, STEM_TOP)),
                material=matte_black,
                name=f"bar_cap_{'outer' if end * bar_off > 0 else 'inner'}",
            )
        # Tiny temperature indicator dot on the front of the stem.
        lever.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, STEM_R - 0.0005, 0.022)),
            material=dot_material,
            name="indicator_dot",
        )
        return lever

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    # Hot on the left (-X), cold on the right (+X); +Y is toward the user.
    hot_lever = _t_lever("hot_lever", -BAR_CENTER_OFF, hot_red)
    cold_lever = _t_lever("cold_lever", BAR_CENTER_OFF, cold_blue)

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

    for joint_name, parent, child in (
        ("hot_lever_turn", hot_valve_column, hot_lever),
        ("cold_lever_turn", cold_valve_column, cold_lever),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=parent,
            child=child,
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=2.0, lower=-math.pi / 2, upper=math.pi / 2
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("sink_deck")
    spout_base = object_model.get_part("spout_base")
    swan_neck = object_model.get_part("swan_neck_spout")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_lever = object_model.get_part("hot_lever")
    cold_lever = object_model.get_part("cold_lever")

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_turn = object_model.get_articulation("hot_lever_turn")
    cold_turn = object_model.get_articulation("cold_lever_turn")

    spout_tube = swan_neck.get_visual("spout_tube")
    base_column = spout_base.get_visual("base_column")
    aerator = swan_neck.get_visual("aerator")

    # Intentional hidden engagements: spout riser and lever stems seat inside
    # their columns so the rotating parts read as mounted, not floating.
    ctx.allow_overlap(
        swan_neck,
        spout_base,
        elem_a=spout_tube,
        elem_b=base_column,
        reason="swan-neck riser tube seats 30 mm into the base column bore",
    )
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.allow_overlap(
            lever,
            col,
            elem_a=lever.get_visual("lever_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="lever stem seats 15 mm into the valve cartridge bore",
        )

    # --- Variant: spout swivel is CONTINUOUS (unlimited vertical rotation) ---
    ctx.check(
        "spout_swivel_is_continuous",
        str(spout_swivel.joint_type).lower().endswith("continuous"),
        f"type={spout_swivel.joint_type}",
    )
    ctx.check(
        "spout_swivel_axis_is_vertical",
        tuple(spout_swivel.axis) == (0.0, 0.0, 1.0),
        f"axis={spout_swivel.axis}",
    )
    # Continuous joints must not have position bounds.
    ml = spout_swivel.motion_limits
    ctx.check(
        "spout_swivel_no_position_bounds",
        ml is None or (ml.lower is None and ml.upper is None),
        f"lower={ml.lower if ml else None} upper={ml.upper if ml else None}",
    )

    # --- Handle joints: revolute, vertical axis, ±90° ---
    for joint, lim in ((hot_turn, math.pi / 2), (cold_turn, math.pi / 2)):
        ctx.check(
            f"{joint.name}_is_vertical_revolute",
            str(joint.joint_type).lower().endswith("revolute")
            and tuple(joint.axis) == (0.0, 0.0, 1.0),
            f"axis={joint.axis}",
        )
        jml = joint.motion_limits
        ctx.check(
            f"{joint.name}_range",
            jml is not None
            and abs(jml.lower + lim) < 1e-6
            and abs(jml.upper - lim) < 1e-6,
            f"lower={jml.lower} upper={jml.upper}",
        )

    # --- Variant: visible stem collars under each handle ---
    for col in (hot_col, cold_col):
        collar = col.get_visual("stem_collar")
        ctx.check(
            f"{col.name}_has_stem_collar",
            collar is not None,
            "stem_collar visual missing",
        )
        # Collar should be wider than the valve body (visible trim ring).
        collar_aabb = ctx.part_element_world_aabb(col, elem=collar)
        body_aabb = ctx.part_element_world_aabb(col, elem=col.get_visual("valve_body"))
        collar_dx = collar_aabb[1][0] - collar_aabb[0][0]
        body_dx = body_aabb[1][0] - body_aabb[0][0]
        ctx.check(
            f"{col.name}_collar_wider_than_body",
            collar_dx > body_dx + 0.005,
            f"collar_dx={collar_dx:.4f} body_dx={body_dx:.4f}",
        )
        # Collar sits at the top of the valve column (contact with body top).
        ctx.expect_gap(
            col,
            col,
            axis="z",
            positive_elem="stem_collar",
            negative_elem="valve_body",
            min_gap=-0.001,
            max_gap=0.001,
            name=f"{col.name}_collar_seated_on_column_top",
        )

    # --- Placement: 0.30 m spread, all three pieces seated on the deck -----
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

    # --- Swan-neck form: high curve rises above 0.32 m, outlet forward ------
    neck_aabb = ctx.part_world_aabb(swan_neck)
    arc_top_above_deck = neck_aabb[1][2] - DECK_T
    ctx.check(
        "swan_neck_high_arc_above_0p32",
        arc_top_above_deck > 0.32,
        f"arc top {arc_top_above_deck:.3f} m above deck (high swan neck should exceed 0.32)",
    )
    tip_aabb = ctx.part_element_world_aabb(swan_neck, elem=aerator)
    outlet_above_deck = 0.5 * (tip_aabb[0][2] + tip_aabb[1][2]) - DECK_T
    ctx.check(
        "spout_outlet_height",
        0.20 < outlet_above_deck < 0.32,
        f"outlet {outlet_above_deck:.3f} m above deck",
    )
    ctx.check(
        "spout_curves_forward",
        tip_aabb[1][1] > 0.06,
        f"outlet front reach y={tip_aabb[1][1]:.3f}",
    )

    # --- Lever form: off-center T-bar overhangs outward --------------------
    for lever, sign in ((hot_lever, -1.0), (cold_lever, 1.0)):
        bar_aabb = ctx.part_element_world_aabb(lever, elem=lever.get_visual("lever_bar"))
        bar_center_x = 0.5 * (bar_aabb[0][0] + bar_aabb[1][0])
        col_x = sign * 0.15
        ctx.check(
            f"{lever.name}_bar_overhangs_outward",
            sign * (bar_center_x - col_x) > 0.02,
            f"bar center x={bar_center_x:.3f} vs column x={col_x:.3f}",
        )
        # Bar clears the valve column top (only the stem enters the column).
        ctx.expect_gap(
            lever,
            (hot_col if sign < 0 else cold_col),
            axis="z",
            positive_elem=lever.get_visual("lever_bar"),
            min_gap=0.02,
        )

    # Indicator dots: red on hot, blue on cold, proud of the stem front.
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}")

    # --- Articulation behavior ---------------------------------------------
    # Off-axis proof: at q=0 the T-bar spans X; at q=+90 deg it spans Y.
    with ctx.pose({hot_turn: 0.0}):
        bar0 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("lever_bar"))
    with ctx.pose({hot_turn: math.pi / 2}):
        bar90 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("lever_bar"))
    span_x0 = bar0[1][0] - bar0[0][0]
    span_y0 = bar0[1][1] - bar0[0][1]
    span_x90 = bar90[1][0] - bar90[0][0]
    span_y90 = bar90[1][1] - bar90[0][1]
    ctx.check(
        "hot_lever_rotates_about_vertical_axis",
        span_x0 > 0.10 and span_y0 < 0.03 and span_y90 > 0.10 and span_x90 < 0.03,
        f"closed span=({span_x0:.3f},{span_y0:.3f}) turned span=({span_x90:.3f},{span_y90:.3f})",
    )

    # Spout continuous swivel: +90 deg swings the outlet toward -X (right-hand
    # rule about +Z), height preserved — proves unlimited rotation works.
    with ctx.pose({spout_swivel: math.pi / 2}):
        tip90 = ctx.part_element_world_aabb(swan_neck, elem=aerator)
    tip90_cx = 0.5 * (tip90[0][0] + tip90[1][0])
    tip90_cz = 0.5 * (tip90[0][2] + tip90[1][2])
    ctx.check(
        "spout_continuous_swivel_at_90deg",
        tip90_cx < -0.06 and abs(tip90_cz - (outlet_above_deck + DECK_T)) < 1e-3,
        f"tip at 90deg x={tip90_cx:.3f} z={tip90_cz:.3f}",
    )
    # Also verify rotation past the old ±45° limit (proves continuous range).
    with ctx.pose({spout_swivel: math.pi}):
        tip180 = ctx.part_element_world_aabb(swan_neck, elem=aerator)
    tip180_cy = 0.5 * (tip180[0][1] + tip180[1][1])
    ctx.check(
        "spout_continuous_swivel_at_180deg",
        tip180_cy < -0.04,
        f"tip at 180deg y={tip180_cy:.3f} (should swing behind center)",
    )

    return ctx.report()


object_model = build_object_model()
