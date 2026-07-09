from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet with swan-neck spout.

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a high curved swan-neck spout
  (continuous rotation about the column's vertical axis),
- hot (left) and cold (right): valve columns with visible stem collars,
  topped by T-style lever handles with separate hot/cold cap disks
  (each revolute about its column's vertical axis, -90..+90 deg).

All faucet surfaces matte black; separate red/blue cap disks on the handle
stems serve as temperature indicators. Modeled at true scale in meters;
deck bottom on z=0.
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

# Swan neck (in the spout part frame, origin at column top)
TUBE_R = 0.0155  # neck tube radius (slimmer than the column)
RISER_EMBED = 0.03  # hidden engagement into the column below the joint
RISER_TOP = 0.22  # straight riser ends here; arc starts
ARC_R = 0.08  # swan neck arc radius (tall graceful curve)
HOOK_DEG = -30.0  # arc end angle; gentle downward curve
COLLAR_R = 0.020
COLLAR_H = 0.016

# Valve pieces
VALVE_FLANGE_R = 0.036
VALVE_FLANGE_H = 0.010
VALVE_COL_R = 0.0225
VALVE_COL_H = 0.10  # column top = lever joint height above the deck surface

# Stem collar (visible ring between valve column and lever)
STEM_COLLAR_R = 0.028
STEM_COLLAR_H = 0.010

# T-lever (in the lever part frame, origin at valve column top)
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.045
BAR_R = 0.0095
BAR_LEN = 0.12
BAR_CENTER_OFF = 0.025  # bar center offset so the stem sits ~1/3 from one end

# Hot/cold cap disk (temperature indicator as geometry)
CAP_R = 0.012
CAP_H = 0.003

ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.016
AERATOR_R = 0.017
# Unit tangent of the arc at the end (pointing out of the spout, downward).
_TX = math.sin(math.radians(HOOK_DEG))  # y component
_TZ = -math.cos(math.radians(HOOK_DEG))  # z component
AERATOR_CY = ARC_END_Y + _TX * (AERATOR_LEN / 2 - 0.004)
AERATOR_CZ = ARC_END_Z + _TZ * (AERATOR_LEN / 2 - 0.004)


def _swan_neck_solid() -> cq.Workplane:
    """Swept swan-neck tube: tall straight riser + graceful forward arc."""
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -RISER_EMBED)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (ARC_END_Y, ARC_END_Z))
    )
    profile = cq.Workplane("XY").workplane(offset=-RISER_EMBED).circle(TUBE_R)
    return profile.sweep(path, isFrenet=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_swan_neck_faucet")

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

    # -------------------------------------------------------- swan-neck spout
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
    # Aerator nozzle at the neck tip, aligned with the arc end tangent.
    swan_neck_spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(
            xyz=(0.0, AERATOR_CY, AERATOR_CZ),
            rpy=(math.radians(HOOK_DEG), 0.0, 0.0),
        ),
        material=matte_black,
        name="aerator",
    )

    # Continuous vertical swivel — no angular limits, full 360 rotation.
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
        # Visible stem collar: decorative ring at column top under the handle.
        col.visual(
            Cylinder(radius=STEM_COLLAR_R, length=STEM_COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H - STEM_COLLAR_H / 2)),
            material=matte_black,
            name="stem_collar",
        )
        return col

    def _t_lever(name: str, bar_off: float, cap_material: object) -> object:
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
        # Hot/cold cap disk on top of the stem — visible temperature indicator.
        lever.visual(
            Cylinder(radius=CAP_R, length=CAP_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_TOP + CAP_H / 2)),
            material=cap_material,
            name="indicator_cap",
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
        reason="swan neck riser tube seats 30 mm into the base column bore",
    )
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.allow_overlap(
            lever,
            col,
            elem_a=lever.get_visual("lever_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="lever stem seats 15 mm into the valve cartridge bore",
        )
        ctx.allow_overlap(
            lever,
            col,
            elem_a=lever.get_visual("lever_stem"),
            elem_b=col.get_visual("stem_collar"),
            reason="lever stem passes through the visible stem collar trim ring",
        )

    # --- joint plan: types, axes, ranges -----------------------------------
    # Spout swivel is continuous (no angular limits).
    ctx.check(
        "spout_swivel_is_continuous",
        str(spout_swivel.joint_type).lower().endswith("continuous")
        and tuple(spout_swivel.axis) == (0.0, 0.0, 1.0),
        f"type={spout_swivel.joint_type} axis={spout_swivel.axis}",
    )

    # Lever joints are revolute with -90..+90 range.
    for joint, lim in ((hot_turn, math.pi / 2), (cold_turn, math.pi / 2)):
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

    # --- placement: 0.30 m spread, all three pieces seated on the deck -----
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

    # --- swan neck form: high arc, outlet above deck -----------------------
    neck_aabb = ctx.part_world_aabb(swan_neck)
    arc_top_above_deck = neck_aabb[1][2] - DECK_T
    ctx.check(
        "swan_neck_high_arc",
        arc_top_above_deck > 0.35,
        f"arc top {arc_top_above_deck:.3f} m above deck (swan neck should be tall)",
    )
    tip_aabb = ctx.part_element_world_aabb(swan_neck, elem=aerator)
    outlet_above_deck = 0.5 * (tip_aabb[0][2] + tip_aabb[1][2]) - DECK_T
    ctx.check(
        "spout_outlet_above_deck",
        outlet_above_deck > 0.22,
        f"outlet {outlet_above_deck:.3f} m above deck",
    )
    ctx.check(
        "spout_curves_forward",
        tip_aabb[1][1] > 0.08,
        f"outlet front reach y={tip_aabb[1][1]:.3f}",
    )

    # --- stem collars: visible ring on each valve column -------------------
    for col in (hot_col, cold_col):
        collar_vis = col.get_visual("stem_collar")
        ctx.check(
            f"{col.name}_has_stem_collar",
            collar_vis is not None,
            f"stem_collar visual missing on {col.name}",
        )
    # Collar sits at column top, wider than column body.
    ctx.check(
        "stem_collar_wider_than_column",
        STEM_COLLAR_R > VALVE_COL_R,
        f"collar_r={STEM_COLLAR_R} col_r={VALVE_COL_R}",
    )
    # Stem passes through the collar — prove captured insertion.
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.expect_overlap(
            lever,
            col,
            axes="z",
            elem_a=lever.get_visual("lever_stem"),
            elem_b=col.get_visual("stem_collar"),
            min_overlap=0.005,
            name=f"{lever.name}_stem_captured_by_collar",
        )

    # --- hot/cold cap disks as geometry ------------------------------------
    for lever, mat_name in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        cap = lever.get_visual("indicator_cap")
        ctx.check(
            f"{lever.name}_has_cap_disk",
            cap is not None,
            f"indicator_cap visual missing on {lever.name}",
        )
        cap_mat = cap.material if isinstance(cap.material, str) else cap.material.name
        ctx.check(
            f"{lever.name}_cap_material",
            cap_mat == mat_name,
            f"material={cap_mat}",
        )

    # --- lever form: off-center T-bar overhangs outward --------------------
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

    # --- articulation behavior ---------------------------------------------
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

    # Spout continuous swivel: rotating by pi moves the outlet to the opposite side.
    with ctx.pose({spout_swivel: math.pi}):
        tip180 = ctx.part_element_world_aabb(swan_neck, elem=aerator)
    tip180_cy = 0.5 * (tip180[0][1] + tip180[1][1])
    ctx.check(
        "spout_continuous_swivel_rotates_outlet",
        tip180_cy < -0.05,
        f"tip at 180deg y={tip180_cy:.3f} (should swing to back)",
    )

    return ctx.report()


object_model = build_object_model()
