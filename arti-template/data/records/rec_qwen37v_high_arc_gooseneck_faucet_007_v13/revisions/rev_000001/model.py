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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Variant 13: squared bridge-like high-arc gooseneck faucet.
#
# Structural changes from the parent monobloc mixer tap:
# - Squared bridge gooseneck with softened (filleted) elbows replaces the
#   smooth swan-neck arc.  The path rises vertically, crosses a horizontal
#   bridge segment, then drops, with quarter-circle fillets at each elbow.
# - Spout swivels left-right on a vertical CONTINUOUS joint at the collar.
# - Visible cold/hot tick marks added as raised geometric Box features on
#   top of each valve end cap (blue = cold, red = hot).
# - Removable circular chrome deck plate added under the base disc.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (direction the gooseneck reaches over the sink), +Z up.
# - A removable chrome deck plate (0.11 m dia, 0.004 m thick) sits just
#   below the deck plane; the chrome base disc and gloss-black column sit
#   above it, linked by a FIXED joint.
# - Horizontal cross-cylinder with valve bodies and flat end caps as before.
# - Two pin levers (REVOLUTE, -90..0 deg) on the valve Y axis.
# - Apex at ~0.380 m.
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020  # 0.04 m diameter
COLUMN_TOP = 0.132

# Deck plate (removable, under the base)
DECK_PLATE_R = 0.055
DECK_PLATE_H = 0.004

# Cross valve cylinder
CROSS_Z = 0.085
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0

# Hot/cold tick marks (raised geometric indicators on valve end cap tops)
TICK_LEN = 0.014
TICK_WIDTH = 0.003
TICK_HEIGHT = 0.002
TICK_Z = CROSS_Z + CAP_R + TICK_HEIGHT / 2.0

# Pin levers
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# Swivel collar + squared bridge gooseneck
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

TUBE_R = 0.015
CORNER_R = 0.025  # softened elbow fillet radius
BRIDGE_Z = 0.225  # centerline height of horizontal bridge segment (spout-local)
RISER_TOP = BRIDGE_Z - CORNER_R  # 0.200 — top of straight riser before fillet
REACH_X = 0.144  # horizontal reach of the drop leg
DROP_END = 0.124  # spout-local z of the tube tip

# Precomputed arc midpoints for the two quarter-circle fillets.
_SQRT2_2 = math.sqrt(2.0) / 2.0
ELBOW1_MID = (CORNER_R * (1.0 - _SQRT2_2), RISER_TOP + CORNER_R * _SQRT2_2)
ELBOW1_END = (CORNER_R, BRIDGE_Z)
ELBOW2_MID = (
    REACH_X - CORNER_R * (1.0 - _SQRT2_2),
    BRIDGE_Z - CORNER_R * (1.0 - _SQRT2_2),
)
ELBOW2_END = (REACH_X, BRIDGE_Z - CORNER_R)

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028
AERATOR_R = 0.0118
AERATOR_LEN = 0.003

APEX_WORLD = SWIVEL_Z + BRIDGE_Z + TUBE_R  # ~0.380 m


def _gooseneck_shape() -> cq.Workplane:
    """Squared bridge-like gooseneck with softened quarter-circle elbows."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc(ELBOW1_MID, ELBOW1_END)
        .lineTo(REACH_X - CORNER_R, BRIDGE_Z)
        .threePointArc(ELBOW2_MID, ELBOW2_END)
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squared_bridge_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    cold_mark = model.material("cold_indicator", rgba=(0.15, 0.25, 0.75, 1.0))
    hot_mark = model.material("hot_indicator", rgba=(0.75, 0.15, 0.15, 1.0))

    # -------------------------------------------------------- removable deck plate
    deck_plate = model.part("deck_plate")
    deck_plate.visual(
        Cylinder(radius=DECK_PLATE_R, length=DECK_PLATE_H),
        origin=Origin(xyz=(0.0, 0.0, -DECK_PLATE_H / 2.0)),
        material=chrome,
        name="plate_disc",
    )

    # -------------------------------------------------------- body column (root)
    column = model.part("body_column")
    column.visual(
        Cylinder(radius=BASE_DISC_R, length=BASE_DISC_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_H / 2.0)),
        material=chrome,
        name="base_disc",
    )
    column.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_TOP - 0.004),
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_TOP + 0.004) / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )
    # Horizontal cross valve cylinder
    column.visual(
        Cylinder(radius=CROSS_R, length=CROSS_TUBE_LEN),
        origin=Origin(xyz=(0.0, 0.0, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="cross_tube",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_0",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, -CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_1",
    )
    # Cold/hot tick marks — raised geometric indicators on valve cap tops
    column.visual(
        Box((TICK_LEN, TICK_WIDTH, TICK_HEIGHT)),
        origin=Origin(xyz=(0.0, CAP_Y, TICK_Z)),
        material=cold_mark,
        name="cold_tick",
    )
    column.visual(
        Box((TICK_LEN, TICK_WIDTH, TICK_HEIGHT)),
        origin=Origin(xyz=(0.0, -CAP_Y, TICK_Z)),
        material=hot_mark,
        name="hot_tick",
    )
    # Chrome collar ring
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # FIXED joint: deck plate under the column base
    model.articulation(
        "column_to_deck",
        ArticulationType.FIXED,
        parent=column,
        child=deck_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # -------------------------------------------------------- gooseneck spout
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    spout.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + SLEEVE_LEN / 2.0)),
        material=chrome,
        name="tip_sleeve",
    )
    spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - 0.001)),
        material=outlet_dark,
        name="outlet_aerator",
    )
    # CONTINUOUS joint: spout swivels freely left-right about vertical axis
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5),
    )

    # -------------------------------------------------------- pin levers
    for idx, y_sign in ((0, 1.0), (1, -1.0)):
        lever = model.part(f"pin_lever_{idx}")
        lever.visual(
            Cylinder(radius=BOSS_R, length=BOSS_LEN),
            origin=Origin(xyz=(0.0, 0.0, BOSS_Z)),
            material=gloss_black,
            name="lever_boss",
        )
        lever.visual(
            Cylinder(radius=PIN_R, length=PIN_LEN),
            origin=Origin(xyz=(0.0, 0.0, PIN_Z0 + PIN_LEN / 2.0)),
            material=gloss_black,
            name="lever_pin",
        )
        model.articulation(
            f"lever_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=column,
            child=lever,
            origin=Origin(xyz=(0.0, y_sign * LEVER_Y, CROSS_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=2.0, lower=-math.pi / 2.0, upper=0.0
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck_plate = object_model.get_part("deck_plate")
    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    column_to_deck = object_model.get_articulation("column_to_deck")
    swivel = object_model.get_articulation("spout_swivel")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

    # Intentional seated insertions: lever bosses embed into valve cylinders
    ctx.allow_overlap(
        lever_0,
        column,
        elem_a="lever_boss",
        elem_b="cross_tube",
        reason="Lever boss intentionally seats a few mm into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1,
        column,
        elem_a="lever_boss",
        elem_b="cross_tube",
        reason="Lever boss intentionally seats a few mm into the valve cylinder.",
    )

    # --- removable deck plate ---
    deck_aabb = ctx.part_world_aabb(deck_plate)
    ctx.check(
        "removable circular deck plate under the base (~0.11 m dia, thin)",
        deck_aabb is not None
        and 0.100 <= (deck_aabb[1][0] - deck_aabb[0][0]) <= 0.115
        and (deck_aabb[1][2] - deck_aabb[0][2]) <= 0.006,
        details=f"deck_plate aabb={deck_aabb}",
    )
    ctx.check(
        "deck plate sits at or just below the deck plane",
        deck_aabb is not None and deck_aabb[0][2] >= -0.006 and deck_aabb[1][2] <= 0.001,
        details=f"deck_plate aabb={deck_aabb}",
    )
    ctx.check(
        "column_to_deck is a FIXED joint connecting column to deck plate",
        column_to_deck.articulation_type == ArticulationType.FIXED,
    )

    # --- hot/cold tick marks as geometry ---
    cold = ctx.part_element_world_aabb(column, elem="cold_tick")
    hot = ctx.part_element_world_aabb(column, elem="hot_tick")
    ctx.check(
        "cold tick mark is a raised geometric feature on the +Y valve cap",
        cold is not None
        and (cold[1][2] - cold[0][2]) <= 0.004
        and (cold[1][0] - cold[0][0]) >= 0.010,
        details=f"cold_tick aabb={cold}",
    )
    ctx.check(
        "hot tick mark is a raised geometric feature on the -Y valve cap",
        hot is not None
        and (hot[1][2] - hot[0][2]) <= 0.004
        and (hot[1][0] - hot[0][0]) >= 0.010,
        details=f"hot_tick aabb={hot}",
    )
    ctx.check(
        "cold and hot ticks are on opposite sides of the column (Y separation)",
        cold is not None
        and hot is not None
        and 0.5 * (cold[0][1] + cold[1][1]) > 0.04
        and 0.5 * (hot[0][1] + hot[1][1]) < -0.04,
        details=f"cold_center_y={0.5*(cold[0][1]+cold[1][1]):.4f}, hot_center_y={0.5*(hot[0][1]+hot[1][1]):.4f}",
    )

    # --- squared bridge gooseneck ---
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m (high-arc silhouette maintained)",
        spout_aabb is not None and 0.372 <= spout_aabb[1][2] <= 0.390,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.130,
        details=f"spout aabb={spout_aabb}",
    )
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "squared bridge tube has substantial vertical extent (riser + bridge + drop)",
        tube is not None and tube[1][2] - tube[0][2] >= 0.15,
        details=f"tube aabb={tube}",
    )

    # --- column and base proportions ---
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "vertical column is ~0.04 m diameter",
        shaft is not None and 0.038 <= (shaft[1][0] - shaft[0][0]) <= 0.042,
        details=f"column shaft aabb={shaft}",
    )

    # --- cross valve cylinder ---
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    cap_0 = ctx.part_element_world_aabb(column, elem="valve_end_cap_0")
    cap_1 = ctx.part_element_world_aabb(column, elem="valve_end_cap_1")
    ctx.check(
        "cross-cylinder is ~0.045 m diameter",
        cross is not None and 0.043 <= (cross[1][2] - cross[0][2]) <= 0.047,
        details=f"cross aabb={cross}",
    )
    ctx.check(
        "valve assembly spans ~0.18 m end-to-end",
        cap_0 is not None
        and cap_1 is not None
        and 0.178 <= (cap_0[1][1] - cap_1[0][1]) <= 0.182,
        details=f"cap_0={cap_0}, cap_1={cap_1}",
    )

    # --- chrome collar ---
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "thin chrome collar sits above the cross and below the spout",
        collar is not None
        and cross is not None
        and collar[0][2] >= cross[1][2]
        and spout_aabb is not None
        and collar[1][2] <= spout_aabb[0][2] + 1e-6,
        details=f"collar={collar}",
    )
    ctx.expect_contact(
        spout,
        column,
        elem_a="gooseneck_tube",
        elem_b="swivel_collar",
        contact_tol=0.001,
        name="gooseneck riser seats on the chrome collar",
    )

    # --- chrome tip sleeve with downward outlet ---
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    aerator = ctx.part_element_world_aabb(spout, elem="outlet_aerator")
    ctx.check(
        "chrome tip sleeve with downward outlet at spout end",
        sleeve is not None
        and aerator is not None
        and 0.25 <= sleeve[0][2] <= 0.29
        and aerator[0][2] < sleeve[0][2],
        details=f"sleeve={sleeve}, aerator={aerator}",
    )

    # --- pin levers ---
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        pin = ctx.part_element_world_aabb(lever, elem="lever_pin")
        ctx.check(
            f"{name} pin is slim (0.012 m dia) and 0.10 m long, vertical at rest",
            pin is not None
            and 0.010 <= (pin[1][0] - pin[0][0]) <= 0.014
            and 0.098 <= (pin[1][2] - pin[0][2]) <= 0.102,
            details=f"pin aabb={pin}",
        )
        ctx.expect_overlap(
            lever,
            column,
            axes="z",
            elem_a="lever_boss",
            elem_b="cross_tube",
            min_overlap=0.003,
            name=f"{name} boss seats into the valve cylinder",
        )

    # --- joint plan: types, axes, ranges ---
    ctx.check(
        "spout swivel is a CONTINUOUS joint about the vertical column axis",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0 deg about the valve left-right axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6,
        )

    # --- at least one non-fixed joint ---
    non_fixed = [
        a
        for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least one non-fixed joint exists (revolute or continuous)",
        len(non_fixed) >= 1,
        details=f"non-fixed joints: {[a.name for a in non_fixed]}",
    )

    # --- swivel pose: spout sweeps sideways under continuous rotation ---
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout continuous swivel carries the outlet sideways about the vertical axis",
        rest_sleeve is not None
        and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 0.01
        and abs(0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1])) > 0.05,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    # --- lever pose: full -90 deg tilt toward the user ---
    rest_0 = ctx.part_world_aabb(lever_0)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
    ctx.check(
        "lever 0 tilts from vertical to horizontal toward the user at q=-90 deg",
        rest_0 is not None
        and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.08
        and tilted_0[1][2] < CROSS_Z + 0.03,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )

    return ctx.report()


object_model = build_object_model()
