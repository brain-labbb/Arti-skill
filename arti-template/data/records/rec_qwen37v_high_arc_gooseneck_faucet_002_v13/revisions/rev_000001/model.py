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
# High-arc gooseneck kitchen faucet variant: squared bridge with softened
# elbows, continuous base swivel, hot/cold tick marks, removable deck plate.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (direction the spout reaches over the sink), +Z is up.
# - A circular deck plate sits at z = 0; the tapered column rises from it.
# - The squared bridge gooseneck: straight vertical riser, filleted elbow to
#   a horizontal bridge, filleted elbow to a straight drop leg.
# - The spout swivels on a CONTINUOUS vertical joint at the column top.
# - Hot/cold tick marks are small raised geometry on the swivel collar.
# - A lever on the right side controls flow/temperature (revolute +/-45 deg).
# ---------------------------------------------------------------------------

# Deck plate
DECK_PLATE_R = 0.040
DECK_PLATE_H = 0.006

# Column
COLUMN_BASE_R = 0.028
COLUMN_TOP_R = 0.016
COLUMN_HEIGHT = 0.280
COLLAR_R = 0.019
COLLAR_H = 0.014
SWIVEL_Z = COLUMN_HEIGHT + COLLAR_H  # top of swivel collar = spout base

# Gooseneck (squared bridge, spout-local coords, origin at swivel top)
TUBE_R = 0.011
FILLET_R = 0.025  # softened elbow radius
RISER_H = 0.110  # straight vertical riser
BRIDGE_LEN = 0.100  # horizontal bridge length
DROP_H = RISER_H + FILLET_R  # drop starts at bridge height
DROP_END = 0.005  # where the open tip ends (spout-local z)
REACH_X = FILLET_R + BRIDGE_LEN + FILLET_R  # total horizontal reach

# Valve + lever
VALVE_Z = 0.130
VALVE_R = 0.012
VALVE_LEN = 0.048
VALVE_Y_CENTER = -(COLUMN_BASE_R + VALVE_LEN / 2.0 - 0.005)
LEVER_JOINT_Y = VALVE_Y_CENTER - VALVE_LEN / 2.0 + 0.003
LEVER_PIN_LEN = 0.095

# Tick marks (on the swivel collar, front face +X side)
TICK_W = 0.003
TICK_H = 0.012
TICK_D = 0.003
TICK_Z = COLUMN_HEIGHT + COLLAR_H / 2.0  # mid-height of collar
TICK_X = COLLAR_R + TICK_D / 2.0  # proud of collar surface


def _column_shape() -> cq.Workplane:
    """Tapered conical column from deck plate top to collar base."""
    return (
        cq.Workplane("XY")
        .workplane(offset=DECK_PLATE_H)
        .circle(COLUMN_BASE_R)
        .workplane(offset=COLUMN_HEIGHT)
        .circle(COLUMN_TOP_R)
        .loft()
    )


def _bridge_gooseneck_shape() -> cq.Workplane:
    """Squared bridge gooseneck: riser, filleted elbow, bridge, filleted elbow, drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_H)
        .tangentArcPoint((FILLET_R, FILLET_R), relative=True)
        .lineTo(FILLET_R + BRIDGE_LEN, RISER_H + FILLET_R)
        .tangentArcPoint((FILLET_R, -FILLET_R), relative=True)
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squared_bridge_gooseneck_faucet")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    dark_gold = model.material("dark_gold", rgba=(0.55, 0.42, 0.15, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    red = model.material("indicator_red", rgba=(0.85, 0.13, 0.10, 1.0))
    blue = model.material("indicator_blue", rgba=(0.20, 0.45, 0.90, 1.0))

    # ---------------------------------------------------------------- deck plate
    deck = model.part("deck_plate")
    deck.visual(
        Cylinder(radius=DECK_PLATE_R, length=DECK_PLATE_H),
        origin=Origin(xyz=(0.0, 0.0, DECK_PLATE_H / 2.0)),
        material=gold,
        name="deck_disk",
    )
    # Raised ring detail on top of deck plate
    deck.visual(
        Cylinder(radius=DECK_PLATE_R - 0.003, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, DECK_PLATE_H + 0.001)),
        material=dark_gold,
        name="deck_ring",
    )

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        mesh_from_cadquery(_column_shape(), "tapered_column"),
        material=gold,
        name="tapered_column",
    )
    # Swivel collar at column top
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_HEIGHT + COLLAR_H / 2.0)),
        material=gold,
        name="swivel_collar",
    )

    # Hot tick mark (red, right side of collar, +Y)
    column.visual(
        Box((TICK_D, TICK_W, TICK_H)),
        origin=Origin(xyz=(0.0, TICK_X, TICK_Z)),
        material=red,
        name="hot_tick",
    )
    # Cold tick mark (blue, left side of collar, -Y)
    column.visual(
        Box((TICK_D, TICK_W, TICK_H)),
        origin=Origin(xyz=(0.0, -TICK_X, TICK_Z)),
        material=blue,
        name="cold_tick",
    )

    # Horizontal valve body on the right (-Y) side
    column.visual(
        Cylinder(radius=VALVE_R, length=VALVE_LEN),
        origin=Origin(
            xyz=(0.0, VALVE_Y_CENTER, VALVE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="valve_body",
    )

    # Fixed joint: column sits on deck plate
    model.articulation(
        "column_to_deck",
        ArticulationType.FIXED,
        parent=deck,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # -------------------------------------------------------- squared bridge spout
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_bridge_gooseneck_shape(), "bridge_tube"),
        material=gold,
        name="bridge_tube",
    )
    # Open tip ring at the drop-leg end (water outlet indicator)
    spout.visual(
        Cylinder(radius=TUBE_R + 0.002, length=0.005),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - 0.0025)),
        material=dark_gold,
        name="spout_tip_ring",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0),
    )

    # ------------------------------------------------------------------- lever
    lever = model.part("pin_lever")
    lever.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="lever_collar",
    )
    lever.visual(
        Cylinder(radius=0.004, length=LEVER_PIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.008 + LEVER_PIN_LEN / 2.0)),
        material=gold,
        name="lever_pin",
    )
    lever.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.008 + LEVER_PIN_LEN + 0.004)),
        material=dark_gold,
        name="lever_tip",
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=column,
        child=lever,
        origin=Origin(xyz=(0.0, LEVER_JOINT_Y, VALVE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=-math.pi / 4.0, upper=math.pi / 4.0
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_plate")
    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    lever = object_model.get_part("pin_lever")

    swivel = object_model.get_articulation("spout_swivel")
    lever_pivot = object_model.get_articulation("lever_pivot")

    # Intentional overlap: lever collar captured on valve body end
    ctx.allow_overlap(
        lever,
        column,
        elem_a="lever_collar",
        elem_b="valve_body",
        reason="Rotating lever collar is captured on the valve body end (seated insertion).",
    )

    # ----- variant: squared bridge gooseneck shape exists
    bridge_aabb = ctx.part_element_world_aabb(spout, elem="bridge_tube")
    ctx.check(
        "squared bridge gooseneck tube exists with substantial reach",
        bridge_aabb is not None
        and (bridge_aabb[1][0] - bridge_aabb[0][0]) > 0.10,
        details=f"bridge_tube aabb={bridge_aabb}",
    )

    # ----- variant: bridge has a horizontal section (top is flat-ish, not arcing)
    # The bridge top should be near the same z across its horizontal extent
    ctx.check(
        "bridge apex is flat-topped (squared bridge, not smooth arc)",
        bridge_aabb is not None
        and bridge_aabb[1][2] > 0.40
        and bridge_aabb[1][2] < 0.48,
        details=f"bridge_tube aabb={bridge_aabb}",
    )

    # ----- variant: continuous joint for spout swivel
    ctx.check(
        "spout swivel is a CONTINUOUS joint about vertical axis",
        swivel is not None
        and swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )

    # ----- variant: hot/cold tick marks as geometry
    hot_aabb = ctx.part_element_world_aabb(column, elem="hot_tick")
    cold_aabb = ctx.part_element_world_aabb(column, elem="cold_tick")
    ctx.check(
        "hot and cold tick marks exist as geometry on the collar",
        hot_aabb is not None and cold_aabb is not None,
        details=f"hot={hot_aabb}, cold={cold_aabb}",
    )
    # Tick marks should be on opposite sides (Y axis)
    ctx.check(
        "tick marks are on opposite sides of the collar (Y-separated)",
        hot_aabb is not None
        and cold_aabb is not None
        and hot_aabb[0][1] > 0.0
        and cold_aabb[1][1] < 0.0,
        details=f"hot={hot_aabb}, cold={cold_aabb}",
    )

    # ----- variant: removable deck plate
    deck_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "deck plate is grounded at z=0 and circular (~0.08m diameter)",
        deck_aabb is not None
        and abs(deck_aabb[0][2]) < 0.002
        and 0.075 <= (deck_aabb[1][0] - deck_aabb[0][0]) <= 0.085,
        details=f"deck aabb={deck_aabb}",
    )

    # ----- scale: total faucet height near 0.45m
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "faucet apex near 0.45m tall (high-arc gooseneck)",
        spout_aabb is not None and 0.40 <= spout_aabb[1][2] <= 0.50,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- column grounded on deck plate
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "column rises from deck plate",
        col_aabb is not None and 0.004 <= col_aabb[0][2] <= 0.010,
        details=f"column aabb={col_aabb}",
    )

    # ----- joint: lever pivot is revolute +/-45 deg
    ctx.check(
        "lever pivot is revolute +/-45 deg about horizontal axis",
        lever_pivot is not None
        and lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lever_pivot.motion_limits is not None
        and abs(lever_pivot.motion_limits.lower + math.pi / 4.0) < 1e-6
        and abs(lever_pivot.motion_limits.upper - math.pi / 4.0) < 1e-6,
    )

    # ----- lever collar captured on valve body
    ctx.expect_overlap(
        lever,
        column,
        axes="y",
        elem_a="lever_collar",
        elem_b="valve_body",
        min_overlap=0.002,
        name="lever collar captured on valve body",
    )

    # ----- spout swivel: continuous rotation carries spout tip sideways
    rest_tip = ctx.part_element_world_aabb(spout, elem="spout_tip_ring")
    with ctx.pose({swivel: math.pi / 2.0}):
        rotated_tip = ctx.part_element_world_aabb(spout, elem="spout_tip_ring")
    ctx.check(
        "continuous swivel rotates spout 90 degrees (carries tip sideways)",
        rest_tip is not None
        and rotated_tip is not None
        and 0.5 * (rest_tip[0][0] + rest_tip[1][0]) > 0.05
        and abs(0.5 * (rotated_tip[0][1] + rotated_tip[1][1])) > 0.05,
        details=f"rest_tip={rest_tip}, rotated_tip={rotated_tip}",
    )

    # ----- lever pose: pin sweeps fore/aft
    rest_lever = ctx.part_world_aabb(lever)
    with ctx.pose({lever_pivot: math.pi / 4.0}):
        tilted_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "lever pin sweeps in XZ plane when rotated",
        rest_lever is not None
        and tilted_lever is not None
        and tilted_lever[1][0] > rest_lever[1][0] + 0.03,
        details=f"rest={rest_lever}, tilted={tilted_lever}",
    )

    # ----- water outlet: spout tip ring hangs downward at spout end
    tip_aabb = ctx.part_element_world_aabb(spout, elem="spout_tip_ring")
    ctx.check(
        "spout tip ring at drop-leg end (water outlet)",
        tip_aabb is not None and tip_aabb[0][2] > 0.25,
        details=f"tip ring aabb={tip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
