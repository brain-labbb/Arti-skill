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
# High-arc gooseneck faucet variant: faceted segmented neck, ~0.45 m tall.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (direction the gooseneck reaches over the sink), +Z is up.
# - Tapered column rises on the Z axis with two mounting collars near the base.
# - Faceted gooseneck spout (polyline segments, visible bends) swivels about
#   the vertical column axis (+/-60 deg).
# - Single side lever on the right (-Y) side, revolute about horizontal axis,
#   +/-45 deg. Cold/hot tick marks as raised geometry near the lever.
# ---------------------------------------------------------------------------

# Column
COLUMN_BASE_R = 0.030
COLUMN_MID_R = 0.022
COLUMN_TOP_R = 0.016
COLUMN_MID_Z = 0.18
COLUMN_TOP_Z = 0.295
SWIVEL_Z = 0.307  # top of swivel collar = base of gooseneck

# Mounting collars on pedestal
COLLAR_LOWER_Z = 0.012
COLLAR_UPPER_Z = 0.035
COLLAR_R = 0.034
COLLAR_THICKNESS = 0.006

# Swivel collar
SWIVEL_COLLAR_R = 0.018
SWIVEL_COLLAR_LEN = 0.012

# Faceted gooseneck path points (in XZ plane, spout-local coords)
# These define a segmented polyline with visible angular bends.
TUBE_R = 0.010
FACET_POINTS = [
    (0.000, 0.000),   # base
    (0.000, 0.055),   # straight riser
    (0.025, 0.095),   # bend 1: angling outward
    (0.060, 0.125),   # bend 2: continuing up
    (0.100, 0.143),   # bend 3: approaching apex
    (0.135, 0.140),   # bend 4: cresting
    (0.160, 0.120),   # bend 5: starting descent
    (0.175, 0.085),   # bend 6: descending
    (0.175, 0.010),   # drop leg end (spout tip)
]

# Valve + lever
VALVE_Z = 0.14
VALVE_R = 0.013
VALVE_LEN = 0.050
VALVE_Y_CENTER = -0.043
LEVER_JOINT_Y = -0.065
LEVER_PIN_LEN = 0.095

# Tick marks (cold/hot indicators as geometry near the lever)
TICK_WIDTH = 0.003
TICK_HEIGHT = 0.012
TICK_DEPTH = 0.003


def _column_shape() -> cq.Workplane:
    """Tapered conical column, 0.06 m diameter at the deck."""
    return (
        cq.Workplane("XY")
        .circle(COLUMN_BASE_R)
        .workplane(offset=COLUMN_MID_Z)
        .circle(COLUMN_MID_R)
        .workplane(offset=COLUMN_TOP_Z - COLUMN_MID_Z)
        .circle(COLUMN_TOP_R)
        .loft()
    )


def _faceted_gooseneck_shape() -> cq.Workplane:
    """Faceted tube built from a polyline path with visible segmented bends."""
    wp = cq.Workplane("XZ")
    wp = wp.moveTo(*FACET_POINTS[0])
    for pt in FACET_POINTS[1:]:
        wp = wp.lineTo(*pt)
    path = wp
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="faceted_gooseneck_faucet")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    dark_gold = model.material("antique_gold", rgba=(0.60, 0.45, 0.18, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    chrome = model.material("chrome_accent", rgba=(0.75, 0.75, 0.78, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        mesh_from_cadquery(_column_shape(), "tapered_column"),
        material=gold,
        name="tapered_column",
    )
    # Swivel collar at top of column
    column.visual(
        Cylinder(radius=SWIVEL_COLLAR_R, length=SWIVEL_COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + SWIVEL_COLLAR_LEN / 2.0)),
        material=dark_gold,
        name="swivel_collar",
    )
    # Mounting collar 1 (lower) - wider ring near deck
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_LOWER_Z + COLLAR_THICKNESS / 2.0)),
        material=dark_gold,
        name="mounting_collar_lower",
    )
    # Mounting collar 2 (upper) - wider ring above the first
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_UPPER_Z + COLLAR_THICKNESS / 2.0)),
        material=dark_gold,
        name="mounting_collar_upper",
    )
    # Horizontal valve body on the right (-Y) side, mid-column height.
    column.visual(
        Cylinder(radius=VALVE_R, length=VALVE_LEN),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER, VALVE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="valve_body",
    )
    # Cold tick mark - small raised box on the valve body (rear/cold side)
    column.visual(
        Box((TICK_WIDTH, TICK_DEPTH, TICK_HEIGHT)),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER - 0.012, VALVE_Z + 0.018)),
        material=black,
        name="cold_tick",
    )
    # Hot tick mark - small raised box on the valve body (front/hot side)
    column.visual(
        Box((TICK_WIDTH, TICK_DEPTH, TICK_HEIGHT)),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER - 0.012, VALVE_Z - 0.018)),
        material=black,
        name="hot_tick",
    )
    # Center tick mark (neutral position indicator)
    column.visual(
        Box((TICK_WIDTH * 1.4, TICK_DEPTH, TICK_HEIGHT * 0.7)),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER - 0.012, VALVE_Z)),
        material=chrome,
        name="center_tick",
    )

    # --------------------------------------------------------- faceted spout
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_faceted_gooseneck_shape(), "faceted_tube"),
        material=gold,
        name="faceted_tube",
    )
    # Aerator ring at spout tip
    spout.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(FACET_POINTS[-1][0], 0.0, FACET_POINTS[-1][1] - 0.004)),
        material=chrome,
        name="aerator_ring",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5, lower=-math.pi / 3.0, upper=math.pi / 3.0
        ),
    )

    # ------------------------------------------------------------------- lever
    lever = model.part("side_lever")
    lever.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="lever_base",
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
        name="lever_knob",
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

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    lever = object_model.get_part("side_lever")

    swivel = object_model.get_articulation("spout_swivel")
    lever_pivot = object_model.get_articulation("lever_pivot")

    # Intentional overlap: lever base seated on valve body end
    ctx.allow_overlap(
        lever,
        column,
        elem_a="lever_base",
        elem_b="valve_body",
        reason="Lever base collar is captured/seated on the valve body end.",
    )

    # ----- Scale and grounding
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "column grounded at deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )

    # ----- Two mounting collars on pedestal
    lower_collar = ctx.part_element_world_aabb(column, elem="mounting_collar_lower")
    upper_collar = ctx.part_element_world_aabb(column, elem="mounting_collar_upper")
    ctx.check(
        "lower mounting collar exists near deck",
        lower_collar is not None and lower_collar[0][2] < 0.025,
        details=f"lower_collar={lower_collar}",
    )
    ctx.check(
        "upper mounting collar exists above lower",
        upper_collar is not None and lower_collar is not None
        and upper_collar[0][2] > lower_collar[1][2] - 0.005,
        details=f"upper_collar={upper_collar}, lower_collar={lower_collar}",
    )
    ctx.check(
        "mounting collars wider than column base",
        lower_collar is not None
        and (lower_collar[1][0] - lower_collar[0][0]) > 0.060,
        details=f"lower_collar width={lower_collar}",
    )

    # ----- Faceted gooseneck: apex height and segmented geometry
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.45 m (tall high-arc silhouette)",
        spout_aabb is not None and 0.42 <= spout_aabb[1][2] <= 0.48,
        details=f"spout aabb={spout_aabb}",
    )
    tube_aabb = ctx.part_element_world_aabb(spout, elem="faceted_tube")
    ctx.check(
        "faceted tube has significant horizontal reach",
        tube_aabb is not None and (tube_aabb[1][0] - tube_aabb[0][0]) > 0.14,
        details=f"tube aabb={tube_aabb}",
    )

    # ----- Tick marks as geometry (cold/hot indicators)
    cold_tick = ctx.part_element_world_aabb(column, elem="cold_tick")
    hot_tick = ctx.part_element_world_aabb(column, elem="hot_tick")
    center_tick = ctx.part_element_world_aabb(column, elem="center_tick")
    ctx.check(
        "cold tick mark exists as geometry",
        cold_tick is not None,
        details=f"cold_tick={cold_tick}",
    )
    ctx.check(
        "hot tick mark exists as geometry",
        hot_tick is not None,
        details=f"hot_tick={hot_tick}",
    )
    ctx.check(
        "cold and hot ticks are separated along Z (above/below lever axis)",
        cold_tick is not None and hot_tick is not None
        and cold_tick[0][2] > hot_tick[1][2] - 0.005,
        details=f"cold={cold_tick}, hot={hot_tick}",
    )
    ctx.check(
        "center tick between cold and hot",
        center_tick is not None and cold_tick is not None and hot_tick is not None
        and center_tick[0][2] <= cold_tick[0][2]
        and center_tick[1][2] >= hot_tick[1][2],
        details=f"center={center_tick}, cold={cold_tick}, hot={hot_tick}",
    )

    # ----- Joint: spout swivel revolute about vertical axis
    ctx.check(
        "spout swivel is revolute +/-60 deg about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + math.pi / 3.0) < 1e-6
        and abs(swivel.motion_limits.upper - math.pi / 3.0) < 1e-6
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )

    # ----- Joint: lever pivot revolute about horizontal axis
    ctx.check(
        "lever pivot is revolute +/-45 deg about horizontal axis",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lever_pivot.motion_limits is not None
        and abs(lever_pivot.motion_limits.lower + math.pi / 4.0) < 1e-6
        and abs(lever_pivot.motion_limits.upper - math.pi / 4.0) < 1e-6
        and tuple(lever_pivot.axis) == (0.0, 1.0, 0.0),
    )

    # ----- Lever pose: pin sweeps fore/aft
    rest_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "lever pin extends ~0.095 m above valve body",
        rest_lever is not None and 0.085 <= (rest_lever[1][2] - VALVE_Z) <= 0.125,
        details=f"lever aabb={rest_lever}",
    )
    with ctx.pose({lever_pivot: math.pi / 4.0}):
        tilted_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "lever sweeps in X when rotated (flow/temperature control)",
        rest_lever is not None
        and tilted_lever is not None
        and abs(tilted_lever[1][0] - rest_lever[1][0]) > 0.04,
        details=f"rest={rest_lever}, tilted={tilted_lever}",
    )

    # ----- Swivel pose: spout tip moves sideways
    rest_spout_tip = ctx.part_element_world_aabb(spout, elem="aerator_ring")
    with ctx.pose({swivel: math.pi / 3.0}):
        swiveled_tip = ctx.part_element_world_aabb(spout, elem="aerator_ring")
    ctx.check(
        "spout swivel carries aerator sideways about column axis",
        rest_spout_tip is not None
        and swiveled_tip is not None
        and abs(swiveled_tip[0][1] - rest_spout_tip[0][1]) > 0.08,
        details=f"rest={rest_spout_tip}, swiveled={swiveled_tip}",
    )

    # ----- Lever base captured on valve body
    ctx.expect_overlap(
        lever,
        column,
        axes="y",
        elem_a="lever_base",
        elem_b="valve_body",
        min_overlap=0.002,
        name="lever base captured on valve body",
    )

    return ctx.report()


object_model = build_object_model()
