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
# Variant 01: High-arc gooseneck faucet with single side lever.
#
# Structural changes from parent monobloc mixer tap:
# - Taller gooseneck spout (~0.44 m apex) with tighter forward bend
#   (arc radius 0.050 m, reach ~0.10 m).
# - Single side lever on the right (+Y) replaces the dual pin levers and
#   cross-cylinder valve bodies.
# - Hot/cold tick marks as small raised geometry on the column (not text).
# - Removable circular deck plate under the chrome base disc.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (spout reach direction), +Z is up.
# - Deck plate (0.116 m dia chrome disc) at z 0..0.005.
# - Chrome base disc (0.084 m dia) at z 0.005..0.013.
# - Gloss-black column shaft (0.040 m dia) rises from z 0.013 to z 0.142.
# - Hot/cold tick marks at z ~0.115, on the ±Y sides of the column.
# - Chrome collar ring at z 0.145..0.155.
# - Gooseneck spout swivels about the vertical axis at z = 0.155.
# - Side lever on +Y at z = 0.090, revolute about X axis, 0..75 deg.
# ---------------------------------------------------------------------------

# Deck plate
DECK_R = 0.058
DECK_H = 0.005

# Base disc
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008

# Column
COLUMN_R = 0.020
COLUMN_BOTTOM = DECK_H + BASE_DISC_H  # 0.013
COLUMN_TOP = 0.148

# Collar
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.155  # collar top = spout swivel origin

# Gooseneck — taller arc, tighter bend
TUBE_R = 0.014
ARC_R = 0.050  # tighter bend (parent was 0.072)
RISER_TOP = 0.230  # centerline apex = 0.280; world apex = 0.155 + 0.280 + 0.014 = 0.449
REACH_X = 2.0 * ARC_R  # 0.100
DROP_END = 0.10  # spout-local z of outlet tip (world z = 0.255)

SLEEVE_R = 0.0155
SLEEVE_LEN = 0.028
AERATOR_R = 0.011
AERATOR_LEN = 0.003

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # ~0.449

SWIVEL_LIMIT = math.radians(110.0)

# Side lever
LEVER_PIVOT_Z = 0.090
LEVER_BOSS_R = 0.010
LEVER_BOSS_LEN = 0.016
LEVER_ARM_LEN = 0.110
LEVER_ARM_R = 0.006
LEVER_GRIP_R = 0.009
LEVER_GRIP_LEN = 0.020
LEVER_UPPER = math.radians(75.0)

# Tick marks (raised geometry, not text)
TICK_SX = 0.004  # tangential width
TICK_SY = 0.002  # radial depth (half embedded in column)
TICK_SZ = 0.008  # vertical height
TICK_Z_CENTER = 0.118
TICK_SPACING = 0.012  # vertical spacing between ticks


def _gooseneck_shape() -> cq.Workplane:
    """Tall swan-neck tube: straight riser, tight semicircular arc, drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    tick_white = model.material("tick_indicator", rgba=(0.92, 0.92, 0.90, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")

    # Removable circular deck plate (wider than base disc, under it)
    column.visual(
        Cylinder(radius=DECK_R, length=DECK_H),
        origin=Origin(xyz=(0.0, 0.0, DECK_H / 2.0)),
        material=chrome,
        name="deck_plate",
    )
    # Chrome base disc on top of deck plate
    column.visual(
        Cylinder(radius=BASE_DISC_R, length=BASE_DISC_H),
        origin=Origin(xyz=(0.0, 0.0, DECK_H + BASE_DISC_H / 2.0)),
        material=chrome,
        name="base_disc",
    )
    # Gloss-black column shaft
    shaft_len = COLUMN_TOP - COLUMN_BOTTOM
    column.visual(
        Cylinder(radius=COLUMN_R, length=shaft_len),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_BOTTOM + shaft_len / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )
    # Chrome collar ring
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # Hot/cold tick marks as raised geometry (3 per side)
    for i, dz in enumerate([-TICK_SPACING, 0.0, TICK_SPACING]):
        # Cold ticks on +Y side
        column.visual(
            Box((TICK_SX, TICK_SY, TICK_SZ)),
            origin=Origin(xyz=(0.0, COLUMN_R, TICK_Z_CENTER + dz)),
            material=tick_white,
            name=f"cold_tick_{i}",
        )
        # Hot ticks on -Y side
        column.visual(
            Box((TICK_SX, TICK_SY, TICK_SZ)),
            origin=Origin(xyz=(0.0, -COLUMN_R, TICK_Z_CENTER + dz)),
            material=tick_white,
            name=f"hot_tick_{i}",
        )

    # --------------------------------------------------------------- gooseneck
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
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5, lower=-SWIVEL_LIMIT, upper=SWIVEL_LIMIT
        ),
    )

    # ----------------------------------------------------------- side lever
    lever = model.part("side_lever")
    # Boss — short cylinder along Y, partially embedded in column
    boss_cy = LEVER_BOSS_LEN / 2.0 - 0.004  # 4 mm embed
    lever.visual(
        Cylinder(radius=LEVER_BOSS_R, length=LEVER_BOSS_LEN),
        origin=Origin(xyz=(0.0, boss_cy, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="lever_boss",
    )
    # Lever arm — extends outward in +Y
    arm_y0 = LEVER_BOSS_LEN - 0.004  # outer face of boss
    arm_cy = arm_y0 + LEVER_ARM_LEN / 2.0
    lever.visual(
        Cylinder(radius=LEVER_ARM_R, length=LEVER_ARM_LEN),
        origin=Origin(xyz=(0.0, arm_cy, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="lever_arm",
    )
    # Grip — slightly thicker end cap
    grip_cy = arm_y0 + LEVER_ARM_LEN + LEVER_GRIP_LEN / 2.0
    lever.visual(
        Cylinder(radius=LEVER_GRIP_R, length=LEVER_GRIP_LEN),
        origin=Origin(xyz=(0.0, grip_cy, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="lever_grip",
    )
    model.articulation(
        "lever_tilt",
        ArticulationType.REVOLUTE,
        parent=column,
        child=lever,
        origin=Origin(xyz=(0.0, COLUMN_R, LEVER_PIVOT_Z)),
        # Rotation about +X: right-hand rule rotates +Y toward +Z,
        # so positive q tilts the lever upward from horizontal.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=LEVER_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    lever = object_model.get_part("side_lever")

    swivel = object_model.get_articulation("spout_swivel")
    tilt = object_model.get_articulation("lever_tilt")

    # ---- intentional overlap: lever boss seats into column
    ctx.allow_overlap(
        lever,
        column,
        elem_a="lever_boss",
        elem_b="column_shaft",
        reason="Lever boss intentionally seats 4 mm into the column for mounting.",
    )

    # ---- deck plate: wider than base disc, at the bottom
    deck = ctx.part_element_world_aabb(column, elem="deck_plate")
    disc = ctx.part_element_world_aabb(column, elem="base_disc")
    ctx.check(
        "removable deck plate is wider than the base disc and sits at deck level",
        deck is not None
        and disc is not None
        and (deck[1][0] - deck[0][0]) > (disc[1][0] - disc[0][0])
        and abs(deck[0][2]) <= 0.002
        and deck[1][2] <= disc[0][2] + 0.001,
        details=f"deck={deck}, base_disc={disc}",
    )

    # ---- taller gooseneck spout
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex is taller than parent (above 0.42 m)",
        spout_aabb is not None and spout_aabb[1][2] >= 0.42,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- tighter forward bend
    ctx.check(
        "gooseneck forward reach is tighter than parent (reach < 0.13 m)",
        spout_aabb is not None and spout_aabb[1][0] < 0.13,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- gooseneck arcs forward over the sink
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.08,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- hot/cold tick marks exist as geometry
    cold_0 = ctx.part_element_world_aabb(column, elem="cold_tick_0")
    hot_0 = ctx.part_element_world_aabb(column, elem="hot_tick_0")
    ctx.check(
        "cold tick marks exist as geometry on the +Y side of the column",
        cold_0 is not None
        and 0.5 * (cold_0[0][1] + cold_0[1][1]) > COLUMN_R - 0.002,
        details=f"cold_tick_0={cold_0}",
    )
    ctx.check(
        "hot tick marks exist as geometry on the -Y side of the column",
        hot_0 is not None
        and 0.5 * (hot_0[0][1] + hot_0[1][1]) < -(COLUMN_R - 0.002),
        details=f"hot_tick_0={hot_0}",
    )

    # ---- tick marks are on opposite sides
    ctx.check(
        "cold and hot tick marks are on opposite sides of the column",
        cold_0 is not None
        and hot_0 is not None
        and cold_0[0][1] > 0.0
        and hot_0[1][1] < 0.0,
        details=f"cold={cold_0}, hot={hot_0}",
    )

    # ---- single side lever exists
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "single side lever exists on the +Y side of the column",
        lever_aabb is not None
        and 0.5 * (lever_aabb[0][1] + lever_aabb[1][1]) > 0.01,
        details=f"lever aabb={lever_aabb}",
    )

    # ---- lever boss seats into the column
    ctx.expect_overlap(
        lever,
        column,
        axes="y",
        elem_a="lever_boss",
        elem_b="column_shaft",
        min_overlap=0.002,
        name="lever boss seats into the column shaft",
    )

    # ---- lever tilt joint: revolute, non-fixed, correct axis and limits
    ctx.check(
        "lever tilt is revolute 0..75 deg about the horizontal X axis",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(tilt.axis) == (1.0, 0.0, 0.0)
        and tilt.motion_limits is not None
        and abs(tilt.motion_limits.lower) < 1e-6
        and abs(tilt.motion_limits.upper - LEVER_UPPER) < 1e-6
        and tilt.mimic is None,
    )

    # ---- spout swivel joint: revolute, -110..+110 deg about Z
    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # ---- lever pose: positive q tilts lever upward
    rest_lever = ctx.part_world_aabb(lever)
    with ctx.pose({tilt: LEVER_UPPER}):
        tilted_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "lever tilts upward at max positive q (z increases)",
        rest_lever is not None
        and tilted_lever is not None
        and tilted_lever[1][2] > rest_lever[1][2] + 0.02,
        details=f"rest={rest_lever}, tilted={tilted_lever}",
    )

    # ---- spout swivel pose: outlet sweeps sideways
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout swivel carries the outlet sideways about the vertical axis",
        rest_sleeve is not None
        and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 0.01
        and abs(0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1])) > 0.03,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    # ---- gooseneck seats on the collar
    ctx.expect_contact(
        spout,
        column,
        elem_a="gooseneck_tube",
        elem_b="swivel_collar",
        contact_tol=0.002,
        name="gooseneck riser seats on the chrome collar",
    )

    # ---- chrome tip sleeve with downward outlet
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    aerator = ctx.part_element_world_aabb(spout, elem="outlet_aerator")
    ctx.check(
        "chrome tip sleeve at spout end with downward outlet below it",
        sleeve is not None
        and aerator is not None
        and aerator[0][2] < sleeve[0][2],
        details=f"sleeve={sleeve}, aerator={aerator}",
    )

    # ---- at least one non-fixed joint exists
    joints = object_model.articulations
    non_fixed = [
        j for j in joints
        if j.articulation_type in (
            ArticulationType.REVOLUTE,
            ArticulationType.PRISMATIC,
            ArticulationType.CONTINUOUS,
        )
    ]
    ctx.check(
        "at least one non-fixed joint exists",
        len(non_fixed) >= 1,
        details=f"non-fixed joints: {[j.name for j in non_fixed]}",
    )

    return ctx.report()


object_model = build_object_model()
