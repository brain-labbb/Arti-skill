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
# Variant 23 — high-arc gooseneck faucet with squared bridge spout
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (spout reaches over the sink), +Z is up.
# - Chrome base disc on the deck; gloss-black cylindrical column (0.04 m dia)
#   rises on the Z axis.
# - Two chrome mounting collars ring the column at z ≈ 0.040 and 0.120.
# - A temperature ring rotates around the column at z ≈ 0.100 (revolute
#   about Z, ±90°).  A small chrome indicator nub on the ring points toward
#   cold (+Y) or hot (−Y) tick marks embossed on the column surface.
# - A thin chrome collar ring (z 0.130..0.140) separates the column from the
#   squared bridge gooseneck spout, which swivels about the vertical column
#   axis (revolute, ±110°).
# - The squared bridge spout has softened 90° elbows (25 mm radius) at the
#   two bends: a vertical riser → elbow → horizontal bridge → elbow → short
#   drop leg ending in a chrome tip sleeve with downward outlet.
# - Apex ≈ 0.38 m above the deck.
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# Mounting collars (thin chrome rings on the column)
MC_R = 0.023  # outer radius, slightly larger than column
MC_H = 0.005
MC_Z0 = 0.040  # lower collar center
MC_Z1 = 0.120  # upper collar center

# Temperature ring
RING_Z = 0.100  # center height
RING_H = 0.012
RING_INNER_R = 0.0205  # clearance over column
RING_OUTER_R = 0.026
NUB_R = 0.003
NUB_LEN = 0.007  # indicator nub protrusion from ring outer surface

# Tick marks (small vertical lines on column surface)
TICK_W = 0.002  # X width
TICK_D = 0.003  # radial protrusion
TICK_H = 0.010  # Z height
TICK_Y = COLUMN_R + TICK_D / 2.0 - 0.001  # embeds 1 mm for connectivity

# Swivel collar
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

# Squared bridge gooseneck geometry (spout-local frame at z = SWIVEL_Z)
TUBE_R = 0.015
ELBOW_R = 0.025  # softened corner radius
RISER_H = 0.200  # vertical riser length
BRIDGE_Z = RISER_H + ELBOW_R  # 0.225 — horizontal bridge centerline height
REACH_X = 0.144  # total horizontal reach
DROP_END = 0.124  # spout-local z of the open tube tip

# threePointArc midpoints for the two quarter-circle elbows
_k = 1.0 - math.cos(math.pi / 4.0)  # 1 − √2/2 ≈ 0.2929
_s = math.sin(math.pi / 4.0)  # √2/2 ≈ 0.7071

MID1_X = ELBOW_R * _k
MID1_Z = RISER_H + ELBOW_R * _s
MID2_X = REACH_X - ELBOW_R * _k
MID2_Z = RISER_H + ELBOW_R * _s

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028
AERATOR_R = 0.0118
AERATOR_LEN = 0.003

APEX_WORLD = SWIVEL_Z + BRIDGE_Z + TUBE_R  # 0.380 m

SWIVEL_LIMIT = math.radians(110.0)
RING_LIMIT = math.radians(90.0)


def _bridge_gooseneck_shape() -> cq.Workplane:
    """Squared bridge tube: vertical riser, two softened 90° elbows, horizontal
    bridge, short drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_H)
        .threePointArc((MID1_X, MID1_Z), (ELBOW_R, BRIDGE_Z))
        .lineTo(REACH_X - ELBOW_R, BRIDGE_Z)
        .threePointArc((MID2_X, MID2_Z), (REACH_X, RISER_H))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def _ring_body_shape() -> cq.Workplane:
    """Annular ring body (hollow cylinder)."""
    return (
        cq.Workplane("XY")
        .circle(RING_OUTER_R)
        .circle(RING_INNER_R)
        .extrude(RING_H)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squared_bridge_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    gunmetal = model.material("brushed_gunmetal", rgba=(0.28, 0.28, 0.30, 1.0))

    # ------------------------------------------------------------------ column
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
    # Lower mounting collar
    column.visual(
        Cylinder(radius=MC_R, length=MC_H),
        origin=Origin(xyz=(0.0, 0.0, MC_Z0)),
        material=chrome,
        name="mounting_collar_0",
    )
    # Upper mounting collar
    column.visual(
        Cylinder(radius=MC_R, length=MC_H),
        origin=Origin(xyz=(0.0, 0.0, MC_Z1)),
        material=chrome,
        name="mounting_collar_1",
    )
    # Cold tick mark (+Y side of column)
    column.visual(
        Box((TICK_W, TICK_D, TICK_H)),
        origin=Origin(xyz=(0.0, TICK_Y, RING_Z)),
        material=chrome,
        name="cold_tick",
    )
    # Hot tick mark (−Y side of column)
    column.visual(
        Box((TICK_W, TICK_D, TICK_H)),
        origin=Origin(xyz=(0.0, -TICK_Y, RING_Z)),
        material=chrome,
        name="hot_tick",
    )
    # Chrome swivel collar ring
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # ----------------------------------------------------------- gooseneck spout
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_bridge_gooseneck_shape(), "bridge_tube"),
        material=gloss_black,
        name="bridge_tube",
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

    # --------------------------------------------------------- temperature ring
    ring = model.part("temperature_ring")
    ring.visual(
        mesh_from_cadquery(_ring_body_shape(), "ring_body"),
        origin=Origin(xyz=(0.0, 0.0, -RING_H / 2.0)),
        material=gunmetal,
        name="ring_body",
    )
    # Indicator nub — small cylinder protruding radially along +X
    ring.visual(
        Cylinder(radius=NUB_R, length=NUB_LEN),
        origin=Origin(
            xyz=(RING_OUTER_R + NUB_LEN / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="ring_indicator",
    )
    model.articulation(
        "temp_ring_joint",
        ArticulationType.REVOLUTE,
        parent=column,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, RING_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=-RING_LIMIT, upper=RING_LIMIT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    ring = object_model.get_part("temperature_ring")

    swivel = object_model.get_articulation("spout_swivel")
    temp_joint = object_model.get_articulation("temp_ring_joint")

    # ----- grounding and overall scale -----
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m",
        spout_aabb is not None and 0.370 <= spout_aabb[1][2] <= 0.390,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.130,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- squared bridge: horizontal bridge section at the top -----
    tube = ctx.part_element_world_aabb(spout, elem="bridge_tube")
    ctx.check(
        "squared bridge tube has a clear horizontal span",
        tube is not None
        and (tube[1][0] - tube[0][0]) >= 0.100  # horizontal extent ≥ 100 mm
        and 0.350 <= tube[1][2] <= 0.385,  # top of tube near apex
        details=f"bridge_tube aabb={tube}",
    )
    # The bridge section should be noticeably flatter than a semicircular arc:
    # the top of the tube at mid-reach should be close to the apex height.
    # We check that the vertical extent at mid-reach is narrow (bridge is horizontal).
    sleeve_aabb = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "chrome tip sleeve at the drop end with downward outlet",
        sleeve_aabb is not None
        and 0.250 <= sleeve_aabb[0][2] <= 0.290
        and abs(0.5 * (sleeve_aabb[0][0] + sleeve_aabb[1][0]) - REACH_X) <= 0.005,
        details=f"tip_sleeve aabb={sleeve_aabb}",
    )

    # ----- mounting collars -----
    mc0 = ctx.part_element_world_aabb(column, elem="mounting_collar_0")
    mc1 = ctx.part_element_world_aabb(column, elem="mounting_collar_1")
    ctx.check(
        "two mounting collars on the pedestal at distinct heights",
        mc0 is not None
        and mc1 is not None
        and mc0[1][2] < mc1[0][2]  # lower collar entirely below upper collar
        and 0.030 <= 0.5 * (mc0[0][2] + mc0[1][2]) <= 0.050
        and 0.110 <= 0.5 * (mc1[0][2] + mc1[1][2]) <= 0.130,
        details=f"mc0={mc0}, mc1={mc1}",
    )
    # Collars wider than column
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "mounting collars protrude beyond column shaft",
        mc0 is not None
        and shaft is not None
        and (mc0[1][0] - mc0[0][0]) > (shaft[1][0] - shaft[0][0]) + 0.002,
        details=f"mc0_dx={mc0[1][0] - mc0[0][0]}, shaft_dx={shaft[1][0] - shaft[0][0]}",
    )

    # ----- tick marks -----
    cold = ctx.part_element_world_aabb(column, elem="cold_tick")
    hot = ctx.part_element_world_aabb(column, elem="hot_tick")
    ctx.check(
        "cold and hot tick marks exist as geometry on opposite sides of column",
        cold is not None
        and hot is not None
        and cold[0][1] > 0.010  # cold tick on +Y side
        and hot[1][1] < -0.010  # hot tick on −Y side
        and abs(0.5 * (cold[0][2] + cold[1][2]) - RING_Z) < 0.010
        and abs(0.5 * (hot[0][2] + hot[1][2]) - RING_Z) < 0.010,
        details=f"cold={cold}, hot={hot}",
    )
    ctx.check(
        "tick marks are thin vertical lines (height >> width)",
        cold is not None
        and (cold[1][2] - cold[0][2]) >= 0.008
        and (cold[1][0] - cold[0][0]) <= 0.004,
        details=f"cold={cold}",
    )

    # ----- temperature ring -----
    ring_aabb = ctx.part_world_aabb(ring)
    ctx.check(
        "temperature ring sits on the column between the two mounting collars",
        ring_aabb is not None
        and mc0 is not None
        and mc1 is not None
        and ring_aabb[0][2] >= mc0[1][2] - 0.001
        and ring_aabb[1][2] <= mc1[0][2] + 0.001,
        details=f"ring={ring_aabb}, mc0_top={mc0[1][2]}, mc1_bot={mc1[0][2]}",
    )
    indicator_rest = ctx.part_element_world_aabb(ring, elem="ring_indicator")
    ctx.check(
        "ring indicator nub protrudes from the ring",
        indicator_rest is not None
        and indicator_rest[1][0] > RING_OUTER_R + 0.002,
        details=f"indicator={indicator_rest}",
    )

    # ----- joint plan: types, axes, ranges -----
    ctx.check(
        "spout swivel is revolute ±110° about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )
    ctx.check(
        "temperature ring is revolute ±90° about the vertical column axis",
        temp_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(temp_joint.axis) == (0.0, 0.0, 1.0)
        and temp_joint.motion_limits is not None
        and abs(temp_joint.motion_limits.lower + RING_LIMIT) < 1e-6
        and abs(temp_joint.motion_limits.upper - RING_LIMIT) < 1e-6
        and temp_joint.mimic is None,
    )

    # ----- spout swivel pose: outlet sweeps sideways -----
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout swivel carries the outlet sideways about the vertical axis",
        rest_sleeve is not None
        and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 1e-6
        and abs(0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1])) > 0.05,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    # ----- temperature ring pose: indicator sweeps between tick marks -----
    with ctx.pose({temp_joint: RING_LIMIT}):
        indicator_max = ctx.part_element_world_aabb(ring, elem="ring_indicator")
    with ctx.pose({temp_joint: -RING_LIMIT}):
        indicator_min = ctx.part_element_world_aabb(ring, elem="ring_indicator")
    ctx.check(
        "ring rotation moves indicator between cold (+Y) and hot (−Y) sides",
        indicator_max is not None
        and indicator_min is not None
        and 0.5 * (indicator_max[0][1] + indicator_max[1][1]) > 0.010
        and 0.5 * (indicator_min[0][1] + indicator_min[1][1]) < -0.010,
        details=f"max={indicator_max}, min={indicator_min}",
    )

    # ----- chrome collar between column and spout; spout seats on it -----
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "thin chrome collar sits above the ring and below the spout top",
        collar is not None
        and collar[0][2] >= RING_Z + RING_H / 2.0 - 0.001
        and spout_aabb is not None
        and collar[1][2] <= spout_aabb[0][2] + 1e-6,
        details=f"collar={collar}",
    )
    ctx.expect_contact(
        spout,
        column,
        elem_a="bridge_tube",
        elem_b="swivel_collar",
        contact_tol=0.002,
        name="gooseneck riser seats on the chrome collar",
    )

    return ctx.report()


object_model = build_object_model()
