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
# Variant 23 — squared bridge gooseneck faucet with temperature ring.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (direction the gooseneck reaches over the sink), +Z is up.
# - Tapered conical column rises on the Z axis to z = COLUMN_TOP_Z.
# - Two mounting collars wrap the column at mid-heights.
# - A temperature ring (revolute about Z) rotates around the lower pedestal,
#   with a pointer indicator. Hot/cold tick marks are static geometry on the
#   column above the ring.
# - The squared bridge gooseneck swivels about the vertical column axis
#   (+/-60 deg). It has straight riser, horizontal bridge, and drop leg
#   with softened (filleted) elbows.
# - A tapered spray head hangs at the spout tip.
# - A pin lever on the right side (-Y) at mid-height rotates about the
#   horizontal valve axis (+/-45 deg) for flow/temperature control.
# ---------------------------------------------------------------------------

# Column
COLUMN_BASE_R = 0.030
COLUMN_MID_R = 0.020
COLUMN_TOP_R = 0.0155
COLUMN_MID_Z = 0.18
COLUMN_TOP_Z = 0.295
SWIVEL_Z = 0.307  # top of swivel collar = base of the gooseneck

# Mounting collars
COLLAR_LOWER_Z = 0.215
COLLAR_UPPER_Z = 0.260
COLLAR_R = 0.0185
COLLAR_H = 0.010

# Temperature ring (around lower pedestal)
RING_Z_BASE = 0.035  # bottom of ring
RING_H = 0.015
RING_INNER_R = 0.027  # smaller than column radius (~0.0276) for reliable mesh contact
RING_OUTER_R = 0.036
RING_POINTER_R = 0.003

# Tick marks (hot/cold, on column above ring)
TICK_Z = 0.058
TICK_W = 0.003
TICK_H = 0.012
TICK_D = 0.003

# Squared bridge gooseneck (spout-local coords, frame at swivel top)
TUBE_R = 0.012
BRIDGE_RISER = 0.128  # straight riser height before fillet
BRIDGE_FILLET = 0.028  # fillet radius at elbows
BRIDGE_REACH = 0.17    # horizontal reach
DROP_END = 0.003       # spout-local z of open tube tip

# Spray head
HEAD_LEN = 0.090
HEAD_TOP_R = 0.013
HEAD_MID_R = 0.016
HEAD_BOT_R = 0.011
NOZZLE_R = 0.009

# Valve + lever
VALVE_Z = 0.14
VALVE_R = 0.013
VALVE_LEN = 0.055
VALVE_Y_CENTER = -0.0455
LEVER_JOINT_Y = -0.070
LEVER_PIN_LEN = 0.102


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


def _bridge_gooseneck_shape() -> cq.Workplane:
    """Squared bridge gooseneck with softened (filleted) elbows.

    Path in XZ plane: straight riser -> filleted elbow -> horizontal bridge ->
    filleted elbow -> drop leg.
    """
    f = BRIDGE_FILLET
    k = 1.0 - math.sqrt(2.0) / 2.0  # ~0.2929, quarter-arc midpoint factor
    rh = BRIDGE_RISER  # riser height (to top of bridge before fillet adds)
    reach = BRIDGE_REACH

    # Fillet midpoint offsets
    fm_x = f * k
    fm_z = f * k

    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        # Riser
        .lineTo(0.0, rh - f)
        # Top-left fillet (quarter arc, vertical to horizontal)
        # Arc center at (f, rh-f); mid at 135° from center
        .threePointArc(
            (fm_x, rh - fm_z),
            (f, rh),
        )
        # Bridge top (horizontal)
        .lineTo(reach - f, rh)
        # Top-right fillet (quarter arc, horizontal to vertical)
        # Arc center at (reach-f, rh-f); mid at 45° from center
        .threePointArc(
            (reach - fm_x, rh - fm_z),
            (reach, rh - f),
        )
        # Drop leg
        .lineTo(reach, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def _ring_shape() -> cq.Workplane:
    """Annular temperature ring (washer shape)."""
    return (
        cq.Workplane("XY")
        .circle(RING_OUTER_R)
        .circle(RING_INNER_R)
        .extrude(RING_H)
    )


def _head_shape() -> cq.Workplane:
    """Tapered pull-down spray head pointing straight down (loft)."""
    return (
        cq.Workplane("XY")
        .circle(HEAD_TOP_R)
        .workplane(offset=-0.028)
        .circle(HEAD_MID_R)
        .workplane(offset=-0.030)
        .circle(HEAD_MID_R + 0.001)
        .workplane(offset=-0.020)
        .circle(HEAD_BOT_R)
        .loft()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_gooseneck_faucet_v23")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    dark_gold = model.material("satin_gold", rgba=(0.65, 0.50, 0.22, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    red = model.material("hot_red", rgba=(0.85, 0.13, 0.10, 1.0))
    blue = model.material("cold_blue", rgba=(0.20, 0.45, 0.90, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("pedestal")
    column.visual(
        mesh_from_cadquery(_column_shape(), "tapered_column"),
        material=gold,
        name="column_body",
    )
    # Swivel collar at top
    column.visual(
        Cylinder(radius=0.0175, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + 0.006)),
        material=gold,
        name="swivel_collar",
    )
    # Two mounting collars on the pedestal
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_LOWER_Z + COLLAR_H / 2.0)),
        material=dark_gold,
        name="mount_collar_lower",
    )
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_UPPER_Z + COLLAR_H / 2.0)),
        material=dark_gold,
        name="mount_collar_upper",
    )
    # Hot tick mark (red, on +X side of column, just above the ring)
    # Column radius at TICK_Z: interpolate from base
    col_r_at_tick = COLUMN_BASE_R - (COLUMN_BASE_R - COLUMN_MID_R) * TICK_Z / COLUMN_MID_Z
    # Embed 1 mm into the column surface for visual connectivity
    column.visual(
        Box((TICK_D, TICK_W, TICK_H)),
        origin=Origin(xyz=(col_r_at_tick + TICK_D / 2.0 - 0.001, 0.0, TICK_Z + TICK_H / 2.0)),
        material=red,
        name="hot_tick",
    )
    # Cold tick mark (blue, on -X side of column)
    column.visual(
        Box((TICK_D, TICK_W, TICK_H)),
        origin=Origin(xyz=(-(col_r_at_tick + TICK_D / 2.0 - 0.001), 0.0, TICK_Z + TICK_H / 2.0)),
        material=blue,
        name="cold_tick",
    )
    # Horizontal valve body on the right (-Y) side, mid-column height.
    column.visual(
        Cylinder(radius=VALVE_R, length=VALVE_LEN),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER, VALVE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="valve_body",
    )

    # -------------------------------------------------------- temperature ring
    # The ring part frame sits at the articulation origin (z = RING_Z_BASE + RING_H/2).
    # Visuals use part-local coordinates relative to that frame.
    ring = model.part("temperature_ring")
    ring.visual(
        mesh_from_cadquery(_ring_shape(), "ring_band"),
        # Extrusion starts at z=-RING_H/2 so the ring is centered on the part frame
        origin=Origin(xyz=(0.0, 0.0, -RING_H / 2.0)),
        material=dark_gold,
        name="ring_band",
    )
    # Pointer/indicator on the ring outer surface (points outward along +X)
    ring.visual(
        Box((0.004, 0.003, 0.008)),
        origin=Origin(
            xyz=(RING_OUTER_R + 0.002, 0.0, 0.0)
        ),
        material=black,
        name="ring_pointer",
    )
    model.articulation(
        "ring_rotate",
        ArticulationType.REVOLUTE,
        parent=column,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, RING_Z_BASE + RING_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0,
            lower=-math.pi / 2.0, upper=math.pi / 2.0,
        ),
    )

    # ------------------------------------------------------ bridge gooseneck
    spout = model.part("bridge_spout")
    spout.visual(
        mesh_from_cadquery(_bridge_gooseneck_shape(), "bridge_tube"),
        material=gold,
        name="bridge_tube",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5,
            lower=-math.pi / 3.0, upper=math.pi / 3.0,
        ),
    )

    # ----------------------------------------------------------- spray head
    head = model.part("spray_head")
    head.visual(
        mesh_from_cadquery(_head_shape(), "head_body"),
        material=gold,
        name="head_body",
    )
    # Nozzle ring embeds 2 mm into head body bottom for connectivity
    head.visual(
        Cylinder(radius=NOZZLE_R, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.079)),
        material=black,
        name="nozzle_ring",
    )
    # Head is rigidly mounted at the spout drop tip
    # The spout frame is at (0,0,SWIVEL_Z) world; in spout-local, the drop
    # end is at (BRIDGE_REACH, 0, DROP_END).
    model.articulation(
        "head_mount",
        ArticulationType.FIXED,
        parent=spout,
        child=head,
        origin=Origin(xyz=(BRIDGE_REACH, 0.0, DROP_END)),
    )

    # --------------------------------------------------------------- lever
    lever = model.part("pin_lever")
    lever.visual(
        Cylinder(radius=0.0135, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="lever_collar",
    )
    lever.visual(
        Cylinder(radius=0.004, length=LEVER_PIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.010 + LEVER_PIN_LEN / 2.0)),
        material=gold,
        name="lever_pin",
    )
    lever.visual(
        Cylinder(radius=0.005, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.010 + LEVER_PIN_LEN + 0.0025)),
        material=gold,
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
            effort=5.0, velocity=2.0,
            lower=-math.pi / 4.0, upper=math.pi / 4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("pedestal")
    ring = object_model.get_part("temperature_ring")
    spout = object_model.get_part("bridge_spout")
    head = object_model.get_part("spray_head")
    lever = object_model.get_part("pin_lever")

    ring_joint = object_model.get_articulation("ring_rotate")
    swivel = object_model.get_articulation("spout_swivel")
    lever_pivot = object_model.get_articulation("lever_pivot")

    # ---- intentional overlaps
    ctx.allow_overlap(
        ring,
        column,
        elem_a="ring_band",
        elem_b="column_body",
        reason="Temperature ring wraps around the pedestal with a seated bore fit.",
    )
    ctx.allow_overlap(
        lever,
        column,
        elem_a="lever_collar",
        elem_b="valve_body",
        reason="Lever collar is captured on the valve body end (seated insertion).",
    )

    # ----- scale and grounding
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "column grounded at deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    base_aabb = ctx.part_element_world_aabb(column, elem="column_body")
    ctx.check(
        "column base diameter ~0.06 m",
        base_aabb is not None and 0.056 <= (base_aabb[1][0] - base_aabb[0][0]) <= 0.064,
        details=f"column element aabb={base_aabb}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "bridge gooseneck apex near 0.45 m",
        spout_aabb is not None and 0.42 <= spout_aabb[1][2] <= 0.48,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- squared bridge shape: bridge has significant horizontal extent
    # The bridge top should have a clear horizontal span (not just a curved arc)
    tube_aabb = ctx.part_element_world_aabb(spout, elem="bridge_tube")
    ctx.check(
        "bridge tube has significant horizontal X reach",
        tube_aabb is not None and (tube_aabb[1][0] - tube_aabb[0][0]) > 0.12,
        details=f"bridge_tube aabb={tube_aabb}",
    )
    # The bridge should have a flat-ish top: the Z extent of the top portion
    # should show a clear horizontal segment (apex z is close across the span)
    ctx.check(
        "bridge top is elevated above the column top",
        tube_aabb is not None and tube_aabb[1][2] > SWIVEL_Z + 0.08,
        details=f"bridge_tube aabb={tube_aabb}",
    )

    # ----- two mounting collars on pedestal
    collar_lo = ctx.part_element_world_aabb(column, elem="mount_collar_lower")
    collar_hi = ctx.part_element_world_aabb(column, elem="mount_collar_upper")
    ctx.check(
        "lower mounting collar exists on pedestal",
        collar_lo is not None and collar_lo[0][2] > 0.15,
        details=f"collar_lower aabb={collar_lo}",
    )
    ctx.check(
        "upper mounting collar exists above lower collar",
        collar_lo is not None and collar_hi is not None
        and collar_hi[0][2] > collar_lo[1][2] + 0.001,
        details=f"collar_lower={collar_lo}, collar_upper={collar_hi}",
    )

    # ----- tick marks as geometry (hot/cold)
    hot_tick = ctx.part_element_world_aabb(column, elem="hot_tick")
    cold_tick = ctx.part_element_world_aabb(column, elem="cold_tick")
    ctx.check(
        "hot tick mark exists on column (+X side)",
        hot_tick is not None and hot_tick[0][0] > 0.01,
        details=f"hot_tick aabb={hot_tick}",
    )
    ctx.check(
        "cold tick mark exists on column (-X side)",
        cold_tick is not None and cold_tick[1][0] < -0.01,
        details=f"cold_tick aabb={cold_tick}",
    )
    ctx.check(
        "tick marks are at distinct X positions (hot right, cold left)",
        hot_tick is not None and cold_tick is not None
        and hot_tick[0][0] > cold_tick[1][0] + 0.02,
        details=f"hot={hot_tick}, cold={cold_tick}",
    )

    # ----- temperature ring joint
    ctx.check(
        "temperature ring is revolute about Z axis, +/-90 deg",
        ring_joint.articulation_type == ArticulationType.REVOLUTE
        and ring_joint.motion_limits is not None
        and abs(ring_joint.motion_limits.lower + math.pi / 2.0) < 1e-6
        and abs(ring_joint.motion_limits.upper - math.pi / 2.0) < 1e-6
        and tuple(ring_joint.axis) == (0.0, 0.0, 1.0),
    )

    # Ring rotates: pointer sweeps from hot side to cold side
    rest_pointer = ctx.part_element_world_aabb(ring, elem="ring_pointer")
    with ctx.pose({ring_joint: math.pi / 2.0}):
        turned_pointer = ctx.part_element_world_aabb(ring, elem="ring_pointer")
    ctx.check(
        "ring rotation sweeps pointer from rest to orthogonal position",
        rest_pointer is not None and turned_pointer is not None
        and abs(
            0.5 * (turned_pointer[0][1] + turned_pointer[1][1])
            - 0.5 * (rest_pointer[0][1] + rest_pointer[1][1])
        ) > 0.02,
        details=f"rest={rest_pointer}, turned={turned_pointer}",
    )

    # ----- spout swivel joint
    ctx.check(
        "spout swivel is revolute +/-60 deg about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + math.pi / 3.0) < 1e-6
        and abs(swivel.motion_limits.upper - math.pi / 3.0) < 1e-6
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )

    # Swivel pose: spray head moves sideways
    rest_head = ctx.part_world_position(head)
    with ctx.pose({swivel: 1.0}):
        sw_head = ctx.part_world_position(head)
    ctx.check(
        "spout swivel carries spray head sideways about column axis",
        sw_head is not None and rest_head is not None
        and abs(sw_head[1]) > 0.08
        and abs(rest_head[1]) < 1e-6,
        details=f"rest={rest_head}, swiveled={sw_head}",
    )

    # ----- lever pivot joint
    ctx.check(
        "lever pivot is revolute +/-45 deg about valve horizontal axis",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lever_pivot.motion_limits is not None
        and abs(lever_pivot.motion_limits.lower + math.pi / 4.0) < 1e-6
        and abs(lever_pivot.motion_limits.upper - math.pi / 4.0) < 1e-6,
    )

    rest_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "pin lever is ~0.10 m long above the valve body",
        rest_lever is not None and 0.095 <= (rest_lever[1][2] - VALVE_Z) <= 0.130,
        details=f"lever aabb={rest_lever}",
    )
    with ctx.pose({lever_pivot: math.pi / 4.0}):
        tilted_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "lever pin sweeps in X when rotated about the valve axis",
        rest_lever is not None and tilted_lever is not None
        and tilted_lever[1][0] > rest_lever[1][0] + 0.05,
        details=f"rest={rest_lever}, tilted={tilted_lever}",
    )

    # ----- ring seated on column (proof for allow_overlap)
    ctx.expect_overlap(
        ring,
        column,
        axes="xy",
        elem_a="ring_band",
        elem_b="column_body",
        min_overlap=0.005,
        name="ring band wraps around the column in XY",
    )
    ctx.expect_within(
        column,
        ring,
        axes="xy",
        inner_elem="column_body",
        outer_elem="ring_band",
        margin=0.005,
        name="column passes through the ring bore",
    )

    # ----- lever collar captured on valve body
    ctx.expect_overlap(
        lever,
        column,
        axes="y",
        elem_a="lever_collar",
        elem_b="valve_body",
        min_overlap=0.002,
        name="lever collar captured on the valve body",
    )

    # ----- spray head hangs below the spout tip
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "spray head hangs below the bridge, pointing down",
        head_aabb is not None and head_aabb[0][2] > 0.15 and head_aabb[0][2] < 0.35,
        details=f"head aabb={head_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
