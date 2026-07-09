from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# High-arc gooseneck faucet variant (~0.49 m tall).
#
# Changes from parent:
# - Taller spout with tighter forward bend (smaller arc radius, taller riser)
# - Small top flow knob on the spout riser (revolute about vertical axis)
# - Visible cold/hot tick marks as raised geometry on the column
# - Two small mounting collars on the pedestal
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (gooseneck reaches over the sink), +Z is up.
# - Tapered column rises to z ~ 0.33.
# - Two mounting collars encircle the pedestal near the deck.
# - Gooseneck spout swivels about the vertical column axis (+/-60 deg).
# - Flow knob collar sits on the spout riser (revolute +/-180 deg).
# - Pull-down spray head hangs at the spout tip (prismatic 0.12 m travel).
# - Valve body on the right (-Y) carries the pin lever (revolute +/-45 deg).
# - Hot/cold tick marks are raised thin boxes on the column front face.
# ---------------------------------------------------------------------------

# Column
COLUMN_BASE_R = 0.030
COLUMN_MID_R = 0.020
COLUMN_TOP_R = 0.0155
COLUMN_MID_Z = 0.20
COLUMN_TOP_Z = 0.33
SWIVEL_Z = 0.342

# Mounting collars on pedestal
MOUNT_COLLAR_R = 0.034
MOUNT_COLLAR_H = 0.007
MOUNT_COLLAR_1_Z = 0.005
MOUNT_COLLAR_2_Z = 0.030

# Gooseneck (taller riser, tighter arc radius)
TUBE_R = 0.012
RISER_TOP = 0.10
ARC_R = 0.050
REACH_X = 2.0 * ARC_R  # 0.10 m horizontal reach
DROP_END = 0.005

# Pull-down stages
STAGE_TRAVEL = 0.060
SLEEVE_R = 0.0075
SLEEVE_LEN = 0.072
INNER_HOSE_R = 0.0048

# Spray head
HEAD_LEN = 0.100
NOZZLE_R = 0.0095

# Valve + lever
VALVE_Z = 0.15
VALVE_R = 0.013
VALVE_LEN = 0.055
VALVE_Y_CENTER = -0.0455
LEVER_JOINT_Y = -0.070
LEVER_PIN_LEN = 0.102

# Flow knob on spout riser (spout-local z)
FLOW_KNOB_LOCAL_Z = 0.045
FLOW_KNOB_D = 0.028
FLOW_KNOB_H = 0.016

# Tick marks on column front face
TICK_W = 0.003
TICK_H = 0.012
TICK_D = 0.003
TICK_Z = 0.285
# Column radius at TICK_Z ~ 0.017; embed tick inner face inside column
TICK_X = 0.016


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


def _gooseneck_shape() -> cq.Workplane:
    """Slim tube: taller riser, tighter inverted-U arc, short drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def _head_shape() -> cq.Workplane:
    """Tapered pull-down spray head pointing straight down."""
    return (
        cq.Workplane("XY")
        .circle(0.0125)
        .workplane(offset=-0.030)
        .circle(0.0155)
        .workplane(offset=-0.030)
        .circle(0.0165)
        .workplane(offset=-0.025)
        .circle(0.0145)
        .workplane(offset=-0.015)
        .circle(0.0105)
        .loft()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    hose_gray = model.material("hose_gray", rgba=(0.20, 0.20, 0.20, 1.0))
    red = model.material("indicator_red", rgba=(0.85, 0.13, 0.10, 1.0))
    blue = model.material("indicator_blue", rgba=(0.20, 0.45, 0.90, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        mesh_from_cadquery(_column_shape(), "tapered_column"),
        material=gold,
        name="tapered_column",
    )
    # Swivel collar at column top
    column.visual(
        Cylinder(radius=0.0175, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + 0.006)),
        material=gold,
        name="swivel_collar",
    )
    # Two mounting collars on the pedestal
    column.visual(
        Cylinder(radius=MOUNT_COLLAR_R, length=MOUNT_COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_COLLAR_1_Z + MOUNT_COLLAR_H / 2.0)),
        material=gold,
        name="mount_collar_lower",
    )
    column.visual(
        Cylinder(radius=MOUNT_COLLAR_R, length=MOUNT_COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_COLLAR_2_Z + MOUNT_COLLAR_H / 2.0)),
        material=gold,
        name="mount_collar_upper",
    )
    # Horizontal valve body on the right (-Y) side
    column.visual(
        Cylinder(radius=VALVE_R, length=VALVE_LEN),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER, VALVE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="valve_body",
    )

    # Hot/cold tick marks as raised thin boxes on the column front face.
    # Embedded 1mm into the column surface so they read as integral marks.
    # Hot tick (+Y side)
    column.visual(
        Box((TICK_D, TICK_W, TICK_H)),
        origin=Origin(xyz=(TICK_X, 0.010, TICK_Z)),
        material=red,
        name="hot_tick",
    )
    # Cold tick (-Y side)
    column.visual(
        Box((TICK_D, TICK_W, TICK_H)),
        origin=Origin(xyz=(TICK_X, -0.010, TICK_Z)),
        material=blue,
        name="cold_tick",
    )
    # Center tick mark (neutral position)
    column.visual(
        Box((TICK_D, TICK_W * 0.7, TICK_H * 0.6)),
        origin=Origin(xyz=(TICK_X, 0.0, TICK_Z)),
        material=black,
        name="center_tick",
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gold,
        name="gooseneck_tube",
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

    # ------------------------------------------- flow knob on the spout riser
    flow_knob = model.part("flow_knob")
    knob_geo = KnobGeometry(
        FLOW_KNOB_D,
        FLOW_KNOB_H,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=20, depth=0.0008),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=True,
    )
    flow_knob.visual(
        mesh_from_geometry(knob_geo, "flow_knob_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=gold,
        name="flow_knob_body",
    )
    model.articulation(
        "flow_knob_rotate",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=flow_knob,
        # Origin is in spout-local frame (spout frame already at SWIVEL_Z)
        origin=Origin(xyz=(0.0, 0.0, FLOW_KNOB_LOCAL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=-math.pi, upper=math.pi
        ),
    )

    # ------------------------------------------------- pull-down stage 1: hose
    hose = model.part("hose_stem")
    hose.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_LEN / 2.0)),
        material=hose_gray,
        name="hose_sleeve",
    )

    # ------------------------------------------------- pull-down stage 2: head
    head = model.part("spray_head")
    head.visual(
        mesh_from_cadquery(_head_shape(), "head_body"),
        material=gold,
        name="head_body",
    )
    head.visual(
        Cylinder(radius=NOZZLE_R, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -(HEAD_LEN + 0.003))),
        material=black,
        name="nozzle_ring",
    )
    head.visual(
        Cylinder(radius=0.004, length=0.005),
        origin=Origin(xyz=(0.0155, 0.0, -0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="spray_button",
    )
    head.visual(
        Cylinder(radius=INNER_HOSE_R, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=hose_gray,
        name="inner_hose",
    )

    model.articulation(
        "spray_pulldown",
        ArticulationType.PRISMATIC,
        parent=hose,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.3, lower=0.0, upper=STAGE_TRAVEL),
    )
    model.articulation(
        "hose_slide",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=hose,
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.3, lower=0.0, upper=STAGE_TRAVEL),
        mimic=Mimic("spray_pulldown", multiplier=1.0),
    )

    # ------------------------------------------------------------------- lever
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
            effort=5.0, velocity=2.0, lower=-math.pi / 4.0, upper=math.pi / 4.0
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    hose = object_model.get_part("hose_stem")
    head = object_model.get_part("spray_head")
    lever = object_model.get_part("pin_lever")
    flow_knob = object_model.get_part("flow_knob")

    swivel = object_model.get_articulation("spout_swivel")
    pulldown = object_model.get_articulation("spray_pulldown")
    hose_slide = object_model.get_articulation("hose_slide")
    lever_pivot = object_model.get_articulation("lever_pivot")
    flow_knob_joint = object_model.get_articulation("flow_knob_rotate")

    # Intentional overlaps: scoped allowances paired with exact checks below.
    ctx.allow_overlap(
        hose,
        spout,
        elem_a="hose_sleeve",
        elem_b="gooseneck_tube",
        reason="Pull-down hose sleeve intentionally nests inside the solid spout tube proxy.",
    )
    ctx.allow_overlap(
        head,
        hose,
        elem_a="inner_hose",
        elem_b="hose_sleeve",
        reason="Inner hose intentionally telescopes inside the hose sleeve proxy.",
    )
    ctx.allow_overlap(
        head,
        spout,
        elem_a="inner_hose",
        elem_b="gooseneck_tube",
        reason="Hidden inner hose sits inside the solid spout tube at the rest pose.",
    )
    ctx.allow_overlap(
        head,
        spout,
        elem_a="head_body",
        elem_b="gooseneck_tube",
        reason="Spray head body seats into the spout tube tip (flush seating contact).",
    )
    ctx.allow_overlap(
        lever,
        column,
        elem_a="lever_collar",
        elem_b="valve_body",
        reason="Rotating lever collar is captured on the valve body end (seated insertion).",
    )
    ctx.allow_overlap(
        flow_knob,
        spout,
        elem_a="flow_knob_body",
        elem_b="gooseneck_tube",
        reason="Flow knob collar wraps around the spout riser tube (rotating capture ring).",
    )

    # ----- variant-specific: taller spout with tighter bend
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex above 0.46 m (taller than parent)",
        spout_aabb is not None and spout_aabb[1][2] >= 0.46,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck horizontal reach below 0.13 m (tighter forward bend)",
        spout_aabb is not None and (spout_aabb[1][0] - spout_aabb[0][0]) <= 0.13,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- variant-specific: two mounting collars on pedestal
    collar_lower = ctx.part_element_world_aabb(column, elem="mount_collar_lower")
    collar_upper = ctx.part_element_world_aabb(column, elem="mount_collar_upper")
    ctx.check(
        "lower mounting collar exists near deck",
        collar_lower is not None and collar_lower[0][2] < 0.020,
        details=f"collar_lower={collar_lower}",
    )
    ctx.check(
        "upper mounting collar exists above lower collar",
        collar_upper is not None
        and collar_lower is not None
        and collar_upper[0][2] > collar_lower[1][2],
        details=f"collar_upper={collar_upper}, collar_lower={collar_lower}",
    )
    ctx.check(
        "mounting collars wider than column base",
        collar_lower is not None
        and (collar_lower[1][0] - collar_lower[0][0]) > 0.060,
        details=f"collar_lower={collar_lower}",
    )

    # ----- variant-specific: hot/cold tick marks as geometry
    hot_tick = ctx.part_element_world_aabb(column, elem="hot_tick")
    cold_tick = ctx.part_element_world_aabb(column, elem="cold_tick")
    center_tick = ctx.part_element_world_aabb(column, elem="center_tick")
    ctx.check(
        "hot tick mark exists on column",
        hot_tick is not None,
        details=f"hot_tick={hot_tick}",
    )
    ctx.check(
        "cold tick mark exists on column",
        cold_tick is not None,
        details=f"cold_tick={cold_tick}",
    )
    ctx.check(
        "hot and cold ticks are separated in Y",
        hot_tick is not None
        and cold_tick is not None
        and hot_tick[0][1] > cold_tick[1][1],
        details=f"hot={hot_tick}, cold={cold_tick}",
    )
    ctx.check(
        "center tick mark sits between hot and cold ticks",
        center_tick is not None
        and hot_tick is not None
        and cold_tick is not None
        and cold_tick[0][1] <= center_tick[0][1]
        and center_tick[1][1] <= hot_tick[1][1],
        details=f"center={center_tick}, hot={hot_tick}, cold={cold_tick}",
    )

    # ----- variant-specific: top flow knob rotates independently
    ctx.check(
        "flow knob is revolute about vertical axis with +/-180 deg range",
        flow_knob_joint.articulation_type == ArticulationType.REVOLUTE
        and flow_knob_joint.motion_limits is not None
        and abs(flow_knob_joint.motion_limits.lower + math.pi) < 1e-4
        and abs(flow_knob_joint.motion_limits.upper - math.pi) < 1e-4
        and tuple(flow_knob_joint.axis) == (0.0, 0.0, 1.0),
    )
    # Knob sits on the spout riser (above the swivel point)
    knob_aabb = ctx.part_world_aabb(flow_knob)
    ctx.check(
        "flow knob sits on the spout riser above the swivel point",
        knob_aabb is not None and knob_aabb[0][2] > SWIVEL_Z,
        details=f"knob_aabb={knob_aabb}, swivel_z={SWIVEL_Z}",
    )
    # Prove the knob rotates independently from the spout
    rest_knob_pos = ctx.part_world_position(flow_knob)
    with ctx.pose({flow_knob_joint: math.pi / 2.0}):
        turned_knob_pos = ctx.part_world_position(flow_knob)
    ctx.check(
        "flow knob rotation changes the knob world pose",
        rest_knob_pos is not None and turned_knob_pos is not None,
        details=f"rest={rest_knob_pos}, turned={turned_knob_pos}",
    )

    # ----- scale and grounding
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "column grounded at deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )

    # ----- joint plan: spout swivel
    ctx.check(
        "spout swivel is revolute +/-60 deg about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + math.pi / 3.0) < 1e-6
        and abs(swivel.motion_limits.upper - math.pi / 3.0) < 1e-6
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )

    # ----- lever pivot
    ctx.check(
        "lever pivot is revolute +/-45 deg about valve horizontal axis",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lever_pivot.motion_limits is not None
        and abs(lever_pivot.motion_limits.lower + math.pi / 4.0) < 1e-6
        and abs(lever_pivot.motion_limits.upper - math.pi / 4.0) < 1e-6,
    )

    # ----- pull-down travel
    total_travel = (pulldown.motion_limits.upper or 0.0) + (hose_slide.motion_limits.upper or 0.0)
    ctx.check(
        "pull-down spray head total prismatic travel is 0.12 m downward",
        pulldown.articulation_type == ArticulationType.PRISMATIC
        and hose_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(total_travel - 0.12) < 1e-9
        and hose_slide.mimic is not None
        and hose_slide.mimic.joint == "spray_pulldown",
        details=f"total_travel={total_travel}, mimic={hose_slide.mimic}",
    )

    # ----- seating and retained insertion at rest
    ctx.expect_contact(
        head,
        spout,
        elem_a="head_body",
        elem_b="gooseneck_tube",
        contact_tol=0.006,
        name="spray head seats flush against the spout tip",
    )
    ctx.expect_overlap(
        hose,
        spout,
        axes="z",
        elem_a="hose_sleeve",
        elem_b="gooseneck_tube",
        min_overlap=0.05,
        name="hose sleeve hidden inside the spout tube at rest",
    )
    ctx.expect_overlap(
        lever,
        column,
        axes="y",
        elem_a="lever_collar",
        elem_b="valve_body",
        min_overlap=0.002,
        name="lever collar captured on the valve body",
    )
    # Flow knob wraps around the riser tube
    ctx.expect_overlap(
        flow_knob,
        spout,
        axes="z",
        elem_a="flow_knob_body",
        elem_b="gooseneck_tube",
        min_overlap=0.010,
        name="flow knob collar overlaps the riser tube in Z",
    )
    ctx.expect_within(
        flow_knob,
        spout,
        axes="xy",
        inner_elem="flow_knob_body",
        outer_elem="gooseneck_tube",
        margin=0.005,
        name="flow knob is centered on the riser tube axis",
    )

    # ----- pull-down pose
    rest_head = ctx.part_world_position(head)
    with ctx.pose({pulldown: STAGE_TRAVEL}):
        ext_head = ctx.part_world_position(head)
        ctx.expect_overlap(
            hose,
            spout,
            axes="z",
            elem_a="hose_sleeve",
            elem_b="gooseneck_tube",
            min_overlap=0.008,
            name="sleeve retains insertion in spout at full pull-down",
        )
    ctx.check(
        "pull-down lowers the spray head by 0.12 m along the spout-tip axis",
        rest_head is not None
        and ext_head is not None
        and abs((rest_head[2] - ext_head[2]) - 0.12) < 1e-6
        and abs(rest_head[0] - ext_head[0]) < 1e-9,
        details=f"rest={rest_head}, extended={ext_head}",
    )

    # ----- swivel pose
    with ctx.pose({swivel: 1.0}):
        sw_head = ctx.part_world_position(head)
    ctx.check(
        "spout swivel carries the spray head sideways about the column axis",
        sw_head is not None and sw_head[1] > 0.05 and rest_head is not None
        and abs(rest_head[1]) < 1e-9,
        details=f"rest={rest_head}, swiveled={sw_head}",
    )

    return ctx.report()


object_model = build_object_model()
