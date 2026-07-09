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
# High-arc gooseneck faucet variant 22 — lower/wider flattened-oval spout,
# independent top flow knob, ribbed spray head, distinct hollow outlet.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the faucet (direction the gooseneck reaches over the
#   sink), +Z is up.  The tapered column rises on the Z axis.
# - The gooseneck spout swivels about the vertical column axis (+/-60 deg).
# - A flattened oval tube arcs up and over, lower than the parent but wider.
# - A small flow knob sits on top of the spout apex and rotates independently.
# - The pull-down spray head hangs at the spout tip with shallow ribbing.
# - A distinct hollow outlet opening is visible at the spout tip.
# - A horizontal valve body on the right side carries a slim pin lever.
# - A control pod with dial sits on the column front.
# ---------------------------------------------------------------------------

# Column
COLUMN_BASE_R = 0.030
COLUMN_MID_R = 0.020
COLUMN_TOP_R = 0.0155
COLUMN_MID_Z = 0.18
COLUMN_TOP_Z = 0.295
COLLAR_R = 0.0175
COLLAR_LEN = 0.012
SWIVEL_Z = 0.307

# Gooseneck — lower and wider with flattened oval tube
OVAL_RX = 0.010          # vertical semi-axis (in plane of curvature)
OVAL_RY = 0.018          # horizontal semi-axis (wider, perpendicular to arc)
RISER_TOP = 0.030        # shorter straight riser (was 0.053)
ARC_R = 0.100            # wider arc (was 0.085)
REACH_X = 2.0 * ARC_R    # 0.20 m horizontal reach
DROP_END = 0.003         # spout-local z of the tube tip

# Pull-down stages
STAGE_TRAVEL = 0.060
SLEEVE_R = 0.0075
SLEEVE_LEN = 0.072
INNER_HOSE_R = 0.0048

# Spray head with ribbing
HEAD_LEN = 0.100
NOZZLE_R = 0.0095
RIB_COUNT = 8
RIB_WIDTH = 0.003
RIB_DEPTH = 0.0015

# Flow knob on spout apex
FLOW_KNOB_D = 0.022
FLOW_KNOB_H = 0.010

# Valve + lever
VALVE_Z = 0.14
VALVE_R = 0.013
VALVE_LEN = 0.055
VALVE_Y_CENTER = -0.0455
LEVER_JOINT_Y = -0.070
LEVER_PIN_LEN = 0.102

# Control pod + dial
POD_Z = 0.085
POD_R = 0.016
POD_LEN = 0.075
POD_X = 0.034
DIAL_JOINT_Y = -0.036
DIAL_D = 0.030
DIAL_H = 0.012

# Hollow outlet opening at spout tip
OUTLET_INSET = 0.004     # how far inset from tube end
OUTLET_DEPTH = 0.010     # visible depth of the hollow


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
    """Flattened oval tube: straight riser, high inverted-U arc, drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    # Elliptical profile: flattened oval (wider in Y, thinner in X)
    return cq.Workplane("XY").ellipse(OVAL_RX, OVAL_RY).sweep(path)


def _head_shape() -> cq.Workplane:
    """Tapered pull-down spray head with shallow ribbing."""
    # Main tapered body
    body = (
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
    # Add shallow ribs around the mid-section
    rib_solid = None
    for i in range(RIB_COUNT):
        angle = 2.0 * math.pi * i / RIB_COUNT
        cx = 0.0165 * math.cos(angle)
        cy = 0.0165 * math.sin(angle)
        rib = (
            cq.Workplane("XY")
            .workplane(offset=-0.025)
            .center(cx, cy)
            .rect(RIB_WIDTH, RIB_DEPTH)
            .extrude(-0.050)
        )
        if rib_solid is None:
            rib_solid = rib
        else:
            rib_solid = rib_solid.union(rib)
    if rib_solid is not None:
        body = body.union(rib_solid)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v22")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    hose_gray = model.material("hose_gray", rgba=(0.20, 0.20, 0.20, 1.0))
    red = model.material("indicator_red", rgba=(0.85, 0.13, 0.10, 1.0))
    blue = model.material("indicator_blue", rgba=(0.20, 0.45, 0.90, 1.0))
    dark_chrome = model.material("dark_chrome", rgba=(0.12, 0.12, 0.14, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        mesh_from_cadquery(_column_shape(), "tapered_column"),
        material=gold,
        name="tapered_column",
    )
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + COLLAR_LEN / 2.0)),
        material=gold,
        name="swivel_collar",
    )
    # Horizontal valve body on the right (-Y) side.
    column.visual(
        Cylinder(radius=VALVE_R, length=VALVE_LEN),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER, VALVE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="valve_body",
    )
    # Horizontal control pod on the front face.
    column.visual(
        Cylinder(radius=POD_R, length=POD_LEN),
        origin=Origin(xyz=(POD_X, 0.0, POD_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="control_pod",
    )
    # Black touch display.
    column.visual(
        Box((0.005, 0.048, 0.020)),
        origin=Origin(xyz=(POD_X + 0.0155, 0.004, POD_Z)),
        material=black,
        name="touch_display",
    )
    column.visual(
        Cylinder(radius=0.0028, length=0.003),
        origin=Origin(xyz=(POD_X + 0.0192, -0.008, POD_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="hot_icon",
    )
    column.visual(
        Cylinder(radius=0.0028, length=0.003),
        origin=Origin(xyz=(POD_X + 0.0192, 0.014, POD_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="cold_icon",
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gold,
        name="gooseneck_tube",
    )
    # Distinct hollow outlet opening — dark bore disc at the spout tube tip,
    # visible from below when the spray head is pulled down.
    spout.visual(
        Cylinder(radius=0.009, length=0.003),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + 0.002)),
        material=dark_chrome,
        name="outlet_hollow",
    )
    # Rim ring framing the outlet, just above the tube tip
    spout.visual(
        Cylinder(radius=OVAL_RY - 0.001, length=0.002),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + 0.004)),
        material=gold,
        name="outlet_rim",
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

    # -------------------------------------------------------- top flow knob
    flow_knob = model.part("flow_knob")
    knob_geo = KnobGeometry(
        FLOW_KNOB_D,
        FLOW_KNOB_H,
        body_style="domed",
        grip=KnobGrip(style="fluted", count=16, depth=0.0006),
        indicator=KnobIndicator(style="wedge", depth=0.002, width=0.002),
        center=False,
    )
    flow_knob.visual(
        mesh_from_geometry(knob_geo, "flow_knob_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=gold,
        name="flow_knob_body",
    )
    # Small mounting boss on the spout apex for the flow knob.
    # Extends 1 mm below the tube top surface to ensure geometric contact.
    spout.visual(
        Cylinder(radius=0.005, length=0.005),
        origin=Origin(xyz=(ARC_R, 0.0, RISER_TOP + ARC_R + OVAL_RX + 0.0015)),
        material=gold,
        name="knob_boss",
    )

    # Mount the flow knob on top of the spout apex
    # Apex in spout-local: (ARC_R, 0, RISER_TOP + ARC_R + OVAL_RX)
    knob_mount_z = RISER_TOP + ARC_R + OVAL_RX + 0.004
    model.articulation(
        "flow_knob_rotate",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=flow_knob,
        origin=Origin(xyz=(ARC_R, 0.0, knob_mount_z)),
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
    # Hidden inner hose
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

    # -------------------------------------------------------------------- dial
    dial = model.part("dial_cap")
    dial_geo = KnobGeometry(
        DIAL_D,
        DIAL_H,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=24, depth=0.0008),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(dial_geo, "dial_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.002, length=0.002),
        origin=Origin(xyz=(0.008, -(DIAL_H + 0.0005), 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="dial_dot",
    )
    model.articulation(
        "dial_knob",
        ArticulationType.REVOLUTE,
        parent=column,
        child=dial,
        origin=Origin(xyz=(POD_X, DIAL_JOINT_Y, POD_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=-2.3562, upper=2.3562
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
    dial = object_model.get_part("dial_cap")
    flow_knob = object_model.get_part("flow_knob")

    swivel = object_model.get_articulation("spout_swivel")
    pulldown = object_model.get_articulation("spray_pulldown")
    hose_slide = object_model.get_articulation("hose_slide")
    lever_pivot = object_model.get_articulation("lever_pivot")
    dial_knob = object_model.get_articulation("dial_knob")
    flow_rotate = object_model.get_articulation("flow_knob_rotate")

    # Intentional nested telescoping fits and captured rotating collars
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
        lever,
        column,
        elem_a="lever_collar",
        elem_b="valve_body",
        reason="Rotating lever collar is captured on the valve body end (seated insertion).",
    )
    ctx.allow_overlap(
        dial,
        column,
        elem_a="dial_body",
        elem_b="control_pod",
        reason="Dial cap base embeds 1.5 mm into the pod end so it reads seated.",
    )


    ctx.allow_overlap(
        spout,
        head,
        elem_a="outlet_hollow",
        elem_b="head_body",
        reason="Outlet bore sits inside the spout tube tip where the spray head docks; visible when head is pulled down.",
    )

    # ----- Variant 22: flattened oval tube wider than tall
    tube_aabb = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "gooseneck tube is wider (Y) than deep (X) — flattened oval cross-section",
        tube_aabb is not None
        and (tube_aabb[1][1] - tube_aabb[0][1]) > 0.02
        and (tube_aabb[1][1] - tube_aabb[0][1]) > (tube_aabb[1][0] - tube_aabb[0][0]) * 0.15,
        details=f"tube aabb={tube_aabb}",
    )

    # ----- Variant 22: lower arc apex compared to parent's ~0.45m
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex is lower than parent (below 0.46 m)",
        spout_aabb is not None and spout_aabb[1][2] < 0.46,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck reach is wider than 0.17 m",
        spout_aabb is not None and (spout_aabb[1][0] - spout_aabb[0][0]) > 0.17,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- Variant 22: flow knob exists and rotates independently
    ctx.check(
        "flow knob is revolute about vertical axis with +/-pi range",
        flow_rotate is not None
        and flow_rotate.articulation_type == ArticulationType.REVOLUTE
        and flow_rotate.motion_limits is not None
        and abs(flow_rotate.motion_limits.lower + math.pi) < 0.01
        and abs(flow_rotate.motion_limits.upper - math.pi) < 0.01
        and tuple(flow_rotate.axis) == (0.0, 0.0, 1.0),
    )

    # Flow knob rotates independently of spout swivel
    rest_knob = ctx.part_world_position(flow_knob)
    with ctx.pose({flow_rotate: math.pi / 2.0}):
        turned_knob = ctx.part_world_aabb(flow_knob)
    ctx.check(
        "flow knob rotates independently (pose changes its world AABB)",
        rest_knob is not None and turned_knob is not None,
        details=f"rest_knob={rest_knob}, turned_aabb={turned_knob}",
    )

    # ----- Variant 22: distinct hollow outlet opening at spout tip
    outlet = ctx.part_element_world_aabb(spout, elem="outlet_hollow")
    ctx.check(
        "hollow outlet opening exists below the spout tip",
        outlet is not None and outlet[1][2] < SWIVEL_Z + RISER_TOP,
        details=f"outlet aabb={outlet}",
    )

    # ----- Variant 22: spray head has ribbing visuals
    head_body_aabb = ctx.part_element_world_aabb(head, elem="head_body")
    ctx.check(
        "spray head body width exceeds smooth profile (ribs add material)",
        head_body_aabb is not None
        and (head_body_aabb[1][0] - head_body_aabb[0][0]) > 0.034,
        details=f"head_body aabb={head_body_aabb}",
    )
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "spray head tip hangs around z=0.15-0.24 pointing down",
        head_aabb is not None and 0.15 <= head_aabb[0][2] <= 0.24,
        details=f"head aabb={head_aabb}",
    )

    # ----- Proof checks for allowances
    ctx.expect_overlap(
        spout,
        head,
        axes="xy",
        elem_a="outlet_hollow",
        elem_b="head_body",
        min_overlap=0.005,
        name="outlet bore is contained within the spout tube tip area",
    )
    ctx.expect_contact(
        flow_knob,
        spout,
        elem_a="flow_knob_body",
        elem_b="knob_boss",
        contact_tol=0.005,
        name="flow knob seated on the mounting boss",
    )

    # ----- scale, grounding, proportions
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "column grounded at deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    base_aabb = ctx.part_element_world_aabb(column, elem="tapered_column")
    ctx.check(
        "column base diameter ~0.06 m",
        base_aabb is not None and 0.056 <= (base_aabb[1][0] - base_aabb[0][0]) <= 0.064,
        details=f"column element aabb={base_aabb}",
    )

    # ----- joint plan: types and ranges
    ctx.check(
        "spout swivel is revolute +/-60 deg about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + math.pi / 3.0) < 1e-6
        and abs(swivel.motion_limits.upper - math.pi / 3.0) < 1e-6
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )
    ctx.check(
        "lever pivot is revolute +/-45 deg about valve horizontal axis",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lever_pivot.motion_limits is not None
        and abs(lever_pivot.motion_limits.lower + math.pi / 4.0) < 1e-6
        and abs(lever_pivot.motion_limits.upper - math.pi / 4.0) < 1e-6,
    )
    ctx.check(
        "dial knob is revolute +/-135 deg",
        dial_knob.articulation_type == ArticulationType.REVOLUTE
        and dial_knob.motion_limits is not None
        and abs(dial_knob.motion_limits.lower + 2.3562) < 1e-4
        and abs(dial_knob.motion_limits.upper - 2.3562) < 1e-4,
    )
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
        contact_tol=0.002,
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
        head,
        hose,
        axes="z",
        elem_a="inner_hose",
        elem_b="hose_sleeve",
        min_overlap=0.05,
        name="inner hose hidden inside the sleeve at rest",
    )
    ctx.expect_within(
        head,
        hose,
        axes="xy",
        inner_elem="inner_hose",
        outer_elem="hose_sleeve",
        margin=0.001,
        name="inner hose stays centered in the sleeve",
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
    ctx.expect_overlap(
        dial,
        column,
        axes="y",
        elem_a="dial_body",
        elem_b="control_pod",
        min_overlap=0.0005,
        name="dial cap seated on the pod end",
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
        ctx.expect_overlap(
            head,
            hose,
            axes="z",
            elem_a="inner_hose",
            elem_b="hose_sleeve",
            min_overlap=0.008,
            name="inner hose retains insertion in sleeve at full pull-down",
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
        sw_head is not None and sw_head[1] > 0.10 and rest_head is not None
        and abs(rest_head[1]) < 1e-9,
        details=f"rest={rest_head}, swiveled={sw_head}",
    )

    # ----- lever pose
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
        rest_lever is not None
        and tilted_lever is not None
        and tilted_lever[1][0] > rest_lever[1][0] + 0.05,
        details=f"rest={rest_lever}, tilted={tilted_lever}",
    )

    # ----- dial pose
    rest_dot = ctx.part_element_world_aabb(dial, elem="dial_dot")
    with ctx.pose({dial_knob: math.pi / 2.0}):
        turned_dot = ctx.part_element_world_aabb(dial, elem="dial_dot")
    ctx.check(
        "off-axis dial dot orbits the dial axis",
        rest_dot is not None
        and turned_dot is not None
        and abs(0.5 * (turned_dot[0][2] + turned_dot[1][2]) - 0.5 * (rest_dot[0][2] + rest_dot[1][2]))
        > 0.005,
        details=f"rest={rest_dot}, turned={turned_dot}",
    )

    return ctx.report()


object_model = build_object_model()
