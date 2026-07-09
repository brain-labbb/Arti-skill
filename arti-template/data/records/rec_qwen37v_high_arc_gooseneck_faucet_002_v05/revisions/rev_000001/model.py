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
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Variant 05: High-arc gooseneck faucet with stepped pedestal base.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the faucet (direction the gooseneck reaches over the
#   sink), +Z is up.
# - A broad escutcheon plate sits on the deck, then a stepped cylindrical
#   pedestal rises in two tiers before the tapered column continues upward.
# - Two mounting collars ring the lower pedestal.
# - The gooseneck spout swivels about the vertical column axis (+/-60 deg).
# - The pull-down spray head hangs at the spout tip. Its 0.12 m downward
#   travel is realized as a two-stage telescoping slide (hose + head).
# - A horizontal valve body on the right side (-Y) carries the slim vertical
#   pin lever (revolute about the valve's Y axis, +/-45 deg).
# - Hot/cold tick marks (raised geometry bars) flank the valve body.
# - A horizontal control pod on the front face carries a black touch display
#   with a gold dial cap on its left end (revolute, +/-135 deg).
# ---------------------------------------------------------------------------

# Pedestal + escutcheon
ESCUTCHEON_R = 0.045
ESCUTCHEON_H = 0.005
PEDESTAL_LOWER_R = 0.034
PEDESTAL_LOWER_H = 0.022
PEDESTAL_UPPER_R = 0.028
PEDESTAL_UPPER_H = 0.018
PEDESTAL_TOP_Z = ESCUTCHEON_H + PEDESTAL_LOWER_H + PEDESTAL_UPPER_H  # 0.045

# Mounting collars on pedestal (both on the wider lower tier where they grip)
MOUNT_COLLAR_R = 0.036
MOUNT_COLLAR_TUBE = 0.004
MOUNT_COLLAR_THICKNESS = MOUNT_COLLAR_TUBE * 1.5  # 0.006
MOUNT_COLLAR_Z1 = ESCUTCHEON_H + 0.004  # 0.009
MOUNT_COLLAR_Z2 = ESCUTCHEON_H + PEDESTAL_LOWER_H - MOUNT_COLLAR_THICKNESS - 0.004  # sits near top of lower tier

# Column (tapers from pedestal top upward)
COLUMN_BASE_R = 0.026
COLUMN_MID_R = 0.020
COLUMN_TOP_R = 0.0155
COLUMN_MID_Z = 0.18
COLUMN_TOP_Z = 0.295
COLLAR_R = 0.0175
COLLAR_LEN = 0.012
SWIVEL_Z = 0.307  # top of the swivel collar = base of the gooseneck riser

# Gooseneck (spout-local coordinates, frame at the top of the collar)
TUBE_R = 0.012
RISER_TOP = 0.053  # straight riser before the arc
ARC_R = 0.085
REACH_X = 2.0 * ARC_R  # 0.17 m horizontal reach of the arch
DROP_END = 0.003  # spout-local z of the open tube tip

# Pull-down stages
STAGE_TRAVEL = 0.060  # per stage; total head travel = 0.12 m
SLEEVE_R = 0.0075
SLEEVE_LEN = 0.072
INNER_HOSE_R = 0.0048

# Spray head (head-local, frame at the tube tip seam, body extends -Z)
HEAD_LEN = 0.100
NOZZLE_R = 0.0095

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
DIAL_JOINT_Y = -0.036  # dial base embeds 1.5 mm into the pod's left end
DIAL_D = 0.030
DIAL_H = 0.012


def _column_shape() -> cq.Workplane:
    """Tapered conical column rising from the pedestal top."""
    return (
        cq.Workplane("XY")
        .workplane(offset=PEDESTAL_TOP_Z)
        .circle(COLUMN_BASE_R)
        .workplane(offset=COLUMN_MID_Z - PEDESTAL_TOP_Z)
        .circle(COLUMN_MID_R)
        .workplane(offset=COLUMN_TOP_Z - COLUMN_MID_Z)
        .circle(COLUMN_TOP_R)
        .loft()
    )


def _pedestal_shape() -> cq.Workplane:
    """Stepped cylindrical pedestal: lower wider tier + upper narrower tier."""
    return (
        cq.Workplane("XY")
        .workplane(offset=ESCUTCHEON_H)
        .circle(PEDESTAL_LOWER_R)
        .extrude(PEDESTAL_LOWER_H)
        .faces(">Z")
        .workplane()
        .circle(PEDESTAL_UPPER_R)
        .extrude(PEDESTAL_UPPER_H)
    )


def _escutcheon_shape() -> cq.Workplane:
    """Broad escutcheon plate at deck level."""
    return (
        cq.Workplane("XY")
        .circle(ESCUTCHEON_R)
        .extrude(ESCUTCHEON_H)
    )


def _mount_collar_shape() -> cq.Workplane:
    """Thin annular mounting collar ring (washer shape)."""
    outer_r = MOUNT_COLLAR_R + MOUNT_COLLAR_TUBE
    inner_r = MOUNT_COLLAR_R - MOUNT_COLLAR_TUBE
    thickness = MOUNT_COLLAR_TUBE * 1.5
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(thickness)
    )


def _gooseneck_shape() -> cq.Workplane:
    """Slim tube: straight riser, high inverted-U arc, short drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def _head_shape() -> cq.Workplane:
    """Tapered pull-down spray head pointing straight down (loft of circles)."""
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
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v05")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    hose_gray = model.material("hose_gray", rgba=(0.20, 0.20, 0.20, 1.0))
    red = model.material("indicator_red", rgba=(0.85, 0.13, 0.10, 1.0))
    blue = model.material("indicator_blue", rgba=(0.20, 0.45, 0.90, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    # Broad escutcheon plate at deck level
    column.visual(
        mesh_from_cadquery(_escutcheon_shape(), "escutcheon"),
        material=gold,
        name="escutcheon",
    )
    # Stepped cylindrical pedestal
    column.visual(
        mesh_from_cadquery(_pedestal_shape(), "pedestal"),
        material=gold,
        name="pedestal",
    )
    # Two mounting collars on the pedestal
    column.visual(
        mesh_from_cadquery(_mount_collar_shape(), "mount_collar_lower"),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_COLLAR_Z1)),
        material=gold,
        name="mount_collar_lower",
    )
    column.visual(
        mesh_from_cadquery(_mount_collar_shape(), "mount_collar_upper"),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_COLLAR_Z2)),
        material=gold,
        name="mount_collar_upper",
    )
    # Tapered column above the pedestal
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
    # Horizontal valve body on the right (-Y) side, mid-column height.
    column.visual(
        Cylinder(radius=VALVE_R, length=VALVE_LEN),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER, VALVE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="valve_body",
    )
    # Hot and cold tick marks: small raised bars flanking the valve body
    # (geometry indicators, not text). Each tick is a thin box proud of the
    # column surface on the -Y side near the valve.
    tick_x = 0.0
    tick_y = -0.030  # on the column surface near the valve root
    tick_w = 0.003   # tick width (X)
    tick_h = 0.012   # tick height (Z)
    tick_d = 0.003   # tick depth (Y, proud of surface)
    column.visual(
        Box((tick_w, tick_d, tick_h)),
        origin=Origin(xyz=(tick_x, tick_y - 0.014, VALVE_Z + 0.001)),
        material=red,
        name="hot_tick",
    )
    column.visual(
        Box((tick_w, tick_d, tick_h)),
        origin=Origin(xyz=(tick_x, tick_y - 0.014, VALVE_Z - 0.014)),
        material=blue,
        name="cold_tick",
    )
    # Horizontal control pod on the front face of the column.
    column.visual(
        Cylinder(radius=POD_R, length=POD_LEN),
        origin=Origin(xyz=(POD_X, 0.0, POD_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="control_pod",
    )
    # Black touch display, slightly proud of the pod front surface.
    column.visual(
        Box((0.005, 0.048, 0.020)),
        origin=Origin(xyz=(POD_X + 0.0155, 0.004, POD_Z)),
        material=black,
        name="touch_display",
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

    # ------------------------------------------------- pull-down stage 1: hose
    # Hidden hose sleeve nested inside the spout drop leg; it extends 0.06 m
    # out of the tube tip, mimic-coupled to the head pull.
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
    # Hidden inner hose: keeps the head engaged with the sleeve at full pull.
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
    # Knob axis is +Z; rpy=(pi/2,0,0) points it along -Y (outward, left pod end).
    dial.visual(
        mesh_from_geometry(dial_geo, "dial_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="dial_body",
    )
    # Off-axis red dot on the dial face (proves rotation about the pod axis).
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

    swivel = object_model.get_articulation("spout_swivel")
    pulldown = object_model.get_articulation("spray_pulldown")
    hose_slide = object_model.get_articulation("hose_slide")
    lever_pivot = object_model.get_articulation("lever_pivot")
    dial_knob = object_model.get_articulation("dial_knob")

    # Intentional nested telescoping fits (pull-down hose proxy) and captured
    # rotating collars: scoped allowances, each paired with exact checks below.
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

    # ----- variant 05: stepped pedestal with broad escutcheon
    esc_aabb = ctx.part_element_world_aabb(column, elem="escutcheon")
    ctx.check(
        "escutcheon plate grounded at deck plane",
        esc_aabb is not None and abs(esc_aabb[0][2]) <= 0.002,
        details=f"escutcheon aabb={esc_aabb}",
    )
    ctx.check(
        "escutcheon is broad (~0.09 m diameter)",
        esc_aabb is not None
        and 0.080 <= (esc_aabb[1][0] - esc_aabb[0][0]) <= 0.100,
        details=f"escutcheon aabb={esc_aabb}",
    )
    ped_aabb = ctx.part_element_world_aabb(column, elem="pedestal")
    ctx.check(
        "stepped pedestal rises above the escutcheon",
        ped_aabb is not None
        and ped_aabb[0][2] >= esc_aabb[1][2] - 0.001
        and (ped_aabb[1][2] - ped_aabb[0][2]) > 0.030,
        details=f"pedestal aabb={ped_aabb}",
    )
    # Lower pedestal tier is wider than upper tier (stepped profile)
    ped_dx = ped_aabb[1][0] - ped_aabb[0][0] if ped_aabb else 0.0
    col_base_aabb = ctx.part_element_world_aabb(column, elem="tapered_column")
    ctx.check(
        "pedestal is wider than the tapered column base (stepped profile)",
        ped_aabb is not None
        and col_base_aabb is not None
        and ped_dx > (col_base_aabb[1][0] - col_base_aabb[0][0]) + 0.005,
        details=f"pedestal_dx={ped_dx}, col_base_dx={col_base_aabb[1][0] - col_base_aabb[0][0] if col_base_aabb else None}",
    )

    # ----- variant 05: two mounting collars on the pedestal
    collar1_aabb = ctx.part_element_world_aabb(column, elem="mount_collar_lower")
    collar2_aabb = ctx.part_element_world_aabb(column, elem="mount_collar_upper")
    ctx.check(
        "lower mounting collar exists on the pedestal",
        collar1_aabb is not None,
        details=f"collar1 aabb={collar1_aabb}",
    )
    ctx.check(
        "upper mounting collar exists above the lower collar",
        collar2_aabb is not None
        and collar1_aabb is not None
        and collar2_aabb[0][2] > collar1_aabb[0][2] + 0.005,
        details=f"collar1={collar1_aabb}, collar2={collar2_aabb}",
    )

    # ----- variant 05: hot/cold tick marks as geometry
    hot_tick = ctx.part_element_world_aabb(column, elem="hot_tick")
    cold_tick = ctx.part_element_world_aabb(column, elem="cold_tick")
    ctx.check(
        "hot tick mark exists as raised geometry near the valve",
        hot_tick is not None,
        details=f"hot_tick aabb={hot_tick}",
    )
    ctx.check(
        "cold tick mark exists as raised geometry below the hot tick",
        cold_tick is not None
        and hot_tick is not None
        and cold_tick[1][2] < hot_tick[0][2],
        details=f"hot_tick={hot_tick}, cold_tick={cold_tick}",
    )

    # ----- scale, grounding, proportions
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "column grounded at deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.45 m",
        spout_aabb is not None and 0.43 <= spout_aabb[1][2] <= 0.48,
        details=f"spout aabb={spout_aabb}",
    )
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "spray head tip hangs around z=0.20 pointing down",
        head_aabb is not None and 0.18 <= head_aabb[0][2] <= 0.23,
        details=f"head aabb={head_aabb}",
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

    # ----- pull-down pose: head drops 0.12 m and both stages stay engaged
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

    # ----- swivel pose: spray head carried sideways about the column axis
    with ctx.pose({swivel: 1.0}):
        sw_head = ctx.part_world_position(head)
    ctx.check(
        "spout swivel carries the spray head sideways about the column axis",
        sw_head is not None and sw_head[1] > 0.10 and rest_head is not None
        and abs(rest_head[1]) < 1e-9,
        details=f"rest={rest_head}, swiveled={sw_head}",
    )

    # ----- lever pose: pin sweeps fore/aft about the valve axis
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

    # ----- dial pose: off-axis dot orbits the pod axis (continuous-rotation proof)
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

    # ----- control pod display
    disp = ctx.part_element_world_aabb(column, elem="touch_display")
    pod = ctx.part_element_world_aabb(column, elem="control_pod")
    ctx.check(
        "touch display sits proud of the pod front surface",
        disp is not None and pod is not None and disp[1][0] > pod[1][0],
        details=f"display={disp}, pod={pod}",
    )

    return ctx.report()


object_model = build_object_model()
