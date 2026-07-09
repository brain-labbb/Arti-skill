from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# High-arc gooseneck faucet variant: stepped cylindrical pedestal with broad
# escutcheon, single side lever, and cold/hot tick marks as geometry.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the faucet (gooseneck reach direction), +Z is up.
# - The stepped pedestal has a broad escutcheon (90 mm dia) at the deck,
#   a transition step (60 mm dia), then the main column (44 mm dia) rising
#   to z = 0.295.  A swivel collar sits on top at z = 0.307.
# - The gooseneck spout swivels about the vertical column axis (+/-60 deg).
# - The pull-down spray head hangs at the spout tip with 0.12 m travel.
# - A horizontal valve body on the right (-Y) side carries the pin lever
#   (revolute about the valve's Y axis, +/-45 deg).
# - Cold and hot tick marks are raised geometric features on opposite sides
#   of the column near the lever pivot height.
# ---------------------------------------------------------------------------

# Stepped pedestal
ESCUTCHEON_R = 0.045       # 90 mm diameter broad escutcheon
ESCUTCHEON_H = 0.005
STEP_R = 0.030             # 60 mm diameter transition step
STEP_H = 0.012
COL_R = 0.022              # 44 mm diameter main column
COLUMN_TOP_Z = 0.295
COLLAR_R = 0.0175
COLLAR_LEN = 0.012
SWIVEL_Z = 0.307

# Gooseneck (spout-local coordinates, frame at the top of the collar)
TUBE_R = 0.012
RISER_TOP = 0.053
ARC_R = 0.085
REACH_X = 2.0 * ARC_R     # 0.17 m horizontal reach
DROP_END = 0.003

# Pull-down stages
STAGE_TRAVEL = 0.060
SLEEVE_R = 0.0075
SLEEVE_LEN = 0.072
INNER_HOSE_R = 0.0048

# Spray head
HEAD_LEN = 0.100
NOZZLE_R = 0.0095

# Valve + lever
VALVE_Z = 0.14
VALVE_R = 0.013
VALVE_LEN = 0.055
VALVE_Y_CENTER = -0.0455
LEVER_JOINT_Y = -0.070
LEVER_PIN_LEN = 0.102

# Tick marks (raised geometric indicators on column surface)
TICK_W = 0.003             # X thickness (protrusion from column)
TICK_D = 0.006             # Y depth
TICK_H = 0.018             # Z height
TICK_Z_CENTER = VALVE_Z + 0.025
TICK_EMBED = 0.0008        # small embed into column for connectivity


def _pedestal_shape() -> cq.Workplane:
    """Stepped cylindrical pedestal: broad escutcheon → step → column."""
    col_h = COLUMN_TOP_Z - ESCUTCHEON_H - STEP_H
    return (
        cq.Workplane("XY")
        .circle(ESCUTCHEON_R)
        .extrude(ESCUTCHEON_H)
        .faces(">Z").workplane()
        .circle(STEP_R)
        .extrude(STEP_H)
        .faces(">Z").workplane()
        .circle(COL_R)
        .extrude(col_h)
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
    """Tapered pull-down spray head pointing straight down (loft)."""
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
    model = ArticulatedObject(name="stepped_pedestal_gooseneck_faucet")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    hose_gray = model.material("hose_gray", rgba=(0.20, 0.20, 0.20, 1.0))
    red = model.material("indicator_red", rgba=(0.85, 0.13, 0.10, 1.0))
    blue = model.material("indicator_blue", rgba=(0.20, 0.45, 0.90, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        mesh_from_cadquery(_pedestal_shape(), "stepped_pedestal"),
        material=gold,
        name="stepped_pedestal",
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
    # Hot tick mark (red) — raised geometric mark on the +X side of column.
    column.visual(
        Box((TICK_W, TICK_D, TICK_H)),
        origin=Origin(xyz=(COL_R + TICK_W / 2.0 - TICK_EMBED, 0.0, TICK_Z_CENTER)),
        material=red,
        name="hot_tick",
    )
    # Cold tick mark (blue) — raised geometric mark on the -X side of column.
    column.visual(
        Box((TICK_W, TICK_D, TICK_H)),
        origin=Origin(xyz=(-(COL_R + TICK_W / 2.0 - TICK_EMBED), 0.0, TICK_Z_CENTER)),
        material=blue,
        name="cold_tick",
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

    swivel = object_model.get_articulation("spout_swivel")
    pulldown = object_model.get_articulation("spray_pulldown")
    hose_slide = object_model.get_articulation("hose_slide")
    lever_pivot = object_model.get_articulation("lever_pivot")

    # Intentional nested telescoping fits and captured rotating collar.
    ctx.allow_overlap(
        hose, spout,
        elem_a="hose_sleeve", elem_b="gooseneck_tube",
        reason="Pull-down hose sleeve intentionally nests inside the solid spout tube proxy.",
    )
    ctx.allow_overlap(
        head, hose,
        elem_a="inner_hose", elem_b="hose_sleeve",
        reason="Inner hose intentionally telescopes inside the hose sleeve proxy.",
    )
    ctx.allow_overlap(
        head, spout,
        elem_a="inner_hose", elem_b="gooseneck_tube",
        reason="Hidden inner hose sits inside the solid spout tube at the rest pose.",
    )
    ctx.allow_overlap(
        lever, column,
        elem_a="lever_collar", elem_b="valve_body",
        reason="Rotating lever collar is captured on the valve body end (seated insertion).",
    )

    # ----- scale, grounding, proportions
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "pedestal grounded at deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )

    # Broad escutcheon check: pedestal widest extent should be ~90 mm diameter.
    ped_aabb = ctx.part_element_world_aabb(column, elem="stepped_pedestal")
    ctx.check(
        "broad escutcheon ~90 mm diameter at base",
        ped_aabb is not None and 0.085 <= (ped_aabb[1][0] - ped_aabb[0][0]) <= 0.095,
        details=f"pedestal aabb={ped_aabb}",
    )

    # Stepped pedestal: column section should be narrower than escutcheon.
    ctx.check(
        "pedestal has visible steps (column narrower than escutcheon)",
        ped_aabb is not None
        and (ped_aabb[1][0] - ped_aabb[0][0]) > 0.080,
        details=f"pedestal aabb={ped_aabb}",
    )

    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.45 m (tall arcing silhouette)",
        spout_aabb is not None and 0.43 <= spout_aabb[1][2] <= 0.48,
        details=f"spout aabb={spout_aabb}",
    )
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "spray head tip hangs around z=0.20 pointing down",
        head_aabb is not None and 0.18 <= head_aabb[0][2] <= 0.23,
        details=f"head aabb={head_aabb}",
    )

    # ----- tick marks: visible geometric cold/hot indicators
    hot_aabb = ctx.part_element_world_aabb(column, elem="hot_tick")
    cold_aabb = ctx.part_element_world_aabb(column, elem="cold_tick")
    ctx.check(
        "hot tick mark exists on +X side of column",
        hot_aabb is not None and hot_aabb[0][0] > 0.01,
        details=f"hot_tick aabb={hot_aabb}",
    )
    ctx.check(
        "cold tick mark exists on -X side of column",
        cold_aabb is not None and cold_aabb[1][0] < -0.01,
        details=f"cold_tick aabb={cold_aabb}",
    )
    ctx.check(
        "tick marks are symmetric about the column center X",
        hot_aabb is not None and cold_aabb is not None
        and abs(abs(hot_aabb[0][0] + hot_aabb[1][0]) / 2.0
                - abs(cold_aabb[0][0] + cold_aabb[1][0]) / 2.0) < 0.005,
        details=f"hot_center_x={(hot_aabb[0][0] + hot_aabb[1][0]) / 2.0}, "
                f"cold_center_x={(cold_aabb[0][0] + cold_aabb[1][0]) / 2.0}",
    )
    ctx.check(
        "tick marks are near lever pivot height",
        hot_aabb is not None and cold_aabb is not None
        and abs((hot_aabb[0][2] + hot_aabb[1][2]) / 2.0 - VALVE_Z) < 0.04
        and abs((cold_aabb[0][2] + cold_aabb[1][2]) / 2.0 - VALVE_Z) < 0.04,
        details=f"hot_z={(hot_aabb[0][2] + hot_aabb[1][2]) / 2.0}, "
                f"cold_z={(cold_aabb[0][2] + cold_aabb[1][2]) / 2.0}, valve_z={VALVE_Z}",
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
        "side lever is revolute +/-45 deg about valve horizontal axis",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lever_pivot.motion_limits is not None
        and abs(lever_pivot.motion_limits.lower + math.pi / 4.0) < 1e-6
        and abs(lever_pivot.motion_limits.upper - math.pi / 4.0) < 1e-6,
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

    # At least one non-fixed joint (lever is the primary side control).
    all_joints = [swivel, pulldown, hose_slide, lever_pivot]
    ctx.check(
        "model has at least one non-fixed revolute or prismatic joint",
        any(
            j.articulation_type in (ArticulationType.REVOLUTE, ArticulationType.PRISMATIC)
            and j.motion_limits is not None
            and j.motion_limits.lower is not None
            and j.motion_limits.upper is not None
            and j.motion_limits.lower != j.motion_limits.upper
            for j in all_joints
        ),
    )

    # ----- seating and retained insertion at rest
    ctx.expect_contact(
        head, spout,
        elem_a="head_body", elem_b="gooseneck_tube",
        contact_tol=0.002,
        name="spray head seats flush against the spout tip",
    )
    ctx.expect_overlap(
        hose, spout,
        axes="z",
        elem_a="hose_sleeve", elem_b="gooseneck_tube",
        min_overlap=0.05,
        name="hose sleeve hidden inside the spout tube at rest",
    )
    ctx.expect_overlap(
        head, hose,
        axes="z",
        elem_a="inner_hose", elem_b="hose_sleeve",
        min_overlap=0.05,
        name="inner hose hidden inside the sleeve at rest",
    )
    ctx.expect_within(
        head, hose,
        axes="xy",
        inner_elem="inner_hose", outer_elem="hose_sleeve",
        margin=0.001,
        name="inner hose stays centered in the sleeve",
    )
    ctx.expect_overlap(
        lever, column,
        axes="y",
        elem_a="lever_collar", elem_b="valve_body",
        min_overlap=0.002,
        name="lever collar captured on the valve body",
    )

    # ----- pull-down pose: head drops 0.12 m
    rest_head = ctx.part_world_position(head)
    with ctx.pose({pulldown: STAGE_TRAVEL}):
        ext_head = ctx.part_world_position(head)
        ctx.expect_overlap(
            hose, spout,
            axes="z",
            elem_a="hose_sleeve", elem_b="gooseneck_tube",
            min_overlap=0.008,
            name="sleeve retains insertion in spout at full pull-down",
        )
        ctx.expect_overlap(
            head, hose,
            axes="z",
            elem_a="inner_hose", elem_b="hose_sleeve",
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

    return ctx.report()


object_model = build_object_model()
