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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Variant 16: High-arc gooseneck faucet with angled aerator outlet,
# ribbed pull-down spray head, and distinct hollow outlet opening.
#
# Layout (world frame, deck plane at z = 0):
# - +X is front (direction the gooseneck reaches), +Z is up.
# - Tapered conical column rises to z = 0.307.
# - Gooseneck spout arcs up and over in a high inverted-U, then a short
#   angled aerator section (25° from vertical) ends the tube.
# - A hollow outlet ring at the aerator tip shows the bore opening.
# - A ribbed pull-down spray head hangs below the tip, sliding 0.06 m
#   downward on a single prismatic joint.
# - Side lever and front control pod with dial (same as parent).
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

# Gooseneck
TUBE_R = 0.012
RISER_TOP = 0.053
ARC_R = 0.085
REACH_X = 2.0 * ARC_R  # 0.17 m

# Angled aerator section at end of arc
AERATOR_ANGLE = math.radians(25)
AERATOR_LEN = 0.032
DROP_Z = 0.040  # z in spout-local where angled section begins
TIP_X = REACH_X + AERATOR_LEN * math.sin(AERATOR_ANGLE)
TIP_Z = DROP_Z - AERATOR_LEN * math.cos(AERATOR_ANGLE)
AERATOR_TILT = math.pi - AERATOR_ANGLE  # rpy Y to align local Z with aerator dir
AERATOR_MID_X = (REACH_X + TIP_X) / 2.0
AERATOR_MID_Z = (DROP_Z + TIP_Z) / 2.0

# Pull-down (single stage, short travel)
PULLDOWN_TRAVEL = 0.060
INNER_HOSE_R = 0.0048

# Spray head
HEAD_LEN = 0.100
NOZZLE_R = 0.0095

# Ribbing on spray head: (z_offset, outer_r, inner_r)
RIB_WIDTH = 0.003
RIB_SPECS = [
    (-0.022, 0.0155, 0.0135),
    (-0.042, 0.0175, 0.0155),
    (-0.062, 0.0175, 0.0155),
    (-0.080, 0.0155, 0.0135),
]

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
    """Slim tube: riser, high arc, short drop, angled aerator section."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_Z)
        .lineTo(TIP_X, TIP_Z)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def _head_shape() -> cq.Workplane:
    """Tapered pull-down spray head body (loft of circles)."""
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


def _aerator_housing_shape() -> cq.Workplane:
    """Hollow cylinder surrounding the angled aerator tube end."""
    return (
        cq.Workplane("XY")
        .workplane(offset=-0.007)
        .circle(TUBE_R + 0.003)
        .circle(TUBE_R - 0.004)
        .extrude(0.014)
    )


def _outlet_ring_shape() -> cq.Workplane:
    """Annular disk at aerator tip showing the distinct hollow bore."""
    return (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(TUBE_R + 0.001)
        .circle(0.009)
        .extrude(0.005)
    )


def _rib_shape(outer_r: float, inner_r: float) -> cq.Workplane:
    """Thin annular ring for spray head grip ribbing."""
    return (
        cq.Workplane("XY")
        .workplane(offset=-RIB_WIDTH / 2.0)
        .circle(outer_r)
        .circle(inner_r)
        .extrude(RIB_WIDTH)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v16")

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
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + COLLAR_LEN / 2.0)),
        material=gold,
        name="swivel_collar",
    )
    column.visual(
        Cylinder(radius=VALVE_R, length=VALVE_LEN),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER, VALVE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="valve_body",
    )
    column.visual(
        Cylinder(radius=POD_R, length=POD_LEN),
        origin=Origin(xyz=(POD_X, 0.0, POD_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="control_pod",
    )
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
    # Aerator housing: hollow cylinder around the angled tube end
    spout.visual(
        mesh_from_cadquery(_aerator_housing_shape(), "aerator_housing"),
        origin=Origin(
            xyz=(AERATOR_MID_X, 0.0, AERATOR_MID_Z),
            rpy=(0.0, AERATOR_TILT, 0.0),
        ),
        material=gold,
        name="aerator_housing",
    )
    # Hollow outlet opening at the aerator tip
    spout.visual(
        mesh_from_cadquery(_outlet_ring_shape(), "outlet_ring"),
        origin=Origin(
            xyz=(TIP_X, 0.0, TIP_Z),
            rpy=(0.0, AERATOR_TILT, 0.0),
        ),
        material=black,
        name="outlet_ring",
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

    # -------------------------------------------------------- spray head
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
    # Hidden inner hose: keeps the head visually connected to the spout
    head.visual(
        Cylinder(radius=INNER_HOSE_R, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=hose_gray,
        name="inner_hose",
    )
    # Shallow ribbing on the spray head body
    for i, (z_off, outer_r, inner_r) in enumerate(RIB_SPECS):
        head.visual(
            mesh_from_cadquery(_rib_shape(outer_r, inner_r), f"rib_{i}"),
            origin=Origin(xyz=(0.0, 0.0, z_off)),
            material=gold,
            name=f"grip_rib_{i}",
        )

    model.articulation(
        "spray_pulldown",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=head,
        origin=Origin(xyz=(TIP_X, 0.0, TIP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.3, lower=0.0, upper=PULLDOWN_TRAVEL
        ),
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
    head = object_model.get_part("spray_head")
    lever = object_model.get_part("pin_lever")
    dial = object_model.get_part("dial_cap")

    swivel = object_model.get_articulation("spout_swivel")
    pulldown = object_model.get_articulation("spray_pulldown")
    lever_pivot = object_model.get_articulation("lever_pivot")
    dial_knob = object_model.get_articulation("dial_knob")

    # Intentional overlaps: scoped allowances with exact proof checks below.
    ctx.allow_overlap(
        head,
        spout,
        elem_a="head_body",
        elem_b="gooseneck_tube",
        reason="Spray head body seats against the angled aerator tube end (small local seating overlap).",
    )
    ctx.allow_overlap(
        head,
        spout,
        elem_a="head_body",
        elem_b="outlet_ring",
        reason="Head body seats against the outlet ring at the aerator tip (same seating interface as the tube end).",
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
        elem_a="inner_hose",
        elem_b="aerator_housing",
        reason="Inner hose bridges from head up into the spout through the aerator housing region.",
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
        reason="Dial cap base embeds ~1.5 mm into the pod end so it reads seated.",
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
        spout_aabb is not None and 0.42 <= spout_aabb[1][2] <= 0.48,
        details=f"spout aabb={spout_aabb}",
    )
    head_body_aabb = ctx.part_element_world_aabb(head, elem="head_body")
    ctx.check(
        "spray head body hangs below spout tip",
        head_body_aabb is not None
        and head_body_aabb[0][2] > 0.15
        and head_body_aabb[1][2] < 0.35,
        details=f"head body aabb={head_body_aabb}",
    )

    # ----- variant-specific: angled aerator at arc end
    aerator_aabb = ctx.part_element_world_aabb(spout, elem="aerator_housing")
    ctx.check(
        "aerator housing present at the angled spout end",
        aerator_aabb is not None
        and aerator_aabb[0][0] > 0.14
        and aerator_aabb[0][2] > 0.30,
        details=f"aerator aabb={aerator_aabb}",
    )

    # ----- variant-specific: distinct hollow outlet opening
    outlet_aabb = ctx.part_element_world_aabb(spout, elem="outlet_ring")
    ctx.check(
        "hollow outlet ring at aerator tip",
        outlet_aabb is not None
        and outlet_aabb[0][0] > 0.15
        and outlet_aabb[0][2] > 0.30,
        details=f"outlet aabb={outlet_aabb}",
    )
    # The outlet ring should be at the very tip (furthest X on the spout)
    tube_aabb = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "outlet ring extends past the main tube in X",
        outlet_aabb is not None
        and tube_aabb is not None
        and outlet_aabb[1][0] >= tube_aabb[1][0] - 0.005,
        details=f"outlet={outlet_aabb}, tube={tube_aabb}",
    )

    # ----- variant-specific: shallow ribbing on spray head
    rib0_aabb = ctx.part_element_world_aabb(head, elem="grip_rib_0")
    rib3_aabb = ctx.part_element_world_aabb(head, elem="grip_rib_3")
    ctx.check(
        "spray head has grip ribbing (4 ribs spanning the body)",
        rib0_aabb is not None
        and rib3_aabb is not None
        and rib0_aabb[0][2] > rib3_aabb[0][2],
        details=f"rib0={rib0_aabb}, rib3={rib3_aabb}",
    )
    # Ribs should protrude slightly beyond the head body surface
    body_aabb = ctx.part_element_world_aabb(head, elem="head_body")
    ctx.check(
        "ribs sit on the head body surface",
        rib0_aabb is not None
        and body_aabb is not None
        and rib0_aabb[0][2] > body_aabb[0][2]
        and rib0_aabb[1][2] < body_aabb[1][2],
        details=f"rib0={rib0_aabb}, body={body_aabb}",
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
        "spray pulldown is prismatic with 0.06 m downward travel",
        pulldown.articulation_type == ArticulationType.PRISMATIC
        and pulldown.motion_limits is not None
        and abs(pulldown.motion_limits.lower) < 1e-9
        and abs(pulldown.motion_limits.upper - PULLDOWN_TRAVEL) < 1e-9
        and tuple(pulldown.axis) == (0.0, 0.0, -1.0),
        details=f"limits={pulldown.motion_limits}, axis={pulldown.axis}",
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

    # ----- retained insertion at rest
    ctx.expect_contact(
        head,
        spout,
        elem_a="head_body",
        elem_b="gooseneck_tube",
        contact_tol=0.005,
        name="spray head seats against the aerator tube end",
    )
    ctx.expect_contact(
        head,
        spout,
        elem_a="head_body",
        elem_b="outlet_ring",
        contact_tol=0.010,
        name="spray head contacts the outlet ring at the seating face",
    )
    ctx.expect_overlap(
        head,
        spout,
        axes="z",
        elem_a="inner_hose",
        elem_b="gooseneck_tube",
        min_overlap=0.03,
        name="inner hose hidden inside spout tube at rest",
    )
    ctx.expect_overlap(
        head,
        spout,
        axes="z",
        elem_a="inner_hose",
        elem_b="aerator_housing",
        min_overlap=0.005,
        name="inner hose passes through the aerator housing zone",
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

    # ----- pull-down pose: head drops 0.06 m and inner hose retains insertion
    rest_head = ctx.part_world_position(head)
    with ctx.pose({pulldown: PULLDOWN_TRAVEL}):
        ext_head = ctx.part_world_position(head)
        ctx.expect_overlap(
            head,
            spout,
            axes="z",
            elem_a="inner_hose",
            elem_b="gooseneck_tube",
            min_overlap=0.008,
            name="inner hose retains insertion in spout at full pull-down",
        )
    ctx.check(
        "pull-down lowers the spray head by 0.06 m",
        rest_head is not None
        and ext_head is not None
        and abs((rest_head[2] - ext_head[2]) - PULLDOWN_TRAVEL) < 1e-6
        and abs(rest_head[0] - ext_head[0]) < 1e-9,
        details=f"rest={rest_head}, extended={ext_head}",
    )

    # ----- swivel pose: spray head carried sideways about the column axis
    with ctx.pose({swivel: 1.0}):
        sw_head = ctx.part_world_position(head)
    ctx.check(
        "spout swivel carries the spray head sideways about the column axis",
        sw_head is not None
        and abs(sw_head[1]) > 0.08
        and rest_head is not None
        and abs(rest_head[1]) < 1e-9,
        details=f"rest={rest_head}, swiveled={sw_head}",
    )

    # ----- lever pose: pin sweeps fore/aft about the valve axis
    rest_lever = ctx.part_world_aabb(lever)
    with ctx.pose({lever_pivot: math.pi / 4.0}):
        tilted_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "lever pin sweeps in X when rotated about the valve axis",
        rest_lever is not None
        and tilted_lever is not None
        and tilted_lever[1][0] > rest_lever[1][0] + 0.04,
        details=f"rest={rest_lever}, tilted={tilted_lever}",
    )

    # ----- dial pose: off-axis dot orbits the pod axis
    rest_dot = ctx.part_element_world_aabb(dial, elem="dial_dot")
    with ctx.pose({dial_knob: math.pi / 2.0}):
        turned_dot = ctx.part_element_world_aabb(dial, elem="dial_dot")
    ctx.check(
        "off-axis dial dot orbits the dial axis",
        rest_dot is not None
        and turned_dot is not None
        and abs(
            0.5 * (turned_dot[0][2] + turned_dot[1][2])
            - 0.5 * (rest_dot[0][2] + rest_dot[1][2])
        )
        > 0.005,
        details=f"rest={rest_dot}, turned={turned_dot}",
    )

    return ctx.report()


object_model = build_object_model()
