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
# High-arc gooseneck faucet variant 02 — lower/wider flattened-oval spout,
# short prismatic hose, ribbed spray head, seam at the swivel collar.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the faucet, +Z is up.
# - Tapered column rises on Z axis to z = 0.307.
# - Gooseneck spout has a flattened oval tube (18 mm x 30 mm cross-section),
#   a lower arc (apex ~0.44 m), and a semicircular reach (~0.16 m).
# - Swivel collar has a thin dark seam ring marking the rotation joint.
# - Spray head is a single-stage prismatic slide (0.06 m travel) with
#   three shallow ribs on the body.
# - Right-side pin lever (revolute, +/-45 deg) and front control pod with
#   dial (revolute, +/-135 deg) are preserved from the parent.
# ---------------------------------------------------------------------------

# Column
COLUMN_BASE_R = 0.030
COLUMN_MID_R = 0.020
COLUMN_TOP_R = 0.0155
COLUMN_MID_Z = 0.18
COLUMN_TOP_Z = 0.295
COLLAR_R = 0.0175
COLLAR_LEN = 0.012
SWIVEL_Z = 0.307  # top of the swivel collar = base of the gooseneck riser

# Gooseneck (spout-local, frame at the top of the collar)
# Lower arc: flattened oval tube cross-section (wider side-to-side)
TUBE_RX = 0.009   # semi-axis along sweep plane (front-back, flatter)
TUBE_RY = 0.015   # semi-axis perpendicular (side-to-side, wider)
RISER_TOP = 0.035  # straight riser before the arc (lower than parent's 0.053)
ARC_R = 0.080      # arc radius (slightly lower than parent's 0.085)
REACH_X = 2.0 * ARC_R  # semicircular arc for reliable sweep tessellation
DROP_END = 0.003   # spout-local z of the open tube tip

# Pull-down (single-stage, short travel)
HEAD_TRAVEL = 0.060
INNER_HOSE_R = 0.005
INNER_HOSE_LEN = 0.070

# Spray head (head-local, frame at the tube tip seam, body extends -Z)
HEAD_LEN = 0.095

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
    """Flattened oval tube: straight riser, lower inverted-U arc, short drop."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    # Flattened oval cross-section (wider side-to-side, flatter front-back)
    return cq.Workplane("XY").ellipse(TUBE_RX, TUBE_RY).sweep(path)


def _head_shape() -> cq.Workplane:
    """Ribbed spray head: loft with alternating ridge/valley radii."""
    return (
        cq.Workplane("XY")
        .circle(0.0130)            # top collar (hose connection)
        .workplane(offset=-0.012)
        .circle(0.0160)            # ridge 1
        .workplane(offset=-0.008)
        .circle(0.0140)            # valley 1
        .workplane(offset=-0.012)
        .circle(0.0160)            # ridge 2
        .workplane(offset=-0.008)
        .circle(0.0140)            # valley 2
        .workplane(offset=-0.012)
        .circle(0.0160)            # ridge 3
        .workplane(offset=-0.008)
        .circle(0.0140)            # valley 3
        .workplane(offset=-0.018)
        .circle(0.0120)            # taper to nozzle end
        .workplane(offset=-0.017)
        .circle(0.0105)            # nozzle face
        .loft()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v02")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    dark_gold_seam = model.material("dark_gold_seam", rgba=(0.50, 0.38, 0.15, 1.0))
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
    # Thin seam ring at the swivel collar (marks the rotation interface)
    column.visual(
        Cylinder(radius=COLLAR_R + 0.0015, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - 0.0006)),
        material=dark_gold_seam,
        name="swivel_seam",
    )
    # Horizontal valve body on the right (-Y) side, mid-column height.
    column.visual(
        Cylinder(radius=VALVE_R, length=VALVE_LEN),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER, VALVE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="valve_body",
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
    # Red (hot) and blue (cold) temperature icons proud of the display glass.
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

    # ------------------------------------------------------- spray head (single-stage)
    head = model.part("spray_head")
    head.visual(
        mesh_from_cadquery(_head_shape(), "head_body"),
        material=gold,
        name="head_body",
    )
    head.visual(
        Cylinder(radius=0.0095, length=0.008),
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
    # Hidden inner hose: keeps the head engaged with the spout at full pull.
    head.visual(
        Cylinder(radius=INNER_HOSE_R, length=INNER_HOSE_LEN),
        origin=Origin(xyz=(0.0, 0.0, INNER_HOSE_LEN / 2.0 - 0.002)),
        material=hose_gray,
        name="inner_hose",
    )

    model.articulation(
        "spray_pulldown",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=head,
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.3, lower=0.0, upper=HEAD_TRAVEL
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

    # Intentional overlap allowances (scoped, paired with exact checks below)
    ctx.allow_overlap(
        head,
        spout,
        elem_a="inner_hose",
        elem_b="gooseneck_tube",
        reason="Inner hose nests inside the solid spout tube proxy at the rest pose.",
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

    # ----- variant geometry checks -----

    # Column grounded and base diameter
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "column grounded at deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )

    # Spout apex is lower than parent (~0.44 m vs parent's ~0.46 m)
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex is lower (~0.40-0.45 m, lower than parent ~0.46 m)",
        spout_aabb is not None and 0.40 <= spout_aabb[1][2] <= 0.45,
        details=f"spout aabb={spout_aabb}",
    )

    # Flattened oval tube: Y-extent (side-to-side) >= 0.028 m (2*TUBE_RY=0.030)
    tube_aabb = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "flattened oval tube is wider side-to-side (Y extent >= 0.028 m)",
        tube_aabb is not None
        and (tube_aabb[1][1] - tube_aabb[0][1]) >= 0.028,
        details=f"tube aabb={tube_aabb}",
    )

    # Ribbed spray head: the head body should be wider at ridges than a smooth cone
    head_body_aabb = ctx.part_element_world_aabb(head, elem="head_body")
    ctx.check(
        "spray head has ribbed profile (max diameter >= 0.030 m)",
        head_body_aabb is not None
        and (head_body_aabb[1][0] - head_body_aabb[0][0]) >= 0.030,
        details=f"head body aabb={head_body_aabb}",
    )

    # Swivel seam ring present on the column
    seam_aabb = ctx.part_element_world_aabb(column, elem="swivel_seam")
    collar_aabb = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "swivel seam ring sits at the collar interface",
        seam_aabb is not None
        and collar_aabb is not None
        and abs(0.5 * (seam_aabb[0][2] + seam_aabb[1][2]) - 0.5 * (collar_aabb[0][2] + collar_aabb[1][2])) < 0.008,
        details=f"seam={seam_aabb}, collar={collar_aabb}",
    )

    # ----- joint plan: types and ranges -----

    # Spout swivel: revolute +/-60 deg about vertical
    ctx.check(
        "spout swivel is revolute +/-60 deg about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + math.pi / 3.0) < 1e-6
        and abs(swivel.motion_limits.upper - math.pi / 3.0) < 1e-6
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )

    # Spray pulldown: single-stage prismatic, 0.06 m travel
    ctx.check(
        "spray pulldown is prismatic with 0.06 m travel (short hose joint)",
        pulldown.articulation_type == ArticulationType.PRISMATIC
        and pulldown.motion_limits is not None
        and abs(pulldown.motion_limits.lower) < 1e-9
        and abs(pulldown.motion_limits.upper - HEAD_TRAVEL) < 1e-9
        and tuple(pulldown.axis) == (0.0, 0.0, -1.0),
        details=f"limits={pulldown.motion_limits}",
    )

    # Lever pivot: revolute +/-45 deg
    ctx.check(
        "lever pivot is revolute +/-45 deg about valve horizontal axis",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lever_pivot.motion_limits is not None
        and abs(lever_pivot.motion_limits.lower + math.pi / 4.0) < 1e-6
        and abs(lever_pivot.motion_limits.upper - math.pi / 4.0) < 1e-6,
    )

    # Dial knob: revolute +/-135 deg
    ctx.check(
        "dial knob is revolute +/-135 deg",
        dial_knob.articulation_type == ArticulationType.REVOLUTE
        and dial_knob.motion_limits is not None
        and abs(dial_knob.motion_limits.lower + 2.3562) < 1e-4
        and abs(dial_knob.motion_limits.upper - 2.3562) < 1e-4,
    )

    # ----- seating and retained insertion at rest -----
    ctx.expect_contact(
        head,
        spout,
        elem_a="head_body",
        elem_b="gooseneck_tube",
        contact_tol=0.003,
        name="spray head seats near the spout tip at rest",
    )
    ctx.expect_overlap(
        head,
        spout,
        axes="z",
        elem_a="inner_hose",
        elem_b="gooseneck_tube",
        min_overlap=0.02,
        name="inner hose hidden inside the spout tube at rest",
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

    # ----- pull-down pose: head drops 0.06 m and hose retains insertion -----
    rest_head = ctx.part_world_position(head)
    with ctx.pose({pulldown: HEAD_TRAVEL}):
        ext_head = ctx.part_world_position(head)
        ctx.expect_overlap(
            head,
            spout,
            axes="z",
            elem_a="inner_hose",
            elem_b="gooseneck_tube",
            min_overlap=0.005,
            name="inner hose retains insertion in spout at full pull-down",
        )
    ctx.check(
        "pull-down lowers the spray head by 0.06 m along the spout-tip axis",
        rest_head is not None
        and ext_head is not None
        and abs((rest_head[2] - ext_head[2]) - HEAD_TRAVEL) < 1e-5
        and abs(rest_head[0] - ext_head[0]) < 1e-9,
        details=f"rest={rest_head}, extended={ext_head}",
    )

    # ----- swivel pose: spray head carried sideways about the column axis -----
    with ctx.pose({swivel: 1.0}):
        sw_head = ctx.part_world_position(head)
    ctx.check(
        "spout swivel carries the spray head sideways about the column axis",
        sw_head is not None and sw_head[1] > 0.08 and rest_head is not None
        and abs(rest_head[1]) < 1e-9,
        details=f"rest={rest_head}, swiveled={sw_head}",
    )

    # ----- lever pose: pin sweeps fore/aft about the valve axis -----
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

    # ----- dial pose: off-axis dot orbits the pod axis -----
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

    # ----- control pod display details -----
    disp = ctx.part_element_world_aabb(column, elem="touch_display")
    hot = ctx.part_element_world_aabb(column, elem="hot_icon")
    cold = ctx.part_element_world_aabb(column, elem="cold_icon")
    pod = ctx.part_element_world_aabb(column, elem="control_pod")
    ctx.check(
        "touch display sits proud of the pod front surface",
        disp is not None and pod is not None and disp[1][0] > pod[1][0],
        details=f"display={disp}, pod={pod}",
    )
    ctx.check(
        "red and blue temperature icons sit proud of the display",
        disp is not None
        and hot is not None
        and cold is not None
        and hot[1][0] > disp[1][0]
        and cold[1][0] > disp[1][0],
        details=f"display={disp}, hot={hot}, cold={cold}",
    )

    return ctx.report()


object_model = build_object_model()
