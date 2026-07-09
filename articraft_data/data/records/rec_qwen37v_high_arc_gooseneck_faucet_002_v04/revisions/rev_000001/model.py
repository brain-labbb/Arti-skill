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
# Brushed-gold single-hole kitchen faucet, ~0.45 m tall.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the faucet (direction the gooseneck reaches over the
#   sink), +Z is up.  The tapered column rises on the Z axis to z = 0.307.
# - The gooseneck spout swivels about the vertical column axis (+/-60 deg).
# - The pull-down spray head hangs at the spout tip (x = 0.17).  Its 0.12 m
#   downward travel is realized as a two-stage telescoping slide: a hidden
#   hose sleeve extends 0.06 m out of the spout tube (mimic-coupled), and the
#   head extends a further 0.06 m out of the sleeve on its own hidden hose.
# - A horizontal valve body on the right side (-Y) at z = 0.14 carries the
#   slim vertical pin lever (revolute about the valve's Y axis, +/-45 deg).
# - A horizontal control pod on the front face carries a black touch display
#   with red/blue temperature icons and a gold dial cap on its left end
#   (revolute about the pod's Y axis, +/-135 deg).
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
HEAD_LEN = 0.090  # ribbed head body total loft height
NOZZLE_R = 0.0095

# Valve + lever (original right-side valve retained)
VALVE_Z = 0.14
VALVE_R = 0.013
VALVE_LEN = 0.055
VALVE_Y_CENTER = -0.0455

# Side-mounted mixer body (variant: distinct from valve body)
MIXER_Z = 0.19
MIXER_R = 0.016
MIXER_LEN = 0.050
MIXER_Y_CENTER = -0.043  # inner end embeds into column surface

# Side lever on mixer body
LEVER_JOINT_Y = MIXER_Y_CENTER - MIXER_LEN / 2.0  # outer end of mixer body
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
    """Tapered pull-down spray head with shallow circumferential ribbing."""
    return (
        cq.Workplane("XY")
        .circle(0.0125)           # top mount collar
        .workplane(offset=-0.020)
        .circle(0.0155)           # upper body swell
        .workplane(offset=-0.007)
        .circle(0.0162)           # rib 1 outer
        .workplane(offset=-0.003)
        .circle(0.0148)           # rib 1 groove
        .workplane(offset=-0.007)
        .circle(0.0165)           # rib 2 outer
        .workplane(offset=-0.003)
        .circle(0.0150)           # rib 2 groove
        .workplane(offset=-0.007)
        .circle(0.0165)           # rib 3 outer
        .workplane(offset=-0.003)
        .circle(0.0150)           # rib 3 groove
        .workplane(offset=-0.007)
        .circle(0.0162)           # rib 4 outer
        .workplane(offset=-0.003)
        .circle(0.0148)           # rib 4 groove
        .workplane(offset=-0.008)
        .circle(0.0140)           # lower taper start
        .workplane(offset=-0.012)
        .circle(0.0115)           # lower taper end
        .workplane(offset=-0.010)
        .circle(0.0105)           # nozzle tip
        .loft()
    )


def _hollow_outlet_shape() -> cq.Workplane:
    """Short annular bezel at the spout tip representing a distinct hollow outlet opening."""
    return (
        cq.Workplane("XY")
        .circle(TUBE_R + 0.001)
        .circle(TUBE_R - 0.003)
        .extrude(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brushed_gold_high_arc_gooseneck_faucet")

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
    # Horizontal valve body on the right (-Y) side, mid-column height.
    column.visual(
        Cylinder(radius=VALVE_R, length=VALVE_LEN),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER, VALVE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="valve_body",
    )
    # Side-mounted mixer body (variant: distinct body above the valve)
    column.visual(
        Cylinder(radius=MIXER_R, length=MIXER_LEN),
        origin=Origin(xyz=(0.0, MIXER_Y_CENTER, MIXER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="mixer_body",
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
    # Distinct hollow outlet opening at the spout tip (short annular bezel)
    spout.visual(
        mesh_from_cadquery(_hollow_outlet_shape(), "hollow_outlet"),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - 0.006)),
        material=gold,
        name="hollow_outlet",
    )
    # Dark interior visible through the hollow outlet bore
    spout.visual(
        Cylinder(radius=TUBE_R - 0.004, length=0.008),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - 0.004)),
        material=black,
        name="outlet_bore",
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
        origin=Origin(xyz=(0.0, LEVER_JOINT_Y, MIXER_Z)),
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
        elem_b="mixer_body",
        reason="Rotating lever collar is captured on the mixer body end (seated insertion).",
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
        elem_a="hollow_outlet",
        elem_b="head_body",
        reason="Hollow outlet bezel seats flush against the spray head top (shared surface contact).",
    )
    ctx.allow_overlap(
        spout,
        head,
        elem_a="outlet_bore",
        elem_b="head_body",
        reason="Dark outlet bore recesses into the spray head top to show the hollow water passage.",
    )
    ctx.allow_overlap(
        spout,
        head,
        elem_a="outlet_bore",
        elem_b="inner_hose",
        reason="Outlet bore and inner hose share the tube-tip passage where the hose enters the spout.",
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
        pulldown is not None
        and swivel.articulation_type == ArticulationType.REVOLUTE
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
        elem_b="mixer_body",
        min_overlap=0.002,
        name="lever collar captured on the mixer body",
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

    # ----- lever pose: pin sweeps fore/aft about the mixer axis
    rest_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "pin lever is ~0.10 m long above the mixer body",
        rest_lever is not None and 0.095 <= (rest_lever[1][2] - MIXER_Z) <= 0.130,
        details=f"lever aabb={rest_lever}",
    )
    with ctx.pose({lever_pivot: math.pi / 4.0}):
        tilted_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "lever pin sweeps in X when rotated about the mixer axis",
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

    # ----- control pod display details
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

    # ===== variant-specific checks =====

    # ----- side-mounted mixer body exists and is distinct from valve body
    mixer_aabb = ctx.part_element_world_aabb(column, elem="mixer_body")
    valve_aabb = ctx.part_element_world_aabb(column, elem="valve_body")
    ctx.check(
        "side-mounted mixer body exists on the column",
        mixer_aabb is not None,
        details=f"mixer_body aabb={mixer_aabb}",
    )
    ctx.check(
        "mixer body is distinct from valve body (separate Z positions)",
        mixer_aabb is not None
        and valve_aabb is not None
        and abs(0.5 * (mixer_aabb[0][2] + mixer_aabb[1][2]) - 0.5 * (valve_aabb[0][2] + valve_aabb[1][2])) > 0.02,
        details=f"mixer={mixer_aabb}, valve={valve_aabb}",
    )

    # ----- side lever is mounted on the mixer body (Y position matches mixer)
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "side lever is positioned at the mixer body Y location",
        lever_aabb is not None
        and mixer_aabb is not None
        and lever_aabb[0][1] < -0.04,
        details=f"lever={lever_aabb}, mixer={mixer_aabb}",
    )

    # ----- hollow outlet opening at the spout tip
    outlet_aabb = ctx.part_element_world_aabb(spout, elem="hollow_outlet")
    bore_aabb = ctx.part_element_world_aabb(spout, elem="outlet_bore")
    ctx.check(
        "distinct hollow outlet ring exists at the spout tip",
        outlet_aabb is not None,
        details=f"hollow_outlet aabb={outlet_aabb}",
    )
    ctx.check(
        "dark outlet bore is visible inside the hollow outlet",
        bore_aabb is not None
        and outlet_aabb is not None
        and bore_aabb[0][2] >= outlet_aabb[0][2] - 0.002
        and bore_aabb[0][2] <= outlet_aabb[1][2],
        details=f"bore={bore_aabb}, outlet={outlet_aabb}",
    )

    # ----- spray head has ribbing (head body is taller than plain version)
    head_body_aabb = ctx.part_element_world_aabb(head, elem="head_body")
    ctx.check(
        "spray head body has ribbed profile with sufficient height",
        head_body_aabb is not None
        and (head_body_aabb[1][2] - head_body_aabb[0][2]) > 0.080,
        details=f"head_body aabb={head_body_aabb}",
    )
    # Prove the outlet/head seating with contact and gap checks
    ctx.expect_contact(
        spout,
        head,
        elem_a="hollow_outlet",
        elem_b="head_body",
        contact_tol=0.003,
        name="hollow outlet bezel contacts the spray head top",
    )
    ctx.expect_gap(
        spout,
        head,
        axis="z",
        elem_a="outlet_bore",
        elem_b="head_body",
        max_penetration=0.010,
        name="outlet bore recesses into the head top within tolerance",
    )
    ctx.expect_overlap(
        spout,
        head,
        axes="z",
        elem_a="outlet_bore",
        elem_b="inner_hose",
        min_overlap=0.003,
        name="outlet bore and inner hose share the tube-tip passage",
    )

    # ----- at least one non-fixed joint exists (side lever revolute)
    ctx.check(
        "side lever has a non-fixed revolute joint",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lever_pivot.motion_limits is not None
        and lever_pivot.motion_limits.lower is not None
        and lever_pivot.motion_limits.upper is not None
        and lever_pivot.motion_limits.lower < lever_pivot.motion_limits.upper,
    )

    return ctx.report()


object_model = build_object_model()
