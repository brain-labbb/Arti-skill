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
# Brushed-gold high-arc gooseneck kitchen faucet variant, ~0.45 m tall.
#
# Variant 06: forked from the pull-down model with these structural changes:
# - Outlet shaped as a short angled aerator at the end of the gooseneck arc
# - Flip-down outlet aerator pivots at the nozzle (revolute, 0 to ~65 deg)
# - Shallow circumferential ribbing on the aerator body
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the faucet (direction the gooseneck reaches over the
#   sink), +Z is up.  The tapered column rises on the Z axis to z = 0.307.
# - The gooseneck spout swivels about the vertical column axis (+/-60 deg).
# - A short angled aerator hangs at the spout tip (x = 0.17).  It tilts about
#   15 degrees forward from vertical at rest and can flip further down by up
#   to ~1.1 rad via a revolute pivot at the nozzle connection.
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

# Aerator (aerator-local, frame at the pivot, body extends -Z at rest)
AERATOR_LEN = 0.048
AERATOR_TOP_R = 0.013
AERATOR_BOT_R = 0.010
AERATOR_REST_TILT = 0.26  # ~15 degrees forward lean from vertical
AERATOR_FLIP_UPPER = 1.10  # ~63 degrees additional flip
N_RIBS = 6
RIB_DEPTH = 0.0008
RIB_WIDTH = 0.002

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


def _aerator_body_shape() -> cq.Workplane:
    """Short tapered aerator body with shallow circumferential ribbing.

    The body extends from z=0 (top, pivot connection) down to z=-AERATOR_LEN.
    Circumferential grooves are cut into the outer surface to create ribs.
    """
    # Main tapered body via loft
    body = (
        cq.Workplane("XY")
        .circle(AERATOR_TOP_R)
        .workplane(offset=-AERATOR_LEN)
        .circle(AERATOR_BOT_R)
        .loft()
    )

    # Cut circumferential grooves to create ribbed texture
    margin = 0.006  # smooth collar at each end
    if N_RIBS > 1:
        spacing = (AERATOR_LEN - 2.0 * margin) / (N_RIBS - 1)
    else:
        spacing = 0.0

    for i in range(N_RIBS):
        z = -(margin + i * spacing)
        frac = abs(z) / AERATOR_LEN
        r = AERATOR_TOP_R + (AERATOR_BOT_R - AERATOR_TOP_R) * frac
        # Annular cutter: removes material from outer surface to form groove
        cutter = (
            cq.Workplane("XY")
            .workplane(offset=z - RIB_WIDTH / 2.0)
            .circle(r + 0.002)
            .circle(r - RIB_DEPTH)
            .extrude(RIB_WIDTH)
        )
        body = body.cut(cutter)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brushed_gold_higharc_gooseneck_faucet")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    chrome = model.material("chrome_ring", rgba=(0.82, 0.82, 0.84, 1.0))
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

    # --------------------------------------------------------- flip-down aerator
    aerator = model.part("outlet_aerator")
    # Ribbed aerator body extends along -Z in local frame.
    # Offset body 5 mm below pivot so the tilted top clears the tube tip;
    # the collar bridges the gap between tube and body.
    BODY_DROP = 0.005
    aerator.visual(
        mesh_from_cadquery(_aerator_body_shape(), "aerator_body"),
        origin=Origin(xyz=(0.0, 0.0, -BODY_DROP)),
        material=gold,
        name="aerator_body",
    )
    # Connection collar at the top (pivot interface with spout tip)
    aerator.visual(
        Cylinder(radius=AERATOR_TOP_R + 0.002, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=gold,
        name="aerator_collar",
    )
    # Aerator face ring at the bottom (chrome ring suggesting mesh aerator)
    aerator.visual(
        Cylinder(radius=AERATOR_BOT_R + 0.0015, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -(BODY_DROP + AERATOR_LEN + 0.002))),
        material=chrome,
        name="aerator_face",
    )
    # Small black nozzle disk inside the aerator face
    aerator.visual(
        Cylinder(radius=AERATOR_BOT_R - 0.002, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -(BODY_DROP + AERATOR_LEN + 0.003))),
        material=black,
        name="nozzle_disk",
    )

    # Flip-down pivot: aerator swings about Y axis at the spout tip.
    # Joint origin sits at the spout tube tip in spout-local coords.
    # The rest-pose rpy tilts the aerator ~15 deg forward toward +X.
    # Axis (0,-1,0) makes positive q swing the free end further toward +X
    # (flip down from the angled rest position).
    model.articulation(
        "aerator_flip",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(
            xyz=(REACH_X, 0.0, DROP_END),
            rpy=(0.0, -AERATOR_REST_TILT, 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=AERATOR_FLIP_UPPER
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
    aerator = object_model.get_part("outlet_aerator")
    lever = object_model.get_part("pin_lever")
    dial = object_model.get_part("dial_cap")

    swivel = object_model.get_articulation("spout_swivel")
    aerator_flip = object_model.get_articulation("aerator_flip")
    lever_pivot = object_model.get_articulation("lever_pivot")
    dial_knob = object_model.get_articulation("dial_knob")

    # Intentional seated overlaps: aerator collar overlaps the spout tube tip
    # at the pivot (seated insertion), lever collar on valve, dial on pod.
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="aerator_collar",
        elem_b="gooseneck_tube",
        reason="Aerator collar seats onto the spout tube tip at the pivot connection.",
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

    # ----- aerator geometry: short angled outlet at end of arc
    aer_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator hangs below the spout tip (z < 0.35)",
        aer_aabb is not None and aer_aabb[0][2] < 0.35,
        details=f"aerator aabb={aer_aabb}",
    )
    aer_body_aabb = ctx.part_element_world_aabb(aerator, elem="aerator_body")
    ctx.check(
        "aerator body is short (~0.04-0.06 m long)",
        aer_body_aabb is not None
        and 0.035 <= (aer_body_aabb[1][2] - aer_body_aabb[0][2]) <= 0.065,
        details=f"aerator body aabb={aer_body_aabb}",
    )
    # Aerator should be forward of the column (positive X, near the spout reach)
    ctx.check(
        "aerator positioned at the spout reach (x > 0.10)",
        aer_aabb is not None and aer_aabb[0][0] > 0.10,
        details=f"aerator aabb={aer_aabb}",
    )

    # Aerator face ring and nozzle disk present
    face_aabb = ctx.part_element_world_aabb(aerator, elem="aerator_face")
    nozzle_aabb = ctx.part_element_world_aabb(aerator, elem="nozzle_disk")
    ctx.check(
        "aerator face ring at the nozzle end",
        face_aabb is not None and nozzle_aabb is not None
        and face_aabb[0][2] < aer_body_aabb[0][2] if aer_body_aabb else False,
        details=f"face={face_aabb}, nozzle={nozzle_aabb}",
    )

    # ----- aerator flip joint: type, axis, limits
    ctx.check(
        "aerator_flip is revolute with 0 to ~1.1 rad range",
        aerator_flip is not None
        and aerator_flip.articulation_type == ArticulationType.REVOLUTE
        and aerator_flip.motion_limits is not None
        and abs(aerator_flip.motion_limits.lower) < 1e-6
        and 0.9 <= aerator_flip.motion_limits.upper <= 1.3
        and tuple(aerator_flip.axis) == (0.0, -1.0, 0.0),
        details=f"flip={aerator_flip.motion_limits if aerator_flip else None}",
    )

    # ----- aerator flip pose: positive q swings the free end further forward
    # The flip-down aerator tilts from mostly-vertical (rest) to mostly-forward
    # (flipped), so the nozzle end moves forward (+X) and rises as the body
    # swings from pointing down to pointing forward.
    rest_aer = ctx.part_world_aabb(aerator)
    rest_face = ctx.part_element_world_aabb(aerator, elem="aerator_face")
    with ctx.pose({aerator_flip: AERATOR_FLIP_UPPER}):
        flipped_aer = ctx.part_world_aabb(aerator)
        flipped_face = ctx.part_element_world_aabb(aerator, elem="aerator_face")

    ctx.check(
        "aerator flip swings the nozzle end forward (+X) at max angle",
        rest_face is not None
        and flipped_face is not None
        and 0.5 * (flipped_face[0][0] + flipped_face[1][0])
        > 0.5 * (rest_face[0][0] + rest_face[1][0]) + 0.02,
        details=f"rest_face={rest_face}, flipped_face={flipped_face}",
    )
    ctx.check(
        "aerator flip tilts the body from mostly-down toward mostly-forward",
        rest_aer is not None
        and flipped_aer is not None
        and flipped_aer[1][0] > rest_aer[1][0] + 0.02,
        details=f"rest={rest_aer}, flipped={flipped_aer}",
    )

    # ----- aerator collar contact with spout tube (seated at pivot)
    ctx.expect_contact(
        aerator,
        spout,
        elem_a="aerator_collar",
        elem_b="gooseneck_tube",
        contact_tol=0.003,
        name="aerator collar contacts spout tube at pivot",
    )

    # ----- spout swivel: type and pose
    ctx.check(
        "spout swivel is revolute +/-60 deg about vertical axis",
        swivel is not None
        and swivel.articulation_type == ArticulationType.REVOLUTE
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + math.pi / 3.0) < 1e-6
        and abs(swivel.motion_limits.upper - math.pi / 3.0) < 1e-6
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )
    with ctx.pose({swivel: 1.0}):
        sw_aer = ctx.part_world_position(aerator)
    rest_aer_pos = ctx.part_world_position(aerator)
    ctx.check(
        "spout swivel carries the aerator sideways about the column axis",
        sw_aer is not None and sw_aer[1] > 0.10 and rest_aer_pos is not None
        and abs(rest_aer_pos[1]) < 0.01,
        details=f"rest={rest_aer_pos}, swiveled={sw_aer}",
    )

    # ----- lever
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
        rest_lever is not None
        and tilted_lever is not None
        and tilted_lever[1][0] > rest_lever[1][0] + 0.05,
        details=f"rest={rest_lever}, tilted={tilted_lever}",
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

    # ----- dial
    ctx.check(
        "dial knob is revolute +/-135 deg",
        dial_knob.articulation_type == ArticulationType.REVOLUTE
        and dial_knob.motion_limits is not None
        and abs(dial_knob.motion_limits.lower + 2.3562) < 1e-4
        and abs(dial_knob.motion_limits.upper - 2.3562) < 1e-4,
    )
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
    ctx.expect_overlap(
        dial,
        column,
        axes="y",
        elem_a="dial_body",
        elem_b="control_pod",
        min_overlap=0.0005,
        name="dial cap seated on the pod end",
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

    return ctx.report()


object_model = build_object_model()
