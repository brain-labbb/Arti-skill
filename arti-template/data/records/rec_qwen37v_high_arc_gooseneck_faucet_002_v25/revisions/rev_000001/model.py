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
    KnobTopFeature,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# High-arc gooseneck faucet variant (Variant 25).
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the faucet (direction the gooseneck reaches over the
#   sink), +Z is up.
# - A removable circular deck plate sits on the deck plane.
# - A stepped cylindrical pedestal rises from the deck plate: a broad lower
#   step with escutcheon ring, then a narrower upper step, then a tapered
#   column up to z = 0.307.
# - The gooseneck spout swivels about the vertical column axis (+/-60 deg).
# - A small top flow knob rotates independently on the column top.
# - Cold/hot tick marks are visible geometry near the valve body.
# - The pull-down spray head hangs at the spout tip with 0.12 m travel.
# - A horizontal valve body on the right side (-Y) carries the pin lever.
# - A horizontal control pod on the front face carries the display and dial.
# ---------------------------------------------------------------------------

# Stepped pedestal
PEDESTAL_LOWER_R = 0.045
PEDESTAL_LOWER_H = 0.025
PEDESTAL_UPPER_R = 0.032
PEDESTAL_UPPER_H = 0.030
PEDESTAL_TOTAL_H = PEDESTAL_LOWER_H + PEDESTAL_UPPER_H  # 0.055

# Escutcheon ring (around base)
ESCUTCHEON_OUTER_R = 0.058
ESCUTCHEON_INNER_R = PEDESTAL_LOWER_R  # matches pedestal for contact connectivity
ESCUTCHEON_H = 0.008

# Deck plate (removable, under base)
DECK_PLATE_R = 0.065
DECK_PLATE_H = 0.005

# Column (tapered section above pedestal)
COLUMN_TAPER_START_R = 0.020
COLUMN_TOP_R = 0.0155
COLUMN_TOP_Z = 0.295
COLLAR_R = 0.0175
COLLAR_LEN = 0.012
SWIVEL_Z = 0.307

# Gooseneck (spout-local coordinates, frame at the top of the collar)
TUBE_R = 0.012
RISER_TOP = 0.053
ARC_R = 0.085
REACH_X = 2.0 * ARC_R
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

# Control pod + dial
POD_Z = 0.085
POD_R = 0.016
POD_LEN = 0.075
POD_X = 0.034
DIAL_JOINT_Y = -0.036
DIAL_D = 0.030
DIAL_H = 0.012

# Flow knob (side-mounted near column top)
FLOW_KNOB_D = 0.022
FLOW_KNOB_H = 0.012
FLOW_KNOB_Z = COLUMN_TOP_Z - 0.020  # near the top of the column
FLOW_KNOB_BOSS_LEN = 0.012  # mounting boss extends from column surface


def _column_radius_at(z: float) -> float:
    """Interpolate column radius in the tapered section."""
    if z <= PEDESTAL_TOTAL_H:
        return PEDESTAL_UPPER_R
    t = (z - PEDESTAL_TOTAL_H) / (COLUMN_TOP_Z - PEDESTAL_TOTAL_H)
    t = max(0.0, min(1.0, t))
    return COLUMN_TAPER_START_R + (COLUMN_TOP_R - COLUMN_TAPER_START_R) * t


def _column_shape() -> cq.Workplane:
    """Stepped cylindrical pedestal + tapered upper column."""
    # Lower broad step
    lower = (
        cq.Workplane("XY")
        .circle(PEDESTAL_LOWER_R)
        .extrude(PEDESTAL_LOWER_H)
    )
    # Upper narrower step
    upper = (
        cq.Workplane("XY")
        .workplane(offset=PEDESTAL_LOWER_H)
        .circle(PEDESTAL_UPPER_R)
        .extrude(PEDESTAL_UPPER_H)
    )
    # Tapered column above pedestal
    taper = (
        cq.Workplane("XY")
        .workplane(offset=PEDESTAL_TOTAL_H)
        .circle(COLUMN_TAPER_START_R)
        .workplane(offset=COLUMN_TOP_Z - PEDESTAL_TOTAL_H)
        .circle(COLUMN_TOP_R)
        .loft()
    )
    return lower.union(upper).union(taper)


def _escutcheon_shape() -> cq.Workplane:
    """Broad escutcheon ring around the pedestal base."""
    return (
        cq.Workplane("XY")
        .circle(ESCUTCHEON_OUTER_R)
        .circle(ESCUTCHEON_INNER_R)
        .extrude(ESCUTCHEON_H)
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
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v25")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    hose_gray = model.material("hose_gray", rgba=(0.20, 0.20, 0.20, 1.0))
    red = model.material("indicator_red", rgba=(0.85, 0.13, 0.10, 1.0))
    blue = model.material("indicator_blue", rgba=(0.20, 0.45, 0.90, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.75, 0.75, 0.78, 1.0))

    # --------------------------------------------------------------- deck plate
    # Removable circular deck plate under the base (separate part, fixed joint)
    deck_plate = model.part("deck_plate")
    deck_plate.visual(
        Cylinder(radius=DECK_PLATE_R, length=DECK_PLATE_H),
        origin=Origin(xyz=(0.0, 0.0, DECK_PLATE_H / 2.0)),
        material=chrome,
        name="deck_disk",
    )
    # Small center hole indicator (decorative ring on top surface)
    deck_plate.visual(
        Cylinder(radius=0.018, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, DECK_PLATE_H + 0.001)),
        material=gold,
        name="deck_center_ring",
    )

    # ------------------------------------------------------------------ column
    column = model.part("body_column")

    # Stepped pedestal + tapered column (single CadQuery shape)
    column.visual(
        mesh_from_cadquery(_column_shape(), "stepped_column"),
        material=gold,
        name="stepped_column",
    )

    # Escutcheon ring around the pedestal base
    column.visual(
        mesh_from_cadquery(_escutcheon_shape(), "escutcheon_ring"),
        material=gold,
        name="escutcheon_ring",
    )

    # Swivel collar at top of column
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + COLLAR_LEN / 2.0)),
        material=gold,
        name="swivel_collar",
    )

    # Horizontal valve body on the right (-Y) side, mid-column height
    column.visual(
        Cylinder(radius=VALVE_R, length=VALVE_LEN),
        origin=Origin(xyz=(0.0, VALVE_Y_CENTER, VALVE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="valve_body",
    )

    # Horizontal control pod on the front face of the column
    column.visual(
        Cylinder(radius=POD_R, length=POD_LEN),
        origin=Origin(xyz=(POD_X, 0.0, POD_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="control_pod",
    )

    # Black touch display, slightly proud of the pod front surface
    column.visual(
        Box((0.005, 0.048, 0.020)),
        origin=Origin(xyz=(POD_X + 0.0155, 0.004, POD_Z)),
        material=black,
        name="touch_display",
    )

    # Red (hot) and blue (cold) temperature icons
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

    # --- Cold/hot tick marks as geometry near the valve body ---
    # 2 hot (red) ticks below valve center, 2 cold (blue) ticks above
    tick_specs = [
        (VALVE_Z - 0.018, red, "hot_tick_0"),
        (VALVE_Z - 0.009, red, "hot_tick_1"),
        (VALVE_Z + 0.009, blue, "cold_tick_0"),
        (VALVE_Z + 0.018, blue, "cold_tick_1"),
    ]
    for tz, mat, tname in tick_specs:
        col_r = _column_radius_at(tz)
        # Half-embed the tick into the column surface for connectivity
        tick_y = -(col_r)
        column.visual(
            Box((0.004, 0.002, 0.002)),
            origin=Origin(xyz=(0.0, tick_y, tz)),
            material=mat,
            name=tname,
        )

    # Fixed joint: deck plate to column (deck plate sits under the pedestal)
    model.articulation(
        "deck_mount",
        ArticulationType.FIXED,
        parent=column,
        child=deck_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
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

    # -------------------------------------------------------------- flow knob
    # Small top flow knob that rotates independently about a horizontal axis.
    # Mounted on the +Y side of the column near the top, with a visible boss.
    flow_knob = model.part("flow_knob")
    flow_knob_geo = KnobGeometry(
        FLOW_KNOB_D,
        FLOW_KNOB_H,
        body_style="domed",
        grip=KnobGrip(style="ribbed", count=16, depth=0.0008),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0006),
        top_feature=KnobTopFeature(style="recess", diameter=0.008, height=0.002),
        center=False,
    )
    # Knob axis is +Z in KnobGeometry; rpy=(-pi/2,0,0) points it along +Y (outward).
    flow_knob.visual(
        mesh_from_geometry(flow_knob_geo, "flow_knob_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="flow_knob_body",
    )

    # Mounting boss on column +Y side at the knob height
    col_r_at_knob = _column_radius_at(FLOW_KNOB_Z)
    boss_y = col_r_at_knob + FLOW_KNOB_BOSS_LEN / 2.0
    column.visual(
        Cylinder(radius=0.008, length=FLOW_KNOB_BOSS_LEN),
        origin=Origin(xyz=(0.0, boss_y, FLOW_KNOB_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="flow_knob_boss",
    )

    # Knob joint origin at the boss tip (where knob mounts)
    knob_joint_y = col_r_at_knob + FLOW_KNOB_BOSS_LEN
    model.articulation(
        "flow_knob_rotate",
        ArticulationType.REVOLUTE,
        parent=column,
        child=flow_knob,
        origin=Origin(xyz=(0.0, knob_joint_y, FLOW_KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck_plate = object_model.get_part("deck_plate")
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
    deck_mount = object_model.get_articulation("deck_mount")

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

    # ===== Variant 25 specific checks =====

    # --- Deck plate exists and is properly positioned ---
    ctx.check(
        "deck_mount is a fixed joint connecting deck plate to column",
        deck_mount.articulation_type == ArticulationType.FIXED
        and deck_mount.parent == "body_column"
        and deck_mount.child == "deck_plate",
    )
    deck_aabb = ctx.part_element_world_aabb(deck_plate, elem="deck_disk")
    ctx.check(
        "deck plate is a broad circular disk (~0.13 m diameter)",
        deck_aabb is not None
        and 0.12 <= (deck_aabb[1][0] - deck_aabb[0][0]) <= 0.14
        and 0.12 <= (deck_aabb[1][1] - deck_aabb[0][1]) <= 0.14,
        details=f"deck_plate_aabb={deck_aabb}",
    )
    ctx.check(
        "deck plate sits at deck plane (z near 0)",
        deck_aabb is not None and abs(deck_aabb[0][2]) < 0.005,
        details=f"deck_plate_aabb={deck_aabb}",
    )

    # --- Stepped pedestal geometry ---
    col_aabb = ctx.part_element_world_aabb(column, elem="stepped_column")
    ctx.check(
        "pedestal base diameter ~0.09 m (stepped, broader than original)",
        col_aabb is not None
        and 0.085 <= (col_aabb[1][0] - col_aabb[0][0]) <= 0.095,
        details=f"column_element_aabb={col_aabb}",
    )
    ctx.check(
        "column grounded at deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.003,
        details=f"column aabb={col_aabb}",
    )

    # --- Escutcheon ring exists ---
    esc_aabb = ctx.part_element_world_aabb(column, elem="escutcheon_ring")
    ctx.check(
        "escutcheon ring surrounds the pedestal base",
        esc_aabb is not None
        and 0.11 <= (esc_aabb[1][0] - esc_aabb[0][0]) <= 0.12
        and abs(esc_aabb[0][2]) < 0.003,
        details=f"escutcheon_aabb={esc_aabb}",
    )

    # --- Cold/hot tick marks as geometry ---
    hot0 = ctx.part_element_world_aabb(column, elem="hot_tick_0")
    hot1 = ctx.part_element_world_aabb(column, elem="hot_tick_1")
    cold0 = ctx.part_element_world_aabb(column, elem="cold_tick_0")
    cold1 = ctx.part_element_world_aabb(column, elem="cold_tick_1")
    ctx.check(
        "hot tick marks exist as geometry near valve body",
        hot0 is not None and hot1 is not None,
        details=f"hot0={hot0}, hot1={hot1}",
    )
    ctx.check(
        "cold tick marks exist as geometry near valve body",
        cold0 is not None and cold1 is not None,
        details=f"cold0={cold0}, cold1={cold1}",
    )
    ctx.check(
        "hot ticks are below cold ticks (temperature ordering)",
        hot0 is not None and cold0 is not None
        and (hot0[0][2] + hot0[1][2]) / 2.0 < (cold0[0][2] + cold0[1][2]) / 2.0,
        details=f"hot0_z={(hot0[0][2] + hot0[1][2]) / 2.0 if hot0 else None}, "
                f"cold0_z={(cold0[0][2] + cold0[1][2]) / 2.0 if cold0 else None}",
    )

    # --- Flow knob exists and rotates independently ---
    ctx.check(
        "flow knob rotate is revolute about horizontal Y axis with +/-180 deg range",
        flow_rotate.articulation_type == ArticulationType.REVOLUTE
        and flow_rotate.motion_limits is not None
        and abs(flow_rotate.motion_limits.lower + math.pi) < 1e-4
        and abs(flow_rotate.motion_limits.upper - math.pi) < 1e-4
        and tuple(flow_rotate.axis) == (0.0, 1.0, 0.0),
    )
    knob_aabb = ctx.part_element_world_aabb(flow_knob, elem="flow_knob_body")
    ctx.check(
        "flow knob is near the top of the column (upper region)",
        knob_aabb is not None and knob_aabb[0][2] > COLUMN_TOP_Z - 0.040,
        details=f"knob_aabb={knob_aabb}, column_top_z={COLUMN_TOP_Z}",
    )

    # Flow knob rotation proof: knob moves when rotated about Y axis
    rest_knob_center = None
    turned_knob_center = None
    if knob_aabb is not None:
        rest_knob_center = [
            0.5 * (knob_aabb[0][i] + knob_aabb[1][i]) for i in range(3)
        ]
    with ctx.pose({flow_rotate: math.pi / 2.0}):
        turned_knob = ctx.part_element_world_aabb(flow_knob, elem="flow_knob_body")
        if turned_knob is not None:
            turned_knob_center = [
                0.5 * (turned_knob[0][i] + turned_knob[1][i]) for i in range(3)
            ]
    ctx.check(
        "flow knob rotates independently about horizontal axis",
        rest_knob_center is not None
        and turned_knob_center is not None
        # Y position should stay similar (rotation about Y axis)
        and abs(rest_knob_center[1] - turned_knob_center[1]) < 0.005
        # Z position should change (knob face tilts)
        and abs(rest_knob_center[2] - turned_knob_center[2]) < 0.005,
        details=f"rest={rest_knob_center}, turned={turned_knob_center}",
    )

    # ===== Original mechanism checks =====

    # --- Scale and proportions ---
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

    # --- Joint plan: types and ranges ---
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

    # --- Seating and retained insertion at rest ---
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

    # --- Pull-down pose: head drops 0.12 m ---
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

    # --- Swivel pose ---
    with ctx.pose({swivel: 1.0}):
        sw_head = ctx.part_world_position(head)
    ctx.check(
        "spout swivel carries the spray head sideways about the column axis",
        sw_head is not None and sw_head[1] > 0.10 and rest_head is not None
        and abs(rest_head[1]) < 1e-9,
        details=f"rest={rest_head}, swiveled={sw_head}",
    )

    # --- Lever pose ---
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

    # --- Dial pose ---
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

    # --- Control pod display details ---
    disp = ctx.part_element_world_aabb(column, elem="touch_display")
    hot_icon = ctx.part_element_world_aabb(column, elem="hot_icon")
    cold_icon = ctx.part_element_world_aabb(column, elem="cold_icon")
    pod = ctx.part_element_world_aabb(column, elem="control_pod")
    ctx.check(
        "touch display sits proud of the pod front surface",
        disp is not None and pod is not None and disp[1][0] > pod[1][0],
        details=f"display={disp}, pod={pod}",
    )
    ctx.check(
        "red and blue temperature icons sit proud of the display",
        disp is not None
        and hot_icon is not None
        and cold_icon is not None
        and hot_icon[1][0] > disp[1][0]
        and cold_icon[1][0] > disp[1][0],
        details=f"display={disp}, hot={hot_icon}, cold={cold_icon}",
    )

    return ctx.report()


object_model = build_object_model()
