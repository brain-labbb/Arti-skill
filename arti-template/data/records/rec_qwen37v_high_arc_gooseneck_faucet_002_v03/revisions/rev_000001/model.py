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
# Squared-bridge gooseneck kitchen faucet variant, ~0.45 m tall.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (direction the gooseneck reaches over the sink), +Z is up.
# - Tapered conical column rises on Z axis to z ≈ 0.307.
# - Squared bridge gooseneck: vertical riser → softened 90° elbow → horizontal
#   bridge deck → second softened elbow → vertical drop leg.
# - Spout swivels about the column vertical axis (±60°).
# - Flip-down aerator at the nozzle tip (revolute about horizontal Y, 0–70°).
# - Right-side (-Y) valve body at z=0.14 carries a slim pin lever (±45°).
# - Front control pod with dial cap (±135°).
# - Hot/cold tick marks as physical geometry near the valve area.
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

# Squared-bridge gooseneck (spout-local, origin at top of collar)
TUBE_R = 0.012
BRIDGE_H = 0.140       # height of horizontal bridge above swivel
ELBOW_R = 0.022        # softened elbow radius
REACH_X = 0.170        # horizontal reach
DROP_END = 0.003       # z of open tube tip in spout-local coords

# Aerator
AERATOR_R = 0.014
AERATOR_LEN = 0.018
AERATOR_RIM_R = 0.016
AERATOR_RIM_H = 0.004

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

# Tick marks
TICK_W = 0.003     # width (x)
TICK_H = 0.007     # height (z)
TICK_D = 0.002     # depth/protrusion (y)
TICK_SPACING = 0.012
N_TICKS = 3


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
    """Squared bridge gooseneck: riser, softened elbow, horizontal bridge deck,
    second softened elbow, drop leg.  All in XZ plane."""
    # Elbow arc midpoints (45° on each arc)
    cos45 = math.cos(math.pi / 4.0)
    sin45 = math.sin(math.pi / 4.0)

    # First elbow: center at (ELBOW_R, BRIDGE_H - ELBOW_R)
    # Arc from (0, BRIDGE_H - ELBOW_R) to (ELBOW_R, BRIDGE_H)
    e1_mid_x = ELBOW_R - ELBOW_R * cos45
    e1_mid_z = (BRIDGE_H - ELBOW_R) + ELBOW_R * sin45

    # Second elbow: center at (REACH_X - ELBOW_R, BRIDGE_H - ELBOW_R)
    # Arc from (REACH_X - ELBOW_R, BRIDGE_H) to (REACH_X, BRIDGE_H - ELBOW_R)
    e2_mid_x = (REACH_X - ELBOW_R) + ELBOW_R * sin45
    e2_mid_z = (BRIDGE_H - ELBOW_R) + ELBOW_R * cos45

    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, BRIDGE_H - ELBOW_R)
        .threePointArc((e1_mid_x, e1_mid_z), (ELBOW_R, BRIDGE_H))
        .lineTo(REACH_X - ELBOW_R, BRIDGE_H)
        .threePointArc((e2_mid_x, e2_mid_z), (REACH_X, BRIDGE_H - ELBOW_R))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squared_bridge_gooseneck_faucet")

    gold = model.material("brushed_gold", rgba=(0.78, 0.62, 0.28, 1.0))
    black = model.material("onyx_black", rgba=(0.05, 0.05, 0.05, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.75, 0.76, 0.78, 1.0))
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

    # ---- Hot/cold tick marks on the column surface near the valve body ----
    # Each tick's y position accounts for the local column radius at its z.
    for i in range(N_TICKS):
        dz = (i + 1) * TICK_SPACING
        for sign, color, prefix in [(1, red, "hot"), (-1, blue, "cold")]:
            tick_z = VALVE_Z + sign * dz
            # Interpolate column radius at tick_z (base→mid taper segment)
            t = min(tick_z / COLUMN_MID_Z, 1.0)
            col_r = COLUMN_BASE_R - (COLUMN_BASE_R - COLUMN_MID_R) * t
            # Embed tick inner face 0.5 mm into the column so it reads attached
            tick_y = -(col_r - 0.0005 + TICK_D / 2.0)
            column.visual(
                Box((TICK_W, TICK_D, TICK_H)),
                origin=Origin(xyz=(0.0, tick_y, tick_z)),
                material=color,
                name=f"{prefix}_tick_{i}",
            )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_bridge_gooseneck_shape(), "bridge_tube"),
        material=gold,
        name="bridge_tube",
    )
    # Spout tip ring (decorative collar at the drop-leg end)
    spout.visual(
        Cylinder(radius=TUBE_R + 0.003, length=0.006),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + 0.003)),
        material=gold,
        name="spout_tip_ring",
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

    # ------------------------------------------------------- flip-down aerator
    aerator = model.part("outlet_aerator")
    # Aerator body hangs below the pivot at rest (local +Z down from frame).
    # Part frame sits at the pivot point; body extends in -Z.
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_LEN / 2.0)),
        material=chrome,
        name="aerator_body",
    )
    # Wider rim at the top of the aerator (pivot collar)
    aerator.visual(
        Cylinder(radius=AERATOR_RIM_R, length=AERATOR_RIM_H),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_RIM_H / 2.0)),
        material=chrome,
        name="aerator_rim",
    )
    # Small screen ring at the aerator bottom
    aerator.visual(
        Cylinder(radius=AERATOR_R - 0.002, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -(AERATOR_LEN + 0.0015))),
        material=black,
        name="aerator_screen",
    )
    model.articulation(
        "aerator_flip",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=math.radians(70.0)
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
    aerator = object_model.get_part("outlet_aerator")
    lever = object_model.get_part("pin_lever")
    dial = object_model.get_part("dial_cap")

    swivel = object_model.get_articulation("spout_swivel")
    aerator_flip = object_model.get_articulation("aerator_flip")
    lever_pivot = object_model.get_articulation("lever_pivot")
    dial_knob = object_model.get_articulation("dial_knob")

    # Scoped intentional overlaps: lever collar captured on valve body,
    # dial cap seated into pod end.
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
        "gooseneck bridge apex near 0.45 m",
        spout_aabb is not None and 0.43 <= spout_aabb[1][2] <= 0.48,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- squared bridge shape: horizontal span at apex
    spout_elem_aabb = ctx.part_element_world_aabb(spout, elem="bridge_tube")
    ctx.check(
        "bridge tube spans horizontally at the apex",
        spout_elem_aabb is not None
        and (spout_elem_aabb[1][0] - spout_elem_aabb[0][0]) > 0.10,
        details=f"bridge_tube aabb={spout_elem_aabb}",
    )

    # ----- flip-down aerator at nozzle
    aer_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator hangs below the spout tip at rest",
        aer_aabb is not None and aer_aabb[0][2] < (DROP_END + SWIVEL_Z),
        details=f"aerator aabb={aer_aabb}",
    )

    # ----- hot/cold tick marks exist as geometry
    for i in range(N_TICKS):
        hot_name = f"hot_tick_{i}"
        cold_name = f"cold_tick_{i}"
        hot_aabb = ctx.part_element_world_aabb(column, elem=hot_name)
        cold_aabb = ctx.part_element_world_aabb(column, elem=cold_name)
        ctx.check(
            f"{hot_name} is visible geometry on the column",
            hot_aabb is not None and (hot_aabb[1][2] - hot_aabb[0][2]) > 0.003,
            details=f"{hot_name} aabb={hot_aabb}",
        )
        ctx.check(
            f"{cold_name} is visible geometry on the column",
            cold_aabb is not None and (cold_aabb[1][2] - cold_aabb[0][2]) > 0.003,
            details=f"{cold_name} aabb={cold_aabb}",
        )

    # Hot ticks above valve, cold ticks below
    hot0 = ctx.part_element_world_aabb(column, elem="hot_tick_0")
    cold0 = ctx.part_element_world_aabb(column, elem="cold_tick_0")
    ctx.check(
        "hot ticks are above cold ticks on the column",
        hot0 is not None and cold0 is not None
        and hot0[0][2] > cold0[1][2],
        details=f"hot0={hot0}, cold0={cold0}",
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
        "aerator flip is revolute with 0 to 70 deg range",
        aerator_flip.articulation_type == ArticulationType.REVOLUTE
        and aerator_flip.motion_limits is not None
        and abs(aerator_flip.motion_limits.lower) < 1e-6
        and abs(aerator_flip.motion_limits.upper - math.radians(70.0)) < 1e-4,
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

    # ----- seating checks
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

    # ----- aerator flip pose: body swings forward and tilts
    rest_aer_body = ctx.part_element_world_aabb(aerator, elem="aerator_body")
    with ctx.pose({aerator_flip: math.radians(45.0)}):
        flipped_aer_body = ctx.part_element_world_aabb(aerator, elem="aerator_body")
    ctx.check(
        "aerator flip swings the body forward in +X from the spout tip",
        rest_aer_body is not None
        and flipped_aer_body is not None
        and flipped_aer_body[1][0] > rest_aer_body[1][0] + 0.005,
        details=f"rest_aabb={rest_aer_body}, flipped_aabb={flipped_aer_body}",
    )

    # ----- swivel pose: spout carries the aerator sideways
    rest_aer_sw = ctx.part_world_position(aerator)
    with ctx.pose({swivel: 1.0}):
        sw_aer = ctx.part_world_position(aerator)
    ctx.check(
        "spout swivel carries the aerator sideways about the column axis",
        sw_aer is not None and abs(sw_aer[1]) > 0.10 and rest_aer_sw is not None
        and abs(rest_aer_sw[1]) < 0.01,
        details=f"rest={rest_aer_sw}, swiveled={sw_aer}",
    )

    # ----- lever pose: pin sweeps fore/aft about the valve axis
    rest_lever = ctx.part_world_aabb(lever)
    with ctx.pose({lever_pivot: math.pi / 4.0}):
        tilted_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "lever pin sweeps in X when rotated about the valve axis",
        rest_lever is not None
        and tilted_lever is not None
        and tilted_lever[1][0] > rest_lever[1][0] + 0.05,
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
        and abs(0.5 * (turned_dot[0][2] + turned_dot[1][2]) - 0.5 * (rest_dot[0][2] + rest_dot[1][2]))
        > 0.005,
        details=f"rest={rest_dot}, turned={turned_dot}",
    )

    return ctx.report()


object_model = build_object_model()
