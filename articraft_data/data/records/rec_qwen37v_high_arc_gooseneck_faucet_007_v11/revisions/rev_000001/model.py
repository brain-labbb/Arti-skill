from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Variant 11: High-arc gooseneck faucet (fork of gloss-black monobloc mixer).
#
# Changes from parent:
# - Taller spout with a tighter forward bend (apex ~0.423 m, reach ~0.096 m).
# - Small top flow knob at the gooseneck apex, independently rotating about Z.
# - Visible cold/hot tick marks as raised geometry on the valve end caps.
# - Two chrome mounting collars on the pedestal column.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (direction the gooseneck reaches over the sink).
# - +Z is up.
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# Cross valve cylinder
CROSS_Z = 0.085
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0  # 0.0875

# Pin levers
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# Swivel collar
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

# Mounting collars on the pedestal (NEW)
MOUNT_COLLAR_R = 0.026
MOUNT_COLLAR_H = 0.005
MOUNT_COLLAR_Z0 = 0.040
MOUNT_COLLAR_Z1 = 0.120

# Taller gooseneck spout with tighter forward bend (CHANGED)
TUBE_R = 0.015
ARC_R = 0.048  # tighter bend (was 0.072)
RISER_TOP = 0.220  # taller riser (was 0.153)
REACH_X = 2.0 * ARC_R  # 0.096 m (was 0.144)
DROP_END = 0.120

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028
AERATOR_R = 0.0118
AERATOR_LEN = 0.003

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.423 m

SWIVEL_LIMIT = math.radians(110.0)

# Flow knob at gooseneck apex (NEW)
KNOB_R = 0.010
KNOB_H = 0.012
KNOB_STEM_R = 0.004
KNOB_INDICATOR_W = 0.002
KNOB_INDICATOR_L = 0.024
KNOB_INDICATOR_H = 0.003
KNOB_LOCAL_X = ARC_R  # 0.048
KNOB_LOCAL_Z = RISER_TOP + ARC_R + TUBE_R  # 0.283
KNOB_ROTATE_LIMIT = math.radians(90.0)

# Tick marks (NEW)
TICK_W = 0.008
TICK_D = 0.002
TICK_H = 0.003
TICK_SPACING = 0.008
TICK_Y_OFFSET = CAP_Y + CAP_LEN / 2.0 + TICK_D / 2.0  # 0.091


def _gooseneck_shape() -> cq.Workplane:
    """Swan-neck tube: tall straight riser, tight semicircular arc, drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v11")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    cold_blue = model.material("cold_indicator_blue", rgba=(0.15, 0.25, 0.65, 1.0))
    hot_red = model.material("hot_indicator_red", rgba=(0.65, 0.12, 0.12, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        Cylinder(radius=BASE_DISC_R, length=BASE_DISC_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_H / 2.0)),
        material=chrome,
        name="base_disc",
    )
    column.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_TOP - 0.004),
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_TOP + 0.004) / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )
    # Horizontal cross valve cylinder through the column, left-right (Y axis).
    column.visual(
        Cylinder(radius=CROSS_R, length=CROSS_TUBE_LEN),
        origin=Origin(xyz=(0.0, 0.0, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="cross_tube",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_0",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, -CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_1",
    )
    # Thin chrome collar ring separating the column from the swivel spout.
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # Two chrome mounting collars on the pedestal (NEW)
    column.visual(
        Cylinder(radius=MOUNT_COLLAR_R, length=MOUNT_COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_COLLAR_Z0)),
        material=chrome,
        name="mount_collar_0",
    )
    column.visual(
        Cylinder(radius=MOUNT_COLLAR_R, length=MOUNT_COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_COLLAR_Z1)),
        material=chrome,
        name="mount_collar_1",
    )

    # Cold tick marks: 3 raised bars on the +Y valve end cap face
    for i, dz in enumerate([-TICK_SPACING, 0.0, TICK_SPACING]):
        column.visual(
            Box((TICK_W, TICK_D, TICK_H)),
            origin=Origin(xyz=(0.0, TICK_Y_OFFSET, CROSS_Z + dz)),
            material=cold_blue,
            name=f"tick_cold_{i}",
        )

    # Hot tick marks: 3 raised bars on the -Y valve end cap face
    for i, dz in enumerate([-TICK_SPACING, 0.0, TICK_SPACING]):
        column.visual(
            Box((TICK_W, TICK_D, TICK_H)),
            origin=Origin(xyz=(0.0, -TICK_Y_OFFSET, CROSS_Z + dz)),
            material=hot_red,
            name=f"tick_hot_{i}",
        )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    spout.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + SLEEVE_LEN / 2.0)),
        material=chrome,
        name="tip_sleeve",
    )
    spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - 0.001)),
        material=outlet_dark,
        name="outlet_aerator",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5, lower=-SWIVEL_LIMIT, upper=SWIVEL_LIMIT
        ),
    )

    # ------------------------------------------------------------- pin levers
    for idx, y_sign in ((0, 1.0), (1, -1.0)):
        lever = model.part(f"pin_lever_{idx}")
        lever.visual(
            Cylinder(radius=BOSS_R, length=BOSS_LEN),
            origin=Origin(xyz=(0.0, 0.0, BOSS_Z)),
            material=gloss_black,
            name="lever_boss",
        )
        lever.visual(
            Cylinder(radius=PIN_R, length=PIN_LEN),
            origin=Origin(xyz=(0.0, 0.0, PIN_Z0 + PIN_LEN / 2.0)),
            material=gloss_black,
            name="lever_pin",
        )
        model.articulation(
            f"lever_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=column,
            child=lever,
            origin=Origin(xyz=(0.0, y_sign * LEVER_Y, CROSS_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=2.0, lower=-math.pi / 2.0, upper=0.0
            ),
        )

    # -------------------------------------------------------- flow knob (NEW)
    flow_knob = model.part("flow_knob")
    # Stem embeds 6 mm into the tube for mounting connectivity.
    flow_knob.visual(
        Cylinder(radius=KNOB_STEM_R, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=chrome,
        name="knob_stem",
    )
    # Main knob body sits on the tube surface.
    flow_knob.visual(
        Cylinder(radius=KNOB_R, length=KNOB_H),
        origin=Origin(xyz=(0.0, 0.0, KNOB_H / 2.0)),
        material=gloss_black,
        name="knob_body",
    )
    # Thin indicator fin on top makes rotation visible.
    flow_knob.visual(
        Box((KNOB_INDICATOR_W, KNOB_INDICATOR_L, KNOB_INDICATOR_H)),
        origin=Origin(xyz=(0.0, 0.0, KNOB_H + KNOB_INDICATOR_H / 2.0)),
        material=chrome,
        name="knob_indicator",
    )
    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=flow_knob,
        origin=Origin(xyz=(KNOB_LOCAL_X, 0.0, KNOB_LOCAL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-KNOB_ROTATE_LIMIT,
            upper=KNOB_ROTATE_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")
    flow_knob = object_model.get_part("flow_knob")

    swivel = object_model.get_articulation("spout_swivel")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")
    knob_rotate = object_model.get_articulation("knob_rotate")

    # Intentional seated insertions
    ctx.allow_overlap(
        lever_0,
        column,
        elem_a="lever_boss",
        elem_b="cross_tube",
        reason="Lever boss intentionally seats a few mm into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1,
        column,
        elem_a="lever_boss",
        elem_b="cross_tube",
        reason="Lever boss intentionally seats a few mm into the valve cylinder.",
    )
    ctx.allow_overlap(
        flow_knob,
        spout,
        elem_a="knob_stem",
        elem_b="gooseneck_tube",
        reason="Knob stem intentionally embeds into the gooseneck tube apex for seated mounting.",
    )

    # ----- grounding, scale, proportions
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    disc = ctx.part_element_world_aabb(column, elem="base_disc")
    ctx.check(
        "single chrome base disc sits on the deck (wide, thin)",
        disc is not None
        and 0.080 <= (disc[1][0] - disc[0][0]) <= 0.090
        and (disc[1][2] - disc[0][2]) <= 0.010,
        details=f"base disc aabb={disc}",
    )
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "vertical column is ~0.04 m diameter",
        shaft is not None and 0.038 <= (shaft[1][0] - shaft[0][0]) <= 0.042,
        details=f"column shaft aabb={shaft}",
    )

    # ----- variant 11: taller spout with tighter forward bend
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex is taller than parent (~0.42 m)",
        spout_aabb is not None and spout_aabb[1][2] >= 0.415,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck has tighter forward bend (reach < 0.13 m)",
        spout_aabb is not None and spout_aabb[1][0] < 0.13,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck still arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.08,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- cross valve cylinder with flat black end caps
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    cap_0 = ctx.part_element_world_aabb(column, elem="valve_end_cap_0")
    cap_1 = ctx.part_element_world_aabb(column, elem="valve_end_cap_1")
    ctx.check(
        "cross-cylinder is ~0.045 m diameter at mid-column height",
        cross is not None
        and 0.043 <= (cross[1][2] - cross[0][2]) <= 0.047
        and 0.06 <= 0.5 * (cross[0][2] + cross[1][2]) <= 0.11,
        details=f"cross aabb={cross}",
    )
    ctx.check(
        "valve assembly spans ~0.18 m end-to-end cap face to cap face",
        cap_0 is not None
        and cap_1 is not None
        and 0.178 <= (cap_0[1][1] - cap_1[0][1]) <= 0.182,
        details=f"cap_0={cap_0}, cap_1={cap_1}",
    )

    # ----- chrome collar ring between column and spout
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "thin chrome collar sits above the cross and below the spout",
        collar is not None
        and cross is not None
        and collar[0][2] >= cross[1][2]
        and spout_aabb is not None
        and collar[1][2] <= spout_aabb[0][2] + 1e-6,
        details=f"collar={collar}, cross_top={cross[1][2] if cross else None}",
    )
    ctx.expect_contact(
        spout,
        column,
        elem_a="gooseneck_tube",
        elem_b="swivel_collar",
        contact_tol=0.001,
        name="gooseneck riser seats on the chrome collar",
    )

    # ----- variant 11: two mounting collars on the pedestal
    mc0 = ctx.part_element_world_aabb(column, elem="mount_collar_0")
    mc1 = ctx.part_element_world_aabb(column, elem="mount_collar_1")
    ctx.check(
        "two mounting collars on the pedestal column",
        mc0 is not None
        and mc1 is not None
        and mc0[0][2] > BASE_DISC_H  # above base disc
        and mc1[1][2] < SWIVEL_Z - COLLAR_LEN  # below swivel collar
        and mc1[0][2] > mc0[1][2],  # upper is above lower
        details=f"mc0={mc0}, mc1={mc1}",
    )
    ctx.check(
        "mounting collars are wider than the column shaft",
        mc0 is not None
        and shaft is not None
        and (mc0[1][0] - mc0[0][0]) > (shaft[1][0] - shaft[0][0]) + 0.005,
        details=f"mc0={mc0}, shaft={shaft}",
    )

    # ----- variant 11: cold/hot tick marks as geometry
    tick_cold = ctx.part_element_world_aabb(column, elem="tick_cold_1")
    tick_hot = ctx.part_element_world_aabb(column, elem="tick_hot_1")
    ctx.check(
        "cold tick marks exist on +Y valve side",
        tick_cold is not None and tick_cold[0][1] > 0.08,
        details=f"tick_cold={tick_cold}",
    )
    ctx.check(
        "hot tick marks exist on -Y valve side",
        tick_hot is not None and tick_hot[1][1] < -0.08,
        details=f"tick_hot={tick_hot}",
    )
    ctx.check(
        "tick marks are at the cross valve height",
        tick_cold is not None
        and tick_hot is not None
        and cross is not None
        and tick_cold[0][2] >= cross[0][2] - 0.005
        and tick_cold[1][2] <= cross[1][2] + 0.005
        and tick_hot[0][2] >= cross[0][2] - 0.005
        and tick_hot[1][2] <= cross[1][2] + 0.005,
        details=f"cold={tick_cold}, hot={tick_hot}, cross={cross}",
    )

    # ----- chrome tip sleeve with downward outlet at the spout end
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    aerator = ctx.part_element_world_aabb(spout, elem="outlet_aerator")
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "chrome tip sleeve wraps the spout drop leg with a downward outlet",
        sleeve is not None
        and aerator is not None
        and tube is not None
        and 0.24 <= sleeve[0][2] <= 0.28
        and aerator[0][2] < sleeve[0][2]
        and aerator[0][2] <= SWIVEL_Z + DROP_END
        and abs(0.5 * (sleeve[0][0] + sleeve[1][0]) - REACH_X) <= 0.005,
        details=f"sleeve={sleeve}, aerator={aerator}, tube={tube}",
    )

    # ----- pin levers: geometry and seating
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        pin = ctx.part_element_world_aabb(lever, elem="lever_pin")
        ctx.check(
            f"{name} pin is slim (0.012 m dia) and 0.10 m long, vertical at rest",
            pin is not None
            and 0.010 <= (pin[1][0] - pin[0][0]) <= 0.014
            and 0.098 <= (pin[1][2] - pin[0][2]) <= 0.102,
            details=f"pin aabb={pin}",
        )
        ctx.expect_overlap(
            lever,
            column,
            axes="z",
            elem_a="lever_boss",
            elem_b="cross_tube",
            min_overlap=0.003,
            name=f"{name} boss seats into the valve cylinder",
        )
    pin0 = ctx.part_element_world_aabb(lever_0, elem="lever_pin")
    pin1 = ctx.part_element_world_aabb(lever_1, elem="lever_pin")
    ctx.check(
        "the two pin levers rise from the tops of the two valve bodies",
        pin0 is not None
        and pin1 is not None
        and cross is not None
        and 0.5 * (pin0[0][1] + pin0[1][1]) > 0.04
        and 0.5 * (pin1[0][1] + pin1[1][1]) < -0.04
        and pin0[0][2] >= cross[1][2] - 0.001
        and pin1[0][2] >= cross[1][2] - 0.001,
        details=f"pin0={pin0}, pin1={pin1}",
    )

    # ----- variant 11: flow knob at the gooseneck apex
    knob_aabb = ctx.part_world_aabb(flow_knob)
    ctx.check(
        "flow knob positioned at the gooseneck apex (z > 0.41)",
        knob_aabb is not None and knob_aabb[1][2] > 0.41,
        details=f"knob aabb={knob_aabb}",
    )
    knob_body = ctx.part_element_world_aabb(flow_knob, elem="knob_body")
    ctx.check(
        "flow knob body is small (radius ~0.010 m)",
        knob_body is not None
        and 0.018 <= (knob_body[1][0] - knob_body[0][0]) <= 0.022,
        details=f"knob_body={knob_body}",
    )

    # ----- knob stem insertion proof
    ctx.expect_overlap(
        flow_knob,
        spout,
        axes="z",
        elem_a="knob_stem",
        elem_b="gooseneck_tube",
        min_overlap=0.003,
        name="knob stem remains inserted in the tube apex",
    )

    # ----- joint plan: types, axes, ranges
    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0 deg about the valve's left-right axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6
            and pivot.mimic is None,
        )
    ctx.check(
        "knob_rotate is revolute ±90° about vertical at the apex",
        knob_rotate.articulation_type == ArticulationType.REVOLUTE
        and tuple(knob_rotate.axis) == (0.0, 0.0, 1.0)
        and knob_rotate.motion_limits is not None
        and abs(knob_rotate.motion_limits.lower + KNOB_ROTATE_LIMIT) < 1e-6
        and abs(knob_rotate.motion_limits.upper - KNOB_ROTATE_LIMIT) < 1e-6
        and knob_rotate.mimic is None,
    )

    # ----- lever pose: full -90 deg tilt brings the pin toward the user (+X)
    rest_0 = ctx.part_world_aabb(lever_0)
    rest_1 = ctx.part_world_aabb(lever_1)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
        still_1 = ctx.part_world_aabb(lever_1)
    ctx.check(
        "lever 0 tilts from vertical to horizontal toward the user at q=-90 deg",
        rest_0 is not None
        and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.10
        and tilted_0[1][2] < CROSS_Z + 0.03,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )
    ctx.check(
        "lever 1 is independent of lever 0 (stays vertical while 0 tilts)",
        rest_1 is not None
        and still_1 is not None
        and abs(still_1[1][2] - rest_1[1][2]) < 1e-9,
        details=f"rest={rest_1}, while_0_tilted={still_1}",
    )
    with ctx.pose({pivot_1: -math.pi / 2.0}):
        tilted_1 = ctx.part_world_aabb(lever_1)
    ctx.check(
        "lever 1 tilts from vertical to horizontal toward the user at q=-90 deg",
        rest_1 is not None
        and tilted_1 is not None
        and tilted_1[1][0] > rest_1[1][0] + 0.10
        and tilted_1[1][2] < CROSS_Z + 0.03,
        details=f"rest={rest_1}, tilted={tilted_1}",
    )

    # ----- swivel pose: spout outlet sweeps sideways about the column axis
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout swivel carries the outlet sideways about the vertical axis",
        rest_sleeve is not None
        and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 1e-6
        and 0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1]) > 0.06,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    # ----- variant 11: flow knob rotation pose
    rest_indicator = ctx.part_element_world_aabb(flow_knob, elem="knob_indicator")
    with ctx.pose({knob_rotate: math.pi / 2.0}):
        rotated_indicator = ctx.part_element_world_aabb(flow_knob, elem="knob_indicator")
    ctx.check(
        "flow knob rotates independently (indicator Y-aligned at rest, X-aligned at 90°)",
        rest_indicator is not None
        and rotated_indicator is not None
        and (rest_indicator[1][1] - rest_indicator[0][1]) > 0.020
        and (rotated_indicator[1][1] - rotated_indicator[0][1]) < 0.005,
        details=f"rest={rest_indicator}, rotated={rotated_indicator}",
    )

    return ctx.report()


object_model = build_object_model()
