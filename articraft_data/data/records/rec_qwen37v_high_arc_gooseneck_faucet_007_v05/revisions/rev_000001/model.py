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
# Variant 05: high-arc gooseneck faucet with stepped pedestal, spray head on
# prismatic hose joint, cold/hot tick marks, and mounting collars.
#
# Layout (world frame, deck plane at z = 0):
# - +X is front (direction the gooseneck reaches over the sink), +Z is up.
# - A stepped cylindrical pedestal with a broad escutcheon sits on the deck.
# - Two small mounting collars ring the pedestal above the escutcheon.
# - A gloss-black cylindrical column (0.04 m dia) rises from the pedestal.
# - A horizontal cross-cylinder passes through the column, forming two valve
#   bodies with flat matte-black end caps.
# - Cold/hot tick marks are small raised geometry indicators on the valve caps.
# - From each valve body's top a slim vertical pin lever points up (revolute).
# - A thin chrome collar ring separates the column from the gooseneck spout.
# - The gooseneck arcs up and over to an apex at ~0.38 m.
# - A spray head slides out on a prismatic joint from the spout tip.
# ---------------------------------------------------------------------------

# Stepped pedestal
ESCUTCHEON_R = 0.055
ESCUTCHEON_H = 0.006
STEP1_R = 0.035
STEP1_H = 0.010
STEP2_R = 0.026
STEP2_H = 0.008
PEDESTAL_TOP = ESCUTCHEON_H + STEP1_H + STEP2_H  # 0.024

# Mounting collars on the pedestal
MOUNT_COLLAR_R = 0.028
MOUNT_COLLAR_H = 0.004
MOUNT_COLLAR_Z0 = ESCUTCHEON_H + 0.002  # first collar just above escutcheon
MOUNT_COLLAR_Z1 = ESCUTCHEON_H + STEP1_H + 0.001  # second collar at step1/step2

# Column
COLUMN_R = 0.020
COLUMN_TOP = PEDESTAL_TOP + 0.108  # shaft reaches into collar region

# Cross valve cylinder
CROSS_Z = PEDESTAL_TOP + 0.061  # ~0.085 from deck
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0

# Tick marks (small raised boxes on the end caps)
TICK_W = 0.003
TICK_H = 0.012
TICK_D = 0.002  # proud of the cap surface

# Pin levers
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# Swivel collar + gooseneck
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = COLUMN_TOP + 0.008

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R
DROP_END = 0.124

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028
AERATOR_R = 0.0118
AERATOR_LEN = 0.003

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R

SWIVEL_LIMIT = math.radians(110.0)

# Spray head (prismatic slide)
SPRAY_BODY_R = 0.014
SPRAY_BODY_LEN = 0.040
SPRAY_NOZZLE_R = 0.008
SPRAY_NOZZLE_LEN = 0.008
SPRAY_TRAVEL = 0.045  # max extension distance


def _gooseneck_shape() -> cq.Workplane:
    """Swan-neck tube: straight riser, high semicircular arc, short drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v05")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    tick_cold = model.material("tick_cold_blue", rgba=(0.15, 0.25, 0.65, 1.0))
    tick_hot = model.material("tick_hot_red", rgba=(0.65, 0.12, 0.12, 1.0))
    spray_rubber = model.material("spray_grip", rgba=(0.12, 0.12, 0.13, 1.0))

    # ----------------------------------------------------------- pedestal+column
    column = model.part("body_column")

    # Stepped pedestal: broad escutcheon + two narrowing steps
    column.visual(
        Cylinder(radius=ESCUTCHEON_R, length=ESCUTCHEON_H),
        origin=Origin(xyz=(0.0, 0.0, ESCUTCHEON_H / 2.0)),
        material=chrome,
        name="escutcheon",
    )
    column.visual(
        Cylinder(radius=STEP1_R, length=STEP1_H),
        origin=Origin(xyz=(0.0, 0.0, ESCUTCHEON_H + STEP1_H / 2.0)),
        material=chrome,
        name="pedestal_step_1",
    )
    column.visual(
        Cylinder(radius=STEP2_R, length=STEP2_H),
        origin=Origin(xyz=(0.0, 0.0, ESCUTCHEON_H + STEP1_H + STEP2_H / 2.0)),
        material=chrome,
        name="pedestal_step_2",
    )

    # Two mounting collars on the pedestal
    column.visual(
        Cylinder(radius=MOUNT_COLLAR_R, length=MOUNT_COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_COLLAR_Z0 + MOUNT_COLLAR_H / 2.0)),
        material=chrome,
        name="mount_collar_0",
    )
    column.visual(
        Cylinder(radius=MOUNT_COLLAR_R, length=MOUNT_COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_COLLAR_Z1 + MOUNT_COLLAR_H / 2.0)),
        material=chrome,
        name="mount_collar_1",
    )

    # Column shaft above the pedestal
    shaft_base = PEDESTAL_TOP
    shaft_len = COLUMN_TOP - shaft_base
    column.visual(
        Cylinder(radius=COLUMN_R, length=shaft_len),
        origin=Origin(xyz=(0.0, 0.0, shaft_base + shaft_len / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )

    # Horizontal cross valve cylinder through the column, left-right (Y axis)
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

    # Cold/hot tick marks on end caps (raised box geometry)
    # Tick on cap_0 (+Y side) = cold (blue), on cap_1 (-Y side) = hot (red)
    column.visual(
        Box((TICK_W, TICK_D, TICK_H)),
        origin=Origin(xyz=(0.0, CAP_Y + CAP_LEN / 2.0 + TICK_D / 2.0, CROSS_Z)),
        material=tick_cold,
        name="tick_cold",
    )
    column.visual(
        Box((TICK_W, TICK_D, TICK_H)),
        origin=Origin(xyz=(0.0, -(CAP_Y + CAP_LEN / 2.0 + TICK_D / 2.0), CROSS_Z)),
        material=tick_hot,
        name="tick_hot",
    )

    # Chrome collar ring separating column from swivel spout
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
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

    # ----------------------------------------------------------- spray head
    # Spray head slides downward (-Z) out of the spout tip on a prismatic joint.
    # In the spray head's local frame, geometry is centered at origin; the joint
    # origin sits at the bottom of the tip sleeve, axis = (0, 0, -1) so positive
    # q extends the spray head downward.
    spray = model.part("spray_head")
    spray.visual(
        Cylinder(radius=SPRAY_BODY_R, length=SPRAY_BODY_LEN),
        origin=Origin(xyz=(0.0, 0.0, -SPRAY_BODY_LEN / 2.0)),
        material=spray_rubber,
        name="spray_body",
    )
    spray.visual(
        Cylinder(radius=SPRAY_NOZZLE_R, length=SPRAY_NOZZLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, -SPRAY_BODY_LEN - SPRAY_NOZZLE_LEN / 2.0)),
        material=chrome,
        name="spray_nozzle",
    )

    # Prismatic joint: origin at the top of the tip sleeve; axis = (0,0,-1) so
    # positive q slides the spray head downward (out of the sleeve).
    # At q=0 the spray body is retracted inside the sleeve; at q=max it
    # extends below.
    spray_joint_local_z = DROP_END + SLEEVE_LEN  # top of sleeve in spout-local
    model.articulation(
        "spray_slide",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray,
        origin=Origin(xyz=(REACH_X, 0.0, spray_joint_local_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.3, lower=0.0, upper=SPRAY_TRAVEL
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    spray = object_model.get_part("spray_head")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    spray_slide = object_model.get_articulation("spray_slide")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

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
    # Spray head docks inside the spout tip when retracted
    ctx.allow_overlap(
        spray,
        spout,
        elem_a="spray_body",
        elem_b="tip_sleeve",
        reason="Spray head retracts inside the tip sleeve when not extended.",
    )
    ctx.allow_overlap(
        spray,
        spout,
        elem_a="spray_body",
        elem_b="gooseneck_tube",
        reason="Spray body nests inside the gooseneck drop leg when docked.",
    )

    # ----- stepped pedestal base
    escutcheon = ctx.part_element_world_aabb(column, elem="escutcheon")
    step1 = ctx.part_element_world_aabb(column, elem="pedestal_step_1")
    step2 = ctx.part_element_world_aabb(column, elem="pedestal_step_2")
    ctx.check(
        "broad escutcheon sits on deck (wide, thin, ~0.11 m diameter)",
        escutcheon is not None
        and 0.100 <= (escutcheon[1][0] - escutcheon[0][0]) <= 0.120
        and (escutcheon[1][2] - escutcheon[0][2]) <= 0.008,
        details=f"escutcheon aabb={escutcheon}",
    )
    ctx.check(
        "stepped pedestal has 3 narrowing tiers",
        escutcheon is not None
        and step1 is not None
        and step2 is not None
        and (escutcheon[1][0] - escutcheon[0][0]) > (step1[1][0] - step1[0][0])
        and (step1[1][0] - step1[0][0]) > (step2[1][0] - step2[0][0])
        and step1[0][2] >= escutcheon[1][2] - 0.001
        and step2[0][2] >= step1[1][2] - 0.001,
        details=f"esc={escutcheon}, s1={step1}, s2={step2}",
    )

    # ----- mounting collars on the pedestal
    mc0 = ctx.part_element_world_aabb(column, elem="mount_collar_0")
    mc1 = ctx.part_element_world_aabb(column, elem="mount_collar_1")
    ctx.check(
        "two mounting collars present on pedestal",
        mc0 is not None
        and mc1 is not None
        and mc0[1][2] < mc1[0][2] + 0.005
        and mc0[0][2] >= escutcheon[1][2] - 0.001
        and mc1[1][2] <= step2[1][2] + 0.005,
        details=f"mc0={mc0}, mc1={mc1}",
    )

    # ----- cold/hot tick marks as geometry
    tick_c = ctx.part_element_world_aabb(column, elem="tick_cold")
    tick_h = ctx.part_element_world_aabb(column, elem="tick_hot")
    ctx.check(
        "cold tick mark is visible geometry on the +Y valve cap",
        tick_c is not None
        and 0.5 * (tick_c[0][1] + tick_c[1][1]) > 0.05,
        details=f"tick_cold aabb={tick_c}",
    )
    ctx.check(
        "hot tick mark is visible geometry on the -Y valve cap",
        tick_h is not None
        and 0.5 * (tick_h[0][1] + tick_h[1][1]) < -0.05,
        details=f"tick_hot aabb={tick_h}",
    )

    # ----- grounding and scale
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "faucet grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "vertical column is ~0.04 m diameter",
        shaft is not None and 0.038 <= (shaft[1][0] - shaft[0][0]) <= 0.042,
        details=f"column shaft aabb={shaft}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m",
        spout_aabb is not None and spout_aabb[1][2] >= 0.34,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.10,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- cross valve cylinder
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    cap_0 = ctx.part_element_world_aabb(column, elem="valve_end_cap_0")
    cap_1 = ctx.part_element_world_aabb(column, elem="valve_end_cap_1")
    ctx.check(
        "cross-cylinder is ~0.045 m diameter",
        cross is not None and 0.043 <= (cross[1][2] - cross[0][2]) <= 0.047,
        details=f"cross aabb={cross}",
    )
    ctx.check(
        "valve assembly spans ~0.18 m end-to-end",
        cap_0 is not None
        and cap_1 is not None
        and 0.178 <= (cap_0[1][1] - cap_1[0][1]) <= 0.182,
        details=f"cap_0={cap_0}, cap_1={cap_1}",
    )

    # ----- spray head prismatic joint
    ctx.check(
        "spray_slide is prismatic with 0..0.045 m travel",
        spray_slide.articulation_type == ArticulationType.PRISMATIC
        and spray_slide.motion_limits is not None
        and abs(spray_slide.motion_limits.lower) < 1e-6
        and abs(spray_slide.motion_limits.upper - SPRAY_TRAVEL) < 1e-3,
    )

    # Spray body docks inside the spout tip at rest (proven by overlap on z)
    ctx.expect_overlap(
        spray,
        spout,
        axes="z",
        elem_a="spray_body",
        elem_b="tip_sleeve",
        min_overlap=0.010,
        name="spray body docks inside the tip sleeve at rest",
    )
    ctx.expect_within(
        spray,
        spout,
        axes="xy",
        inner_elem="spray_body",
        outer_elem="tip_sleeve",
        margin=0.005,
        name="spray body stays centered in the tip sleeve",
    )

    # Spray head extends downward when joint is actuated
    rest_spray = ctx.part_world_aabb(spray)
    with ctx.pose({spray_slide: SPRAY_TRAVEL}):
        extended_spray = ctx.part_world_aabb(spray)
    ctx.check(
        "spray head slides downward when extended",
        rest_spray is not None
        and extended_spray is not None
        and extended_spray[0][2] < rest_spray[0][2] - 0.02,
        details=f"rest={rest_spray}, extended={extended_spray}",
    )

    # ----- spout swivel
    ctx.check(
        "spout swivel is revolute about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )

    # ----- lever joints
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0 deg about the valve left-right axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6,
        )

    # ----- lever pose check
    rest_0 = ctx.part_world_aabb(lever_0)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
    ctx.check(
        "lever 0 tilts from vertical toward user at q=-90 deg",
        rest_0 is not None
        and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.08,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )

    # ----- swivel pose check
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout swivel carries the outlet sideways",
        rest_sleeve is not None
        and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 0.01
        and abs(0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1])) > 0.05,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    return ctx.report()


object_model = build_object_model()
