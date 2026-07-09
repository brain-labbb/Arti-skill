from __future__ import annotations

"""Compact single-hole basin faucet with squat oval-pedestal body and swiveling spout.

Layout (meters, +Z up, ground at z=0, spout extends along +X at rest):
- Wide oval pedestal base plate sits on the sink deck.
- A short cylindrical column rises from the pedestal.
- A swivel boss at the column top carries the spout.
- The spout arm cantilevers forward from the boss with subtle grip grooves
  on its top surface and a dark outlet recess near the tip.
- Two small screw caps on the column back face.
- The spout swivels ±60° about a vertical axis through the body top.
"""

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
# Key dimensions (meters)
# ---------------------------------------------------------------------------
# Oval pedestal (wider left-right than front-back)
PEDESTAL_RX = 0.030          # semi-axis along X (front-back depth)
PEDESTAL_RY = 0.040          # semi-axis along Y (left-right width)
PEDESTAL_H = 0.014

# Column
COLUMN_R = 0.020
COLUMN_BOT_Z = PEDESTAL_H    # 0.014
COLUMN_TOP_Z = 0.098
COLUMN_H = COLUMN_TOP_Z - COLUMN_BOT_Z   # 0.084

# Swivel boss (slightly wider than column)
BOSS_R = 0.024
BOSS_H = 0.010
BOSS_TOP_Z = COLUMN_TOP_Z + BOSS_H       # 0.108

# Spout (child part, swivels)
SPOUT_BASE_R = 0.023
SPOUT_BASE_H = 0.008
SPOUT_ARM_W = 0.028           # width (Y)
SPOUT_ARM_H = 0.018           # height (Z)
SPOUT_ARM_LEN = 0.105         # length along X
# Arm center Z in spout-local frame; embed 0.5mm into base for connectivity
SPOUT_ARM_CZ = SPOUT_BASE_H + SPOUT_ARM_H / 2.0 - 0.0005

# Outlet recess on spout underside near tip
OUTLET_R = 0.007
OUTLET_H = 0.005
OUTLET_X = SPOUT_ARM_LEN - 0.015

# Grip grooves — three thin ridges on the spout top
GROOVE_LEN = 0.040
GROOVE_W = 0.002
GROOVE_H = 0.0012
GROOVE_SPACING = 0.007
GROOVE_START_X = 0.025
GROOVE_EMBED = 0.0005         # embed into arm top for connectivity

# Screw caps on the column back face
SCREW_CAP_R = 0.004
SCREW_CAP_H = 0.003
SCREW_CAP_Z1 = 0.048          # upper cap height
SCREW_CAP_Z2 = 0.074          # lower cap height
SCREW_CAP_Y_OFF = 0.008       # left-right offset from center

# Swivel motion
SWIVEL_RANGE = math.radians(60.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    groove_mat = model.material("groove_chrome", rgba=(0.55, 0.57, 0.60, 1.0))
    cap_mat = model.material("cap_chrome", rgba=(0.72, 0.74, 0.78, 1.0))

    # ------------------------------------------------------------------
    # Body (root): pedestal, column, boss, screw caps
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Oval pedestal
    pedestal = (
        cq.Workplane("XY")
        .ellipse(PEDESTAL_RX, PEDESTAL_RY)
        .extrude(PEDESTAL_H)
    )
    body.visual(
        mesh_from_cadquery(pedestal, "pedestal"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="pedestal",
    )

    # Short transition shoulder (slightly tapered cylinder from pedestal to column)
    shoulder_h = 0.008
    shoulder = (
        cq.Workplane("XY")
        .circle(COLUMN_R + 0.004)
        .workplane(offset=shoulder_h)
        .circle(COLUMN_R)
        .loft()
    )
    body.visual(
        mesh_from_cadquery(shoulder, "shoulder"),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H)),
        material=chrome,
        name="shoulder",
    )

    # Column
    col_start_z = PEDESTAL_H + shoulder_h
    col_h = COLUMN_TOP_Z - col_start_z
    body.visual(
        Cylinder(radius=COLUMN_R, length=col_h),
        origin=Origin(xyz=(0.0, 0.0, col_start_z + col_h / 2.0)),
        material=chrome,
        name="column",
    )

    # Swivel boss at column top
    body.visual(
        Cylinder(radius=BOSS_R, length=BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + BOSS_H / 2.0)),
        material=chrome,
        name="swivel_boss",
    )

    # Screw caps on the back (-X) face of the column
    cap_x = -(COLUMN_R + SCREW_CAP_H / 2.0 - 0.001)   # embed 1mm into column
    body.visual(
        Cylinder(radius=SCREW_CAP_R, length=SCREW_CAP_H),
        origin=Origin(
            xyz=(cap_x, SCREW_CAP_Y_OFF, SCREW_CAP_Z1),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cap_mat,
        name="screw_cap_upper",
    )
    body.visual(
        Cylinder(radius=SCREW_CAP_R, length=SCREW_CAP_H),
        origin=Origin(
            xyz=(cap_x, -SCREW_CAP_Y_OFF, SCREW_CAP_Z2),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cap_mat,
        name="screw_cap_lower",
    )

    # ------------------------------------------------------------------
    # Spout (child, swivels about vertical axis at boss top)
    # ------------------------------------------------------------------
    spout = model.part("spout")

    # Base collar sits on the boss
    spout.visual(
        Cylinder(radius=SPOUT_BASE_R, length=SPOUT_BASE_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_BASE_H / 2.0)),
        material=chrome,
        name="spout_base",
    )

    # Arm cantilever
    spout.visual(
        Box((SPOUT_ARM_LEN, SPOUT_ARM_W, SPOUT_ARM_H)),
        origin=Origin(xyz=(SPOUT_ARM_LEN / 2.0, 0.0, SPOUT_ARM_CZ)),
        material=chrome,
        name="spout_arm",
    )

    # Outlet recess on arm underside near tip
    arm_bot_z = SPOUT_ARM_CZ - SPOUT_ARM_H / 2.0
    spout.visual(
        Cylinder(radius=OUTLET_R, length=OUTLET_H),
        origin=Origin(xyz=(OUTLET_X, 0.0, arm_bot_z - OUTLET_H / 2.0 + 0.001)),
        material=dark,
        name="outlet",
    )

    # Grip grooves — three parallel ridges on the spout top surface
    arm_top_z = SPOUT_ARM_CZ + SPOUT_ARM_H / 2.0
    groove_cz = arm_top_z + GROOVE_H / 2.0 - GROOVE_EMBED
    for i, y_off in enumerate([-GROOVE_SPACING, 0.0, GROOVE_SPACING]):
        spout.visual(
            Box((GROOVE_LEN, GROOVE_W, GROOVE_H)),
            origin=Origin(
                xyz=(GROOVE_START_X + GROOVE_LEN / 2.0, y_off, groove_cz)
            ),
            material=groove_mat,
            name=f"grip_groove_{i}",
        )

    # ------------------------------------------------------------------
    # Articulation: spout swivels about vertical axis
    # ------------------------------------------------------------------
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, BOSS_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-SWIVEL_RANGE,
            upper=SWIVEL_RANGE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    spout = object_model.get_part("spout")
    swivel = object_model.get_articulation("spout_swivel")

    # --- Joint plan ---
    ctx.check(
        "spout swivel is revolute about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and abs(swivel.axis[0]) < 1e-9
        and abs(swivel.axis[1]) < 1e-9
        and abs(abs(swivel.axis[2]) - 1.0) < 1e-9,
        details=f"axis={swivel.axis}",
    )
    ctx.check(
        "swivel limits are ±60 degrees",
        swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_RANGE) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_RANGE) < 1e-6,
        details=f"limits={swivel.motion_limits}",
    )

    # --- Oval pedestal geometry ---
    ped_aabb = ctx.part_element_world_aabb(body, elem="pedestal")
    ctx.check(
        "pedestal is oval and wider left-right (Y) than front-back (X)",
        ped_aabb is not None
        and (ped_aabb[1][1] - ped_aabb[0][1]) > (ped_aabb[1][0] - ped_aabb[0][0]) * 1.1,
        details=f"pedestal_aabb={ped_aabb}",
    )

    # --- Squat proportions ---
    body_aabb = ctx.part_world_aabb(body)
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "body is grounded at z≈0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-4,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height is squat (0.10 to 0.16 m)",
        spout_aabb is not None and 0.10 <= spout_aabb[1][2] <= 0.16,
        details=f"spout_aabb max z={None if spout_aabb is None else spout_aabb[1][2]}",
    )

    # --- Screw caps on back of body ---
    cap1_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_upper")
    cap2_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_lower")
    ctx.check(
        "two screw caps are on the back (-X) face of the column",
        cap1_aabb is not None
        and cap2_aabb is not None
        and cap1_aabb[0][0] < -COLUMN_R * 0.4
        and cap2_aabb[0][0] < -COLUMN_R * 0.4
        and abs(cap1_aabb[0][0] - cap2_aabb[0][0]) < 0.005,
        details=f"cap1={cap1_aabb}, cap2={cap2_aabb}",
    )
    ctx.check(
        "screw caps are at different heights on the column",
        cap1_aabb is not None
        and cap2_aabb is not None
        and abs((cap1_aabb[0][2] + cap1_aabb[1][2]) / 2.0
                - (cap2_aabb[0][2] + cap2_aabb[1][2]) / 2.0) > 0.015,
        details=f"cap1_z={(cap1_aabb[0][2]+cap1_aabb[1][2])/2 if cap1_aabb else None}, "
                f"cap2_z={(cap2_aabb[0][2]+cap2_aabb[1][2])/2 if cap2_aabb else None}",
    )

    # --- Grip grooves on spout top ---
    arm_aabb = ctx.part_element_world_aabb(spout, elem="spout_arm")
    g0_aabb = ctx.part_element_world_aabb(spout, elem="grip_groove_0")
    g1_aabb = ctx.part_element_world_aabb(spout, elem="grip_groove_1")
    g2_aabb = ctx.part_element_world_aabb(spout, elem="grip_groove_2")
    ctx.check(
        "three grip grooves sit on top of the spout arm",
        arm_aabb is not None
        and g0_aabb is not None
        and g1_aabb is not None
        and g2_aabb is not None
        and g0_aabb[0][2] > arm_aabb[0][2] + SPOUT_ARM_H * 0.7
        and g1_aabb[0][2] > arm_aabb[0][2] + SPOUT_ARM_H * 0.7
        and g2_aabb[0][2] > arm_aabb[0][2] + SPOUT_ARM_H * 0.7,
        details=f"arm_top={arm_aabb[1][2] if arm_aabb else None}, "
                f"g0_bot={g0_aabb[0][2] if g0_aabb else None}",
    )
    ctx.check(
        "grip grooves are spaced apart in Y",
        g0_aabb is not None
        and g2_aabb is not None
        and abs((g0_aabb[0][1] + g0_aabb[1][1]) / 2.0
                - (g2_aabb[0][1] + g2_aabb[1][1]) / 2.0) > 0.010,
        details=f"g0_y={(g0_aabb[0][1]+g0_aabb[1][1])/2 if g0_aabb else None}, "
                f"g2_y={(g2_aabb[0][1]+g2_aabb[1][1])/2 if g2_aabb else None}",
    )

    # --- Spout mounts on body ---
    ctx.expect_contact(
        spout,
        body,
        elem_a="spout_base",
        elem_b="swivel_boss",
        contact_tol=1e-4,
        name="spout base seats on the swivel boss",
    )

    # --- Outlet is on spout underside ---
    outlet_aabb = ctx.part_element_world_aabb(spout, elem="outlet")
    ctx.check(
        "outlet is recessed on the spout underside near the tip",
        arm_aabb is not None
        and outlet_aabb is not None
        and outlet_aabb[0][2] < arm_aabb[0][2] + 0.002
        and outlet_aabb[1][0] > SPOUT_ARM_LEN * 0.6,
        details=f"outlet={outlet_aabb}, arm={arm_aabb}",
    )

    # --- Decisive pose: swivel rotates spout sideways ---
    rest_spout_aabb = spout_aabb
    with ctx.pose({swivel: SWIVEL_RANGE}):
        swiveled_aabb = ctx.part_world_aabb(spout)
        ctx.check(
            "positive swivel rotates spout tip sideways (+Y)",
            rest_spout_aabb is not None
            and swiveled_aabb is not None
            and swiveled_aabb[1][1] > rest_spout_aabb[1][1] + 0.03,
            details=f"rest={rest_spout_aabb}, swiveled={swiveled_aabb}",
        )

    with ctx.pose({swivel: -SWIVEL_RANGE}):
        neg_aabb = ctx.part_world_aabb(spout)
        ctx.check(
            "negative swivel rotates spout tip to the other side (-Y)",
            rest_spout_aabb is not None
            and neg_aabb is not None
            and neg_aabb[0][1] < rest_spout_aabb[0][1] - 0.03,
            details=f"rest={rest_spout_aabb}, neg_swivel={neg_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
