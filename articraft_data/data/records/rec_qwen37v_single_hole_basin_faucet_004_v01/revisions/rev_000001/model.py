from __future__ import annotations

"""Polished-chrome single-hole basin faucet with tall straight tower and short forward spout.

Variant of the single-lever vessel faucet, redesigned as a compact single-hole
basin faucet with:
- An oval base gasket seating the faucet on the countertop
- A taller straight rectangular tower column
- A short forward-cantilevered spout with a real hollow outlet cut through its underside
- A top lever that lifts (flow) and swivels (temperature) on revolute joints

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- Oval rubber gasket at z=0, chrome base plate on top
- Slim rectangular column rises to ~0.24 m
- Short spout blade extends ~0.10 m forward from column top
- Hollow outlet hole cut through the spout underside near the tip
- Chrome post + pivot block + flat lever handle on top
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

# ----------------------------------------------------------------------------
# Key dimensions (meters)
# ----------------------------------------------------------------------------
# Oval base gasket
GASKET_RX = 0.038  # semi-major axis (X direction)
GASKET_RY = 0.028  # semi-minor axis (Y direction)
GASKET_H = 0.004

# Chrome base plate (round, sits on gasket)
BASE_R = 0.030
BASE_H = 0.012
BASE_TOP_Z = GASKET_H + BASE_H  # 0.016

# Tall straight column
COLUMN_DEPTH_X = 0.038
COLUMN_WIDTH_Y = 0.032
COLUMN_TOP_Z = 0.240

# Short forward spout
SPOUT_WIDTH_Y = 0.040
SPOUT_THICK_Z = 0.018
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0  # flush with column rear
SPOUT_TIP_X = 0.110  # short forward reach (~0.10 m past column front face)
SPOUT_TOP_Z = COLUMN_TOP_Z
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.222

# Hollow outlet — a real hole cut through the spout underside
OUTLET_X = 0.095  # outlet center X, near the spout tip
OUTLET_R = 0.009  # outlet hole radius

# Chrome mounting post on column top
POST_R = 0.012
POST_H = 0.012
POST_TOP_Z = COLUMN_TOP_Z + POST_H  # 0.252

# Pivot block (swivel child)
BLOCK_DEPTH_X = 0.040
BLOCK_WIDTH_Y = 0.040
BLOCK_H = 0.032
BLOCK_TOP_REL = BLOCK_H

# Lever handle
HANDLE_LEN_X = 0.140
HANDLE_WIDTH_Y = 0.042
HANDLE_THICK_Z = 0.012
HANDLE_FLOAT = 0.0015
HANDLE_REAR_REL_X = -BLOCK_DEPTH_X / 2.0

# Joint ranges
LIFT_RANGE = math.radians(25.0)
SWIVEL_RANGE = math.radians(45.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("gasket_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    red = model.material("hot_red", rgba=(0.80, 0.08, 0.08, 1.0))
    blue = model.material("cold_blue", rgba=(0.10, 0.30, 0.78, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: gasket, base plate, column, spout (with hollow outlet), post
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Oval base gasket — CadQuery ellipse extrusion
    gasket = (
        cq.Workplane("XY")
        .ellipse(GASKET_RX, GASKET_RY)
        .extrude(GASKET_H)
    )
    body.visual(
        mesh_from_cadquery(gasket, "base_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rubber,
        name="base_gasket",
    )

    # Chrome base plate (round disk on top of gasket)
    body.visual(
        Cylinder(radius=BASE_R, length=BASE_H),
        origin=Origin(xyz=(0.0, 0.0, GASKET_H + BASE_H / 2.0)),
        material=chrome,
        name="base_plate",
    )

    # Tall straight column
    column_h = COLUMN_TOP_Z - BASE_TOP_Z
    body.visual(
        Box((COLUMN_DEPTH_X, COLUMN_WIDTH_Y, column_h)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + column_h / 2.0)),
        material=chrome,
        name="column",
    )

    # Short forward spout — CadQuery box with a hollow outlet hole cut through
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    spout_solid = (
        cq.Workplane("XY")
        .box(spout_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z)
    )
    # Cut a through-hole for the outlet (vertical cylinder through the full thickness)
    outlet_cutter = (
        cq.Workplane("XY")
        .pushPoints([(OUTLET_X - (SPOUT_BACK_X + SPOUT_TIP_X) / 2.0, 0.0)])
        .circle(OUTLET_R)
        .extrude(SPOUT_THICK_Z + 0.01, both=True)
    )
    spout_with_hole = spout_solid.cut(outlet_cutter)

    body.visual(
        mesh_from_cadquery(spout_with_hole, "spout_blade"),
        origin=Origin(
            xyz=((SPOUT_BACK_X + SPOUT_TIP_X) / 2.0, 0.0, SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0)
        ),
        material=chrome,
        name="spout_blade",
    )

    # Dark recessed ring inside the outlet hole (aerator hint)
    aerator_ring = (
        cq.Workplane("XY")
        .circle(OUTLET_R)
        .circle(OUTLET_R - 0.003)
        .extrude(0.004)
    )
    body.visual(
        mesh_from_cadquery(aerator_ring, "outlet_aerator"),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.002)),
        material=dark,
        name="outlet_aerator",
    )

    # Chrome mounting post
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + POST_H / 2.0)),
        material=chrome,
        name="mounting_post",
    )

    # ------------------------------------------------------------------
    # Swivel stage: lever pivot block on the mounting post (temperature)
    # ------------------------------------------------------------------
    block = model.part("lever_pivot_block")
    block.visual(
        Box((BLOCK_DEPTH_X, BLOCK_WIDTH_Y, BLOCK_H)),
        origin=Origin(xyz=(0.0, 0.0, BLOCK_H / 2.0)),
        material=chrome,
        name="pivot_block",
    )
    # Temperature indicator dots on the block side face
    dot_x = BLOCK_DEPTH_X / 2.0
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, 0.006, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="hot_dot",
    )
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, -0.006, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="cold_dot",
    )

    model.articulation(
        "handle_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=block,
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=3.0, lower=-SWIVEL_RANGE, upper=SWIVEL_RANGE
        ),
    )

    # ------------------------------------------------------------------
    # Lift stage: flat rectangular lever handle (flow control)
    # ------------------------------------------------------------------
    handle = model.part("lever_handle")
    handle.visual(
        Box((HANDLE_LEN_X, HANDLE_WIDTH_Y, HANDLE_THICK_Z)),
        origin=Origin(xyz=(HANDLE_LEN_X / 2.0, 0.0, HANDLE_THICK_Z / 2.0)),
        material=chrome,
        name="handle_blade",
    )
    # Short pivot heel under the blade rear
    heel_h = HANDLE_FLOAT + 0.004
    handle.visual(
        Box((0.016, 0.028, heel_h)),
        origin=Origin(xyz=(0.008, 0.0, -HANDLE_FLOAT + heel_h / 2.0)),
        material=chrome,
        name="pivot_heel",
    )

    model.articulation(
        "handle_lift",
        ArticulationType.REVOLUTE,
        parent=block,
        child=handle,
        origin=Origin(xyz=(HANDLE_REAR_REL_X, 0.0, BLOCK_TOP_REL + HANDLE_FLOAT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=LIFT_RANGE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    block = object_model.get_part("lever_pivot_block")
    handle = object_model.get_part("lever_handle")
    swivel = object_model.get_articulation("handle_swivel")
    lift = object_model.get_articulation("handle_lift")

    # --- joint plan: at least one non-fixed revolute joint ---
    ctx.check(
        "lift joint is revolute 0..25 deg about horizontal axis",
        lift.articulation_type == ArticulationType.REVOLUTE
        and abs(lift.axis[0]) < 1e-9
        and abs(abs(lift.axis[1]) - 1.0) < 1e-9
        and abs(lift.axis[2]) < 1e-9
        and lift.motion_limits is not None
        and abs(lift.motion_limits.lower - 0.0) < 1e-9
        and abs(lift.motion_limits.upper - math.radians(25.0)) < 1e-6,
        details=f"axis={lift.axis}, limits={lift.motion_limits}",
    )
    ctx.check(
        "swivel joint is revolute -45..+45 deg about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and abs(swivel.axis[0]) < 1e-9
        and abs(swivel.axis[1]) < 1e-9
        and abs(abs(swivel.axis[2]) - 1.0) < 1e-9
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + math.radians(45.0)) < 1e-6
        and abs(swivel.motion_limits.upper - math.radians(45.0)) < 1e-6,
        details=f"axis={swivel.axis}, limits={swivel.motion_limits}",
    )
    ctx.check(
        "serial chain: body -> swivel -> block -> lift -> handle",
        swivel.child == block.name and lift.parent == block.name and lift.child == handle.name,
        details=f"swivel.child={swivel.child}, lift.parent={lift.parent}, lift.child={lift.child}",
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "gasket is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-4,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height is in basin-faucet range (0.26..0.32 m)",
        handle_aabb is not None and 0.26 <= handle_aabb[1][2] <= 0.32,
        details=f"handle_aabb={handle_aabb}",
    )

    # --- tall tower column ---
    ctx.check(
        "body top (post) is at ~0.252 m, implying tall column to ~0.24 m",
        body_aabb is not None and abs(body_aabb[1][2] - POST_TOP_Z) < 0.005,
        details=f"body_aabb={body_aabb}",
    )

    # --- short forward spout ---
    ctx.check(
        "spout extends ~0.10 m forward of column front face (short reach)",
        body_aabb is not None and 0.08 <= body_aabb[1][0] - COLUMN_DEPTH_X / 2.0 <= 0.12,
        details=f"body max x={body_aabb[1][0] if body_aabb else None}",
    )

    # --- oval base gasket exists and is wider in X than Y ---
    gasket_aabb = ctx.part_element_world_aabb(body, elem="base_gasket")
    ctx.check(
        "oval base gasket exists at ground level with rx > ry",
        gasket_aabb is not None
        and abs(gasket_aabb[0][2]) < 1e-4
        and (gasket_aabb[1][0] - gasket_aabb[0][0]) > (gasket_aabb[1][1] - gasket_aabb[0][1]),
        details=f"gasket_aabb={gasket_aabb}",
    )

    # --- hollow outlet at spout mouth ---
    aerator_aabb = ctx.part_element_world_aabb(body, elem="outlet_aerator")
    ctx.check(
        "outlet aerator ring sits at spout underside near the tip",
        aerator_aabb is not None
        and abs(aerator_aabb[0][2] - (SPOUT_BOT_Z - 0.002)) < 1e-3
        and aerator_aabb[0][0] > COLUMN_DEPTH_X / 2.0,
        details=f"aerator_aabb={aerator_aabb}",
    )

    # --- mounting contacts ---
    ctx.expect_contact(
        block,
        body,
        elem_a="pivot_block",
        elem_b="mounting_post",
        contact_tol=1e-5,
        name="pivot block seats on the chrome mounting post",
    )
    ctx.expect_contact(
        handle,
        block,
        elem_a="pivot_heel",
        elem_b="pivot_block",
        contact_tol=1e-5,
        name="handle pivot heel seats on the pivot block top",
    )
    ctx.expect_gap(
        handle,
        block,
        axis="z",
        min_gap=0.0005,
        max_gap=0.004,
        positive_elem="handle_blade",
        negative_elem="pivot_block",
        name="handle blade floats slightly above the pivot block",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="z",
        min_gap=0.02,
        name="handle assembly stays clear above the fixed spout blade",
    )

    # --- decisive pose: lift raises handle ---
    rest_tip_z = handle_aabb[1][2] if handle_aabb is not None else None
    with ctx.pose({lift: LIFT_RANGE}):
        lifted_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive lift raises the handle grip tip upward",
            rest_tip_z is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_tip_z + 0.02,
            details=f"rest_top={rest_tip_z}, lifted_aabb={lifted_aabb}",
        )
        ctx.expect_gap(
            handle,
            block,
            axis="z",
            max_penetration=0.0,
            name="lifted handle does not dig into the pivot block",
        )

    # --- decisive pose: swivel slews handle sideways ---
    rest_handle_aabb = handle_aabb
    with ctx.pose({swivel: SWIVEL_RANGE}):
        swung_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive swivel slews the handle sideways about the vertical post axis",
            rest_handle_aabb is not None
            and swung_aabb is not None
            and swung_aabb[1][1] > rest_handle_aabb[1][1] + 0.03,
            details=f"rest={rest_handle_aabb}, swung={swung_aabb}",
        )
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            min_gap=0.02,
            name="swiveled handle still clears the fixed spout",
        )

    return ctx.report()


object_model = build_object_model()
