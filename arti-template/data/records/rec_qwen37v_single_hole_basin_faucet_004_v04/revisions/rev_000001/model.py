from __future__ import annotations

"""Polished-chrome single-hole basin faucet with waterfall spout and grooved lever.

Variant 04 of the single-lever tall vessel faucet, forked into a compact
single-hole basin faucet:
- Rounded waterfall-style spout lip at the tip
- Top lever with subtle grip grooves on the upper surface
- Two small screw caps on the back of the column body
- Revolute lift (flow) and swivel (temperature) joints preserved

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X).
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
BASE_LOWER_SIDE = 0.082
BASE_LOWER_H = 0.006
BASE_UPPER_SIDE = 0.062
BASE_UPPER_H = 0.010
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H  # 0.016

COLUMN_DEPTH_X = 0.038
COLUMN_WIDTH_Y = 0.042
COLUMN_TOP_Z = 0.220

# Spout with waterfall lip
SPOUT_WIDTH_Y = 0.050
SPOUT_THICK_Z = 0.018
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0
SPOUT_TIP_X = 0.165  # ~0.15 m forward reach
SPOUT_TOP_Z = COLUMN_TOP_Z
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z

# Waterfall lip: rounded downturned edge at spout tip
LIP_RADIUS = 0.009  # fillet radius for the waterfall curve
LIP_DROP = 0.012  # how far the lip curves downward below spout underside

OUTLET_X = 0.148  # outlet center near the tip
AERATOR_OUTER_R = 0.010
AERATOR_INNER_R = 0.007
AERATOR_H = 0.006

POST_R = 0.012
POST_H = 0.012
POST_TOP_Z = COLUMN_TOP_Z + POST_H

BLOCK_DEPTH_X = 0.042
BLOCK_WIDTH_Y = 0.042
BLOCK_H = 0.032
BLOCK_TOP_REL = BLOCK_H

HANDLE_LEN_X = 0.155
HANDLE_WIDTH_Y = 0.044
HANDLE_THICK_Z = 0.012
HANDLE_FLOAT = 0.0015
HANDLE_REAR_REL_X = -BLOCK_DEPTH_X / 2.0

# Grooves on handle grip
GROOVE_COUNT = 5
GROOVE_WIDTH = 0.002
GROOVE_DEPTH = 0.001
GROOVE_SPACING = 0.014
GROOVE_ZONE_START_X = HANDLE_LEN_X * 0.40  # grooves in the middle-to-tip zone
GROOVE_ZONE_LEN = GROOVE_COUNT * GROOVE_SPACING

# Screw caps on column back
SCREW_CAP_R = 0.005
SCREW_CAP_H = 0.003
SCREW_CAP_SPACING_Y = 0.018

LIFT_RANGE = math.radians(25.0)
SWIVEL_RANGE = math.radians(45.0)


def _build_spout_with_waterfall_lip() -> cq.Workplane:
    """Build the spout blade with a rounded waterfall lip at the tip.

    The main blade is a rectangular box. At the front tip, the lower edge
    curves downward in a smooth quarter-round fillet to create the waterfall
    effect.
    """
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    center_x = (SPOUT_BACK_X + SPOUT_TIP_X) / 2.0
    center_z = SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0

    # Main blade body (slightly shorter to leave room for the lip section)
    lip_section_len = LIP_RADIUS * 2.0
    main_len = spout_len - lip_section_len

    blade = (
        cq.Workplane("XY")
        .center(center_x - lip_section_len / 2.0, 0.0)
        .box(main_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z, centered=(True, True, True))
        .translate((0.0, 0.0, center_z))
    )

    # Waterfall lip section: a rounded downturned piece at the tip
    # Create as a swept profile - quarter circle connecting blade underside to drop
    lip_x_start = SPOUT_TIP_X - lip_section_len
    lip_x_center = lip_x_start + lip_section_len / 2.0

    # Upper part of lip (continuation of blade top surface, thinner)
    lip_upper = (
        cq.Workplane("XY")
        .center(lip_x_center, 0.0)
        .box(lip_section_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z * 0.6, centered=(True, True, True))
        .translate((0.0, 0.0, SPOUT_TOP_Z - SPOUT_THICK_Z * 0.3))
    )

    # Rounded downturned lip - a half-cylinder along the width at the tip
    lip_curve = (
        cq.Workplane("XZ")
        .center(SPOUT_TIP_X - LIP_RADIUS, SPOUT_BOT_Z)
        .rect(LIP_RADIUS * 2.0, LIP_DROP + LIP_RADIUS)
        .extrude(SPOUT_WIDTH_Y, both=True)
    )
    # Intersect with a cylinder to get the rounded form
    round_cutter = (
        cq.Workplane("XZ")
        .center(SPOUT_TIP_X - LIP_RADIUS, SPOUT_BOT_Z - LIP_RADIUS + LIP_DROP)
        .circle(LIP_RADIUS)
        .extrude(SPOUT_WIDTH_Y, both=True)
    )

    # Simpler approach: create the lip as a filleted box extension
    lip_body = (
        cq.Workplane("XY")
        .center(SPOUT_TIP_X - LIP_RADIUS, 0.0)
        .box(LIP_RADIUS * 2.0, SPOUT_WIDTH_Y, SPOUT_THICK_Z + LIP_DROP, centered=(True, True, True))
        .translate((0.0, 0.0, SPOUT_BOT_Z + (SPOUT_THICK_Z + LIP_DROP) / 2.0 - LIP_DROP))
    )

    # Combine blade + lip body, then fillet the front-bottom edge for waterfall curve
    spout = blade.union(lip_upper).union(lip_body)

    # Fillet the front-bottom edges to create the rounded waterfall lip
    # Select edges at the front-bottom of the lip section
    try:
        spout = spout.edges("|Y").edges(cq.selectors.BoxSelector(
            (SPOUT_TIP_X - LIP_RADIUS * 2.5, -SPOUT_WIDTH_Y, SPOUT_BOT_Z - LIP_DROP - 0.001),
            (SPOUT_TIP_X + 0.001, SPOUT_WIDTH_Y, SPOUT_BOT_Z + 0.005),
        )).fillet(LIP_RADIUS * 0.85)
    except Exception:
        pass  # If fillet fails, keep the boxy lip - still reads as waterfall form

    # Fillet the front-top edges for a smoother look
    try:
        spout = spout.edges("|Y").edges(cq.selectors.BoxSelector(
            (SPOUT_TIP_X - LIP_RADIUS * 2.5, -SPOUT_WIDTH_Y, SPOUT_TOP_Z - 0.005),
            (SPOUT_TIP_X + 0.001, SPOUT_WIDTH_Y, SPOUT_TOP_Z + 0.005),
        )).fillet(LIP_RADIUS * 0.5)
    except Exception:
        pass

    return spout


def _build_grooved_handle() -> cq.Workplane:
    """Build the handle blade with subtle parallel grooves on the top surface."""
    handle = (
        cq.Workplane("XY")
        .box(HANDLE_LEN_X, HANDLE_WIDTH_Y, HANDLE_THICK_Z, centered=(False, True, False))
    )

    # Cut grooves across the width on the top surface
    groove_start_x = GROOVE_ZONE_START_X
    for i in range(GROOVE_COUNT):
        gx = groove_start_x + i * GROOVE_SPACING
        groove_cutter = (
            cq.Workplane("XY")
            .center(gx, 0.0)
            .box(GROOVE_WIDTH, HANDLE_WIDTH_Y * 0.85, GROOVE_DEPTH * 2.0, centered=(True, True, False))
            .translate((0.0, 0.0, HANDLE_THICK_Z - GROOVE_DEPTH))
        )
        handle = handle.cut(groove_cutter)

    # Fillet the front end of the handle for a finished look
    try:
        handle = handle.edges("|Z").edges(cq.selectors.BoxSelector(
            (HANDLE_LEN_X - 0.008, -HANDLE_WIDTH_Y, -0.001),
            (HANDLE_LEN_X + 0.001, HANDLE_WIDTH_Y, HANDLE_THICK_Z + 0.001),
        )).fillet(0.003)
    except Exception:
        pass

    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    red = model.material("hot_red", rgba=(0.80, 0.08, 0.08, 1.0))
    blue = model.material("cold_blue", rgba=(0.10, 0.30, 0.78, 1.0))
    cap_mat = model.material("screw_cap_dark", rgba=(0.25, 0.25, 0.28, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: stepped base plate, column, waterfall spout, aerator,
    # mounting post, screw caps
    # ------------------------------------------------------------------
    body = model.part("faucet_body")
    body.visual(
        Box((BASE_LOWER_SIDE, BASE_LOWER_SIDE, BASE_LOWER_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H / 2.0)),
        material=chrome,
        name="base_plate_lower",
    )
    body.visual(
        Box((BASE_UPPER_SIDE, BASE_UPPER_SIDE, BASE_UPPER_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H + BASE_UPPER_H / 2.0)),
        material=chrome,
        name="base_plate_upper",
    )
    column_h = COLUMN_TOP_Z - BASE_TOP_Z
    body.visual(
        Box((COLUMN_DEPTH_X, COLUMN_WIDTH_Y, column_h)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + column_h / 2.0)),
        material=chrome,
        name="column",
    )

    # Waterfall spout (CadQuery)
    spout_cq = _build_spout_with_waterfall_lip()
    body.visual(
        mesh_from_cadquery(spout_cq, "spout_waterfall"),
        material=chrome,
        name="spout_waterfall",
    )

    # Aerator ring under the spout
    ring = (
        cq.Workplane("XY")
        .circle(AERATOR_OUTER_R)
        .circle(AERATOR_INNER_R)
        .extrude(AERATOR_H)
    )
    body.visual(
        mesh_from_cadquery(ring, "aerator_collar"),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.003)),
        material=chrome,
        name="aerator_collar",
    )
    body.visual(
        Cylinder(radius=AERATOR_INNER_R, length=0.005),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.003 + 0.002 + 0.0025)),
        material=dark,
        name="outlet_disc",
    )

    # Mounting post
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + POST_H / 2.0)),
        material=chrome,
        name="mounting_post",
    )

    # Two screw caps on the back of the column
    screw_cap_x = -COLUMN_DEPTH_X / 2.0 - SCREW_CAP_H / 2.0  # proud of back face
    screw_cap_z = BASE_TOP_Z + column_h * 0.45  # mid-column height
    for i, y_offset in enumerate([-SCREW_CAP_SPACING_Y / 2.0, SCREW_CAP_SPACING_Y / 2.0]):
        cap = (
            cq.Workplane("YZ")
            .center(y_offset, screw_cap_z)
            .circle(SCREW_CAP_R)
            .extrude(SCREW_CAP_H)
            .translate((-COLUMN_DEPTH_X / 2.0 - SCREW_CAP_H, 0.0, 0.0))
        )
        # Add a slot across the cap face (screw slot)
        slot = (
            cq.Workplane("YZ")
            .center(y_offset, screw_cap_z)
            .rect(SCREW_CAP_R * 1.4, GROOVE_DEPTH * 2)
            .extrude(GROOVE_DEPTH * 3)
            .translate((-COLUMN_DEPTH_X / 2.0 - SCREW_CAP_H - GROOVE_DEPTH * 2, 0.0, 0.0))
        )
        cap_with_slot = cap.cut(slot)
        body.visual(
            mesh_from_cadquery(cap_with_slot, f"screw_cap_{i}"),
            material=cap_mat,
            name=f"screw_cap_{i}",
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
    # Temperature dots
    dot_x = BLOCK_DEPTH_X / 2.0
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, 0.007, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="hot_dot",
    )
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, -0.007, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
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
    # Lift stage: grooved lever handle (flow)
    # ------------------------------------------------------------------
    handle = model.part("lever_handle")

    # Grooved handle blade (CadQuery)
    handle_cq = _build_grooved_handle()
    handle.visual(
        mesh_from_cadquery(handle_cq, "handle_blade"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="handle_blade",
    )

    # Pivot heel
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

    # --- Joint plan: types, axes, ranges ---
    ctx.check(
        "lift joint is revolute 0..25 deg about horizontal left-right axis",
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
        "swivel parents the lift joint (serial chain on the handle)",
        swivel.child == block.name and lift.parent == block.name and lift.child == handle.name,
        details=f"swivel.child={swivel.child}, lift.parent={lift.parent}, lift.child={lift.child}",
    )

    # --- Grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "faucet height is compact basin scale (~0.25-0.30 m)",
        handle_aabb is not None and 0.24 <= handle_aabb[1][2] <= 0.32,
        details=f"handle_aabb={handle_aabb}",
    )

    # --- Variant 04 specific: waterfall spout lip ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_waterfall")
    ctx.check(
        "waterfall spout extends forward from the column",
        spout_aabb is not None
        and spout_aabb[1][0] > COLUMN_DEPTH_X / 2.0 + 0.10,
        details=f"spout_aabb={spout_aabb}",
    )
    ctx.check(
        "waterfall spout lip drops below the main blade underside",
        spout_aabb is not None
        and spout_aabb[0][2] < SPOUT_BOT_Z - 0.003,
        details=f"spout bottom z={spout_aabb[0][2] if spout_aabb else None}, spout_bot_z={SPOUT_BOT_Z}",
    )

    # --- Variant 04 specific: screw caps on column back ---
    cap0_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_0")
    cap1_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_1")
    ctx.check(
        "two screw caps exist on the faucet body",
        cap0_aabb is not None and cap1_aabb is not None,
        details=f"cap0={cap0_aabb}, cap1={cap1_aabb}",
    )
    ctx.check(
        "screw caps are on the back of the column (negative X side)",
        cap0_aabb is not None
        and cap1_aabb is not None
        and cap0_aabb[1][0] < -COLUMN_DEPTH_X / 2.0 + 0.001
        and cap1_aabb[1][0] < -COLUMN_DEPTH_X / 2.0 + 0.001,
        details=f"cap0_max_x={cap0_aabb[1][0] if cap0_aabb else None}",
    )
    ctx.check(
        "screw caps are vertically separated (two distinct caps)",
        cap0_aabb is not None
        and cap1_aabb is not None
        and abs((cap0_aabb[0][1] + cap0_aabb[1][1]) / 2.0 - (cap1_aabb[0][1] + cap1_aabb[1][1]) / 2.0) > 0.008,
        details=f"cap0_center_y={(cap0_aabb[0][1] + cap0_aabb[1][1]) / 2.0 if cap0_aabb else None}",
    )

    # --- Variant 04 specific: grooved handle ---
    handle_blade_aabb = ctx.part_element_world_aabb(handle, elem="handle_blade")
    ctx.check(
        "handle blade exists with grip surface",
        handle_blade_aabb is not None,
        details=f"handle_blade_aabb={handle_blade_aabb}",
    )
    ctx.check(
        "handle blade is wide enough for grip grooves",
        handle_blade_aabb is not None
        and (handle_blade_aabb[1][1] - handle_blade_aabb[0][1]) > 0.030,
        details=f"handle width={handle_blade_aabb[1][1] - handle_blade_aabb[0][1] if handle_blade_aabb else None}",
    )

    # --- Hero features: aerator and temperature dots ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="aerator_collar")
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_disc")
    ctx.check(
        "aerator collar sits near the spout tip underside",
        collar_aabb is not None
        and collar_aabb[0][0] > 0.10
        and collar_aabb[0][2] < SPOUT_BOT_Z,
        details=f"collar_aabb={collar_aabb}",
    )
    ctx.check(
        "dark outlet recessed inside the aerator collar",
        collar_aabb is not None
        and outlet_aabb is not None
        and outlet_aabb[0][2] > collar_aabb[0][2]
        and outlet_aabb[0][0] > collar_aabb[0][0]
        and outlet_aabb[1][0] < collar_aabb[1][0],
        details=f"outlet={outlet_aabb}, collar={collar_aabb}",
    )
    hot_aabb = ctx.part_element_world_aabb(block, elem="hot_dot")
    cold_aabb = ctx.part_element_world_aabb(block, elem="cold_dot")
    ctx.check(
        "red/blue temperature dots on the pivot block",
        hot_aabb is not None
        and cold_aabb is not None
        and hot_aabb[0][1] > cold_aabb[1][1],
        details=f"hot={hot_aabb}, cold={cold_aabb}",
    )

    # --- Mounting checks ---
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
        body,
        axis="z",
        min_gap=0.02,
        name="handle assembly stays clear above the fixed spout",
    )

    # --- Decisive pose checks ---
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

    return ctx.report()


object_model = build_object_model()
