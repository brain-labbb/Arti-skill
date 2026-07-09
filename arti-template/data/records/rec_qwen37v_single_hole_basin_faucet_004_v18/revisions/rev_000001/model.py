from __future__ import annotations

"""Polished-chrome single-lever tall vessel faucet with strictly rectangular geometry.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A square stepped base plate carries a slim rectangular column.
- A flat rectangular spout blade cantilevers forward from the column top, with a
  round aerator outlet recessed in its underside near the tip.
- Above the spout root, a short chrome post carries the lever pivot block, which
  swivels about a vertical axis (temperature, -45..+45 deg).
- The flat rectangular lever handle lifts on a horizontal left-right axis through
  the pivot block (flow, 0..25 deg).
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
BASE_LOWER_SIDE = 0.090
BASE_LOWER_H = 0.006
BASE_UPPER_SIDE = 0.068
BASE_UPPER_H = 0.012
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H  # 0.018

COLUMN_DEPTH_X = 0.035
COLUMN_WIDTH_Y = 0.045
COLUMN_TOP_Z = 0.235

SPOUT_WIDTH_Y = 0.050
SPOUT_THICK_Z = 0.020
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0  # flush with column rear face
SPOUT_TIP_X = 0.1825  # ~0.17 m forward reach past the column front face
SPOUT_TOP_Z = COLUMN_TOP_Z  # blade top flush with column top
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.215

OUTLET_X = 0.162  # aerator center, near the spout tip
AERATOR_OUTER_R = 0.011
AERATOR_INNER_R = 0.008
AERATOR_H = 0.008  # ring height; protrudes 0.004 below the spout underside

POST_R = 0.013
POST_H = 0.013
POST_TOP_Z = COLUMN_TOP_Z + POST_H  # 0.248 — swivel joint height

BLOCK_DEPTH_X = 0.045
BLOCK_WIDTH_Y = 0.044
BLOCK_H = 0.0365
BLOCK_TOP_REL = BLOCK_H  # in swivel-child frame (origin at post top)

HANDLE_LEN_X = 0.170
HANDLE_WIDTH_Y = 0.050
HANDLE_THICK_Z = 0.013
HANDLE_FLOAT = 0.0015  # blade floats just above the block top
HANDLE_REAR_REL_X = -BLOCK_DEPTH_X / 2.0  # blade rear flush with block rear

LIFT_RANGE = math.radians(25.0)
SWIVEL_RANGE = math.radians(45.0)

# --- Variant 18: raised circular base collar ---
COLLAR_OUTER_R = 0.042
COLLAR_INNER_R = 0.028
COLLAR_H = 0.016
COLLAR_BASE_Z = BASE_TOP_Z  # sits on top of the upper base plate

# --- Variant 18: grip grooves on the handle blade ---
GROOVE_COUNT = 5
GROOVE_WIDTH = 0.002
GROOVE_DEPTH = 0.0012
GROOVE_SPACING = 0.018
GROOVE_START_X = 0.065  # first groove offset from handle origin (rear edge)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_lever_vessel_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    red = model.material("hot_red", rgba=(0.80, 0.08, 0.08, 1.0))
    blue = model.material("cold_blue", rgba=(0.10, 0.30, 0.78, 1.0))
    groove_mat = model.material("grip_groove", rgba=(0.55, 0.57, 0.60, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: stepped base plate, column, spout blade, aerator, post
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
    # Raised circular collar (escutcheon ring) around the column base
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_R)
        .circle(COLLAR_INNER_R)
        .extrude(COLLAR_H)
    )
    body.visual(
        mesh_from_cadquery(collar, "base_collar"),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_BASE_Z)),
        material=chrome,
        name="base_collar",
    )

    column_h = COLUMN_TOP_Z - BASE_TOP_Z
    body.visual(
        Box((COLUMN_DEPTH_X, COLUMN_WIDTH_Y, column_h)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + column_h / 2.0)),
        material=chrome,
        name="column",
    )
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    body.visual(
        Box((spout_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z)),
        origin=Origin(
            xyz=((SPOUT_BACK_X + SPOUT_TIP_X) / 2.0, 0.0, SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0)
        ),
        material=chrome,
        name="spout_blade",
    )
    # Hollow chrome aerator ring under the spout tip (true annulus so the dark
    # outlet stays visible, recessed inside it).
    ring = (
        cq.Workplane("XY")
        .circle(AERATOR_OUTER_R)
        .circle(AERATOR_INNER_R)
        .extrude(AERATOR_H)
    )
    body.visual(
        mesh_from_cadquery(ring, "aerator_collar"),
        # Ring spans z = SPOUT_BOT_Z - 0.004 .. SPOUT_BOT_Z + 0.004
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.004)),
        material=chrome,
        name="aerator_collar",
    )
    body.visual(
        Cylinder(radius=AERATOR_INNER_R, length=0.006),
        # Dark outlet face recessed 1.5 mm above the collar's bottom rim.
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.004 + 0.0025 + 0.003)),
        material=dark,
        name="outlet_disc",
    )
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + POST_H / 2.0)),
        material=chrome,
        name="mounting_post",
    )

    # ------------------------------------------------------------------
    # Swivel stage: lever pivot block on the mounting post (temperature)
    # Child frame sits at the post top, on the column axis.
    # ------------------------------------------------------------------
    block = model.part("lever_pivot_block")
    block.visual(
        Box((BLOCK_DEPTH_X, BLOCK_WIDTH_Y, BLOCK_H)),
        origin=Origin(xyz=(0.0, 0.0, BLOCK_H / 2.0)),
        material=chrome,
        name="pivot_block",
    )
    # Tiny hot/cold temperature dots on the block front face (off-axis markers).
    dot_x = BLOCK_DEPTH_X / 2.0  # dot cylinder centered on the face, half proud
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, 0.007, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="hot_dot",
    )
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, -0.007, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
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
    # Lift stage: flat rectangular lever handle (flow)
    # Joint frame at the blade rear-bottom edge so positive q lifts the
    # forward grip end without digging into the block.
    # ------------------------------------------------------------------
    handle = model.part("lever_handle")
    handle.visual(
        Box((HANDLE_LEN_X, HANDLE_WIDTH_Y, HANDLE_THICK_Z)),
        origin=Origin(xyz=(HANDLE_LEN_X / 2.0, 0.0, HANDLE_THICK_Z / 2.0)),
        material=chrome,
        name="handle_blade",
    )
    # Short pivot heel under the blade rear: it spans the float gap and seats on
    # the block top, carrying the handle. Its bottom face is at the joint height
    # minus HANDLE_FLOAT, so lifting (rotation about the rear edge) only raises it.
    heel_h = HANDLE_FLOAT + 0.004
    handle.visual(
        Box((0.018, 0.030, heel_h)),
        origin=Origin(xyz=(0.009, 0.0, -HANDLE_FLOAT + heel_h / 2.0)),
        material=chrome,
        name="pivot_heel",
    )

    # Variant 18: subtle grooves across the grip surface (top of handle blade)
    groove_len_y = HANDLE_WIDTH_Y - 0.008  # slightly shorter than full width
    groove_z_center = HANDLE_THICK_Z - GROOVE_DEPTH / 2.0
    for i in range(GROOVE_COUNT):
        gx = GROOVE_START_X + i * GROOVE_SPACING
        handle.visual(
            Box((GROOVE_WIDTH, groove_len_y, GROOVE_DEPTH)),
            origin=Origin(xyz=(gx, 0.0, groove_z_center)),
            material=groove_mat,
            name=f"grip_groove_{i}",
        )

    model.articulation(
        "handle_lift",
        ArticulationType.REVOLUTE,
        parent=block,
        child=handle,
        origin=Origin(xyz=(HANDLE_REAR_REL_X, 0.0, BLOCK_TOP_REL + HANDLE_FLOAT)),
        # Blade extends along local +X; -Y makes positive q lift the grip tip.
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

    # --- joint plan: types, axes, ranges ---
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

    # --- grounding and true scale ---
    body_aabb = ctx.part_world_aabb(body)
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height ~0.30 m (handle is the topmost part)",
        handle_aabb is not None and 0.28 <= handle_aabb[1][2] <= 0.32,
        details=f"handle_aabb={handle_aabb}",
    )
    ctx.check(
        "spout blade cantilevers ~0.17 m forward of the column front face",
        body_aabb is not None and 0.16 <= body_aabb[1][0] - COLUMN_DEPTH_X / 2.0 <= 0.19,
        details=f"body max x={None if body_aabb is None else body_aabb[1][0]}",
    )

    # --- Variant 18: raised circular collar around the base ---
    base_collar_aabb = ctx.part_element_world_aabb(body, elem="base_collar")
    ctx.check(
        "raised circular collar sits around the column base",
        base_collar_aabb is not None
        and abs(base_collar_aabb[0][2] - COLLAR_BASE_Z) < 1e-4
        and abs(base_collar_aabb[1][2] - (COLLAR_BASE_Z + COLLAR_H)) < 1e-4,
        details=f"base_collar_aabb={base_collar_aabb}",
    )
    ctx.check(
        "collar outer diameter spans ~0.08 m",
        base_collar_aabb is not None
        and 0.078 <= (base_collar_aabb[1][0] - base_collar_aabb[0][0]) <= 0.088
        and 0.078 <= (base_collar_aabb[1][1] - base_collar_aabb[0][1]) <= 0.088,
        details=f"base_collar_aabb={base_collar_aabb}",
    )

    # --- Variant 18: grip grooves on the handle surface ---
    groove0_aabb = ctx.part_element_world_aabb(handle, elem="grip_groove_0")
    groove4_aabb = ctx.part_element_world_aabb(handle, elem="grip_groove_4")
    ctx.check(
        "handle has subtle grooves on the grip surface",
        groove0_aabb is not None and groove4_aabb is not None,
        details=f"groove0={groove0_aabb}, groove4={groove4_aabb}",
    )
    ctx.check(
        "grooves sit near the top of the handle blade (recessed into surface)",
        groove0_aabb is not None
        and handle_aabb is not None
        and groove0_aabb[1][2] >= handle_aabb[1][2] - 0.003
        and groove0_aabb[0][2] > handle_aabb[0][2] + 0.005,
        details=f"groove0_top={groove0_aabb[1][2] if groove0_aabb else None}, handle_top={handle_aabb[1][2] if handle_aabb else None}",
    )
    ctx.check(
        "grooves are spaced along the handle grip length",
        groove0_aabb is not None
        and groove4_aabb is not None
        and (groove4_aabb[0][0] + groove4_aabb[1][0]) / 2.0
        - (groove0_aabb[0][0] + groove0_aabb[1][0]) / 2.0
        > 0.06,
        details=f"groove0_x={(groove0_aabb[0][0] + groove0_aabb[1][0]) / 2.0 if groove0_aabb else None}, groove4_x={(groove4_aabb[0][0] + groove4_aabb[1][0]) / 2.0 if groove4_aabb else None}",
    )

    # --- hero features in place ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="aerator_collar")
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_disc")
    ctx.check(
        "aerator collar protrudes just below the spout underside near the tip",
        collar_aabb is not None
        and abs(collar_aabb[0][2] - (SPOUT_BOT_Z - 0.004)) < 1e-4
        and 0.14 <= (collar_aabb[0][0] + collar_aabb[1][0]) / 2.0 <= SPOUT_TIP_X,
        details=f"collar_aabb={collar_aabb}",
    )
    ctx.check(
        "dark round outlet is recessed inside the aerator collar",
        collar_aabb is not None
        and outlet_aabb is not None
        and outlet_aabb[0][2] > collar_aabb[0][2] + 0.001
        and outlet_aabb[0][0] > collar_aabb[0][0]
        and outlet_aabb[1][0] < collar_aabb[1][0],
        details=f"outlet_aabb={outlet_aabb}, collar_aabb={collar_aabb}",
    )
    hot_aabb = ctx.part_element_world_aabb(block, elem="hot_dot")
    cold_aabb = ctx.part_element_world_aabb(block, elem="cold_dot")
    ctx.check(
        "red/blue temperature dots sit proud on the pivot block face, off the swivel axis",
        hot_aabb is not None
        and cold_aabb is not None
        and hot_aabb[1][0] > BLOCK_DEPTH_X / 2.0  # proud of the block front face
        and hot_aabb[0][1] > cold_aabb[1][1],  # red and blue are distinct dots
        details=f"hot={hot_aabb}, cold={cold_aabb}",
    )

    # --- mounting: block seats on the post, handle floats just above block ---
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
        min_gap=0.03,
        name="handle assembly stays clear above the fixed spout blade",
    )
    ctx.expect_overlap(
        handle,
        block,
        axes="xy",
        min_overlap=0.02,
        name="handle blade root covers the pivot block footprint",
    )

    # --- decisive pose checks ---
    rest_tip_z = handle_aabb[1][2] if handle_aabb is not None else None
    with ctx.pose({lift: LIFT_RANGE}):
        lifted_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive lift raises the handle grip tip upward",
            rest_tip_z is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_tip_z + 0.04,
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
            and swung_aabb[1][1] > rest_handle_aabb[1][1] + 0.05,
            details=f"rest={rest_handle_aabb}, swung={swung_aabb}",
        )
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            min_gap=0.03,
            name="swiveled handle still clears the fixed spout",
        )

    return ctx.report()


object_model = build_object_model()
