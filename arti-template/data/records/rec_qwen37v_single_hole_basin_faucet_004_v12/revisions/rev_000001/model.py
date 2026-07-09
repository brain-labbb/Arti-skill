from __future__ import annotations

"""Polished-chrome single-hole basin faucet — compact squat variant.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A wide oval pedestal carries a short rectangular column.
- A flat rectangular spout blade cantilevers forward from the column top, with a
  round aerator outlet recessed in its underside near the tip.
- Above the spout root, a short chrome post carries the lever pivot block, which
  swivels about a vertical axis (temperature, -45..+45 deg).
- The flat rectangular lever handle lifts on a horizontal left-right axis through
  the pivot block (flow, 0..25 deg). The grip surface has subtle lateral grooves.
- A thin drain rod slides vertically behind the body on a prismatic joint
  (pull-up drain, 0..0.035 m travel).
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
PEDESTAL_RX = 0.055  # oval half-extent along X (front-back)
PEDESTAL_RY = 0.040  # oval half-extent along Y (left-right)
PEDESTAL_H = 0.015

COLUMN_DEPTH_X = 0.035
COLUMN_WIDTH_Y = 0.042
COLUMN_TOP_Z = 0.115
COLUMN_H = COLUMN_TOP_Z - PEDESTAL_H  # 0.100

SPOUT_WIDTH_Y = 0.042
SPOUT_THICK_Z = 0.018
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0  # flush with column rear face
SPOUT_TIP_X = 0.140  # ~0.12 m forward reach past column front
SPOUT_TOP_Z = COLUMN_TOP_Z  # blade top flush with column top
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.097

OUTLET_X = 0.122  # aerator center, near the spout tip
AERATOR_OUTER_R = 0.010
AERATOR_INNER_R = 0.007
AERATOR_H = 0.007  # ring height

POST_R = 0.012
POST_H = 0.012
POST_TOP_Z = COLUMN_TOP_Z + POST_H  # 0.127

BLOCK_DEPTH_X = 0.040
BLOCK_WIDTH_Y = 0.040
BLOCK_H = 0.028
BLOCK_TOP_REL = BLOCK_H  # in swivel-child frame (origin at post top)

HANDLE_LEN_X = 0.140
HANDLE_WIDTH_Y = 0.042
HANDLE_THICK_Z = 0.012
HANDLE_FLOAT = 0.0015
HANDLE_REAR_REL_X = -BLOCK_DEPTH_X / 2.0  # blade rear flush with block rear

# Drain rod dimensions
DRAIN_ROD_R = 0.004
DRAIN_ROD_LEN = 0.075
DRAIN_KNOB_R = 0.008
DRAIN_KNOB_H = 0.010
DRAIN_ROD_X = -COLUMN_DEPTH_X / 2.0 - 0.008  # behind the column
DRAIN_ROD_BASE_Z = PEDESTAL_H + 0.008  # rod starts near pedestal top
DRAIN_TRAVEL = 0.035  # vertical travel range

LIFT_RANGE = math.radians(25.0)
SWIVEL_RANGE = math.radians(45.0)


def _build_oval_pedestal():
    """Build the wide oval pedestal as a CadQuery solid."""
    pedestal = (
        cq.Workplane("XY")
        .ellipse(PEDESTAL_RX, PEDESTAL_RY)
        .extrude(PEDESTAL_H)
    )
    return pedestal


def _build_grooved_handle():
    """Build the handle blade with subtle lateral grip grooves."""
    # Start with a solid box
    handle = (
        cq.Workplane("XY")
        .box(HANDLE_LEN_X, HANDLE_WIDTH_Y, HANDLE_THICK_Z)
        .translate((HANDLE_LEN_X / 2.0, 0.0, HANDLE_THICK_Z / 2.0))
    )
    # Cut shallow grooves across the top surface (perpendicular to length)
    # Grooves run along Y, spaced along X, cut from the top face downward
    groove_depth = 0.0015
    groove_width = 0.003
    groove_spacing = 0.012
    # Place grooves along the grip region (forward 60% of handle length)
    groove_start_x = HANDLE_LEN_X * 0.35
    groove_end_x = HANDLE_LEN_X - 0.010
    n_grooves = int((groove_end_x - groove_start_x) / groove_spacing)

    # Build cutters on a fixed global workplane to avoid face-drift
    for i in range(n_grooves):
        gx = groove_start_x + i * groove_spacing
        cutter = (
            cq.Workplane("XY")
            .transformed(offset=(gx, 0.0, HANDLE_THICK_Z - groove_depth / 2.0))
            .box(groove_width, HANDLE_WIDTH_Y + 0.002, groove_depth)
        )
        handle = handle.cut(cutter)

    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    red = model.material("hot_red", rgba=(0.80, 0.08, 0.08, 1.0))
    blue = model.material("cold_blue", rgba=(0.10, 0.30, 0.78, 1.0))
    rod_mat = model.material("rod_chrome", rgba=(0.78, 0.80, 0.84, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: oval pedestal, column, spout blade, aerator, post
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Wide oval pedestal
    body.visual(
        mesh_from_cadquery(_build_oval_pedestal(), "oval_pedestal"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="oval_pedestal",
    )

    # Short rectangular column
    body.visual(
        Box((COLUMN_DEPTH_X, COLUMN_WIDTH_Y, COLUMN_H)),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H + COLUMN_H / 2.0)),
        material=chrome,
        name="column",
    )

    # Spout blade
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    body.visual(
        Box((spout_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z)),
        origin=Origin(
            xyz=((SPOUT_BACK_X + SPOUT_TIP_X) / 2.0, 0.0, SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0)
        ),
        material=chrome,
        name="spout_blade",
    )

    # Hollow chrome aerator ring under the spout tip
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

    # Dark outlet disc recessed inside aerator
    body.visual(
        Cylinder(radius=AERATOR_INNER_R, length=0.005),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.003 + 0.002 + 0.0025)),
        material=dark,
        name="outlet_disc",
    )

    # Mounting post on column top
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
    # Temperature dots on the block front face
    dot_x = BLOCK_DEPTH_X / 2.0
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, 0.007, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="hot_dot",
    )
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, -0.007, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
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
    # Lift stage: flat rectangular lever handle with grip grooves (flow)
    # ------------------------------------------------------------------
    handle = model.part("lever_handle")

    # Grooved handle blade (CadQuery mesh with grooves cut in top surface)
    handle.visual(
        mesh_from_cadquery(_build_grooved_handle(), "handle_grip"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="handle_grip",
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

    # ------------------------------------------------------------------
    # Drain rod: pull-up rod behind the body, prismatic vertical
    # ------------------------------------------------------------------
    drain = model.part("drain_rod")

    # Thin rod shaft
    drain.visual(
        Cylinder(radius=DRAIN_ROD_R, length=DRAIN_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LEN / 2.0)),
        material=rod_mat,
        name="rod_shaft",
    )

    # Small knob on top of the rod
    drain.visual(
        Cylinder(radius=DRAIN_KNOB_R, length=DRAIN_KNOB_H),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LEN + DRAIN_KNOB_H / 2.0)),
        material=chrome,
        name="drain_knob",
    )

    model.articulation(
        "drain_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain,
        origin=Origin(xyz=(DRAIN_ROD_X, 0.0, DRAIN_ROD_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=1.0, lower=0.0, upper=DRAIN_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    block = object_model.get_part("lever_pivot_block")
    handle = object_model.get_part("lever_handle")
    drain = object_model.get_part("drain_rod")
    swivel = object_model.get_articulation("handle_swivel")
    lift = object_model.get_articulation("handle_lift")
    drain_slide = object_model.get_articulation("drain_slide")

    # --- body is squat (not tall like the parent vessel faucet) ---
    body_aabb = ctx.part_world_aabb(body)
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "faucet body is squat — total height well under 0.25 m",
        handle_aabb is not None and handle_aabb[1][2] < 0.25,
        details=f"handle_aabb={handle_aabb}",
    )
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )

    # --- oval pedestal is wider than column ---
    pedestal_aabb = ctx.part_element_world_aabb(body, elem="oval_pedestal")
    column_aabb = ctx.part_element_world_aabb(body, elem="column")
    ctx.check(
        "oval pedestal is wider than column (Y extent)",
        pedestal_aabb is not None
        and column_aabb is not None
        and (pedestal_aabb[1][1] - pedestal_aabb[0][1]) > (column_aabb[1][1] - column_aabb[0][1]) + 0.01,
        details=f"pedestal_aabb={pedestal_aabb}, column_aabb={column_aabb}",
    )
    ctx.check(
        "oval pedestal has oval aspect ratio (X extent > Y extent)",
        pedestal_aabb is not None
        and (pedestal_aabb[1][0] - pedestal_aabb[0][0]) > (pedestal_aabb[1][1] - pedestal_aabb[0][1]) + 0.005,
        details=f"pedestal_aabb={pedestal_aabb}",
    )

    # --- handle has grooved grip (handle_grip visual exists as mesh) ---
    grip_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")
    ctx.check(
        "handle grip visual exists with grooved surface",
        grip_aabb is not None
        and (grip_aabb[1][0] - grip_aabb[0][0]) > 0.10,
        details=f"grip_aabb={grip_aabb}",
    )

    # --- joint plan: lift, swivel, drain slide ---
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
        "drain rod joint is prismatic along Z with 0..0.035 m travel",
        drain_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(drain_slide.axis[0]) < 1e-9
        and abs(drain_slide.axis[1]) < 1e-9
        and abs(abs(drain_slide.axis[2]) - 1.0) < 1e-9
        and drain_slide.motion_limits is not None
        and abs(drain_slide.motion_limits.lower - 0.0) < 1e-9
        and abs(drain_slide.motion_limits.upper - DRAIN_TRAVEL) < 1e-6,
        details=f"axis={drain_slide.axis}, limits={drain_slide.motion_limits}",
    )
    ctx.check(
        "drain rod is parented to the faucet body",
        drain_slide.parent == body.name and drain_slide.child == drain.name,
        details=f"parent={drain_slide.parent}, child={drain_slide.child}",
    )

    # --- drain rod is behind the body column ---
    drain_aabb = ctx.part_world_aabb(drain)
    ctx.check(
        "drain rod sits behind the column (negative X of column center)",
        drain_aabb is not None
        and column_aabb is not None
        and (drain_aabb[0][0] + drain_aabb[1][0]) / 2.0 < (column_aabb[0][0] + column_aabb[1][0]) / 2.0,
        details=f"drain_aabb={drain_aabb}, column_aabb={column_aabb}",
    )

    # --- hero features in place ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="aerator_collar")
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_disc")
    ctx.check(
        "aerator collar sits under the spout tip",
        collar_aabb is not None
        and collar_aabb[0][2] < SPOUT_BOT_Z + 0.001
        and 0.10 <= (collar_aabb[0][0] + collar_aabb[1][0]) / 2.0 <= SPOUT_TIP_X,
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

    # --- mounting: block seats on post, handle floats above block ---
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
        positive_elem="handle_grip",
        negative_elem="pivot_block",
        name="handle grip floats slightly above the pivot block",
    )

    # --- decisive pose checks ---
    # Handle lift raises the grip tip
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

    # Handle swivel slews the handle sideways
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

    # Drain rod slides upward when pulled
    drain_rest_z = drain_aabb[1][2] if drain_aabb is not None else None
    with ctx.pose({drain_slide: DRAIN_TRAVEL}):
        drain_raised_aabb = ctx.part_world_aabb(drain)
        ctx.check(
            "drain rod slides upward when pulled (prismatic +Z)",
            drain_rest_z is not None
            and drain_raised_aabb is not None
            and drain_raised_aabb[1][2] > drain_rest_z + DRAIN_TRAVEL - 0.005,
            details=f"rest_top={drain_rest_z}, raised_aabb={drain_raised_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
