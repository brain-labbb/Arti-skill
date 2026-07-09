from __future__ import annotations

"""Polished-chrome single-lever single-hole basin faucet with squat oval pedestal.

Variant of the tall vessel faucet reworked into a compact basin faucet:
- Wide oval pedestal base replaces the square stepped plate.
- Squat oval body column (~0.11 m total height) replaces the tall rectangular column.
- Shorter spout blade with a hinged aerator flap at the outlet.
- Cartridge cap seam ring visible below the lever mount.
- Subtle grip grooves on the lever handle top surface.

Articulations:
- handle_swivel: revolute about vertical axis through the mounting post (-45..+45 deg, temperature).
- handle_lift: revolute about horizontal left-right axis through the pivot block (0..25 deg, flow).
- aerator_hinge: revolute about horizontal left-right axis at spout tip (0..70 deg, flip-open aerator).
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
# Oval pedestal base
PED_RX = 0.052  # half-width (X, side-to-side)
PED_RY = 0.036  # half-depth (Y, front-to-back)
PED_H = 0.014

# Squat oval body column
BODY_RX = 0.027  # half-width
BODY_RY = 0.022  # half-depth
BODY_TOP_Z = 0.108
BODY_H = BODY_TOP_Z - PED_H  # ~0.094

# Spout blade
SPOUT_WIDTH_Y = 0.042
SPOUT_THICK_Z = 0.018
SPOUT_BACK_X = 0.005  # starts just forward of body center
SPOUT_TIP_X = 0.125  # forward reach
SPOUT_LEN = SPOUT_TIP_X - SPOUT_BACK_X
SPOUT_TOP_Z = BODY_TOP_Z - 0.008  # slightly below body top
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z

# Cartridge cap seam – thin ring around body just below lever area
CAP_Z = BODY_TOP_Z - 0.012
CAP_RING_H = 0.003
CAP_RING_OUTER_RX = BODY_RX + 0.004
CAP_RING_OUTER_RY = BODY_RY + 0.004
CAP_RING_INNER_RX = BODY_RX + 0.001
CAP_RING_INNER_RY = BODY_RY + 0.001

# Mounting post
POST_R = 0.011
POST_H = 0.011
POST_TOP_Z = BODY_TOP_Z + POST_H

# Pivot block (swivel child)
BLOCK_DEPTH_X = 0.040
BLOCK_WIDTH_Y = 0.038
BLOCK_H = 0.028
BLOCK_TOP_REL = BLOCK_H

# Lever handle
HANDLE_LEN_X = 0.140
HANDLE_WIDTH_Y = 0.044
HANDLE_THICK_Z = 0.011
HANDLE_FLOAT = 0.0015
HANDLE_REAR_REL_X = -BLOCK_DEPTH_X / 2.0

# Grip grooves on handle top surface
GROOVE_COUNT = 5
GROOVE_WIDTH = 0.003
GROOVE_DEPTH = 0.001
GROOVE_SPACING = 0.018
GROOVE_LEN = HANDLE_WIDTH_Y * 0.70

# Aerator flap (hinged at spout tip underside)
AERATOR_WIDTH_Y = 0.034
AERATOR_DEPTH_X = 0.022
AERATOR_THICK_Z = 0.004
# Hinge sits at the rear edge of the aerator recess near spout tip
AERATOR_HINGE_X = SPOUT_TIP_X - AERATOR_DEPTH_X
AERATOR_HINGE_Z = SPOUT_BOT_Z

# Dark outlet recess in spout underside (visible when aerator flips open)
OUTLET_R = 0.009
OUTLET_X = SPOUT_TIP_X - AERATOR_DEPTH_X / 2.0

# Motion limits
LIFT_RANGE = math.radians(25.0)
SWIVEL_RANGE = math.radians(45.0)
AERATOR_RANGE = math.radians(70.0)


def _oval_ring(outer_rx, outer_ry, inner_rx, inner_ry, height):
    """Build a thin oval annular ring (hollow oval cylinder) via CadQuery."""
    outer = (
        cq.Workplane("XY")
        .ellipse(outer_rx, outer_ry)
        .extrude(height)
    )
    inner = (
        cq.Workplane("XY")
        .ellipse(inner_rx, inner_ry)
        .extrude(height)
    )
    return outer.cut(inner)


def _oval_cylinder(rx, ry, height):
    """Build a solid oval cylinder via CadQuery."""
    return cq.Workplane("XY").ellipse(rx, ry).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    red = model.material("hot_red", rgba=(0.80, 0.08, 0.08, 1.0))
    blue = model.material("cold_blue", rgba=(0.10, 0.30, 0.78, 1.0))
    seam = model.material("cap_seam", rgba=(0.55, 0.56, 0.58, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: oval pedestal, squat oval column, spout, post, cap seam
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Wide oval pedestal base
    body.visual(
        mesh_from_cadquery(_oval_cylinder(PED_RX, PED_RY, PED_H), "pedestal"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="pedestal",
    )

    # Squat oval body column
    body.visual(
        mesh_from_cadquery(_oval_cylinder(BODY_RX, BODY_RY, BODY_H), "column"),
        origin=Origin(xyz=(0.0, 0.0, PED_H)),
        material=chrome,
        name="column",
    )

    # Spout blade cantilevering forward
    body.visual(
        Box((SPOUT_LEN, SPOUT_WIDTH_Y, SPOUT_THICK_Z)),
        origin=Origin(
            xyz=((SPOUT_BACK_X + SPOUT_TIP_X) / 2.0, 0.0, SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0)
        ),
        material=chrome,
        name="spout_blade",
    )

    # Dark outlet recess in spout underside (visible when aerator opens)
    body.visual(
        Cylinder(radius=OUTLET_R, length=0.004),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z + 0.002)),
        material=dark,
        name="outlet_disc",
    )

    # Cartridge cap seam ring – thin visible groove around body below lever
    cap_ring = _oval_ring(
        CAP_RING_OUTER_RX, CAP_RING_OUTER_RY,
        CAP_RING_INNER_RX, CAP_RING_INNER_RY,
        CAP_RING_H,
    )
    body.visual(
        mesh_from_cadquery(cap_ring, "cartridge_cap"),
        origin=Origin(xyz=(0.0, 0.0, CAP_Z)),
        material=seam,
        name="cartridge_cap",
    )

    # Chrome mounting post on top of body
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z + POST_H / 2.0)),
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
    # Temperature dots on the block side face
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
    # Lift stage: flat lever handle with grip grooves (flow)
    # ------------------------------------------------------------------
    handle = model.part("lever_handle")
    handle.visual(
        Box((HANDLE_LEN_X, HANDLE_WIDTH_Y, HANDLE_THICK_Z)),
        origin=Origin(xyz=(HANDLE_LEN_X / 2.0, 0.0, HANDLE_THICK_Z / 2.0)),
        material=chrome,
        name="handle_blade",
    )
    # Pivot heel under the blade rear
    heel_h = HANDLE_FLOAT + 0.004
    handle.visual(
        Box((0.016, 0.028, heel_h)),
        origin=Origin(xyz=(0.008, 0.0, -HANDLE_FLOAT + heel_h / 2.0)),
        material=chrome,
        name="pivot_heel",
    )

    # Subtle grooves on the grip surface (top face of handle blade)
    groove_start_x = HANDLE_LEN_X * 0.45  # grooves on the grip half
    for i in range(GROOVE_COUNT):
        gx = groove_start_x + i * GROOVE_SPACING
        handle.visual(
            Box((GROOVE_WIDTH, GROOVE_LEN, GROOVE_DEPTH)),
            origin=Origin(
                xyz=(gx, 0.0, HANDLE_THICK_Z + GROOVE_DEPTH / 2.0)
            ),
            material=seam,
            name=f"grip_groove_{i}",
        )

    model.articulation(
        "handle_lift",
        ArticulationType.REVOLUTE,
        parent=block,
        child=handle,
        origin=Origin(xyz=(HANDLE_REAR_REL_X, 0.0, BLOCK_TOP_REL + HANDLE_FLOAT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0, lower=0.0, upper=LIFT_RANGE
        ),
    )

    # ------------------------------------------------------------------
    # Aerator flap: hinged at spout tip underside (flip-open outlet cover)
    # Part frame at hinge line; flap extends forward (+X local) and down.
    # ------------------------------------------------------------------
    aerator = model.part("aerator_flap")
    aerator.visual(
        Box((AERATOR_DEPTH_X, AERATOR_WIDTH_Y, AERATOR_THICK_Z)),
        origin=Origin(xyz=(AERATOR_DEPTH_X / 2.0, 0.0, -AERATOR_THICK_Z / 2.0)),
        material=chrome,
        name="aerator_plate",
    )
    # Small hinge barrel visual at the hinge line
    aerator.visual(
        Cylinder(radius=0.003, length=AERATOR_WIDTH_Y * 0.6),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=seam,
        name="hinge_barrel",
    )

    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(AERATOR_HINGE_X, 0.0, AERATOR_HINGE_Z)),
        # Flap extends along +X from hinge; +Y axis makes positive q flip it downward/open.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=AERATOR_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    block = object_model.get_part("lever_pivot_block")
    handle = object_model.get_part("lever_handle")
    aerator = object_model.get_part("aerator_flap")
    swivel = object_model.get_articulation("handle_swivel")
    lift = object_model.get_articulation("handle_lift")
    aerator_hinge = object_model.get_articulation("aerator_hinge")

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
        "aerator hinge is revolute 0..70 deg about horizontal left-right axis",
        aerator_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(aerator_hinge.axis[0]) < 1e-9
        and abs(aerator_hinge.axis[1] - 1.0) < 1e-9
        and abs(aerator_hinge.axis[2]) < 1e-9
        and aerator_hinge.motion_limits is not None
        and abs(aerator_hinge.motion_limits.lower - 0.0) < 1e-9
        and abs(aerator_hinge.motion_limits.upper - math.radians(70.0)) < 1e-6,
        details=f"axis={aerator_hinge.axis}, limits={aerator_hinge.motion_limits}",
    )
    ctx.check(
        "swivel parents the lift joint (serial chain on the handle)",
        swivel.child == block.name and lift.parent == block.name and lift.child == handle.name,
        details=f"swivel.child={swivel.child}, lift.parent={lift.parent}, lift.child={lift.child}",
    )
    ctx.check(
        "aerator hinge parents from the fixed body",
        aerator_hinge.parent == body.name and aerator_hinge.child == aerator.name,
        details=f"parent={aerator_hinge.parent}, child={aerator_hinge.child}",
    )

    # --- grounding and squat proportions ---
    body_aabb = ctx.part_world_aabb(body)
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "pedestal is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "faucet is squat: total height under 0.18 m",
        handle_aabb is not None and handle_aabb[1][2] < 0.18,
        details=f"handle_aabb={handle_aabb}",
    )
    ctx.check(
        "oval pedestal is wider than tall (squat base)",
        body_aabb is not None
        and (body_aabb[1][0] - body_aabb[0][0]) > 0.08
        and (body_aabb[1][1] - body_aabb[0][1]) > 0.05,
        details=f"body_aabb={body_aabb}",
    )

    # --- cartridge cap seam visible below lever ---
    cap_aabb = ctx.part_element_world_aabb(body, elem="cartridge_cap")
    ctx.check(
        "cartridge cap seam ring sits below the lever mount area",
        cap_aabb is not None
        and cap_aabb[0][2] > BODY_TOP_Z - 0.025
        and cap_aabb[1][2] < BODY_TOP_Z + 0.005,
        details=f"cap_aabb={cap_aabb}",
    )
    ctx.check(
        "cartridge cap ring wraps wider than the body column",
        cap_aabb is not None
        and (cap_aabb[1][0] - cap_aabb[0][0]) > 2.0 * BODY_RX + 0.003,
        details=f"cap_aabb={cap_aabb}",
    )

    # --- grip grooves present on handle ---
    groove_aabbs = [
        ctx.part_element_world_aabb(handle, elem=f"grip_groove_{i}")
        for i in range(GROOVE_COUNT)
    ]
    ctx.check(
        "grip grooves are present on the handle top surface",
        all(a is not None for a in groove_aabbs)
        and all(a[0][2] > HANDLE_THICK_Z * 0.9 for a in groove_aabbs),
        details=f"groove_count={sum(1 for a in groove_aabbs if a is not None)}",
    )

    # --- aerator flap at spout tip ---
    aerator_aabb = ctx.part_world_aabb(aerator)
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_disc")
    ctx.check(
        "aerator flap is positioned near the spout tip underside",
        aerator_aabb is not None
        and aerator_aabb[0][2] < SPOUT_BOT_Z + 0.005
        and aerator_aabb[1][0] > SPOUT_TIP_X - AERATOR_DEPTH_X - 0.01,
        details=f"aerator_aabb={aerator_aabb}",
    )
    ctx.check(
        "dark outlet is recessed in the spout underside behind the aerator",
        outlet_aabb is not None
        and outlet_aabb[0][2] < SPOUT_BOT_Z + 0.006
        and outlet_aabb[1][0] > SPOUT_TIP_X - AERATOR_DEPTH_X - 0.005,
        details=f"outlet_aabb={outlet_aabb}",
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
        positive_elem="handle_blade",
        negative_elem="pivot_block",
        name="handle blade floats slightly above the pivot block",
    )

    # --- decisive pose checks ---
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

    # Aerator hinge: positive q flips the aerator downward (open)
    rest_aerator_z = aerator_aabb[0][2] if aerator_aabb is not None else None
    with ctx.pose({aerator_hinge: AERATOR_RANGE}):
        open_aerator_aabb = ctx.part_world_aabb(aerator)
        ctx.check(
            "positive aerator hinge flips the flap downward (open)",
            rest_aerator_z is not None
            and open_aerator_aabb is not None
            and open_aerator_aabb[0][2] < rest_aerator_z - 0.008,
            details=f"rest_bot={rest_aerator_z}, open_aabb={open_aerator_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
