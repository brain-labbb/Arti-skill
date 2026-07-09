from __future__ import annotations

"""Polished-chrome single-hole basin faucet with rectangular geometry and swiveling spout.

Variant 08: forked from the polished-chrome single-lever tall vessel faucet.
Changes from parent:
- Raised circular collar around the base.
- Spout swivels around the vertical body axis (independent spout rotation).
- Subtle grip grooves on the lever handle surface.
- Thin cartridge cap seam ring below the lever pivot block.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A square stepped base plate carries a raised circular collar.
- A rectangular column rises from the collar; a cylindrical bearing section
  caps the column top for the spout swivel.
- A flat rectangular spout blade with an annular bearing ring cantilevers
  forward from the bearing section and swivels about the vertical axis.
- A thin cartridge cap seam ring sits above the bearing section.
- A chrome mounting post carries the lever pivot block above the cap.
- The flat rectangular lever handle lifts on a horizontal axis (flow) and
  the pivot block swivels on the vertical axis (temperature).
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

# Circular collar around the base
COLLAR_OUTER_R = 0.050
COLLAR_INNER_R = 0.022  # smaller than column half-width (0.0225) so collar grips column
COLLAR_H = 0.018
COLLAR_BOT_Z = BASE_TOP_Z  # 0.018
COLLAR_TOP_Z = COLLAR_BOT_Z + COLLAR_H  # 0.036

# Rectangular column (extends from base plate top through collar zone)
COLUMN_DEPTH_X = 0.035
COLUMN_WIDTH_Y = 0.045
COLUMN_RECT_BOT_Z = BASE_TOP_Z  # 0.018 — contacts upper base plate directly
COLUMN_RECT_TOP_Z = 0.208

# Cylindrical bearing section (upper column, for spout swivel)
BEARING_R = 0.028
BEARING_BOT_Z = COLUMN_RECT_TOP_Z  # 0.208
BEARING_TOP_Z = 0.233
BEARING_H = BEARING_TOP_Z - BEARING_BOT_Z  # 0.025
BEARING_CENTER_Z = (BEARING_BOT_Z + BEARING_TOP_Z) / 2.0  # 0.2205

# Mounting post
POST_R = 0.013
POST_H = 0.013

# Cartridge cap seam (thin ring above bearing, below lever assembly)
CARTRIDGE_R = BEARING_R + 0.004  # 0.032
CARTRIDGE_INNER_R = POST_R - 0.002  # 0.011 — grips the post for connectivity
CARTRIDGE_H = 0.003
CARTRIDGE_BOT_Z = BEARING_TOP_Z  # 0.233
CARTRIDGE_TOP_Z = CARTRIDGE_BOT_Z + CARTRIDGE_H  # 0.236
POST_BOT_Z = CARTRIDGE_TOP_Z  # 0.236
POST_TOP_Z = POST_BOT_Z + POST_H  # 0.249

# Spout bearing ring (child part, wraps around bearing section with slight interference fit)
SPOUT_BEARING_INNER_R = BEARING_R - 0.001  # 0.027 — slight interference for bearing contact
SPOUT_BEARING_OUTER_R = 0.039
SPOUT_BEARING_H = BEARING_H  # 0.025

# Spout blade
SPOUT_WIDTH_Y = 0.050
SPOUT_THICK_Z = 0.020
SPOUT_BLADE_START_X = SPOUT_BEARING_OUTER_R  # 0.039
SPOUT_TIP_X = 0.1825
SPOUT_BLADE_LEN = SPOUT_TIP_X - SPOUT_BLADE_START_X  # ~0.1435

# Aerator / outlet (in spout local frame)
OUTLET_X = 0.162
AERATOR_OUTER_R = 0.011
AERATOR_INNER_R = 0.008
AERATOR_H = 0.008

# Pivot block
BLOCK_DEPTH_X = 0.045
BLOCK_WIDTH_Y = 0.044
BLOCK_H = 0.0365
BLOCK_TOP_REL = BLOCK_H

# Handle
HANDLE_LEN_X = 0.170
HANDLE_WIDTH_Y = 0.050
HANDLE_THICK_Z = 0.013
HANDLE_FLOAT = 0.0015
HANDLE_REAR_REL_X = -BLOCK_DEPTH_X / 2.0

# Grip grooves on handle
GROOVE_COUNT = 5
GROOVE_WIDTH_X = 0.003
GROOVE_DEPTH_Y = HANDLE_WIDTH_Y - 0.006  # slightly narrower than handle
GROOVE_HEIGHT_Z = 0.0018
GROOVE_START_X = 0.070  # first groove x position in handle local frame
GROOVE_SPACING_X = 0.018

# Motion limits
LIFT_RANGE = math.radians(25.0)
SWIVEL_RANGE = math.radians(45.0)
SPOUT_SWIVEL_RANGE = math.radians(90.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    red = model.material("hot_red", rgba=(0.80, 0.08, 0.08, 1.0))
    blue = model.material("cold_blue", rgba=(0.10, 0.30, 0.78, 1.0))
    groove_mat = model.material("grip_groove", rgba=(0.70, 0.72, 0.75, 1.0))
    seam_mat = model.material("cartridge_seam", rgba=(0.75, 0.77, 0.80, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: base plates, collar, column, bearing, cartridge cap, post
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

    # Raised circular collar around the base
    collar_ring = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_R)
        .circle(COLLAR_INNER_R)
        .extrude(COLLAR_H)
    )
    body.visual(
        mesh_from_cadquery(collar_ring, "base_collar"),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_BOT_Z)),
        material=chrome,
        name="base_collar",
    )

    # Rectangular column section
    column_h = COLUMN_RECT_TOP_Z - COLUMN_RECT_BOT_Z
    body.visual(
        Box((COLUMN_DEPTH_X, COLUMN_WIDTH_Y, column_h)),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_RECT_BOT_Z + column_h / 2.0)),
        material=chrome,
        name="column",
    )

    # Cylindrical bearing section (upper column for spout swivel)
    body.visual(
        Cylinder(radius=BEARING_R, length=BEARING_H),
        origin=Origin(xyz=(0.0, 0.0, BEARING_BOT_Z + BEARING_H / 2.0)),
        material=chrome,
        name="bearing_section",
    )

    # Cartridge cap seam ring (thin visible seam below the lever)
    cartridge_ring = (
        cq.Workplane("XY")
        .circle(CARTRIDGE_R)
        .circle(CARTRIDGE_INNER_R)
        .extrude(CARTRIDGE_H)
    )
    body.visual(
        mesh_from_cadquery(cartridge_ring, "cartridge_cap"),
        origin=Origin(xyz=(0.0, 0.0, CARTRIDGE_BOT_Z)),
        material=seam_mat,
        name="cartridge_cap",
    )

    # Mounting post
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, 0.0, POST_BOT_Z + POST_H / 2.0)),
        material=chrome,
        name="mounting_post",
    )

    # ------------------------------------------------------------------
    # Spout: swivels around the vertical body axis on the bearing section
    # Child frame origin at the bearing center so the ring aligns at q=0.
    # ------------------------------------------------------------------
    spout = model.part("spout")

    # Annular bearing ring (wraps around the bearing section)
    spout_ring = (
        cq.Workplane("XY")
        .circle(SPOUT_BEARING_OUTER_R)
        .circle(SPOUT_BEARING_INNER_R)
        .extrude(SPOUT_BEARING_H)
    )
    spout.visual(
        mesh_from_cadquery(spout_ring, "spout_bearing_ring"),
        # Ring bottom at local z = -SPOUT_BEARING_H/2
        origin=Origin(xyz=(0.0, 0.0, -SPOUT_BEARING_H / 2.0)),
        material=chrome,
        name="spout_bearing_ring",
    )

    # Rectangular spout blade extending forward from the bearing ring
    blade_center_x = (SPOUT_BLADE_START_X + SPOUT_TIP_X) / 2.0
    spout.visual(
        Box((SPOUT_BLADE_LEN, SPOUT_WIDTH_Y, SPOUT_THICK_Z)),
        origin=Origin(xyz=(blade_center_x, 0.0, 0.0)),
        material=chrome,
        name="spout_blade",
    )

    # Aerator collar under the spout tip
    aerator_ring = (
        cq.Workplane("XY")
        .circle(AERATOR_OUTER_R)
        .circle(AERATOR_INNER_R)
        .extrude(AERATOR_H)
    )
    aerator_z = -SPOUT_THICK_Z / 2.0 - 0.004  # just below blade underside
    spout.visual(
        mesh_from_cadquery(aerator_ring, "aerator_collar"),
        origin=Origin(xyz=(OUTLET_X, 0.0, aerator_z)),
        material=chrome,
        name="aerator_collar",
    )

    # Dark outlet disc recessed inside the aerator
    spout.visual(
        Cylinder(radius=AERATOR_INNER_R, length=0.006),
        origin=Origin(xyz=(OUTLET_X, 0.0, aerator_z + 0.0025 + 0.003)),
        material=dark,
        name="outlet_disc",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, BEARING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.0,
            lower=-SPOUT_SWIVEL_RANGE, upper=SPOUT_SWIVEL_RANGE,
        ),
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
    # Lift stage: flat rectangular lever handle with grip grooves (flow)
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
        Box((0.018, 0.030, heel_h)),
        origin=Origin(xyz=(0.009, 0.0, -HANDLE_FLOAT + heel_h / 2.0)),
        material=chrome,
        name="pivot_heel",
    )

    # Grip grooves on the handle top surface
    for i in range(GROOVE_COUNT):
        gx = GROOVE_START_X + i * GROOVE_SPACING_X
        handle.visual(
            Box((GROOVE_WIDTH_X, GROOVE_DEPTH_Y, GROOVE_HEIGHT_Z)),
            origin=Origin(xyz=(gx, 0.0, HANDLE_THICK_Z + GROOVE_HEIGHT_Z / 2.0)),
            material=groove_mat,
            name=f"grip_groove_{i}",
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
    spout = object_model.get_part("spout")
    block = object_model.get_part("lever_pivot_block")
    handle = object_model.get_part("lever_handle")
    spout_swivel = object_model.get_articulation("spout_swivel")
    handle_swivel = object_model.get_articulation("handle_swivel")
    lift = object_model.get_articulation("handle_lift")

    # --- joint plan: types, axes, ranges ---
    ctx.check(
        "spout_swivel is revolute about vertical axis with ±90 deg range",
        spout_swivel.articulation_type == ArticulationType.REVOLUTE
        and abs(spout_swivel.axis[0]) < 1e-9
        and abs(spout_swivel.axis[1]) < 1e-9
        and abs(abs(spout_swivel.axis[2]) - 1.0) < 1e-9
        and spout_swivel.motion_limits is not None
        and abs(spout_swivel.motion_limits.lower + math.radians(90.0)) < 1e-6
        and abs(spout_swivel.motion_limits.upper - math.radians(90.0)) < 1e-6,
        details=f"axis={spout_swivel.axis}, limits={spout_swivel.motion_limits}",
    )
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
        "handle_swivel is revolute -45..+45 deg about vertical axis",
        handle_swivel.articulation_type == ArticulationType.REVOLUTE
        and abs(abs(handle_swivel.axis[2]) - 1.0) < 1e-9
        and handle_swivel.motion_limits is not None
        and abs(handle_swivel.motion_limits.lower + math.radians(45.0)) < 1e-6
        and abs(handle_swivel.motion_limits.upper - math.radians(45.0)) < 1e-6,
        details=f"axis={handle_swivel.axis}, limits={handle_swivel.motion_limits}",
    )

    # --- variant-specific geometry ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="base_collar")
    ctx.check(
        "raised circular collar exists around the base",
        collar_aabb is not None
        and abs(collar_aabb[0][2] - COLLAR_BOT_Z) < 1e-4
        and collar_aabb[1][2] - collar_aabb[0][2] > COLLAR_H - 0.002,
        details=f"collar_aabb={collar_aabb}",
    )

    cartridge_aabb = ctx.part_element_world_aabb(body, elem="cartridge_cap")
    ctx.check(
        "thin cartridge cap seam ring exists below the lever assembly",
        cartridge_aabb is not None
        and abs(cartridge_aabb[0][2] - CARTRIDGE_BOT_Z) < 1e-4
        and cartridge_aabb[1][2] - cartridge_aabb[0][2] < 0.005,
        details=f"cartridge_aabb={cartridge_aabb}",
    )

    # Grip grooves on handle
    groove_aabbs = []
    for i in range(GROOVE_COUNT):
        ga = ctx.part_element_world_aabb(handle, elem=f"grip_groove_{i}")
        groove_aabbs.append(ga)
    ctx.check(
        "grip grooves exist on the handle surface",
        all(g is not None for g in groove_aabbs),
        details=f"groove_aabbs={groove_aabbs}",
    )
    if groove_aabbs[0] is not None and groove_aabbs[-1] is not None:
        ctx.check(
            "grip grooves are raised above the handle blade top",
            groove_aabbs[0][0][2] > HANDLE_THICK_Z + 0.0005,
            details=f"groove_bottom_z={groove_aabbs[0][0][2]}",
        )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height ~0.30 m",
        handle_aabb is not None and 0.27 <= handle_aabb[1][2] <= 0.32,
        details=f"handle_aabb={handle_aabb}",
    )

    # --- spout features ---
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout blade cantilevers forward from the body",
        spout_aabb is not None and spout_aabb[1][0] > 0.15,
        details=f"spout_aabb={spout_aabb}",
    )

    aerator_aabb = ctx.part_element_world_aabb(spout, elem="aerator_collar")
    outlet_aabb = ctx.part_element_world_aabb(spout, elem="outlet_disc")
    ctx.check(
        "aerator collar protrudes below the spout blade underside",
        aerator_aabb is not None
        and aerator_aabb[0][2] < -SPOUT_THICK_Z / 2.0 + BEARING_CENTER_Z - 0.001,
        details=f"aerator_aabb={aerator_aabb}",
    )
    ctx.check(
        "dark outlet is recessed inside the aerator collar",
        aerator_aabb is not None
        and outlet_aabb is not None
        and outlet_aabb[0][2] > aerator_aabb[0][2] + 0.001,
        details=f"outlet={outlet_aabb}, collar={aerator_aabb}",
    )

    # --- mounting checks ---
    ctx.expect_contact(
        block,
        body,
        elem_a="pivot_block",
        elem_b="mounting_post",
        contact_tol=1e-4,
        name="pivot block seats on the mounting post",
    )
    ctx.expect_gap(
        handle,
        block,
        axis="z",
        min_gap=0.0005,
        max_gap=0.008,
        positive_elem="handle_blade",
        negative_elem="pivot_block",
        name="handle blade floats above the pivot block",
    )

    # --- spout bearing fit: intentional interference on the bearing section ---
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_bearing_ring",
        elem_b="bearing_section",
        reason="The spout bearing ring wraps around the body bearing section with a slight interference fit representing the swivel bushing.",
    )
    ctx.expect_overlap(
        spout,
        body,
        axes="xy",
        elem_a="spout_bearing_ring",
        elem_b="bearing_section",
        min_overlap=0.01,
        name="spout bearing ring surrounds the body bearing section in XY",
    )
    ctx.expect_within(
        body,
        spout,
        axes="xy",
        inner_elem="bearing_section",
        outer_elem="spout_bearing_ring",
        margin=0.002,
        name="body bearing section stays within the spout bearing ring footprint",
    )

    # --- decisive pose: spout swivel ---
    rest_spout_aabb = spout_aabb
    with ctx.pose({spout_swivel: SPOUT_SWIVEL_RANGE}):
        swiveled_aabb = ctx.part_world_aabb(spout)
        ctx.check(
            "positive spout swivel rotates the spout tip sideways",
            rest_spout_aabb is not None
            and swiveled_aabb is not None
            and abs(swiveled_aabb[1][1]) > abs(rest_spout_aabb[1][1]) + 0.05,
            details=f"rest={rest_spout_aabb}, swiveled={swiveled_aabb}",
        )
        # Spout blade should clear the rectangular column when swiveled 90°
        ctx.expect_gap(
            spout,
            body,
            axis="z",
            min_gap=-0.030,
            positive_elem="spout_blade",
            negative_elem="column",
            name="swiveled spout blade stays near the bearing elevation",
        )

    # --- decisive pose: handle lift ---
    rest_tip_z = handle_aabb[1][2] if handle_aabb is not None else None
    with ctx.pose({lift: LIFT_RANGE}):
        lifted_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive lift raises the handle grip tip upward",
            rest_tip_z is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_tip_z + 0.03,
            details=f"rest_top={rest_tip_z}, lifted_aabb={lifted_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
