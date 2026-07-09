from __future__ import annotations

"""Polished-chrome single-hole basin faucet — variant of the tall vessel faucet.

Structural changes from parent:
- Spout detachable-looking with a visible collar seam at the column junction.
- Real hollow outlet at the spout mouth (through-hole in the spout blade).
- Outlet aerator as a separate flip cap on a tiny revolute hinge.
- Oval base gasket under the stepped base plate.
- Same lever handle with swivel (temperature) and lift (flow) articulations.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- Oval dark rubber gasket on the countertop.
- Square stepped base plate rises from the gasket.
- Slim rectangular column carries the spout and handle assembly.
- Flat rectangular spout blade with a through-hole outlet near the tip.
- Chrome collar seam ring at the spout-column junction.
- Hinged aerator flip cap covers the outlet from below.
- Lever handle on a swivel+lift serial chain above the spout root.
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

# Oval gasket under the base
GASKET_SEMI_MAJOR = 0.052  # oval extends slightly beyond the base plate
GASKET_SEMI_MINOR = 0.040
GASKET_INNER_SEMI_MAJOR = 0.038
GASKET_INNER_SEMI_MINOR = 0.028
GASKET_H = 0.003

COLUMN_DEPTH_X = 0.035
COLUMN_WIDTH_Y = 0.045
COLUMN_TOP_Z = 0.235

SPOUT_WIDTH_Y = 0.050
SPOUT_THICK_Z = 0.020
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0  # flush with column rear face
SPOUT_TIP_X = 0.1825  # ~0.17 m forward reach past the column front face
SPOUT_TOP_Z = COLUMN_TOP_Z  # blade top flush with column top
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.215

OUTLET_X = 0.162  # outlet center, near the spout tip
OUTLET_R = 0.010  # through-hole radius (hollow outlet)

# Collar seam at spout-column junction
COLLAR_WIDTH_Y = SPOUT_WIDTH_Y + 0.008
COLLAR_DEPTH_X = 0.010
COLLAR_THICK_Z = 0.006

POST_R = 0.013
POST_H = 0.013
POST_TOP_Z = COLUMN_TOP_Z + POST_H  # 0.248

BLOCK_DEPTH_X = 0.045
BLOCK_WIDTH_Y = 0.044
BLOCK_H = 0.0365
BLOCK_TOP_REL = BLOCK_H  # in swivel-child frame

HANDLE_LEN_X = 0.170
HANDLE_WIDTH_Y = 0.050
HANDLE_THICK_Z = 0.013
HANDLE_FLOAT = 0.0015
HANDLE_REAR_REL_X = -BLOCK_DEPTH_X / 2.0

LIFT_RANGE = math.radians(25.0)
SWIVEL_RANGE = math.radians(45.0)

# Aerator flip cap dimensions
AERATOR_CAP_R = 0.011  # cap radius (slightly larger than outlet)
AERATOR_CAP_THICK = 0.003
AERATOR_HINGE_ANGLE = math.radians(80.0)  # max flip-open angle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("gasket_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    red = model.material("hot_red", rgba=(0.80, 0.08, 0.08, 1.0))
    blue = model.material("cold_blue", rgba=(0.10, 0.30, 0.78, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: gasket, stepped base, column, spout with hollow outlet,
    # collar seam, mounting post
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Oval base gasket (dark rubber, sits on countertop)
    gasket = (
        cq.Workplane("XY")
        .ellipse(GASKET_SEMI_MAJOR, GASKET_SEMI_MINOR)
        .ellipse(GASKET_INNER_SEMI_MAJOR, GASKET_INNER_SEMI_MINOR)
        .extrude(GASKET_H)
    )
    body.visual(
        mesh_from_cadquery(gasket, "base_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rubber,
        name="base_gasket",
    )

    # Stepped base plate
    body.visual(
        Box((BASE_LOWER_SIDE, BASE_LOWER_SIDE, BASE_LOWER_H)),
        origin=Origin(xyz=(0.0, 0.0, GASKET_H + BASE_LOWER_H / 2.0)),
        material=chrome,
        name="base_plate_lower",
    )
    body.visual(
        Box((BASE_UPPER_SIDE, BASE_UPPER_SIDE, BASE_UPPER_H)),
        origin=Origin(xyz=(0.0, 0.0, GASKET_H + BASE_LOWER_H + BASE_UPPER_H / 2.0)),
        material=chrome,
        name="base_plate_upper",
    )

    # Column
    column_base_z = GASKET_H + BASE_TOP_Z
    column_h = COLUMN_TOP_Z - column_base_z
    body.visual(
        Box((COLUMN_DEPTH_X, COLUMN_WIDTH_Y, column_h)),
        origin=Origin(xyz=(0.0, 0.0, column_base_z + column_h / 2.0)),
        material=chrome,
        name="column",
    )

    # Spout blade with real hollow outlet (through-hole) using CadQuery
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    spout_cx = (SPOUT_BACK_X + SPOUT_TIP_X) / 2.0
    spout_cz = SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0
    spout_solid = (
        cq.Workplane("XY")
        .transformed(offset=(spout_cx, 0.0, spout_cz))
        .box(spout_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z)
    )
    # Cut a through-hole at the outlet position
    spout_with_hole = (
        spout_solid
        .faces(">Z")
        .workplane()
        .center(OUTLET_X - spout_cx, 0.0)
        .circle(OUTLET_R)
        .cutThruAll()
    )
    body.visual(
        mesh_from_cadquery(spout_with_hole, "spout_blade"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="spout_blade",
    )

    # Dark outlet ring visible inside the through-hole (inner wall)
    outlet_ring = (
        cq.Workplane("XY")
        .transformed(offset=(OUTLET_X, 0.0, SPOUT_BOT_Z))
        .circle(OUTLET_R)
        .circle(OUTLET_R - 0.002)
        .extrude(SPOUT_THICK_Z)
    )
    body.visual(
        mesh_from_cadquery(outlet_ring, "outlet_bore_wall"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark,
        name="outlet_bore_wall",
    )

    # Collar seam — visible ring at the spout-column junction
    collar_solid = (
        cq.Workplane("XY")
        .transformed(offset=(COLUMN_DEPTH_X / 2.0 + COLLAR_DEPTH_X / 2.0, 0.0,
                            SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0))
        .box(COLLAR_DEPTH_X, COLLAR_WIDTH_Y, COLLAR_THICK_Z)
    )
    body.visual(
        mesh_from_cadquery(collar_solid, "collar_seam"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="collar_seam",
    )

    # Mounting post
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + POST_H / 2.0)),
        material=chrome,
        name="mounting_post",
    )

    # ------------------------------------------------------------------
    # Aerator flip cap — separate part, hinged at the front edge of outlet
    # ------------------------------------------------------------------
    aerator = model.part("spout_aerator")

    # Cap is a disc that covers the outlet from below
    cap_geom = (
        cq.Workplane("XY")
        .circle(AERATOR_CAP_R)
        .extrude(AERATOR_CAP_THICK)
    )
    aerator.visual(
        mesh_from_cadquery(cap_geom, "aerator_cap"),
        # Cap center offset from hinge: hinge is at front edge, cap extends backward
        origin=Origin(xyz=(-AERATOR_CAP_R, 0.0, 0.0)),
        material=chrome,
        name="aerator_cap",
    )

    # Tiny hinge barrel visual on the cap (connects to the spout)
    hinge_barrel = Cylinder(radius=0.002, length=0.012)
    aerator.visual(
        hinge_barrel,
        origin=Origin(xyz=(0.0, 0.0, AERATOR_CAP_THICK / 2.0),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="aerator_hinge_barrel",
    )

    # Aerator hinge joint: revolute about Y-axis at the front edge of outlet
    # When q=0, cap is horizontal (closed, covering outlet from below)
    # Positive q swings the cap rear edge downward (flip open)
    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(OUTLET_X + AERATOR_CAP_R, 0.0, SPOUT_BOT_Z - AERATOR_CAP_THICK)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=AERATOR_HINGE_ANGLE
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
    # Temperature dots
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
    # Lift stage: flat rectangular lever handle (flow)
    # ------------------------------------------------------------------
    handle = model.part("lever_handle")
    handle.visual(
        Box((HANDLE_LEN_X, HANDLE_WIDTH_Y, HANDLE_THICK_Z)),
        origin=Origin(xyz=(HANDLE_LEN_X / 2.0, 0.0, HANDLE_THICK_Z / 2.0)),
        material=chrome,
        name="handle_blade",
    )
    heel_h = HANDLE_FLOAT + 0.004
    handle.visual(
        Box((0.018, 0.030, heel_h)),
        origin=Origin(xyz=(0.009, 0.0, -HANDLE_FLOAT + heel_h / 2.0)),
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
    aerator = object_model.get_part("spout_aerator")
    block = object_model.get_part("lever_pivot_block")
    handle = object_model.get_part("lever_handle")
    swivel = object_model.get_articulation("handle_swivel")
    lift = object_model.get_articulation("handle_lift")
    hinge = object_model.get_articulation("aerator_hinge")

    # --- Variant-specific: oval base gasket ---
    gasket_aabb = ctx.part_element_world_aabb(body, elem="base_gasket")
    ctx.check(
        "oval base gasket exists under the base plate",
        gasket_aabb is not None and gasket_aabb[0][2] < 0.001,
        details=f"gasket_aabb={gasket_aabb}",
    )
    ctx.check(
        "gasket is wider in one axis (oval, not circular)",
        gasket_aabb is not None
        and (gasket_aabb[1][0] - gasket_aabb[0][0]) > (gasket_aabb[1][1] - gasket_aabb[0][1]) + 0.005,
        details=f"gasket_aabb={gasket_aabb}",
    )

    # --- Variant-specific: collar seam at spout-column junction ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="collar_seam")
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_blade")
    ctx.check(
        "collar seam exists at the spout-column junction",
        collar_aabb is not None
        and spout_aabb is not None
        and abs(collar_aabb[0][0] - COLUMN_DEPTH_X / 2.0) < 0.015,
        details=f"collar_aabb={collar_aabb}",
    )

    # --- Variant-specific: real hollow outlet (through-hole) ---
    outlet_bore_aabb = ctx.part_element_world_aabb(body, elem="outlet_bore_wall")
    ctx.check(
        "hollow outlet bore wall is present inside the spout",
        outlet_bore_aabb is not None
        and outlet_bore_aabb[0][2] >= SPOUT_BOT_Z - 0.002
        and outlet_bore_aabb[1][2] <= SPOUT_TOP_Z + 0.002
        and abs((outlet_bore_aabb[0][0] + outlet_bore_aabb[1][0]) / 2.0 - OUTLET_X) < 0.005,
        details=f"outlet_bore_aabb={outlet_bore_aabb}",
    )

    # --- Variant-specific: aerator hinge joint ---
    ctx.check(
        "aerator hinge is revolute with 0..80 deg range about a horizontal axis",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(hinge.axis[0]) < 1e-9
        and abs(abs(hinge.axis[1]) - 1.0) < 1e-9
        and abs(hinge.axis[2]) < 1e-9
        and hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower) < 1e-9
        and hinge.motion_limits.upper > math.radians(60.0),
        details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
    )
    ctx.check(
        "aerator hinge parents the aerator to the faucet body",
        hinge.parent == body.name and hinge.child == aerator.name,
        details=f"parent={hinge.parent}, child={hinge.child}",
    )

    # --- Variant-specific: aerator cap near outlet ---
    cap_aabb = ctx.part_element_world_aabb(aerator, elem="aerator_cap")
    ctx.check(
        "aerator cap sits near the spout outlet position",
        cap_aabb is not None
        and abs((cap_aabb[0][0] + cap_aabb[1][0]) / 2.0 - OUTLET_X) < 0.015
        and cap_aabb[1][2] <= SPOUT_BOT_Z + 0.002,
        details=f"cap_aabb={cap_aabb}",
    )

    # --- Decisive pose: aerator flip opens downward ---
    with ctx.pose({hinge: AERATOR_HINGE_ANGLE}):
        flipped_aabb = ctx.part_world_aabb(aerator)
        ctx.check(
            "aerator cap swings away from the spout underside when flipped open",
            cap_aabb is not None
            and flipped_aabb is not None
            and flipped_aabb[0][2] < cap_aabb[0][2] - 0.005,
            details=f"closed_bottom={None if cap_aabb is None else cap_aabb[0][2]}, "
                    f"flipped_bottom={None if flipped_aabb is None else flipped_aabb[0][2]}",
        )

    # --- Joint plan: handle lift and swivel preserved ---
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
        "serial chain: swivel -> block -> lift -> handle",
        swivel.child == block.name and lift.parent == block.name and lift.child == handle.name,
        details=f"swivel.child={swivel.child}, lift.parent={lift.parent}, lift.child={lift.child}",
    )

    # --- Grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "gasket is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-4,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height ~0.30 m",
        handle_aabb is not None and 0.27 <= handle_aabb[1][2] <= 0.32,
        details=f"handle_aabb={handle_aabb}",
    )

    # --- Handle mounting checks preserved ---
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
        min_gap=0.03,
        name="handle assembly stays clear above the fixed spout blade",
    )

    # --- Decisive pose checks for handle ---
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

    rest_handle_aabb = handle_aabb
    with ctx.pose({swivel: SWIVEL_RANGE}):
        swung_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive swivel slews the handle sideways",
            rest_handle_aabb is not None
            and swung_aabb is not None
            and swung_aabb[1][1] > rest_handle_aabb[1][1] + 0.05,
            details=f"rest={rest_handle_aabb}, swung={swung_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
