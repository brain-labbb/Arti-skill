from __future__ import annotations

"""Polished-chrome single-hole basin faucet with offset side lever, hinged aerator,
grip grooves, and rear screw caps.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A square stepped base plate carries a slim rectangular column (one deck penetration).
- A flat rectangular spout blade cantilevers forward from the column top, with a
  hinged aerator flap at its underside near the tip.
- An offset side lever housing extends from the column top toward +Y, carrying
  the pivot block and handle assembly.
- Two small screw caps on the column rear face.
- Subtle grooves on the handle grip surface.
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
BASE_LOWER_SIDE = 0.080
BASE_LOWER_H = 0.005
BASE_UPPER_SIDE = 0.060
BASE_UPPER_H = 0.010
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H  # 0.015

COLUMN_DEPTH_X = 0.032
COLUMN_WIDTH_Y = 0.040
COLUMN_TOP_Z = 0.200

SPOUT_WIDTH_Y = 0.044
SPOUT_THICK_Z = 0.018
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0
SPOUT_TIP_X = 0.165
SPOUT_TOP_Z = COLUMN_TOP_Z
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z

OUTLET_X = 0.148  # aerator center near spout tip
AERATOR_WIDTH_Y = 0.030
AERATOR_DEPTH_X = 0.022
AERATOR_THICK_Z = 0.005
AERATOR_HINGE_X = OUTLET_X - AERATOR_DEPTH_X / 2.0  # hinge at rear edge of aerator

# Side lever housing - offset to +Y from column
SIDE_HOUSING_WIDTH_X = 0.040
SIDE_HOUSING_DEPTH_Y = 0.030
SIDE_HOUSING_H = 0.028
SIDE_HOUSING_OFFSET_Y = COLUMN_WIDTH_Y / 2.0 + SIDE_HOUSING_DEPTH_Y / 2.0 - 0.005
SIDE_HOUSING_TOP_Z = COLUMN_TOP_Z + SIDE_HOUSING_H

POST_R = 0.010
POST_H = 0.010
POST_TOP_Z = SIDE_HOUSING_TOP_Z + POST_H

BLOCK_DEPTH_X = 0.038
BLOCK_WIDTH_Y = 0.038
BLOCK_H = 0.030
BLOCK_TOP_REL = BLOCK_H

HANDLE_LEN_X = 0.150
HANDLE_WIDTH_Y = 0.042
HANDLE_THICK_Z = 0.011
HANDLE_FLOAT = 0.0015
HANDLE_REAR_REL_X = -BLOCK_DEPTH_X / 2.0

# Grooves on handle grip
GROOVE_COUNT = 5
GROOVE_WIDTH = 0.002
GROOVE_DEPTH = 0.001
GROOVE_SPACING = 0.018

# Screw caps on column rear
SCREW_CAP_R = 0.004
SCREW_CAP_H = 0.003
SCREW_CAP_SPACING_Y = 0.018

LIFT_RANGE = math.radians(25.0)
SWIVEL_RANGE = math.radians(45.0)
AERATOR_HINGE_RANGE = math.radians(90.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    red = model.material("hot_red", rgba=(0.80, 0.08, 0.08, 1.0))
    blue = model.material("cold_blue", rgba=(0.10, 0.30, 0.78, 1.0))
    groove_mat = model.material("groove_dark", rgba=(0.25, 0.25, 0.28, 1.0))
    cap_mat = model.material("screw_cap", rgba=(0.70, 0.72, 0.75, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: stepped base, column, spout blade, side housing, screw caps
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
    # Offset side lever housing - a small rectangular boss extending from column top to +Y
    body.visual(
        Box((SIDE_HOUSING_WIDTH_X, SIDE_HOUSING_DEPTH_Y, SIDE_HOUSING_H)),
        origin=Origin(
            xyz=(0.0, SIDE_HOUSING_OFFSET_Y, COLUMN_TOP_Z + SIDE_HOUSING_H / 2.0)
        ),
        material=chrome,
        name="side_lever_housing",
    )
    # Connecting bridge between column top and side housing (ensures connectivity)
    bridge_width = SIDE_HOUSING_OFFSET_Y - COLUMN_WIDTH_Y / 2.0 + 0.005
    body.visual(
        Box((SIDE_HOUSING_WIDTH_X * 0.7, bridge_width, SIDE_HOUSING_H * 0.6)),
        origin=Origin(
            xyz=(
                0.0,
                COLUMN_WIDTH_Y / 2.0 + bridge_width / 2.0 - 0.005,
                COLUMN_TOP_Z + SIDE_HOUSING_H * 0.3,
            )
        ),
        material=chrome,
        name="housing_bridge",
    )
    # Mounting post on side housing top
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, SIDE_HOUSING_OFFSET_Y, SIDE_HOUSING_TOP_Z + POST_H / 2.0)),
        material=chrome,
        name="mounting_post",
    )
    # Dark outlet recess on spout underside (visible when aerator closed)
    body.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.002)),
        material=dark,
        name="outlet_recess",
    )
    # Two screw caps on column rear face (-X side)
    screw_x = -COLUMN_DEPTH_X / 2.0 - SCREW_CAP_H / 2.0 + 0.001
    screw_z = COLUMN_TOP_Z - 0.040
    body.visual(
        Cylinder(radius=SCREW_CAP_R, length=SCREW_CAP_H),
        origin=Origin(
            xyz=(screw_x, SCREW_CAP_SPACING_Y / 2.0, screw_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cap_mat,
        name="screw_cap_upper",
    )
    body.visual(
        Cylinder(radius=SCREW_CAP_R, length=SCREW_CAP_H),
        origin=Origin(
            xyz=(screw_x, -SCREW_CAP_SPACING_Y / 2.0, screw_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cap_mat,
        name="screw_cap_lower",
    )

    # ------------------------------------------------------------------
    # Swivel stage: lever pivot block on side housing post (temperature)
    # ------------------------------------------------------------------
    block = model.part("lever_pivot_block")
    block.visual(
        Box((BLOCK_DEPTH_X, BLOCK_WIDTH_Y, BLOCK_H)),
        origin=Origin(xyz=(0.0, 0.0, BLOCK_H / 2.0)),
        material=chrome,
        name="pivot_block",
    )
    # Temperature dots on block side face
    dot_x = BLOCK_DEPTH_X / 2.0
    block.visual(
        Cylinder(radius=0.002, length=0.002),
        origin=Origin(xyz=(dot_x, 0.006, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="hot_dot",
    )
    block.visual(
        Cylinder(radius=0.002, length=0.002),
        origin=Origin(xyz=(dot_x, -0.006, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="cold_dot",
    )

    model.articulation(
        "handle_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=block,
        origin=Origin(xyz=(0.0, SIDE_HOUSING_OFFSET_Y, POST_TOP_Z)),
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
    # Grip grooves on the handle top surface - thin dark strips
    groove_start_x = HANDLE_LEN_X * 0.45
    for i in range(GROOVE_COUNT):
        gx = groove_start_x + i * GROOVE_SPACING
        handle.visual(
            Box((GROOVE_WIDTH, HANDLE_WIDTH_Y * 0.7, GROOVE_DEPTH)),
            origin=Origin(
                xyz=(gx, 0.0, HANDLE_THICK_Z + GROOVE_DEPTH / 2.0 - 0.0002)
            ),
            material=groove_mat,
            name=f"grip_groove_{i}",
        )
    # Pivot heel
    heel_h = HANDLE_FLOAT + 0.003
    handle.visual(
        Box((0.016, 0.026, heel_h)),
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
    # Aerator flap: hinged outlet cover at spout tip underside
    # Part frame at hinge line (rear edge of aerator)
    # ------------------------------------------------------------------
    aerator = model.part("aerator_flap")
    # Flat rectangular flap extending +X from hinge line
    aerator.visual(
        Box((AERATOR_DEPTH_X, AERATOR_WIDTH_Y, AERATOR_THICK_Z)),
        origin=Origin(xyz=(AERATOR_DEPTH_X / 2.0, 0.0, -AERATOR_THICK_Z / 2.0)),
        material=chrome,
        name="aerator_plate",
    )
    # Dark mesh/screen on the inner face (top side when closed)
    aerator.visual(
        Box((AERATOR_DEPTH_X * 0.7, AERATOR_WIDTH_Y * 0.7, 0.001)),
        origin=Origin(xyz=(AERATOR_DEPTH_X / 2.0, 0.0, 0.0005)),
        material=dark,
        name="aerator_screen",
    )
    # Small hinge knuckle at the pivot end (connects flap to spout)
    aerator.visual(
        Cylinder(radius=0.003, length=AERATOR_WIDTH_Y * 0.4),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="hinge_knuckle",
    )

    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=aerator,
        # Hinge at rear edge of aerator position, on spout underside
        origin=Origin(xyz=(AERATOR_HINGE_X, 0.0, SPOUT_BOT_Z)),
        # Y axis so positive q swings the flap downward (open)
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=AERATOR_HINGE_RANGE
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
        "swivel joint is revolute about vertical axis",
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
        "aerator hinge is revolute with non-trivial range",
        aerator_hinge.articulation_type == ArticulationType.REVOLUTE
        and aerator_hinge.motion_limits is not None
        and aerator_hinge.motion_limits.upper > math.radians(30.0)
        and abs(aerator_hinge.motion_limits.lower) < 1e-9,
        details=f"axis={aerator_hinge.axis}, limits={aerator_hinge.motion_limits}",
    )

    # --- offset side lever housing ---
    housing_aabb = ctx.part_element_world_aabb(body, elem="side_lever_housing")
    column_aabb = ctx.part_element_world_aabb(body, elem="column")
    ctx.check(
        "side lever housing is offset to the side of the column",
        housing_aabb is not None
        and column_aabb is not None
        and housing_aabb[0][1] > column_aabb[0][1],
        details=f"housing={housing_aabb}, column={column_aabb}",
    )
    ctx.check(
        "side lever housing is connected to column via bridge",
        housing_aabb is not None
        and ctx.part_element_world_aabb(body, elem="housing_bridge") is not None,
        details="housing_bridge must exist",
    )

    # --- screw caps on body rear ---
    cap1_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_upper")
    cap2_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_lower")
    ctx.check(
        "two screw caps exist on the body rear face",
        cap1_aabb is not None
        and cap2_aabb is not None
        and cap1_aabb[0][0] < 0.0
        and cap2_aabb[0][0] < 0.0
        and cap1_aabb[1][1] > cap2_aabb[1][1],
        details=f"cap_upper={cap1_aabb}, cap_lower={cap2_aabb}",
    )

    # --- grip grooves on handle ---
    groove_aabbs = [
        ctx.part_element_world_aabb(handle, elem=f"grip_groove_{i}")
        for i in range(GROOVE_COUNT)
    ]
    ctx.check(
        "handle has multiple grip grooves on its surface",
        all(g is not None for g in groove_aabbs)
        and len(groove_aabbs) >= 3,
        details=f"groove_count={len([g for g in groove_aabbs if g is not None])}",
    )

    # --- aerator flap is a separate part with hinge ---
    aerator_aabb = ctx.part_world_aabb(aerator)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "aerator flap is a separate part near spout tip underside",
        aerator_aabb is not None
        and body_aabb is not None
        and aerator_aabb[0][2] < SPOUT_BOT_Z + 0.005
        and aerator_aabb[0][0] > 0.10,
        details=f"aerator_aabb={aerator_aabb}",
    )

    # --- grounding and scale ---
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "total faucet height is reasonable (0.22..0.32 m)",
        handle_aabb is not None and 0.22 <= handle_aabb[1][2] <= 0.32,
        details=f"handle_aabb={handle_aabb}",
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
    ctx.expect_contact(
        handle,
        block,
        elem_a="pivot_heel",
        elem_b="pivot_block",
        contact_tol=1e-4,
        name="handle pivot heel seats on the pivot block top",
    )
    ctx.expect_gap(
        handle,
        block,
        axis="z",
        min_gap=0.0005,
        max_gap=0.005,
        positive_elem="handle_blade",
        negative_elem="pivot_block",
        name="handle blade floats slightly above the pivot block",
    )

    # --- decisive pose: handle lift raises grip ---
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

    # --- decisive pose: aerator hinge opens the flap downward ---
    rest_aerator_z = aerator_aabb[0][2] if aerator_aabb is not None else None
    with ctx.pose({aerator_hinge: AERATOR_HINGE_RANGE}):
        open_aabb = ctx.part_world_aabb(aerator)
        ctx.check(
            "aerator hinge opens flap downward (lower z when open)",
            rest_aerator_z is not None
            and open_aabb is not None
            and open_aabb[0][2] < rest_aerator_z - 0.005,
            details=f"rest_bottom={rest_aerator_z}, open_aabb={open_aabb}",
        )

    # --- swivel moves handle sideways ---
    rest_handle_aabb = handle_aabb
    with ctx.pose({swivel: SWIVEL_RANGE}):
        swung_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive swivel slews the handle sideways",
            rest_handle_aabb is not None
            and swung_aabb is not None
            and abs(swung_aabb[1][1] - rest_handle_aabb[1][1]) > 0.02,
            details=f"rest={rest_handle_aabb}, swung={swung_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
