from __future__ import annotations

"""Polished-chrome single-hole basin faucet with a cylindrical column, gently
curved downward spout, and top-mounted lever handle.

Layout (meters, +Z up, ground at z=0, spout extends along +X):
- A circular escutcheon ring sits on the deck around the column base.
- A cylindrical column rises from the base.
- A curved tubular spout emerges from the column front near the top and arcs
  gently downward to a hollow outlet with a chrome rim.
- A short mounting post on the column top carries a swiveling pivot block.
- A flat lever handle lifts from the pivot block (flow, 0..25 deg) and the
  whole assembly swivels on the post (temperature, -45..+45 deg).
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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Key dimensions (meters)
# ---------------------------------------------------------------------------
BASE_RADIUS = 0.033
BASE_H = 0.012

COLUMN_R = 0.020
COLUMN_BOT_Z = BASE_H  # 0.012
COLUMN_TOP_Z = 0.220
COLUMN_H = COLUMN_TOP_Z - COLUMN_BOT_Z  # 0.208

# Spout tube
SPOUT_TUBE_R = 0.012
SPOUT_PATH = [
    (0.0, 0.0, COLUMN_TOP_Z - 0.005),              # start inside column
    (COLUMN_R + 0.012, 0.0, COLUMN_TOP_Z + 0.005),  # emerges from column
    (COLUMN_R + 0.055, 0.0, COLUMN_TOP_Z + 0.008),  # slight rise
    (COLUMN_R + 0.095, 0.0, COLUMN_TOP_Z - 0.005),  # starts descending
    (COLUMN_R + 0.130, 0.0, COLUMN_TOP_Z - 0.040),  # mouth, downward
]
MOUTH_X = SPOUT_PATH[-1][0]   # 0.150
MOUTH_Z = SPOUT_PATH[-1][2]   # 0.180
SPOUT_ROOT_X = SPOUT_PATH[0][0]
SPOUT_ROOT_Z = SPOUT_PATH[0][2]

# Tangent at spout end for outlet orientation
_dx = SPOUT_PATH[-1][0] - SPOUT_PATH[-2][0]
_dz = SPOUT_PATH[-1][2] - SPOUT_PATH[-2][2]
_tan_len = math.sqrt(_dx * _dx + _dz * _dz)
END_TAN_X = _dx / _tan_len
END_TAN_Z = _dz / _tan_len
# Rotation about Y to align default Z-axis with the tangent direction
MOUTH_PITCH = math.atan2(END_TAN_X, END_TAN_Z)

# Outlet at mouth
OUTLET_DISC_R = SPOUT_TUBE_R * 1.02  # slightly overfills tube bore for mesh connectivity
OUTLET_RING_INNER = SPOUT_TUBE_R * 0.76
OUTLET_RING_OUTER = SPOUT_TUBE_R * 1.08
OUTLET_RING_H = 0.005

# Post and handle
POST_R = 0.011
POST_H = 0.016
POST_TOP_Z = COLUMN_TOP_Z + POST_H  # 0.236

BLOCK_DEPTH_X = 0.040
BLOCK_WIDTH_Y = 0.038
BLOCK_H = 0.030

HANDLE_LEN_X = 0.130
HANDLE_WIDTH_Y = 0.040
HANDLE_THICK_Z = 0.011
HANDLE_FLOAT = 0.002
HANDLE_REAR_X = -BLOCK_DEPTH_X / 2.0

LIFT_RANGE = math.radians(25.0)
SWIVEL_RANGE = math.radians(45.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark = model.material("outlet_dark", rgba=(0.06, 0.06, 0.08, 1.0))
    red = model.material("hot_dot", rgba=(0.80, 0.08, 0.08, 1.0))
    blue = model.material("cold_dot", rgba=(0.10, 0.30, 0.78, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: escutcheon, column, spout, outlet, mounting post
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Escutcheon ring (annular base plate on the deck)
    escutcheon = (
        cq.Workplane("XY")
        .circle(BASE_RADIUS)
        .circle(COLUMN_R - 0.001)
        .extrude(BASE_H)
    )
    body.visual(
        mesh_from_cadquery(escutcheon, "escutcheon"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="escutcheon",
    )

    # Cylindrical column
    body.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_BOT_Z + COLUMN_H / 2.0)),
        material=chrome,
        name="column",
    )

    # Column cap disc (closes the column top, provides a seat for the post)
    body.visual(
        Cylinder(radius=COLUMN_R, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z - 0.002)),
        material=chrome,
        name="column_cap",
    )

    # Curved spout tube (open ends → hollow mouth)
    spout_mesh = mesh_from_geometry(
        tube_from_spline_points(
            SPOUT_PATH,
            radius=SPOUT_TUBE_R,
            samples_per_segment=16,
            radial_segments=20,
            cap_ends=False,
        ),
        "spout_tube",
    )
    body.visual(
        spout_mesh,
        material=chrome,
        name="spout_tube",
    )

    # Chrome outlet ring at the spout mouth (annulus aligned to tube tangent)
    ring = (
        cq.Workplane("XY")
        .circle(OUTLET_RING_OUTER)
        .circle(OUTLET_RING_INNER)
        .extrude(OUTLET_RING_H)
    )
    body.visual(
        mesh_from_cadquery(ring, "outlet_ring"),
        origin=Origin(
            xyz=(MOUTH_X - 0.002 * END_TAN_X, 0.0, MOUTH_Z - 0.002 * END_TAN_Z),
            rpy=(0.0, MOUTH_PITCH, 0.0),
        ),
        material=chrome,
        name="outlet_ring",
    )

    # Dark outlet disc (recessed inside the tube mouth — the hollow outlet)
    body.visual(
        Cylinder(radius=OUTLET_DISC_R, length=0.004),
        origin=Origin(
            xyz=(
                MOUTH_X - 0.003 * END_TAN_X,
                0.0,
                MOUTH_Z - 0.003 * END_TAN_Z,
            ),
            rpy=(0.0, MOUTH_PITCH, 0.0),
        ),
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
    # Swivel stage: lever pivot block (temperature)
    # ------------------------------------------------------------------
    block = model.part("lever_pivot_block")
    block.visual(
        Box((BLOCK_DEPTH_X, BLOCK_WIDTH_Y, BLOCK_H)),
        origin=Origin(xyz=(0.0, 0.0, BLOCK_H / 2.0)),
        material=chrome,
        name="pivot_block",
    )
    # Red/blue temperature dots on the front face
    dot_x = BLOCK_DEPTH_X / 2.0
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, 0.006, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="hot_dot",
    )
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, -0.006, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
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
    # Lift stage: flat lever handle (flow)
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
        origin=Origin(xyz=(HANDLE_REAR_X, 0.0, BLOCK_H + HANDLE_FLOAT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0, lower=0.0, upper=LIFT_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    block = object_model.get_part("lever_pivot_block")
    handle = object_model.get_part("lever_handle")
    swivel = object_model.get_articulation("handle_swivel")
    lift = object_model.get_articulation("handle_lift")

    # --- joint plan ---
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

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "base escutcheon is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height 0.24..0.30 m (compact single-hole basin faucet)",
        handle_aabb is not None and 0.24 <= handle_aabb[1][2] <= 0.30,
        details=f"handle_aabb={handle_aabb}",
    )

    # --- cylindrical column (not rectangular) ---
    column_aabb = ctx.part_element_world_aabb(body, elem="column")
    ctx.check(
        "column has a circular cross-section (dx ≈ dy for a cylinder)",
        column_aabb is not None
        and abs(
            (column_aabb[1][0] - column_aabb[0][0])
            - (column_aabb[1][1] - column_aabb[0][1])
        ) < 0.002,
        details=f"column_aabb={column_aabb}",
    )

    # --- curved downward spout ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_tube")
    ctx.check(
        "spout tube extends forward from the column",
        spout_aabb is not None and spout_aabb[1][0] > COLUMN_R + 0.08,
        details=f"spout_aabb={spout_aabb}",
    )
    ctx.check(
        "spout mouth is lower than spout root (gentle downward curve)",
        spout_aabb is not None
        and MOUTH_Z < SPOUT_ROOT_Z
        and spout_aabb[0][2] < COLUMN_TOP_Z - 0.02,
        details=f"mouth_z={MOUTH_Z}, root_z={SPOUT_ROOT_Z}, spout_min_z={spout_aabb[0][2] if spout_aabb else None}",
    )

    # --- hollow outlet at spout mouth ---
    ring_aabb = ctx.part_element_world_aabb(body, elem="outlet_ring")
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_disc")
    ctx.check(
        "chrome outlet ring present at the spout mouth",
        ring_aabb is not None
        and ring_aabb[1][0] > MOUTH_X - 0.01
        and ring_aabb[0][0] < MOUTH_X + 0.02,
        details=f"ring_aabb={ring_aabb}",
    )
    ctx.expect_within(
        body,
        body,
        axes="xy",
        inner_elem="outlet_disc",
        outer_elem="outlet_ring",
        margin=0.003,
        name="dark outlet disc is within the outlet ring footprint (hollow outlet)",
    )

    # --- mounting ---
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
        max_gap=0.005,
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
