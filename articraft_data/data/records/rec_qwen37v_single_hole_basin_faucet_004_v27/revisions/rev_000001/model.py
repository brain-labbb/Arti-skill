from __future__ import annotations

"""Polished-chrome single-hole basin faucet with rectangular geometry.

A compact single-hole basin faucet (~0.18 m tall) with:
- Stepped base plate and slim rectangular column.
- Flat rectangular spout blade cantilevering forward from the column top, with a
  real hollow rectangular slot outlet cut through the spout mouth.
- Side lever on a short horizontal axle through the column side, rotating about
  a horizontal left-right axis (revolute, 0..30 deg) for flow control.
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
BASE_LOWER_SIDE = 0.060
BASE_LOWER_H = 0.005
BASE_UPPER_SIDE = 0.048
BASE_UPPER_H = 0.010
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H  # 0.015

COLUMN_DEPTH_X = 0.032
COLUMN_WIDTH_Y = 0.038
COLUMN_TOP_Z = 0.150

SPOUT_WIDTH_Y = 0.038
SPOUT_THICK_Z = 0.016
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0  # flush with column rear face
SPOUT_TIP_X = 0.135  # ~0.12 m forward reach past the column front face
SPOUT_TOP_Z = COLUMN_TOP_Z  # blade top flush with column top
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.134

# Rectangular slot outlet dimensions (hollow through-hole at spout mouth)
SLOT_WIDTH_Y = 0.028
SLOT_HEIGHT_Z = 0.006
SLOT_DEPTH_X = 0.012  # cut extends back from spout tip face

# Side lever axle and handle
AXLE_RADIUS = 0.005
AXLE_LENGTH = 0.016
# Axle center on column right face (+Y side), near the column top
AXLE_CENTER_Y = COLUMN_WIDTH_Y / 2.0 + AXLE_LENGTH / 2.0
AXLE_CENTER_Z = COLUMN_TOP_Z - 0.025  # 0.125

HANDLE_LEN = 0.095  # lever length from axle center to grip tip
HANDLE_WIDTH = 0.020
HANDLE_THICK = 0.010

LEVER_RANGE = math.radians(30.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark = model.material("outlet_dark", rgba=(0.06, 0.06, 0.07, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: stepped base plate, column, spout with hollow slot outlet
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

    # Spout blade with real hollow rectangular slot outlet at the mouth.
    # Build with CadQuery: solid spout block, then cut a rectangular through-slot
    # at the forward tip to form the flat rectangular outlet.
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    spout_center_x = (SPOUT_BACK_X + SPOUT_TIP_X) / 2.0
    spout_center_z = SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0

    spout = (
        cq.Workplane("XY")
        .box(spout_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z)
    )
    # Cut a rectangular slot from the front face inward.
    # The slot is centered in Y and Z on the spout, and cuts SLOT_DEPTH_X from the tip.
    slot_cut_x = SPOUT_TIP_X - SLOT_DEPTH_X / 2.0 - spout_center_x
    spout = (
        spout
        .faces(">X")
        .workplane()
        .rect(SLOT_WIDTH_Y, SLOT_HEIGHT_Z)
        .cutBlind(-SLOT_DEPTH_X)
    )
    body.visual(
        mesh_from_cadquery(spout, "spout_blade"),
        origin=Origin(xyz=(spout_center_x, 0.0, spout_center_z)),
        material=chrome,
        name="spout_blade",
    )

    # Dark interior face visible through the slot (a thin dark plate recessed
    # inside the slot to represent the waterway interior).
    body.visual(
        Box((0.002, SLOT_WIDTH_Y - 0.002, SLOT_HEIGHT_Z - 0.002)),
        origin=Origin(xyz=(SPOUT_TIP_X - SLOT_DEPTH_X + 0.001, 0.0, spout_center_z)),
        material=dark,
        name="slot_interior",
    )

    # Small boss on the column right face to carry the lever axle
    boss_radius = 0.009
    boss_h = 0.006
    body.visual(
        Cylinder(radius=boss_radius, length=boss_h),
        origin=Origin(
            xyz=(0.0, COLUMN_WIDTH_Y / 2.0 + boss_h / 2.0, AXLE_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="lever_boss",
    )

    # ------------------------------------------------------------------
    # Side lever on horizontal axle (flow control)
    # ------------------------------------------------------------------
    lever = model.part("side_lever")

    # Short cylindrical axle stub that inserts into the boss
    lever.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
        origin=Origin(
            xyz=(0.0, AXLE_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="lever_axle",
    )

    # Flat rectangular lever handle extending downward from the axle
    # (at rest it hangs roughly vertical; positive rotation lifts it outward)
    lever.visual(
        Box((HANDLE_WIDTH, HANDLE_THICK, HANDLE_LEN)),
        origin=Origin(xyz=(0.0, 0.0, -HANDLE_LEN / 2.0)),
        material=chrome,
        name="lever_handle",
    )

    # Small grip cap at the lever tip
    lever.visual(
        Box((HANDLE_WIDTH + 0.004, HANDLE_THICK + 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -HANDLE_LEN + 0.004)),
        material=chrome,
        name="lever_grip",
    )

    model.articulation(
        "lever_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        # Joint origin at the axle center on the column side
        origin=Origin(xyz=(0.0, COLUMN_WIDTH_Y / 2.0 + boss_h, AXLE_CENTER_Z)),
        # Horizontal axle along +Y (left-right axis through the column)
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=LEVER_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    lever = object_model.get_part("side_lever")
    joint = object_model.get_articulation("lever_rotate")

    # --- joint plan: revolute about horizontal Y axis, 0..30 deg ---
    ctx.check(
        "lever joint is revolute about horizontal Y axis, 0..30 deg",
        joint.articulation_type == ArticulationType.REVOLUTE
        and abs(joint.axis[0]) < 1e-9
        and abs(abs(joint.axis[1]) - 1.0) < 1e-9
        and abs(joint.axis[2]) < 1e-9
        and joint.motion_limits is not None
        and abs(joint.motion_limits.lower - 0.0) < 1e-9
        and abs(joint.motion_limits.upper - LEVER_RANGE) < 1e-6,
        details=f"axis={joint.axis}, limits={joint.motion_limits}",
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "faucet height ~0.15-0.18 m (compact basin faucet)",
        body_aabb is not None and 0.14 <= body_aabb[1][2] <= 0.20,
        details=f"body_aabb={body_aabb}",
    )

    # --- hollow rectangular slot outlet ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_blade")
    slot_aabb = ctx.part_element_world_aabb(body, elem="slot_interior")
    ctx.check(
        "spout blade cantilevers forward from the column",
        spout_aabb is not None and spout_aabb[1][0] > COLUMN_DEPTH_X / 2.0 + 0.08,
        details=f"spout_aabb={spout_aabb}",
    )
    ctx.check(
        "dark slot interior is recessed inside the spout mouth",
        spout_aabb is not None
        and slot_aabb is not None
        and slot_aabb[0][0] > spout_aabb[0][0]
        and slot_aabb[1][0] <= spout_aabb[1][0] + 1e-6,
        details=f"slot_aabb={slot_aabb}, spout_aabb={spout_aabb}",
    )
    ctx.check(
        "slot outlet is flat rectangular (wider than tall)",
        slot_aabb is not None
        and (slot_aabb[1][1] - slot_aabb[0][1]) > (slot_aabb[1][2] - slot_aabb[0][2]) * 2.0,
        details=f"slot_aabb={slot_aabb}",
    )

    # --- side lever mounted on column side ---
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "side lever extends beyond the column right face",
        lever_aabb is not None
        and lever_aabb[1][1] > COLUMN_WIDTH_Y / 2.0 + 0.01,
        details=f"lever_aabb={lever_aabb}",
    )
    ctx.check(
        "lever axle is near column top height",
        lever_aabb is not None
        and lever_aabb[1][2] > COLUMN_TOP_Z - 0.05
        and lever_aabb[0][2] < COLUMN_TOP_Z,
        details=f"lever_aabb={lever_aabb}",
    )

    # --- lever boss support ---
    ctx.expect_contact(
        lever,
        body,
        elem_a="lever_axle",
        elem_b="lever_boss",
        contact_tol=0.003,
        name="lever axle contacts the mounting boss",
    )

    # --- decisive pose: positive rotation lifts the lever ---
    rest_lever_aabb = lever_aabb
    with ctx.pose({joint: LEVER_RANGE}):
        rotated_aabb = ctx.part_world_aabb(lever)
        ctx.check(
            "positive lever rotation lifts the handle tip upward",
            rest_lever_aabb is not None
            and rotated_aabb is not None
            and rotated_aabb[0][2] > rest_lever_aabb[0][2] + 0.005,
            details=f"rest={rest_lever_aabb}, rotated={rotated_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
