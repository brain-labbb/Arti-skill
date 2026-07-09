from __future__ import annotations

"""Polished-chrome single-lever single-hole basin faucet with rectangular geometry.

Variant 07: fork of the tall vessel faucet into a compact single-hole basin sibling.
Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- An oval rubber gasket sits on the countertop; a square stepped base plate rests on it.
- A slim rectangular column rises from the base.
- A flat rectangular spout blade cantilevers forward from the column top, with a
  hollow rectangular slot outlet cut into its underside near the tip.
- A side lever is mounted on the column's right side (+Y) via a short horizontal
  axle (along X). The lever tilts upward to open flow (revolute, 0..30 deg).
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

# Oval gasket: elliptical ring under the base plate
GASKET_SEMI_A = 0.052  # semi-major along X
GASKET_SEMI_B = 0.042  # semi-minor along Y
GASKET_INNER_A = 0.034  # inner semi-major
GASKET_INNER_B = 0.026  # inner semi-minor
GASKET_THICK = 0.003

COLUMN_DEPTH_X = 0.035
COLUMN_WIDTH_Y = 0.045
COLUMN_TOP_Z = 0.235

SPOUT_WIDTH_Y = 0.050
SPOUT_THICK_Z = 0.020
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0  # flush with column rear face
SPOUT_TIP_X = 0.1825  # ~0.17 m forward reach past the column front face
SPOUT_TOP_Z = COLUMN_TOP_Z  # blade top flush with column top
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.215

# Rectangular slot outlet: hollow channel cut into spout underside near tip
SLOT_WIDTH_Y = 0.032   # slot width (along Y)
SLOT_DEPTH_X = 0.018   # slot length along X (front-back extent of the opening)
SLOT_HEIGHT_Z = 0.012  # how far up into the spout the channel goes
SLOT_CENTER_X = SPOUT_TIP_X - 0.020  # near the tip
SLOT_BOT_Z = SPOUT_BOT_Z  # opens at the spout underside

# Side lever dimensions
AXLE_R = 0.008
AXLE_LEN = 0.018  # protrudes from column side
AXLE_CENTER_Z = 0.160  # height on column where axle is mounted
LEVER_LEN_Y = 0.120  # lever arm extends outward along +Y
LEVER_WIDTH_X = 0.030
LEVER_THICK_Z = 0.012
LEVER_TILT_RANGE = math.radians(30.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("gasket_rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: gasket, stepped base plate, column, spout with hollow slot
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Oval base gasket — elliptical ring sitting on the countertop
    gasket = (
        cq.Workplane("XY")
        .ellipse(GASKET_SEMI_A, GASKET_SEMI_B)
        .ellipse(GASKET_INNER_A, GASKET_INNER_B)
        .extrude(GASKET_THICK)
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
        origin=Origin(xyz=(0.0, 0.0, GASKET_THICK + BASE_LOWER_H / 2.0)),
        material=chrome,
        name="base_plate_lower",
    )
    body.visual(
        Box((BASE_UPPER_SIDE, BASE_UPPER_SIDE, BASE_UPPER_H)),
        origin=Origin(xyz=(0.0, 0.0, GASKET_THICK + BASE_LOWER_H + BASE_UPPER_H / 2.0)),
        material=chrome,
        name="base_plate_upper",
    )

    # Column
    column_base_z = GASKET_THICK + BASE_TOP_Z
    column_h = COLUMN_TOP_Z - column_base_z
    body.visual(
        Box((COLUMN_DEPTH_X, COLUMN_WIDTH_Y, column_h)),
        origin=Origin(xyz=(0.0, 0.0, column_base_z + column_h / 2.0)),
        material=chrome,
        name="column",
    )

    # Spout blade with hollow rectangular slot channel cut from underside
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    spout_solid = (
        cq.Workplane("XY")
        .box(spout_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z)
    )
    # Cut rectangular channel from underside near the tip to create hollow outlet
    # The channel is open at the bottom (spout underside) forming the slot
    slot_cutter = (
        cq.Workplane("XY")
        .transformed(offset=(SLOT_CENTER_X - (SPOUT_BACK_X + SPOUT_TIP_X) / 2.0,
                             0.0,
                             -SPOUT_THICK_Z / 2.0 + SLOT_HEIGHT_Z / 2.0 - 0.001))
        .box(SLOT_DEPTH_X, SLOT_WIDTH_Y, SLOT_HEIGHT_Z + 0.002)
    )
    spout_hollow = spout_solid.cut(slot_cutter)
    body.visual(
        mesh_from_cadquery(spout_hollow, "spout_blade"),
        origin=Origin(
            xyz=((SPOUT_BACK_X + SPOUT_TIP_X) / 2.0, 0.0, SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0)
        ),
        material=chrome,
        name="spout_blade",
    )

    # Dark outlet recess inside the hollow channel (visible through the slot).
    # Extends from just below the spout underside up into the channel ceiling
    # so it contacts the spout blade inner surface for part connectivity.
    outlet_h = SLOT_HEIGHT_Z + 0.002
    body.visual(
        Box((SLOT_DEPTH_X - 0.004, SLOT_WIDTH_Y - 0.004, outlet_h)),
        origin=Origin(
            xyz=(SLOT_CENTER_X, 0.0, SPOUT_BOT_Z + outlet_h / 2.0 - 0.001)
        ),
        material=dark,
        name="outlet_slot",
    )

    # Axle stub on column right side (+Y face) — fixed to body
    body.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(
            xyz=(0.0, COLUMN_WIDTH_Y / 2.0 + AXLE_LEN / 2.0, AXLE_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="lever_axle",
    )

    # ------------------------------------------------------------------
    # Side lever: tilts on the horizontal axle (flow control)
    # Child frame at the axle center, on the column side face.
    # ------------------------------------------------------------------
    lever = model.part("side_lever")

    # Lever arm extends outward along +Y from the axle end
    # Arm origin: centered on axle tip, extends along +Y
    lever.visual(
        Box((LEVER_WIDTH_X, LEVER_LEN_Y, LEVER_THICK_Z)),
        origin=Origin(
            xyz=(0.0, LEVER_LEN_Y / 2.0, 0.0)
        ),
        material=chrome,
        name="lever_arm",
    )

    # Lever grip cap at the tip (slightly wider for ergonomics)
    lever.visual(
        Box((LEVER_WIDTH_X + 0.006, 0.015, LEVER_THICK_Z + 0.004)),
        origin=Origin(
            xyz=(0.0, LEVER_LEN_Y - 0.005, 0.0)
        ),
        material=chrome,
        name="lever_grip",
    )

    # Axle collar that wraps around the fixed axle (visual hub)
    collar_outer = AXLE_R + 0.004
    collar_len = 0.012
    collar = (
        cq.Workplane("XY")
        .circle(collar_outer)
        .circle(AXLE_R - 0.001)
        .extrude(collar_len)
    )
    lever.visual(
        mesh_from_cadquery(collar, "lever_collar"),
        origin=Origin(xyz=(0.0, -collar_len / 2.0, 0.0)),
        material=chrome,
        name="lever_collar",
    )

    model.articulation(
        "lever_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        # Joint frame at axle center on column side
        origin=Origin(xyz=(0.0, COLUMN_WIDTH_Y / 2.0 + AXLE_LEN, AXLE_CENTER_Z)),
        # Axle along X-axis; positive q tilts lever tip upward (flow on)
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0, lower=0.0, upper=LEVER_TILT_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    lever = object_model.get_part("side_lever")
    tilt = object_model.get_articulation("lever_tilt")

    # --- joint plan: at least one non-fixed revolute joint ---
    ctx.check(
        "lever_tilt is revolute about horizontal X-axis, 0..30 deg",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and abs(abs(tilt.axis[0]) - 1.0) < 1e-9
        and abs(tilt.axis[1]) < 1e-9
        and abs(tilt.axis[2]) < 1e-9
        and tilt.motion_limits is not None
        and abs(tilt.motion_limits.lower - 0.0) < 1e-9
        and abs(tilt.motion_limits.upper - math.radians(30.0)) < 1e-6,
        details=f"axis={tilt.axis}, limits={tilt.motion_limits}",
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base gasket is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height ~0.25-0.30 m",
        body_aabb is not None and 0.23 <= body_aabb[1][2] <= 0.32,
        details=f"body top z={None if body_aabb is None else body_aabb[1][2]}",
    )
    ctx.check(
        "spout cantilevers ~0.17 m forward of column front face",
        body_aabb is not None and 0.14 <= body_aabb[1][0] - COLUMN_DEPTH_X / 2.0 <= 0.20,
        details=f"body max x={None if body_aabb is None else body_aabb[1][0]}",
    )

    # --- oval base gasket exists and is wider than the base plate ---
    gasket_aabb = ctx.part_element_world_aabb(body, elem="base_gasket")
    ctx.check(
        "oval gasket sits at ground level under the base",
        gasket_aabb is not None
        and abs(gasket_aabb[0][2]) < 1e-4
        and gasket_aabb[1][2] < 0.006,
        details=f"gasket_aabb={gasket_aabb}",
    )
    ctx.check(
        "gasket is oval (wider along X than Y)",
        gasket_aabb is not None
        and (gasket_aabb[1][0] - gasket_aabb[0][0]) > (gasket_aabb[1][1] - gasket_aabb[0][1]) + 0.005,
        details=f"gasket_aabb={gasket_aabb}",
    )

    # --- hollow rectangular slot outlet at spout mouth ---
    slot_aabb = ctx.part_element_world_aabb(body, elem="outlet_slot")
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_blade")
    ctx.check(
        "outlet slot is a flat rectangular opening (wider than tall)",
        slot_aabb is not None
        and (slot_aabb[1][1] - slot_aabb[0][1]) > (slot_aabb[1][2] - slot_aabb[0][2]) + 0.005,
        details=f"slot_aabb={slot_aabb}",
    )
    ctx.check(
        "outlet slot is recessed inside the spout blade underside",
        slot_aabb is not None
        and spout_aabb is not None
        and slot_aabb[0][2] >= spout_aabb[0][2] - 0.001
        and slot_aabb[1][2] < spout_aabb[1][2],
        details=f"slot={slot_aabb}, spout={spout_aabb}",
    )
    ctx.check(
        "outlet slot is near the spout tip (forward half)",
        slot_aabb is not None
        and spout_aabb is not None
        and (slot_aabb[0][0] + slot_aabb[1][0]) / 2.0 > (spout_aabb[0][0] + spout_aabb[1][0]) / 2.0,
        details=f"slot center x={slot_aabb}, spout={spout_aabb}",
    )

    # --- side lever on horizontal axle ---
    lever_aabb = ctx.part_world_aabb(lever)
    axle_aabb = ctx.part_element_world_aabb(body, elem="lever_axle")
    ctx.check(
        "side lever extends outward along +Y from the column side",
        lever_aabb is not None
        and axle_aabb is not None
        and lever_aabb[1][1] > COLUMN_WIDTH_Y / 2.0 + 0.05,
        details=f"lever_aabb={lever_aabb}",
    )
    ctx.check(
        "lever axle is on the column side (above base, below spout)",
        axle_aabb is not None
        and axle_aabb[0][2] > 0.08
        and axle_aabb[1][2] < COLUMN_TOP_Z,
        details=f"axle_aabb={axle_aabb}",
    )

    # --- mounting: lever collar wraps the fixed axle (intentional bearing fit) ---
    ctx.allow_overlap(
        body,
        lever,
        elem_a="lever_axle",
        elem_b="lever_collar",
        reason="The lever collar wraps around the fixed axle stub to represent a bearing hub fit.",
    )
    ctx.expect_overlap(
        lever,
        body,
        axes="y",
        min_overlap=0.005,
        elem_a="lever_collar",
        elem_b="lever_axle",
        name="lever collar overlaps the axle hub region along Y",
    )
    ctx.expect_overlap(
        lever,
        body,
        axes="xz",
        min_overlap=0.005,
        elem_a="lever_collar",
        elem_b="lever_axle",
        name="lever collar is centered on the axle in XZ",
    )

    # --- decisive pose: positive tilt raises the lever tip ---
    rest_lever_aabb = lever_aabb
    rest_tip_z = rest_lever_aabb[1][2] if rest_lever_aabb is not None else None
    with ctx.pose({tilt: LEVER_TILT_RANGE}):
        tilted_aabb = ctx.part_world_aabb(lever)
        ctx.check(
            "positive lever tilt raises the grip tip upward (flow on)",
            rest_tip_z is not None
            and tilted_aabb is not None
            and tilted_aabb[1][2] > rest_tip_z + 0.02,
            details=f"rest_tip_z={rest_tip_z}, tilted_aabb={tilted_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
