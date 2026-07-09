from __future__ import annotations

"""Polished-chrome single-lever single-hole basin faucet with detachable-look spout.

Variant 29 — forked from the tall vessel faucet into a compact single-hole
basin faucet sibling.  Structural changes over the parent:
- Spout has a visible rectangular collar seam at the column junction that
  gives a detachable / serviceable appearance.
- A proper hollow outlet tube (open-ended chrome annulus) protrudes below
  the spout underside near the tip — a real bore, not a solid plug.
- A separate circular aerator insert part is seated inside the hollow
  outlet tube, connected by a FIXED joint.
- The top lever still lifts (flow, 0..25 deg) and swivels (temperature,
  -45..+45 deg) on revolute joints.
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
SPOUT_TIP_X = 0.1825
SPOUT_TOP_Z = COLUMN_TOP_Z
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.215

# Spout collar seam (detachable look)
COLLAR_THICK_X = 0.006
COLLAR_OVERSIZE = 0.006  # extra width/height beyond spout cross-section

# Hollow outlet tube at spout mouth
OUTLET_X = 0.162  # tube center X, near spout tip
OUTLET_OUTER_R = 0.012
OUTLET_INNER_R = 0.009
OUTLET_TUBE_H = 0.014  # protrudes below spout underside
OUTLET_EMBED = 0.002  # top of tube embeds into spout for visual connection

# Aerator insert (separate part)
AERATOR_DISC_R = 0.0075
AERATOR_DISC_H = 0.003
AERATOR_RIM_OUTER_R = 0.0091  # press-fit interference with the tube bore (0.009)
AERATOR_RIM_INNER_R = 0.0075  # matches disc radius
AERATOR_RIM_H = 0.004

POST_R = 0.013
POST_H = 0.013
POST_TOP_Z = COLUMN_TOP_Z + POST_H  # 0.248

BLOCK_DEPTH_X = 0.045
BLOCK_WIDTH_Y = 0.044
BLOCK_H = 0.0365
BLOCK_TOP_REL = BLOCK_H

HANDLE_LEN_X = 0.170
HANDLE_WIDTH_Y = 0.050
HANDLE_THICK_Z = 0.013
HANDLE_FLOAT = 0.0015
HANDLE_REAR_REL_X = -BLOCK_DEPTH_X / 2.0

LIFT_RANGE = math.radians(25.0)
SWIVEL_RANGE = math.radians(45.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    red = model.material("hot_red", rgba=(0.80, 0.08, 0.08, 1.0))
    blue = model.material("cold_blue", rgba=(0.10, 0.30, 0.78, 1.0))
    aerator_mat = model.material("aerator_mesh", rgba=(0.50, 0.52, 0.55, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: stepped base, column, spout blade, collar seam,
    # hollow outlet tube, mounting post
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
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    body.visual(
        Box((spout_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z)),
        origin=Origin(
            xyz=((SPOUT_BACK_X + SPOUT_TIP_X) / 2.0, 0.0,
                 SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0)
        ),
        material=chrome,
        name="spout_blade",
    )

    # --- Spout collar seam ---
    # Rectangular frame that wraps around the spout at the column junction,
    # giving a visible service / detach seam.
    collar_cz = SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0
    spout_collar = (
        cq.Workplane("YZ")
        .rect(SPOUT_WIDTH_Y + COLLAR_OVERSIZE, SPOUT_THICK_Z + COLLAR_OVERSIZE)
        .rect(SPOUT_WIDTH_Y + 0.002, SPOUT_THICK_Z + 0.002)
        .extrude(COLLAR_THICK_X / 2.0, both=True)
    )
    body.visual(
        mesh_from_cadquery(spout_collar, "spout_collar_seam"),
        origin=Origin(xyz=(SPOUT_BACK_X + COLLAR_THICK_X / 2.0, 0.0, collar_cz)),
        material=chrome,
        name="spout_collar_seam",
    )

    # --- Hollow outlet tube ---
    # Open-ended chrome annulus protruding below the spout underside.
    outlet_tube = (
        cq.Workplane("XY")
        .circle(OUTLET_OUTER_R)
        .circle(OUTLET_INNER_R)
        .extrude(OUTLET_TUBE_H)
    )
    body.visual(
        mesh_from_cadquery(outlet_tube, "outlet_tube"),
        origin=Origin(
            xyz=(OUTLET_X, 0.0,
                 SPOUT_BOT_Z - OUTLET_TUBE_H + OUTLET_EMBED)
        ),
        material=chrome,
        name="outlet_tube",
    )
    # Dark ceiling disc inside the tube top (visible from below).
    # Sized to match the tube bore and tall enough to bridge into the
    # spout blade, ensuring it reads as one connected body island.
    body.visual(
        Cylinder(radius=OUTLET_INNER_R, length=0.006),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.002)),
        material=dark,
        name="outlet_ceiling",
    )

    # Mounting post for handle assembly
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + POST_H / 2.0)),
        material=chrome,
        name="mounting_post",
    )

    # ------------------------------------------------------------------
    # Aerator insert — separate circular part seated in the outlet tube
    # ------------------------------------------------------------------
    aerator = model.part("aerator_insert")
    # Main mesh disc (the aerator screen body)
    aerator.visual(
        Cylinder(radius=AERATOR_DISC_R, length=AERATOR_DISC_H),
        material=aerator_mat,
        name="aerator_disc",
    )
    # Chrome retaining rim ring around the disc
    aerator_rim = (
        cq.Workplane("XY")
        .circle(AERATOR_RIM_OUTER_R)
        .circle(AERATOR_RIM_INNER_R)
        .extrude(AERATOR_RIM_H / 2.0, both=True)
    )
    aerator.visual(
        mesh_from_cadquery(aerator_rim, "aerator_rim"),
        material=chrome,
        name="aerator_rim",
    )

    # FIXED articulation seats the insert inside the outlet tube bore.
    # The aerator part origin is placed near the bottom of the tube.
    aerator_z = SPOUT_BOT_Z - OUTLET_TUBE_H + OUTLET_EMBED + AERATOR_RIM_H / 2.0 + 0.001
    model.articulation(
        "aerator_seat",
        ArticulationType.FIXED,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(OUTLET_X, 0.0, aerator_z)),
    )

    # ------------------------------------------------------------------
    # Swivel stage: lever pivot block (temperature, -45..+45 deg)
    # ------------------------------------------------------------------
    block = model.part("lever_pivot_block")
    block.visual(
        Box((BLOCK_DEPTH_X, BLOCK_WIDTH_Y, BLOCK_H)),
        origin=Origin(xyz=(0.0, 0.0, BLOCK_H / 2.0)),
        material=chrome,
        name="pivot_block",
    )
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
            effort=8.0, velocity=3.0,
            lower=-SWIVEL_RANGE, upper=SWIVEL_RANGE,
        ),
    )

    # ------------------------------------------------------------------
    # Lift stage: flat rectangular lever handle (flow, 0..25 deg)
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
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0,
            lower=0.0, upper=LIFT_RANGE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    block = object_model.get_part("lever_pivot_block")
    handle = object_model.get_part("lever_handle")
    aerator = object_model.get_part("aerator_insert")
    swivel = object_model.get_articulation("handle_swivel")
    lift = object_model.get_articulation("handle_lift")
    aerator_seat = object_model.get_articulation("aerator_seat")

    # ---- joint plan: types, axes, ranges ----
    ctx.check(
        "lift joint is revolute 0..25 deg about horizontal left-right axis",
        lift.articulation_type == ArticulationType.REVOLUTE
        and abs(lift.axis[0]) < 1e-9
        and abs(abs(lift.axis[1]) - 1.0) < 1e-9
        and abs(lift.axis[2]) < 1e-9
        and lift.motion_limits is not None
        and abs(lift.motion_limits.lower) < 1e-9
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
        "aerator seat is FIXED, joining insert to the faucet body",
        aerator_seat.articulation_type == ArticulationType.FIXED
        and aerator_seat.parent == body.name
        and aerator_seat.child == aerator.name,
        details=(
            f"type={aerator_seat.articulation_type}, "
            f"parent={aerator_seat.parent}, child={aerator_seat.child}"
        ),
    )
    ctx.check(
        "at least two non-fixed joints exist (lift + swivel)",
        sum(
            1
            for a in object_model.articulations
            if a.articulation_type in (ArticulationType.REVOLUTE,
                                       ArticulationType.CONTINUOUS,
                                       ArticulationType.PRISMATIC)
        ) >= 2,
    )

    # ---- variant: spout collar seam ----
    collar_aabb = ctx.part_element_world_aabb(body, elem="spout_collar_seam")
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_blade")
    ctx.check(
        "spout collar seam wraps the spout root wider than the blade",
        collar_aabb is not None
        and spout_aabb is not None
        and collar_aabb[0][0] < spout_aabb[0][0] + 0.02
        and (collar_aabb[1][1] - collar_aabb[0][1]) > SPOUT_WIDTH_Y + 0.003,
        details=f"collar_aabb={collar_aabb}",
    )

    # ---- variant: hollow outlet tube ----
    tube_aabb = ctx.part_element_world_aabb(body, elem="outlet_tube")
    ctx.check(
        "hollow outlet tube protrudes below the spout underside near the tip",
        tube_aabb is not None
        and tube_aabb[0][2] < SPOUT_BOT_Z - 0.005
        and 0.14 <= (tube_aabb[0][0] + tube_aabb[1][0]) / 2.0 <= SPOUT_TIP_X,
        details=f"tube_aabb={tube_aabb}",
    )

    # ---- variant: separate aerator insert ----
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator insert is a separate part positioned near the outlet",
        aerator_aabb is not None
        and aerator_aabb[0][2] < SPOUT_BOT_Z
        and aerator_aabb[1][0] > OUTLET_X - 0.02
        and aerator_aabb[0][0] < OUTLET_X + 0.02,
        details=f"aerator_aabb={aerator_aabb}",
    )
    ctx.expect_within(
        aerator, body,
        axes="xy",
        margin=0.005,
        name="aerator insert sits within the outlet tube XY footprint",
    )
    # The aerator insert is a press-fit seated inside the hollow outlet tube
    # bore.  The tiny radial gap / interference between rim and tube wall is
    # intentional for a detachable insert representation.
    ctx.allow_overlap(
        body, aerator,
        elem_a="outlet_tube", elem_b="aerator_rim",
        reason=(
            "The aerator rim is intentionally represented as a press-fit "
            "insert inside the outlet tube bore (0.1 mm interference)."
        ),
    )
    ctx.allow_isolated_part(
        aerator,
        reason=(
            "The aerator insert is a detachable press-fit seated inside the "
            "hollow outlet tube, connected by a FIXED articulation."
        ),
    )

    # ---- grounding and scale ----
    body_aabb = ctx.part_world_aabb(body)
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height ~0.30 m",
        handle_aabb is not None and 0.28 <= handle_aabb[1][2] <= 0.32,
        details=f"handle_aabb={handle_aabb}",
    )
    ctx.check(
        "spout blade cantilevers ~0.17 m forward of the column front face",
        body_aabb is not None
        and 0.16 <= body_aabb[1][0] - COLUMN_DEPTH_X / 2.0 <= 0.19,
        details=f"body max x={None if body_aabb is None else body_aabb[1][0]}",
    )

    # ---- mounting checks ----
    ctx.expect_contact(
        block, body,
        elem_a="pivot_block", elem_b="mounting_post",
        contact_tol=1e-5,
        name="pivot block seats on the chrome mounting post",
    )
    ctx.expect_contact(
        handle, block,
        elem_a="pivot_heel", elem_b="pivot_block",
        contact_tol=1e-5,
        name="handle pivot heel seats on the pivot block top",
    )
    ctx.expect_gap(
        handle, block,
        axis="z",
        min_gap=0.0005, max_gap=0.004,
        positive_elem="handle_blade", negative_elem="pivot_block",
        name="handle blade floats slightly above the pivot block",
    )
    ctx.expect_gap(
        handle, body,
        axis="z",
        min_gap=0.03,
        name="handle assembly stays clear above the fixed spout blade",
    )
    ctx.expect_overlap(
        handle, block,
        axes="xy",
        min_overlap=0.02,
        name="handle blade root covers the pivot block footprint",
    )

    # ---- decisive pose checks ----
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
            handle, block,
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
            handle, body,
            axis="z",
            min_gap=0.03,
            name="swiveled handle still clears the fixed spout",
        )

    return ctx.report()


object_model = build_object_model()
