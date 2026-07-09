from __future__ import annotations

"""Polished-chrome single-hole basin faucet with offset side lever and pull-up drain rod.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A round stepped base plate sits on the deck with one hole penetration.
- A slim rectangular column rises from the base.
- A flat rectangular spout blade cantilevers forward from the column top.
- An offset side lever housing protrudes from the +Y side of the column,
  with a thin cartridge cap seam ring at its base.
- A flat lever handle extends sideways from the housing with grip grooves.
- A pull-up drain rod slides vertically behind the column (-X side).
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
BASE_LOWER_R = 0.038
BASE_LOWER_H = 0.005
BASE_UPPER_R = 0.030
BASE_UPPER_H = 0.010
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H  # 0.015

COLUMN_DEPTH_X = 0.032
COLUMN_WIDTH_Y = 0.038
COLUMN_TOP_Z = 0.220

SPOUT_WIDTH_Y = 0.042
SPOUT_THICK_Z = 0.018
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0
SPOUT_TIP_X = 0.155
SPOUT_TOP_Z = COLUMN_TOP_Z
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z

OUTLET_X = 0.138
AERATOR_OUTER_R = 0.010
AERATOR_INNER_R = 0.007
AERATOR_H = 0.007

# Side lever housing (offset on +Y side of column)
HOUSING_WIDTH_X = 0.030
HOUSING_DEPTH_Y = 0.028
HOUSING_HEIGHT_Z = 0.040
HOUSING_CENTER_Y = COLUMN_WIDTH_Y / 2.0 + HOUSING_DEPTH_Y / 2.0 - 0.005
HOUSING_BOT_Z = COLUMN_TOP_Z - 0.060
HOUSING_TOP_Z = HOUSING_BOT_Z + HOUSING_HEIGHT_Z

# Cartridge cap seam ring at the base of the housing
SEAM_RING_OUTER_R = 0.018
SEAM_RING_INNER_R = 0.014
SEAM_RING_H = 0.003
SEAM_RING_Z = HOUSING_BOT_Z - SEAM_RING_H / 2.0

# Lever handle dimensions
HANDLE_LEN_Y = 0.120
HANDLE_WIDTH_X = 0.028
HANDLE_THICK_Z = 0.011
HANDLE_ORIGIN_Z = HOUSING_BOT_Z + HOUSING_HEIGHT_Z - HANDLE_THICK_Z / 2.0

# Grip grooves on handle (small ridges)
GROOVE_COUNT = 5
GROOVE_DEPTH = 0.0015
GROOVE_WIDTH = 0.003
GROOVE_SPACING = 0.015

# Drain rod (behind column on -X side)
DRAIN_ROD_R = 0.003
DRAIN_ROD_LENGTH = 0.080
DRAIN_ROD_X = -COLUMN_DEPTH_X / 2.0 - DRAIN_ROD_R - 0.003
DRAIN_ROD_REST_Z = COLUMN_TOP_Z - DRAIN_ROD_LENGTH + 0.010
DRAIN_ROD_KNOB_R = 0.008
DRAIN_ROD_KNOB_H = 0.010
DRAIN_SLIDE_RANGE = 0.040

LIFT_RANGE = math.radians(25.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    seam_dark = model.material("seam_ring", rgba=(0.25, 0.25, 0.27, 1.0))
    grip_mat = model.material("grip_rubber", rgba=(0.15, 0.15, 0.17, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: stepped base plate, column, spout blade, aerator,
    # side lever housing, cartridge cap seam
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Round stepped base plate
    body.visual(
        Cylinder(radius=BASE_LOWER_R, length=BASE_LOWER_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H / 2.0)),
        material=chrome,
        name="base_plate_lower",
    )
    body.visual(
        Cylinder(radius=BASE_UPPER_R, length=BASE_UPPER_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H + BASE_UPPER_H / 2.0)),
        material=chrome,
        name="base_plate_upper",
    )

    # Column
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

    # Aerator collar (hollow chrome ring under spout tip)
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
    # Dark outlet recessed inside aerator
    body.visual(
        Cylinder(radius=AERATOR_INNER_R, length=0.005),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.003 + 0.002 + 0.0025)),
        material=dark,
        name="outlet_disc",
    )

    # Offset side lever housing (on +Y side of column)
    body.visual(
        Box((HOUSING_WIDTH_X, HOUSING_DEPTH_Y, HOUSING_HEIGHT_Z)),
        origin=Origin(xyz=(0.0, HOUSING_CENTER_Y, HOUSING_BOT_Z + HOUSING_HEIGHT_Z / 2.0)),
        material=chrome,
        name="lever_housing",
    )

    # Cartridge cap seam ring at base of housing
    seam_ring = (
        cq.Workplane("XY")
        .circle(SEAM_RING_OUTER_R)
        .circle(SEAM_RING_INNER_R)
        .extrude(SEAM_RING_H)
    )
    body.visual(
        mesh_from_cadquery(seam_ring, "cartridge_seam"),
        origin=Origin(xyz=(0.0, HOUSING_CENTER_Y, SEAM_RING_Z)),
        material=seam_dark,
        name="cartridge_seam",
    )

    # ------------------------------------------------------------------
    # Lever handle (articulated, side-mounted, with grip grooves)
    # Origin at the pivot point on the housing outer face
    # ------------------------------------------------------------------
    handle = model.part("lever_handle")

    # Main handle blade extends in +Y from pivot
    handle.visual(
        Box((HANDLE_WIDTH_X, HANDLE_LEN_Y, HANDLE_THICK_Z)),
        origin=Origin(xyz=(0.0, HANDLE_LEN_Y / 2.0, 0.0)),
        material=chrome,
        name="handle_blade",
    )

    # Pivot boss (small cylinder connecting handle to housing)
    handle.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=chrome,
        name="pivot_boss",
    )

    # Grip grooves on handle top surface (small raised ridges)
    for i in range(GROOVE_COUNT):
        groove_y = 0.030 + i * GROOVE_SPACING
        handle.visual(
            Box((HANDLE_WIDTH_X - 0.006, GROOVE_WIDTH, GROOVE_DEPTH)),
            origin=Origin(xyz=(0.0, groove_y, HANDLE_THICK_Z / 2.0 + GROOVE_DEPTH / 2.0)),
            material=grip_mat,
            name=f"grip_groove_{i}",
        )

    # Lever lift articulation: revolute about X axis at housing top face
    # Handle extends in +Y; rotating about +X lifts the far end upward
    pivot_y = HOUSING_CENTER_Y + HOUSING_DEPTH_Y / 2.0
    pivot_z = HOUSING_BOT_Z + HOUSING_HEIGHT_Z
    model.articulation(
        "handle_lift",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, pivot_y, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0, lower=0.0, upper=LIFT_RANGE
        ),
    )

    # ------------------------------------------------------------------
    # Drain rod (prismatic, slides vertically behind column)
    # ------------------------------------------------------------------
    drain = model.part("drain_rod")

    # Rod shaft
    drain.visual(
        Cylinder(radius=DRAIN_ROD_R, length=DRAIN_ROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LENGTH / 2.0)),
        material=chrome,
        name="drain_shaft",
    )

    # Knob on top of rod
    drain.visual(
        Cylinder(radius=DRAIN_ROD_KNOB_R, length=DRAIN_ROD_KNOB_H),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LENGTH + DRAIN_ROD_KNOB_H / 2.0)),
        material=chrome,
        name="drain_knob",
    )

    # Small collar at base of rod (guide ring)
    guide_ring = (
        cq.Workplane("XY")
        .circle(DRAIN_ROD_R + 0.003)
        .circle(DRAIN_ROD_R)
        .extrude(0.006)
    )
    drain.visual(
        mesh_from_cadquery(guide_ring, "drain_guide"),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=chrome,
        name="drain_guide",
    )

    # Prismatic joint: slides upward along +Z
    model.articulation(
        "drain_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain,
        origin=Origin(xyz=(DRAIN_ROD_X, 0.0, DRAIN_ROD_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=2.0, lower=0.0, upper=DRAIN_SLIDE_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    handle = object_model.get_part("lever_handle")
    drain = object_model.get_part("drain_rod")
    lift = object_model.get_articulation("handle_lift")
    slide = object_model.get_articulation("drain_slide")

    # --- joint plan: types, axes, ranges ---
    ctx.check(
        "handle lift is revolute 0..25 deg about horizontal axis",
        lift.articulation_type == ArticulationType.REVOLUTE
        and abs(abs(lift.axis[0]) - 1.0) < 1e-9
        and abs(lift.axis[1]) < 1e-9
        and abs(lift.axis[2]) < 1e-9
        and lift.motion_limits is not None
        and abs(lift.motion_limits.lower - 0.0) < 1e-9
        and abs(lift.motion_limits.upper - math.radians(25.0)) < 1e-6,
        details=f"axis={lift.axis}, limits={lift.motion_limits}",
    )
    ctx.check(
        "drain slide is prismatic along Z with 0..0.04m range",
        slide.articulation_type == ArticulationType.PRISMATIC
        and abs(slide.axis[0]) < 1e-9
        and abs(slide.axis[1]) < 1e-9
        and abs(abs(slide.axis[2]) - 1.0) < 1e-9
        and slide.motion_limits is not None
        and abs(slide.motion_limits.lower - 0.0) < 1e-9
        and abs(slide.motion_limits.upper - DRAIN_SLIDE_RANGE) < 1e-6,
        details=f"axis={slide.axis}, limits={slide.motion_limits}",
    )

    # --- structural: offset side lever housing ---
    housing_aabb = ctx.part_element_world_aabb(body, elem="lever_housing")
    ctx.check(
        "lever housing is offset to the +Y side of the column",
        housing_aabb is not None
        and housing_aabb[0][1] > COLUMN_WIDTH_Y / 2.0 - 0.010,
        details=f"housing_aabb={housing_aabb}",
    )

    # --- cartridge cap seam below the lever ---
    seam_aabb = ctx.part_element_world_aabb(body, elem="cartridge_seam")
    ctx.check(
        "cartridge cap seam ring sits below the lever housing",
        seam_aabb is not None
        and housing_aabb is not None
        and seam_aabb[1][2] <= housing_aabb[0][2] + 0.005,
        details=f"seam_aabb={seam_aabb}, housing_aabb={housing_aabb}",
    )

    # --- grip grooves on handle ---
    groove_names = [v.name for v in handle.visuals if v.name.startswith("grip_groove_")]
    ctx.check(
        f"handle has {GROOVE_COUNT} grip grooves on the surface",
        len(groove_names) >= GROOVE_COUNT,
        details=f"found grooves: {groove_names}",
    )

    # --- drain rod behind the body ---
    drain_shaft_aabb = ctx.part_element_world_aabb(drain, elem="drain_shaft")
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "drain rod is positioned behind the column (-X side)",
        drain_shaft_aabb is not None
        and body_aabb is not None
        and drain_shaft_aabb[1][0] < -COLUMN_DEPTH_X / 2.0,
        details=f"drain_shaft_aabb={drain_shaft_aabb}",
    )

    # --- grounding and scale ---
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "faucet height is ~0.25-0.30m",
        body_aabb is not None and 0.22 <= body_aabb[1][2] <= 0.32,
        details=f"body top z={body_aabb[1][2] if body_aabb else None}",
    )

    # --- spout features ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="aerator_collar")
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_disc")
    ctx.check(
        "aerator collar protrudes below spout underside near tip",
        collar_aabb is not None
        and collar_aabb[0][2] < SPOUT_BOT_Z
        and collar_aabb[0][0] > 0.08,
        details=f"collar_aabb={collar_aabb}",
    )
    ctx.check(
        "dark outlet recessed inside aerator collar",
        collar_aabb is not None
        and outlet_aabb is not None
        and outlet_aabb[0][2] > collar_aabb[0][2] + 0.001
        and outlet_aabb[0][0] > collar_aabb[0][0]
        and outlet_aabb[1][0] < collar_aabb[1][0],
        details=f"outlet={outlet_aabb}, collar={collar_aabb}",
    )

    # --- handle mounting ---
    boss_aabb = ctx.part_element_world_aabb(handle, elem="pivot_boss")
    ctx.expect_contact(
        handle,
        body,
        elem_a="pivot_boss",
        elem_b="lever_housing",
        contact_tol=0.003,
        name="handle pivot boss contacts the lever housing",
    )

    # --- drain rod connectivity to body ---
    ctx.expect_overlap(
        drain,
        body,
        axes="x",
        min_overlap=0.0,
        name="drain rod is near the body on the X axis",
    )

    # --- decisive pose: handle lift raises the grip end ---
    handle_rest_aabb = ctx.part_world_aabb(handle)
    rest_tip_z = handle_rest_aabb[1][2] if handle_rest_aabb is not None else None
    with ctx.pose({lift: LIFT_RANGE}):
        lifted_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive lift raises the handle tip upward",
            rest_tip_z is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_tip_z + 0.02,
            details=f"rest_top={rest_tip_z}, lifted={lifted_aabb}",
        )

    # --- decisive pose: drain rod slides upward ---
    drain_rest_pos = ctx.part_world_position(drain)
    with ctx.pose({slide: DRAIN_SLIDE_RANGE}):
        drain_ext_pos = ctx.part_world_position(drain)
        ctx.check(
            "drain rod slides upward when actuated",
            drain_rest_pos is not None
            and drain_ext_pos is not None
            and drain_ext_pos[2] > drain_rest_pos[2] + DRAIN_SLIDE_RANGE - 0.005,
            details=f"rest={drain_rest_pos}, extended={drain_ext_pos}",
        )

    return ctx.report()


object_model = build_object_model()
