from __future__ import annotations

"""Single-hole basin faucet with offset side lever housing and cylindrical flow knob.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A round escutcheon base plate carries a slim cylindrical column (~0.20 m tall).
- A flat rectangular spout blade cantilevers forward from the column top,
  with a round aerator outlet recessed in its underside near the tip.
- An offset side lever housing (cylindrical) is mounted to the column's
  right side (+Y) via a rectangular mounting boss.
- A cylindrical flow knob with ribbed grip sits on top of the housing
  and rotates about a vertical axis for flow control (0..90 deg).
- A lever handle extends from the housing outer side and tilts about a
  horizontal axis for temperature mixing (0..25 deg).
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Key dimensions (meters)
# ----------------------------------------------------------------------------
BASE_R = 0.028
BASE_H = 0.008

COLUMN_R = 0.015
COLUMN_TOP_Z = 0.200

SPOUT_WIDTH_Y = 0.038
SPOUT_THICK_Z = 0.014
SPOUT_BACK_X = COLUMN_R * 0.5  # starts slightly inside column for visual connection
SPOUT_TIP_X = 0.135
SPOUT_LEN = SPOUT_TIP_X - SPOUT_BACK_X
SPOUT_TOP_Z = COLUMN_TOP_Z - 0.002  # 0.198
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.184

OUTLET_X = 0.120
AERATOR_OUTER_R = 0.009
AERATOR_INNER_R = 0.006
AERATOR_H = 0.006

# Side lever housing (offset to +Y)
HOUSING_R = 0.018
HOUSING_H = 0.040
HOUSING_OFFSET_Y = 0.036
HOUSING_CENTER_Z = 0.155
HOUSING_BOT_Z = HOUSING_CENTER_Z - HOUSING_H / 2.0  # 0.135
HOUSING_TOP_Z = HOUSING_CENTER_Z + HOUSING_H / 2.0  # 0.175

# Mounting boss connecting housing to column
BOSS_X = 0.022
BOSS_Y_INNER = 0.012  # slightly inside column radius for visual merge
BOSS_Y_OUTER = 0.020  # slightly inside housing inner surface
BOSS_DEPTH_Y = BOSS_Y_OUTER - BOSS_Y_INNER  # 0.008
BOSS_H = 0.028
BOSS_CENTER_Z = HOUSING_CENTER_Z

# Flow knob on top of housing
KNOB_DIAMETER = 0.032
KNOB_HEIGHT = 0.018

# Lever handle extending from housing outer wall
LEVER_LEN = 0.055
LEVER_WIDTH = 0.013
LEVER_THICK = 0.010
LEVER_PIVOT_Y = HOUSING_OFFSET_Y + HOUSING_R  # 0.054 at housing outer wall

# Joint ranges
KNOB_RANGE = math.radians(90.0)
LEVER_RANGE = math.radians(25.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    red_dot = model.material("hot_red", rgba=(0.80, 0.08, 0.08, 1.0))
    blue_dot = model.material("cold_blue", rgba=(0.10, 0.30, 0.78, 1.0))
    brushed = model.material("brushed_chrome", rgba=(0.78, 0.80, 0.83, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: base, column, spout, aerator, housing
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Round escutcheon base plate
    body.visual(
        Cylinder(radius=BASE_R, length=BASE_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=chrome,
        name="base_escutcheon",
    )

    # Cylindrical column
    column_h = COLUMN_TOP_Z - BASE_H
    body.visual(
        Cylinder(radius=COLUMN_R, length=column_h),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + column_h / 2.0)),
        material=chrome,
        name="column",
    )

    # Flat spout blade cantilevering forward
    body.visual(
        Box((SPOUT_LEN, SPOUT_WIDTH_Y, SPOUT_THICK_Z)),
        origin=Origin(
            xyz=((SPOUT_BACK_X + SPOUT_TIP_X) / 2.0, 0.0, SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0)
        ),
        material=chrome,
        name="spout_blade",
    )

    # Hollow chrome aerator ring under spout tip
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
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.003 + 0.004)),
        material=dark,
        name="outlet_disc",
    )

    # Mounting boss bridging column to housing
    boss_y_center = (BOSS_Y_INNER + BOSS_Y_OUTER) / 2.0
    body.visual(
        Box((BOSS_X, BOSS_DEPTH_Y, BOSS_H)),
        origin=Origin(xyz=(0.0, boss_y_center, BOSS_CENTER_Z)),
        material=chrome,
        name="housing_boss",
    )

    # Cylindrical housing shell (offset to +Y side)
    body.visual(
        Cylinder(radius=HOUSING_R, length=HOUSING_H),
        origin=Origin(xyz=(0.0, HOUSING_OFFSET_Y, HOUSING_BOT_Z + HOUSING_H / 2.0)),
        material=chrome,
        name="housing_shell",
    )

    # Small lip ring at housing top (visual detail)
    body.visual(
        Cylinder(radius=HOUSING_R + 0.001, length=0.002),
        origin=Origin(xyz=(0.0, HOUSING_OFFSET_Y, HOUSING_TOP_Z - 0.002)),
        material=chrome,
        name="housing_lip",
    )

    # ------------------------------------------------------------------
    # Lever handle: tilts about horizontal axis at housing outer wall
    # Part frame at the pivot point; arm extends along +Y.
    # ------------------------------------------------------------------
    lever = model.part("lever_handle")

    # Lever arm extending outward (+Y in part frame)
    lever.visual(
        Box((LEVER_WIDTH, LEVER_LEN, LEVER_THICK)),
        origin=Origin(xyz=(0.0, LEVER_LEN / 2.0, 0.0)),
        material=chrome,
        name="lever_arm",
    )

    # Pivot collar at the lever base
    lever.visual(
        Cylinder(radius=0.008, length=LEVER_THICK + 0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_collar",
    )

    # Temperature indicator dots on lever arm side face
    lever.visual(
        Cylinder(radius=0.002, length=0.002),
        origin=Origin(
            xyz=(LEVER_WIDTH / 2.0 + 0.001, 0.015, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=red_dot,
        name="hot_dot",
    )
    lever.visual(
        Cylinder(radius=0.002, length=0.002),
        origin=Origin(
            xyz=(LEVER_WIDTH / 2.0 + 0.001, 0.035, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=blue_dot,
        name="cold_dot",
    )

    model.articulation(
        "lever_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, LEVER_PIVOT_Y, HOUSING_CENTER_Z)),
        # Axis along +X: positive q raises the +Y lever tip (right-hand rule).
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0, lower=0.0, upper=LEVER_RANGE
        ),
    )

    # ------------------------------------------------------------------
    # Flow knob: cylindrical knob with ribbed grip, rotates on housing top
    # Part frame at the knob mounting face (bottom of knob).
    # ------------------------------------------------------------------
    knob = model.part("flow_knob")

    knob_geo = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="ribbed", count=14, depth=0.0006, width=0.0012),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,  # mounting face at z=0
    )
    knob.visual(
        mesh_from_geometry(knob_geo, "knob_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed,
        name="knob_body",
    )

    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, HOUSING_OFFSET_Y, HOUSING_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=5.0, lower=0.0, upper=KNOB_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    lever = object_model.get_part("lever_handle")
    knob = object_model.get_part("flow_knob")
    lever_tilt = object_model.get_articulation("lever_tilt")
    knob_rotate = object_model.get_articulation("knob_rotate")

    # --- joint plan ---
    ctx.check(
        "knob_rotate is revolute about vertical axis, 0..90 deg",
        knob_rotate.articulation_type == ArticulationType.REVOLUTE
        and abs(knob_rotate.axis[2] - 1.0) < 1e-9
        and abs(knob_rotate.axis[0]) < 1e-9
        and abs(knob_rotate.axis[1]) < 1e-9
        and knob_rotate.motion_limits is not None
        and abs(knob_rotate.motion_limits.lower) < 1e-9
        and abs(knob_rotate.motion_limits.upper - math.radians(90.0)) < 1e-6,
        details=f"axis={knob_rotate.axis}, limits={knob_rotate.motion_limits}",
    )
    ctx.check(
        "lever_tilt is revolute about horizontal X axis, 0..25 deg",
        lever_tilt.articulation_type == ArticulationType.REVOLUTE
        and abs(lever_tilt.axis[0] - 1.0) < 1e-9
        and abs(lever_tilt.axis[1]) < 1e-9
        and abs(lever_tilt.axis[2]) < 1e-9
        and lever_tilt.motion_limits is not None
        and abs(lever_tilt.motion_limits.lower) < 1e-9
        and abs(lever_tilt.motion_limits.upper - math.radians(25.0)) < 1e-6,
        details=f"axis={lever_tilt.axis}, limits={lever_tilt.motion_limits}",
    )
    ctx.check(
        "both non-fixed joints parent to faucet_body",
        lever_tilt.parent == body.name
        and knob_rotate.parent == body.name
        and lever_tilt.child == lever.name
        and knob_rotate.child == knob.name,
        details=(
            f"lever_tilt: {lever_tilt.parent}->{lever_tilt.child}, "
            f"knob_rotate: {knob_rotate.parent}->{knob_rotate.child}"
        ),
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base escutcheon grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )

    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check(
        "compact basin faucet height ~0.20-0.22 m",
        body_aabb is not None
        and knob_aabb is not None
        and 0.18 <= max(body_aabb[1][2], knob_aabb[1][2]) <= 0.24,
        details=f"body_top={body_aabb[1][2] if body_aabb else None}, knob_top={knob_aabb[1][2] if knob_aabb else None}",
    )

    # --- offset side housing: knob is offset from column center ---
    ctx.check(
        "flow knob is offset to the side (mounted on side housing)",
        knob_aabb is not None
        and (knob_aabb[0][1] + knob_aabb[1][1]) / 2.0 > 0.020,
        details=f"knob center y={(knob_aabb[0][1] + knob_aabb[1][1]) / 2.0 if knob_aabb else None}",
    )

    # --- spout reach ---
    ctx.check(
        "spout blade cantilevers ~0.12 m forward of column center",
        body_aabb is not None and 0.10 <= body_aabb[1][0] <= 0.15,
        details=f"body max x={body_aabb[1][0] if body_aabb else None}",
    )

    # --- aerator and outlet ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="aerator_collar")
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_disc")
    ctx.check(
        "aerator collar under spout near tip",
        collar_aabb is not None
        and collar_aabb[0][2] < SPOUT_BOT_Z
        and 0.10 <= (collar_aabb[0][0] + collar_aabb[1][0]) / 2.0 <= SPOUT_TIP_X,
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

    # --- knob grip grooves: KnobGeometry produces a mesh-backed visual ---
    knob_visual = knob.get_visual("knob_body")
    ctx.check(
        "flow knob uses KnobGeometry with ribbed grip (mesh-backed)",
        knob_visual is not None and hasattr(knob_visual.geometry, "filename"),
    )

    # --- mounting: lever pivot collar contacts housing shell ---
    ctx.allow_overlap(
        lever,
        body,
        elem_a="pivot_collar",
        elem_b="housing_shell",
        reason="Pivot collar is half-seated in the housing wall at the lever mount point.",
    )
    ctx.expect_contact(
        lever,
        body,
        elem_a="pivot_collar",
        elem_b="housing_shell",
        contact_tol=0.005,
        name="lever pivot collar contacts the housing shell",
    )

    # --- lever arm clears the knob vertically ---
    ctx.expect_gap(
        knob,
        lever,
        axis="z",
        min_gap=-0.002,
        max_gap=0.025,
        name="flow knob and lever handle do not vertically clash",
    )

    # --- lever arm root contacts the housing shell at the pivot point ---
    ctx.expect_contact(
        lever,
        body,
        elem_a="lever_arm",
        elem_b="housing_shell",
        contact_tol=0.003,
        name="lever arm root contacts the housing shell for visual mounting",
    )

    # --- knob sits on housing top ---
    ctx.expect_overlap(
        knob,
        body,
        axes="xy",
        min_overlap=0.010,
        elem_a="knob_body",
        elem_b="housing_shell",
        name="flow knob footprint covers the housing top",
    )

    # --- temperature dots on lever ---
    hot_aabb = ctx.part_element_world_aabb(lever, elem="hot_dot")
    cold_aabb = ctx.part_element_world_aabb(lever, elem="cold_dot")
    ctx.check(
        "red/blue temperature dots visible on lever arm side",
        hot_aabb is not None
        and cold_aabb is not None
        and hot_aabb[1][0] > cold_aabb[0][0],  # dots are proud on the +X face
        details=f"hot={hot_aabb}, cold={cold_aabb}",
    )

    # --- decisive pose: lever tilt raises the lever tip ---
    rest_lever_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({lever_tilt: LEVER_RANGE}):
        tilted_aabb = ctx.part_world_aabb(lever)
        ctx.check(
            "positive lever tilt raises the lever tip upward",
            rest_lever_aabb is not None
            and tilted_aabb is not None
            and tilted_aabb[1][2] > rest_lever_aabb[1][2] + 0.003,
            details=f"rest_top={rest_lever_aabb[1][2] if rest_lever_aabb else None}, tilted_top={tilted_aabb[1][2] if tilted_aabb else None}",
        )

    # --- decisive pose: knob rotation ---
    rest_knob_aabb = ctx.part_world_aabb(knob)
    with ctx.pose({knob_rotate: KNOB_RANGE}):
        rotated_knob_aabb = ctx.part_world_aabb(knob)
        ctx.check(
            "knob rotation preserves the knob position (cylindrical symmetry)",
            rest_knob_aabb is not None
            and rotated_knob_aabb is not None
            and abs(rotated_knob_aabb[1][2] - rest_knob_aabb[1][2]) < 0.002,
            details=f"rest={rest_knob_aabb}, rotated={rotated_knob_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
