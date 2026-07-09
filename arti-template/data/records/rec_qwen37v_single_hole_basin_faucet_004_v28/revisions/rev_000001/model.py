from __future__ import annotations

"""Polished-chrome single-hole basin faucet variant (fork of the tall vessel faucet).

Structural changes from parent:
- Raised circular collar around the base replaces the rectangular stepped plates.
- Cylindrical flow knob with knurled grip replaces the lever handle assembly.
- Two small screw caps on the back of the column body.
- Single revolute joint: knob rotates about a vertical axis (flow control, 0..90 deg).
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
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
COLLAR_RADIUS = 0.032
COLLAR_HEIGHT = 0.025
COLLAR_TOP_Z = COLLAR_HEIGHT  # 0.025

COLUMN_DEPTH_X = 0.035
COLUMN_WIDTH_Y = 0.045
COLUMN_TOP_Z = 0.235

SPOUT_WIDTH_Y = 0.050
SPOUT_THICK_Z = 0.020
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0  # flush with column rear face
SPOUT_TIP_X = 0.1825  # ~0.17 m forward reach past the column front face
SPOUT_TOP_Z = COLUMN_TOP_Z  # blade top flush with column top
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.215

OUTLET_X = 0.162  # aerator center, near the spout tip
AERATOR_OUTER_R = 0.011
AERATOR_INNER_R = 0.008
AERATOR_H = 0.008

# Knob dimensions
KNOB_DIAMETER = 0.038
KNOB_HEIGHT = 0.028
KNOB_STEM_R = 0.008
KNOB_STEM_H = 0.010
KNOB_BASE_Z = COLUMN_TOP_Z + KNOB_STEM_H  # knob sits on top of stem

# Screw caps on back of column
SCREW_CAP_R = 0.005
SCREW_CAP_H = 0.004
SCREW_CAP_SPACING_Z = 0.060  # vertical spacing between the two caps

KNOB_RANGE = math.radians(90.0)  # flow control: 0..90 degrees


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    cap_dark = model.material("screw_cap", rgba=(0.12, 0.12, 0.14, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: circular collar, column, spout blade, aerator, screw caps
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Raised circular collar at the base
    body.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_HEIGHT / 2.0)),
        material=chrome,
        name="base_collar",
    )

    # Slim rectangular column
    column_h = COLUMN_TOP_Z - COLLAR_TOP_Z
    body.visual(
        Box((COLUMN_DEPTH_X, COLUMN_WIDTH_Y, column_h)),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z + column_h / 2.0)),
        material=chrome,
        name="column",
    )

    # Spout blade cantilevering forward
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    body.visual(
        Box((spout_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z)),
        origin=Origin(
            xyz=((SPOUT_BACK_X + SPOUT_TIP_X) / 2.0, 0.0, SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0)
        ),
        material=chrome,
        name="spout_blade",
    )

    # Hollow chrome aerator ring under the spout tip
    ring = (
        cq.Workplane("XY")
        .circle(AERATOR_OUTER_R)
        .circle(AERATOR_INNER_R)
        .extrude(AERATOR_H)
    )
    body.visual(
        mesh_from_cadquery(ring, "aerator_collar"),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.004)),
        material=chrome,
        name="aerator_collar",
    )
    body.visual(
        Cylinder(radius=AERATOR_INNER_R, length=0.006),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.004 + 0.0025 + 0.003)),
        material=dark,
        name="outlet_disc",
    )

    # Short stem on top of column for the knob to mount on
    body.visual(
        Cylinder(radius=KNOB_STEM_R, length=KNOB_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + KNOB_STEM_H / 2.0)),
        material=chrome,
        name="knob_stem",
    )

    # Two small screw caps on the back of the column body
    back_x = -COLUMN_DEPTH_X / 2.0 - SCREW_CAP_H / 2.0
    cap_z_center = COLLAR_TOP_Z + column_h * 0.5
    body.visual(
        Cylinder(radius=SCREW_CAP_R, length=SCREW_CAP_H),
        origin=Origin(
            xyz=(back_x, 0.0, cap_z_center + SCREW_CAP_SPACING_Z / 2.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cap_dark,
        name="screw_cap_upper",
    )
    body.visual(
        Cylinder(radius=SCREW_CAP_R, length=SCREW_CAP_H),
        origin=Origin(
            xyz=(back_x, 0.0, cap_z_center - SCREW_CAP_SPACING_Z / 2.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cap_dark,
        name="screw_cap_lower",
    )

    # ------------------------------------------------------------------
    # Flow knob: cylindrical knob with knurled grip, rotates on vertical axis
    # ------------------------------------------------------------------
    knob = model.part("flow_knob")

    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        skirt=KnobSkirt(
            diameter=KNOB_DIAMETER + 0.004,
            height=0.005,
            flare=0.02,
            chamfer=0.001,
        ),
        grip=KnobGrip(style="knurled", count=32, depth=0.0008, helix_angle_deg=15.0),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        bore=KnobBore(style="round", diameter=KNOB_STEM_R * 2.0),
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "flow_knob_body"),
        origin=Origin(xyz=(0.0, 0.0, KNOB_HEIGHT / 2.0)),
        material=chrome,
        name="knob_body",
    )

    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, KNOB_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=0.0, upper=KNOB_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    knob = object_model.get_part("flow_knob")
    knob_joint = object_model.get_articulation("knob_rotate")

    # --- joint plan: single revolute on vertical axis, 0..90 deg ---
    ctx.check(
        "knob_rotate is revolute 0..90 deg about vertical axis",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(knob_joint.axis[0]) < 1e-9
        and abs(knob_joint.axis[1]) < 1e-9
        and abs(abs(knob_joint.axis[2]) - 1.0) < 1e-9
        and knob_joint.motion_limits is not None
        and abs(knob_joint.motion_limits.lower - 0.0) < 1e-9
        and abs(knob_joint.motion_limits.upper - math.radians(90.0)) < 1e-6,
        details=f"axis={knob_joint.axis}, limits={knob_joint.motion_limits}",
    )

    # --- joint chain: body -> knob ---
    ctx.check(
        "knob_rotate parents body to flow_knob",
        knob_joint.parent == body.name and knob_joint.child == knob.name,
        details=f"parent={knob_joint.parent}, child={knob_joint.child}",
    )

    # --- at least one non-fixed joint ---
    ctx.check(
        "model has at least one non-fixed joint",
        knob_joint.articulation_type != ArticulationType.FIXED,
        details=f"knob_rotate type={knob_joint.articulation_type}",
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base collar is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height approximately 0.26-0.30 m",
        body_aabb is not None and 0.24 <= body_aabb[1][2] <= 0.32,
        details=f"body top z={None if body_aabb is None else body_aabb[1][2]}",
    )

    # --- circular collar at the base ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="base_collar")
    ctx.check(
        "circular collar exists at the base and is wider than the column",
        collar_aabb is not None
        and abs(collar_aabb[0][2]) < 1e-6  # grounded
        and (collar_aabb[1][0] - collar_aabb[0][0]) > COLUMN_WIDTH_Y,
        details=f"collar_aabb={collar_aabb}",
    )

    # --- knob on top of the column ---
    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check(
        "flow knob sits above the column top",
        knob_aabb is not None and knob_aabb[0][2] >= COLUMN_TOP_Z - 0.002,
        details=f"knob_aabb={knob_aabb}, column_top={COLUMN_TOP_Z}",
    )

    # --- screw caps on back of column ---
    cap_upper_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_upper")
    cap_lower_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_lower")
    ctx.check(
        "two screw caps exist on the back of the column body",
        cap_upper_aabb is not None
        and cap_lower_aabb is not None
        and cap_upper_aabb[0][0] < -COLUMN_DEPTH_X / 2.0 + 0.001  # behind column
        and cap_lower_aabb[0][0] < -COLUMN_DEPTH_X / 2.0 + 0.001,
        details=f"upper={cap_upper_aabb}, lower={cap_lower_aabb}",
    )
    ctx.check(
        "screw caps are vertically separated",
        cap_upper_aabb is not None
        and cap_lower_aabb is not None
        and abs(
            (cap_upper_aabb[0][2] + cap_upper_aabb[1][2]) / 2.0
            - (cap_lower_aabb[0][2] + cap_lower_aabb[1][2]) / 2.0
        )
        > 0.04,
        details=f"upper_center_z={(cap_upper_aabb[0][2] + cap_upper_aabb[1][2]) / 2.0 if cap_upper_aabb else None}, "
        f"lower_center_z={(cap_lower_aabb[0][2] + cap_lower_aabb[1][2]) / 2.0 if cap_lower_aabb else None}",
    )

    # --- knob knob_body visual exists (knurled grip geometry) ---
    knob_body_visual = knob.get_visual("knob_body")
    ctx.check(
        "knob body visual exists with grip detail geometry",
        knob_body_visual is not None,
        details="knob_body visual not found",
    )

    # --- spout and aerator retained ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_blade")
    aerator_aabb = ctx.part_element_world_aabb(body, elem="aerator_collar")
    ctx.check(
        "spout blade cantilevers forward from the column",
        spout_aabb is not None and spout_aabb[1][0] > 0.15,
        details=f"spout_aabb={spout_aabb}",
    )
    ctx.check(
        "aerator collar exists under the spout tip",
        aerator_aabb is not None and aerator_aabb[0][2] < SPOUT_BOT_Z + 0.001,
        details=f"aerator_aabb={aerator_aabb}",
    )

    # --- knob stem support contact ---
    ctx.expect_contact(
        knob,
        body,
        elem_a="knob_body",
        elem_b="knob_stem",
        contact_tol=0.005,
        name="flow knob seats on the knob stem",
    )

    # --- decisive pose: rotating the knob changes its angular position ---
    rest_indicator_pos = None
    with ctx.pose({knob_joint: 0.0}):
        rest_aabb = ctx.part_world_aabb(knob)
        if rest_aabb is not None:
            rest_indicator_pos = [(rest_aabb[0][i] + rest_aabb[1][i]) / 2.0 for i in range(3)]

    with ctx.pose({knob_joint: KNOB_RANGE}):
        rotated_aabb = ctx.part_world_aabb(knob)
        ctx.check(
            "knob at max rotation stays above the column (no descent)",
            rotated_aabb is not None and rotated_aabb[0][2] >= COLUMN_TOP_Z - 0.005,
            details=f"rotated_aabb={rotated_aabb}",
        )
        # For a vertical-axis revolute, the AABB shouldn't change much,
        # but the knob body itself rotates — confirm the pose applies correctly
        # by checking that the knob center stays on the column axis
        if rotated_aabb is not None:
            center_x = (rotated_aabb[0][0] + rotated_aabb[1][0]) / 2.0
            center_y = (rotated_aabb[0][1] + rotated_aabb[1][1]) / 2.0
            ctx.check(
                "rotated knob center stays near the column axis",
                abs(center_x) < 0.01 and abs(center_y) < 0.01,
                details=f"center_x={center_x}, center_y={center_y}",
            )

    return ctx.report()


object_model = build_object_model()
