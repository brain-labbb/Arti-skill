from __future__ import annotations

"""Single-hole basin faucet with top-mounted flow knob.

A vertical cylindrical body (~0.20 m tall, ~0.055 m diameter) on a wider round
base flange with a raised circular collar above it. A flat-bottomed
open-channel spout projects forward and slightly down from the front of the
body. A cylindrical flow knob with ribbed grip sits on top and rotates about
the vertical axis for flow control. Two small screw caps sit on the back of the
body.

Articulation:
- ``knob_rotate``: revolute about the vertical (Z) axis at body top,
  -90..+90 deg; rotation controls water flow rate.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
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
# Dimensions (meters). World frame: +X forward (spout direction), +Z up.
# ----------------------------------------------------------------------------
BODY_RADIUS = 0.0275  # 0.055 m diameter column
BODY_HEIGHT = 0.200
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

# Raised collar around the base (between flange and column)
COLLAR_OUTER_RADIUS = 0.033
COLLAR_HEIGHT = 0.014

SPOUT_ROOT_Z = 0.118  # channel centerline height where it leaves the body
SPOUT_OUTER_W = 0.034
SPOUT_OUTER_H = 0.022
SPOUT_WALL = 0.005
SPOUT_FLOOR = 0.006

# Flow knob on top
KNOB_DIAMETER = 0.038
KNOB_HEIGHT = 0.022
KNOB_SHAFT_RADIUS = 0.006
KNOB_SHAFT_HEIGHT = 0.008

# Screw caps on back (-Y) of body
SCREW_CAP_RADIUS = 0.004
SCREW_CAP_LENGTH = 0.003
SCREW_CAP_Z_LOWER = 0.065
SCREW_CAP_Z_UPPER = 0.140

KNOB_RANGE = math.radians(90.0)


def _build_spout() -> cq.Workplane:
    """Open-top U-channel swept forward and curving down at the tip."""
    path = cq.Workplane("XZ").spline(
        [
            (0.000, 0.000),
            (0.060, -0.004),
            (0.105, -0.014),
            (0.135, -0.028),
            (0.155, -0.048),
        ]
    )
    hw = SPOUT_OUTER_W / 2.0
    hh = SPOUT_OUTER_H / 2.0
    inner_hw = hw - SPOUT_WALL
    floor_v = -hh + SPOUT_FLOOR
    profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-hw, hh),
                (-hw, -hh),
                (hw, -hh),
                (hw, hh),
                (inner_hw, hh),
                (inner_hw, floor_v),
                (-inner_hw, floor_v),
                (-inner_hw, hh),
            ]
        )
        .close()
    )
    return profile.sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_steel", rgba=(0.55, 0.56, 0.58, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    # Base flange
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )
    # Raised circular collar (solid ring around column, hidden overlap with column)
    body.visual(
        Cylinder(radius=COLLAR_OUTER_RADIUS, length=COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT + COLLAR_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_collar",
    )
    # Main column
    column_len = BODY_HEIGHT - 0.010
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, 0.010 + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    # Flat top cap for the body
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.0015)),
        material="brushed_steel",
        name="body_top_cap",
    )
    # Spout
    body.visual(
        mesh_from_cadquery(_build_spout(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )

    # Knob shaft boss (small cylinder protruding from top of body)
    body.visual(
        Cylinder(radius=KNOB_SHAFT_RADIUS, length=KNOB_SHAFT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + KNOB_SHAFT_HEIGHT / 2.0)),
        material="dark_steel",
        name="knob_shaft",
    )

    # Screw caps on the back (-Y side) of the body
    # Lower screw cap
    body.visual(
        Cylinder(radius=SCREW_CAP_RADIUS, length=SCREW_CAP_LENGTH),
        origin=Origin(
            xyz=(0.0, -(BODY_RADIUS + SCREW_CAP_LENGTH / 2.0 - 0.001), SCREW_CAP_Z_LOWER),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="dark_steel",
        name="screw_cap_lower",
    )
    # Upper screw cap
    body.visual(
        Cylinder(radius=SCREW_CAP_RADIUS, length=SCREW_CAP_LENGTH),
        origin=Origin(
            xyz=(0.0, -(BODY_RADIUS + SCREW_CAP_LENGTH / 2.0 - 0.001), SCREW_CAP_Z_UPPER),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="dark_steel",
        name="screw_cap_upper",
    )

    # -------------------------------------------------------- flow knob
    flow_knob = model.part("flow_knob")
    # Knob body with ribbed grip grooves
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="ribbed", count=16, depth=0.001, width=0.0018),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,
    )
    flow_knob.visual(
        mesh_from_geometry(knob_geom, "flow_knob_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="bright_steel",
        name="knob_body",
    )
    # Flow pointer tab - extends radially from the knob edge (+X direction)
    # so rotation about Z visibly moves the tab position.
    pointer_length = 0.012
    pointer_width = 0.005
    flow_knob.visual(
        Cylinder(radius=pointer_width / 2.0, length=pointer_length),
        origin=Origin(
            xyz=(KNOB_DIAMETER / 2.0 + pointer_length / 2.0 - 0.002, 0.0, KNOB_HEIGHT * 0.7),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="dark_steel",
        name="flow_pointer",
    )

    # Articulation: knob rotates about vertical (Z) axis on top of body
    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flow_knob,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + KNOB_SHAFT_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=-KNOB_RANGE, upper=KNOB_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    knob = object_model.get_part("flow_knob")
    knob_joint = object_model.get_articulation("knob_rotate")
    spout = body.get_visual("spout_channel")
    collar = body.get_visual("base_collar")
    knob_body = knob.get_visual("knob_body")
    screw_lower = body.get_visual("screw_cap_lower")
    screw_upper = body.get_visual("screw_cap_upper")
    column = body.get_visual("body_column")

    # Intentional overlap: knob shaft boss is seated into the knob bore
    ctx.allow_overlap(
        body,
        knob,
        elem_a="knob_shaft",
        elem_b="knob_body",
        reason="knob shaft boss is seated inside the knob bore for captured rotation",
    )

    # --- static form checks ------------------------------------------------

    # Body sits on ground
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and abs(aabb[0][2]) < 1e-6,
        f"base flange must sit on z=0, got {aabb}",
    )
    # Body height ~0.20m (plus knob on top)
    ctx.check(
        "body_height_about_0p20",
        aabb is not None and 0.195 < aabb[1][2] < 0.245,
        f"body top should be ~0.20 m up (knob adds height), got {aabb}",
    )

    # Spout projects forward
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.145,
        f"channel spout should reach ~0.155 m forward, got {spout_aabb}",
    )

    # Collar exists and is wider than column
    collar_aabb = ctx.part_element_world_aabb(body, elem=collar)
    column_aabb = ctx.part_element_world_aabb(body, elem=column)
    ctx.check(
        "collar_wider_than_column",
        collar_aabb is not None
        and column_aabb is not None
        and (collar_aabb[1][0] - collar_aabb[0][0])
        > (column_aabb[1][0] - column_aabb[0][0]) + 0.002,
        f"collar must be wider than body column: collar={collar_aabb}, column={column_aabb}",
    )
    # Collar sits above flange (collar bottom above flange top)
    ctx.check(
        "collar_above_flange",
        collar_aabb is not None
        and collar_aabb[0][2] >= FLANGE_HEIGHT - 0.001,
        f"collar should sit at or above flange top ({FLANGE_HEIGHT}m), got {collar_aabb}",
    )

    # Knob sits on top of body
    knob_aabb = ctx.part_element_world_aabb(knob, elem=knob_body)
    ctx.check(
        "knob_on_top_of_body",
        knob_aabb is not None and knob_aabb[0][2] > BODY_HEIGHT - 0.005,
        f"knob should sit on top of body (>0.195m), got {knob_aabb}",
    )

    # Knob diameter check
    ctx.check(
        "knob_diameter_correct",
        knob_aabb is not None
        and abs((knob_aabb[1][0] - knob_aabb[0][0]) - KNOB_DIAMETER) < 0.004,
        f"knob should be ~{KNOB_DIAMETER}m diameter, got {knob_aabb}",
    )

    # Screw caps on back of body
    screw_lower_aabb = ctx.part_element_world_aabb(body, elem=screw_lower)
    screw_upper_aabb = ctx.part_element_world_aabb(body, elem=screw_upper)
    ctx.check(
        "screw_caps_on_back",
        screw_lower_aabb is not None
        and screw_upper_aabb is not None
        and screw_lower_aabb[0][1] < -BODY_RADIUS + 0.002
        and screw_upper_aabb[0][1] < -BODY_RADIUS + 0.002,
        f"screw caps must be on -Y (back) side of body: lower={screw_lower_aabb}, upper={screw_upper_aabb}",
    )
    ctx.check(
        "screw_caps_vertically_separated",
        screw_lower_aabb is not None
        and screw_upper_aabb is not None
        and abs(
            (screw_upper_aabb[0][2] + screw_upper_aabb[1][2]) / 2.0
            - (screw_lower_aabb[0][2] + screw_lower_aabb[1][2]) / 2.0
        )
        > 0.050,
        "two screw caps should be separated vertically by >0.05m",
    )

    # --- joint plan checks -------------------------------------------------
    ctx.check(
        "knob_axis_vertical",
        abs(knob_joint.axis[2]) == 1.0
        and knob_joint.axis[0] == 0.0
        and knob_joint.axis[1] == 0.0,
        f"knob must rotate about vertical Z axis, got {knob_joint.axis}",
    )
    ctx.check(
        "knob_range_pm90deg",
        knob_joint.motion_limits is not None
        and abs(knob_joint.motion_limits.lower + KNOB_RANGE) < 1e-6
        and abs(knob_joint.motion_limits.upper - KNOB_RANGE) < 1e-6,
        "knob range must be -90..+90 deg",
    )
    ctx.check(
        "knob_joint_is_revolute",
        knob_joint.articulation_type == ArticulationType.REVOLUTE,
        f"knob joint must be revolute, got {knob_joint.articulation_type}",
    )

    # --- motion proof: flow pointer orbits the Z axis ---------------------
    pointer = knob.get_visual("flow_pointer")
    rest_pointer = ctx.part_element_world_aabb(knob, elem=pointer)
    with ctx.pose({knob_joint: KNOB_RANGE}):
        rotated_pointer = ctx.part_element_world_aabb(knob, elem=pointer)
        ctx.check(
            "knob_rotation_moves_pointer",
            rest_pointer is not None
            and rotated_pointer is not None
            and (
                abs(rest_pointer[0][0] - rotated_pointer[0][0]) > 0.003
                or abs(rest_pointer[1][0] - rotated_pointer[1][0]) > 0.003
                or abs(rest_pointer[0][1] - rotated_pointer[0][1]) > 0.003
                or abs(rest_pointer[1][1] - rotated_pointer[1][1]) > 0.003
            ),
            f"knob rotation should move the pointer: rest={rest_pointer}, rotated={rotated_pointer}",
        )

    # Knob stays on top at rotated pose (Z position unchanged)
    with ctx.pose({knob_joint: KNOB_RANGE}):
        rotated_knob_aabb = ctx.part_world_aabb(knob)
        ctx.check(
            "knob_stays_on_top_when_rotated",
            rotated_knob_aabb is not None and rotated_knob_aabb[0][2] > BODY_HEIGHT - 0.005,
            f"knob should stay on top when rotated, got {rotated_knob_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
