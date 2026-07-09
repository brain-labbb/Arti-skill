from __future__ import annotations

"""Single-hole basin faucet variant with rectangular slot outlet and side lever.

A compact single-hole basin faucet (~0.20 m tall) with:
- Vertical cylindrical body (~0.055 m diameter) on a round base flange
- Oval base gasket beneath the flange
- Forward-projecting spout with a flat rectangular slot outlet (hollow tube)
- Side-mounted lever on a short horizontal axle (right side of body)

Articulation:
- ``lever_tilt``: revolute about the horizontal sideways (Y) axle axis,
  -40..+40 deg; positive q lifts the lever tip upward (flow control).
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
# Dimensions (meters). World frame: +X forward (spout direction), +Z up,
# lever on the -Y (right) side of the body.
# ----------------------------------------------------------------------------
BODY_RADIUS = 0.0275  # 0.055 m diameter column
BODY_HEIGHT = 0.200
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

# Oval gasket
GASKET_SEMI_MAJOR = 0.038  # outer extent along X
GASKET_SEMI_MINOR = 0.032  # outer extent along Y
GASKET_CS_RADIUS = 0.004   # cross-section radius of the gasket ring
GASKET_Z = -0.001           # sits just below z=0

# Spout geometry
SPOUT_ROOT_Z = 0.140        # where spout exits the body
SPOUT_LENGTH = 0.130        # forward reach of spout
SPOUT_OUTER_W = 0.028       # outer width (Y) of rectangular tube
SPOUT_OUTER_H = 0.016       # outer height (Z) of rectangular tube
SPOUT_WALL = 0.003          # wall thickness

# Slot outlet dimensions (inner opening at mouth)
SLOT_W = SPOUT_OUTER_W - 2 * SPOUT_WALL  # ~0.022 m
SLOT_H = SPOUT_OUTER_H - 2 * SPOUT_WALL  # ~0.010 m

# Side lever
AXLE_RADIUS = 0.008
AXLE_LENGTH = 0.016
AXLE_CENTER_Y = -0.0345     # axle inner end overlaps body wall (seated embed)
AXLE_CENTER_Z = 0.165       # near the top of the body

LEVER_ARM_WIDTH = 0.012
LEVER_ARM_HEIGHT = 0.006
LEVER_ARM_LENGTH = 0.100    # lever extends 0.10 m from axle

LEVER_TILT_RANGE = math.radians(40.0)


def _build_spout() -> cq.Workplane:
    """Hollow rectangular tube spout swept forward and slightly downward.

    The outlet at the mouth end is naturally a flat rectangular slot (the
    hollow interior opening). Built in spout-local coordinates with the
    channel centerline starting at the origin heading +X.
    """
    # Path: forward and slightly down at the tip
    path = cq.Workplane("XZ").spline(
        [
            (0.000, 0.000),
            (0.040, -0.002),
            (0.080, -0.008),
            (0.110, -0.018),
            (SPOUT_LENGTH, -0.032),
        ]
    )
    hw = SPOUT_OUTER_W / 2.0
    hh = SPOUT_OUTER_H / 2.0
    ihw = hw - SPOUT_WALL
    ihh = hh - SPOUT_WALL

    # Hollow rectangular cross-section (outer rectangle minus inner rectangle)
    profile = (
        cq.Workplane("YZ")
        .rect(SPOUT_OUTER_W, SPOUT_OUTER_H)
        .rect(SPOUT_OUTER_W - 2 * SPOUT_WALL, SPOUT_OUTER_H - 2 * SPOUT_WALL)
    )
    return profile.sweep(path)


def _build_gasket() -> cq.Workplane:
    """Oval (elliptical) ring gasket sitting beneath the base flange.

    Built as an elliptical torus: a circle cross-section swept along an
    elliptical path.
    """
    # Elliptical path (centerline of the gasket ring)
    rx = GASKET_SEMI_MAJOR - GASKET_CS_RADIUS
    ry = GASKET_SEMI_MINOR - GASKET_CS_RADIUS

    # Build elliptical path as a wire using an ellipse
    path_wire = (
        cq.Workplane("XY")
        .ellipse(rx, ry)
    )

    # Cross-section circle at one point on the path
    gasket = (
        cq.Workplane("XZ")
        .center(rx, 0.0)
        .circle(GASKET_CS_RADIUS)
        .sweep(path_wire)
    )
    return gasket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")

    # Oval base gasket (sits just below z=0, under the flange)
    body.visual(
        mesh_from_cadquery(_build_gasket(), "oval_gasket"),
        origin=Origin(xyz=(0.0, 0.0, GASKET_Z)),
        material="rubber_black",
        name="oval_gasket",
    )

    # Base flange
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )

    # Body column
    column_len = BODY_HEIGHT - 0.010
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, 0.010 + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )

    # Spout with hollow rectangular slot outlet
    body.visual(
        mesh_from_cadquery(_build_spout(), "spout_tube"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_tube",
    )

    # Flat top cap (small disc closing the top of the column)
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.002)),
        material="brushed_steel",
        name="top_cap",
    )

    # ---------------------------------------------------------------- lever
    # Side lever: axle + lever arm as one rigid part, rotating about the
    # horizontal Y-axis through the axle center.
    lever = model.part("side_lever")

    # Short horizontal axle cylinder (along Y axis)
    lever.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="lever_axle",
    )

    # Lever arm: slim rectangular bar extending forward (+X) from the axle
    lever.visual(
        Box((LEVER_ARM_LENGTH, LEVER_ARM_WIDTH, LEVER_ARM_HEIGHT)),
        origin=Origin(xyz=(LEVER_ARM_LENGTH / 2.0 + AXLE_LENGTH / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="lever_arm",
    )

    # Small rounded knob at lever tip for grip
    lever.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(
            xyz=(LEVER_ARM_LENGTH + AXLE_LENGTH / 2.0 + 0.002, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bright_steel",
        name="lever_grip",
    )

    model.articulation(
        "lever_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, AXLE_CENTER_Y, AXLE_CENTER_Z)),
        # -Y so positive q lifts the forward (+X) lever tip upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-LEVER_TILT_RANGE, upper=LEVER_TILT_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    lever = object_model.get_part("side_lever")
    tilt = object_model.get_articulation("lever_tilt")
    spout = body.get_visual("spout_tube")
    gasket = body.get_visual("oval_gasket")
    axle = lever.get_visual("lever_axle")
    arm = lever.get_visual("lever_arm")

    # Intentional seated embedding: axle seats into the body wall.
    ctx.allow_overlap(
        lever,
        body,
        elem_a="lever_axle",
        elem_b="body_column",
        reason="lever axle is seated 1.5 mm into the curved body wall",
    )

    # --- static form -------------------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and aabb[0][2] > -0.010 and aabb[0][2] < 0.002,
        f"base flange must sit near z=0, got {aabb}",
    )
    ctx.check(
        "body_height_about_0p20",
        aabb is not None and 0.195 < aabb[1][2] < 0.210,
        f"body top should be ~0.20 m up, got {aabb}",
    )

    # Spout projects forward
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.100,
        f"spout should reach >0.10 m forward, got {spout_aabb}",
    )

    # Rectangular slot outlet: the spout cross-section width (Y) should be
    # SPOUT_OUTER_W (~0.028 m), confirming flat rectangular profile.
    # The overall Z extent is larger due to the downward curve of the spout.
    ctx.check(
        "slot_outlet_width",
        spout_aabb is not None
        and abs((spout_aabb[1][1] - spout_aabb[0][1]) - SPOUT_OUTER_W) < 0.005,
        f"spout outer width should be ~{SPOUT_OUTER_W} m, got {spout_aabb}",
    )
    # Verify the spout is a flat rectangular tube (wider in Y than Z per section).
    # The overall Z span is dominated by the curved path, but the tube height
    # SPOUT_OUTER_H (0.016 m) must be less than the width SPOUT_OUTER_W (0.028 m).
    ctx.check(
        "flat_rectangular_spout_profile",
        SPOUT_OUTER_H < SPOUT_OUTER_W,
        "spout must be flat rectangular (height < width) for slot outlet",
    )

    # Oval gasket exists and has correct proportions (wider in X than Y)
    gasket_aabb = ctx.part_element_world_aabb(body, elem=gasket)
    ctx.check(
        "oval_gasket_exists",
        gasket_aabb is not None,
        "oval gasket must be present",
    )
    if gasket_aabb is not None:
        gasket_dx = gasket_aabb[1][0] - gasket_aabb[0][0]
        gasket_dy = gasket_aabb[1][1] - gasket_aabb[0][1]
        ctx.check(
            "oval_gasket_elliptical",
            gasket_dx > gasket_dy,
            f"gasket should be wider in X (oval), got dx={gasket_dx:.4f}, dy={gasket_dy:.4f}",
        )
        ctx.check(
            "gasket_below_flange",
            gasket_aabb[1][2] < FLANGE_HEIGHT + 0.001,
            f"gasket should sit at or below the flange, got {gasket_aabb}",
        )

    # --- joint plan --------------------------------------------------------
    ctx.check(
        "tilt_axis_sideways",
        abs(tilt.axis[1]) == 1.0 and tilt.axis[0] == 0.0 and tilt.axis[2] == 0.0,
        f"tilt must rotate about horizontal Y axis, got {tilt.axis}",
    )
    ctx.check(
        "tilt_range_pm40deg",
        tilt.motion_limits is not None
        and abs(tilt.motion_limits.lower + LEVER_TILT_RANGE) < 1e-6
        and abs(tilt.motion_limits.upper - LEVER_TILT_RANGE) < 1e-6,
        "tilt range must be -40..+40 deg",
    )

    # Lever axle contacts body (mounted)
    ctx.expect_contact(body, lever, name="axle_seats_on_body")

    # --- motion proof ------------------------------------------------------
    # At rest, lever arm extends roughly horizontally forward
    rest_arm_aabb = ctx.part_element_world_aabb(lever, elem=arm)

    # Tilt up: lever tip sweeps upward
    with ctx.pose({tilt: LEVER_TILT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(lever, elem=arm)
        ctx.check(
            "tilt_up_raises_lever",
            up_aabb is not None and up_aabb[1][2] > 0.220,
            f"at +40 deg the arm should rise, got {up_aabb}",
        )

    # Tilt down: lever tip sweeps downward
    with ctx.pose({tilt: -LEVER_TILT_RANGE}):
        down_aabb = ctx.part_element_world_aabb(lever, elem=arm)
        ctx.check(
            "tilt_down_lowers_lever",
            down_aabb is not None and down_aabb[0][2] < 0.120,
            f"at -40 deg the arm should drop, got {down_aabb}",
        )

    # Prove non-fixed joint: lever arm element z-position changes between poses
    with ctx.pose({tilt: LEVER_TILT_RANGE}):
        up_arm_aabb = ctx.part_element_world_aabb(lever, elem=arm)
    with ctx.pose({tilt: -LEVER_TILT_RANGE}):
        down_arm_aabb = ctx.part_element_world_aabb(lever, elem=arm)
    ctx.check(
        "joint_is_nonfixed",
        up_arm_aabb is not None and down_arm_aabb is not None
        and (up_arm_aabb[1][2] - down_arm_aabb[0][2]) > 0.050,
        f"lever arm must move between tilt poses: up_max_z={up_arm_aabb[1] if up_arm_aabb else None}, down_min_z={down_arm_aabb[0] if down_arm_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
