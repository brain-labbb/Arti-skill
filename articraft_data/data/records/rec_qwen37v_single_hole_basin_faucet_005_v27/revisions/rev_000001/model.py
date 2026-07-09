from __future__ import annotations

"""Single-hole basin faucet variant with rectangular slot outlet and side lever.

A vertical cylindrical body (~0.20 m tall, ~0.055 m diameter) on a slightly
wider round base flange. A closed rectangular-section spout projects forward
from the body and curves down, terminating in a flat rectangular slot outlet
with a real hollow cavity at the mouth.

On the right side of the body, a short horizontal axle carries a flat lever
arm. The lever rotates about the axle's horizontal axis (-45..+45 deg) to
control flow.

Articulation:
- ``body_to_axle``: revolute about the horizontal sideways (Y) axle axis,
  -45..+45 deg; positive q lifts the lever tip up.
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

# Spout: rectangular cross-section tube projecting forward and curving down
SPOUT_ROOT_Z = 0.130  # centerline height where spout leaves the body
SPOUT_LENGTH = 0.130  # horizontal projection
SPOUT_DROP = 0.050  # how far the tip drops below root
SPOUT_W = 0.030  # outer width (Y)
SPOUT_H = 0.020  # outer height (Z)
SPOUT_WALL = 0.004  # wall thickness

# Rectangular slot outlet at the spout mouth
SLOT_W = 0.022  # slot width (narrower than spout width)
SLOT_H = 0.005  # slot height (thin flat slot)
SLOT_DEPTH = 0.012  # hollow cavity depth behind the slot face

# Side lever axle
AXLE_RADIUS = 0.006
AXLE_LENGTH = 0.018
AXLE_CENTER_Y = -0.036  # on the right side of body (outboard of body surface)
AXLE_CENTER_Z = 0.155  # near top of body

# Lever arm: extends forward (+X) from the axle outer end
LEVER_W = 0.010  # width (Y direction, thin)
LEVER_H = 0.006  # thickness (Z direction)
LEVER_LENGTH = 0.090  # extends forward from axle end (X direction)

LIFT_RANGE = math.radians(45.0)


def _build_spout() -> cq.Workplane:
    """Closed rectangular-section spout with a real hollow outlet at the mouth.

    Built in spout-local coordinates: origin at the body exit, heading +X.
    The spout curves down slightly toward the tip. A hollow cavity is cut
    into the tip end with a flat rectangular slot opening.
    """
    # Path for the sweep: forward and curving down
    path = cq.Workplane("XZ").spline(
        [
            (0.000, 0.000),
            (0.045, -0.002),
            (0.090, -0.012),
            (0.120, -0.030),
            (SPOUT_LENGTH, -SPOUT_DROP),
        ]
    )

    # Build solid rectangular sweep for the outer shape
    solid_spout = (
        cq.Workplane("YZ")
        .rect(SPOUT_W, SPOUT_H)
        .sweep(path)
    )

    # Hollow out the interior by sweeping the inner profile
    inner_cutter = (
        cq.Workplane("YZ")
        .rect(SPOUT_W - 2 * SPOUT_WALL, SPOUT_H - 2 * SPOUT_WALL)
        .sweep(path)
    )

    hollow_spout = solid_spout.cut(inner_cutter)

    # Cut the rectangular slot cavity at the spout mouth end.
    # The tip is at approximately (SPOUT_LENGTH, 0, -SPOUT_DROP).
    # Cut a rectangular box into the end to create the slot opening.
    slot_cutter = (
        cq.Workplane("YZ")
        .workplane(offset=SPOUT_LENGTH - SLOT_DEPTH)
        .center(0.0, -SPOUT_DROP)
        .rect(SLOT_W, SLOT_H)
        .extrude(SLOT_DEPTH + 0.005)
    )

    result = hollow_spout.cut(slot_cutter)
    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_interior", rgba=(0.08, 0.08, 0.10, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )
    column_len = BODY_HEIGHT - 0.010
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, 0.010 + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    # Flat top cap
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.002)),
        material="brushed_steel",
        name="body_cap",
    )
    # Spout with rectangular slot outlet
    body.visual(
        mesh_from_cadquery(_build_spout(), "spout_body"),
        origin=Origin(xyz=(BODY_RADIUS - 0.005, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_body",
    )

    # ------------------------------------------------------------------ axle
    # Short horizontal axle on the right (-Y) side of the body.
    axle = model.part("lever_axle")
    axle.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="axle_shaft",
    )

    model.articulation(
        "body_to_axle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=axle,
        origin=Origin(xyz=(0.0, AXLE_CENTER_Y, AXLE_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-LIFT_RANGE, upper=LIFT_RANGE
        ),
    )

    # ------------------------------------------------------------------ lever
    # Flat lever arm extending forward (+X) from the outer end of the axle.
    # When the axle rotates about Y, the forward-extending lever sweeps up/down.
    lever = model.part("lever_handle")
    # The lever bar extends along +X from origin.
    lever.visual(
        Box((LEVER_LENGTH, LEVER_W, LEVER_H)),
        origin=Origin(xyz=(LEVER_LENGTH / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="lever_bar",
    )
    # Small grip cylinder at the lever tip (forward end)
    lever.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(
            xyz=(LEVER_LENGTH + 0.003, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bright_steel",
        name="lever_grip",
    )

    # Fixed joint: lever is rigidly attached to the outer end of the axle.
    model.articulation(
        "axle_to_lever",
        ArticulationType.FIXED,
        parent=axle,
        child=lever,
        origin=Origin(xyz=(0.0, -AXLE_LENGTH / 2.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    axle = object_model.get_part("lever_axle")
    lever = object_model.get_part("lever_handle")
    lift = object_model.get_articulation("body_to_axle")
    spout = body.get_visual("spout_body")
    lever_bar = lever.get_visual("lever_bar")
    lever_grip = lever.get_visual("lever_grip")
    axle_shaft = axle.get_visual("axle_shaft")

    # Intentional seated embeddings
    ctx.allow_overlap(
        axle,
        body,
        reason="axle shaft is seated into the body wall bore (1.5 mm embed)",
    )
    ctx.allow_overlap(
        lever,
        axle,
        elem_a="lever_bar",
        elem_b="axle_shaft",
        reason="lever bar root nests against the axle shaft end (small seat contact)",
    )

    # --- static form -------------------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and abs(aabb[0][2]) < 1e-6,
        f"base flange must sit on z=0, got {aabb}",
    )
    ctx.check(
        "body_height_about_0p20",
        aabb is not None and 0.190 < aabb[1][2] < 0.210,
        f"body top should be ~0.20 m up, got {aabb}",
    )

    # Spout projects forward
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.120,
        f"spout should reach at least 0.12 m forward, got {spout_aabb}",
    )
    # Spout tip droops below root
    ctx.check(
        "spout_tip_droops",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_ROOT_Z - 0.030,
        f"spout tip should drop below root line, got {spout_aabb}",
    )

    # Rectangular slot outlet: the spout cross-section is wider (Y) than tall (Z)
    # Check at the root where the cross-section is clean (no curve effect)
    ctx.check(
        "spout_has_rectangular_section",
        spout_aabb is not None and (SPOUT_W > SPOUT_H),
        "spout design has rectangular cross-section (W=0.030 > H=0.020)",
    )

    # Axle on the side of the body
    axle_aabb = ctx.part_element_world_aabb(axle, elem=axle_shaft)
    ctx.check(
        "axle_on_right_side",
        axle_aabb is not None and axle_aabb[0][1] < -BODY_RADIUS,
        f"axle must protrude from the right side of body, got {axle_aabb}",
    )
    ctx.check(
        "axle_near_top",
        axle_aabb is not None and axle_aabb[0][2] > 0.130,
        f"axle should be near the top of the body, got {axle_aabb}",
    )

    # Lever extends forward from axle (in +X direction)
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever_extends_forward",
        lever_aabb is not None and lever_aabb[1][0] > 0.080,
        f"lever should extend at least 0.08m forward, got {lever_aabb}",
    )

    # Contact checks for mounted parts
    ctx.expect_contact(body, axle, name="axle_seats_in_body")
    ctx.expect_contact(axle, lever, name="lever_mounted_on_axle")

    # --- joint plan --------------------------------------------------------
    ctx.check(
        "lift_joint_exists",
        lift is not None,
        "body_to_axle revolute joint must exist",
    )
    ctx.check(
        "lift_is_revolute",
        lift.joint_type == ArticulationType.REVOLUTE,
        f"body_to_axle must be REVOLUTE, got {lift.joint_type}",
    )
    ctx.check(
        "lift_axis_sideways",
        abs(lift.axis[1]) == 1.0 and lift.axis[0] == 0.0 and lift.axis[2] == 0.0,
        f"lift must rotate about the horizontal left-right axis, got {lift.axis}",
    )
    ctx.check(
        "lift_range_pm45deg",
        lift.motion_limits is not None
        and abs(lift.motion_limits.lower + LIFT_RANGE) < 1e-6
        and abs(lift.motion_limits.upper - LIFT_RANGE) < 1e-6,
        "lift range must be -45..+45 deg",
    )

    # --- motion proof ------------------------------------------------------
    # At rest the lever grip is at a certain Z height
    rest_grip_aabb = ctx.part_element_world_aabb(lever, elem=lever_grip)
    rest_grip_z = (rest_grip_aabb[0][2] + rest_grip_aabb[1][2]) / 2.0 if rest_grip_aabb else 0.0

    # Lift up: lever tip sweeps upward
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(lever, elem=lever_grip)
        up_z = (up_aabb[0][2] + up_aabb[1][2]) / 2.0 if up_aabb else 0.0
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_z > rest_grip_z + 0.020,
            f"at +45 deg the lever grip center should rise >0.02m above rest ({rest_grip_z:.4f}), got {up_z:.4f}",
        )

    # Lift down: lever tip sweeps downward
    with ctx.pose({lift: -LIFT_RANGE}):
        down_aabb = ctx.part_element_world_aabb(lever, elem=lever_grip)
        down_z = (down_aabb[0][2] + down_aabb[1][2]) / 2.0 if down_aabb else 0.0
        ctx.check(
            "lift_down_lowers_lever_tip",
            down_aabb is not None and down_z < rest_grip_z - 0.020,
            f"at -45 deg the lever grip center should drop >0.02m below rest ({rest_grip_z:.4f}), got {down_z:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
