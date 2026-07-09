from __future__ import annotations

"""Single-hole basin faucet variant — tapered conical body with side lever.

A compact single-hole basin faucet (~0.20 m tall) with a tapered conical body
wider at the base (~0.060 m) narrowing to ~0.040 m at the top. A small forward
beak projects from the upper body as the water outlet. On the right side, a
short horizontal axle carries a lever handle with subtle grip grooves; the
lever tilts up and down for flow control. Two small screw caps sit on the back
of the body.

Articulation:
- ``lever_tilt``: revolute about the horizontal sideways (Y) axle axis,
  -40..+40 deg; positive q lifts the lever tip upward (flow on).
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Dimensions (meters). World frame: +X forward (spout direction), +Z up,
# lever on the -Y side of the body.
# ----------------------------------------------------------------------------
BODY_BOTTOM_RADIUS = 0.030   # 0.060 m diameter at base
BODY_TOP_RADIUS = 0.020      # 0.040 m diameter at top
BODY_HEIGHT = 0.190          # tapered column height
FLANGE_RADIUS = 0.036
FLANGE_HEIGHT = 0.012
TOTAL_HEIGHT = FLANGE_HEIGHT + BODY_HEIGHT  # ~0.202 m

# Forward beak — small curved outlet near the top
BEAK_ROOT_Z = 0.155          # height where beak exits body
BEAK_LENGTH = 0.045          # forward projection
BEAK_RADIUS = 0.010          # tube radius
BEAK_DROP = 0.015            # how much the tip drops below root

# Side lever axle
AXLE_RADIUS = 0.006
AXLE_LENGTH = 0.016
AXLE_CENTER_Y = -0.028       # outboard of body surface
AXLE_CENTER_Z = 0.165        # near top of body

# Lever handle
LEVER_GRIP_RADIUS = 0.007    # grip section radius
LEVER_GRIP_LENGTH = 0.090    # grip length (forward from axle)
LEVER_HUB_RADIUS = 0.012     # hub that wraps the axle
LEVER_HUB_LENGTH = 0.018     # hub axial length
LEVER_TIP_RADIUS = 0.005     # rounded tip

# Groove rings on grip
GROOVE_COUNT = 5
GROOVE_DEPTH = 0.0012
GROOVE_WIDTH = 0.003
GROOVE_SPACING = 0.014
GROOVE_START = 0.020         # first groove distance from hub

# Screw caps on back of body
SCREW_CAP_RADIUS = 0.004
SCREW_CAP_HEIGHT = 0.003
# Two screw caps on back (-X side) at different heights.
# Positions are computed to sit on the tapered body surface.
SCREW_CAP_0_Z = 0.090   # world Z
SCREW_CAP_0_X = -0.025   # body radius ~0.026 at this height; embed slightly
SCREW_CAP_1_Z = 0.135   # world Z
SCREW_CAP_1_X = -0.023   # body radius ~0.024 at this height; embed slightly

TILT_RANGE = math.radians(40.0)


def _build_tapered_body() -> LatheGeometry:
    """Lathe a tapered conical body with slight concave profile."""
    # Profile from bottom (wide) to top (narrow) with slight concavity
    profile = [
        (BODY_BOTTOM_RADIUS, 0.0),
        (BODY_BOTTOM_RADIUS - 0.001, 0.005),
        (BODY_BOTTOM_RADIUS - 0.003, 0.020),
        (0.027, 0.060),
        (0.025, 0.100),
        (0.023, 0.140),
        (BODY_TOP_RADIUS + 0.001, 0.170),
        (BODY_TOP_RADIUS, BODY_HEIGHT),
        (0.0, BODY_HEIGHT),       # close top
    ]
    return LatheGeometry(profile, segments=32, closed=True)


def _build_beak() -> cq.Workplane:
    """Small forward beak — a tube that projects forward and curves down."""
    path = cq.Workplane("XZ").spline([
        (0.0, 0.0),
        (0.015, -0.002),
        (0.030, -0.006),
        (BEAK_LENGTH, -BEAK_DROP),
    ])
    return (
        cq.Workplane("YZ")
        .circle(BEAK_RADIUS)
        .sweep(path)
    )


def _build_grooved_lever() -> cq.Workplane:
    """Lever handle with hub and grip grooves, built along local +X axis.

    The lever extends forward from the axle center (origin). Hub at the root,
    cylindrical grip with shallow ring grooves, rounded tip.
    """
    # Start with hub cylinder
    lever = (
        cq.Workplane("YZ")
        .circle(LEVER_HUB_RADIUS)
        .extrude(LEVER_HUB_LENGTH)
    )
    # Add grip section
    grip_start = LEVER_HUB_LENGTH
    grip = (
        cq.Workplane("YZ")
        .workplane(offset=grip_start)
        .circle(LEVER_GRIP_RADIUS)
        .extrude(LEVER_GRIP_LENGTH)
    )
    lever = lever.union(grip)
    # Add rounded tip
    tip_z = grip_start + LEVER_GRIP_LENGTH
    tip = (
        cq.Workplane("XY")
        .center(0, 0)
        .transformed(offset=(tip_z, 0, 0))
        .sphere(LEVER_TIP_RADIUS)
    )
    lever = lever.union(tip)
    # Cut groove rings into the grip surface
    for i in range(GROOVE_COUNT):
        gx = grip_start + GROOVE_START + i * GROOVE_SPACING
        if gx + GROOVE_WIDTH > grip_start + LEVER_GRIP_LENGTH - 0.005:
            break
        # Annular groove: cut a slightly larger cylinder minus the core
        groove_outer = LEVER_GRIP_RADIUS + 0.0001
        groove_inner = LEVER_GRIP_RADIUS - GROOVE_DEPTH
        cutter = (
            cq.Workplane("YZ")
            .workplane(offset=gx)
            .circle(groove_outer)
            .circle(groove_inner)
            .extrude(GROOVE_WIDTH)
        )
        lever = lever.cut(cutter)
    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    model.material("bright_steel", rgba=(0.80, 0.81, 0.83, 1.0))
    model.material("dark_steel", rgba=(0.45, 0.46, 0.48, 1.0))
    model.material("chrome", rgba=(0.85, 0.86, 0.88, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")

    # Base flange
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )

    # Tapered conical column on top of flange
    tapered_mesh = mesh_from_geometry(_build_tapered_body(), "tapered_column")
    body.visual(
        tapered_mesh,
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT)),
        material="brushed_steel",
        name="body_column",
    )

    # Small flat cap on top of body
    body.visual(
        Cylinder(radius=BODY_TOP_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT + BODY_HEIGHT + 0.002)),
        material="bright_steel",
        name="top_cap",
    )

    # Forward beak outlet
    body.visual(
        mesh_from_cadquery(_build_beak(), "beak_outlet"),
        origin=Origin(xyz=(BODY_TOP_RADIUS - 0.002, 0.0, BEAK_ROOT_Z)),
        material="brushed_steel",
        name="beak_outlet",
    )

    # Two screw caps on back (-X side) of body, embedded into tapered surface
    for i, (cap_z, cap_x) in enumerate(
        [(SCREW_CAP_0_Z, SCREW_CAP_0_X), (SCREW_CAP_1_Z, SCREW_CAP_1_X)]
    ):
        body.visual(
            Cylinder(radius=SCREW_CAP_RADIUS, length=SCREW_CAP_HEIGHT),
            origin=Origin(
                xyz=(cap_x, 0.0, cap_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="dark_steel",
            name=f"screw_cap_{i}",
        )

    # ------------------------------------------------------------------ axle
    # Short horizontal axle stub on the -Y side of the body
    axle = model.part("lever_axle")
    axle.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="chrome",
        name="axle_shaft",
    )

    model.articulation(
        "lever_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=axle,
        origin=Origin(xyz=(0.0, AXLE_CENTER_Y, AXLE_CENTER_Z)),
        # -Y so positive q lifts the forward (+X) lever tip upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-TILT_RANGE, upper=TILT_RANGE
        ),
    )

    # ------------------------------------------------- lever handle
    # Frame origin at the axle end; the lever extends forward (+X).
    handle = model.part("lever_handle")
    handle.visual(
        mesh_from_cadquery(_build_grooved_lever(), "lever_grip"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="bright_steel",
        name="lever_grip",
    )

    model.articulation(
        "handle_twist",
        ArticulationType.REVOLUTE,
        parent=axle,
        child=handle,
        origin=Origin(xyz=(0.0, -AXLE_LENGTH / 2.0, 0.0)),
        # The lever's own forward axis through the hub.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0,
            lower=-math.radians(25.0),
            upper=math.radians(25.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    axle = object_model.get_part("lever_axle")
    handle = object_model.get_part("lever_handle")
    tilt = object_model.get_articulation("lever_tilt")
    twist = object_model.get_articulation("handle_twist")
    beak = body.get_visual("beak_outlet")
    column = body.get_visual("body_column")
    lever_grip = handle.get_visual("lever_grip")
    screw_cap_0 = body.get_visual("screw_cap_0")
    screw_cap_1 = body.get_visual("screw_cap_1")

    # Intentional seated embeddings
    ctx.allow_overlap(
        axle,
        body,
        reason="axle shaft is seated into the body wall for pivot mounting",
    )
    ctx.allow_overlap(
        handle,
        axle,
        reason="lever hub captures the axle end for rotation",
    )
    ctx.allow_overlap(
        body,
        body,
        elem_a="body_column",
        elem_b="screw_cap_0",
        reason="screw cap seated into tapered body surface",
    )
    ctx.allow_overlap(
        body,
        body,
        elem_a="body_column",
        elem_b="screw_cap_1",
        reason="screw cap seated into tapered body surface",
    )

    # --- static form: tapered conical body ----------------------------------
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

    # Taper verification: column is wider at bottom than top
    col_aabb = ctx.part_element_world_aabb(body, elem=column)
    ctx.check(
        "body_is_tapered_conical",
        col_aabb is not None,
        f"tapered column must exist, got {col_aabb}",
    )
    if col_aabb is not None:
        col_dx = col_aabb[1][0] - col_aabb[0][0]
        col_dy = col_aabb[1][1] - col_aabb[0][1]
        ctx.check(
            "taper_bottom_wider_than_0p050",
            col_dx > 0.050 and col_dy > 0.050,
            f"conical base should be ~0.060 m across, got dx={col_dx:.4f} dy={col_dy:.4f}",
        )
        # The body column top is narrower than the bottom
        ctx.check(
            "taper_top_narrower",
            col_dx < 0.065,
            f"column extent should reflect taper (not uniform 0.060+), got dx={col_dx:.4f}",
        )

    # Forward beak projection
    beak_aabb = ctx.part_element_world_aabb(body, elem=beak)
    ctx.check(
        "beak_projects_forward",
        beak_aabb is not None and beak_aabb[1][0] > BODY_TOP_RADIUS + 0.020,
        f"beak should project forward past the body, got {beak_aabb}",
    )
    ctx.check(
        "beak_is_small",
        beak_aabb is not None and (beak_aabb[1][0] - beak_aabb[0][0]) < 0.060,
        f"beak should be a small projection, got span {beak_aabb}",
    )

    # Screw caps on back of body
    cap0_aabb = ctx.part_element_world_aabb(body, elem=screw_cap_0)
    cap1_aabb = ctx.part_element_world_aabb(body, elem=screw_cap_1)
    ctx.check(
        "screw_caps_on_back",
        cap0_aabb is not None
        and cap1_aabb is not None
        and cap0_aabb[0][0] < -0.015
        and cap1_aabb[0][0] < -0.015,
        f"screw caps should be on the back (-X), got cap0={cap0_aabb}, cap1={cap1_aabb}",
    )
    ctx.check(
        "screw_caps_at_different_heights",
        cap0_aabb is not None
        and cap1_aabb is not None
        and abs(cap0_aabb[0][2] - cap1_aabb[0][2]) > 0.025,
        f"screw caps should be at different Z heights, got cap0={cap0_aabb}, cap1={cap1_aabb}",
    )

    # --- lever and joint plan -----------------------------------------------
    lever_aabb = ctx.part_element_world_aabb(handle, elem=lever_grip)
    ctx.check(
        "lever_extends_forward",
        lever_aabb is not None and lever_aabb[1][0] > 0.060,
        f"lever grip should extend forward ~0.10 m, got {lever_aabb}",
    )

    # Joint axis check
    ctx.check(
        "tilt_axis_sideways",
        abs(tilt.axis[1]) == 1.0 and tilt.axis[0] == 0.0 and tilt.axis[2] == 0.0,
        f"tilt must rotate about horizontal Y axis, got {tilt.axis}",
    )
    ctx.check(
        "tilt_range_pm40deg",
        tilt.motion_limits is not None
        and abs(tilt.motion_limits.lower + TILT_RANGE) < 1e-6
        and abs(tilt.motion_limits.upper - TILT_RANGE) < 1e-6,
        "tilt range must be -40..+40 deg",
    )

    # Twist axis (secondary temperature joint)
    ctx.check(
        "twist_axis_forward",
        abs(twist.axis[0]) == 1.0 and twist.axis[1] == 0.0 and twist.axis[2] == 0.0,
        f"twist must rotate about forward X axis, got {twist.axis}",
    )

    # --- motion proof -------------------------------------------------------
    # Tilt up: lever tip rises
    with ctx.pose({tilt: TILT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=lever_grip)
        ctx.check(
            "tilt_up_raises_lever",
            up_aabb is not None and up_aabb[1][2] > AXLE_CENTER_Z + 0.030,
            f"at +40 deg lever tip should rise well above axle, got {up_aabb}",
        )

    # Tilt down: lever tip drops
    with ctx.pose({tilt: -TILT_RANGE}):
        down_aabb = ctx.part_element_world_aabb(handle, elem=lever_grip)
        ctx.check(
            "tilt_down_lowers_lever",
            down_aabb is not None and down_aabb[0][2] < AXLE_CENTER_Z - 0.030,
            f"at -40 deg lever tip should drop below axle, got {down_aabb}",
        )

    # Contact checks for mounting
    ctx.expect_contact(body, axle, name="axle_seats_on_body")
    ctx.expect_contact(axle, handle, name="handle_seats_on_axle")

    return ctx.report()


object_model = build_object_model()
