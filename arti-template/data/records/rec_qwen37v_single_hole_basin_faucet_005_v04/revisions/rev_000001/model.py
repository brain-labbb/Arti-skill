from __future__ import annotations

"""Single-hole basin faucet with waterfall spout and top-mounted lever.

A vertical cylindrical body (~0.20 m tall, ~0.055 m diameter) on a slightly
wider round base flange. A wide, flat waterfall-style spout projects forward
from the upper body with a rounded lip at the tip. On the top of the body, a
slim cylindrical lever bar extends forward, mounted on a short pivot stem.
Two small screw caps are visible on the back of the body column.

Articulation chain (body -> pivot -> lever):
- ``lever_lift``: revolute about the horizontal sideways (Y) axis at the top
  of the body, -40..+40 deg; positive q lifts the lever tip up (flow control).
- ``lever_swing``: revolute about the vertical (Z) axis through the pivot
  stem, -30..+30 deg (side-to-side temperature mix).
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Dimensions (meters). World frame: +X forward (spout direction), +Z up.
# ----------------------------------------------------------------------------
BODY_RADIUS = 0.0275  # 0.055 m diameter column
BODY_HEIGHT = 0.200
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

# Waterfall spout
SPOUT_ROOT_Z = 0.140  # height where spout exits body
SPOUT_WIDTH = 0.048   # wide flat spout
SPOUT_HEIGHT = 0.012  # thin profile
SPOUT_WALL = 0.003
SPOUT_LENGTH = 0.120  # forward projection

# Top lever pivot
PIVOT_RADIUS = 0.012
PIVOT_HEIGHT = 0.018
PIVOT_Z = BODY_HEIGHT  # on top of body

# Lever bar
BAR_RADIUS = 0.006
BAR_LENGTH = 0.140
BAR_Z = PIVOT_Z + PIVOT_HEIGHT  # lever rides on top of pivot

# Grip grooves
GROOVE_COUNT = 8
GROOVE_WIDTH = 0.003
GROOVE_DEPTH = 0.0015
GRIP_START = 0.050  # where grooves begin along the bar
GRIP_END = 0.130    # where grooves end

# Screw caps on back of body
SCREW_CAP_RADIUS = 0.004
SCREW_CAP_HEIGHT = 0.003
SCREW_CAP_Z_LOWER = 0.090
SCREW_CAP_Z_UPPER = 0.120

LIFT_RANGE = math.radians(40.0)
SWING_RANGE = math.radians(30.0)


def _build_waterfall_spout() -> cq.Workplane:
    """Wide flat spout with rounded waterfall lip at the tip.

    Built in spout-local coordinates: spout centerline starts at origin
    heading +X; the visual is placed at the body front at SPOUT_ROOT_Z.
    """
    # Sweep path: gentle downward curve
    path = cq.Workplane("XZ").spline(
        [
            (0.000, 0.000),
            (0.040, -0.002),
            (0.080, -0.006),
            (0.110, -0.012),
            (0.125, -0.020),
        ]
    )
    # Cross-section: wide flat rectangle with rounded corners
    hw = SPOUT_WIDTH / 2.0
    hh = SPOUT_HEIGHT / 2.0
    inner_hw = hw - SPOUT_WALL
    floor_v = -hh + SPOUT_WALL
    # U-channel profile (open top)
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
    spout = profile.sweep(path)

    # Add rounded waterfall lip at the tip: a half-round torus segment
    # at the front edge curving downward
    lip_radius = SPOUT_HEIGHT / 2.0
    lip = (
        cq.Workplane("XY")
        .transformed(offset=(0.125, 0.0, -0.020))
        .transformed(rotate=(0, 0, 0))
        .rect(SPOUT_WIDTH - 2 * SPOUT_WALL, lip_radius * 2)
        .extrude(lip_radius)
    )
    # Fillet the leading edge to create the waterfall lip
    spout = spout.union(
        cq.Workplane("XZ")
        .transformed(offset=(0.0, 0.0, 0.0))
        .center(0.125, -0.020)
        .rect(SPOUT_WIDTH - 2 * SPOUT_WALL, lip_radius * 2.5)
        .extrude(0.001)
    )

    return spout


def _build_grooved_lever() -> cq.Workplane:
    """Cylindrical lever bar with subtle grip grooves along the handle section.

    Built in lever-local coords: bar along +X from origin, grooves are
    circumferential cuts.
    """
    # Main bar cylinder
    bar = (
        cq.Workplane("YZ")
        .circle(BAR_RADIUS)
        .extrude(BAR_LENGTH)
    )

    # Cut circumferential grooves along the grip section
    for i in range(GROOVE_COUNT):
        t = GRIP_START + i * (GRIP_END - GRIP_START) / (GROOVE_COUNT - 1)
        groove = (
            cq.Workplane("YZ")
            .transformed(offset=(t, 0.0, 0.0))
            .circle(BAR_RADIUS + 0.001)
            .circle(BAR_RADIUS - GROOVE_DEPTH)
            .extrude(GROOVE_WIDTH)
        )
        bar = bar.cut(groove)

    return bar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_steel", rgba=(0.55, 0.56, 0.58, 1.0))

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
    # Waterfall spout
    body.visual(
        mesh_from_cadquery(_build_waterfall_spout(), "waterfall_spout"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="waterfall_spout",
    )
    # Two screw caps on back of body (-X side)
    body.visual(
        Cylinder(radius=SCREW_CAP_RADIUS, length=SCREW_CAP_HEIGHT),
        origin=Origin(
            xyz=(-(BODY_RADIUS + SCREW_CAP_HEIGHT / 2.0 - 0.001), 0.0, SCREW_CAP_Z_LOWER),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="dark_steel",
        name="screw_cap_lower",
    )
    body.visual(
        Cylinder(radius=SCREW_CAP_RADIUS, length=SCREW_CAP_HEIGHT),
        origin=Origin(
            xyz=(-(BODY_RADIUS + SCREW_CAP_HEIGHT / 2.0 - 0.001), 0.0, SCREW_CAP_Z_UPPER),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="dark_steel",
        name="screw_cap_upper",
    )

    # ---------------------------------------------------------- pivot stem
    # Short cylindrical stem on top of body; carries the lever assembly.
    pivot = model.part("lever_pivot")
    pivot.visual(
        Cylinder(radius=PIVOT_RADIUS, length=PIVOT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT / 2.0)),
        material="bright_steel",
        name="pivot_stem",
    )

    model.articulation(
        "lever_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pivot,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        # Z axis for side-to-side swing
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.0, lower=-SWING_RANGE, upper=SWING_RANGE
        ),
    )

    # --------------------------------------------------------- lever handle
    # Frame origin at the pivot top; the lift axis is sideways (Y).
    handle = model.part("lever_handle")
    handle.visual(
        mesh_from_cadquery(_build_grooved_lever(), "lever_bar"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="bright_steel",
        name="lever_bar",
    )
    # Small hub/collar where lever meets pivot (vertical cylinder)
    handle.visual(
        Cylinder(radius=PIVOT_RADIUS + 0.002, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material="bright_steel",
        name="lever_hub",
    )

    model.articulation(
        "lever_lift",
        ArticulationType.REVOLUTE,
        parent=pivot,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        # -Y so positive q lifts the forward (+X) lever tip upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-LIFT_RANGE, upper=LIFT_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    pivot = object_model.get_part("lever_pivot")
    handle = object_model.get_part("lever_handle")
    swing = object_model.get_articulation("lever_swing")
    lift = object_model.get_articulation("lever_lift")
    spout = body.get_visual("waterfall_spout")
    bar = handle.get_visual("lever_bar")
    hub = handle.get_visual("lever_hub")
    cap_lower = body.get_visual("screw_cap_lower")
    cap_upper = body.get_visual("screw_cap_upper")

    # Intentional seated embeddings
    ctx.allow_overlap(
        pivot,
        body,
        reason="pivot stem seats 1 mm into the body top surface",
    )
    ctx.allow_overlap(
        handle,
        pivot,
        reason="lever hub captures the pivot stem top (0.5 mm seat)",
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
        aabb is not None and 0.195 < aabb[1][2] < 0.210,
        f"body top should be ~0.20 m up, got {aabb}",
    )

    # Waterfall spout checks
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.100,
        f"waterfall spout should project forward ~0.12 m, got {spout_aabb}",
    )
    ctx.check(
        "spout_is_wide",
        spout_aabb is not None and (spout_aabb[1][1] - spout_aabb[0][1]) > 0.035,
        f"waterfall spout should be wider than 0.035 m, got {spout_aabb}",
    )
    ctx.check(
        "spout_tip_droops",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_ROOT_Z - 0.010,
        f"waterfall lip should droop below root, got {spout_aabb}",
    )

    # Screw caps on back of body
    cap_lower_aabb = ctx.part_element_world_aabb(body, elem=cap_lower)
    cap_upper_aabb = ctx.part_element_world_aabb(body, elem=cap_upper)
    ctx.check(
        "screw_caps_on_back",
        cap_lower_aabb is not None
        and cap_upper_aabb is not None
        and cap_lower_aabb[0][0] < -BODY_RADIUS + 0.002
        and cap_upper_aabb[0][0] < -BODY_RADIUS + 0.002,
        f"screw caps should be on the -X back of the body: lower={cap_lower_aabb}, upper={cap_upper_aabb}",
    )
    ctx.check(
        "screw_caps_vertically_separated",
        cap_lower_aabb is not None
        and cap_upper_aabb is not None
        and cap_upper_aabb[1][2] - cap_lower_aabb[0][2] > 0.020,
        "two screw caps should be vertically separated by >20 mm",
    )

    # Lever on top of body
    hub_aabb = ctx.part_element_world_aabb(handle, elem=hub)
    ctx.check(
        "lever_on_top_of_body",
        hub_aabb is not None and hub_aabb[0][2] > BODY_HEIGHT - 0.010,
        f"lever hub should be on top of body (~0.20 m), got {hub_aabb}",
    )

    bar_aabb = ctx.part_element_world_aabb(handle, elem=bar)
    ctx.check(
        "lever_bar_length",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.120,
        f"lever bar should be ~0.14 m long, got {bar_aabb}",
    )

    # Contact checks
    ctx.expect_contact(body, pivot, name="pivot_seats_on_body")
    ctx.expect_contact(pivot, handle, name="lever_seats_on_pivot")

    # --- joint plan --------------------------------------------------------
    ctx.check(
        "swing_axis_vertical",
        abs(swing.axis[2]) == 1.0 and swing.axis[0] == 0.0 and swing.axis[1] == 0.0,
        f"swing must rotate about vertical Z axis, got {swing.axis}",
    )
    ctx.check(
        "swing_range_pm30deg",
        swing.motion_limits is not None
        and abs(swing.motion_limits.lower + SWING_RANGE) < 1e-6
        and abs(swing.motion_limits.upper - SWING_RANGE) < 1e-6,
        "swing range must be -30..+30 deg",
    )
    ctx.check(
        "lift_axis_sideways",
        abs(lift.axis[1]) == 1.0 and lift.axis[0] == 0.0 and lift.axis[2] == 0.0,
        f"lift must rotate about horizontal Y axis, got {lift.axis}",
    )
    ctx.check(
        "lift_range_pm40deg",
        lift.motion_limits is not None
        and abs(lift.motion_limits.lower + LIFT_RANGE) < 1e-6
        and abs(lift.motion_limits.upper - LIFT_RANGE) < 1e-6,
        "lift range must be -40..+40 deg",
    )

    # --- motion proof ------------------------------------------------------
    # Lift up: forward lever tip sweeps upward
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > PIVOT_Z + 0.040,
            f"at +40 deg the bar tip should rise well above pivot, got {up_aabb}",
        )

    # Lift down: tip sweeps downward
    with ctx.pose({lift: -LIFT_RANGE}):
        down_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_down_lowers_lever_tip",
            down_aabb is not None and down_aabb[0][2] < PIVOT_Z - 0.020,
            f"at -40 deg the bar tip should drop below pivot, got {down_aabb}",
        )

    # Swing: lever swings side-to-side
    rest_bar = ctx.part_element_world_aabb(handle, elem=bar)
    with ctx.pose({swing: SWING_RANGE}):
        swung_bar = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "swing_moves_lever_sideways",
            rest_bar is not None
            and swung_bar is not None
            and abs(swung_bar[1][1] - rest_bar[1][1]) > 0.020,
            f"at +30 deg swing the bar should move sideways: rest={rest_bar}, swung={swung_bar}",
        )

    return ctx.report()


object_model = build_object_model()
