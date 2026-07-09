from __future__ import annotations

"""Single-hole basin faucet variant with raised base collar and grooved lever grip.

A vertical cylindrical body (~0.20 m tall, ~0.055 m diameter) on a slightly
wider round base flange, with a raised circular collar (torus) around the base.
A flat-bottomed open-channel spout projects forward and slightly down ~0.13 m
from the front of the body, with a curved trough-like tip. On the side near
the top, a round flat control disc (~0.055 m diameter, with red/blue index
dots at top/bottom) is carried on a short horizontal boss; a slim grooved
cylindrical lever bar runs forward from the disc, parallel under the body top.

Articulation chain (body -> boss -> disc/lever):
- ``boss_lift``: revolute about the horizontal sideways (Y) boss axis,
  -40..+40 deg; positive q lifts the lever tip up (flow control).
- ``lever_twist``: revolute about the lever's own forward (X) axis through
  the disc center, -30..+30 deg (temperature mix).
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
# Dimensions (meters). World frame: +X forward (spout direction), +Z up,
# control disc on the -Y side of the body.
# ----------------------------------------------------------------------------
BODY_RADIUS = 0.0275  # 0.055 m diameter column
BODY_HEIGHT = 0.200
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

# Raised circular collar around the base
COLLAR_MAJOR_R = 0.031  # center of torus tube from axis
COLLAR_MINOR_R = 0.006  # tube radius
COLLAR_Z = FLANGE_HEIGHT + COLLAR_MINOR_R  # sits on top of flange

SPOUT_ROOT_Z = 0.118  # channel centerline height where it leaves the body
SPOUT_OUTER_W = 0.034
SPOUT_OUTER_H = 0.022
SPOUT_WALL = 0.005
SPOUT_FLOOR = 0.006

DISC_RADIUS = 0.0275  # 0.055 m diameter control disc
DISC_THICKNESS = 0.012
DISC_CENTER_Y = -0.048  # disc mid-plane (outboard of the body surface)
DISC_CENTER_Z = 0.163

BOSS_RADIUS = 0.011
BOSS_LENGTH = 0.018
BOSS_CENTER_Y = -0.0335  # lift joint origin on the boss axis

BAR_RADIUS = 0.005
BAR_X_START = 0.014  # clears the boss (boss radius 0.011)
BAR_X_END = 0.150
BAR_OFFSET_Y = 0.0095  # inboard of the disc mid-plane, toward the body

DOT_RADIUS = 0.0035
DOT_LENGTH = 0.0018
DOT_Z = 0.019  # radial offset of the paint dots on the disc face

LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)

# Groove parameters for lever grip
GROOVE_DEPTH = 0.0008
GROOVE_WIDTH = 0.0015
N_GROOVES = 12


def _build_spout() -> cq.Workplane:
    """Open-top U-channel swept forward and curving down at the tip.

    Built in spout-local coordinates: the channel centerline starts at the
    origin heading +X; the visual is placed at the body front at
    ``SPOUT_ROOT_Z``. The root is buried in the body column so the channel
    reads as one piece with the body.
    """
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


def _build_collar() -> cq.Workplane:
    """Raised circular collar (torus) around the base.

    Built in world-aligned coordinates with the torus centered on the Z axis.
    The visual origin translates it to the correct height.
    """
    # XZ workplane: local X = world X (radial), local Y = world Z (vertical).
    # Draw circle at radial offset COLLAR_MAJOR_R, revolve around world Z axis.
    return (
        cq.Workplane("XZ")
        .moveTo(COLLAR_MAJOR_R, 0.0)
        .circle(COLLAR_MINOR_R)
        .revolve(360, (0.0, 0.0), (0.0, 1.0))
    )


def _build_grooved_bar() -> cq.Workplane:
    """Cylindrical lever bar with circumferential grooves on the grip section.

    Built along local Z (CadQuery default cylinder axis); the visual origin
    applies rpy=(0, pi/2, 0) to align it with the lever's forward X axis.
    """
    bar_len = BAR_X_END - BAR_X_START
    half = bar_len / 2.0
    R = BAR_RADIUS
    d = GROOVE_DEPTH
    w = GROOVE_WIDTH
    n = N_GROOVES

    # Profile points (r, z) on the XZ workplane: r = radial, z = along bar axis
    pts = [
        (0.0, -half),
        (R, -half),
    ]

    # Grip area: outer ~65% of bar (away from disc)
    grip_lo = -half + 0.030
    grip_hi = half - 0.006

    pts.append((R, grip_lo))

    # Add groove notches
    span = grip_hi - grip_lo
    step = span / (n + 1)
    for i in range(n):
        zc = grip_lo + (i + 1) * step
        pts.append((R, zc - w / 2.0))
        pts.append((R - d, zc - w / 2.0))
        pts.append((R - d, zc + w / 2.0))
        pts.append((R, zc + w / 2.0))

    pts.append((R, grip_hi))
    pts.append((R, half))
    pts.append((0.0, half))

    # Revolve around the Z axis (local Y on XZ workplane)
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360, (0.0, 0.0), (0.0, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )
    # Raised circular collar around the base
    body.visual(
        mesh_from_cadquery(_build_collar(), "base_collar"),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        material="brushed_steel",
        name="base_collar",
    )
    column_len = BODY_HEIGHT - 0.010
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, 0.010 + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    body.visual(
        mesh_from_cadquery(_build_spout(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )

    # ------------------------------------------------------------------ boss
    # Short horizontal boss on the side of the body that carries the
    # disc-and-lever assembly; it rotates about its own sideways axis for the
    # lift (flow) motion.
    boss = model.part("lever_boss")
    boss.visual(
        Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="boss_shaft",
    )

    model.articulation(
        "boss_lift",
        ArticulationType.REVOLUTE,
        parent=body,
        child=boss,
        origin=Origin(xyz=(0.0, BOSS_CENTER_Y, DISC_CENTER_Z)),
        # -Y so positive q lifts the forward (+X) lever tip upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-LIFT_RANGE, upper=LIFT_RANGE
        ),
    )

    # ------------------------------------------------- disc + lever assembly
    # Frame origin at the disc center; the twist axis is the lever's own
    # forward (X) axis through this point.
    handle = model.part("lever_handle")
    handle.visual(
        Cylinder(radius=DISC_RADIUS, length=DISC_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="control_disc",
    )
    dot_face_y = -DISC_THICKNESS / 2.0 - DOT_LENGTH / 2.0 + 0.0003
    handle.visual(
        Cylinder(radius=DOT_RADIUS, length=DOT_LENGTH),
        origin=Origin(xyz=(0.0, dot_face_y, DOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hot_red",
        name="hot_dot",
    )
    handle.visual(
        Cylinder(radius=DOT_RADIUS, length=DOT_LENGTH),
        origin=Origin(xyz=(0.0, dot_face_y, -DOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="cold_blue",
        name="cold_dot",
    )
    bar_len = BAR_X_END - BAR_X_START
    handle.visual(
        mesh_from_cadquery(_build_grooved_bar(), "grooved_lever_bar"),
        origin=Origin(
            xyz=(BAR_X_START + bar_len / 2.0, BAR_OFFSET_Y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bright_steel",
        name="grooved_lever_bar",
    )

    model.articulation(
        "lever_twist",
        ArticulationType.REVOLUTE,
        parent=boss,
        child=handle,
        origin=Origin(xyz=(0.0, DISC_CENTER_Y - BOSS_CENTER_Y, 0.0)),
        # The lever's own forward axis through the disc center.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.0, lower=-TWIST_RANGE, upper=TWIST_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    boss = object_model.get_part("lever_boss")
    handle = object_model.get_part("lever_handle")
    lift = object_model.get_articulation("boss_lift")
    twist = object_model.get_articulation("lever_twist")
    spout = body.get_visual("spout_channel")
    bar = handle.get_visual("grooved_lever_bar")
    disc = handle.get_visual("control_disc")
    hot_dot = handle.get_visual("hot_dot")
    collar = body.get_visual("base_collar")

    # Intentional seated embeddings of the rotating stack.
    ctx.allow_overlap(
        boss,
        body,
        reason="boss shaft is seated 1.5 mm into the curved body wall",
    )
    ctx.allow_overlap(
        handle,
        boss,
        reason="disc hub captures the boss shaft end (0.5 mm seat)",
    )

    # --- variant geometry checks -------------------------------------------

    # Collar: raised ring around the base, sits above flange
    collar_aabb = ctx.part_element_world_aabb(body, elem=collar)
    ctx.check(
        "collar_exists",
        collar_aabb is not None,
        "base collar visual must exist",
    )
    ctx.check(
        "collar_above_flange",
        collar_aabb is not None and collar_aabb[1][2] > FLANGE_HEIGHT + 0.008,
        f"collar should extend above flange (>{FLANGE_HEIGHT + 0.008:.3f} m), got {collar_aabb}",
    )
    ctx.check(
        "collar_wider_than_body",
        collar_aabb is not None
        and (collar_aabb[1][0] - collar_aabb[0][0]) > 2.0 * BODY_RADIUS,
        f"collar should be wider than body column ({2*BODY_RADIUS:.3f} m), got {collar_aabb}",
    )

    # Grooved lever bar: verify it exists and has correct length
    bar_aabb = ctx.part_element_world_aabb(handle, elem=bar)
    ctx.check(
        "grooved_lever_bar_exists",
        bar_aabb is not None,
        "grooved lever bar visual must exist",
    )
    ctx.check(
        "lever_bar_length",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.130,
        f"lever bar should run ~0.136 m forward, got {bar_aabb}",
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
        aabb is not None and 0.195 < aabb[1][2] < 0.205,
        f"flat body top should be ~0.20 m up, got {aabb}",
    )
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.145,
        f"channel spout should reach ~0.155 m forward, got {spout_aabb}",
    )
    ctx.check(
        "spout_tip_droops",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_ROOT_Z - 0.035,
        f"curved tip should drop below the root line, got {spout_aabb}",
    )

    disc_aabb = ctx.part_element_world_aabb(handle, elem=disc)
    ctx.check(
        "disc_outboard_of_body",
        disc_aabb is not None and disc_aabb[1][1] < -BODY_RADIUS - 0.010,
        f"control disc must sit clear of the body side, got {disc_aabb}",
    )
    ctx.check(
        "disc_diameter_0p055",
        disc_aabb is not None
        and abs((disc_aabb[1][2] - disc_aabb[0][2]) - 2.0 * DISC_RADIUS) < 1e-4,
        f"disc should be 0.055 m across, got {disc_aabb}",
    )

    # The lever bar rides above the spout channel at rest.
    ctx.expect_gap(
        handle,
        body,
        axis="z",
        min_gap=0.020,
        positive_elem=bar,
        negative_elem=spout,
        name="lever_bar_above_spout",
    )
    # Boss shaft actually bridges body and disc hub.
    ctx.expect_contact(body, boss, name="boss_seats_on_body")
    ctx.expect_contact(boss, handle, name="disc_seats_on_boss")

    # --- joint plan --------------------------------------------------------
    ctx.check(
        "lift_axis_sideways",
        abs(lift.axis[1]) == 1.0 and lift.axis[0] == 0.0 and lift.axis[2] == 0.0,
        f"lift must rotate about the horizontal left-right axis, got {lift.axis}",
    )
    ctx.check(
        "lift_range_pm40deg",
        lift.motion_limits is not None
        and abs(lift.motion_limits.lower + LIFT_RANGE) < 1e-6
        and abs(lift.motion_limits.upper - LIFT_RANGE) < 1e-6,
        "lift range must be -40..+40 deg",
    )
    ctx.check(
        "twist_axis_forward",
        abs(twist.axis[0]) == 1.0 and twist.axis[1] == 0.0 and twist.axis[2] == 0.0,
        f"twist must rotate about the lever's forward axis, got {twist.axis}",
    )
    ctx.check(
        "twist_range_pm30deg",
        twist.motion_limits is not None
        and abs(twist.motion_limits.lower + TWIST_RANGE) < 1e-6
        and abs(twist.motion_limits.upper - TWIST_RANGE) < 1e-6,
        "twist range must be -30..+30 deg",
    )

    # --- motion proof ------------------------------------------------------
    # Lift up: the forward lever tip sweeps upward, well above the disc top.
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > 0.245,
            f"at +40 deg the bar tip should rise to ~0.26 m, got {up_aabb}",
        )
    # Lift down: tip sweeps below the spout root line, still beside the spout.
    with ctx.pose({lift: -LIFT_RANGE}):
        down_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_down_lowers_lever_tip",
            down_aabb is not None and down_aabb[0][2] < 0.080,
            f"at -40 deg the bar tip should drop to ~0.066 m, got {down_aabb}",
        )
        ctx.expect_gap(
            body,
            handle,
            axis="y",
            min_gap=0.010,
            positive_elem=spout,
            negative_elem=bar,
            name="lowered_bar_passes_beside_spout",
        )

    # Twist: the off-axis paint dot orbits the lever's forward axis, so its
    # sideways (Y) position must move; this proves a continuous rotation of
    # the whole disc-and-lever assembly about X.
    rest_dot = ctx.part_element_world_aabb(handle, elem=hot_dot)
    with ctx.pose({twist: TWIST_RANGE}):
        twist_dot = ctx.part_element_world_aabb(handle, elem=hot_dot)
        ctx.check(
            "twist_swings_index_dot",
            rest_dot is not None
            and twist_dot is not None
            and (rest_dot[0][1] - twist_dot[0][1]) > 0.005,
            f"hot dot should swing outboard at +30 deg: rest={rest_dot}, twisted={twist_dot}",
        )
        # Disc rim tilts toward the body but must not touch it.
        ctx.expect_gap(
            body,
            handle,
            axis="y",
            max_penetration=0.0,
            elem_a=body.get_visual("body_column"),
            elem_b=disc,
            name="twisted_disc_clears_body",
        )

    return ctx.report()


object_model = build_object_model()
