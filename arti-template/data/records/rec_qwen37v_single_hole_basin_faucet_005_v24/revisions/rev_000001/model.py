from __future__ import annotations

"""Single-hole basin faucet with waterfall spout and side lever.

A vertical cylindrical body (~0.20 m tall, ~0.055 m diameter) on a slightly
wider round base flange. A channel spout projects forward and slightly down
~0.13 m from the front of the body, terminating in a rounded waterfall-style
lip (~0.048 m wide). On the side near the top, a round flat control disc
(~0.055 m diameter, with red/blue index dots at top/bottom) is carried on a
short horizontal boss; a slim cylindrical lever bar runs forward from the
disc with subtle grip grooves along its surface.

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

SPOUT_ROOT_Z = 0.118  # channel centerline height where it leaves the body
SPOUT_OUTER_W = 0.034
SPOUT_OUTER_H = 0.022
SPOUT_WALL = 0.005
SPOUT_FLOOR = 0.006
WATERFALL_LIP_WIDTH = 0.048  # wider rounded lip at spout tip
WATERFALL_LIP_RADIUS = 0.008  # bullnose radius of the lip edge

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

GRIP_GROOVE_COUNT = 7  # number of grooves along the lever bar
GRIP_GROOVE_WIDTH = 0.0018
GRIP_GROOVE_DEPTH = 0.0008
GRIP_GROOVE_START_X = 0.060  # first groove X position
GRIP_GROOVE_SPACING = 0.012  # spacing between grooves along bar

LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)


def _build_spout_channel() -> cq.Workplane:
    """Open-top U-channel swept forward and curving down, stopping short of tip.

    Built in spout-local coordinates: the channel centerline starts at the
    origin heading +X; the visual is placed at the body front at
    ``SPOUT_ROOT_Z``. The root is buried in the body column so the channel
    reads as one piece with the body. The channel ends just before the tip
    where the waterfall lip attaches.
    """
    path = cq.Workplane("XZ").spline(
        [
            (0.000, 0.000),
            (0.060, -0.004),
            (0.100, -0.012),
            (0.125, -0.024),
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


def _build_waterfall_lip() -> cq.Workplane:
    """Rounded waterfall-style spout lip at the tip.

    A wider, rounded bullnose shape at the end of the spout where water
    cascades over the broad rounded edge. Built in spout-local coordinates
    with the lip centered at the forward end of the channel.
    """
    # The lip is a rounded shape: wider than the channel, with a bullnose
    # cross-section. We build it as a box with filleted edges, then add a
    # rounded top profile.
    lip_w = WATERFALL_LIP_WIDTH
    lip_d = 0.038  # depth (forward extent) of the lip section
    lip_h = SPOUT_OUTER_H + 0.004  # slightly taller to accommodate rounding
    r = WATERFALL_LIP_RADIUS

    # Build as a rounded box using a workplane at the spout tip area
    # The lip sits at the end of the channel path (approximately x=0.125, z=-0.024)
    lip = (
        cq.Workplane("XY")
        .transformed(offset=(0.125 + lip_d / 2.0, 0.0, -0.024))
        .box(lip_d, lip_w, lip_h, centered=(True, True, True))
    )
    # Fillet the front edges to create the waterfall bullnose effect
    lip = lip.edges("|Y").fillet(r)
    # Add the inner trough cutout for water flow
    inner_w = lip_w - 2.0 * SPOUT_WALL
    inner_d = lip_d - SPOUT_WALL
    inner_h = lip_h - SPOUT_FLOOR
    trough = (
        cq.Workplane("XY")
        .transformed(offset=(0.125 + lip_d / 2.0 - 0.002, 0.0, -0.024 + 0.002))
        .box(inner_d, inner_w, inner_h, centered=(True, True, False))
    )
    return lip.cut(trough)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_waterfall")

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
    column_len = BODY_HEIGHT - 0.010
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, 0.010 + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    body.visual(
        mesh_from_cadquery(_build_spout_channel(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )
    body.visual(
        mesh_from_cadquery(_build_waterfall_lip(), "waterfall_lip"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="waterfall_lip",
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
        Cylinder(radius=BAR_RADIUS, length=bar_len),
        origin=Origin(
            xyz=(BAR_X_START + bar_len / 2.0, BAR_OFFSET_Y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bright_steel",
        name="lever_bar",
    )
    # Subtle grip grooves: thin annular ridges on the bar surface.
    groove_r = BAR_RADIUS + GRIP_GROOVE_DEPTH
    for i in range(GRIP_GROOVE_COUNT):
        gx = GRIP_GROOVE_START_X + i * GRIP_GROOVE_SPACING
        handle.visual(
            Cylinder(radius=groove_r, length=GRIP_GROOVE_WIDTH),
            origin=Origin(
                xyz=(gx, BAR_OFFSET_Y, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="bright_steel",
            name=f"grip_groove_{i}",
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
    waterfall = body.get_visual("waterfall_lip")
    bar = handle.get_visual("lever_bar")
    disc = handle.get_visual("control_disc")
    hot_dot = handle.get_visual("hot_dot")

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

    # --- waterfall spout lip -----------------------------------------------
    lip_aabb = ctx.part_element_world_aabb(body, elem=waterfall)
    ctx.check(
        "waterfall_lip_exists",
        lip_aabb is not None,
        "waterfall lip visual must exist on the body",
    )
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.100,
        f"channel spout should project forward, got {spout_aabb}",
    )
    # Waterfall lip must be wider than the channel body
    if lip_aabb is not None and spout_aabb is not None:
        lip_width_y = lip_aabb[1][1] - lip_aabb[0][1]
        channel_width_y = spout_aabb[1][1] - spout_aabb[0][1]
        ctx.check(
            "waterfall_lip_wider_than_channel",
            lip_width_y > channel_width_y + 0.005,
            f"waterfall lip ({lip_width_y:.4f}m) must be wider than channel ({channel_width_y:.4f}m)",
        )
    # Lip should reach forward of the channel
    if lip_aabb is not None and spout_aabb is not None:
        ctx.check(
            "waterfall_lip_at_spout_tip",
            lip_aabb[1][0] > spout_aabb[1][0] - 0.010,
            f"waterfall lip should be at or beyond channel tip: lip_x_max={lip_aabb[1][0]:.4f}, channel_x_max={spout_aabb[1][0]:.4f}",
        )
    # Overall spout (channel + lip) reaches forward
    ctx.check(
        "spout_assembly_reaches_forward",
        lip_aabb is not None and lip_aabb[1][0] > 0.130,
        f"spout with waterfall lip should reach ~0.14+ m forward, got {lip_aabb}",
    )

    # --- grip grooves on lever bar ----------------------------------------
    groove_names = [f"grip_groove_{i}" for i in range(GRIP_GROOVE_COUNT)]
    groove_aabbs = []
    for gn in groove_names:
        gv = handle.get_visual(gn)
        ga = ctx.part_element_world_aabb(handle, elem=gv)
        groove_aabbs.append(ga)
    ctx.check(
        "grip_grooves_exist",
        all(ga is not None for ga in groove_aabbs),
        f"all {GRIP_GROOVE_COUNT} grip grooves must exist on the lever bar",
    )
    # Grooves should be slightly proud of the bar surface (larger radius)
    bar_aabb = ctx.part_element_world_aabb(handle, elem=bar)
    if groove_aabbs[0] is not None and bar_aabb is not None:
        bar_height_z = bar_aabb[1][2] - bar_aabb[0][2]
        groove_height_z = groove_aabbs[0][1][2] - groove_aabbs[0][0][2]
        ctx.check(
            "grip_grooves_proud_of_bar",
            groove_height_z >= bar_height_z - 1e-6,
            f"groove Z extent ({groove_height_z:.5f}m) should be >= bar Z extent ({bar_height_z:.5f}m)",
        )
    # Grooves should be distributed along the bar (X axis)
    if all(ga is not None for ga in groove_aabbs):
        first_gx = groove_aabbs[0][0][0]
        last_gx = groove_aabbs[-1][1][0]
        ctx.check(
            "grip_grooves_distributed_along_bar",
            last_gx - first_gx > 0.040,
            f"grooves should span >0.040m along bar, got span {last_gx - first_gx:.4f}m",
        )

    # --- disc and bar geometry -------------------------------------------
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

    ctx.check(
        "lever_bar_length",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.130,
        f"lever bar should run ~0.136 m forward, got {bar_aabb}",
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
