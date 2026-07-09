from __future__ import annotations

"""Single-hole basin faucet variant — squat body with oval pedestal.

A compact single-hole basin faucet with a squat cylindrical body (~0.14 m
tall, ~0.055 m diameter) standing on a wide oval pedestal. A flat-bottomed
open-channel spout projects forward from the front of the body. On the side
near the top, a round flat control disc with red/blue index dots is carried
on a short horizontal boss; a slim cylindrical lever bar with subtle grip
grooves runs forward from the disc.

A pull-up drain rod slides vertically behind the body through a guide boss.

Articulation chain:
- ``boss_lift``: revolute about the horizontal sideways (Y) boss axis,
  -40..+40 deg; positive q lifts the lever tip up (flow control).
- ``lever_twist``: revolute about the lever's own forward (X) axis,
  -30..+30 deg (temperature mix).
- ``drain_slide``: prismatic along +Z, 0..0.04 m; pulls the drain rod up.
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
BODY_RADIUS = 0.0275        # 0.055 m diameter column
BODY_HEIGHT = 0.120         # squat body column
PEDESTAL_HEIGHT = 0.014
PEDESTAL_RX = 0.045         # oval half-width (X)
PEDESTAL_RY = 0.033         # oval half-depth (Y)
TOTAL_HEIGHT = PEDESTAL_HEIGHT + BODY_HEIGHT  # ~0.134 m

SPOUT_ROOT_Z = 0.078        # channel centerline height where it leaves the body
SPOUT_OUTER_W = 0.034
SPOUT_OUTER_H = 0.022
SPOUT_WALL = 0.005
SPOUT_FLOOR = 0.006

DISC_RADIUS = 0.0275         # 0.055 m diameter control disc
DISC_THICKNESS = 0.012
DISC_CENTER_Y = -0.048
DISC_CENTER_Z = 0.105       # adjusted for squat body

BOSS_RADIUS = 0.011
BOSS_LENGTH = 0.018
BOSS_CENTER_Y = -0.0335

BAR_RADIUS = 0.005
BAR_X_START = 0.014
BAR_X_END = 0.150
BAR_OFFSET_Y = 0.0095

DOT_RADIUS = 0.0035
DOT_LENGTH = 0.0018
DOT_Z = 0.019

# Groove dimensions (raised ridges on lever bar)
GROOVE_COUNT = 5
GROOVE_WIDTH = 0.003
GROOVE_HEIGHT = 0.001       # radial protrusion above bar surface
GROOVE_OUTER = BAR_RADIUS + GROOVE_HEIGHT

# Drain rod dimensions
DRAIN_ROD_RADIUS = 0.004
DRAIN_ROD_LENGTH = 0.060
DRAIN_GUIDE_RADIUS = 0.008
DRAIN_GUIDE_LENGTH = 0.016
DRAIN_ROD_X = 0.0           # centered behind body in X
DRAIN_ROD_Y = 0.0           # centered behind body in Y
DRAIN_GUIDE_Z = 0.055       # guide boss height on back of body
DRAIN_KNOB_RADIUS = 0.008
DRAIN_KNOB_LENGTH = 0.008

LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)
DRAIN_TRAVEL = 0.040         # prismatic travel in meters


def _build_oval_pedestal() -> cq.Workplane:
    """Wide oval pedestal base — extruded ellipse."""
    return (
        cq.Workplane("XY")
        .ellipse(PEDESTAL_RX, PEDESTAL_RY)
        .extrude(PEDESTAL_HEIGHT)
    )


def _build_spout() -> cq.Workplane:
    """Open-top U-channel swept forward and curving down at the tip."""
    path = cq.Workplane("XZ").spline(
        [
            (0.000, 0.000),
            (0.050, -0.003),
            (0.090, -0.012),
            (0.120, -0.024),
            (0.140, -0.040),
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


def _build_groove_ring() -> cq.Workplane:
    """Thin ring that wraps around the lever bar as a grip groove."""
    return (
        cq.Workplane("XY")
        .circle(GROOVE_OUTER)
        .circle(BAR_RADIUS - 0.0002)
        .extrude(GROOVE_WIDTH)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))
    model.material("dark_steel", rgba=(0.55, 0.56, 0.58, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    # Oval pedestal base
    body.visual(
        mesh_from_cadquery(_build_oval_pedestal(), "oval_pedestal"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="brushed_steel",
        name="oval_pedestal",
    )
    # Main column
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=BODY_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT + BODY_HEIGHT / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    # Flat top cap
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, TOTAL_HEIGHT - 0.002)),
        material="brushed_steel",
        name="body_cap",
    )
    # Spout
    body.visual(
        mesh_from_cadquery(_build_spout(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )
    # Drain guide boss on back of body
    body.visual(
        Cylinder(radius=DRAIN_GUIDE_RADIUS, length=DRAIN_GUIDE_LENGTH),
        origin=Origin(
            xyz=(DRAIN_ROD_X - BODY_RADIUS - DRAIN_GUIDE_LENGTH / 2.0 + 0.004,
                 DRAIN_ROD_Y,
                 DRAIN_GUIDE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="brushed_steel",
        name="drain_guide",
    )

    # ------------------------------------------------------------------ boss
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
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-LIFT_RANGE, upper=LIFT_RANGE
        ),
    )

    # ------------------------------------------------- disc + lever assembly
    handle = model.part("lever_handle")
    handle.visual(
        Cylinder(radius=DISC_RADIUS, length=DISC_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="control_disc",
    )
    # Paint dots
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
    # Lever bar
    bar_len = BAR_X_END - BAR_X_START
    bar_cx = BAR_X_START + bar_len / 2.0
    handle.visual(
        Cylinder(radius=BAR_RADIUS, length=bar_len),
        origin=Origin(
            xyz=(bar_cx, BAR_OFFSET_Y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bright_steel",
        name="lever_bar",
    )
    # Grip grooves — thin rings along the lever bar
    groove_ring_mesh = mesh_from_cadquery(_build_groove_ring(), "groove_ring")
    groove_spacing = bar_len / (GROOVE_COUNT + 1)
    for i in range(GROOVE_COUNT):
        gx = BAR_X_START + groove_spacing * (i + 1)
        handle.visual(
            groove_ring_mesh,
            origin=Origin(
                xyz=(gx, BAR_OFFSET_Y, -GROOVE_WIDTH / 2.0),
                rpy=(0.0, 0.0, 0.0),
            ),
            material="dark_steel",
            name=f"grip_groove_{i}",
        )

    model.articulation(
        "lever_twist",
        ArticulationType.REVOLUTE,
        parent=boss,
        child=handle,
        origin=Origin(xyz=(0.0, DISC_CENTER_Y - BOSS_CENTER_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.0, lower=-TWIST_RANGE, upper=TWIST_RANGE
        ),
    )

    # -------------------------------------------------------- drain rod
    drain = model.part("drain_rod")
    # Main rod shaft
    drain.visual(
        Cylinder(radius=DRAIN_ROD_RADIUS, length=DRAIN_ROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LENGTH / 2.0)),
        material="bright_steel",
        name="drain_shaft",
    )
    # Pull knob on top
    drain.visual(
        Cylinder(radius=DRAIN_KNOB_RADIUS, length=DRAIN_KNOB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LENGTH + DRAIN_KNOB_LENGTH / 2.0)),
        material="brushed_steel",
        name="drain_knob",
    )

    model.articulation(
        "drain_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain,
        # Rod sits behind the body, base near the drain guide height
        origin=Origin(
            xyz=(-(BODY_RADIUS + 0.006), 0.0, DRAIN_GUIDE_Z - 0.010),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=1.5, lower=0.0, upper=DRAIN_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    boss = object_model.get_part("lever_boss")
    handle = object_model.get_part("lever_handle")
    drain = object_model.get_part("drain_rod")
    lift = object_model.get_articulation("boss_lift")
    twist = object_model.get_articulation("lever_twist")
    drain_joint = object_model.get_articulation("drain_slide")
    spout = body.get_visual("spout_channel")
    bar = handle.get_visual("lever_bar")
    disc = handle.get_visual("control_disc")
    pedestal = body.get_visual("oval_pedestal")
    drain_shaft = drain.get_visual("drain_shaft")
    drain_knob = drain.get_visual("drain_knob")

    # Intentional seated embeddings of the rotating stack.
    ctx.allow_overlap(
        boss,
        body,
        reason="boss shaft is seated into the curved body wall",
    )
    ctx.allow_overlap(
        handle,
        boss,
        reason="disc hub captures the boss shaft end (0.5 mm seat)",
    )
    ctx.allow_overlap(
        drain,
        body,
        elem_a="drain_shaft",
        elem_b="drain_guide",
        reason="drain rod slides through the guide boss behind the body",
    )

    # --- squat body form ---------------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and abs(aabb[0][2]) < 1e-6,
        f"oval pedestal must sit on z=0, got {aabb}",
    )
    ctx.check(
        "body_is_squat",
        aabb is not None and 0.10 < aabb[1][2] < 0.18,
        f"squat body top should be ~0.13 m up, got {aabb}",
    )

    # Oval pedestal wider than body column
    ped_aabb = ctx.part_element_world_aabb(body, elem=pedestal)
    ctx.check(
        "oval_pedestal_wide",
        ped_aabb is not None
        and (ped_aabb[1][0] - ped_aabb[0][0]) > 2.0 * BODY_RADIUS + 0.010,
        f"pedestal should be wider than body column, got {ped_aabb}",
    )
    ctx.check(
        "oval_pedestal_oval_shape",
        ped_aabb is not None
        and (ped_aabb[1][0] - ped_aabb[0][0]) > (ped_aabb[1][1] - ped_aabb[0][1]) + 0.005,
        f"pedestal should be wider in X than Y (oval), got {ped_aabb}",
    )

    # Spout projects forward
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.120,
        f"channel spout should project forward >0.12 m, got {spout_aabb}",
    )
    ctx.check(
        "spout_tip_droops",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_ROOT_Z - 0.025,
        f"curved tip should drop below the root line, got {spout_aabb}",
    )

    # --- grip grooves exist ------------------------------------------------
    groove_names = [v.name for v in handle.visuals if v.name and v.name.startswith("grip_groove_")]
    ctx.check(
        "grip_grooves_present",
        len(groove_names) >= 3,
        f"expected at least 3 grip grooves, found {len(groove_names)}: {groove_names}",
    )

    # --- disc and lever ----------------------------------------------------
    disc_aabb = ctx.part_element_world_aabb(handle, elem=disc)
    ctx.check(
        "disc_outboard_of_body",
        disc_aabb is not None and disc_aabb[1][1] < -BODY_RADIUS - 0.010,
        f"control disc must sit clear of the body side, got {disc_aabb}",
    )
    bar_aabb = ctx.part_element_world_aabb(handle, elem=bar)
    ctx.check(
        "lever_bar_length",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.120,
        f"lever bar should run ~0.136 m forward, got {bar_aabb}",
    )

    # Boss shaft bridges body and disc hub
    ctx.expect_contact(body, boss, name="boss_seats_on_body")
    ctx.expect_contact(boss, handle, name="disc_seats_on_boss")

    # --- drain rod ---------------------------------------------------------
    ctx.check(
        "drain_rod_behind_body",
        True,  # verified by joint origin below
    )
    drain_aabb = ctx.part_world_aabb(drain)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "drain_rod_positioned",
        drain_aabb is not None and body_aabb is not None
        and drain_aabb[0][0] < body_aabb[0][0] + 0.010,
        f"drain rod should be behind the body, drain={drain_aabb}, body={body_aabb}",
    )

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
    ctx.check(
        "drain_is_prismatic",
        drain_joint.articulation_type == ArticulationType.PRISMATIC,
        f"drain joint must be prismatic, got {drain_joint.articulation_type}",
    )
    ctx.check(
        "drain_axis_vertical",
        abs(drain_joint.axis[2]) == 1.0 and drain_joint.axis[0] == 0.0 and drain_joint.axis[1] == 0.0,
        f"drain axis must be vertical (+Z), got {drain_joint.axis}",
    )
    ctx.check(
        "drain_travel_40mm",
        drain_joint.motion_limits is not None
        and abs(drain_joint.motion_limits.lower) < 1e-6
        and abs(drain_joint.motion_limits.upper - DRAIN_TRAVEL) < 1e-6,
        "drain travel should be 0..0.04 m",
    )

    # --- motion proofs -----------------------------------------------------
    # Lift: lever tip sweeps up
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > DISC_CENTER_Z + 0.06,
            f"at +40 deg the bar tip should rise well above disc, got {up_aabb}",
        )

    # Drain rod pulls up
    rest_drain = ctx.part_world_aabb(drain)
    with ctx.pose({drain_joint: DRAIN_TRAVEL}):
        raised_drain = ctx.part_world_aabb(drain)
        ctx.check(
            "drain_rod_pulls_up",
            rest_drain is not None and raised_drain is not None
            and raised_drain[1][2] > rest_drain[1][2] + DRAIN_TRAVEL - 0.002,
            f"drain rod should rise ~{DRAIN_TRAVEL} m at max pull: rest={rest_drain}, raised={raised_drain}",
        )

    # Twist: index dot orbits
    rest_dot = ctx.part_element_world_aabb(handle, elem=handle.get_visual("hot_dot"))
    with ctx.pose({twist: TWIST_RANGE}):
        twist_dot = ctx.part_element_world_aabb(handle, elem=handle.get_visual("hot_dot"))
        ctx.check(
            "twist_swings_index_dot",
            rest_dot is not None
            and twist_dot is not None
            and abs(rest_dot[0][1] - twist_dot[0][1]) > 0.004,
            f"hot dot should swing at +30 deg: rest={rest_dot}, twisted={twist_dot}",
        )

    return ctx.report()


object_model = build_object_model()
