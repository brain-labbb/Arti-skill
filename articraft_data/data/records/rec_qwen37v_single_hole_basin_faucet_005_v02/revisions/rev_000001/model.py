from __future__ import annotations

"""Squat single-hole basin faucet variant with oval pedestal.

A compact single-lever basin faucet (~0.13 m tall) with a wide oval pedestal
base, a short cylindrical body, and a forward-projecting channel spout. The
outlet aerator at the spout tip flips open on a tiny hinge. Subtle grip
grooves are machined into the lever bar surface, and a thin cartridge cap
seam ring sits below the lever assembly on the body.

Articulation chain (body -> boss -> disc/lever, body -> aerator):
- ``boss_lift``: revolute about the horizontal sideways (Y) boss axis,
  -40..+40 deg; positive q lifts the lever tip up (flow control).
- ``lever_twist``: revolute about the lever's own forward (X) axis through
  the disc center, -30..+30 deg (temperature mix).
- ``aerator_flip``: revolute about the spout-tip hinge axis, 0..90 deg;
  positive q opens the aerator disc outward for cleaning.
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
# -- Squat body variant
BODY_RADIUS = 0.030       # 0.060 m diameter column (wider than parent)
BODY_HEIGHT = 0.130       # squat: 0.13 m total

# -- Wide oval pedestal
PEDESTAL_RX = 0.048       # semi-axis in X (forward-back)
PEDESTAL_RY = 0.038       # semi-axis in Y (side-side)
PEDESTAL_HEIGHT = 0.016

# -- Spout (shorter projection for squat variant)
SPOOT_ROOT_Z = 0.078      # channel centerline height where it leaves the body
SPOUT_OUTER_W = 0.030
SPOUT_OUTER_H = 0.020
SPOUT_WALL = 0.004
SPOUT_FLOOR = 0.005

# -- Cartridge cap seam
SEAM_Z = 0.090            # height on body where seam ring sits
SEAM_OUTER_R = 0.033      # slightly proud of body
SEAM_THICKNESS = 0.002

# -- Control disc and boss (kept similar to parent)
DISC_RADIUS = 0.0275
DISC_THICKNESS = 0.012
DISC_CENTER_Y = -0.048
DISC_CENTER_Z = 0.105     # adjusted for squat body

BOSS_RADIUS = 0.011
BOSS_LENGTH = 0.018
BOSS_CENTER_Y = -0.0335

# -- Lever bar with grip grooves
BAR_RADIUS = 0.005
BAR_X_START = 0.014
BAR_X_END = 0.140
BAR_OFFSET_Y = 0.0095

# -- Grip grooves (rings on lever bar)
GROOVE_COUNT = 6
GROOVE_DEPTH = 0.0012
GROOVE_WIDTH = 0.003

# -- Paint dots on disc
DOT_RADIUS = 0.0035
DOT_LENGTH = 0.0018
DOT_Z = 0.019

# -- Aerator at spout tip
AERATOR_RADIUS = 0.009
AERATOR_THICKNESS = 0.004
# Position of aerator hinge at spout tip
AERATOR_TIP_X = 0.119     # approximate spout tip forward position
AERATOR_TIP_Z = 0.047     # approximate spout tip height (drooped, seated into spout wall)

# -- Joint ranges
LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)
AERATOR_RANGE = math.radians(90.0)


def _build_oval_pedestal() -> cq.Workplane:
    """Wide oval pedestal base using an elliptical extrusion."""
    # Build an ellipse profile and extrude upward
    pedestal = (
        cq.Workplane("XY")
        .ellipse(PEDESTAL_RX, PEDESTAL_RY)
        .extrude(PEDESTAL_HEIGHT)
    )
    return pedestal


def _build_spout() -> cq.Workplane:
    """Open-top U-channel swept forward and curving down at the tip.

    Built in spout-local coordinates: channel centerline starts at the
    origin heading +X; placed at body front at SPOOT_ROOT_Z.
    """
    path = cq.Workplane("XZ").spline(
        [
            (0.000, 0.000),
            (0.045, -0.003),
            (0.080, -0.010),
            (0.105, -0.020),
            (0.120, -0.032),
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


def _build_grooved_lever_bar() -> cq.Workplane:
    """Slim cylindrical lever bar with subtle grip grooves (ring cuts)."""
    bar_len = BAR_X_END - BAR_X_START
    # Start with the base cylinder along X
    bar = (
        cq.Workplane("YZ")
        .circle(BAR_RADIUS)
        .extrude(bar_len)
    )
    # Cut shallow ring grooves along the grip region (middle 60% of bar)
    grip_start = bar_len * 0.25
    grip_end = bar_len * 0.85
    groove_spacing = (grip_end - grip_start) / (GROOVE_COUNT - 1) if GROOVE_COUNT > 1 else 0.0
    for i in range(GROOVE_COUNT):
        x_pos = grip_start + i * groove_spacing
        # Cut a thin ring groove: remove a torus-like ring
        groove = (
            cq.Workplane("YZ")
            .workplane(offset=x_pos - GROOVE_WIDTH / 2.0)
            .circle(BAR_RADIUS + 0.0001)
            .circle(BAR_RADIUS - GROOVE_DEPTH)
            .extrude(GROOVE_WIDTH)
        )
        bar = bar.cut(groove)
    return bar


def _build_seam_ring() -> cq.Workplane:
    """Thin cartridge cap seam ring that sits proud of the body."""
    ring = (
        cq.Workplane("XY")
        .circle(SEAM_OUTER_R)
        .circle(BODY_RADIUS - 0.001)
        .extrude(SEAM_THICKNESS)
    )
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_v02")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))
    model.material("dark_seam", rgba=(0.45, 0.46, 0.48, 1.0))
    model.material("aerator_mesh", rgba=(0.55, 0.56, 0.58, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    # Oval pedestal base
    body.visual(
        mesh_from_cadquery(_build_oval_pedestal(), "oval_pedestal"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="brushed_steel",
        name="oval_pedestal",
    )
    # Short cylindrical column
    column_len = BODY_HEIGHT - PEDESTAL_HEIGHT - 0.005
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    # Flat top cap
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.0025)),
        material="brushed_steel",
        name="body_cap",
    )
    # Spout channel
    body.visual(
        mesh_from_cadquery(_build_spout(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, SPOOT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )
    # Cartridge cap seam ring
    body.visual(
        mesh_from_cadquery(_build_seam_ring(), "cartridge_seam"),
        origin=Origin(xyz=(0.0, 0.0, SEAM_Z)),
        material="dark_seam",
        name="cartridge_seam",
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
    # Grooved lever bar
    bar_len = BAR_X_END - BAR_X_START
    handle.visual(
        mesh_from_cadquery(_build_grooved_lever_bar(), "grooved_lever_bar"),
        origin=Origin(
            xyz=(BAR_X_START, BAR_OFFSET_Y, 0.0),
        ),
        material="bright_steel",
        name="lever_bar",
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

    # ------------------------------------------------------- aerator on hinge
    aerator = model.part("aerator")
    aerator.visual(
        Cylinder(radius=AERATOR_RADIUS, length=AERATOR_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="aerator_mesh",
        name="aerator_disc",
    )
    # Small hinge pin visual connecting aerator to spout tip
    aerator.visual(
        Cylinder(radius=0.002, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, AERATOR_THICKNESS / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="hinge_pin",
    )

    model.articulation(
        "aerator_flip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=aerator,
        # Place at spout tip, hinge axis along Y (sideways)
        origin=Origin(xyz=(AERATOR_TIP_X, 0.0, AERATOR_TIP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.5, lower=0.0, upper=AERATOR_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    boss = object_model.get_part("lever_boss")
    handle = object_model.get_part("lever_handle")
    aerator = object_model.get_part("aerator")
    lift = object_model.get_articulation("boss_lift")
    twist = object_model.get_articulation("lever_twist")
    aerator_flip = object_model.get_articulation("aerator_flip")
    spout = body.get_visual("spout_channel")
    bar = handle.get_visual("lever_bar")
    disc = handle.get_visual("control_disc")
    hot_dot = handle.get_visual("hot_dot")
    pedestal = body.get_visual("oval_pedestal")
    seam = body.get_visual("cartridge_seam")
    aerator_disc = aerator.get_visual("aerator_disc")

    # Intentional seated embeddings of the rotating stack.
    ctx.allow_overlap(
        boss,
        body,
        reason="boss shaft is seated into the curved body wall",
    )
    ctx.allow_overlap(
        handle,
        boss,
        reason="disc hub captures the boss shaft end (seat)",
    )
    ctx.allow_overlap(
        aerator,
        body,
        elem_a="hinge_pin",
        elem_b="spout_channel",
        reason="aerator hinge pin is seated into the spout tip wall",
    )

    # --- squat body form --------------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and abs(aabb[0][2]) < 1e-6,
        f"pedestal must sit on z=0, got {aabb}",
    )
    ctx.check(
        "body_is_squat",
        aabb is not None and 0.120 < aabb[1][2] < 0.140,
        f"squat body top should be ~0.13 m up, got {aabb}",
    )

    # --- wide oval pedestal -----------------------------------------------
    ped_aabb = ctx.part_element_world_aabb(body, elem=pedestal)
    ctx.check(
        "pedestal_wider_than_body",
        ped_aabb is not None
        and (ped_aabb[1][0] - ped_aabb[0][0]) > 2.0 * BODY_RADIUS + 0.010
        and (ped_aabb[1][1] - ped_aabb[0][1]) > 2.0 * BODY_RADIUS + 0.005,
        f"oval pedestal must extend beyond the body column, got {ped_aabb}",
    )
    ctx.check(
        "pedestal_is_oval",
        ped_aabb is not None
        and abs((ped_aabb[1][0] - ped_aabb[0][0]) - (ped_aabb[1][1] - ped_aabb[0][1])) > 0.005,
        f"pedestal should be oval (X extent != Y extent), got {ped_aabb}",
    )

    # --- spout projects forward -------------------------------------------
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.100,
        f"channel spout should reach >0.10 m forward, got {spout_aabb}",
    )
    ctx.check(
        "spout_tip_droops",
        spout_aabb is not None and spout_aabb[0][2] < SPOOT_ROOT_Z - 0.020,
        f"curved tip should drop below the root line, got {spout_aabb}",
    )

    # --- cartridge cap seam -----------------------------------------------
    seam_aabb = ctx.part_element_world_aabb(body, elem=seam)
    ctx.check(
        "seam_below_lever",
        seam_aabb is not None and seam_aabb[1][2] < DISC_CENTER_Z - 0.005,
        f"cartridge seam should be below the lever, got {seam_aabb}",
    )
    ctx.check(
        "seam_is_thin_ring",
        seam_aabb is not None and (seam_aabb[1][2] - seam_aabb[0][2]) < 0.005,
        f"seam should be a thin ring (<5 mm tall), got {seam_aabb}",
    )

    # --- grip grooves on lever bar ----------------------------------------
    bar_aabb = ctx.part_element_world_aabb(handle, elem=bar)
    ctx.check(
        "lever_bar_exists",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.100,
        f"grooved lever bar should be >0.10 m long, got {bar_aabb}",
    )

    # --- disc position ----------------------------------------------------
    disc_aabb = ctx.part_element_world_aabb(handle, elem=disc)
    ctx.check(
        "disc_outboard_of_body",
        disc_aabb is not None and disc_aabb[1][1] < -BODY_RADIUS - 0.010,
        f"control disc must sit clear of the body side, got {disc_aabb}",
    )

    # --- aerator exists at spout tip --------------------------------------
    aer_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_disc)
    ctx.check(
        "aerator_at_spout_tip",
        aer_aabb is not None and aer_aabb[0][0] > 0.080,
        f"aerator should be near spout tip (x>0.08), got {aer_aabb}",
    )

    # --- joint plan -------------------------------------------------------
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
        "aerator_flip_exists",
        aerator_flip is not None
        and aerator_flip.articulation_type == ArticulationType.REVOLUTE,
        "aerator_flip articulation must be revolute",
    )
    ctx.check(
        "aerator_flip_range",
        aerator_flip.motion_limits is not None
        and abs(aerator_flip.motion_limits.lower) < 1e-6
        and abs(aerator_flip.motion_limits.upper - AERATOR_RANGE) < 1e-6,
        "aerator flip range must be 0..90 deg",
    )

    # --- motion proof: lift -----------------------------------------------
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > DISC_CENTER_Z + 0.040,
            f"at +40 deg the bar tip should rise above disc center, got {up_aabb}",
        )
    with ctx.pose({lift: -LIFT_RANGE}):
        down_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_down_lowers_lever_tip",
            down_aabb is not None and down_aabb[0][2] < DISC_CENTER_Z - 0.030,
            f"at -40 deg the bar tip should drop below disc center, got {down_aabb}",
        )

    # --- motion proof: twist swings index dot ------------------------------
    rest_dot = ctx.part_element_world_aabb(handle, elem=hot_dot)
    with ctx.pose({twist: TWIST_RANGE}):
        twist_dot = ctx.part_element_world_aabb(handle, elem=hot_dot)
        ctx.check(
            "twist_swings_index_dot",
            rest_dot is not None
            and twist_dot is not None
            and abs(rest_dot[0][1] - twist_dot[0][1]) > 0.003,
            f"hot dot should swing at +30 deg: rest={rest_dot}, twisted={twist_dot}",
        )

    # --- motion proof: aerator flips open ----------------------------------
    rest_aer = ctx.part_element_world_aabb(aerator, elem=aerator_disc)
    with ctx.pose({aerator_flip: AERATOR_RANGE}):
        open_aer = ctx.part_element_world_aabb(aerator, elem=aerator_disc)
        ctx.check(
            "aerator_opens_outward",
            rest_aer is not None
            and open_aer is not None
            and open_aer[1][2] > rest_aer[1][2] + 0.005,
            f"at 90 deg the aerator should rise: rest={rest_aer}, open={open_aer}",
        )

    # Boss shaft bridges body and disc hub.
    ctx.expect_contact(body, boss, name="boss_seats_on_body")
    ctx.expect_contact(boss, handle, name="disc_seats_on_boss")

    return ctx.report()


object_model = build_object_model()
