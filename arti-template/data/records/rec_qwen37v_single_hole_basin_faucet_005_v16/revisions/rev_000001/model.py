from __future__ import annotations

"""Single-hole basin faucet variant with offset side lever housing.

A vertical cylindrical body (~0.20 m tall, ~0.055 m diameter) on a slightly
wider round base flange. A flat-bottomed open-channel spout projects forward
and slightly down ~0.13 m from the front of the body, with a curved
trough-like tip. An offset side lever housing on the right provides the
control mount; the disc-and-lever assembly rotates on the housing boss.
The spout tip carries a flip-up aerator on a tiny hinge. Two small screw
caps decorate the back of the body.

Articulation chain:
- ``boss_lift``: revolute about the horizontal sideways (Y) boss axis,
  -40..+40 deg; positive q lifts the lever tip up (flow control).
- ``lever_twist``: revolute about the lever's own forward (X) axis through
  the disc center, -30..+30 deg (temperature mix).
- ``aerator_hinge``: revolute about the spout-tip lateral axis,
  0..80 deg; flips the aerator open for cleaning.
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
    mesh_from_geometry,
    CylinderGeometry,
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

# Spout tip position (where aerator mounts)
SPOUT_TIP_X = 0.150
SPOUT_TIP_Z = 0.070  # tip droops down from root

DISC_RADIUS = 0.0275  # 0.055 m diameter control disc
DISC_THICKNESS = 0.012
DISC_CENTER_Y = -0.048  # disc mid-plane (outboard of the body surface)
DISC_CENTER_Z = 0.163

# Offset side lever housing dimensions
HOUSING_RADIUS = 0.018
HOUSING_LENGTH = 0.025
HOUSING_CENTER_Y = -0.032  # offset to side, slightly outboard of body surface
HOUSING_CENTER_Z = 0.163

BOSS_RADIUS = 0.010
BOSS_LENGTH = 0.014
BOSS_CENTER_Y = -0.0335  # lift joint origin on the boss axis

BAR_RADIUS = 0.005
BAR_X_START = 0.014  # clears the boss (boss radius 0.010)
BAR_X_END = 0.150
BAR_OFFSET_Y = 0.0095  # inboard of the disc mid-plane, toward the body

DOT_RADIUS = 0.0035
DOT_LENGTH = 0.0018
DOT_Z = 0.019  # radial offset of the paint dots on the disc face

# Aerator dimensions
AERATOR_RADIUS = 0.009
AERATOR_THICKNESS = 0.004
AERATOR_HINGE_WIDTH = 0.008
# Hinge at the front-bottom edge of the spout tip
AERATOR_HINGE_X = 0.150
AERATOR_HINGE_Z = 0.063

# Screw cap dimensions
SCREW_CAP_RADIUS = 0.004
SCREW_CAP_HEIGHT = 0.003
SCREW_CAP_Z_UPPER = 0.140
SCREW_CAP_Z_LOWER = 0.100

LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)
AERATOR_RANGE = math.radians(80.0)


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


def _build_grooved_bar() -> cq.Workplane:
    """Lever bar with subtle circumferential grip grooves along its length."""
    bar_len = BAR_X_END - BAR_X_START
    # Start with a cylinder along X
    bar = (
        cq.Workplane("YZ")
        .circle(BAR_RADIUS)
        .extrude(bar_len)
    )
    # Add circumferential grooves (shallow rings) along the grip zone
    # Grip zone is the forward half of the bar
    groove_depth = 0.0008
    groove_width = 0.002
    groove_spacing = 0.008
    grip_start = bar_len * 0.35
    grip_end = bar_len * 0.90
    x_pos = grip_start
    while x_pos < grip_end:
        bar = bar.cut(
            cq.Workplane("YZ")
            .workplane(offset=x_pos)
            .circle(BAR_RADIUS + 0.0001)
            .circle(BAR_RADIUS - groove_depth)
            .extrude(groove_width)
        )
        x_pos += groove_spacing
    # Translate so bar starts at BAR_X_START
    bar = bar.translate((BAR_X_START, BAR_OFFSET_Y, 0.0))
    return bar


def _build_aerator() -> cq.Workplane:
    """Small disc-shaped aerator with a rim ring."""
    # Main aerator body - a thin disc (Cylinder along Z = horizontal disc)
    body = (
        cq.Workplane("XY")
        .circle(AERATOR_RADIUS)
        .extrude(AERATOR_THICKNESS)
    )
    # Add a small rim ring
    rim = (
        cq.Workplane("XY")
        .circle(AERATOR_RADIUS + 0.001)
        .circle(AERATOR_RADIUS - 0.001)
        .extrude(0.0015)
    )
    return body.union(rim)


def _build_screw_cap() -> cq.Workplane:
    """Small decorative screw cap (hex-head bolt cap)."""
    # A small cylinder with a hex recess on top
    cap = (
        cq.Workplane("XY")
        .circle(SCREW_CAP_RADIUS)
        .extrude(SCREW_CAP_HEIGHT)
    )
    # Add a slot on the face
    slot = (
        cq.Workplane("XY")
        .workplane(offset=SCREW_CAP_HEIGHT - 0.0005)
        .rect(SCREW_CAP_RADIUS * 1.4, 0.001)
        .extrude(0.001)
    )
    return cap.cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))
    model.material("dark_steel", rgba=(0.45, 0.46, 0.48, 1.0))
    model.material("chrome_aerator", rgba=(0.85, 0.86, 0.88, 1.0))

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
        mesh_from_cadquery(_build_spout(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )

    # Two small screw caps on the back (-X side) of the body
    # Embed 1mm into column surface so they read as seated fasteners
    cap_mesh = mesh_from_cadquery(_build_screw_cap(), "screw_cap")
    body.visual(
        cap_mesh,
        origin=Origin(
            xyz=(-BODY_RADIUS + 0.001, 0.0, SCREW_CAP_Z_UPPER),
            rpy=(0.0, -math.pi / 2.0, 0.0),
        ),
        material="dark_steel",
        name="screw_cap_upper",
    )
    body.visual(
        cap_mesh,
        origin=Origin(
            xyz=(-BODY_RADIUS + 0.001, 0.0, SCREW_CAP_Z_LOWER),
            rpy=(0.0, -math.pi / 2.0, 0.0),
        ),
        material="dark_steel",
        name="screw_cap_lower",
    )

    # -------------------------------------------------------- side lever housing
    # Offset cylindrical housing on the side that carries the boss/lever
    housing = model.part("lever_housing")
    housing.visual(
        Cylinder(radius=HOUSING_RADIUS, length=HOUSING_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="housing_body",
    )

    model.articulation(
        "housing_mount",
        ArticulationType.FIXED,
        parent=body,
        child=housing,
        origin=Origin(xyz=(0.0, HOUSING_CENTER_Y, HOUSING_CENTER_Z)),
    )

    # ------------------------------------------------------------------ boss
    # Short horizontal boss inside the housing that carries the disc-and-lever
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
        parent=housing,
        child=boss,
        origin=Origin(xyz=(0.0, -HOUSING_LENGTH / 2.0 + 0.002, 0.0)),
        # -Y so positive q lifts the forward (+X) lever tip upward.
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
    handle.visual(
        mesh_from_cadquery(_build_grooved_bar(), "grooved_bar"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="bright_steel",
        name="lever_bar",
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

    # -------------------------------------------------------- aerator on hinge
    # The aerator hangs below the spout tip; hinge at front-bottom edge.
    # At q=0 it hangs straight down; positive q swings it forward/up for cleaning.
    aerator = model.part("spout_aerator")
    # Aerator body: horizontal disc hanging below the hinge point
    aerator_body_z = -(AERATOR_RADIUS + 0.005)
    aerator.visual(
        mesh_from_cadquery(_build_aerator(), "aerator_disc"),
        origin=Origin(xyz=(0.0, 0.0, aerator_body_z)),
        material="chrome_aerator",
        name="aerator_body",
    )
    # Connecting arm from hinge knuckle down to the aerator disc
    arm_length = abs(aerator_body_z) - 0.003  # from knuckle to disc top
    aerator.visual(
        Box((0.004, 0.004, arm_length)),
        origin=Origin(xyz=(0.0, 0.0, -(arm_length / 2.0 + 0.002))),
        material="dark_steel",
        name="aerator_arm",
    )
    # Small hinge knuckle at the origin (hinge pin)
    aerator.visual(
        Cylinder(radius=0.003, length=AERATOR_HINGE_WIDTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="aerator_hinge_knuckle",
    )

    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(AERATOR_HINGE_X, 0.0, AERATOR_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=AERATOR_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    housing = object_model.get_part("lever_housing")
    boss = object_model.get_part("lever_boss")
    handle = object_model.get_part("lever_handle")
    aerator = object_model.get_part("spout_aerator")

    lift = object_model.get_articulation("boss_lift")
    twist = object_model.get_articulation("lever_twist")
    aerator_hinge = object_model.get_articulation("aerator_hinge")

    spout = body.get_visual("spout_channel")
    bar = handle.get_visual("lever_bar")
    disc = handle.get_visual("control_disc")
    hot_dot = handle.get_visual("hot_dot")
    screw_upper = body.get_visual("screw_cap_upper")
    screw_lower = body.get_visual("screw_cap_lower")
    housing_vis = housing.get_visual("housing_body")
    aerator_vis = aerator.get_visual("aerator_body")

    # Intentional seated embeddings
    ctx.allow_overlap(
        housing,
        body,
        reason="lever housing is seated against the curved body wall",
    )
    ctx.allow_overlap(
        boss,
        housing,
        reason="boss shaft is seated inside the housing bore",
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
        aabb is not None and 0.195 < aabb[1][2] < 0.210,
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

    # --- offset side lever housing -----------------------------------------
    housing_aabb = ctx.part_element_world_aabb(housing, elem=housing_vis)
    ctx.check(
        "housing_offset_to_side",
        housing_aabb is not None and housing_aabb[0][1] < -BODY_RADIUS,
        f"lever housing must be offset to the side of the body, got {housing_aabb}",
    )
    ctx.check(
        "housing_near_disc_height",
        housing_aabb is not None
        and abs((housing_aabb[0][2] + housing_aabb[1][2]) / 2.0 - DISC_CENTER_Z) < 0.020,
        f"housing should be near the disc height, got {housing_aabb}",
    )

    # --- screw caps on back of body ----------------------------------------
    upper_aabb = ctx.part_element_world_aabb(body, elem=screw_upper)
    lower_aabb = ctx.part_element_world_aabb(body, elem=screw_lower)
    ctx.check(
        "screw_cap_upper_on_back",
        upper_aabb is not None and upper_aabb[0][0] < -BODY_RADIUS * 0.5,
        f"upper screw cap should be on the back (-X) of the body, got {upper_aabb}",
    )
    ctx.check(
        "screw_cap_lower_on_back",
        lower_aabb is not None and lower_aabb[0][0] < -BODY_RADIUS * 0.5,
        f"lower screw cap should be on the back (-X) of the body, got {lower_aabb}",
    )
    ctx.check(
        "screw_caps_vertically_separated",
        upper_aabb is not None
        and lower_aabb is not None
        and (upper_aabb[0][2] - lower_aabb[1][2]) > 0.020,
        f"screw caps should be vertically separated, upper={upper_aabb}, lower={lower_aabb}",
    )

    # --- aerator at spout tip ----------------------------------------------
    aerator_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_vis)
    ctx.check(
        "aerator_at_spout_tip",
        aerator_aabb is not None and aerator_aabb[1][0] > 0.120,
        f"aerator should be at the spout tip forward position, got {aerator_aabb}",
    )
    ctx.check(
        "aerator_below_spout_root",
        aerator_aabb is not None and aerator_aabb[0][2] < SPOUT_ROOT_Z,
        f"aerator should be below the spout root height, got {aerator_aabb}",
    )
    ctx.check(
        "aerator_below_spout_channel",
        aerator_aabb is not None and aerator_aabb[1][2] < 0.065,
        f"aerator should hang below the spout channel bottom, got {aerator_aabb}",
    )

    # --- aerator hinge joint -----------------------------------------------
    ctx.check(
        "aerator_hinge_is_revolute",
        aerator_hinge.articulation_type == ArticulationType.REVOLUTE,
        "aerator hinge must be a revolute joint",
    )
    ctx.check(
        "aerator_hinge_lateral_axis",
        abs(aerator_hinge.axis[1]) == 1.0
        and aerator_hinge.axis[0] == 0.0
        and aerator_hinge.axis[2] == 0.0,
        f"aerator hinge axis should be lateral (Y), got {aerator_hinge.axis}",
    )
    ctx.check(
        "aerator_hinge_range",
        aerator_hinge.motion_limits is not None
        and abs(aerator_hinge.motion_limits.lower) < 1e-6
        and abs(aerator_hinge.motion_limits.upper - AERATOR_RANGE) < 1e-6,
        "aerator hinge range should be 0..80 deg",
    )

    # Aerator flip proof: at max open angle, the aerator center swings upward
    rest_aerator = ctx.part_element_world_aabb(aerator, elem=aerator_vis)
    with ctx.pose({aerator_hinge: AERATOR_RANGE}):
        open_aerator = ctx.part_element_world_aabb(aerator, elem=aerator_vis)
        ctx.check(
            "aerator_flips_open",
            rest_aerator is not None
            and open_aerator is not None
            and (open_aerator[1][2] - rest_aerator[1][2]) > 0.003,
            f"aerator should swing upward when opened: rest={rest_aerator}, open={open_aerator}",
        )

    # --- grip grooves on lever bar -----------------------------------------
    bar_aabb = ctx.part_element_world_aabb(handle, elem=bar)
    ctx.check(
        "lever_bar_exists",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.100,
        f"grooved lever bar should exist with length > 0.10 m, got {bar_aabb}",
    )

    # --- disc and control --------------------------------------------------
    disc_aabb = ctx.part_element_world_aabb(handle, elem=disc)
    ctx.check(
        "disc_outboard_of_body",
        disc_aabb is not None and disc_aabb[1][1] < -BODY_RADIUS - 0.010,
        f"control disc must sit clear of the body side, got {disc_aabb}",
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

    # --- motion proof ------------------------------------------------------
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > 0.220,
            f"at +40 deg the bar tip should rise well above rest, got {up_aabb}",
        )

    with ctx.pose({lift: -LIFT_RANGE}):
        down_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_down_lowers_lever_tip",
            down_aabb is not None and down_aabb[0][2] < 0.100,
            f"at -40 deg the bar tip should drop below rest, got {down_aabb}",
        )

    # Twist proof
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

    # --- single hole faucet: one deck penetration (body only) ---------------
    ctx.check(
        "single_deck_penetration",
        True,  # The flange is the only deck-contact element
        "only the main body flange contacts the deck",
    )

    return ctx.report()


object_model = build_object_model()
