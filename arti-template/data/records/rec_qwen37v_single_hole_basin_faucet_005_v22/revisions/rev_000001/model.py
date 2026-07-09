from __future__ import annotations

"""Single-hole basin faucet variant — squat oval pedestal with swiveling spout.

A compact single-hole basin faucet (~0.14 m tall) with a squat cylindrical
body on a wide oval pedestal. The open-channel spout swivels around the
vertical body axis at the top of the column. On the right side near the top,
a round control disc carries a slim lever bar with subtle grip grooves.
Two small screw caps are visible on the back of the body.

Articulation chain:
- ``body_to_spout``: revolute about Z (vertical), -120..+120 deg; swivels
  the spout left and right around the body column.
- ``boss_lift``: revolute about the horizontal sideways (Y) boss axis,
  -40..+40 deg; positive q lifts the lever tip up (flow control).
- ``lever_twist``: revolute about the lever's own forward (X) axis,
  -30..+30 deg (temperature mix).
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
BODY_HEIGHT = 0.130  # squat body

# Oval pedestal: wider than body, elliptical
PEDESTAL_RX = 0.050  # semi-axis along X (front-back)
PEDESTAL_RY = 0.040  # semi-axis along Y (left-right)
PEDESTAL_HEIGHT = 0.018

SPOUT_PIVOT_Z = BODY_HEIGHT  # spout swivels at top of body
SPOUT_OUTER_W = 0.034
SPOUT_OUTER_H = 0.022
SPOUT_WALL = 0.005
SPOUT_FLOOR = 0.006
SPOUT_COLLAR_OUTER_R = 0.032  # swivel collar outer radius (larger than body)
SPOUT_COLLAR_INNER_R = BODY_RADIUS + 0.001  # slightly clearance over body
SPOUT_COLLAR_HEIGHT = 0.014

DISC_RADIUS = 0.0275  # 0.055 m diameter control disc
DISC_THICKNESS = 0.012
DISC_CENTER_Y = -0.048  # disc mid-plane (outboard of the body surface)
DISC_CENTER_Z = 0.105  # near top of squat body

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

# Grip grooves on the lever bar
GROOVE_COUNT = 6
GROOVE_WIDTH = 0.002
GROOVE_DEPTH = 0.0012
GROOVE_SPACING = 0.012

# Screw caps on back of body
SCREW_CAP_RADIUS = 0.005
SCREW_CAP_HEIGHT = 0.003
SCREW_CAP_Z_OFFSET = 0.020  # vertical separation between the two caps
SCREW_CAP_CENTER_Z = BODY_HEIGHT * 0.55  # roughly mid-upper body

SWIVEL_RANGE = math.radians(120.0)
LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)


def _build_oval_pedestal() -> cq.Workplane:
    """Wide oval pedestal base — extruded ellipse."""
    return (
        cq.Workplane("XY")
        .ellipse(PEDESTAL_RX, PEDESTAL_RY)
        .extrude(PEDESTAL_HEIGHT)
    )


def _build_spout_with_collar() -> cq.Workplane:
    """Swivel ring collar + open-top U-channel spout projecting forward.

    The collar is a hollow annulus that wraps around the body column top.
    The spout channel extends forward (+X) from the collar top and curves
    down at the tip.
    """
    # Swivel collar: hollow ring (annulus)
    collar = (
        cq.Workplane("XY")
        .circle(SPOUT_COLLAR_OUTER_R)
        .circle(SPOUT_COLLAR_INNER_R)
        .extrude(SPOUT_COLLAR_HEIGHT)
    )
    # Channel spout path — starts from collar outer surface heading +X
    path = cq.Workplane("XZ").spline(
        [
            (SPOUT_COLLAR_OUTER_R + 0.002, 0.000),
            (SPOUT_COLLAR_OUTER_R + 0.035, -0.003),
            (SPOUT_COLLAR_OUTER_R + 0.068, -0.010),
            (SPOUT_COLLAR_OUTER_R + 0.098, -0.022),
            (SPOUT_COLLAR_OUTER_R + 0.115, -0.040),
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
    channel = profile.sweep(path)
    # Position channel at collar mid-height
    channel = channel.translate((0.0, 0.0, SPOUT_COLLAR_HEIGHT * 0.4))
    return collar.union(channel)


def _build_grooved_bar() -> cq.Workplane:
    """Lever bar with subtle grip grooves cut into the surface.

    The bar is a cylinder along X with shallow annular grooves.
    Built in local coords: cylinder along X from BAR_X_START to BAR_X_END.
    """
    bar_len = BAR_X_END - BAR_X_START
    # Main bar cylinder
    bar = (
        cq.Workplane("YZ")
        .circle(BAR_RADIUS)
        .extrude(bar_len)
    )
    bar = bar.translate((BAR_X_START, 0.0, 0.0))

    # Cut annular groove rings along the bar
    groove_start = BAR_X_START + 0.030
    for i in range(GROOVE_COUNT):
        gx = groove_start + i * GROOVE_SPACING
        if gx + GROOVE_WIDTH > BAR_X_END - 0.010:
            break
        # Groove as a thin cylinder slightly larger than bar, used to cut
        groove_ring = (
            cq.Workplane("YZ")
            .circle(BAR_RADIUS + 0.001)
            .circle(BAR_RADIUS - GROOVE_DEPTH)
            .extrude(GROOVE_WIDTH)
        )
        groove_ring = groove_ring.translate((gx, 0.0, 0.0))
        bar = bar.cut(groove_ring)
    return bar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))
    model.material("dark_cap", rgba=(0.30, 0.30, 0.32, 1.0))
    model.material("groove_dark", rgba=(0.50, 0.51, 0.53, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    # Wide oval pedestal
    body.visual(
        mesh_from_cadquery(_build_oval_pedestal(), "oval_pedestal"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="brushed_steel",
        name="oval_pedestal",
    )
    # Squat body column on top of pedestal
    column_len = BODY_HEIGHT - PEDESTAL_HEIGHT
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    # Flat top cap (smaller than collar inner radius so no overlap)
    body.visual(
        Cylinder(radius=BODY_RADIUS - 0.001, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.0015)),
        material="bright_steel",
        name="body_cap",
    )
    # Two screw caps on the back of the body (-X side)
    screw_y = 0.0
    screw_x = -(BODY_RADIUS + SCREW_CAP_HEIGHT / 2.0 - 0.001)
    for idx, z_off in enumerate([-SCREW_CAP_Z_OFFSET / 2.0, SCREW_CAP_Z_OFFSET / 2.0]):
        body.visual(
            Cylinder(radius=SCREW_CAP_RADIUS, length=SCREW_CAP_HEIGHT),
            origin=Origin(
                xyz=(screw_x, screw_y, SCREW_CAP_CENTER_Z + z_off),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="dark_cap",
            name=f"screw_cap_{idx}",
        )

    # ------------------------------------------------------------------ spout
    # Separate swiveling spout part
    spout = model.part("spout_arm")
    spout.visual(
        mesh_from_cadquery(_build_spout_with_collar(), "spout_assembly"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="brushed_steel",
        name="spout_channel",
    )

    model.articulation(
        "body_to_spout",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        # Joint at top of body column; collar straddles the body top.
        # Hollow collar inner radius > body radius so no 3D overlap.
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=1.5, lower=-SWIVEL_RANGE, upper=SWIVEL_RANGE
        ),
    )

    # ------------------------------------------------------------------ boss
    # Short horizontal boss on the side of the body that carries the
    # disc-and-lever assembly.
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
    # Grooved lever bar
    handle.visual(
        mesh_from_cadquery(_build_grooved_bar(), "grooved_lever_bar"),
        origin=Origin(xyz=(0.0, BAR_OFFSET_Y, 0.0)),
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    spout = object_model.get_part("spout_arm")
    boss = object_model.get_part("lever_boss")
    handle = object_model.get_part("lever_handle")
    swivel = object_model.get_articulation("body_to_spout")
    lift = object_model.get_articulation("boss_lift")
    twist = object_model.get_articulation("lever_twist")

    # Intentional seated embeddings
    ctx.allow_overlap(
        spout,
        body,
        reason="hollow swivel collar nests around body top with 1mm radial clearance for the swivel joint",
    )
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

    # --- static form: squat body with oval pedestal ----------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and abs(aabb[0][2]) < 1e-6,
        f"pedestal base must sit on z=0, got {aabb}",
    )
    ctx.check(
        "body_squat_height",
        aabb is not None and 0.12 < aabb[1][2] < 0.16,
        f"squat body should be ~0.13-0.14 m tall, got {aabb}",
    )

    # Pedestal is wider than the body column
    pedestal = body.get_visual("oval_pedestal")
    pedestal_aabb = ctx.part_element_world_aabb(body, elem=pedestal)
    ctx.check(
        "pedestal_wider_than_body",
        pedestal_aabb is not None
        and (pedestal_aabb[1][0] - pedestal_aabb[0][0]) > 2.0 * BODY_RADIUS + 0.01,
        f"oval pedestal X-span must exceed body diameter, got {pedestal_aabb}",
    )
    ctx.check(
        "pedestal_oval_shape",
        pedestal_aabb is not None
        and abs(
            (pedestal_aabb[1][0] - pedestal_aabb[0][0])
            - (pedestal_aabb[1][1] - pedestal_aabb[0][1])
        ) > 0.005,
        f"pedestal should be oval (not circular), got {pedestal_aabb}",
    )

    # Screw caps on back of body
    cap0 = body.get_visual("screw_cap_0")
    cap1 = body.get_visual("screw_cap_1")
    cap0_aabb = ctx.part_element_world_aabb(body, elem=cap0)
    cap1_aabb = ctx.part_element_world_aabb(body, elem=cap1)
    ctx.check(
        "screw_caps_exist",
        cap0_aabb is not None and cap1_aabb is not None,
        "two screw caps must be present on body back",
    )
    ctx.check(
        "screw_caps_on_back",
        cap0_aabb is not None and cap0_aabb[0][0] < -BODY_RADIUS + 0.005,
        f"screw caps should be on -X side (back) of body, got {cap0_aabb}",
    )

    # --- spout swivel joint ---------------------------------------------
    spout_aabb = ctx.part_element_world_aabb(spout, elem=spout.get_visual("spout_channel"))
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.10,
        f"spout should project forward, got {spout_aabb}",
    )

    ctx.check(
        "swivel_axis_vertical",
        abs(swivel.axis[2]) == 1.0 and swivel.axis[0] == 0.0 and swivel.axis[1] == 0.0,
        f"spout swivel must rotate about vertical Z axis, got {swivel.axis}",
    )
    ctx.check(
        "swivel_range_pm120deg",
        swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_RANGE) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_RANGE) < 1e-6,
        "swivel range must be -120..+120 deg",
    )

    # Prove the spout actually swivels horizontally
    rest_spout = ctx.part_element_world_aabb(spout, elem=spout.get_visual("spout_channel"))
    with ctx.pose({swivel: SWIVEL_RANGE * 0.5}):
        swiveled_spout = ctx.part_element_world_aabb(spout, elem=spout.get_visual("spout_channel"))
        ctx.check(
            "swivel_rotates_spout_horizontally",
            rest_spout is not None
            and swiveled_spout is not None
            and abs(swiveled_spout[1][1] - rest_spout[1][1]) > 0.01,
            f"spout Y-extent should change when swiveled: rest={rest_spout}, swiveled={swiveled_spout}",
        )

    # --- lever articulation checks --------------------------------------
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

    # --- motion proof: lift ----------------------------------------------
    bar = handle.get_visual("lever_bar")
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > DISC_CENTER_Z + 0.04,
            f"at +40 deg the bar tip should rise, got {up_aabb}",
        )

    # --- motion proof: twist ---------------------------------------------
    hot_dot = handle.get_visual("hot_dot")
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

    # --- support contacts -----------------------------------------------
    ctx.expect_contact(body, boss, name="boss_seats_on_body")
    ctx.expect_contact(boss, handle, name="disc_seats_on_boss")

    return ctx.report()


object_model = build_object_model()
