from __future__ import annotations

"""Single-hole basin faucet variant with swivel spout, base collar, grip
grooves and cartridge cap seam.

A vertical cylindrical body (~0.20 m tall, ~0.055 m diameter) on a slightly
wider round base flange with a raised circular collar.  A flat-bottomed
open-channel spout projects forward and slightly down ~0.13 m, swiveling about
the vertical body axis.  On the side near the top, a round flat control disc
(~0.055 m diameter, with red/blue index dots) is carried on a short horizontal
boss; a thin cartridge cap seam ring sits below the lever.  A slim cylindrical
lever bar with subtle circumferential grip grooves runs forward from the disc.

Articulation:
- ``spout_swivel``: revolute about the vertical (Z) body axis, ±90 deg.
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
COLLAR_OUTER_RADIUS = 0.039
COLLAR_INNER_RADIUS = BODY_RADIUS + 0.0008
COLLAR_HEIGHT = 0.009

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

# Cartridge cap seam (thin annular ring just below the disc/lever assembly)
SEAM_OUTER_RADIUS = BODY_RADIUS + 0.002
SEAM_INNER_RADIUS = BODY_RADIUS - 0.002
SEAM_HEIGHT = 0.0018
SEAM_Z_CENTER = DISC_CENTER_Z - 0.028

# Subtle grip grooves along the lever bar
GROOVE_RADIUS = BAR_RADIUS + 0.0007  # slightly proud to suggest recessed ring
GROOVE_WIDTH = 0.0012
GROOVE_COUNT = 6

LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)
SWIVEL_RANGE = math.radians(90.0)


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
    """Raised annular collar around the base, sitting atop the flange."""
    return (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_RADIUS)
        .circle(COLLAR_INNER_RADIUS)
        .extrude(COLLAR_HEIGHT)
    )


def _build_seam_ring() -> cq.Workplane:
    """Thin annular ring representing the cartridge cap seam below the lever."""
    return (
        cq.Workplane("XY")
        .circle(SEAM_OUTER_RADIUS)
        .circle(SEAM_INNER_RADIUS)
        .extrude(SEAM_HEIGHT)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))
    model.material("seam_dark", rgba=(0.45, 0.46, 0.48, 1.0))

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
    # Raised circular collar around the base
    body.visual(
        mesh_from_cadquery(_build_collar(), "base_collar"),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT)),
        material="brushed_steel",
        name="base_collar",
    )
    # Thin cartridge cap seam ring below the lever
    body.visual(
        mesh_from_cadquery(_build_seam_ring(), "cartridge_seam"),
        origin=Origin(xyz=(0.0, 0.0, SEAM_Z_CENTER - SEAM_HEIGHT / 2.0)),
        material="seam_dark",
        name="cartridge_seam",
    )

    # ------------------------------------------------------------------ spout
    # The spout is a separate part that swivels about the vertical body axis.
    # Its root is nested inside the body column at SPOUT_ROOT_Z.
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="brushed_steel",
        name="spout_channel",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=1.5, lower=-SWIVEL_RANGE, upper=SWIVEL_RANGE
        ),
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

    # Subtle circumferential grip grooves along the lever bar
    groove_margin = 0.018
    groove_start_x = BAR_X_START + groove_margin
    groove_end_x = BAR_X_END - groove_margin
    groove_spacing = (groove_end_x - groove_start_x) / max(GROOVE_COUNT - 1, 1)
    for i in range(GROOVE_COUNT):
        gx = groove_start_x + i * groove_spacing
        handle.visual(
            Cylinder(radius=GROOVE_RADIUS, length=GROOVE_WIDTH),
            origin=Origin(
                xyz=(gx, BAR_OFFSET_Y, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
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
    spout = object_model.get_part("spout")
    boss = object_model.get_part("lever_boss")
    handle = object_model.get_part("lever_handle")
    swivel = object_model.get_articulation("spout_swivel")
    lift = object_model.get_articulation("boss_lift")
    twist = object_model.get_articulation("lever_twist")
    spout_vis = spout.get_visual("spout_channel")
    bar = handle.get_visual("lever_bar")
    disc = handle.get_visual("control_disc")
    hot_dot = handle.get_visual("hot_dot")
    collar = body.get_visual("base_collar")
    seam = body.get_visual("cartridge_seam")

    # Intentional seated embeddings of the rotating stack.
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_channel",
        elem_b="body_column",
        reason="spout root is nested inside the body column at the swivel bearing",
    )
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

    # --- raised circular collar -------------------------------------------
    collar_aabb = ctx.part_element_world_aabb(body, elem=collar)
    ctx.check(
        "collar_exists",
        collar_aabb is not None,
        "raised circular collar must be present around the base",
    )
    ctx.check(
        "collar_sits_above_flange",
        collar_aabb is not None and collar_aabb[0][2] >= FLANGE_HEIGHT - 1e-6,
        f"collar base should sit at or above flange top ({FLANGE_HEIGHT}), got {collar_aabb}",
    )
    ctx.check(
        "collar_wider_than_body",
        collar_aabb is not None
        and (collar_aabb[1][0] - collar_aabb[0][0]) / 2.0 > BODY_RADIUS + 0.005,
        f"collar outer radius should exceed body radius, got {collar_aabb}",
    )

    # --- cartridge cap seam ------------------------------------------------
    seam_aabb = ctx.part_element_world_aabb(body, elem=seam)
    ctx.check(
        "cartridge_seam_exists",
        seam_aabb is not None,
        "thin cartridge cap seam must be present below the lever",
    )
    ctx.check(
        "cartridge_seam_below_disc",
        seam_aabb is not None and seam_aabb[1][2] < DISC_CENTER_Z - 0.010,
        f"seam should sit below the disc center ({DISC_CENTER_Z}), got {seam_aabb}",
    )

    # --- spout form and swivel --------------------------------------------
    spout_aabb = ctx.part_element_world_aabb(spout, elem=spout_vis)
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

    # Swivel joint properties
    ctx.check(
        "swivel_axis_vertical",
        abs(swivel.axis[2]) == 1.0 and swivel.axis[0] == 0.0 and swivel.axis[1] == 0.0,
        f"spout swivel must rotate about the vertical Z axis, got {swivel.axis}",
    )
    ctx.check(
        "swivel_range_pm90deg",
        swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_RANGE) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_RANGE) < 1e-6,
        "spout swivel range must be -90..+90 deg",
    )

    # Prove spout actually swivels: at +90° the spout tip should be in +Y
    rest_spout_aabb = ctx.part_element_world_aabb(spout, elem=spout_vis)
    with ctx.pose({swivel: SWIVEL_RANGE}):
        swiveled_aabb = ctx.part_element_world_aabb(spout, elem=spout_vis)
        ctx.check(
            "spout_swivels_sideways",
            rest_spout_aabb is not None
            and swiveled_aabb is not None
            and swiveled_aabb[1][1] > 0.10
            and swiveled_aabb[1][0] < 0.05,
            f"at +90 deg spout should point in +Y: rest_x_max={rest_spout_aabb[1][0] if rest_spout_aabb else None}, "
            f"swiveled_y_max={swiveled_aabb[1][1] if swiveled_aabb else None}, "
            f"swiveled_x_max={swiveled_aabb[1][0] if swiveled_aabb else None}",
        )

    # --- grip grooves on lever bar ----------------------------------------
    groove_0 = handle.get_visual("grip_groove_0")
    groove_last = handle.get_visual(f"grip_groove_{GROOVE_COUNT - 1}")
    g0_aabb = ctx.part_element_world_aabb(handle, elem=groove_0)
    gl_aabb = ctx.part_element_world_aabb(handle, elem=groove_last)
    ctx.check(
        "grip_grooves_present",
        g0_aabb is not None and gl_aabb is not None,
        "subtle grip grooves must be present on the lever bar",
    )
    ctx.check(
        "grip_grooves_span_bar",
        g0_aabb is not None
        and gl_aabb is not None
        and (gl_aabb[1][0] - g0_aabb[0][0]) > 0.060,
        f"grooves should span most of the lever bar length, got first={g0_aabb}, last={gl_aabb}",
    )

    # --- disc and lever form -----------------------------------------------
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

    bar_aabb = ctx.part_element_world_aabb(handle, elem=bar)
    ctx.check(
        "lever_bar_length",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.130,
        f"lever bar should run ~0.136 m forward, got {bar_aabb}",
    )
    # The lever bar rides above the spout channel at rest.
    ctx.expect_gap(
        handle,
        spout,
        axis="z",
        min_gap=0.020,
        positive_elem=bar,
        negative_elem=spout_vis,
        name="lever_bar_above_spout",
    )
    # Boss shaft actually bridges body and disc hub.
    ctx.expect_contact(body, boss, name="boss_seats_on_body")
    ctx.expect_contact(boss, handle, name="disc_seats_on_boss")

    # --- joint plan (lift and twist) --------------------------------------
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
            spout,
            handle,
            axis="y",
            min_gap=0.010,
            positive_elem=spout_vis,
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
