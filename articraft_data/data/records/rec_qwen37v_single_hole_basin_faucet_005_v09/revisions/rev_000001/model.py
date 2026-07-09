from __future__ import annotations

"""Single-hole basin faucet variant with detachable-collar spout and drain rod.

A vertical cylindrical body (~0.20 m tall, ~0.055 m diameter) on a slightly
wider round base flange. A channel spout projects forward and slightly down
~0.13 m from the front of the body, with a collar seam at the root suggesting
a detachable connection, and a real hollow outlet bore at the tip. On the
right side near the top, a round flat control disc (~0.055 m diameter, with
red/blue index dots) is carried on a short horizontal boss; a slim cylindrical
lever bar runs forward from the disc. Behind the body, a slim pull-up drain
rod slides vertically on a prismatic joint.

Articulation chain:
- ``boss_lift``: revolute about the horizontal sideways (Y) boss axis,
  -40..+40 deg; positive q lifts the lever tip up (flow control).
- ``lever_twist``: revolute about the lever's own forward (X) axis through
  the disc center, -30..+30 deg (temperature mix).
- ``drain_rod_slide``: prismatic along +Z, 0..0.04 m travel.
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

# Collar seam at spout root (detachable appearance)
COLLAR_RADIUS = 0.022
COLLAR_THICKNESS = 0.008
COLLAR_RING_WIDTH = 0.004

# Hollow outlet at spout mouth
OUTLET_BORE_RADIUS = 0.009
OUTLET_BORE_DEPTH = 0.012

# Drain rod
DRAIN_ROD_RADIUS = 0.003
DRAIN_ROD_LENGTH = 0.075
DRAIN_ROD_KNOB_RADIUS = 0.006
DRAIN_ROD_KNOB_HEIGHT = 0.008
DRAIN_ROD_X = -0.030  # behind the body (-X)
DRAIN_ROD_REST_Z = 0.165  # top of rod shaft at rest (knob visible above spout root)
DRAIN_ROD_TRAVEL = 0.040

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


def _build_spout() -> cq.Workplane:
    """Open-top U-channel swept forward and curving down at the tip.

    Built in spout-local coordinates: the channel centerline starts at the
    origin heading +X; the visual is placed at the body front at
    ``SPOUT_ROOT_Z``.
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


def _build_spout_collar() -> cq.Workplane:
    """Collar ring at the spout root, suggesting a detachable connection.

    Built as a torus-like ring: a short hollow cylinder with slightly larger
    outer radius than the spout body, centered at the spout root.
    """
    outer_r = COLLAR_RADIUS
    inner_r = outer_r - COLLAR_RING_WIDTH
    # Build a short tube (ring)
    ring = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(COLLAR_THICKNESS)
    )
    return ring


def _build_outlet_tip() -> cq.Workplane:
    """Closed tube section at the spout mouth with a visible bore opening.

    A short rectangular tube (outer matches spout cross-section, inner bore)
    placed at the spout tip. The bore reads as a dark hollow outlet.
    """
    hw = SPOUT_OUTER_W / 2.0
    hh = SPOUT_OUTER_H / 2.0
    # Outer rectangular shell
    outer = (
        cq.Workplane("YZ")
        .rect(SPOUT_OUTER_W, SPOUT_OUTER_H)
        .extrude(OUTLET_BORE_DEPTH)
    )
    # Inner bore cutout (circular)
    bore = (
        cq.Workplane("YZ")
        .circle(OUTLET_BORE_RADIUS)
        .extrude(OUTLET_BORE_DEPTH + 0.001)
    )
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))
    model.material("dark_bore", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("chrome_cap", rgba=(0.85, 0.86, 0.88, 1.0))

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

    # Spout collar seam at root (detachable appearance)
    body.visual(
        mesh_from_cadquery(_build_spout_collar(), "spout_collar"),
        origin=Origin(
            xyz=(0.0, 0.0, SPOUT_ROOT_Z - COLLAR_THICKNESS / 2.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bright_steel",
        name="spout_collar",
    )

    # Hollow outlet bore at spout tip - dark recessed cylinder to read as opening
    # The spout tip ends at roughly (0.155, 0, SPOUT_ROOT_Z - 0.048)
    tip_x = 0.148
    tip_z = SPOUT_ROOT_Z - 0.044
    body.visual(
        Cylinder(radius=OUTLET_BORE_RADIUS, length=OUTLET_BORE_DEPTH),
        origin=Origin(
            xyz=(tip_x + OUTLET_BORE_DEPTH / 2.0, 0.0, tip_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="dark_bore",
        name="outlet_bore",
    )

    # Outlet bezel ring around the bore opening
    body.visual(
        mesh_from_cadquery(_build_outlet_tip(), "outlet_bezel"),
        origin=Origin(
            xyz=(tip_x, 0.0, tip_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bright_steel",
        name="outlet_bezel",
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

    # ------------------------------------------------- drain rod (prismatic)
    drain_rod = model.part("drain_rod")
    # Main rod shaft
    drain_rod.visual(
        Cylinder(radius=DRAIN_ROD_RADIUS, length=DRAIN_ROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LENGTH / 2.0)),
        material="brushed_steel",
        name="rod_shaft",
    )
    # Pull knob at top
    drain_rod.visual(
        Cylinder(radius=DRAIN_ROD_KNOB_RADIUS, length=DRAIN_ROD_KNOB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LENGTH + DRAIN_ROD_KNOB_HEIGHT / 2.0)),
        material="chrome_cap",
        name="rod_knob",
    )

    model.articulation(
        "drain_rod_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain_rod,
        origin=Origin(xyz=(DRAIN_ROD_X, 0.0, DRAIN_ROD_REST_Z - DRAIN_ROD_LENGTH)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=0.5, lower=0.0, upper=DRAIN_ROD_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    boss = object_model.get_part("lever_boss")
    handle = object_model.get_part("lever_handle")
    drain_rod = object_model.get_part("drain_rod")
    lift = object_model.get_articulation("boss_lift")
    twist = object_model.get_articulation("lever_twist")
    slide = object_model.get_articulation("drain_rod_slide")
    spout = body.get_visual("spout_channel")
    bar = handle.get_visual("lever_bar")
    disc = handle.get_visual("control_disc")
    hot_dot = handle.get_visual("hot_dot")
    collar = body.get_visual("spout_collar")
    outlet_bore = body.get_visual("outlet_bore")

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
    ctx.allow_overlap(
        drain_rod,
        body,
        elem_a="rod_shaft",
        elem_b="body_column",
        reason="drain rod shaft passes through a guide hole in the body rear",
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

    bar_aabb = ctx.part_element_world_aabb(handle, elem=bar)
    ctx.check(
        "lever_bar_length",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.130,
        f"lever bar should run ~0.136 m forward, got {bar_aabb}",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="z",
        min_gap=0.020,
        positive_elem=bar,
        negative_elem=spout,
        name="lever_bar_above_spout",
    )
    ctx.expect_contact(body, boss, name="boss_seats_on_body")
    ctx.expect_contact(boss, handle, name="disc_seats_on_boss")

    # --- collar seam at spout root -----------------------------------------
    collar_aabb = ctx.part_element_world_aabb(body, elem=collar)
    ctx.check(
        "collar_at_spout_root",
        collar_aabb is not None
        and abs((collar_aabb[0][2] + collar_aabb[1][2]) / 2.0 - SPOUT_ROOT_Z) < 0.010,
        f"collar should be centered at spout root height, got {collar_aabb}",
    )
    ctx.check(
        "collar_wider_than_spout",
        collar_aabb is not None
        and (collar_aabb[1][1] - collar_aabb[0][1]) > SPOUT_OUTER_W * 0.9,
        f"collar ring should encircle the spout root, got {collar_aabb}",
    )

    # --- hollow outlet at spout mouth --------------------------------------
    bore_aabb = ctx.part_element_world_aabb(body, elem=outlet_bore)
    ctx.check(
        "outlet_bore_at_spout_tip",
        bore_aabb is not None and bore_aabb[1][0] > 0.140,
        f"outlet bore should be at the spout tip (x>0.14), got {bore_aabb}",
    )
    ctx.check(
        "outlet_bore_size",
        bore_aabb is not None
        and abs((bore_aabb[1][2] - bore_aabb[0][2]) - 2.0 * OUTLET_BORE_RADIUS) < 0.004,
        f"outlet bore diameter should be ~{2*OUTLET_BORE_RADIUS:.3f}m, got {bore_aabb}",
    )

    # --- drain rod geometry and joint --------------------------------------
    rod_shaft = drain_rod.get_visual("rod_shaft")
    rod_knob = drain_rod.get_visual("rod_knob")
    rod_aabb = ctx.part_world_aabb(drain_rod)
    ctx.check(
        "drain_rod_behind_body",
        rod_aabb is not None and rod_aabb[0][0] < -BODY_RADIUS,
        f"drain rod should be behind body (-X), got {rod_aabb}",
    )
    ctx.check(
        "drain_rod_vertical_extent",
        rod_aabb is not None and (rod_aabb[1][2] - rod_aabb[0][2]) > 0.060,
        f"drain rod should be at least 0.06 m tall, got {rod_aabb}",
    )

    # Prismatic joint checks
    ctx.check(
        "slide_axis_vertical",
        abs(slide.axis[2]) == 1.0 and slide.axis[0] == 0.0 and slide.axis[1] == 0.0,
        f"drain rod slide must be along Z, got {slide.axis}",
    )
    ctx.check(
        "slide_range_0_to_40mm",
        slide.motion_limits is not None
        and abs(slide.motion_limits.lower) < 1e-6
        and abs(slide.motion_limits.upper - DRAIN_ROD_TRAVEL) < 1e-6,
        "drain rod slide range must be 0..0.04 m",
    )

    # Motion proof: rod rises when pulled up
    rest_rod_pos = ctx.part_world_position(drain_rod)
    with ctx.pose({slide: DRAIN_ROD_TRAVEL}):
        raised_rod_pos = ctx.part_world_position(drain_rod)
        ctx.check(
            "drain_rod_rises_on_positive_slide",
            rest_rod_pos is not None
            and raised_rod_pos is not None
            and raised_rod_pos[2] > rest_rod_pos[2] + 0.030,
            f"rod should rise ~0.04 m: rest={rest_rod_pos}, raised={raised_rod_pos}",
        )
        # Knob should be above body top when fully raised
        knob_aabb = ctx.part_element_world_aabb(drain_rod, elem=rod_knob)
        ctx.check(
            "raised_knob_above_body_top",
            knob_aabb is not None and knob_aabb[0][2] > BODY_HEIGHT - 0.01,
            f"raised knob should be near body top, got {knob_aabb}",
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

    # --- motion proof (lever) ----------------------------------------------
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > 0.245,
            f"at +40 deg the bar tip should rise to ~0.26 m, got {up_aabb}",
        )
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
