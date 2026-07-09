from __future__ import annotations

"""Single-hole basin faucet — tall tower variant.

A taller straight cylindrical tower body (~0.26 m tall, ~0.055 m diameter) on
a slightly wider round base flange. A short forward-projecting tube spout
(~0.08 m) exits the front of the body near the top, terminating in a real
hollow circular outlet. On the side near the top, a round flat control disc
(~0.055 m diameter, with red/blue index dots at top/bottom) is carried on a
short horizontal boss; a slim cylindrical lever bar runs forward from the
disc.

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
BODY_HEIGHT = 0.260  # taller tower
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

# Short forward spout tube (along +X)
SPOUT_ROOT_Z = 0.200  # where spout exits the body (near top of tower)
SPOUT_LENGTH = 0.080  # forward projection
SPOUT_OUTER_R = 0.014  # outer radius of spout tube
SPOUT_INNER_R = 0.010  # inner bore radius (hollow outlet)
SPOUT_DROOP = 0.012  # slight downward offset over the length

DISC_RADIUS = 0.0275  # 0.055 m diameter control disc
DISC_THICKNESS = 0.012
DISC_CENTER_Y = -0.048  # disc mid-plane (outboard of the body surface)
DISC_CENTER_Z = 0.225  # near top of taller tower

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


def _build_spout_with_hollow_outlet() -> tuple[cq.Workplane, cq.Workplane]:
    """Short forward-projecting spout tube with a real hollow outlet.

    Returns (spout_body, outlet_ring) as two CadQuery solids.
    The tube runs along +X from x=0 (buried inside body column) to x=SPOUT_LENGTH.
    The outlet ring is a short wider annular rim at the spout tip.
    """
    # Outer tube along +X using YZ workplane (normal = +X)
    outer = (
        cq.Workplane("YZ")
        .circle(SPOUT_OUTER_R)
        .extrude(SPOUT_LENGTH)
    )
    outer = outer.translate((0, 0, -SPOUT_DROOP / 2.0))

    # Through-bore (slightly longer to ensure clean cut)
    bore = (
        cq.Workplane("YZ")
        .circle(SPOUT_INNER_R)
        .extrude(SPOUT_LENGTH + 0.010)
    )
    bore = bore.translate((-0.005, 0, -SPOUT_DROOP / 2.0))

    spout_tube = outer.cut(bore)

    # Outlet rim ring at the spout tip — wider than the tube body
    rim_start = SPOUT_LENGTH - 0.002
    rim_length = 0.007
    outlet_outer = (
        cq.Workplane("YZ")
        .workplane(offset=rim_start)
        .circle(SPOUT_OUTER_R + 0.003)
        .extrude(rim_length)
    )
    outlet_outer = outlet_outer.translate((0, 0, -SPOUT_DROOP))

    outlet_bore = (
        cq.Workplane("YZ")
        .workplane(offset=rim_start - 0.001)
        .circle(SPOUT_INNER_R)
        .extrude(rim_length + 0.004)
    )
    outlet_bore = outlet_bore.translate((0, 0, -SPOUT_DROOP))

    outlet_ring = outlet_outer.cut(outlet_bore)

    return spout_tube, outlet_ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_tower")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))
    model.material("dark_bore", rgba=(0.12, 0.12, 0.14, 1.0))

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

    # Spout with hollow outlet — origin at body center axis so tube root
    # is well inside the body column for geometric connectivity.
    spout_tube, outlet_ring = _build_spout_with_hollow_outlet()
    body.visual(
        mesh_from_cadquery(spout_tube, "spout_tube"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_tube",
    )
    body.visual(
        mesh_from_cadquery(outlet_ring, "outlet_ring"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="bright_steel",
        name="outlet_ring",
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    boss = object_model.get_part("lever_boss")
    handle = object_model.get_part("lever_handle")
    lift = object_model.get_articulation("boss_lift")
    twist = object_model.get_articulation("lever_twist")
    spout_tube = body.get_visual("spout_tube")
    outlet_ring = body.get_visual("outlet_ring")
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
    # Taller tower: ~0.26 m
    ctx.check(
        "body_height_tower",
        aabb is not None and 0.255 < aabb[1][2] < 0.275,
        f"tower body should be ~0.26 m tall, got {aabb}",
    )

    # Spout is short (~0.08 m forward projection from body center)
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout_tube)
    ctx.check(
        "spout_short_forward",
        spout_aabb is not None and spout_aabb[1][0] < 0.100,
        f"short spout should not reach past 0.10 m forward, got {spout_aabb}",
    )
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.055,
        f"spout must project at least 0.055 m forward, got {spout_aabb}",
    )

    # Hollow outlet ring exists at the spout tip and shows a visible rim
    outlet_aabb = ctx.part_element_world_aabb(body, elem=outlet_ring)
    ctx.check(
        "outlet_ring_at_spout_tip",
        outlet_aabb is not None and outlet_aabb[1][0] > 0.065,
        f"outlet ring should be near spout tip (x>0.065), got {outlet_aabb}",
    )
    # Outlet ring Z extent should reflect the wider rim diameter (2*(0.014+0.003)=0.034)
    ctx.check(
        "outlet_ring_visible_rim",
        outlet_aabb is not None
        and (outlet_aabb[1][2] - outlet_aabb[0][2]) > 0.025,
        f"outlet ring rim should span >0.025 m in Z, got {outlet_aabb}",
    )

    # Disc still outboard of body
    disc_aabb = ctx.part_element_world_aabb(handle, elem=disc)
    ctx.check(
        "disc_outboard_of_body",
        disc_aabb is not None and disc_aabb[1][1] < -BODY_RADIUS - 0.010,
        f"control disc must sit clear of the body side, got {disc_aabb}",
    )

    # Lever bar length
    bar_aabb = ctx.part_element_world_aabb(handle, elem=bar)
    ctx.check(
        "lever_bar_length",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.130,
        f"lever bar should run ~0.136 m forward, got {bar_aabb}",
    )

    # Boss shaft bridges body and disc hub.
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
    # Lift up: the forward lever tip sweeps upward.
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > DISC_CENTER_Z + 0.05,
            f"at +40 deg the bar tip should rise well above disc center, got {up_aabb}",
        )
    # Lift down: tip sweeps below disc level.
    with ctx.pose({lift: -LIFT_RANGE}):
        down_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_down_lowers_lever_tip",
            down_aabb is not None and down_aabb[0][2] < DISC_CENTER_Z - 0.05,
            f"at -40 deg the bar tip should drop below disc center, got {down_aabb}",
        )

    # Twist: the off-axis paint dot orbits the lever's forward axis.
    rest_dot = ctx.part_element_world_aabb(handle, elem=hot_dot)
    with ctx.pose({twist: TWIST_RANGE}):
        twist_dot = ctx.part_element_world_aabb(handle, elem=hot_dot)
        ctx.check(
            "twist_swings_index_dot",
            rest_dot is not None
            and twist_dot is not None
            and abs(rest_dot[0][1] - twist_dot[0][1]) > 0.005,
            f"hot dot should swing at +30 deg: rest={rest_dot}, twisted={twist_dot}",
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
