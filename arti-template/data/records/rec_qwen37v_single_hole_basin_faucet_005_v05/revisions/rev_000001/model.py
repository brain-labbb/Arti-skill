from __future__ import annotations

"""Single-hole basin faucet variant: curved tubular spout with hinged aerator.

A vertical cylindrical body (~0.20 m tall, ~0.055 m diameter) on a slightly
wider round base flange. From the body a curved tubular spout sweeps forward
and gently downward, ending in a real hollow outlet. A separate circular
aerator insert sits at the spout mouth and flips open on a tiny hinge.

On the right side near the top, a round flat control disc (~0.055 m diameter,
with red/blue index dots) is mounted on a short horizontal boss; a slim
cylindrical lever bar runs forward from the disc.

Articulation chain:
- ``boss_lift``: revolute about the horizontal sideways (Y) boss axis,
  -40..+40 deg; positive q lifts the lever tip up (flow control).
- ``lever_twist``: revolute about the lever's own forward (X) axis through
  the disc center, -30..+30 deg (temperature mix).
- ``aerator_hinge``: revolute about the horizontal sideways (Y) axis at the
  top edge of the spout mouth, 0..90 deg; positive q flips the aerator open.
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

SPOUT_ROOT_Z = 0.140  # height where spout leaves the body
SPOUT_OUTER_R = 0.013  # outer radius of the tubular spout
SPOUT_WALL = 0.0025
SPOUT_INNER_R = SPOUT_OUTER_R - SPOUT_WALL

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
AERATOR_HINGE_RANGE = math.radians(90.0)

# Aerator disc dimensions
AERATOR_RADIUS = SPOUT_INNER_R - 0.0005  # slight clearance inside the tube
AERATOR_THICKNESS = 0.003

# Spout mouth position (end of the sweep path)
SPOUT_MOUTH_X = 0.130
SPOUT_MOUTH_Z = SPOUT_ROOT_Z - 0.040  # curves down 40mm


def _build_spout_tube() -> cq.Workplane:
    """Curved tubular spout swept along a gentle downward arc.

    Built in body-local coordinates: the tube centerline starts at the body
    front surface heading +X and curves gently downward. An annular cross-
    section is swept along the path to produce a real hollow tube with an
    open mouth at the far end.
    """
    # Define the spine path in the XZ plane (local frame, before origin offset)
    # The visual origin is at z=SPOUT_ROOT_Z, so local Z=0 is at that height.
    # Path starts inside the body and sweeps forward while curving downward.
    mouth_drop = SPOUT_MOUTH_Z - SPOUT_ROOT_Z  # -0.040 m
    path = cq.Workplane("XZ").spline(
        [
            (BODY_RADIUS * 0.6, 0.0),       # start slightly inside body
            (0.045, -0.003),
            (0.080, -0.012),
            (0.110, -0.025),
            (SPOUT_MOUTH_X, mouth_drop),
        ]
    )
    # Annular profile at the start of the path
    # The profile plane is YZ at the path start
    outer_r = SPOUT_OUTER_R
    inner_r = SPOUT_INNER_R
    profile = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_RADIUS * 0.6)
        .circle(outer_r)
        .circle(inner_r)
    )
    return profile.sweep(path, isFrenet=True)


def _build_aerator_disc() -> cq.Workplane:
    """Thin circular aerator insert that fits inside the spout mouth.

    Modeled in aerator-local coordinates with the disc centered at the
    origin, lying flat in XY (thin along Z). The hinge axis passes through
    the top edge (+Z local in the mounted frame).
    """
    disc = (
        cq.Workplane("XY")
        .circle(AERATOR_RADIUS)
        .extrude(AERATOR_THICKNESS)
    )
    # Add a small grid pattern (3 crossing slots) to read as an aerator
    slot_w = 0.0015
    slot_len = AERATOR_RADIUS * 1.6
    for angle in (0.0, 60.0, 120.0):
        rad = math.radians(angle)
        cx = 0.0
        cy = 0.0
        slot = (
            cq.Workplane("XY")
            .workplane(offset=-0.0001)
            .transformed(rotate=(0, 0, angle))
            .rect(slot_len, slot_w)
            .extrude(AERATOR_THICKNESS + 0.0002)
        )
        disc = disc.cut(slot)
    return disc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))
    model.material("aerator_mesh", rgba=(0.60, 0.62, 0.64, 1.0))

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
    # Flat top cap on the column
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.002)),
        material="brushed_steel",
        name="column_cap",
    )
    # Curved tubular spout
    body.visual(
        mesh_from_cadquery(_build_spout_tube(), "spout_tube"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_tube",
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

    # ------------------------------------------------- aerator on hinge
    # The aerator disc sits at the spout mouth. Its part frame origin is at
    # the hinge line (top edge of the mouth). The disc extends downward from
    # the hinge in the closed position.
    aerator = model.part("aerator")
    # Disc modeled hanging below the hinge axis: center is at (0, 0, -AERATOR_RADIUS)
    # in aerator-local frame, thickness along Y (across the spout).
    aerator.visual(
        mesh_from_cadquery(_build_aerator_disc(), "aerator_disc"),
        origin=Origin(
            xyz=(0.0, 0.0, -AERATOR_RADIUS),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="aerator_mesh",
        name="aerator_disc",
    )
    # Small hinge knuckle visual connecting aerator to spout
    aerator.visual(
        Cylinder(radius=0.002, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="bright_steel",
        name="hinge_knuckle",
    )

    # Hinge joint: the aerator rotates about the -Y axis at the top of the
    # spout mouth. Positive q swings the disc forward and outward (open).
    # The hinge origin is at the spout mouth top edge in world coords.
    hinge_z = SPOUT_ROOT_Z + (SPOUT_MOUTH_Z - SPOUT_ROOT_Z) + SPOUT_OUTER_R
    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(SPOUT_MOUTH_X, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=AERATOR_HINGE_RANGE
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
    hinge = object_model.get_articulation("aerator_hinge")
    spout = body.get_visual("spout_tube")
    disc = handle.get_visual("control_disc")
    bar = handle.get_visual("lever_bar")
    hot_dot = handle.get_visual("hot_dot")
    aero_disc = aerator.get_visual("aerator_disc")

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
        aerator,
        body,
        elem_a="hinge_knuckle",
        elem_b="spout_tube",
        reason="hinge knuckle is seated into the spout mouth rim",
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

    # Spout curves gently downward from the column
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.120,
        f"curved spout should reach at least 0.12 m forward, got {spout_aabb}",
    )
    ctx.check(
        "spout_curves_downward",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_ROOT_Z - 0.020,
        f"spout mouth should curve below the root line, got {spout_aabb}",
    )
    ctx.check(
        "spout_tubular_not_flat",
        spout_aabb is not None
        and (spout_aabb[1][2] - spout_aabb[0][2]) > 0.015,
        f"spout should have vertical extent from curvature, got {spout_aabb}",
    )

    # Disc and lever checks (inherited from parent)
    disc_aabb = ctx.part_element_world_aabb(handle, elem=disc)
    ctx.check(
        "disc_outboard_of_body",
        disc_aabb is not None and disc_aabb[1][1] < -BODY_RADIUS - 0.010,
        f"control disc must sit clear of the body side, got {disc_aabb}",
    )
    bar_aabb = ctx.part_element_world_aabb(handle, elem=bar)
    ctx.check(
        "lever_bar_length",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.130,
        f"lever bar should run ~0.136 m forward, got {bar_aabb}",
    )

    # Boss seats properly
    ctx.expect_contact(body, boss, name="boss_seats_on_body")
    ctx.expect_contact(boss, handle, name="disc_seats_on_boss")

    # --- aerator mechanism -------------------------------------------------
    # Aerator exists as a separate part at the spout mouth
    aero_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator_at_spout_mouth",
        aero_aabb is not None
        and aero_aabb[1][0] > 0.100
        and aero_aabb[0][0] < SPOUT_MOUTH_X + 0.020,
        f"aerator should be near the spout mouth, got {aero_aabb}",
    )
    # Aerator disc is within the spout mouth area (XY containment)
    ctx.expect_within(
        aerator,
        body,
        axes="xy",
        margin=0.015,
        inner_elem=aero_disc,
        outer_elem=spout,
        name="aerator_disc_within_spout_mouth_xy",
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
    # Aerator hinge: -Y axis, 0 to 90 deg
    ctx.check(
        "aerator_hinge_axis_sideways",
        hinge.axis[1] == -1.0 and hinge.axis[0] == 0.0 and hinge.axis[2] == 0.0,
        f"aerator hinge must rotate about horizontal -Y axis, got {hinge.axis}",
    )
    ctx.check(
        "aerator_hinge_range",
        hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower) < 1e-6
        and abs(hinge.motion_limits.upper - AERATOR_HINGE_RANGE) < 1e-6,
        "aerator hinge range must be 0..90 deg",
    )

    # --- motion proof: lever -----------------------------------------------
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > 0.245,
            f"at +40 deg the bar tip should rise to ~0.26 m, got {up_aabb}",
        )

    # --- motion proof: aerator hinge flips open ----------------------------
    rest_aero = ctx.part_element_world_aabb(aerator, elem=aero_disc)
    with ctx.pose({hinge: AERATOR_HINGE_RANGE}):
        open_aero = ctx.part_element_world_aabb(aerator, elem=aero_disc)
        ctx.check(
            "aerator_flips_forward",
            rest_aero is not None
            and open_aero is not None
            and (open_aero[0][0] + open_aero[1][0]) / 2.0
            > (rest_aero[0][0] + rest_aero[1][0]) / 2.0 + 0.005,
            f"aerator disc center should swing forward when opened: rest={rest_aero}, open={open_aero}",
        )
        # When open, the disc rises from its closed hanging position
        ctx.check(
            "aerator_rises_when_open",
            open_aero is not None
            and rest_aero is not None
            and open_aero[0][2] > rest_aero[0][2] + 0.005,
            f"opened aerator bottom should rise above closed position: rest={rest_aero}, open={open_aero}",
        )

    return ctx.report()


object_model = build_object_model()
