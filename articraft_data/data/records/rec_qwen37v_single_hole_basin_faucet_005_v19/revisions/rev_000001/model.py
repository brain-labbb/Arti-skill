from __future__ import annotations

"""Single-hole basin faucet variant with detachable spout collar, flip-up aerator,
hollow outlet, and oval base gasket.

Based on a brushed stainless single-lever basin faucet (~0.20 m tall, ~0.055 m
body diameter). The variant adds:
- An oval rubber base gasket ring
- A detachable-looking spout collar seam (chrome ring at body junction)
- A real hollow outlet tube at the spout mouth
- A flip-up aerator disc on a tiny revolute hinge

Articulation chain:
- ``boss_lift``: revolute about Y (flow control), -40..+40 deg
- ``lever_twist``: revolute about X (temperature mix), -30..+30 deg
- ``aerator_hinge``: revolute about Y at spout tip, 0..80 deg (aerator flip)
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

# Oval gasket
GASKET_OUTER_X = 0.040  # semi-major (forward-back)
GASKET_OUTER_Y = 0.036  # semi-minor (side-side)
GASKET_INNER_R = FLANGE_RADIUS - 0.001  # fits around flange
GASKET_THICK = 0.004

SPOUT_ROOT_Z = 0.118
SPOUT_OUTER_W = 0.034
SPOUT_OUTER_H = 0.022
SPOUT_WALL = 0.005
SPOUT_FLOOR = 0.006

# Spout collar seam
COLLAR_LENGTH = 0.012
COLLAR_OUTER_R = 0.020
COLLAR_INNER_R = 0.016

# Hollow outlet tube at spout tip
OUTLET_RADIUS = 0.012
OUTLET_BORE_RADIUS = 0.009
OUTLET_LENGTH = 0.018
# Position of outlet center (approximate spout tip in spout-local coords)
OUTLET_TIP_X = 0.155
OUTLET_TIP_Z_OFFSET = -0.048  # relative to SPOUT_ROOT_Z

DISC_RADIUS = 0.0275
DISC_THICKNESS = 0.012
DISC_CENTER_Y = -0.048
DISC_CENTER_Z = 0.163

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

LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)
AERATOR_RANGE = math.radians(80.0)

# Aerator disc
AERATOR_RADIUS = 0.011
AERATOR_THICK = 0.003


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


def _build_spout_collar() -> cq.Workplane:
    """Thin chrome ring at the spout-body junction (detachable-look seam).
    Built centered at origin along Z with symmetric extent."""
    half = COLLAR_LENGTH / 2.0
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -half))
        .circle(COLLAR_OUTER_R)
        .extrude(COLLAR_LENGTH)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -half))
        .circle(COLLAR_INNER_R)
        .extrude(COLLAR_LENGTH)
    )
    collar = outer.cut(inner)
    # Seam ring at mid-length for the detachable look
    seam = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_R + 0.0015)
        .circle(COLLAR_OUTER_R - 0.001)
        .extrude(0.002)
    )
    return collar.union(seam)


def _build_outlet_tube() -> cq.Workplane:
    """Short hollow cylinder with real bore at the spout mouth."""
    outer = cq.Workplane("XY").circle(OUTLET_RADIUS).extrude(OUTLET_LENGTH)
    bore = cq.Workplane("XY").circle(OUTLET_BORE_RADIUS).extrude(OUTLET_LENGTH)
    tube = outer.cut(bore)
    # Add a small lip ring at the open end
    lip = (
        cq.Workplane("XY")
        .circle(OUTLET_RADIUS + 0.002)
        .circle(OUTLET_RADIUS - 0.001)
        .extrude(0.003)
    )
    return tube.union(lip)


def _build_aerator_disc() -> cq.Workplane:
    """Small aerator disc with concentric ring pattern (mesh screen look)."""
    base = cq.Workplane("XY").circle(AERATOR_RADIUS).extrude(AERATOR_THICK)
    # Add a small rim
    rim = (
        cq.Workplane("XY")
        .circle(AERATOR_RADIUS + 0.001)
        .circle(AERATOR_RADIUS - 0.001)
        .extrude(AERATOR_THICK + 0.001)
    )
    # Concentric rings for screen pattern
    inner_ring = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, AERATOR_THICK * 0.5))
        .circle(AERATOR_RADIUS * 0.6)
        .circle(AERATOR_RADIUS * 0.6 - 0.001)
        .extrude(0.001)
    )
    center_dot = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, AERATOR_THICK))
        .circle(0.002)
        .extrude(0.001)
    )
    return base.union(rim).union(inner_ring).union(center_dot)


def _build_oval_gasket() -> cq.Workplane:
    """Oval rubber gasket ring at the faucet base."""
    outer = (
        cq.Workplane("XY")
        .ellipse(GASKET_OUTER_X, GASKET_OUTER_Y)
        .extrude(GASKET_THICK)
    )
    inner = (
        cq.Workplane("XY")
        .circle(GASKET_INNER_R)
        .extrude(GASKET_THICK)
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))
    model.material("gasket_rubber", rgba=(0.12, 0.12, 0.12, 1.0))
    model.material("chrome_accent", rgba=(0.88, 0.88, 0.90, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    # Oval base gasket (under the flange)
    body.visual(
        mesh_from_cadquery(_build_oval_gasket(), "oval_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="gasket_rubber",
        name="base_gasket",
    )
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, GASKET_THICK + FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )
    column_len = BODY_HEIGHT - 0.010
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, GASKET_THICK + 0.010 + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    body.visual(
        mesh_from_cadquery(_build_spout(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )
    # Spout collar seam (at the junction where spout meets body)
    # Orient along X (spout direction) using Y rotation, centered at body front
    body.visual(
        mesh_from_cadquery(_build_spout_collar(), "spout_collar"),
        origin=Origin(
            xyz=(BODY_RADIUS, 0.0, SPOUT_ROOT_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="chrome_accent",
        name="spout_collar",
    )
    # Hollow outlet tube at spout tip
    # The outlet hangs downward from the spout tip, overlapping into the
    # channel bottom wall so it reads as one piece with the spout
    outlet_world_z = SPOUT_ROOT_Z + OUTLET_TIP_Z_OFFSET
    outlet_origin_z = outlet_world_z - OUTLET_LENGTH / 2.0 + 0.006
    body.visual(
        mesh_from_cadquery(_build_outlet_tube(), "outlet_tube"),
        origin=Origin(
            xyz=(OUTLET_TIP_X, 0.0, outlet_origin_z),
            rpy=(math.pi, 0.0, 0.0),  # flip so bore faces down
        ),
        material="bright_steel",
        name="outlet_tube",
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
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator_disc(), "aerator_disc"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="chrome_accent",
        name="aerator_disc",
    )

    # The aerator hinge origin is at the bottom edge of the outlet tube opening.
    # The hinge axis is Y (sideways) so positive q flips the aerator downward.
    hinge_x = OUTLET_TIP_X
    hinge_z = outlet_origin_z - OUTLET_LENGTH  # bottom of outlet tube
    hinge_y = 0.0

    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(hinge_x, hinge_y + AERATOR_RADIUS, hinge_z)),
        # Y axis so positive q rotates the disc downward (flips open)
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=AERATOR_RANGE
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
    aerator_hinge = object_model.get_articulation("aerator_hinge")
    spout = body.get_visual("spout_channel")
    bar = handle.get_visual("lever_bar")
    disc = handle.get_visual("control_disc")
    hot_dot = handle.get_visual("hot_dot")
    gasket = body.get_visual("base_gasket")
    collar = body.get_visual("spout_collar")
    outlet = body.get_visual("outlet_tube")
    aerator_disc = aerator.get_visual("aerator_disc")

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
    # Oval base gasket exists and sits below the flange
    gasket_aabb = ctx.part_element_world_aabb(body, elem=gasket)
    ctx.check(
        "gasket_exists_at_base",
        gasket_aabb is not None and gasket_aabb[0][2] >= -1e-6,
        f"oval gasket should sit at z>=0, got {gasket_aabb}",
    )
    ctx.check(
        "gasket_is_oval",
        gasket_aabb is not None
        and abs((gasket_aabb[1][0] - gasket_aabb[0][0])
              - (gasket_aabb[1][1] - gasket_aabb[0][1])) > 0.003,
        f"gasket should be visibly oval (x extent != y extent), got {gasket_aabb}",
    )

    # Spout collar seam exists between body and spout
    collar_aabb = ctx.part_element_world_aabb(body, elem=collar)
    ctx.check(
        "collar_seam_exists",
        collar_aabb is not None,
        "spout collar seam must exist",
    )
    ctx.check(
        "collar_at_spout_junction",
        collar_aabb is not None
        and collar_aabb[0][0] > 0.0
        and collar_aabb[1][0] < BODY_RADIUS + 0.025,
        f"collar should be near the spout-body junction, got {collar_aabb}",
    )

    # Hollow outlet tube exists at spout mouth
    outlet_aabb = ctx.part_element_world_aabb(body, elem=outlet)
    ctx.check(
        "outlet_tube_exists",
        outlet_aabb is not None,
        "hollow outlet tube must exist at spout mouth",
    )
    ctx.check(
        "outlet_below_spout_root",
        outlet_aabb is not None and outlet_aabb[1][2] < SPOUT_ROOT_Z,
        f"outlet tube should hang below spout root z={SPOUT_ROOT_Z}, got {outlet_aabb}",
    )

    # Aerator part exists
    aerator_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_disc)
    ctx.check(
        "aerator_exists",
        aerator_aabb is not None,
        "aerator disc must exist at the spout outlet",
    )

    # --- aerator hinge checks ----------------------------------------------
    ctx.check(
        "aerator_hinge_is_revolute",
        aerator_hinge.articulation_type == ArticulationType.REVOLUTE,
        f"aerator hinge must be revolute, got {aerator_hinge.articulation_type}",
    )
    ctx.check(
        "aerator_hinge_sideways_axis",
        abs(aerator_hinge.axis[1]) == 1.0
        and aerator_hinge.axis[0] == 0.0
        and aerator_hinge.axis[2] == 0.0,
        f"aerator hinge axis must be sideways (Y), got {aerator_hinge.axis}",
    )
    ctx.check(
        "aerator_hinge_range",
        aerator_hinge.motion_limits is not None
        and aerator_hinge.motion_limits.lower is not None
        and aerator_hinge.motion_limits.upper is not None
        and aerator_hinge.motion_limits.lower >= -0.01
        and aerator_hinge.motion_limits.upper > math.radians(40),
        "aerator hinge should have a meaningful flip range",
    )

    # Aerator flip proof: open pose moves the aerator disc away from closed position
    rest_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_disc)
    with ctx.pose({aerator_hinge: AERATOR_RANGE}):
        open_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_disc)
        ctx.check(
            "aerator_flips_open",
            rest_aabb is not None
            and open_aabb is not None
            and (abs(open_aabb[0][0] - rest_aabb[0][0]) > 0.003
                 or abs(open_aabb[1][2] - rest_aabb[1][2]) > 0.003),
            f"aerator should visibly move when hinge opens: rest={rest_aabb}, open={open_aabb}",
        )

    # --- static form -------------------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and abs(aabb[0][2]) < 1e-4,
        f"base must sit on z~=0, got {aabb}",
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

    return ctx.report()


object_model = build_object_model()
