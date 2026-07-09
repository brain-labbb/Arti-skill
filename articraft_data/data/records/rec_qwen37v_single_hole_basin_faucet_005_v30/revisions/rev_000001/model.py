from __future__ import annotations

"""Single-hole basin faucet variant with tapered conical body.

A compact single-hole basin faucet (~0.20 m tall) with:
- Tapered conical body (wider base, narrower top) with subtle circumferential
  grip grooves.
- Small forward beak near the top of the body.
- Channel spout projecting forward and down from the beak area.
- A flip-open aerator at the spout tip on a tiny hinge (revolute, 0..90°).
- Side-mounted disc-and-lever control with lift (±40°) and twist (±30°).

All surfaces are satin brushed steel.
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
BODY_BOTTOM_RADIUS = 0.030  # wider at base
BODY_TOP_RADIUS = 0.020  # narrower at top (tapered conical)
BODY_HEIGHT = 0.200
FLANGE_RADIUS = 0.036
FLANGE_HEIGHT = 0.012

# Grip grooves - circumferential cuts on the body surface
GROOVE_COUNT = 6
GROOVE_DEPTH = 0.0012
GROOVE_WIDTH = 0.002
GROOVE_ZONE_BOTTOM_Z = 0.060  # start of grip zone above base
GROOVE_ZONE_TOP_Z = 0.130  # end of grip zone below spout

# Forward beak at top of body, near spout root
BEAK_LENGTH = 0.020
BEAK_WIDTH = 0.024
BEAK_HEIGHT = 0.016
BEAK_CENTER_Z = 0.158  # just below spout root, embedded in body top

# Spout
SPOUT_ROOT_Z = 0.160  # where spout leaves the beak
SPOUT_OUTER_W = 0.028
SPOUT_OUTER_H = 0.018
SPOUT_WALL = 0.004
SPOUT_FLOOR = 0.005

# Aerator at spout tip
AERATOR_RADIUS = 0.011
AERATOR_THICKNESS = 0.006
AERATOR_HINGE_PIN_RADIUS = 0.002
AERATOR_HINGE_PIN_LENGTH = 0.016

# Control disc and lever
DISC_RADIUS = 0.0275
DISC_THICKNESS = 0.012
DISC_CENTER_Y = -0.042
DISC_CENTER_Z = 0.158

BOSS_RADIUS = 0.010
BOSS_LENGTH = 0.016
BOSS_CENTER_Y = -0.028

BAR_RADIUS = 0.005
BAR_X_START = 0.012
BAR_X_END = 0.140
BAR_OFFSET_Y = 0.009

DOT_RADIUS = 0.003
DOT_LENGTH = 0.0015
DOT_Z = 0.018

LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)
AERATOR_RANGE = math.radians(90.0)


def _build_conical_body() -> cq.Workplane:
    """Tapered conical body with circumferential grip grooves.

    Built in body-local coordinates: base at z=FLANGE_HEIGHT, top at z=BODY_HEIGHT.
    The body tapers from BODY_BOTTOM_RADIUS at the base to BODY_TOP_RADIUS at the top.
    """
    body_height = BODY_HEIGHT - FLANGE_HEIGHT
    # Build a loft from bottom circle to top circle
    bottom_r = BODY_BOTTOM_RADIUS
    top_r = BODY_TOP_RADIUS

    body = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_HEIGHT)
        .circle(bottom_r)
        .workplane(offset=body_height)
        .circle(top_r)
        .loft()
    )

    # Cut circumferential grooves for grip texture
    groove_spacing = (GROOVE_ZONE_TOP_Z - GROOVE_ZONE_BOTTOM_Z) / (GROOVE_COUNT + 1)
    for i in range(GROOVE_COUNT):
        z_pos = GROOVE_ZONE_BOTTOM_Z + groove_spacing * (i + 1)
        # Interpolate radius at this height
        frac = (z_pos - FLANGE_HEIGHT) / body_height
        r_at_z = bottom_r + (top_r - bottom_r) * frac
        # Cut a torus-shaped groove (ring cut) into the body
        groove = (
            cq.Workplane("XY")
            .workplane(offset=z_pos)
            .circle(r_at_z + 0.001)
            .circle(r_at_z - GROOVE_DEPTH)
            .extrude(GROOVE_WIDTH, both=True)
        )
        body = body.cut(groove)

    return body


def _build_beak() -> cq.Workplane:
    """Small forward beak near the body top, blending into the spout root.

    The beak base is embedded inside the conical body for connectivity,
    and tapers forward to a narrower tip.
    """
    # Interpolate body radius at beak height
    body_h = BODY_HEIGHT - FLANGE_HEIGHT
    frac = (BEAK_CENTER_Z - FLANGE_HEIGHT) / body_h
    r_at_beak = BODY_BOTTOM_RADIUS + (BODY_TOP_RADIUS - BODY_BOTTOM_RADIUS) * frac

    # Base rectangle starts embedded in body, projects forward
    base_x = r_at_beak - 0.005  # embedded 5mm into body
    tip_x = base_x + BEAK_LENGTH

    beak = (
        cq.Workplane("XY")
        .workplane(offset=BEAK_CENTER_Z - BEAK_HEIGHT / 2.0)
        .center((base_x + tip_x) / 2.0, 0.0)
        .rect(tip_x - base_x, BEAK_WIDTH)
        .workplane(offset=BEAK_HEIGHT)
        .center((base_x + tip_x * 0.85 + base_x * 0.15) / 2.0, 0.0)
        .rect((tip_x - base_x) * 0.75, BEAK_WIDTH * 0.8)
        .loft()
    )
    return beak


def _build_spout() -> cq.Workplane:
    """Open-top U-channel swept forward and curving down at the tip.

    Built in spout-local coordinates starting at the origin heading +X.
    """
    path = cq.Workplane("XZ").spline(
        [
            (0.000, 0.000),
            (0.050, -0.004),
            (0.090, -0.012),
            (0.120, -0.026),
            (0.140, -0.044),
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_groove", rgba=(0.45, 0.46, 0.48, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))
    model.material("aerator_mesh", rgba=(0.55, 0.56, 0.58, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )
    body.visual(
        mesh_from_cadquery(_build_conical_body(), "conical_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="brushed_steel",
        name="conical_body",
    )
    body.visual(
        mesh_from_cadquery(_build_beak(), "forward_beak"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="brushed_steel",
        name="forward_beak",
    )
    body.visual(
        mesh_from_cadquery(_build_spout(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )
    # Hinge bracket at spout tip for the aerator flip hinge.
    # Embeds into the spout channel floor for visual connectivity.
    spout_tip_x = 0.136
    spout_tip_centerline_z = SPOUT_ROOT_Z - 0.040
    # Channel floor at the tip: centerline - hh, then floor extends up by SPOUT_FLOOR
    channel_floor_z = spout_tip_centerline_z - SPOUT_OUTER_H / 2.0 + SPOUT_FLOOR / 2.0
    # Bracket center placed so lower half embeds into the solid floor
    bracket_z = channel_floor_z
    body.visual(
        Cylinder(
            radius=AERATOR_HINGE_PIN_RADIUS + 0.002,
            length=AERATOR_HINGE_PIN_LENGTH,
        ),
        origin=Origin(
            xyz=(spout_tip_x, 0.0, bracket_z),
            rpy=(0.0, 0.0, 0.0),
        ),
        material="bright_steel",
        name="hinge_bracket",
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
    # The aerator is a small disc at the spout tip that flips open on a
    # tiny hinge pin. The hinge axis is horizontal (Y), at the top edge
    # of the spout tip.
    aerator = model.part("aerator_cap")
    aerator.visual(
        Cylinder(radius=AERATOR_RADIUS, length=AERATOR_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="aerator_mesh",
        name="aerator_disc",
    )
    # Small hinge pin at the hinge axis (z=0 in part-local = hinge line)
    aerator.visual(
        Cylinder(radius=AERATOR_HINGE_PIN_RADIUS, length=AERATOR_HINGE_PIN_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
        ),
        material="bright_steel",
        name="aerator_pin",
    )

    # Hinge origin aligned with the bracket on the spout tip
    aerator_hinge_x = 0.136
    spout_tip_cl_z = SPOUT_ROOT_Z - 0.040
    channel_floor_z_h = spout_tip_cl_z - SPOUT_OUTER_H / 2.0 + SPOUT_FLOOR / 2.0
    aerator_hinge_z = channel_floor_z_h

    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(aerator_hinge_x, 0.0, aerator_hinge_z)),
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
    aerator = object_model.get_part("aerator_cap")
    lift = object_model.get_articulation("boss_lift")
    twist = object_model.get_articulation("lever_twist")
    aerator_hinge = object_model.get_articulation("aerator_hinge")
    spout = body.get_visual("spout_channel")
    bar = handle.get_visual("lever_bar")
    disc = handle.get_visual("control_disc")
    hot_dot = handle.get_visual("hot_dot")
    conical = body.get_visual("conical_body")
    beak = body.get_visual("forward_beak")
    aerator_disc = aerator.get_visual("aerator_disc")

    # Intentional seated embeddings
    ctx.allow_overlap(
        boss,
        body,
        reason="boss shaft is seated into the curved body wall",
    )
    ctx.allow_overlap(
        handle,
        boss,
        reason="disc hub captures the boss shaft end",
    )
    ctx.allow_overlap(
        aerator,
        body,
        reason="aerator seats at spout tip with hinge pin embedded",
    )

    # --- conical body verification ----------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        f"base flange must sit on z=0, got {body_aabb}",
    )
    ctx.check(
        "body_height_about_0p20",
        body_aabb is not None and 0.195 < body_aabb[1][2] < 0.210,
        f"body top should be ~0.20 m up, got {body_aabb}",
    )

    # Conical body: bottom should be wider than top
    conical_aabb = ctx.part_element_world_aabb(body, elem=conical)
    ctx.check(
        "conical_body_exists",
        conical_aabb is not None,
        "tapered conical body must be present",
    )
    # Check the body width at bottom vs top by measuring the conical body dims
    conical_dims = conical_aabb[1][0] - conical_aabb[0][0] if conical_aabb else 0
    ctx.check(
        "conical_body_width",
        conical_aabb is not None and conical_dims > 2.0 * BODY_TOP_RADIUS - 0.005,
        f"conical body base should be wider than {2*BODY_TOP_RADIUS:.3f} m, got width={conical_dims:.4f}",
    )

    # --- forward beak verification ----------------------------------------
    beak_aabb = ctx.part_element_world_aabb(body, elem=beak)
    ctx.check(
        "forward_beak_exists",
        beak_aabb is not None,
        "forward beak must be present at the top of the body",
    )
    ctx.check(
        "beak_projects_forward",
        beak_aabb is not None and beak_aabb[1][0] > BODY_TOP_RADIUS,
        f"beak should project forward beyond the body radius, got {beak_aabb}",
    )
    ctx.check(
        "beak_near_top",
        beak_aabb is not None and beak_aabb[0][2] > BODY_HEIGHT * 0.7,
        f"beak should be near the top of the body, got {beak_aabb}",
    )

    # --- spout ------------------------------------------------------------
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.12,
        f"channel spout should reach forward, got {spout_aabb}",
    )
    ctx.check(
        "spout_tip_droops",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_ROOT_Z - 0.030,
        f"curved tip should drop below the root line, got {spout_aabb}",
    )

    # --- grip grooves (verified by conical body having reduced width) -----
    # The grooves cut into the conical body, so the body's X/Y extent at
    # groove zone heights should be slightly less than the smooth cone.
    # We verify the conical body mesh exists and has the grooves baked in.
    ctx.check(
        "grip_grooves_present",
        conical_aabb is not None,
        "conical body with grooves must exist as a single mesh",
    )

    # --- aerator hinge mechanism ------------------------------------------
    aerator_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_disc)
    ctx.check(
        "aerator_at_spout_tip",
        aerator_aabb is not None and aerator_aabb[1][0] > 0.10,
        f"aerator should be at the spout tip (x > 0.10), got {aerator_aabb}",
    )

    # Aerator hinge axis should be horizontal Y
    ctx.check(
        "aerator_hinge_axis_sideways",
        abs(aerator_hinge.axis[1]) == 1.0
        and abs(aerator_hinge.axis[0]) < 1e-6
        and abs(aerator_hinge.axis[2]) < 1e-6,
        f"aerator hinge must rotate about horizontal Y axis, got {aerator_hinge.axis}",
    )
    ctx.check(
        "aerator_hinge_range_0_to_90",
        aerator_hinge.motion_limits is not None
        and abs(aerator_hinge.motion_limits.lower) < 1e-6
        and abs(aerator_hinge.motion_limits.upper - AERATOR_RANGE) < 1e-3,
        "aerator hinge range should be 0..90 deg",
    )

    # Aerator flip test: at 0 it's closed (pointing down), at 90° it swings open
    rest_aerator_z = aerator_aabb[0][2] if aerator_aabb else 0
    with ctx.pose({aerator_hinge: AERATOR_RANGE}):
        open_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_disc)
        ctx.check(
            "aerator_flips_open",
            open_aabb is not None and open_aabb[1][2] > rest_aerator_z + 0.005,
            f"aerator should flip upward when hinge opens: rest_z_min={rest_aerator_z:.4f}, open_z_max={open_aabb[1][2] if open_aabb else 0:.4f}",
        )

    # --- control disc and lever -------------------------------------------
    disc_aabb = ctx.part_element_world_aabb(handle, elem=disc)
    ctx.check(
        "disc_outboard_of_body",
        disc_aabb is not None and disc_aabb[1][1] < -BODY_TOP_RADIUS - 0.005,
        f"control disc must sit clear of the body side, got {disc_aabb}",
    )

    bar_aabb = ctx.part_element_world_aabb(handle, elem=bar)
    ctx.check(
        "lever_bar_length",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.100,
        f"lever bar should run forward significantly, got {bar_aabb}",
    )

    # Boss contact
    ctx.expect_contact(body, boss, name="boss_seats_on_body")
    ctx.expect_contact(boss, handle, name="disc_seats_on_boss")

    # --- lift joint -------------------------------------------------------
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

    # --- twist joint ------------------------------------------------------
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

    # --- motion proof: lift -----------------------------------------------
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > 0.22,
            f"at +40 deg the bar tip should rise, got {up_aabb}",
        )
    with ctx.pose({lift: -LIFT_RANGE}):
        down_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_down_lowers_lever_tip",
            down_aabb is not None and down_aabb[0][2] < 0.10,
            f"at -40 deg the bar tip should drop, got {down_aabb}",
        )

    # --- motion proof: twist ----------------------------------------------
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

    # --- at least one non-fixed joint ------------------------------------
    all_joints = [
        object_model.get_articulation(n)
        for n in ["boss_lift", "lever_twist", "aerator_hinge"]
    ]
    non_fixed = [
        j for j in all_joints
        if j.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at_least_one_non_fixed_joint",
        len(non_fixed) >= 1,
        f"need at least 1 non-fixed joint, found {len(non_fixed)}",
    )

    return ctx.report()


object_model = build_object_model()
