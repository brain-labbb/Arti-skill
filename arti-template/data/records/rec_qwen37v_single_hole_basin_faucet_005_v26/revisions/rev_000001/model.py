from __future__ import annotations

"""Single-hole basin faucet variant with offset side lever and pull-up drain rod.

A vertical cylindrical body (~0.20 m tall, ~0.055 m diameter) on a slightly
wider round base flange, single-hole deck mount. A flat-bottomed open-channel
spout projects forward and slightly down ~0.13 m from the front of the body.

An offset side lever housing sits on the -Y side of the body near the top,
with a thin cartridge cap seam ring below it. A round flat control disc
(~0.055 m diameter, with red/blue index dots) is mounted on a short horizontal
boss protruding from the housing; a slim cylindrical lever bar with subtle grip
grooves runs forward from the disc.

A pull-up drain rod slides vertically behind the body column.

Articulation chain:
- ``boss_lift``: revolute about the horizontal sideways (Y) boss axis,
  -40..+40 deg; positive q lifts the lever tip up (flow control).
- ``lever_twist``: revolute about the lever's own forward (X) axis through
  the disc center, -30..+30 deg (temperature mix).
- ``drain_slide``: prismatic along +Z, 0..0.04 m; pull-up to open drain.
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

# Lever housing: offset cylindrical bump on the -Y side
HOUSING_RADIUS = 0.019
HOUSING_HEIGHT = 0.028
HOUSING_CENTER_Y = -0.036  # offset outboard of body surface
HOUSING_CENTER_Z = 0.165

# Cartridge cap seam: thin trim ring around body column below lever area
CAP_RADIUS = 0.030  # slightly proud of body surface (body radius 0.0275)
CAP_HEIGHT = 0.002
CAP_CENTER_Z = 0.138  # clearly below the lever housing (housing Z range ~0.146-0.184)

DISC_RADIUS = 0.0275  # 0.055 m diameter control disc
DISC_THICKNESS = 0.012
DISC_CENTER_Y = -0.058  # disc mid-plane (outboard of housing)
DISC_CENTER_Z = HOUSING_CENTER_Z

BOSS_RADIUS = 0.011
BOSS_LENGTH = 0.018
BOSS_CENTER_Y = -0.043  # lift joint origin on the boss axis

BAR_RADIUS = 0.005
BAR_X_START = 0.014  # clears the boss (boss radius 0.011)
BAR_X_END = 0.150
BAR_OFFSET_Y = 0.0095  # inboard of the disc mid-plane, toward the body

DOT_RADIUS = 0.0035
DOT_LENGTH = 0.0018
DOT_Z = 0.019  # radial offset of the paint dots on the disc face

# Drain rod
DRAIN_ROD_RADIUS = 0.003
DRAIN_ROD_LENGTH = 0.055
DRAIN_ROD_X = -0.035  # behind the body, clear of body column (radius 0.0275)
DRAIN_ROD_Y = 0.0
DRAIN_ROD_BASE_Z = 0.060  # bottom of rod travel starts here
DRAIN_CAP_RADIUS = 0.006
DRAIN_CAP_HEIGHT = 0.008
DRAIN_SLIDE_RANGE = 0.040  # 40 mm travel
# Guide bracket on body back to support drain rod visually
BRACKET_SIZE_X = 0.012  # bridge from body surface to drain rod
BRACKET_SIZE_Y = 0.010
BRACKET_SIZE_Z = 0.008
BRACKET_Z = 0.090  # height of bracket on body back

LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)


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


def _build_grooved_lever_bar() -> cq.Workplane:
    """Slim cylindrical lever bar with subtle grip grooves cut along its length.

    Built in bar-local coordinates: the cylinder axis runs along local X,
    centered at origin. Grooves are small circumferential cuts.
    """
    bar_len = BAR_X_END - BAR_X_START
    # Main bar cylinder along X
    bar = (
        cq.Workplane("YZ")
        .circle(BAR_RADIUS)
        .extrude(bar_len)
    )
    # Cut subtle grip grooves along the forward portion (last 60% of bar)
    groove_depth = 0.0008
    groove_width = 0.0012
    groove_spacing = 0.005
    groove_start = bar_len * 0.40
    groove_end = bar_len * 0.95
    n_grooves = int((groove_end - groove_start) / groove_spacing)
    for i in range(n_grooves):
        gx = groove_start + i * groove_spacing
        groove_ring = (
            cq.Workplane("YZ")
            .workplane(offset=gx - groove_width / 2.0)
            .circle(BAR_RADIUS + 0.0001)
            .circle(BAR_RADIUS - groove_depth)
            .extrude(groove_width)
        )
        bar = bar.cut(groove_ring)
    return bar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))
    model.material("dark_seam", rgba=(0.35, 0.36, 0.38, 1.0))
    model.material("chrome_cap", rgba=(0.82, 0.83, 0.85, 1.0))

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
    # Offset side lever housing
    body.visual(
        Cylinder(radius=HOUSING_RADIUS, length=HOUSING_HEIGHT),
        origin=Origin(
            xyz=(0.0, HOUSING_CENTER_Y, HOUSING_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="brushed_steel",
        name="lever_housing",
    )
    # Cartridge cap seam ring around body column below lever
    body.visual(
        Cylinder(radius=CAP_RADIUS, length=CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CAP_CENTER_Z)),
        material="dark_seam",
        name="cartridge_cap",
    )
    # Guide bracket on body back to visually support the drain rod
    body.visual(
        Box((BRACKET_SIZE_X, BRACKET_SIZE_Y, BRACKET_SIZE_Z)),
        origin=Origin(
            xyz=(-BODY_RADIUS - BRACKET_SIZE_X / 2.0 + 0.002, 0.0, BRACKET_Z),
        ),
        material="brushed_steel",
        name="drain_guide_bracket",
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
        mesh_from_cadquery(_build_grooved_lever_bar(), "grooved_bar"),
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

    # ------------------------------------------------- drain rod
    drain = model.part("drain_rod")
    drain.visual(
        Cylinder(radius=DRAIN_ROD_RADIUS, length=DRAIN_ROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LENGTH / 2.0)),
        material="bright_steel",
        name="drain_shaft",
    )
    drain.visual(
        Cylinder(radius=DRAIN_CAP_RADIUS, length=DRAIN_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LENGTH + DRAIN_CAP_HEIGHT / 2.0)),
        material="chrome_cap",
        name="drain_cap",
    )

    model.articulation(
        "drain_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain,
        origin=Origin(xyz=(DRAIN_ROD_X, DRAIN_ROD_Y, DRAIN_ROD_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=0.1, lower=0.0, upper=DRAIN_SLIDE_RANGE
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
    slide = object_model.get_articulation("drain_slide")
    spout = body.get_visual("spout_channel")
    bar = handle.get_visual("lever_bar")
    disc = handle.get_visual("control_disc")
    hot_dot = handle.get_visual("hot_dot")
    housing = body.get_visual("lever_housing")
    cap = body.get_visual("cartridge_cap")
    drain_shaft = drain.get_visual("drain_shaft")
    drain_cap = drain.get_visual("drain_cap")

    # Intentional seated embeddings of the rotating stack.
    ctx.allow_overlap(
        boss,
        body,
        reason="boss shaft is seated into the lever housing wall",
    )
    ctx.allow_overlap(
        handle,
        boss,
        reason="disc hub captures the boss shaft end (0.5 mm seat)",
    )
    # Guide bracket embeds into the drain rod shaft (guide hole representation)
    ctx.allow_overlap(
        body,
        drain,
        elem_a=body.get_visual("drain_guide_bracket"),
        elem_b=drain_shaft,
        reason="bracket guide hole captures the drain rod shaft",
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

    # --- variant-specific: lever housing on side --------------------------
    housing_aabb = ctx.part_element_world_aabb(body, elem=housing)
    ctx.check(
        "lever_housing_on_side",
        housing_aabb is not None and housing_aabb[0][1] < -BODY_RADIUS,
        f"lever housing must protrude outboard of body, got {housing_aabb}",
    )

    # --- variant-specific: cartridge cap seam below lever -----------------
    cap_aabb = ctx.part_element_world_aabb(body, elem=cap)
    ctx.check(
        "cartridge_cap_below_housing",
        cap_aabb is not None and housing_aabb is not None
        and cap_aabb[1][2] < housing_aabb[0][2],
        f"cartridge cap seam must sit below the housing bottom, cap={cap_aabb}, housing={housing_aabb}",
    )
    ctx.check(
        "cartridge_cap_around_body",
        cap_aabb is not None
        and cap_aabb[1][0] > BODY_RADIUS
        and cap_aabb[0][0] < -BODY_RADIUS,
        f"cartridge cap ring must extend beyond body radius on both sides, got {cap_aabb}",
    )

    # --- variant-specific: drain rod behind body --------------------------
    drain_aabb = ctx.part_world_aabb(drain)
    ctx.check(
        "drain_rod_behind_body",
        drain_aabb is not None and drain_aabb[0][0] < 0.0,
        f"drain rod must sit behind the body (negative X), got {drain_aabb}",
    )
    cap_vis_aabb = ctx.part_element_world_aabb(drain, elem=drain_cap)
    ctx.check(
        "drain_cap_on_top",
        cap_vis_aabb is not None
        and cap_vis_aabb[1][2] > DRAIN_ROD_BASE_Z + DRAIN_ROD_LENGTH - 0.005,
        f"drain cap must be at the top of the rod, got {cap_vis_aabb}",
    )

    # --- joint plan: lift --------------------------------------------------
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

    # --- joint plan: twist -------------------------------------------------
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

    # --- joint plan: drain slide (prismatic) ------------------------------
    ctx.check(
        "drain_slide_axis_vertical",
        abs(slide.axis[2]) == 1.0 and slide.axis[0] == 0.0 and slide.axis[1] == 0.0,
        f"drain slide must be along +Z, got {slide.axis}",
    )
    ctx.check(
        "drain_slide_range",
        slide.motion_limits is not None
        and abs(slide.motion_limits.lower) < 1e-6
        and abs(slide.motion_limits.upper - DRAIN_SLIDE_RANGE) < 1e-6,
        f"drain slide range must be 0..{DRAIN_SLIDE_RANGE} m",
    )

    # --- motion proof: lift ------------------------------------------------
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > 0.245,
            f"at +40 deg the bar tip should rise well above body top, got {up_aabb}",
        )

    # --- motion proof: drain rod slides up ---------------------------------
    rest_drain = ctx.part_world_aabb(drain)
    with ctx.pose({slide: DRAIN_SLIDE_RANGE}):
        raised_drain = ctx.part_world_aabb(drain)
        ctx.check(
            "drain_rod_pulls_up",
            rest_drain is not None and raised_drain is not None
            and raised_drain[0][2] > rest_drain[0][2] + DRAIN_SLIDE_RANGE - 0.002,
            f"drain rod should rise ~{DRAIN_SLIDE_RANGE} m when pulled, rest={rest_drain}, raised={raised_drain}",
        )

    # --- motion proof: twist -----------------------------------------------
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
