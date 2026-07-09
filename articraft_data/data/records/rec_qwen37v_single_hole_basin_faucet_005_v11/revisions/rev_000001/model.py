from __future__ import annotations

"""Single-hole basin faucet — tall tower variant with swiveling spout.

A tall cylindrical tower body (~0.28 m, ~0.055 m diameter) on a round base
flange. A short hollow tube spout projects forward from near the top and
swivels around the vertical body axis. A separate circular aerator insert
sits at the spout mouth. On the side, a lever-and-disc control assembly
rides on a horizontal boss (same mechanism as the parent faucet).

Articulation chain:
- ``spout_swivel``: body → spout_neck, REVOLUTE about Z (vertical),
  -90..+90 deg; swivels the spout left/right.
- ``boss_lift``: body → lever_boss, REVOLUTE about -Y (horizontal sideways),
  -40..+40 deg; lifts/lowers the lever (flow control).
- ``lever_twist``: lever_boss → lever_handle, REVOLUTE about X (forward),
  -30..+30 deg; temperature mix.
- ``aerator_seat``: spout_neck → aerator, FIXED.
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
BODY_RADIUS = 0.0275       # 0.055 m diameter column
BODY_HEIGHT = 0.280        # taller tower
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

SPOUT_ROOT_Z = 0.250       # spout height near body top
SPOUT_OUTER_R = 0.012      # tube outer radius
SPOUT_INNER_R = 0.009      # tube bore radius
SPOUT_LENGTH = 0.085       # tube length along +X

COLLAR_RADIUS = 0.016      # swivel collar outer radius
COLLAR_LENGTH = 0.012      # collar width along X

AERATOR_RADIUS = 0.0092    # slight press-fit into bore (r=0.009)
AERATOR_THICKNESS = 0.004

DISC_RADIUS = 0.0275       # 0.055 m diameter control disc
DISC_THICKNESS = 0.012
DISC_CENTER_Y = -0.048
DISC_CENTER_Z = 0.235      # adjusted for taller body

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

SWIVEL_RANGE = math.radians(90.0)
LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)


# ---------------------------------------------------------------------------
# CadQuery geometry builders
# ---------------------------------------------------------------------------

def _build_spout_tube() -> cq.Workplane:
    """Hollow tube with annular cross-section extruded along +X.

    The bore is open at both ends; the mouth end shows the real hollow outlet.
    """
    tube = (
        cq.Workplane("YZ")
        .circle(SPOUT_OUTER_R)
        .circle(SPOUT_INNER_R)
        .extrude(SPOUT_LENGTH)
    )
    return tube


def _build_aerator_disc() -> cq.Workplane:
    """Thin circular disc with a center hole and ring of smaller holes.

    Built in XY plane, extruded along +Z. The articulation rotates it to
    face along +X at the spout mouth.
    """
    r = AERATOR_RADIUS
    t = AERATOR_THICKNESS
    center_hole_r = r * 0.30
    ring_hole_r = r * 0.15
    ring_r = r * 0.60
    n_holes = 8

    pts = [
        (ring_r * math.cos(2.0 * math.pi * i / n_holes),
         ring_r * math.sin(2.0 * math.pi * i / n_holes))
        for i in range(n_holes)
    ]

    disc = (
        cq.Workplane("XY")
        .circle(r)
        .extrude(t)
        .faces(">Z").workplane()
        .circle(center_hole_r)
        .cutThruAll()
        .faces(">Z").workplane()
        .pushPoints(pts)
        .circle(ring_hole_r)
        .cutThruAll()
    )
    return disc


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("aerator_mesh", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))

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

    # --------------------------------------------------------- spout (swivel)
    spout = model.part("spout_neck")
    # Hollow tube — the annular extrusion leaves the bore open at both ends.
    spout.visual(
        mesh_from_cadquery(_build_spout_tube(), "spout_tube"),
        origin=Origin(),
        material="brushed_steel",
        name="spout_tube",
    )
    # Swivel collar at the body junction
    collar_cx = 0.024 + COLLAR_LENGTH / 2.0
    spout.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(xyz=(collar_cx, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="spout_collar",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=2.0,
            lower=-SWIVEL_RANGE, upper=SWIVEL_RANGE,
        ),
    )

    # --------------------------------------------------- aerator (fixed seat)
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator_disc(), "aerator_disc"),
        origin=Origin(),
        material="aerator_mesh",
        name="aerator_disc",
    )

    # Aerator seated at the spout mouth.  The disc (built in XY, extruded
    # along +Z) is rotated so +Z maps to +X; the origin places it at the
    # mouth end of the tube.
    aerator_x = SPOUT_LENGTH - AERATOR_THICKNESS
    model.articulation(
        "aerator_seat",
        ArticulationType.FIXED,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(aerator_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
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
            effort=8.0, velocity=2.0,
            lower=-LIFT_RANGE, upper=LIFT_RANGE,
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
            effort=6.0, velocity=2.0,
            lower=-TWIST_RANGE, upper=TWIST_RANGE,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    spout = object_model.get_part("spout_neck")
    aerator = object_model.get_part("aerator")
    boss = object_model.get_part("lever_boss")
    handle = object_model.get_part("lever_handle")

    swivel = object_model.get_articulation("spout_swivel")
    lift = object_model.get_articulation("boss_lift")
    twist = object_model.get_articulation("lever_twist")

    tube = spout.get_visual("spout_tube")
    collar = spout.get_visual("spout_collar")
    disc = handle.get_visual("control_disc")
    bar = handle.get_visual("lever_bar")
    hot_dot = handle.get_visual("hot_dot")

    # --- intentional overlaps -----------------------------------------------
    ctx.allow_overlap(
        spout, body,
        elem_a="spout_tube",
        elem_b="body_column",
        reason="spout tube root is embedded in the body column for the swivel joint",
    )
    ctx.allow_overlap(
        spout, body,
        elem_a="spout_collar",
        elem_b="body_column",
        reason="spout collar wraps around the tube at the body surface for the swivel joint",
    )
    ctx.allow_overlap(
        boss, body,
        reason="boss shaft is seated into the curved body wall",
    )
    ctx.allow_overlap(
        handle, boss,
        reason="disc hub captures the boss shaft end (0.5 mm seat)",
    )
    ctx.allow_overlap(
        aerator, spout,
        elem_a="aerator_disc",
        elem_b="spout_tube",
        reason="aerator disc is press-fit into the spout bore (0.2 mm radial interference)",
    )

    # --- static form --------------------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and abs(aabb[0][2]) < 1e-6,
        f"base flange must sit on z=0, got {aabb}",
    )
    ctx.check(
        "body_taller_tower",
        aabb is not None and 0.275 < aabb[1][2] < 0.285,
        f"taller tower body should be ~0.280 m, got {aabb}",
    )

    # Spout is a separate part projecting forward
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout_is_separate_part",
        spout_aabb is not None,
        "spout must be a separate part from the body",
    )
    ctx.check(
        "spout_short_forward",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.065
        and (spout_aabb[1][0] - spout_aabb[0][0]) < 0.120,
        f"short forward spout should project ~0.085 m, got {spout_aabb}",
    )

    # Spout height near body top
    ctx.check(
        "spout_near_body_top",
        spout_aabb is not None and spout_aabb[0][2] > 0.230,
        f"spout should be near the top of the tower, got {spout_aabb}",
    )

    # Hollow outlet tube exists
    ctx.check(
        "hollow_outlet_tube_exists",
        tube is not None,
        "spout must have a hollow tube (annular CadQuery mesh) for the outlet",
    )

    # Collar at swivel joint
    ctx.check(
        "swivel_collar_exists",
        collar is not None,
        "spout should have a visible collar at the body junction",
    )

    # Aerator is a separate part at the mouth
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator_is_separate_part",
        aerator_aabb is not None,
        "aerator must be a separate part",
    )
    ctx.check(
        "aerator_at_spout_mouth",
        aerator_aabb is not None and aerator_aabb[0][0] > 0.060,
        f"aerator should be near the spout mouth, got {aerator_aabb}",
    )

    # Aerator fits within the spout bore cross-section (YZ containment)
    ctx.expect_within(
        aerator, spout,
        axes="yz",
        margin=0.004,
        name="aerator_within_spout_bore_cross_section",
    )
    # Aerator press-fit contacts the bore wall
    ctx.expect_contact(
        aerator, spout,
        elem_a="aerator_disc",
        elem_b="spout_tube",
        name="aerator_press_fit_contacts_bore",
    )

    # --- spout swivel joint -------------------------------------------------
    ctx.check(
        "swivel_is_revolute",
        swivel.articulation_type == ArticulationType.REVOLUTE,
        f"spout swivel must be revolute, got {swivel.articulation_type}",
    )
    ctx.check(
        "swivel_axis_vertical",
        abs(swivel.axis[2]) == 1.0
        and swivel.axis[0] == 0.0
        and swivel.axis[1] == 0.0,
        f"swivel must rotate about vertical Z axis, got {swivel.axis}",
    )
    ctx.check(
        "swivel_range_pm90deg",
        swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_RANGE) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_RANGE) < 1e-6,
        "swivel range must be -90..+90 deg",
    )

    # Swivel motion proof: at +90° the spout should point to +Y
    rest_spout = ctx.part_world_aabb(spout)
    with ctx.pose({swivel: SWIVEL_RANGE}):
        swiveled = ctx.part_world_aabb(spout)
        ctx.check(
            "spout_swivels_to_side",
            swiveled is not None and swiveled[1][1] > 0.060,
            f"at +90° the spout should point to +Y (max_y > 0.060), got {swiveled}",
        )
        # The spout's forward extent should decrease (now pointing sideways)
        ctx.check(
            "swiveled_spout_less_forward",
            rest_spout is not None
            and swiveled is not None
            and swiveled[1][0] < rest_spout[1][0] - 0.020,
            f"swiveled spout should reach less forward than rest: rest={rest_spout}, swiveled={swiveled}",
        )

    # --- lever mechanism (preserved from parent) ----------------------------
    ctx.check(
        "lift_axis_sideways",
        abs(lift.axis[1]) == 1.0 and lift.axis[0] == 0.0 and lift.axis[2] == 0.0,
        f"lift must rotate about horizontal Y axis, got {lift.axis}",
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
        f"twist must rotate about lever's forward axis, got {twist.axis}",
    )
    ctx.check(
        "twist_range_pm30deg",
        twist.motion_limits is not None
        and abs(twist.motion_limits.lower + TWIST_RANGE) < 1e-6
        and abs(twist.motion_limits.upper - TWIST_RANGE) < 1e-6,
        "twist range must be -30..+30 deg",
    )

    # Lift motion proof
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > 0.300,
            f"at +40° the bar tip should rise above 0.30 m, got {up_aabb}",
        )

    with ctx.pose({lift: -LIFT_RANGE}):
        down_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_down_lowers_lever_tip",
            down_aabb is not None and down_aabb[0][2] < 0.160,
            f"at -40° the bar tip should drop below 0.16 m, got {down_aabb}",
        )

    # Contact checks for the lever stack
    ctx.expect_contact(body, boss, name="boss_seats_on_body")
    ctx.expect_contact(boss, handle, name="disc_seats_on_boss")

    # Non-fixed joint count check (at least the spout swivel)
    non_fixed = [
        j for j in object_model.articulations
        if j.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "has_non_fixed_joints",
        len(non_fixed) >= 1,
        f"must have at least 1 non-fixed joint, found {len(non_fixed)}",
    )

    return ctx.report()


object_model = build_object_model()
