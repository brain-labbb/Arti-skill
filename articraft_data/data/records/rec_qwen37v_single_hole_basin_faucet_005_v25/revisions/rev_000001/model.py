from __future__ import annotations

"""Single-hole basin faucet variant with swiveling curved spout.

A vertical cylindrical column (~0.20 m tall, ~0.055 m diameter) on a wider
round base flange with an oval rubber gasket. A hollow tubular spout curves
gently downward from the upper body and swivels around the vertical body axis.
On the right side near the top, a control disc with red/blue index dots is
mounted on a short horizontal boss; a slim lever bar runs forward.

Articulation chain:
- ``spout_swivel``: body -> spout, revolute about Z (vertical), ±90 deg.
- ``boss_lift``: body -> boss, revolute about Y (sideways), ±40 deg.
- ``lever_twist``: boss -> handle, revolute about X (forward), ±30 deg.
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
# Dimensions (meters). World frame: +X forward (spout default direction),
# +Z up, control disc on the -Y side of the body.
# ----------------------------------------------------------------------------
BODY_RADIUS = 0.0275   # 0.055 m diameter column
BODY_HEIGHT = 0.200
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

# Oval gasket
GASKET_A = 0.040       # semi-major axis (X direction)
GASKET_B = 0.036       # semi-minor axis (Y direction)
GASKET_INNER = 0.028   # inner cutout radius (clears body column)
GASKET_THICK = 0.004

# Spout tube
SPOUT_ROOT_Z = 0.155   # where the spout exits the body column (near top)
SPOUT_OUTER_R = 0.012  # outer radius of the spout tube
SPOUT_INNER_R = 0.009  # inner radius (hollow channel)
SPOUT_LENGTH = 0.140   # approximate arc length of the spout

# Spout collar (ring where spout meets body)
COLLAR_RADIUS = 0.016
COLLAR_LENGTH = 0.010

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

SWIVEL_RANGE = math.radians(90.0)
LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)


def _build_spout_tube() -> cq.Workplane:
    """Hollow tubular spout that curves gently forward and downward.

    Built in spout-local coords: origin at the body column surface where the
    spout emerges (at x=BODY_RADIUS, z=0 relative to the spout part frame).
    The tube centerline heads +X and droops. The outlet end is open (hollow).
    """
    # Path in XZ plane: starts at body surface, sweeps forward and down
    x0 = BODY_RADIUS
    path = cq.Workplane("XZ").spline(
        [
            (x0, 0.000),
            (x0 + 0.030, -0.004),
            (x0 + 0.065, -0.014),
            (x0 + 0.095, -0.030),
            (x0 + 0.120, -0.052),
        ]
    )
    # Annular cross-section (hollow tube with open ends)
    profile = (
        cq.Workplane("YZ")
        .transformed(offset=(x0, 0.0, 0.0))
        .circle(SPOUT_OUTER_R)
        .circle(SPOUT_INNER_R)
    )
    return profile.sweep(path)


def _build_oval_gasket() -> cq.Workplane:
    """Flat oval (elliptical) ring gasket for the base."""
    gasket = (
        cq.Workplane("XY")
        .ellipse(GASKET_A, GASKET_B)
        .ellipse(GASKET_INNER, GASKET_INNER)
        .extrude(GASKET_THICK)
    )
    return gasket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("hot_red", rgba=(0.80, 0.12, 0.10, 1.0))
    model.material("cold_blue", rgba=(0.10, 0.25, 0.80, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    # Base flange
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, GASKET_THICK + FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )
    # Oval gasket under the flange
    body.visual(
        mesh_from_cadquery(_build_oval_gasket(), "oval_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="rubber_black",
        name="oval_gasket",
    )
    # Main column
    column_len = BODY_HEIGHT - GASKET_THICK - FLANGE_HEIGHT - 0.005
    column_base = GASKET_THICK + FLANGE_HEIGHT
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, column_base + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    # Flat top cap
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, column_base + column_len + 0.0025)),
        material="brushed_steel",
        name="top_cap",
    )

    # ------------------------------------------------------------------ spout
    # The spout is a separate part that swivels around the body Z axis.
    # Its local frame origin is at the swivel point on the body centerline.
    spout = model.part("spout")
    # Collar ring where spout meets body - wraps around the body column
    spout.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bright_steel",
        name="spout_collar",
    )
    # Curved hollow tube - starts at body surface, sweeps forward and down
    spout.visual(
        mesh_from_cadquery(_build_spout_tube(), "spout_tube"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="brushed_steel",
        name="spout_tube",
    )

    # Spout swivel articulation: rotates about Z at the body-spout junction
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

    spout_tube = spout.get_visual("spout_tube")
    spout_collar = spout.get_visual("spout_collar")
    gasket = body.get_visual("oval_gasket")
    column = body.get_visual("body_column")
    disc = handle.get_visual("control_disc")
    bar = handle.get_visual("lever_bar")

    # Intentional seated embeddings
    ctx.allow_overlap(
        spout,
        body,
        reason="spout collar wraps around body column at the swivel joint; tube root seats at column wall",
    )
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

    # --- static form -------------------------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        f"gasket base must sit on z=0, got {body_aabb}",
    )
    ctx.check(
        "body_height_about_0p20",
        body_aabb is not None and 0.190 < body_aabb[1][2] < 0.210,
        f"body top should be ~0.20 m up, got {body_aabb}",
    )

    # --- oval gasket -------------------------------------------------------
    gasket_aabb = ctx.part_element_world_aabb(body, elem=gasket)
    ctx.check(
        "oval_gasket_exists",
        gasket_aabb is not None,
        "oval base gasket must be present",
    )
    if gasket_aabb is not None:
        gasket_dx = gasket_aabb[1][0] - gasket_aabb[0][0]
        gasket_dy = gasket_aabb[1][1] - gasket_aabb[0][1]
        ctx.check(
            "oval_gasket_shape",
            gasket_dx > gasket_dy,
            f"gasket should be wider in X (oval), got dx={gasket_dx:.4f} dy={gasket_dy:.4f}",
        )
        ctx.check(
            "gasket_at_base",
            gasket_aabb[0][2] < 0.005,
            f"gasket should sit at the base, got z_min={gasket_aabb[0][2]:.4f}",
        )

    # --- spout geometry ----------------------------------------------------
    tube_aabb = ctx.part_element_world_aabb(spout, elem=spout_tube)
    ctx.check(
        "spout_projects_forward",
        tube_aabb is not None and tube_aabb[1][0] > 0.10,
        f"spout tube should project forward >0.10 m, got {tube_aabb}",
    )
    ctx.check(
        "spout_curves_downward",
        tube_aabb is not None and tube_aabb[0][2] < SPOUT_ROOT_Z - 0.020,
        f"spout tip should droop below root, got z_min={tube_aabb[0][2] if tube_aabb else None}",
    )

    # Hollow outlet: the spout tube is a hollow annular sweep, so the
    # Y-extent at the cross-section should match the tube outer diameter.
    ctx.check(
        "spout_tube_diameter",
        tube_aabb is not None
        and abs((tube_aabb[1][1] - tube_aabb[0][1]) - 2 * SPOUT_OUTER_R) < 0.004,
        f"spout tube Y-extent should be ~{2*SPOUT_OUTER_R:.3f} m, got {tube_aabb}",
    )

    # --- spout swivel joint ------------------------------------------------
    ctx.check(
        "swivel_axis_vertical",
        abs(swivel.axis[2]) == 1.0 and swivel.axis[0] == 0.0 and swivel.axis[1] == 0.0,
        f"spout swivel must rotate about vertical Z axis, got {swivel.axis}",
    )
    ctx.check(
        "swivel_range_pm90deg",
        swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_RANGE) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_RANGE) < 1e-6,
        "swivel range must be -90..+90 deg",
    )

    # Swivel motion proof: rotating the spout about Z should move the tube
    # tip sideways (in Y).
    rest_tube_aabb = ctx.part_element_world_aabb(spout, elem=spout_tube)
    with ctx.pose({swivel: SWIVEL_RANGE}):
        swung_aabb = ctx.part_element_world_aabb(spout, elem=spout_tube)
        ctx.check(
            "swivel_moves_spout_sideways",
            rest_tube_aabb is not None and swung_aabb is not None
            and abs(swung_aabb[1][1] - rest_tube_aabb[1][1]) > 0.05,
            f"at +90 deg the spout tip should swing sideways: rest_y={rest_tube_aabb[1][1] if rest_tube_aabb else None}, swung_y={swung_aabb[1][1] if swung_aabb else None}",
        )

    # --- boss and lever (preserved from parent) ----------------------------
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
        f"twist must rotate about forward X axis, got {twist.axis}",
    )
    ctx.check(
        "twist_range_pm30deg",
        twist.motion_limits is not None
        and abs(twist.motion_limits.lower + TWIST_RANGE) < 1e-6
        and abs(twist.motion_limits.upper - TWIST_RANGE) < 1e-6,
        "twist range must be -30..+30 deg",
    )

    # Boss support contacts
    ctx.expect_contact(body, boss, name="boss_seats_on_body")
    ctx.expect_contact(boss, handle, name="disc_seats_on_boss")

    # Spout junction proof (paired with allow_overlap for spout-body)
    ctx.expect_contact(body, spout, name="spout_seats_at_body_junction")
    ctx.expect_overlap(
        body, spout,
        axes="z",
        min_overlap=0.005,
        name="spout_retained_at_body_z",
    )

    # Lift motion proof
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > 0.22,
            f"at +40 deg the bar tip should rise, got {up_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
