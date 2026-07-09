from __future__ import annotations

"""Squared modern single-hole basin faucet variant.

A sharply squared monobloc body (~0.20 m tall, ~0.050 x 0.050 m cross-section)
sits on an oval base gasket. A rectangular channel spout projects forward and
slightly downward from the upper front face, with a real hollow open outlet at
the mouth. On the right side near the top, a short horizontal axle carries a
flat side lever that pivots up/down for flow control.

Articulation:
- ``lever_lift``: revolute about the horizontal sideways (Y) axle axis,
  -40..+40 deg; positive q lifts the lever tip up.
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

# ---------------------------------------------------------------------------
# Dimensions (meters). World frame: +X forward (spout direction), +Z up,
# lever on the -Y (right) side of the body.
# ---------------------------------------------------------------------------
BODY_W = 0.050       # body width (X)
BODY_D = 0.050       # body depth (Y)
BODY_HEIGHT = 0.200  # total body height
BODY_WALL = 0.005    # shell wall thickness (for squared look we use solid)
BODY_FILLET = 0.003  # small edge fillet for manufactured realism

GASKET_A = 0.038     # oval semi-major (X direction)
GASKET_B = 0.032     # oval semi-minor (Y direction)
GASKET_INNER_A = 0.024
GASKET_INNER_B = 0.020
GASKET_HEIGHT = 0.005

SPOUT_W = 0.030      # spout channel outer width (Y direction)
SPOUT_H = 0.022      # spout channel outer height (Z direction)
SPOUT_WALL = 0.004
SPOUT_LENGTH = 0.130 # spout projection length
SPOUT_ROOT_Z = 0.140 # where the spout leaves the body front face

OUTLET_DEPTH = 0.012 # hollow outlet cavity depth at spout mouth
OUTLET_W = SPOUT_W - 2 * SPOUT_WALL
OUTLET_H = SPOUT_H - 2 * SPOUT_WALL

AXLE_RADIUS = 0.008
AXLE_LENGTH = 0.018
AXLE_CENTER_Y = -(BODY_D / 2.0 + AXLE_LENGTH / 2.0)
AXLE_CENTER_Z = 0.165

LEVER_W = 0.014      # lever bar width
LEVER_H = 0.008      # lever bar height
LEVER_LENGTH = 0.110 # lever bar forward reach
LEVER_TIP_RADIUS = 0.007  # rounded end cap

LIFT_RANGE = math.radians(40.0)


def _build_squared_body() -> cq.Workplane:
    """Squared monobloc column with small fillets on vertical edges."""
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_HEIGHT, centered=(True, True, False))
    )
    # Fillet vertical edges for a manufactured modern look
    body = body.edges("|Z").fillet(BODY_FILLET)
    return body


def _build_oval_gasket() -> cq.Workplane:
    """Oval ring gasket (elliptical annulus extruded to GASKET_HEIGHT)."""
    outer = (
        cq.Workplane("XY")
        .ellipse(GASKET_A, GASKET_B)
    )
    inner = (
        cq.Workplane("XY")
        .ellipse(GASKET_INNER_A, GASKET_INNER_B)
    )
    # Build outer solid, then cut inner hole
    solid = outer.extrude(GASKET_HEIGHT)
    hole = inner.extrude(GASKET_HEIGHT)
    return solid.cut(hole)


def _build_spout_with_hollow_outlet() -> cq.Workplane:
    """Rectangular channel spout projecting forward with hollow open outlet.

    Built in local coords: origin at spout root, +X forward, spout body
    extends forward and slightly down. The mouth has a real hollow cavity
    cut into the front face.
    """
    # Spout outer shell - a box that angles slightly down
    # Use a sweep along a gentle downward path
    path = cq.Workplane("XZ").spline(
        [
            (0.0, 0.0),
            (0.040, -0.003),
            (0.085, -0.010),
            (0.120, -0.020),
            (SPOUT_LENGTH, -0.030),
        ]
    )
    hw = SPOUT_W / 2.0
    hh = SPOUT_H / 2.0
    profile = (
        cq.Workplane("YZ")
        .rect(SPOUT_W, SPOUT_H)
    )
    outer = profile.sweep(path)

    # Hollow interior channel (smaller rect swept along same path)
    inner_hw = hw - SPOUT_WALL
    inner_hh = hh - SPOUT_WALL
    inner_profile = (
        cq.Workplane("YZ")
        .rect(SPOUT_W - 2 * SPOUT_WALL, SPOUT_H - 2 * SPOUT_WALL)
    )
    inner_path = cq.Workplane("XZ").spline(
        [
            (0.0, 0.0),
            (0.040, -0.003),
            (0.085, -0.010),
            (0.120, -0.020),
            (SPOUT_LENGTH + 0.005, -0.030),  # extend slightly past to cut through
        ]
    )
    inner_solid = inner_profile.sweep(inner_path)

    # Cut interior from outer to make hollow tube
    hollow_spout = outer.cut(inner_solid)
    return hollow_spout


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("rubber_gasket", rgba=(0.18, 0.18, 0.20, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")

    # Squared monobloc column
    body.visual(
        mesh_from_cadquery(_build_squared_body(), "squared_body"),
        origin=Origin(xyz=(0.0, 0.0, GASKET_HEIGHT)),
        material="brushed_steel",
        name="body_column",
    )

    # Oval base gasket
    body.visual(
        mesh_from_cadquery(_build_oval_gasket(), "oval_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="rubber_gasket",
        name="base_gasket",
    )

    # Spout with hollow outlet
    body.visual(
        mesh_from_cadquery(_build_spout_with_hollow_outlet(), "spout_hollow"),
        origin=Origin(xyz=(BODY_W / 2.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )

    # ------------------------------------------------------------------ axle
    # Short horizontal axle boss on the right side, rigidly part of the body.
    body.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
        origin=Origin(
            xyz=(0.0, AXLE_CENTER_Y, AXLE_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="bright_steel",
        name="axle_shaft",
    )

    # ------------------------------------------------- lever
    # The lever part frame sits at the axle center; its geometry extends +X.
    lever = model.part("side_lever")

    # Lever bar - flat rectangular bar extending forward, overlapping the hub
    # for geometric connectivity (both visuals are in the same part).
    hub_radius = AXLE_RADIUS + 0.003
    bar_start = 0.005  # inside hub radius for connectivity
    bar_len = LEVER_LENGTH - bar_start
    lever.visual(
        Box((bar_len, LEVER_W, LEVER_H)),
        origin=Origin(xyz=(bar_start + bar_len / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="lever_bar",
    )

    # Rounded tip cap
    lever.visual(
        Cylinder(radius=LEVER_TIP_RADIUS, length=LEVER_W),
        origin=Origin(
            xyz=(LEVER_LENGTH, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="bright_steel",
        name="lever_tip",
    )

    # Lever hub (wraps around the axle)
    lever.visual(
        Cylinder(radius=AXLE_RADIUS + 0.003, length=LEVER_W),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="lever_hub",
    )

    model.articulation(
        "lever_lift",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, AXLE_CENTER_Y, AXLE_CENTER_Z)),
        # -Y axis so positive q lifts the forward (+X) lever tip upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-LIFT_RANGE, upper=LIFT_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    lever = object_model.get_part("side_lever")
    lift = object_model.get_articulation("lever_lift")

    body_col = body.get_visual("body_column")
    gasket = body.get_visual("base_gasket")
    spout = body.get_visual("spout_channel")
    lever_bar = lever.get_visual("lever_bar")

    # Lever hub and bar overlap the axle shaft (intentional: hub wraps the axle,
    # bar is nested inside the hub for geometric connectivity)
    ctx.allow_overlap(
        lever,
        body,
        elem_a="lever_hub",
        elem_b="axle_shaft",
        reason="lever hub captures the axle shaft end for rotation",
    )
    ctx.allow_overlap(
        lever,
        body,
        elem_a="lever_bar",
        elem_b="axle_shaft",
        reason="lever bar root is nested inside the hub that wraps the axle shaft",
    )

    # --- squared body geometry -------------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-3,
        f"gasket base must sit near z=0, got {body_aabb}",
    )
    ctx.check(
        "body_height_about_0p20",
        body_aabb is not None and 0.195 < body_aabb[1][2] < 0.210,
        f"body top should be ~0.205 m up (gasket + body), got {body_aabb}",
    )

    # Squared body: check the column element only (spout inflates the full-part AABB)
    col_aabb = ctx.part_element_world_aabb(body, elem=body_col)
    col_dx = col_aabb[1][0] - col_aabb[0][0] if col_aabb else 0
    col_dy = col_aabb[1][1] - col_aabb[0][1] if col_aabb else 0
    ctx.check(
        "body_is_squared",
        col_aabb is not None and 0.044 < col_dx < 0.056 and 0.044 < col_dy < 0.056,
        f"body column cross-section should be ~0.050x0.050 squared, got dx={col_dx:.4f} dy={col_dy:.4f}",
    )

    # --- oval base gasket ------------------------------------------------
    gasket_aabb = ctx.part_element_world_aabb(body, elem=gasket)
    ctx.check(
        "gasket_at_base",
        gasket_aabb is not None and gasket_aabb[0][2] < 0.001,
        f"oval gasket should sit at z=0, got {gasket_aabb}",
    )
    gasket_dx = gasket_aabb[1][0] - gasket_aabb[0][0] if gasket_aabb else 0
    gasket_dy = gasket_aabb[1][1] - gasket_aabb[0][1] if gasket_aabb else 0
    ctx.check(
        "gasket_is_oval",
        gasket_aabb is not None and gasket_dx > gasket_dy + 0.005,
        f"gasket should be wider in X than Y (oval), got dx={gasket_dx:.4f} dy={gasket_dy:.4f}",
    )

    # --- hollow spout outlet ---------------------------------------------
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > BODY_W / 2.0 + 0.100,
        f"spout should project >0.10 m forward from body, got {spout_aabb}",
    )
    ctx.check(
        "spout_tip_droops",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_ROOT_Z - 0.020,
        f"spout tip should drop below root line, got {spout_aabb}",
    )

    # --- lever on side ---------------------------------------------------
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever_on_right_side",
        lever_aabb is not None and lever_aabb[0][1] < -(BODY_D / 2.0),
        f"lever should be on the -Y side of the body, got {lever_aabb}",
    )

    # --- joint plan ------------------------------------------------------
    ctx.check(
        "lift_joint_exists",
        lift is not None,
        "lever_lift articulation must exist",
    )
    ctx.check(
        "lift_axis_sideways",
        abs(lift.axis[1]) == 1.0 and lift.axis[0] == 0.0 and lift.axis[2] == 0.0,
        f"lift must rotate about horizontal sideways axis, got {lift.axis}",
    )
    ctx.check(
        "lift_range_pm40deg",
        lift.motion_limits is not None
        and abs(lift.motion_limits.lower + LIFT_RANGE) < 1e-6
        and abs(lift.motion_limits.upper - LIFT_RANGE) < 1e-6,
        "lift range must be -40..+40 deg",
    )

    # --- motion proof ----------------------------------------------------
    # At rest, lever bar should be roughly horizontal
    rest_bar_aabb = ctx.part_element_world_aabb(lever, elem=lever_bar)
    ctx.check(
        "lever_bar_horizontal_at_rest",
        rest_bar_aabb is not None
        and abs(rest_bar_aabb[1][2] - rest_bar_aabb[0][2]) < 0.020,
        f"lever bar should be roughly horizontal at rest, got {rest_bar_aabb}",
    )

    # Lift up: forward lever tip sweeps upward
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(lever, elem=lever_bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > AXLE_CENTER_Z + 0.040,
            f"at +40 deg the lever tip should rise significantly, got {up_aabb}",
        )

    # Lift down: tip sweeps downward
    with ctx.pose({lift: -LIFT_RANGE}):
        down_aabb = ctx.part_element_world_aabb(lever, elem=lever_bar)
        ctx.check(
            "lift_down_lowers_lever_tip",
            down_aabb is not None and down_aabb[0][2] < AXLE_CENTER_Z - 0.040,
            f"at -40 deg the lever tip should drop significantly, got {down_aabb}",
        )

    # Prove the joint is non-fixed (range > 0)
    ctx.check(
        "joint_is_non_fixed",
        lift.motion_limits is not None
        and (lift.motion_limits.upper - lift.motion_limits.lower) > 0.1,
        "lever_lift must be a non-fixed joint with meaningful range",
    )

    # Contact: lever hub seats on the axle boss
    ctx.expect_contact(body, lever, elem_a="axle_shaft", elem_b="lever_hub",
                       name="lever_seats_on_axle")
    # Bar root is within hub overlap zone
    ctx.expect_overlap(lever, body, axes="xy",
                       elem_a="lever_bar", elem_b="axle_shaft",
                       min_overlap=0.001,
                       name="bar_root_within_hub_zone")

    return ctx.report()


object_model = build_object_model()
