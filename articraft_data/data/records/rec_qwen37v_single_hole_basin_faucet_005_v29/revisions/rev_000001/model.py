from __future__ import annotations

"""Single-hole basin faucet variant with detachable-looking spout and top lever.

A vertical cylindrical body (~0.20 m tall, ~0.055 m diameter) on a slightly
wider round base flange. The spout projects forward and slightly down ~0.13 m
from the front of the body, with a visible collar seam at the body junction
(detachable-look) and a real hollow outlet at the mouth. A separate circular
aerator insert sits recessed in the spout mouth.

A top-mounted lever assembly sits on a dome cap above the body:
- ``lever_swing``: revolute about the vertical (Z) axis for side-to-side
  temperature selection, -30..+30 deg.
- ``lever_tilt``: revolute about the horizontal sideways (Y) axis for
  flow control (up = on), -40..+40 deg.
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
# Dimensions (meters). World frame: +X forward (spout direction), +Z up.
# ----------------------------------------------------------------------------
BODY_RADIUS = 0.0275       # 0.055 m diameter column
BODY_HEIGHT = 0.200
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

SPOUT_ROOT_Z = 0.118       # centerline height where spout leaves body
SPOUT_OUTER_W = 0.032
SPOUT_OUTER_H = 0.022
SPOUT_WALL = 0.004
SPOUT_LENGTH = 0.130       # approximate horizontal reach

# Collar seam at spout-body junction
COLLAR_RADIUS = 0.022
COLLAR_WIDTH = 0.006       # axial thickness of collar ring
COLLAR_GROOVE = 0.002      # depth of visible groove

# Hollow outlet at spout mouth
OUTLET_OUTER_R = 0.012
OUTLET_INNER_R = 0.008
OUTLET_DEPTH = 0.010

# Aerator insert
AERATOR_RADIUS = OUTLET_INNER_R  # matches outlet bore for press-fit contact
AERATOR_THICKNESS = 0.004

# Top lever assembly
DOME_RADIUS = 0.018
DOME_HEIGHT = 0.012
PIVOT_Z = BODY_HEIGHT + DOME_HEIGHT  # top of dome

LEVER_ARM_RADIUS = 0.005
LEVER_ARM_LENGTH = 0.120
LEVER_BASE_RADIUS = 0.010
LEVER_BASE_HEIGHT = 0.008

SWING_RANGE = math.radians(30.0)
TILT_RANGE = math.radians(40.0)


def _build_spout_channel() -> cq.Workplane:
    """Build the main spout channel as a swept U-profile.

    Built in spout-local coordinates: origin at body exit, +X forward.
    """
    path = cq.Workplane("XZ").spline(
        [
            (0.000, 0.000),
            (0.050, -0.003),
            (0.090, -0.010),
            (0.120, -0.022),
            (0.135, -0.038),
        ]
    )
    hw = SPOUT_OUTER_W / 2.0
    hh = SPOUT_OUTER_H / 2.0
    inner_hw = hw - SPOUT_WALL
    floor_v = -hh + 0.005
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
    """Build the collar seam ring (oriented along +X axis).

    A short annular cylinder centered at origin, axis along X.
    """
    # Build on YZ plane so the cylinder axis is along X
    collar = (
        cq.Workplane("YZ")
        .circle(COLLAR_RADIUS)
        .circle(COLLAR_RADIUS - COLLAR_WIDTH)
        .extrude(COLLAR_WIDTH)
    )
    return collar


def _build_outlet() -> cq.Workplane:
    """Build the hollow outlet ring at spout tip (oriented downward, -Z).

    An annular cylinder at the spout mouth facing downward.
    """
    tip_x = 0.135
    tip_z = -0.038
    # Build annular cylinder along Z, then position it
    outlet = (
        cq.Workplane("XY")
        .workplane(offset=tip_z)
        .transformed(offset=(tip_x, 0, 0))
        .circle(OUTLET_OUTER_R)
        .circle(OUTLET_INNER_R)
        .extrude(-OUTLET_DEPTH)  # extrude downward
    )
    return outlet


def _build_aerator() -> cq.Workplane:
    """Build a circular aerator insert at local origin (axis along Z).

    A thin annular disc with cross-bar mesh pattern, centered at origin.
    """
    # Outer disc with inner hole (annular)
    disc = (
        cq.Workplane("XY")
        .circle(AERATOR_RADIUS)
        .circle(AERATOR_RADIUS - 0.002)
        .extrude(AERATOR_THICKNESS)
    )
    # Add a cross-bar pattern inside the ring for the aerator look
    bar_w = 0.001
    bar_len = 2.0 * (AERATOR_RADIUS - 0.002)
    bar_h = AERATOR_THICKNESS * 0.8
    hbar = (
        cq.Workplane("XY")
        .rect(bar_len, bar_w)
        .extrude(bar_h)
    )
    vbar = (
        cq.Workplane("XY")
        .rect(bar_w, bar_len)
        .extrude(bar_h)
    )
    disc = disc.union(hbar).union(vbar)
    return disc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_groove", rgba=(0.35, 0.36, 0.38, 1.0))
    model.material("aerator_mesh", rgba=(0.60, 0.62, 0.65, 1.0))

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
    # Flat top cap
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.0015)),
        material="brushed_steel",
        name="body_cap",
    )

    # Spout assembly (channel + collar + outlet)
    body.visual(
        mesh_from_cadquery(_build_spout_channel(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )
    # Collar seam ring at spout-body junction
    body.visual(
        mesh_from_cadquery(_build_collar(), "spout_collar"),
        origin=Origin(xyz=(-COLLAR_WIDTH / 2.0, 0.0, SPOUT_ROOT_Z)),
        material="dark_groove",
        name="spout_collar",
    )
    # Hollow outlet ring at spout mouth (already positioned in spout-local coords)
    body.visual(
        mesh_from_cadquery(_build_outlet(), "spout_outlet"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="bright_steel",
        name="spout_outlet",
    )

    # ---------------------------------------------------------------- aerator
    aerator = model.part("aerator")
    aerator_geom = _build_aerator()
    # Place aerator at the bottom of the outlet bore
    # Outlet bore bottom in world: z = SPOUT_ROOT_Z + (-0.038) - OUTLET_DEPTH = 0.070
    aerator_center_x = 0.135
    aerator_center_z = SPOUT_ROOT_Z - 0.038 - OUTLET_DEPTH
    aerator.visual(
        mesh_from_cadquery(aerator_geom, "aerator_disc"),
        origin=Origin(xyz=(aerator_center_x, 0.0, aerator_center_z)),
        material="aerator_mesh",
        name="aerator_disc",
    )

    # Fixed articulation: aerator is seated in the spout outlet (no motion)
    model.articulation(
        "aerator_seat",
        ArticulationType.FIXED,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---------------------------------------------------------- lever pivot
    # Dome cap on top of body, rotates about Z for side-to-side swing
    lever_pivot = model.part("lever_pivot")
    lever_pivot.visual(
        Cylinder(radius=DOME_RADIUS, length=DOME_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, DOME_HEIGHT / 2.0)),
        material="brushed_steel",
        name="pivot_dome",
    )
    # Small base ring on the dome
    lever_pivot.visual(
        Cylinder(radius=LEVER_BASE_RADIUS, length=LEVER_BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, DOME_HEIGHT + LEVER_BASE_HEIGHT / 2.0)),
        material="bright_steel",
        name="pivot_base",
    )

    model.articulation(
        "lever_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever_pivot,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.0, lower=-SWING_RANGE, upper=SWING_RANGE
        ),
    )

    # -------------------------------------------------------- lever handle
    # Lever arm extends forward from pivot base, tilts up/down about Y axis
    handle = model.part("lever_handle")
    handle.visual(
        Cylinder(radius=LEVER_ARM_RADIUS, length=LEVER_ARM_LENGTH),
        origin=Origin(
            xyz=(LEVER_ARM_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bright_steel",
        name="lever_arm",
    )
    # Small grip knob at the lever tip
    handle.visual(
        Cylinder(radius=LEVER_ARM_RADIUS * 1.8, length=0.012),
        origin=Origin(
            xyz=(LEVER_ARM_LENGTH + 0.006, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="brushed_steel",
        name="lever_grip",
    )

    model.articulation(
        "lever_tilt",
        ArticulationType.REVOLUTE,
        parent=lever_pivot,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, DOME_HEIGHT + LEVER_BASE_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-TILT_RANGE, upper=TILT_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    pivot = object_model.get_part("lever_pivot")
    handle = object_model.get_part("lever_handle")
    aerator = object_model.get_part("aerator")
    swing = object_model.get_articulation("lever_swing")
    tilt = object_model.get_articulation("lever_tilt")

    # Allow small intentional overlaps for seated parts
    ctx.allow_overlap(
        pivot,
        body,
        reason="pivot dome sits flush on top of body cap with small seating embed",
    )
    ctx.allow_overlap(
        handle,
        pivot,
        elem_a="lever_arm",
        elem_b="pivot_base",
        reason="lever arm base nests into the pivot base cylinder at the tilt joint",
    )
    ctx.allow_overlap(
        aerator,
        body,
        elem_a="aerator_disc",
        elem_b="spout_outlet",
        reason="aerator disc is recessed inside the hollow outlet bore",
    )

    # --- static form checks ------------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and abs(aabb[0][2]) < 1e-6,
        f"base flange must sit on z=0, got {aabb}",
    )
    ctx.check(
        "body_height_about_0p20",
        aabb is not None and 0.195 < aabb[1][2] < 0.215,
        f"body top should be ~0.20 m up, got {aabb}",
    )

    # Spout projects forward
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_channel")
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.12,
        f"spout should project forward >0.12 m, got {spout_aabb}",
    )

    # Collar seam exists at spout root
    collar_aabb = ctx.part_element_world_aabb(body, elem="spout_collar")
    ctx.check(
        "collar_seam_exists",
        collar_aabb is not None,
        "spout collar seam must be present",
    )
    ctx.expect_overlap(
        body, body,
        axes="x",
        elem_a="spout_collar",
        elem_b="spout_channel",
        min_overlap=0.002,
        name="collar_overlaps_spout_root",
    )

    # Hollow outlet at spout mouth
    outlet_aabb = ctx.part_element_world_aabb(body, elem="spout_outlet")
    ctx.check(
        "hollow_outlet_at_mouth",
        outlet_aabb is not None and outlet_aabb[0][0] > 0.10,
        f"outlet ring should be near spout tip (>0.10 m forward), got {outlet_aabb}",
    )

    # Aerator is separate part near spout tip
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator_near_spout_tip",
        aerator_aabb is not None and aerator_aabb[1][0] > 0.10,
        f"aerator should be near spout mouth, got {aerator_aabb}",
    )
    # Aerator is seated inside the outlet bore
    ctx.expect_overlap(
        aerator,
        body,
        axes="xy",
        elem_a="aerator_disc",
        elem_b="spout_outlet",
        min_overlap=0.005,
        name="aerator_inside_outlet_bore_xy",
    )
    ctx.expect_overlap(
        aerator,
        body,
        axes="z",
        elem_a="aerator_disc",
        elem_b="spout_outlet",
        min_overlap=0.002,
        name="aerator_inside_outlet_bore_z",
    )

    # --- joint plan --------------------------------------------------------
    ctx.check(
        "swing_axis_vertical",
        abs(swing.axis[2]) == 1.0 and swing.axis[0] == 0.0 and swing.axis[1] == 0.0,
        f"swing must rotate about vertical Z axis, got {swing.axis}",
    )
    ctx.check(
        "swing_range_pm30deg",
        swing.motion_limits is not None
        and abs(swing.motion_limits.lower + SWING_RANGE) < 1e-6
        and abs(swing.motion_limits.upper - SWING_RANGE) < 1e-6,
        "swing range must be -30..+30 deg",
    )
    ctx.check(
        "tilt_axis_sideways",
        abs(tilt.axis[1]) == 1.0 and tilt.axis[0] == 0.0 and tilt.axis[2] == 0.0,
        f"tilt must rotate about horizontal Y axis, got {tilt.axis}",
    )
    ctx.check(
        "tilt_range_pm40deg",
        tilt.motion_limits is not None
        and abs(tilt.motion_limits.lower + TILT_RANGE) < 1e-6
        and abs(tilt.motion_limits.upper - TILT_RANGE) < 1e-6,
        "tilt range must be -40..+40 deg",
    )

    # --- motion proof: tilt up raises lever tip ----------------------------
    rest_aabb = ctx.part_element_world_aabb(handle, elem="lever_arm")
    with ctx.pose({tilt: TILT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem="lever_arm")
        ctx.check(
            "tilt_up_raises_lever",
            rest_aabb is not None and up_aabb is not None
            and up_aabb[1][2] > rest_aabb[1][2] + 0.01,
            f"at +40 deg the lever arm should rise: rest_top={rest_aabb[1][2]:.4f}, up_top={up_aabb[1][2]:.4f}",
        )

    # --- motion proof: swing moves lever side-to-side ---------------------
    with ctx.pose({swing: SWING_RANGE}):
        swing_right_aabb = ctx.part_element_world_aabb(handle, elem="lever_arm")
        ctx.check(
            "swing_moves_lever_sideways",
            rest_aabb is not None and swing_right_aabb is not None
            and abs(swing_right_aabb[1][1] - rest_aabb[1][1]) > 0.01,
            f"at +30 deg swing, lever should move sideways: rest_y={rest_aabb[1][1]:.4f}, swung_y={swing_right_aabb[1][1]:.4f}",
        )

    # Lever pivot dome sits on body top
    ctx.expect_contact(body, pivot, name="pivot_seats_on_body")

    return ctx.report()


object_model = build_object_model()
