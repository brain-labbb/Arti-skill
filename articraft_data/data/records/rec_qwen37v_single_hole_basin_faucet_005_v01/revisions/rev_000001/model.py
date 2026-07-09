from __future__ import annotations

"""Single-hole basin faucet — tall tower variant.

A tall straight tower body (~0.26 m, ~0.055 m diameter) on an oval rubber base
gasket. A short forward tubular spout projects from the upper body with a real
hollow bore outlet at the mouth. A top-mounted lever assembly carries a slim
bar that tilts up/down (lift, flow control) and swings side-to-side (swing,
temperature mix) through two revolute joints.

Articulation chain (body -> swing_ring -> lever_handle):
- ``lever_swing``: revolute about vertical (Z) axis at top of tower,
  -45..+45 deg; positive q swings the lever to the right.
- ``lever_lift``: revolute about horizontal sideways (Y) axis,
  -15..+55 deg; positive q lifts the lever tip upward.
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

# ---------------------------------------------------------------------------
# Dimensions (meters). World: +X forward (spout direction), +Z up.
# ---------------------------------------------------------------------------
BODY_RADIUS = 0.0275       # 0.055 m diameter tower column
BODY_HEIGHT = 0.260        # total tower height (above z=0)
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.010

# Oval base gasket
GASKET_RX = 0.038          # semi-axis forward/backward
GASKET_RY = 0.030          # semi-axis left/right
GASKET_HEIGHT = 0.004

# Short forward tubular spout
SPOUT_CENTER_Z = 0.210     # height where spout leaves the body
SPOUT_LENGTH = 0.065       # forward projection from root
SPOUT_OUTER_R = 0.012      # outer tube radius
SPOUT_INNER_R = 0.009      # hollow bore radius
SPOUT_PITCH = -0.06        # slight downward tilt (~3.4 deg)

# Fixed dome cap at tower top
DOME_RADIUS = 0.016
DOME_HEIGHT = 0.014

# Swing collar (annular ring around dome)
COLLAR_OUTER_R = 0.021
COLLAR_INNER_R = DOME_RADIUS + 0.0008   # slight running clearance
COLLAR_HEIGHT = 0.010

# Lever
LEVER_RADIUS = 0.005
LEVER_LENGTH = 0.080
HUB_RADIUS = 0.009
HUB_HEIGHT = 0.008

SWING_RANGE = math.radians(45.0)
LIFT_LOWER = math.radians(-15.0)
LIFT_UPPER = math.radians(55.0)


def _build_oval_gasket() -> cq.Workplane:
    """Thin oval disc gasket, extruded along +Z from origin."""
    return (
        cq.Workplane("XY")
        .ellipse(GASKET_RX, GASKET_RY)
        .extrude(GASKET_HEIGHT)
    )


def _build_hollow_spout() -> cq.Workplane:
    """Short tubular spout with a real hollow bore, along +X from origin."""
    outer = cq.Workplane("YZ").circle(SPOUT_OUTER_R).extrude(SPOUT_LENGTH)
    inner = cq.Workplane("YZ").circle(SPOUT_INNER_R).extrude(SPOUT_LENGTH)
    return outer.cut(inner)


def _build_collar() -> cq.Workplane:
    """Annular ring that wraps around the fixed dome with running clearance."""
    outer = cq.Workplane("XY").circle(COLLAR_OUTER_R).extrude(COLLAR_HEIGHT)
    bore = cq.Workplane("XY").circle(COLLAR_INNER_R).extrude(COLLAR_HEIGHT)
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))

    # ---------------------------------------------------------- faucet body
    body = model.part("faucet_body")

    # Oval base gasket at z=0
    body.visual(
        mesh_from_cadquery(_build_oval_gasket(), "oval_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="rubber_black",
        name="oval_gasket",
    )

    # Base flange transition
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, GASKET_HEIGHT + FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )

    # Tall straight tower column
    tower_base_z = GASKET_HEIGHT + FLANGE_HEIGHT
    tower_len = BODY_HEIGHT - tower_base_z
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=tower_len),
        origin=Origin(xyz=(0.0, 0.0, tower_base_z + tower_len / 2.0)),
        material="brushed_steel",
        name="tower_column",
    )

    # Short forward spout with hollow outlet — root buried 8 mm inside the
    # tower so the mesh reads as one piece with the body.
    body.visual(
        mesh_from_cadquery(_build_hollow_spout(), "spout_tube"),
        origin=Origin(
            xyz=(BODY_RADIUS - 0.008, 0.0, SPOUT_CENTER_Z),
            rpy=(0.0, SPOUT_PITCH, 0.0),
        ),
        material="brushed_steel",
        name="spout_tube",
    )

    # Fixed dome cap at tower top (the lever assembly rotates around it)
    body.visual(
        Cylinder(radius=DOME_RADIUS, length=DOME_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + DOME_HEIGHT / 2.0)),
        material="brushed_steel",
        name="top_dome",
    )

    # ---------------------------------------------------------- swing ring
    # Annular collar that sits around the dome and swings left/right.
    collar_base_z = BODY_HEIGHT + DOME_HEIGHT - COLLAR_HEIGHT
    swing_ring = model.part("swing_ring")
    swing_ring.visual(
        mesh_from_cadquery(_build_collar(), "collar_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="bright_steel",
        name="collar_ring",
    )

    model.articulation(
        "lever_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swing_ring,
        origin=Origin(xyz=(0.0, 0.0, collar_base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0,
            lower=-SWING_RANGE, upper=SWING_RANGE,
        ),
    )

    # ---------------------------------------------------------- lever handle
    handle = model.part("lever_handle")

    # Small hub that bridges the collar top to the lever bar
    handle.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT),
        origin=Origin(
            xyz=(HUB_RADIUS, 0.0, HUB_HEIGHT / 2.0),
        ),
        material="bright_steel",
        name="lever_hub",
    )

    # Slim cylindrical lever bar extending forward from the hub
    bar_len = LEVER_LENGTH
    handle.visual(
        Cylinder(radius=LEVER_RADIUS, length=bar_len),
        origin=Origin(
            xyz=(HUB_RADIUS * 2.0 + bar_len / 2.0, 0.0, HUB_HEIGHT / 2.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bright_steel",
        name="lever_bar",
    )

    model.articulation(
        "lever_lift",
        ArticulationType.REVOLUTE,
        parent=swing_ring,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_HEIGHT)),
        # -Y so positive q lifts the forward (+X) lever tip upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.0,
            lower=LIFT_LOWER, upper=LIFT_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    swing_ring = object_model.get_part("swing_ring")
    handle = object_model.get_part("lever_handle")
    swing = object_model.get_articulation("lever_swing")
    lift = object_model.get_articulation("lever_lift")

    gasket = body.get_visual("oval_gasket")
    tower = body.get_visual("tower_column")
    spout = body.get_visual("spout_tube")
    dome = body.get_visual("top_dome")
    collar = swing_ring.get_visual("collar_ring")
    lever_bar = handle.get_visual("lever_bar")
    lever_hub = handle.get_visual("lever_hub")

    # --- intentional overlap allowances ------------------------------------
    # The spout tube root is intentionally buried ~8 mm inside the body column
    # so the two read as one continuous piece.
    ctx.allow_overlap(
        body, body,
        elem_a="spout_tube", elem_b="tower_column",
        reason="spout root is intentionally embedded in the tower wall",
    )

    # --- static form -------------------------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        f"gasket base must sit on z=0, got {body_aabb}",
    )
    ctx.check(
        "body_taller_than_parent",
        body_aabb is not None and body_aabb[1][2] > 0.255,
        f"tower body top should be ~0.26+ m, got {body_aabb}",
    )

    # Oval gasket exists and is clearly oval (rx != ry)
    gasket_aabb = ctx.part_element_world_aabb(body, elem=gasket)
    ctx.check(
        "oval_gasket_present",
        gasket_aabb is not None,
        "oval base gasket must exist",
    )
    if gasket_aabb is not None:
        gasket_dx = gasket_aabb[1][0] - gasket_aabb[0][0]
        gasket_dy = gasket_aabb[1][1] - gasket_aabb[0][1]
        ctx.check(
            "gasket_is_oval_not_round",
            abs(gasket_dx - gasket_dy) > 0.005,
            f"gasket should be oval (dx={gasket_dx:.4f}, dy={gasket_dy:.4f})",
        )

    # Spout projects forward from the body
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > BODY_RADIUS + 0.040,
        f"spout should project at least 40 mm past body, got {spout_aabb}",
    )

    # Dome and collar are vertically adjacent
    dome_aabb = ctx.part_element_world_aabb(body, elem=dome)
    collar_aabb = ctx.part_element_world_aabb(swing_ring, elem=collar)
    ctx.check(
        "collar_near_dome_top",
        dome_aabb is not None and collar_aabb is not None
        and abs(collar_aabb[1][2] - dome_aabb[1][2]) < 0.002,
        f"collar top should be near dome top: dome={dome_aabb}, collar={collar_aabb}",
    )

    # --- joint plan --------------------------------------------------------
    ctx.check(
        "swing_axis_vertical",
        abs(swing.axis[2]) == 1.0 and swing.axis[0] == 0.0 and swing.axis[1] == 0.0,
        f"swing must rotate about vertical Z, got {swing.axis}",
    )
    ctx.check(
        "swing_range_pm45deg",
        swing.motion_limits is not None
        and abs(swing.motion_limits.lower + SWING_RANGE) < 1e-6
        and abs(swing.motion_limits.upper - SWING_RANGE) < 1e-6,
        "swing range must be -45..+45 deg",
    )
    ctx.check(
        "lift_axis_sideways",
        abs(lift.axis[1]) == 1.0 and lift.axis[0] == 0.0 and lift.axis[2] == 0.0,
        f"lift must rotate about horizontal Y, got {lift.axis}",
    )
    ctx.check(
        "lift_range",
        lift.motion_limits is not None
        and abs(lift.motion_limits.lower - LIFT_LOWER) < 1e-6
        and abs(lift.motion_limits.upper - LIFT_UPPER) < 1e-6,
        "lift range must be -15..+55 deg",
    )

    # --- motion proof ------------------------------------------------------
    # Lift up: lever tip rises well above the rest position.
    rest_bar_aabb = ctx.part_element_world_aabb(handle, elem=lever_bar)
    with ctx.pose({lift: LIFT_UPPER}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=lever_bar)
        ctx.check(
            "lift_raises_lever_tip",
            rest_bar_aabb is not None and up_aabb is not None
            and up_aabb[1][2] > rest_bar_aabb[1][2] + 0.020,
            f"at +55 deg the bar tip should rise above rest: rest={rest_bar_aabb}, up={up_aabb}",
        )

    # Lift down: lever tip drops below rest.
    with ctx.pose({lift: LIFT_LOWER}):
        down_aabb = ctx.part_element_world_aabb(handle, elem=lever_bar)
        ctx.check(
            "lift_lowers_lever_tip",
            rest_bar_aabb is not None and down_aabb is not None
            and down_aabb[0][2] < rest_bar_aabb[0][2] - 0.005,
            f"at -15 deg the bar tip should drop below rest: rest={rest_bar_aabb}, down={down_aabb}",
        )

    # Swing: lever moves sideways (Y) from rest.
    with ctx.pose({swing: SWING_RANGE}):
        swing_aabb = ctx.part_element_world_aabb(handle, elem=lever_bar)
        ctx.check(
            "swing_moves_lever_sideways",
            rest_bar_aabb is not None and swing_aabb is not None
            and (swing_aabb[1][1] - rest_bar_aabb[1][1]) > 0.020,
            f"at +45 deg swing, lever tip Y max should increase: rest={rest_bar_aabb}, swung={swing_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
