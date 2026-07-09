from __future__ import annotations

"""Single-hole basin faucet variant.

A compact single-hole basin faucet (~0.20 m tall) with:
- Vertical cylindrical column body (~0.055 m diameter) on a round base flange.
- A tube spout that curves gently downward from the column, with a real
  hollow outlet at the mouth end.
- A top-mounted lever assembly: a pivot base on top of the column carries
  a slim lever bar that tilts up/down and rotates side-to-side.

Articulation chain (body -> pivot -> lever):
- ``lever_turn``: revolute about vertical Z at the body top, ±30 deg;
  positive q swings the lever to the right (temperature mix).
- ``lever_lift``: revolute about horizontal Y at the pivot, 0..+45 deg;
  positive q tilts the lever tip upward (flow control).
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
BODY_RADIUS = 0.0275  # 0.055 m diameter column
BODY_HEIGHT = 0.200
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

SPOUT_ROOT_Z = 0.130  # height where the tube emerges from the column
SPOUT_OUTER_R = 0.012  # 24 mm outer diameter tube
SPOUT_INNER_R = 0.009  # 18 mm bore (hollow outlet)

PIVOT_RADIUS = 0.016
PIVOT_HEIGHT = 0.014
PIVOT_TOP_Z = BODY_HEIGHT  # pivot sits on top of the column

LEVER_RADIUS = 0.006  # slim cylindrical lever bar
LEVER_LENGTH = 0.120

TURN_RANGE = math.radians(30.0)
LIFT_LOWER = 0.0
LIFT_UPPER = math.radians(45.0)


def _build_spout_tube() -> cq.Workplane:
    """Hollow tube spout that curves gently downward from the body.

    Built in spout-local coordinates: the tube centerline starts at the
    origin heading +X and curves gently downward. The result is placed at
    the body front at ``SPOUT_ROOT_Z``.
    """
    # Path: gentle forward-and-down curve
    path = cq.Workplane("XZ").spline(
        [
            (0.000, 0.000),
            (0.040, -0.003),
            (0.080, -0.012),
            (0.110, -0.028),
            (0.135, -0.050),
        ]
    )
    # Outer tube
    outer_profile = cq.Workplane("YZ").circle(SPOUT_OUTER_R)
    outer_tube = outer_profile.sweep(path)

    # Inner bore (hollow channel)
    inner_profile = cq.Workplane("YZ").circle(SPOUT_INNER_R)
    inner_bore = inner_profile.sweep(path)

    # Cut bore from outer to create hollow tube with open ends
    return outer_tube.cut(inner_bore)


def _build_lever_handle() -> cq.Workplane:
    """Slim cylindrical lever bar extending forward from the pivot.

    Built in lever-local coordinates: origin at the pivot center, bar
    extends along +X (forward), slightly angled upward.
    """
    # Lever bar: slim cylinder along +X
    bar = (
        cq.Workplane("YZ")
        .circle(LEVER_RADIUS)
        .extrude(LEVER_LENGTH)
    )
    return bar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.82, 0.83, 0.85, 1.0))
    model.material("dark_bore", rgba=(0.15, 0.15, 0.17, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )
    column_len = BODY_HEIGHT - FLANGE_HEIGHT
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    # Flat top cap on the column
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.0015)),
        material="brushed_steel",
        name="column_cap",
    )
    # Tube spout curving gently downward from the column
    body.visual(
        mesh_from_cadquery(_build_spout_tube(), "spout_tube"),
        origin=Origin(xyz=(BODY_RADIUS * 0.7, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_tube",
    )

    # ----------------------------------------------------------- pivot base
    # Small cylindrical pivot dome on top of the body column. This part
    # rotates side-to-side (about Z) for temperature mix.
    pivot = model.part("lever_pivot")
    pivot.visual(
        Cylinder(radius=PIVOT_RADIUS, length=PIVOT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT / 2.0)),
        material="bright_steel",
        name="pivot_dome",
    )

    model.articulation(
        "lever_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pivot,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT)),
        # Vertical axis: positive q swings lever to the right (+Y side)
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.0, lower=-TURN_RANGE, upper=TURN_RANGE
        ),
    )

    # ---------------------------------------------------------- lever arm
    # Slim lever bar that tilts up/down from the pivot for flow control.
    lever = model.part("lever_arm")
    lever.visual(
        mesh_from_cadquery(_build_lever_handle(), "lever_bar"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -0.08, 0.0)),
        material="bright_steel",
        name="lever_bar",
    )
    # Small rounded tip cap on the lever end
    lever.visual(
        Cylinder(radius=LEVER_RADIUS * 1.3, length=0.008),
        origin=Origin(
            xyz=(LEVER_LENGTH - 0.004, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bright_steel",
        name="lever_tip",
    )

    model.articulation(
        "lever_lift",
        ArticulationType.REVOLUTE,
        parent=pivot,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        # Horizontal Y axis: positive q tilts the forward lever tip upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=LIFT_LOWER, upper=LIFT_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    pivot = object_model.get_part("lever_pivot")
    lever = object_model.get_part("lever_arm")
    turn = object_model.get_articulation("lever_turn")
    lift = object_model.get_articulation("lever_lift")

    spout = body.get_visual("spout_tube")
    bar = lever.get_visual("lever_bar")
    pivot_dome = pivot.get_visual("pivot_dome")

    # Intentional seated embeddings
    ctx.allow_overlap(
        pivot,
        body,
        reason="pivot dome seats 1 mm into the column top cap",
    )
    ctx.allow_overlap(
        lever,
        pivot,
        reason="lever bar root is captured inside the pivot dome bore",
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
        aabb is not None and 0.195 < aabb[1][2] < 0.210,
        f"body top should be ~0.20 m up, got {aabb}",
    )

    # Spout projects forward from the body
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.120,
        f"tube spout should reach at least 0.12 m forward, got {spout_aabb}",
    )
    # Spout curves gently downward: tip is below root
    ctx.check(
        "spout_curves_downward",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_ROOT_Z - 0.025,
        f"spout mouth should droop below root line, got {spout_aabb}",
    )
    # Spout tip is above the base (not drooping too far)
    ctx.check(
        "spout_tip_above_base",
        spout_aabb is not None and spout_aabb[0][2] > 0.040,
        f"spout mouth should remain above 0.04 m, got {spout_aabb}",
    )

    # Pivot sits on top of the body
    pivot_aabb = ctx.part_element_world_aabb(pivot, elem=pivot_dome)
    ctx.check(
        "pivot_on_top_of_body",
        pivot_aabb is not None and pivot_aabb[0][2] > BODY_HEIGHT - 0.005,
        f"pivot dome should sit on column top (~{BODY_HEIGHT} m), got {pivot_aabb}",
    )

    # --- hollow outlet check -----------------------------------------------
    # The spout tube should have an inner bore, making it narrower than a
    # solid cylinder of the same outer radius. We check that the spout's
    # lateral extent is consistent with a tube (not a solid rod).
    ctx.check(
        "spout_is_hollow_tube",
        spout_aabb is not None
        and (spout_aabb[1][1] - spout_aabb[0][1]) > 2.0 * SPOUT_INNER_R
        and (spout_aabb[1][1] - spout_aabb[0][1]) < 2.0 * SPOUT_OUTER_R + 0.002,
        f"spout lateral width should match tube OD (~{2*SPOUT_OUTER_R:.3f} m), "
        f"got {spout_aabb}",
    )

    # --- joint plan --------------------------------------------------------
    ctx.check(
        "turn_axis_vertical",
        abs(turn.axis[2]) == 1.0 and turn.axis[0] == 0.0 and turn.axis[1] == 0.0,
        f"turn must rotate about vertical Z axis, got {turn.axis}",
    )
    ctx.check(
        "turn_range_pm30deg",
        turn.motion_limits is not None
        and abs(turn.motion_limits.lower + TURN_RANGE) < 1e-6
        and abs(turn.motion_limits.upper - TURN_RANGE) < 1e-6,
        "turn range must be -30..+30 deg",
    )
    ctx.check(
        "lift_axis_sideways",
        abs(lift.axis[1]) == 1.0 and lift.axis[0] == 0.0 and lift.axis[2] == 0.0,
        f"lift must rotate about horizontal Y axis, got {lift.axis}",
    )
    ctx.check(
        "lift_range_0_to_45deg",
        lift.motion_limits is not None
        and abs(lift.motion_limits.lower - LIFT_LOWER) < 1e-6
        and abs(lift.motion_limits.upper - LIFT_UPPER) < 1e-6,
        "lift range must be 0..+45 deg",
    )

    # --- motion proof: lift raises lever tip -------------------------------
    rest_bar_aabb = ctx.part_element_world_aabb(lever, elem=bar)
    with ctx.pose({lift: LIFT_UPPER}):
        up_aabb = ctx.part_element_world_aabb(lever, elem=bar)
        ctx.check(
            "lift_up_raises_lever",
            rest_bar_aabb is not None
            and up_aabb is not None
            and up_aabb[1][2] > rest_bar_aabb[1][2] + 0.020,
            f"at +45 deg the lever should rise significantly: "
            f"rest_top={rest_bar_aabb[1][2] if rest_bar_aabb else None}, "
            f"up_top={up_aabb[1][2] if up_aabb else None}",
        )

    # --- motion proof: turn swings lever side-to-side ----------------------
    rest_center_y = (
        rest_bar_aabb[1][1] + rest_bar_aabb[0][1]
    ) / 2.0 if rest_bar_aabb else 0.0
    with ctx.pose({turn: TURN_RANGE}):
        turned_aabb = ctx.part_element_world_aabb(lever, elem=bar)
        turned_center_y = (
            turned_aabb[1][1] + turned_aabb[0][1]
        ) / 2.0 if turned_aabb else 0.0
        ctx.check(
            "turn_swings_lever_sideways",
            turned_aabb is not None
            and abs(turned_center_y - rest_center_y) > 0.010,
            f"at +30 deg the lever should swing sideways: "
            f"rest_y={rest_center_y:.4f}, turned_y={turned_center_y:.4f}",
        )

    # --- support connectivity -----------------------------------------------
    ctx.expect_contact(body, pivot, name="pivot_seats_on_body")
    ctx.expect_contact(pivot, lever, name="lever_seats_on_pivot")

    return ctx.report()


object_model = build_object_model()
