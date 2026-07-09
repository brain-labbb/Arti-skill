from __future__ import annotations

"""Single-hole basin faucet variant with offset side lever housing and top flow knob.

A compact single-hole basin faucet (~0.20 m tall, ~0.055 m body diameter) on a
round base flange. The vertical cylindrical body carries:
- A flat-bottomed open-channel spout projecting forward and slightly down ~0.13 m.
- An offset side lever housing (short cylindrical boss on the right side near
  mid-height) that reads as the single-lever cartridge housing.
- A cylindrical flow knob on top that rotates about the vertical axis for flow
  control, with subtle fluted grip grooves on the outer surface.

Articulation:
- ``knob_turn``: revolute about the vertical (Z) axis through the top knob,
  -90..+90 deg; positive q opens flow (rotates the indicator forward).
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobBore,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Dimensions (meters). World frame: +X forward (spout direction), +Z up.
# ----------------------------------------------------------------------------
BODY_RADIUS = 0.0275  # 0.055 m diameter column
BODY_HEIGHT = 0.200
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

SPOUT_ROOT_Z = 0.118
SPOUT_OUTER_W = 0.034
SPOUT_OUTER_H = 0.022
SPOUT_WALL = 0.005
SPOUT_FLOOR = 0.006

# Offset side lever housing (cylindrical boss on the right side)
HOUSING_RADIUS = 0.013
HOUSING_LENGTH = 0.025
HOUSING_CENTER_Y = -0.026  # inner face slightly inside body wall for seated contact
HOUSING_CENTER_Z = 0.105   # mid-height of body

# Top flow knob
KNOB_DIAMETER = 0.036
KNOB_HEIGHT = 0.020
KNOB_CENTER_Z = BODY_HEIGHT + KNOB_HEIGHT / 2.0  # sits on body top

# Knob shaft (connects knob to body top)
SHAFT_RADIUS = 0.006
SHAFT_LENGTH = 0.010

KNOB_RANGE = math.radians(90.0)


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


def _build_housing() -> cq.Workplane:
    """Offset side lever housing: a short rounded cylinder with a cap detail."""
    housing = (
        cq.Workplane("XY")
        .circle(HOUSING_RADIUS)
        .extrude(HOUSING_LENGTH)
    )
    # Add a small ring/cap at the outer end for visual detail
    cap = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, HOUSING_LENGTH))
        .circle(HOUSING_RADIUS + 0.002)
        .extrude(0.003)
    )
    return housing.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_steel", rgba=(0.55, 0.56, 0.58, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    # Base flange (single deck penetration)
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )
    # Main column
    column_len = BODY_HEIGHT - 0.010
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, 0.010 + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    # Flat top cap
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.002)),
        material="brushed_steel",
        name="body_top_cap",
    )
    # Channel spout
    body.visual(
        mesh_from_cadquery(_build_spout(), "spout_channel"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )
    # Offset side lever housing (on -Y side, mid-height)
    body.visual(
        mesh_from_cadquery(_build_housing(), "side_housing"),
        origin=Origin(
            xyz=(0.0, HOUSING_CENTER_Y, HOUSING_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="bright_steel",
        name="side_housing",
    )
    # Knob shaft stub on top (connects body to knob)
    body.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + SHAFT_LENGTH / 2.0)),
        material="dark_steel",
        name="knob_shaft",
    )

    # ---------------------------------------------------------- flow knob
    knob = model.part("flow_knob")
    # Cylindrical knob with fluted grip grooves
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=16, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        bore=KnobBore(style="round", diameter=SHAFT_RADIUS * 2.0),
        center=True,
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "flow_knob_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="bright_steel",
        name="knob_shell",
    )
    # Pointer tab: small radial protrusion on the knob edge that makes
    # rotation visible. Points forward (+X) at rest.
    tab_w = 0.004
    tab_h = KNOB_HEIGHT * 0.6
    tab_len = 0.006
    knob.visual(
        Cylinder(radius=tab_len / 2.0, length=tab_h),
        origin=Origin(
            xyz=(KNOB_DIAMETER / 2.0 + tab_len / 2.0 - 0.001, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
        ),
        material="dark_steel",
        name="knob_pointer",
    )

    # Knob sits on top of the body at the shaft position
    model.articulation(
        "knob_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + SHAFT_LENGTH + KNOB_HEIGHT / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=-KNOB_RANGE, upper=KNOB_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    knob = object_model.get_part("flow_knob")
    turn = object_model.get_articulation("knob_turn")
    spout = body.get_visual("spout_channel")
    housing = body.get_visual("side_housing")
    knob_shell = knob.get_visual("knob_shell")
    flange = body.get_visual("base_flange")

    # Intentional overlap: knob shaft is captured inside the knob bore
    ctx.allow_overlap(
        body,
        knob,
        elem_a="knob_shaft",
        elem_b="knob_shell",
        reason="knob shaft stub is captured inside the knob bore for rotational coupling",
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
        aabb is not None and 0.195 < aabb[1][2] < 0.215,
        f"body top should be ~0.20 m up, got {aabb}",
    )

    # Spout projects forward
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.145,
        f"channel spout should reach ~0.155 m forward, got {spout_aabb}",
    )
    ctx.check(
        "spout_tip_droops",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_ROOT_Z - 0.035,
        f"curved tip should drop below root line, got {spout_aabb}",
    )

    # Side housing is offset from body center (on -Y side)
    housing_aabb = ctx.part_element_world_aabb(body, elem=housing)
    ctx.check(
        "side_housing_offset",
        housing_aabb is not None and housing_aabb[0][1] < -BODY_RADIUS - 0.005,
        f"side housing must be offset outboard of body, got {housing_aabb}",
    )
    ctx.check(
        "side_housing_midheight",
        housing_aabb is not None
        and 0.060 < (housing_aabb[0][2] + housing_aabb[1][2]) / 2.0 < 0.150,
        f"side housing should be near mid-height, got {housing_aabb}",
    )

    # Single deck penetration: flange is the only base element touching z=0
    flange_aabb = ctx.part_element_world_aabb(body, elem=flange)
    ctx.check(
        "single_deck_flange",
        flange_aabb is not None and abs(flange_aabb[0][2]) < 1e-6,
        f"base flange should sit at deck level, got {flange_aabb}",
    )

    # Knob sits on top of body
    knob_aabb = ctx.part_element_world_aabb(knob, elem=knob_shell)
    ctx.check(
        "knob_on_top",
        knob_aabb is not None and knob_aabb[0][2] > BODY_HEIGHT - 0.005,
        f"knob should sit on body top (~0.20 m), got {knob_aabb}",
    )
    ctx.check(
        "knob_diameter_about_0p036",
        knob_aabb is not None
        and abs((knob_aabb[1][0] - knob_aabb[0][0]) - KNOB_DIAMETER) < 0.004,
        f"knob should be ~0.036 m across, got {knob_aabb}",
    )

    # Knob is centered over body in XY
    ctx.expect_overlap(
        knob,
        body,
        axes="xy",
        min_overlap=0.010,
        elem_a=knob_shell,
        elem_b=body.get_visual("body_column"),
        name="knob_centered_over_body",
    )

    # --- joint plan --------------------------------------------------------
    ctx.check(
        "turn_axis_vertical",
        abs(turn.axis[2]) == 1.0 and turn.axis[0] == 0.0 and turn.axis[1] == 0.0,
        f"knob must rotate about vertical Z axis, got {turn.axis}",
    )
    ctx.check(
        "turn_is_revolute",
        turn.articulation_type == ArticulationType.REVOLUTE,
        f"knob joint must be revolute, got {turn.articulation_type}",
    )
    ctx.check(
        "turn_range_pm90deg",
        turn.motion_limits is not None
        and abs(turn.motion_limits.lower + KNOB_RANGE) < 1e-6
        and abs(turn.motion_limits.upper - KNOB_RANGE) < 1e-6,
        "knob range must be -90..+90 deg",
    )

    pointer = knob.get_visual("knob_pointer")

    # --- motion proof ------------------------------------------------------
    # The pointer tab should rotate when the knob turns.
    # At rest it points forward (+X). At +90 deg it should swing to +Y.
    rest_ptr = ctx.part_element_world_aabb(knob, elem=pointer)
    with ctx.pose({turn: KNOB_RANGE}):
        turned_ptr = ctx.part_element_world_aabb(knob, elem=pointer)
        ctx.check(
            "knob_rotation_moves_pointer",
            rest_ptr is not None
            and turned_ptr is not None
            and (
                abs(rest_ptr[0][0] - turned_ptr[0][0]) > 0.003
                or abs(rest_ptr[0][1] - turned_ptr[0][1]) > 0.003
            ),
            f"pointer should swing when knob rotates: rest={rest_ptr}, turned={turned_ptr}",
        )

    # Knob stays at same height when rotated (no vertical drift)
    rest_shell = ctx.part_element_world_aabb(knob, elem=knob_shell)
    with ctx.pose({turn: KNOB_RANGE}):
        turned_z = ctx.part_element_world_aabb(knob, elem=knob_shell)
        ctx.check(
            "knob_no_vertical_drift",
            rest_shell is not None
            and turned_z is not None
            and abs(rest_shell[0][2] - turned_z[0][2]) < 0.001,
            f"knob should not drift vertically when rotated: rest={rest_shell}, turned={turned_z}",
        )

    # Contact: knob sits on or very near the shaft top
    ctx.expect_contact(
        body,
        knob,
        elem_a="knob_shaft",
        elem_b=knob_shell,
        name="knob_seats_on_shaft",
    )

    return ctx.report()


object_model = build_object_model()
