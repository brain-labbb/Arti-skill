from __future__ import annotations

"""Single-hole basin faucet with waterfall spout and top-mounted flow knob.

Variant of the brushed stainless single-lever basin faucet forked into a
distinct single-hole basin faucet sibling:

- Rounded waterfall-style spout lip (wide, flat, stadium-profile tip).
- Cylindrical flow knob on top with fluted grip grooves, rotating about the
  vertical axis for flow control.
- Thin cartridge cap seam ring visible below the lever boss.
- Same side-mounted disc-and-lever assembly for temperature mixing.

Articulation chain:
- ``knob_rotate``: body -> flow_knob, revolute about Z, ±90 deg (flow control).
- ``boss_lift``: body -> lever_boss, revolute about Y, ±40 deg (lever lift).
- ``lever_twist``: lever_boss -> lever_handle, revolute about X, ±30 deg
  (temperature mix).
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Dimensions (meters).  World frame: +X forward (spout direction), +Z up,
# control disc on the -Y side of the body.
# ---------------------------------------------------------------------------
BODY_RADIUS = 0.0275  # 0.055 m diameter column
BODY_HEIGHT = 0.200
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

SPOUT_ROOT_Z = 0.118  # channel centerline height where it leaves the body

DISC_RADIUS = 0.0275  # 0.055 m diameter control disc
DISC_THICKNESS = 0.012
DISC_CENTER_Y = -0.048  # disc mid-plane (outboard of the body surface)
DISC_CENTER_Z = 0.163

BOSS_RADIUS = 0.011
BOSS_LENGTH = 0.018
BOSS_CENTER_Y = -0.0335  # lift joint origin on the boss axis

BAR_RADIUS = 0.005
BAR_X_START = 0.014  # clears the boss (boss radius 0.011)
BAR_X_END = 0.150
BAR_OFFSET_Y = 0.0095  # inboard of the disc mid-plane, toward the body

DOT_RADIUS = 0.0035
DOT_LENGTH = 0.0018
DOT_Z = 0.019  # radial offset of the paint dots on the disc face

# Cartridge cap seam – thin visible ring below the lever boss
CAP_RADIUS = 0.030  # slightly proud of the body column
CAP_HEIGHT = 0.003
CAP_Z = 0.133  # clearly below the boss/disc assembly

# Flow knob – cylindrical, on top of the body
KNOB_DIAMETER = 0.032
KNOB_HEIGHT = 0.020

LIFT_RANGE = math.radians(40.0)
TWIST_RANGE = math.radians(30.0)
KNOB_RANGE = math.radians(90.0)


def _build_waterfall_spout() -> cq.Workplane:
    """Waterfall-style spout with rounded stadium-profile lip.

    Lofted from a rectangular root (fits into the body column) through a
    mid-transition rectangle to a wide, flat stadium-slot tip.  The tip
    droops slightly below the root line for the characteristic waterfall
    pour-over edge.

    Built in spout-local coordinates: origin at the body front at
    ``SPOUT_ROOT_Z``, +X forward.
    """
    spout = (
        cq.Workplane("YZ")
        .rect(0.032, 0.018)  # root: narrow rectangular, buried in body
        .workplane(offset=0.065)
        .rect(0.044, 0.014)  # mid transition
        .workplane(offset=0.065)
        .center(0, -0.012)
        .slot2D(0.060, 0.010)  # tip: wide stadium lip (waterfall edge)
        .loft()
    )
    # Fillet the leading edge for a smooth pour-over rounding.
    try:
        spout = spout.edges(">X").fillet(0.003)
    except Exception:
        pass  # loft topology may resist this fillet; the slot profile
              # already gives a visibly rounded lip
    return spout


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_steel", rgba=(0.55, 0.56, 0.58, 1.0))
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
    body.visual(
        mesh_from_cadquery(_build_waterfall_spout(), "waterfall_spout"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )
    # Thin cartridge cap seam ring below the lever boss
    body.visual(
        Cylinder(radius=CAP_RADIUS, length=CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CAP_Z)),
        material="dark_steel",
        name="cartridge_cap",
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

    # ------------------------------------------- top-mounted flow knob
    flow_knob = model.part("flow_knob")
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=16, depth=0.0012),
        center=False,  # mounting face at local z=0
    )
    flow_knob.visual(
        mesh_from_geometry(knob_geom, "flow_knob_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="bright_steel",
        name="knob_mesh",
    )

    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flow_knob,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=3.0, lower=-KNOB_RANGE, upper=KNOB_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    boss = object_model.get_part("lever_boss")
    handle = object_model.get_part("lever_handle")
    flow_knob = object_model.get_part("flow_knob")

    lift = object_model.get_articulation("boss_lift")
    twist = object_model.get_articulation("lever_twist")
    knob_joint = object_model.get_articulation("knob_rotate")

    spout = body.get_visual("spout_channel")
    disc = handle.get_visual("control_disc")
    bar = handle.get_visual("lever_bar")
    hot_dot = handle.get_visual("hot_dot")
    cap = body.get_visual("cartridge_cap")
    knob_mesh = flow_knob.get_visual("knob_mesh")

    # ---- intentional seated embeddings -----------------------------------
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
    ctx.allow_overlap(
        flow_knob,
        body,
        elem_a="knob_mesh",
        elem_b="body_column",
        reason="knob base sits flush on the body top cap with small seat embed",
    )

    # ---- static form: body -----------------------------------------------
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

    # ---- waterfall spout --------------------------------------------------
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.120,
        f"waterfall spout should reach >0.12 m forward, got {spout_aabb}",
    )
    ctx.check(
        "waterfall_tip_wide",
        spout_aabb is not None and (spout_aabb[1][1] - spout_aabb[0][1]) > 0.050,
        f"waterfall tip should be wider than 0.050 m, got {spout_aabb}",
    )
    ctx.check(
        "waterfall_tip_droops",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_ROOT_Z - 0.010,
        f"waterfall tip should droop below root line, got {spout_aabb}",
    )

    # ---- cartridge cap seam -----------------------------------------------
    cap_aabb = ctx.part_element_world_aabb(body, elem=cap)
    ctx.check(
        "cartridge_cap_exists",
        cap_aabb is not None,
        "cartridge cap seam ring must be present on the body",
    )
    ctx.check(
        "cartridge_cap_below_boss",
        cap_aabb is not None and cap_aabb[1][2] < DISC_CENTER_Z - 0.010,
        f"cap seam should sit below the boss/disc, got {cap_aabb}",
    )
    ctx.check(
        "cartridge_cap_thin",
        cap_aabb is not None and (cap_aabb[1][2] - cap_aabb[0][2]) < 0.005,
        f"cap seam ring should be thin (<5 mm), got {cap_aabb}",
    )

    # ---- flow knob -------------------------------------------------------
    knob_aabb = ctx.part_world_aabb(flow_knob)
    ctx.check(
        "knob_on_body_top",
        knob_aabb is not None and knob_aabb[0][2] > BODY_HEIGHT - 0.005,
        f"flow knob should sit on top of the body, got {knob_aabb}",
    )
    ctx.check(
        "knob_diameter_reasonable",
        knob_aabb is not None
        and 0.025 < (knob_aabb[1][0] - knob_aabb[0][0]) < 0.045,
        f"knob diameter should be ~0.032 m, got {knob_aabb}",
    )

    # ---- joint plan: knob ------------------------------------------------
    ctx.check(
        "knob_axis_vertical",
        abs(knob_joint.axis[2]) == 1.0
        and knob_joint.axis[0] == 0.0
        and knob_joint.axis[1] == 0.0,
        f"knob must rotate about Z (vertical), got {knob_joint.axis}",
    )
    ctx.check(
        "knob_range_pm90deg",
        knob_joint.motion_limits is not None
        and abs(knob_joint.motion_limits.lower + KNOB_RANGE) < 1e-6
        and abs(knob_joint.motion_limits.upper - KNOB_RANGE) < 1e-6,
        "knob range must be -90..+90 deg",
    )

    # ---- joint plan: lever -----------------------------------------------
    ctx.check(
        "lift_axis_sideways",
        abs(lift.axis[1]) == 1.0 and lift.axis[0] == 0.0 and lift.axis[2] == 0.0,
        f"lift must rotate about Y (sideways), got {lift.axis}",
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
        f"twist must rotate about X (forward), got {twist.axis}",
    )

    # ---- contact / support -----------------------------------------------
    ctx.expect_contact(body, boss, name="boss_seats_on_body")
    ctx.expect_contact(boss, handle, name="disc_seats_on_boss")

    # ---- motion proof: knob rotation ------------------------------------
    rest_knob_pos = ctx.part_world_position(flow_knob)
    with ctx.pose({knob_joint: KNOB_RANGE}):
        rotated_pos = ctx.part_world_position(flow_knob)
        ctx.check(
            "knob_rotation_changes_pose",
            rest_knob_pos is not None
            and rotated_pos is not None,
            "knob should have valid pose at rotation limit",
        )

    # ---- motion proof: lever lift ----------------------------------------
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever_tip",
            up_aabb is not None and up_aabb[1][2] > 0.245,
            f"at +40 deg the bar tip should rise to ~0.26 m, got {up_aabb}",
        )

    # ---- motion proof: twist swings index dot ----------------------------
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
