from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet with cross handles.

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (revolute about the column's vertical axis, -45..+45 deg),
  outlet aerator on a small downward-pivoting hinge (revolute, 0..30 deg).
- hot (left) and cold (right): valve columns topped by cross-shaped handles
  (each revolute about its column's vertical axis, -90..+90 deg),
  with visible stem collars under each handle.

All faucet surfaces matte black; tiny red/blue indicator dots on the
handle stems. Modeled at true scale in meters; deck bottom on z=0.
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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
DECK_X = 0.46  # deck slab length along the spread axis
DECK_Y = 0.18  # deck slab depth (front/back)
DECK_T = 0.02  # deck slab thickness; deck top at z = DECK_T

SPREAD_HALF = 0.15  # valve columns at x = -0.15 / +0.15 (0.30 m spread)

# Center spout piece
SPOUT_FLANGE_R = 0.042
SPOUT_FLANGE_H = 0.012
SPOUT_COL_R = 0.025
SPOUT_COL_H = 0.12  # column top = joint height above the deck surface

# Gooseneck (in the spout part frame, origin at column top)
TUBE_R = 0.0155  # gooseneck tube radius (slimmer than the column)
RISER_EMBED = 0.03  # hidden engagement into the column below the joint
RISER_TOP = 0.14  # straight riser ends here; arc starts
ARC_R = 0.062  # gooseneck arc radius (arc center at (y=ARC_R, z=RISER_TOP))
HOOK_DEG = -12.0  # arc end angle; past vertical = forward-down hook
COLLAR_R = 0.020
COLLAR_H = 0.016

# Valve pieces
VALVE_FLANGE_R = 0.036
VALVE_FLANGE_H = 0.010
VALVE_COL_R = 0.0225
VALVE_COL_H = 0.10  # column top = handle joint height above the deck surface

# Cross handle (in the handle part frame, origin at valve column top)
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.045
CROSS_ARM_R = 0.008  # cross arm radius
CROSS_ARM_LEN = 0.08  # full length of each cross arm
HUB_R = 0.013  # central hub where arms meet
HUB_H = 0.012  # hub height
DOT_R = 0.0035

# Stem collar (visible ring under each handle)
STEM_COLLAR_R = 0.018
STEM_COLLAR_H = 0.008

# Aerator
AERATOR_LEN = 0.016
AERATOR_R = 0.017
# Hinge pivot geometry
HINGE_KNUCKLE_R = 0.006
HINGE_KNUCKLE_LEN = 0.022

ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
# Unit tangent of the arc at the hook end (pointing out of the spout, downward).
_TX = math.sin(math.radians(HOOK_DEG))  # y component
_TZ = -math.cos(math.radians(HOOK_DEG))  # z component


def _gooseneck_solid() -> cq.Workplane:
    """Swept gooseneck tube: straight riser + ~192 deg forward-down arc."""
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -RISER_EMBED)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (ARC_END_Y, ARC_END_Z))
    )
    profile = cq.Workplane("XY").workplane(offset=-RISER_EMBED).circle(TUBE_R)
    return profile.sweep(path, isFrenet=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_cross_handle_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.08, 0.08, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))
    chrome_accent = model.material("chrome_accent", rgba=(0.55, 0.55, 0.55, 1.0))

    # ------------------------------------------------------------- sink deck
    sink_deck = model.part("sink_deck")
    sink_deck.visual(
        Box((DECK_X, DECK_Y, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, DECK_T / 2)),
        material=deck_stone,
        name="deck_slab",
    )

    # ----------------------------------------------------- center spout base
    spout_base = model.part("spout_base")
    spout_base.visual(
        Cylinder(radius=SPOUT_FLANGE_R, length=SPOUT_FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_FLANGE_H / 2)),
        material=matte_black,
        name="base_flange",
    )
    spout_base.visual(
        Cylinder(radius=SPOUT_COL_R, length=SPOUT_COL_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_COL_H / 2)),
        material=matte_black,
        name="base_column",
    )

    model.articulation(
        "deck_to_spout_base",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=spout_base,
        origin=Origin(xyz=(0.0, 0.0, DECK_T)),
    )

    # -------------------------------------------------------- gooseneck spout
    gooseneck_spout = model.part("gooseneck_spout")
    gooseneck_spout.visual(
        mesh_from_cadquery(_gooseneck_solid(), "gooseneck_tube"),
        material=matte_black,
        name="spout_tube",
    )
    gooseneck_spout.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H / 2)),
        material=matte_black,
        name="swivel_collar",
    )
    # Hinge knuckle at the spout tip — a small cylindrical pivot along X axis
    # at the arc end position, acting as the hinge mount.
    hinge_y = ARC_END_Y + _TX * 0.002
    hinge_z = ARC_END_Z + _TZ * 0.002
    gooseneck_spout.visual(
        Cylinder(radius=HINGE_KNUCKLE_R, length=HINGE_KNUCKLE_LEN),
        origin=Origin(
            xyz=(0.0, hinge_y, hinge_z),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=matte_black,
        name="hinge_knuckle",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=spout_base,
        child=gooseneck_spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_COL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=2.0, lower=-math.pi / 4, upper=math.pi / 4
        ),
    )

    # -------------------------------------------------------- aerator (hinged)
    aerator_part = model.part("aerator")
    # The aerator body extends from the hinge pivot along the spout tangent direction.
    # In the aerator part frame, origin is at the hinge pivot point.
    # The aerator cylinder aligns with the tangent direction (tilted by HOOK_DEG).
    aerator_part.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(
            xyz=(0.0, _TX * (AERATOR_LEN / 2), _TZ * (AERATOR_LEN / 2)),
            rpy=(math.radians(HOOK_DEG), 0.0, 0.0),
        ),
        material=matte_black,
        name="aerator_body",
    )
    # Small hinge barrel on the aerator side
    aerator_part.visual(
        Cylinder(radius=HINGE_KNUCKLE_R * 0.85, length=HINGE_KNUCKLE_LEN * 0.8),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=chrome_accent,
        name="aerator_hinge_barrel",
    )

    # Aerator hinge: revolute about X axis at the spout tip.
    # Positive rotation tilts the aerator downward (forward pitch).
    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=gooseneck_spout,
        child=aerator_part,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=1.5, lower=0.0, upper=math.radians(30)
        ),
    )

    # --------------------------------------------------- hot / cold valves
    def _valve_column(name: str) -> object:
        col = model.part(name)
        col.visual(
            Cylinder(radius=VALVE_FLANGE_R, length=VALVE_FLANGE_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_FLANGE_H / 2)),
            material=matte_black,
            name="valve_flange",
        )
        col.visual(
            Cylinder(radius=VALVE_COL_R, length=VALVE_COL_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H / 2)),
            material=matte_black,
            name="valve_body",
        )
        return col

    def _cross_handle(name: str, dot_material: object) -> object:
        handle = model.part(name)
        # Stem (seats into valve column)
        handle.visual(
            Cylinder(radius=STEM_R, length=STEM_TOP + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (STEM_TOP - STEM_EMBED) / 2)),
            material=matte_black,
            name="handle_stem",
        )
        # Visible stem collar — wider ring at the base of the stem above the column
        handle.visual(
            Cylinder(radius=STEM_COLLAR_R, length=STEM_COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_COLLAR_H / 2)),
            material=matte_black,
            name="stem_collar",
        )
        # Central hub where cross arms meet
        handle.visual(
            Cylinder(radius=HUB_R, length=HUB_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_TOP - HUB_H / 2)),
            material=matte_black,
            name="cross_hub",
        )
        # Cross arm along X axis
        handle.visual(
            Cylinder(radius=CROSS_ARM_R, length=CROSS_ARM_LEN),
            origin=Origin(xyz=(0.0, 0.0, STEM_TOP), rpy=(0.0, math.pi / 2, 0.0)),
            material=matte_black,
            name="cross_arm_x",
        )
        # Cross arm along Y axis
        handle.visual(
            Cylinder(radius=CROSS_ARM_R, length=CROSS_ARM_LEN),
            origin=Origin(xyz=(0.0, 0.0, STEM_TOP), rpy=(math.pi / 2, 0.0, 0.0)),
            material=matte_black,
            name="cross_arm_y",
        )
        # End caps on cross arms (4 ends)
        for dx, dy in [
            (CROSS_ARM_LEN / 2, 0.0),
            (-CROSS_ARM_LEN / 2, 0.0),
            (0.0, CROSS_ARM_LEN / 2),
            (0.0, -CROSS_ARM_LEN / 2),
        ]:
            handle.visual(
                Sphere(radius=CROSS_ARM_R),
                origin=Origin(xyz=(dx, dy, STEM_TOP)),
                material=matte_black,
                name=f"arm_cap_{dx:+.0f}_{dy:+.0f}",
            )
        # Tiny temperature indicator dot on the stem front
        handle.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, STEM_R - 0.0005, 0.022)),
            material=dot_material,
            name="indicator_dot",
        )
        return handle

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    hot_handle = _cross_handle("hot_handle", hot_red)
    cold_handle = _cross_handle("cold_handle", cold_blue)

    model.articulation(
        "deck_to_hot_valve",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=hot_valve_column,
        origin=Origin(xyz=(-SPREAD_HALF, 0.0, DECK_T)),
    )
    model.articulation(
        "deck_to_cold_valve",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=cold_valve_column,
        origin=Origin(xyz=(SPREAD_HALF, 0.0, DECK_T)),
    )

    for joint_name, parent, child in (
        ("hot_handle_turn", hot_valve_column, hot_handle),
        ("cold_handle_turn", cold_valve_column, cold_handle),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=parent,
            child=child,
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=2.0, lower=-math.pi / 2, upper=math.pi / 2
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("sink_deck")
    spout_base = object_model.get_part("spout_base")
    gooseneck = object_model.get_part("gooseneck_spout")
    aerator = object_model.get_part("aerator")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_handle = object_model.get_part("hot_handle")
    cold_handle = object_model.get_part("cold_handle")

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_turn = object_model.get_articulation("hot_handle_turn")
    cold_turn = object_model.get_articulation("cold_handle_turn")
    aerator_hinge = object_model.get_articulation("aerator_hinge")

    spout_tube = gooseneck.get_visual("spout_tube")
    base_column = spout_base.get_visual("base_column")

    # Intentional hidden engagements: spout riser and handle stems seat inside
    # their columns so the rotating parts read as mounted, not floating.
    ctx.allow_overlap(
        gooseneck,
        spout_base,
        elem_a=spout_tube,
        elem_b=base_column,
        reason="gooseneck riser tube seats 30 mm into the base column bore",
    )
    for handle, col in ((hot_handle, hot_col), (cold_handle, cold_col)):
        ctx.allow_overlap(
            handle,
            col,
            elem_a=handle.get_visual("handle_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="handle stem seats 15 mm into the valve cartridge bore",
        )
    # Aerator hinge barrel sits at the spout tube tip (pivot capture)
    ctx.allow_overlap(
        aerator,
        gooseneck,
        elem_a=aerator.get_visual("aerator_hinge_barrel"),
        elem_b=gooseneck.get_visual("hinge_knuckle"),
        reason="aerator hinge barrel wraps around the spout hinge knuckle pivot",
    )
    ctx.allow_overlap(
        aerator,
        gooseneck,
        elem_a=aerator.get_visual("aerator_hinge_barrel"),
        elem_b=spout_tube,
        reason="aerator hinge barrel seats against the gooseneck tube end at the pivot",
    )
    ctx.allow_overlap(
        aerator,
        gooseneck,
        elem_a=aerator.get_visual("aerator_body"),
        elem_b=gooseneck.get_visual("hinge_knuckle"),
        reason="aerator body top seats against the hinge knuckle at the pivot point",
    )
    # Proof: the hinge barrel stays near the spout tip (retained at pivot)
    ctx.expect_contact(
        aerator,
        gooseneck,
        elem_a=aerator.get_visual("aerator_hinge_barrel"),
        elem_b=gooseneck.get_visual("hinge_knuckle"),
        contact_tol=0.005,
        name="aerator hinge barrel retained at knuckle pivot",
    )

    # --- joint plan: types, axes, ranges -----------------------------------
    for joint, lower_lim, upper_lim, expected_axis in (
        (spout_swivel, -math.pi / 4, math.pi / 4, (0.0, 0.0, 1.0)),
        (hot_turn, -math.pi / 2, math.pi / 2, (0.0, 0.0, 1.0)),
        (cold_turn, -math.pi / 2, math.pi / 2, (0.0, 0.0, 1.0)),
        (aerator_hinge, 0.0, math.radians(30), (1.0, 0.0, 0.0)),
    ):
        ctx.check(
            f"{joint.name}_is_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ctx.check(
            f"{joint.name}_axis",
            tuple(joint.axis) == expected_axis,
            f"axis={joint.axis}",
        )
        ml = joint.motion_limits
        ctx.check(
            f"{joint.name}_range",
            ml is not None
            and abs(ml.lower - lower_lim) < 1e-3
            and abs(ml.upper - upper_lim) < 1e-3,
            f"lower={ml.lower} upper={ml.upper}",
        )

    # --- placement: 0.30 m spread, all three pieces seated on the deck -----
    hot_pos = ctx.part_world_position(hot_col)
    cold_pos = ctx.part_world_position(cold_col)
    spout_pos = ctx.part_world_position(spout_base)
    ctx.check(
        "widespread_0p30_spread",
        abs(hot_pos[0] + 0.15) < 1e-6
        and abs(cold_pos[0] - 0.15) < 1e-6
        and abs(spout_pos[0]) < 1e-6,
        f"hot_x={hot_pos[0]} cold_x={cold_pos[0]} spout_x={spout_pos[0]}",
    )
    for piece in (spout_base, hot_col, cold_col):
        ctx.expect_contact(piece, deck, contact_tol=1e-5)

    deck_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "deck_grounded_at_z0",
        abs(deck_aabb[0][2]) < 1e-6 and abs(deck_aabb[1][2] - DECK_T) < 1e-6,
        f"deck z {deck_aabb[0][2]}..{deck_aabb[1][2]}",
    )

    # --- gooseneck form: rises ~0.32 above deck, outlet ~0.25 above deck ---
    neck_aabb = ctx.part_world_aabb(gooseneck)
    arc_top_above_deck = neck_aabb[1][2] - DECK_T
    ctx.check(
        "gooseneck_arc_top_height",
        0.28 < arc_top_above_deck < 0.36,
        f"arc top {arc_top_above_deck:.3f} m above deck",
    )

    # --- cross handle geometry ---------------------------------------------
    for handle, col, sign in (
        (hot_handle, hot_col, -1.0),
        (cold_handle, cold_col, 1.0),
    ):
        # Cross arms: both arms visible, forming a + shape
        arm_x = handle.get_visual("cross_arm_x")
        arm_y = handle.get_visual("cross_arm_y")
        arm_x_aabb = ctx.part_element_world_aabb(handle, elem=arm_x)
        arm_y_aabb = ctx.part_element_world_aabb(handle, elem=arm_y)
        span_x = arm_x_aabb[1][0] - arm_x_aabb[0][0]
        span_y = arm_y_aabb[1][1] - arm_y_aabb[0][1]
        ctx.check(
            f"{handle.name}_cross_arm_x_spans_x",
            span_x > 0.06,
            f"arm_x span={span_x:.3f}",
        )
        ctx.check(
            f"{handle.name}_cross_arm_y_spans_y",
            span_y > 0.06,
            f"arm_y span={span_y:.3f}",
        )

        # Stem collar is visible between handle and column
        collar = handle.get_visual("stem_collar")
        collar_aabb = ctx.part_element_world_aabb(handle, elem=collar)
        collar_diam = max(
            collar_aabb[1][0] - collar_aabb[0][0],
            collar_aabb[1][1] - collar_aabb[0][1],
        )
        ctx.check(
            f"{handle.name}_stem_collar_visible",
            collar_diam > 0.030,
            f"collar diameter={collar_diam:.3f}",
        )

        # Handle clears the valve column top (only stem enters)
        ctx.expect_gap(
            handle,
            col,
            axis="z",
            positive_elem=arm_x,
            min_gap=0.02,
        )

    # Indicator dots: red on hot, blue on cold
    for handle, mat in ((hot_handle, "hot_red"), (cold_handle, "cold_blue")):
        dot = handle.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{handle.name}_dot_material", mat_name == mat, f"material={mat_name}")

    # --- aerator hinge: pivots downward ------------------------------------
    aerator_body = aerator.get_visual("aerator_body")
    # At rest (q=0): aerator follows the spout tangent (mostly downward)
    rest_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_body)
    rest_y_min = rest_aabb[0][1]

    # At max tilt (q=30 deg): aerator outlet swings further forward (+Y)
    with ctx.pose({aerator_hinge: math.radians(30)}):
        tilted_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_body)
    tilted_y_min = tilted_aabb[0][1]
    ctx.check(
        "aerator_pivots_forward_on_hinge",
        tilted_y_min > rest_y_min + 0.0005,
        f"rest_y_min={rest_y_min:.4f} tilted_y_min={tilted_y_min:.4f}",
    )

    # Aerator is connected to gooseneck via hinge (contact at pivot)
    ctx.expect_contact(
        aerator,
        gooseneck,
        contact_tol=0.005,
    )

    # --- articulation behavior: handle rotation ----------------------------
    # At q=0 the cross arms span X and Y; at q=+45 deg they rotate 45 deg.
    with ctx.pose({hot_turn: 0.0}):
        arm0 = ctx.part_element_world_aabb(hot_handle, elem=hot_handle.get_visual("cross_arm_x"))
    with ctx.pose({hot_turn: math.pi / 4}):
        arm45 = ctx.part_element_world_aabb(hot_handle, elem=hot_handle.get_visual("cross_arm_x"))
    span_x0 = arm0[1][0] - arm0[0][0]
    span_y0 = arm0[1][1] - arm0[0][1]
    span_x45 = arm45[1][0] - arm45[0][0]
    span_y45 = arm45[1][1] - arm45[0][1]
    ctx.check(
        "hot_handle_rotates_about_vertical_axis",
        span_x0 > 0.06 and span_y0 < 0.03 and span_y45 > 0.03,
        f"closed span=({span_x0:.3f},{span_y0:.3f}) turned span=({span_x45:.3f},{span_y45:.3f})",
    )

    # Spout swivel: +45 deg swings the forward outlet toward -X
    with ctx.pose({spout_swivel: math.pi / 4}):
        tip45 = ctx.part_element_world_aabb(aerator, elem=aerator_body)
    tip45_cx = 0.5 * (tip45[0][0] + tip45[1][0])
    ctx.check(
        "spout_swivels_about_column_axis",
        tip45_cx < -0.04,
        f"tip at 45deg x={tip45_cx:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
