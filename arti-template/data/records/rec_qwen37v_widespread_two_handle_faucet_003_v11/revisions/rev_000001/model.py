from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet set (variant 11).

Three independent deck-mounted pieces on a sink deck (total spread 0.36 m):
- center: taller cylindrical base column with a swiveling gooseneck spout
  (revolute about the column's vertical axis, -45..+45 deg),
- hot (left) and cold (right): valve columns topped by T-style lever
  handles that rotate forward-back (revolute about lateral X axis,
  -90..+90 deg).  Visible stem collars sit under each handle.

All bases expose mounting pipes and hex-style nuts below the deck.
All faucet surfaces matte black; tiny red/blue indicator dots on the
handle stems.  Modeled at true scale in meters; deck bottom on z=0.
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
DECK_X = 0.50  # deck slab length along the spread axis
DECK_Y = 0.18  # deck slab depth (front/back)
DECK_T = 0.02  # deck slab thickness; deck top at z = DECK_T

SPREAD_HALF = 0.18  # valve columns at x = ±0.18  (0.36 m total spread)

# Center spout piece (taller than parent)
SPOUT_FLANGE_R = 0.042
SPOUT_FLANGE_H = 0.012
SPOUT_COL_R = 0.025
SPOUT_COL_H = 0.15  # taller column (parent was 0.12)

# Gooseneck (in the spout part frame, origin at column top)
TUBE_R = 0.0155
RISER_EMBED = 0.03
RISER_TOP = 0.17  # taller riser (parent was 0.14)
ARC_R = 0.065  # slightly larger arc (parent was 0.062)
HOOK_DEG = -12.0
COLLAR_R = 0.020
COLLAR_H = 0.016

# Valve pieces
VALVE_FLANGE_R = 0.036
VALVE_FLANGE_H = 0.010
VALVE_COL_R = 0.0225
VALVE_COL_H = 0.10

# T-lever (in the lever part frame, origin at valve column top)
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.045
BAR_R = 0.0095
BAR_LEN = 0.12
BAR_CENTER_OFF = 0.025  # bar offset along +Y so it overhangs toward user
DOT_R = 0.0035

# Stem collar (visible ring under each handle, NEW)
STEM_COLLAR_R = 0.015
STEM_COLLAR_H = 0.008

# Underside mounting hardware (NEW)
MOUNT_PIPE_R = 0.012
MOUNT_PIPE_H = 0.035
NUT_R = 0.018
NUT_H = 0.010

ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.016
AERATOR_R = 0.017
_TX = math.sin(math.radians(HOOK_DEG))
_TZ = -math.cos(math.radians(HOOK_DEG))
AERATOR_CY = ARC_END_Y + _TX * (AERATOR_LEN / 2 - 0.004)
AERATOR_CZ = ARC_END_Z + _TZ * (AERATOR_LEN / 2 - 0.004)


def _gooseneck_solid() -> cq.Workplane:
    """Swept gooseneck tube: straight riser + forward-down arc."""
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -RISER_EMBED)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (ARC_END_Y, ARC_END_Z))
    )
    profile = cq.Workplane("XY").workplane(offset=-RISER_EMBED).circle(TUBE_R)
    return profile.sweep(path, isFrenet=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.08, 0.08, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))

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
    # Mounting pipe through deck
    spout_base.visual(
        Cylinder(radius=MOUNT_PIPE_R, length=MOUNT_PIPE_H),
        origin=Origin(xyz=(0.0, 0.0, -MOUNT_PIPE_H / 2)),
        material=matte_black,
        name="mount_pipe",
    )
    # Mounting nut below deck
    spout_base.visual(
        Cylinder(radius=NUT_R, length=NUT_H),
        origin=Origin(xyz=(0.0, 0.0, -(MOUNT_PIPE_H + NUT_H / 2))),
        material=matte_black,
        name="mount_nut",
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
    # Aerator nozzle at the hook tip, aligned with the arc end tangent.
    gooseneck_spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(
            xyz=(0.0, AERATOR_CY, AERATOR_CZ),
            rpy=(math.radians(HOOK_DEG), 0.0, 0.0),
        ),
        material=matte_black,
        name="aerator",
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
        # Mounting pipe through deck
        col.visual(
            Cylinder(radius=MOUNT_PIPE_R, length=MOUNT_PIPE_H),
            origin=Origin(xyz=(0.0, 0.0, -MOUNT_PIPE_H / 2)),
            material=matte_black,
            name="mount_pipe",
        )
        # Mounting nut below deck
        col.visual(
            Cylinder(radius=NUT_R, length=NUT_H),
            origin=Origin(xyz=(0.0, 0.0, -(MOUNT_PIPE_H + NUT_H / 2))),
            material=matte_black,
            name="mount_nut",
        )
        return col

    def _t_lever(name: str, bar_off: float, dot_material: object) -> object:
        lever = model.part(name)
        lever.visual(
            Cylinder(radius=STEM_R, length=STEM_TOP + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (STEM_TOP - STEM_EMBED) / 2)),
            material=matte_black,
            name="lever_stem",
        )
        # Visible stem collar: ring at the base of the handle
        lever.visual(
            Cylinder(radius=STEM_COLLAR_R, length=STEM_COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_COLLAR_H / 2)),
            material=matte_black,
            name="stem_collar",
        )
        # Horizontal T-bar along Y (forward-back toward user)
        lever.visual(
            Cylinder(radius=BAR_R, length=BAR_LEN),
            origin=Origin(
                xyz=(0.0, bar_off, STEM_TOP), rpy=(math.pi / 2, 0.0, 0.0)
            ),
            material=matte_black,
            name="lever_bar",
        )
        for end in (-1.0, 1.0):
            lever.visual(
                Sphere(radius=BAR_R),
                origin=Origin(
                    xyz=(0.0, bar_off + end * BAR_LEN / 2, STEM_TOP)
                ),
                material=matte_black,
                name=f"bar_cap_{'front' if end > 0 else 'rear'}",
            )
        # Tiny temperature indicator dot on the side of the stem
        lever.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, STEM_R - 0.0005, 0.022)),
            material=dot_material,
            name="indicator_dot",
        )
        return lever

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    # Both levers overhang toward the user (+Y)
    hot_lever = _t_lever("hot_lever", BAR_CENTER_OFF, hot_red)
    cold_lever = _t_lever("cold_lever", BAR_CENTER_OFF, cold_blue)

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

    # Lever handles rotate forward-back about the lateral (X) axis
    for joint_name, parent_col, child_lever in (
        ("hot_lever_turn", hot_valve_column, hot_lever),
        ("cold_lever_turn", cold_valve_column, cold_lever),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=parent_col,
            child=child_lever,
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=-math.pi / 2,
                upper=math.pi / 2,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("sink_deck")
    spout_base = object_model.get_part("spout_base")
    gooseneck = object_model.get_part("gooseneck_spout")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_lever = object_model.get_part("hot_lever")
    cold_lever = object_model.get_part("cold_lever")

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_turn = object_model.get_articulation("hot_lever_turn")
    cold_turn = object_model.get_articulation("cold_lever_turn")

    spout_tube = gooseneck.get_visual("spout_tube")
    base_column = spout_base.get_visual("base_column")
    aerator = gooseneck.get_visual("aerator")

    # --- intentional hidden engagements ------------------------------------
    ctx.allow_overlap(
        gooseneck,
        spout_base,
        elem_a=spout_tube,
        elem_b=base_column,
        reason="gooseneck riser tube seats 30 mm into the base column bore",
    )
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.allow_overlap(
            lever,
            col,
            elem_a=lever.get_visual("lever_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="lever stem seats 15 mm into the valve cartridge bore",
        )
    # Mounting pipes pass through deck holes to reach underside nuts
    for base_part in (spout_base, hot_col, cold_col):
        ctx.allow_overlap(
            deck,
            base_part,
            elem_a=deck.get_visual("deck_slab"),
            elem_b=base_part.get_visual("mount_pipe"),
            reason="mounting pipe passes through deck hole to underside nut",
        )

    # --- joint plan: types, axes, ranges ---------------------------------
    # Spout swivel: vertical revolute
    ctx.check(
        "spout_swivel_is_vertical_revolute",
        str(spout_swivel.joint_type).lower().endswith("revolute")
        and tuple(spout_swivel.axis) == (0.0, 0.0, 1.0),
        f"axis={spout_swivel.axis}",
    )
    ml = spout_swivel.motion_limits
    ctx.check(
        "spout_swivel_range",
        ml is not None
        and abs(ml.lower + math.pi / 4) < 1e-6
        and abs(ml.upper - math.pi / 4) < 1e-6,
        f"lower={ml.lower} upper={ml.upper}",
    )
    # Lever handles: lateral (X-axis) revolute for forward-back motion
    for joint in (hot_turn, cold_turn):
        ctx.check(
            f"{joint.name}_is_lateral_revolute",
            str(joint.joint_type).lower().endswith("revolute")
            and tuple(joint.axis) == (1.0, 0.0, 0.0),
            f"axis={joint.axis}",
        )
        ml = joint.motion_limits
        ctx.check(
            f"{joint.name}_range",
            ml is not None
            and abs(ml.lower + math.pi / 2) < 1e-6
            and abs(ml.upper - math.pi / 2) < 1e-6,
            f"lower={ml.lower} upper={ml.upper}",
        )

    # --- placement: 0.36 m spread, all three pieces seated on deck --------
    hot_pos = ctx.part_world_position(hot_col)
    cold_pos = ctx.part_world_position(cold_col)
    spout_pos = ctx.part_world_position(spout_base)
    ctx.check(
        "widespread_0p36_spread",
        abs(hot_pos[0] + SPREAD_HALF) < 1e-6
        and abs(cold_pos[0] - SPREAD_HALF) < 1e-6
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

    # --- variant 11: stem collars present on each lever -------------------
    for lever in (hot_lever, cold_lever):
        collar = lever.get_visual("stem_collar")
        ctx.check(
            f"{lever.name}_has_stem_collar",
            collar is not None,
            "stem_collar visual missing",
        )
        collar_aabb = ctx.part_element_world_aabb(lever, elem=collar)
        stem_aabb = ctx.part_element_world_aabb(
            lever, elem=lever.get_visual("lever_stem")
        )
        collar_dx = collar_aabb[1][0] - collar_aabb[0][0]
        stem_dx = stem_aabb[1][0] - stem_aabb[0][0]
        ctx.check(
            f"{lever.name}_collar_wider_than_stem",
            collar_dx > stem_dx + 0.005,
            f"collar_dx={collar_dx:.4f} stem_dx={stem_dx:.4f}",
        )

    # --- variant 11: underside nuts present on every base -----------------
    for base_part, base_name in (
        (spout_base, "spout_base"),
        (hot_col, "hot_valve_column"),
        (cold_col, "cold_valve_column"),
    ):
        nut = base_part.get_visual("mount_nut")
        ctx.check(
            f"{base_name}_has_mount_nut",
            nut is not None,
            "mount_nut visual missing",
        )
        nut_aabb = ctx.part_element_world_aabb(base_part, elem=nut)
        ctx.check(
            f"{base_name}_nut_below_deck",
            nut_aabb[1][2] < 0.0,
            f"nut top z={nut_aabb[1][2]:.4f}",
        )

    # --- gooseneck form: taller spout ------------------------------------
    neck_aabb = ctx.part_world_aabb(gooseneck)
    arc_top_above_deck = neck_aabb[1][2] - DECK_T
    ctx.check(
        "gooseneck_arc_top_height",
        0.34 < arc_top_above_deck < 0.45,
        f"arc top {arc_top_above_deck:.3f} m above deck",
    )
    tip_aabb = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    outlet_above_deck = 0.5 * (tip_aabb[0][2] + tip_aabb[1][2]) - DECK_T
    ctx.check(
        "spout_outlet_height",
        0.27 < outlet_above_deck < 0.35,
        f"outlet {outlet_above_deck:.3f} m above deck",
    )
    ctx.check(
        "spout_hooks_forward",
        tip_aabb[1][1] > 0.10,
        f"outlet front reach y={tip_aabb[1][1]:.3f}",
    )

    # --- lever form: bar along Y overhangs toward user -------------------
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        bar_aabb = ctx.part_element_world_aabb(
            lever, elem=lever.get_visual("lever_bar")
        )
        bar_center_y = 0.5 * (bar_aabb[0][1] + bar_aabb[1][1])
        col_pos = ctx.part_world_position(col)
        ctx.check(
            f"{lever.name}_bar_overhangs_toward_user",
            bar_center_y > col_pos[1] + 0.01,
            f"bar center y={bar_center_y:.3f} vs column y={col_pos[1]:.3f}",
        )
        ctx.expect_gap(
            lever,
            col,
            axis="z",
            positive_elem=lever.get_visual("lever_bar"),
            min_gap=0.02,
        )

    # Indicator dots
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(
            f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}"
        )

    # --- articulation: forward-back lever rotation proof ------------------
    with ctx.pose({hot_turn: 0.0}):
        bar0 = ctx.part_element_world_aabb(
            hot_lever, elem=hot_lever.get_visual("lever_bar")
        )
    with ctx.pose({hot_turn: math.pi / 2}):
        bar90 = ctx.part_element_world_aabb(
            hot_lever, elem=hot_lever.get_visual("lever_bar")
        )
    span_y0 = bar0[1][1] - bar0[0][1]
    span_z0 = bar0[1][2] - bar0[0][2]
    span_y90 = bar90[1][1] - bar90[0][1]
    span_z90 = bar90[1][2] - bar90[0][2]
    ctx.check(
        "hot_lever_forward_back_rotation",
        span_y0 > 0.10
        and span_z0 < 0.03
        and span_z90 > 0.10
        and span_y90 < 0.05,
        f"rest span=(y:{span_y0:.3f},z:{span_z0:.3f}) "
        f"turned span=(y:{span_y90:.3f},z:{span_z90:.3f})",
    )

    # Spout swivel proof
    with ctx.pose({spout_swivel: math.pi / 4}):
        tip45 = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    tip45_cx = 0.5 * (tip45[0][0] + tip45[1][0])
    tip45_cz = 0.5 * (tip45[0][2] + tip45[1][2])
    ctx.check(
        "spout_swivels_about_column_axis",
        tip45_cx < -0.06
        and abs(tip45_cz - (outlet_above_deck + DECK_T)) < 1e-3,
        f"tip at 45deg x={tip45_cx:.3f} z={tip45_cz:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
