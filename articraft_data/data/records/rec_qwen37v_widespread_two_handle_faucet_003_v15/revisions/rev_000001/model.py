from __future__ import annotations

"""Matte-black widespread two-handle wall-mounted bathroom faucet set.

Three-piece layout on a sink deck (total spread 0.30 m):
- center: gooseneck spout wall-mounted on a horizontal arm from a vertical
  escutcheon plate, swiveling about the vertical axis (-45..+45 deg),
- hot (left) and cold (right): deck-mounted valve columns topped by T-style
  lever handles with visible stem collars (each revolute about its column's
  vertical axis, -90..+90 deg).

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

# ---- Wall-mount bracket (carries the gooseneck spout) ----
# Vertical escutcheon plate
WALL_PLATE_W = 0.080
WALL_PLATE_T = 0.015
WALL_PLATE_H = 0.200
# Horizontal support arm extending forward from plate face
WALL_ARM_R = 0.018
WALL_ARM_LEN = 0.070  # free length from plate front face
# Arm boss / escutcheon ring where arm exits the plate
ARM_BOSS_R = 0.024
ARM_BOSS_H = 0.010
# Arm centre-line height above deck surface
ARM_CENTER_Z = 0.120
# Arm starts at the plate front face
ARM_START_Y = WALL_PLATE_T / 2
# Spout pivot sits on top of the arm end
SPOUT_PIVOT_Y = ARM_START_Y + WALL_ARM_LEN
SPOUT_PIVOT_Z = ARM_CENTER_Z + WALL_ARM_R

# ---- Gooseneck (in the spout part frame, origin at pivot) ----
TUBE_R = 0.0155
RISER_EMBED = 0.03  # hidden engagement into the arm bore
RISER_TOP = 0.14
ARC_R = 0.062
HOOK_DEG = -12.0
COLLAR_R = 0.020
COLLAR_H = 0.016

# ---- Valve columns ----
VALVE_FLANGE_R = 0.036
VALVE_FLANGE_H = 0.010
VALVE_COL_R = 0.0225
VALVE_COL_H = 0.10

# ---- T-lever (in the lever part frame, origin at valve column top) ----
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.045
STEM_COLLAR_R = 0.015  # visible collar ring around stem at column top
STEM_COLLAR_H = 0.008
BAR_R = 0.0095
BAR_LEN = 0.12
BAR_CENTER_OFF = 0.025
DOT_R = 0.0035

ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.016
AERATOR_R = 0.017
_TX = math.sin(math.radians(HOOK_DEG))
_TZ = -math.cos(math.radians(HOOK_DEG))
AERATOR_CY = ARC_END_Y + _TX * (AERATOR_LEN / 2 - 0.004)
AERATOR_CZ = ARC_END_Z + _TZ * (AERATOR_LEN / 2 - 0.004)


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
    model = ArticulatedObject(name="widespread_wall_mount_faucet")

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

    # ------------------------------------------------------- wall-mount bracket
    # Part origin at the wall surface / deck rear edge, on the deck top.
    wall_mount = model.part("wall_mount")
    # Vertical escutcheon plate, centred on the mount origin in Y so it
    # straddles the deck rear edge (half behind the wall, half above the deck).
    wall_mount.visual(
        Box((WALL_PLATE_W, WALL_PLATE_T, WALL_PLATE_H)),
        origin=Origin(xyz=(0.0, 0.0, WALL_PLATE_H / 2)),
        material=matte_black,
        name="wall_plate",
    )
    # Horizontal support arm from plate front face forward.
    arm_cy = ARM_START_Y + WALL_ARM_LEN / 2
    wall_mount.visual(
        Cylinder(radius=WALL_ARM_R, length=WALL_ARM_LEN),
        origin=Origin(
            xyz=(0.0, arm_cy, ARM_CENTER_Z),
            rpy=(-math.pi / 2, 0.0, 0.0),
        ),
        material=matte_black,
        name="support_arm",
    )
    # Boss / escutcheon ring where the arm exits the plate.
    wall_mount.visual(
        Cylinder(radius=ARM_BOSS_R, length=ARM_BOSS_H),
        origin=Origin(
            xyz=(0.0, ARM_START_Y + ARM_BOSS_H / 2, ARM_CENTER_Z),
            rpy=(-math.pi / 2, 0.0, 0.0),
        ),
        material=matte_black,
        name="arm_boss",
    )

    model.articulation(
        "deck_to_wall_mount",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=wall_mount,
        origin=Origin(xyz=(0.0, -DECK_Y / 2, DECK_T)),
    )

    # ---------------------------------------------------------- gooseneck spout
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
        parent=wall_mount,
        child=gooseneck_spout,
        origin=Origin(xyz=(0.0, SPOUT_PIVOT_Y, SPOUT_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=2.0, lower=-math.pi / 4, upper=math.pi / 4
        ),
    )

    # --------------------------------------------------- hot / cold valve columns
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

    def _t_lever(name: str, bar_off: float, dot_material: object) -> object:
        lever = model.part(name)
        # Visible stem collar – wider ring sitting on the column top where the
        # stem exits the valve cartridge.
        lever.visual(
            Cylinder(radius=STEM_COLLAR_R, length=STEM_COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_COLLAR_H / 2)),
            material=matte_black,
            name="stem_collar",
        )
        lever.visual(
            Cylinder(radius=STEM_R, length=STEM_TOP + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (STEM_TOP - STEM_EMBED) / 2)),
            material=matte_black,
            name="lever_stem",
        )
        # Horizontal T-bar along X; off-center so it overhangs outward.
        lever.visual(
            Cylinder(radius=BAR_R, length=BAR_LEN),
            origin=Origin(xyz=(bar_off, 0.0, STEM_TOP), rpy=(0.0, math.pi / 2, 0.0)),
            material=matte_black,
            name="lever_bar",
        )
        for end in (-1.0, 1.0):
            lever.visual(
                Sphere(radius=BAR_R),
                origin=Origin(xyz=(bar_off + end * BAR_LEN / 2, 0.0, STEM_TOP)),
                material=matte_black,
                name=f"bar_cap_{'outer' if end * bar_off > 0 else 'inner'}",
            )
        # Tiny temperature indicator dot on the front of the stem.
        lever.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, STEM_R - 0.0005, 0.022)),
            material=dot_material,
            name="indicator_dot",
        )
        return lever

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    hot_lever = _t_lever("hot_lever", -BAR_CENTER_OFF, hot_red)
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

    for joint_name, parent, child in (
        ("hot_lever_turn", hot_valve_column, hot_lever),
        ("cold_lever_turn", cold_valve_column, cold_lever),
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
    wall_mount = object_model.get_part("wall_mount")
    gooseneck = object_model.get_part("gooseneck_spout")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_lever = object_model.get_part("hot_lever")
    cold_lever = object_model.get_part("cold_lever")

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_turn = object_model.get_articulation("hot_lever_turn")
    cold_turn = object_model.get_articulation("cold_lever_turn")

    spout_tube = gooseneck.get_visual("spout_tube")
    support_arm = wall_mount.get_visual("support_arm")
    aerator = gooseneck.get_visual("aerator")

    # Intentional hidden engagements: gooseneck riser seats into the arm bore,
    # lever stems seat inside their valve columns.
    ctx.allow_overlap(
        gooseneck,
        wall_mount,
        elem_a=spout_tube,
        elem_b=support_arm,
        reason="gooseneck riser tube seats 30 mm into the wall-mount arm bore",
    )
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.allow_overlap(
            lever,
            col,
            elem_a=lever.get_visual("lever_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="lever stem seats 15 mm into the valve cartridge bore",
        )

    # --- joint plan: types, axes, ranges -----------------------------------
    for joint, lim in (
        (spout_swivel, math.pi / 4),
        (hot_turn, math.pi / 2),
        (cold_turn, math.pi / 2),
    ):
        ctx.check(
            f"{joint.name}_is_vertical_revolute",
            str(joint.joint_type).lower().endswith("revolute")
            and tuple(joint.axis) == (0.0, 0.0, 1.0),
            f"axis={joint.axis}",
        )
        ml = joint.motion_limits
        ctx.check(
            f"{joint.name}_range",
            ml is not None
            and abs(ml.lower + lim) < 1e-6
            and abs(ml.upper - lim) < 1e-6,
            f"lower={ml.lower} upper={ml.upper}",
        )

    # --- wall mount is at deck rear, spout is wall-mounted -----------------
    wm_pos = ctx.part_world_position(wall_mount)
    ctx.check(
        "wall_mount_at_deck_rear",
        abs(wm_pos[0]) < 1e-6 and wm_pos[1] < -DECK_Y / 2 + 0.01,
        f"wall_mount pos=({wm_pos[0]:.3f}, {wm_pos[1]:.3f})",
    )
    # The wall plate contacts the deck top surface near the rear edge.
    ctx.expect_gap(
        wall_mount,
        deck,
        axis="z",
        positive_elem=wall_mount.get_visual("wall_plate"),
        negative_elem=deck.get_visual("deck_slab"),
        max_gap=0.002,
        name="wall_plate seated on deck surface",
    )

    # Handle valve columns seated on deck
    for piece in (hot_col, cold_col):
        ctx.expect_contact(piece, deck, contact_tol=1e-5)

    # --- placement: 0.30 m spread, handles on deck -------------------------
    hot_pos = ctx.part_world_position(hot_col)
    cold_pos = ctx.part_world_position(cold_col)
    ctx.check(
        "widespread_0p30_spread",
        abs(hot_pos[0] + 0.15) < 1e-6 and abs(cold_pos[0] - 0.15) < 1e-6,
        f"hot_x={hot_pos[0]} cold_x={cold_pos[0]}",
    )

    deck_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "deck_grounded_at_z0",
        abs(deck_aabb[0][2]) < 1e-6 and abs(deck_aabb[1][2] - DECK_T) < 1e-6,
        f"deck z {deck_aabb[0][2]}..{deck_aabb[1][2]}",
    )

    # --- stem collars present and positioned on both levers -------------------
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        collar = lever.get_visual("stem_collar")
        ctx.check(
            f"{lever.name}_has_stem_collar",
            collar is not None,
            "missing stem_collar visual",
        )
        # Collar bottom sits at the column top surface (lever frame z=0).
        collar_aabb = ctx.part_element_world_aabb(lever, elem=collar)
        col_top_z = DECK_T + VALVE_COL_H
        ctx.check(
            f"{lever.name}_collar_at_column_top",
            abs(collar_aabb[0][2] - col_top_z) < 0.002,
            f"collar bottom z={collar_aabb[0][2]:.4f} expected ~{col_top_z:.4f}",
        )
        # Collar is visibly wider than the stem.
        stem_aabb = ctx.part_element_world_aabb(lever, elem=lever.get_visual("lever_stem"))
        collar_dx = collar_aabb[1][0] - collar_aabb[0][0]
        stem_dx = stem_aabb[1][0] - stem_aabb[0][0]
        ctx.check(
            f"{lever.name}_collar_wider_than_stem",
            collar_dx > stem_dx + 0.005,
            f"collar_dx={collar_dx:.4f} stem_dx={stem_dx:.4f}",
        )

    # --- gooseneck form: rises above deck, outlet ~0.25 m above deck -------
    neck_aabb = ctx.part_world_aabb(gooseneck)
    arc_top_above_deck = neck_aabb[1][2] - DECK_T
    ctx.check(
        "gooseneck_arc_top_height",
        0.28 < arc_top_above_deck < 0.40,
        f"arc top {arc_top_above_deck:.3f} m above deck",
    )
    tip_aabb = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    outlet_above_deck = 0.5 * (tip_aabb[0][2] + tip_aabb[1][2]) - DECK_T
    ctx.check(
        "spout_outlet_about_0p25_above_deck",
        0.22 < outlet_above_deck < 0.28,
        f"outlet {outlet_above_deck:.3f} m above deck",
    )
    ctx.check(
        "spout_hooks_forward",
        tip_aabb[1][1] > 0.04,
        f"outlet front reach y={tip_aabb[1][1]:.3f}",
    )
    # Gooseneck is supported by the wall-mount arm, not the deck.
    pivot_world_z = DECK_T + SPOUT_PIVOT_Z
    ctx.check(
        "spout_pivot_above_deck_on_arm",
        abs(neck_aabb[0][2] - (pivot_world_z - 0.04)) < 0.04,
        f"neck bottom z={neck_aabb[0][2]:.3f} pivot={pivot_world_z:.3f}",
    )

    # --- lever form: off-center T-bar overhangs outward --------------------
    for lever, sign in ((hot_lever, -1.0), (cold_lever, 1.0)):
        bar_aabb = ctx.part_element_world_aabb(lever, elem=lever.get_visual("lever_bar"))
        bar_center_x = 0.5 * (bar_aabb[0][0] + bar_aabb[1][0])
        col_x = sign * 0.15
        ctx.check(
            f"{lever.name}_bar_overhangs_outward",
            sign * (bar_center_x - col_x) > 0.02,
            f"bar center x={bar_center_x:.3f} vs column x={col_x:.3f}",
        )
        # Bar clears the valve column top (only the stem enters the column).
        ctx.expect_gap(
            lever,
            (hot_col if sign < 0 else cold_col),
            axis="z",
            positive_elem=lever.get_visual("lever_bar"),
            min_gap=0.02,
        )

    # Indicator dots: red on hot, blue on cold, proud of the stem front.
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}")

    # --- articulation behavior ---------------------------------------------
    # Off-axis proof: at q=0 the T-bar spans X; at q=+90 deg it spans Y.
    with ctx.pose({hot_turn: 0.0}):
        bar0 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("lever_bar"))
    with ctx.pose({hot_turn: math.pi / 2}):
        bar90 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("lever_bar"))
    span_x0 = bar0[1][0] - bar0[0][0]
    span_y0 = bar0[1][1] - bar0[0][1]
    span_x90 = bar90[1][0] - bar90[0][0]
    span_y90 = bar90[1][1] - bar90[0][1]
    ctx.check(
        "hot_lever_rotates_about_vertical_axis",
        span_x0 > 0.10 and span_y0 < 0.03 and span_y90 > 0.10 and span_x90 < 0.03,
        f"closed span=({span_x0:.3f},{span_y0:.3f}) turned span=({span_x90:.3f},{span_y90:.3f})",
    )

    # Spout swivel: +45 deg swings the forward outlet toward -X (right-hand
    # rule about +Z), keeping its height unchanged.
    with ctx.pose({spout_swivel: math.pi / 4}):
        tip45 = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    tip45_cx = 0.5 * (tip45[0][0] + tip45[1][0])
    tip45_cz = 0.5 * (tip45[0][2] + tip45[1][2])
    ctx.check(
        "spout_swivels_about_arm_axis",
        tip45_cx < -0.04 and abs(tip45_cz - (outlet_above_deck + DECK_T)) < 1e-3,
        f"tip at 45deg x={tip45_cx:.3f} z={tip45_cz:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
