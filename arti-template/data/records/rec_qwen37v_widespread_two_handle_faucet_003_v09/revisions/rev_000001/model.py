from __future__ import annotations

"""Matte-black widespread three-piece bathroom faucet set.

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (revolute about the column's vertical axis, -45..+45 deg),
- hot (left) and cold (right): valve columns topped by T-style lever
  handles (each revolute about its column's vertical axis, -90..+90 deg).

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
    place_on_surface,
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
VALVE_COL_H = 0.10  # column top = lever joint height above the deck surface

# T-lever (in the lever part frame, origin at valve column top)
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.045
BAR_R = 0.0095
BAR_LEN = 0.12
BAR_CENTER_OFF = 0.025  # bar center offset so the stem sits ~1/3 from one end
DOT_R = 0.0035

ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.016
AERATOR_R = 0.017
# Unit tangent of the arc at the hook end (pointing out of the spout, downward).
_TX = math.sin(math.radians(HOOK_DEG))  # y component
_TZ = -math.cos(math.radians(HOOK_DEG))  # z component
AERATOR_CY = ARC_END_Y + _TX * (AERATOR_LEN / 2 - 0.004)
AERATOR_CZ = ARC_END_Z + _TZ * (AERATOR_LEN / 2 - 0.004)

# Escutcheon plate (oval, raised above deck under all three posts)
ESC_HALF_X = 0.19  # half-width along spread axis (covers ±0.15 + margin)
ESC_HALF_Y = 0.045  # half-depth front/back
ESC_T = 0.006  # plate thickness
ESC_RAISE = 0.003  # small raised lip above deck surface

# Stem collars (visible ring under each handle)
STEM_COLLAR_R = 0.016
STEM_COLLAR_H = 0.008

# Diverter knob (behind spout, prismatic vertical)
DIV_STEM_R = 0.006
DIV_STEM_H = 0.030
DIV_KNOB_R = 0.012
DIV_KNOB_H = 0.014
DIV_TRAVEL = 0.025  # max upward travel in meters
DIV_Y_OFFSET = -(SPOUT_COL_R + DIV_STEM_R * 0.3)  # mounted on the rear column surface, stem half-seated


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


def _escutcheon_plate() -> cq.Workplane:
    """Raised oval escutcheon plate that spans all three faucet posts.

    Three circular through-holes let the column bodies pass through cleanly.
    """
    plate = (
        cq.Workplane("XY")
        .ellipse(ESC_HALF_X, ESC_HALF_Y)
        .extrude(ESC_T)
    )
    # Cut holes at the three column positions (spout center + two valve positions).
    hole_r = max(SPOUT_COL_R, VALVE_COL_R) + 0.001  # small clearance
    holes = (
        cq.Workplane("XY")
        .center(0.0, 0.0)
        .circle(hole_r)
        .center(SPREAD_HALF, 0.0)
        .circle(hole_r)
        .center(-2 * SPREAD_HALF, 0.0)
        .circle(hole_r)
        .extrude(ESC_T)
    )
    return plate.cut(holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_black_bathroom_faucet")

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

    # --------------------------------------------------- escutcheon plate
    escutcheon = model.part("escutcheon_plate")
    escutcheon.visual(
        mesh_from_cadquery(_escutcheon_plate(), "escutcheon_oval"),
        origin=Origin(xyz=(0.0, 0.0, ESC_RAISE)),
        material=matte_black,
        name="escutcheon_oval",
    )

    model.articulation(
        "deck_to_escutcheon",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=escutcheon,
        origin=Origin(xyz=(0.0, 0.0, DECK_T)),
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

    # --------------------------------------------------- diverter knob
    # A small pull knob behind the spout column that slides vertically to
    # divert water between spout and shower.
    diverter = model.part("diverter_knob")
    diverter.visual(
        Cylinder(radius=DIV_STEM_R, length=DIV_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, DIV_STEM_H / 2)),
        material=matte_black,
        name="diverter_stem",
    )
    diverter.visual(
        Cylinder(radius=DIV_KNOB_R, length=DIV_KNOB_H),
        origin=Origin(xyz=(0.0, 0.0, DIV_STEM_H + DIV_KNOB_H / 2)),
        material=matte_black,
        name="diverter_cap",
    )

    model.articulation(
        "diverter_slide",
        ArticulationType.PRISMATIC,
        parent=spout_base,
        child=diverter,
        origin=Origin(xyz=(0.0, DIV_Y_OFFSET, SPOUT_COL_H - 0.015)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=0.5, lower=0.0, upper=DIV_TRAVEL
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

    def _t_lever(name: str, bar_off: float, dot_material: object) -> object:
        lever = model.part(name)
        lever.visual(
            Cylinder(radius=STEM_R, length=STEM_TOP + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (STEM_TOP - STEM_EMBED) / 2)),
            material=matte_black,
            name="lever_stem",
        )
        # Visible stem collar: a ring at the base of the stem where it
        # meets the valve column top.
        lever.visual(
            Cylinder(radius=STEM_COLLAR_R, length=STEM_COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, -STEM_COLLAR_H / 2)),
            material=matte_black,
            name="stem_collar",
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
    # Hot on the left (-X), cold on the right (+X); +Y is toward the user.
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
    escutcheon = object_model.get_part("escutcheon_plate")
    spout_base = object_model.get_part("spout_base")
    gooseneck = object_model.get_part("gooseneck_spout")
    diverter = object_model.get_part("diverter_knob")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_lever = object_model.get_part("hot_lever")
    cold_lever = object_model.get_part("cold_lever")

    spout_swivel = object_model.get_articulation("spout_swivel")
    diverter_slide = object_model.get_articulation("diverter_slide")
    hot_turn = object_model.get_articulation("hot_lever_turn")
    cold_turn = object_model.get_articulation("cold_lever_turn")

    spout_tube = gooseneck.get_visual("spout_tube")
    base_column = spout_base.get_visual("base_column")
    aerator = gooseneck.get_visual("aerator")

    # Intentional hidden engagements: spout riser and lever stems seat inside
    # their columns so the rotating parts read as mounted, not floating.
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
    # Diverter stem slides inside the spout base column region (represented
    # as penetrating the base at its mount point).
    ctx.allow_overlap(
        diverter,
        spout_base,
        elem_a=diverter.get_visual("diverter_stem"),
        elem_b=spout_base.get_visual("base_column"),
        reason="diverter stem seats inside the spout column bore for prismatic slide",
    )
    # Stem collars wrap around the stem at the valve column top - small
    # intentional overlap where the collar ring meets the column surface.
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.allow_overlap(
            lever,
            col,
            elem_a=lever.get_visual("stem_collar"),
            elem_b=col.get_visual("valve_body"),
            reason="stem collar sits flush at the valve column top surface",
        )
    # Escutcheon plate sits under the flanges of all three columns on the deck.
    for col_part in (spout_base, hot_col, cold_col):
        flange_name = "base_flange" if col_part is spout_base else "valve_flange"
        ctx.allow_overlap(
            escutcheon,
            col_part,
            elem_a=escutcheon.get_visual("escutcheon_oval"),
            elem_b=col_part.get_visual(flange_name),
            reason="escutcheon plate sits flush under the column flange on the deck",
        )

    # --- joint plan: types, axes, ranges -----------------------------------
    for joint, lim in ((spout_swivel, math.pi / 4), (hot_turn, math.pi / 2), (cold_turn, math.pi / 2)):
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

    # Diverter is prismatic along Z with 0..DIV_TRAVEL range.
    ctx.check(
        "diverter_slide_is_vertical_prismatic",
        str(diverter_slide.joint_type).lower().endswith("prismatic")
        and tuple(diverter_slide.axis) == (0.0, 0.0, 1.0),
        f"type={diverter_slide.joint_type} axis={diverter_slide.axis}",
    )
    div_ml = diverter_slide.motion_limits
    ctx.check(
        "diverter_slide_range",
        div_ml is not None
        and abs(div_ml.lower) < 1e-6
        and abs(div_ml.upper - DIV_TRAVEL) < 1e-6,
        f"lower={div_ml.lower} upper={div_ml.upper}",
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

    # --- escutcheon plate: spans all three posts, sits on deck -------------
    esc_aabb = ctx.part_world_aabb(escutcheon)
    esc_min_x = esc_aabb[0][0]
    esc_max_x = esc_aabb[1][0]
    ctx.check(
        "escutcheon_covers_all_posts",
        esc_min_x < hot_pos[0] and esc_max_x > cold_pos[0],
        f"esc x=[{esc_min_x:.3f},{esc_max_x:.3f}] hot_x={hot_pos[0]:.3f} cold_x={cold_pos[0]:.3f}",
    )
    # Escutcheon sits just above the deck surface.
    ctx.expect_gap(
        escutcheon,
        deck,
        axis="z",
        min_gap=-0.001,
        max_gap=0.010,
        name="escutcheon_sits_on_deck",
    )

    # --- stem collars exist on each lever ----------------------------------
    for lever in (hot_lever, cold_lever):
        collar = lever.get_visual("stem_collar")
        ctx.check(
            f"{lever.name}_has_stem_collar",
            collar is not None,
            f"stem_collar visual missing from {lever.name}",
        )
        # Collar radius should be larger than the stem radius.
        collar_aabb = ctx.part_element_world_aabb(lever, elem=collar)
        collar_dx = collar_aabb[1][0] - collar_aabb[0][0]
        ctx.check(
            f"{lever.name}_collar_wider_than_stem",
            collar_dx > 2 * STEM_R + 0.005,
            f"collar_dx={collar_dx:.4f} stem_d={2*STEM_R:.4f}",
        )

    # --- diverter knob: behind spout, slides up on prismatic joint ---------
    div_rest_pos = ctx.part_world_position(diverter)
    with ctx.pose({diverter_slide: DIV_TRAVEL}):
        div_raised_pos = ctx.part_world_position(diverter)
    ctx.check(
        "diverter_slides_upward",
        div_rest_pos is not None
        and div_raised_pos is not None
        and div_raised_pos[2] > div_rest_pos[2] + DIV_TRAVEL * 0.9,
        f"rest_z={div_rest_pos[2]:.4f} raised_z={div_raised_pos[2]:.4f}",
    )
    # Diverter is behind the spout (negative Y relative to spout).
    spout_center = ctx.part_world_position(spout_base)
    ctx.check(
        "diverter_behind_spout",
        div_rest_pos[1] < spout_center[1] - 0.02,
        f"div_y={div_rest_pos[1]:.3f} spout_y={spout_center[1]:.3f}",
    )

    # --- gooseneck form: rises ~0.32 above deck, outlet ~0.25 above deck ---
    neck_aabb = ctx.part_world_aabb(gooseneck)
    arc_top_above_deck = neck_aabb[1][2] - DECK_T
    ctx.check(
        "gooseneck_arc_top_height",
        0.28 < arc_top_above_deck < 0.36,
        f"arc top {arc_top_above_deck:.3f} m above deck",
    )
    tip_aabb = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    outlet_above_deck = 0.5 * (tip_aabb[0][2] + tip_aabb[1][2]) - DECK_T
    ctx.check(
        "spout_outlet_about_0p25_above_deck",
        abs(outlet_above_deck - 0.25) < 0.02,
        f"outlet {outlet_above_deck:.3f} m above deck",
    )
    ctx.check(
        "spout_hooks_forward",
        tip_aabb[1][1] > 0.10,
        f"outlet front reach y={tip_aabb[1][1]:.3f}",
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
        "spout_swivels_about_column_axis",
        tip45_cx < -0.06 and abs(tip45_cz - (outlet_above_deck + DECK_T)) < 1e-3,
        f"tip at 45deg x={tip45_cx:.3f} z={tip45_cz:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
