from __future__ import annotations

"""Matte-black widespread bridge-style two-handle bathroom faucet set.

Three deck-mounted columns linked by a horizontal bridge bar (spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (revolute about the column's vertical axis, -45..+45 deg),
  outlet aerator pivots on a small hinge (revolute, 0..0.45 rad),
- hot (left) and cold (right): valve columns topped by T-style lever
  handles (each revolute about its column's vertical axis, -90..+90 deg).

Bridge bar visually links all three posts at flange level.
Narrow seam rings at each deck base. All surfaces matte black;
tiny red/blue indicator dots on the handle stems.
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

# Bridge bar (horizontal pipe linking the three posts)
BRIDGE_R = 0.010

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

# Bridge bar height: sits on top of the valve flanges
BRIDGE_Z = VALVE_FLANGE_H + BRIDGE_R  # center at flange top + bar radius

# T-lever (in the lever part frame, origin at valve column top)
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.045
BAR_R = 0.0095
BAR_LEN = 0.12
BAR_CENTER_OFF = 0.025  # bar center offset so the stem sits ~1/3 from one end
DOT_R = 0.0035

# Seam rings (thin dark discs at deck-column junctions)
SEAM_H = 0.002
SEAM_SPOUT_R = SPOUT_FLANGE_R + 0.003
SEAM_VALVE_R = VALVE_FLANGE_R + 0.003

# Aerator dimensions
ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.016
AERATOR_R = 0.017
# Unit tangent of the arc at the hook end (pointing out of the spout, downward).
_TX = math.sin(math.radians(HOOK_DEG))  # y component
_TZ = -math.cos(math.radians(HOOK_DEG))  # z component

# Hinge bracket at the aerator pivot
HINGE_BARREL_R = 0.007
HINGE_BARREL_LEN = 0.022


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
    model = ArticulatedObject(name="widespread_bridge_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.08, 0.08, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.03, 0.03, 0.03, 1.0))

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
    # Bridge bar: horizontal pipe spanning hot to cold, linking all three posts
    spout_base.visual(
        Cylinder(radius=BRIDGE_R, length=SPREAD_HALF * 2),
        origin=Origin(
            xyz=(0.0, 0.0, BRIDGE_Z),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=matte_black,
        name="bridge_bar",
    )
    # Narrow seam ring at deck-column junction
    spout_base.visual(
        Cylinder(radius=SEAM_SPOUT_R, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2)),
        material=seam_dark,
        name="base_seam",
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

    # ------------------------------------------------------- pivoting aerator
    aerator = model.part("aerator")
    # Hinge barrel: horizontal pivot cylinder at the tube end
    aerator.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=matte_black,
        name="hinge_barrel",
    )
    # Aerator nozzle body: hangs downward from the hinge along local -Z
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_LEN / 2)),
        material=matte_black,
        name="aerator_body",
    )

    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=gooseneck_spout,
        child=aerator,
        # Hinge at the tube end; rpy pre-rotates frame so local -Z matches
        # the tube tangent direction at rest.
        origin=Origin(
            xyz=(0.0, ARC_END_Y, ARC_END_Z),
            rpy=(math.radians(HOOK_DEG), 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.0, lower=0.0, upper=0.45
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
        # Narrow seam ring at deck-column junction
        col.visual(
            Cylinder(radius=SEAM_VALVE_R, length=SEAM_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2)),
            material=seam_dark,
            name="valve_seam",
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
    spout_base = object_model.get_part("spout_base")
    gooseneck = object_model.get_part("gooseneck_spout")
    aerator = object_model.get_part("aerator")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_lever = object_model.get_part("hot_lever")
    cold_lever = object_model.get_part("cold_lever")

    spout_swivel = object_model.get_articulation("spout_swivel")
    aerator_hinge = object_model.get_articulation("aerator_hinge")
    hot_turn = object_model.get_articulation("hot_lever_turn")
    cold_turn = object_model.get_articulation("cold_lever_turn")

    spout_tube = gooseneck.get_visual("spout_tube")
    base_column = spout_base.get_visual("base_column")
    aerator_body = aerator.get_visual("aerator_body")
    hinge_barrel = aerator.get_visual("hinge_barrel")

    # --- intentional overlaps -----------------------------------------------
    # Spout riser seats inside the base column
    ctx.allow_overlap(
        gooseneck,
        spout_base,
        elem_a=spout_tube,
        elem_b=base_column,
        reason="gooseneck riser tube seats 30 mm into the base column bore",
    )
    # Lever stems seat inside valve columns
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.allow_overlap(
            lever,
            col,
            elem_a=lever.get_visual("lever_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="lever stem seats 15 mm into the valve cartridge bore",
        )
    # Bridge bar ends seat into the valve column bodies
    bridge_bar = spout_base.get_visual("bridge_bar")
    for col in (hot_col, cold_col):
        ctx.allow_overlap(
            spout_base,
            col,
            elem_a=bridge_bar,
            elem_b=col.get_visual("valve_body"),
            reason="bridge bar pipe ends seat into the valve column bodies to visually link the posts",
        )
    # Aerator hinge barrel seats at the spout tube end
    ctx.allow_overlap(
        aerator,
        gooseneck,
        elem_a=hinge_barrel,
        elem_b=spout_tube,
        reason="hinge barrel nests at the spout tube outlet end",
    )
    # Aerator body top contacts the tube end
    ctx.allow_overlap(
        aerator,
        gooseneck,
        elem_a=aerator_body,
        elem_b=spout_tube,
        reason="aerator body screws into the spout tube outlet",
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

    # Aerator hinge: revolute about X, 0 to 0.45 rad
    ctx.check(
        "aerator_hinge_is_revolute",
        str(aerator_hinge.joint_type).lower().endswith("revolute"),
        f"type={aerator_hinge.joint_type}",
    )
    ctx.check(
        "aerator_hinge_axis_is_x",
        tuple(aerator_hinge.axis) == (1.0, 0.0, 0.0),
        f"axis={aerator_hinge.axis}",
    )
    ah_ml = aerator_hinge.motion_limits
    ctx.check(
        "aerator_hinge_range",
        ah_ml is not None
        and abs(ah_ml.lower) < 1e-6
        and abs(ah_ml.upper - 0.45) < 1e-3,
        f"lower={ah_ml.lower} upper={ah_ml.upper}",
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

    # --- bridge bar: spans the full 0.30 m spread --------------------------
    bridge_aabb = ctx.part_element_world_aabb(spout_base, elem=bridge_bar)
    bridge_span_x = bridge_aabb[1][0] - bridge_aabb[0][0]
    ctx.check(
        "bridge_bar_spans_full_spread",
        bridge_span_x > 0.28,
        f"bridge x-span = {bridge_span_x:.3f} m",
    )
    # Bridge connects to both valve columns (contact at endpoints)
    for col in (hot_col, cold_col):
        ctx.expect_contact(spout_base, col, contact_tol=0.005)

    # --- seam rings: present at all three deck bases -----------------------
    base_seam = spout_base.get_visual("base_seam")
    hot_seam = hot_col.get_visual("valve_seam")
    cold_seam = cold_col.get_visual("valve_seam")
    for seam_visual, piece_name in (
        (base_seam, "spout_base"),
        (hot_seam, "hot_valve_column"),
        (cold_seam, "cold_valve_column"),
    ):
        seam_aabb = ctx.part_element_world_aabb(
            object_model.get_part(piece_name), elem=seam_visual
        )
        seam_height = seam_aabb[1][2] - seam_aabb[0][2]
        ctx.check(
            f"{piece_name}_seam_ring_present",
            seam_height < 0.005 and seam_height > 0.0005,
            f"seam height = {seam_height:.4f} m",
        )

    # --- gooseneck form: rises ~0.32 above deck, outlet ~0.25 above deck ---
    neck_aabb = ctx.part_world_aabb(gooseneck)
    arc_top_above_deck = neck_aabb[1][2] - DECK_T
    ctx.check(
        "gooseneck_arc_top_height",
        0.28 < arc_top_above_deck < 0.36,
        f"arc top {arc_top_above_deck:.3f} m above deck",
    )
    tip_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_body)
    outlet_above_deck = 0.5 * (tip_aabb[0][2] + tip_aabb[1][2]) - DECK_T
    ctx.check(
        "spout_outlet_about_0p25_above_deck",
        abs(outlet_above_deck - 0.25) < 0.03,
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
        ctx.expect_gap(
            lever,
            (hot_col if sign < 0 else cold_col),
            axis="z",
            positive_elem=lever.get_visual("lever_bar"),
            min_gap=0.02,
        )

    # Indicator dots: red on hot, blue on cold
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}")

    # --- articulation behavior ---------------------------------------------
    # Hot lever: at q=0 bar spans X; at q=+90 deg bar spans Y
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

    # Spout swivel: +45 deg swings the forward outlet toward -X
    with ctx.pose({spout_swivel: math.pi / 4}):
        tip45 = ctx.part_element_world_aabb(aerator, elem=aerator_body)
    tip45_cx = 0.5 * (tip45[0][0] + tip45[1][0])
    tip45_cz = 0.5 * (tip45[0][2] + tip45[1][2])
    ctx.check(
        "spout_swivels_about_column_axis",
        tip45_cx < -0.06 and abs(tip45_cz - (outlet_above_deck + DECK_T)) < 1e-3,
        f"tip at 45deg x={tip45_cx:.3f} z={tip45_cz:.3f}",
    )

    # Aerator hinge: at q=0.45 rad the outlet tip moves forward (+Y)
    with ctx.pose({aerator_hinge: 0.0}):
        aer_rest = ctx.part_element_world_aabb(aerator, elem=aerator_body)
    with ctx.pose({aerator_hinge: 0.45}):
        aer_tilted = ctx.part_element_world_aabb(aerator, elem=aerator_body)
    rest_cy = 0.5 * (aer_rest[0][1] + aer_rest[1][1])
    tilted_cy = 0.5 * (aer_tilted[0][1] + aer_tilted[1][1])
    ctx.check(
        "aerator_hinge_tilts_outlet",
        abs(tilted_cy - rest_cy) > 0.002,
        f"rest_cy={rest_cy:.4f} tilted_cy={tilted_cy:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
