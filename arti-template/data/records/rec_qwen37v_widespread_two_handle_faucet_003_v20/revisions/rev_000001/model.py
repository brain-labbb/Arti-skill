from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet set (variant 20).

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (revolute about the column's vertical axis, -45..+45 deg) and a hinged
  hollow aerator at the outlet tip (revolute about a horizontal axis,
  0..30 deg downward tilt)
- hot (left) and cold (right): valve columns topped by T-style lever
  handles (each revolute about its column's vertical axis, -90..+90 deg),
  asymmetrically angled but balanced around the spout
- narrow seam rings at all three deck bases

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

# Narrow seam ring at each deck base
SEAM_RING_H = 0.002
SEAM_RING_EXTRA_R = 0.003  # extends slightly past the flange edge

# Gooseneck (in the spout part frame, origin at column top)
TUBE_R = 0.0155  # gooseneck tube radius
RISER_EMBED = 0.03  # hidden engagement into the column below the joint
RISER_TOP = 0.14  # straight riser ends here; arc starts
ARC_R = 0.062  # gooseneck arc radius
HOOK_DEG = -12.0  # arc end angle; past vertical = forward-down hook
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
# Asymmetric bar offsets: hot overhangs further outward, cold is closer
HOT_BAR_OFF = -0.035  # hot bar center offset (larger overhang)
COLD_BAR_OFF = 0.020  # cold bar center offset (smaller overhang)
DOT_R = 0.0035

ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.024  # longer aerator for visible pivot displacement
AERATOR_R = 0.017
AERATOR_BORE_R = AERATOR_R - 0.004  # hollow bore for central outlet
AERATOR_EMBED = 0.002  # 2 mm embed into tube end for visual connection

# Unit tangent of the arc at the hook end (pointing out of the spout, downward).
_TX = math.sin(math.radians(HOOK_DEG))  # y component
_TZ = -math.cos(math.radians(HOOK_DEG))  # z component
# Aerator center offset from hinge origin along tangent direction
_AERATOR_CENTER_DIST = AERATOR_LEN / 2 - AERATOR_EMBED  # ~10 mm from hinge
AERATOR_LOCAL_CY = _TX * _AERATOR_CENTER_DIST
AERATOR_LOCAL_CZ = _TZ * _AERATOR_CENTER_DIST


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


def _hollow_aerator() -> cq.Workplane:
    """Hollow aerator tube: annular cross-section extruded along Z."""
    return (
        cq.Workplane("XY")
        .circle(AERATOR_R)
        .circle(AERATOR_BORE_R)
        .extrude(AERATOR_LEN)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_black_bathroom_faucet_v20")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.03, 0.03, 0.03, 1.0))
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
    # Narrow seam ring at spout deck base
    spout_base.visual(
        Cylinder(radius=SPOUT_FLANGE_R + SEAM_RING_EXTRA_R, length=SEAM_RING_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_RING_H / 2)),
        material=seam_dark,
        name="deck_seam",
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

    # ------------------------------------------- hinged hollow aerator head
    aerator_head = model.part("aerator_head")
    aerator_head.visual(
        mesh_from_cadquery(_hollow_aerator(), "hollow_aerator"),
        origin=Origin(
            xyz=(0.0, AERATOR_LOCAL_CY, AERATOR_LOCAL_CZ),
            rpy=(math.radians(HOOK_DEG), 0.0, 0.0),
        ),
        material=matte_black,
        name="hollow_aerator",
    )

    model.articulation(
        "aerator_pivot",
        ArticulationType.REVOLUTE,
        parent=gooseneck_spout,
        child=aerator_head,
        # Hinge at the tube end where the aerator attaches
        origin=Origin(xyz=(0.0, ARC_END_Y, ARC_END_Z)),
        axis=(1.0, 0.0, 0.0),  # horizontal axis; positive q tilts downward
        motion_limits=MotionLimits(
            effort=3.0, velocity=1.0, lower=0.0, upper=0.52
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
        # Narrow seam ring at valve deck base
        col.visual(
            Cylinder(radius=VALVE_FLANGE_R + SEAM_RING_EXTRA_R, length=SEAM_RING_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_RING_H / 2)),
            material=seam_dark,
            name="deck_seam",
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
    # Asymmetric bar offsets: hot overhangs further, cold sits closer.
    hot_lever = _t_lever("hot_lever", HOT_BAR_OFF, hot_red)
    cold_lever = _t_lever("cold_lever", COLD_BAR_OFF, cold_blue)

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
    aerator_head = object_model.get_part("aerator_head")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_lever = object_model.get_part("hot_lever")
    cold_lever = object_model.get_part("cold_lever")

    spout_swivel = object_model.get_articulation("spout_swivel")
    aerator_pivot = object_model.get_articulation("aerator_pivot")
    hot_turn = object_model.get_articulation("hot_lever_turn")
    cold_turn = object_model.get_articulation("cold_lever_turn")

    spout_tube = gooseneck.get_visual("spout_tube")
    base_column = spout_base.get_visual("base_column")
    hollow_aerator = aerator_head.get_visual("hollow_aerator")

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
    # Aerator partially embeds into the gooseneck tube end at the hinge.
    ctx.allow_overlap(
        aerator_head,
        gooseneck,
        elem_a=hollow_aerator,
        elem_b=spout_tube,
        reason="aerator tube partially embeds 2 mm into the gooseneck tube end at the hinge",
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

    # Aerator pivot: horizontal revolute about X, 0..~30 deg downward
    ctx.check(
        "aerator_pivot_is_horizontal_revolute",
        str(aerator_pivot.joint_type).lower().endswith("revolute")
        and tuple(aerator_pivot.axis) == (1.0, 0.0, 0.0),
        f"type={aerator_pivot.joint_type} axis={aerator_pivot.axis}",
    )
    ap_ml = aerator_pivot.motion_limits
    ctx.check(
        "aerator_pivot_range_0_to_30deg",
        ap_ml is not None
        and abs(ap_ml.lower) < 1e-6
        and abs(ap_ml.upper - 0.52) < 0.02,
        f"lower={ap_ml.lower} upper={ap_ml.upper}",
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
    tip_aabb = ctx.part_element_world_aabb(aerator_head, elem=hollow_aerator)
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

    # --- asymmetric handles: different bar center offsets ------------------
    hot_bar_aabb = ctx.part_element_world_aabb(
        hot_lever, elem=hot_lever.get_visual("lever_bar")
    )
    cold_bar_aabb = ctx.part_element_world_aabb(
        cold_lever, elem=cold_lever.get_visual("lever_bar")
    )
    hot_bar_cx = 0.5 * (hot_bar_aabb[0][0] + hot_bar_aabb[1][0])
    cold_bar_cx = 0.5 * (cold_bar_aabb[0][0] + cold_bar_aabb[1][0])
    hot_offset = abs(hot_bar_cx - (-SPREAD_HALF))
    cold_offset = abs(cold_bar_cx - SPREAD_HALF)
    ctx.check(
        "handles_asymmetric_bar_offsets",
        abs(hot_offset - cold_offset) > 0.008,
        f"hot_offset={hot_offset:.4f} cold_offset={cold_offset:.4f}",
    )
    # Both still overhang outward from their columns
    ctx.check(
        "hot_bar_overhangs_outward",
        hot_bar_cx < -SPREAD_HALF - 0.01,
        f"hot bar center x={hot_bar_cx:.3f}",
    )
    ctx.check(
        "cold_bar_overhangs_outward",
        cold_bar_cx > SPREAD_HALF + 0.01,
        f"cold bar center x={cold_bar_cx:.3f}",
    )

    # Bar clears the valve column top (only the stem enters the column).
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.expect_gap(
            lever,
            col,
            axis="z",
            positive_elem=lever.get_visual("lever_bar"),
            min_gap=0.02,
        )

    # Indicator dots: red on hot, blue on cold, proud of the stem front.
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(
            f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}"
        )

    # --- narrow seam rings at all three deck bases -------------------------
    for base_part in (spout_base, hot_col, cold_col):
        seam = base_part.get_visual("deck_seam")
        ctx.check(
            f"{base_part.name}_has_deck_seam",
            seam is not None,
            f"missing deck_seam visual on {base_part.name}",
        )
    # Seam ring radius exceeds flange radius (visible ring outside the base)
    for base_part, flange_r in (
        (spout_base, SPOUT_FLANGE_R),
        (hot_col, VALVE_FLANGE_R),
        (cold_col, VALVE_FLANGE_R),
    ):
        seam_aabb = ctx.part_element_world_aabb(
            base_part, elem=base_part.get_visual("deck_seam")
        )
        seam_dx = seam_aabb[1][0] - seam_aabb[0][0]
        ctx.check(
            f"{base_part.name}_seam_wider_than_flange",
            seam_dx > 2.0 * flange_r,
            f"seam dx={seam_dx:.4f} vs 2*flange_r={2 * flange_r:.4f}",
        )

    # --- hollow aerator: annular geometry check ----------------------------
    # The aerator is a separate part with its own hollow tube visual
    ctx.check(
        "aerator_head_is_separate_part",
        aerator_head.name != gooseneck.name,
        "aerator_head must be a distinct part from gooseneck_spout",
    )
    # Aerators hollow bore should make the aerator wider than it is deep
    # (annular cross-section is still roughly cylindrical in AABB)
    aer_dims = ctx.part_element_world_aabb(aerator_head, elem=hollow_aerator)
    aer_dx = aer_dims[1][0] - aer_dims[0][0]
    aer_dz = aer_dims[1][2] - aer_dims[0][2]
    ctx.check(
        "hollow_aerator_has_annular_profile",
        aer_dx > 0.02 and aer_dz > 0.01,
        f"aerator dx={aer_dx:.4f} dz={aer_dz:.4f}",
    )

    # --- aerator pivot behavior: bottom goes lower at positive q -----------
    # The aerator hangs from the hinge; positive q about X tilts it so the
    # outlet end swings lower. Measure the AABB min_z (bottom of aerator).
    with ctx.pose({aerator_pivot: 0.0}):
        rest_box = ctx.part_element_world_aabb(aerator_head, elem=hollow_aerator)
    with ctx.pose({aerator_pivot: 0.52}):
        tilt_box = ctx.part_element_world_aabb(aerator_head, elem=hollow_aerator)
    rest_min_z = rest_box[0][2]
    tilt_min_z = tilt_box[0][2]
    ctx.check(
        "aerator_pivots_downward_at_positive_q",
        tilt_min_z < rest_min_z - 0.003,
        f"rest_min_z={rest_min_z:.4f} tilt_min_z={tilt_min_z:.4f}",
    )
    # Y extent increases as the aerator tilts away from vertical
    rest_dy = rest_box[1][1] - rest_box[0][1]
    tilt_dy = tilt_box[1][1] - tilt_box[0][1]
    ctx.check(
        "aerator_tilt_increases_y_extent",
        tilt_dy > rest_dy + 0.005,
        f"rest_dy={rest_dy:.4f} tilt_dy={tilt_dy:.4f}",
    )

    # --- articulation behavior: lever rotation proof -----------------------
    with ctx.pose({hot_turn: 0.0}):
        bar0 = ctx.part_element_world_aabb(
            hot_lever, elem=hot_lever.get_visual("lever_bar")
        )
    with ctx.pose({hot_turn: math.pi / 2}):
        bar90 = ctx.part_element_world_aabb(
            hot_lever, elem=hot_lever.get_visual("lever_bar")
        )
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
        tip45 = ctx.part_element_world_aabb(aerator_head, elem=hollow_aerator)
    tip45_cx = 0.5 * (tip45[0][0] + tip45[1][0])
    tip45_cz = 0.5 * (tip45[0][2] + tip45[1][2])
    ctx.check(
        "spout_swivels_about_column_axis",
        tip45_cx < -0.06 and abs(tip45_cz - (outlet_above_deck + DECK_T)) < 0.005,
        f"tip at 45deg x={tip45_cx:.3f} z={tip45_cz:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
