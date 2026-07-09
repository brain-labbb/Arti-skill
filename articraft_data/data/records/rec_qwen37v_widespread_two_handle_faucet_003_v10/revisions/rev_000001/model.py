from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet set (variant 10).

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (continuous vertical joint, unlimited rotation),
- hot (left) and cold (right): valve columns topped by T-style lever
  handles (each revolute about its column's vertical axis, -90..+90 deg).

Variant features:
- Handles are asymmetrically angled but balanced around the spout.
- Central spout swivels on a continuous vertical joint.
- Narrow seam rings at all three deck bases.
- Decorative ring ridges on the handle pedestals.

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
VALVE_COL_H = 0.10  # column top = lever joint height above the deck surface

# T-lever (in the lever part frame, origin at valve column top)
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.045
BAR_R = 0.0095
BAR_LEN = 0.12
BAR_CENTER_OFF = 0.025  # bar center offset so the stem sits ~1/3 from one end
DOT_R = 0.0035

# Variant 10: asymmetric handle rest angles (degrees, measured from +X in local frame)
HOT_LEVER_YAW_DEG = 25.0   # hot lever angled forward-outward
COLD_LEVER_YAW_DEG = -12.0  # cold lever angled slightly back-outward

# Variant 10: decorative rings and seam dimensions
SEAM_RING_TUBE_R = 0.0012   # thin seam torus tube radius
RIDGE_TUBE_R = 0.0018       # decorative ridge torus tube radius
RIDGE_COUNT = 3             # number of ridges per valve pedestal
RIDGE_SPACING = 0.018       # vertical spacing between ridges

ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.016
AERATOR_R = 0.017
# Unit tangent of the arc at the hook end (pointing out of the spout, downward).
_TX = math.sin(math.radians(HOOK_DEG))  # y component
_TZ = -math.cos(math.radians(HOOK_DEG))  # z component
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


def _torus_ring(center_r: float, tube_r: float) -> cq.Workplane:
    """Build a torus ring centered on the Z axis at z=0.

    The ring cross-section center traces a circle of radius `center_r`
    in the XY plane; the tube cross-section has radius `tube_r`.
    """
    return (
        cq.Workplane("XZ")
        .moveTo(center_r, 0.0)
        .circle(tube_r)
        .revolve(360, (-center_r, 0.0), (-center_r, 1.0))
    )


def _seam_ring(flange_r: float) -> cq.Workplane:
    """Narrow seam ring at the deck-flange junction (just above the deck)."""
    return _torus_ring(flange_r, SEAM_RING_TUBE_R)


def _decorative_ridge(col_r: float) -> cq.Workplane:
    """Raised decorative ridge band around a column."""
    return _torus_ring(col_r + RIDGE_TUBE_R * 0.4, RIDGE_TUBE_R)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet_v10")

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
    # Variant 10: narrow seam ring at the deck-flange junction.
    spout_base.visual(
        mesh_from_cadquery(_seam_ring(SPOUT_FLANGE_R), "spout_seam_ring"),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_FLANGE_H)),
        material=matte_black,
        name="base_seam_ring",
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
        ArticulationType.CONTINUOUS,
        parent=spout_base,
        child=gooseneck_spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_COL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0),
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
        # Variant 10: narrow seam ring at the deck-flange junction.
        col.visual(
            mesh_from_cadquery(_seam_ring(VALVE_FLANGE_R), f"{name}_seam_ring"),
            origin=Origin(xyz=(0.0, 0.0, VALVE_FLANGE_H)),
            material=matte_black,
            name="valve_seam_ring",
        )
        # Variant 10: decorative ring ridges on the pedestal.
        for i in range(RIDGE_COUNT):
            z_ridge = VALVE_FLANGE_H + 0.015 + i * RIDGE_SPACING
            col.visual(
                mesh_from_cadquery(
                    _decorative_ridge(VALVE_COL_R), f"{name}_ridge_{i}"
                ),
                origin=Origin(xyz=(0.0, 0.0, z_ridge)),
                material=matte_black,
                name=f"ridge_ring_{i}",
            )
        return col

    def _t_lever(name: str, bar_off: float, dot_material: object, yaw_deg: float) -> object:
        lever = model.part(name)
        yaw = math.radians(yaw_deg)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        # Stem (vertical) is unaffected by yaw — it's rotationally symmetric.
        lever.visual(
            Cylinder(radius=STEM_R, length=STEM_TOP + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (STEM_TOP - STEM_EMBED) / 2)),
            material=matte_black,
            name="lever_stem",
        )
        # Horizontal T-bar rotated by yaw in the local XY plane.
        bar_cx = bar_off * cos_y
        bar_cy = bar_off * sin_y
        lever.visual(
            Cylinder(radius=BAR_R, length=BAR_LEN),
            origin=Origin(
                xyz=(bar_cx, bar_cy, STEM_TOP),
                rpy=(0.0, math.pi / 2, yaw),
            ),
            material=matte_black,
            name="lever_bar",
        )
        for end in (-1.0, 1.0):
            ex = bar_cx + end * (BAR_LEN / 2) * cos_y
            ey = bar_cy + end * (BAR_LEN / 2) * sin_y
            lever.visual(
                Sphere(radius=BAR_R),
                origin=Origin(xyz=(ex, ey, STEM_TOP)),
                material=matte_black,
                name=f"bar_cap_{'outer' if end * bar_off > 0 else 'inner'}",
            )
        # Tiny temperature indicator dot on the front of the stem, rotated with yaw.
        dot_y = (STEM_R - 0.0005)
        lever.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(dot_y * sin_y, dot_y * cos_y, 0.022)),
            material=dot_material,
            name="indicator_dot",
        )
        return lever

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    # Hot on the left (-X), cold on the right (+X); +Y is toward the user.
    # Variant 10: asymmetric rest angles — hot angled forward, cold slightly back.
    hot_lever = _t_lever("hot_lever", -BAR_CENTER_OFF, hot_red, HOT_LEVER_YAW_DEG)
    cold_lever = _t_lever("cold_lever", BAR_CENTER_OFF, cold_blue, COLD_LEVER_YAW_DEG)

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

    # --- variant 10: spout swivel is CONTINUOUS (unlimited) ---------------
    ctx.check(
        "spout_swivel_is_continuous",
        str(spout_swivel.joint_type).lower().endswith("continuous"),
        f"joint_type={spout_swivel.joint_type}",
    )
    ctx.check(
        "spout_swivel_vertical_axis",
        tuple(spout_swivel.axis) == (0.0, 0.0, 1.0),
        f"axis={spout_swivel.axis}",
    )
    # Continuous joints have effort/velocity but no position bounds.
    ml = spout_swivel.motion_limits
    ctx.check(
        "spout_swivel_no_position_bounds",
        ml is not None and ml.lower is None and ml.upper is None,
        f"lower={getattr(ml, 'lower', None)} upper={getattr(ml, 'upper', None)}",
    )

    # --- lever joints: still revolute with -90..+90 -----------------------
    for joint, lim in ((hot_turn, math.pi / 2), (cold_turn, math.pi / 2)):
        ctx.check(
            f"{joint.name}_is_vertical_revolute",
            str(joint.joint_type).lower().endswith("revolute")
            and tuple(joint.axis) == (0.0, 0.0, 1.0),
            f"axis={joint.axis}",
        )
        jml = joint.motion_limits
        ctx.check(
            f"{joint.name}_range",
            jml is not None
            and abs(jml.lower + lim) < 1e-6
            and abs(jml.upper - lim) < 1e-6,
            f"lower={jml.lower} upper={jml.upper}",
        )

    # --- variant 10: asymmetric handle angles ------------------------------
    # The bars are at different yaw angles in their local frames, so they
    # should not be parallel at rest.
    hot_bar = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("lever_bar"))
    cold_bar = ctx.part_element_world_aabb(cold_lever, elem=cold_lever.get_visual("lever_bar"))
    hot_span_x = hot_bar[1][0] - hot_bar[0][0]
    hot_span_y = hot_bar[1][1] - hot_bar[0][1]
    cold_span_x = cold_bar[1][0] - cold_bar[0][0]
    cold_span_y = cold_bar[1][1] - cold_bar[0][1]
    # The asymmetric angles mean the span ratios differ between hot and cold.
    hot_angle = math.atan2(hot_span_y, hot_span_x)
    cold_angle = math.atan2(cold_span_y, cold_span_x)
    ctx.check(
        "handles_asymmetric_angles",
        abs(hot_angle - cold_angle) > math.radians(5.0),
        f"hot_angle={math.degrees(hot_angle):.1f}° cold_angle={math.degrees(cold_angle):.1f}°",
    )

    # Both handles remain balanced: their bar centers stay near their column x-positions.
    for lever, sign, yaw_deg in (
        (hot_lever, -1.0, HOT_LEVER_YAW_DEG),
        (cold_lever, 1.0, COLD_LEVER_YAW_DEG),
    ):
        bar_aabb = ctx.part_element_world_aabb(lever, elem=lever.get_visual("lever_bar"))
        bar_center_x = 0.5 * (bar_aabb[0][0] + bar_aabb[1][0])
        col_x = sign * 0.15
        ctx.check(
            f"{lever.name}_bar_overhangs_outward",
            sign * (bar_center_x - col_x) > 0.01,
            f"bar center x={bar_center_x:.3f} vs column x={col_x:.3f}",
        )
        # Bar clears the valve column top.
        ctx.expect_gap(
            lever,
            (hot_col if sign < 0 else cold_col),
            axis="z",
            positive_elem=lever.get_visual("lever_bar"),
            min_gap=0.02,
        )

    # --- variant 10: seam rings at all three deck bases --------------------
    spout_base.get_visual("base_seam_ring")
    for col in (hot_col, cold_col):
        col.get_visual("valve_seam_ring")
    ctx.check("seam_rings_present", True, "")

    # --- variant 10: decorative ring ridges on handle pedestals -----------
    for col in (hot_col, cold_col):
        for i in range(RIDGE_COUNT):
            col.get_visual(f"ridge_ring_{i}")
    ctx.check("decorative_ridges_present", True, "")

    # --- placement: 0.30 m spread, all three pieces seated on the deck ----
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

    # --- gooseneck form: rises ~0.32 above deck, outlet ~0.25 above deck --
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

    # Indicator dots: red on hot, blue on cold.
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}")

    # --- articulation behavior ---------------------------------------------
    # Lever rotation proof: at q=0 the bar has its yaw offset; at q=+90 deg
    # it rotates 90° about Z.
    with ctx.pose({hot_turn: 0.0}):
        bar0 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("lever_bar"))
    with ctx.pose({hot_turn: math.pi / 2}):
        bar90 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("lever_bar"))
    span_x0 = bar0[1][0] - bar0[0][0]
    span_y0 = bar0[1][1] - bar0[0][1]
    span_x90 = bar90[1][0] - bar90[0][0]
    span_y90 = bar90[1][1] - bar90[0][1]
    # The yaw offset means the bar isn't purely along X at q=0, but at q=+90
    # the major span should shift from X-dominant toward Y-dominant.
    ctx.check(
        "hot_lever_rotates_about_vertical_axis",
        span_x0 > 0.05 and span_y90 > 0.05 and abs(span_x0 - span_y90) < 0.04,
        f"closed span=({span_x0:.3f},{span_y0:.3f}) turned span=({span_x90:.3f},{span_y90:.3f})",
    )

    # Spout continuous swivel: 90° rotation swings the forward outlet toward -X.
    with ctx.pose({spout_swivel: math.pi / 2}):
        tip90 = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    tip90_cx = 0.5 * (tip90[0][0] + tip90[1][0])
    tip90_cz = 0.5 * (tip90[0][2] + tip90[1][2])
    ctx.check(
        "spout_continuous_swivel_full_rotation",
        tip90_cx < -0.06 and abs(tip90_cz - (outlet_above_deck + DECK_T)) < 1e-3,
        f"tip at 90deg x={tip90_cx:.3f} z={tip90_cz:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
