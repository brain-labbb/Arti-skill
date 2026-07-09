from __future__ import annotations

"""Matte-black widespread two-handle waterfall bathroom faucet set.

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a short rectangular waterfall channel
  spout (revolute swivel about the column's vertical axis, -45..+45 deg),
- hot (left) and cold (right): valve columns topped by T-style lever
  handles (each revolute about its column's vertical axis, -90..+90 deg).

Narrow visible seams at all three deck bases. All faucet surfaces matte black;
tiny red/blue indicator dots on the handle stems. Modeled at true scale in
meters; deck bottom on z=0.
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

# Waterfall channel (in the spout part frame, origin at column top)
CHANNEL_W = 0.052  # width along X
CHANNEL_H = 0.014  # total channel wall height along Z
CHANNEL_L = 0.078  # length along Y (extends forward)
CHANNEL_WALL = 0.004  # side and bottom wall thickness
CHANNEL_FLOOR_T = 0.003  # floor thickness
# Collar at swivel joint
COLLAR_R = 0.028
COLLAR_H = 0.014
# Forward lip drop: the channel slopes down slightly at the outlet
LIP_DROP = 0.006

# Deck seam ring dimensions (inner R must be < flange R for connectivity)
SEAM_OUTER_R = 0.045  # slightly larger than flange (0.042)
SEAM_WIDTH = 0.004  # ring width; inner R = 0.041 < flange R = 0.042

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

# Valve seam ring dimensions (inner R must be < flange R for connectivity)
VALVE_SEAM_OUTER_R = 0.039  # slightly larger than valve flange (0.036)
VALVE_SEAM_WIDTH = 0.004  # ring width; inner R = 0.035 < flange R = 0.036


def _waterfall_channel() -> cq.Workplane:
    """Short rectangular waterfall channel: U-shaped open trough extending forward.

    The channel origin is at the swivel joint (column top). It extends along +Y
    from the column center. The front lip drops slightly below the channel floor
    to create the waterfall cascade edge.
    """
    # Shift everything up by LIP_DROP so the lip bottom sits at z=0
    z_off = LIP_DROP
    # Outer box
    outer = (
        cq.Workplane("XY")
        .workplane(offset=z_off)
        .box(CHANNEL_W, CHANNEL_L, CHANNEL_H, centered=(True, False, False))
    )
    # Inner cavity (hollow trough) - slightly smaller, shifted up for floor thickness
    inner_w = CHANNEL_W - 2 * CHANNEL_WALL
    inner_l = CHANNEL_L - CHANNEL_WALL  # open at the front end
    inner_h = CHANNEL_H - CHANNEL_FLOOR_T
    inner = (
        cq.Workplane("XY")
        .workplane(offset=z_off + CHANNEL_FLOOR_T)
        .box(inner_w, inner_l, inner_h, centered=(True, False, False))
    )
    channel = outer.cut(inner)

    # Front lip drop: extends from z=0 up to the channel floor at the front edge
    lip = (
        cq.Workplane("XY")
        .transformed(offset=(0, CHANNEL_L - CHANNEL_WALL, 0))
        .box(CHANNEL_W, CHANNEL_WALL, z_off + CHANNEL_FLOOR_T, centered=(True, False, False))
    )
    channel = channel.union(lip)

    return channel


def _deck_seam_ring(outer_r: float, width: float, height: float) -> cq.Workplane:
    """Thin annular ring representing a visible seam at the deck base."""
    inner_r = outer_r - width
    ring = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(height)
    )
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_waterfall_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.08, 0.08, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.02, 0.02, 0.02, 1.0))

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
    # Narrow seam ring at spout deck base: thin dark annulus seated between
    # deck surface and flange bottom; overlaps flange slightly for connectivity.
    spout_base.visual(
        mesh_from_cadquery(
            _deck_seam_ring(SEAM_OUTER_R, SEAM_WIDTH, 0.002),
            "spout_deck_seam",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
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

    # -------------------------------------------------------- waterfall spout
    waterfall_spout = model.part("waterfall_spout")
    waterfall_spout.visual(
        mesh_from_cadquery(_waterfall_channel(), "waterfall_channel"),
        material=matte_black,
        name="spout_channel",
    )
    waterfall_spout.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H / 2)),
        material=matte_black,
        name="swivel_collar",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=spout_base,
        child=waterfall_spout,
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
        # Narrow seam ring at valve deck base: thin dark annulus seated between
        # deck surface and flange bottom; overlaps flange slightly for connectivity.
        col.visual(
            mesh_from_cadquery(
                _deck_seam_ring(VALVE_SEAM_OUTER_R, VALVE_SEAM_WIDTH, 0.002),
                f"{name}_deck_seam",
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
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
    waterfall = object_model.get_part("waterfall_spout")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_lever = object_model.get_part("hot_lever")
    cold_lever = object_model.get_part("cold_lever")

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_turn = object_model.get_articulation("hot_lever_turn")
    cold_turn = object_model.get_articulation("cold_lever_turn")

    spout_channel = waterfall.get_visual("spout_channel")
    base_column = spout_base.get_visual("base_column")

    # Intentional hidden engagements: lever stems seat inside their columns
    # so the rotating parts read as mounted, not floating.
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

    # --- waterfall channel form: rectangular, extends forward, wider than tall
    channel_aabb = ctx.part_element_world_aabb(waterfall, elem=spout_channel)
    channel_dx = channel_aabb[1][0] - channel_aabb[0][0]
    channel_dy = channel_aabb[1][1] - channel_aabb[0][1]
    channel_dz = channel_aabb[1][2] - channel_aabb[0][2]
    ctx.check(
        "waterfall_channel_wider_than_tall",
        channel_dx > channel_dz * 2.0,
        f"channel width={channel_dx:.4f} height={channel_dz:.4f}",
    )
    ctx.check(
        "waterfall_channel_extends_forward",
        channel_dy > 0.05,
        f"channel forward reach={channel_dy:.4f}",
    )
    ctx.check(
        "waterfall_channel_short_rectangular_form",
        channel_dz < 0.04 and channel_dy < 0.12 and channel_dx < 0.08,
        f"channel dims=({channel_dx:.4f},{channel_dy:.4f},{channel_dz:.4f})",
    )
    # Channel outlet is above the deck surface
    channel_outlet_z = channel_aabb[0][2] - DECK_T
    ctx.check(
        "waterfall_outlet_above_deck",
        channel_outlet_z > 0.08,
        f"channel bottom above deck={channel_outlet_z:.4f}",
    )

    # --- deck seams present at all three bases -----------------------------
    spout_seam = spout_base.get_visual("deck_seam")
    hot_seam = hot_col.get_visual("deck_seam")
    cold_seam = cold_col.get_visual("deck_seam")
    ctx.check(
        "spout_base_has_deck_seam",
        spout_seam is not None,
        "missing deck seam on spout base",
    )
    ctx.check(
        "hot_valve_has_deck_seam",
        hot_seam is not None,
        "missing deck seam on hot valve",
    )
    ctx.check(
        "cold_valve_has_deck_seam",
        cold_seam is not None,
        "missing deck seam on cold valve",
    )
    # Seam rings are thin annular shapes wider than the column
    for piece, seam_name in (
        (spout_base, "spout seam"),
        (hot_col, "hot valve seam"),
        (cold_col, "cold valve seam"),
    ):
        seam_vis = piece.get_visual("deck_seam")
        seam_aabb = ctx.part_element_world_aabb(piece, elem=seam_vis)
        seam_dx = seam_aabb[1][0] - seam_aabb[0][0]
        seam_dz = seam_aabb[1][2] - seam_aabb[0][2]
        ctx.check(
            f"{seam_name}_is_thin_ring",
            seam_dz < 0.005 and seam_dx > 0.06,
            f"{seam_name} height={seam_dz:.5f} width={seam_dx:.4f}",
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

    # Spout swivel: +45 deg swings the channel toward -X (right-hand rule
    # about +Z), keeping its height unchanged.
    with ctx.pose({spout_swivel: 0.0}):
        ch0 = ctx.part_element_world_aabb(waterfall, elem=spout_channel)
    with ctx.pose({spout_swivel: math.pi / 4}):
        ch45 = ctx.part_element_world_aabb(waterfall, elem=spout_channel)
    ch0_cx = 0.5 * (ch0[0][0] + ch0[1][0])
    ch45_cx = 0.5 * (ch45[0][0] + ch45[1][0])
    ch0_cz = 0.5 * (ch0[0][2] + ch0[1][2])
    ch45_cz = 0.5 * (ch45[0][2] + ch45[1][2])
    ctx.check(
        "spout_swivels_about_column_axis",
        ch45_cx < ch0_cx - 0.02 and abs(ch45_cz - ch0_cz) < 1e-3,
        f"rest cx={ch0_cx:.4f} swiveled cx={ch45_cx:.4f} dz={abs(ch45_cz - ch0_cz):.5f}",
    )

    return ctx.report()


object_model = build_object_model()
