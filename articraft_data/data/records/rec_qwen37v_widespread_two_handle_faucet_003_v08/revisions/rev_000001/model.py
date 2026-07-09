from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet with waterfall spout.

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a short rectangular waterfall channel
  spout (swivels about the column's vertical axis, -45..+45 deg),
- hot (left) and cold (right): valve columns topped by T-style lever handles
  that tilt forward-back (each revolute about the spread axis, -90..+90 deg).

Narrow seam rings at all three deck bases. Hollow waterfall channel with a
visible central outlet slot. All faucet surfaces matte black; tiny red/blue
indicator dots on the handle stems. Modeled at true scale in meters; deck
bottom on z=0.
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
CH_W = 0.042  # channel outer width (X)
CH_L = 0.090  # channel outer length (Y, extends forward)
CH_H = 0.022  # channel outer height (Z)
CH_WALL = 0.005  # side wall thickness
CH_FLOOR = 0.006  # bottom floor thickness
CH_SLOT_W = CH_W - 2 * CH_WALL  # hollow slot width
CH_SLOT_L = CH_L - CH_WALL  # slot length (open at front for waterfall lip)
CH_SLOT_D = CH_H - CH_FLOOR  # slot depth from top
COLLAR_R = 0.020
COLLAR_H = 0.016

# Seam ring dimensions
SEAM_H = 0.0025  # seam ring height
SEAM_EXTRA = 0.003  # how much wider than the flange

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


def _waterfall_channel() -> cq.Workplane:
    """Rectangular waterfall spout channel with hollow interior slot."""
    # Outer shell box, centered at (0, CH_L/2, CH_H/2) so it extends forward
    outer = (
        cq.Workplane("XY")
        .box(CH_W, CH_L, CH_H, centered=(True, False, False))
        .translate((0.0, 0.0, 0.0))
    )
    # Hollow slot cut from the top, leaving walls on sides and bottom.
    # The slot extends to the front edge (open at front = waterfall lip).
    slot_x = CH_SLOT_W
    slot_y = CH_SLOT_L
    slot_z = CH_SLOT_D
    # Position: centered in X, starting from y=CH_WALL (back wall), extending
    # to front edge, from z=CH_FLOOR to z=CH_H
    cutter = (
        cq.Workplane("XY")
        .workplane(offset=CH_FLOOR)
        .center(0.0, CH_WALL + slot_y / 2)
        .rect(slot_x, slot_y)
        .extrude(slot_z)
    )
    return outer.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_waterfall_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.08, 0.08, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.04, 0.04, 0.04, 1.0))

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
    # Seam ring at spout base
    spout_base.visual(
        Cylinder(radius=SPOUT_FLANGE_R + SEAM_EXTRA, length=SEAM_H),
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

    # --------------------------------------------------- waterfall spout
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
        # Seam ring at valve base
        col.visual(
            Cylinder(radius=VALVE_FLANGE_R + SEAM_EXTRA, length=SEAM_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2)),
            material=seam_dark,
            name="valve_seam",
        )
        return col

    def _t_lever(name: str, bar_off: float, dot_material: object) -> object:
        """T-lever with bar extending along Y (front-back) for forward-back tilt."""
        lever = model.part(name)
        lever.visual(
            Cylinder(radius=STEM_R, length=STEM_TOP + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (STEM_TOP - STEM_EMBED) / 2)),
            material=matte_black,
            name="lever_stem",
        )
        # Horizontal T-bar along Y (front-back); off-center so it overhangs
        # toward the user (+Y). Rotate 90 deg about X so cylinder aligns with Y.
        lever.visual(
            Cylinder(radius=BAR_R, length=BAR_LEN),
            origin=Origin(
                xyz=(0.0, bar_off, STEM_TOP),
                rpy=(math.pi / 2, 0.0, 0.0),
            ),
            material=matte_black,
            name="lever_bar",
        )
        for end in (-1.0, 1.0):
            lever.visual(
                Sphere(radius=BAR_R),
                origin=Origin(xyz=(0.0, bar_off + end * BAR_LEN / 2, STEM_TOP)),
                material=matte_black,
                name=f"bar_cap_{'front' if end * bar_off > 0 else 'rear'}",
            )
        # Tiny temperature indicator dot on the front of the stem.
        lever.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(STEM_R - 0.0005, 0.0, 0.022)),
            material=dot_material,
            name="indicator_dot",
        )
        return lever

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    # Hot on the left (-X), cold on the right (+X); +Y is toward the user.
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

    # Lever handles tilt forward-back: axis along X so positive q tilts the
    # front end (+Y side of bar) upward.
    for joint_name, parent, child in (
        ("hot_lever_tilt", hot_valve_column, hot_lever),
        ("cold_lever_tilt", cold_valve_column, cold_lever),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=parent,
            child=child,
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H)),
            axis=(1.0, 0.0, 0.0),
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
    hot_tilt = object_model.get_articulation("hot_lever_tilt")
    cold_tilt = object_model.get_articulation("cold_lever_tilt")

    spout_channel = waterfall.get_visual("spout_channel")
    base_column = spout_base.get_visual("base_column")

    # Intentional hidden engagements: spout collar and lever stems seat inside
    # their columns so the rotating parts read as mounted, not floating.
    ctx.allow_overlap(
        waterfall,
        spout_base,
        elem_a=waterfall.get_visual("swivel_collar"),
        elem_b=base_column,
        reason="swivel collar seats 16 mm into the base column top bore",
    )
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.allow_overlap(
            lever,
            col,
            elem_a=lever.get_visual("lever_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="lever stem seats 15 mm into the valve cartridge bore",
        )

    # --- joint plan: spout swivel is vertical revolute ---------------------
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

    # --- lever joints are forward-back revolute (axis along X) -------------
    for joint, lim in ((hot_tilt, math.pi / 2), (cold_tilt, math.pi / 2)):
        ctx.check(
            f"{joint.name}_is_forward_back_revolute",
            str(joint.joint_type).lower().endswith("revolute")
            and tuple(joint.axis) == (1.0, 0.0, 0.0),
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

    # --- waterfall channel: rectangular, extends forward, has hollow slot --
    channel_aabb = ctx.part_element_world_aabb(waterfall, elem=spout_channel)
    ch_dx = channel_aabb[1][0] - channel_aabb[0][0]
    ch_dy = channel_aabb[1][1] - channel_aabb[0][1]
    ch_dz = channel_aabb[1][2] - channel_aabb[0][2]
    ctx.check(
        "waterfall_channel_is_rectangular",
        ch_dx > 0.030 and ch_dy > 0.060 and 0.010 < ch_dz < 0.035,
        f"channel dims=({ch_dx:.4f}, {ch_dy:.4f}, {ch_dz:.4f})",
    )
    # Channel is wider than tall (flat rectangular profile)
    ctx.check(
        "waterfall_channel_wider_than_tall",
        ch_dx > ch_dz * 1.5,
        f"width={ch_dx:.4f} height={ch_dz:.4f}",
    )
    # Channel extends forward from the column center
    channel_center_y = 0.5 * (channel_aabb[0][1] + channel_aabb[1][1])
    ctx.check(
        "waterfall_channel_extends_forward",
        channel_center_y > 0.02,
        f"channel center y={channel_center_y:.4f}",
    )

    # --- seam rings present at all three deck bases ------------------------
    for part_obj, seam_name in (
        (spout_base, "base_seam"),
        (hot_col, "valve_seam"),
        (cold_col, "valve_seam"),
    ):
        seam = part_obj.get_visual(seam_name)
        ctx.check(
            f"{part_obj.name}_has_seam_ring",
            seam is not None,
            f"missing visual '{seam_name}'",
        )

    # --- seam rings sit at deck level (near z=0 in part-local frame) -------
    for part_obj, seam_name in (
        (spout_base, "base_seam"),
        (hot_col, "valve_seam"),
        (cold_col, "valve_seam"),
    ):
        seam_aabb = ctx.part_element_world_aabb(part_obj, elem=part_obj.get_visual(seam_name))
        seam_z_min = seam_aabb[0][2]
        ctx.check(
            f"{part_obj.name}_seam_at_deck_level",
            abs(seam_z_min - DECK_T) < 0.005,
            f"seam z_min={seam_z_min:.4f} vs deck_top={DECK_T}",
        )

    # --- lever form: bar extends front-back, overhangs toward user ---------
    for lever, col_x_sign in ((hot_lever, -1.0), (cold_lever, 1.0)):
        bar_aabb = ctx.part_element_world_aabb(lever, elem=lever.get_visual("lever_bar"))
        bar_span_y = bar_aabb[1][1] - bar_aabb[0][1]
        bar_span_x = bar_aabb[1][0] - bar_aabb[0][0]
        ctx.check(
            f"{lever.name}_bar_extends_front_back",
            bar_span_y > 0.08 and bar_span_x < 0.03,
            f"bar span x={bar_span_x:.3f} y={bar_span_y:.3f}",
        )
        # Bar clears the valve column top (only the stem enters the column).
        ctx.expect_gap(
            lever,
            (hot_col if col_x_sign < 0 else cold_col),
            axis="z",
            positive_elem=lever.get_visual("lever_bar"),
            min_gap=0.02,
        )

    # Indicator dots: red on hot, blue on cold, proud of the stem side.
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}")

    # --- articulation behavior: lever tilts forward-back -------------------
    # At q=0 the bar spans Y (front-back); at q=+90 deg it should stand vertical
    # (the front end goes up).
    with ctx.pose({hot_tilt: 0.0}):
        bar0 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("lever_bar"))
    with ctx.pose({hot_tilt: math.pi / 2}):
        bar90 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("lever_bar"))
    span_y0 = bar0[1][1] - bar0[0][1]
    span_z0 = bar0[1][2] - bar0[0][2]
    span_y90 = bar90[1][1] - bar90[0][1]
    span_z90 = bar90[1][2] - bar90[0][2]
    ctx.check(
        "hot_lever_tilts_forward_back",
        span_y0 > 0.08 and span_z0 < 0.03 and span_z90 > 0.08 and span_y90 < 0.03,
        f"rest span=(y:{span_y0:.3f},z:{span_z0:.3f}) tilted span=(y:{span_y90:.3f},z:{span_z90:.3f})",
    )

    # Spout swivel: +45 deg swings the forward channel toward -X.
    with ctx.pose({spout_swivel: math.pi / 4}):
        ch45 = ctx.part_element_world_aabb(waterfall, elem=spout_channel)
    ch45_cx = 0.5 * (ch45[0][0] + ch45[1][0])
    ch0_cx = 0.5 * (channel_aabb[0][0] + channel_aabb[1][0])
    ctx.check(
        "spout_swivels_about_column_axis",
        ch45_cx < ch0_cx - 0.02,
        f"rest cx={ch0_cx:.3f} swiveled cx={ch45_cx:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
