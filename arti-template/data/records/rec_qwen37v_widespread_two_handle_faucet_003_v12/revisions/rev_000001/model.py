from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet with low bridge arch.

Three-piece widespread layout (0.30 m spread) on a sink deck:
- center: short column with a low bridge arch spanning between handles,
  plus a prismatic diverter knob behind the arch
- hot (left) and cold (right): valve columns with T-style lever handles
  (revolute, -90..+90 deg about vertical)
- narrow seam rings at all three deck bases

All surfaces matte black; red/blue indicator dots on handle stems.
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

# ---- deck ----
DECK_X = 0.46
DECK_Y = 0.18
DECK_T = 0.02

SPREAD_HALF = 0.15

# ---- seam rings ----
SEAM_H = 0.001
SEAM_EXTRA = 0.002

# ---- center spout base ----
SPOUT_FLANGE_R = 0.042
SPOUT_FLANGE_H = 0.012
SPOUT_COL_R = 0.025
SPOUT_COL_H = 0.045

# ---- bridge arch ----
BRIDGE_HALF_SPAN = 0.10
BRIDGE_RISE = 0.060
BRIDGE_TUBE_R = 0.012

# ---- spout nozzle ----
NOZZLE_R = 0.014
NOZZLE_LEN = 0.020

# ---- diverter ----
DIV_Y_OFF = -0.035
DIV_STEM_R = 0.006
DIV_STEM_H = 0.040
DIV_KNOB_R = 0.012
DIV_KNOB_H = 0.015
DIV_SLIDE = 0.030

# ---- valve columns ----
VALVE_FLANGE_R = 0.036
VALVE_FLANGE_H = 0.010
VALVE_COL_R = 0.0225
VALVE_COL_H = 0.10

# ---- T-lever ----
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.045
BAR_R = 0.0095
BAR_LEN = 0.12
BAR_OFF = 0.025
DOT_R = 0.0035

# Derived z-offsets within part-local frames
_SPOUT_BASE_TOP = SEAM_H + SPOUT_FLANGE_H + SPOUT_COL_H
_VALVE_TOP = SEAM_H + VALVE_FLANGE_H + VALVE_COL_H

# Arch riser: extends from below column top up to arch underside
_RISER_EMBED = 0.010  # hidden embed into column
_RISER_H = BRIDGE_RISE - BRIDGE_TUBE_R + _RISER_EMBED  # total riser length

# Diverter boss: mount housing on spout base behind the column
_BOSS_H = 0.014
_BOSS_EMBED_Z = _SPOUT_BASE_TOP - (_BOSS_H - 0.002)  # boss top 2mm above column top


def _bridge_arch_solid() -> cq.Workplane:
    """Swept bridge arch tube: semicircular arc in XZ from -span to +span."""
    path = (
        cq.Workplane("XZ")
        .moveTo(-BRIDGE_HALF_SPAN, 0.0)
        .threePointArc((0.0, BRIDGE_RISE), (BRIDGE_HALF_SPAN, 0.0))
    )
    profile = (
        cq.Workplane("YZ")
        .workplane(offset=-BRIDGE_HALF_SPAN)
        .circle(BRIDGE_TUBE_R)
    )
    return profile.sweep(path, isFrenet=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_bridge_arch_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.02, 0.02, 0.02, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.08, 0.08, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))

    # -- sink deck --
    sink_deck = model.part("sink_deck")
    sink_deck.visual(
        Box((DECK_X, DECK_Y, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, DECK_T / 2)),
        material=deck_stone,
        name="deck_slab",
    )

    # -- center spout base (with seam ring) --
    spout_base = model.part("spout_base")
    spout_base.visual(
        Cylinder(radius=SPOUT_FLANGE_R + SEAM_EXTRA, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2)),
        material=seam_dark,
        name="base_seam",
    )
    spout_base.visual(
        Cylinder(radius=SPOUT_FLANGE_R, length=SPOUT_FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H + SPOUT_FLANGE_H / 2)),
        material=matte_black,
        name="base_flange",
    )
    spout_base.visual(
        Cylinder(radius=SPOUT_COL_R, length=SPOUT_COL_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H + SPOUT_FLANGE_H + SPOUT_COL_H / 2)),
        material=matte_black,
        name="base_column",
    )
    # Diverter mounting boss behind the column
    spout_base.visual(
        Cylinder(radius=0.015, length=_BOSS_H),
        origin=Origin(xyz=(0.0, DIV_Y_OFF, _BOSS_EMBED_Z + _BOSS_H / 2)),
        material=matte_black,
        name="diverter_boss",
    )

    model.articulation(
        "deck_to_spout_base",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=spout_base,
        origin=Origin(xyz=(0.0, 0.0, DECK_T)),
    )

    # -- bridge arch spout --
    bridge_spout = model.part("bridge_spout")
    bridge_spout.visual(
        mesh_from_cadquery(_bridge_arch_solid(), "arch_tube"),
        material=matte_black,
        name="arch_tube",
    )
    # Central riser connecting column top to arch underside
    bridge_spout.visual(
        Cylinder(radius=0.015, length=_RISER_H),
        origin=Origin(xyz=(0.0, 0.0, _RISER_H / 2 - _RISER_EMBED)),
        material=matte_black,
        name="arch_riser",
    )
    # Rounded end caps at arch feet
    for sign, side in ((-1.0, "left"), (1.0, "right")):
        bridge_spout.visual(
            Sphere(radius=BRIDGE_TUBE_R),
            origin=Origin(xyz=(sign * BRIDGE_HALF_SPAN, 0.0, 0.0)),
            material=matte_black,
            name=f"arch_foot_{side}",
        )
    # Downward nozzle at arch apex
    bridge_spout.visual(
        Cylinder(radius=NOZZLE_R, length=NOZZLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_RISE - NOZZLE_LEN / 2)),
        material=matte_black,
        name="spout_nozzle",
    )

    model.articulation(
        "spout_base_to_bridge",
        ArticulationType.FIXED,
        parent=spout_base,
        child=bridge_spout,
        origin=Origin(xyz=(0.0, 0.0, _SPOUT_BASE_TOP)),
    )

    # -- diverter knob (prismatic up-down behind the bridge) --
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
    diverter.visual(
        Sphere(radius=DIV_KNOB_R),
        origin=Origin(xyz=(0.0, 0.0, DIV_STEM_H + DIV_KNOB_H)),
        material=matte_black,
        name="diverter_dome",
    )

    model.articulation(
        "diverter_slide",
        ArticulationType.PRISMATIC,
        parent=spout_base,
        child=diverter,
        origin=Origin(xyz=(0.0, DIV_Y_OFF, _SPOUT_BASE_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.5, lower=0.0, upper=DIV_SLIDE
        ),
    )

    # -- valve columns (with seam rings) --
    def _valve(name: str) -> object:
        col = model.part(name)
        col.visual(
            Cylinder(radius=VALVE_FLANGE_R + SEAM_EXTRA, length=SEAM_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2)),
            material=seam_dark,
            name="valve_seam",
        )
        col.visual(
            Cylinder(radius=VALVE_FLANGE_R, length=VALVE_FLANGE_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_H + VALVE_FLANGE_H / 2)),
            material=matte_black,
            name="valve_flange",
        )
        col.visual(
            Cylinder(radius=VALVE_COL_R, length=VALVE_COL_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_H + VALVE_FLANGE_H + VALVE_COL_H / 2)),
            material=matte_black,
            name="valve_body",
        )
        return col

    def _lever(name: str, bar_off: float, dot_mat: object) -> object:
        lev = model.part(name)
        lev.visual(
            Cylinder(radius=STEM_R, length=STEM_TOP + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (STEM_TOP - STEM_EMBED) / 2)),
            material=matte_black,
            name="lever_stem",
        )
        lev.visual(
            Cylinder(radius=BAR_R, length=BAR_LEN),
            origin=Origin(
                xyz=(bar_off, 0.0, STEM_TOP), rpy=(0.0, math.pi / 2, 0.0)
            ),
            material=matte_black,
            name="lever_bar",
        )
        for end in (-1.0, 1.0):
            lev.visual(
                Sphere(radius=BAR_R),
                origin=Origin(
                    xyz=(bar_off + end * BAR_LEN / 2, 0.0, STEM_TOP)
                ),
                material=matte_black,
                name=f"bar_cap_{'outer' if end * bar_off > 0 else 'inner'}",
            )
        lev.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, STEM_R - 0.0005, 0.022)),
            material=dot_mat,
            name="indicator_dot",
        )
        return lev

    hot_col = _valve("hot_valve_column")
    cold_col = _valve("cold_valve_column")
    hot_lever = _lever("hot_lever", -BAR_OFF, hot_red)
    cold_lever = _lever("cold_lever", BAR_OFF, cold_blue)

    model.articulation(
        "deck_to_hot_valve",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=hot_col,
        origin=Origin(xyz=(-SPREAD_HALF, 0.0, DECK_T)),
    )
    model.articulation(
        "deck_to_cold_valve",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=cold_col,
        origin=Origin(xyz=(SPREAD_HALF, 0.0, DECK_T)),
    )

    for jname, par, chi in (
        ("hot_lever_turn", hot_col, hot_lever),
        ("cold_lever_turn", cold_col, cold_lever),
    ):
        model.articulation(
            jname,
            ArticulationType.REVOLUTE,
            parent=par,
            child=chi,
            origin=Origin(xyz=(0.0, 0.0, _VALVE_TOP)),
            axis=(0.0, 0.0, 1.0),
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
    bridge = object_model.get_part("bridge_spout")
    diverter = object_model.get_part("diverter_knob")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_lever = object_model.get_part("hot_lever")
    cold_lever = object_model.get_part("cold_lever")

    hot_turn = object_model.get_articulation("hot_lever_turn")
    cold_turn = object_model.get_articulation("cold_lever_turn")
    div_slide = object_model.get_articulation("diverter_slide")

    # Overlap allowances: lever stems embed into valve columns
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.allow_overlap(
            lever,
            col,
            elem_a=lever.get_visual("lever_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="lever stem seats 15 mm into the valve cartridge bore",
        )

    # Arch riser embeds into the spout column for structural mounting
    ctx.allow_overlap(
        bridge,
        spout_base,
        elem_a=bridge.get_visual("arch_riser"),
        elem_b=spout_base.get_visual("base_column"),
        reason="arch riser seats 10 mm into the spout column bore for structural support",
    )
    ctx.expect_contact(
        bridge,
        spout_base,
        contact_tol=1e-4,
        name="bridge_arch_mounted_on_column",
    )

    # Diverter stem seats into the mounting boss
    ctx.allow_overlap(
        spout_base,
        diverter,
        elem_a=spout_base.get_visual("diverter_boss"),
        elem_b=diverter.get_visual("diverter_stem"),
        reason="diverter stem seats 2 mm into the mounting boss housing at rest",
    )

    # --- Bridge arch is low (not a tall gooseneck) ---
    bridge_aabb = ctx.part_world_aabb(bridge)
    arch_apex_above_deck = bridge_aabb[1][2] - DECK_T
    ctx.check(
        "bridge_arch_is_low",
        arch_apex_above_deck < 0.18,
        f"arch apex {arch_apex_above_deck:.3f} m above deck (should be < 0.18)",
    )
    arch_x_span = bridge_aabb[1][0] - bridge_aabb[0][0]
    ctx.check(
        "bridge_arch_spans_between_handles",
        arch_x_span > 0.15,
        f"arch X span = {arch_x_span:.3f} m (should be > 0.15)",
    )

    # --- Diverter prismatic joint ---
    ctx.check(
        "diverter_is_prismatic",
        str(div_slide.joint_type).lower().endswith("prismatic"),
        f"type={div_slide.joint_type}",
    )
    ctx.check(
        "diverter_axis_vertical",
        tuple(div_slide.axis) == (0.0, 0.0, 1.0),
        f"axis={div_slide.axis}",
    )
    div_ml = div_slide.motion_limits
    ctx.check(
        "diverter_slide_range",
        div_ml is not None
        and abs(div_ml.lower) < 1e-6
        and abs(div_ml.upper - DIV_SLIDE) < 1e-6,
        f"lower={div_ml.lower} upper={div_ml.upper}",
    )

    # Diverter actually slides upward at positive q
    rest_pos = ctx.part_world_position(diverter)
    with ctx.pose({div_slide: DIV_SLIDE}):
        ext_pos = ctx.part_world_position(diverter)
    ctx.check(
        "diverter_slides_upward",
        rest_pos is not None
        and ext_pos is not None
        and ext_pos[2] > rest_pos[2] + 0.02,
        f"rest_z={rest_pos[2] if rest_pos else None} "
        f"ext_z={ext_pos[2] if ext_pos else None}",
    )

    # --- Seam rings at all three deck bases ---
    for part_obj, seam_name in (
        (spout_base, "base_seam"),
        (hot_col, "valve_seam"),
        (cold_col, "valve_seam"),
    ):
        seam = part_obj.get_visual(seam_name)
        ctx.check(
            f"{part_obj.name}_has_seam",
            seam is not None,
            f"missing visual {seam_name} on {part_obj.name}",
        )

    # --- Lever joints: type, axis, range ---
    for joint, lim in ((hot_turn, math.pi / 2), (cold_turn, math.pi / 2)):
        ctx.check(
            f"{joint.name}_is_revolute",
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

    # --- 0.30 m spread, all three pieces centered correctly ---
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

    # All bases seated on deck
    for piece in (spout_base, hot_col, cold_col):
        ctx.expect_contact(piece, deck, contact_tol=1e-5)

    # --- Nozzle hangs below arch apex ---
    nozzle_aabb = ctx.part_element_world_aabb(
        bridge, elem=bridge.get_visual("spout_nozzle")
    )
    nozzle_bottom_z = nozzle_aabb[0][2]
    arch_top_z = bridge_aabb[1][2]
    ctx.check(
        "nozzle_below_arch_apex",
        nozzle_bottom_z < arch_top_z - 0.01,
        f"nozzle bottom z={nozzle_bottom_z:.3f} arch top z={arch_top_z:.3f}",
    )

    # --- Lever rotation proof (off-axis at +90 deg) ---
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
        "hot_lever_rotates_about_vertical",
        span_x0 > 0.10
        and span_y0 < 0.03
        and span_y90 > 0.10
        and span_x90 < 0.03,
        f"closed=({span_x0:.3f},{span_y0:.3f}) "
        f"turned=({span_x90:.3f},{span_y90:.3f})",
    )

    # Indicator dots: red on hot, blue on cold
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(
            f"{lever.name}_dot_material",
            mat_name == mat,
            f"material={mat_name}",
        )

    return ctx.report()


object_model = build_object_model()
