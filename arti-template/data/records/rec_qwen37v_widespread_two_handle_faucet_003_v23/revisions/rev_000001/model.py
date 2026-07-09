from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet set (variant 23).

Three-piece widespread layout on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (revolute about the column's vertical axis, -45..+45 deg),
  plus a prismatic diverter knob behind the column that slides up-down.
- hot (left) and cold (right): cylindrical valve columns on round flanges,
  each topped by a cross-shaped handle with a visible stem collar.
  Each handle is an independent revolute joint (-90..+90 deg about Z).
- Small hexagonal mounting nuts visible below each base flange.

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
HANDLE_STEM_R = 0.009
HANDLE_STEM_EMBED = 0.015
HANDLE_STEM_H = 0.040  # stem height above column top
STEM_COLLAR_R = 0.016  # visible collar ring under the cross
STEM_COLLAR_H = 0.008
CROSS_BAR_R = 0.008
CROSS_BAR_LEN = 0.09  # each bar of the cross
CROSS_HUB_R = 0.012
CROSS_HUB_H = 0.014
DOT_R = 0.0035

# Diverter knob (behind spout column, prismatic)
DIVERTER_KNOB_R = 0.012
DIVERTER_KNOB_H = 0.018
DIVERTER_STEM_R = 0.005
DIVERTER_STEM_H = 0.020  # stem that protrudes behind the column
DIVERTER_TRAVEL = 0.025  # max upward travel

# Underside nuts
NUT_AF = 0.014  # across-flats of the hex nut
NUT_H = 0.008

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


def _hex_nut_solid(af: float, height: float) -> cq.Workplane:
    """Hexagonal nut with a through-hole, centered at the origin."""
    # Build a hex prism
    r = af / 2.0 / math.cos(math.radians(30))  # circumradius from across-flats
    hex_body = (
        cq.Workplane("XY")
        .polygon(6, r * 2)
        .extrude(height)
    )
    # Through-hole in the center
    hole_r = af * 0.35
    hex_body = hex_body.faces(">Z").workplane().circle(hole_r).cutThruAll()
    # Center at origin (extrude goes from z=0 to z=height)
    hex_body = hex_body.translate((0, 0, -height / 2))
    return hex_body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet_v23")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.08, 0.08, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))
    zinc_alloy = model.material("zinc_alloy", rgba=(0.55, 0.55, 0.52, 1.0))

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
    # Underside nut for spout base (top flush with flange bottom at z=0)
    spout_base.visual(
        mesh_from_cadquery(_hex_nut_solid(NUT_AF, NUT_H), "spout_nut"),
        origin=Origin(xyz=(0.0, 0.0, -NUT_H / 2)),
        material=zinc_alloy,
        name="underside_nut",
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

    # ------------------------------------------------------- diverter knob
    # Mounted behind the spout column (-Y), prismatic up-down
    diverter_knob = model.part("diverter_knob")
    # Horizontal stem from the column back toward the user (-Y).
    # Embed inner end 5 mm into the column bore for a connected mount.
    _div_embed = 0.005
    _div_stem_cy = -(SPOUT_COL_R - _div_embed + DIVERTER_STEM_H / 2)
    diverter_knob.visual(
        Cylinder(radius=DIVERTER_STEM_R, length=DIVERTER_STEM_H),
        origin=Origin(
            xyz=(0.0, _div_stem_cy, 0.0),
            rpy=(math.pi / 2, 0.0, 0.0),
        ),
        material=matte_black,
        name="diverter_stem",
    )
    # Vertical knob cap at the outer end of the stem (shifted inward 5mm for
    # clear mesh connectivity with the stem)
    _div_cap_cy = -(SPOUT_COL_R - _div_embed + DIVERTER_STEM_H - 0.005)
    diverter_knob.visual(
        Cylinder(radius=DIVERTER_KNOB_R, length=DIVERTER_KNOB_H),
        origin=Origin(xyz=(0.0, _div_cap_cy, 0.0)),
        material=matte_black,
        name="diverter_cap",
    )

    model.articulation(
        "diverter_slide",
        ArticulationType.PRISMATIC,
        parent=spout_base,
        child=diverter_knob,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_COL_H * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.5, lower=0.0, upper=DIVERTER_TRAVEL
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
        # Underside nut (top flush with flange bottom at z=0)
        col.visual(
            mesh_from_cadquery(_hex_nut_solid(NUT_AF, NUT_H), f"{name}_nut"),
            origin=Origin(xyz=(0.0, 0.0, -NUT_H / 2)),
            material=zinc_alloy,
            name="underside_nut",
        )
        return col

    def _cross_handle(name: str, dot_material: object) -> object:
        handle = model.part(name)
        # Stem that embeds into column bore
        handle.visual(
            Cylinder(radius=HANDLE_STEM_R, length=HANDLE_STEM_H + HANDLE_STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (HANDLE_STEM_H - HANDLE_STEM_EMBED) / 2)),
            material=matte_black,
            name="handle_stem",
        )
        # Visible stem collar under the cross
        handle.visual(
            Cylinder(radius=STEM_COLLAR_R, length=STEM_COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, HANDLE_STEM_H - STEM_COLLAR_H / 2)),
            material=matte_black,
            name="stem_collar",
        )
        # Cross hub at top of stem
        hub_z = HANDLE_STEM_H + CROSS_HUB_H / 2
        handle.visual(
            Cylinder(radius=CROSS_HUB_R, length=CROSS_HUB_H),
            origin=Origin(xyz=(0.0, 0.0, hub_z)),
            material=matte_black,
            name="cross_hub",
        )
        # Two perpendicular bars forming the cross (+ shape)
        bar_z = HANDLE_STEM_H + CROSS_HUB_H / 2
        # Bar along X
        handle.visual(
            Cylinder(radius=CROSS_BAR_R, length=CROSS_BAR_LEN),
            origin=Origin(xyz=(0.0, 0.0, bar_z), rpy=(0.0, math.pi / 2, 0.0)),
            material=matte_black,
            name="cross_bar_x",
        )
        # Bar along Y
        handle.visual(
            Cylinder(radius=CROSS_BAR_R, length=CROSS_BAR_LEN),
            origin=Origin(xyz=(0.0, 0.0, bar_z), rpy=(math.pi / 2, 0.0, 0.0)),
            material=matte_black,
            name="cross_bar_y",
        )
        # End caps on bars
        for axis_end in [
            (CROSS_BAR_LEN / 2, 0.0, bar_z),
            (-CROSS_BAR_LEN / 2, 0.0, bar_z),
            (0.0, CROSS_BAR_LEN / 2, bar_z),
            (0.0, -CROSS_BAR_LEN / 2, bar_z),
        ]:
            handle.visual(
                Sphere(radius=CROSS_BAR_R),
                origin=Origin(xyz=axis_end),
                material=matte_black,
                name=f"bar_cap_{axis_end[0]:.3f}_{axis_end[1]:.3f}",
            )
        # Temperature indicator dot on stem front
        handle.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, HANDLE_STEM_R - 0.0005, 0.022)),
            material=dot_material,
            name="indicator_dot",
        )
        return handle

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    # Hot on the left (-X), cold on the right (+X); +Y is toward the user.
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
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_handle = object_model.get_part("hot_handle")
    cold_handle = object_model.get_part("cold_handle")
    diverter = object_model.get_part("diverter_knob")

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_turn = object_model.get_articulation("hot_handle_turn")
    cold_turn = object_model.get_articulation("cold_handle_turn")
    diverter_slide = object_model.get_articulation("diverter_slide")

    spout_tube = gooseneck.get_visual("spout_tube")
    base_column = spout_base.get_visual("base_column")
    aerator = gooseneck.get_visual("aerator")

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

    # Diverter stem embeds behind the spout column
    ctx.allow_overlap(
        diverter,
        spout_base,
        elem_a=diverter.get_visual("diverter_stem"),
        elem_b=base_column,
        reason="diverter stem protrudes into the rear of the spout column bore",
    )

    # Underside nuts embed into the deck slab (mounting hardware through deck holes)
    for piece_name in ("spout_base", "hot_valve_column", "cold_valve_column"):
        piece = object_model.get_part(piece_name)
        ctx.allow_overlap(
            piece,
            deck,
            elem_a=piece.get_visual("underside_nut"),
            elem_b=deck.get_visual("deck_slab"),
            reason="mounting nut sits in the deck bore below the base flange",
        )

    # Diverter knob is mounted through the column back via prismatic joint;
    # the FCL mesh approximation creates a sub-0.1 mm gap despite exact overlap.
    ctx.allow_isolated_part(
        diverter,
        reason="diverter knob is supported by the prismatic joint through the spout column bore; exact geometry confirms overlap",
    )

    # --- joint plan: types, axes, ranges -----------------------------------
    # Revolute joints
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

    # Diverter: prismatic, vertical Z, 0 to DIVERTER_TRAVEL
    ctx.check(
        "diverter_slide_is_prismatic",
        str(diverter_slide.joint_type).lower().endswith("prismatic"),
        f"type={diverter_slide.joint_type}",
    )
    ctx.check(
        "diverter_slide_axis_z",
        tuple(diverter_slide.axis) == (0.0, 0.0, 1.0),
        f"axis={diverter_slide.axis}",
    )
    div_ml = diverter_slide.motion_limits
    ctx.check(
        "diverter_slide_range",
        div_ml is not None
        and abs(div_ml.lower) < 1e-6
        and abs(div_ml.upper - DIVERTER_TRAVEL) < 1e-6,
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

    # --- cross handle geometry: two perpendicular bars ---------------------
    for handle, sign in ((hot_handle, -1.0), (cold_handle, 1.0)):
        bar_x = handle.get_visual("cross_bar_x")
        bar_y = handle.get_visual("cross_bar_y")
        aabb_x = ctx.part_element_world_aabb(handle, elem=bar_x)
        aabb_y = ctx.part_element_world_aabb(handle, elem=bar_y)
        span_x_x = aabb_x[1][0] - aabb_x[0][0]  # bar_x spans along X
        span_y_y = aabb_y[1][1] - aabb_y[0][1]  # bar_y spans along Y
        ctx.check(
            f"{handle.name}_cross_shape",
            span_x_x > 0.06 and span_y_y > 0.06,
            f"bar_x span_x={span_x_x:.3f} bar_y span_y={span_y_y:.3f}",
        )

    # --- stem collars present under each handle ----------------------------
    for handle in (hot_handle, cold_handle):
        collar = handle.get_visual("stem_collar")
        collar_aabb = ctx.part_element_world_aabb(handle, elem=collar)
        collar_r = (collar_aabb[1][0] - collar_aabb[0][0]) / 2
        ctx.check(
            f"{handle.name}_stem_collar_visible",
            collar_r > STEM_COLLAR_R - 0.002,
            f"collar radius={collar_r:.4f}",
        )

    # --- underside nuts below each base -----------------------------------
    for piece_name in ("spout_base", "hot_valve_column", "cold_valve_column"):
        piece = object_model.get_part(piece_name)
        nut = piece.get_visual("underside_nut")
        nut_aabb = ctx.part_element_world_aabb(piece, elem=nut)
        # Nut top should be at or below the flange top (below deck surface)
        ctx.check(
            f"{piece_name}_nut_below_deck",
            nut_aabb[1][2] < DECK_T + 1e-6,
            f"nut top z={nut_aabb[1][2]:.4f}",
        )
        # Proof: nut is within the flange XY footprint
        flange_name = "base_flange" if piece_name == "spout_base" else "valve_flange"
        flange = piece.get_visual(flange_name)
        ctx.expect_within(
            piece,
            piece,
            axes="xy",
            inner_elem=nut,
            outer_elem=flange,
            margin=0.002,
            name=f"{piece_name}_nut_within_flange_xy",
        )

    # --- diverter knob behind the spout column ----------------------------
    div_cap = diverter.get_visual("diverter_cap")
    div_aabb = ctx.part_element_world_aabb(diverter, elem=div_cap)
    div_center_y = 0.5 * (div_aabb[0][1] + div_aabb[1][1])
    ctx.check(
        "diverter_behind_spout",
        div_center_y < -SPOUT_COL_R,
        f"diverter center y={div_center_y:.4f} column back y={-SPOUT_COL_R:.4f}",
    )

    # Indicator dots: red on hot, blue on cold
    for handle, mat in ((hot_handle, "hot_red"), (cold_handle, "cold_blue")):
        dot = handle.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{handle.name}_dot_material", mat_name == mat, f"material={mat_name}")

    # --- articulation behavior ---------------------------------------------
    # Cross handle rotation: at q=0 bars span X and Y; at q=+90 deg they swap.
    with ctx.pose({hot_turn: 0.0}):
        bar0 = ctx.part_element_world_aabb(hot_handle, elem=hot_handle.get_visual("cross_bar_x"))
    with ctx.pose({hot_turn: math.pi / 2}):
        bar90 = ctx.part_element_world_aabb(hot_handle, elem=hot_handle.get_visual("cross_bar_x"))
    span_x0 = bar0[1][0] - bar0[0][0]
    span_y0 = bar0[1][1] - bar0[0][1]
    span_x90 = bar90[1][0] - bar90[0][0]
    span_y90 = bar90[1][1] - bar90[0][1]
    ctx.check(
        "hot_handle_rotates_about_vertical_axis",
        span_x0 > 0.06 and span_y0 < 0.03 and span_y90 > 0.06 and span_x90 < 0.03,
        f"closed span=({span_x0:.3f},{span_y0:.3f}) turned span=({span_x90:.3f},{span_y90:.3f})",
    )

    # Spout swivel: +45 deg swings the forward outlet toward -X
    with ctx.pose({spout_swivel: math.pi / 4}):
        tip45 = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    tip45_cx = 0.5 * (tip45[0][0] + tip45[1][0])
    tip45_cz = 0.5 * (tip45[0][2] + tip45[1][2])
    ctx.check(
        "spout_swivels_about_column_axis",
        tip45_cx < -0.06 and abs(tip45_cz - (outlet_above_deck + DECK_T)) < 1e-3,
        f"tip at 45deg x={tip45_cx:.3f} z={tip45_cz:.3f}",
    )

    # Diverter prismatic motion: knob moves upward at max travel
    with ctx.pose({diverter_slide: 0.0}):
        div_rest = ctx.part_element_world_aabb(diverter, elem=div_cap)
    with ctx.pose({diverter_slide: DIVERTER_TRAVEL}):
        div_up = ctx.part_element_world_aabb(diverter, elem=div_cap)
    rest_z = 0.5 * (div_rest[0][2] + div_rest[1][2])
    up_z = 0.5 * (div_up[0][2] + div_up[1][2])
    ctx.check(
        "diverter_slides_upward",
        up_z > rest_z + 0.01,
        f"rest_z={rest_z:.4f} up_z={up_z:.4f}",
    )
    # Proof: diverter stem overlaps with column on the Y axis (embedded mount)
    ctx.expect_overlap(
        diverter,
        spout_base,
        axes="y",
        elem_a=diverter.get_visual("diverter_stem"),
        elem_b=base_column,
        min_overlap=0.003,
        name="diverter_stem_embedded_in_column",
    )

    return ctx.report()


object_model = build_object_model()