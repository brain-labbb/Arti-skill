from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet with waterfall spout.

Three-piece widespread layout on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a short rectangular waterfall channel
  that swivels about the column's vertical axis (-45..+45 deg). The outlet
  aerator pivots downward on a small hinge at the channel lip.
- hot (left) and cold (right): valve columns with decorative ring ridges,
  topped by T-style lever handles (each revolute -90..+90 deg).

Narrow seam rings at all three deck bases. All surfaces matte black; tiny
red/blue indicator dots on handle stems. Modeled at true scale in meters;
deck bottom on z=0.
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
SPOUT_COL_H = 0.12  # column top = joint height above deck surface

# Waterfall channel (in spout part frame, origin at column top)
CHAN_W = 0.048  # channel width (X)
CHAN_L = 0.110  # channel length (Y, forward)
CHAN_T = 0.018  # channel body thickness (Z)
CHAN_RISE = 0.09  # how much the channel rises from column top
CHAN_TROUGH_W = 0.036  # trough width
CHAN_TROUGH_D = 0.008  # trough depth cut into top
CHAN_EMBED = 0.020  # hidden engagement shank into column
COLLAR_R = 0.022
COLLAR_H = 0.014

# Aerator tab (hinged at channel outlet)
AER_W = 0.044
AER_L = 0.018
AER_T = 0.006
PIVOT_PIN_R = 0.004
PIVOT_PIN_LEN = CHAN_W - 0.008  # spans channel width

# Valve pieces
VALVE_FLANGE_R = 0.036
VALVE_FLANGE_H = 0.010
VALVE_COL_R = 0.0225
VALVE_COL_H = 0.10

# Decorative ring ridges on valve pedestals
RING_R_OUTER = VALVE_COL_R + 0.004  # slightly proud of column surface
RING_R_INNER = VALVE_COL_R - 0.001
RING_H = 0.004
RING_POSITIONS = (0.030, 0.055, 0.078)  # Z heights on valve column

# Seam rings at deck bases
SEAM_R_SPOUT = SPOUT_FLANGE_R + 0.002
SEAM_R_VALVE = VALVE_FLANGE_R + 0.002
SEAM_H = 0.002

# T-lever (in the lever part frame, origin at valve column top)
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.045
BAR_R = 0.0095
BAR_LEN = 0.12
BAR_CENTER_OFF = 0.025
DOT_R = 0.0035


def _waterfall_channel_solid() -> cq.Workplane:
    """Rectangular waterfall channel with trough and embed shank."""
    # Main body: rises from column top, extends forward (+Y), slight upward angle
    # The shank goes down into the column for mounting
    body = (
        cq.Workplane("XY")
        .box(CHAN_W, CHAN_L, CHAN_T, centered=(True, False, False))
    )
    # Cut the trough channel on top (centered in X, running full length in Y)
    trough = (
        cq.Workplane("XY")
        .workplane(offset=CHAN_T - CHAN_TROUGH_D)
        .box(CHAN_TROUGH_W, CHAN_L - 0.006, CHAN_TROUGH_D + 0.001, centered=(True, False, False))
    )
    body = body.cut(trough)
    # Add embed shank below (cylindrical, fits into column bore)
    shank = (
        cq.Workplane("XY")
        .workplane(offset=-CHAN_EMBED)
        .circle(COLLAR_R - 0.003)
        .extrude(CHAN_EMBED)
    )
    body = body.union(shank)
    return body


def _decorative_ring() -> cq.Workplane:
    """Thin ring ridge for valve pedestal decoration."""
    outer = (
        cq.Workplane("XY")
        .circle(RING_R_OUTER)
        .circle(RING_R_INNER)
        .extrude(RING_H)
    )
    return outer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_waterfall_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.03, 0.03, 0.03, 1.0))
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
    # Seam ring at spout deck base
    spout_base.visual(
        Cylinder(radius=SEAM_R_SPOUT, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2)),
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

    # -------------------------------------------------- waterfall spout channel
    waterfall_spout = model.part("waterfall_spout")
    waterfall_spout.visual(
        mesh_from_cadquery(_waterfall_channel_solid(), "waterfall_channel"),
        material=matte_black,
        name="channel_body",
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

    # ------------------------------------------------------- hinged aerator
    aerator = model.part("aerator")
    # Pivot pin sits at the hinge line and bridges the channel width for contact
    aerator.visual(
        Cylinder(radius=PIVOT_PIN_R, length=PIVOT_PIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=matte_black,
        name="pivot_pin",
    )
    # Tab extends forward and slightly below the hinge line
    aerator.visual(
        Box((AER_W, AER_L, AER_T)),
        origin=Origin(xyz=(0.0, AER_L / 2, -AER_T / 2)),
        material=matte_black,
        name="aerator_tab",
    )

    # Aerator hinges at the forward lip of the channel, pivoting downward
    # The hinge line is at the channel outlet end (y = CHAN_L), at channel top
    # In the waterfall_spout frame, channel top is at z = CHAN_T
    aerator_hinge_z = CHAN_T
    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=waterfall_spout,
        child=aerator,
        origin=Origin(xyz=(0.0, CHAN_L, aerator_hinge_z)),
        axis=(-1.0, 0.0, 0.0),  # pivot about -X axis; positive q tilts the +Y end toward -Z (downward)
        motion_limits=MotionLimits(
            effort=3.0, velocity=1.5, lower=0.0, upper=math.pi / 4
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
        # Seam ring at valve deck base
        col.visual(
            Cylinder(radius=SEAM_R_VALVE, length=SEAM_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2)),
            material=seam_dark,
            name="deck_seam",
        )
        # Decorative ring ridges on the pedestal
        for i, z_pos in enumerate(RING_POSITIONS):
            col.visual(
                mesh_from_cadquery(_decorative_ring(), f"ring_{name}_{i}"),
                origin=Origin(xyz=(0.0, 0.0, z_pos)),
                material=matte_black,
                name=f"deco_ring_{i}",
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
    aerator = object_model.get_part("aerator")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_lever = object_model.get_part("hot_lever")
    cold_lever = object_model.get_part("cold_lever")

    spout_swivel = object_model.get_articulation("spout_swivel")
    aerator_hinge = object_model.get_articulation("aerator_hinge")
    hot_turn = object_model.get_articulation("hot_lever_turn")
    cold_turn = object_model.get_articulation("cold_lever_turn")

    channel_body = waterfall.get_visual("channel_body")
    base_column = spout_base.get_visual("base_column")
    aerator_tab = aerator.get_visual("aerator_tab")
    pivot_pin = aerator.get_visual("pivot_pin")

    # Intentional hidden engagements: channel shank seats inside column bore,
    # lever stems seat inside valve columns, aerator pivot pin sits in channel end.
    ctx.allow_overlap(
        waterfall,
        spout_base,
        elem_a=channel_body,
        elem_b=base_column,
        reason="waterfall channel shank embeds 20 mm into the spout column bore",
    )
    ctx.allow_overlap(
        aerator,
        waterfall,
        elem_a=pivot_pin,
        elem_b=channel_body,
        reason="aerator pivot pin sits at the channel outlet lip as the hinge bearing",
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

    # Aerator hinge: horizontal revolute about -X
    ctx.check(
        "aerator_hinge_is_revolute",
        str(aerator_hinge.joint_type).lower().endswith("revolute")
        and tuple(aerator_hinge.axis) == (-1.0, 0.0, 0.0),
        f"axis={aerator_hinge.axis}",
    )
    ml_a = aerator_hinge.motion_limits
    ctx.check(
        "aerator_hinge_range",
        ml_a is not None
        and abs(ml_a.lower) < 1e-6
        and abs(ml_a.upper - math.pi / 4) < 1e-6,
        f"lower={ml_a.lower} upper={ml_a.upper}",
    )

    # Lever turns: vertical revolute
    for joint, lim in ((hot_turn, math.pi / 2), (cold_turn, math.pi / 2)):
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

    # --- waterfall channel: rectangular, extends forward, rises above column
    chan_aabb = ctx.part_element_world_aabb(waterfall, elem=channel_body)
    chan_dx = chan_aabb[1][0] - chan_aabb[0][0]
    chan_dy = chan_aabb[1][1] - chan_aabb[0][1]
    chan_dz = chan_aabb[1][2] - chan_aabb[0][2]
    ctx.check(
        "waterfall_channel_rectangular",
        chan_dx > 0.035 and chan_dy > 0.08 and chan_dz < 0.04,
        f"channel dims=({chan_dx:.4f}, {chan_dy:.4f}, {chan_dz:.4f})",
    )
    # Channel extends forward (positive Y) from the spout center
    chan_cy = 0.5 * (chan_aabb[0][1] + chan_aabb[1][1])
    ctx.check(
        "waterfall_channel_extends_forward",
        chan_cy > 0.03,
        f"channel center y={chan_cy:.4f}",
    )
    # Channel top is above column top (risen)
    chan_top_z = chan_aabb[1][2]
    ctx.check(
        "waterfall_channel_rises_above_column",
        chan_top_z > DECK_T + SPOUT_COL_H + 0.01,
        f"channel top z={chan_top_z:.4f}",
    )

    # --- aerator hinge: pivots downward at the outlet ----------------------
    # Prove the pivot pin contacts the channel body (supporting the aerator)
    ctx.expect_contact(aerator, waterfall, elem_a=pivot_pin, elem_b=channel_body, contact_tol=0.005)

    with ctx.pose({aerator_hinge: 0.0}):
        aer_rest_z = ctx.part_element_world_aabb(aerator, elem=aerator_tab)
    with ctx.pose({aerator_hinge: math.pi / 4}):
        aer_tilt_z = ctx.part_element_world_aabb(aerator, elem=aerator_tab)
    rest_bottom = aer_rest_z[0][2]
    tilt_bottom = aer_tilt_z[0][2]
    ctx.check(
        "aerator_pivots_downward",
        tilt_bottom < rest_bottom - 0.001,
        f"rest bottom z={rest_bottom:.4f} tilt bottom z={tilt_bottom:.4f}",
    )

    # --- seam rings at all three deck bases --------------------------------
    spout_seam = spout_base.get_visual("deck_seam")
    hot_seam = hot_col.get_visual("deck_seam")
    cold_seam = cold_col.get_visual("deck_seam")
    for seam, label in ((spout_seam, "spout"), (hot_seam, "hot"), (cold_seam, "cold")):
        ctx.check(
            f"{label}_deck_seam_exists",
            seam is not None,
            f"seam visual missing on {label} base",
        )
    # Seam rings are thin (small Z extent)
    seam_aabb = ctx.part_element_world_aabb(spout_base, elem=spout_seam)
    seam_dz = seam_aabb[1][2] - seam_aabb[0][2]
    ctx.check(
        "spout_seam_is_thin",
        seam_dz < 0.005,
        f"seam thickness={seam_dz:.4f}",
    )

    # --- decorative ring ridges on valve pedestals ------------------------
    for col, label in ((hot_col, "hot"), (cold_col, "cold")):
        rings = [col.get_visual(f"deco_ring_{i}") for i in range(len(RING_POSITIONS))]
        ctx.check(
            f"{label}_valve_has_deco_rings",
            all(r is not None for r in rings),
            f"rings={[r is not None for r in rings]}",
        )
    # Rings should be slightly proud of the column surface
    ring0_aabb = ctx.part_element_world_aabb(hot_col, elem=hot_col.get_visual("deco_ring_0"))
    ring_dx = ring0_aabb[1][0] - ring0_aabb[0][0]
    ctx.check(
        "deco_ring_proud_of_column",
        ring_dx > 2 * VALVE_COL_R + 0.002,
        f"ring diameter={ring_dx:.4f} vs column diameter={2*VALVE_COL_R:.4f}",
    )

    # --- articulation behavior: lever turn ---------------------------------
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

    # Spout swivel proof
    with ctx.pose({spout_swivel: math.pi / 4}):
        chan45 = ctx.part_element_world_aabb(waterfall, elem=channel_body)
    chan45_cx = 0.5 * (chan45[0][0] + chan45[1][0])
    ctx.check(
        "spout_swivels_about_column_axis",
        chan45_cx < -0.03,
        f"channel center at 45deg x={chan45_cx:.4f}",
    )

    # Indicator dots
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}")

    return ctx.report()


object_model = build_object_model()
