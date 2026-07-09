from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet set (variant 24).

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (CONTINUOUS about the column's vertical axis, unlimited rotation),
- hot (left) and cold (right): valve columns topped by cylindrical lever
  handles on tapered pedestals (each revolute about its column's vertical
  axis, -90..+90 deg).

Narrow installation seams visible at all three deck bases.
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

# Cylindrical lever handle with tapered pedestal
PEDESTAL_R_BOTTOM = 0.022  # matches valve column radius at base
PEDESTAL_R_TOP = 0.014  # tapers narrower at top
PEDESTAL_H = 0.030  # pedestal height
HANDLE_R = 0.013  # cylindrical grip radius
HANDLE_H = 0.055  # grip height
STEM_EMBED = 0.012  # hidden engagement into column
DOT_R = 0.0035

# Seam rings (narrow annular seams at deck bases)
SEAM_WIDTH = 0.003  # seam ring height
SEAM_PROUD = 0.002  # how far the seam extends beyond flange radius

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


def _tapered_pedestal() -> cq.Workplane:
    """Truncated cone pedestal: wide at bottom, narrow at top."""
    return (
        cq.Workplane("XY")
        .circle(PEDESTAL_R_BOTTOM)
        .workplane(offset=PEDESTAL_H)
        .circle(PEDESTAL_R_TOP)
        .loft()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_black_bathroom_faucet")

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
    # Narrow seam ring at spout deck base
    spout_base.visual(
        Cylinder(radius=SPOUT_FLANGE_R + SEAM_PROUD, length=SEAM_WIDTH),
        origin=Origin(xyz=(0.0, 0.0, SEAM_WIDTH / 2)),
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

    # Central spout swivels on a CONTINUOUS vertical joint (unlimited rotation)
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
        # Narrow seam ring at valve deck base
        col.visual(
            Cylinder(radius=VALVE_FLANGE_R + SEAM_PROUD, length=SEAM_WIDTH),
            origin=Origin(xyz=(0.0, 0.0, SEAM_WIDTH / 2)),
            material=seam_dark,
            name="deck_seam",
        )
        return col

    def _cylindrical_lever(name: str, dot_material: object) -> object:
        """Cylindrical lever handle on a tapered pedestal."""
        lever = model.part(name)
        # Tapered pedestal (truncated cone) via CadQuery
        lever.visual(
            mesh_from_cadquery(_tapered_pedestal(), f"{name}_pedestal"),
            origin=Origin(xyz=(0.0, 0.0, -STEM_EMBED)),
            material=matte_black,
            name="tapered_pedestal",
        )
        # Cylindrical grip handle on top of pedestal
        lever.visual(
            Cylinder(radius=HANDLE_R, length=HANDLE_H),
            origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H - STEM_EMBED + HANDLE_H / 2)),
            material=matte_black,
            name="lever_grip",
        )
        # Rounded cap on top of grip
        lever.visual(
            Sphere(radius=HANDLE_R),
            origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H - STEM_EMBED + HANDLE_H)),
            material=matte_black,
            name="grip_cap",
        )
        # Tiny temperature indicator dot inset into the pedestal front.
        # At z=0.010 in lever frame, the pedestal local z = 0.010 + STEM_EMBED = 0.022;
        # fraction = 0.022/0.030 ≈ 0.733; radius ≈ 0.022 - 0.733*0.008 ≈ 0.0161.
        # Embed the dot halfway into the surface for connectivity.
        lever.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, 0.014, 0.010)),
            material=dot_material,
            name="indicator_dot",
        )
        return lever

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    # Hot on the left (-X), cold on the right (+X); +Y is toward the user.
    hot_lever = _cylindrical_lever("hot_lever", hot_red)
    cold_lever = _cylindrical_lever("cold_lever", cold_blue)

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
            elem_a=lever.get_visual("tapered_pedestal"),
            elem_b=col.get_visual("valve_body"),
            reason="lever pedestal seats 12 mm into the valve cartridge bore",
        )

    # --- joint plan: spout is CONTINUOUS, levers are REVOLUTE ---------------
    ctx.check(
        "spout_swivel_is_continuous",
        str(spout_swivel.joint_type).lower().endswith("continuous"),
        f"type={spout_swivel.joint_type}",
    )
    ctx.check(
        "spout_swivel_axis_is_vertical",
        tuple(spout_swivel.axis) == (0.0, 0.0, 1.0),
        f"axis={spout_swivel.axis}",
    )
    # Continuous joints should not have position bounds
    spout_ml = spout_swivel.motion_limits
    ctx.check(
        "spout_swivel_no_position_limits",
        spout_ml is None or (spout_ml.lower is None and spout_ml.upper is None),
        f"limits={spout_ml}",
    )

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

    # --- deck seam rings present at all three bases -------------------------
    for piece, seam_name in (
        (spout_base, "spout_base"),
        (hot_col, "hot_valve"),
        (cold_col, "cold_valve"),
    ):
        seam = piece.get_visual("deck_seam")
        seam_aabb = ctx.part_element_world_aabb(piece, elem=seam)
        seam_height = seam_aabb[1][2] - seam_aabb[0][2]
        ctx.check(
            f"{seam_name}_has_deck_seam",
            seam is not None and 0.001 < seam_height < 0.006,
            f"seam height={seam_height:.4f}",
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

    # --- lever form: cylindrical grips on tapered pedestals -----------------
    for lever in (hot_lever, cold_lever):
        grip = lever.get_visual("lever_grip")
        pedestal = lever.get_visual("tapered_pedestal")
        grip_aabb = ctx.part_element_world_aabb(lever, elem=grip)
        pedestal_aabb = ctx.part_element_world_aabb(lever, elem=pedestal)
        # Grip sits above pedestal
        ctx.check(
            f"{lever.name}_grip_above_pedestal",
            grip_aabb[0][2] >= pedestal_aabb[1][2] - 0.005,
            f"grip_bottom={grip_aabb[0][2]:.4f} pedestal_top={pedestal_aabb[1][2]:.4f}",
        )
        # Grip is cylindrical (taller than wide)
        grip_dx = grip_aabb[1][0] - grip_aabb[0][0]
        grip_dz = grip_aabb[1][2] - grip_aabb[0][2]
        ctx.check(
            f"{lever.name}_grip_is_cylindrical",
            grip_dz > grip_dx * 1.5,
            f"grip dx={grip_dx:.4f} dz={grip_dz:.4f}",
        )

    # Indicator dots: red on hot, blue on cold.
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}")

    # --- articulation behavior ---------------------------------------------
    # Spout continuous swivel: rotation at +90 deg swings outlet toward -X.
    with ctx.pose({spout_swivel: math.pi / 2}):
        tip90 = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    tip90_cx = 0.5 * (tip90[0][0] + tip90[1][0])
    tip90_cz = 0.5 * (tip90[0][2] + tip90[1][2])
    ctx.check(
        "spout_swivels_continuously_about_column_axis",
        tip90_cx < -0.06 and abs(tip90_cz - (outlet_above_deck + DECK_T)) < 1e-3,
        f"tip at 90deg x={tip90_cx:.3f} z={tip90_cz:.3f}",
    )

    # Lever rotation: at q=0 grip is centered; at q=+90 deg the indicator dot moves.
    with ctx.pose({hot_turn: 0.0}):
        dot0 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("indicator_dot"))
    with ctx.pose({hot_turn: math.pi / 2}):
        dot90 = ctx.part_element_world_aabb(hot_lever, elem=hot_lever.get_visual("indicator_dot"))
    dot0_y = 0.5 * (dot0[0][1] + dot0[1][1])
    dot90_y = 0.5 * (dot90[0][1] + dot90[1][1])
    ctx.check(
        "hot_lever_rotates_about_vertical_axis",
        abs(dot90_y - dot0_y) > 0.01,
        f"dot y: rest={dot0_y:.4f} turned={dot90_y:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
