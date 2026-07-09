from __future__ import annotations

"""Variant 14: Matte-black widespread two-handle bathroom faucet.

Three-piece widespread layout (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (revolute about the column's vertical axis, -45..+45 deg) and a
  hollow outlet aerator that pivots downward on a small hinge
  (revolute about a horizontal axis, 0..+0.45 rad);
- hot (left) and cold (right): valve columns with tapered pedestals
  topped by cylindrical lever handles (each revolute about its column's
  vertical axis, -90..+90 deg).

Narrow seams at all three deck bases. All surfaces matte black; tiny
red/blue indicator dots on the handle pedestals. Modeled at true scale
in meters; deck bottom on z=0.
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

# Tapered pedestal lever (in the lever part frame, origin at valve column top)
PED_BOTTOM_R = 0.020  # pedestal base radius (slightly less than column)
PED_TOP_R = 0.012  # pedestal top radius (tapered)
PED_H = 0.025  # pedestal height
GRIP_R = 0.008  # cylindrical grip radius
GRIP_LEN = 0.065  # grip length
BOSS_R = 0.010  # mounting boss radius (engages into column)
BOSS_LEN = 0.012  # mounting boss length
DOT_R = 0.0035  # indicator dot radius

# Aerator
ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.016
AERATOR_R = 0.017
AERATOR_INNER_R = AERATOR_R * 0.62  # hollow bore
HINGE_BARREL_R = 0.004
HINGE_BARREL_LEN = 0.018

# Seam rings
SEAM_WIDTH = 0.003
SEAM_H = 0.001

# Unit tangent of the arc at the hook end (pointing out of the spout, downward).
_TX = math.sin(math.radians(HOOK_DEG))  # y component
_TZ = -math.cos(math.radians(HOOK_DEG))  # z component


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


def _tapered_pedestal_solid() -> cq.Workplane:
    """Truncated cone pedestal: wider base to narrower top."""
    return (
        cq.Workplane("XY")
        .circle(PED_BOTTOM_R)
        .workplane(offset=PED_H)
        .circle(PED_TOP_R)
        .loft()
    )


def _hollow_aerator_solid() -> cq.Workplane:
    """Hollow cylindrical aerator shell (annular cross-section)."""
    return (
        cq.Workplane("XY")
        .circle(AERATOR_R)
        .circle(AERATOR_INNER_R)
        .extrude(AERATOR_LEN)
    )


def _seam_ring_solid(flange_r: float) -> cq.Workplane:
    """Thin annular seam ring at a deck base flange."""
    return (
        cq.Workplane("XY")
        .circle(flange_r + SEAM_WIDTH)
        .circle(flange_r - 0.001)
        .extrude(SEAM_H)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet_v14")

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
    # Narrow seam ring at the deck base
    spout_base.visual(
        mesh_from_cadquery(_seam_ring_solid(SPOUT_FLANGE_R), "spout_seam_ring"),
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
    # Hinge barrel at the tube end (visible hinge hardware)
    gooseneck_spout.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_LEN),
        origin=Origin(
            xyz=(0.0, ARC_END_Y, ARC_END_Z),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=matte_black,
        name="hinge_barrel",
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
    aerator.visual(
        mesh_from_cadquery(_hollow_aerator_solid(), "aerator_shell"),
        origin=Origin(
            xyz=(0.0, 0.0, -AERATOR_LEN),
            rpy=(math.radians(HOOK_DEG), 0.0, 0.0),
        ),
        material=matte_black,
        name="aerator_shell",
    )
    # Small ring at the outlet end for visual detail
    aerator.visual(
        Cylinder(radius=AERATOR_R + 0.002, length=0.003),
        origin=Origin(
            xyz=(0.0, 0.0, -AERATOR_LEN + 0.0015),
            rpy=(math.radians(HOOK_DEG), 0.0, 0.0),
        ),
        material=matte_black,
        name="aerator_tip_ring",
    )

    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=gooseneck_spout,
        child=aerator,
        origin=Origin(xyz=(0.0, ARC_END_Y, ARC_END_Z)),
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
        # Narrow seam ring at the deck base
        col.visual(
            mesh_from_cadquery(
                _seam_ring_solid(VALVE_FLANGE_R), f"{name}_seam_ring"
            ),
            material=seam_dark,
            name="deck_seam",
        )
        return col

    def _pedestal_lever(name: str, dot_material: object) -> object:
        lever = model.part(name)
        # Mounting boss: embeds into the valve column bore
        lever.visual(
            Cylinder(radius=BOSS_R, length=BOSS_LEN),
            origin=Origin(xyz=(0.0, 0.0, -BOSS_LEN / 2)),
            material=matte_black,
            name="lever_boss",
        )
        # Tapered pedestal (truncated cone)
        lever.visual(
            mesh_from_cadquery(_tapered_pedestal_solid(), f"{name}_pedestal"),
            material=matte_black,
            name="pedestal",
        )
        # Cylindrical grip bar extending toward the user (+Y)
        lever.visual(
            Cylinder(radius=GRIP_R, length=GRIP_LEN),
            origin=Origin(
                xyz=(0.0, GRIP_LEN / 2 + 0.003, PED_H),
                rpy=(-math.pi / 2, 0.0, 0.0),
            ),
            material=matte_black,
            name="lever_grip",
        )
        # End caps on the grip
        for end_y, cap_name in (
            (0.003, "grip_cap_inner"),
            (GRIP_LEN + 0.003, "grip_cap_outer"),
        ):
            lever.visual(
                Sphere(radius=GRIP_R),
                origin=Origin(xyz=(0.0, end_y, PED_H)),
                material=matte_black,
                name=cap_name,
            )
        # Tiny temperature indicator dot on the pedestal front
        lever.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, PED_BOTTOM_R - 0.0005, PED_H * 0.45)),
            material=dot_material,
            name="indicator_dot",
        )
        return lever

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    # Hot on the left (-X), cold on the right (+X); +Y is toward the user.
    hot_lever = _pedestal_lever("hot_lever", hot_red)
    cold_lever = _pedestal_lever("cold_lever", cold_blue)

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
    aerator_shell = aerator.get_visual("aerator_shell")

    # --- intentional hidden engagements ------------------------------------
    # Spout riser seats inside the base column bore.
    ctx.allow_overlap(
        gooseneck,
        spout_base,
        elem_a=spout_tube,
        elem_b=base_column,
        reason="gooseneck riser tube seats 30 mm into the base column bore",
    )
    # Lever bosses seat inside the valve column bores.
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.allow_overlap(
            lever,
            col,
            elem_a=lever.get_visual("lever_boss"),
            elem_b=col.get_visual("valve_body"),
            reason="lever mounting boss seats 12 mm into the valve cartridge bore",
        )
    # Aerator shell seats into the gooseneck tube end at the pivot hinge.
    ctx.allow_overlap(
        aerator,
        gooseneck,
        elem_a=aerator_shell,
        elem_b=spout_tube,
        reason="aerator shell seats into the tube end at the pivot hinge junction",
    )
    # Hinge barrel captures the aerator pivot bore.
    ctx.allow_overlap(
        aerator,
        gooseneck,
        elem_a=aerator_shell,
        elem_b=gooseneck.get_visual("hinge_barrel"),
        reason="aerator shell wraps around the hinge barrel at the pivot axis",
    )

    # --- joint plan: types, axes, ranges --------------------------------
    # Spout swivel: vertical revolute, -45..+45 deg
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

    # Handle turns: vertical revolute, -90..+90 deg
    for joint in (hot_turn, cold_turn):
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
            and abs(ml.lower + math.pi / 2) < 1e-6
            and abs(ml.upper - math.pi / 2) < 1e-6,
            f"lower={ml.lower} upper={ml.upper}",
        )

    # Aerator hinge: horizontal revolute about X, 0..0.45 rad
    ctx.check(
        "aerator_hinge_is_revolute",
        str(aerator_hinge.joint_type).lower().endswith("revolute")
        and tuple(aerator_hinge.axis) == (1.0, 0.0, 0.0),
        f"axis={aerator_hinge.axis}",
    )
    ml = aerator_hinge.motion_limits
    ctx.check(
        "aerator_hinge_range",
        ml is not None
        and abs(ml.lower) < 1e-6
        and abs(ml.upper - 0.45) < 1e-6,
        f"lower={ml.lower} upper={ml.upper}",
    )

    # --- placement: 0.30 m spread, all three pieces seated on the deck ---
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
    tip_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_shell)
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

    # --- variant 14: tapered pedestals on both levers --------------------
    for lever in (hot_lever, cold_lever):
        ped = lever.get_visual("pedestal")
        ped_aabb = ctx.part_element_world_aabb(lever, elem=ped)
        ped_dx = ped_aabb[1][0] - ped_aabb[0][0]
        ped_dz = ped_aabb[1][2] - ped_aabb[0][2]
        ctx.check(
            f"{lever.name}_has_tapered_pedestal",
            ped_dx > 0.020 and ped_dz > 0.015,
            f"pedestal width={ped_dx:.4f} height={ped_dz:.4f}",
        )

    # --- variant 14: cylindrical grip handles (not T-bars) ---------------
    for lever, sign in ((hot_lever, -1.0), (cold_lever, 1.0)):
        grip_aabb = ctx.part_element_world_aabb(
            lever, elem=lever.get_visual("lever_grip")
        )
        grip_span_y = grip_aabb[1][1] - grip_aabb[0][1]
        grip_span_x = grip_aabb[1][0] - grip_aabb[0][0]
        # Grip extends along Y (toward user), not along X (not a T-bar)
        ctx.check(
            f"{lever.name}_grip_extends_toward_user",
            grip_span_y > 0.04 and grip_span_x < 0.025,
            f"grip span y={grip_span_y:.3f} x={grip_span_x:.3f}",
        )
        # Grip clears the valve column top
        ctx.expect_gap(
            lever,
            (hot_col if sign < 0 else cold_col),
            axis="z",
            positive_elem=lever.get_visual("lever_grip"),
            min_gap=0.01,
        )

    # Indicator dots: red on hot, blue on cold
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}")

    # --- variant 14: narrow seams at all three deck bases ----------------
    for base_part, part_name in (
        (spout_base, "spout_base"),
        (hot_col, "hot_valve_column"),
        (cold_col, "cold_valve_column"),
    ):
        seam = base_part.get_visual("deck_seam")
        seam_aabb = ctx.part_element_world_aabb(base_part, elem=seam)
        seam_dz = seam_aabb[1][2] - seam_aabb[0][2]
        ctx.check(
            f"{part_name}_has_deck_seam",
            seam_dz < 0.003,
            f"seam thickness={seam_dz:.4f}",
        )

    # Proof: aerator remains seated at the tube end (Z overlap at junction).
    ctx.expect_overlap(
        aerator,
        gooseneck,
        axes="z",
        elem_a=aerator_shell,
        elem_b=spout_tube,
        min_overlap=0.005,
        name="aerator seated at tube end on Z",
    )
    # Proof: aerator contacts the hinge barrel at the pivot.
    ctx.expect_contact(
        aerator,
        gooseneck,
        elem_a=aerator_shell,
        elem_b=gooseneck.get_visual("hinge_barrel"),
        contact_tol=0.005,
        name="aerator contacts hinge barrel at pivot",
    )

    # --- variant 14: hollow aerator outlet geometry ----------------------
    # The aerator shell should be wider than its inner bore (hollow tube).
    aer_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_shell)
    aer_span_x = aer_aabb[1][0] - aer_aabb[0][0]
    ctx.check(
        "aerator_is_hollow_shell",
        aer_span_x > 2 * AERATOR_INNER_R,
        f"aerator x-span={aer_span_x:.4f} > inner bore dia {2*AERATOR_INNER_R:.4f}",
    )

    # --- variant 14: aerator hinge pivots downward -----------------------
    with ctx.pose({aerator_hinge: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_shell)
        rest_cy = 0.5 * (rest_aabb[0][1] + rest_aabb[1][1])
    with ctx.pose({aerator_hinge: 0.45}):
        pivoted_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_shell)
        pivoted_cy = 0.5 * (pivoted_aabb[0][1] + pivoted_aabb[1][1])
    ctx.check(
        "aerator_pivots_forward_on_hinge",
        pivoted_cy > rest_cy + 0.001,
        f"rest y={rest_cy:.4f} pivoted y={pivoted_cy:.4f}",
    )

    # --- articulation behavior: lever rotation proof ---------------------
    with ctx.pose({hot_turn: 0.0}):
        bar0 = ctx.part_element_world_aabb(
            hot_lever, elem=hot_lever.get_visual("lever_grip")
        )
    with ctx.pose({hot_turn: math.pi / 2}):
        bar90 = ctx.part_element_world_aabb(
            hot_lever, elem=hot_lever.get_visual("lever_grip")
        )
    span_x0 = bar0[1][0] - bar0[0][0]
    span_y0 = bar0[1][1] - bar0[0][1]
    span_x90 = bar90[1][0] - bar90[0][0]
    span_y90 = bar90[1][1] - bar90[0][1]
    ctx.check(
        "hot_lever_rotates_about_vertical_axis",
        span_y0 > 0.04 and span_x0 < 0.025 and span_x90 > 0.04 and span_y90 < 0.025,
        f"closed span=({span_x0:.3f},{span_y0:.3f}) turned span=({span_x90:.3f},{span_y90:.3f})",
    )

    # Spout swivel: +45 deg swings the forward outlet toward -X
    with ctx.pose({spout_swivel: math.pi / 4}):
        tip45 = ctx.part_element_world_aabb(aerator, elem=aerator_shell)
    tip45_cx = 0.5 * (tip45[0][0] + tip45[1][0])
    tip45_cz = 0.5 * (tip45[0][2] + tip45[1][2])
    ctx.check(
        "spout_swivels_about_column_axis",
        tip45_cx < -0.06 and abs(tip45_cz - (outlet_above_deck + DECK_T)) < 0.01,
        f"tip at 45deg x={tip45_cx:.3f} z={tip45_cz:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
