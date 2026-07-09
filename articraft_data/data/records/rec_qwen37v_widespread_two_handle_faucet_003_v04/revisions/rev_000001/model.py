from __future__ import annotations

"""Matte-black widespread three-piece bathroom faucet set — variant 04.

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (revolute about the column's vertical axis, -45..+45 deg),
- hot (left) and cold (right): valve columns topped by cylindrical lever
  handles on tapered pedestals with decorative ring ridges
  (each revolute about its column's vertical axis, -90..+90 deg).

Narrow seam rings at all three deck bases. All faucet surfaces matte black;
tiny red/blue indicator dots on the handle pedestals.
Modeled at true scale in meters; deck bottom on z=0.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
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

# Tapered pedestal (in the lever part frame, origin at valve column top)
PEDESTAL_BOTTOM_R = 0.025  # slightly wider than valve column
PEDESTAL_TOP_R = 0.014
PEDESTAL_H = 0.042

# Decorative ring ridges on the pedestal
RING_COUNT = 3
RING_TUBE_R = 0.0018  # tube radius of each ridge torus

# Cylindrical lever handle on top of pedestal
HANDLE_R = 0.009
HANDLE_LEN = 0.085
HANDLE_OVERHANG = 0.022  # offset toward +Y (user side)

# Stem embedding pedestal into valve column
STEM_R = 0.009
STEM_EMBED = 0.015

# Indicator dot
DOT_R = 0.0035

# Seam rings at deck bases
SEAM_TUBE_R = 0.0012  # thin visible seam torus

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


def _tapered_pedestal_solid() -> cq.Workplane:
    """Truncated-cone pedestal: wider at base, narrower at top."""
    return (
        cq.Workplane("XY")
        .circle(PEDESTAL_BOTTOM_R)
        .workplane(offset=PEDESTAL_H)
        .circle(PEDESTAL_TOP_R)
        .loft()
    )


def _ring_ridge_mesh(z_frac: float) -> tuple:
    """Return (torus_mesh, z_position) for a decorative ring ridge at the
    given fractional height on the pedestal."""
    r = PEDESTAL_BOTTOM_R + (PEDESTAL_TOP_R - PEDESTAL_BOTTOM_R) * z_frac
    z = PEDESTAL_H * z_frac
    return mesh_from_geometry(TorusGeometry(r, RING_TUBE_R), "ring_ridge"), z


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_black_bathroom_faucet_v04")

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
    # Narrow seam ring at the flange top (visible base-deck construction line)
    spout_base.visual(
        mesh_from_geometry(
            TorusGeometry(SPOUT_FLANGE_R, SEAM_TUBE_R), "spout_seam"
        ),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_FLANGE_H)),
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

    # --------------------------------------------------- hot / cold valves
    def _valve_column(name: str, flange_r: float, flange_h: float) -> object:
        col = model.part(name)
        col.visual(
            Cylinder(radius=flange_r, length=flange_h),
            origin=Origin(xyz=(0.0, 0.0, flange_h / 2)),
            material=matte_black,
            name="valve_flange",
        )
        col.visual(
            Cylinder(radius=VALVE_COL_R, length=VALVE_COL_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H / 2)),
            material=matte_black,
            name="valve_body",
        )
        # Narrow seam ring at the flange top
        col.visual(
            mesh_from_geometry(
                TorusGeometry(flange_r, SEAM_TUBE_R), f"{name}_seam"
            ),
            origin=Origin(xyz=(0.0, 0.0, flange_h)),
            material=seam_dark,
            name="base_seam",
        )
        return col

    def _lever_handle(name: str, overhang_sign: float, dot_material: object) -> object:
        """Cylindrical lever handle on a tapered pedestal with ring ridges."""
        lever = model.part(name)

        # Stem: embeds into valve column bore
        lever.visual(
            Cylinder(radius=STEM_R, length=PEDESTAL_H + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (PEDESTAL_H - STEM_EMBED) / 2)),
            material=matte_black,
            name="lever_stem",
        )

        # Tapered pedestal (truncated cone)
        lever.visual(
            mesh_from_cadquery(_tapered_pedestal_solid(), "pedestal_body"),
            material=matte_black,
            name="pedestal",
        )

        # Decorative ring ridges on the pedestal
        for i in range(RING_COUNT):
            frac = (i + 1) / (RING_COUNT + 1)
            ring_mesh, ring_z = _ring_ridge_mesh(frac)
            lever.visual(
                ring_mesh,
                origin=Origin(xyz=(0.0, 0.0, ring_z)),
                material=matte_black,
                name=f"ring_ridge_{i}",
            )

        # Cylindrical lever handle: horizontal bar along Y, overhanging toward
        # user (+Y for hot side via overhang_sign).
        handle_z = PEDESTAL_H + HANDLE_R
        handle_y = HANDLE_OVERHANG * overhang_sign
        lever.visual(
            Cylinder(radius=HANDLE_R, length=HANDLE_LEN),
            origin=Origin(
                xyz=(0.0, handle_y, handle_z),
                rpy=(math.pi / 2, 0.0, 0.0),
            ),
            material=matte_black,
            name="lever_bar",
        )
        # End caps for the lever bar
        for end in (-1.0, 1.0):
            lever.visual(
                Sphere(radius=HANDLE_R),
                origin=Origin(
                    xyz=(0.0, handle_y + end * HANDLE_LEN / 2, handle_z)
                ),
                material=matte_black,
                name=f"bar_cap_{'front' if end * overhang_sign > 0 else 'rear'}",
            )

        # Indicator dot on the pedestal front (+Y face)
        dot_z = PEDESTAL_H * 0.5
        dot_y = PEDESTAL_BOTTOM_R + (PEDESTAL_TOP_R - PEDESTAL_BOTTOM_R) * 0.5 - 0.0005
        lever.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, dot_y, dot_z)),
            material=dot_material,
            name="indicator_dot",
        )
        return lever

    hot_valve_column = _valve_column("hot_valve_column", VALVE_FLANGE_R, VALVE_FLANGE_H)
    cold_valve_column = _valve_column("cold_valve_column", VALVE_FLANGE_R, VALVE_FLANGE_H)
    # Hot on the left (-X), cold on the right (+X); +Y is toward the user.
    hot_lever = _lever_handle("hot_lever", 1.0, hot_red)
    cold_lever = _lever_handle("cold_lever", 1.0, cold_blue)

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

    # --- gooseneck form: rises ~0.32 above deck, outlet ~0.25 above deck ---
    neck_aabb = ctx.part_world_aabb(gooseneck)
    arc_top_above_deck = neck_aabb[1][2] - DECK_T
    ctx.check(
        "gooseneck_arc_top_height",
        0.28 < arc_top_above_deck < 0.36,
        f"arc top {arc_top_above_deck:.3f} m above deck",
    )
    aerator = gooseneck.get_visual("aerator")
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

    # --- tapered pedestal: wider at bottom than top -----------------------
    for lever in (hot_lever, cold_lever):
        ped = lever.get_visual("pedestal")
        ped_aabb = ctx.part_element_world_aabb(lever, elem=ped)
        # Measure X extent at bottom vs top of pedestal to confirm taper.
        # The pedestal is a truncated cone: bottom radius > top radius.
        # We verify the overall bounding box is consistent with a tapered form
        # by checking that the pedestal height is approximately correct.
        ped_h = ped_aabb[1][2] - ped_aabb[0][2]
        ctx.check(
            f"{lever.name}_pedestal_height",
            abs(ped_h - PEDESTAL_H) < 0.003,
            f"pedestal height {ped_h:.4f} expected ~{PEDESTAL_H}",
        )
        # Pedestal X extent should be ~2*PEDESTAL_BOTTOM_R at widest (bottom)
        ped_dx = ped_aabb[1][0] - ped_aabb[0][0]
        ctx.check(
            f"{lever.name}_pedestal_tapered_wider_at_base",
            ped_dx > 2 * PEDESTAL_TOP_R + 0.005,
            f"pedestal dx={ped_dx:.4f} should exceed 2*top_r={2*PEDESTAL_TOP_R:.4f}",
        )

    # --- decorative ring ridges on pedestals --------------------------------
    for lever in (hot_lever, cold_lever):
        for i in range(RING_COUNT):
            ring_name = f"ring_ridge_{i}"
            ctx.check(
                f"{lever.name}_has_{ring_name}",
                lever.get_visual(ring_name) is not None,
                f"missing visual {ring_name}",
            )

    # --- narrow seams at all three deck bases ------------------------------
    for base_part, base_name in (
        (spout_base, "spout_base"),
        (hot_col, "hot_valve"),
        (cold_col, "cold_valve"),
    ):
        seam = base_part.get_visual("base_seam")
        ctx.check(
            f"{base_name}_has_seam_ring",
            seam is not None,
            f"missing base_seam on {base_name}",
        )

    # --- lever handle overhangs toward user (+Y) ---------------------------
    for lever in (hot_lever, cold_lever):
        bar_aabb = ctx.part_element_world_aabb(lever, elem=lever.get_visual("lever_bar"))
        bar_center_y = 0.5 * (bar_aabb[0][1] + bar_aabb[1][1])
        lever_pos = ctx.part_world_position(lever)
        ctx.check(
            f"{lever.name}_handle_overhangs_toward_user",
            bar_center_y > lever_pos[1] + 0.010,
            f"bar center y={bar_center_y:.3f} vs lever y={lever_pos[1]:.3f}",
        )

    # Indicator dots: red on hot, blue on cold.
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}")

    # --- articulation behavior ---------------------------------------------
    # Off-axis proof: at q=0 the lever bar spans Y (toward user);
    # at q=+90 deg it spans X (rotated about Z).
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
        span_y0 > 0.06 and span_x0 < 0.03 and span_x90 > 0.06 and span_y90 < 0.03,
        f"closed span=({span_x0:.3f},{span_y0:.3f}) turned span=({span_x90:.3f},{span_y90:.3f})",
    )

    # Spout swivel: +45 deg swings the forward outlet toward -X (right-hand
    # rule about +Z), keeping its height unchanged.
    with ctx.pose({spout_swivel: math.pi / 4}):
        tip45 = ctx.part_element_world_aabb(gooseneck, elem=aerator)
    tip45_cx = 0.5 * (tip45[0][0] + tip45[1][0])
    tip45_cz = 0.5 * (tip45[0][2] + tip45[1][2])
    ctx.check(
        "spout_swivels_about_column_axis",
        tip45_cx < -0.06 and abs(tip45_cz - (outlet_above_deck + DECK_T)) < 1e-3,
        f"tip at 45deg x={tip45_cx:.3f} z={tip45_cz:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
