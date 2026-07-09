from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet set with escutcheon.

Three-piece widespread layout on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (revolute about the column's vertical axis, -45..+45 deg),
- hot (left) and cold (right): valve columns topped by cross handles
  (each revolute about its column's vertical axis, -90..+90 deg).

Variant features:
- Raised oval escutcheon plate under all three posts.
- Cross handles (two perpendicular bars) rotate on short vertical axles.
- Visible stem collars under each handle.
- Separate hot (red) and cold (blue) cap disks as geometry.

All surfaces matte black. Modeled at true scale in meters; deck bottom on z=0.
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

# Escutcheon plate
ESC_LEN = 0.38  # total length along X (stadium long axis)
ESC_WID = 0.082  # width along Y (stadium short axis = end-cap diameter)
ESC_THICK = 0.007  # plate thickness
ESC_RAISE = 0.003  # small raised lip above deck surface (plate sits at deck top)

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
VALVE_COL_H = 0.10  # column top = handle joint height above deck surface

# Stem collar (visible ring on valve column below handle)
STEM_COLLAR_R = 0.026
STEM_COLLAR_H = 0.009

# Cross handle (in the handle part frame, origin at valve column top)
HANDLE_STEM_R = 0.009
HANDLE_STEM_EMBED = 0.015
HANDLE_STEM_TOP = 0.048  # top of stem above column top
CROSS_BAR_R = 0.008
CROSS_BAR_HALF = 0.038  # half-length of each cross arm
CROSS_HUB_R = 0.013  # central hub radius where bars meet
CROSS_HUB_H = 0.012  # hub height

# Cap disks (hot/cold indicators as geometry)
CAP_R = 0.011
CAP_H = 0.004

# Indicator dot (on stem front)
DOT_R = 0.0035

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


def _escutcheon_solid() -> cq.Workplane:
    """Raised oval escutcheon plate (stadium shape) spanning all three posts."""
    plate = (
        cq.Workplane("XY")
        .slot2D(ESC_LEN, ESC_WID)
        .extrude(ESC_THICK)
    )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    hot_red = model.material("hot_red", rgba=(0.78, 0.08, 0.08, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))
    chrome_ring = model.material("chrome_ring", rgba=(0.18, 0.18, 0.19, 1.0))

    # ------------------------------------------------------------- sink deck
    sink_deck = model.part("sink_deck")
    sink_deck.visual(
        Box((DECK_X, DECK_Y, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, DECK_T / 2)),
        material=deck_stone,
        name="deck_slab",
    )

    # ----------------------------------------------------- escutcheon plate
    escutcheon = model.part("escutcheon")
    escutcheon.visual(
        mesh_from_cadquery(_escutcheon_solid(), "escutcheon_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="escutcheon_plate",
    )
    model.articulation(
        "deck_to_escutcheon",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=escutcheon,
        origin=Origin(xyz=(0.0, 0.0, DECK_T)),
    )

    # The flange mounting surface is the top of the escutcheon plate.
    flange_z = DECK_T + ESC_THICK

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

    model.articulation(
        "deck_to_spout_base",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=spout_base,
        origin=Origin(xyz=(0.0, 0.0, flange_z)),
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
        # Visible stem collar: ring on top of valve column below the handle.
        col.visual(
            Cylinder(radius=STEM_COLLAR_R, length=STEM_COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H + STEM_COLLAR_H / 2)),
            material=chrome_ring,
            name="stem_collar",
        )
        return col

    def _cross_handle(name: str, cap_material: object) -> object:
        """Cross handle: two perpendicular bars on a vertical stem with cap disk."""
        handle = model.part(name)
        # Vertical stem (embeds into valve column below, extends above)
        handle.visual(
            Cylinder(radius=HANDLE_STEM_R, length=HANDLE_STEM_TOP + HANDLE_STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (HANDLE_STEM_TOP - HANDLE_STEM_EMBED) / 2)),
            material=matte_black,
            name="handle_stem",
        )
        # Central hub where the cross bars meet
        handle.visual(
            Cylinder(radius=CROSS_HUB_R, length=CROSS_HUB_H),
            origin=Origin(xyz=(0.0, 0.0, HANDLE_STEM_TOP - CROSS_HUB_H / 2)),
            material=matte_black,
            name="cross_hub",
        )
        # Cross bar along X
        handle.visual(
            Cylinder(radius=CROSS_BAR_R, length=CROSS_BAR_HALF * 2),
            origin=Origin(
                xyz=(0.0, 0.0, HANDLE_STEM_TOP),
                rpy=(0.0, math.pi / 2, 0.0),
            ),
            material=matte_black,
            name="cross_bar_x",
        )
        # Cross bar along Y
        handle.visual(
            Cylinder(radius=CROSS_BAR_R, length=CROSS_BAR_HALF * 2),
            origin=Origin(
                xyz=(0.0, 0.0, HANDLE_STEM_TOP),
                rpy=(math.pi / 2, 0.0, 0.0),
            ),
            material=matte_black,
            name="cross_bar_y",
        )
        # Rounded arm tips (4 ends)
        for axis_idx, sign in [(0, -1.0), (0, 1.0), (1, -1.0), (1, 1.0)]:
            x = sign * CROSS_BAR_HALF if axis_idx == 0 else 0.0
            y = sign * CROSS_BAR_HALF if axis_idx == 1 else 0.0
            handle.visual(
                Sphere(radius=CROSS_BAR_R),
                origin=Origin(xyz=(x, y, HANDLE_STEM_TOP)),
                material=matte_black,
                name=f"arm_tip_{'x' if axis_idx == 0 else 'y'}_{'pos' if sign > 0 else 'neg'}",
            )
        # Cap disk on top of the stem (hot=red, cold=blue)
        handle.visual(
            Cylinder(radius=CAP_R, length=CAP_H),
            origin=Origin(xyz=(0.0, 0.0, HANDLE_STEM_TOP + CROSS_HUB_H / 2 + CAP_H / 2)),
            material=cap_material,
            name="cap_disk",
        )
        return handle

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    hot_handle = _cross_handle("hot_handle", hot_red)
    cold_handle = _cross_handle("cold_handle", cold_blue)

    model.articulation(
        "deck_to_hot_valve",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=hot_valve_column,
        origin=Origin(xyz=(-SPREAD_HALF, 0.0, flange_z)),
    )
    model.articulation(
        "deck_to_cold_valve",
        ArticulationType.FIXED,
        parent=sink_deck,
        child=cold_valve_column,
        origin=Origin(xyz=(SPREAD_HALF, 0.0, flange_z)),
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
    escutcheon = object_model.get_part("escutcheon")
    spout_base = object_model.get_part("spout_base")
    gooseneck = object_model.get_part("gooseneck_spout")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_handle = object_model.get_part("hot_handle")
    cold_handle = object_model.get_part("cold_handle")

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_turn = object_model.get_articulation("hot_handle_turn")
    cold_turn = object_model.get_articulation("cold_handle_turn")

    spout_tube = gooseneck.get_visual("spout_tube")
    base_column = spout_base.get_visual("base_column")
    aerator = gooseneck.get_visual("aerator")

    # --- Escutcheon plate checks ---
    esc_aabb = ctx.part_world_aabb(escutcheon)
    hot_pos = ctx.part_world_position(hot_col)
    cold_pos = ctx.part_world_position(cold_col)
    spout_pos = ctx.part_world_position(spout_base)
    ctx.check(
        "escutcheon_spans_all_posts",
        esc_aabb[0][0] < hot_pos[0] - 0.01
        and esc_aabb[1][0] > cold_pos[0] + 0.01
        and abs(0.5 * (esc_aabb[0][0] + esc_aabb[1][0]) - spout_pos[0]) < 0.01,
        f"esc x=[{esc_aabb[0][0]:.3f},{esc_aabb[1][0]:.3f}] hot_x={hot_pos[0]:.3f} cold_x={cold_pos[0]:.3f}",
    )
    # Escutcheon sits on deck surface
    ctx.expect_contact(escutcheon, deck, contact_tol=1e-4)

    # --- Stem collar checks ---
    for col, col_name in ((hot_col, "hot"), (cold_col, "cold")):
        collar = col.get_visual("stem_collar")
        ctx.check(
            f"{col_name}_stem_collar_exists",
            collar is not None,
            f"stem_collar visual on {col_name} column",
        )
        # Collar sits above the valve body top
        collar_aabb = ctx.part_element_world_aabb(col, elem=collar)
        body_aabb = ctx.part_element_world_aabb(col, elem=col.get_visual("valve_body"))
        ctx.check(
            f"{col_name}_collar_above_valve_body",
            collar_aabb[0][2] > body_aabb[1][2] - 0.002,
            f"collar_bottom={collar_aabb[0][2]:.4f} body_top={body_aabb[1][2]:.4f}",
        )

    # --- Cap disk checks (hot=red, cold=blue as geometry) ---
    for handle, expected_mat, handle_name in (
        (hot_handle, "hot_red", "hot"),
        (cold_handle, "cold_blue", "cold"),
    ):
        cap = handle.get_visual("cap_disk")
        ctx.check(
            f"{handle_name}_cap_disk_exists",
            cap is not None,
            f"cap_disk visual on {handle_name} handle",
        )
        mat_name = cap.material if isinstance(cap.material, str) else cap.material.name
        ctx.check(
            f"{handle_name}_cap_disk_color",
            mat_name == expected_mat,
            f"expected={expected_mat} got={mat_name}",
        )
        # Cap disk sits above the cross hub
        cap_aabb = ctx.part_element_world_aabb(handle, elem=cap)
        hub_aabb = ctx.part_element_world_aabb(handle, elem=handle.get_visual("cross_hub"))
        ctx.check(
            f"{handle_name}_cap_above_hub",
            cap_aabb[0][2] > hub_aabb[1][2] - 0.002,
            f"cap_bottom={cap_aabb[0][2]:.4f} hub_top={hub_aabb[1][2]:.4f}",
        )

    # --- Cross handle geometry: two perpendicular bars ---
    for handle, handle_name in ((hot_handle, "hot"), (cold_handle, "cold")):
        bar_x = handle.get_visual("cross_bar_x")
        bar_y = handle.get_visual("cross_bar_y")
        bar_x_aabb = ctx.part_element_world_aabb(handle, elem=bar_x)
        bar_y_aabb = ctx.part_element_world_aabb(handle, elem=bar_y)
        span_x_x = bar_x_aabb[1][0] - bar_x_aabb[0][0]
        span_y_y = bar_y_aabb[1][1] - bar_y_aabb[0][1]
        ctx.check(
            f"{handle_name}_cross_bar_x_spans_x",
            span_x_x > 0.06,
            f"bar_x span_x={span_x_x:.4f}",
        )
        ctx.check(
            f"{handle_name}_cross_bar_y_spans_y",
            span_y_y > 0.06,
            f"bar_y span_y={span_y_y:.4f}",
        )

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
        ctx.allow_overlap(
            handle,
            col,
            elem_a=handle.get_visual("handle_stem"),
            elem_b=col.get_visual("stem_collar"),
            reason="handle stem passes through the visible stem collar ring",
        )
        # Proof: stem stays centered within the collar bore.
        ctx.expect_within(
            handle,
            col,
            axes="xy",
            inner_elem=handle.get_visual("handle_stem"),
            outer_elem=col.get_visual("stem_collar"),
            margin=0.002,
            name=f"{'hot' if handle is hot_handle else 'cold'}_stem_centered_in_collar",
        )
        ctx.expect_overlap(
            handle,
            col,
            axes="z",
            elem_a=handle.get_visual("handle_stem"),
            elem_b=col.get_visual("stem_collar"),
            min_overlap=0.005,
            name=f"{'hot' if handle is hot_handle else 'cold'}_stem_passes_through_collar",
        )

    # --- joint plan: types, axes, ranges -----------------------------------
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

    # --- placement: 0.30 m spread, all three pieces seated on the deck -----
    ctx.check(
        "widespread_0p30_spread",
        abs(hot_pos[0] + 0.15) < 1e-6
        and abs(cold_pos[0] - 0.15) < 1e-6
        and abs(spout_pos[0]) < 1e-6,
        f"hot_x={hot_pos[0]} cold_x={cold_pos[0]} spout_x={spout_pos[0]}",
    )
    # Columns and spout base sit on the escutcheon (escutcheon-deck contact
    # already checked above in the escutcheon section).
    for piece in (spout_base, hot_col, cold_col):
        ctx.expect_contact(piece, escutcheon, contact_tol=1e-4)

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

    # --- articulation behavior ---------------------------------------------
    # Cross handle: at q=0, bar_x spans X and bar_y spans Y.
    # At q=+90 deg, bar_x spans Y and bar_y spans X (swapped).
    with ctx.pose({hot_turn: 0.0}):
        bx0 = ctx.part_element_world_aabb(hot_handle, elem=hot_handle.get_visual("cross_bar_x"))
    with ctx.pose({hot_turn: math.pi / 2}):
        bx90 = ctx.part_element_world_aabb(hot_handle, elem=hot_handle.get_visual("cross_bar_x"))
    span_x_0 = bx0[1][0] - bx0[0][0]
    span_y_0 = bx0[1][1] - bx0[0][1]
    span_x_90 = bx90[1][0] - bx90[0][0]
    span_y_90 = bx90[1][1] - bx90[0][1]
    ctx.check(
        "hot_handle_cross_rotates_about_vertical",
        span_x_0 > 0.06 and span_y_0 < 0.025 and span_y_90 > 0.06 and span_x_90 < 0.025,
        f"q=0 span=({span_x_0:.3f},{span_y_0:.3f}) q=90 span=({span_x_90:.3f},{span_y_90:.3f})",
    )

    # Spout swivel: +45 deg swings the forward outlet toward -X.
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
