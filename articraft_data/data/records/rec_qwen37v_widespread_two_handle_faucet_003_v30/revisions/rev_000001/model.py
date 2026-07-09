from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet with cross handles.

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (revolute about the column vertical axis, -45..+45 deg),
- hot (left) and cold (right): valve columns topped by cross-style handles
  on short vertical axles (each revolute about its column vertical axis,
  -90..+90 deg). Cross handles are asymmetrically angled but visually
  balanced around the spout.

Narrow seams at all three deck bases (integral stepped flanges). All
faucet surfaces matte black; tiny red/blue indicator dots on handle hubs.
Modeled at true scale in meters; deck bottom on z=0.
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

# Cross handle (replaces T-lever)
AXLE_R = 0.007  # visible vertical axle radius
AXLE_VISIBLE = 0.020  # axle height above valve column top
AXLE_EMBED = 0.015  # axle embedment into column bore
CROSS_HUB_R = 0.012  # central hub radius
CROSS_HUB_H = 0.012  # hub height
CROSS_ARM_LEN = 0.042  # each arm total bar length (center to tip * 2)
CROSS_ARM_R = 0.005  # arm cylinder radius

# Asymmetric rest angles for cross handles
HOT_ANGLE_DEG = 35
COLD_ANGLE_DEG = -10
HOT_REST_ANGLE = math.radians(HOT_ANGLE_DEG)
COLD_REST_ANGLE = math.radians(COLD_ANGLE_DEG)

# Seam ring at deck bases
SEAM_EXTRA = 0.002  # radial extension beyond flange radius
SEAM_H = 0.001  # seam step height

# Indicator dots
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


def _flange_with_seam(flange_r: float, flange_h: float, name: str):
    """Flange cylinder with integral thin seam ring at the base.

    The seam ring extends SEAM_EXTRA beyond the flange radius and is
    SEAM_H tall, creating a visible stepped seam at the deck junction.
    """
    outer_r = flange_r + SEAM_EXTRA
    body = cq.Workplane("XY").circle(flange_r).extrude(flange_h)
    ring = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(flange_r - 0.001)  # 1 mm inside for clean boolean union
        .extrude(SEAM_H)
    )
    return mesh_from_cadquery(body.union(ring), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_black_cross_handle_faucet")

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
        _flange_with_seam(SPOUT_FLANGE_R, SPOUT_FLANGE_H, "spout_flange_seam"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
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
    def _valve_column(name: str) -> object:
        col = model.part(name)
        col.visual(
            _flange_with_seam(VALVE_FLANGE_R, VALVE_FLANGE_H, f"{name}_flange_seam"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=matte_black,
            name="valve_flange",
        )
        col.visual(
            Cylinder(radius=VALVE_COL_R, length=VALVE_COL_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H / 2)),
            material=matte_black,
            name="valve_body",
        )
        return col

    def _cross_handle(name: str, rest_angle_rad: float, dot_material: object) -> object:
        """Cross handle: 4 arms on a central hub atop a short vertical axle."""
        lever = model.part(name)

        # Short vertical axle (visible + embedded)
        axle_total = AXLE_VISIBLE + AXLE_EMBED
        lever.visual(
            Cylinder(radius=AXLE_R, length=axle_total),
            origin=Origin(xyz=(0.0, 0.0, (AXLE_VISIBLE - AXLE_EMBED) / 2)),
            material=matte_black,
            name="handle_axle",
        )

        # Central hub at top of axle
        hub_z = AXLE_VISIBLE + CROSS_HUB_H / 2
        lever.visual(
            Cylinder(radius=CROSS_HUB_R, length=CROSS_HUB_H),
            origin=Origin(xyz=(0.0, 0.0, hub_z)),
            material=matte_black,
            name="cross_hub",
        )

        # Two crossing bars (forming 4 arms) at hub center height
        bar_z = AXLE_VISIBLE + CROSS_HUB_H / 2
        arm_half = CROSS_ARM_LEN / 2

        for ang in (rest_angle_rad, rest_angle_rad + math.pi / 2):
            deg_label = round(math.degrees(ang)) % 360
            cos_a = math.cos(ang)
            sin_a = math.sin(ang)

            # Horizontal bar along direction (cos_a, sin_a)
            lever.visual(
                Cylinder(radius=CROSS_ARM_R, length=CROSS_ARM_LEN),
                origin=Origin(
                    xyz=(0.0, 0.0, bar_z),
                    rpy=(0.0, math.pi / 2, ang),
                ),
                material=matte_black,
                name=f"cross_arm_{deg_label}",
            )

            # End caps (spheres at each bar end)
            for end_sign, end_label in ((1.0, "tip"), (-1.0, "tail")):
                lever.visual(
                    Sphere(radius=CROSS_ARM_R),
                    origin=Origin(xyz=(
                        end_sign * arm_half * cos_a,
                        end_sign * arm_half * sin_a,
                        bar_z,
                    )),
                    material=matte_black,
                    name=f"arm_cap_{deg_label}_{end_label}",
                )

        # Tiny temperature indicator dot on the hub front
        lever.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, CROSS_HUB_R - 0.0005, hub_z)),
            material=dot_material,
            name="indicator_dot",
        )
        return lever

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    # Hot on the left (-X), cold on the right (+X); +Y is toward the user.
    hot_lever = _cross_handle("hot_lever", HOT_REST_ANGLE, hot_red)
    cold_lever = _cross_handle("cold_lever", COLD_REST_ANGLE, cold_blue)

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

    # Intentional hidden engagements: spout riser and handle axles seat inside
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
            elem_a=lever.get_visual("handle_axle"),
            elem_b=col.get_visual("valve_body"),
            reason="handle axle seats 15 mm into the valve cartridge bore",
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

    # --- narrow seams at all three deck bases ------------------------------
    for col, flange_name, flange_r in (
        (spout_base, "base_flange", SPOUT_FLANGE_R),
        (hot_col, "valve_flange", VALVE_FLANGE_R),
        (cold_col, "valve_flange", VALVE_FLANGE_R),
    ):
        flange_aabb = ctx.part_element_world_aabb(col, elem=flange_name)
        x_span = flange_aabb[1][0] - flange_aabb[0][0]
        min_with_seam = 2.0 * (flange_r + SEAM_EXTRA * 0.8)
        ctx.check(
            f"{col.name}_has_deck_seam",
            x_span >= min_with_seam,
            f"flange x_span={x_span:.4f} expected>={min_with_seam:.4f}",
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

    # --- cross handle geometry: 2 crossing bars (4 arms) per handle --------
    for lever in (hot_lever, cold_lever):
        arm_bars = [v for v in lever.visuals if v.name.startswith("cross_arm_")]
        ctx.check(
            f"{lever.name}_has_cross_shape",
            len(arm_bars) == 2,
            f"found {len(arm_bars)} crossing bars (expected 2)",
        )
        # Short vertical axle present
        ctx.check(
            f"{lever.name}_has_vertical_axle",
            lever.get_visual("handle_axle") is not None,
            "missing handle_axle visual",
        )

    # --- cross handles asymmetrically angled but balanced ------------------
    hot_arm_name = f"cross_arm_{HOT_ANGLE_DEG}"
    cold_arm_name = f"cross_arm_{COLD_ANGLE_DEG % 360}"
    hot_arm_aabb = ctx.part_element_world_aabb(hot_lever, elem=hot_arm_name)
    cold_arm_aabb = ctx.part_element_world_aabb(cold_lever, elem=cold_arm_name)
    hot_x_span = hot_arm_aabb[1][0] - hot_arm_aabb[0][0]
    hot_y_span = hot_arm_aabb[1][1] - hot_arm_aabb[0][1]
    cold_x_span = cold_arm_aabb[1][0] - cold_arm_aabb[0][0]
    cold_y_span = cold_arm_aabb[1][1] - cold_arm_aabb[0][1]
    # X/Y span ratio of individual arms differs because the angles differ
    hot_ratio = hot_x_span / max(hot_y_span, 1e-6)
    cold_ratio = cold_x_span / max(cold_y_span, 1e-6)
    ctx.check(
        "cross_handles_asymmetric_angles",
        abs(hot_ratio - cold_ratio) > 0.5,
        f"hot_ratio={hot_ratio:.2f} cold_ratio={cold_ratio:.2f}",
    )

    # --- articulation behavior ---------------------------------------------
    # Cross handle rotation proof: track an arm tip position at q=0 vs q=90°
    hot_tip_name = f"arm_cap_{HOT_ANGLE_DEG}_tip"
    with ctx.pose({hot_turn: 0.0}):
        cap_q0 = ctx.part_element_world_aabb(hot_lever, elem=hot_tip_name)
    with ctx.pose({hot_turn: math.pi / 2}):
        cap_q90 = ctx.part_element_world_aabb(hot_lever, elem=hot_tip_name)
    cap_x_0 = 0.5 * (cap_q0[0][0] + cap_q0[1][0])
    cap_x_90 = 0.5 * (cap_q90[0][0] + cap_q90[1][0])
    ctx.check(
        "hot_cross_handle_rotates_about_vertical",
        abs(cap_x_0 - cap_x_90) > 0.01,
        f"cap x at q=0: {cap_x_0:.4f}, at q=90: {cap_x_90:.4f}",
    )

    cold_tip_name = f"arm_cap_{COLD_ANGLE_DEG % 360}_tip"
    with ctx.pose({cold_turn: 0.0}):
        ccap_q0 = ctx.part_element_world_aabb(cold_lever, elem=cold_tip_name)
    with ctx.pose({cold_turn: math.pi / 2}):
        ccap_q90 = ctx.part_element_world_aabb(cold_lever, elem=cold_tip_name)
    ccap_x_0 = 0.5 * (ccap_q0[0][0] + ccap_q0[1][0])
    ccap_x_90 = 0.5 * (ccap_q90[0][0] + ccap_q90[1][0])
    ctx.check(
        "cold_cross_handle_rotates_about_vertical",
        abs(ccap_x_0 - ccap_x_90) > 0.01,
        f"cap x at q=0: {ccap_x_0:.4f}, at q=90: {ccap_x_90:.4f}",
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

    # Indicator dots: red on hot, blue on cold, proud of the hub front.
    for lever, mat in ((hot_lever, "hot_red"), (cold_lever, "cold_blue")):
        dot = lever.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{lever.name}_dot_material", mat_name == mat, f"material={mat_name}")

    return ctx.report()


object_model = build_object_model()
