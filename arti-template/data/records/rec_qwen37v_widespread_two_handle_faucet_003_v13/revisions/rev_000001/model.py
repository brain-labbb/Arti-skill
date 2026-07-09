from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet with cross handles.

Three independent deck-mounted columns on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a swiveling gooseneck spout
  (continuous rotation about the column's vertical axis),
- hot (left) and cold (right): valve columns topped by cross-shaped handles
  (each revolute about its column's vertical axis, -90..+90 deg).

Cross handles have four arms radiating from a central hub, with visible
stem collars beneath and separate hot/cold indicator cap disks on top.
All faucet surfaces matte black. Modeled at true scale in meters;
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

# Stem collar (visible ring between valve column and handle)
STEM_COLLAR_R = 0.028
STEM_COLLAR_H = 0.008

# Cross handle (in the handle part frame, origin at valve column top)
HANDLE_STEM_R = 0.009
HANDLE_STEM_EMBED = 0.015
HANDLE_STEM_RISE = 0.030  # stem rises above collar
HUB_R = 0.014  # central hub radius
HUB_H = 0.016  # hub height
ARM_R = 0.007  # cross arm radius
ARM_LEN = 0.055  # arm half-length (from hub center outward)
CAP_DISK_R = 0.012
CAP_DISK_H = 0.003

ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(HOOK_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(HOOK_DEG))
AERATOR_LEN = 0.016
AERATOR_R = 0.017
# Unit tangent of the arc at the hook end (pointing out of the spout, downward).
_TX = math.sin(math.radians(HOOK_DEG))  # y component
_TZ = -math.cos(math.radians(HOOK_DEG))  # z component
AERATOR_CY = ARC_END_Y + _TX * (AERATOR_LEN / 2 - 0.004)
AERATOR_CZ = ARC_END_Z + _TZ * (AERATOR_LEN / 2 - 0.004)

# Hub center Z in the handle part frame (above stem embed)
HUB_Z = HANDLE_STEM_RISE + HUB_H / 2


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_cross_handle_faucet")

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

    # Continuous vertical swivel joint for the spout
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
        return col

    def _cross_handle(name: str, cap_material: object) -> object:
        """Cross-shaped handle: stem + hub + 4 arms + cap disk."""
        handle = model.part(name)
        # Vertical stem (partially embedded in valve column)
        handle.visual(
            Cylinder(radius=HANDLE_STEM_R, length=HANDLE_STEM_RISE + HANDLE_STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (HANDLE_STEM_RISE - HANDLE_STEM_EMBED) / 2)),
            material=matte_black,
            name="handle_stem",
        )
        # Central hub at top of stem
        handle.visual(
            Cylinder(radius=HUB_R, length=HUB_H),
            origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
            material=matte_black,
            name="handle_hub",
        )
        # Cross arms: one bar along X, one bar along Y, both at hub center height
        arm_z = HANDLE_STEM_RISE + HUB_H / 2
        # Bar along X axis
        handle.visual(
            Cylinder(radius=ARM_R, length=ARM_LEN * 2),
            origin=Origin(xyz=(0.0, 0.0, arm_z), rpy=(0.0, math.pi / 2, 0.0)),
            material=matte_black,
            name="cross_arm_x",
        )
        # Bar along Y axis
        handle.visual(
            Cylinder(radius=ARM_R, length=ARM_LEN * 2),
            origin=Origin(xyz=(0.0, 0.0, arm_z), rpy=(math.pi / 2, 0.0, 0.0)),
            material=matte_black,
            name="cross_arm_y",
        )
        # Rounded arm tips (4 spheres on X-bar ends, 4 on Y-bar ends)
        for dx in (-1.0, 1.0):
            handle.visual(
                Sphere(radius=ARM_R),
                origin=Origin(xyz=(dx * ARM_LEN, 0.0, arm_z)),
                material=matte_black,
                name=f"arm_tip_x{'n' if dx < 0 else 'p'}",
            )
        for dy in (-1.0, 1.0):
            handle.visual(
                Sphere(radius=ARM_R),
                origin=Origin(xyz=(0.0, dy * ARM_LEN, arm_z)),
                material=matte_black,
                name=f"arm_tip_y{'n' if dy < 0 else 'p'}",
            )
        # Cap disk on top of hub (hot=red, cold=blue)
        cap_z = HANDLE_STEM_RISE + HUB_H + CAP_DISK_H / 2
        handle.visual(
            Cylinder(radius=CAP_DISK_R, length=CAP_DISK_H),
            origin=Origin(xyz=(0.0, 0.0, cap_z)),
            material=cap_material,
            name="cap_disk",
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

    # Stem collars are separate visuals on the handle parts, positioned
    # at the top of the valve column to show the visible collar ring.
    collar_z_in_handle = -HANDLE_STEM_EMBED + VALVE_COL_H - VALVE_COL_H + STEM_COLLAR_H / 2
    # Actually, stem collar sits right at the top of valve column.
    # The handle part frame origin is at valve column top, so collar center
    # is at z = STEM_COLLAR_H/2 (just above column top, below handle stem rise).
    for handle in (hot_handle, cold_handle):
        handle.visual(
            Cylinder(radius=STEM_COLLAR_R, length=STEM_COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_COLLAR_H / 2)),
            material=matte_black,
            name="stem_collar",
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

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_turn = object_model.get_articulation("hot_handle_turn")
    cold_turn = object_model.get_articulation("cold_handle_turn")

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

    # --- joint plan: spout is continuous, handles are revolute ---------------
    ctx.check(
        "spout_swivel_is_continuous",
        str(spout_swivel.joint_type).lower().endswith("continuous")
        and tuple(spout_swivel.axis) == (0.0, 0.0, 1.0),
        f"type={spout_swivel.joint_type} axis={spout_swivel.axis}",
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

    # --- cross handle form: 4 arms in a + pattern --------------------------
    for handle, sign in ((hot_handle, -1.0), (cold_handle, 1.0)):
        arm_x = handle.get_visual("cross_arm_x")
        arm_y = handle.get_visual("cross_arm_y")
        arm_x_aabb = ctx.part_element_world_aabb(handle, elem=arm_x)
        arm_y_aabb = ctx.part_element_world_aabb(handle, elem=arm_y)
        span_x = arm_x_aabb[1][0] - arm_x_aabb[0][0]
        span_y = arm_y_aabb[1][1] - arm_y_aabb[0][1]
        ctx.check(
            f"{handle.name}_cross_arms_perpendicular",
            span_x > 0.08 and span_y > 0.08,
            f"x-arm span={span_x:.3f} y-arm span={span_y:.3f}",
        )
        # Arms clear the valve column top
        ctx.expect_gap(
            handle,
            (hot_col if sign < 0 else cold_col),
            axis="z",
            positive_elem=arm_x,
            min_gap=0.01,
        )

    # --- stem collars visible under handles --------------------------------
    for handle in (hot_handle, cold_handle):
        collar = handle.get_visual("stem_collar")
        collar_aabb = ctx.part_element_world_aabb(handle, elem=collar)
        hub_aabb = ctx.part_element_world_aabb(handle, elem=handle.get_visual("handle_hub"))
        ctx.check(
            f"{handle.name}_collar_below_hub",
            collar_aabb[1][2] < hub_aabb[0][2] + 0.005,
            f"collar top z={collar_aabb[1][2]:.4f} hub bottom z={hub_aabb[0][2]:.4f}",
        )
        # Collar is wider than the stem
        collar_span = max(
            collar_aabb[1][0] - collar_aabb[0][0],
            collar_aabb[1][1] - collar_aabb[0][1],
        )
        ctx.check(
            f"{handle.name}_collar_wider_than_stem",
            collar_span > 2 * HANDLE_STEM_R + 0.01,
            f"collar span={collar_span:.4f}",
        )

    # --- cap disks: hot=red, cold=blue, on top of handle hubs --------------
    for handle, mat_name in ((hot_handle, "hot_red"), (cold_handle, "cold_blue")):
        cap = handle.get_visual("cap_disk")
        cap_mat = cap.material if isinstance(cap.material, str) else cap.material.name
        ctx.check(
            f"{handle.name}_cap_disk_material",
            cap_mat == mat_name,
            f"material={cap_mat}",
        )
        # Cap disk sits above the hub
        cap_aabb = ctx.part_element_world_aabb(handle, elem=cap)
        hub_aabb = ctx.part_element_world_aabb(handle, elem=handle.get_visual("handle_hub"))
        ctx.check(
            f"{handle.name}_cap_above_hub",
            cap_aabb[0][2] > hub_aabb[1][2] - 0.002,
            f"cap bottom z={cap_aabb[0][2]:.4f} hub top z={hub_aabb[1][2]:.4f}",
        )

    # --- articulation behavior: handle rotation proof ----------------------
    with ctx.pose({hot_turn: 0.0}):
        arm0 = ctx.part_element_world_aabb(hot_handle, elem=hot_handle.get_visual("cross_arm_x"))
    with ctx.pose({hot_turn: math.pi / 2}):
        arm90 = ctx.part_element_world_aabb(hot_handle, elem=hot_handle.get_visual("cross_arm_x"))
    span_x0 = arm0[1][0] - arm0[0][0]
    span_y0 = arm0[1][1] - arm0[0][1]
    span_x90 = arm90[1][0] - arm90[0][0]
    span_y90 = arm90[1][1] - arm90[0][1]
    ctx.check(
        "hot_handle_rotates_about_vertical_axis",
        span_x0 > 0.08 and span_y0 < 0.03 and span_y90 > 0.08 and span_x90 < 0.03,
        f"closed span=({span_x0:.3f},{span_y0:.3f}) turned span=({span_x90:.3f},{span_y90:.3f})",
    )

    # Spout continuous swivel: rotation swings the outlet sideways.
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
