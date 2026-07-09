from __future__ import annotations

"""Matte-black widespread two-handle faucet with low bridge arch spout.

Three-piece widespread layout on a sink deck (total spread 0.30 m):
- center: cylindrical base column with a low bridge arch spout that arcs
  forward from the column top to a hollow downward outlet (~0.10 m above deck).
- hot (left) and cold (right): valve columns topped by cross-style handles
  (four radiating arms on a vertical stem), each revolute about its column's
  vertical axis, -90..+90 deg.

Narrow seam rings at all three deck bases. All surfaces matte black;
red/blue indicator dots on the handle stems.
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
SPOUT_COL_H = 0.07  # shorter column for low bridge arch

# Seam ring (thin annular groove at deck base of each column)
SEAM_RING_RO = 0.046  # outer radius (slightly wider than flange)
SEAM_RING_RI = 0.038  # inner radius (slightly narrower than flange)
SEAM_RING_H = 0.002  # very thin

# Low bridge arch tube (in spout part frame, origin at column top)
TUBE_OR = 0.014  # outer radius of arch tube
TUBE_IR = 0.010  # inner radius (hollow bore)
RISER_EMBED = 0.025  # hidden engagement into the column below the joint
RISER_TOP = 0.04  # straight riser ends here; arc starts (above column top)
ARC_R = 0.055  # arch arc radius
ARC_END_DEG = -25.0  # arc sweeps forward and slightly down
COLLAR_R = 0.019
COLLAR_H = 0.014

# Compute arc endpoint
ARC_END_Y = ARC_R + ARC_R * math.cos(math.radians(ARC_END_DEG))
ARC_END_Z = RISER_TOP + ARC_R * math.sin(math.radians(ARC_END_DEG))

# Aerator / outlet nozzle at arch tip
AERATOR_LEN = 0.018
AERATOR_OR = 0.016  # outer radius
AERATOR_IR = 0.011  # inner bore radius
# Tangent at arc end (pointing forward-down)
_TX = math.sin(math.radians(ARC_END_DEG))
_TZ = -math.cos(math.radians(ARC_END_DEG))
AERATOR_CY = ARC_END_Y + _TX * (AERATOR_LEN / 2 - 0.003)
AERATOR_CZ = ARC_END_Z + _TZ * (AERATOR_LEN / 2 - 0.003)

# Valve pieces
VALVE_FLANGE_R = 0.036
VALVE_FLANGE_H = 0.010
VALVE_COL_R = 0.0225
VALVE_COL_H = 0.08  # shorter columns for cross-handle style

# Valve seam ring
VALVE_SEAM_RO = 0.040
VALVE_SEAM_RI = 0.032

# Cross handle (in handle part frame, origin at valve column top)
STEM_R = 0.009
STEM_EMBED = 0.012
STEM_TOP = 0.032  # height of stem above column top
HUB_R = 0.013  # central hub radius
HUB_H = 0.012  # hub height
ARM_R = 0.007  # arm cylinder radius
ARM_LEN = 0.040  # arm length from hub center to tip
DOT_R = 0.0035


def _bridge_arch_outer() -> cq.Workplane:
    """Swept low-bridge arch tube (outer shell)."""
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -RISER_EMBED)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (ARC_END_Y, ARC_END_Z))
    )
    profile = cq.Workplane("XY").workplane(offset=-RISER_EMBED).circle(TUBE_OR)
    return profile.sweep(path, isFrenet=True)


def _bridge_arch_bore() -> cq.Workplane:
    """Swept hollow bore through the arch tube."""
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -RISER_EMBED)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (ARC_END_Y, ARC_END_Z))
    )
    profile = cq.Workplane("XY").workplane(offset=-RISER_EMBED).circle(TUBE_IR)
    return profile.sweep(path, isFrenet=True)


def _hollow_arch_tube() -> cq.Workplane:
    """Hollow arch tube: outer shell minus inner bore."""
    outer = _bridge_arch_outer()
    bore = _bridge_arch_bore()
    return outer.cut(bore)


def _seam_ring(outer_r: float, inner_r: float) -> cq.Workplane:
    """Thin annular seam ring."""
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(SEAM_RING_H)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_bridge_arch_faucet")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.07, 1.0))
    deck_stone = model.material("deck_stone", rgba=(0.80, 0.79, 0.76, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.02, 0.02, 0.02, 1.0))
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
    # Seam ring at deck base
    spout_base.visual(
        mesh_from_cadquery(_seam_ring(SEAM_RING_RO, SEAM_RING_RI), "spout_seam"),
        origin=Origin(xyz=(0.0, 0.0, -SEAM_RING_H)),
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

    # -------------------------------------------------------- bridge arch spout
    bridge_spout = model.part("bridge_spout")
    bridge_spout.visual(
        mesh_from_cadquery(_hollow_arch_tube(), "arch_tube"),
        material=matte_black,
        name="arch_tube",
    )
    # Swivel collar at base of arch
    bridge_spout.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H / 2)),
        material=matte_black,
        name="swivel_collar",
    )
    # Hollow aerator outlet at arch tip - outer shell
    bridge_spout.visual(
        Cylinder(radius=AERATOR_OR, length=AERATOR_LEN),
        origin=Origin(
            xyz=(0.0, AERATOR_CY, AERATOR_CZ),
            rpy=(math.radians(ARC_END_DEG), 0.0, 0.0),
        ),
        material=matte_black,
        name="aerator_shell",
    )
    # Visible hollow bore inside aerator (dark interior ring)
    bridge_spout.visual(
        Cylinder(radius=AERATOR_IR, length=AERATOR_LEN * 0.6),
        origin=Origin(
            xyz=(0.0, AERATOR_CY, AERATOR_CZ),
            rpy=(math.radians(ARC_END_DEG), 0.0, 0.0),
        ),
        material=seam_dark,
        name="aerator_bore",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=spout_base,
        child=bridge_spout,
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
        # Seam ring at deck base
        col.visual(
            mesh_from_cadquery(_seam_ring(VALVE_SEAM_RO, VALVE_SEAM_RI), f"{name}_seam"),
            origin=Origin(xyz=(0.0, 0.0, -SEAM_RING_H)),
            material=seam_dark,
            name="deck_seam",
        )
        return col

    def _cross_handle(name: str, dot_material: object) -> object:
        handle = model.part(name)
        # Vertical stem (embedded into valve column)
        handle.visual(
            Cylinder(radius=STEM_R, length=STEM_TOP + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (STEM_TOP - STEM_EMBED) / 2)),
            material=matte_black,
            name="handle_stem",
        )
        # Central hub
        handle.visual(
            Cylinder(radius=HUB_R, length=HUB_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_TOP + HUB_H / 2)),
            material=matte_black,
            name="cross_hub",
        )
        # Four cross arms at 0°, 90°, 180°, 270°
        hub_center_z = STEM_TOP + HUB_H / 2
        for i, angle_deg in enumerate([0.0, 90.0, 180.0, 270.0]):
            angle_rad = math.radians(angle_deg)
            # Arm center at hub_center + ARM_LEN/2 along the direction
            cx = (ARM_LEN / 2) * math.cos(angle_rad)
            cy = (ARM_LEN / 2) * math.sin(angle_rad)
            # Cylinder along the arm direction: rotate so local Z aligns with arm
            # rpy: pitch = 90° to lay horizontal, then yaw to aim
            handle.visual(
                Cylinder(radius=ARM_R, length=ARM_LEN),
                origin=Origin(
                    xyz=(cx, cy, hub_center_z),
                    rpy=(0.0, math.pi / 2, angle_rad),
                ),
                material=matte_black,
                name=f"cross_arm_{i}",
            )
            # Spherical cap at arm tip
            tip_x = ARM_LEN * math.cos(angle_rad)
            tip_y = ARM_LEN * math.sin(angle_rad)
            handle.visual(
                Sphere(radius=ARM_R),
                origin=Origin(xyz=(tip_x, tip_y, hub_center_z)),
                material=matte_black,
                name=f"arm_cap_{i}",
            )
        # Temperature indicator dot on the stem front
        handle.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, STEM_R - 0.0005, 0.018)),
            material=dot_material,
            name="indicator_dot",
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
    bridge_spout = object_model.get_part("bridge_spout")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_handle = object_model.get_part("hot_handle")
    cold_handle = object_model.get_part("cold_handle")

    spout_swivel = object_model.get_articulation("spout_swivel")
    hot_turn = object_model.get_articulation("hot_handle_turn")
    cold_turn = object_model.get_articulation("cold_handle_turn")

    arch_tube = bridge_spout.get_visual("arch_tube")
    base_column = spout_base.get_visual("base_column")

    # Intentional hidden engagements: arch tube riser seats inside column,
    # handle stems seat inside valve columns.
    ctx.allow_overlap(
        bridge_spout,
        spout_base,
        elem_a=arch_tube,
        elem_b=base_column,
        reason="arch tube riser seats 25 mm into the spout column bore",
    )
    for handle, col in ((hot_handle, hot_col), (cold_handle, cold_col)):
        ctx.allow_overlap(
            handle,
            col,
            elem_a=handle.get_visual("handle_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="handle stem seats 12 mm into the valve cartridge bore",
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

    # --- low bridge arch: peak is modest, well below gooseneck height ------
    spout_aabb = ctx.part_world_aabb(bridge_spout)
    arch_top_above_deck = spout_aabb[1][2] - DECK_T
    ctx.check(
        "low_bridge_arch_peak",
        0.08 < arch_top_above_deck < 0.18,
        f"arch peak {arch_top_above_deck:.3f} m above deck (should be low)",
    )

    # --- hollow outlet: aerator bore exists as a distinct dark element -----
    aerator_bore = bridge_spout.get_visual("aerator_bore")
    bore_aabb = ctx.part_element_world_aabb(bridge_spout, elem=aerator_bore)
    ctx.check(
        "hollow_outlet_bore_exists",
        bore_aabb is not None and (bore_aabb[1][2] - bore_aabb[0][2]) > 0.001,
        f"bore aabb={bore_aabb}",
    )
    # Outlet is lower than gooseneck would be
    aerator_shell = bridge_spout.get_visual("aerator_shell")
    outlet_aabb = ctx.part_element_world_aabb(bridge_spout, elem=aerator_shell)
    outlet_above_deck = 0.5 * (outlet_aabb[0][2] + outlet_aabb[1][2]) - DECK_T
    ctx.check(
        "low_outlet_height",
        0.05 < outlet_above_deck < 0.16,
        f"outlet center {outlet_above_deck:.3f} m above deck",
    )

    # --- cross handles: 4 arms radiating from hub -------------------------
    for handle in (hot_handle, cold_handle):
        hub = handle.get_visual("cross_hub")
        arm0 = handle.get_visual("cross_arm_0")
        arm1 = handle.get_visual("cross_arm_1")
        # Arms span wider than the hub
        arm0_aabb = ctx.part_element_world_aabb(handle, elem=arm0)
        arm1_aabb = ctx.part_element_world_aabb(handle, elem=arm1)
        arm0_span = arm0_aabb[1][0] - arm0_aabb[0][0]
        arm1_span = arm1_aabb[1][1] - arm1_aabb[0][1]
        ctx.check(
            f"{handle.name}_cross_arms_span",
            arm0_span > 0.03 and arm1_span > 0.03,
            f"arm0_x_span={arm0_span:.4f} arm1_y_span={arm1_span:.4f}",
        )

    # --- seam rings: each base has a visible seam element ------------------
    for base_part in (spout_base, hot_col, cold_col):
        seam = base_part.get_visual("deck_seam")
        ctx.check(
            f"{base_part.name}_has_deck_seam",
            seam is not None,
            f"missing deck_seam visual on {base_part.name}",
        )

    # --- indicator dots: red on hot, blue on cold -------------------------
    for handle, mat in ((hot_handle, "hot_red"), (cold_handle, "cold_blue")):
        dot = handle.get_visual("indicator_dot")
        mat_name = dot.material if isinstance(dot.material, str) else dot.material.name
        ctx.check(f"{handle.name}_dot_material", mat_name == mat, f"material={mat_name}")

    # --- articulation behavior: cross handle rotates about vertical axis --
    # At q=0 arms span X and Y; at q=+45° the X-arm swings to diagonal
    with ctx.pose({hot_turn: 0.0}):
        arm0_at_0 = ctx.part_element_world_aabb(hot_handle, elem=hot_handle.get_visual("cross_arm_0"))
    with ctx.pose({hot_turn: math.pi / 4}):
        arm0_at_45 = ctx.part_element_world_aabb(hot_handle, elem=hot_handle.get_visual("cross_arm_0"))
    # The arm center should shift in Y when rotated 45°
    cx0 = 0.5 * (arm0_at_0[0][0] + arm0_at_0[1][0])
    cy0 = 0.5 * (arm0_at_0[0][1] + arm0_at_0[1][1])
    cx45 = 0.5 * (arm0_at_45[0][0] + arm0_at_45[1][0])
    cy45 = 0.5 * (arm0_at_45[0][1] + arm0_at_45[1][1])
    ctx.check(
        "hot_handle_rotates_about_vertical",
        abs(cy45 - cy0) > 0.01 or abs(cx45 - cx0) > 0.01,
        f"arm0 center: q0=({cx0:.4f},{cy0:.4f}) q45=({cx45:.4f},{cy45:.4f})",
    )

    # --- spout swivel proof -----------------------------------------------
    with ctx.pose({spout_swivel: 0.0}):
        outlet_at_0 = ctx.part_element_world_aabb(bridge_spout, elem=aerator_shell)
    with ctx.pose({spout_swivel: math.pi / 4}):
        outlet_at_45 = ctx.part_element_world_aabb(bridge_spout, elem=aerator_shell)
    ox0 = 0.5 * (outlet_at_0[0][0] + outlet_at_0[1][0])
    ox45 = 0.5 * (outlet_at_45[0][0] + outlet_at_45[1][0])
    ctx.check(
        "spout_swivels_laterally",
        abs(ox45 - ox0) > 0.03,
        f"outlet x: q0={ox0:.4f} q45={ox45:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
