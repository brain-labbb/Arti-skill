from __future__ import annotations

"""Matte-black widespread two-handle bathroom faucet with low bridge arch spout.

Three-piece widespread layout (total spread 0.30 m):
- center: cylindrical base column supporting a low bridge arch that spans
  laterally between the handle positions, with a short downward outlet
- hot (left) and cold (right): valve columns with decorative ring ridges,
  topped by lever handles that tilt forward-back (revolute about X axis,
  -45..+45 deg each)

Narrow seam rings at all three deck bases. All surfaces matte black; tiny
red/blue indicator dots on handle stems. Modeled at true scale in meters;
deck bottom on z=0.
"""

import math

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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
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
SPOUT_COL_H = 0.08  # column height above deck surface (shorter for low bridge)

# Bridge arch (in bridge_spout part frame, origin at column top)
BRIDGE_HALF = 0.11  # arch spans from -BRIDGE_HALF to +BRIDGE_HALF
BRIDGE_RISE = 0.040  # peak height above column top
TUBE_R = 0.012  # bridge tube radius

# Connector from column top to bridge tube underside
CONNECTOR_H = BRIDGE_RISE - TUBE_R  # fills gap between column top and tube bottom

# Downward outlet nozzle from bridge center
OUTLET_R = 0.010
OUTLET_LEN = 0.030  # hangs from bridge peak downward

# Valve pieces
VALVE_FLANGE_R = 0.036
VALVE_FLANGE_H = 0.010
VALVE_COL_R = 0.0225
VALVE_COL_H = 0.10  # column top = lever joint height above the deck surface

# Decorative ring ridges on valve pedestals
RING_MAJOR = VALVE_COL_R + 0.002  # sits proud of column surface
RING_TUBE = 0.003
RING_HEIGHTS = (0.04, 0.07)  # heights above column base

# Seam rings (thin dark ridges at flange-column junctions)
SEAM_MAJOR = VALVE_COL_R  # at column surface
SEAM_TUBE = 0.0015
SPOUT_SEAM_MAJOR = SPOUT_COL_R

# Lever (tilts forward-back about X axis)
STEM_R = 0.009
STEM_EMBED = 0.015
STEM_TOP = 0.040
BAR_R = 0.009
BAR_LEN = 0.10  # arm extends toward user
DOT_R = 0.0035


def _bridge_arch_mesh():
    """Smooth tube mesh for the lateral bridge arch."""
    return tube_from_spline_points(
        [
            (-BRIDGE_HALF, 0.0, 0.0),
            (-BRIDGE_HALF * 0.55, 0.0, BRIDGE_RISE * 0.72),
            (0.0, 0.0, BRIDGE_RISE),
            (BRIDGE_HALF * 0.55, 0.0, BRIDGE_RISE * 0.72),
            (BRIDGE_HALF, 0.0, 0.0),
        ],
        radius=TUBE_R,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_bridge_faucet")

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
    # Narrow seam ring at flange-column junction
    spout_base.visual(
        mesh_from_geometry(
            TorusGeometry(SPOUT_SEAM_MAJOR, SEAM_TUBE),
            "spout_seam_ring",
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

    # -------------------------------------------------------- bridge arch spout
    bridge_spout = model.part("bridge_spout")
    # Connector from column top to bridge arch underside
    bridge_spout.visual(
        Cylinder(radius=TUBE_R + 0.003, length=CONNECTOR_H),
        origin=Origin(xyz=(0.0, 0.0, CONNECTOR_H / 2)),
        material=matte_black,
        name="bridge_connector",
    )
    # Bridge arch tube spanning laterally
    bridge_spout.visual(
        mesh_from_geometry(_bridge_arch_mesh(), "bridge_arch_tube"),
        material=matte_black,
        name="bridge_arch",
    )
    # Downward outlet nozzle from bridge center
    bridge_spout.visual(
        Cylinder(radius=OUTLET_R, length=OUTLET_LEN),
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_RISE - OUTLET_LEN / 2)),
        material=matte_black,
        name="outlet_nozzle",
    )

    model.articulation(
        "spout_base_to_bridge",
        ArticulationType.FIXED,
        parent=spout_base,
        child=bridge_spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_COL_H)),
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
        # Narrow seam ring at flange-column junction
        col.visual(
            mesh_from_geometry(
                TorusGeometry(SEAM_MAJOR, SEAM_TUBE),
                f"{name}_seam_ring",
            ),
            origin=Origin(xyz=(0.0, 0.0, VALVE_FLANGE_H)),
            material=seam_dark,
            name="base_seam",
        )
        # Decorative ring ridges on the pedestal
        for i, h in enumerate(RING_HEIGHTS):
            col.visual(
                mesh_from_geometry(
                    TorusGeometry(RING_MAJOR, RING_TUBE),
                    f"{name}_ring_{i}",
                ),
                origin=Origin(xyz=(0.0, 0.0, h)),
                material=matte_black,
                name=f"decorative_ring_{i}",
            )
        return col

    def _lever_handle(name: str, dot_material: object) -> object:
        lever = model.part(name)
        # Vertical stem (seats into valve column bore)
        lever.visual(
            Cylinder(radius=STEM_R, length=STEM_TOP + STEM_EMBED),
            origin=Origin(xyz=(0.0, 0.0, (STEM_TOP - STEM_EMBED) / 2)),
            material=matte_black,
            name="lever_stem",
        )
        # Horizontal arm extending toward user (+Y direction)
        lever.visual(
            Cylinder(radius=BAR_R, length=BAR_LEN),
            origin=Origin(
                xyz=(0.0, BAR_LEN / 2, STEM_TOP),
                rpy=(-math.pi / 2, 0.0, 0.0),
            ),
            material=matte_black,
            name="lever_arm",
        )
        # End cap sphere
        lever.visual(
            Sphere(radius=BAR_R * 1.1),
            origin=Origin(xyz=(0.0, BAR_LEN, STEM_TOP)),
            material=matte_black,
            name="arm_cap",
        )
        # Tiny temperature indicator dot on the stem front
        lever.visual(
            Sphere(radius=DOT_R),
            origin=Origin(xyz=(0.0, STEM_R - 0.0005, 0.020)),
            material=dot_material,
            name="indicator_dot",
        )
        return lever

    hot_valve_column = _valve_column("hot_valve_column")
    cold_valve_column = _valve_column("cold_valve_column")
    hot_lever = _lever_handle("hot_lever", hot_red)
    cold_lever = _lever_handle("cold_lever", cold_blue)

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

    # Lever tilt joints: forward-back rotation about X axis
    for joint_name, parent, child in (
        ("hot_lever_tilt", hot_valve_column, hot_lever),
        ("cold_lever_tilt", cold_valve_column, cold_lever),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=parent,
            child=child,
            origin=Origin(xyz=(0.0, 0.0, VALVE_COL_H)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=-math.pi / 4,
                upper=math.pi / 4,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("sink_deck")
    spout_base = object_model.get_part("spout_base")
    bridge = object_model.get_part("bridge_spout")
    hot_col = object_model.get_part("hot_valve_column")
    cold_col = object_model.get_part("cold_valve_column")
    hot_lever = object_model.get_part("hot_lever")
    cold_lever = object_model.get_part("cold_lever")

    hot_tilt = object_model.get_articulation("hot_lever_tilt")
    cold_tilt = object_model.get_articulation("cold_lever_tilt")

    # --- intentional hidden engagements: lever stems seat inside columns ----
    for lever, col in ((hot_lever, hot_col), (cold_lever, cold_col)):
        ctx.allow_overlap(
            lever,
            col,
            elem_a=lever.get_visual("lever_stem"),
            elem_b=col.get_visual("valve_body"),
            reason="lever stem seats 15 mm into the valve cartridge bore",
        )

    # --- joint plan: forward-back tilt about X axis, +/-45 deg ------------
    for joint in (hot_tilt, cold_tilt):
        ctx.check(
            f"{joint.name}_is_x_axis_revolute",
            str(joint.joint_type).lower().endswith("revolute")
            and tuple(joint.axis) == (1.0, 0.0, 0.0),
            f"axis={joint.axis}",
        )
        ml = joint.motion_limits
        ctx.check(
            f"{joint.name}_range_pm45",
            ml is not None
            and abs(ml.lower + math.pi / 4) < 1e-6
            and abs(ml.upper - math.pi / 4) < 1e-6,
            f"lower={ml.lower} upper={ml.upper}",
        )

    # --- placement: 0.30 m widespread spread ------------------------------
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

    # --- all three pieces seated on deck ----------------------------------
    for piece in (spout_base, hot_col, cold_col):
        ctx.expect_contact(piece, deck, contact_tol=1e-5)

    # --- bridge arch is LOW (not a tall gooseneck) -----------------------
    bridge_aabb = ctx.part_world_aabb(bridge)
    bridge_peak_above_deck = bridge_aabb[1][2] - DECK_T
    ctx.check(
        "bridge_arch_is_low",
        0.06 < bridge_peak_above_deck < 0.18,
        f"bridge peak {bridge_peak_above_deck:.3f} m above deck",
    )
    # Bridge spans laterally between handles
    bridge_span_x = bridge_aabb[1][0] - bridge_aabb[0][0]
    ctx.check(
        "bridge_spans_laterally",
        bridge_span_x > 0.16,
        f"bridge X span = {bridge_span_x:.3f} m",
    )

    # --- narrow seams at all three deck bases ----------------------------
    for piece in (spout_base, hot_col, cold_col):
        seam = piece.get_visual("base_seam")
        ctx.check(
            f"{piece.name}_has_seam_ring",
            seam is not None,
            f"missing seam ring on {piece.name}",
        )

    # --- decorative ring ridges on handle pedestals ---------------------
    for col in (hot_col, cold_col):
        for ring_idx in range(len(RING_HEIGHTS)):
            ring = col.get_visual(f"decorative_ring_{ring_idx}")
            ctx.check(
                f"{col.name}_has_ring_{ring_idx}",
                ring is not None,
                f"missing decorative ring {ring_idx} on {col.name}",
            )

    # --- lever arms extend toward user (+Y) ------------------------------
    for lever in (hot_lever, cold_lever):
        arm_aabb = ctx.part_element_world_aabb(
            lever, elem=lever.get_visual("lever_arm")
        )
        arm_span_y = arm_aabb[1][1] - arm_aabb[0][1]
        ctx.check(
            f"{lever.name}_arm_extends_forward",
            arm_span_y > 0.07,
            f"arm Y span = {arm_span_y:.3f} m",
        )

    # --- articulation behavior: lever tilt changes arm endpoint height ---
    with ctx.pose({hot_tilt: 0.0}):
        cap_rest = ctx.part_element_world_aabb(
            hot_lever, elem=hot_lever.get_visual("arm_cap")
        )
    with ctx.pose({hot_tilt: math.pi / 4}):
        cap_tilted = ctx.part_element_world_aabb(
            hot_lever, elem=hot_lever.get_visual("arm_cap")
        )
    z_rest = 0.5 * (cap_rest[0][2] + cap_rest[1][2])
    z_tilted = 0.5 * (cap_tilted[0][2] + cap_tilted[1][2])
    ctx.check(
        "hot_lever_tilt_raises_arm",
        z_tilted > z_rest + 0.015,
        f"rest z={z_rest:.4f} tilted z={z_tilted:.4f}",
    )

    # --- indicator dots: red on hot, blue on cold -----------------------
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
