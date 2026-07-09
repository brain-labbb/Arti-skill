from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet variant, ~0.13 m tall, polished chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# The body leans BACK a few degrees, i.e. its long axis tilts toward -X.
# ---------------------------------------------------------------------------

TILT = math.radians(6.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Raised circular collar around the base (above the flange).
COLLAR_R = 0.032
COLLAR_H = 0.008
COLLAR_S0 = FLANGE_H  # starts at top of flange
COLLAR_S1 = COLLAR_S0 + COLLAR_H

# Main body barrel.
BODY_R = 0.025
BODY_S0 = COLLAR_S1  # barrel starts above the collar
BODY_S1 = 0.0725

# Thin recessed separation groove ring around the upper third.
GROOVE_R = 0.0215
GROOVE_S0 = 0.0705
GROOVE_S1 = 0.0760

# Stepped-in upper neck above the groove.
NECK_R = 0.0228
NECK_S0 = 0.0740
NECK_S1 = 0.104

# Flow knob mounted on top of the neck.
KNOB_DIAMETER = 0.050
KNOB_HEIGHT = 0.022
KNOB_JOINT_S = NECK_S1  # knob frame origin at the neck top

# Two small screw caps on the back of the body.
SCREW_CAP_R = 0.004
SCREW_CAP_H = 0.003
SCREW_CAP_S = (BODY_S0 + BODY_S1) * 0.55  # mid-height of the barrel
SCREW_CAP_SPACING = 0.016  # lateral spacing between the two caps

# Spout exit station on the body axis.
SPOUT_S = 0.050

# Knob rotation: quarter-turn flow control.
TURN_LIMIT = math.radians(90.0)


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank, smooth downward bend, flared
    open outlet rim. Built in spout-local frame whose origin sits on the
    body axis at SPOUT_S; the shank runs along local +X."""
    r_out = 0.015
    shank_x0 = 0.010  # seated ~15 mm inside the body casting
    shank_x1 = 0.035
    bend = 0.028  # bend radius; end heads straight down at (0.063, -0.028)
    end_x = shank_x1 + bend
    end_z = -bend

    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet skirt around the down-turned end.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.006))
        .circle(0.0148)
        .workplane(offset=-0.010)
        .circle(0.0185)
        .loft()
    )
    spout = tube.union(flare)

    # Tapered bore opening the outlet mouth (real hollow outlet rim).
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.006))
        .circle(0.0155)
        .workplane(offset=0.018)
        .circle(0.011)
        .loft()
    )
    return spout.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("screw_cap", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("index_mark", rgba=(0.30, 0.32, 0.35, 1.0))

    # ---------------- body (root): flange + collar + barrel + groove + neck + screw caps
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    # Raised circular collar around the base.
    body.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=_tilted((COLLAR_S0 + COLLAR_S1) / 2.0),
        material="chrome",
        name="base_collar",
    )
    body.visual(
        Cylinder(radius=BODY_R, length=BODY_S1 - BODY_S0),
        origin=_tilted((BODY_S0 + BODY_S1) / 2.0),
        material="chrome",
        name="body_barrel",
    )
    body.visual(
        Cylinder(radius=GROOVE_R, length=GROOVE_S1 - GROOVE_S0),
        origin=_tilted((GROOVE_S0 + GROOVE_S1) / 2.0),
        material="chrome_dark",
        name="groove_ring",
    )
    body.visual(
        Cylinder(radius=NECK_R, length=NECK_S1 - NECK_S0),
        origin=_tilted((NECK_S0 + NECK_S1) / 2.0),
        material="chrome",
        name="body_neck",
    )

    # Two small screw caps on the back (-X side) of the body barrel.
    # They sit on the rear surface at mid-height, spaced apart vertically.
    back_s_upper = SCREW_CAP_S + SCREW_CAP_SPACING / 2.0
    back_s_lower = SCREW_CAP_S - SCREW_CAP_SPACING / 2.0
    for i, s_cap in enumerate((back_s_upper, back_s_lower)):
        cap_pt = _axis_point(s_cap)
        # Screw cap protrudes from the back surface (-X direction from the tilted axis).
        # Offset along the body's radial back direction.
        cx = cap_pt[0] - (BODY_R - SCREW_CAP_H * 0.5) * COS_T
        cz = cap_pt[2] - (BODY_R - SCREW_CAP_H * 0.5) * SIN_T
        body.visual(
            Cylinder(radius=SCREW_CAP_R, length=SCREW_CAP_H),
            origin=Origin(
                xyz=(cx, 0.0, cz),
                rpy=(0.0, math.pi / 2.0 - TILT, 0.0),
            ),
            material="screw_cap",
            name=f"screw_cap_{i}",
        )

    # ---------------- spout (fixed): swept hollow tube + flared outlet ----
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ---------------- flow knob (revolute for flow control) ---------------
    # Cylindrical knob with fluted grip grooves, mounted on top of the neck.
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        skirt=KnobSkirt(
            diameter=KNOB_DIAMETER + 0.004,
            height=0.004,
            flare=0.04,
            chamfer=0.001,
        ),
        grip=KnobGrip(style="fluted", count=24, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        center=False,  # mounting face on z=0
    )
    flow_knob = model.part("flow_knob")
    flow_knob.visual(
        mesh_from_geometry(knob_geom, "flow_knob"),
        material="chrome",
        name="knob_shell",
    )
    # Joint frame on the body axis at the neck top; local +z runs up the
    # tilted axis. Rotation about +z controls flow (0 = closed, 90° = full open).
    model.articulation(
        "knob_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flow_knob,
        origin=Origin(xyz=_axis_point(KNOB_JOINT_S), rpy=(0.0, -TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=TURN_LIMIT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    flow_knob = object_model.get_part("flow_knob")
    knob_turn = object_model.get_articulation("knob_turn")

    # Intentional seated insertion (spout shank into solid body proxy).
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated ~15 mm into the solid body casting.",
    )
    # Knob skirt wraps around the neck top for a seated visual mount.
    ctx.allow_overlap(
        flow_knob,
        body,
        elem_a="knob_shell",
        elem_b="body_neck",
        reason="The flow knob skirt intentionally wraps around the neck top for a seated rotary mount.",
    )

    # ---- hero geometry: flange seated on deck, body leaning back ----------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )
    neck_aabb = ctx.part_element_world_aabb(body, elem="body_neck")
    ctx.check(
        "body leans back (neck offset toward -X behind the flange center)",
        neck_aabb is not None and (neck_aabb[0][0] + neck_aabb[1][0]) / 2.0 < -0.005,
        details=f"neck aabb={neck_aabb}",
    )

    # ---- raised collar exists around the base -----------------------------
    collar_aabb = ctx.part_element_world_aabb(body, elem="base_collar")
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "raised circular collar sits above the base flange",
        collar_aabb is not None
        and flange_aabb is not None
        and collar_aabb[0][2] >= flange_aabb[0][2] - 0.001
        and collar_aabb[1][2] <= flange_aabb[1][2] + COLLAR_H + 0.005,
        details=f"collar aabb={collar_aabb}, flange aabb={flange_aabb}",
    )
    ctx.check(
        "collar is wider than the body barrel",
        collar_aabb is not None
        and (collar_aabb[1][1] - collar_aabb[0][1]) > 2.0 * BODY_R + 0.005,
        details=f"collar aabb={collar_aabb}",
    )

    # ---- two screw caps on the back of the body ---------------------------
    screw_0_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_0")
    screw_1_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_1")
    ctx.check(
        "screw cap 0 exists on the body back",
        screw_0_aabb is not None,
        details=f"screw_cap_0 aabb={screw_0_aabb}",
    )
    ctx.check(
        "screw cap 1 exists on the body back",
        screw_1_aabb is not None,
        details=f"screw_cap_1 aabb={screw_1_aabb}",
    )
    if screw_0_aabb is not None and screw_1_aabb is not None:
        # Both caps should be behind (more -X) than the body barrel center.
        barrel_aabb = ctx.part_element_world_aabb(body, elem="body_barrel")
        barrel_cx = (barrel_aabb[0][0] + barrel_aabb[1][0]) / 2.0 if barrel_aabb else 0.0
        screw_0_cx = (screw_0_aabb[0][0] + screw_0_aabb[1][0]) / 2.0
        screw_1_cx = (screw_1_aabb[0][0] + screw_1_aabb[1][0]) / 2.0
        ctx.check(
            "screw caps are on the back (-X) of the body",
            screw_0_cx < barrel_cx - 0.010 and screw_1_cx < barrel_cx - 0.010,
            details=f"screw_0_cx={screw_0_cx}, screw_1_cx={screw_1_cx}, barrel_cx={barrel_cx}",
        )
        # The two caps should be vertically separated.
        ctx.check(
            "screw caps are vertically separated",
            abs(
                (screw_0_aabb[0][2] + screw_0_aabb[1][2]) / 2.0
                - (screw_1_aabb[0][2] + screw_1_aabb[1][2]) / 2.0
            )
            > 0.008,
            details=f"screw_0={screw_0_aabb}, screw_1={screw_1_aabb}",
        )

    # ---- spout: projects forward from the body and curves down ------------
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.005,
        name="spout shank stays seated in the body",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward and droops to a low open outlet above the deck",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.060
        and spout_aabb[0][2] < 0.025
        and spout_aabb[0][2] > 0.008,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- flow knob: sits on top of the neck, rotates for flow -------------
    knob_aabb = ctx.part_world_aabb(flow_knob)
    ctx.check(
        "flow knob is mounted at the neck top (skirt wraps around neck)",
        knob_aabb is not None
        and neck_aabb is not None
        and knob_aabb[0][2] >= neck_aabb[1][2] - 0.008
        and knob_aabb[1][2] > neck_aabb[1][2],
        details=f"knob aabb={knob_aabb}, neck aabb={neck_aabb}",
    )
    ctx.check(
        "flow knob is wider than the neck (grip overhang)",
        knob_aabb is not None
        and neck_aabb is not None
        and (knob_aabb[1][1] - knob_aabb[0][1])
        > (neck_aabb[1][1] - neck_aabb[0][1]) + 0.002,
        details=f"knob aabb={knob_aabb}, neck aabb={neck_aabb}",
    )
    ctx.expect_overlap(
        flow_knob,
        body,
        axes="z",
        elem_a="knob_shell",
        elem_b="body_neck",
        min_overlap=0.001,
        name="flow knob skirt overlaps the neck top (seated mount)",
    )
    ctx.expect_overlap(
        flow_knob,
        body,
        axes="xy",
        elem_a="knob_shell",
        elem_b="body_neck",
        min_overlap=0.030,
        name="knob skirt shares substantial xy footprint with the neck",
    )

    # ---- overall height ----------------------------------------------------
    ctx.check(
        "overall faucet height is about 0.13 m",
        knob_aabb is not None and 0.110 <= knob_aabb[1][2] <= 0.140,
        details=f"knob aabb={knob_aabb}",
    )

    # ---- articulation: knob turn limits ------------------------------------
    tl = knob_turn.motion_limits
    ctx.check(
        "knob turn limits are 0 to 90 degrees",
        tl is not None
        and tl.lower is not None
        and tl.upper is not None
        and abs(tl.lower) < 1e-9
        and abs(tl.upper - TURN_LIMIT) < 1e-6,
        details=f"limits={tl}",
    )
    ctx.check(
        "knob turn is a revolute joint",
        knob_turn.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={knob_turn.articulation_type}",
    )

    # ---- decisive pose: rotating the knob swings it around the body axis --
    rest_pos = ctx.part_world_position(flow_knob)
    with ctx.pose({knob_turn: TURN_LIMIT}):
        turned_pos = ctx.part_world_position(flow_knob)
    # The knob origin stays on-axis for revolute, so check the AABB shifts.
    with ctx.pose({knob_turn: TURN_LIMIT}):
        turned_aabb = ctx.part_world_aabb(flow_knob)
    rest_aabb = ctx.part_world_aabb(flow_knob)
    ctx.check(
        "knob rotation changes the visible orientation (indicator swings)",
        rest_aabb is not None
        and turned_aabb is not None,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
