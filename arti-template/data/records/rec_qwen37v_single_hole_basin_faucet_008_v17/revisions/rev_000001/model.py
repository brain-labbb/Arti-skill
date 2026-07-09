from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobBore,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet with flat rectangular slot outlet, ~0.13 m tall.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# The body leans back a few degrees from vertical.
# ---------------------------------------------------------------------------

TILT = math.radians(6.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Main body barrel.
BODY_R = 0.025
BODY_S0 = 0.006
BODY_S1 = 0.0725

# Thin recessed separation groove ring around the upper third.
GROOVE_R = 0.0215
GROOVE_S0 = 0.0705
GROOVE_S1 = 0.0760

# Stepped-in upper neck above the groove (extended for knob mount).
NECK_R = 0.0228
NECK_S0 = 0.0740
NECK_S1 = 0.110

# Spout exit station on the body axis.
SPOUT_S = 0.050

# Flow knob on top.
KNOB_D = 0.024
KNOB_H = 0.020
KNOB_TURN_LIMIT = math.radians(90.0)

# Aerator insert.
AERATOR_R = 0.016
AERATOR_H = 0.003

# Spout nozzle geometry constants (in spout local frame).
SPOUT_END_X = 0.063
SPOUT_NOZZLE_TOP_Z = -0.025
SPOUT_NOZZLE_BOT_Z = -0.040


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_shape() -> cq.Workplane:
    """Chrome spout with flat rectangular slot outlet and hollow bore.
    Built in spout-local frame whose origin sits on the body axis at SPOUT_S;
    the shank runs along local +X."""
    r_out = 0.015
    shank_x0 = 0.010  # seated ~15 mm inside the body casting
    shank_x1 = 0.035
    bend = 0.028  # bend radius; end heads straight down
    end_x = shank_x1 + bend  # 0.063
    end_z = -bend  # -0.028

    # Outer tube sweep: straight shank then 90° downward arc.
    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Nozzle housing: wider cylindrical section at the spout end.
    nozzle_r = 0.019
    nozzle_h = 0.015
    nozzle_top_z = end_z + 0.003  # -0.025
    nozzle_bot_z = nozzle_top_z - nozzle_h  # -0.040

    nozzle = (
        cq.Workplane("XY", origin=(end_x, 0.0, nozzle_bot_z))
        .circle(nozzle_r)
        .extrude(nozzle_h)
    )
    spout = tube.union(nozzle)

    # Hollow cavity inside the nozzle (visible from below — real hollow outlet).
    cavity_r = 0.015
    cavity_depth = 0.011
    cavity = (
        cq.Workplane("XY", origin=(end_x, 0.0, nozzle_bot_z - 0.001))
        .circle(cavity_r)
        .extrude(cavity_depth + 0.002)
    )
    spout = spout.cut(cavity)

    # Vertical bore through the tube connecting to the cavity.
    bore_r = 0.011
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, nozzle_bot_z + 0.001))
        .circle(bore_r)
        .extrude(0.038)
    )
    spout = spout.cut(bore)

    # Flat rectangular slot at the bottom of the nozzle (the outlet).
    slot_w = 0.022  # width in Y
    slot_d = 0.004  # depth in X
    slot_cut_h = 0.008
    slot = (
        cq.Workplane("XY", origin=(end_x, 0.0, nozzle_bot_z - 0.002))
        .rect(slot_d, slot_w)
        .extrude(slot_cut_h)
    )
    spout = spout.cut(slot)

    return spout


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_slot_outlet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("aerator_mesh", rgba=(0.55, 0.58, 0.60, 1.0))
    model.material("indicator_mark", rgba=(0.18, 0.20, 0.23, 1.0))

    # ---------------- body (root): flange + barrel + groove + neck ---------
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
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

    # ---------------- spout (fixed): swept tube + nozzle + rectangular slot -
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout_shell", tolerance=0.0003),
        material="chrome",
        name="spout_shell",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ---------------- aerator (fixed to spout, circular insert) -------------
    aerator = model.part("aerator")
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_H),
        origin=Origin(xyz=(0.0, 0.0, AERATOR_H / 2.0)),
        material="aerator_mesh",
        name="aerator_disc",
    )
    # Aerator seated inside the nozzle cavity, slightly press-fit into walls.
    # Joint origin in spout local frame: at (end_x, 0, nozzle_bot_z + 0.001)
    # so aerator disc spans z = [nozzle_bot_z+0.001, nozzle_bot_z+0.004] in spout frame.
    aerator_joint_z = SPOUT_NOZZLE_BOT_Z + 0.001
    model.articulation(
        "aerator_mount",
        ArticulationType.FIXED,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(SPOUT_END_X, 0.0, aerator_joint_z)),
    )

    # ---------------- flow knob (revolute on body top for flow control) -----
    knob = model.part("flow_knob")
    knob_geom = KnobGeometry(
        KNOB_D,
        KNOB_H,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=16, depth=0.001),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        bore=KnobBore(style="round", diameter=0.006),
        center=False,  # mounting face at z=0
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "flow_knob_mesh"),
        origin=Origin(),
        material="chrome",
        name="knob_shell",
    )
    # Small indicator dot on the knob top for tracking rotation.
    knob.visual(
        Cylinder(radius=0.003, length=0.001),
        origin=Origin(xyz=(0.008, 0.0, KNOB_H + 0.0005)),
        material="indicator_mark",
        name="flow_indicator_dot",
    )
    # Revolute joint about the tilted body axis at the neck top.
    # Positive q rotates the knob for flow-on (quarter turn).
    model.articulation(
        "knob_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=_axis_point(NECK_S1), rpy=(0.0, -TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=KNOB_TURN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    knob = object_model.get_part("flow_knob")
    knob_turn = object_model.get_articulation("knob_turn")

    # ---- intentional seated insertions (scoped per element) ---------------
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_shell",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated ~15 mm into the solid body casting.",
    )
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="aerator_disc",
        elem_b="spout_shell",
        reason="Aerator disc is press-fit into the hollow nozzle cavity walls.",
    )

    # ---- base flange seated on deck ----------------------------------------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )

    # ---- body leans back ---------------------------------------------------
    neck_aabb = ctx.part_element_world_aabb(body, elem="body_neck")
    ctx.check(
        "body leans back (neck offset toward -X behind flange center)",
        neck_aabb is not None and (neck_aabb[0][0] + neck_aabb[1][0]) / 2.0 < -0.005,
        details=f"neck aabb={neck_aabb}",
    )

    # ---- spout: rectangular slot outlet, reaches forward and down ----------
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward and curves down above the deck",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.055
        and spout_aabb[0][2] < 0.025
        and spout_aabb[0][2] > -0.015,
        details=f"spout aabb={spout_aabb}",
    )

    # The nozzle housing is wider than the tube — proves the slot housing exists.
    spout_shell_aabb = ctx.part_element_world_aabb(spout, elem="spout_shell")
    ctx.check(
        "spout shell includes nozzle housing wider than tube for slot outlet",
        spout_shell_aabb is not None
        and (spout_shell_aabb[1][1] - spout_shell_aabb[0][1]) > 0.034,
        details=f"spout_shell aabb={spout_shell_aabb}",
    )

    # Spout bottom extends below the bend center — proves the nozzle drops down.
    ctx.check(
        "spout nozzle drops below the bend (slot outlet is low)",
        spout_aabb is not None
        and spout_aabb[0][2] < 0.015,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- aerator: circular disc seated inside spout nozzle -----------------
    ctx.expect_within(
        aerator,
        spout,
        axes="xy",
        inner_elem="aerator_disc",
        outer_elem="spout_shell",
        margin=0.005,
        name="aerator centered within spout nozzle in xy",
    )
    ctx.expect_overlap(
        aerator,
        spout,
        axes="z",
        elem_a="aerator_disc",
        elem_b="spout_shell",
        min_overlap=0.001,
        name="aerator seated inside spout nozzle in z",
    )

    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator is a compact circular disc",
        aerator_aabb is not None
        and abs(
            (aerator_aabb[1][0] - aerator_aabb[0][0])
            - (aerator_aabb[1][1] - aerator_aabb[0][1])
        )
        < 0.005
        and (aerator_aabb[1][2] - aerator_aabb[0][2]) < 0.006,
        details=f"aerator aabb={aerator_aabb}",
    )

    # ---- flow knob: cylindrical, sits on top of body ----------------------
    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check(
        "flow knob sits above the neck top",
        knob_aabb is not None
        and neck_aabb is not None
        and knob_aabb[0][2] > neck_aabb[1][2] - 0.004,
        details=f"knob aabb={knob_aabb}, neck aabb={neck_aabb}",
    )

    knob_shell_aabb = ctx.part_element_world_aabb(knob, elem="knob_shell")
    ctx.check(
        "flow knob has cylindrical proportions (roughly equal XY extents, taller than wide ratio)",
        knob_shell_aabb is not None
        and abs(
            (knob_shell_aabb[1][0] - knob_shell_aabb[0][0])
            - (knob_shell_aabb[1][1] - knob_shell_aabb[0][1])
        )
        < 0.005
        and (knob_shell_aabb[1][2] - knob_shell_aabb[0][2]) > 0.012,
        details=f"knob_shell aabb={knob_shell_aabb}",
    )

    # ---- overall height ~0.13 m --------------------------------------------
    ctx.check(
        "overall faucet height is about 0.13 m",
        knob_aabb is not None and 0.115 <= knob_aabb[1][2] <= 0.140,
        details=f"knob aabb={knob_aabb}",
    )

    # ---- articulation: knob_turn is revolute with 0–90° limits -------------
    tl = knob_turn.motion_limits
    ctx.check(
        "knob turn limits are 0 to 90 degrees",
        tl is not None
        and tl.lower is not None
        and tl.upper is not None
        and abs(tl.lower) < 1e-9
        and abs(tl.upper - KNOB_TURN_LIMIT) < 1e-6,
        details=f"limits={tl}",
    )

    ctx.check(
        "knob_turn is a revolute articulation (non-fixed joint)",
        knob_turn.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={knob_turn.articulation_type}",
    )

    # ---- decisive pose: knob rotation swings the indicator dot --------------
    dot_rest = ctx.part_element_world_aabb(knob, elem="flow_indicator_dot")
    with ctx.pose({knob_turn: KNOB_TURN_LIMIT}):
        dot_turned = ctx.part_element_world_aabb(knob, elem="flow_indicator_dot")

    ctx.check(
        "turning the knob 90° swings the flow indicator around the axis",
        dot_rest is not None
        and dot_turned is not None
        and abs((dot_rest[0][1] + dot_rest[1][1]) / 2.0) < 0.005
        and abs((dot_turned[0][1] + dot_turned[1][1]) / 2.0) > 0.004,
        details=f"rest={dot_rest}, turned={dot_turned}",
    )

    return ctx.report()


object_model = build_object_model()
