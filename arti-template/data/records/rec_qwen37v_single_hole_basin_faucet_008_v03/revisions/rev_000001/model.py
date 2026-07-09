from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    place_on_face,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet, modern squared monobloc, ~0.13 m tall, chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# ---------------------------------------------------------------------------

# Base flange (round, sits flat on deck).
FLANGE_R = 0.033
FLANGE_H = 0.005

# Squared monobloc body dimensions.
BODY_X = 0.044  # width (left-right)
BODY_Y = 0.036  # depth (front-back)
BODY_Z = 0.098  # height

# Spout exit: front face of body, at about 60% body height.
SPOUT_EXIT_Z = 0.005 + BODY_Z * 0.60  # from deck

# Flow knob on top.
KNOB_DIAMETER = 0.030
KNOB_HEIGHT = 0.016
KNOB_STEM_R = 0.005
KNOB_STEM_H = 0.010  # stem that nests into the body bore

# Knob rotation limits for flow control.
KNOB_LOWER = math.radians(-90.0)
KNOB_UPPER = math.radians(90.0)


def _build_body_shape() -> cq.Workplane:
    """Squared monobloc body with a bore on top for the knob stem and a
    circular port on the front face for the spout."""
    # Main rectangular column, base at z=0.
    body = (
        cq.Workplane("XY")
        .box(BODY_X, BODY_Y, BODY_Z, centered=(True, True, False))
    )
    # Slight edge chamfer for a crisp modern look (small).
    body = body.edges("|Z").chamfer(0.001)

    # Bore on top for the knob stem (blind hole).
    body = (
        body.faces(">Z").workplane()
        .circle(KNOB_STEM_R + 0.001)
        .cutBlind(-KNOB_STEM_H + 0.002)
    )

    # Circular port on the front face for the spout shank.
    port_r = 0.016
    port_z = SPOUT_EXIT_Z - 0.005  # relative to body base
    body = (
        body.faces(">X").workplane(centerOption="CenterOfBoundBox")
        .center(0.0, port_z - BODY_Z / 2.0)
        .circle(port_r)
        .cutBlind(-0.012)
    )
    return body


def _build_spout_shape() -> cq.Workplane:
    """Chrome spout with a straight shank, smooth downward curve, and a real
    hollow outlet at the mouth. Built in spout-local frame: origin at the
    front face of the body, shank runs along +X."""
    r_out = 0.014
    shank_x0 = -0.010  # starts inside the body port
    shank_x1 = 0.040
    bend = 0.030
    end_x = shank_x1 + bend
    end_z = -bend

    # Sweep path: straight then arc down.
    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = (
        cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0))
        .circle(r_out)
        .sweep(path)
    )

    # Flared outlet skirt at the down-turned end.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.006))
        .circle(0.0138)
        .workplane(offset=-0.012)
        .circle(0.018)
        .loft()
    )
    spout = tube.union(flare)

    # Real hollow outlet: tapered bore cutting through the outlet mouth.
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.008))
        .circle(0.015)
        .workplane(offset=0.020)
        .circle(0.010)
        .loft()
    )
    return spout.cut(bore)


def _build_knob_shape() -> cq.Workplane:
    """Cylindrical flow-control knob with a knurled grip and pointer line.
    Local origin at the knob base (mounting face), extends along +Z."""
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=32, depth=0.0008, helix_angle_deg=18.0),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        bore=KnobBore(style="round", diameter=KNOB_STEM_R * 2.0),
        center=False,
    )
    # Build stem that nests into the body bore.
    from sdk import mesh_from_geometry as _mfg
    knob_mesh = _mfg(knob_geom, "knob_body")
    # We'll add the stem as a separate visual on the same part; return raw CQ
    # for the stem only.
    stem = (
        cq.Workplane("XY")
        .circle(KNOB_STEM_R)
        .extrude(-KNOB_STEM_H)
    )
    return stem


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_squared")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("index_mark", rgba=(0.25, 0.27, 0.30, 1.0))

    # ---------- body (root): flange + squared monobloc --------------------
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "monobloc_body", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H)),
        material="chrome",
        name="body_block",
    )

    # ---------- spout (fixed): swept tube with hollow outlet --------------
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
        origin=Origin(xyz=(BODY_X / 2.0, 0.0, SPOUT_EXIT_Z + FLANGE_H)),
    )

    # ---------- flow knob (revolute on top of body) -----------------------
    knob = model.part("flow_knob")
    # Knob mesh (cylindrical with grip and indicator, base at z=0).
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=32, depth=0.0008, helix_angle_deg=18.0),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        bore=KnobBore(style="round", diameter=KNOB_STEM_R * 2.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "knob_shell"),
        material="chrome",
        name="knob_shell",
    )
    # Stem that nests into body bore.
    knob.visual(
        Cylinder(radius=KNOB_STEM_R, length=KNOB_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, -KNOB_STEM_H / 2.0)),
        material="chrome_dark",
        name="knob_stem",
    )

    # Knob frame origin sits at the top face of the body; the knob sits proud.
    body_top_z = FLANGE_H + BODY_Z
    model.articulation(
        "knob_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, body_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=KNOB_LOWER, upper=KNOB_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    knob = object_model.get_part("flow_knob")
    knob_turn = object_model.get_articulation("knob_turn")

    # --- Intentional overlaps: spout seated in body port, knob stem in bore --
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_block",
        reason="Spout shank is intentionally seated into the body front port.",
    )
    ctx.allow_overlap(
        knob,
        body,
        elem_a="knob_stem",
        elem_b="body_block",
        reason="Knob stem nests inside the body top bore for rotation.",
    )

    # --- Squared monobloc body geometry checks -------------------------------
    body_block_aabb = ctx.part_element_world_aabb(body, elem="body_block")
    ctx.check(
        "body is a squared monobloc (box-like, taller than wide)",
        body_block_aabb is not None,
        details=f"body_block aabb={body_block_aabb}",
    )
    if body_block_aabb is not None:
        dx = body_block_aabb[1][0] - body_block_aabb[0][0]
        dy = body_block_aabb[1][1] - body_block_aabb[0][1]
        dz = body_block_aabb[1][2] - body_block_aabb[0][2]
        ctx.check(
            "body is taller than wide (squared column)",
            dz > dx and dz > dy,
            details=f"dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}",
        )
        ctx.check(
            "body width is roughly 0.04-0.05 m (squared profile)",
            0.038 <= dx <= 0.050,
            details=f"dx={dx:.4f}",
        )

    # --- Base flange on deck -------------------------------------------------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )

    # --- Spout: projects forward, curves down, hollow outlet -----------------
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout projects forward from the body and curves downward",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.060
        and spout_aabb[0][2] < 0.040
        and spout_aabb[0][2] > 0.005,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.003,
        name="spout shank stays seated in the body",
    )

    # --- Flow knob on top of body --------------------------------------------
    ctx.expect_gap(
        knob,
        body,
        axis="z",
        min_gap=-0.012,
        max_gap=0.005,
        elem_a="knob_shell",
        elem_b="body_block",
        name="knob shell sits at or just above the body top face",
    )
    ctx.expect_overlap(
        knob,
        body,
        axes="z",
        elem_a="knob_stem",
        elem_b="body_block",
        min_overlap=0.004,
        name="knob stem retained inside the body bore",
    )
    ctx.expect_within(
        knob,
        body,
        axes="xy",
        inner_elem="knob_stem",
        margin=0.003,
        name="knob stem stays centered in the body bore",
    )

    # --- Overall height check ------------------------------------------------
    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check(
        "overall faucet height is about 0.11-0.14 m",
        knob_aabb is not None and 0.110 <= knob_aabb[1][2] <= 0.140,
        details=f"knob aabb={knob_aabb}",
    )

    # --- Articulation: knob revolute limits ----------------------------------
    kl = knob_turn.motion_limits
    ctx.check(
        "knob turn limits are -90 to +90 degrees",
        kl is not None
        and kl.lower is not None
        and kl.upper is not None
        and abs(kl.lower - KNOB_LOWER) < 1e-6
        and abs(kl.upper - KNOB_UPPER) < 1e-6,
        details=f"limits={kl}",
    )
    ctx.check(
        "knob articulation is revolute",
        knob_turn.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={knob_turn.articulation_type}",
    )

    # --- Decisive pose: turning knob rotates the indicator -------------------
    knob_rest = ctx.part_world_position(knob)
    with ctx.pose({knob_turn: KNOB_UPPER}):
        knob_turned = ctx.part_world_position(knob)
    # The knob origin should not translate (revolute only).
    ctx.check(
        "knob rotation does not translate the knob origin",
        knob_rest is not None
        and knob_turned is not None
        and abs(knob_rest[2] - knob_turned[2]) < 0.001
        and abs(knob_rest[0] - knob_turned[0]) < 0.001,
        details=f"rest={knob_rest}, turned={knob_turned}",
    )

    return ctx.report()


object_model = build_object_model()
