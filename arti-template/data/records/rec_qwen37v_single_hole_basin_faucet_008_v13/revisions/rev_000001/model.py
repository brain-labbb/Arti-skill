from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet – squared modern monobloc variant.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# Body is vertical (no tilt), sharply squared with filleted edges.
# Side lever rotates on a short horizontal axle (X-axis) for flow control.
# ---------------------------------------------------------------------------

# Base gasket (oval, sits flat on the deck).
GASKET_MAJOR = 0.032  # half-length along X
GASKET_MINOR = 0.024  # half-length along Y
GASKET_H = 0.004      # slightly thicker for compression overlap with body

# Body block (squared monobloc).
BODY_W = 0.044  # width along X
BODY_D = 0.036  # depth along Y
BODY_H = 0.098  # height along Z
BODY_Z0 = 0.002  # body bottom slightly overlaps gasket top (compression)

# Spout exit height on the body (center of spout bore).
SPOUT_Z = BODY_Z0 + BODY_H * 0.62

# Lever pivot location: on the +Y face of the body, at ~70% height.
PIVOT_Z = BODY_Z0 + BODY_H * 0.70
PIVOT_Y = BODY_D / 2.0

# Lever travel limits (radians).
LEVER_LOWER = math.radians(-40.0)
LEVER_UPPER = math.radians(40.0)


def _build_body_shape() -> cq.Workplane:
    """Squared monobloc body with filleted vertical edges and a spout bore.
    Built in world-aligned frame, bottom at z = BODY_Z0."""
    body = (
        cq.Workplane("XY", origin=(0.0, 0.0, BODY_Z0))
        .rect(BODY_W, BODY_D)
        .extrude(BODY_H)
    )
    # Fillet the four vertical edges for a modern look.
    body = body.edges("|Z").fillet(0.004)
    # Slight chamfer on the top edges.
    body = body.edges(">Z").chamfer(0.0015)
    # Spout bore hole through the front face at SPOUT_Z height.
    bore_z_local = SPOUT_Z - BODY_Z0
    body = (
        body
        .faces(">X")
        .workplane()
        .center(0.0, bore_z_local - BODY_H / 2.0)
        .circle(0.013)
        .cutBlind(-0.020)
    )
    return body


def _build_gasket_shape() -> cq.Workplane:
    """Oval base gasket: elliptical ring sitting on the deck (z=0 to z=GASKET_H).
    Top overlaps slightly with body bottom for compression representation."""
    outer = (
        cq.Workplane("XY")
        .ellipse(GASKET_MAJOR, GASKET_MINOR)
        .extrude(GASKET_H)
    )
    inner = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .ellipse(GASKET_MAJOR - 0.005, GASKET_MINOR - 0.005)
        .extrude(GASKET_H + 0.002)
    )
    return outer.cut(inner)


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank from the body front face, smooth
    downward bend, flared open outlet rim with real hollow bore.
    Origin at the body front face (X = BODY_W/2), at SPOUT_Z height."""
    r_out = 0.013
    shank_x0 = -0.008  # seated into the body bore
    shank_x1 = 0.032
    bend = 0.028
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
        .circle(0.0128)
        .workplane(offset=-0.010)
        .circle(0.0170)
        .loft()
    )
    spout = tube.union(flare)

    # Real hollow outlet: tapered bore cutting through the outlet mouth.
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.010))
        .circle(0.0140)
        .workplane(offset=0.024)
        .circle(0.009)
        .loft()
    )
    return spout.cut(bore)


def _build_lever_shape() -> cq.Workplane:
    """Side lever: pivot boss + flat handle arm, built in lever-local frame.
    Origin at pivot center. Boss is a box centered at origin.
    Arm extends along +Y (outward from body side)."""
    # Pivot boss: compact box centered at origin, representing the pivot housing.
    boss = cq.Workplane("XY").box(0.016, 0.012, 0.016)

    # Handle arm: long thin bar extending outward along +Y, overlapping boss.
    arm = (
        cq.Workplane("XY", origin=(0.0, 0.030, 0.0))
        .box(0.010, 0.054, 0.008)
    )

    # Union with intentional overlap (boss y extent: -0.006 to 0.006,
    # arm y extent: 0.003 to 0.057 -> 3mm overlap).
    lever = boss.union(arm)
    
    # Fillet the combined shape edges for a smoother look.
    lever = lever.edges("|Z").fillet(0.002)
    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_squared")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("gasket_rubber", rgba=(0.12, 0.12, 0.14, 1.0))

    # ---------------- body (root): squared monobloc -------------------------
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_block", tolerance=0.0003),
        material="chrome",
        name="body_block",
    )

    # ---------------- base gasket (fixed, under body) ----------------------
    gasket = model.part("base_gasket")
    gasket.visual(
        mesh_from_cadquery(_build_gasket_shape(), "gasket_ring", tolerance=0.0003),
        material="gasket_rubber",
        name="gasket_ring",
    )
    # Gasket geometry at z=0..GASKET_H. Body bottom at BODY_Z0=0.002 overlaps
    # gasket by 2mm (compression seal representation).
    model.articulation(
        "gasket_mount",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---------------- spout (fixed): hollow tube + flared outlet -----------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout_tube", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=(BODY_W / 2.0, 0.0, SPOUT_Z)),
    )

    # ---------------- side lever (revolute on horizontal axle) -------------
    lever = model.part("side_lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_shape(), "lever_arm", tolerance=0.0003),
        material="chrome_brushed",
        name="lever_arm",
    )
    # Joint frame: origin at pivot point on body +Y face.
    # Axis along X (horizontal, perpendicular to lever arm direction).
    # Positive q lifts the lever arm upward (+Z) for flow-on.
    model.articulation(
        "lever_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=LEVER_LOWER, upper=LEVER_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    gasket = object_model.get_part("base_gasket")
    spout = object_model.get_part("spout")
    lever = object_model.get_part("side_lever")
    lever_joint = object_model.get_articulation("lever_rotate")

    # --- Intentional overlaps ---
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_block",
        reason="Spout shank is intentionally seated into the body bore.",
    )
    ctx.allow_overlap(
        lever,
        body,
        elem_a="lever_arm",
        elem_b="body_block",
        reason="Lever pivot boss is intentionally embedded into the body side face for pivot mounting.",
    )
    ctx.allow_overlap(
        gasket,
        body,
        elem_a="gasket_ring",
        elem_b="body_block",
        reason="Gasket top surface is compressed against the body bottom (realistic seal).",
    )

    # --- Squared body check: rectangular cross-section, not cylindrical ---
    body_aabb = ctx.part_element_world_aabb(body, elem="body_block")
    ctx.check(
        "body is a squared monobloc (rectangular cross-section with width and depth)",
        body_aabb is not None
        and (body_aabb[1][0] - body_aabb[0][0]) > 0.035
        and (body_aabb[1][1] - body_aabb[0][1]) > 0.025,
        details=f"body aabb={body_aabb}",
    )

    # --- Body height check ---
    ctx.check(
        "body height is approximately 0.10 m",
        body_aabb is not None
        and 0.090 <= (body_aabb[1][2] - body_aabb[0][2]) <= 0.110,
        details=f"body aabb={body_aabb}",
    )

    # --- Oval base gasket: sits on deck, wider than body ---
    gasket_aabb = ctx.part_element_world_aabb(gasket, elem="gasket_ring")
    ctx.check(
        "oval base gasket sits on the deck (bottom near z=0)",
        gasket_aabb is not None
        and gasket_aabb[0][2] >= -0.001
        and gasket_aabb[0][2] <= 0.002,
        details=f"gasket aabb={gasket_aabb}",
    )
    ctx.check(
        "gasket has oval shape (wider in X than Y)",
        gasket_aabb is not None
        and (gasket_aabb[1][0] - gasket_aabb[0][0]) > (gasket_aabb[1][1] - gasket_aabb[0][1]) + 0.010,
        details=f"gasket aabb={gasket_aabb}",
    )
    ctx.expect_overlap(
        gasket,
        body,
        axes="xy",
        min_overlap=0.010,
        name="gasket footprint overlaps the body base",
    )
    # Prove gasket-body compression overlap in Z.
    ctx.expect_gap(
        body,
        gasket,
        axis="z",
        max_penetration=0.004,
        min_gap=-0.004,
        name="gasket compressed against body bottom",
    )

    # --- Spout: projects forward from body and curves down ---
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward from the body (x extent well beyond body)",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.055,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout outlet droops below the spout exit height",
        spout_aabb is not None
        and spout_aabb[0][2] < SPOUT_Z - 0.010,
        details=f"spout aabb={spout_aabb}, SPOUT_Z={SPOUT_Z}",
    )
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.003,
        name="spout shank stays seated in the body",
    )

    # --- Side lever: mounted on the +Y side of the body ---
    lever_aabb = ctx.part_world_aabb(lever)
    body_aabb_for_lever = ctx.part_world_aabb(body)
    ctx.check(
        "side lever extends beyond body +Y face",
        lever_aabb is not None
        and body_aabb_for_lever is not None
        and lever_aabb[1][1] > body_aabb_for_lever[1][1] + 0.010,
        details=f"lever aabb={lever_aabb}, body aabb={body_aabb_for_lever}",
    )

    # --- Lever joint is revolute with correct limits ---
    ll = lever_joint.motion_limits
    ctx.check(
        "lever joint is revolute with -40 to +40 degree limits",
        ll is not None
        and ll.lower is not None
        and ll.upper is not None
        and abs(ll.lower - LEVER_LOWER) < 1e-6
        and abs(ll.upper - LEVER_UPPER) < 1e-6,
        details=f"limits={ll}",
    )

    # --- Decisive pose: lever rotates upward for flow-on ---
    # Use AABB to detect arm motion (part origin stays at pivot point).
    lever_rest_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({lever_joint: LEVER_UPPER}):
        lever_up_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever positive rotation lifts the arm upward (flow on direction)",
        lever_rest_aabb is not None
        and lever_up_aabb is not None
        and lever_up_aabb[1][2] > lever_rest_aabb[1][2] + 0.003,
        details=f"rest_top={lever_rest_aabb[1][2]}, up_top={lever_up_aabb[1][2]}",
    )

    # --- Negative pose: lever rotates downward ---
    with ctx.pose({lever_joint: LEVER_LOWER}):
        lever_down_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever negative rotation drops the arm downward (flow off direction)",
        lever_rest_aabb is not None
        and lever_down_aabb is not None
        and lever_down_aabb[0][2] < lever_rest_aabb[0][2] - 0.003,
        details=f"rest_bottom={lever_rest_aabb[0][2]}, down_bottom={lever_down_aabb[0][2]}",
    )

    # --- Overall faucet height ---
    ctx.check(
        "overall faucet height is roughly 0.10 m",
        body_aabb is not None and 0.090 <= body_aabb[1][2] <= 0.115,
        details=f"body top aabb={body_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
