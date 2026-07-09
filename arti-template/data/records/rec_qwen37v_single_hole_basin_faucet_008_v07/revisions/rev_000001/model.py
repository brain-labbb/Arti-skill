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
# Single-hole basin faucet variant (v07), ~0.13 m tall, mirror chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# The body leans BACK a few degrees, i.e. its long axis tilts toward -X.
# ---------------------------------------------------------------------------

TILT = math.radians(6.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Oval base gasket sits under the flange.
GASKET_RX = 0.033
GASKET_RY = 0.028
GASKET_H = 0.003

# Main body barrel.
BODY_R = 0.025
BODY_S0 = 0.006
BODY_S1 = 0.0725

# Thin recessed separation groove ring around the upper third.
GROOVE_R = 0.0215
GROOVE_S0 = 0.0705
GROOVE_S1 = 0.0760

# Stepped-in upper neck above the groove.
NECK_R = 0.0228
NECK_S0 = 0.0740
NECK_S1 = 0.104

# Top dome cap (fixed, replaces push cap).
DOME_S = NECK_S1
DOME_R = NECK_R
DOME_H = 0.012

# Spout exit station on the body axis.
SPOUT_S = 0.050

# Side lever on a short horizontal axle.
LEVER_AXLE_S = 0.055
LEVER_HUB_R = 0.009
LEVER_HUB_LEN = 0.010
LEVER_HANDLE_LEN = 0.055
LEVER_HANDLE_W = 0.010
LEVER_HANDLE_H = 0.006

# Lever motion: rotates from 0 (closed/up) to about 90 deg (open/forward).
LEVER_LOWER = 0.0
LEVER_UPPER = math.radians(90.0)


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_tube() -> cq.Workplane:
    """Hollow chrome spout tube: straight shank + smooth downward bend.
    Built in spout-local frame whose origin sits on the body axis at SPOUT_S;
    the shank runs along local +X. Does NOT include the rectangular nozzle."""
    r_out = 0.015
    shank_x0 = 0.010  # seated ~10 mm inside the body casting
    shank_x1 = 0.035
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

    # Internal bore through the tube (hollow waterway).
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.002))
        .circle(0.009)
        .extrude(0.006)
    )
    return tube.cut(bore)


def _build_rect_nozzle() -> cq.Workplane:
    """Flat rectangular slot nozzle that attaches below the spout bend end.
    Built in the same spout-local frame as the tube."""
    shank_x1 = 0.035
    bend = 0.028
    end_x = shank_x1 + bend
    end_z = -bend

    slot_w = 0.024  # width (Y direction)
    slot_d = 0.008  # depth (X direction)
    nozzle_h = 0.012

    # Outer nozzle box (rect first arg = X direction, second = Y direction)
    outer = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z))
        .rect(slot_d, slot_w)
        .extrude(-nozzle_h)
    )

    # Real hollow rectangular slot cut through the nozzle (open bottom).
    inner = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - nozzle_h - 0.002))
        .rect(slot_d - 0.003, slot_w - 0.006)
        .extrude(nozzle_h + 0.006)
    )
    return outer.cut(inner)


def _build_lever_shape() -> cq.Workplane:
    """Side lever: a pivot hub (short thick cylinder along -Y, CadQuery XZ
    convention) + flat handle extending upward (+Z in local frame at rest).
    The handle base is embedded inside the hub for a connected mesh."""
    # Pivot hub: cylinder along -Y (CadQuery XZ plane extrudes along -Y)
    hub = (
        cq.Workplane("XZ")
        .circle(LEVER_HUB_R)
        .extrude(LEVER_HUB_LEN)
    )
    # Handle: flat bar centered in the hub Y-wise (y = -LEVER_HUB_LEN/2),
    # starting below z=0 so it's embedded inside the hub cylinder
    handle = (
        cq.Workplane("XY", origin=(0.0, -LEVER_HUB_LEN / 2.0, -0.005))
        .rect(LEVER_HANDLE_W, LEVER_HANDLE_H)
        .extrude(LEVER_HANDLE_LEN + 0.005)
    )
    return hub.union(handle)


def _build_gasket_shape() -> cq.Workplane:
    """Oval base gasket: a thin elliptical ring that sits under the flange."""
    outer = cq.Workplane("XY").ellipse(GASKET_RX, GASKET_RY).extrude(GASKET_H)
    inner = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .ellipse(GASKET_RX - 0.004, GASKET_RY - 0.004)
        .extrude(GASKET_H + 0.002)
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_v07")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("gasket_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("lever_chrome", rgba=(0.80, 0.82, 0.85, 1.0))

    # ---------------- body (root): gasket + flange + barrel + groove + neck --
    body = model.part("body")

    # Oval base gasket under the flange
    body.visual(
        mesh_from_cadquery(_build_gasket_shape(), "gasket", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, -GASKET_H / 2.0)),
        material="gasket_rubber",
        name="base_gasket",
    )
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
    # Fixed dome cap on top of the neck (replaces the push cap).
    body.visual(
        Cylinder(radius=DOME_R, length=DOME_H),
        origin=_tilted(DOME_S + DOME_H / 2.0),
        material="chrome_brushed",
        name="top_dome",
    )

    # ---------------- spout (fixed): tube + rectangular slot nozzle ---------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_tube(), "spout_tube", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    spout.visual(
        mesh_from_cadquery(_build_rect_nozzle(), "rect_nozzle", tolerance=0.0003),
        material="chrome",
        name="rect_nozzle",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ---------------- side lever (revolute on horizontal axle) ---------------
    lever = model.part("side_lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_shape(), "lever", tolerance=0.0003),
        material="lever_chrome",
        name="lever_handle",
    )

    # Axle on the body side (+Y), at mid-height station LEVER_AXLE_S.
    # The hub extends along -Y in local frame (CadQuery XZ extrude convention),
    # so we offset the joint origin outward by the hub length so the hub's
    # inner face sits just outside the body surface.
    # Handle extends along +Z at rest (closed/up), axis=(0,1,0) makes positive q
    # swing the handle toward +X (forward/open).
    axle_world = _axis_point(LEVER_AXLE_S)
    model.articulation(
        "lever_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(axle_world[0], BODY_R + LEVER_HUB_LEN - 0.002, axle_world[2])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=LEVER_LOWER,
            upper=LEVER_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    lever = object_model.get_part("side_lever")
    lever_joint = object_model.get_articulation("lever_rotate")

    # Intentional seated insertions (solid proxies, scoped per element).
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated ~10 mm into the solid body casting.",
    )
    ctx.allow_overlap(
        lever,
        body,
        elem_a="lever_handle",
        elem_b="body_barrel",
        reason="Lever hub is mounted against the body side surface for the axle pivot.",
    )

    # ---- hero geometry: gasket under flange, flange on deck, body leaning --
    gasket_aabb = ctx.part_element_world_aabb(body, elem="base_gasket")
    ctx.check(
        "oval base gasket exists and sits below the deck",
        gasket_aabb is not None and gasket_aabb[0][2] < -0.001,
        details=f"gasket aabb={gasket_aabb}",
    )
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

    # ---- gasket is oval (wider in one axis than the other) -----------------
    ctx.check(
        "base gasket is oval (X extent differs from Y extent)",
        gasket_aabb is not None
        and abs((gasket_aabb[1][0] - gasket_aabb[0][0]) - (gasket_aabb[1][1] - gasket_aabb[0][1])) > 0.003,
        details=f"gasket aabb={gasket_aabb}",
    )

    # ---- spout: projects forward, ends in rectangular slot ------------------
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.005,
        name="spout shank stays seated in the body",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward and droops to a low outlet above the deck",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.050
        and spout_aabb[0][2] < 0.025
        and spout_aabb[0][2] > 0.005,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- rectangular slot outlet: nozzle element is wider in Y than X ------
    nozzle_aabb = ctx.part_element_world_aabb(spout, elem="rect_nozzle")
    ctx.check(
        "rectangular slot nozzle is wider (Y) than deep (X) — flat slot shape",
        nozzle_aabb is not None
        and (nozzle_aabb[1][1] - nozzle_aabb[0][1]) > (nozzle_aabb[1][0] - nozzle_aabb[0][0]) * 1.2,
        details=f"nozzle aabb={nozzle_aabb}",
    )

    # ---- side lever: mounted on body side, rotates on horizontal axle ------
    lever_rest_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "side lever is mounted to the side of the body (positive Y offset)",
        lever_rest_aabb is not None
        and (lever_rest_aabb[0][1] + lever_rest_aabb[1][1]) / 2.0 > 0.015,
        details=f"lever rest aabb={lever_rest_aabb}",
    )

    # ---- articulation: lever joint is revolute with correct limits ---------
    lj = lever_joint.motion_limits
    ctx.check(
        "lever joint is revolute with 0 to 90 degree limits",
        lj is not None
        and lj.lower is not None
        and lj.upper is not None
        and abs(lj.lower) < 1e-9
        and abs(lj.upper - LEVER_UPPER) < 1e-6,
        details=f"limits={lj}",
    )

    # ---- decisive pose: lever swings from closed (up) to open (forward) ----
    with ctx.pose({lever_joint: LEVER_UPPER}):
        lever_open_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever rotation swings the handle from upward (rest) to forward (open)",
        lever_rest_aabb is not None
        and lever_open_aabb is not None
        # At rest the handle extends upward in Z; when open it extends in +X.
        # Rest: max Z should be higher than open max Z.
        and lever_rest_aabb[1][2] > lever_open_aabb[1][2] + 0.01
        # Open: max X should be further forward than rest max X.
        and lever_open_aabb[1][0] > lever_rest_aabb[1][0] + 0.01,
        details=f"rest={lever_rest_aabb}, open={lever_open_aabb}",
    )

    # ---- overall height check ----------------------------------------------
    dome_aabb = ctx.part_element_world_aabb(body, elem="top_dome")
    ctx.check(
        "overall faucet height is about 0.11-0.14 m",
        dome_aabb is not None and 0.10 <= dome_aabb[1][2] <= 0.14,
        details=f"dome aabb={dome_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
