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
# Single-hole basin faucet variant: waterfall spout lip, top lever with
# revolute lift (flow) and swing (temperature), circumferential grip
# grooves, two rear screw caps.  ~0.13 m tall, mirror chrome.
# World: +Z up, deck at z=0, spout toward +X (front).
# Body tilts back ~6° (long axis toward -X).
# ---------------------------------------------------------------------------

TILT = math.radians(6.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange
FLANGE_R = 0.030
FLANGE_H = 0.006

# Main body barrel
BODY_R = 0.025
BODY_S0 = 0.006
BODY_S1 = 0.0725

# Separation groove ring (upper third)
GROOVE_R = 0.0215
GROOVE_S0 = 0.0705
GROOVE_S1 = 0.0760

# Upper neck above the groove
NECK_R = 0.0228
NECK_S0 = 0.0740
NECK_S1 = 0.104

# Spout exit station on the body axis
SPOUT_S = 0.050

# Lever cartridge (temperature pivot on top of neck)
CARTRIDGE_R = 0.016
CARTRIDGE_H = 0.014

# Lever handle
LEVER_LEN = 0.082
LEVER_R = 0.006
GRIP_R = 0.0075
GRIP_START = 0.028
GRIP_GROOVES = 6

# Rear screw caps
SCREW_R = 0.004
SCREW_H = 0.002

# Joint limits
SWING_LIMIT = math.radians(45.0)   # ±45° temperature
LIFT_LIMIT = math.radians(35.0)    # 0–35° flow control


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position on the tilted body axis at station *s*."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station *s*, Z along the tilted axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


# ────────────────────────── CadQuery builders ──────────────────────────


def _build_waterfall_spout() -> cq.Workplane:
    """Waterfall spout: wide flat channel sweeping forward and down,
    ending in a wider rounded lip."""
    w = 0.048          # channel width (Y) — wide for waterfall
    h = 0.012          # channel thickness (Z) — thin / flat

    # Sweep path: straight from inside body, then gentle downward curve.
    x0 = 0.008         # seated ~8 mm inside the body casting
    x1 = 0.040         # end of straight section
    bend_r = 0.018     # bend radius (gentle)

    path = (
        cq.Workplane("XZ")
        .moveTo(x0, 0.0)
        .lineTo(x1, 0.0)
        .tangentArcPoint((bend_r, -bend_r), relative=True)
    )

    # Oval profile at the path start (wide and flat for waterfall look).
    profile = (
        cq.Workplane("YZ", origin=(x0, 0.0, 0.0))
        .ellipse(w / 2, h / 2)
    )
    channel = profile.sweep(path)

    # Waterfall lip: distinctly wider rounded plate at the spout end.
    end_x = x1 + bend_r          # 0.058
    end_z = -bend_r              # -0.018
    lip_w = w + 0.014            # ~62 mm — wide waterfall edge
    lip_h = h + 0.004            # slightly thicker
    lip = (
        cq.Workplane("XY", origin=(end_x - 0.002, 0.0, end_z - 0.002))
        .ellipse(lip_w / 2, lip_h / 2)
        .extrude(0.010)
    )
    return channel.union(lip)


def _build_lever() -> cq.Workplane:
    """Lever handle with thicker grip section and circumferential grooves.
    Local origin at the pivot; handle extends along +X."""
    # Main shaft
    shaft = cq.Workplane("YZ").circle(LEVER_R).extrude(LEVER_LEN)

    # Grip section (slightly wider)
    grip_len = LEVER_LEN - GRIP_START
    grip = (
        cq.Workplane("YZ", origin=(GRIP_START, 0.0, 0.0))
        .circle(GRIP_R)
        .extrude(grip_len)
    )
    lever = shaft.union(grip)

    # Base collar (pivot housing)
    collar = (
        cq.Workplane("YZ", origin=(-0.004, 0.0, 0.0))
        .circle(LEVER_R + 0.003)
        .extrude(0.008)
    )
    lever = lever.union(collar)

    # Circumferential grip grooves
    groove_w = 0.0018
    groove_d = 0.0012
    spacing = grip_len / (GRIP_GROOVES + 1)
    for i in range(GRIP_GROOVES):
        gx = GRIP_START + spacing * (i + 1)
        ring = (
            cq.Workplane("YZ", origin=(gx - groove_w / 2, 0.0, 0.0))
            .circle(GRIP_R + 0.001)
            .circle(GRIP_R - groove_d)
            .extrude(groove_w)
        )
        lever = lever.cut(ring)

    return lever


# ────────────────────────── Object model ──────────────────────────


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("screw_cap", rgba=(0.55, 0.57, 0.60, 1.0))

    # ── body (root): flange + barrel + groove + neck + rear screw caps ──
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=BODY_R, length=BODY_S1 - BODY_S0),
        origin=_tilted((BODY_S0 + BODY_S1) / 2),
        material="chrome",
        name="body_barrel",
    )
    body.visual(
        Cylinder(radius=GROOVE_R, length=GROOVE_S1 - GROOVE_S0),
        origin=_tilted((GROOVE_S0 + GROOVE_S1) / 2),
        material="chrome_dark",
        name="groove_ring",
    )
    body.visual(
        Cylinder(radius=NECK_R, length=NECK_S1 - NECK_S0),
        origin=_tilted((NECK_S0 + NECK_S1) / 2),
        material="chrome",
        name="body_neck",
    )

    # Two small screw caps on the back (-X side) of the body.
    for i, s in enumerate((0.032, 0.055)):
        ax, _, az = _axis_point(s)
        nx, nz = -COS_T, -SIN_T  # outward-backward normal
        cx = ax + (BODY_R + SCREW_H / 2) * nx
        cz = az + (BODY_R + SCREW_H / 2) * nz
        body.visual(
            Cylinder(radius=SCREW_R, length=SCREW_H),
            origin=Origin(
                xyz=(cx, 0.0, cz),
                rpy=(0.0, -(math.pi / 2 + TILT), 0.0),
            ),
            material="screw_cap",
            name=f"screw_cap_{i}",
        )

    # ── spout (fixed): waterfall-style wide flat channel ──
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(
            _build_waterfall_spout(), "waterfall_spout", tolerance=0.0003
        ),
        material="chrome",
        name="spout_channel",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ── lever cartridge (temperature swing about body axis) ──
    cartridge = model.part("lever_cartridge")
    cartridge.visual(
        Cylinder(radius=CARTRIDGE_R, length=CARTRIDGE_H),
        origin=Origin(xyz=(0.0, 0.0, CARTRIDGE_H / 2)),
        material="chrome",
        name="cartridge_body",
    )
    cartridge.visual(
        Cylinder(radius=CARTRIDGE_R - 0.003, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, CARTRIDGE_H + 0.0025)),
        material="chrome_brushed",
        name="cartridge_dome",
    )

    # Joint frame at the neck top; tilted so +Z aligns with body axis.
    # Positive q swings the lever side-to-side (temperature).
    model.articulation(
        "lever_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cartridge,
        origin=Origin(xyz=_axis_point(NECK_S1), rpy=(0.0, -TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-SWING_LIMIT,
            upper=SWING_LIMIT,
        ),
    )

    # ── lever handle (flow-control lift) ──
    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_build_lever(), "lever_handle", tolerance=0.0003),
        material="chrome",
        name="lever_shaft",
    )

    # Pivot at the top of the cartridge dome; axis -Y so positive q
    # lifts the +X handle end upward (opens flow).
    model.articulation(
        "lever_lift",
        ArticulationType.REVOLUTE,
        parent=cartridge,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, CARTRIDGE_H + 0.003)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=LIFT_LIMIT,
        ),
    )

    return model


# ────────────────────────── Tests ──────────────────────────


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    cartridge = object_model.get_part("lever_cartridge")
    lever = object_model.get_part("lever")
    swing = object_model.get_articulation("lever_swing")
    lift = object_model.get_articulation("lever_lift")

    # ── Intentional seated / nested overlaps ──
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_channel",
        elem_b="body_barrel",
        reason="Spout shank is seated ~8 mm inside the solid body casting.",
    )
    ctx.allow_overlap(
        lever,
        cartridge,
        elem_a="lever_shaft",
        elem_b="cartridge_body",
        reason="Lever shaft passes through the cartridge housing at the pivot.",
    )
    ctx.allow_overlap(
        lever,
        cartridge,
        elem_a="lever_shaft",
        elem_b="cartridge_dome",
        reason="Lever collar seats onto the cartridge dome pivot cap.",
    )

    # ── Body: flange on deck, leaning back ──
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.001,
        details=f"flange_aabb={flange_aabb}",
    )
    neck_aabb = ctx.part_element_world_aabb(body, elem="body_neck")
    ctx.check(
        "body leans back (neck center behind flange center)",
        neck_aabb is not None
        and (neck_aabb[0][0] + neck_aabb[1][0]) / 2 < -0.005,
        details=f"neck_aabb={neck_aabb}",
    )

    # ── Screw caps on back of body ──
    for i in range(2):
        v = body.get_visual(f"screw_cap_{i}")
        ctx.check(
            f"screw_cap_{i} exists on body rear",
            v is not None,
            details=f"missing screw_cap_{i}",
        )

    # ── Waterfall spout: wide flat channel shape ──
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "waterfall spout is wider (Y) than tall (Z) — flat channel shape",
        spout_aabb is not None
        and (spout_aabb[1][1] - spout_aabb[0][1])
        > (spout_aabb[1][2] - spout_aabb[0][2]) * 1.3,
        details=f"spout_aabb={spout_aabb}",
    )
    ctx.check(
        "spout projects forward from the body",
        spout_aabb is not None and spout_aabb[1][0] > 0.055,
        details=f"spout_aabb={spout_aabb}",
    )

    # ── Lever extends forward from the cartridge ──
    lever_aabb = ctx.part_world_aabb(lever)
    cart_aabb = ctx.part_world_aabb(cartridge)
    ctx.check(
        "lever handle extends forward beyond the cartridge",
        lever_aabb is not None
        and cart_aabb is not None
        and lever_aabb[1][0] > cart_aabb[1][0] + 0.04,
        details=f"lever_aabb={lever_aabb}, cart_aabb={cart_aabb}",
    )

    # ── Cartridge sits at the neck top ──
    ctx.expect_gap(
        cartridge,
        body,
        axis="z",
        min_gap=-0.005,
        max_gap=0.005,
        name="cartridge seats at the top of the neck",
    )

    # ── Joint limits ──
    swl = swing.motion_limits
    ctx.check(
        "lever swing limits ±45° for temperature",
        swl is not None
        and swl.lower is not None
        and swl.upper is not None
        and abs(swl.lower + SWING_LIMIT) < 1e-6
        and abs(swl.upper - SWING_LIMIT) < 1e-6,
        details=f"limits={swl}",
    )
    lfl = lift.motion_limits
    ctx.check(
        "lever lift limits 0–35° for flow control",
        lfl is not None
        and lfl.lower is not None
        and lfl.upper is not None
        and abs(lfl.lower) < 1e-6
        and abs(lfl.upper - LIFT_LIMIT) < 1e-6,
        details=f"limits={lfl}",
    )

    # ── Decisive poses ──

    # Lift: positive q raises the lever tip upward (flow on).
    # The lever origin sits on the pivot axis, so use AABB max-Z to
    # detect the tip rising.
    rest_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({lift: LIFT_LIMIT}):
        raised_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever lift raises the handle tip upward",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.005,
        details=f"rest_aabb={rest_aabb}, raised_aabb={raised_aabb}",
    )

    # Swing: positive / negative q moves the lever tip side-to-side.
    with ctx.pose({swing: SWING_LIMIT}):
        left_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({swing: -SWING_LIMIT}):
        right_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever swing moves handle side-to-side (temperature)",
        left_aabb is not None
        and right_aabb is not None
        and (
            abs(left_aabb[1][1] - right_aabb[1][1]) > 0.015
            or abs(left_aabb[0][1] - right_aabb[0][1]) > 0.015
        ),
        details=f"left_aabb={left_aabb}, right_aabb={right_aabb}",
    )

    # ── Overall height ──
    ctx.check(
        "faucet height is about 0.12–0.15 m",
        lever_aabb is not None and 0.11 <= lever_aabb[1][2] <= 0.16,
        details=f"lever_aabb={lever_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
