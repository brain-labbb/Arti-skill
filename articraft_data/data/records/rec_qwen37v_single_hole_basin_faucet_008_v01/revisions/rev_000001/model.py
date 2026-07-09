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
# Single-hole basin faucet variant: tall straight tower with mixer lever.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# Body is VERTICAL (no tilt).
# ---------------------------------------------------------------------------

# Base flange
FLANGE_R = 0.030
FLANGE_H = 0.008

# Oval base gasket (ellipse under the flange)
GASKET_RX = 0.033  # semi-axis along X
GASKET_RY = 0.026  # semi-axis along Y
GASKET_H = 0.003

# Main body tower (straight vertical)
BODY_R = 0.020
BODY_H = 0.155  # tower height from top of flange
BODY_Z0 = FLANGE_H  # body starts at top of flange

# Decorative ring near top
RING_R = 0.023
RING_H = 0.005
RING_Z = BODY_Z0 + BODY_H - 0.025  # near top

# Lever pivot housing (small dome/cylinder at top of body)
PIVOT_R = 0.014
PIVOT_H = 0.012
PIVOT_Z = BODY_Z0 + BODY_H  # sits on top of body

# Spout exit point on body
SPOUT_Z = BODY_Z0 + BODY_H * 0.55  # about 55% up the body
SPOUT_R = 0.012

# Lever dimensions
LEVER_W = 0.010  # width
LEVER_H = 0.008  # height (thickness)
LEVER_L = 0.080  # length

# Articulation limits
LEVER_LIFT_MAX = math.radians(40.0)  # lever lifts up 40 degrees
LEVER_SWING_LIMIT = math.radians(45.0)  # swings ±45 degrees side to side


def _build_spout_shape() -> cq.Workplane:
    """Short forward-projecting spout with smooth downward curve and real
    hollow outlet at the mouth. Built in local frame: origin at the spout
    mounting point on the body surface, shank runs along +X."""
    r_out = SPOUT_R
    shank_x0 = -0.005  # starts slightly inside the body wall
    shank_x1 = 0.035   # straight section length
    bend_r = 0.020     # bend radius
    end_x = shank_x1 + bend_r
    end_z = -bend_r

    # Sweep path: straight then arc down
    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend_r, -bend_r), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet rim at the downward end
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.005))
        .circle(r_out + 0.001)
        .workplane(offset=-0.008)
        .circle(r_out + 0.005)
        .loft()
    )
    spout = tube.union(flare)

    # Hollow bore through the outlet mouth (real hollow outlet)
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.005))
        .circle(r_out - 0.002)
        .extrude(0.018)
    )
    return spout.cut(bore)


def _build_lever_shape() -> cq.Workplane:
    """Mixer lever handle: elongated bar with rounded tip.
    Local origin at the pivot point; lever extends along +X."""
    # Main bar centered on Y and Z
    bar = (
        cq.Workplane("XY", origin=(LEVER_L / 2.0, 0.0, 0.0))
        .box(LEVER_L, LEVER_W, LEVER_H)
    )
    # Rounded tip at the far end (half-cylinder cap)
    tip = (
        cq.Workplane("XZ", origin=(LEVER_L, 0.0, 0.0))
        .circle(LEVER_W / 2.0)
        .extrude(LEVER_H, both=True)
    )
    # Actually let's use a simpler approach: just a box with a cylinder tip
    tip_cyl = (
        cq.Workplane("XY", origin=(LEVER_L, 0.0, 0.0))
        .cylinder(LEVER_H, LEVER_W / 2.0)
    )
    lever = bar.union(tip_cyl)
    return lever


def _build_gasket_shape() -> cq.Workplane:
    """Oval base gasket: elliptical ring shape.
    Local origin at center, flat on XY plane."""
    outer = (
        cq.Workplane("XY")
        .ellipse(GASKET_RX, GASKET_RY)
        .extrude(GASKET_H)
    )
    inner = (
        cq.Workplane("XY")
        .ellipse(GASKET_RX - 0.005, GASKET_RY - 0.005)
        .extrude(GASKET_H)
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_tower_lever")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_satin", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))

    # ---------------- body (root): flange + tower + ring + pivot housing ----
    body = model.part("body")
    # Base flange
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    # Main tower
    body.visual(
        Cylinder(radius=BODY_R, length=BODY_H),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0 + BODY_H / 2.0)),
        material="chrome",
        name="body_tower",
    )
    # Decorative ring
    body.visual(
        Cylinder(radius=RING_R, length=RING_H),
        origin=Origin(xyz=(0.0, 0.0, RING_Z + RING_H / 2.0)),
        material="chrome_dark",
        name="deco_ring",
    )
    # Pivot housing dome at top
    body.visual(
        Cylinder(radius=PIVOT_R, length=PIVOT_H),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z + PIVOT_H / 2.0)),
        material="chrome_satin",
        name="pivot_housing",
    )

    # ---------------- base gasket (fixed, under flange) ---------------------
    gasket = model.part("base_gasket")
    gasket.visual(
        mesh_from_cadquery(_build_gasket_shape(), "base_gasket", tolerance=0.0003),
        material="rubber_black",
        name="gasket_ring",
    )
    model.articulation(
        "gasket_mount",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, -GASKET_H)),
    )

    # ---------------- spout (fixed): short forward curve + hollow outlet ----
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
        origin=Origin(xyz=(BODY_R - 0.002, 0.0, SPOUT_Z)),
    )

    # ---------------- lever pivot carrier (revolute swing around Z) ---------
    lever_carrier = model.part("lever_carrier")
    # Small invisible carrier - just a tiny pin visual for connectivity
    lever_carrier.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="chrome_dark",
        name="carrier_pin",
    )
    # Swing joint: rotates around vertical Z axis for temperature
    model.articulation(
        "lever_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever_carrier,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z + PIVOT_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0,
            lower=-LEVER_SWING_LIMIT, upper=LEVER_SWING_LIMIT,
        ),
    )

    # ---------------- lever handle (revolute lift around Y) -----------------
    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_shape(), "lever", tolerance=0.0003),
        material="chrome",
        name="lever_handle",
    )
    # Lift joint: rotates around horizontal Y axis to lift lever up for flow
    model.articulation(
        "lever_lift",
        ArticulationType.REVOLUTE,
        parent=lever_carrier,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0,
            lower=0.0, upper=LEVER_LIFT_MAX,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    gasket = object_model.get_part("base_gasket")
    spout = object_model.get_part("spout")
    lever_carrier = object_model.get_part("lever_carrier")
    lever = object_model.get_part("lever")
    swing = object_model.get_articulation("lever_swing")
    lift = object_model.get_articulation("lever_lift")

    # ---- Intentional overlaps (spout seated into body wall) ----------------
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_tower",
        reason="Spout shank is intentionally seated into the body tower wall.",
    )

    # ---- Hero geometry: tall straight tower --------------------------------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )
    tower_aabb = ctx.part_element_world_aabb(body, elem="body_tower")
    ctx.check(
        "body tower is straight vertical (centered over flange)",
        tower_aabb is not None
        and abs((tower_aabb[0][0] + tower_aabb[1][0]) / 2.0) < 0.003,
        details=f"tower aabb={tower_aabb}",
    )
    # Taller than the parent tap (~0.13m): this variant is ~0.18-0.22m total
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "overall faucet height is taller than parent (~0.18-0.22 m)",
        lever_aabb is not None and 0.170 <= lever_aabb[1][2] <= 0.240,
        details=f"lever aabb={lever_aabb}",
    )

    # ---- Oval base gasket present and under the flange ---------------------
    gasket_aabb = ctx.part_world_aabb(gasket)
    ctx.check(
        "oval base gasket sits below the deck surface",
        gasket_aabb is not None and gasket_aabb[0][2] < 0.001,
        details=f"gasket aabb={gasket_aabb}",
    )
    ctx.check(
        "gasket is oval (wider along X than Y or vice versa)",
        gasket_aabb is not None
        and abs((gasket_aabb[1][0] - gasket_aabb[0][0]) - (gasket_aabb[1][1] - gasket_aabb[0][1])) > 0.005,
        details=f"gasket aabb={gasket_aabb}",
    )
    ctx.expect_gap(
        body,
        gasket,
        axis="z",
        min_gap=-0.001,
        max_gap=0.005,
        name="gasket is directly under the flange",
    )

    # ---- Spout: short forward projection with hollow outlet ----------------
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout projects forward from body and curves down",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.040  # extends forward
        and spout_aabb[0][2] < SPOUT_Z - 0.010  # curves below exit point
        and spout_aabb[0][2] > 0.020,  # stays above deck
        details=f"spout aabb={spout_aabb}",
    )
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.003,
        name="spout shank seated in body wall",
    )

    # ---- Lever: mounted at top, has two revolute joints --------------------
    ctx.expect_gap(
        lever,
        body,
        axis="z",
        min_gap=-0.005,
        max_gap=0.020,
        name="lever is at top of the faucet body",
    )

    # ---- Articulation limits -----------------------------------------------
    sl = swing.motion_limits
    ctx.check(
        "lever swing limits are ±45 degrees",
        sl is not None
        and sl.lower is not None
        and sl.upper is not None
        and abs(sl.lower + LEVER_SWING_LIMIT) < 1e-6
        and abs(sl.upper - LEVER_SWING_LIMIT) < 1e-6,
        details=f"limits={sl}",
    )
    ll = lift.motion_limits
    ctx.check(
        "lever lift limits are 0 to 40 degrees",
        ll is not None
        and ll.lower is not None
        and ll.upper is not None
        and abs(ll.lower) < 1e-9
        and abs(ll.upper - LEVER_LIFT_MAX) < 1e-6,
        details=f"limits={ll}",
    )

    # ---- Decisive pose: lever lifts up ------------------------------------
    lever_rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_handle")
    with ctx.pose({lift: LEVER_LIFT_MAX}):
        lever_lifted_aabb = ctx.part_element_world_aabb(lever, elem="lever_handle")
    ctx.check(
        "lifting the lever raises its tip upward",
        lever_rest_aabb is not None
        and lever_lifted_aabb is not None
        and lever_lifted_aabb[1][2] > lever_rest_aabb[1][2] + 0.010,
        details=f"rest_aabb={lever_rest_aabb}, lifted_aabb={lever_lifted_aabb}",
    )

    # ---- Decisive pose: lever swings side to side --------------------------
    lever_center_aabb = ctx.part_element_world_aabb(lever, elem="lever_handle")
    with ctx.pose({swing: LEVER_SWING_LIMIT}):
        lever_swung_aabb = ctx.part_element_world_aabb(lever, elem="lever_handle")
    ctx.check(
        "swinging the lever moves it sideways (Y axis)",
        lever_center_aabb is not None
        and lever_swung_aabb is not None
        and abs((lever_swung_aabb[0][1] + lever_swung_aabb[1][1]) / 2.0
                - (lever_center_aabb[0][1] + lever_center_aabb[1][1]) / 2.0) > 0.010,
        details=f"center_aabb={lever_center_aabb}, swung_aabb={lever_swung_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
