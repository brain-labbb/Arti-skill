from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet variant: tall straight tower with short forward
# spout and side lever. ~0.22 m tall, mirror chrome finish.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# ---------------------------------------------------------------------------

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Main tower body (straight vertical).
TOWER_R = 0.020
TOWER_H = 0.195
TOWER_Z0 = FLANGE_H
TOWER_Z1 = TOWER_Z0 + TOWER_H

# Decorative cap ring at tower top.
CAP_RING_R = 0.022
CAP_RING_H = 0.005

# Spout exit: projects forward from upper portion of tower.
SPOUT_EXIT_Z = TOWER_Z1 - 0.035  # ~35mm from the top

# Axle boss on the body side for the lever.
AXLE_BOSS_R = 0.010
AXLE_BOSS_LEN = 0.012
AXLE_Z = TOWER_Z0 + TOWER_H * 0.55  # slightly above mid-height

# Side lever handle.
LEVER_ARM_LEN = 0.075
LEVER_ARM_W = 0.012
LEVER_ARM_H = 0.008
LEVER_TIP_R = 0.007  # rounded grip end

LEVER_LOWER = math.radians(-30.0)
LEVER_UPPER = math.radians(40.0)


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: short forward projection with smooth downward
    curve, ending in a real hollow outlet with bore cut through.
    Built in spout-local frame: origin at body wall, shank along +X."""
    r_out = 0.013
    shank_x0 = -0.005  # seated inside the body wall
    shank_x1 = 0.035
    bend = 0.020  # bend radius
    end_x = shank_x1 + bend
    end_z = -bend

    # Swept tube path: straight then curved down.
    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet skirt around the down-turned end.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.005))
        .circle(r_out - 0.001)
        .workplane(offset=-0.008)
        .circle(0.016)
        .loft()
    )
    spout = tube.union(flare)

    # Real hollow outlet: bore cut through the spout end (open mouth).
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.008))
        .circle(0.012)
        .workplane(offset=0.022)
        .circle(0.008)
        .loft()
    )
    spout = spout.cut(bore)

    # Internal channel bore through the straight shank for water passage.
    channel = (
        cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0))
        .circle(0.007)
        .extrude(shank_x1 - shank_x0 + bend * 0.5)
    )
    spout = spout.cut(channel)

    return spout


def _build_lever_shape() -> cq.Workplane:
    """Side lever handle: straight arm with rounded grip end.
    Built in lever-local frame: origin at axle center, arm extends along +X.
    The Y-axis axle will swing this arm up/down in the XZ plane.
    All Y extents stay in +Y (outward from body) to avoid tower penetration."""
    # Main arm bar extending along +X, centered at Y=+0.003 (slightly outward).
    arm = cq.Workplane("XY").box(LEVER_ARM_LEN, LEVER_ARM_W, LEVER_ARM_H)
    arm = arm.translate((LEVER_ARM_LEN / 2.0, LEVER_ARM_W * 0.25, 0.0))

    # Rounded grip tip at the end of the arm.
    tip = (
        cq.Workplane("XZ", origin=(LEVER_ARM_LEN, LEVER_ARM_W * 0.25, 0.0))
        .circle(LEVER_TIP_R)
        .extrude(LEVER_ARM_W)
        .translate((0.0, -LEVER_ARM_W / 2.0, 0.0))
    )
    lever = arm.union(tip)

    # Collar ring at the base wrapping around the axle boss.
    collar = (
        cq.Workplane("XZ")
        .circle(AXLE_BOSS_R + 0.002)
        .extrude(AXLE_BOSS_LEN * 0.4)
        .translate((0.0, 0.0, 0.0))
    )
    lever = lever.union(collar)
    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_tower")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))

    # ---------------- body (root): flange + tower + cap ring + bosses -------
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=TOWER_R, length=TOWER_H),
        origin=Origin(xyz=(0.0, 0.0, (TOWER_Z0 + TOWER_Z1) / 2.0)),
        material="chrome",
        name="tower_column",
    )
    body.visual(
        Cylinder(radius=CAP_RING_R, length=CAP_RING_H),
        origin=Origin(xyz=(0.0, 0.0, TOWER_Z1 - CAP_RING_H / 2.0)),
        material="chrome_dark",
        name="cap_ring",
    )
    # Axle boss protruding from body side (+Y) for lever mount.
    body.visual(
        Cylinder(radius=AXLE_BOSS_R, length=AXLE_BOSS_LEN),
        origin=Origin(
            xyz=(0.0, TOWER_R + AXLE_BOSS_LEN / 2.0, AXLE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="chrome",
        name="axle_boss",
    )

    # ---------------- spout (fixed): hollow tube with open outlet ----------
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
        origin=Origin(xyz=(TOWER_R, 0.0, SPOUT_EXIT_Z)),
    )

    # ---------------- lever (revolute on horizontal axle) ------------------
    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_shape(), "lever", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="chrome",
        name="lever_arm",
    )

    # Joint frame: at the outer face of the axle boss, horizontal axle.
    # axis=(0, -1, 0): right-hand rule around -Y means positive q lifts the
    # arm tip upward (+Z), negative q drops it down. Matches faucet convention.
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, TOWER_R + AXLE_BOSS_LEN, AXLE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=LEVER_LOWER, upper=LEVER_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    lever = object_model.get_part("lever")
    pivot = object_model.get_articulation("lever_pivot")

    # Intentional seated insertions (scoped per element).
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="tower_column",
        reason="Spout shank is intentionally seated into the solid tower wall.",
    )
    ctx.allow_overlap(
        lever,
        body,
        elem_a="lever_arm",
        elem_b="axle_boss",
        reason="Lever arm base wraps around the axle boss for pivot mount.",
    )

    # ---- Hero geometry: straight tall tower on deck flange -----------------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )
    tower_aabb = ctx.part_element_world_aabb(body, elem="tower_column")
    ctx.check(
        "tower is straight vertical (centered over flange)",
        tower_aabb is not None
        and abs((tower_aabb[0][0] + tower_aabb[1][0]) / 2.0) < 0.003
        and abs((tower_aabb[0][1] + tower_aabb[1][1]) / 2.0) < 0.003,
        details=f"tower aabb={tower_aabb}",
    )
    ctx.check(
        "tower is tall (~0.18m body, total faucet >0.20m)",
        tower_aabb is not None
        and (tower_aabb[1][2] - tower_aabb[0][2]) > 0.16,
        details=f"tower aabb={tower_aabb}",
    )

    # ---- Spout: projects forward with real hollow outlet -------------------
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout projects forward from the tower",
        spout_aabb is not None and spout_aabb[1][0] > 0.040,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout outlet droops below spout exit height",
        spout_aabb is not None
        and spout_aabb[0][2] < SPOUT_EXIT_Z - 0.005,
        details=f"spout aabb={spout_aabb}",
    )

    # Verify the spout has a real hollow outlet: the spout's bottom face should
    # extend lower than a solid spout would (the flare+opening reaches down).
    spout_center_z = (spout_aabb[0][2] + spout_aabb[1][2]) / 2.0 if spout_aabb else None
    ctx.check(
        "spout has extended outlet rim (flared open end)",
        spout_aabb is not None
        and (spout_aabb[1][2] - spout_aabb[0][2]) > 0.025,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- Lever: side-mounted, articulates on horizontal axle ---------------
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever is mounted on the side (+Y) of the body",
        lever_aabb is not None
        and (lever_aabb[0][1] + lever_aabb[1][1]) / 2.0 > 0.015,
        details=f"lever aabb={lever_aabb}",
    )

    # Verify lever pivot axis is horizontal (Y axis).
    pl = pivot.motion_limits
    ctx.check(
        "lever pivot limits are -30 to +40 degrees",
        pl is not None
        and pl.lower is not None
        and pl.upper is not None
        and abs(pl.lower - LEVER_LOWER) < 1e-6
        and abs(pl.upper - LEVER_UPPER) < 1e-6,
        details=f"limits={pl}",
    )

    # Decisive pose: lever movement changes height of the lever handle.
    # Use AABB since part origin is at the axle center (doesn't move).
    rest_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({pivot: LEVER_UPPER}):
        raised_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({pivot: LEVER_LOWER}):
        lowered_aabb = ctx.part_world_aabb(lever)

    ctx.check(
        "lever pivot moves the handle up at positive angle",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.005,
        details=f"rest_top={rest_aabb[1][2]}, raised_top={raised_aabb[1][2]}",
    )
    ctx.check(
        "lever pivot moves the handle down at negative angle",
        rest_aabb is not None
        and lowered_aabb is not None
        and lowered_aabb[0][2] < rest_aabb[0][2] - 0.005,
        details=f"rest_bot={rest_aabb[0][2]}, lowered_bot={lowered_aabb[0][2]}",
    )

    # Verify the lever stays connected to the body across its range.
    with ctx.pose({pivot: LEVER_UPPER}):
        ctx.expect_overlap(
            lever,
            body,
            axes="y",
            min_overlap=0.002,
            name="lever stays connected to axle boss at max raise",
        )

    # Overall faucet height check.
    cap_aabb = ctx.part_element_world_aabb(body, elem="cap_ring")
    ctx.check(
        "overall faucet height is about 0.22m (tall tower variant)",
        cap_aabb is not None and 0.19 <= cap_aabb[1][2] <= 0.24,
        details=f"cap_ring aabb={cap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
