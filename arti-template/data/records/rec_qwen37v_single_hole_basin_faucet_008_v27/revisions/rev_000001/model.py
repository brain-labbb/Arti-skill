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
# Single-hole basin faucet with side lever, ~0.13 m tall, mirror chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# The body leans BACK a few degrees (long axis tilts toward -X).
# ---------------------------------------------------------------------------

TILT = math.radians(5.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange.
FLANGE_R = 0.030
FLANGE_H = 0.006

# Main body barrel.
BODY_R = 0.025
BODY_S0 = 0.006
BODY_S1 = 0.072

# Decorative separation groove ring.
GROOVE_R = 0.0215
GROOVE_S0 = 0.070
GROOVE_S1 = 0.075

# Upper neck above the groove.
NECK_R = 0.023
NECK_S0 = 0.073
NECK_S1 = 0.105

# Fixed top dome cap.
CAP_S = 0.105
CAP_R = 0.023
CAP_H = 0.012

# Spout exit station on the body axis.
SPOUT_S = 0.048

# Lever pivot station on the body axis (above spout, on the +Y side).
LEVER_S = 0.065
LEVER_BOSS_R = 0.008
LEVER_BOSS_LEN = 0.010

# Lever arm dimensions.
LEVER_ARM_LEN = 0.058
LEVER_ARM_R = 0.005
LEVER_TIP_R = 0.008

# Lever pivot limits (radians).
LEVER_LOWER = -0.25  # slightly below horizontal
LEVER_UPPER = 0.70   # ~40 degrees above horizontal


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank, smooth downward bend, flat
    rectangular slot outlet. Built in spout-local frame whose origin sits on
    the body axis at SPOUT_S; the shank runs along local +X."""
    r_out = 0.015
    shank_x0 = 0.010  # seated ~10 mm inside the body casting
    shank_x1 = 0.038
    bend = 0.026
    end_x = shank_x1 + bend
    end_z = -bend

    # Swept path: straight then arc downward.
    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flat rectangular nozzle head at the spout end.
    # Wide in Y (horizontal), moderate in X, short in Z.
    nozzle_w = 0.038   # width in Y
    nozzle_d = 0.022   # depth in X
    nozzle_h = 0.014   # height in Z (extends downward from tube end)

    nozzle = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z))
        .rect(nozzle_d, nozzle_w)
        .extrude(-nozzle_h)
    )
    # Fillet the vertical edges of the nozzle for a smooth look.
    nozzle = nozzle.edges("|Z").fillet(0.003)

    spout = tube.union(nozzle)

    # Real hollow rectangular slot cut through the nozzle mouth.
    slot_w = 0.028   # slot width in Y
    slot_d = 0.006   # slot depth in X (thin slot)
    slot_cut_h = nozzle_h + 0.006  # cut through completely

    slot = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.002))
        .rect(slot_d, slot_w)
        .extrude(-slot_cut_h)
    )
    spout = spout.cut(slot)

    return spout


def _build_lever_shape() -> cq.Workplane:
    """Side lever: hub fits over the body boss, arm extends in +Y, handle
    tip at the end. Lever-local frame: origin at pivot center, arm along +Y.
    All geometry is contiguous along the +Y axis.

    Note: cq.Workplane("XZ") has normal = -Y, so extrude(-d) goes in +Y."""
    hub_len = 0.012

    # Hub: short cylinder along +Y that wraps the pivot boss.
    hub = (
        cq.Workplane("XZ", origin=(0.0, 0.0, 0.0))
        .circle(LEVER_BOSS_R * 0.85)
        .extrude(-hub_len)
    )

    # Arm: rod from hub end outward, slight overlap for clean union.
    arm_start = hub_len - 0.002
    arm = (
        cq.Workplane("XZ", origin=(0.0, arm_start, 0.0))
        .circle(LEVER_ARM_R)
        .extrude(-(LEVER_ARM_LEN + 0.002))
    )

    # Handle tip: larger rounded end for grip.
    tip_start = hub_len + LEVER_ARM_LEN - LEVER_TIP_R * 0.5
    tip = (
        cq.Workplane("XZ", origin=(0.0, tip_start, 0.0))
        .circle(LEVER_TIP_R)
        .extrude(-(LEVER_TIP_R * 1.5))
    )

    lever = hub.union(arm).union(tip)
    return lever


def _build_top_cap() -> cq.Workplane:
    """Fixed dome cap on top of the neck. Cap-local z=0 is the base."""
    base = cq.Workplane("XY").circle(CAP_R).extrude(CAP_H * 0.4)
    dome = (
        cq.Workplane("XY", origin=(0.0, 0.0, CAP_H * 0.4))
        .circle(CAP_R)
        .workplane(offset=CAP_H * 0.6)
        .circle(CAP_R * 0.3)
        .loft()
    )
    cap = base.union(dome)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_side_lever")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))

    # ---------------- body (root): flange + barrel + groove + neck + cap ---
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
    # Fixed dome cap on top of the neck.
    body.visual(
        mesh_from_cadquery(_build_top_cap(), "top_cap", tolerance=0.0003),
        origin=_tilted(CAP_S),
        material="chrome_brushed",
        name="top_cap",
    )
    # Pivot boss on the +Y side of the body for the lever axle mount.
    lever_axis_pt = _axis_point(LEVER_S)
    body.visual(
        Cylinder(radius=LEVER_BOSS_R, length=LEVER_BOSS_LEN),
        origin=Origin(
            xyz=(lever_axis_pt[0], BODY_R + LEVER_BOSS_LEN / 2.0 - 0.002, lever_axis_pt[2]),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="chrome",
        name="lever_pivot_boss",
    )

    # ---------------- spout (fixed): swept tube + rectangular slot outlet --
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_body",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ---------------- side lever (revolute on horizontal axle) -------------
    lever = model.part("side_lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_shape(), "side_lever", tolerance=0.0003),
        material="chrome",
        name="lever_arm",
    )
    # Joint: lever rotates around X axis (horizontal, front-to-back).
    # At q=0 the arm extends horizontally in +Y.
    # Positive q lifts the arm tip upward (+Z).
    # Hub sits 3 mm over the boss for a realistic captured fit.
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(
            xyz=(lever_axis_pt[0], BODY_R + LEVER_BOSS_LEN - 0.005, lever_axis_pt[2]),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=LEVER_LOWER, upper=LEVER_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    lever = object_model.get_part("side_lever")
    lever_joint = object_model.get_articulation("lever_pivot")

    # Intentional seated insertion: spout shank inside the solid body.
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_body",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated ~10 mm into the solid body casting.",
    )
    # Lever hub fits over the pivot boss on the body side.
    ctx.allow_overlap(
        body,
        lever,
        elem_a="lever_pivot_boss",
        elem_b="lever_arm",
        reason="Lever hub is intentionally represented as fitting over the pivot boss.",
    )

    # ---- body geometry: flange on deck, body leaning back ----
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )
    neck_aabb = ctx.part_element_world_aabb(body, elem="body_neck")
    ctx.check(
        "body leans back (neck center offset toward -X behind flange center)",
        neck_aabb is not None and (neck_aabb[0][0] + neck_aabb[1][0]) / 2.0 < -0.003,
        details=f"neck aabb={neck_aabb}",
    )

    # ---- overall height approximately 0.13 m ----
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "overall faucet height is about 0.12-0.14 m",
        body_aabb is not None and 0.11 <= body_aabb[1][2] <= 0.14,
        details=f"body aabb={body_aabb}",
    )

    # ---- spout: projects forward, curves down, has rectangular slot outlet ----
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.005,
        name="spout shank stays seated in the body",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward and curves downward",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.055
        and spout_aabb[0][2] < 0.030
        and spout_aabb[0][2] > 0.005,
        details=f"spout aabb={spout_aabb}",
    )
    # The rectangular slot outlet makes the spout wider in Y than the tube.
    ctx.check(
        "spout outlet is wider in Y than the tube diameter (rectangular slot head)",
        spout_aabb is not None
        and (spout_aabb[1][1] - spout_aabb[0][1]) > 0.034,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- side lever: mounted on +Y side, has non-fixed articulation ----
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "side lever extends to the +Y side of the body",
        lever_aabb is not None and lever_aabb[1][1] > 0.040,
        details=f"lever aabb={lever_aabb}",
    )
    ctx.check(
        "side lever pivot is above the deck",
        lever_aabb is not None and lever_aabb[0][2] > 0.040,
        details=f"lever aabb={lever_aabb}",
    )

    # ---- lever joint is revolute with correct limits ----
    ctx.check(
        "lever pivot is a revolute joint",
        lever_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={lever_joint.articulation_type}",
    )
    ll = lever_joint.motion_limits
    ctx.check(
        "lever pivot limits span below-horizontal to above-horizontal",
        ll is not None
        and ll.lower is not None
        and ll.upper is not None
        and ll.lower < 0.0
        and ll.upper > 0.3,
        details=f"limits lower={ll.lower}, upper={ll.upper}",
    )

    # ---- decisive pose: lever lifts upward when positive q applied ----
    lever_rest_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({lever_joint: LEVER_UPPER}):
        lever_raised_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "positive lever pivot raises the lever tip in +Z",
        lever_rest_aabb is not None
        and lever_raised_aabb is not None
        and lever_raised_aabb[1][2] > lever_rest_aabb[1][2] + 0.005,
        details=f"rest_max_z={lever_rest_aabb[1][2]}, raised_max_z={lever_raised_aabb[1][2]}",
    )

    # ---- lever stays connected to body at rest and raised pose ----
    ctx.expect_overlap(
        lever,
        body,
        axes="y",
        elem_a="lever_arm",
        elem_b="lever_pivot_boss",
        min_overlap=0.001,
        name="lever hub retains insertion over the pivot boss",
    )
    ctx.expect_within(
        lever,
        body,
        axes="xz",
        inner_elem="lever_arm",
        outer_elem="lever_pivot_boss",
        margin=0.004,
        name="lever hub stays centered on the pivot boss",
    )

    return ctx.report()


object_model = build_object_model()
