from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Sphere,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet variant: tapered conical body, side lever on
# horizontal axle, grip grooves, two rear screw caps.  ~0.13 m tall, chrome.
# World frame: +Z up, deck at z = 0, spout/beak points toward +X (front).
# ---------------------------------------------------------------------------

# Base flange on the deck.
FLANGE_R = 0.030
FLANGE_H = 0.005

# Tapered conical body: wider at bottom, narrower at top.
BODY_R_BOTTOM = 0.026
BODY_R_TOP = 0.016
BODY_Z0 = 0.005  # body starts just above flange
BODY_Z1 = 0.105  # body top
BODY_H = BODY_Z1 - BODY_Z0

# Forward beak / short spout projecting from upper body.
BEAK_ORIGIN_Z = 0.085  # where the beak exits the body
BEAK_LEN = 0.060  # how far it projects forward (+X)
BEAK_R = 0.012  # spout tube radius
BEAK_DROP = 0.025  # downward curve at the tip

# Lever pivot station (height, on right side of body).
LEVER_Z = 0.065
LEVER_PIVOT_Y = 0.0  # centered at body side in Y; offset computed from body radius at that Z
LEVER_LEN = 0.055  # lever arm length
LEVER_W = 0.012  # lever width
LEVER_H = 0.008  # lever thickness
LEVER_AXLE_R = 0.005
LEVER_AXLE_LEN = 0.012

# Lever swing limits (revolute around Y-axis for up/down flow control).
LEVER_LOWER = math.radians(-15.0)   # slightly below horizontal = off
LEVER_UPPER = math.radians(45.0)    # raised = on

# Grip grooves: small indentations along the lever handle.
GRIP_COUNT = 5
GRIP_DEPTH = 0.0015
GRIP_SPACING = 0.008

# Two rear screw caps on the back of the body.
SCREW_CAP_R = 0.004
SCREW_CAP_H = 0.003
SCREW_CAP_Z = [0.040, 0.060]  # heights of the two screw caps


def _body_radius_at(z: float) -> float:
    """Linearly interpolate body radius at height z."""
    t = (z - BODY_Z0) / BODY_H
    return BODY_R_BOTTOM + t * (BODY_R_TOP - BODY_R_BOTTOM)


def _build_body_shape() -> cq.Workplane:
    """Tapered conical body with rounded top edge, built via CadQuery loft."""
    # Outer tapered cone from bottom to top.
    body = (
        cq.Workplane("XY", origin=(0.0, 0.0, BODY_Z0))
        .circle(BODY_R_BOTTOM)
        .workplane(offset=BODY_H)
        .circle(BODY_R_TOP)
        .loft()
    )
    # Dome-like rounded cap on top.
    top_cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, BODY_Z1))
        .circle(BODY_R_TOP)
        .extrude(0.004)
    )
    top_dome = (
        cq.Workplane("XY", origin=(0.0, 0.0, BODY_Z1 + 0.004))
        .sphere(BODY_R_TOP)
    )
    # Cut bottom half of sphere to make a dome.
    cutter = (
        cq.Workplane("XY", origin=(0.0, 0.0, BODY_Z1 + 0.004 - BODY_R_TOP))
        .box(BODY_R_TOP * 3, BODY_R_TOP * 3, BODY_R_TOP * 2)
    )
    dome_top = top_dome.cut(cutter)
    body = body.union(top_cap).union(dome_top)
    return body


def _build_beak_shape() -> cq.Workplane:
    """Forward beak / short spout: straight section then curves down, with
    flared open outlet. Built in local frame: origin at body exit, +X forward."""
    # Straight horizontal tube section.
    straight_x = BEAK_LEN * 0.55
    bend_r = 0.020

    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(straight_x, 0.0)
        .tangentArcPoint((bend_r, -bend_r), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(0.0, 0.0, 0.0)).circle(BEAK_R).sweep(path)

    end_x = straight_x + bend_r
    end_z = -bend_r

    # Flared outlet rim at the downturn end.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.005))
        .circle(BEAK_R * 0.98)
        .workplane(offset=-0.008)
        .circle(BEAK_R * 1.35)
        .loft()
    )
    beak = tube.union(flare)

    # Hollow bore through the outlet.
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.005))
        .circle(BEAK_R * 1.05)
        .workplane(offset=0.016)
        .circle(BEAK_R * 0.70)
        .loft()
    )
    return beak.cut(bore)


def _build_lever_shape() -> cq.Workplane:
    """Side lever arm with grip grooves. Origin at pivot axle center.
    Lever extends along local +Y (outward from body side).
    Axle runs along local X (front-back through the body).
    Grip grooves cut on the upper face of the arm."""
    # Main lever arm extending along +Y from pivot.
    arm = (
        cq.Workplane("XZ", origin=(0.0, LEVER_LEN / 2.0, 0.0))
        .rect(LEVER_W, LEVER_H)
        .extrude(LEVER_LEN / 2.0)
    )
    # Rebuild as box centered on +Y.
    arm = (
        cq.Workplane("XY")
        .center(0.0, LEVER_LEN / 2.0)
        .rect(LEVER_W, LEVER_LEN)
        .extrude(LEVER_H)
    )
    # Rounded tip at the far end.
    tip = (
        cq.Workplane("XY", origin=(0.0, LEVER_LEN, LEVER_H / 2.0))
        .sphere(LEVER_W / 2.0)
    )
    lever = arm.union(tip)

    # Axle boss at the pivot end (short cylinder along X).
    boss = (
        cq.Workplane("YZ", origin=(-LEVER_AXLE_LEN / 2.0, 0.0, LEVER_H / 2.0))
        .circle(LEVER_AXLE_R * 1.5)
        .extrude(LEVER_AXLE_LEN)
    )
    lever = lever.union(boss)

    # Grip grooves: small cuts along the top face of the arm.
    for i in range(GRIP_COUNT):
        gy = 0.012 + i * GRIP_SPACING
        groove = (
            cq.Workplane("XY", origin=(0.0, gy, LEVER_H - GRIP_DEPTH + 0.0002))
            .box(LEVER_W * 0.85, 0.003, GRIP_DEPTH * 2.5)
        )
        lever = lever.cut(groove)

    # Shift so axle center is at local origin (Z centered on lever thickness).
    lever = lever.translate((0.0, 0.0, -LEVER_H / 2.0))

    return lever


def _build_screw_cap_shape() -> cq.Workplane:
    """Small screw cap: low-profile cylinder with a cross-slot on top."""
    cap = (
        cq.Workplane("XY")
        .circle(SCREW_CAP_R)
        .extrude(SCREW_CAP_H)
    )
    # Cross slot on top face for visual detail.
    slot1 = (
        cq.Workplane("XY", origin=(0.0, 0.0, SCREW_CAP_H - 0.0005))
        .box(SCREW_CAP_R * 1.6, 0.001, 0.002)
    )
    slot2 = (
        cq.Workplane("XY", origin=(0.0, 0.0, SCREW_CAP_H - 0.0005))
        .box(0.001, SCREW_CAP_R * 1.6, 0.002)
    )
    cap = cap.cut(slot1).cut(slot2)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_variant")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("groove_fill", rgba=(0.35, 0.36, 0.38, 1.0))

    # ===================== body (root) =====================================
    body = model.part("body")

    # Base flange on the deck.
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )

    # Tapered conical body.
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell", tolerance=0.0003),
        material="chrome",
        name="body_cone",
    )

    # ===================== beak/spout (fixed to body) ======================
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_beak_shape(), "beak_spout", tolerance=0.0003),
        material="chrome",
        name="beak_tube",
    )
    # Fixed mount at the front of the body where the beak exits.
    body_r_at_beak = _body_radius_at(BEAK_ORIGIN_Z)
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=(body_r_at_beak * 0.7, 0.0, BEAK_ORIGIN_Z)),
    )

    # ===================== side lever (revolute) ===========================
    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_shape(), "lever_arm", tolerance=0.0003),
        material="chrome",
        name="lever_shell",
    )

    # Lever pivot: on the right side of the body (+Y), horizontal axle along X.
    # At rest the lever arm extends outward along +Y.
    # Rotation around world X lifts the arm from +Y toward +Z.
    body_r_at_lever = _body_radius_at(LEVER_Z)
    lever_pivot_y = body_r_at_lever  # at the body surface
    model.articulation(
        "lever_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(
            xyz=(0.0, lever_pivot_y, LEVER_Z),
            rpy=(0.0, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),  # rotation around world X
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=LEVER_LOWER,
            upper=LEVER_UPPER,
        ),
    )

    # ===================== screw caps (fixed to body rear) =================
    screw_0 = model.part("screw_cap_0")
    screw_0.visual(
        mesh_from_cadquery(_build_screw_cap_shape(), "screw_cap_0_shell", tolerance=0.0003),
        material="chrome_dark",
        name="screw_cap_0_shell",
    )
    # Position on the back (-X side) of the body at SCREW_CAP_Z[0].
    body_r_0 = _body_radius_at(SCREW_CAP_Z[0])
    model.articulation(
        "screw_cap_0_mount",
        ArticulationType.FIXED,
        parent=body,
        child=screw_0,
        origin=Origin(
            xyz=(-body_r_0 + 0.001, 0.0, SCREW_CAP_Z[0]),
            rpy=(0.0, -math.pi / 2.0, 0.0),  # face outward along -X
        ),
    )

    screw_1 = model.part("screw_cap_1")
    screw_1.visual(
        mesh_from_cadquery(_build_screw_cap_shape(), "screw_cap_1_shell", tolerance=0.0003),
        material="chrome_dark",
        name="screw_cap_1_shell",
    )
    body_r_1 = _body_radius_at(SCREW_CAP_Z[1])
    model.articulation(
        "screw_cap_1_mount",
        ArticulationType.FIXED,
        parent=body,
        child=screw_1,
        origin=Origin(
            xyz=(-body_r_1 + 0.001, 0.0, SCREW_CAP_Z[1]),
            rpy=(0.0, -math.pi / 2.0, 0.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    lever = object_model.get_part("lever")
    screw_0 = object_model.get_part("screw_cap_0")
    screw_1 = object_model.get_part("screw_cap_1")
    lever_joint = object_model.get_articulation("lever_rotate")

    # Allow small overlap where beak seats into body and lever axle penetrates body.
    ctx.allow_overlap(
        spout,
        body,
        elem_a="beak_tube",
        elem_b="body_cone",
        reason="Beak spout shank is intentionally seated into the solid body cone.",
    )
    ctx.allow_overlap(
        lever,
        body,
        elem_a="lever_shell",
        elem_b="body_cone",
        reason="Lever axle boss is intentionally nested into the body side wall.",
    )
    ctx.allow_overlap(
        screw_0,
        body,
        elem_a="screw_cap_0_shell",
        elem_b="body_cone",
        reason="Screw cap is seated flush into the body rear surface.",
    )
    ctx.allow_overlap(
        screw_1,
        body,
        elem_a="screw_cap_1_shell",
        elem_b="body_cone",
        reason="Screw cap is seated flush into the body rear surface.",
    )

    # ---- Tapered conical body: bottom wider than top ---------------------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    cone_aabb = ctx.part_element_world_aabb(body, elem="body_cone")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.001,
        details=f"flange aabb={flange_aabb}",
    )
    ctx.check(
        "tapered body is wider at the base than at the top (conical shape)",
        cone_aabb is not None
        and (cone_aabb[1][0] - cone_aabb[0][0]) > 0.030,
        details=f"cone aabb={cone_aabb}",
    )

    # ---- Beak/spout projects forward and droops ---------------------------
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "beak spout projects forward (+X) from the body",
        spout_aabb is not None and spout_aabb[1][0] > 0.040,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "beak spout outlet droops downward (open rim below beak origin)",
        spout_aabb is not None and spout_aabb[0][2] < BEAK_ORIGIN_Z - 0.010,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- Lever exists on the side, has grooves, and rotates ---------------
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever extends to the right side of the body (+Y)",
        lever_aabb is not None and lever_aabb[1][1] > 0.020,
        details=f"lever aabb={lever_aabb}",
    )

    # Lever joint limits match side-lever mechanism.
    ll = lever_joint.motion_limits
    ctx.check(
        "lever revolute limits allow upward swing for flow control",
        ll is not None
        and ll.lower is not None
        and ll.upper is not None
        and ll.upper > math.radians(30.0)
        and ll.lower < 0.0,
        details=f"limits={ll}",
    )

    # Decisive pose: raising the lever lifts the arm tip upward (+Z).
    rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_shell")
    with ctx.pose({lever_joint: LEVER_UPPER}):
        raised_aabb = ctx.part_element_world_aabb(lever, elem="lever_shell")
    ctx.check(
        "raising the lever to upper limit lifts the arm tip upward (+Z)",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.005,
        details=f"rest_zmax={rest_aabb[1][2]:.4f}, raised_zmax={raised_aabb[1][2]:.4f}",
    )

    # ---- Two screw caps on the back (-X side) of the body -----------------
    s0_aabb = ctx.part_world_aabb(screw_0)
    s1_aabb = ctx.part_world_aabb(screw_1)
    ctx.check(
        "screw_cap_0 sits on the rear (-X) side of the body",
        s0_aabb is not None and s0_aabb[0][0] < -0.005,
        details=f"screw_cap_0 aabb={s0_aabb}",
    )
    ctx.check(
        "screw_cap_1 sits on the rear (-X) side of the body",
        s1_aabb is not None and s1_aabb[0][0] < -0.005,
        details=f"screw_cap_1 aabb={s1_aabb}",
    )
    ctx.check(
        "two screw caps are vertically separated on the body rear",
        s0_aabb is not None
        and s1_aabb is not None
        and abs((s0_aabb[0][2] + s0_aabb[1][2]) / 2.0 - (s1_aabb[0][2] + s1_aabb[1][2]) / 2.0) > 0.012,
        details=f"s0={s0_aabb}, s1={s1_aabb}",
    )

    # ---- Overall height roughly 0.13 m -----------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "overall faucet height is approximately 0.10-0.14 m",
        body_aabb is not None
        and cone_aabb is not None
        and 0.095 <= cone_aabb[1][2] <= 0.140,
        details=f"cone aabb={cone_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
