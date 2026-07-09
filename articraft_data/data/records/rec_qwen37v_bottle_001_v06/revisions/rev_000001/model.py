from __future__ import annotations

# Canteen-style flat oval bottle with swing-top stopper on side hinge arms.
# Frame: bottle axis along +Z, base at z=0, neck/stopper at the top (+Z).
# Body: flat oval (elliptical) hollow shell with molded volume bands and side
# carrying loops.  Neck carries two fixed hinge arms that support a REVOLUTE
# swing-top stopper pivoting about the Y axis.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- body dimensions (meters) ----
BODY_RX = 0.040          # semi-major (X, wider)
BODY_RY = 0.022          # semi-minor (Y, flatter)
WALL = 0.002
BODY_TOP_Z = 0.120
SHOULDER_TOP_Z = 0.140
NECK_R = 0.012
NECK_TOP_Z = 0.155

# ---- hinge arms (fixed to body) ----
ARM_THICKNESS = 0.003
ARM_WIDTH = 0.007
ARM_BOTTOM_Z = SHOULDER_TOP_Z
ARM_TOP_Z = 0.164
ARM_Y_CENTER = NECK_R + ARM_THICKNESS / 2.0 - 0.001

# ---- pivot ----
PIVOT_Z = 0.161

# ---- stopper (local frame: origin at pivot) ----
PLUG_R = 0.010
PLUG_H = 0.012
CAP_R = 0.014
CAP_H = 0.008
S_PLUG_TOP = NECK_TOP_Z - PIVOT_Z
S_PLUG_BOT = S_PLUG_TOP - PLUG_H
S_CAP_BOT = S_PLUG_TOP
S_CAP_TOP = S_CAP_BOT + CAP_H

LUG_DX = 0.006
LUG_DY = 0.005
LUG_DZ = 0.005

# ---- volume bands ----
BAND_ZS = [0.030, 0.060, 0.090]
BAND_W = 0.003
BAND_RAISE = 0.0012

# ---- side loops ----
LOOP_Z = 0.100
LOOP_DX = 0.012
LOOP_DY = 0.010
LOOP_DZ = 0.016
LOOP_HOLE_R = 0.004


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------

def _build_body_outer():
    """Outer body as a single loft from base ellipse to neck circle."""
    outer = (
        cq.Workplane("XY")
        .ellipse(BODY_RX, BODY_RY)
        .workplane(offset=NECK_TOP_Z)
        .circle(NECK_R)
        .loft()
    )
    outer = outer.edges("<Z").fillet(0.005)
    return outer


def _build_body_cavity():
    """Simple interior cavity open at the neck."""
    # Elliptical cavity in the body region
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, WALL * 3))
        .ellipse(BODY_RX - WALL, BODY_RY - WALL)
        .extrude(BODY_TOP_Z - WALL)
    )
    # Neck bore through shoulder and neck
    bore = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, BODY_TOP_Z))
        .circle(NECK_R - WALL)
        .extrude(NECK_TOP_Z - BODY_TOP_Z + 0.005)
    )
    return cavity.union(bore)


def _build_stopper_solid():
    """Swing-top stopper: plug + cap + pivot lugs."""
    plug = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, S_PLUG_BOT))
        .circle(PLUG_R)
        .extrude(PLUG_H)
    )
    cap = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, S_CAP_BOT))
        .circle(CAP_R)
        .extrude(CAP_H)
    )
    cap = cap.edges(">Z").fillet(0.001)

    result = plug.union(cap)

    # Pivot lugs on ±Y sides
    for sign in (1, -1):
        lug_y = sign * (CAP_R + LUG_DY / 2.0)
        lug = (
            cq.Workplane("XY")
            .transformed(offset=(0, lug_y, -LUG_DZ / 2.0))
            .box(LUG_DX, LUG_DY, LUG_DZ, centered=(True, True, False))
        )
        result = result.union(lug)

    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canteen_bottle")

    # Materials
    translucent = model.material("canteen_green", rgba=(0.22, 0.46, 0.36, 0.55))
    rubber = model.material("stopper_rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    # ---- Build body CadQuery solid ----
    outer = _build_body_outer()
    cavity = _build_body_cavity()
    body_solid = outer.cut(cavity)

    # Hinge arms (union onto body)
    arm_h = ARM_TOP_Z - ARM_BOTTOM_Z
    for sign in (1, -1):
        arm = (
            cq.Workplane("XY")
            .transformed(offset=(0, sign * ARM_Y_CENTER, ARM_BOTTOM_Z))
            .box(ARM_WIDTH, ARM_THICKNESS, arm_h, centered=(True, True, False))
        )
        body_solid = body_solid.union(arm)

    # Volume bands (elliptical protrusions around body)
    for bz in BAND_ZS:
        band = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, bz))
            .ellipse(BODY_RX + BAND_RAISE, BODY_RY + BAND_RAISE)
            .extrude(BAND_W)
        )
        body_solid = body_solid.union(band)

    # Side loops (bracket-style boxes with hole impression)
    for sign in (1, -1):
        y_center = sign * (BODY_RY + LOOP_DY / 2.0 - 0.001)
        loop = (
            cq.Workplane("XY")
            .transformed(offset=(0, y_center, LOOP_Z))
            .box(LOOP_DX, LOOP_DY, LOOP_DZ, centered=(True, True, True))
        )
        body_solid = body_solid.union(loop)

    # ---- Parts ----
    body = model.part("bottle")
    body.visual(
        mesh_from_cadquery(body_solid, "canteen_shell"),
        material=translucent,
        name="canteen_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_RX, NECK_TOP_Z),
        mass=0.060,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    stopper = model.part("stopper")
    stopper_solid = _build_stopper_solid()
    stopper.visual(
        mesh_from_cadquery(stopper_solid, "stopper_shell"),
        material=rubber,
        name="stopper_shell",
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, PLUG_H + CAP_H),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, (S_PLUG_BOT + S_CAP_TOP) / 2.0)),
    )

    # ---- Swing-top revolute joint ----
    model.articulation(
        "stopper_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=2.6,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    stopper = object_model.get_part("stopper")
    swing = object_model.get_articulation("stopper_swing")

    body_shell = body.get_visual("canteen_shell")
    stopper_shell = stopper.get_visual("stopper_shell")

    # --- body is flat oval (wider in X than Y) ---
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        mn, mx = body_aabb
        dx = mx[0] - mn[0]
        dy = mx[1] - mn[1]
        ctx.check(
            "body is flat oval (X extent significantly > Y extent)",
            dx > dy * 1.2,
            details=f"dx={dx:.4f}, dy={dy:.4f}",
        )

    # --- translucent body material ---
    ctx.check(
        "body material is translucent (alpha < 1)",
        body_shell.material.rgba is not None and body_shell.material.rgba[3] < 1.0,
        details=f"rgba={body_shell.material.rgba}",
    )

    # --- stopper material is opaque rubber ---
    ctx.check(
        "stopper material is opaque dark rubber",
        stopper_shell.material.rgba is not None
        and stopper_shell.material.rgba[3] >= 0.99
        and max(stopper_shell.material.rgba[:3]) < 0.25,
        details=f"rgba={stopper_shell.material.rgba}",
    )

    # --- stopper seated on neck at rest ---
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_shell",
        elem_b="canteen_shell",
        reason="Stopper plug is intentionally seated inside the neck opening.",
    )

    stopper_rest_z = ctx.part_world_position(stopper)
    ctx.check(
        "stopper is at neck height when closed",
        stopper_rest_z is not None and stopper_rest_z[2] > BODY_TOP_Z,
        details=f"stopper_pos={stopper_rest_z}",
    )

    # --- swing-top opens: stopper bounding box changes at mid-swing ---
    # The part origin is at the pivot (rotation center), so we check the AABB
    # which captures the geometry extent change during rotation.
    rest_aabb = ctx.part_world_aabb(stopper)
    with ctx.pose({swing: 1.3}):
        open_aabb = ctx.part_world_aabb(stopper)
    if rest_aabb is not None and open_aabb is not None:
        rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
        open_dx = open_aabb[1][0] - open_aabb[0][0]
        rest_dz = rest_aabb[1][2] - rest_aabb[0][2]
        open_dz = open_aabb[1][2] - open_aabb[0][2]
        # At rest the stopper extends mostly in Z (plug hangs down).
        # At mid-swing, the plug rotates sideways so X extent grows and Z shrinks.
        ctx.check(
            "swing-top opens (stopper AABB changes at mid-swing)",
            abs(open_dx - rest_dx) > 0.003 or abs(open_dz - rest_dz) > 0.003,
            details=f"rest_dx={rest_dx:.4f}, open_dx={open_dx:.4f}, "
                    f"rest_dz={rest_dz:.4f}, open_dz={open_dz:.4f}",
        )

    # --- joint is revolute with proper limits ---
    limits = swing.motion_limits
    ctx.check(
        "stopper_swing is revolute with bounded limits",
        swing.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.upper > limits.lower + 0.5,
        details=f"type={swing.articulation_type}, limits={limits}",
    )

    return ctx.report()


object_model = build_object_model()
