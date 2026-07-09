from __future__ import annotations

# Tall cylindrical glass storage jar with clamp lid and prismatic stopper.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: tall cylindrical clear glass shell, hollow, with wide mouth,
#     a flared rim, and two bracket mounts for the bail pivot. (root)
#   - clamp_bail: metal U-shaped bail that pivots on the bracket mounts,
#     swings open (revolute about Y) to release the stopper.
#   - stopper: rubber stopper with plug, flange, and knob, seated in the
#     mouth, lifts vertically on a prismatic joint along +Z.

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

# ---- key dimensions (meters) ----
JAR_R = 0.042           # jar body outer radius (84 mm diameter)
JAR_H = 0.180          # jar body height
WALL = 0.004           # glass wall thickness
BASE_T = 0.006         # solid glass base thickness
MOUTH_R = JAR_R - WALL  # inner radius = mouth opening (0.038 — wide mouth)
RIM_R = JAR_R + 0.004   # rim outer radius (0.046)
RIM_H = 0.012          # rim height above jar body

# Bracket mounts on rim for bail pivot
BRACKET_W = 0.010       # bracket width along X
BRACKET_T = 0.006       # bracket thickness along Y
BRACKET_H = 0.015       # bracket height above rim top

# Stopper
PLUG_R = MOUTH_R - 0.002  # plug radius (0.036, fits inside mouth)
PLUG_H = 0.008           # plug height (goes into the mouth)
FLANGE_R = MOUTH_R + 0.004  # flange radius (0.042, rests on rim)
FLANGE_H = 0.004         # flange thickness
KNOB_R = 0.010           # grip knob radius
KNOB_H = 0.012           # grip knob height

# Clamp bail
WIRE_R = 0.003           # bail rod radius
ARM_H = 0.040            # bail arm height above pivot
ARM_SPAN = RIM_R  # arms at same Y as brackets (pivot capture fit)
ARM_EMBED = 0.004  # arms embed into bracket tops for pivot support

PIVOT_Z = JAR_H + RIM_H + BRACKET_H       # bail pivot height (bracket top)
STOPPER_MOUNT_Z = JAR_H + RIM_H            # stopper sits on rim top


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------

def _jar_body_cq() -> cq.Workplane:
    """Tall hollow glass cylinder with rim, wide mouth, and bracket mounts."""
    # Outer cylinder
    outer = cq.Workplane("XY").circle(JAR_R).extrude(JAR_H)

    # Flared rim at top (wider than body)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=JAR_H)
        .circle(RIM_R)
        .extrude(RIM_H)
    )

    # Bracket mounts — two rectangular posts on opposite sides of the rim
    bracket_l = (
        cq.Workplane("XY")
        .workplane(offset=JAR_H + RIM_H)
        .center(0.0, RIM_R)
        .rect(BRACKET_W, BRACKET_T)
        .extrude(BRACKET_H)
    )
    bracket_r = (
        cq.Workplane("XY")
        .workplane(offset=JAR_H + RIM_H)
        .center(0.0, -RIM_R)
        .rect(BRACKET_W, BRACKET_T)
        .extrude(BRACKET_H)
    )

    solid = outer.union(rim).union(bracket_l).union(bracket_r)

    # Hollow cavity open at the top (wide mouth)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BASE_T)
        .circle(MOUTH_R)
        .extrude(JAR_H + RIM_H + 0.001 - BASE_T)
    )

    return solid.cut(cavity)


def _stopper_cq() -> cq.Workplane:
    """Rubber stopper: plug recesses into mouth, flange seats on rim, knob on top."""
    # Plug (below local origin — goes into the mouth)
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-PLUG_H)
        .circle(PLUG_R)
        .extrude(PLUG_H)
    )
    # Flange (at and above local origin — sits on rim surface)
    flange = (
        cq.Workplane("XY")
        .circle(FLANGE_R)
        .extrude(FLANGE_H)
    )
    # Knob (grip handle)
    knob = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H)
        .circle(KNOB_R)
        .extrude(KNOB_H)
    )
    stopper = plug.union(flange).union(knob)
    try:
        stopper = stopper.faces(">Z").chamfer(0.002)
    except Exception:
        pass
    return stopper


def _clamp_bail_cq() -> cq.Workplane:
    """U-shaped metal bail: two vertical arms + horizontal cross-bar.
    Arms extend below the local origin (pivot) to embed into the bracket
    mounts, creating a captured pivot fit."""
    arm_total = ARM_H + ARM_EMBED  # total arm length including embedded portion
    # Left arm (vertical cylinder, starts below origin)
    arm_l = (
        cq.Workplane("XY")
        .workplane(offset=-ARM_EMBED)
        .center(0.0, ARM_SPAN)
        .circle(WIRE_R)
        .extrude(arm_total)
    )
    # Right arm (vertical cylinder, starts below origin)
    arm_r = (
        cq.Workplane("XY")
        .workplane(offset=-ARM_EMBED)
        .center(0.0, -ARM_SPAN)
        .circle(WIRE_R)
        .extrude(arm_total)
    )
    # Cross-bar at top — square-section rod along Y
    crossbar = (
        cq.Workplane("XY")
        .workplane(offset=ARM_H)
        .box(WIRE_R * 2.0, 2.0 * ARM_SPAN, WIRE_R * 2.0)
    )
    return arm_l.union(arm_r).union(crossbar)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_cylindrical_clamp_jar")

    glass = model.material("clear_glass", rgba=(0.80, 0.85, 0.88, 0.30))
    rubber = model.material("rubber", rgba=(0.25, 0.22, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.65, 0.67, 0.70, 1.0))

    # ---- jar body (root): tall hollow cylinder + rim + brackets ----
    body = model.part("jar_body")
    body.visual(
        mesh_from_cadquery(_jar_body_cq(), "jar_glass"),
        material=glass,
        name="jar_glass",
    )
    total_h = JAR_H + RIM_H + BRACKET_H
    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_R, total_h),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, total_h / 2.0)),
    )

    # ---- clamp bail: U-shaped metal bracket ----
    bail = model.part("clamp_bail")
    bail.visual(
        mesh_from_cadquery(_clamp_bail_cq(), "bail_steel"),
        material=steel,
        name="bail_steel",
    )
    bail.inertial = Inertial.from_geometry(
        Box((WIRE_R * 2.0, 2.0 * ARM_SPAN, ARM_H)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, ARM_H / 2.0)),
    )

    # ---- stopper: rubber plug + flange + knob ----
    stopper = model.part("stopper")
    stopper.visual(
        mesh_from_cadquery(_stopper_cq(), "stopper_rubber"),
        material=rubber,
        name="stopper_rubber",
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(FLANGE_R, PLUG_H + FLANGE_H + KNOB_H),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
    )

    # ---- clamp_swing: bail pivots about Y on the brackets ----
    model.articulation(
        "clamp_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=1.4, effort=3.0, velocity=2.0,
        ),
    )

    # ---- stopper_lift: stopper lifts vertically out of the mouth ----
    model.articulation(
        "stopper_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=0.060, effort=2.0, velocity=1.0,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    bail = object_model.get_part("clamp_bail")
    stopper = object_model.get_part("stopper")
    swing = object_model.get_articulation("clamp_swing")
    lift = object_model.get_articulation("stopper_lift")

    # Allow the stopper flange to contact the rim top (seated fit).
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_rubber",
        elem_b="jar_glass",
        reason="The stopper flange is intentionally seated on the rim top surface.",
    )

    # Allow the bail arm pivot capture inside the bracket mounts.
    ctx.allow_overlap(
        bail,
        body,
        elem_a="bail_steel",
        elem_b="jar_glass",
        reason="The bail arms are intentionally captured inside the bracket mounts at the pivot.",
    )
    ctx.expect_contact(
        bail, body, elem_a="bail_steel", elem_b="jar_glass",
        name="bail contacts brackets at pivot",
    )

    # --- jar body is cylindrical and tall ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body has roughly circular cross-section",
        abs(bext[0] - bext[1]) < 0.020,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is tall (height > 1.5× diameter)",
        bext[2] > bext[0] * 1.5,
        details=f"height={bext[2]:.4f}, x-ext={bext[0]:.4f}",
    )

    # --- stopper sits at the jar mouth ---
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper is at the jar mouth level",
        stopper_pos is not None and stopper_pos[2] > JAR_H - 0.01,
        details=f"stopper z={stopper_pos[2] if stopper_pos else None}",
    )
    ctx.expect_overlap(
        stopper, body, axes="xy", min_overlap=0.02,
        name="stopper overlaps jar mouth footprint",
    )

    # --- stopper_lift raises the stopper vertically ---
    z_rest = ctx.part_world_position(stopper)[2]
    with ctx.pose({lift: 0.060}):
        z_lift = ctx.part_world_position(stopper)[2]
    ctx.check(
        "stopper lifts vertically out of the mouth",
        z_lift > z_rest + 0.040,
        details=f"rest z={z_rest:.4f}, lifted z={z_lift:.4f}",
    )

    # --- clamp bail is mounted at the top of the jar ---
    bail_pos = ctx.part_world_position(bail)
    ctx.check(
        "clamp bail is at the top of the jar",
        bail_pos is not None and bail_pos[2] > JAR_H + RIM_H,
        details=f"bail z={bail_pos[2] if bail_pos else None}",
    )

    # --- clamp_swing opens the bail (X extent changes as arms swing) ---
    bail_aabb_rest = ctx.part_world_aabb(bail)
    with ctx.pose({swing: 1.2}):
        bail_aabb_open = ctx.part_world_aabb(bail)
    x_rest = bail_aabb_rest[1][0] - bail_aabb_rest[0][0]
    x_open = bail_aabb_open[1][0] - bail_aabb_open[0][0]
    ctx.check(
        "clamp_swing rotates the bail (X extent changes)",
        abs(x_open - x_rest) > 0.010,
        details=f"rest x-ext={x_rest:.4f}, open x-ext={x_open:.4f}",
    )

    # --- joint types and axes ---
    ctx.check(
        "stopper_lift is prismatic along +Z",
        lift.articulation_type == ArticulationType.PRISMATIC
        and lift.axis == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )
    ctx.check(
        "clamp_swing is revolute about Y",
        swing.articulation_type == ArticulationType.REVOLUTE
        and swing.axis == (0.0, 1.0, 0.0),
        details=f"type={swing.articulation_type}, axis={swing.axis}",
    )

    # --- wide mouth: mouth radius is close to jar body radius ---
    ctx.check(
        "jar has a wide mouth (mouth_r > 0.80 * jar_r)",
        MOUTH_R > JAR_R * 0.80,
        details=f"mouth_r={MOUTH_R:.4f}, jar_r={JAR_R:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
