from __future__ import annotations

# Square glass storage jar with a hinged flip-top lid and clamp bail.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: square-section clear glass shell with rounded vertical edges,
#     hollow inside, wide round mouth opening, short neck, two side pivot lugs
#     for the bail, and a rear hinge pin. (root)
#   - lid: round glass disc with rubber gasket ring and hinge knuckle, hinged
#     at the rear of the mouth via a revolute joint.
#   - bail: U-shaped steel wire clamp bail that pivots on the side lugs via
#     a revolute joint; swings forward to release the lid.
# Articulations:
#   body_to_lid  (REVOLUTE): lid flips open at the rear hinge, axis +X
#   body_to_bail (REVOLUTE): bail swings forward to release, axis -X

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
BODY_HALF = 0.040          # half-width of square section (80 mm)
BODY_FILLET = 0.012        # rounded vertical-edge radius
WALL = 0.003               # glass wall thickness
BODY_TOP = 0.120           # top of square body section
SHOULDER_TOP = 0.130       # top of tapered shoulder
MOUTH_R = 0.035            # mouth outer radius (70 mm wide mouth)
NECK_TOP = 0.140           # top of neck rim

LID_R = 0.037              # lid disc radius (slightly > mouth)
LID_THICK = 0.005          # lid disc thickness
GASKET_OR = 0.031          # gasket outer radius
GASKET_IR = 0.023          # gasket inner radius
GASKET_THICK = 0.002       # gasket thickness

# Bail pivot is on the body sides just above the square section
PIVOT_Z = BODY_TOP + 0.003  # 0.123
BAIL_HALF_W = 0.046         # half-spacing between bail arms
# Arm height: cross-bar should land on lid top when closed
BAIL_ARM_Z = (NECK_TOP + LID_THICK) - PIVOT_Z  # 0.022
WIRE_D = 0.003              # bail wire square cross-section side

# Pivot lugs on body sides
LUG_R = 0.004               # lug cylinder radius
LUG_LEN = 0.008             # lug protrusion length

# Hinge at rear of mouth
HINGE_Y = -(MOUTH_R - 0.003)  # -0.032
HINGE_Z = NECK_TOP              # 0.140
HINGE_PIN_R = 0.002             # hinge pin radius
HINGE_PIN_HALF = 0.010          # half-length of hinge pin along X


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------

def _body_solid() -> cq.Workplane:
    """Hollow square glass jar with wide mouth, pivot lugs, and hinge pin."""
    # --- outer shell ---
    outer_box = (
        cq.Workplane("XY")
        .box(2 * BODY_HALF, 2 * BODY_HALF, BODY_TOP, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_FILLET)
    )
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .rect(2 * (BODY_HALF - 0.004), 2 * (BODY_HALF - 0.004))
        .workplane(offset=SHOULDER_TOP - BODY_TOP)
        .circle(MOUTH_R)
        .loft(ruled=False)
    )
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP)
        .circle(MOUTH_R)
        .extrude(NECK_TOP - SHOULDER_TOP)
    )
    outer = outer_box.union(shoulder).union(neck)

    # --- inner cavity (hollow, opens at the mouth top) ---
    inner_box = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(
            2 * (BODY_HALF - WALL),
            2 * (BODY_HALF - WALL),
            BODY_TOP - WALL,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(max(BODY_FILLET - WALL, 0.001))
    )
    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .rect(2 * (BODY_HALF - 0.004 - WALL), 2 * (BODY_HALF - 0.004 - WALL))
        .workplane(offset=(SHOULDER_TOP - BODY_TOP) + 0.001)
        .circle(MOUTH_R - WALL)
        .loft(ruled=False)
    )
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP)
        .circle(MOUTH_R - WALL)
        .extrude((NECK_TOP - SHOULDER_TOP) + 0.001)
    )
    cavity = inner_box.union(inner_shoulder).union(inner_neck)
    shell = outer.cut(cavity)

    # --- pivot lugs on body sides (for bail) ---
    # Left lug: protrudes in -X from body face
    left_lug = (
        cq.Workplane("YZ")
        .workplane(offset=-(BODY_HALF + LUG_LEN))
        .center(0, PIVOT_Z)
        .circle(LUG_R)
        .extrude(LUG_LEN + 0.003)
    )
    # Right lug: protrudes in +X from body face
    right_lug = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_HALF - 0.003)
        .center(0, PIVOT_Z)
        .circle(LUG_R)
        .extrude(LUG_LEN + 0.003)
    )
    shell = shell.union(left_lug).union(right_lug)

    # --- hinge pin at rear of mouth (short cylinder along X) ---
    hinge_pin = (
        cq.Workplane("YZ")
        .workplane(offset=-HINGE_PIN_HALF)
        .center(HINGE_Y, HINGE_Z)
        .circle(HINGE_PIN_R)
        .extrude(2 * HINGE_PIN_HALF)
    )
    # Small hinge lug (rectangular tab connecting pin to neck)
    hinge_lug = (
        cq.Workplane("XY")
        .workplane(offset=HINGE_Z - 0.005)
        .center(0, HINGE_Y)
        .rect(0.012, 0.008)
        .extrude(0.008)
    )
    shell = shell.union(hinge_pin).union(hinge_lug)

    return shell


def _lid_solid() -> cq.Workplane:
    """Round glass lid disc with hinge knuckle at the rear."""
    # Main disc – center offset from lid origin (which is at the hinge point)
    # In lid frame: hinge at origin, disc center at (0, MOUTH_R-0.003, LID_THICK/2)
    disc_cy = MOUTH_R - 0.003  # 0.032
    disc_cz = LID_THICK / 2.0  # 0.0025

    disc = (
        cq.Workplane("XY")
        .workplane(offset=disc_cz - LID_THICK / 2.0)
        .center(0, disc_cy)
        .circle(LID_R)
        .extrude(LID_THICK)
    )

    # Hinge knuckle: short cylinder along X at the lid origin
    knuckle = (
        cq.Workplane("YZ")
        .workplane(offset=-0.008)
        .center(0, 0)
        .circle(HINGE_PIN_R + 0.0012)
        .extrude(0.016)
    )
    # Small tab connecting knuckle to disc
    tab = (
        cq.Workplane("XY")
        .workplane(offset=0)
        .center(0, 0.006)
        .rect(0.012, 0.014)
        .extrude(LID_THICK)
    )
    lid = disc.union(knuckle).union(tab)
    return lid


def _gasket_solid() -> cq.Workplane:
    """Rubber gasket ring on the underside of the lid."""
    disc_cy = MOUTH_R - 0.003
    # Gasket sits just below the lid disc
    gasket_z = -GASKET_THICK
    ring = (
        cq.Workplane("XY")
        .workplane(offset=gasket_z)
        .center(0, disc_cy)
        .circle(GASKET_OR)
        .circle(GASKET_IR)
        .extrude(GASKET_THICK)
    )
    return ring


def _bail_solid() -> cq.Workplane:
    """U-shaped steel wire bail (two arms + cross-bar)."""
    w = WIRE_D
    half_w = BAIL_HALF_W
    arm_z = BAIL_ARM_Z

    # Left arm (vertical box along Z)
    left_arm = (
        cq.Workplane("XY")
        .center(-half_w, 0)
        .rect(w, w)
        .extrude(arm_z)
    )
    # Right arm
    right_arm = (
        cq.Workplane("XY")
        .center(half_w, 0)
        .rect(w, w)
        .extrude(arm_z)
    )
    # Cross-bar (horizontal box along X at top of arms)
    cross_bar = (
        cq.Workplane("XY")
        .workplane(offset=arm_z - w / 2.0)
        .rect(2 * half_w + w, w)
        .extrude(w)
    )
    bail = left_arm.union(right_arm).union(cross_bar)
    return bail


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_glass_bail_jar")

    glass = model.material("clear_glass", rgba=(0.82, 0.86, 0.88, 0.25))
    rubber = model.material("rubber_orange", rgba=(0.85, 0.30, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))

    # ---- jar body (root): hollow glass shell with lugs and hinge pin ----
    body = model.part("jar_body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "jar_glass"),
        material=glass,
        name="jar_glass",
    )
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_HALF, 2 * BODY_HALF, NECK_TOP)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP / 2.0)),
    )

    # ---- lid: glass disc with gasket and hinge knuckle ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_disc"),
        material=glass,
        name="lid_disc",
    )
    lid.visual(
        mesh_from_cadquery(_gasket_solid(), "lid_gasket"),
        material=rubber,
        name="lid_gasket",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_THICK),
        mass=0.04,
        origin=Origin(xyz=(0.0, MOUTH_R - 0.003, LID_THICK / 2.0)),
    )

    # ---- bail: steel wire U-shape ----
    bail = model.part("bail")
    bail.visual(
        mesh_from_cadquery(_bail_solid(), "bail_wire"),
        material=steel,
        name="bail_wire",
    )
    bail.inertial = Inertial.from_geometry(
        Box((2 * BAIL_HALF_W, WIRE_D, BAIL_ARM_Z)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, BAIL_ARM_Z / 2.0)),
    )

    # ---- articulation: lid hinge (revolute at rear of mouth) ----
    # Hinge origin at the hinge pin center in body frame.
    # Lid disc extends in +Y from hinge; axis +X makes positive q open upward.
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=2.2,
        ),
    )

    # ---- articulation: bail pivot (revolute at side pivot axis) ----
    # Bail arms extend in +Z from pivot; axis -X makes positive q swing forward.
    model.articulation(
        "body_to_bail",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=2.5,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    lid = object_model.get_part("lid")
    bail = object_model.get_part("bail")
    lid_joint = object_model.get_articulation("body_to_lid")
    bail_joint = object_model.get_articulation("body_to_bail")

    # ---- intentional overlaps ----
    ctx.allow_overlap(
        bail, lid,
        elem_a="bail_wire", elem_b="lid_disc",
        reason="Bail cross-bar clamps down onto the lid when closed.",
    )
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_gasket", elem_b="jar_glass",
        reason="Rubber gasket compresses against the neck rim for sealing.",
    )
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_disc", elem_b="jar_glass",
        reason="Lid hinge knuckle wraps around the body hinge pin (captured pin).",
    )

    # ---- jar body is square section (lugs make X slightly wider) ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body Y matches expected square width",
        abs(bext[1] - 2 * BODY_HALF) < 0.005,
        details=f"y={bext[1]:.4f}, expected={2*BODY_HALF:.4f}",
    )
    ctx.check(
        "jar is taller than wide",
        bext[2] > bext[1] + 0.02,
        details=f"height={bext[2]:.4f}, width_y={bext[1]:.4f}",
    )

    # ---- body has pivot lugs protruding in X (wider than bare body) ----
    ctx.check(
        "body has pivot lugs (X wider than square body)",
        bext[0] > 2 * BODY_HALF + 0.008,
        details=f"body x extent={bext[0]:.4f}, expected > {2*BODY_HALF + 0.008:.4f}",
    )

    # ---- lid is a disc, sits on neck ----
    lext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid is disc-shaped (footprint >> thickness)",
        lext[0] > lext[2] * 3 and lext[1] > lext[2] * 3,
        details=f"lid extents={lext}",
    )
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at neck height",
        lid_pos is not None and lid_pos[2] >= HINGE_Z - 0.010,
        details=f"lid origin z={lid_pos[2] if lid_pos else None}",
    )

    # ---- bail exists and is above pivot height ----
    bail_ext = _ext(ctx.part_world_aabb(bail))
    bail_pos = ctx.part_world_position(bail)
    ctx.check(
        "bail is wider than body (spans the jar)",
        bail_ext[0] > 2 * BODY_HALF + 0.004,
        details=f"bail x={bail_ext[0]:.4f}",
    )

    # ---- both joints are revolute with correct limits ----
    ctx.check(
        "body_to_lid is revolute",
        lid_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={lid_joint.articulation_type}",
    )
    ctx.check(
        "body_to_bail is revolute",
        bail_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={bail_joint.articulation_type}",
    )
    ctx.check(
        "lid joint has non-trivial range",
        lid_joint.motion_limits.upper > 1.0,
        details=f"upper={lid_joint.motion_limits.upper}",
    )
    ctx.check(
        "bail joint has non-trivial range",
        bail_joint.motion_limits.upper > 1.0,
        details=f"upper={bail_joint.motion_limits.upper}",
    )

    # ---- opening the lid raises the front edge ----
    rest_lid_aabb = ctx.part_world_aabb(lid)
    rest_lid_zmax = rest_lid_aabb[1][2]
    with ctx.pose({lid_joint: 1.5}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        open_lid_zmax = open_lid_aabb[1][2]
    ctx.check(
        "lid opens upward (zmax increases at q=1.5)",
        open_lid_zmax > rest_lid_zmax + 0.015,
        details=f"rest zmax={rest_lid_zmax:.4f}, open zmax={open_lid_zmax:.4f}",
    )

    # ---- opening the bail swings it forward (+Y) ----
    rest_bail_aabb = ctx.part_world_aabb(bail)
    rest_bail_ymax = rest_bail_aabb[1][1]
    with ctx.pose({bail_joint: 1.5}):
        open_bail_aabb = ctx.part_world_aabb(bail)
        open_bail_ymax = open_bail_aabb[1][1]
    ctx.check(
        "bail swings forward (+Y) when opened",
        open_bail_ymax > rest_bail_ymax + 0.008,
        details=f"rest ymax={rest_bail_ymax:.4f}, open ymax={open_bail_ymax:.4f}",
    )

    # ---- at rest, bail cross-bar overlaps lid footprint (clamping) ----
    # The cross-bar is narrow (3mm wire) so Y overlap is small; check X only.
    ctx.expect_overlap(
        bail, lid, axes="x", min_overlap=0.020,
        elem_a="bail_wire", elem_b="lid_disc",
        name="bail cross-bar spans lid width when closed",
    )

    # ---- lid overlaps body mouth footprint (seated on rim) ----
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.020,
        name="lid seated on mouth footprint",
    )

    return ctx.report()


object_model = build_object_model()
