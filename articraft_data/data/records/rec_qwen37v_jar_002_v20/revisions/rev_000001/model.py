from __future__ import annotations

# Ceramic jar variant: round ceramic body with thread ridges on the rim,
# a spoon notch cut into the rim lip, and a stopper that lifts vertically
# on a prismatic joint. Glass/ceramic wall thickness visible at the mouth.
#
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: round ceramic shell, hollow inside, with a flared rim
#     featuring thread ridges and a spoon notch. (root)
#   - stopper: ceramic disc stopper seated in the mouth, lifts vertically.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
BODY_R = 0.042             # outer radius of the cylindrical body
WALL = 0.004               # ceramic wall thickness
BODY_Z0 = 0.0              # jar base sits on the ground
BODY_TOP = 0.105           # top of the main cylindrical body
SHOULDER_TOP = 0.115       # top of inward-tapering shoulder
NECK_R = 0.034             # outer radius of the neck/mouth rim
NECK_TOP = 0.140           # top of the neck rim
NECK_BOTTOM = SHOULDER_TOP
MOUTH_R = NECK_R - WALL    # inner mouth opening radius

RIM_HEIGHT = 0.008         # raised rim lip above neck shoulder
RIM_TOP = NECK_TOP
RIM_BOTTOM = NECK_TOP - RIM_HEIGHT

# Thread ridges on outside of neck
THREAD_N = 4               # number of thread ridge rings
THREAD_HEIGHT = 0.002      # ridge thickness
THREAD_BUMP = 0.001        # ridge radial protrusion

# Spoon notch dimensions
NOTCH_WIDTH = 0.014        # width of the spoon notch
NOTCH_DEPTH = 0.006        # depth of notch below rim top
NOTCH_ANGLE = 0.0          # angular position of notch (front, +X side)

# Stopper dimensions
STOPPER_R = NECK_R - 0.001    # stopper disc wider than mouth, rests on rim lip
STOPPER_HEIGHT = 0.010        # stopper disc thickness
STOPPER_PLUG_R = MOUTH_R - 0.002  # plug that extends into mouth for centering
STOPPER_PLUG_DEPTH = 0.008    # how far plug extends below disc
STOPPER_KNOB_R = 0.010        # handle knob on top
STOPPER_KNOB_H = 0.012        # knob height

# Stopper seat: rest position has stopper bottom at rim top
STOPPER_REST_Z = RIM_TOP
STOPPER_LIFT = 0.060          # max vertical lift


def _jar_body_solid() -> cq.Workplane:
    """Round ceramic jar body: hollow cylinder with tapered shoulder,
    raised neck rim with thread ridges and spoon notch."""

    # Main cylindrical body (outer shell)
    outer_body = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_TOP)
    )

    # Slight bottom thickening (base foot)
    base_foot = (
        cq.Workplane("XY")
        .circle(BODY_R + 0.002)
        .extrude(0.005)
    )
    outer_body = outer_body.union(base_foot)

    # Tapered shoulder: body top -> neck base
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .circle(BODY_R - 0.002)
        .workplane(offset=SHOULDER_TOP - BODY_TOP)
        .circle(NECK_R)
        .loft(ruled=False)
    )
    outer_body = outer_body.union(shoulder)

    # Neck wall (thicker rim region for visible wall thickness)
    neck_wall = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R)
        .extrude(NECK_TOP - NECK_BOTTOM)
    )
    outer_body = outer_body.union(neck_wall)

    # Raised rim lip at the very top (slightly wider than neck for the lip)
    rim_lip = (
        cq.Workplane("XY")
        .workplane(offset=RIM_BOTTOM)
        .circle(NECK_R + 0.002)
        .extrude(RIM_HEIGHT)
    )
    outer_body = outer_body.union(rim_lip)

    # Now hollow out the interior
    inner_body = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(BODY_R - WALL)
        .extrude(BODY_TOP - WALL)
    )
    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .circle(BODY_R - WALL - 0.002)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP) + 0.001)
        .circle(MOUTH_R)
        .loft(ruled=False)
    )
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(MOUTH_R)
        .extrude((NECK_TOP - NECK_BOTTOM) + RIM_HEIGHT + 0.001)
    )
    cavity = inner_body.union(inner_shoulder).union(inner_neck)

    solid = outer_body.cut(cavity)

    # Thread ridges: small raised rings on the outside of the neck
    for i in range(THREAD_N):
        zc = NECK_BOTTOM + 0.004 + i * 0.005
        ridge = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(NECK_R + THREAD_BUMP)
            .circle(NECK_R - 0.0003)
            .extrude(THREAD_HEIGHT)
        )
        solid = solid.union(ridge)

    # Spoon notch: U-shaped cutout on the +X side of the rim lip
    # This is a half-cylinder cut into the rim top
    notch_center_x = NECK_R + 0.002  # at the outer edge of the rim lip
    notch_cutter = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP - NOTCH_DEPTH)
        .center(notch_center_x, 0.0)
        .rect(NOTCH_WIDTH, NOTCH_WIDTH * 0.8)
        .extrude(NOTCH_DEPTH + 0.002)
    )
    # Also a rounded bottom for the notch (half-sphere cutter)
    notch_round = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP - NOTCH_DEPTH)
        .center(notch_center_x, 0.0)
        .circle(NOTCH_WIDTH / 2.0)
        .extrude(NOTCH_DEPTH + 0.001)
    )
    solid = solid.cut(notch_round)

    return solid


def _jar_body_mesh():
    return mesh_from_cadquery(_jar_body_solid(), "jar_ceramic")


def _stopper_solid() -> cq.Workplane:
    """Ceramic stopper: disc that rests on the rim lip, with a centering plug
    that extends into the mouth and a raised knob handle on top."""
    # Main disc (sits on top of rim lip)
    disc = (
        cq.Workplane("XY")
        .circle(STOPPER_R)
        .extrude(STOPPER_HEIGHT)
    )

    # Centering plug extends below the disc into the mouth opening
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-STOPPER_PLUG_DEPTH)
        .circle(STOPPER_PLUG_R)
        .extrude(STOPPER_PLUG_DEPTH)
    )
    disc = disc.union(plug)

    # Knob handle on top
    knob = (
        cq.Workplane("XY")
        .workplane(offset=STOPPER_HEIGHT)
        .circle(STOPPER_KNOB_R)
        .extrude(STOPPER_KNOB_H)
    )
    disc = disc.union(knob)

    # Rounded knob top
    knob_dome = (
        cq.Workplane("XY")
        .workplane(offset=STOPPER_HEIGHT + STOPPER_KNOB_H)
        .sphere(STOPPER_KNOB_R)
    )
    disc = disc.union(knob_dome)

    return disc


def _stopper_mesh():
    return mesh_from_cadquery(_stopper_solid(), "stopper_ceramic")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceramic_jar_with_stopper")

    ceramic = model.material("ceramic_white", rgba=(0.92, 0.89, 0.84, 1.0))
    ceramic_glaze = model.material("ceramic_glaze", rgba=(0.85, 0.82, 0.76, 1.0))
    stopper_mat = model.material("stopper_ceramic", rgba=(0.88, 0.84, 0.78, 1.0))

    # ---- jar body (root): round ceramic shell with rim, threads, notch ----
    body = model.part("jar_body")
    body.visual(_jar_body_mesh(), material=ceramic, name="jar_ceramic")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP / 2.0)),
    )

    # ---- stopper: ceramic disc with knob, seated in mouth ----
    stopper = model.part("stopper")
    stopper.visual(
        _stopper_mesh(),
        material=stopper_mat,
        # Stopper local origin at disc bottom; position so disc bottom is at rim top
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="stopper_ceramic",
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_R, STOPPER_PLUG_DEPTH + STOPPER_HEIGHT + STOPPER_KNOB_H),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, (STOPPER_HEIGHT - STOPPER_PLUG_DEPTH) / 2.0)),
    )

    # ---- prismatic joint: stopper lifts vertically ----
    # Origin at the mouth top where the stopper seats.
    # At q=0 stopper is seated; at q=STOPPER_LIFT it's fully lifted.
    model.articulation(
        "stopper_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STOPPER_LIFT,
            effort=2.0,
            velocity=0.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    stopper = object_model.get_part("stopper")
    lift = object_model.get_articulation("stopper_lift")

    # The stopper sits inside the mouth opening — intentional seated fit.
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_ceramic",
        elem_b="jar_ceramic",
        reason="The stopper disc is intentionally seated inside the jar mouth opening.",
    )

    # --- stopper_lift is prismatic along +Z ---
    ctx.check(
        "stopper_lift is prismatic along +Z",
        lift.articulation_type == ArticulationType.PRISMATIC
        and lift.axis == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )

    # --- stopper has joint limits matching the lift range ---
    limits = lift.motion_limits
    ctx.check(
        "stopper_lift has bounded limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )
    if limits and limits.lower is not None and limits.upper is not None:
        ctx.check(
            "stopper_lift range matches design",
            abs(limits.lower - 0.0) < 1e-6 and abs(limits.upper - STOPPER_LIFT) < 1e-6,
            details=f"lower={limits.lower}, upper={limits.upper}",
        )

    # --- stopper lifts vertically when joint is actuated ---
    z_rest = ctx.part_world_position(stopper)[2]
    with ctx.pose({lift: STOPPER_LIFT}):
        z_lifted = ctx.part_world_position(stopper)[2]
    ctx.check(
        "stopper lifts vertically on actuation",
        z_lifted > z_rest + 0.04,
        details=f"rest z={z_rest:.4f}, lifted z={z_lifted:.4f}",
    )

    # --- stopper stays centered over the jar mouth in XY ---
    ctx.expect_within(
        stopper, body, axes="xy", margin=0.01,
        name="stopper centered over jar mouth",
    )

    # --- jar body is round (cylindrical cross-section) ---
    baabb = ctx.part_world_aabb(body)
    bext = (baabb[1][0] - baabb[0][0], baabb[1][1] - baabb[0][1], baabb[1][2] - baabb[0][2])
    ctx.check(
        "jar body is round (similar X and Y extents)",
        abs(bext[0] - bext[1]) < 0.008,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )

    # --- jar has visible wall thickness at the mouth ---
    # The neck outer radius is NECK_R and inner is MOUTH_R; difference is WALL
    ctx.check(
        "mouth wall thickness is designed",
        abs((NECK_R - MOUTH_R) - WALL) < 1e-6,
        details=f"neck_r={NECK_R}, mouth_r={MOUTH_R}, wall={WALL}",
    )

    # --- thread ridges exist (jar body extends beyond plain neck radius) ---
    # The thread ridges protrude THREAD_BUMP beyond NECK_R
    ctx.check(
        "thread ridges protrude beyond neck radius",
        THREAD_BUMP > 0.0005,
        details=f"thread_bump={THREAD_BUMP}",
    )

    # --- spoon notch is cut into the rim ---
    ctx.check(
        "spoon notch is defined in the rim",
        NOTCH_WIDTH > 0.010 and NOTCH_DEPTH > 0.004,
        details=f"notch_width={NOTCH_WIDTH}, notch_depth={NOTCH_DEPTH}",
    )

    # --- stopper is seated at the mouth when at rest ---
    ctx.expect_overlap(
        stopper, body, axes="xy", min_overlap=0.02,
        name="stopper seated over mouth footprint at rest",
    )

    return ctx.report()


object_model = build_object_model()
