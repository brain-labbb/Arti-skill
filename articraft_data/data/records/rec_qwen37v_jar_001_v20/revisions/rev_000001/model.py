from __future__ import annotations

# CERAMIC JAR with spoon notch and lift-off stopper.
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# A squat ceramic jar with thick walls, a wide mouth, decorative thread ridges
# around the rim, and a spoon notch cut into one side of the rim. The stopper
# is a plug with a grip knob that lifts vertically out of the mouth.
#
# Articulation:
#   - stopper_lift: PRISMATIC vertical lift of the stopper along +Z

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
JAR_OUTER_R = 0.040           # outer radius of the ceramic body (~80mm dia)
JAR_BODY_H = 0.050            # height of the ceramic body
WALL = 0.006                  # thick ceramic wall

MOUTH_OUTER_R = 0.034         # outer radius of the rim/neck
MOUTH_INNER_R = 0.028         # inner opening radius (6mm wall at mouth)
RIM_H = 0.012                 # rim height above the shoulder
RIM_TOP_Z = JAR_BODY_H + RIM_H

# Stopper dimensions (authored in stopper part frame, origin at rim top)
STOPPER_PLUG_R = MOUTH_INNER_R - 0.001   # plug fits inside the mouth
STOPPER_PLUG_H = 0.008                    # plug insertion depth
STOPPER_CAP_R = MOUTH_OUTER_R + 0.002    # cap overhangs the rim
STOPPER_CAP_H = 0.005                     # cap disc thickness
STOPPER_KNOB_R = 0.008                    # grip knob radius
STOPPER_KNOB_H = 0.012                    # grip knob height


def _jar_ceramic_solid() -> cq.Workplane:
    # Hollow thick-walled ceramic jar built as a revolve profile. The profile
    # traces the outer wall up, across a shoulder into the rim/neck, then back
    # down the inner wall to form a real open cavity with visible wall thickness
    # at the mouth.
    pts = [
        (0.0, 0.0),                             # center of base
        (JAR_OUTER_R, 0.0),                     # outer base edge
        (JAR_OUTER_R, JAR_BODY_H - 0.008),      # outer wall up
        (JAR_OUTER_R - 0.004, JAR_BODY_H),      # rounded shoulder
        (MOUTH_OUTER_R, JAR_BODY_H + 0.002),    # step in to the rim
        (MOUTH_OUTER_R, RIM_TOP_Z),             # rim outer up to top
        (MOUTH_INNER_R, RIM_TOP_Z),             # across the rim top (wall thickness visible)
        (MOUTH_INNER_R, JAR_BODY_H - 0.002),    # inner rim wall down
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.008),
        (JAR_OUTER_R - WALL, WALL),             # inner body wall down to thick base
        (0.0, WALL),                            # across inner base
        (0.0, 0.0),                             # close
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    jar = profile.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Spoon notch: a rectangular cutout on the +X side of the rim.
    # Cuts through the full rim height and into the rim wall.
    notch_width = 0.014
    notch_depth = MOUTH_OUTER_R - MOUTH_INNER_R + 0.003  # cuts through rim wall + a bit
    notch_cutter = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H - 0.001)
        .center(MOUTH_OUTER_R + 0.001, 0.0)
        .rect(notch_depth * 2, notch_width)
        .extrude(RIM_H + 0.004)
    )
    jar = jar.cut(notch_cutter)

    return jar


def _rim_threads() -> cq.Workplane:
    # Decorative thread ridges around the rim exterior: thin stacked rings.
    threads = None
    n_ridges = 4
    z_start = JAR_BODY_H + 0.003
    spacing = (RIM_H - 0.004) / max(n_ridges - 1, 1)
    for i in range(n_ridges):
        z = z_start + i * spacing
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(MOUTH_OUTER_R + 0.0008)
            .circle(MOUTH_OUTER_R - 0.0002)
            .extrude(0.0018)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _stopper_solid() -> cq.Workplane:
    # Stopper: a plug that inserts into the mouth, a cap disc that sits on the
    # rim top, and a grip knob on top for lifting. Authored in the stopper part
    # frame whose origin is at the rim top (world z = RIM_TOP_Z at rest).
    # Plug extends downward (negative local Z) into the mouth.
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-STOPPER_PLUG_H)
        .circle(STOPPER_PLUG_R)
        .extrude(STOPPER_PLUG_H)
    )
    # Taper the plug bottom for easier insertion
    plug_tip = (
        cq.Workplane("XY")
        .workplane(offset=-STOPPER_PLUG_H - 0.003)
        .circle(STOPPER_PLUG_R * 0.7)
        .workplane(offset=0.003)
        .circle(STOPPER_PLUG_R)
        .loft(ruled=True)
    )
    # Cap disc sits on the rim top
    cap = (
        cq.Workplane("XY")
        .circle(STOPPER_CAP_R)
        .extrude(STOPPER_CAP_H)
    )
    cap = cap.edges(">Z").fillet(0.002)
    # Grip knob on top
    knob = (
        cq.Workplane("XY")
        .workplane(offset=STOPPER_CAP_H)
        .circle(STOPPER_KNOB_R)
        .extrude(STOPPER_KNOB_H)
    )
    knob = knob.edges(">Z").fillet(STOPPER_KNOB_R * 0.6)
    return plug.union(plug_tip).union(cap).union(knob)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceramic_jar")

    ceramic_cream = model.material("ceramic_cream", rgba=(0.92, 0.88, 0.80, 1.0))
    ceramic_rim = model.material("ceramic_rim", rgba=(0.85, 0.80, 0.72, 1.0))
    stopper_cork = model.material("stopper_cork", rgba=(0.68, 0.52, 0.34, 1.0))
    stopper_knob = model.material("stopper_knob", rgba=(0.55, 0.40, 0.25, 1.0))

    # ---- jar body (root): ceramic shell + thread ridges ----
    body = model.part("body")

    jar_shell = _jar_ceramic_solid().union(_rim_threads())
    body.visual(
        mesh_from_cadquery(jar_shell, "jar_ceramic"),
        material=ceramic_cream,
        name="jar_ceramic",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- stopper: plugs into the mouth, lifts vertically ----
    stopper = model.part("stopper")

    stopper_solid = _stopper_solid()
    stopper.visual(
        mesh_from_cadquery(stopper_solid, "stopper_body"),
        material=stopper_cork,
        name="stopper_body",
    )

    stopper.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_CAP_R, STOPPER_CAP_H + STOPPER_KNOB_H + STOPPER_PLUG_H),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, (STOPPER_CAP_H + STOPPER_KNOB_H - STOPPER_PLUG_H) * 0.5)),
    )

    # Prismatic joint: stopper lifts vertically from the jar body.
    # Origin at the rim top; axis +Z so positive q lifts the stopper upward.
    # Limits: 0 = seated, upper = fully lifted clear of the mouth.
    lift_distance = STOPPER_PLUG_H + 0.010 + RIM_H  # plug clears rim top
    model.articulation(
        "stopper_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=lift_distance,
            effort=2.0,
            velocity=0.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    stopper = object_model.get_part("stopper")
    lift = object_model.get_articulation("stopper_lift")

    # The stopper plug is intentionally nested inside the jar mouth at rest.
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_body",
        elem_b="jar_ceramic",
        reason="The stopper plug is intentionally inserted into the jar mouth opening.",
    )

    # ---- jar is wider than tall (squat jar proportions) ----
    body_aabb = ctx.part_world_aabb(body)
    bx = body_aabb[1][0] - body_aabb[0][0]
    by = body_aabb[1][1] - body_aabb[0][1]
    bz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "jar is squat (wider than tall)",
        bx > bz + 0.005 and by > bz + 0.005,
        details=f"body extents=({bx:.4f}, {by:.4f}, {bz:.4f})",
    )

    # ---- stopper sits on top of the jar at rest ----
    body_pos = ctx.part_world_position(body)
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper is on top of the jar",
        stopper_pos is not None and body_pos is not None
        and stopper_pos[2] > JAR_BODY_H,
        details=f"stopper_z={stopper_pos}, jar_body_h={JAR_BODY_H}",
    )

    # ---- stopper overlaps the body in XY at rest (plug is in the mouth) ----
    ctx.expect_overlap(
        stopper, body, axes="xy", min_overlap=0.02,
        name="stopper plug is inside the mouth opening",
    )

    # ---- stopper_lift articulation is prismatic ----
    ctx.check(
        "stopper_lift is prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={lift.articulation_type}",
    )

    # ---- stopper_lift axis is vertical ----
    ctx.check(
        "stopper_lift axis is vertical (+Z)",
        abs(lift.axis[2] - 1.0) < 0.01 and abs(lift.axis[0]) < 0.01 and abs(lift.axis[1]) < 0.01,
        details=f"axis={lift.axis}",
    )

    # ---- stopper_lift has positive upper limit ----
    limits = lift.motion_limits
    ctx.check(
        "stopper_lift has bounded lift range",
        limits is not None and limits.lower is not None and limits.upper is not None
        and limits.upper > limits.lower + 0.005,
        details=f"limits=({limits.lower}, {limits.upper})" if limits else "no limits",
    )

    # ---- lifting the stopper raises it above the rim ----
    rest_z = ctx.part_world_position(stopper)[2]
    lift_upper = limits.upper if limits and limits.upper else 0.03
    with ctx.pose({lift: lift_upper}):
        lifted_z = ctx.part_world_position(stopper)[2]
        # When fully lifted, the stopper clears the rim top.
        ctx.expect_gap(
            stopper, body, axis="z",
            min_gap=0.0,
            positive_elem="stopper_body",
            negative_elem="jar_ceramic",
            name="lifted stopper clears the rim",
        )
    ctx.check(
        "stopper_lift raises the stopper upward",
        lifted_z > rest_z + 0.005,
        details=f"rest_z={rest_z:.4f}, lifted_z={lifted_z:.4f}",
    )

    # ---- thread ridges exist as distinct geometry on the body ----
    # The jar ceramic visual should extend above the body height (rim + threads).
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "rim extends above body height (threads present)",
        body_aabb[1][2] > JAR_BODY_H + 0.005,
        details=f"body_max_z={body_aabb[1][2]:.4f}, expected > {JAR_BODY_H + 0.005}",
    )

    return ctx.report()


object_model = build_object_model()
