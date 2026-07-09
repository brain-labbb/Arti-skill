from __future__ import annotations

# Ceramic jar with wide mouth, spoon notch, thread ridges, and a lift-off stopper.
# Variant of the tall square glass bottle — now a round ceramic jar.
#
# Frame: vertical axis +Z, jar centered on world Z axis, base on z=0.
#   - body: a cylindrical ceramic jar with hollow interior, wide mouth,
#           thick wall at mouth, thread ridges around rim, spoon notch in rim.
#   - stopper: a ceramic stopper that sits in the mouth and lifts vertically.
#
# Articulation:
#   - body_to_stopper: PRISMATIC along +Z. At q=0 the stopper is seated in the
#     mouth; positive q lifts the stopper straight up (~0.06 m).

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

# ----- key dimensions (meters) -----
JAR_OD = 0.082        # outer diameter of jar body (slightly narrower than rim)
JAR_ID = 0.068        # inner diameter (cavity)
BODY_WALL = 0.007     # ceramic wall thickness
JAR_H = 0.090         # total jar body height
BASE_THICK = 0.008    # solid ceramic floor

# Mouth / rim section (wider than body — prominent rim)
MOUTH_OD = 0.094      # outer diameter of mouth rim
MOUTH_ID = 0.070      # inner diameter of mouth (wide opening)
MOUTH_WALL = 0.012    # thick wall at mouth (visible glass/ceramic thickness)
MOUTH_H = 0.016       # height of the rim/mouth section above body top
BODY_TOP_Z = JAR_H    # where the main body ends and mouth begins
MOUTH_TOP_Z = BODY_TOP_Z + MOUTH_H

# Thread ridges around the outside of the rim
NUM_THREADS = 20
THREAD_W = 0.003      # width of each ridge
THREAD_H = 0.010      # height of each ridge (covers most of mouth height)
THREAD_DEPTH = 0.002  # how far ridges protrude outward from rim

# Spoon notch — semicircular cutout in the rim on the +Y side
NOTCH_RADIUS = 0.008  # radius of the semicircular notch
NOTCH_ANGLE = 0.0     # angle where notch is located (0 = +X, 90 = +Y)

# Stopper dimensions (in stopper-local frame)
STOPPER_OD = 0.068       # outer diameter, fits inside mouth (MOUTH_ID=0.070)
STOPPER_PLUG_H = 0.012   # plug portion that inserts into the mouth
STOPPER_FLANGE_OD = 0.082  # wider flange that rests on rim top
STOPPER_FLANGE_H = 0.004   # thin flange
STOPPER_KNOB_R = 0.010     # grip knob radius on top
KNOB_Z = STOPPER_PLUG_H + STOPPER_FLANGE_H + STOPPER_KNOB_R

# At rest (q=0), stopper plug is inserted into the mouth.
# The stopper part frame sits at the articulation origin.
# Plug bottom at local z=0, plug extends to z=STOPPER_PLUG_H,
# flange at z=STOPPER_PLUG_H to z=STOPPER_PLUG_H+STOPPER_FLANGE_H,
# knob on top.
STOPPER_SEAT_Z = MOUTH_TOP_Z - STOPPER_PLUG_H - STOPPER_FLANGE_H


def _jar_body() -> cq.Workplane:
    """Build the ceramic jar body: hollow cylinder with wide mouth, thick walls,
    thread ridges, and spoon notch."""
    # Main body: outer cylinder minus inner cavity
    outer = (
        cq.Workplane("XY")
        .circle(JAR_OD / 2.0)
        .extrude(JAR_H)
    )
    # Inner cavity: open at top, solid floor at bottom
    inner = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICK)
        .circle(JAR_ID / 2.0)
        .extrude(JAR_H + MOUTH_H + 0.01)  # over-extrude to open through top
    )
    body = outer.cut(inner)

    # Mouth / rim section on top of the body
    mouth_outer = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .circle(MOUTH_OD / 2.0)
        .extrude(MOUTH_H)
    )
    mouth_inner = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z - 0.001)
        .circle(MOUTH_ID / 2.0)
        .extrude(MOUTH_H + 0.002)  # cut through completely
    )
    mouth = mouth_outer.cut(mouth_inner)
    body = body.union(mouth)

    # Thread ridges: small protruding ribs around outside of mouth rim.
    # Each ridge overlaps slightly into the rim solid to ensure mesh connectivity.
    ridge_overlap = 0.001  # how far the ridge inner face penetrates into the rim
    ridge_r = MOUTH_OD / 2.0 + THREAD_DEPTH / 2.0 - ridge_overlap
    ridge_z_bottom = BODY_TOP_Z + (MOUTH_H - THREAD_H) / 2.0
    for i in range(NUM_THREADS):
        angle_deg = i * (360.0 / NUM_THREADS)
        # Rotate workplane so local X points radially outward at this angle,
        # then center on the ridge radius and draw the box.
        ridge = (
            cq.Workplane("XY")
            .workplane(offset=ridge_z_bottom)
            .transformed(rotate=(0, 0, angle_deg))
            .center(ridge_r, 0)
            .rect(THREAD_DEPTH + ridge_overlap, THREAD_W)
            .extrude(THREAD_H)
        )
        body = body.union(ridge)

    # Spoon notch: semicircular cutout in the rim on the +Y side
    # Cut a cylinder from the rim wall at the notch location
    notch_cx = 0.0
    notch_cy = (MOUTH_OD / 2.0 + MOUTH_ID / 2.0) / 2.0  # center of rim wall
    notch_cutter = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z - 0.001)
        .center(notch_cx, notch_cy)
        .circle(NOTCH_RADIUS)
        .extrude(MOUTH_H + 0.002)
    )
    body = body.cut(notch_cutter)

    return body


def _stopper_solid() -> cq.Workplane:
    """Build the stopper in its local frame.
    Local z=0 is the bottom of the plug.
    Plug -> flange -> knob stack upward."""
    # Plug: cylinder that inserts into the mouth
    plug = (
        cq.Workplane("XY")
        .circle(STOPPER_OD / 2.0)
        .extrude(STOPPER_PLUG_H)
    )
    # Flange: wider disk that rests on top of the rim
    flange = (
        cq.Workplane("XY")
        .workplane(offset=STOPPER_PLUG_H)
        .circle(STOPPER_FLANGE_OD / 2.0)
        .extrude(STOPPER_FLANGE_H)
    )
    stopper = plug.union(flange)

    # Grip knob on top: small sphere
    knob = (
        cq.Workplane("XY")
        .workplane(offset=STOPPER_PLUG_H + STOPPER_FLANGE_H)
        .sphere(STOPPER_KNOB_R)
    )
    stopper = stopper.union(knob)

    return stopper


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceramic_jar")

    ceramic = model.material("ceramic_body", rgba=(0.85, 0.80, 0.72, 1.0))
    ceramic_dark = model.material("ceramic_stopper", rgba=(0.78, 0.73, 0.65, 1.0))

    # ---- body (root): ceramic jar with wide mouth ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_jar_body(), "jar_body"),
        material=ceramic,
        name="jar_body",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=MOUTH_OD / 2.0, length=JAR_H + MOUTH_H),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, (JAR_H + MOUTH_H) / 2.0)),
    )

    # ---- stopper: ceramic stopper seated in the mouth ----
    stopper = model.part("stopper")
    stopper.visual(
        mesh_from_cadquery(_stopper_solid(), "stopper_mesh"),
        material=ceramic_dark,
        name="stopper_mesh",
        origin=Origin(),
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(radius=STOPPER_FLANGE_OD / 2.0, length=KNOB_Z + STOPPER_KNOB_R),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, KNOB_Z / 2.0)),
    )

    # Stopper lifts straight up (PRISMATIC +Z).
    # At q=0 the stopper is seated; positive q lifts it out of the mouth.
    model.articulation(
        "body_to_stopper",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.3, lower=0.0, upper=0.06),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    stopper = object_model.get_part("stopper")
    lift = object_model.get_articulation("body_to_stopper")

    # The stopper plug is intentionally inserted into the jar mouth at rest,
    # which means the stopper and body overlap in Z where the plug sits.
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_mesh",
        elem_b="jar_body",
        reason="Stopper plug is intentionally seated inside the jar mouth at rest (friction-fit stopper).",
    )

    # ---- Jar proportions: wider than or comparable to tall (jar, not bottle) ----
    body_aabb = ctx.part_world_aabb(body)
    body_ext = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "jar is round in section (X and Y extents similar)",
        abs(body_ext[0] - body_ext[1]) < 0.006,
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "jar proportions: diameter is at least 60% of height (jar, not bottle)",
        max(body_ext[0], body_ext[1]) > 0.60 * body_ext[2],
        details=f"body extents={body_ext}",
    )

    # ---- Prismatic joint exists and lifts stopper vertically ----
    ctx.check(
        "body_to_stopper is prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={lift.articulation_type}",
    )
    limits = lift.motion_limits
    ctx.check(
        "prismatic joint has bounded limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )

    # ---- Stopper at rest: seated in the mouth ----
    rest_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper sits near the mouth top at rest",
        rest_pos is not None and rest_pos[2] > BODY_TOP_Z - 0.02,
        details=f"stopper_z={rest_pos[2] if rest_pos else None}, body_top={BODY_TOP_Z}",
    )

    # Stopper plug overlaps the mouth in Z at rest (inserted)
    ctx.expect_overlap(
        stopper,
        body,
        axes="z",
        min_overlap=0.004,
        name="stopper plug is inserted into the mouth at rest",
    )

    # ---- Stopper lifts straight up when q > 0 ----
    with ctx.pose({lift: 0.06}):
        lifted_pos = ctx.part_world_position(stopper)
        # Fully lifted: stopper clears the mouth entirely
        ctx.expect_gap(
            stopper,
            body,
            axis="z",
            min_gap=0.0,
            name="lifted stopper clears the jar mouth",
        )

    ctx.check(
        "stopper lifts upward (Z increases with positive q)",
        lifted_pos is not None and lifted_pos[2] > rest_pos[2] + 0.05,
        details=f"rest_z={rest_pos[2]}, lifted_z={lifted_pos[2] if lifted_pos else None}",
    )
    ctx.check(
        "stopper does not translate sideways while lifting",
        lifted_pos is not None
        and abs(lifted_pos[0] - rest_pos[0]) < 1e-5
        and abs(lifted_pos[1] - rest_pos[1]) < 1e-5,
        details=f"rest_xy={rest_pos[:2]}, lifted_xy={lifted_pos[:2] if lifted_pos else None}",
    )

    # ---- Materials: ceramic body and stopper are distinct ----
    body_mat = body.get_visual("jar_body").material
    stopper_mat = stopper.get_visual("stopper_mesh").material
    ctx.check(
        "body and stopper have distinct ceramic materials",
        body_mat is not None
        and stopper_mat is not None
        and getattr(body_mat, "name", None) != getattr(stopper_mat, "name", None),
        details=f"body_mat={getattr(body_mat, 'name', None)}, stopper_mat={getattr(stopper_mat, 'name', None)}",
    )

    # ---- Mouth is wide (opening is at least 60% of body outer diameter) ----
    ctx.check(
        "jar has a wide mouth opening",
        MOUTH_ID > 0.60 * JAR_OD,
        details=f"mouth_id={MOUTH_ID}, jar_od={JAR_OD}",
    )

    # ---- Thread ridges extend the body beyond plain mouth OD ----
    ctx.check(
        "thread ridges protrude beyond the mouth outer diameter",
        MOUTH_OD / 2.0 + THREAD_DEPTH > MOUTH_OD / 2.0,
        details=f"mouth_r={MOUTH_OD/2.0}, thread_tip_r={MOUTH_OD/2.0 + THREAD_DEPTH}",
    )

    return ctx.report()


object_model = build_object_model()
