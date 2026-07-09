from __future__ import annotations

# Square glass storage jar with a brass screw lid.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar body: square-section clear glass shell with rounded vertical edges,
#     hollow inside, topped by a short round threaded neck. (root)
#   - brass screw lid: round knurled/scalloped cap that screws onto the neck.
#     Modeled as a SCREW CAP via two INDEPENDENT decoupled joints sharing +Z,
#     routed through a massless carrier link:
#       lid_rotate (CONTINUOUS, body->carrier): lid spins about +Z
#       lid_slide  (PRISMATIC, carrier->lid):   lid lifts up off the neck
# Square cross-section is taller than wide; lid sits on the round neck.

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
BODY_HALF = 0.040          # half-width of the square section (0.080 square)
BODY_FILLET = 0.012        # rounded vertical-edge radius
WALL = 0.0035              # glass wall thickness
BODY_Z0 = 0.0              # jar base sits on the ground
BODY_TOP = 0.118           # top of the square body section
SHOULDER_TOP = 0.130       # top of the tapered shoulder where the neck begins
NECK_R = 0.0265            # outer radius of the round threaded neck
NECK_TOP = 0.150           # top of the neck (z)
NECK_BOTTOM = SHOULDER_TOP

LID_R = 0.0300             # brass lid skirt outer radius
LID_HEIGHT = 0.024         # full height of the lid skirt + top
SCALLOP_N = 22             # number of scallops on the knurled skirt
# Lid mount height: the skirt drops over the neck so its bore wraps the neck
# top. The lid local origin is at the skirt bottom (z=0).
LID_MOUNT_Z = NECK_TOP - 0.016


def _body_solid() -> cq.Workplane:
    # Hollow square glass jar with rounded vertical edges and a round neck.
    # Outer square prism, vertical edges filleted, then a tapered shoulder
    # transitions to a round neck. The whole thing is shelled hollow with the
    # top of the neck left open (a real jar mouth).
    outer = (
        cq.Workplane("XY")
        .box(2 * BODY_HALF, 2 * BODY_HALF, BODY_TOP, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_FILLET)
    )

    # Tapered shoulder: square body top -> round neck base.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .rect(2 * (BODY_HALF - 0.004), 2 * (BODY_HALF - 0.004))
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R)
        .loft(ruled=False)
    )

    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R)
        .extrude(NECK_TOP - NECK_BOTTOM)
    )

    solid = outer.union(shoulder).union(neck)

    # Hollow it out: cut an inner cavity that opens at the neck top.
    inner = (
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
        .circle(NECK_R - WALL)
        .loft(ruled=False)
    )
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R - WALL)
        .extrude((NECK_TOP - NECK_BOTTOM) + 0.001)
    )
    cavity = inner.union(inner_shoulder).union(inner_neck)
    return solid.cut(cavity)


def _thread_ridges() -> cq.Workplane:
    # A couple of helical-ish thread ridges on the neck (represented as thin
    # rings) so the neck reads as a threaded screw neck.
    rings = None
    for i, zc in enumerate((NECK_BOTTOM + 0.006, NECK_BOTTOM + 0.013)):
        ring = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(NECK_R + 0.0008)
            .circle(NECK_R - 0.0004)
            .extrude(0.0022)
        )
        rings = ring if rings is None else rings.union(ring)
    return rings


def _body_mesh():
    solid = _body_solid().union(_thread_ridges())
    return mesh_from_cadquery(solid, "jar_glass")


def _lid_solid() -> cq.Workplane:
    # Round brass lid: a flat top disc + a knurled/scalloped cylindrical skirt
    # that wraps over the neck. The skirt is hollow (open at the bottom) so it
    # caps over the neck, and its outer edge is scalloped by subtracting a ring
    # of small vertical flutes.
    skirt = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_HEIGHT)
    )
    # Hollow underside so it caps over the neck (leave a top plate).
    bore = (
        cq.Workplane("XY")
        .circle(NECK_R - 0.0006)
        .extrude(LID_HEIGHT - 0.004)
    )
    lid = skirt.cut(bore)

    # Scallops / knurling: subtract small vertical flutes around the rim.
    for k in range(SCALLOP_N):
        ang = 2.0 * math.pi * k / SCALLOP_N
        fx = LID_R * math.cos(ang)
        fy = LID_R * math.sin(ang)
        flute = (
            cq.Workplane("XY")
            .center(fx, fy)
            .circle(0.0026)
            .extrude(LID_HEIGHT)
        )
        lid = lid.cut(flute)

    # Slight chamfer on the top outer edge.
    try:
        lid = lid.faces(">Z").edges().chamfer(0.0015)
    except Exception:
        pass
    return lid


def _lid_mesh():
    return mesh_from_cadquery(_lid_solid(), "lid_brass")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_glass_storage_jar")

    glass = model.material("clear_glass", rgba=(0.82, 0.86, 0.88, 0.25))
    brass = model.material("brass", rgba=(0.72, 0.55, 0.20, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.52, 0.38, 0.12, 1.0))

    # ---- jar body (root): square hollow glass shell + round threaded neck ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_HALF, 2 * BODY_HALF, NECK_TOP)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP / 2.0)),
    )

    # ---- massless carrier (NO visuals): routes the spin joint ----
    carrier = model.part("lid_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- brass screw lid: knurled skirt + off-axis marker ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=brass, name="lid_brass")
    # Off-axis marker (a small brass nub) so rotation is observable in tests.
    marker = CylinderGeometry(0.0022, 0.004).translate(LID_R - 0.004, 0.0, LID_HEIGHT)
    lid.visual(mesh_from_geometry(marker, "lid_marker"), material=brass_dark, name="lid_marker")
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_HEIGHT),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, LID_HEIGHT / 2.0)),
    )

    # ---- two INDEPENDENT decoupled joints sharing +Z, through the carrier ----
    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, LID_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_HEIGHT, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    lid = object_model.get_part("lid")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")

    # The lid skirt is intentionally seated over the round neck (capture fit).
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_brass",
        elem_b="jar_glass",
        reason="The brass lid skirt is intentionally screwed down over the round neck.",
    )

    # --- jar body is a square section, taller than wide ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is square in cross-section",
        abs(bext[0] - bext[1]) < 0.006,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is taller than wide",
        bext[2] > bext[0] + 0.04 and bext[2] > bext[1] + 0.04,
        details=f"extents={bext}",
    )

    # --- brass lid is round and seated on top of the jar (on the neck) ---
    lext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid is round (square footprint bounding a disc)",
        abs(lext[0] - lext[1]) < 0.003 and lext[0] < bext[0],
        details=f"lid x={lext[0]:.4f}, y={lext[1]:.4f}, body x={bext[0]:.4f}",
    )
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at the top of the jar on the neck",
        lid_pos is not None and lid_pos[2] > NECK_BOTTOM,
        details=f"lid origin z={lid_pos[2] if lid_pos else None}, neck_bottom={NECK_BOTTOM}",
    )
    # Lid skirt overlaps the neck region in the XY footprint (seated on neck).
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02, name="lid seated over neck footprint"
    )

    # --- lid_rotate spins the lid: off-axis marker moves around +Z ---
    m0 = ctx.part_element_world_aabb(lid, elem="lid_marker")
    m0c = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(lid, elem="lid_marker")
        m1c = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    marker_shift = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "rotating lid_rotate spins the lid (marker moves)",
        marker_shift > 0.01,
        details=f"marker moved {marker_shift:.4f} m on a quarter turn",
    )

    # --- lid_slide lifts the lid up off the neck ---
    z_rest = ctx.part_world_position(lid)[2]
    with ctx.pose({slide: LID_HEIGHT}):
        z_lift = ctx.part_world_position(lid)[2]
    ctx.check(
        "lid_slide lifts the lid up off the neck",
        z_lift > z_rest + 0.015,
        details=f"rest z={z_rest:.4f}, lifted z={z_lift:.4f}",
    )

    # --- the two joints are decoupled: sliding does not rotate, etc. ---
    ctx.check(
        "lid_rotate is continuous about +Z",
        rotate.axis == (0.0, 0.0, 1.0),
        details=f"axis={rotate.axis}, type={rotate.articulation_type}",
    )
    ctx.check(
        "lid_slide is prismatic about +Z",
        slide.axis == (0.0, 0.0, 1.0),
        details=f"axis={slide.axis}, type={slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
