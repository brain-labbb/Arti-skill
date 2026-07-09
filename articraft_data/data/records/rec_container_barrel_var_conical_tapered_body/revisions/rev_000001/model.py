from __future__ import annotations

# Blue plastic open-top storage drum (~30 L), conical tapered variant.
# The barrel is clearly wider at the base (0.29 m dia) and narrows toward a
# smaller top rim (0.21 m dia), lathed as a real tapered shell with rib bands
# following the cone. Lift-off lid and front swinging buckle clasp are retained.
# Frame: vertical drum axis along +Z, base of the barrel at z=0, centerline at
#   x=y=0. "Front" is the +Y face (where the buckle clasp lives).
# Articulations:
#   - clasp_base: FIXED buckle body (lever plate + cam block) bolted to the front
#       rim. It does NOT move; it is the static mount the ring swings on.
#   - ring_swing: REVOLUTE U-shaped catch wire (the swinging bail/ring) that hangs
#       off the cam block and swings up about its own pivot (~0-120 deg). Only this
#       wire ring moves; the buckle body stays put.
#   - lid: PRISMATIC lift-off disc that rises straight up off the rim (~0.05 m)

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tessellate_cadquery,
)

# ---- key dimensions (meters) --------------------------------------------------
BARREL_H = 0.35  # overall barrel height
BASE_R = 0.145  # outer radius at the base (wider, ~0.29 m diameter)
TOP_R = 0.105  # radius at the top rim (narrower, ~0.21 m diameter)
BARREL_R = BASE_R  # reference radius for inertial (use base as max)
WALL_T = 0.010  # nominal wall thickness
RIM_Z = BARREL_H  # top rim plane
LID_Z = RIM_Z - 0.002  # lid disc underside seats onto the rolled rim lip
CLASP_PIVOT_Z = RIM_Z - 0.045  # buckle pivot sits below the rim on the front face


def _taper_r(z: float) -> float:
    """Linear taper from BASE_R at z=0 to TOP_R at z=BARREL_H."""
    return BASE_R + (TOP_R - BASE_R) * z / BARREL_H


def _cq_geom(solid: cq.Workplane) -> MeshGeometry:
    # Tessellate a cadquery solid into a MeshGeometry so it can be merged with
    # primitive geometries before becoming a single visual.
    verts, faces = tessellate_cadquery(solid)
    geom = MeshGeometry()
    for vx, vy, vz in verts:
        geom.add_vertex(vx, vy, vz)
    for a, b, c in faces:
        geom.add_face(a, b, c)
    return geom


def _profile_loft(sections) -> cq.Workplane:
    # sections: list of (z, r) circular cross-sections stacked along +Z.
    wp = cq.Workplane("XY")
    prev = 0.0
    for i, (z, r) in enumerate(sections):
        off = z if i == 0 else z - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(r)
        prev = z
    return wp.loft(ruled=False)


def _barrel_solid() -> cq.Workplane:
    # Open-top conical tapered barrel: wider at the base, narrowing toward the
    # top rim. Loft with shell to create the open-top container.
    outer = _profile_loft(
        [
            (0.000, BASE_R),  # base
            (0.012, BASE_R + 0.003),  # base rib flare
            (0.175, _taper_r(0.175)),  # mid body
            (RIM_Z, TOP_R),  # top rim
        ]
    )
    # Hollow it from the top into a single connected open-top shell.
    shell = outer.faces(">Z").shell(-0.012)

    # Reinforcing ribs around the body following the taper (typical molded drum
    # stiffening rings). Each rib protrudes slightly beyond the shell surface
    # and the inner radius is kept well inside the wall so every rib fuses.
    for z in (0.090, 0.150, 0.210, 0.270):
        r_at_z = _taper_r(z)
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(r_at_z + 0.003)  # protrude slightly past shell
            .circle(r_at_z - 0.020)  # inner well inside wall
            .extrude(0.006)
        )
        shell = shell.union(ring)

    # Rolled top rim lip the lid seats onto and the clasp hooks under. Unioned
    # into the body so the barrel reads as one welded blue shell.
    lip = (
        cq.Workplane("XY")
        .workplane(offset=RIM_Z - 0.014)
        .circle(TOP_R + 0.005)  # slightly proud of top rim
        .circle(TOP_R - 0.010)
        .extrude(0.016)
    )
    shell = shell.union(lip)
    return shell


def _lid_mesh():
    # Black lift-off lid: a slightly domed disc whose bottom sits flat on top of
    # the rim, with a short depending skirt that caps the rim from OUTSIDE the
    # barrel wall. Sized to match the tapered top rim (TOP_R).
    # Local z=0 is the underside of the disc (which rests on the rim).
    lid_outer = TOP_R + 0.005  # match the rolled rim lip outer
    disc = (
        cq.Workplane("XY")
        .circle(lid_outer)
        .extrude(0.012)
        .faces(">Z")
        .workplane()
        .circle(lid_outer)
        .workplane(offset=0.006)
        .circle(TOP_R * 0.75)  # dome apex narrower
        .loft(ruled=False)
    )
    # Depending outer skirt that wraps down around the outside of the rim.
    # Overlaps up into the disc edge so the union welds into one solid.
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=-0.022)
        .circle(lid_outer + 0.005)
        .circle(lid_outer - 0.001)
        .extrude(0.030)
    )
    lid = disc.union(skirt)
    return mesh_from_cadquery(lid, "lid_shell")


def _clasp_base_mesh():
    # Static buckle body: the lever back plate (flush against the barrel wall) and
    # the cam/handle block. This is the FIXED mount; it never moves. Built in the
    # clasp-local frame whose origin is the fixed-joint mount point on the rim.
    plate = (
        cq.Workplane("XZ")
        .rect(0.060, 0.050, centered=True)
        .extrude(0.012)
        .translate((0.0, 0.012, -0.020))
    )
    # Cam/handle block; its inner face overlaps the plate so the union welds solid.
    block = (
        cq.Workplane("XY")
        .box(0.052, 0.030, 0.034, centered=True)
        .translate((0.0, 0.018, -0.022))
    )
    base = plate.union(block)
    return mesh_from_cadquery(base, "clasp_base_shell")


def _clasp_ring_mesh():
    # The swinging U-shaped catch wire (the bail/ring) -- the ONLY moving part of
    # the buckle. Built in a ring-local frame whose origin is the ring's own pivot
    # (the line where the two legs enter the cam block). The two legs hang straight
    # down from the pivot and the cross-bar joins them at the bottom, so the ring
    # hangs down at rest and swings up about the local +X axis.
    ring = None
    for sx in (-0.024, 0.024):
        leg = (
            cq.Workplane("XY")
            .circle(0.005)
            .extrude(0.050)
            .translate((sx, 0.0, -0.050))
        )
        ring = leg if ring is None else ring.union(leg)
    cross = (
        cq.Workplane("YZ")
        .circle(0.005)
        .extrude(0.052)
        .translate((-0.026, 0.0, -0.048))
    )
    ring = ring.union(cross)
    return mesh_from_cadquery(ring, "clasp_ring_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tapered_storage_drum")

    blue = model.material("drum_blue", rgba=(0.16, 0.36, 0.72, 1.0))
    black = model.material("lid_black", rgba=(0.09, 0.09, 0.10, 1.0))
    clasp_black = model.material("clasp_black", rgba=(0.12, 0.12, 0.13, 1.0))

    # ---- barrel body (root) ----
    body = model.part("barrel_body")
    body.visual(
        mesh_from_cadquery(_barrel_solid(), "barrel_shell"),
        material=blue,
        name="barrel_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BARREL_R, length=BARREL_H),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, BARREL_H / 2.0)),
    )

    # ---- lid: lifts straight up off the rim (prismatic, no thread) ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=black, name="lid_shell")
    # Central grab boss on top of the lid.
    boss = CylinderGeometry(0.024, 0.012).translate(0.0, 0.0, 0.024)
    lid.visual(mesh_from_geometry(boss, "lid_boss"), material=black, name="lid_boss")
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=TOP_R + 0.005, length=0.04), mass=0.30, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )
    model.articulation(
        "lid_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.2, lower=0.0, upper=0.05),
    )

    # ---- front buckle body: FIXED mount on the front rim (does not move) ----
    clasp_base = model.part("clasp_base")
    clasp_base.visual(_clasp_base_mesh(), material=clasp_black, name="clasp_base_shell")
    clasp_base.inertial = Inertial.from_geometry(
        Box((0.06, 0.04, 0.05)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.015, -0.020)),
    )
    # Bolted to the front face (+Y) just below the rim; the mount frame origin is
    # the buckle-local origin used by both meshes. Y is set to the barrel surface
    # at the clasp height (tapered radius at CLASP_PIVOT_Z).
    clasp_y = _taper_r(CLASP_PIVOT_Z) + 0.001  # on the outer surface
    model.articulation(
        "clasp_mount",
        ArticulationType.FIXED,
        parent=body,
        child=clasp_base,
        origin=Origin(xyz=(0.0, clasp_y, CLASP_PIVOT_Z)),
    )

    # ---- swinging catch wire: ONLY the ring moves, hinged on the cam block ----
    clasp_ring = model.part("clasp_ring")
    clasp_ring.visual(_clasp_ring_mesh(), material=clasp_black, name="clasp_ring_shell")
    clasp_ring.inertial = Inertial.from_geometry(
        Box((0.06, 0.012, 0.05)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
    )
    # Pivot is where the legs enter the cam block (expressed in the fixed base's
    # frame); axis horizontal along X so the wire ring swings up in the YZ plane.
    model.articulation(
        "ring_swing",
        ArticulationType.REVOLUTE,
        parent=clasp_base,
        child=clasp_ring,
        origin=Origin(xyz=(0.0, 0.020, -0.016)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=math.radians(120.0)
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("barrel_body")
    lid = object_model.get_part("lid")
    clasp_base = object_model.get_part("clasp_base")
    clasp_ring = object_model.get_part("clasp_ring")
    lid_lift = object_model.get_articulation("lid_lift")
    ring_swing = object_model.get_articulation("ring_swing")

    # --- barrel is taller than it is wide ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "barrel taller than wide",
        bext[2] > bext[0] + 0.05 and bext[2] > bext[1] + 0.05,
        details=f"barrel extents={bext}",
    )

    # --- barrel is clearly tapered: wider at the base than at the top ---
    # Direct check: the base radius should be clearly larger than top radius
    ctx.check(
        "barrel is conical tapered (wider base, narrower top)",
        BASE_R > TOP_R + 0.030,
        details=f"BASE_R={BASE_R}, TOP_R={TOP_R}, taper_delta={BASE_R - TOP_R:.3f}",
    )

    # --- lid sits on top of the barrel rim ---
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits on top of the barrel",
        lid_pos is not None and lid_pos[2] > 0.30,
        details=f"lid origin={lid_pos}",
    )
    # The lid skirt caps over the rolled rim lip; that seated overlap is intentional.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="barrel_shell",
        reason="Lift-off lid skirt is intentionally seated down over the rolled rim lip.",
    )

    # --- lid lifts straight up off the rim (prismatic, +Z only) ---
    lz0 = ctx.part_world_position(lid)[2]
    with ctx.pose({lid_lift: 0.05}):
        lifted = ctx.part_world_position(lid)
    lz1 = lifted[2]
    ctx.check(
        "lid lifts straight up off the rim",
        lz1 > lz0 + 0.045
        and abs(lifted[0] - lid_pos[0]) < 1e-4
        and abs(lifted[1] - lid_pos[1]) < 1e-4,
        details=f"rest_z={lz0}, lifted={lifted}",
    )

    # --- buckle body is the fixed mount on the front (+Y) rim ---
    base_pos = ctx.part_world_position(clasp_base)
    ctx.check(
        "buckle body mounted on the front rim",
        base_pos is not None and base_pos[1] > 0.10 and base_pos[2] > 0.28,
        details=f"clasp_base origin={base_pos}",
    )
    # The buckle plate is mounted flush against the front barrel wall just below
    # the rim; that small seated overlap is the real mounting, not a clash.
    ctx.allow_overlap(
        clasp_base,
        body,
        elem_a="clasp_base_shell",
        elem_b="barrel_shell",
        reason="Buckle plate is intentionally riveted flush against the front barrel wall.",
    )
    ctx.expect_contact(clasp_base, body, name="buckle body mounted on barrel wall")
    # The ring legs are seated up into the cam block at the hinge; intentional.
    ctx.allow_overlap(
        clasp_ring,
        clasp_base,
        elem_a="clasp_ring_shell",
        elem_b="clasp_base_shell",
        reason="Wire ring legs are intentionally seated into the cam block at the hinge.",
    )

    # --- ONLY the ring swings: it rotates up about its pivot while the buckle
    #     body stays put (this is the corrected articulation) ---
    base_before = ctx.part_world_position(clasp_base)
    z_rest = ctx.part_world_aabb(clasp_ring)[1][2]  # top of ring at rest
    r_rest = _ext(ctx.part_world_aabb(clasp_ring))
    with ctx.pose({ring_swing: math.radians(120.0)}):
        z_swung = ctx.part_world_aabb(clasp_ring)[1][2]  # top after swinging up
        r_swung = _ext(ctx.part_world_aabb(clasp_ring))
        base_during = ctx.part_world_position(clasp_base)
    ctx.check(
        "ring swings up about its own pivot",
        z_swung > z_rest + 0.02,
        details=f"ring top_z rest={z_rest}, swung={z_swung}, ext rest={r_rest}, swung={r_swung}",
    )
    ctx.check(
        "buckle body does not move when the ring swings",
        base_before is not None
        and base_during is not None
        and max(abs(a - b) for a, b in zip(base_before, base_during)) < 1e-6,
        details=f"base before={base_before}, during ring swing={base_during}",
    )

    return ctx.report()


object_model = build_object_model()
